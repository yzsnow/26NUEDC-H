"""Inertial telemetry parser and bounded feed-forward for HOLD mode.

The bottom controller sends one 32-byte little-endian frame at 50 Hz.  This
module deliberately contains no Maix imports so the parser and control law can
be tested on a PC.  Hardware UART setup stays in main.py.

Feed-forward is represented as a finite motor-angle offset.  The offset tracks
the acceleration-derived target, and its derivative is added to the existing
visual cascade's motor angular velocity.  This avoids integrating a constant
acceleration into an ever-growing angle and gives timeout/status changes a
well-defined, smooth release path.
"""

import struct


FRAME_HEADER = b"\xA5\x5A"
FRAME_VERSION = 0x01
PAYLOAD_LENGTH = 26
FRAME_LENGTH = 32

FLAG_IMU_VALID = 1 << 0
FLAG_ENCODER_VALID = 1 << 1
FLAG_TASK_456 = 1 << 2
FLAG_MOTOR_OUTPUT = 1 << 3

STATE_IDLE = 0
STATE_RUNNING = 1
STATE_SOFT_STOP = 2
STATE_STABLE = 3


def _clamp(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def crc16_modbus(data):
    """CRC16-Modbus, polynomial 0xA001, initial value 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


class InertiaTelemetryFrame:
    __slots__ = (
        "sequence", "task", "state", "flags",
        "ax_mg", "ay_mg", "az_mg", "roll_cdeg", "pitch_cdeg",
        "yaw_rate_cdps", "encoder_speed_mms", "encoder_accel_mms2",
        "forward_pwm_tenths", "steer_pwm_tenths", "forward_pwm_rate",
        "sequence_advanced",
    )

    def __init__(self, sequence, task, state, flags, values):
        self.sequence = sequence
        self.task = task
        self.state = state
        self.flags = flags
        (self.ax_mg, self.ay_mg, self.az_mg,
         self.roll_cdeg, self.pitch_cdeg, self.yaw_rate_cdps,
         self.encoder_speed_mms, self.encoder_accel_mms2,
         self.forward_pwm_tenths, self.steer_pwm_tenths,
         self.forward_pwm_rate) = values
        self.sequence_advanced = True

    @property
    def imu_valid(self):
        return bool(self.flags & FLAG_IMU_VALID)

    @property
    def encoder_valid(self):
        return bool(self.flags & FLAG_ENCODER_VALID)

    @property
    def task_456(self):
        return bool(self.flags & FLAG_TASK_456)

    @property
    def motor_output(self):
        return bool(self.flags & FLAG_MOTOR_OUTPUT)

    def imu_axis_mg(self, axis):
        axis = int(axis)
        if axis == 1:
            return self.ay_mg
        if axis == 2:
            return self.az_mg
        return self.ax_mg


class InertiaTelemetryParser:
    """Incremental parser with header resync, CRC checking and frame stats."""

    def __init__(self, max_buffer=512):
        self.buffer = bytearray()
        self.max_buffer = max(FRAME_LENGTH * 2, int(max_buffer))
        self.frames_ok = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.noise_bytes = 0
        self.dropped_frames = 0
        self.duplicate_frames = 0
        self.last_sequence = None

    def reset(self):
        self.buffer = bytearray()
        self.last_sequence = None

    def feed(self, data):
        if data:
            self.buffer.extend(bytes(data))
        if len(self.buffer) > self.max_buffer:
            excess = len(self.buffer) - self.max_buffer
            del self.buffer[:excess]
            self.noise_bytes += excess

        frames = []
        while True:
            if len(self.buffer) < 2:
                break

            header_at = self.buffer.find(FRAME_HEADER)
            if header_at < 0:
                # Preserve a trailing 0xA5 because it may be the first byte of
                # a header split across two UART reads.
                keep = 1 if self.buffer[-1] == FRAME_HEADER[0] else 0
                discard = len(self.buffer) - keep
                if discard:
                    del self.buffer[:discard]
                    self.noise_bytes += discard
                break
            if header_at:
                del self.buffer[:header_at]
                self.noise_bytes += header_at

            if len(self.buffer) < FRAME_LENGTH:
                break
            if (self.buffer[2] != FRAME_VERSION
                    or self.buffer[3] != PAYLOAD_LENGTH):
                del self.buffer[0]
                self.format_errors += 1
                continue

            raw = bytes(self.buffer[:FRAME_LENGTH])
            expected_crc = raw[30] | (raw[31] << 8)
            if crc16_modbus(raw[:30]) != expected_crc:
                # Drop one byte, not the whole candidate: a genuine header can
                # start inside noisy data that happened to begin with A5 5A.
                del self.buffer[0]
                self.crc_errors += 1
                continue

            del self.buffer[:FRAME_LENGTH]
            values = struct.unpack_from("<11h", raw, 8)
            frame = InertiaTelemetryFrame(
                raw[4], raw[5], raw[6], raw[7], values)
            self._track_sequence(frame)
            self.frames_ok += 1
            frames.append(frame)
        return frames

    def _track_sequence(self, frame):
        if self.last_sequence is None:
            self.last_sequence = frame.sequence
            return
        delta = (frame.sequence - self.last_sequence) & 0xFF
        if delta == 0:
            frame.sequence_advanced = False
            self.duplicate_frames += 1
            return
        if delta > 1:
            self.dropped_frames += delta - 1
        self.last_sequence = frame.sequence


class InertiaReceiver:
    """Non-blocking Maix UART adapter; accepts any UART-like test double."""

    def __init__(self, serial, parser=None, max_read=256):
        self.serial = serial
        self.parser = parser or InertiaTelemetryParser()
        self.max_read = max(FRAME_LENGTH, int(max_read))
        self.read_errors = 0
        self.last_error = ""

    def poll(self):
        frames = []
        try:
            # Bound each poll even if the sender outruns the vision loop.  At
            # 50 Hz this still drains many camera frames' worth in one call.
            for _ in range(4):
                available = self.serial.available(0)
                if available <= 0:
                    break
                data = self.serial.read(min(available, self.max_read), timeout=0)
                if not data:
                    break
                frames.extend(self.parser.feed(data))
        except Exception as exc:
            self.read_errors += 1
            self.last_error = str(exc)
        return frames


class InertiaFeedForward:
    """Turn valid chassis telemetry into a bounded motor angular velocity."""

    def __init__(self, cfg):
        self.latest = None
        self.last_new_ms = None
        self.imu_bias_mg = 0.0
        self.imu_bias_valid = False
        self.theta_target_deg = 0.0
        self.theta_deg = 0.0
        self.omega_dps = 0.0
        self.active = False
        self.reason = "no-data"
        self.sync(cfg)

    def sync(self, cfg):
        previous_axis = getattr(self, "imu_axis", None)
        self.enabled = bool(cfg["ff_enabled"])
        self.k_pwm = float(cfg["ff_k_pwm"])
        self.k_encoder = float(cfg["ff_k_encoder"])
        self.k_imu = float(cfg["ff_k_imu"])
        self.imu_axis = int(cfg["ff_imu_axis"])
        if previous_axis is not None and self.imu_axis != previous_axis:
            self.imu_bias_mg = 0.0
            self.imu_bias_valid = False
        self.imu_sign = -1.0 if cfg["ff_imu_invert"] else 1.0
        self.max_deg = abs(float(cfg["ff_max_deg"]))
        self.attack_ms = max(1.0, float(cfg["ff_attack_ms"]))
        self.release_ms = max(1.0, float(cfg["ff_release_ms"]))
        self.timeout_ms = max(1.0, float(cfg["ff_timeout_ms"]))
        self.omega_max_dps = abs(float(cfg["ff_omega_max_dps"]))
        self.bias_lpf = min(1.0, max(0.0, float(cfg["ff_bias_lpf"])))

    def note_frame(self, frame, now_ms):
        self.latest = frame
        if not frame.sequence_advanced:
            return
        self.last_new_ms = now_ms
        if frame.state == STATE_STABLE and frame.imu_valid:
            sample = float(frame.imu_axis_mg(self.imu_axis))
            if not self.imu_bias_valid:
                self.imu_bias_mg = sample
                self.imu_bias_valid = True
            else:
                self.imu_bias_mg += self.bias_lpf * (
                    sample - self.imu_bias_mg)

    def frame_age_ms(self, now_ms):
        if self.last_new_ms is None:
            return -1
        return max(0, now_ms - self.last_new_ms)

    def reset_output(self):
        """Forget only the applied offset; keep frame history and IMU bias."""
        self.theta_target_deg = 0.0
        self.theta_deg = 0.0
        self.omega_dps = 0.0
        self.active = False
        self.reason = "mode"

    def update(self, now_ms, dt, allow=True):
        """Advance one control sample and return feed-forward telemetry.

        ``allow`` is owned by main.py and is true only while HOLD has a valid
        visual ball position.  Protocol gates are checked independently here.
        """
        dt = max(0.0, float(dt))
        gate, reason = self._gate(now_ms, allow)
        self.active = gate
        self.reason = reason
        self.theta_target_deg = self._target() if gate else 0.0

        if dt <= 0.0:
            self.omega_dps = 0.0
            return self.telemetry(now_ms)

        if gate:
            alpha = min(1.0, dt / (self.attack_ms / 1000.0))
            step = (self.theta_target_deg - self.theta_deg) * alpha
        else:
            # Linear release reaches zero within release_ms even from max_deg,
            # satisfying the protocol's 200-500 ms withdrawal requirement.
            release_rate = self.max_deg / (self.release_ms / 1000.0)
            step = _clamp(-self.theta_deg, release_rate * dt)

        step = _clamp(step, self.omega_max_dps * dt)
        if abs(step) >= abs(self.theta_deg) and not gate:
            step = -self.theta_deg
        self.theta_deg = _clamp(self.theta_deg + step, self.max_deg)
        self.omega_dps = _clamp(step / dt, self.omega_max_dps)
        return self.telemetry(now_ms)

    def telemetry(self, now_ms):
        return {
            "active": self.active,
            "reason": self.reason,
            "target_deg": self.theta_target_deg,
            "theta_deg": self.theta_deg,
            "omega_dps": self.omega_dps,
            "age_ms": self.frame_age_ms(now_ms),
            "imu_bias_mg": self.imu_bias_mg if self.imu_bias_valid else None,
        }

    def _gate(self, now_ms, allow):
        if not allow:
            return False, "mode"
        if not self.enabled:
            return False, "disabled"
        if self.latest is None or self.last_new_ms is None:
            return False, "no-data"
        if self.frame_age_ms(now_ms) > self.timeout_ms:
            return False, "timeout"
        if not self.latest.task_456:
            return False, "task"
        if self.latest.state not in (STATE_RUNNING, STATE_SOFT_STOP):
            return False, "state"
        return True, "active"

    def _target(self):
        frame = self.latest
        target = self.k_pwm * frame.forward_pwm_rate
        if frame.encoder_valid:
            target += self.k_encoder * frame.encoder_accel_mms2
        if frame.imu_valid and self.imu_bias_valid:
            corrected = (frame.imu_axis_mg(self.imu_axis)
                         - self.imu_bias_mg)
            target += self.k_imu * self.imu_sign * corrected
        return _clamp(target, self.max_deg)
