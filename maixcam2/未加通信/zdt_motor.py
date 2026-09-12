"""ZDT Y42 second-gen closed-loop stepper, Emm firmware, UART TTL protocol.

Frame format (host -> motor):  addr + code + payload + checksum(0x6B fixed)
Motor replies:                 addr + code + 02(OK)/E2(cond err)/EE(fmt err) + 6B
                               addr + FD + 9F + 6B  (position reached, motor-initiated)

Two ways to drive the cascade's angular-velocity command:

  move_abs_deg()  absolute position (0xFD) at a speed matched to the commanded
                  angular rate.  Resolution is one microstep (0.1125 deg) and
                  the motor stops on its own if the host stops talking, so this
                  is the default.
  move_vel_dps()  velocity mode (0xF6).  Truly continuous motion, but the speed
                  field is an INTEGER rpm, i.e. 6 motor deg/s per LSB, so fine
                  trim near the setpoint quantises badly.  It also keeps
                  spinning until told otherwise -- the caller must command
                  every frame and stop on every exit path.

Angle convention: absolute degrees around the power-on zero (0x0A clears the
motor's position register, so call set_zero() once at startup while the beam
is level).  Positive/negative sign is mapped to the direction byte; the Emm FD
command takes an unsigned pulse count + CW/CCW flag in absolute mode.
"""

PULSES_PER_REV = 3200  # 1.8deg motor, 16 microsteps -> 0.1125 deg/pulse
DEG_PER_SEC_PER_RPM = 6.0  # 1 rpm = 360 deg / 60 s


class ZdtEmmMotor:
    def __init__(self, serial, addr=0x01,
                 max_abs_deg=90.0,
                 heartbeat_ms=200):
        """serial: an opened maix uart.UART (or anything with write/read)."""
        self.serial = serial
        self.addr = addr
        self.max_abs_deg = float(max_abs_deg)
        self.heartbeat_ms = heartbeat_ms

        self.last_sent_deg = None
        self.last_sent_pulses = None
        self.last_sent_dir = None
        self.last_sent_rpm = None
        self.last_sent_ms = None
        self.last_pos_req_ms = None
        self.tx_count = 0
        self.rx_count = 0
        self.e2_count = 0
        self.e2_streak = 0
        self.ee_count = 0
        self.reached_flag = False
        self._rx = bytearray()

    # ---------- frame helpers ----------

    def _send(self, payload_bytes):
        frame = bytes([self.addr]) + bytes(payload_bytes) + b"\x6b"
        self.serial.write(frame)
        self.tx_count += 1
        return frame

    def _forget_last(self):
        self.last_sent_deg = None
        self.last_sent_pulses = None
        self.last_sent_dir = None
        self.last_sent_rpm = None

    @staticmethod
    def deg_to_pulses(deg):
        return int(round(abs(deg) * PULSES_PER_REV / 360.0))

    @staticmethod
    def rpm_for_dps(dps, rpm_min=8, rpm_max=300, lead=1.2):
        """Motor speed that covers |dps| deg/s, with a little margin.

        Matching the move speed to the commanded angular rate is what makes a
        position-mode cascade look continuous: the motor arrives just as the
        next frame's command lands, instead of snapping to the target and
        waiting (which is what a fixed high rpm does, and it feels like a
        vibrating beam at 30 fps).
        """
        rpm = int(round(abs(dps) * lead / DEG_PER_SEC_PER_RPM))
        return max(int(rpm_min), min(int(rpm_max), max(1, rpm)))

    # ---------- commands ----------

    def enable(self, on=True):
        # addr F3 AB 01/00 sync(00) 6B
        self._send([0xF3, 0xAB, 0x01 if on else 0x00, 0x00])

    def set_zero(self):
        """Clear current position register -> current pose becomes 0 deg."""
        self._send([0x0A, 0x6D])
        self._forget_last()

    def stop_now(self):
        # addr FE 98 sync(00) 6B
        self._send([0xFE, 0x98, 0x00])
        self._forget_last()

    def request_position(self):
        """Ask for realtime position (0x36). Reply parsed by poll()."""
        self._send([0x36])

    def maybe_request_position(self, now_ms, period_ms=0):
        """Throttled 0x36 so the travel guard sees the real angle.

        DISABLED BY DEFAULT (period_ms <= 0).  The 0x36 function code is not
        accepted by every ZDT firmware revision -- on this rig the motor answers
        every 0x36 with EE (bad frame), which floods the alarm line and hides
        real faults.  The readback is telemetry only: theta_cmd is the
        authoritative angle, so losing it costs nothing.  Set pos_poll_ms > 0
        only after confirming your firmware answers 0x36 properly.
        """
        if now_ms is None or period_ms <= 0:
            return False
        if (self.last_pos_req_ms is not None
                and now_ms - self.last_pos_req_ms < period_ms):
            return False
        self.last_pos_req_ms = now_ms
        self.request_position()
        return True

    def move_abs_deg(self, deg, rpm, acc, now_ms=None, force=False):
        """Absolute position command (Emm 0xFD).  Returns True if a frame went out.

        Skipped when the target rounds to the same microstep as the last one --
        the motor literally cannot act on a smaller change -- unless
        heartbeat_ms has elapsed or force=True.  The final safety clamp to
        +/-max_abs_deg lives here on purpose.
        """
        deg = max(-self.max_abs_deg, min(self.max_abs_deg, float(deg)))
        direction = 0x00 if deg >= 0 else 0x01  # CW for +, CCW for -
        rpm = int(max(1, min(3000, rpm)))
        acc = int(max(0, min(255, acc)))
        # the travel limit is usually not a whole number of microsteps, so round
        # the target but truncate the limit -- otherwise rounding up walks the
        # beam a fraction of a step past the boundary the clamp is protecting
        max_pulses = int(abs(self.max_abs_deg) * PULSES_PER_REV / 360.0)
        pulses = min(self.deg_to_pulses(deg), max_pulses)

        if not force:
            same = (pulses == self.last_sent_pulses
                    and direction == self.last_sent_dir
                    and rpm == self.last_sent_rpm)
            fresh = (now_ms is not None and self.last_sent_ms is not None
                     and now_ms - self.last_sent_ms < self.heartbeat_ms)
            if same and fresh:
                return False

        self._send([
            0xFD, direction,
            (rpm >> 8) & 0xFF, rpm & 0xFF,
            acc,
            (pulses >> 24) & 0xFF, (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF, pulses & 0xFF,
            0x01,  # absolute move
            0x00,  # execute immediately
        ])
        self.last_sent_deg = deg
        self.last_sent_pulses = pulses
        self.last_sent_dir = direction
        self.last_sent_rpm = rpm
        self.last_sent_ms = now_ms
        return True

    def move_vel_dps(self, dps, acc=0, now_ms=None, force=False):
        """Velocity mode (Emm 0xF6): addr F6 dir vel_h vel_l acc sync 6B.

        WARNING: the rpm field is an integer, so the smallest non-zero step is
        6 motor deg/s.  Anything below ~3 deg/s becomes a dead zone.  Also, the
        motor keeps turning after the last frame -- always call stop_now() (or
        move_vel_dps(0)) when leaving a closed-loop mode.
        """
        acc = int(max(0, min(255, acc)))
        direction = 0x00 if dps >= 0 else 0x01
        rpm = int(round(abs(dps) / DEG_PER_SEC_PER_RPM))
        rpm = max(0, min(3000, rpm))

        if not force:
            same = (rpm == self.last_sent_rpm
                    and (rpm == 0 or direction == self.last_sent_dir))
            fresh = (now_ms is not None and self.last_sent_ms is not None
                     and now_ms - self.last_sent_ms < self.heartbeat_ms)
            if same and fresh:
                return False

        self._send([
            0xF6, direction,
            (rpm >> 8) & 0xFF, rpm & 0xFF,
            acc,
            0x00,  # no multi-motor sync
        ])
        self.last_sent_deg = None
        self.last_sent_pulses = None
        self.last_sent_dir = direction
        self.last_sent_rpm = rpm
        self.last_sent_ms = now_ms
        return True

    # ---------- rx parsing ----------

    # reply lengths per function code: addr + code + data... + 6B
    _REPLY_LEN = {
        0xF3: 4, 0xFD: 4, 0xF6: 4, 0xFE: 4, 0x0A: 4, 0x06: 4, 0x08: 4,
        0x0E: 4, 0x93: 4, 0x9A: 4,
        0x36: 8,  # addr 36 sign uint32 6B; Emm: deg = raw * 360 / 65536
    }

    def poll(self):
        """Non-blocking drain of RX. Returns a list of event strings.

        Events: 'ok', 'reached', 'e2', 'ee', ('pos', deg).
        'e2' bumps e2_streak so the caller can escalate (re-enable / estop).
        """
        events = []
        try:
            available = int(self.serial.available(0))
        except Exception:
            available = 0
        if available > 0:
            data = self.serial.read(available, 0)
            if data:
                self._rx.extend(bytes(data))

        while len(self._rx) >= 3:
            if self._rx[0] != self.addr:
                self._rx.pop(0)
                continue
            code = self._rx[1]
            need = self._REPLY_LEN.get(code, 4)
            # error replies (E2/EE) are always addr+code+err+6B
            if len(self._rx) < 4:
                break
            if self._rx[2] in (0xE2, 0xEE):
                need = 4
            if len(self._rx) < need:
                break
            frame = bytes(self._rx[:need])
            del self._rx[:need]
            # counted before the checksum test: "frames arrived at all" is the
            # question this answers, and a silent bus looks exactly like a
            # motor that is enabled and simply refusing to move
            self.rx_count += 1
            if frame[-1] != 0x6B:
                # lost sync: drop one byte and rescan
                self._rx[0:0] = frame[1:]
                continue

            status = frame[2]
            if status == 0xE2:
                self.e2_count += 1
                self.e2_streak += 1
                events.append("e2")
            elif status == 0xEE:
                self.ee_count += 1
                events.append("ee")
            elif code == 0xFD and status == 0x9F:
                self.reached_flag = True
                events.append("reached")
            elif code == 0x36 and len(frame) == 8:
                sign = -1.0 if frame[2] == 0x01 else 1.0
                raw = (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]
                events.append(("pos", sign * raw * 360.0 / 65536.0))
                self.e2_streak = 0
            else:
                events.append("ok")
                self.e2_streak = 0
        return events
