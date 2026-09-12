"""Three-level cascaded ball-and-beam controller (2026 NUEDC problem H).

    x_ref  -> [position loop]      -> v_ref      cm/s  (+ braking speed cap)
    v_ref  -> [ball velocity loop] -> omega_ref  motor deg/s   (+ accel damping)
           -> [travel limiter]     -> omega      respects the crank travel
           -> integrate            -> theta_cmd  motor deg

Why a cascade instead of one position PID: the plant from motor angle to ball
position is a double integrator sitting behind a gravity term, so a flat
position PID has to simultaneously hold a static tilt (to fight the mechanical
zero offset) and damp the ball, using the same single output.  Here the static
tilt is produced by theta = integral(omega), so a zero offset is trimmed out by
the structure itself and no position integral is needed -- ki_x defaults to 0,
matching the reference design.  That is also why the ball can settle exactly on
O instead of parking a few millimetres off.

Units: the ball side is cm, cm/s, cm/s^2.  The actuator side is MOTOR degrees
and motor deg/s (not beam degrees) because motor degrees are what gets
commanded and what the travel limit is calibrated in.

Sign convention lives in exactly one place -- `dir_sign` below, fed from
CONFIG["dir_invert"].  Never add a second negation inside a loop.

HOLD deliberately uses the same closed-loop ideas as TASK3's final -5 cm hold:
fast position-difference velocity, braking speed cap, and blended far/near gains.
It remains a separate controller so HOLD tuning cannot change TASK3.

This module is hardware independent: no pixels, no UART, no maix imports.
"""

import math


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _blend(far, near, err_cm, band_cm):
    """Linearly blend near/far gains without a discontinuous threshold."""
    if band_cm <= 0.0:
        return far
    t = min(1.0, abs(err_cm) / band_cm)
    return near + (far - near) * t


class PositionLoop:
    """Position outer loop with TASK3-style gain blending and braking cap."""

    def __init__(self, kp_far=1.50, kp_near=1.50, near_band_cm=2.0,
                 ki=0.0, v_max_cms=12.0, v_floor_cms=3.0,
                 a_brake_cms2=16.0, i_limit_cms=3.0, i_zone_cm=2.0):
        self.kp_far = kp_far
        self.kp_near = kp_near
        self.near_band_cm = near_band_cm
        self.ki = ki
        self.v_max_cms = v_max_cms
        self.v_floor_cms = v_floor_cms
        self.a_brake_cms2 = a_brake_cms2
        self.i_limit_cms = i_limit_cms
        self.i_zone_cm = i_zone_cm
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.err_cm = 0.0

    def update(self, x_ref_cm, x_est_cm, dt):
        err = x_ref_cm - x_est_cm
        self.err_cm = err
        kp = _blend(self.kp_far, self.kp_near, err, self.near_band_cm)
        v_ref = kp * err
        # The position object already integrates; a large ki_x would wind up
        # whenever the ball is stuck or vision drops out, so it stays optional,
        # zone limited and clamped.
        if self.ki > 0.0 and dt > 0.0 and abs(err) < self.i_zone_cm:
            self.integral = _clamp(self.integral + self.ki * err * dt,
                                   self.i_limit_cms)
            v_ref += self.integral
        else:
            self.integral = 0.0 if self.ki <= 0.0 else self.integral

        # Do not approach the target faster than the configured deceleration
        # can stop.  The floor preserves authority for the final small error.
        v_brake = math.sqrt(2.0 * max(0.0, self.a_brake_cms2) * abs(err))
        v_cap = min(self.v_max_cms, max(self.v_floor_cms, v_brake))
        return _clamp(v_ref, v_cap)


class BallSpeedEstimator:
    """Low-latency ball speed used by the control loop.

    The shared alpha-beta filter provides a smooth position, but its velocity
    state lags by several frames.  TASK3 avoids that lag by differentiating the
    filtered position, limiting physically impossible one-frame acceleration,
    then applying a short LPF.  HOLD uses the same method here.
    """

    def __init__(self, from_position=True, lpf=0.7, a_max_cms2=200.0):
        self.from_position = from_position
        self.lpf = lpf
        self.a_max_cms2 = a_max_cms2
        self.reset()

    def reset(self):
        self.x_prev = None
        self.v_est = 0.0

    def update(self, x_cm, external_v_cms, dt):
        if not self.from_position:
            self.x_prev = x_cm
            self.v_est = external_v_cms
            return self.v_est
        if self.x_prev is None or dt <= 0.0:
            self.x_prev = x_cm
            return self.v_est

        v_raw = (x_cm - self.x_prev) / dt
        self.x_prev = x_cm
        max_step = abs(self.a_max_cms2) * dt
        v_raw = max(self.v_est - max_step, min(self.v_est + max_step, v_raw))
        alpha = min(max(self.lpf, 0.0), 1.0)
        self.v_est += alpha * (v_raw - self.v_est)
        return self.v_est


class BallVelocityLoop:
    """Ball velocity loop with acceleration damping: -> motor deg/s.

    omega = +kv * (v_ref - v_est) - ka * a_est

    SIGN CONVENTION -- this matches the rig, not the reference document.  The
    reference writes omega = -Kv*err because its beam-angle positive direction is
    defined the other way round; on this machine the previously working flat-PID
    version drove `out_deg = +kp * (x_ref - x)` with dir_invert = 0 and tilted
    the correct way.  So a positive position error must produce a POSITIVE motor
    angle here too, otherwise dir_invert = 0 means "wrong way" and every sign in
    the project ends up inverted relative to the working baseline.

    The ka term is lead compensation from the measured acceleration; it carries
    the opposite sign to the main term so it brakes the ball before it reaches
    the setpoint instead of after.
    """

    def __init__(self, kv_far=4.4, kv_near=4.4,
                 ka_far=1.5, ka_near=1.5, near_band_cm=2.0,
                 omega_max_dps=90.0, a_lpf=0.5, a_max_cms2=200.0):
        self.kv_far = kv_far
        self.kv_near = kv_near
        self.ka_far = ka_far
        self.ka_near = ka_near
        self.near_band_cm = near_band_cm
        self.omega_max_dps = omega_max_dps
        self.a_lpf = a_lpf
        self.a_max_cms2 = a_max_cms2
        self.reset()

    def reset(self):
        self.v_prev = None
        self.a_est = 0.0
        self.err_cms = 0.0

    def update(self, v_ref_cms, v_est_cms, err_cm, dt):
        if self.v_prev is None or dt <= 0.0:
            a_raw = 0.0
        else:
            a_raw = (v_est_cms - self.v_prev) / dt
        self.v_prev = v_est_cms
        # a single bad detection frame becomes a huge a_raw; clamp before it
        # reaches the filter so one outlier cannot slam the beam
        a_raw = _clamp(a_raw, self.a_max_cms2)
        self.a_est += self.a_lpf * (a_raw - self.a_est)

        self.err_cms = v_ref_cms - v_est_cms
        kv = _blend(self.kv_far, self.kv_near, err_cm, self.near_band_cm)
        ka = _blend(self.ka_far, self.ka_near, err_cm, self.near_band_cm)
        omega = kv * self.err_cms - ka * self.a_est
        return _clamp(omega, self.omega_max_dps)


class TravelLimiter:
    """Scale down angular velocity that would push the crank past its travel.

    Inside the soft band the outward command is scaled by the remaining travel,
    so the beam decelerates into the limit instead of hitting it.  Motion back
    towards centre is never restricted.
    """

    # Scaling in the soft band is asymptotic: omega -> 0 as theta -> limit, so
    # theta creeps towards the limit and never actually equals it.  Treating
    # "saturated" as travel >= limit would therefore never be true and the
    # stuck detector downstream could never fire, so saturation means "the
    # outward command has been choked down to nothing" instead.
    SATURATED_SCALE = 0.05

    def __init__(self, theta_limit_deg=30.0, soft_frac=0.75):
        self.theta_limit_deg = theta_limit_deg
        self.soft_frac = soft_frac
        self.active = False
        self.saturated = False

    def apply(self, omega_dps, theta_deg):
        self.active = False
        self.saturated = False
        limit = abs(self.theta_limit_deg)
        if limit <= 0.0:
            self.active = True
            self.saturated = True
            return 0.0

        outward = (omega_dps > 0.0 and theta_deg > 0.0) or \
                  (omega_dps < 0.0 and theta_deg < 0.0)
        if not outward:
            return omega_dps

        travel = abs(theta_deg)
        if travel >= limit:
            self.active = True
            self.saturated = True
            return 0.0

        soft = limit * min(max(self.soft_frac, 0.0), 1.0)
        if travel <= soft:
            return omega_dps

        span = limit - soft
        scale = 0.0 if span <= 1e-6 else (limit - travel) / span
        scale = min(max(scale, 0.0), 1.0)
        self.active = True
        self.saturated = scale <= self.SATURATED_SCALE
        return omega_dps * scale


class StuckMonitor:
    """Detect a ball that will not move while the crank is already at its stop.

    Position error present + ball essentially stationary + travel limiter
    saturated means more command cannot help, so the caller should fault out
    and stop asking for it (reference section 12.2).
    """

    def __init__(self, err_cm=1.5, v_cms=0.5, hold_ms=1500):
        self.err_cm = err_cm
        self.v_cms = v_cms
        self.hold_ms = hold_ms
        self.since_ms = None

    def reset(self):
        self.since_ms = None

    def update(self, err_cm, v_cms, limiter_saturated, now_ms):
        bad = (abs(err_cm) > self.err_cm
               and abs(v_cms) < self.v_cms
               and limiter_saturated)
        if not bad:
            self.since_ms = None
            return False
        if self.since_ms is None:
            self.since_ms = now_ms
            return False
        return now_ms - self.since_ms >= self.hold_ms


class CascadeController:
    """The three loops plus crank-angle bookkeeping.

    Owns `theta_cmd`, the commanded motor angle.  It is a pure integration of
    the applied omega and is never touched by the motor position readback, so
    the absolute-position frames sent to the driver stay a clean monotone
    signal.  The readback is kept separately in `theta_meas` and only feeds the
    travel guard, which uses whichever of the two is further out.
    """

    def __init__(self, cfg):
        self.pos = PositionLoop()
        self.speed = BallSpeedEstimator()
        self.vel = BallVelocityLoop()
        self.limiter = TravelLimiter()
        self.stuck = StuckMonitor()
        self.dir_sign = 1.0
        self.x_guard_cm = 10.0
        self.x_ref_max_cm = 10.0
        self.relax_dps = 25.0
        self.theta_cmd = 0.0
        self.theta_meas = None
        self.omega = 0.0
        self.omega_feedback = 0.0
        self.omega_feedforward = 0.0
        self.v_ref_cms = 0.0
        self.sync(cfg)

    # ---------- configuration ----------

    def sync(self, cfg):
        """Pull the live tunables.  Safe to call every frame."""
        self.pos.kp_far = cfg["kp_far"]
        self.pos.kp_near = cfg["kp_near"]
        self.pos.near_band_cm = cfg["near_band_cm"]
        self.pos.ki = cfg["ki_x"]
        self.pos.v_max_cms = cfg["v_max_cms"]
        self.pos.v_floor_cms = cfg["v_floor_cms"]
        self.pos.a_brake_cms2 = cfg["a_brake_cms2"]
        self.pos.i_limit_cms = cfg["i_lim_cms"]
        self.pos.i_zone_cm = cfg["i_zone_cm"]

        self.speed.from_position = bool(cfg["v_from_pos"])
        self.speed.lpf = cfg["v_lpf"]
        self.speed.a_max_cms2 = cfg["a_max_cms2"]

        self.vel.kv_far = cfg["kv_far"]
        self.vel.kv_near = cfg["kv_near"]
        self.vel.ka_far = cfg["ka_far"]
        self.vel.ka_near = cfg["ka_near"]
        self.vel.near_band_cm = cfg["near_band_cm"]
        self.vel.omega_max_dps = cfg["omega_max_dps"]
        self.vel.a_lpf = cfg["a_lpf"]
        self.vel.a_max_cms2 = cfg["a_max_cms2"]

        self.limiter.theta_limit_deg = cfg["max_motor_deg"]
        self.limiter.soft_frac = cfg["soft_frac"]

        self.stuck.err_cm = cfg["stuck_err_cm"]
        self.stuck.v_cms = cfg["stuck_v_cms"]
        self.stuck.hold_ms = cfg["stuck_ms"]

        self.dir_sign = -1.0 if cfg["dir_invert"] else 1.0
        self.x_guard_cm = cfg["x_guard_cm"]
        self.x_ref_max_cm = cfg["x_ref_max_cm"]
        self.relax_dps = cfg["relax_dps"]

    def reset(self, theta_deg=None):
        """Reset the loops.  theta_deg=0.0 after a set_zero, else keep the pose."""
        self.pos.reset()
        self.speed.reset()
        self.vel.reset()
        self.stuck.reset()
        self.omega = 0.0
        self.omega_feedback = 0.0
        self.omega_feedforward = 0.0
        self.v_ref_cms = 0.0
        if theta_deg is not None:
            self.theta_cmd = float(theta_deg)
            self.theta_meas = None

    def note_measured_theta(self, deg):
        self.theta_meas = float(deg)

    @property
    def theta_guard(self):
        """Most conservative angle estimate for the travel limiter."""
        if self.theta_meas is None:
            return self.theta_cmd
        return self.theta_cmd if abs(self.theta_cmd) >= abs(self.theta_meas) \
            else self.theta_meas

    # ---------- control ----------

    def update(self, x_ref_cm, x_est_cm, v_est_cms, dt, now_ms,
               omega_ff_dps=0.0):
        """Run all three levels.  Returns a telemetry dict."""
        x_ref_cm = _clamp(x_ref_cm, self.x_ref_max_cm)
        v_ctrl_cms = self.speed.update(x_est_cm, v_est_cms, dt)

        v_ref = self.pos.update(x_ref_cm, x_est_cm, dt)
        v_ref = self._guard_ends(v_ref, x_est_cm)
        self.v_ref_cms = v_ref

        omega_feedback = self.vel.update(
            v_ref, v_ctrl_cms, self.pos.err_cm, dt) * self.dir_sign
        # Feed-forward is already expressed in final motor coordinates.  Clamp
        # after summation so it cannot push the combined command past the same
        # angular-velocity ceiling that protects visual feedback alone.
        self.omega_feedback = omega_feedback
        self.omega_feedforward = float(omega_ff_dps)
        omega = _clamp(omega_feedback + self.omega_feedforward,
                       self.vel.omega_max_dps)
        omega = self.limiter.apply(omega, self.theta_guard)
        self.omega = omega

        self.theta_cmd = _clamp(self.theta_cmd + omega * dt,
                                self.limiter.theta_limit_deg)

        stuck = self.stuck.update(self.pos.err_cm, v_ctrl_cms,
                                  self.limiter.saturated, now_ms)
        if stuck:
            self.pos.integral = 0.0

        return {
            "err_cm": self.pos.err_cm,
            "v_ref_cms": v_ref,
            "v_est_cms": v_ctrl_cms,
            "omega_dps": omega,
            "omega_feedback_dps": self.omega_feedback,
            "omega_feedforward_dps": self.omega_feedforward,
            "theta_cmd": self.theta_cmd,
            "a_est_cms2": self.vel.a_est,
            "limited": self.limiter.active,
            "stuck": stuck,
        }

    def relax(self, dt):
        """Vision lost: walk the beam back to level at a bounded rate.

        Snapping straight to 0 deg would throw the ball; ramping keeps the pose
        predictable and stops the tilt from growing (reference section 12.1).
        """
        self.pos.reset()
        self.speed.reset()
        self.vel.reset()
        self.stuck.reset()
        self.omega_feedback = 0.0
        self.omega_feedforward = 0.0
        if abs(self.theta_cmd) <= 1e-3:
            self.theta_cmd = 0.0
            self.omega = 0.0
        else:
            step = self.relax_dps * dt
            direction = -1.0 if self.theta_cmd > 0.0 else 1.0
            if step >= abs(self.theta_cmd):
                self.theta_cmd = 0.0
                self.omega = 0.0
            else:
                self.omega = direction * self.relax_dps
                self.theta_cmd += direction * step
        self.v_ref_cms = 0.0
        return {
            "err_cm": 0.0,
            "v_ref_cms": 0.0,
            "omega_dps": self.omega,
            "omega_feedback_dps": 0.0,
            "omega_feedforward_dps": 0.0,
            "theta_cmd": self.theta_cmd,
            "a_est_cms2": 0.0,
            "limited": False,
            "stuck": False,
        }

    # ---------- helpers ----------

    def _guard_ends(self, v_ref_cms, x_est_cm):
        """Fade out any target speed that drives the ball further into an end."""
        guard = abs(self.x_guard_cm)
        travel = abs(x_est_cm)
        if travel <= guard:
            return v_ref_cms
        outward = (v_ref_cms > 0.0 and x_est_cm > 0.0) or \
                  (v_ref_cms < 0.0 and x_est_cm < 0.0)
        if not outward:
            return v_ref_cms
        span = max(1.0, 12.5 - guard)
        scale = min(max((12.5 - travel) / span, 0.0), 1.0)
        return v_ref_cms * scale
