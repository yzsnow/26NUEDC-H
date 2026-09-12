"""PID for ball-and-beam: PD-dominant, derivative on measurement.

out_deg = Kp*err_p - Kd*v_meas + I
  - err in cm, v_meas in cm/s (from the alpha-beta filter, NOT differentiated
    position), output is motor target angle in degrees.
  - Derivative-on-measurement means a target step produces no D kick, which is
    what makes the TASK3 +5 -> -5 setpoint switch safe.
  - The deadband is applied to the P term only (err_p), never to the output:
    pixel noise no longer dithers the beam, but the integral keeps trimming the
    real mechanical zero offset all the way in.  Freezing the whole output
    inside the deadband (the earlier design) was what left the ball parked a
    few millimetres off centre and then let it drift back out again.
  - Conditional integration: only near the target, clamped, and rejected when
    it would push further into output saturation.
"""


class PID:
    def __init__(self, kp, ki, kd,
                 out_limit_deg=30.0,
                 i_limit_deg=8.0,
                 i_zone_cm=4.0,
                 err_deadband_cm=0.08,
                 v_deadband_cms=0.3,
                 d_lpf_alpha=0.5,
                 slew_deg_per_update=6.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit_deg = out_limit_deg
        self.i_limit_deg = i_limit_deg
        self.i_zone_cm = i_zone_cm
        self.err_deadband_cm = err_deadband_cm
        self.v_deadband_cms = v_deadband_cms
        self.d_lpf_alpha = d_lpf_alpha
        self.slew_deg_per_update = slew_deg_per_update
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.v_filt = 0.0
        self.last_out = 0.0

    def update(self, err_cm, vel_cm_s, dt):
        a = self.d_lpf_alpha
        self.v_filt = a * vel_cm_s + (1.0 - a) * self.v_filt

        # deadband on the P/D terms only -- the integral below still sees the
        # raw error, so a static offset gets trimmed out instead of parked in
        err_p = 0.0 if abs(err_cm) < self.err_deadband_cm else err_cm
        v_d = 0.0 if abs(self.v_filt) < self.v_deadband_cms else self.v_filt
        pd = self.kp * err_p - self.kd * v_d

        if dt > 0 and abs(err_cm) < self.i_zone_cm:
            cand = self.integral + self.ki * err_cm * dt
            cand = max(-self.i_limit_deg, min(self.i_limit_deg, cand))
            # anti-windup: accept only if it does not deepen output saturation
            if (abs(pd + cand) < self.out_limit_deg
                    or abs(cand) < abs(self.integral)):
                self.integral = cand

        out = pd + self.integral
        out = max(-self.out_limit_deg, min(self.out_limit_deg, out))

        # per-update slew limit protects the crank linkage from detection jumps
        step = self.slew_deg_per_update
        out = max(self.last_out - step, min(self.last_out + step, out))

        self.last_out = out
        return out
