"""Open-loop plant probe: how many motor degrees move the ball, and which way.

Not part of the contest app -- a bench tool.  Gains cannot be chosen from first
principles here because two rig numbers are unknown: the crank ratio (motor
degrees per beam degree) and the breakaway friction angle.  Guessing them in
closed loop is how the beam ends up pinned against its stop, so this walks the
commanded angle up a staircase with the loop OFF and records what the ball does.

It reuses the detection path from main.py verbatim (same ROI crop, same YOLO26
call, same alpha-beta filter) so the position numbers are directly comparable.

Sequence: 0 -> +step ... -> +max (or until the ball breaks away) -> 0 -> pause
-> the same on the negative side -> 0 -> stop.  Every step is short and the
angle ceiling is deliberately below the travel the flat-PID version already
exercised on this rig.

PROBE_MODE=ramp instead walks the angle up continuously at PROBE_RAMP_DPS and
reports the angle at which the ball first moves.  That number, not the staircase
one, is what the controller actually faces: a 4 deg step at 360 deg/s delivers an
impulse that knocks the ball out of stiction, while a closed loop easing the beam
over at 20 deg/s does not, so the staircase flatters the rig.

Env knobs (set by .maixpy/tools/run_stub.py):
    PROBE_MODE      "stair" (default) or "ramp"
    PROBE_MAX_DEG   angle ceiling, default 24
    PROBE_STEP_DEG  staircase increment, default 4
    PROBE_HOLD_MS   dwell per step, default 800
    PROBE_BREAK_CM  ball travel that counts as "broke away", default 2.5
    PROBE_RAMP_DPS  ramp rate in ramp mode, default 6
"""

import os

from maix import camera, display, image, nn, app, time, sys, uart, touchscreen

from ball_position import (
    AdaptiveAlphaBetaFilter,
    position_from_pixel,
    validate_calibration,
)
from zdt_motor import ZdtEmmMotor

MODE = os.environ.get("PROBE_MODE", "stair").strip().lower()
RAMP_DPS = float(os.environ.get("PROBE_RAMP_DPS", "6"))
MAX_DEG = float(os.environ.get("PROBE_MAX_DEG", "24"))
STEP_DEG = float(os.environ.get("PROBE_STEP_DEG", "4"))
HOLD_MS = int(os.environ.get("PROBE_HOLD_MS", "800"))
BREAK_CM = float(os.environ.get("PROBE_BREAK_CM", "2.5"))
RPM = int(os.environ.get("PROBE_RPM", "60"))
ACC = int(os.environ.get("PROBE_ACC", "10"))
CONF_TH = float(os.environ.get("PROBE_CONF", "0.7"))

MIN_FRAME_HEIGHT = 180
DETECTION_ROI_HEIGHT = 170
AXIS_START_CM = -12.5
AXIS_END_CM = 12.5
MAIXCAM_MODEL_PATH = "models/yolo26_all_maixcam_yolo26_480_160/yolo26_all.mud"
MAIXCAM2_MODEL_PATH = "models/my_ball_maixcam2/best.mud"


def model_path_for_device(device_name):
    name = device_name.strip().lower()
    if name == "maixcam2":
        return MAIXCAM2_MODEL_PATH
    if name in ("maixcam", "maixcam-pro", "maixcam_pro"):
        return MAIXCAM_MODEL_PATH
    raise ValueError("unsupported device: {}".format(device_name))


detector = nn.YOLO26(model=model_path_for_device(sys.device_name()),
                     dual_buff=False)
FRAME_WIDTH = detector.input_width()
FRAME_HEIGHT = max(detector.input_height(), MIN_FRAME_HEIGHT)
ROI_X = 0
ROI_Y = (FRAME_HEIGHT - DETECTION_ROI_HEIGHT) // 2
AXIS_START_PX = (5, FRAME_HEIGHT // 2)
AXIS_END_PX = (FRAME_WIDTH - 5, FRAME_HEIGHT // 2)
AXIS_LEN_PX = AXIS_END_PX[0] - AXIS_START_PX[0]
validate_calibration(AXIS_START_PX, AXIS_END_PX, AXIS_START_CM, AXIS_END_CM,
                     FRAME_WIDTH, FRAME_HEIGHT)

cam = camera.Camera(FRAME_WIDTH, FRAME_HEIGHT, detector.input_format())
disp = display.Display()
ts = touchscreen.TouchScreen()
position_filter = AdaptiveAlphaBetaFilter()

from maix import err
try:
    from maix.peripheral import pinmap
except ImportError:
    from maix import pinmap
err.check_raise(pinmap.set_pin_function("B0", "UART2_TX"), "probe: UART TX pin")
err.check_raise(pinmap.set_pin_function("B1", "UART2_RX"), "probe: UART RX pin")
serial = uart.UART("/dev/ttyS2", 115200)
motor = ZdtEmmMotor(serial, addr=0x01, max_abs_deg=MAX_DEG)


def build_plan():
    """(label, angle, dwell_ms) steps.  Both signs, with a rest at level."""
    steps = []
    n = max(1, int(round(MAX_DEG / STEP_DEG)))
    for sign, tag in ((1.0, "+"), (-1.0, "-")):
        for i in range(1, n + 1):
            deg = sign * min(MAX_DEG, i * STEP_DEG)
            steps.append(("A{}{:02.0f}".format(tag, abs(deg)), deg, HOLD_MS))
        steps.append(("REST" + tag, 0.0, 1500))
    return steps


PLAN = build_plan()
if MODE == "ramp":
    print("probe plan: ramp +/-{:.0f} deg at {:.1f} deg/s, break at {:.1f} cm"
          .format(MAX_DEG, RAMP_DPS, BREAK_CM))
else:
    print("probe plan:", " ".join("{}@{:+.0f}".format(s[0], s[1]) for s in PLAN))
print("T,ms,mode,dt_ms,target_cm,pos_cm,v_cms,v_ref_cms,omega_dps,theta_deg,"
      "a_est,limited,tx,e2,ee")

motor.enable(True)
time.sleep_ms(50)
motor.set_zero()
print("probe: motor zeroed at current pose, rpm", RPM, "acc", ACC)

step_idx = 0
step_since = None
step_start_pos = None
t0 = None
theta = 0.0
last_ms = time.ticks_ms()
aborted = ""
results = []
ramp_phase = 0
ramp_start_pos = None
ramp_wait_until = None

try:
    while not app.need_exit():
        now_ms = time.ticks_ms()
        loop_ms = now_ms - last_ms
        last_ms = now_ms
        if t0 is None:
            t0 = now_ms

        img = cam.read()
        roi = img.crop(ROI_X, ROI_Y, FRAME_WIDTH, DETECTION_ROI_HEIGHT)
        objs = detector.detect(roi, conf_th=CONF_TH, iou_th=0.45)
        del roi
        objs = [o for o in objs if o.score >= CONF_TH]
        ball = max(objs, key=lambda o: o.score) if objs else None

        pos_cm = None
        v_cms = 0.0
        if ball is not None:
            bx = ROI_X + ball.x
            by = ROI_Y + ball.y
            fx, fy = position_filter.update(bx + ball.w * 0.5,
                                            by + ball.h * 0.5, now_ms)
            pos_cm, _r, _d = position_from_pixel((fx, fy), AXIS_START_PX,
                                                 AXIS_END_PX, AXIS_START_CM,
                                                 AXIS_END_CM)
            v_cms = position_filter.vx * 25.0 / AXIS_LEN_PX
            img.draw_rect(bx, by, ball.w, ball.h, image.COLOR_GREEN, 2)
        else:
            position_filter.mark_missing(now_ms)

        # a touch anywhere aborts: the only exit path that does not need SSH
        _tx, _ty, pressed = ts.read()
        if pressed:
            aborted = "touch"
            break

        if MODE == "ramp":
            dt = max(0.005, min(0.1, loop_ms / 1000.0))
            if ramp_phase in (0, 2):
                sign = 1.0 if ramp_phase == 0 else -1.0
                label = "RMP+" if ramp_phase == 0 else "RMP-"
                if ramp_start_pos is None and pos_cm is not None:
                    ramp_start_pos = pos_cm
                theta += sign * RAMP_DPS * dt
                # None, not 0.0: a ball that was never detected must not be
                # reported as a ball that did not move -- both print "+0.00 cm"
                # and one of them means the measurement is worthless
                moved = (None if (pos_cm is None or ramp_start_pos is None)
                         else pos_cm - ramp_start_pos)
                if moved is not None and abs(moved) >= BREAK_CM:
                    results.append((label, theta, moved, "BREAK"))
                    ramp_phase += 1
                elif abs(theta) >= MAX_DEG:
                    results.append((label, theta, moved,
                                    "ceiling, no break" if moved is not None
                                    else "ceiling, BALL NEVER SEEN"))
                    ramp_phase += 1
            elif ramp_phase in (1, 3):
                label = "REST"
                theta = 0.0
                if ramp_wait_until is None:
                    ramp_wait_until = now_ms + 2500
                elif now_ms >= ramp_wait_until:
                    ramp_wait_until = None
                    ramp_start_pos = None
                    if ramp_phase == 1:
                        ramp_phase = 2
                    else:
                        aborted = "ramp complete"
            else:
                aborted = "ramp complete"
            motor.move_abs_deg(theta, RPM, ACC, now_ms)
            for ev in motor.poll():
                if ev == "e2":
                    aborted = "motor E2 at {:+.1f} deg".format(theta)
            if aborted:
                break
            print("T,{},{},{},{:.2f},{},{:.2f},{:.2f},{:.2f},{:.3f},{:.1f},"
                  "{},{},{},{}".format(
                      now_ms - t0, label, loop_ms, 0.0,
                      "nan" if pos_cm is None else "{:.3f}".format(pos_cm),
                      v_cms, 0.0, 0.0, theta, 0.0, 0,
                      motor.tx_count, motor.e2_count, motor.ee_count))
            img.draw_string(6, 4, "RAMP {} theta{:+.1f} pos{}".format(
                label, theta,
                "?" if pos_cm is None else "{:+.2f}".format(pos_cm)),
                image.COLOR_WHITE, scale=1.4, thickness=2)
            img.draw_string(6, FRAME_HEIGHT - 28, "touch anywhere to abort",
                            image.COLOR_RED, scale=1.2, thickness=2)
            disp.show(img)
            continue

        if step_idx < len(PLAN):
            label, deg, dwell = PLAN[step_idx]
            if step_since is None:
                step_since = now_ms
                step_start_pos = pos_cm
                theta = deg
                motor.move_abs_deg(theta, RPM, ACC, now_ms, force=True)
            moved = (None if (pos_cm is None or step_start_pos is None)
                     else pos_cm - step_start_pos)
            done = now_ms - step_since >= dwell
            broke = (moved is not None and abs(moved) >= BREAK_CM
                     and not label.startswith("REST"))
            if done or broke:
                results.append((label, deg, moved, "BREAK" if broke else "hold"))
                step_since = None
                step_idx += 1
                if broke:
                    # the ball is rolling; anything past this angle only tells us
                    # about the far end stop, so drop to the rest step
                    while (step_idx < len(PLAN)
                           and not PLAN[step_idx][0].startswith("REST")):
                        step_idx += 1
        else:
            theta = 0.0
            motor.move_abs_deg(0.0, RPM, ACC, now_ms)
            if now_ms - t0 > 1000:
                aborted = "plan complete"
                break

        motor.move_abs_deg(theta, RPM, ACC, now_ms)
        for ev in motor.poll():
            if ev == "e2":
                aborted = "motor E2 (stall/protection) at {:+.1f} deg".format(theta)
        if aborted:
            break

        label = PLAN[step_idx][0] if step_idx < len(PLAN) else "END"
        print("T,{},{},{},{:.2f},{},{:.2f},{:.2f},{:.2f},{:.3f},{:.1f},"
              "{},{},{},{}".format(
                  now_ms - t0, label, loop_ms, 0.0,
                  "nan" if pos_cm is None else "{:.3f}".format(pos_cm),
                  v_cms, 0.0, 0.0, theta, 0.0, 0,
                  motor.tx_count, motor.e2_count, motor.ee_count))

        img.draw_string(6, 4, "PROBE {} theta{:+.1f} pos{}".format(
            label, theta,
            "?" if pos_cm is None else "{:+.2f}".format(pos_cm)),
            image.COLOR_WHITE, scale=1.4, thickness=2)
        img.draw_string(6, FRAME_HEIGHT - 28, "touch anywhere to abort",
                        image.COLOR_RED, scale=1.2, thickness=2)
        disp.show(img)
finally:
    print("probe end:", aborted or "app exit")
    for label, deg, moved, why in results:
        print("STEP {:<6} theta {:+6.1f} deg -> ball moved {} cm  {}".format(
            label, deg, "n/a" if moved is None else "{:+.2f}".format(moved),
            why))
    try:
        motor.stop_now()
        motor.reached_flag = False
        motor.move_abs_deg(0.0, RPM, 200, force=True)
        t_exit = time.ticks_ms()
        while time.ticks_ms() - t_exit < 2000:
            motor.poll()
            if motor.reached_flag:
                break
            time.sleep_ms(20)
        motor.stop_now()
    except Exception as e:
        print("probe shutdown error:", e)
