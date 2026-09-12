"""Motor-only diagnostic: does the shaft actually turn when we command it?

No camera, no ball, no control loop.  The ramp probe showed the ball sitting at
+4.55 cm +/-0.01 cm (100 % detection) while theta was commanded through
+45 -> 0 -> -45 deg with zero UART errors, yet an earlier abrupt staircase moved
the ball 2.66 cm.  Those two facts cannot both be true of a healthy drivetrain,
so this asks the driver itself for its encoder position instead of inferring
motion from the ball.

Phase 1 sweeps the Emm/ZDT read function codes and prints the raw reply bytes.
This rig answers 0x36 with EE, so which codes DO work is unknown and worth
knowing: a position readback turns every later tuning question from a guess into
a measurement.

Phase 2 commands real moves and reads the position back before and after.  The
three outcomes are distinguishable:
  * readback follows the command  -> shaft turns; the beam/linkage is slipping
  * readback stays put            -> driver is not stepping (disabled, no power
                                     to the motor phases, or blocked)
  * readback lags by a lot        -> mechanically blocked, losing steps

Run via .maixpy/tools/run_stub.py with ENTRY = "motor_diag.py".
Env: DIAG_DEG (test angle, default 20), DIAG_RPM (default 60), DIAG_ACC (0).
"""

import os

from maix import app, time, uart, err

try:
    from maix.peripheral import pinmap
except ImportError:
    from maix import pinmap

from zdt_motor import ZdtEmmMotor, PULSES_PER_REV

DEG = float(os.environ.get("DIAG_DEG", "20"))
RPM = int(os.environ.get("DIAG_RPM", "60"))
ACC = int(os.environ.get("DIAG_ACC", "0"))
ADDR = int(os.environ.get("DIAG_ADDR", "1"))
BAUD = int(os.environ.get("DIAG_BAUD", "115200"))
SWEEP = os.environ.get("DIAG_SWEEP", "1") == "1"

err.check_raise(pinmap.set_pin_function("B0", "UART2_TX"), "diag: UART TX pin")
err.check_raise(pinmap.set_pin_function("B1", "UART2_RX"), "diag: UART RX pin")
serial = uart.UART("/dev/ttyS2", BAUD)
motor = ZdtEmmMotor(serial, addr=ADDR, max_abs_deg=90.0)


def drain(wait_ms=200):
    """Collect everything the driver sends back, as hex. Raw on purpose.

    poll() throws away frames it cannot parse, which is exactly the information
    needed here -- an EE reply and silence look identical after parsing.
    """
    out = bytearray()
    t_end = time.ticks_ms() + wait_ms
    while time.ticks_ms() < t_end:
        try:
            n = int(serial.available(0))
        except Exception:
            n = 0
        if n > 0:
            data = serial.read(n, 0)
            if data:
                out.extend(bytes(data))
        else:
            time.sleep_ms(5)
    return " ".join("{:02X}".format(b) for b in out) if out else "(silent)"


def ask(label, payload, wait_ms=200):
    drain(20)  # clear anything left from the previous exchange
    motor._send(payload)
    reply = drain(wait_ms)
    print("  {:<26} tx {:<14} rx {}".format(
        label, " ".join("{:02X}".format(b) for b in payload), reply))
    return reply


# ---------- phase 0: is anything listening at all? ----------

# A wrong baud rate or a driver whose address is not 1 looks EXACTLY like a dead
# bus: frames go out, nothing comes back, the beam never moves, and e2/ee stay 0
# because no reply is ever parsed.  That is a two-minute check and it is the
# difference between "rewire the rig" and "the driver is set to 38400".
# Only read codes are sent here: even misinterpreted at the wrong baud they
# cannot command motion.
SWEEP_BAUDS = (115200, 9600, 19200, 38400, 57600)
SWEEP_ADDRS = (0x01, 0x02, 0x03)  # 0x00 is broadcast: by design it never replies


def sweep_link():
    """Return (baud, addr) of the first combination that answers, else None."""
    global serial, motor
    found = None
    for baud in SWEEP_BAUDS:
        serial = uart.UART("/dev/ttyS2", baud)
        motor = ZdtEmmMotor(serial, addr=ADDR, max_abs_deg=90.0)
        hits = []
        for addr in SWEEP_ADDRS:
            motor.addr = addr
            drain(20)
            motor._send([0x3A])            # read status flags: harmless
            reply = drain(120)
            if reply != "(silent)":
                hits.append((addr, reply))
        print("  baud {:>6}: {}".format(
            baud, "; ".join("addr {:02X} -> {}".format(a, r) for a, r in hits)
            if hits else "silent on all addresses"))
        if hits and found is None:
            found = (baud, hits[0][0])
    return found


if SWEEP:
    print("=== phase 0: baud/address sweep ===")
    link = sweep_link()
    if link:
        print("LINK FOUND at baud {} addr {:02X} -- set these and retune"
              .format(link[0], link[1]))
        serial = uart.UART("/dev/ttyS2", link[0])
        motor = ZdtEmmMotor(serial, addr=link[1], max_abs_deg=90.0)
    else:
        print("LINK DEAD: no reply at any baud or address. The driver is not "
              "talking -- power, wiring or driver state, not software.")
        serial = uart.UART("/dev/ttyS2", BAUD)
        motor = ZdtEmmMotor(serial, addr=ADDR, max_abs_deg=90.0)

# ---------- phase 1: which read codes does this firmware accept? ----------

# Emm V5 read codes.  Read-only by definition, so sweeping them cannot move the
# motor or write a parameter -- the worst case is another EE.
READ_CODES = (
    (0x30, "encoder raw"),
    (0x31, "input pulse count"),
    (0x32, "pulse count b"),
    (0x33, "target position"),
    (0x34, "realtime target"),
    (0x35, "realtime speed"),
    (0x36, "realtime position"),
    (0x37, "position error"),
    (0x3A, "status flags"),
    (0x3B, "homing status"),
)

print("=== phase 1: read-code sweep (EE = firmware rejects the frame) ===")
alive = []
for code, name in READ_CODES:
    reply = ask("{:02X} {}".format(code, name), [code])
    if reply != "(silent)" and " EE " not in " " + reply + " ":
        alive.append((code, name))
print("read codes answering with data:",
      ", ".join("{:02X}".format(c) for c, _ in alive) or "NONE")


def read_pos_deg():
    """Best-effort shaft angle from whichever readback this firmware supports."""
    for code in (0x36, 0x30, 0x31):
        drain(20)
        motor._send([code])
        t_end = time.ticks_ms() + 200
        buf = bytearray()
        while time.ticks_ms() < t_end:
            try:
                n = int(serial.available(0))
            except Exception:
                n = 0
            if n > 0:
                data = serial.read(n, 0)
                if data:
                    buf.extend(bytes(data))
            else:
                time.sleep_ms(5)
        if len(buf) >= 8 and buf[0] == ADDR and buf[1] == code:
            sign = -1.0 if buf[2] == 0x01 else 1.0
            raw = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6]
            if code == 0x36:
                return sign * raw * 360.0 / 65536.0, "36"
            return sign * raw * 360.0 / PULSES_PER_REV, "{:02X}".format(code)
    return None, "-"


# ---------- phase 2: command a move, read the shaft back ----------

print("\n=== phase 2: commanded move vs readback ===")
ask("enable", [0xF3, 0xAB, 0x01, 0x00])
time.sleep_ms(100)
ask("status after enable", [0x3A])
motor.set_zero()
time.sleep_ms(100)

plan = ((+DEG, RPM), (0.0, RPM), (-DEG, RPM), (0.0, RPM),
        (+DEG, 200), (0.0, 200))
for target, rpm in plan:
    if app.need_exit():
        break
    before, src = read_pos_deg()
    motor.reached_flag = False
    motor.move_abs_deg(target, rpm, ACC, force=True)
    # generous: a 20 deg move at 60 rpm (360 deg/s) needs ~60 ms, so 1.5 s of
    # silence means the driver never started rather than "still travelling"
    t_end = time.ticks_ms() + 1500
    events = []
    while time.ticks_ms() < t_end:
        events.extend(motor.poll())
        if motor.reached_flag:
            break
        time.sleep_ms(20)
    after, _src = read_pos_deg()
    print("  cmd {:+7.2f} deg @ {:>3} rpm | readback({}) {} -> {} | moved {} "
          "| reached {} | ev {}".format(
              target, rpm, src,
              "n/a" if before is None else "{:+8.2f}".format(before),
              "n/a" if after is None else "{:+8.2f}".format(after),
              "n/a" if (before is None or after is None)
              else "{:+7.2f} deg".format(after - before),
              1 if motor.reached_flag else 0,
              ",".join(str(e) for e in events) or "-"))

ask("status at end", [0x3A])
ask("position error", [0x37])
print("\nmotor frames: tx {} rx {} e2 {} ee {}".format(
    motor.tx_count, motor.rx_count, motor.e2_count, motor.ee_count))

motor.move_abs_deg(0.0, RPM, ACC, force=True)
time.sleep_ms(600)
motor.stop_now()
print("diag done: motor stopped at zero")
