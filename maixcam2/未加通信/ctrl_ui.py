"""Run/Tune touch UI for the ball-and-beam controller.

Contract (maixcam-master template): run view is the default, tune view is
table-driven, one shared map_touch + touch_hit path for every button, taps fire
on release, config persists as dotted-path JSON.

The tune view holds several independent parameter groups (MAIN and TASK3),
cycled with PAGE.  Each group is a separate config dict with its own save file,
so tuning one mode can never move the other mode's gains.
The camera frame is wide and short (~640x180), so both views use a passive
top status bar and one row of large buttons along the bottom edge.
"""

import json
import os

from maix import image


# MaixCAM ships a few CJK-capable fonts; paths differ between images, so try
# them in order.  These all contain full ASCII too, so making one the default
# does not break the English text elsewhere.
_CJK_FONT_CANDIDATES = (
    "/maixapp/share/font/sourcehansans/SourceHanSansCN-Regular.otf",
    "/maixapp/share/font/SourceHanSansCN-Regular.otf",
    "/maixapp/share/font/unifont.otf",
    "/usr/share/fonts/SourceHanSansCN-Regular.otf",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
)


CJK_FONT_NAME = "cjk"
_cjk_loaded = False


def init_cjk_font(size=32):
    """Load a Chinese-capable font under the name CJK_FONT_NAME.

    Returns True on success.  The caller uses the result to pick Chinese or
    English parameter labels: with no CJK font loaded MaixPy draws Chinese as
    empty boxes, which is worse than English, so falling back keeps the tune page
    readable either way.

    The font is deliberately NOT made the global default -- that would change the
    glyph widths of the run view's status bar, which is already nearly as wide as
    the frame.  Only the tune page asks for it, per draw call.
    """
    global _cjk_loaded
    if _cjk_loaded:
        return True
    for path in _CJK_FONT_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            image.load_font(CJK_FONT_NAME, path, size=size)
            _cjk_loaded = True
            print("tune page font:", path)
            return True
        except Exception as exc:
            print("font load failed:", path, exc)
    print("no CJK font found, tune page stays in English")
    return False


def _text_size(text, scale, thickness, font=None):
    """image.string_size with the optional font arg, tolerating older MaixPy."""
    if font:
        try:
            return image.string_size(text, scale=scale, thickness=thickness,
                                     font=font)
        except Exception:
            pass
    return image.string_size(text, scale=scale, thickness=thickness)


def _draw_text(img, x, y, text, color, scale, thickness, font=None):
    """img.draw_string with the optional font arg, tolerating older MaixPy."""
    if font:
        try:
            img.draw_string(x, y, text, color, scale=scale,
                            thickness=thickness, font=font)
            return
        except Exception:
            pass
    img.draw_string(x, y, text, color, scale=scale, thickness=thickness)


def get_param_value(cfg, path):
    value = cfg
    for key in path:
        value = value[key]
    return value


def set_param_value(cfg, path, value):
    target = cfg
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value


def save_config(cfg, tunable_params, file_path, version=None):
    data = {}
    if version is not None:
        data["_ver"] = version
    for label, path, step, low, high in tunable_params:
        data[".".join(str(p) for p in path)] = get_param_value(cfg, path)
    with open(file_path, "w") as f:
        json.dump(data, f)


def load_config(cfg, tunable_params, file_path, version=None):
    """Restore saved tunables.  A version mismatch discards the whole file.

    Gains carry meaning only relative to the travel calibration they were tuned
    against, so silently restoring a file written for a different set of defaults
    can hand the rig a dangerously aggressive configuration -- exactly what the
    saved file is supposed to protect against.  Bump the version whenever the
    defaults change scale.
    """
    if not os.path.exists(file_path):
        return False
    try:
        with open(file_path) as f:
            data = json.load(f)
    except Exception:
        return False
    if version is not None and data.get("_ver") != version:
        print("ignoring stale config (want _ver", version,
              "got", data.get("_ver"), ") ->", file_path)
        return False
    for label, path, step, low, high in tunable_params:
        key = ".".join(str(p) for p in path)
        if key in data:
            set_param_value(cfg, path, data[key])
    return True


class CtrlUI:
    RUN_BUTTONS = ("HOLD", "TASK3", "STOP", "ZERO", "LIGHT", "TUNE")
    TUNE_BUTTONS = ("<", ">", "-", "+", "STEP", "PAGE", "SAVE", "BACK")
    STEP_MULTS = (1, 5, 10)

    # Upper bounds, not fixed sizes: each row is measured at build time and any
    # label too wide for its button is stepped down until it fits (_fit_scale).
    # The row tiles the full frame width, so overflow has nowhere to go.
    BTN_TEXT_SCALE = 1.8
    TUNE_BTN_TEXT_SCALE = 1.4
    BAR_TEXT_SCALE = 1.4
    # the task clock is the one number read from across the bench, so it gets
    # its own size class -- roughly twice the status bar
    TIME_TEXT_SCALE = 3.0

    def __init__(self, param_groups, frame_w, frame_h, disp, ts,
                 touch_slop=16):
        """param_groups: ((title, cfg_dict, tunable_params), ...).

        Each group is an independent config store with its own save file; the
        UI only edits values in place and reports "save", it never decides where
        anything is written.  PAGE cycles groups, and each group keeps its own
        cursor so switching back does not lose your place.
        """
        self.groups = tuple(param_groups)
        self.group_idx = 0
        self.sels = [0] * len(self.groups)
        self.w = frame_w
        self.h = frame_h
        self.disp = disp
        self.ts = ts
        self.slop = touch_slop

        self.mode = "run"
        self.step_idx = 0
        self.saved_msg_ms = 0

        self._pressed = False
        self._press_pt = None
        self._press_label = None

        # buttons drawn as latched-on (e.g. LIGHT while the LED is lit), so the
        # state is visible without spending a line of the crowded status bar
        self.on_labels = set()

        self.top_h = 30
        # ~30% of the frame height, never below 48 px
        self.btn_h = max(48, int(frame_h * 0.30))
        self.btn_y = frame_h - self.btn_h
        self._btn_scale = {}
        self.run_rects = self._make_row(self.RUN_BUTTONS, self.BTN_TEXT_SCALE)
        self.tune_rects = self._make_row(self.TUNE_BUTTONS,
                                         self.TUNE_BTN_TEXT_SCALE)

        self.c_bar = image.Color.from_rgb(0, 0, 0)
        self.c_btn = image.Color.from_rgb(40, 40, 40)
        self.c_btn_hot = image.Color.from_rgb(0, 90, 160)
        self.c_btn_on = image.Color.from_rgb(150, 110, 0)
        self.c_txt = image.COLOR_WHITE
        self.c_warn = image.COLOR_RED
        self.c_ok = image.COLOR_GREEN
        self.c_target = image.Color.from_rgb(255, 220, 0)
        self.c_dark = image.COLOR_BLACK
        self.c_plate = image.COLOR_WHITE
        # cyan is deliberately unused elsewhere: the axis is yellow, the mode
        # text green, faults red.  The clock must not be mistaken for any of
        # them at a glance.
        self.c_time = image.Color.from_rgb(0, 255, 255)

    @property
    def group_title(self):
        return self.groups[self.group_idx][0]

    @property
    def cfg(self):
        return self.groups[self.group_idx][1]

    @property
    def params(self):
        return self.groups[self.group_idx][2]

    @property
    def sel(self):
        return self.sels[self.group_idx]

    @sel.setter
    def sel(self, value):
        self.sels[self.group_idx] = value

    @property
    def tune_font(self):
        """CJK font name if one loaded, else None (falls back to the default).

        Read live rather than cached in __init__ so the UI does not depend on
        whether init_cjk_font ran before or after construction.
        """
        return CJK_FONT_NAME if _cjk_loaded else None

    def _fit_scale(self, label, bw, scale):
        """Largest scale <= `scale` whose label still fits inside the button.

        Measured once at build time, not per frame.  Without this, adding one
        button to a row silently pushes the longest label past the button edge
        and there is no way to notice except on the rig.
        """
        while scale > 0.6:
            if image.string_size(label, scale=scale,
                                 thickness=2).width() <= bw - 8:
                return scale
            scale = round(scale - 0.1, 2)
        return scale

    def _make_row(self, labels, scale):
        n = len(labels)
        gap = 4
        bw = (self.w - gap * (n + 1)) // n
        rects = []
        for i, label in enumerate(labels):
            x = gap + i * (bw + gap)
            rects.append((label, x, self.btn_y, bw, self.btn_h))
            # keyed by width too: the same label in two rows of different
            # button widths needs two different scales
            self._btn_scale[(label, bw)] = self._fit_scale(label, bw, scale)
        return rects

    # ---------- touch ----------

    def _map_touch(self, x, y):
        # touch coords are in display space; map back onto the camera frame
        return image.resize_map_pos_reverse(
            self.w, self.h,
            self.disp.width(), self.disp.height(),
            image.Fit.FIT_CONTAIN, int(x), int(y))

    def _pick(self, pt, rects):
        """Nearest button in the bottom row, or None if the tap is above it.

        The row tiles the full width, so instead of per-rect hit boxes (whose
        slop would overlap and bias the leftmost button) we accept anything in
        the row band and take the closest centre.  Vertical slop is generous
        because the row sits on the bottom edge of a short frame.
        """
        if pt is None or pt[1] < self.btn_y - self.slop:
            return None
        best, best_d = None, None
        for label, x, _y, w, _h in rects:
            d = abs(pt[0] - (x + w * 0.5))
            if best_d is None or d < best_d:
                best, best_d = label, d
        return best

    def read_action(self):
        """Poll touchscreen; return a tap action label on release, else None."""
        x, y, pressed = self.ts.read()
        action = None
        rects = self.run_rects if self.mode == "run" else self.tune_rects
        if pressed:
            self._pressed = True
            self._press_pt = self._map_touch(x, y)
            self._press_label = self._pick(self._press_pt, rects)
        elif self._pressed:
            self._pressed = False
            action = self._pick(self._press_pt, rects)
            self._press_pt = None
            self._press_label = None
        return self._apply(action)

    def _apply(self, action):
        """Handle tune-view edits internally; pass run actions to the caller."""
        if action is None:
            return None
        if self.mode == "run":
            if action == "TUNE":
                self.mode = "tune"
                return None
            return action.lower()  # hold / task3 / stop / zero

        if action == "BACK":
            self.mode = "run"
        elif action == "<":
            self.sel = (self.sel - 1) % len(self.params)
        elif action == ">":
            self.sel = (self.sel + 1) % len(self.params)
        elif action == "STEP":
            self.step_idx = (self.step_idx + 1) % len(self.STEP_MULTS)
        elif action == "PAGE":
            self.group_idx = (self.group_idx + 1) % len(self.groups)
        elif action in ("-", "+"):
            label, path, step, low, high = self.params[self.sel]
            step = step * self.STEP_MULTS[self.step_idx]
            value = get_param_value(self.cfg, path)
            value = value - step if action == "-" else value + step
            value = max(low, min(high, value))
            if isinstance(get_param_value(self.cfg, path), int) and step >= 1:
                value = int(round(value))
            set_param_value(self.cfg, path, round(value, 4))
        elif action == "SAVE":
            return "save"
        return None

    def toggle_view(self):
        self.mode = "tune" if self.mode == "run" else "run"

    # ---------- drawing ----------

    def _draw_buttons(self, img, rects):
        for label, x, y, w, h in rects:
            if self._pressed and label == self._press_label:
                fill = self.c_btn_hot
            elif label in self.on_labels:
                fill = self.c_btn_on
            else:
                fill = self.c_btn
            img.draw_rect(x, y, w, h, fill, -1)
            img.draw_rect(x, y, w, h, self.c_txt, 1)
            s = self._btn_scale.get((label, w), self.BTN_TEXT_SCALE)
            size = image.string_size(label, scale=s, thickness=2)
            img.draw_string(x + (w - size.width()) // 2,
                            y + (h - size.height()) // 2,
                            label, self.c_txt, scale=s, thickness=2)

    def draw_run(self, img, ctx):
        """ctx: dict with pos_cm, target_cm, motor_deg, omega_dps, v_ref_cms,
        mode, sub, fps, ball_valid, task_time_s, task_limit_s, task_prompt,
        task_prompt_ready, task_result, alarm, warn."""
        s = self.BAR_TEXT_SCALE
        img.draw_rect(0, 0, self.w, self.top_h, self.c_bar, -1)
        if ctx.get("ball_valid"):
            left = "P{:+5.2f} T{:+.1f} E{:+5.2f}cm".format(
                ctx.get("pos_cm", 0.0), ctx.get("target_cm", 0.0),
                ctx.get("target_cm", 0.0) - ctx.get("pos_cm", 0.0))
            color = self.c_txt
        else:
            left = "BALL LOST"
            color = self.c_warn
        img.draw_string(6, 4, left, color, scale=s, thickness=2)

        right = "{} {} M{:+.1f} FPS{}".format(
            ctx.get("mode", ""), ctx.get("sub", ""),
            ctx.get("motor_deg", 0.0), ctx.get("fps", 0))
        size = image.string_size(right, scale=s, thickness=2)
        img.draw_string(self.w - size.width() - 6, 4, right,
                        self.c_ok, scale=s, thickness=2)

        # The task clock, as the largest thing on screen.  It counts up live
        # during the run and freezes on the total at DONE -- a number that only
        # appears after the run is over is useless for judging "did we make it",
        # which is the one question being asked while the ball is moving.
        # Colour carries the verdict: cyan inside the limit, red past it.
        time_w = 0
        t_s = ctx.get("task_time_s")
        ts = self.TIME_TEXT_SCALE
        if t_s is not None:
            txt = "{:.2f}s".format(t_s)
            over = t_s > ctx.get("task_limit_s", 5.0)
            size = image.string_size(txt, scale=ts, thickness=3)
            img.draw_string(6, self.top_h + 2, txt,
                            self.c_warn if over else self.c_time,
                            scale=ts, thickness=3)
            time_w = size.width()
        elif ctx.get("task_prompt"):
            # same slot, same size: before the run it says what the next press
            # will do (ARMING -> wait, START -> press TASK3 again to go), after
            # it starts the clock takes the slot over.  Green means "your move".
            txt = ctx["task_prompt"]
            ready = ctx.get("task_prompt_ready")
            size = image.string_size(txt, scale=ts, thickness=3)
            img.draw_string(6, self.top_h + 2, txt,
                            self.c_ok if ready else self.c_target,
                            scale=ts, thickness=3)
            time_w = size.width()

        # the two scored errors sit beside the clock, not centred: centring
        # would put them under it and there is no room above the axis line
        if ctx.get("task_result"):
            img.draw_string(6 + time_w + 12, self.top_h + 14,
                            ctx["task_result"], self.c_txt,
                            scale=1.6, thickness=2)

        # faults go below the axis: the band above it now belongs to the clock
        if ctx.get("alarm"):
            img.draw_string(6, self.btn_y - 24, ctx["alarm"],
                            self.c_warn, scale=s, thickness=2)
        elif ctx.get("warn"):
            img.draw_string(6, self.btn_y - 24, ctx["warn"],
                            self.c_target, scale=s, thickness=2)

        # cascade internals: target ball speed and commanded beam rate.  Without
        # these two on screen the middle loop is untunable -- you cannot tell a
        # position loop asking for too much from a velocity loop tracking badly.
        cascade = "V*{:+.1f} W{:+.0f}".format(
            ctx.get("v_ref_cms", 0.0), ctx.get("omega_dps", 0.0))
        size = image.string_size(cascade, scale=1.2, thickness=2)
        img.draw_string(self.w - size.width() - 6, self.top_h + 8, cascade,
                        self.c_txt, scale=1.2, thickness=2)

        self._draw_buttons(img, self.run_rects)

    def draw_tune(self, img, now_ms):
        s = self.BAR_TEXT_SCALE
        img.draw_rect(0, 0, self.w, self.top_h, self.c_bar, -1)
        label, path, step, low, high = self.params[self.sel]
        value = get_param_value(self.cfg, path)
        # the group title is the only thing telling you WHICH config you are
        # about to change -- two pages with identically-named gains would
        # otherwise be indistinguishable
        head = "{} [{}/{}] step x{}".format(
            self.group_title, self.sel + 1, len(self.params),
            self.STEP_MULTS[self.step_idx])
        img.draw_string(6, 4, head, self.c_txt, scale=s, thickness=2)
        # black on a white plate, not the run view's yellow: this line lands on
        # the live camera frame, so without an opaque backing it is only readable
        # against whatever happens to be behind it
        body = "{} = {}".format(label, value)
        font = self.tune_font
        size = _text_size(body, 2.0, 2, font)
        bx = (self.w - size.width()) // 2
        by = self.top_h + 6
        pad = 8
        img.draw_rect(max(0, bx - pad), max(0, by - pad),
                      min(self.w, size.width() + pad * 2),
                      size.height() + pad * 2, self.c_plate, -1)
        _draw_text(img, bx, by, body, self.c_dark, 2.0, 2, font)
        if now_ms - self.saved_msg_ms < 1200:
            img.draw_string(6, self.top_h + 6, "SAVED",
                            self.c_ok, scale=s, thickness=2)
        self._draw_buttons(img, self.tune_rects)

    def mark_saved(self, now_ms):
        self.saved_msg_ms = now_ms

    def set_button_on(self, label, on):
        """Latch a button's highlight to some external state."""
        if on:
            self.on_labels.add(label)
        else:
            self.on_labels.discard(label)
