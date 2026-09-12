"""Automatic clean-frame dataset capture for MaixCAM2.

This is a standalone foreground application. It does not import or initialise
object detection, motors, UART, PID, linkage control, or contest tasks.
"""

import os

from maix import app, camera, display, image, time, touchscreen


CONFIG = {
    # Matches the default capture resolution used by D:/CODE/maixvision/train.
    "frame_width": 640,
    "frame_height": 480,
    "fps": 30,
    "buff_num": 3,
    "capture_interval_ms": 3000,
    "light_settle_ms": 500,
    # Matches D:/CODE/maixvision/train.
    "save_dir": "/root/data/image",
    "project_prefix": "project_",
    "file_prefix": "img_",
    "jpeg_quality": 95,
}


_CJK_FONT_NAME = "capture_cjk"
_CJK_FONT_CANDIDATES = (
    "/maixapp/share/font/sourcehansans/SourceHanSansCN-Regular.otf",
    "/maixapp/share/font/SourceHanSansCN-Regular.otf",
    "/maixapp/share/font/unifont.otf",
    "/usr/share/fonts/SourceHanSansCN-Regular.otf",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
)


def load_cjk_font(size=22):
    for path in _CJK_FONT_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            image.load_font(_CJK_FONT_NAME, path, size=size)
            print("中文字体:", path)
            return _CJK_FONT_NAME
        except Exception as exc:
            print("中文字体加载失败:", path, exc)
    print("未找到可用中文字体")
    return None


def text_size(text, scale, thickness, font=None):
    if font:
        try:
            return image.string_size(
                text, scale=scale, thickness=thickness, font=font)
        except Exception:
            pass
    return image.string_size(text, scale=scale, thickness=thickness)


def draw_text(frame, x, y, text, color, scale, thickness, font=None):
    if font:
        try:
            frame.draw_string(
                x, y, text, color, scale=scale,
                thickness=thickness, font=font)
            return
        except Exception:
            pass
    frame.draw_string(
        x, y, text, color, scale=scale, thickness=thickness)


class BoardLight:
    """Self-contained board illumination control."""

    def __init__(self, on_at_start=False):
        self.available = False
        self.is_on = False
        self._gpio = None
        self.reason = ""
        try:
            from maix import err, gpio, sys
            try:
                from maix.peripheral import pinmap
            except ImportError:
                from maix import pinmap

            if sys.device_id() == "maixcam2":
                pin, func = "B25", "GPIOB25"
            else:
                pin, func = "B3", "GPIOB3"
            err.check_raise(pinmap.set_pin_function(pin, func),
                            "board light pin")
            self._gpio = gpio.GPIO(func, gpio.Mode.OUT)
            self.available = True
            self.set(on_at_start)
            if self.available:
                print("board light: {} ({})".format(pin, func))
        except Exception as exc:
            self.reason = str(exc)
            print("board light unavailable:", exc)

    def set(self, on):
        if not self.available:
            return False
        try:
            self._gpio.value(1 if on else 0)
        except Exception as exc:
            self.available = False
            self.reason = str(exc)
            print("board light write failed:", exc)
            return False
        self.is_on = bool(on)
        return True

    def toggle(self):
        self.set(not self.is_on)
        return self.is_on

    def off(self):
        self.set(False)


def ensure_dir(path):
    try:
        os.makedirs(path, exist_ok=True)
    except TypeError:
        if not os.path.exists(path):
            os.makedirs(path)


def find_capture_projects(cfg):
    ensure_dir(cfg["save_dir"])
    prefix = cfg["project_prefix"]
    projects = []
    try:
        names = os.listdir(cfg["save_dir"])
    except Exception:
        names = []

    for name in names:
        if not name.startswith(prefix):
            continue
        project_dir = os.path.join(cfg["save_dir"], name)
        try:
            project_number = int(name[len(prefix):])
        except ValueError:
            continue
        try:
            if not os.path.isdir(project_dir):
                continue
        except Exception:
            continue
        projects.append((project_number, name, project_dir, False))

    projects.sort(key=lambda item: item[0])
    next_project = projects[-1][0] + 1 if projects else 1
    while True:
        project_name = "%s%04d" % (prefix, next_project)
        project_dir = os.path.join(cfg["save_dir"], project_name)
        if not os.path.exists(project_dir):
            projects.append((next_project, project_name, project_dir, True))
            return projects
        next_project += 1


def find_next_image_index(project_dir, cfg):
    next_index = 1
    prefix = cfg["file_prefix"]
    try:
        names = os.listdir(project_dir)
    except Exception:
        return next_index

    for name in names:
        if not name.startswith(prefix) or not name.lower().endswith(".jpg"):
            continue
        number_text = name[len(prefix):].split("_", 1)[0]
        try:
            next_index = max(next_index, int(number_text) + 1)
        except ValueError:
            pass
    return next_index


def load_project_choice(state, cfg):
    choice = state["project_choices"][state["project_choice_index"]]
    _number, project_name, project_dir, is_new = choice
    state["project_name"] = project_name
    state["project_dir"] = project_dir
    state["project_is_new"] = is_new
    state["next_index"] = (
        1 if is_new else find_next_image_index(project_dir, cfg))


def refresh_project_choices(state, cfg, preferred_dir=None):
    state["project_choices"] = find_capture_projects(cfg)
    selected_index = len(state["project_choices"]) - 1
    if preferred_dir is not None:
        for index, choice in enumerate(state["project_choices"]):
            if choice[2] == preferred_dir:
                selected_index = index
                break
    state["project_choice_index"] = selected_index
    load_project_choice(state, cfg)


def change_project(state, cfg, direction):
    if state["collecting"]:
        state["message"] = "请先停止采集"
        state["message_color"] = image.COLOR_YELLOW
        return
    count = len(state["project_choices"])
    state["project_choice_index"] = (
        state["project_choice_index"] + direction) % count
    load_project_choice(state, cfg)
    state["saved_count"] = 0
    state["message"] = (
        "新建工程" if state["project_is_new"] else "继续已有工程")
    state["message_color"] = image.COLOR_WHITE
    print("selected: {} next image: {:06d}".format(
        state["project_name"], state["next_index"]))


def start_selected_project(state, cfg, now_ms):
    ensure_dir(state["project_dir"])
    state["project_is_new"] = False
    state["next_index"] = find_next_image_index(state["project_dir"], cfg)
    state["saved_count"] = 0
    state["collecting"] = True
    state["next_capture_ms"] = now_ms + cfg["capture_interval_ms"]
    state["message"] = "开始采集 编号:%06d" % state["next_index"]
    state["message_color"] = image.COLOR_GREEN
    print("capture started: {} at {:06d}".format(
        state["project_dir"], state["next_index"]))


def save_raw_frame(frame, state, cfg):
    light_tag = "light_on" if state["light_on"] else "light_off"
    while True:
        filename = "%s%06d_%s.jpg" % (
            cfg["file_prefix"], state["next_index"], light_tag)
        path = os.path.join(state["project_dir"], filename)
        if not os.path.exists(path):
            break
        state["next_index"] += 1
    try:
        # Save before UI drawing so the training image stays clean.
        frame.save(path, quality=cfg["jpeg_quality"])
        state["next_index"] += 1
        state["saved_count"] += 1
        state["message"] = "已保存 " + filename
        state["message_color"] = image.COLOR_GREEN
        print("saved:", path)
        return True
    except Exception as exc:
        state["message"] = "保存失败 " + str(exc)[:24]
        state["message_color"] = image.COLOR_RED
        state["collecting"] = False
        print("save error:", exc)
        return False


class CaptureUI:
    BUTTONS = (
        ("prev", "上一个"),
        ("next", "下一个"),
        ("start", "开始"),
        ("stop", "停止"),
        ("light", "补光"),
    )
    TEXT_SCALE = 1.0
    BUTTON_SCALE = 1.15

    def __init__(self, frame_w, frame_h, disp, touch, cfg, font=None):
        self.w = frame_w
        self.h = frame_h
        self.disp = disp
        self.touch = touch
        self.cfg = cfg
        self.font = font
        self.top_h = 30
        self.btn_h = max(48, int(frame_h * 0.30))
        self.btn_y = frame_h - self.btn_h
        self.slop = 16
        self._pressed = False
        self._press_pt = None
        self._press_action = None

        self.c_bar = image.Color.from_rgb(0, 0, 0)
        self.c_btn = image.Color.from_rgb(40, 40, 40)
        self.c_btn_disabled = image.Color.from_rgb(20, 20, 20)
        self.c_btn_hot = image.Color.from_rgb(0, 90, 160)
        self.c_start = image.Color.from_rgb(0, 105, 70)
        self.c_stop = image.Color.from_rgb(135, 35, 35)
        self.c_light = image.Color.from_rgb(150, 110, 0)
        self.c_txt = image.COLOR_WHITE
        self.c_ok = image.COLOR_GREEN

        self.rects = self._make_row()
        self._button_scales = {}
        for action, label, _x, _y, width, _height in self.rects:
            self._button_scales[action] = self._fit_scale(
                label, width, self.BUTTON_SCALE)

    def _make_row(self):
        gap = 4
        count = len(self.BUTTONS)
        width = (self.w - gap * (count + 1)) // count
        return tuple(
            (action, label, gap + i * (width + gap),
             self.btn_y, width, self.btn_h)
            for i, (action, label) in enumerate(self.BUTTONS)
        )

    def _fit_scale(self, label, width, scale):
        while scale > 0.7:
            size = text_size(
                label, scale=scale, thickness=1, font=self.font)
            if size.width() <= width - 12:
                return scale
            scale = round(scale - 0.1, 2)
        return scale

    def _map_touch(self, x, y):
        return image.resize_map_pos_reverse(
            self.w, self.h,
            self.disp.width(), self.disp.height(),
            image.Fit.FIT_CONTAIN, int(x), int(y))

    def _pick(self, point):
        if point is None or point[1] < self.btn_y - self.slop:
            return None
        best_action = None
        best_distance = None
        for action, _label, x, _y, width, _height in self.rects:
            distance = abs(point[0] - (x + width * 0.5))
            if best_distance is None or distance < best_distance:
                best_action = action
                best_distance = distance
        return best_action

    def read_action(self):
        try:
            x, y, pressed = self.touch.read()
        except Exception:
            return None
        if pressed:
            self._pressed = True
            self._press_pt = self._map_touch(x, y)
            self._press_action = self._pick(self._press_pt)
            return None
        if not self._pressed:
            return None
        self._pressed = False
        action = self._pick(self._press_pt)
        self._press_pt = None
        self._press_action = None
        return action

    def _draw_buttons(self, frame, state):
        for action, label, x, y, width, height in self.rects:
            if self._pressed and self._press_action == action:
                fill = self.c_btn_hot
            elif action == "start" and state["collecting"]:
                fill = self.c_start
            elif action == "stop" and state["collecting"]:
                fill = self.c_stop
            elif action == "light" and state["light_on"]:
                fill = self.c_light
            elif action in ("prev", "next") and state["collecting"]:
                fill = self.c_btn_disabled
            else:
                fill = self.c_btn
            frame.draw_rect(x, y, width, height, fill, -1)
            frame.draw_rect(x, y, width, height, self.c_txt, 1)
            scale = self._button_scales[action]
            size = text_size(
                label, scale=scale, thickness=1, font=self.font)
            draw_text(
                frame,
                x + (width - size.width()) // 2,
                y + (height - size.height()) // 2,
                label, self.c_txt, scale, 1, self.font)

    def draw(self, frame, state, now_ms, fps):
        frame.draw_rect(0, 0, self.w, self.top_h, self.c_bar, -1)
        mode = "采集中" if state["collecting"] else "就绪"
        left = "采集:{} 已拍:{:04d}".format(mode, state["saved_count"])
        right = "{}x{} 帧率:{}".format(self.w, self.h, fps)
        scale = self.TEXT_SCALE
        draw_text(frame, 6, 4, left, self.c_txt, scale, 1, self.font)
        size = text_size(right, scale, 1, self.font)
        draw_text(frame, self.w - size.width() - 6, 4, right,
                  self.c_ok, scale, 1, self.font)

        project_mode = "新建" if state["project_is_new"] else "续采"
        project_text = "工程:{} {} 下一张:{:06d}".format(
            state["project_name"], project_mode, state["next_index"])
        draw_text(frame, 6, self.top_h + 5, project_text,
                  self.c_txt, 1.0, 1, self.font)
        if state["collecting"]:
            exposure_wait = max(
                0, self.cfg["light_settle_ms"]
                - (now_ms - state["last_light_change_ms"]))
            if exposure_wait:
                status = "等待曝光:{:.1f}秒".format(exposure_wait / 1000.0)
            else:
                remain = max(0, state["next_capture_ms"] - now_ms)
                status = "距下次:{:.1f}秒".format(remain / 1000.0)
            draw_text(frame, 6, self.top_h + 31, status,
                      self.c_ok, 1.0, 1, self.font)
        draw_text(frame, 6, self.btn_y - 25, state["message"],
                  state["message_color"], 0.9, 1, self.font)
        self._draw_buttons(frame, state)


def open_camera(cfg):
    cam = camera.Camera(
        cfg["frame_width"], cfg["frame_height"],
        image.Format.FMT_RGB888,
        fps=cfg["fps"], buff_num=cfg["buff_num"])
    try:
        cam.skip_frames(10)
    except Exception:
        pass
    return cam


def main():
    cfg = CONFIG
    project_choices = find_capture_projects(cfg)
    project_choice_index = len(project_choices) - 1
    _number, project_name, project_dir, project_is_new = (
        project_choices[project_choice_index])
    cam = open_camera(cfg)
    disp = display.Display()
    touch = touchscreen.TouchScreen()
    light = BoardLight(on_at_start=False)
    cjk_font = load_cjk_font()
    ui = CaptureUI(
        cfg["frame_width"], cfg["frame_height"], disp, touch, cfg,
        font=cjk_font)

    now_ms = time.ticks_ms()
    state = {
        "project_choices": project_choices,
        "project_choice_index": project_choice_index,
        "project_name": project_name,
        "project_dir": project_dir,
        "project_is_new": project_is_new,
        "next_index": 1,
        "saved_count": 0,
        "collecting": False,
        "next_capture_ms": now_ms + cfg["capture_interval_ms"],
        "light_on": light.is_on,
        "last_light_change_ms": now_ms - cfg["light_settle_ms"],
        "message": "就绪",
        "message_color": image.COLOR_WHITE,
    }
    load_project_choice(state, cfg)
    last_ms = now_ms
    print("dataset folders found:", len(project_choices) - 1)
    print("selected:", project_dir)
    print("resolution: {}x{}, interval: {} ms".format(
        cfg["frame_width"], cfg["frame_height"],
        cfg["capture_interval_ms"]))

    try:
        while not app.need_exit():
            frame = cam.read()
            if frame is None:
                continue
            now_ms = time.ticks_ms()
            loop_ms = max(1, now_ms - last_ms)
            last_ms = now_ms

            action = ui.read_action()
            if action == "start" and not state["collecting"]:
                start_selected_project(state, cfg, now_ms)
            elif action == "stop":
                state["collecting"] = False
                state["message"] = "已停止"
                state["message_color"] = image.COLOR_WHITE
                print("capture stopped")
                refresh_project_choices(
                    state, cfg, preferred_dir=state["project_dir"])
            elif action == "prev":
                change_project(state, cfg, -1)
            elif action == "next":
                change_project(state, cfg, 1)
            elif action == "light":
                if light.available:
                    light.toggle()
                    state["light_on"] = light.is_on
                    state["last_light_change_ms"] = now_ms
                    state["message"] = "补光灯:" + (
                        "开" if light.is_on else "关")
                    state["message_color"] = image.COLOR_GREEN
                else:
                    state["message"] = "补光灯不可用"
                    state["message_color"] = image.COLOR_RED

            due = (state["collecting"]
                   and now_ms >= state["next_capture_ms"])
            light_stable = (
                now_ms - state["last_light_change_ms"]
                >= cfg["light_settle_ms"])
            if due and light_stable:
                if save_raw_frame(frame, state, cfg):
                    state["next_capture_ms"] = (
                        now_ms + cfg["capture_interval_ms"])

            loop_fps = int(1000 // loop_ms)
            try:
                camera_fps = int(cam.fps())
            except Exception:
                camera_fps = 0
            fps = min(camera_fps, loop_fps) if camera_fps > 0 else loop_fps
            ui.draw(frame, state, now_ms, fps)
            disp.show(frame)
    finally:
        light.off()
        try:
            cam.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
