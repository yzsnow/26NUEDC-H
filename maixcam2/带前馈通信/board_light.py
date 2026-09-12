"""板载补光灯（照明 LED）开关。

MaixCAM 系列把照明 LED 挂在一个普通 GPIO 上，高电平点亮：
MaixCAM2 是 B25，MaixCAM-Pro 是 B3（初代 MaixCAM 没有这颗灯）。
引脚名 B25 是 MaixPy 的写法，对应原理图里的 GPIO1_A25。

**每一步失败都只打印一行然后把自己标成不可用**：补光灯是锦上添花，控制环才是
任务本体。为了点不亮一颗灯就让整个程序起不来（或者中途抛异常把摆杆撂在半路）
是本末倒置的，所以这里一个异常都不往外抛。

功耗提醒：这颗灯和电机共用同一块电池，而电机堵转拉低电池正是板子无故重启的
原因。不用的时候就关掉，程序退出时也会自动关。
"""


class BoardLight:
    """板载补光灯。`available` 为假时所有操作都是空动作。"""

    def __init__(self, on_at_start=False):
        self.available = False
        self.is_on = False
        self._gpio = None
        self.reason = ""
        try:
            from maix import gpio, sys, err
            try:
                from maix.peripheral import pinmap
            except ImportError:
                from maix import pinmap

            if sys.device_id() == "maixcam2":
                pin, func = "B25", "GPIOB25"
            else:
                pin, func = "B3", "GPIOB3"
            # 引脚默认可能是别的复用功能，必须先切成 GPIO 再建对象；
            # check_raise 是为了把"引脚名写错"变成一句明确的报错而不是静默失效。
            err.check_raise(pinmap.set_pin_function(pin, func),
                            "board light pin")
            self._gpio = gpio.GPIO(func, gpio.Mode.OUT)
            self.available = True
            # 先显式写一次，别沿用上电时的随机电平。这一步也顺便验证引脚真的能写，
            # 失败的话 set() 会自己把 available 翻回 False。
            self.set(on_at_start)
            if self.available:
                print("board light: {} ({})".format(pin, func))
        except Exception as e:
            self.reason = str(e)
            print("board light unavailable:", e)

    def set(self, on):
        """点亮/熄灭。返回是否真的写下去了。"""
        if not self.available:
            return False
        try:
            self._gpio.value(1 if on else 0)
        except Exception as e:
            # 写失败通常意味着引脚被别的东西抢走了，再试下去只会每帧刷屏
            print("board light write failed:", e)
            self.available = False
            self.reason = str(e)
            return False
        self.is_on = bool(on)
        return True

    def toggle(self):
        self.set(not self.is_on)
        return self.is_on

    def off(self):
        self.set(False)
