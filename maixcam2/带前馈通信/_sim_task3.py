"""TASK3 离线验证台 —— 临时文件，验证完可以删。

python _sim_task3.py math      校验距离闭式解
python _sim_task3.py nominal   标称工况，开环 vs 闭环
python _sim_task3.py delay     执行滞后敏感度
python _sim_task3.py drop      掉帧 / 离群点敏感度（开环最大的卖点）
python _sim_task3.py calib     标定收敛性（故意把 ol_a 设错）
python _sim_task3.py sweep     216 工况扫描
"""

import math
import sys

import task3_config as C
import task3_ctrl as T

G = 981.0
LOST_TIMEOUT_MS = 500        # main.py CONFIG["lost_timeout_ms"] 的默认值
# 观测器常数照抄 ball_position.AdaptiveAlphaBetaFilter，只是换算到 cm：
# 640px 画幅、两端各留 5px、对应 25cm  ->  约 25.2 px/cm
PX_PER_CM = 25.2
FAST_ERR_CM = 10.0 / PX_PER_CM
OBS_RESET_MS = 250           # 丢球这么久观测器整个复位（vx 归零）
DEFAULTS = dict(C.CONFIG)
QUIET = [False]

_print = print


def print(*a, **k):          # noqa: A001 - 扫描时把控制器的日志压掉
    if not QUIET[0]:
        _print(*a, **k)


T.print = print


class Plant:
    """球-杆模型。a = roll·g·sin(摆杆角) − 滚阻，摆杆角 = 电机角 / 连杆比。

    motor 三档执行器模型：
      instant  命令当帧立刻到位 —— 复现 README 那份 162 工况基线的假设
      slew     本帧内从上一个角度线性走到新角度 —— drive_mode=0 的真实行为
               （发送转速就是按"刚好下一帧到位"挑的），等效半帧滞后
      hold     整帧保持旧角度、帧末才跳 —— 整帧滞后，最坏情况
    """

    def __init__(self, crank=3.0, roll=5.0 / 7.0, zero_off=0.0, fric=0.0,
                 fps=25.0, beta_scale=1.0, noise=0.0, motor="slew", seed=1,
                 drop=0.0, burst=1, outlier=0.0, out_cm=3.0):
        self.crank, self.roll = crank, roll
        self.zero_off = zero_off      # 机械零偏（电机度）
        self.fric = fric              # 滚动阻力（cm/s²）
        self.dt = 1.0 / fps
        self.beta_scale = beta_scale  # 观测器速度增益相对标称的倍数
        self.noise = noise
        self.motor = motor
        self.drop = drop              # 每帧开始一次丢球的概率
        self.burst = max(1, burst)    # 一次丢球持续几帧
        self.outlier = outlier        # 离群点概率
        self.out_cm = out_cm          # 离群点幅度
        self.x = self.v = 0.0
        self.theta_act = 0.0
        self.hat_x = None             # 观测器状态
        self.hat_v = 0.0
        self._miss_ms = 0.0
        self._lost_left = 0
        self._rs = seed

    def _rand(self):
        self._rs = (1103515245 * self._rs + 12345) & 0x7FFFFFFF
        return self._rs / 0x7FFFFFFF * 2.0 - 1.0

    def _observe(self, meas):
        """ball_position.AdaptiveAlphaBetaFilter 的一维照抄版。

        速度是**位置残差积出来的**，不是真值滤波 —— 所以位置噪声和离群点会
        原样进到 v_cms 里。这一点很关键：控制器的加速度阻尼吃的就是这个 v，
        用"真值加低通"当速度会把噪声敏感度整个抹掉，据此调出来的 a_lpf 不能信。
        """
        if self.hat_x is None:
            self.hat_x, self.hat_v = meas, 0.0
            return meas, 0.0
        pred = self.hat_x + self.hat_v * self.dt
        e = meas - pred
        motion = min(1.0, abs(e) / FAST_ERR_CM) ** 2
        alpha = 0.20 + (0.95 - 0.20) * motion
        beta = (0.01 + (0.12 - 0.01) * motion) * self.beta_scale
        self.hat_x = pred + alpha * e
        self.hat_v += beta * e / self.dt
        return self.hat_x, self.hat_v

    def _detect(self):
        """返回这一帧的 (位置, 速度)，丢球返回 (None, 上一次的速度)。

        丢球期间观测器不更新（主程序那一帧根本不进控制分支）；连续丢够
        OBS_RESET_MS 主程序会 mark_missing 把观测器整个复位，速度归零。
        """
        if self._lost_left > 0:
            self._lost_left -= 1
        elif self.drop > 0.0 and (self._rand() + 1.0) * 0.5 < self.drop:
            self._lost_left = self.burst - 1
        else:
            self._miss_ms = 0.0
            pos = self.x + self.noise * self._rand()
            if self.outlier > 0.0 and (self._rand() + 1.0) * 0.5 < self.outlier:
                pos += math.copysign(self.out_cm, self._rand())
            return self._observe(pos)
        self._miss_ms += self.dt * 1000.0
        if self._miss_ms >= OBS_RESET_MS:
            self.hat_x, self.hat_v = None, 0.0
        return None, self.hat_v

    def step(self, theta_cmd, sub=20):
        th0 = self.theta_act
        h = self.dt / sub
        for i in range(sub):
            if self.motor == "instant":
                th = theta_cmd
            elif self.motor == "slew":
                th = th0 + (theta_cmd - th0) * (i + 1) / sub
            else:
                th = th0
            beam = math.radians((th + self.zero_off) / self.crank)
            a = self.roll * G * math.sin(beam)
            if abs(self.v) < 0.05 and abs(a) < self.fric:
                a = 0.0                       # 静摩擦死区
            elif self.v > 0.0:
                a -= self.fric
            elif self.v < 0.0:
                a += self.fric
            self.v += a * h
            self.x += self.v * h
            if abs(self.x) > C.BEAM_HALF_CM:  # 撞到端头
                self.x = math.copysign(C.BEAM_HALF_CM, self.x)
                self.v = 0.0
        self.theta_act = theta_cmd
        return self._detect()


def one_run(mk, hold_s=2.0, cap_s=15.0):
    """跑一趟 TASK3。返回 (结果 dict, fsm)，fsm 用来抄标定建议。

    帧的处理顺序照抄 main.py：丢球那一帧**整个控制分支都不进**（连电机命令
    都不发），连续丢超过 lost_timeout_ms 且状态不是 DONE 就被主程序抢去
    SAFE_LOST —— 这一趟就废了，记 "SAFE_LOST"。

    误差同时统计两份：`ep/en` 是屏幕上看到的（喂给状态机的量测值），
    `ep_t/en_t` 是球的真值。有离群点时两者会分家，而评委看的是真值。
    """
    for k, v in DEFAULTS.items():
        if not k.startswith("ol_"):
            C.CONFIG[k] = v               # 每趟都从默认闭环增益开始
    plant = mk()
    ctrl, fsm = T.Task3Controller(), T.Task3FSM()
    fsm.start()
    frame_ms = int(round(plant.dt * 1000.0))
    now, done_at, theta_pk, drift = 0, None, 0.0, 0.0
    peak, trough, settle_err = None, None, 0.0
    lost_ms, why = 0, None
    while now / 1000.0 < cap_s:
        pos, vel = plant.step(ctrl.theta_cmd)
        if pos is None:
            lost_ms += frame_ms
            if lost_ms > LOST_TIMEOUT_MS and fsm.state != T.Task3FSM.DONE:
                why = "SAFE_LOST"
                break
            now += frame_ms
            continue
        lost_ms = 0
        tgt = fsm.step(pos, vel, now)
        # The real UI requires a second TASK3 press after ARM reaches READY.
        # Simulate that operator confirmation immediately so the harness tests
        # the current state machine instead of waiting in READY until timeout.
        if fsm.state == T.Task3FSM.READY:
            fsm.confirm_start()
            tgt = fsm.step(pos, vel, now)
        if ctrl.update(tgt, pos, vel, plant.dt, now):
            why = "STUCK"
            break
        theta_pk = max(theta_pk, abs(ctrl.theta_cmd))
        if fsm.state not in (T.Task3FSM.IDLE, T.Task3FSM.ARM):
            peak = plant.x if peak is None else max(peak, plant.x)
        if fsm.state in (T.Task3FSM.GO_NEG, T.Task3FSM.SETTLE, T.Task3FSM.DONE):
            trough = plant.x if trough is None else min(trough, plant.x)
        if fsm.state in (T.Task3FSM.SETTLE, T.Task3FSM.DONE):
            settle_err = max(settle_err, abs(plant.x - C.TASK_NEG_CM))
        if fsm.state == T.Task3FSM.DONE:
            if done_at is None:
                done_at = now
            drift = max(drift, abs(plant.x - C.TASK_NEG_CM))
            if (now - done_at) / 1000.0 >= hold_s:
                break
        now += frame_ms
    if why is None and fsm.total_s is None:
        why = "TIMEOUT"
    if why:
        return dict(ok=False, why=why), fsm
    return dict(ok=True, t=fsm.total_s, ep=fsm.err_pos_cm, en=fsm.err_neg_cm,
                ep_t=abs((peak or 0.0) - C.TASK_POS_CM),
                en_t=max(abs((trough or 0.0) - C.TASK_NEG_CM), settle_err),
                theta=theta_pk, drift=drift, a1=C.CONFIG["ol_a1_cms2"],
                a2=C.CONFIG["ol_a2_cms2"]), fsm


def adopt(fsm):
    """照 _report_openloop 打印的建议把 ol_a 抄回配置 —— 模拟现场标定动作。

    可信度闸门必须和 task3_ctrl 里那套**一模一样**，否则测的是一个现场
    根本不会执行的标定流程。
    """
    for key, x0, x1, tgt in (("ol_a1_cms2", fsm.ol_x0_cm, fsm.ol_x1_cm,
                              C.TASK_POS_CM),
                             ("ol_a2_cms2", fsm.ol_x1_cm, fsm.ol_x2_cm,
                              C.TASK_NEG_CM)):
        if x0 is None or x1 is None:
            continue
        want, got = tgt - x0, x1 - x0
        if abs(want) < 0.1:
            continue
        ratio = got / want
        if ratio < T.CAL_MIN_RATIO:
            continue                       # 这趟不可信，配置不动
        C.CONFIG[key] *= min(ratio, T.CAL_MAX_RATIO)


def run_n(mk, rounds):
    """跑 rounds 趟，每趟结束抄一次建议值，返回最后一趟的结果。"""
    res = None
    for _ in range(rounds):
        res, fsm = one_run(mk)
        adopt(fsm)
    return res


def line(tag, r):
    """误差报真值（评委看的是球，不是量测值）。"""
    if not r["ok"]:
        return "{:<24} {}".format(tag, r["why"])
    return ("{:<24} t={:.2f}s  +5:{:.2f}  -5:{:.2f}  θpk={:4.1f}°  "
            "漂移{:.2f}  a=({:.0f},{:.0f})").format(
        tag, r["t"], r["ep_t"], r["en_t"], r["theta"], r["drift"],
        r["a1"], r["a2"])


def preset(mode, a=36.0):
    C.CONFIG["ol_enable"] = mode
    C.CONFIG["ol_a1_cms2"] = C.CONFIG["ol_a2_cms2"] = a


def compare(title, mk):
    _print("== {} ==".format(title))
    QUIET[0] = True
    preset(0)
    a = one_run(mk)[0]
    preset(1)
    b = one_run(mk)[0]
    preset(1)
    c = run_n(mk, 3)
    QUIET[0] = False
    _print(line("纯闭环", a))
    _print(line("双段 a=36 未标定", b))
    _print(line("双段 标定3趟", c))
    _print("")


def main_nominal():
    compare("标称：连杆3 滚动5/7 25fps 无噪无摩擦",
            lambda: Plant())
    compare("实感：+摩擦2.5 +噪声0.8mm +20fps +整帧执行滞后",
            lambda: Plant(fric=2.5, noise=0.08, fps=20.0, motor="hold"))


def main_delay():
    _print("== 执行滞后敏感度（其余全标称）==")
    for m in ("instant", "slew", "hold"):
        compare("执行器 = {}".format(m), lambda m=m: Plant(motor=m))


DROP_CASES = (
    ("干净",                 dict()),
    ("掉帧 5% 单帧",         dict(drop=0.05, burst=1)),
    ("掉帧 10% 单帧",        dict(drop=0.10, burst=1)),
    ("掉帧 5% 连丢3帧",      dict(drop=0.05, burst=3)),
    ("掉帧 8% 连丢5帧",      dict(drop=0.08, burst=5)),
    ("离群点 3% ±3cm",       dict(outlier=0.03)),
    ("离群点 6% ±3cm",       dict(outlier=0.06)),
    ("掉帧5%连3 + 离群3%",   dict(drop=0.05, burst=3, outlier=0.03)),
)


def main_drop():
    """开环真正的卖点：走位段不看球，所以掉帧和离群点动不了它。

    每种工况换 12 个随机种子，统计"跑完的趟数 / 最坏误差"。
    闭环那一列的失败几乎全是 SAFE_LOST —— 主程序丢球超时会把 TASK3 抢走，
    这一条开环也躲不掉（它同样要靠检测活着），所以两列都会中招；
    区别在跑完的那些趟里误差差多少。
    """
    _print("== 检测退化敏感度（25fps 摩擦1.5 噪声0.8mm，12 种子）==")
    _print("{:<22} {:^26} {:^26}".format("工况", "纯闭环", "双段(标定过)"))
    for label, kw in DROP_CASES:
        cells = []
        for mode in (0, 1):
            ok = 0
            ep = en = tmax = 0.0
            fails = {}
            for seed in range(1, 13):
                QUIET[0] = True
                preset(mode)
                r = run_n(lambda seed=seed, kw=kw: Plant(
                    fric=1.5, noise=0.08, seed=seed, **kw), 3)
                QUIET[0] = False
                if not r["ok"]:
                    fails[r["why"]] = fails.get(r["why"], 0) + 1
                    continue
                ok += 1
                ep = max(ep, r["ep_t"])
                en = max(en, r["en_t"])
                tmax = max(tmax, r["t"])
            note = " ".join("{}x{}".format(k[:4], v)
                            for k, v in sorted(fails.items()))
            cells.append("{:>2}/12 t{:.2f} +5:{:.2f} -5:{:.2f} {}".format(
                ok, tmax, ep, en, note))
        _print("{:<22} {:<26} {:<26}".format(label, *cells))


def main_calib():
    _print("== 标定收敛：起始 ol_a 故意设错，看几趟收敛 ==")
    for start in (12.0, 36.0, 100.0):
        for n in (1, 2, 3):
            QUIET[0] = True
            preset(1, start)
            r = run_n(lambda: Plant(fric=2.0), n)
            QUIET[0] = False
            _print(line("起始 a={:<5.0f} 第{}趟".format(start, n), r))
        _print("")


CASES = []
for crank in (2.5, 3.0, 4.0):
    for roll in (5.0 / 7.0, 0.55):
        for zero in (0.0, 2.0, -2.0):
            for fps in (20.0, 25.0, 30.0):
                for beta in (0.5, 2.0):
                    for noise in (0.0, 0.08):
                        CASES.append(dict(crank=crank, roll=roll,
                                          zero_off=zero, fps=fps,
                                          beta_scale=beta,
                                          noise=noise, fric=1.5))


def main_sweep():
    _print("== {} 工况扫描（滚阻 1.5cm/s²）==".format(len(CASES)))
    for motor in ("instant", "slew", "hold"):
        _print("-- 执行器 = {} --".format(motor))
        for label, mode, rounds in (("纯闭环", 0, 1),
                                    ("双段 a=36 未标定", 1, 1),
                                    ("双段 标定3趟", 1, 3)):
            bad = []
            tmax = epmax = enmax = thmax = drmax = 0.0
            for kw in CASES:
                QUIET[0] = True
                preset(mode)
                r = run_n(lambda kw=kw: Plant(motor=motor, **kw), rounds)
                QUIET[0] = False
                if not r["ok"]:
                    bad.append((kw, r["why"]))
                    continue
                tmax = max(tmax, r["t"])
                epmax = max(epmax, r["ep"])
                enmax = max(enmax, r["en"])
                thmax = max(thmax, r["theta"])
                drmax = max(drmax, r["drift"])
            _print("  {:<18} t_max={:.2f}  +5_max={:.2f}  -5_max={:.2f}  "
                   "θ_max={:4.1f}°  漂移_max={:.2f}  跑不完={}".format(
                       label, tmax, epmax, enmax, thmax, drmax, len(bad)))
        _print("")


def main_math():
    _print("== 距离闭式解 vs 数值积分（h=0.1ms）==")
    preset(1)
    for a, d in ((36.0, 5.0), (36.0, 10.0), (20.0, 5.0), (60.0, 10.0)):
        preset(1, a)
        p = T.OpenLoopPlan()
        if not p.build(d, d):
            _print("  a={} d={}  不可行: {}".format(a, d, p.error))
            continue
        h, t, v, x = 1e-4, 0.0, 0.0, 0.0
        while t < p.move2_start_s:
            for t0, t1, th0, th1, k in p.segs:
                if t0 <= t < t1:
                    u = (t - t0) / (t1 - t0)
                    v += k * (th0 + (th1 - th0) * u) * h
                    break
            x += v * h
            t += h
        _print("  a={:<5.0f} d={:<5.1f} T={:.4f}s  ->  末位移 {:.4f}cm  "
               "末速 {:+.4f}cm/s".format(a, d, p.t1_s, x, v))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "nominal"
    {"nominal": main_nominal, "sweep": main_sweep, "delay": main_delay,
     "calib": main_calib, "math": main_math, "drop": main_drop}[what]()
