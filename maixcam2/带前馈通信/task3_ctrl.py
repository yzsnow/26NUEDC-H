"""TASK3 的控制律与状态机 —— 纯算法，无任何硬件调用，可在 PC 上单独跑。

由 main.py 在 TASK3 模式下调用；HOLD / SAFE_LOST 走的仍是 cascade.py，两边
互不影响。参数全部从 task3_config.CONFIG 现场读取，所以屏幕上改一个值下一帧
就生效，不需要任何 sync()。

丢球恢复不在这里：那条路径由 main.py 的 SAFE_LOST 用 cascade 统一处理，
一个程序里只留一套回位逻辑。

双段控制（ol_enable = 1）—— 走位和守位交给两套完全不同的机制：

    走位  O -> +5 -> -5   OpenLoopPlan  开环播表，θ 只是时间的函数
    守位  锁定在 -5cm      Task3Controller  三级级联闭环

**为什么走位要开环**：这一段唯一难的是"停得下来"，而反对称的倾角波形让
∫a dt 恒等于 0，到点速度与加速度大小无关 —— 摩擦、电池电压、连杆比误差都
动摇不了它。开环还顺带免疫掉帧和识别抖动，这两个正是闭环在高速段最怕的。

**为什么守位必须闭环**：一个细分步（0.1125 电机度）的零点误差就有
0.46 cm/s² 的残余加速度，3 秒漂 2 cm，直接超题目 1 cm 的要求。开环没有任何
机制能发现这件事，所以最后那段必须有反馈。

ol_enable = 0 时整个播表机制不参与，回到原来的纯闭环，用来做 A/B 对比。

闭环部分的结构（三级级联，电机命令的是倾角**速度**而不是倾角）：

    x_ref ─▶ ① 位置外环 + 制动限速 ─▶ v_ref (cm/s)
    v_ref ─▶ ② 球速环 + 加速度阻尼  ─▶ omega (电机°/s)
    omega ─▶ ③ 行程限幅 ─▶ 积分 ─▶ theta_cmd (电机°)

为什么不是单环 PID：从电机角度到球位置中间夹着"倾角→重力分量→加速度→
速度→位置"两重积分，单环必须用同一个输出既撑出静态倾角对抗机械零偏、
又给球提供阻尼，两者互相打架，表现就是球停在目标旁边几毫米不动、到点
来回过冲。级联里静态倾角由 theta = ∫omega 自己长出来，所以 ki_x 可以是 0。

单位：球侧 cm / cm/s / cm/s^2，执行器侧**电机度**（不是摆杆度）。
"""

import math

import task3_config as C

CFG = C.CONFIG

# 开环取样点取几帧的中位数。5 帧在 25fps 下是 200ms，能扛住 3 帧里坏 2 帧，
# 又短到球还没来得及动 —— 不做成可调参数，它不是增益，是抗离群点的固定手段。
LANDMARK_FRAMES = 5

# omega 收敛判据的慢速平均系数。0.08 ≈ 12 帧 ≈ 0.5s（25fps），
# 比闭环自己的收敛时间（ωn=3 时约 1.6s）短一截，所以它跟得上收敛过程，
# 又足够长到把 ±9°/s 的差分噪声压到 1°/s 以下。不做成可调参数：它不是增益。
TRIM_LPF = 0.08

# 配平角取**滑窗的中点**（(min+max)/2），窗口取这么多帧。
# 32 帧在 25fps 下是 1.3 秒，比 ARM 段极限环的周期（实测约 1.1s）长一点 ——
# 这是这个数的唯一约束：窗口盖住整个周期，中点才等于周期的均值。
#
# 为什么是中点而不是低通：ARM 段闭环在摩擦死区里必然打极限环，theta_cmd
# 绕着真配平角来回摆，**任何**在固定时刻取的瞬时值（或没盖满一个周期的
# 平均）都是在这个周期上随机取相位。离线扫参里 arm_ms 1200/1500/2000/2500
# 对应最坏误差 0.81/3.21/0.80/3.66，非单调 —— 好坏纯看混叠到哪个相位，
# 从这种数据里挑 arm_ms 是自欺欺人。而对称振荡的中点与相位无关。
#
# 一阶低通解决不了这件事：想压住 1.1s 的周期，时间常数得 2s 以上，
# 那它从 0 爬到 3° 要 5 秒还没到位（实测 2.8s 时只爬到 2.3/3.0），
# 于是判据测的变成"滤波器追上了没"，而不是"配平稳了没"。滑窗没有这个
# 爬升过程：窗口一满，中点立刻就是对的。
TRIM_WIN = 32

# 配平角的**收缩死区**（电机度）：只补超出这个值的那一部分，abs 小于它就当 0。
#
# 配平补偿是个偏差-方差权衡。零偏大的时候它收益巨大（离线最坏误差 4.90 -> 0.88），
# 但零偏本来就是 0 的时候，估出来的全是噪声，照着补等于**往开环表里注入一个
# 随机直流偏置**：离线里离群点 3% 那档，装了配平补偿是 2.68cm，关掉是 1.28cm。
#
# 0.5° 是两边同时最好的点（零偏 0/±1/±2/±3 -> 0.51/0.68/0.58/0.53cm，
# 离群 0/3%/6% -> 0.33/1.28/2.62cm，离群那列正好等于"完全不补偿"的基线）。
# 不做成可调参数：它是这个估计器自身的不确定度（ARM 段极限环幅度的一半量级），
# 不是可以按机器调的增益。往大调会开始抵消真实零偏（1.2° 时 ±1° 零偏直接 5.0cm）。
TRIM_DEAD_DEG = 0.5

# 单趟标定允许把 ol_a 改动的比例上下限，见 _report_openloop。
CAL_MIN_RATIO = 0.3
CAL_MAX_RATIO = 2.0

# 试过两种"识别出机械零偏、然后拒绝给标定建议"的闸门，**两种都让结果更差，
# 别再加第三种**（离线数据在 _report_openloop 的注释里）。根本原因是：
# 即使标定用错了模型（把常值偏置当成 A 的比例误差去拟合），它给出的 ol_a
# 仍然在**减小**总误差 —— A 的比例部分确实抵消掉了一部分偏置位移。
# 拒绝标定等于把这部分抵消也丢掉，剩下完整的偏置误差，实测 2.96cm -> 7.49cm。
# 标定的正确定位是"最后一道经验补偿"，不是"A 的真值测量"，所以别替它把关。


def clamp(value, limit):
    return max(-limit, min(limit, value))


def blend(far, near, err_cm, band_cm):
    """误差大用 far 档、误差小用 near 档，中间线性过渡。

    直接按阈值切换会让增益在分界线上跳变，球恰好停在分界附近时表现为
    高频抖动；线性混合是最省事的消除办法。
    """
    if band_cm <= 0.0:
        return far
    t = min(1.0, abs(err_cm) / band_cm)
    return near + (far - near) * t


class OpenLoopPlan:
    """走位段的 θ(t) 时间表。一旦起跑就不再看球。

    一段走位由五段折线组成，关于自己的中点反对称：

        θ                             r = ol_ramp_ms   斜坡
     +θ │     ╱‾‾‾‾╲                  T = 解出来的保持时长
        │    ╱      ╲                 f = ol_flip_ms   反向斜坡
      0 │───╱   r T  ╲f
        │             ╲      ╱
     -θ │              ╲____╱
             └r┘└T┘└─f─┘└T┘└r┘

    反对称 => ∫a dt = 0 => **到点速度恒为零，与 a 的大小无关**。这是敢在
    走位段用开环的全部理由：a 标定错了只会停错地方，不会停不下来。

    距离有闭式解。设 A = 满倾角下的球加速度，逐段积分（下面 _hold_for 的
    推导）得到

        d = A · ( T² + T(r+f) + r²/3 + r·f/2 + f²/6 )

    正着用：知道要走多远，解二次方程得到 T，每次起跑现算。
    反着用：跑一趟量出实际走了多远，反解出真实的 A —— 所以标定是"跑一趟
    抄一个数"，不是试凑毫秒值。见 task3_config 第 4.1 节。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.active = False
        self.t_start_ms = 0
        self.segs = ()            # (t0, t1, θ起, θ止, 每度加速度)
        self.move2_start_s = 0.0  # 停顿结束、第二段开始
        self.total_s = 0.0
        self.t1_s = 0.0
        self.t2_s = 0.0
        self.theta_deg = 0.0
        self.error = ""
        self.trim_deg = 0.0
        self.omega_dps = 0.0      # 控制器每帧回填，给状态机的 ARM 判据用
        self.trim_settled = 9e9   # 同上：配平角滑窗的峰峰值（窗口没满时是 9e9）
        self._r = self._f = self._th = self._a2 = 0.0
        self._move2_idx = 0

    # ---------- 建表 ----------

    @staticmethod
    def _hold_for(d_cm, a_cms2, r_s, f_s):
        """解 d = a·(T² + T(r+f) + r²/3 + rf/2 + f²/6) 中的 T，无解返回 None。

        推导：a(t) 按 θ(t) 同形（小角近似 a ∝ sinθ ≈ θ）逐段两次积分，
        末速 (r/2)A + TA + 0 − TA − (r/2)A = 0 自动成立，位移合并同类项即得
        上式。r = f = 0 时退化成教科书的 bang-bang d = A·T²。

        无解只有一种情形：a 太大，光是两条斜坡走过的距离就超过 d 了
        （T 会解出负数）。这时该减小 ol_theta_deg，缩斜坡是治标。
        """
        if a_cms2 <= 0.0:
            return None
        b = r_s + f_s
        c = (r_s * r_s / 3.0 + r_s * f_s / 2.0 + f_s * f_s / 6.0
             - abs(d_cm) / a_cms2)
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        t = (-b + math.sqrt(disc)) / 2.0
        return t if t >= 0.0 else None

    def _move_segs(self, t0_s, sign, hold_s, per_deg):
        """一段走位的五段折线。返回 (segs, 结束时刻)。"""
        a = sign * self._th
        out = []
        t = t0_s
        for dur, s0, s1 in ((self._r, 0.0, a), (hold_s, a, a),
                            (self._f, a, -a), (hold_s, -a, -a),
                            (self._r, -a, 0.0)):
            out.append((t, t + dur, s0, s1, per_deg))
            t += dur
        return out, t

    def build(self, d1_cm, d2_cm):
        """按两段距离建表。返回 False 表示参数不可行，调用方应退回闭环。"""
        self.reset()
        self._r = max(0.0, CFG["ol_ramp_ms"] / 1000.0)
        self._f = max(0.0, CFG["ol_flip_ms"] / 1000.0)
        # 倾角先夹进机械行程 —— 这个值是主程序镜像过来的机器事实
        self._th = min(abs(CFG["ol_theta_deg"]), abs(CFG["max_motor_deg"]))
        a1 = abs(CFG["ol_a1_cms2"])
        self._a2 = abs(CFG["ol_a2_cms2"])
        dwell = max(0.0, CFG["ol_dwell_ms"] / 1000.0)

        if self._th <= 0.0 or a1 <= 0.0 or self._a2 <= 0.0:
            self.error = "OL: 倾角或加速度为零"
            return False
        t1 = self._hold_for(d1_cm, a1, self._r, self._f)
        t2 = self._hold_for(d2_cm, self._a2, self._r, self._f)
        if t1 is None or t2 is None:
            self.error = "OL: 倾角过大，光斜坡就走过头了（减小 ol_theta_deg）"
            return False

        segs, t = self._move_segs(0.0, 1.0 if d1_cm >= 0.0 else -1.0,
                                  t1, a1 / self._th)
        segs.append((t, t + dwell, 0.0, 0.0, 0.0))
        t += dwell
        self.move2_start_s = t
        self._move2_idx = len(segs)
        more, t = self._move_segs(t, 1.0 if d2_cm >= 0.0 else -1.0,
                                  t2, self._a2 / self._th)
        segs.extend(more)

        self.segs = tuple(segs)
        self.t1_s, self.t2_s = t1, t2
        self.total_s = t
        self.theta_deg = self._th
        if t > C.TASK_LIMIT_S:
            print("OL 警告: 播表全长 {:.2f}s 已超题目 {:.0f}s，"
                  "加大 ol_a1/ol_a2 或 ol_theta_deg".format(t, C.TASK_LIMIT_S))
        return True

    def replan_move2(self, d2_cm):
        """用第一段的实际落点重算第二段 —— 开环全程唯一一次看球。

        不做这一步的话，第一段差多少就原样带到 -5cm 去，而 -5cm 的误差是
        直接计分的。只重算尾巴，前面已经播过的部分不动。
        """
        t2 = self._hold_for(d2_cm, self._a2, self._r, self._f)
        if t2 is None or not self.segs:
            return False
        segs = list(self.segs[:self._move2_idx])
        more, t = self._move_segs(self.move2_start_s,
                                  1.0 if d2_cm >= 0.0 else -1.0,
                                  t2, self._a2 / self._th)
        segs.extend(more)
        self.segs = tuple(segs)
        self.t2_s = t2
        self.total_s = t
        return True

    # ---------- 播放 ----------

    def start(self, now_ms):
        self.active = True
        self.t_start_ms = now_ms
        self.trim_deg = 0.0     # 由控制器在播表第一帧填入，见 theta_now

    def finish(self):
        self.active = False

    def clock(self, now_ms):
        """播表时钟（秒，墙上时间）。相位判断、结束判断都用它，状态机和控制器
        必须用同一个，否则会出现"控制器已经播完、状态机还以为在播"这种半截状态。

        **提前量不在这里**：它只能加在查表上（见 theta_now），不能加进时钟。
        加进时钟等于把整张表的末尾也提前砍掉 lead 毫秒，最后那段回零斜坡被
        截断，球反而多带走一截速度 —— 离线量过，lead=60ms 时落点从 -5.19
        变成 -5.37，比不加提前量还差。
        """
        return (now_ms - self.t_start_ms) / 1000.0

    def theta_at(self, t_s):
        """表上 t 时刻的倾角（物理方向，未含 dir_invert）。"""
        if t_s <= 0.0:
            return 0.0
        for t0, t1, th0, th1, _k in self.segs:
            if t_s < t1:
                if t1 <= t0:
                    continue
                return th0 + (th1 - th0) * (t_s - t0) / (t1 - t0)
        return 0.0

    def pred_v_at(self, t_s):
        """模型预测的球速。屏幕上的 V* 在播表期间显示的就是它 —— 和实际球速
        一对比就知道 ol_a* 标定得准不准，不用等跑完看日志。"""
        v = 0.0
        for t0, t1, th0, th1, k in self.segs:
            if t_s <= t0 or t1 <= t0:
                continue
            te = t1 if t_s > t1 else t_s
            u = (te - t0) / (t1 - t0)
            v += k * (th0 + (th1 - th0) * u * 0.5) * (te - t0)
        return v

    def theta_now(self, now_ms):
        """播表期间的电机命令角（已含 dir_invert）；不在播表期返回 None。

        取的是**墙上时间**而不是累加 dt：掉一帧只是下一帧直接跳到该在的角度，
        整条时间表不会被拖慢 —— 开环抗掉帧就是这么来的。

        ol_lead_ms 是提前量，**只加在查表上**。命令每帧才更新一次、电机还要
        花时间走过去，实际倾角整体比表慢半拍；反对称保证 ∫a dt = 0 是对**理想
        波形**说的，慢半拍的阶梯波形积不到零，球就带着零点几 cm/s 冲过目标。
        提前一点取值正好把这一拍还回去：离线量到交棒残速 -0.22 -> -0.07 cm/s
        （整帧滞后工况）。理论值就是半帧，25fps 下 20ms，所以默认 20。

        超出表尾的查表返回 0，于是最后 lead 毫秒命令的是水平 —— 这正是想要的，
        实际倾角滞后一拍，刚好在表尾那一刻回到水平。

        **trim_deg 是整张表的直流偏置，不加它开环必崩。** 反对称保证 ∫a dt = 0
        只对"命令角产生的那部分加速度"成立；机械零点偏了 2°，就多出一个全程
        不变的偏置加速度 a_off，它不反对称，一路积成 ½·a_off·T²。连杆比 2.5 时
        2° 折合 9.8 cm/s²，move1 跑 1.4s 就能多走 9cm —— 球直接出杆。
        （离线扫参里 +5 误差最坏 7.7cm 就是这么来的，全部集中在 zero_off≠0。）

        而这个偏置是**免费**拿得到的：ARM 阶段闭环正托着球停在中心，theta_cmd
        已经积到了配平角，那个值就是 -a_off 对应的角度。控制器在播表第一帧把它
        抄进 trim_deg，整张表抬平，偏置加速度就被抵消掉了。所以它加在 sign 之后
        —— theta_at 是球侧的量，trim 是已经成形的电机命令角。
        """
        if not self.active:
            return None
        t = self.clock(now_ms)
        # 播过头也返回 None：万一状态机没来得及收尾，也是退回闭环而不是
        # 一直举着一个过期的角度
        if t > self.total_s:
            return None
        sign = -1.0 if CFG["dir_invert"] else 1.0
        return sign * self.theta_at(t + CFG["ol_lead_ms"] / 1000.0) \
            + self.trim_deg


# 走位表是"这一次 TASK3 运行"的状态，而 main.py 是分别构造 Task3Controller 和
# Task3FSM 的（状态机排表、控制器播表），我们不改主程序就没法把它从一个传给
# 另一个 —— 所以放在模块级，和上面 `CFG = C.CONFIG` 是同一个模式。
# 一个程序里只可能有一个 TASK3 在跑，单例是安全的。
PLAN = OpenLoopPlan()


class Task3Controller:
    """三级级联 + 行程保护 + 卡球检测。

    theta_cmd 是发给电机的命令角，纯粹由 omega 积分而来，永远不被电机回读
    污染，这样发出去的绝对位置帧才是干净的单调信号。
    """

    SATURATED_SCALE = 0.05  # 软限位把外向指令压到这个比例以下就算"顶住了"

    def __init__(self):
        self.theta_cmd = 0.0
        self.omega = 0.0
        self.v_ref_cms = 0.0
        self.err_cm = 0.0
        self.a_est = 0.0
        self.limited = False
        self.saturated = False
        self._was_ol = False
        self.reset_trim()
        self.reset()

    def reset(self, theta_deg=None):
        self.integral = 0.0
        self.v_prev = None
        self.a_est = 0.0
        self.omega = 0.0
        self.v_ref_cms = 0.0
        self.err_cm = 0.0
        self.stuck_since = None
        self.x_prev = None
        self.v_est = 0.0
        self.was_playing = False
        if theta_deg is not None:
            self.theta_cmd = float(theta_deg)

    def reset_trim(self):
        """清掉配平滑窗。**只在水平基准变了的时候调**（按 ZERO）。

        注意它不在 reset() 里：arm_skip=1 时配平角是在 HOLD 段量的，而按下
        TASK3 会先 reset() 交棒 —— 顺手清掉窗口就等于把刚量到的配平角扔了，
        走位段拿到 0，机械零偏一点都补不掉。窗口是跨模式的观测量，不是控制
        器状态，所以它的生命周期跟 reset() 无关，只跟"水平在哪"有关。
        """
        self.trim_hist = []       # 平衡角滑窗，均值 = 配平角（见 TRIM_WIN）
        self.trim_est = 0.0       # 滑窗均值，就是抄给走位段的配平角
        self.omega_slow = 0.0     # omega 的慢速平均，判断闭环收没收敛
        PLAN.trim_settled = 9e9

    def observe_trim(self, theta_deg, omega_dps):
        """喂一帧"闭环托着球停在中心"的平衡角进滑窗。

        调用方有两个，喂的是同一个物理量（电机度）：
          · 本控制器的 ARM 段（arm_skip=0），theta_cmd 来自 TASK3 的级联；
          · main.py 的 HOLD 段（arm_skip=1），theta_cmd 来自 cascade.py。
        HOLD 的目标恒为中心 O，和 ARM 一样，所以两边的平衡角就是同一个
        -机械零偏。增益不同只影响极限环的形状，不影响它的均值。
        """
        self.omega_slow += TRIM_LPF * (omega_dps - self.omega_slow)
        self.trim_hist.append(theta_deg)
        if len(self.trim_hist) > TRIM_WIN:
            self.trim_hist.pop(0)
        lo, hi = min(self.trim_hist), max(self.trim_hist)
        # 取窗口的**时间平均**当配平角，不取 (min+max)/2。极限环并不对称
        # （慢慢漂过去、快速弹回来），中点只对对称波形才等于均值 ——
        # 离线对照过：中点偶尔会偏到 -1.59° / +5.06°，同一时刻的平均值
        # 还在 ±0.5° 以内。峰峰值仍然由 min/max 给，它量的是幅度不是均值。
        # 也别改成中位数 —— 试过，抗离群点反而更差（离群 3% 时 5.52cm，
        # 平均值 2.68cm）：污染不是来自个别尖峰，而是闭环对尖峰的整段反应。
        self.trim_est = sum(self.trim_hist) / len(self.trim_hist)
        # 窗口没盖满一个极限环周期之前，峰峰值是**偏小**的，不是"稳"。
        # 尤其是第一帧：窗口里只有一个值，峰峰值 0，闸门大开 ——
        # 起跑瞬间 theta 还是 0，抄走的配平角就是 0，等于没补偿。
        # （arm_ms=200 那一档最坏 5.47cm 就是从这个假窗口里溜出去的。）
        PLAN.trim_settled = (hi - lo if len(self.trim_hist) >= TRIM_WIN
                             else 9e9)
        PLAN.omega_dps = abs(self.omega_slow)

    # ---------- ⓪ 球速：自己算，不用主程序传进来的 ----------

    def _ball_speed(self, x_cm, v_ext_cms, dt):
        """把 main.py 传进来的 v_cms 换成对位置的一阶差分。

        为什么要换：主程序的 v_cms 来自 ball_position 的 alpha-beta 观测器，
        速度是位置残差**积**出来的，增益 beta 只有 0.01~0.12，时间常数
        1/beta ≈ 8 帧 ≈ 0.3 秒。位置估计是准的，唯独速度慢了三分之一秒。

        0.3 秒的相位滞后对这个速度环是致命的：离线量到走位段结束那一瞬间，
        球真值已经停住（v=-0.06 cm/s），观测器还报 -5.9 cm/s，闭环于是去
        "刹一个并不存在的速度"，一来一回就发散了。差分只滞后一帧。

        代价是位置噪声被 1/dt 放大，所以有 v_lpf 低通；单帧离群点则靠
        "一帧之内速度变化不可能超过 a_max_cms2·dt" 挡掉 —— 和加速度那一路
        用的是同一个物理判据，不必再多一个参数。

        观测器要是哪天调快了、或者实物上差分噪声太大，v_from_pos 设 0 就退回
        原来的行为，两种都留在调参页上。
        """
        if not CFG["v_from_pos"]:
            self.x_prev = x_cm
            self.v_est = v_ext_cms
            return v_ext_cms
        if self.x_prev is None or dt <= 0.0:
            self.x_prev = x_cm
            return self.v_est
        v_raw = (x_cm - self.x_prev) / dt
        self.x_prev = x_cm
        step = abs(CFG["a_max_cms2"]) * dt
        v_raw = max(self.v_est - step, min(self.v_est + step, v_raw))
        self.v_est += min(max(CFG["v_lpf"], 0.0), 1.0) * (v_raw - self.v_est)
        return self.v_est

    # ---------- ① 位置外环 ----------

    def _position_loop(self, x_ref_cm, x_cm, dt):
        err = x_ref_cm - x_cm
        self.err_cm = err

        kp = blend(CFG["kp_far"], CFG["kp_near"], err, CFG["near_band_cm"])
        v = kp * err

        if CFG["ki_x"] > 0.0 and dt > 0.0 and abs(err) < CFG["i_zone_cm"]:
            self.integral = clamp(self.integral + CFG["ki_x"] * err * dt,
                                  CFG["i_lim_cms"])
            v += self.integral
        elif CFG["ki_x"] <= 0.0:
            self.integral = 0.0

        # 制动限速：接近目标时的速度上限 = "以 a_brake 减速刚好能停住"的速度。
        # 这一项才是快和准能同时拿到的原因 —— 允许把 kp 调很大来抢起步时间，
        # 同时保证进场速度始终在可刹停范围内。v_floor 是下限，否则误差趋零时
        # 上限也趋零，末端一点小偏差都补不动。
        v_brake = math.sqrt(2.0 * max(0.0, CFG["a_brake_cms2"]) * abs(err))
        v_cap = min(CFG["v_max_cms"], max(CFG["v_floor_cms"], v_brake))
        return clamp(v, v_cap)

    def _guard_ends(self, v_ref_cms, x_cm):
        """把继续往摆杆末端推的速度指令淡出，避免把球顶出去。"""
        guard = abs(CFG["x_guard_cm"])
        travel = abs(x_cm)
        if travel <= guard:
            return v_ref_cms
        outward = (v_ref_cms > 0.0 and x_cm > 0.0) or \
                  (v_ref_cms < 0.0 and x_cm < 0.0)
        if not outward:
            return v_ref_cms
        span = max(1.0, C.BEAM_HALF_CM - guard)
        scale = min(max((C.BEAM_HALF_CM - travel) / span, 0.0), 1.0)
        return v_ref_cms * scale

    # ---------- ② 球速环 ----------

    def _track_accel(self, v_cms, dt):
        """更新球加速度估计。播表期间也照跑 —— 交棒那一帧滤波器已经是热的，
        不会因为冷启动甩出一个假的阻尼脉冲。"""
        if self.v_prev is None or dt <= 0.0:
            a_raw = 0.0
        else:
            a_raw = (v_cms - self.v_prev) / dt
        self.v_prev = v_cms
        # 单帧误检会产生一个巨大的 a_raw，先砍掉再进滤波器，
        # 否则一个离群点就能把摆杆猛甩一下
        a_raw = clamp(a_raw, CFG["a_max_cms2"])
        self.a_est += CFG["a_lpf"] * (a_raw - self.a_est)

    def _velocity_loop(self, v_ref_cms, v_cms, dt):
        self._track_accel(v_cms, dt)

        kv = blend(CFG["kv_far"], CFG["kv_near"], self.err_cm,
                   CFG["near_band_cm"])
        ka = blend(CFG["ka_far"], CFG["ka_near"], self.err_cm,
                   CFG["near_band_cm"])
        # ka 是超前项，符号与主项相反，所以是在球**到达目标之前**就开始回倾，
        # 而不是冲过去再拉回来。
        omega = kv * (v_ref_cms - v_cms) - ka * self.a_est
        return clamp(omega, CFG["omega_max_dps"])

    # ---------- ③ 行程限幅 ----------

    def _limit(self, omega_dps, theta_deg):
        self.limited = False
        self.saturated = False
        limit = abs(CFG["max_motor_deg"])
        if limit <= 0.0:
            self.limited = True
            self.saturated = True
            return 0.0

        outward = (omega_dps > 0.0 and theta_deg > 0.0) or \
                  (omega_dps < 0.0 and theta_deg < 0.0)
        if not outward:
            return omega_dps

        travel = abs(theta_deg)
        if travel >= limit:
            self.limited = True
            self.saturated = True
            return 0.0

        soft = limit * min(max(CFG["soft_frac"], 0.0), 1.0)
        if travel <= soft:
            return omega_dps
        span = limit - soft
        scale = 0.0 if span <= 1e-6 else (limit - travel) / span
        scale = min(max(scale, 0.0), 1.0)
        self.limited = True
        # 软限位里 omega 渐近趋零，theta 永远不会真的等于 limit，所以"顶住了"
        # 不能用 travel >= limit 判断，只能看外向指令被压掉了多少。
        self.saturated = scale <= self.SATURATED_SCALE
        return omega_dps * scale

    # ---------- 对外 ----------

    def update(self, x_ref_cm, x_cm, v_cms, dt, now_ms):
        """跑完三级，返回 True 表示卡球（调用方应当停机）。

        走位段（PLAN 在播）整条级联被旁路，theta_cmd 直接由时间表给定；
        接口和返回值不变，所以 main.py 一行都不用改。
        """
        x_ref_cm = clamp(x_ref_cm, CFG["x_ref_max_cm"])
        # 走位段也要更新 —— 交棒那一帧速度估计必须已经是热的、且是**当前**的，
        # 这正是原来那套观测器速度做不到的事。
        v_cms = self._ball_speed(x_cm, v_cms, dt)
        # 播表刚起的那一帧：把 ARM 段闭环积出来的配平角抄给走位表当直流偏置。
        # 此刻 theta_cmd 还是闭环托着球停在中心时的角度，也就是 -机械零偏，
        # 正是抵消偏置加速度需要的那个值（详见 OpenLoopPlan.theta_now）。
        # omega 的收敛判据取**慢速平均**，不取瞬时值。位置噪声 0.8mm 经过
        # 差分放大 1/dt 之后，在 omega 上就是 ±9°/s 的抖动 —— 拿瞬时 omega 当
        # "收敛了没"的判据，门槛松了没意义、紧了永远进不去（试过 1.5°/s，
        # 全部 TIMEOUT）。平均之后噪声按 sqrt(N) 掉下去，判据才有分辨力。
        #
        # 配平角走另一条路：取 theta_cmd 滑窗的**中点**，判据取同一个窗口的
        # **峰峰值**。两件事一个窗口办完 —— 中点是极限环的均值（与相位无关），
        # 峰峰值就是极限环的幅度，也就是"稳没稳"本身。理由见 TRIM_WIN。
        if not PLAN.active:
            self.observe_trim(self.theta_cmd, self.omega)
        PLAN.omega_dps = abs(self.omega_slow)

        if PLAN.active and not self.was_playing:
            # 只补超出 TRIM_DEAD_DEG 的那部分，理由见那里。日志两个数都打：
            # 前面是估计值（现场判断机械零点做得好不好，看这个），
            # 后面是实际加进播放表的量。
            est = self.trim_est
            if len(self.trim_hist) < TRIM_WIN:
                # arm_skip=1 且没先按过 HOLD：窗口是空的或半满的。半满窗口的
                # 均值来自一段瞬态，比 0 还坏（它带方向），所以宁可不补 ——
                # 但必须说出来，否则现场只会看到"标定怎么标都收敛不了"。
                print("OL 配平: 滑窗只有 {}/{} 帧，本趟不补配平角"
                      "（先按 HOLD 托住球两秒，再按 TASK3）"
                      .format(len(self.trim_hist), TRIM_WIN))
                est = 0.0
            PLAN.trim_deg = (0.0 if abs(est) <= TRIM_DEAD_DEG
                             else est - (TRIM_DEAD_DEG if est > 0.0
                                         else -TRIM_DEAD_DEG))
            print("OL 配平: 估计 {:+.2f}°  实补 {:+.2f}°".format(
                est, PLAN.trim_deg))
        self.was_playing = PLAN.active

        theta_ol = PLAN.theta_now(now_ms)
        if theta_ol is not None:
            return self._playback(theta_ol, x_ref_cm, x_cm, v_cms, dt, now_ms)
        if self._was_ol:
            # 交棒：角度保持连续（不 reset theta_cmd，否则摆杆会瞬间回水平
            # 把刚送到位的球放跑），只清掉播表期间没意义的积分和卡球计时。
            # v_prev / a_est 一路都在更新，这里不能清。
            self._was_ol = False
            self.integral = 0.0
            self.stuck_since = None

        v_ref = self._position_loop(x_ref_cm, x_cm, dt)
        v_ref = self._guard_ends(v_ref, x_cm)
        self.v_ref_cms = v_ref

        omega = self._velocity_loop(v_ref, v_cms, dt)
        # 符号只在这一处翻转。正的位置误差必须产生正的电机角度；方向反了
        # 只改 dir_invert，**绝对不要**在别处再加负号 —— 两处翻转各自看着
        # 都对、合起来抵消，是最难查的一类 bug。
        omega *= (-1.0 if CFG["dir_invert"] else 1.0)
        omega = self._limit(omega, self.theta_cmd)
        self.omega = omega
        self.theta_cmd = clamp(self.theta_cmd + omega * dt,
                               CFG["max_motor_deg"])

        return self._check_stuck(v_cms, now_ms)

    def _playback(self, theta_ol, x_ref_cm, x_cm, v_cms, dt, now_ms):
        """开环走位：theta_cmd 由时间表直接给定，不看球。"""
        self._was_ol = True
        limit = abs(CFG["max_motor_deg"])
        theta_ol = clamp(theta_ol, limit)
        # omega 只用来给电机挑发送转速（位置模式下要"刚好下一帧到位"），
        # 所以按角度差算并夹在上限内。夹住不影响 theta_cmd 的准确性 ——
        # 绝对位置帧照发，电机慢一两帧自己会追上。
        self.omega = 0.0 if dt <= 0.0 else clamp(
            (theta_ol - self.theta_cmd) / dt, CFG["omega_max_dps"])
        self.theta_cmd = theta_ol

        self.err_cm = x_ref_cm - x_cm
        # 播表期间屏幕上的 V* 显示的是**模型预测球速**（闭环时它是目标球速）。
        # 拿它和球的实际运动一对比，当场就知道 ol_a* 标偏了没有。
        self.v_ref_cms = PLAN.pred_v_at(PLAN.clock(now_ms))
        self._track_accel(v_cms, dt)
        self.integral = 0.0
        self.limited = limit > 0.0 and abs(theta_ol) >= limit - 1e-6
        self.saturated = False
        # 卡球检测在这里必须关掉：开环本来就有"倾角很大而球暂时不动"的时刻
        # （斜坡刚起、静摩擦还没被克服），照闭环那套判据会误停机。
        self.stuck_since = None
        return False

    def _check_stuck(self, v_cms, now_ms):
        """有误差 + 球不动 + 已经顶住限位 = 再加指令也没用，该停机了。"""
        bad = (abs(self.err_cm) > CFG["stuck_err_cm"]
               and abs(v_cms) < CFG["stuck_v_cms"]
               and self.saturated)
        if not bad:
            self.stuck_since = None
            return False
        if self.stuck_since is None:
            self.stuck_since = now_ms
            return False
        if now_ms - self.stuck_since >= CFG["stuck_ms"]:
            self.integral = 0.0
            return True
        return False


class Task3FSM:
    """O -> +5 -> 折返 -> -5 -> 稳定 -> 永久锁定在 -5。

    没有 FAIL 态：题目要求球最终停在 -5cm，所以超时只是把 overtime 标红，
    目标永远不会退回中心，也永远不会松开摆杆。
    """

    IDLE, ARM, READY, GO_POS, GO_NEG, SETTLE, DONE = (
        "IDLE", "ARM", "READY", "GO+5", "GO-5", "SETTLE", "DONE")

    def __init__(self):
        self.state = self.IDLE
        self._clear()

    def _clear(self):
        self.go_pressed = False   # 二次确认：READY 段收到的第二次 TASK3 按键
        self.t0 = None            # 计时起点 = 离开 ARM/READY 的瞬间
        self.arm_since = None
        self.arm_wait0 = None     # 进 ARM 的时刻，给配平兜底超时用
        self.arrive_since = None
        self.settle_since = None
        self.total_s = None
        self.overtime = False
        self.peak_pos_cm = None    # 全程最高点 = +5 处真正的折返点
        self.trough_pos_cm = None  # 返程最低点 = -5 处的过冲/欠冲极值
        self.err_settle_cm = 0.0   # 进入稳定段之后的最大偏差
        self.ol_x0_cm = None       # 开环三个取样点，只用来标定 ol_a1/ol_a2
        self.ol_x1_cm = None
        self.ol_x2_cm = None
        self._pos_hist = []        # 最近几帧位置，给 _landmark 取中位数
        PLAN.reset()

    def start(self):
        self.state = self.ARM
        self._clear()

    def stop(self):
        self.state = self.IDLE
        self._clear()

    @property
    def running(self):
        return self.state != self.IDLE

    @property
    def err_pos_cm(self):
        """+5cm 处的误差绝对值 = |折返点 - 5|。

        用全程最高点而不是"进窗那一帧的位置"：命令折返之后球还会靠惯性
        再往前滑一段，那个最高点才是评分看到的折返位置。
        """
        if self.peak_pos_cm is None:
            return 0.0
        return abs(self.peak_pos_cm - C.TASK_POS_CM)

    @property
    def err_neg_cm(self):
        """-5cm 处的误差绝对值。

        取两者较大：返程最低点（过冲，或者根本没走到的欠冲）和稳定段里的
        最大偏差（稳态跑偏）。只看其中一个都会漏掉另一种失分方式。
        不能用"第一次进入某个窗口"当统计起点——那样量到的是进窗那一刻的
        距离，等于把窗口宽度当成误差报出来。
        """
        extreme = 0.0 if self.trough_pos_cm is None \
            else abs(self.trough_pos_cm - C.TASK_NEG_CM)
        return max(extreme, self.err_settle_cm)

    def _landmark(self, pos_cm):
        """开环取样点：取最近几帧的**中位数**，不要单帧值。

        三个取样点都是拿来算距离的，而距离要除到 ol_a 里去 —— 一个 ±3cm 的
        识别离群点正好落在取样那一帧，标定值就错 60%，下一趟直接把球送出杆外。
        ol_x1 更糟：它当场喂给 replan_move2()，坏的是**本趟**的第二段。
        离线扫参里"离群点 3%"那一档，单帧取样时 12 趟有 3 趟把球顶到端头
        （误差 7.5cm），换成中位数之后为 0。

        三个取样点都在球接近静止的时刻（起跑前、+5 顶点、播完），所以拿几帧
        前的位置来做中位数不会引入偏差 —— 这也是敢用中位数的前提。
        """
        h = self._pos_hist
        if not h:
            return pos_cm
        return sorted(h)[len(h) // 2]

    def confirm_start(self):
        """第二次按 TASK3 = 发车。只置标志，真正起跑在本帧的 step() 里。

        起跑要用**当帧的球位置**排表（第一段距离是 5 − 实际起跑位置），而按键
        是在识别之后、控制之前处理的，所以同一帧就能吃到，不欠一帧延迟。
        """
        if self.state == self.READY:
            self.go_pressed = True

    def _arm_done(self, pos_cm, now_ms):
        """配平完成。要么直接发车，要么停在 READY 等第二次按键。"""
        if CFG["arm_confirm"]:
            if self.state != self.READY:
                self.state = self.READY
                print("OL 配平完成，等第二次按 TASK3 发车")
            return
        self._leave_arm(pos_cm, now_ms)

    def _leave_arm(self, pos_cm, now_ms):
        """真正发车：计时起点和播表起点必须是同一帧，否则计时白跑一帧。"""
        self.state = self.GO_POS
        self.t0 = now_ms
        self._start_plan(pos_cm, now_ms)

    def step(self, pos_cm, v_cms, now_ms):
        """推进状态机，返回本帧的目标位置 (cm)。"""
        self._track_errors(pos_cm)
        self._pos_hist.append(pos_cm)
        if len(self._pos_hist) > LANDMARK_FRAMES:
            self._pos_hist.pop(0)

        if self.state == self.IDLE:
            return 0.0

        if self.state == self.ARM and CFG["arm_skip"]:
            # 免回正：球由人摆到位，按下 TASK3 的**这一帧**就起跑，不做配平。
            # 代价有两条，都是操作纪律能挡住的：
            #   · 播表假设起跑速度为 0（反对称保证的是"命令角积分为零"），
            #     手还没离开球就按，那点初速会原样加到落点上；
            #   · 配平角只能从 HOLD 段的滑窗里拿（见 observe_trim）。没按过
            #     HOLD 就直接按 TASK3，本趟不补配平，日志会明说。
            # 这一档不走二次确认 —— 没有配平段，也就没有"等它配平完"这回事，
            # 按键本身就是发车信号。
            self._leave_arm(pos_cm, now_ms)

        if self.state == self.ARM:
            # 球停住**还不够，摆杆也得停住**。开环走位要拿这一刻的 theta_cmd
            # 当配平角（抵消机械零偏，见 OpenLoopPlan.theta_now），而 300ms 不
            # 够 theta 在 ωn=3 下积到位 —— 抄走一个没收敛的角度，等于给走位段
            # 留了个常值偏置加速度：它帮正向那一段、拖反向那一段，于是标定把
            # ol_a1 越标越小、ol_a2 越标越大（离线里见过 a=(25,100) 这种分叉，
            # 一眼就能认出来是配平没收敛，而不是加速度真的差了 4 倍）。
            # omega 小就说明闭环已经在平衡点上，这正是 theta 收敛的判据。
            # 等不到就硬起跑：摩擦死区大的机器上 omega 可能永远压不到门槛以下，
            # 而"按了 TASK3 没反应"在现场是不可接受的。宁可带着一个不太准的
            # 配平角跑（标定还能把大部分补回来），也不能不跑。
            if self.arm_wait0 is None:
                self.arm_wait0 = now_ms
            forced = now_ms - self.arm_wait0 > CFG["arm_max_ms"]
            steady = (abs(pos_cm) < CFG["arm_pos_cm"]
                      and abs(v_cms) < CFG["arm_v_cms"]
                      and (forced
                           or (PLAN.omega_dps < CFG["arm_omega_dps"]
                               and PLAN.trim_settled < CFG["arm_trim_deg"])))
            if forced and steady and self.arm_since is None:
                print("OL 配平未收敛（等了 {:.1f}s），带着当前配平角起跑"
                      .format((now_ms - self.arm_wait0) / 1000.0))
            if steady:
                if self.arm_since is None:
                    self.arm_since = now_ms
                elif now_ms - self.arm_since >= CFG["arm_ms"]:
                    self._arm_done(pos_cm, now_ms)
            else:
                self.arm_since = None
            if self.state == self.ARM:
                return 0.0

        if self.state == self.READY:
            # 配平好了，等人按第二次 TASK3。这段闭环照跑（目标仍是中心 O），
            # 所以球不会滚走，配平滑窗也一直在刷新 —— 等得越久配平角越准，
            # 等待本身不花任务时间（t0 要到发车那一帧才置）。
            if not self.go_pressed:
                return 0.0
            self._leave_arm(pos_cm, now_ms)

        if self.t0 is not None and not self.overtime:
            if now_ms - self.t0 > CFG["task_timeout_ms"]:
                self.overtime = True

        if PLAN.active:
            return self._step_openloop(pos_cm, v_cms, now_ms)

        if self.state == self.GO_POS:
            # 位置窗 + 速度够小 + 保持一小段：评分量的是**在 +5cm 处**的误差，
            # 高速穿过 +5 不能算"到过"。
            at_pos = (abs(pos_cm - C.TASK_POS_CM) < CFG["arrive_win_cm"]
                      and abs(v_cms) < CFG["arrive_v_cms"])
            if at_pos:
                if self.arrive_since is None:
                    self.arrive_since = now_ms
                elif now_ms - self.arrive_since >= CFG["arrive_ms"]:
                    self.state = self.GO_NEG
            else:
                self.arrive_since = None
            return C.TASK_POS_CM

        if self.state in (self.GO_NEG, self.SETTLE):
            in_win = (abs(pos_cm - C.TASK_NEG_CM) < CFG["settle_pos_cm"]
                      and abs(v_cms) < CFG["settle_v_cms"])
            if in_win:
                if self.settle_since is None:
                    self.settle_since = now_ms
                    self.state = self.SETTLE
                elif now_ms - self.settle_since >= CFG["settle_ms"]:
                    self.state = self.DONE
                    self.total_s = (now_ms - self.t0) / 1000.0
            else:
                self.settle_since = None
                self.state = self.GO_NEG
            return C.TASK_NEG_CM

        return C.TASK_NEG_CM  # DONE：永远锁在 -5cm

    # ---------- 开环走位段 ----------

    def _start_plan(self, pos_cm, now_ms):
        """离开 ARM 的瞬间排表。排不出来就安静地退回闭环，绝不拒跑。"""
        if not CFG["ol_enable"]:
            return
        # 用**实际**起跑位置算第一段距离：ARM 只要求球在中心附近，不是正好 0
        if not PLAN.build(C.TASK_POS_CM - pos_cm,
                          C.TASK_NEG_CM - C.TASK_POS_CM):
            print(PLAN.error, "-> 本次退回闭环")
            return
        self.ol_x0_cm = self._landmark(pos_cm)
        PLAN.start(now_ms)
        print("OL 起跑: θ±{:.1f}°  T1 {:.0f}ms  T2 {:.0f}ms  播表 {:.2f}s"
              .format(PLAN.theta_deg, PLAN.t1_s * 1000.0,
                      PLAN.t2_s * 1000.0, PLAN.total_s))

    def _step_openloop(self, pos_cm, v_cms, now_ms):
        """播表期间的状态推进。返回值只喂给屏幕和 err 显示，控制器不看它。"""
        t = PLAN.clock(now_ms)

        if t < PLAN.move2_start_s:
            # 第一段 + 到位停顿：状态留在 GO+5，峰值统计照常在跑
            return C.TASK_POS_CM

        if self.state != self.GO_NEG:
            # 停顿刚结束 —— 开环全程唯一一次看球。不拿实际落点重算第二段的话，
            # 第一段差多少就原样带到 -5cm，而 -5cm 的误差是直接计分的。
            # 查表带着 ol_lead_ms 的提前量，所以第二段的起步斜坡其实已经播了
            # lead 毫秒了；无所谓 —— 重算只改保持时长 T2，起步斜坡的形状与 T2
            # 无关，而 ol_ramp_ms(200) 远大于 lead(20)，改到的部分还没播到。
            self.ol_x1_cm = self._landmark(pos_cm)
            if PLAN.replan_move2(C.TASK_NEG_CM - self.ol_x1_cm):
                print("OL 重算: +5 落点 {:+.2f}cm  T2 -> {:.0f}ms".format(
                    self.ol_x1_cm, PLAN.t2_s * 1000.0))
            self.state = self.GO_NEG

        if t < PLAN.total_s:
            return C.TASK_NEG_CM

        # 播完，交棒给闭环去守 -5cm（状态留在 GO-5，稳定判定接着原来那套走）
        self.ol_x2_cm = self._landmark(pos_cm)
        PLAN.finish()
        self._report_openloop(v_cms)
        return C.TASK_NEG_CM

    def _report_openloop(self, v_cms):
        """把这一趟反解出的 ol_a1 / ol_a2 打出来 —— 抄回配置就完成标定。

        d = A·K(T)，T 是按设定的 A 解出来的，所以 A_真 = A_设 × d_实测/d_目标，
        不需要再去算 K。推导见 task3_config 第 4.1 节。

        **这里的可信度检查比公式本身重要。** A 是个物理常数，一趟标定给出的修正
        比例应该在 1 附近；一旦这趟跑受了扰动（连丢几帧把摆杆冻在半路、球顶到
        端头、识别离群点），d_实测 就不再是"A 造成的位移"，此时照着抄只会把
        一个好参数改坏。而且抄完下一趟更歪，是个没有阻尼的乘法递推 —— 离线里
        连抄 3 趟能把 ol_a1 从 36 抄成 4.6，然后球直接飞出杆外。

        所以两道闸：方向不对或几乎没走，直接拒绝给建议；比例超过 CAL_MAX_RATIO
        就夹住并标 (夹). 夹住的代价只是标定多跑一趟（2 倍一趟，4 倍两趟），
        换来的是单趟异常绝不会污染参数。
        """
        print("OL 交棒: 位置 {:+.2f}cm  速度 {:+.1f}cm/s".format(
            self.ol_x2_cm, v_cms))
        legs = []
        for name, key, x_from, x_to, target in (
                ("move1", "ol_a1_cms2", self.ol_x0_cm, self.ol_x1_cm,
                 C.TASK_POS_CM),
                ("move2", "ol_a2_cms2", self.ol_x1_cm, self.ol_x2_cm,
                 C.TASK_NEG_CM)):
            if x_from is None or x_to is None:
                continue
            want = target - x_from
            got = x_to - x_from
            if abs(want) < 0.1:
                continue
            legs.append((name, key, want, got, got / want))

        # 两个失败的尝试记在这里，省得以后重走（数据都是 21 工况最坏误差）：
        #
        # 1) 用"两段修正比例反向"识别机械零偏 —— **签名不成立**。两段反向是
        #    正常现象：move2 更快、摩擦占总加速度的比例更小，有效 A 天生就比
        #    move1 大（task3_config 4.1 节）。配平角精确等于真值那趟，标定也是
        #    (36,36) -> (26,51) 一降一升。装上之后正常标定全被拦，0.7 -> 4.07cm。
        # 2) 用"两段建议值比值 > 2 倍"识别 —— 签名**成立**（同倾角同连杆比，
        #    差别只来自摩擦占比，正常收敛是 (39,43)/(31,37)/(21,31)，1.1~1.5 倍），
        #    但拒绝标定之后反而 2.96 -> 7.49cm。见 CAL_MIN_RATIO 上面那段：
        #    错模型的标定仍然在减小误差，拦掉它等于把这份抵消也丢了。
        #
        # 所以这里只做"单趟异常"的防护（下面两道闸），不做"病因诊断"。
        for name, key, want, got, ratio in legs:
            if ratio < CAL_MIN_RATIO:
                # 反向或基本没动：这趟根本没测到 A，多半是丢帧/顶端头/卡球
                print("OL {}: 该走 {:+.2f} 实走 {:+.2f} cm  ->  这趟不可信，"
                      "不给建议（重跑一趟）".format(name, want, got))
                continue
            note = ""
            if ratio > CAL_MAX_RATIO:
                ratio, note = CAL_MAX_RATIO, " (夹)"
            old = CFG[key]
            print("OL {}: 该走 {:+.2f} 实走 {:+.2f} cm  ->  {} {:.1f} -> {:.1f}{}"
                  .format(name, want, got, key, old, old * ratio, note))

    def _track_errors(self, pos_cm):
        if self.state in (self.IDLE, self.ARM):
            return
        if self.peak_pos_cm is None or pos_cm > self.peak_pos_cm:
            self.peak_pos_cm = pos_cm
        if self.state in (self.GO_NEG, self.SETTLE, self.DONE):
            if self.trough_pos_cm is None or pos_cm < self.trough_pos_cm:
                self.trough_pos_cm = pos_cm
        if self.state in (self.SETTLE, self.DONE):
            self.err_settle_cm = max(self.err_settle_cm,
                                     abs(pos_cm - C.TASK_NEG_CM))
