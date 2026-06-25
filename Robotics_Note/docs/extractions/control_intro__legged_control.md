# 抽取留痕：控制导论 / 机器人控制（计算力矩·逆动力学·WBC·足式控制概览）

> **抽取专员说明（务必先读）。** 本文件是项目内部「抽取留痕」，**非成书正文**。目标：把多权威源**全量保真**抽取下来，供综合 agent 写成自包含书章。本章服务于章节《控制导论》，重点覆盖：**反馈与稳定性 / PID / 状态空间与 LQR（Riccati 完整推导）/ MPC / 面向机器人的控制（逆动力学·WBC 概览）/ 足式控制概览**。
>
> **本主题的源情况（重要）。** 个人笔记 `SLAM理论.md` 未同步（见交接 [[project-relocation]]、[[rebuild-orchestration]]），故本抽取**主要据权威教材/原论文/官方教程联网研究**。凡据二手综述重建或本抽取员自行补全（如某些标准 Lyapunov 证明的逐步代数）而原文未逐字给出者，**标 `\rebuilt`（重建·待核对）**，提示综合 agent 与一手教材（Spong、Murray-Li-Sastry、Khalil、Borrelli-Bemporad-Morari）二次核对。
>
> **铁律·禁摘要·全量保真。** 每一步推导（中间代数不跳）、每一道例题/数值例、每一条定义/定理/引理 + 完整证明、每一张表/分类/算法伪码，均完整记录。公式用 LaTeX 写全，标【源出处】。

---

## §0 记号约定与本书统一约定的差异（务必先读）

本抽取跨多源，记号不一致处先在此统一登记，供综合时转换。**本书统一约定**（来自 [[rebuild-orchestration]]）：旋转 $\mathbf R\in\mathrm{SO}(3)$、**右扰动为主**、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）、**Hamilton 四元数**。控制章的记号差异如下：

| 概念 | 本抽取/源记号 | 本书统一约定 | 转换/备注 |
|---|---|---|---|
| 系统状态 | $\mathbf x$（控制界惯例） | $\mathbf x$ | 一致 |
| 控制输入 | $\mathbf u$ | $\mathbf u$ | 一致 |
| LQR 代价权重 | $Q\succeq0$（状态）、$R\succ0$（输入）、$N$（交叉项） | 同 | 与 SLAM 章信息矩阵 $\boldsymbol\Lambda$ 无关，勿混 |
| **Riccati 解矩阵** | 维基/本抽取记 $P$；Tedrake(MIT) 记 $\mathbf S$ | 综合时**统一记 $\mathbf P$**（与 Kalman 章的协方差 $\mathbf P$ **同字母但不同对象**——LQR 的 $\mathbf P$ 是代价-to-go 的 Hessian/值函数权重，Kalman 的 $\mathbf P$ 是状态协方差） | **⚠️ 强烈建议**综合时在 LQR 处明确写"此 $\mathbf P$ 为值函数权重，非协方差"，避免与 ESKF/Kalman 章 $\mathbf P$ 混淆。或控制章用 $\mathbf S$。 |
| LQR 增益 | $K$（连续/无穷时域）、$F_k$ 或 $\mathbf K$（离散） | $\mathbf K$ | 注意符号约定：本抽取多源取 $\mathbf u=-\mathbf K\mathbf x$ |
| 机器人惯性矩阵 | 控制界 $\mathbf M(\mathbf q)$ 或 $\mathbf H(\mathbf q)$；浮基界（Wensing）$\mathbf M(\mathbf q)$ | $\mathbf M(\mathbf q)$ | 一致 |
| 科氏/离心矩阵 | $\mathbf C(\mathbf q,\dot{\mathbf q})$ | $\mathbf C$ | 注意 $\mathbf C$ 也被某些源用作输出矩阵，按上下文区分 |
| 重力项 | Spong/MIT 记 $\boldsymbol\tau_g(\mathbf q)$ 或 $\mathbf g(\mathbf q)$；维基计算力矩记 $\vec\tau_g$ | $\mathbf g(\mathbf q)$ | $\boldsymbol\tau_g=-\partial U/\partial\mathbf q^\top$，**注意有的源把重力放等式右边（外力），有的放左边，符号差一负号**，抄录时务必随源标号 |
| 广义速度（浮基） | Wensing 记 $\boldsymbol\nu=(\boldsymbol\nu_b,\boldsymbol\nu_j)$ | $\boldsymbol\nu$ | 浮基机器人 $\dot{\mathbf q}\neq\boldsymbol\nu$（位形含 $\mathrm{SE}(3)$，速度在李代数），Wensing 显式区分 |
| 接触力 | Wensing 记 $\boldsymbol\lambda$；Cheetah 记 $\mathbf f_i$（地面反力，GRF） | $\boldsymbol\lambda$（接触力）/ $\mathbf f$（GRF） | 同义 |
| 接触雅可比 | $\mathbf J_c(\mathbf q)$ 或 $\mathbf J(\mathbf q)$ | $\mathbf J_c$ | — |
| 选择矩阵（驱动自由度） | $\mathbf S$（Wensing：$\mathbf S=[\mathbf 0\ \mathbf I]$） | $\mathbf S$ | **⚠️ 与 Riccati 的 $\mathbf S$ 同字母**！控制章若用 $\mathbf S$ 表 Riccati，则选择矩阵改记 $\mathbf B_{\mathrm{act}}$ 或保留 $\mathbf S$ 但分节隔离 |
| 误差（PID/伺服） | 维基 PID：$e=\mathrm{SP}-\mathrm{PV}$（设定值减测量）；计算力矩：$\vec\theta_e=\vec\theta_d-\vec\theta$ | $\mathbf e=\mathbf x_d-\mathbf x$ | **符号约定关键**：本抽取统一用"期望减实际"。注意部分稳定性教材用 $\tilde{\mathbf q}=\mathbf q-\mathbf q_d$（实际减期望），证明里 Lyapunov 项符号会随之翻转，综合时统一。 |
| Euler 角（足式） | Cheetah：$\Theta=[\phi\ \theta\ \psi]^\top$（roll-pitch-yaw），Z-Y-X 序，$\mathbf R=\mathbf R_z(\psi)\mathbf R_y(\theta)\mathbf R_x(\phi)$ | 本书旋转主用 $\mathbf R/$李代数；足式 MPC 处保留 Euler 角（源如此） | 综合时注明"足式凸 MPC 用 ZYX Euler 仅为线性化便利，与本书李群主线不冲突" |
| 反对称（叉乘）算子 | Cheetah/Wensing：$[\mathbf x]_\times$；本书：$\mathbf x^\wedge$（$\mathfrak{so}(3)$） | $[\cdot]_\times=(\cdot)^\wedge$ | 同一算子 |

**总览（源清单与可信度）。** 见 §10 参考文献块。核心一手源：① 维基百科多页（LQR/PID/MPC/计算力矩/反馈线性化/代数 Riccati/Lyapunov/LaSalle，作概念与公式骨架，二手但与教材一致）；② **Tedrake, *Underactuated Robotics*（MIT，underactuated.mit.edu）**——LQR 的 HJB→Riccati 推导、多体动力学，一手教程级；③ **Di Carlo, Wensing, Katz, Bledt, Kim, "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control", IROS 2018**——足式凸 MPC + 操作空间控制，一手论文，已取全文 PDF 逐式抄录；④ **Wensing, Posa, Hu, Escande, Mansard, Del Prete, "Optimization-Based Control for Dynamic Legged Robots", IEEE T-RO 2024（arXiv:2211.11644）**——WBC/逆动力学 QP、分层 HQP、质心动力学综述，一手综述，已取全文 PDF 逐式抄录。

---

# 第一部分　反馈与稳定性（控制导论基础）

## §1.1 反馈控制的基本结构【源：维基 Closed-loop controller / 通识 \rebuilt】

**开环 vs 闭环。** 开环控制 $\mathbf u=\mathbf u(t)$ 不用输出反馈；闭环（反馈）控制 $\mathbf u=\mathbf u(\mathbf x,t)$ 用测量的输出/状态修正。标准单回路结构：参考 $r\to$ 误差 $e=r-y\to$ 控制器 $C\to$ 控制量 $u\to$ 被控对象 $P\to$ 输出 $y$，$y$ 经传感器反馈到求和点。

**闭环传递函数（SISO，LTI）\rebuilt。** 设对象 $P(s)$、控制器 $C(s)$、单位负反馈。环路传递函数 $L(s)=C(s)P(s)$。则
$$
\text{参考到输出}:\quad T(s)=\frac{Y(s)}{R(s)}=\frac{L(s)}{1+L(s)}=\frac{CP}{1+CP},
$$
$$
\text{灵敏度函数}:\quad S(s)=\frac{E(s)}{R(s)}=\frac{1}{1+L(s)}=\frac{1}{1+CP},
$$
满足恒等式 $S(s)+T(s)=1$。扰动 $d$（对象输入处）到输出：$Y/D=P/(1+CP)$；测量噪声 $n$ 到输出：$Y/N=-T(s)$。**设计折中**：低频要 $|S|$ 小（跟踪好、抑扰好），高频要 $|T|$ 小（抑噪），而 $S+T=1$ 限制了二者不能同时任意小（Bode 灵敏度积分约束）。

**反馈的作用（定性）【维基 PID】。** 比例项保证基本稳定/跟踪；积分项消除阶跃扰动的稳态误差；微分项提供阻尼、整形瞬态响应。

## §1.2 稳定性概念：Lyapunov 意义下的稳定【源：维基 Lyapunov_stability，逐字】

考虑自治系统 $\dot{\mathbf x}=f(\mathbf x(t))$，$\mathbf x(0)=\mathbf x_0$，平衡点 $f(\mathbf x_e)=\mathbf 0$。

**定义（Lyapunov 稳定）。** 平衡点 $\mathbf x_e$ Lyapunov 稳定，若
$$
\forall\,\epsilon>0,\ \exists\,\delta>0\ \text{s.t.}\ \|\mathbf x(0)-\mathbf x_e\|<\delta\ \Rightarrow\ \forall t\ge0,\ \|\mathbf x(t)-\mathbf x_e\|<\epsilon.
$$

**定义（渐近稳定）。** Lyapunov 稳定 **且** $\exists\,\delta>0$ s.t. $\|\mathbf x(0)-\mathbf x_e\|<\delta\Rightarrow\lim_{t\to\infty}\|\mathbf x(t)-\mathbf x_e\|=0$。

**定义（指数稳定）。** 渐近稳定，且 $\exists\,\alpha>0,\beta>0,\delta>0$ s.t. $\|\mathbf x(0)-\mathbf x_e\|<\delta\Rightarrow$
$$
\|\mathbf x(t)-\mathbf x_e\|\le\alpha\,\|\mathbf x(0)-\mathbf x_e\|\,e^{-\beta t}\qquad\forall t\ge0.
$$

## §1.3 Lyapunov 第二（直接）法【源：维基 Lyapunov_stability，逐字】

**定理（直接法）。** 对 $\dot{\mathbf x}=f(\mathbf x)$，平衡点在 $\mathbf x=\mathbf 0$。若存在 $V:\mathbb R^n\to\mathbb R$ 满足
- $V(\mathbf x)=0\iff\mathbf x=\mathbf 0$；
- $V(\mathbf x)>0\iff\mathbf x\neq\mathbf 0$（正定）；
- $\dot V(\mathbf x)=\nabla V\cdot f(\mathbf x)\le0$，$\forall\mathbf x\neq\mathbf 0$（沿轨迹半负定），

则原点 **Lyapunov 稳定**。若进一步 $\dot V(\mathbf x)<0,\ \forall\mathbf x\neq\mathbf 0$（负定），则原点 **渐近稳定**。若再加 $V$ 径向无界（$V(\mathbf x)\to\infty$ 当 $\|\mathbf x\|\to\infty$），则 **全局渐近稳定**。

**线性系统的 Lyapunov 判据【维基，逐字】。** 对 $\dot{\mathbf x}=A\mathbf x$，$A$ 渐近（指数）稳定 $\iff$ 所有特征值实部为负 $\iff$ 存在正定对称 $M=M^\top\succ0$ 使
$$
A^\top M+MA \ \text{负定（取 } A^\top M+MA=-Q,\ Q\succ0\text{ 即 Lyapunov 方程）},
$$
对应 Lyapunov 函数 $V(\mathbf x)=\mathbf x^\top M\mathbf x$。

> **本抽取员补（Lyapunov 方程求解，\rebuilt）。** 给定 $Q\succ0$，解 $A^\top M+MA=-Q$ 得 $M$；$A$ 稳定时 $M=\int_0^\infty e^{A^\top t}Q\,e^{At}\,dt\succ0$。代入 $\dot V=\mathbf x^\top(A^\top M+MA)\mathbf x=-\mathbf x^\top Q\mathbf x<0$，故渐近稳定。综合时可作 LQR 稳定性证明的引理。

## §1.4 LaSalle 不变性原理【源：维基 LaSalle's_invariance_principle，逐字】

当 $\dot V$ 仅半负定（$\dot V\le0$，非严格负）时，渐近稳定靠 LaSalle 原理判定——这正是机器人 PD+重力补偿证明的关键工具（§7.4）。

**全局版。** 系统 $\dot{\mathbf x}=f(\mathbf x)$，$f(\mathbf 0)=\mathbf 0$。
- **条件 1：** 存在 $C^1$ 函数 $V(\mathbf x)$ 使 $\dot V(\mathbf x)\le0$，$\forall\mathbf x$。
- **一般结论：** 任意轨迹的极限点集（accumulation points）含于 $\mathcal I$，其中 $\mathcal I=$「完全包含于集合 $\{\mathbf x:\dot V(\mathbf x)=0\}$ 内的完整轨迹之并」（即 $\{\dot V=0\}$ 中的**最大不变集**）。
- **条件 2（渐近稳定）：** 另设 $V$ 正定（$V(\mathbf x)>0,\ \forall\mathbf x\neq\mathbf 0$，$V(\mathbf 0)=0$），且 $\mathcal I$ 中除 $\mathbf x(t)\equiv\mathbf 0$（$t\ge0$）外无其他轨迹 $\Rightarrow$ 原点**渐近稳定**。
- **条件 3（全局渐近稳定）：** 再加 $V$ 径向无界（$V(\mathbf x)\to\infty,\ \|\mathbf x\|\to\infty$）$\Rightarrow$ **全局渐近稳定**。

**局部版。** 当 $V$ 正定、$\dot V\le0$ 仅在原点邻域 $D$ 成立，且 $\{\dot V(\mathbf x)=0\}\cap D$ 中除平凡轨迹外无其他轨迹 $\Rightarrow$ 原点**局部渐近稳定**。

## §1.5 控制 Lyapunov 函数（CLF）与 CLF-QP【源：Wensing T-RO §VI-A3，逐字转写】

把任务误差 $\mathbf e_i$ 视为系统输出，设计 CLF $V_i$ 使其稳定到 0。则不等式
$$
\dot V_i\le-\gamma_i V_i
$$
（$\gamma_i$ 设期望指数收敛率）强制 $\mathbf e_i\to0$。这是 **CLF-QP**（[176]）的思想：只要不等式总可行，带收敛保证。但实践中保证持久可行性仍是开放挑战。
（综合提示：CLF-QP 把"$\dot V\le-\gamma V$"作为 QP 的一条线性不等式约束，与 §7 的逆动力学 QP 自然融合。）

---

# 第二部分　PID 控制

## §2.1 PID 控制律：时域与拉氏域【源：维基 PID_controller，逐字】

**时域控制律：**
$$
u(t)=K_p\,e(t)+K_i\int_0^t e(\tau)\,d\tau+K_d\,\frac{de(t)}{dt},
$$
其中 $K_p$ 比例增益，$K_i$ 积分增益，$K_d$ 微分增益，$e(t)=\mathrm{SP}-\mathrm{PV}(t)$（设定值减过程变量）。

**各项分解：** 比例 $P_{\mathrm{out}}=K_p e(t)$；积分 $I_{\mathrm{out}}=K_i\int_0^t e(\tau)d\tau$；微分 $D_{\mathrm{out}}=K_d\,\frac{de(t)}{dt}$。

**拉氏域传递函数：**
$$
L(s)=K_p+\frac{K_i}{s}+K_d s=\frac{K_d s^2+K_p s+K_i}{s}.
$$

**标准（ISA）形式**（$K_p$ 提到三项之外）：
$$
u(t)=K_p\left(e(t)+\frac{1}{T_i}\int_0^t e(\tau)\,d\tau+T_d\,\frac{de(t)}{dt}\right),
$$
参数关系：
$$
T_i=\frac{K_p}{K_i}\ (\text{积分时间常数}),\qquad T_d=\frac{K_d}{K_p}\ (\text{微分时间常数}).
$$

## §2.2 Ziegler–Nichols 整定（临界比例度法）【源：维基 PID_controller / Ziegler–Nichols_method，逐字表】

**方法。** 设 $K_i=K_d=0$，逐步增大 $K_p$ 直到闭环出现**等幅持续振荡**，此时的增益记 $K_u$（极限/临界增益），振荡周期记 $T_u$。然后按下表设增益：

| 控制类型 | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|
| **P** | $0.50\,K_u$ | — | — |
| **PI** | $0.45\,K_u$ | $0.54\,K_u/T_u$ | — |
| **PID** | $0.60\,K_u$ | $1.2\,K_u/T_u$ | $3K_u T_u/40$ |

> **本抽取员核验（\rebuilt）。** 把上表换算为 ISA 形式 $(K_p,T_i,T_d)$：对经典 PID，$T_i=K_p/K_i=0.60K_u/(1.2K_u/T_u)=0.5T_u$；$T_d=K_d/K_p=(3K_uT_u/40)/(0.60K_u)=0.125T_u=T_u/8$。即经典 Z-N PID：$K_p=0.6K_u,\ T_i=0.5T_u,\ T_d=0.125T_u$——与教材经典值一致。

**继电法（relay/Åström-Hägglund）求极限增益【维基，逐字】：**
$$
K_u=\frac{4b}{\pi a},
$$
其中 $a$ 为过程变量振荡幅值，$b$ 为控制输出变化幅值。

**一阶加纯滞后（FOPDT）模型【维基，逐字】：** 传递函数
$$
y(s)=\frac{k_p\,e^{-\theta s}}{\tau_p s+1}\,u(s),
$$
阶跃时域响应
$$
y(t)=k_p\,\Delta u\left(1-e^{\frac{-(t-\theta)}{\tau_p}}\right),
$$
$k_p$ 过程增益，$\tau_p$ 时间常数，$\theta$ 纯滞后。
（综合提示：FOPDT 是 Cohen-Coon、IMC 等整定法的基础模型，可在本书 PID 节作"基于模型整定"的引子。）

## §2.3 微分滤波、微分先行、积分抗饱和【源：维基 PID + Integral_windup，公式部分 \rebuilt】

维基此节多为定性描述，公式由本抽取员据通行教材补全（标 \rebuilt），综合时与 Åström-Murray《Feedback Systems》核对。

**(a) 微分项滤波（real/filtered derivative）。** 纯微分 $K_d s$ 放大高频噪声，实用形式加一阶低通：
$$
D(s)=\frac{K_d s}{1+\tfrac{T_d}{N}s}\quad(N\approx 8\text{–}20),
$$
即微分项带截止 $\omega_c=N/T_d$ 的滤波，限制高频增益为 $K_d N/T_d$。

**(b) 微分先行 / 测量微分（derivative on measurement，避免"derivative kick"）。** 设定值阶跃突变时 $de/dt$ 出现冲激（微分踢）。改为只对测量量 $y$（=PV）微分：
$$
u=K_p e+K_i\!\int e\,d\tau-K_d\frac{dy}{dt}.
$$

**(c) 设定值加权（setpoint weighting，2-DOF PID）。**
$$
u=K_p(b\,r-y)+K_i\!\int(r-y)\,d\tau+K_d\frac{d}{dt}(c\,r-y),\quad b,c\in[0,1].
$$
取 $c=0$ 即微分先行；$b<1$ 减小阶跃超调。

**(d) 积分抗饱和（anti-windup）【维基 Integral_windup，机制逐字 + 公式 \rebuilt】。** 现象：设定值大变化时积分项在上升段累积大量误差，导致超调并继续增大（"unwound"）。常用对策：
- **限幅积分**：把控制输出/积分限制到可行范围；
- **条件积分（clamping）**：当执行器饱和（$u\neq\mathrm{sat}(u)$）且误差使饱和加剧时，停止积分累加；
- **反计算（back-calculation / external reset feedback）**：
$$
\dot I=K_i e+\frac{1}{T_t}\big(\mathrm{sat}(u)-u\big),
$$
其中 $T_t$ 为跟踪时间常数（常取 $T_t\approx\sqrt{T_i T_d}$ 或 $T_t=T_i$），$\mathrm{sat}(u)$ 为实际饱和后输出，差值 $(\mathrm{sat}(u)-u)$ 在饱和时反向泄放积分。

## §2.4 离散（数字）PID：位置式与增量式【源：通行教材 \rebuilt】

采样周期 $T_s$，$e_k=e(kT_s)$。

**位置式（absolute/position form）：**
$$
u_k=K_p e_k+K_i T_s\sum_{j=0}^{k}e_j+\frac{K_d}{T_s}(e_k-e_{k-1}).
$$

**增量式（velocity/incremental form）**（对积分抗饱和、无扰切换更友好）：
$$
\Delta u_k=u_k-u_{k-1}=K_p(e_k-e_{k-1})+K_i T_s\,e_k+\frac{K_d}{T_s}(e_k-2e_{k-1}+e_{k-2}),
$$
$$
u_k=u_{k-1}+\Delta u_k.
$$
（增量式无需显式积分累加项，输出限幅天然抑制 windup。）

## §2.5 PID 闭环稳定性与 Routh–Hurwitz【本抽取员补，\rebuilt】

PID 接在对象上构成闭环特征多项式 $1+L(s)=1+C(s)P(s)=0$。**例（二阶对象）**：$P(s)=\frac{1}{s^2+a_1 s+a_0}$，PD 控制 $C(s)=K_p+K_d s$，闭环
$$
1+\frac{K_p+K_d s}{s^2+a_1 s+a_0}=0\ \Rightarrow\ s^2+(a_1+K_d)s+(a_0+K_p)=0.
$$
二阶 Routh–Hurwitz 稳定 $\iff$ 全系数同号且为正：$a_1+K_d>0$ 且 $a_0+K_p>0$。可见 $K_d$ 增阻尼、$K_p$ 提刚度/频率。加积分变三阶：$C=K_p+K_i/s+K_d s$，特征方程
$$
s^3+(a_1+K_d)s^2+(a_0+K_p)s+K_i=0,
$$
三阶 Routh–Hurwitz 稳定条件：所有系数 $>0$ 且 $(a_1+K_d)(a_0+K_p)>K_i$（即积分增益不能过大，否则失稳——这给出 $K_i$ 的上界，呼应 Z-N 整定）。综合时与控制教材（Ogata/Franklin）核对此 Routh 阵列。

---

# 第三部分　状态空间与 LQR（Riccati 完整推导）

## §3.1 状态空间模型【源：通识 + 维基 LQR】

连续 LTI：$\dot{\mathbf x}=A\mathbf x+B\mathbf u$，$\mathbf y=C\mathbf x+D\mathbf u$。离散 LTI：$\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k$。

**可控性（controllability）。** $(A,B)$ 可控 $\iff$ 可控性矩阵 $\mathcal C=[B\ AB\ A^2B\ \cdots\ A^{n-1}B]$ 满秩（rank $=n$）。
**可镇定（stabilizable）。** $(A,B)$ 可镇定 $\iff$ 所有不可控模态都已稳定（实部 $<0$ / 单位圆内）。
**可观性/可检测性**对偶定义（$(A,C)$）。LQR 无穷时域解的存在唯一性正需可镇定 + 可检测条件（§3.5）。

## §3.2 有限时域连续 LQR：问题与结论【源：维基 LQR，逐字】

**系统：** $\dot{\mathbf x}=A\mathbf x+B\mathbf u$，$t\in[t_0,t_1]$。
**代价（含末端项与交叉项）：**
$$
J=\mathbf x^\top(t_1)F(t_1)\mathbf x(t_1)+\int_{t_0}^{t_1}\!\Big(\mathbf x^\top Q\,\mathbf x+\mathbf u^\top R\,\mathbf u+2\mathbf x^\top N\,\mathbf u\Big)\,dt,
$$
要求 $Q=Q^\top\succeq0$、$R=R^\top\succ0$、$F\succeq0$。

**最优反馈（结论）：**
$$
\mathbf u=-K\mathbf x,\qquad K=R^{-1}\big(B^\top P(t)+N^\top\big).
$$

**连续时间 Riccati 微分方程（RDE）：**
$$
-\dot P(t)=A^\top P(t)+P(t)A-\big(P(t)B+N\big)R^{-1}\big(B^\top P(t)+N^\top\big)+Q,
$$
**末端边界条件** $P(t_1)=F(t_1)$（向后积分）。

## §3.3 ★ LQR 的完整推导（HJB → Riccati）【源：Tedrake, *Underactuated Robotics*, lqr.html，逐字 + 本抽取员补全中间步】

> 这是本章**头号要求**：Riccati 完整推导不跳步。以无交叉项（$N=0$）无穷时域为主线推导，再给有限时域、离散版。

**问题（无穷时域，$N=0$）。** $\dot{\mathbf x}=A\mathbf x+B\mathbf u$，代价
$$
J=\int_0^\infty\!\big(\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u\big)\,dt,\quad Q=Q^\top\succeq\mathbf 0,\ R=R^\top\succ\mathbf 0.
$$

**第 1 步：HJB 方程（动态规划最优性必要条件）【Tedrake 逐字】。** 设最优代价-to-go（值函数）$J^*(\mathbf x)$。无穷时域定常问题的 HJB（Hamilton–Jacobi–Bellman）条件为
$$
0=\min_{\mathbf u}\left[\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+\frac{\partial J^*}{\partial\mathbf x}\big(A\mathbf x+B\mathbf u\big)\right].
$$
> **本抽取员补（HJB 来历，\rebuilt）。** 由 Bellman 最优性原理：$J^*(\mathbf x(t))=\min_{\mathbf u[t,t+dt]}\big[\ell(\mathbf x,\mathbf u)dt+J^*(\mathbf x(t+dt))\big]$，其中 $\ell=\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u$。Taylor 展 $J^*(\mathbf x(t+dt))\approx J^*(\mathbf x)+\frac{\partial J^*}{\partial\mathbf x}\dot{\mathbf x}\,dt$，代入并令 $dt\to0$，对定常无穷时域 $\partial J^*/\partial t=0$，即得上式。连续物理：HJB 与 19 世纪经典力学的 Hamilton–Jacobi 方程同构（Kálmán 首先指出此联系，见维基 HJB）。

**第 2 步：二次型值函数拟设【Tedrake 逐字】。** 设
$$
J^*(\mathbf x)=\mathbf x^\top S\,\mathbf x,\qquad S=S^\top\succeq\mathbf 0,
$$
则
$$
\frac{\partial J^*}{\partial\mathbf x}=2\mathbf x^\top S.
$$
（注：本书统一记此 $S$ 为 $\mathbf P$，见 §0 记号表。）

**第 3 步：对 $\mathbf u$ 求极小（内层最小化）【Tedrake 逐字 + 补全】。** 把 $\partial J^*/\partial\mathbf x=2\mathbf x^\top S$ 代入 HJB 方括号，对 $\mathbf u$ 求梯度置零：
$$
\frac{\partial}{\partial\mathbf u}\Big[\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+2\mathbf x^\top S(A\mathbf x+B\mathbf u)\Big]=2\mathbf u^\top R+2\mathbf x^\top S B=0.
$$
> **本抽取员补（解出 $\mathbf u$ 的代数）。** 转置：$2R\mathbf u+2B^\top S\mathbf x=0\Rightarrow R\mathbf u=-B^\top S\mathbf x$。因 $R\succ0$ 可逆：
$$
\boxed{\ \mathbf u^*=\pi^*(\mathbf x)=-R^{-1}B^\top S\,\mathbf x=-K\mathbf x,\qquad K=R^{-1}B^\top S\ }.
$$
二阶条件：Hessian $\partial^2/\partial\mathbf u^2=2R\succ0$，故确为极小。

**第 4 步：回代 HJB，得代数 Riccati 方程（ARE）【Tedrake 逐字 + 补全】。** 把 $\mathbf u^*=-R^{-1}B^\top S\mathbf x$ 代回 HJB：
$$
0=\mathbf x^\top Q\mathbf x+(\mathbf u^*)^\top R\mathbf u^*+2\mathbf x^\top S(A\mathbf x+B\mathbf u^*).
$$
> **本抽取员补（逐项展开，不跳步）。**
> - $(\mathbf u^*)^\top R\mathbf u^*=(R^{-1}B^\top S\mathbf x)^\top R(R^{-1}B^\top S\mathbf x)=\mathbf x^\top S B R^{-1}\underbrace{R R^{-1}}_{=I}B^\top S\mathbf x=\mathbf x^\top S B R^{-1}B^\top S\mathbf x$（用 $R=R^\top$）。
> - $2\mathbf x^\top S B\mathbf u^*=2\mathbf x^\top S B(-R^{-1}B^\top S\mathbf x)=-2\mathbf x^\top S B R^{-1}B^\top S\mathbf x$。
> - $2\mathbf x^\top S A\mathbf x$：因 $\mathbf x^\top S A\mathbf x$ 是标量，等于其转置 $\mathbf x^\top A^\top S\mathbf x$，故 $2\mathbf x^\top S A\mathbf x=\mathbf x^\top(SA+A^\top S)\mathbf x$（对称化）。
>
> 合并：
> $$
> 0=\mathbf x^\top\Big[Q+SA+A^\top S+\underbrace{(1-2)}_{=-1}SBR^{-1}B^\top S\Big]\mathbf x=\mathbf x^\top\big[Q+SA+A^\top S-SBR^{-1}B^\top S\big]\mathbf x.
> $$
> 对所有 $\mathbf x$ 成立 $\Rightarrow$ 方括号（对称部分）为零：

$$
\boxed{\ \mathbf 0=S A+A^\top S-S B R^{-1}B^\top S+Q\ }\qquad(\text{连续代数 Riccati 方程 CARE}).
$$
**该方程当系统可镇定（且 $(A,Q^{1/2})$ 可检测）时存在唯一正定解 $S\succ\mathbf 0$**【Tedrake 逐字："admits a unique positive-definite solution $S\succ0$ if and only if the system is stabilizable"】。

**第 5 步：闭环稳定性【Tedrake 逐字 + 本抽取员证】。** 最优闭环
$$
\dot{\mathbf x}=(A-BK)\mathbf x.
$$
Tedrake 断言："Since $S\succ0$ and satisfies the ARE, the eigenvalues of $A-BK$ have negative real parts, ensuring asymptotic stability at the origin."
> **本抽取员补（用 $V=\mathbf x^\top S\mathbf x$ 作 Lyapunov 证明，\rebuilt）。** 取 $V=\mathbf x^\top S\mathbf x\succ0$。沿闭环求导：
> $$
> \dot V=\dot{\mathbf x}^\top S\mathbf x+\mathbf x^\top S\dot{\mathbf x}=\mathbf x^\top\big[(A-BK)^\top S+S(A-BK)\big]\mathbf x.
> $$
> 把 $K=R^{-1}B^\top S$ 代入、并用 CARE（$SA+A^\top S=SBR^{-1}B^\top S-Q$）：
> $$
> (A-BK)^\top S+S(A-BK)=A^\top S+SA-K^\top B^\top S-SBK=(SBR^{-1}B^\top S-Q)-2SBR^{-1}B^\top S=-Q-SBR^{-1}B^\top S.
> $$
> 故 $\dot V=-\mathbf x^\top\big(Q+SBR^{-1}B^\top S\big)\mathbf x\le0$。因 $SBR^{-1}B^\top S\succeq0$，且 $Q\succeq0$；当 $(A,Q^{1/2})$ 可检测时 $\dot V$ 沿非平凡轨迹不恒为零，由 LaSalle（§1.4）得**全局渐近稳定**。$\blacksquare$

## §3.4 有限时域连续 LQR 的 RDE 推导【源：Tedrake，逐字 + 补全】

有限时域 $J=h(\mathbf x(t_f))+\int_0^{t_f}\ell(\mathbf x,\mathbf u)dt$，设**含时**值函数 $J^*(\mathbf x,t)=\mathbf x^\top S(t)\mathbf x$。时变 HJB 含 $\partial J^*/\partial t=\mathbf x^\top\dot S(t)\mathbf x$ 项。重复 §3.3 第 3–4 步（此时不令 $\partial_t J^*=0$），得**连续 Riccati 微分方程（RDE）**：
$$
-\dot S(t)=S(t)A+A^\top S(t)-S(t)BR^{-1}B^\top S(t)+Q,
$$
**末端条件** $S(t_f)=Q_f$（即末端权重 $F$）。最优**时变**控制：
$$
\mathbf u^*=-R^{-1}B^\top S(t)\,\mathbf x.
$$
（向后积分 RDE 从 $t_f$ 到 $0$；$t_f\to\infty$ 时 $S(t)\to S_\infty=$ CARE 的稳定解。）

## §3.5 无穷时域连续 LQR 与 CARE（含交叉项一般式）【源：维基 LQR，逐字】

**代价：** $J=\int_0^\infty(\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+2\mathbf x^\top N\mathbf u)\,dt$。

**CARE（含 $N$）：**
$$
A^\top P+PA-(PB+N)R^{-1}(B^\top P+N^\top)+Q=\mathbf 0.
$$
**最优定常增益：** $K=R^{-1}(B^\top P+N^\top)$，$\mathbf u=-K\mathbf x$。

**等价形式（消去交叉项）【维基逐字】：**
$$
\mathcal A^\top P+P\mathcal A-PBR^{-1}B^\top P+\mathcal Q=\mathbf 0,
$$
其中
$$
\mathcal A=A-BR^{-1}N^\top,\qquad \mathcal Q=Q-NR^{-1}N^\top.
$$
（即作变量替换 $\mathbf u\to\mathbf u+R^{-1}N^\top\mathbf x$ 把交叉项吸收。）

## §3.6 代数 Riccati 方程的求解：Hamilton 矩阵法【源：维基 Algebraic_Riccati_equation，逐字】

**CARE 的精炼形式（$N=0$）：** $A^\top P+PA-PBR^{-1}B^\top P+Q=0$，$P$ 为未知 $n\times n$ 对称阵，$Q,R$ 对称。

**Hamilton 矩阵解法。** 定义 $2n\times2n$ Hamilton 矩阵
$$
Z=\begin{pmatrix}A & -BR^{-1}B^\top\\ -Q & -A^\top\end{pmatrix}.
$$
若 $Z$ 无虚轴特征值，则恰有一半特征值实部为负。设其稳定特征子空间的 $2n\times n$ 基为 $\begin{pmatrix}U_{11}\\ U_{21}\end{pmatrix}$，则
$$
\boxed{\ P=U_{21}U_{11}^{-1}\ }.
$$
且**闭环极点** = $A-BR^{-1}B^\top P$ 的特征值 = $Z$ 的负实部特征值【维基逐字】。

**DARE 的辛矩阵（symplectic）解法**（$A$ 可逆时）：
$$
Z=\begin{pmatrix}A+BR^{-1}B^\top(A^{-1})^\top Q & -BR^{-1}B^\top(A^{-1})^\top\\ -(A^{-1})^\top Q & (A^{-1})^\top\end{pmatrix},
$$
同样 $P=U_{21}U_{11}^{-1}$，闭环极点 = $Z$ 在单位圆内的特征值。

## §3.7 离散时间 LQR（有限与无穷时域）【源：维基 LQR + Tedrake，逐字】

**系统：** $\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k$。
**有限时域代价（含 $N$）：**
$$
J=\mathbf x_{H_p}^\top Q_{H_p}\mathbf x_{H_p}+\sum_{k=0}^{H_p-1}\big(\mathbf x_k^\top Q\mathbf x_k+\mathbf u_k^\top R\mathbf u_k+2\mathbf x_k^\top N\mathbf u_k\big).
$$
**离散 Riccati 差分方程（DRE，向后递推）：**
$$
P_{k-1}=A^\top P_k A-(A^\top P_k B+N)(R+B^\top P_k B)^{-1}(B^\top P_k A+N^\top)+Q,
$$
**末端** $P_{H_p}=Q_{H_p}$。
**最优增益与控制：**
$$
F_k=(R+B^\top P_{k+1}B)^{-1}(B^\top P_{k+1}A+N^\top),\qquad \mathbf u_k=-F_k\mathbf x_k.
$$

**无穷时域离散（DARE）【维基逐字】：**
$$
P=A^\top PA-(A^\top PB+N)(R+B^\top PB)^{-1}(B^\top PA+N^\top)+Q,
$$
$$
F=(R+B^\top PB)^{-1}(B^\top PA+N^\top),\qquad \mathbf u_k=-F\mathbf x_k.
$$

**Tedrake 版（$N=0$）DRE【逐字】：**
$$
S[n-1]=Q+A^\top S[n]A-\big(A^\top S[n]B\big)\big(R+B^\top S[n]B\big)^{-1}\big(B^\top S[n]A\big),\quad S[N]=\mathbf 0,
$$
无穷时域 $S=Q+A^\top SA-(A^\top SB)(R+B^\top SB)^{-1}(B^\top SA)$，$\mathbf K=(R+B^\top SB)^{-1}B^\top SA$，$\mathbf u[n]=-\mathbf K\mathbf x[n]$。

> **本抽取员补（离散 LQR 的动态规划一步推导，\rebuilt，供综合作完整证）。** 设 $J_k^*(\mathbf x)=\mathbf x^\top S_k\mathbf x$。Bellman 递归
> $$
> J_k^*(\mathbf x)=\min_{\mathbf u}\Big[\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+\underbrace{(A\mathbf x+B\mathbf u)^\top S_{k+1}(A\mathbf x+B\mathbf u)}_{J_{k+1}^*(\mathbf x_{k+1})}\Big].
> $$
> 对 $\mathbf u$ 求导置零：$2R\mathbf u+2B^\top S_{k+1}(A\mathbf x+B\mathbf u)=0\Rightarrow(R+B^\top S_{k+1}B)\mathbf u=-B^\top S_{k+1}A\mathbf x$，故 $\mathbf u^*=-(R+B^\top S_{k+1}B)^{-1}B^\top S_{k+1}A\,\mathbf x=-\mathbf K_k\mathbf x$（即上 $F_k$，$N=0$）。回代得 $S_k=Q+A^\top S_{k+1}A-A^\top S_{k+1}B(R+B^\top S_{k+1}B)^{-1}B^\top S_{k+1}A$——正是上 DRE。$\blacksquare$

## §3.8 LQE/LQR 对偶与 LQG【源：维基 LQR / 矩阵差分方程，逐字】

两个矩阵 Riccati 方程结构相似——一个**前向**（时间增），一个**后向**（时间减），此相似称**对偶（duality）**：前向解 **LQE**（线性二次估计，即 Kalman 滤波器增益的 Riccati），后向解 **LQR**。把"最优状态估计（Kalman/LQE）+ 最优状态反馈（LQR）"合并即 **LQG（线性二次高斯）控制**（分离原理：估计与控制可分开设计）。
（综合提示：本书 Kalman/ESKF 章的协方差预测-更新 Riccati 与本章 LQR 的 Riccati 是对偶物，可作"估计↔控制对偶"的呼应小节；但 §0 已警示 $\mathbf P$ 字母双关，务必分清"协方差 $\mathbf P$"与"值函数权重 $\mathbf P/S$"。）

---

# 第四部分　模型预测控制（MPC）

## §4.1 MPC 概念与滚动时域原理【源：维基 Model_predictive_control，逐字】

MPC 是一种"在满足约束集合的前提下控制过程"的先进控制方法。**滚动时域（receding horizon）**：
> 在时刻 $t$ 采样当前对象状态，求解一个在有限未来时域 $[t,t+T]$ 上的代价极小化控制策略；**只施加第一步控制**；下一采样从新状态重新求解；预测时域不断前移——故名"滚动时域控制"。

**代价函数（维基示例形式，逐字）：**
$$
J=\sum_{i=1}^{N}w_{x_i}(r_i-x_i)^2+\sum_{i=1}^{M}w_{u_i}(\Delta u_i)^2,
$$
$x_i$ 被控变量、$r_i$ 参考、$u_i$ 操纵变量、$w_{x_i}$ 跟踪权、$w_{u_i}$ 罚控制增量 $\Delta u_i$ 的权——且优化"不违反约束（上下限）"。

**与 LQR 的关系【维基逐字】：** "LQR optimizes across the entire time window (horizon) whereas MPC optimizes in a receding time window." LQR 给整个时域单一解；MPC 在较短窗口频繁重算，可能次优但满足约束；MPC 能处理非线性与硬约束，LQR 假设线性。**当 LQR 在滚动时域下反复运行，即成为一种 MPC**。

**算法步骤（维基隐式，逐字整理）：**
1. 时刻 $t$ 采样当前对象状态；
2. 用数值算法构造代价极小化；
3. 解从 $t$ 到 $t+T$ 的轨迹（一般经 Euler–Lagrange / 数值优化）；
4. 提取并施加第一步控制；
5. 推进到下一步，回到 1。

## §4.2 线性 MPC 的凝聚（condensed/dense）QP 形式【源：MIT Cheetah IROS 2018 §IV-D，逐字 + 通行推导补全】

> 维基未给批量矩阵（凝聚）形式；本节用 **Cheetah 凸 MPC 论文**给出的标准凝聚 QP（足式落地，但形式通用），并补全预测方程代入的中间代数（\rebuilt）。

**标准式 MPC 问题（horizon $k$）【Cheetah 式 (18)–(21)，逐字】：**
$$
\min_{\mathbf x,\mathbf u}\ \sum_{i=0}^{k-1}\|\mathbf x_{i+1}-\mathbf x_{i+1,\mathrm{ref}}\|_{Q_i}+\|\mathbf u_i\|_{R_i}\tag{18}
$$
$$
\text{s.t.}\quad \mathbf x_{i+1}=A_i\mathbf x_i+B_i\mathbf u_i,\quad i=0\ldots k-1\tag{19}
$$
$$
\underline{\mathbf c}_i\le C_i\mathbf u_i\le\overline{\mathbf c}_i,\quad i=0\ldots k-1\tag{20}
$$
$$
D_i\mathbf u_i=\mathbf 0,\quad i=0\ldots k-1\tag{21}
$$
其中 $\mathbf x_i$ 为第 $i$ 步状态，$\mathbf u_i$ 为控制，$Q_i,R_i$ 对角半正定权重，$A_i,B_i$ 离散动力学，$C_i,\underline{\mathbf c}_i,\overline{\mathbf c}_i$ 给输入不等式约束，$D_i$ 选出"摆动腿（不接触）对应的力"以置零；$\|\mathbf a\|_S\equiv\mathbf a^\top S\mathbf a$（加权范数）。

**凝聚（condensed）为只含 $\mathbf u$ 的稠密 QP【本抽取员补全预测代入，\rebuilt】。** 把 (19) 递推展开（时变 $A_i,B_i$，初值 $\mathbf x_0$ 已知）：
$$
\mathbf X=\mathbf A_{qp}\,\mathbf x_0+\mathbf B_{qp}\,\mathbf U,\qquad
\mathbf X=\begin{bmatrix}\mathbf x_1\\\vdots\\\mathbf x_k\end{bmatrix},\ \mathbf U=\begin{bmatrix}\mathbf u_0\\\vdots\\\mathbf u_{k-1}\end{bmatrix},
$$
其中（以定常 $A,B$ 为例展示结构）
$$
\mathbf A_{qp}=\begin{bmatrix}A\\A^2\\\vdots\\A^{k}\end{bmatrix},\quad
\mathbf B_{qp}=\begin{bmatrix}B&0&\cdots&0\\AB&B&\cdots&0\\\vdots&&\ddots&\\A^{k-1}B&A^{k-2}B&\cdots&B\end{bmatrix}.
$$
代价写为 $\|\mathbf X-\mathbf X_{\mathrm{ref}}\|_{\mathbf L}^2+\|\mathbf U\|_{\mathbf K}^2$（$\mathbf L=\mathrm{blkdiag}(Q_i)$，$\mathbf K=\mathrm{blkdiag}(R_i)$）。代入 $\mathbf X$、令 $\mathbf y=\mathbf X_{\mathrm{ref}}$，展开（去掉与 $\mathbf U$ 无关常数）得**稠密 QP**【Cheetah 式 (29)–(32)，逐字】：
$$
\min_{\mathbf U}\ \tfrac12\mathbf U^\top H\mathbf U+\mathbf U^\top\mathbf g\tag{29}
$$
$$
\text{s.t.}\quad \underline{\mathbf c}\le C\mathbf U\le\overline{\mathbf c}\tag{30}
$$
$$
H=2\big(\mathbf B_{qp}^\top\mathbf L\,\mathbf B_{qp}+\mathbf K\big)\tag{31}
$$
$$
\mathbf g=2\,\mathbf B_{qp}^\top\mathbf L\big(\mathbf A_{qp}\mathbf x_0-\mathbf y\big)\tag{32}
$$
$C$ 为约束矩阵。Cheetah 注：足式取 $\mathbf K=\alpha\mathbf 1_{3nk}$（等权罚力）；$H\in\mathbb R^{3nk\times3nk}$、$\mathbf g\in\mathbb R^{3nk}$ 的维度只依赖足数 $n$ 与时域 $k$，**与状态数无关**（凝聚把状态消去）——这是把动力学约束 (19) 移入代价、删掉状态轨迹变量后的关键收益。

> **本抽取员补全 (31)–(32) 的来历（\rebuilt，不跳步）。** 代入 $\mathbf X=\mathbf A_{qp}\mathbf x_0+\mathbf B_{qp}\mathbf U$：
> $$
> \|\mathbf X-\mathbf y\|_{\mathbf L}^2=(\mathbf A_{qp}\mathbf x_0+\mathbf B_{qp}\mathbf U-\mathbf y)^\top\mathbf L(\mathbf A_{qp}\mathbf x_0+\mathbf B_{qp}\mathbf U-\mathbf y).
> $$
> 展开关于 $\mathbf U$ 的二次项 $\mathbf U^\top\mathbf B_{qp}^\top\mathbf L\mathbf B_{qp}\mathbf U$ 与一次项 $2\mathbf U^\top\mathbf B_{qp}^\top\mathbf L(\mathbf A_{qp}\mathbf x_0-\mathbf y)$；加上 $\|\mathbf U\|_{\mathbf K}^2=\mathbf U^\top\mathbf K\mathbf U$。配成 $\tfrac12\mathbf U^\top H\mathbf U+\mathbf U^\top\mathbf g$ 即得 $H=2(\mathbf B_{qp}^\top\mathbf L\mathbf B_{qp}+\mathbf K)$、$\mathbf g=2\mathbf B_{qp}^\top\mathbf L(\mathbf A_{qp}\mathbf x_0-\mathbf y)$。$\blacksquare$

**复杂度【Cheetah 逐字】：** 不利用稀疏的求解器对式 (18) 有"关于时域长度、状态数、约束数的立方时间复杂度"；把动力学约束与状态轨迹移入代价（凝聚）显著加速。Cheetah 用开源 QP 解器（ECOS [23]、qpOASES）在 $<1$ ms 解出（20–30 Hz）。

## §4.3 MPC 的稳定性：终端代价与终端约束【源：维基 MPC + 通行 MPC 理论 \rebuilt】

维基对终端代价/稳定仅定性提及。通行（Mayne 等）线性 MPC 稳定性框架（标 \rebuilt，综合时与 Borrelli-Bemporad-Morari《Predictive Control》或 Rawlings-Mayne-Diehl 核对）：

带**终端代价** $V_f(\mathbf x_N)=\mathbf x_N^\top P_f\mathbf x_N$ 与**终端约束** $\mathbf x_N\in\mathcal X_f$ 的有限时域问题
$$
\min_{\mathbf u_{0:N-1}}\ \sum_{i=0}^{N-1}\big(\mathbf x_i^\top Q\mathbf x_i+\mathbf u_i^\top R\mathbf u_i\big)+\mathbf x_N^\top P_f\mathbf x_N
$$
s.t. 动力学、状态/输入约束、$\mathbf x_N\in\mathcal X_f$。**渐近稳定的充分条件（标准四条件，\rebuilt）：**
1. $\mathcal X_f$ 是**控制不变集**（存在局部反馈 $\kappa_f$ 使 $\mathbf x\in\mathcal X_f\Rightarrow A\mathbf x+B\kappa_f(\mathbf x)\in\mathcal X_f$ 且满足约束）；
2. 终端代价 $V_f$ 在 $\mathcal X_f$ 上是该局部反馈的**控制 Lyapunov 函数**，满足 $V_f(A\mathbf x+B\kappa_f(\mathbf x))-V_f(\mathbf x)\le-(\mathbf x^\top Q\mathbf x+\kappa_f^\top R\kappa_f)$；
3. $\mathcal X_f\subseteq$ 状态约束集；$\kappa_f(\mathcal X_f)\subseteq$ 输入约束集。

满足时，最优代价 $J^*(\mathbf x)$ 是闭环 Lyapunov 函数（$J^*(\mathbf x^+)-J^*(\mathbf x)\le-\mathbf x^\top Q\mathbf x$），保证递归可行 + 渐近稳定。**常用取法**：$P_f=$ 局部 LQR 的 Riccati 解（CARE/DARE 的 $P$），$\kappa_f=-K_{\mathrm{LQR}}\mathbf x$，$\mathcal X_f$ 取该 LQR 的（约束容许）不变椭球。这把 §3 的 LQR 与本节 MPC 直接打通：**MPC = 带约束的滚动时域 LQR，终端用无约束 LQR"收尾"以保稳定**。

---

# 第五部分　面向机器人的控制：刚体动力学与逆动力学/计算力矩

## §5.1 操纵臂动力学方程（Euler–Lagrange）【源：Tedrake multibody.html + 维基，逐字】

**Lagrange 方程**（保守系统，广义力 $\boldsymbol\tau$）：$\frac{d}{dt}\frac{\partial L}{\partial\dot{\mathbf q}}-\frac{\partial L}{\partial\mathbf q}=\boldsymbol\tau$，$L=K-U$（动能减势能）。整理得 $n$ 连杆机器人的**操纵臂方程（manipulator equation）**【Tedrake 逐字】：
$$
\boxed{\ \mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}=\boldsymbol\tau_g(\mathbf q)+\mathbf B\mathbf u+\sum\mathbf J^\top\mathbf F\ }
$$
$\mathbf M(\mathbf q)$ 惯性（质量）矩阵；$\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}$ 科氏/离心力；$\boldsymbol\tau_g(\mathbf q)$ 重力广义力；$\mathbf B$ 输入映射；$\mathbf u$ 驱动；$\sum\mathbf J^\top\mathbf F$ 外接触力的广义力。

**重力项【Tedrake 逐字】：** $\boldsymbol\tau_g(\mathbf q)=-\dfrac{\partial U(\mathbf q)}{\partial\mathbf q}^\top$，$U$ 为势能。

## §5.2 科氏矩阵与第一类 Christoffel 符号【源：Tedrake / 维基，逐字】

科氏矩阵由质量矩阵的**第一类 Christoffel 符号**定义【Tedrake/维基逐字】：
$$
\big[\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}\big]_i=\dot{\mathbf q}^\top\bar{\mathbf C}_i\,\dot{\mathbf q},\qquad
\bar C_{i;j,k}(\mathbf q)=\frac12\left[\frac{\partial M_{ij}}{\partial q_k}+\frac{\partial M_{ik}}{\partial q_j}-\frac{\partial M_{jk}}{\partial q_i}\right].
$$
等价分量式（维基 Property 3，逐字）：
$$
\sum_i d_{k,i}(\mathbf q)\,\ddot q_i+\sum_{i,j}c_{i,j,k}(\mathbf q)\,\dot q_i\dot q_j+\Phi_k(\mathbf q)=\tau_k,
$$
$c_{i,j,k}$ 即 Christoffel 符号（关节速度间的耦合项），$\Phi_k$ 即重力项分量。

## §5.3 ★ 关键结构性质（综合时作为引理，控制证明都靠它）【源：维基 + Tedrake，逐字 + 本抽取员补证】

**性质 1（$\mathbf M$ 对称正定）【维基逐字】。** $\mathbf M(\mathbf q)=\mathbf M(\mathbf q)^\top\succ0$ 且一致正定：$\sigma_{\min}\le\sigma(\mathbf M(\mathbf q))\le\sigma_{\max}$（特征值有正的上下界）。

**性质 2（斜对称/无源性）【Tedrake/维基逐字】。** 对适当选取的 $\mathbf C$，
$$
\mathbf N(\mathbf q,\dot{\mathbf q})\equiv\dot{\mathbf M}(\mathbf q)-2\mathbf C(\mathbf q,\dot{\mathbf q})\ \text{是斜对称矩阵}\ (\mathbf N=-\mathbf N^\top),
$$
即 $\forall\,\mathbf z:\ \mathbf z^\top\big(\dot{\mathbf M}-2\mathbf C\big)\mathbf z=0$。
> **本抽取员补（无源性物理意义与能量恒等式证，\rebuilt）。** 系统动能 $K=\tfrac12\dot{\mathbf q}^\top\mathbf M(\mathbf q)\dot{\mathbf q}$。其对时间导
> $$
> \dot K=\dot{\mathbf q}^\top\mathbf M\ddot{\mathbf q}+\tfrac12\dot{\mathbf q}^\top\dot{\mathbf M}\dot{\mathbf q}.
> $$
> 由运动方程（无外力、记重力在左）$\mathbf M\ddot{\mathbf q}=\boldsymbol\tau-\mathbf C\dot{\mathbf q}-\mathbf g$，代入：
> $$
> \dot K=\dot{\mathbf q}^\top\boldsymbol\tau-\dot{\mathbf q}^\top\mathbf C\dot{\mathbf q}-\dot{\mathbf q}^\top\mathbf g+\tfrac12\dot{\mathbf q}^\top\dot{\mathbf M}\dot{\mathbf q}=\dot{\mathbf q}^\top\boldsymbol\tau-\dot{\mathbf q}^\top\mathbf g+\tfrac12\dot{\mathbf q}^\top(\dot{\mathbf M}-2\mathbf C)\dot{\mathbf q}.
> $$
> 物理上"内力（惯性+科氏/离心）不产生也不耗散能量"$\Rightarrow$ 末项为零 $\Rightarrow\dot{\mathbf q}^\top(\dot{\mathbf M}-2\mathbf C)\dot{\mathbf q}=0,\ \forall\dot{\mathbf q}\Rightarrow\dot{\mathbf M}-2\mathbf C$ 斜对称。$\blacksquare$ 这正是 §7.4 PD+重力补偿 Lyapunov 证明里"科氏项不进 $\dot V$"的根据。

**性质 3（线性可参数化）\rebuilt。** 动力学对惯性参数 $\boldsymbol\Theta$ 线性：$\mathbf M\ddot{\mathbf q}+\mathbf C\dot{\mathbf q}+\mathbf g=\mathbf Y(\mathbf q,\dot{\mathbf q},\ddot{\mathbf q})\boldsymbol\Theta$，$\mathbf Y$ 为回归矩阵——自适应控制基础（综合可作补充）。

## §5.4 逆动力学（inverse dynamics）【源：维基 Inverse_dynamics（概念）+ 通识】

**概念【维基逐字】。** 逆（刚体）动力学是"由刚体的运动学与惯性属性计算所需力/力矩"的方法；机器人中"用于计算机器人电机为使末端按规定方式运动所需的力矩"。正动力学（已知力矩求运动）的反问题。

**逆动力学公式（综合补，\rebuilt）。** 给定期望 $(\mathbf q,\dot{\mathbf q},\ddot{\mathbf q})$，所需力矩
$$
\boldsymbol\tau=\mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\mathbf g(\mathbf q).
$$
**递归 Newton–Euler 算法（RNEA）**以 $O(n)$ 复杂度数值实现此式（前向递推算速度/加速度，后向递推算力/力矩）——综合时可补 RNEA 伪码（本抽取未取到逐行原文，标 \rebuilt）。

## §5.5 ★ 计算力矩控制（computed torque control）/ 逆动力学控制【源：维基 Computed_torque_control，逐字】

**机器人动力学（维基记号，重力在左）：**
$$
\mathbf M(\vec\theta)\ddot{\vec\theta}+\mathbf C(\vec\theta,\dot{\vec\theta})\dot{\vec\theta}+\vec\tau_g(\vec\theta)=\vec\tau,
$$
$\vec\theta\in\mathbb R^N$ 关节角；$\mathbf M$ 惯性阵；$\mathbf C\dot{\vec\theta}$ 科氏/离心力；$\vec\tau_g$ 重力力矩；$\vec\tau$ 关节力矩输入。

**近似模型【维基逐字】。** 用近似 $(\tilde{\mathbf M},\tilde{\mathbf C},\tilde{\vec\tau}_g)$，满足 $\mathbf M(\vec\theta)^{-1}\tilde{\mathbf M}(\vec\theta)\approx\mathbf 1$（即近似惯性阵足够准）。

**计算力矩控制律【维基逐字】：**
$$
\boxed{\ \vec\tau(t)=\tilde{\mathbf M}(\vec\theta)\Big[\ddot{\vec\theta}_d(t)+K_p\vec\theta_e(t)+K_i\!\int_0^t\!\ddot{\vec\theta}_e(t')\,dt'+K_d\dot{\vec\theta}_e(t)\Big]+\tilde{\mathbf C}(\vec\theta,\dot{\vec\theta})+\tilde{\vec\tau}_g(\vec\theta)\ }
$$
误差定义 $\vec\theta_e(t)=\vec\theta_d(t)-\vec\theta(t)$；$K_p,K_i,K_d$ 为 PID 增益（用标准线性控制法整定）。
> （注：维基原式 $\tilde{\mathbf C}$、$\tilde{\vec\tau}_g$ 应理解为 $\tilde{\mathbf C}\dot{\vec\theta}+\tilde{\vec\tau}_g$ 即科氏与重力补偿前馈；维基略去了 $\dot{\vec\theta}$ 因子，综合时补回 $\tilde{\mathbf C}(\vec\theta,\dot{\vec\theta})\dot{\vec\theta}$。标 \rebuilt 提示核对。）

**闭环误差动力学【维基逐字 + 本抽取员补证】。** 当近似精确（$\tilde{\mathbf M}=\mathbf M$ 等），代入得线性误差方程：
$$
\boxed{\ \ddot{\vec\theta}_e+K_d\dot{\vec\theta}_e+K_p\vec\theta_e+K_i\!\int_0^t\!\ddot{\vec\theta}_e\,dt'=\mathbf 0\ }
$$
> **本抽取员补（误差动力学推导，不跳步，\rebuilt）。** 取 $\tilde{\mathbf M}=\mathbf M,\tilde{\mathbf C}=\mathbf C,\tilde{\vec\tau}_g=\vec\tau_g$。把控制律代入动力学 $\mathbf M\ddot{\vec\theta}+\mathbf C\dot{\vec\theta}+\vec\tau_g=\vec\tau$：
> $$
> \mathbf M\ddot{\vec\theta}+\mathbf C\dot{\vec\theta}+\vec\tau_g=\mathbf M\big[\ddot{\vec\theta}_d+K_p\vec\theta_e+K_i\!\int\ddot{\vec\theta}_e+K_d\dot{\vec\theta}_e\big]+\mathbf C\dot{\vec\theta}+\vec\tau_g.
> $$
> 两边消去 $\mathbf C\dot{\vec\theta}+\vec\tau_g$，再左乘 $\mathbf M^{-1}$（$\mathbf M\succ0$ 可逆）：$\ddot{\vec\theta}=\ddot{\vec\theta}_d+K_p\vec\theta_e+K_i\!\int\ddot{\vec\theta}_e+K_d\dot{\vec\theta}_e$。移项、用 $\vec\theta_e=\vec\theta_d-\vec\theta\Rightarrow\ddot{\vec\theta}_e=\ddot{\vec\theta}_d-\ddot{\vec\theta}$：
> $$
> \ddot{\vec\theta}_d-\ddot{\vec\theta}=-\big(K_p\vec\theta_e+K_i\!\int\ddot{\vec\theta}_e+K_d\dot{\vec\theta}_e\big)\Rightarrow\ddot{\vec\theta}_e+K_d\dot{\vec\theta}_e+K_p\vec\theta_e+K_i\!\int\ddot{\vec\theta}_e=\mathbf 0.\ \blacksquare
> $$
> **维基原话**："converts the nonlinear robot control problem into a relatively simple linear control problem." 即**前馈逆动力学项把非线性精确抵消**（feedback linearization），剩下解耦的线性误差动力学，再用 PD/PID 配极点（取 $K_p,K_d$ 使每个关节 $\ddot e+K_d\dot e+K_p e=0$ 临界阻尼 $K_d=2\sqrt{K_p}$）。

**与反馈线性化的关系。** 计算力矩控制是机器人版的**精确反馈线性化**：见 §6。

---

# 第六部分　反馈线性化（计算力矩的理论基础）

## §6.1 SISO 输入-输出反馈线性化【源：维基 Feedback_linearization，逐字】

**系统：** $\dot x=f(x)+\sum_{i=1}^m g_i(x)u_i$，SISO 时 $u\in\mathbb R,y\in\mathbb R$，输出 $y=h(x)$。

**Lie 导数：**
$$
L_f h(x)\triangleq\frac{\partial h(x)}{\partial x}f(x),\qquad L_g h(x)\triangleq\frac{\partial h(x)}{\partial x}g(x),
$$
高阶：$L_f^2 h=L_f L_f h=\frac{\partial(L_f h)}{\partial x}f$，$L_gL_f h=\frac{\partial(L_f h)}{\partial x}g$。

**相对阶（relative degree）$r$。** 在 $x_0$ 处相对阶为 $r$，若 $L_gL_f^k h(x)=0,\ \forall k\le r-2$（$x_0$ 邻域内）且 $L_gL_f^{r-1}h(x_0)\neq0$。即"输出需微分多少次输入才显式出现"。

**输出逐次微分（当 $r=n$）：**
$$
y=h(x),\ \dot y=L_f h,\ \ddot y=L_f^2 h,\ \ldots,\ y^{(n-1)}=L_f^{n-1}h,\ y^{(n)}=L_f^n h+L_gL_f^{n-1}h\cdot u.
$$

**坐标变换（微分同胚）：**
$$
z=T(x)=\big[h(x),\ L_f h(x),\ \ldots,\ L_f^{n-1}h(x)\big]^\top.
$$

**反馈线性化控制律：**
$$
\boxed{\ u=\frac{1}{L_gL_f^{n-1}h(x)}\big(-L_f^n h(x)+v\big)\ }
$$
$v$ 为新外部输入。**结果**：积分器链
$$
\dot z_1=z_2,\ \dot z_2=z_3,\ \ldots,\ \dot z_n=v,\qquad\text{即}\ \dot z=Az+bv\ (\text{线性能控标准型}).
$$

## §6.2 MIMO 扩展与解耦矩阵【源：维基，逐字】

$m$ 输入 $m$ 输出，定义**解耦矩阵**
$$
A=\begin{bmatrix}
L_{g_1}L_f^{r_1-1}h_1 & \cdots & L_{g_m}L_f^{r_1-1}h_1\\
\vdots & \ddots & \vdots\\
L_{g_1}L_f^{r_m-1}h_m & \cdots & L_{g_m}L_f^{r_m-1}h_m
\end{bmatrix},
$$
线性化控制律 $\mathbf u=A^{-1}(\mathbf v-\mathbf b)$，$\mathbf b=[L_f^{r_1}h_1,\ldots,L_f^{r_m}h_m]^\top$。

**输入-状态线性化条件（综合补，\rebuilt）。** 全状态线性化要求分布 $\{g,\mathrm{ad}_f g,\ldots,\mathrm{ad}_f^{n-1}g\}$ **可控（满秩）且对合（involutive）**（Frobenius 定理）。机器人操纵臂方程因 $\mathbf M$ 可逆而恰好满足，故计算力矩控制（§5.5）即其特例：取 $\mathbf v=\ddot{\vec\theta}_d-K_d\dot{\vec\theta}_e-K_p\vec\theta_e$，$\mathbf u=\boldsymbol\tau$，解耦矩阵即 $\mathbf M$。

---

# 第七部分　基于无源性/PD 的机器人控制与稳定性证明

## §7.1 PD + 重力补偿控制律【源：维基搜索（Takegaki-Arimoto, Springer "PD Control with Gravity Compensation"）+ 本抽取员补证】

**控制律（点到点定位）。** 目标位形 $\mathbf q_d$（常值），误差 $\tilde{\mathbf q}=\mathbf q_d-\mathbf q$。控制律【综述逐字】：
$$
\boldsymbol\tau=\mathbf g(\mathbf q)+K_p\tilde{\mathbf q}-K_d\dot{\mathbf q},\qquad K_p,K_d\succ0,
$$
（即"无源性静态非线性控制器 / PD plus gravity compensation"，搜索原文给的等价写法 $u=G(q)-K_p q-K_d\dot q$ 是把 $\tilde q$ 展开、定点取 $q_d=0$ 的特例。）

**历史里程碑【综述逐字】。** Takegaki 与 Arimoto 证明：尽管机器人动力学高度非线性，**简单 PD 律即可全局解决全驱动操纵臂的点到点定位任务**。原文 *"A new feedback method for dynamic control of manipulators"*, Trans. ASME, J. Dyn. Syst. Meas. Control, Vol. 103, pp. 119–125 (1981)。

## §7.2 ★ PD + 重力补偿的 Lyapunov 稳定性证明【本抽取员据 Spong/Takegaki-Arimoto 标准证补全，\rebuilt】

> 搜索未取到逐字原证，本节据通行教材（Spong-Hutchinson-Vidyasagar《Robot Modeling and Control》§定理、Murray-Li-Sastry）标准证给出完整逐步推导，标 \rebuilt，综合时与一手教材核对。

**闭环误差动力学。** 把 $\boldsymbol\tau=\mathbf g(\mathbf q)+K_p\tilde{\mathbf q}-K_d\dot{\mathbf q}$ 代入 $\mathbf M\ddot{\mathbf q}+\mathbf C\dot{\mathbf q}+\mathbf g=\boldsymbol\tau$，重力项 $\mathbf g$ 抵消：
$$
\mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}=K_p\tilde{\mathbf q}-K_d\dot{\mathbf q}.
$$
因 $\mathbf q_d$ 常值，$\dot{\tilde{\mathbf q}}=-\dot{\mathbf q},\ \ddot{\tilde{\mathbf q}}=-\ddot{\mathbf q}$。平衡点 $(\tilde{\mathbf q},\dot{\mathbf q})=(\mathbf 0,\mathbf 0)$。

**Lyapunov 候选（动能 + 势能整形）。**
$$
V=\tfrac12\dot{\mathbf q}^\top\mathbf M(\mathbf q)\dot{\mathbf q}+\tfrac12\tilde{\mathbf q}^\top K_p\tilde{\mathbf q}.
$$
$\mathbf M\succ0,K_p\succ0\Rightarrow V$ 正定（$V>0$ 当 $(\tilde{\mathbf q},\dot{\mathbf q})\neq\mathbf 0$，$V(\mathbf 0)=0$）。

**求导（关键用斜对称性质 2）。**
$$
\dot V=\dot{\mathbf q}^\top\mathbf M\ddot{\mathbf q}+\tfrac12\dot{\mathbf q}^\top\dot{\mathbf M}\dot{\mathbf q}+\tilde{\mathbf q}^\top K_p\dot{\tilde{\mathbf q}}.
$$
代入 $\mathbf M\ddot{\mathbf q}=K_p\tilde{\mathbf q}-K_d\dot{\mathbf q}-\mathbf C\dot{\mathbf q}$ 与 $\dot{\tilde{\mathbf q}}=-\dot{\mathbf q}$：
$$
\dot V=\dot{\mathbf q}^\top\big(K_p\tilde{\mathbf q}-K_d\dot{\mathbf q}-\mathbf C\dot{\mathbf q}\big)+\tfrac12\dot{\mathbf q}^\top\dot{\mathbf M}\dot{\mathbf q}-\tilde{\mathbf q}^\top K_p\dot{\mathbf q}.
$$
$\dot{\mathbf q}^\top K_p\tilde{\mathbf q}$ 与 $-\tilde{\mathbf q}^\top K_p\dot{\mathbf q}$ 互消（标量 + $K_p$ 对称），整理：
$$
\dot V=-\dot{\mathbf q}^\top K_d\dot{\mathbf q}+\underbrace{\tfrac12\dot{\mathbf q}^\top(\dot{\mathbf M}-2\mathbf C)\dot{\mathbf q}}_{=0\ (\text{性质 2 斜对称})}=-\dot{\mathbf q}^\top K_d\dot{\mathbf q}\le0.
$$
故**半负定**（$\dot V=0\iff\dot{\mathbf q}=\mathbf 0$）。

**用 LaSalle 得渐近稳定（§1.4）。** 令 $\dot V=0\Rightarrow\dot{\mathbf q}=\mathbf 0$。在集合 $\{\dot{\mathbf q}=\mathbf 0\}$ 中找最大不变集：$\dot{\mathbf q}\equiv\mathbf 0\Rightarrow\ddot{\mathbf q}=\mathbf 0$，代入闭环 $\mathbf M\cdot\mathbf 0+\mathbf C\cdot\mathbf 0=K_p\tilde{\mathbf q}-K_d\cdot\mathbf 0\Rightarrow K_p\tilde{\mathbf q}=\mathbf 0\Rightarrow\tilde{\mathbf q}=\mathbf 0$（$K_p\succ0$）。故最大不变集仅 $\{(\tilde{\mathbf q},\dot{\mathbf q})=(\mathbf 0,\mathbf 0)\}$。由 LaSalle，平衡点**（全局，若 $V$ 径向无界）渐近稳定**。$\blacksquare$

> **要点（综合务必强调）：** 此证**不需要 $\mathbf M,\mathbf C$ 的精确知识进入稳定性**（只用 $\mathbf M\succ0$ + 斜对称 + 重力补偿），是无源性控制的威力；对比计算力矩（§5.5）需精确逆动力学抵消但给线性误差动力学。二者是机器人控制的两大范式。

## §7.3 PD+重力补偿 vs 计算力矩（对比表）【综合，\rebuilt】

| 维度 | 计算力矩 / 逆动力学控制（§5.5） | PD + 重力补偿（§7.1） |
|---|---|---|
| 控制律 | $\boldsymbol\tau=\tilde{\mathbf M}(\ddot{\mathbf q}_d+K_d\dot{\mathbf e}+K_p\mathbf e)+\tilde{\mathbf C}\dot{\mathbf q}+\tilde{\mathbf g}$ | $\boldsymbol\tau=\mathbf g+K_p\tilde{\mathbf q}-K_d\dot{\mathbf q}$ |
| 任务 | 轨迹跟踪（$\mathbf q_d(t)$ 时变） | 点到点定位（$\mathbf q_d$ 常值） |
| 模型需求 | 需**完整**动力学（$\mathbf M,\mathbf C,\mathbf g$）且精确 | 仅需**重力** $\mathbf g$（+ $\mathbf M\succ0$、斜对称结构） |
| 闭环 | 精确线性解耦误差动力学 $\ddot{\mathbf e}+K_d\dot{\mathbf e}+K_p\mathbf e=0$ | 非线性闭环，Lyapunov+LaSalle 证渐近稳定 |
| 计算量 | 大（每周期算逆动力学，RNEA $O(n)$） | 小（仅重力补偿） |
| 鲁棒性 | 对模型误差敏感（线性化不精确则误差耦合） | 对 $\mathbf M,\mathbf C$ 误差鲁棒（无源性） |

## §7.4 操作空间 / 任务空间控制（Khatib）【源：MIT Cheetah IROS 2018 §II（操作空间惯性）+ Wensing 综述 + 通识 \rebuilt】

**操作空间动力学（综合补，Khatib 1987，\rebuilt）。** 设末端任务坐标 $\mathbf x$，雅可比 $\mathbf J$（$\dot{\mathbf x}=\mathbf J\dot{\mathbf q}$）。把关节空间动力学投影到任务空间：
$$
\boldsymbol\Lambda(\mathbf q)\ddot{\mathbf x}+\boldsymbol\mu(\mathbf q,\dot{\mathbf q})+\mathbf p(\mathbf q)=\mathbf F,
$$
其中**操作空间惯性矩阵** $\boldsymbol\Lambda=(\mathbf J\mathbf M^{-1}\mathbf J^\top)^{-1}$，$\boldsymbol\mu$ 任务空间科氏/离心，$\mathbf p$ 任务空间重力，$\mathbf F$ 末端广义力。关节力矩由 $\boldsymbol\tau=\mathbf J^\top\mathbf F$ 给出。

**Cheetah 论文中的腿部操作空间控制（逐字）。** 摆动腿 $i$ 关节力矩：
$$
\boldsymbol\tau_i=\mathbf J_i^\top\big[K_p({}^B\mathbf p_{i,\mathrm{ref}}-{}^B\mathbf p_i)+K_d({}^B\mathbf v_{i,\mathrm{ref}}-{}^B\mathbf v_i)\big]+\boldsymbol\tau_{i,\mathrm{ff}},\tag{Cheetah 1}
$$
前馈
$$
\boldsymbol\tau_{i,\mathrm{ff}}=\mathbf J_i^\top\boldsymbol\Lambda_i\big({}^B\mathbf a_{i,\mathrm{ref}}-\dot{\mathbf J}_i\dot{\mathbf q}_i\big)+\mathbf C_i\dot{\mathbf q}_i+\mathbf G_i,\tag{Cheetah 2}
$$
$\mathbf J_i\in\mathbb R^{3\times3}$ 足雅可比，$K_p,K_d\in\mathbb R^{3\times3}$ 对角正定，${}^B\mathbf p_i,{}^B\mathbf v_i$ 第 $i$ 足位置/速度，$\boldsymbol\Lambda_i\in\mathbb R^{3\times3}$ 操作空间惯性阵，${}^B\mathbf a_{i,\mathrm{ref}}$ 体坐标参考加速度，$\mathbf C_i\dot{\mathbf q}_i+\mathbf G_i$ 腿的科氏+重力力矩。

**等自然频率增益整定（Cheetah 逐字）：** 为在各腿位形下保持闭环自然频率近常，对角 $K_p$ 第 $i$ 项
$$
K_{p,i}=\omega_i^2\,\Lambda_{i,i},\tag{Cheetah 3}
$$
$\Lambda_{i,i}$ 为质量阵第 $(i,i)$ 项（沿第 $i$ 轴的视在质量），$\omega_i$ 为目标自然频率。

**站立腿地面力控制（Cheetah 逐字）：**
$$
\boldsymbol\tau_i=\mathbf J_i^\top\mathbf R^\top\mathbf f_i,\tag{Cheetah 4}
$$
$\mathbf R$ 体到世界旋转，$\mathbf f_i$ 由 MPC（§8）算出的地面反力。**操作空间控制是 WBC（§7.5）的前身**【Wensing 逐字："These QP techniques have represented an evolution of previous operational-space control paradigms"】。

## §7.5 ★ 全身控制（WBC）概览：逆动力学 QP【源：Wensing T-RO §VI，逐字】

> 本节是"WBC 简介"的核心，全量抄录 Wensing 综述 §VI 的 QP 公式。

**(1) 任务动力学【Wensing 式 (7)，逐字】。** 一般任务（$\dot{\mathbf q}=\boldsymbol\nu$，$\mathbf e_i\in\mathbb R^m$），把 $\mathbf e_i(\mathbf q,t)$ 对时间二次微分：
$$
\ddot{\mathbf e}_i(\mathbf q,\boldsymbol\nu,\dot{\boldsymbol\nu},t)=\mathbf J_i(\mathbf q)\dot{\boldsymbol\nu}+\dot{\mathbf J}_i(\mathbf q,\boldsymbol\nu)\boldsymbol\nu+\mathbf a_i(\mathbf q,\boldsymbol\nu,t),\tag{7}
$$
$\mathbf J_i$ 任务雅可比，$\mathbf a_i=\partial^2\mathbf e_i/\partial t^2$。

**任务正则化（regulation）【Wensing 逐字】：**
- 几何等式任务（$l=2$）PD：$\ddot{\mathbf e}_i^d=-K_d\dot{\mathbf e}_i-K_p\mathbf e_i$，$K_d$ 常取 $2K_p^{1/2}$ 得临界阻尼。
- 速度级等式任务：$\dot{\mathbf e}_i^d=K_p\mathbf e_i$。
- 加速/力级等式任务：或省略（$\mathbf e_i^d=0$），或 PI：$\mathbf e_i^d=-K_p\mathbf e_i-K_i\!\int\mathbf e_i\,dt$。
- **加速度级不等式任务**（关节力矩限、摩擦锥）：无需正则化，直接施加。
- 速度级不等式（关节速度限）：令下一步 $\mathbf e_i^+=\mathbf e_i+\Delta t\,\dot{\mathbf e}_i\ge0\Rightarrow\dot{\mathbf e}_i^d=-\mathbf e_i/\Delta t$。
- 位置级不等式（关节位置限、避障）：最难，计算 $\ddot{\mathbf e}_i^d$ 使 $\mathbf e_i^+$ 为正，易与加速度级约束冲突；用控制屏障函数（CBF）等，但瞬时控制本质上难保证（需预测策略/反复求 TO）。

**(2) 聚合所有任务为线性约束（摩擦锥近似为金字塔）【Wensing 式 (10a)–(10c)，逐字】：**
$$
\mathbf M\dot{\boldsymbol\nu}-\mathbf J^\top\boldsymbol\lambda-\mathbf S^\top\boldsymbol\tau=-\mathbf C\boldsymbol\nu-\boldsymbol\tau_g\tag{10a}
$$
$$
\mathbf J_c\dot{\boldsymbol\nu}=\mathbf a_c\tag{10b}
$$
$$
\mathbf A_{\dot\nu,i}\dot{\boldsymbol\nu}+\mathbf A_{\lambda,i}\boldsymbol\lambda+\mathbf A_{\tau,i}\boldsymbol\tau=\mathbf b_i\quad(\text{非接触任务 }i)\tag{10c}
$$
(10a) 即动力学 (1b) 的重排；(10b) 单独列出的**接触约束**（接触点零加速度，$\mathbf a_c=-\dot{\mathbf J}_c\boldsymbol\nu$）；(10c) 任务（直接写在 $\dot{\boldsymbol\nu},\boldsymbol\tau,\boldsymbol\lambda$ 上的等式/不等式的推广）。决策变量 $(\dot{\boldsymbol\nu},\boldsymbol\tau,\boldsymbol\lambda)$。

**关键观察【Wensing 逐字】：** 动力学 (1b/10a) **对 $\dot{\boldsymbol\nu},\boldsymbol\tau,\boldsymbol\lambda$ 都是线性的**——这就是为何在加速度级写正则化，$\boldsymbol\tau,\boldsymbol\lambda$ 也能直接影响运动，从而整个问题保持**线性约束**，可写成 QP。

**(3) 典型 WBC QP（跟踪 LIP 质心参考、用最小关节力矩）【Wensing 逐字给的实例】：**
$$
\begin{aligned}
\min_{\dot{\boldsymbol\nu},\boldsymbol\tau,\boldsymbol\lambda}\ & w_1\big\|\mathbf J_G\dot{\boldsymbol\nu}+\dot{\mathbf J}_G\boldsymbol\nu-(\ddot{\mathbf p}_G^d+\ddot{\mathbf e}_G^d)\big\|^2+w_2\|\boldsymbol\tau\|^2\\
\text{s.t.}\ & \mathbf M\dot{\boldsymbol\nu}+\mathbf C\boldsymbol\nu+\boldsymbol\tau_g=\mathbf S^\top\boldsymbol\tau+\mathbf J_c^\top\boldsymbol\lambda\quad(\text{动力学})\\
& \mathbf J_c\dot{\boldsymbol\nu}=\mathbf a_c\quad(\text{固定接触})\\
& \mathbf C\boldsymbol\lambda\le\mathbf 0\quad(\text{摩擦锥})\\
& \ldots(\text{其他最高优先级约束})\ldots
\end{aligned}
$$
（$\mathbf J_G$ 质心雅可比，$w_1,w_2$ 软优先级权。）**QP 反应式控制现已近乎通用**【Wensing 逐字】。优点：依赖成熟现成 QP 解器（qpOASES [193]、OSQP/ECOS [194] 等，多免费）。**局限**：无法在更低优先级处理不等式；任务多时权重整定难。

**(4) 选择矩阵与变量消元（预解）【Wensing §VI-B1，逐字】。**
- $\mathbf S$ 简单形式（足式 $\mathbf S=[\mathbf 0\ \mathbf I]$）允许把 $\boldsymbol\tau$ 表为 $\dot{\boldsymbol\nu},\boldsymbol\lambda$ 的线性函数并消去，只保留 (10a) 的非驱动行——降低计算量 [64,174]。
- 或用 (10a) 消 $\dot{\boldsymbol\nu}$（利用 $\mathbf M\succ0$ 的 Cholesky 分解廉价求解）[173,184]。
- 当 $\mathbf J_c=\mathbf J$，可用 (10b)+(10a) 消 $\boldsymbol\lambda$——这是**在接触约束零空间上投影**，把操作空间控制推广到欠驱动机器人 [185,186,187]。

## §7.6 ★ 分层 / 严格优先级 WBC（HQP）【源：Wensing T-RO §VI-B2，逐字】

**软优先级 vs 硬优先级。** 任务冲突时（(10) 无解），给每约束按 $L_2$ 范数定违背度，两法可组合：**加权各违背（软优先级）** 或 **定义层级（严格优先级）**。

**QP 法（软优先级，前述实例）。** 所有不等式 + 部分等式当作 QP 约束（top priority），其余约束违背的加权平方和作目标。

**分层 / 词典最小二乘（HQP，严格优先级）【Wensing 逐字】。** 设计者为每个任务指定明确优先级；同级任务可加权组合；最高级约束不必可行。所得问题称**词典最小二乘（lexicographic least-squares）/ 分层 QP（HQP）**。**Kanoun [200] 的级联 QP** 是首个能在任意优先级处理不等式约束的解器，后续 [172,174] 改进其计算时间，专用解器 [201,202] 一遍高效求解。**HQP 是 QP 法的严格超集**，也是任务权重比趋于无穷时的极限 [203]。文献 [202] 表明（与常识相反）专用解器解 HQP 可比 QP 更快（理论与实践）；但级联解器计算成本与奇异性处理 [204] 限制了应用。

**HQP 数学结构（综合补，\rebuilt）。** 优先级 $1,2,\ldots,p$。第 $k$ 级解
$$
\min_{\mathbf x}\ \|\mathbf A_k\mathbf x-\mathbf b_k\|^2\quad\text{s.t.}\quad\mathbf A_j\mathbf x-\mathbf b_j=\mathbf w_j^*\ (j<k),
$$
即在不破坏更高优先级最优残差 $\mathbf w_j^*$ 的前提下最小化本级残差——等价于在更高级约束的零空间内逐级求解（null-space projection）。综合时可补零空间投影递推 $\mathbf x_k=\mathbf x_{k-1}+\mathbf N_{k-1}\mathbf z_k$（$\mathbf N_{k-1}$ 为前 $k-1$ 级雅可比的零空间投影）。

**QP 形式融入其他范式【Wensing 逐字】。** QP 的灵活性可纳入其他控制范式：[49,195] 把简化模型的最优控制经"下降任务空间值函数"的代价项嵌入（瞬时 QP 由值函数的长期考量缓解）；[196] 纳入无源性控制提升对未建模效应的鲁棒；[197] 让控制对冲击事件的速度跳变不变；[198,199] 纳入软接触。

---

# 第八部分　足式机器人控制概览

## §8.1 足式控制的总体架构与简化模型层级【源：Wensing T-RO §II，逐字】

**根本难点【Wensing 逐字】。** 足式 OCP 难解因：与环境的**间歇单向接触**（非光滑/刚性动力学）、**高自由度**。社区共识：通过求解 OCP（模型驱动或数据驱动）生成运动控制律；但最一般问题在线求解仍不可行，故须**简化模型 + 选择接触处理方式 + 选数值方法**。

**架构分层（Wensing Fig. 2，逐字整理）。** 把大问题拆成小块的几种策略（自上而下）：
- **接触规划器**（运动学模型，长时域，低速率）→ 接触块；
- **简化模型 MPC**（质心/SRB 模型，中时域，中速率）→ 质心轨迹；
- **全身控制 WBC**（完整模型，短时域，高速率）→ 执行器力矩。
另有 Kinodynamic 接触规划 + WBC，或 Whole-Body MPC（完整模型中时域高速率），或端到端控制器。

## §8.2 浮基刚体动力学（含接触）【源：Wensing 式 (1b)–(2)，逐字】

**全机器人动力学【Wensing 式 (1b)，逐字】：**
$$
\boxed{\ \mathbf M(\mathbf q)\dot{\boldsymbol\nu}+\mathbf C(\mathbf q,\boldsymbol\nu)\boldsymbol\nu+\boldsymbol\tau_g(\mathbf q)=\mathbf S^\top\boldsymbol\tau+\mathbf J(\mathbf q)^\top\boldsymbol\lambda\ }
$$
$\mathbf M(\mathbf q)$ 质量阵；$\boldsymbol\nu=(\boldsymbol\nu_b,\boldsymbol\nu_j)$ 广义速度（基 + 关节）；$\mathbf C\boldsymbol\nu$ 科氏/离心；$\boldsymbol\tau_g$ 重力；$\mathbf S$ 驱动自由度选择阵；$\boldsymbol\tau$ 关节力矩；$\mathbf J(\mathbf q)$ 接触雅可比；$\boldsymbol\lambda$ 接触力。

**分块形式【Wensing 式 (2)，逐字】**（基自由度不驱动，故右侧上块为 $\mathbf 0$）：
$$
\begin{bmatrix}\mathbf M_{bb}&\mathbf M_{bj}\\\mathbf M_{jb}&\mathbf M_{jj}\end{bmatrix}
\begin{bmatrix}\dot{\boldsymbol\nu}_b\\\dot{\boldsymbol\nu}_j\end{bmatrix}+\mathbf C\boldsymbol\nu+\boldsymbol\tau_g=\begin{bmatrix}\mathbf 0\\\boldsymbol\tau\end{bmatrix}+\mathbf J^\top\boldsymbol\lambda.
$$
**接触约束（综合补，\rebuilt）：** 刚性接触点零加速度 $\mathbf J_c\dot{\boldsymbol\nu}+\dot{\mathbf J}_c\boldsymbol\nu=\mathbf 0$，即 $\mathbf J_c\dot{\boldsymbol\nu}=-\dot{\mathbf J}_c\boldsymbol\nu=\mathbf a_c$（即 Wensing 式 10b）。

## §8.3 质心动力学（centroidal dynamics）【源：Wensing 式 (3)–(4)，逐字】

**质心动量定义【式 (3)，逐字】：** $\mathbf h_G=\mathbf A_G(\mathbf q)\boldsymbol\nu$，$\mathbf h_G=(\mathbf k_G,\mathbf l_G)\in\mathbb R^6$（角动量 + 线动量），$\mathbf A_G$ 质心动量矩阵（CMM）。

**质心动力学（Newton–Euler 关于质心）【式 (4)，逐字】：**
$$
\dot{\mathbf h}_G=\begin{bmatrix}\dot{\mathbf k}_G\\\dot{\mathbf l}_G\end{bmatrix}=\begin{bmatrix}\mathbf 0\\-M\mathbf a_g\end{bmatrix}+\sum_{i=1}^{n_c}\begin{bmatrix}(\mathbf p_i-\mathbf p_{\mathrm{CoM}})\times\boldsymbol\lambda_i\\\boldsymbol\lambda_i\end{bmatrix},
$$
$\mathbf p_{\mathrm{CoM}}=[c_x,c_y,c_z]^\top$ 质心位置，$M$ 总质量，$\mathbf a_g$ 重力加速度，$n_c$ 接触点数。**质心位置关系：** $M\frac{d}{dt}\mathbf p_{\mathrm{CoM}}=\mathbf l_G$。

## §8.4 线性倒立摆模型（LIP）【源：Wensing，逐字】

**LIP 动力学【逐字】：**
$$
\ddot c_{x,y}=\omega^2(c_{x,y}-p_{x,y}),
$$
$c_{x,y}$ 质心水平位置，$p_{x,y}$ 压力中心（CoP）水平位置，$\omega=\sqrt{g/h}$ 自然频率（$h$ 固定质心高度）。**简化假设**：质心高度恒定。

## §8.5 接触力锥与接触扳手锥（CWC）【源：Wensing III + IV-A，逐字】

**接触建模（综合 + Wensing III，逐字摘）。** 符号距离函数 $\boldsymbol\phi(\mathbf q)$；法向/切向雅可比 $\mathbf J_n,\mathbf J_t$（$\mathbf J_n\boldsymbol\nu,\mathbf J_t\boldsymbol\nu$ 为接触帧速度）；法向/切向力 $\boldsymbol\lambda_n,\boldsymbol\lambda_t$。接触力为弹簧阻尼函数 $\boldsymbol\lambda=\mathbf F_{\mathrm{contact}}(\boldsymbol\phi,\mathbf J\boldsymbol\nu)$。**单向约束**：接触不能"拉"，$\lambda_n\ge0$，$\lambda_n<0$ 时分离。Coulomb 干摩擦：$\boldsymbol\lambda_t$ 依切向速度符号（不连续）。

**混合动力学 / 冲击【Wensing 逐字】。** 无穷刚度极限下接触初始化产生冲量、冲击致速度瞬跳。混合系统由 mode（接触状态）、guard（mode 转换条件，如建立/断开接触）、reset map $\mathbf R$（转换结果，如冲击）定义：
$$
\mathbf x^+=\mathbf R(\mathbf x^-).
$$

**接触扳手锥 CWC【Wensing 式，逐字】：**
$$
\mathrm{CWC}=\left\{\sum_{i=1}^{n_c}\begin{bmatrix}(\mathbf p_i-\mathbf p_{\mathrm{CoM}})\times\boldsymbol\lambda_i\\\boldsymbol\lambda_i\end{bmatrix}\ \middle|\ \boldsymbol\lambda_i\in\mathcal C_i,\ \forall i\right\},
$$
$\mathcal C_i$ 为接触 $i$ 的摩擦锥。**每个摩擦锥用多边形（金字塔）近似时，CWC 是多面体**【Wensing 逐字】——这使 WBC/MPC 的接触约束变成线性不等式。

## §8.6 ★ MIT Cheetah 3 凸 MPC（单刚体模型）完整推导【源：Di Carlo et al., IROS 2018，全文逐字】

> 这是"足式控制概览"的旗舰例，全量抄录单刚体动力学→线性化→离散→凸 QP。

### §8.6.1 单刚体动力学（世界系）【Cheetah 式 (5)–(8)，逐字】
对每个地面反力 $\mathbf f_i\in\mathbb R^3$，质心到力作用点向量 $\mathbf r_i\in\mathbb R^3$。**世界系刚体动力学：**
$$
\ddot{\mathbf p}=\frac{\sum_{i=1}^n\mathbf f_i}{m}-\mathbf g\tag{5}
$$
$$
\frac{d}{dt}(\mathbf I\boldsymbol\omega)=\sum_{i=1}^n\mathbf r_i\times\mathbf f_i\tag{6}
$$
$$
\dot{\mathbf R}=[\boldsymbol\omega]_\times\mathbf R\tag{7}
$$
$\mathbf p$ 质心位置，$m$ 质量，$\mathbf g$ 重力加速度，$\mathbf I$ 惯性张量（世界系），$\boldsymbol\omega$ 角速度，$\mathbf R$ 体到世界旋转，$[\mathbf x]_\times$ 反对称阵（$[\mathbf x]_\times\mathbf y=\mathbf x\times\mathbf y$）。

**姿态用 Z-Y-X Euler 角【式 (8)，逐字】：** $\Theta=[\phi\ \theta\ \psi]^\top$（$\psi$ yaw、$\theta$ pitch、$\phi$ roll），
$$
\mathbf R=\mathbf R_z(\psi)\mathbf R_y(\theta)\mathbf R_x(\phi),
$$
$\mathbf R_n(\alpha)$ 表绕 $n$ 轴转 $\alpha$。

### §8.6.2 角速度与 Euler 角速率【Cheetah 式 (9)–(12)，逐字】
$$
\boldsymbol\omega=\begin{bmatrix}\cos\theta\cos\psi&-\sin\psi&0\\\cos\theta\sin\psi&\cos\psi&0\\-\sin\theta&0&1\end{bmatrix}\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}\tag{9}
$$
若 $\cos\theta\neq0$（机体非竖直），(9) 可逆得
$$
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}=\begin{bmatrix}\cos\psi/\cos\theta&\sin\psi/\cos\theta&0\\-\sin\psi&\cos\psi&0\\\cos\psi\tan\theta&\sin\psi\tan\theta&1\end{bmatrix}\boldsymbol\omega\tag{10}
$$
**小 roll/pitch（$\phi,\theta$ 小）近似**【式 (11)–(12)，逐字】：
$$
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}\approx\begin{bmatrix}\cos\psi&\sin\psi&0\\-\sin\psi&\cos\psi&0\\0&0&1\end{bmatrix}\boldsymbol\omega=\mathbf R_z(\psi)^\top\boldsymbol\omega.\tag{11–12}
$$
**Cheetah 注（逐字）：** Euler 角旋转次序很重要；换序则近似在合理姿态下不准。

### §8.6.3 惯性近似与角动量近似【Cheetah 式 (13)–(15)，逐字】
**世界系惯性张量** $\mathbf I=\mathbf R\,{}_B\mathbf I\,\mathbf R^\top$（式 14），小 roll/pitch 近似
$$
\hat{\mathbf I}=\mathbf R_z(\psi)\,{}_B\mathbf I\,\mathbf R_z(\psi)^\top,\tag{15}
$$
${}_B\mathbf I$ 体坐标惯性。**角动量导数近似（式 13，逐字）：**
$$
\frac{d}{dt}(\mathbf I\boldsymbol\omega)=\mathbf I\dot{\boldsymbol\omega}+\boldsymbol\omega\times(\mathbf I\boldsymbol\omega)\approx\mathbf I\dot{\boldsymbol\omega}.\tag{13}
$$
**Cheetah 注（逐字）：** 此近似（如 [13] 也用）舍去了旋转体的进动/章动；$\boldsymbol\omega\times(\mathbf I\boldsymbol\omega)$ 项对小角速度体很小，对动力学贡献不大。

### §8.6.4 线性状态空间（含重力增广态）【Cheetah 式 (16)–(17)，逐字】
合并近似姿态动力学与平动动力学：
$$
\frac{d}{dt}\begin{bmatrix}\hat{\boldsymbol\Theta}\\\hat{\mathbf p}\\\hat{\boldsymbol\omega}\\\hat{\dot{\mathbf p}}\end{bmatrix}=\begin{bmatrix}\mathbf 0_3&\mathbf 0_3&\mathbf R_z(\psi)^\top&\mathbf 0_3\\\mathbf 0_3&\mathbf 0_3&\mathbf 0_3&\mathbf 1_3\\\mathbf 0_3&\mathbf 0_3&\mathbf 0_3&\mathbf 0_3\\\mathbf 0_3&\mathbf 0_3&\mathbf 0_3&\mathbf 0_3\end{bmatrix}\begin{bmatrix}\hat{\boldsymbol\Theta}\\\hat{\mathbf p}\\\hat{\boldsymbol\omega}\\\hat{\dot{\mathbf p}}\end{bmatrix}+\begin{bmatrix}\mathbf 0_3&\cdots&\mathbf 0_3\\\mathbf 0_3&\cdots&\mathbf 0_3\\\hat{\mathbf I}^{-1}[\mathbf r_1]_\times&\cdots&\hat{\mathbf I}^{-1}[\mathbf r_n]_\times\\\mathbf 1_3/m&\cdots&\mathbf 1_3/m\end{bmatrix}\begin{bmatrix}\mathbf f_1\\\vdots\\\mathbf f_n\end{bmatrix}+\begin{bmatrix}\mathbf 0\\\mathbf 0\\\mathbf 0\\\mathbf g\end{bmatrix}\tag{16}
$$
加一个**重力增广状态**把仿射项 $\mathbf g$ 并入状态空间，得便利形式【式 (17)，逐字】：
$$
\dot{\mathbf x}(t)=\mathbf A_c(\psi)\mathbf x(t)+\mathbf B_c(\mathbf r_1,\ldots,\mathbf r_n,\psi)\mathbf u(t),\tag{17}
$$
$\mathbf A_c\in\mathbb R^{13\times13},\mathbf B_c\in\mathbb R^{13\times3n}$。**此形式只依赖 yaw 与落足位置**；若二者可提前算出，动力学成为**线性时变（LTV）**，适合凸 MPC。
（状态 $\mathbf x=[\boldsymbol\Theta;\mathbf p;\boldsymbol\omega;\dot{\mathbf p};g]\in\mathbb R^{13}$，即 3 姿态 + 3 位置 + 3 角速度 + 3 线速度 + 1 重力 = 13。）

### §8.6.5 离散化【Cheetah 式 (25)–(26)，逐字】
$\mathbf B_c,\mathbf A_c$ 用**扩展线性系统的状态转移矩阵**做零阶保持（ZOH）离散化：
$$
\frac{d}{dt}\begin{bmatrix}\mathbf x\\\mathbf u\end{bmatrix}=\begin{bmatrix}\mathbf A&\mathbf B\\\mathbf 0&\mathbf 0\end{bmatrix}\begin{bmatrix}\mathbf x\\\mathbf u\end{bmatrix},\tag{25}
$$
得离散形式
$$
\mathbf x[n+1]=\hat{\mathbf A}\,\mathbf x[n]+\hat{\mathbf B}[n]\,\mathbf u[n].\tag{26}
$$
**Cheetah 注（逐字）：** (26) 仅当机器人能跟随参考轨迹时才准；大偏离致 $\hat{\mathbf B}[n]$ 不准，但**第一步** $\hat{\mathbf B}[n]$ 由当前状态算、总是对的；扰动后下一次 MPC（至多 40 ms 后）按扰动状态重算参考，从而补偿扰动。

### §8.6.6 凸 MPC 的 QP 与摩擦金字塔约束【Cheetah 式 (18)–(24)，逐字】
**MPC 标准式**（已在 §4.2 抄 (18)–(21)、(29)–(32)）。**力约束（Cheetah §IV-A，逐字）：**
- 等式约束 (21) $D_i\mathbf u_i=\mathbf 0$：把不接触足的力置零（强制步态）。
- 不等式约束 (20)：对每个站立足给 **6 条**不等式：
$$
f_{\min}\le f_z\le f_{\max}\tag{22}
$$
$$
-\mu f_z\le f_x\le\mu f_z\tag{23}
$$
$$
-\mu f_z\le f_y\le\mu f_z\tag{24}
$$
即限制最小/最大法向力 + 摩擦锥的**方形金字塔近似**（$\mu$ 摩擦系数）。

### §8.6.7 实验参数（Cheetah Table I，逐字）
| 参数 | 值 | 参数 | 值 |
|---|---|---|---|
| $m$ | 43 kg | $\Theta$ 权重 | 1 |
| $I_{xx}$ | 0.41 kg·m² | $z$ 权重 | 50 |
| $I_{yy}$ | 2.1 kg·m² | yaw rate 权重 | 1 |
| $I_{zz}$ | 2.1 kg·m² | $v$ 权重 | 1 |
| $\mu$ | 0.6 | 力权重 $\alpha$ | $1\times10^{-6}$ |
| $g_z$ | $-9.8$ m/s² | $f_{\min}$ | 10 N |
| $\tau_{\max}$ | 250 N·m | $f_{\max}$ | 666 N |

**性能（Cheetah 逐字）：** 时域 0.5 s（一个步态周期 0.33–0.5 s，分 10–16 步），$<1$ ms 解到最优，20–30 Hz（实测 25–50 Hz）。步态：stand, trot, flying-trot, pronk, bound, pace, 3-legged, full 3D gallop。前进速度达 3 m/s、横向 1 m/s、角速度 180 deg/s。求解器 ECOS、qpOASES。结果直接（无滤波）用于式 (4) 算关节力矩；状态估计、摆动腿规划、腿阻抗控制在 1 kHz 运行。

## §8.7 足式控制的简化模型谱系与简化代价【源：Wensing II-B，逐字】

**简化模型谱系（自简到繁）。** 点质量 / LIP（只建质心与接触位置，忽略关节角与速度细节）→ 单刚体 SRB（Cheetah 用）→ 质心动力学 → 完整刚体。模型越简，状态越小、动力学越线性，越利于在线快速计算；但其简化假设（如**恒定质心高度**）或被忽略的约束（如**关节位置/力矩限**）会**严重限制可生成的运动**【Wensing 逐字】。

## §8.8 足式 WBC 的接触维持任务与稳定化【源：Wensing VI-A1, VI-A3，逐字】

**轨迹稳定化 via WBC【Wensing 逐字】。** 当 OCP 计算太慢无法放进快速 MPC 回路时，需反应式稳定控制器在真实硬件上执行计算出的运动，或控制轨迹优化中被忽略的方面（如建模简化、第一/第二种配置）。近十年足式社区收敛到一类**基于小凸 QP 快速求解**的反应式 WBC，把电机指令作为状态反馈的函数计算。

**LIP 跟踪任务【Wensing 逐字】。** 要质心跟随 LIP 计划：令 $\mathbf e_i(\mathbf q,t)=\mathbf 0$，$\mathbf e_i\in\mathbb R^3$ 为 LIP 目标位置与正运动学推出的真实位置之误差。避障等任务由不等式 $\mathbf e_i(\mathbf q)\ge0$（如机体到物体距离）自然刻画。

**接触维持等式任务【Wensing 逐字】。** 最常见且关键的等式任务是维持接触；多数情形假设硬接触，$\ddot{\mathbf e}_i^d$ 设为 0（虽然为防滑设阻尼项可能更好 [64]）。接触初始化附近此约束致问题表述与控制快速变化。

---

# 第九部分　综合写作建议（给综合 agent）

1. **章结构建议**：① 反馈与稳定性（§1，Lyapunov/LaSalle/CLF 作全章稳定性工具箱）→ ② PID（§2，含 Z-N 表、抗饱和、离散式、Routh 稳定性）→ ③ 状态空间与 LQR（§3，**HJB→Riccati 完整推导是本章重头**，连续/离散/有限/无穷 + Hamilton 矩阵解 + LQE 对偶）→ ④ MPC（§4，凝聚 QP + 终端代价/约束接 LQR）→ ⑤ 面向机器人的控制（§5 逆动力学/计算力矩 + §6 反馈线性化 + §7 PD+重力补偿/操作空间/WBC/HQP）→ ⑥ 足式控制概览（§8，Cheetah 凸 MPC 作旗舰例 + 质心/LIP/CWC + 架构分层）。
2. **记号统一**：务必处理 §0 表中两处字母双关——（a）LQR Riccati 解 $\mathbf P$/$\mathbf S$ vs Kalman 协方差 $\mathbf P$；（b）选择矩阵 $\mathbf S$ vs Riccati $\mathbf S$。建议控制章 Riccati 用 $\mathbf P$ 并加显式注；足式 WBC 的选择矩阵保留 $\mathbf S$（分节隔离）。
3. **`\rebuilt` 标注**：所有标 \rebuilt 的逐步代数（§2.3–2.5 抗饱和/离散/Routh、§3.3/§3.7 推导补全、§4.3 MPC 稳定性、§5.4 RNEA、§7.2 PD 证、§7.4 操作空间 $\boldsymbol\Lambda$、§7.6 HQP 零空间）须与一手教材（Spong、Murray-Li-Sastry、Khalil《Nonlinear Systems》、Åström-Murray《Feedback Systems》、Rawlings-Mayne-Diehl《MPC》）核对后落地。
4. **图（tikz）建议**：闭环反馈框图（§1.1）；PID 三项结构图；LQR 值函数/极点配置；MPC 滚动时域示意；计算力矩"内环线性化+外环 PD"框图；足式控制分层架构（Wensing Fig.2）；单刚体 + 地面反力 + 摩擦金字塔。
5. **每式 `\cite`**：LQR 推导引 Tedrake underactuated；计算力矩/反馈线性化引维基+Spong；WBC/质心/HQP 引 Wensing T-RO（arXiv:2211.11644）；足式凸 MPC 引 Di Carlo IROS 2018。

---

## §10 参考文献（出处，供综合 agent 引用 / 合入 refs.bib）

> **注意（[[rebuild-orchestration]] 铁律）：** 抽取员**不直接改 `refs.bib`**（并发写冲突）。下列为建议 bib 条目，供综合/编排 agent 中央合并。

1. **Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., Kim, S.** "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control." *2018 IEEE/RSJ IROS*, Madrid, Oct 2018. PDF: `https://dspace.mit.edu/handle/1721.1/138000`（bitstream: `https://dspace.mit.edu/server/api/core/bitstreams/474e8173-7b22-46e6-a51b-3d8e8a383357/content`）；IEEE: `https://ieeexplore.ieee.org/document/8594448/`. 建议 key: `dicarlo2018cheetah`.
2. **Wensing, P. M., Posa, M., Hu, Y., Escande, A., Mansard, N., Del Prete, A.** "Optimization-Based Control for Dynamic Legged Robots." *IEEE Trans. on Robotics*, vol. 40, pp. 43–63, 2024. arXiv:2211.11644. `https://arxiv.org/abs/2211.11644`. 建议 key: `wensing2024legged`.
3. **Tedrake, R.** *Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation.* MIT course notes. LQR: `https://underactuated.mit.edu/lqr.html`；Multibody: `https://underactuated.mit.edu/multibody.html`. 建议 key: `tedrake_underactuated`.
4. **Takegaki, M., Arimoto, S.** "A new feedback method for dynamic control of manipulators." *Trans. ASME, J. Dyn. Syst. Meas. Control*, vol. 103, pp. 119–125, 1981. 建议 key: `takegaki1981feedback`.
5. **Khatib, O.** "A unified approach for motion and force control of robot manipulators: The operational space formulation." *IEEE J. on Robotics and Automation*, 1987.（Wensing 综述 [166] 引）。建议 key: `khatib1987operational`.
6. **Wikipedia**（概念与公式骨架，二手但与教材一致，综合时优先以教材替换为一手引用）：
   - LQR: `https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator`
   - Algebraic Riccati equation: `https://en.wikipedia.org/wiki/Algebraic_Riccati_equation`
   - PID controller: `https://en.wikipedia.org/wiki/PID_controller`
   - Ziegler–Nichols method: `https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method`
   - Integral windup: `https://en.wikipedia.org/wiki/Integral_windup`
   - Model predictive control: `https://en.wikipedia.org/wiki/Model_predictive_control`
   - Computed torque control: `https://en.wikipedia.org/wiki/Computed_torque_control`
   - Feedback linearization: `https://en.wikipedia.org/wiki/Feedback_linearization`
   - Inverse dynamics: `https://en.wikipedia.org/wiki/Inverse_dynamics`
   - Lyapunov stability: `https://en.wikipedia.org/wiki/Lyapunov_stability`
   - LaSalle's invariance principle: `https://en.wikipedia.org/wiki/LaSalle%27s_invariance_principle`
7. **教材建议补充一手源**（本抽取据二手重建处的核对目标，综合 agent 自行获取）：Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*（PD/计算力矩/无源性证）；Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*（操作空间/反馈线性化）；Khalil, *Nonlinear Systems*（Lyapunov/LaSalle）；Åström, Murray, *Feedback Systems*（PID/抗饱和）；Rawlings, Mayne, Diehl, *Model Predictive Control: Theory, Computation, and Design*（MPC 稳定性/终端代价）。

---

**抽取完成度自评（见 schema completeness 字段）。** 已全量覆盖本章六大聚焦点：反馈与稳定性（Lyapunov 定义/直接法/LaSalle/CLF，§1）、PID（时域/拉氏/ISA/Z-N 表/抗饱和/离散/Routh，§2）、状态空间与 LQR（**HJB→Riccati 完整逐步推导**、连续/离散、有限/无穷、CARE/DARE、Hamilton 矩阵解、LQE 对偶、闭环 Lyapunov 稳定证，§3）、MPC（概念/滚动时域/凝聚 QP 完整代数/终端代价稳定性，§4）、面向机器人控制（逆动力学/计算力矩**完整误差动力学证**、反馈线性化 SISO/MIMO、PD+重力补偿**完整 Lyapunov+LaSalle 证**、操作空间、WBC 逆动力学 QP、分层 HQP，§5–7）、足式控制（浮基动力学、质心动力学、LIP、CWC/摩擦金字塔、**MIT Cheetah 凸 MPC 全式逐字**、架构分层，§8）。一手论文（Cheetah IROS18、Wensing T-RO）已取全文 PDF 逐式抄录。**未尽/待核对部分**：① 个人笔记 `SLAM理论.md` 未同步，工程实践细节缺；② 标 `\rebuilt` 的逐步代数（PID 抗饱和/离散/Routh、PD+重力补偿证、操作空间 $\boldsymbol\Lambda$ 推导、RNEA 伪码、MPC 终端集构造、HQP 零空间递推）据通行教材重建，需与 Spong/Murray/Khalil/Rawlings 一手核对；③ Wensing PDF 个别公式含 latex-base64 嵌入块未能逐字取，已用其 HTML 解析结果（WebFetch）与 PDF 文本交叉锁定核心式。
