# 写作种子：四足运动控制·实践章「宇树控制框架与开源控制器」（tag=prac）

> **文件性质**：成书前的「写作种子」，非正文。服务于「四足机器人运动控制」部之**实践章**。
> **章定位**：实践章 = 把前六章理论（单刚体动力学、足底力 QP、凸 MPC、WBC、状态估计、步态）锚到**可跑起来的代码模块**。三套真实开源工程：
> 1. **unitree_guide**（宇树官方教学框架，FSM+OOP，QP 足底力分配 + 简化逆动力学，源=宇树书 ch2/ch3）；
> 2. **legged_control**（OCS2 的 NMPC + WBC，硬件抽象/控制器生命周期/URDF 部署，源=个人笔记 1541–2457）；
> 3. **A1-QP-MPC-Controller**（MIT Cheetah 凸 MPC 的极简复刻，双线程力控，源=个人笔记 1525–1540）。
> **铁律执行**：本章遵「跳过纯工程」——框架/ROS/Gazebo/电机配置/代码细节凝练进 `practice` 盒与实践节，**不做全量推导**。公式极少（只收混合控制 PD+前馈、单腿静力学 J^T、摆动腿修正力、NMPC/WBC 标准形），理论一律 `\cref` 回前六章与 `ch:engineering` 附录。
> **主线骨架**：以个人笔记的「作者思路」串联——**控制器不该知道自己面对仿真还是实物**（硬件抽象/动态绑定）这一条贯穿全章的工程哲学，把三套框架融为「从理论到跑起来」的一张工程地图，而非三段拼接。

---

## 源映射与角色分工

| 代号 | 来源 | 行号 | 承担 |
|---|---|---|---|
| **[笔记·A1QP]** | 个人笔记 §A1-QP-MPC-Controller | 1525–1540 | MIT Cheetah 凸 MPC 极简复刻；`MainGazebo.cpp` 双线程（力控线程 + 主更新流程）。入门级对照。 |
| **[笔记·legged]** | 个人笔记 §Legged Control 项目解析 | 1541–2457 | 主线源。NMPC+WBC 架构、硬件抽象层（句柄/接口/插件）、控制器基类生命周期（init/update/starting/stopping）、URDF→xacro 部署、碰撞体简化、姿态/零点配置。 |
| **[笔记·RL]** | 个人笔记 §强化学习环境配置实录 | 2458–2564 | Mujoco 源码安装、Docker 镜像。**点到为止**。 |
| **[宇树书·ch2]** | 宇树书 第2章 关节电机 | 247–644 | FOC/硬件/混合控制/报文/配置。**凝练**，仅抽混合控制式 (2.1) 与幅度上限 (2.2)。 |
| **[宇树书·ch3]** | 宇树书 第3章 仿真与控制框架 | 645–1466 | ROS/Gazebo、OOP 三核心（数据抽象/继承/动态绑定）、FSM 实现、关节控制、关节编号约定、编译部署、真机网络。**凝练**，抽 OOP/FSM 思想 + 关节编号表。 |

> **交叉引用（前六章 \cref 锚点）**：单刚体动力学→`ch:srbd`；足底力 QP+摩擦锥→`ch:balance_qp`；凸 MPC 标准形→`ch:convex_mpc`；WBC→`ch:wbc`；状态估计（KF）→`ch:state_estimation`；步态/摆线→`ch:gait`；单腿运动学/雅可比→`ch:leg_kinematics`；工程细节大全→附录 `ch:engineering`。
> **外部文献**：`\cite{unitree2023quadruped}`（宇树书）、`\cite{yu2021quadruped`（于宪元论文，本部已建条目）、`\cite{dicarlo2018mit}`（Cheetah 3 凸 MPC）、`\cite{sleiman2021unified}`（OCS2 统一 MPC 框架，笔记行 1604）、`\cite{grandia2022perceptive}`（感知 NMPC，行 1606）、`\cite{bellicoso2016whole}`（分层 WBC，行 1608）。

---

# 一、本章拟定节结构（对应 v5.0：动机→反面→历史→理论锚定→陷阱→练习）

## §prac.1 动机：理论到底怎么「跑起来」
- 前六章给了完整理论闭环（SRBD → 足底力 QP → 凸 MPC → WBC），但读者面对的真实问题是：**这些公式最终如何变成电机上的力矩？** 本章给出三套开源工程的「工程地图」，把每个理论模块对应到具体代码文件/类。
- `insight`（作者洞见·主线）：**控制器不应该知道自己面对的是仿真还是实物**（[笔记·legged] 行 1616）。这是贯穿全章的设计哲学——controller 只和标准接口 `hardware_interface` 通信，仿真器（`legged_gazebo`）「假扮」成真实硬件。一次代码、两处复用。

## §prac.2 反面：朴素写法为什么不行
- **反面 1**：仿真/实物各写一套收发函数 → 移植时要替换所有收发调用（[宇树书·ch3] 行 690）。`pitfall`。
- **反面 2**：IMU 走 ROS 话题 → 网络延迟/抖动/异步，高频控制循环（数百 Hz）无法接受（[笔记·legged] 行 1759）。
- **反面 3**：SolidWorks 导出的 STL 网格直接做碰撞体 → OCS2 求解超时 → 机器人乱动（[笔记·legged] 行 1901）。
- **反面 4**：非标准垂直构型坐标系 → 碰撞检测穿模、惯量变换复杂（[笔记·legged] 行 1810、1847）。

## §prac.3 历史与生态：三套框架的定位
- **unitree_guide**（宇树书配套）：教学优先，FSM+OOP，瞬时 QP 足底力分配（**无 MPC**），ROS melodic/Gazebo9。
- **A1-QP-MPC-Controller**（YY 硕）：MIT Cheetah 凸 MPC 的极简入门复刻。
- **legged_control**：工业级，OCS2 的 NMPC（centroidal dynamics + SQP + HPIPM）+ 分层/加权 WBC。
- 强化学习路线（Mujoco/Isaac）作为「另一条腿」点到为止（§prac.7）。

## §prac.4 理论锚定：把六章理论钉到代码
- §prac.4.1 关节层：混合控制 PD+前馈（式 prac.1）→ 对应 `ch:engineering` 电机层。
- §prac.4.2 单腿层：静力学 J^T 映射（式 prac.2）、摆动腿修正力（式 prac.3）→ `\cref{ch:leg_kinematics}`。
- §prac.4.3 整机层：unitree_guide 的 QP 足底力分配 → `\cref{ch:balance_qp}`；legged_control 的 NMPC（式 prac.4）→ `\cref{ch:convex_mpc}`；WBC（式 prac.5）→ `\cref{ch:wbc}`。
- §prac.4.4 架构层：硬件抽象（句柄/接口/插件）、控制器生命周期（init/update/starting/stopping）、FSM。

## §prac.5 工程陷阱清单（`pitfall` 集）
- 关节名称硬编码必须一致；URDF 路径用 `package://`；碰撞体须用简单几何体；姿态/零点三层概念；`/tmp/legged_control/<type>.urdf` 必须生成。

## §prac.6 练习（`practice` 集）
- 部署自定义四足到 Gazebo（URDF→xacro→transmission→Gazebo 插件→姿态配置）。
- 把混合控制式 (2.1) 与 legged_control 的 `setCommand(q,dq,Kp,Kd,tau)` 字段一一对应。

## §prac.7 强化学习环境（点到为止）
- Mujoco 源码编译、Docker 镜像，仅给出可复现入口与常见报错，正文不展开。

---

# 二、逐条收录的关键公式（公式少、来由清晰、出处明确）

## 式 prac.1 —— 关节电机混合控制（PD + 前馈）★核心
出处：[宇树书·ch2] §2.3 式 (2.1)，源行 324。`\cite{unitree2023quadruped}`
$$
\tau = \tau_{\mathrm{ff}} + k_{p}\,(p_{\mathrm{des}} - p) + k_{d}\,(\omega_{\mathrm{des}} - \omega)
$$
- **来由**：电机底层只接受力矩目标；机器人却要同时给位置/速度/力矩 → 用一个 PD 把位置/速度偏差折算成力矩，叠加前馈力矩。$\tau_{\mathrm{ff}}$=前馈力矩，$p_{\mathrm{des}},\omega_{\mathrm{des}}$=期望转子角/角速度，$k_p$=位置刚度，$k_d$=速度刚度（阻尼）。
- **5 个控制指令**：$\tau_{\mathrm{ff}},p_{\mathrm{des}},\omega_{\mathrm{des}},k_p,k_d$（源行 311–319）。
- **主线锚点**：这正是 legged_control 里 `HybridJointInterface` 要一次性传 5 个量、而 `ros_control` 标准接口（位置/速度/力矩单一）传不了的原因（[笔记·legged] 行 1632）。底层最终仍是力矩控制 `EffortJointInterface`。
- **退化模式**（`note`）：阻尼模式 = $k_p=0,\tau_{\mathrm{ff}}=0,p_{\mathrm{des}}=\omega_{\mathrm{des}}=0,k_d=8$（[宇树书·ch3] 行 931，纯阻尼安全姿态）。
- **\cref**：电机 FOC/减速比换算→`ch:engineering`。**笔记盒提醒**：ch2 参量针对转子（含减速比），ch3 关节命令针对关节（无需减速比）（[宇树书·ch3] 行 942）。

## 式 prac.2 —— 单腿静力学（关节力矩 ↔ 足端力）
出处：[宇树书·ch2/ch5] §5.3.2 式 (5.46)/(5.47)，源行 2906/2910。
$$
\boldsymbol\tau = \boldsymbol J^{\mathrm T}\boldsymbol F,
\qquad
\boldsymbol F = \boldsymbol J^{-\mathrm T}\boldsymbol\tau
$$
- **来由**：虚功原理 $\boldsymbol\tau^{\mathrm T}\dot{\boldsymbol q}=\boldsymbol F^{\mathrm T}\boldsymbol J\dot{\boldsymbol q}$（源行 2900）对任意 $\dot{\boldsymbol q}$ 成立 ⇒ $\boldsymbol\tau=\boldsymbol J^{\mathrm T}\boldsymbol F$。$\boldsymbol F$ 是足端**对地**作用力。
- **作用反作用**（`note`）：若用地面对足端力 $\boldsymbol F'$，则 $\boldsymbol\tau=-\boldsymbol J^{\mathrm T}\boldsymbol F'$（源行 2913）。这条与平衡控制器式 (8.39) $\boldsymbol\tau_i=-\boldsymbol J_i^{\mathrm T}\boldsymbol R_{sb}^{\mathrm T}\boldsymbol f_{is}$（源行 4549）一致——把世界系足底力转到关节力矩，即 QP/MPC 输出 → 电机指令的「最后一公里」。
- **\cref**：`ch:leg_kinematics`（雅可比）、`ch:balance_qp`（足底力来源）。

## 式 prac.3 —— 摆动腿足端修正力（笛卡尔 PD）
出处：[宇树书·ch5] §5.4 式 (5.48)，源行 2922。
$$
\boldsymbol f_{d} = \boldsymbol K_{p}\,(\boldsymbol p_{0d}-\boldsymbol p_{0f}) + \boldsymbol K_{d}\,(\dot{\boldsymbol p}_{0d}-\dot{\boldsymbol p}_{0f})
$$
- **来由**：仅控关节角/角速度时足端误差难修正 → 在笛卡尔空间对足端位置/速度再加一层 PD，得修正力 $\boldsymbol f_d$，再经式 prac.2 折算成关节力矩 $\boldsymbol\tau=\boldsymbol J^{\mathrm T}\boldsymbol f_d$。$\boldsymbol K_p,\boldsymbol K_d$ 为正定对角阵。
- **工程提示**（`practice`）：摆动腿目标速度设 0 → 操纵足端有较大延迟属正常；调参只需避免足端抖动/位置振荡（源行 2977）。
- **\cref**：`ch:gait`（摆线轨迹给 $\boldsymbol p_{0d}$）。

## 式 prac.4 —— legged_control 的 NMPC 最优控制问题（OCS2 formulation）
出处：[笔记·legged] 行 1563–1571。`\cite{sleiman2021unified}`
$$
\begin{aligned}
\min_{\boldsymbol u(t)}\;& \phi(\boldsymbol x(t_f)) + \int_{t_0}^{t_f} l(\boldsymbol x(t),\boldsymbol u(t),t)\,\mathrm dt\\
\text{s.t.}\;& \boldsymbol x(t_0)=\boldsymbol x_0,\quad \dot{\boldsymbol x}(t)=\boldsymbol f(\boldsymbol x(t),\boldsymbol u(t),t),\\
& \boldsymbol g_1(\boldsymbol x,\boldsymbol u,t)=\boldsymbol 0,\quad \boldsymbol g_2(\boldsymbol x,t)=\boldsymbol 0,\quad \boldsymbol h(\boldsymbol x,\boldsymbol u,t)\ge\boldsymbol 0
\end{aligned}
$$
- **状态/输入**（行 1575）：$\boldsymbol x=[\boldsymbol h_{\mathrm{CoM}}^{\mathrm T},\boldsymbol q_b^{\mathrm T},\boldsymbol q_j^{\mathrm T}]^{\mathrm T}$（质心动量、机体广义坐标、关节角）；$\boldsymbol u=[\boldsymbol f_c^{\mathrm T},\boldsymbol v_j^{\mathrm T}]^{\mathrm T}$（地反力、关节速度）。
- **约束三条**（行 1580）：地反力在摩擦锥内；触地腿不动；摆动足 z 满足步态曲线。
- **求解链**（行 1586）：centroidal dynamics → Multiple Shooting → NLP → SQP，子 QP 用 **HPIPM**。
- **对照**（`insight`）：与宇树书的瞬时 QP（N=1 退化）相比，这是带预测时域的完整 NMPC。`\cref{ch:convex_mpc}`。

## 式 prac.5 —— legged_control 的 WBC 优化变量与分层
出处：[笔记·legged] 行 1594。`\cite{bellicoso2016whole}`
$$
\boldsymbol x_{\mathrm{wbc}} = [\ddot{\boldsymbol q}^{\mathrm T},\,\boldsymbol f_c^{\mathrm T},\,\boldsymbol\tau^{\mathrm T}]^{\mathrm T}
$$
- **来由**：WBC 只考虑当前时刻，把多个任务（每个=对优化变量的等式/不等式约束）按优先级求解；高优先级等式约束的零空间内最小化低优先级松弛 → 严格优先级，同时满足浮动基多刚体动力学求全电机最优扭矩（行 1596）。
- **\cref**：`ch:wbc`。

## 式 prac.6 —— 前馈力矩报文幅度上限（工程，凝练登记）
出处：[宇树书·ch2] §2.6 式 (2.2)，源行 604。
$$
|\tau_{\mathrm{ff}}| < \frac{2^{15}}{256} = 128
$$
- **来由**：2 字节 signed short，×256 倍描述，符号位占 1 位 ⇒ 数值上限。仅作 `practice`/附录登记，不在正文推导。`\cref{ch:engineering}`。

> **公式收录小结**：核心 3 式（prac.1/2/3）+ 架构 2 式（prac.4/5）+ 工程 1 式（prac.6）= **6 条**。其余宇树书 ch2/ch3 公式（FOC、报文）全部凝练登记，不录。

---

# 三、个人笔记中的「作者思路 / 洞见」点（本章主线 + insight 盒）

> 这些是教科书不直述的工程哲学，作主线骨架，逐条标「作者洞见」做 `insight`。

1. **〔主线·硬件抽象〕** 控制器不知道自己面对仿真还是实物，只和 `hardware_interface` 标准接口通信；仿真器 `legged_gazebo` 假扮真实硬件 → 一次代码两处复用，避免为两种情况分别编程（行 1616）。**全章串联骨架。**
2. **〔句柄 vs 接口〕** 句柄=指向状态/命令缓冲区的指针+安全检查（零拷贝实时交互）；接口=同类句柄的容器+资源声明/冲突检查（注册与查找）（行 1626–1628）。这是「资源管理」视角，呼应宇树书 OOP 三核心。
3. **〔为何要自定义 HybridJointInterface〕** 现代四足都用 PD 混合控制，要一次传 5 个量，`ros_control` 任一标准接口都传不了，故自定义；但底层仍是 `EffortJointInterface` 力矩控制（行 1632）。直接锚式 prac.1。
4. **〔IMU 不走 ROS 话题〕** 话题=网络传输+延迟+抖动+异步，高频控制不可接受；改走 `Gazebo→readSim()→ImuSensorInterface（C++ 共享内存）→update()`，与关节角/接触数据**完美对齐同一时间戳**（行 1759–1761）。`insight`。
5. **〔被动控制器 vs 主动管理器〕** `LeggedController` 是被动类，只定义回调（init/update/starting/stopping）；`ControllerManager` 是主动执行者，按 read→update→write 顺序高频调用（行 1974–1994）。理解生命周期的关键比喻。
6. **〔read/update/write 三段〕** write 阶段 `setCommand` 只写缓冲区，update 完成后 `write()` 才真正打包发给电机（行 1992）。解释「为什么命令不是立即生效」。
7. **〔严格分离实时/非实时〕** 控制环路内部不用 ROS 通信，只有对外（接收目标点）才用 ROS → 严格隔离实时与非实时任务（行 1559）。
8. **〔MPC 单独线程〕** MPC 频率远低于 WBC（=update），故 `setupMrt()` 启动 MPC 后台线程（行 2035）。解释级联 MPC+WBC 的频率解耦。
9. **〔标准垂直构型的取舍〕** 膝在髋正下方、髋在侧摆左右方——与真机有差异但**计算容易**；自定义非标准构型会在碰撞检测/惯量变换上「带来麻烦」，建议设计之初就用标准垂直构型（行 1806、1847）。`insight`+`pitfall`。
10. **〔碰撞体用简单几何体〕** Gazebo 底层用几何基元判交集；机身用长方体、髋用圆柱、大小腿用长方体、足端用球——不精确但极大降算量；STL 网格做碰撞体会让 OCS2 超时乱动（行 1816、1901、1937）。`insight`+`pitfall`。
11. **〔姿态/零点三层概念〕** 运动学零点（URDF q=0 参考构型，选对称中位数值稳定姿态）/ 机械·传感器零点（装配限位、编码器 0）/ 控制零点（机械零点+offset 映射到运动学零点）；初始姿态 `initialState` vs 参考姿态 `defaultJointState`+`comHeight`；配置错→乱动/抖动（行 2401–2431）。`insight`+`pitfall`。
12. **〔关节名称硬编码哲学〕** `LeggedController::init` 硬编码 "LF_HAA"… 12 个名；xacro 与控制器代码必须完全一致否则无法控制（行 1855、2013、2052）。`pitfall`。
13. **〔A1QP 双线程〕** `MainGazebo.cpp` 分力控线程（`update_foot_forces_grf`，算期望地面支撑力）与主更新流程（`main_update`+`send_cmd`，急停/期望状态/模式切换/步态/估计）（行 1535–1537）。最小可读的 MPC 工程范例。

---

# 四、OCR / 转写可疑处清单（原印 vs 推断订正）

> MinerU 与个人笔记转写均可能含 OCR 瑕疵。逐条标 `note`「OCR订正」。

1. **[宇树书 行 324]** 式 (2.1) MinerU 渲染含多余空格 `k _ {\mathrm{p}}`、`\omega_ {\mathrm{des}}`。**订正**：$\tau=\tau_{\mathrm{ff}}+k_p(p_{\mathrm{des}}-p)+k_d(\omega_{\mathrm{des}}-\omega)$。属排版空格，物理无误。
2. **[宇树书 行 604]** 式 (2.2) 印作 `\frac{2^{15}}{256}=128`，数字间含空格 `1 2 8`、`2 ^ {1 5}`。**订正**：$|\tau_{\mathrm{ff}}|<2^{15}/256=128$。正确。
3. **[宇树书 行 2906/2910]** 式 (5.46)/(5.47) 大量空格化下标 `\tau_ {1}`、`J^{-T}`。**订正**：$\boldsymbol\tau=\boldsymbol J^{\mathrm T}\boldsymbol F$、$\boldsymbol F=\boldsymbol J^{-\mathrm T}\boldsymbol\tau$。注意 $J^{-T}=(J^{\mathrm T})^{-1}$，成书用 $\boldsymbol J^{-\mathrm T}$。
4. **[宇树书 行 4549]** 式 (8.39) $\boldsymbol\tau_i=-\boldsymbol J_i^{\mathrm T}\boldsymbol f_{ib}=-\boldsymbol J_i^{\mathrm T}\boldsymbol R_{bs}\boldsymbol f_{is}=-\boldsymbol J_i^{\mathrm T}\boldsymbol R_{sb}^{\mathrm T}\boldsymbol f_{is}$，OCR 多空格。**注意全局约定**：本书右扰动主线、旋转记 $\boldsymbol R_{sb}$（机身→世界），$\boldsymbol R_{bs}=\boldsymbol R_{sb}^{\mathrm T}$，与宇树书一致，无需换算。
5. **[笔记 行 1575]** NMPC 状态 $x=[\mathbf h^T_{CoM},\mathbf q^T_b,\mathbf q^T_j]^T$ 与输入 $u=[\mathbf f^T_c,\mathbf v^T_j]^T$ 上标 T 紧贴下标，易读作 $h^T_{CoM}$。**订正**：$\boldsymbol x=[\boldsymbol h_{\mathrm{CoM}}^{\mathrm T},\boldsymbol q_b^{\mathrm T},\boldsymbol q_j^{\mathrm T}]^{\mathrm T}$。
6. **[笔记 行 1564]** NMPC 代价 `\int l(...) dt` 缺 `\,`；约束中混入中文「初始条件」等注释串入公式块。**订正**：成书时中文移到公式外说明，公式体保留式 prac.4 形式。
7. **[宇树书 行 2925]** $\tau=J^{T}f_{d}$ 与 $\boldsymbol K_p,\boldsymbol K_d$ 对角阵——OCR 正确，仅符号粗细需统一为粗体向量/矩阵。
8. **[笔记 行 2561]** ONNXRuntime cmake 命令被反斜杠转义打散（`onnxruntime\_BUILD`）。属转写，工程命令，正文不录，仅附录登记。

> **OCR 总体判定**：本章公式少且简单，OCR 风险低；主要是 MinerU 的「下标/上标空格化」与「中文串入公式块」两类，均据物理意义订正，无影响结论的错误。

---

# 五、三源对同一主题的差异 / 互补点（供融合）

| 主题 | unitree_guide（宇树书） | legged_control（笔记） | A1-QP-MPC（笔记） | 融合写法 |
|---|---|---|---|---|
| **控制范式** | 瞬时 QP 足底力分配（**无 MPC/无预测**）+ 简化逆动力学 | NMPC（OCS2/SQP/HPIPM）+ 分层 WBC | 凸 MPC（Cheetah 3 复刻） | 以「预测能力」为轴：QP(N=1) → 凸MPC → NMPC，呼应 §prac.4.3 |
| **仿真/实物切换** | OOP 动态绑定：`IOInterface*` 指基类，指向 `IOROS`/`IOSDK` | 硬件抽象插件：`LeggedHWSim` 假扮硬件，controller 不知情 | 单一 Gazebo 主程序双线程 | 同一哲学两种实现（虚函数 vs 插件），作主线 `insight` |
| **架构骨架** | FSM（Passive/FixedStand/FreeStand/Trotting/move_base），状态 4 函数 enter/run/exit/checkChange | 控制器生命周期 init/update/starting/stopping + ControllerManager | 双线程（力控+主更新） | 「状态机」vs「生命周期回调」对照表 |
| **关节命令** | `MotorCmd{mode,q,dq,tau,Kp,Kd}`，式 (2.1) | `HybridJointInterface.setCommand(...)`，底层 `EffortJointInterface` | `update_foot_forces_grf`→`send_cmd` | 同一 PD+前馈式 prac.1 的不同 API 封装 |
| **关节编号** | 显式约定：RF/LF/RH/LH × (ab/ad,hip,knee)=0..11；零角 (0,0.67,-1.3) | 硬编码名 LF_HAA/LF_HFE/LF_KFE…（注意命名体系不同！） | — | `pitfall`：两套命名体系不可混用 |
| **状态估计** | KF（ch7）+ 接触/QP 前置 | cheater（Gazebo 真值）/ KF 二选一（行 2119） | 含状态估计子任务 | `\cref{ch:state_estimation}` |
| **部署门槛** | ROS melodic/Gazebo9，教学最低 | Docker 镜像 + OCS2 重依赖，工业级 | 极简，入门 | 由易到难推荐路径 |
| **碰撞/模型** | unitree_ros 现成模型 | 自建：URDF→xacro→简单几何碰撞体（行 1937） | A1 现成 | 自建机器人完整流程只在 legged 讲（练习） |

> **互补关键**：宇树书给「最易懂的架构思想（OOP/FSM）+ 最完整的关节/单腿一手公式」；legged_control 给「工业级 NMPC+WBC 的真实部署细节（硬件抽象/生命周期/URDF/碰撞/零点）」；A1QP 给「最小可读的 MPC 工程范例」。三者按「教学→入门→工业」递进，**不是平行三段而是难度阶梯**，这是本章组织的核心融合策略。

---

# 六、编译安全备忘（写正文时遵守）

- 可用环境（已 Read styles.tex 确认）：`derivation`、`insight`（洞见，可 \cref）、`paper`（论文精读）、`pitfall`（陷阱）、`practice`（练习）、`note`/`remark`/`definition`/`theorem`（ElegantBook 自带）、`finenote`（小字段落）。
- `\cnum{1}` 已全局定义画圈数字，**勿在章内重定义**。
- 中文不裸入 math mode → 用 `\text{}`；NMPC 约束块的「初始条件」等中文移出公式。
- 代码块用 styles.tex 的 tcolorbox+listings；本章代码多，正文只留**最小代表性片段**（如 `IOInterface* ioInter;` 动态绑定 3 行、`setCommand` 字段、FSM run 核心 if），其余 `\cref{ch:engineering}`。
- 表格 `\centering`；图用 TikZ 重绘（架构图：controller↔hardware_interface↔(Gazebo/真机)；FSM 状态转移图；read-update-write 时序）；原始截图 URL 不可用，**全部 TikZ 重绘或省略**。
