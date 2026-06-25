# Robot 工程吸收 · legged_control_ros2（双足点足 + 四足）

> 来源项目：`/home/ziren2/pengfei/Robot/legged_control_ros2`（**只读吸收**，源码未改动）。
> 规模：182 md / 1720 code / 832M（四大项目中内容最多）。本文是其全部有价值内容的去重·结构化·凝练。
> 体量分布：`docs/quadruped/`（8 篇技术 curated 文档，Go1 四足 MPC+WBC+估计+sim）+ `docs/research/braver_locomotion/`（一套"把四足栈移植到点足双足 Braver"的研发侦探故事，~130 个 phase 蒸馏为 200–206 教训文档 + 124–130 里程碑）。
> **本文最大价值在第 3 章（工程实战与踩坑/失败经验）**——尤其 §3.6 那条"花了 ~120 个 phase 才定位的一个被置零的反馈增益"的根因复盘。

---

## 0. 一句话全景

把一套基于 **OCS2（非线性 MPC）+ Pinocchio + WBC（全身 QP）+ MuJoCo** 的腿式控制栈，用于：
1. **Go1 四足**（12 关节）——curated 技术文档的主体，讲清 MPC/WBC/估计/感知 locomotion 的实现与公式；
2. **Braver 点足双足**（2 腿 6 关节，stateDim=18/inputDim=12/WBC=24）——从四足栈移植，目标"平地稳定行走 + 跟踪 `/cmd_vel`"。移植过程踩坑极深，最终突破：确定性周期 gait + 纯 MPC 落脚，0.1–0.9 m/s 稳定行走，速度跟踪误差 1e-4 量级。

三机型同框架（Go1 / ANYmal C / Braver），还含一条**感知 locomotion 栈**（self-filter → elevation mapping → plane segmentation → perceptive MPC）。

---

## 1. 项目概览（目标 / 技术栈 / 架构）

### 1.1 三层控制架构（通用于全部机型）

| 层 | 模块 | 频率 | 作用 | 求解器 |
|---|---|---|---|---|
| **MPC** | `*_interface`（定义 OCP：动力学/代价/约束） | 100 Hz（独立线程） | 求未来 ~1s 最优轨迹（state+input） | SQP + HPIPM |
| **WBC** | `*_wbc` | 500–1000 Hz | 把质心级最优轨迹 → 具体 12/6 关节力矩 | Hierarchical QP（HoQP/qpOASES）或 Weighted QP |
| **PD + Sim/HW** | `*_mujoco_sim`（Python） | 1000 Hz | 电机执行 `τ = τ_ff + Kp(q_des−q) + Kd(v_des−v)` | — |

数据流：`MuJoCo → /jointsPosVel,/ground_truth/state,/imu → 状态估计 → centroidal state(24/18D) → MPC(MRT) → optimizedState/Input → WBC → torque/posDes/velDes → /targetTorque... → MuJoCo PD`。

### 1.2 包职责（Go1 / 通用化命名）

- `*_interface`（C++）：定义优化问题——动力学、代价、约束、gait schedule、初始猜测。
- `*_controllers`（C++）：中央控制器节点，连 MPC↔WBC↔估计；跑 update 循环、发命令。
- `*_estimation`（C++）：状态估计（base 位姿/速度、接触力、早晚触地）。
- `*_wbc`（C++）：全身控制 QP。
- `*_legged_description`（URDF/MJCF）：几何/质量/惯量。
- `*_mujoco_sim`（Python）：MuJoCo 仿真 + ROS2 接口。
- `*_dummy` / `gait` 节点：gait 命令 + 目标轨迹发布。
- 感知：`legged_perceptive_*`（perceptive interface/controller）、`robot_self_filter`、`elevation_mapping_cupy`、`convex_plane_decomposition_ros`。
- 第三方：`ocs2_ros2`、`qpoases_catkin`。

### 1.3 关键维度公式（移植腿式机器人通用，从 `CentroidalModelInfo` 读取而非硬编码）

```
actuated DoF = 每腿关节数 × 腿数
numContacts  = 腿数（点足/3DoF 接触）
stateDim  = 6(归一化质心动量) + 6(base 位姿:位置+ZYX欧拉) + actuatedDof
inputDim  = 3*numContacts(接触力) + actuatedDof(关节速度)
WBC 决策变量 = generalizedCoords + 3*numContacts + actuatedDof
RBD state = 2*generalizedCoords
```
- Go1 四足：stateDim=24, inputDim=24, WBC=42, RBD=36, numContacts=4。
- Braver 点足双足：stateDim=18, inputDim=12, WBC=24, RBD=24, numContacts=2。

### 1.4 关节顺序铁律（极易出错，见 §3.1 Bug 7）

Go1 全系统统一 **LF → LH → RF → RH**，每腿 HAA/HFE/KFE。此顺序必须在 URDF（决定 Pinocchio frame 索引）、`ModelSettings.h`、`task.info`（initialState/Q/R）、`reference.info`、MuJoCo 映射表 **五处一致**。

---

## 2. 核心方法与算法（分模块）

### 2.1 MPC / 最优控制（OCS2）

**OCP 离散形式**（multiple shooting，直接转写）：
$$\min_{\{x_k,u_k\}}\sum_{k=0}^{N-1}\ell(x_k,u_k)\Delta t+\Phi(x_N)\quad\text{s.t.}\quad x_{k+1}=f_d(x_k,u_k),\ g=0,\ h\ge0$$
- 决策向量 stack 后 `z=[x_0..x_N, u_0..u_{N-1}]`；动力学变成 defect 约束 `x_{k+1}−f_d(x_k,u_k)=0`；KKT 系统呈**时间块带状**，HPIPM 利用此结构高效求解。
- **SQP**：每次迭代把非线性 KKT 局部二次近似成 QP（线性化动力学 A/B + 代价二阶 H/g + 线性化约束），求 Δz，line-search/merit 全局化。OCS2 实时配置很激进：`sqpIteration=1`（Go1）/`=3`（Braver），靠 warm-start。

**★ OCS2 的 OCP「五块拼图」与 ReferenceManager 的设计因果链（架构最重要的设计动机，移植/扩展前必须建立的心智模型）**：要把抽象 OCP 落到一台具体腿式机器人，OCS2 工程上把它拆成**必须分别提供的五块**：① **动力学** `f`（质心模型）② **代价** `ℓ/Φ`（往 `costPtr->add` 叠加多项）③ **约束** `g/h`（逐脚 add，分等式/不等式/软约束容器）④ **参考** `x_ref`（`/cmd_vel`→TargetTrajectories→未来目标 base 轨迹）⑤ **模式调度** ModeSchedule（gait）。装配入口 `*Interface::setupOptimalControlProblem` 的全部工作就是把 `task.info`/URDF/`reference.info` 三文件「编译」成一个 `OptimalControlProblem` 容器 + 一个 `ReferenceManager`——它**不求解、不下发**。
  - **静态/动态分界线 = ReferenceManager**：前三块（动力学/代价/约束）是**相对静态**的——开机装配一次、运行中基本不变，直接挂在 `problemPtr_` 上；后两块（参考/模式调度）是**动态**的——随指令在线变，**不挂 problem，而封装进 `referenceManagerPtr_`**。
  - **为何必须如此（关键因果链）**：SQP/DDP 求解器**为并行（多线程做 rollout/线性化）给每个工作线程持有一份 OCP 的拷贝**。后果有二——(a) 你在外面改原始 `problemPtr_` 的参考值，**求解器线程用的是自己的副本根本看不到 = 改了等于没改**；(b) 即便共享字段，前台写后台多线程读 = **数据竞争**，且可能拼出「半新半旧」不自洽问题致策略发散。于是 OCS2 把 OCP 设计成「可被自由拷贝的纯问题描述」以换取安全并行，代价是失去「直接改 problem 更新运行时参数」的能力，必须另给一条受框架管理、线程安全、所有副本都能看到的更新通道：**ReferenceManager + SolverSynchronizedModule**。落地形态是三段式——**ROS 回调线程只加锁缓冲 → 求解前回调 `preSolverRun` 安全写入 → 求解器 `modifyReferences` 查询**。一句话因果链：**为了并行 → OCP 必须可拷贝 → 不能直接改 problem → 需要 ReferenceManager/SynchronizedModule**。
  - **直接推论（解释 §3.5/§3.6 的「同步边界是落脚/mode 更新的唯一正确位置」与 §3.7 的「运行时注入 gait 有 ~1 horizon 延迟」）**：任何「在线改 OCS2 行为」的功能（换步态、改参考、注入落脚 XY）都必须套这个三段式经 ReferenceManager 灌入；post-MPC 在 WBC 层注入或把动态参考焊死在骨架代价里，都违背此分界 → 改了没反应或与 MPC reference 打架。

**状态/输入（Go1 24D）**：
- `x=[v_com(3), ℓ/m(3), p_base(3), θ_ZYX(3), q_joints(12)]`；Q 权重：p_z=1500、pitch=1500/300（doc 两处取值不同）、p_xy=500、yaw=100、v_com=15/100、关节 2.5–5（腿留自由度让 MPC 自己找 gait）。
- `u=[f_contacts(12), q̇_joints(12)]`；R：接触力 1e-3（几乎自由），关节速度 5000（**经足 Jacobian 变换**）。

**关键工程技巧**：
- **R 矩阵 Jacobian 变换**：task.info 里 R 是 task-space 足速度惩罚，但优化变量是关节速度 `q̇`。因 `v_foot=J·q̇`，故 `R_joint = Jᵀ R_task J`（取 base→foot Jacobian 的关节列 6..17）。
- **u_nominal ≠ 0**：是重力补偿力 `weightCompensatingInput`——`Fz_per_leg = m·g / numStanceLegs` 均摊到各支撑脚。所以"负 stance Fz"不能归因于 R 把 input 拉向零。
- **质心动力学**：状态存 `x=[h̄; q]`，`h̄=h_G/m`。`ḣ_G` 只由**外部力矩**决定（`ṗ=Σf+mg`，`ℓ̇=Σ r_i×f_i`，无关节项，因关节驱动是内力）。关节通过**几何 r_i(q)** 与**动量映射 A(q)v=A_b v_b + A_j q̇** 间接影响；摆腿产生角动量靠 base 反向运动维持总动量守恒。
- **centroidalModelType**：`0`=Full（每次算完整 A(q)，重建 v_b 时减 A_j q̇，更"全身"但慢/数值敏感）；`1`=SRBD（A_b 用名义惯量+名义 CoM-base 偏移预算一次，`v_b≈A_b⁻¹ h_G` 忽略关节动量贡献，快/稳健，实时 MPC 首选）。SRBD 改的是动量映射近似，不改力平衡。
- **CppAD 自动微分**：动力学/约束的 Jacobian 由 CppAD 记录"tape"→ 编译成 `.so`（缓存 `/tmp/ocs2/<robot>`）→ 后续同时算 f 与 ∂f/∂x,∂f/∂u。代码生成慢，故线程压 1–2。

**4 类约束**：① 摩擦锥（软 relaxed barrier：`μ(Fz+Fgrip)−√(Fx²+Fy²+ε)≥0`）② Zero-Force（swing 脚 f=0，等式）③ Zero-Velocity / no-slip（stance 脚 `v_foot=Jq̇=0`）④ Normal-Velocity（swing 脚 z 速度跟随 cubic spline 抬腿轨迹，liftOff/touchDown 两段，swingHeight≈8cm；braver `liftOffVelocity=0.05/touchDownVelocity=−0.1/swingTimeScale=0.15`）。

**★ Go1 与 Braver 的真实配置差异（移植/调参直接可用，文档其余处多为 Go1 值，对双足是错的）**：
- **摩擦系数**：Go1 用 `μ=0.3`（保守）；**Braver 实际 `frictionCoefficient=0.6`**——且 OCS2 软锥 `frictionConeSoftConstraint.frictionCoefficient` 与 WBC `frictionConeTask.frictionCoefficient` 必须**一致**（否则两层可行集失配），并须 ≤ 仿真 MJCF 地面 μ（大于则规划出"打滑解"实际打滑摔）。
- **命名陷阱（新人必踩）**：`frictionConeSoftConstraint.mu=1.0` 里的 `mu` 是 `RelaxedBarrierPenalty` 的**惩罚权重**（配 `delta` 软化半径），**不是摩擦系数**。把它当摩擦系数调大 → 摩擦可行集没变、反而把软约束惩罚调重使 Hessian 更刚、求解病态。改"红线在哪"动 `frictionCoefficient`；改"越红线罚多重"才动 `mu/delta`。
- **Braver Q/R（点足无静稳→姿态必须压紧）**：base 位姿段 `Q[6:12]=[1000,1000,2000,100,400,400]`（**z=2000 全表最大**=守站高，pitch/roll=400 压姿态，刻意偏大）；动量段 `Q[0:6]=[15,15,100,20,40,30]`（竖直动量 100 最大）；关节段 `Q≈[5,5,2.5]×2`；接触力段 `R[0:6]` 全 1.0；关节速度段 `R=5000`；`R.scaling=1e-3`（输入比状态便宜 3 个量级，MPC 几乎"免费"用力/动关节，重点全在摁住状态）。
- `comHeight=0.42737`、`sqpIteration=3`（非 Go1 的 1，靠 warm-start）、`timeHorizon=1.0s`（须 ≥ 一个步态周期 0.6s）、`defaultModeSequenceTemplate=standing_trot`（见 §3.7：必须改默认序列才能"从一开始就走"）。

**Gait schedule（switched model）**：4-bit mode（Go1）每脚 1 bit（0 swing/1 stance），`mode=LF+2RF+4LH+8RH`；mode 决定每个 node 哪些约束激活。8 种 gait：stance/trot(9,6 对角)/standing_trot(带 STANCE 过渡)/flying_trot(带 FLY 飞行相)/pace/standing_pace/dynamic_walk(8 相,3 脚支撑)/static_walk(始终 3 脚支撑,最稳)。

**写自定义 cost/constraint 的三个高频踩坑（移植/自定义 OCP 时最易犯）**：
1. **Hessian 必须 PSD**：照实算二阶导填 `dfdxx/dfduu`、或 `task.info` 把 Q/R 配成非对称/含负对角 → Hessian 非半正定 → SQP 子问题非凸、KKT 不可逆、求解失败（日志见 "Hessian not positive definite"）。正解用 **Gauss-Newton**：残差线性化 `J`，`Hessian=JᵀWJ`（`W={Q,R}`）天然 PSD，再加 `hessianDiagonalShift` 微量对角移位保严格凸。
2. **"谁让步"决定 cost / 硬约束 / 软约束**：红线（脚不打滑）写成软代价 → 求解器判"宁可稍打滑也追参考"时主动牺牲它 → 失稳；偏好（省电）写成硬不等式 → 被推一下需大力恢复时 QP 不可行。
3. **写完约束必须重写 `isActive`**：摩擦锥/ZeroForce 写完不重写 `isActive`（用默认 `true`）→ 约束在错误模式下生效（摩擦锥应 contact 时 active、ZeroForce 应 !contact 时 active）→ 机器人塌陷或抖。

### 2.2 WBC（全身控制）

决策变量 `x=[q̈_gen(18); f_contact(12); τ(12)] ∈ ℝ⁴²`（Go1）。**7 个 task**：

1. **FloatingBaseEoM**（等式，最高优先）：`M q̈ + h = J_cᵀ f + Sᵀ τ`，写成 `[M | −J_cᵀ | −Sᵀ]x = −h`。M 来自 `crba`，h(科氏+重力) 来自 `nonLinearEffects`，S=[0|I] 选择矩阵。
2. **TorqueLimits**（不等式）：`−τ_max ≤ τ ≤ τ_max`。Go1：HAA/HFE=23.7Nm, KFE=35.55Nm。
3. **NoContactMotion**（等式）：stance 脚加速度=0 → `J_i q̈ = −J̇_i q̇`。
4. **FrictionCone**：swing 脚 f=0（等式）+ stance 脚**线性金字塔**（每脚 5 行：`−Fz≤0`、`±Fx−μFz≤0`、`±Fy−μFz≤0`）。
5. **BaseAccelTask**：从质心动量算 base 期望加速度 `q̈_base = A_b⁻¹(ḣ_des − Ȧq̇ − A_j q̈_j)`（**注意：纯前馈写法对点足致命，见 §3.6**）。
6. **SwingLegTask**：swing 脚 PD `a_foot = Kp(p_des−p)+Kd(v_des−v)`，再 `J_i q̈ = a_foot − J̇_i q̇`。swingKp=350/Kd=37(≈2√350)。
7. **ContactForceTask**（最低优先）：`f = f_MPC`。

**两种求解器**：
- **HierarchicalWbc（HoQP）**：3 层严格优先级。Task0(EoM+limits+friction+noslip) > Task1(baseAccel+swing) > Task2(force)。核心是**零空间投影**：`x_k = x_{k-1} + Z_{k-1} z_k`，每层在上层零空间求解，`Z_new = Z_prev · kernel(A·Z_prev)`（LU 求核），保证不破坏高优先级 task。底层用 qpOASES（`options.setToMPC()`, nWsr=20）。
- **WeightedWbc**：单个加权 QP，`min Σ w_i‖A_i x − b_i‖²` s.t. 硬约束（EoM+torque+friction）。权重 swingLeg=100/baseAccel=1/contactForce=0.01。速度快、需调权重、约束保证较弱。Go1 实际用 Weighted。

**输出**：`τ = x[30:42]`，posDes/velDes 从 optimizedState/Input 取，发给 PD（Go1: Kp=0/Kd=3 ≈ 纯前馈力矩，Kp=0 因 WBC 已算全力矩，加位置反馈会干扰）。

### 2.3 状态估计

- **rbdState（36D，Go1）**：`[θ_ZYX(3), p_base(3), q_joints(12), ω_base(3), ṗ_base(3), q̇_joints(12)]`。
- **两种估计器**：
  - `FromTopicEstimate`（cheater，仿真用）：直接读 `/ground_truth/state`，quat→Euler ZYX。
  - `KalmanFilterEstimate`（实机）：EKF，状态 `[p_base, v_base, p_foot1..4] ∈ ℝ¹⁸`；预测用 IMU（`v̇=R·a_imu+g`），更新用 FK（stance 脚位置固定→观测）。
- **IMU 处理**：quat→ZYX 减 bias；局部角速度→全局 `ω_global=T_ZYX·ω_local`。
- **控制器两个运行时实现细节（腿式常见、不做就出诡异 bug）**：(1) **关节速度低通滤波** `vel_filtered=kf·old+(1−kf)·new`（`kf=0.3`，降关节速度噪声）；(2) **yaw 解缠绕**：`state(9)=yawLast+shortest_angular_distance(yawLast, newYaw)`，避免 yaw 在 ±π 处跳变污染 MPC。
- **接触力估计（广义动量观测器）**：`p=Mq̇`，`σ=βp+Sᵀτ_cmd+Cᵀq̇−g`（β=(1−γ)/(γdt), γ=e^{−λdt} 低通），扰动力矩 `τ_dist=βp−σ̂`，`f_i=(S_i J_iᵀ)⁻¹ S_i τ_dist`。cutoffFreq=250Hz, contactThreshold=75N。
- **早/晚触地检测**：早触地=gait 说 swing 但已摆 >75% 且估计力 > 阈值；晚触地=gait 说 stance 但 <0.04s 且估计力 < 阈值。
- **rbdState→centroidalState**：用 OCS2 `CentroidalModelRbdConversions`（FK 算 CoM、质心动量矩阵算动量、按质量归一化）。

### 2.4 感知 locomotion（perceptive MPC，ANYmal C）

栈：`robot_self_filter`（滤掉机器人自身点云）→ `elevation_mapping_cupy`（高程图，GPU）→ `convex_plane_decomposition_ros`（平面分割→凸区域）→ perceptive interface（地形约束进 OCP）。

**盲 OCP → 感知 OCP**：在盲约束基础上加两组 per-foot **软约束**（`StateSoftConstraint` + `RelaxedBarrierPenalty`，非硬约束以防不可行）：
- **落脚点约束**：落在凸多边形可行域内。流程：Raibert 式名义落脚 `p_nom = p_ref − R_z(ψ)ᵀ o + √(h/g)(v_meas−v_des)`（反馈项 clamp）→ 选最优平面区域 → `GrowConvexPolygonInsideShape`（numVertices=16, growthFactor=0.9 留边距）→ 多边形转半空间 `A_poly p_2d + b_poly ≥ 0` → 平面系转世界系 `A_w = A_poly P R_pw`, `b_w = b_poly + A_poly t_pw,xy`。约束 `h_place = A_w p_foot(x) + b_w ≥ 0`，state-only，雅可比 `G = A_w ∂p_foot/∂x`。多边形每条边→半空间的精确构造（与代码一致）：`a_ℓ=[y_{ℓ+1}−y_ℓ; x_ℓ−x_{ℓ+1}]`、`b_ℓ=y_ℓx_{ℓ+1}−x_ℓy_{ℓ+1}`，再用下一顶点判内法向 `a_ℓᵀv_{ℓ+2}+b_ℓ<0 ⇒ (a_ℓ,b_ℓ)←(−a_ℓ,−b_ℓ)` 翻转。
- **碰撞约束（SDF）**：`h_coll = d(p_foot(x)) − c ≥ 0`（clearance c=0.05m），swing 优先激活——active-set 时间窗 `¬c_i(t) ∧ ¬c_i(t+0.025) ∧ ¬c_i(t−0.05)`（仅在脚即将/刚离触地的窗口内启用碰撞约束）。
- **地形自适应 base 参考**：从平面法向推 pitch/roll（`θ_pitch=atan2(n_x,n_z)` 等），`z_base = z_terrain + h_com/max(n_z,ε)`。

惩罚配置：placement `RelaxedBarrierPenalty(1e-2,1e-4)`、collision `(1e-2,1e-3)`。QP 层这些只是节点上的 state-only 局部几何修正。

### 2.5 MuJoCo 仿真桥

- 1000Hz 物理，500Hz 发传感器，60Hz 渲染，40Hz 发 debug 力矩。
- 电机 PD：`τ = τ_ff + Kp(q_des−q) + Kd(v_des−v)`。
- 话题：发 `/jointsPosVel(24/12)`,`/ground_truth/state`,`/imu`,`/realTorque`,`/pauseFlag`；收 `/targetTorque,/targetPos,/targetVel,/targetKp,/targetKd`。
- pause 模式：仍发传感器（vel=0, accel=gravity），controller 不崩、恢复平滑。

---

## 3. 工程实战与踩坑 / 失败经验（**用户最看重，逐条 现象→根因→解决/教训**）

### 3.1 移植 G1→Go1 的 9 个 bug（每条都是"换机器人"通用教训）

1. **URDF mesh 路径未改**：`.urdf` 里 `<mesh>` 仍指向 g1 包名 → RViz 不显示/解析失败。**教训**：换机器人必须全局替换 URDF/MJCF 里所有包名。
2. **启动竞态首拍 NaN**：`FromTopicEstimate` 订 `/ground_truth/state`，控制循环先于首条消息跑→空 buffer→state=0→quat(0,0,0,0)→姿态算出 NaN→传遍 MPC→WBC→力矩 NaN→机器人秒摔。**修复**：构造时用物理有效值初始化 buffer（`z=comHeight`、`quat.w=1` 单位四元数）。**教训**：buffer/状态必须初始化为物理有效值。
3. **MuJoCo 关节序 ≠ URDF**：腿往错方向伸、乱动。MJCF 导出的 joint 序与 controller(LF→LH→RF→RH) 不同。**修复**：`go1_sim.py` 建 4 张映射表（qpos↔ctrl 双向、actuator↔qpos）。**教训**：永远先打印 MuJoCo joint/actuator 名再手工对齐。
4. **四元数约定 Scipy vs MuJoCo**：开机狂转。Scipy/ROS2 用 `(x,y,z,w)`，MuJoCo `qpos[3:7]` 是 `(w,x,y,z)`。**教训**：跨库连接必查四元数约定（两种常见次序）。
5. **WBC 力矩限幅过大**：用了 G1 的 200Nm（大机器人），WBC 放行过大力矩→仿真爆。**修复**：改 Go1 实际值 23.7/35.55Nm。**教训**：力矩限必须匹配真实电机。
6. **dummy_link + WBC DOF 错**：G1 URDF 有虚拟 `dummy_link`→Pinocchio 多 1 DOF；Go1 没有但 WBC 仍假设有→矩阵尺寸错、QP 失败。且 `FloatingBaseEoM` 只跟 3 DOF（xyz）而非 6 DOF（xyz+rpy）。**修复**：删 dummy_link + WBC 用 6-DoF contact。**教训**：必须理解 URDF 结构→它决定 Pinocchio 模型→决定 WBC 所有矩阵尺寸。
7. **URDF 腿序 ≠ ModelSettings（最隐蔽）**：前腿当后腿、接触力分错腿，**代码无报错只是行为错**。URDF 声明序 RF→LF→RH→LH，但 ModelSettings 定义 LF→LH→RF→RH，Pinocchio 按 URDF 声明序索引→偏移。**修复**：重排 URDF link/joint 声明序。**教训**：URDF 声明序决定 Pinocchio 索引，必须匹配 ModelSettings；此类 bug 极难查（无错误，只是行为不对）。
8. **SelfCollision 约束致 NaN**：SQP 发散、MPC 几步后 NaN。SelfCollision 用 CppAD 算梯度，两 link 距离趋 0 时梯度爆炸→cost Hessian NaN。**修复**：调试期 `activate false`，稳定后再开。**教训**：小机器人 link 更近、数值问题更易触发；调试先关 selfCollision。
9. **CentroidalModelType 选错**：用 Full(=0) 算完整质心动量矩阵→CppAD 慢、SQP 不稳。**修复**：切 SRBD(=1)。**教训**：从 SRBD 起步，需高精度再切 Full。

> **外部佐证**：此类四足→双足 port bug 是社区已知通病——`legged_control` GitHub issues #68（port 后前倾）、#72（四足→双足移植）、#45（nle 已含 gravity，勿重复加）可追溯。

### 3.2 ★ Braver 点足双足移植的元教训：~120 个 phase 都在追错方向（全套文档最高价值）

**病根（一句话）**：从不在 OCS2 上游（落脚/步时/力可行性），而在 WBC 执行层——`formulateBaseAccelTask` 被换成 go1 四足的**纯前馈版本，base 姿态/高度反馈增益恒为 0**。四足靠 4 脚静稳能活；点足是**倒立摆**，没有 base 反馈必然发散。

**为什么是致命的**：Go1 用**完全相同**的纯前馈 base task 能站——因为 4 脚张成的 2D 支撑多边形含住 CoM，WBC 再弱再振也被多边形接住；点足支撑多边形退化成**一条线**（两脚同 x≈0.0235m），是主动不稳倒立摆。把四足的"base 反馈增益=0"原样 port 过来，等于把点足唯一的姿态反馈通道掐断。更隐蔽：Braver 实为从 **hunter_bipedal_control(BridgeDP)** fork（WbcBase.h 顶部还留着 BridgeDP 版权、`basePoseDes_/baseVelocityDes_/baseAccelerationDes_` 成员已声明）——**本来就有 hunter 的正确拆分 base 任务，却被人改回了 go1 写法**。

**这个 bug 是怎么亲手埋下的（教训：别把"加增益更早饱和"误读为"反馈有害"）**：Phase 8 第一次在 `formulateBaseAccelTask` 里加了 base 反馈通道（根因解药雏形），却因"激进增益让 QP 更早饱和、低增益也失败"主动把 4 个增益**默认全置 0**——把最终根因写成了默认配置。没识破的三个原因：(a) 四足直觉认为 base 反馈"可选"；(b) 把饱和误读为反馈有害（真相是同时还在对冻结物理空跑 MPC + 站姿非平衡点）；(c) 没去对照 go1 原版确认 base 任务本应是带反馈的"拆分"形式。

**为什么 ~120 phase 没找到（四种归因谬误，移植时必须警惕）**：
- **层级归因谬误**：坚信瓶颈在 OCS2 上游（落脚/步时/力可行性），而非 WBC 执行层。117/119 反复把主线压在"接触力可行性/法向力地板/SQP 迭代"上。
- **四足直觉误导**：base 反馈"可选"对四足成立、对点足致命。
- **症状掩盖**：roll 失稳 + 40Nm 力矩饱和 + desired 接触力不可行被当**三个独立问题**逐个调，真相是同一根因的三种表象。更隐蔽的是 stepping 的"roll 摔倒"掩盖了静态站立的 pitch 振荡，phase131 隔离站立才暴露。
- **创可贴陷阱**：capture stepping 越精雕越远离根因，每轮都在不稳的地基上优化落脚目标，注定"单次偶尔接近 baseline、重复必不复现"。

**最强的单一定位杠杆（省 ~120 phase 的动作）**：每拍同时打印 **controller 实测量 与 MPC 对未来的预测量**。131c/131j 一旦做了"MPC 预测 vs 实测"对照就立刻证明 MPC 规划全程是对的（完美 48N/脚 站立解、预测 pitch→0，实测却越振越大、50ms 内 pitch 0.004→−0.25 比倒立摆自然发散快 17 倍 = 控制器**主动推反**）→ 问题 100% 在执行层，不必再碰上游一个参数。
> "快 17 倍"的可复算基线：LIP 自然频率 `ω₀=√(g/h)≈√(9.81/0.427)≈4.67~4.8 rad/s`，自然倒下需 ~0.9s；而实测 50ms 内 pitch 0.004→−0.25 → 比自然发散快约 17 倍，量化证明这不是"没接住自然下倒"而是控制器主动推反。

### 3.3 站立诊断的方法论与"反复路过根因"（203）

整条"让点足站住"线（torque standing→标量 PD 稳定器→reduced-state LQR 辨识→base 任务多轮扫参）全部失败，无一例外 unpause 后 0.5–1.7s 倒、几乎总是 roll 失稳 + 撞 40Nm 力矩饱和。多次摸到根因却擦肩：
- **Phase 16（信号最清晰、最可惜）**：实验数据白纸黑字——`baseAngularTask` kp 0→2 把 max pitch 从 **1.566 压到 0.189**，证明反馈通道生效且默认=0。距根因仅一步，却因"roll 仍倒 + 力矩饱和把信号掩盖"，把"单独打开角反馈仍 roll 失败"读成"base 反馈无用"，反强化"保持 0"。**真相**：缺的是**完整拆分任务（角+高度+XY 线性，且 roll 用世界系误差）**，只补 pitch 一路当然救不了 roll。
- **Phase 56/91/109-110（系统性误判固化）**：连续在不同上游条件下扫 base 任务，结论一致"base 任务 off 反而最佳"→把"base 任务该常开带反馈"彻底反向钉死。原因：高增益单路反馈与已饱和的 40Nm 力矩抢权，短期看更差。
- **Phase 9/10（方向对、落点错）**：明确写出"应做 model-based floating-base 反馈"、引 Riccati/WBC 论文——方向与解药一致，但动作落在"关节目标偏置/离线 LQR rollout"，且 LQR 围绕非平衡静站位姿线性化（affine drift 竖直项 −0.196 主导）注定失败 → 转去 capture/stepping，没人回 WBC 检查被置 0 的 base 反馈通道。

**LQR 辨识的可复用否定结果**：围绕双支撑站姿做 20ms 仿射辨识，仿射 drift 向量 `affine_c=[0,0.0005,0,−0.00004,−0.00206,−0.19568]`（**竖直项 −0.196 主导**），一步抵消 drift 需力矩 `[114,−232]Nm`，远超 ±0.15 clamp / 40Nm → 局部 reduced-state LQR 对点足静站**结构性不可行**。结论：点足不要指望静站，平衡=移动支撑点(stepping)。（另：加 `captureMaxRecoverablePitchRate=8.0 rad/s` 守卫、`initialPolicyMaxIterations=200`、`standPitchHipKp=1.5/standPitchKneeKp=−1.5` 等多为锦上添花的调参常数。）

**反直觉的排除项：sim 端 armature/阻尼其实在「帮忙」稳点足，别当失配清零**（archive/123 已排除方向表）：怀疑 MuJoCo 的关节 armature 是模型失配源、把它 `0.01→0` 想消除失配，结果**反而更快倒（0.67s）**。机理：armature/阻尼给点足这个主动不稳倒立摆提供了一点被动稳定，清零等于撤掉这点帮助。教训——移植时**不要盲目把 sim 端 armature/关节阻尼一律归为「模型失配」去置零**；它对不稳系统可能是净正贡献，是否失配要看闭环表现而非「sim 有、模型没有」就判它有害。（注：这是当时被排除的次嫌，非主根因；主根因仍是 WBC base 反馈被置零，见 §3.2/§3.7。）

### 3.4 接触力/摩擦可行性——一整条"症状追逐"线（204）

整条线（Phase21→130）反复验证同一症状——OCS2 desired stance force 频繁负 Fz、违反摩擦金字塔（千牛级残差），而 WBC 自身硬金字塔残差恒≈0.0；所有力侧调参（hard cone / soft barrier / pyramid penalty / normalWeight / minNormalForce / hard 法向力约束 / SQP 迭代 / input split / WBC tracking weight）都只买到秒级寿命、无一稳定。

**关键概念纠错（高价值）**：
- **金字塔 vs 圆锥 feasible-set 不一致**：OCS2 软约束是**圆锥** `μ(Fz+grip)−√(Fx²+Fy²+reg)≥0`；WBC QP 是**线性金字塔**（每脚 5 行）。两者非同集。Phase39 决定性纠错：旧 WBC "circular friction violation"(~95N) 是**用圆锥公式量金字塔约束的度量假象**，WBC 并未违反自己的约束（exact pyramid residual=0.0）。约定：OCS2 侧用圆锥度量、WBC 侧用金字塔残差。**补强机理**：OCS2 这个软圆锥默认 `regularization=25`（软锥在 `Fz≈8.3N` 才到边界）**且不含 `Fz≥0` 项 → 负法向力对软锥是「免费」的**，这解释了为何 OCS2 desired 频繁出现负 stance Fz（不只是身体倒，软锥本身对负 Fz 无惩罚）。
- **`sqp.g_max=1e-2` 收敛 ≠ 硬法向力不等式满足**：本地 OCS2 `totalConstraintViolation` 只算 `√(dynamicsSSE + equalitySSE)`，**不含 inequality SSE**。判断 hard 不等式是否被 active policy 满足的**唯一** solver 级证据是 `inequalityConstraintsSSE`。
- **三层 force-feasibility 诊断**：(1) current tick `optimizedInput` 的 Fz−floor；(2) `getPerformanceIndices().inequalityConstraintsSSE`（solver 级）；(3) active policy `inputTrajectory_` 配 `modeSchedule` 逐 knot 统计。current 高+horizon 低→MRT 采样/插值/时序问题；都高→active policy 本身不可行。
- **升级为通用三层可行性诊断（不止"力"，Ch07 方法论）**：① 当前 tick WBC QP `lastSolveSucceeded()`（此刻解出来了吗）；② MPC 解本身 `getPerformanceIndices()` 六字段（`dynamicsViolationSSE/equalityConstraintsSSE/inequalityConstraintsSSE/cost/merit/Lagrangian`，MPC 自己的解可行吗，看 `*_pre_failure` 版本定位恶化起点）；③ policy 预测窗口 `evaluatePolicy(t+0.3/+0.6)` 看 pitch/height 是否单调发散（"计划里就要摔"）。**最隐蔽的失败模式**：WBC 每 tick 都解成功（①过）+ MPC PerformanceIndex 不爆（②过，因 MPC 用质心模型**不知道** WBC 把姿态反馈调弱了）+ base 反馈不足导致缓慢倒——三层标量指标可能都不报警，**必须靠多维指标（roll/pitch 缓慢漂移）+ repeat 方差才能抓**。这正是本项目根因"增益被悄悄置零为何 ~120 phase 难定位"的诊断学解释。

**为何力侧调参全是治标**：力不可行是 base 失稳的**后果**（身体倾倒→OCS2 为追踪漂移的 base/momentum reference 而要求不可执行的 contact force）。加 minNormalForce / hard `Fz≥floor` / 更多 SQP 迭代只是在不可行 reference 上更用力求解——结果 desired force 爆更大，寿命无实质改善。**注册 hard 约束 ≠ 闭环满足**。把结论砸实的实测数字（Phase127–129）：
- hard `Fz≥50N` 时 `inequalityConstraintsSSE=66680.78`、`dynamicsViolationSSE 3.59→66.38`（hard 约束反引入更大动态不一致）、current floor violation **1633N**；policy horizon violation **1358.1N**（count 17562，遍布整条 horizon 而非仅当前采样点）。
- `SQP10`（更多迭代）：survival 2.839s 是假象，desired force 爆到 **~15338N**、force err **~30020N**。
- 唯一摸到病根的 Phase130（放松 base/momentum Q，见下）才把 desired residual 真正降下来——证明力不可行由 **base/momentum tracking 需求**驱动，不是摩擦不够。
- **「力矩饱和 ≠ 力矩不足」的量化锚点**（archive/121 外部审计已排除清单）：所有失败 run 都撞 40Nm 力矩饱和，易误判成「电机力矩权限不够」。但外部审计算出**单支撑横向支撑实际只需约 14Nm，而限幅 40Nm = 2.8× 余量**（总质量 9.805kg）→ 力矩物理上绰绰有余。所以 40Nm 饱和不是「力矩不够」，而是**被失控倒立摆逼到饱和的后果**——同 §3.2 那条「饱和误读为反馈有害」的陷阱。把「不是力的问题、根因纯在反馈缺失」这一判断砸实的就是这个 2.8× 数字。

**遥测载体（这些教训能复现的实操基础，逐阶段前向兼容追加字段）**：
- `/braver_debug/wbc_status`（Float32MultiArray）：`0..5` controller_time/planned_mode/solved/used_fallback/solve_status/n_wsr；`6..10` stance 金字塔违反/swing 力/接触脚数/μ/负 Fz 违反；`11..17` OCS2 侧同类度量 + max|OCS2 力|/max|WBC 力|/力误差/max|WBC 力矩|；`18..19` fallback 不可用/模式不匹配；`20..28` performance_index（merit/cost/dual/dynamicsSSE/equalitySSE/**inequalitySSE**/...）；`29..36` policy 法向力裕度（floor/min_margin/max_violation/violation_count/worst_time）。旧日志靠 `len≥29`（perf）、`≥37`（policy）向后兼容。
- `/braver_debug/command_breakdown`：`[t, planned_mode, kp, kd, wbc_torque6, pd_est6, raw_est6, pos_cmd6, vel_cmd6, jpos6, jvel6, wbc_contact_force6, optimized_input12]`——这是"Kp80 下低层 PD 单项 377Nm 被全 clip"这条教训的发现载体；**`/realTorque` 必须输出 clip 后值**否则误导。

**可复用工程范式（接 §3.7 的修复）**：
- **default-compatible 旋钮哨兵模式**：力侧 knob 用哨兵值保历史行为（`normalWeight=-1.0` 继承、`contactForceTangential/NormalScale=-1.0` 继承、`minNormalForce=0.0`、`projectToPyramid=false`、`enabled=false`），R_contact 缩放用 `*= sqrt(axis_w[r]*axis_w[c])` 保 off-diagonal 等价——新旋钮默认值必须复刻"加之前"的精确行为，才能干净 A/B。
- **OCS2 接入 hard 不等式清单**：继承 `StateInputConstraint` → 声明 `ConstraintOrder` → 实现 `isActive()/getNumConstraints()/getValue()/getLinearApproximation()` → 在 OCP `inequalityConstraintPtr` 按脚注册。**首次构建曾因漏 `getNumConstraints()` 编译失败**；且注册≠闭环满足，必须另查 violation/SSE/mode-active-window/WBC torque 可跟踪性。

**唯一摸到病根指纹的实验（Phase130）**：放松 base/momentum 的 Q（`very_loose_xy`）把 desired residual 从 ~1719N 降到 ~442N、policy Fz≥0 violation 2311→582、dynamics SSE 0.71→0.226，但 survival 没升、仍 roll。这证明"力不可行"是 **base/momentum tracking 需求**驱动的——若当时读成"是 base 维度而非力维度的问题"再去查 WBC base task 增益，就能直达修复。

**一句话签名**：roll + 40Nm 力矩饱和 + desired-force 不可行三件套同时恒定出现，是"身体站不住"的指纹，不是"摩擦不够"的指纹。

### 3.5 Capture 踏步创可贴的兴衰——最大治标支线（205）

为补偿倒着的身体，堆了一整套 capture 补偿层（触发器→落脚点→swing 平滑→侧向 capture→ownership 三态→AUTO 生成器→body-delta 限幅→capture-point 投影→bounded DCM 修正），**每个候选都 roll 失败、单次优势在 repeat 中一律破产**。phase133 修好 base 任务后整层 capture 被关掉，纯 MPC 直接走起来。

**为什么 repeat sweep 一再戳破"领先候选"（统计学教训）**：身体倒下的初始 roll/pitch 状态是随机主导项，任何落脚启发式的单次"长 run"都是 early-state 方差不是控制收益。Phase 54 `clamp_strong_010` 单次 mean 2.476s（含 3.408s 长 run）→ Phase55 只对它各跑 3 次双双跌回 ≈1s。Phase 61/68/73/78/85/87/120/123 全是同一剧情：单次领先→3-4 次 repeat 被 off baseline 反超。**方法论铁律：单次优势必须 ≥3 repeat A/B 才能成默认。**

**仍有价值的概念/遥测**（即便整层被删）：
- **OCS2 同步边界是落脚/mode 更新的正确位置**：target/mode 应经 `ReferenceManager`/`SolverSynchronizedModule`/`SwitchedModelReferenceManager` 在 solver 同步边界更新；post-MPC 在 WBC 层注入会与 MPC reference 打架。但同步边界只解决 ownership/timing，**不保证落脚动力学合理**。
- **ModeSequenceTemplate 平铺陷阱**（真实工程 bug）：单模板 `[RIGHT_STANCE]` 被 `tileModeSequenceTemplate` 平铺成连续右支撑→左脚整个 horizon 都 swing，导致 `auto_query_delta` 飙到 2.5–3.1s（而非一步 0.45s）。biped 一次性步需 `stance→DS→opposite stance→DS` 才能平铺成交替步态。通用 OCS2 legged 易踩坑。**代码级根因**：`GaitSchedule::tileModeSequenceTemplate` 在模板为空（`numTemplateSubsystems==0`）时直接 `return`，导致"最后一个模式永久持续"；平铺逻辑 `while eventTimes.back()<finalTime` 整体重复贴模板，且 `deltaTime=templateTimes[i+1]-templateTimes[i]` 故 `switchingTimes` 必比 `modeSequence` 多一个——新手定义步态最常见的崩溃源就是"长度对不上少一个时刻"。
- **target 拥有权三态分离**：target 是否存在于 planner / OCS2 cost 是否用它 / WBC 是否也收到——三者必须分开，否则 sweep 会无意中混比 ownership 模式得出误导结论（`swingFootTargetEnabled=false` 只关 WBC，receiver 仍写 OCS2，需 `swingFootOcs2TargetReceiverEnabled` 才真正关）。
- **body-frame 落脚分解遥测**（最有诊断价值）：`foot_body=R(−yaw)(foot−base)`、`raw_delta_body=target−foot_body` 区分失配是 forward 还是 lateral 主导。本项目发现是 **forward-body reset 主导**（生成器把"已远在身前的脚"reset 回 nominal base-relative foothold，被 cap 截成 0.10m）。
- **窗口去重正确做法**：dedup key 必须用 raw `swingTiming.start`，不能用 `max(swingStart, solverFinalTime)`（每 tick 变→每窗数百次重写，auto count 627，不可信）。
- **WBC fallback guard**（正确性修复，应保留）：QP 仅 `isSolved()` 后读 primal 并查返回码；fallback 仅当 size 匹配**且 `lastMode_==mode`**（防跨 mode 复用违反 swing/stance 约束）。
- **LIP / DCM / capture-point 完整推导（"为什么迈步能稳"的概念底座）**：LIP 下 `ẍ=ω²x`（**正号 → 不稳定**，离支撑点越远越被推远），`ω=√(g/h)`，Braver `h≈0.427m → ω≈4.8 rad/s`。把不稳定二阶动力学的发散模态单独拎出，定义 **DCM（瞬时捕获点）** `ξ=x+ẋ/ω`，求导代入得 `ξ̇=ωξ`（一阶线性发散，`ξ(t)=ξ(0)e^{ωt}`）——控制 DCM 就控制了全部不稳定性。**capture point** = 令 `ξ−p=0` 的落脚点 `p_capture=ξ=x+ẋ/ω`（身体冲得越快越要把脚迈得越靠前去接住；落在 capture point 一步停稳，想继续走则落在偏后一点让残余 DCM 驱动下一步）。**为何纯 PD 稳 DCM 而不迈步不行**：唯一能影响 ξ 的物理抓手是接触力，而它必须穿过那个**固定的点接触**（受摩擦锥限、无力臂调 CoP），ξ 一旦跑出脚附近 `e^{ωt}` 就超出反馈修正能力 → **必须移动支撑点（迈步）才能从根本上重置 ξ**。这正是点足"无静稳、横向平衡只能靠落脚"的数学根。（本项目 MPC 不显式算此公式，但隐式优化落脚 XY 在数学上就是在做 capture-point-like 的事。）
- **ALIP 横向落脚闭式律**：横向（frontal plane）平均速度为 0 却须左右交替以非零步宽 W 走，形成 period-2 极限环；交替步宽满足 `L_x^des = ±½ mHW·ℓsinh(ℓT)/(1+cosh(ℓT))`（ℓ=横向 LIP 频率，T=步时），少了主动横向落脚反馈 roll 模态必发散。
- **capture-point/DCM 投影几何**（本项目反应式 capture 层用法）：`dcm=base_xy+world_vel/ω`、`projected_dcm=stance_xy+exp(clamp(ω·t,0,1.5))·(dcm−stance_xy)`（指数须 clamp）；归一化线动量(state[0:3]/mass)可作 CoM 速度近似。
- **文献一致结论**：DCM/CP/ALIP/capture-step 落脚律都不把 capture point 当无约束目标，而结合 reachability、step timing、ZMP/contact feasibility、angular momentum。点足横向平衡**只能靠落脚**。关键文献：ALIP（绕接触点角动量 LIP）Gong & Grizzle arXiv:2105.08170 / 2008.10763；ALIP-MPC（落脚为决策变量）Gibson/Gong/Grizzle arXiv:2109.14862；MPFC Acosta & Posa arXiv:2309.07993；H-LIP Xiong & Ames arXiv:2101.09588；DCM 步位+步时调整 Khadiv arXiv:1609.09822 / 1610.02377；感知/MPC 落脚 2210.13371 / 2010.08198；点足 WBOSC arXiv:1501.02855；非线性质心 MPC humanoid arXiv:2203.04489；OCS2 Perceptive Locomotion（Grandia et al.）arXiv:2208.08373；underactuated point-foot biped 的 Riccati/LQR-WBC arXiv:2404.00591；Whole-Body MPC w/ MuJoCo finite-diff arXiv:2503.04613；Kuo 1999 / Bauby & Kuo（人类靠调步宽而非步长稳横向）。

### 3.6 OCS2-侧落脚/步时探索（206，"正路"但同样救不活）

swing-xy soft cost → mode-schedule timing → atomic ReferenceManager step → internal reachable planner → touchdown-time planner，架构全跑通且可观测，但**没有一个版本让身体停止倒下**（全 roll + 40Nm）。讽刺的是其中"放手让 MPC 自己拥有落点(OCS2-only swing xy)"方向**恰恰是对的**——phase133 最终突破正是恢复 base 任务后纯 MPC 自由规划 swing XY。

可复用要点：
- **SwingTrajectoryPlanner 只管 Z**（暴露 z position/velocity constraint），脚 XY 最初只活在 WBC 层，OCS2 horizon 看不到落脚点。把落脚进 OCP 用**软 cost 优于硬 xy 约束**（`SwingFootXYTrackingCost`，手写 Gauss-Newton StateInputCost，残差 `√(posW)(foot_xy−target_xy)`，仅 swing 脚 active，target 存为带 mutex 的两端零速度三次样条）。
- **mode-schedule timing 优于固定 offset**：`getModeSchedule()` 取目标腿下一有限 swing 区间 startTime/duration，把 solver-to-start delta 从 0.10s 降到 ~0.015s；结构化时序优于标量时序常数。
- **reachability clamp**：落脚目标生成应在 OCS2 同步 ReferenceManager 路径内、用 OCS2 自己 `initState`+Pinocchio FK，并**相对当前 swing 脚 clamp 使构造上可达**（post-planner clamp delta=0）。但纯启发式 ≠ DCM/MPC planner。
- **sync module 设计**：两个独立 synchronized module **不应分别提交同一语义步**（gait+foot target 应从同一 request 原子生成）；`SwingTrajectoryPlanner::update(modeSchedule)` 必须在最终 ModeSchedule 已知后。
- **"WBC-disabled 的 err=0.0"是陷阱**：那是"无 WBC 参考激活"不是"完美跟踪"，正确指标看 OCS2 遥测（disabled 也有 active OCS2 target 样本）。
- **铁律重申**：force/desired-residual 指标改善 **≠** 闭环 survival 改善。

### 3.7 突破：修复 + 确定性周期 gait（124/125/128/129/130）

- **修复（phase133）**：恢复 hunter 拆分 base 任务 = `formulateBaseAccelTask = BaseXYLinearAccel(前馈) + BaseHeightMotion(Jacobian+PD) + BaseAngularMotion(Jacobian + 旋转矩阵误差 + 世界系ω)`。角任务正确形式：`a=base_j_(3:6)`，`b=accDes + Kp·rotationErrorInWorld(R_des,R_meas) + Kd·(ω_des−ω_meas) − base_dj_(3:6)·v`；期望量由 `rbdConversions_.computeBaseKinematicsFromCentroidalModel` **解析**得到（不做有限差分）。task.info `baseHeight/baseAngular kp 0→20, kd 0→3, jointKd 0→3`。
  - **旧前馈写法错在哪（两块根因拼图）**：(1) **frame bug**：旧角反馈把 ZYX-euler 角误差 + euler 速率直接加到 **body 系角加速度** `b.tail<3>()` 上——三个不同切空间相加，近 gimbal 奇异、跨轴耦合（agent 分歧结论："近直立小角度可能够用、大角度必错"）；正解必须用 `rotationErrorInWorld` + 世界系 ω + base 角 Jacobian。这是"Phase16 开角反馈反而让站立更糟"的精确机理。(2) **有限差分噪声放大**：纯前馈项里 `jointAccel=(u−u_last)/period`，`period≈0.0021s`（实时抖动周期）做分母把逐拍抖动放大约 **476×**——但 SRBD 下 `Aⱼ≈0` 该项近 0 才**幸免**，**切到 FullCentroidal 才致命**（解释了 `centroidalModelType` 选择与 base 任务噪声的耦合，故文档强调期望量须解析、非有限差分）。
  - **次级教训**：修复后单纯站立仍 ~1.3s 倒，但 onset trace 显示反馈**已起效**（pitch 初始后倒被拉回过零→前向发散）——是**欠阻尼过冲**非符号错（佐证：加大 base 权重反而最快倒，与欠阻尼一致）。问题降级为阻尼调参。
- **点足无静稳平衡（正确建模而非 hack）**：两点接触在 ±0.1475m 横向但**前后同一 x 线**→pitch 方向无支撑力臂→必前扑。点足双足**没有静止站立稳态**；"站立"必须是"踏步"。
- **运行时注入 gait 有 ~1 horizon 延迟**：`GaitReceiver` 在 `finalTime`（horizon 末）才 insert。要"从一开始就走"必须改 `reference.info` 的 `defaultModeSequenceTemplate`（从 DOUBLE_SUPPORT 改成 standing_trot 踏步序列，首个 MPC policy 即规划踏步），不能靠运行时注入。
- **`/cmd_vel`→MPC TargetTrajectories 接口公式**（`cmdVelToTargetTrajectories()`，速度跟踪复现细节）：取当前 base pose `state[6:12]`，`cmdVelWorld=R_zyx·cmd[0:3]`，`target_pos=cur_pos+cmdVelWorld·timeToTarget`，`target_yaw=cur_yaw+wz·timeToTarget`，`z=comHeight`、`pitch=roll=0`，`normalizedMomentum[0:6]=cmdVelWorld`；`timeToTarget` 按位移/转角幅度估。
- **突破结果**：WBC 修复 + 默认踏步 gait + `swingFootTargetEnabled=false`（纯 MPC 落脚，无外部 XY 目标，capture OFF）→ 18s 不摔、速度跟踪误差 1e-4、走直线（横漂 <2cm/3.85m）；0.1–0.5 m/s standing_trot 稳定，pitch 均值 ≤1.5°。高速转 trot（无双支撑）达 ~0.9 m/s 且平衡更好（pitch 0.8°）。**capture stepping 确实只是给坏掉的 base 任务打的补丁**——根因修复后简单路线即通，不要被 130 个 phase 的复杂度误导。
- **分速度区间用不同步态**：低速/起步 standing_trot（稳）→ 已在踏步状态注入 trot（高速，1 horizon 延迟期间不会摔，平滑过渡）。

### 3.8 实验卫生 / 复现性教训（贯穿性、ROS2+Docker 批量实验通用）

- **干预必须真正作用到对象**：phase131a–f 设 `startWithCommandsEnabled:=false` → 控制器根本不发命令（`hwSwitch_=false`），sim 退回自身默认 PD(Kp=20) 把关节拽向零位、机器人**与控制器无关地**缓慢塌陷——造成诡异的 ~1.58s 不变性，浪费多轮并污染结论。**站立/行走测试必须 `startWithCommandsEnabled:=true`**。
- **ROS2 + Docker 批量实验三铁律**（否则容器泄漏 + ROS graph 串扰**静默污染数据**并拖垮机器）：首批 s02/s03/s04 跑完发现 5 个容器残留未退、内存吃到只剩 1GB。两根因叠加：① `ros2 launch` 子节点不在 launch 进程组，`kill -INT -$PID` 组杀只杀父进程、sim/controller 子节点活着→`--rm` 不触发；② `--network host` 共享 ROS graph，多 sim 同发 `/ground_truth/state`、多 controller 抢 `/cmd_vel`→结果被污染。**修复**：(a) 每容器独立网络命名空间 + 唯一 `ROS_DOMAIN_ID` 隔离；(b) teardown 用 `pkill -9 -f` 按名杀进程树而非进程组；(c) 外层 `timeout --signal=KILL` + `docker rm -f` 硬超时兜底；(d) 一次只跑一个容器。
- **日志爆炸**：摔倒后 `SafetyChecker` 每 tick 刷错（一次 23s 跑出 46MB），sweep 前必须在日志写入端过滤。
- **`/realTorque` 必须输出 clip 后值**：否则误导——Kp80 下低层 PD 单项就要 377Nm 被全 clip，命令分解遥测才能发现。
- **采样率混叠陷阱**：60fps 采 50Hz 指令、数值微分得 0.366 m/s²（>约束 0.30），但 50Hz 指令序列自身微分=精确 0.300。**核实约束要用信号自身采样率的序列**，差速率采样求导会高估斜率。
- **headless docker 离线出 demo 视频的可复用避坑清单**：EGL 渲染成功但 `__del__` 抛 `EGLError`（清理时 benign 可忽略）；**osmesa** 软件渲染干净无此噪声、速度相当（~75ms/帧@640×480），demo 用 osmesa。MuJoCo 默认离屏 framebuffer **640×480**，超出须在模型 XML 加 `<visual><global offwidth=.../>`。容器常无 ffmpeg/imageio 但有 cv2，用 `cv2.VideoWriter(fourcc='mp4v')` 写 mp4。用**自由跟随相机**（lookat 锁 base、方位固定）以免随机身偏航转动致画面眩晕。
- **失败时间抖几百 ms、非 bitwise 确定**：别信单点 failure_time，看分布 + roll/height/force 复合排名；repeat≥2–3 取 mean+min/max。

### 3.9 移植腿式机器人的可复用知识（202 + 201 蒸馏，**最实用的清单**）

1. **先审 fork 来源、再信代码**：移植项目第一动作应是"确认 fork 自谁"并 **diff 关键 task（base/contact/swing）vs 上游原版**——port 过程最易发生"用错误机型的实现覆盖正确实现"（本案正是 go1 纯前馈 base task 覆盖了 hunter 拆分 base task）。
2. **质疑一切被默认禁用/置零的反馈与约束**：`kp=kd=0`、`enabled=false`、`weight≈0` 不是中性默认而是高优先级嫌疑。逐一问"为什么是 0？这个对象（点足倒立摆）能承受它为 0 吗？"——131 甚至发现开角反馈让站立更糟（潜在符号 bug），说明那些 0 不仅是缺失还藏着错误。
3. **用"规划 vs 执行"对照快速二分**：每拍同时打印 controller 实测量 与 MPC 对未来预测量。MPC 预测收敛而实测发散→立刻把战场从 OCS2/OCP 转到 WBC/执行层，不必再碰上游一个参数。**本案省 ~120 phase 的单一最高杠杆。**
4. **善用同框架的 working 对照体**：存在用同一代码能站的机型（go1）时，"它能、我不能"的逐行 diff 比任何盲扫都快。
5. **把对象的结构先验当硬约束**：点足=零面积支撑多边形=踝力矩 0→**无静稳平衡、横向平衡只能靠落脚位置**。带着这个先验，"静站站不住"不是 bug 而是预期；默认 gait 该是连续 stepping。
6. **机器人无关化的那一行**：`torqueLimits_ = vector_t(actuatedDofNum / numThreeDofContacts)`（每腿关节数）。凡 go1 里 `/4`、`12`、`*4` 的硬编码都要审查替换；维度从 `CentroidalModelInfo` 读取而非硬编码。
7. **contact frame 对齐铁律**：OCS2/Pinocchio 的 contact name 来自 URDF（保留 fixed foot link 做命名 frame），MuJoCo foot site 来自 MJCF，两者手动对齐（统一 `L_foot`/`R_foot`）。
   - **更深的一致性陷阱：惯量 product 项失配（contact frame/joint order 之外的第三维）**（archive/123 剩余主嫌之一，已回源核实）：Pinocchio 读 URDF 的**全惯量张量**（含交叉项 ixy/ixz/iyz），但 MuJoCo MJCF 普遍用 **`diaginertia`（只给主轴对角惯量，丢掉 product 项）**——两个动力学模型的同一连杆惯量就**不是同一个矩阵**。本项目实测：Braver base 在 URDF 里 `ixy=0.00028, ixz=0.00564, iyz=0.00101`（其中 ixz 最大，是矢状面耦合，直接进 pitch 动力学），而 MJCF 对应 body 写成 `diaginertia="0.04388 0.05802 0.04926"` 把这三项全丢了。**distal 关节惯量本身小、product 项相对不可忽略时，这一失配会污染 roll/pitch 动力学**——MPC（用 URDF/Pinocchio 模型规划）以为的姿态动力学与 sim（用 MJCF）实际的不一致，正是点足这种姿态边缘系统易被放大的差异。教训：URDF↔MJCF 对齐不止 contact frame + joint order，还要核对**惯量是否被 MJCF 降成对角**；要么 MJCF 用 `fullinertia` 保留 product 项，要么确认这些交叉项小到可忽略再用 `diaginertia`。
8. **MJCF 最小要件**：`freejoint` + 按 controller order 的 hinge + 命名 foot geom/site + motor actuator(ctrlrange=力矩限) + IMU site & 4 sensors(framequat/framepos/gyro/accelerometer) + standing keyframe；`nq=7+njoint, ctrl=njoint`。直接 URDF 导入 MuJoCo 不可用（无 floating base、无 actuator），只能当 joint order/limits 来源。
9. **CMake 修复模式**：中间 `src` 库也要 `target_link_libraries(src ${DEPS})`，否则其 `.cc` 里 include 找不到头（`mujoco/mujoco.h: No such file`，configure 成功但编译失败）。
10. **CppAD 提示**：唯一 `modelFolderCppAd=/tmp/<robot>`，必要时 `recompileLibrariesCppAd=true`，solver/build 线程压 1~2。
11. **fork-as-new-family 决策准则**：模板里有 4-foot 数组 / 4-bit mode / N-joint 消息 / per-leg KF 假设时，复刻为新包而非加 `if(robot==...)`；cheater 估计先行、KF 后补（go1 KF 死绑 4 足/12 关节/固定矩阵，不移植）。
12. **★ fork WBC 时必审 base-acceleration task 的反馈增益**：不能假设"能 build + 单次 solve 通过 + torque 近零"就正确——四足静稳会掩盖 base task 反馈增益=0 的缺陷，点足倒立摆不会。WBC 单测要包含**闭环/单支撑**而非只验"构造+一次双支撑求解"。

---

## 4. 可吸收进《机器人学笔记》的点

> 标注建议进哪一部/章（按笔记现有"规划/控制/估计"部组织习惯）。

### 4.1 优先级最高——失败方法论与诊断（独立成节，价值最大）
- **"层级归因谬误"与"规划 vs 执行"快速二分法** → 适合放进**控制部的"调试与系统诊断"章**或一个独立的"机器人控制工程方法论"附录。内容：§3.2 元教训 + §3.9 清单 1-5。这是跨项目可迁移的硬核经验，远超单个算法。
- **复现性与实验卫生**（§3.8：repeat≥3 铁律、ROS2+Docker 隔离三铁律、采样率混叠、日志爆炸）→ **附录"机器人实验工程"**。
- **"指标改善 ≠ 闭环改善"+ 三层 force-feasibility 诊断 + `g_max` 不含 inequality SSE**（§3.4）→ **控制部"MPC 可行性诊断"小节**。

### 4.2 控制部 · MPC/最优控制章
- OCS2 质心动力学 MPC 完整推导（§2.1）：OCP→multiple shooting→SQP→QP 块带状 KKT→HPIPM；质心动量"外力唯一驱动 + 关节经几何/映射间接影响"的物理解释；SRBD vs Full 的动量映射近似差异；R 矩阵 Jacobian 变换 `R_joint=JᵀR_task J`；u_nominal=重力补偿。**这是一份可直接入书的非线性 MPC 实战推导。**
- Switched-system gait schedule（mode 编码决定每节点激活约束）+ 4 类接触约束。

### 4.3 控制部 · WBC 章
- 7-task WBC（§2.2）+ **HoQP 零空间投影算法**（`Z_new=Z_prev·kernel(A·Z_prev)`，分层严格优先级）+ Weighted vs Hierarchical 取舍。
- ★ **拆分 base 任务 vs 纯前馈 base 任务**（§3.7）：角任务 `rotationErrorInWorld + 世界系ω + base Jacobian`，期望量解析（非有限差分）。配合"四足靠支撑多边形容忍、点足倒立摆不容忍"的物理直觉——**这是 WBC 章一个绝佳的"为什么要姿态反馈"案例**。
- 金字塔 vs 圆锥摩擦约束的 feasible-set 不一致（§3.4）。

### 4.4 估计部
- 广义动量观测器估接触力（§2.3 完整公式）+ 早/晚触地检测 + EKF（IMU 预测 + FK 更新，stance 脚固定为观测）→ **估计部"接触估计/腿式状态估计"章**。

### 4.5 规划部 / 双足专题
- **点足/双足落脚规划**（§3.5/3.6）：点足横向平衡只能靠落脚（ALIP/H-LIP/DCM/capture-step 文献谱）；OCS2 同步边界是落脚更新的正确位置；ModeSequenceTemplate 平铺成交替步态的正确写法；reachability clamp；swing 软 cost 优于硬约束。→ **规划部"足式落脚与步态"章**或双足专题。
- DCM/capture-point 投影几何（`projected_dcm=stance+exp(ωt)(dcm−stance)`）。

### 4.6 感知部
- **Perceptive MPC**（§2.4）：plane segmentation→凸区域→多边形半空间→落脚点约束 + SDF 碰撞约束 + 地形自适应 base 参考，全部用 RelaxedBarrier 软约束嵌入 OCP。→ **感知部"地形感知运动/perceptive locomotion"章**，与盲 MPC 对照。

### 4.7 实践/移植附录
- "换机器人 9-bug 清单"（§3.1）+ "移植腿式机器人 12 条可复用知识"（§3.9）→ **实践附录"把控制器移植到新机器人"**。URDF 声明序决定 Pinocchio 索引、四元数约定、维度公式、CMake 链接坑等。

---

## 附：关键文件定位（如需回溯源码）

- 四足 curated 技术文档：`docs/quadruped/01..08_*.md`（架构/bugs/MPC/WBC/估计/MuJoCo/config/build）+ `03.5_perceptive_locomotion.md`。
- Braver 失败叙事蒸馏：`docs/research/braver_locomotion/200..206_*.md`（200 索引/201 根因复盘必读/202 bringup/203 站立/204 力可行性/205 capture/206 落脚）；里程碑 `124..130_*.md`；`codebase_map.md`（file:line 速查）。
- OCS2 腿式控制实战教程：`docs/research/braver_locomotion/tutorial/Ch00..08`（以本项目为贯穿案例）。
- 源码：`src/{go1,anymal_c,braver}/*`（interface/controllers/wbc/estimation/mujoco_sim/description），`src/legged_perceptive_*`，`src/ocs2_ros2`，`src/qpoases_catkin`；WBC 核心 `src/braver/braver_wbc/src/{WbcBase,HoQp,HierarchicalWbc,WeightedWbc}.cpp`。
- 实验/可视化脚本：`docs/research/braver_locomotion/scripts/`（gait_velocity_injector、phase15_metrics、phase17_sweep、**两个 LQR 脚本** `braver_lqr_standing_id.py` 与 `braver_lqr_torque_standing_id.py`、`braver_model_check.py`、random_cmd_node、demo_logger、render_video）。回溯实验原始数据：`archive/experiment_runs/`（141 个 phase sweep 目录 / ~275M 原始数据）。
