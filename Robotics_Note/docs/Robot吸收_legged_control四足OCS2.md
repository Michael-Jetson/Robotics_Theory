# Robot 工程吸收 · legged_control 四足 OCS2(NMPC+WBC+KF+sim2real)

> 来源项目: `/home/ziren2/pengfei/Robot/legged_control`(qiayuanl/legged_control, ROS1/Noetic, 基于 OCS2 + ros_control 的四足 NMPC-WBC-状态估计-sim2real 框架, 引 liao2023walking)。
> 本文是对该项目 `docs/dmgo_port/`(自定义四足 dmgo 移植进 legged_control 的全程文档)+ 核心源码的**凝练吸收**: 架构/方法/算法 + **工程实战踩坑与失败根因**(本项目最值钱部分: 三大根因侦探故事)。
> 关键数字/机理已回源码亲验(file:line 标注)。**只读源, 未改动原项目任何文件。**

---

## 第一部分 · 项目概览(目标 / 技术栈 / 架构)

### 1.1 定位与血缘
- **legged_control** = 基于 [OCS2](https://github.com/leggedrobotics/ocs2)(非线性 MPC 框架) + ros_control 的腿式 **NMPC + WBC + 状态估计 + sim2real** 全栈。作者 Qiayuan Liao(UC Berkeley)。**已停止维护**(作者转新框架)。
- 本仓 = OCS2 `legged_robot` 例子的**平地、非感知子集**: Grandia 论文里的 elevation-map / SDF 碰撞 / 落脚多边形约束**不在**本仓(那些在 `legged_perceptive`)。本仓约束只有: **摩擦锥 + 支撑足零速 + 摆动足 z 曲线 + 自碰撞软约束**。
- 衍生谱系: → `hunter_bipedal_control`(双足分支, 给 WBC 加 base 反馈)→ braver(点足双足); → `legged_control_ros2`(ROS2 重打包 + MuJoCo); → `legged_perceptive`(感知/不平地)。
- 本项目实践(dmgo 移植)定位: **标准四足是感知四足的平地地基/前传**。

### 1.2 在线数据流(三层频率, 仿真同环 ~100Hz)
```
用户指令 (/cmd_vel Twist 或 /move_base_simple/goal PoseStamped)
   │  TargetTrajectoriesPublisher: 速度/位姿目标 → OCS2 TargetTrajectories
   ▼
SwitchedModelReferenceManager (GaitSchedule + SwingTrajectoryPlanner)
   │  注入 ModeSchedule(接触序列) + 摆动足 z 曲线
   ▼
NMPC = SqpMpc (异步线程 ~100Hz)  ← legged_interface 装配的 OptimalControlProblem
   │  多重打靶 → RTI-SQP(1 iter) → HPIPM QP → optimizedState/Input 轨迹
   ▼  (MPC_MRT_Interface: updatePolicy 非阻塞读 / evaluatePolicy 时间插值)
WBC = WeightedWbc (每 ros_control tick ~100Hz, qpOASES)
   │  MPC 期望 state/input + 实测 RBD 态 → 关节力矩 τ (+ 接触力 F + 广义加速度 q̈)
   ▼
HybridJoint 命令 (posDes, velDes, kp=0, kd=3, τ_ff) ×12 关节
   ▼
LeggedHWSim (Gazebo 插件) → 物理仿真
   ▲                         │ readSim: 关节编码器/IMU/接触
   └── 线性 Kalman 估计 ◄─────┘  (base 位姿/速度 ← IMU + 足端运动学)
```
注: 论文里 MPC 100Hz / WBC 400Hz 是真机分频; 本仓**仿真同环跑**(MPC 异步线程, WBC 在 ros_control update)。真机算力上界: README 记作者在 **11 代 NUC 上 NMPC 频率可接近 200Hz**(质心 MPC 的实时可达频率上界数据点)。

### 1.3 核心维度速查(四足 12-DOF, 已亲验源码)
| 量 | 值 | 构成 |
|---|---|---|
| stateDim | 24 | `h_com(6)归一化质心动量` + `q_b(6)base位姿:3pos+3ZYX欧拉` + `q_j(12)关节角` |
| inputDim | 24 | `f_c(12)4足×3接触力` + `v_j(12)关节速度` |
| generalizedCoordinatesNum | 18 | base6 + 关节12 |
| **WBC 决策变量** | **42** | `q̈(18) + F(12) + τ(12)` |
| RBD 估计状态 | 36 | (姿态3+位置3+关节12)×2 |
| 接触/模式顺序 | `{LF,RF,LH,RH}` | **硬编码**(`MotionPhaseDefinition.h`), 全栈须同序 |

> ⚠ **WBC=42 的亲验**(`WbcBase.cpp:24`): `numDecisionVars_ = generalizedCoordinatesNum(18) + 3·numThreeDofContacts(12) + actuatedDofNum(12) = 42`。**两个子 agent 摘要曾误写 30/base6**——q̈ 是全 18 维广义加速度而非只 base 6 维。教训: 核心数字必回源码验。

### 1.4 "加新机器人"要提供什么(移植清单)
1. **URDF/xacro**: 连杆/关节名**必须同 legged_unitree**(`LF/LH/RF/RH × HAA/HFE/KFE` + `*_FOOT` + `base` + `base_imu`); **关节零位+轴方向对齐参考**(#1 杀手)。
2. `config/<robot>/{task,reference,gait}.info`: 从 go1 复制, 改 `modelFolderCppAd`, 按需调 initialState/defaultJointState/comHeight/Q/R。
3. Gazebo 接入: `gazebo.xacro`(LeggedHWSim 插件) + `imu.xacro` + `transmission.xacro`(HybridJoint) + p3d 地面真值。
4. launch: 仿 `empty_world.launch` + `load_controller.launch`, 认 `robot_type`。
- **机器人专属 = 仅 3 个 config.info + URDF**(URDF 提供连杆树/惯量/关节限位/碰撞几何/**关节零位与轴向**); 通用算法代码(LeggedInterface/各 constraint·cost/WBC/估计/控制器)**全不动**。
- **⚠ 薄包可复用 unitree 的 `gazebo/imu/transmission/materials` 宏(机器人无关), 但 `leg.xacro`(运动学)绝不能复用**: unitree leg.xacro 把"大腿垂直零位"硬编码进关节 origin(`KFE origin xyz="0 0 -thigh_length"`), 若你的机器人零位约定不同(如 dmgo 是"水平向后零位"), 强行复用会让 STL 网格方向全错。判定准则: **同作者参考件里, 与零位/轴向约定耦合的几何宏不可复用, 必须用自带运动学**——这条陷阱藏在关节 origin 里, 移植清单只说"连杆/关节名须同 legged_unitree"时极易漏。

---

## 第二部分 · 核心方法与算法(分模块)

### 2.1 NMPC 层(`legged_interface`)
核心类 `LeggedInterface`(`LeggedInterface.cpp`)。`setupOptimalControlProblem()` 装配:
1. **setupModel**(cpp:142): `centroidal_model::createPinocchioInterface(urdf, jointNames)` 建 Pinocchio + `createCentroidalModelInfo()`(质心动力学, 类型 0=FullCentroidalDynamics)。
2. setupReferenceManager(cpp:158): `SwingTrajectoryPlanner` + `SwitchedModelReferenceManager`。
3. 动力学 `LeggedRobotDynamicsAD`(质心动量 + Pinocchio 雅可比, CppAD 自动微分; base 位姿由动量积分, 关节角由 v_j 积分; **机器人专属信息全来自 URDF**)。
4. 代价 + 逐足约束 + 自碰撞软约束 + warm-start initializer。

**代价**: `½‖x−x_ref‖²_Q + ½‖u−u_ref‖²_R`。Q 对角(动量 10–100、**base z 高度 1500**、关节角 2.5–5); R 对角(**接触力 ~1、关节速度 5000×1e-3**)。R 代价参考 = `weightCompensatingInput`(=mg)(`LeggedRobotQuadraticTrackingCost.h:61`), **平衡点零拉力, 模型无 below-mg 偏置**(重要: 见踩坑 3.1 红鲱鱼 A)。

**约束(平地)**:
| 约束 | 类型 | 激活 | 数学 |
|---|---|---|---|
| 摩擦锥 | 不等式(软,可硬) | 支撑足 | `μ·F_z − √(F_x²+F_y²+ε²) ≥ 0` **平滑锥**(μ≈0.3–0.5; mu/delta 是松弛 log-barrier 参数) |
| 摆动足零力 | 等式 | 摆动足 | `u[3i:3i+3]=0` |
| 支撑足零速 | 等式 | 支撑足 | 足端速度=0 |
| 法向速度 | 等式 | 摆动足 | 足端法向跟 z 曲线 |
| 自碰撞 | 软(barrier) | always | 连杆对距离 ≥ minimumDistance(0.05) |
> 细节: MPC 侧是**平滑摩擦锥**; WBC 侧是**摩擦金字塔**(5 行/足, 线性化)——两层可行集表示不同(已知细节)。
> **摆动足 z 曲线参数**(task.info `swing_trajectory_config`, `SwingTrajectoryPlanner` 据此生成): `swingHeight=0.08`、`liftOffVelocity=0.05`、`touchDownVelocity=−0.1`、`swingTimeScale=0.15`(go1/dmgo 完全相同)——这是 MPC 侧约束的落脚 z 轨迹来源, 与 WBC 侧跟踪增益(swingKp=350/kd=37)分属两层。

**求解器**: `SqpMpc` = 多重打靶 → **RTI-SQP(每周期 1 次 Newton 步)** → QP 子问题 **HPIPM**(Riccati 一遍解) → 滤波线搜索 → warm-start。`mpc.timeHorizon=1.0s`, `mpcDesiredFrequency=100`, `sqp.dt=0.015`, **`sqpIteration=1`**(实时关键, 见踩坑 3.3)。task.info 同含 ddp(SLQ)/ipm 设置可切。

### 2.2 步态调度与目标轨迹
- `SwitchedModelReferenceManager` = `GaitSchedule` + `SwingTrajectoryPlanner`; `getContactFlags(t)→modeNumber2StanceLeg()` 解 4 位接触标志。
- **模式编号 = 4 足接触位** `[LF,RF,LH,RH]=8/4/2/1`(`MotionPhaseDefinition.h`): `FLY=0`、`RF_LH=6`、`LF_RH=9`、`STANCE=15`。→ trot `modeSequence:[LF_RH,RF_LH]=[9,6]`(gait.info 用符号名 `LF_RH/RF_LH`, 等价整数 [9,6])。**周期: go1=0.6s `[0,0.3,0.6]`; 而 dmgo 实测 gait.info 为 `[0.0,0.35,0.70]`=0.7s**(P3-087 刻意放慢=aliengo 重机时序: 每步更长支撑相, 在俯仰步顶用以遏制 roll, 与关节阻尼修复 P3-086 配套, 详见 §3.10)——**移植不能照抄 go1 周期, 重机器人需更慢 trot**。gait.info 命名步态全名单: stance/trot/standing_trot/flying_trot/pace/standing_pace/dynamic_walk/static_walk/amble/lindyhop/skipping/pawup(共 12 个)。
- **触发步态切换**: `GaitReceiver`(实例化在控制器内 `LeggedController.cpp:236`)订阅 latched topic `legged_robot_mpc_mode_schedule`(`ocs2_msgs/mode_schedule`)→ `insertModeSequenceTemplate` 铺到 MPC horizon。
- **`/cmd_vel` 路径**: 独立节点 `legged_target_trajectories_publisher`(`cmdVelToTargetTrajectories`)按当前 yaw 旋转 + 积分 timeHorizon 得目标位姿(z 锁 comHeight)→ 发 `legged_robot_mpc_target`。

### 2.3 WBC 层(`legged_wbc`)
决策变量 `x_wbc = [q̈(18), F(12), τ(12)]` = 42。各 task(`a·x=b` 等式 / `d·x≤f` 不等式):
| task | cpp | 类型 | 数学 |
|---|---|---|---|
| 浮动基 EoM | 97 | 等式(硬) | `[M │ −Jᶜᵀ │ −Sᵀ]·x = −nle`, 18×42(S=选择阵, base 6 行零) |
| 力矩限 | 110 | 不等式(硬) | `−τ_lim ≤ τ ≤ τ_lim` |
| 摩擦金字塔 | 142 | 不等式(硬) | 每支撑足 5 行, μ=0.3; 摆动足力=0 |
| 支撑足不动 | 125 | 等式(硬) | `Jᶜ·q̈ = −dJᶜ·v` |
| **base 加速度** | 174 | 等式(软) | **四足纯前馈, 见下** |
| 摆动腿 | 202 | 等式(软) | `Jᶜ·q̈ = kp(p_des−p)+kd(v_des−v)−dJᶜ·v`, swingKp=350/kd=37 |
| 接触力 | 227 | 等式(软) | `F = F_des(MPC)` |

**★ base 加速度 task = 四足纯前馈**(`formulateBaseAccelTask` cpp:174): `b = A_b⁻¹·(质心动量率_MPC − Ȧ·v − A_j·q̈_j)`, **无任何 base 位姿/高度/姿态反馈增益**, desired 动量率纯来自 MPC。
- **对四足正确**(4 足静稳 + MPC 隐式稳基, 纯前馈够用)。
- **双足必须加反馈**: hunter_bipedal 把它拆成 BaseHeightMotion(kp/kd) + BaseAngularMotion(kp/kd) + BaseXYLinearAccel(仍前馈)。**braver 当年 bug = 误用 go1 纯前馈版(增益=0), 点足倒立摆必发散, 耗 ~120 phase 才揪出**——dmgo(四足)直接用原版**不会重蹈**。
- 注意: EoM 的 `nle`(Pinocchio `data.nle`)已含重力+Coriolis+离心, **EoM task 别重复加重力**(issue#45)。

`WeightedWbc`(默认): 硬约束(EoM+力矩限+摩擦+支撑不动) + 软任务(摆动腿×100 + base加速度×1 + 接触力×0.01)堆成单 QP→qpOASES。可选 `HierarchicalWbc`+`HoQp`(严格优先级零空间投影, 理论 Bellicoso16)。

### 2.4 状态估计(`legged_estimation`)
- `StateEstimateBase`: 输出 `rbdState_`(36 维 = 姿态3+位置3+关节12 各 ×2)。
- **`LinearKalmanFilter`(生产)**: 状态=[base位置3, base速度3, 4足位置12]=18; 观测=足端位置/速度(各12)+足高(4); **IMU 加速度做预测, 足端正运动学做更新**; **按接触相调噪声**(支撑足信任运动学; 摆动足噪声×100)。可选融合 `/tracking_camera/odom`。出处 Flayols17。
- `FromTopicStateEstimate`(仿真 cheater): 直接取 `/ground_truth/state`(Gazebo p3d)透传不滤波。对应 `legged_cheater_controller`——**bring-up 调试首选**(排除估计误差)。
- **噪声/误差配置在哪**(切 KF 必看): ① IMU 传感器噪声 = `imu.xacro`+`gazebo.xacro` 的 `gaussianNoise`(go1=0); ② KF 协方差 = task.info `kalmanFilter` block(`imuProcessNoise{Pos,Vel}`=0.02、`footProcessNoisePosition`=0.002、`footSensorNoise{Pos,Vel}`=0.005/0.1、`footHeightSensorNoise`=0.01); ③ p3d gaussianNoise(仅 cheater)。
- **cheater→KF 切换 = bring-up 真门槛**: cheater 用零噪地真; KF 引入真实漂移 → cheater 下指标是乐观下界。

### 2.5 ros_control 运行环(`legged_controllers`)
`LeggedController : MultiInterfaceController<HybridJointInterface, ImuSensorInterface, ContactSensorInterface>`。
- **update()** 每 tick(cpp:101): ① `updateStateEstimation` 读关节/接触/IMU→估计→`measuredRbdState_`(yaw 解卷绕); ② `setCurrentObservation` 非阻塞写; ③ `updatePolicy` 非阻塞读最新策略; ④ `evaluatePolicy(t,x)` 时间插值; ⑤ `wbc_->update`→`x=[q̈,F,τ]`, `torque=x.tail(12)`; ⑥ `safetyChecker_`; ⑦ 逐关节 `setCommand(posDes, velDes, kp=0, kd=3, τ_ff)`。
- **HybridJoint 命令约定 kp=0, kd=3**(亲验 `LeggedController.cpp:176`): 位置无反馈、低速度阻尼、跟踪靠 WBC 前馈力矩。
- **starting()**(cpp:79): bootstrap 估计 → 把 MPC 目标设成**当前测量姿态**(不是 defaultJointState)→ 接管瞬间不跳 → 阻塞等首个 MPC 策略。
- **SafetyChecker**(`SafetyChecker.h`, 亲读): **仅检查 `pose(5)`**(ZYX 欧拉的 roll 分量)`> M_PI_2 || < -M_PI_2` 即 stop。⚠ 注意: 仅单分量姿态检查(非 doc 早期所述"roll/pitch 双查")。
- **MPC 诊断**(`LeggedController.cpp:212-217`): 每若干周期打印 `mpcTimer_`/`wbcTimer_` 的 Max/Avg `[ms]`(只在析构 flush)。**判据: MPC Max/Avg < 10ms(100Hz 预算)= 实时**。

### 2.6 Gazebo 仿真层(`legged_gazebo`)
- 插件 `LeggedHWSim`(`liblegged_hw_sim.so`): 实现 HybridJoint(存 pos/vel/kp/kd/ff)/ContactSensor/ImuSensor 三接口; `writeSim` 施加 `τ = ff + kd·(vel_des−vel_meas)`。
- **无 bumper 接触传感器**——接触由 KF 估计; base 地面真值用 `libgazebo_ros_p3d` 挂 `base`(给 cheater)。
- **IMU/接触不在 URDF, 靠 ROS 参数按 link 名查找**: `legged_gazebo/config/default.yaml` 的 `gazebo/imus`(`base_imu`) + `gazebo/contacts`(`[LF/LH/RF/RH_FOOT]`)。这解释了为何 go1 也没在 URDF 里写 IMU sensor。
- **⚠ fixed-joint lumping 两条静默坑(移植自定义机器人必踩, 精确根因)**: Gazebo 默认把 fixed joint 合并进父 link, 而本框架靠 link 名查找 → 两处必须显式禁合并:
  - **`*_foot_fixed` 必加 `<disableFixedJointLumping>`(或 `dont_collapse`)**: 否则 `*_FOOT` 被合并掉、不再是真 Gazebo link → ContactManager 报不出足接触 → **KF 收不到接触相 → 估计错**(URDF 改对了, Gazebo 行为却静默退化)。
  - **`base_imu_joint` 必加 `<disableFixedJointLumping>`**: 否则 `base_imu` 被合并, `LeggedHWSim::parseImu` 的 `GetLink("base_imu")` 返 null → **触发 `ROS_ASSERT` 崩溃**。
  - 这与根因 #1「无名 material 丢碰撞」同类(都是 URDF→Gazebo 解析期静默丢东西), 移植任何机器人都易踩。
- **足端/连杆 Gazebo 物理参数(站不滑不陷的关键数值, go1 与 dmgo 完全相同, 必抄)**: 足端 `mu=0.6 + kp=1e6 + kd=100`(接触刚度/摩擦, "站立不滑不陷的关键"); calf `mu=0.6 + self_collide`; thigh `mu=0.2 + kp/kd`; hip `mu=0.2`。传动 = **`EffortJointInterface`**(力矩接口, WBC 前馈力矩直接走传动)。"站不稳/打滑/陷地"时即此组参数的直接调参点。

### 2.7 理论出处
- **Grandia 2022 arXiv:2208.08373**(Perceptive Locomotion through NMPC)= 本栈 NMPC 本体(本仓取平地子集)。
- **Sleiman RAL 2021**(统一全身 MPC, 质心模型+多重打靶); **Bellicoso Humanoids 2016**(HoQp 分层 WBC); **Flayols Humanoids 2017**(线性 KF); **Liao IROS 2023 arXiv:2212.14199**(Walking in Narrow Spaces, duality-CBF, 作者安全约束应用层)。

---

## 第三部分 · 工程实战与踩坑 / 失败经验(本项目精华)

> 实践场景: 把 SolidWorks 导出的自定义四足 **dmgo** 移植进 legged_control(仅 Gazebo, 对标 go1)。M1(站稳+trot 跟 cmd_vel 直走)卡了**多个会话**, 三大根因**都不是"看上去那个"**。每条按 **症状 → 看似合理的错误理论(红鲱鱼)→ 真根因 → 怎么确认 → 治本 → 教训**。

### 3.0 方法论母题(贯穿全程, 最值钱)
- **对照实验 + 不变量守恒核对 >> 在自己机器人上盲调**(本项目最大收获): 用已知好的机器人(go1)走同流程做对照, 再核对碰撞数/维度/质量等守恒量, 比盲调姿态/参数高效**一个数量级**。
- **"研究同栈可用参考的『为何能 work』→ diff config → 适配"**: go1 的 config 就是 dmgo 的答案表; 逐项 diff **两次一击命中**(腿宏对称、sqpIteration)。
- **连续 >3 次治标失败必转调研**(three-strikes rule); 不闭门造车(源码 + deepwiki + WebSearch + 用户笔记交叉验)。

### 3.1 根因 #1: 无名 `<material>` → 静默丢碰撞 → 穿地(M1 站立堵点)
- **症状**: dmgo cheater STANCE 站不住, WBC qpOASES "Maximum number of working set recalculations" 刷屏 → 垃圾力矩 → 倾倒 → SafetyChecker 自停 → base 塌到 z≈−0.17(每次稳定同值, 纯 roll qx→−0.87)。go1 同流程却腹部正立静息。
- **红鲱鱼 A「MPC 欠支撑, Fz/mg≈0.90」**: 插桩测 Σf_z/mg≈0.90<1, 直觉"支撑不足→塌"。**证伪**: 亲读 `LeggedRobotQuadraticTrackingCost.h:61`——R 代价参考=mg, 模型无 below-mg 偏置; **0.90 是塌陷中的瞬态假象, 是果不是因**。
- **红鲱鱼 B「initialState 失配 / limp 倾倒」**: 把 initialState 改趴卧后 core-dump 消失(看似坐实)。**证伪**: 机器人**仍在穿地**; core-dump 消失只因趴卧初值给了 SQP 可行猜测。走过的治标弯路(50Hz `set_model_configuration` pin→关节速度尖峰→连 go1 都坏 qpErr 19250; 关节阻尼 2→25→脚陷地几何退化 core-dump; base-pin 没生效)——**仓库根本无 pin 机制**(deepwiki 证实), 方向就错。
- **真根因**: dmgo URDF(SolidWorks 导出+手改, 左右腿格式不一致)有 **14 个无 `name` 属性的 `<material>`**(非法 URDF)→ urdfdom/sdformat 在 URDF→SDF 解析时**静默丢弃 17/20 个 `<collision>`**(`<visual>` 仍保留 → **RViz 看得到但 Gazebo 物理穿模**)→ 几乎无地面碰撞 → 穿地下沉。
- **怎么确认(金标准, 可复现)**:
  1. **go1 同容器对照**(`go1_rest.sh`): go1 自然落体稳息 z=0.057 w=1.0 → harness/world/cheater 健康 → 堵点 100% dmgo 专属。
  2. **碰撞数守恒核对**: `gz sdf -p dmgo.urdf | grep -c collision`: **URDF 20 → SDF 3, 丢 17, 零 warning**。
  3. **隔离铁证**: 给无名 material 加 name → SDF 碰撞 **3→17**。修后 limp 腹部静息 z=0.120 w=1.0。
- **治本**: 所有 link 的 material 命名(后并入新腿宏架构, 全 link 命名 material, 永久根治)。
- **教训**: ①**`gz sdf -p robot.urdf | grep -c collision` 核对 URDF↔SDF 碰撞数守恒 = 排查穿地的金标准**(URDF 数 == SDF 数; 丢失多半是无名 `<material>` 或其他非法 URDF 让解析器静默丢碰撞); ②用瞬态插桩值(0.90)反推稳态结论会被带沟里, 先读代价/约束的数学定义; ③一个 hack 让症状部分缓解(趴卧初值消 core-dump)≠ 找到根因。
- **保留的正确实践(非根因但仍对)**: `initialState` 应 ≈ 实际 spawn 姿态——OCS2 移植自定义机器人的真实坑, 失配症状(乱动/求解崩/core-dump)极易误判成模型/惯量错。

### 3.2 根因 #2: 逐 link 导出左右永不 bit-exact → veer/自旋(trot 走直堵点)
- **症状**: trot 跟 cmd_vel 严重左偏(x:−0.04→10.4m, y:0→8.6m, w 1.0→0.73), 复合弧线/侧 crab; 开环甚至自旋出局仅 0.67m。
- **被证伪的补偿器(band-aid, 三连试均不干净)**: ①航向锁 `wz=−K·yaw`→把方向漂移转成侧向 crab(y→−8.7m); ②path-follower(锁 world y+yaw)→半奏效但纯补偿非治本; ③KFE z 单点修正→**翻转**漂移方向(左偏→右偏+螺旋)→暴露是**多因素残差非单点可补**。
- **侦探链关键一环(M1-029/030, 最先锁定的主导病灶)**: `check_lr_symmetry.py`+`verify_pose` FK 最先揪出的**头号几何不对称 = RF/RH KFE origin z=−0.00305(右膝低 3mm → 右脚低 3mm → 非对称站姿 → 左偏)**, verify_pose 实测 z-spread 3mm 吻合(数量级 ~0.3mm 级, 远大于后述 µm 级残差)。M1-030 单修它(z-spread 3mm→0.4mm 几何已更对称)却使 veer **方向翻转**(左→右+螺旋, w 1.0→−0.76, 转~140°)——"修掉最大病灶反而翻车"正是顿悟"需结构性对称、单点磨收敛慢"的转折点, 直接催生了下面的腿宏重构决策。
- **真根因**: dmgo 4 条腿逐 link 独立硬编码 → 左右**永远不是 bit-exact 镜像**, 残留 L/R 不对称(KFE origin z 3mm 为最大项; 修后仍余 HFE origin y +0.256mm、thigh inertia ixz ~27%、base com y +2.7mm 等)→ 每步态周期注入微小左右力/力矩差 → cmd_vel **无绝对航向锁**下自由积分成弧线/自旋。
- **怎么确认**: go1 决定性对照——go1 同 harness 走直 Δy/Δx=0.03 w=1.0 vs dmgo 0.82 w→0.73 → dmgo 专属。**为何 go1 走直**: go1 4 腿由 `common/leg.xacro` 实例化 4 次(`mirror=±1`/`front_hind=±1`)= **构造性 bit-exact 镜像**: HFE/KFE/foot origin 的 x 恒 0 → 前后腿运动学完全相同; 所有 y 偏置和非对角惯量(ixy/iyz)恰好 ×±1 = 结构上零腿级 L/R 不对称。
- **治本(非补偿)**: 仿 go1 把 dmgo 4 腿重构成**一个参数化宏 `dmgo_leg(prefix,mirror,front_hind)` 实例化 4 次**(go1 式 3 层: `const.xacro` 参数 → `leg.xacro` 腿类+per-leg gazebo+嵌套 `transmission.xacro` → `robot.xacro` 总汇总)。镜像规则 **verbatim 抄 go1**: `origin y / com_y / ixy / iyz → ×mirror`; 对角惯量/ixz 不变; hip 段 x/com_x/hip ixz → ×front_hind; **关节轴绝不镜像**(HAA `(1,0,0)`/HFE·KFE `(0,1,0)` 4 腿全同, 左右靠站姿角 ∓HAA 实现——翻轴会双重取反反而破坏对称)。**最易写错的特例: hip 的 `ixy` 同时受 mirror 与 front_hind 影响 = `ixy×mirror×front_hind`**(xy 惯量积对 x 翻和 y 翻都变号)——漏掉这层双重取反, hip 惯量积符号错会重新引入左右残差。
- **验证**: 展开 URDF **L/R 最大残差 = 0.000e+00**(bit-exact); SDF 17 collision 0 drop、0 空名 material(顺带永久根治 #1)。结果开环 cmd_vel trot 从"自旋出局 0.67m"→"前进 7.5m 跟踪不摔不旋"。
- **教训**: ①**左右不对称是真 bug**(残差), **前后不对称站姿是非 bug**(构造)——go1 的 mirror 乘子一眼区分两者; ②治本是改几何对称, **补偿器只是把弧线 veer 换成 crab veer(搬家不是修)**; ③**结构性对称 > 一次性脚本镜像**(改一处 const.xacro, 4 腿自动保持 bit-exact, 永不退化)。

### 3.3 根因 #3: `sqpIteration=10` 诊断 crutch → "边缘稳定"(最隐蔽、最核心)
- **症状**: cheater 下 0.13 横/偏航摆(被误判成"残余不对称")、真 KF 下 veer/自旋/摔、comHeight 0.28 摔——**全是同一个根因的下游**。
- **真根因**: `task.info` `sqpIteration=10`(诊断期为强制 MPC 收敛留下的 crutch)。每 MPC 周期跑 10 次 SQP ≈ 13.5ms **串行** > 100Hz 的 10ms 预算 → MPC 落后实时 → `MPC_MRT_Interface` 喂 **stale/laggy policy** → 控制不够脆 = 边缘稳定。cheater(完美态)下只表现为 0.13 摆; 真 KF(带估计噪声)下滞后+噪声叠加 → 翻。
- **怎么确认**: ①切真 KF 后 dmgo veer/摔, 但 `kf_estimate_vs_truth.py` 实测航向误差 max 3.7°/mean 0.21°(**KF 本身没问题**); ②跑 **go1 真 KF 基线**(`m1_trot_go1_kf.sh`)Δy/Δx=0.01 稳如磐石 → dmgo 专属 gap; ③**diff go1 vs dmgo task.info**, 标志差 = sqpIteration go1=**1** / dmgo=**10**; ④适配 `10→1` → 一发命中: 真 KF 下 Δy/Δx=0.013, qpErr 14→0, w=0.998, 14.4m 直走不摔(复现 2 次)。
- **机制更正(关键, session-7 实测推翻初判)**: Phase 5 初写"≤4-CPU 撑不住"——**核数不是因**。mpcTimer 实证(`m1_bench.sh`): sqp10 在 4 核(13.73ms)与 8 核(13.43ms)耗时**相同**; 加核 + 加 `nThreads`(3→6: LQ 近似 2.02→1.25ms)都救不了(仍 11.69ms>10ms)。
  - **SQP 各部分可并行性**(读 `SqpSolver.cpp` + 实测): **LQ 近似/线性化** = embarrassingly parallel(nThreads 3→6: 2.02→1.25ms, 亚线性); **线搜 rollout** = 并行; **HPIPM 解 QP**(沿时域 Riccati 递归, **本质串行**)→ 始终 0.29ms 不变 = **并行地板**(这就是压不到 10ms 以下的原因)。`OMP_NUM_THREADS=1` 只限 BLAS, 不限 OCS2 std::thread 池。
  - **`sqpIteration=1` = Real-Time Iteration (RTI) 方案 = 正确实时设计, 非妥协**: RTI(Diehl 等)每采样时刻只做 1 次 SQP 迭代, 靠周期间 warm-start 连续性 + 拆 preparation/feedback 两相消除反馈延迟。go1/a1/aliengo 全用 1(deepwiki+WebSearch+读源码三方独立印证)。
  - **"RTI 还不够准"时的真·SQP 提速菜单(超出 RTI 之后怎么办, 全有论文出处)**: ①**GNRK**(Gauss-Newton Runge-Kutta 高效离散化, 长时域 + 最小二乘代价减 LQ 成本, arXiv:2310.20390); ②**Multi-level RTI**(混合廉价/昂贵更新跨周期, optimization-online 6425); ③**partial condensing**(HPIPM 支持, 调 horizon 块大小权衡 QP 成本); ④**move-blocking / 缩短时域 / 粗 dt** 减节点 → 减 LQ+QP work。即"实时的加速永远是少算, 但少算有多种比'RTI 1 迭代'更聪明的菜单"。
- **教训**: ①**"边缘稳定/漂移"先怀疑控制实时性**(MPC 频率/迭代数/算力预算), 别急着归因模型不对称/调参——一个 sqpIteration=10 让 0.13 摆、comHeight 全被误判成下游红鲱鱼; ②**"加核能否让昂贵 solver 实时"正解常是"不"**——先分清可并行部分(LQ)与串行地板(Riccati QP); 实时的"加速"是**少算**(RTI 1 迭代 + warm-start)不是多并行; ③**`mpcTimer` Max/Avg < 预算是判据, 移植任何机器人后必查**(比盲调稳定性参数更早定位"边缘稳定"是否其实是 MPC 算不过来)。

### 3.4 诊断期 crutch 必须登记 + 事后清零(独立审计教训)
本项目踩过 **3 个同类 crutch, 全是诊断遗留、全误导后续分析**:
- **R 接触力权重 1.0→0.01**(源于已证伪的 Fz/mg 红鲱鱼; 弱 100× 力正则让 MPC 激进、放大左右力不均)→ 已回退 1.0。
- **torqueLimitsTask 真实值→80/80/80**(诊断 qpOASES RET_MAX_NWSR 临时抬, 但根因是模型不对称非力矩不可行)→ 已回退。
- **sqpIteration 1→10**(根因 #3 本体)→ 已回退 1。
- **独立审计(M1-051, 用最小提示词的独立 opus 子 agent 复核 headline)揪出**: ①**真 bug**: WBC torque 限 20/55/55(task.info)与 URDF effort(const.xacro 注释自陈已改 90)矛盾——flat 20 / perceptive 55 / URDF 90 三处冲突, WBC 卡死本该有的 roll 恢复力矩, **待统一真电机规格**; ②**过度声明**: go1-parity Δy/Δx=0.013 是单次最佳, 实测 csv 是 0.067(5×)、均速 0.488(非 0.64)、yaw 漂 15.2°——诚实表述应为"横漂 0.013–0.067 随 run, 同 go1 量级"; ③**过度声明**: KF "带噪 IMU" 实为干净(`LeggedHWSim.cpp:96` TODO 未加噪、只播协方差; 估计滞后真值 13%); ④**crutch 未清**: `recompileLibrariesCppAd=true` 未还原(启动慢)。
- **电机规格冲突的定案 + 失败机理(P3-031, const.xacro:21-25; 把上面 ① 的"待统一冲突"收成结论)**: 定案=**三关节同一电机, 额定 30 Nm / 峰值 90+ Nm**, URDF effort 统一取峰值 **90**(早先 20/55/55 拆分是错的——同一电机必同 effort, HAA 绝不该是 20)。**精确失败机理**: HAA 力矩上界被卡在 20 → 在 **0.16 台阶顶(step crest)无法产生足够的髋外展(hip-abduction)力矩去遏制侧向 roll → 侧翻**; 统一到 90 才给出真正的 roll 恢复权限(sub-agent URDF diff 锁定 HAA=20 为"头号侧翻嫌犯")。**⚠ 至今残留的悬坑(只读源亲验)**: URDF effort 已改 90, 但 `task.info:296-301` 的 `torqueLimitsTask` **仍是 20/55/55**(注释 reverted M1-033)——而 **WBC 力矩上界恰恰读 task.info 不读 URDF**, 故"修了 URDF 没修 task.info", WBC 的 HAA 力矩仍被 20 饿死、roll 恢复权限未真正放开。这条"两份配置只改一份"的不一致是移植期典型陷阱(URDF 与 OCS2 config 各管一摊)。
- **教训**: ①诊断期临时调参必须登记 + 事后清零(建专门清单核对), 否则误导后续分析; ②**独立审计防自我说服**(最小提示词独立 agent 揪出 headline 过度声明 + 真 bug)。

### 3.5 步态触发的 UDP-only 大坑
- **症状**(M1-025): 用 `rostopic pub`/`rospy` 发步态 mode_schedule → `gaitRx=0`、机器人不 trot、stance 下追 cmd_vel 前倾 → qpErr 爆。
- **根因**: `GaitReceiver` 订阅用 **`::ros::TransportHints().udp()`(UDP-only)**(亲验 ocs2 `ocs2_legged_robot_ros/src/gait/GaitReceiver.cpp:43`)→ **rospy/rostopic(仅 TCP)连不上**。
- **治本**: 必须用 roscpp 键盘节点 `legged_robot_gait_command`(支持 UDP, 从 stdin getline 读 gait 名, 管道可喂)。另坑: 该节点链接 `libhpp-fcl.so` 实在 `/usr/local/lib`, launch 默认 LD path 不含 → 报 `cannot open shared object` → 跑前补 **`LD_LIBRARY_PATH=/usr/local/lib`**。非交互用法 `( printf 'trot\n'; sleep 600 ) | LD_LIBRARY_PATH=... <bin>`(sleep 保 stdin 开免 EOF busy-loop)。

### 3.6 cmd_vel 无绝对航向锁 → 横/斜向漂移(M2)
- **症状**: 开环纯横移漂 ~57°、斜向 ~110° 且倒退。
- **真根因**: `cmdVelToTargetTrajectories` 用 `target_yaw = currentPose(3) + wz·timeToTarget`(亲验 `TargetTrajectoriesPublisher.cpp:78`, 相对当前可能已漂的 yaw、**无绝对纠正**); `cmdVelRot=R(当前姿态)·cmdVel`。= OCS2 **速度控制(非位置控制)设计本身**——**go1 同代码同样漂**(只是 L/R 更对称漂得少), 非 dmgo 特有。
- **治本**: 外加闭环 **heading-hold**(`wz=−Kp·yawErr` 拉回绝对 0)→ 横/斜 yaw 漂降到 <8.5°; trot/standing_trot/dynamic_walk/static_walk 全三向干净直线。转向(原地旋/转圈)则 **wz 通道直传**(无 heading-hold)= 设计转向输入。
- **物理**: 对角(trot)/波动(walk)支撑宽 → strafe 横向稳; 同侧步态(pace/amble)支撑窄 → 侧移即倾覆翻车。
- **M2 全功能定量验收(硬证据)**: ①**转向近完美对称**——原地左旋 +111° / 右旋 −112°(xy 漂 <0.25m), 印证"腿宏 bit-exact 对称从直走延续到转向"; ②**转圈走** closure=0.18m(r≈0.6m, 闭合); ③**goal 导航** err_pos=0.00m / err_yaw=0°(REACHED); ④**amble 步态的物理边界**: 前进可走但侧偏最大(dy=0.839、tilt 8.5° 为所有 OK 项最高, 已在翻车边缘), 而**横移则灾难性翻车**(tilt 175.4° / yaw 28.4° / z=0.003 FELL)——量化佐证"同侧步态支撑窄、侧移即倾覆"; ⑤原地转**实测 ~0.32 rad/s < 命令 0.5**(MPC yaw 率跟踪有滞后但方向正确)。
- **教训**: "非前进方向走不直" 先查参考轨迹的航向逻辑, 别急着调步态/增益。

### 3.7 测试判摔的陷阱
- **绝不能用四元数 w 判摔**: `w=cos(yaw/2)` 是偏航分量, 纯旋转会让 w→0 被误判成"摔"(M2 v1 踩中, 把上层 yaw 漂当成 fall 而提前中止)。**正解: 用 z + roll/pitch 判摔**。

### 3.8 其他工程实证与坑
- **comHeight 误判为稳定因素**: "0.28/0.36 摔"曾归因高 CoM 不稳; 实为 sqpIteration=10 的 stale policy 下游。sqp1 下 comHeight 0.36(68% 腿伸展)站 z0.344 + trot 走 38m 全稳——**再证根因 #3 的下游性**。
- **"相机录制不可行"被翻案(算力受限的表象要区分是否真不可行)**: session-6 曾记"gazebo 相机软渲太重→4 核饿死控制器→trot 摔", 据此判定"相机录制不可行"; session-7 在 **8 核**下不复现(720p@20fps offscreen 相机 + 真 KF trot 走 38m 不摔)→ 证明"相机录制是**算力(核数)问题、非根本不可行**"。与根因 #3 互为镜像(那条是"加核救不了串行 solver"; 这条是"加核确实救了相机渲染")——元教训: **算力不足导致的失败, 要区分到底是不是真不可行**。
- **幻影 qpErr = CPU 争用/stale-gazebo 进程堆积, 非模型不可行(M1-035/036, 第 4 种 qpErr 成因)**: 对称模型 + config 回退后**开环 trot 首测 qpErr=8**(易误判成"对称没修干净/模型仍不可行"); `docker restart dmgo` 清掉残留的 stale gazebo 进程堆积后**复测 qpErr=0**(同一脚本同一模型)→ 铁证 qpErr=8 是**残留 ROS+gazebo 进程争用 CPU** 致 MPC/qpOASES 算不过来产生的**假 qpErr**, 而非 QP 不可行。**教训**: 归因 qpErr 到模型/约束前, 先确认容器干净、无残留 gazebo 进程争 CPU——幻影 qpErr 会 `docker restart` 后消失(这是与 §3.3 sqpIteration 同源的"算力不够→假象", 但触发物是进程堆积而非迭代数过高)。本栈 qpErr 至此共 4 种成因: pin teleport(19250)/ 步态 UDP 没接上致 stance 追 cmd_vel 前倾 / sqpIteration=10 stale policy(14→0)/ **stale-gazebo CPU 争用(8→0)**。
- **字节级抽取 > 手抄**: dmgo URDF 1526 行, 运动学(纯静态无 `${}`)用 `sed -n '24,848p'` 字节级抽取 + sed 改 mesh 路径, **避免手抄 800 行转写错误恰好落在 #1 杀手区**(关节零位/轴向)。
- **CppAD 缓存陈旧**(issue#79): 改 URDF 后旧缓存让 MPC 用旧模型/线性化巨慢 → 首跑 `recompileLibrariesCppAd=true` 再关, 或清 `/tmp/legged_control/<robot>`。
- **诚实的已知未优化项: Q 权重未按质量重标定**: dmgo **14.7kg** vs go1 **12.0kg**(+22%), 但 Q 全盘照抄 go1 未按新质量重调。已判定**非 yaw 偏置源**(故移植期暂不动), 但属移植后应随质量重标定的工程 caveat——质量是守恒核对量, 重机器人沿用轻机器人 Q 是潜在隐患。
- **DEBUG 构建致 Gazebo RTF~0.05**(issue#4)→ 必 `RelWithDebInfo`/`Release`。
- **OCS2 是 monorepo, 禁编全量**(缺 convex_plane_decomposition/grid_map, issue#87)→ 只编 `ocs2_legged_robot_ros` + `ocs2_self_collision_visualization`(会把依赖子图拉全)。
- **pinocchio/hpp-fcl 用 leggedrobotics fork + `--recurse-submodules`**(自带兼容版 eigenpy; **eigenpy 不 apt 装**否则版本冲突)。
- **关节序映射(非漂移源, 但易误判)**: OCS2 默认 `jointNames` 序 **LF,RF,LH,RH**(`ModelSettings.h:51-54`)≠ URDF 声明序/reference.info 注释序 **LF,LH,RF,RH**——但 `createPinocchioInterface` **按关节名建模/映射(不按位置)**, 只要 URDF 含全部 12 名即可。**风险提示**: 若站立时左右髋劈叉方向反/不对称, 第一个怀疑点就是这个序映射(回看 defaultJointState 左右 HAA 符号 ∓0.20 与实际腿对应)。
- **关节零位换算(dmgo vs go1)**: go1 零位=腿垂直向下(−z), dmgo 零位=腿水平向后(−x), 差绕 +y 轴 +π/2。解析推 HFE 偏置 −π/2; 但 dmgo 前后腿不镜像 + KFE=0 小腿已预弯 ~40° → **FK 实算取代解析值**: 前 HFE−0.14/后−0.68, KFE−1.42, HAA∓0.20(FK 验 4 足 centroid 对称、共面)。

### 3.9 资源护栏铁律(把失败域圈进容器)
- 主机内存是唯一硬约束(31G 仅 ~13G available)→ 容器 `--cpus=4 --memory --memory-swap`; **编译爆内存只 OOM 杀容器, 主机绝不冻**(cgroup 圈住失败域)。
- catkin `-j2 -p1`(OOM→j1); `OMP/OPENBLAS_NUM_THREADS=1`; 一次一容器; 长任务后台跑。
- **网络/进程隔离(护栏的另一半, 同属"圈失败域"方法论)**: 容器**无 `--network host`**、唯一 `ROS_MASTER_URI`/端口隔离(ROS1)、**pkill-by-name**(从文件跑脚本以避免 pkill 自杀)、`timeout` / `docker rm -f` 兜底防容器泄漏占内存。任何并行跑 ROS+Gazebo 的调试都需要这半边。
- 镜像只做 apt+git clone **不编译**(CppAD 模板单翻译单元吃 3–8G, `docker build` 无法限内存); **重编译挪到带 `--memory` cap 的运行时容器**。bind-mount 本仓到容器(改动留主机可迭代; 只挂 dmgo 专属 3 路径不覆盖原 .cpp 避免误触重编/.so 失配)。

### 3.10 PHASE3 准备期新发现(修正"M1/M2 已圆满"的乐观叙事)
> M1(站稳+trot 直走)/M2(多步态+转向+goal)定量验收通过后, 团队转 PHASE3(台阶/感知)准备, 在自定义四足 dmgo 上又揪出**两条独立于上述三大根因的新根因**, 并把"头号问题"从台阶/感知**重定到平地本身**。这两条直接修正了"comHeight 0.36 安全、三大根因全治本"的过度乐观结论, 是本项目最新、也最易被"M1 已完成"叙事掩盖的失败经验。

- **新根因 A · 关节摩擦/阻尼的模型-仿真失配致裕度侵蚀(P3-086, `const.xacro:26-27`, 全新 sim2real 类根因)**: dmgo URDF 原本 **`joint_damping=2.0`(=go1 `0.01` 的 200×)、`joint_friction=1.0`(=go1 `0.2` 的 5×)**(SolidWorks 导出/手填的离谱值)。**机理(源码注释亲验)**: **MPC 质心模型完全忽略关节摩擦/阻尼**, 而 Gazebo 物理真实施加它 → 相对 MPC 心目中"无摩擦的指令力矩", 实物响应变**迟钝(sluggish)** → 同样的前馈力矩产生不了预期加速度 → **稳定裕度被持续侵蚀(margin erosion)→ 翻**。**治本=回退 go1 值**(damping `0.01` / friction `0.2`, 源现已是此值)。**为何高价值**: 这是"模型与仿真在关节摩擦上失配"这一**极典型却隐蔽**的移植坑——它不报错、不丢碰撞、维度也对, 只让裕度悄悄变薄; 移植任何自带 URDF 的机器人都该先核对 joint_damping/friction 是否被 CAD 导出污染成大值, 与已吸的"无名 material 丢碰撞/左右不对称/sqpIteration"四者并列, 是第四类 sim2real 根因。配套手段=**trot 周期 0.6→0.70s 放慢(P3-087, 抄 aliengo 重机时序, 每步更长支撑相在俯仰步顶遏制 roll)**。

- **新根因 B · 根因重定性: 平地(flat)不稳才是 REAL root, 台阶/感知是下游(P3-078, `reference.info:4-10`)**: 源码注释直陈 **"dmgo 平地 trot 在 5m 处就 FLIPS, 而 a1 平地 rock-stable → dmgo 平地不稳才是 REAL root, step 是 downstream"**。即团队在 PHASE3 准备期发现**dmgo 即便在平地仍有残留翻车不稳**, 遂把它判定为比台阶更根本的头号问题(先治平地再上台阶)。**附 comHeight 实验(亲验注释)**: **0.40→稳(0/4 翻)、0.30→翻**, 最终恢复到 M1 可用的 **0.36**(配 stance: FRONT HFE−0.50 / HIND HFE−0.86 / 全 KFE−0.96, 足在髋下)。**为何高价值**: 这条"M1 看似完成、平地却仍存残余不稳, 且被重定为头号根因"的反转, 直接修正了本文(及源 00_index"当前阶段")定格的"M1/M2 完成、三大根因全治本"乐观叙事——**移植验收"通过"不等于"鲁棒"**, 平地的残余不稳必须在上感知前清掉, 否则台阶上的翻车只是它的下游放大。

### 3.11 其余 sim2real 实操约束(可迁移小经验)
- **`/cmd_vel` 须平滑限加速 |a|≤0.3**(`m1_trot_cmdvel.sh` 等脚本约定, 00_index/02 明载): 突变速度指令会让 MPC/WBC 难跟→失稳, 限加速度是跑直走/演示的隐性前提。
- **goal 导航(`/move_base_simple/goal`)必须发 `odom` 帧**(M1-050): `goalCallback`(`TargetTrajectoriesPublisher.h:40`)内 `buffer.transform(...,"odom")`, 发别的 TF 帧会查不到→**指令静默丢弃**(与 §3.5 步态 UDP-only 同类"指令发了但没收到"); 且验证须用 `/odom`(估计帧=goal 所在帧)而非 Gazebo 真值(KF 有位置偏移)。附洞察: goal 版无 cmd_vel hold, stand 步仍趴卧(z0.12), 但 trot+首个 goal 一来 MPC 直接"趴卧→站→走到目标"(无需预先 hold target)。

---

## 第四部分 · 可吸收进《机器人学笔记》的点

| # | 内容 | 适合进哪一部/章 | 价值 |
|---|---|---|---|
| 1 | **OCS2 质心动力学 NMPC 全栈数据流**(TargetTraj→ReferenceManager→SqpMpc→MRT→WBC→HybridJoint→KF 闭环) + 三层频率 | 最优控制/MPC 部 或 腿足运动控制章 | 一张图讲透 NMPC+WBC+估计如何拼成实时栈, 教学级 |
| 2 | **质心模型 MPC 的状态/输入/约束/代价构造**(stateDim24/inputDim24; 平滑摩擦锥 vs WBC 摩擦金字塔的两层表示) | MPC/腿足规划章 | 具体可落地的 OCS2 legged_robot OCP 装配模板 |
| 3 | **RTI(Real-Time Iteration)= sqpIteration=1 的实时 NMPC 原理 + 为何"加核救不了"**(LQ 可并行 / Riccati QP 串行地板; 实时靠少算非多并行) | 数值最优控制/实时 NMPC 章 | 罕见的"实时性 vs 算力"实证 + RTI 出处, 配 mpcTimer 诊断法 |
| 4 | **WBC 分层 QP 各 task 数学**(EoM/摩擦金字塔/支撑零速/摆动腿/接触力 + base task) + **四足纯前馈 vs 双足必须 base 反馈的对照** | 全身控制(WBC)章 | base task 前馈/反馈之辨是 quadruped↔biped 的核心区别, 含 braver 反面教训 |
| 5 | **线性 KF 腿式状态估计**(IMU 预测 + 足端正运动学更新, 按接触相调噪声) + cheater→KF 的 bring-up 门槛 | 状态估计部 | 经典 Flayols KF 的工程实现 + sim2real 调试范式 |
| 6 | **方法论: 对照实验 + 不变量守恒核对 + "研究可用参考的为何 work→diff→适配"** | 工程实践/调试方法论章(若有) | 可迁移到任何机器人移植/调试, 本项目最大收获 |
| 7 | **URDF→SDF 静默丢碰撞(无名 material)+ `gz sdf grep -c collision` 守恒核对** | 仿真/URDF 工程章 | 极高频隐蔽坑 + 金标准排查法, 凡用 Gazebo 必踩 |
| 8 | **构造性对称(参数化腿宏 mirror 乘子)消除 veer** + 区分"左右残差=bug / 前后站姿不对称=非 bug" | URDF 建模/对称性章 | "结构性对称 > 补偿器"的工程哲学 + 具体镜像乘子规则 |
| 9 | **cmd_vel 速度控制无绝对航向锁 + heading-hold 闭环治本** | 运动指令/轨迹生成章 | 速度控制 vs 位置控制的设计性差异 + 外环补偿范式 |
| 10 | **诊断期 crutch 登记清零 + 独立审计防自我说服 + 瞬态插桩反推稳态的陷阱** | 调试方法论章 | 元教训, 跨项目通用 |

> **跨项目**: 本项目(标准四足平地)是感知四足(legged_perceptive)的地基/前传; 摩擦锥/质心 MPC/WBC 的感知扩展(elevation-map/SDF/落脚多边形)在感知四足项目, 可联动吸收。

---

## 附 · 关键 file:line 索引(便于回查源码)
- NMPC: `legged_interface/src/LeggedInterface.cpp`(setupModel 142 / setupReferenceManager 158)、`cost/LeggedRobotQuadraticTrackingCost.h:61`、`SwitchedModelReferenceManager.cpp`、`MotionPhaseDefinition.h`。
- WBC: `legged_wbc/src/WbcBase.cpp`(vars 24 / EoM 97 / 力矩限 110 / 支撑不动 125 / 摩擦 142 / **base加速度 174** / 摆动 202 / 接触力 227)、`WeightedWbc.cpp`、`HoQp.cpp`。
- 控制器: `legged_controllers/src/LeggedController.cpp`(init 28 / starting 79 / update 101 / 估计 146 / setCommand 176 / mpcTimer 212-217 / MPC线程 225 / GaitReceiver 236)、`TargetTrajectoriesPublisher.cpp`(cmdVel 67-90, target_yaw 78)、`SafetyChecker.h`。
- 估计: `legged_estimation/src/LinearKalmanFilter.cpp`、`FromTopicEstimate.cpp`、`StateEstimateBase.cpp`。
- 仿真/接口: `legged_gazebo/.../LeggedHWSim`(`liblegged_hw_sim.so`)、`legged_gazebo/config/default.yaml`、`legged_common/.../HybridJointInterface.h`。
- OCS2 侧: `ocs2_legged_robot_ros/src/gait/GaitReceiver.cpp:43`(UDP)、`ocs2_legged_robot/.../ModelSettings.h:51-54`(默认 jointNames)、`SqpSolver.cpp`(threadPool 61)。
- 配置样板: `legged_controllers/config/{go1,dmgo}/{task,reference,gait}.info`(dmgo: sqpIteration 73 / torqueLimitsTask 296)。
