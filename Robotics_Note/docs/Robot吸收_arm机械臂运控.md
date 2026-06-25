# Robot 项目吸收 · arm_dm 六轴力矩控制机械臂 全栈运动控制

> 本文是对用户亲手做的传统规控工程项目 `arm_control`（六轴**力矩控制**机械臂 `arm_dm` 全栈运动控制）的消化吸收稿。
> 源项目：`/home/ziren2/pengfei/Robot/arm_control`（只读，未改动）。素材 = `docs/00–05` + `debug_log.json`（A-001..026，每条 hypothesis→action→result→verdict）+ `scripts/` 核心源码。
> 编排：① 项目概览 → ② 核心方法与算法（分模块、含关键公式/参数）→ ③ **工程实战与踩坑/失败经验**（现象→根因→解决/教训，用户最看重）→ ④ 可吸收进《机器人学笔记》的点。
> 全文凝练，去掉了纯环境配置噪声；失败/踩坑保留完整根因分析与教训。

---

## 一、项目概览

### 1.1 目标
自定义 6 轴**力矩控制**机械臂 `arm_dm`（达妙电机驱动）的全栈运动控制，覆盖用户四大目标：
**规划（参数轨迹优化）+ 避障 + 动力学力控（柔顺/阻抗/计算力矩/混合力位）**。
核心范式：**纯力矩驱动臂 → 真阻抗**（测位置出力矩，**无需 F/T 力传感器**）。

### 1.2 机器人本体
- 6R 串联：J1z / J2y / J3y（腰+肩+肘）+ J4x / J5y / J6x（球腕）。
- 关节限位：J1 ±1.57 / J2 −3.14..0 / J3 0..2 / J4 ±3 / J5 ±1.28 / J6 ±1.57 rad。
- 每关节 effort 26 N·m / vel 10 rad/s（达妙电机，力矩控制设计意图）。
- **关键本体特性（贯穿全项目，决定算法选择）**：**重肩肘 + 轻腕**的异质惯量臂——肩肘 J2/J3 惯量大、良态；腕 J4/5/6 真实 CAD 惯量仅 ~2e-4，是后续所有「腕病态」问题的物理根源。

### 1.3 技术栈（三层分层架构）
| 层 | 管什么（where/when/how） | 工具 |
|---|---|---|
| **规划** path | 无碰撞几何路径 | MoveIt2（OMPL / CHOMP；STOMP 镜像无插件 skip） |
| **轨迹** time | 给路径配时间 → vel/accel/jerk/**力矩** 可行 | TOPP-RA / Ruckig（+ 自写参数优化） |
| **控制** torque | 关节力矩执行 + 力/阻抗 | ros2_control（JTC + `JointGroupEffortController` + 自写力控节点）+ Pinocchio |
| 物理仿真 | 接触 + 渲染 | MuJoCo + `mujoco_ros2_control`（effort 桥） |
| 动力学 | RNEA/CRBA/Jacobian/重力 | Pinocchio |
| 环境 | — | ROS2 Humble，Docker 镜像 `arm-control:humble` |

**规划/控制解耦的落地方式**：规划**离线**（MoveIt move_group，position/mock 接口）产出关节路点 `.npz`；控制**在线**（力矩桥，effort 接口）加载执行。两者不同进程、不同接口，靠一个**轨迹文件**解耦——正是「MoveIt 规划 → ros2_control 执行」范式。

### 1.4 里程碑（M0–M4 全部完成）
| 里程碑 | 内容 | 标志成果 |
|---|---|---|
| **M0 地基** | 清洗 URDF → MuJoCo↔ros2_control 桥 + MoveIt plan→execute | 端到端到位误差 ~0.01（实测 joint2 −1.5786≈−1.57、joint3 0.9905≈1.0、error_code=1）；物理稳定+跟踪 |
| **M1 轨迹优化** | TOPP-RA / Ruckig / 自写参数优化 / 力矩约束 TOPP-RA / 物理执行 / OMPL vs CHOMP | TOPP-RA 时间最优 3.496s；自写=闭式 `Tₛ∝aₛ^{1/6}`；**力矩约束守真 ±26Nm 比 accel 盒快 4.6×** |
| **M2 避障** | 规划期 PlanningScene / 自写 CBF 反应避障 / 运动学零空间冗余 | OMPL 2/3 绕行 vs CHOMP 局部最优 0/3；CBF h≥0；**运动学零空间 EE 锁 ‖Δx‖=0.00mm** |
| **M3 力控（核心）** | 重力补偿 / Cartesian 阻抗 / 扰动柔顺 / 计算力矩 / 混合力位 | 柔顺 50N→0.2m=F/K；**计算力矩 4.1 vs PD+g 107mrad=26×**；混合打磨 |
| **M4 全栈 capstone** | 规划→计算力矩执行→阻抗力控（一条连续力矩会话） | 各层交接干净、可组合接力 |
| **独立审核 A-026** | opus 子 agent 复核 headline | 揪出过度陈述（"钉墙"未触墙、"干净矩阵"实病态等） |

---

## 二、核心方法与算法（分模块）

### 2.1 模型清洗（URDF → ROS2/effort/MuJoCo 三方可用）
SolidWorks 导出的 ROS1 URDF 有力控硬伤，`clean_urdf.py` **逐字复制几何/惯量数字、只做针对性修**（避免手抄错）：

| 修了什么 | 为什么 |
|---|---|
| link6 mass 0.003→0.15 + 惯量 0→diag(1e-3) | 零惯量=**质量矩阵奇异**→MuJoCo 编译 + 计算力矩/阻抗第一步就崩；1e-5 又太小→stiff-actuator 数值爆（A-007）→定 **1e-3 占位**（真 EE 待精修） |
| 加 world link + fixed joint→base_link | 固定基座臂 |
| 删 6 个 `<transmission>`（ROS1 PositionJointInterface）+ 8 个 `<gazebo>` 块 | ROS2/effort/MuJoCo 不兼容 |
| 空 `<material name="">` → `mat_<link>` | Gazebo/MuJoCo **静默丢弃无名 material**（同 dmgo 穿地 bug） |
| **未动**：base_link 500kg（SolidWorks 默认密度残留，固定后无害）、link3 惯量（重验**正定**合法 CAD：minors>0, ixz<√(ixx·izz)；反直觉例：**ixz=0.00082>ixx 仍正定合法**，曾因此过度标注后撤回） | 不乱改有效 CAD |

**ros2_control xacro 参数化**：顶层 `arm_dm.xacro` 带 `arg hardware=mock|mujoco|mujoco_torque`，对应三套硬件接口（mock GenericSystem / MuJoCo position / MuJoCo effort）。**双命令接口**（position + effort），M0–M2 走 position（JTC），M3–M4 走 effort。

### 2.2 M1 轨迹优化：path ≠ trajectory
**核心区分**：path `q(s)`（无时间，几何形状，规划器产）vs trajectory `q(t)`（有时间，含速度/加速度/jerk/力矩，时间参数化产）。一条 path 乱配时间硬跑 → 违反限速/限加速/限力矩、jerk 无穷（激振、齿轮冲击）。

**时间参数化三件套（区别是重点）**：
| 算法 | 最优性 | 约束 | jerk | 在线? |
|---|---|---|---|---|
| **TOTG**（Kunz–Stilman 2012） | 时间最优 | vel + accel | ∞（bang-bang） | 否；MoveIt 默认 |
| **TOPP-RA**（Pham 2018） | 时间最优 | vel + accel + **力矩/2 阶** | ∞ | 否；凸稳快，**可吃机器人动力学** |
| **Ruckig** | 非路径时间最优但 **jerk 有界** | vel + accel + **jerk** | 有界（S 曲线） | **是**（每周期重算） |

- **TOPP-RA / TOTG** = "在 vel/accel 盒子里贴边跑最快"，加速度像开关（bang-bang）→ 最快有冲击。
- **Ruckig** = "限制 jerk 让加速度连续"→ S 曲线，慢一点但护电机/减速器，适合在线到点/避障重规划。

**自写参数轨迹优化（用户原话强调，非只调库）**：
- 表示 = 分段 **quintic 五次多项式**（位置/速度/加速度边界都可指定 + jerk 连续）或 B-spline（控制点=决策变量）。
- 决策变量 = 各段时长 `Tₛ`（rest-to-rest）；目标 = min ∫‖jerk‖²（min-jerk 五次的代价闭式 = `Σ 720·Δ²/Tₛ⁵`）；约束 = `ΣTₛ=预算` + `Tₛ≥Tₛᵐⁱⁿ`。其中 `Tₛᵐⁱⁿ` 由 min-jerk 五次段的三个峰值闭式系数反推后取 max：**peak\|vel\|=1.875·\|Δ\|/T、peak\|accel\|=5.7735·\|Δ\|/T²、peak\|jerk\|=60·\|Δ\|/T³** → `Tₛᵐⁱⁿ=max(1.875Δ/Vmax, √(5.7735Δ/Amax), ∛(60Δ/Jmax))`（这三系数是约束反推能落地复现的关键定量）。
- 求解 = `scipy.optimize` SLSQP；**关键验证**：scipy 数值解**精确等于**闭式 Lagrangian 解 `Tₛ∝aₛ^{1/6}`（把时间分给大位移段；jerk 代价 8910→7805 即 −12.4% vs 等分）→ 证明吃透了"参数化 + 带约束优化"内核。

**力矩约束 TOPP-RA（M1-⑥，桥接 M1↔M3 力矩主题，最有价值的轨迹结论）**：
- 动机：固定加速度盒（±8）**忽略位形相关惯量 M(q)**——同一加速度在不同位形需的关节力矩天差地别（`τ=M·q̈+C·q̇+g`）。accel 限的时间律可能要 >26Nm（真机不可行）或浪费驱动力。
- 做法：toppra `JointTorqueConstraint(inv_dyn, ±26, fs=0)`，`inv_dyn` = Pinocchio `rnea`（全逆动力学）。
- 结果：(A) accel±8 → T=3.496s 但 **peak|τ|=仅 4.1Nm（浪费 84% 的 26Nm 驱动力）**；(B) torque±26 → **T=0.752s = 快 4.6×**，**peak|τ|=26.0Nm 精确**（J1/J2 贴满 26 = 力矩限下 bang-bang 时间最优）。
- **discretization 踩坑（守约束的隐蔽坑）**：默认 Collocation + 稀疏栅格做 const-accel 重构时，栅格点之间 M(q) 在变 → 重构出的峰值力矩**超 ±26 达 6%**（看似守约束实则越界）；修法 = `discretization_scheme=Interpolation` + **401 栅格点** 才修正到 ≤26。对任何用 toppra 做力矩/二阶约束参数化者有直接迁移价值。
- 教训：**力矩感知的轨迹时序必须走逆动力学**；固定 accel 盒既不安全（可能超力矩）又浪费（留余量）。

### 2.3 M2 避障：规划期 + 反应式两层
| | 规划期 / global | 反应式 / local / online |
|---|---|---|
| 何时 | 执行前搜整条无碰撞路径 | 执行中每周期微调速度 |
| 谁 | OMPL（采样拒碰）/ CHOMP（障碍代价梯度） | MoveIt Servo / **CBF** / 冗余零空间 |
| 保证 | 全局（概率完备/局部最优） | 局部安全，不保证到目标 |
| 局限 | 障碍变了要重规划 | 可能陷局部、非全局最优 |

实务 = 两层叠加（规划期出标称路径 + 反应式兜底动态/未建模障碍）。

**规划期（M2-①）**：MoveIt 用 **FCL** 查碰撞；加障碍 = 发 `CollisionObject`（service `/apply_planning_scene` 同步 或 topic `/planning_scene` 异步 diff），frame_id=模型根 `world`。**核心教学点**：自由空间 OMPL≈CHOMP（同测地线，len 2.762），**加障碍后分野**——OMPL（采样、概率完备）成功绕障（2/3，len→3.184 即 +15%，smooth 0.0106→0.0148 即路径更曲/jerkier）vs CHOMP（局部优化）卡死（直线初值穿过 box，障碍梯度推不出局部最优 = CHOMP 经典弱点）。

**自写 CBF 反应避障（M2-②，贴控制内核）**：
- 标称 resolved-rate `q̇=J⁺·k·(p_goal−p_ee)` 冲向障碍后方目标。
- 安全函数 `h(q)=‖p_ee−p_obs‖−(r+margin)≥0`；每步**闭式投影**到安全集 `{q̇:∇h·q̇≥−α·h}`——单约束**无需 QP 解器**：若 `∇h·q̇_nom<−αh` 则 `q̇=q̇_nom+(−αh−∇h·q̇_nom)/‖∇h‖²·∇hᵀ`。∇h 用 Pinocchio 位置 Jacobian。
- 结果：标称 min h=−0.046 穿入；CBF min h=+0.008 安全（h≥0）且到目标 = 绕行。**形式化安全**，可叠在任意标称控制上。

### 2.4 冗余零空间解析（M2-④ 运动学 vs M3-⑥ 力矩级 —— 全项目最深的对照）
本臂 6 DOF，主任务若是 6D 位姿则**无运动学冗余**；**主任务降维到 3D EE 位置** → 留 6−3=3 维零空间做自运动（绕固定工具重构姿态）。

**运动学/速度级（M2-④，干净正解）**：
```
q̇ = J⁺·(ẋ_des + K·(x_des − x)) + N·q̇_sec ,   N = I − J⁺J  (运动学投影，无质量矩阵)
```
- `J⁺` = 阻尼右伪逆 `Jᵀ(JJᵀ+λI)⁻¹`；ẋ_des=0 → J⁺ 项闭环锁 EE，N·q̇_sec 在零空间驱动姿态自运动。
- 结果：**EE 精确锁 ‖Δx‖=0.00mm**（仅积分误差），自运动 J1=0.44rad（基座 yaw 摆 ~25° 工具不动）。MuJoCo 位置 JTC 真物理执行。
- **自运动为何总 J1（基座 yaw）主导**：EE 几乎位于基座 yaw 轴上，对 3D 位置任务 J1 是最自由的 DOF——把"臂绕固定工具重构"从现象升为几何洞察。

**力矩级 OSC（M3-⑥，受病态 M 限制，详见三、根因 2）**：动力学一致投影 `N=I−Jᵀ(JM⁻¹Jᵀ)⁻¹JM⁻¹`（Khatib）——自运动出现了但 **EE 漂 60-110mm**，根因=质量矩阵病态。**结论：异质惯量臂的冗余解析正解 = 运动学/速度级（无 M⁻¹），力矩级 OSC 只在良态臂可行。**

### 2.5 M3 力控（项目核心）：阻抗 / 导纳 / 混合
| | 阻抗 impedance | 导纳 admittance | 混合 hybrid |
|---|---|---|---|
| 测→出 | 测位置/速度→**出力(力矩)** | 测**力(F/T)**→出位置 | 选择矩阵分方向 |
| 硬件 | **力矩臂**（本项目），无需 F/T | 位置臂 + F/T | 位置/力轴混合 |
| 公式 | `τ=Jᵀ(−KΔx−Dẋ)` | `ẋ=M⁻¹(F_ext−…)` | Raibert-Craig 选择矩阵 S |
| 特点 | 稳、接触友好、简单 | 高刚度跟踪好、接触易不稳 | 某轴控位某轴控力（打磨） |

**本项目=阻抗**（力矩臂→真阻抗，无需力传感器，最佳路线）。effort 控制路径：
```
Python 力控节点 ──Float64MultiArray τ──▶ /arm_eff/commands
 (sub /joint_states, Pinocchio 算 J,g,M, 200Hz)        │
                                                       ▼
   effort_controllers/JointGroupEffortController ──▶ mujoco_ros2_control ──▶ MuJoCo <motor>(torque)
```
- MJCF：`<position>`（kp150）→ `<motor>`（gear=1, ctrl=±26）；ros2_control `hardware:=mujoco_torque` 走 effort 命令接口。
- Pinocchio API：g(q)=`computeGeneralizedGravity`；M(q)=`crba`；逆动力学 τ=`rnea(q,q̇,q̈)`；J=`computeFrameJacobian`。

**M3-① 重力补偿** `τ=g(q)`：验 effort 通路（joint1 在 8Nm 下动、joint2 在 −6Nm 下到限位 = 强力矩权限）。

**M3-② Cartesian 阻抗**：`τ=Jᵀ(−K_c·Δx−D_c·ẋ_ee)+g(q)`，K_c=250 N/m，**D_c=2√K_c（临界阻尼，无超调）**，clip±26。HOLD ‖Δx‖=0.0mm（平衡处弹簧项=0，τ=纯 g(q)）；WIGGLE 跟踪 ±8cm 振荡目标 ‖Δx‖≈30-36mm 滞后（有限刚度柔顺）。

**M3-③ 外力扰动柔顺**：MuJoCo `external_wrench` plugin 给 link6 施世界系外力 → EE 退让 → 撤力弹回。**教科书结果：50N 推 → ‖Δx‖=199.9mm = F/K = 50/250 = 0.20m 精确吻合**；撤力弹回 1.2→0.1mm（线性弹簧律 + 临界阻尼无振荡）。纯力矩、无 F/T。

**M3-④ 计算力矩（computed-torque，头牌动力学控制算法）**：
```
τ = M(q)(q̈_d + Kp·e + Kd·ė) + C(q,q̇)q̇ + g(q) = rnea(q, q̇, a_cmd),  a_cmd = q̈_d+Kp·e+Kd·ė
```
- 反馈线性化：精确 M,C,g → 闭环 `ë+Kd·ė+Kp·e=0`（解耦、临界阻尼，任何可行轨迹都能跟）。Pinocchio `rnea` 一次算全逆动力学（M·a+C·q̇+g）。
- 时序用 `/joint_states` header stamp（sim time）→ 速度/加速度前馈正确，与回调率无关。
- **公平对比**（各自全新 spawn、同 home 起点）：**CT RMS=4.1mrad vs PD+重力 RMS=107.4mrad → CT 紧 26×**。PD+g 的周期滞后 50-200mrad 正是 CT 前馈的 `M·q̈+C·q̇` 项。
  - **隔离被测变量的精巧设计**：增益**异质** `KP=diag([150,150,150,150,100,60])`（腕端 J5/J6 故意降低，匹配其小惯量），D=2√Kp；只让高惯量 J2/J3 摆动（`ACTIVE`），其余握住。关键：被 hold 的 J1/J4-6 在 CT 和 PD+g **两种模式下都用 CT 的 hold**，度量只在 commanded joints 上算——这样既隔离"跟踪律"本身、又避免均匀增益 PD 去摇晃 2e-4 惯量的腕（否则 PD+g 基线会因腕失稳而不公平）。是"如何设计一个公平的算法对比（隔离被测变量、防无关 DOF 污染基线）"的范例。

**M3-⑤ 混合力/位控（Raibert-Craig 选择矩阵，打磨范式）**：
- 法向 x = **力控**（恒力 Fd=15N 顶墙，Jᵀ 开环力，无 F/T）；切向 y,z = **位控**（保持 + 横扫）。选择矩阵 S=diag(1,0,0) 取力轴，I−S 取位轴。`F=[Fd, −Kp(y−y_d)−Dẏ, −Kp(z−z0)−Dż]`，`τ=Jᵀ(3×6)·F+g`。
- 结果：法向 x 钉墙 mean=0.144 std=3.1mm；切向 y 跟 ±0.05 横扫（基座 yaw）。**±22mm 切向滞后=接触摩擦**（μ·15N vs Kp·0.08）= 物理正确，正是打磨需力控之因。
- ⚠ 诚实边界（A-026）：实际 EE 停在 0.144 < 墙面 0.20，差 5.6cm，**未真触墙**；15N 是开环无力反馈，std 3mm 是开环抖非接触摩擦。更硬的证据：受控 frame 停在 0.144，但 **link6 凸包前伸使 EE 几何最前端实际停在 0.116**（既≠墙面 0.20 也≠接触相记录的 0.144），EE 真实离墙更远。

### 2.6 M4 全栈集成 capstone（组合证明）
**「全栈」≠ 每个算法各跑一遍**，而是证明**层与层的交接（hand-off）干净、能组合接力**。一条连续力矩会话三相：
| 相 | 层 | 做什么 | 算法 |
|---|---|---|---|
| Phase 0（离线） | 规划 | MoveIt OMPL 规划 READY→precontact，`plan_only`，存 npz | OMPL |

**Phase 0 的两个预防性设计决策（吸取前期教训）**：① 规划起点选 **READY（q2=−1.0、重力矩 g2≈−0.2Nm 轻、M3-④ 已证可保持）而非重力重的 q2=−1.57 home**（后者会 under-track）；② 路点**全程锁腕 WRIST=0**，使规划路径永不驱动 2e-4 惯量腕 → 从源头规避计算力矩腕不稳（根因 3 教训的预防性应用）。
| Phase A（在线） | 控制 | 余弦缓动 仿真起姿→规划起点 | cosine ease |
| Phase B（在线） | 控制 | **计算力矩**跟踪规划路点 → 到 precontact | rnea |
| Phase C（在线） | 力控 | **笛卡尔阻抗**柔顺保持 + 摆动 | Jᵀ(−KΔx−Dẋ)+g |

结果：规划→计算力矩精确到位（q2→−1.00、precontact [−0.01,−0.54,1.36]，τmax~4）→阻抗柔顺接力（EE-z ~10cm 弹簧式摆动）。一条连续会话，纯力矩 + Pinocchio + MuJoCo，无 F/T。

---

## 三、工程实战与踩坑 / 失败经验（现象 → 根因 → 解决 / 教训）

> 这是项目精华。每条都不是"修了"，而是**为什么坏、怎么定位、根因、教训**。

### 根因 1 · MuJoCo 物理数值爆炸（link6 占位惯量 + stiff actuator + 无阻尼）
- **现象**（A-007）：MuJoCo 桥跑起来后物理发散——joint6 初值 −38.55 飞到 122.77 rad（命令仅 0.2），全关节飞出限位。
- **根因**：link6 占位惯量 **1e-5 太小** + kp=150 stiff position actuator + **无关节阻尼** → 刚性不稳定系统（ωn=√(kp/I)≈3873 rad/s，ωn·dt≫2 → 显式积分爆炸），最严重在惯量最小的 joint6。
- **解决**：link6 惯量 1e-5→**1e-3** + mass 0.05→0.15 + 注入全局 `<joint damping=1.0>` → 物理稳定（joint6 −38.55→1.35e-06 守 home）。
- **教训**：力矩臂/小惯量末端做物理仿真，**惯量量级 + 阻尼 + actuator 刚度三者要匹配**，否则显式积分发散；占位惯量不能太小。

### 根因 2 · 病态质量矩阵 → 力矩级 OSC 零空间在本臂做不干净（**最深洞察，穷尽验证**）
- **现象**（A-022）：M3-⑥ 实现 Khatib 动力学一致 OSC 投影 `N=I−Jᵀ(JM⁻¹Jᵀ)⁻¹JM⁻¹`，想 EE 锁死 + 零空间自运动。自运动出现了（J2/J3 重构 1.1-1.4rad），但 **EE 没干净保持（‖Δx‖~70-110mm 而非 <10mm）**。
- **定位 4 次尝试**：(1) 运动学投影 N=I−J⁺J 在力矩级泄漏 85mm；(2) 动力学投影 → **爆炸 ‖τ_null‖→18000**（腕 2e-4 惯量使 M⁻¹ 巨大 + 奇异处 (JM⁻¹Jᵀ)⁻¹ 爆）；(3) 正则 `M+λI`/阻尼逆 → 稳但 EE 70mm；(4) 锁腕 → 107mm，均无改善。
- **真根因**：**质量矩阵病态**。腕惯量 2e-4，cond(M)≈527-1625。精确投影数值不稳；为稳定加的正则**破坏 N 的投影性**（特征值不再 {0,1}）→ 放大 τ₀ 10-50× 漏入任务空间 → EE 漂。
- **量化定论**（A-023，`m3_link6_inertia.py` trimesh 真几何 + Pinocchio cond）：link6 是 1-2cm 小法兰（vol 3e-6 m³），真几何惯量 ~5e-6（比 1e-3 占位**小 200×**）→ 裸网格 cond **爆 13k-56k**；**1e-3 占位实为条件数拐杖**；realistic gripper（0.3kg/1e-2）砍半 cond 到 300+，但 **link4/link5 真 CAD 惯量 2-3e-4 是固有地板，不根治**。
- **gripper 实证收口**：更好条件数确实驯服投影（‖τ_null‖降 7× 1300→189），但 **EE 仍漂 62mm 且自运动塌缩**（J2/J3 仅 0.07/0.24rad）——"漏但动"换成"稳但几乎不动且仍偏 60mm"，都不干净。
- **教训**：① 病态不是占位 bug，是**本臂小腕设计固有**；② **冗余解析的层次选择**——异质惯量臂正解=运动学/速度级（无 M⁻¹，见 M2-④ EE 锁 0.00mm），力矩级 OSC 只在良态臂可行；③ **用第一性原理（条件数）判断算法可行性，而非在力矩层硬刚**。OSC 投影对质量矩阵条件数极敏感。

### 根因 3 · 异质惯量臂必须解耦低惯量腕（计算力矩跟踪发散）
- **现象**（A-019）：首版全 6 关节振 → RMS 813mrad 不跟。仪表化（log q/q_d/τ 逐关节）后发现：缓动相（慢）全关节 sub-mrad（CT 机理正确）；振荡相肩肘 J2/J3 完美，但**腕 J4/5/6 全幅发散**（j6 期望 0 却摆 ±1），J1（巨大等效惯量）卡住。
- **根因**：腕惯量 ~2e-4，自然时间常数 `I/damping≈2e-4/0.1=2ms（≈500Hz）` **快于控制率（~100-200Hz）→ 欠采样闭环发散**；MJCF `<joint damping>` 在 500Hz 物理层稳住它们，所以**不能补偿它**。
- **试错**：黏滞前馈 `τ+=0.1·q̇`（想让模型匹配 plant）反而**更糟**（撤掉了稳定阻尼），RMS 795 → 撤回。
- **解决**：换轨迹——**只振高惯量肩肘 J2/J3、握住腕+基座、温和 0.2Hz**。
- **另一发现**：均匀增益 PD 控全关节会**失稳**（腕 ωn=√(150/2e-4)≈866 rad/s ≫ 控制率）→ **计算力矩的 M(q) 加权天然给每关节正确力矩权限 = 异质惯量臂用逆动力学控制的第二理由**。
- **触目惊心的量化实证（A-025/M4）**：对 2e-4 惯量腕施原始 PD，**pre-clip τ 冲到 888 Nm，即便 clip 到 26Nm 后仍产生 130000 rad/s²（1.3e5）加速度发散**——这是"小惯量末端在控制率下原始 PD 必炸、不可 PD 控"最有说服力的实证。
- **教训**：重肩肘+轻腕的异质惯量臂，大范围计算力矩跟踪**必须解耦低惯量腕**，否则腕误差经质量矩阵耦合腐蚀主关节力矩（见根因 4）。

### 根因 4 · 腕误差经 rnea 质量矩阵耦合腐蚀主关节（M4 大范围跟踪三根因之一）
- **现象**（A-025）：M4 大范围重定位时，腕治好后 J2/J3 仍跟不上，**τ2 该 3.5 却跳到 15**，手臂卡住。
- **根因**：`a_cmd=Kp·e` 对**全 6 关节**反馈；漂掉的腕误差 e_wrist 经 **rnea 质量矩阵耦合项 `M[2,5]·a_cmd5`** 注入 τ2/τ3 → 污染。
- **解决**：**解耦** `a_cmd[腕]=0`（腕确实近静止），rnea 算 J2/J3 力矩时腕加速度按 0，耦合不再腐蚀；腕力矩另行温和保持（MJCF 隐式高阻尼 25 + 温和 Kp=3 定心）。**对比验证**：去耦合后 τ2 回到 ~3.5（与 M3-④ 基线一致），精确跟踪。
- **教训**：M3-④「只动高惯量关节、握住腕」在大范围重定位下要升级成「解耦腕」（零其 a_cmd + 阻尼兜底）。

### 根因 5 · M4 大墙挡住够臂（隔离变量法定位）
- **现象**（A-025）：手臂 Phase A 卡在 q2=−0.25（远未到 READY 的 −1.0），**τ 不饱和却不动**。
- **根因**：M3-⑤ 的大墙（0.6m 高、face x=0.20）**碰撞凸包挡住够臂运动**。
- **定位法**：跑**无墙基线**（proven m3_computed_torque）→ q2 干净到 −1.0、τ=3.5；有墙则卡 → **隔离变量法**锁定墙。
- **解决（双管）**：① prep 脚本本身把墙**缩小**——M3-⑤ 是 size 0.03×0.25×0.30 的大墙（高 0.6m、宽 0.5m、face x=0.20），M4 换成 half-extents 0.02×0.12×0.12 的**小面板**（只挡 EE 前方、不挡 link 凸包）；② capstone 重点是**组合**非重证接触 → Phase C 改用鲁棒的 M3-② 阻抗柔顺摆动（自包含、无脆弱接触几何）。

### 根因 6 · MuJoCo bridge headless 起不来（GLFW 强制）
- **现象**（A-006）：`mujoco_ros2_control` 加载 MJCF 成功，但 MuJoCo Simulate GUI 强制 GLFW → headless 直跑 `could not initialize GLFW` → 进程死 → controller_manager 没起 → spawner 永等。
- **解决**：Xvfb 虚拟显示（`xvfb-run` 套 launch），顺带 x11grab 录 mp4。
- **教训**：仿真器的 GUI/渲染依赖在无头服务器上是常见集成坑；虚拟屏 + 录屏一举两得。

### 根因 7 · `/joint_states` 名序错乱（red herring，定位浪费）
- **现象**（A-007/008）：看似 joint 1/2/3 控制坏了（裸 positional 读 mis-attribute）。
- **根因**：`mujoco_ros2_control` 的 `/joint_states` 按 **MuJoCo 内部 map/注册序**发布 = `[joint2,joint3,joint1,joint4,joint5,joint6]`，**非 joint1-6 名序**。裸下标读把腕/肩张冠李戴。
- **解决**：**永远按 name 数组读 `/joint_states`**；JTC 控制不受影响（它按关节名映射）。
- **教训**："joints 1,2,3 broken" 是 red herring，根因是索引错乱——**先验证数据通道再怀疑算法**。

### 根因 8 · external_wrench plugin 三连坑（plugin 加载机制不透明）
- **现象**（A-018）：M3-③ 第一次跑外力推，ApplyExternalWrench service 不存在 → 退化到"只 hold"。
- **根因 + 三处修**：
  1. **plugin 加载机制**：`mujoco_ros2_control` **只从 `mujoco_plugins` 参数字典加载 plugin**（非 MJCF `<extension>`、非自动）→ 建 `mujoco_plugins.yaml` 注册实例 `external_wrench`→`mujoco_ros2_control_plugins/ExternalWrenchPlugin`，`ParameterFile` 挂到 launch 的 ros2_control_node。
  2. **service 名**：是 `~/apply_wrench`（**非** `/apply_external_wrench`）→ 解析为 `/external_wrench/apply_wrench`（实例名命名空间）→ 运行时 `grep apply_wrench` 发现。
  3. **request 字段**：目标 body 名放 **`wrench.header.frame_id='link6'`**（srv 文档明确），力是世界系；之前 link6 放外层 header、'world' 放 wrench header（反了）。
- **附带坑**：`install/` 是**拷贝非软链** → 改了 src 要手动同步那 2 个文件到 install。
- **教训**：第三方 ros2_control plugin 的加载/命名/字段约定要查官方 srv/demo，不能猜；install 拷贝语义易踩。

### 根因 9 · 重力补偿/计算力矩的几个"非 bug"（避免误诊）
- **"home 不下垂"非模型 bug**（A-016）：MJCF 质量与 URDF **完全一致**（link1-6 0.51/0.92/0.74/0.39/0.39/0.15）、Pinocchio g(q) 正确。真因 = **joint2 起始在上限 0，重力把它推向限位 → 被限位夹住不动**（−6Nm 推离 → 摆到 −3.14）。→ 重力补偿正确。
- **damping 坑**（A-016）：M0-③ 为稳 kp150 位置 actuator 加的 `damping=1.0` 对**力矩控制太大**（遮蔽重力 + 抗阻抗 D 项）→ torque MJCF 降到 **0.1**（`make_torque_mjcf.py`）。
- **教训**：诊断驱动（零力矩自由落体 + 已知力矩权限 + URDF↔MJCF 质量对比）定位 = 限位夹持，非盲调增益。同一阻尼值对位置控制是"稳定剂"、对力矩控制是"干扰项"。

### 根因 10 · CHOMP 局部最优卡死（诚实失败，非 bug）
- **现象**（A-013）：加 box 后 CHOMP **0/3 失败**（−2 INVALID_MOTION_PLAN），OMPL 2/3 绕行成功。
- **根因**：直线初值穿过 box，**障碍距离场梯度推不出局部最优**（CHOMP 协变梯度的经典弱点）。
- **教训**：**失败是诚实结果非 bug**；缓解=多迭代/随机下降/OMPL 暖启动，但本质局部。自由空间 OMPL≈CHOMP，**障碍梯度的价值需有障碍才显**。

### 根因 11 · PlanningScene box 手调放置 2 次失败（自动化驱动）
- **现象**（A-013）：手调 box 尺寸 2 次失败——臂在小工作空间折叠，路径中点 box 总碰 GOAL 构型的 link **网格**（点检查不够，网格有体积；`-27=GOAL_STATE_INVALID`）。
- **解决**：**`/check_state_validity` 驱动自动放置**——Pinocchio FK 沿直线 EE 路径取候选中心，扫 `s∈{0.5,0.45,0.55,0.4,0.6,0.35,0.65}×size∈{0.10,0.08,0.06}`，判据=「START+GOAL valid 且 MID invalid 的首个」→ 选中 s=0.45 size=0.06。可复用细节：同 id 的 `CollisionObject.ADD` 即 **REPLACE**，故扫描时反复 ADD 即原位替换。
- **教训**："多花一次验数据 > 盲调尺寸"——用碰撞检查 API 自动驱动放置，而非手试。

### 根因 12 · 混合力位控 5 次诊断历程（折叠臂奇异 + 工作空间边界）
- **现象**（A-020）：混合控制相 B 反复飞/卡。
- **5 次迭代**：(v1) 关节切分 J1/J2/J3+腕 PD → 趋近饱和+乱飞；(v2) 关节切分+CT 趋近 → 趋近净但相 B 飞（切分相冲、折叠位 sub-Jacobian 奇异）；(v3) **全 6 列 Jacobian**（腕入零空间）+ 笛卡尔趋近 → 相 B 铁稳但笛卡尔直线趋近卡奇异；(v4) 全 Jac + **关节 CT 趋近**（可靠展开）→ 稳接触但 z 扫失败（压点在工作空间边界，径向+竖向被钉）；(v5=终) 扫掠改**零约束 DOF=横向 y（基座 yaw，等半径不受 reach 限）**→ WORKS。
- **三教训**：(1) **全 Jacobian 笛卡尔控制非关节切分**（冗余腕安全入零空间）；(2) **关节 CT 展开折叠臂**（笛卡尔直线撞奇异）；(3) **扫掠选 reach-free DOF**（横向/基座 yaw，非边界钉死的径向/竖向）。

### 根因 13 · 进程/线程生命周期坑（rclpy + pkill + effort 保持）
- **rclpy 回调内 shutdown 不停 spin**（A-019）：`rclpy.shutdown()` 从回调里调**不会**打断 `rclpy.spin()` → 用 `os._exit(0)` 存完硬退。
- **effort 控制器节点退出后保持最后力矩**（A-019）：run 间隙臂漂到限位 → 公平对比必须**各自全新 spawn**。
- **pkill 自杀**（A-017）：`pkill -f m3_impedance` 匹配到 wrapper 脚本自身（含"m3_impedance"）→ **137 自杀**（workflow 已知坑）→ 修 = `pkill -f m3_impedance.py`（精确到 node 名）。
- **教训**：清理进程的匹配串要精确到 `.py`/node 名，别误杀 wrapper；effort 控制器无命令时保持力矩，对比实验要隔离起点。

### 根因 14 · 独立审核揪出 headline 过度陈述（诚实记录是工程素养）
debug_log 本身诚实，但独立 opus 审核（A-026，最小提示词防错误先验）揪出 headline 过度：
1. M3-⑤"钉墙"实际 EE 未碰墙（停 0.144 vs 墙 0.20 差 5.6cm；开环 15N 无力反馈）。
2. A-019"cond(M)=527 干净矩阵"实测 **1625**（与 A-023"固有病态 600-1600"矛盾）。
3. link6 惯量 diag(1e-3) 是**虚构占位**（mesh 体积 2.98e-6→0.15kg=密度 50381 kg/m³=钢 6×；真几何 ~5e-6 小 200×），静默撑全部 M3/M4 动力学。
4. M4 计算力矩跟踪**部分靠注入 MJCF 阻尼 25 + 腕解耦**（非纯 `ë+Kd·ė+Kp·e`）。
5. M3-⑥ 零空间正则化（M+=0.02I 破投影）保证失败，再 over-generalize"OSC 本臂不可能"。
- **真 solid（独立验证通过）**：URDF↔MJCF 惯量一致 / 重力补偿 g(home) / M3-③ 柔顺 F/K=0.2m（τ1.8Nm 无 clip）/ M3-④ CT 26× / M1-③ 闭式 `Tₛ∝aₛ^{1/6}` / effort 通路。
- **教训**：headline 改诚实框定（"占位惯量下"/"开环力"），删"钉墙""干净可逆矩阵""全部验证"等绝对语。**诚实记录局限是工程素养**。

---

## 四、可吸收进《机器人学笔记》的点

> 标注哪些内容、适合进哪一部/章。本书已有 28 章 + Tutorial 融入多卷（含规划部、控制部、运动学/动力学）。下列按主题归并。

### 4.1 运动学/动力学部
- **path vs trajectory 的严格区分**（q(s) 无时间 vs q(t) 有时间）+ 时间参数化的必要性（限速/限加速/限力矩/jerk）→ 适合**轨迹生成/时间参数化**章开篇。
- **质量矩阵条件数与算法可行性**（cond(M) 决定 OSC 投影能否干净、决定腕能否 PD 控）——把"病态质量矩阵"作为第一性原理工具讲透，是难得的实证案例 → **动力学/数值稳定性**章。
- **异质惯量臂（重肩肘+轻腕）的固有病态**：小腕惯量 2e-4 → 自然时间常数 2ms(500Hz) → 控制率欠采样发散；ωn=√(kp/I) 判稳 → **动力学/控制率选择**章的工程注记。
- **Pinocchio API 速查**（rnea/crba/computeGeneralizedGravity/computeFrameJacobian 的用途映射）→ 动力学**计算工具**小节。

### 4.2 轨迹优化/时间参数化章
- **三件套对比表**（TOTG / TOPP-RA / Ruckig：最优性·约束·jerk·在线性）——可直接入书。
- **力矩约束 TOPP-RA**（`JointTorqueConstraint`+rnea，比 accel 盒快 4.6× 且精确守 ±26Nm；accel 盒既不安全又浪费 84% 驱动力）——**位形相关惯量 M(q) 使力矩感知时序必须走逆动力学**，是时间参数化章的高价值结论 + 实测数据。
- **自写 min-jerk 五次参数优化**：决策=段时长，闭式 `Tₛ∝aₛ^{1/6}` 与 scipy SLSQP 数值解精确吻合 → **参数轨迹优化/最优性验证**小节（讲"调库 vs 吃透内核"）。

### 4.3 运动规划/避障章
- **避障两层框架表**（规划期 global vs 反应式 local）+ 实务两层叠加 → 避障章总览。
- **OMPL（采样、概率完备）vs CHOMP（局部优化）**：自由空间等价、加障碍后 OMPL 绕行 vs CHOMP 局部最优卡死 → **采样 vs 优化规划**对比的经典实证（含"失败是诚实结果非 bug"的工程态度）。
- **CBF 速度滤波（单约束闭式投影，无需 QP）**：`h≥0` + `q̇=q̇_nom+(−αh−∇h·q̇_nom)/‖∇h‖²·∇hᵀ`，∇h 用位置 Jacobian → **控制屏障函数/反应避障**小节，干净可教的最小实现。
- **障碍正后方→反应式死锁** + 用 nominal 误差区分可达性 vs CBF 问题 → CBF 局限注记。**两个失败模式的具体破解（可复现关键，源 `m2_cbf.py:38-45`）**：① 死锁根因="sphere squarely between start and goal"（障碍正卡在起点↔目标直线上，EE 贴障碍面停滞）——破解 = 障碍中心沿**垂直于路径方向**偏置 `P_OBS=p_start+0.13·DIR+0.04·PERP`（`PERP=DIR×ẑ`），让 EE 从一侧 **skirt** 绕过；② 不可达根因=goal 太远或落入障碍阴影——破解 = goal 放**障碍阴影之外且靠近** `P_GOAL=p_start+0.25·DIR`（off the shadow, kept close = 可达无奇异）。**"死锁/不可达怎么破"比"死锁存在"更可迁移**——是反应式避障能落地复现的工程细节。

### 4.4 冗余/零空间章（高价值对照）
- **降维主任务造冗余**（6D→3D EE 位置留 3 维零空间）+ 运动学 resolved-rate `q̇=J⁺(ẋ_des+K·e)+N·q̇_sec, N=I−J⁺J` → 零空间章主线。
- **运动学级 vs 力矩级（OSC Khatib `N=I−Jᵀ(JM⁻¹Jᵀ)⁻¹JM⁻¹`）的对照**：前者无 M⁻¹ → EE 锁 0.00mm；后者受 cond(M) 限制 → EE 漂 60-110mm。**结论：异质惯量臂冗余解析正解=运动学/速度级**——是冗余章最深的工程洞察，强烈建议入书（含"正则破坏投影性、特征值不再 {0,1}"的机理）。

### 4.5 力控部（项目核心，最厚重）
- **阻抗/导纳/混合三分类表** + **力矩臂→真阻抗（测位置出力矩，无需 F/T）** 范式 → 力控章总论。
- **Cartesian 阻抗** `τ=Jᵀ(−K_cΔx−D_cẋ)+g`，临界阻尼 `D=2√K`；HOLD 平衡处弹簧项=0=纯重力补偿 → 阻抗控制小节。
- **扰动柔顺的线性弹簧律实证**：50N→0.2m=F/K 精确 → 阻抗刚度物理验证的教学范例。
- **计算力矩反馈线性化** `τ=rnea(q,q̇,q̈_d+Kp·e+Kd·ė)`，闭环 `ë+Kd·ė+Kp·e=0`；CT vs PD+g 紧 26×（滞后正是 M·q̈+C·q̇ 前馈）→ 计算力矩章 + 对比数据。
- **混合力/位控选择矩阵** S=diag(1,0,0)，法向力控+切向位控，接触摩擦致切向滞后 → 混合控制/打磨小节。

### 4.6 工程实践/调试方法论（可做附录或穿插各章 box）
- **6 条元教训**：① 数据驱动诊断非盲调（"多花一次验数据省十次盲调"）；② 异质惯量臂解耦低惯量腕；③ 冗余解析的层次选择（第一性原理判可行性）；④ 力矩感知时序走逆动力学；⑤ "全栈"=证明层间交接而非各跑一遍；⑥ 诚实记录局限是工程素养。
- **隔离变量法定位**（M4 无墙基线锁定大墙挡路）、**red herring 警惕**（joint_states 名序错乱）、**第三方 plugin 加载/命名/字段约定**（external_wrench 三连坑）、**rclpy/进程生命周期坑**（回调内 shutdown 不停 spin、effort 保持力矩、pkill 自杀）。
- **MuJoCo 数值稳定三要素**（惯量量级 + 阻尼 + actuator 刚度匹配，否则显式积分爆）、**无名 material 静默丢弃**、**install 拷贝非软链** → 仿真/建模工程注记。
- **MJCF 转换器两个集成坑**（`inject_actuators.py`）：① 转换器在 geom 上写了 `class=collision/visual` 却不生成对应 `<default>` 定义 → MuJoCo 报 `unknown default class name collision` → 须手补 `<default>` 的 visual(`contype=0` render-only 不碰)/collision(可碰)两类；② 高瘦立式臂的默认自由相机会把臂裁切/偏心 → 须注入 `<statistic center/extent>` + `<visual global azimuth/elevation>`（center 取臂中、extent>auto 拉远）才完整入镜。
- **力矩跟踪故障特征分类法**（诊断可视化 `m3_ct_diag_plot.py`/`m4_diag.py` 内建）：把时间序列波形按四种典型签名归类——**stuck-at-home / wrong-sign-runaway / unstable / lagging**，看到哪种波形→对应哪类根因，是可复用的力矩调试诊断清单。

---

## 附 · 源文件索引（均在 `/home/ziren2/pengfei/Robot/arm_control/`，只读）
- 文档：`docs/00_index.md`（范式+里程碑入口）、`01_trajectory_optimization.md`、`02_obstacle_avoidance.md`、`03_force_control.md`、`04_fullstack_capstone.md`、`05_过程复盘与教训.md`（叙事级复盘）、`debug_log.json`（A-001..026 hypothesis→action→result→verdict）。
- 模型清洗：`scripts/clean_urdf.py`、`inject_actuators.py`、`make_torque_mjcf.py`、`urdf_to_mjcf.sh`、`m3_link6_inertia.py`/`set_link6_inertia.py`（惯量条件数分析）。
- M1：`m1_toppra.py`、`m1_ruckig.py`、`m1_spline_opt.py`、`m1_torque_toppra.py`、`m1_chomp_vs_ompl.py`、`m1_exec_traj.py`。
- M2：`m2_obstacle_avoid.py`、`m2_cbf.py`、`m2_nullspace_kin.py`、`m2_exec_traj.py`、`inject_sphere.py`。
- M3：`m3_gravity_comp.py`、`m3_impedance.py`、`m3_computed_torque.py`、`m3_hybrid.py`、`m3_nullspace.py`、`inject_contact_surface.py`。
- M4：`m4_plan_offline.py`、`m4_capstone.py`、`m4_prep_mjcf.py`、`m4_diag.py`、`m4_fullstack_demo.sh`。
- 包：`src/arm_dm_description`（URDF/xacro/MJCF/meshes）、`src/arm_dm_bringup`（controllers.yaml/mujoco_plugins.yaml/launch）、`src/arm_dm_moveit_config`（SRDF/kinematics/joint_limits/chomp_planning.yaml）。
