# Robot 工程吸收 · legged_perceptive 四足感知规划（dmgo 移植 PHASE3）

> 来源项目：`/home/ziren2/pengfei/Robot/legged_perceptive`（只读吸收，未改动任何源文件）。
> 本文 = 对该项目 37 篇 `docs/dmgo_port/*.md`（~4135 行）+ 核心 C++ 源码的凝练。
> 主题 = 把自定义四足 **dmgo** 移植进 `qiayuanl/legged_perceptive`（OCS2 感知 NMPC + WBC + Gazebo），对标 a1/aliengo，目标「a1 能过的地形 dmgo 都过、≥95%」。
> **价值核心 = 一套证伪驱动的调研方法论 + 一长串血泪根因/教训**（错的转弯比对的结论更有教学价值）。文中标注的 `P3-NNN` 是项目 `debug_log.json` 的过程编号，可回溯。
> ⚠ 传输完整性：本项目 3 个 catkin 包（interface/controllers/description）源码 + 37 篇 docs + 4 个 demo mp4 已齐全，无明显缺失；上游依赖（grid_map/elevation_mapping/convex_plane_decomposition 等）不在本仓库内（README 列依赖）。

---

## 第一部分 · 项目概览（目标 / 技术栈 / 架构）

### 1.1 一句话定位
**legged_perceptive = legged_control 的「感知叠加层」**。它**不替换** legged_control 的 WBC / Gazebo / ros_control / KF 估计运行环，而是**继承并扩展** NMPC 的 interface 与 controller，往 MPC 里加**地形感知约束**（落足点必须落在可行凸区域、足不撞地形 SDF），使机器人能在**非平地（楼梯/台阶/缓坡/碎石/间隙）**上规划落足。

- 同作者 Qiayuan Liao，同 OCS2 质心动力学 NMPC + WBC(qpOASES) 栈。
- 论文血缘：legged_control = Grandia et al. *Perceptive Locomotion through NMPC*（arXiv:2208.08373，IEEE T-RO 2023）的**平地非感知子集**；legged_perceptive = 该论文的**完整感知版**。
- ⚠ README 顶部声明：**作者已停止维护此项目**（转新框架）→ 以本地源码为准，遇坑自解，不指望上游修。

### 1.2 三个包 + 类继承
| 包 | 作用 | 核心类（继承自 base） |
|---|---|---|
| `legged_perceptive_interface` | 感知 NMPC 问题构建 | `PerceptiveLeggedInterface : LeggedInterface` |
| `legged_perceptive_controllers` | 感知控制器 + 收地形 | `PerceptiveController : LeggedController` |
| `legged_perceptive_description` | 给机器人挂传感器（深度相机/lidar），纯 xacro 无代码 | — |

**关键**：controllers depend `legged_controllers`，interface depend `legged_interface` → **WBC（QP）、Gazebo 层、KF 估计、ros_control 全部复用 legged_control，零改动**。感知只在 NMPC 的 cost/constraint 层叠加 + 换 ReferenceManager。

### 1.3 感知数据流（运行时）
```
深度相机(d435)/livox lidar 点云
  → body_filter → elevation_mapping(ANYbotics，机体跟随的 2.5D 高程图 grid_map)
  → convex_plane_decomposition_ros(高程图分割成「凸平面区域」=可落足地块 + 生成 smooth_planar 平滑层)
  → /planar_terrain (convex_plane_decomposition_msgs)
  → PlanarTerrainReceiver (controllers 的 SynchronizedModule，注入 interface 的 planarTerrainPtr_/sdfPtr_，重建 SDF)
  → ConvexRegionSelector(选落脚点凸区域+投影高度) + PerceptiveLeggedReferenceManager(抬 base-z+算 base-pitch)
  → NMPC(FootPlacement + FootCollision 约束) → SQP/HPIPM 求最优 state/input(质心动力学)
  → WeightedWbc(qpOASES) 解成关节力矩 → HybridJoint: τ = kp·(q*-q) + kd·(q̇*-q̇) + τ_ff
```
- `SynchronizedModule` 机制 = OCS2 在每个 MPC 周期前线程安全地把外部地形喂进求解器。
- 两层频率不同：MPC ~100Hz（质心模型，凸性好但不直接出力矩），WBC ~500Hz-1kHz（全刚体，把期望翻译成可执行力矩 + 处理接触/限幅）。

### 1.4 启动机制
`ROBOT_TYPE` env 选机器人。**taskFile = perceptive 专属**（每机器人一份，含感知约束/权重）；**referenceFile / gaitFile 零改动复用 `legged_controllers/config/$ROBOT/`**（flat 阶段产物）。gait 经 `legged_robot_gait_command`（keyboard UDP 节点）下发。

---

## 第二部分 · 核心方法与算法

> 一句话心智模型：**感知 NMPC = 「启发式选带高度的落脚点(where)」→「质心动力学 NMPC 求力/轨迹(how)」→「WBC 把力/轨迹解成关节力矩」**。地形只通过两条途径进入：①`ConvexRegionSelector` 把落脚点投影到凸区域（落在哪）；②`ReferenceManager::modifyReferences` 用地形高/梯度抬 base-z + 给 base-pitch（身体怎么抬）。**没有显式 ZMP / 支撑多边形 / roll 约束**，稳定全靠落脚点 + 摩擦锥 + 地形法向 pitch 隐式保证。

### 2.1 感知 NMPC 加了什么（`PerceptiveLeggedInterface.cpp`，逐行实证）
`setupOptimalControlProblem()`：
1. 先建默认 5×5m 平地 `PlanarTerrain`（占位，运行时由 PlanarTerrainReceiver 替换）+ 对应 `SignedDistanceField`（层 `elevation_before_postprocess`，分辨率 0.03，SDF 范围 ±0.1）。
2. **调 base `LeggedInterface::setupOptimalControlProblem()`** —— 即 flat 的全部（质心动力学、摩擦锥、支撑零速、摆动 z、自碰撞…）原样保留。
3. **对每只脚**加两条软约束（`RelaxedBarrierPenalty`）：
   - **`FootPlacementConstraint`**（落足点必须在所选凸区域内多边形，`numVertices_` 顶点）—— 感知核心，强制脚落在可行地块。
   - **`FootCollisionConstraint`**（脚 vs 地形 SDF，间隙 `clearance=0.03`；只在脚摆动时 active，前后 ±0.05s 也判触地）—— 防脚插进台阶立面。
4. **体碰撞**：4 根小腿 `{LF_calf, RF_calf, LH_calf, RH_calf}` 贴球（半径余量 calf 0.02），构 `SphereSdfConstraint`（球 vs 地形 SDF）。⚠ **源码里默认被注释禁用（L87-89）** → 开箱只有「足落点 + 足碰撞」两类感知约束生效，体-SDF 是死代码（小腿能蹭台阶棱无约束）。
- **移植极干净的原因**：感知层代码**几乎机器人无关**，唯一硬编码 = 小腿 link 名 `*_calf`（dmgo 的 go1 式 `${prefix}_calf` 宏正好匹配，无需改源码）。

### 2.2 落脚点选择 `ConvexRegionSelector`（最易踩坑的环）
- **名义落脚点 `getNominalFoothold`**：`foothold = FK(desiredState)[leg] - R^T·offsetVector`。落点来自**期望状态的足位**（按当前参考轨迹这只脚该在哪）。`offsetVector = tan(-pitch)·0.4` → **base-pitch 通过这里把落点往坡上推**（pitch↑ → 落点前移上台阶，爬升机制的一环）。⚠ capture-point 反馈 `√(h/9.81)·(v_meas−v_des)` 在源码里**被注释掉**，实际没用。**名义落点 z 来自期望 base 高 → 平地假设下 z≈0，不知道台阶。**
- **投影到「最优」凸区域 `getBestPlanarRegionAtPositionInWorld`**（在 `convex_plane_decomposition/SegmentedPlaneProjection.cpp`）：`cost = distanceCost(含 dz²) + penaltyFunction`；其中 `penaltyFunction` 在 `ConvexRegionSelector.cpp:81` = `return 0.0`（**no-op**）→ 纯按 3D 距离选 = **唯一可偏置区选择的钩子目前没用**。

### 2.3 参考生成 `PerceptiveLeggedReferenceManager::modifyReferences`（爬升核心驱动 + 调试重灾区）
每 MPC cycle 重算 11 个 horizon 节点的 base 参考。源码实证三大通道：
- **base-PITCH（爬升核心）**：在 footprint 四角（±hx=0.30, ±hy=0.20）采样 `smooth_planar` 做最小二乘平面拟合 → 算前后坡度 `dzdx` → `pitch = atan(dzdx)`，**clamp ±0.12rad(~7°)**。
  - **why 4 足拟合而非 2 点中心差分**：前样本落在台阶上 → base 还没过沿就有平滑非零 pitch → 通过 `tan(pitch)·0.4` 把落点前推 → 破鸡生蛋。2 点中心差分=不连续→曾把 dmgo 掀翻（P3-102）。
  - **clamp 0.12 的血泪边界**：clamp 放到 0.25 让 dmgo 抬头 17° 后倒翻（P3-121）—— pitch 权限不是越大越好。
- **base-ROLL = 强制 0**（默认）。直台阶无真实 roll，命令任何 roll = 纯误差注入；地形耦合 roll 在漂移 base 上重采样会变正反馈 veer 环（见 3.x）。（坡上 banking 修复见 2.5。）
- **base-Z（heave，最高权 Q(8,8)=1000）**：`zBase = smooth_planar(base 单点); base_z = zBase + comHeight / max(cos(pitch),0.5)`。`/cos(pitch)` 抬身体使脚不被拽进立面。canonical 单点（a1/go1/anymal_c 都单点）。⚠ 曾把 base-z 改成「4 角 footprint-mean」并丢了 `/cos(pitch)` 升高修正（兄弟版本都有）→ 身体抬头时 base-z 命令偏低、前腿被拉进竖板 → 自注入扰动（P3-126 已修回单点+/cos）。
- **落足/摆动高度**：`updateSwingTrajectoryPlanner` 用 `ConvexRegionSelector` 投影点的 z 作 liftOff/touchDown 高度喂 `SwingTrajectoryPlanner`（swing apex = `max(liftoff,touchdown)+swingHeight`）。
  - ⚠ **`max(...)` 是 qiayuanl 对 vanilla OCS2（用 `min()`）的关键改动**：台阶 swing 顶 = `max(0,0.16)+0.08 = 0.24` 自动抬过台阶 → 这正是「swingHeight 0.08 就够、不需 0.15」成立的原因（a1 同 0.08 爬同样 0.16）。误以为「摆高必须 ≥riser」是错前提（蹭立面真因是 dz² 选地面区 + positionErrorGain=0，见 3.4）。
- **⚠ framework 级误配置 `positionErrorGain=0`（摆腿 z 开环）**：`task.info model_settings positionErrorGain=0.0`，而 OCS2/anymal 上游默认=20.0；**a1/aliengo/dmgo 港口三机全继承这个 0**（实测 live task.info 仍为 0.0，即便一度改 20 也已回退/仅 env-gated 构建生效）。后果：`LeggedRobotPreComputation` 在 =0 时**跳过摆腿 z 位置反馈块** → 摆腿只受 z **速度**约束、无位置拉回 spline → 撞竖板/漂走无人纠正 → 落点非对称 → roll。task.info 注释明确把它与 dz² 并列为 riser-clip 两面之一（`dz^2 foothold-selection bias + positionErrorGain=0`）。**这是与 dz² 坑同级的真结构缺陷**（上游默认被港口改坏），凝练前文未提。
- **⚠ 设计哲学反差：上游本就倾向「隐藏 base 参考」，terrain-pitch 是 qiayuanl 的非规范简化**。文献一致：重机器人靠**落脚点+摆动/膝避障**爬台阶，base 参考软/平/或**完全隐藏**，绝非刚性 terrain-pitch —— Grandia 2208.08373 base ref=4-髋平面拟合+additive（orient 权重温和，「无楼梯专用参数」）；Corbères 2305.08926（ANYmal 40cm 台阶）**移除 base/CoM 跟踪约束**；DTC 2309.15462（Science Robotics）**刻意对控制器隐藏 base-pose 参考**（降对地图误差敏感），纯落脚点+摆动反射爬 0.18m 楼梯/0.48m 箱（ANYmal C 50kg 同算法爬更大台阶）。→ **坐实 qiayuanl 港口的 `base-z = smooth_planar + comHeight/cos(pitch)` 这套强 base-z 参考是非规范简化**，正解释了为何 dmgo base-z 参考这么爱出事（over-launch 触发层）——它本就该被弱化/隐藏（参见 §3.3 的 reference-free 方向与 §4 的剩余路径）。

### 2.4 WBC（`WeightedWbc`/`WbcBase`，力矩怎么来）
把 NMPC 的最优 state/input 翻译成关节力矩。任务表（Task = {A,b,D,f}）：
| 任务 | 类型 | 作用 |
|---|---|---|
| FloatingBaseEom | 等式 | 全刚体动力学 M·a+h=Sᵀτ+Jᵀf |
| TorqueLimits | 不等式 | \|τ\|≤limit |
| FrictionCone | 不等式 | 接触力在摩擦锥内（金字塔近似） |
| NoContactMotion | **硬等式** | 支撑脚加速度=0（完美 stiction） |
| BaseAccel | 等式 | base 6-DOF 加速度跟 NMPC 质心动量率 |
| SwingLeg | 等式（权 100，远高于 base orientation 1） | 摆动脚跟轨迹 |

- **⚠ 权重进 H=AᵀA 是平方**：`WeightedWbc` 把任务按 √weight 堆进 stacked A 再解 `min‖Ax−b‖²` → 有效权重比是**平方**。`swingLeg:baseAccel = 100²:1 = 10000:1` → 摆腿任务**碾压** base-roll 恢复（这才是「SwingLeg 权 100 高于 base orientation 1」的真实量级，不是 100:1）。备选路径 `HierarchicalWbc`（HoQp，**已实现但未用**，`LeggedController.cpp:69`）可把 baseAccel/swingLeg 设**同级** > contactForce，更干净；Grandia 反对强硬优先级（不等式 active 时会饿死低优先任务）—— 故 qiayuanl 用软加权而非分层。
- **⚠ `formulateBaseAccelTask` 的 jointAccel 是 1-tick 有限差分**（`WbcBase.cpp:195`）：`jointAccel = getJointVelocities(inputDesired − inputLast_)/period`，再 `centroidalMomentumRate −= Aj·jointAccel`（Aj=质心动量矩阵的关节列）。触地/抬腿瞬间 MPC 关节速度参考跳变 → jointAccel 尖峰；dmgo 重远端腿（calf 0.240 vs a1 0.151）→ Aj 大 → 伪 base-accel 目标大 → WBC 追它 → base 竖直扰动（最贴合「竖直过冲」数据）。**这是 over-launch 证伪链里唯一从未实测的强假设**（需改代码 env-gated 钳制/置零重编，session-11 始终没做），典型「共享码但 dmgo 重远端腿特有放大」类钩子。
- **底层下发**（`LeggedController.cpp:135`）：`setCommand(posDes, velDes, kp=0, kd=3, ff=torque)` = **纯力矩 + 小速度阻尼**。所有机器人共享此值 → 「调底层 PD」对 dmgo vs a1 无差异。

### 2.5 侧向轴控制：闭环航向锁（早期窄楼梯）+ 坡上 banking（Branch B）
> 侧向 veer/漂移是一条贯穿始终的轴，下面两条是它早期与后期的修复（同一根问题）。
- **早期：闭环航向锁（窄楼梯侧偏冲出边沿）**。缓+慢（rise0.04 vx0.08）真爬上 ~3 级（z 0.356→0.525），但 y 漂 −0.52→−1.26 **侧偏出 1.5m 宽楼梯边摔下**。根因=开环 cmd_vel **无绝对航向锁**（flat 上的 veer 老问题在窄楼梯被放大）。治法=cmd_vel 发布器订 `/odom` 取 yaw，`wz = clamp(−1.5·yaw, ±0.3)` 驱 yaw→0 直行上楼（与后期 Branch A banking 是同一侧向轴问题的早期显现）。
- **后期坡上 banking（Branch B 修复，恢复上游 terrain-roll）**：港口已算**真平面** `z = a·dx + b·dy + c`（最小二乘拟合 4 个 planned footholds）。`pitch = atan(a·cosyaw + b·sinyaw)`（前后梯度）；**新增 `roll = atan(−a·sinyaw + b·cosyaw)`（侧向梯度）= banking**。
- **why 安全**：源 = planned footholds（非漂移 base 重采样）→ 非正反馈环；平地 a=b=0 → roll=0（零回归）；坡上 yaw 偏 → 侧向梯度≠0 → 身体倾向支撑侧抵消侧滑。env `ZPLANE_ROLL` gated。

> ⚠ **cmd_vel 过速是 legged_control 固有（非 dmgo bug）**：dmgo cmd0.30→实速 ~0.57（~2×），一度疑为 dmgo 问题瞎调权重（vcom 15→60 → MPC 过约束 → 更快+摔=**错方向**）；用户指引「看 go1 配置」→ diff 出 Q/R 权重**完全相同** → 测 go1 基线 cmd0.30→实速 0.58 **完全一致** → 过速是**位置追逐固有**（base Q 1000 ≫ vcom 15，目标 1s 前方），`cmd_vel = 意图非精确速度`，**不修**。这是「先比 go1 基线再判 dmgo bug」的范例。

### 2.6 关键参数定论（dmgo task.info 实证，KEPT 配置）
- `frictionCoefficient = 0.3`（MPC 摩擦锥）= sim Gazebo μ=0.6 的一半 = **2× 安全余量**（见 3.1，最重要的修复）。
- `comHeight = 0.30`；`(8,8) p_base_z = 1000`、`(2,2) vcom_z = 500`（over-launch 修复后同步值，见 3.3）。
- `sqpIteration = 1`（RTI 正解，不是把 N 个 iter 塞进周期）；`recompileLibrariesCppAd false` + `modelFolderCppAd /tmp/legged_control/dmgo_perceptive`（见 3.2 缓存坑）。

---

## 第三部分 · 工程实战与踩坑 / 失败经验（用户最看重，逐条 现象→根因→解决/教训）

> 整个 PHASE3 的 90% 精力耗在**爬单级台阶**（0.16 是 benchmark）这一个点上：dmgo 翻车，a1 不翻。移植本身极干净（站立+平地 trot 直走很快达成），硬骨头全在这里。下面按主题分类。

### 3.1 ✅ friction-cone μ 失配（最重要的真修，跨验证）
- **现象**：dmgo 上台阶处 ±90Nm 40ms 翻号 chatter、随机方向、roll 发散翻车；QP 干净（qpErr=0，非 solver 崩）；a1/aliengo 同代码能过。
  - **⚠ ±90 chatter 的真相澄清**：这个 ±90Nm **不是 WBC**（WBC torqueLimitsTask 内部封顶 55），而是**底层 ros_control PD 在 URDF effort=90 处 clamp 饱和**；且它发生在**翻倒最后 0.3s = 后果非前驱**（脚滑→PD 救→饱和→再滑的 stick-slip 极限环），别误读成「力矩饱和/欠驱动」是病因。`NoContactMotion` 硬约束键于 gait 时钟而非实测接触（脚没真踩时仍硬约束加速度=0）也喂这个极限环。
- **定位方法**：连测惯量/torque/CoM 全证伪后，按「反盲调铁律」停单变量旋钮，换**系统性「MPC 模型假设 vs Gazebo 仿真现实」一致性审计**（clean-QP 但发散的 bug 必住在失配层）。审计第 4 轴命中。
- **根因**：Gazebo 脚 `mu1=mu2=0.6`（所有机器人相同）。a1/aliengo 的 MPC `frictionCoefficient=0.3`（=sim 的一半，**故意 2× 安全余量**）；**dmgo perceptive config 误设 0.6（=sim，零余量）**。MPC 按 μ=0.6 规划到摩擦锥边缘 → crest 大剪切力时脚实际 0.6 滑限被触碰 → 脚滑 → 丢接触 → roll 发散翻。完美解释每条线索（clean QP / 有效命令发散 / chatter / dmgo 特有 / 方向随机）。
- **解决**：`task.info` 两处 `0.6→0.3`（frictionConeSoftConstraint + frictionConeTask）。**两个能爬的机器人都 0.3 = 跨验证。**
- **教训（可复用）**：**移植新机器人到这类框架，必查 controller frictionCoefficient 与 sim 接触 μ 的比值，保持上游安全余量（通常 ½）。** 这是隐蔽的「有效命令动态发散」源（QP 干净但仿真里脚滑）。

### 3.2 ✅ CppAD 自动微分缓存陈旧（经典静默坑）
- **现象**：改了 dmgo 的 urdf（左右对称重建），但 MPC 行为不变，疑似「用旧模型规划」。
- **根因（OCS2 源码级证实）**：`recompileLibrariesCppAd=false` 下，`CppAdInterface::isLibraryAvailable()` **只检查 `.so` 文件是否存在，不校验内容/时间戳/hash**。库按 `modelName` 字符串定名（非内容 hash）→ 改模型但 modelName/folder 不变 → 同路径 → **静默加载旧 `.so`** → MPC 用旧 dynamics/kinematics 导数。（DeepWiki 原文确认；qiayuanl#27 维护者答「靠 boot 脚本手动失效」=无内置机制。）
- **解决/规范**：**任何修改 urdf/xacro 中影响动力学的量（mass/inertia/com/link 几何/关节轴）后，必须 `rm -rf <modelFolderCppAd>` 强制重编**，或临时 `recompileLibrariesCppAd=true`。更稳：folder 按模型版本/hash 命名。⚠ 关节阻尼/摩擦等纯 Gazebo 仿真参数**不进 CppAD**，改它们无需重编；只改 reference manager / cost 权重也无需删缓存（只改 URDF 才必删）。
- **⚠ 诚实修正（贵教训）**：删缓存重编后**首次 climb 成功**，一度判定「陈旧缓存=弹射根因」；但 N=6 复测 0/6（那 1 次是噪声）→ **缓存陈旧是真坑但不是弹射根因（红鲱鱼）**。→ 「别对单次随机成功过度解读」（见 3.7）。

### 3.3 ⭐⭐⭐ over-launch（弹射）根因侦破全弧 —— 项目最有价值的证伪链
**现象**：dmgo 上 0.16 台阶时身体竖直冲到 z=0.48–0.68（从 0.30）+ 前冲 + 随机方向 + 翻 roll±180；QP 干净（物理自洽，是 MPC 给的力）；方向随机（对称前进指令却随机爆 = 关键线索）。

**逐个被提出又被实验证伪的根因（这串证伪是核心学习材料）**：
| # | 假设 | 怎么测 | 结果 |
|---|---|---|---|
| 1 | 感知覆盖/落脚点冻结/TF | 时序探针 | ❌ 证伪（map 出台阶、落脚前进；崩/冻是翻的 downstream） |
| 2 | swing 高度/速度激进 | 查配置/扫参 | ❌ dmgo swing **已 a1-parity**（0.05/-0.1/0.08）；激进结论来自已证伪的注入 harness |
| 3 | **2.27× trunk roll 惯量** | 换 a1 惯量 / EXTREME mass 1.5kg | ❌❌ **REFUTED — 更轻翻更狠（弹更高）**！惯量是 amplifier 不是 gate；F=ma，质量越小同 MPC 竖直力弹越高 |
| 4 | 静态 CoM 高 / 支撑窄 | FK 第一性原理复算 | ❌ 证伪（dmgo 静态 roll 裕度 26.8° = 2×a1 13.7°，CoM 更低、足更宽）|
| 5 | 关节阻尼/摩擦失配 | →0/0 对齐 a1 | ❌ 对 step over-launch 仍翻；但**对平地 trot 基础失稳是真修**（同旋钮两面，见 3.3b）|
| 6 | CppAD 缓存陈旧 | 删缓存重编 | ⚠ 真坑但非弹射根因（红鲱鱼）|
| 7 | WBC 用 gait-clock 非实测接触 | AND-gate measured contact | ⚠ canonical 缺失属实但补丁非唯一触发，raw gate 8/8 更糟 |
| 8 | MPC 力权 Q(10,10) 300→650 | 扫权重 | ❌（Q650 仍侧翻）|
| 9 | **base-z/vcom_z 参考不一致** | vcom_z 全变体（CLIMB_VEL/SUPPORT/PERSIST/PLANE/STRUCT_EXTRAP）| ⚠ 移动 cliff 但封顶（见下）|
| 10 | base-ori 软任务（飞行中无效）| scalar/roll-only kp 扫 | ❌（飞行相角动量守恒，姿态只能 stance 改）|
| 11 | p_base_z 力权软化 | 1500/1000/800 | 0.08 甜点但 0.10 irrelevant，maxz0.46 不降 |
| 12 | 非飞行步态 static_walk/amble | 切步态 | ❌（over-launch 是 force-driven 非 gait-scheduled，与 2vs3 脚无关）|
| 13 | 速度/动量调度（提速给动量）| VX 0.10/0.18/0.26 扫 + terrain_guard | ❌（speed not a lever；dmgo 是**过驱动**不是欠驱动，提速火上浇油；vx→0 全停仍 over-launch，因预抬由 footholds/lookahead 锚定，velocity-independent）|

> **⚠ 表行 10 的反直觉精确根因「升 roll 权重是错方向」（P3-082，最易踩的调权陷阱）**：直觉会去「加大 roll 姿态权重」帮重机器人刹横滚，**方向恰恰反了**。OCS2 的 MPC 代价是**质量+惯量归一化**的 `Q(3,3)·(I_xx·ω_x/m)²`（状态 x[3:6] 是归一化质心动量 L=I·ω）。dmgo 的 2.27× roll 惯量进 `(I_xx·ω/m)²` 已让同权重 `Q(3,3)` 对 roll 校正力矩**自动放大 2.27×**（罚 L² 已是 a1 的 ~5×）→ 要把阻尼**匹配到 a1 水平**，`Q(3,3)` 应**降到 ~×0.44 而非升 ×4**；硬升 `theta_base_x` 到 600 = 高 P 增益**极限环风险**（=双重计数：代价已归一化，再手动放大）。交叉佐证：aliengo @3.2× a1 roll 惯量（比 dmgo 还大）反而用 `L_x=10/theta_x=300` **不变**，裕度来自 `R=5+慢 gait[0,0.35,0.70]+comHeight0.40+friction0.3` **作为一整套**而非靠堆 roll 权重。教训可复用到任何 OCS2/质心 MPC 调权重场景：**代价已惯量归一化时，重机器人的「同权重」其实已等效更强阻尼，本能加权 = 双重计数 → 失稳**。

**实测确认弹射本身**：仪表化 `[CDBG]` 打印每 tick planned vs measured 接触：台阶处 `plan=0110 meas=0000` = **MPC 计划 2 脚支撑但四脚全离地**，roll 发散 12–54 rad/s（vs 静态翻倒阈 ~2.5 rad/s = 5–25× = 动态飞行失稳，非准静态翻倒）。

**机制收束（部分修对方向）**：
- **触发层**：base-z 参考**升位置但 vcom_z 留 0**（自相矛盾的参考：Q1500 升 pos-z + Q100 持零竖直速度）→ MPC 用**瞬态竖直力**调和 → 重 dmgo 过执行 = 弹射。canonical OCS2 anymal 一致做 `vcom = d(base_pos)/dt`（ReferenceExtrapolation），qiayuanl 港口**省略了速度半边**。
- **修法演化（matched-look-ahead，cliff-discriminator 验证）**：让 base-z / vcom_z / pitch **三通道全从同一组 planned footholds 导出**（support-plane + pitch-同源）→ 用「崖点测试」（在 0.05-pass/0.06-fail 临界崖测，而非 0.16 饱和区）证明**它能移动 cliff = 它就是 gate**：cliff 0.05→0.06→0.07，**且把 FLIP 模式从根上杀死**（roll 180°→±2°，姿态参考一致 → 飞行相 roll 不发散 → 落地直立）。
- **决定性负结论（参考路线已穷尽）**：把所有「参考一致性」杠杆（support/pitch-同源/persist 跨周期/真平面 z=ax+by+c）叠满后 **maxz ~0.40-0.49 全部 UNCHANGED** → over-launch **不是参考跳变，是 MPC tracking 力峰**（base-z 参考升起时 MPC 过度施竖直 GRF，把重躯干推过参考 4cm）。cost 层（p_base_z 1500→800、vcom_z 100→500）也只微调 0.08 的翻/recover 平衡，不杀 maxz。

**最终诚实定论**：
- **maxz 双峰**（0.31 卡 / 0.46 弹，**永远没有 0.38 干净渐爬**）= dmgo 重躯干在此步态/速度下**只能靠「弹」上台阶，做不出 a1 那样的受控渐爬 = 内禀动力学属性**，非某个权重调错。
  - **⚠ 校准「内禀」不等于「物理不可能」——这是方法的墙不是物理的墙（doc32 §3 正面存在性证据）**：重躯干四足**能**爬大台阶——**HyQ（80kg 液压）爬 24cm ＝ 30% 腿长 / 53% 可伸缩范围**，靠 **centroidal dynamics（角动量+CoM 调度）而非位置跟踪 reference**。即 dmgo 的 over-launch 墙是**「position-tracking base 参考」这套方法的墙**，换 full-centroidal/角动量调度（§4 的 P2-C 重构）理论上可破，**不是物理不可能**。所以「内禀动力学属性」应读作「在本港口这套强 base-z 位置参考的方法约束下内禀」，避免误判为「重机器人物理上爬不了」。
- frontier 失败 = over-launch（4 脚弹离）+ 侧向 veer + 感知 flicker **交织的涌现多模不稳，无干净单根**。~15 轮单杠杆全证伪 = 真硬问题的诚实标志。
- **文献独立交叉验证我们的证伪**（非记忆，深读 arxiv 2605.05707 Go1 MuJoCo 的 Thm 1/5/6/7）：
  - **Thm 5**：N=2 stance(trot) 摩擦锥对角动量率 ‖Ḣ_G‖/m 有几何下界（沿 foot-separation 方向不可消残差）「independent of weight tuning」→ 独立坐实「~15 轮调权重修不了」是结构性的。
  - **Thm 6**：临界水平加速度 `a_x⋆ = μg/(1+2μκ)`，Go1(μ0.6)≈**3.72 m/s²**，超之摩擦锥连几何地板都够不到（非光滑 kink）；κ 仅由 foot 几何定，**dmgo 更重 → 阈值更低 → 更易超**。
  - **Thm 1**：N≥3(full-rank 接触)→ 障碍消失，N=4 实测 ‖Ḣ_G‖/m 比 N=2 降 **~227×** → 真 lever = 爬升时多同时接触(N≥3)，**但只在「没被弹离、留 stance」时兑现**（解释 static_walk 为何仍 REFUTED：它只动了调节层②没动触发层① base-z ramp，触发层不治则脚先被弹离、N≥3 兑不了现）。
  - **Thm 7（关键负结论）**：给 WBC 加「注入角动量/动量参考」**FUTILE** —— 只滑离 pendular manifold 朝任务参考，**不 escape 摩擦锥下界**。→ 砍掉「注入 Ḣ_G 参考」这条曾被设想的 over-launch 分支。
  - arxiv 2501.17351 把 over-launch 命名为**飞行相 CAM 失稳**（flight-phase centroidal angular momentum），是有名文献现象。
  - **诚实标注「唯一真未试的杠杆」**：上面 13 行杠杆全 falsified（含 reference/cost/gait/速度），但 **input loopshaping**（对 contact-force 高频内容的频率惩罚，原版 Grandia 有、qiayuanl 港口缺，港口只有 plain `R(force)=5`）是**唯一真正未试**的 over-launch lever（高工作量 OCS2 augment，研究分支，不确定能否破墙）。复盘/续做时不应误以为「真的一个不剩」。

### 3.3b ⭐⭐ 平地 trot 真根因 + 两个真 config bug（被 over-launch 症状掩盖的「上游」病灶）

> 「90% 精力死磕台阶顶（症状），真根却有一截在平地（基础运动）」—— 用户的「平地也有 bug」提示是对的。下面三条是与 over-launch **不同阶段/不同失败模式**的真 bug，凝练前文（含 §3.3 证伪表）只把它们当 step over-launch 的红鲱鱼一笔带过。

- **⭐ 真根因①：关节阻尼/摩擦 200×/5× go1 = 平地 trot 边缘失稳（已验证修复，平地 4/4）**。dmgo URDF `const.xacro:26-27` `joint_damping=2.0 / joint_friction=1.0` = go1 `0.01/0.2` 的 **200× / 5×**。机理：**OCS2 质心动力学不建模关节摩擦** → MPC 按无摩擦算力矩，但仿真关节有 `2.0×关节速度` 的巨大阻尼力矩 → 实际运动滞后指令 → 控制裕度被**均匀侵蚀** → 平地 trot 5m 就翻（a1 稳）。降到 go1 `0.01/0.2` → 平地 trot **0%→4/4**（roll±2° 直走）。
  - **双面性（用户最看重的「错的转弯」教学点）**：**同一旋钮对两种失败模式一真一假** —— 对**平地基础失稳是真修**（先降到 go1 值修好平地），对**台阶 over-launch 是 REFUTED**（后再试 →0/0 对 step 仍翻，§3.3 表行 5）。这是「先在台阶（症状）死磕、真根在平地（基础运动）」这条元教训的具体载体。教科书级「MPC 模型 vs 仿真现实失配 → 静默裕度侵蚀」失败经验（与 friction-cone μ 失配同一族）。⚠ 这类纯 Gazebo 仿真参数**不进 CppAD**，改它无需删缓存。

- **⭐ 真 config bug②：R 力代价矩阵 L/R 不对称（P3-090，对抗 Agent 审出的隐蔽 bug）**。dmgo `task.info` 足力代价 R 矩阵 12 项本应全 1.0（a1 值），实际整块是 5.0（aliengo 残留，注释**撒谎**写「reverted to 1.0」），且**右后腿(RH) y/z 只改了 3/12 个 cell 留 1.0**（P3-074「半吊子回退」）→ MPC 对 RH 腿侧向/垂直力惩罚比其它 3 腿小 5× → 优化器**偏用 RH 出力** → 构造性 L/R 力不对称 → 侧向/滚转偏置 = 正好是「y 随机漂、台阶顶翻」失效模式。**污染了 P3-082..089 所有台阶实验（结论作废）**。修复=全 12 项→1.0。教训=「**半吊子回退留隐藏不对称、逐变量纪律被违反**」，且这是**对抗 Agent 独立审 config 抓出的「我自己 90 个 phase 没发现的 bug」**。（注：§3.9「L/R 位精确镜像、无横向不对称 bug」指的是 URDF *模型* 对称，与这个 *代价矩阵* 的不对称 bug 不矛盾。）

- **⭐ 真 config bug③：HAA 力矩限被误设 20Nm（太弱刹不住横滚）**。dmgo `task.info torqueLimitsTask` 的 HAA 原为 20.0，注释自承「`=#1 lateral-topple cause: too weak to arrest sideways roll at a step crest`」；实际电机三关节同型号（额定 30/峰值 90+）。统一改到 55（配 HFE/KFE；URDF effort 同步抬到 90）。HAA 是横滚刹车关节，设太弱直接掉 roll 权限 = 又一条「参数误设致翻」真根因（§3.3 把 torque 当「55→90 扫描的被证伪杠杆」，漏了「原本错设 20→修到 55」这个修复本身）。

- **元层诊断：N=4 统计无功效 = 11-session「自信宣布根因→下 session 证伪」循环的统计学根源**。Agent-B 功效分析：区分 30%↔10% 通过率需 **~40-85 trial**，N=4 功效是**个位数** → 本项目所有 N≤8 结论（含 μ0.6「1/4 首过」、各「0/4 证伪」、pitch=0「0/4」实为方差→单跑反而干净爬）**统计上全部无效**，是反复翻车循环的真正根因。§3.7 第 9 条只说「N≥10，N≤5 太吵」，这里给出更尖锐的量化锚：**N≤4 区分两个相近 pass-rate 需 40-85 trial = 单数字功效 = 整个调研循环漂移的统计根源**，并据此推翻了一整批早期结论。

### 3.4 ⭐ foothold-z 架构缺陷（dz² 坑，多 Agent 收束的最佳诊断）
- **现象**：dmgo 走到台阶前卡住（x~1.40，台阶前 0.1m），z 不升（前脚上不了台阶），pitch 抬试爬 → 横漂翻。a1 同代码 `on_step=5-7`（脚上台阶 z0.18），dmgo `on_step=0`（脚留地面）。
- **根因（两面）**：名义落点 z≈0（平地 Raibert，不知台阶）+ 投影区选择的 `penaltyFunction` 是 no-op → 纯按 3D 距离（含 dz²）选区：
  - 名义落点在台阶沿、z≈0 时：地面区 dz²=0（赢）vs 台阶顶区 dz²=0.16²=0.0256（输）→ **偏选地面区 → 脚留地面**。
  - 更坏：base 摇晃 → 赢家 step↔ground **逐周期闪烁** → 落点不一致 → 对角 trot 撑不住 → 横滑翻。
  - swing-apex 饥饿：touchdown z≈0 → apex `0+0.08=0.08 < 0.16` → 脚**撞 riser 立面**。
  - **鸡生蛋**：脚上台阶要 base 先抬，base 抬要脚先上。
- **解决（C+D 独立收敛，diff 过已证伪的 P3-162）**：把 no-op penalty 换成**对「区平面低于脚下局部地形高」的惩罚** `penalty = w·max(0, terrainZ − p.z())²`（w=1000）→ 台阶顶区 firmly 赢、消闪烁、apex 修对（投影 z=0.16→apex 0.24 清 riser）。**关键不踩 P3-162**（直接设 foothold.z=0.16 暴冲翻）：penalty 留 footPos.z≈0，**base 仍先抬、落脚跟上**（GOAL 意图的顺序）；`smooth_planar` 是平滑层 → terrainZ 跨 riser 渐变无跳变。
- **辅助修**：`frontReach`（前脚名义 xy 朝航向前移 ~0.14，因 dmgo 默认站姿前脚欠伸 x1.56 vs a1 x1.70）→ on_step 0→8-14。⚠ 硬编码共享码，终版须 config-driven 每机器人。
- **残留**：flicker 修好后暴露更深的 **base/foot 协调失败**（base 单点采样滞后于已 commit 的高落脚 → catapult），与 over-launch 是同一回事（见 3.3）。

### 3.5 ✅ KF base-z 欠读（terrain-aware 修，本项目最干净的真 win）
- **现象**：机器人 spawn 在抬高平台（如 downslope 起始平台 H=0.10）→ stood z 偏高 0.117 → walk-start trot 即冲出平台边 eject。a1 同 eject（harness 级，非能力）。
- **根因定位（逐环 instrument + 读 KF 源码）**：先证伪「smooth_planar 放大 1.57×」假设（`[ZBASE_DBG]` 实测 reference 完全正确：smoothPlanar@base=0.20、baseZref≈0.50 理想）→ 矛头转 **state estimator**。读 `LinearKalmanFilter.cpp`：`feetHeights_` 初始化为 0 + self-referential 更新（脚 z 自指），**无地形锚点 = 假设 contact 脚在 z=0（平地）**。抬高平台上脚在 H，KF 欠读 base-z ≈ H → WBC 以为 base 太低 → 推 base 升 → 站太高 → trot-start 过高站姿失稳 eject。
- **量化铁证**：平地 gap(z_gt−z_est)≈0.001（KF 正确）；爬上 0.08 后 gap≈0.081（=恰好台阶高）。**欠读量 = 地形高度。**
- **悖论**：渐进 climb 带着 gap=0.08 照样干净爬完（欠读随高度逐步累积，动态步态容忍）；瞬时 spawn 致命（t=0 瞬间欠读 H → 推升 → eject）。即**欠读在渐进中良性、在瞬时 spawn 中致命，但客观错误**。
- **解决（算法修，非调参）**：terrain-aware KF —— 构造函数订阅 elevation map，把 `feetHeights_[i]` 锚到脚下**地形真高**替代恒 0。env `KF_TERRAIN_AWARE=1` gated。
  - **⚠ 关键：touchdown-latch（非连续更新）**：attempt-1 每 tick 连续更新 → z_est std=0.11 平地剧烈跳 → trot 起步摔（修反成元凶）；attempt-2 改**只在 swing→contact 首接触锁一次、stance 期间 hold** → 平滑每步常量。守门：仅在稳定 stance + map valid + frame valid 时 latch，否则 fallback 原 KF。**2 次原则性收敛（非盲调）。**
- **payoff 确证 + a1-parity**：downslope elevated-spawn gap 0.102→0.001、不 eject、ceiling≥0.40；aliengo（轻身）同场景无修也能过 → **dmgo+修 = aliengo = a1-parity**。即修闭的是 dmgo *专属*缺陷（estimator bug 被重躯干放大），**不触碰 over-launch 墙**（正交，预测正确）。gap 跨越修好 spawn 后仍失败（独立 foothold-planning 问题，a1 也失败）。
- **连续融合 jitter 是预期的（非 bug）**：rigid-contact stance 模型要 foot 世界位置 stance 期恒定，每 tick 喂新噪声 map = 注入时变目标 → z chatter；robot-centric map 用估计 pose 建 → 从 map 修 base-z = **正反馈闭环**振荡。**touchdown-latch 正确打破这个闭环**（与 Bledt MIT Cheetah foothold-history、arXiv 2602.17393 Contact-Anchored Odometry 几乎同设计）。
- **⚠ 生产级鲁棒化还需加的 gate（按价值，现有 `contact+map-valid+isfinite` 只是必要地板）**：① **chi-square/Mahalanobis innovation gate**（拒 stale-map/wrong-cell/slip touchdown，**最高价值**）；② **contact-confidence + debounce**（别在 slip/partial contact 上 latch）；③ **finite map-covariance**（软测量替硬 latch，filter 自拒坏 cell）；④ per-cell variance check + patch median。—— 部署到真硬件前应补这套。

### 3.6 ✅ gait「从未被下达」混淆（ROS transport 静默失败）
- **现象**：所有 clean-map 测试机器人早期漂到 x~0.26 后卡死，永不前进到台阶，误判为「稳定但 stall at step」（多轮误诊）。
- **根因**：`GaitReceiver` 用 **UDP(UDPROS)** 订阅 `legged_robot_mpc_mode_schedule`（roscpp）。自写的 `set_gait.py` 是 **rospy 默认 TCP** publish → 消息**永远到不了 UDP 订阅者**（静默失败，无报错）→ gait 不切 → MPC 全程 STANCE（mode=15）→ 机器人只是站着（x~0.26 是起身前冲非走路）。
- **解决**：keyboard 节点 `legged_robot_gait_command`（roscpp UDP）+ `( printf 'trot\n'; sleep 600 )`（sleep 保 stdin 开免 EOF 崩）+ `LD_LIBRARY_PATH=/usr/local/lib`。
- **教训**：① ROS transport(TCP/UDP) 不匹配 = 静默失败，最难查。② 任何「stall/can't climb」结论前，**先探 `/legged_robot_mpc_observation` 的 `mode` 字段确认机器人在执行目标 gait**（15=全脚触地 STANCE）—— 一眼看穿「gait 没下达」。③ 早记录此坑却没查 memory 就造 set_gait.py = 重复踩坑。**推翻了一批 clean-map「stall/stable」结论。**

### 3.7 ⭐⭐ 方法学元教训（最可迁移，反复救场或被违反）
1. **setup ≠ result**（本项目两次自我误判都源于此）。记忆/总结里的「铁证」必须**回溯原始 `debug_log.json` 的 `result` 字段核对**。doc29 初版误把 P3-150 的 setup（"换惯量测试"）当成 result（实为 REFUTED）→ 差点给用户「改质量分布」的错建议。doc17 把单次 climb 当突破。
2. **别死磕调参**（≥3 次无效 = 算法/模型/结构问题，停手研究）。session-8 死磕 swing/torque/VX/HAA 是整个项目最大弯路，三-strikes 规则由此诞生。**多花一次验证数据，省十次盲调。**
3. **proxy ≠ truth（代理量≠真值）**：collision-box≠真几何（用 mesh 积分）；clean_map≠真感知（用官方 demo）；崩溃≠主因（看完整故障链）；comHeight 配置≠整体 CoM（用 FK 独算）；关节角≠落脚点（跨模型约定不同，FK 到 world 足位才是真值——曾据关节角误判「dmgo 站姿窄」，实测反而宽 1.6×）；planned-stance≠measured；maxz≠over-launch；catkin "succeeded"≠.so 真更新。
4. **查 N 路是否真独立（common-mode 假收敛 / confirmation-laundering）**：「多路印证」必须各路不共用同一假设/代理量，否则=同一错误重复 N 次（≠独立，算 1 票）。session-10「惯量超界」3 路里 2 路共用同一坏 collision-box 基准；A+B 两 Agent「pitch authority」共用同一推理链。**信任收敛前先问：这些源是独立到达，还是共享一个前提？再找能证伪它的最便宜实验。**
   - **具体反面教材：`SOLUTION_s11` 的「3-part config 真解」（comHeight 0.36→0.30 + μ + yaw 权重 100→300）= confirmation-laundering**（已标 SUPERSEDED）。当时两 Agent 都收敛到 comHeight，但**喂的是同一错误前提「task.info 相同」**（一致≠独立）+ N=1 侥幸把 ~17% 通过率误判为真解。yaw 100→300 这个旋钮曾被当真解、后被证伪。「一致收敛」+「单次成功」双陷阱叠加 = 假真解。
5. **证伪优先**：任何「X 是根因/physically impossible/confirmed」结论，**必须先说出能证伪 X 的最便宜实验、先跑它、再写 confirmed**。session-11 砍掉 6 个貌似合理假设全靠此。
6. **直接测量胜过推断**：`gz topic -e /gazebo/.../contacts` 拿真实接触流形一锤定音证伪「足端撞台阶」（实测翻时与台阶接触=0，脚根本没碰台阶）。
7. **EXTREME-BRACKET（区间夹逼证伪）**：怀疑某变量是主因 → 推到物理极端（mass 6.889→1.5kg）。极端仍失败 → 该变量被夹逼证伪（不只「a1 级不行」而是「从 a1 级到极端全不行」）。比单点测试强得多。
8. **CLIFF-DISCRIMINATOR**：别在饱和失败区（0.16，什么都 0/3）测（无法区分「修好 gate」和「削弱 amplifier」）。在临界崖（0.05-pass/0.06-fail）测：杠杆移动 cliff = 它是 gate；cliff 不动 = 顶多 amplifier；只把 FLIP 变 STALL = amplifier 被削弱、gate 未解决。
9. **多试次按 pass-rate（N≥10，N≤5 太吵）**：随机失败下单次结论无效。1 次 climb 可能是噪声；定量必用确定性（RANSAC 关 / clean_map）+ 钉核 worker。
10. **基线先行 + measured≠planned**：dmgo 存疑先验 a1/aliengo（reference）同场景，再判真 bug。a1 head-to-head 两次防误诊（a1@0.16 自己也 0/5 → 重定目标）。系统标称的接触/状态未必是实测的（WBC 用 gait-clock 接触、KF 用平地假设 base-z 都是「标称代替实测」坑）。
11. **广度先于深度**：GOAL 是地形覆盖广度，却在单级台阶高度一个点钻了 90% 精力 = 赢战役输战争。Branch A 广度扫描翻转了叙事。
12. **纯 source-level Agent 研究必须 ④ 交叉验证项目失败史**：Agent 联网研究强（诊断机制+文献），但项目内失败史只有 dblog/memory 有 = Agent 盲区。两次抓错：Agent B 自信推荐「vcom_z 修」（实际已实现+已失败）；deepwiki 看源码现状把 P2-A 速度调度当「新 lever」（漏查项目已用 terrain_guard 试过同机制且证伪）。**build 前必 grep dblog。**

**「纪律即生产力」实证 —— 同一会话 3 连救场**（上面几条纪律不是空话，单会话内各抓一次真错）：① **④交叉验证** 抓 Agent B 的 vcom_z 误判（已实现+已失败）；② **三-strikes archaeology / setup≠result** 抓我自己 doc29 把 P3-150 的 setup（换惯量测试）误读成 result（实为 REFUTED）；③ **N=5 pass-rate** 抓 track-2 的「0.10 单 run 爬上」过读（钉核 N=5 揭示 0/4+ = 单 run 是 ~1/6 rare lucky variance，墙 STANDS）。第③例正是 doc17「别对单次随机成功过度解读」老教训的再次应验。

### 3.8 仿真/工程基建坑（环境噪声但有复用价值）
- **控制器跑在 gzserver 进程内**（gazebo_ros_control）→ ① 控制器崩 = 崩在 `gazebo.log`（非 controller.log，易误判 gazebo 自身 bug）；② `std::cerr` 调试打印落 `gazebo.log`；③ attach gdb 需 `CAP_SYS_PTRACE`（容器默认无）→ **让 gzserver 作 gdb 子进程**（`PTRACE_TRACEME` 不需 cap，patch gazebo_ros/gdbrun 为 -batch + 自动 bt）绕过。
- **崩溃 `std::length_error vector::reserve / free(): invalid next size`**：上游 `grid_map_publisher` 的 `map.move()` 移入新格=NaN 边界（只 test 原地踏步用，长走触发）→ 下游 SDF/convex 在退化数据上巨量/非法 reserve → 堆损坏崩。**治本=自写 `CleanMapPublisher.cpp`**（跟随 base、每格按世界坐标全填充高度、永不 NaN，仿 elevation_mapping 跟随输出）→ 平地直走 11.5m 不崩。**堆崩是退化态副产物（timing 依赖），非主因，别追 OOB 行；修横向漂/弹射才是正路。**
- **僵尸进程墙**：每 trial 漏 gzserver/rosmaster/roslaunch defunct（PID1 不收割），累积数百 defunct → ROS master "Connection refused" → 静默卡死。**比「卡死」更隐蔽的危害 = 主动 deflate pass-rate 制造假摔**：~40 trial/容器 ~230 defunct 时，对照组（如 vcom_z=100）会被假摔到 3/10（真值 ~9/10，P3-239 实遇）→ **污染定量对照结论**（不只卡死，是悄悄压低成功率）。**每 ~40 trial `docker restart` 收割**（收割频率从估的 80 trial **修正为 ~40 trial**；fs/binaries/cache/真模型持久）。这是 timing 敏感定量实验的隐蔽陷阱。
- **容器漂核 vs 钉核**：dmgo 容器 cpuset 空（漂核）→ 与 worker 抢核 → mpcTimer 偶超预算 → policy stale → 边缘弹射假象（同配置同地形 dmgo 自跑 2/2 摔 / 钉核 worker 3/3 干净）。**所有定量结论与 demo 一律用 fresh-restart + 钉核 worker。** HPIPM QP 是串行 Riccati 递归，nThreads 只并行 LQ 近似，多核不加速 QP 但需保满求解率。
- **多容器隔离并行**（非单 Gazebo 多机器人，后者单线程物理 → RTF 掉 → stale policy 污染 timing 敏感的边缘测试）：`docker commit dmgo dmgo:par` 精确复制（含 warm CppAD cache + 当前 .so）→ workers 钉**不相交核** + 独立 network-ns（roscore 自动隔离）→ ~3× 提速。⚠ `docker commit` **不含 bind-mount 目录**（config 挂载的 → 不进镜像）→ worker 缺 config：spawn 不挂 config + `docker cp` + `sed` 改 per-worker task.info（实现并行扫不同 cost 权重，task.info 共享挂载时做不到）。
- **bind-mount 不一致**：`legged_perceptive`(整包) 是 bind-mount（host 改 .cpp 直达容器）；但 `legged_wbc` + `legged_controllers/SRC` **非 bind-mount**（冻结拷贝）→ 改 WBC/KF 必 `docker cp` + `catkin build` + `strings .so | grep ENV` 验（空编译 6.5s/0 串；真编译 19-35s）。改 KF 加成员 = ABI 变 → 须重编全链。
- **核查「跑的是最新代码」**：`devel/lib/*.so` 是符号链接（stat 看符号链接本身显示旧时间）→ 看链接目标 `devel/.private/<pkg>/lib/*.so` 的 mtime；**内容核查胜过时间核查**（`strings .so | grep ENV_FLAG` + `nm -C .so | grep <符号>`）；`md5sum` 跨容器一致性。
- **inline-pkill 自杀**：`pkill -9 -f <pat>` 若 `<pat>` 匹配启动 shell 自身 cmdline → 第一次 pkill 自杀（launch 没跑）。用精确 node 名 / `pgrep -x` / cleanup pattern 放 .sh 文件（非 inline cmdline）。
- **demo 录制需 X server**：Gazebo classic 相机 sensor 经 OGRE 渲染需 GL context = 需 Xvfb（`DISPLAY=:1`）。中途 `docker restart` 杀 Xvfb → 相机无帧 → recorder 静默空写 → 旧 mp4 没被覆盖。**「mp4 文件存在 ≠ 本次录的」必核 mtime+字节变化 + recorder.log 非空。** ⚠ `/tmp/.X11-unix` bind-mount 自 host 可能占用 `:1` → 用高号 display（`:99`）。
- **catkin OOM**：`PerceptiveLeggedReferenceManager.cpp` 含 OCS2+Eigen 模板 = 编译极耗内存，多 sim 跑着时强编会被 OOM killer 杀（`*.cpp.o] Killed`，非源码错）→ 空闲时编；catkin `-j2`（OOM→`-j1`），`OMP/OPENBLAS_NUM_THREADS=1`。
- **容器 recreate gotcha**：recreate 后 devel 回镜像基线 → 运行时 build 的包（如 `legged_dmgo_description`）丢失 → roslaunch RLException → recreate 后先重建。
- **URDF hygiene**：诊断 hack（改 CoM/mass）改在 robot.xacro **同时喂 Gazebo 物理 + OCS2 model 两边** → 结论前必 revert 真值 + grep 验证；无名 `<material>` → URDF→SDF 静默丢 collision → 穿地（dmgo 已全 link 命名）；OCS2 self-collision 用 primitive(box/cylinder/sphere) 不用 STL mesh（会炸）。
- **⭐ 确定性测量硬化配方（让噪声 harness 变确定性，N=1-4 可靠）**——定量复现 timing 敏感 bug 的核心手艺，叠满后 final-y 抖动从 ~1m 塌到几 cm（N=2 即决定性）：① **关 RANSAC 精化**（`convex_plane_decomposition.yaml include_ransac_refinement:false`，**最大杠杆**；clean map 滑窗拟合已够，RANSAC 是 run-to-run 抖动源）；② **MPC 单线程**（`sqp.nThreads 3→1`，多线程 LQ 浮点归约序变→微非确定）；③ **Gazebo `real_time_update_rate 1000→0`**（解耦墙钟避免丢步）；④ **前进运动按 base-x 触发（非墙钟）**——每次同 gait 相位撞台阶边 → 可复现（**最大 outcome 决定子**）；⑤ **graded 指标**（记 `max|roll|` / `min(base_z)` / final-y，非 pass/fail → N=2-3 即可分辨配置）。

### 3.9 dmgo vs a1 模型差异（边缘物理，非 bug）
| 量 | a1 | dmgo | 比 |
|---|---|---|---|
| 总质量 | 12.8kg | 14.7kg | +15% |
| trunk Ixx(roll 惯量) | 0.0159 | 0.0360 | **2.27×** |
| trunk CoM z 偏移 | ≈0 | −0.069 | base 帧在质心上方 6.9cm |
| 小腿质量 | 0.151 | 0.240 | +59%（远端质量→摆腿迟钝）|
| 整体 CoM 离足高 | 0.285 | 0.221（更低）| dmgo 静态更稳 |
| 足横向 track | 0.139 | 0.223（更宽）| dmgo 静态 roll 裕度 26.8° vs 13.7°|
- L/R 位精确镜像（**URDF 模型**无横向不对称 bug —— 但注意 task.info **代价矩阵** R 曾有 RH 腿 y/z 不对称 bug，见 3.3b，那是 config 不是模型）；前后 HFE 不对称是关节约定差异（落脚几何对称）；碰撞体与 a1 同量级。**模型真实而健全，不能靠改模型作弊。**

- **⭐ 反直觉对照：dmgo 出厂 `task.info` 其实比 a1 *更优待*，却仍翻 → 坐实「根因在方法不在参数」（doc10 §1，Agent-B 全数值 diff）**。把 perceptive `task.info` 逐项 diff（dmgo *修复前*的出厂值 vs a1），dmgo 拿的全是**更有利**的旋钮：

  | 旋钮 | a1 | dmgo(出厂) | 谁占优 |
  |---|---|---|---|
  | frictionCoefficient(MPC+WBC) | 0.3 | **0.6** | dmgo（更多横向力余量） |
  | torqueLimitsTask | 33.5 | **55** | dmgo（+64% 力矩权限） |
  | trot 周期(gait.info) | 0.60s | **0.70s** | dmgo（更长支撑相） |
  | Q（含 L_x/L_y/L_z、roll/pitch、base-z）/ R / swing / comHeight 0.30 / SQP 时序 | — | **完全相同** | 中性 |

  → dmgo 同时拿了**更多摩擦 + 更多力矩 + 更长支撑**却仍翻 ⇒ **没有任何「a1 占优而 dmgo 缺」的便宜旋钮可抄**，唯一真劣势就是上表的 **2.27× roll 惯量 + 重小腿（边缘物理）**。（注：此处 0.6/0.70 是出厂值；其中 friction 0.6 后被 §3.1 修成 0.3＝去掉零余量隐患，但这不改变「配置层无可抄旋钮」的结论。）这条对照把分散在 §2.6/§3.1/§3.9 的数字收成一处方法论：**移植存疑先 diff 全部数值差异、确认没有可抄的便宜旋钮，再判「边缘物理」vs「bug」——而非反射性瞎调权重。**

- ⚠ **诚实修正**：早期（doc09）把 2.27× roll 惯量当 roll-flip 根因，**后被 EXTREME-BRACKET 决定性证伪**（更轻翻更狠，惯量在 RESIST roll）。惯量是 amplifier（影响飞行相 roll 多快 H=Iω）不是 gate。**❌ 不要再提改惯量/质量分布。**
  - **附（calf 惯量是「真 artifact 但量化无关」的教科书案例，P3-095/P3-117）**：用 Eberly 多面体积分核 `RF_calf.STL` 实测——URDF `calf iyy=0.005316` vs mesh 正确 `0.001699`＝**3.13×**、`izz=0.004359` vs `0.000985`＝**4.43×**（密度 4412 kg/m³ 合理故**质量 OK，仅 iyy/izz 被 SolidWorks 导出高估 3-4×**，真小腿近端有膝电机、质量应**低于**均匀实心，故 3-4× 偏高＝确属导出误差）。**但良性**：平行轴定理 `m·d²` 主导，calf 自身惯量只占整体 roll 惯量 **~1%**，修了几乎不动 → 别误以为 calf 没问题、也别去白修它。

---

## 第四部分 · 诚实能力定论（同 clean_map harness，confound-free）

- **a1 reliable ~0.08–0.12**（0.08=5/5, 0.12=2/5, **0.16=0/5 cannot mount**，纯动量不足）。
- **dmgo reliable ~0.08**（0.08=9/10=a1-parity；0.10=0/10）。
- **真 gap = ~1 步级（~0.04m）在 over-launch frontier（0.10–0.12），非鸿沟。**
- ⚠ **a1@0.16 成功是真感知 harness 混淆**（run-to-run flaky），别用它定 dmgo 目标。两个具体量化机制解释「为何同代码 a1 容忍而边缘 dmgo 被推翻」：① **d435 深度噪声 stddev=0.10m vs 台阶 0.16m = 62% 噪信比**（`gazebo_d400.xacro`）→ 脚扎进噪声图，且让一切 N≤8 真感知判决既统计无效又被噪声污染；② **无接触检测=纯时钟步态**（grep `GaitAdaptation/contactDetection`=空）→ 台阶处前脚晚着地、调度已置 STANCE → 质心模型假设不存在的前脚 GRF → base wrench 错 → roll（这是 WBC measured-contact 坑在 MPC/调度侧的对应面）。

| 地形 | a1 | dmgo | parity? | 受限轴/机制 |
|---|---|---|---|---|
| 平地 / step≤0.08 / rough≤0.04 / downslope / elevated-spawn | ✅ | ✅ | **✅ parity** | downslope+elevated-spawn 是本 session 的 P0 KF 真 win |
| step 0.10–0.12 / slope crest / rough0.06 | ✅(略余) | ❌ | ✗ 窄 gap | 竖直 over-launch（force-peak）+ 侧向 veer + 感知 flicker 涌现不稳 |
| 缓坡 body(tilt) | ✅ | 🟡(banking 修，maxy 4-20×↓) | 部分 | 倾面侧向失稳 → banking 修侧向轴（坡身居中），crest 仍竖直墙 |
| step 0.16 | ❌(动量) | ❌ | 两机都难 | clean_map 都过不了，需更高接近动量/真感知 |
| gap 间隙 | ❌(无 planner) | ❌ | 两机都难 | 无 footstep planner（spawn 修好后仍跨越失败）|

**dmgo 缺陷专属 edge-stability 地形（重躯干暴露在竖直 over-launch + 倾面侧向 veer 两轴），非全地形差。** GOAL「0.16 ≥95%」实质：① ≤0.08 已 parity；② 0.10-0.12 窄 frontier gap 是 ~15 轮未破的涌现不稳；③ 0.16 本身高于 a1 自身能力。真正剩余路径 = phase-2 动量调度（已被 terrain_guard+VX sweep 证伪 reference 层 vx 调度无效）或 P2-C full-centroidal 重构（超 perceptive-control 范畴）。

**超包络地形「优雅降级停下」的工程结论（perceptual gating）+ 一条决定性负结论**：
- **gating 为何失败的真根因**：① 0.08 ledge 仍 steppable（<30° 阈值）→ controller 仍当可爬、仍 anticipate base-z ramp；② `smooth_planar` 被 `smoothing_dilation_size:0.2`（取邻域 max 的 dilation）**把邻近 raised cell re-grow 回** base 前向投影的查询位置 → 只 clip step cells 不够。
- **正确修法 = clip-to-FLAT（降到 local ground）而非 clip-to-ledge**（→ smooth_planar 在前向投影下保持平、无 base-z ramp）+ **clip ≥0.2m margin 打败 dilation** + **velocity stop gate**（因 FootPlacement/FootCollision 全是 soft RelaxedBarrier，持续 forward cmd 能压过 → 真停必须靠 zero cmd_vel，非软约束）+ `ZBASE_PLANE=1`（脚真踩上才升 base）。
- **决定性负结论（terrain_guard 证伪）**：实测证明 over-launch 是 **base-z ANTICIPATION 驱动**，**降速/全停（vx→0）都不治** —— 预抬由 footholds/lookahead 锚定、velocity-independent。即「优雅停下」能避免撞超包络 step，但**不能用减速来解 over-launch**（与 §3.3 表行 13 一致）。

---

## 第五部分 · 可吸收进《机器人学笔记》的点

> 标注「内容 + 建议进哪一部/章」。这些点技术准确、基于源码实证，多为通用知识或高价值失败经验。

### A. 进「足式机器人运动控制 / 感知 locomotion」部
1. **规划/运控分离范式**（where 选落脚点 vs how 求力）+ 感知 locomotion 完整链（elevation map → 凸平面分割 → 落脚点投影选区 → NMPC FootPlacement/FootCollision 约束 → WBC）。配 2.1-2.4 的数据流图。→ 新章「感知四足运动控制（OCS2 范式）」。
2. **落脚点启发式**（Raibert/capture-point：髋投影 + `√(h/g)(v−v_des)` 速度反馈 + `tan(pitch)·h` 地形俯仰偏移）+ **凸区域投影选区的 dz² 排序坑**（带高度筛选的隐含 bug）。→ 落脚点规划章。
3. **感知 NMPC 的 base 参考构造**（base-z = terrain+comHeight/cos(pitch) 单点 / base-pitch = 4 足平面拟合 / base-roll banking = 平面侧向梯度）+ **「base 先抬、落脚点跟上」的鸡生蛋耦合**。→ 参考轨迹生成章。
4. **WBC 任务化（QP）**：FloatingBaseEom/TorqueLimits/FrictionCone/NoContactMotion/BaseAccel/SwingLeg 六任务 + 质心动量率→base 加速度的映射 + 纯力矩下发（kp=0,kd=3,ff=τ）。→ 全身控制(WBC)章。
5. **摩擦锥安全余量约定**（controller μ = ½ sim μ）—— 一个简洁深刻的工程定律，可作「为何 MPC 摩擦系数要比真实小」的范例。→ 接触/摩擦建模章 或 sim-to-real 章。
6. **飞行相角动量不可控**（Raibert：姿态只能 stance 改、flight 守恒）+ **over-launch=flight-phase CAM 失稳** + N≥2 stance 摩擦锥对 ‖Ḣ_G‖ 的下界（arxiv 2605.05707）。→ 动态稳定性/角动量章（理论深度点）。
7. **接触状态估计**：WBC 用 gait-clock 接触 vs 实测接触（动量观测器/Kalman 融合 gait 先验，MIT Cheetah/Grandia event-based execution/downward regaining motion）。→ 状态估计/接触检测章。
8. **KF base-z 平地假设的局限 + terrain-aware contact-height 修**（touchdown-latch）—— 抬高地形上腿式估计器的典型缺陷与修法。→ 状态估计章。

### B. 进「工程实践 / 调试方法论 / sim-to-real」部（最有价值，用户最看重）
9. **证伪驱动的根因调研方法论**（3.7 全部 12 条元教训 + 3.3b 统计功效铁律）：setup≠result、别死磕调参（三-strikes）、proxy≠truth、common-mode 假收敛/confirmation-laundering 查独立性、证伪优先、直接测量胜推断、EXTREME-BRACKET、CLIFF-DISCRIMINATOR、多试次 pass-rate、基线先行、广度先于深度、Agent 研究必交叉验证失败史；**附「统计功效铁律：N=4 区分不了 30%↔10%（需 40-85 trial）= 反复误判的统计根源」+「纪律即生产力：单会话 3 连救场」实证**。→ 独立成「机器人调试方法论」章/附录，**这是全书最可迁移的横切内容**。
10. **完整的 over-launch 证伪链**（3.3 的 13 行假设表 + 最终「涌现多模不稳无单根」诚实定论 + 文献交叉验证 Thm 1/5/6/7 含 a_x⋆=μg/(1+2μκ)、N≥3 降 ‖Ḣ_G‖~227×、注入角动量 FUTILE + **诚实标注唯一真未试杠杆=input loopshaping**）—— 一个教科书级「错误转弯比正确结论更有教学价值」的完整案例研究。→ 案例研究/复盘章。
11. **CppAD/codegen 缓存陈旧静默坑**（改模型不自动失效缓存，`isLibraryAvailable` 只查文件存在）—— 通用「生成的二进制比源码活得久」缓存失效问题。→ 自动微分/codegen 工程章。
12. **ROS transport(TCP/UDP) 不匹配静默失败** + 「先确认机器人在执行目标 gait（探 mode 字段）再下 stall 结论」。→ ROS 工程踩坑章。
13. **OCS2/Gazebo 仿真工程坑合集**（控制器跑在 gzserver 内 → 崩在 gazebo.log + gdb 子进程绕 ptrace；NaN 边界致堆崩；僵尸进程墙；漂核 vs 钉核致 MPC-stale 假象；多容器隔离并行；bind-mount 不一致；.so mtime 符号链接陷阱；demo 录制需 Xvfb）。→ 仿真基建/CI 章。
14. **移植新机器人到感知框架的 checklist**（link 名匹配硬编码 / primitive collision + 全命名 material / 真 mesh 积分核惯量 / μ=½sim / 站姿标定配 frontReach / torqueLimit 对齐 URDF effort / gait UDP / climb 用 clean_map 确定性 / 改 modifyReferences 守连续有界有 fallback / 多试次 pass-rate）。→ 移植/bring-up 实践章。

---

> 复盘入口（源项目）：`docs/dmgo_port/00_index.md`（导航）→ `30_journey_retrospective_LESSONS.md`（全弧蒸馏）→ `29_capability_matrix_HONEST_FINAL.md`（诚实定论）→ `15_ocs2_perceptive_engineering_knowledge.md`（工程知识）。过程/决策细节在 `debug_log.json`(P3-NNN) + `decision_log.json`(D-NNN)。demo：`video/`（4 个 mp4）。
