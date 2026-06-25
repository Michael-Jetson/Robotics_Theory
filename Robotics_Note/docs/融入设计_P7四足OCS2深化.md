# 融入设计 · P7 四足部「OCS2 / legged_control」有机深化

> 设计师产物，**只出融入设计，不写正文**。
> 料源：`docs/Robot吸收_legged_control四足OCS2.md`（qiayuanl/legged_control = OCS2 NMPC+WBC+线性KF+sim2real 全栈 + 三大根因侦探故事 + PHASE3 两新根因 + 方法论）；
> 平衡控制器补料：`docs/extractions/quadruped_mpc__unitree.md`（宇树平衡控制器，已大量入书）。
> **核心纪律**：有机结合、融会贯通，**不是直接加入**。能深化现有节的不另起炉灶；失败经验/根因变成现有章里的「实战案例/为什么」；即便新增章也必须承上启下、接 `\cref`。

---

## 0. 总评与关键结构决策（先读这一节）

### 0.1 现状盘点（已读全 7 章骨架 + 关键节深读 + 料源全文）

P7 现有 7 章：`quad_arch`(726) / `quad_kin`(1420) / `quad_gait`(1230) / `quad_est`(1091) / `quadruped_mpc`(1410) / `quad_wbc`(1225) / `quad_prac`(607)。

**重大发现：P7 在「概念层」已经与 OCS2/legged_control 高度融合**，不是从零开始。已存在的锚点（grep + 深读亲验）：

| 已有锚点 | 位置 | label |
|---|---|---|
| 三框架难度阶梯（unitree_guide→A1-QP-MPC→legged_control），明确「瞬时 QP→凸 MPC→NMPC」预测能力递进 | `quad_prac` §ecosystem L136–181 | `ins:qp-ladder`, `tab:qp-frameworks` |
| OCS2 NMPC 标准形（质心动量态 `h_CoM`/`q_b`/`q_j`，输入 `f_c`/`v_j`，多重打靶→SQP→HPIPM） | `quad_prac` §anchor L251–268 | `eq:qp-nmpc`, `eq:qp-nmpc-state` |
| NMPC↔WBC 频率解耦（MPC 后台线程 `setupMrt`） | `quad_prac` §anchor L278 | `ins:qp-freq-decouple` |
| 宇树平衡控制器 = WBC「第 0 档」（力级、单刚体、软优先级退化 WBC） | `quad_wbc` §history L987–995 | `ins:qw-unitree-balance` |
| 瞬时 QP = 凸 MPC 的 N=1 退化（逐式对照） | `quadruped_mpc` §relation L1079–1097 | `ins:unitree-motivation`, `eq:srbd-matrix-unitree`, `eq:unitree-qp-cost` |
| 三模型谱系：LIPM / 质心动力学 / 单刚体凸 MPC | `quadruped_mpc` §srbd L194–217 | `tab:three-models` |
| STL 网格做碰撞体→OCS2 求解超时→乱抖（陷阱） | `quad_prac` §pitfalls L427–433 | `ins:qp-collision` |
| 线性 KF 18 维态 + 28 维观测 + 按接触相膨胀协方差(n_big=100) + Joseph 形 | `quad_est` §kf/§obs L739–824 | `eq:qe-cov-inflate`, `eq:qe-kf-cov` |
| cheater/KF 二选一（仿真可 cheater、真机只能滤波） | `quad_prac` §deploy L476；`quad_est` §practice L920+ | `sec:qp-deploy` |

**结论：平衡控制器这条「balance→MPC→WBC 难度阶梯」已经做透了**（§3 仅做小补强，见下）。真正的 GAP 是 OCS2 的**深层机理**与**失败经验**——它们当前几乎为零。

### 0.2 四大 GAP（料源里最值钱、书里几乎没有的）

1. **质心 NMPC vs 凸 SRBD-MPC 的深度对照 + ReferenceManager 在线更新 + RTI 实时机理**。
   `quadruped_mpc` 严格停在「凸 SRBD」范式，NMPC/质心/RTI 只在 §frontier 作 forward-reference（亲验：无 RTI / sqpIteration / 多重打靶细节 / ReferenceManager / HPIPM-串行地板）。
2. **42 维 WBC QP（`[q̈(18),F(12),τ(12)]`）+ base 加速度任务「四足纯前馈 vs 双足必须反馈」之辨**（braver 点足倒立摆反面教训）。
   `quad_wbc` §relax 用的是更瘦的松弛形 `X=[δq̈(6),δf]`，且**全章零提**四足↔双足 base-task 差异——这是最干净的高价值增点。
3. **四类 sim2real 失败根因作「实战案例」**：穿地(无名 material 静默丢碰撞) / 左右不对称(逐 link 非 bit-exact) / `sqpIteration=10`(边缘稳定) / 关节阻尼 200×(裕度侵蚀) + PHASE3「平地才是 REAL root」重定性 + 残留悬坑(改了 URDF effort 没改 task.info)。
4. **调试方法论**：对照实验 + 不变量守恒核对 + three-strikes + 诊断 crutch 登记清零 + 独立审计防自我说服 + 瞬态插桩反推稳态的陷阱 + 「加核救不了串行 solver / 加核救得了相机渲染」镜像对。

### 0.3 关键结构决策（三句话）

- **新增 1 章 `quad_advanced`「OCS2 NMPC 实战与 sim2real 调试方法论」**，作为 GAP 3+4（失败经验叙事 + 方法论 + 故障排查）与 GAP 1 中「ReferenceManager/RTI 实战」的统一落点。理由：这些料**横跨 MPC+WBC+est+prac 四章**、是叙事性的侦探故事 + 可迁移方法论，塞进任一现有章都破坏其单一焦点；独立成章并大量 `\cref` 回钉，最符合「承上启下」。放在 P7 末尾（`quad_prac` 之后）。
- **深化（不另起炉灶）3 处现有节**：
  (a) `quadruped_mpc` §frontier 末加「质心 NMPC vs 凸 SRBD」深对照子节（GAP 1 理论部分）；
  (b) `quad_wbc` §floating 末 +「四足纯前馈 vs 双足必须 base 反馈」+ 42 维 QP 形对照（GAP 2）；
  (c) `quad_est` §practice +「cheater→KF bring-up 门槛」、§frontier/标定 +「左右不对称作为估计/标定症状」（GAP 3 的估计相关根因）。
- **个别根因就地变陷阱**：`quad_prac` §pitfalls 新增「URDF→SDF 静默丢碰撞」「fixed-joint lumping」「关节阻尼/摩擦被 CAD 污染」「CppAD 缓存陈旧」「两份 config 只改一份(task.info vs URDF)」5 条陷阱，并补 `tab:qp-pitfalls` / 故障排查手册行；其余 sim2real 小经验(cmd_vel 限加速/goal 须 odom 帧/步态 UDP-only)登记进 §deploy 或 `quad_advanced`。
- **平衡控制器**：阶梯已做透，**只做小补强**（§3）：在 `quad_prac` §ecosystem 补一句「unitree balance 是入门最易上手的实跑入口」、确认 `quad_wbc`/`quadruped_mpc` 既有 `\cref` 互链无需动。

> **净增量**：新章 1（`quad_advanced`）；深化既有节 ~3 处；新增陷阱/案例 ~7 条。**绝不重复**已有的 `eq:qp-nmpc`/`ins:qw-unitree-balance`/`tab:three-models`，新内容一律 `\cref` 它们。

---

## 1. 逐章映射表（每章：哪些料融进哪一节，深化/案例/不动）

> 粒度到「在 §X 加『子节标题』」。失败根因明确分配到章。

### 1.1 `quad_arch.tex`（导论与控制架构）—— 基本不动 + 1 处微补

- **§qa-arch「分层架构」/ §qa-two-streams**：现有架构图已含 状态估计→规划→MPC+WBC。**微补**：在 §qa-arch 的「怎么去 = MPC+WBC」末，加一句**前向指针**——「本部把这套分层在 OCS2/legged_control 上的**工业级实例**(质心 NMPC + 加权/分层 WBC + 线性 KF)留到 `\cref{ch:quad_advanced}` 完整展开」。**不新增节**。料源 §1.1–1.2（血缘 + 在线数据流三层频率）。
- 其余节不动。

### 1.2 `quad_kin.tex`（浮动基建模与运动学）—— 不动 + 1 处案例钩子

- **§qk-misconceptions「常见误解辨析」(L1230) 或 §qk-wholebody §电机零点与模型零点(L1212)**：现有已讲「电机零点 vs 模型零点」。**加 1 条误区/案例**：「**同作者参考件里与零位/轴向耦合的几何宏不可复用**」——legged_control 的 `leg.xacro` 把「大腿垂直零位」硬编码进关节 origin(`KFE origin xyz="0 0 -thigh_length"`)，若自定义机器人零位约定不同(水平向后)强行复用会让 STL 网格方向全错。这正是「电机零点≠模型零点」的工程放大。**料源 §1.4**。一段 `insight` 即可，主战场仍在 `quad_advanced` 的移植清单。
- 运动学正文(FK/IK/雅可比/Plücker/零空间)已极完整，**不动**。

### 1.3 `quad_gait.tex`（步态生成与轨迹规划）—— 不动 + 2 处实战补强

- **§qg-scheduler §模式编号 / §qg-swing 摆动足轨迹**：现有讲了相位变量、占空比、Raibert、摆线 z 曲线。**补料(案例钩子，非另起节)**：
  - 在 §qg-scheduler 末补一条 `insight`「**重机器人需更慢 trot**」：go1 trot 周期 0.6s `[0,0.3,0.6]`，aliengo 类重机放慢到 0.7s `[0,0.35,0.70]`——每步更长支撑相、在俯仰步顶遏制 roll（与 PHASE3 关节阻尼修复配套）。**料源 §2.2 + §3.10**。这条把「步态周期是可调的 sim2real 旋钮」钉到既有调度器节。
  - 在 §qg-swing 末补一句：OCS2 `SwingTrajectoryPlanner` 的 z 曲线参数(`swingHeight=0.08`/`liftOff=0.05`/`touchDown=−0.1`/`swingTimeScale=0.15`)是 **MPC 侧约束**的落脚 z 来源，与 **WBC 侧跟踪增益**(swingKp=350/kd=37)分属两层——呼应 `\cref{ch:quad_wbc}` 摆动腿任务。**料源 §2.1 脚注**。
- 落足点/坡度估计正文**不动**（书里已比料源细）。

### 1.4 `quad_est.tex`（足式状态估计）—— **深化 §practice + §frontier**（GAP 3 估计部分）

现有：线性 KF 18 维态/28 维观测、按接触相膨胀协方差(`eq:qe-cov-inflate`, n_big=100)、Joseph 形、§practice 已讲 Q/R 整定 + cheater/KF 提及 + getRotMat/雅可比 API 胶囊、§frontier 已有 Bloesch/Bledt/De Luca/InEKF 谱系。

- **§qe-practice「实践与部署」新增子节『cheater→KF：bring-up 的真门槛』**（深化，非另起章）：
  - `FromTopicStateEstimate`(仿真 cheater，直接取 `/ground_truth/state` p3d 透传不滤波) vs `LinearKalmanFilter`(生产，IMU 预测 + 足端 FK 更新)。**cheater 用零噪地真 → 指标是乐观下界；切 KF 引入真实漂移 = bring-up 真门槛**。bring-up 调试首选 cheater(排除估计误差)，对应 `legged_cheater_controller`。**料源 §2.4**。
  - 噪声配置三处落点(切 KF 必看)：①IMU 传感器噪声=`imu.xacro`/`gazebo.xacro` 的 `gaussianNoise`；②KF 协方差=task.info `kalmanFilter` block；③p3d gaussianNoise(仅 cheater)。**料源 §2.4**。
  - **诚实 caveat**(呼应方法论的「独立审计」)：legged_control 仿真 KF 实为「干净」(`LeggedHWSim.cpp:96` TODO 未加噪、只播协方差)，估计滞后真值~13%——「带噪 IMU」是过度声明。**料源 §3.4**。
- **§qe-frontier 或 §qe-practice 新增 `insight`『左右不对称会先在状态估计/标定里现形』**（GAP 3 的「左右不对称」根因落点）：
  - 四足逐 link 独立硬编码→左右**永不 bit-exact 镜像**→残留 L/R 不对称(KFE origin z 3mm 最大项)→`check_lr_symmetry.py`+`verify_pose` FK **最先**在站姿 z-spread(3mm)里揪出。把它当「估计/标定阶段的可观测症状」：站立时若左右髋劈叉方向反/不对称，**第一怀疑点是关节序映射或几何不对称**(OCS2 默认 jointNames 序 LF,RF,LH,RH ≠ URDF 声明序 LF,LH,RF,RH，但按名建模故不是漂移源)。**料源 §3.2 + §3.8 关节序**。
  - 治本(腿宏 bit-exact 镜像)的完整侦探故事放 `quad_advanced`；这里只钉「估计/标定先现形」这一面 + `\cref{ch:quad_advanced}`。
- KF 推导正文、GM 接触观测器**不动**（书里已极完整）。

### 1.5 `quadruped_mpc.tex`（单刚体 MPC）—— **深化 §frontier**（GAP 1 理论部分）

现有：严格凸 SRBD 范式；§relation 讲透「瞬时 QP=N=1 退化」「升级路径」「Di Carlo 标准形」「控制栈图」；§frontier 已列 Di Carlo / 几何 MPC / WBIC / Wensing 谱系。**§frontier 现在只把 NMPC 作 forward-reference**，正是深化口。

- **§frontier 末新增子节『从凸 SRBD 到质心 NMPC：OCS2 这一档』**（深化既有 §frontier，**不重复** `eq:qp-nmpc`，而是把 `quad_prac` 已建的 NMPC 标准形拉进来做**深度对照**）：
  - **建模保真度对照**：凸 SRBD(单刚体 + 忽略陀螺 + 小角 + 重力增广，**换凸性**) vs 质心动力学 NMPC(6 维质心动量含角动量，**保非线性**)。一张「三处近似各换回什么」对照表，接 `\cref{tab:three-models}`。
  - **约束表示两层**(料源亲验的精确细节)：MPC 侧=**平滑摩擦锥** `μ·F_z−√(F_x²+F_y²+ε²)≥0`(log-barrier μ/δ)；WBC 侧=**摩擦金字塔**(5 行/足线性化)——同一可行集两种表示，接 `\cref{ch:quad_wbc}`。**料源 §2.1 + §2.3**。
  - **代价构造**：`½‖x−x_ref‖²_Q+½‖u−u_ref‖²_R`，R 代价参考=`weightCompensatingInput`(=mg)→平衡点零拉力、模型无 below-mg 偏置（——顺带破料源「Fz/mg≈0.90 是塌陷瞬态假象」红鲱鱼，但红鲱鱼本体留 `quad_advanced`）。**料源 §2.1**。
  - **求解链 + RTI 实时机理**(GAP 1 核心，书里完全没有)：多重打靶→**RTI-SQP(每周期 1 次 Newton 步, `sqpIteration=1`)**→HPIPM(Riccati 一遍解)→滤波线搜索→warm-start。**为何「加核救不了」**：LQ 近似/线性化=embarrassingly parallel(nThreads 3→6: 2.02→1.25ms)，但 **HPIPM 解 QP 沿时域 Riccati 递归=本质串行→并行地板~0.29ms**。**实时的加速是少算(RTI 1 迭代+warm-start)不是多并行**。RTI 之外的提速菜单：GNRK / Multi-level RTI / partial condensing / move-blocking。**料源 §2.1 + §3.3**。
  - **ReferenceManager 在线更新**：`SwitchedModelReferenceManager`=`GaitSchedule`+`SwingTrajectoryPlanner`，`getContactFlags(t)`→注入 ModeSchedule(接触序列)+摆动足 z 曲线到 MPC horizon——**这是「凸的前提：步态由独立 FSM 给、不进优化」**(接 `\cref{ins:selection-matrix}`)的工业级实例。**料源 §2.2**。
- **§practice「参数整定」补一句**：单刚体 MPC 部署的「质量属性实验整定」已讲；补「**Q 权重应随质量重标定**」caveat——dmgo 14.7kg vs go1 12.0kg(+22%) 却照抄 Q，质量是守恒核对量，重机沿用轻机 Q 是隐患。**料源 §3.8**。一句话即可，不另起节。
- §srbd / §convexify / §prediction / §qp **不动**（凸 MPC 推导是本章主线，已极完整）。

### 1.6 `quad_wbc.tex`（全身控制 WBC）—— **深化 §floating + §practice**（GAP 2）

现有：§floating 讲浮动基动力学标准方程 + 四子任务 + 加速度级递推；§relax 用松弛形 `X=[δq̈(6),δf]`；§history 有 NSP/WQP/HQP + 宇树「第 0 档」；§practice 5 陷阱 + 故障排查。**全章零提四足↔双足 base-task 差异**——最干净的高价值增点。

- **§floating「浮动基 WBC 与四足应用」末新增子节『base 加速度任务：四足纯前馈 vs 双足必须反馈』**（GAP 2 核心，**最高价值**）：
  - **四足 base 加速度任务=纯前馈**(`formulateBaseAccelTask`)：`b=A_b⁻¹·(质心动量率_MPC−Ȧ·v−A_j·q̈_j)`，**无任何 base 位姿/高度/姿态反馈增益**，desired 动量率纯来自 MPC。**对四足正确**(4 足静稳 + MPC 隐式稳基)。
  - **双足必须加反馈**：hunter_bipedal 把它拆成 BaseHeightMotion(kp/kd)+BaseAngularMotion(kp/kd)+BaseXYLinearAccel(仍前馈)。
  - **反面教训(case)**：braver(点足双足)误用 go1 纯前馈版(增益=0)→点足倒立摆**必发散**，耗~120 phase 才揪出；dmgo(四足)直接用原版不会重蹈。**「四足为何能纯前馈、双足为何不能」是 quadruped↔biped 的核心区别**。
  - 顺带钉一个易错点：EoM 的 `nle`(Pinocchio `data.nle`)已含重力+Coriolis+离心，**EoM task 别重复加重力**(issue#45)。**料源 §2.3**。
- **§floating 或 §relax 补 `insight`『42 维 WBC QP 的精确变量构成』**（深化，与既有松弛形 `X=[δq̈(6),δf]` 做对照而非取代）：
  - 决策变量 `x_wbc=[q̈(18),F(12),τ(12)]=42`(亲验 `WbcBase.cpp:24`: `generalizedCoordinatesNum(18)+3·numThreeDofContacts(12)+actuatedDofNum(12)`)。**注意 q̈ 是全 18 维广义加速度(base6+关节12)而非只 base6**——料源记「两子 agent 曾误写 30/base6」，正好作「核心维度必回源码验」的诫语，呼应方法论。
  - 各 task 精确数学(浮动基 EoM 18×42 / 力矩限 / 摩擦金字塔 5 行 / 支撑足不动 `Jᶜ·q̈=−dJᶜ·v` / 摆动腿 swingKp=350 / 接触力=F_des(MPC))。接 `\cref{eq:qw-floating-dyn}`、`\cref{ch:quad_mpc}`(接触力上游)。**料源 §2.3 + 附录 file:line**。
  - **WeightedWbc vs HierarchicalWbc 落到代码**：`WeightedWbc`(硬约束 EoM+力矩限+摩擦+支撑不动 + 软任务 摆动×100+base加速度×1+接触力×0.01 堆单 QP→qpOASES) vs `HierarchicalWbc`+`HoQp`(严格优先级零空间投影, Bellicoso16)——这正是既有 §history WQP↔HQP 谱系的**工业实例**，接 `\cref{tab:qw-spectrum}`、`\cref{ins:qw-unitree-balance}`。**料源 §2.3**。
- §practice **补 1 条陷阱**：「**力矩上界写在哪**」——WBC 力矩上界读 `task.info` 的 `torqueLimitsTask` **不读 URDF effort**，故「修了 URDF effort 没修 task.info」会让某关节力矩仍被旧值饿死(HAA 卡 20→台阶顶髋外展力矩不足→侧翻)。**这是 GAP 3「残留悬坑」根因在 WBC 章的落点**(完整故事在 `quad_advanced`，此处钉「两份 config 各管一摊」)。**料源 §3.4 P3-031**。
- NSP/伪逆/零空间推导、§relax 松弛 QP 正文**不动**。

### 1.7 `quad_prac.tex`（实践：宇树框架与开源控制器）—— **新增陷阱/路线图条目**（GAP 3 部署相关 + 小经验登记）

现有：§ecosystem 三框架阶梯、§anchor NMPC 标准形、§pitfalls 4 大坑(关节名/STL/构型/零点)+速查表+故障排查手册、§deploy 6 步路线图。**这是失败根因「部署陷阱」类的天然落点**。

- **§qp-pitfalls 新增陷阱（承上启下，扩展既有 `ins:qp-collision` 而非另立体系）**：
  - **`pit:qp-urdf-sdf`「URDF→SDF 静默丢碰撞」**(GAP 3 根因#1 穿地)：无 `name` 属性的 `<material>`(非法 URDF)→urdfdom/sdformat 在 URDF→SDF 解析时**静默丢弃 collision**(`<visual>` 仍在→**RViz 看得到、Gazebo 物理穿模**)→穿地下沉。**金标准排查**：`gz sdf -p robot.urdf | grep -c collision` 核对 **URDF 数==SDF 数**(料源铁证：URDF 20→SDF 3 丢 17，给无名 material 加 name 后 3→17)。**与既有 `ins:qp-collision`(STL 网格)同类**——都是 URDF→Gazebo 解析期静默丢东西。**料源 §3.1**。
  - **`pit:qp-fixed-lumping`「fixed-joint lumping 两条静默坑」**：Gazebo 默认把 fixed joint 合并进父 link，而框架靠 link 名查找→`*_foot_fixed` 必加 `<disableFixedJointLumping>`(否则 `*_FOOT` 被合并→ContactManager 报不出足接触→KF 收不到接触相→估计错)；`base_imu_joint` 必加(否则 `GetLink("base_imu")` 返 null→`ROS_ASSERT` 崩)。**料源 §2.6**。
  - **`pit:qp-cad-damping`「关节阻尼/摩擦被 CAD 导出污染」**(GAP 3 根因 PHASE3-A 的部署面)：SolidWorks 导出常把 `joint_damping`/`joint_friction` 填成离谱大值(dmgo 200×/5×)；**MPC 质心模型完全忽略关节摩擦/阻尼**而 Gazebo 真实施加→响应迟钝→**裕度悄悄侵蚀→翻**(不报错、不丢碰撞、维度也对)。移植自带 URDF 必先核对。完整 sim2real 机理留 `quad_advanced`。**料源 §3.10-A**。
  - **`pit:qp-cppad-cache`「CppAD 缓存陈旧」**：改 URDF 后旧缓存让 MPC 用旧模型/巨慢→首跑 `recompileLibrariesCppAd=true` 再关，或清 `/tmp/legged_control/<robot>`(issue#79)。**料源 §3.8**。
  - **`pit:qp-task-vs-urdf`「两份 config 只改一份」**：URDF(连杆树/惯量/限位/碰撞/关节零位)与 OCS2 `*.info`(Q/R/约束/力矩限/步态)**各管一摊**，改一份漏另一份是移植期典型陷阱(力矩限的例子见 `\cref{ch:quad_wbc}`)。**料源 §3.4**。
- **`tab:qp-pitfalls` 速查表 + 故障排查手册**：各新增对应行(穿地→gz sdf 守恒核对；足接触报不出→disableFixedJointLumping；翻车且无报错→查 joint_damping/effort 不一致)。
- **§qp-deploy 路线图补**：第 1 步「写 URDF」补「核对碰撞数守恒 + 关节阻尼/摩擦非 CAD 污染值 + 全 link material 命名」；第 5 步「选状态估计」补「bring-up 先 cheater 再 KF」(`\cref{ch:quad_est}`)。
- **§ecosystem 微补**(平衡控制器小补强，见 §3)：补一句「unitree_guide 平衡控制器是**入门最易上手的实跑入口**，先在它上面跑通『站稳+力分配』再上 MPC/NMPC」。
- §anchor / §hwabs / §lifecycle / §fsm 正文**不动**（已极完整）。
- **小经验登记**(可迁移但不值得正文展开的，登记进 §deploy 的 `note` 或直接进 `quad_advanced` 的「sim2real 实操约束」节)：`/cmd_vel` 须平滑限加速 |a|≤0.3；goal 导航(`/move_base_simple/goal`)必须发 `odom` 帧否则静默丢弃；步态切换 `GaitReceiver` 是 **UDP-only**(rospy/rostopic 仅 TCP 连不上)→须用 roscpp 键盘节点 + `LD_LIBRARY_PATH=/usr/local/lib`；OCS2 是 monorepo 禁编全量；DEBUG 构建致 RTF~0.05 必 Release。**料源 §3.5/§3.6/§3.11/§3.9**。

---

## 2. 新增章 `quad_advanced`：是 / 否 + 节级大纲

**决定：是。** 章名暂定**「OCS2 NMPC 实战与 sim2real 调试方法论」**，label `ch:quad_advanced`，放 P7 末尾（`\input{parts/P7_legged/quad_prac}` 之后，加 `\input{parts/P7_legged/quad_advanced}`）。

**定位（承上启下，一句话）**：前 7 章把四足控制的**理论拼图**讲全(运动学/步态/估计/MPC/WBC/落地)；本章把它们**钉到一个真实的工业级开源栈(qiayuanl/legged_control = OCS2 NMPC+WBC+KF)上跑起来**，并把「移植一台自定义四足」过程中**真实踩过的失败经验、根因侦探故事、可迁移调试方法论**凝练成案例——是全部前章的**实战合流与方法论收束**。

**为何独立成章而非塞进现有章**：①料是**横跨 MPC+WBC+est+prac** 的叙事(三大根因 + PHASE3 两新根因 + 方法论)，塞任一章都破坏其单一焦点；②大量「症状→红鲱鱼→真根因→怎么确认→治本→教训」侦探结构，是**方法论教学体裁**，与前章「公式推导」体裁不同；③可整章 `\cref` 回钉前 7 章，最符合「有机融入、承上启下」。

### 节级大纲（节标题列表）

```
\chapter{OCS2 NMPC 实战与 sim2real 调试方法论}\label{ch:quad_advanced}

§1 全栈数据流：把前七章拼成一个实时闭环              \label{sec:qadv-stack}
   —— TargetTrajectories→ReferenceManager→SqpMpc→MPC_MRT_Interface→WeightedWbc
      →HybridJoint→线性KF→闭环；三层频率(MPC异步~100Hz / WBC每tick / KF每tick)。
      一张图讲透「分层架构(\cref{ch:quad_arch})在 OCS2 上长什么样」。承上：钉回
      \cref{ch:quad_mpc}(NMPC层) / \cref{ch:quad_wbc}(WBC层) / \cref{ch:quad_est}(KF层)。

§2 移植一台自定义四足：要提供什么、什么绝不能复用      \label{sec:qadv-port}
   —— 机器人专属=仅 3 个 config.info + URDF；通用算法代码全不动。
      薄包可复用 unitree 的 gazebo/imu/transmission/materials 宏(机器人无关)，但
      leg.xacro(运动学)绝不能复用(零位/轴向耦合进关节 origin)——接
      \cref{ch:quad_kin}「电机零点≠模型零点」。关节零位 FK 实算取代解析值。

§3 根因侦探故事一：穿地 —— URDF→SDF 静默丢碰撞        \label{sec:qadv-rc1}
   —— 症状(STANCE 站不住/qpOASES 刷屏/塌到 z≈−0.17)→红鲱鱼A(Fz/mg≈0.90 瞬态假象)
      →红鲱鱼B(initialState/趴卧 hack 消 core-dump≠根因)→真根因(14 个无名 material
      静默丢 17/20 碰撞)→金标准确认(go1 同容器对照 + gz sdf grep -c collision 守恒核对)
      →治本(全 link material 命名)→教训。陷阱本体已进 \cref{ch:quad_prac}，本节给完整侦探链。

§4 根因侦探故事二：veer/自旋 —— 逐 link 左右永不 bit-exact \label{sec:qadv-rc2}
   —— 症状(trot 左偏 10m/自旋出局)→补偿器三连试均「搬家不是修」(航向锁→crab/path-follower
      /KFE 单点修正翻转方向)→真根因(逐 link 硬编码左右非镜像，KFE origin z 3mm 最大项)
      →go1 决定性对照(为何 go1 走直=leg.xacro 构造性 bit-exact 镜像)→治本(参数化腿宏
      dmgo_leg(prefix,mirror,front_hind) 实例化 4 次，镜像乘子规则 verbatim 抄 go1；
      最易写错特例 hip ixy=×mirror×front_hind)→验证(L/R 残差 0.000e+00)→教训
      (左右不对称=bug / 前后站姿不对称=非 bug；结构性对称>一次性脚本镜像)。
      估计/标定先现形那一面已进 \cref{ch:quad_est}；腿宏接 \cref{ch:quad_kin} 整机运动学对称。

§5 根因侦探故事三：边缘稳定 —— sqpIteration=10 诊断 crutch \label{sec:qadv-rc3}
   —— 最隐蔽最核心。症状(0.13 摆/veer/摔/comHeight 0.28 摔全是下游)→真根因
      (sqpIteration=10 → 13.5ms 串行 > 100Hz 的 10ms 预算 → stale/laggy policy = 边缘稳定)
      →确认(KF 本身没问题 max 3.7°；go1 真 KF 基线稳；diff task.info sqpIteration go1=1/dmgo=10
      →10→1 一发命中)→机制更正(核数不是因！sqp10 在 4 核/8 核耗时相同；LQ 可并行、HPIPM
      Riccati 串行地板)→「sqpIteration=1=RTI 正确实时设计非妥协」→教训(「边缘稳定」先怀疑
      控制实时性、mpcTimer Max/Avg<预算是判据)。RTI 机理本体已进 \cref{ch:quad_mpc} §frontier。

§6 PHASE3 反转：M1「通过」不等于「鲁棒」               \label{sec:qadv-phase3}
   —— 修正「三大根因全治本、M1/M2 圆满」的乐观叙事。新根因A(关节阻尼/摩擦模型-仿真失配致
      裕度侵蚀，陷阱已进 \cref{ch:quad_prac}，此处给 sim2real 机理 + comHeight 0.40 稳/0.30 翻
      实验 + trot 周期 0.6→0.70s 放慢配套)；新根因B(根因重定性：平地不稳才是 REAL root、
      台阶/感知是下游——「dmgo 平地 5m FLIPS、a1 平地 rock-stable」)。教训：移植验收「通过」
      ≠「鲁棒」，平地残余不稳必须在上感知前清掉。

§7 诊断 crutch 登记清零 + 独立审计防自我说服          \label{sec:qadv-crutch}
   —— 3 个同类 crutch(R 力权重 1→0.01 / torqueLimit 真值→80 / sqpIteration 1→10)全是诊断
      遗留、全误导后续分析、全须事后清零。独立审计(最小提示词独立子 agent 复核 headline)
      揪出：真 bug(WBC torque 限 task.info 20/55/55 vs URDF effort 90 三处冲突，HAA=20 饿死
      roll 恢复力矩→残留悬坑「修了 URDF 没修 task.info」，接 \cref{ch:quad_wbc})、过度声明
      (go1-parity 单次最佳 vs 实测均值 5×)、crutch 未清。

§8 sim2real 调试方法论（可迁移到任何机器人移植）       \label{sec:qadv-method}
   —— 本章最值钱、最可迁移。①对照实验 + 不变量守恒核对(碰撞数/维度/质量)>>盲调，高效一个
      数量级；②「研究同栈可用参考的『为何能 work』→diff config→适配」(go1 config 就是答案表)；
      ③three-strikes(连续>3 次治标失败必转调研、不闭门造车)；④瞬态插桩反推稳态的陷阱(0.90
      红鲱鱼)；⑤幻影 qpErr 4 成因(pin teleport / 步态 UDP 没接上 / sqpIteration stale /
      stale-gazebo CPU 争用——归因模型前先确认容器干净)；⑥「加核救不了串行 solver vs 加核救得了
      相机渲染」镜像对(算力不足的失败要区分是否真不可行)；⑦字节级抽取>手抄(避免转写错落在
      关节零位/轴向杀手区)；⑧测试判摔用 z+roll/pitch 不用四元数 w(w=cos(yaw/2)纯旋转误判摔)。

§9 资源护栏与运行环境（把失败域圈进容器）             \label{sec:qadv-guardrail}
   —— sim2real 实操约束登记：容器 --cpus/--memory 圈住失败域(编译爆内存只 OOM 杀容器)；
      catkin -j2;OMP=1;一次一容器;pkill-by-name(从文件跑脚本避免自杀);OCS2 monorepo 禁编全量;
      cmd_vel 限加速|a|≤0.3;goal 须 odom 帧;步态 GaitReceiver UDP-only;DEBUG 构建 RTF~0.05 必 Release。
      —— 纯工程细节一律 \cref{ch:engineering}，本章只登记「不懂会卡死」的项。

§* 本章小结 + 故障排查总表 + 方法论速查 + 延伸阅读 + 与相关章节关系表
```

> **节数弹性**：§3–§5 三个侦探故事是本章脊梁，必写；§6/§7 可视篇幅合并为「M1 之后：反转与审计」一节；§8 方法论必须独立、是全章价值顶点；§9 可压成 §8 的子节或登记表。建议**实际成章 7–9 节**。

### 新章接的 `\cref`（承上启下钉点）

| 接哪个 label | 在哪节 | 关系 |
|---|---|---|
| `\cref{ch:quad_arch}` (`sec:qa-arch`) | §1 | 全栈数据流 = 架构图的工业实例 |
| `\cref{ch:quad_kin}` (`sec:qk-wholebody` 电机零点 / `sec:qk-wholebody` 整机对称) | §2/§4 | 零位/轴向不可复用；腿宏 bit-exact 镜像 |
| `\cref{ch:quad_gait}` (`sec:qg-scheduler`) | §6 | 重机 trot 周期放慢 |
| `\cref{ch:quad_est}` (`sec:qe-practice`, `sec:qe-kf`) | §1/§4 | KF 层；左右不对称估计先现形；cheater→KF 门槛 |
| `\cref{ch:quad_mpc}` (`sec:frontier`, `ins:selection-matrix`, `sec:relation`) | §1/§5 | NMPC 层 + RTI 机理 + ReferenceManager=「凸的前提」实例 |
| `\cref{ch:quad_wbc}` (`sec:qw-floating`, `tab:qw-spectrum`, `ins:qw-unitree-balance`) | §1/§7 | WBC 层 42 维 QP；力矩限残留悬坑 |
| `\cref{ch:quad_prac}` (`sec:qp-pitfalls`, `ins:qp-collision`, `sec:qp-deploy`) | §3/§6 | 陷阱本体在 prac，本章给完整侦探链 |
| `\cref{ch:engineering}` (`ch:engineering`) | §9 | 纯工程细节(命令/容器/网络)承接 |

---

## 3. 平衡控制器（P7 补充）：落点 + 取料出处

**判定：balance→MPC→WBC 难度阶梯在书里已经做透了**，不需要新节，**只做小补强 + 确认互链**。

- **难度阶梯落点（已存在，不动）**：
  - **balance 档**：`quadruped_mpc` §relation `ins:unitree-motivation`(瞬时 QP=N=1 退化，逐式 `eq:srbd-matrix-unitree`/`eq:unitree-qp-cost`) + `quad_wbc` §history `ins:qw-unitree-balance`(力级、单刚体、软优先级=WBC「第 0 档」)。
  - **MPC 档**：`quadruped_mpc` 全章(凸 SRBD)。
  - **WBC 档**：`quad_wbc` 全章 + §history 谱系 NSP/WQP/HQP。
  - **统一阶梯表**：`quad_prac` §ecosystem `tab:qp-frameworks` + `ins:qp-ladder`(瞬时 QP→凸 MPC→NMPC 预测能力递进)。
  - 取料出处：宇树《四足机器人控制算法》第 8 章平衡控制器 = `docs/extractions/quadruped_mpc__unitree.md` §5(机身位姿 PD→足底力 QP→简化逆动力学 §5.1–5.6) + §6(QP→MPC/WBC 对照)。**已 `\cite{unitree2023quadruped}` 入书**。

- **小补强（唯一动作）**：在 `quad_prac` §ecosystem 补一句——「**unitree_guide 的平衡控制器是这条阶梯里入门最易上手的实跑入口**：先在它上面把『站稳 + 瞬时 QP 力分配 + 单腿静力学反解』跑通(`\cref{ch:quad_mpc}` 的 `ins:unitree-motivation`)，再拾级到凸 MPC(A1-QP-MPC)、NMPC(legged_control)」。这把「平衡控制器 = 难度阶梯第一级实跑入口」这一**学习路径**显式钉出来(料源 `quadruped_mpc__unitree.md` §5.0 动机 + §6 对照骨架)。

- **确认互链无需动**：`quad_wbc` L988/L1013/L1165/L1180 与 `quadruped_mpc` L1080/L1096/L1146/L1350 已双向 `\cref` 互链(平衡控制器↔MPC↔WBC「首尾相接」)，**结构已健全，写作 agent 不要重复造**。

---

## 4. cref 接线清单（新内容引用现有哪些 label）

> 写作 agent 据此连线，**严禁重复定义**下列已有 label；新内容一律 `\cref` 它们。

### 4.1 已存在、可被引用的 label（亲验）

| label | 所在 | 含义 |
|---|---|---|
| `ch:quad_arch` `sec:qa-arch` `sec:qa-two-streams` | quad_arch | 分层架构 / 优化与 RL 两条腿 |
| `ch:quad_kin` `sec:qk-wholebody` `sec:qk-jacobian` `sec:qk-nullspace` | quad_kin | 整机运动学/对称 / 雅可比 / 零空间 |
| `ch:quad_gait` `sec:qg-scheduler` `sec:qg-swing` | quad_gait | 步态调度 / 摆动足轨迹 |
| `ch:quad_est` `sec:qe-kf` `sec:qe-practice` `sec:qe-frontier` `eq:qe-cov-inflate` `ins:qe-inekf` | quad_est | 线性 KF / 实践部署 / 前沿 / 协方差膨胀 / InEKF 指针 |
| `ch:quad_mpc` `sec:relation` `sec:frontier` `sec:practice` `tab:three-models` `ins:unitree-motivation` `ins:selection-matrix` `eq:qp-nmpc`(在 prac) `eq:dicarlo-form` `fig:control-arch` | quadruped_mpc(+prac) | 凸 MPC 全套 + NMPC 标准形 |
| `ch:quad_wbc` `sec:qw-floating` `sec:qw-relax` `sec:qw-history` `tab:qw-spectrum` `ins:qw-unitree-balance` `eq:qw-floating-dyn` `eq:qw-slack` | quad_wbc | WBC 全套 + 谱系 + 第 0 档 |
| `ch:quad_prac` `sec:qp-ecosystem` `sec:qp-anchor` `sec:qp-pitfalls` `sec:qp-deploy` `ins:qp-collision` `ins:qp-ladder` `ins:qp-freq-decouple` `tab:qp-pitfalls` `tab:qp-frameworks` | quad_prac | 三框架 / 锚定 / 陷阱 / 路线图 |
| `ch:engineering` | P5 | 纯工程细节承接 |
| `ch:rl-basic` `ch:rlmc-quadloco` | P? | RL「另一条腿」入口 |
| `ch:eskf` `ch:invariant-filter` `ch:imu` `ch:lie` `ch:rigid_body` | 估计/数学部 | 通用 KF/ESKF/InEKF/李群外引 |

### 4.2 新建 label（写作 agent 负责创建，命名约定）

- 新章：`ch:quad_advanced`；节 `sec:qadv-stack/port/rc1/rc2/rc3/phase3/crutch/method/guardrail`。
- 新陷阱(quad_prac)：`pit:qp-urdf-sdf` / `pit:qp-fixed-lumping` / `pit:qp-cad-damping` / `pit:qp-cppad-cache` / `pit:qp-task-vs-urdf`。
- 深化新增(quadruped_mpc §frontier)：`ins:mpc-nmpc-compare` / `ins:mpc-rti` / `ins:mpc-refmgr`。
- 深化新增(quad_wbc §floating)：`ins:qw-base-feedforward`(四足前馈vs双足反馈) / `ins:qw-42dof`(42维QP) / `pit:qw-torque-task-vs-urdf`。
- 深化新增(quad_est §practice)：`ins:qe-cheater-gate` / `ins:qe-lr-asymmetry`。

### 4.3 引用键(cite)登记 —— **refs.bib 状态(亲验)**

**已存在、直接用**：`sleiman2021unified`(OCS2 统一 MPC) / `grandia2022perceptive`(感知 NMPC) / `bellicoso2016whole`(HoQp 分层 WBC) / `dicarlo2018cheetah` / `kim2019wbic`(WBIC) / `ding2021repfree`(几何 MPC) / `bledt2018cheetah3` / `orin2013centroidal` / `kajita2001lipm` / `raibert1986legged` / `wensing2024legged` / `unitree2023quadruped` / `rawlings2020mpc`。

**refs.bib 中缺失、写作前需新增的 bib 条目**(已 grep 确认不存在，写作 agent 须先补 refs.bib)：
- **RTI / Real-Time Iteration**(Diehl et al. 2002/2005，或 Gros 2020 "From linear to nonlinear MPC: RTI")——§frontier RTI 机理与 `quad_advanced` §5 必引。
- **HPIPM / acados**(Frison & Diehl 2020 HPIPM；Verschueren et al. 2022 acados)——QP 求解器串行地板论据。
- **Flayols 2017**(线性 KF 腿式估计，Humanoids 2017)——quad_est cheater→KF 节出处(注：书里 KF 出处现挂 Bloesch 2013，Flayols 是 legged_control 实际实现出处，可补)。
- **liao2023walking**(Liao IROS 2023 arXiv:2212.14199, Walking in Narrow Spaces, duality-CBF)——legged_control 安全约束应用层、本栈血缘出处(可选)。
- 提速菜单论文(arXiv:2310.20390 GNRK / optimization-online Multi-level RTI)——§frontier「RTI 之外」可作脚注引(可选)。

> ⚠ 写作 agent **先补 refs.bib 这 3 个必需键(RTI/HPIPM/Flayols)再写**，否则 `\cite` 编译断。

---

## 5. 写作分工建议（确保同一 .tex 不被两 agent 并行写）

> 原则：**一个 .tex 文件 = 至多一个写作 agent**，避免并行写冲突。新章独立文件可单独一 agent。refs.bib 由「整合 agent」串行补，或指定 W1 先补完再放行其余。

### 分波 + 文件归属（每文件唯一 owner）

| 波次 | Agent | 独占文件 | 任务 | 依赖 |
|---|---|---|---|---|
| **W0(串行前置)** | **bib-agent** | `refs.bib` | 补 3 个必需 cite 键(RTI/HPIPM/Flayols)+可选键 | 无；**必须先完成**，否则后续 `\cite` 断 |
| **W1(并行)** | **A-mpc** | `quadruped_mpc.tex` | §frontier 末加「质心 NMPC vs 凸 SRBD + RTI + ReferenceManager」子节 + §practice 补 Q 重标定 caveat。**只 `\cref` `eq:qp-nmpc`/`tab:three-models`/`ins:selection-matrix`，不重定义** | W0 |
| **W1(并行)** | **B-wbc** | `quad_wbc.tex` | §floating 末加「base 前馈vs反馈 + 42 维 QP + WeightedWbc/HoQp 落代码」+ §practice 补「力矩限读 task.info」陷阱。`\cref` `tab:qw-spectrum`/`ins:qw-unitree-balance` | W0 |
| **W1(并行)** | **C-est** | `quad_est.tex` | §practice 加「cheater→KF 门槛」子节 + §frontier 加「左右不对称估计先现形」insight。`\cref` `eq:qe-cov-inflate`/`ins:qe-inekf` | W0 |
| **W1(并行)** | **D-prac** | `quad_prac.tex` | §pitfalls 加 5 陷阱 + 补 `tab:qp-pitfalls`/故障排查手册行 + §deploy 路线图补 + §ecosystem 平衡控制器小补强 + 小经验登记。`\cref` `ins:qp-collision` | W0 |
| **W1(并行)** | **E-misc** | `quad_kin.tex` + `quad_gait.tex` + `quad_arch.tex` | 三处微补(kin: 零位/轴向不可复用 insight；gait: 重机 trot 周期 + 摆动 z 参数；arch: 前向指针到 ch:quad_advanced)。**三文件改动都极小**，合一个 agent 串行做 | W0；需知 `ch:quad_advanced` label 名(W2 定义，但 label 名本设计已固定) |
| **W2(可与 W1 并行)** | **F-adv** | `parts/P7_legged/quad_advanced.tex`(新建) + `parts/P7_legged/part.tex`(加 1 行 `\input`) | 写新章全 9 节 + 在 part.tex `quad_prac` 后加 `\input{parts/P7_legged/quad_advanced}` | W0；**part.tex 只此 agent 动** |

### 并行安全性校验

- **每个 .tex 文件只有一个 owner**：`quadruped_mpc`=A、`quad_wbc`=B、`quad_est`=C、`quad_prac`=D、`quad_kin`+`quad_gait`+`quad_arch`=E、`quad_advanced`+`part.tex`=F、`refs.bib`=W0。**无文件被两 agent 写** ✓。
- **W0 必须先于 W1/W2 完成**(cite 键依赖)。W1 的 A/B/C/D/E 与 W2 的 F **可全并行**(文件互不相交)。
- **label 命名已在 §4.2 全部固定**，跨 agent 引用(如 E-misc 的 arch 指针 `\cref{ch:quad_advanced}`、A-mpc 的 RTI 节被 F-adv §5 `\cref`)用约定 label 名即可，**无需运行时协调**。
- **建议**：W1/W2 全部完成后，由一个 **整合/编译 agent** 串行做：①`grep` 校验无重复 label；②编译 P7(或全书)修 `\cref`/`\cite` 悬空；③核对新章 `\input` 顺序。**编译是唯一收口门**。

---

## 附 · 料源关键 file:line 索引（写作 agent 回查用）

> 取自 `docs/Robot吸收_legged_control四足OCS2.md` 第四部分 + 附录；写作时如需更细回查源码行号见该 doc 末「附 · 关键 file:line 索引」。

- **数据流/频率**：料源 §1.2(三层频率图) / §2.5(LeggedController update 七步) → `quad_advanced` §1。
- **维度速查**：料源 §1.3(stateDim24/inputDim24/WBC 42 亲验 `WbcBase.cpp:24`) → quad_wbc `ins:qw-42dof`。
- **移植清单**：料源 §1.4(3 config+URDF / leg.xacro 不可复用) → quad_advanced §2 + quad_kin。
- **NMPC 层**：料源 §2.1(代价 Q/R / 平滑摩擦锥 / SqpMpc / sqpIteration=1 / 摆动 z 参数) → quadruped_mpc §frontier。
- **步态/ReferenceManager**：料源 §2.2(SwitchedModelReferenceManager / 模式编号 / GaitReceiver / trot 周期) → quadruped_mpc §frontier + quad_gait + quad_advanced §6.
- **WBC 层**：料源 §2.3(42 维各 task / base 加速度纯前馈 / WeightedWbc vs HoQp / braver 教训) → quad_wbc §floating。
- **估计**：料源 §2.4(线性 KF / cheater / 噪声三处落点) → quad_est §practice。
- **三大根因**：料源 §3.1(穿地)→quad_advanced §3+quad_prac；§3.2(左右不对称)→§4+quad_est；§3.3(sqpIteration)→§5+quadruped_mpc。
- **crutch+审计+残留悬坑**：料源 §3.4 → quad_advanced §7 + quad_wbc 力矩限陷阱。
- **PHASE3 两新根因**：料源 §3.10(关节阻尼 / 平地 REAL root) → quad_advanced §6 + quad_prac `pit:qp-cad-damping`。
- **方法论**：料源 §3.0 + §3.7 + §3.8 + §3.9 + §3.11 → quad_advanced §8+§9。
- **可吸收点总表**：料源第四部分表(10 项) = 本设计逐章映射的料源依据。
