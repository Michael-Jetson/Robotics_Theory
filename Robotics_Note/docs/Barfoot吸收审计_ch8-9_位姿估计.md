# Barfoot 位姿估计应用章（ch8+ch9）吸收审计 —— 扫描式 gap 报告

> 审计对象：Barfoot《State Estimation for Robotics》2nd ed 的两章**应用章**——
> `08_Pose_Estimation_Problems`（内部式号 **9.x**：点云配准/Wahba、点云跟踪 EKF+批量、位姿图松弛、惯性导航 SE₂(3)/预积分）与
> `09_Pose-and-Point_Estimation_Problems`（内部式号 **10.x**：束调整 BA、SLAM = BA+运动先验）。
> 这两章把前面的“李群 + 估计机理”落到实际 SLAM 问题（点云配准、位姿图、BA 全在流形上做）。
> 落点全树 grep 核查范围：`parts/`（重点 `P2_slam/{point_cloud_processing,visual_odometry,lidar_slam,vio,imu_model,loop_closure,camera_model,slam_system}.tex` + `P1_estimation/{nonlinear_optimization,slam_state_estimation,kalman_eskf}.tex`）。
> 记号映射：教材主线右扰动 $\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$、Hamilton 四元数、$\boldsymbol\Lambda$ 信息阵、$\boldsymbol\Sigma_{\mathbf w}/\boldsymbol\Sigma_{\mathbf v}$；Barfoot 左扰动、$\mathbf C$ 旋转、$\mathbf T$ 位姿。**右扰动版吸收左扰动结论判为合法吸收，不因左右差异报缺。**
> 标准 A（知识全量吸收：每条不可替代推导/例/定理/表/数值实验都应自包含存在）；标准 C（独立性：external_punt / ventriloquize / narration_dependence）。只报 confirmed_missing 与 partial_thin；found_elsewhere 已剔除并注明落点。本报告不与 ch1–5（`docs/Barfoot吸收审计.md`）、ch6/ch7 重复。

---

## ① 总体结论

**两章整体吸收度高（估计 ~80–85% 的不可替代严谨内容已落地并自包含），但应用形态被“重排”了：教材没有照搬 Barfoot 的“先讲 toy 标量问题、再上传感器”的章节骨架，而是把同一批数学打散进按传感器/工程系统组织的章里（点云配准→`point_cloud_processing`；BA/Schur→`nonlinear_optimization`+`camera_model`；位姿图→`nonlinear_optimization`+`loop_closure`；IMU 预积分→`imu_model`+`vio`；激光紧耦合滤波→`lidar_slam`）。** 这种重排本身符合“独立著作”原则，多数落点是高质量逐式重写。

**核心机理的覆盖盘点：**
- **点云配准 / Wahba（ch9.1）**：吸收度极高。SVD 闭式解（Arun/Kabsch，含 $\det$ 反射修正）、质心去中心化、平移=质心差、四元数特征值解（Horn）、右扰动迭代 GN、退化/唯一性直觉、ICP 全局最优性命题——全在 `point_cloud_processing.tex` 自包含展开。
- **束调整 BA（ch10.1）**：吸收度极高。$\mathbf g=\mathbf s\circ\mathbf z$ 复合观测、位姿雅可比 $\mathbf S(\mathbf T\mathbf p)^\odot$、路标雅可比 $\mathbf S\mathbf T\mathbf D$、箭头矩阵稀疏、Schur 补边缘化路标、Cholesky + 协方差恢复、Newton vs GN 丢弃二阶项、gauge 自由度——全部落地（`camera_model`+`nonlinear_optimization`），且保到二阶的均值修正 $\boldsymbol{\mathcal G}_j$ 也提到（`camera_model:1054`）。
- **SLAM = BA + 运动先验（ch10.2）**：吸收度极高。MAP=ML+先验、联合状态、$\mathbf A_{11}$ 块三对角 + $\mathbf A_{22}$ 块对角、因子图统一视角、toy 例（3 位姿 2/3 路标）、增量平滑 iSAM/Bayes 树（点到为止）——`slam_state_estimation.tex` 全量。
- **位姿图松弛（ch9.3）**：吸收度高但**形态偏“工程后端”**。目标函数 + 相对位姿残差 + 信息阵 + gauge 锚定 + 稀疏 Cholesky 在 `nonlinear_optimization:799-892`，回环边来源在 `loop_closure`。**缺 Barfoot 特有的两件：生成树初始化、约束链/悬臂链的链式稀疏分解。**
- **IMU 预积分（ch9.4 的核心目标）**：吸收度高，但**走的是 Forster SO(3)×ℝ³ 路线而非 Barfoot 的 SE₂(3) 路线**。噪声分离、协方差递推、偏置一阶修正（5 个偏置雅可比）、MAP 残差+解析雅可比、VINS-Mono 工程对照、AINS 不可观子空间——`imu_model.tex` 自包含且完整。**数学目标达成，但 SE₂(3) 这套群结构整支缺失（被显式 punt）。**

**两处 major 缺口（confirmed_missing，按重要性）：**
1. **SE₂(3) 扩展位姿群整支缺失**（Barfoot ch9.4.2/9.4.7：5×5 群、9×9 伴随、左雅可比、time machines）——`imu_model.tex:1045` 用 `\rebuilt{...}` 标记“$\mathrm{SE}_2(3)$ 预积分细节属不变滤波专题，**本章未展开**”，`vio.tex:1146` 同样只提一句。
2. **不变卡尔曼滤波（Invariant EKF, Barrau–Bonnabel）几乎零吸收**——全树仅 `vio.tex:994` 一行清单提“(右)不变卡尔曼滤波”，无群仿射/不变误差/状态无关雅可比的任何推导。注意：**“IEKF=迭代 EKF”≠“invariant EKF”**，教材的 IEKF（`kalman_eskf`、`lidar_slam` FAST-LIO）是迭代滤波，不能顶替 Barfoot 9.2 的 invariant EKF 小节。

**partial_thin（机理在、但缺 Barfoot 的特定推导/例）3 处：**
- 点云**跟踪**问题（ch9.2）缺 Barfoot 干净的“SE(3)-EKF / SE(3)-批量 跟踪一个观测已知点的动体”教学范式（含 $\mathbf G=\mathbf D^\top(\check{\mathbf T}\mathbf p)^\odot$ 点观测雅可比、五式 EKF 在 SE(3) 上的逐式）。
- 位姿图**生成树初始化** + **约束/悬臂链链式稀疏**（ch9.3.3–9.3.5）缺。
- Wahba **de Ruiter–Forbes 完整 case 分析**（ch9.1.3 的 $\det\mathbf W$ 符号分类 + 无穷多解 + 局部极小判定 + “三非共线点”充分条件）只给结论性命题，缺逐 case 推导。

**C 独立性整体合格**，核心推导自包含、`\cite` 多为合法出处与平衡对照；问题集中在上面两处 `\rebuilt` punt（SE₂(3)）与一行清单（invariant EKF），以及 BA/位姿图细节多处“完整见 \cref{...}”的章间转手（多数合法，个别需补“此处给出去向”而非空指）。详见 ③。

---

## ② 逐节 gap 清单

### ch9.1 点云配准 / Wahba 问题 → 主落点 `point_cloud_processing.tex`

**总判：A 近全量吸收，质量很高。** Barfoot 三参数化（四元数特征值 / 旋转矩阵 SVD / 变换矩阵迭代）全部对应：
- SVD 闭式解 `thm:pc-svd`：$\mathbf H=\sum\mathbf p_i'\mathbf q_i'^\top$、$\mathbf R=\mathbf V\,\mathrm{diag}(1,1,\det(\mathbf V\mathbf U^\top))\mathbf U^\top$（Arun/Umeyama/Kabsch），含反射 pitfall（`:645`）。
- 平移=质心差 $\mathbf t=\bar{\mathbf q}-\mathbf R\bar{\mathbf p}$、去中心化引理。
- 四元数特征值解 `thm:pc-quat`（Horn 最大特征值法，4×4 矩阵 $\mathbf N$），并证与 SVD 等价（`:584`）。
- 右扰动迭代 GN（本书主线，雅可比 $-\mathbf R\mathbf p^\wedge$）。
- ICP 收敛性 `thm:pc-convergence`（单调有界→局部极小）、全局最优性命题（唯一解情形任一极小=全局；共线/共面/点太少退化）。

**A 知识缺口：**
- **partial_thin — Wahba 唯一性的 de Ruiter–Forbes 完整 case 分析**。Barfoot 9.1.3（式 9.37–9.81）逐 case 给：$\det\mathbf W>0$ / $<0$ + 奇异值是否相异 / $\mathrm{rank}\,\mathbf W=2,1,0$ 的全部解结构（含无穷多解的 $\mathbf Y$ 自由参数族），并用二阶扰动 $\delta J$ 判定哪些临界点是极小/极大/鞍点，最后给“三非共线点”的 $\det\mathbf I\neq0$ 充分条件证明。教材只给**结论性**命题（“唯一解或无穷多解；唯一时无非全局局部极小；共线/共面退化”），**缺逐 case 的判别式推导与局部极小测试**。为何重要：这是 RANSAC 三点最小集、近共面退化“为什么会反射/不唯一”的严谨根基；目前 pitfall 给了现象（`:645,:648`）但没给 Barfoot 那套可判定的充要条件。建议落点：`point_cloud_processing.tex` SVD 小节后补一个 `\star` derivation 盒（可压缩为“$\det\mathbf W$ 符号 + 最小奇异值相异 + rank 三档”表 + 一句二阶判据）。
- **partial_thin — RANSAC 在“点配准 / 位姿求解”中的独立处理**。Barfoot 9.1.1 把闭式三点解明确放进 RANSAC 外点剔除语境。教材 `point_cloud_processing` 只在粗配准一句带过“FPFH+RANSAC”（`:649`），RANSAC 的迭代次数/内点阈值/最小集没有独立算法盒。**注意**：RANSAC 通用机理 found_elsewhere（已在估计核心审计标注于 `kalman_eskf` ch4 鲁棒小节），故此处仅记“与配准的结合点偏薄”，非整体缺失。

**C 独立性：** 合格。`point_cloud_processing` 开篇明示复用 `ch:lie` 右扰动 + `ch:nlopt` GN/LM 框架，配准误差“塞进这两章框架求解”——属合法 narration（声明吸收 + 自包含）。`barfoot2024state` / `gaoxiang2019slam14` 多书平衡对照。无 punt。

---

### ch9.2 点云跟踪（SE(3) 上 EKF + 批量 MAP）→ 落点散在 `lidar_slam.tex` / `kalman_eskf.tex` / `nonlinear_optimization.tex`

**总判：机理 found_elsewhere 但 partial_thin——缺 Barfoot 的“干净教学范式”。** Barfoot 9.2 是一个刻意简化的范式问题：动体观测**位置已知**的点 $\mathbf y_{jk}=\mathbf D^\top\mathbf T_k\mathbf p_j+\mathbf n$，分别用 (i) SE(3) 上 EKF（均值存李群、协方差存李代数，五式 9.141）和 (ii) SE(3) 上批量 GN（运动先验残差 $\ln(\boldsymbol\Xi_k\mathbf T_{k-1}\mathbf T_k^{-1})^\vee$ + 块三对角 $\mathbf A$）求解，并讨论 invariant EKF 关系。

教材里这套数学的**各零件都在**，但**没有这个统一的范式落点**：
- SE(3)/流形上 EKF 五式、均值李群更新 $\hat{\mathbf T}=\exp((\mathbf K\nu)^\wedge)\check{\mathbf T}$：`kalman_eskf.tex` 的 ESKF（名义+误差态、注入/重置、右扰动）覆盖了**机理**；`lidar_slam.tex` FAST-LIO 给了一个**更难**的实例（紧耦合迭代 ESKF、scan-to-map、新增益公式 `thm:lidar-gain-equiv`）。
- 批量 MAP + 块三对角 + 运动先验残差：`slam_state_estimation.tex`（线性高斯批量、块三对角来自马尔可夫性）+ `nonlinear_optimization.tex`（非线性 GN/LM、SE(3) 残差、Schur/Cholesky）覆盖。
- 左雅可比 $\mathcal J(-\mathbf e)^{-1}$ 进残差线性化：`nonlinear_optimization.tex:818-834`（用伴随把扰动搬一侧）+ `vio`/`imu` 右雅可比 $\mathbf J_r$。

**A 知识缺口：**
- **partial_thin — Barfoot 9.2 的点观测雅可比 $\mathbf G_{jk}=\mathbf D^\top(\check{\mathbf T}_k\mathbf p_j)^\odot$ 与“SE(3)-EKF 跟踪已知点”最小范式缺独立落点**。教材的 $\odot$ 观测雅可比是在 **BA/相机** 语境给的（`camera_model` 的 $\mathbf S(\mathbf T\mathbf p)^\odot$），“观测一个 3D 点在动体系坐标”这个更基础的 $\mathbf D^\top(\mathbf T\mathbf p)^\odot$（无相机非线性）没有单列。为何重要：它是激光/位置传感器最小观测模型，也是初学者理解“流形 EKF 怎么算雅可比”的最干净入口（比 FAST-LIO 简单一个量级）。建议落点：`kalman_eskf.tex` 或 `lidar_slam.tex` 开头补一个“在 SE(3) 上跟踪已知点”的 1 页范式盒，作为 FAST-LIO 的前置 toy。工作量小、教学收益高。

**C 独立性：** 合格。FAST-LIO 推导“逐行展开”、自带先验搬运 $\mathbf J^\kappa$ 的根因/对策/自检，未依赖原书。

---

### ch9.3 位姿图松弛（Pose-Graph Relaxation）→ 落点 `nonlinear_optimization.tex:799` + `loop_closure.tex`

**总判：A 吸收度高，但缺 Barfoot 特有的初始化与链式稀疏。** 位姿图目标函数 `eq:nlopt-pg-cost`（顶点=位姿、边=相对位姿约束）、相对位姿残差 $\mathbf e_{ij}=\ln(\mathbf T_{ij}^{-1}\mathbf T_i^{-1}\mathbf T_j)^\vee$、信息阵加权、gauge 自由度致 $\mathbf H$ 奇异 + `setFixed` 锚定（`:884`）、稀疏 Cholesky、回环边来源（`loop_closure` 全章）、质点-弹簧直觉（`ins:loop-spring`）——全部落地且自包含。

**A 知识缺口：**
- **partial_thin — 生成树初始化（spanning-tree initialization）缺**。Barfoot 9.3.3：从固定的 0 号节点沿生成树**复合**相对测量得各位姿初值（浅树优于深树以少累积不确定）。教材讲透了“里程计链累积漂移”（`loop_closure:87,95,98`）这个**问题侧**，但没把它翻成“生成树给位姿图 GN 的初值”这个**解侧**算法。为何重要：位姿图 GN 强依赖初值（`nonlinear_optimization:252` 已强调非凸+好初值），生成树是教科书级标准初始化。建议落点：`nonlinear_optimization.tex` 位姿图小节补半页（与 `:252` 初值讨论呼应）。
- **partial_thin — 约束链/悬臂链的链式稀疏分解缺**。Barfoot 9.3.4–9.3.5：把只有 1–2 条边的链节点分两类（constrained / cantilevered），先压缩链上相对测量、对 junction 节点跑小位姿图、再回填链节点；约束链的 $\mathbf A$ 块三对角，用稀疏 Cholesky + 前向-后向 pass，代价随链长线性，并给一个 0–5 号链的逐 $\mathbf U_{ij}$ Cholesky 块例（式 9.189–9.194）。教材有“稀疏 Cholesky/块三对角”通用机理（`slam_state_estimation`、`nonlinear_optimization`），但**没有位姿图链拓扑这个具体稀疏化例子**。重要性中等（通用稀疏求解器能自动吃掉这种稀疏，Barfoot 自己也这么说），但作为“位姿图为什么能线性时间”的具体例仍有教学价值。建议：可作 `\star` 选读例补于位姿图小节，或显式标注“通用 Schur/Cholesky 已覆盖，链分解从略”使 punt 合法化。

**C 独立性：** 基本合格。位姿图小节大量 `\cref{ch:nlopt}`/`\cref{ch:lidar_slam}` 章间指引，但都指向**本书已展开处**（非空指、非依赖外书），属合法 narration。

---

### ch9.4 惯性导航（SE₂(3) / IMU 预积分）→ 落点 `imu_model.tex` + `vio.tex`

**总判：数学目标（IMU 进滤波 + 进批量）达成且高质量，但参数化路线与 Barfoot 不同，且 SE₂(3) 整支被显式 punt。** Barfoot 9.4 用**扩展位姿群 SE₂(3)**（把 $\mathbf C,\mathbf r,\mathbf v$ 装进一个 5×5 群）统一推导 IMU 运动、均值/协方差传播、time machines、预积分。教材改用 **Forster 的 SO(3)×ℝ³ 误差态**路线，把同一目标做到了：
- IMU 测量+偏置+噪声模型（`imu_model:122`，含尺度/失准/g-sensitivity 扩展）。
- SO(3) 流形预积分核心三式、噪声分离（`derivation` 盒 ×2，`:434,:451`）。
- 迭代式协方差递推 $\boldsymbol\Sigma_{k+1}=\mathbf A\boldsymbol\Sigma\mathbf A^\top+\mathbf B\boldsymbol\Sigma_\eta\mathbf B^\top$（`:516`）+ 连续↔离散噪声（PSD/Allan，`:323,:790`）。
- **偏置一阶修正 + 全部 5 个偏置雅可比**（`:570-606`，避免重积分）——这正对应 Barfoot 9.4.8 perturbing-the-biases 的工程目标。
- 预积分 IMU 因子残差 + lift-solve-retract 解析雅可比（`:651-712`）、在线预积分算法盒、VINS-Mono 四元数预积分工程对照（`:1003`）。
- AINS 四维不可观子空间定理（`thm:imu-obs`）——对应 IMU+视觉的可观测性讨论。
- 进因子图 / VIO 紧耦合接口（`imu_model:954`、`vio` 全章 MSCKF/滑窗/structureless+Schur）。

**A 知识缺口（major）：**
- **confirmed_missing — SE₂(3) 扩展位姿群整支**。Barfoot 9.4.2（SE₂(3) 群、指数映射、9×9 伴随 `Ad`、5×5/9×9 逆、左雅可比 $\mathcal J$ 含 $\mathbf Q(\phi,\rho)$ 块）+ 9.4.7（time machines $\boldsymbol\Delta$ 群、$\mathcal D(\boldsymbol\Delta)$、把引力项合并的 $\mathbf T_{g,j:\ell}$ 恒等式）+ 9.4.3–9.4.5 的 SE₂(3) 版均值/协方差传播（$\mathbf N(\phi)$ 矩阵、$\bar{\boldsymbol\Phi}$ 转移）。教材 `imu_model:1045` 用 `\rebuilt{$\mathrm{SE}_2(3)$ 预积分细节属不变滤波专题，本章未展开}` 显式标注未做，`vio:1146` 同样一句带过。为何重要：SE₂(3) 是现代 VINS/不变滤波（IEKF）一致性改进的核心结构，也是 Barfoot 2nd ed 这一章的标志性新增内容；教材选了 Forster 路线把“预积分”讲全，但把 Barfoot 的“群论叙事”整支留白。**注意 `\rebuilt` 宏渲染为可见的橙色“[重建待核对：…]”脚注**——即这是作者已知的、显式标在 PDF 里的留白，不是隐藏缺口。建议落点：要么在 `imu_model` 或新的不变滤波小节补 SE₂(3) 群定义 + time machines（工作量中-大），要么把该 `\rebuilt` 改为一句合法的“本书走 SO(3)×ℝ³ 路线，SE₂(3) 等价路线见 Barfoot 9.4 / 不变滤波专题”定向 punt（工作量极小，先消除“待核对”观感）。
- **partial_thin — time machines 与“N(φ) 中点矩阵”** 即便不引入完整 SE₂(3)，Barfoot 用来精确离散化重力/中点加速度的 $\mathbf N(\phi)$ 矩阵（式 9.211）也未在教材出现（教材预积分用 Euler/中点近似，`vio:1144` 把“连续时间解析积分”列为改进方向但 `\cite` 外书）。重要性中等。

**C 独立性：** 预积分主体合格（自包含、derivation 盒齐、`lidar_slam:574` 明示“本章自包含展开”预积分）。**唯一 punt 在 SE₂(3)（见上，已显式标记）。** `carlone2026handbook`/`forster2017preintegration`/`barfoot2024state` 多书对照得当。

---

### ch10.1 束调整 BA → 落点 `camera_model.tex` + `nonlinear_optimization.tex`

**总判：A 全量吸收，质量最高的应用块之一。** 全部核心件落地：
- $\mathbf g=\mathbf s\circ\mathbf z$ 复合观测、链式雅可比（`camera_model:777,830` `thm` BA 投影雅可比）。
- **位姿雅可比 $\mathbf G_1=\mathbf S(\mathbf T\mathbf p)^\odot$（右扰动）+ 路标雅可比 $\mathbf G_2=\mathbf S\mathbf T\mathbf D$**，$\mathbf D$ dilation——精确对应 Barfoot 10.27。
- 箭头矩阵稀疏（`camera_model:921`、`nonlinear_optimization:567`，$\mathbf H_{cc}/\mathbf H_{pp}$ 块对角 + $\mathbf H_{cp}$ 耦合）。
- Schur 补先消点再解相机（`eq:schur-1`，复杂度 $O((K+M)^3)\to O(K^3+K^2M)$）。
- Cholesky 路线 + **协方差恢复 $\mathbf A^{-1}$**（`sec:nlopt-cov`，`:598` 明示 $\boldsymbol\Sigma_j=(\mathbf R_j^\top\mathbf R_j)^{-1}$ 是协方差恢复雏形；`camera_model:921` 指出“要协方差用 Cholesky 更合适”）。
- **Newton vs Gauss-Newton 丢弃的二阶项**（`nonlinear_optimization:30,282,293`，明说 GN 用 $\mathbf J^\top\mathbf W^{-1}\mathbf J$ 近似海森“丢掉哪一项”及强非线性+大残差失效）。
- gauge 自由度 / 固定 0 号位姿（`nonlinear_optimization:884`）。
- **保二阶均值修正 $\boldsymbol{\mathcal G}_j$**（`camera_model:1054`，对应 Barfoot 10.1.2 的二阶扰动模型）——连这个进阶件都吸收了。

**A 知识缺口：**
- **partial_thin — 位姿插值 BA 例（constant-velocity / $\mathbf T_1=\mathbf T^\alpha$）缺**。Barfoot 10.1.5：当路标太少/卷帘快门/边扫边动导致 $\mathbf A$ 不可逆时，假设匀速、用位姿插值 $\mathbf T^\alpha$（插值雅可比 $\mathcal A(\alpha,\boldsymbol\xi_{op})$、插值矩阵 $\mathbf I$）把自由位姿数降一、$\mathbf A'=\mathbf I^\top\mathbf A\mathbf I$ 仍是箭头矩阵。教材 `visual_odometry:739` 只一句“相机连续时可用匀速/不动假设作初值”，未给插值雅可比/插值矩阵/“插值后仍保稀疏”的推导。为何重要：这是卷帘快门/激光边扫边动 BA 的标准降维技巧，也是连续时间 BA 的桥。重要性中等。建议落点：`nonlinear_optimization` BA 小节或 `vio` 连续时间附录补一个 `\star` 例。

**C 独立性：** 合格。BA 推导自包含；`camera_model:921` 末“（`\cref{ch:nlopt}`、`\cref{ch:vo}` 展开）”指向本书已展开处。

---

### ch10.2 SLAM = BA + 运动先验 → 落点 `slam_state_estimation.tex`

**总判：A 全量吸收。** 全部件落地：
- **概念关系**：MAP=ML+先验（`:558`），BA 无先验=ML、SLAM 加运动先验=MAP（`:631-637`）。
- 联合状态 $\{$位姿, 路标$\}$、堆叠正规方程 $\mathbf H^\top\mathbf W^{-1}\mathbf H\hat{\mathbf x}=\mathbf H^\top\mathbf W^{-1}\mathbf z$、信息阵=逆协方差=海森=Fisher（`:672-725`）。
- 加运动先验**不破坏箭头稀疏**：$\mathbf A_{11}=\mathbf F^{-\top}\mathbf Q^{-1}\mathbf F^{-1}+\mathbf G_1^\top\mathbf R^{-1}\mathbf G_1$ 块三对角 + $\mathbf A_{22}$ 块对角（`:803-826` + `nonlinear_optimization:567`）。
- **因子图统一视角**（`:884-985`，二部图、因子=负对数似然项、prior/odometry/motion/observation/loop 因子表），含定理“因子图 MAP=非线性最小二乘”。
- toy SLAM 例（3 位姿 + 2/3 路标、1D 小车，`:775-900`）。
- 增量平滑 iSAM/iSAM2/Bayes 树（`:1025-1032`，点到点、给去向 GTSAM）。
- 可观测性/gauge：BA 固定 $\mathbf T_0$ vs SLAM 加先验估计 $\mathbf T_0$（`:728-800`，存在唯一性=可观测性=$\boldsymbol\Lambda$ 可逆，质点弹簧直觉）。

**A 知识缺口：** 无 confirmed_missing。`slam_state_estimation` 对 ch10.2 的概念骨架吸收完整。

**C 独立性：** 合格且优秀。`:71` 明示因子图/连续时间/iSAM“点到点、给出去向”，滤波/求解器“刻意推迟”到 `ch:eskf`/`ch:nlopt`——属合法 narration 与本书内部分工，非依赖外书。

---

## ③ 独立性问题（标准 C 汇总）

**整体合格。** 两章应用数学的核心推导都做到了自包含逐式重写，`\cite{barfoot2024state,gaoxiang2019slam14,carlone2026handbook}` 多为合法出处标注与多书平衡对照。需处理的具体项：

1. **external_punt（需修，1 处 major）— SE₂(3)**：`imu_model.tex:1045` 的 `\rebuilt{$\mathrm{SE}_2(3)$ 预积分细节属不变滤波专题，本章未展开}` + `vio.tex:1146` 一句。`\rebuilt` 宏（`common/preamble.tex:22`）渲染为可见橙色“[重建待核对：…]”，当前是“待核对的留白”观感。**最低成本**：把它改写成一句完整的定向 punt（“本书取 Forster 的 SO(3)×ℝ³ 等价路线；SE₂(3) 群论路线见 Barfoot 9.4 与不变滤波专题”）并去掉 `\rebuilt`，即从“待核对缺口”升级为“合法 `\cite` + 本书口吻的去向交代”。**完整解**见 ④。

2. **narration_dependence（轻微，需补一句去向）— invariant EKF**：`vio.tex:994` 清单式“(右)不变卡尔曼滤波、迭代 EKF：用群结构改善滤波一致性”——这是“提名词不展开也不给去向”的轻度依赖。建议补半句：要么“详见不变滤波专题/Barrau–Bonnabel”，要么干脆在 `kalman_eskf` 加一个不变 EKF 小节（见 ④ 第 2 项）。**勿与教材已充分展开的 IEKF（迭代 EKF）混淆**——后者在 `kalman_eskf`/`lidar_slam` 是自包含的，无独立性问题。

3. **narration（合法，无需改）— 大量章间 `\cref`**：位姿图（`nonlinear_optimization` ↔ `loop_closure` ↔ `lidar_slam`）、BA（`camera_model` ↔ `nonlinear_optimization` ↔ `visual_odometry`）、预积分（`imu_model` ↔ `vio`）之间的相互指引密集，但抽查均指向**本书已展开处**（非空指、非依赖外书），符合“按系统重排”的著作结构。仅 `point_cloud_processing` 个别“完整系统见 `\cref{ch:lidar_slam}`”属正常分工。

4. **ventriloquize（轻微，可选润色）**：`camera_model:1054`“Barfoot 给出保二阶的均值修正”、`vio:1144`“较标准预积分精度 ≥1 个数量级\cite{...}”等处以原书/外书口吻陈述结论。因紧跟自包含推导或明确 `\cite`，属可接受范围；若追求纯本书口吻可改为“可证…”“本书在此给出…”。

---

## ④ 最该补的 3–5 项（按 贡献 × 工作量 排序）

1. **【高贡献 / 极低工作量】把 SE₂(3) 的 `\rebuilt` punt 改为合法定向 punt**（`imu_model:1045`、`vio:1146`）。一句话交代“本书走 SO(3)×ℝ³ 等价路线、SE₂(3) 群论路线见 Barfoot 9.4 / 不变滤波专题”，删掉 `\rebuilt`。**立即消除 PDF 里可见的“[重建待核对]”观感与 external_punt 判定**，且不需要写新数学。优先做。

2. **【高贡献 / 中工作量】补一个“在 SE(3) 上跟踪已知点”的最小流形 EKF 范式**（落 `kalman_eskf` 或 `lidar_slam` 开头，~1 页）。给出 $\mathbf y=\mathbf D^\top\mathbf T\mathbf p+\mathbf n$、点观测雅可比 $\mathbf G=\mathbf D^\top(\check{\mathbf T}\mathbf p)^\odot$、SE(3)-EKF 五式（均值李群更新、协方差李代数），作为 FAST-LIO 紧耦合迭代 ESKF 的前置 toy。**补齐 Barfoot 9.2 最干净的教学范式，且是初学者理解“流形 EKF 雅可比”的最佳入口；与现有 `camera_model` 的 $\odot$ 雅可比、`lidar_slam` 的 FAST-LIO 直接衔接。**

3. **【中-高贡献 / 中工作量】补 invariant EKF（Barrau–Bonnabel）小节**（落 `kalman_eskf`，~1–2 页）。群仿射系统、左/右不变误差、状态无关的 $\mathbf F$/$\mathbf G$、作为稳定观测器的结论；并显式区分“invariant EKF ≠ iterated EKF（IEKF）”。**这是 Barfoot 9.2 与现代 VINS 一致性改进的核心，目前全树仅一行清单；补上同时解决独立性 narration_dependence 与 SE₂(3) 的语境（两者同属不变滤波专题）。** 可与第 1 项合并为一个“不变滤波专题”小节，一举吸收 SE₂(3)+invariant EKF 两个 major 缺口。

4. **【中贡献 / 低-中工作量】补位姿图生成树初始化**（落 `nonlinear_optimization` 位姿图小节，~半页）。从固定节点沿浅生成树复合相对测量得初值；与 `:252` 的“非凸+好初值”讨论、`loop_closure` 的“里程计链漂移”呼应闭环。**把已讲透的“漂移问题”补上“初始化解法”，教学闭环。**

5. **【中贡献 / 中工作量】补 Wahba 唯一性的判别式表 + 局部极小判据**（落 `point_cloud_processing` SVD 小节，一个 `\star` derivation 盒）。把 de Ruiter–Forbes 的 $\det\mathbf W$ 符号 / 最小奇异值相异 / rank 三档压成一张“何时唯一、何时无穷多解”表，加一句二阶 $\delta J$ 判据与“三非共线点”充分条件。**为现有 ICP 全局最优性命题与近共面反射 pitfall 提供严谨充要根基。**

（备选 / 低优先：位姿图约束链-悬臂链链式稀疏例（ch9.3.4–9.3.5）、BA 位姿插值 $\mathbf T^\alpha$ 例（ch10.1.5）、IMU 预积分 $\mathbf N(\phi)$ 中点矩阵精确离散化——这三项通用机理已被覆盖，属“具体例补全”，可作 `\star` 选读，或显式标“通用方法已覆盖、特例从略”使 punt 合法化。）
