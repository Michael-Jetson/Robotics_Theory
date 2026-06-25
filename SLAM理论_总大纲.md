# SLAM 理论总大纲（视觉 · 激光 · 惯性）— 一站式

> 单一自包含文档。汇总：另一 Agent 的两份 v1 研究（理论母树 + 标定体系，完整原文）、两次 Workflow 补遗（理论 131 条 + 标定 122 条，完整全文）、评审补充。
> 配套教材 MD（同目录）：`视觉SLAM十四讲_md/`(中·缓坡15章) · `Barfoot_SER2_md/`(估计圣经16章) · `SLAM_Handbook_md/`(现代+前沿20章)。
> 全文结构：本导览(大纲+研读路线) → 第I部 理论母树(完整) → 第II部 标定体系(完整) → 第III部 理论补遗131 → 第IV部 标定补遗122。

## A. 组织原则
**SLAM = 几何建模 + 概率状态估计 + 非线性优化/滤波 + 传感器物理模型 + 数据关联/鲁棒性。**
母树：根=状态估计；主干=Lie群·概率·优化·因子图(+连续时间·可证最优)；枝=视觉前端·LiDAR配准·IMU预积分·VIO/LIO/LVI·标定·回环·建图。
学习主线（纵切优于横切）：`状态估计 → Lie与优化 → 三模块测量模型 → VIO/LIO → LVI → 标定/可观性/退化/鲁棒 → 前沿桥`。

## B. 分层母树速查（骨架，详见第I部+第III部）
| 层 | 作用 | 书锚点 | 核心(KNOWN) | 新补支柱[新] |
|---|---|---|---|---|
| L0 数学地基 | 最小二乘 HΔx=-b 底层 | 十四讲2;Barfoot1-2,附A | linalg/Schur/MLE-MAP/Fisher/NEES | 鲁棒理论根(Black-Rangarajan/Barron) |
| L1 Lie/三维运动 | 流形上残差与Jacobian | 十四讲3-4;Barfoot6-7;Solà micro-Lie | exp/log/Adjoint/左右Jac/BCH/⊞⊟ | SE(3)高斯/B样条Lie导数 |
| L2 滤波 | p(x_k\|z) 实时 | 十四讲9;Barfoot3-4;Handbook2 | EKF/ESKF/MSCKF/IEKF/InEKF/OC-EKF | [新]不变-等变(InEKF/EqF/EqVIO) |
| L2 优化/因子图 | p(x_{0:k},m\|z) | 十四讲6,10;Barfoot8-9;Handbook1;Dellaert-Kaess | GN/LM/BA/PoseGraph/iSAM2/Schur/gauge | BA Modern Synthesis |
| L2+ 连续时间[新] | B样条/GP轨迹 | Barfoot10 | — | Furgale/STEAM-GP/样条/CT综述 |
| L2+ 可证最优[新] | 凸松弛+证书 | Handbook6 | — | SE-Sync/对偶/旋转平均/CORA/STRIDE |
| L3 视觉 | 图像→运动+结构 | 十四讲5,7,8;Handbook7 | pinhole/对极/PnP/BA/特征-直接/Sim3 | MVG/5点/ORB-SLAM3/学习前端 |
| L3 惯性 | 高频动力学+时间参考 | Handbook11;Barfoot3 | IMU模型/预积分(Forster)/ESKF/init/VIO | Solà ESKF/Martinelli闭式/Galilean预积分 |
| L3 激光 | 点云→位姿+地图 | 十四讲12;Handbook8 | ICP/NDT/点到面/LOAM/FAST-LIO2/deskew | GICP/KISS-ICP/SuMa++/配准monograph |
| L4 融合 | VIO/LIO/LVI 紧耦合 | 十四讲9-11;Handbook1;LVI-SAM | 紧松耦合/滑窗/marg | Gaussian-LIC/Kimera/Khronos |
| L6 地图表示[新] | 经典→神经 | Handbook5,14 | 特征/点/surfel/mesh | OctoMap→Voxblox→iMAP→SplaTAM/MonoGS |
| L7 回环+关联[新] | 全局一致入口 | 十四讲11;Handbook1 | loop/Sim3/PCM | DBoW2/NetVLAD/Scan-Context/JCBB/ROBIN |
| L8 可观性/退化/一致性 | 研究分水岭 | Barfoot4;Handbook2,6 | observability/gauge | FEJ/OC-EKF根(Huang IJRR10) |
| L9 鲁棒估计 | 真实场景必需 | Handbook3 | RANSAC/Huber/switchable/DCS/PCM | GNC/TEASER/Estimation-Contracts |
| L10 前沿桥[新] | 通往前沿 | Handbook4,13-18 | — | DROID/Theseus/DUSt3R/VGGT/MASt3R-SLAM |

## C. 标定垂直专题速查（依赖 L0–L4，挂 L4 后；详见第II/IV部）
坐标系：`标定 = 内参 + 空间外参 + 时间外参 + 在线自标定 + 可观性/退化`。优先级=精读序：**Camera-IMU → LiDAR-IMU → LiDAR-Camera → LVI 联合**(GNSS后置)。
8类缺口(评审+Workflow印证)：手眼AX=XB · 可观性根/FEJ/OC-EKF · 退化检测(Zhang ICRA16) · 不确定性传播 · 主动激励 · 时间同步硬件 · 单传感器内参 · 新兴传感器(event/4D-radar/thermal)。

## D. 统一研读路线（阶段 0–6）
- **0 数学&Lie地基**：Barfoot 1-2,6-7 + 十四讲2-4 → Solà micro-Lie；目标 SO(3)/SE(3)/exp/log/Adjoint/Jr/Jl。`[码]`Sophus/manif。
- **1 估计引擎**：Barfoot 3-4,8-9 + Handbook1。滤波线 EKF→ESKF→MSCKF→InEKF(Barrau TAC17)→EqF→EqVIO/MSCEqF→统一(Automatica25)；一致性 Huang FEJ/OC-EKF(IJRR10)。优化线 GN/LM→BA(Modern Synthesis)→PoseGraph→iSAM2→可证最优(对偶→SE-Sync→旋转平均→CORA/STRIDE)。`[码]`GTSAM/Ceres 手写 PGO→BA。
- **1+ 连续时间**：Barfoot10 → Furgale→STEAM-GP→样条(Sommer/basalt)→CT综述。`[码]`STEAM/basalt-headers。
- **2 三模块测量**：Handbook7/8/11 + 十四讲5,7,8,12。视觉 MVG→5点→ORB-SLAM3→SuperPoint/Glue；惯性 Solà ESKF→Forster预积分→Martinelli init；激光 NDT/GICP/点到面→KISS-ICP→SuMa++。
- **3 融合**：Handbook1 + 十四讲9-11。OKVIS/VINS(VIO)→LIO-SAM/FAST-LIO2(LIO)→LVI-SAM→Gaussian-LIC。`[码]`复现一个 VIO/LIO。
- **4 标定垂直**：Camera-IMU(Li-Mourikis→Qin-Shen→Kalibr;补 hand-eye init + OC-EKF/FEJ)→ LiDAR-IMU(LI-Calib/OA-LICalib/LI-Init)→ LiDAR-Camera → LVI 联合。退化/可观性贯穿。详见第II/IV部。
- **5 横切强化**：可观性/退化(L8)、鲁棒理论(L9:Black-Rangarajan→GNC→TEASER)、地图表示(L6)、回环+关联(L7)。
- **6 前沿桥**：Handbook4,13-18。DL-SLAM综述→端到端(DROID/DPVO)→可微(Theseus/gradSLAM)→神经地图(iMAP→NICE-SLAM→SplaTAM)→基础模型(DUSt3R→VGGT→MASt3R-SLAM)→空间AI(场景图/HOV-SG)。

## E. 溯源
第I部=另一Agent理论母树v1完整原文；第II部=另一Agent标定体系v1完整原文；第III部=Workflow `slam-theory-gap-resource-scout`(131条·联网+对抗核验)；第IV部=Workflow `calib-gap-resource-scout`(122条)。导览(本节)=两者合并+评审补充。
> 注：第III部教材/课程类条目(理论Workflow `verify:edu` facet核验失败)与少数2026预印本，用前自查。

## F. 评审要点与修订（本会话分析补入）
**理论母树小修（针对第I部）**：
- DSO 本体 = Engel《Direct Sparse Odometry》**arXiv:1607.02565**；第I部 [5] 挂的 1804.05625 实为 VI-DSO，分清。
- OC-EKF 的孪生 = **FEJ-EKF**(首次估计雅可比)，一致性双子；第I部"Invariant-EKF/OC-EKF" 与第III部 §C(不变-等变)、§F(一致性) 应**合并研读**——FEJ/OC-EKF 与 InEKF/EqF 是解同一"伪可观/不一致"问题的两条路线(雅可比固定 vs 几何对称)，Automatica 2025(arXiv:2309.03765) 给统一视角。
- 开篇总纲补 **Cadena《Past/Present/Future of SLAM》TRO2016**(arXiv:1606.05830) 做全景坐标系。

**标定评审（针对第II部）**：评审 v1 得出的 7 类缺口 = 第IV部 Workflow 独立跑出的 8 类(手眼AX=XB · 可观性-FEJ/OC-EKF · 退化检测 · 不确定性传播 · 主动激励 · 时间同步硬件 · 单传感器内参 · 新兴传感器)——**两路独立印证**，缺口判断稳。标定理论三线锚到教材：连续时间→Barfoot ch10+Furgale ICRA12；Lie 雅可比(推 t_d)→Barfoot ch7+Solà micro-Lie；因子图 MAP→Handbook ch1。

**方法论(硬指标)**：每层配代码练习(GTSAM/Ceres 手写 PGO→BA→IMU预积分因子→点到面)，理论才焊牢；第I部 §14 自述"能自己写简单 pose graph/BA"即此意。



---

# 第 I 部 · 理论母树（地基→引擎→模块→融合）· 完整原文

## SLAM 理论母树 v1（来源：另一 Agent 的研究成果 · 完整原文存档）

> 存档说明：另一 Agent 产出的「视觉-激光-惯性 SLAM 理论母树」第一版**完整原文**（公式从复制粘贴的渲染态清理为可读 LaTeX/代码，内容未删减）。这棵树的根是状态估计，主干是李群、概率估计、滤波、优化和因子图，分支才是视觉前端、LiDAR 配准、IMU 预积分、多传感器融合、标定、回环和建图。

一句话概括：

$$\text{SLAM} = \text{几何建模} + \text{概率状态估计} + \text{非线性优化/滤波} + \text{传感器物理模型} + \text{数据关联与鲁棒性}$$

对视觉、激光、惯性三类模块来说，可以先按下面这个体系学习。

---

### 1. 第一层：SLAM 的数学地基

#### 1.1 线性代数与数值计算

这是所有 SLAM 的底层工具。重点不是普通矩阵运算，而是这些内容：

| 内容 | 在 SLAM 中的作用 |
|---|---|
| 矩阵分解 | QR、Cholesky、SVD 用于求解最小二乘 |
| 稀疏矩阵 | 大规模图优化、BA、Pose Graph 的计算核心 |
| 特征值/奇异值 | 退化检测、可观性分析、Hessian rank 判断 |
| Schur Complement | Bundle Adjustment 消元 landmark 的核心 |
| 条件数 | 判断优化问题是否病态 |

现代 SLAM 后端本质上经常要解：

$$H \Delta x = -b, \quad H = J^\top W J, \quad b = J^\top W r$$

所以你必须理解 Jacobian、Hessian、信息矩阵、协方差、稀疏求解之间的关系。

#### 1.2 概率论与估计理论

SLAM 不是"几何匹配后直接算位姿"，而是带噪声测量下的状态估计。核心目标是 $p(x_{0:k}, m \mid z_{1:k}, u_{1:k})$，或对应的 MAP 优化 $x^\star = \arg\max_x p(x \mid z)$，常见形式：

$$x^\star = \arg\min_x \sum_i \|r_i(x)\|^2_{\Sigma_i^{-1}}$$

你需要掌握：

| 概念 | 为什么重要 |
|---|---|
| MLE / MAP | 滤波和优化的统一来源 |
| Gaussian noise | 为什么残差用二范数加权 |
| covariance / information matrix | 不确定性传播、传感器权重 |
| Fisher information | 可观性、信息量选择 |
| NEES / NIS | 滤波一致性检验 |
| robust estimation | 抗外点、动态物体、错误回环 |

Li & Mourikis 的 camera–IMU 时间标定论文就是典型例子：他们不是简单估一个时间偏移，而是把 $t_d$ 作为状态变量，并通过 EKF 协方差显式表达其不确定性和对运动估计的影响。

---

### 2. 第二层：李群李代数与三维刚体运动

视觉、激光、惯性 SLAM 都绕不开 $SO(3)$（三维旋转）、$SE(3)$（三维刚体位姿）、$Sim(3)$（带尺度相似变换，单目 SLAM 回环）。

你要掌握的不只是"指数映射怎么写"，而是这些概念在估计里的作用：

| 内容 | 用途 |
|---|---|
| $\exp(\cdot), \log(\cdot)$ | 在流形上做扰动和残差 |
| hat / vee | 向量和李代数矩阵之间转换 |
| Adjoint | 坐标系变换下扰动传播 |
| left/right perturbation | 决定 Jacobian 符号和误差定义 |
| left/right Jacobian | IMU 预积分、姿态协方差传播 |
| $\boxplus,\boxminus$ | Ceres/GTSAM 中的 manifold optimization |
| BCH 近似 | 小扰动组合和误差传播 |

典型位姿残差 $r_{ij} = \log(T_{ij}^{-1} T_i^{-1} T_j)$。IMU 预积分、LiDAR scan-to-map、camera reprojection、外参标定，本质上都需要在 $SE(3)$ 上定义残差和 Jacobian。Barfoot《State Estimation for Robotics》把状态估计、三维运动表示和应用放在同一框架下，是机器人状态估计的系统教材；Solà 等《A micro Lie theory for state estimation in robotics》则是专门面向机器人状态估计的李群入门材料。[1]

---

### 3. 第三层：滤波与优化两条主线

SLAM 状态估计主要有两大范式：Filtering vs. Smoothing/Optimization。

#### 3.1 滤波路线

典型路径：$KF \rightarrow EKF \rightarrow ESKF \rightarrow MSCKF \rightarrow IEKF \rightarrow \text{Invariant EKF}$。滤波方法关注当前状态 $p(x_k \mid z_{1:k})$，常见于实时、高频、资源受限系统。

| 方法 | 重点 |
|---|---|
| EKF | 线性化、协方差传播、Kalman update |
| Error-State EKF | 姿态、速度、位置、bias 的误差状态表示 |
| MSCKF | 不把 landmark 长期放入状态，用多帧约束更新相机/IMU 状态 |
| Iterated EKF | 反复线性化，提升非线性测量更新效果 |
| Invariant EKF | 利用群结构改善一致性 |
| Observability-Constrained EKF | 显式保持不可观子空间，避免虚假信息 |

Li & Mourikis 属于 EKF/MSCKF 路线：把 IMU pose、velocity、bias、camera–IMU 外参、时间偏移和特征状态放入估计框架，并分析时间偏移的可辨识性。

#### 3.2 优化路线

典型路径：$\text{least squares} \rightarrow GN/LM \rightarrow BA \rightarrow \text{Pose Graph} \rightarrow \text{Factor Graph} \rightarrow iSAM2$。优化方法关注一段甚至全局轨迹 $p(x_{0:k},m \mid z_{1:k})$。代表式：

$$\min_X \sum_i \rho_i\!\left(\|r_i(X)\|^2_{\Sigma_i^{-1}}\right)$$

| 内容 | 用途 |
|---|---|
| Gauss–Newton | 非线性最小二乘的基本方法 |
| Levenberg–Marquardt | 更稳健的非线性优化 |
| marginalization | 滑窗优化中移除旧状态 |
| prior factor | 被边缘化信息转成先验 |
| Schur complement | BA 中消元地图点 |
| robust kernel | 抵抗误匹配和异常值 |
| gauge freedom | 处理全局平移/yaw/scale 不可观 |
| sparse Cholesky / QR | 大规模稀疏系统求解 |

Qin & Shen 的 VIO 时间标定属于优化路线：把时间偏移 $t_d$ 增广进滑窗状态，并通过带时间偏移的视觉重投影因子联合优化 camera/IMU 状态、特征和 $t_d$。

---

### 4. 第四层：因子图是现代 SLAM 的统一语言

现在很多系统都可以统一写成因子图 $p(X \mid Z) \propto \prod_i \phi_i(X_i)$，对应优化：

$$X^\star = \arg\min_X \sum_i \|r_i(X_i)\|^2_{\Sigma_i^{-1}}$$

不同传感器只是不同 factor：

| 传感器/约束 | 因子类型 |
|---|---|
| 相机 | reprojection factor / photometric factor |
| IMU | preintegration factor |
| LiDAR | point-to-plane / point-to-line / scan-to-map factor |
| GNSS | global position factor |
| wheel | velocity / nonholonomic factor |
| 回环 | relative pose / Sim(3) factor |
| 标定 | extrinsic / time-offset factor |

GTSAM 文档把 factor graph 描述为适合 SLAM/SfM 等复杂估计问题的二分图，变量表示未知量，factor 表示来自测量或先验的概率约束。[2] iSAM2 用 Bayes tree 建立增量平滑和稀疏矩阵分解之间的联系，是现代增量式图优化 SLAM 的核心文献之一。[3]

---

### 5. 第五层：视觉 SLAM 理论基础

视觉模块主要回答：图像 → 相机运动 + 地图结构。

#### 5.1 相机模型与多视几何

| 内容 | 作用 |
|---|---|
| pinhole camera model | 投影函数 $\pi(\cdot)$ |
| distortion model | 畸变校正和相机内参 |
| epipolar geometry | 两帧匹配、Essential/Fundamental matrix |
| triangulation | 从多帧观测恢复 3D 点 |
| PnP | 已知 3D–2D 匹配求相机位姿 |
| bundle adjustment | 联合优化相机位姿和地图点 |
| scale ambiguity | 单目视觉的尺度不可观 |

典型视觉重投影残差 $r_C = z_{ij} - \pi(T_{CI} T_{IW_j} P_i)$。

#### 5.2 特征法与直接法

| 路线 | 残差 | 代表 |
|---|---|---|
| 特征法 | 几何重投影误差 | ORB-SLAM |
| 直接法 | 光度误差 | LSD-SLAM / DSO |
| 半直接法 | sparse direct tracking + feature-like mapping | SVO |

ORB-SLAM 是特征法视觉 SLAM 的经典系统，强调同一套 ORB 特征用于 tracking、mapping、relocalization 和 loop closing。[4] DSO 代表直接稀疏法路线，核心是选择有梯度的像素并最小化光度误差；VI-DSO 则把直接视觉误差和 IMU 预积分结合。[5] 学习重点不是只会跑 ORB-SLAM，而是理解：特征匹配 → 几何约束 → BA → 回环 → 全局一致地图。

---

### 6. 第六层：IMU 与惯性导航理论基础

IMU 是 VIO、LIO、LVI 的动力学骨架。它提供高频短时运动约束，但会随时间漂移。

#### 6.1 连续时间 IMU 模型

$$\omega_m = \omega + b_g + n_g, \qquad a_m = R^\top(a-g) + b_a + n_a$$

其中 $b_g,b_a$ 是陀螺仪/加速度计 bias，$n_g,n_a$ 是白噪声，bias 通常建模为 random walk。IMU 状态 $x_I = \{R,p,v,b_g,b_a\}$。Li & Mourikis 采用类似 IMU 状态（姿态、位置、速度、陀螺 bias、加速度 bias）。

#### 6.2 IMU 预积分

IMU 频率很高，不能把每个测量都作为优化变量。VIO/LIO 通常把两个关键帧之间的 IMU 测量压缩成一个 preintegration factor：$\Delta R_{ij},\ \Delta v_{ij},\ \Delta p_{ij}$。Forster 等的 on-manifold preintegration 在旋转流形上推导 IMU 预积分，把预积分测量作为 factor graph 中的相对运动约束。[6]

| 内容 | 作用 |
|---|---|
| strapdown integration | 从 IMU 积分得到姿态、速度、位置 |
| preintegration | 降低优化变量数量 |
| bias correction | bias 变化后快速修正预积分 |
| covariance propagation | 给 IMU factor 正确权重 |
| gravity alignment | 初始化姿态、尺度、重力 |
| scale observability | 单目 VIO 为什么能恢复尺度 |
| IMU excitation | 运动不足时初始化失败 |

---

### 7. 第七层：LiDAR SLAM 理论基础

LiDAR 模块主要回答：点云 → 位姿 + 几何地图。

#### 7.1 点云配准

核心问题 $T^\star = \arg\min_T \sum_i d(Tp_i, \mathcal{M})^2$。

| 残差 | 用途 |
|---|---|
| point-to-point | ICP 基础形式 |
| point-to-plane | 地面车、室内结构常用 |
| point-to-line | LOAM 边缘特征 |
| point-to-surfel | 局部面片地图 |
| point-to-distribution | NDT |
| point-to-map | scan-to-map LIO 主流 |

点到面残差 $r_L = n^\top (Tp_i - q)$，其中 $q,n$ 是地图中的局部平面点和法向。

#### 7.2 LiDAR 运动畸变

LiDAR 一帧点云不是同时采集的，一个 scan 中每个点都有自己的采样时刻 $p_i = p(t_i)$。运动较快不做 deskew 会导致点云扭曲。LOAM 指出 3D LiDAR 点在不同时间接收，运动估计误差会导致点云错配；其核心思想是把复杂 SLAM 拆成高频 odometry 和低频 mapping 两个线程。[7]

#### 7.3 LiDAR SLAM 代表路线

| 路线 | 代表 | 理论重点 |
|---|---|---|
| LOAM 系列 | LOAM / A-LOAM / LeGO-LOAM | edge/plane feature、scan-to-map |
| factor graph LIO | LIO-SAM | IMU preintegration + LiDAR factor + loop factor |
| EKF LIO | FAST-LIO / FAST-LIO2 | iterated EKF、direct point-to-map |
| continuous-time LIO | CT-ICP / LI-Calib 类 | B-spline/GP trajectory、deskew |
| dense/surfel map | SuMa / surfel-based LIO | surfel map、projective association |

LIO-SAM 把 LiDAR–inertial odometry 建在 factor graph 上，可融合相对/绝对测量、回环等不同 factor，用 IMU preintegration 去畸变并给 LiDAR odometry 提供初值。[8] FAST-LIO2 代表高效 EKF 路线，直接把原始点注册到地图，不依赖手工边缘/平面特征，用 ikd-Tree 维护增量地图。[9]

---

### 8. 第八层：视觉–惯性 VIO 理论基础

VIO 是视觉和 IMU 的互补融合：

| 视觉 | IMU |
|---|---|
| 长期几何约束强 | 短时运动约束强 |
| 低频 | 高频 |
| 单目尺度不可观 | 提供尺度和重力 |
| 易受纹理/光照影响 | 易受 bias 漂移影响 |

VIO 状态 $x_k = \{R_k,p_k,v_k,b_{g,k},b_{a,k}\}$，加上 landmark、外参、时间偏移 $\{P_i,T_{CI},\Delta t_{CI}\}$。后端典型代价：

$$\min_X \sum \|r_C\|^2_{\Sigma_C^{-1}} + \sum \|r_I\|^2_{\Sigma_I^{-1}} + \|r_p\|^2_{\Sigma_p^{-1}}$$

OKVIS 是关键帧非线性优化 VIO 的代表，提出严格概率意义下视觉重投影误差和惯性项联合 cost function。[10] Qin & Shen 显示同样的滑窗 VIO 框架可通过增广 $t_d$ 完成在线时间标定。VIO 必学：IMU 预积分；单目初始化；尺度/重力/bias 可观性；滑窗优化；marginalization；MSCKF nullspace projection；feature-based vs direct VIO；camera–IMU 外参和时间偏移；rolling shutter。

---

### 9. 第九层：激光–惯性 LIO 理论基础

| LiDAR | IMU |
|---|---|
| 几何约束强 | 高频运动预测 |
| 易受几何退化影响 | 易漂移 |
| scan 有运动畸变 | 可用于 deskew |
| 低频 | 高频 |

LIO 状态 $x_k = \{R_k,p_k,v_k,b_g,b_a,T_{LI}\}$，有些系统还估 $\Delta t_{LI},\theta_I,\theta_L$。残差 $[r_I; r_L]$，其中 $r_I$ 是 IMU 预积分或 ESKF 传播残差，$r_L$ 是点到地图几何残差。

| 内容 | 为什么重要 |
|---|---|
| scan deskew | 不去畸变，点云会系统性错位 |
| point-to-map Jacobian | EKF/优化更新的核心 |
| local map management | 实时性和精度的平衡 |
| degeneracy detection | 走廊、平面、隧道会退化 |
| extrinsic calibration | $T_{LI}$ 错会直接影响点云投影 |
| time offset | $\Delta t_{LI}$ 错会影响 deskew |
| IMU bias observability | bias 错会让轨迹和地图漂移 |

LIO 的底层理论和多传感器标定高度耦合。

---

### 10. 第十层：视觉–激光–惯性 LVI 理论基础

LVI 状态可能包含 $X = \{R,p,v,b_g,b_a,P_i,T_{CI},T_{LI},T_{LC},\Delta t_{CI},\Delta t_{LI}\}$。后端统一写成：

$$\min_X \sum\|r_C\|^2 + \sum\|r_L\|^2 + \sum\|r_I\|^2 + \sum\|r_{calib}\|^2 + \sum\|r_{loop}\|^2$$

其中 $r_C$ 视觉重投影/光度、$r_L$ LiDAR 点到线/面/地图、$r_I$ IMU 预积分/传播、$r_{calib}$ 外参/时延/先验、$r_{loop}$ 回环。真正的紧耦合 LVI 要处理：

| 问题 | 解释 |
|---|---|
| 视觉退化 | 低纹理、强光照、运动模糊 |
| LiDAR 退化 | 长走廊、单平面、结构重复 |
| IMU 漂移 | bias、噪声、激励不足 |
| 标定耦合 | $T_{CI},T_{LI},T_{LC}$ 相互约束 |
| 时间同步 | 相机帧、LiDAR scan、IMU 高频异步 |
| 地图表达 | sparse feature map、point cloud map、surfel map、mesh |
| 回环一致性 | pose graph / submap / place recognition |

LVI-SAM 代表 factor graph LVI 路线，由 visual-inertial 和 lidar-inertial 两个子系统组成，互相辅助初始化、深度估计、scan matching 和回环。[11]

---

### 11. 第十一层：可观性、退化与一致性

这部分是区分"会跑系统"和"能做研究"的关键。SLAM 里很多量不是天然可估的：

| 系统 | 常见不可观/弱可观量 |
|---|---|
| 单目 VO | 绝对尺度 |
| VIO | 全局位置、global yaw |
| LIO | 几何退化方向，如长走廊横向 |
| 地面车 VIO/LIO | roll/pitch/vertical/extrinsic 某些方向弱可观 |
| 多传感器标定 | 时间偏移和外参平移可能耦合 |
| Pose Graph | global gauge freedom |

Li & Mourikis 中时间偏移 $t_d$ 是否可辨识是核心理论问题：$t_d$ 在一般轨迹下局部可辨识，但零旋转、常角速度、常加速度/常速度等会导致退化或弱可辨识。读高水平 TRO/IJRR/RSS 论文要特别关注：observability、identifiability、degeneracy、consistency、gauge freedom、unobservable subspace —— 往往比单纯 RMSE 更重要。

---

### 12. 第十二层：鲁棒估计与数据关联

实际 SLAM 前端会产生大量错误：视觉误匹配、动态物体、LiDAR 错误对应、回环误检、时间戳抖动、标定误差、IMU 饱和、光照变化、雨雾灰尘、玻璃反光稀疏纹理。必须学：

| 内容 | 作用 |
|---|---|
| RANSAC | 几何模型估计中的外点剔除 |
| Huber/Cauchy/Tukey | robust kernel |
| chi-square gating | 滤波更新前的异常测量剔除 |
| switchable constraints | 错误回环处理 |
| dynamic covariance scaling | 鲁棒图优化 |
| PCM | 回环一致性筛选 |
| semantic filtering | 动态物体过滤 |
| degeneracy-aware weighting | 退化方向降权 |

鲁棒估计不是附加模块，而是 SLAM 系统能否在真实机器人场景工作的必要条件。

---

### 13. 理论基础总表

| 理论模块 | 视觉 SLAM | LiDAR SLAM | 惯性 / VIO / LIO |
|---|---|---|---|
| 线性代数 | BA、PnP、triangulation | ICP、NDT、plane fitting | 协方差传播、预积分 |
| 概率估计 | 重投影误差建模 | 点云残差权重 | IMU 噪声、bias |
| 李群李代数 | 相机位姿优化 | SE(3) scan matching | 姿态积分、误差状态 |
| 非线性优化 | BA、pose graph | scan-to-map optimization | VIO/LIO smoothing |
| 滤波 | EKF-SLAM、MSCKF | FAST-LIO 类 ESKF | ESKF、IEKF |
| 因子图 | ORB-SLAM 后端、VINS | LIO-SAM | IMU factor |
| 多视几何 | 核心 | 辅助相机–LiDAR融合 | VIO visual factor |
| 点云几何 | 辅助深度 | 核心 | LIO LiDAR factor |
| IMU理论 | VIO 必需 | LIO 必需 | 核心 |
| 可观性 | 尺度、yaw、外参 | 几何退化 | bias、gravity、外参、时延 |
| 鲁棒估计 | 误匹配、动态物体 | 错误对应、退化 | 异常 IMU、同步误差 |
| 标定理论 | 相机内参/外参 | LiDAR 外参/内参 | VI/LI/LVI 时空标定 |

---

### 14. 推荐学习顺序

**阶段 1 状态估计总论**：知道 SLAM 为什么是概率状态估计问题。读 Bayesian filtering、MLE/MAP、least squares、covariance/information、Kalman filter、factor graph。主教材 Barfoot《State Estimation for Robotics》。[1]
**阶段 2 李群李代数**：能看懂 $SO(3),SE(3),\exp,\log,\operatorname{Ad},J_r,J_l$。读 Solà micro-Lie；Barfoot 三维运动部分；GTSAM/Ceres manifold parameterization 示例。[12]
**阶段 3 非线性优化与因子图**：能自己写简单 pose graph / BA。学 GN、LM、robust kernel、Schur complement、marginalization、iSAM2/Bayes tree。iSAM2 是理解增量式 smoothing 的核心。[3]
**阶段 4 视觉几何与视觉 SLAM**：理解 ORB-SLAM/DSO/VINS 的视觉部分。学 camera model、epipolar geometry、triangulation、PnP、BA、feature/direct、loop closure、Sim(3)。ORB-SLAM 是特征法系统级代表。[4]
**阶段 5 IMU 与 VIO**：理解 VINS-Mono、OKVIS、MSCKF、OpenVINS。学 IMU 连续模型、ESKF、MSCKF、IMU preintegration、VIO initialization、scale/gravity/bias observability、camera–IMU 标定。Forster on-manifold preintegration 与 OKVIS 是核心入口。[6]
**阶段 6 LiDAR 几何与 LIO**：理解 LOAM、LIO-SAM、FAST-LIO2。学 ICP/NDT、point-to-plane、edge/plane feature、scan deskew、运动畸变、local map、ESKF LIO、factor graph LIO、degeneracy detection。三者分别代表 feature-based LOAM、factor-graph LIO、direct iterated-EKF LIO。[7]
**阶段 7 LVI 多传感器融合与标定**：把视觉、激光、惯性统一到一个状态估计框架。学 LVI factor graph、camera–LiDAR–IMU 外参、时间同步、rolling shutter、LiDAR per-point timestamp、observability-aware calibration、failure-aware fusion。与多传感器标定直接衔接。

---

### 15. 当前结论

8 个核心方向：① 概率状态估计（MLE、MAP、Bayes filter、covariance、information）② 李群李代数（$SO(3),SE(3)$、扰动模型、Jacobian）③ 非线性优化（GN、LM、Schur、marginalization、robust kernel）④ 因子图和平滑（factor graph、iSAM2、Bayes tree、fixed-lag smoothing）⑤ 滤波理论（EKF、ESKF、MSCKF、IEKF、Invariant EKF）⑥ 视觉几何（投影、多视几何、PnP、三角化、BA、回环）⑦ LiDAR 几何（ICP、NDT、LOAM、scan deskew、point-to-map）⑧ 惯性导航与融合（IMU 模型、预积分、bias、gravity、VIO/LIO/LVI）。

最合适的学习主线是纵向：

$$\text{状态估计} \rightarrow \text{李群与优化} \rightarrow \text{视觉/激光/惯性测量模型} \rightarrow \text{VIO/LIO} \rightarrow \text{LVI} \rightarrow \text{标定、可观性、退化、鲁棒性}$$

学完之后再看 FAST-LIO、LIO-SAM、VINS-Mono、OpenVINS、ORB-SLAM3、R3LIVE、FAST-LIVO、LVI-SAM，会更容易看出真正的理论结构，而不是只停留在工程流程。

---

#### 引用
[1] State Estimation for Robotics (Barfoot), Cambridge. [2] Factor Graphs and GTSAM, gtsam.org/tutorials/intro.html. [3] iSAM2: Incremental smoothing and mapping using the Bayes tree, IJRR 2012, 10.1177/0278364911430419. [4] ORB-SLAM, arXiv:1502.00956. [5] Direct Sparse VIO / Dynamic Marginalization, arXiv:1804.05625. [6] On-Manifold Preintegration for Real-Time VIO, arXiv:1512.02363. [7] LOAM: Lidar Odometry and Mapping in Real-time, CMU RI. [8] LIO-SAM, Stevens. [9] FAST-LIO2: Fast Direct LiDAR-inertial Odometry, arXiv:2107.06829. [10] Keyframe-based visual–inertial odometry (OKVIS), IJRR 2015, 10.1177/0278364914554813. [11] LVI-SAM, arXiv:2104.10831. [12] A micro Lie theory for state estimation in robotics, arXiv:1812.01537.



---

# 第 II 部 · 多传感器标定 体系 · 完整原文

## 多传感器SLAM标定 总体系报告 + 核心文献地图 v1（来源：另一 Agent · 完整原文存档）

> 存档说明：另一 Agent 产出的标定方向第一版总体系报告**完整原文**（公式清理为可读形式，内容未删减）。先搭领域骨架、主线文献、方法分类、传感器组合、近年前沿和推荐阅读路径；后续可对 VI、LI、LVI、GNSS-aided SLAM 分章节精读。其缺口由我评审(7条)与 `标定_补遗与资源_workflow.md`(122条)补强。

### 0. 总结先行：核心问题
SLAM 多传感器标定不是单独的"预处理工具问题"，而是多传感器状态估计系统的组成部分。对 LiDAR、RGB 相机、IMU、GNSS 四类，标定目标统一写成：

$$\mathcal{C} = \{\theta_C,\theta_L,\theta_I,\theta_G,\ T_{CI},T_{LI},T_{LC},T_{GI},\ \Delta t_C,\Delta t_L,\Delta t_G,\ \alpha_C,\alpha_L,\alpha_G\}$$

- $\theta_C$：相机内参、畸变、rolling shutter readout time；
- $\theta_L$：LiDAR 内参、beam model、range bias、scan pattern、per-point timestamp；
- $\theta_I$：IMU scale、axis misalignment、g-sensitivity、bias/noise model；
- $\theta_G$：GNSS 接收机/天线相关模型，主要是 antenna lever-arm、clock bias/drift；
- $T_{ab}$：不同传感器坐标系之间的 SE(3) 外参；
- $\Delta t_s$：传感器 $s$ 相对参考时间轴的时间偏移；
- $\alpha_s$：clock skew/drift，线性时间模型 $t_\text{ref} = \alpha_s t_s + \beta_s + \epsilon_s$（$\beta_s$ offset，$\alpha_s$ 时钟比例误差，$\epsilon_s$ jitter）。

对 SLAM 最重要三类量：**内参 + 空间外参 + 时间外参**。很多系统只显式讨论空间外参 $T$，但真正影响融合精度的通常是 空间误差、时间误差、运动畸变、IMU bias、退化运动 的**耦合**。

### 1. 完整知识体系
#### 1.1 三层标定问题
| 层次 | 代表参数 | 典型传感器 | 对 SLAM 的影响 |
|---|---|---|---|
| 单传感器内参 | 相机焦距/畸变、IMU scale/misalignment、LiDAR beam/range bias | Camera/IMU/LiDAR | 决定单传感器测量是否可信 |
| 多传感器空间外参 | $T_{CI},T_{LI},T_{LC},T_{GI}$ | VI/LI/LC/LVI/GNSS-INS | 决定多模态数据能否落到同一坐标系 |
| 多传感器时间外参 | $\Delta t_{CI},\Delta t_{LI},\Delta t_{GI}$、clock skew、jitter | 所有异步组合 | 决定观测是否对应同一真实物理时刻 |

Li & Mourikis 的 IJRR 论文明确指出：VI 融合中每个测量的采样时刻必须精确已知；他们把 camera–IMU 时间偏移 $t_d$ 作为额外状态，与 IMU pose/velocity/bias、camera–IMU 外参、特征点一起估计，并分析了 $t_d$ 的可辨识性。Qin & Shen 的 IROS 2018 论文从优化式 VIO 出发，指出低成本/自组装系统时间偏移可达数毫秒到数百毫秒，几十毫秒即严重破坏 VIO；他们把时间偏移和相机/IMU 状态、特征一起联合优化，集成到 VINS-Mono。

#### 1.2 四种传感器在标定中的本质差异
| 传感器 | 测量类型 | 典型频率 | 时间特性 | 标定难点 |
|---|---|---|---|---|
| RGB Camera | 2D bearing/intensity/feature | 10–60 Hz | 全局/滚动快门；曝光中心≠timestamp | 内参、畸变、rolling shutter、与IMU/LiDAR外参和时延 |
| IMU | 角速度、加速度 | 100–1000 Hz | 高频、连续时间近似好，常作时间参考 | bias、noise density、scale、轴失准、g-sensitivity |
| LiDAR | 3D range point/scan | 5–20 Hz 或更高 | 一帧非同时采集；每点有采样时间 | scan distortion、per-point timing、LiDAR–IMU外参/时延 |
| GNSS | pseudorange/Doppler/RTK pose | 1–20 Hz | 低频，绝对时间强，但 latency 和 lever-arm 重要 | antenna lever-arm、GNSS clock、global frame alignment、NLOS/multipath |

IMU 通常作动力学和时间参考；LiDAR/相机提供外部几何约束；GNSS 提供全局约束（机器人 SLAM 标定中通常次级）。

### 2. 领域主线：从离线标定到在线自标定
#### 2.1 靶标法（最早、最稳定，不适合长期在线）
依赖 checkerboard、AprilTag、平面板、球体、专用 3D target。优点精度高、约束明确、易复现；缺点需人工布置、要求共同视场、不能处理运行中外参变化。Geiger et al. ICRA 2012 "Automatic Camera and Range Sensor Calibration using a Single Shot" 是重要早期工作，单帧图像+range scan 自动恢复相机内外参及 camera–range 变换。[1] HKU-MARS 的 FAST-Calib 代表 2025 后目标法工程化：设计更适合机械式/固态 LiDAR 的 3D target，使 LiDAR–camera 外参标定变成"一秒级、无需初值、支持不同扫描模式"。[2]

#### 2.2 无靶标法（从自然场景找跨模态约束）
LiDAR–camera 常见约束：edge alignment、plane alignment、mutual information、semantic consistency、depth-image consistency。Pandey et al. JFR 2015 "Automatic Extrinsic Calibration of Vision and Lidar by Maximizing Mutual Information" 是 LiDAR–camera targetless 里程碑，用互信息对齐 camera image 与 LiDAR reflectivity/depth。[3] Levinson & Thrun RSS 2013 "Automatic Online Calibration of Cameras and Lasers" 代表在线自然场景标定，不依赖已知靶。[4] 近年转向 edge/semantic/learning-assisted：Koide et al. ICRA 2023 提供 general/single-shot/target-less/automatic 工具箱；2025–26 引入 MobileSAM、semantic distribution alignment、joint intrinsic/extrinsic targetless，但需谨慎评估跨域泛化与几何可解释性。[5]

#### 2.3 连续时间标定（时空联合估计核心理论框架）
异步采样本质 $z_s(t_s) \rightarrow h_s(x(t_s+\Delta t_s), \theta_s, T_s)$。若轨迹只在离散关键帧定义，处理高频 IMU、LiDAR per-point timestamp、rolling shutter、非同步相机都很别扭。连续时间用 B-spline 或 GP 表示 $T(t)$，任意时刻取 pose/velocity/acceleration。Furgale/Barfoot/Sibley ICRA 2012 "Continuous-Time Batch Estimation using Temporal Basis Functions" 是奠基。[6] Furgale/Rehder/Siegwart IROS 2013 "Unified Temporal and Spatial Calibration for Multi-Sensor Systems" 把时间偏移和空间外参放进统一最大似然框架，是 Kalibr 主线理论起点。[7] Rehder et al. TRO 2016 "A General Approach to Spatiotemporal Calibration in Multisensor Systems" 是顶刊集大成。[8] Kalibr 把 camera intrinsics、camera–IMU extrinsics、time offset、multi-camera、multi-IMU、IMU intrinsics 放进同一框架。[9]

#### 2.4 在线自标定（把 calibration 当 SLAM 状态）
$$x = \{R,p,v,b_g,b_a, T_{CI},T_{LI},T_{LC}, \Delta t_{CI},\Delta t_{LI}, \theta_C,\theta_I,\theta_L, \text{map}\}$$
Li & Mourikis IJRR 2014 是 camera–IMU 在线时间标定理论基准：$t_d$ 增广进 EKF，证明一般运动下局部可辨识；退化主要包括零旋转、常角速度、常加速度/常速度。Qin & Shen IROS 2018 是优化式 VIO 工程基准：用图像平面 feature velocity 近似时间偏移带来的观测位移 $z_l^k(t_d)=z_l^k+t_d V_l^k$，并放进滑窗非线性优化。UDel/RPNG/OpenVINS 系列：Yang et al. RAL 2019 分析 aided INS 在线时空标定退化；TRO 2023 "Online Self-Calibration for VINS: Models, Analysis, and Degeneracy" 把 IMU/camera intrinsics、IMU-camera spatial-temporal extrinsics、rolling shutter readout time 纳入完整可观性分析。[10] RSS 2020 "Online IMU Intrinsic Calibration: Is It Necessary?" 讨论 IMU intrinsic 在线标定必要性与可观性。[11]

### 3. 按传感器组合构建知识图谱
#### 3.1 Camera–IMU（理论最成熟）
估计 $T_{CI},\Delta t_{CI},\theta_C,\theta_I,b_g,b_a,g,s$。相机只给 bearing，IMU 给尺度/重力/短时运动约束。运动充分可联合估；退化（纯平移、纯单轴旋转、匀速直线）则某些参数不可观。

里程碑：
| 阶段 | 代表 | 地位 |
|---|---|---|
| EKF 外参标定 | Mirzaei & Roumeliotis TRO 2008 | 早期严肃处理 camera–IMU 外参和可观性的顶刊[12] |
| VI 自标定 | Kelly & Sukhatme IJRR 2011 | UKF + 微分几何可观性分析 transform/bias/gravity/structure[13] |
| 连续时间 batch | Furgale/Rehder/Siegwart IROS2013/TRO2016 | Kalibr 主线[7] |
| EKF 在线时间标定 | Li & Mourikis IJRR 2014 | $t_d$ 作状态，identifiability analysis |
| 优化式在线时间标定 | Qin & Shen IROS 2018 | $t_d$ 写进 visual factor，集成 VINS-Mono |
| 多相机在线标定 | Eckenhoff et al. ICRA 2019 | 任意数量异步相机 online intrinsic/extrinsic[14] |
| full self-calibration | Yang et al. TRO 2023 | VINS 全量内参/外参/时间/卷帘可观性与退化[15] |
| 多 IMU/多相机扩展 | Yang/Geneva/Huang IJRR 2024 | 任意数量异步 IMU/gyro/global-or-rolling-shutter cameras[16] |

方法主线：① 离线 batch（Kalibr）② 在线 EKF（MSCKF/OpenVINS）③ 在线滑窗优化（VINS/OKVIS/ORB-SLAM3）④ full self-calibration ⑤ multi-camera/multi-IMU。Camera–IMU 是最好的入口，暴露所有核心概念：状态增广、measurement Jacobian、时间偏移对观测的导数、可观性、退化运动、离散vs连续时间建模。

#### 3.2 LiDAR–IMU（激光 SLAM 最关键）
估计 $T_{LI},\Delta t_{LI},\theta_L,\theta_I,b_g,b_a,g$。LiDAR 扫描式，一帧内每点采样时刻不同。$\Delta t_{LI}$ 不准→deskew 错；$T_{LI}$ 不准→点到线/面残差系统偏移；IMU intrinsic/bias 不准→快速运动地图变厚、边缘发散。

里程碑：
| 代表 | 地位 |
|---|---|
| LI-Calib IROS2020 | Targetless，B-spline 连续时间轨迹，联合 IMU cost 与 LiDAR point-to-surfel，估 6DoF 外参和时间偏移[17] |
| OA-LICalib TRO2022 | LiDAR–IMU full calibration 集大成；无靶标连续时间 batch，同估 LiDAR/IMU intrinsics 与时空外参，observability-aware data selection + TSVD 处理退化[18] |
| LI-Init IROS2022 | 实时 LiDAR-inertial 初始化；估 temporal offset/extrinsic/gravity/IMU bias，无需靶标/额外传感器/先验地图/外参初值[19] |
| 多 LiDAR 在线标定 | observability-aware，车辆平台、无 fiducial[20] |

残差 $r_L = n^\top(T_{WL}(t_i+\Delta t_L)\,p_L^i - q)$，$p_L^i$ LiDAR 点，$q,n$ 局部平面/surfel，$T_{WL}(t)$ 来自连续轨迹或 IMU 推算。spinning/Livox/solid-state 的关键差异在 $t_i$ 定义和 scan pattern。退化来自：① 运动退化（直线匀速、纯 yaw、平面运动、低加速度）② 环境退化（单平面、长走廊、少结构、重复结构）。OA-LICalib 把可观性/信息量放进标定流程：选信息量高的数据片段；退化不可避免时只更新可辨识方向。[18]

#### 3.3 LiDAR–Camera（工程论文最多，质量差异最大）
估计 $T_{LC},\Delta t_{LC},\theta_C,\theta_L$。无 IMU 动力学约束，跨模态数据关联是核心难题。
| 方法类型 | 约束来源 | 优点 | 缺点 |
|---|---|---|---|
| 靶标法 | checkerboard/AprilTag/3D target/sphere/edge | 精度高约束明确 | 需人工、共同视场、现场不便 |
| 互信息法 | image intensity 与 LiDAR intensity/depth | 不需显式特征匹配 | 目标函数非凸，对初值敏感 |
| 边缘/线法 | image edge 与 point cloud edge/depth | 几何直观 | 光照、纹理、场景结构影响大 |
| 平面法 | 视觉平面与 LiDAR 平面 | 室内/结构场景好 | 平面退化明显 |
| 语义法 | semantic mask/object/lane/pole | 城市自然场景可用 | 依赖感知模型，泛化风险 |
| 学习法 | deep feature/calibration network | 初值容忍度可能更好 | 数据域迁移和可解释性不足 |

Levinson & Thrun RSS2013、Pandey JFR2015、Koide ICRA2023；2025–26 集中在 MobileSAM/semantic/targetless/joint。[4] 警惕：很多论文只报重投影、不充分分析可观性/退化/时延/下游 SLAM 性能。更有价值的是 **calibration-aware SLAM**（外参在 LVI 后端被观测、约束、冻结或更新）。

#### 3.4 LiDAR–Camera–IMU（多传感器 SLAM 标定核心）
状态 $\mathcal{X}_{calib} = \{T_{CI},T_{LI},T_{LC},\Delta t_{CI},\Delta t_{LI},\Delta t_{LC},\theta_C,\theta_L,\theta_I\}$。约束 $T_{LC}=T_{LI}T_{IC}$（可间接得，但直引 LiDAR–camera 约束往往增强一致性）。难点：多外参/多时延相互补偿（错 $\Delta t_{LI}$ 可能被错 $T_{LI}$ 或轨迹形变吸收）。

| 工作 | 贡献 |
|---|---|
| LIC-Fusion IROS2019 | 紧耦合 LiDAR-inertial-camera odometry，在线估三类异步传感器 spatial/temporal calibration[21] |
| LIC-Fusion 2.0 IROS2020 | 滑窗 LiDAR 平面特征跟踪；分析 IMU–LiDAR 平面特征标定退化[22] |
| LVI-SAM ICRA2021 | factor graph LiDAR-visual-inertial smoothing and mapping（融合系统，非标定论文）[23] |
| R3LIVE/R3LIVE++ RAL2022 | HKU-MARS LiDAR-inertial-visual fusion，鲁棒实时 RGB-colored mapping（系统基线）[24] |
| LVI-ExC ACM MM2022 | Target-free 联合 LiDAR/camera/IMU 外参；称首个完全 target-free LVI 外参联合标定[25] |
| Two-Step LCI Calib TIE2024 | 连续时间 target-free LCI 时空标定，支持 GS/RS 相机和内参 refinement[26] |
| iKalibr 2024–25 | integrated inertial systems 的 unified targetless spatiotemporal calibration，支持 IMU/radar/LiDAR/camera；动态初始化 + 连续时间 batch，开源[27] |
| GLIC-Calib IROS2025 | 地面车 targetless one-shot LiDAR-IMU-camera spatial-temporal calibration[28] |

关键问题：① 多外参耦合 ② 多时延耦合 ③ LiDAR scan 内部时间 ④ 视觉与 LiDAR 约束频率不同 ⑤ 退化环境常见（地面车平面运动、室内长走廊、无人机匀速、结构重复）⑥ 在线更新风险（退化运动可能发散→需 observability-aware update/freezing）。

#### 3.5 GNSS（次级但必要的全局约束）
涉及 $T_{GI}, p_{\text{antenna}}^I, \Delta t_{GI}$, global frame alignment。$p_{\text{antenna}}^I$ 是 GNSS 天线相对 IMU/body 的 lever-arm。GNSS 提供全局定位约束，抑制漂移、提供 ENU/ECEF 对齐，而非稠密几何地图。GVINS 紧耦合 GNSS raw + 视觉 + IMU，实时无漂移 6DoF 全局定位，coarse-to-fine 初始化在线标定局部状态与全局测量的变换。[29] GAINS（UDel/RPNG）tightly-coupled GNSS-aided VI localization。[30] GICI-LIB GNSS/INS/Camera 集成导航开源库。[31] 数据集 MARS-LVIG（HKU-MARS, IJRR2024）：航拍下视角 LiDAR-Visual-Inertial-GNSS，飞行 80–130m，21 序列，覆盖机场/岛屿/乡镇/山谷。[32] GNSS 建议作补充章节，重点 lever-arm、time delay、global frame alignment、GNSS factor 与局部 SLAM 状态耦合。

### 4. 理论主线：可观性、可辨识性和退化运动
#### 4.1 时间偏移为何可观
视觉残差依赖 $T(t+\Delta t)$，对时间偏移求导 $\frac{\partial r}{\partial \Delta t} = \frac{\partial r}{\partial T}\frac{\partial T(t)}{\partial t}$，而 $\frac{\partial T(t)}{\partial t}$ 由平台线速度/角速度/加速度决定。运动足够丰富 → 时间偏移导致可观测残差变化；静止/匀速/常角速度 → 与外参/轨迹变形混淆。Li & Mourikis 证 $t_d$ 一般运动下局部可辨识，不可辨识情形主要是已知会导致 VINS 可观性丢失的退化运动。Yang et al. RAL 2019 推广到 aided INS。[10]

#### 4.2 空间外参与时间偏移为何耦合
传感器安装外参平移 $p_{SI}$，若存在时间偏差 $\Delta t$，短时近似 $p(t+\Delta t)\approx p(t)+v(t)\Delta t$ → 时间偏差表现成沿速度方向的"伪平移"。直线匀速时 $\Delta t$ 与 $T$ 平移部分易混淆。LiDAR–IMU 中错误时间偏移还表现成 scan deskew 误差；LiDAR–camera 快速运动时 camera 与 LiDAR 错位表现成投影边缘偏移。所以标定数据须含 多轴旋转 + 变速平移 + 加减速 + 非共面运动。

#### 4.3 地面机器人为何更难
平面运动、roll/pitch 变化小、主要 yaw、速度近恒、长走廊/墙面/单平面、GNSS 城市峡谷遮挡多路径 → $T_{LI},T_{CI},\Delta t_{LI},\Delta t_{CI}$、IMU intrinsic 中某些方向弱/不可观。OA-LICalib、Yang 退化系、LIC-Fusion 2.0 技术报告都与此直接相关。[18]

### 5. 核心综述文献
| 优先级 | 综述 | 价值 |
|---|---|---|
| 1 | Camera, LiDAR, IMU Spatiotemporal Calibration: Methodological Review, Sensors 2025 | 最贴合，覆盖 camera–IMU/LiDAR–IMU/camera–LiDAR/camera–LiDAR–IMU，强调外参与时间标定[33] |
| 2 | Automatic targetless LiDAR–camera calibration: a survey, AI Review 2023 | 专门梳理 targetless LiDAR–camera 外参[34] |
| 3 | LiDAR, IMU, and camera fusion for SLAM, Applied Intelligence 2025 | 按 LI/VI/LV/LVI 梳理，含 calibration 前置[35] |
| 4 | Camera, LiDAR, IMU Based Multi-Sensor Fusion SLAM: A Survey, 2024 | 以 SLAM 系统为主线[36] |
| 5 | A Review of Visual-LiDAR Fusion based SLAM, Sensors 2020 | visual–LiDAR SLAM 背景[37] |
| 6 | A Review of Multi-Sensor Fusion SLAM Systems Based on 3D LiDAR, Remote Sensing 2022 | 偏融合系统和数据集[38] |

建议：先读 Sensors 2025 spatiotemporal calibration 综述，再读 LiDAR–camera targetless survey，再回顶级论文原文。

### 6. 里程碑时间线
2007/2008 Mirzaei & Roumeliotis (VI, EKF外参+可观性)[12] · 2010 Kelly-Sukhatme temporal calibration framework (ISER)[39] · 2011 Kelly-Sukhatme IJRR (VI self-calib)[13] · 2011 TICSync ICRA (time sync, 多时钟精确映射)[40] · 2012 Furgale/Barfoot/Sibley ICRA (连续时间 batch 奠基)[6] · 2012 Geiger ICRA (Camera–LiDAR 单帧自动标定)[1] · 2013 Furgale/Rehder/Siegwart IROS (统一时空, Kalibr主线)[7] · 2013 Levinson-Thrun RSS (在线 camera–laser)[4] · 2014 Li-Mourikis IJRR (VI时间标定基准) · 2015 Pandey JFR (互信息 targetless)[3] · 2016 Rehder TRO (连续时间多传感器集大成)[8] · 2016 Extending Kalibr ICRA (多IMU/IMU内参)[41] · 2018 Qin-Shen IROS (优化式 VIO 时间标定) · 2019 退化运动分析 RAL/ICRA[10] · 2019 LIC-Fusion IROS (LVI 在线时空标定)[21] · 2020 LI-Calib IROS (LI targetless 连续时间)[17] · 2020 Online IMU Intrinsic RSS[11] · 2020 LIC-Fusion 2.0 IROS[22] · 2021 Qiu et al. TRO (异构 temporal/rotation)[42] · 2021 Peršić et al. TRO (GP moving object)[43] · 2021 LVI-SAM ICRA[44] · 2022 OA-LICalib TRO (LI full calib)[18] · 2022 LI-Init IROS[19] · 2023 Online Self-Calib for VINS TRO[45] · 2024 Multi-Visual-Inertial System IJRR[46] · 2024 MARS-LVIG IJRR (LVI-GNSS dataset)[47] · 2024 Two-Step LCI TIE[48] · 2025 iKalibr[27] · 2025 GLIC-Calib IROS[28] · 2025 FAST-Calib HKU-MARS[49] · 2026 Joint Target-Less Intrinsic+Extrinsic Camera-LiDAR (preprint)[50]

### 7. 最近两三年前沿
**7.1 从 pairwise 到 full-system**：把多传感器放进一个系统联合 spatial-temporal-intrinsic（Two-Step LCI/LVI-ExC/iKalibr/GLIC-Calib）[25]。
**7.2 从外参到内参+外参+时间一起估**：OA-LICalib（LI full）、TRO 2023 VINS full self-calibration（含卷帘 readout）[18]。
**7.3 从离线到在线但须 observability-aware**：检测运动/环境是否提供足够信息→只更新可观方向→退化时冻结或降权（OA-LICalib 信息论数据选择+TSVD；Yang 退化系；LIC-Fusion 2.0 平面特征退化）[51]。
**7.4 连续时间仍是异步多传感器核心**：LiDAR per-point time/卷帘/多相机/多 IMU/不同频率/时间偏移与 clock skew → 连续时间比离散关键帧自然（iKalibr/LI-Calib/OA-LICalib/Two-Step LCI）[17]。
**7.5 大模型/语义/学习辅助 LiDAR–camera 出现但非主流理论**：MobileSAM、semantic distribution alignment、joint targetless intrinsic/extrinsic 变多，适合做初值/跨模态匹配/鲁棒特征，仍需几何优化与可观性分析支撑[52]。
**7.6 标定友好硬件与数据集**：Science Robotics 2025 SUPER 高速无人机（对安装/时间同步/稳定性要求高）[53]；MARS-LVIG、M3DGR、Hilti 等强调 rigorous calibration/temporal alignment[47]。

### 8. 核心文献矩阵（第一批必读）
| 优先级 | 论文 | Venue | 传感器 | 标定对象 | 类型 | 为什么必读 |
|---|---|---|---|---|---|---|
| S | Mirzaei & Roumeliotis | TRO2008 | Cam-IMU | $T_{CI}$ | EKF/obs | VI 外参标定早期理论基准[12] |
| S | Kelly & Sukhatme | IJRR2011 | Cam-IMU | $T_{CI}$,bias,gravity,scale | UKF/self-calib | VI self-calibration 经典[13] |
| S | Furgale et al. CT batch | ICRA2012/IJRR2015 | general | continuous trajectory | continuous-time | Kalibr与异步多传感器数学入口[6] |
| S | Furgale/Rehder/Siegwart 统一时空 | IROS2013 | multi-sensor | $T,\Delta t$ | CT batch | Kalibr 主线起点[7] |
| S | Rehder et al. | TRO2016 | multi-sensor | $T,\Delta t$ | CT MLE | 多传感器时空标定顶刊基准[8] |
| S | Li & Mourikis | IJRR2014 | Cam-IMU | $\Delta t_{CI},T_{CI}$ | EKF online | 核心论文,时间偏移可辨识性 |
| A | Qin & Shen | IROS2018 | Cam-IMU | $\Delta t_{CI}$ | 滑窗优化 | VINS-Mono 工程主线 |
| S | Yang et al. Degenerate Motion | RAL/ICRA2019 | aided INS | spatial+temporal | observability | 退化运动分析必读[10] |
| S | Yang et al. Online Self-Calib VINS | TRO2023 | Cam-IMU | full calibration | obs/self-calib | VI full self-calibration 集大成[45] |
| S | Yang/Geneva/Huang Multi-VI | IJRR2024 | multi-cam/IMU | full calibration | multi-VI | 多IMU/多相机/异步/卷帘[46] |
| S | OA-LICalib | TRO2022 | LiDAR-IMU | intrinsics+$T,\Delta t$ | CT/obs-aware | LI full calibration 代表[18] |
| A | LI-Calib | IROS2020 | LiDAR-IMU | $T_{LI},\Delta t_{LI}$ | targetless CT | LI 标定工具基线[54] |
| A | LI-Init | IROS2022 | LiDAR-IMU | $T,\Delta t,g,bias$ | online init | LIO 初始化与在线标定桥梁[19] |
| S | Levinson & Thrun | RSS2013 | Cam-Laser | $T_{LC}$ | online targetless | LC 在线标定经典[4] |
| A | Pandey et al. mutual info | JFR2015 | Cam-LiDAR | $T_{LC}$ | targetless MI | LC targetless 经典[3] |
| A | Koide et al. single-shot | ICRA2023 | Cam-LiDAR | $T_{LC}$ | toolbox | LC 工具化代表[5] |
| S | Qiu et al. | TRO2021 | heterog+IMU | $\Delta t,R$ | motion correlation | 异构传感器时间/旋转标定经典[42] |
| S | Peršić et al. GP | TRO2021 | heterogeneous | $T,\Delta t$ | GP/object | 移动目标轨迹标定经典[43] |
| A | LIC-Fusion | IROS2019 | LVI | spatial+temporal | online LVI | LVI 在线标定代表[21] |
| A | LIC-Fusion 2.0 | IROS2020 | LVI | spatial+temporal | plane features | LVI 平面特征和退化[22] |
| A | LVI-SAM | ICRA2021 | LVI | mostly fixed | factor graph | LVI 系统基线[44] |
| A | R3LIVE | RAL2022 | LVI | mostly fixed | tightly-coupled | HKU-MARS 系统基线[55] |
| A | GVINS | ICRA/RAL | GNSS-Cam-IMU | global transform | GNSS raw factor | GNSS-aided VINS 代表[56] |
| P | iKalibr | 2024/25 | IMU/LiDAR/Cam/Radar | $T,\Delta t$,intrinsics | unified targetless | 近年前沿[27] |
| P/A | GLIC-Calib | IROS2025 | LiDAR-IMU-Cam | spatial-temporal | one-shot targetless | 2025 LCI 新进展[28] |
| P | FAST-Calib | 2025 | LiDAR-Cam | $T_{LC}$ | target-based fast | HKU-MARS 工程前沿[49] |

### 9. 工具链和代码生态
| 工具 | 主要用途 | 传感器 |
|---|---|---|
| Kalibr | 相机内参/cam–IMU外参/时间偏移/多相机/多IMU/IMU intrinsic | Cam-IMU/multi |
| OpenVINS | 在线 VI 估计与 calibration research platform（cam-IMU transform/time offset/cam intrinsics/IMU intrinsics） | Cam-IMU |
| VINS-Mono/Fusion | 优化式 VIO，时间标定模块 | Cam-IMU/stereo/GNSS |
| LI-Calib | LiDAR–IMU targetless 外参/时间偏移 | LiDAR-IMU |
| OA-LICalib | LiDAR/IMU 内参+时空外参+observability-aware | LiDAR-IMU |
| LI-Init | LIO 初始化（时延/外参/重力/IMU bias） | LiDAR-IMU |
| LIC-Fusion/2.0 | LVI 在线时空标定与里程计 | LVI |
| LVI-SAM | LVI factor graph SLAM 基线 | LVI |
| R3LIVE/FAST-LIVO2 | HKU-MARS LVI 系统基线 | LVI |
| FAST-Calib | 快速 LiDAR–camera target-based | LiDAR-Cam |
| iKalibr | 统一 targetless spatiotemporal（IMU/LiDAR/camera/radar） | generalized |

OpenVINS 支持 camera-to-IMU transform/time offset/camera intrinsics/inertial intrinsics(含 g-sensitivity)[57]；Kalibr 列多 IMU/多相机/IMU intrinsic/统一时空标定核心论文[58]。

### 10. 推荐学习路径
**阶段1 数学与问题定义**：SE(3)/Lie algebra/IMU preintegration/factor graph/EKF-MSCKF/continuous-time B-spline + 各传感器测量模型：
$z_C = \pi_{\theta_C}(T_{CI}T_{IW}(t+\Delta t_C)P_W)$；$z_L = T_{LI}\,p_L(t+\Delta t_L)$；$\omega_m = M_g\omega + b_g + n_g$；$a_m = M_a(R^\top(a-g)) + b_a + n_a$；$z_G = p_I(t+\Delta t_G) + R_I^W p_{\text{ant}}^I + n_G$。
**阶段2 Camera–IMU 主线**：Mirzaei2008 → Kelly2011 → Furgale/Kalibr2013+TRO2016 → Li-Mourikis2014 → Qin-Shen2018 → Yang RAL19/RSS20/TRO23 → Multi-VI IJRR24。
**阶段3 LiDAR–IMU 主线**：LI-Calib → OA-LICalib → LI-Init → FAST-LIO2/LIO-SAM deskew/外参 → multi-LiDAR/observability-aware。重点 $\Delta t_{LI}\to$ deskew error $\to$ point-to-plane residual bias $\to$ map thickness。
**阶段4 LiDAR–Camera 与 LVI**：Geiger2012 → Levinson-Thrun2013 → Pandey2015 → Koide2023 → LIC-Fusion(2) → LVI-ExC → Two-Step LCI → GLIC/iKalibr/FAST-Calib。重点跨模态约束设计（edge/plane/MI/semantic/continuous trajectory/joint）。
**阶段5 GNSS-aided**：GVINS → GAINS → GICI-LIB → MARS-LVIG dataset → lever-arm/time delay/global frame alignment。

### 11. 对研究方向的判断
**11.1 LiDAR–IMU 时空标定与 LIO 初始化**：退化运动下可靠估 $T_{LI},\Delta t_{LI},\theta_I,\theta_L$，与 scan deskew/地图残差/连续时间轨迹联合建模。基线 OA-LICalib、LI-Init。
**11.2 LVI 系统中 online calibration-aware fusion**：不是单独标 $T_{LC}$，而是 视觉因子+LiDAR surfel/plane factor+IMU factor+calibration factor 同时在线估/校正 $T_{CI},T_{LI},\Delta t_{CI},\Delta t_{LI}$；关键 observability-aware update（何时更新/冻结/部分方向）。
**11.3 地面机器人退化场景多传感器自标定**：退化检测、motion excitation assessment、信息量驱动数据选择、多源约束增强可观性、轮速/GNSS/非完整约束辅助、在线外参漂移监测。

### 12. 当前结论
一句话：**SLAM 多传感器标定正从"离线估一个固定外参"演化为"在 SLAM 后端中持续维护传感器模型、外参、时延和不确定性的在线自标定问题"。**
三条理论线：continuous-time trajectory / observability-degeneracy analysis / calibration-aware state estimation。三条工程线：Kalibr-OpenVINS / LI-Calib-OA-LICalib-LI-Init / LIC-Fusion-LVI-SAM-R3LIVE-FAST-LIVO2。下一步最适合深入 Camera–IMU 标定主线（所有多传感器标定理论入口），之后 LiDAR–IMU，最后 LiDAR–Camera–IMU 联合标定。

---

#### 引用 [1]-[58]
[1] Geiger ICRA2012 cvlibs.net/publications/Geiger2012ICRA.pdf · [2] FAST-Calib github.com/hku-mars/FAST-Calib · [3] Pandey JFR2015 doi 10.1002/rob.21542 · [4] Levinson-Thrun RSS2013 roboticsproceedings.org/rss09/p29.pdf · [5] Koide ICRA2023 · [6] Furgale ICRA2012 furgalep.github.io/bib/furgale_icra12.pdf · [7] Furgale-Rehder IROS2013 · [8] Rehder TRO2016 doi 10.1109/TRO.2016.2529645 · [9] Kalibr wiki github.com/ethz-asl/kalibr · [10] Yang RAL2019 yangyulin.net/papers/2019_icra_degenerate.pdf · [11] Online IMU Intrinsic RSS2020 · [12] Mirzaei-Roumeliotis TRO2008 · [13] Kelly-Sukhatme IJRR2011 doi 10.1177/0278364910382802 · [14] Eckenhoff ICRA2019 · [15] Yang TRO2023 arXiv:2201.09170 · [16] Multi-VI arXiv:2308.05303 · [17] LI-Calib arXiv:2007.14759 · [18] OA-LICalib arXiv:2205.03276 · [19] LI-Init github.com/hku-mars/LiDAR_IMU_Init · [20] Obs-aware multi-lidar robots.ox.ac.uk/~mfallon/publications/2023RAL_das.pdf · [21] LIC-Fusion IROS2019 · [22] LIC-Fusion2 pgeneva.com/downloads/papers/Zuo2020IROS.pdf · [23] LVI-SAM dl.acm · [24] R3LIVE github.com/hku-mars/r3live · [25] LVI-ExC fst.um.edu.mo · [26] Two-Step LCI researchgate · [27] iKalibr arXiv:2407.11420 · [28] GLIC-Calib researchgate · [29] GVINS github.com/HKUST-Aerial-Robotics/GVINS · [30] GAINS pgeneva.com/downloads/papers/Lee2022ICRA.pdf · [31] GICI-LIB arXiv:2306.13268 · [32] MARS-LVIG journals.sagepub doi 10.1177/02783649241227968 · [33] Sensors2025 mdpi.com/1424-8220/25/17/5409 · [34] AIR2023 doi 10.1007/s10462-022-10317-y · [35] ApplInt2025 springer · [36] MSF-SLAM survey sciopen · [37] Sensors2020 mdpi.com/1424-8220/20/7/2068 · [38] RemoteSens2022 mdpi.com/2072-4292/14/12/2835 · [39] Kelly ISER2014 framework starslab.ca · [40] TICSync ICRA2011 ori.ox.ac.uk · [41] Extending Kalibr ICRA2016 · [42] Qiu TRO2021 · [43] Peršić TRO2021 · [44] LVI-SAM arXiv:2104.10831 · [45] Yang TRO2023 full calib yangyulin.net/papers/2023_tro_full_calib.pdf · [46] Multi-VI IJRR2024 · [47] MARS-LVIG IJRR2024 · [48] Two-Step LCI TIE2024 researchr.org/publication/LiLCZW24 · [49] FAST-Calib arXiv:2507.17210 · [50] Joint Target-Less Cam-LiDAR arXiv:2605.23397 · [51] OA-LICalib april.zju.edu.cn · [52] Online Target-Free LC arXiv:2404.18083 · [53] SUPER science.org/doi/10.1126/scirobotics.ado6187 · [54] LI-Calib github.com/APRIL-ZJU/lidar_IMU_calib · [55] R3LIVE BUAA · [56] GVINS arXiv:2103.07899 · [57] OpenVINS docs.openvins.com · [58] Kalibr github.com/ethz-asl/kalibr



---

# 第 III 部 · 理论母树 补遗与资源（131 条 · Workflow）


## SLAM 理论基础(视觉-激光-惯性)缺口与资源补强报告

> 范围:在 v1 理论树 KNOWN 覆盖基础上,仅纳入 scout 验证状态为 `verified`/`corrected` 的条目;已在 KNOWN 中的条目一律剔除;跨 facet 去重。venue/ID 以 scout 修正后为准。
> **工具可达性诚实声明**:本报告为离线综合任务,未调用任何检索工具(`accessible_tools=[]`)。全部条目依赖输入中已独立核验的 scout JSON(WebSearch/WebFetch/arXiv 直取),arXiv MCP 在 scout 阶段因 SOCKS 代理失败不可用。条目 ID/venue 准确性继承 scout 的核验结论,未在本环节二次联网复核。

---

### ① 母树缺口/薄弱补强(按层归类)

#### A. 连续时间估计(母树仅"已flag待补",此处给深度根)

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| Continuous-time batch estimation using temporal basis functions | ICRA 2012, 10.1109/ICRA.2012.6225005 | canonical 起源 | 时间基函数批量轨迹估计的原始论文 |
| 同上(期刊全论) | IJRR 2015, 10.1177/0278364915585860 | canonical 全理论 | 基函数框架完整推导 |
| Batch CT Trajectory Estimation as Exactly Sparse GP Regression | arXiv:1412.0630 (AURO/RSS 2014) | GP-prior 根(STEAM 核) | 白噪声驱动 LTV-SDE 先验 → 块三对角逆核 |
| Full STEAM ahead: Exactly Sparse GP on SE(3) | IROS 2015, 10.1109/IROS.2015.7353368 | GP-prior 上 SE(3) | 常体速先验扩展到 SE(3) |
| Sparse GP for CT Estimation on Matrix Lie Groups | arXiv:1705.06020 (tech report) | 一般矩阵李群 | Dong/Boots/Dellaert 推广到一般李群 |
| Efficient Derivative Computation for Cumulative B-Splines on Lie Groups | CVPR 2020, arXiv:1911.08860 | 样条派根 | O(k) 递推导数 + SO(3)/SE(3) 解析节点 Jacobian |
| Jacobian Computation for Cumulative B-Splines on SE(3) | RA-L 2022, arXiv:2201.10602 | 解析 Jacobian | 可复用的 SE(3) 样条解析 Jacobian |
| CT State Estimation Methods in Robotics: A Survey | arXiv:2411.03951 (T-RO 投稿) | 统一综述 | 样条+GP 统一表述+开放问题 |
| CT Trajectory Estimation: GP vs Spline 对比 | arXiv:2402.00399 | 头对头实证 | GP 与 B-样条精度/求解时间对照 |
| CT Radar/Lidar-Inertial Odometry w/ GP Motion Prior | T-RO 2025, arXiv:2402.06174 | STEAM 落地 | 白噪声加速度 GP 先验 + 滑窗 LIO/RIO(steam_icp) |
| CT Spline VIO (rolling-shutter, async) | ICRA 2021, arXiv:2109.09035 | 实战样条 VIO | 卷帘+异步传感器,样条边界作相机-IMU约束 |

#### B. 可证全局最优 / 证书化(母树仅一句"certifiable"提示,此处建完整支)

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| SE-Sync: Certifiably Correct SE(3) Synchronization | IJRR 2019, **arXiv:1612.07386**(scout 修正:原引 1611.00128 为同组早期版) | PGO 全局最优根 | SDP 松弛+Riemannian Staircase+事后最优性证书 |
| Lagrangian Duality in 3D SLAM | IROS 2015, arXiv:1506.00746 | 对偶验证根 | SLAM 对偶=凸 SDP,零对偶间隙→全局最优 |
| PGO in the Complex Domain (SZEP) | arXiv:1505.03437 (T-RO) | 零间隙条件 | 平面 PGO 复域,单零特征值性质 |
| Convex Global 3D Registration with Lagrangian Duality | CVPR 2017, 10.1109/CVPR.2017.595 | 全局配准 | 强化对偶 SDP 求解全局 SE(3) 配准 |
| Rotation Averaging(教程) | IJCV 2013, 10.1007/s11263-012-0601-0 | 旋转平均根 | L1/L2 测地旋转平均,Weiszfeld,收敛半径 |
| Rotation Averaging and Strong Duality | CVPR 2018, arXiv:1705.01362 | 无间隙证明 | 谱图论证明除非噪声极大否则无对偶间隙 |
| Shonan Rotation Averaging | ECCV 2020, arXiv:2008.02737 | Staircase 旋转平均 | SO(p)^n 流形冲浪求全局最优旋转 |
| Sparse-BSOS Rotation Averaging | RSS 2019, arXiv:1904.01645 | 任意噪声证书 | 四元数多项式规划,稀疏有界度 SOS,任意实例 |
| Certifiably Optimal Anisotropic Rotation Averaging | arXiv:2503.07353 (2025) | 各向异性前沿 | 全协方差不确定性,各向同性求解器会失败 |
| Globally Optimal Planar PGO+Landmark via Sparse-BSOS | ICRA 2019, arXiv:1809.07744 | pose+landmark 缺口 | 超越纯 PGO,填补带路标 SLAM |
| QUASAR: Certifiable Wahba with Outliers | ICCV 2019, arXiv:1905.12536 | 鲁棒旋转搜索 | TLS Wahba=四元数 QCQP,95% 外点仍精确 |
| One Ring to Rule Them All | NeurIPS 2020, arXiv:2006.06769 | Lasserre/SOS 证书 | 矩松弛最小阶紧+DR 对偶证书验证 RANSAC/GNC 解 |
| STRIDE: Certifiably Optimal Robust Perception | TPAMI 2022/2023, arXiv:2109.03349 | 通用鲁棒 SDP | 统一 TLS/max-consensus/GM/Tukey 为 POP+可扩展求解 |
| Fast Robust Certifiable Relative Pose | CVIU 2021, arXiv:2101.08524 | 两视相对位姿 | SO(3)×S² 最优性证书+GNC |
| C2P: Non-minimal Certifiable Relative Pose | CVPR 2024, arXiv:2312.05995 | 无歧义前沿 | 单 QCQP 直接返回 cheirality 合法位姿,免四重歧义 |
| CORA: Certifiably Correct Range-Aided SLAM | T-RO 2024(2024 King-Sun Fu Best Paper), arXiv:2302.11614 | 距离辅助前沿 | 首个证书化 RA-SLAM,QCQP→SDP→Staircase |
| SCORE: SOCP Init for RA-SLAM | ICRA 2023, arXiv:2210.03177 | 凸初始化 | 首个 RA-SLAM 凸(SOCP)松弛初始化 |
| Exploiting Chordal Sparsity for Fast Global Optimality | WAFR 2024, arXiv:2406.02365 | SDP 可扩展前沿 | 弦稀疏分解使 SDP 复杂度线性,ADMM |
| Distributed Certifiably Correct PGO (RBCD) | T-RO 2021, arXiv:1911.03721 | 分布式/多机 | 稀疏 SDP+分布式 Riemannian 块坐标下降 |
| SIM-Sync: Certifiable Sim(3) Synchronization | RA-L 2024, arXiv:2309.05184 | 桥接 PGO↔BA | SE-Sync 推广到 Sim(3),联合位姿+单目深度尺度 |

#### C. 不变-等变估计(母树有 InEKF/Invariant-EKF,缺等变理论与新对称)

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| The InEKF as a Stable Observer | IEEE TAC 2017, arXiv:1410.1465 | InEKF 稳定性根 | group-affine 类+对数线性自治误差动力学 |
| An EKF-SLAM with Consistency Properties | arXiv:1510.06263 (2015) | InEKF-SLAM 一致性 | 不变 EKF 消除 gauge/全局帧诱导的不一致 |
| Exploiting Symmetries to Design EKFs | IEEE Sensors J. 2019, arXiv:1903.05384 | 对称-EKF | 自动捕获不可观方向 |
| Affine EKF (Aff-EKF) | arXiv:2412.10809 (2024) | 可观性维持充要条件 | 充要可观性维持条件+仿射变换设计法 |
| Equivariant Systems Theory and Observer Design | arXiv:2006.08276 (2020) | 等变理论根 | 齐性空间(流形+传递李群作用)系统理论 |
| Observer Design w/ Equivariance (EqF) | Annual Review of Control, Robotics, and Autonomous Systems 2022(**scout 修正:非"Annual Reviews in Control"**), arXiv:2108.09387 | EqF 推导根 | 对等变误差用 EKF 法+流形曲率 Riccati 修正 |
| EqF: General Filter on Homogeneous Spaces | **IEEE CDC 2020**(scout 修正:非 2021), arXiv:2107.05193 | EqF 一般构造 | 任意齐性空间系统可提升为等变系统 |
| EqVIO: Equivariant Filter for VIO | T-RO 2023, arXiv:2205.01980 | 等变 VIO | 新 VIO 李群对称,group-affine 无偏 IMU 动力学 |
| MSCEqF: Multi-State-Constraint Equivariant Filter | RA-L 2024, arXiv:2311.11649 | 等变 MSCKF | 对称群含 IMU bias+相机内外参(对比 IEKF 硬挂 SE₂(3)) |
| Equivariant Symmetries for INS | Automatica 2025, arXiv:2309.03765 | 统一对称分类 | MEKF/IEKF/EqF 皆为不同对称下的 EqF 实例 |
| Contact-Aided InEKF | IJRR 2020, arXiv:1904.09251 | InEKF 本体感知应用 | 接触辅助,轨迹无关吸引域,world/robot-centric |
| Convergence/Consistency of 3D Invariant-EKF SLAM | RA-L 2017, arXiv:1702.06680 | RI-EKF 收敛证明 | 无真值 Jacobian 假设的收敛证明 |
| AI-IMU Dead-Reckoning | IEEE T-IV 2020, arXiv:1904.06064 | 学习辅助不变滤波 | InEKF + 深网自适应测量协方差,KITTI 1.10% |
| Equivariant IMU Preintegration (Galilean) | RA-L 2025, arXiv:2411.05548 | 等变预积分前沿 | Gal(3) 左平凡切空间预积分,导航态-bias 几何耦合 |
| Associating Uncertainty with 3D Poses | T-RO 2014, 10.1109/TRO.2014.2298059 | SE(3) 上高斯根 | 均值群元+李代数扰动,位姿复合/融合 |
| Tutorial on Aided INS (Lie-Group) | arXiv:2603.07143(**scout 修正:单作者 Soulaimane Berkane,非 ANU/Klagenfurt**) | SE₂(3) 教学 | 面向实现的 SE₂(3)/等变滤波教程 |
| DRIFT: Proprioceptive Invariant State Estimation | ICRA 2024, arXiv:2311.04320 | InEKF 教程+框架 | 不变 KF 入门讲解+开源 dead-reckoning |

#### D. 地图表示(含神经-NeRF/3DGS)(母树仅"已flag",此处建完整谱系)

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| OctoMap | AURO 2013, 10.1007/s10514-012-9321-0 | 占据/八叉树根 | 概率八叉树体素地图 |
| Voxblox | IROS 2017, arXiv:1611.03631 | TSDF→ESDF | 单 CPU 增量 ESDF+网格提取 |
| nvblox | ICRA 2024, arXiv:2311.00626 | GPU TSDF/ESDF | GPU 加速(177× 表面/31× ESDF) |
| iMAP | ICCV 2021, arXiv:2103.12352 | 神经隐式根 | 单 MLP 作唯一场景表示实时 SLAM |
| NICE-SLAM | CVPR 2022, arXiv:2112.12130 | 可扩展神经隐式 | 分层特征网格,修复 MLP 过平滑/扩展性 |
| NeRF-SLAM | IROS 2023, arXiv:2210.13641 | 稠密单目+NeRF | BA 深度不确定性损失 |
| Point-SLAM | ICCV 2023, arXiv:2304.04278 | 神经点云 | 输入自适应动态稠密化点云 |
| SplaTAM | CVPR 2024, arXiv:2312.02126 | 3DGS RGB-D 地标 | 显式 3D 高斯,silhouette 引导稠密化 |
| Gaussian Splatting SLAM (MonoGS) | CVPR 2024 Highlight, arXiv:2312.06741 | 3DGS 单目地标 | 首个单目 3DGS,直接对高斯优化位姿 |
| Photo-SLAM | CVPR 2024, arXiv:2311.16728 | 混合显式+光度 | hyper-primitives(几何跟踪+光度渲染),Jetson 可跑 |
| Gaussian-LIC | ICRA 2025, arXiv:2404.06926 | LIC 融合+3DGS 前沿 | 紧 LiDAR-Inertial-Camera 融合,三角化+LiDAR 初始化高斯 |
| How NeRFs and 3DGS are Reshaping SLAM: a Survey | arXiv:2402.13255 (2024) | 辐射场 SLAM 综述 | 手工→深度→NeRF→3DGS 统一分类 |

#### E. Place-Recognition + 数据关联(母树仅"已flag")

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| DBoW2 | T-RO 2012, 10.1109/TRO.2012.2197158 | 二进 BoW 根 | FAST+BRIEF 词袋,正/逆索引(ORB-SLAM 回环底座) |
| NetVLAD | CVPR 2016, arXiv:1511.07247 | 学习全局 VPR 根 | 可微 VLAD 池化,弱监督排序损失 |
| PointNetVLAD | CVPR 2018, arXiv:1804.03492 | 3D 全局描述子 | NetVLAD 的点云版,lazy triplet/quadruplet |
| STD: Stable Triangle Descriptor | ICRA 2023, arXiv:2209.12435 | 旋转不变三角描述子 | 刚变不变,提供回环验证对应 |
| BTC: Binary+Triangle Descriptor | RA-L 2024, IEEE 10388464 | STD 期刊后继 | 全局三角+局部二进制 |
| Intensity Scan Context (ISC) | ICRA 2020, arXiv:2003.05656 | 强度+几何 LiDAR | 两阶层级重识别 |
| OverlapNet | RSS 2020/AURO 2021, arXiv:2105.11344 | 学习 LiDAR 回环 | Siamese 网预测 range-image overlap+yaw |
| JCBB | T-RA 2001, 10.1109/70.976019 | 联合相容数据关联根 | 联合 Mahalanobis 相容+分支定界 |
| ROBIN: Graph-Theoretic Outlier Rejection | ICRA 2021, arXiv:2011.03659 | 不变量剪枝 | 相容图内点成团,max k-core 近似 max-clique |
| Group-k Consistent Measurement Set Max | IROS 2022/IJRR 2024, arXiv:2209.02658 | PCM 推广 | 成对/group-k 相容性最大化(超图 max-clique) |
| General Place Recognition Survey | arXiv:2405.04812 (T-RO Survey) | 跨模态 PR 综述 | "SLAM 2.0" 下 PR 形式化/分类/数据集 |

#### F. 一致性-FEJ-OC(母树有 OC-EKF/FEJ/NEES/NIS,补根文献与新理论)

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| Observability-based Rules for Consistent EKF SLAM | IJRR 2010, 10.1177/0278364909353640 | FEJ/OC-EKF 根 | 伪可观子空间秩论证+OC-EKF 推导 |
| (其余一致性补强见 C 节不变-等变,二者强耦合) | — | — | InEKF/EqF 是 FEJ 之外消除不一致的对称路径 |

#### G. 鲁棒 M-估计 / 异常值理论(母树有 RANSAC/Huber/Cauchy/Tukey/DCS/PCM,缺统一理论根)

| 条目 | venue/id | 缺口性质 | 补什么 |
|---|---|---|---|
| A General and Adaptive Robust Loss Function | CVPR 2019, arXiv:1701.03077 | 自适应损失根 | 单参数泛化 Cauchy/GM/Welsch/Charbonnier/Huber/L2 |
| Line Processes, Outlier Rejection & Robust Statistics(Black-Rangarajan) | IJCV 1996, 10.1007/BF00131148 | 鲁棒↔外点过程对偶根 | GNC/TEASER 的对偶理论基础 |
| GNC for Robust Spatial Perception | RA-L 2020(ICRA 2020 Best Robot Vision), arXiv:1909.08605 | GNC 根 | B-R 对偶+GNC,70-80% 外点,无需初值 |
| TEASER: Fast and Certifiable Registration | T-RO 2021, arXiv:2001.07715 | 证书化鲁棒配准 | TLS+max-clique+SDP 旋转松弛+GNC(TEASER++),>99% 外点 |
| Outlier-Robust Estimation: Hardness+ADAPT/GNC | T-RO 2022, arXiv:2007.15109 | 鲁棒理论硬度 | 广义 max-consensus/TLS,不可近似性,ADAPT |
| Estimation Contracts for Outlier-Robust Perception | FnT Robotics 2023(monograph), arXiv:2208.10521 | 性能保证理论 | "估计契约"桥接证书化感知与鲁棒/list-decodable 回归 |

#### H. 新发现薄弱处(母树未单列、scout 暴露)

| 薄弱点 | 关键条目 | 为何是缺口 |
|---|---|---|
| 端到端/可微 SLAM 与可微优化工具 | DROID-SLAM(arXiv:2108.10869,已为 KNOWN 锚)、DPVO(arXiv:2208.04726)、DPV-SLAM(arXiv:2408.01654)、BA-Net(arXiv:1806.04807)、gradSLAM(arXiv:1910.10672)、Theseus(arXiv:2207.09442) | 母树有传统 BA/FactorGraph,缺"BA 作为可微层"与端到端轨迹学习 |
| 学习型前端(检测/匹配) | SuperPoint(arXiv:1712.07629)、SuperGlue(arXiv:1911.11763)、LightGlue(arXiv:2306.13643) | 母树视觉层为 ORB/直接法,缺学习特征替代 |
| 度量-语义 / 动态 / 场景图 | Kimera(arXiv:1910.02490)、3D Dynamic Scene Graphs(arXiv:2101.06894)、Khronos(arXiv:2402.13817)、Kimera-Multi(arXiv:2106.14386)、SuMa++(arXiv:2105.11320) | 母树无语义层与动态环境推理 |
| 几何基础模型作前端 | DUSt3R(arXiv:2312.14132)、VGGT(CVPR 2025 Best Paper, arXiv:2503.11651)、MASt3R-SLAM(CVPR 2025, arXiv:2412.12392)、GO-SLAM(arXiv:2309.02436) | 母树无"foundation model→SLAM 前端"范式 |
| 开放词表空间 AI | HOV-SG(RSS 2024, arXiv:2403.17846) | 母树无语言接地空间表示 |
| VI 初始化/可识别性 | Closed-Form VI-SfM(IJCV 2014, 10.1007/s11263-013-0647-7)、Inertial-Only Optimization(ICRA 2020, arXiv:2003.05766) | 母树 init 仅"gravity-scale",缺闭式可识别性与 MAP 初始化 |
| 视觉多视几何根文献 | MVG(Hartley-Zisserman 2004)、BA Modern Synthesis(2000)、Nistér 5-point(TPAMI 2004) | 母树有 epipolar/PnP/triangulation 但缺奠基文献 |

---

### ② 补充必读(分类)

#### 【教材 / 课程 / 教程】
- **Multiple View Geometry in CV (2nd ed)** — 多视几何唯一权威参考(投影几何/基础-本质矩阵/三角化/BA)。Cambridge 2004, ISBN 9780521540513。
- **Quaternion Kinematics for the ESKF** — 四元数约定+IMU 运动模型+误差态 KF 详尽推导,Solà micro-Lie 姊妹篇。arXiv:1711.02508。
- **Tutorial on Aided INS (Lie-Group)** — SE₂(3)/同步观测器/等变滤波面向实现教程。arXiv:2603.07143(单作者 Berkane)。
- **DRIFT(Proprioceptive Invariant State Estimation)** — 不变 KF 入门+开源框架。ICRA 2024, arXiv:2311.04320。
- **Rotation Averaging(tutorial)** — 单/多旋转平均的权威教程。IJCV 2013, 10.1007/s11263-012-0601-0。

#### 【经典奠基论文】
- **Bundle Adjustment — A Modern Synthesis** — BA 的代价/鲁棒/稀疏 Newton-Schur/gauge 不变权威综合。LNCS 1883, 2000, 10.1007/3-540-44480-7_21。
- **Five-Point Relative Pose** — 标定 5 点本质矩阵 10 次多项式解。TPAMI 2004, 10.1109/TPAMI.2004.17。
- **The InEKF as a Stable Observer** — InEKF 稳定性与 group-affine 理论根。IEEE TAC 2017, arXiv:1410.1465。
- **Observability-based Rules for Consistent EKF SLAM** — FEJ/OC-EKF 理论根。IJRR 2010, 10.1177/0278364909353640。
- **Equivariant Systems Theory and Observer Design** — 齐性空间等变滤波理论根。arXiv:2006.08276。
- **SE-Sync** — 证书化全局最优 PGO 奠基。IJRR 2019, arXiv:1612.07386。
- **Black-Rangarajan Duality** — 鲁棒统计↔外点过程对偶,GNC/TEASER 之源。IJCV 1996, 10.1007/BF00131148。
- **Generalized-ICP** — plane-to-plane 概率 ICP。RSS 2009, 10.15607/RSS.2009.V.021。
- **NDT** — distribution-to-distribution 配准根。IROS 2003, 10.1109/IROS.2003.1249285。
- **Point-to-Plane ICP LLS(Kok-Lim Low)** — point-to-plane 误差度量线性最小二乘推导。UNC TR04-004, 2004。
- **Closed-Form VI-SfM(Martinelli)** — VIO 尺度/重力/速度闭式解与可识别性。IJCV 2014, 10.1007/s11263-013-0647-7。
- **Associating Uncertainty with 3D Poses** — SE(3) 上高斯不确定性传播/融合。T-RO 2014, 10.1109/TRO.2014.2298059。
- **ORB-SLAM3** — 特征法 VI-SLAM 系统集大成(Atlas 多图)。T-RO 2021, arXiv:2007.11898。

#### 【顶刊顶会综述】
- **Continuous-Time State Estimation: A Survey** — 样条+GP 统一连续时间综述。arXiv:2411.03951。
- **Estimation Contracts for Outlier-Robust Perception(monograph)** — 证书化感知↔鲁棒统计的性能保证理论。FnT Robotics 11(2-3), 2023, arXiv:2208.10521。
- **A Review of Point Cloud Registration(monograph)** — 配准分类法/ICP 变体/不确定性。FnT Robotics 4(1), 2015, 10.1561/2300000035。
- **LiDAR Odometry Survey** — LiDAR/LIO 按传感器集成分类。arXiv:2312.17487(Intell. Service Robotics 2024)。
- **How NeRFs and 3DGS are Reshaping SLAM: a Survey** — 辐射场 SLAM 首个全面综述。arXiv:2402.13255。
- **A Survey on Deep Learning for Localization and Mapping** — DL-SLAM 分类法。arXiv:2006.12567。
- **Local Feature Matching Using Deep Learning: A Survey** — 学习匹配 detector-based/free 分类。Information Fusion 2024, arXiv:2401.17592。
- **General Place Recognition Survey** — 跨模态 PR 形式化/数据集。arXiv:2405.04812(T-RO Survey)。
- **Advances in Feed-Forward 3D Reconstruction: A Survey** — 前馈 3D/基础模型分类含 SLAM 应用。arXiv:2507.14501。

#### 【近年前沿 2023-2026】
- **Affine EKF(Aff-EKF)** — 可观性维持充要条件+仿射设计。arXiv:2412.10809 (2024)。
- **Equivariant Symmetries for INS** — 统一对称分类,EKF 变体皆为 EqF 实例。Automatica 2025, arXiv:2309.03765。
- **MSCEqF** — 等变 MSCKF,对称群含 bias+内外参。RA-L 2024, arXiv:2311.11649。
- **EqVIO** — 等变 VIO。T-RO 2023, arXiv:2205.01980。
- **Equivariant IMU Preintegration (Galilean)** — Gal(3) 几何耦合预积分。RA-L 2025, arXiv:2411.05548。
- **CORA** — 证书化 RA-SLAM(2024 King-Sun Fu Best Paper)。T-RO 2024, arXiv:2302.11614。
- **Chordal Sparsity for Fast Global Optimality** — 弦稀疏 SDP 线性化。WAFR 2024, arXiv:2406.02365。
- **C2P** — 无歧义证书化相对位姿。CVPR 2024 Highlight, arXiv:2312.05995。
- **Certifiably Optimal Anisotropic Rotation Averaging** — 各向异性证书化旋转平均。arXiv:2503.07353 (2025)。
- **KISS-ICP** — 极简现代 LiDAR 里程计(无 IMU/无调参)。RA-L 2023, arXiv:2209.15397。
- **MASt3R-SLAM** — 基础模型驱动实时稠密 SLAM。CVPR 2025, arXiv:2412.12392。
- **VGGT** — 前馈 3D 基础模型(CVPR 2025 Best Paper)。arXiv:2503.11651。
- **Khronos** — 时空度量-语义动态 SLAM。RSS 2024, arXiv:2402.13817。
- **Gaussian-LIC** — LiDAR-Inertial-Camera+3DGS 融合。ICRA 2025, arXiv:2404.06926。
- **DPV-SLAM** — 单 GPU 端到端深度 SLAM+回环。ECCV 2024, arXiv:2408.01654。
- **HOV-SG** — 开放词表分层 3D 场景图。RSS 2024, arXiv:2403.17846。

---

### ③ 开源/代码与教程

> 母树标准栈(GTSAM / Ceres / g2o / Sophus / manif / OpenVINS / ORB-SLAM 系)假定已知;此处仅列 scout 验证产生的新代码工件,并补注少量标准栈定位以便挂接。

| 名称 | 覆盖主题 | 用途 | url / id |
|---|---|---|---|
| STEAM / pySTEAM | 连续时间 GP-prior 批量估计(SO(3)/SE(3)) | STEAM 引擎,GN 风格约束敏感扰动;steam_icp LIO 底座 | https://github.com/utiasASRL/steam |
| basalt-headers | Rᵈ/SO(3)/SE(3) 均匀 B-样条+高效李群导数 | 样条融合规范代码(Sommer CVPR20 实现) | https://gitlab.com/VladyslavUsenko/basalt-headers |
| Theseus | 可微非线性最小二乘(李群/二阶/稀疏/隐式微分) | PyTorch DNLS 库,端到端可微 SLAM 工具 | https://github.com/facebookresearch/theseus |
| TEASER++ | 证书化鲁棒点云配准(TLS+max-clique+GNC) | 极端外点配准 | https://github.com/MIT-SPARK/TEASER-plusplus |
| GNC-and-ADAPT | 通用鲁棒估计求解器(广义 max-consensus/TLS) | 无需噪声界的鲁棒求解 | https://github.com/MIT-SPARK/GNC-and-ADAPT |
| Affine-EKF | 一致性 EKF(仿射变换设计) | 可观性维持 EKF 参考实现 | https://github.com/YangSONG-SLAM/Affine-EKF |
| MSCEqF | 等变 MSCKF(bias+内外参在群内) | 等变 VINS 滤波器 | https://github.com/aau-cns/MSCEqF |
| KISS-ICP | 极简 LiDAR 里程计 | 无 IMU/无调参 LiDAR odom 基线 | https://github.com/PRBonn/kiss-icp |
| semantic_suma (SuMa++) | surfel+语义 LiDAR SLAM | 投影 ICP+语义约束 | https://github.com/PRBonn/semantic_suma |
| STD / BTC descriptor | 三角(+二进制)3D 描述子回环 | LiDAR 回环检测与验证 | https://github.com/hku-mars/STD , https://github.com/hku-mars/btc_descriptor |
| OverlapNet | range-image overlap+yaw 学习回环 | LiDAR 回环 | https://github.com/PRBonn/OverlapNet |
| DUSt3R | 无位姿稠密 pointmap 回归 | 几何基础模型前端 | https://github.com/naver/dust3r |
| VGGT | 前馈 3D 基础模型 | <1s 推相机/深度/pointmap/track | https://github.com/facebookresearch/vggt |
| LightGlue | 自适应早停学习匹配 | SuperGlue 高效后继 | https://github.com/cvg/LightGlue |
| Kimera | 度量-语义 VIO+鲁棒 PGO+网格 | 度量-语义 SLAM 系统 | https://github.com/MIT-SPARK/Kimera |
| C2P | 无歧义证书化相对位姿 | 两视位姿求解 | https://github.com/javrtg/C2P |

---

### ④ 前沿桥资源(学习型 / 神经地图 / 语义 / 可微优化)

**综述层(先读)**
- 辐射场 SLAM:How NeRFs and 3DGS are Reshaping SLAM(arXiv:2402.13255)
- DL-SLAM 分类:A Survey on DL for Localization and Mapping(arXiv:2006.12567)
- 学习匹配:Local Feature Matching Using DL: A Survey(arXiv:2401.17592)
- 前馈 3D/基础模型:Advances in Feed-Forward 3D Reconstruction: A Survey(arXiv:2507.14501)

**学习型前端**:SuperPoint(arXiv:1712.07629)→ SuperGlue(arXiv:1911.11763)→ LightGlue(arXiv:2306.13643)

**端到端 / 可微优化**:BA-Net(arXiv:1806.04807)、gradSLAM(arXiv:1910.10672)、Theseus(arXiv:2207.09442)、DPVO(arXiv:2208.04726)、DPV-SLAM(arXiv:2408.01654)、GO-SLAM(arXiv:2309.02436)

**神经/3DGS 地图地标系统**:iMAP(arXiv:2103.12352)→ NICE-SLAM(arXiv:2112.12130)→ Point-SLAM(arXiv:2304.04278)→ SplaTAM(arXiv:2312.02126)/ MonoGS(arXiv:2312.06741)→ Photo-SLAM(arXiv:2311.16728)→ NeRF-SLAM(arXiv:2210.13641)

**语义 / 度量-语义 / 动态 / 场景图**:Kimera(arXiv:1910.02490)→ 3D Dynamic Scene Graphs(arXiv:2101.06894)→ Kimera-Multi(arXiv:2106.14386)→ Khronos(arXiv:2402.13817);开放词表:HOV-SG(arXiv:2403.17846)

**基础模型作前端**:DUSt3R(arXiv:2312.14132)→ VGGT(arXiv:2503.11651, CVPR25 Best Paper)→ MASt3R-SLAM(arXiv:2412.12392)

---

### ⑤ 把补充挂回母树各层的研读顺序建议

| 母树层 | 先决(KNOWN) | 研读顺序(新补) |
|---|---|---|
| **Math/概率** | linalg, prob, NEES/NIS | Black-Rangarajan 对偶(IJCV96)→ Barron 自适应损失(CVPR19)→ 作为 ③ 鲁棒理论根 |
| **Lie 群** | SO(3)/SE(3)/Sim(3), Adjoint, 左右 Jacobian | Barfoot-Furgale SE(3) 高斯(T-RO14)→ Sommer 累积 B-样条导数(CVPR20)→ Tirado SE(3) 样条 Jacobian(RA-L22) |
| **Filter/一致性** | EKF/ESKF/MSCKF/InEKF/OC-EKF | ① Huang FEJ/OC-EKF 根(IJRR10)→ ② Barrau-Bonnabel InEKF 观测器(TAC17)+EKF-SLAM 一致性(1510.06263)→ ③ Mahony 等变系统理论(2006.08276)→ EqF(CDC20 2107.05193 / ARC 2022 2108.09387)→ ④ EqVIO/MSCEqF/INS 对称(2025)→ Aff-EKF(2412.10809);教程穿插 DRIFT + Aided-INS tutorial |
| **Optimization/证书化** | GN/LM/BA/PoseGraph/iSAM2/Schur/gauge | ① BA Modern Synthesis(2000)→ ② Lagrangian Duality in SLAM(1506.00746)→ SE-Sync(1612.07386)→ ③ Rotation Averaging 教程(IJCV13)→ Shonan/Sparse-BSOS/Strong-Duality → ④ QUASAR→TEASER→One-Ring→STRIDE(鲁棒证书)→ ⑤ CORA/SCORE/Chordal/Distributed-PGO/SIM-Sync(前沿) |
| **连续时间(新层)** | — | Furgale 基函数(ICRA12→IJRR15)→ Barfoot 稀疏 GP(1412.0630→IROS15→1705.06020)→ 样条派(CVPR20→RA-L22→spline-VIO)→ GP-vs-Spline 对比(2402.00399)→ CT-survey(2411.03951)→ 落地 CT-LIO(2402.06174);代码 STEAM/basalt-headers |
| **Visual** | pinhole/epipolar/PnP/BA/ORB/DSO | MVG 教材 → Nistér 5-point(TPAMI04)→ 光度标定 TUM-Mono(1607.02555)→ ORB-SLAM3(2007.11898)→ 学习前端 SuperPoint/Glue/LightGlue |
| **Inertial** | IMU/bias/preintegration/ESKF | Solà ESKF(1711.02508)→ Martinelli 闭式可识别性(IJCV14)→ Inertial-Only Init(2003.05766)→ Galilean 等变预积分(2411.05548) |
| **LiDAR** | ICP/NDT/LOAM/FAST-LIO2/CT-ICP | NDT(IROS03)+GICP(RSS09)+Point-to-Plane LLS(TR04-004)→ 配准 monograph(FnT15)→ KISS-ICP(RA-L23)/SuMa++(IROS19)→ LiDAR-odom survey(2312.17487) |
| **地图表示(新层)** | (母树主为特征/surfel) | OctoMap → Voxblox → nvblox → iMAP → NICE-SLAM → Point-SLAM → SplaTAM/MonoGS → Photo-SLAM/NeRF-SLAM;survey 2402.13255 先行 |
| **PR+数据关联(新层)** | (母树仅 loop/Sim3) | DBoW2 → NetVLAD/PointNetVLAD → STD/BTC/ISC/OverlapNet(LiDAR)→ JCBB → ROBIN → Group-k(PCM 推广)→ PR survey 2405.04812 |
| **LVI/融合** | LVI-SAM, factor-graph fusion | Gaussian-LIC(LIC+3DGS)→ 度量-语义 Kimera 系 → 动态 Khronos |
| **前沿桥(新层)** | — | DL-SLAM survey(2006.12567)→ DROID-SLAM→DPVO→DPV-SLAM→GO-SLAM(端到端)→ 可微 BA-Net/gradSLAM/Theseus → 基础模型 DUSt3R→VGGT→MASt3R-SLAM → 空间 AI:Dynamic Scene Graphs→HOV-SG |

> **强耦合提示**:C(不变-等变)与 F(一致性-FEJ-OC)应合并研读——FEJ/OC-EKF 与 InEKF/EqF 是解决同一"伪可观/不一致"问题的两条路线(几何对称 vs 雅可比固定),Automatica 2025(2309.03765)给出二者统一视角,宜作为该主题收口阅读。



---

# 第 IV 部 · 标定 补遗与资源（122 条 · Workflow）


## 多传感器SLAM标定：缺口与资源综合报告 (Gap-and-Resource Report)

> 说明：本报告仅纳入 scout 结果中 `status=verified/corrected` 的条目；与 v1 报告 KNOWN 列表重复者已剔除（如 Sensors2025 spatiotemporal-calib、KNOWN 中已有的 iKalibr/eKalibr/RIs-Calib 系列在 KNOWN 未列出故保留）；跨 6 个 facet 已去重。所有 facet 的 `accessible_tools` 均提供了可访问检索工具（arXiv/IEEE/PMC/WebSearch 等），**无 facet 出现 accessible_tools=[] 的盲检情况**，覆盖性无隐藏盲区（见文末说明）。

---

### ① v1 报告缺口（按理论/能力类别）

v1 报告偏重"系统级在线标定方法"（VINS/LIC/LVI 系列）与 B-spline 连续时间标定，但在**标定的数学基础与可靠性理论**上覆盖稀薄。下列 8 类为 KNOWN 列表明显缺失：

| 缺口类别 | 缺口描述 | 关键补强文献 (verified) |
|---|---|---|
| **手眼 AX=XB / 运动法标定** | KNOWN 完全无 AX=XB 经典闭式解谱系，亦无 certifiable/QCQP-SDP 全局最优现代解 | Tsai-Lenz 1989；Park-Martin 1994 (Lie)；Daniilidis 1999 (对偶四元数)；Horaud-Dornaika 1995；Taylor-Nieto TRO16 (motion-based multimodal)；Brookshire-Teller RSS12；Giamou RA-L19 (certifiable) |
| **可观性基础 / FEJ / OC-EKF 一致性** | KNOWN 有 Yang 可观性应用，但缺非线性可观性根理论与一致性估计器设计源头 | Hermann-Krener TAC77 (可观性秩条件)；Huang-Mourikis-Roumeliotis IJRR10 (OC-EKF)；Huang ISER08/09 (FEJ 起源) |
| **退化检测 (Degeneracy Detection)** | KNOWN 完全无退化因子/TSVD/概率退化检测线，仅有 Yang 退化运动分析 | Zhang-Kaess-Singh ICRA16 (退化因子+solution remapping，奠基)；Hinduja IROS19 (退化感知因子)；X-ICP TRO24；Hatleskog-Alexis RA-L24 (概率退化)；Tuna 2408.11809 (退化处理实战对比) |
| **标定不确定性传播** | KNOWN 无标定协方差量化/consider-state 机制 | Censi ICRA07 (ICP 协方差闭式)；Geneva CVPR19 (Schmidt-EKF/consider-state)；M-LOAM TRO21 (外参协方差传播进里程计) |
| **主动激励 / 最优运动标定** | KNOWN 标定均为被动；缺"规划运动使参数可观"主动范式 | Hausman RA-L17 (可观性感知轨迹优化)；Maye IV13/IJRR16 (自监督锁定不可观方向)；Nobre-Heckman IJRR19 (RL 引导标定)；Wang 2506.13420 (FIM 驱动主动标定+声学模态) |
| **时间同步硬件 / 离散时间高效标定** | KNOWN 时间偏移多为软件估计；缺离散时间高效替代连续时间方案 | DT-VI-Calib (2509.12846, ICRA26) 离散时间 IMU-cam 时空标定（对照 B-spline）；GPTR (2410.22931) GP 连续时间替代 B-spline 基底 |
| **单传感器内参标定** | KNOWN 有 IMU 内参，但缺事件相机内参/联合内外参/IMU-里程计内参谱系 | eKalibr (2501.05688, RA-L25) 事件相机内参；joint-lidar-camera-calib (2308.12629) 联合内外参；Song 2510.08880 IMU-里程计内参+GNSS |
| **新兴传感器 (event/4D-radar/thermal)** | KNOWN 几乎无事件相机、4D 毫米波雷达、热成像标定 | 详见 ② 与 ③（eKalibr 系列、RIs-Calib、EvMultiCalib、4D-radar 系列、lvt2calib 热成像等） |

---

### ② 新增重要论文（按 venue 分层；每条 1 行 why）

#### 【顶刊顶会 IJRR / TRO / RSS / CVPR / 旗舰】
| 论文 | venue/year | sensors | why (1行) |
|---|---|---|---|
| CMRNext | **TRO 2025** (2402.00129) | Cam+LiDAR | 学习型跨模态匹配(光流+PnP)实现零样本泛化 LiDAR-相机外参+单目地图定位 |
| Taylor-Nieto Motion-Based Calib | **TRO 2016** | Cam+LiDAR+GPS/INS | 经典无目标多模态外参+时延联合估计，运动法标定基石 |
| Brookshire-Teller | **RSS 2012** | 通用刚联传感器 | 对偶四元数 ML 手眼，李代数噪声建模→FIM/CRB（运动法+退化+不确定性三线合一） |
| Giamou et al. Certifiable Calib | **RA-L 2019** (1809.03554) | egomotion | 手眼建为 QCQP，SDP 对偶证明全局最优（含未知尺度） |
| Huang-Mourikis-Roumeliotis OC-EKF | **IJRR 2010** | EKF-SLAM | 揭示线性化伪可观方向并给出 OC-EKF/FEJ 修正，一致性理论源头 |
| Nobre-Heckman RL Calib | **IJRR 2019** | Cam+IMU | 将 VI 标定建模为 MDP，RL 推荐易行运动使参数可观 |
| X-ICP | **TRO 2024** (2211.16335) | 3D LiDAR | 细粒度逐方向可定位性检测+约束 ICP，极端退化环境鲁棒 |
| M-LOAM | **TRO 2021** (2010.14294) | 多 LiDAR | 在线多 LiDAR 外参+协方差量化+退化感知传播 |
| FusionPortableV2 | **IJRR 2025** (2404.08563) | LiDAR+stereo+event+IMU+GPS | 4 平台统一多传感器 SLAM 数据集，公开标定 |
| Hilti-Oxford | **RA-L 2022** (2208.09825) | LiDAR+5cam+IMU | 微米级扫描仪验证外参+硬件时间同步，标定金标准 |

#### 【RA-L / ICRA / IROS / T-IV / TIM / MFI】
| 论文 | venue/year | sensors | why (1行) |
|---|---|---|---|
| eKalibr | **RA-L 2025** (2501.05688) | Event cam | 从原始事件第一性原理做事件相机内参（圆栅+法向流椭圆拟合） |
| EF-Calib | **RA-L 2024** (2405.17278) | Event+frame cam | 连续时间 B-spline 联合事件+帧相机内/外参+时延 |
| DT-VI-Calib | **ICRA 2026** (2509.12846) | IMU+Cam | 离散时间(非 B-spline)目标式 IMU-相机时空标定，超快速 |
| GRIL-Calib | **RA-L 2024** (2312.14035) | LiDAR+IMU | 地面机器人平面运动下解 6-DoF LiDAR-IMU，破 LI-Calib 退化 |
| MIAS-LCEC (C3M) | **T-IV 2024** (2404.18083) | LiDAR+Cam | 大视觉模型(MobileSAM)跨模态掩码匹配，在线无目标 |
| Hinduja Degeneracy-Aware Factors | **IROS 2019** | sonar/LiDAR | 退化感知 ICP+部分约束回环因子（DOI 修正为 ...8968577） |
| Hatleskog-Alexis DRPM | **RA-L 2024** (2410.10784) | LiDAR | 概率退化检测替代手调特征值阈值 |
| Hausman Obs-Aware Traj | **RA-L 2017** (1604.07905) | GPS-IMU | 可观性感知轨迹优化，主动激励标定奠基 |
| Multi-LiCa | **MFI 2024** (2501.11088) | 多 LiDAR | 无目标/无初值运动法多 LiDAR-LiDAR 标定 |
| Robust 2DGS Calib | **RA-L 2025** (2504.00525) | LiDAR+Cam | 2D 高斯泼溅可微渲染无目标外参，新范式 |
| TLC-Calib | **RA-L 2026** (2504.04597) | LiDAR+Cam | 神经高斯+可微渲染联合优化位姿，新范式 |
| EKF Radar-Inertial td | RA-L line (2502.00661) | 4D radar+IMU | EKF 内在线雷达-IMU 时间偏移自标定 |

#### 【arXiv / lab-preprint】
| 论文 | id/year | sensors | why (1行) |
|---|---|---|---|
| eKalibr-Stereo | 2504.04451 (2025) | 事件双目 | 连续时间事件双目外参+时延标定 |
| eKalibr-Inertial | 2509.05923 (2025) | Event+IMU | 首个开源事件-惯性时空标定 |
| EvMultiCalib | 2508.12564 (2025) | Event+异构 | 法向流估角速度→无目标时间+旋转标定 |
| OKVIS2-X | 2510.04612 (2025) | Cam+IMU+LiDAR/depth+GNSS | 统一关键帧 VI-SLAM 含在线相机外参自标定（ETH/TUM） |
| Wang Obs-Aware Active Calib | 2506.13420 (2025) | mic-array+LiDAR+encoder | FIM 最小特征值驱动 B-spline 轨迹优化（主动+声学模态） |
| Lv 多LiDAR+多cam+IMU | 2501.02821 (2025) | 多 LiDAR+多 cam+IMU | 全套无重叠 FoV 无目标连续时间联合内外参+时延 |
| Wang LiDAR-to-GINS | 2507.08349 (2025) | 多 LiDAR+GNSS-INS | 无目标多 LiDAR-GNSS/INS 外参，平面运动可观性处理 |
| DLBAcalib | 2507.09176 (2025) | 非重叠多 LiDAR | 双 LBA 无初值鲁棒(容忍 0.4m/30°) inter-LiDAR 外参 |
| TLC-Calib (neural GS) | 2504.04597 (2025) | LiDAR+Cam | 见上(RA-L26)，differentiable rendering 范式 |
| EdO-LCEC | 2502.00801 (2025) | LiDAR+Cam | 首个环境驱动在线标定，场景判别器+双路匹配 |
| CalibRefine | 2502.17648 (2025) | LiDAR+Cam | ViT 交叉注意力迭代后精修，全自动在线无目标 |
| Song IMU-odom+GNSS | 2510.08880 (2025) | IMU+轮速+GNSS | 紧耦合在线里程计尺度+IMU-里程计外参（原始 GNSS） |
| 2D GS Calib | 2504.00525 | LiDAR+Cam | (亦 RA-L25) GS 几何+光度无目标 |
| One-target-tri-modal | 2511.12291 (**BMVC 2025**) | LiDAR+RGB+event | 单 3D 靶(平面+ChArUco+LED)一次性三模态联合外参 |
| 4D-Radar-Cam 3DUPnP | 2507.19829 (2025) | 4D radar+Cam | 球坐标噪声建模的不确定性 PnP 外参 |
| CLRNet | 2603.15767 (2026, 投 T-IV) | Cam+LiDAR+4D radar | 端到端深度网联合/成对三模态外参 |
| RLCNet | 2512.08262 (2025) | LiDAR+radar+Cam | 端到端同时在线三模态标定+异常剔除 |
| Radar-IMU CT Calib | 2603.19958 (2026) | Radar+IMU | 立方 B-spline 在线雷达-IMU 时空自标定（Persic/Markovic 组） |
| GNSS Lever-Arm Globally-Optimal | 2406.09866 (2024) | 多 GNSS 天线 | 全局最优杆臂标定（拉格朗日对偶+平面运动扩展） |

---

### ③ 开源库与工具链

| repo | 传感器 | 标定对象 | 实验室/作者 | url |
|---|---|---|---|---|
| OpenCalib/SensorsCalibration | Cam/LiDAR/IMU/Radar | 内参+工厂+在线全对组合 | PJLab-ADG (上海AILab) | github.com/PJLab-ADG/SensorsCalibration |
| direct_visual_lidar_calibration | LiDAR(各类)+Cam(各投影) | 无目标 LiDAR-相机外参(SuperGlue+NID) | AIST (Koide) | github.com/koide3/direct_visual_lidar_calibration |
| livox_camera_calib | Livox 固态 LiDAR+Cam | 边缘法无目标外参 | HKU-MARS | github.com/hku-mars/livox_camera_calib |
| mlcc | 多小 FoV LiDAR+多 Cam | 自适应体素多 LiDAR-多相机外参 | HKU-MARS | github.com/hku-mars/mlcc |
| joint-lidar-camera-calib | LiDAR+Cam | 联合内参+外参(平面约束BA) | HKU-MARS | github.com/hku-mars/joint-lidar-camera-calib |
| tier4/CalibrationTools | Cam/LiDAR/Radar | 生产级 ROS2 多对标定(Autoware) | TIER IV / Autoware | github.com/tier4/CalibrationTools |
| MC-Calib | N 相机(透视+鱼眼) | 多相机(含非重叠)内+外参 | Rameau et al. | github.com/rameau-fr/MC-Calib |
| CamOdoCal | 多相机(含鱼眼/全向)+轮速 | 相机内参+相机-里程计手眼 | ETH (Heng) | github.com/hengli/camodocal |
| InfrasCal | 多相机(鱼眼) | 基于地图/基础设施内+外参 | ETH (Lin et al.) | github.com/youkely/InfrasCal |
| imu_utils | IMU | Allan 方差噪声/偏置估计 | gaowenliang | github.com/gaowenliang/imu_utils |
| allan_variance_ros | IMU | Allan 方差(直出 Kalibr YAML) | Oxford ORI-DRS | github.com/ori-drs/allan_variance_ros |
| RIs-Calib | 多 3D radar+多 IMU | 连续时间 radar-IMU 时空标定 | WHU (Unsigned-Long) | github.com/Unsigned-Long/RIs-Calib |
| eKalibr | Event cam(+IMU/stereo) | 事件相机内参(+companion 外参) | WHU (Unsigned-Long) | github.com/Unsigned-Long/eKalibr |
| GRIL-Calib | LiDAR+IMU(地面机器人) | 平面运动约束无目标外参 | Yonsei | github.com/Taeyoung96/GRIL-Calib |
| LiDAR2INS | LiDAR+INS(IMU/GNSS) | LiDAR-INS 杆臂外参(道路场景) | OpenCalib/PJLab | github.com/OpenCalib/LiDAR2INS |
| JointCalib | Cam+LiDAR | 4 圆孔板联合内参+外参 | OpenCalib/PJLab | github.com/OpenCalib/JointCalib |
| ACSC | 非重复扫描固态 LiDAR+Cam | 自动固态 LiDAR-相机外参 | 北航 (Cui et al.) | github.com/HViktorTsoi/ACSC |
| CalibAnything | LiDAR+Cam | SAM 零训练无目标外参 | PJLab (Luo et al.) | github.com/OpenCalib/CalibAnything |
| lvt2calib (L2V2T2Calib) | LiDAR+视觉+**热成像**Cam | 统一目标式 LiDAR-视觉-热外参 | Zhang et al. | github.com/Clothooo/lvt2calib |
| easy_handeye | Cam+机械臂 | 手眼 AX=XB(eye-in-hand/on-base) | IFL-CAMP (TUM) | github.com/IFL-CAMP/easy_handeye |
| Multi_LiCa | 多 LiDAR | 无目标/无初值运动法 L2L 标定 | TUM-FTM | github.com/TUMFTM/Multi_LiCa |
| DLBAcalib | 非重叠多 LiDAR | 双 LBA 无初值外参 | ZJUT (Silentbarber) | github.com/Silentbarber/DLBAcalib |
| Multisensor-Calibration | mic-array+LiDAR+encoder | 可观性驱动主动标定 | SUSTech AISLAB | github.com/AISLAB-sustech/Multisensor-Calibration |
| 2DGS RobustCalibration | LiDAR+Cam | 2D 高斯泼溅无目标外参 | UTokyo (Oishi Lab) | github.com/ShuyiZhou495/RobustCalibration |
| EvMultiCalib | Event+异构 | 时间+旋转标定 | HNU-NAIL | github.com/NAIL-HNU/EvMultiCalib |
| GPTR | Vis/IMU/UWB/LiDAR | GP 连续时间标定基底库 | Nguyen/Barfoot et al. | (开源库，见 arXiv 2410.22931) |
| certifiable-calibration | egomotion | 手眼全局最优(QCQP-SDP) | UTIAS STARS | github.com/utiasSTARS/certifiable-calibration |
| ocekf-slam | EKF-SLAM | OC-EKF 一致性参考实现 | RPNG | github.com/rpng/ocekf-slam |
| M-LOAM | 多 LiDAR | 在线外参+协方差传播 | HKUST (Jiao) | github.com/gogojjh/M-LOAM |

---

### ④ 优秀综述（顶刊优先）

| 综述 | venue/year | 主题轴 | 备注 |
|---|---|---|---|
| An et al. LiDAR-Camera Extrinsic Survey | **TITS 2024** (25(11):15342) | LiDAR-相机外参专题 | 显式/隐式对应分类+离线→在线框架，顶刊专题综述 |
| Cadena et al. Past-Present-Future SLAM | **TRO 2016** (1606.05830) | SLAM 教程/立场论文 | 将自标定/可观性列为开放前沿，经典 |
| Huang VIN: A Concise Review | **ICRA 2019** (1906.02650) | VINS 标定/可观性 | 在线相机-IMU 外参/时延/内参+可观性综述 |
| Chen-Pan Deep Learning Inertial Positioning | **TITS 2024** (2303.03757) | 数据驱动惯性 | 含传感器标定+多传感器融合 |
| Dellaert-Kaess Factor Graphs | **FnT Robotics 2017** | 因子图教程 | GTSAM/批量自标定理论底座，规范教程 |
| Talbot et al. Continuous-Time State Estimation | arXiv 2024 (2411.03951) | 连续时间(spline+GP) | 连续时间时空标定理论骨干，最详尽综述 |
| Sola et al. Micro Lie Theory | arXiv 2018 (1812.01537) | 李群估计教程 | 外参参数化/可观性的流形微积分，manif 库 |
| Davison-Ortiz FutureMapping 2 (GBP) | arXiv 2019 (1910.14139) | 因子图/GBP 教程 | 分布式推断教程，补 Dellaert-Kaess |
| Liao et al. DL Camera Calibration | arXiv 2023 (2303.10559) | 学习型内/外参 | 含跨视/跨传感器模型+公开 benchmark，首个学习标定综述 |
| Cohen-Klein Inertial Nav Meets DL | arXiv 2023 (2307.00014) | 数据驱动 IMU 标定/去噪 | IMU 内参学习轴 |
| An et al. Multi-modal Fusion AD | arXiv 2022 (2202.02703) | 融合侧(含 misalignment) | 上下文综述 |
| Shi et al. Radar-Camera Fusion | arXiv 2024 (2410.19872) | radar-相机标定轴 | 专设标定/对齐章节 |
| Zhang et al. Event Sensor Fusion Odometry | arXiv 2024 (2410.15480) | 事件相机融合轴 | 标定处理仅轻度支持 |
| Zhang et al. Cooperative Visual-LiDAR Calib (V2X) | arXiv 2024 (2405.10132) | 基础设施/V2X 标定轴 | 车/路/协同三视角 |
| Ruan et al. MSF for Embodied AI | arXiv 2025 (2506.19769) | 融合侧(具身 AI) | 标定为边缘话题 |

---

### ⑤ 标定级数据集

| 名称 | 传感器 | 提供的标定 GT / 同步 |
|---|---|---|
| FusionPortableV2 (IJRR25) | LiDAR+stereo frame+stereo event+IMU+GPS/INS，4 平台 | 公开标定细节+GT 轨迹+RGB 点云图 |
| FusionPortable (IROS22) | LiDAR+stereo+event+IMU+GPS | 硬件同步，发布原始+标定数据 |
| M2DGR (RA-L22) | 6 鱼眼+天空 RGB+红外+event+VI+IMU+LiDAR+GNSS+RTK | 全标定/同步，3 源 GT(MoCap+激光跟踪+RTK) |
| M3DGR / Ground-Fusion++ (IROS25) | GNSS+RGB-D+LiDAR+IMU+轮速 | 诱导退化(视觉/LiDAR退化/打滑/GNSS拒止)基准 |
| Hilti-Oxford (RA-L22) | LiDAR+5cam+IMU | **微米级扫描仪验证外参**+在线硬件时间同步+mm 级位姿 GT |
| Hilti SLAM Challenge 2023 (arXiv24) | LiDAR+多cam+IMU，多星座 | mm 级基准点 GT，单+多会话 |
| NTU VIRAL (IJRR22) | 2×LiDAR+2×硬同步 cam+多 IMU+多 UWB(空中) | 含标定结果+激光跟踪 GT，UWB 测距模态 |
| VBR (ICRA24) | RGB+LiDAR+IMU+GPS | 内+外参+时间同步流程，LiDAR-BA 精化 RTK 的 6-DoF GT |
| GEODE (IJRR26) | 多异构 LiDAR+stereo+IMU；64 轨/>64km/7 退化设定 | 退化/可观性基准 |
| USTC FLICAR (IJRR23) | 4×LiDAR+2 stereo+2 mono+IMU+GNSS/INS，空/地双平台 | 良好标定/同步，激光跟踪 mm 级 GT，发布原始标定集 |
| TUM VI (IROS18) | stereo+IMU，硬件同步 | 光度标定+MoCap GT，相机-IMU 时空标定经典 |
| TUM-VIE (IROS21) | stereo event+stereo frame+IMU | 全硬件时间戳同步+MoCap GT |
| VECtor (RA-L22) | event stereo+regular stereo+RGB-D+LiDAR+IMU | 全硬件同步+精确外参+MoCap GT |
| Boreas (IJRR23) | 128 线 LiDAR+扫描 radar+5MP cam，>350km 多季 | cm 级后处理 GNSS-INS GT，radar-LiDAR-cam 外参 |
| Boreas Road Trip (arXiv26) | FLIR cam+Navtech RAS6 Doppler radar+Velodyne+Aeva FMCW LiDAR+IMU+轮速 | 精确内+外参，cm 级 Applanix GT，FMCW/Doppler |
| ColoRadar (IJRR22) | 2×FMCW mmWave radar(含原始 ADC/热图/点云)+LiDAR+IMU | 原始 radar 内/外参，精确 GT |
| SubT-MRS (CVPR24) | LiDAR+鱼眼+IMU+**thermal**，多平台，>30 退化场景 | 联合标定+退化鲁棒性指标 |
| Pohang Canal (IJRR23) | stereo+红外+全向 cam+3×LiDAR+marine radar+GPS+AHRS | 海事多模态外参，7.5km |
| WHU-Helmet (TGRS23) | 头盔 Livox LiDAR+IMU+cam | 光纤 IMU+PTC GT，GNSS 拒止可穿戴 |
| ShanghaiTech Mapping Robot (arXiv24) | RGB/RGB-D/event/IR cam+LiDAR+mmWave radar+IMU+超声+GNSS-RTK | 中心化电源+同步，逐传感器标定流程+发布标定数据 |
| MUN-FRL (IJRR24) | mono+IMU+LiDAR+RTK-GNSS，无人机+全尺寸直升机 | 内+外参+原始标定集+硬件时间戳+时空对齐 GT |
| Brno Urban (ICRA20) | 4×WUXGA cam+2×LiDAR+IMU+IR+差分 RTK-GNSS，>350km | cm 级 RTK，亚 ms 时间戳 |
| GRACO (RA-L23) | LiDAR+stereo+GNSS/INS，地+空协同 | 自制同步模块 ms 级，cm 级 RTK GT |
| M2UD (arXiv25) | 多模 LiDAR+cam+IMU(Livox/Fu Zhang)，不平地形 | RTK 平滑 GT+激光扫描仪高精地图 GT |
| Odyssey (arXiv25) | LiDAR+IMU，导航级 INS(RLG) | 首个 RLG-INS GT，长时 GNSS 拒止 |
| RTK-SLAM Dataset (arXiv26) | 手持 LiDAR+cam+IMU+RTK-GNSS | **全站仪独立 GT**(RTK 仅输入)，发布标定文件，揭示 SE(3) ATE 低估陷阱 |
| MOANA (arXiv24) | 短程 LiDAR+W 波段 radar+X 波段 marine radar+stereo | 多波段 radar-LiDAR-cam 标定/里程计，7 海事序列 |

---

### ⑥ 重点实验室产出地图

| 实验室 | 代表性新增产出 (本报告内) | 主攻方向 |
|---|---|---|
| **HKU-MARS** (Fu Zhang) | livox_camera_calib, mlcc, joint-lidar-camera-calib；M2UD 数据集(co-author) | 固态/多 LiDAR-相机无目标外参、自适应体素、联合内外参 |
| **UDel/GWU RPNG** (Guoquan Huang) | OC-EKF(IJRR10), FEJ(ISER08), Schmidt-EKF(CVPR19), VIN Review(ICRA19); ocekf-slam | 可观性/一致性/consider-state 理论根基(VINS 自标定背书) |
| **WHU 测绘** (Xingxing Li, iKalibr/eKalibr 系) | eKalibr/-Stereo/-Inertial, RIs-Calib | 连续时间事件相机&radar-IMU 时空标定 |
| **ETH ASL/RSL** (Hutter, Cadena) | X-ICP(TRO24), Tuna 2408.11809, Talbot CT survey, OKVIS2-X(TUM/ETH 谱系) | 退化检测/可定位性、连续时间状态估计综述 |
| **UZH RPG** (Scaramuzza) | Hilti-Oxford & Hilti2023 数据集(co-author), Boreas(间接) | mm 级标定 GT 基准、事件视觉 |
| **TUM** (Cremers, Leutenegger, FTM) | TUM VI/TUM-VIE 数据集, Multi_LiCa, easy_handeye(IFL-CAMP), OKVIS2-X | VI/event 基准、多 LiDAR 运动法、手眼工具 |
| **MIT SPARK / 历史 MIT** | Brookshire-Teller(RSS12, 运动法+CRB) | 运动法标定+不确定性奠基 |
| **NTU** (Xie Lihua 等) | NTU VIRAL 数据集, GPTR(Nguyen et al.) | LVI+UWB 空中标定、GP 连续时间基底 |
| **HKUST** (Shaojie Shen, Ming Liu→) | M-LOAM(TRO21), EvMultiCalib(Shen 参与) | 多 LiDAR 在线外参+协方差、事件时间/旋转标定 |
| **Oxford ORI** | allan_variance_ros (DRS Lab) | IMU 噪声标定工具链 |
| **SUSTech** (He Kong AISLAB) | Wang 2506.13420 主动标定+声学 | 可观性驱动主动标定、声学模态 |
| **Tongji** (Rui Fan) | MIAS-LCEC/C3M(T-IV24), EdO-LCEC | LVM 辅助/环境驱动在线 LiDAR-相机标定 |
| **SnT Luxembourg** (Olivares-Mendez) | DT-VI-Calib(ICRA26) | 离散时间高效 IMU-相机时空标定 |
| **PJLab-ADG** (Yikang Li) | OpenCalib, JointCalib, CalibAnything, LiDAR2INS | 生产级 AD 多传感器全栈标定+基础模型(SAM) |
| **Uni Freiburg** (Valada) | CMRNext(TRO25) | 学习型零样本 LiDAR-相机外参+地图定位 |
| **UTIAS** (Barfoot, Kelly) | certifiable-calibration(RA-L19), GPTR, Boreas | 全局最优手眼、GP 轨迹、多季节数据集 |
| **PolyMI/PoliTo/TU Delft** | One-target-tri-modal(BMVC25), RLCNet, CLRNet | 三模态(含 event/4D-radar)联合/端到端标定 |
| **U Zagreb** (Persic/Markovic) | Radar-IMU CT Calib(2603.19958) | 连续时间 radar-IMU 在线时空标定(承接 Persic GP) |

---

#### 覆盖性诚实声明
- 全部 6 个 facet 均报告 `dropped:0` 且均具备可访问检索工具（arXiv MCP、IEEE Xplore、PubMed/PMC、WebSearch 等），**无 facet 出现 `accessible_tools=[]`（无检索权限）**，故各类别无因工具缺失导致的盲检盲区。
- 后截止日期条目（2026 ID：2603.15767 CLRNet、2603.19958 Radar-IMU CT、2604.07151 RTK-SLAM、2602.16870 Boreas Road Trip）均经 WebSearch 独立二次验证，已标注 venue/年份。
- 已剔除 KNOWN 重复项：Sensors2025 spatiotemporal-calib (=s25175409，disambiguation only)；scout 中与 KNOWN 同名的 iKalibr/OA-LICalib 等未重复纳入。
- 仅"轻度支持相关性"的条目（如 Zhang event fusion odometry 标定非核心、Ruan embodied-AI 融合综述标定为边缘）已在 ④ 备注列明确标注，供读者判断纳入与否。

