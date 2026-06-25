# 抽取留痕：SLAM Handbook ch7 视觉SLAM (Visual SLAM)

> 本文件为《机器人学笔记》项目内部「抽取留痕」，目标是把源材料【全量保真】抽取，供后续综合 agent 写成自包含书章。禁摘要、禁凝练。
>
> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/07_Visual_SLAM/07_Visual_SLAM.md`（共 461 行，已完整读取）
>
> **作者**：Jakob Engel, Juan D. Tardós, Javier Civera, Margarita Chli, Stefan Leutenegger, Frank Dellaert, and Daniel Cremers
>
> **服务章节**：视觉里程计 (Visual Odometry)

---

## ⚠️ 抽取专员的重要说明（覆盖度与源性质判断）—— 综合 agent 务必先读

本任务【本章聚焦】要求重点覆盖：特征点法(ORB/匹配)；对极几何(本质矩阵/基础矩阵/单应)8 点法完整推导；三角化；PnP(DLT/EPnP/BA)；ICP(3D-3D)；直接法/光流(LK)完整推导；前端架构。

**经完整通读全文，SLAM Handbook ch7 是一篇"高层综述/导览型"章节（survey/overview chapter），不是教科书式的推导章节。** 它对许多聚焦主题只做**概念性描述或一句话提及**，并不给出代数推导。具体对照如下：

| 聚焦主题 | 本源(HB ch7)是否覆盖 | 覆盖深度 |
| --- | --- | --- |
| 特征点法 / ORB / 匹配 | ✅ 有 | §7.3.2 较详细：Harris / Shi-Tomasi / SIFT / FAST / SURF / BRIEF / ORB / BRISK 的历史脉络与原理描述；学习型 LIFT/SuperPoint/HF-Net/D2-Net；§7.9 SuperGlue/MASt3R 匹配。**但只有文字描述，无 Harris 矩阵公式、无 ORB 二进制描述子构造的代数细节。** |
| 对极几何（本质矩阵 E / 基础矩阵 F / 单应 H）| ❌ **几乎没有** | 全文**未出现**本质矩阵、基础矩阵、单应矩阵的定义或公式。仅 §7.3.7 一句话提到单目初始化需要好的初值。 |
| 8 点法完整推导 | ❌ **没有** | 全文无 8 点法。仅 §7.1.1 历史背景提到"minimal solutions for obtaining an initial camera configuration"和 RANSAC。 |
| 三角化 (Triangulation) | ❌ **没有** | 全文无三角化公式或算法。 |
| PnP（DLT / EPnP / BA）| ⚠️ **仅 BA 形式的 pose-only** | §7.4.1 给出 **pose-only BA**（式 7.4），即"已知 3D 点、求当前相机位姿"的重投影误差最小化——这正是 PnP 的 BA/重投影优化形式。**但无 DLT、无 EPnP 的代数推导。** |
| ICP (3D-3D) | ⚠️ **仅一句提及** | §7.7 提到早期 RGB-D SLAM 用 ICP 对齐深度点云 [78]，并被直接法（颜色+深度残差）取代。**无 ICP 的 SVD 闭式解或迭代推导。** |
| 直接法 / 光度误差 | ✅ 有（中等深度）| §7.3.5 + §7.4.2：给出光度误差/直接法的完整目标函数（式 7.6）、warping 概念、LSD-SLAM/DSO 架构、PGO/PGBA。 |
| 光流 (LK) 完整推导 | ❌ **没有** | 全文**未出现** Lucas-Kanade / 光流方程 / 亮度恒定约束的代数推导。§7.2.1 仅一句话把直接法关联到"optical flow"。 |
| 前端架构 | ✅ 有（较详细）| §7.2（pipeline 总览）+ §7.4.1（并行跟踪与建图 PTAM、关键帧、局部性/共视图、局部 BA）。 |

**给综合 agent 的转换提示**：本书"视觉里程计"章若要覆盖对极几何 8 点法、三角化、PnP 的 DLT/EPnP、ICP 的 SVD 解、LK 光流推导，这些**必须从另一源（高翔《视觉SLAM十四讲》ch7，即 sf14_07）抽取**——本 HB ch7 源里没有。本文件忠实抽取了 HB ch7 实际存在的全部内容（相机模型、关键点综述、重投影/光度误差、BA/Schur、直接法、RGB-D、VIO 等），并对缺失项做了显式标注，绝不杜撰源中不存在的推导。

---

## 0. 本源记号约定（与本书统一约定的差异）

> 本书统一约定：旋转 R∈SO(3)；右扰动为主；ξ=[ρ;φ]（平移在前、旋转在后）；Hamilton 四元数；协方差常用 Σ。

本源（HB ch7）的记号：

- **旋转**：用 **R**（矩阵），如 $R_w^i \in SO(3)$。与本书一致（用 R，非 C）。
- **位姿**：$T_w^i \in SE(3)$ 表示相机 i 的位姿；下标/上标记号为 $T_w^i$，含义为"world → camera i"的变换（把世界系点变到相机 i 系：$R_w^i x_j^w + t_w^i$）。**注意**：本源把 $T_w^i$ 写成下标 w、上标 i，读法是"从 w 到 i"。本书需留意此上下标方向（源用 $\boldsymbol{x}^c = R_w^i \boldsymbol{x}_j^w + \boldsymbol{t}_w^i$ 把世界点变到相机系）。
- **相机内参**：用 **$\boldsymbol{\xi}$**（注意！）表示**相机内参向量**（intrinsic parameters），如 $\boldsymbol{\xi}_p=[f_u\ f_v\ u_0\ v_0]^\top$。⚠️ **这与本书 ξ=[ρ;φ]（SE(3)/se(3) 的李代数切向量）完全不同含义！** 本源在 §7.7 才用 $\xi \in \mathfrak{se}(3)$ 表示李代数（$g=\exp(\xi)$）。综合时务必区分：HB 的 ξ 在 §7.3.1 是内参、在 §7.7 是李代数。
- **se(3) 李代数 / 指数映射**：§7.7 用 $g \in SE(3)$、$\xi \in \mathfrak{se}(3)$、$g=\exp(\xi)$。未显式说明左/右扰动，但其 coarse-to-fine Gauss-Newton "in $\xi \in \mathfrak{se}(3)$"的写法（DTAM/Kerl 风格）通常是**左扰动**（变换在世界/参考系一侧复合）。源未明示，综合时按上下文判断。
- **3D 点**：相机系点 $\boldsymbol{x}^c=[x\ y\ z]^\top \in \mathbb{R}^3$；世界系点 $\boldsymbol{x}_j^w \in \mathbb{R}^3$。
- **2D 观测/像点**：$\boldsymbol{z}=[u\ v]^\top \in \Omega \subset \mathbb{R}^2$（$\Omega$ 为图像域）；点 j 在图像 i 中的观测记 $\boldsymbol{z}_{ij}$。⚠️ **本源用 z 表示像素观测**（measurement），而本书/很多文献用 z 表示状态量或深度。源中 z 同时还作为 3D 点的第三分量（深度）出现，需按上下文区分。
- **投影函数**：$\pi(\cdot)$；反投影 $\pi^{-1}(\cdot)$。
- **协方差**：用 **$\boldsymbol{\Sigma}$**（如 $\boldsymbol{\Sigma}_j$、$\boldsymbol{\Sigma}_{ij}$），与本书一致。各向同性时 $\boldsymbol{\Sigma}_j = \sigma I_2$。
- **鲁棒核**：$\rho(\cdot)$（Huber / Tukey）。
- **图像强度/像素值**：$I$（如 $I_i(\cdot)$）；辐照度 (irradiance) $E$。
- **参考系**：$\mathcal{F}^w$ 世界系。
- **集合记号**：点集 $\mathsf{P}$（点 $P_j \in \mathsf{P}$，简记 $j\in\mathsf{P}$）；相机/图像集 $\mathsf{C}$（$C_i\in\mathsf{C}$，简记 $i\in\mathsf{C}$）；关键帧集 $\mathsf{K}\subset\mathsf{C}$；局部 BA 用 $\mathsf{K}_1,\mathsf{K}_2,\mathsf{P}_1$。
- **per-image 全局描述子**：$\boldsymbol{d}=f(I)\in\mathbb{R}^d$（VPR 用）。

**与本书差异小结**：旋转记号 R 一致、协方差 Σ 一致；最大陷阱是 **ξ 在本源 §7.3.1 指"相机内参"**（而非李代数）；位姿上下标 $T_w^i$ 方向需注意；像素观测用 z。

---

# 7. Visual SLAM（章首导言，源开篇）

从相机重建世界与传感器运动的挑战称为视觉同时定位与建图（Visual SLAM / VSLAM）。相机无处不在、廉价、低功耗，使视觉 SLAM 在自主机器人、自动驾驶、混合/增强现实中潜力无穷。本章结构（源行5）：
1. §7.1 历史背景与术语；
2. §7.2 典型视觉 SLAM 流水线总览；
3. §7.3 视觉 SLAM 公式化的关键要素；
4. §7.4 图像对齐与 BA 的进阶主题；
5. §7.5 视觉 SLAM 系统实例；
6. §7.6 实时三维稠密重建；
7. §7.7 深度相机 SLAM；
8. §7.8 视觉与其他模态（IMU、GPS、WiFi）结合的优势；
9. §7.9 新趋势简讨。

---

# 7.1 历史背景与术语 (Historical Background and Terminology)

## 7.1.1 从摄影测量到光束法平差与视觉 SLAM（源 §7.1.1）

视觉 SLAM 的历史建立在一般 SLAM 方法之上，但也根植于摄影测量 (photogrammetry) 与计算机视觉社区的进展。本节强调与后两个领域的历史联系（与 SLAM 的联系已在全书其他处讨论）。

历史脉络（源行13，全量保留人物/年份）：
- **1822**：Nicéphore Niépce 发明现代摄影；现存最古老照片为 **1827** 年的 "A View from a Window at Le Gras"。
- **约 1851**：法国军官 **Aimé Laussedat** 被视为摄影测量的发明者，开创用地面照片做地形测绘。
- **1867**：德国工程师 **Albrecht Meydenbauer** 进一步发展摄影测量用于建筑测量。
- **1890 起**：德国数学家、登山家 **Sebastian Finsterwalder**（1915 起任德国数学会主席）开创用航空影像对阿尔卑斯冰川做摄影测量重建，并倡导射影几何技术 [357]。
- 其博士生 **Otto von Gruber** 形式化了**光束法平差 (bundle adjustment, BA)** 的数学框架，用于从多图像中观测的对应点重建结构与运动。这些概念在 20 世纪初提出，**远早于计算机出现**。
- **1950 年代**：德国火箭科学家 **Hellmut H. Schmid** 开发了 BA 的矩阵计算技术，并与美国人 **Duane C. Brown** 合作，把这些方法部署到当时最大的计算机上。

重建方法与运动恢复结构 (structure from motion, SFM) 研究建立在多种相机模型上，从针孔相机模型与射影几何出发，刻画 2D 点观测与对应 3D 世界坐标的关系（源行15）：
- **1990 年代初**：Tomasi 和 Kanade [1102] 在正交投影 (orthographic projection) 简化假设下，开发了**矩阵分解技术**重建静态场景。
- 早期方法常聚焦于两视图重建；**2000 年代**社区转向**多视图 SFM** 问题。
- 传统重建流水线包含：特征提取 → 对应估计 → 用**最小解 (minimal solutions)** 获得初始相机配置 → 后续 **BA** 得到全局一致重建。因此大量精力投入特征提取与匹配算法，描述子如 **SIFT** [699]、**SURF** [62]，以及更近期众多基于学习的描述子。
- 为应对错误对应，研究者开发了采样策略如 **RANSAC** [328]，允许在对应估计与模型拟合之间交替进行（cf. Chapter 3）。

SFM 常聚焦于从无序图像集做**精确（一般离线）大规模 3D 世界重建**；而视觉 SLAM 通常聚焦于从运动相机的**在线、实时**重建（源行17）。在线实时方法的前提是**因果方法 (causal methods)** 的发展，如 [209]，它只用过去图像（而非整个图像集或视频）来解最优 SFM。首批可实时的 SFM/视觉 SLAM 方法约在 **2000** 年出现 [524, 244]。

## 7.1.2 术语 (Terminology)（源 §7.1.2）

以下术语常被互换使用以描述相似过程，但因应用与社区不同而侧重各异（源行21–31，逐条全量）：

- **摄影测量 (Photogrammetry)**：从 2D 照片中提取精确测量、空间信息和 3D 重建的科学。通过分析从不同视角拍摄的重叠图像，摄影测量能创建物体或环境的几何表示。广泛用于测绘、勘测和 3D 建模，是许多视觉里程计与 SLAM 算法的基础。

- **光束法平差 (Bundle Adjustment, BA)**：一种用于精化 3D 重建的数学优化方法。它调整相机的位置、朝向，以及可选的内参，连同被观测 3D 点的位置，以最小化**重投影误差 (reprojection error)**——即观测图像点与从 3D 模型投影点之间的差异。优化通常用二阶方法如 **Gauss-Newton** 或 **Levenberg-Marquardt**，需要仔细的初始化和鲁棒的外点剔除才能有效收敛。近期研究也探索**免初始化 BA (initialization-free BA)**，旨在简化优化过程。

- **运动恢复结构 (Structure from Motion, SFM)**：从不同视角拍摄的 2D 图像集合重建 3D 结构的过程。与常假设相机位置已知的摄影测量不同，SFM **同时**估计场景的 3D 结构和相机的运动。SFM 通常用于**非因果、非实时**场景，图像可来自多种来源（如互联网照片集）而非连续视频流。通常在最后阶段 SFM 会借助 BA 做优化。地标项目如 "Building Rome in a Day" [14] 展示了其可扩展性与大规模应用潜力。

- **视觉里程计 (Visual Odometry, VO)**：聚焦于通过分析视频中连续视觉帧来估计相机运动。它识别连续图像间的视觉对应来测量传感器随时间的相对运动。VO 主要处理**局部运动估计**，在最近观测的**滑动窗口**内操作，**不构建全局地图**。然而 VO 常与回环检测和建图组件结合以构成完整的视觉 SLAM 系统。

- **视觉 SLAM (Visual SLAM)**：使系统能在未知环境中**实时**同时自定位并构建该环境地图的计算技术。它结合摄影测量、视觉里程计和 SFM 的要素来处理图像数据、跟踪相机运动、构建详细 3D 地图。与 VO 不同，它通常会显式具备识别先前访问地点、相对其重定位、并可选地在此"回环"周围调整位姿估计的功能——此过程称为**回环 (loop closure)**。视觉 SLAM 是机器人、自动驾驶、增强现实的基石技术。

---

# 7.2 视觉 SLAM 系统的处理流水线（前端架构总览）（源 §7.2）

构建完整视觉 SLAM 系统需将各组件组合成能应对实时操作、可扩展性和鲁棒性需求的统一框架。关键考量包括：决定每个组件何时与如何运作、高效组织计算与数据流、确保对多样环境的适应性。

与 LiDAR SLAM 类似（cf. Chapter I 与 Chapter 8），视觉 SLAM 问题可分多阶段处理（即把计算拆为 **SLAM 前端 (front-end)** 与 **SLAM 后端 (back-end)**）。但与基于 LiDAR 的系统不同，**3D 几何不是被直接测量的**——人们观测的是场景辐照度 (scene irradiance) 在屏幕上的**投影**。这使视觉 SLAM 问题中的整体估计更具挑战性。

现代完整视觉 SLAM 系统通常包含三个互补的核心子功能（源行35）：
1. **里程计前端 (odometry front-end)**；
2. **建图后端 (mapping back-end)**；
3. **回环与重定位组件 (loop closure and re-localization component)**。

## 7.2.1 视觉里程计前端 (Visual Odometry Front-End)（源 §7.2.1）

视觉 SLAM 系统的核心元素是视觉里程计，旨在估计**连续相机帧之间的相对运动**。此阶段为相机位姿提供初始估计。

**计算视觉里程计的两种替代方法**（源行39，关键分类，全量）：

- **(i) 基于特征的方法 (Feature-based approaches)**：把挑战分为三阶段：
  1. **检测并提取特征点 (feature points)**；
  2. 计算图像间的**成对对应 (pairwise correspondence)**；
  3. 随后通过相对相机运动与 3D 点坐标**最小化重投影误差 (re-projection error)** 来确定相对相机运动。
  
  最后一阶段与经典 BA 相当类似。

- **(ii) 直接方法 (Direct approaches)**：在一步中处理问题，直接对相机运动和 3D 结构优化一个**光度损失函数 (photometric loss function)**。因此它们与**光流 (optical flow)** 方法以及有时被称为**光度 BA (photometric BA)** 的方法相当相关。

**两类方法的对比**（源行41）：至少在它们朴素的最初形式中，**基于特征的方法因显式数据关联而展现出更大的收敛域 (basin of convergence)**。为缓解这一点，**直接方法常采用由粗到精 (coarse-to-fine) 策略**，即从对齐降采样图像开始，甚至尝试对齐稠密的（学习的）特征而非亮度或颜色。

## 7.2.2 建图后端 (Mapping Back-End)（源 §7.2.2）

后端用全局优化技术如 **BA** 或**位姿图优化 (pose-graph optimization)** 来优化轨迹和地图。此步骤精化前端提供的估计，并将观测（包括来自地点识别的观测）整合进一致地图。结果是获得更长期的一致性，并减少长程畸变。

## 7.2.3 视觉地点识别与重定位 (Visual Place Recognition and Relocalization)（源 §7.2.3）

视觉里程计易**漂移 (drift)**，因为相机跟踪的误差会随时间累积。在缺乏绝对定位传感器（如 GPS）时，可通过将当前图像与先前观测的图像对齐来消除漂移并强制全局一致性。为此需在潜在的大图像集上计算对应。这可通过：
- 经典特征描述子（如 SIFT、SURF 或 BRIEF）的高效匹配；
- 或通过适当训练的神经网络（近年越来越流行的方法）。

由此产生的组件**检测相机何时重访先前建图区域（回环检测 loop closure detection）**，校正累积漂移，并在跟踪失败时重建定位。

## 7.2.4 计算与数据流 (Compute and Data Flow)（源 §7.2.4）

高效数据流对性能良好的 SLAM 系统至关重要（源行53–59，三条全量）：

- **流水线并行 (Pipeline Parallelism)**：不同组件如跟踪 (tracking)、建图 (mapping) 和优化 (optimization) 常**并行运行**以最大化效率。
- **数据共享 (Data Sharing)**：中间输出如关键点或位姿在组件间共享，以最小化冗余计算。
- **自适应调度 (Adaptive Scheduling)**：计算密集任务如全局优化，根据系统需求调度，优先保证实时响应性。

---

# 7.3 视觉 SLAM 基础 (Visual SLAM Fundamentals)（源 §7.3）

回顾视觉 SLAM 前端、后端、视觉地点识别实现中涉及的各要素。

## 7.3.1 相机模型 (Camera Model)（源 §7.3.1）

对传感器的参数化描述（建模从观测场景到图像的成像）应包含：
- **几何分量 (geometric component)**（也称**投影函数 projection function**）：描述 3D 点如何映射到 2D 像素；
- **光度分量 (photometric component)**：描述物理光强 (radiance) 如何映射到像素值。

**图 7.1（源行71）**：视觉 SLAM 系统的核心要求是选择合适镜头。图中所示镜头：BF2M2020S23 (195°)、BF5M13720 (183°)、BM4018S118 (126°)、BM2820 (122°)、GoPro 替换镜头 (150°)。鱼眼和广角镜头提供更宽视场 (FOV)，但需要合适的投影模型。流行选择有：
- **Brown-Conrady (BC) 模型** [283]；
- **Kannala-Brandt (KB) 模型** [537]；
- **Double Sphere (DS) 模型** [1117]。

6 参数的 DS 模型提供与 8 参数 KB 模型可比的重投影精度，同时投影函数计算速度快约 5 倍。(©2018 IEEE)

### 7.3.1.1 几何相机模型 (Geometric Camera Models)（源 §7.3.1.1）

**透视相机 (Perspective Cameras)。** 投影函数一般记为：

$$z = \pi(x^c, \xi),$$

其中 $\boldsymbol{x}^c = \begin{bmatrix} x & y & z \end{bmatrix}^{\top} \in \mathbb{R}^3$ 是相机坐标系中的 3D 点，$\boldsymbol{z} = \begin{bmatrix} u & v \end{bmatrix}^{\top} \in \Omega \subset \mathbb{R}^2$ 是对应投影 2D 点在图像域 $\Omega$ 中的坐标，$\boldsymbol{\xi} \in \mathbb{R}^n$ 表示相机内参（在视觉 SLAM 中通常预标定）。$\boldsymbol{\xi}$ 的维数取决于所用**相机模型**。

> ⚠️ 记号提醒：此处 $\boldsymbol{\xi}$ 是**相机内参向量**，与本书 ξ=[ρ;φ]（李代数）不同。

反投影 (unprojection) 操作记为：

$$\boldsymbol{x}^c = \pi^{-1}(\boldsymbol{z}, \boldsymbol{\xi}),$$

它从 2D 图像点 z 重建 3D 中的一条射线。由于点的深度未知，得到的 $\boldsymbol{x}^c$ 只在**尺度意义下 (up to scale)** 已知。

实践中存在适用于不同镜头几何和相机类型的多种投影函数（见 [687]）。

**直线/针孔/透视相机模型 (Rectilinear / pinhole / perspective)** 是最简单的，可用于无畸变的窄角镜头：

$$\boldsymbol{\xi}_p = \begin{bmatrix} f_u & f_v & u_0 & v_0 \end{bmatrix}^\top, \quad \pi_p(\boldsymbol{x}^c, \boldsymbol{\xi}) = \begin{bmatrix} f_u \frac{x}{z} + u_0 \\ f_v \frac{y}{z} + v_0 \end{bmatrix}, \quad \pi_p^{-1}(\boldsymbol{z}, \boldsymbol{\xi}) \sim \begin{bmatrix} \frac{u - u_0}{f_u} \\ \frac{v - v_0}{f_v} \\ 1 \end{bmatrix},$$

其中 $f_u$、$f_v$ 是水平和垂直方向的焦距（像素单位），$u_0$、$v_0$ 是主点 (principal point) 的 2D 坐标，并假设矩形像素。对于典型的方形像素，预期 $f_u \approx f_v$。

> 注：源式中反投影第三分量在原文排版里写为 $\sim \begin{bmatrix}(u-u_0)/f_u\\(v-v_0)/f_v\end{bmatrix}$（"∼"表示在尺度意义下，对应射线方向，齐次第三分量为 1）。

**径向-切向模型 (radial-tangential model)**，用于考虑一定镜头畸变（最常用）：

$$\boldsymbol{\xi}_{RT} = \begin{bmatrix} \boldsymbol{\xi}_p^\top & k_1 & k_2 & p_1 & p_2 \end{bmatrix}^\top, \qquad \begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} \frac{x}{z} \\ \frac{y}{z} \end{bmatrix}, \quad r = \sqrt{x'^2 + y'^2},$$

$$\begin{bmatrix} x'' \\ y'' \end{bmatrix} = \begin{bmatrix} x'(1+k_1r^2+k_2r^4)+2p_1x'y'+p_2(r^2+2{x'}^2) \\ y'(1+k_1r^2+k_2r^4)+p_1(r^2+2{y'}^2)+2p_2x'y' \end{bmatrix}, \qquad \pi_p(\boldsymbol{x}^c,\boldsymbol{\xi}) = \begin{bmatrix} f_ux''+u_0 \\ f_vy''+v_0 \end{bmatrix}.$$

> 注：源式 (源行95) 中 $\begin{bmatrix}x'\\y'\end{bmatrix}=\begin{bmatrix}\frac{x}{z}\\\frac{y}{z}\end{bmatrix}$ 为归一化坐标（原文 OCR 排版略含糊，写作 $\frac{\overline{x}}{z},\frac{y}{z}$，即 x/z 与 y/z）。$k_1,k_2$ 为径向畸变系数，$p_1,p_2$ 为切向畸变系数。

**关键结论（无解析反投影）**：无法为反投影模型 $\pi_{RT}^{-1}(z, \xi)$ 提取解析表达式，因为不存在把 $x',y'$ 表为 $x'',y''$ 函数的解析解（即去畸变 undistortion 无闭式）。但可借助迭代方法，例如 **Newton-Raphson 法**。

**广角与鱼眼相机 (Wide-angle and Fisheye Cameras)。**（源行101–113）

- 对于广角镜头（见图 7.1），FOV 直到 **180°**，带几个径向畸变系数的针孔模型通常足够。
- **Brown-Conrady 模型** [283] 相当于上述径向-切向模型去掉切向系数，即 $\boldsymbol{\xi}_{BC} = [\boldsymbol{\xi}_p^{\top}\ \ k_1\ \ k_2]$。**注意**：尽管相对径向-切向模型有简化，仍**无解析去畸变**。

- 对于 FOV 大于 **180°** 的鱼眼镜头，使用 **Kannala-Brandt (KB) 模型** [537]（例如用于 [142]）：

$$\boldsymbol{\xi}_{KB} = \begin{bmatrix} \boldsymbol{\xi}_p^\top & \boldsymbol{k}_{KB}^\top \end{bmatrix}^\top, \qquad \pi_{KB}(\boldsymbol{x}^c, \boldsymbol{\xi}) = \begin{bmatrix} f_u\, r(\theta) \cos \psi + u_0 \\ f_v\, r(\theta) \sin \psi + v_0 \end{bmatrix}^\top.$$

其中入射射线由角度参数化：
$$\theta = \arctan \frac{\sqrt{x^2+y^2}}{z}, \qquad \psi = \arctan \frac{y}{x},$$
畸变由四个系数 $\mathbf{k}_{KB} = \begin{bmatrix} k_1 & \dots & k_4 \end{bmatrix}^{\top}$ 参数化，畸变表达式为
$$r(\theta) = \theta + \sum_{n=1}^{4} k_n\, \theta^{2n+1}.$$

- **Double Sphere (DS) 模型** [1117]：鱼眼和广角镜头的流行替代，在精度与速度间取得良好折中。在 DS 模型中，点被**连续投影到两个单位球**上，球心相差 $\gamma$。然后用偏移 $\frac{\alpha}{1-\alpha}$ 的针孔模型把点投影到图像平面。得到：

$$\pi_{DS}(\boldsymbol{x}^{c},\boldsymbol{\xi}) = \begin{bmatrix} f_{u} \dfrac{x}{\alpha d_{2} + (1-\alpha)(\gamma d_{1}+z)} + c_{u} \\[2mm] f_{v} \dfrac{y}{\alpha d_{2} + (1-\alpha)(\gamma d_{1}+z)} + c_{v} \end{bmatrix}^{\top}, \qquad \text{with } \boldsymbol{\xi} = \begin{bmatrix} \boldsymbol{\xi}_{p}^{\top} & c_{u} & c_{v} & \gamma & \alpha \end{bmatrix}^{\top}.$$

> 注：源式 (源行111) 分子原文 OCR 显示为 $u, v$，按 DS 模型语义应为相机系点的 $x, y$ 分量（这里照源保留语义，提示综合 agent 校核：DS 模型中 $d_1=\sqrt{x^2+y^2+z^2}$、$d_2=\sqrt{x^2+y^2+(\gamma d_1+z)^2}$，源未列出 $d_1,d_2$ 的展开式）。

如 [1117] 所示，此模型提供**闭式反投影解**。因此 6 参数 DS 模型计算比 8 参数 KB 模型快约 5 倍，同时提供可比的重投影误差。

### 7.3.1.2 光度模型 (Photometric Models)（源 §7.3.1.2）

光度标定把辐照度映射到像素值：

$$I = f(E,T),$$

其中 $I$ 是像素强度，$E$ 是辐照度 (irradiance)，$T$ 包含影响此映射的其他相机属性，如曝光时间、模拟与数字增益、伽马校正、去拜耳 (de-bayering) 和镜头渐晕 (vignetting)。此光度标定通常**只在尺度意义下重要**，且对那些旨在获得稠密、有纹理场景表示或依赖跨图像光度一致性 (photo-consistency) 的方法才重要。

### 7.3.1.3 时间相关效应 (Time-Dependent Effects)（源 §7.3.1.3）

在相机拍摄图像时运动的情形（对 SLAM 或 VO 系统几乎总是如此），也需考虑此运动对成像过程的影响。
- 大多数现代消费相机使用**卷帘快门 (rolling shutter)**，按时间顺序逐行捕获图像行。
- 相比之下，**全局快门相机 (global shutter)**（常用于机器感知应用）同时捕获所有图像行。

实践建议：要么使用全局快门相机，要么通过为各图像行变化相机位姿来把卷帘快门效应纳入相机模型。注意，在**视觉-惯性系统**中这变得明显更实用，因为 IMU 有效地在相机曝光窗口内测量局部运动，从而显著简化卷帘快门相机的使用与建模。

### 7.3.1.4 实用考量 (Practical Considerations)（源 §7.3.1.4）

选择相机模型时，首要目标是确保参数模型能有效且精确地逼近传感器与镜头系统的行为。
- 用鱼眼镜头时，推荐用**球面模型 (spherical model)**；
- 用直线镜头时，应用**线性基模型 (linear base-models)**。

更一般地，选择相机模型涉及在计算效率与精度之间权衡：使用不合适的相机模型会引入不准确和系统性偏差，显著降低视觉 SLAM 系统的精度和鲁棒性。

## 7.3.2 关键点 (Keypoints)——特征点法核心（源 §7.3.2）

SLAM 系统在建图与精确定位上的成功，取决于其检测、描述、匹配场景中关键特征的能力——通常称为"**关键点 (keypoints)**"或"**特征 (features)**"。

- **关键点检测 (keypoint detection)**：识别图像中显著、独特且可重复的区域，确保从不同视角、跨多次运行、在不同条件下可靠地重检测。理想关键点检测器应在光照、视角或遮挡变化下保持这些属性。
- **关键点描述子 (keypoint descriptor)**：一旦检测到，描述子把对应关键点检测的局部外观编码为紧凑且独特的表示。理想情况下，描述子应唯一刻画关键点周围环境，同时对光照变化、旋转、尺度等变换保持不变。

**图 7.2（源行139）**：塑造视觉 SLAM 与图像匹配中视觉关键点文献的若干最著名算法的时间线。

这确保了高召回率 (high recall，跨不同条件匹配同一关键点) 和高精度 (high precision，避免与相似但不相关特征的错误对应)。

实践中，剧烈光照变化、无纹理表面、动态场景等真实挑战会在检测和匹配中引入误差，直接影响 SLAM 性能。最理想属性包括（源行143）：
- **尺度与旋转不变性 (scale and rotation invariance)**：确保视角变化下检测一致；
- **可重复性与独特性 (repeatability and distinctiveness)**：允许可靠重检测与唯一识别；
- **效率 (efficiency)**：在计算约束下实现实时操作。

为满足这些需求，关键点检测与描述从经典手工 (handcrafted) 技术演化到近期基于深度学习的方法。

### 7.3.2.1 经典关键点检测器与描述子 (Classical Keypoint Detectors & Descriptors)（源 §7.3.2.1）

> 注：本节为**文字描述**各方法原理，源中**无公式**（如无 Harris 二阶矩矩阵公式）。以下逐一全量记录。

- **Harris-Stephens 关键点检测器** [432]（广称 "Harris 角点检测器 Harris corner detector"）：最有影响力的方法之一。它通过**分析图像块内二阶矩矩阵 (second-moment matrix) 的特征值 (eigenvalues)** 来识别角点，当**两个特征值都大**时（表示两个正交方向上的强强度变化）把图像块分类为关键点。

- **Shi-Tomasi 角点检测器** [1004]：建立在相同原理上，但**直接使用较小特征值 (smaller eigenvalue)** 做角点选择。相比之下，Harris 定义一个"角点性 (cornerness)"响应函数来近似该过程以提高效率。Harris 检测器鲁棒且计算高效，但**缺乏尺度不变性**——这一局限后被 SIFT、SURF 等方法设法解决。

- **尺度不变特征变换 (Scale Invariant Feature Transform, SIFT)** [699]：关键点检测的里程碑，在挑战性场景下为精度和召回率设立新标准。SIFT 高度**尺度与旋转不变**，且对光照变化**部分不变**。它遵循结构化三阶段过程（源行149，全量）：
  1. 在尺度空间**高斯差分 (Difference of Gaussians, DoG) 金字塔**中检测关键点作为极值，并通过 **3D 二次拟合 (3D quadratic fit)** 精化其定位；
  2. 基于主导局部图像梯度**分配朝向 (orientation)**；
  3. 从离散化梯度朝向的直方图计算 **128 维描述子**。
  
  然而 SIFT 卓越的鲁棒性代价是高计算成本，使其对实时应用更昂贵。

- **加速分割测试特征 (Features from Accelerated Segment Test, FAST)** [946]：为提高效率而开发的高速角点检测器。它评估候选关键点位置周围**圆形邻域**中的像素强度，并采用**早期拒绝策略 (early rejection strategy)** 最小化计算。FAST 进一步用**决策树 (decision tree)** 和**非极大值抑制 (non-maximum suppression)** 增强速度。但与 SIFT 不同，它**缺乏尺度不变性**，对显著变换敏感。

- **加速鲁棒特征 (Speeded-Up Robust Features, SURF)** [63]：通过用**积分图像 (integral images)** 和**盒滤波器 (box filters)** 代替高斯导数来近似 SIFT，从而提高效率，在速度与鲁棒性间取得更好平衡。

- **二进制鲁棒独立基本特征 (Binary Robust Independent Elementary Features, BRIEF)** [141]：对快速且鲁棒描述子的需求催生了二进制方法。BRIEF 引入了使用**二进制强度比较 (binary intensity comparisons)** 的紧凑描述子，能用 **Hamming 距离**快速匹配描述子。虽然 BRIEF 缺乏尺度和旋转不变性，但它证明了对 SIFT 鲁棒性至关重要的局部梯度可通过简化的二进制测试有效捕获。

- **ORB** [953]：受 BRIEF 成功启发提出，**建立在 FAST 关键点检测和 BRIEF 描述之上**，并通过**图像金字塔 (image pyramid)** 和**强度质心法 (intensity centroid method)** 添加**尺度和旋转不变性**。
  > 注：ORB = Oriented FAST and Rotated BRIEF（源 §7.5 行365 给出全称）。

- **BRISK** [643]：同期提出，在**尺度空间金字塔**上采用 FAST 或 Harris 角点，并通过识别类似 SIFT 的主导关键点方向，在二进制描述子中纳入对旋转变化的不变性。

**经典方法选型权衡**（源行153）：虽然 ORB 和 BRISK 等方法增加的旋转和尺度不变性对输出关键点的独特性影响很小，但它们在视觉 SLAM 和实时机器人应用中已证明有效。然而当计算成本不是要求时（例如计算机视觉应用），SIFT 和 SURF 可能仍更可取，因为它们在挑战性光照或视角变化下仍提供更大鲁棒性。

经典手工关键点方法在 CV 和机器人中是基础性的，仔细平衡鲁棒性、效率和不变性。但对适应日益复杂、动态环境的关键点的需求仍是持续挑战，这推动研究转向基于学习的方法。

### 7.3.2.2 基于深度学习的关键点检测与描述 (Deep-Learning-based Keypoint Detection & Description)（源 §7.3.2.2）

到 2010 年代后期，基于深度学习的图像关键点方法开始获得关注，利用大规模数据集和**卷积神经网络 (CNN)** 直接从数据学习鲁棒特征。与手工设计的关键点不同，这些方法自动发现并优化特征表示，在以前不可行的场景中展现无与伦比的适应性和精度（例如在极端光照变化下检测稳定关键点——经典手工方法在此挣扎的领域）。视觉 SLAM 社区因此日益从传统特征工程转向数据驱动的表示学习。

逐一记录（源行161–165，全量）：

- **学习不变特征变换 (Learned Invariant Feature Transform, LIFT)** [1245]：最早把关键点检测、朝向估计和描述子计算整合进**端到端可训练流水线**的深度学习方法之一。用 CNN 从小图像块提取特征，LIFT 比经典方法对尺度、光照和旋转变化有更大鲁棒性。它采用**顺序学习策略 (sequential learning strategy)**：先训练描述子，然后朝向估计，再然后关键点检测，确保稳定有效的特征提取。

- **SuperPoint** [267]：引入**自监督框架 (self-supervised framework)**，在单次前向传播中检测关键点并计算描述子，对实时应用高效。与基于图像块的网络不同，SuperPoint 在**整幅图像**上操作，并利用**单应自适应 (homographic adaptation)**——一种自监督学习技术，生成伪真值关键点。此方法相比 SIFT、ORB、FAST 等经典方法显著改善关键点可重复性和描述子质量，尤其在光照变化下。

- **HF-Net** [970]：引入**分层定位 (hierarchical localization)** 方法，结合全局图像检索与精确局部特征匹配。HF-Net 通过把关键点检测、局部描述子和全局描述子整合进单个 CNN，在保持高鲁棒性的同时提高计算效率。此架构通过限制用于匹配的图像数量来减少运行时间，对大规模 SLAM 和实时应用尤其有效（甚至在夜间场景等极端外观变化下）。HF-Net 的学习特征比 SuperPoint 更稀疏但更具判别性，使其成为 DX-SLAM [649] 等深度学习 SLAM 系统的首选。

- **D2-Net** [295]：引入**先描述后检测 (describe-and-detect)** 方法，颠倒了传统的"先检测关键点、后提取描述子"顺序。它不先检测关键点，而是用 CNN 计算稠密特征图，然后把关键点识别为这些图中的局部极大值。此方法捕获高层语义信息，对极端光照变化和弱纹理环境鲁棒。与分离检测和描述的 SuperPoint 不同，D2-Net 联合优化两个任务，增强描述子一致性。然而此稠密方法虽提高鲁棒性，但比经典稀疏方法计算更密集。尽管有此权衡，D2-Net 对视觉定位和 SFM 任务仍高度有效。

## 7.3.3 重投影误差 (Reprojection Error)（源 §7.3.3）

视觉重投影误差测量观测图像点 $z_i \in \mathbb{R}^2$ 与重投影点 $\pi(x_i^c, \xi) \in \mathbb{R}^2$ 之间的差异：

$$e_{\mathrm{reproj}} = z_j - \pi(x_j^c, \xi),$$

其中 $z_j$ 是观测的 2D 图像点，$x_j^c \in \mathbb{R}^3$ 是相机坐标系中的 3D 点，$\xi$ 是相机内参标定参数，$\pi$ 是投影函数。

**假设观测图像点 $z_j$ 被高斯噪声扰动**，似然函数可表为：

$$p(\boldsymbol{z}_{j}\,|\,\boldsymbol{x}_{j}^{c},\boldsymbol{\xi}) \sim \mathcal{N}\!\left(\pi(\boldsymbol{x}_{j}^{c},\,\boldsymbol{\xi})\,,\,\boldsymbol{\Sigma}_{j}\right),$$

其中 $\Sigma_j$ 是特征在图像中位置的高斯噪声协方差。最简单情况下，噪声假设为各向同性且沿图像恒定：$\Sigma_j = \sigma I_2$。

**最大化一组观测的似然等价于最小化其负对数似然**（推导，全量）：

$$\mathcal{L} = -\sum_{j} \log p(\boldsymbol{z}_{j} | \boldsymbol{x}_{j}^{c}, \boldsymbol{\xi}) = \frac{1}{2} \sum_{j} \left\| \boldsymbol{z}_{j} - \pi(\boldsymbol{x}_{j}^{c}, \boldsymbol{\xi}) \right\|_{\boldsymbol{\Sigma}_{j}}^{2} + \text{const.}$$

去掉常数，得到一幅图像中观测点集的**加权平方重投影误差**：

$$\boxed{E_{\text{reproj}} = \frac{1}{2} \sum_{j} \left\| \boldsymbol{z}_{j} - \pi(\boldsymbol{x}_{j}^{c}, \boldsymbol{\xi}) \right\|_{\boldsymbol{\Sigma}_{j}}^{2}.} \tag{7.1}$$

**处理外点**：应用鲁棒核 (robust kernels)，如 Huber 或 Tukey 核（cf. Chapter 3）。例如鲁棒重投影误差可表为：

$$E_{\text{robust}} = \frac{1}{2} \sum_{j} \rho\!\left( \left\| \boldsymbol{z}_{j} - \pi(\boldsymbol{x}_{j}^{c}, \boldsymbol{\xi}) \right\|_{\boldsymbol{\Sigma}_{j}} \right), \tag{7.2}$$

其中 $\rho(\cdot)$ 是减小大残差影响的鲁棒核函数。在下文中，为简便起见，丢弃对相机内参 $\boldsymbol{\xi}$ 的依赖。

> $\|\cdot\|_{\Sigma}$ 记号说明：马氏范数 (Mahalanobis norm)，$\|e\|_\Sigma^2 = e^\top \Sigma^{-1} e$（源未显式写出此定义，但这是标准含义）。

**图 7.3（源行200）**：基于特征的视觉 SLAM 问题：给定每幅图像中匹配的特征集（左），估计其对应 3D 点的位置和获取每幅图像的位姿（右）。图用 ORB-SLAM [787] 生成。

## 7.3.4 基于关键点的视觉 SLAM (Keypoint-Based Visual SLAM)（源 §7.3.4）

基于特征的视觉 SLAM 的核心是**最小化重投影误差**。设有一组环境点 $P_j \in \mathsf{P}$，在用运动相机拍摄的一组图像 $C_i \in \mathsf{C}$ 中被观测。为简化记号，仅用点和相机标识符，写 $j \in \mathsf{P}$ 和 $i \in \mathsf{C}$。

视觉 SLAM 的目标是估计：
- 世界参考系 $\mathcal{F}^w$ 中的点坐标 $\boldsymbol{x}_j^w \in \mathbb{R}^3$；
- 相机位姿 $T_w^i \in SE(3)$。

在**单目情形**中，每个点在每幅图像中的观测就是观测图像坐标 $\boldsymbol{z}_{ij} \in \Omega_i \subset \mathbb{R}^2$（图 7.3）。

把重投影误差最小化应用于所有点和相机位姿，称为 **完整 BA (full BA)**：

$$\boxed{\{ \boldsymbol{T_w^i}^*, \, \boldsymbol{x_j^w}^* \mid i \in \mathsf{C}, \, j \in \mathsf{P} \} = \underset{\boldsymbol{T_w^i}, \, \boldsymbol{x_j^w}}{\arg \min} \, \frac{1}{2} \sum_{i,j} \rho\!\left( \left\| \boldsymbol{z}_{ij} - \pi\!\left( \boldsymbol{R}_w^i \boldsymbol{x}_j^w + \boldsymbol{t}_w^i \right) \right\|_{\boldsymbol{\Sigma}_{ij}} \right).} \tag{7.3}$$

> 这里 $\boldsymbol{R}_w^i, \boldsymbol{t}_w^i$ 是 $T_w^i$ 的旋转和平移分量，把世界点 $\boldsymbol{x}_j^w$ 变换到相机 i 系后投影。

**优化方法（四种，全量记录，每种权衡）**（源行212–222）：

- **批量优化 (Batch Optimization)**：在所有观测上、对所有位姿和路标，迭代最小化整体重投影误差，即求解式 (7.3)。流行最小化算法是 **Gauss-Newton (GN)** 和 **Levenberg-Marquardt (LM)**。此方法是 SFM 的黄金标准，但对实时 SLAM 中每幅图像运行太昂贵。

- **基于滤波的方法 (Filtering-Based Approaches)**：用序贯因果方法如**扩展卡尔曼滤波 (EKF)** 或**信息滤波 (Information Filter)** 做实时状态估计。它们通过**只保留最后一个相机位姿**来简化问题，但不幸的是这**破坏了稀疏性 (destroys sparsity)**（图 7.4），把它们限制在几百个特征。此外，滤波方法**不重新线性化过去的观测**，损失精度。

- **基于关键帧的方法 (Keyframe-Based Approaches)**：通过只保留少数图像（称为**关键帧 keyframes** [583]）来简化问题。中间图像及其观测在地图估计中被丢弃。对于相同的计算量，它们能构建比滤波方法更长更精确的地图 [1039]。

- **因子图 (Factor Graphs)**：上述所有方法都可用因子图形式化。为此，把 SLAM 问题表示为图，其中节点对应变量（如位姿、路标），因子表示重投影误差以及对标定和/或相机位姿的先验（在本书 Part I 详细讨论）。

**图 7.4（源行217）**：含四个相机位姿和五个特征的视觉 SLAM 示例：贝叶斯网络 (Bayes network)（上）及其对应的马尔可夫随机场 (Markov random field)（中）。**EKF SLAM 边缘化掉过去的相机位姿，导致稠密图 (dense graph)**（左下），限制其能力。**关键帧 SLAM 只保留少数相机位姿并丢弃中间图像的观测，保持 SLAM 稀疏性 (sparsity)**（右下）[1039]。

> 这是 EKF SLAM vs. 关键帧 BA 稀疏性对比的关键图（综合时重点呈现）。

## 7.3.5 光度误差与直接方法 (Photometric Error and Direct Methods)（源 §7.3.5）

光度误差通过提供**直接最小化观测与投影图像区域间像素强度差异**的方式，为重投影误差提供替代。此方法植根于**光度一致性 (photometric consistency)** 原理，它假设连续帧中的对应像素在一致光照条件下表示同一场景点。

> 注：§7.3.5 在源中很短（仅此一段），具体的光度损失公式在 §7.4.2（式 7.6）给出。

## 7.3.6 视觉地点识别与全局定位 (Visual Place Recognition and Global Localization)（源 §7.3.6）

视觉地点识别 (Visual Place Recognition, VPR) 是：给定查询图像，在已注册图像数据库中找到来自同一地点的一幅。这通常通过计算**逐图像描述子 (per-image descriptor)** $\boldsymbol{d}=f(I)\in\mathbb{R}^d$ 来解决，该描述子总结图像内容，并通过 **k-NN 搜索**在此描述子空间中检索最近的一个。

- **Galvez-Lopez 和 Tardos** [362] 引入**词袋 (bag-of-words)** 方法，基本上通过把 ORB 或其他描述子量化为视觉聚类或"**词 (words)**"来聚合它们。虽然它们在小时空范围内表现优异，但受手工特征对视觉纹理变化的低不变性限制。
- 对于这些情况，已提出深度架构并训练用于特征提取和聚合，对视觉外观变化有高不变性 [35, 508]。

## 7.3.7 初始化 (Initialization)（源 §7.3.7）

式 (7.3) 重投影误差的最小化通常用**非线性迭代优化**处理，这需要足够精确的初始猜测才能收敛。视频序列前几帧中视觉 SLAM 状态的初始化因此对正确跟踪很重要，**尤其对单目设置 (monocular setups)，其完整状态从单帧不可观测 (not observable from a single frame)**。

> 注：这是源中唯一与"单目初始化/两视图几何"沾边的内容，但**未给出对极几何/8 点法/三角化的任何公式**。综合 agent 若需这些，须从 sf14_07 取。

## 7.3.8 地图表示 (Map Representations)（源 §7.3.8）

地图表示是视觉 SLAM 系统的关键方面，它定义环境如何被建模和存储以用于导航、建图和定位。良好设计的地图表示在精度、内存效率和计算成本间取得平衡。（更广泛的稠密地图表示讨论见 Chapter 5。）

---

# 7.4 关于图像对齐与 BA 的进一步考量（前端架构 + 直接法 + BA 求解）（源 §7.4）

这里提供关于**图像对齐 (image alignment)**（即如何从图像估计当前相机位姿——这是里程计和回环检测的关键）的更多细节，并简要提及 BA 的求解技术。

**图 7.5（源行246）**：ORB-SLAM [787] 中局部性 (locality) 的表示。
- **共视图 (covisibility graph)**（左）：连接看到至少 θ 个共同点的关键帧（此例中 θ = 15），用于**局部 BA**。
- **本质图 (essential graph)**（右）：一个更稀疏的版本，此例中连接看到至少 θ = 100 个共同点的关键帧，用于回环校正期间的**位姿图优化 (pose-graph optimization)**。©2015 IEEE。

## 7.4.1 基于关键点的图像对齐 (Keypoint-based Image Alignment)——前端架构 + pose-only BA（即 PnP 的 BA 形式）（源 §7.4.1）

虽然完整 BA（式 7.3）是 SFM 的黄金标准，但它太昂贵无法在帧率（通常 10–50 Hz）运行。为实时操作，大多数基于关键点的视觉 SLAM 流水线使用两个关键思想：

### 思想 1：并行跟踪与建图 (Parallel Tracking and Mapping, PTAM)

把 SLAM 过程拆成两个并行运行的线程 [583]：

- **跟踪线程 (tracking thread)**：为当前图像 $i \in \mathsf{C}$ 找特征匹配并计算其相机位姿，**不更新估计的地图点**，使用 **pose-only BA（仅位姿 BA）**：

$$\boxed{\mathbf{T}_{w}^{i*} = \underset{\mathbf{T}_{w}^{i}}{\operatorname{arg\,min}} \sum_{j \in \mathsf{P}} \rho\!\left( \left\| \mathbf{z}_{ij} - \pi\!\left( \mathbf{R}_{w}^{i} \mathbf{x}_{j}^{w} + \mathbf{t}_{w}^{i} \right) \right\|_{\Sigma_{ij}} \right).} \tag{7.4}$$

> ⭐ **这是 PnP 的 BA/重投影优化形式**：3D 点 $\mathbf{x}_j^w$ 已知（地图已固定），只优化当前相机位姿 $\mathbf{T}_w^i$。这是本源对"PnP"主题的唯一覆盖（无 DLT/EPnP 闭式推导）。

- **建图线程 (mapping thread)**：只为图像的一个子集 $\mathsf{K} \subset \mathsf{C}$（称为**关键帧 keyframes**）求解 BA，只有这些关键帧的位姿会被纳入地图。这样 BA 只需在关键帧率（通常 0.5–5 Hz）运行。关键帧可以恒定频率插入，但更明智的选项是把那些**包含显著新信息**的帧升级为关键帧。

### 思想 2：局部性 (Locality)

当相机在大环境中操作时，其观测对远处地图部分的影响可忽略，回环事件除外。通常做法是：把**回环校正 (loop correction) 交给一个运行很不频繁的第三线程**，并在建图线程中对一个关键帧窗口运行**局部 BA (local BA)**。

局部窗口的定义：
- 可用**时间准则 (temporal criterion)** 定义为最后 k 帧或关键帧——这是视觉里程计或视觉-惯性 SLAM 系统的常用选择。
- 在视觉 SLAM 系统中，更好的选项是基于**共视性准则 (covisibility criterion)**，例如把与当前关键帧有多于 θ 个共同观测点的关键帧纳入局部窗口 [1038, 787]（见图 7.5 示例）。

这样，**局部 BA** 可以只更新一组共视关键帧和它们观测的点（图 7.6）：

$$\boxed{\left\{\boldsymbol{T}_{w}^{k^{*}},\boldsymbol{x}_{j}^{w^{*}} \mid k \in \mathsf{K}_{1}, j \in \mathsf{P}_{1}\right\} = \underset{\boldsymbol{T}_{w}^{k},\boldsymbol{x}_{j}^{w}}{\arg\min} \frac{1}{2} \sum_{\substack{i \in \mathsf{K}_{1} \cup \mathsf{K}_{2}, \\ j \in \mathsf{P}_{1}}} \rho\!\left(\left\|\boldsymbol{z}_{ij} - \pi\!\left(\boldsymbol{R}_{w}^{i} \boldsymbol{x}_{j}^{w} + \boldsymbol{t}_{w}^{i}\right)\right\|_{\boldsymbol{\Sigma}_{ij}}\right).} \tag{7.5}$$

**图 7.6（源行264）**：ORB-SLAM [787] 中局部 BA 的实现。局部地图由以下定义：
- 集合 $\mathsf{K}_1$：包含当前关键帧 k 及其在共视图中的邻居；
- 集合 $\mathsf{P}_1$：被它们看到的点（红色）；
- 集合 $\mathsf{K}_2$：地图中其余看到 $\mathsf{P}_1$ 中某点的关键帧。

> 注意式 (7.5) 中：优化变量只有 $\mathsf{K}_1$ 的位姿和 $\mathsf{P}_1$ 的点；但求和遍历 $\mathsf{K}_1 \cup \mathsf{K}_2$——$\mathsf{K}_2$ 的位姿**固定**（提供约束但不被优化），这是局部 BA 的标准做法。

**完整四线程流水线**（源行272）：完整视觉 SLAM 流水线示例（图 7.7）有四个线程：
1. **跟踪 (tracking)**：以帧率运行；
2. **局部建图 (local mapping)**：以关键帧率运行；
3. **回环闭合 (loop closing)**：对每个关键帧尝试检测回环，检测到时校正；
4. **完整 BA (full BA)**：可选运行，在回环闭合后改善地图。

**图 7.7（源行280）**：ORB-SLAM2 系统 [785] 的结构，展示地图、地点识别数据库及其完整处理流水线，由四个线程组成：跟踪、局部建图、回环闭合、完整 BA。©2017 IEEE。

## 7.4.2 直接图像对齐 (Direct Image Alignment)——直接法完整目标函数（源 §7.4.2）

直接方法如 **LSD-SLAM** [307] 或 **DSO** [308] 通常追求类似的流水线：先估计帧到帧的跟踪和建图，然后确保某种形式的全局一致性。但与"先提取、匹配、跟踪点，再最小化几何重投影误差"不同，它们**直接使用传感器的亮度信息 (brightness information)**，旨在给定原始传感数据计算 3D 结构和相机运动的**最大后验估计 (maximum a posteriori estimate)**。

这相当于通过最小化如下形式的**光度损失 (photometric loss)** 来**联合求解对应估计和 SLAM 问题**：

$$\boxed{E_{photo} = \sum_{i \in \mathcal{F}} \sum_{z \in \mathcal{P}_i} \sum_{j \in obs(z)} \rho\!\Big( I_i(z) - I_j\big(\omega(z, d_z, \mathbf{T}_j^i)\big) \Big),} \tag{7.6}$$

优化变量为所有相机参数 $T_j^i \in SO(3)$ 和所有深度值 $d_z \in \mathbb{R}$。

> ⚠️ 记号校核：源行286 写 $T_j^i \in SO(3)$，但此处 $T_j^i$ 是含旋转+平移的刚体运动（warping 用到完整 $SE(3)$），源把它写成 $SO(3)$ 应为笔误/OCR 误，语义应为 $SE(3)$。综合时按 $SE(3)$ 处理。

**此损失的含义（warping 概念，全量）**：此损失确保所有帧对 i 和 j 中对应点的颜色 $I_i$ 和 $I_j$ 一致。更具体地：
- 对所有关键帧的集合 $\mathcal{F}$ 求和；
- 对关键帧 $\mathcal{P}_i$ 中每个点 z，对该点可见的所有帧 $obs(z)$ 确保颜色一致性；
- **warping $\omega$**（图像间扭曲）：取点 z 及其深度值 $d_z$，用刚体运动 $\mathbf{T}_j^i$ 把它从帧 i 变换到帧 j，并透视投影回图像 $I_j$。

> 即 $\omega(z, d_z, \mathbf{T}_j^i) = \pi\big(\mathbf{T}_j^i \cdot \pi^{-1}(z, d_z)\big)$（源未显式展开 ω，但语义如此：反投影 z 到 3D → 用 $\mathbf{T}_j^i$ 变换 → 投影回 $I_j$）。

**图 7.8（源行294）**：LSD-SLAM [307] 的示意总览，展示三个组件：直接相机跟踪、直接建图、PGO（保证全局一致性），全部交替运行。相比之下，DSO [308] 在单个 Gauss-Newton 优化中执行结构和运动的优化以达到更高精度。直接方法如 DSO 被证明比基于关键点的方法提供更高精度，因为它们**不执行任何中间抽象**，能从极其微妙的亮度变化中确定相机运动 [308]（©2017 Springer）。

**LSD-SLAM 工作方式**：如图 7.8，LSD-SLAM 交替优化深度图和相机运动（跟踪），并执行**位姿图优化 (PGO)** 来确保估计相机运动与计算的成对图像对齐的全局一致性。

**DSO 工作方式**（源行290）：相比之下，DSO [308] 以**光度 BA (photometric BA)** 的形式**联合优化结构和运动**。鲁棒损失 $\rho$ 通过在一个小图像块上的加权平方差之和实现，其中包含自动曝光时间适应（在曝光时间未知的情况下）。各残差的依赖以因子图形式捕获。CPU 上的实时性能通过把优化限制在关键帧子集的**滑动窗口 (sliding window)** 同时边缘化掉旧帧的影响来实现。这带来精度的显著提升，因为它依赖于给定所有传感亮度数据的结构和运动的统计最优估计。

**实时 SLAM 中实现全局一致性的几个步骤**（源行298，全量）：
1. **首先**，可以像 DSO 那样在最后 k 个关键帧上联合优化，以确保滑动时间窗口内的一致性（**滑动窗口光度 BA, sliding window photometric BA**）。
2. **其次**，可以额外运行 **PGO** [553, 365]（见图 7.9），以重新计算与所有估计的成对图像对齐最大一致的相机轨迹。
3. **第三**，可以执行 PGO 的一个自适应版本，称为**位姿图光束法平差 (Pose Graph Bundle Adjustment, PGBA)** [1135]，它以相同的计算效率额外纳入 BA 的完整光度不确定性（因为只更新相机位姿）。

**图 7.9（源行306）**：在直接视觉 SLAM 方法中，可以执行位姿图优化以计算与所有先前估计的成对图像对齐全局一致的轨迹 [553]。(©2013 IEEE)

## 7.4.3 求解 BA (Solving BA)——Schur 补完整推导（源 §7.4.3）

虽然 BA 可用 Chapter 2 讨论的一般变量消元技术求解，但有能利用问题稀疏结构的专门求解器。

**图 7.10（源行351）**：含 4 个相机和 9 个从中观测的点的玩具示例。
- 观测雅可比 (observation Jacobian) **非常稀疏**，因为每个观测 $z_{ij}$ 只依赖于相机 i 和点 j。
- 结果是，每个观测在 Hessian 中引入：一个相机对角块、一个点对角块、一个相机-点非对角块。
- 由于点数通常比相机数大几个数量级，好的解法是**先消去点，再求解相机**。
- 第一行展示因子图和点观测的雅可比；第二行展示完整 Hessian 和约化相机系统 (reduced camera system) 的 Hessian。

### Schur 补 (Schur complement) 推导（全量代数，不跳步）

如果有一个线性系统，其中 D 可逆，可通过左乘一个矩阵来变换它：

$$\begin{pmatrix} A & B \\ C & D \end{pmatrix} \begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} b_1 \\ b_2 \end{pmatrix},$$

左乘 $\begin{pmatrix} I & -BD^{-1} \\ 0 & I \end{pmatrix}$：

$$\begin{pmatrix} I & -BD^{-1} \\ 0 & I \end{pmatrix}\begin{pmatrix} A & B \\ C & D \end{pmatrix} \begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} I & -BD^{-1} \\ 0 & I \end{pmatrix}\begin{pmatrix} b_1 \\ b_2 \end{pmatrix},$$

化简得到：

$$\begin{pmatrix} A-BD^{-1}C & 0 \\ C & D \end{pmatrix} \begin{pmatrix} x_1 \\ x_2 \end{pmatrix} = \begin{pmatrix} b_1-BD^{-1}b_2 \\ b_2 \end{pmatrix},$$

> 中间步骤说明（源照此给出三个矩阵的连乘等式）：左乘矩阵后，第一块行变为 $(A-BD^{-1}C,\ B-BD^{-1}D) = (A-BD^{-1}C,\ 0)$，右端第一块变为 $b_1 - BD^{-1}b_2$；第二块行不变。

得到一个可以**先解 $x_1$ 再解 $x_2$** 的系统：

$$\boxed{\big(\boldsymbol{A} - \boldsymbol{B}\boldsymbol{D}^{-1}\boldsymbol{C}\big)\boldsymbol{x}_1 = \boldsymbol{b}_1 - \boldsymbol{B}\boldsymbol{D}^{-1}\boldsymbol{b}_2, \qquad \boldsymbol{D}\boldsymbol{x}_2 = \boldsymbol{b}_2 - \boldsymbol{C}\boldsymbol{x}_1.}$$

### 应用到 BA

BA 问题需在每次迭代求解如下形式的方程：

$$\begin{pmatrix} \mathbf{H}_{cc} & \mathbf{H}_{cp} \\ \mathbf{H}_{cp}^{\top} & \mathbf{H}_{pp} \end{pmatrix} \begin{pmatrix} \boldsymbol{d}_c \\ \boldsymbol{d}_p \end{pmatrix} = \begin{pmatrix} \boldsymbol{b}_c \\ \boldsymbol{b}_p \end{pmatrix}.$$

> 下标 c = cameras（相机），p = points（点）。$\boldsymbol{d}_c, \boldsymbol{d}_p$ 是相机和点的增量。

这可以**分三步求解**：计算点的 Schur 补以获得约化相机系统、为相机求解、最后为点求解：

$$\mathbf{H}_{cc}^{red} = \mathbf{H}_{cc} - \mathbf{H}_{cp} \mathbf{H}_{pp}^{-1} \mathbf{H}_{cp}^{\top}, \tag{7.7a}$$

$$\mathbf{H}_{cc}^{red}\, \mathbf{d}_{c} = \mathbf{b}_{c} - \mathbf{H}_{cp} \mathbf{H}_{pp}^{-1} \mathbf{b}_{p}, \tag{7.7b}$$

$$\mathbf{H}_{pp}\, \mathbf{d}_{p} = \mathbf{b}_{p} - \mathbf{H}_{cp}^{\top} \mathbf{d}_{c}. \tag{7.7c}$$

（源式编号统一为 (7.7)）

**效率要点**（源行332）：
- 由于 $\mathbf{H}_{pp}$ 是**块对角 (block diagonal)** 的，Schur 补和最终点求解可以**逐点 (point by point) 非常高效地完成**。
- 如图 7.10 所示，**约化相机系统更不稀疏 (less sparse)**，因为它包含关联"看到某共同点的相机对"的块。在示例中，相机 1 和 2 之间有块，但相机 1 和 3 之间没有。
- 在**局部 BA** 中，约化相机系统几乎是满的，可用**稠密矩阵求解器 (dense matrix solvers)**。
- 相比之下，**完整 BA** 有大得多的关键帧数，但它们之间的共视性更少，因此可从约化相机系统的**稀疏求解器 (sparse solver)** 中获益。

## 7.4.4 BA 再探 (Bundle Adjustment Revisited)——免初始化 BA / Power BA（源 §7.4.4）

BA 已被研究一个多世纪，§7.4.3 详述的经典方法已在众多开创性论文中确立并证明运作良好 [1107, 14, 978]。然而此流水线有两个重要缺点：
1. 相应的解需要路标和相机位姿的合适初始化；
2. 对大规模问题，计算和内存需求会增长到过大。

近年有一系列论文解决这些缺点并挑战传统计算流水线 [469, 470, 258, 259, 1170, 1171, 1172]。

**关键计算瓶颈**是约化相机系统 (7.7) 的求解。

### Power Bundle Adjustment（幂级数法）

**Power Bundle Adjustment** [1171] 不用迭代共轭梯度算法求解，而是用**矩阵幂级数 (matrix power series)** 来**近似 Schur 矩阵的逆**。

Schur 矩阵（重述）：

$$\mathbf{H}_{cc}^{red} = \mathbf{H}_{cc} - \mathbf{H}_{cp} \mathbf{H}_{pp}^{-1} \mathbf{H}_{cp}^{\top}, \tag{7.8}$$

幂级数近似：

$$\boxed{(\mathbf{H}_{cc}^{red})^{-1} \approx \sum_{i=0}^{m} \big(\mathbf{H}_{cc}^{-1} \mathbf{H}_{cp} \mathbf{H}_{pp}^{-1} \mathbf{H}_{cp}^{\top}\big)^{i}\, \mathbf{H}_{cc}^{-1}.} \tag{7.9}$$

此幂级数对增大的截断参数 m 可证明收敛到真逆 [1171]。主要优势是繁琐的矩阵求逆被简单的矩阵乘法替代，后者可以更快、更省内存地完成。

### Variable Projection（变量投影，免初始化）

对初始化的依赖在 [469, 470] 中通过回归**变量投影 (variable projection)** 概念缓解（见图 7.11）。为此，BA 问题分为两阶段：
1. **第一阶段**：把复杂的透视投影替换为一个通用的射影矩阵 (generic projective matrix)，使得到的优化可以**解析地把路标求解为相机参数的函数**。这消除了路标和相机位姿之间的"鸡生蛋"依赖，在优化相机位姿时带来更大的吸引域 (basin of attraction)。
2. **第二阶段（射影精化 projective refinement）**：用计算出的解作为原始（透视）重建的初始化。

虽然此策略对超过 100 个相机计算上太苛刻，但它与幂级数方法的结合（如 [1172] 提出）为大规模 BA 问题提供了**无需初始化的可扩展解**。

**图 7.11（源行359）**：在一系列论文 [469, 470, 1171, 1172] 中，研究者倡导用变量投影方法和矩阵幂级数来以运行时和内存高效的方式求解大规模 BA 问题而无需初始化。如上所示，相机位姿和路标可以从**随机初始化**开始计算。(©2024 Springer)

---

# 7.5 完整视觉 SLAM 系统实例 (Examples of Full Visual SLAM Systems)（源 §7.5）

逐一全量记录各系统（源行363–369）：

- **LSD-SLAM (Large-Scale Direct SLAM)** [307]：一个**直接 SLAM 系统**，聚焦于稠密跟踪和半稠密建图。它依赖**光度误差最小化**而非基于关键点的方法，使其在低纹理环境中特别有效。系统实时操作，提供场景的半稠密重建，非常适合室内和小规模室外环境的单目相机。

- **ORB-SLAM** [787, 785]：因鲁棒性和灵活性而成为最广泛采用的 SLAM 系统之一。它整合：
  - 使用 **ORB (Oriented FAST and Rotated BRIEF)** 描述子的基于关键点的跟踪；
  - 有效的回环检测；
  - 稀疏地图表示。
  
  ORB-SLAM 支持单目、双目和 RGB-D 相机，使其在不同设置中高度通用。它在需要高精度和鲁棒重定位能力的场景中表现优异。在 **ORB-SLAM3** [142] 中扩展到鱼眼相机、多地图 (multimap) 和视觉-惯性 SLAM。

- **OKVIS** [645]：最初构想为视觉-惯性里程计系统。最新版本 **OKVIS2** [644] 是视觉-惯性 SLAM 系统，也可在纯视觉（多相机）模式运行。与各 ORB-SLAM 版本类似，它使用关键点和描述子（**BRISK**）。

- **Direct Sparse Odometry (DSO)** [308]：一个用于估计 3D 点云和相机轨迹的直接方法。与 LSD-SLAM 相比，相机运动和路标点在**单个 Gauss-Newton 优化中联合估计**。为实现实时性能，只更新最后 k 个关键帧，得到**滑动窗口光度 BA**。考虑的关键帧数 k 提供速度和精度间的权衡。此外，DSO 使用带相机响应函数和渐晕的**完整光度标定**。传统的**亮度恒定假设 (brightness-constancy assumption)** 因此被**辐照度恒定假设 (irradiance-constancy assumption)** 替代，即假设 3D 世界中相应点随时间发出相同辐照度。已提出此方法到双目系统 [1160] 和全向相机 [739] 的扩展。也添加了回环检测以减少较长序列中的漂移 [365]。

**直接法 vs. 关键点法的实践权衡**（源行369，全量，重要）：
- 在可实时方法中，直接方法如 DSO 被证明比基于关键点的方法提供更多精度和鲁棒性 [308]。
- 但为达最优性能，直接法通常需要良好的**光度标定**和**全局快门相机**。卷帘快门效应导致几何畸变。虽然这可在直接 SLAM 方法中建模 [980, 981]，但得到的方法常不再可实时。因此它们在基于关键点的方法中更好处理（后者按设计最小化几何畸变）。
- 尽管如此，直接方法在**低分辨率视频**上常表现更好，因为由于模糊和降采样，可能更难识别可靠特征点 [1227]。
- 此外，特征点对高效**重定位和回环闭合**有价值。
- 因此，实践的视觉 SLAM 系统常回归**混合方法 (hybrid approaches)**，结合两者优点。混合方法的一例是 [385]，其中基于特征的重定位信息被紧密整合进直接视觉 SLAM 方法，以进一步提升其鲁棒性和精度。

---

# 7.6 实时稠密重建 (Real-time Dense Reconstruction)（源 §7.6）

虽然传统上 BA 和 SLAM 处理重建相机运动和稀疏路标点集的问题，但对从增强现实到自主机器人的众多应用，人们更希望有观测世界的**稠密重建 (dense reconstruction)**。为此，多年来已倡导若干从单目相机实时稠密重建的算法 [1046, 800, 1180, 875]。传统上它们回归**变分方法 (variational methods)**，通过最小化损失函数来估计连续 3D 结构 [1046]：

$$\boxed{\min_{h:\Omega\to\mathbb{R}} \frac{1}{2} \sum_{i\in\mathcal{I}(\boldsymbol{x})} \int_{\Omega} \rho_i(\boldsymbol{z}, h) \,\mathrm{d}\boldsymbol{z} + \lambda \int |\nabla h| \,\mathrm{d}^2\boldsymbol{z},} \tag{7.10}$$

优化变量为一个稠密地图 $h$，它为图像平面 $\Omega \subset \mathbb{R}^2$ 中每个像素 z 分配一个深度值。

**残差 (residual)**：

$$\boxed{\rho_i(\boldsymbol{z}, h) = \left| I_i\!\left( \pi\!\left( \boldsymbol{R}_w^i\, \boldsymbol{x}^w(\boldsymbol{z}, h) + \boldsymbol{t}_w^i \right) \right) - I_0(\boldsymbol{z}) \right|,} \tag{7.11}$$

它强制**参考图像 $I_0$ 中像素 z 处的亮度**与**一组相邻图像 $I_i$ 中对应像素**的一致性。对应像素通过以下获得：取 3D 点 $\boldsymbol{x}^w(\boldsymbol{z}, h)$，用对应旋转矩阵 $\boldsymbol{R}_w^i \in SO(3)$ 和平移向量 $\boldsymbol{t}_w^i \in \mathbb{R}^3$ 把它从世界系变换到相机 i，最后用模型 $\pi(\cdot)$ 投影到图像 $I_i$。

**全变分正则项 (total variation regularizer)**（由 $\lambda$ 加权）强制计算深度图的空间平滑性，并为未观测区域诱导类似肥皂膜 (soap-film-like) 的填充，如图 7.12 所示。

**图 7.12（源行385）**：3D 世界的稠密重建（上）可用变分方法从手持单目相机（下）实时计算，该方法在 GPU 上高效并行化 [1046]。相关方法在 [800, 1180, 875] 中提出。(©2010 Springer)

---

# 7.7 深度相机 SLAM (SLAM with Depth-sensing Cameras)——含 ICP 提及 + RGB-D 直接法（源 §7.7）

随着 Microsoft Kinect 的引入，深度感知相机成为商品。这些所谓的 **RGB-D 相机**通常基于**结构光 (structured-light)** 或**飞行时间 (time-of-flight)**，提供深度图像流，常与彩色图像流结合。因此它们在概念上介于标准相机和 LiDAR 传感器之间。但与 LiDAR 传感器不同，它们提供**即时的 2D 深度值阵列**。配备合适算法，RGB-D 相机对 3D 感知非常强大，尽管限于室内应用（因为基于红外的感知与阳光冲突）和一定范围（通常约 5 米内）。

**图 7.13（源行397）**：从移动 RGB-D 相机计算的多办公室大规模走廊场景的稠密重建 [1030]。用八叉树 (octrees) [1030] 或体素哈希 (voxel hashing) [808] 和直接相机跟踪 [554]，这些重建被证明可在 GPU 上 [1030] 甚至平板 CPU 上 [1031] 实时运行。(©IEEE)

### KinectFusion 与 TSDF 融合

**KinectFusion** [801] 建立在先前距离图像融合工作 [234] 之上，倡导一种从移动 RGB-D 相机重建相机运动和 3D 结构的方法。

把各深度图像融合成连贯 3D 重建的基本思想：把每个深度图像编码为一个**（投影）符号距离函数 (signed distance function)** $d_i(\boldsymbol{x})$，它对每个体素 $\boldsymbol{x} \in V$ 编码到最近表面点的（符号）距离（沿视线方向）。随后可为所有体素计算一个聚合距离函数 $D(\boldsymbol{x})$，作为各个距离函数的**加权平均**：

$$\boxed{D(\boldsymbol{x}) = \frac{\sum_{i} \omega_{i}(\boldsymbol{x})\, d_{i}(\boldsymbol{x})}{\sum_{i} \omega_{i}(\boldsymbol{x})}.} \tag{7.12}$$

**最大似然解释**（源行404）：在深度方向高斯噪声假设下，此加权平均不过是距离函数的**最大似然估计 (maximum-likelihood estimate)**。

为更鲁棒，通常对**截断符号距离函数 (truncated signed distance functions, TSDF)** 求平均，使每个感测表面点仅对重建有局部影响。权重 $\omega_i(\boldsymbol{x})$ 编码相应表面测量的确定性（依赖传感器，通常随距物体距离衰减）。为填充重建中的孔洞，可回归后处理技术 [801]，或修改上述加权方案以确保重建的水密性 (watertight-ness) [1044]。

### RGB-D 直接跟踪（颜色+深度残差）vs. ICP

**ICP 提及**（源行406）：虽然早期 RGB-D SLAM 方法通过用 **ICP 算法** [78] 对齐相应深度点云来跟踪相机，但后续工作倡导**直接最小化测量两个连续 RGB-D 帧 $(I_1, d_1)$ 和 $(I_2, d_2)$ 的颜色和深度一致性的残差** [554]：

$$\boxed{r_I(\xi) = I_2(\tau_g(\boldsymbol{z})) - I_1(\boldsymbol{z}), \qquad r_d = d_2(\tau_g(\boldsymbol{z})) - \big[g\,\pi^{-1}(\boldsymbol{z}, d_1(\boldsymbol{z}))\big]_z,} \tag{7.13}$$

其中：
- $g \in SE(3)$ 表示期望的刚体运动；
- $\tau_g(\boldsymbol{z}) = \pi\, g\, \pi^{-1}(\boldsymbol{x}, d_1(\boldsymbol{z}))$ 表示对应像素间诱导的 warping（扭曲）；
- $[\cdot]_z$ 返回点的 z 分量（深度分量）。

> 记号注：这里 $\xi \in \mathfrak{se}(3)$ 是**李代数**（与 §7.3.1 的内参 ξ 不同！），$g=\exp(\xi)$。$r_I$ 是光度（颜色）残差，$r_d$ 是深度残差。源式 $\tau_g$ 定义中第一个参数写 $\boldsymbol{x}$，应与 $\boldsymbol{z}$ 一致（OCR/排版），语义为"反投影像素 z 到 3D、用 g 变换、再投影"。

然后可以用合适的分布拟合所有残差 $r = (r_I, r_d)$ 的分布，并通过在 $\xi \in \mathfrak{se}(3)$ 中的**由粗到精 Gauss-Newton 优化**确定相机变换 $g = \exp(\xi)$ 的最大后验估计 [1029, 553]。

**与 ICP 的对比结论**（重要）：与经典 ICP 方法相比，此直接跟踪方法在既定基准上把均方根跟踪误差降低近一个数量级，同时显著更快——详见 [554]。

### 大规模与回环

为更大规模建图，**均匀体素表示太占内存**。因此通常回归自适应表示，用**体素哈希 (voxel hashing)** [808] 或**八叉树 (octrees)** [1030]（见图 7.13，亦参 Chapter 5）。

后续方法进一步展示了对更大场景的可扩展性：
- 采用移动融合窗口 (moving fusion window)，如 **Kintinuous** [1183]；
- 或纳入回环，如 **ElasticFusion** [1184]，它通过**变形图 (deformation graph)** 使整个稠密地图表示变形。

---

# 7.8 视觉与其他模态结合 (Combining Vision with Other Modalities)（源 §7.8）

出于众多原因，建议把视觉与其他模态结合。这能提供额外的鲁棒性和精度，也能提供**绝对尺度信息 (absolute scale)**。具体而言：
- **惯性传感器 (inertial sensors)** 提供度量尺度 (metric scale) 和高度可靠鲁棒的局部相对运动测量；
- **GPS/WiFi** 可用于全局（重）定位和地理定位。

这些互补输入解决了纯视觉系统固有的局限，使它们在实际应用中不可或缺。

## 7.8.1 惯性测量单元 (IMU)（源 §7.8.1）

IMU 提供角速度和线加速度的高频测量，实现局部运动的精确估计。

**松耦合 (loosely coupled) vs. 紧耦合 (tightly coupled)**：
- **松耦合**：来自每个传感器的传感信息独立聚合成位姿估计，随后用**卡尔曼滤波**融合各估计。虽然这在实践中常工作得相当好（例如四旋翼 [306] 或纳米直升机 [291] 的自主导航），但它**不如传感信息的紧耦合积分精确**。

- **首批紧耦合方法**利用 EKF 方案，其中 IMU 运动学 (kinematics) 在**预测步 (prediction step)** 积分，视觉关键点测量作为**更新 (updates)**——如开创性工作 **MSCKF** [777]。或者，可采用 Chapter 11 详细讨论的因子图方法形式化**滑动窗口和批量优化器**。

**图 7.14（源行429）**：多传感器的整合可用因子图以紧耦合方式优雅执行。这是视觉与 IMU 紧耦合融合的因子图示例 [1116]。它显著提高相机运动和 3D 结构的精度和鲁棒性——见图 7.15。(©2016 IEEE)

图 7.14 展示了为**双目-惯性里程计 (stereo-inertial odometry)** [1116] 提出的因子图，它结合双目 LSD-SLAM 与 IMU 信息。

**图 7.15（源行433）**：IMU 测量与直接图像对齐的紧融合相比仅依赖图像对齐的纯视觉里程计系统，产生更精确的位置跟踪（左 vs. 右）。融合用图 7.14 的因子图实现。重建的点云来自纯里程计，未强制回环。[1116]（©2016 IEEE）

这种因子图紧耦合也被最先进的基于关键点的方法采用，如 ORB-SLAM3 [142]、OKVIS2 [644] 等。

**实践要点（IMU 的价值，全量）**（源行437–443）：
- 实践中，一些最精确的视觉-惯性里程计系统以**惯性积分为基础**，主要用视觉来校正因（双重）积分噪声和偏差随时间快速增长的漂移。
- IMU 允许观测**运动的度量尺度**，以及传感器相对**重力**的朝向——从而把系统的**规范自由度 (gauge freedom) 减少到只有 4 个未知量**（全局 x, y, z 位置，以及偏航 yaw），相比纯视觉系统的 **7 个未知量**（全局 x, y, z、横滚 roll、俯仰 pitch、偏航 yaw，以及尺度 scale）。
- 在许多方面，IMU 与视觉作为模态是**互补的**，因此非常适合在视觉-惯性 SLAM 或里程计系统中结合。

**流行的视觉-惯性里程计/SLAM 方法**（源行445，全量列举）：常作为现有视觉 SLAM 系统的扩展，包括：
- ORB-SLAM 的视觉-惯性版本 [786]；
- Direct Sparse Visual-Inertial Odometry [1136]；
- VINS-Mono [896]；
- BASALT [1118]；
- DM-VIO [1135]——一种单目-惯性公式，利用**延迟边缘化 (delayed marginalization)** 概念以更好捕获各传感器中运动的可观测性。

## 7.8.2 GPS 与 WiFi 用于全局定位 (GPS and WiFi for Global Localization)（源 §7.8.2）

虽然 IMU 改善局部运动估计，但 GPS 和 WiFi 对全局定位至关重要，尤其在大规模环境中。
- 在**室外**环境，GPS 提供绝对位置数据，使系统能把 SLAM 地图锚定到全局坐标。这对自动驾驶车辆等室外应用至关重要 [1181, 193]（见图 7.16）。
- 在**室内**场景，WiFi 信号在 GPS 信号不可用时实现粗定位，补充基于视觉地图的定位。

**图 7.16（源行441）**：通过结合包括双目相机、IMU 和 RTK-GPS 信息（左）的多传感器，以因子图紧耦合方式融合，可获得高度精确鲁棒的轨迹和点云，无论室内还是室外，如停车场多层楼中行驶汽车的重建所示（右）[1181]。(©2020 Springer)

---

# 7.9 延伸阅读与近期趋势 (Further Readings & Recent Trends)（源 §7.9）

视觉 SLAM 是极其活跃和动态的研究领域。本章只覆盖了相关工作的一小部分。

## 关键点检测与匹配 (Keypoint Detection and Matching)（源行455–459）

随着基于学习的关键点在特定设置中相对传统手工对应物的空前不变性和适应性，基于深度学习的关键点提取技术一直在变革该领域。这一转变不仅增强了关键点检测和描述，也推动了**关键点匹配 (keypoint matching)** 技术的重大进步。基于学习的方法现在纳入**全局空间感知 (global spatial awareness)** 来更鲁棒地建立对应，而非依赖手工设计的描述子匹配启发式。

逐一记录（全量）：

- **SuperGlue** [971] 和 **MASt3R** [642]：利用**图神经网络 (GNN)** 和 **Transformer** 通过考虑更广泛的图像结构来精化匹配。这些方法解决了传统匹配器的根本局限，适应极端视角变化、光照变化和遮挡——常规局部描述子比较通常在此挣扎。

- **SuperGlue** 通过整合**自注意力 (self-attention)** 和**交叉注意力 (cross-attention)** 机制增强特征匹配，使其能解决重复纹理和遮挡引起的歧义。与仅在局部描述子上操作的传统关键点匹配器不同，SuperGlue 纳入上下文信息，显著提高室内外环境的精度。其**最优匹配层 (optimal matching layer)** 进一步确保关键点对应稳健建立，同时在必要时允许未匹配的关键点，使其对真实场景高度有效。

- **MASt3R** 把此思想扩展到 **3D 感知特征匹配**，从两幅图像重建 3D 场景表示，以改善无纹理区域和极端视角变化下的性能。

- **MASt3R-SLAM** [789]：在此基础上构建，把这些进步整合进 SLAM 流水线，改善相机位姿估计、全局地图一致性和回环闭合策略。

通过从手工 2D 特征匹配转向基于学习的、上下文感知的、3D 知情的方法，这些方法标志着视觉 SLAM 的范式转变，增强了复杂环境中的场景理解、跟踪鲁棒性和长期稳定性。

**总体评价**（源行459）：从传统手工方法到基于深度学习方法的关键点提取演化是显著的。虽然经典方法因其效率和可解释性至今仍非常相关，但它们在无纹理表面、极端视角变化和光照变化场景中挣扎。深度学习解决了这些局限。然而，在平衡计算效率与实时性能（尤其在资源受限平台上）以及确保多样、无偏训练数据集的可用性方面仍存在挑战。

## 其他前沿 (Other Frontiers)（源行461）

强调近期视觉 SLAM 有许多激动人心的发展，包括：
- 视觉 SLAM 到**动态环境**（含移动和潜在可变形物体）的泛化；
- 基于学习的视觉 SLAM 方法日益流行：在众多出版物中，越来越多经典视觉 SLAM 流水线的组件（特征提取、对应估计、图像对齐、相机跟踪、BA、稠密重建等）正被基于学习的公式增强或替代。

所有这些想法将在本手册 Part III 详细讨论。

---

# 附录 A：本源全部带编号公式清单（便于综合 agent 索引）

| 编号 | 内容 | 源位置 |
| --- | --- | --- |
| (无号) | 投影函数 $z=\pi(x^c,\xi)$ | §7.3.1.1 |
| (无号) | 反投影 $x^c=\pi^{-1}(z,\xi)$ | §7.3.1.1 |
| (无号) | 针孔模型 $\xi_p$、$\pi_p$、$\pi_p^{-1}$ | §7.3.1.1 |
| (无号) | 径向-切向模型 $\xi_{RT}$、$x',y',r$、$x'',y''$、$\pi_p$ | §7.3.1.1 |
| (无号) | KB 鱼眼模型 $\pi_{KB}$、$\theta,\psi,r(\theta)$ | §7.3.1.1 |
| (无号) | DS 双球模型 $\pi_{DS}$ | §7.3.1.1 |
| (无号) | 光度模型 $I=f(E,T)$ | §7.3.1.2 |
| (无号) | 重投影误差 $e_{reproj}=z_j-\pi(x_j^c,\xi)$ | §7.3.3 |
| (无号) | 似然 $p(z_j|\cdot)\sim\mathcal{N}(\pi,\Sigma_j)$ | §7.3.3 |
| (无号) | 负对数似然 $\mathcal{L}$ | §7.3.3 |
| **(7.1)** | 加权平方重投影误差 $E_{reproj}$ | §7.3.3 |
| **(7.2)** | 鲁棒重投影误差 $E_{robust}$（鲁棒核 ρ） | §7.3.3 |
| **(7.3)** | 完整 BA (full BA) | §7.3.4 |
| **(7.4)** | pose-only BA（=PnP 的 BA 形式） | §7.4.1 |
| **(7.5)** | 局部 BA (local BA) | §7.4.1 |
| **(7.6)** | 直接法光度损失 $E_{photo}$ | §7.4.2 |
| (无号) | Schur 补一般推导（3 个矩阵等式 + 解 $x_1,x_2$） | §7.4.3 |
| (无号) | BA 线性系统 $\begin{psmallmatrix}H_{cc}&H_{cp}\\H_{cp}^\top&H_{pp}\end{psmallmatrix}$ | §7.4.3 |
| **(7.7)** | BA 三步求解（$H_{cc}^{red}$、解相机、解点） | §7.4.3 |
| **(7.8)** | Schur 矩阵 $H_{cc}^{red}$（重述） | §7.4.4 |
| **(7.9)** | Power BA 幂级数近似逆 | §7.4.4 |
| **(7.10)** | 变分稠密重建损失 | §7.6 |
| **(7.11)** | 变分残差 $\rho_i(z,h)$ | §7.6 |
| **(7.12)** | TSDF 融合 $D(x)$（KinectFusion） | §7.7 |
| **(7.13)** | RGB-D 直接跟踪残差 $r_I, r_d$ | §7.7 |

# 附录 B：本源全部图表清单

| 图号 | 主题 | 源位置 |
| --- | --- | --- |
| 7.1 | 镜头选择 / BC、KB、DS 模型对比 | §7.3.1 |
| 7.2 | 视觉关键点算法时间线 | §7.3.2 |
| 7.3 | 基于特征的视觉 SLAM 问题示意 (ORB-SLAM) | §7.3.3 |
| 7.4 | EKF SLAM (稠密图) vs. 关键帧 SLAM (稀疏) 的贝叶斯网络/MRF | §7.3.4 |
| 7.5 | ORB-SLAM 共视图 vs. 本质图 | §7.4 |
| 7.6 | ORB-SLAM 局部 BA 的 $K_1,K_2,P_1$ | §7.4.1 |
| 7.7 | ORB-SLAM2 四线程结构 | §7.4.1 |
| 7.8 | LSD-SLAM vs. DSO 架构 | §7.4.2 |
| 7.9 | 直接法 PGO | §7.4.2 |
| 7.10 | 4 相机 9 点玩具例：因子图/雅可比/Hessian/约化相机系统 | §7.4.3 |
| 7.11 | 变量投影 + 幂级数免初始化 BA | §7.4.4 |
| 7.12 | 变分稠密重建 (DTAM 风格) | §7.6 |
| 7.13 | RGB-D 大规模走廊稠密重建 | §7.7 |
| 7.14 | 视觉+IMU 紧耦合因子图 | §7.8.1 |
| 7.15 | 紧耦合 VIO vs. 纯视觉里程计对比 | §7.8.1 |
| 7.16 | 双目+IMU+RTK-GPS 紧融合 (停车场) | §7.8.2 |

# 附录 C：本源未涵盖（但本章聚焦要求）的内容 —— 综合 agent 必须从其他源补齐

以下"视觉里程计"核心主题在 SLAM Handbook ch7 中**完全没有推导或公式**，本抽取无法提供（因源中不存在，绝不杜撰）。建议从**高翔《视觉SLAM十四讲》第 7 讲（sf14_07）**或对极几何专著抽取：

1. **对极几何**：对极约束 $x_2^\top E x_1 = 0$、$p_2^\top F p_1 = 0$ 的推导；本质矩阵 $E=t^\wedge R$；基础矩阵 $F=K^{-\top}EK^{-1}$；对极线/对极点。
2. **8 点法 (eight-point algorithm)**：把 $x_2^\top E x_1=0$ 展开为 $e$ 的线性方程、构造系数矩阵 $A$、SVD 求 $e$、对 $E$ 做奇异值修正（强制两个相等奇异值、第三个为 0）。
3. **从 E 恢复 R, t**：SVD $E=U\Sigma V^\top$、四组 $(R,t)$ 解、用深度为正筛选。
4. **单应矩阵 H**：平面情形 $p_2 = H p_1$、直接线性变换 (DLT) 求 H、从 H 分解 $R,t,n$；以及 E 退化（纯旋转/共面点）时为何用 H。
5. **三角化 (triangulation)**：从两视图匹配 + 已知 $R,t$ 求 3D 点深度，线性三角化（DLT）与 SVD 解。
6. **PnP 的闭式/线性解**：
   - **DLT**：从 3D-2D 对应线性求投影矩阵 $[R|t]$；
   - **EPnP**：4 个控制点的重心坐标表示、$Mx=0$ 求解、$\beta$ 系数确定；
   - **P3P** 等最小解。
   - （本源只给了 PnP 的 BA/重投影形式，即式 7.4。）
7. **ICP (3D-3D)**：
   - **SVD 闭式解**：去质心 $q_i=p_i-\bar p$、构造 $W=\sum q_i' q_i^\top$、SVD $W=U\Sigma V^\top$、$R=UV^\top$、$t=\bar p' - R\bar p$；
   - **非线性优化解**（李代数 BA 形式）。
   - （本源只一句提及早期 RGB-D 用 ICP，并给了取代它的 RGB-D 直接残差式 7.13。）
8. **光流 (optical flow) / Lucas-Kanade 完整推导**：
   - **灰度不变假设** $I(x+dx,y+dy,t+dt)=I(x,y,t)$；
   - 一阶泰勒展开 → **光流约束方程** $I_x u + I_y v + I_t = 0$；
   - LK 假设窗口内运动一致 → 最小二乘 $\begin{psmallmatrix}\sum I_x^2&\sum I_xI_y\\\sum I_xI_y&\sum I_y^2\end{psmallmatrix}\begin{psmallmatrix}u\\v\end{psmallmatrix}=-\begin{psmallmatrix}\sum I_xI_t\\\sum I_yI_t\end{psmallmatrix}$；
   - 可解条件（$A^\top A$ 可逆，即角点）。
   - （本源仅在 §7.2.1 一句话把直接法关联到"optical flow"，§7.4.2 给了多帧直接法光度损失式 7.6，但**无单帧 LK 推导**。）
9. **特征点法细节**：Harris 二阶矩矩阵 $M=\sum w\begin{psmallmatrix}I_x^2&I_xI_y\\I_xI_y&I_y^2\end{psmallmatrix}$ 与响应 $R=\det M - k(\mathrm{tr}M)^2$；ORB 的 FAST 角点判据、灰度质心法定向、BRIEF 二进制描述子构造、汉明距离匹配、快速近似最近邻。
   - （本源 §7.3.2 只有文字描述，无这些公式。）

---

**抽取完成说明**：本文件已按源小节顺序（§7.1–§7.9）全量抽取 SLAM Handbook ch7 的全部正文、全部 26 处公式（含 13 个带编号式 7.1–7.13）、Schur 补完整代数推导、全部 16 张图的说明、四种 BA 优化方法分类、四线程前端架构、所有引用系统（LSD-SLAM/DSO/ORB-SLAM/OKVIS/MSCKF/VINS-Mono/BASALT/DM-VIO 等）。记号差异（尤其 ξ 在本源指相机内参 vs. 本书指李代数）已显式标注。附录 C 明确列出本源缺失而本章聚焦要求的 9 类内容（对极几何/8 点法/三角化/PnP-DLT/EPnP/ICP-SVD/LK 光流/Harris-ORB 公式），供综合 agent 从 sf14_07 补齐。
