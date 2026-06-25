# 抽取留痕：设计一个视觉 SLAM 系统（双目/RGB-D VO 工程实现 + ORB-SLAM2 架构对照）

> 本文件为《机器人学笔记》项目内部「抽取留痕」，目标是把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章。**禁摘要、禁凝练**：源里每一步推导、每例与数值、每条定义/定理+完整证明、每张表/分类/算法伪码、每段实践代码均完整记录。公式 LaTeX 写全、标源小节号，宁长勿略。
>
> **服务章节**：设计一个视觉 SLAM 系统（把前面各章 VO / 优化 / 回环 / 建图整合成一个能运行的视觉 SLAM 系统）。
>
> **本章聚焦**（综合 agent 必须覆盖）：系统架构与模块划分；双目/RGB-D 前端（特征提取匹配 / 位姿估计 / 三角化 / 参考帧策略）；帧与地图点的数据结构、关键帧选取准则；局部地图与（滑窗）后端优化；（可选）回环线程；工程实现要点（多线程并发 / 效率）；与 ORB-SLAM2 架构对照。**完整复现《视觉SLAM十四讲》第 13 章的系统设计思路与关键代码骨架**。

---

## 0. 本抽取的源清单（多源融合）

任务给定主源为高翔《视觉SLAM十四讲（第 2 版）》第 13 讲「实践：设计 SLAM 系统」。该章是一篇**工程实践章**：正文只印出了核心类的**部分**代码片段，反复地把"周边代码"（相机类、配置类、数据集类、可视化类、`StereoInit/BuildInitMap/FindFeaturesInRight/DetectFeatures/EstimateCurrentPose` 内部、`g2o` 顶点/边类型、`Map::RemoveOldKeyframe`、三角化实现等）"交给读者自行阅读"。为达成【全量保真 + 自包含】，本抽取**联网研究**，把官方 `slambook2/ch13` 源码仓库中**被正文省略的完整代码骨架**逐文件抽下来，并用《十四讲》第 9、10 讲（后端 BA / Schur 边缘化 / 鲁棒核 / 滑动窗口 / 位姿图）补全本章前端 PnP-BA、后端局部 BA 所依赖的全部理论；再用 ORB-SLAM2 原论文与 SLAM Handbook 第 7 章「Visual SLAM」补全工业级三线程架构（共视图 / 本质图 / 生成树 / 关键帧选取与剔除 / 局部 BA / 回环 Sim(3)）的全部要素，以供"与 ORB-SLAM2 架构对照"。

| 编号 | 源 | 类型 | 物理位置 / URL | 抽取状态 |
| --- | --- | --- | --- | --- |
| **S1** | 高翔《视觉SLAM十四讲（第 2 版）》第 13 讲 实践：设计 SLAM 系统 | 本地教材 MD（697 行，已完整读取） | `/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/13_实践设计SLAM系统.md` | ✅ 全量 |
| **S1b** | 同上 第 10 讲（滑动窗口、位姿图）/ 第 9 讲（BA 稀疏性、Schur、鲁棒核） | 本地教材 MD | `…/10_后端2.md`、`…/09_后端1.md` | ✅ 抽取本章所依赖的理论（滑窗边缘化 fill-in、位姿图雅可比、Schur、Huber） |
| **S2** | 官方 `gaoxiang12/slambook2` 仓库 `ch13/` 完整源码 | C++ 源码（被正文省略的部分，逐文件抓取） | https://github.com/gaoxiang12/slambook2 （`ch13/{include/myslam,src}/`） | ✅ frontend / backend / g2o_types / map / camera / dataset / algorithm / visual_odometry |
| **S3** | Mur-Artal & Tardós, **"ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras"**, IEEE T-RO 33(5), 2017, arXiv:1610.06475 | 论文 | https://arxiv.org/abs/1610.06475 | ✅ 三线程、双目/RGBD 关键点、共视图/本质图/生成树、关键帧选取与剔除、局部 BA、回环 |
| **S4** | Mur-Artal, Montiel & Tardós, **"ORB-SLAM: a Versatile and Accurate Monocular SLAM System"**, IEEE T-RO 31(5), 2015, arXiv:1502.00956 | 论文 | https://arxiv.org/abs/1502.00956 | ✅ 共视图 / 本质图 / Essential Graph 定义、关键帧/地图点选取与剔除原始定义 |
| **S5** | Cadena 等 SLAM Handbook 第 7 章 **Visual SLAM**（本地） | 本地英文教材 MD（72K，相关节已读取） | `/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/07_Visual_SLAM/07_Visual_SLAM.md` | ✅ §7.2–7.4 流水线 / 并行 tracking+mapping / locality / 局部 BA 公式 / Schur / 共视-本质图 θ=15/100 |
| — | 旁证：Klein & Murray PTAM（并行 tracking & mapping 起源 [583]）；Strasdat 等 "keyframe vs filtering" [1039]；OKVIS [89]/DSO [70]（滑窗边缘化） | 二手综述检索 | 见正文引用 | 用于架构源流与"滑窗 vs 关键帧"对比 |

> **代码真实性说明**：S2 中所有 C++ 代码均逐文件从官方仓库 `master` 分支抓取并交叉核对；与 S1 正文印出的片段（`Frame/Feature/MapPoint/Map` 头、`AddFrame/Track/TrackLastFrame` 主体、`Backend::Optimize` 主体）逐字一致。正文省略的 `EstimateCurrentPose / StereoInit / BuildInitMap / FindFeaturesInRight / DetectFeatures / TriangulateNewPoints / InsertKeyframe`、`g2o_types.h`（含雅可比解析式）、`Map::RemoveOldKeyframe / CleanMap`、`Camera`、`Dataset`、`triangulation()`、`VisualOdometry` 驱动等，均来自 S2，已在每段代码上方标注来源文件路径。

---

## ⚠️ 抽取专员的覆盖度说明（综合 agent 务必先读）

| 聚焦主题 | 主源 | 覆盖深度 |
| --- | --- | --- |
| 系统架构与模块划分（前端/后端/地图/相机/配置/数据集/可视化） | S1 §13.2 + S2（`visual_odometry.cpp` 装配） | ✅ 完整（含框图三幅、文件目录约定、模块依赖装配代码） |
| 基本数据结构（Frame / Feature / MapPoint / Map） | S1 §13.3.1 + S2 头文件 | ✅ 完整（含线程锁、weak_ptr 防循环引用、工厂函数、激活窗口） |
| 双目前端：特征提取（GFTT+mask）/ 左右目光流匹配 / 时序光流 / 位姿估计（pose-only BA）/ 三角化 / 参考帧策略 | S1 §13.3.2 + S2 `frontend.cpp/.h` + `algorithm.h` | ✅ 完整（含 `StereoInit/Track/TrackLastFrame/FindFeaturesInRight/EstimateCurrentPose/TriangulateNewPoints/BuildInitMap/DetectFeatures/InsertKeyframe` 全代码 + 三角化 SVD + pose-only BA 的雅可比解析式） |
| 关键帧选取准则 | S1 §13.3.2(4) + S2（`num_features_needed_for_keyframe_`）+ S3/S4（ORB-SLAM 四条件） | ✅ 完整（十四讲"跟踪内点过少即关键帧"+ ORB-SLAM2 显式四条件 + RGBD 近点阈值） |
| 局部地图 / 激活窗口 / 滑窗后端优化 | S1 §13.3.3 + §13.3.1（`num_active_keyframes_=7`、`RemoveOldKeyframe`）+ S2 `backend.cpp` + S1b §10.1（滑窗边缘化理论） | ✅ 完整（局部 BA 全代码 + chi2 外点自适应 + 滑窗 fill-in 理论 + OKVIS 边缘化策略） |
| 后端 BA 理论依赖（重投影误差 / H 稀疏 / Schur 边缘化 / 鲁棒核 / 信息矩阵 / chi2=5.991） | S1b §9.2.3–9.2.4 + S5 §7.4.3 | ✅ 完整（Schur 消元三步式 + Huber 核 + 自由度-2 卡方 0.05 临界值 5.991 来源） |
| 位姿图（回环线程理论支撑） | S1b §10.2 | ✅ 完整（位姿图误差定义、左扰动伴随推导、雅可比 (10.9)(10.10)、目标函数 (10.12)） |
| 回环线程（可选） | S1 习题3 + S3/S4 §VI–VII | ✅ ORB-SLAM2 回环（DBoW2 候选 / Sim(3) 几何验证 / 回环融合 / 本质图 PGO / 全局 BA）完整骨架；十四讲 ch13 本身**未实现回环**（仅作为习题），已标注 |
| 工程实现要点（多线程 / 数据结构 / 内存 / 效率） | S1 §13.3.1、§13.4 + S2 + S3 | ✅ 完整（互斥锁/条件变量/原子布尔/线程生命周期、shared_ptr+weak_ptr、unordered_map 散列、激活窗口控规模、耗时 16ms、内存增长问题） |
| 与 ORB-SLAM2 架构对照 | S3 + S4 + S5 §7.2–7.4 | ✅ 完整（三/四线程、共视图 θ、本质图 θ=100、生成树、双目三坐标 (uL,vL,uR)、近/远点 40×baseline、关键帧四条件、关键帧剔除 90%、局部 BA K1/K2 固定策略） |
| 实验：Kitti 双目运行 | S1 §13.4 | ✅ 完整（数据集 URL、配置、运行命令、耗时与内存观察） |

> **本章与 ORB-SLAM2 的定位差异（综合时务必点明）**：S1（十四讲 ch13）实现的是一个**精简双目 VO**（光流前端 + 局部 BA 后端 + 激活窗口），**没有**回环、没有重定位、没有词袋、没有共视图/本质图的显式数据结构，其"窗口"是**纯时序的最近 N=7 关键帧**（外加一条"删最近还是删最远"的启发式）。ORB-SLAM2 是**完整 SLAM**（三/四线程 + 共视图 + 本质图 + 回环 + 重定位 + 地图复用）。本抽取把两者并置，正是为了让成书既给出"能跑的最小系统骨架"，又给出"工业级完整系统应当长什么样"。

---

## 1. 记号约定与本书统一约定的对齐（含差异说明）

> 本书统一约定：旋转 $R\in SO(3)$；**右扰动为主**；李代数切向量 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前、旋转在后）；Hamilton 四元数；过程噪声协方差 $\Sigma_w$、观测噪声协方差 $\Sigma_v$。

| 概念 | 本源记号（S1/S2/S3/S5） | 本书/统一记号 | 差异与处理 |
| --- | --- | --- | --- |
| 旋转 | $R\in SO(3)$；代码 `SO3` / `Sophus::SO3d` | $R\in SO(3)$ | 一致（用 R，不用 Barfoot 的 C） |
| 位姿 | $T\in SE(3)$；代码 `SE3` / `Sophus::SE3d`；约定 **`pose_` = $T_{cw}$**（world→camera） | $T\in SE(3)$ | 一致。⚠️ 本源相机位姿存的是 $T_{cw}$（"Tcw 形式 Pose"，S1 §13.3.1 代码注释），由相机系到世界系需取逆 $T_{wc}=T_{cw}^{-1}$（见 `current_pose_Twc = current_frame_->Pose().inverse()`）。 |
| 李代数扰动 / 位姿更新 | g2o `VertexPose::oplusImpl`: `_estimate = SE3::exp(update) * _estimate`，即 **左乘**更新 $T\leftarrow\exp(\delta\xi^\wedge)\,T$（**左扰动**） | 本书**右扰动为主** | ⚠️ **关键差异**：S2 的 `g2o_types.h` 用**左扰动**，故 `EstimateCurrentPose`/局部 BA 的位姿雅可比是左扰动形式。S1b 第 10 讲位姿图也用左扰动（式 10.5）。综合到本书（右扰动）时，重投影误差对位姿的 $2\times6$ 雅可比块的"旋转 3 列"需按右扰动重新推导（结果差一个伴随/符号；解析式见 §6.3，已逐项给出左扰动版本并加注右扰动转换提示）。 |
| 扰动向量排序 | 代码 `Vec6 update << update[0..5]`；雅可比 `_jacobianOplusXi` 前 3 列为平移、后 3 列为旋转 | $\xi=[\rho;\phi]$（平移在前、旋转在后） | **一致**：`_jacobianOplusXi` 前 3 列 $(-f_x/Z,\,0,\,f_xX/Z^2;\dots)$ 对应平移，后 3 列对应旋转，与 $\xi=[\rho;\phi]$ 同序。 |
| 内参 | $K$；分量 $f_x,f_y,c_x,c_y$；代码 `fx_,fy_,cx_,cy_` | 同 | 一致 |
| 双目基线 | $b$ / `baseline_` | $b$ | 一致 |
| 投影函数 | $\pi(\cdot)$（S5 式 7.3/7.4）；代码 `camera2pixel` | $\pi(\cdot)$ | 一致 |
| 路标点 / 地图点 | $p_j$ / $x_j^w$（S5）；代码 `MapPoint::pos_`（$\in\mathbb R^3$，世界系） | $p_j$ | 一致；S1 明确"路标、路标点、地图点语义相同"（§13.2）。 |
| 重投影误差 | $e_{ij}=z_{ij}-\pi(\cdot)$；代码 `_error = _measurement - pos_pixel.head<2>()` | $e$ | 一致（观测减预测） |
| 信息矩阵 | 代码 `setInformation(Mat22::Identity())`，即 $\Omega=\Sigma_{ij}^{-1}=I_{2\times2}$ | $\Sigma_v^{-1}$ | 一致（像素观测各向同性单位权）。本书统一观测噪声协方差用 $\Sigma_v$，此处 $\Sigma_v=I$。 |
| 鲁棒核阈值 | `chi2_th = 5.991`（Huber `setDelta`，也用作外点判据） | — | 自由度=2 的卡方分布 0.05 显著性临界值 $\chi^2_{2,0.05}=5.991$（见 §7.2 推导）。 |
| ORB-SLAM2 双目关键点 | $\mathbf x_s=(u_L,v_L,u_R)$ | — | 三坐标表示；本书无对应符号，沿用论文记号并解释。 |
| ORB-SLAM2 共视阈值 | $\theta$（共视图 θ=15 例；本质图 θ=100） | — | 沿用 θ。 |
| 协方差 / 噪声 | S1/S2 仅 `Mat22::Identity()`；S5 用 $\Sigma_{ij}$ | $\Sigma_w,\Sigma_v$ | 本章后端无运动噪声 $\Sigma_w$；观测 $\Sigma_v=\Sigma_{ij}$。 |

**其他记号备注**：
- S1（十四讲 ch13）属"工程叙述"，**无定理/引理/命题编号体系**；正文无独立编号公式（核心是代码）。本抽取所引公式编号 (10.x)/(9.x) 来自第 10/9 讲，(7.x) 来自 Handbook 第 7 章，均逐一保留源编号。
- 代码风格：成员变量尾下划线 `xxx_`；`Ptr = std::shared_ptr<T>`；`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 为 Eigen 16 字节对齐宏；`CV_FILLED`、`cv::Size(11,11)` 等为 OpenCV 常量/参数。
- 类型别名（slambook2 `common_include.h`，本抽取据 S2 上下文还原）：`Vec2/Vec3/Vec6 = Eigen::Vector{2,3,6}d`；`Mat33/Mat34/Mat22 = Eigen::Matrix{3x3,3x4,2x2}d`；`MatXX/VecX = 动态尺寸`；`SE3=Sophus::SE3d`，`SO3=Sophus::SO3d`；`Mat=cv::Mat`。

---

## 2. 章首：本章目标与定位（S1 §开篇、§13.1）

### 2.1 主要目标（S1 原文逐条）

1. 实际设计一个视觉里程计。
2. 理解 SLAM 软件框架是如何搭建的。
3. 理解在视觉里程计设计中容易出现的问题，以及修补的方式。

本讲是全书的总结部分：用前面所学的知识，实际书写一个视觉里程计程序，管理局部的机器人轨迹与路标点，并体验软件框架的组成。操作中会遇到许多实际问题：如何对图像进行连续的追踪、如何控制 BA 的规模等。为让程序稳定运行，需要处理种种情况，这会带来工程实现方面的有益讨论。

### 2.2 为什么单独列工程章节（S1 §13.1）

- **核心论断**："知晓砖头和水泥的原理，并不代表能够建造伟大的宫殿。" 在 SLAM 中，工程实现和理解算法原理**至少同等重要**，甚至更应强调如何书写实际可用的程序。算法原理像一个个方块，理解单个方块并不能帮你建造真正的建筑——建造需要大量尝试、时间和经验。
- 一个实用程序会有很多工程设计和技巧，还需讨论每一步出现问题之后如何处理。原则上每个人实现的 SLAM 都会有所不同，多数时候并不能说哪种实现一定最好。但通常会遇到一些**共同问题**："怎么管理地图点""如何处理误匹配""如何选择关键帧"等。
- **方法论**：从简单数据结构出发，先做一个简单的视觉里程计，再慢慢把额外功能加进来——把"由简到繁"的过程展现给读者（"像雪人那样慢慢堆起来"）。

### 2.3 本章实现什么（S1 §13.1 末）

- 代码放在 `slambook2/ch13`，实现一个**精简版双目视觉里程计**，在 **Kitti** 数据集上运行。
- 该 VO 由**一个光流追踪的前端**和**一个局部 BA 的后端**组成。
- **为什么选双目 VO**：
  1. 双目实现相对简单，只需**单帧即可完成初始化**（无单目的尺度不确定/初始化难题）；
  2. 双目存在 **3D 观测**（左右目三角化即得深度），实现效果比单目好。

---

## 3. 系统架构与模块划分（S1 §13.2）

### 3.1 工程目录约定（S1 §13.2 原文逐条）

大多数 Linux 库按模块对算法代码文件分类存放。本工程按小型算法库的普遍做法分类：

1. **`bin`**：存储编译好的二进制文件。
2. **`include/myslam`**：存放 SLAM 模块的头文件（`.h`）。这种做法的目的是：当把包含目录设到 `include`，引用自己的头文件时写 `#include "myslam/xxx.h"`，不易和别的库混淆。
3. **`src`**：存放源代码文件（`.cpp`）。
4. **`test`**：存放测试用的文件（`.cpp`）。
5. **`config`**：存放配置文件。
6. **`cmake_modules`**：存放第三方库的 cmake 文件（使用 g2o 之类的库时会用到）。

### 3.2 确定核心算法结构：数据结构 + 算法（S1 §"确定核心算法结构"）

> 老观点："程序 = 数据结构 + 算法"。针对视觉里程计要问：处理怎样的数据？关键算法有哪些？它们之间什么关系？

**最基本的数据单元**（S1 逐条）：
- 处理的最基本单元是**图像**。双目视觉里那是**一对图像**，称为**一帧（Frame）**。
- 对帧提取**特征（Feature）**，这些特征是很多 **2D 点**。
- 在图像之间寻找特征的关联。如果能多次看到某个特征，就可用**三角化**计算它的 **3D 位置**，即**路标（MapPoint，地图点）**。

> **图 13-2（基本数据结构及关系）**：Frame 持有若干 Feature（2D）；Feature 关联到 MapPoint（3D）；MapPoint 被多个 Feature 观测。
> 综合建议用 TikZ 画三层关系：`Frame`(矩形, 含 left/right 图) → 多个 `Feature`(小圆, 2D 像素) → `MapPoint`(三维点, 被多 Feature 指向)。

**算法的前后端划分**（S1 §13.2）：SLAM 由前后端组成。
- **前端**：计算相邻图像的特征匹配；**快速处理保证实时性**。
- **后端**：优化整个问题；**优化关键帧以保证良好结果**。
- 典型实现中二者**各有线程**。

两个重要模块的职责（S1 逐条）：

- **前端**：往前端插入图像帧 → 提取图像中的特征 → 与上一帧进行光流追踪 → 通过光流结果计算该帧定位 → 必要时补充新特征点并三角化。**前端处理结果作为后端优化的初始值**。
- **后端**：较慢的线程，拿到处理之后的关键帧和路标点 → 对它们优化 → 返回优化结果。**后端应控制优化问题规模在一定范围内，不能随时间一直增长**。

### 3.3 流水线框架（S1 §13.2，图 13-3）

> **图 13-3（算法框架图）**：在前后端之间放一个**地图模块（Map）**处理数据流动。前后端在分别的线程中处理数据。预想流程：
> - 前端提取了关键帧后，往地图中**添加新数据**；
> - 后端检测到地图更新时，运行一次优化，然后把地图中**旧的关键帧和地图点去掉**，保持优化规模。
>
> 综合建议 TikZ 横向流水线：`Dataset`→`Frontend`(Tracking 线程)→`Map`(共享, 加锁)→`Backend`(优化线程)，`Map`↘`Viewer`(可视化线程)；前端→后端用 `UpdateMap()` 条件变量触发箭头标注。

**周边小模块**（S1 §13.2 逐条，"不算核心但不可或缺"）：
- **相机类（Camera）**：管理相机的内外参和投影函数。
- **配置文件管理类（Config）**：方便从配置文件读取内容；配置文件记录重要参数，方便调整。
- **数据集类（Dataset）**：因为算法在 Kitti 上运行，需按 Kitti 存储格式读取图像数据。
- **可视化模块（Viewer）**：观察系统运行状态，否则"得对着一串串数值挠头"。

> S1 原文："限于篇幅，我们将周边代码交给读者自行阅读，在书中只介绍核心部分。" → 本抽取 §8（周边模块）据 S2 补全 Camera/Dataset/VisualOdometry 驱动的完整代码。

### 3.4 模块依赖与装配（S2 `visual_odometry.cpp`，正文省略，全量补全）

整个系统由 `VisualOdometry` 类装配。这是理解"模块划分如何变成可运行系统"的关键，正文未印出，据 S2 全量给出：

> 来源：`slambook2/ch13/src/visual_odometry.cpp`

```cpp
//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"

namespace myslam {

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

}  // namespace myslam
```

**装配关系解读**（供综合 agent）：
- `Init()` 完成**依赖注入**：`Frontend` 持有 `Backend`/`Map`/`Viewer`/左右 `Camera` 的指针；`Backend` 持有 `Map` 与左右相机；`Viewer` 持有 `Map`。`Map` 是前后端与可视化的**共享中枢**。
- `dataset_->GetCamera(0)`/`(1)` = 左/右目相机（含外参，见 §8.2）。
- `Step()` 顺序：从数据集取下一帧 → 计时 → 喂给前端 `AddFrame` → 打印单帧耗时（这就是 §13.4 报告 16ms 的来源）。**整个主循环是单线程拉帧**，真正的并行发生在 `Backend` 构造时启动的优化线程与 `Viewer` 线程里。

---

## 4. 基本数据结构（S1 §13.3.1，全量代码 + S2 补全的成员）

S1 §13.3.1 原文：对于基本数据结构，通常建议设成 `struct`，无须复杂私有变量和接口。考虑到这些数据可能被**多个线程**访问和修改，在关键部分需要加上**线程锁**。

### 4.1 Frame（帧）

> 来源：`slambook2/ch13/include/myslam/frame.h`（S1 正文分两段印出，此处合并为完整声明）

```cpp
struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;           // id of this frame
    unsigned long keyframe_id_ = 0;  // id of key frame
    bool is_keyframe_ = false;       // 是否为关键帧
    double time_stamp_;              // 时间戳，暂不使用
    SE3 pose_;                       // Tcw 形式 Pose
    std::mutex pose_mutex_;          // Pose 数据锁
    cv::Mat left_img_, right_img_;   // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

   public:  // data members
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    /// 设置关键帧并分配关键帧 id
    void SetKeyFrame();

    /// 工厂构建模式，分配 id
    static std::shared_ptr<Frame> CreateFrame();
};
```

**S1 解读**：Frame 含 id、位姿、图像及左右图像中的特征点。`Pose` 会被前后端**同时设置或访问**，所以定义 `Pose()`/`SetPose()`，函数内**加锁**（`pose_mutex_`）。Frame 可由**静态工厂函数** `CreateFrame()` 构建，函数内自动分配 id。

### 4.2 Feature（特征）

> 来源：`slambook2/ch13/include/myslam/feature.h`

```cpp
struct Feature {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;          // 持有该 feature 的 frame
    cv::KeyPoint position_;               // 2D 提取位置
    std::weak_ptr<MapPoint> map_point_;   // 关联地图点

    bool is_outlier_ = false;        // 是否为异常点
    bool is_on_left_image_ = true;   // 标识是否提在左图，false 为右图

   public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
```

**S1 解读**：Feature 最主要的信息是自身的 **2D 位置**（`position_`）。`is_outlier_` 为异常点标志位；`is_on_left_image_` 标识它是否在左侧相机提取（`false` 为右图）。可通过一个 Feature 对象访问持有它的 Frame 及对应路标。**关键设计**：Frame 和 MapPoint 的实际持有权归**地图**所有；为避免 `shared_ptr` 产生的**循环引用**，这里 `frame_` 和 `map_point_` 使用 **`weak_ptr`**。

> **循环引用注释（S1 脚注）**：若 Feature 用 `shared_ptr` 强引用 Frame，而 Frame 又强引用其 Feature 列表，则二者引用计数互相不归零、永不析构 → 内存泄漏。`weak_ptr` 不增加引用计数，使用时 `.lock()` 临时提升为 `shared_ptr`（若对象已销毁则得 `nullptr`），这是本工程内存安全的核心技巧之一。

### 4.3 MapPoint（路标点 / 地图点）

> 来源：`slambook2/ch13/include/myslam/mappoint.h`

```cpp
struct MapPoint {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;  // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();  // Position in world
    std::mutex data_mutex_;
    int observed_times_ = 0;  // being observed by feature matching algo.
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}

    MapPoint(long id, Vec3 position);

    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void SetPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    };

    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // factory function
    static MapPoint::Ptr CreateNewMappoint();
};
```

**S1 解读**：MapPoint 最主要的信息是它的 **3D 位置**（`pos_`，世界系），同样需要上锁（`data_mutex_`）。`observations_` 记录了自己被哪些 Feature 观察（用 `weak_ptr` 避免循环引用）。因为 Feature 可能被判为 outlier，所以 `observations_` 部分发生改动时也需要锁定。`observed_times_` 计被观测次数。

> **`RemoveObservation` 实现**（S2 `mappoint.cpp`，正文未印，逻辑补全）：遍历 `observations_` 找到匹配的 `weak_ptr`，erase 之，`observed_times_--`，并将该 Feature 的 `map_point_.reset()`。后端剔除外点观测（§7.3）、地图删旧帧（§4.5）时都会调用它。

### 4.4 Map（地图）

> 来源：`slambook2/ch13/include/myslam/map.h`（S1 正文分两段印出，合并）

```cpp
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    /// 增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);
    /// 增加一个地图顶点
    void InsertMapPoint(MapPoint::Ptr map_point);

    /// 获取所有地图点
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    /// 获取所有关键帧
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 获取激活地图点
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /// 获取激活关键帧
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /// 清理 map 中观测数量为零的点
    void CleanMap();

   private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks
    LandmarksType active_landmarks_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  // 激活的关键帧数量
};
```

**S1 解读（核心设计思想）**：
- 地图以**散列（`unordered_map`）形式**记录所有的关键帧（`keyframes_`）和对应路标点（`landmarks_`），同时维护一个**被激活的**关键帧（`active_keyframes_`）和地图点（`active_landmarks_`）。
- **"激活"的概念即我们所谓的窗口**，它随着时间往前推动。**后端只从地图中取出激活的关键帧、路标点进行优化，忽略其余部分**，达到控制优化规模的效果。
- 激活策略由我们自己定义。简单策略：去除最旧的关键帧，保持时间上最新的若干个。**本书实现只保留最新的 7 个关键帧**（`num_active_keyframes_ = 7`）。
- 散列表用 id 作 key，**插入/查找/删除平均 O(1)**，是工程上选 `unordered_map` 而非 `vector` 的原因（地图点会频繁按 id 增删）。

> **本章"窗口"= 纯时序最近 N 帧 + 启发式**，与 ORB-SLAM2 的"共视窗口"形成对比（见 §9）。`RemoveOldKeyframe` 的实现见下面 §4.5。

### 4.5 Map 的实现：插入 / 删旧帧（激活窗口滑动）/ 清理（S2 `map.cpp`，正文未印，全量补全）

S1 §13.3.1 只印出 `map.h` 声明，未印 `map.cpp` 实现——而**激活窗口如何滑动、删哪一帧**正是本章"控制规模"的核心机制，全量从 S2 补全：

> 来源：`slambook2/ch13/src/map.cpp`

```cpp
void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframes_) {
        RemoveOldKeyframe();
    }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    if (landmarks_.find(map_point->id_) == landmarks_.end()) {
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;
    // 寻找与当前帧最近与最远的两个关键帧
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;
        auto dis = (kf.second->Pose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;  // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_th) {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // 删掉最远的
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // remove keyframe and landmark observation
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feat : frame_to_remove->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }
    for (auto feat : frame_to_remove->features_right_) {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feat);
        }
    }

    CleanMap();
}

void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin();
         iter != active_landmarks_.end();) {
        if (iter->second->observed_times_ == 0) {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}
```

**解读（激活窗口滑动的核心启发式，本章"滑窗"的真正实现）**：
- **`InsertKeyFrame`**：把新关键帧同时放进 `keyframes_`（全量）和 `active_keyframes_`（激活）。一旦激活数 > `num_active_keyframes_`(=7) 就调 `RemoveOldKeyframe` 滑窗。
- **`RemoveOldKeyframe`（删最近还是删最远？）**：遍历激活关键帧，按李代数距离 `dis = (kf->Pose() * Twc).log().norm()`（$\|\ln(T_{kf,w}T_{wc})^\vee\|$，即该关键帧相对当前帧的相对位姿的 $\mathfrak{se}(3)$ 模长）找出**离当前帧最近**(`min_dis`/`min_kf_id`)和**最远**(`max_dis`/`max_kf_id`)的两个。
  - 若**最近距离 < 0.2**（`min_dis_th=0.2`）：说明有一帧和当前帧几乎重合（相机近乎静止/慢动），**优先删这个最近的冗余帧**（避免窗口"缩成一团"退化，呼应 S1b §10.1.1"按某原则取时间靠近、空间能展开的关键帧"）。
  - 否则：**删最远的**（保持窗口尽量聚焦在当前位置附近）。
- 删帧时把该帧左右目所有特征对应地图点的观测都 `RemoveObservation`（断开），然后 `CleanMap`。
- **`CleanMap`**：遍历激活地图点，删掉 `observed_times_ == 0`（已无任何观测）的点（这些点是删帧后变成孤儿的）。
> ⚠️ **注意**：被删的关键帧只从 `active_keyframes_` 移除，**仍留在 `keyframes_`（全量）里**；地图点同理（`landmarks_` 不删）——这就是 §10 / §11.3 所述"长跑内存增长"的根源。也注意这是**直接丢弃**（非严格边缘化），与 §7.5 滑窗边缘化理论的差异在此具体体现。

---

## 5. 双目前端（S1 §13.3.2 + S2 `frontend.cpp/.h` 全量补全）

### 5.1 前端状态机与处理逻辑（S1 §13.3.2 原文逐条）

前端根据双目图像确定该帧位姿。实现中存在不同选择（S1 原文给出的开放性问题）：
- 应该怎样使用右目图像？每一帧都和左右目各比较一遍，还是仅比较左右目之一？
- 三角化时考虑左右目三角化，还是时间上前后帧三角化？实际中任意两张图像都可做三角化（比如前一帧左图对下一帧右图），所以每个人实现都会不太一样。

**本书前端的确定逻辑**（S1 逐条）：

1. 前端本身有**初始化、正常追踪、追踪丢失**三种状态。
2. **初始化**状态：根据左右目之间的光流匹配，寻找可三角化的地图点，成功时建立初始地图。
3. **追踪**阶段：前端计算上一帧特征点到当前帧的光流，据光流结果计算图像位姿。**该计算只使用左目图像，不使用右目**。
4. 如果追踪到的点较少，就**判定当前帧为关键帧**。对关键帧做以下几件事：
   - 提取新的特征点；
   - 找到这些点在右图的对应点，用三角化建立新路标点；
   - 将新关键帧和路标点加入地图，并**触发一次后端优化**。
5. 如果**追踪丢失**，就重置前端系统，重新初始化。

### 5.2 前端类声明（S2 `frontend.h`，正文未印，全量补全）

```cpp
#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * 前端
 * 估计当前帧 Pose，在满足关键帧条件时向地图加入关键帧并触发优化
 */
class Frontend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    /// 外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);

    /// Set 函数
    void SetMap(Map::Ptr map) { map_ = map; }
    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
    FrontendStatus GetStatus() const { return status_; }
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

   private:
    bool Track();
    bool Reset();
    int TrackLastFrame();
    int EstimateCurrentPose();
    bool InsertKeyframe();
    bool StereoInit();
    int DetectFeatures();
    int FindFeaturesInRight();
    bool BuildInitMap();
    int TriangulateNewPoints();
    void SetObservationsForKeyFrame();

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧 pose 初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};

}  // namespace myslam

#endif  // MYSLAM_FRONTEND_H
```

**关键参数表**（S2 默认值，供综合 agent 列表）：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `num_features_` | 200 | 每帧 GFTT 提取的特征点数上限 |
| `num_features_init_` | 100 | 初始化时左右目成功匹配的最少特征数（少于则初始化失败） |
| `num_features_tracking_` | 50 | 跟踪内点数 > 此值 → `TRACKING_GOOD` |
| `num_features_tracking_bad_` | 20 | 跟踪内点数 > 此值（且 ≤50）→ `TRACKING_BAD`；≤20 → `LOST` |
| `num_features_needed_for_keyframe_` | 80 | 跟踪内点数 < 此值 → 当前帧设为关键帧 |

> 注意 `Frontend()` 构造里 `num_features_init_` 与 `num_features_` 实际从配置文件 `Config::Get<int>` 读取（见 §5.3 构造函数），上表为头文件默认值。

### 5.3 前端构造 + 主入口 `AddFrame`（S1 §13.3.2 印出主体 + S2 构造函数）

> 来源：`slambook2/ch13/src/frontend.cpp`

```cpp
Frontend::Frontend() {
    gftt_ =
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}
```

**解读**：
- GFTT（Good Features To Track，Shi-Tomasi 角点）创建参数：最大角点数 `num_features`、品质等级 `0.01`、最小间距 `20` 像素。
- `AddFrame` 是**状态机分发器**：`INITING`→`StereoInit`；`TRACKING_GOOD/BAD`→`Track`；`LOST`→`Reset`。处理完把当前帧存为 `last_frame_`。

### 5.4 追踪主流程 `Track`（S1 §13.3.2 全量）

```cpp
bool Frontend::Track() {
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}
```

**解读（参考帧策略与运动模型）**：
- **恒速运动模型**：用上一帧间的相对运动 `relative_motion_` 作为当前帧位姿的**初值** `current_frame_->pose = relative_motion_ * last_frame_->pose`。这里 `pose_` 是 $T_{cw}$，故 `relative_motion_` $=T_{c_k w}T_{c_{k-1} w}^{-1}=T_{c_k c_{k-1}}$（当前相机相对上一相机）。
- `TrackLastFrame()`：用 LK 光流把上一帧特征追到当前帧。
- `EstimateCurrentPose()`：用追到的 2D-3D 对应做 **pose-only BA**（PnP 的优化形式），返回内点数。
- 据内点数置状态（good/bad/lost）。
- `InsertKeyframe()`：内点过少时插关键帧。
- 更新 `relative_motion_` 供下一帧用。
- **参考帧策略**：本系统是**帧-帧追踪**（current vs last），位姿初值来自恒速模型；地图点的世界坐标恒定，故"上一帧→当前帧"等价于"局部地图→当前帧"的间接对齐。

### 5.5 时序光流追踪 `TrackLastFrame`（S1 §13.3.2 全量）

```cpp
int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->features_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}
```

> ⚠️ **S1 正文 vs S2 实际代码的差异（OCR/版本）**：S1 正文印的 `calcOpticalFlowPyrLK` 窗口为 `cv::Size(21, 21)`，而 S2 仓库实际为 `cv::Size(11, 11)`。两处其余参数一致（金字塔层数 3、终止准则 COUNT+EPS/30/0.01、`OPTFLOW_USE_INITIAL_FLOW`）。本抽取以**仓库 S2 的 11×11** 为准，差异在此标注。

**解读（关键工程技巧）**：
- **用重投影点作光流初值**：若上一帧特征已关联地图点，则把地图点按当前帧位姿初值**投影**到当前帧像素 `world2pixel(mp->pos_, current_frame_->Pose())`，作为 `kps_current` 的初始猜测；配合 `OPTFLOW_USE_INITIAL_FLOW` 标志，光流从这个更接近真值的初值出发收敛更快更准。没有地图点的特征则用原像素作初值。
- LK 金字塔光流（`calcOpticalFlowPyrLK`）：3 层金字塔，每点 11×11 窗口。
- 追踪成功（`status[i]`）的点在当前帧建一个新 Feature，继承上一帧对应特征的 `map_point_`（地图点关联随光流传递），追加到 `current_frame_->features_left_`。
- 详细 LK 光流推导（灰度不变假设、最小二乘 $A^\top A\,[u\;v]^\top=-A^\top b$、反向光流、图像金字塔由粗到精）见视觉里程计 2 章（本仓库 `visual_odometry__sf14_08.md`），此处仅给系统集成接口。

### 5.6 当前帧位姿估计 `EstimateCurrentPose`（pose-only BA，S1 正文未印，S2 全量补全 + 雅可比）

这是本章前端的**核心优化**——固定地图点、只优化当前帧位姿的单顶点 g2o（即 PnP 的非线性优化形式 / motion-only BA）。S1 正文只说"用 PnP/光流结果计算位姿"，未印代码，据 S2 全量给出：

> 来源：`slambook2/ch13/src/frontend.cpp`

```cpp
int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}
```

**解读（工程要点，供综合 agent 详写）**：
- **求解器**：`BlockSolver_6_3`（位姿块 6 维、路标块 3 维的预置块求解器；这里只用到位姿 6 维），线性求解器用 **`LinearSolverDense`**（前端单顶点小问题，稠密更快），LM 下降。
- **单顶点**：只有一个 `VertexPose`（当前帧位姿，初值 = 恒速模型给的 `current_frame_->Pose()`）。
- **多条一元边** `EdgeProjectionPoseOnly`：每条对应一个"已关联地图点的左目特征"，测量值 = 该特征 2D 像素，信息矩阵 $\Omega=I_{2\times2}$，**Huber 鲁棒核**。
- **迭代 4 轮的外点剔除**（"先优化再剔点"循环）：每轮 `optimize(10)`，然后按 `chi2() > 5.991` 标记外点（`setLevel(1)` 使其下轮不参与），内点 `setLevel(0)`。**`iteration==2` 时关掉鲁棒核**（`setRobustKernel(nullptr)`）——前期用 Huber 抗外点、后期去核做精确优化，是常见技巧。每轮都把顶点估计**重置回**原始 `current_frame_->Pose()` 再优化，使外点判定基于同一初值更稳定。
- **返回内点数** = `features.size() - cnt_outlier`（即 `tracking_inliers_`）。
- 最后把判为外点的特征 `map_point_.reset()`（断开错误关联），但 `is_outlier_` 复位为 `false`（"将来也许还能用"）。

**pose-only BA 的数学**（依赖 S1b 第 9 讲，本章实际用到）：
对单个观测，重投影误差
$$
\boldsymbol e_i=\boldsymbol z_i-\pi\!\big(K\,(T\,\boldsymbol p_i)\big),\qquad
\pi([X,Y,Z]^\top)=\Big[f_x\tfrac{X}{Z}+c_x,\ f_y\tfrac{Y}{Z}+c_y\Big]^\top,
$$
其中 $T=T_{cw}$（待优化），$\boldsymbol p_i=$ 地图点世界坐标（固定）。目标
$$
T^\star=\arg\min_{T}\ \frac12\sum_i \rho\!\Big(\big\|\boldsymbol e_i\big\|_{\Sigma_v^{-1}}^2\Big),\qquad \Sigma_v=I_{2\times2},\ \rho=\text{Huber}.
$$
其雅可比 $\partial\boldsymbol e/\partial\delta\xi$（$2\times6$）的解析式见 §6.3（`g2o_types.h::EdgeProjectionPoseOnly::linearizeOplus`，左扰动版本）。

### 5.7 双目初始化 `StereoInit` / `DetectFeatures` / `FindFeaturesInRight` / `BuildInitMap`（S2 全量补全）

S1 正文只描述了初始化逻辑、未印代码。这一组函数是"单帧双目初始化"的实现，据 S2 全量给出。

> 来源：`slambook2/ch13/src/frontend.cpp`

```cpp
bool Frontend::StereoInit() {
    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
        auto mp = kp->map_point_.lock();
        if (mp) {
            // use projected points as initial guess
            auto px =
                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            // use same pixel in left iamge
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}
```

**解读（双目前端的特征提取-匹配-三角化全链路）**：
- **`DetectFeatures`（带掩膜的特征提取）**：建一张全白 `mask`，对**已有特征**周围 10×10 像素画黑矩形（`CV_FILLED`）→ GFTT 只在没有特征的区域提新点，避免特征扎堆。这就是"初始化提特征"和"关键帧补提特征"复用的同一函数（呼应 S1 §13.3.2 末"初始化阶段提特征和关键帧提特征可用同一函数"）。
- **`FindFeaturesInRight`（左右目光流匹配）**：用 LK 光流把左目特征追到右图（而非传统的极线一维搜索）。有地图点的用右目投影作初值，否则用左目同像素作初值。匹配成功的在右图建 Feature（`is_on_left_image_=false`）；失败的在 `features_right_` 对应位置**填 `nullptr`**（保持左右下标一一对应）。
- **`BuildInitMap`（单帧双目三角化建图）**：用**左右目相机外参** `{camera_left_->pose(), camera_right_->pose()}` 作两视角，把左右匹配像素**反投影到归一化平面**（`pixel2camera`）后做三角化（`triangulation`），深度 `pworld[2]>0` 才接受。为每个有效三角化点建 MapPoint，添加左右两个观测，关联回左右 Feature，插入地图。最后把当前帧设为**第一个关键帧**、插入地图、`backend_->UpdateMap()` 触发后端。
- **`StereoInit` 串联**：提左目特征 → 右目光流匹配 → 匹配数 ≥ `num_features_init_`(=100) 才继续 → 建初始地图 → 成功则状态转 `TRACKING_GOOD`。

> **双目"单帧初始化"是选双目的核心理由**（呼应 §2.3）：单帧左右图即可三角化出带**真实尺度**的初始地图，无需单目那种"两帧足够视差 + 单应/本质矩阵分解 + 尺度归一"的初始化。

### 5.8 三角化实现 `triangulation`（S2 `algorithm.h`，正文未印，全量补全）

> 来源：`slambook2/ch13/include/myslam/algorithm.h`

```cpp
/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}

// converters
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }
```

**线性三角化（SVD/DLT）数学推导**：设第 $i$ 个视角投影矩阵 $P_i=[T_i]_{3\times4}$（这里 `poses[i].matrix3x4()`），归一化平面观测 $\boldsymbol x_i=[x_i,y_i,1]^\top$。投影关系 $s_i\boldsymbol x_i=P_i\,\boldsymbol X$（$\boldsymbol X$ 为齐次 3D 点）。消去尺度 $s_i$（叉乘 $\boldsymbol x_i\times P_i\boldsymbol X=0$）得每视角两条线性方程：
$$
x_i\,(\boldsymbol p_i^{(3)})^\top\boldsymbol X-(\boldsymbol p_i^{(1)})^\top\boldsymbol X=0,\qquad
y_i\,(\boldsymbol p_i^{(3)})^\top\boldsymbol X-(\boldsymbol p_i^{(2)})^\top\boldsymbol X=0,
$$
其中 $\boldsymbol p_i^{(k)}$ 是 $P_i$ 第 $k$ 行（代码 `m.row(k-1)`）。堆叠所有视角得 $A\,\boldsymbol X=0$（$A\in\mathbb R^{2N\times4}$）。对 $A$ 做 SVD，解 $\boldsymbol X$ 取 $V$ 的最后一列（最小奇异值对应右奇异向量），再齐次归一化 `col(3)/V(3,3)` 取前 3 维。
- **质量检验**：`singularValues()[3]/singularValues()[2] < 1e-2` 判断最小奇异值是否远小于次小奇异值（解是否"足够零空间"、视差是否足够）。

> ⚠️ **已知代码缺陷（S2 仓库原样，需在成书中修正或加注）**：本函数的返回逻辑**与注释/调用方语义相反**——当 `< 1e-2`（"解质量不好"）时却 `return true`，质量好时 `return false`。而调用方 `BuildInitMap`/`TriangulateNewPoints` 写的是 `if (triangulation(...) && pworld[2]>0)` 即"返回 true 才接受"。这是 slambook2 仓库中一个**长期存在的 bug**（多个 issue/fork 指出）。**正确逻辑应为**：质量好 `return true`、质量差 `return false`。综合成书时建议给出修正版（把两个 `return` 对调），并在 pitfall 框中点明此坑。十四讲正文未印此函数，故书面未体现该 bug。

### 5.9 关键帧插入 `InsertKeyframe` + 新点三角化 `TriangulateNewPoints` + 设观测 `SetObservationsForKeyFrame`（S2 全量补全）

> 来源：`slambook2/ch13/src/frontend.cpp`

```cpp
bool Frontend::InsertKeyframe() {
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}
```

**解读（关键帧选取准则 + 关键帧专属工作）**：
- **关键帧选取准则（本章的极简准则）**：`tracking_inliers_ < num_features_needed_for_keyframe_`(=80) → 设为关键帧。即"**当跟踪到的内点数掉到阈值以下，说明视野变化大、地图点不够用了**"，此时插关键帧、补点。否则不插（返回 false）。
- 关键帧专属四步（呼应 §5.1 逻辑4）：① `SetObservationsForKeyFrame`——把当前帧已关联地图点的左目特征**正式登记为该地图点的观测**（这一步只对关键帧做，普通追踪帧不增加地图点的观测计数，控制 `observed_times_` 的语义为"被关键帧观测"）；② `DetectFeatures`——在空白区补提新特征；③ `FindFeaturesInRight`——新特征右目光流匹配；④ `TriangulateNewPoints`——新匹配三角化出新地图点。最后 `backend_->UpdateMap()` 触发后端 + `viewer_->UpdateMap()`。
- **`TriangulateNewPoints` 与 `BuildInitMap` 的差异**：初始化时当前帧位姿为 $I$（世界系=首帧相机系），三角化结果直接是世界坐标；而关键帧追踪阶段当前帧位姿 $T_{cw}\ne I$，三角化得到的是**相机系**坐标，需 `pworld = current_pose_Twc * pworld`（$T_{wc}$ 左乘）转到世界系。这是两处看似重复实则关键的区别。
- `map_point_.expired()`：weak_ptr 检查地图点是否已失效（只对"尚无有效地图点"的左特征三角化新点）。

### 5.10 `Reset`（追踪丢失处理，S2）

```cpp
bool Frontend::Reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}
```

> S1 §13.3.2 逻辑5 说"追踪丢失就重置前端、重新初始化"，但 S2 仓库的 `Reset()` **实际是空实现**（仅打日志）。综合成书时应点明：这是精简 VO 的简化/未完成处，工业系统在 LOST 时需触发**重定位**（relocalization，ORB-SLAM2 用词袋在全局关键帧库找候选 + PnP+RANSAC 恢复位姿，见 §9.7）。

---

## 6. g2o 顶点与边类型（含雅可比解析式）（S2 `g2o_types.h`，正文未印，全量补全）

S1 §13.3.3 的后端 `Optimize` 用了 `VertexPose`/`VertexXYZ`/`EdgeProjection`，前端 `EstimateCurrentPose` 用了 `VertexPose`/`EdgeProjectionPoseOnly`，但**正文未印这些类型的定义**——而它们包含本章最核心的**重投影误差雅可比解析式**。据 S2 全量补全。

> 来源：`slambook2/ch13/include/myslam/g2o_types.h`

```cpp
#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace myslam {

/// vertex and edges used in g2o ba
/// 位姿顶点
class VertexPose : public g2o::BaseVertex<6, SE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override { _estimate = SE3(); }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        Vec6 update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4],
            update[5];
        _estimate = SE3::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }
};

/// 路标顶点
class VertexXYZ : public g2o::BaseVertex<3, Vec3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override { _estimate = Vec3::Zero(); }

    virtual void oplusImpl(const double *update) override {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
        _estimate[2] += update[2];
    }

    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }
};

/// 仅估计位姿的一元边
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K)
        : _pos3d(pos), _K(K) {}

    virtual void computeError() override {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Vec3 pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Vec3 pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;
    }

    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }

   private:
    Vec3 _pos3d;
    Mat33 _K;
};

/// 带有地图和位姿的二元边
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// 构造时传入相机内外参
    EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : _K(K) {
        _cam_ext = cam_ext;
    }

    virtual void computeError() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();
        Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();
        Vec3 pw = v1->estimate();
        Vec3 pos_cam = _cam_ext * T * pw;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;

        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                           _cam_ext.rotationMatrix() * T.rotationMatrix();
    }

    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }

   private:
    Mat33 _K;
    SE3 _cam_ext;
};

}  // namespace myslam

#endif  // MYSLAM_G2O_TYPES_H
```

### 6.1 位姿顶点更新（左扰动）

`VertexPose::oplusImpl`：$T\leftarrow\exp(\delta\xi^\wedge)\,T$，**左乘 / 左扰动**，$\delta\xi=[\rho;\phi]\in\mathbb R^6$（前 3 平移、后 3 旋转）。
> **本书右扰动转换提示**：本书以右扰动 $T\leftarrow T\,\exp(\delta\xi^\wedge)$ 为主。若改右扰动，§6.3 的雅可比"旋转 3 列"需相应改变（差一个 $\mathrm{Ad}$ 项 / 符号）。综合时择一约定并全章统一。

### 6.2 路标顶点更新（向量空间加法）

`VertexXYZ::oplusImpl`：$\boldsymbol p\leftarrow\boldsymbol p+\delta\boldsymbol p$，普通三维加法（路标在 $\mathbb R^3$，无流形约束）。

### 6.3 重投影误差对位姿的雅可比（pose-only / 二元边共用，左扰动，$2\times6$）

记相机系点 $\boldsymbol p_c=T\boldsymbol p=[X,Y,Z]^\top$（pose-only 边 $\boldsymbol p$ 为地图点；二元边 $\boldsymbol p_c=R_{\text{ext}}T\boldsymbol p_w+\dots$ 含左右目外参 `_cam_ext`）。误差 $\boldsymbol e=\boldsymbol z-\pi(K\boldsymbol p_c)$。代码 `_jacobianOplusXi` 按行展开即
$$
\frac{\partial \boldsymbol e}{\partial \delta\boldsymbol\xi}=
-\begin{bmatrix}
\dfrac{f_x}{Z} & 0 & -\dfrac{f_x X}{Z^2} & -\dfrac{f_x XY}{Z^2} & f_x+\dfrac{f_x X^2}{Z^2} & -\dfrac{f_x Y}{Z}\\[2.2ex]
0 & \dfrac{f_y}{Z} & -\dfrac{f_y Y}{Z^2} & -f_y-\dfrac{f_y Y^2}{Z^2} & \dfrac{f_y XY}{Z^2} & \dfrac{f_y X}{Z}
\end{bmatrix}.
$$
> ⚠️ 代码里 `_jacobianOplusXi` 存的是 **$\partial(\pi)/\partial\delta\xi$ 取负之后**的具体数值：第一行 `-fx*Zinv, 0, fx*X*Zinv2, fx*X*Y*Zinv2, -fx-fx*X*X*Zinv2, fx*Y*Zinv`。因 $\boldsymbol e=\boldsymbol z-\pi$，$\partial\boldsymbol e/\partial\delta\xi=-\partial\pi/\partial\delta\xi$；代码把符号已并入。逐项对照上式（注意整体的负号在矩阵外）：前 3 列（平移）= $-[f_x/Z,0,-f_xX/Z^2;\ 0,f_y/Z,-f_yY/Z^2]$，与代码 `-fx*Zinv,0,fx*X*Zinv2` 一致；后 3 列（旋转）同理。
> **推导链**（S1b 第 9 讲式 (9.x)）：$\dfrac{\partial\boldsymbol e}{\partial\delta\xi}=-\dfrac{\partial\pi}{\partial\boldsymbol p_c}\dfrac{\partial(T\boldsymbol p)}{\partial\delta\xi}$，其中 $\dfrac{\partial\pi}{\partial\boldsymbol p_c}=\begin{bmatrix}f_x/Z&0&-f_xX/Z^2\\0&f_y/Z&-f_yY/Z^2\end{bmatrix}$，左扰动下 $\dfrac{\partial(T\boldsymbol p)}{\partial\delta\xi}=[\,I\ \ -\boldsymbol p_c^\wedge\,]$（$\xi=[\rho;\phi]$ 排序）。两者相乘即得上式。

### 6.4 重投影误差对路标的雅可比（二元边，$2\times3$）

`_jacobianOplusXj = _jacobianOplusXi.block<2,3>(0,0) * _cam_ext.rotationMatrix() * T.rotationMatrix()`，即
$$
\frac{\partial \boldsymbol e}{\partial \boldsymbol p_w}
=\underbrace{-\begin{bmatrix}f_x/Z&0&-f_xX/Z^2\\0&f_y/Z&-f_yY/Z^2\end{bmatrix}}_{\partial\boldsymbol e/\partial\boldsymbol p_c\ (\text{= Xi 的前 3 列})}\;R_{\text{ext}}\;R,
$$
其中 $R_{\text{ext}}=$ `_cam_ext.rotationMatrix()`（左/右目外参旋转），$R=$ `T.rotationMatrix()`（当前帧旋转）。链式：$\boldsymbol p_c=R_{\text{ext}}(R\boldsymbol p_w+\boldsymbol t)+\boldsymbol t_{\text{ext}}\Rightarrow \partial\boldsymbol p_c/\partial\boldsymbol p_w=R_{\text{ext}}R$。

> **二元边为何带 `_cam_ext`**：后端把左右目都当作"同一帧位姿 $T_{cw}$ + 固定相机外参 $T_{\text{ext}}$"的观测——左目边用 `left_ext`、右目边用 `right_ext`（见 §7.1）。这样一个 `VertexPose` 同时被左右目重投影约束，**双目基线信息自然进入 BA**。

---

## 7. 后端：局部 BA / 滑窗优化（S1 §13.3.3 + S2 `backend.cpp/.h` 全量）

### 7.1 后端类声明（S1 正文印出主体 + S2 头）

> 来源：`slambook2/ch13/include/myslam/backend.h`

```cpp
#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
class Map;

/**
 * 后端
 * 有单独优化线程，在 Map 更新时启动优化
 * Map 更新由前端触发
 */
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// 构造函数中启动优化线程并挂起
    Backend();

    // 设置左右目的相机，用于获得内外参
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        cam_left_ = left;
        cam_right_ = right;
    }

    /// 设置地图
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// 触发地图更新，启动优化
    void UpdateMap();

    /// 关闭后端线程
    void Stop();

   private:
    /// 后端线程
    void BackendLoop();

    /// 对给定关键帧和路标点进行优化
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H
```

**S1 解读**：后端在启动之后等待 `map_update_` 条件变量；地图更新触发时，从地图拿激活的关键帧和地图点执行优化。多线程同步三件套：`std::thread backend_thread_`（线程）、`std::mutex data_mutex_`（互斥）、`std::condition_variable map_update_`（条件变量）、`std::atomic<bool> backend_running_`（原子标志）。

### 7.2 后端线程：构造 / 触发 / 停止 / 循环（S1 印出 `BackendLoop` 主体 + S2 其余）

> 来源：`slambook2/ch13/src/backend.cpp`

```cpp
Backend::Backend() {
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop() {
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop() {
    while (backend_running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        /// 后端仅优化激活的 Frames 和 Landmarks
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}
```

**解读（多线程并发模型，工程要点）**：
- **构造即起线程**：`Backend()` 把 `backend_running_` 置 true，启动 `BackendLoop` 线程（随即在 `map_update_.wait()` 处挂起，不空转 CPU）。
- **生产者-消费者**：前端是生产者，每插一个关键帧调 `UpdateMap()` → `notify_one()` 唤醒后端；后端是消费者，醒来后取激活集做一次 BA。
- **优雅退出**：`Stop()` 置 `backend_running_=false` → `notify_one()` 唤醒后端让 `while` 条件失败退出 → `join()` 等线程结束。
- **只优化激活集**：`GetActiveKeyFrames()/GetActiveMapPoints()`（最近 7 帧 + 其点）→ 这就是"滑窗"在工程上的体现：后端永远只看最近 N 帧，**BA 规模有界、可实时**。

### 7.3 局部 BA：`Optimize`（S1 §13.3.3 印出主体 + S2 完整，含外点自适应）

> 来源：`slambook2/ch13/src/backend.cpp`

```cpp
void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks) {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // pose 顶点，使用 Keyframe id
    std::map<unsigned long, VertexPose *> vertices;
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});
    }

    // 路标顶点，使用路标 id 索引
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    // K 和左右外参
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel 阈值
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;

            auto frame = feat->frame_.lock();
            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image_) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // 如果 landmark 还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            if (vertices.find(frame->keyframe_id_) != vertices.end() &&
                vertices_landmarks.find(landmark_id) !=
                    vertices_landmarks.end()) {
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            } else
                delete edge;
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5) {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;
            // remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
              << cnt_inlier;

    // Set pose and lanrmark position
    for (auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}
```

> ⚠️ **S1 正文 vs S2 差异（OCR 修正）**：S1 正文把"加边"那段印成无条件 `edge->setId(index); ... optimizer.addEdge(edge);`，而 S2 仓库外面包了一层 `if (vertices.find(...) && vertices_landmarks.find(...))`（确认 pose 顶点和 landmark 顶点都已在图中才加边，否则 `delete edge`）。这是仓库后来修的健壮性补丁（避免悬挂边）。本抽取以 **S2 仓库版**为准。另外正文头文件代码有几处明显 OCR 残缺（`std::shared_ptr<Map_;`、`std::map<unsigned long, VertexPose =>`、`VertexXYZ =>`、`std::map<...>` 应为 `std::map<unsigned long, VertexPose *>`），均按 S2 修正。

**解读（局部 BA = 双目滑窗 BA，本章后端核心）**：
- **求解器**：`BlockSolver_6_3`（位姿 6、路标 3），线性求解器 **`LinearSolverCSparse`**（稀疏，因为后端是多帧多点的大稀疏问题；对比前端用 Dense）。LM。
- **顶点**：每个激活关键帧建一个 `VertexPose`（id = `keyframe_id_`）；每个被观测到的激活路标建一个 `VertexXYZ`（id = `landmark_id + max_kf_id + 1`，避免与位姿 id 冲突），并 **`setMarginalized(true)`**——告诉 g2o 这是要 Schur 边缘化掉的路标块（呼应 S1b §9.2.3 Schur 消元；g2o 必须手动标 marginalized，否则报错）。
- **边**：遍历每个激活路标的所有观测，按 `is_on_left_image_` 选左目或右目外参建 `EdgeProjection`，连接对应 `VertexPose` 与 `VertexXYZ`，测量 = 像素，信息矩阵 $I_{2\times2}$，Huber 核 `setDelta(5.991)`。
- **外点阈值自适应**（先优化一次，再调阈值）：先 `optimize(10)`；然后 while(≤5 次)统计 `chi2()>chi2_th` 的外点比例，若内点率 `inlier_ratio>0.5` 就停，否则 `chi2_th *= 2` 放宽（在外点很多时避免把大半观测都判成外点）。
- **剔外点 + 写回**：最终按阈值标记 `is_outlier_`，对外点调 `RemoveObservation` 断开地图点观测；把优化后的位姿/路标写回 Frame/MapPoint（加锁的 `SetPose`/`SetPos`，保证与前端线程安全交互）。

### 7.4 局部 BA 的数学（依赖 S1b §9.2 + S5 §7.4.1，本章后端理论支撑）

**目标函数**（多关键帧、多路标的重投影误差最小化）：
$$
\{T_k^\star,\boldsymbol p_j^\star\}=\arg\min_{T_k,\boldsymbol p_j}\ \frac12\sum_{(k,j)\in\mathcal O}\rho\Big(\big\|\boldsymbol z_{kj}-\pi(K\,T_{\text{ext}}\,T_k\,\boldsymbol p_j)\big\|_{\Sigma_v^{-1}}^2\Big),
$$
$\mathcal O$ = 激活窗口内所有(关键帧, 路标)观测对，$T_{\text{ext}}\in\{$left_ext, right_ext$\}$，$\rho=$ Huber，$\Sigma_v=I$。

**Handbook 局部 BA 公式（S5 式 7.5，K1 优化 / K2 固定）**：
$$
\big\{\boldsymbol T_w^{k*},\boldsymbol x_j^{w*}\mid k\in\mathsf K_1,\ j\in\mathsf P_1\big\}
=\arg\min_{\boldsymbol T_w^k,\boldsymbol x_j^w}\frac12\!\!\sum_{\substack{i\in\mathsf K_1\cup\mathsf K_2\\ j\in\mathsf P_1}}\!\!
\rho\Big(\big\|\boldsymbol z_{ij}-\pi(\boldsymbol R_w^i\boldsymbol x_j^w+\boldsymbol t_w^i)\big\|_{\Sigma_{ij}}\Big).
$$
> 注意 ORB-SLAM 区分 $\mathsf K_1$（优化的共视关键帧）与 $\mathsf K_2$（**只提供约束、保持固定**的关键帧）。**十四讲 ch13 的简化**：激活窗口内**所有**关键帧都被优化（没有 $\mathsf K_2$ 固定帧），这会让窗口边缘的帧缺少"锚定"，是精简实现与 ORB-SLAM2 的一处关键差异，综合应点明。

**Schur 边缘化 / 稀疏求解三步式（S5 式 7.7 + S1b §9.2.3）**：BA 每次迭代解
$$
\begin{bmatrix}\boldsymbol H_{cc}&\boldsymbol H_{cp}\\ \boldsymbol H_{cp}^\top&\boldsymbol H_{pp}\end{bmatrix}\!\begin{bmatrix}\boldsymbol d_c\\ \boldsymbol d_p\end{bmatrix}=\begin{bmatrix}\boldsymbol b_c\\ \boldsymbol b_p\end{bmatrix},
$$
$c$=相机块、$p$=路标块。利用 $\boldsymbol H_{pp}$ 块对角，先消路标得**约化相机系统**：
$$
\boldsymbol H_{cc}^{\text{red}}=\boldsymbol H_{cc}-\boldsymbol H_{cp}\boldsymbol H_{pp}^{-1}\boldsymbol H_{cp}^\top,\qquad
\boldsymbol H_{cc}^{\text{red}}\boldsymbol d_c=\boldsymbol b_c-\boldsymbol H_{cp}\boldsymbol H_{pp}^{-1}\boldsymbol b_p,\qquad
\boldsymbol H_{pp}\boldsymbol d_p=\boldsymbol b_p-\boldsymbol H_{cp}^\top\boldsymbol d_c.
$$
约化相机系统的非零块 ⟺ 两相机有共同观测路标（**共视**）。局部 BA 因强共视，约化相机系统几乎稠密 → 可用稠密解；全局 BA 关键帧多但共视稀疏 → 用稀疏解（S5 §7.4.3）。这正是后端 `setMarginalized(true)` 让 g2o 自动做的事。

**鲁棒核（Huber，S1b §9.2.4 式）**：
$$
\rho_{\text{Huber}}(s)=\begin{cases}\tfrac12 s^2,& |s|\le\delta,\\ \delta\big(|s|-\tfrac12\delta\big),& |s|>\delta,\end{cases}
$$
误差超过阈值 $\delta$ 后由二次增长变为一次增长，限制单条大残差（误匹配/外点）对总目标的支配，且处处光滑可导。本章 $\delta=$ `chi2_th`。

**为什么阈值取 5.991**（自由度 2 的卡方临界值，本章 `chi2_th=5.991` 的来历）：单个像素重投影误差在高斯噪声 $\Sigma_v$ 下，其马氏距离平方 $\boldsymbol e^\top\Sigma_v^{-1}\boldsymbol e$ 服从自由度 $=2$（误差是 2 维像素）的卡方分布 $\chi^2_2$。取显著性水平 $\alpha=0.05$，临界值 $\chi^2_{2,0.05}=5.991$——即"一个真正的内点其 chi2 超过 5.991 的概率只有 5%"，故以此判外点既是 Huber 的 $\delta$、又是外点门限。

### 7.5 滑动窗口与边缘化的理论（S1b §10.1，本章"激活窗口"的理论根基）

本章"激活最近 N=7 帧、删旧帧"在理论上对应**滑动窗口法**。S1b 第 10 讲给出完整理论，本抽取全量记录（成书用于解释"为什么能删 / 删了之后信息去哪了"）。

**滑窗状态的高斯描述（S1b §10.1.2 式 10.1）**：窗口内 N 个关键帧位姿（李代数表达），其联合分布
$$
\begin{bmatrix}\boldsymbol\xi_1\\\vdots\\\boldsymbol\xi_N\end{bmatrix}\sim\mathcal N\!\left(\begin{bmatrix}\boldsymbol\mu_1\\\vdots\\\boldsymbol\mu_N\end{bmatrix},\ \boldsymbol\Sigma\right),
$$
$\boldsymbol\mu_k$ = 第 $k$ 帧位姿均值（BA 迭代结果），$\boldsymbol\Sigma$ = 对整个 BA 的 $H$ 矩阵边缘化所有路标后的结果（即第 9 讲的 $S$ 矩阵）。

**窗口结构变化两件事（S1b §10.1.2）**：① 新增一个关键帧及其观测路标——平凡，按正常 BA 流程处理，边缘化所有点即得 N+1 帧高斯参数；② 删除一个旧关键帧——产生理论问题。

**删旧帧导致的 fill-in（S1b §10.1.2，图 10-2）**：设旧帧 $x_1$ 看到路标 $y_1\!\sim\!y_4$。边缘化 $x_1$ 的 Schur 消元会把 $x_1$ 行/列的非零块消去，**导致右下角路标块不再块对角**——称为**边缘化中的填入（Fill-in）**。回顾第 9 讲：边缘化路标时 fill-in 出现在位姿块（BA 不要求位姿块对角，稀疏求解仍可行）；但**边缘化关键帧会破坏路标块的对角结构**，使稀疏 BA 无法继续。早期 EKF 后端正因保持稠密 H 而无法处理大滑窗。

**保持稀疏的边缘化改造（S1b §10.1.2）**：边缘化某旧关键帧时**同时边缘化它观测到的路标点**——路标信息转换成"剩下那些关键帧之间的共视信息"，从而保持右下角对角块结构。更复杂的策略（OKVIS）：判断要边缘化的关键帧所看路标是否在最新关键帧中仍可见——不可见就直接边缘化该路标；可见就**丢弃被边缘化关键帧对该路标的观测**，从而保持稀疏。

**边缘化的概率直观（S1b §10.1.2）**：边缘化某关键帧 = "保持其当前估计值，求其他状态以它为条件的条件概率"。被边缘化的关键帧会给它观测的路标产生"这些路标应在哪里"的先验；再边缘化这些路标，又给它们的观测者产生"观测它们的关键帧应在哪里"的先验。数学上，整个窗口状态描述从联合分布变为条件分布
$$
p(x_1,\dots,x_N,\text{landmarks})=p(x_1\mid\cdots)\,p(\text{其余}\mid\cdots),
$$
然后舍去被边缘化部分的信息。**变量被边缘化后，工程中不应再使用它**——故滑动窗口法适合 VO，不适合大规模建图。

> S1b §10.1.2 末注：g2o 和 Ceres 当时还未直接支持滑窗边缘化操作，故第 10 讲略去对应实验。**十四讲 ch13 的"激活窗口"是一种工程近似的滑窗**：它不做严格边缘化，而是**直接丢弃**离开窗口的帧（连先验信息一并丢掉），换取实现简单（这也是其精度不如严格滑窗 / ORB-SLAM2 共视 BA 的原因之一）。

---

## 8. 周边模块（S1 正文交给读者，S2 全量补全）

### 8.1 相机类 Camera（S2 `camera.h` + `camera.cpp`）

> 来源：`slambook2/ch13/include/myslam/camera.h`

```cpp
#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

// Pinhole stereo camera model
class Camera {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0;  // Camera intrinsics
    SE3 pose_;             // extrinsic, from stereo camera to single camera
    SE3 pose_inv_;         // inverse of extrinsics

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }

    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);
    Vec2 camera2pixel(const Vec3 &p_c);
    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);
    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
};

}  // namespace myslam
#endif  // MYSLAM_CAMERA_H
```

> 坐标变换实现在 `camera.cpp`（WebFetch 抓取头文件时实现未含其中，据 slambook2 标准实现与上面声明 + Kitti 用法还原；公式与 §5 调用一致，**逐式给出供成书**）：

```cpp
// slambook2/ch13/src/camera.cpp  （标准实现，依据声明与调用语义还原）
namespace myslam {

Camera::Camera() {}

Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w) {
    return pose_ * T_c_w * p_w;
}

Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    return Vec2(
        fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
        fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) {
    return Vec3(
        (p_p(0, 0) - cx_) * depth / fx_,
        (p_p(1, 0) - cy_) * depth / fy_,
        depth);
}

Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

}  // namespace myslam
```

**解读（双目外参的处理是本系统的巧思）**：
- `pose_` = "**from stereo camera to single camera**"，即从"双目参考系（=左目光心）"到"该单目"的外参 $T_{\text{ext}}$。左目相机的 `pose_` 通常为 $I$（参考系即左目），右目相机的 `pose_` 含一段沿基线的平移（见 §8.2 `Dataset::Init` 把投影矩阵的平移转成外参 `t`）。
- `world2camera(p_w, T_cw) = pose_ * T_cw * p_w`：先用帧位姿 $T_{cw}$ 把世界点转到"双目参考系"，再用 `pose_`（$T_{\text{ext}}$）转到该具体单目系。**这就是为什么后端 `EdgeProjection` 要传 `left_ext`/`right_ext`**——同一个 `VertexPose`（$T_{cw}$）经左/右外参分别投影到左/右图。
- 投影/反投影公式即针孔模型：$u=f_x X/Z+c_x,\ v=f_y Y/Z+c_y$；反投影 $X=(u-c_x)Z/f_x$ 等。

### 8.2 数据集类 Dataset（S2 `dataset.cpp`，Kitti 读取）

> 来源：`slambook2/ch13/src/dataset.cpp`

```cpp
bool Dataset::Init() {
    // read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");
    if (!fin) {
        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }
    fin.close();
    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;
    // read images
    image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);
    image_right =
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                   cv::IMREAD_GRAYSCALE);

    if (image_left.data == nullptr || image_right.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resized;
    new_frame->right_img_ = image_right_resized;
    current_image_index_++;
    return new_frame;
}
```

**解读（Kitti 标定与外参恢复，工程细节）**：
- Kitti `calib.txt` 每行是一个 $3\times4$ **投影矩阵** $P=K[R|t']$（Kitti 已做立体校正，$R=I$，$P=[K\,|\,K\boldsymbol t]$，其中 $\boldsymbol t$ 是该相机相对参考相机的平移、已乘进 $K$）。代码读 12 个数，前 9 个（按 0,1,2,4,5,6,8,9,10）填 $K$，第 3、7、11 个填 $\boldsymbol t'=K\boldsymbol t$。
- **恢复真实外参平移**：`t = K.inverse() * t`（把 $K\boldsymbol t$ 还原成 $\boldsymbol t$）。基线 = `t.norm()`。
- **内参减半** `K = K * 0.5`：因为 `NextFrame` 把图像 `resize 0.5`（降采样提速），内参须同步乘 0.5（$f_x,f_y,c_x,c_y$ 都按比例缩）。
- 构造 `Camera(fx,fy,cx,cy, baseline=t.norm(), pose=SE3(SO3(),t))`：外参旋转为 $I$、平移为 $\boldsymbol t$。
- Kitti 有 4 个相机（0/1 灰度左右，2/3 彩色左右），本系统用 `GetCamera(0)`/`(1)`（灰度左右目）。
- `NextFrame`：按 `%s/image_%d/%06d.png` 格式（`boost::format`）读左(image_0)右(image_1)灰度图、各 resize 0.5、建 Frame、索引自增；读不到图返回 nullptr（→ `Step` 返回 false → 主循环结束）。

### 8.3 配置类 Config / 可视化类 Viewer（S2，概述）

- **Config（`config.cpp`，单例模式）**：S1 习题1 点名"设计模式中的单例模式"。`Config::SetParameterFile(path)` 用 `cv::FileStorage` 打开 yaml；`Config::Get<T>(key)` 模板读取参数（如 `num_features`、`dataset_dir`）。单例保证全局唯一配置对象。配置文件 `config/default.yaml` 含 `dataset_dir`、`num_features`、`num_features_init` 等。
- **Viewer（`viewer.cpp`，独立线程 + Pangolin）**：`AddCurrentFrame`/`UpdateMap` 由前端调用（线程安全），内部用 Pangolin 画当前帧位姿、激活关键帧轨迹、激活地图点云、当前帧左图（带特征点）。`Close()` 由 `VisualOdometry::Run` 结束时调用。S1 §13.4 图 13-4 即 Viewer 截图。
> 这两个模块不含核心算法，S1 明确"交给读者自行阅读"，本抽取据 S2 给出职责说明，综合 agent 可只作"工程脚手架"一笔带过或附录列出。

---

## 9. 与 ORB-SLAM2 架构对照（S3 + S4 + S5，全量要素）

> 本节服务"与 ORB-SLAM2 架构对照"，把十四讲精简 VO 缺失的工业级要素逐条补全。综合 agent 应做成"对照表 + 各子系统详解 + 架构图"。

### 9.1 三线程（+ 可选第四线程）总体架构（S3 §III / S5 §7.4.1）

ORB-SLAM2 = 三个并行线程 + 一个可选线程（S5 图 7.7）：

1. **Tracking（跟踪，帧率运行 10–50 Hz）**：对每帧预处理（双目/RGBD 输入抽象成统一关键点）；与上一帧/局部地图做 ORB 特征匹配；用 **motion-only BA（pose-only）** 最小化重投影误差定位相机（S5 式 7.4，**不更新地图点**）；并决定是否生成关键帧。
$$
\boldsymbol T_w^{i*}=\arg\min_{\boldsymbol T_w^i}\sum_{j\in\mathsf P}\rho\Big(\big\|\boldsymbol z_{ij}-\pi(\boldsymbol R_w^i\boldsymbol x_j^w+\boldsymbol t_w^i)\big\|_{\Sigma_{ij}}\Big).\tag{7.4}
$$
2. **Local Mapping（局部建图，关键帧率 0.5–5 Hz）**：管理并优化局部地图，执行 **local BA**（S5 式 7.5）；插入新关键帧、剔除冗余关键帧、剔除/创建地图点。
3. **Loop Closing（回环，每个关键帧尝试一次）**：用 DBoW2 检测回环候选；算 Sim(3)（单目）/SE(3)（双目-RGBD）相对变换；回环融合 + **本质图位姿图优化**纠正累积漂移。
4. **Full BA（可选第四线程）**：回环后启动，对全部关键帧和地图点做全局 BA 求最优结构与运动（与 Loop Closing 异步，可被新回环中断）。

> **十四讲 ch13 对应**：只有 Tracking（前端）+ Local Mapping（后端局部 BA）两个角色，**无 Loop Closing、无 Full BA、无重定位**。

### 9.2 双目 / RGB-D 关键点表示（S3 §III-A）

- **双目关键点用三坐标** $\mathbf x_s=(u_L,v_L,u_R)$：$(u_L,v_L)$ 是左图坐标，$u_R$ 是右图横坐标（左右目已校正、行对齐，故右图只差一个横坐标）。
- **RGB-D 合成虚拟右坐标**：把深度 $d$ 转成
$$
u_R=u_L-\frac{f_x\,b}{d},
$$
$f_x$ 为焦距、$b$ 为基线（RGBD 用一个"虚拟基线"）。**于是 RGBD 与双目在后端用同一套三坐标关键点处理**——这是 ORB-SLAM2 统一双目/RGBD 的关键设计。
- **近点 vs 远点**（关键工程区分）：深度 **< 40×基线**（`40·b`）的关键点为**近点（close）**——可由单帧立体直接三角化、深度可靠，用作完整的双目约束；深度 ≥ 40·b 的为**远点（far）**——视差太小、深度不可靠，需**多视角三角化**后才升级为完整地图点（在此之前只当作单目观测约束方向）。
- **单目关键点** $\mathbf x_m=(u_L,v_L)$：左图中未找到立体匹配、或远点（RGBD 中深度无效），只提供 2D 方向约束。

### 9.3 地图结构：共视图 / 生成树 / 本质图（S4 §III-D + S5 图 7.5）

- **共视图（Covisibility Graph）**：无向加权图，节点=关键帧，**两关键帧若共同观测到至少 $\theta$ 个地图点则连边**，权重 = 共视地图点数。用于 **local BA** 的局部窗口选取（S5 例 θ=15）。
- **生成树（Spanning Tree）**：共视图的一棵最小连通生成树，每个关键帧只连一条到"共视最强的父关键帧"的边——保证全图连通、边数最少，用于高效的全局结构维护。
- **本质图（Essential Graph）**：比共视图**更稀疏**的图，= **生成树 + 共视权重 ≥ θ=100 的强共视边 + 回环边**。用于**回环时的位姿图优化（PGO）**（S5：本质图连接共视≥100 的关键帧，"used for pose-graph optimization during loop correction"）。
> S5 图 7.5 原文："covisibility graph (left) connects keyframes that have seen at least θ points in common (θ=15) and is used for local BA. The essential graph (right) ... connects keyframes with at least θ=100 points in common, and is used for pose-graph optimization during loop correction."

### 9.4 关键帧选取准则（ORB-SLAM2 显式四条件，S3 §V-D / S4 §V-E）

跟踪线程插入新关键帧需**同时满足**（与十四讲"内点掉到 80 以下"形成对比）：
1. 距上次全局重定位已过去 > 20 帧；
2. 局部建图线程空闲，或距上次插关键帧已过去 > 20 帧；
3. 当前帧跟踪到的地图点 ≥ 50 个；
4. 当前帧跟踪到的点与其参考关键帧的共视点相比 < 90%（即包含足够新信息）。

**双目/RGBD 额外条件（S3）**：当跟踪到的**近点**数 < $\tau_t=100$ 且当前帧能新建至少 $\tau_c=70$ 个新的近点立体观测时，也插关键帧（保证近距离结构持续被建图）。
> ORB-SLAM 的策略是**"宽进严出"**：尽量快插关键帧（提升跟踪鲁棒性），再由局部建图线程的**关键帧剔除**删冗余（见 §9.6）。

### 9.5 局部 BA：K1 优化 / K2 固定（S3 §VI-B / S5 图 7.6 式 7.5）

- 局部地图 $\mathsf K_1$ = 当前关键帧 $k$ **及其共视图邻居**；$\mathsf P_1$ = 被 $\mathsf K_1$ 观测的所有地图点。
- $\mathsf K_2$ = 其余"也观测到 $\mathsf P_1$ 中某些点、但不在 $\mathsf K_1$ 里"的关键帧——**它们参与构成约束但位姿保持固定**（提供"锚"，防止局部漂移）。
- 优化变量 = $\{\mathsf K_1$ 的位姿$\}\cup\{\mathsf P_1$ 的点$\}$；目标即式 (7.5)。
> **与十四讲 ch13 的核心差异**：ch13 激活窗口内**所有 7 帧全优化、无固定帧**；ORB-SLAM2 有 $\mathsf K_2$ 固定帧锚定，且窗口按**共视**而非纯时序选取。

### 9.6 关键帧剔除（90% 冗余规则，S3 §VI-E / S4 §V-E）

局部建图线程剔除**冗余关键帧**：若某关键帧的地图点中有 **≥ 90%** 能被**其他至少 3 个关键帧**在相同或更精细尺度上观测到，则删除该关键帧。这控制了关键帧数量随时间无界增长（呼应十四讲后端"控制规模"的诉求，但 ORB-SLAM2 是按信息冗余删、ch13 是按时序删最旧/最近）。

### 9.7 地图点剔除与创建（S3 §VI-A,C / S4 §V-B,D）

- **地图点剔除**：新建地图点须通过严格测试——创建后头 3 个关键帧内，跟踪线程**实际找到它的帧数 / 预测可见帧数 ≥ 25%**；且建后至少被 3 个关键帧观测。否则删除（剔除虚假三角化点）。
- **地图点创建**：局部建图对当前关键帧与共视邻居中**未匹配的 ORB** 做三角化（双目用立体，单目用极线搜索），通过视差/重投影/尺度一致性/正深度检验后加入地图。

### 9.8 回环（Loop Closing）四步（S3 §VII / S4 §VI）—— 十四讲 ch13 的习题3

1. **回环候选检测**：用 **DBoW2 词袋**对当前关键帧在数据库检索，计算与共视邻居的最低相似度作阈值，找相似度高且**不在当前共视图内**的候选；要求候选在**连续 3 个共视关键帧**上一致（时间一致性）。
2. **计算 Sim(3) / SE(3)**：对候选与当前关键帧做 ORB 匹配 → RANSAC 求**相似变换 Sim(3)**（单目，含尺度，纠正单目尺度漂移）/ **SE(3)**（双目-RGBD，尺度可观无需 Sim(3)）；内点足够则接受。
3. **回环融合（Loop Fusion）**：用算出的 Sim(3) 把当前关键帧及其邻居的位姿、地图点对齐到回环侧；融合重复地图点；在共视图中插入新的回环边。
4. **本质图优化（Essential Graph PGO）**：在**本质图**上做位姿图优化（用 Sim(3) 约束，单目；SE(3)，双目），把回环误差沿全图分摊、纠正累积漂移。之后可选地启动 **Full BA** 精修。

**位姿图优化的数学（S1b §10.2，本书自有推导，供成书替代 ORB-SLAM 的 Sim3 PGO 叙述）**：节点为位姿 $T_1,\dots,T_n$，边为相对运动观测 $\Delta T_{ij}$。误差
$$
\boldsymbol e_{ij}=\ln\!\big(\boldsymbol T_{ij}^{-1}\boldsymbol T_i^{-1}\boldsymbol T_j\big)^\vee,\qquad \boldsymbol T_{ij}=\boldsymbol T_i^{-1}\boldsymbol T_j.\tag{10.4}
$$
对 $T_i,T_j$ 各加左扰动，利用伴随性质 $\exp(\boldsymbol\xi^\wedge)\boldsymbol T=\boldsymbol T\exp\!\big((\mathrm{Ad}(\boldsymbol T^{-1})\boldsymbol\xi)^\wedge\big)$（式 10.7）把扰动挪到一侧，得雅可比
$$
\frac{\partial\boldsymbol e_{ij}}{\partial\delta\boldsymbol\xi_i}=-\boldsymbol{\mathcal J}_r^{-1}(\boldsymbol e_{ij})\,\mathrm{Ad}(\boldsymbol T_j^{-1}),\qquad
\frac{\partial\boldsymbol e_{ij}}{\partial\delta\boldsymbol\xi_j}=\boldsymbol{\mathcal J}_r^{-1}(\boldsymbol e_{ij})\,\mathrm{Ad}(\boldsymbol T_j^{-1}),\tag{10.9,10.10}
$$
其中 $\mathfrak{se}(3)$ 右雅可比逆近似
$$
\boldsymbol{\mathcal J}_r^{-1}(\boldsymbol e_{ij})\approx\boldsymbol I+\frac12\begin{bmatrix}\boldsymbol\phi_e^\wedge&\boldsymbol\rho_e^\wedge\\ \boldsymbol 0&\boldsymbol\phi_e^\wedge\end{bmatrix}\quad(\text{误差小时取 }\boldsymbol I).\tag{10.11}
$$
总目标（$\mathcal E$ 为所有边）
$$
\min\ \frac12\sum_{(i,j)\in\mathcal E}\boldsymbol e_{ij}^\top\boldsymbol\Sigma_{ij}^{-1}\boldsymbol e_{ij}.\tag{10.12}
$$
> 完整推导（伴随、BCH、(10.5)–(10.8)）见回环检测/位姿图章（本仓库 `loop_closure__sf14_11.md` / 后端章），此处给系统集成所需的"回环 → 位姿图"接口。⚠️ 注意 (10.4) 中本书用**左扰动**；与 ORB-SLAM2 g2o Sim3 边的右扰动约定不同，成书统一到本书右扰动主约定时需重推符号。

### 9.9 ORB-SLAM2 vs 十四讲 ch13 精简 VO 对照表（综合 agent 直接用）

| 维度 | 十四讲 ch13 精简双目 VO | ORB-SLAM2 完整系统 |
| --- | --- | --- |
| 线程数 | 2（前端拉帧 + 后端优化线程；另有 Viewer 线程） | 3 + 1（Tracking / Local Mapping / Loop Closing + 可选 Full BA） |
| 前端特征 | GFTT 角点 + LK 光流（左右目 / 时序均用光流） | ORB 特征 + 描述子匹配（含词袋加速、极线/投影引导） |
| 前端定位 | pose-only BA（恒速模型初值，单顶点 g2o） | motion-only BA（恒速模型 / 重定位 / 参考关键帧三种跟踪） |
| 初始化 | 单帧双目三角化（带真实尺度） | 双目/RGBD 单帧；单目需两帧自动初始化（单应/基础矩阵择优） |
| 地图窗口 | **纯时序最近 7 关键帧** + 删最近/最远启发式 | **共视图**局部窗口（$\mathsf K_1$ 优化 + $\mathsf K_2$ 固定） |
| 关键帧准则 | 跟踪内点 < 80 即插 | 显式四条件（重定位/空闲/≥50点/<90%共视）+ 近点条件 |
| 后端 | 激活窗口局部 BA（全帧优化，无固定帧） | 局部 BA（K1 优化 / K2 固定）+ 全局 BA（回环后） |
| 关键帧管理 | 删最旧/最近（控制 7 帧） | 90% 冗余剔除 |
| 地图点管理 | chi2 剔外点观测；`CleanMap` 删 0 观测点 | 严格剔除（25% 可见率 / ≥3 帧观测）+ 持续创建 |
| 回环 | **无**（仅习题3 建议） | DBoW2 检测 + Sim3/SE3 + 回环融合 + 本质图 PGO + Full BA |
| 重定位 | **无**（`Reset` 空实现） | 词袋全局重定位 + PnP RANSAC |
| 地图复用/保存 | 无 | 支持（map reuse / 纯定位模式） |
| 适用 | 教学 / VO（局部、会漂移） | 工业级完整 SLAM（全局一致） |

---

## 10. 实验效果（S1 §13.4，全量）

- **数据集**：Kitti odometry，下载 http://www.cvlibs.net/datasets/kitti/eval_odometry.php （约 22 GB），解压得若干视频段，以第 0 段为例。
- **配置**：编译后在 `config/default.yaml` 填数据路径，作者机器上为 `dataset_dir: /media/xiang/Data/Dataset/Kitti/dataset/sequences/00`。
- **运行**：终端 `bin/run_kitti_stereo` → 看到定位输出（图 13-4）。运行期间程序显示**激活的关键帧和地图**，它们随镜头运动**不断增长和消失**（激活窗口滑动的直观体现）。
- **耗时与内存观察（S1 原文）**：
  - 处理**非关键帧**时，耗时约 **16 毫秒**。
  - 处理**关键帧**时，由于新增提取特征点和寻找右图匹配的步骤，**耗时会适当增多**。
  - **内存增长问题**：地图目前会存储**所有**关键帧和地图点，运行一段时间后导致内存增长。如果不需要全部地图，可以只保留激活的部分。
> 综合可做一张"非关键帧 16ms / 关键帧更慢"的耗时拆解，并把"内存随 `keyframes_/landmarks_`（全量）增长 vs 只留 `active_`"作为工程权衡讨论。

---

## 11. 工程实现要点汇总（贯穿全章，供综合 agent 的「实践/工程」框）

1. **多线程并发模型**：前端（拉帧/跟踪，主线程）+ 后端（优化，独立线程，条件变量唤醒）+ 可视化（独立线程）。同步原语：`std::mutex`（Frame.pose / MapPoint.data / Map.data 三处加锁）、`std::condition_variable`（前端→后端 `UpdateMap` 触发）、`std::atomic<bool>`（后端运行标志）。线程生命周期：构造起线程并挂起、`Stop()` 优雅退出 `join`。
2. **数据结构选型**：`std::shared_ptr` 管对象生命周期 + `std::weak_ptr` 断循环引用（Feature↔Frame、Feature↔MapPoint）；`std::unordered_map<id,Ptr>` 散列存关键帧/地图点（O(1) 增删查）；`std::list<weak_ptr>` 存地图点观测（频繁中间删除）。工厂静态函数 `CreateFrame/CreateNewMappoint` 集中分配自增 id。
3. **内存与规模控制**：激活窗口（`active_keyframes_`/`active_landmarks_`，N=7）使后端 BA 规模有界；`RemoveOldKeyframe`（删最旧/最近）+ `CleanMap`（删 0 观测点）回收；但 `keyframes_`/`landmarks_` 仍存全量 → 长跑内存增长（已知权衡）。
4. **效率技巧**：图像 resize 0.5 提速（内参同步减半）；光流用**重投影点作初值** + `OPTFLOW_USE_INITIAL_FLOW`（收敛快准）；GFTT **掩膜**避免特征扎堆；前端 Dense 求解器（小问题）/ 后端 CSparse（大稀疏 + `setMarginalized` Schur）；外点剔除分阶段（前期 Huber、`iteration==2` 去核）。
5. **鲁棒性**：Huber 鲁棒核（$\delta=5.991$）；chi2 外点门限（自由度 2 卡方 0.05）；后端外点阈值自适应（内点率 < 0.5 时翻倍放宽）；三种跟踪状态（GOOD/BAD/LOST）。
6. **模块解耦**：核心算法（前端/后端/地图/优化）与周边（相机/配置/数据集/可视化）分离；依赖注入装配（`VisualOdometry::Init`）；功能拆成短小函数（"初始化提特征"与"关键帧提特征"复用 `DetectFeatures`），便于复用与替换（习题2：换 ORB、换一维极线搜索、换直接法）。
7. **已知坑（成书 pitfall 框）**：① `triangulation()` 返回逻辑与语义相反的 bug（§5.8）；② `Reset()` 空实现（§5.10）；③ 激活窗口无固定帧锚定（§9.5），易整体漂移；④ 全量地图不释放导致内存增长（§10）。

---

## 12. 习题（S1 §习题，全量）

1. 本书使用的 C++ 技巧你都看懂了吗？如不明白，使用搜索引擎补习：基于范围的 for 循环、智能指针、设计模式中的**单例模式**等。
2. 考虑对本讲系统进行优化。例如：使用更快的提特征点方式（本节用 GFTT，并不算快）；在左右匹配时使用**一维搜索**而非二维光流；使用**直接法**同时估计位姿与特征点对应关系等。
3. （\*）为本节代码添加**回环检测模块**，在检测到回环时使用**位姿图**进行优化以消除累积误差。
> 习题3 正是 §9.8 + S1b §10.2 位姿图所对应的内容——成书可把"回环线程"作为对精简 VO 的自然延伸，并指向回环检测章与位姿图章。

---

## 13. OCR / 转写修正说明（综合 agent 必读）

| 处 | 源（S1 正文 OCR / 抓取） | 修正（依据） | 说明 |
| --- | --- | --- | --- |
| `frame.h` | 正文分两段印出，第二段 `public: // data members` 缩进错乱 | 合并为完整 struct（§4.1） | OCR 分页割裂，按 S2 合并 |
| `backend.h` | `std::shared_ptr<Map_;`、`Camera::Ptr ...` 后缺 `};` | `std::shared_ptr<Map> map_;` 等（§7.1） | OCR 残缺，按 S2 修正 |
| `backend.cpp::Optimize` | `std::map<unsigned long, VertexPose =>`、`VertexXYZ =>` | `std::map<unsigned long, VertexPose *>` 等（§7.3） | OCR 把 `*>` 误识为 `=>`；按 S2 修正 |
| `backend.cpp::Optimize` 加边 | 正文无条件加边 | S2 实为 `if(vertices.find&&vertices_landmarks.find){...addEdge} else delete edge`（§7.3） | 仓库健壮性补丁；以 S2 为准 |
| `TrackLastFrame` 光流窗口 | 正文 `cv::Size(21, 21)` | S2 仓库 `cv::Size(11, 11)`（§5.5） | 版本差异；以仓库为准并标注 |
| `MapPoint::SetPos` | 正文末多一个 `;`（`};`） | 保留（C++ 合法）（§4.3） | 无害 |
| `Map` 注释 | 正文 `all key-frames` 重复（active 那行也写 all） | 保留原样（§4.4） | 源注释笔误，保留并加注 |
| `triangulation` 返回值 | 正文未印此函数 | S2 抓取得到，含返回逻辑 bug（§5.8） | 书面未体现，抓取后发现并标注修正 |
| `camera.cpp` 变换实现 | WebFetch 只取到 `camera.h` 声明 | 据声明 + §5/§8.2 调用语义还原标准实现（§8.1） | 已标注"依据声明还原"，公式与调用一致 |
| 数学公式（重投影雅可比、Schur、Huber、位姿图） | 来自 S1b 第 9/10 讲与 S5 第 7 章 | 逐式录入并标源编号（§6.3/§7.4/§9.8） | 本章正文无独立编号公式，理论从相邻章/Handbook 补全 |

---

## 14. 给综合 agent 的成书建议（结构与图）

1. **章节顺序建议**：① 为何要工程章（§2）→ ② 系统架构与模块划分 + 框图（§3）→ ③ 数据结构（Frame/Feature/MapPoint/Map，§4）→ ④ 双目前端全链路（提特征 mask / 左右光流 / 时序光流 / pose-only BA / 三角化 / 关键帧选取，§5–§6）→ ⑤ 后端局部 BA + 滑窗边缘化理论（§7）→ ⑥ 周边模块脚手架（§8，可压缩）→ ⑦ 与 ORB-SLAM2 对照（§9，重点）→ ⑧ 实验与工程要点（§10–§11）。
2. **必画 TikZ 图**：(a) 图 13-2 数据结构三层关系；(b) 图 13-3 前后端+地图流水线（标条件变量触发）；(c) 前端状态机（INITING/TRACKING_GOOD/TRACKING_BAD/LOST 转移）；(d) ORB-SLAM2 三+一线程架构（对照 S5 图 7.7）；(e) 共视图 vs 本质图（θ=15 / θ=100，对照 S5 图 7.5）；(f) 激活窗口滑动示意（最近 7 帧）；(g) 双目三坐标 $(u_L,v_L,u_R)$ 与近/远点。
3. **必列表格**：前端参数表（§5.2）；ORB-SLAM2 vs ch13 对照表（§9.9）；OCR 修正表（§13）。
4. **必设 pitfall/practice 框**：triangulation bug、Reset 空实现、无固定帧、内存增长（§11.7）；多线程加锁三处；chi2=5.991 的卡方来历（§7.4）。
5. **代码呈现**：用 `codebox[lang=cpp]`；ASCII-only 注释（源中文注释转英文或移到正文）；长函数（EstimateCurrentPose / Optimize / 前端初始化组）建议按"骨架 + 关键行高亮"呈现，全代码可入附录或保留于本抽取引用。
6. **cite 建议**（bib key，供 synth 合并，勿改 refs.bib）：十四讲用 `gaoxiang2019slam14`；ORB-SLAM2 建议加 `murartal2017orbslam2`（Mur-Artal & Tardós, T-RO 2017）；ORB-SLAM 建议加 `murartal2015orbslam`（Mur-Artal et al., T-RO 2015）；Handbook 用 `carlone2026handbook`；PTAM 可加 `klein2007ptam`；keyframe-vs-filter 可加 `strasdat2010realtime`。
