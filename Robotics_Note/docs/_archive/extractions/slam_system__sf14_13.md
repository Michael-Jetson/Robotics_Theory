# 抽取留痕：《视觉SLAM十四讲》第13讲 实践——设计 SLAM 系统

> 本文件是项目内部「抽取留痕」，目标是把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章。**禁摘要、禁凝练**：每一步推导、每一道例题/数值例、每一条定义/原理、每一段代码/伪码、每一张表、每一个工程权衡均完整记录。源中关键代码骨架逐字记录（含 OCR 修正后的可编译版本）。宁长勿略。
>
> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/13_实践设计SLAM系统.md`（共 **697 行**，已完整读取）
> **源章节**：《视觉SLAM十四讲》（第二版，高翔等）第 13 讲「实践：设计 SLAM 系统」（对应代码仓库 `slambook2/ch13`）
> **⚠ 源为图像 OCR 所得，必有识别错**。本源**以 C++ 代码为主**（约占全章 70%），OCR 对代码的破坏尤其严重（`>` 被认成 `=`、模板尖括号丢失、闭合括号丢失、`map_` 等成员名被截断、`landmark`→`lanmark` 等）。本抽取已用 C++/g2o/Eigen/Sophus/OpenCV 领域知识重建可编译版本，所有修正逐处在文末「OCR 修正说明」及行内 `[OCR修正]` 标注；不确定处标 `[OCR?]`。

---

## 0. 本章覆盖与范围说明（给综合 agent 的提示）

本第 13 讲是《十四讲》的**最终工程实践章（总结章）**，把前面各章（VO/优化/回环/建图）的"砖块"整合成一个**能在 Kitti 数据集上实际运行的精简版双目视觉里程计**。它**不引入新的数学理论**，而是讲"软件框架如何搭建""数据结构如何设计""多线程如何组织""BA 规模如何控制"等**工程实现**问题。

### 本章对【本章聚焦】清单的覆盖

| 【本章聚焦】要点 | 源对应小节 | 本文是否覆盖 |
|---|---|---|
| 把 VO/优化/回环/建图整合成能运行的视觉 SLAM 系统 | §13.1、§13.2 | ✅ 完整（设计哲学 + 总体框架图 + 模块清单） |
| 系统架构与模块划分 | §13.2「工程框架」「确定核心算法结构」 | ✅ 完整（目录结构 6 项 + 前端/后端/地图三大模块 + 4 个周边小模块 + 图 13-3 流水线） |
| 双目/RGBD 前端：特征提取匹配/位姿估计/三角化/参考帧策略 | §13.3.2 | ✅ 完整（**双目**前端三状态机、LK 光流追踪、`Track`/`TrackLastFrame` 完整代码、关键帧策略；右目/三角化策略讨论。**注**：本源实现的是**双目**，非 RGBD；RGBD 思路相通但本源未演示） |
| 帧与地图点的数据结构、关键帧选取准则 | §13.3.1 | ✅ 完整（`Frame`/`Feature`/`MapPoint`/`Map` 四个类完整代码 + 线程锁 + weak_ptr 循环引用规避 + 工厂模式；关键帧准则=追踪内点数过少） |
| 局部地图与（滑窗）后端优化 | §13.3.1（Map 类）、§13.3.3 | ✅ 完整（**激活窗口=滑窗**=最新 7 个关键帧；后端 `BackendLoop`/`Optimize` 完整 g2o BA 代码 + 鲁棒核 + 外点剔除自适应阈值） |
| （可选）回环线程 | 习题 3 | ⚠ 仅作**习题**提出（"为本节代码添加回环检测模块，检测到回环时用位姿图优化消累积误差"）。本源**未实现**回环线程。回环检测的数学/实现见第 11 讲（`loop_closure__sf14_11.md`、`loop_closure__dbow.md`），位姿图优化见第 10 讲。 |
| 工程实现要点（多线程并发/效率） | §13.2、§13.3.1/§13.3.3、§13.4 | ✅ 完整（前后端各自线程、`std::mutex`/`std::condition_variable`/`std::atomic`、Pose/Pos 读写加锁、`weak_ptr` 防循环引用、地图规模控制、运行耗时实测 16ms/非关键帧） |
| 与 ORB-SLAM2 架构对照 | （本源未直接对照） | ⚠ **本源未与 ORB-SLAM2 做架构对照**。本文在 §A「与 ORB-SLAM2 架构对照（综合 agent 补充材料）」给出对照表，供综合时使用，并明确标注哪些来自本源、哪些是补充。 |

### 本源未含、需别处补的内容（给综合 agent）

- **回环线程的实现**：本源只在习题 3 提出，未实现。需从第 11 讲（词袋/DBoW3）+ 第 10 讲（位姿图优化）综合。
- **ORB-SLAM2 三线程架构细节**（Tracking / Local Mapping / Loop Closing、共视图 Covisibility Graph、本质图 Essential Graph、ORB 词袋重定位）：本源**完全未提**，§A 的对照属补充材料，非源内容。
- **单目初始化**：本源选双目正是为了"单帧即可初始化"，刻意回避了单目的初始化难题（见 §13.1 末）。单目初始化（2D-2D 对极几何/单应分解）见第 7 讲。
- **g2o 顶点/边的定义**（`VertexPose`、`VertexXYZ`、`EdgeProjection`）：本源在 `Optimize` 中**使用**了它们，但**未给出其类定义**（说"交给读者自行阅读"）。本文在 §13.3.3 末给出这些类的**重建骨架**（基于第 7 讲 BA 代码与 g2o 惯例），并明确标注"源未给定义、为综合补充"。
- **相机类 `Camera`、配置类 `Config`、数据集类 `Dataset`、可视化类 `Viewer`、`VisualOdometry` 总成类**：本源明言"限于篇幅交给读者自行阅读，书中只介绍核心部分"，**未给代码**。本文给出其**接口职责说明**（从源文字与 `Optimize`/`Track` 的调用反推），并标注"源未给完整代码"。

---

## 记号约定（本源《十四讲》第 13 讲 vs 本书统一约定）

本章是**工程实现章，数学符号极少**，主要是代码中的类型别名。下表对齐到本书统一约定。

| 项目 | 本源记号 / 代码类型 | 含义 | 本书统一约定 | 差异/转换提示 |
|---|---|---|---|---|
| 位姿 | `SE3`（=`Sophus::SE3d`）；成员 `pose_` | 帧位姿，**Tcw 形式**（world→camera） | $T\in SE(3)$ | ⚠ 源注释明写 `// Tcw形式Pose`，即 $T_{cw}$（把世界点变到相机系）。本书统一可记 $T_{cw}$；注意与"相机到世界" $T_{wc}=T_{cw}^{-1}$ 的区别。`relative_motion_` 为相邻帧相对位姿。 |
| 旋转 | （隐含于 `SE3`，未单独出现 `R`） | $R\in SO(3)$ | $R\in SO(3)$ | 一致（用 R，不用 C） |
| 3D 点 | `Vec3`（=`Eigen::Vector3d`）；`pos_` | 路标点世界坐标 | $\boldsymbol p_w\in\mathbb R^3$ | 一致 |
| 2D 点 | `Vec2`（=`Eigen::Vector2d`）；`cv::Point2f`、`cv::KeyPoint` | 像素坐标观测 | $\boldsymbol p\in\mathbb R^2$ | 一致；`toVec2(feat->position_.pt)` 把 OpenCV 点转 Eigen 二维 |
| 内参矩阵 | `Mat33 K`（=`Eigen::Matrix3d`） | 相机内参 $K$ | $K\in\mathbb R^{3\times3}$ | 一致 |
| 外参 | `SE3 left_ext`、`right_ext`；`cam_left_->pose()` | 左/右目相对于某基准（相机组中心/左目）的外参 | $T\in SE(3)$ | 双目左右目外参，用于把世界点投到对应目像素 |
| 信息矩阵 | `Mat22::Identity()` = $I_{2\times2}$ | 重投影边的信息矩阵 $\Omega$ | $\Omega=\Sigma^{-1}$ | 这里取单位阵，等价观测噪声 $\Sigma_v=I$（各向同性、单位方差）；对应本书 $\Sigma_v$ |
| 鲁棒核阈值 | `chi2_th = 5.991` | Huber 核的 $\delta$，卡方阈值 | — | 5.991 = 自由度 2、置信 95% 的 $\chi^2$ 分位数（$\chi^2_{2,0.95}$）；二维重投影残差用此剔除外点 |
| 卡方值 | `edge->chi2()` | 单条边的 $\chi^2$ 残差 = $\boldsymbol e^\top\Omega\boldsymbol e$ | $\chi^2=\boldsymbol e^\top\Sigma^{-1}\boldsymbol e$ | g2o 内部量 |
| 过程/观测噪声 | （本源未显式建模） | — | $\Sigma_w$（过程）/ $\Sigma_v$（观测） | ⚠ 本章是 BA（最小二乘重投影），**无运动方程的过程噪声 $\Sigma_w$**；观测噪声体现为信息矩阵 `Mat22::Identity()`（即 $\Sigma_v=I$）。 |
| 四元数/Hamilton | （本章未显式用四元数；位姿走 Sophus `SE3d`） | — | **Hamilton**（实部在前） | Sophus 内部用 Eigen 四元数（Hamilton 约定），与本书一致。本章代码层不直接触四元数。 |
| 右/左扰动 | （本章未写位姿雅可比，封装在 g2o `VertexPose` 的 `oplusImpl` 中） | — | **右扰动** | ⚠ 本源 `Optimize` 只调用 `VertexPose`，**未给其 `oplusImpl` 定义**，故源未明示左/右扰动。第 7 讲 BA 用的是**左扰动模型**（《十四讲》惯例 $\exp(\delta\boldsymbol\xi^\wedge)T$）。**本书统一用右扰动**，综合时若复用本章 BA 代码，须把 `VertexPose::oplusImpl` 改为右乘 $T\exp(\delta\boldsymbol\xi^\wedge)$ 并相应调整 `EdgeProjection::linearizeOplus` 的雅可比。详见 §13.3.3 末「g2o 顶点/边补充」。 |
| 李代数向量 | （本章未出现 $\boldsymbol\xi$ 显式拆分） | — | $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前、旋转在后） | 本章不涉及；提示综合时与第 7 讲/李代数章对齐。 |

**类型别名总表**（源散见于各 `.h`，本书统一为 `common_include.h` 风格；本源未集中列出，下表为从代码归纳）：

| 别名 | 实际类型 | 出处库 |
|---|---|---|
| `SE3` | `Sophus::SE3d` | Sophus |
| `Vec2` | `Eigen::Vector2d` | Eigen |
| `Vec3` | `Eigen::Vector3d` | Eigen |
| `Mat22` | `Eigen::Matrix2d` | Eigen |
| `Mat33` | `Eigen::Matrix3d` | Eigen |
| `Mat` | `cv::Mat` | OpenCV |

**术语对照（中英）**：
- 视觉里程计 = Visual Odometry (VO)；前端 = Frontend；后端 = Backend。
- 帧 = Frame；特征 = Feature；路标 / 路标点 / 地图点 = Landmark / Map Point（源明确三者语义相同，均指 3D 空间点）。
- 关键帧 = Keyframe；地图 = Map；激活（窗口）= Active (window)。
- 光流 = Optical Flow；LK 光流 = Lucas-Kanade Optical Flow（金字塔，`calcOpticalFlowPyrLK`）。
- 三角化 = Triangulation；外点 / 异常点 = Outlier；内点 = Inlier。
- 局部 BA = Local Bundle Adjustment；滑窗 = Sliding Window；鲁棒核 = Robust Kernel（Huber）。
- 边缘化 = Marginalization（`setMarginalized(true)`）；循环引用 = Circular Reference（`shared_ptr` 互持导致内存泄漏，用 `weak_ptr` 破解）。
- 工厂模式 = Factory Pattern；单例模式 = Singleton Pattern；GFTT = Good Features To Track（Shi-Tomasi 角点）。

---

# 第 13 讲 实践：设计 SLAM 系统（正文逐节抽取）

## 主要目标（源 §"主要目标"）

1. 实际设计一个视觉里程计。
2. 理解 SLAM 软件框架是如何搭建的。
3. 理解在视觉里程计设计中容易出现的问题，以及修补的方式。

**本讲引言（源开篇逐字）**：本讲是全书的总结部分。我们将用到前面所学的知识，实际书写一个视觉里程计程序。你会管理局部的机器人轨迹与路标点，并体验一个软件框架是如何组成的。在操作过程中，我们会遇到许多实际问题：如何对图像进行连续的追踪，如何控制 BA 的规模，等等。为了让程序稳定运行，我们需要处理以上的种种情况，这将带来许多工程实现方面的、有益的讨论。

---

## 13.1 为什么要单独列工程章节（源 §13.1）

**核心论点（源逐字保真，含比喻）**：

> 知晓砖头和水泥的原理，并不代表能够建造伟大的宫殿。

源用《我的世界》（Minecraft）作比喻（图 13-1）：玩家拥有的只是色彩、纹理不同的方块，性质极其简单；理解一个方块极为简单，但初学者往往只能搭建简单的"火柴盒房屋"，而有经验、有创造力的玩家则能用这些简单方块建造民居、园林、楼台亭榭乃至城市。

**对 SLAM 的类比论证**：
- 在 SLAM 中，**工程实现和理解算法原理至少同等重要**，甚至更应强调如何书写实际可用的程序。
- 算法原理就像一个个方块，可以清楚明确地讨论其原理和性质；但仅理解一个个方块并不能帮你建造真正的建筑——建造建筑需要大量的尝试、时间和经验。
- 就像在 Minecraft 里需要掌握各种立柱、墙面、屋顶的结构、墙面的雕花、几何形体角度的计算一样，SLAM 的具体实现也会有很多工程设计和技巧，还需讨论**每一步出现问题之后该如何处理**。

**会遇到的共性问题（源列举）**：
- "怎么管理地图点"
- "如何处理误匹配"
- "如何选择关键帧"

源强调：原则上每个人实现的 SLAM 都会有所不同，多数时候并不能说哪种实现方式一定最好；但通常会遇到上述共同问题，希望读者对这些可能出现的问题产生**直观的感觉**——作者认为这种感觉非常重要。

**本讲的策略（"由简到繁"/"堆雪人"）**：从简单的数据结构出发，先做一个简单的视觉里程计，再慢慢把额外功能加进来——"把从简单到复杂的过程展现给读者，这样读者才会明白一个库是如何像雪人那样慢慢堆起来的"。

**本讲实现目标（源 §13.1 末，关键设计决策）**：
- 代码放在 `slambook2/ch13`。
- 实现一个**精简版的双目视觉里程计**，在 **Kitti 数据集**中运行。
- 这个 VO 由 **一个光流追踪的前端** 和 **一个局部 BA 的后端** 组成。
- **为什么选双目 VO？**（两条理由，源逐字）：
  1. 双目实现相对简单，**只需单帧即可完成初始化**（回避单目初始化难题）；
  2. 双目存在 **3D 观测**，实现效果也会比单目好。

> **[综合提示]** 此处是关键的设计折中：双目 = 单帧初始化 + 直接 3D 观测，避免了单目的尺度不确定与初始化脆弱。RGBD 在这两点上与双目同理（深度图直接给 3D），故本章数据结构与流程对 RGBD 几乎可直接复用，差别仅在"右目匹配/三角化"换成"读深度图"。

---

## 13.2 工程框架（源 §13.2）

### 13.2.1 目录结构（源 §13.2，逐条）

大多数 Linux 库都会按模块对算法代码文件分类存放（头文件、源代码、配置、测试、三方库等）。源按小型算法库的普遍做法分类文件：

| 目录 | 存放内容 | 源说明 |
|---|---|---|
| `bin/` | 编译好的二进制文件 | 可执行程序输出目录 |
| `include/myslam/` | SLAM 模块的头文件（主要是 `.h`） | **目的**：把包含目录设到 `include` 后，引用自己的头文件时需写 `#include "myslam/xxx.h"`，这样**不容易和别的库混淆** |
| `src/` | 源代码文件（主要是 `.cpp`） | 实现 |
| `test/` | 测试用的文件（也是 `.cpp`） | 单元/集成测试 |
| `config/` | 配置文件 | 如 `config/default.yaml`（记录数据集路径等可调参数） |
| `cmake_modules/` | 第三方库的 cmake 文件 | 在使用 **g2o** 之类的库时会用到（`FindG2O.cmake` 等） |

### 13.2.2 确定核心算法结构（源 §"确定核心算法结构"）

**设计方法论（源逐字）**：写代码之前应明确要写什么。引用"老观点"——**程序就是数据结构 + 算法**。故针对视觉里程计要问三个问题：
1. 视觉里程计需要处理怎样的**数据**？
2. 涉及的关键**算法**有哪些？
3. 它们之间是怎样的**关系**？

**基本数据单元的推导（源逐条）**：
- 处理的**最基本单元是图像**。在双目视觉里那就是**一对图像**，称为**一帧（Frame）**。
- 会对帧**提取特征（Feature）**。这些特征是很多 **2D 的点**。
- 在图像之间寻找特征的**关联**。如果能**多次看到**某个特征，就可以用**三角化**方法计算它的 **3D 位置，即路标（Landmark / Map Point）**。

> **结论**：**图像（帧）、特征、路标**是系统中最基本的三个结构（关系见图 13-2）。源明确：后续说明中，"路标、路标点或地图点"均指代 **3D 空间中的点，语义一样**。

**算法模块的划分（源逐字）**：SLAM 由**前后端**组成。
- **前端**：负责计算相邻图像的特征匹配。
- **后端**：负责优化整个问题。
- 在典型实现中，**二者应有各自的线程**：前端快速处理保证**实时性**，后端优化关键帧以保证**良好的结果**。

故程序有两个重要模块（源逐字）：

- **前端**：往前端插入图像帧 → 前端提取图像中的特征 → 与上一帧做**光流追踪** → 通过光流结果计算该帧的**定位** → 必要时补充新特征点并做**三角化**。**前端处理的结果作为后端优化的初始值**。
- **后端**：一个**较慢的线程**，拿到处理后的**关键帧和路标点** → 对它们优化 → 返回优化结果。**后端应控制优化问题的规模在一定范围内，不能随时间一直增长**。

**总体框架（图 13-3，源文字描述）**：以"流水线框图"表示。**在前后端之间放了一个"地图模块"来处理数据流动**。前后端在各自线程中处理数据，预想流程：

```
前端线程：提取了关键帧后 ──► 往地图中添加新数据
                                    │
                                    ▼（地图更新触发）
后端线程：检测到地图更新时 ──► 运行一次优化 ──► 把地图中"旧的"关键帧和地图点去掉，保持优化规模
```

> **[综合提示·图 13-3 重建]** 源图 13-3 为图片（OCR 不可读），上述文字流程即其内容。可重画为三栏流水线：**前端（Frontend）** → **地图（Map，含 active window）** ↔ **后端（Backend）**；前端向地图 `InsertKeyFrame/InsertMapPoint` 并 `UpdateMap` 通知后端；后端 `GetActiveKeyFrames/GetActiveMapPoints` 取激活集优化、`RemoveOldKeyframe` 维护窗口。

**周边小模块（源逐条，"不算核心但不可或缺"）**：
- **相机类（Camera）**：管理相机的**内外参和投影函数**。
- **配置文件管理类（Config）**：方便从配置文件读取内容；配置文件记录重要参数，方便调整。
- **数据集类（Dataset）**：因为算法在 Kitti 上运行，需按 **Kitti 的存储格式**读取图像数据，由单独的类处理。
- **可视化模块（Viewer）**：观察系统运行状态，"否则就得对着一串串的数值挠头"。

源明言：这些模块**限于篇幅交给读者自行阅读，书中只介绍核心部分**（即数据结构、前端、后端）。

> **[综合提示]** 因此本章可演示的"自包含可编译"部分是**数据结构 + 前端骨架 + 后端 BA**；`Camera/Config/Dataset/Viewer/VisualOdometry` 需综合 agent 用接口说明补齐（见 §B「周边模块接口职责」）。

---

## 13.3 实现（源 §13.3）

## 13.3.1 实现基本数据结构（源 §13.3.1）

**设计原则（源逐字）**：先实现**帧、特征、路标点**这三个类。对于基本数据结构，**通常建议设成 `struct`**，无须定义复杂的私有变量和接口。考虑到这些数据可能被**多个线程访问和修改**，在关键部分需要加上**线程锁**。

### (1) Frame 结构（源：`slambook2/ch13/include/myslam/frame.h`）

源给出的 `Frame` 定义（OCR 原文分两段，下面合并为完整可编译版本，行内标注 OCR 修正）：

```cpp
// slambook2/ch13/include/myslam/frame.h
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;            // id of this frame
    unsigned long keyframe_id_ = 0;   // id of key frame
    bool is_keyframe_ = false;        // 是否为关键帧
    double time_stamp_;               // 时间戳，暂不使用
    SE3 pose_;                        // Tcw形式Pose
    std::mutex pose_mutex_;           // Pose数据锁
    cv::Mat left_img_, right_img_;    // stereo images

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

    /// 设置关键帧并分配关键帧id
    void SetKeyFrame();

    /// 工厂构建模式，分配id
    static std::shared_ptr<Frame> CreateFrame();
};
```

**源对 Frame 的说明（逐字）**：`Frame` 含有 **id、位姿、图像及左右图像中的特征点**。其中 **Pose 会被前后端同时设置或访问**，所以定义 Pose 的 **Set/Get 函数，在函数内加锁**。同时，`Frame` 可由**静态函数构建（工厂模式）**，在静态函数中**自动分配 id**。

**关键设计点（抽取注解）**：
- `id_`（全局帧 id）与 `keyframe_id_`（关键帧 id）**分开**——后端 g2o 顶点用 `keyframe_id_` 编号（见 §13.3.3）。
- `pose_` 注释 `// Tcw形式Pose`：位姿是 $T_{cw}$（world→camera）。
- 左目特征 `features_left_` 与右目特征 `features_right_` 分开存；右目无对应时置 `nullptr`。
- `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`：因含 Eigen 固定大小成员（`SE3` 内含定长矩阵），需 16 字节对齐 `new`。

### (2) Feature 类（源：`slambook2/ch13/include/myslam/feature.h`）

源给出的 `Feature` 定义（OCR 原文分两段，合并）：

```cpp
// slambook2/ch13/include/myslam/feature.h
struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;          // 持有该feature的frame
    cv::KeyPoint position_;               // 2D提取位置
    std::weak_ptr<MapPoint> map_point_;   // 关联地图点

    bool is_outlier_ = false;             // 是否为异常点
    bool is_on_left_image_ = true;        // 标识是否提在左图，false为右图

public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
```

**源对 Feature 的说明（逐字）**：`Feature` 最主要的信息是自身的 **2D 位置**；此外 `is_outlier_` 为异常点标志位，`is_on_left_image_` 为它是否在左侧相机提取的标志位。可以通过一个 `Feature` 对象**访问持有它的 `Frame` 及它对应的路标**。不过，**`Frame` 和 `MapPoint` 的实际持有权归地图所有**，为了**避免 `shared_ptr` 产生的循环引用**$^{①}$，这里使用了 **`weak_ptr`**。

> **脚注①（循环引用，抽取补全）**：若 `Frame` 用 `shared_ptr` 持有 `Feature`、`Feature` 又用 `shared_ptr` 反持 `Frame`，二者引用计数互相 ≥1，永远不归零 → 内存泄漏。解法：让其中一方（这里是 `Feature→Frame`、`Feature→MapPoint`）改用 **`weak_ptr`**（不增加引用计数），用时 `.lock()` 提升为 `shared_ptr`（若对象已析构则得 `nullptr`，正好用于判活）。这一模式在前端 `TrackLastFrame`（`kp->map_point_.lock()`）和后端 `Optimize`（`obs.lock()`、`feat->frame_.lock()`）中反复出现，是本章最重要的 C++ 工程技巧。

### (3) MapPoint 类（路标点）（源：`slambook2/ch13/include/myslam/mappoint.h`）

源给出的 `MapPoint` 定义（OCR 原文分两段，合并；OCR 把第一段末尾 `void SetPos(...)` 体断到第二段，已接合）：

```cpp
// slambook2/ch13/include/myslam/mappoint.h
struct MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;            // ID
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();        // Position in world
    std::mutex data_mutex_;
    int observed_times_ = 0;         // being observed by feature matching algo.
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
    }

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

**源对 MapPoint 的说明（逐字）**：`MapPoint` 最主要的信息是它的 **3D 位置**，即 `pos_` 变量，同样需要对它**上锁**。它的 `observations_` 变量记录了自己**被哪些 `Feature` 观察**。因为 `Feature` 可能被判断为 outlier，所以 `observations_` 部分发生改动时也需要**锁定**。

**关键设计点（抽取注解）**：
- `observations_` 用 `std::list<std::weak_ptr<Feature>>`：链表便于 `RemoveObservation` 中部删除；`weak_ptr` 防循环引用（与 (2) 同理，路标↔特征互持）。
- `observed_times_`：被观测次数，用于地图清理判断（`CleanMap` 删除观测为 0 的点）。
- `AddObservation` 在源中给了内联实现（push + 计数++ + 加锁）；`RemoveObservation` 仅声明（实现中需对应 `observed_times_--` 并从链表移除——源未给体，综合时补）。

### (4) Map 类（地图，实际持有者）（源：`slambook2/ch13/include/myslam/map.h`）

源说明：在框架中，**让地图类实际持有这些 `Frame` 和 `MapPoint` 的对象**，所以需要定义一个地图类。源给出的 `Map` 定义（OCR 原文分两段，合并）：

```cpp
// slambook2/ch13/include/myslam/map.h
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

    /// 清理map中观测数量为零的点
    void CleanMap();

private:
    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;          // all landmarks
    LandmarksType active_landmarks_;   // active landmarks
    KeyframesType keyframes_;          // all key-frames
    KeyframesType active_keyframes_;   // all key-frames（[OCR?] 应为 active key-frames）

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;     // 激活的关键帧数量
};
```

**源对 Map 的说明（逐字，含滑窗核心思想）**：地图以**散列形式（`unordered_map`）**记录了所有的关键帧和对应的路标点，同时维护一个**被激活的关键帧和地图点**。这里**"激活"的概念即我们所谓的"窗口"，它会随着时间往前推动**。**后端将从地图中取出激活的关键帧、路标点进行优化，忽略其余的部分，达到控制优化规模的效果**。当然，**激活的策略是由我们自己定义的**，简单的激活策略就是**去除最旧的关键帧而保持时间上最新的若干个关键帧**。在本书的实现中，**只保留最新的 7 个关键帧**（`num_active_keyframes_ = 7`）。

**关键设计点（抽取注解 — 这是"滑窗后端"的实现机理）**：
- **两套容器**：`landmarks_`/`keyframes_`（全部历史）与 `active_landmarks_`/`active_keyframes_`（激活窗口）。后端只动激活集 → **优化规模恒定**（≤7 关键帧 + 它们观测到的路标）。
- **滑窗推进**：`InsertKeyFrame` 加入新关键帧到激活集；当激活关键帧数 > 7，`RemoveOldKeyframe` 把"最旧"的移出激活集（仍保留在 `keyframes_` 全集中）。`RemoveOldKeyframe` 为 `private`（仅地图内部调用）。
- **散列键 = id**：`LandmarksType` 键为 `MapPoint::id_`，`KeyframesType` 键为 `keyframe_id_`（与后端顶点编号一致）。
- 所有 getter 加 `data_mutex_` 锁（前后端并发安全）。

> **[综合提示]** `Map` 类就是 §13.2 框架图中"前后端之间的地图模块"。它把"滑窗后端"落地为"激活集 + RemoveOldKeyframe"；本书统一概念"局部地图/滑窗"在此对应。`CleanMap` 删观测为 0 的路标，避免地图膨胀（但全集 `landmarks_/keyframes_` 仍随时间增长，见 §13.4 内存说明）。

---

## 13.3.2 前端（源 §13.3.2）

### 前端的实现自由度（源逐字讨论）

前端需根据双目图像确定该帧位姿，但实现有多种方法，源列出几个**开放性设计问题**：
- 怎样使用右目图像？是每一帧都和**左右目各比较一遍**，还是仅比较左右目之一？
- 三角化时，是考虑**左右目图像**的三角化，还是考虑**时间上前后帧**的三角化？
- 实际中**任意两张图像都可以做三角化**（比如前一帧的左图对下一帧的右图），所以每个人实现起来都会不太一样。

### 前端处理逻辑（源逐条，本实现的确定方案）

为简单起见，源确定前端逻辑如下：

1. 前端本身有 **初始化（INITING）、正常追踪（TRACKING_GOOD/BAD）、追踪丢失（LOST）** 三种状态。
2. **初始化状态**：根据**左右目之间的光流匹配**，寻找可以三角化的地图点，成功时建立初始地图。
3. **追踪阶段**：前端计算**上一帧的特征点到当前帧的光流**，根据光流结果计算图像位姿。**该计算只使用左目图像，不使用右目**。
4. **如果追踪到的点较少，就判定当前帧为关键帧**。对于关键帧，做以下几件事：
   - 提取新的特征点；
   - 找到这些点在**右图的对应点**，用**三角化**建立新的路标点；
   - 将新的关键帧和路标点加入地图，并**触发一次后端优化**。
5. **如果追踪丢失，就重置前端系统，重新初始化**。

> **关键帧选取准则（本书统一表述）**：本章准则是 **"追踪到的内点数过少 → 设为关键帧"**（见下方 `Track` 中 `tracking_inliers_` 与阈值比较，以及 `InsertKeyframe()` 内部判据）。这与 ORB-SLAM2 的多条件关键帧策略不同（见 §A）。

### (1) 前端状态机入口 `AddFrame`（源：`slambook2/ch13/src/frontend.cpp`）

```cpp
// slambook2/ch13/src/frontend.cpp
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

**抽取注解**：典型**状态机**入口。`status_` 为枚举 `FrontendStatus{ INITING, TRACKING_GOOD, TRACKING_BAD, LOST }`（源未单列枚举定义，但由代码可确知）。每帧处理后把 `current_frame_` 存为 `last_frame_`，供下一帧光流参考。`StereoInit/Reset` 源未给体（"自行阅读"），职责见前端逻辑第 2、5 条。

### (2) 追踪函数 `Track`（源：`slambook2/ch13/src/frontend.cpp`）

```cpp
// slambook2/ch13/src/frontend.cpp
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

**抽取注解（逐行逻辑）**：
- **匀速运动模型预测初值**：`current_frame_->SetPose(relative_motion_ * last_frame_->Pose())`。即假设本帧相对上帧的运动 ≈ 上帧相对上上帧的运动（`relative_motion_`），用它把上帧位姿外推为本帧初值。因 `pose_` 是 $T_{cw}$，相对运动左乘：$T_{cw}^{(k)}\approx \Delta T\cdot T_{cw}^{(k-1)}$。
- `TrackLastFrame()`：LK 光流把上帧左目特征追到本帧左目（见 (3)），返回追踪上的点数 `num_track_last`。
- `EstimateCurrentPose()`：用追踪到的 2D-3D 对应做**仅优化位姿**的 BA（g2o，源未给体；职责=PnP/位姿图单点优化），返回内点数 `tracking_inliers_`。
- **三档状态判定**（阈值 `num_features_tracking_`、`num_features_tracking_bad_` 为成员配置）：
  - `inliers > num_features_tracking_` → `TRACKING_GOOD`；
  - 否则 `inliers > num_features_tracking_bad_` → `TRACKING_BAD`；
  - 否则 → `LOST`。
- `InsertKeyframe()`：内部判断"是否需要设为关键帧"（追踪内点过少时设），若是则提新特征 + 右目三角化 + 加地图 + 触发后端（源未给体，职责见前端逻辑第 4 条）。
- **更新相对运动**：`relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse()`，即 $\Delta T = T_{cw}^{(k)}\,(T_{cw}^{(k-1)})^{-1}$，供下一帧预测。
- 若有可视化器，推送当前帧。

> **[OCR 修正注]** 源此处 `EstimateCurrentPose` 与三档判定逻辑完整；`relative_motion_` 的两处用法（预测用左乘、更新用右乘 inverse）自洽，无 OCR 错。

### (3) LK 光流追踪 `TrackLastFrame`（源：`slambook2/ch13/src/frontend.cpp`）

源逐字（OCR 原文分两段，且 `for` 循环大括号被 OCR 错位——`std::vector<uchar> status;` 等被卷入 `for` 体内，实际应在循环外。下面给出**修正后的正确结构**，行内标注）：

```cpp
// slambook2/ch13/src/frontend.cpp
int Frontend::TrackLastFrame() {
    // use LK flow to estimate points in the right image  [OCR? 注释原文如此；
    // 实际本函数追踪的是左目上一帧→当前帧，非右图，疑为复制粘贴遗留的注释]
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
    }   // [OCR修正] 此处闭合 for；源 OCR 把下面几行误并入循环体

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
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

**抽取注解（逐行逻辑）**：
- **构造光流初值**：遍历上帧左目每个特征 `kp`：
  - **若该特征已关联地图点**（`kp->map_point_.lock()` 非空）：用相机把该 3D 点按**当前帧位姿**投影到像素 `px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose())`，作为光流的**初始猜测**（更准）。`kps_last` 放上帧像素，`kps_current` 放投影像素。
  - **否则**（无地图点）：`kps_current` 初值就用上帧像素（原地起步）。
- **调用 LK 金字塔光流** `cv::calcOpticalFlowPyrLK`：
  - 输入：上帧左目 `left_img_` → 当前帧左目 `left_img_`，点集 `kps_last`→`kps_current`；
  - 窗口 `cv::Size(21, 21)`，金字塔层数 `3`；
  - 终止准则 `COUNT + EPS`：最多 30 次迭代或精度 0.01；
  - 标志 `cv::OPTFLOW_USE_INITIAL_FLOW`：**使用 `kps_current` 的初值**（即上面的投影猜测）——这是用 3D 投影加速光流收敛的关键。
- **回收成功追踪的点**：`status[i]` 为真者，构造一个尺寸 7 的 `cv::KeyPoint`，新建 `Feature`（属于当前帧），**继承上帧对应特征的地图点关联** `feature->map_point_ = last_frame_->features_left_[i]->map_point_`，push 进当前帧左目特征。
- 返回成功点数，并 `LOG(INFO)` 打印。

> **关键工程技巧（抽取强调）**：用"已知 3D 点的重投影"做光流初值（`OPTFLOW_USE_INITIAL_FLOW`），把"运动先验"注入光流，显著提升大位移下的追踪鲁棒性与速度。这正是"前端结果作为后端初值"思想在像素级的体现。

### 前端的代码组织哲学（源 §13.3.2 末，逐字）

> 实现时，尽量利用逻辑上的分拆，把复杂功能拆成一些短小的函数，直到底层才调用 OpenCV 或 g2o 实现特定功能。这样有利于提升程序的可读性和复用性，比如**初始化阶段的提特征和关键帧的提特征就可以使用同一个函数**。

源建议读者自行阅读以节省篇幅——**前端一共不到 400 行代码**。

> **[综合提示·前端未给体的函数清单]**（源用到但未给实现，综合时按职责补）：`StereoInit()`（左右目光流匹配+三角化建初始地图）、`Reset()`（重置状态）、`EstimateCurrentPose()`（g2o 仅位姿优化 PnP，返回内点数）、`InsertKeyframe()`（判关键帧→提新特征 `DetectFeatures()`→`FindFeaturesInRight()`→`TriangulateNewPoints()`→加地图→触发后端 `backend_->UpdateMap()`）。这些是 ch13 frontend.cpp 的标准成员，可据第 7/8 讲与本章逻辑复现。

---

## 13.3.3 后端（源 §13.3.3）

**总述（源逐字）**：相比前端，后端实现的逻辑会复杂一些。源给出后端整体实现（含一张 OCR 不可读的图，疑为后端类图/流程图）。

### (1) 后端类定义（源：`slambook2/ch13/include/myslam/backend.h`）

源逐字（OCR 有两处明显错：成员 `std::shared_ptr<Map_;` 被截断、`}` 与 `private` 段落 OCR 错位。下面为修正后的完整版本）：

```cpp
// slambook2/ch13/include/myslam/backend.h
class Backend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// 构造函数中启动优化线程并挂起
    Backend();

    /// 设置左右目的相机，用于获得内外参
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
    void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    std::shared_ptr<Map> map_;                       // [OCR修正] 源作 "std::shared_ptr<Map_;"
    std::thread backend_thread_;
    std::mutex data_mutex_;
    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};
```

**抽取注解（多线程同步三件套）**：
- `std::thread backend_thread_`：后端独立线程（构造函数中启动并挂起）。
- `std::condition_variable map_update_`：**条件变量**——前端 `UpdateMap()` 唤醒后端；后端 `BackendLoop` 在其上 `wait`。
- `std::atomic<bool> backend_running_`：**原子标志**，控制后端循环运行/停止（`Stop()` 置 false 并唤醒以退出）。
- `std::mutex data_mutex_`：保护后端内部数据 + 配合条件变量。
- `SetCameras`：注入左右目相机（取内参 K、左右外参），供 `Optimize` 构边。

### (2) 后端主循环 `BackendLoop`（源：`slambook2/ch13/src/backend.cpp`）

```cpp
// slambook2/ch13/src/backend.cpp
void Backend::BackendLoop() {
    while (backend_running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        /// 后端仅优化激活的Frames和Landmarks
        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}
```

**源说明（逐字）**：后端在启动之后，将**等待 `map_update_` 的条件变量**。当地图更新被触发时，**从地图中拿取激活的关键帧和地图点，执行优化**。

**抽取注解**：经典"生产者-消费者"模式。`map_update_.wait(lock)` 阻塞直到前端 `UpdateMap()`（内部 `map_update_.notify_one()`）唤醒。醒来后只取**激活集**（≤7 关键帧 + 其观测路标）做 `Optimize` → **优化规模恒定**。`backend_running_.load()` 为 false 时循环退出（`Stop` 触发）。

### (3) 后端优化 `Optimize`——局部 BA（源：`slambook2/ch13/src/backend.cpp`）

**源说明（逐字）**：优化函数和之前使用的 BA 类似，只是**数据要从 `Frame`、`MapPoint` 对象中获得**。

源逐字（OCR 原文分四段，破坏严重：`make_unique` 闭合括号丢、`std::map<…, VertexPose =>` 中 `*>` 被认成 `=>`、外点剔除 while 块的大括号错位。下面给出**修正后的完整可编译版本**，所有修正行内标注）：

```cpp
// slambook2/ch13/src/backend.cpp
void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks) {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));   // [OCR修正] 补 2 个闭合右括号
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // pose顶点，使用Keyframe id
    std::map<unsigned long, VertexPose *> vertices;   // [OCR修正] "VertexPose =>" → "VertexPose *>"
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();    // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }
        vertices.insert({kf->keyframe_id_, vertex_pose});
    }   // [OCR修正] 此处闭合关键帧 for；源 OCR 把后续行误并入

    // 路标顶点，使用路标id索引
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;  // [OCR修正] "VertexXYZ =>" → "VertexXYZ *>"

    // K和左右外参
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel阈值
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

            // 如果landmark还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0, vertices.at(frame->keyframe_id_));       // pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id));     // landmark
            edge->setMeasurement(toVec2(feat->position_.pt));
            edge->setInformation(Mat22::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);

            index++;
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
    }   // [OCR修正] 源此处大括号错位（多出/少一对 }），按逻辑闭合 while

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

    // Set pose and landmark position  [OCR修正] 源 "lanmark" → "landmark"
    for (auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}
```

**抽取注解（这是本章最核心的算法——局部 BA，逐块解析）**：

**① g2o 求解器配置**
- `BlockSolver_6_3`：位姿块 6 维（$\mathfrak{se}(3)$）、路标块 3 维（$\mathbb R^3$）——标准 BA 块结构。
- `LinearSolverCSparse`：稀疏线性求解（CSparse）。
- `OptimizationAlgorithmLevenberg`：**LM 算法**。
- `make_unique<BlockSolver>(make_unique<LinearSolver>())`：嵌套构造（OCR 丢了闭合括号，已补）。

**② 位姿顶点（VertexPose）**
- 对每个**激活关键帧**建一个 `VertexPose`，`setId(keyframe_id_)`，`setEstimate(kf->Pose())`（用前端给的位姿作初值）。
- 记录 `max_kf_id`（关键帧最大 id），用于给路标顶点编号让其**不与位姿顶点 id 冲突**。
- 散列 `vertices[keyframe_id_] = vertex_pose` 便于建边时索引。

**③ 路标顶点（VertexXYZ）**
- 用散列 `vertices_landmarks[landmark_id]`，**惰性创建**：只在第一次遇到该路标的观测时建顶点。
- `setId(landmark_id + max_kf_id + 1)`：**偏移编号**避免与位姿顶点 id 撞车（位姿用 `[0, max_kf_id]`，路标用 `> max_kf_id`）。
- **`v->setMarginalized(true)`**：把路标设为**被边缘化**变量 → g2o 内部用 **Schur 补**先消元路标、只解位姿块，再回代路标。这是 BA 高效求解的关键（利用 $H$ 矩阵的箭头/块稀疏结构）。

**④ 重投影边（EdgeProjection）—— 一元/二元边**
- **遍历每个激活路标的每个观测**（`landmark.second->GetObs()`）。
- **多重判活/跳过**（鲁棒性）：路标 `is_outlier_` 跳过；观测 `obs.lock()==nullptr`（特征已析构）跳过；特征 `is_outlier_` 或其 `frame_.lock()==nullptr`（帧已析构）跳过。这些 `weak_ptr.lock()` 判空是**防止访问已被地图清理对象**的关键。
- **左右目分别构边**：`is_on_left_image_` → `EdgeProjection(K, left_ext)`，否则 `EdgeProjection(K, right_ext)`。即**同一路标在左目和右目的观测各构一条边**，外参不同 → 实现了"双目两个视角约束同一 3D 点"。
- 边连接：顶点 0 = 该观测所属帧的位姿顶点 `vertices.at(frame->keyframe_id_)`；顶点 1 = 路标顶点 `vertices_landmarks.at(landmark_id)`。
- `setMeasurement(toVec2(feat->position_.pt))`：观测 = 特征像素坐标（2D）。
- `setInformation(Mat22::Identity())`：信息矩阵 $\Omega=I_{2\times2}$（即各向同性单位方差观测噪声 $\Sigma_v=I$）。
- **鲁棒核 Huber**：`RobustKernelHuber`，`setDelta(chi2_th)`，`chi2_th = 5.991`（$\chi^2_{2,0.95}$）—— 抑制误匹配/外点对优化的拉扯。
- `edges_and_features[edge] = feat`：建立"边 ↔ 特征"映射，供优化后回查哪些特征是外点。

**⑤ 优化 + 自适应外点剔除（两阶段）**
- `optimizer.optimize(10)`：先优化 10 次。
- **自适应阈值循环**（最多 5 次）：统计当前 `chi2_th` 下的内/外点数；若**内点率 > 0.5** 则停；否则 **`chi2_th *= 2`**（放宽阈值）再统计。其目的：当外点过多导致内点率过低时，逐步放宽阈值避免把太多点误杀（一种"软"外点判定）。

  > 内点率定义：$\text{inlier\_ratio} = \dfrac{\text{cnt\_inlier}}{\text{cnt\_inlier} + \text{cnt\_outlier}}$，停止条件 $\text{inlier\_ratio} > 0.5$。

- **最终外点标记 + 摘除观测**：用最终 `chi2_th`，把 `chi2() > chi2_th` 的边对应特征 `is_outlier_ = true`，并从其地图点中 `RemoveObservation(feat)`（删该观测，`observed_times_--`）；否则 `is_outlier_ = false`（**复活**之前可能被误判的点）。
- `LOG(INFO)` 打印外/内点数。

**⑥ 写回优化结果**
- 遍历位姿顶点：`keyframes.at(v.first)->SetPose(v.second->estimate())`（线程安全 set）。
- 遍历路标顶点：`landmarks.at(v.first)->SetPos(v.second->estimate())`。

**源结语（逐字）**：后端相比前端在代码上更加简短，**只有不到 200 行**。

> **[关键概念·这就是"局部 BA / 滑窗后端"]** 整个 `Optimize` 即对**激活窗口内**所有关键帧位姿 + 它们观测到的路标点做**联合最小二乘重投影优化**（双目左右目两类边、Huber 鲁棒核、Schur 补边缘化路标、自适应外点剔除）。目标函数（本书统一写法）：
> $$
> \{T^*_{cw,i},\,\boldsymbol p^*_{w,j}\}=\arg\min \sum_{i\in\text{active KF}}\ \sum_{(i,j)\in\text{obs}}\ \rho\!\Big(\big\|\boldsymbol z_{ij}-\pi\big(K,\,T_{\text{ext}}\,T_{cw,i}\,\boldsymbol p_{w,j}\big)\big\|^2_{\Sigma_v^{-1}}\Big),
> $$
> 其中 $\boldsymbol z_{ij}$ 为特征像素观测，$\pi$ 为投影函数，$T_{\text{ext}}\in\{\,$`left_ext`$,\,$`right_ext`$\,\}$ 为左/右目外参，$\Sigma_v=I$（信息矩阵单位阵），$\rho$ 为 Huber 核（$\delta^2=5.991$）。源代码本身**未写出此式**（封装在 g2o `EdgeProjection` 的 `computeError/linearizeOplus` 中）；此式为综合 agent 补充，便于成书时把"代码"与"数学"打通。

### (4) g2o 顶点/边定义（**源未给，综合补充骨架**）

> **重要说明**：源 `Optimize` **使用**了 `VertexPose`、`VertexXYZ`、`EdgeProjection`、`toVec2`，但**未在本章给出它们的定义**（属于 ch13 的 `g2o_types.h`，源说"自行阅读"）。下列骨架为**综合 agent 补充材料**，依据第 7 讲 BA 代码 + g2o 惯例 + 本章调用签名重建，**非源原文**。⚠ **本书统一用右扰动**，故 `VertexPose::oplusImpl` 写成右乘（与《十四讲》第 7 讲原版的左扰动相反，综合时务必统一）。

```cpp
// slambook2/ch13/include/myslam/g2o_types.h  —— [综合补充骨架，非源原文]

// 位姿顶点：估计量为 SE3（Tcw），6 维李代数增量
class VertexPose : public g2o::BaseVertex<6, SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override { _estimate = SE3(); }

    /// 增量更新：本书统一【右扰动】 T <- T * exp(dξ^)
    virtual void oplusImpl(const double *update) override {
        Vec6 dx;
        dx << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = _estimate * SE3::exp(dx);   // 右乘（本书右扰动约定）
        // 注：《十四讲》第7讲原版为左乘 SE3::exp(dx) * _estimate（左扰动）
    }
    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }
};

// 路标顶点：估计量为 3D 点
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

// 重投影边：二元边，连接 VertexPose(0) 与 VertexXYZ(1)，残差 2 维
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 构造时传入内参 K 与该目（左/右）的外参 cam_ext
    EdgeProjection(const Mat33 &K, const SE3 &cam_ext) : K_(K), cam_ext_(cam_ext) {}

    virtual void computeError() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ  *v1 = static_cast<VertexXYZ  *>(_vertices[1]);
        SE3 T = v0->estimate();                       // Tcw
        Vec3 pos_pixel = K_ * (cam_ext_ * (T * v1->estimate()));
        pos_pixel /= pos_pixel[2];                    // 归一化
        _error = _measurement - pos_pixel.head<2>();
    }

    // linearizeOplus()：解析雅可比 ∂e/∂ξ（右扰动版本）与 ∂e/∂p
    //   —— 形式同第7讲 BA，右扰动下 ∂e/∂ξ 的平移/旋转块需相应调整。
    virtual void linearizeOplus() override { /* 解析雅可比，略；见李代数/BA 章 */ }

    virtual bool read(std::istream &in) override { return true; }
    virtual bool write(std::ostream &out) const override { return true; }
private:
    Mat33 K_;
    SE3 cam_ext_;
};

// 小工具：cv::Point2f -> Eigen::Vector2d
inline Vec2 toVec2(const cv::Point2f &p) { return Vec2(p.x, p.y); }
```

> **[综合提示·右扰动一致性]** 投影链 `pos_pixel = K * (cam_ext * (Tcw * p_w))` 与本书"$T_{cw}$ 把世界点变到相机系、再经外参到对应目、再 $K$ 投影"一致。雅可比 `linearizeOplus` 在**右扰动**下与第 7 讲（左扰动）形式不同，综合写书时取本书右扰动版本（参见李代数章 $\partial(Tp)/\partial\boldsymbol\xi$ 的右扰动公式），并与第 7 讲交叉注明差异。

---

## 13.4 实验效果（源 §13.4）

### 运行步骤（源逐字）

1. **下载 Kitti 数据集**：`http://www.cvlibs.net/datasets/kitti/eval_odometry.php`。其 odometry 数据约 **22GB**。下载后解压得到若干视频段，**以第 0 段为例**。
2. **编译本节程序**后，在配置文件 `config/default.yaml` 中填上数据对应路径。作者机器上为：
   ```yaml
   dataset_dir: /media/xiang/Data/Dataset/Kitti/dataset/sequences/00
   ```
3. **运行**（终端输入）：
   ```txt
   bin/run_kitti_stereo
   ```
   即可看到定位输出（图 13-4）。运行期间，程序将**显示激活的关键帧和地图**，它们应**随着镜头运动不断增长和消失**（即滑窗推进的可视化）。

### 性能与已知局限（源逐字 + 量化）

- **单次运行耗时实测**：
  - **处理非关键帧时，耗时约 16 毫秒**。
  - **处理关键帧时**，由于新增了**提取特征点和寻找右图匹配**的步骤，**耗时会适当增多**。
- **内存增长问题**：由于地图目前会**存储所有关键帧和地图点**（全集 `landmarks_/keyframes_`），运行一段时间之后**将导致内存增长**。
- **优化建议（源给）**：如果读者不需要全部地图，则可以**只保留激活的部分**（即不维护全集，只留 active 窗口）。

> **[综合提示]** 16ms/非关键帧 ≈ 60+ FPS 的前端实时性；关键帧帧因提特征+右目匹配+触发后端而变慢。内存随时间线性增长是"保留全集"的代价——这是与 ORB-SLAM2"局部地图 + 全局地图但有共视/冗余剔除"相比的简化取舍。

---

## 习题（源 §习题，逐条保真）

1. 本书使用的 **C++ 技巧**你都看懂了吗？如果有不明白的地方，使用搜索引擎补习相关知识，包括**基于范围的 for 循环、智能指针、设计模式中的单例模式**，等等。
2. 考虑对本讲介绍的系统进行**优化**。例如：
   - 使用**更快的提特征点方式**（本节使用了 **GFTT**，它并不算快）；
   - 在左右匹配时使用**一维的搜索**而非二维的光流（利用双目极线已校正、视差只沿水平方向）；
   - 使用**直接法**同时估计位姿与特征点对应关系；
   - 等等。
3. \*（带星，进阶）为本节代码**添加回环检测模块**，在检测到回环时使用**位姿图进行优化**以消除累积误差。

> **[综合提示·习题映射]** 习题 1 → C++ 工程技巧（智能指针/范围 for/单例，本章已大量用）；习题 2 → 前端加速（一维极线搜索是双目标准做法；直接法见第 8 讲）；习题 3 → 回环线程（词袋见第 11 讲、位姿图见第 10 讲），正是【本章聚焦】中"（可选）回环线程"的落点——本源未实现，留作综合补充。

---

# 附录 A：与 ORB-SLAM2 架构对照（综合 agent 补充材料，**非源内容**）

> ⚠ **本源（《十四讲》第 13 讲）完全未提及 ORB-SLAM2 的架构**。下表为满足【本章聚焦】"与 ORB-SLAM2 架构对照"要求而提供的**补充材料**，基于通用 SLAM 知识，供综合 agent 写"架构对照"小节时取用。**凡本源有的列已注明源小节；ORB-SLAM2 列均为补充。**

| 维度 | 本章双目 VO（slambook2/ch13，源） | ORB-SLAM2（补充对照） |
|---|---|---|
| 线程数 | **2 个**：前端 + 后端（源 §13.2） | **3 个**：Tracking + Local Mapping + Loop Closing |
| 前端追踪 | **LK 光流**追踪左目特征（源 §13.3.2） | **ORB 描述子匹配**（恒速模型/参考关键帧/重定位三级追踪） |
| 特征 | **GFTT 角点 + LK 光流**（源习题 2 提 GFTT） | **ORB 特征**（FAST + 旋转 BRIEF），有描述子可做词袋/重定位 |
| 传感器 | **双目**（也易扩展 RGBD；源选双目因单帧可初始化，§13.1） | 单目 / 双目 / RGBD 三种 |
| 初始化 | 双目**单帧**左右目三角化（源 §13.3.2 StereoInit） | 单目需两帧对极/单应模型选择；双目/RGBD 单帧 |
| 关键帧准则 | **追踪内点数过少**即设关键帧（源 §13.3.2 第 4 条 + Track） | 多条件（距上次插入帧数、Local Mapping 空闲、追踪点数比例、共视等） |
| 局部地图 | **激活窗口 = 最新 7 关键帧**（源 §13.3.1 Map） | 由**共视图（Covisibility Graph）**动态确定局部关键帧集 |
| 后端优化 | **激活窗口局部 BA**（双目重投影边 + Huber + Schur 边缘化路标，源 §13.3.3） | Local Mapping 做**局部 BA**；Loop Closing 后做**本质图（Essential Graph）位姿图优化 + 全局 BA** |
| 外点处理 | 自适应 chi2 阈值 + RemoveObservation（源 §13.3.3） | 卡方剔除 + 关键帧/地图点冗余剔除（Culling） |
| 回环 | **无**（仅习题 3 提出，源） | **有**：DBoW2 词袋检测 + Sim(3) 求解 + 本质图优化 + 全局 BA |
| 重定位 | **无**（追踪丢失只 Reset 重新初始化，源 §13.3.2 第 5 条） | **有**：DBoW2 词袋 + PnP（EPnP）+ RANSAC |
| 地图管理 | 保留全集→内存随时间增长（源 §13.4） | 关键帧/地图点冗余剔除，控制规模 |
| 数据结构防循环引用 | `weak_ptr`（Feature→Frame/MapPoint，源 §13.3.1） | 类似（指针 + 观测列表，注意所有权） |

**对照要点小结（供综合）**：本章 VO 是 ORB-SLAM2 的"骨架简化版"——**前端追踪 + 局部 BA 后端 + 滑窗地图**三要素齐备，但**砍掉了回环、重定位、共视图/本质图、冗余剔除**，并用"光流 + GFTT"替代"ORB 描述子匹配"，用"固定 7 帧滑窗"替代"共视图动态局部地图"。理解本章足以读懂 ORB-SLAM2 的 Tracking + Local Mapping 主干。

---

# 附录 B：周边模块接口职责（源未给完整代码，从源文字 + 调用反推）

> 源明言 `Camera/Config/Dataset/Viewer/VisualOdometry` 等"交给读者自行阅读，书中只介绍核心部分"（§13.2 末）。下列职责说明依据源 §13.2 周边模块清单 + `Track`/`Optimize` 中的调用反推，供综合 agent 补全。

| 类 | 源依据 | 职责 / 关键接口（反推） |
|---|---|---|
| `Camera` | §13.2"相机类管理内外参和投影函数"；`Optimize` 中 `cam_left_->K()`、`cam_left_->pose()`、`cam_right_->pose()`；`TrackLastFrame` 中 `camera_left_->world2pixel(pos, T_cw)` | 持有内参 $K$（`Mat33 K()`）、外参 `SE3 pose()`、基线等；投影函数族：`world2pixel(p_w, T_cw)`、`camera2pixel`、`pixel2camera`、`world2camera`、`camera2world`（标准针孔模型，见第 5 讲相机模型） |
| `Config` | §13.2"配置文件管理类，从配置读参数" | **单例模式**（习题 1 点名单例）；`Config::Get<T>("key")` 读 `default.yaml`（如 `dataset_dir`、特征数、关键帧阈值等） |
| `Dataset` | §13.2"按 Kitti 格式读图像" | 读 Kitti `sequences/00`：相机标定（`calib.txt` → 左右目 $K$、基线）、按帧号读左右图、`NextFrame()` 返回 `Frame::Ptr` |
| `Viewer` | §13.2"可视化模块"；`Track` 中 `viewer_->AddCurrentFrame(current_frame_)` | 独立线程 + Pangolin 显示轨迹/激活关键帧/激活地图点；`AddCurrentFrame`、`UpdateMap` |
| `VisualOdometry` | §13.2 总成（源未命名但框架隐含） | 系统总成：`Init()` 读配置/建数据集/相机/前端/后端/地图/可视化并互相 `Set*`；`Run()`/`Step()` 主循环 = `Dataset::NextFrame()` → `Frontend::AddFrame()` |
| `Frontend` 未给体函数 | §13.3.2 文字逻辑 | `StereoInit()`、`Reset()`、`EstimateCurrentPose()`、`InsertKeyframe()`、`DetectFeatures()`、`FindFeaturesInRight()`、`TriangulateNewPoints()`、`BuildInitMap()`、`SetMap/SetBackend/SetViewer/SetCameras` |
| `Map` 未给体函数 | §13.3.1 文字逻辑 | `InsertKeyFrame()`（含 active 维护 + 超 7 调 `RemoveOldKeyframe`）、`InsertMapPoint()`、`RemoveOldKeyframe()`、`CleanMap()` |
| `MapPoint::RemoveObservation` | §13.3.1 + `Optimize` 调用 | 从 `observations_` 链表删指定特征 + `observed_times_--` + 解关联（加锁） |

---

# OCR 修正说明（逐处）

本源 **70% 以上是 C++ 代码**，OCR 对代码破坏远重于文字。下表列出所有**已发现并修正**的 OCR 错误（按出现顺序）。修正依据：C++ 语法 + g2o/Eigen/Sophus/OpenCV API 惯例 + 本章上下文逻辑自洽。

### A. 代码类（影响可编译性，必须修正）

| # | 源 OCR 原文（错） | 修正后（对） | 位置 / 依据 |
|---|---|---|---|
| 1 | `std::shared_ptr<Map_;`（成员被截断，缺类型名与变量名） | `std::shared_ptr<Map> map_;` | backend.h 私有成员；由 `SetMap(...){ map_ = map; }` 及 `map_->GetActiveKeyFrames()` 反推变量名为 `map_` |
| 2 | `g2o::make_unique<BlockSolverType>(\n g2o::make_unique<LinearSolverType>());`（仅 1 个 `)`，缺 2 个闭合括号，`OptimizationAlgorithmLevenberg(` 未闭合） | `g2o::make_unique<BlockSolverType>(\n g2o::make_unique<LinearSolverType>()));` 且最外层 `auto solver = new g2o::OptimizationAlgorithmLevenberg(... );` 闭合 | backend.cpp Optimize；g2o 求解器标准嵌套构造写法 |
| 3 | `std::map<unsigned long, VertexPose => vertices;` | `std::map<unsigned long, VertexPose *> vertices;` | `=>` 是 OCR 把 `*>` 认错；vertices 存指针（后续 `new VertexPose()`） |
| 4 | `std::map<unsigned long, VertexXYZ => vertices_landmarks;` | `std::map<unsigned long, VertexXYZ *> vertices_landmarks;` | 同上 |
| 5 | 关键帧 `for` 循环大括号缺失/错位（`vertices.insert(...)` 后续 `// 路标顶点` 段被并入循环内） | 在 `vertices.insert({kf->keyframe_id_, vertex_pose});` 后补 `}` 闭合关键帧 for；其后路标顶点声明、K/外参、edges 段移到循环外 | 逻辑：路标顶点声明、内外参获取只需一次，不应在每关键帧循环内 |
| 6 | 外点剔除 `while (iteration < 5) {...}` 后**多出一对孤立 `}`**（OCR 把代码块边界 643→645 行错切） | 按 while 体正确闭合（while 内含统计 for + inlier_ratio 判断 + chi2_th*=2），while 后紧接最终外点标记 for | backend.cpp；分页 OCR 在 642/644 行把 `}` 错位 |
| 7 | `// Set pose and lanmark position` | `// Set pose and landmark position` | 注释拼写错 `lanmark`→`landmark` |
| 8 | `MapPoint` 中 `void SetPos(const Vec3 &pos) {` 函数体被 OCR 断到下一段（`std::unique_lock... pos_ = pos; };`，且多一个 `;`） | 接合为完整 `void SetPos(const Vec3 &pos){ std::unique_lock<std::mutex> lck(data_mutex_); pos_ = pos; }`（去掉多余分号） | mappoint.h；分页断裂 |
| 9 | `Frame`/`Feature`/`Map` 三类均被 OCR 切成两段（成员区 + 方法区分页），段间衔接处缩进/大括号丢失 | 合并为完整类定义（见 §13.3.1 各代码块），补齐 `public:`/成员/大括号 | frame.h / feature.h / map.h；分页断裂，按 struct/class 语法接合 |

### B. 文字类（不影响理解，标注存疑）

| # | 源 OCR | 说明 |
|---|---|---|
| 10 | `TrackLastFrame` 注释 `// use LK flow to estimate points in the right image` | **注释疑有误**（本函数追踪的是**左目**上一帧→当前帧，非右图）。代码本身正确（用 `left_img_`）。疑为 ch13 源码中复制 `FindFeaturesInRight` 注释的遗留，非 OCR 错；标 `[OCR?]` 提示综合时核对原仓库。 |
| 11 | `active_keyframes_` 注释 `// all key-frames`（与 `keyframes_` 注释重复） | 应为 `// active key-frames`；OCR 复制了上一行注释。标 `[OCR?]`。 |
| 12 | 图 13-1/13-2/13-3/13-4 均为图片链接（cdn URL），OCR 不可读 | 图内容已由源**正文文字**重建（见 §13.1/§13.2 各图说明）；图 13-3（算法框架）与图 13-4（运行截图）的语义在正文中完整给出。 |
| 13 | 脚注标记 `$^{①}$`（§13.1 Minecraft、§13.3.1 Feature 循环引用）正文未给脚注内容 | 循环引用脚注内容已由领域知识在 §13.3.1 (2) 补全（标"抽取补全"）；Minecraft 脚注疑为游戏出处说明，无技术内容。 |

### C. 数学/符号类

本章数学符号极少。`chi2_th = 5.991`（$\chi^2_{2,0.95}$）、`Mat22::Identity()`（$\Omega=I$）、`BlockSolver_6_3`（6+3 维）均未见 OCR 错，与领域常识一致，已在正文注解其数学含义。`toVec2`、`world2pixel` 等函数名拼写正确。

---

# 给综合 agent 的写章建议（非源内容，收尾）

1. **本章是"工程整合章"**，成书时宜以**系统架构图（图 13-3 重画）→ 数据结构（4 类）→ 前端（状态机 + 光流 + 关键帧）→ 后端（局部 BA + 滑窗 + 外点剔除）→ 实验 → ORB-SLAM2 对照**为主线。
2. **数学与代码打通**：源代码本身不含 BA 目标函数公式，§13.3.3 末已补出统一记号下的目标函数式与 g2o 顶点/边骨架（右扰动），成书时把"激活窗口局部 BA"与第 7 讲 BA、李代数右扰动雅可比交叉引用。
3. **右扰动一致性是唯一需要主动改写的地方**：本章 `Optimize` 复用第 7 讲 BA（《十四讲》原版左扰动）；本书统一右扰动，须改 `VertexPose::oplusImpl` 为右乘并相应改 `EdgeProjection::linearizeOplus`。已在 §记号约定 + §13.3.3(4) 双处标注。
4. **回环线程**（习题 3 / 【本章聚焦】可选项）本源未实现，从 `loop_closure__sf14_11.md`/`loop_closure__dbow.md`（第 11 讲词袋）+ 第 10 讲位姿图综合，作为本章的"扩展"小节。
5. **RGBD 变体**：本源只演示双目；成书可加一段"RGBD 前端 = 把右目三角化换成读深度图"（深度直接给 3D 观测），数据结构/后端不变。
