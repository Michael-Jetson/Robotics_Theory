# 抽取留痕：《视觉SLAM十四讲》第8章 视觉里程计2（光流与直接法）

> 本文件是项目内部「抽取留痕」，目标是把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章。**禁摘要、禁凝练**：每一步推导、每一道例题/数值例、每一条定义/定理、每一段代码/伪码、每一张表均完整记录。
>
> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/08_视觉里程计2.md`（共 946 行，已完整读取）
> **源章节**：《视觉SLAM十四讲》（第二版，高翔等）第 8 讲「视觉里程计 2」

---

## 0. 重要范围说明（给综合 agent 的提示）

本任务【本章聚焦】清单里列的多数主题（**特征点法 ORB/匹配、对极几何本质/基础/单应矩阵、8 点法完整推导、三角化、PnP(DLT/EPnP/BA)、ICP(3D-3D)**）**不在本源文件内**。它们属于《视觉SLAM十四讲》**第 7 讲「视觉里程计 1」**。本第 8 讲在多处明确地把这些内容"指回第 7 讲"：

- §8.3.3 原文："我们可以通过光流跟踪的特征点，用 PnP、ICP 或对极几何来估计相机运动，这些方法在第 7 讲中介绍过，这里不再讨论。"
- §8.4.1 原文："使用特征点法估计相机运动时…通过最小化重投影误差（Reprojection error）优化相机运动。"（仅作回顾，无推导）
- §8.5.1 原文："基于特征点的深度恢复（即三角化）已经在第 7 讲介绍过…"

因此**本第 8 讲实际覆盖的内容**为：
1. 直接法的引出（§8.1）；
2. 2D 光流 / Lucas-Kanade（LK）光流完整推导（§8.2）；
3. LK 光流实践：OpenCV 调用、高斯牛顿单层光流、反向光流、图像金字塔多层光流（§8.3）；
4. 直接法的推导（光度误差、李代数左扰动雅可比链式分解）（§8.4.1）；
5. 直接法的讨论与分类（稀疏/半稠密/稠密）（§8.4.2）；
6. 直接法实践：单层、多层、结果讨论（含一次迭代的数值例）（§8.5）；
7. 直接法优缺点总结（§8.5.4）+ 习题（§习题）。

本抽取把本第 8 讲全量记录。**第 7 讲（对极几何/8点法/三角化/PnP/ICP）需另行抽取 `07_视觉里程计1.md`**，不在本文件范围。

---

## 记号约定（本源 vs 本书统一约定）

| 项目 | 本源（《十四讲》第 8 讲）记号 | 含义 | 本书统一约定 | 差异/转换提示 |
|---|---|---|---|---|
| 旋转矩阵 | **R** | 旋转 $R\in SO(3)$ | $R\in SO(3)$ | 一致（用 R，不用 C） |
| 位姿/变换 | **T**（李群 SE(3)），$\boldsymbol T\in SE(3)$ | 第一相机到第二相机的相对位姿 | $T\in SE(3)$ | 一致；本源代码中写作 `Sophus::SE3d`，变量名 `T21 / T_cur_ref`（"cur←ref"含义） |
| 平移 | **t** | 平移向量 | $t$ | 一致 |
| 李代数扰动 | **δξ（左扰动）** | $\partial e/\partial T$ 用**左扰动模型**展开，更新为 $T \leftarrow \exp(\delta\xi^\wedge)\,T$ | 本书**以右扰动为主** | ⚠️ **关键差异**：本源直接法雅可比 §(8.15)(8.17)(8.18) 用的是**左扰动**。代码中 `T21 = Sophus::SE3d::exp(update) * T21;` 也是左乘更新，证实左扰动。综合到本书（右扰动为主）时，雅可比中 $\partial q/\partial\delta\xi=[I,\,-q^\wedge]$（左扰动形式）需要按右扰动重新推导（右扰动会得到不同的 $\xi^\wedge$ 块）。 |
| 扰动向量排序 | 本源代码 `Vector6d`、`Matrix26d`，雅可比 §(8.18) 列序为 **[平移 3 列 | 旋转 3 列]**，即先平移后旋转 | $\xi=[\rho;\phi]$（平移在前、旋转在后） | $\xi=[\rho;\phi]$ | **一致**：§(8.18) 前 3 列对应平移（$f_x/Z,\,0,\,-f_xX/Z^2$ …），后 3 列对应旋转，与 $\xi=[\rho;\phi]$ 排序相同。 |
| $\wedge$ 算子 | $q^\wedge$ | 三维向量到反对称矩阵（hat） | $\cdot^\wedge$ | 一致 |
| 相机内参 | **K**；分量 $f_x,f_y,c_x,c_y$ | 内参矩阵与焦距/主点 | 同 | 一致 |
| 图像/灰度 | $\boldsymbol I(x,y,t)$，$I_1,I_2$ | 图像看作位置与时间的函数，值域为灰度 | 同 | 一致 |
| 像素坐标 | $p_1,p_2$；$u=[u,v]^\top$ | 像素齐次坐标 | 同 | 一致 |
| 深度 | $Z_1,Z_2$ | 点在两相机系下的深度 | 同 | 一致 |
| 光流速度 | $u,v$（注意与像素坐标 $u$ 同名，靠上下文区分） | $dx/dt,\,dy/dt$ | — | ⚠️ 同一字母 $u$ 在 §8.2 既指 x 方向像素运动速度、又在 §8.4 指像素坐标向量，综合时需区分。 |
| 图像梯度 | $I_x,I_y$ | $\partial I/\partial x,\ \partial I/\partial y$ | 同 | 一致 |
| 时间梯度 | $I_t$ | $\partial I/\partial t$ | 同 | 一致 |
| 误差 | $e$（直接法为标量光度误差） | $e=I_1(p_1)-I_2(p_2)$ | 同 | 一致 |
| 四元数 | 本章未使用四元数 | — | Hamilton | 无相关内容 |
| 协方差 | 本章未使用协方差字母 | — | — | 无相关内容 |

**其他记号备注**：
- 本源无定理/引理/命题编号体系（属于教材式叙述），公式编号为 (8.1)~(8.19)，本文逐一保留。
- "由粗至精"= Coarse-to-fine；"反向光流"= Inverse optical flow；"重投影误差"= Reprojection error；"光度误差"= Photometric error；"灰度不变假设"= brightness/grayscale constancy assumption；"归一化相关性"= Normalized Cross Correlation (NCC)。
- 代码常量：`half_patch_size`、`iterations`、`pyramids`、`pyramid_scale`、`scales[]`、`baseline`、`fx/fy/cx/cy`。

---

## 章首：主要目标与三个引入公式（源 第8讲开篇）

**主要目标**（原文逐条）：
1. 理解光流法跟踪特征点的原理。
2. 理解直接法是如何估计相机位姿的。
3. 实现多层直接法的计算。

引言（原文）："直接法是视觉里程计的另一个主要分支，它与特征点法有很大不同。虽然它还没有成为现在视觉里程计中的主流，但经过近几年的发展，直接法在一定程度上已经能和特征点法平分秋色。本讲，我们将介绍直接法的原理，并实现直接法中的核心部分。"

章首给出的两个"预告"公式（原文照录，注意这是开篇示意，正式推导在 §8.4）：

$$
\min \left\| I _ {1} (p) - I _ {2} (k (R p + t)) \right\| _ {2}
$$

$$
J = \frac {\partial I}{\partial n} \frac {\partial n}{\partial q} \frac {\partial q}{\partial z}
$$

> 抽取注：上式中 $k$ 应为相机内参 $K$ 的投影映射、$n$/$q$/$z$ 为中间变量的占位记号，与 §8.4.1 中正式的 $\partial e/\partial T=\frac{\partial I_2}{\partial u}\frac{\partial u}{\partial q}\frac{\partial q}{\partial \delta\xi}$ 对应（章首记号较随意，正式版见 §8.4.1）。

---

## 8.1 直接法的引出

第 7 讲介绍了使用特征点估计相机运动的方法。尽管特征点法在视觉里程计中占据主流地位，研究者们认识到它至少有以下几个缺点（原文逐条）：

1. **关键点的提取与描述子的计算非常耗时。** 实践中，SIFT 目前在 CPU 上是无法实时计算的，而 ORB 也需要近 **20 毫秒**的计算时长。如果整个 SLAM 以 **30 毫秒/帧**的速度运行，那么一大半时间都将花在计算特征点上。
2. **使用特征点时，忽略了除特征点以外的所有信息。** 一幅图像有几十万个像素，而特征点只有几百个。只使用特征点丢弃了大部分可能有用的图像信息。
3. **相机有时会运动到特征缺失的地方**，这些地方往往没有明显的纹理信息。例如，有时我们会面对一堵白墙，或者一个空荡荡的走廊。这些场景下特征点数量会明显减少，可能找不到足够的匹配点来计算相机运动。

**克服缺点的几种思路**（原文逐条）：
- **思路一**：保留特征点，但只计算关键点，不计算描述子。同时，使用**光流法（Optical Flow）**跟踪特征点的运动。这样可以回避计算和匹配描述子带来的时间，而光流本身的计算时间要小于描述子的计算与匹配。
- **思路二**：只计算关键点，不计算描述子。同时，使用**直接法（Direct Method）**计算特征点在下一时刻图像中的位置。这同样可以跳过描述子的计算过程，也省去了光流的计算时间。

**两种方法的区别**（原文）：
- 第一种方法仍然使用特征点，只是把匹配描述子替换成了光流跟踪，**估计相机运动时仍使用对极几何、PnP 或 ICP 算法**。这依然会要求提取到的关键点具有可区别性，即需要提到角点。
- 而在**直接法**中，根据图像的像素灰度信息**同时估计相机运动和点的投影**，不要求提取到的点必须为角点。后文中将看到，它们甚至可以是**随机的选点**。

**特征点法 vs 直接法的核心对比**（原文）：
- 使用**特征点法**估计相机运动时，把特征点看作固定在三维空间的不动点。根据它们在相机中的投影位置，通过最小化**重投影误差（Reprojection error）**优化相机运动。在这个过程中，需要精确地知道空间点在两个相机中投影后的像素位置——这也就是要对特征进行匹配或跟踪的原因。计算、匹配特征需要付出大量的计算量。
- 相对地，在**直接法**中，**并不需要知道点与点之间的对应关系**，而是通过**最小化光度误差（Photometric error）**来求得它们。

**直接法的意义**（原文）："直接法根据像素的亮度信息估计相机的运动，可以完全不用计算关键点和描述子，于是，既避免了特征的计算时间，也避免了特征缺失的情况。只要场景中存在**明暗变化**（可以是渐变，不形成局部的图像梯度），直接法就能工作。根据使用像素的数量，直接法分为**稀疏、稠密和半稠密**三种。与特征点法只能重构稀疏特征点（稀疏地图）相比，直接法还具有**恢复稠密或半稠密结构**的能力。"

**历史脉络**（原文，含参考文献编号）："历史上，早期也有对直接法的使用 $^{[67]}$。随着一些使用直接法的开源项目的出现（如 **SVO** $^{[68]}$、**LSD-SLAM** $^{[69]}$、**DSO** $^{[70]}$ 等），它们逐渐走上主流舞台，成为视觉里程计算法中重要的一部分。"

---

## 8.2 2D 光流

引言（原文）："直接法是从光流演变而来的。它们非常相似，具有相同的假设条件。**光流描述了像素在图像中的运动，而直接法则附带着一个相机运动模型。**为了说明直接法，我们先来介绍光流。"

光流定义与分类（原文）：
- 光流是一种描述**像素随时间在图像之间运动**的方法（图 8-1）。随着时间的流逝，同一个像素会在图像中运动，希望追踪它的运动过程。
- **稀疏光流**：计算部分像素运动，以 **Lucas-Kanade 光流** $^{[71]}$ 为代表，可在 SLAM 中用于跟踪特征点位置。
- **稠密光流**：计算所有像素，以 **Horn-Schunck 光流** $^{[72]}$ 为代表。
- 本节主要介绍 **Lucas-Kanade 光流**，也称为 **LK 光流**。

图 8-1 注解（原文）：**灰度不变假设**：$I(x_{1},y_{1},t_{1})=I(x_{2},y_{2},t_{2})=I(x_{3},y_{3},t_{3})$（同一像素在不同时刻图像中灰度相同）。

### Lucas-Kanade 光流（完整推导）

**设定**（原文）：在 LK 光流中，认为来自相机的图像是随时间变化的，图像看作时间的函数 $I(t)$。一个在 $t$ 时刻、位于 $(x,y)$ 处的像素，其灰度写成

$$
\boldsymbol {I} (x, y, t).
$$

这种方式把图像看成关于位置与时间的函数，值域是图像中像素的灰度。现在考虑某个**固定的空间点**，它在 $t$ 时刻的像素坐标为 $(x,y)$。由于相机的运动，它的图像坐标将发生变化。希望估计这个空间点在其他时刻图像中的位置。

**【基本假设】灰度不变假设**：同一个空间点的像素灰度值，在各个图像中是固定不变的。

**第 1 步：灰度不变方程。** 对于 $t$ 时刻位于 $(x,y)$ 处的像素，设 $t+\mathrm{d}t$ 时刻它运动到 $(x+\mathrm{d}x,\,y+\mathrm{d}y)$ 处。由于灰度不变：

$$
\boldsymbol {I} (x + \mathrm{d} x, y + \mathrm{d} y, t + \mathrm{d} t) = \boldsymbol {I} (x, y, t). \tag{8.1}
$$

**【关于假设强度的讨论】**（原文）："注意灰度不变假设是一个**很强的假设**，实际中很可能不成立。事实上，由于物体的材质不同，像素会出现高光和阴影部分；有时，相机会自动调整曝光参数，使得图像整体变亮或变暗。这时灰度不变假设都是不成立的，因此光流的结果也不一定可靠。然而，从另一方面来说，所有算法都是在一定假设下工作的。如果我们什么假设都不做，就没法设计实用的算法。所以，让我们暂且认为该假设成立，看看如何计算像素的运动。"

**第 2 步：一阶泰勒展开。** 对式 (8.1) 左边进行泰勒展开，保留一阶项，得

$$
\boldsymbol {I} (x + \mathrm{d} x, y + \mathrm{d} y, t + \mathrm{d} t) \approx \boldsymbol {I} (x, y, t) + \frac {\partial \boldsymbol {I}}{\partial x} \mathrm{d} x + \frac {\partial \boldsymbol {I}}{\partial y} \mathrm{d} y + \frac {\partial \boldsymbol {I}}{\partial t} \mathrm{d} t. \tag{8.2}
$$

**第 3 步：代入灰度不变。** 因为假设了灰度不变，下一个时刻的灰度等于之前的灰度（即式 (8.2) 左边 = $I(x,y,t)$），从而消去 $I(x,y,t)$：

$$
\frac {\partial \boldsymbol {I}}{\partial x} \mathrm{d} x + \frac {\partial \boldsymbol {I}}{\partial y} \mathrm{d} y + \frac {\partial \boldsymbol {I}}{\partial t} \mathrm{d} t = 0. \tag{8.3}
$$

**第 4 步：两边除以 dt。**

$$
\frac {\partial \boldsymbol {I}}{\partial x} \frac {\mathrm{d} x}{\mathrm{d} t} + \frac {\partial \boldsymbol {I}}{\partial y} \frac {\mathrm{d} y}{\mathrm{d} t} = - \frac {\partial \boldsymbol {I}}{\partial t}. \tag{8.4}
$$

**第 5 步：记号替换并写成矩阵形式。** 其中 $\mathrm{d}x/\mathrm{d}t$ 为像素在 $x$ 轴上的运动速度，$\mathrm{d}y/\mathrm{d}t$ 为 $y$ 轴上的速度，记为 $u,v$。同时 $\partial I/\partial x$ 为图像在该点处 $x$ 方向的梯度，$\partial I/\partial y$ 为 $y$ 方向的梯度，记为 $I_x,I_y$。把图像灰度对时间的变化量记为 $I_t$。写成矩阵形式：

$$
\left[ \begin{array}{l l} \boldsymbol {I} _ {x} & \boldsymbol {I} _ {y} \end{array} \right] \left[ \begin{array}{l} u \\ v \end{array} \right] = - \boldsymbol {I} _ {t}. \tag{8.5}
$$

**第 6 步：欠定问题，引入窗口约束。** 想计算的是像素的运动 $u,v$，但式 (8.5) 是带有两个变量的一次方程，仅凭它**无法**计算出 $u,v$。因此必须引入额外的约束。**在 LK 光流中，假设某一个窗口内的像素具有相同的运动。**

考虑一个大小为 $w\times w$ 的窗口，含有 $w^2$ 数量的像素。该窗口内像素具有同样的运动，因此共有 $w^2$ 个方程：

$$
\left[ \begin{array}{l l} \boldsymbol {I} _ {x} & \boldsymbol {I} _ {y} \end{array} \right] _ {k} \left[ \begin{array}{l} u \\ v \end{array} \right] = - \boldsymbol {I} _ {t k}, \quad k = 1, \dots , w ^ {2}. \tag{8.6}
$$

**第 7 步：堆叠成超定方程。** 记

$$
\boldsymbol {A} = \left[ \begin{array}{c} {[ I _ {x}, I _ {y} ] _ {1}} \\ {\vdots} \\ {[ I _ {x}, I _ {y} ] _ {k}} \end{array} \right], \quad \boldsymbol {b} = \left[ \begin{array}{c} {I _ {t 1}} \\ {\vdots} \\ {I _ {t k}} \end{array} \right]. \tag{8.7}
$$

于是整个方程为

$$
\boldsymbol {A} \left[ \begin{array}{l} u \\ v \end{array} \right] = - \boldsymbol {b}. \tag{8.8}
$$

**第 8 步：最小二乘解。** 这是一个关于 $u,v$ 的**超定线性方程**，传统解法是求最小二乘解：

$$
\left[ \begin{array}{l} u \\ v \end{array} \right] ^ {*} = - \left(\boldsymbol {A} ^ {\mathrm{T}} \boldsymbol {A}\right) ^ {- 1} \boldsymbol {A} ^ {\mathrm{T}} \boldsymbol {b}. \tag{8.9}
$$

**结论与说明**（原文）："这样就得到了像素在图像间的运动速度 $u,v$。当 $t$ 取离散的时刻而不是连续时间时，可以估计某块像素在若干个图像中出现的位置。由于**像素梯度仅在局部有效**，所以如果一次迭代不够好，会**多迭代几次**这个方程。在 SLAM 中，LK 光流常被用来跟踪角点的运动。"

> 抽取注（雅可比/梯度关键）：式 (8.9) 即标准 LK 闭式解，$A^\top A$ 为 $2\times2$ 的二阶矩（结构张量/Harris 矩阵雏形）。当窗口内梯度退化（如纯色块），$A^\top A$ 不可逆 → 光流不可解，对应后文代码中 "black or white patch and H is irreversible / update is nan" 的情形。

---

## 8.3 实践：LK 光流

### 8.3.1 使用 LK 光流（OpenCV）

实践设定（原文）：使用两张来自 **Euroc 数据集**的示例图像，在第一张图像中提取角点，然后用光流追踪它们在第二张图像中的位置。首先使用 OpenCV 中的 LK 光流。

**代码：`slambook2/ch8/optical_flow.cpp`（片段，OpenCV 验证）**
```cpp
// use opencv's flow for validation
vector<Point2f> pt1, pt2;
for (auto &kp: kp1) pt1.push_back(kp.pt);
vector<uchar> status;
vector<float> error;
cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
```

说明（原文）："OpenCV 的光流在使用上十分简单，只需调用 `cv::calcOpticalFlowPyrLK` 函数，提供前后两张图像及对应的特征点，即可得到追踪后的点，以及各点的状态、误差。我们可以根据 `status` 变量是否为 1 来确定对应的点是否被正确追踪到。该函数还有一些可选的参数，但在演示中只使用默认参数。"（省略其他提取特征、画出结果的代码。）

### 8.3.2 用高斯牛顿法实现光流

#### 单层光流

引言（原文）："光流也可以看成一个**优化问题**：通过最小化灰度误差估计最优的像素偏移。所以，类似于之前实现的各种高斯牛顿法，现在也来实现一个基于高斯牛顿法的光流。"

**代码：`slambook2/ch8/optical_flow.cpp`（片段，OpticalFlowTracker 类与单层入口函数）**
```cpp
class OpticalFlowTracker {
public:
    OpticalFlowTracker(
    const Mat &img1_,
    const Mat &img2_,
    const vector<KeyPoint> &kp1_,
    vector<KeyPoint> &kp2_,
    vector<bool> &success_,
    bool inverse_ = true, bool has_initial_ = false) :
    img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
    has_initial(has_initial_) {}

    void calculateOpticalFlow(const Range &range);

private:
    const Mat &img1;
    const Mat &img2;
    const vector<KeyPoint> &kp1;
    vector<KeyPoint> &kp2;
    vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;
};

void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse, bool has_initial) {
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    parallel_for_(Range(0, kp1.size()),
    std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, placeholders::_1));
}

void OpticalFlowTracker::calculateOpticalFlow(const Range &range) {
    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    for (size_t i = range.start; i < range.end; i++) {
    auto kp = kp1[i];
    double dx = 0, dy = 0; // dx,dy need to be estimated
    if (has_initial) {
    dx = kp2[i].pt.x - kp.pt.x;
    dy = kp2[i].pt.y - kp.pt.y;
    }

    double cost = 0, lastCost = 0;
    bool succ = true; // indicate if this point succeeded

    // Gauss-Newton iterations
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero(); // hessian
    Eigen::Vector2d b = Eigen::Vector2d::Zero(); // bias
    Eigen::Vector2d J; // jacobian
    for (int iter = 0; iter < iterations; iter++) {
    if (inverse == false) {
    H = Eigen::Matrix2d::Zero();
    b = Eigen::Vector2d::Zero();
    } else {
    // only reset b
    b = Eigen::Vector2d::Zero();
    }

    cost = 0;

    // compute cost and jacobian
    for (int x = -half_patch_size; x < half_patch_size; x++)
    for (int y = -half_patch_size; y < half_patch_size; y++) {
    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                   GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);  // Jacobian
    if (inverse == false) {
    J = -1.0 * Eigen::Vector2d(
    0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
           GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
    0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
           GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
    );
    } else if (iter == 0) {
    // in inverse mode, J keeps same for all iterations
    // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
    J = -1.0 * Eigen::Vector2d(
    0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
           GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
    0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
           GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1))
    );
    }
    // compute H, b and set cost;
    b += -error * J;
    cost += error * error;
    if (inverse == false || iter == 0) {
        // also update H
        H += J * J.transpose();
    }
    }  // end of patch loops

    // compute update
    Eigen::Vector2d update = H.ldlt().solve(b);

    if (std::isnan(update[0])) {
        // sometimes occurred when we have a black or white patch and H is irreversible
        cout << "update is nan" << endl;
        succ = false;
        break;
    }

    if (iter > 0 && cost > lastCost) {
        break;
    }

    // update dx, dy
    dx += update[0];
    dy += update[1];
    lastCost = cost;
    succ = true;

    if (update.norm() < 1e-2) {
        // converge
        break;
    }
    }  // end of GN iterations

    success[i] = succ;

    // set kp2
    kp2[i].pt = kp.pt + Point2f(dx, dy);
    }
}
```

> 抽取注：源 md 把同一函数分成多个代码块（rust/cpp/txt 高亮标记不一致，且括号缩进有断裂），上面已按逻辑合并为一段完整函数（保留全部语句、注释、判据），仅补回因分块丢失的少量花括号/缩进以保证可读，未改动任何计算语句。`GetPixelValue` 为双线性插值取灰度的辅助函数（源未在本节列出其定义）。

**并行实现说明**（原文）："我们在 `OpticalFlowSingleLevel` 函数中实现了单层光流函数，其中调用了 `cv::parallel_for_` 并行调用 `OpticalFlowTracker::calculateOpticalFlow`，该函数计算指定范围内特征点的光流。这个并行 for 循环内部是 Intel **tbb** 库实现的，我们只需按照其接口，将函数本体定义出来，然后将函数作为 `std::function` 对象传递给它。"

**单层光流求解的优化问题**（原文）：在具体函数实现中（即 `calculateOpticalFlow`），求解这样一个问题：

$$
\min _ {\Delta x, \Delta y} \| I _ {1} (x, y) - I _ {2} (x + \Delta x, y + \Delta y) \| _ {2} ^ {2}. \tag{8.10}
$$

**雅可比与反向光流**（原文）："因此，残差为括号内部的部分，对应的**雅可比为第二个图像在 $x+\Delta x,\,y+\Delta y$ 处的梯度**。此外，根据参考文献 [73]，这里的梯度也可以用**第一个图像的梯度 $I_1(x,y)$ 来代替**。这种代替的方法称为**反向（Inverse）光流法**。在反向光流中，$I_1(x,y)$ 的梯度是保持不变的，所以可以在第一次迭代时保留计算的结果，在后续迭代中使用。**当雅可比不变时，$H$ 矩阵不变，每次迭代只需计算残差**，这可以节省一部分计算量。"

> 抽取注（高斯牛顿对应关系，结合代码）：
> - 残差 $e=I_1(x+x',y+y')-I_2(x+x'+dx,\,y+y'+dy)$（patch 内逐像素，$x',y'\in[-4,4)$）。
> - 雅可比 $J=-\nabla I$（代码中 `J = -1.0 * (梯度)`）。正向法用 $I_2$ 在当前位置的梯度（每次迭代重算）；反向法用 $I_1$ 在原位置的梯度（仅第 0 次迭代算一次）。
> - $H=\sum J J^\top$（$2\times2$），$b=\sum(-e\,J)$，增量 $\Delta=[ \Delta x,\Delta y]=H^{-1}b$（代码 `H.ldlt().solve(b)`）。
> - 收敛/退出判据：`isnan(update)`→失败退出；`iter>0 && cost>lastCost`→代价上升退出；`update.norm()<1e-2`→收敛退出；最多 `iterations=10` 次。

#### 多层光流（图像金字塔，Coarse-to-fine）

**动机**（原文）："我们把光流写成了优化问题，就必须假设优化的初始值靠近最优值，才能在一定程度上保障算法的收敛。如果相机运动较快，两张图像差异较明显，那么单层图像光流法容易达到一个**局部极小值**。这种情况可以通过引入**图像金字塔**来改善。"

**图像金字塔**（原文）："图像金字塔是指对同一个图像进行缩放，得到不同分辨率下的图像（图 8-2）。以原始图像作为金字塔底层，每往上一层，就对下层图像进行一定倍率的缩放，就得到了一个金字塔。"

**Coarse-to-fine 流程**（原文）："然后，在计算光流时，**先从顶层的图像开始计算，然后把上一层的追踪结果，作为下一层光流的初始值**。由于上层的图像相对粗糙，所以这个过程也称为**由粗至精（Coarse-to-fine）**的光流，也是实用光流法的通常流程。"

**【数值例：金字塔对大位移的缓解】**（原文照录）："由粗至精的好处在于，当原始图像的像素运动较大时，在金字塔顶层的图像看来，运动仍然在一个很小范围内。例如，原始图像的特征点运动了 **20 个像素**，很容易由于图像非凸性导致优化困在极小值里。但现在假设有缩放倍率为 **0.5 倍**的金字塔，那么往上两层图像里，像素运动就只有 **5 个像素**了（20 × 0.5 × 0.5 = 5），这时结果就明显好于直接在原始图像上优化。"

**代码：`slambook2/ch8/optical_flow.cpp`（片段，多层光流）**
```cpp
void OpticalFlowMultiLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse) {
    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++) {
    if (i == 0) {
    pyr1.push_back(img1);
    pyr2.push_back(img2);
    } else {
    Mat img1_pyr, img2_pyr;
    cv::resize(pyr1[i - 1], img1_pyr,
    cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
    cv::resize(pyr2[i - 1], img2_pyr,
    cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
    pyr1.push_back(img1_pyr);
    pyr2.push_back(img2_pyr);
    }
    }

    // coarse-to-fine LK tracking in pyramids
    vector<KeyPoint> kp1_pyr, kp2_pyr;
    for (auto &kp:kp1) {
    auto kp_top = kp;
    kp_top.pt *= scales[pyramids - 1];
    kp1_pyr.push_back(kp_top);
    kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--) {
    // from coarse to fine
    success.clear();
    OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);

    if (level > 0) {
    for (auto &kp:kp1_pyr)
    kp.pt /= pyramid_scale;
    for (auto &kp:kp2_pyr)
    kp.pt /= pyramid_scale;
    }
    }

    for (auto &kp:kp2_pyr)
    kp2.push_back(kp);
}
```

说明（原文）："这段代码构造了一个**四层的、倍率为 0.5** 的金字塔，并调用单层光流函数实现了多层光流。在主函数中，分别对两张图像测试了 OpenCV 的光流、单层光流和多层光流的表现，计算了它们的运行时间。"

**【数值例：运行时间终端输出】**（原文照录）：
```text
./build/optical_flow
build pyramid time: 0.000150349
track pyr 3 cost time: 0.000304633
track pyr 2 cost time: 0.000392889
track pyr 1 cost time: 0.000382347
track pyr 0 cost time: 0.000375099
optical flow by gauss-newton: 0.00189268
optical flow by opencv: 0.00220134
```

结果讨论（原文）："从运行时间上看，**多层光流法的耗时和 OpenCV 的大致相当**。由于并行化程序在每次运行时的表现不尽相同，在读者机器上这些数字不会精确相同。光流的对比图如图 8-3 所示。从结果图上看，**多层光流与 OpenCV 的效果相当，单层光流要明显弱于多层光流**。"（图 8-3 含三张子图：单层光流 / 多层光流 / OpenCV 光流。）

### 8.3.3 光流实践小结

原文逐条：
- "LK 光流跟踪能够**直接得到特征点的对应关系**。这个对应关系就像是描述子的匹配，只是光流对图像的连续性和光照稳定性要求更高一些。我们可以通过光流跟踪的特征点，用 **PnP、ICP 或对极几何**来估计相机运动，这些方法在**第 7 讲**中介绍过，这里不再讨论。"
- "从运行时间上来看，演示实验大约有 **230 个特征点**，OpenCV 和多层光流需要大约 **2 毫秒**完成追踪（笔者用的 CPU 是 **Intel I7-8550U**），这实际上是相当快的。如果前面使用 **FAST** 这样的关键点，那么整个光流计算可以做到 **5 毫秒**左右，相比于特征匹配来说算是非常快了。不过，如果角点提的位置不好，光流也容易跟丢或给出错误的结果，这就需要后续算法拥有一定的**异常值去除机制**，这部分内容留到工程章节再谈。"
- **总结**（原文）："光流法可以加速基于特征点的视觉里程计算法，避免计算和匹配描述子的过程，但**要求相机运动较平滑（或采集频率较高）**。"

---

## 8.4 直接法

引言（原文）："接下来，我们讨论与光流有一定相似性的直接法。与前面内容相似，先介绍直接法的原理，然后实现一遍直接法。"

### 8.4.1 直接法的推导（完整）

**动机/思路**（原文）："在光流中，会首先追踪特征点的位置，再根据这些位置确定相机的运动。这样一种**两步走的方案，很难保证全局的最优性**。读者可能会问，能不能在后一步中，调整前一步的结果呢？例如，如果认为相机右转了 $15^\circ$，那么光流能不能以这个 $15^\circ$ 运动作为初始值的假设，调整光流的计算结果呢？**直接法就是遵循这样的思路得到的结果。**"

**几何设定**（原文，图 8-4）：考虑某个空间点 $P$ 和两个时刻的相机。$P$ 的世界坐标为 $[X,Y,Z]$，它在两个相机上成像，记像素坐标为 $p_1,p_2$。

目标：求第一个相机到第二个相机的相对位姿变换。**以第一个相机为参照系**，设第二个相机的旋转和平移为 $R,t$（对应李群为 $T$）。两相机的内参相同，记为 $K$。完整的投影方程：

$$
\begin{array}{l}
\boldsymbol {p} _ {1} = \left[ \begin{array}{l} u \\ v \\ 1 \end{array} \right] _ {1} = \dfrac {1}{Z _ {1}} \boldsymbol {K P}, \\[3ex]
\boldsymbol {p} _ {2} = \left[ \begin{array}{l} u \\ v \\ 1 \end{array} \right] _ {2} = \dfrac {1}{Z _ {2}} \boldsymbol {K} (\boldsymbol {R P} + \boldsymbol {t}) = \dfrac {1}{Z _ {2}} \boldsymbol {K} (\boldsymbol {T P}) _ {1: 3}.
\end{array}
$$

（此式在源中未编号。）说明（原文）："其中 $Z_1$ 是 $P$ 的深度，$Z_2$ 是 $P$ 在第二个相机坐标系下的深度，也就是 $RP+t$ 的第 3 个坐标值。由于 $T$ 只能和**齐次坐标**相乘，所以乘完之后要**取出前 3 个元素**（记为 $(\cdot)_{1:3}$）。这和第 5 讲的内容是一致的。"

**与特征点法的差别**（原文）："回忆特征点法中，由于通过匹配描述子知道了 $p_1,p_2$ 的像素位置，所以可以计算重投影的位置。但在直接法中，**由于没有特征匹配，无从知道哪一个 $p_2$ 与 $p_1$ 对应着同一个点**。直接法的思路是**根据当前相机的位姿估计值寻找 $p_2$ 的位置**。但若相机位姿不够好，$p_2$ 的外观和 $p_1$ 会有明显差别。于是，为了减小这个差别，我们优化相机的位姿，来寻找与 $p_1$ 更相似的 $p_2$。这同样可以通过解一个优化问题完成，但此时最小化的不是重投影误差，而是**光度误差**，也就是 $P$ 的两个像素的亮度误差："

**第 1 步：定义光度误差（标量）。**

$$
e = \boldsymbol {I} _ {1} \left(\boldsymbol {p} _ {1}\right) - \boldsymbol {I} _ {2} \left(\boldsymbol {p} _ {2}\right). \tag{8.11}
$$

（原文强调："这里的 $e$ 是一个**标量**。"）

**第 2 步：单点优化目标（暂取不加权二范数）。**

$$
\min _ {\boldsymbol {T}} J (\boldsymbol {T}) = \| e \| ^ {2}. \tag{8.12}
$$

理由（原文）："能够做这种优化的理由，仍是基于**灰度不变假设**。我们假设一个空间点在各个视角下成像的灰度是不变的。"

**第 3 步：多点（N 个空间点 $P_i$）的整体优化问题。**

$$
\min _ {\boldsymbol {T}} J (\boldsymbol {T}) = \sum_ {i = 1} ^ {N} e _ {i} ^ {\mathrm{T}} e _ {i}, \quad e _ {i} = \boldsymbol {I} _ {1} \left(\boldsymbol {p} _ {1, i}\right) - \boldsymbol {I} _ {2} \left(\boldsymbol {p} _ {2, i}\right). \tag{8.13}
$$

关键点（原文）："注意，这里的**优化变量是相机位姿 $T$**，而不像光流那样优化各个特征点的运动。"

**第 4 步：定义两个中间变量（链式求导准备）。** 为求解此优化问题，关心误差 $e$ 如何随相机位姿 $T$ 变化，需要分析它们的导数关系。定义：

$$
\boldsymbol {q} = \boldsymbol {T P},
$$

$$
\boldsymbol {u} = \frac {1}{Z _ {2}} \boldsymbol {K} \boldsymbol {q}.
$$

（两式未编号。）说明（原文）："这里的 $q$ 为 $P$ 在**第二个相机坐标系下的坐标**，而 $u$ 为它的**像素坐标**。显然 $q$ 是 $T$ 的函数，$u$ 是 $q$ 的函数，从而也是 $T$ 的函数。"

**第 5 步：左扰动 + 一阶泰勒展开。** 考虑李代数的**左扰动模型**，利用一阶泰勒展开。因为：

$$
e (\boldsymbol {T}) = \boldsymbol {I} _ {1} (\boldsymbol {p} _ {1}) - \boldsymbol {I} _ {2} (\boldsymbol {u}), \tag{8.14}
$$

所以（链式法则展开导数）：

$$
\frac {\partial e}{\partial \boldsymbol {T}} = \frac {\partial \boldsymbol {I} _ {2}}{\partial \boldsymbol {u}} \frac {\partial \boldsymbol {u}}{\partial \boldsymbol {q}} \frac {\partial \boldsymbol {q}}{\partial \delta \boldsymbol {\xi}} \delta \boldsymbol {\xi}, \tag{8.15}
$$

其中 $\delta\boldsymbol\xi$ 为 $T$ 的**左扰动**。

> ⚠️ **本书统一约定差异**：本书以**右扰动**为主，本源此处用**左扰动**。综合时需把式 (8.15)(8.17)(8.18) 转为右扰动形式（右扰动下 $\partial q/\partial\delta\xi$ 的旋转块会带 $-(TP)^\wedge$ 的不同组合）。代码更新 `T21 = exp(update) * T21`（左乘）也证实左扰动。

**第 6 步：链式三项逐一计算**（原文）："一阶导数由于链式法则分成了 3 项，而这 3 项都是容易计算的："

**(1)** $\partial I_2/\partial u$ 为 **$u$ 处的像素梯度**（$1\times2$ 行向量）。

**(2)** $\partial u/\partial q$ 为**投影方程关于相机坐标系下的三维点的导数**。记 $q=[X,Y,Z]^\top$，根据第 7 讲的推导，导数为（$2\times3$ 矩阵）：

$$
\frac {\partial \boldsymbol {u}}{\partial \boldsymbol {q}} =
\left[ \begin{array}{l l l}
\frac {\partial u}{\partial X} & \frac {\partial u}{\partial Y} & \frac {\partial u}{\partial Z} \\
\frac {\partial v}{\partial X} & \frac {\partial v}{\partial Y} & \frac {\partial v}{\partial Z}
\end{array} \right] =
\left[ \begin{array}{c c c}
\frac {f _ {x}}{Z} & 0 & - \frac {f _ {x} X}{Z ^ {2}} \\
0 & \frac {f _ {y}}{Z} & - \frac {f _ {y} Y}{Z ^ {2}}
\end{array} \right]. \tag{8.16}
$$

**(3)** $\partial q/\partial\delta\xi$ 为**变换后的三维点对变换的导数**，这在李代数一讲（第 4 讲）介绍过（$3\times6$ 矩阵）：

$$
\frac {\partial \boldsymbol {q}}{\partial \delta \boldsymbol {\xi}} = [ \boldsymbol {I}, \ - \boldsymbol {q} ^ {\wedge} ]. \tag{8.17}
$$

（即左侧 $3\times3$ 为单位阵 $I$（对应平移），右侧 $3\times3$ 为 $-q^\wedge$（对应旋转）。这是**左扰动**形式。）

**第 7 步：合并后两项（与图像无关，常合并）。**（原文）："在实践中，由于后两项只与三维点 $q$ 有关，而与图像无关，经常把它合并在一起"（$2\times6$ 矩阵）：

$$
\frac {\partial \boldsymbol {u}}{\partial \delta \boldsymbol {\xi}} =
\left[ \begin{array}{c c c c c c}
\frac {f _ {x}}{Z} & 0 & - \frac {f _ {x} X}{Z ^ {2}} & - \frac {f _ {x} X Y}{Z ^ {2}} & f _ {x} + \frac {f _ {x} X ^ {2}}{Z ^ {2}} & - \frac {f _ {x} Y}{Z} \\
0 & \frac {f _ {y}}{Z} & - \frac {f _ {y} Y}{Z ^ {2}} & - f _ {y} - \frac {f _ {y} Y ^ {2}}{Z ^ {2}} & \frac {f _ {y} X Y}{Z ^ {2}} & \frac {f _ {y} X}{Z}
\end{array} \right]. \tag{8.18}
$$

（原文："这个 $2\times6$ 的矩阵在第 7 讲中也出现过。"列序：前 3 列对应平移 $\rho$、后 3 列对应旋转 $\phi$，即 $\xi=[\rho;\phi]$，与本书一致。）

> 抽取注（推导校验，源未展开但 §(8.18) = §(8.16)·§(8.17) 的乘积）：
> $\frac{\partial u}{\partial\delta\xi}=\frac{\partial u}{\partial q}\cdot\frac{\partial q}{\partial\delta\xi}$。其中 $-q^\wedge=\begin{bmatrix}0&Z&-Y\\-Z&0&X\\Y&-X&0\end{bmatrix}$。把 §(8.16)（$2\times3$）右乘 $[I,-q^\wedge]$（$3\times6$）即得 §(8.18)。例如第一行旋转块第 1 项 = $\frac{f_x}{Z}\cdot0 + 0\cdot(-Z) + (-\frac{f_xX}{Z^2})\cdot Y = -\frac{f_xXY}{Z^2}$，与 §(8.18) 第 (0,3) 元一致。

**第 8 步：误差关于李代数的总雅可比（$1\times6$）。** 于是推导出误差相对于李代数的雅可比矩阵：

$$
\boldsymbol {J} = - \frac {\partial \boldsymbol {I} _ {2}}{\partial \boldsymbol {u}} \frac {\partial \boldsymbol {u}}{\partial \delta \boldsymbol {\xi}}. \tag{8.19}
$$

**求解**（原文）："对于 $N$ 个点的问题，可以用这种方法计算优化问题的雅可比矩阵，然后使用**高斯牛顿法或列文伯格—马夸尔特方法**计算增量，迭代求解。至此，我们推导了直接法估计相机位姿的整个流程。"

> 抽取注（负号来源）：$e=I_1(p_1)-I_2(u)$，对 $u$ 求导得 $\partial e/\partial u = -\partial I_2/\partial u$，故总雅可比 (8.19) 带负号。高斯牛顿装配：$H=\sum J^\top J$（$6\times6$），$b=\sum -J^\top e$（注意 $e$ 标量、$J$ 为 $1\times6$ 行向量，代码中以 $6\times1$ 列向量 `J` 存储，故写作 $H=\sum J J^\top$、$b=\sum -e\,J$），增量 $\Delta\xi=H^{-1}b$，更新 $T\leftarrow\exp(\Delta\xi^\wedge)T$（左乘）。

### 8.4.2 直接法的讨论

**$P$ 的来源**（原文）："在上面的推导中，$P$ 是一个已知位置的空间点，它是怎么来的呢？
- 在 **RGB-D 相机**下，可以把任意像素反投影到三维空间，然后投影到下一幅图像中。
- 在**双目相机**中，同样可以根据视差来计算像素的深度。
- 在**单目相机**中，这件事情要更为困难，因为还须考虑由 $P$ 的深度带来的不确定性。详细的深度估计放到第 13 讲中讨论。"
现在先考虑简单的情况，即 $P$ 深度已知的情况。

**【分类表】根据 $P$ 的来源，直接法分为三类**（原文逐条）：

| 类别 | $P$ 的来源 | 点数量级 | 是否算描述子 | 速度 | 重构能力 | 备注 |
|---|---|---|---|---|---|---|
| **稀疏直接法**（Sparse） | 稀疏关键点 | 数百~上千个关键点 | 不必计算描述子 | **最快** | 只能计算稀疏的重构 | 像 LK 光流那样，假设关键点周围像素也不变 |
| **半稠密直接法**（Semi-Dense） | 部分像素（只用带梯度的像素，舍弃梯度不明显处） | 部分像素 | 否 | 中 | 可重构半稠密结构 | 见式 (8.19)：**若像素梯度为零，则整项雅可比为零，对运动增量无任何贡献**，故可舍弃无梯度像素 |
| **稠密直接法**（Dense） | 所有像素 | 几十万~几百万个 | 否 | 慢，**多数不能在 CPU 上实时**，需 **GPU 加速** | 可建立完整地图 | 梯度不明显的点在运动估计中贡献小，重构时也难以估计位置 |

**总结**（原文）："从稀疏到稠密重构，都可以用直接法计算。它们的计算量是逐渐增长的。稀疏方法可以快速地求解相机位姿，而稠密方法可以建立完整地图。具体使用哪种方法，需要视机器人的应用环境而定。特别地，在低端的计算平台上，**稀疏直接法可以做到非常快速的效果，适用于实时性较高且计算资源有限的场合** $^{[70]}$。"

---

## 8.5 实践：直接法

### 8.5.1 单层直接法

设定（原文）："现在演示如何使用**稀疏的直接法**。由于本书不涉及 GPU 编程，稠密的直接法省略了。同时，为了保持程序简单，**使用带深度的数据而非单目数据**，这样可以省略单目的深度恢复部分。基于特征点的深度恢复（即三角化）已经在第 7 讲介绍过，而基于块匹配的深度恢复将在后面介绍。所以本节考虑**双目的稀疏直接法**。"

实现思路（原文）："求解直接法最后等价于求解一个优化问题，因此可以使用 g2o 或 Ceres 这些优化库来帮助求解，也可以自己实现高斯牛顿法。和光流类似，直接法也可以分为单层直接法和金字塔式的多层直接法。在单层直接法中，类似于并行的光流，也可以并行地计算每个像素点的误差和雅可比，为此定义一个求雅可比的类。"

**代码：`slambook2/ch8/direct_method.cpp`（片段，JacobianAccumulator 类定义）**
```cpp
/// class for accumulator jacobians in parallel
class JacobianAccumulator {
public:
    JacobianAccumulator(
    const cv::Mat &img1_,
    const cv::Mat &img2_,
    const VecVector2d &px_ref_,
    const vector<double> depth_ref_,
    Sophus::SE3d &T21_) :
    img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_) {
    projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
    }

    /// accumulate jacobians in a range
    void accumulate_jacobian(const cv::Range &range);

    /// get hessian matrix
    Matrix6d hessian() const { return H; }

    /// get bias
    Vector6d bias() const { return b; }

    /// get total cost
    double cost_func() const { return cost; }

    /// get projected points
    VecVector2d projected_points() const { return projection; }

    /// reset h, b, cost to zero
    void reset() {
    H = Matrix6d::Zero();
    b = Vector6d::Zero();
    cost = 0;
    }

private:
    const cv::Mat &img1;
    const cv::Mat &img2;
    const VecVector2d &px_ref;
    const vector<double> depth_ref;
    Sophus::SE3d &T21;
    VecVector2d projection; // projected points

    std::mutex hessian_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};
```
> 抽取注：源构造函数初始化列表里有笔误 `px_ref(px_ref_, depth_ref(depth_ref_), T21(T21_)`（少一个右括号、逗号错位），上面已修正为 `px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_)`，语义不变。

**代码：`slambook2/ch8/direct_method.cpp`（片段，accumulate_jacobian 实现）**
```cpp
void JacobianAccumulator::accumulate_jacobian(const cv::Range &range) {

    // parameters
    const int half_patch_size = 1;
    int cnt_good = 0;
    Matrix6d hessian = Matrix6d::Zero();
    Vector6d bias = Vector6d::Zero();
    double cost_tmp = 0;

    for (size_t i = range.start; i < range.end; i++) {
    // compute the projection in the second image
    Eigen::Vector3d point_ref =
    depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
    Eigen::Vector3d point_cur = T21 * point_ref;
    if (point_cur[2] < 0) // depth invalid
    continue;

    float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy;
    if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
    v > img2.rows - half_patch_size)
    continue;

    projection[i] = Eigen::Vector2d(u, v);
    double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
    Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
    cnt_good++;

    // and compute error and jacobian
    for (int x = -half_patch_size; x <= half_patch_size; x++)
    for (int y = -half_patch_size; y <= half_patch_size; y++) {
    double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                   GetPixelValue(img2, u + x, v + y);
    Matrix26d J_pixel_xi;
    Eigen::Vector2d J_img_pixel;

    J_pixel_xi(0, 0) = fx * Z_inv;
    J_pixel_xi(0, 1) = 0;
    J_pixel_xi(0, 2) = -fx * X * Z2_inv;
    J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
    J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
    J_pixel_xi(0, 5) = -fx * Y * Z_inv;

    J_pixel_xi(1, 0) = 0;
    J_pixel_xi(1, 1) = fy * Z_inv;
    J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
    J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
    J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
    J_pixel_xi(1, 5) = fy * X * Z_inv;

    J_img_pixel = Eigen::Vector2d(
    0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
    0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y))
    );

    // total jacobian
    Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
    hessian += J * J.transpose();
    bias += -error * J;
    cost_tmp += error * error;
    }
    }

    if (cnt_good) {
    // set hessian, bias and cost
    unique_lock<mutex> lck(hessian_mutex);
    H += hessian;
    b += bias;
    cost += cost_tmp / cnt_good;
    }
}
```

说明（原文）："在这个类的 `accumulate_jacobian` 函数中，对指定范围内的像素点，按照之前的推导计算像素误差和雅可比矩阵，最后加到整体的 $H$ 矩阵中。"

> 抽取注（代码与推导一一对应）：
> - 反投影（用 ref 帧深度）：`point_ref = depth * K^{-1} * [px;1]`，即 $P=Z\,[(u-c_x)/f_x,\ (v-c_y)/f_y,\ 1]^\top$。
> - 变换到当前帧：`point_cur = T21 * point_ref`，即 $q=TP$。
> - 投影到第二图：$u=f_xX/Z+c_x,\ v=f_yY/Z+c_y$。
> - 深度无效（$Z<0$）或投影越界则跳过该点（`continue`）。
> - `J_pixel_xi`（$2\times6$）= §(8.18)，逐元素照搬。
> - `J_img_pixel`（$2\times1$）= $u$ 处像素梯度 $\partial I_2/\partial u$（中心差分 /2）。
> - 总雅可比 `J = -1.0 * (J_img_pixel^T * J_pixel_xi)^T` = §(8.19) 的 $6\times1$ 列形式。
> - 装配：`hessian += J*J^T`（$6\times6$），`bias += -error*J`（$6\times1$），`cost_tmp += error^2`。
> - `half_patch_size = 1` → 每点用 $3\times3$ 的 patch（$x,y\in\{-1,0,1\}$）。
> - 互斥锁 `hessian_mutex` 保证并行累加线程安全；代价按好点数归一 `cost += cost_tmp / cnt_good`。

**代码：`slambook2/ch8/direct_method.cpp`（片段，单层位姿估计迭代器）**
```cpp
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21) {
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, T21);

    for (int iter = 0; iter < iterations; iter++) {
    jaco_accu.reset();
    cv::parallel_for_(cv::Range(0, px_ref.size()),
    std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1));
    Matrix6d H = jaco_accu.hessian();
    Vector6d b = jaco_accu.bias();

    // solve update and put it into estimation
    Vector6d update = H.ldlt().solve(b);
    T21 = Sophus::SE3d::exp(update) * T21;
    cost = jaco_accu.cost_func();

    if (std::isnan(update[0])) {
        // sometimes occurred when we have a black or white patch and H is irreversible
        cout << "update is nan" << endl;
        break;
    }
    if (iter > 0 && cost > lastCost) {
        cout << "cost increased: " << cost << ", " << lastCost << endl;
        break;
    }
    if (update.norm() < 1e-3) {
        // converge
        break;
    }

    lastCost = cost;
    cout << "iteration: " << iter << ", cost: " << cost << endl;
    }
}
```

说明（原文）："该函数根据计算的 $H$ 和 $b$，求出对应的位姿更新量，然后更新到当前的估计值上。因为在理论部分已经把细节都介绍清楚了，所以这部分代码看起来不会特别困难。"

> 抽取注：核心更新 `T21 = Sophus::SE3d::exp(update) * T21;` —— **左乘更新**，对应左扰动模型 §(8.15)。退出判据：`isnan`→退出；`iter>0 && cost>lastCost`→代价上升退出；`update.norm()<1e-3`→收敛；最多 `iterations=10`。

### 8.5.2 多层直接法

引言（原文）："类似于光流，再把单层直接法拓展到金字塔式的多层直接法上，用 Coarse-to-fine 的过程计算相对运动。这部分代码和光流的也非常相似。"

**代码：`slambook2/ch8/direct_method.cpp`（片段，多层直接法）**
```cpp
void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21) {
    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++) {
    if (i == 0) {
    pyr1.push_back(img1);
    pyr2.push_back(img2);
    } else {
    cv::Mat img1_pyr, img2_pyr;
    cv::resize(pyr1[i - 1], img1_pyr,
    cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
    cv::resize(pyr2[i - 1], img2_pyr,
    cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
    pyr1.push_back(img1_pyr);
    pyr2.push_back(img2_pyr);
    }
    }

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy; // backup the old values

    for (int level = pyramids - 1; level >= 0; level--) {
    VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
    for (auto &px: px_ref) {
    px_ref_pyr.push_back(scales[level] * px);
    }

    // scale fx, fy, cx, cy in different pyramid levels
    fx = fxG * scales[level];
    fy = fyG * scales[level];
    cx = cxG * scales[level];
    cy = cyG * scales[level];
    DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }
}
```

**关键提醒**（原文）："需要注意的是，直接法求雅可比的时候**带上了相机的内参**，而当金字塔对图像进行缩放时，**对应的内参也需要乘以相应的倍率**。"（代码中 `fx=fxG*scales[level]` 等，且每层把参考像素 `px` 也乘以 `scales[level]`；深度 `depth_ref` 不缩放，因为是物理深度。）

### 8.5.3 结果讨论

实验设定（原文）："用一些示例图片测试直接法的结果。会用到几张 **Kitti** $^{[74]}$ 自动驾驶数据集的图像。首先，读取第一个图像 `left.png`，在对应的视差图 `disparity.png` 中，计算每个像素对应的深度，然后对 `000001.png`~`000005.png` 这五张图像，利用直接法计算相机的位姿。为了展示直接法对特征点的不敏感性，**随机地在第一张图像中选取一些点，不使用任何角点或特征点提取算法**，来看看它的结果。"

**代码：`slambook2/ch8/direct_method.cpp`（片段，main 主函数）**
```cpp
int main(int argc, char **argv) {
    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng;
    int nPoints = 2000;
    int boarder = 20;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
    int x = rng.uniform(boarder, left_img.cols - boarder); // don't pick pixels close to boarder
    int y = rng.uniform(boarder, left_img.rows - boarder); // don't pick pixels close to boarder
    int disparity = disparity_img.at<uchar>(y, x);
    double depth = fx * baseline / disparity; // you know this is disparity to depth
    depth_ref.push_back(depth);
    pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref;

    for (int i = 1; i < 6; i++) { // 1~10
    cv::Mat img = cv::imread((fmt_others % i).str(), 0);
    DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    }
    return 0;
}
```

> 抽取注（视差→深度，双目）：`depth = fx * baseline / disparity`，即 $Z=\dfrac{f_x\cdot b}{d}$（$b$ 为基线 baseline，$d$ 为视差 disparity）。随机选 `nPoints=2000` 个点，避开边界 `boarder=20` 像素。对 5 张图像累积估计 `T_cur_ref`（当前帧 ← 参考帧）。

**【数值例：实验结果】**（原文照录）："读者可以尝试在你的计算机上运行本段程序，它将输出每个图像的每层金字塔上的追踪点，并输出运行时间。多层直接法的结果如图 8-5 所示。根据程序输出结果，可以看到**第五张追踪图像反映的大约是相机往前运动 3.8 米时的情况**。可见，即使我们随机选点，直接法也能够正确追踪大部分的像素，同时估计相机的运动。这中间没有任何的特征提取、匹配或光流的过程。从运行时间上看，**在 2000 个点时，直接法每迭代一层需要 1~2 毫秒，所以四层金字塔约耗时 8 毫秒**。相比之下，**2000 个点的光流耗时大约在十几毫秒**，还不包括后续的位姿估计。所以，**直接法相比于传统的特征点和光流通常更快一些**。"

（图 8-5 子图：左上=原始图像；右上=原始视差图；左下=第五张追踪图像；右下=直接法追踪点。）

**【数值例：一次迭代的图形化解释（图 8-6）】**（原文照录，含具体灰度数字）：

下面对直接法的迭代过程做解释（原文）："相比于特征点法，直接法完全依靠优化来求解相机位姿。从式 (8.19) 中可以看到，**像素梯度引导着优化的方向**。如果想要得到正确的优化结果，就必须保证大部分像素梯度能够把优化引导到正确的方向。"

具体数值场景（原文逐句）：
- "假设对于参考图像，我们测量到一个灰度值为 **229** 的像素。并且，由于我们知道它的深度，可以推断出空间点 $P$ 的位置（图 8-6 所示在 $I_1$ 中测量到的灰度）。"
- "此时又得到了一幅新的图像，需要估计它的相机位姿。这个位姿是由一个初值不断地优化迭代得到的。假设初值比较差，在这个初值下，空间点 $P$ 投影后的像素灰度值是 **126**。于是，此像素的误差为 $229 - 126 = 103$。为了减小这个误差，希望微调相机的位姿，使像素更亮一些。"
- "怎么知道往哪里微调像素会更亮呢？这就需要用到局部的像素梯度。我们在图像中发现，**沿 $u$ 轴往前走一步，该处的灰度值变成了 123，即减去了 3**。同样地，**沿 $v$ 轴往前走一步，灰度值减了 18，变成 108**。在这个像素周围，看到梯度是 **$[-3,-18]$**，为了提高亮度，会建议优化算法微调相机，使 $P$ 的像往**左上方**移动。在这个过程中，用像素的局部梯度近似了它附近的灰度分布，不过请注意，**真实图像并不是光滑的，所以这个梯度在远处就不成立了**。"
- "但是，优化算法不能只听这个像素的一面之词，还需要听取其他像素的建议（原文有脚注①）。综合听取了许多像素的意见之后，优化算法选择了一个和我们建议的方向偏离不远的地方，**计算出一个更新量 $\exp(\xi^\wedge)$**。加上更新量后，图像从 $I_2$ 移动到了 $I_2'$，像素的投影位置也变到了一个更亮的地方。通过这次更新，误差变小了。在理想情况下，期望误差会不断下降，最后收敛。"

**【非凸性讨论（图 8-7）】**（原文）："但是实际是不是这样呢？我们是否真的只要沿着梯度方向走，就能走到一个最优值呢？注意，直接法的梯度是直接由图像梯度确定的，因此必须保证沿着图像梯度走时，灰度误差会不断下降。然而，**图像通常是一个很强烈的非凸函数**（图 8-7）。实际中，如果沿着图像梯度前进，很容易由于图像本身的非凸性（或噪声）落进一个**局部极小值**中，无法继续优化。**只有当相机运动很小，图像中的梯度不会有很强的非凸性时，直接法才能成立**。"

（图 8-7 注解：一张图像的三维化显示。从图像中的一个点运动到另一个点的路径不见得是"笔直的下坡路"，而需要经常"翻山越岭"。这体现了图像本身的非凸性。）

**patch 与差异度量讨论**（原文）："在例程中，我们只计算了单个像素的差异，并且这个差异是由灰度直接相减得到的。然而，**单个像素没有什么区分性**，周围很可能有好多像素和它的亮度差不多。所以，我们有时会使用**小的图像块（patch）**，并且使用更复杂的差异度量方式，例如**归一化相关性（Normalized Cross Correlation，NCC）**等。而例程为了简单起见，使用了误差的平方和，以保持与推导的一致性。"

### 8.5.4 直接法优缺点总结

**优点**（原文逐条）：
- 可以**省去计算特征点、描述子的时间**。
- **只要求有像素梯度即可，不需要特征点**。因此，直接法可以在特征缺失的场合下使用。比较极端的例子是只有渐变的一幅图像。它可能无法提取角点类特征，但可以用直接法估计它的运动。在演示实验中，直接法对随机选取的点亦能正常工作。这一点在实用中非常关键。
- 可以构建**半稠密乃至稠密的地图**，这是特征点法无法做到的。

**缺点**（原文逐条）：
- **非凸性。** 直接法完全依靠梯度搜索，降低目标函数来计算相机位姿。其目标函数中需要取像素点的灰度值，而图像是强烈非凸的函数。这使得优化算法容易进入极小，**只在运动很小时直接法才能成功**。针对于此，**金字塔的引入可以在一定程度上减小非凸性的影响**。
- **单个像素没有区分度。** 和它像的实在太多了！于是我们要么计算图像块，要么计算复杂的相关性。由于每个像素对改变相机运动的"意见"不一致，只能少数服从多数，**以数量代替质量**。所以，直接法在选点较少时的表现下降明显，**通常建议用 500 个点以上**。
- **灰度值不变是很强的假设。** 如果相机是自动曝光的，当它调整曝光参数时，会使得图像整体变亮或变暗。光照变化时也会出现这种情况。特征点法对光照具有一定的容忍性，而直接法由于计算灰度间的差异，整体灰度变化会破坏灰度不变假设，使算法失败。针对这一点，**实用的直接法会同时估计相机的曝光参数** $^{[70]}$，以便在曝光时间变化时也能工作。

---

## 习题（源 第8讲 习题，原文照录）

1. 除了 LK 光流，还有哪些光流方法？它们各有什么特点？
2. 在本节程序的求图像梯度过程中，我们简单地求了 $u+1$ 和 $u-1$ 的灰度之差除以 2，作为 $u$ 方向上的梯度值。这种做法有什么缺点？提示：对于距离较近的特征，变化应该较快；而距离较远的特征在图像中变化较慢，求梯度时能否利用此信息？
3. 直接法是否能和光流一样，提出"反向法"的概念？即，使用原始图像的梯度代替目标图像的梯度？
4. \* 使用 Ceres 或 g2o 实现稀疏直接法和半稠密直接法。
5. 相比于 RGB-D 的直接法，单目直接法往往更加复杂。除了匹配未知，像素的距离也是待估计的，我们需要在优化时把像素深度也作为优化变量。阅读参考文献 [69, 75]，你能理解它的原理吗？

---

## 参考文献编号对照（本章出现的引用）

- [67] 早期直接法的使用
- [68] SVO
- [69] LSD-SLAM（习题 5 亦引用）
- [70] DSO（多处引用：直接法主流项目、低端平台稀疏直接法、估计曝光参数）
- [71] Lucas-Kanade 光流
- [72] Horn-Schunck 光流
- [73] 反向光流法的梯度替代依据
- [74] Kitti 自动驾驶数据集
- [75] 单目直接法（习题 5 引用）

---

## 抽取自检（覆盖性确认）

- **公式**：(8.1)~(8.19) 全部抽取（含两个未编号的中间变量定义 $q=TP$、$u=KQ/Z_2$，以及未编号的双视图投影方程组、章首两个预告公式），逐一保留 LaTeX。
- **代码/伪码**：OpenCV LK 调用、`OpticalFlowTracker` 类+单层入口、`calculateOpticalFlow` 完整实现（正向/反向分支）、`OpticalFlowMultiLevel`、`JacobianAccumulator` 类定义+`accumulate_jacobian`、`DirectPoseEstimationSingleLayer`、`DirectPoseEstimationMultiLayer`、`main`，共 8 段，全部保留并修正了源 md 的分块/笔误（已注明）。
- **数值例**：金字塔 20→5 像素例、光流运行时间终端输出、直接法 2000 点/8 毫秒/3.8 米、图 8-6 灰度 229/126/103/梯度[-3,-18]，全部保留。
- **表/分类**：记号约定表、直接法稀疏/半稠密/稠密分类表，已制表。
- **关键约定差异**：左扰动 vs 本书右扰动（已多处标注），$\xi=[\rho;\phi]$ 排序一致（已确认），雅可比链式三项及合并矩阵 §(8.18)=§(8.16)·§(8.17) 已给推导校验。
- **范围说明**：已在 §0 明确本第 8 讲不含对极几何/8点法/三角化/PnP/ICP（属第 7 讲），并指明这些主题在本源中均"指回第 7 讲"。
