# 抽取留痕：稠密地图表示与稠密建图（点云 / OctoMap / TSDF-KinectFusion / surfel / 网格重建）——服务于【稠密建图】章

> **本抽取的性质**：项目内部「抽取留痕」（非成书正文）。目标 = 把"稠密建图"主题的多个权威源【全量保真】抽下来（每一步推导、每个公式、每条定义/算法、每个数值/默认参数、每段工程权衡），供综合 agent 写成自包含书章。**铁律：禁摘要、禁凝练。** 公式一律 LaTeX 写全并标源。
>
> **本抽取服务的成书章节**：**稠密建图**（dense mapping）。
>
> **聚焦主题（任务要求须重点覆盖）**：① 地图用途与分类（度量/拓扑/语义、稀疏 vs 稠密）；② 单目稠密（极线搜索 + 块匹配 + 深度滤波完整推导）；③ RGB-D 稠密（点云、八叉树 OctoMap 的 log-odds + clamping、TSDF/KinectFusion、surfel/ElasticFusion）；④ 网格重建简介（Marching Cubes / 泊松重建）；⑤ 工程权衡。
>
> **联网研究方式**：用 WebSearch/WebFetch 检索权威源（原论文 / 官方源码 / 官方文档），凡能下载到的论文 PDF 一律用 `pdftotext -layout` 本地抽取后**逐字核对公式**，再写入本件。下文每节标出处。

---

## 0 源清单与抽取方式（出处可追溯）

本抽取**逐字核对**了以下一手源（均已用 `pdftotext` 本地抽出公式逐行比对，非凭记忆）：

| 编号 | 源 | 类型 | 出处 / URL | 本抽取用到的核心内容 |
|---|---|---|---|---|
| **S1** | Hornung, Wurm, Bennewitz, Stachniss, Burgard, *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees*, **Autonomous Robots 2013**, DOI 10.1007/s10514-012-9321-0 | 原论文 | `https://www.arminhornung.de/Research/pub/hornung13auro.pdf` | 占据栅格贝叶斯更新（式1）、log-odds 加性更新（式2-3）、clamping 更新策略（式4）、内节点聚合（式5-6）、beam 逆传感器模型（式7）、全部实验默认值 |
| **S2** | Hornung, *3D Mapping with OctoMap*, **RosCon 2013** tutorial slides | 官方教程 | `https://www.arminhornung.de/Research/pub/hornung13roscon.pdf` | OctoMap 库的**默认参数**：`setOccupancyThres(0.5)`、`setProbHit(0.7)`、`setProbMiss(0.3)`、`setClampingThresMin(0.1)`、`setClampingThresMax(0.95)` |
| **S3** | OctoMap 源码 `octomap_utils.h`（devel 分支） | 官方源码 | `https://github.com/OctoMap/octomap` | `logodds()` / `probability()` 内联函数的 C++ 实现 |
| **S4** | OctoMap 源码 `OccupancyOcTreeBase.hxx`（devel 分支） | 官方源码 | 同上 | `updateNodeLogOdds()` 的 clamping 实现 |
| **S5** | Newcombe, Izadi, Hilliges, Molyneaux, Kim, Davison, Kohli, Shotton, Hodges, Fitzgibbon, *KinectFusion: Real-Time Dense Surface Mapping and Tracking*, **ISMAR 2011** | 原论文 | `https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf` | TSDF 表示（式5）、空间约束、投影式 TSDF（式6-9）、加权运行平均融合（式10-13）、光线投射表面预测（式14-15） |
| **S6** | Curless, Levoy, *A Volumetric Method for Building Complex Models from Range Images*, **SIGGRAPH 1996**, pp.303-312 | 原论文（TSDF 鼻祖） | `https://dl.acm.org/doi/10.1145/237170.237269` | 累积加权符号距离函数 D(x)、W(x) 的原始定义与增量更新（KinectFusion 式11-12 即引用此文 [7]） |
| **S7** | Whelan, Leutenegger, Salas-Moreno, Glocker, Davison, *ElasticFusion: Dense SLAM Without A Pose Graph*, **RSS 2015** | 原论文 | `https://www.roboticsproceedings.org/rss11/p01.pdf` | surfel 的属性集合（位置/法线/颜色/权重/半径/双时间戳）、active/inactive 划分、几何+光度联合跟踪 |
| **S8** | Keller, Lefloch, Lambers, Izadi, Weyrich, Kolb, *Real-Time 3D Reconstruction in Dynamic Scenes Using Point-Based Fusion*, **3DV 2013** | 原论文（surfel 融合算法本体） | `https://ieeexplore.ieee.org/document/...`（ElasticFusion 的 surfel 初始化/融合规则均引用此文 [9]） | surfel 半径 $r$ 的计算、置信权重、运行平均融合（ElasticFusion 把细节"follows the same rules as Keller et al."甩给本文） |
| **S9** | Vogiatzis, Hernández, *Video-based, Real-Time Multi View Stereo*, **Image and Vision Computing 29(7):434-441, 2011** | 原论文（深度滤波器本体） | （本地 PDF 已抽）| 深度+内点率的 Gaussian+Uniform 混合测量模型（式1）、贝叶斯后验（式2）、Gaussian×Beta 参数化近似（式3-4）、矩匹配序贯更新 |
| **S10** | Pizzoli, Forster, Scaramuzza, *REMODE: Probabilistic, Monocular Dense Reconstruction in Real Time*, **ICRA 2014** | 原论文 | `https://rpg.ifi.uzh.ch/...`（本地 PDF 已抽）| 贝叶斯深度估计（式1-4，沿用 S9）、**逐像素几何测量不确定度推导（式13-19，"一像素视差→深度方差 $\tau_k^2$"）**、收敛/发散判据、TV 正则去噪 |
| **S11** | 高翔、张涛等《视觉SLAM十四讲》第 12/13 讲「建图」 | 中文教材（本章主骨架） | （本书三大主源之一）| 单目稠密重建完整管线（极线搜索/块匹配/深度滤波/逆深度）、RGB-D 建图（点云/八叉树/TSDF）；其数学与 S9/S10 一致，几何不确定度推导即 S10 式13-19 |
| **S12** | Lorensen, Cline, *Marching Cubes: A High Resolution 3D Surface Construction Algorithm*, **SIGGRAPH 1987**, Computer Graphics 21(4):163-169 | 原论文 | `https://dl.acm.org/doi/10.1145/37402.37422` | $2^8=256$ 立方体配置 → 15 基本情形、边线性插值、查找表 |
| **S13** | Kazhdan, Bolitho, Hoppe, *Poisson Surface Reconstruction*, **SGP 2006** | 原论文 | `http://people.eecs.berkeley.edu/~jrs/meshpapers/KazhdanBolithoHoppe.pdf` | 指示函数梯度 = 定向法向场，泊松方程 $\Delta\chi=\nabla\cdot\vec V$，自适应八叉树多重网格 |
| **S14** | Cadena et al.（SLAM 综述）/ Active SLAM 综述等 | 综述 | `https://arxiv.org/pdf/2207.00254` 等 | 地图分类：度量（稀疏/稠密）/拓扑/度量-语义/混合 |

> **综合 agent 提示（覆盖能力实话实说）**：
> - **能给真材实料的硬核推导**：OctoMap 全套占据更新与 clamping（S1，含全部公式与默认值）；KinectFusion TSDF 融合（S5，式5-15 逐字）；单目深度滤波器（S9/S10/S11，含**逐像素几何不确定度的完整三角推导**式13-19）；NCC 块匹配公式；Marching Cubes 边插值。
> - **概念/算法级（无逐式推导，但本抽取已记全可用要素）**：surfel 融合（S7 给属性表+active/inactive，融合细节甩给 S8；本抽取已据 S8/InfiniTAM 给出半径/权重/平均公式的标准形）；泊松重建（S13 给泊松方程主干）；地图分类（S14）。
> - **本套抽取中已有的姊妹件**：点云数据结构/KD-tree/ICP/NDT 见 `point_cloud_processing__*.md`、`lidar_slam__*.md`；本件**不重复** ICP，专注"稠密地图表示与建图"。

---

## 1 本抽取记号约定（Notation）及与本书统一约定的差异

下表把各源记号统一到本书约定（$\mathbf R\in SO(3)$、右扰动主、Hamilton 四元数、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$、过程/观测噪声 $\boldsymbol\Sigma_w/\boldsymbol\Sigma_v$），并注明差异。

| 量 | 本抽取/源记号 | 含义 | 与本书统一约定的关系 / 差异 |
|---|---|---|---|
| 旋转 | $\mathbf R\in SO(3)$ | 旋转矩阵 | **一致**。S7(ElasticFusion) 用李代数 $\boldsymbol\xi=[\boldsymbol\omega^\top\ \mathbf x^\top]^\top$，**旋转分量在前、平移在后**（$\boldsymbol\omega$ 旋转、$\mathbf x$ 平移）——**与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）排序相反**，综合时须翻转并注明。 |
| 位姿 | $\mathbf T\in SE(3)$，$\mathbf T_{g,k}$ / $\mathbf T_{k,w}$ | 相机位姿 | S5 用 $\mathbf T_{g,k}$=相机 $k$ 到全局 $g$；S10 用 $\mathbf T_{k,w}$=世界到相机 $k$（**与 S5 方向相反**）。本书统一下标语义 $\mathbf T_{wc}$=相机到世界，综合时按物理意义对齐。 |
| 占据概率 | $P(n)$ / $P(n\mid z_{1:t})$ | 体素 $n$ 被占据的概率 | 一致（本书占据栅格章用 $P(\text{occ})$）。 |
| log-odds（对数几率） | $L(n)$ | $\log\frac{P(n)}{1-P(n)}$ | 一致；OctoMap 源码内联函数名为 `logodds()`。 |
| TSDF 值 / 权重 | $F(\mathbf p)$ / $F_k(\mathbf p)$，$W(\mathbf p)$ / $W_k(\mathbf p)$ | 体素截断符号距离 / 累积权重 | S6(Curless-Levoy) 记为 $D(\mathbf x)$ / $W(\mathbf x)$；S5(KinectFusion) 记为 $F$ / $W$。**同一量、不同字母**，综合统一用 $F,W$，注明 $D\equiv F$。 |
| 截断距离 | $\mu$ | TSDF 截断半带宽 | 一致（亦常记 $\delta$ 或 trunc）。 |
| 深度（单目滤波） | $d$ / $Z$，均值 $\mu$（或 $\mu_k$）、方差 $\sigma^2$（或 $\sigma_k^2$） | 像素深度的高斯估计 | S9 用 $Z$；S10/S11 用 $d$。一致。 |
| 内点率 | $\rho$（或 $\pi$） | 测量为内点（好测量）的概率 | S9 用 $\pi$，S10 用 $\rho$。**同一量、不同字母**，综合统一用 $\rho$。 |
| 内点率 Beta 计数 | $a_k,b_k$ | 累积内点/外点"概率计数" | 一致。 |
| 测量方差 | $\tau_k^2$ | 第 $k$ 次三角化深度测量的方差 | 一致。 |
| surfel 属性 | $\mathbf p,\mathbf n,\mathbf c,w,r,t_0,t$ | 位置/法线/颜色/置信权重/半径/创建时刻/末次更新时刻 | 一致。注意 surfel 的 $w$ 是"置信权重"，与 TSDF 的 $W$ 同名但语义略不同（皆为累积置信），综合时勿混。 |

> **⚠️ 综合统一注意（务必照办）**：
> 1. **TSDF 符号约定方向**：本抽取按 KinectFusion/Curless 约定——**符号距离在表面前方（朝相机一侧、自由空间）为正、在表面后方（不可见侧）为负**，零交叉即表面。**部分实现（如部分 voxblox/InfiniTAM 教程）取相反符号**，综合时全章统一一种符号并显式声明。
> 2. **内点率字母** $\pi$ vs $\rho$、**TSDF 字母** $D$ vs $F$、**李代数排序**（ElasticFusion 旋转在前）——以上三处不同源不一致，已在表中标出，综合时统一。
> 3. 本主题源**几乎不涉及**右/左扰动、四元数 Hamilton/JPL、$\boldsymbol\Sigma_w/\boldsymbol\Sigma_v$ 等（这些是估计/李群章的约定），故"无差异可比"。唯一出现李代数的是 ElasticFusion 的位姿优化（式5-7），按本书右扰动主线综合时复述即可。

---

## 2 地图的用途与分类（度量 / 拓扑 / 语义；稀疏 vs 稠密）（源 S11 §12/13 导言 + S14 综述）

### 2.1 为什么稀疏地图不够用——稠密建图的动机（源 S11）

经典基于特征点的 SLAM（如 ORB-SLAM）所建的地图是**稀疏路标地图**：地图 = 一堆被三角化的特征点的三维坐标。这种地图**足以支撑定位**（重投影约束、回环），但**不足以支撑很多下游任务**：

- **导航 / 避障 / 路径规划**：机器人需要知道"哪里是障碍、哪里可通行"。稀疏点之间的空白处是"未知"还是"自由"无法判断——稀疏地图对"两个路标点之间能不能走过去"无能为力。
- **重建 / 交互 / AR**：需要稠密的表面（mesh / surfel / 隐式场）才能做碰撞、遮挡、放置虚拟物体。
- **可视化 / 人机交互**：稀疏点云不直观。

> **本质洞察（源 S11 导言）**：定位与建图是 SLAM 的两个目的；**特征点法把"建图"退化成了"定位的副产品"**。稠密建图的任务是：在已知相机轨迹（位姿由前端 VO/VIO/SLAM 给出）的前提下，**估计每一个（或大部分）像素 / 体素的几何状态**，得到可用于导航、避障、重建、交互的地图。

### 2.2 地图分类（综合 S11 + S14）

**按几何抽象层次分（S14 综述给出 4 类）**：

| 类型 | 定义 | 顶点/元素语义 | 典型用途 | 代表 |
|---|---|---|---|---|
| **度量地图（metric map）** | 精确编码环境的**几何位置**（"在哪"）。可再分稀疏/稠密。 | 三维坐标、占据概率、距离场等 | 精确定位、避障、运动规划 | 占据栅格、OctoMap、TSDF、surfel、点云 |
| **拓扑地图（topological map）** | 用**轻量图**描述环境的**连通性**（"能不能到"），弱化精确度量。顶点=自由空间中的凸区域/地点，边=区域间的可达连接。 | 地点节点 + 可达边 | 大尺度全局路径、地点识别 | 地点图、路标连接图 |
| **度量-语义地图（metric-semantic map）** | 在几何之上叠加**语义**（"这是什么"：桌子/门/墙）。近十年随深度学习兴起。 | 几何 + 物体类别/实例标签 | 任务级规划、人机交互、主动感知 | SemanticFusion、稠密语义网格、稀疏物体级地图（质心/立方体/椭球/圆柱/网格模型） |
| **混合地图（hybrid map）** | 度量 + 拓扑（+ 语义）分层组合：局部度量、全局拓扑。 | 分层 | 兼顾大尺度效率与局部精度 | 分层 SLAM |

**按密度分（S14：度量地图再分两类）**：

| | **稀疏地图（sparse）** | **稠密地图（dense）** |
|---|---|---|
| 内容 | 只存显著特征/路标（角点、物体质心/立方体/椭球/圆柱） | 存大量/全部表面或体素（mesh、体素、surfel、2.5D 栅格） |
| 用途 | 定位、实时下游任务（主动信息采集、多机协作） | 避障、重建、交互（稠密度量-语义地图适合避障） |
| 内存/算力 | 小 | 大 |

> **源 S14 原文要点（逐条保真）**：
> - "There are four different types of map representations: topological, metric, metric-semantic, and hybrid maps."
> - "Topological maps use lightweight graphs to describe information about the topology of the environment, where vertices represent convex regions in the free space, while edges model connections between them."
> - "Metric maps are the most used representations ... can be further divided into two categories: sparse and dense maps."
> - "Dense semantic maps include meshes, volumetric maps, surfels, and 2.5D grid maps, while sparse object-level maps use prior information of object shapes such as centroids, cubes, ellipsoids, cylinders, and mesh-based models."
> - "While dense metric-semantic maps are suitable for obstacle avoidance, sparse object-level maps are desirable for real-time downstream tasks such as active information gathering and multi-robot collaboration."

**本书"稠密建图"章主要落在"度量-稠密"这一格**，并简介语义。下文按传感器分两大支：**单目稠密**（§3）与 **RGB-D / 深度稠密**（§4-§6）。

---

# 第一部分 单目稠密建图（极线搜索 + 块匹配 + 深度滤波器）

> 主源：S11《视觉SLAM十四讲》§13；其数学骨架与 S9(Vogiatzis-Hernández)、S10(REMODE) 完全一致——**S11 的几何不确定度推导即 S10 式13-19**，S11 的深度滤波即 S9/S10 的简化（高斯-only）版本。本抽取把 S11 的管线讲全，并用 S9/S10 的原始论文公式**补足严格版**（混合模型、Gaussian×Beta、矩匹配）。

## 3 单目稠密重建（Monocular Dense Reconstruction）

### 3.1 问题设定与总览（源 S11 §13.2）

**设定**：相机轨迹（各帧位姿 $\mathbf T_{k,w}\in SE(3)$）已由前端给出（VO/VIO/SLAM）。取一帧为**参考帧** $I_r$，要估计参考帧中**每个像素**的深度 $d$，从而把参考帧"长出"稠密点云/深度图。

**核心难点**：单目无尺度、单帧无深度。靠**多视几何 + 跨帧匹配**恢复深度，并用**概率滤波**逐帧累积、抑制误匹配。

**单目稠密重建的完整流程（源 S11 §13.2，逐步保真）**：

1. 假设参考帧所有像素深度满足某初始分布（如均值取场景中值、方差很大）。
2. 当新数据（新帧 $I_k$）到来：遍历参考帧每个待估深度像素，通过**极线搜索（epipolar search）+ 块匹配（block matching）**确定它在 $I_k$ 中的投影点位置。
3. 根据几何关系，**三角化**算出该次观测的深度（均值）与**不确定度（方差）**。
4. 把当前观测**融合**进上一次的估计（高斯分布相乘 / 贝叶斯更新）。
5. 检测方差是否收敛；若收敛则停止该像素，否则回到第 2 步等下一帧。

> **本质洞察（源 S11）**：单目稠密重建 = "**像素级、概率化、序贯**的多视立体（MVS）"。与双目"一次性左右匹配"不同，单目用**运动产生基线**、用**滤波累积多帧**来弥补单帧无深度。这正是 S9/S10 的"per-pixel probabilistic depth estimation"。

### 3.2 极线搜索（Epipolar Search）（源 S11 §13.2.1 + S9 §3）

**原理**：参考帧像素 $\mathbf p_1$ 的真实深度未知，但被约束在 $[d_{\min},d_{\max}]$（或 $[Z_{\min},Z_{\max}]$）之间。把这一段深度区间投影到当前帧 $I_k$，得到一条**线段**——**极线段（epipolar segment）**。$\mathbf p_1$ 的匹配点必在这条极线段上。于是匹配的搜索范围**从整幅图像缩小为一条直线**。

- 设参考帧像素 $\mathbf p_1$，归一化坐标 $\mathbf x_1=\mathbf K^{-1}\dot{\mathbf p}_1$（$\dot{\cdot}$ 为齐次）。
- 深度取最小值 $d_{\min}$ 时对应空间点 $d_{\min}\mathbf x_1$，取最大值（含无穷远）时对应 $d_{\max}\mathbf x_1$。
- 两端点经 $\mathbf T_{k,r}$（参考帧到当前帧）变换并投影到 $I_k$，得极线段两端 $\mathbf p_{\min},\mathbf p_{\max}$；连线即极线。

> **源 S9 原文（图1注，逐字）**："Searching for a match along an optic ray. For a given pixel p we wish to find the depth Z along ... we can exploit epipolar geometry." 即沿光线/极线一维搜索。

**几何收益**：搜索复杂度从 $O(\text{图像面积})$ 降到 $O(\text{极线长度})$。极线段长度随深度不确定度收缩——方差越小、搜索段越短（S10 §III.B 明确用当前方差**限制极线搜索范围**，并基于**逆深度**搜索以适配大深度范围）。

### 3.3 块匹配（Block Matching）与相关性度量（源 S11 §13.2.2）

**为什么用块不用单像素**：单像素灰度无区分度（很多像素灰度相同），匹配会到处都"像"。改用**图像块（patch / window）**的"灰度分布相似度"。前提是**灰度不变性假设**：同一空间点在不同视角下成像灰度近似不变。

设参考帧在 $\mathbf p_1$ 周围取 $w\times w$ 块 $A$，当前帧在极线上候选点 $\mathbf p_2$ 周围取同尺寸块 $B$。常用相似/差异度量（源 S11 §13.2.2 全部列出）：

**(1) SAD（Sum of Absolute Differences，绝对差之和）**——越小越像：
$$
S_{\text{SAD}}(A,B)=\sum_{i,j}\bigl|A(i,j)-B(i,j)\bigr|.
$$

**(2) SSD（Sum of Squared Differences，平方差之和）**——越小越像：
$$
S_{\text{SSD}}(A,B)=\sum_{i,j}\bigl(A(i,j)-B(i,j)\bigr)^2.
$$

**(3) NCC（Normalized Cross-Correlation，归一化互相关）**——越接近 $+1$ 越像（对线性光照变化鲁棒）：
$$
S_{\text{NCC}}(A,B)=
\frac{\displaystyle\sum_{i,j}\bigl(A(i,j)-\overline A\bigr)\bigl(B(i,j)-\overline B\bigr)}
{\sqrt{\displaystyle\sum_{i,j}\bigl(A(i,j)-\overline A\bigr)^2}\ \sqrt{\displaystyle\sum_{i,j}\bigl(B(i,j)-\overline B\bigr)^2}},
$$
其中 $\overline A=\frac{1}{w^2}\sum_{i,j}A(i,j)$、$\overline B=\frac{1}{w^2}\sum_{i,j}B(i,j)$ 为两块均值。分子是去均值后的互相关，分母是两块去均值后的标准差之积（归一化）。$S_{\text{NCC}}\in[-1,1]$；$=1$ 完全相关（同一表面），$=0$ 不相关，$=-1$ 反相关。

> **三度量对比（源 S11 + 网络 IP 教程）**：
> - SAD/SSD 计算最快，但**对整体亮度/对比度变化不鲁棒**（曝光一变就失配）。
> - **NCC 去均值 + 归一化方差**，对线性光照变化（$B=\alpha A+\beta$）天然不变，**鲁棒性最好**，但计算最贵。
> - 也有"去均值 SSD（ZNCC/ZSSD）"等折中。S9/S10 实现用以 NCC 为代表的块相似度。

**匹配判定**：沿极线对每个候选点算度量，取**最优**（SAD/SSD 取最小、NCC 取最大）者为匹配。可设阈值（如 NCC > 0.85）拒绝弱匹配；亚像素可对度量曲线做抛物线拟合求极值。

> **陷阱（源 S11）**：极线搜索 + 块匹配**只在有纹理处可靠**。在**无纹理/重复纹理/被遮挡**区域，极线上会出现多个极大值（多模态），匹配歧义大——这正是为何后面要用**概率深度滤波**而非"一次匹配定深度"：用多帧、多基线观测 + 内点率建模来甄别。

### 3.4 三角化求深度（均值）（源 S11 §13.2.3 / S10 §II.B）

确定匹配点 $\mathbf p_2$ 后，由两帧的归一化坐标 $\mathbf x_1,\mathbf x_2$ 与相对位姿 $(\mathbf R,\mathbf t)=\mathbf T_{k,r}$ 三角化求深度。空间点在参考帧下满足
$$
d_1\,\mathbf x_1 = d_2\,(\mathbf R\,\mathbf x_1)\ ?\ \dots
$$
更标准地，对应关系为
$$
d_2\,\mathbf x_2 = d_1\,\mathbf R\,\mathbf x_1 + \mathbf t ,
$$
其中 $d_1,d_2$ 分别是该点在参考帧、当前帧下的深度（沿各自光线的尺度），$\mathbf x_1,\mathbf x_2$ 为归一化平面坐标。这是关于 $(d_1,d_2)$ 的两条独立方程（三维向量去掉一维冗余），可解出 $d_1$（即参考帧深度）。S11 用与十四讲第 7 讲三角测量同一套线性最小二乘解（左乘 $\mathbf x_2^\wedge$ 等消元），此处不重复（见 `visual_odometry__*.md` 三角测量节）。**这一步得到本次观测的深度均值，记为 $d_{\text{obs}}$（或三角化值 $d_k$）。**

### 3.5 ⭐核心推导：单像素视差 → 深度测量不确定度 $\tau_k^2$（源 S10 REMODE 式13-19，S11 §13.2.3 逐字复现此推导）

> **这是单目稠密建图最精彩的几何推导**，S11《十四讲》§13.2.3"深度的不确定性分析"与 S10 REMODE 图2/式13-19 **完全相同**。下面按 S10 原文逐式保真，并给出 S11 的几何叙述。

**设定（源 S10 图2）**：参考帧相机中心 $C_r$、当前帧相机中心 $C_k$，两者位姿由 $\mathbf T_{k,r}$ 关联。设当前对像素 $u$（在 $I_r$ 中）的场景点估计为 $^r\mathbf p$（参考帧坐标系下）。相机中心 $C_r,C_k$ 与场景点 $^r\mathbf p$ 共面（**极平面**）。要估计的是：**当匹配点在当前帧极线上偏移"一个像素"时，深度 $\|^r\mathbf p\|$ 会变化多少**——这个变化量（的平方）即测量方差 $\tau_k^2$。

**符号（源 S10 式13-19，逐字）**：
- $\mathbf t$ = $\mathbf T_{k,r}$ 的平移分量（两相机中心连线向量，即基线）。
- $f=\dfrac{^r\mathbf p}{\|^r\mathbf p\|}$ = 参考帧下指向场景点的**单位光线方向**（注意：S10 此处 $f$ 是单位向量，非焦距；焦距另记，见式16）。
- 定义向量
$$
\mathbf a = {}^r\mathbf p - \mathbf t. \tag{S10-13}
$$
（$\mathbf a$ 是"从当前相机中心 $C_k$ 指向场景点"的向量在参考系下的表示——因为 $^r\mathbf p$ 是 $C_r\to$ 点，减去基线 $\mathbf t$ 即 $C_k\to$ 点。）

- 两个角（用点积定义）：
$$
\alpha = \arccos\!\left(\frac{f\cdot \mathbf t}{\|\mathbf t\|}\right), \tag{S10-14}
$$
$$
\beta = \arccos\!\left(-\,\frac{\mathbf a\cdot \mathbf t}{\|\mathbf a\|\,\|\mathbf t\|}\right). \tag{S10-15}
$$
其中 $\alpha$ 是参考光线 $^r\mathbf p$ 与基线 $\mathbf t$ 的夹角；$\beta$ 是当前光线 $\mathbf a$ 与基线 $-\mathbf t$ 的夹角（即极平面三角形在 $C_k$ 处的内角）。

- **一个像素对应的角增量**：设相机焦距为 $f_{\text{focal}}$（S10 式16 中的 "camera focal length"，与上面单位向量 $f$ 区分）。一个像素张成的角度约为 $2\tan^{-1}\!\frac{1}{2f_{\text{focal}}}$。把这个角加到 $\beta$ 上得到 $\beta^+$：
$$
\beta^+ = \beta + 2\tan^{-1}\!\left(\frac{1}{2 f_{\text{focal}}}\right). \tag{S10-16}
$$

- 由极平面三角形内角和，第三角
$$
\gamma = \pi - \alpha - \beta^+. \tag{S10-17}
$$

- 由**正弦定理**，恢复"偏移一个像素后"的场景点向量模长 $\|^r\mathbf p^+\|$：
$$
\|^r\mathbf p^+\| = \|\mathbf t\|\,\frac{\sin\beta^+}{\sin\gamma}. \tag{S10-18}
$$
（三角形三顶点 $C_r,C_k,$ 点；边 $\|\mathbf t\|$ 对角 $\gamma$，边 $\|^r\mathbf p^+\|$ 对角 $\beta^+$。）

- **测量不确定度（深度方差）= 偏一个像素引起的深度变化（之平方）**：
$$
\boxed{\ \tau_k^2 = \bigl(\|^r\mathbf p^+\| - \|^r\mathbf p\|\bigr)^{2}.\ } \tag{S10-19}
$$

> **源 S10 图2注（逐字保真）**："Computation of the measurement uncertainty. The camera poses acquiring the views $I_r$ and $I_k$ are related by the transformation $T_{k,r}$. The camera centres $C_r, C_k$ and the current estimation of the scene point $^r p$ lie on the epipolar plane. The variance corresponding to one pixel along the epipolar line passing through $e_0$ and $u_0$ is computed as $\tau_k^2 = (\|^r p_+\| - \|^r p\|)^2$."
>
> **源 S10 §II.C（逐字）**："Let $^r p$ be the current estimation of the scene point corresponding to the pixel $u$ in the image $I_r$. The variance on the position of $^r p$ is obtained by back-projecting a constant variance of one pixel in the image $I_k$. Let $t$ be the translation component of $T_{k,r}$ and $f=\frac{^r p}{\|^r p\|}$ ..."

**S11《十四讲》§13.2.3 的等价叙述（保真）**：十四讲把上面的向量记为 $\mathbf a=\mathbf p-\mathbf t$、$\mathbf p$=参考帧下场景点、$\mathbf t$=平移；先算 $\mathbf p$、$\mathbf a$ 的模和 $\mathbf t$ 的模；用点积得 $\alpha=\arccos\langle \mathbf p,\mathbf t\rangle$、$\beta=\arccos\langle \mathbf a,-\mathbf t\rangle$；扰动一个像素得 $\beta'=\beta+\arctan\frac{1}{f}$（十四讲取单像素张角 $\approx\arctan\frac1f$，与 S10 的 $2\tan^{-1}\frac{1}{2f}$ 同阶等价）；$\gamma=\pi-\alpha-\beta'$；正弦定理 $\|\mathbf p'\|=\|\mathbf t\|\frac{\sin\beta'}{\sin\gamma}$；不确定度 $\sigma_{\text{obs}}=\|\mathbf p\|-\|\mathbf p'\|$（十四讲取深度方向的一维标准差，REMODE 取平方为方差 $\tau_k^2$）。

> **本质洞察（综合 S10/S11）**：
> - **基线越大、角 $\alpha$ 越大（视差大）→ $\tau_k^2$ 越小（深度越可信）**；基线趋零时三角形退化、$\tau_k^2\to\infty$（纯旋转无法测深）。这定量解释了"为什么需要足够平移基线"。
> - 但**基线大 → 块匹配遮挡/形变风险大、匹配易错**。这就是 S10 §II.C 强调的权衡："frames taken from nearby vantage points are less affected by occlusions ...; a large baseline enables a more reliable depth estimation but with a higher chance to incur in occluded regions." **小基线匹配稳但深度糙、大基线深度准但匹配险**——多帧滤波正是为兼得两者。

### 3.6 ⭐深度滤波器：高斯深度融合（源 S11 §13.2.3）与严格的 Gaussian×Beta 模型（源 S9/S10）

#### 3.6.1 十四讲的简化版：纯高斯深度融合（源 S11 §13.2.3）

S11 把像素深度建模为**高斯分布** $d\sim\mathcal N(\mu,\sigma^2)$。每来一次观测，三角化得均值 $d_{\text{obs}}$、不确定度 $\tau^2$（=上节 $\tau_k^2$）。新观测 $\mathcal N(d_{\text{obs}},\tau^2)$ 与先验 $\mathcal N(\mu,\sigma^2)$ **相乘**（两高斯之积仍是高斯），得融合后高斯 $\mathcal N(\mu_{\text{fuse}},\sigma_{\text{fuse}}^2)$：
$$
\boxed{\ \mu_{\text{fuse}}=\frac{\sigma^2\, d_{\text{obs}} + \tau^2\,\mu}{\sigma^2+\tau^2},\qquad
\sigma_{\text{fuse}}^2=\frac{\sigma^2\,\tau^2}{\sigma^2+\tau^2}.\ }
$$
（即信息形式 $\frac{1}{\sigma_{\text{fuse}}^2}=\frac{1}{\sigma^2}+\frac{1}{\tau^2}$，均值是按精度加权平均——这就是一维卡尔曼/高斯乘积的标准结论。）

**收敛判据（源 S11）**：当 $\sigma_{\text{fuse}}^2$ 降到阈值以下，认为该像素深度收敛，固定其值、停止更新，并写入稠密点云/深度图。

> **推导补全（高斯乘积，源 S11 引用标准结论）**：两高斯 $\mathcal N(a,A)$、$\mathcal N(b,B)$ 之积 $\propto\mathcal N(c,C)$，$C=\frac{AB}{A+B}$、$c=\frac{Ab+Ba}{A+B}$。代 $a=\mu,A=\sigma^2,b=d_{\text{obs}},B=\tau^2$ 即得上式。属"假定背景"标准代数，本书估计章已证，此处直接用。

#### 3.6.2 严格版：Gaussian + Uniform 混合测量模型 + Gaussian×Beta 后验（源 S9 式1-4，S10 式2-4）

> 十四讲的纯高斯版**没有显式建模外点（误匹配）**。S9/S10 的原版用**混合模型 + 内点率**，能在概率层面甄别误匹配——这是 SVO/REMODE/DSO 等系统实际用的"深度滤波器"。本抽取据原论文逐式保真。

**测量模型（源 S9 式1 / S10 式2，逐字保真）**：第 $n$ 次（或第 $k$ 帧）三角化得到深度测量 $x_n$（S10 记 $d_k$）。传感器以概率 $\pi$（S10 记 $\rho$）产生"好测量"（绕真深 $Z$（S10 $\hat d$）的高斯），以 $1-\pi$ 产生"外点"（在 $[Z_{\min},Z_{\max}]$（S10 $[d_{\min},d_{\max}]$）上均匀）：
$$
p(x_n\mid Z,\pi)=\pi\,\mathcal N\!\bigl(x_n\mid Z,\tau_n^2\bigr)+(1-\pi)\,\mathcal U\!\bigl(x_n\mid Z_{\min},Z_{\max}\bigr). \tag{S9-1}
$$
（S10 式2 同形：$p(d_k\mid\hat d,\rho)=\rho\,\mathcal N(d_k\mid\hat d,\tau_k^2)+(1-\rho)\,\mathcal U(d_k\mid d_{\min},d_{\max})$。）其中**好测量方差 $\tau_n^2$ 即 §3.5 由"一像素视差"几何反投影算出的 $\tau_k^2$**（S9 原文："The variance of a good measurement $\tau_n^2$ can be obtained from the relative position of the cameras ... we assume that the measurement $x_n$ has a fixed variance of one pixel when projected in $I'$. We then back-project this variance in 3d space..."）。

**贝叶斯后验（源 S9 式2 / S10 式3，逐字）**：测量独立，
$$
p(Z,\pi\mid x_1,\dots,x_N)\ \propto\ p(Z,\pi)\prod_{n} p(x_n\mid Z,\pi), \tag{S9-2}
$$
$p(Z,\pi)$ 为深度与内点率的先验（取均匀）。**完整后验是多模态的、且需二维直方图存储**（S9：250,000 个种子 × 500 深度档 × 100 内点率档 ⇒ 125 亿 float，不可行）。

**参数化近似（源 S9 式3 / S10 式4，逐字保真）**：用 **"深度高斯 × 内点率 Beta"** 近似后验：
$$
q(Z,\pi\mid a_n,b_n,\mu_n,\sigma_n)\ :=\ \mathrm{Beta}(\pi\mid a_n,b_n)\,\cdot\,\mathcal N\!\bigl(Z\mid \mu_n,\sigma_n^2\bigr). \tag{S9-3}
$$
（S10 式4 同形：$q(\hat d,\rho\mid a_k,b_k,\mu_k,\sigma_k^2)=\mathrm{Beta}(\rho\mid a_k,b_k)\,\mathcal N(\hat d\mid\mu_k,\sigma_k^2)$。）
- $\mu_n,\sigma_n^2$：深度估计的均值与方差。
- $a_n,b_n$：可理解为该种子生命周期内累积的**内点计数 / 外点计数**（"probabilistic counters of how many inlier and outlier measurements have occurred"）。
- **为何选此形式**：S9 在补充材料用变分论证——在满足弱因子分解性质的一大类近似分布中，**Gaussian×Beta 与真后验的 KL 散度最小**。

**序贯（矩匹配）更新（源 S9 式4，逐字保真）**：若 $q(\cdot\mid a_{n-1},b_{n-1},\mu_{n-1},\sigma_{n-1}^2)$ 是前 $n-1$ 次测量后的（近似）后验，则观测 $x_n$ 后新后验
$$
C\times p(x_n\mid Z,\pi)\,q(Z,\pi\mid a_{n-1},b_{n-1},\mu_{n-1},\sigma_{n-1}^2) \tag{S9-4}
$$
（$C$ 为归一化常数）**不再是 Gaussian×Beta 形式**，于是用**矩匹配（moment matching）**：令新参数 $a_n,b_n,\mu_n,\sigma_n^2$ 使近似 $q(\cdot\mid a_n,b_n,\mu_n,\sigma_n^2)$ 与式(S9-4) 对 $Z$ 和 $\pi$ 的**一、二阶矩相等**。S9 原文："This update is straightforward to calculate analytically but we refer the reader to the supplementary material for the actual formulae." 即闭式更新公式在补充材料（其形式：用混合权重 $C_1,C_2$（内/外点后验责任）加权更新 $\mu,\sigma^2$ 与 $a,b$）。

> **综合 agent 备注（缺口诚实标注）**：S9/S10 正文**未把矩匹配的逐项闭式更新公式列出**（在 supplementary）。本套抽取无该补充材料。若成书要给"完整闭式更新公式"，需另取 SVO 源码 `depth_filter.cpp`（uzh-rpg/rpg_svo，函数 `updateSeed`）或 Vogiatzis-Hernández 补充材料；其标准形为（记 $s^2=\sigma_{n-1}^2+\tau_n^2$，先算内点责任 $C_1\propto \frac{a_{n-1}}{a_{n-1}+b_{n-1}}\mathcal N(x_n\mid\mu_{n-1},s^2)$、外点责任 $C_2\propto\frac{b_{n-1}}{a_{n-1}+b_{n-1}}\mathcal U$，归一化 $C_1+C_2=1$）：
> $$ m=\mu_{n-1}+\frac{\sigma_{n-1}^2}{s^2}(x_n-\mu_{n-1}),\quad s_{\text{new}}^2=\sigma_{n-1}^2-\frac{\sigma_{n-1}^4}{s^2}, $$
> $$ \mu_n=C_1 m + C_2\mu_{n-1},\quad \sigma_n^2=C_1(s_{\text{new}}^2+m^2)+C_2(\sigma_{n-1}^2+\mu_{n-1}^2)-\mu_n^2, $$
> 并对 $a,b$ 做相应矩匹配更新。**此式标 `\rebuilt` 待核**（源自实现/补充材料而非正文逐字），综合时务必对照 SVO 源码核验。

**初始化与收敛判据（源 S10 §III.B，逐字保真）**：
- 初值：$a_0=10,\ b_0=10,\ \mu_0=0.5(d_{\min}+d_{\max}),\ \sigma_0=\sigma_{\max}$，其中 $\sigma_{\max}$ 取使 99% 概率质量落在 $[d_{\min},d_{\max}]$。
- 三态判据（$E_\rho[q]$=内点率期望，阈 $\eta_{\text{inlier}},\eta_{\text{outlier}}$；$\sigma_{\text{thr}}$=方差阈）：
  - 若 $E_\rho[q]>\eta_{\text{inlier}}$ 且 $\sigma_k^2<\sigma_{\text{thr}}^2$ → **收敛**（接受深度）；
  - 否则若 $E_\rho[q]<\eta_{\text{outlier}}$ → **发散**（丢弃）；
  - 否则继续。
- 为适配大深度范围，**用逆深度（inverse depth）参数化**，并用当前方差限制极线搜索长度。

> **REMODE 的额外一步——TV 正则去噪（源 S10 §II.B.2，式6-8，保真）**：S10 在逐像素贝叶斯估计之上加**空间正则**，得去噪深度图 $F(u)$：
> $$ \min_F \int_\Omega\Bigl\{ G(u)\,\|\nabla F(u)\|_\epsilon + \lambda\,\|F(u)-D(u)\|_1 \Bigr\}\,du, \tag{S10-6} $$
> Huber 范数
> $$ \|\nabla F(u)\|_\epsilon=\begin{cases}\dfrac{\|\nabla F(u)\|_2^2}{2\epsilon}, & \|\nabla F(u)\|_2\le\epsilon,\\[4pt]\|\nabla F(u)\|_1-\dfrac{\epsilon}{2}, & \text{otherwise},\end{cases} \tag{S10-7} $$
> 置信加权 $G(u)=E_\rho[q](u)\dfrac{\sigma^2(u)}{\sigma_{\max}^2}+\{1-E_\rho[q](u)\}$（式8），即**可信像素少正则、不可信像素多正则**。$\lambda$ 平衡数据项与正则项。用原对偶（primal-dual）迭代求解（式9-12，proximal/shrink 算子）。**REMODE = REgularized MOnocular DEnse**，正则是其相对 S9 的主要增量。这部分超出十四讲范围，综合时可作为"前沿/选读"简介。

### 3.7 单目稠密重建：完整算法（伪码，综合 S11 §13.2 + S10）

```text
输入: 参考帧 I_r 及其位姿; 后续帧流 {I_k, T_{k,r}}; 深度范围 [d_min, d_max]
输出: I_r 上的稠密深度图 / 点云

# 初始化（对 I_r 每个像素 u）
for each pixel u in I_r:
    a0=10; b0=10                          # Beta 内/外点计数 (S10)
    mu[u]   = 0.5*(d_min + d_max)         # 深度均值
    sigma2[u]= sigma_max^2                # 深度方差(很大)
    state[u]= UPDATING

# 逐帧更新
for each new frame (I_k, T_{k,r}):
    for each pixel u with state[u]==UPDATING:
        # 1. 极线搜索: 把 [mu-3σ, mu+3σ] (或 [d_min,d_max]) 投到 I_k 得极线段
        seg = project_epipolar_segment(u, mu[u], sigma2[u], T_{k,r}, K)
        # 2. 块匹配: 沿 seg 取 w×w 块, 算 NCC, 取最大
        (u2, score) = argmax_NCC(patch(I_r,u), I_k, seg)
        if score < ncc_thresh: continue           # 弱匹配, 跳过(将计入外点)
        # 3. 三角化: 由 x1=K^{-1}u, x2=K^{-1}u2, (R,t)=T_{k,r} 解深度
        d_obs = triangulate(u, u2, T_{k,r}, K)
        # 4. 几何不确定度 τ^2 : S10 式13-19 (一像素视差→深度方差)
        tau2  = depth_uncertainty(u, d_obs, T_{k,r}, f_focal)   # 式13-19
        # 5. 融合(简化高斯版, S11)：
        mu_new     = (sigma2[u]*d_obs + tau2*mu[u]) / (sigma2[u]+tau2)
        sigma2_new = (sigma2[u]*tau2) / (sigma2[u]+tau2)
        mu[u]=mu_new; sigma2[u]=sigma2_new
        #   (严格版改用 Gaussian×Beta 矩匹配, 同时更新 a,b; S9 式1-4)
        # 6. 收敛检测
        if sigma2[u] < sigma_thr^2:  state[u]=CONVERGED   # 接受
        # (严格版: 若内点率期望过低 → state=DIVERGED, 丢弃)
# 输出: 对 CONVERGED 像素 u, 用 mu[u] 反投影成 3D 点
```

> **工程陷阱清单（源 S11 §13.2.4「实验」与讨论，逐条保真）**：
> 1. **像素梯度问题**：无梯度（无纹理）像素深度无法估计——稠密重建在白墙/天空处必然失败，得到"空洞"。
> 2. **逆深度（inverse depth）**：远处点深度方差极大，**直接用深度高斯不合适**；改用逆深度 $\rho=1/d$ 建高斯，远近一致、更接近高斯（S10 §III.B 也强调用逆深度）。
> 3. **图像块去畸变/仿射变换**：大视角差时块会发生透视形变，简单平移块匹配失配——需对块做仿射 warp（用平面诱导单应）。
> 4. **并行化**：逐像素独立 → 天然 GPU 并行（S9/S10 均 CUDA 实现，S9 维护 250k 种子实时）。
> 5. **NCC 阈值与窗口大小**：窗口大→鲁棒但糊边、丢细节；小→保细节但易误匹配。
> 6. **运动需有平移**：纯旋转无基线，$\tau^2\to\infty$，无法建图（§3.5 已定量说明）。

---

# 第二部分 RGB-D / 深度相机稠密建图

> 主源：S11《十四讲》§13.3（点云/八叉树/TSDF 总览）；S1(OctoMap)、S5(KinectFusion)、S6(Curless-Levoy)、S7(ElasticFusion)、S8(Keller) 给硬核公式。RGB-D 直接给每像素深度，**无需§3 的极线搜索/滤波**，建图重点转为"**如何把多帧带噪点云融合成一致、紧凑、可查询的地图**"。

## 4 点云地图（Point Cloud Map）（源 S11 §13.3.1）

**最朴素的稠密地图**：把每帧 RGB-D 反投影成相机系点云，用位姿 $\mathbf T_{wc}$ 变到世界系，拼接累积。

- 单像素 $(u,v)$、深度 $d$ → 相机系点：$\mathbf p_c=d\,\mathbf K^{-1}\dot{\mathbf p}=\bigl(\frac{(u-c_x)d}{f_x},\frac{(v-c_y)d}{f_y},d\bigr)^\top$。
- 世界系：$\mathbf p_w=\mathbf R_{wc}\mathbf p_c+\mathbf t_{wc}$。
- 全图点云 = $\bigcup_k \{\mathbf p_w\}_k$，可附颜色（RGB）。

**后处理（源 S11）**：
- **外点滤波**：统计离群点去除（SOR）——按每点 $K$ 近邻平均距离的均值±$n\sigma$ 剔除孤立点；
- **降采样**：体素栅格滤波（voxel grid）——每个小立方体只留一个点（质心），控制密度与内存。
（这些滤波算法的细节见 `point_cloud_processing__pcl_icp.md`，本件不展开。）

> **点云地图的优缺点（源 S11，逐条保真）**：
> - 优：直观、生成简单、保留稠密外观（可直接看)。
> - 缺：① **体量随轨迹无上限增长**（无压缩）；② **无空间占据/自由语义**——只有"看到的表面点"，不知道"两点之间是空的还是没探索"，**不能直接用于导航/避障**；③ 有噪声、有重影（多帧未真正融合，只是叠加）。
> ⇒ 这三条缺点正是 OctoMap（占据 + 压缩）、TSDF（融合去噪 + 隐式表面）、surfel（带法线半径的面元融合）各自要解决的。

## 5 八叉树占据地图 OctoMap（源 S1 论文 + S2 教程 + S3/S4 源码）

> **OctoMap 解决点云地图的"无占据语义 + 无压缩"两大痛点**：用**八叉树**多分辨率组织空间，每个叶节点存**占据概率（以 log-odds 存）**，并用**贝叶斯递归 + clamping** 在线更新、用**剪枝**压缩。下文全部公式逐字来自 S1（已 `pdftotext` 核对）。

### 5.1 八叉树数据结构（源 S1 §3.1）

- **八叉树（octree）**：3D 空间层次细分树，每个内节点把所在立方体均分为 8 个子立方体（八个卦限），递归到叶。叶 = 最小体素（分辨率由树深决定）。
- **复杂度（源 S1 §3.1 逐字）**：含 $n$ 节点、深度 $d$ 的树，单次随机查询 $O(d)=O(\log n)$；深度优先遍历整树 $O(n)$。实际限制最大深度 $d_{\max}$（实验用 16），随机查询 $O(d_{\max})$=常数。深度 16、1 cm 分辨率可覆盖 $(655.36\,\mathrm m)^3$ 立方体。
- **内存（源 S1 §4.1）**：节点不存自身中心坐标与体素尺寸（遍历时可重建）；用"每节点 1 个子指针指向 8 指针数组"（仅有子时才分配），叶节点只存数据 + 1 空指针。叶节点占 8 字节（32 位架构）/16 字节（64 位）；内节点 40/80 字节。实测 80%-85% 节点为叶；该实现比"每节点 8 指针"省 60%-65% 内存。
- **多分辨率查询（源 S1 §3.3）**：因是层次结构，遍历到非叶的某深度即得**粗分辨率**地图。

### 5.2 ⭐占据概率的贝叶斯递归更新（源 S1 §3.2，式1）

OctoMap 用 Moravec-Elfes (1985) 的**占据栅格映射**。叶节点 $n$ 在测量序列 $z_{1:t}$ 下被占据的概率按下式递归估计（**源 S1 式(1)，逐字保真**）：
$$
P(n\mid z_{1:t})=\left[1+\frac{1-P(n\mid z_t)}{P(n\mid z_t)}\,\frac{1-P(n\mid z_{1:t-1})}{P(n\mid z_{1:t-1})}\,\frac{P(n)}{1-P(n)}\right]^{-1}. \tag{S1-1}
$$
**符号（源 S1 逐字）**：
- $P(n\mid z_{1:t})$：给定 $z_{1:t}$ 后 $n$ 占据的后验概率；
- $P(n\mid z_t)$：**逆传感器模型**——仅由当前测量 $z_t$ 给出的 $n$ 占据概率（传感器相关）；
- $P(n\mid z_{1:t-1})$：上一时刻后验（递归先验）；
- $P(n)$：**先验**占据概率。

这是标准**二值贝叶斯滤波（binary Bayes filter）**对静态占据状态的递归。

### 5.3 ⭐log-odds 加性更新（源 S1 §3.2，式2-3）

定义 log-odds（对数几率，**源 S1 式(3)**）：
$$
L(n)=\log\frac{P(n)}{1-P(n)}. \tag{S1-3}
$$
则式(S1-1) 可**重写为加法**（**源 S1 式(2)，逐字**）：
$$
\boxed{\ L(n\mid z_{1:t})=L(n\mid z_{1:t-1})+L(n\mid z_t).\ } \tag{S1-2}
$$
**意义（源 S1 逐字）**：把乘法换成加法，更新更快；若传感器模型预先算好，更新时连对数都不必算。log-odds 与概率可互转，故**每体素只存 log-odds 这一个 float**（不存概率）。当采用**均匀先验** $P(n)=0.5$ 时 $L(n)=0$（式(S1-2) 中的先验项消失，故写成两项相加）。

> **源 S1 补充洞察（逐字保真）**：对**对称传感器模型**（hit 与 miss 权重相等），这个概率更新等价于"数 hit/miss 次数"（同 Kelly et al. 2006）。

> **OctoMap 源码实现（源 S3 `octomap_utils.h`，逐字 C++）**：
> ```cpp
> inline float logodds(double probability){
>     return (float) log(probability/(1-probability));
> }
> inline double probability(double logodds){
>     return 1. - ( 1. / (1. + exp(logodds)));
> }
> ```
> 即 $L=\log\frac{p}{1-p}$、$p=1-\frac{1}{1+e^{L}}=\frac{e^L}{1+e^L}=\sigma(L)$（sigmoid）。两式互逆。

### 5.4 ⭐clamping 更新策略（可更新性 + 压缩）（源 S1 §3.2，式4）

**问题（源 S1 逐字）**：由式(S1-2)，要**翻转**一个已被观测 $k$ 次的体素状态，需再观测相反结果至少 $k$ 次——静态环境好，但**动态/变化环境适应太慢**（置信度无界增长）。

**Yguel et al. (2007) 的 clamping 策略（源 S1 式(4)，逐字保真）**：给 log-odds 设上下界 $l_{\min},l_{\max}$，更新改为
$$
\boxed{\ L(n\mid z_{1:t})=\max\!\Bigl(\min\!\bigl(L(n\mid z_{1:t-1})+L(n\mid z_t),\ l_{\max}\bigr),\ l_{\min}\Bigr).\ } \tag{S1-4}
$$
**两大好处（源 S1 逐字）**：① 地图置信度**有界**，从而能**快速适应环境变化**；② 配合剪枝（§5.6）可**压缩**。代价：靠近 0/1 的全概率信息丢失（有损），但**界内全概率保真**。

> **OctoMap 源码 clamping（源 S4 `OccupancyOcTreeBase.hxx`，逐字 C++）**：
> ```cpp
> void updateNodeLogOdds(NODE* occupancyNode, const float& update) const {
>   occupancyNode->addValue(update);                         // L += update
>   if (occupancyNode->getLogOdds() < this->clamping_thres_min) {
>     occupancyNode->setLogOdds(this->clamping_thres_min);
>     return;
>   }
>   if (occupancyNode->getLogOdds() > this->clamping_thres_max) {
>     occupancyNode->setLogOdds(this->clamping_thres_max);
>   }
> }
> ```
> 即先加增量、再夹到 $[l_{\min},l_{\max}]$，与式(S1-4) 一致。

### 5.5 逆传感器模型与全部默认参数（源 S1 §5.1 式7 + S2 教程）

**beam 逆传感器模型（源 S1 式(7)，逐字保真）**：射线投射（3D Bresenham / Amanatides-Woo）从传感器原点到测量端点，端点体素记"占据"、沿途体素记"自由"：
$$
L(n\mid z_t)=\begin{cases} l_{\text{occ}}, & \text{beam is reflected within volume (端点)},\\ l_{\text{free}}, & \text{beam traversed volume (穿过)}.\end{cases} \tag{S1-7}
$$

**⭐两套数值（务必区分，逐字保真）**：

| 参数 | **S1 论文实验值（激光雷达）** | **S2/库默认值（`octomap::OcTree` 构造）** |
|---|---|---|
| 先验 $P(n)$ | $0.5$（均匀） | $0.5$ |
| 占据更新概率 $p_{\text{hit}}$ | $0.7$（对应 $l_{\text{occ}}=0.85$） | `setProbHit(0.7)` |
| 自由更新概率 $p_{\text{miss}}$ | $0.4$（对应 $l_{\text{free}}=-0.4$） | `setProbMiss(0.3)` |
| clamping 下界 | $0.12$（概率）⇒ $l_{\min}=\mathrm{logodds}(0.12)$ | `setClampingThresMin(0.1)` |
| clamping 上界 | $0.97$（概率）⇒ $l_{\max}=\mathrm{logodds}(0.97)$ | `setClampingThresMax(0.95)` |
| 占据判定阈 | 0.5 | `setOccupancyThres(0.5)` |

> **源 S1 §5.1 逐字**："Throughout our experiments, we used log-odds values of $l_{\text{occ}}=0.85$ and $l_{\text{free}}=-0.4$, corresponding to probabilities of 0.7 and 0.4 for occupied and free volumes, respectively." 以及 clamping "corresponding to the probabilities of 0.12 and 0.97. We experimentally determined these values to work best..."
>
> **源 S2（RosCon 教程 slide，逐字）**：
> ```cpp
> octree.setOccupancyThres(0.5);
> octree.setProbHit(0.7);   // setProbMiss(0.3)
> octree.setClampingThresMin(0.1);  // setClampingThresMax(0.95)
> ```
>
> ⚠️ **综合 agent 注意**：论文(S1)与库默认(S2)的 $p_{\text{miss}}$（0.4 vs 0.3）、clamping（0.12/0.97 vs 0.1/0.95）**不同**——前者是论文激光雷达实验调优值，后者是库出厂默认。**本书引用时须注明用的是哪一套**，勿张冠李戴。换算可用 $l=\log\frac{p}{1-p}$：$\mathrm{logodds}(0.1)\approx-2.197$、$\mathrm{logodds}(0.95)\approx2.944$、$\mathrm{logodds}(0.7)\approx0.847$、$\mathrm{logodds}(0.3)\approx-0.847$。
>
> **稳定节点（源 S1 §3.4）**："With the parameters chosen in our experiments, for example, five agreeing measurements are sufficient to render an unknown voxel into a stable voxel." 即默认参数下 ~5 次一致观测即可让体素到达 clamping 界（"稳定"）。

**3D 扫描的薄面消失问题（源 S1 §5.1，逐字）**：扫掠激光在平坦面浅角扫描时，一帧标占据的体素可能被后续帧的射线投射标成自由，导致表面出现空洞（图10/11）。**解法**：把一次 sweep 的多条扫描线当作**单个点云**整体更新，且**同一次更新中，已标占据的体素不再标自由**（"whenever a voxel is updated as occupied ..., it is not updated as free in the same measurement update"）。

### 5.6 内节点聚合与剪枝压缩（源 S1 §3.3 式5-6, §3.4）

**内节点占据如何由 8 个子节点聚合（源 S1，两种策略，逐字保真）**：
- **平均占据（average，式5）**：
$$
\bar l(n)=\frac{1}{8}\sum_{i=1}^{8}L(n_i). \tag{S1-5}
$$
- **最大占据（maximum，式6）**：
$$
\hat l(n)=\max_i L(n_i). \tag{S1-6}
$$
$L(n_i)$ 为第 $i$ 个子体素当前 log-odds 占据值。

> **源 S1 §3.3 逐字**：导航用**最大占据**（保守——只要任一子体素被测占据就视整体占据，可规划无碰路径）："the maximum occupancy update is used in our system." 更保守设置可让 $L(n)$ 对未知体素也返回正占据。

**剪枝压缩（源 S1 §3.4，逐字）**：对**叶**做概率更新；当某内节点的 8 个子节点都"稳定"（log-odds 到达 $l_{\min}$ 或 $l_{\max}$）**且占据状态相同**时，剪掉这些子节点（合并为父）。若后续测量与父矛盾，则**重新生成**子节点。此压缩仅在接近 $P=0$、$P=1$ 处有损，界内保真。**剪枝 + clamping 联合，压缩率最多提升 44%**。最大似然地图（只存占据/自由）可进一步按占据阈做有损压缩 + 剪枝。

**序列化（源 S1 图7注，逐字）**：完整树可压成紧凑位流，每节点 8 个子用 2 bit 标签（`00`未知 / `01`占据 / `10`自由 / `11`内节点，子节点紧随其后）；图2 的整树可仅用 6 字节存全部最大似然占据信息。

> **OctoMap 优缺点小结（源 S1 + S11）**：
> - 优：**显式区分 占据/自由/未知 三态**（点云只有"占据"）；**多分辨率**；**概率更新可处理噪声与动态**（clamping）；**剪枝压缩省内存**；可序列化（.bt 二进制树极小）。
> - 缺：仅占据语义、**无表面法线/外观细节**（不适合精细重建/渲染）；分辨率固定后细节受限；动态物体留拖影需 clamping 缓解。
> - 用途：**导航 / 避障 / 运动规划的首选度量地图**。

## 6 TSDF 与 KinectFusion（体素截断符号距离 + 光线投射）（源 S6 Curless-Levoy + S5 KinectFusion）

> **TSDF 解决"如何把多帧带噪深度融合成光滑、去噪、可提网格的隐式表面"**。思想：不存"点"或"占据"，而在每个体素存它**到最近表面的（截断）符号距离** $F$ 与**累积权重** $W$；表面 = $F=0$ 的零交叉等值面。下文式(5)-(15) 逐字来自 S5（已 `pdftotext` 核对），原始累积加权 SDF 思想来自 S6（KinectFusion 引为 [7]）。

### 6.1 TSDF 的概念与符号约定（源 S5 §3.3，逐字）

**真符号距离函数（true SDF，源 S5 逐字）**："the value corresponds to the signed distance to the closest zero crossing (the surface interface), taking on **positive and increasing values moving from the visible surface into free space**, and **negative and decreasing values on the non-visible side**." 即：
- 体素在表面**前方（朝相机、自由空间）→ $F>0$**；
- 体素在表面**后方（不可见侧、物体内部）→ $F<0$**；
- $F=0$ 即表面。

**全局 TSDF（源 S5，逐字）**：把第 $1\dots k$ 帧配准深度融合后的全局 TSDF 记 $S_k(\mathbf p)$，$\mathbf p\in\mathbb R^3$ 为待重建体积中的全局点。离散化为 GPU 体素网格。每个体素位置存两个量（**源 S5 式(5)，逐字**）：
$$
S_k(\mathbf p)\ \mapsto\ \bigl[F_k(\mathbf p),\ W_k(\mathbf p)\bigr]. \tag{S5-5}
$$
$F_k(\mathbf p)$=当前截断符号距离值，$W_k(\mathbf p)$=累积权重。

### 6.2 ⭐截断与空间三约束（源 S5 §3.3，式9）

**为何截断（源 S5，逐字保真）**：一次稠密深度测量（原始深度图 $R_k$）对被重建表面给两个约束——
1. 沿每条深度射线，距相机中心 $r<(\lambda R_k(\mathbf u)-\mu)$ 处是**自由空间**（measurement of free space），其中 $\lambda=\|\mathbf K^{-1}\dot{\mathbf u}\|_2$ 把"沿像素射线的尺度"折算；
2. $r>(\lambda R_k(\mathbf u)+\mu)$ 处**无表面信息**（unknown）。

故 SDF 只需表示**不确定带** $|r-\lambda R_k(\mathbf u)|\le\mu$（$\mu$=截断半带宽）。可见侧距最近表面 $>\mu$ 处截到最大正值 $\mu$（归一化后 +1）；不可见侧距表面 $>\mu$ 处**不测量**（null）。

**截断函数（源 S5 式(9)，逐字保真）**：设 $\eta$ 为某体素沿射线的符号距离原值，截断算子
$$
\Psi(\eta)=\begin{cases} \min\!\bigl(1,\ \dfrac{\eta}{\mu}\bigr)\,\mathrm{sgn}(\eta), & \eta\ge -\mu,\\[6pt] \text{null}, & \text{otherwise}.\end{cases} \tag{S5-9}
$$
即：$\eta\ge-\mu$ 时归一化到 $[-1,1]$ 并保号（超过 $+\mu$ 截到 $+1$）；$\eta<-\mu$（表面后方太远、不可见）记 null（不更新）。

### 6.3 ⭐投影式 TSDF 测量（源 S5 §3.3，式6-8）

KinectFusion 不算真欧氏 SDF（太慢），用**投影式（projective）TSDF**——沿相机射线把"体素深度"与"该像素测得深度"相减。对位姿 $\mathbf T_{g,k}$ 已知的原始深度图 $R_k$，体素 $\mathbf p$ 的投影 TSDF $[F_{R_k},W_{R_k}]$（**源 S5 式(6)-(8)，逐字保真**）：
$$
F_{R_k}(\mathbf p)=\Psi\!\left(\lambda^{-1}\,\bigl\|\mathbf t_{g,k}-\mathbf p\bigr\|_2-R_k(\mathbf x)\right), \tag{S5-6}
$$
$$
\lambda=\bigl\|\mathbf K^{-1}\dot{\mathbf x}\bigr\|_2, \tag{S5-7}
$$
$$
\mathbf x=\Bigl\lfloor \pi\!\bigl(\mathbf K\,\mathbf T_{g,k}^{-1}\,\mathbf p\bigr)\Bigr\rfloor. \tag{S5-8}
$$
**符号（源 S5 逐字）**：
- $\mathbf t_{g,k}$：相机 $k$ 在全局系的平移（相机中心），$\|\mathbf t_{g,k}-\mathbf p\|_2$ 是体素到相机中心的距离（沿射线）；
- $\mathbf x$：体素 $\mathbf p$ 投影到第 $k$ 帧图像的像素坐标（$\pi$=去齐次投影 $\pi([x,y,z]^\top)=(x/z,y/z)^\top$，$\lfloor\cdot\rfloor$=最近邻取整，**不插值**以免深度不连续处糊化）；
- $R_k(\mathbf x)$：该像素测得深度；
- $\lambda^{-1}$：把"沿射线到 $\mathbf p$ 的距离"折算成"沿光轴的深度"，使二者可减（源 S5："$1/\lambda$ converts the ray distance to $\mathbf p$ to a depth"）。
- 故括号内 $=$（体素的深度）$-$（测得深度）$=$ 符号距离 $\eta$，再过 $\Psi$ 截断。

**测量权重（源 S5 §3.3，逐字）**：$W_{R_k}(\mathbf p)\propto \cos(\theta)/R_k(\mathbf x)$，$\theta$=该像素射线方向与局部表面法线的夹角（正对表面权重大、斜视权重小、远处权重小）。**但实践中令 $W_{R_k}(\mathbf p)=1$（简单平均）即效果良好**（"in practice simply letting $W_{R_k}(\mathbf p)=1$, resulting in a simple average, provides good results"）。

> **源 S5 逐字补充**：投影 TSDF 只在表面处 $F_{R_k}(\mathbf p)=0$ 精确；非表面处沿射线最近点可能不是该像素对应点，但"对来自多视角的上百帧融合后，截断带内会收敛到一种伪欧氏度量，不影响建图与跟踪"。

### 6.4 ⭐加权运行平均融合（源 S5 §3.3，式10-13；原始式见 S6）

**全局融合 = 各帧 TSDF 的加权平均**（可视为对多帧噪声 TSDF 去噪）。L2 范数下，去噪表面是逐点 SDF $F$ 的零交叉，$F$ 最小化（**源 S5 式(10)，逐字保真**）：
$$
\min_{F\in\mathbb F}\ \sum_k \bigl\|W_{R_k}\,(F_{R_k}-F)\bigr\|^2. \tag{S5-10}
$$
该凸 L2 去噪问题的全局解**可增量地用简单加权运行平均求得**（逐点，对 $\{\mathbf p\mid F_{R_k}(\mathbf p)\ne\text{null}\}$）（**源 S5 式(11)-(12)，逐字保真**）：
$$
\boxed{\ F_k(\mathbf p)=\frac{W_{k-1}(\mathbf p)\,F_{k-1}(\mathbf p)+W_{R_k}(\mathbf p)\,F_{R_k}(\mathbf p)}{W_{k-1}(\mathbf p)+W_{R_k}(\mathbf p)},\ } \tag{S5-11}
$$
$$
\boxed{\ W_k(\mathbf p)=W_{k-1}(\mathbf p)+W_{R_k}(\mathbf p).\ } \tag{S5-12}
$$
对式(S5-9) 判为不可测（null）的体素**不更新**。

**权重截断（动态场景的移动平均）（源 S5 式(13)，逐字保真）**：把累积权重封顶在 $W_\eta$：
$$
W_k(\mathbf p)\leftarrow \min\!\bigl(W_{k-1}(\mathbf p)+W_{R_k}(\mathbf p),\ W_\eta\bigr). \tag{S5-13}
$$
这把"全历史平均"变成"**移动平均（moving average）**"——旧观测随新观测淡出，从而**支持动态物体运动的场景重建**。

> **与 OctoMap clamping 的对照（综合洞察）**：式(S5-13) 的 $W_\eta$ 截断在精神上等同于 OctoMap 的 log-odds clamping（式S1-4）——**都给"置信度"封顶，以换取对环境变化的快速适应**；前者封 TSDF 权重，后者封占据 log-odds。
>
> **与 Curless-Levoy（S6）的关系**：式(S5-11)(S5-12) 正是 KinectFusion 引用的 [7]=Curless-Levoy 1996 的**累积加权符号距离函数**增量更新。S6 原式记号为 $D(\mathbf x),W(\mathbf x)$：$D_{k}(\mathbf x)=\frac{W_{k-1}(\mathbf x)D_{k-1}(\mathbf x)+w_k(\mathbf x)d_k(\mathbf x)}{W_{k-1}(\mathbf x)+w_k(\mathbf x)}$、$W_k(\mathbf x)=W_{k-1}(\mathbf x)+w_k(\mathbf x)$——与 S5 同构（$D\equiv F$）。S6 是 TSDF 的**鼻祖**（1996 年用于多幅 range image 融合 + Marching Cubes 提网格），KinectFusion 把它搬到 GPU 实时 + 投影式近似 + 光线投射跟踪。

**精度/内存（源 S5，逐字）**：$S(\mathbf p)$ 每分量用 16 bit（实验证实 SDF 值仅需 6 bit）；现代 GPU >65 gigavoxels/s（$512^3$ 体积全量更新 ≈2 ms/帧）。640×480@30fps ⇒ 每秒 >900 万新测量点。

### 6.5 ⭐光线投射提取表面（Raycasting）（源 S5 §3.4，式14-15）

有了全局 SDF，可**逐像素光线投射**渲染零等值面 $F_k=0$，得预测的顶点图 $\hat{\mathbf V}_k$、法线图 $\hat{\mathbf N}_k$（用于下一帧 ICP 跟踪）。**算法（源 S5 §3.4，逐字保真）**：
- 每像素 $\mathbf u$ 的射线 $\mathbf T_{g,k}\mathbf K^{-1}\dot{\mathbf u}$，从最小深度起**步进（march）**；
- 当 $F$ 由 **+ 变 −**（正到负）出现零交叉 → 找到可见表面；
- 若 **− 变 +**（背面）或步出工作体积 → 无表面测量。

**零交叉的亚体素插值（源 S5 式(15)，逐字保真）**：设步进到 $t$ 时 SDF 值 $F_t^+$、再走 $\Delta t$ 到 $F_{t+\Delta t}^+$，则更精确的交点参数
$$
t^*=t-\frac{\Delta t\,F_t^+}{F_{t+\Delta t}^+-F_t^+}. \tag{S5-15}
$$
（线性插值求 $F=0$ 处。）**加速（源 S5）**：因截断 SDF 值是"到最近表面距离的保守估计"，可在远离表面处沿射线**大步前进**（"larger steps"），近表面再细步。

**表面法线（源 S5 §3.4，逐字）**：在 $F_k(\mathbf p)=0$ 处假设 TSDF 梯度正交于零等值面，故像素 $\mathbf u$ 的表面法线由**TSDF 的数值梯度** $\nabla F$ 归一化给出：$\hat{\mathbf N}_k(\mathbf u)=\frac{\nabla F}{\|\nabla F\|}$（梯度由相邻体素差分）。

> **KinectFusion 整体流水线（源 S5 §3，图3，逐字 4 步）**：
> 1. **Surface measurement**：对原始深度图 $R_k$ 做双边滤波得 $D_k$，反投影成顶点图 $\mathbf V_k$（式3）、由邻域叉积得法线图 $\mathbf N_k$。（注：式2 的双边滤波只用于**跟踪**，TSDF **融合用原始深度** $R_k$ 而非滤波后，以保细节。）
> 2. **ICP**：预测表面（上一帧光投得的 $\hat{\mathbf V},\hat{\mathbf N}$）与当前测量表面做**point-to-plane ICP**（投影数据关联），求当前位姿 $\mathbf T_{g,k}$。
> 3. **Integrate**：按式(S5-6)-(S5-13) 把当前深度融入全局 TSDF。
> 4. **Raycast**：从全局 TSDF 光投出新的预测表面，供下一帧 ICP（即 **frame-to-model** 跟踪，而非 frame-to-frame，抑制漂移）。

> **TSDF/KinectFusion 优缺点（源 S5 + S11）**：
> - 优：**多帧加权融合天然去噪**；**隐式表面 + 零交叉**可直接 Marching Cubes 提光滑网格；**frame-to-model** ICP 精度高、漂移小；GPU 实时；可表示任意亏格闭合表面。
> - 缺：**稠密体素内存随体积立方增长**（$512^3$×几字节即上百 MB；大场景需 voxel hashing / 移动体积 Kintinuous）；**预设固定体积、固定分辨率**；无回环（原版），长轨迹漂移累积；动态物体需 $W_\eta$ 移动平均缓解。
> - 用途：**桌面级/房间级高精度稠密重建、AR**。

## 7 Surfel 面元地图与 ElasticFusion（源 S7 论文 + S8 Keller 融合算法）

> **Surfel（surface element，面元）= 一种"无连接的、带朝向与大小的点"表示**：每个 surfel 是一个局部小圆盘（面片）。相比 TSDF 体素，surfel **只在表面处分配元素**（无空体素），内存更省、且天然适配**非刚性形变回环**（ElasticFusion 无位姿图，直接形变 surfel 云）。下文 surfel 属性与 active/inactive 逐字来自 S7；融合公式 S7 甩给 S8（Keller），本抽取据 S8/InfiniTAM 给标准形并标注。

### 7.1 ⭐Surfel 的属性集合（源 S7 §III，逐字保真）

场景表示为**无序 surfel 列表** $\mathcal M$。每个 surfel $\mathcal M^s$ 有如下属性（**源 S7 逐字**）：
$$
\mathcal M^s=\Bigl\{\ \underbrace{\mathbf p\in\mathbb R^3}_{\text{位置}},\ \underbrace{\mathbf n\in\mathbb R^3}_{\text{法线}},\ \underbrace{\mathbf c\in\mathbb N^3}_{\text{颜色}},\ \underbrace{w\in\mathbb R}_{\text{权重/置信}},\ \underbrace{r\in\mathbb R}_{\text{半径}},\ \underbrace{t_0}_{\text{创建时刻}},\ \underbrace{t}_{\text{末次更新时刻}}\ \Bigr\}.
$$
> **源 S7 逐字**："each surfel $\mathcal M^s$ has the following attributes; a position $\mathbf p\in\mathbb R^3$, normal $\mathbf n\in\mathbb R^3$, colour $\mathbf c\in\mathbb N^3$, weight $w\in\mathbb R$, radius $r\in\mathbb R$, initialisation timestamp $t_0$ and last updated timestamp $t$."
- **半径 $r$**（源 S7 逐字）："The radius of each surfel is intended to represent the local surface area around a given point while minimising visible holes, computed as done by Salas-Moreno et al."（即 surfel 圆盘大小，使其拼起来无可见空洞）。
- **权重 $w$**（源 S8/S9 语义）："the weight of a surfel signifies the level of confidence in the estimation of its parameters."（参数估计置信度）。

### 7.2 Surfel 半径、权重与融合更新（源 S8 Keller；S7 "follows the same rules as Keller et al."）

> **S7 原文明确把 surfel 初始化与深度图融合规则甩给 S8（Keller et al. [9]）**："Our system follows the same rules as described by Keller et al. for performing surfel initialisation and depth map fusion (where surfel colours follow the same moving average scheme)." 故以下公式来自 S8（Keller 2013）的标准形式，本抽取据 S8 论文与 InfiniTAM 实现给出，**标 `\rebuilt` 待综合 agent 对照 S8 原文/源码核验**。

**(a) surfel 半径（源 S8，`\rebuilt`）**：由深度 $d$、焦距 $f$、法线与视线夹角决定，使圆盘投影约覆盖一个像素：
$$
r=\frac{\sqrt{2}\,d}{f\,|n_z|}\quad(\text{常见形})\qquad\text{或}\qquad r=\frac{d}{\sqrt 2\,f\,|n_z|},
$$
其中 $n_z$ 是法线在相机光轴方向分量（$|n_z|$ 小=斜视=圆盘大）。**两种系数形式在不同文献/实现中均出现，综合时须按 S8 原式定一种**。

**(b) 测量置信权重（源 S8，`\rebuilt`）**：新测量的置信权重随该像素到相机光心射线的径向距离衰减（中心可信、边缘不可信）：
$$
w_{\text{new}}=\exp\!\Bigl(-\frac{\gamma^2}{2\sigma^2}\Bigr),\qquad \gamma=\frac{\|(u,v)-(c_x,c_y)\|}{\text{(归一化)}},
$$
$\gamma$ 为像素到主点的归一化径向距离，$\sigma$ 常取 0.6（Keller）。（部分实现简化为 $w_{\text{new}}=1$。）

**(c) surfel 与新测量的融合（加权平均，源 S8，`\rebuilt`）**：当新深度像素经投影数据关联匹配到已有 surfel（位置近、法线一致、半径相容），用**加权平均**更新位置、法线、颜色，并累加权重：
$$
\mathbf p\leftarrow\frac{w\,\mathbf p+w_{\text{new}}\,\mathbf p_{\text{new}}}{w+w_{\text{new}}},\quad
\mathbf n\leftarrow\frac{w\,\mathbf n+w_{\text{new}}\,\mathbf n_{\text{new}}}{w+w_{\text{new}}},\quad
\mathbf c\leftarrow\frac{w\,\mathbf c+w_{\text{new}}\,\mathbf c_{\text{new}}}{w+w_{\text{new}}},
$$
$$
w\leftarrow w+w_{\text{new}},\qquad t\leftarrow t_{\text{current}}.
$$
（半径取较小者或加权平均；**这与 TSDF 式(S5-11)(S5-12) 同构——都是"加权运行平均 + 累积权重"**。）

**(d) 新增 vs 更新 vs 剔除（源 S8/S7 逐字要点）**：
- 若新像素**无匹配 surfel** → **新建** surfel（$w=w_{\text{new}}$，$t_0=t=$ 当前帧）。
- 若**有匹配** → 按 (c) **融合更新**。
- **剔除（culling）**：长期低置信（$w$ 小）或不稳定的 surfel 删除（S7 提及 "surfel culling"）。

### 7.3 ⭐Active / Inactive 划分（源 S7 §III，逐字保真）

ElasticFusion 用时间窗阈 $\delta_t$ 把地图 $\mathcal M$ 分为**活跃（active）**与**非活跃（inactive）**（**源 S7 逐字**）：
- **只有 active surfel 用于位姿估计与深度融合**（"Only surfels which are marked as active model surfels are used for camera pose estimation and depth map fusion."）；
- 一个 surfel **末次更新（有原始深度关联融合）距今 > $\delta_t$** 即标 inactive（"A surfel in $\mathcal M$ is declared as inactive when the time since that surfel was last updated ... is greater than $\delta_t$."）。

**回环（源 S7 §III/图2，逐字要点）**：每帧尝试把 active 模型投影渲染（surfel splatting）出的预测深度/颜色，与其下方的 inactive 模型对齐——
- 对齐成功 = **局部回环**：触发对整张 surfel 地图（active+inactive）的**非刚性空间形变（deformation graph）**，并把肇事 inactive 区**重新激活**（reactivate）。
- 也有**全局回环**（global loop closure）：用外观（ferns/随机蕨）检索历史，对齐 active 与 inactive 全局模型。
> **ElasticFusion 的核心创新（源 S7 摘要逐字）**：**"Dense SLAM Without A Pose Graph"**——不维护关键帧位姿图，而是**直接非刚性形变稠密 surfel 地图**来反映回环，始终贴近地图分布的众数（map-centric）。

### 7.4 ElasticFusion 的相机跟踪（几何 + 光度联合）（源 S7 §III，式2-7，逐字保真）

ElasticFusion 用 **frame-to-model**：当前 RGB-D 与 active 模型的 surfel-splatted 预测对齐。

**图像/反投影记号（源 S7 逐字）**：图像域 $\Omega\subset\mathbb N^2$；深度图 $D$、颜色图 $C$；反投影 $\mathbf p(\mathbf u,D)=\mathbf K^{-1}\mathbf u\,d(\mathbf u)$；投影 $\mathbf u=\pi(\mathbf K\mathbf p)$、$\pi([x,y,z]^\top)=(x/z,y/z)^\top$；像素强度 $I(\mathbf u,C)=(c_1+c_2+c_3)/3$。位姿增量用李代数 $\boldsymbol\xi=[\boldsymbol\omega^\top\ \mathbf x^\top]^\top$，$\boldsymbol\omega,\mathbf x\in\mathbb R^3$（**注意：旋转 $\boldsymbol\omega$ 在前、平移 $\mathbf x$ 在后——与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前相反**）。

**光度（强度）误差（源 S7 式(3)，逐字保真）**：
$$
E_{\text{rgb}}=\sum_{\mathbf u\in\Omega}\Bigl(I(\mathbf u,C_t^l)-I\bigl(\pi(\mathbf K\exp(\hat{\boldsymbol\xi})\mathbf T\,\mathbf p(\mathbf u,D_t^l)),\ \hat C_{t-1}^{a}\bigr)\Bigr)^2, \tag{S7-3}
$$
$C_t^l$=当前实测彩色、$\hat C_{t-1}^a$=上一帧 active 模型预测彩色，$\mathbf T$=上一帧到当前帧的当前估计，$\exp(\hat{\boldsymbol\xi})$=李代数 $\mathfrak{se}_3\to SE_3$ 指数映射。

**几何（ICP, point-to-plane）误差（源 S7 式(2)，结构保真）**：当前深度反投影点 $\mathbf v_t^k$ 与上一帧模型对应点 $\mathbf v^k$、法线 $\mathbf n^k$（投影数据关联）的点-面残差之和。

**联合代价与求解（源 S7 式(4)-(7)，逐字保真）**：
$$
E_{\text{track}}=E_{\text{icp}}+w_{\text{rgb}}\,E_{\text{rgb}},\qquad w_{\text{rgb}}=0.1, \tag{S7-4}
$$
高斯-牛顿、三层金字塔由粗到精；每次迭代解最小二乘
$$
\arg\min_{\boldsymbol\xi}\ \|\mathbf J\boldsymbol\xi+\mathbf r\|_2^2, \tag{S7-5}
$$
更新位姿
$$
\mathbf T'=\exp(\hat{\boldsymbol\xi})\,\mathbf T,\qquad \hat{\boldsymbol\xi}=\begin{bmatrix}[\boldsymbol\omega]_\times & \mathbf x\\ \mathbf 0 & 0\end{bmatrix}. \tag{S7-6,7}
$$
$6\times6$ 正规方程在 GPU 上树规约求和、CPU 上 Cholesky 解。最终位姿 $\mathbf P_t=\mathbf T\mathbf P_{t-1}$。

> **Surfel 地图优缺点（源 S7 + S8）**：
> - 优：**只在表面分配元素**（无空体素），内存随表面积而非体积增长；**带法线/半径/颜色**，渲染质量好；**无需位姿图，直接非刚性形变做回环**（ElasticFusion）；支持动态（active/inactive + culling）。
> - 缺：无显式自由/未知空间（不像 OctoMap 利于规划）；无连接（非网格），碰撞/物理需额外提网格；splatting 渲染与形变图实现复杂。
> - 用途：**房间级稠密 RGB-D SLAM + 重建（带回环）**。

---

# 第三部分 网格重建简介（从隐式场/点云到三角网格）

> 度量稠密地图（TSDF 体素 / surfel / 点云）常需转成**三角网格（mesh）**以利渲染、碰撞、物理、传输。两大经典：**Marching Cubes**（从体素标量场提等值面，配 TSDF 天作之合）与**泊松重建**（从定向点云解隐式场再提面）。

## 8 Marching Cubes（移动立方体）（源 S12 Lorensen-Cline 1987）

**用途**：从三维**标量场**（如 TSDF $F(\mathbf p)$、占据场、医学 CT 密度）提取**等值面（isosurface）** $F=\text{iso}$ 的三角网格。与 TSDF 的零交叉（iso=0）天然契合（S6 Curless-Levoy 即用 MC 从 $D(\mathbf x)$ 提面）。

**算法（源 S12 + Wikipedia，逐步保真）**：
1. **遍历体素立方体**：每次取 8 个相邻格点构成一个立方体单元。
2. **角点分类**：每个角点标量值与 iso 比较——大于 iso 记 1（内）、否则 0（外）。8 个角点 → **8 bit 索引**（$0\sim255$）。
3. **查边表 + 三角表**：8 bit 索引查预计算的 **256 项查找表**，得该立方体内等值面穿过哪些**边**、如何连成三角形。
4. **边上线性插值定顶点**：对每条被穿过的边（两端点 $\mathbf P_1,\mathbf P_2$、标量 $v_1,v_2$），等值面交点按**线性插值**：
$$
\boxed{\ \mathbf P=\mathbf P_1+\frac{\text{iso}-v_1}{v_2-v_1}\,(\mathbf P_2-\mathbf P_1).\ }
$$
5. 汇总所有立方体的三角形 → 完整网格。法线可由标量场梯度（中心差分）插值得到（利于光照）。

**配置数（源 S12，逐字保真）**：8 角点各 2 态 ⇒ $2^8=256$ 种配置；利用**旋转、反射对称 + 符号翻转**约简为 **15 个基本情形（unique cases）**。

**歧义问题与改进（源 Wikipedia/Chernyaev）**：原 MC 有**面歧义**（面四角符号交替）和**内部歧义**（仅靠角点符号不足以定唯一三角化），可能在表面产生空洞/不连续。**Marching Cubes 33**（Chernyaev 1995）把查找表扩到 33 情形、用**渐近判定器（asymptotic decider）**解内部歧义、保拓扑正确。

> **MC 优缺点**：
> - 优：简单、快、查表 O(1)/立方体、天然并行；与 TSDF 直接对接。
> - 缺：原版有歧义（需 MC33）；规则网格分辨率固定（细节受体素大小限）；可能产生大量小三角形（需后续简化/抽稀）。

## 9 泊松表面重建（Poisson Surface Reconstruction）（源 S13 Kazhdan 2006）

**用途**：从**定向点云**（每点带位置 + 向内法线）重建**水密（watertight）**三角网格。相比 MC（需先有体素标量场），泊松法**直接从点云解出隐式指示函数**再提面，对噪声鲁棒、产生光滑闭合面。

**核心思想（源 S13，逐字保真）**：要重建模型的**指示函数 $\chi$**（内部=1、外部=0）。关键观察——**指示函数的梯度 $\nabla\chi$ 几乎处处为零，仅在表面附近非零，且方向 = 表面向内法线**。于是把输入定向点的法线视作一个**向量场 $\vec V$**（指示函数梯度的样本）。求 $\chi$ 使 $\nabla\chi\approx\vec V$。两边取散度，得**泊松方程**：
$$
\boxed{\ \Delta\chi=\nabla\cdot\vec V,\ }
$$
$\Delta=\nabla\cdot\nabla$ 为拉普拉斯算子。解此泊松方程得 $\chi$，再对 $\chi$ 取**等值面**（用自适应 Marching Cubes，等值取 $\chi$ 在样本点处的平均值）得网格。

**实现（源 S13，逐字要点）**：$\chi$ 在**自适应八叉树**（非规则网格）上离散表示，用**多重网格（multigrid）**解线性系统；属**全局解**（一次性考虑所有点，无启发式分块/混合）；对带噪点云产生很光滑的逼近。

> **Screened Poisson（Kazhdan-Hoppe 2013）**：在泊松能量中加**点位插值约束项**（"screening"），使重建面更贴近输入点（原版偏光滑、会缩进），是现今 MeshLab 等的常用版本。

> **泊松 vs Marching Cubes（综合）**：
> - MC：输入=**体素标量场**（TSDF/占据）；局部、逐立方体；适合"已有体素场"（KinectFusion 直接 MC）。
> - 泊松：输入=**定向点云**；全局解隐式场；适合"只有带法线点云"（激光/多视点云）；水密、抗噪、光滑，但需可靠法线、计算重、可能过度光滑（screened 版缓解）。

---

## 10 工程权衡总表（综合 S1/S5/S7/S11/S14）

| 表示 | 内存随… | 占据/自由/未知 | 表面细节/法线 | 去噪/融合 | 回环友好 | 实时性 | 主要用途 | 典型系统 |
|---|---|---|---|---|---|---|---|---|
| **点云** | 轨迹/观测数（无界） | 无（仅占据点） | 点级、无法线（可估） | 无（仅叠加） | 差（拼接） | 高 | 可视化、配准输入 | RGB-D 拼接 |
| **占据栅格 / OctoMap** | 占据体积（八叉树压缩） | **三态齐全** | 无 | 概率（clamping） | 中 | 高 | **导航/避障/规划** | OctoMap |
| **TSDF 体素** | **体积立方**（大场景需 hashing） | 隐含（截断带） | 高（零交叉提光滑网格） | **强（加权平均）** | 差（原版无）/ 需 hashing/Kintinuous | GPU 实时 | **高精度桌面/房间重建、AR** | KinectFusion, Voxblox |
| **Surfel 面元** | **表面积** | 无（仅面元） | 高（法线/半径/颜色） | 强（加权平均） | **好（非刚性形变）** | GPU 实时 | **房间级 RGB-D SLAM+重建** | ElasticFusion, Keller |
| **Mesh（MC/泊松）** | 表面积 | 无 | 高（连接面） | （取决于输入） | — | 离线/准实时 | 渲染/碰撞/物理/传输 | MC, Poisson |

**关键工程取舍（逐条，综合各源）**：
1. **稀疏 vs 稠密**：定位用稀疏（省、够）；避障/重建/交互用稠密（贵、必需）。**很多系统双地图并存**（稀疏定位 + 稠密建图）。
2. **度量 vs 拓扑**：局部精度用度量、大尺度全局用拓扑，**混合分层**最实用。
3. **OctoMap vs TSDF**：要"能不能走"（规划）选 OctoMap（三态 + 压缩）；要"长什么样"（重建/渲染）选 TSDF/surfel。
4. **TSDF vs surfel**：固定小体积、要最高几何精度选 TSDF；大房间、要回环、内存敏感选 surfel。
5. **TSDF 内存爆炸**：$O(\text{体积})$ → 大场景必须 **voxel hashing**（只存表面附近体素）或**移动体积**（Kintinuous）。
6. **置信封顶（clamping / $W_\eta$）**：静态环境不封（精度高），动态环境必封（适应快）——OctoMap 的 $l_{\min}/l_{\max}$、TSDF 的 $W_\eta$ 同理。
7. **单目 vs RGB-D 稠密**：单目省硬件但需平移基线 + 概率滤波（慢、无纹理处空洞）；RGB-D 直接给深度（快、稠密）但受深度量程/材质限制（玻璃/黑物/室外阳光失效）。
8. **网格化时机**：TSDF→MC 即时可得；点云→泊松需可靠法线且偏离线。

---

## 11 本抽取的 OCR / 公式核对修正说明（务必读）

> 本抽取的所有论文公式均经 `pdftotext -layout` 本地抽取后**逐字核对**。PDF 抽取过程中数学符号常被破坏，下面列出**发现并修正**的问题，以及**源材料本身的小瑕疵**：

1. **KinectFusion 式(S5-6) 缺右括号**：`pdftotext` 抽出为 `FRk (p) = Ψ λ −1 k(tg,k − pk2 − Rk (x)`——`k...k2` 是 PDF 把 $\|\cdot\|_2$ 的双竖线渲染成字母 `k`，且范数右竖线丢失。**已修正**为 $F_{R_k}(\mathbf p)=\Psi\bigl(\lambda^{-1}\|\mathbf t_{g,k}-\mathbf p\|_2-R_k(\mathbf x)\bigr)$（式S5-6）。同理式(S5-7) `kK−1 ẋk2`→$\|\mathbf K^{-1}\dot{\mathbf x}\|_2$。

2. **KinectFusion 式(S5-10) 抽出残缺**：`min ∑ kWR FR − F)k2` 中 $\sum_k\|W_{R_k}(F_{R_k}-F)\|^2$ 的下标 $k$、范数竖线被破坏，左括号丢失。**已据上下文修正**为 $\min_{F}\sum_k\|W_{R_k}(F_{R_k}-F)\|^2$。

3. **REMODE 式(S10-13) 焦距与单位向量同名 $f$**：原文 $f=\frac{^r\mathbf p}{\|^r\mathbf p\|}$（单位光线，式13附近）与式(S10-16) 的 "camera focal length $f$" **同字母不同义**——这是**源材料本身的记号瑕疵**。本抽取**把焦距改记 $f_{\text{focal}}$** 以消歧，并在文中显式说明。综合 agent 务必沿用区分。

4. **REMODE 图2注 $\tau_k^2$ 平方位置**：`pdftotext` 抽出为 `τk2 = ||r p+ || − ||r p||` 后面单独一个 `2`（平方跑到下一行）。**已修正**为 $\tau_k^2=\bigl(\|^r\mathbf p^+\|-\|^r\mathbf p\|\bigr)^2$（式S10-19），与正文式19 一致。

5. **OctoMap 式(S1-2) 的"先验项"**：WebFetch 的小模型一度把式(2) 误写成 $L(n\mid z_{1:t})=L(n\mid z_{1:t-1})+L(n\mid z_t)-L(n)$（多减一个 $L(n)$）。**经 `pdftotext` 核对原文 S1 式(2) 为两项相加** $L(n\mid z_{1:t})=L(n\mid z_{1:t-1})+L(n\mid z_t)$（因 OctoMap 取均匀先验 $P(n)=0.5\Rightarrow L(n)=0$，先验项已并入）。本抽取以**原文两项式**为准；若先验非 0.5，一般占据栅格公式确有 $-L(n)$ 项，已在 §5.3 注明"均匀先验下 $L(n)=0$"。

6. **OctoMap 默认参数两套数值冲突**：WebSearch 摘要混报了 $p_{\text{miss}}$=0.3 vs 0.4、clamp=0.1/0.95 vs 0.12/0.97。**经核对**：S1 论文实验值 = $p_{\text{hit}}0.7/p_{\text{miss}}0.4$、clamp 概率 0.12/0.97（$l_{\text{occ}}0.85/l_{\text{free}}{-}0.4$）；S2 库默认 = `setProbHit(0.7)/setProbMiss(0.3)`、`setClampingThresMin(0.1)/Max(0.95)`。**两套都正确但出处不同**，已在 §5.5 表中分列并提醒勿混。

7. **OctoMap 式(S1-1) 分式排版**：`pdftotext` 把式(1) 的连分式拆成多行错位。**已据原文重排**为 $P=\bigl[1+\frac{1-P(n|z_t)}{P(n|z_t)}\frac{1-P(n|z_{1:t-1})}{P(n|z_{1:t-1})}\frac{P(n)}{1-P(n)}\bigr]^{-1}$（式S1-1）。

8. **ElasticFusion 式(S7-3) 抽取错位**：双边/投影项 `I π(K exp(ξ̂)Tp(u, Dt )), Cˆt−1` 的括号与上下标被打散。**已据论文结构重建**为式(S7-3) 完整形式，并标注 $\boldsymbol\xi$ 旋转在前的排序差异。

9. **Surfel 半径/权重公式标 `\rebuilt`**：S7 把 surfel 融合细节甩给 S8（Keller），**S7 正文未给半径/权重闭式**；本抽取 §7.2 的 $r$、$w$、融合式来自 S8/InfiniTAM 标准形，**已显式打 `\rebuilt` 待综合 agent 对照 S8 原文核验**（尤其半径系数 $\sqrt2$ 在分子还是分母、$w$ 的 $\sigma$ 取值，不同实现有别）。

10. **深度滤波器矩匹配闭式更新缺失**：S9/S10 正文均把逐项闭式更新公式放在 supplementary（本套抽取无该补充材料）；§3.6.2 给出的内/外点责任加权更新式来自 SVO 实现，**已打 `\rebuilt` 待核**。

---

## 12 给综合 agent 的写章建议（结构 + 缺口 + 去重）

**建议章节顺序（循序渐进）**：
1. **动机**：稀疏地图为何不够 → 稠密建图的任务（§2.1）。
2. **地图分类**：度量/拓扑/语义、稀疏/稠密（§2.2）——作为全章导航。
3. **单目稠密**：极线搜索 → 块匹配（NCC）→ 三角化 → **几何不确定度推导（§3.5 式13-19，本章最硬核单目推导）** → 深度滤波（先讲十四讲高斯版 §3.6.1，再给 S9/S10 严格 Gaussian×Beta 版 §3.6.2 作进阶）→ 工程陷阱（逆深度/无纹理/仿射 warp）。
4. **RGB-D 稠密**：点云 → **OctoMap（log-odds + clamping 全套 §5，硬核）** → **TSDF/KinectFusion（式5-15 全套 §6，硬核）** → surfel/ElasticFusion（§7）。
5. **网格重建简介**：MC（§8）+ 泊松（§9）。
6. **工程权衡总表**（§10）。

**本章能自给自足的硬核推导**（无需外补）：单目深度不确定度（§3.5）、高斯深度融合（§3.6.1）、OctoMap 全套（§5）、TSDF 全套（§6）、NCC、MC 边插值。

**须打 `\rebuilt` / 另核的缺口**：① 深度滤波器矩匹配闭式更新（§3.6.2，对照 SVO 源码）；② surfel 半径/权重/融合系数（§7.2，对照 Keller S8 原文）；③ Curless-Levoy 原式记号（§6.4 已给同构形）。

**与本套其它抽取的去重**：
- **ICP / point-to-plane / KD-tree / NDT / 点云滤波**：本件**不重复**，见 `point_cloud_processing__pcl_icp.md`、`point_cloud_processing__hb_08.md`、`lidar_slam__*.md`。本件只在 §6.5 提 KinectFusion 用 point-to-plane ICP 做跟踪（一句带过，细节指向点云处理章）。
- **三角测量线性解**：见 `visual_odometry__*.md`，本件 §3.4 只给对应方程、不重复线性最小二乘解。
- **李代数指数映射 / 右扰动**：见 `lie_theory__*.md`，本件 §7.4 ElasticFusion 位姿优化按本书右扰动主线复述即可。

---

*（抽取留痕结束。本件已逐字核对 S1/S5/S6/S7/S9/S10 论文 PDF 公式、S2/S3/S4 官方源码与教程默认值；S8/S11/S12/S13/S14 据论文/教材主干 + 权威综述抽取，凡正文未给闭式者已打 `\rebuilt` 标注待核。）*
