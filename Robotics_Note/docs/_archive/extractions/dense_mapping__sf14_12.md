# 抽取留痕：《视觉SLAM十四讲》第12讲 建图（稠密建图）

> 本文件是项目内部「抽取留痕」，目标是把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章。**禁摘要、禁凝练**：每一步推导（中间代数不跳）、每一道例题/数值例、每一条定义/原理、每一段代码/伪码、每一张表、每一个工程权衡均完整记录。公式用 LaTeX 写全并标源小节号。宁长勿略。
>
> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/12_建图.md`（共 **1273 行**，已完整读取）
> **源章节**：《视觉SLAM十四讲》（第二版，高翔等）第 12 讲「建图」
> **⚠ 源为图像 OCR 所得，必有识别错**。本抽取已用领域知识重建正确公式/文字并交叉核对，OCR 修正逐处在文末「OCR 修正说明」及行内 `[OCR修正]` 标注。

---

## 0. 本章覆盖与范围说明（给综合 agent 的提示）

本第 12 讲是《十四讲》的最后实质性技术章，主题为**建图**，重点是**稠密建图**。完整覆盖了【本章聚焦】所要求的全部主题：

1. **地图用途与分类**（§12.1）：定位/导航/避障/重建/交互五大用途；度量 vs 拓扑、稀疏 vs 稠密的区分。
2. **单目稠密重建**（§12.2、§12.3）：立体视觉概述；极线搜索 + 块匹配（SAD/SSD/NCC）；高斯分布深度滤波器的**完整推导**（高斯乘积融合 + 几何不确定性的正弦定理推导）；完整 C++ 实践代码；实验分析（像素梯度问题、逆深度、图像间仿射变换、并行化、外点处理）。
3. **RGB-D 稠密建图**（§12.4）：点云地图（生成 + 统计滤波 + 体素降采样，完整代码）；从点云重建网格（MLS + 贪婪投影三角化，完整代码）；八叉树地图 OctoMap 的 **log-odds（概率对数）完整推导**（logit 变换 + 递推更新）+ 完整代码。
4. **TSDF 地图与 Fusion 系列**（§12.5）：实时三维重建；TSDF（截断符号距离函数）原理；KinectFusion 及 Dynamic/Elastic/Fusion4D/VolumeDeform 系列；TSDF 的"定位"（GPU-ICP）与"建图"（并行更新）。
5. **小结 + 习题**（§12.6、习题）。

**与本书其他章的关系**：
- 极线/对极几何回指 §7.3（第 7 讲对极几何）；三角化回指 §7.5；块匹配与直接法的相似性回指第 8 讲；点云内外参拼接回指 §5.4.2；RGB-D 深度测量原理回指第 11 讲（原文写"第11讲中详细讨论的深度估计"，疑指 RGB-D 相机模型，[OCR?]——《十四讲》二版 RGB-D 相机模型实际在第 5 讲，此处编号可能为原书排版/OCR 问题，见 OCR 说明）；高斯乘积融合回指附录 A。
- **surfel/网格/泊松重建仅作简介**，TSDF/KinectFusion 涉及 GPU 编程**只讲原理不演示**。

---

## 记号约定（本源 vs 本书统一约定）

| 项目 | 本源（《十四讲》第 12 讲）记号 | 含义 | 本书统一约定 | 差异/转换提示 |
|---|---|---|---|---|
| 旋转矩阵 | $\boldsymbol R$ | 旋转 $R\in SO(3)$ | $R\in SO(3)$ | 一致（用 R，不用 C） |
| 位姿/变换 | $\boldsymbol T$（$T\in SE(3)$）；代码 `SE3d`、`Isometry3d` | 相对位姿；如 $T_{C,R}$=参考帧到当前帧、$T_{WC}$=相机到世界 | $T\in SE(3)$ | 一致；本源**未涉及李代数扰动/雅可比**，故无右/左扰动差异问题（本章是滤波而非优化） |
| 平移 | $\boldsymbol t$ | 平移向量 | $t$ | 一致；几何推导中 $\boldsymbol t = O_1O_2$ 为相机平移 |
| 内参 | $\boldsymbol K$；分量 $f_x,f_y,c_x,c_y$ | 内参矩阵 | 同 | 一致；代码内参 `fx=481.2, fy=-480.0, cx=319.5, cy=239.5`（注意 $f_y$ 为**负**，见下方备注） |
| 像素坐标 | $p_1,p_2$；$u,v$；$P_R,P_C$ | 像素（齐次）坐标 | 同 | 一致 |
| 深度 | $d$；代码 `depth`/`depth_mu` | 像素深度，**本章特指 $O_1P$ 的长度（射线距离 range），非针孔模型的 $z$ 值** | $d$ | ⚠ §12.2.3 末明确：此"深度"= $O_1P$ 长度，与针孔相机的"深度=像素 z 值"**有差异**，综合时须注明 |
| 深度分布 | $P(d)=N(\mu,\sigma^2)$ | 深度服从高斯 | 同 | 一致 |
| 观测/融合 | $\mu_{\rm obs},\sigma_{\rm obs}^2$；$\mu_{\rm fuse},\sigma_{\rm fuse}^2$ | 观测分布、融合后分布 | 用 $\Sigma_w/\Sigma_v$ 区分过程/观测噪声 | 本源用 $\sigma_{\rm obs}$ 表观测不确定性（对应观测噪声，本书 $\Sigma_v$ 类）；本章**无运动方程**故无 $\Sigma_w$（见 §12.2.3 说明） |
| 三角形几何量 | $\boldsymbol p=O_1P$，$\boldsymbol a=O_2P$，$\boldsymbol t=O_1O_2$；角 $\alpha,\beta,\gamma$；扰动后 $\beta',\boldsymbol p',\boldsymbol p_2'$ | 不确定性分析三角形 | — | 详见 §12.2.3 推导 |
| 块匹配相似度 | $S(A,B)_{\rm SAD/SSD/NCC}$ | 图像块 $A,B\in\mathbb R^{w\times w}$ 的相似度 | — | NCC 接近 1 相似、接近 0 不相似；SAD/SSD 接近 0 相似 |
| 占据概率 | $x\in[0,1]$，对数几率 $y\in\mathbb R$，$L(n\mid z)$ | 八叉树节点占据概率与 log-odds | — | logit 变换，见 §12.4.3 |
| TSDF | （截断符号距离值，未给统一字母，图中 $d_x,d_y,d_z$=空间尺寸，$V_x,V_y,V_z$=体素数） | 截断符号距离函数 | — | 表面前为正、表面后为负、截断到 $[-1,1]$ |
| 四元数 | 代码 `Eigen::Quaterniond q(data[6],data[3],data[4],data[5])` | 读位姿 | **Hamilton** | ⚠ Eigen 构造 `Quaterniond(w,x,y,z)`，数据文件存储顺序为 `[tx ty tz qx qy qz qw]`（位置在前、四元数 xyz 在前 w 在后）；构造时把 `data[6]=qw` 放第一参数，符合 Hamilton 约定（实部在前），**与本书 Hamilton 约定一致** |

**关键记号备注**：
- **$f_y$ 为负值**（`fy=-480.0`）：这是 REMODE/合成数据集的相机内参约定（图像 y 轴方向与相机坐标系 y 轴反向），并非笔误；点云生成式 `point[1]=(v-cy)*point[2]/fy` 中负 $f_y$ 用于把像素 v 轴翻转到世界坐标。同样出现在点云代码 `fy=-480.0`。
- 本源**无定理/引理/命题编号体系**（教材式叙述）。公式编号为 (12.1)~(12.19)，本文逐一保留并核对。
- 术语对照：MVS = Moving View Stereo（移动视角立体视觉）；SAD = Sum of Absolute Difference；SSD = Sum of Squared Distance（注意原文戏言"不是固态硬盘"）；NCC = Normalized Cross Correlation（归一化互相关）；Log-odds = 概率对数值（对数几率）；TSDF = Truncated Signed Distance Function（截断符号距离函数）；Surfel = surface element（面元/面片）；MLS = Moving Least Squares（移动最小二乘）；GP3 = Greedy Projection Triangulation（贪婪投影三角化）。

---

## 第12讲 建图 — 主要目标（源 开篇）

**主要目标**（原文逐条）：
1. 理解单目 SLAM 中稠密深度估计的原理。
2. 通过实验了解单目稠密重建的过程。
3. 了解几种 RGB-D 重建中的地图形式。

**引言**（原文要点）：前端和后端中重点关注同时估计相机运动轨迹与特征点空间位置。但实际使用 SLAM 时，除对相机本体定位，还存在许多其他需求——放在机器人上的 SLAM 会希望地图能用于**定位、导航、避障和交互**，特征点地图显然不能满足所有需求。本讲更详细讨论各种形式的地图，并指出目前视觉 SLAM 地图中存在的缺陷。

---

## 12.1 概述

### 12.1.1 建图为何单独讨论

建图本应是 SLAM 的两大目标之一（SLAM = 同时定位与建图）。但此前讨论的都是定位问题（特征点定位、直接法定位、后端优化）。这是否暗示建图不重要？**答案是否定的**。

经典 SLAM 模型中，所谓地图即**所有路标点的集合**。一旦确定路标点位置即完成建图。视觉里程计、BA 实际都建模了路标点位置并对其优化——从这个角度，建图问题已被探讨。那为何还要单独介绍？

**因为人们对建图的需求不同**。SLAM 作为底层技术，为上层应用提供信息：
- 上层是机器人：可能希望用 SLAM 做全局定位，并让机器人在地图中导航——例如扫地机需要计算一条能覆盖整张地图的路径。
- 上层是增强现实设备：可能希望将虚拟物体叠加在现实物体中，特别地还需处理虚拟物体与真实物体的**遮挡关系**。

应用层面对"定位"的需求是相似的（希望 SLAM 提供相机/主体的空间位姿）；而对"地图"则存在许多不同需求。从视觉 SLAM 角度，"建图"服务于"定位"；但从应用层面，"建图"明显带有许多其他需求。

### 12.1.2 地图的五大用途（原文逐条）

1. **定位**。定位是地图的一项基本功能。视觉里程计部分讨论了如何利用局部地图实现定位；回环检测部分也看到，只要有全局描述子信息，也能通过回环检测确定机器人位置。还希望能把地图**保存**下来，让机器人下次开机后依然能在地图中定位，这样只需对地图建模一次，而不必每次启动都重做完整 SLAM。

2. **导航**。机器人在地图中进行**路径规划**，在任意两个地图点间寻找路径，然后控制自己运动到目标点。至少需要知道地图中哪些地方不可通过、哪些可以通过。这超出了稀疏特征点地图的能力范围，必须有另外的地图形式——**至少得是一种稠密的地图**。

3. **避障**。与导航类似但更注重**局部的、动态的**障碍物处理。仅有特征点无法判断某特征点是否为障碍物，所以需要稠密地图。

4. **重建**。利用 SLAM 获得周围环境的重建效果，主要用于**向人展示**，希望看上去舒服、美观；也可用于**通信**（三维视频通话、网上购物等）。这种地图亦是稠密的，且对外观有要求——可能不满足于稠密点云，更希望构建**带纹理的平面**（像电子游戏中的三维场景）。

5. **交互**。指人与地图之间的互动。例如增强现实中放置虚拟物体并与之互动（点击墙面虚拟浏览器看视频、向墙面投掷物体希望有虚拟物理碰撞）。机器人应用中也有与人、与地图的交互——例如收到命令"取桌子上的报纸"，机器人除了有环境地图，还需知道哪块地图是"桌子"、什么叫"之上"、什么叫"报纸"。这需要对地图有更高层面的认知——也称为**语义地图**。

### 12.1.3 稀疏 vs 稠密地图（地图分类核心段，§12.1 配图 12-1）

> 图 12-1：各种地图的示意图（例子分别来自参考文献 [88, 119, 120]）——形象解释了各种地图类型与用途之间的关系。

- **此前讨论基本集中于"稀疏路标地图"**，还没探讨稠密地图。
- **稀疏地图**：只建模感兴趣的部分，即特征点（路标点）。对同一张桌子，稀疏地图可能只建模桌子的四个角。
- **稠密地图**：建模**所有看到过的部分**。稠密地图会建模整个桌面。
- **关键区别**：虽然从定位角度只有四个角的地图也可用于对相机定位，但**无法从四个角推断这几个点之间的空间结构**，所以无法仅用四个角完成导航、避障等需要稠密地图才能完成的工作。

> **分类小结**（综合 agent 用）：本书地图分类维度——
> (a) 按用途：定位 / 导航 / 避障 / 重建 / 交互；
> (b) 按稀疏 vs 稠密：稀疏（路标/特征点）vs 稠密（全部观测）；
> (c) 度量地图 vs 拓扑地图（§12.6 小结点明：本讲偏重**度量地图**，拓扑地图因与 SLAM 研究差别大未展开）；
> (d) 语义地图（§12.1 用途 5 引出，带高层认知）。

**核心结论**：稠密地图占据非常重要的位置。剩下的问题是：通过视觉 SLAM 能建立稠密地图吗？如果能，怎么建？

---

## 12.2 单目稠密重建

### 12.2.1 立体视觉

相机被认为是**只有角度的传感器（Bearing only）**。单幅图像中的像素只能提供物体与相机成像平面的**角度**及采集到的**亮度**，而无法提供物体的**距离（Range）**。稠密重建中需要知道每一个（或大部分）像素点的距离，大致有如下解决方案：

1. 使用**单目**相机，估计相机运动，并三角化计算像素距离。
2. 使用**双目**相机，利用左右目的**视差**计算像素距离（多目原理相同）。
3. 使用 **RGB-D** 相机直接获得像素距离。

- 前两种方式称为**立体视觉（Stereo Vision）**，其中移动单目相机的又称为**移动视角的立体视觉（Moving View Stereo，MVS）**。
- 相比 RGB-D 直接测量深度，单目和双目对深度的获取往往"**费力不讨好**"——计算量巨大，最后得到一些不怎么可靠的深度估计（原文脚注①标注其不可靠性）。
- RGB-D 也有量程、应用范围和光照的限制，但相比单目/双目，RGB-D 稠密重建往往是更常见的选择。
- 单目/双目的好处：在目前 RGB-D 还无法被很好应用的**室外、大场景**场合中，仍能通过立体视觉估计深度信息。

**本节任务设定**（简化建图问题，先不考虑 SLAM）：假定有一段视频序列，通过某种"魔法"得到了每一帧对应的轨迹（也很可能由视觉里程计前端估计所得）。**以第一幅图像为参考帧，计算参考帧中每个像素的深度（或距离）**。

**回忆特征点法如何完成此过程**：
1. 对图像提取特征，根据描述子计算特征间匹配。即通过特征对某空间点进行了跟踪，知道了它在各图像间的位置。
2. 由于无法仅用一幅图像确定特征点位置，必须通过不同视角下的观测估计其深度，原理即**三角测量**。

**稠密深度图估计的不同之处**：无法把每个像素都当作特征点计算描述子。因此**匹配**成为很重要的一环：如何确定第一幅图的某像素出现在其他图里的位置？——这需要用到**极线搜索和块匹配**技术（参考文献 [121]）。知道某像素在各图中的位置后，就能像特征点那样用三角测量法确定深度。**不同的是**：这里要使用**很多次**三角测量法让深度估计收敛，而不仅一次。希望深度估计随测量增加从一个非常不确定的量逐渐收敛到稳定值——这就是**深度滤波器**技术。

### 12.2.2 极线搜索与块匹配

#### 极线几何（图 12-2）

> 图 12-2：极线搜索示意图。

探讨不同视角下观察同一点产生的几何关系，非常像 §7.3 讨论的对极几何。如图 12-2：
- 左边相机观测到某个像素 $p_1$。由于是单目相机，无从知道其深度，假设深度在某区域内，不妨说从某最小值到无穷远之间 $(d_{\min}, +\infty)$。
- 因此该像素对应的空间点分布在某条**线段**（本例中是**射线**）上。
- 从另一视角（右侧相机）看，这条线段的投影形成图像平面上的一条线——这称为**极线**。
- 知道两部相机间运动时，这条极线也能确定（原文脚注①）。
- 问题：**极线上的哪一个点是刚才看到的 $p_1$ 点？**

**与特征点法对比**：特征点法通过特征匹配找到 $p_2$ 位置。现在没有描述子，只能在极线上搜索和 $p_1$ 长得比较相似的点——沿第二幅图像中极线的一头走到另一头，逐个比较每个像素与 $p_1$ 的相似程度。从直接比较像素角度看，这与**直接法**有异曲同工之妙。

#### 从单像素到块匹配

- 单个像素的亮度值并不稳定可靠（直接法已讨论）。若极线上有很多和 $p_1$ 相似的点，难以确定哪个是真实的。这回到回环检测中的问题：如何确定两幅图像（或两点）的相似性？回环检测用词袋解决，但这里没有特征，只好另寻途径。
- **块匹配思路**：单个像素亮度没有区分性，改为比较**像素块**。在 $p_1$ 周围取一个大小为 $w\times w$ 的小块，在极线上也取很多同样大小的小块进行比较，可在一定程度上提高区分性。
- **假设的强化**：只有假设不同图像间**整个小块的灰度值不变**，这种比较才有意义。所以算法假设从**像素的灰度不变性**变成了**图像块的灰度不变性**——在一定程度上变得更强。

记 $p_1$ 周围的小块为 $A\in\mathbb R^{w\times w}$，极线上的 $n$ 个小块为 $B_i,\ i=1,\cdots,n$。计算小块间差异有若干方法：

#### 三种块匹配度量

**1. SAD（Sum of Absolute Difference，绝对差之和）**（源 式12.1）：

$$
S(\boldsymbol A,\boldsymbol B)_{\mathrm{SAD}} = \sum_{i,j}\bigl|\boldsymbol A(i,j) - \boldsymbol B(i,j)\bigr|. \tag{12.1}
$$

**2. SSD（Sum of Squared Distance，平方和）**（源 式12.2，原文戏言"并不是固态硬盘"）：

$$
S(\boldsymbol A,\boldsymbol B)_{\mathrm{SSD}} = \sum_{i,j}\bigl(\boldsymbol A(i,j) - \boldsymbol B(i,j)\bigr)^2. \tag{12.2}
$$

**3. NCC（Normalized Cross Correlation，归一化互相关）**（源 式12.3）：比前两者复杂，计算两个小块的相关性：

$$
S(\boldsymbol A,\boldsymbol B)_{\mathrm{NCC}} = \frac{\displaystyle\sum_{i,j}\boldsymbol A(i,j)\,\boldsymbol B(i,j)}{\sqrt{\displaystyle\sum_{i,j}\boldsymbol A(i,j)^2 \;\sum_{i,j}\boldsymbol B(i,j)^2}}. \tag{12.3}
$$

**相似度方向（重要约定）**：
- NCC 用的是相关性，**相关性接近 0 表示不相似，接近 1 表示相似**。
- SAD/SSD 是反过来的，**接近 0 表示相似，大数值表示不相似**。

**精度-效率矛盾**：精度好的方法往往需要复杂计算，简单快速的算法效果不佳，需在实际工程中取舍。

**去均值版本**：可先把每个小块的均值去掉，称为**去均值的 SSD、去均值的 NCC** 等。去掉均值后，允许像"小块 B 比 A 整体上亮一些，但仍很相似"这样的情况（原文脚注①），因此比之前更可靠。更多块匹配度量见参考文献 [122, 123]。

#### 沿极线的相似度分布（图 12-3）

> 图 12-3：匹配得分沿距离的分布（图像来自参考文献 [124]）。

- 在极线上计算 $A$ 与每个 $B_i$ 的相似度（假设用 NCC），得到一个**沿极线的 NCC 分布**。分布形状取决于图像数据。
- 在搜索距离较长的情况下，通常得到一个**非凸函数**：存在许多峰值，然而真实对应点必定只有一个。
- 这种情况下倾向于用**概率分布**描述深度值，而非单一数值。
- 问题转到：在不断对不同图像进行极线搜索时，估计的深度分布将如何变化——这就是**深度滤波器**。

### 12.2.3 高斯分布的深度滤波器 ★（核心推导）

#### 建模思路：滤波 vs 优化

对像素点深度的估计本身可建模为**状态估计问题**，存在**滤波器**与**非线性优化**两种求解思路。虽然非线性优化效果较好，但 SLAM 实时性要求强、前端已占不少计算量，建图通常采用**计算量较少的滤波器**方式。

对深度分布的假设有若干做法：
- **简单假设**：深度值服从高斯分布，得到一种**类卡尔曼式**的方法（但实际上只是**归一化积**，稍后可见）。
- **复杂假设**：参考文献 [68, 124] 采用**均匀-高斯混合分布**的假设，推导出形式更复杂的深度滤波器。

本节先介绍并演示**高斯分布**假设下的深度滤波器，把均匀-高斯混合分布滤波器作为习题。

#### 高斯乘积融合推导

设某像素点深度 $d$ 服从（源 式12.4）：

$$
P(d) = N(\mu, \sigma^2). \tag{12.4}
$$

每当新数据到来观测到它的深度，假设该观测也是高斯分布（源 式12.5）：

$$
P(d_{\mathrm{obs}}) = N(\mu_{\mathrm{obs}}, \sigma_{\mathrm{obs}}^2). \tag{12.5}
$$

**问题**：如何用观测信息更新原先 $d$ 的分布——这是一个**信息融合**问题。根据附录 A，**两个高斯分布的乘积依然是高斯分布**。设融合后 $d$ 的分布为 $N(\mu_{\mathrm{fuse}}, \sigma_{\mathrm{fuse}}^2)$，那么根据高斯分布的乘积有（源 式12.6）：

$$
\mu_{\mathrm{fuse}} = \frac{\sigma_{\mathrm{obs}}^2\,\mu + \sigma^2\,\mu_{\mathrm{obs}}}{\sigma^2 + \sigma_{\mathrm{obs}}^2}, \qquad
\sigma_{\mathrm{fuse}}^2 = \frac{\sigma^2\,\sigma_{\mathrm{obs}}^2}{\sigma^2 + \sigma_{\mathrm{obs}}^2}. \tag{12.6}
$$

> **抽取补全推导（式12.6 的来历，对应习题 1「推导式(12.6)」）**：
> 两个一维高斯密度
> $$N(x;\mu,\sigma^2)\propto \exp\!\Big(-\tfrac{(x-\mu)^2}{2\sigma^2}\Big),\quad N(x;\mu_{\rm obs},\sigma_{\rm obs}^2)\propto \exp\!\Big(-\tfrac{(x-\mu_{\rm obs})^2}{2\sigma_{\rm obs}^2}\Big)$$
> 相乘，指数相加：
> $$-\frac{(x-\mu)^2}{2\sigma^2}-\frac{(x-\mu_{\rm obs})^2}{2\sigma_{\rm obs}^2}.$$
> 对 $x$ 配方，二次项系数给出融合方差倒数（信息相加）：
> $$\frac{1}{\sigma_{\rm fuse}^2}=\frac{1}{\sigma^2}+\frac{1}{\sigma_{\rm obs}^2}\;\Rightarrow\;\sigma_{\rm fuse}^2=\frac{\sigma^2\sigma_{\rm obs}^2}{\sigma^2+\sigma_{\rm obs}^2},$$
> 一次项给出融合均值（信息加权）：
> $$\mu_{\rm fuse}=\sigma_{\rm fuse}^2\Big(\frac{\mu}{\sigma^2}+\frac{\mu_{\rm obs}}{\sigma_{\rm obs}^2}\Big)=\frac{\sigma_{\rm obs}^2\mu+\sigma^2\mu_{\rm obs}}{\sigma^2+\sigma_{\rm obs}^2}.$$
> 与式(12.6)一致。此即"两个高斯归一化积"的标准结论。

**与卡尔曼的关系**：由于仅有观测方程没有运动方程，深度仅用到**信息融合**部分，无须像完整卡尔曼那样进行预测和更新（也可看成"运动方程为深度值固定不动"的卡尔曼滤波器）。

> **记号对齐提示**：本源**无运动方程**（$\Sigma_w$ 无对应）；$\sigma_{\rm obs}^2$ 是观测不确定性（对应本书观测噪声 $\Sigma_v$ 类）。这与本书"完整卡尔曼/EKF（含 $\Sigma_w,\Sigma_v$）"的差异在于此处退化为纯量测更新。

**剩余问题**：如何确定观测深度的分布，即如何计算 $\mu_{\rm obs}, \sigma_{\rm obs}$？

#### 几何不确定性的正弦定理推导（图 12-4）★

> 图 12-4：不确定性分析。

**处理方式说明**：参考文献 [75] 考虑了**几何不确定性 + 光度不确定性二者之和**，参考文献 [124] 仅考虑**几何不确定性**。本节**只考虑几何不确定性**。

**问题**：已通过极线搜索和块匹配确定参考帧某像素在当前帧的投影位置。这个位置对深度的不确定性有多大？

**几何量定义**（图 12-4）：极线搜索找到 $p_1$ 对应的 $p_2$ 点，观测到 $p_1$ 的深度值，认为 $p_1$ 对应三维点为 $P$。记：
- $\boldsymbol p = O_1P$（参考相机光心到 $P$，即待求深度方向向量）；
- $\boldsymbol t = O_1O_2$（相机的平移）；
- $\boldsymbol a = O_2P$（当前相机光心到 $P$）；
- 三角形下面两个角记作 $\alpha$（在 $O_1$ 处）、$\beta$（在 $O_2$ 处）。

**扰动**：考虑极线 $l_2$ 上存在**一个像素大小的误差**，使 $\beta$ 角变成 $\beta'$，$p_2$ 变成 $p_2'$，记上面那个角（$P$ 处）为 $\gamma$。**问**：这个像素的误差会导致 $p'$ 与 $p$ 产生多大差距？

**几何关系**（源 式12.7）：

$$
\boldsymbol a = \boldsymbol p - \boldsymbol t,
$$
$$
\alpha = \arccos\langle \boldsymbol p, \boldsymbol t\rangle, \tag{12.7}
$$
$$
\beta = \arccos\langle \boldsymbol a, -\boldsymbol t\rangle.
$$

> 注：$\langle\cdot,\cdot\rangle$ 为两向量夹角的归一化内积（即 $\arccos$ 作用于单位向量点积，得夹角）。$\alpha$ 是 $\boldsymbol p$ 与 $\boldsymbol t$ 的夹角；$\beta$ 是 $\boldsymbol a$ 与 $-\boldsymbol t$ 的夹角。

**对 $p_2$ 扰动一个像素**，使 $\beta$ 产生变化成为 $\beta'$（源 式12.8）：

$$
\beta' = \arccos\bigl\langle \overrightarrow{O_2 p_2'},\; -\boldsymbol t\bigr\rangle, \tag{12.8}
$$
$$
\gamma = \pi - \alpha - \beta'.
$$

> 注：$\gamma=\pi-\alpha-\beta'$ 来自三角形内角和为 $\pi$（扰动后三角形 $O_1O_2P'$ 的三个内角分别约为 $\alpha,\beta',\gamma$；这里近似认为 $O_1$ 处角度 $\alpha$ 不变，因为 $p_1$ 未扰动）。

**由正弦定理求 $p'$ 的大小**（源 式12.9）：

$$
\|\boldsymbol p'\| = \|\boldsymbol t\|\,\frac{\sin\beta'}{\sin\gamma}. \tag{12.9}
$$

> **抽取补全推导（式12.9）**：在三角形 $O_1O_2P'$ 中，边 $O_1P'=\|\boldsymbol p'\|$ 对的角是 $O_2$ 处的角 $\beta'$；边 $O_1O_2=\|\boldsymbol t\|$ 对的角是 $P'$ 处的角 $\gamma$。由正弦定理 $\dfrac{\|\boldsymbol p'\|}{\sin\beta'}=\dfrac{\|\boldsymbol t\|}{\sin\gamma}$，即得式(12.9)。

**由此确定单像素不确定性引起的深度不确定性**。若认为极线搜索的块匹配仅有一个像素误差，则设（源 式12.10）：

$$
\sigma_{\mathrm{obs}} = \|\boldsymbol p\| - \|\boldsymbol p'\|. \tag{12.10}
$$

> 注：实际取标准差为深度估计 $\|\boldsymbol p\|$（无扰动）与扰动后 $\|\boldsymbol p'\|$ 之差。代码中对应 `d_cov = p_prime - depth_estimation`（符号相反但平方后无影响），再平方得方差 `d_cov2 = d_cov*d_cov`。

**放大规则**：若极线搜索的不确定性大于一个像素，可按此推导**放大**这个不确定性。接下来的深度数据融合（式12.6）已介绍。实际工程中，当**不确定性小于一定阈值**时，认为深度数据已收敛。

#### 稠密深度估计完整流程（源 §12.2.3 末，算法伪码级）★

> **算法 12-A：高斯深度滤波器估计稠密深度（完整过程）**
> 1. 假设所有像素的深度满足某个**初始的高斯分布**。
> 2. 当新数据产生时，通过**极线搜索和块匹配**确定投影点位置。
> 3. 根据**几何关系**计算三角化后的深度及不确定性。
> 4. 将当前观测**融合**进上一次的估计中。若**收敛则停止**计算，否则返回第 2 步。

**重要澄清**：这里说的深度值是 $O_1P$ 的长度，它和针孔相机模型里的"深度"有少许不同——**针孔相机中的深度是指像素的 $z$ 值**。

---

## 12.3 实践：单目稠密重建

### 数据集与设定（源 §12.3 开头）

- 使用 **REMODE**（参考文献 [121, 125]）测试数据集：一架无人机采集的单目俯视图像，共 **200 张**，同时提供每张图像的**真实位姿**。
- 任务：估算**第一帧**图像每个像素对应的深度值（单目稠密重建）。
- 下载地址：`http://rpg.ifi.uzh.ch/datasets/remode_test_data.zip`。
- 解压后 `test_data/Images` 中有 0~200 的所有图像；`test_data` 目录下一个文本文件记录每幅图像对应的位姿，格式为 `文件名 tx ty tz qx qy qz qw`：

```text
scene_000.png 1.086410 4.766730 -1.449960 0.789455 0.051299 -0.000779 0.611661
scene_001.png 1.086390 4.766370 -1.449530 0.789180 0.051881 -0.001131 0.611966
scene_002.png 1.086120 4.765520 -1.449090 0.788982 0.052159 -0.000735 0.612198
.....
```

> 图 12-5：若干时刻的图像（t=0, 50, 100, 150, 200）。场景主要由**地面、桌子及桌子上的杂物**组成。若深度估计大致正确，至少可看出桌子与地面深度值不同。

程序为方便理解写成 **C 语言风格**放在单个文件中。

### 源文件：`slambook2/ch12/dense_monocular/dense_mapping.cpp`（片段）

#### (1) 程序头注释

```cpp
/******************************************************************************************
* 本程序演示了单目相机在已知轨迹下的稠密深度估计
* 使用极线搜索 + NCC 匹配的方式，与书本的 12.2 节对应
* 请注意本程序并不完美，你完全可以改进它——笔者其实在故意暴露一些问题
******************************************************************************************/
```

#### (2) 参数定义

```cpp
// ------------------------------------------------------------------
// parameters
const int boarder = 20;         // 边缘宽度
const int width = 640;          // 图像宽度
const int height = 480;         // 图像高度
const double fx = 481.2f;       // 相机内参
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const int ncc_window_size = 3;  // NCC 取的窗口半宽度
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1); // NCC窗口面积
const double min_cov = 0.1;     // 收敛判定：最小方差
const double max_cov = 10;      // 发散判定：最大方差
```

> 注：`boarder` 是源码原拼写（应为 border，[OCR/源码原样保留]）。NCC 窗口半宽 3 → 窗口为 $7\times7=49$ 像素（`ncc_area=49`）。

#### (3) 重要函数声明（含 doxygen 注释）

```cpp
// ------------------------------------------------------------------
// 重要的函数
/**
 * 根据新的图像更新深度估计
 * @param ref          参考图像
 * @param curr         当前图像
 * @param T_C_R        参考图像到当前图像的位姿
 * @param depth        深度
 * @param depth_cov    深度方差
 * @return             是否成功
 */
bool update(
    const Mat &ref, const Mat &curr, const SE3d &T_C_R,
    Mat &depth, Mat &depth_cov2);

/**
 * 极线搜索
 * @param ref                 参考图像
 * @param curr                当前图像
 * @param T_C_R               位姿
 * @param pt_ref              参考图像中点的位置
 * @param depth_mu            深度均值
 * @param depth_cov           深度方差
 * @param pt_curr             当前点
 * @param epipolar_direction  极线方向
 * @return                    是否成功
 */
bool epipolarSearch(
    const Mat &ref, const Mat &curr, const SE3d &T_C_R,
    const Vector2d &pt_ref, const double &depth_mu, const double &depth_cov,
    Vector2d &pt_curr, Vector2d &epipolar_direction);

/**
 * 更新深度滤波器
 * @param pt_ref              参考图像点
 * @param pt_curr             当前图像点
 * @param T_C_R               位姿
 * @param epipolar_direction  极线方向
 * @param depth               深度均值
 * @param depth_cov2          深度方差
 * @return                    是否成功
 */
bool updateDepthFilter(
    const Vector2d &pt_ref, const Vector2d &pt_curr, const SE3d &T_C_R,
    const Vector2d &epipolar_direction, Mat &depth, Mat &depth_cov2);

/**
 * 计算 NCC 评分
 * @param ref      参考图像
 * @param curr     当前图像
 * @param pt_ref   参考点
 * @param pt_curr  当前点
 * @return         NCC 评分
 */
double NCC(const Mat &ref, const Mat &curr, const Vector2d &pt_ref, const Vector2d &pt_curr);

// 双线性灰度插值
inline double getBilinearInterpolatedValue(const Mat &img, const Vector2d &pt) {
    uchar *d = &img.data[int(pt(1, 0)) * img.step + int(pt(0, 0))];
    double xx = pt(0, 0) - floor(pt(0, 0));
    double yy = pt(1, 0) - floor(pt(1, 0));
    return ((1 - xx) * (1 - yy) * double(d[0]) +
            xx * (1 - yy) * double(d[1]) +
            (1 - xx) * yy * double(d[img.step]) +
            xx * yy * double(d[img.step + 1])) / 255.0;
}
```

> 双线性插值说明：`d[0]` 为左上、`d[1]` 右上、`d[img.step]` 左下、`d[img.step+1]` 右下；`xx,yy` 为亚像素小数部分权重；除以 255.0 归一化到 [0,1]。

#### (4) main 函数

```cpp
int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: dense_mapping path_to_test_dataset" << endl;
        return -1;
    }

    // 从数据集读取数据
    vector<string> color_image_files;
    vector<SE3d> poses_TWC;
    Mat ref_depth;
    bool ret = readDatasetFiles(argv[1], color_image_files, poses_TWC, ref_depth);
    if (ret == false) {
        cout << "Reading image files failed!" << endl;
        return -1;
    }
    cout << "read total " << color_image_files.size() << " files." << endl;

    // 第一张图
    Mat ref = imread(color_image_files[0], 0);   // gray-scale image
    SE3d pose_ref_TWC = poses_TWC[0];
    double init_depth = 3.0;    // 深度初始值
    double init_cov2 = 3.0;     // 方差初始值
    Mat depth(height, width, CV_64F, init_depth);       // 深度图
    Mat depth_cov2(height, width, CV_64F, init_cov2);   // 深度图方差

    for (int index = 1; index < color_image_files.size(); index++) {
        cout << "*** loop " << index << " *** " << endl;
        Mat curr = imread(color_image_files[index], 0);
        if (curr.data == nullptr) continue;
        SE3d pose_curr_TWC = poses_TWC[index];
        SE3d pose_T_C_R = pose_curr_TWC.inverse() * pose_ref_TWC; // T_C_W * T_W_R = T_C_R
        update(ref, curr, pose_T_C_R, depth, depth_cov2);
        evaluateDepth(ref_depth, depth);
        plotDepth(ref_depth, depth);
        imshow("image", curr);
        waitKey(1);
    }

    cout << "estimation returns, saving depth map ..." << endl;
    imwrite("depth.png", depth);
    cout << "done." << endl;

    return 0;
}
```

> 关键行：`pose_T_C_R = pose_curr_TWC.inverse() * pose_ref_TWC;` 即 $T_{C,R} = T_{C,W}\,T_{W,R} = (T_{W,C})^{-1} T_{W,R}$（参考帧→当前帧的相对位姿）。初始深度与方差均取 **3.0**。

#### (5) update 函数（遍历像素）

```cpp
bool update(const Mat &ref, const Mat &curr, const SE3d &T_C_R, Mat &depth, Mat &depth_cov2)
{
    for (int x = boarder; x < width - boarder; x++)
        for (int y = boarder; y < height - boarder; y++) {
            // 遍历每个像素
            if (depth_cov2.ptr<double>(y)[x] < min_cov || depth_cov2.ptr<double>(y)[x] > max_cov)
                // 深度已收敛或发散
                continue;
            // 在极线上搜索 (x,y) 的匹配
            Vector2d pt_curr;
            Vector2d epipolar_direction;
            bool ret = epipolarSearch(
                ref, curr, T_C_R, Vector2d(x, y), depth.ptr<double>(y)[x],
                sqrt(depth_cov2.ptr<double>(y)[x]), pt_curr, epipolar_direction);
            if (ret == false) // 匹配失败
                continue;
            // 取消该注释以显示匹配
            // showEpipolarMatch(ref, curr, Vector2d(x, y), pt_curr);
            // 匹配成功，更新深度图
            updateDepthFilter(Vector2d(x, y), pt_curr, T_C_R, epipolar_direction, depth, depth_cov2);
        }
}
```

> 收敛/发散判据：`depth_cov2 < min_cov(0.1)` 视为收敛、`> max_cov(10)` 视为发散，均跳过。注意传给 `epipolarSearch` 的是**标准差** `sqrt(depth_cov2)`。

#### (6) epipolarSearch 函数（极线搜索，对应 §12.2、§12.3）

```cpp
// 极线搜索
// 方法见 12.2 节和 12.3 节
bool epipolarSearch(
    const Mat &ref, const Mat &curr,
    const SE3d &T_C_R, const Vector2d &pt_ref,
    const double &depth_mu, const double &depth_cov,
    Vector2d &pt_curr, Vector2d &epipolar_direction) {
    Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    Vector3d P_ref = f_ref * depth_mu;   // 参考帧的 P 向量

    Vector2d px_mean_curr = cam2px(T_C_R * P_ref);   // 按深度均值投影的像素
    double d_min = depth_mu - 3 * depth_cov, d_max = depth_mu + 3 * depth_cov;
    if (d_min < 0.1) d_min = 0.1;
    Vector2d px_min_curr = cam2px(T_C_R * (f_ref * d_min));  // 按最小深度投影的像素
    Vector2d px_max_curr = cam2px(T_C_R * (f_ref * d_max));  // 按最大深度投影的像素

    Vector2d epipolar_line = px_max_curr - px_min_curr;  // 极线（线段形式）
    epipolar_direction = epipolar_line;   // 极线方向
    epipolar_direction.normalize();
    double half_length = 0.5 * epipolar_line.norm();   // 极线线段的半长度
    if (half_length > 100) half_length = 100;   // 我们不希望搜索太多东西

    // 取消此句注释以显示极线（线段）
    // showEpipolarLine(ref, curr, pt_ref, px_min_curr, px_max_curr);

    // 在极线上搜索，以深度均值点为中心，左右各取半长度
    double best_ncc = -1.0;
    Vector2d best_px_curr;
    for (double l = -half_length; l <= half_length; l += 0.7) {  // l += sqrt(2)
        Vector2d px_curr = px_mean_curr + l * epipolar_direction;  // 待匹配点
        if (!inside(px_curr))
            continue;
        // 计算待匹配点与参考帧的 NCC
        double ncc = NCC(ref, curr, pt_ref, px_curr);
        if (ncc > best_ncc) {
            best_ncc = ncc;
            best_px_curr = px_curr;
        }
    }
    if (best_ncc < 0.85f)   // 只相信 NCC 很高的匹配
        return false;
    pt_curr = best_px_curr;
    return true;
}
```

> 关键细节：
> - 极线由 $\pm3\sigma$（`depth_mu ± 3*depth_cov`）的最小/最大深度投影确定，$d_{\min}$ 下限钳到 0.1。
> - 极线半长上限钳到 100 像素（"不希望搜索太多东西"）。
> - 搜索步长 `0.7`，注释为 `l += sqrt(2)`，实为 $\sqrt2/2\approx0.707$ 的近似（**源注释 `sqrt(2)` 略有误导**，§12.3 正文说"步长取 $\sqrt2/2$ 的近似值 0.7"，二者口径以正文为准，[OCR/源码注释不一致]）。
> - NCC 阈值 `0.85`，低于则匹配失败。

#### (7) NCC 函数（零均值归一化互相关，对应式12.11）

```cpp
double NCC(
    const Mat &ref, const Mat &curr,
    const Vector2d &pt_ref, const Vector2d &pt_curr) {
    // 零均值-归一化互相关
    // 先算均值
    double mean_ref = 0, mean_curr = 0;
    vector<double> values_ref, values_curr;   // 参考帧和当前帧的均值
    for (int x = -ncc_window_size; x <= ncc_window_size; x++)
        for (int y = -ncc_window_size; y <= ncc_window_size; y++) {
            double value_ref = double(ref.ptr<uchar>(int(y + pt_ref(1, 0)))[int(x + pt_ref(0, 0))]) / 255.0;
            mean_ref += value_ref;

            double value_curr = getBilinearInterpolatedValue(curr, pt_curr + Vector2d(x, y));
            mean_curr += value_curr;

            values_ref.push_back(value_ref);
            values_curr.push_back(value_curr);
        }

    mean_ref /= ncc_area;
    mean_curr /= ncc_area;

    // 计算 Zero mean NCC
    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < values_ref.size(); i++) {
        double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10);   // 防止分母出现零
}
```

> 注：`demoniator` 是源码原拼写（应为 denominator，源码原样保留）。末尾 `+1e-10` 防分母为零。参考帧用整数索引取值、当前帧用双线性插值取值（亚像素）。

#### (8) updateDepthFilter 函数（三角化 + 不确定性 + 高斯融合）★

```cpp
bool updateDepthFilter(
    const Vector2d &pt_ref, const Vector2d &pt_curr, const SE3d &T_C_R,
    const Vector2d &epipolar_direction, Mat &depth, Mat &depth_cov2) {
    // 不知道这段还有没有人看
    // 用三角化计算深度
    SE3d T_R_C = T_C_R.inverse();
    Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    Vector3d f_curr = px2cam(pt_curr);
    f_curr.normalize();

    // 方程
    // d_ref * f_ref = d_cur * (R_RC * f_cur) + t_RC
    // f2 = R_RC * f_cur
    // 转化成下面这个矩阵方程组
    // => [ f_ref^T f_ref,  -f_ref^T f2 ] [d_ref]   [f_ref^T t]
    //    [ f_cur^T f_ref,  -f2^T   f2  ] [d_cur] = [f2^T   t ]
    Vector3d t = T_R_C.translation();
    Vector3d f2 = T_R_C.so3() * f_curr;
    Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
    Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f2.dot(f2);
    Vector2d ans = A.inverse() * b;
    Vector3d xm = ans[0] * f_ref;       // ref 侧的结果
    Vector3d xn = t + ans[1] * f2;      // cur 结果
    Vector3d p_esti = (xm + xn) / 2.0;  // P 的位置，取两者的平均
    double depth_estimation = p_esti.norm();  // 深度值

    // 计算不确定性（以一个像素为误差）
    Vector3d p = f_ref * depth_estimation;
    Vector3d a = p - t;
    double t_norm = t.norm();
    double a_norm = a.norm();
    double alpha = acos(f_ref.dot(t) / t_norm);
    double beta = acos(-a.dot(t) / (a_norm * t_norm));
    Vector3d f_curr_prime = px2cam(pt_curr + epipolar_direction);
    f_curr_prime.normalize();
    double beta_prime = acos(f_curr_prime.dot(-t) / t_norm);
    double gamma = M_PI - alpha - beta_prime;
    double p_prime = t_norm * sin(beta_prime) / sin(gamma);
    double d_cov = p_prime - depth_estimation;
    double d_cov2 = d_cov * d_cov;

    // 高斯融合
    double mu = depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double sigma2 = depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];

    double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

    depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
    depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;

    return true;
}
```

**三角化方程对应数学**（代码注释展开，对应 §7.5 三角化）：
设参考帧归一化方向 $\boldsymbol f_{\rm ref}$、当前帧归一化方向经旋转到参考系 $\boldsymbol f_2 = R_{RC}\boldsymbol f_{\rm curr}$，深度 $d_{\rm ref}, d_{\rm cur}$ 满足

$$
d_{\rm ref}\,\boldsymbol f_{\rm ref} = d_{\rm cur}\,(R_{RC}\boldsymbol f_{\rm cur}) + \boldsymbol t_{RC} = d_{\rm cur}\,\boldsymbol f_2 + \boldsymbol t.
$$

两边分别左乘 $\boldsymbol f_{\rm ref}^\top$ 与 $\boldsymbol f_2^\top$，得线性方程组（与代码 `A`、`b` 对应）：

$$
\begin{bmatrix} \boldsymbol f_{\rm ref}^\top\boldsymbol f_{\rm ref} & -\boldsymbol f_{\rm ref}^\top\boldsymbol f_2 \\ \boldsymbol f_{\rm cur}^\top\boldsymbol f_{\rm ref} & -\boldsymbol f_2^\top\boldsymbol f_2 \end{bmatrix}
\begin{bmatrix} d_{\rm ref}\\ d_{\rm cur}\end{bmatrix} =
\begin{bmatrix} \boldsymbol f_{\rm ref}^\top\boldsymbol t \\ \boldsymbol f_2^\top\boldsymbol t \end{bmatrix}.
$$

> 注：代码 `A(1,0) = -A(0,1)` 即 $A_{10} = \boldsymbol f_2^\top\boldsymbol f_{\rm ref}$（因 $\boldsymbol f_{\rm ref}^\top\boldsymbol f_2=\boldsymbol f_2^\top\boldsymbol f_{\rm ref}$ 标量对称，且 $A_{01}=-\boldsymbol f_{\rm ref}^\top\boldsymbol f_2$ 故 $A_{10}=-A_{01}=\boldsymbol f_{\rm ref}^\top\boldsymbol f_2$）。解出 $d_{\rm ref}, d_{\rm cur}$ 后，分别得参考侧点 $\boldsymbol{xm}=d_{\rm ref}\boldsymbol f_{\rm ref}$、当前侧点 $\boldsymbol{xn}=\boldsymbol t + d_{\rm cur}\boldsymbol f_2$，取**两者平均** $\boldsymbol p_{\rm esti}=(\boldsymbol{xm}+\boldsymbol{xn})/2$ 作为 $P$，深度 = $\|\boldsymbol p_{\rm esti}\|$。
>
> **不确定性段**完全实现式(12.7)~(12.10)：`alpha`=$\alpha$、`beta`=$\beta$、`beta_prime`=$\beta'$（用 `pt_curr + epipolar_direction` 即沿极线方向偏移一个像素）、`gamma`=$\gamma=\pi-\alpha-\beta'$、`p_prime`=$\|\boldsymbol p'\|$（式12.9）、`d_cov`=$\|\boldsymbol p'\|-\|\boldsymbol p\|$、`d_cov2`=方差。
>
> **高斯融合段**完全实现式(12.6)：`mu_fuse, sigma_fuse2`，其中观测方差用 `d_cov2`、先验用 `sigma2`（即 $\sigma_{\rm obs}^2 \to$ `d_cov2`，$\sigma^2 \to$ `sigma2`，$\mu_{\rm obs}\to$ `depth_estimation`，$\mu\to$ `mu`）。

#### 五点关键函数说明（源 §12.3 正文）

1. **main** 函数非常简单：只负责从数据集读图，然后交给 `update` 函数更新深度图。
2. **update** 函数：遍历参考帧每个像素，先在当前帧寻找极线匹配，能匹配则更新深度图估计。
3. **极线搜索**：原理同 §12.2，实现添加细节——因深度服从高斯分布，以均值为中心左右各取 $\pm3\sigma$ 为半径在当前帧寻极线投影；遍历极线像素（步长取 $\sqrt2/2$ 的近似值 **0.7**）寻 NCC 最高点为匹配点；最高 NCC 也低于阈值（**0.85**）则匹配失败。
4. **NCC** 使用去均值化做法（源 式12.11）：

$$
\mathrm{NCC}_z(\boldsymbol A,\boldsymbol B) = \frac{\displaystyle\sum_{i,j}\bigl(\boldsymbol A(i,j)-\bar{\boldsymbol A}\bigr)\bigl(\boldsymbol B(i,j)-\bar{\boldsymbol B}\bigr)}{\sqrt{\displaystyle\sum_{i,j}\bigl(\boldsymbol A(i,j)-\bar{\boldsymbol A}\bigr)^2 \;\sum_{i,j}\bigl(\boldsymbol B(i,j)-\bar{\boldsymbol B}\bigr)^2}}. \tag{12.11}
$$

> [OCR修正] 源 OCR 把式(12.11)分子写成 $(\boldsymbol A(i,j)-\bar{\boldsymbol A}(i,j))$，其中 $\bar{\boldsymbol A}(i,j)$ 的 $(i,j)$ 是 OCR 冗余——$\bar{\boldsymbol A}$（块均值）是标量，不依赖 $(i,j)$。已按数学意义修正为 $\bar{\boldsymbol A},\bar{\boldsymbol B}$。

5. **三角化**计算方式与 §7.5 一致；不确定性计算与高斯融合方法与 §12.2 一致。

### 实验结果（源 §12.3 实验结果）

编译后以数据集目录为参数运行（原文脚注①）：

```text
$ build/dense_mapping ~/dataset/test_data
read total 202 files.
*** loop 1 ***
*** loop 2 ***
......
```

- 程序输出简洁：仅显示迭代次数、当前图像和深度图。
- 深度图显示的是**深度值乘以 0.4** 后的结果——纯白点（数值 1.0）的深度约 **2.5 米**，颜色越深表示深度值越小（物体离我们越近）。
- 深度估计是**动态过程**——从不确定的初始值逐渐收敛到稳定值。初始值用**均值和方差均为 3.0** 的分布。
- 图 12-6（迭代 10 次与 30 次的结果）：迭代超过一定值后深度图趋于稳定。大致可看出地板和桌子的区别，桌上物体深度接近桌子。**大部分正确，但存在大量错误估计**（表现为与周围数据不一致的过大/过小估计）；**边缘处**因运动中看到次数少而没得到正确估计。

### 12.3.1 实验分析与讨论

- 代码相对简单直接，没用许多技巧（trick），出现实际工程常见情形——**简单的往往并不是最有效的**。
- 真实数据复杂，能实际工作的程序往往需大量工程技巧，极其复杂、难向初学者解释，所以这里用不那么有效但易读易写的实现。
- 下面从**计算机视觉**和**滤波器**两个角度分析。

### 12.3.2 像素梯度的问题 ★（立体视觉核心局限）

- 块匹配正确与否依赖于图像块是否具有**区分度**。若图像块仅是一片黑或一片白（缺有效信息），NCC 计算很可能错误匹配。例：演示程序中的**打印机表面**是均匀白色，极易误匹配，深度信息多半不正确——出现明显不该有的**条纹状深度估计**（直观上打印机表面应是光滑的）。
- 此问题在**直接法**中已见过：块匹配（和 NCC）须假设小块不变。**有明显梯度的小块**区分度好、不易误匹配；**梯度不明显的像素**难以有效估计深度。例：桌面上的杂志、电话等有明显纹理的物体深度较准确。
- 这体现立体视觉一个常见问题：**对物体纹理的依赖性**。双目视觉中也极常见，重建质量十分依赖环境纹理。
- 演示程序刻意用纹理好的环境（棋盘格地板、木纹桌面）才得到看似不错的结果。实际中墙面、光滑物体表面等亮度均匀处经常出现，影响深度估计。**该问题无法在现有算法流程上加以改进解决**——如果依然只关心某像素周围邻域（小块）的话。

#### 像素梯度与极线方向的关系（图 12-7）★

> 图 12-7：像素梯度与极线之关系示意图。参考文献 [75] 详细讨论过它们的关系。

两种极端情况：
- **像素梯度垂直于极线方向**：即使小块有明显梯度，沿极线做块匹配时**匹配程度都一样**，得不到有效匹配。
- **像素梯度平行于极线方向**：能**精确确定**匹配度最高点出现在何处。
- 实际介于二者之间：**梯度与极线夹角较大时，极线匹配的不确定性大；夹角较小时，不确定性变小**。

演示程序把这些情况都当成一个像素的误差，**实际不够精细**。考虑极线与像素梯度的关系，应使用**更精确的不确定性模型**（具体调整改进留作习题，对应习题 3 的"添加仿射变换"思路之外的精细化）。

### 12.3.3 逆深度（Inverse depth）★（参数化技巧）

**参数化问题**：把像素深度假设成高斯分布是否合适？

- 此前常用点的世界坐标 $x,y,z$ 三个量描述（认为三者服从三维高斯）。本讲用图像坐标 $u,v$ 和深度 $d$ 描述空间点（认为 $u,v$ 不动、$d$ 服从一维高斯）。
- **不同参数化的协方差结构**：相机看到某点时其图像坐标 $u,v$ 比较确定（脚注①），深度 $d$ 非常不确定。
  - 用世界坐标 $x,y,z$：根据相机当前位姿，$x,y,z$ 三者间可能存在明显**相关性**，协方差矩阵**非对角元素不为零**。
  - 用 $u,v,d$ 参数化：$u,v$ 和 $d$ 至少近似独立，甚至 $u,v$ 也独立——协方差矩阵**近似对角阵，更简洁**。

**逆深度**是近年 SLAM 研究中广泛使用的参数化技巧（参考文献 [126, 127]）。演示程序假设 $d\sim N(\mu,\sigma^2)$，但深度的正态分布存在问题：

1. 实际想表达"场景深度大概 5~10 米，可能有更远点，但近处肯定不小于相机焦距（或认为不小于 0）"。这个分布**不是对称形状**：尾部可能稍长，负数区域为零。
2. 室外应用可能存在距离非常远乃至无穷远的点。初始值难以涵盖，且用高斯描述会有数值计算困难。

**于是逆深度应运而生**：仿真中发现假设深度的倒数（逆深度）为高斯分布比较有效 [127]；实际应用中逆深度有更好的**数值稳定性**，逐渐成为通用技巧，存在于现有 SLAM 方案的标准做法中 [68, 69, 88]。

**改造方法**：把演示程序从正深度改成逆深度不复杂——只要在前面深度推导中将 $d$ 改成逆深度 $d^{-1}$ 即可（留作习题，对应习题 3）。

### 12.3.4 图像间的变换（仿射变换预处理）★

**动机**：块匹配前做一次图像到图像间的变换是常见预处理。假设了图像小块在相机运动时保持不变，这在相机**平移**时成立（示例数据集基本如此），但相机明显**旋转**时难以保持。特别地，相机绕光心旋转时，一块下黑上白的图像块可能变成上黑下白，导致相关性直接变成负数（尽管仍是同一块）。

**做法**：块匹配前把参考帧与当前帧间的运动考虑进来。根据相机模型，参考帧像素 $P_R$ 与真实三维点世界坐标 $P_W$ 有关系（源 式12.12）：

$$
d_{\mathrm R}\,\boldsymbol P_{\mathrm R} = \boldsymbol K\bigl(\boldsymbol R_{\mathrm{RW}}\boldsymbol P_{\mathrm W} + \boldsymbol t_{\mathrm{RW}}\bigr). \tag{12.12}
$$

对当前帧，$P_W$ 在其上的投影记作 $P_C$（源 式12.13）：

$$
d_{\mathrm C}\,\boldsymbol P_{\mathrm C} = \boldsymbol K\bigl(\boldsymbol R_{\mathrm{CW}}\boldsymbol P_{\mathrm W} + \boldsymbol t_{\mathrm{CW}}\bigr). \tag{12.13}
$$

代入消去 $P_W$，得两幅图像间的像素关系（源 式12.14）：

$$
d_{\mathrm C}\,\boldsymbol P_{\mathrm C} = d_{\mathrm R}\,\boldsymbol K\,\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol K^{-1}\boldsymbol P_{\mathrm R} + \boldsymbol K\,\boldsymbol t_{\mathrm{CW}} - \boldsymbol K\,\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol K\,\boldsymbol t_{\mathrm{RW}}. \tag{12.14}
$$

> **[OCR?] 式(12.14) 末项的潜在错误**：从 (12.12) 解出 $\boldsymbol P_W = \boldsymbol R_{RW}^{\rm T}(d_R\boldsymbol K^{-1}\boldsymbol P_R - \boldsymbol t_{RW})$，代入 (12.13)，末项应为 $-\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\rm T}\boldsymbol t_{RW}$（**不含 $\boldsymbol K$ 在 $\boldsymbol t_{RW}$ 前**，即 $\boldsymbol t_{RW}$ 前的 $\boldsymbol K$ 应不存在）。源 OCR 写作 $\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\rm T}\boldsymbol K\boldsymbol t_{RW}$，末项里 $\boldsymbol t_{RW}$ 前多了一个 $\boldsymbol K$，按量纲与推导应删去。已在此标注，正确式为：
> $$d_C\boldsymbol P_C = d_R\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\rm T}\boldsymbol K^{-1}\boldsymbol P_R + \boldsymbol K\boldsymbol t_{CW} - \boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\rm T}\boldsymbol t_{RW}.$$
> （此为综合 agent 应采用的修正版；保留源式于上以备核对。）

**构造仿射变换**：知道 $d_R, P_R$ 时可算出 $P_C$ 投影位置。再给 $P_R$ 两个分量各一个增量 $\mathrm du, \mathrm dv$，可求 $P_C$ 的增量 $\mathrm du_c, \mathrm dv_c$。由此算出局部范围内参考帧与当前帧图像坐标变换的线性关系，构成**仿射变换**（源 式12.15）：

$$
\begin{bmatrix} \mathrm du_c \\ \mathrm dv_c \end{bmatrix} =
\begin{bmatrix} \dfrac{\mathrm du_c}{\mathrm du} & \dfrac{\mathrm du_c}{\mathrm dv} \\[2mm] \dfrac{\mathrm dv_c}{\mathrm du} & \dfrac{\mathrm dv_c}{\mathrm dv} \end{bmatrix}
\begin{bmatrix} \mathrm du \\ \mathrm dv \end{bmatrix}. \tag{12.15}
$$

根据仿射变换矩阵可将当前帧（或参考帧）像素变换后再块匹配，以期对旋转获得更好效果。

### 12.3.5 并行化：效率的问题 ★（工程权衡）

- 稠密深度图估计**非常费时**：要估计的点从数百个特征点变成几十万个像素点，主流 CPU 也无法实时计算。
- **关键性质**：这几十万个像素点的深度估计**彼此无关**！使并行化有用武之地。
- 示例程序在二重循环里串行遍历所有像素。实际上下一个像素无须等待上一个，可用**多线程**分别计算每个像素再统一结果。理论上若有 30 万个线程，计算时间和计算一个像素一样。
- **GPU 并行架构非常适合**：单目和双目稠密重建中常用 GPU 并行加速。本书不涉及 GPU 编程，仅指出可能性。根据类似工作，利用 GPU 的稠密深度估计**可在主流 GPU 上实时化**。

### 12.3.6 其他的改进（源 §12.3.6，工程改进清单）

可提出许多改进方案，例如：

1. **空间正则项**：现在各像素完全独立计算，可能存在这个像素深度很小、边上一个又很大的情况。可假设深度图中相邻深度变化不会太大，给深度估计加上**空间正则项**，使深度图更平滑。
2. **显式处理外点（Outlier）**：没有显式处理外点。由于遮挡、光照、运动模糊等，不可能每像素都成功匹配。演示程序只要 NCC 大于一定值就认为成功，**没考虑错误匹配**。
3. **处理错误匹配的方式**：例如参考文献 [124] 提出的**均匀-高斯混合分布**深度滤波器，显式区分内点与外点并概率建模，能较好处理外点数据。但理论较复杂，本书不过多涉及。

**总结性结论**：若细致改进每步做法，有希望得到良好的稠密建图方案。但有些问题存在**理论上的困难**：对环境纹理的依赖、像素梯度与极线方向的关联（及平行情况），很难通过调整代码实现来解决。所以目前虽然双目和移动单目能建立稠密地图，但**通常认为它们过于依赖环境纹理和光照、不够可靠**。

---

## 12.4 RGB-D 稠密建图

### 概述（源 §12.4 引言）

- RGB-D 相机在适用范围内是更好的选择：深度可完全通过传感器硬件测量得到，无须大量计算资源估计。
- RGB-D 的**结构光或飞时（ToF）原理**保证了**深度数据对纹理的无关性**——即使面对纯色物体，只要能反射光就能测到深度。这是 RGB-D 的一大优势。
- **[OCR?] 章节回指**：原文"在第 11 讲中详细讨论的深度估计问题，在 RGB-D 相机中可以完全通过传感器中硬件的测量得到"——《十四讲》二版 RGB-D 相机模型实际在第 5 讲（相机与图像），此处"第 11 讲"可能是 OCR 误识或指代相邻章节的深度滤波/估计内容，综合 agent 请按本书实际章节调整。

**RGB-D 三种主流建图方式**（地图形式分类）：
1. **点云地图（Point Cloud Map）**：最直观最简单——根据估算的相机位姿把 RGB-D 数据转化为点云然后拼接，得到由离散点组成的地图。
2. **表面重建**：对外观有进一步要求、希望估计物体表面，可用**三角网格（Mesh）、面片（Surfel）**建图。
3. **占据网格地图（Occupancy Map）**：希望知道障碍物信息并导航，可通过**体素（Voxel）**建立。

RGB-D 建图涉及的理论知识不多，下面几节直接以实践介绍。GPU 建图超出本书范围，只简单讲原理不演示。

### 12.4.1 实践：点云地图

**点云定义**：由一组离散点表示的地图。最基本的点包含 $x,y,z$ 三维坐标，也可带 $r,g,b$ 彩色信息。RGB-D 相机提供彩色图和深度图，很容易根据内参计算 RGB-D 点云。有相机位姿后，直接把点云加和即得全局点云（§5.4.2 曾给出内外参拼接点云的例子）。实际建图还会对点云加滤波处理。

**本程序用两种滤波器**：
- **统计外点去除滤波器（StatisticalOutlierRemoval）**
- **体素网格降采样滤波器（VoxelGrid filter）**

#### 源文件：`slambook/ch12/dense_RGBD/pointcloud_mapping.cpp`（片段）

```cpp
int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;       // 彩色图和深度图
    vector<Eigen::Isometry3d> poses;            // 相机位姿

    ifstream fin("./data/pose.txt");
    if (!fin) {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    for (int i = 0; i < 5; i++) {
        boost::format fmt("./data/%s/%d.%s");   // 图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "png").str(), -1)); // 使用 -1 读取原始图像

        double data[7] = {0};
        for (int i = 0; i < 7; i++) {
            fin >> data[i];
        }
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 计算点云并拼接
    // 相机内参
    double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;

    cout << "正在将图像转换为点云..." << endl;

    // 定义点云使用的格式：这里用的是 XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < 5; i++) {
        PointCloud::Ptr current(new PointCloud);
        cout << "转换图像中：" << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u];   // 深度值
                if (d == 0) continue;   // 为 0 表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }

        // depth filter and statistical removal
        PointCloud::Ptr tmp(new PointCloud);
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);
        (*pointCloud) += *tmp;
    }

    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;

    // voxel filter
    pcl::VoxelGrid<PointT> voxel_filter;
    double resolution = 0.03;
    voxel_filter.setLeafSize(resolution, resolution, resolution);   // resolution
    PointCloud::Ptr tmp(new PointCloud);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);

    cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;

    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
    return 0;
}
```

**RGB-D 像素→三维点的反投影公式**（代码对应数学）：
$$
z = \frac{d}{\text{depthScale}},\qquad x = \frac{(u-c_x)\,z}{f_x},\qquad y = \frac{(v-c_y)\,z}{f_y},
$$
其中 `depthScale=5000.0`（深度图整数值到米的比例尺）；$d=0$ 表示未测到，跳过。世界坐标 $\boldsymbol P_W = T\,\boldsymbol P_{\rm cam}$。彩色通道按 BGR 顺序从 `color.data` 取（`p.b, p.g, p.r`）。

**安装 PCL**（Ubuntu 18.04）：

```bash
sudo apt-get install libpcl-dev pcl-tools
```

**与第 5 讲的不同之处**（三点工程处理）：
1. 生成每帧点云时**去掉深度值无效的点**：考虑 Kinect 有效量程，超过量程的深度值误差大或返回零。
2. **统计滤波器去孤立点**：统计每个点与距它最近的 $N$ 个点的距离值分布，去除距离均值过大的点。保留"粘在一起"的点、去掉孤立噪声点。（代码 `setMeanK(50)` 即 $N=50$，`setStddevMulThresh(1.0)` 即标准差倍数阈值 1.0）
3. **体素网格滤波器降采样**：多视角视野重叠区域存在大量位置相近的点，占内存。体素滤波保证某一定大小立方体（体素）内仅有一个点，相当于三维空间降采样，节省存储。

**数据集与效果**：用 **ICL-NUIM** 数据集（仿真 RGB-D，无噪声深度，方便实验）。`data/` 下存 5 张图像、深度图及相机位姿。体素滤波分辨率设 **0.03**（每 $0.03\times0.03\times0.03$ 格子只存一点）——高分辨率，肉眼感觉不出地图差异，但点数明显减少：**从 130 万个点减少到 3 万个点，只需 2% 的存储空间**。

**运行**：

```bash
./build/pointcloud_mapping
```

得到点云文件 `map.pcd`，用 `pcl_viewer` 打开查看（图 12-8：ICL-NUIM 五张图像重建的结果，体素滤波之后的点云）。

#### 点云地图对地图需求的满足度（源 §12.4.1 评价，逐条）

1. **定位需求**：取决于前端视觉里程计方式。基于特征点的 VO——点云中没存特征点信息，无法用于基于特征点的定位。前端是点云 ICP——可考虑将局部点云对全局点云做 ICP 估计位姿，但要求全局点云精度较好。本程序没对点云本身优化，所以不够。
2. **导航与避障需求**：**无法直接用于导航和避障**。纯点云无法表示"是否有障碍物"，也无法做"任意空间点是否被占据"的查询（这是导航避障的基本需要）。可在点云基础上加工得到更适合的地图形式。
3. **可视化和交互**：具有基本能力，能看场景外观、能漫游。但点云只含离散点、**没有物体表面信息（如法线）**，不太符合可视化习惯——从正面和背面看点云物体一样，还能透过物体看到背后的东西，都不符合日常经验。

**结论**：点云地图是"基础/初级的"，更接近传感器原始数据。有基本功能，通常用于**调试和基本显示**，不便直接用于应用。但它是好的出发点：
- 针对导航：可从点云构建**占据网格（Occupancy Grid）地图**供导航算法查询某点是否可通过。
- SfM 常用的**泊松重建 [128]** 能通过点云重建物体网格地图，得物体表面信息。
- **Surfel [129]**：以面元为地图基本单位，能建立漂亮的可视化地图。

> 图 12-9：泊松重建与 Surfel 重建示意图——视觉效果明显优于纯点云，且都可通过点云构建。大部分由点云转换的地图形式都在 PCL 库中提供。

### 12.4.2 从点云重建网格

**思路**：先计算点云的**法线**，再从法线计算**网格**。

#### 源文件：`slambook2/ch12/dense_RGBD/surfel_mapping.cpp`

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/impl/mls.hpp>

// typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloud;
typedef pcl::PointCloud<SurfelT>::Ptr SurfelCloudPtr;

SurfelCloudPtr reconstructSurface(
    const PointCloudPtr &input, float radius, int polynomial_order) {
    pcl::MovingLeastSquares<PointT, SurfelT> mls;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(radius * radius);
    mls.setPolynomialFit(polynomial_order > 1);
    mls.setPolynomialOrder(polynomial_order);
    mls.setInputCloud(input);
    SurfelCloudPtr output(new SurfelCloud);
    mls.process(*output);
    return (output);
}

pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr &surfels) {
    // Create search tree
    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.05);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);   // 45 degrees
    gp3.minimumAngle(M_PI / 18);            // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);      // 120 degrees
    gp3.setNormalConsistency(true);

    // Get result
    gp3.setInputCloud(surfels);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);

    return triangles;
}

int main(int argc, char **argv) {
    // Load the points
    PointCloudPtr cloud(new PointCloud);
    if (argc == 0 || pcl::io::loadPCDFile(argv[1], *cloud)) {
        cout << "failed to load point cloud!";
        return 1;
    }
    cout << "point cloud loaded, points: " << cloud->points.size() << endl;

    // Compute surface elements
    cout << "computing normals ... " << endl;
    double mls_radius = 0.05, polynomial_order = 2;
    auto surfels = reconstructSurface(cloud, mls_radius, polynomial_order);

    // Compute a greedy surface triangulation
    cout << "computing mesh ... " << endl;
    pcl::PolygonMeshPtr mesh = triangulateMesh(surfels);

    cout << "display mesh ... " << endl;
    pcl::visualization::PCLVisualizer vis;
    vis.addPolylineFromPolygonMesh(*mesh, "mesh frame");
    vis.addPolygonMesh(*mesh, "mesh");
    vis.resetCamera();
    vis.spin();
}
```

**关键参数**：MLS 搜索半径 `mls_radius=0.05`、多项式阶 `polynomial_order=2`；GP3 最大边长 `setSearchRadius(0.05)`、`setMu(2.5)`、最大近邻 100、最大表面角 45°、最小角 10°、最大角 120°、法线一致性 true。

**运行**：

```bash
./build/surfel_mapping map.pcd
```

将点云转换为网格地图（图 12-10：从点云重建得到的表面和网格模型）。重建网格后原本没有表面信息的点云可构建出**法线、纹理**等信息。本节算法 **Moving Least Square（MLS）和 Greedy Projection（GP3）** 见参考文献 [130]、[131]（经典重建算法）。

### 12.4.3 八叉树地图（OctoMap）★（log-odds 完整推导）

**引出**：介绍一种导航中常用、本身有较好压缩性能的地图形式：**八叉树地图（OctoMap）**。

**点云地图的两个明显缺陷**：
1. **规模很大**：640×480 图像产生 30 万个空间点，需大量存储；滤波后 pcd 仍很大。且"大"并非必需——提供很多不必要细节（地毯褶皱、阴暗影子）。除非降分辨率否则有限内存无法建模较大环境，但降分辨率会致地图质量下降。
2. **无法处理运动物体**：做法里只有"添加点"没有"点消失时移除"。运动物体普遍存在使点云地图不够实用。

**八叉树（Octree）[132]**：灵活、压缩、能随时更新的地图形式。

#### 八叉树结构（图 12-11）

> 图 12-11：八叉树示意图。

- 把三维空间建模为许多小方块（体素）。把一个小方块每个面平均切成两片，小方块变成同样大小的**八个**小方块。不断重复直到方块大小达最高精度。
- "将一个小方块分成八个"= "从一个节点展开成八个子节点"，整个从最大空间细分到最小空间的过程就是一棵**八叉树**。
- 大方块=**根节点**，最小块=**叶子节点**。由下一层往上走一层，地图体积扩大为**八倍**。

**体积计算（数值例）**：若叶子节点方块大小为 $1\,\mathrm{cm}^3$，限制八叉树为 **10 层**时，总共能建模体积约

$$
8^{10}\ \mathrm{cm}^3 = 1{,}073{,}741{,}824\ \mathrm{cm}^3 \approx 1{,}073\ \mathrm{m}^3,
$$

足够建模一间屋子。体积与深度呈指数关系，更大深度时建模体积增长非常快。

> 注：原文写 "$8^{10}\,\mathrm{cm}^3 = 1{,}073\,\mathrm{m}^3$"。核对：$8^{10}=2^{30}=1{,}073{,}741{,}824\,\mathrm{cm}^3$，$1\,\mathrm m^3=10^6\,\mathrm{cm}^3$，故 $\approx 1073.7\,\mathrm m^3$，与原文 1,073 一致。

#### 八叉树为何省空间

- 点云体素滤波也限制一体素一点，为何说点云占空间、八叉树省空间？
- **因为八叉树节点存储"是否被占据"信息**：当某方块的所有子节点**都被占据或都不被占据**时，**无须展开**这个节点。地图为空白时只需一个根节点。添加信息时，实际物体常连在一起、空白也常连在一起，所以大多数节点无须展开到叶子层面——**八叉树比点云节省大量存储**。

#### 占据概率与 log-odds 推导 ★

- **0-1 表示的不足**：可用 0 表示空白、1 表示占据（一个比特存储），但过于简单。噪声会使某点一会儿 0 一会儿 1；或还有"未知"状态。
- **概率表示**：用浮点数 $x\in[0,1]$ 表达占据。初始 $x=0.5$；不断观测到占据则增大，不断观测到空白则减小。
- **问题**：让 $x$ 不断增减可能跑到 $[0,1]$ 之外，处理不便。

**解决：用概率对数值（Log-odds）描述**。设 $y\in\mathbb R$ 为概率对数值，$x$ 为 0~1 的概率，两者由 **logit 变换**联系（源 式12.16）：

$$
y = \operatorname{logit}(x) = \log\!\left(\frac{x}{1-x}\right). \tag{12.16}
$$

其**反变换**为（源 式12.17）：

$$
x = \operatorname{logit}^{-1}(y) = \frac{\exp(y)}{\exp(y)+1}. \tag{12.17}
$$

**性质**：$y$ 从 $-\infty$ 变到 $+\infty$ 时，$x$ 相应从 0 变到 1；$y=0$ 时 $x=0.5$。因此存储 $y$ 表达节点是否被占据：观测到"占据"则 $y$ 增加一个值，否则减小一个值；查询概率时用逆 logit 变换。

**递推更新公式**：设某节点为 $n$，观测数据为 $z$。从开始到 $t$ 时刻该节点的概率对数值为 $L(n\mid z_{1:t})$，则 $t+1$ 时刻为（源 式12.18）：

$$
L(n\mid z_{1:t+1}) = L(n\mid z_{1:t}) + L(n\mid z_{t+1}). \tag{12.18}
$$

> **[OCR修正] 式(12.18)**：源 OCR 写作 $L(n\mid z_{1:t+1}) = L(n\mid z_{1:t-1}) + L(n\mid z_t)$，**下标错乱**（左边 $t+1$、右边出现 $t-1$ 和 $t$）。按 OctoMap 标准 log-odds 递推（当前估计 + 新观测的对数几率增量），正确应为 $L(n\mid z_{1:t+1}) = L(n\mid z_{1:t}) + L(n\mid z_{t+1})$。已修正。

**概率形式（不用对数）**（源 式12.19，较复杂）：

$$
P(n\mid z_{1:T}) = \left[\,1 + \frac{1-P(n\mid z_T)}{P(n\mid z_T)}\,\frac{1-P(n\mid z_{1:T-1})}{P(n\mid z_{1:T-1})}\,\frac{P(n)}{1-P(n)}\,\right]^{-1}. \tag{12.19}
$$

> 注：式(12.19) 是 OctoMap 论文 [132] 的标准占据概率融合公式（$P(n)$ 为先验，一般取 0.5；$P(n\mid z_T)$ 为本次测量的逆传感器模型概率）。对照 log-odds 形式可见对数形式（式12.18）远简洁，这正是采用 log-odds 的动机。

**基于 RGB-D 更新八叉树**：观测到某像素带深度 $d$，说明：在深度值对应的空间点上观测到一个**占据**数据；且从相机光心到该点的线段上应**没有物体**（否则会被遮挡）。利用这个信息能很好地更新八叉树地图，**并能处理运动的结构**。

### 12.4.4 实践：八叉树地图

**安装 octomap 库**（Ubuntu 18.04 之后，八叉树及可视化工具 octovis 已集成）：

```bash
sudo apt-get install liboctomap-dev octovis
```

#### 源文件：`slambook/ch13/dense_RGBD/octomap_mapping.cpp`（片段）

> 注：源标注路径为 `ch13`，疑为 OCR/排版误（其余 RGB-D 程序均在 `ch12`），[OCR?] 应为 `ch12`。

```cpp
// octomap tree
octomap::OcTree tree(0.01);   // 参数为分辨率

for (int i = 0; i < 5; i++) {
    cout << "转换图像中：" << i + 1 << endl;
    cv::Mat color = colorImgs[i];
    cv::Mat depth = depthImgs[i];
    Eigen::Isometry3d T = poses[i];

    octomap::Pointcloud cloud;   // the point cloud in octomap

    for (int v = 0; v < color.rows; v++)
        for (int u = 0; u < color.cols; u++) {
            unsigned int d = depth.ptr<unsigned short>(v)[u];   // 深度值
            if (d == 0) continue;   // 为 0 表示没有测量到
            Eigen::Vector3d point;
            point[2] = double(d) / depthScale;
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;
            Eigen::Vector3d pointWorld = T * point;
            // 将世界坐标系的点放入点云
            cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]);
        }

    // 将点云存入八叉树地图，给定原点，这样可以计算投射线
    tree.insertPointCloud(cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
}

// 更新中间节点的占据信息并写入磁盘
tree.updateInnerOccupancy();
cout << "saving octomap ... " << endl;
tree.writeBinary("octomap.bt");
```

> 关键点：
> - `octomap::OcTree tree(0.01)` 构造分辨率 0.01 m 的八叉树。
> - `tree.insertPointCloud(cloud, origin)` 中 `origin = octomap::point3d(T(0,3),T(1,3),T(2,3))` 即相机光心（位姿平移分量），**给定原点才能计算投射线**（光心→点连线上的体素标记为空闲，端点标记为占据）。
> - `updateInnerOccupancy()` 更新中间节点占据信息；`writeBinary("octomap.bt")` 存为压缩二进制。

**说明**（源 §12.4.4 正文）：
- octomap 提供多种八叉树（带地图的、带占据信息的，可自定义节点变量）。本例用**不带颜色信息**的最基本八叉树。
- 八叉树内部点云结构比 PCL 简单，只携带空间位置。流程：RGB-D 图 + 位姿 → 转世界坐标 → 放入八叉树点云 → 交给八叉树 → 根据投影信息更新内部占据概率 → 保存压缩八叉树（`octomap.bt`）。
- 可视化用 **octovis**（编译 octovis 时已安装）。

> 图 12-12：八叉树地图在不同分辨率下的显示结果（0.05 米分辨率、0.1 米分辨率两图）。打开地图显示灰色（无颜色信息），按 "**1**" 键可根据高度信息染色。

**分辨率调节**：octovis 右侧是八叉树**深度限制条**调节分辨率。构造时默认深度 **16 层**，故 16 层即最高分辨率（每小块边长 **0.05 m**）。减少一层，叶子节点上提一层，小块边长**增加一倍**变 0.1 m。可很容易调节地图分辨率适应不同场合。

**八叉树优势（文件大小对比，数值例）**：可方便查询任意点的占据概率，设计导航方法 [133]。**文件大小对比**：§12.3 生成的点云地图磁盘文件约 **6.9 MB**，而八叉树地图只有 **56 KB**——**连点云地图的 1% 都不到**，可有效建模较大场景。

---

## 12.5 * TSDF 地图和 Fusion 系列（实时三维重建，仅原理）★

> 本节涉及 GPU 编程，未提供参考例子，作为**可选阅读材料**。

### 实时三维重建 vs SLAM 的范式差异

- **此前地图模型以定位为主体**，地图拼接作为后续加工步骤放在 SLAM 框架中。这种框架成主流因为：定位可满足实时性需求，地图加工可在关键帧处处理无须实时响应。定位通常轻量（尤其稀疏特征/稀疏直接法）；地图表达与存储重量级、规模和计算需求大，**稠密地图往往只能在关键帧层面计算**。
- **现有做法没有对稠密地图优化**：两幅图像都观察到同一把椅子时，只根据两幅图像位姿把两处点云叠加。位姿估计带误差，直接拼接不够准确——同一把椅子点云无法完美叠加，地图中出现椅子的**两个重影**，称为"**鬼影**"。
- **新范式**：以"**建图**"为主体、定位居次的做法——**实时三维重建**。把重建准确地图作为主要目标，需 GPU 加速（甚至非常高级的 GPU 或多 GPU 并行），通常需较重的计算设备。
- **方向对比**：SLAM 朝**轻量级、小型化**发展（有些方案甚至放弃建图和回环检测、只保留 VO）；实时重建则朝**大规模、大型动态场景**重建发展。

### Fusion 系列（实时重建谱系，图 12-13）

自 RGB-D 传感器出现，利用 RGB-D 图像实时重建成为重要方向，陆续产生：
- **Kinect Fusion [134]**：完成基本模型重建，但仅限**小型场景**。
- **Dynamic Fusion [135]**
- **Elastic Fusion [136]**
- **Fusion4D [137]**
- **Volumn Deform [138]**（VolumeDeform）

后续工作把 Kinect Fusion 向**大型的、运动的、甚至变形场景**拓展。看成实时重建一大类工作，种类繁多。

> 图 12-13：各种实时三维重建模型——(a) Kinect Fusion；(b) Dynamic Fusion；(c) Volumn Deform；(d) Fusion4D；(e) Elastic Fusion。建模结果非常精细，比单纯拼接点云细腻得多。

### TSDF 地图原理（图 12-14）★

**TSDF = Truncated Signed Distance Function = 截断符号距离函数**（"函数"称"地图"虽不太妥当，但暂称 TSDF 地图/TSDF 重建）。

**结构**（与八叉树相似，网格式/方块式）：
- 先选定要建模的三维空间（比如 $3\,\mathrm m\times3\,\mathrm m\times3\,\mathrm m$），按一定分辨率分成许多小块，存储每个小块内部信息。
- **不同点**：TSDF 地图整个存储在**显存**而非内存。利用 GPU 并行特性，可**并行**对每个体素计算和更新（不像 CPU 遍历内存那样串行）。

**每个 TSDF 体素存储该小块与距其最近物体表面的距离**：
- 小块在物体表面**前方** → **正值**；
- 小块在表面**之后** → **负值**；
- 物体表面通常很薄一层，把值太大/太小的都取成 **1 和 -1**（截断），得到**截断后的距离 TSDF**。
- 按定义 **TSDF 为 0 处就是表面本身**；或由于数值误差，**TSDF 由负号变正号处就是表面**。

> 图 12-14：TSDF 示意图。空间尺寸 $d_x=d_y=d_z=3\,[\mathrm{meters}]$；体素数 $V_x=V_y=V_z=\{32,64,128,256,512\}\,[\mathrm{voxels}]$；下部可见类似人脸的表面出现在 TSDF 改变符号处。

**TSDF 的"定位"与"建图"**（与 SLAM 相似但形式不同）：
- **定位**：把当前 RGB-D 图像与 GPU 中的 TSDF 地图比较，**估计相机位姿**。
- **建图**：根据估计的相机位姿对 TSDF 地图**更新**。
- 传统做法还会对 RGB-D 图像做一次**双边贝叶斯滤波**去除深度图噪声。

**TSDF 定位的特点**（GPU-ICP）：
- 类似前面介绍的 **ICP**，但由于 GPU 并行化，可对**整张深度图和 TSDF 地图**做 ICP 计算，不必像传统 VO 先算特征点。
- TSDF 没有颜色信息 → **只用深度图、不用彩色图**就能完成位姿估计——在一定程度上摆脱了 VO 对光照和纹理的依赖，使 RGB-D 重建更稳健（原文脚注①）。
- 建图部分也是并行更新 TSDF 数值的过程，使估计表面更平滑可靠。

（具体 GPU 方法不展开，请参阅相关文献。）

---

## 12.6 小结（源 §12.6）

- 本讲介绍了常见地图类型，**尤其稠密地图形式**。
- 单目/双目可构建稠密地图，相比之下 **RGB-D 地图往往更容易、稳定**。
- 本讲地图**偏重度量地图**；**拓扑地图**因和 SLAM 研究差别大，**没有详细展开**。

---

## 习题（源 习题，逐条照录）

1. 推导式(12.6)。〔即两个高斯归一化积的均值/方差，见本文 §12.2.3 补全推导〕
2. 把本讲的稠密深度估计改成**半稠密**——可以先把梯度明显的地方筛选出来。
3. \* 把本讲演示的单目稠密重建代码从**正深度改成逆深度**，并**添加仿射变换**。实验效果是否有改进？〔对应 §12.3.3、§12.3.4〕
4. 你能论证如何在八叉树中进行**导航或路径规划**吗？
5. 研究参考文献 [134]（KinectFusion），探讨 **TSDF 地图如何进行位姿估计和更新**，它和之前讲过的定位建图算法有何异同？

---

## 本章参考文献编号速查（出现于正文）

- [68][69][88]：逆深度成为 SLAM 标准做法的方案；[68][124]：均匀-高斯混合分布深度滤波器。
- [75]：观测不确定性同时考虑几何 + 光度不确定性；像素梯度与极线关系详述。
- [88][119][120]：图 12-1 各种地图示例来源。
- [121]：极线搜索 + 块匹配技术（REMODE）；[121][125]：REMODE 测试数据集。
- [122][123]：更多块匹配度量方法。
- [124]：仅考虑几何不确定性的深度滤波器；图 12-3 匹配得分分布来源；均匀-高斯混合分布滤波器原始论文。
- [126][127]：逆深度参数化技巧；[127]：仿真发现逆深度为高斯有效。
- [128]：泊松重建（SfM）。
- [129]：Surfel 面元建图。
- [130][131]：MLS（Moving Least Squares）与 GP3（Greedy Projection）经典重建算法。
- [132]：八叉树（OctoMap）。
- [133]：基于八叉树占据概率的导航方法。
- [134]：Kinect Fusion；[135]：Dynamic Fusion；[136]：Elastic Fusion；[137]：Fusion4D；[138]：Volumn Deform（VolumeDeform）。

---

## OCR 修正说明（本抽取发现并修正的 OCR/源识别错）

> 源为图像 OCR 所得。以下逐处列出发现的错误与修正依据。综合 agent 应采用「修正后」版本。

1. **式(12.18) 下标错乱**（源行 1080）：源 OCR 作 $L(n\mid z_{1:t+1}) = L(n\mid z_{1:t-1}) + L(n\mid z_t)$。按 OctoMap 标准 log-odds 递推（旧估计 + 新观测对数几率），修正为 $L(n\mid z_{1:t+1}) = L(n\mid z_{1:t}) + L(n\mid z_{t+1})$。**已修正**。

2. **式(12.14) 末项多余的 $\boldsymbol K$**（源行 700）：源作 $-\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\rm T}\boldsymbol K\boldsymbol t_{RW}$。从 (12.12)→(12.13) 消元推导，末项 $\boldsymbol t_{RW}$ 前不应有 $\boldsymbol K$，正确为 $-\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\rm T}\boldsymbol t_{RW}$。**已在正文标注修正**（保留源式备核）。

3. **式(12.11) 块均值的冗余下标**（源行 603）：源把去均值项写成 $(\boldsymbol A(i,j)-\bar{\boldsymbol A}(i,j))$，块均值 $\bar{\boldsymbol A}$ 是标量不依赖 $(i,j)$。修正为 $\bar{\boldsymbol A},\bar{\boldsymbol B}$。**已修正**。

4. **极线搜索步长注释口径不一**（源行 468 vs 598）：代码注释 `l += sqrt(2)`，但实际步长是 `0.7`、正文说"$\sqrt2/2$ 的近似值 0.7"。即 `sqrt(2)` 注释误导，真实是 $\sqrt2/2\approx0.707\to0.7$。**已在代码处标注**（保留源码注释原样）。

5. **`px2cam` 行 OCR 拼接错位**（源行 444-447、497）：原 OCR 多处把函数体跨代码块断裂（如 `epipolarSearch` 头被拆成两个 fenced block、`NCC` 内层循环被拆开）。已按 C++ 语法重组为完整连续函数。**已重建**。

6. **源码原拼写保留**（非 OCR 错，源码本身如此）：`boarder`（应为 border）、`demoniator`（应为 denominator）——保留原样以与 GitHub 源码一致，仅在注释指出。

7. **章节/路径疑似错**：
   - §12.4 引言"第 11 讲中详细讨论的深度估计"——《十四讲》二版 RGB-D 相机模型在第 5 讲，"第 11 讲"疑为 OCR 误识或指代深度滤波相邻内容，**已标 [OCR?]**，请综合 agent 按本书实际章节调整。
   - octomap 程序源路径标注 `slambook/ch13/dense_RGBD/...`，其余 RGB-D 程序均在 `ch12`，疑为 `ch13`→`ch12` 误识，**已标 [OCR?]**。

8. **代码语言围栏标记混乱**（源把 C++ 片段标成 `c`/`txt`/`matlab`/`batch`/`shell` 等）：这是 OCR/Markdown 转换误标，本抽取统一改为 `cpp`/`bash`/`text` 等正确语言标记，**不影响代码内容**。

9. **数学符号 OCR 常见错的核对**：源中 `∈` 多处正确（如 $A\in\mathbb R^{w\times w}$），但个别位置 OCR 边界符号、上下标、$\langle\cdot\rangle$ 内积记号需结合上下文重建；式(12.7)~(12.10) 几何量已逐一对照代码 `updateDepthFilter` 实现核验无误。所有保留公式均经与代码或标准文献交叉核对。

10. **图注/独立文字行混入正文**（源行 230-262、886-890、1150-1160、1192-1226 等）：OCR 把图片标题、子图标号 (a)~(e)、"t=0/50/..."、"体素滤波之后的点云"等图内文字单独成行混在正文中。已将其归并为对应图的图注说明，不再作为正文段落。

**完整性自评**：源全部 12.1~12.6 小节 + 习题 + 全部公式 (12.1)~(12.19) + 全部 4 段代码（dense_mapping.cpp 含 8 个函数片段、pointcloud_mapping.cpp、surfel_mapping.cpp、octomap_mapping.cpp）+ 全部数值例（$8^{10}$ 体积、130万→3万点 2%、6.9MB vs 56KB、深度×0.4≈2.5m、初始 3.0 等）+ 全部工程权衡（精度-效率、纹理依赖、并行化、外点处理、逆深度、仿射变换、鬼影、GPU）均已全量抽取，无凝练。
