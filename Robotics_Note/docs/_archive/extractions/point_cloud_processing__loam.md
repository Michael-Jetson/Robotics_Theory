# 抽取留痕 — 点云处理经典算法：表示与数据结构 / KD-tree / 滤波降采样 / ICP 系列(point-to-point·point-to-plane·GICP) / NDT / LOAM 特征(边·面)提取与匹配 / scan-to-map 配准

> **本抽取服务章节**：`点云处理`（point cloud processing）。
>
> **定位（极重要，综合 agent 必读）**：同目录已有两份点云/激光相关抽取——
> - `point_cloud_processing__hb_08.md`（SLAM Handbook 第8章，**高层综述**：只有 ICP 统一目标函数 + 距离度量分类 + KD-tree/NDT/GICP 的**用途级**描述，**无任何推导/收敛性/伪码**；其 §"重大缺口"明确点名要"另觅经典源"补 ICP 闭式解与收敛性、point-to-plane 法方程、**GICP（连名字都无）**、**NDT 数学**、**KD-tree 算法**、LOAM 曲率公式）。
> - `lidar_slam__loam_family.md`（LOAM/LeGO-LOAM/LIO-SAM/FAST-LIO，**从 SLAM 系统/里程计角度**抽取 LOAM 特征与 scan-to-scan/scan-to-map 管线，含 LOAM 曲率式与边/面残差几何）。
>
> **本文件正是上面两件点名缺失的"经典源专门件"**：把【点云处理】这门课需要的**全部经典推导**——ICP 点-点闭式解(SVD/四元数)及其**收敛性定理与证明**、point-to-plane 线性化最小二乘法方程、**Generalized-ICP(GICP) 概率生成模型完整推导**(point-to-point / point-to-plane / plane-to-plane 三者统一)、**NDT 概率密度·score·牛顿法梯度海森完整推导**、**KD-tree 构建/最近邻搜索/复杂度的完整伪码**、**体素栅格/统计离群/半径滤波算法**——逐式逐步、全量保真地记录下来。LOAM 特征部分以「点云特征提取」视角重述其**精确曲率定义式与点-边/点-面残差几何**（与 `lidar_slam__loam_family.md` 必有重叠，属同一源不同视角，综合时取其一并互相补全）。
>
> **来源（联网研究，权威原始文献，每条注出处）**：
> 1. **ICP 原论文**：P. J. Besl, N. D. McKay, *"A Method for Registration of 3-D Shapes"*, IEEE Trans. PAMI, vol. 14, no. 2, pp. 239–256, 1992. DOI: 10.1109/34.121791. <https://www.semanticscholar.org/paper/A-Method-for-Registration-of-3-D-Shapes-Besl-McKay/8458412282496da39f71c9f80b266a173e26027e>（ICP 算法、closest-point 算子、单调收敛定理）。
> 2. **点-面 ICP 原论文**：Y. Chen, G. Medioni, *"Object Modeling by Registration of Multiple Range Images"*, Proc. IEEE ICRA 1991, pp. 2724–2729（point-to-plane 度量首次提出）。
> 3. **点-点闭式解（四元数）**：B. K. P. Horn, *"Closed-form solution of absolute orientation using unit quaternions"*, J. Opt. Soc. Am. A, vol. 4, no. 4, pp. 629–642, 1987（Davenport/Horn N 矩阵特征向量解）。
> 4. **点-点闭式解（SVD）**：K. S. Arun, T. S. Huang, S. D. Blostein, *"Least-Squares Fitting of Two 3-D Point Sets"*, IEEE Trans. PAMI, vol. 9, no. 5, pp. 698–700, 1987；与 W. Kabsch, *Acta Cryst.* A32 (1976) 922 同解（trace 最大化 + SVD + 行列式修正）。整理参照 Kabsch/Wahba 词条 <https://en.wikipedia.org/wiki/Kabsch_algorithm>、<https://en.wikipedia.org/wiki/Wahba%27s_problem> 及推导教程 <https://livey.github.io/posts/2024-12-icp/>（已逐字精读，含完整 trace 最大化证明）。
> 5. **GICP 原论文**：A. V. Segal, D. Haehnel, S. Thrun, *"Generalized-ICP"*, Robotics: Science and Systems (RSS) 2009. PDF: <https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf>（**已用 pdftotext 逐字提取全文 509 行**；含 Alg.1、式(1)–(7)、plane-to-plane 协方差构造、收敛/参数讨论）。
> 6. **NDT 2D 原论文**：P. Biber, W. Straßer, *"The Normal Distributions Transform: A New Approach to Laser Scan Matching"*, IEEE/RSJ IROS 2003, pp. 2743–2748（2D NDT、score、牛顿法）。
> 7. **NDT 3D 推广**：M. Magnusson, *"The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection"*, PhD thesis, Örebro University, 2009（3D NDT、稳健化 d1/d2、梯度海森完整式）。概念校核见 <https://en.wikipedia.org/wiki/Normal_distributions_transform>、PCL 教程 <https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html>（参数表）。
> 8. **LOAM 原论文**：J. Zhang, S. Singh, *"LOAM: Lidar Odometry and Mapping in Real-time"*, RSS 2014. PDF: <https://www.roboticsproceedings.org/rss10/p07.pdf>（曲率式、点-线/点-面距离、L-M 求解）。
> 9. **KD-tree**：J. L. Bentley, *"Multidimensional binary search trees used for associative searching"*, Comm. ACM 18(9), 1975；NN 搜索 J. H. Friedman, J. L. Bentley, R. A. Finkel, ACM TOMS 3(3), 1977。算法整理见 <https://en.wikipedia.org/wiki/K-d_tree>（已逐字精读，含构建与 NN 回溯伪码、复杂度）。
>
> **保真承诺**：每一条定义、每一道公式（LaTeX 写全、保留原编号）、每一步代数（中间不跳）、每一段算法伪码、每一个数值/参数，逐条记录，不摘要、不凝练。凡需补全原论文略去的中间步骤（如 ICP 收敛证明的逐项、NDT 牛顿步的链式求导），均明确标注"[抽取补全推导]"。

---

## 0. 记号约定 + 与本书统一约定的差异

### 0.0 本书统一约定（编写规范 §五）

- 旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$（不用 $\mathbf C$）；位姿 $\mathbf T\in\mathrm{SE}(3)$；$\mathbf T=\begin{bmatrix}\mathbf R&\mathbf t\\\mathbf 0^\top&1\end{bmatrix}$。
- 四元数：**Hamilton** 约定（$ij=k$，区别于 JPL）。单位四元数 $q=[q_w,q_x,q_y,q_z]^\top$，标量在前。
- **扰动以右扰动为主**：$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$；右雅可比 $\mathbf J_r$ 为主。
- $\mathfrak{se}(3)$ 排序 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）。
- 协方差用 $\boldsymbol\Sigma$，信息矩阵 $\boldsymbol\Sigma^{-1}=\boldsymbol\Omega$。
- 反对称算子：$\lfloor\mathbf a\rfloor_\times=\hat{\mathbf a}=[\mathbf a]_\times$，满足 $\mathbf a\times\mathbf b=\lfloor\mathbf a\rfloor_\times\mathbf b$。

### 0.1 配准问题的统一记号（本抽取自定义，覆盖 ICP/GICP/NDT/LOAM）

为让各源可比，本文件统一用如下记号（各源原始记号在对应小节括注，并给转换）：

| 记号 | 含义 |
|---|---|
| $\mathcal S=\{\mathbf p_i\}_{i=1}^{N}$ | **source / 待配准点云**（要被变换的那一帧，常记 model/moving）。$\mathbf p_i\in\mathbb R^3$。|
| $\mathcal T=\{\mathbf q_i\}$ | **target / 参考点云**（固定那一帧，常记 reference/fixed/scene）。|
| $(\mathbf R,\mathbf t)$ | 待求刚体变换：$\mathbf R\in\mathrm{SO}(3)$、$\mathbf t\in\mathbb R^3$；点变换 $\mathbf T\!\cdot\!\mathbf p=\mathbf R\mathbf p+\mathbf t$。|
| $\mathbf q_{c(i)}$ 或 $\mathbf m_i$ | 与 $\mathbf p_i$ 对应的最近点（correspondence）。|
| $\mathbf n_i,\boldsymbol\mu_i$ | 点处的**单位法向量**（用于点-面、plane-to-plane）。|
| $d_{\max}$ | 最大匹配距离阈值（剔除外点）。|
| $\boldsymbol\omega\in\mathbb R^3$ | 小角度旋转向量（轴角），$\mathbf R\approx\mathbf I+\lfloor\boldsymbol\omega\rfloor_\times$。|
| $\bar{\mathbf p},\bar{\mathbf q}$ | 点云质心。|
| $\mathbf H,\mathbf W,\mathbf M$ | 互协方差（cross-covariance）矩阵（各源字母不同，见下）。|

**各源原始记号差异速查**：
- **Besl-McKay (ICP 原论文)**：data 点集 $P$（要被配准）、model 点集 $X$；变换写作四元数 $\vec q_R$ + 平移 $\vec q_T$，整体注册向量 $\vec q=[\vec q_R\mid\vec q_T]^\top$；目标函数记 $f(\vec q)$、均方误差 $d_{ms}$。**与本书差异**：Besl 用四元数主线，本书右扰动/Hamilton 四元数兼容；Besl 的 data→model 方向（把 $P$ 配到 $X$）即本书 source→target。
- **GICP (Segal)**：两云 $A=\{\mathbf a_i\}$、$B=\{\mathbf b_i\}$，求 $\mathbf T$ 把 $A$ 对到 $B$（**注意方向：$\mathbf b_i\approx\mathbf T\mathbf a_i$，故 $A$ 是 source、$B$ 是 target**）；残差 $d_i^{(\mathbf T)}=\mathbf b_i-\mathbf T\mathbf a_i$；协方差 $C_i^A,C_i^B$。**与本书差异**：GICP 把 $\mathbf T$ 当作直接作用在齐次/欧氏点上的刚体算子，雅可比对 $\mathbf T$；本书右扰动 $\mathbf T\,\mathrm{Exp}(\delta\boldsymbol\xi)$。GICP 协方差字母 $C$ 即本书 $\boldsymbol\Sigma$。
- **NDT (Biber/Magnusson)**：位姿参数向量 $\mathbf p$（2D 为 $[t_x,t_y,\phi]^\top$，3D 为 $[t_x,t_y,t_z,\text{欧拉}]^\top$ 或李代数）；体素均值 $\mathbf q$（或 $\boldsymbol\mu$）、协方差 $\boldsymbol\Sigma$；得分 score $s(\mathbf p)$。**注意 NDT 的 $\mathbf p$ 是"位姿参数"，与点 $\mathbf p_i$ 同字母**——本抽取在 NDT 节将位姿参数改记 $\mathbf p$（粗体、无下标）、点改记 $\mathbf x_i$，并显式提示。**与本书差异**：NDT 用欧拉角参数化（原论文），移植时换成本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 李代数参数化（牛顿步形式不变，仅雅可比 $\mathbf J=\partial\mathbf x'/\partial\mathbf p$ 换成右扰动雅可比）。
- **LOAM (Zhang)**：见 `lidar_slam__loam_family.md` §0.1；本文件复用其曲率记号 $c$、点-线距 $d_{\mathcal E}$、点-面距 $d_{\mathcal H}$。

---

# 第一部分：点云表示与数据结构

> 本部分把【点云处理】章开篇所需的"点云是什么、怎么存"的基础设施一次讲清——这些在 SLAM Handbook 第8章只点名、技术细节指向其 Chapter 5，故在此用经典文献补全。

## §1.1 点云的形式化定义（源：通用 / SLAM Handbook §8.2.1）

一帧点云是 $\mathbb R^3$（或带属性的高维空间）中有限点集：
$$\mathcal P=\{\mathbf p_i\in\mathbb R^3\}_{i=1}^{N}.\tag{1.1}$$

点可携带**属性**（attributes）：强度 intensity $I_i$、法向量 $\mathbf n_i$、颜色 $(r,g,b)_i$、时间戳 $\tau_i$、ring/扫描线号、回波数等。带属性时一个点是 $\mathbb R^3\times\mathcal A$ 的元素。点云**无序**（点的索引不含几何意义），这是与图像（规则栅格）最本质的区别——任何遍历邻域的操作都需先建空间索引（见 §2）。

**点云的典型来源先验**（源：SLAM Handbook Fig.8.2 图注，机械式 64 线 LiDAR）：
- **近密远疏**：点密度随距离平方衰减（同一立体角内点数固定，远处覆盖面积大）。
- **沿扫描线分层**：机械式多线 LiDAR 的点天然按 ring（仰角通道）分层，同 ring 内沿方位角密集、跨 ring 间距大。
- 这两条先验是后续体素化、降采样、最近邻搜索、特征提取都必须考虑的。

## §1.2 常见点云表示（representations）分类（源：综述综合）

| 表示 | 结构 | 优点 | 缺点 / 适用 |
|---|---|---|---|
| **原始点列 (raw point list)** | $N\times 3$（或 $N\times d$）数组 | 无损、最通用 | 无空间索引，邻域查询需额外结构 |
| **距离图像 (range image / spherical projection)** | 把点按 $(\text{方位角},\text{仰角})$ 投到 $H\times W$ 像素，存 range $r$ | 有序、可借 2D 卷积/图像分割；O(1) 邻域 | 投影有遮挡冲突、量化损失；仅适合单视角扫描 |
| **体素栅格 (voxel grid)** | 把空间均分为边长 $l$ 的立方体素，每体素存占据/统计量 | 规则、可做 3D 卷积、便于降采样 | 内存 $O((\text{范围}/l)^3)$，稀疏场景浪费 → 用**哈希体素 (hashed voxels)** 只存非空体素 |
| **八叉树 (octree)** | 递归把立方均分八子立方，遇空/达最小分辨率/最小点数停 | 自适应分辨率、省内存、支持 kNN 与盒搜索 | 树深处随机访问慢 |
| **KD 树 (k-d tree)** | 二叉树，每节点一个轴对齐分裂超平面（见 §2） | 低维主存 kNN 最快之一 | 插入/删除破坏平衡需重建（→ 增量 KD-tree / ikd-Tree，见 `lidar_slam__fastlio.md`） |
| **面元 (surfel)** | 每个元素 = 中心 + 法向 + 半径(+协方差) 的小圆盘 | 紧凑表面表示、便于融合 | 需估法向；薄结构易丢 |
| **鸟瞰图 (BEV, bird's-eye view)** | 投到地面 $x$-$y$ 栅格，每格存高度/密度 | 适合地面车、可接 2D 检测网络 | 丢竖直细节 |
| **隐式表示 (implicit, e.g. SDF/NeRF)** | 用函数 $f(\mathbf x)$（如符号距离场）表面 | 连续、可微、便于重建 | 训练/求值代价高 |

> **数据结构 vs 表示**：上表"体素/八叉树/KD 树"既是表示也是**空间索引数据结构**——它们的核心价值是把无序点云的**邻域查询**从 $O(N)$ 降到 $O(\log N)$。下一部分专门展开最常用的 KD 树。

---

# 第二部分：最近邻搜索与 KD 树（k-d tree）

> 来源：Bentley 1975（结构）、Friedman-Bentley-Finkel 1977（NN 搜索算法），算法整理逐字精读自 <https://en.wikipedia.org/wiki/K-d_tree>。
> ICP/GICP/NDT 每次迭代都要为 source 的每个点找 target 中的最近点；朴素扫描是 $O(NM)$，KD 树把单次查询降到平均 $O(\log M)$，是点云配准实时化的基石（GICP 原论文 §II 明言"kd-trees for closest-point look up"是 ICP 类方法相对全概率方法的主要速度优势）。

## §2.1 最近邻问题（nearest neighbor search）定义

给定 target 点集 $\mathcal T=\{\mathbf q_j\}_{j=1}^{M}$ 与查询点 $\mathbf p$，**最近邻**为
$$\mathrm{NN}(\mathbf p)=\arg\min_{\mathbf q_j\in\mathcal T}\|\mathbf p-\mathbf q_j\|_2.\tag{2.1}$$
**k 近邻 (kNN)** 返回距离最小的 $k$ 个点（GICP/NDT/LOAM 中拟合局部平面常取 $k=5\sim20$）。**半径搜索 (radius search)** 返回 $\{\mathbf q_j:\|\mathbf p-\mathbf q_j\|\le r\}$。

朴素法逐点比较，单次 $O(M)$；对 $N$ 个查询点共 $O(NM)$。KD 树把 target 预处理成二叉空间划分树，单次平均 $O(\log M)$。

## §2.2 KD 树构建（canonical median-split construction）

**思想**：每层选一个坐标轴（cycling through axes），用该轴上**中位点 (median)** 作分裂超平面，把点集二分，递归。深度 $\text{depth}$ 处用轴 $\text{axis}=\text{depth}\bmod k$（$k$ = 维度，点云 $k=3$）。

**伪码（源 Wikipedia 逐字）**：
```
function kdtree(list of points pointList, int depth):
    var int axis := depth mod k
    select median by axis from pointList         # 按第 axis 维排序取中位点
    node.location   := median
    node.leftChild  := kdtree(points in pointList before median, depth+1)
    node.rightChild := kdtree(points in pointList after  median, depth+1)
    return node
```

要点：
- "As one moves down the tree, one cycles through the axes used to select the splitting planes."（逐层轮换分裂轴。）
- 取**中位点**保证左右子树点数近似相等，得到近似平衡的树（树高 $\approx\log_2 M$）。
- **变体**：可不轮换轴，而在每节点选**方差最大维**或 **range（极差）最大维**作分裂轴，划分更紧凑（这正是 LOAM/FAST-LIO 等"取最长维中位点分裂"的做法，见 `lidar_slam__fastlio.md` §12 ikd-Tree 构建：「递归在最长维上的中位点分裂空间」）。
- **叶桶 (leaf bucket)**：实现上常在叶节点存"一桶点"（bucket size $>1$）以减小树深、提升缓存命中；ikd-Tree 则内部与叶节点都存点（见 fastlio 抽取）。

**构建复杂度**：每层做一次中位选择。若用线性时间中位选择（quickselect / median-of-medians），单层 $O(M)$，共 $\log M$ 层 → **$O(M\log M)$**。若每层排序则 $O(M\log^2 M)$。空间 $O(M)$。

## §2.3 KD 树最近邻搜索（含回溯 / backtracking）

NN 搜索分三阶段（源 Wikipedia 逐字框架 + [抽取补全推导] 的距离判据）：

**伪码（整理）**：
```
function NN_search(node, query q, depth, best):
    if node is null: return best
    axis := depth mod k
    # 1) 下降：按分裂轴决定先进哪侧
    if q[axis] < node.location[axis]:
        near := node.leftChild ;  far := node.rightChild
    else:
        near := node.rightChild;  far := node.leftChild
    best := NN_search(near, q, depth+1, best)          # 先递归更可能含 NN 的一侧
    # 2) 回溯：当前节点自身是否更近
    if dist(q, node.location) < dist(q, best):
        best := node.location
    # 3) 跨平面剪枝：以 q 为心、当前最优距离为半径的超球是否越过分裂面
    if |q[axis] - node.location[axis]| < dist(q, best):
        best := NN_search(far, q, depth+1, best)        # 越过则必须搜另一侧
    return best
```

**剪枝判据的几何含义（[抽取补全推导]）**：当前找到的最近距离为 $r^\ast=\|\mathbf q-\text{best}\|$。分裂超平面 $\{\mathbf x:x_{\text{axis}}=\text{node}[\text{axis}]\}$ 与查询点 $\mathbf q$ 的垂直距离是 $|q_{\text{axis}}-\text{node}_{\text{axis}}|$。"以 $\mathbf q$ 为心、半径 $r^\ast$ 的球"与另一侧半空间相交 $\iff$ 该垂距 $<r^\ast$。仅当相交时，另一侧才**可能**含更近的点，需递归；否则整支被安全剪掉。

- kNN 搜索：把 `best` 换成"距离最大的第 $k$ 近"的优先队列（max-heap，大小 $k$），剪枝半径用堆顶距离。
- 半径搜索：剪枝半径固定为 $r$，收集所有满足者。

**搜索复杂度**：低维、点近似均匀时平均 **$O(\log M)$**；**最坏 $O(M)$**（高维或退化分布时回溯几乎遍历全树——即"维度灾难"，$k\gtrsim10$ 时 KD 树常退化为线性扫描，此时改用近似 NN / ball tree / 哈希）。点云 $k=3$ 故 KD 树非常高效。

## §2.4 增量更新与近似搜索（点名，详见 fastlio 抽取）

- **增量 KD 树 (ikd-Tree)**：原始 KD 树插入/删除点会破坏平衡，需全量重建（代价大）。ikd-Tree（基于 scapegoat tree）支持点级**插入、树上降采样、盒删除**与**部分子树重建再平衡**，使 scan-to-map 配准每步增量更新地图而不卡顿。完整伪码与 $\alpha$-平衡判据见 `lidar_slam__fastlio.md` §12。
- **近似最近邻 (ANN)**：FLANN、nanoflann、libnabo 等库；以及 **range-image 投影搜索**（把 target 投成距离图像，查询点投到同一图像取邻域像素，O(1) 近似邻居）、**体素栅格近似搜索**（查询点所在体素 ± 邻接体素内找）。

---

# 第三部分：滤波与降采样（filtering / downsampling）

> 来源：Point Cloud Library (PCL) 标准滤波器 + LOAM/LIO-SAM 实践参数。原始 LiDAR 单帧动辄数万~数十万点，配准前几乎都要先降采样/去噪——既提速又提精（外点会把 ICP 拉偏）。

## §3.1 体素栅格降采样（voxel-grid downsampling）

**算法**：给定叶尺寸（leaf size）$l$（可各轴不同 $l_x,l_y,l_z$）。
1. 把空间划分为边长 $l$ 的规则体素。每个点 $\mathbf p_i=(x_i,y_i,z_i)$ 落入体素索引
$$\big(\lfloor x_i/l_x\rfloor,\ \lfloor y_i/l_y\rfloor,\ \lfloor z_i/l_z\rfloor\big).\tag{3.1}$$
2. 同一体素内的所有点用**一个代表点**取代。两种代表：
   - **质心 (centroid)**（PCL 默认）：$\displaystyle \mathbf p_{\text{rep}}=\frac{1}{|V|}\sum_{\mathbf p\in V}\mathbf p$。更精确（贴近真实表面），但代价略高。
   - **体素中心 (voxel center)**：取体素几何中心。更快，量化更整齐（ikd-Tree 的"树上降采样"即保留离体素中心最近的那个真实点，见 fastlio §12）。
3. 输出 = 各非空体素的代表点。点数从 $N$ 降到非空体素数。

**性质**：近似均匀化点密度（消除近密远疏带来的偏置——否则 ICP 被近处稠密点主导）；$O(N)$ 时间（哈希体素）。**典型参数**：LOAM 地图降采样 $l=5\,\mathrm{cm}$（`lidar_slam__loam_family.md` §VI）；LIO-SAM 边缘子图 $0.2\,\mathrm m$、平面子图 $0.4\,\mathrm m$；NDT 前常下采到 $0.1\sim0.2\,\mathrm m$。

## §3.2 统计离群点去除（Statistical Outlier Removal, SOR）

**目的**：去除测量噪声/反射造成的孤立离群点。**算法**：
1. 对每个点 $\mathbf p_i$，求其 $k$ 个最近邻，计算到这 $k$ 邻的**平均距离** $\bar d_i=\frac1k\sum_{j=1}^{k}\|\mathbf p_i-\mathbf q_{ij}\|$。
2. 假设全体 $\{\bar d_i\}$ 服从高斯分布，估其全局均值 $\mu$ 与标准差 $\sigma$：
$$\mu=\frac1N\sum_i\bar d_i,\qquad \sigma=\sqrt{\frac1N\sum_i(\bar d_i-\mu)^2}.\tag{3.2}$$
3. **剔除判据**：若 $\bar d_i>\mu+\alpha\,\sigma$ 则 $\mathbf p_i$ 判为离群点删除（$\alpha$ = 标准差倍数阈值，PCL 默认 $k=50,\ \alpha=1.0$）。

直觉：离群点远离邻居 → 其 $\bar d_i$ 异常大 → 落在分布右尾被切掉。

## §3.3 半径离群点去除（Radius Outlier Removal）

**算法**：给定半径 $r$ 与最少邻居数 $k_{\min}$。对每个点统计半径 $r$ 球内的邻居数 $n_i=|\{\mathbf q:\|\mathbf p_i-\mathbf q\|\le r\}|$；若 $n_i<k_{\min}$ 则删除。比 SOR 简单、更快，但需手调 $r$。

## §3.4 直通滤波与地面分割（pass-through / ground removal）

- **直通滤波 (pass-through)**：沿某轴设区间 $[a,b]$，删去坐标在区间外的点（如去掉 $z<-2\,\mathrm m$ 的地面、$z>30\,\mathrm m$ 的远空）。
- **地面去除**：LeGO-LOAM 对距离图像**逐列**估地面平面、标地面点（不参与边特征提取），去除草地/地面带来的伪高曲率边特征（详见 `lidar_slam__loam_family.md` §LeGO.III-B）。一般可用 RANSAC 拟合最大平面作地面。

---

# 第四部分：ICP — 迭代最近点（Iterative Closest Point）

> 原论文：Besl & McKay 1992（PAMI）。本部分给出：(A) ICP 总框架与 closest-point 算子；(B) **点-点闭式解**两条路（SVD/Arun 与四元数/Horn）的**完整推导**；(C) **单调收敛定理与证明**；(D) **点-面 ICP** 的完整线性化最小二乘；(E) 鲁棒化与外点处理。

## §4.A ICP 总框架（源 Besl-McKay；GICP Alg.1 逐字）

ICP 的核心两步循环（GICP 原论文 §III-A 逐字）：
> 1) compute correspondences between the two scans.（计算两云间对应。）
> 2) compute a transformation which minimizes distance between corresponding points.（求最小化对应点间距离的变换。）
迭代重复这两步通常收敛到目标变换。

**最近点算子 (closest point operator)**（源 Besl-McKay）：点 $\mathbf p$ 到点集 $\mathcal A$ 的距离
$$d(\mathbf p,\mathcal A)=\min_{\mathbf a\in\mathcal A}\|\mathbf a-\mathbf p\|,\qquad \mathbf m=\arg\min_{\mathbf a\in\mathcal A}\|\mathbf a-\mathbf p\|.\tag{4.1}$$
对 source 全体 $\mathcal P$ 作此算子得对应集 $\mathcal Y=C(\mathcal P,\mathcal A)$。

**最大匹配阈值 $d_{\max}$**（源 GICP §III-A 逐字）：因实际两云**非完全重叠**（违反 Besl 的"full overlap"假设），须加阈值 $d_{\max}$：对 $\|\mathbf m_i-\mathbf T\mathbf b_i\|>d_{\max}$ 的对应**剔除**（赋权 $w_i=0$）。$d_{\max}$ 在收敛半径与精度间权衡：**取小→"短视"收敛差；取大→错误对应把对齐拉偏**（"A low value results in bad convergence ... a large value causes incorrect correspondences to pull the final alignment away"）。

**标准 ICP 算法（GICP 原论文 Algorithm 1，逐行）**：
> **input** : Two pointclouds $A=\{\mathbf a_i\}$, $B=\{\mathbf b_i\}$；An initial transformation $T_0$
> **output**: The correct transformation $T$ aligning $A$ and $B$
> 1  $T\leftarrow T_0$;
> 2  **while** not converged **do**
> 3 &nbsp;&nbsp;**for** $i\leftarrow 1$ **to** $N$ **do**
> 4 &nbsp;&nbsp;&nbsp;&nbsp;$\mathbf m_i\leftarrow$ FindClosestPointInA$(T\cdot\mathbf b_i)$;
> 5 &nbsp;&nbsp;&nbsp;&nbsp;**if** $\|\mathbf m_i-T\cdot\mathbf b_i\|\le d_{\max}$ **then**
> 6 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$w_i\leftarrow 1$;
> 7 &nbsp;&nbsp;&nbsp;&nbsp;**else**
> 8 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$w_i\leftarrow 0$;
> 9 &nbsp;&nbsp;&nbsp;&nbsp;**end**
> 10 &nbsp;&nbsp;**end**
> 11 &nbsp;&nbsp;$\displaystyle T\leftarrow\arg\min_{T}\Big\{\sum_i w_i\,\|T\cdot\mathbf b_i-\mathbf m_i\|^2\Big\}$;
> 12 **end**

（注：GICP 这里把 $B$ 当 source 配到 $A$；line 11 的最小化即"点-点"目标。下文统一改回 $\mathcal S\to\mathcal T$ 方向：$\min\sum w_i\|\mathbf R\mathbf p_i+\mathbf t-\mathbf q_{c(i)}\|^2$。）

## §4.B 点-点 ICP 的闭式解

固定对应后，**点-点目标函数**：
$$E(\mathbf R,\mathbf t)=\sum_{i=1}^{N}w_i\big\|\mathbf R\mathbf p_i+\mathbf t-\mathbf q_i\big\|_2^2,\qquad \mathbf R\in\mathrm{SO}(3),\ \mathbf t\in\mathbb R^3.\tag{4.2}$$
（此处 $\mathbf q_i:=\mathbf q_{c(i)}$ 已是 $\mathbf p_i$ 的对应；权 $w_i$ 可吸收进点，下设 $w_i=1$，加权情形把质心改成加权质心即可，见末尾备注。）

### §4.B.1 第一步：用质心解耦平移（源 Arun/Kabsch；逐步）

对 $\mathbf t$ 求偏导并置零：
$$\frac{\partial E}{\partial\mathbf t}=\sum_i 2(\mathbf R\mathbf p_i+\mathbf t-\mathbf q_i)=\mathbf 0
\ \Longrightarrow\ \mathbf t=\frac1N\sum_i(\mathbf q_i-\mathbf R\mathbf p_i)=\bar{\mathbf q}-\mathbf R\bar{\mathbf p},\tag{4.3}$$
其中质心 $\bar{\mathbf p}=\frac1N\sum_i\mathbf p_i$、$\bar{\mathbf q}=\frac1N\sum_i\mathbf q_i$。

**定义去心坐标**（[抽取补全推导] 用 prime 标记，避免与原点混淆）：
$$\mathbf p_i'=\mathbf p_i-\bar{\mathbf p},\qquad \mathbf q_i'=\mathbf q_i-\bar{\mathbf q}.\tag{4.4}$$
把 (4.3) 代回 (4.2)：
$$
\begin{aligned}
E&=\sum_i\big\|\mathbf R\mathbf p_i+(\bar{\mathbf q}-\mathbf R\bar{\mathbf p})-\mathbf q_i\big\|^2
=\sum_i\big\|\mathbf R(\mathbf p_i-\bar{\mathbf p})-(\mathbf q_i-\bar{\mathbf q})\big\|^2\\
&=\sum_i\big\|\mathbf R\mathbf p_i'-\mathbf q_i'\big\|^2.
\end{aligned}\tag{4.5}
$$
**平移已被消掉**，只剩对 $\mathbf R$ 的旋转配准问题。

### §4.B.2 第二步：旋转化为 trace 最大化（源 livey 教程逐字 + Arun）

展开 (4.5)：
$$
\sum_i\|\mathbf R\mathbf p_i'-\mathbf q_i'\|^2
=\sum_i\Big(\underbrace{\mathbf p_i'^\top\mathbf R^\top\mathbf R\mathbf p_i'}_{=\|\mathbf p_i'\|^2\ (\mathbf R^\top\mathbf R=\mathbf I)}-2\,\mathbf q_i'^\top\mathbf R\mathbf p_i'+\|\mathbf q_i'\|^2\Big).
$$
前后两项与 $\mathbf R$ 无关，故
$$\min_{\mathbf R\in\mathrm{SO}(3)}E\iff \max_{\mathbf R\in\mathrm{SO}(3)}\sum_i\mathbf q_i'^\top\mathbf R\mathbf p_i'.\tag{4.6}$$
用 $\mathbf a^\top\mathbf b=\mathrm{tr}(\mathbf b\mathbf a^\top)$ 把标量写成 trace（[抽取补全推导]）：
$$\sum_i\mathbf q_i'^\top\mathbf R\mathbf p_i'=\sum_i\mathrm{tr}\!\big(\mathbf R\mathbf p_i'\mathbf q_i'^\top\big)=\mathrm{tr}\!\Big(\mathbf R\underbrace{\textstyle\sum_i\mathbf p_i'\mathbf q_i'^\top}_{=:\mathbf M}\Big)=\mathrm{tr}(\mathbf R\mathbf M),\tag{4.7}$$
定义**互协方差矩阵** $\mathbf M=\sum_i\mathbf p_i'\mathbf q_i'^\top\in\mathbb R^{3\times3}$。于是
$$\boxed{\ \max_{\mathbf R\in\mathrm{SO}(3)}\ \mathrm{tr}(\mathbf R\mathbf M).\ }\tag{4.8}$$
（不同文献 $\mathbf M$ 的转置约定不同：Kabsch 用 $\mathbf H=\sum\mathbf p_i'\mathbf q_i'^\top$ 求 $\max\mathrm{tr}(\mathbf R\mathbf H)$；有的写 $\mathbf W=\sum\mathbf q_i'\mathbf p_i'^\top$ 求 $\max\mathrm{tr}(\mathbf R^\top\mathbf W)$。结论的 SVD 解相应换 $\mathbf U\leftrightarrow\mathbf V$，本节用 (4.8) 约定。）

### §4.B.3 第三步：SVD 解与最优性证明（源 livey 逐字）

对 $\mathbf M$ 作奇异值分解 $\mathbf M=\mathbf U\boldsymbol\Sigma\mathbf V^\top$（$\mathbf U,\mathbf V$ 正交，$\boldsymbol\Sigma=\mathrm{diag}(\sigma_1,\sigma_2,\sigma_3)$，$\sigma_i\ge0$）。代入并用 trace 循环性：
$$\mathrm{tr}(\mathbf R\mathbf M)=\mathrm{tr}(\mathbf R\mathbf U\boldsymbol\Sigma\mathbf V^\top)=\mathrm{tr}(\mathbf V^\top\mathbf R\mathbf U\,\boldsymbol\Sigma)=:\mathrm{tr}(\mathbf Q\boldsymbol\Sigma),\quad \mathbf Q:=\mathbf V^\top\mathbf R\mathbf U.\tag{4.9}$$
$\mathbf Q$ 是正交矩阵（三正交阵乘积），故 $|q_{ii}|\le1$（正交阵各列单位长，对角元绝对值不超 1）。因此
$$\mathrm{tr}(\mathbf Q\boldsymbol\Sigma)=\sum_{i=1}^{3}q_{ii}\sigma_i\le\sum_{i=1}^{3}\sigma_i,\tag{4.10}$$
等号成立当且仅当 $q_{ii}=1\ \forall i$，即 $\mathbf Q=\mathbf I$（正交阵对角元全为 1 ⇒ 单位阵）。于是
$$\mathbf V^\top\mathbf R\mathbf U=\mathbf I\ \Longrightarrow\ \boxed{\ \mathbf R=\mathbf V\mathbf U^\top.\ }\tag{4.11}$$

**行列式修正（反射问题，源 livey/Kabsch 逐字）**：(4.11) 给出的可能是**反射**（$\det=-1$）而非旋转。$\mathrm{SO}(3)$ 要求 $\det\mathbf R=+1$。
- **情形 1**：若 $\det(\mathbf V\mathbf U^\top)=+1$，则 $\mathbf R=\mathbf V\mathbf U^\top$ 即最优旋转。
- **情形 2**：若 $\det(\mathbf V\mathbf U^\top)=-1$，则在 $\mathbf Q$ 上把对应**最小奇异值**的那个对角元改为 $-1$（牺牲最小的 $\sigma_3$），得
$$\boxed{\ \mathbf R=\mathbf V\begin{bmatrix}1&0&0\\0&1&0\\0&0&\det(\mathbf V\mathbf U^\top)\end{bmatrix}\mathbf U^\top.\ }\tag{4.12}$$
（Kabsch 等价写法：$d=\mathrm{sign}\big(\det(\mathbf V\mathbf U^\top)\big)$，$\mathbf R=\mathbf V\,\mathrm{diag}(1,1,d)\,\mathbf U^\top$。Wahba/姿态确定文献写 $\mathbf R=\mathbf U\,\mathrm{diag}(1,1,\det\mathbf U\det\mathbf V)\,\mathbf V^\top$，因其 $\mathbf B=\sum a_i\mathbf w_i\mathbf v_i^\top$ 的转置约定相反。）

最后由 (4.3) 得平移 $\mathbf t=\bar{\mathbf q}-\mathbf R\bar{\mathbf p}$。

> **加权情形**：若各对应有权 $w_i$（如 §4.E 鲁棒权，或 GICP 的信息矩阵权简化），把质心换成加权质心 $\bar{\mathbf p}=\frac{\sum w_i\mathbf p_i}{\sum w_i}$，互协方差换成 $\mathbf M=\sum_i w_i\mathbf p_i'\mathbf q_i'^\top$，其余不变。

### §4.B.4 等价的四元数闭式解（源 Horn 1987 / Davenport q-method）

Besl-McKay 原论文用的是 **Horn 单位四元数法**（与 SVD 等价，给同一最优解，但用 $4\times4$ 特征问题）。

把旋转写成单位四元数 $\mathbf q=[q_w,q_x,q_y,q_z]^\top$（Hamilton，标量在前），最大化 (4.6) 等价于二次型 $\max_{\|\mathbf q\|=1}\mathbf q^\top\mathbf N\mathbf q$，其中对称 $4\times4$ 矩阵 $\mathbf N$ 由互协方差 $\mathbf M$（记其元素 $M_{ab}$）构造。**Davenport K 矩阵构造（源 Wikipedia q-method 逐字，与 Horn 的 N 等价）**：先由"姿态廓形矩阵" $\mathbf B$（即互协方差，$\mathbf B=\sum_i\mathbf q_i'\mathbf p_i'^\top$）算三个量
$$\mathbf S=\mathbf B+\mathbf B^\top,\qquad \mathbf z=\sum_i (\mathbf q_i'\times\mathbf p_i')\ \big(=[B_{23}-B_{32},\ B_{31}-B_{13},\ B_{12}-B_{21}]^\top\big),\qquad \sigma=\mathrm{tr}(\mathbf B),\tag{4.13}$$
组成
$$\mathbf K=\begin{bmatrix}\mathbf S-\sigma\mathbf I & \mathbf z\\[2pt] \mathbf z^\top & \sigma\end{bmatrix}\in\mathbb R^{4\times4}.\tag{4.14}$$
**最优四元数 = $\mathbf K$ 的最大特征值 $\lambda_{\max}$ 对应的单位特征向量**：
$$\mathbf q^\ast=\arg\max_{\|\mathbf q\|=1}\mathbf q^\top\mathbf K\mathbf q=\text{eigvec}(\mathbf K,\lambda_{\max}).\tag{4.15}$$
再由 $\mathbf q^\ast$ 转成 $\mathbf R$。

**为何是最大特征值（[抽取补全推导]）**：对单位约束 $\|\mathbf q\|=1$ 的二次型 $\max\mathbf q^\top\mathbf K\mathbf q$，由 Rayleigh 商 $\frac{\mathbf q^\top\mathbf K\mathbf q}{\mathbf q^\top\mathbf q}$ 的极值理论，最大值在最大特征值处取得，最大化向量即对应特征向量。这与 SVD 解给出**同一旋转**（Horn 1987 与 Arun 1987 同年证明两路等价）。优点：四元数自动满足 $\mathrm{SO}(3)$ 约束（无需 (4.12) 的反射修正——四元数天然是旋转），且数值稳健。

## §4.C ICP 的单调收敛定理与证明（源 Besl-McKay 定理）

**定理（Besl-McKay 1992）**：ICP 迭代产生的均方误差序列 **单调非增、有下界（$\ge0$）**，因而**收敛**；ICP 总是单调收敛到均方距离度量的某个**局部极小**。原文："The ICP algorithm always converges monotonically to the nearest local minimum of a mean-square distance metric, and the rate of convergence is rapid during the first few iterations."

记第 $k$ 次迭代后的注册向量为 $\vec q_k$（变换），均方误差
$$d_k=\frac1N\sum_i\big\|\mathbf T_k\mathbf p_i-\mathbf m_i^{(k)}\big\|^2,\tag{4.16}$$
其中 $\mathbf m_i^{(k)}=C(\mathbf T_k\mathbf p_i)$ 是用第 $k$ 步变换后的点找到的最近对应。

**证明（[抽取补全推导]，Besl-McKay 论文的两步夹逼）**：一次 ICP 迭代分两子步，**各自都不增大误差**。
1. **对应更新步（更新 $\mathbf m_i$，固定变换 $\mathbf T_k$）**：对每个点重新取最近点。由最近点定义 (4.1)，$\mathbf m_i^{(k+1)}=\arg\min_{\mathbf a\in\mathcal A}\|\mathbf T_k\mathbf p_i-\mathbf a\|$ 给出**不大于**旧对应 $\mathbf m_i^{(k)}$ 的距离：$\|\mathbf T_k\mathbf p_i-\mathbf m_i^{(k+1)}\|\le\|\mathbf T_k\mathbf p_i-\mathbf m_i^{(k)}\|$。逐点求和：记此中间误差 $e_k=\frac1N\sum_i\|\mathbf T_k\mathbf p_i-\mathbf m_i^{(k+1)}\|^2\le d_k$。
2. **变换更新步（更新 $\mathbf T$，固定对应 $\mathbf m_i^{(k+1)}$）**：第 $(k{+}1)$ 步的变换 $\mathbf T_{k+1}$ 是 (4.2) 在当前对应下的**全局最小**（由 §4.B 闭式解精确求得）。因此它给出的误差不超过用旧变换 $\mathbf T_k$ 的误差：$d_{k+1}=\frac1N\sum_i\|\mathbf T_{k+1}\mathbf p_i-\mathbf m_i^{(k+1)}\|^2\le \frac1N\sum_i\|\mathbf T_k\mathbf p_i-\mathbf m_i^{(k+1)}\|^2=e_k$。

合并两步：
$$0\le d_{k+1}\le e_k\le d_k.\tag{4.17}$$
即 $\{d_k\}$ 单调非增且有下界 $0$。由**单调有界数列收敛定理**，$d_k\to d_\infty\ge0$。又因变换空间紧致（$\mathrm{SO}(3)$ 紧、平移落在有界域）、对应只有有限种组合，迭代必在有限步内对应不再变化、变换不再更新，到达一个**不动点**（局部极小）。$\blacksquare$

**关于"局部"**（源 GICP §IV / point-set-registration 词条）：ICP 只保证收敛到**局部**极小，强烈依赖初值——初始位姿差时易陷错误极小。实践对策：好的初值（IMU/里程计预对齐）、coarse-to-fine、多初值、扩大 $d_{\max}$ 增大收敛半径（但牺牲精度）。"It is difficult to prove that ICP will in fact converge exactly to the local optimum"（point-set-registration 词条对实际数值收敛的注记：因对应是离散跳变，严格意义上不一定收敛到代价函数的局部最小，但 (4.17) 的单调性在实践中保证稳定下降）。

**收敛速率**（源 Besl-McKay）：前几次迭代下降很快（"rapid during the first few iterations"），后期变慢（接近极小时对应基本稳定，退化为局部二次收敛）。

## §4.D 点-面 ICP（point-to-plane）的完整线性化最小二乘

> 源：度量首见 Chen & Medioni 1991；线性化标准做法（Low 2004 风格）。点-面 ICP 在结构化环境（墙、地面）显著优于点-点，因它**不惩罚沿表面切向的滑动**（解决了两云离散化不同导致点永不精确重合的问题——GICP §II 逐字："Point-to-plane ... solves the discretization problem by not penalizing offsets along a surface"）。

**点-面目标函数**：把 source 点 $\mathbf p_i$ 变换后到对应点 $\mathbf q_i$ 所在**切平面**（法向 $\mathbf n_i$）的距离平方求和：
$$E_{\perp}(\mathbf R,\mathbf t)=\frac12\sum_i\Big(\mathbf n_i^\top\big(\mathbf R\mathbf p_i+\mathbf t-\mathbf q_i\big)\Big)^2.\tag{4.18}$$
（与点-点 (4.2) 的差别：残差先投到法向 $\mathbf n_i$，只剩"点到平面"的垂距。GICP §III-B 给出对应改写：line 11 改成 $T\leftarrow\arg\min_T\sum_i\|\mathbf n_i\cdot(T\mathbf b_i-\mathbf m_i)\|^2$。）

**小角度线性化（源 livey 逐字 + 逐步）**：把旋转写成在当前估计 $\mathbf R_0$ 上的小扰动 $\mathbf R=\exp(\lfloor\boldsymbol\omega\rfloor_\times)\mathbf R_0\approx(\mathbf I+\lfloor\boldsymbol\omega\rfloor_\times)\mathbf R_0$（$\boldsymbol\omega\in\mathbb R^3$ 小轴角；舍去二阶项）。代入残差 $r_i=\mathbf n_i^\top(\mathbf R\mathbf p_i+\mathbf t-\mathbf q_i)$：
$$r_i\approx\mathbf n_i^\top\big(\mathbf R_0\mathbf p_i+\lfloor\boldsymbol\omega\rfloor_\times\mathbf R_0\mathbf p_i+\mathbf t-\mathbf q_i\big).$$
用反对称恒等式 $\lfloor\boldsymbol\omega\rfloor_\times\mathbf x=-\lfloor\mathbf x\rfloor_\times\boldsymbol\omega$（令 $\mathbf x_i:=\mathbf R_0\mathbf p_i$）：
$$r_i\approx\mathbf n_i^\top(\mathbf x_i-\mathbf q_i)+\mathbf n_i^\top\mathbf t-\mathbf n_i^\top\lfloor\mathbf x_i\rfloor_\times\boldsymbol\omega.\tag{4.19}$$
把待求量堆成 6 维 $\mathbf x=[\boldsymbol\omega;\mathbf t]\in\mathbb R^6$，残差线性化为 $r_i\approx\mathbf J_i\,\mathbf x - c_i$，其中
$$\mathbf J_i=\big[\,-\mathbf n_i^\top\lfloor\mathbf x_i\rfloor_\times\ \big|\ \mathbf n_i^\top\,\big]\in\mathbb R^{1\times6},\qquad c_i=\mathbf n_i^\top(\mathbf q_i-\mathbf x_i)=-\mathbf n_i^\top(\mathbf x_i-\mathbf q_i).\tag{4.20}$$
（$c_i$ 是当前残差的负偏置；目标 $\min_{\mathbf x}\sum_i(\mathbf J_i\mathbf x-c_i)^2$。）

**法方程（$6\times6$ 线性系统，源 livey 逐字）**：
$$\Big(\sum_i\mathbf J_i^\top\mathbf J_i\Big)\mathbf x=\sum_i\mathbf J_i^\top c_i\quad\Longleftrightarrow\quad (\mathbf J^\top\mathbf J)\,\mathbf x=\mathbf J^\top\mathbf c.\tag{4.21}$$
解出 $\mathbf x=[\boldsymbol\omega;\mathbf t]$ 后，更新 $\mathbf R\leftarrow(\mathbf I+\lfloor\boldsymbol\omega\rfloor_\times)\mathbf R_0$（或精确 $\exp(\lfloor\boldsymbol\omega\rfloor_\times)\mathbf R_0$ 再正交化），$\mathbf t$ 累加；迭代直至 $\boldsymbol\omega,\mathbf t$ 足够小。**Hessian 显式块结构（源 livey 逐字）**：
$$\mathbf H=\mathbf J^\top\mathbf J=\begin{bmatrix}\displaystyle\sum_i\lfloor\mathbf x_i\rfloor_\times^\top\mathbf n_i\mathbf n_i^\top\lfloor\mathbf x_i\rfloor_\times & -\displaystyle\sum_i\lfloor\mathbf x_i\rfloor_\times^\top\mathbf n_i\mathbf n_i^\top\\[8pt] -\displaystyle\sum_i\mathbf n_i\mathbf n_i^\top\lfloor\mathbf x_i\rfloor_\times & \displaystyle\sum_i\mathbf n_i\mathbf n_i^\top\end{bmatrix},\quad \mathbf x_i=\mathbf R_0\mathbf p_i.\tag{4.22}$$

> 点-面 ICP **无简单闭式解**（GICP §II 逐字："Aside from point-to-plane, most ICP variations use a closed form solution"——意即点-面是例外，需迭代线性化或非线性优化）。本节即其标准高斯-牛顿迭代。**与本书右扰动一致**：$\boldsymbol\omega$ 作为右扰动轴角，$\mathbf J_i$ 的旋转块 $-\mathbf n_i^\top\lfloor\mathbf R_0\mathbf p_i\rfloor_\times$ 即对右扰动 $\delta\boldsymbol\phi$ 的雅可比。

## §4.E 鲁棒化与外点处理

- **硬阈值**：$d_{\max}$ 直接砍掉 $w_i=0$（Alg.1 line 5-9）。
- **鲁棒核 (M-estimator)**：把 $\sum r_i^2$ 换成 $\sum\rho(r_i)$（Huber、Tukey bisquare、Cauchy 等），等价于给每个残差迭代重加权 $w_i=\frac{1}{r_i}\frac{\partial\rho}{\partial r_i}$（IRLS）。LOAM 用 **bisquare 权**（`lidar_slam__loam_family.md` Alg.1 line 13：距对应越远权越小，超阈值赋零）。
- **Trimmed ICP**：只保留残差最小的一定比例（如 70%）对应。
- **点-面/GICP 的隐式鲁棒性**：法向投影/协方差加权使方向不一致的错误对应自动被弱化（见 §5 plane-to-plane 的"自动 discount"）。

---

# 第五部分：Generalized-ICP（GICP，广义 ICP）完整推导

> 原论文逐字：A. V. Segal, D. Haehnel, S. Thrun, *"Generalized-ICP"*, RSS 2009（本文件已 pdftotext 提取全文）。
> **一句话**：GICP 把"点-点 ICP"与"点-面 ICP"统一进**一个概率(MLE)框架**，并据此自然推广出**两云都用表面结构**的 **plane-to-plane**。它只改 Alg.1 的 line 11（变换最小化步），其余（用欧氏距离 + kd-tree 找对应）不变，从而**保持 ICP 的速度与简洁**，同时获得概率方法的鲁棒性。SLAM Handbook 第8章**全文没有 GICP**，故此节是该缺口的唯一来源。

## §5.A 概率生成模型（源 §III-A 逐字 + 逐步）

GICP 只对 line 11 附加概率模型。设最近点查找已完成，两云按对应索引对齐：$A=\{\mathbf a_i\}_{i=1,\dots,N}$、$B=\{\mathbf b_i\}_{i=1,\dots,N}$（$\mathbf a_i$ 对应 $\mathbf b_i$），且已剔除 $\|\mathbf m_i-T\mathbf b_i\|>d_{\max}$ 者。

**生成假设**：存在一组**真实底层点** $\hat A=\{\hat{\mathbf a}_i\}$、$\hat B=\{\hat{\mathbf b}_i\}$，测量点是其加噪样本：
$$\mathbf a_i\sim\mathcal N(\hat{\mathbf a}_i,\,C_i^A),\qquad \mathbf b_i\sim\mathcal N(\hat{\mathbf b}_i,\,C_i^B),\tag{5.0}$$
$C_i^A,C_i^B$ 是各测量点的协方差矩阵。**完美对应假设**：若对应几何上完全正确（无遮挡/采样误差），且变换真值 $\mathbf T^\ast$，则
$$\hat{\mathbf b}_i=\mathbf T^\ast\hat{\mathbf a}_i.\tag{1}$$

对任意刚体变换 $\mathbf T$，定义残差 $d_i^{(\mathbf T)}=\mathbf b_i-\mathbf T\mathbf a_i$。考察在真值 $\mathbf T^\ast$ 处的残差分布：因 $\mathbf a_i,\mathbf b_i$ 抽自独立高斯，
$$
\begin{aligned}
d_i^{(\mathbf T^\ast)}&\sim\mathcal N\big(\hat{\mathbf b}_i-\mathbf T^\ast\hat{\mathbf a}_i,\ C_i^B+\mathbf T^\ast C_i^A\,\mathbf T^{\ast\top}\big)\\
&=\mathcal N\big(\mathbf 0,\ C_i^B+\mathbf T^\ast C_i^A\,\mathbf T^{\ast\top}\big),
\end{aligned}
$$
末行用 (1) 把均值消为零。[抽取补全推导：$\mathbf b_i$ 与 $-\mathbf T^\ast\mathbf a_i$ 独立相加，均值相加、协方差相加；线性变换 $\mathbf T^\ast\mathbf a_i$ 的协方差为 $\mathbf T^\ast C_i^A\mathbf T^{\ast\top}$，旋转部分作用于 $C_i^A$。]

## §5.B 最大似然 → GICP 核心代价（源 §III-A 逐字，式 (2)）

用 MLE 迭代求 $\mathbf T$：
$$\mathbf T=\arg\max_{\mathbf T}\prod_i p\big(d_i^{(\mathbf T)}\big)=\arg\max_{\mathbf T}\sum_i\log p\big(d_i^{(\mathbf T)}\big).$$
代入多元高斯密度 $p(d)\propto\exp\!\big(-\tfrac12 d^\top\Sigma^{-1}d\big)$ 并取对数（常数项略去、负号转 $\arg\min$）：
$$\boxed{\ \mathbf T=\arg\min_{\mathbf T}\sum_i {d_i^{(\mathbf T)}}^{\!\top}\big(C_i^B+\mathbf T\,C_i^A\,\mathbf T^\top\big)^{-1} d_i^{(\mathbf T)}.\ }\tag{2}$$
这是 **GICP 算法的核心步**。每个残差用其**自身协方差的逆**（信息矩阵）加权——马氏距离。注意权矩阵 $C_i^B+\mathbf T C_i^A\mathbf T^\top$ 含 $\mathbf T$，故是非线性最小二乘（用共轭梯度/高斯-牛顿迭代解；原论文 §IV 用共轭梯度）。

## §5.C 退化到点-点 ICP（源 §III-A 逐字，式后）

令
$$C_i^B=\mathbf I,\qquad C_i^A=\mathbf 0.$$
则权矩阵 $C_i^B+\mathbf T C_i^A\mathbf T^\top=\mathbf I$，(2) 退化为
$$\mathbf T=\arg\min_{\mathbf T}\sum_i {d_i^{(\mathbf T)}}^{\!\top} d_i^{(\mathbf T)}=\arg\min_{\mathbf T}\sum_i\|d_i^{(\mathbf T)}\|^2,$$
正是标准点-点 ICP 的 line 11。即**点-点 ICP = 各点各向同性单位协方差**的 GICP 特例。

## §5.D 退化到点-面 ICP（源 §III-A 逐字，式 (3)–(7)）

点-面更新（GICP 式 (4)，逐字）：
$$\mathbf T=\arg\min_{\mathbf T}\Big\{\sum_i\|P_i\cdot d_i^{(\mathbf T)}\|^2\Big\},\tag{4}$$
$P_i$ 是到 $\mathbf b_i$ 处表面法向所张子空间的**正交投影矩阵**（即 $P_i=\mathbf n_i\mathbf n_i^\top$，把残差投到法向）。这最小化 $\mathbf T\mathbf a_i$ 到由 $\mathbf b_i$ 及其法向定义平面的距离。因 $P_i$ 是正交投影：$P_i=P_i^2=P_i^\top$，故
$$\|P_i\cdot d_i\|^2=(P_i d_i)^\top(P_i d_i)=d_i^\top P_i^\top P_i d_i=d_i^\top P_i d_i.$$
代回 (4)：
$$\mathbf T=\arg\min_{\mathbf T}\Big\{\sum_i d_i^\top P_i\,d_i\Big\}.\tag{5}$$
对比 (2)，可见点-面是 GICP 取
$$C_i^B=P_i^{-1},\qquad C_i^A=\mathbf 0\tag{6,7}$$
的**极限情形**。严格说 $P_i$ 秩亏（不可逆，投影矩阵秩 1），但用可逆 $Q_i$ 逼近 $P_i$ 时，GICP 随 $Q_i\to P_i$ 趋于点-面。**直观**：$\mathbf b_i$ 被约束在沿平面法向方向（法向上方差极小），而其在平面内的位置完全未知（平面内方差无穷大）。

## §5.E plane-to-plane：两云都用表面结构（源 §III-B 逐字，含协方差构造）

**动机**（源逐字）：为提性能并增强模型对称性，把**第二个云（A）的局部表面信息**也纳入 (7)。直接把 $C_i^A$ 设成秩亏投影在数学上不可行（矩阵奇异），故用点-面的直觉构造一个**良态概率模型**。

**核心洞察**（源逐字）：点云不是 3 维空间中任意点集，而是被测距传感器采样的**曲面（2-manifold）**。真实曲面至少分片可微 → 数据**局部平面**。又因从两个不同视角采样，**对应永远不精确**（不会采到同一点）；每个测量点本质上只沿其**表面法向**提供约束。

**建模**：每个采样点沿其局部平面方向高方差、沿法向方向极低方差。若某点法向是 $\mathbf e_1$（第一基向量），其协方差为
$$\begin{bmatrix}\epsilon&0&0\\0&1&0\\0&0&1\end{bmatrix},\tag{$\star$}$$
$\epsilon$ 是表示沿法向方差的**小常数**（"a small constant representing covariance along the normal"）。这对应：沿法向位置**高置信**（方差 $\epsilon\ll1$），平面内位置**不确定**（方差 1）。$\mathbf a_i,\mathbf b_i$ 都按此建模。

**显式协方差（源逐字，式见下）**：给定 $\mathbf b_i,\mathbf a_i$ 处的法向 $\boldsymbol\mu_i,\boldsymbol\nu_i$，把 $(\star)$ 旋转使 $\epsilon$ 项对齐表面法向。设 $\mathbf R_{\mathbf x}$ 为把基向量 $\mathbf e_1\mapsto\mathbf x$ 的某个旋转，则
$$\boxed{\ C_i^B=\mathbf R_{\boldsymbol\mu_i}\begin{bmatrix}\epsilon&0&0\\0&1&0\\0&0&1\end{bmatrix}\mathbf R_{\boldsymbol\mu_i}^\top,\qquad C_i^A=\mathbf R_{\boldsymbol\nu_i}\begin{bmatrix}\epsilon&0&0\\0&1&0\\0&0&1\end{bmatrix}\mathbf R_{\boldsymbol\nu_i}^\top.\ }\tag{plane-to-plane}$$
变换 $\mathbf T$ 再由 (2) 求出。

**法向估计（源逐字脚注）**：对每个点取其 **20 个最近邻**做 PCA——经验协方差 $\hat\Sigma=\mathbf U\mathbf D\mathbf U^\top$（特征分解），**最小特征值对应的特征向量 = 表面法向**。实现时直接用 $\mathbf U$ 代替旋转矩阵（等价于把 $\mathbf D$ 换成 $\mathrm{diag}(\epsilon,1,1)$ 得到"表面对齐"协方差）。此法同时用于点-面与 GICP 的法向。

**自动剔除错误对应（源逐字，Fig.1 极端例）**：若所有沿某垂直段的（绿）点都被错配到（红）云中**单个**点上——因表面朝向不一致，plane-to-plane **自动 discount** 这些匹配：该对应的总协方差 $C_i^B+\mathbf T C_i^A\mathbf T^\top$ 会近似**各向同性（isotropic）**，相对那些薄而锐的（正确对应的）协方差，对目标函数贡献极小。等价视角：每个对应是一个**软约束**；不一致匹配让红点沿 $x$ 轴自由移动、绿点沿 $y$ 轴自由移动 → 这些错误对应形成**很弱、无信息**的约束。这是 GICP 对 $d_{\max}$ 不敏感、易调参的根因。

## §5.F 收敛、实现与参数（源 §IV 逐字）

- **仍在 ICP 框架内**：最小化用共轭梯度（原文为简化比较，三算法都用共轭梯度而非闭式）。性能以"已知偏移后收敛到正确解"衡量。
- **迭代上限**：标准 ICP 限 **250 次**；point-to-plane 与 GICP 限 **50 次**（通常更早收敛）。
- **$d_{\max}$ 的作用（源逐字）**：取小→降低收敛机会但增精度；取大→增大收敛半径但因更多错误对应降精度。GICP 通过**discount 错误对应**大幅降低取大 $d_{\max}$ 的代价 → 易在多种环境取得好性能而无需逐场景手调 $d_{\max}$。
- **测试设置**：仿真（光线追踪 SICK）+ 真实（车顶 Velodyne，郊区环路）。初始偏移 = 真值 + 均匀误差（$\pm1.5\,\mathrm m$、$\pm15^\circ$ 各轴）。各算法多随机初值平均定位误差。结论：GICP 对匹配阈值更鲁棒、总体精度更好；旋转误差在所有测试中可忽略。
- **可扩展性（源逐字）**：高斯核可与均匀分布混合建模外点；高斯 RV 可换成给残差留"松弛"的分布以显式建模非精确对应（但都无简单闭式，留作未来工作）。
- **复杂度**：因仍用 kd-tree 找对应，$O(n\log n)$ 显式点比较，可处理大点云（区别于需对每点比较每点的全概率法）。

> **与本书统一约定的转换提示**：(2) 的 $\mathbf T$ 直接作用于点；本书用右扰动 $\mathbf T\,\mathrm{Exp}(\delta\boldsymbol\xi)$ 做高斯-牛顿时，残差 $d_i=\mathbf b_i-\mathbf T\mathbf a_i$ 对 $\delta\boldsymbol\xi=[\delta\boldsymbol\rho;\delta\boldsymbol\phi]$ 的雅可比为 $\frac{\partial d_i}{\partial\delta\boldsymbol\xi}=-\mathbf T\big[\mathbf I\ \big|\ -\lfloor\mathbf a_i\rfloor_\times\big]$（平移块 $-\mathbf R$、旋转块 $\mathbf R\lfloor\mathbf a_i\rfloor_\times$，取决于扰动约定）；GICP 协方差 $C$ 即本书 $\boldsymbol\Sigma$，权矩阵 $(C_i^B+\mathbf TC_i^A\mathbf T^\top)^{-1}$ 即信息矩阵 $\boldsymbol\Omega_i$。

---

# 第六部分：正态分布变换（Normal Distributions Transform, NDT）完整推导

> 源：Biber & Straßer 2003（2D 原论文）；Magnusson 2009 博士论文（3D 推广 + 稳健化）。概念/参数校核见 Wikipedia NDT 词条与 PCL NDT 教程。SLAM Handbook 第8章**只给 NDT 概念、无任何数学**，故此节是该缺口来源。
> **记号提醒**：本节**位姿参数向量记 $\mathbf p$（粗体无下标）**、**点记 $\mathbf x_i$**，避免与点 $\mathbf p_i$ 冲突（NDT 原论文用 $\mathbf p$ 表位姿参数）。

## §6.A NDT 的核心思想（源 Biber 2003）

ICP 把 target 表示为**离散点集**，每次迭代要重新找最近点（离散对应，代价曲面不光滑、有大量局部极小）。NDT 反其道：把 target 点云转成**分片连续可微的概率密度**——
1. 把 target 所在空间划分为规则**网格/体素 (cells)**（2D 为正方形格、3D 为立方体素）。
2. 每个 cell 内拟合一个**正态分布**，存其**均值** $\mathbf q$ 与**协方差** $\boldsymbol\Sigma$。
3. NDT 把"扫描匹配"变成"最大化 source 点落在此密度上的似然"——**无须显式最近点查找**（GICP/Handbook 称之"分布-分布/分布匹配"），代价曲面**光滑可微**，可用牛顿法。

**优缺点**（源 Wikipedia 逐字）："NDT is very fast and accurate ... suitable for large scale data, but it is also sensitive to initialisation, requiring a sufficiently accurate initial guess, and for this reason it is typically used in a coarse-to-fine alignment strategy."（很快很准、适合大规模；但对初值敏感，需较准初值，常用 coarse-to-fine。）

## §6.B 每个 cell 的正态分布（源逐字）

设某 cell 内含点 $\{\mathbf x_1,\dots,\mathbf x_n\}$（$n\ge$ 某阈值，如 $\ge5$，否则该 cell 不可靠、忽略）。**均值**与**协方差**：
$$\mathbf q=\frac1n\sum_{i=1}^{n}\mathbf x_i,\qquad \boldsymbol\Sigma=\frac1{n-1}\sum_{i=1}^{n}(\mathbf x_i-\mathbf q)(\mathbf x_i-\mathbf q)^\top.\tag{6.1}$$
（原论文用 $\frac1n$；很多实现用无偏 $\frac1{n-1}$。）则在该 cell 内"采到点 $\mathbf x$"的概率密度（未归一化高斯）：
$$p(\mathbf x)=\exp\!\left(-\frac{(\mathbf x-\mathbf q)^\top\boldsymbol\Sigma^{-1}(\mathbf x-\mathbf q)}{2}\right).\tag{6.2}$$
全空间的 NDT 即各 cell 这些分布的分片拼接（分片连续可微）。**协方差正则化**：若 $\boldsymbol\Sigma$ 近奇异（点共线/共面），把其最小特征值抬到最大特征值的某比例（如 $0.001\lambda_{\max}$）以保证可逆、避免数值爆炸。

## §6.C 配准的 score 函数（源逐字）

设位姿参数 $\mathbf p$（2D：$[t_x,t_y,\phi]^\top$；3D：6 维，欧拉角或李代数），点变换 $\mathbf x_i'=T(\mathbf p,\mathbf x_i)=\mathbf R(\mathbf p)\mathbf x_i+\mathbf t(\mathbf p)$。把 source 每个点 $\mathbf x_i$ 用当前 $\mathbf p$ 变换、落入某 target cell（均值 $\mathbf q_i$、协方差 $\boldsymbol\Sigma_i$），其似然为 (6.2)。**总得分**（要**最大化**）：
$$s(\mathbf p)=\sum_{i=1}^{N}\exp\!\left(-\frac{(\mathbf x_i'-\mathbf q_i)^\top\boldsymbol\Sigma_i^{-1}(\mathbf x_i'-\mathbf q_i)}{2}\right).\tag{6.3}$$
配准 = $\max_{\mathbf p}s(\mathbf p)$，或等价 $\min_{\mathbf p}\big(-s(\mathbf p)\big)$（Wikipedia 写 $\arg\min_{R,t}\{-\sum_i\mathrm{NDT}(f_{R,t}(\mathbf x_i))\}$）。

## §6.D 稳健化的高斯混合（源 Magnusson 2009，d1/d2）

纯高斯 (6.2) 的尾部衰减太快、对外点不鲁棒。Magnusson 用**高斯 + 均匀分布的混合**逼近，得到分片可微且尾部更胖的得分：用形如
$$\tilde p(\mathbf x)=-d_1\exp\!\left(-\frac{d_2}{2}(\mathbf x-\mathbf q)^\top\boldsymbol\Sigma^{-1}(\mathbf x-\mathbf q)\right)\tag{6.4}$$
其中 $d_1,d_2$ 是把"高斯+均匀混合"拟合成单一指数形式得到的常数（由 cell 内离群概率 $c$、外点率等定标；典型推导令 $\tilde p$ 在 $0$ 与 $\infty$ 处匹配混合分布的值与曲率，解出 $d_1,d_2$）。**单点稳健 score**记 $s_i=\tilde p(\mathbf x_i')$，总得分 $s(\mathbf p)=\sum_i s_i$（仍最小化 $-s$，但因 (6.4) 已带负号，实际最小化 $\sum_i d_1\exp(\cdots)$ 的负值；不同文献符号略异，核心是指数型 score）。

## §6.E 牛顿法优化：梯度与海森（源 Magnusson 2009；[抽取补全推导] 逐步链式求导）

对 $\min_{\mathbf p}\big(-s(\mathbf p)\big)$ 用**牛顿法**：每步解
$$\mathbf H\,\Delta\mathbf p=-\mathbf g,\qquad \mathbf p\leftarrow\mathbf p+\Delta\mathbf p,\tag{6.5}$$
$\mathbf g=\nabla_{\mathbf p}(-s)$ 是梯度、$\mathbf H=\nabla^2_{\mathbf p}(-s)$ 是海森。下面对单个点（略去下标 $i$）求导，记
$$\mathbf x'=T(\mathbf p,\mathbf x),\quad \mathbf e=\mathbf x'-\mathbf q,\quad s_{\text{pt}}=-d_1\exp\!\Big(-\frac{d_2}{2}\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf e\Big).$$
设位姿参数维数 $m$（2D $m=3$、3D $m=6$），雅可比 $\mathbf J=\dfrac{\partial\mathbf x'}{\partial\mathbf p}\in\mathbb R^{3\times m}$（第 $k$ 列 $\mathbf J_k=\partial\mathbf x'/\partial p_k$）。

**梯度第 $k$ 元（[抽取补全推导]，链式法则）**：令标量 $u=-\tfrac{d_2}{2}\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf e$，则 $s_{\text{pt}}=-d_1 e^{u}$。
$$\frac{\partial s_{\text{pt}}}{\partial p_k}=-d_1 e^{u}\frac{\partial u}{\partial p_k},\qquad
\frac{\partial u}{\partial p_k}=-\frac{d_2}{2}\cdot 2\,\mathbf e^\top\boldsymbol\Sigma^{-1}\frac{\partial\mathbf e}{\partial p_k}=-d_2\,\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_k,$$
（用 $\partial\mathbf e/\partial p_k=\partial\mathbf x'/\partial p_k=\mathbf J_k$）。故 Magnusson 论文式（单点对 score $s_{\text{pt}}$ 的梯度，取最大化 $s$ 时常写正号）：
$$\boxed{\ g_k=\frac{\partial s_{\text{pt}}}{\partial p_k}=d_1 d_2\,\big(\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_k\big)\exp\!\Big(-\frac{d_2}{2}\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf e\Big).\ }\tag{6.6}$$
（符号取决于对 $s$ 最大化还是对 $-s$ 最小化；本式给 $\partial s_{\text{pt}}/\partial p_k$ 在 $s_{\text{pt}}=-d_1e^u$ 约定下 $=d_1d_2(\cdots)e^u$。）

**海森第 $(k,l)$ 元（[抽取补全推导]，对 (6.6) 再求 $\partial/\partial p_l$）**：
$$
\frac{\partial^2 s_{\text{pt}}}{\partial p_k\partial p_l}
= d_1 d_2\,\exp(u)\Big[\,
\underbrace{-d_2\big(\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_k\big)\big(\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_l\big)}_{\text{指数因子求导}}
+\underbrace{\mathbf J_l^\top\boldsymbol\Sigma^{-1}\mathbf J_k}_{\partial(\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_k)/\partial p_l\ \text{第一项}}
+\underbrace{\mathbf e^\top\boldsymbol\Sigma^{-1}\frac{\partial^2\mathbf x'}{\partial p_k\partial p_l}}_{\text{二阶导项}}
\Big].\tag{6.7}
$$
推导细节：对 (6.6) 方括号内 $\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_k$ 与 $\exp(u)$ 同时含 $p_l$，乘积法则得三项——(i) $\exp(u)$ 对 $p_l$ 导出 $\frac{\partial u}{\partial p_l}=-d_2\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_l$ 乘原式 → 第一项；(ii) $\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf J_k$ 对 $p_l$ 导，$\mathbf e$ 部分 $\partial\mathbf e/\partial p_l=\mathbf J_l$ 给 $\mathbf J_l^\top\boldsymbol\Sigma^{-1}\mathbf J_k$；(iii) $\mathbf J_k=\partial\mathbf x'/\partial p_k$ 对 $p_l$ 导给二阶导 $\partial^2\mathbf x'/\partial p_k\partial p_l$（旋转部分非零，平移部分为零）。

把所有点的 $g_k,H_{kl}$ 累加得总 $\mathbf g,\mathbf H$。若 $\mathbf H$ 非正定（远离极小时），加阻尼 $\mathbf H+\lambda\mathbf I$（即 Levenberg-Marquardt 化），再解 (6.5)。**关键性质（源 Magnusson 逐字转述）**：梯度与海森的表达式**与配准维度（2D/3D）无关、与变换参数化方式无关**——换李代数 $\boldsymbol\xi$ 参数化只需换 $\mathbf J=\partial\mathbf x'/\partial\boldsymbol\xi$（右扰动雅可比 $\mathbf J=[\mathbf R\ |\ -\mathbf R\lfloor\mathbf x\rfloor_\times]$ 之类）与二阶导项，(6.6)(6.7) 形式不变。

## §6.F NDT 算法与典型参数（源 Biber/Magnusson + PCL 教程）

**3D-NDT 配准算法（整理）**：
```
输入：target 点云、source 点云、初始位姿 p0、cell 尺寸 r
1. 用 cell 尺寸 r 把 target 空间分格；对每个含点数≥阈值的 cell，
   按 (6.1) 算并存均值 q、协方差 Σ（并正则化 Σ）。
2. p ← p0
3. repeat:
4.    g ← 0 ; H ← 0
5.    for 每个 source 点 x:
6.        x' ← T(p, x)               # 用当前 p 变换
7.        定位 x' 所在 target cell，取其 (q, Σ)
8.        累加该点对 g (6.6)、H (6.7)
9.    解 H Δp = -g                    # 牛顿步；H 非正定则加阻尼
10.   用 line search（如 More-Thuente）定步长，p ← p + 步长·Δp
11. until |Δp| < ε 或 达最大迭代
12. 输出 p
```
**PCL 默认参数（源 PCL 教程逐字）**：Transformation Epsilon（位姿增量收敛阈）$=0.01$（m/rad）；Step Size（More-Thuente 线搜最大步长）$=0.1$；Resolution（体素网格分辨率）$=1.0\,\mathrm m$；Maximum Iterations $=35$。Fitness（输出云到 target 最近点的平方距离和，欧氏 fitness score）用于评估配准质量。

> **与 ICP/GICP 对比**：NDT 不需每步找最近点（建格一次即可），对初值敏感但代价曲面更光滑；GICP §II 评 NDT "也配准原始点但稳定性低于 ICP、某些场景发散"（见 `lidar_slam__fastlio.md` 引述）。三者各有适用：ICP 通用、GICP 结构化最稳、NDT 大规模快。

---

# 第七部分：LOAM 系点云特征（边/面）提取与匹配（点云处理视角）

> 源：J. Zhang & S. Singh, *"LOAM"*, RSS 2014（PDF: roboticsproceedings.org/rss10/p07.pdf）。
> **本节定位**：`lidar_slam__loam_family.md` 已从 SLAM 里程计/建图角度完整抽取 LOAM（含 Algorithm 1、L-M 求解、IMU 辅助、KITTI 实验）。**本节只从"点云特征提取与匹配"这门技术本身**重述其**精确曲率定义式、边/面点选取规则、点-线/点-面残差几何**——这是【点云处理】章"特征提取(LOAM)"小节的核心，且 Handbook 第8章对此只有定性描述。系统层细节请并读 loam_family 件。

## §7.A 局部光滑度 / 曲率（curvature）的精确定义（源 LOAM §V-A，式 (1)）

LiDAR 单帧点云不均匀（一条 scan 内角分辨率高、跨 scan 间分辨率低），故特征点**仅用单条 scan 内信息**提取。设 $i\in\mathcal P_k$ 为点，$\mathcal S$ 是激光在**同一 scan** 内 $i$ 两侧的连续点集（CW/CCW 顺序，两侧各半）。定义点 $i$ 的**局部曲面光滑度（曲率）**：
$$c=\frac{1}{|\mathcal S|\cdot\big\|\mathbf X^L_{(k,i)}\big\|}\left\|\sum_{j\in\mathcal S,\,j\neq i}\Big(\mathbf X^L_{(k,i)}-\mathbf X^L_{(k,j)}\Big)\right\|.\tag{LOAM-1}$$
其中 $\mathbf X^L_{(k,i)}$ 是点 $i$ 在 LiDAR 系 $\{L\}$ 中坐标，$|\mathcal S|$ 是邻居数，分母含 $\|\mathbf X^L_{(k,i)}\|$ 做距离归一化（远点位置噪声大，归一后曲率可比）。

**几何含义（[抽取补全]）**：$\sum_{j}(\mathbf X_i-\mathbf X_j)$ 是 $i$ 与左右邻居的差向量之和——在**平直区**左右邻居对称、差向量相消，$c$ 小；在**尖锐边/角**处不对称、差不相消，$c$ 大。这正是离散二阶差分（曲率）的度量。

**边/面点判定**：scan 内点按 $c$ 排序——
- **$c$ 最大** ⇒ **边缘点 (edge points)** $\mathcal E$（角、尖锐边）；
- **$c$ 最小** ⇒ **平面点 (planar points)** $\mathcal H$（平坦面）。
用阈值 $c_{th}$ 区分：$c>c_{th}$ 为边、$c<c_{th}$ 为面。

## §7.B 特征均匀分布与坏特征剔除（源 LOAM §V-A）

**均匀分布**：把一条 scan 分成**四个相同子区域**；每个子区域最多 **2 个边缘点、4 个平面点**。点 $i$ 入选当且仅当其 $c$ 超/低于阈值、且该类已选数未达上限。

**剔除不可靠特征**（两类，源逐字）：
1. **近平行于激光束的局部平面上的点**（Fig.4(a) 点 B）：激光束与表面近平行时返回不可靠 → 不选。判据：邻居集 $\mathcal S$ 构成的曲面片与激光束近平行则弃。
2. **遮挡边界点**（Fig.4(b) 点 A）：点 A 在边缘上，因其相连曲面被另一物体挡住；换视角时遮挡区可能变可见 → 这类"伪边"不可靠。判据：$\mathcal S$ 中有点在激光束方向上因间隙与 $i$ 断开、且比 $i$ 更近 LiDAR（如点 B）则弃。

**总规则**：边从最大 $c$ 选、面从最小 $c$ 选；入选需满足 (i) 子区域未超上限、(ii) 周围点尚未被选、(iii) 不在近平行曲面片上、(iv) 不在遮挡边界上。

## §7.C 特征对应：点-线与点-面（源 LOAM §V-B，式 (2)(3)）

把上一 sweep 重投影点云 $\bar{\mathcal P}_k$ 存入 **3D KD-tree**（见本件 §2）。对当前重投影特征点逐点找最近邻。

**边缘线对应（点-线）**：设 $i\in\tilde{\mathcal E}_{k+1}$。边缘线由两点表示：$j$ = $i$ 在 $\bar{\mathcal P}_k$ 的最近邻，$l$ = $i$ 在 $j$ 所在 scan 的**相邻两条 scan** 中的最近邻（要求 $j,l$ 来自**不同 scan**——单 scan 不能含同一边缘线多于一点）。基于 (LOAM-1) 验证 $j,l$ 是边缘点。**点到线距离**：
$$d_{\mathcal E}=\frac{\big\|(\tilde{\mathbf X}^L_{(k+1,i)}-\bar{\mathbf X}^L_{(k,j)})\times(\tilde{\mathbf X}^L_{(k+1,i)}-\bar{\mathbf X}^L_{(k,l)})\big\|}{\big\|\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,l)}\big\|}.\tag{LOAM-2}$$
**几何含义**：分子 = 以 $i,j,l$ 为顶点的平行四边形面积（= 底 $\|j-l\|$ × 高）；除以底 $\|j-l\|$ 得点 $i$ 到过 $j,l$ 直线的**垂距**。

**平面片对应（点-面）**：设 $i\in\tilde{\mathcal H}_{k+1}$。平面由三点表示：$j$ = 最近邻，$l$ = $j$ **同 scan** 另一点，$m$ = $j$ 相邻 scan 一点（保证三点不共线）。验证 $j,l,m$ 是平面点。**点到面距离**：
$$d_{\mathcal H}=\frac{\Big|(\tilde{\mathbf X}^L_{(k+1,i)}-\bar{\mathbf X}^L_{(k,j)})\cdot\big((\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,l)})\times(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,m)})\big)\Big|}{\big\|(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,l)})\times(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,m)})\big\|}.\tag{LOAM-3}$$
**几何含义**：分母 = $\triangle jlm$ 法向量模（= 平行四边形面积）；分子 = $(i-j)$ 在该法向上的投影 × 面积；商 = 点 $i$ 到过 $j,l,m$ 平面的**垂距**。

> **注**：(LOAM-2)(LOAM-3) 是 LOAM 的"点-线/点-面"残差——与 §4.D 的点-面 ICP 残差同源（都是点到几何基元的距离），但 LOAM 把"边"也作基元（点-线），且对应基元用 KD-tree 近邻 + scan 结构动态构造而非固定法向。

## §7.D 运动估计与 L-M 求解（点云处理视角，源 LOAM §V-C，式 (9)–(12)）

把每个边/面特征的距离 (LOAM-2)/(LOAM-3) 经运动插值（sweep 内匀速假设，式 (LOAM-4)–(LOAM-8)，详见 loam_family 件）写成关于 6-DOF 位姿 $\mathbf T^L_{k+1}$ 的非线性函数 $f_{\mathcal E}=d_{\mathcal E}$、$f_{\mathcal H}=d_{\mathcal H}$，对所有特征堆叠成
$$\mathbf f(\mathbf T^L_{k+1})=\mathbf d.\tag{LOAM-11}$$
**Levenberg-Marquardt 更新**（让 $\mathbf d\to\mathbf0$）：
$$\mathbf T^L_{k+1}\leftarrow\mathbf T^L_{k+1}-\big(\mathbf J^\top\mathbf J+\lambda\,\mathrm{diag}(\mathbf J^\top\mathbf J)\big)^{-1}\mathbf J^\top\mathbf d,\qquad \mathbf J=\frac{\partial\mathbf f}{\partial\mathbf T^L_{k+1}}.\tag{LOAM-12}$$
配 **bisquare 鲁棒权**（距对应越远权越小、超阈值赋零，见 §4.E）。完整 Algorithm 1、mapping（10× 特征 + 协方差特征值判边/面 + 5 cm 体素降采样）、IMU 辅助、KITTI #1（平均位置误差 **0.88%**）等系统层内容见 `lidar_slam__loam_family.md` §LOAM.V-VII。

## §7.E mapping 中用 PCA 特征值判别边/面（源 LOAM §VI）

scan-to-map 时，在特征点邻域取 target 地图点集 $\mathcal S'$，算其**协方差矩阵 $\mathbf M$** 的特征值 $\mathbf V$ 与特征向量 $\mathbf E$：
- $\mathcal S'$ 在**边缘线**上 ⇒ $\mathbf V$ 含**一个**显著大特征值，对应特征向量 = 边缘线方向；
- $\mathcal S'$ 在**平面片**上 ⇒ $\mathbf V$ 含**两个**大特征值、第三个显著小，最小特征值对应特征向量 = 平面法向。
基元位置取 $\mathcal S'$ 几何中心。这与 §5.E（GICP 用 PCA 取最小特征值方向作法向）、§1.2（surfel）一脉相承——**PCA 协方差特征值/向量是点云局部几何（线/面/法向）的统一刻画工具**。

---

# 第八部分：scan-to-scan 与 scan-to-map 配准范式（汇总）

> 把前面所有算法放进"配准范式"框架收尾。来源综合 LOAM/Handbook §8.2.2.4 / FAST-LIO2。

## §8.1 两种配准范式

- **scan-to-scan**：把当前帧配准到**上一帧**（或上一去畸变帧）。优点：实时、轻；缺点：误差逐帧累积、漂移快（LOAM 的 odometry 模块即此，~10 Hz）。
- **scan-to-map（scan-to-model）**：把当前帧配准到围绕传感器维护的**局部地图**（由历史多帧融合，远比单帧稠密）。优点：内点关联多得多、漂移低；代价：需可增量更新的点云数据结构（KD-tree/ikd-Tree/体素地图）来存与查地图。LOAM 的 mapping（~1 Hz）、LIO-SAM、FAST-LIO2 皆此（FAST-LIO2 更"直接把原始点配准到地图"免特征，见 fastlio 件）。
- **双阶段 vs 单阶段**（源 Handbook §8.2.2.4）：早期因算力交错高频 scan-to-scan + 低频 scan-to-map；现代用**单阶段 scan-to-map**（体素化局部地图 + 直接逐点对应）。

## §8.2 配准算法选型小结

| 算法 | target 表示 | 对应方式 | 度量 | 闭式解 | 强项 / 弱项 |
|---|---|---|---|---|---|
| 点-点 ICP | 离散点 + KD-tree | 最近点 | 点-点欧氏 | **有**(SVD/四元数) | 通用；初值敏感、离散化误差、慢收敛 |
| 点-面 ICP | 点 + 法向 | 最近点 | 点到切平面 | 无（线性化迭代） | 结构化环境优；需法向；不罚切向滑动 |
| GICP | 点 + 局部协方差 | 最近点 | 马氏(协方差加权) | 无（共轭梯度/GN） | 最稳、对 $d_{\max}$ 不敏感；算协方差略贵 |
| NDT | 体素正态分布 | **无须最近点**(落格) | 分布似然 | 无（牛顿法） | 大规模快、代价曲面光滑；对初值敏感、偶发散 |
| LOAM 特征法 | 边/面特征 + KD-tree | 近邻构边线/面片 | 点-线 + 点-面 | 无（L-M） | 实时、特征少；依赖特征质量、退化环境弱 |

**统一视角**：除 NDT 外都是"找对应 + 最小化几何残差"的迭代；NDT 用分布把对应隐式化。所有几何残差最小化都可纳入本书右扰动高斯-牛顿/L-M 框架，区别只在**残差定义（点/线/面/分布）与权矩阵（单位/投影/协方差逆）**。

---

# 第九部分：与本书统一约定的转换要点（综合 agent 务必照办）

1. **旋转/位姿**：各源（ICP/GICP/NDT/LOAM）多把 $\mathbf R,\mathbf t$ 或四元数当作直接作用在点上的变换、用四元数或欧拉角参数化。本书统一 $\mathbf R\in\mathrm{SO}(3)$、$\mathbf T\in\mathrm{SE}(3)$、Hamilton 四元数、**右扰动** $\mathbf T\,\mathrm{Exp}(\delta\boldsymbol\xi)$、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$。移植迭代解时，把"$\mathbf R\approx(\mathbf I+\lfloor\boldsymbol\omega\rfloor_\times)\mathbf R_0$"换成右扰动 $\mathbf R_0\mathrm{Exp}(\delta\boldsymbol\phi)$，雅可比相应换号（§4.D/§5.F 已给提示）。
2. **点-点闭式解的行列式修正**：SVD 路必须做 (4.12)；四元数路天然免修正。本书讲解建议四元数与 SVD 并陈，强调二者等价。
3. **协方差字母**：GICP 的 $C$、NDT 的 $\boldsymbol\Sigma$ 统一为本书 $\boldsymbol\Sigma$；信息矩阵 $\boldsymbol\Sigma^{-1}=\boldsymbol\Omega$。注意 NDT 节"位姿参数 $\mathbf p$"与"点 $\mathbf p_i$"已被本抽取改记（位姿 $\mathbf p$、点 $\mathbf x_i$），成书时与本书点记号 $\mathbf p_i$ 统一须再核。
4. **LOAM 坐标系**：LOAM 的 $\{L\}$ 是"左-上-前"非常规轴向，6-DOF 排序"平移在前、旋转(轴角)在后"，与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前一致但旋转分量是轴角而非 $\mathfrak{se}(3)$ 的 $\boldsymbol\phi$（详见 `lidar_slam__loam_family.md` §0.1）。
5. **去重**：LOAM 特征(§7)与 `lidar_slam__loam_family.md` 必然重叠；ICP/GICP/NDT/KD-tree(§2–6)是本件**独有**（Handbook 与 loam_family 件均无完整推导）。综合 LOAM 时取一份并互补，综合 ICP/GICP/NDT/KD-tree 时以本件为主源。

---

## 附：本抽取覆盖范围与未尽部分（completeness）

**已全量覆盖**（含完整公式/推导/伪码/参数）：
- 点云表示与数据结构（§1：定义、8 种表示对照表、传感器先验）。
- KD-tree（§2：构建伪码、NN 回溯搜索伪码 + 剪枝判据几何推导、复杂度、增量/近似指引）。
- 滤波/降采样（§3：体素栅格质心/中心、SOR 高斯判据、半径滤波、直通/地面）。
- ICP（§4：Alg.1 逐行、closest-point 算子、**点-点 SVD 闭式解完整 trace 最大化证明 + 行列式修正**、**四元数/Davenport-K 闭式解**、**单调收敛定理两步夹逼证明**、**点-面线性化法方程 + Hessian 块**、鲁棒化）。
- GICP（§5：**概率生成模型 → MLE → 核心代价式 (2) 完整推导**、退化到点-点/点-面、**plane-to-plane 协方差构造 + PCA 法向 + 自动 discount 机制**、参数与收敛讨论）。**这是同套抽取中 GICP 的唯一来源（Handbook 连名字都无）。**
- NDT（§6：cell 均值/协方差、密度、score、**d1/d2 稳健化**、**牛顿法梯度 (6.6)/海森 (6.7) 完整链式求导**、算法、PCL 参数）。**这是同套抽取中 NDT 数学的唯一来源。**
- LOAM 特征（§7：**曲率定义式 (LOAM-1) + 几何含义**、边/面判定与坏特征剔除、**点-线 (LOAM-2)/点-面 (LOAM-3) 距离 + 几何含义**、L-M 求解、PCA 判别）。
- 配准范式（§8：scan-to-scan vs scan-to-map、选型对照表）+ 记号转换（§9）。

**未尽 / 须他源补全**：
1. **NDT 的 d1/d2 闭式定标公式与二阶导 $\partial^2\mathbf x'/\partial p_k\partial p_l$ 的逐元表达**：本件给出其在牛顿步中的位置与链式推导框架，但 Magnusson 2009 用欧拉角参数化的二阶导逐元长表未逐一抄录（联网未取到该 PDF 全文，证书错误）——若成书需欧拉角逐元式，建议直取 Magnusson 2009 论文 §6.2；本书若改用李代数右扰动参数化则 $\mathbf J,\partial^2\mathbf x'$ 另推（更简洁），形式 (6.5)(6.6)(6.7) 不变。
2. **Besl-McKay 原论文的"加速 ICP"（外推/抛物线加速注册向量）**：本件证了基础单调收敛，未抄其加速启发式（论文 §中 quadratic/linear extrapolation 段）。
3. **ICP 配准结果的协方差估计**（Censi/Bengtsson 等的 Hessian 近似 $\boldsymbol\Sigma\approx(\mathbf J^\top\mathbf J)^{-1}\sigma^2$）：Handbook 与本批源均未含完整推导，若成书需"配准不确定度"小节须另觅源。
4. **增量 KD-tree (ikd-Tree) 完整伪码与 $\alpha$-平衡判据**：本件 §2.4 仅指路，完整内容在 `lidar_slam__fastlio.md` §12（同目录，已抽取）。
5. **LOAM 系统层（Algorithm 1 全、mapping、LeGO-LOAM 分割与两步 L-M、LIO-SAM/FAST-LIO）**：见 `lidar_slam__loam_family.md` 与 `lidar_slam__fastlio.md`，本件不重复。
