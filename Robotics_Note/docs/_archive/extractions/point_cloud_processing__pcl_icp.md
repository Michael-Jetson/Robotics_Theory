# 抽取留痕 — 点云处理：ICP / point-to-plane ICP / GICP / NDT 配准完整推导，KD-tree 最近邻，体素降采样，点云滤波

> **本抽取服务章节**：`点云处理`（点云表示与数据结构；滤波/降采样；最近邻 KD-tree；ICP 系列 point-to-point / point-to-plane / GICP 完整推导与收敛性；NDT；特征提取 LOAM）。
>
> **来源（联网研究，权威原始文献 + 官方文档）**：
> 1. **ICP 原始论文**：P. J. Besl and N. D. McKay, *"A Method for Registration of 3-D Shapes"*, IEEE Trans. Pattern Analysis and Machine Intelligence (PAMI), vol. 14, no. 2, pp. 239–256, 1992. DOI 10.1109/34.121791. <https://ieeexplore.ieee.org/document/121791>（含 ICP 收敛定理、四元数闭式解）。
> 2. **point-to-plane ICP 原始思想**：Y. Chen and G. Medioni, *"Object modelling by registration of multiple range images"*, Image and Vision Computing, 10(3):145–155, 1992（point-to-plane 误差度量的提出）。
> 3. **point-to-plane 线性最小二乘**：Kok-Lim Low, *"Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration"*, Technical Report TR04-004, Dept. CS, UNC Chapel Hill, Feb. 2004. PDF: <https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf>（小角度线性化 → 6×6 线性系统的完整推导）。
> 4. **GICP（Generalized-ICP / plane-to-plane）**：A. Segal, D. Haehnel, S. Thrun, *"Generalized-ICP"*, Robotics: Science and Systems (RSS) V, 2009. DOI 10.15607/RSS.2009.V.021. PDF: <https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf>（概率配准框架，统一 point-to-point / point-to-plane / plane-to-plane）。
> 5. **NDT 原始论文（2D）**：P. Biber and W. Straßer, *"The Normal Distributions Transform: A New Approach to Laser Scan Matching"*, IEEE/RSJ IROS 2003, pp. 2743–2748. DOI 10.1109/IROS.2003.1249285。
> 6. **3D-NDT 博士论文（权威全推导）**：M. Magnusson, *"The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection"*, PhD thesis, Örebro University, 2009. 全文 PDF: <https://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf>（NDT score / 梯度 6.12 / Hessian 6.13 / Jacobian 6.18-6.19 / Hessian 项 6.20-6.21 / Newton 法 / 离群点混合 / P2D-NDT 与 D2D-NDT）。
> 7. **绝对定向闭式解（四元数）**：B. K. P. Horn, *"Closed-form solution of absolute orientation using unit quaternions"*, J. Opt. Soc. Am. A, 4(4):629–642, 1987. PDF: <https://people.csail.mit.edu/bkph/papers/Absolute_Orientation.pdf>（4×4 对称矩阵 N，最大特征值法）。
> 8. **ICP 变体分类与加速**：S. Rusinkiewicz and M. Levoy, *"Efficient Variants of the ICP Algorithm"*, 3DIM 2001, pp. 145–152. DOI 10.1109/IM.2001.924423. <https://gfx.cs.princeton.edu/pubs/Rusinkiewicz_2001_EVO/index.php>（六阶段分类法 + 加速变体）。
> 9. **官方文档（滤波/降采样/NDT 实现/GICP 实现/KD-tree）**：Point Cloud Library (PCL) 官方文档与源码：VoxelGrid 教程 <https://pointclouds.org/documentation/tutorials/voxel_grid.html>；StatisticalOutlierRemoval 教程 <https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html>；RadiusOutlierRemoval / ConditionalRemoval 教程 <https://pointclouds.org/documentation/tutorials/remove_outliers.html>；NDT 教程 <https://pointclouds.org/documentation/tutorials/normal_distributions_transform.html>；NDT 实现 ndt.hpp <https://pointclouds.org/documentation/ndt_8hpp_source.html>；GICP 实现 gicp.hpp <https://github.com/PointCloudLibrary/pcl/blob/master/registration/include/pcl/registration/impl/gicp.hpp>。
> 10. **OpenCV KinFu point-to-plane ICP**：<https://docs.opencv.org/4.x/d7/dbe/kinfu_icp.html>（point-to-plane 线性化 → G(p) 矩阵与 6×6 正规方程）。
> 11. **Kabsch / Wahba / KD-tree 数学背景**：Wikipedia 条目 *Kabsch algorithm*、*Wahba's problem*、*Point-set registration*、*k-d tree*、*Iterative closest point*、*Normal distributions transform*（开放教育资源，方法层面互相印证）。
> 12. **LOAM 特征（曲率/边-面分类/点到线-点到面距离）**：J. Zhang and S. Singh, *"LOAM: Lidar Odometry and Mapping in Real-time"*, RSS X, 2014（本书已在 `extractions/lidar_slam__loam_family.md` 完整抽取，本文仅作点云特征视角的自包含补述并交叉引用）。
>
> **保真承诺**：本文件按主题（=源材料的逻辑小节）逐条记录每一个定义、每一条公式（LaTeX 写全，保留原编号）、每一步推导（中间代数不跳）、每一道数值例/参数缺省值、每一段算法伪码。不做摘要、不做凝练。涉及版权 PDF 的内容，记录的是其中的**数学事实与方法步骤**（公式、算法、定理；这些不可版权化），并以多份权威来源交叉印证以保证转写正确。凡重建/转写的公式编号或细节，标 `\rebuilt` / `\pz` 待核对。

---

## 0. 记号约定（各源）+ 与本书统一约定的差异

本章涉及多篇论文 + PCL/OpenCV 文档，记号体系各不相同。先列**与本书统一约定的全局对照**，再逐源列细节。**配准类问题的记号陷阱极多**（谁是 source 谁是 target、协方差矩阵叉乘 $H$ 是 $\sum a b^T$ 还是 $\sum b a^T$、SVD 解是 $R=VU^T$ 还是 $UV^T$），本节务必厘清，综合时照此转换。

### 0.0 本书统一约定（编写规范 §五）

- 旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$（不用 $\mathbf C$）；位姿 $\mathbf T\in\mathrm{SE}(3)$。
- 四元数：Hamilton。
- **扰动以右扰动为主**：$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$，局部坐标，右雅可比 $\mathbf J_r$ 为主。
- $\mathfrak{se}(3)$ 排序 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）。
- 坐标系下标语义 $\mathbf T_{wc}$ = 相机/体到世界。
- 协方差用 $\boldsymbol\Sigma$，信息矩阵 $\boldsymbol\Sigma^{-1}$。
- 反对称（叉乘）算子 $\lfloor\mathbf a\rfloor_\times$（亦记 $\mathbf a^\wedge$、$[\mathbf a]_\times$、$\mathrm{skew}(\mathbf a)$），满足 $\lfloor\mathbf a\rfloor_\times\mathbf b=\mathbf a\times\mathbf b$。

### 0.1 配准问题的统一记号（本文采用）

为减少混乱，本文统一用如下记号叙述所有配准方法（综合时直接照搬）：

| 记号 | 含义 |
|---|---|
| **source / 待配准点云** $\mathcal A=\{\mathbf a_i\}$ | 被变换的"动"点云（亦称 model / scene / scan / input source）。本文一律记为 $\mathbf a_i$ 或 $\mathbf s_i$（source）。 |
| **target / 参考点云** $\mathcal B=\{\mathbf b_i\}$ | 保持固定的"静"点云（亦称 reference / data / map / target）。本文一律记为 $\mathbf b_i$ 或 $\mathbf q_i$（query 的近邻、或 target）。 |
| $\mathbf T=(\mathbf R,\mathbf t)$ | 待求刚体变换，$\mathbf R\in\mathrm{SO}(3)$、$\mathbf t\in\mathbb R^3$；满足 $\mathbf b_i\approx\mathbf R\mathbf a_i+\mathbf t$。 |
| $\mathbf n_i$ | target 上对应点处的单位法向量（point-to-plane / GICP 用）。 |
| $\mathbf C_i^A,\mathbf C_i^B$ | source 点 $\mathbf a_i$、target 点 $\mathbf b_i$ 的 $3\times3$ 协方差（GICP 用）。 |
| $\mathbf d_i$ | 残差，$\mathbf d_i=\mathbf b_i-(\mathbf R\mathbf a_i+\mathbf t)$。 |
| $N$（或 $n$） | 对应点对数。 |

> ⚠️ **方向陷阱（综合必读）**：不同库的 source/target 命名相反。**PCL**：`setInputSource`=待配准动点云，`setInputTarget`=固定参考，求的变换把 source 对齐到 target。**Besl-McKay 原文**：用 "data shape" $P$ 配准到 "model shape" $X$，data 是动的。**Horn**：用 "left" / "right" 两套坐标，求把 left 转到 right。本文统一：**source/动=$\mathbf a$/$\mathbf s$，target/静=$\mathbf b$/$\mathbf q$，变换 $\mathbf R\mathbf a+\mathbf t\to\mathbf b$**。

### 0.2 Besl-McKay ICP 记号（源 PAMI 1992）

| 记号 | 含义（Besl-McKay 用法） |
|---|---|
| $P=\{\mathbf p_i\}$，$N_p$ | **data shape**（待配准的"动"点云）及其点数。 |
| $X=\{\mathbf x_i\}$，$N_x$ | **model shape**（参考几何，可以是点集/线段/三角网/参数曲面）。 |
| $d(\mathbf p,X)$ | 点 $\mathbf p$ 到几何体 $X$ 的最小欧氏距离，$d(\mathbf p,X)=\min_{\mathbf x\in X}\|\mathbf x-\mathbf p\|$。 |
| $C$ | **closest point operator**：$\mathbf y_i=C(\mathbf p_i,X)$ 给出 $X$ 中离 $\mathbf p_i$ 最近的点。 |
| $\mathbf q=[q_0,q_1,q_2,q_3,q_4,q_5,q_6]^T$ | 配准状态向量 = 单位四元数 $\mathbf q_R=[q_0,q_1,q_2,q_3]^T$（$q_0\ge0$，$q_0^2+q_1^2+q_2^2+q_3^2=1$）+ 平移 $\mathbf q_T=[q_4,q_5,q_6]^T$。 |
| $f(\mathbf q)$ | 均方目标函数（mean square objective）。 |
| $d_k$ | 第 $k$ 次迭代的均方误差值。 |
| $\tau$ | 收敛阈值。 |

**Besl-McKay 与本书的差异**：(1) Besl 用四元数闭式解旋转，与本书 Hamilton 四元数一致（Besl 的 $q_0$ 为实部，$q_0\ge0$ 选主值，符合 Hamilton 约定）。(2) Besl 的"data→model"方向 = 本书"source→target"。(3) Besl 是 batch 优化每次重算闭式解，无左右扰动之分。

### 0.3 Low / OpenCV point-to-plane 记号（源 TR04-004 / OpenCV KinFu）

| 记号 | 含义 |
|---|---|
| $\mathbf s_i=(s_{ix},s_{iy},s_{iz},1)^T$ | source 点（齐次坐标）。 |
| $\mathbf d_i=(d_{ix},d_{iy},d_{iz},1)^T$ | 对应 destination 点（target，齐次坐标）。 |
| $\mathbf n_i=(n_{ix},n_{iy},n_{iz},0)^T$ | $\mathbf d_i$ 处的单位法向。 |
| $\mathbf M$ | $4\times4$ 刚体变换（旋转 $\times$ 平移）。 |
| $\alpha,\beta,\gamma$ | 绕 $x,y,z$ 轴的小角度旋转。 |
| $t_x,t_y,t_z$ | 平移分量。 |
| $\mathbf x=(\alpha,\beta,\gamma,t_x,t_y,t_z)^T$ | 6-DOF 待求未知量。 |

**Low 与本书差异**：Low 用 $(\alpha,\beta,\gamma)$ 欧拉小角，本书用 $\mathfrak{so}(3)$ 指数坐标 $\delta\boldsymbol\phi$——小角度下二者一阶等价（$\mathbf R\approx\mathbf I+\lfloor\delta\boldsymbol\phi\rfloor_\times$）。Low 未知量排序"旋转在前、平移在后"，与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）相反，移植时重排。

### 0.4 GICP 记号（源 RSS 2009）

| 记号 | 含义 |
|---|---|
| $A=\{\mathbf a_i\}$，$B=\{\mathbf b_i\}$ | 两帧点云（$\mathbf a_i$ 与 $\mathbf b_i$ 为已建立的对应点对）。 |
| $\mathbf T$ | 待求变换（GICP 中作用在 $A$ 上：$\mathbf T\mathbf a_i$）。 |
| $\hat{\mathbf a}_i,\hat{\mathbf b}_i$ | 点的（未知）真值位置；$\mathbf a_i\sim\mathcal N(\hat{\mathbf a}_i,\mathbf C_i^A)$。 |
| $\mathbf C_i^A,\mathbf C_i^B$ | 各点的 $3\times3$ 协方差矩阵（由局部邻域估计）。 |
| $\mathbf d_i^{(\mathbf T)}$ | 在变换 $\mathbf T$ 下的残差，$\mathbf d_i^{(\mathbf T)}=\mathbf b_i-\mathbf T\mathbf a_i$。 |
| $d_{\max}$ | 最大匹配距离（剔除错误对应）。 |
| $\epsilon$ | 沿法向的小协方差（plane-to-plane 构造），缺省 $\epsilon=0.001$。 |

**GICP 与本书差异**：GICP 把变换作用在 source $A$ 上（$\mathbf T\mathbf a_i$），与本文统一约定一致。GICP 协方差用 $\mathbf C$，本书用 $\boldsymbol\Sigma$；移植时 $\mathbf C\to\boldsymbol\Sigma$。GICP 用 BFGS 在 $(\mathbf R,\mathbf t)$（经欧拉角参数化）上优化，本书可改右扰动 + Gauss-Newton/LM。

### 0.5 NDT 记号（源 Biber 2003 / Magnusson 2009）

| 记号 | 含义 |
|---|---|
| $\mathbf x_k$ | 一个 NDT cell（voxel）内的第 $k$ 个 target 点。 |
| $\mathbf q$（或 $\boldsymbol\mu$） | cell 内点的均值（mean）。 |
| $\boldsymbol\Sigma$（Biber 记 $\mathbf S$） | cell 内点的协方差。 |
| $p$（参数向量，Magnusson 记 $\vec p$） | 变换参数：2D 为 $(t_x,t_y,\phi)$；3D 为 $(t_x,t_y,t_z,\phi_x,\phi_y,\phi_z)$（平移 + 欧拉角，3D 用 Euler ZYX）。 |
| $\mathbf x_k'=T(\vec p,\mathbf x_k)$ | source 点 $\mathbf x_k$ 经 $\vec p$ 变换后的位置。 |
| $s(\vec p)$ | 配准 score 函数（待优化）。 |
| $d_1,d_2,d_3$ | score 的离群点混合常数（由 outlier ratio 与 cell 尺寸推出）。 |
| $\mathbf g$，$\mathbf H$ | score 的梯度、Hessian（Newton 法）。 |
| $\mathbf J_E,\mathbf H_E$ | 变换雅可比、变换 Hessian（点对参数的导数）。 |

**NDT 与本书差异**：(1) Biber/Magnusson 的 score 是**负对数似然取负后最大化**（Magnusson 实际写成最小化 $-\sum\tilde p$），符号约定须留意（详见 §5.4）。(2) Magnusson 用 Euler 角参数化旋转，本书右扰动 $\mathfrak{so}(3)$——移植时雅可比 $\mathbf J_E$ 重新推（本书法向版更简洁）。(3) PCL 实现把 score 取负成最小化，且把 $d_1$ 提为正常数，与论文差一负号，综合时统一为"最小化负对数似然"。

### 0.6 Horn 绝对定向记号（源 JOSA-A 1987）

| 记号 | 含义 |
|---|---|
| $\mathbf r_{l,i}$（left），$\mathbf r_{r,i}$（right） | 第 $i$ 对对应点在两坐标系中的坐标。 |
| $\mathbf r_{l,i}',\mathbf r_{r,i}'$ | 去质心后的坐标。 |
| $S_{jk}=\sum_i (r_l')_j (r_r')_k$ | 九个交叉乘积和，$j,k\in\{x,y,z\}$。 |
| $\mathbf M=\sum_i\mathbf r_{l,i}'(\mathbf r_{r,i}')^T$ | $3\times3$ 交叉协方差矩阵（元素即 $S_{jk}$）。 |
| $\mathbf N$ | 由 $S_{jk}$ 构造的 $4\times4$ 对称矩阵；其最大特征值对应特征向量 = 最优单位四元数。 |
| $s$ | 尺度因子（含 scale 的版本）。 |

**Horn 与本书差异**：Horn 求"left→right"旋转，对应本文"source→target"。Horn 四元数实部在前 $\mathring{\mathbf q}=(q_0,q_1,q_2,q_3)$，与本书 Hamilton 一致。

---

## 1. 点云表示与数据结构（综合背景，PCL 官方）

> 本节为后续滤波/配准提供数据结构语境（来源：PCL 官方文档、`pcl::PointCloud` / `pcl::PointXYZ` 等类型文档；与配准论文的"点集 $\{\mathbf p_i\}$"抽象对接）。

### 1.1 点云的数学抽象

**定义 1.1（点云）**。点云是 $\mathbb R^3$（或带属性的高维空间）中有限点的集合
$$\mathcal P=\{\mathbf p_i\in\mathbb R^3\mid i=1,\dots,N\}.$$
若每点附带属性（法向 $\mathbf n_i$、颜色 $(r,g,b)$、强度 $I_i$、时间戳 $t_i$、曲率 $c_i$ 等），则点云是 $\mathbb R^{3}\times\mathcal F$ 上的有限集，$\mathcal F$ 为属性空间。点云**无固定拓扑/无序**（这是与图像/网格的本质区别）——配准、滤波、最近邻都建立在"无序点集 + 空间邻近"之上，故需空间索引数据结构（KD-tree / octree / voxel grid）。

### 1.2 PCL 点类型（数据结构落地）

PCL 用模板化的点结构体表达属性组合：

| 类型 | 字段 | 用途 |
|---|---|---|
| `pcl::PointXYZ` | $x,y,z$（float，含 4 字节 padding 对齐到 16 字节，便于 SSE） | 纯几何点云。 |
| `pcl::PointXYZI` | $x,y,z,\text{intensity}$ | LiDAR 强度点云。 |
| `pcl::PointXYZRGB` / `PointXYZRGBA` | $x,y,z$ + 打包的 RGB(A) | RGB-D / 彩色点云。 |
| `pcl::PointNormal` | $x,y,z,n_x,n_y,n_z,\text{curvature}$ | 带法向（point-to-plane / 法向估计）。 |
| `pcl::PointXYZINormal` | 几何 + 强度 + 法向 + 曲率 | LOAM 类特征点。 |

点云容器 `pcl::PointCloud<PointT>`：
- `points`：`std::vector<PointT>`（连续内存，SSE/AVX 友好）。
- `width`、`height`：若 `height==1` 为**无组织点云**（unorganized，纯列表）；若 `height>1` 为**有组织点云**（organized，如 RGB-D 的 $H\times W$ 像素网格，保留邻接关系，便于快速近邻 = 像素邻域）。
- `is_dense`：是否含 NaN/Inf（无效点）。
- `sensor_origin_`、`sensor_orientation_`：采集位姿。

> **本质洞察**：有组织点云（organized）天然带 2D 邻接 → 法向估计/近邻可走整数像素邻域 $O(1)$，无须 KD-tree；无组织点云必须建空间索引。LiDAR 的 range image（行=scan ring、列=方位角）本质也是 organized 表示——LOAM/LeGO-LOAM 正是利用这点做高效特征提取（见 §7 与 `lidar_slam__loam_family.md`）。

### 1.3 空间索引数据结构概览

- **KD-tree**（k-维二叉树）：递归用轴对齐超平面二分空间；最近邻 $O(\log N)$ 平均；静态点云首选（见 §3）。PCL `pcl::KdTreeFLANN`（封装 FLANN 库）。
- **Octree**（八叉树）：递归把立方体八等分；适合体素查询、变更检测、可变分辨率；PCL `pcl::octree::OctreePointCloudSearch`。
- **Voxel grid**（体素栅格）：规则立方格 hash；$O(1)$ 体素定位；降采样/NDT 用（见 §2.1、§5）。

---

## 2. 点云滤波与降采样（PCL 官方文档）

> 来源：PCL 滤波模块官方教程（VoxelGrid、StatisticalOutlierRemoval、RadiusOutlierRemoval、ConditionalRemoval、PassThrough）。配准前的预处理标准管线。

### 2.1 体素栅格降采样 VoxelGrid（centroid 法）

**原理**（PCL 官方）：在输入点云上覆盖一个**3D 体素栅格**（一组空间中的小立方盒子）。每个体素（leaf）内的所有点被其**质心（centroid）**近似（即下采样）。

**算法 2.1（VoxelGrid 降采样）**：

```
输入：点云 P，leaf size (lx, ly, lz)
1. 计算点云轴对齐包围盒 (min, max)
2. 体素栅格维数: D_x = ceil((max_x - min_x)/lx), 同理 D_y, D_z
3. for 每点 p_i in P:
4.     体素索引 (ix,iy,iz) = floor((p_i - min)/leaf_size)  // 逐分量
5.     线性键 key = ix + iy*D_x + iz*D_x*D_y
6.     把 p_i 累加到 hash[key]（累加坐标和、计数）
7. for 每个非空体素 key:
8.     输出点 = (1/n_key) * sum_{p in key} p   // 质心
```

**质心公式**：体素 $V$ 内有 $n$ 个点 $\{\mathbf p_1,\dots,\mathbf p_n\}$，输出代表点
$$\bar{\mathbf p}_V=\frac1n\sum_{i=1}^n\mathbf p_i. \tag{2.1}$$

**质心 vs 体素中心**（官方明确）：用**质心**比用**体素几何中心**慢一点，但**更准确地表示底层曲面**（surface），因为质心落在数据上。`ApproximateVoxelGrid` 则用更快的近似（hash + 体素中心化）牺牲精度换速度（NDT 教程即用之，leaf 0.2 m）。

**参数与缺省**：`setLeafSize(lx, ly, lz)`（单位米；例 `0.01f, 0.01f, 0.01f` = 1 cm 立方体素）。

**官方数值例**：`table_scene_lms400.pcd` 原 **460400** 点，leaf 0.01 m 体素滤波后降到 **41049** 点（约减 91%），几何信息基本保留。

**PCL 代码**：
```cpp
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
sor.setInputCloud(cloud);
sor.setLeafSize(0.01f, 0.01f, 0.01f);
sor.filter(*cloud_filtered);
```

> **本质洞察**：体素降采样是配准（ICP/NDT）前的**头号预处理**——既降算力（点数↓ → KD-tree 近邻↓），又起空间均匀化（消除近密远疏的采样偏置，避免 ICP 被高密度区主导）。LOAM/LIO-SAM 的 map 也用体素降采样去重（见 `lidar_slam__loam_family.md`：LOAM 地图 5 cm 体素，LIO-SAM 边/面子图 0.2 m/0.4 m）。

### 2.2 直通滤波 PassThrough

**原理**：沿某一字段（如 $z$ 坐标、强度）设区间 $[l,u]$，保留落在区间内（或外，`setFilterLimitsNegative(true)`）的点。
$$\mathcal P_{\text{out}}=\{\mathbf p\in\mathcal P\mid l\le p.\text{field}\le u\}. \tag{2.2}$$
用途：裁剪 ROI、去地面/天花板（按 $z$）、按距离/强度门限。
```cpp
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(cloud);
pass.setFilterFieldName("z");
pass.setFilterLimits(0.0, 1.0);
pass.filter(*cloud_filtered);
```

### 2.3 统计离群点剔除 StatisticalOutlierRemoval（SOR）

**原理**（PCL 官方）：对每个点，计算它到其 $k$ 个最近邻的**平均距离**；假设这些平均距离服从**高斯分布**（全局均值 $\mu$、标准差 $\sigma$）；平均距离落在区间 $[\mu-\alpha\sigma,\ \mu+\alpha\sigma]$ 之外的点判为离群点剔除。

**算法 2.2（SOR）**：

```
输入：点云 P，邻居数 k (setMeanK)，倍数 alpha (setStddevMulThresh)
1. for 每点 p_i:
2.     用 KD-tree 找 p_i 的 k 个最近邻
3.     d_i = (1/k) * sum_{j=1..k} ||p_i - neighbor_j||   // 平均近邻距离
4. mu  = mean(d_i over all i)
5. sigma = std(d_i over all i)
6. 阈值 T = mu + alpha * sigma
7. 保留 {p_i : d_i <= T}   (对称地，可加下界 mu - alpha*sigma)
```

数学表述：每点平均近邻距离
$$\bar d_i=\frac1k\sum_{j=1}^k\|\mathbf p_i-\mathbf p_{i_j}\|,\qquad
\mu=\frac1N\sum_i\bar d_i,\quad
\sigma=\sqrt{\frac1N\sum_i(\bar d_i-\mu)^2}. \tag{2.3}$$
判据（剔除离群）：
$$\mathbf p_i\ \text{是离群点}\iff \bar d_i>\mu+\alpha\sigma\ \ (\text{或}\ \bar d_i<\mu-\alpha\sigma). \tag{2.4}$$

**参数与缺省**：`setMeanK(50)`（$k=50$ 个邻居）、`setStddevMulThresh(1.0)`（$\alpha=1.0$）。`setNegative(true)` 反选（提取离群点本身）。
```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud(cloud);
sor.setMeanK(50);
sor.setStddevMulThresh(1.0);
sor.filter(*cloud_filtered);
```

> **陷阱**：SOR 假设近邻距离高斯——在**密度强烈非均匀**（近密远疏的 LiDAR 原始点云）时会误删远处稀疏但有效的点。常先做体素降采样均匀化、或对 LiDAR 用按距离自适应的门限。

### 2.4 半径离群点剔除 RadiusOutlierRemoval（ROR）

**原理**（PCL 官方）：对每点，统计其**半径 $r$ 球内**的邻居数；邻居数少于阈值 $k_{\min}$ 的点判为离群点剔除。
$$\mathbf p_i\ \text{是内点}\iff \big|\{\mathbf p_j:\ \|\mathbf p_i-\mathbf p_j\|\le r,\ j\ne i\}\big|\ge k_{\min}. \tag{2.5}$$
**参数**：`setRadiusSearch(r)`（球半径，例 0.8）、`setMinNeighborsInRadius(k_min)`（例 2）。
```cpp
pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
outrem.setInputCloud(cloud);
outrem.setRadiusSearch(0.8);
outrem.setMinNeighborsInRadius(2);
outrem.filter(*cloud_filtered);
```
**与 SOR 对比**：ROR 不假设分布、直接看局部密度，更简单更快，但需手调 $r$；SOR 自适应全局分布，但假设高斯。

### 2.5 条件滤波 ConditionalRemoval

按任意布尔条件（字段比较的 AND/OR 组合）保留点。例：保留 $0.0\le z\le 0.8$ 的点：
```cpp
pcl::ConditionAnd<pcl::PointXYZ>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZ>());
cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
    new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
condrem.setCondition(cond);
condrem.setInputCloud(cloud);
condrem.setKeepOrganized(true);
condrem.filter(*cloud_filtered);
```

### 2.6 标准预处理管线（综合）

典型配准前管线：**PassThrough（ROI 裁剪）→ VoxelGrid（降采样均匀化）→ SOR/ROR（去噪）→（可选）法向估计**。法向估计（point-to-plane/GICP/NDT-D2D 需要）：对每点取 $k$ 近邻，求邻域协方差，最小特征值对应特征向量即法向（见 §4.2 的协方差-PCA 法，与 GICP 协方差构造同源）。

---

## 3. 最近邻搜索：KD-tree（k-维树）

> 来源：Wikipedia *k-d tree*（开放教育资源，方法层面）；与 Friedman, Bentley, Finkel 1977 的经典 KD-tree、PCL `KdTreeFLANN`（封装 FLANN 库的随机化 KD-tree forest）一致。ICP/NDT/GICP 每次迭代的对应步都靠 KD-tree 找最近邻，是配准的算力瓶颈与加速关键。

### 3.1 动机与问题

配准的"对应步"需对 source 每点在 target 中找最近邻（NN）。暴力法 $O(N_a N_b)$（两点云各 $N$ 点则 $O(N^2)$），不可接受。KD-tree 把 target 预处理成二叉空间划分树，**单次 NN 平均 $O(\log N)$**，建树 $O(N\log N)$ 一次性摊销。

### 3.2 KD-tree 构造

**定义 3.1（KD-tree）**。$k$ 维 KD-tree 是二叉树，每个非叶结点存一个 $k$ 维点，并**隐式定义一个轴对齐分割超平面**，把空间分为两半空间（half-spaces）；左/右子树分别落在超平面两侧。

**构造法（经典平衡建树，按中位数分割）**：沿深度循环切换分割轴（$\text{axis}=\text{depth}\bmod k$），取当前轴上的**中位数点**作为分割点，使树平衡（每个叶到根近似等距）。

**算法 3.1（KD-tree 建树伪码）**：

```
function kdtree(pointList, depth):
    if pointList is empty: return null
    axis = depth mod k                         // 循环切换分割维
    sort pointList by coordinate[axis]         // 或用 median-of-medians 选中位
    median = select_median(pointList, axis)
    node.location   = median
    node.leftChild  = kdtree(points before median along axis, depth + 1)
    node.rightChild = kdtree(points after  median along axis, depth + 1)
    return node
```

**构造复杂度**：
- 每层用 $O(n\log n)$ 排序找中位 → 总 $O(n\log^2 n)$。
- 用 $O(n)$ 的 median-of-medians 选中位 → 总 $O(n\log n)$。
- 若各维已预排序 → $O(k\,n\log n)$。

### 3.3 最近邻搜索（含剪枝）

**算法 3.2（KD-tree 最近邻搜索）**（核心是"超球是否穿过分割面"的剪枝）：

```
function nearestNeighbor(node, query, depth, best):
    if node is null: return best
    // 1. 更新当前最优（含内部结点的点）
    if distance(query, node.location) < best.distance:
        best = {point: node.location, distance: distance(query, node.location)}
    axis = depth mod k
    // 2. 先递归"近"侧子树（query 落在的那侧）
    if query[axis] < node.location[axis]:
        near_child = node.leftChild;  far_child = node.rightChild
    else:
        near_child = node.rightChild; far_child = node.leftChild
    best = nearestNeighbor(near_child, query, depth+1, best)
    // 3. 剪枝判定：以 query 为心、best.distance 为半径的超球，
    //    是否跨过分割超平面？若跨过，"远"侧可能含更近点，须搜
    if |query[axis] - node.location[axis]| < best.distance:
        best = nearestNeighbor(far_child, query, depth+1, best)
    return best
```

**剪枝原理（关键直觉）**：第 3 步中 $|q_{\text{axis}}-\text{node}_{\text{axis}}|$ 是 query 到分割超平面的垂直距离。若它 $\ge$ 当前最优半径，则**整个远侧半空间到 query 的距离都 $\ge$ 当前最优**，可整支剪掉，无须递归——这是 KD-tree 加速的本质。

**搜索复杂度**：
- 平均（点随机分布）：$O(\log N)$。
- 最坏：$O(N)$。

### 3.4 范围搜索（radius / box search）

KD-tree 每层把域二分，适合范围查询。半径 $r$ 球内搜索 = NN 搜索把"best.distance"固定为 $r$、收集所有满足的点。框范围搜索最坏复杂度
$$O\!\big(k\,N^{1-1/k}+m\big), \tag{3.1}$$
$m$ = 命中点数。ICP 的"最大匹配距离 $d_{\max}$"即用半径搜索过滤。

### 3.5 近似最近邻（ANN）与高维退化

**ANN 加速手段**：限制检查点数上限、限时搜索可被实时中断、best-bin-first（优先搜最可能含近邻的桶，按到分割面距离排优先队列）。FLANN（PCL 默认）用**随机化 KD-tree forest + 优先队列**做 ANN，大规模点云常用。

**维度灾难（curse of dimensionality）**：KD-tree 高维退化。经验规则 $N\gg 2^k$ 才有效；否则近邻搜索退化为近似线性扫描。对 $\mathbb R^3$ 点云（$k=3$）KD-tree 非常高效；但对高维特征描述子（如 FPFH 33 维）KD-tree 优势减弱 → 用 ANN/FLANN。

> **本质洞察**：ICP 系列每次迭代都重建/重查 KD-tree（source 变换后位置变，对应关系要重算），故"对应步 = KD-tree 近邻"是 ICP 的算力主项。NDT 的优势正在于**无须显式对应**（用 voxel hash $O(1)$ 定位 cell，不查 KD-tree），故 NDT 通常比 ICP 快（见 §5、§6）。

---

## 4. ICP 系列之一：点到点 ICP（point-to-point）

> 来源：Besl & McKay 1992（PAMI）；Horn 1987（四元数闭式解）；Arun et al. 1987 / Umeyama 1991 / Kabsch（SVD 闭式解）；Wikipedia *Iterative closest point* / *Point-set registration* / *Kabsch algorithm*。这是 ICP 的奠基形态。

### 4.1 问题与代价函数

给定 source 点云 $\mathcal A=\{\mathbf a_i\}$（"动"，data shape $P$）与 target $\mathcal B=\{\mathbf b_i\}$（"静"，model shape $X$），求刚体变换 $(\mathbf R,\mathbf t)$ 极小化对应点对的平方欧氏距离和：
$$E(\mathbf R,\mathbf t)=\sum_{i=1}^{N}\big\|\,(\mathbf R\mathbf a_i+\mathbf t)-\mathbf b_i\,\big\|_2^2. \tag{4.1}$$
难点：(a) 对应关系 $i\leftrightarrow$ target 中哪个点未知；(b) $\mathbf R\in\mathrm{SO}(3)$ 非凸约束。**ICP 用交替（alternating）策略**：固定对应求最优变换（闭式，第 4.4/4.5 节），固定变换重算对应（KD-tree 最近邻），迭代。

### 4.2 Besl-McKay ICP 算法（PAMI 1992 原文）

**closest point operator**：data 点 $\mathbf p$ 到 model $X$ 的最近点
$$d(\mathbf p,X)=\min_{\mathbf x\in X}\|\mathbf x-\mathbf p\|,\qquad
\mathbf y=C(\mathbf p,X)=\arg\min_{\mathbf x\in X}\|\mathbf x-\mathbf p\|. \tag{4.2}$$
$C$ 对点集用 KD-tree 实现；Besl-McKay 还给出到线段/三角面/参数曲面的最近点闭式（model 可为非点集几何）。

**算法 4.1（Besl-McKay ICP）**：

```
输入：data shape P (N_p 点)，model shape X，阈值 tau
初始化：P_0 = P,  q_0 = [1,0,0,0, 0,0,0]^T (单位四元数+零平移),  k = 0
repeat:
  (a) 对应步: 对每个 p_i in P_0 (原始 data)，在 X 上求最近点
            y_i = C(R(q_k) p_i + t(q_k), X)        // 当前位姿下的最近点
            得对应集 Y_k = {y_i}
  (b) 配准步: 求 (R, t) 极小化 sum ||R p_i + t - y_i||^2   // 闭式解, §4.4/4.5
            得 q_{k+1},  均方误差 d_{k+1} = (1/N_p) sum ||R p_i + t - y_i||^2
  (c) 变换:  (隐式) 更新当前位姿
  (d) k = k+1
until  d_k - d_{k+1} < tau          // 均方误差改变量低于阈值
输出：最终变换 q
```

**收敛定理（Besl-McKay 核心结果）**：

> **定理 4.1（ICP 单调收敛）**。ICP 算法对均方距离度量**单调收敛到（最近的）局部极小**（"always converges monotonically to the nearest local minimum of a mean-square distance metric"），且**前几次迭代收敛极快**（"rate of convergence is rapid during the first few iterations"）。

**证明思路（Besl-McKay）**：每次迭代两步都不增大目标 $f$：
1. **对应步**：固定变换，对每个 $\mathbf p_i$ 取**最近**点 $\mathbf y_i$ —— 由最近点定义，$\sum\|\cdot-\mathbf y_i\|^2$ 不大于固定任何其他对应时的值，故目标不增。
2. **配准步**：固定对应 $\{\mathbf y_i\}$，求**全局最优**刚体变换（闭式解，是该子问题的全局最小），故目标不增。
两步都使 $f$ 单调不增，且 $f\ge0$ 有下界 → 单调有下界序列收敛。因每步是贪心局部改进、对应关系离散，收敛到局部极小（非全局，故需好初值）。$\blacksquare$

> ⚠️ **陷阱**：ICP 只保证收敛到**局部**极小 → 强依赖初值。初值差会陷入错误对齐。实务用粗配准（特征匹配 / NDT 粗对齐 / 里程计先验）给初值，再 ICP 精配准。

### 4.3 配准子问题（固定对应求最优刚体变换）

配准步即"绝对定向 / Procrustes / Wahba"问题：给定对应点对 $\{(\mathbf a_i,\mathbf y_i)\}_{i=1}^N$，求
$$(\mathbf R^\star,\mathbf t^\star)=\arg\min_{\mathbf R\in\mathrm{SO}(3),\,\mathbf t}\sum_{i=1}^N\|\mathbf R\mathbf a_i+\mathbf t-\mathbf y_i\|^2. \tag{4.3}$$

**第一步：消去平移（去质心）**。设质心
$$\bar{\mathbf a}=\frac1N\sum_i\mathbf a_i,\qquad \bar{\mathbf y}=\frac1N\sum_i\mathbf y_i. \tag{4.4}$$
去质心坐标 $\mathbf a_i'=\mathbf a_i-\bar{\mathbf a}$、$\mathbf y_i'=\mathbf y_i-\bar{\mathbf y}$。代入 (4.3) 展开：
$$\sum_i\|\mathbf R\mathbf a_i+\mathbf t-\mathbf y_i\|^2
=\sum_i\|\mathbf R\mathbf a_i'-\mathbf y_i'+(\mathbf R\bar{\mathbf a}+\mathbf t-\bar{\mathbf y})\|^2.$$
令 $\mathbf c=\mathbf R\bar{\mathbf a}+\mathbf t-\bar{\mathbf y}$，交叉项 $\sum_i(\mathbf R\mathbf a_i'-\mathbf y_i')$ 因 $\sum_i\mathbf a_i'=\sum_i\mathbf y_i'=\mathbf 0$ 而为零，故
$$\sum_i\|\cdot\|^2=\sum_i\|\mathbf R\mathbf a_i'-\mathbf y_i'\|^2+N\|\mathbf c\|^2. \tag{4.5}$$
第二项 $N\|\mathbf c\|^2\ge0$ 与 $\mathbf R$ 无关，最优时取 $\mathbf c=\mathbf 0$，得**平移闭式解**
$$\boxed{\ \mathbf t^\star=\bar{\mathbf y}-\mathbf R^\star\bar{\mathbf a}\ }. \tag{4.6}$$
于是只剩**纯旋转子问题**：
$$\mathbf R^\star=\arg\min_{\mathbf R\in\mathrm{SO}(3)}\sum_i\|\mathbf R\mathbf a_i'-\mathbf y_i'\|^2. \tag{4.7}$$
展开 (4.7)：$\sum_i(\|\mathbf R\mathbf a_i'\|^2+\|\mathbf y_i'\|^2-2\mathbf y_i'^T\mathbf R\mathbf a_i')$，因 $\|\mathbf R\mathbf a_i'\|=\|\mathbf a_i'\|$（旋转保模），前两项与 $\mathbf R$ 无关，故
$$\mathbf R^\star=\arg\max_{\mathbf R\in\mathrm{SO}(3)}\sum_i\mathbf y_i'^T\mathbf R\mathbf a_i'
=\arg\max_{\mathbf R}\operatorname{tr}\!\Big(\mathbf R\underbrace{\sum_i\mathbf a_i'\mathbf y_i'^T}_{=:\mathbf H}\Big). \tag{4.8}$$
（用 $\mathbf y'^T\mathbf R\mathbf a'=\operatorname{tr}(\mathbf R\mathbf a'\mathbf y'^T)$。）即极大化 $\operatorname{tr}(\mathbf R\mathbf H)$，$\mathbf H=\sum_i\mathbf a_i'\mathbf y_i'^T$ 为 $3\times3$ 交叉协方差矩阵。

### 4.4 SVD 闭式解（Arun / Umeyama / Kabsch）

**定理 4.2（旋转的 SVD 闭式解）**。设 $\mathbf H=\sum_i\mathbf a_i'\mathbf y_i'^T$ 的奇异值分解为 $\mathbf H=\mathbf U\boldsymbol\Sigma\mathbf V^T$，则 (4.8) 的最优旋转为
$$\boxed{\ \mathbf R^\star=\mathbf V\,\mathbf U^T\ }\quad(\text{若 }\det(\mathbf V\mathbf U^T)>0). \tag{4.9}$$
若 $\det(\mathbf V\mathbf U^T)<0$（出现反射，非真旋转），修正为
$$\boxed{\ \mathbf R^\star=\mathbf V\,\mathrm{diag}(1,1,\det(\mathbf V\mathbf U^T))\,\mathbf U^T\ }
=\mathbf V\,\mathrm{diag}(1,1,-1)\,\mathbf U^T. \tag{4.10}$$

**证明**。$\operatorname{tr}(\mathbf R\mathbf H)=\operatorname{tr}(\mathbf R\mathbf U\boldsymbol\Sigma\mathbf V^T)=\operatorname{tr}(\boldsymbol\Sigma\mathbf V^T\mathbf R\mathbf U)$。令 $\mathbf Z=\mathbf V^T\mathbf R\mathbf U$，为正交矩阵（$\mathbf R,\mathbf U,\mathbf V$ 正交）。则 $\operatorname{tr}(\mathbf R\mathbf H)=\operatorname{tr}(\boldsymbol\Sigma\mathbf Z)=\sum_{j}\sigma_j Z_{jj}$，$\sigma_j\ge0$。正交矩阵元素满足 $|Z_{jj}|\le1$，故 $\sum_j\sigma_j Z_{jj}\le\sum_j\sigma_j$，等号当 $Z_{jj}=1\ \forall j$ 即 $\mathbf Z=\mathbf I$，得 $\mathbf V^T\mathbf R\mathbf U=\mathbf I\Rightarrow\mathbf R=\mathbf V\mathbf U^T$。
若 $\det(\mathbf V\mathbf U^T)=-1$，则 $\mathbf V\mathbf U^T$ 是反射不属 $\mathrm{SO}(3)$；在 $\mathrm{SO}(3)$ 内的最优解是把最小奇异值方向翻号：$\mathbf Z=\mathrm{diag}(1,1,-1)$（牺牲最小 $\sigma_3$），得 (4.10)。$\blacksquare$

> ⚠️ **方向/转置陷阱（综合必读）**：闭式解的形式**强依赖 $\mathbf H$ 的定义与 source/target 谁在前**：
> - 本文 / Arun：$\mathbf H=\sum\mathbf a'(\mathbf y')^T$（source 在前），$\Rightarrow\mathbf R=\mathbf V\mathbf U^T$，满足 $\mathbf y\approx\mathbf R\mathbf a$。
> - Wikipedia *Kabsch*：$\mathbf H=\mathbf P^T\mathbf Q$（行为点、$\mathbf P$=待转、$\mathbf Q$=目标），$\Rightarrow\mathbf R=\mathbf U\,\mathrm{diag}(1,1,d)\mathbf V^T$，$d=\mathrm{sign}\det(\mathbf U\mathbf V^T)$，满足 $\mathbf Q\approx\mathbf P\mathbf R$（**右乘**，因点是行向量）。
> - 二者经转置等价：$\big(\sum\mathbf a'\mathbf y'^T\big)=\big(\mathbf P^T\mathbf Q\big)$ 当 $\mathbf P$ 行为 $\mathbf a'$、$\mathbf Q$ 行为 $\mathbf y'$；$\mathbf R_{\text{Arun}}=\mathbf R_{\text{Kabsch}}^T$（一个作用于列向量左乘、一个作用于行向量右乘）。**移植时务必核对 $\mathbf H$ 定义、行/列约定、det 修正放在 $\mathbf U$ 侧还是 $\mathbf V$ 侧**——这是 ICP 实现最常见 bug。
>
> nghiaho.com 的实用版（与本文一致）：$\mathbf H=\sum(\mathbf A_i-\bar{\mathbf A})(\mathbf B_i-\bar{\mathbf B})^T$，$[\mathbf U,\mathbf S,\mathbf V]=\mathrm{SVD}(\mathbf H)$，$\mathbf R=\mathbf V\mathbf U^T$；若 $\det(\mathbf R)<0$ 则把 $\mathbf V$ 第 3 列乘 $-1$ 后重算 $\mathbf R=\mathbf V\mathbf U^T$；$\mathbf t=\bar{\mathbf B}-\mathbf R\bar{\mathbf A}$。

**Umeyama 扩展（含尺度）**：若允许相似变换 $\mathbf y\approx s\mathbf R\mathbf a+\mathbf t$，最优尺度
$$s^\star=\frac{1}{\sigma_a^2}\operatorname{tr}\!\big(\boldsymbol\Sigma\,\mathrm{diag}(1,1,d)\big),\qquad
\sigma_a^2=\frac1N\sum_i\|\mathbf a_i'\|^2, \tag{4.11}$$
$\boldsymbol\Sigma$ 为 $\mathbf H/N$ 的奇异值矩阵，$d$ 为 det 修正号。旋转同 (4.10)，平移 $\mathbf t^\star=\bar{\mathbf y}-s^\star\mathbf R^\star\bar{\mathbf a}$。

### 4.5 四元数闭式解（Horn 1987 / Besl-McKay 采用）

Besl-McKay 原文用**单位四元数**解旋转（Horn 法），避免 SVD 反射问题（四元数天然给真旋转）。

**第一步：交叉协方差与九个和**。去质心后 $3\times3$ 交叉协方差
$$\mathbf M=\sum_i\mathbf a_i'(\mathbf y_i')^T,\qquad
S_{jk}=\sum_i (a_i')_j (y_i')_k,\ \ j,k\in\{x,y,z\}. \tag{4.12}$$
（即 $\mathbf M$ 的元素 $M_{jk}=S_{jk}$；本文 source=$\mathbf a$ 对应 Horn 的 left，target=$\mathbf y$ 对应 right。）

**第二步：构造 $4\times4$ 对称矩阵 $\mathbf N$**（Horn 原式）：
$$\mathbf N=\begin{pmatrix}
S_{xx}+S_{yy}+S_{zz} & S_{yz}-S_{zy} & S_{zx}-S_{xz} & S_{xy}-S_{yx}\\[2pt]
S_{yz}-S_{zy} & S_{xx}-S_{yy}-S_{zz} & S_{xy}+S_{yx} & S_{zx}+S_{xz}\\[2pt]
S_{zx}-S_{xz} & S_{xy}+S_{yx} & -S_{xx}+S_{yy}-S_{zz} & S_{yz}+S_{zy}\\[2pt]
S_{xy}-S_{yx} & S_{zx}+S_{xz} & S_{yz}+S_{zy} & -S_{xx}-S_{yy}+S_{zz}
\end{pmatrix}. \tag{4.13}$$
（对称，$\operatorname{tr}\mathbf N=0$。注意 $\mathbf N$ 的迹为 0：四个对角和 $(S_{xx}+S_{yy}+S_{zz})+(S_{xx}-S_{yy}-S_{zz})+(-S_{xx}+S_{yy}-S_{zz})+(-S_{xx}-S_{yy}+S_{zz})=0$。）

**第三步：最大特征值法**。

> **定理 4.3（Horn）**。使 $\sum_i\mathbf y_i'^T\mathbf R(\mathring{\mathbf q})\mathbf a_i'$ 最大（即 (4.8)）的最优单位四元数 $\mathring{\mathbf q}=(q_0,q_1,q_2,q_3)$，是 $4\times4$ 对称矩阵 $\mathbf N$ 的**最大（最正）特征值对应的（单位）特征向量**。

**证明梗概（Horn）**。把待最大化量写成四元数双线性型 $\sum_i\mathring{\mathbf y}_i'^*\,\mathring{\mathbf q}\,\mathring{\mathbf a}_i'\,\mathring{\mathbf q}^*$（用四元数旋转 $\mathbf R(\mathring{\mathbf q})\mathbf v\leftrightarrow\mathring{\mathbf q}\mathring{\mathbf v}\mathring{\mathbf q}^*$），整理为 $\mathring{\mathbf q}^T\mathbf N\mathring{\mathbf q}$（$\mathbf N$ 由 $S_{jk}$ 如 (4.13) 构造）。在约束 $\|\mathring{\mathbf q}\|=1$ 下极大化 Rayleigh 商 $\mathring{\mathbf q}^T\mathbf N\mathring{\mathbf q}$ → 解为 $\mathbf N$ 最大特征值的特征向量（Rayleigh-Ritz）。$\blacksquare$

**第四步：平移**。得 $\mathring{\mathbf q}$ 后旋转 $\mathbf R=\mathbf R(\mathring{\mathbf q})$，平移
$$\mathbf t=\bar{\mathbf y}-\mathbf R\,\bar{\mathbf a}\quad(\text{含尺度时}\ \mathbf t=\bar{\mathbf y}-s\mathbf R\bar{\mathbf a}). \tag{4.14}$$
四元数 → 旋转矩阵（Hamilton，与本书一致）：
$$\mathbf R(\mathring{\mathbf q})=\begin{pmatrix}
q_0^2+q_1^2-q_2^2-q_3^2 & 2(q_1q_2-q_0q_3) & 2(q_1q_3+q_0q_2)\\
2(q_1q_2+q_0q_3) & q_0^2-q_1^2+q_2^2-q_3^2 & 2(q_2q_3-q_0q_1)\\
2(q_1q_3-q_0q_2) & 2(q_2q_3+q_0q_1) & q_0^2-q_1^2-q_2^2+q_3^2
\end{pmatrix}. \tag{4.15}$$

> **本质洞察（SVD vs 四元数）**：两法等价、都闭式。SVD 法（Arun/Kabsch）更通用（直接给 $\mathbf R$，可扩展尺度），但需 det 修正防反射；四元数法（Horn/Besl）天然落在 $\mathrm{SO}(3)$（无反射问题），且对最大特征值法数值稳健。本书右扰动主线下，配准子问题也可用 Gauss-Newton 在 $\mathfrak{so}(3)$ 上迭代（小角度），但有闭式解时闭式更快更稳。

### 4.6 点到点 ICP 的 Gauss-Newton / 李代数视角（本书右扰动主线，供综合）

> 综合时本书优先用右扰动 + 流形优化（对接 GTSAM/Ceres）。给出点到点 ICP 残差对右扰动的雅可比，便于与本书李群章对接。`\rebuilt`（标准结果，照本书李群章约定重写）。

残差 $\mathbf r_i(\mathbf R,\mathbf t)=\mathbf R\mathbf a_i+\mathbf t-\mathbf b_i$。右扰动 $\mathbf R\leftarrow\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$、$\mathbf t\leftarrow\mathbf t+\mathbf R\,\delta\boldsymbol\rho$（或直接 $\mathbf t+\delta\mathbf t$）。用 $\mathrm{Exp}(\delta\boldsymbol\phi)\approx\mathbf I+\lfloor\delta\boldsymbol\phi\rfloor_\times$：
$$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)\mathbf a_i\approx\mathbf R\mathbf a_i+\mathbf R\lfloor\delta\boldsymbol\phi\rfloor_\times\mathbf a_i
=\mathbf R\mathbf a_i-\mathbf R\lfloor\mathbf a_i\rfloor_\times\delta\boldsymbol\phi.$$
故残差对 $(\delta\boldsymbol\phi,\delta\mathbf t)$ 的雅可比（右扰动）：
$$\frac{\partial\mathbf r_i}{\partial\delta\boldsymbol\phi}=-\mathbf R\lfloor\mathbf a_i\rfloor_\times,\qquad
\frac{\partial\mathbf r_i}{\partial\delta\mathbf t}=\mathbf I_3. \tag{4.16}$$
Gauss-Newton 正规方程 $\big(\sum_i\mathbf J_i^T\mathbf J_i\big)\Delta\mathbf x=-\sum_i\mathbf J_i^T\mathbf r_i$，$\mathbf J_i=[-\mathbf R\lfloor\mathbf a_i\rfloor_\times\ \ \mathbf I_3]$，$\Delta\mathbf x=[\delta\boldsymbol\phi;\delta\mathbf t]$；更新 $\mathbf R\leftarrow\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$、$\mathbf t\leftarrow\mathbf t+\delta\mathbf t$。迭代收敛即点到点 ICP 配准步（与闭式解殊途同归）。

### 4.7 鲁棒化与现代视角（Wikipedia point-set registration）

实际 ICP 的对应含外点，用 M-估计鲁棒核 $\rho(\cdot)$：
$$(\,l^\star,\mathbf R^\star,\mathbf t^\star)=\arg\min_{l>0,\mathbf R\in\mathrm{SO}(3),\mathbf t}\sum_{i=1}^N\rho\!\Big(\tfrac1{\sigma_i}\|\mathbf b_i-l\mathbf R\mathbf a_i-\mathbf t\|_2\Big), \tag{4.17}$$
$\rho$ 取 $L_1$、Huber、truncated least squares（TLS）等。**TEASER**（Yang et al.）用 TLS + 半定松弛做可证鲁棒配准：
$$\arg\min_{l,\mathbf R,\mathbf t}\sum_{i=1}^N\min\!\Big(\tfrac1{\sigma_i^2}\|\mathbf b_i-l\mathbf R\mathbf a_i-\mathbf t\|_2^2,\ \bar c^2\Big), \tag{4.18}$$
通过不变量解耦 scale/rotation/translation 分别求解。**Coherent Point Drift（CPD）**（Myronenko-Song）把 source 视为高斯混合中心，EM 优化负对数似然
$$E(\theta,\sigma^2)=-\sum_{j=1}^N\log\sum_{i=1}^{M+1}P(i)\,p(\mathbf b_j\mid i),\quad
p(\mathbf b\mid i)=\tfrac{1}{(2\pi\sigma^2)^{3/2}}\exp\!\Big(-\tfrac{\|\mathbf b-\mathbf m_i\|^2}{2\sigma^2}\Big). \tag{4.19}$$
（NDT 与 CPD/GMM 思想同源：都用高斯/混合替代硬对应，见 §5、§6。）

---

## 5. ICP 系列之二：点到面 ICP（point-to-plane）

> 来源：Chen & Medioni 1992（提出 point-to-plane 度量）；Kok-Lim Low 2004 TR04-004（小角度线性化 → 6×6 线性最小二乘的完整推导）；OpenCV KinFu ICP（同款线性系统的 $G(p)$ 矩阵实现）。point-to-plane 收敛通常比 point-to-point 快得多（沿切平面"滑动"不受惩罚），结构化场景（室内/城市）尤甚。

### 5.1 动机与点到面误差度量

point-to-point 把残差取整个向量 $\|\mathbf R\mathbf a_i+\mathbf t-\mathbf b_i\|$，但真实是两个**曲面**对齐——只要落在对方切平面上即可，沿切平面方向的偏移不应罚。**Chen-Medioni point-to-plane 度量**只罚残差沿 target 法向 $\mathbf n_i$ 的投影：
$$E_{\text{p2pl}}(\mathbf R,\mathbf t)=\sum_{i=1}^N\Big(\big[(\mathbf R\mathbf s_i+\mathbf t)-\mathbf d_i\big]\cdot\mathbf n_i\Big)^2
=\sum_i\big(\mathbf n_i^T(\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i)\big)^2, \tag{5.1}$$
其中 $\mathbf s_i$=source 点、$\mathbf d_i$=对应 target 点、$\mathbf n_i$=$\mathbf d_i$ 处单位法向。

> **本质洞察**：(5.1) 允许 source 沿 target 切平面自由滑动而不增代价 → 在平面/墙面丰富处对齐自由度被法向约束精确锁定、切向不被噪声/采样错位干扰，故收敛更快、对部分重叠更鲁棒。代价：需先估 target 法向（PCA，见 §2.6）。

### 5.2 刚体变换与小角度线性化（Low 推导）

完整刚体变换（Low 用 Euler $\alpha,\beta,\gamma$ 绕 $x,y,z$ + 平移）：
$$\mathbf M=\mathbf T(t_x,t_y,t_z)\,\mathbf R(\alpha,\beta,\gamma),\qquad
\mathbf R=\mathbf R_z(\gamma)\mathbf R_y(\beta)\mathbf R_x(\alpha). \tag{5.2}$$
其 $3\times3$ 旋转部分（展开）：
$$\mathbf R=\begin{pmatrix}
\cos\gamma\cos\beta & -\sin\gamma\cos\alpha+\cos\gamma\sin\beta\sin\alpha & \sin\gamma\sin\alpha+\cos\gamma\sin\beta\cos\alpha\\
\sin\gamma\cos\beta & \cos\gamma\cos\alpha+\sin\gamma\sin\beta\sin\alpha & -\cos\gamma\sin\alpha+\sin\gamma\sin\beta\cos\alpha\\
-\sin\beta & \cos\beta\sin\alpha & \cos\beta\cos\alpha
\end{pmatrix}. \tag{5.3}$$
**小角度近似**（ICP 每次迭代相对位姿很小）：$\sin\theta\to\theta$、$\cos\theta\to1$，并**丢弃二阶以上小量**（$\sin\sin$、$\theta^2$ 项）。则 (5.3) 退化为
$$\mathbf R\approx\begin{pmatrix}1 & -\gamma & \beta\\ \gamma & 1 & -\alpha\\ -\beta & \alpha & 1\end{pmatrix}
=\mathbf I+\lfloor(\alpha,\beta,\gamma)\rfloor_\times=\mathbf I+\mathrm{skew}\big((\alpha,\beta,\gamma)^T\big). \tag{5.4}$$
（OpenCV KinFu 直接写为 $\mathbf R=\mathbf I+\mathrm{skew}([\alpha,\beta,\gamma]^T)$，与 Low 一致。）

### 5.3 线性化残差与 6×6 线性最小二乘（Low 核心推导）

把 (5.4) 代入残差 $\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i$。记 $\mathbf r=(\alpha,\beta,\gamma)^T$、$\mathbf t=(t_x,t_y,t_z)^T$：
$$\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i
\approx\big(\mathbf I+\lfloor\mathbf r\rfloor_\times\big)\mathbf s_i+\mathbf t-\mathbf d_i
=(\mathbf s_i-\mathbf d_i)+\lfloor\mathbf r\rfloor_\times\mathbf s_i+\mathbf t.$$
用叉乘恒等式 $\lfloor\mathbf r\rfloor_\times\mathbf s_i=\mathbf r\times\mathbf s_i=-\mathbf s_i\times\mathbf r=-\lfloor\mathbf s_i\rfloor_\times\mathbf r$：
$$\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i\approx(\mathbf s_i-\mathbf d_i)+(\mathbf s_i\times\mathbf r)\ \text{?}\ \cdots$$
按 OpenCV/Low 的符号（$\lfloor\mathbf r\rfloor_\times\mathbf s_i=\mathbf r\times\mathbf s_i$，但写成对 $\mathbf r$ 线性需 $=-\lfloor\mathbf s_i\rfloor_\times\mathbf r=\lfloor\mathbf s_i\rfloor_\times^T\mathbf r$），点到面残差（标量）：
$$\mathbf n_i^T(\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i)
\approx\mathbf n_i^T(\mathbf s_i-\mathbf d_i)+\mathbf n_i^T(\mathbf r\times\mathbf s_i)+\mathbf n_i^T\mathbf t. \tag{5.5}$$
用标量三重积循环性 $\mathbf n_i^T(\mathbf r\times\mathbf s_i)=\mathbf n_i\cdot(\mathbf r\times\mathbf s_i)=\mathbf r\cdot(\mathbf s_i\times\mathbf n_i)=(\mathbf s_i\times\mathbf n_i)^T\mathbf r$，得**对未知量线性**的形式：
$$\mathbf n_i^T(\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i)
\approx (\mathbf s_i\times\mathbf n_i)^T\,\mathbf r+\mathbf n_i^T\,\mathbf t+\mathbf n_i^T(\mathbf s_i-\mathbf d_i). \tag{5.6}$$

**组装线性系统**。令 6 维未知量 $\mathbf x=(\alpha,\beta,\gamma,t_x,t_y,t_z)^T=[\mathbf r;\mathbf t]$，每个对应贡献一行
$$\mathbf a_i^T=\big[(\mathbf s_i\times\mathbf n_i)^T\ \ \mathbf n_i^T\big]\in\mathbb R^{1\times6},\qquad
b_i=\mathbf n_i^T(\mathbf d_i-\mathbf s_i)=-\mathbf n_i^T(\mathbf s_i-\mathbf d_i). \tag{5.7}$$
点到面代价 (5.1) 线性化为
$$E_{\text{p2pl}}\approx\sum_i(\mathbf a_i^T\mathbf x-b_i)^2=\|\mathbf A\mathbf x-\mathbf b\|^2, \tag{5.8}$$
$\mathbf A=[\mathbf a_1^T;\dots;\mathbf a_N^T]\in\mathbb R^{N\times6}$，$\mathbf b=[b_1;\dots;b_N]\in\mathbb R^N$。最小二乘解（正规方程）：
$$\boxed{\ \mathbf x^\star=(\mathbf A^T\mathbf A)^{-1}\mathbf A^T\mathbf b\ },\qquad
\mathbf A^T\mathbf A=\sum_i\mathbf a_i\mathbf a_i^T\in\mathbb R^{6\times6}. \tag{5.9}$$
$\mathbf A^T\mathbf A$ 是 $6\times6$ 对称正定（满秩时），用 Cholesky 解。

**OpenCV KinFu 的等价矩阵形式**：定义每点的 $3\times6$ 矩阵
$$\mathbf G(\mathbf s_i)=\big[\ \lfloor\mathbf s_i\rfloor_\times^T\ \ \big|\ \ \mathbf I_3\ \big]
=\big[\ \mathrm{skew}(\mathbf s_i)^T\ \ \big|\ \ \mathbf I_3\ \big], \tag{5.10}$$
使位移 $f(\mathbf x,\mathbf s_i)=\mathbf G(\mathbf s_i)\mathbf x$（旋转线性部分 + 平移）。则正规方程写成
$$\Big(\sum_i\mathbf G(\mathbf s_i)^T\,[\mathbf n_i\mathbf n_i^T]\,\mathbf G(\mathbf s_i)\Big)\mathbf x
=\sum_i\mathbf G(\mathbf s_i)^T\,[\mathbf n_i\mathbf n_i^T]\,(-(\mathbf s_i-\mathbf d_i)). \tag{5.11}$$
（其中 $[\mathbf n_i\mathbf n_i^T]$ 是法向投影矩阵——把残差投到法向再平方，正是 point-to-plane 的"只罚法向分量"。展开 (5.11) 与 (5.9) 完全一致：$\mathbf G^T[\mathbf n\mathbf n^T]\mathbf G=(\mathbf n^T\mathbf G)^T(\mathbf n^T\mathbf G)=\mathbf a_i\mathbf a_i^T$，因 $\mathbf n_i^T\mathbf G(\mathbf s_i)=[(\mathbf s_i\times\mathbf n_i)^T\ \mathbf n_i^T]=\mathbf a_i^T$。）

**算法 5.1（point-to-plane ICP 一次迭代）**：

```
输入：source S, target D（带法向 n），当前位姿 (R,t)
1. 对应步：对每个 s_i，变换 s_i' = R s_i + t，KD-tree 在 D 找最近点 d_i，取其法向 n_i
2. （过滤）丢弃 ||s_i' - d_i|| > d_max 或法向夹角过大的对应
3. 组装：a_i = [ (s_i' × n_i)^T , n_i^T ]，b_i = n_i^T (d_i - s_i')
4. 解 6×6 正规方程 (A^T A) x = A^T b （Cholesky），x = [α,β,γ,t_x,t_y,t_z]
5. 由 (α,β,γ) 经 (5.4) 或精确 Euler 重构 ΔR，Δt=(t_x,t_y,t_z)
6. 更新：R ← ΔR · R,  t ← ΔR · t + Δt（左乘增量；或按实现约定）
7. 重复直到 ||x|| 或残差变化 < 阈值
```

**从 $(\alpha,\beta,\gamma)$ 恢复旋转**：解得小角后，可用线性近似 (5.4)，或回代精确 Euler (5.3) 重建 $\Delta\mathbf R$ 再正交化（SVD 投影到 $\mathrm{SO}(3)$）以免累积误差。

> **本质洞察（为何 point-to-plane 收敛快）**：point-to-point 的 Hessian 在切平面方向也有大曲率（罚切向滑移），导致沿曲面"爬行"慢；point-to-plane 的法向投影使切向自由度代价为零、只约束法向，等价于在曲面流形上做了正确的"滑动 + 法向贴合"，故每步走得更远、迭代更少。Rusinkiewicz-Levoy 实测 point-to-plane（及其变体）显著快于 point-to-point。

### 5.4 与本书右扰动的对接（综合）

Low 的 $(\alpha,\beta,\gamma)$ 在小角度下即 $\mathfrak{so}(3)$ 指数坐标 $\delta\boldsymbol\phi$（$\mathbf R\approx\mathbf I+\lfloor\delta\boldsymbol\phi\rfloor_\times$，对照 (5.4)），故 (5.7) 的雅可比行 $[(\mathbf s_i\times\mathbf n_i)^T\ \mathbf n_i^T]$ 直接是本书点到面残差对 $[\delta\boldsymbol\phi;\delta\mathbf t]$ 的雅可比（差一排序：本书平移在前则交换两块）。本书综合时建议用右扰动 + LM，每步线性化即得 (5.9) 同款系统。

---

## 6. ICP 系列之三：广义 ICP（Generalized-ICP / GICP，plane-to-plane）

> 来源：Segal, Haehnel, Thrun, *"Generalized-ICP"*, RSS 2009；PCL `gicp.hpp` 实现（缺省参数与协方差构造）。GICP 把 point-to-point / point-to-plane / plane-to-plane 统一进一个**概率框架**：对每点配一个协方差，残差按对应协方差的 Mahalanobis 距离加权。plane-to-plane（两侧都建局部平面）对错误对应更鲁棒，且 $d_{\max}$ 更易调。

### 6.1 动机：从硬最近邻到概率对应

标准 ICP（point-to-point）算法骨架（GICP 原文 Algorithm 1，与 §4.2 一致）：

```
GICP/ICP 主循环 (Algorithm 1):
输入: 两点云 A={a_i}, B={b_i}, 初值 T_0
T ← T_0
while not converged:
  for i: 
     m_i ← FindClosestPointInA(T·b_i)   // 或 T·a_i ↔ B，按实现
     if ||m_i - T·b_i|| ≤ d_max: w_i ← 1  else w_i ← 0   // 最大匹配距离过滤
  T ← argmin_T { sum_i w_i ·  (距离度量) }                 // 不同度量 → 不同 ICP
return T
```

**三种距离度量**（GICP 的统一点）：
- **point-to-point**：$\sum_i w_i\|\mathbf b_i-\mathbf T\mathbf a_i\|^2$。
- **point-to-plane**：$\sum_i w_i\big(\mathbf n_i\cdot(\mathbf b_i-\mathbf T\mathbf a_i)\big)^2$。
- **GICP（plane-to-plane）**：见下，用 Mahalanobis 距离。

### 6.2 概率生成模型（GICP 核心）

GICP 假设存在"真"对应点的真值位置 $\hat{\mathbf a}_i,\hat{\mathbf b}_i$，在完美对应与完美变换 $\mathbf T^\star$ 下 $\hat{\mathbf b}_i=\mathbf T^\star\hat{\mathbf a}_i$。观测点是真值加各自高斯噪声：
$$\mathbf a_i\sim\mathcal N(\hat{\mathbf a}_i,\ \mathbf C_i^A),\qquad
\mathbf b_i\sim\mathcal N(\hat{\mathbf b}_i,\ \mathbf C_i^B). \tag{6.1}$$
定义**变换下的残差**（在真变换 $\mathbf T^\star$ 处）
$$\mathbf d_i^{(\mathbf T)}=\mathbf b_i-\mathbf T\mathbf a_i. \tag{6.2}$$
在真对应 + 真变换下 $\hat{\mathbf d}_i^{(\mathbf T^\star)}=\hat{\mathbf b}_i-\mathbf T^\star\hat{\mathbf a}_i=\mathbf 0$。由于 $\mathbf a_i,\mathbf b_i$ 独立高斯，且线性变换 $\mathbf T^\star$ 作用于高斯仍是高斯（协方差按 $\mathbf R\mathbf C\mathbf R^T$ 变换，平移不改协方差）：
$$\mathbf d_i^{(\mathbf T^\star)}\sim\mathcal N\!\Big(\hat{\mathbf b}_i-\mathbf T^\star\hat{\mathbf a}_i,\ \ \mathbf C_i^B+\mathbf T^\star\mathbf C_i^A\,\mathbf T^{\star T}\Big)
=\mathcal N\!\Big(\mathbf 0,\ \ \mathbf C_i^B+\mathbf R^\star\mathbf C_i^A\,\mathbf R^{\star T}\Big). \tag{6.3}$$
（推导：$\mathbf b_i\sim\mathcal N(\hat{\mathbf b}_i,\mathbf C_i^B)$，$\mathbf T^\star\mathbf a_i\sim\mathcal N(\mathbf T^\star\hat{\mathbf a}_i,\ \mathbf R^\star\mathbf C_i^A\mathbf R^{\star T})$，二者独立之差的均值相减、协方差相加；平移项在差中抵消故只剩旋转作用于 $\mathbf C_i^A$。）

### 6.3 MLE → GICP 代价（完整推导）

用最大似然估计 $\mathbf T$：
$$\mathbf T^\star=\arg\max_{\mathbf T}\prod_i p\big(\mathbf d_i^{(\mathbf T)}\big)
=\arg\max_{\mathbf T}\sum_i\log p\big(\mathbf d_i^{(\mathbf T)}\big). \tag{6.4}$$
高斯对数似然（记 $\boldsymbol\Lambda_i=\mathbf C_i^B+\mathbf T\mathbf C_i^A\mathbf T^T$）：
$$\log p(\mathbf d_i^{(\mathbf T)})=-\tfrac12\mathbf d_i^{(\mathbf T)T}\boldsymbol\Lambda_i^{-1}\mathbf d_i^{(\mathbf T)}
-\tfrac12\log\big((2\pi)^3\det\boldsymbol\Lambda_i\big). \tag{6.5}$$
GICP 论文**略去 $\det\boldsymbol\Lambda_i$ 项与常数**（视协方差随 $\mathbf T$ 变化对 $\det$ 的影响为二阶小，且实务中忽略以简化），最大化等价于最小化加权平方和：
$$\boxed{\ \mathbf T^\star=\arg\min_{\mathbf T}\sum_i \mathbf d_i^{(\mathbf T)T}\Big(\mathbf C_i^B+\mathbf T\mathbf C_i^A\,\mathbf T^T\Big)^{-1}\mathbf d_i^{(\mathbf T)}\ }. \tag{6.6}$$
这就是 **GICP 代价**：残差 $\mathbf d_i=\mathbf b_i-\mathbf T\mathbf a_i$ 按"两侧协方差经变换合成"的逆（信息矩阵）做 Mahalanobis 加权。记
$$\mathbf M_i=\Big(\mathbf C_i^B+\mathbf R\mathbf C_i^A\mathbf R^T\Big)^{-1}\quad(\text{Mahalanobis / 信息矩阵}), \tag{6.7}$$
则 $\mathbf T^\star=\arg\min_{\mathbf T}\sum_i\mathbf d_i^T\mathbf M_i\mathbf d_i$。

> **关键巧思**：$\mathbf M_i$ 含 $\mathbf T$（经 $\mathbf R\mathbf C_i^A\mathbf R^T$），故每次迭代须用当前 $\mathbf R$ 更新 $\mathbf M_i$。优化用非线性最小二乘（GICP 原文用 BFGS / 共轭梯度，PCL 用 BFGS 在 Euler 参数化上，或 GN/LM）。

### 6.4 三种 ICP 作为特例

由 (6.6) 选取特定协方差即恢复经典 ICP：

**(a) point-to-point**：令 $\mathbf C_i^A=\mathbf 0$（source 点无噪声）、$\mathbf C_i^B=\mathbf I$，则 $\mathbf M_i=\mathbf I$，(6.6) 退化为
$$\mathbf T^\star=\arg\min_{\mathbf T}\sum_i\|\mathbf b_i-\mathbf T\mathbf a_i\|^2 \tag{6.8}$$
即标准 point-to-point ICP (4.1)。

**(b) point-to-plane**：令 $\mathbf C_i^A=\mathbf 0$，$\mathbf C_i^B=\mathbf P_i^{-1}$，其中 $\mathbf P_i$ 是 target 点 $\mathbf b_i$ 处沿法向 $\mathbf n_i$ 的**投影矩阵** $\mathbf P_i=\mathbf n_i\mathbf n_i^T$（秩 1，只在法向有"信息"）。则 $\mathbf M_i=\mathbf P_i=\mathbf n_i\mathbf n_i^T$，
$$\mathbf d_i^T\mathbf M_i\mathbf d_i=\mathbf d_i^T\mathbf n_i\mathbf n_i^T\mathbf d_i=(\mathbf n_i^T\mathbf d_i)^2=\big(\mathbf n_i\cdot(\mathbf b_i-\mathbf T\mathbf a_i)\big)^2, \tag{6.9}$$
即 point-to-plane (5.1)。GICP 论文表述为：把 $\mathbf C_i^B$ 取为"沿法向方差为 0、切平面内方差无穷"的极限（用 $\mathbf C_i^B=\mathrm{diag}$ 在法向 0、切向大，逆后只剩法向投影）。

### 6.5 plane-to-plane 协方差构造（GICP 本体）

GICP 的核心创新：**两侧点云都建局部平面模型**。对每点用 $k$ 个最近邻（PCL 缺省 $k=20$）估局部协方差，再"塑形"为代表局部平面的协方差：沿曲面法向方差极小（$\epsilon$），切平面内方差为 1。

**步骤**：
1. 对点 $\mathbf a_i$（或 $\mathbf b_i$）取 $k$ 近邻，求经验协方差
$$\boldsymbol\Sigma_i=\frac1k\sum_{j=1}^k(\mathbf p_j-\bar{\mathbf p})(\mathbf p_j-\bar{\mathbf p})^T. \tag{6.10}$$
2. 特征分解 $\boldsymbol\Sigma_i=\mathbf U\,\mathrm{diag}(\lambda_1,\lambda_2,\lambda_3)\,\mathbf U^T$（$\lambda_1\ge\lambda_2\ge\lambda_3$，$\mathbf U=[\mathbf u_1,\mathbf u_2,\mathbf u_3]$，$\mathbf u_3$ 即最小特征值方向 = **曲面法向**）。
3. **重塑协方差**：把切平面两方向方差设为 1、法向方差设为 $\epsilon$（小）：
$$\mathbf C_i=\mathbf U\,\mathrm{diag}(\epsilon,\,1,\,1)\,\mathbf U^T
=\epsilon\,\mathbf u_3\mathbf u_3^T+\mathbf u_2\mathbf u_2^T+\mathbf u_1\mathbf u_1^T. \tag{6.11}$$
（即沿法向 $\mathbf u_3$ 方差 $\epsilon$、沿切向 $\mathbf u_1,\mathbf u_2$ 方差 1。PCL 实现写成 $\mathbf C_i=\sum_{n=1}^{2}\mathbf u_n\mathbf u_n^T+\epsilon\,\mathbf u_3\mathbf u_3^T$，其中 $\mathbf u_1,\mathbf u_2$ 是大特征值对应的切向，与 (6.11) 一致——切向 1、法向 $\epsilon$。）

**参数（PCL 缺省）**：`k_correspondences_ = 20`（估协方差的近邻数）、`gicp_epsilon_ = 0.001`（$\epsilon$）、`rotation_epsilon_`、`corr_dist_threshold_`（= $d_{\max}$，最大对应距离）、`max_inner_iterations_`（内层 BFGS 迭代）。

**几何意义**：$\mathbf C_i$ 表示"该点几乎确定落在某局部平面上、但平面内位置不确定"。代入 Mahalanobis (6.7)：$\mathbf M_i=(\mathbf C_i^B+\mathbf R\mathbf C_i^A\mathbf R^T)^{-1}$ → 两侧平面法向**夹角越接近**，残差沿两法向交线方向被强约束（plane-to-plane = 两平面应共面/平行）。这比 point-to-plane（只用单侧法向）更鲁棒：当对应有偏差但两侧都在平面上时，代价对切向滑移不敏感，故 $d_{\max}$ 可设大而不易被错误对应带偏。

### 6.6 GICP 优化与雅可比（PCL 实现）

残差 $\mathbf d_i=\mathbf R\mathbf a_i+\mathbf t-\mathbf b_i$，总代价 $f=\frac1m\sum_i\mathbf d_i^T\mathbf M_i\mathbf d_i$。PCL 用 `OptimizationFunctorWithIndices` 经 Euler ZYX 参数化算梯度（`df`）与（近似）Hessian（`dfddf`）喂给 BFGS。对单项 $e_i=\mathbf d_i^T\mathbf M_i\mathbf d_i$，对平移 $\partial e_i/\partial\mathbf t=2\mathbf M_i\mathbf d_i$；对旋转（右扰动版，本书风格 `\rebuilt`）：$\partial\mathbf d_i/\partial\delta\boldsymbol\phi=-\mathbf R\lfloor\mathbf a_i\rfloor_\times$，故
$$\frac{\partial e_i}{\partial\delta\boldsymbol\phi}=2\,\mathbf d_i^T\mathbf M_i\big(-\mathbf R\lfloor\mathbf a_i\rfloor_\times\big),\qquad
\frac{\partial e_i}{\partial\delta\mathbf t}=2\,\mathbf d_i^T\mathbf M_i. \tag{6.12}$$
（注意 $\mathbf M_i$ 也随 $\mathbf R$ 变，严格梯度含 $\partial\mathbf M_i/\partial\mathbf R$ 项；GICP/PCL 实务中常**冻结 $\mathbf M_i$**（用当前 $\mathbf R$ 算好后视为常数）做高斯-牛顿，是良好近似。）

**算法 6.1（GICP 完整流程）**：

```
输入：A, B, 初值 T_0
预处理：对 A 每点与 B 每点，KD-tree 取 k=20 近邻，按 (6.10)-(6.11) 算 C_i^A, C_i^B
T ← T_0
repeat:
  对应步：对每个 a_i，变换 T·a_i，KD-tree 在 B 找最近邻 b_i（过滤 d_max）
  对每对：M_i = (C_i^B + R C_i^A R^T)^{-1}                 // (6.7)
  优化：T ← argmin_T sum_i (b_i - T a_i)^T M_i (b_i - T a_i)  // BFGS/GN, (6.6)
until 收敛（变换增量 < rotation_epsilon_ / transformation_epsilon_）
输出 T
```

> **本质洞察（GICP 三连）**：GICP = 概率 ICP 的统一框架。`C^A=0,C^B=I` → point-to-point；`C^A=0,C^B=nn^T 的逆极限` → point-to-plane；`两侧都 diag(ε,1,1)` → plane-to-plane。后者对错误对应最鲁棒、对 $d_{\max}$ 最不敏感（论文实测优于前两者）。GICP 与 NDT 都用"高斯/协方差"软化对应，但 GICP 仍需逐点最近邻（KD-tree），NDT 用 voxel 的 cell 高斯免最近邻（见 §5/§6 对比）。

---

## 7. NDT：正态分布变换（Normal Distributions Transform）

> 来源：Biber & Straßer 2003（2D NDT 原始）；Magnusson 2009 博士论文（3D-NDT 完整推导，本节公式编号 6.x 即指该论文）；PCL `ndt.hpp` 实现（常数 $d_1,d_2,d_3$、梯度 Eq 6.12、Hessian Eq 6.13、点雅可比 Eq 6.18-6.19、点 Hessian Eq 6.20-6.21）与 NDT 教程（参数与代码）。NDT 把 target 离散成体素、每体素建一个高斯，配准 = 让 source 点在这些高斯下似然最大；**无须显式点对应**（用 voxel hash $O(1)$ 定位 cell），故通常比 ICP 快。

### 7.1 NDT 表示构造

**步骤**（Biber/Magnusson）：把 target 点云所在空间划分为规则 cell（2D 方格 / 3D 立方体素，PCL 缺省 `setResolution(1.0)` = 1 m）。对每个含 $\ge$ 若干点（建议 $\ge6$）的 cell，由其内点 $\{\mathbf x_k\}_{k=1}^n$ 计算高斯参数：
$$\boldsymbol\mu=\mathbf q=\frac1n\sum_{k=1}^n\mathbf x_k,\qquad
\boldsymbol\Sigma=\frac{1}{n-1}\sum_{k=1}^n(\mathbf x_k-\boldsymbol\mu)(\mathbf x_k-\boldsymbol\mu)^T. \tag{7.1}$$
（Biber 原文用 $\frac1n$；Magnusson/PCL 用无偏 $\frac1{n-1}$。）于是 target 被表示成**分段连续可微的概率密度**：cell 内点 $\mathbf x$ 的"被测概率"
$$p(\mathbf x)\ \propto\ \exp\!\Big(-\tfrac12(\mathbf x-\boldsymbol\mu)^T\boldsymbol\Sigma^{-1}(\mathbf x-\boldsymbol\mu)\Big). \tag{7.2}$$

> **本质洞察**：NDT 把"离散点云"变成"每 cell 一个高斯"的**平滑曲面密度场**——既压缩了表示（每 cell 只存 $\boldsymbol\mu,\boldsymbol\Sigma$），又使配准目标处处可导（可用 Newton 法），且**无须对应**（source 点直接查所在 cell 的高斯）。这是与 ICP 的根本区别。

**协方差正则化**：若 cell 内点近共面/共线，$\boldsymbol\Sigma$ 病态（小特征值近 0）。Magnusson 给出修正：把过小特征值提升（如设 $\lambda_i\leftarrow\max(\lambda_i,\,0.001\lambda_{\max})$）以保 $\boldsymbol\Sigma^{-1}$ 数值稳定——与 GICP 的 $\mathrm{diag}(\epsilon,1,1)$ 思想相通。

### 7.2 离群点混合模型与 score 常数（Magnusson Eq 6.7-6.9）

纯高斯对离群点（不在任何模型 cell 内）惩罚过猛。Magnusson 用**高斯 + 均匀分布混合**：单点的概率近似为
$$\bar p(\mathbf x)=c_1\exp\!\Big(-\tfrac{(\mathbf x-\boldsymbol\mu)^T\boldsymbol\Sigma^{-1}(\mathbf x-\boldsymbol\mu)}{2}\Big)+c_2, \tag{7.3}$$
$c_1$ 为高斯权、$c_2$ 为均匀（离群）项。为便于求导，再用一个高斯**拟合** $-\log\bar p$，得**可微近似**
$$\tilde p(\mathbf x)=-d_1\exp\!\Big(-\frac{d_2}{2}(\mathbf x-\boldsymbol\mu)^T\boldsymbol\Sigma^{-1}(\mathbf x-\boldsymbol\mu)\Big), \tag{7.4}$$
其中常数（PCL `ndt.hpp` 由 `outlier_ratio` 与 cell 分辨率 `resolution` 计算）：
$$c_1=10(1-p_o),\quad c_2=\frac{p_o}{V_{\text{cell}}},\quad d_3=-\ln(c_2), \tag{7.5}$$
$$d_1=-\ln(c_1+c_2)-d_3,\qquad
d_2=-2\ln\!\Big(\frac{-\ln\!\big(c_1 e^{-1/2}+c_2\big)-d_3}{d_1}\Big), \tag{7.6}$$
$p_o$=outlier ratio（缺省约 0.55），$V_{\text{cell}}=\text{resolution}^3$（3D）。（这两条常数公式直接录自 PCL 实现 lines 93-99，对应 Magnusson Eq 6.8。）

> 注：(7.4) 中 $-d_1$ 使 $\tilde p$ 为负、最小化 $\sum\tilde p$ 等价于最大似然；PCL 把 score 定为最小化 $-\sum(\text{likelihood})$，符号一致。

### 7.3 配准 score 函数（Magnusson Eq 6.10）

待求变换参数 $\vec p$（2D：$(t_x,t_y,\phi)$；3D：$(t_x,t_y,t_z,\phi_x,\phi_y,\phi_z)$，Euler）。source 点 $\mathbf x_k$ 变换为
$$\mathbf x_k'=T(\vec p,\mathbf x_k)=\mathbf R(\vec p)\,\mathbf x_k+\mathbf t(\vec p). \tag{7.7}$$
对每个 $\mathbf x_k'$ 查其所在 cell 的 $(\boldsymbol\mu_k,\boldsymbol\Sigma_k)$，**score**（要最大化的似然，或取负后最小化）：
$$s(\vec p)=\sum_{k=1}^N \tilde p(\mathbf x_k')
=-\sum_{k=1}^N d_1\exp\!\Big(-\frac{d_2}{2}\,(\mathbf x_k'-\boldsymbol\mu_k)^T\boldsymbol\Sigma_k^{-1}(\mathbf x_k'-\boldsymbol\mu_k)\Big). \tag{7.8}$$
记 $\mathbf x_k''=\mathbf x_k'-\boldsymbol\mu_k$（已减去 cell 均值的残差），$q_k=\mathbf x_k''^T\boldsymbol\Sigma_k^{-1}\mathbf x_k''$（Mahalanobis 平方）。

### 7.4 梯度（Magnusson Eq 6.12）

对参数 $p_i$ 求偏导（链式法则，$\partial\mathbf x_k''/\partial p_i=\partial\mathbf x_k'/\partial p_i$ 即变换雅可比第 $i$ 列）：
$$g_i=\frac{\partial s}{\partial p_i}
=\sum_{k}d_1 d_2\,\Big(\mathbf x_k''^T\boldsymbol\Sigma_k^{-1}\frac{\partial\mathbf x_k''}{\partial p_i}\Big)
\exp\!\Big(-\frac{d_2}{2}\,\mathbf x_k''^T\boldsymbol\Sigma_k^{-1}\mathbf x_k''\Big). \tag{7.9}$$
（这是 Magnusson Eq 6.12 / PCL `updateDerivatives` 实现：`score_gradient[i] += x''^T (Σ^{-1} ∂x''/∂p_i) · d1·d2·exp(-d2/2 · x''^T Σ^{-1} x'')`。记 $\mathbf c_i=\boldsymbol\Sigma_k^{-1}\,\partial\mathbf x_k''/\partial p_i$，则 $g_i=\sum_k d_1 d_2(\mathbf x_k''^T\mathbf c_i)e^{-d_2 q_k/2}$。）

### 7.5 Hessian（Magnusson Eq 6.13）

二阶偏导（对 $p_i,p_j$）：
$$H_{ij}=\frac{\partial^2 s}{\partial p_i\partial p_j}
=\sum_k d_1 d_2\,e^{-\frac{d_2}{2}q_k}\Big[
-d_2\Big(\mathbf x_k''^T\boldsymbol\Sigma_k^{-1}\tfrac{\partial\mathbf x_k''}{\partial p_i}\Big)
\Big(\mathbf x_k''^T\boldsymbol\Sigma_k^{-1}\tfrac{\partial\mathbf x_k''}{\partial p_j}\Big)
+\mathbf x_k''^T\boldsymbol\Sigma_k^{-1}\tfrac{\partial^2\mathbf x_k''}{\partial p_i\partial p_j}
+\tfrac{\partial\mathbf x_k''^T}{\partial p_j}\boldsymbol\Sigma_k^{-1}\tfrac{\partial\mathbf x_k''}{\partial p_i}
\Big]. \tag{7.10}$$
（Magnusson Eq 6.13 / PCL `updateHessian`：三项分别是 (i) 指数项的二阶交叉（$-d_2\cdot$两个一阶投影之积）、(ii) 变换 Hessian $\partial^2\mathbf x''/\partial p_i\partial p_j$ 经 $\boldsymbol\Sigma^{-1}$ 与残差作用、(iii) 两个一阶雅可比经 $\boldsymbol\Sigma^{-1}$ 内积。）

### 7.6 变换雅可比与 Hessian（点对参数的导数，Magnusson Eq 6.18-6.21）

需要 $\partial\mathbf x_k'/\partial p_i$（雅可比 $\mathbf J_E$，$3\times6$）与 $\partial^2\mathbf x_k'/\partial p_i\partial p_j$（Hessian $\mathbf H_E$）。

**平移部分**：$\partial\mathbf x'/\partial t_x=(1,0,0)^T$ 等，即 $\mathbf J_E$ 前三列 = $\mathbf I_3$（平移对位置的导数）。

**旋转部分**（Euler $\phi_x,\phi_y,\phi_z$，Magnusson Eq 6.18/6.19）：$\partial\mathbf x'/\partial\phi_i=(\partial\mathbf R/\partial\phi_i)\mathbf x$，其中 $\partial\mathbf R/\partial\phi_i$ 是旋转矩阵对各 Euler 角的偏导（含 $\sin/\cos$ 组合）。Magnusson 把这些角度导数**预计算**为常向量 $\mathbf a,\mathbf b,\dots,\mathbf h$（Eq 6.19），乘以点坐标 $(x,y,z)$ 得 $\mathbf J_E$ 的旋转三列。形式（PCL `computeAngleDerivatives`，Euler ZYX，$\phi=\phi_x,\theta=\phi_y,\psi=\phi_z$）：
$$\frac{\partial\mathbf x'}{\partial\phi}=\begin{pmatrix}0\\ a\\ b\end{pmatrix}\!\cdot,\ \
\frac{\partial\mathbf x'}{\partial\theta}=\begin{pmatrix}c\\ d\\ e\end{pmatrix}\!\cdot,\ \
\frac{\partial\mathbf x'}{\partial\psi}=\begin{pmatrix}f\\ g\\ h\end{pmatrix}\!\cdot, \tag{7.11}$$
其中（部分，row a/b 等为 $\sin\phi\sin\psi$ 类组合点乘 $(x,y,z)$）：
$$\begin{aligned}
a&=(-\sin\phi\sin\psi+\cos\phi\sin\theta\cos\psi)x+\cdots,\\
b&=(\cos\phi\sin\psi+\sin\phi\sin\theta\cos\psi)x+\cdots,\quad(\text{余项 row c–h 类似})
\end{aligned} \tag{7.12}$$
（完整 8 个角度雅可比项 a–h 见 Magnusson Eq 6.19 / PCL lines 331-346。）

**旋转二阶导**（Magnusson Eq 6.20/6.21）：$\partial^2\mathbf x'/\partial\phi_i\partial\phi_j=(\partial^2\mathbf R/\partial\phi_i\partial\phi_j)\mathbf x$，预计算为 15 个角度 Hessian 项（a2,a3,b2,b3,c2,c3,d1-d3,e1-e3,f1-f3，Magnusson Eq 6.21 / PCL lines 352-387），存为 $18\times6$ 矩阵 $\mathbf H_E$（每 $3\times1$ 块对应一对参数）。平移的二阶导为 0（平移线性）。

> 注：本书右扰动主线下，旋转用 $\mathfrak{so}(3)$ 指数坐标，雅可比 $\partial\mathbf x'/\partial\delta\boldsymbol\phi=-\mathbf R\lfloor\mathbf x\rfloor_\times$（与 (4.16) 同），比 Magnusson 的 Euler 角导数 a–h 简洁得多。综合时**建议用右扰动版替换 (7.11)-(7.12) 的 Euler 雅可比**，Hessian 项也随之大幅简化（仅含 $\lfloor\cdot\rfloor_\times$ 与二阶 BCH 修正）。

### 7.7 Newton 优化（Magnusson Algorithm 2）

每次迭代解线性系统求下降方向 $\Delta\vec p$：
$$\mathbf H\,\Delta\vec p=-\mathbf g, \tag{7.13}$$
$$\vec p\leftarrow\vec p+\eta\,\Delta\vec p, \tag{7.14}$$
$\eta$ 为线搜索步长（PCL 用 **More-Thuente 线搜索**保证充分下降，`setStepSize` 设上限）。若 $\mathbf H$ 非正定（远离极小时可能），把 $\mathbf H$ 加正则 $\mathbf H+\lambda\mathbf I$（或投影到正定）以保 $\Delta\vec p$ 为下降方向。

**算法 7.1（NDT 配准，Newton 法）**：

```
输入：target 点云（建 NDT：每 cell 算 μ,Σ）, source 点云, 初值 p_0
p ← p_0
repeat:
  s ← 0; g ← 0; H ← 0
  for 每个 source 点 x_k:
     x_k' = T(p, x_k)                       // (7.7)
     找 x_k' 所在 cell → (μ_k, Σ_k)         // voxel hash, O(1), 无须最近邻!
     x'' = x_k' - μ_k;  q = x''^T Σ_k^{-1} x''
     算变换雅可比 J_E、Hessian H_E（(7.11)-(7.12) 或右扰动版）
     s += -d1 exp(-d2/2 q)                  // (7.8)
     g += 梯度项                            // (7.9)
     H += Hessian 项                        // (7.10)
  解 H Δp = -g                              // (7.13)
  线搜索 η（More-Thuente）；p ← p + η Δp     // (7.14)
until ||Δp|| < transformation_epsilon
输出 p（最终变换）
```

**PCL NDT 参数与缺省**（来自官方教程）：

| 参数 | 含义 | 例值 |
|---|---|---|
| `setTransformationEpsilon` | 迭代终止阈值：变换增量 $[x,y,z,\text{roll},\text{pitch},\text{yaw}]$ 的最小允许变化（米与弧度） | 0.01 |
| `setStepSize` | More-Thuente 线搜索的最大步长 | 0.1 |
| `setResolution` | 内部 NDT 体素栅格分辨率（最依赖尺度的参数；须足够大使每 cell $\ge6$ 点） | 1.0 |
| `setMaximumIterations` | 优化最大迭代次数 | 35 |

**PCL NDT 数值例**（`room_scan1/2.pcd`）：target 112586 点、source 112624 点，source 经 ApproximateVoxelGrid（leaf 0.2 m）降到 12433 点；NDT 收敛，fitness score 0.638694。

**完整 PCL NDT 代码**：
```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

// 载入 target / source
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan1.pcd", *target_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan2.pcd", *input_cloud);

// 降采样 source（加速）
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
approximate_voxel_filter.setInputCloud(input_cloud);
approximate_voxel_filter.filter(*filtered_cloud);

// 配置 NDT
pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
ndt.setTransformationEpsilon(0.01);
ndt.setStepSize(0.1);
ndt.setResolution(1.0);
ndt.setMaximumIterations(35);
ndt.setInputSource(filtered_cloud);
ndt.setInputTarget(target_cloud);

// 初值（如来自里程计）
Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

// 配准
pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
ndt.align(*output_cloud, init_guess);
std::cout << "converged: " << ndt.hasConverged()
          << " score: " << ndt.getFitnessScore() << std::endl;
pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
```

### 7.8 2D NDT（Biber 2003 原始）

Biber 原文是 2D（激光 scan 匹配）：参数 $\vec p=(t_x,t_y,\phi)$，变换 $\mathbf x'=\mathbf R(\phi)\mathbf x+\mathbf t$，$\mathbf R(\phi)=\begin{pmatrix}\cos\phi&-\sin\phi\\\sin\phi&\cos\phi\end{pmatrix}$。score $\,s(\vec p)=\sum_i\exp\!\big(-\tfrac12(\mathbf x_i'-\boldsymbol\mu_i)^T\boldsymbol\Sigma_i^{-1}(\mathbf x_i'-\boldsymbol\mu_i)\big)$（Biber 用正号最大化，无离群混合常数）。变换雅可比（$2\times3$）：
$$\mathbf J=\frac{\partial\mathbf x'}{\partial\vec p}
=\begin{pmatrix}1 & 0 & -x\sin\phi-y\cos\phi\\ 0 & 1 & x\cos\phi-y\sin\phi\end{pmatrix}, \tag{7.15}$$
第三列即 $\partial(\mathbf R\mathbf x)/\partial\phi$。Newton 更新同 (7.13)-(7.14)。Biber 实测在室内无里程计可实时建图。

### 7.9 P2D-NDT 与 D2D-NDT（Magnusson / Stoyanov）

- **P2D-NDT**（point-to-distribution，上述）：source 是**离散点**，target 是 NDT（分布）；score = source 点在 target 高斯下的似然 (7.8)。
- **D2D-NDT**（distribution-to-distribution，Stoyanov et al. 2012）：**两侧都建 NDT**；source cell $i$（$\boldsymbol\mu_i^A,\boldsymbol\Sigma_i^A$）与 target cell（$\boldsymbol\mu_j^B,\boldsymbol\Sigma_j^B$）的相似度用两高斯的 $L_2$ 距离 / 期望似然度量。D2D score（核心项）：
$$s_{\text{D2D}}=-\sum_{i}d_1\exp\!\Big(-\frac{d_2}{2}\,\boldsymbol\mu_{ij}^T(\mathbf R\boldsymbol\Sigma_i^A\mathbf R^T+\boldsymbol\Sigma_j^B)^{-1}\boldsymbol\mu_{ij}\Big), \tag{7.16}$$
其中 $\boldsymbol\mu_{ij}=\mathbf R\boldsymbol\mu_i^A+\mathbf t-\boldsymbol\mu_j^B$（两均值经变换后之差），合成协方差 $\mathbf R\boldsymbol\Sigma_i^A\mathbf R^T+\boldsymbol\Sigma_j^B$。

> **本质洞察（D2D-NDT ≈ GICP 的 NDT 版）**：(7.16) 与 GICP 代价 (6.6) 形式高度一致——都是"两侧协方差经变换合成的逆 (Mahalanobis) 加权两侧均值之差"。区别：GICP 逐点最近邻 + 逐点协方差；D2D-NDT 用 cell 均值/协方差 + cell 对应（voxel hash），免最近邻、更快。NDT 与 GICP 殊途同归地用"协方差软化"提升鲁棒性。

### 7.10 NDT vs ICP（综合对比表）

| 维度 | ICP (point-to-point/plane) | GICP (plane-to-plane) | NDT (P2D/D2D) |
|---|---|---|---|
| 对应 | 显式最近邻（KD-tree，每迭代 $O(N\log N)$） | 显式最近邻 + 逐点协方差 | 隐式（voxel hash $O(1)$，无最近邻） |
| 优化 | 闭式（p2p）/ 线性 LS（p2pl）/ 非线性 | 非线性（BFGS/GN，Mahalanobis） | Newton（解析梯度/Hessian + 线搜索） |
| 收敛速度 | p2p 慢、p2pl 快 | 中（鲁棒） | 快（连续可微、免对应） |
| 初值依赖 | 强（局部极小） | 强 | 强（但比 ICP 略宽，常 coarse-to-fine） |
| 鲁棒性（错对应） | p2p 弱、p2pl 中 | 强（$d_{\max}$ 易调） | 中-强（cell 高斯平滑 + 离群混合） |
| 内存 | 存两点云 + KD-tree | + 逐点协方差 | 存 NDT（每 cell μ,Σ，压缩） |
| 适用 | 通用、精配准 | 结构化、LiDAR | 大规模、实时、室内/城市 |

---

## 8. ICP 变体分类法（Rusinkiewicz-Levoy 2001）

> 来源：Rusinkiewicz & Levoy, *"Efficient Variants of the ICP Algorithm"*, 3DIM 2001。把 ICP 拆成六个可独立替换的阶段，是理解/调优 ICP 的标准框架。

**ICP 六阶段分类**（每阶段可独立选择，组合出各种 ICP 变体）：

1. **Selection（点选取）**：选用哪些点参与配准。
   - 全部点；均匀子采样（uniform subsampling）；随机采样（random，每迭代重采）；**法向空间均匀采样（normal-space sampling）**——按法向方向均匀选点，使各方向约束充分（Rusinkiewicz 的关键贡献，对"近平面带小特征"如刻字曲面收敛更好）；按颜色/强度梯度选。
2. **Matching（对应建立）**：source 点匹配到 target 的哪个点/面。
   - 最近点（KD-tree）；沿法向投影找交点（normal shooting）；反向校准（reverse calibration，投影到 range image）；可加颜色/法向相容性约束。
3. **Weighting（对应加权）**：给每对对应赋权。
   - 等权；按点对距离 $w\propto1/(1+d)$；按法向相容性 $w\propto\mathbf n_a\cdot\mathbf n_b$；按采集不确定度（扫描仪噪声模型）。
4. **Rejection（对应剔除）**：剔除坏对应。
   - 距离 $>d_{\max}$ 剔除；剔除最差 $x\%$（如 trimmed ICP）；法向夹角过大剔除；剔除落在网格边界的对应（避免部分重叠的伪匹配）；剔除"不一致"对（双向最近邻不互为最近）。
5. **Error metric（误差度量）**：配准目标。
   - **point-to-point**（§4）；**point-to-plane**（§5）；point-to-point + 颜色；**plane-to-plane / GICP**（§6）；LM 优化非线性度量。
6. **Minimization（最小化）**：求解变换。
   - 闭式（SVD/四元数，p2p）；线性 LS（p2pl 小角，§5）；非线性（LM，可对任意度量）；随机梯度。

**Rusinkiewicz-Levoy 的高速组合**：normal-space sampling + point-to-plane + 投影匹配 + 常数权 + 距离剔除 + 线性最小化 → 在近平面带小特征的网格上收敛最快。**实测结论**：(a) point-to-plane 比 point-to-point 收敛快得多；(b) 对应策略（matching/sampling）对收敛速度影响最大；(c) 随机重采样 + point-to-plane 鲁棒高效。

> **本质洞察**：ICP 不是单一算法而是**算法族**——六阶段每格选不同方法组合出几十种 ICP。调优 ICP = 针对数据特性（噪声/重叠/平面度/初值质量）在六阶段做选择。本书综合时按此框架组织"ICP 工程实践"小节。

---

## 9. 特征提取：LOAM 边/面特征（点云特征视角，自包含补述）

> **交叉引用**：LOAM 的完整抽取（曲率公式、点到线/点到面距离、scan-to-scan / scan-to-map 双算法、运动畸变补偿、LM 求解）已在 `extractions/lidar_slam__loam_family.md` §LOAM 完整记录。本节仅从**点云特征提取**视角补述其核心公式，使"点云处理"章自包含；不重复 LOAM 的里程计/建图全推导。

### 9.1 局部曲面光滑度（曲率）c（LOAM Eq 1）

LOAM 在每条 scan line（range image 同一 ring）上，对点 $i$ 取其同一行左右各若干连续点 $\mathcal S$（LeGO-LOAM 用 $|\mathcal S|=10$，两侧各 5），定义**光滑度 / 曲率**
$$c=\frac{1}{|\mathcal S|\cdot\|\mathbf X^L_{(k,i)}\|}\Big\|\sum_{j\in\mathcal S,\,j\ne i}\big(\mathbf X^L_{(k,i)}-\mathbf X^L_{(k,j)}\big)\Big\|, \tag{9.1}$$
$\mathbf X^L_{(k,i)}$ 为点 $i$ 在 LiDAR 系坐标。$c$ 度量局部点偏离同行邻居平均的程度：
- $c$ **大** → 局部不光滑 → **边缘/角点特征**（edge point）。
- $c$ **小** → 局部光滑 → **平面特征**（planar point）。
按阈值 $c_{th}$ 分类，并按 $c$ 排序、分区域选取（每行分若干扇区，各取若干最大 $c$ 为边、若干最小 $c$ 为面），保证特征空间分布均匀（不扎堆）。

### 9.2 点到边缘线距离 d_E（LOAM Eq 2）

边缘特征点 $\tilde{\mathbf X}^L_{(k+1,i)}$ 与上一帧地图中两点 $(\bar{\mathbf X}^L_{(k,j)},\bar{\mathbf X}^L_{(k,l)})$ 定义的边缘线，点到线距离
$$d_{\mathcal E}=\frac{\big|(\tilde{\mathbf X}_{(k+1,i)}-\bar{\mathbf X}_{(k,j)})\times(\tilde{\mathbf X}_{(k+1,i)}-\bar{\mathbf X}_{(k,l)})\big|}{\big|\bar{\mathbf X}_{(k,j)}-\bar{\mathbf X}_{(k,l)}\big|}. \tag{9.2}$$
（分子=三点张成平行四边形面积，分母=底边长 → 高 = 点到直线距离。）

### 9.3 点到平面距离 d_H（LOAM Eq 3）

平面特征点与地图中三点 $(j,l,m)$ 定义的平面，点到面距离
$$d_{\mathcal H}=\frac{\big|(\tilde{\mathbf X}_{(k+1,i)}-\bar{\mathbf X}_{(k,j)})\cdot\big[(\bar{\mathbf X}_{(k,j)}-\bar{\mathbf X}_{(k,l)})\times(\bar{\mathbf X}_{(k,j)}-\bar{\mathbf X}_{(k,m)})\big]\big|}{\big|(\bar{\mathbf X}_{(k,j)}-\bar{\mathbf X}_{(k,l)})\times(\bar{\mathbf X}_{(k,j)}-\bar{\mathbf X}_{(k,m)})\big|}. \tag{9.3}$$
（分子=三向量混合积绝对值=体积，分母=底面三角形法向模 → 高 = 点到平面距离；叉积 $(\cdots)\times(\cdots)$ 即平面法向。）

LOAM 把所有 $d_{\mathcal E},d_{\mathcal H}$ 作残差，用 LM 求解 6-DOF 位姿（详见 `lidar_slam__loam_family.md`）。注意 $d_{\mathcal H}$ 与点到面 ICP (5.1) 同源——分母归一化后 $d_{\mathcal H}=\mathbf n^T(\mathbf p-\mathbf p_{\text{plane}})$，正是 point-to-plane 残差。

> **本质洞察（特征 ICP = LOAM）**：LOAM 本质是"特征点 + point-to-line / point-to-plane 的 ICP"——先用曲率 (9.1) 把海量点云压成稀疏边/面特征（降算力、抗噪），再对特征做点到线 (9.2) / 点到面 (9.3) 配准。这把 §5 的 point-to-plane ICP 与 §3 的 KD-tree 近邻、§2 的体素降采样全部串起来，是点云处理在 LiDAR SLAM 的集大成应用。

---

## 10. 全章公式速查（综合便检）

| 主题 | 关键式 | 编号 |
|---|---|---|
| 体素质心 | $\bar{\mathbf p}_V=\frac1n\sum\mathbf p_i$ | (2.1) |
| SOR 判据 | $\bar d_i>\mu+\alpha\sigma$ 剔除 | (2.4) |
| ROR 判据 | 半径 $r$ 内邻居 $<k_{\min}$ 剔除 | (2.5) |
| KD-tree 建树 | 中位数分割，$O(n\log n)$ | 算法 3.1 |
| KD-tree NN 剪枝 | $|q_{\text{axis}}-\text{node}_{\text{axis}}|<r_{\text{best}}$ 才搜远侧 | 算法 3.2 |
| ICP 代价 (p2p) | $E=\sum\|\mathbf R\mathbf a_i+\mathbf t-\mathbf b_i\|^2$ | (4.1) |
| ICP 收敛 | 单调收敛到局部极小 | 定理 4.1 |
| 平移闭式 | $\mathbf t^\star=\bar{\mathbf y}-\mathbf R^\star\bar{\mathbf a}$ | (4.6) |
| 旋转 SVD 解 | $\mathbf R^\star=\mathbf V\,\mathrm{diag}(1,1,\det\mathbf V\mathbf U^T)\,\mathbf U^T$，$\mathbf H=\sum\mathbf a'\mathbf y'^T$ | (4.9)-(4.10) |
| 旋转四元数解 | $\mathring{\mathbf q}=\arg\max$ 特征值 of $\mathbf N$（4×4） | 定理 4.3, (4.13) |
| p2p 右扰动雅可比 | $\partial\mathbf r/\partial\delta\boldsymbol\phi=-\mathbf R\lfloor\mathbf a\rfloor_\times$ | (4.16) |
| 点到面代价 | $E=\sum(\mathbf n_i^T(\mathbf R\mathbf s_i+\mathbf t-\mathbf d_i))^2$ | (5.1) |
| 点到面线性化行 | $\mathbf a_i=[(\mathbf s_i\times\mathbf n_i)^T,\ \mathbf n_i^T]$ | (5.7) |
| 点到面正规方程 | $\mathbf x=(\mathbf A^T\mathbf A)^{-1}\mathbf A^T\mathbf b$，6×6 | (5.9) |
| GICP 残差分布 | $\mathbf d_i\sim\mathcal N(\mathbf 0,\mathbf C_i^B+\mathbf R\mathbf C_i^A\mathbf R^T)$ | (6.3) |
| GICP 代价 | $\min\sum\mathbf d_i^T(\mathbf C_i^B+\mathbf T\mathbf C_i^A\mathbf T^T)^{-1}\mathbf d_i$ | (6.6) |
| GICP 平面协方差 | $\mathbf C_i=\mathbf U\,\mathrm{diag}(\epsilon,1,1)\,\mathbf U^T$，$\epsilon=0.001$ | (6.11) |
| NDT cell 高斯 | $\boldsymbol\mu,\boldsymbol\Sigma$ by (7.1)，$p\propto e^{-\frac12(\mathbf x-\boldsymbol\mu)^T\boldsymbol\Sigma^{-1}(\mathbf x-\boldsymbol\mu)}$ | (7.1)-(7.2) |
| NDT score | $s=-\sum d_1 e^{-\frac{d_2}2 q_k}$ | (7.8) |
| NDT 梯度 | $g_i=\sum d_1 d_2(\mathbf x''^T\boldsymbol\Sigma^{-1}\partial\mathbf x''/\partial p_i)e^{-d_2 q/2}$ | (7.9) |
| NDT Hessian | 三项（指数二阶 + 变换 Hessian + 雅可比内积） | (7.10) |
| NDT Newton | $\mathbf H\Delta\vec p=-\mathbf g$ | (7.13) |
| D2D-NDT | $s=-\sum d_1 e^{-\frac{d_2}2\boldsymbol\mu_{ij}^T(\mathbf R\boldsymbol\Sigma^A\mathbf R^T+\boldsymbol\Sigma^B)^{-1}\boldsymbol\mu_{ij}}$ | (7.16) |
| LOAM 曲率 | $c=\frac{1}{|\mathcal S|\|\mathbf X\|}\|\sum(\mathbf X_i-\mathbf X_j)\|$ | (9.1) |
| LOAM 点到线 | $d_{\mathcal E}$ = 叉积/底边 | (9.2) |
| LOAM 点到面 | $d_{\mathcal H}$ = 混合积/法向模 | (9.3) |

---

## 11. 与本书统一约定的转换要点汇总（综合 agent 备查）

1. **source/target 方向**：本文统一 source=动=$\mathbf a/\mathbf s$、target=静=$\mathbf b/\mathbf q$，变换 $\mathbf R\mathbf a+\mathbf t\to\mathbf b$。PCL `setInputSource/Target` 与此一致；Besl-McKay "data→model"、Horn "left→right" 亦对应。
2. **SVD 闭式解的转置陷阱**：本文/Arun $\mathbf H=\sum\mathbf a'\mathbf y'^T\Rightarrow\mathbf R=\mathbf V\mathbf U^T$；Wikipedia-Kabsch $\mathbf H=\mathbf P^T\mathbf Q\Rightarrow\mathbf R=\mathbf U\,\mathrm{diag}(1,1,d)\mathbf V^T$（行向量约定，右乘）。移植务必核对 $\mathbf H$ 定义、行/列、det 修正位置。
3. **扰动方向**：Low 的 Euler 小角 $(\alpha,\beta,\gamma)$、OpenCV 的 skew 向量、Magnusson 的 Euler 角，小角度下均 $\leftrightarrow$ 本书右扰动 $\delta\boldsymbol\phi$（$\mathbf R\approx\mathbf I+\lfloor\delta\boldsymbol\phi\rfloor_\times$）。本书综合时统一用右扰动 + $\partial\mathbf x'/\partial\delta\boldsymbol\phi=-\mathbf R\lfloor\mathbf x\rfloor_\times$，可大幅简化 NDT 的 Euler 雅可比 (7.11)-(7.12) 与 Hessian。
4. **未知量排序**：Low/OpenCV/Magnusson 多为"旋转在前、平移在后"$[\boldsymbol\phi;\mathbf t]$，本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）——重排雅可比行/列块与正规方程。
5. **协方差字母**：GICP 用 $\mathbf C$、Biber 用 $\mathbf S$、本书用 $\boldsymbol\Sigma$；信息矩阵 $\boldsymbol\Sigma^{-1}$。NDT 的 $\mathbf R$ 测量噪声 vs 本书 $\mathbf R\in\mathrm{SO}(3)$ 须显式区分（NDT 文中 $\mathbf R$ 多指旋转，无歧义）。
6. **四元数**：Besl/Horn 实部在前 $(q_0,\mathbf q_v)$，与本书 Hamilton 一致；$q_0\ge0$ 取主值。
7. **score 符号**：NDT 论文最大化似然 vs PCL 最小化负似然，差一负号 + $d_1$ 正负；本书统一为"最小化负对数似然"，按 (7.8) 取 $s=-\sum d_1 e^{(\cdot)}$（$d_1>0$ 则 $s<0$，最小化 $s$ = 最大化似然）。

---

*（抽取完毕。所有公式按权威来源转写并交叉印证；涉及版权 PDF（GICP/Low/Magnusson）处记录的是其数学事实与方法步骤，并以多源印证以保转写正确。凡编号细节存疑处已在正文标注；NDT 的 Euler 雅可比/Hessian 具体角度项（Magnusson Eq 6.19/6.21 的 a–h、15 项二阶）因原文 PDF 超 10 MB 无法整篇精读，仅录其结构与 PCL 实现对应、关键项形式，综合时如需逐项展开建议改用本书右扰动版重推（更简且自洽）——已在 §7.6 给出替换方案。）*
