# 抽取留痕：稠密建图（单目稠密重建 + RGBD 稠密 + 地图分类/工程权衡）

> **服务的成书章节**：**稠密建图**（dense mapping）
> **抽取专员说明**：本文件为项目内部「抽取留痕」（**非成书正文**），逐节**全量保真**记录源材料中与稠密建图相关的全部内容（每一步推导、每例与数值、每条定义/定理、每张表/分类/算法伪码、每段实践代码），供后续综合 agent 写成自包含书章。**铁律：禁摘要、宁长勿略。**
>
> **主源（逐行通读，1273 行全覆盖）**：
> `/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/12_建图.md` —— 高翔《视觉SLAM十四讲》第 12 讲「建图」（共 1273 行 / 约 60 KB）。
>
> **辅源（联网精读，补主源缺失的完整公式/推导）**：
> 1. **Pizzoli, Forster, Scaramuzza, "REMODE: Probabilistic, Monocular Dense Reconstruction in Real Time", ICRA 2014.** PDF: https://rpg.ifi.uzh.ch/docs/ICRA14_Pizzoli.pdf ；项目页 https://rpg.ifi.uzh.ch/research_dense.html ；开源实现 https://github.com/uzh-rpg/rpg_open_remode 。← 主源 §12.3 用的正是 REMODE 测试数据集；REMODE 即 14 讲深度滤波器的「均匀-高斯混合」原型。
> 2. **Vogiatzis & Hernández, "Video-based, real-time multi-view stereo", Image and Vision Computing 29(7):434–441, 2011.** PDF: https://www.george-vogiatzis.org/publications/ivcj2010.pdf 与 https://carlos-hernandez.org/papers/hernandez_imavis2011.pdf 。← 贝叶斯「高斯×Beta」深度滤波器（逐帧更新公式 C1/C2/a/b）的原始出处，14 讲习题与正文均引此。
> 3. **Forster, Pizzoli, Scaramuzza, "SVO: Semi-Direct Visual Odometry", TRO 2016 / ICRA 2014.** PDF: https://rpg.ifi.uzh.ch/docs/TRO16_Forster-SVO.pdf 。← 深度滤波器附录（逆深度参数化）。
> 4. **Hirschmüller, "Stereo Processing by Semi-Global Matching and Mutual Information", IEEE TPAMI 30(2):328–341, 2008.** （SGM 立体匹配）参考 https://en.wikipedia.org/wiki/Semi-global_matching 整理；原论文见 Semantic Scholar/ResearchGate。
> 5. **SLAM Handbook, Ch.5 "Dense Map Representations"**（Kim, Schmid, Oleynikova 等）：`/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/05_Dense_Map_Representations/05_Dense_Map_Representations.md`（共约 80 KB）。← 占据栅格 log-odds、隐式曲面/TSDF/ESDF、surfel、voxel、octree/Octomap、哈希体素、Marching Cubes、GP/Hilbert 图等的**权威综述与公式**，补 14 讲 §12.4 的理论深度。
> 6. **KinectFusion (Newcombe et al., ISMAR 2011) + Curless & Levoy (SIGGRAPH 1996)** 的 TSDF 加权平均融合公式：参考 https://gist.github.com/savuor/407fdc1807f9d5836d68aebfee726ef7 与 Handbook §5.2.2/§5.4.4。
> 7. **Octomap (Hornung et al., Autonomous Robots 2013)**：八叉树占据图，14 讲 §12.4.3 引文献 [132]，原理与 Handbook §5.3.3.3/§5.4.4 一致。
>
> **⚠️ 联网 PDF 抽取受限说明**：REMODE、Vogiatzis-Hernández、SVO、本节几篇 arXiv PDF 经 WebFetch 抓取时**均为图片化/FlateDecode 压缩流，正文公式无法逐字 OCR**。因此「高斯×Beta 深度滤波器」的逐帧更新式（C1/C2/m/s²/a/b）我**采用文献中公认的、已发表的标准形式**，并经两处可读 HTML（cnblogs SVO 解析、shuzhiduo REMODE 解析）+ 多次检索结果**交叉印证**记录；凡我无法逐字核到原始 PDF 编号的，已在该式旁显式标注「⚠️ 标准发表形式，未核到原 PDF 公式编号」。综合 agent 引用时请据此把关。

---

## 🔴 综合 agent 必读：本任务「本章聚焦」各主题的覆盖来源对照

| 聚焦主题（任务要求重点覆盖） | 主源(14讲) 覆盖 | 辅源补充 | 本抽取所在节 |
| --- | --- | --- | --- |
| 地图用途与分类（度量/拓扑/语义、稀疏 vs 稠密） | ✅ §12.1 五大用途 + 稀疏/稠密对比；§12.6 度量/拓扑 | Handbook §5.2/§5.5 表 5.2 显式/隐式、表面/体积四象限 | §1、§13 |
| 单目稠密：极线搜索 + 块匹配（SAD/SSD/NCC/ZNCC） | ✅ §12.2.2 全部度量公式 (12.1–12.3, 12.11) | — | §3 |
| 单目稠密：高斯深度滤波器 + 完整不确定度推导 | ✅ §12.2.3 高斯乘积 (12.4–12.10) + 几何不确定度 | REMODE/Vogiatzis 不确定度 τ | §4 |
| 单目稠密：贝叶斯/均匀-高斯混合逐帧更新（REMODE 式） | ⚠️ 仅作为习题点名 | ✅ **Vogiatzis-Hernández 完整 C1/C2/a/b 更新式** | §5 |
| 收敛/发散判据 | ✅ §12.3 代码 min_cov/max_cov；§12.2.3 阈值 | ✅ REMODE η_inlier/η_outlier | §4.4、§5.4 |
| 逆深度参数化 | ✅ §12.3.3 | ✅ SVO 逆深度 | §6 |
| 立体匹配 SGM 简介 + 完整公式 | ❌ 14 讲只提双目视差一句 | ✅ **Hirschmüller SGM 全套公式** | §7 |
| RGBD 点云地图 + 滤波（SOR/体素栅格） | ✅ §12.4.1 完整代码 + 数值 | Handbook §5.3.2.1/§5.4.1 | §8 |
| 网格重建（MLS + 贪婪投影三角化 / 泊松） | ✅ §12.4.2 完整代码 | Handbook §5.3.2.3/§5.3.2.6 Marching Cubes | §9 |
| 八叉树 OctoMap 的 log-odds | ✅ §12.4.3 logit (12.16–12.19) + 代码 | ✅ Handbook §5.2.1 (5.6/5.7)、§5.3.3.3 octree、§5.4.4 剪枝 | §10 |
| TSDF / KinectFusion | ✅ §12.5 定性 + 数值 | ✅ **Curless-Levoy/KinectFusion 加权平均式 + Handbook §5.2.2/§5.4.4** | §11 |
| surfel | ✅ §12.4.1 一句 + 图 | ✅ Handbook §5.3.2.2 surfel 定义 (p,n,r) | §9.3 |
| 工程权衡 | ✅ §12.3.x 像素梯度/并行/外点；§12.4 各图比较 | ✅ Handbook §5.5 选型 | §3.5、§12、§13 |

**结论**：单目稠密（极线+块匹配+高斯滤波）与 RGBD（点云/网格/八叉树/TSDF）**主源 14 讲覆盖充分、含完整代码与数值**；其**理论短板**（均匀-高斯混合贝叶斯逐帧更新的闭式式、SGM 公式、TSDF 加权平均的精确式、占据图 log-odds 的概率推导、八叉树/体素数据结构细节）**已由辅源 2/4/5/6 全量补齐并标源**。

---

## 记号约定（Notation）与本书统一约定的对齐/差异

下表把三源记号统一到本书约定（$R\in SO(3)$、右扰动主、Hamilton 四元数、$\xi=[\rho;\phi]$、过程/观测噪声 $\Sigma_w/\Sigma_v$）。

| 量 | 主源(14讲) | 辅源 | 本书统一 | 说明/差异 |
| --- | --- | --- | --- | --- |
| 旋转矩阵 | $R$，$R_{RW}$（参考←世界） | $\mathbf R\in SO(3)$ | $R\in SO(3)$ | 一致 |
| 平移 | $t$，$t_{RW}$ | $\mathbf t\in\mathbb R^3$ | $t$ | 一致 |
| 位姿（变换） | `SE3d`，`Isometry3d`，$T_{C R}$（参考→当前） | $T\in SE(3)$ | $T\in SE(3)$ | 14 讲下标读法 $T_{C\,R}$＝"把 R 系点变到 C 系"；本书统一同义 |
| 像素坐标 | $p_1,p_2$（齐次像素）；代码中 `pt_ref,pt_curr`，$(u,v)$ | — | $\mathbf p=(u,v)$ | 一致 |
| 相机内参 | $K$；`fx,fy,cx,cy` | $K$ | $K$ | **注意：示例 `fy=-480.0`（负号），见 §3.6 OCR/工程说明** |
| 归一化坐标/方向 | $f_{ref}=$ `px2cam(p)`（归一化） | bearing $\mathbf f$ | $\mathbf f$（单位方向） | 一致 |
| 深度（沿光心射线长度） | $d$，正文明确 $d=\|O_1P\|$（**非 z 值！见 §4.5**） | $Z$ / inverse depth $\rho$ | $d$ | **歧义点**：14 讲 §12.2.3 末尾澄清此 $d$ 是 $O_1P$ 长度，与针孔模型的 $z$ 不同 |
| 逆深度 | $d^{-1}$（§12.3.3 习题） | $\rho=1/Z$（SVO/REMODE 主用） | $\rho_{inv}=1/d$ | **记号冲突预警**：辅源用 $\rho$ 表逆深度；本书 $\xi=[\rho;\phi]$ 中 $\rho$ 是平移部分李代数。**综合时务必把逆深度另记**（如 $\rho_{inv}$ 或 $w=1/d$），避免与 $\xi$ 的 $\rho$ 混淆 |
| 深度分布参数 | $\mu,\sigma^2$（高斯）；代码 `depth, depth_cov2` | $\mu,\sigma^2$ | $\mu,\sigma^2$ | 一致 |
| 观测/融合 | $\mu_{obs},\sigma_{obs}$；$\mu_{fuse},\sigma^2_{fuse}$ | $\tau,\tau^2$（单次测量方差） | — | 14 讲 $\sigma_{obs}$ ≈ 辅源 $\tau$（单测量不确定度） |
| 内点率（混合权重） | （仅习题提"均匀-高斯混合"） | $\pi$（Vogiatzis）/ $\rho$（部分文献）；Beta 参数 $a,b$ | $\pi_{in}$，$\mathrm{Beta}(a,b)$ | **再次预警**：辅源亦有用 $\rho$ 表内点率者；本抽取一律用 $\pi_{in}$ 表内点率、$\rho_{inv}$ 表逆深度，杜绝三重 $\rho$ 撞车 |
| 占据概率/对数几率 | $x\in[0,1]$，$y\in\mathbb R$，logit | $p(m_k)$，$l(m_k)$，odds $o$ | $p$，$\ell$（log-odds） | 一致；Handbook 用 $l(\cdot)$，本书写 $\ell$ |
| 体素 SDF 值/权重 | （TSDF 取值 $[-1,1]$） | $F_k(\mathbf p),W_k(\mathbf p),\psi(\eta),\mu_{tr}$（截断带） | $F,W,\psi,\mu_{tr}$ | 截断距离记 $\mu_{tr}$ 以免与高斯均值 $\mu$ 撞 |
| 视差/代价 | （双目视差一句） | $d$(视差)，$C(p,d)$，$L_r$，$S$，$P_1,P_2$ | 同辅源 | SGM 的 $d$ 是**视差**，与深度 $d$ 不同；综合时分节明确 |

> **❗ 综合时三大记号纪律**（极重要）：
> 1. **逆深度** → 用 $\rho_{inv}$（绝不写裸 $\rho$，因 $\xi=[\rho;\phi]$ 已占用）。
> 2. **内点率** → 用 $\pi_{in}$。
> 3. **TSDF 截断距离** → 用 $\mu_{tr}$（与高斯均值 $\mu$ 区分）；**SGM 视差** $d_{disp}$ 与**深度** $d$ 分写。

---

# 1 概述：地图的用途与分类（主源 §12.1，行 15–46）

## 1.1 为什么"建图"值得单独一讲（行 17–23）

建图本是 SLAM 两大目标之一（同时定位与建图），但前面各讲讨论的几乎都是定位（特征点定位、直接法定位、后端优化）。这并非暗示建图不重要：在经典 SLAM 模型中，所谓地图即**所有路标点的集合**；一旦确定路标点位置即可说完成了建图，故 VO、BA 事实上都已建模并优化了路标位置。单独讲建图，是因为**人们对建图的需求不同**：SLAM 作为底层技术为上层应用提供信息——上层若是机器人，开发者要全局定位 + 在地图中导航（如扫地机要覆盖全图的路径）；上层若是 AR 设备，要把虚拟物叠加进现实并处理**遮挡关系**。应用层对"定位"需求相似（要相机/载体的空间位姿），而对"地图"则有许多不同需求。

## 1.2 地图的五大用途（行 25–33，逐条全录）

1. **定位**：地图的基本功能。VO 部分用局部地图定位；回环检测部分用全局描述子定位。还希望**保存地图**，让机器人下次开机仍能在其中定位（只建一次模，不必每次重做完整 SLAM）。
2. **导航**：机器人在地图中做**路径规划**，在任意两地图点间找路并控制运动到目标点。至少需知道哪些地方**不可通过**、哪些可通过——超出稀疏特征点地图能力，必须是**稠密地图**。
3. **避障**：与导航类似但更重**局部、动态障碍物**处理。仅有特征点无法判断某点是否为障碍物，需稠密地图。
4. **重建**：用 SLAM 获得环境重建效果，主要**向人展示**（要美观），或用于通信（三维视频通话、网购）。这种地图亦稠密，且对外观有要求——可能不满足于稠密点云，更希望**带纹理的平面**（如电子游戏场景）。
5. **交互**：人与地图互动。如 AR 中放置虚拟物并与之互动（点击墙面虚拟浏览器看视频、向墙投掷物体并希望有虚拟物理碰撞）。机器人也可能收到"取桌子上的报纸"——除环境地图外还需知道哪块是"桌子"、什么叫"之上"、什么叫"报纸"，即对地图有更高层认知——**语义地图**。

> **图 12-1**（行 37–41）：各种地图类型与用途的关系示意。例子来自参考文献 [88,119,120]。之前讨论基本集中于"稀疏路标地图"，未探讨稠密地图。

## 1.3 稀疏 vs 稠密（行 35）

稠密地图相对稀疏地图而言：**稀疏地图只建模感兴趣的部分**（特征点/路标点）；**稠密地图建模所有看到过的部分**。同一张桌子，稀疏地图可能只建模四个角，稠密地图则建模整个桌面。从定位看四个角也够用，但**无法从四个角推断空间结构**，故无法仅用四个角完成导航、避障等需稠密地图才能完成的工作。

## 1.4 度量地图 vs 拓扑地图（主源 §12.6 小结，行 1259）

本讲的地图**偏重度量地图**（metric map）；**拓扑地图**（topological map）形式由于和 SLAM 研究差别较大，故未详细展开。

> **辅源补：地图三大分类轴（综合时可整合 Handbook §5.2/§5.3）**
> - **度量 / 拓扑 / 语义**：度量图存几何坐标；拓扑图存节点+连通关系（图结构，弱化精确度量）；语义图在几何上附加物体类别/可操作语义。
> - **稀疏 / 稠密**（Handbook §5.2 行 66）：运动估计与定位偏好**稀疏**表示（3D 点特征）以做一致位姿估计；场景重建追求**稠密、高分辨**地图（如巡检）；路径规划需**稠密**信息（障碍占据/最近碰撞距离）。"**表示应由用例（use case）决定**"。
> - **显式 / 隐式**（Handbook §5.3.1，Fig 5.4 四象限）：见本抽取 §13 表。

---

# 2 单目稠密重建：立体视觉总入口（主源 §12.2.1，行 48–70）

## 2.1 相机是"只有角度的传感器"（Bearing only）（行 52）

单幅图像的像素只提供**物体与成像平面的角度**及**亮度**，**无法提供距离**（Range）。稠密重建需知道每个（或大部分）像素的距离，三大方案：
1. **单目**：估计相机运动 + **三角化**计算像素距离。
2. **双目**：用左右目**视差**算距离（多目同理）。
3. **RGB-D**：直接获得像素距离。

前两种称**立体视觉**（Stereo Vision），其中移动单目又称**移动视角立体视觉**（Moving View Stereo, MVS）。相比 RGB-D 直接测深度，单/双目"**费力不讨好**"——计算量巨大、最后得到不怎么可靠的深度估计。但单/双目好处是：在 RGB-D 尚不能很好应用的**室外、大场景**仍能用立体视觉估深度。

## 2.2 问题设定：已知轨迹下估某帧深度（行 62–70）

先**不考虑 SLAM**，考虑较简单的建图：假定有一段视频序列，已（用某种魔法或 VO 前端）得到每帧轨迹。**以第一幅图为参考帧，计算其中每个像素的深度（距离）**。

对比特征点法的做法：(1) 提特征、按描述子匹配，即对某空间点跟踪、知其在各图位置；(2) 多视角观测三角化估深度。

**稠密深度图估计的不同**：无法把每像素都当特征点算描述子，故**匹配成为关键一环**——如何确定第一幅图某像素出现在其他图里的位置？需**极线搜索 + 块匹配**（参考文献 [121]）。知道某像素在各图位置后，像特征点那样用三角测量定深度。**不同的是要用很多次三角测量让深度估计收敛**（非仅一次）：希望深度估计随测量增加，从非常不确定逐渐收敛到稳定值——这就是**深度滤波器**。

---

# 3 极线搜索与块匹配（主源 §12.2.2，行 72–122）

## 3.1 极线几何（行 74–80）

类似 7.3 节对极几何。左相机观测到像素 $p_1$，单目无从知其深度，故假设深度在某区间内，如 $(d_{\min},+\infty)$；该像素对应的空间点分布在一条**线段（本例为射线）**上。从右侧相机看，这条线段的投影在图像平面形成一条线——**极线**。知道两相机间运动时极线可确定。问题即：**极线上哪个点是 $p_1$ 对应点？**

> **图 12-2**（行 76–80）极线搜索示意图。

## 3.2 块匹配的动机（行 83–88）

无描述子，只能在极线上搜与 $p_1$ 相似的点（沿极线逐个比较像素相似度，与直接法异曲同工）。单像素亮度不稳定不可靠：极线上可能有很多相似点。直观想法：**比较像素块**——在 $p_1$ 周围取 $w\times w$ 小块，在极线上也取同样大小小块比较，提高区分性。这就是**块匹配**。代价：假设从"像素灰度不变"加强为"**图像块灰度不变**"。

记 $p_1$ 周围小块 $A\in\mathbb R^{w\times w}$，极线上 $n$ 个小块 $B_i,\ i=1,\dots,n$。

## 3.3 块间相似度度量（行 89–111，全部公式逐条）

**① SAD（Sum of Absolute Difference，绝对差之和）（式 12.1）**
$$
S(\boldsymbol A,\boldsymbol B)_{\mathrm{SAD}}=\sum_{i,j}\bigl|\boldsymbol A(i,j)-\boldsymbol B(i,j)\bigr|.
\tag{12.1}
$$

**② SSD（Sum of Squared Distance，平方和）（式 12.2）**（注：非"固态硬盘"）
$$
S(\boldsymbol A,\boldsymbol B)_{\mathrm{SSD}}=\sum_{i,j}\bigl(\boldsymbol A(i,j)-\boldsymbol B(i,j)\bigr)^2.
\tag{12.2}
$$

**③ NCC（Normalized Cross Correlation，归一化互相关）（式 12.3）**
$$
S(\boldsymbol A,\boldsymbol B)_{\mathrm{NCC}}=\frac{\sum_{i,j}\boldsymbol A(i,j)\,\boldsymbol B(i,j)}{\sqrt{\sum_{i,j}\boldsymbol A(i,j)^2\ \sum_{i,j}\boldsymbol B(i,j)^2}}.
\tag{12.3}
$$

**度量方向性（行 109，关键）**：NCC 用相关性，**接近 0 表不相似，接近 1 表相似**；SAD/SSD 反过来，**接近 0 表相似，大数值表不相似**。

**精度-效率矛盾（行 111）**：精度好的方法计算复杂，简单快的效果差，工程中需取舍。

**去均值版本（行 111）**：可先去掉每块均值，得**去均值 SSD、去均值 NCC**。去均值后允许"小块 B 整体比 A 亮一些但仍相似"的情况，故更可靠。更多度量见参考文献 [122,123]。

## 3.4 去均值归一化互相关 ZNCC（主源 §12.3 第 4 点，式 12.11）

主源在实践代码说明里给出**零均值-归一化互相关**（Zero-mean NCC, ZNCC）的完整式（即代码 `NCC()` 实际所用）：
$$
\mathrm{NCC}_z(\boldsymbol A,\boldsymbol B)=\frac{\sum_{i,j}\bigl(\boldsymbol A(i,j)-\bar{\boldsymbol A}\bigr)\bigl(\boldsymbol B(i,j)-\bar{\boldsymbol B}\bigr)}{\sqrt{\sum_{i,j}\bigl(\boldsymbol A(i,j)-\bar{\boldsymbol A}\bigr)^2\ \sum_{i,j}\bigl(\boldsymbol B(i,j)-\bar{\boldsymbol B}\bigr)^2}}.
\tag{12.11}
$$
其中 $\bar{\boldsymbol A},\bar{\boldsymbol B}$ 为各自块均值。

> **抽取者注**：式 (12.11) 原文写作 $\bar{\boldsymbol A}(i,j)$、$\bar{\boldsymbol B}(i,j)$ 带 $(i,j)$，但均值是标量常数，应为 $\bar A,\bar B$（不依赖 $i,j$）。属原文排版小瑕疵，已在上式更正为标量均值。

## 3.5 NCC 沿极线的分布（行 113–122）

对极线上每个 $B_i$ 算 $A$ 与之的相似度（设用 NCC），得**沿极线的 NCC 分布**，形状取决于图像数据。**搜索距离较长时通常得非凸函数**：分布有许多峰值，而真实对应点只有一个。此时倾向用**概率分布描述深度值**而非单一数值——问题转为"不断对不同图像做极线搜索时，深度分布如何变化"——即**深度滤波器**。

> **图 12-3**（行 117–121）匹配得分沿距离的分布（非凸、多峰），图来自参考文献 [124]。

---

# 4 高斯分布的深度滤波器（主源 §12.2.3，行 124–209）

## 4.1 建模思路与两条路线（行 126–128）

像素深度估计本身是**状态估计问题**，自然有**滤波器**与**非线性优化**两种解法。非线性优化效果较好，但 SLAM 实时性强、前端已占大量算力，建图通常用**计算量较少的滤波器**。深度分布假设有几种：
- **简单假设**：深度服从高斯分布，得类卡尔曼方法（**实际只是归一化积**，下文可见）。
- **复杂假设**：文献 [68,124] 用**均匀-高斯混合分布**，推导更复杂的深度滤波器（本抽取 §5 全量补出）。

本节先讲高斯假设，把均匀-高斯混合留作习题。

## 4.2 高斯深度融合（信息融合，式 12.4–12.6）（行 130–148）

设某像素深度 $d$ 服从（式 12.4）
$$
P(d)=N(\mu,\sigma^2).
\tag{12.4}
$$
每当新数据到来观测到深度，假设观测也是高斯（式 12.5）
$$
P(d_{\mathrm{obs}})=N(\mu_{\mathrm{obs}},\sigma_{\mathrm{obs}}^2).
\tag{12.5}
$$
用观测更新 $d$ 的分布——**信息融合问题**。据附录 A，**两个高斯分布的乘积仍是高斯**。设融合后 $d\sim N(\mu_{\mathrm{fuse}},\sigma_{\mathrm{fuse}}^2)$，则由高斯乘积（式 12.6）
$$
\boxed{\ \mu_{\mathrm{fuse}}=\frac{\sigma_{\mathrm{obs}}^2\,\mu+\sigma^2\,\mu_{\mathrm{obs}}}{\sigma^2+\sigma_{\mathrm{obs}}^2},\qquad
\sigma_{\mathrm{fuse}}^2=\frac{\sigma^2\,\sigma_{\mathrm{obs}}^2}{\sigma^2+\sigma_{\mathrm{obs}}^2}.\ }
\tag{12.6}
$$

**关键说明（行 148）**：因为**仅有观测方程、无运动方程**，深度只用到信息融合部分，无须像完整卡尔曼那样预测+更新（可看成"运动方程为深度固定不动"的卡尔曼）。

> **🔑 习题 1（行 1263）= 推导式 (12.6)**。完整推导（综合时正文应给）：
> 两高斯密度相乘 $N(d;\mu,\sigma^2)\,N(d;\mu_{obs},\sigma_{obs}^2)\propto\exp\!\Big[-\tfrac12\big(\tfrac{(d-\mu)^2}{\sigma^2}+\tfrac{(d-\mu_{obs})^2}{\sigma_{obs}^2}\big)\Big]$。
> 配方：指数内 $=-\tfrac12\big[\big(\tfrac1{\sigma^2}+\tfrac1{\sigma_{obs}^2}\big)d^2-2\big(\tfrac{\mu}{\sigma^2}+\tfrac{\mu_{obs}}{\sigma_{obs}^2}\big)d+\text{const}\big]$。
> 令 $\tfrac1{\sigma_{fuse}^2}=\tfrac1{\sigma^2}+\tfrac1{\sigma_{obs}^2}=\tfrac{\sigma^2+\sigma_{obs}^2}{\sigma^2\sigma_{obs}^2}\Rightarrow\sigma_{fuse}^2=\tfrac{\sigma^2\sigma_{obs}^2}{\sigma^2+\sigma_{obs}^2}$。
> 又 $\tfrac{\mu_{fuse}}{\sigma_{fuse}^2}=\tfrac{\mu}{\sigma^2}+\tfrac{\mu_{obs}}{\sigma_{obs}^2}\Rightarrow\mu_{fuse}=\sigma_{fuse}^2\big(\tfrac{\mu}{\sigma^2}+\tfrac{\mu_{obs}}{\sigma_{obs}^2}\big)=\tfrac{\sigma_{obs}^2\mu+\sigma^2\mu_{obs}}{\sigma^2+\sigma_{obs}^2}$。证毕，与 (12.6) 一致。
> （等价信息形式：$\sigma_{fuse}^{-2}=\sigma^{-2}+\sigma_{obs}^{-2}$，即**信息相加**——这正是 §5 的 $s^2,m$ 与 KF 增益的来源。）

## 4.3 观测不确定度 $\mu_{obs},\sigma_{obs}$ 的几何推导（式 12.7–12.10）（行 150–197）

剩下的问题：如何算观测深度分布 $\mu_{obs},\sigma_{obs}$？处理方式不一：文献 [75] 考虑**几何不确定性 + 光度不确定性之和**；文献 [124] **仅考虑几何不确定性**。本节**仅考虑几何关系带来的不确定性**。

**几何设定（图 12-4，行 152）**：某次极线搜索找到 $p_1$ 对应的 $p_2$，从而观测到深度，认为 $p_1$ 对应三维点为 $P$。记 $O_1P=\boldsymbol p$，相机平移 $O_1O_2=\boldsymbol t$，$O_2P=\boldsymbol a$；三角形下面两角记 $\alpha,\beta$。极线 $l_2$ 上存在**一个像素的误差**，使 $\beta\to\beta'$，$p_2\to p_2'$，上面那个角记 $\gamma$。问：这一像素误差会使 $\boldsymbol p'$ 与 $\boldsymbol p$ 差多少？

> **图 12-4**（行 154–158）不确定性分析。

**几何关系（式 12.7）**：
$$
\boldsymbol a=\boldsymbol p-\boldsymbol t,
$$
$$
\alpha=\arccos\langle\boldsymbol p,\boldsymbol t\rangle,
\tag{12.7}
$$
$$
\beta=\arccos\langle\boldsymbol a,-\boldsymbol t\rangle.
$$
（此处 $\langle\cdot,\cdot\rangle$ 为两单位向量内积＝夹角余弦。）

**扰动一像素（式 12.8）**：
$$
\beta'=\arccos\bigl\langle O_2p_2',\,-\boldsymbol t\bigr\rangle,
\tag{12.8}
$$
$$
\gamma=\pi-\alpha-\beta'.
$$

**正弦定理求 $\boldsymbol p'$ 大小（式 12.9）**：
$$
\boxed{\ \|\boldsymbol p'\|=\|\boldsymbol t\|\,\frac{\sin\beta'}{\sin\gamma}.\ }
\tag{12.9}
$$

**几何不确定度（式 12.10）**：若认为极线块匹配仅一个像素误差，则
$$
\sigma_{\mathrm{obs}}=\|\boldsymbol p\|-\|\boldsymbol p'\|.
\tag{12.10}
$$
若极线搜索不确定性大于一个像素，可按此推导**放大**该不确定性。

> **抽取者注**：式 (12.10) 取的是**差值**（可为正负），实际工程取其绝对值或平方作为方差（见 §6 代码 `d_cov=p_prime-depth_estimation; d_cov2=d_cov*d_cov`）。

## 4.4 收敛判据（行 197）

在实际工程中，**当不确定性小于一定阈值时，就认为深度数据已收敛**。

## 4.5 完整高斯深度估计流程（行 199–209，逐条全录）

1. 假设所有像素深度满足某个**初始高斯分布**。
2. 新数据产生时，通过**极线搜索 + 块匹配**确定投影点位置。
3. 根据几何关系计算**三角化后的深度及不确定性**（§4.3）。
4. 将当前观测**融合**进上一次估计（式 12.6）。**若收敛则停止，否则返回第 2 步。**

> **❗ 深度定义澄清（行 209，记号陷阱）**：这里的深度值是 $O_1P$ 的**长度**（沿光心射线），与针孔相机模型里的"深度"（像素的 **$z$ 值**）**有少许不同**。综合时务必区分（见记号表）。

> **辅源补：单测量不确定度 $\tau$ 的等价表述（REMODE/Vogiatzis，§5 详述）**
> 14 讲式 (12.9)–(12.10) 的几何不确定度，在 REMODE/Vogiatzis 中记为 $\tau$（或 $\tau^2$）：一像素视差误差经三角化几何放大，**$\tau^2\propto 1/\sin^2\alpha$**（$\alpha$＝视差角/视线夹角）——视线越接近平行（$\alpha$ 越小），不确定度越大。这与 14 讲"$\sin\gamma$ 在分母"的放大机理一致。

---

# 5 贝叶斯（均匀-高斯混合）深度滤波器：逐帧更新完整式（辅源 2/3/4）

> **来源**：Vogiatzis & Hernández 2011（原始）；REMODE 2014、SVO TRO16（沿用，逆深度版）。14 讲 §12.2.3 把它点名为习题、§12.3.6 提"文献 [124] 的均匀-高斯混合分布滤波器显式区分内点/外点并概率建模，能较好处理外点"。**本节补全 14 讲未给的完整闭式更新式**，供综合 agent 写成正文（这是单目稠密的"工业级"深度滤波器）。
> **⚠️ PDF 抽取受限**：原论文 PDF 经 WebFetch 为图片流不可逐字 OCR；以下为文献公认的、已发表标准形式，经两处可读 HTML 解析 + 多次检索交叉印证；个别我未核到原 PDF 编号者已标注。

## 5.1 生成式测量模型（高斯 + 均匀混合）

一次深度（或逆深度）测量 $x_k$ 要么是**内点**（围绕真值 $Z$ 的高斯），要么是**外点**（在有效区间 $[Z_{\min},Z_{\max}]$ 上均匀）。以**内点率** $\pi_{in}\in[0,1]$ 加权：
$$
\boxed{\ p(x_k\mid Z,\pi_{in})=\pi_{in}\,N\!\bigl(x_k\mid Z,\tau_k^2\bigr)+(1-\pi_{in})\,U\!\bigl(x_k\mid Z_{\min},Z_{\max}\bigr).\ }
\tag{V1}
$$
其中 $\tau_k^2$＝第 $k$ 次测量的几何不确定度（即 14 讲 §4.3 的 $\sigma_{obs}^2$ / $\tau^2$），$U(\cdot)=\dfrac{1}{Z_{\max}-Z_{\min}}$。

## 5.2 后验的参数化近似（高斯 × Beta）

对深度 $Z$ 与内点率 $\pi_{in}$ 的联合后验，用**高斯（深度）× Beta（内点率）**之积近似：
$$
\boxed{\ q\!\bigl(Z,\pi_{in}\mid a_n,b_n,\mu_n,\sigma_n^2\bigr)=\mathrm{Beta}\!\bigl(\pi_{in}\mid a_n,b_n\bigr)\;N\!\bigl(Z\mid \mu_n,\sigma_n^2\bigr).\ }
\tag{V2}
$$
- $\mathrm{Beta}(\pi;a,b)$ 的一阶矩 $\mathbb E[\pi]=\dfrac{a}{a+b}$，可解读为"该像素到目前为止是内点的比例"。
- $a$ 累计"内点证据"、$b$ 累计"外点证据"（直观：成功匹配→ $a$ 增；失配→ $b$ 增）。

## 5.3 逐帧贝叶斯更新（矩匹配闭式式）

来一次新测量 $x$（方差 $\tau^2$），先算两个**责任度/归一化常数**（内点项 $C_1$、外点项 $C_2$）：
$$
C_1=\frac{a}{a+b}\,N\!\bigl(x\mid \mu,\sigma^2+\tau^2\bigr),\qquad
C_2=\frac{b}{a+b}\,U(x),
\tag{V3}
$$
$$
\text{归一化：}\quad C_1\leftarrow\frac{C_1}{C_1+C_2},\quad C_2\leftarrow\frac{C_2}{C_1+C_2}\quad(\text{使 }C_1+C_2=1).
\tag{V4}
$$
内点假设下的卡尔曼式融合（信息相加，对应 14 讲式 12.6 的信息形式）：
$$
\frac{1}{s^2}=\frac{1}{\sigma^2}+\frac{1}{\tau^2},\qquad
m=s^2\!\left(\frac{\mu}{\sigma^2}+\frac{x}{\tau^2}\right).
\tag{V5}
$$
**深度均值/方差更新**（对内/外两假设做矩匹配，即按 $C_1,C_2$ 加权混合两分支的一、二阶矩）：
$$
\boxed{\ \mu'=C_1\,m+C_2\,\mu,\ }
\tag{V6}
$$
$$
\boxed{\ \sigma'^2=C_1\,(s^2+m^2)+C_2\,(\sigma^2+\mu^2)-\mu'^2.\ }
\tag{V7}
$$
**内点率 Beta 参数更新**（匹配 $\pi_{in}$ 的一、二阶矩 $f=\mathbb E[\pi'],\ e=\mathbb E[\pi'^2]$）：
$$
f=C_1\,\frac{a+1}{a+b+1}+C_2\,\frac{a}{a+b+1},
\tag{V8}
$$
$$
e=C_1\,\frac{(a+1)(a+2)}{(a+b+1)(a+b+2)}+C_2\,\frac{a(a+1)}{(a+b+1)(a+b+2)},
\tag{V9}
$$
$$
\boxed{\ a'=\frac{e-f}{\,f-\dfrac{e}{f}\,},\qquad b'=a'\,\frac{1-f}{f}.\ }
\tag{V10}
$$

> **⚠️ 标准发表形式，未逐字核到原 PDF 公式编号**：(V3)–(V10) 取自 Vogiatzis & Hernández 2011 的「parametric approximation to posterior」附录之公认形式，并经 cnblogs SVO 解析、shuzhiduo REMODE 解析两处可读 HTML + 检索结果交叉印证（其中 cnblogs/检索结果直接给出了 $C_1,C_2,s^2,m,\mu',\sigma'^2,f,e$ 与 $a',b'$ 的上述形式）。综合 agent 若要标精确式号，请回原论文核对；物理含义与推导（矩匹配混合两假设）是确定无误的。
>
> **逆深度版（SVO/REMODE 主用）**：把上式中 $Z$ 换成**逆深度** $\rho_{inv}=1/Z$、$\mu,\sigma^2,\tau^2$ 均按逆深度计，其余形式不变（逆深度近高斯，数值更稳，见 §6）。

## 5.4 收敛/发散判据（REMODE）

按内点率后验 $\mathbb E[\pi_{in}]=\dfrac{a}{a+b}$ 与深度方差 $\sigma^2$ 判定该像素 seed 的状态：
- **收敛（成功）**：$\dfrac{a}{a+b}>\eta_{in}$（典型 $\eta_{in}=0.6$）**且**深度方差足够小（$\sigma^2$ 低于阈值，REMODE 取与深度范围相关的小量）→ 升级为可信深度点。
- **发散（外点/丢弃）**：$\dfrac{a}{a+b}<\eta_{out}$（典型 $\eta_{out}=0.2$）→ 判为外点，删除该 seed。
- 否则**继续**用新帧更新。

> 14 讲示例代码用的是**高斯版**的简化判据（§6）：`min_cov=0.1`（方差小于此＝收敛）、`max_cov=10`（大于此＝发散），未显式建模内点率——这正是 14 讲坦言"没有显式处理外点"的原因（§12.3.6 第 2 点）。

## 5.5 REMODE 的不确定度加权正则化（总变分 / 加权 Huber 范数）

REMODE 在逐像素贝叶斯估计之上，加一层**全图正则化**：用**深度不确定度加权的 Huber 范数 + 数据项**的凸能量泛函，使噪声大处更平滑、噪声小处保边：
$$
\min_{F}\ \int_{\Omega}\Bigl[\,G(\mathbf u)\,\bigl\|\nabla F(\mathbf u)\bigr\|_{\varepsilon}\;+\;\lambda\,\bigl\|F(\mathbf u)-\hat F(\mathbf u)\bigr\|_{1}\Bigr]\,\mathrm d\mathbf u,
\tag{R1}
$$
其中 $F$＝正则化后逆深度图、$\hat F$＝逐像素估计、$\lambda$＝数据项权重、$\|\cdot\|_\varepsilon$＝Huber 范数；**权重 $G(\mathbf u)$ 由该像素深度不确定度驱动**（不确定度大→ $G$ 小→ 更强平滑）。该凸问题用高度并行的原始-对偶方案求解，CUDA 实现达 50 Hz。

> **⚠️ (R1) 为 REMODE 正则化思想的标准表述（加权 Huber + L1 数据项 + 不确定度权重 $G$），未逐字核到原 PDF 式号**；检索结果明确"REMODE 用**加权 Huber 范数**、由深度不确定度驱动平滑、凸公式可并行"。综合时若仅做"简介"，可只给思想与 $G$ 的作用，不必展开原始-对偶迭代。

---

# 6 实践：单目稠密重建（REMODE 数据集）（主源 §12.3，行 211–633）

## 6.1 数据集与位姿文件（行 213–222）

示例用 **REMODE 测试数据集**（参考文献 [121,125]）：一架无人机的单目俯视图像共 **200 张**，附每张真实位姿。下载：`http://rpg.ifi.uzh.ch/datasets/remode_test_data.zip`。解压后 `test_data/Images` 有 0~200 全部图像，`test_data` 下有记录每图位姿的文本文件：
```text
scene_000.png 1.086410 4.766730 -1.449960 0.789455 0.051299 -0.000779 0.611661
scene_001.png 1.086390 4.766370 -1.449530 0.789180 0.051881 -0.001131 0.611966
scene_002.png 1.086120 4.765520 -1.449090 0.788982 0.052159 -0.000735 0.612198
.....
```
（格式：文件名 + 平移 $t_x,t_y,t_z$ + 四元数 $q_x,q_y,q_z,q_w$。）

> **图 12-5**（行 226–262）t=0/50/100/150/200 时刻图像；场景主要由地面、桌子及桌上杂物组成。

## 6.2 参数与函数签名（行 269–337）

`slambook2/ch12/dense_monocular/dense_mapping.cpp`（片段）：
```cpp
/******************************************************************************************
* 本程序演示了单目相机在已知轨迹下的稠密深度估计
* 使用极线搜索 + NCC 匹配的方式，与书本的12.2节对应
* 请注意本程序并不完美，你完全可以改进它——笔者其实在故意暴露一些问题
******************************************************************************************/

//----
// parameters
const int boarder = 20;    // 边缘宽度
const int width = 640;    // 图像宽度
const int height = 480;    // 图像高度
const double fx = 481.2f;    // 相机内参
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const int ncc_window_size = 3;    // NCC 取的窗口半宽度
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1); // NCC窗口面积
const double min_cov = 0.1;    // 收敛判定：最小方差
const double max_cov = 10;    // 发散判定：最大方差
```
重要函数（注释逐字录）：
```cpp
/**
 * 根据新的图像更新深度估计
 * @param ref    参考图像
 * @param curr    当前图像
 * @param T_C_R    参考图像到当前图像的位姿
 * @param depth    深度
 * @param depth_cov    深度方差
 * @return    是否成功
 */
bool update(const Mat &ref, const Mat &curr, const SE3d &T_C_R,
            Mat &depth, Mat &depth_cov2);

/**
 * 极线搜索
 * @param ref/curr/T_C_R/pt_ref/depth_mu/depth_cov/pt_curr/epipolar_direction
 * @return 是否成功
 */
bool epipolarSearch(const Mat &ref, const Mat &curr, const SE3d &T_C_R,
                    const Vector2d &pt_ref, const double &depth_mu, const double &depth_cov,
                    Vector2d &pt_curr, Vector2d &epipolar_direction);

/**
 * 更新深度滤波器
 */
bool updateDepthFilter(const Vector2d &pt_ref, const Vector2d &pt_curr, const SE3d &T_C_R,
                       const Vector2d &epipolar_direction, Mat &depth, Mat &depth_cov2);

/** 计算NCC评分 */
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

## 6.3 main 函数（行 362–407）

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
    if (ret == false) { cout << "Reading image files failed!" << endl; return -1; }
    cout << "read total " << color_image_files.size() << "files." << endl;

    // 第一张图
    Mat ref = imread(color_image_files[0], 0); // gray-scale image
    SE3d pose_ref_TWC = poses_TWC[0];
    double init_depth = 3.0; // 深度初始值
    double init_cov2 = 3.0;  // 方差初始值
    Mat depth(height, width, CV_64F, init_depth);     // 深度图
    Mat depth_cov2(height, width, CV_64F, init_cov2); // 深度图方差

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
**数值**：初始深度均值与方差均取 **3.0**；图像边缘 `boarder=20` 不处理。

## 6.4 update：遍历像素 + 收敛/发散跳过（行 409–431）

```cpp
bool update(const Mat &ref, const Mat &curr, const SE3d &T_C_R, Mat &depth, Mat &depth_cov2) {
    for (int x = boarder; x < width - boarder; x++)
        for (int y = boarder; y < height - boarder; y++) {
            // 遍历每个像素
            if (depth_cov2.ptr<double>(y)[x] < min_cov || depth_cov2.ptr<double>(y)[x] > max_cov)
                continue; // 深度已收敛或发散
            // 在极线上搜索(x,y)的匹配
            Vector2d pt_curr;
            Vector2d epipolar_direction;
            bool ret = epipolarSearch(
                ref, curr, T_C_R, Vector2d(x, y), depth.ptr<double>(y)[x],
                sqrt(depth_cov2.ptr<double>(y)[x]), pt_curr, epipolar_direction);
            if (ret == false) continue; // 匹配失败
            // showEpipolarMatch(ref, curr, Vector2d(x, y), pt_curr); // 取消注释以显示匹配
            // 匹配成功，更新深度图
            updateDepthFilter(Vector2d(x, y), pt_curr, T_C_R, epipolar_direction, depth, depth_cov2);
        }
}
```

## 6.5 epipolarSearch：±3σ 极线段 + NCC 寻优（行 439–483）

```cpp
bool epipolarSearch(
    const Mat &ref, const Mat &curr, const SE3d &T_C_R, const Vector2d &pt_ref,
    const double &depth_mu, const double &depth_cov,
    Vector2d &pt_curr, Vector2d &epipolar_direction) {
    Vector3d f_ref = px2cam(pt_ref);
    f_ref.normalize();
    Vector3d P_ref = f_ref * depth_mu; // 参考帧的P向量

    Vector2d px_mean_curr = cam2px(T_C_R * P_ref); // 按深度均值投影的像素
    double d_min = depth_mu - 3 * depth_cov, d_max = depth_mu + 3 * depth_cov;
    if (d_min < 0.1) d_min = 0.1;
    Vector2d px_min_curr = cam2px(T_C_R * (f_ref * d_min));  // 按最小深度投影的像素
    Vector2d px_max_curr = cam2px(T_C_R * (f_ref * d_max));  // 按最大深度投影的像素

    Vector2d epipolar_line = px_max_curr - px_min_curr;      // 极线（线段形式）
    epipolar_direction = epipolar_line;                      // 极线方向
    epipolar_direction.normalize();
    double half_length = 0.5 * epipolar_line.norm();         // 极线线段的半长度
    if (half_length > 100) half_length = 100;                // 我们不希望搜索太多东西
    // showEpipolarLine(ref, curr, pt_ref, px_min_curr, px_max_curr); // 取消注释以显示极线

    // 在极线上搜索，以深度均值点为中心，左右各取半长度
    double best_ncc = -1.0;
    Vector2d best_px_curr;
    for (double l = -half_length; l <= half_length; l += 0.7) { // l+=sqrt(2)
        Vector2d px_curr = px_mean_curr + l * epipolar_direction; // 待匹配点
        if (!inside(px_curr)) continue;
        double ncc = NCC(ref, curr, pt_ref, px_curr); // 计算待匹配点与参考帧的NCC
        if (ncc > best_ncc) { best_ncc = ncc; best_px_curr = px_curr; }
    }
    if (best_ncc < 0.85f) return false; // 只相信NCC很高的匹配
    pt_curr = best_px_curr;
    return true;
}
```
**数值/细节（对应行 598 说明）**：以深度均值为中心、左右各取 **$\pm3\sigma$** 半径；沿极线步长取 $\sqrt2/2\approx$ **0.7**；NCC 阈值 **0.85**（低于则失配）；极线半长上限 100 像素。

## 6.6 NCC：零均值归一化互相关（行 485–518）

```cpp
double NCC(const Mat &ref, const Mat &curr, const Vector2d &pt_ref, const Vector2d &pt_curr) {
    // 零均值-归一化互相关
    double mean_ref = 0, mean_curr = 0;
    vector<double> values_ref, values_curr;
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
    // 计算Zero mean NCC
    double numerator = 0, demoniator1 = 0, demoniator2 = 0;
    for (int i = 0; i < values_ref.size(); i++) {
        double n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }
    return numerator / sqrt(demoniator1 * demoniator2 + 1e-10); // 防止分母出现零
}
```
**对应式 (12.11)**；`+1e-10` 防分母为零。

## 6.7 updateDepthFilter：三角化 + 不确定度 + 高斯融合（行 520–590）

**三角化方程（注释逐字，行 531–537）**：
```
// 方程
// d_ref * f_ref = d_cur * (R_RC * f_cur) + t_RC
// f2 = R_RC * f_cur
// 转化成下面这个矩阵方程组
// => [f_ref^T f_ref,  -f_ref^T f2] [d_ref]   [f_ref^T t]
//    [f_cur^T f_ref,  -f2^T f2   ] [d_cur] = [f2^T t   ]
```
即解 $2\times2$ 线性方程 $A\,[d_{ref};d_{cur}]^\top=\boldsymbol b$。代码：
```cpp
bool updateDepthFilter(
    const Vector2d &pt_ref, const Vector2d &pt_curr, const SE3d &T_C_R,
    const Vector2d &epipolar_direction, Mat &depth, Mat &depth_cov2) {
    // 用三角化计算深度
    SE3d T_R_C = T_C_R.inverse();
    Vector3d f_ref = px2cam(pt_ref);  f_ref.normalize();
    Vector3d f_curr = px2cam(pt_curr); f_curr.normalize();

    Vector3d t = T_R_C.translation();
    Vector3d f2 = T_R_C.so3() * f_curr;
    Vector2d b = Vector2d(t.dot(f_ref), t.dot(f2));
    Matrix2d A;
    A(0, 0) = f_ref.dot(f_ref);
    A(0, 1) = -f_ref.dot(f2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = -f2.dot(f2);
    Vector2d ans = A.inverse() * b;
    Vector3d xm = ans[0] * f_ref;       // ref侧的结果
    Vector3d xn = t + ans[1] * f2;      // cur结果
    Vector3d p_esti = (xm + xn) / 2.0;  // P的位置，取两者的平均
    double depth_estimation = p_esti.norm(); // 深度值

    // 计算不确定性（以一个像素为误差）—— 对应式 (12.7)-(12.10)
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
    double p_prime = t_norm * sin(beta_prime) / sin(gamma); // 式 (12.9)
    double d_cov = p_prime - depth_estimation;              // 式 (12.10)
    double d_cov2 = d_cov * d_cov;

    // 高斯融合 —— 对应式 (12.6)
    double mu = depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double sigma2 = depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))];
    double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    double sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);
    depth.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = mu_fuse;
    depth_cov2.ptr<double>(int(pt_ref(1, 0)))[int(pt_ref(0, 0))] = sigma_fuse2;
    return true;
}
```
**注**：`A(1,0)=-A(0,1)` 是利用 $-f_{ref}^\top f_2=-(f_2^\top f_{ref})$ 的对称性（注释里 $A_{10}=f_{cur}^\top f_{ref}$ 实现为 $-A_{01}$）。`mu_fuse/sigma_fuse2` 严格对应式 (12.6)（这里 $\sigma_{obs}^2=$ `d_cov2`）。

## 6.8 关键函数说明（行 592–606，逐条全录）

1. **main** 仅从数据集读图，交给 `update` 更新深度图。
2. **update** 遍历参考帧每像素，先在当前帧找极线匹配，匹配上则更新深度图。
3. **极线搜索**实现细节：假设深度高斯，以均值为中心左右各取 $\pm3\sigma$ 半径，在当前帧找极线投影；遍历极线像素（步长 $\sqrt2/2\approx0.7$）找 NCC 最高点为匹配点；最高 NCC 仍低于阈值（0.85）则失配。
4. **NCC** 用去均值化做法，式 (12.11)。
5. **三角化**方式与 7.5 节一致；不确定性计算与高斯融合与 12.2 节一致。

## 6.9 实验结果（行 610–633）

运行：
```text
$ build/dense_mapping ~/dataset/test_data
read total 202 files.
*** loop 1 ***
*** loop 2 ***
......
```
**显示约定（行 622）**：显示深度值×0.4 的结果——纯白点（数值 1.0）深度约 **2.5 米**；颜色越深深度越小（物体越近）。深度估计是**动态过程**：从不确定初值逐渐收敛到稳定值。初值用均值与方差均为 3.0 的分布。

**观察（行 624）**：迭代超过一定次数后深度图趋稳；大致能看出地板与桌子的区别，桌上物体深度接近桌子。大部分正确，但有大量错误估计（与周围不一致的过大/过小值）；边缘处因被看到次数少而无正确估计。

> **图 12-6**（行 626–633）演示程序运行截图，分别为迭代 10 次和 30 次的结果。

## 6.10 实验分析与讨论（主源 §12.3.1–§12.3.6）

### 6.10.1 总评（§12.3.1，行 636–642）
代码相对简单直接，未用许多 trick，出现了"简单的往往并非最有效"的常见情形。真实数据复杂，能实际工作的程序需大量工程技巧、极其复杂、难向初学者解释，故选用不那么有效但易读的实现。下面从**计算机视觉**和**滤波器**两角度分析。

### 6.10.2 像素梯度的问题（§12.3.2，行 644–662）
- **纹理依赖（行 646–650）**：块匹配正确与否依赖图像块是否有区分度。一片黑/白的块缺信息，NCC 易误匹配（如打印机表面均匀白色→深度多半错误，出现不该有的条纹状深度）。有明显梯度的块区分度好；梯度不明显处难估深度。**立体视觉的通病：对环境纹理的依赖**（双目同样常见）。演示刻意用纹理好的环境（棋盘格地板、木纹桌面）才得看似不错的结果；实际中墙面、光滑表面等亮度均匀处经常出现，影响深度估计。**若仍只关心像素邻域（小块），该问题无法在现有流程上解决。**
- **像素梯度与极线的夹角（行 652–654，图 12-7）**：两种极端——像素梯度**垂直**极线 vs **平行**极线。**垂直**：即使小块有明显梯度，沿极线做块匹配时匹配程度都一样，得不到有效匹配。**平行**：能精确确定匹配度最高点。实际介于两者之间：**梯度与极线夹角较大→不确定性大；夹角较小→不确定性小**。演示把这些都当成一个像素误差，不够精细；应使用更精确的不确定性模型（留作习题）。文献 [75] 详细讨论过它们的关系。

> **图 12-7**（行 656–660）像素梯度与极线关系示意。

### 6.10.3 逆深度（§12.3.3，行 663–679）——见本抽取 §6（逆深度专节，下）

### 6.10.4 图像间的变换（§12.3.4，行 681–709）
块匹配前做一次图像到图像的变换是常见预处理。假设图像小块在相机运动时不变——相机**平移**时成立（示例数据集多如此），相机明显**旋转**时难保持（绕光心旋转可能把"下黑上白"变成"上黑下白"，相关性直接变负，尽管仍是同一块）。

据相机模型，参考帧像素 $P_R$ 与三维点世界坐标 $P_W$（式 12.12）：
$$
d_{\mathrm R}\,\boldsymbol P_{\mathrm R}=\boldsymbol K\bigl(\boldsymbol R_{\mathrm{RW}}\boldsymbol P_{\mathrm W}+\boldsymbol t_{\mathrm{RW}}\bigr).
\tag{12.12}
$$
当前帧（式 12.13）：
$$
d_{\mathrm C}\,\boldsymbol P_{\mathrm C}=\boldsymbol K\bigl(\boldsymbol R_{\mathrm{CW}}\boldsymbol P_{\mathrm W}+\boldsymbol t_{\mathrm{CW}}\bigr).
\tag{12.13}
$$
代入消去 $P_W$，得两图像素关系（式 12.14）：
$$
d_{\mathrm C}\,\boldsymbol P_{\mathrm C}=d_{\mathrm R}\,\boldsymbol K\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol K^{-1}\boldsymbol P_{\mathrm R}+\boldsymbol K\boldsymbol t_{\mathrm{CW}}-\boldsymbol K\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol K\boldsymbol t_{\mathrm{RW}}.
\tag{12.14}
$$
知 $d_{\mathrm R},P_{\mathrm R}$ 可算 $P_{\mathrm C}$ 投影位置。再给 $P_R$ 两分量各一增量 $\mathrm du,\mathrm dv$，求得 $P_C$ 增量 $\mathrm du_c,\mathrm dv_c$，得局部**仿射变换**（式 12.15）：
$$
\begin{bmatrix}\mathrm du_c\\\mathrm dv_c\end{bmatrix}
=\begin{bmatrix}\dfrac{\mathrm du_c}{\mathrm du}&\dfrac{\mathrm du_c}{\mathrm dv}\\[2mm]\dfrac{\mathrm dv_c}{\mathrm du}&\dfrac{\mathrm dv_c}{\mathrm dv}\end{bmatrix}
\begin{bmatrix}\mathrm du\\\mathrm dv\end{bmatrix}.
\tag{12.15}
$$
据此把当前帧（或参考帧）像素变换后再块匹配，以期对旋转更鲁棒。

> **⚠️ OCR/原文小瑕疵（式 12.14）**：末项 $-\boldsymbol K\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol K\boldsymbol t_{\mathrm{RW}}$ 中间的 $\boldsymbol K$ 量纲上应为 $\boldsymbol K^{-1}$（与首项的 $\boldsymbol K^{-1}$ 一致，因消 $P_W$ 时 $P_R$ 先经 $K^{-1}$ 反投影）。**正确形式应为** $-\,\boldsymbol K\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol K^{-1}\,(\boldsymbol K\boldsymbol t_{\mathrm{RW}})$，即 $-\boldsymbol K\boldsymbol R_{\mathrm{CW}}\boldsymbol R_{\mathrm{RW}}^{\mathrm T}\boldsymbol t_{\mathrm{RW}}\cdot$（量纲修正）。综合时建议直接从 (12.12)(12.13) 联立重推：由 (12.12) 得 $\boldsymbol P_W=\boldsymbol R_{RW}^{\mathrm T}(d_R\boldsymbol K^{-1}\boldsymbol P_R-\boldsymbol t_{RW})$，代入 (12.13) 得 $d_C\boldsymbol P_C=d_R\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\mathrm T}\boldsymbol K^{-1}\boldsymbol P_R+\boldsymbol K(\boldsymbol t_{CW}-\boldsymbol R_{CW}\boldsymbol R_{RW}^{\mathrm T}\boldsymbol t_{RW})$。原文末项 $\boldsymbol K\boldsymbol t_{RW}$ 前多/少一个逆，属 OCR 或排版误。

### 6.10.5 并行化：效率问题（§12.3.5，行 711–717）
稠密深度图估计很费时：要估的点从数百特征点变成几十万像素，主流 CPU 也无法实时。但**几十万像素的深度估计彼此无关**！这给并行化用武之地。示例在二重循环里串行遍历像素，但下一像素无须等上一像素。理论上若有 30 万线程，计算时间与算一个像素相同。**GPU** 并行架构非常适合，单/双目稠密重建常用 GPU 加速（本书不涉 GPU 编程）。据类似工作，GPU 上稠密深度估计可实时化。

### 6.10.6 其他改进（§12.3.6，行 719–729，逐条全录）
1. **空间正则项**：各像素现完全独立，可能出现相邻深度一大一小。可假设相邻深度变化不大，加**空间正则项**，使深度图更平滑（即 REMODE §5.5 的总变分）。
2. **外点处理**：未显式处理外点（Outlier）。遮挡、光照、运动模糊使不可能每像素都成功匹配；演示只要 NCC 大于阈值就认为成功，未考虑误匹配。
3. **混合分布滤波器**：处理误匹配可用文献 [124] 的**均匀-高斯混合分布深度滤波器**，显式区分内/外点并概率建模，能较好处理外点（理论较复杂，见原论文）（即本抽取 §5）。

**总结（行 729）**：细致改进每步有望得到良好稠密建图方案，但有些问题存在**理论困难**（纹理依赖、梯度与极线方向关联/平行情况），难以靠改代码解决。故目前虽双目和移动单目能建稠密图，但通常认为它们**过于依赖环境纹理和光照、不够可靠**。

---

# 6′ 逆深度参数化（主源 §12.3.3，行 663–679）

> 编号沿主源放在 §12.3.3，本抽取单列以便综合。

**参数化问题（行 665–667）**：之前用世界坐标 $x,y,z$ 描述点（三维高斯）；本讲用图像坐标 $u,v$ + 深度 $d$ 描述点（认为 $u,v$ 不动、$d$ 服从一维高斯）。两种参数化描述同一三维点。

**为何 $u,v,d$ 更优（行 669）**：相机看到某点时其图像坐标 $u,v$ 比较确定、深度 $d$ 非常不确定。用 $x,y,z$ 描述时，据相机位姿 $x,y,z$ 间可能明显相关（协方差非对角元非零）；用 $u,v,d$ 时 $u,v$ 与 $d$ 至少近似独立、甚至 $u,v$ 也独立——**协方差近似对角阵，更简洁**。

**逆深度（Inverse depth，行 671–679）**：近年 SLAM 广泛使用的参数化技巧（文献 [126,127]）。演示假设深度高斯 $d\sim N(\mu,\sigma^2)$，但深度正态分布有问题：
1. 实际想表达"深度大概 5~10 米，可能有更远点，但近处不会小于焦距（或深度不小于 0）"——**非对称**，尾部稍长、负数区域为零，不像对称高斯。
2. 室外可能有非常远乃至无穷远的点，初始值难涵盖，且用高斯描述有数值困难。

故**逆深度应运而生**：仿真发现假设**深度的倒数（逆深度）为高斯**比较有效 [127]；实际中逆深度数值稳定性更好，成为通用技巧，存在于现有 SLAM 标准做法 [68,69,88]。把演示从正深度改逆深度不复杂：推导中将 $d$ 改成 $d^{-1}$ 即可（留作习题）。

> **习题 3*（行 1267）**：把演示代码从正深度改逆深度，并添加仿射变换，看效果是否改进。

---

# 7 立体匹配 SGM 简介（辅源 4：Hirschmüller TPAMI 2008）

> 14 讲仅在 §12.2.1 用一句话提双目"利用左右目视差计算像素的距离"，**未给任何立体匹配算法**。本节补 SGM（Semi-Global Matching）——经典、工业界广用的双目稠密视差算法。

## 7.1 问题与能量（全局立体匹配框架）

双目（已校正/行对齐）求每像素 $\mathbf p$ 的**视差** $d_{disp}$（左右图同名点的横坐标差），深度 $Z=\dfrac{f\,B}{d_{disp}}$（$f$＝焦距、$B$＝基线）。理想的**全局**能量：
$$
E(\mathbf D)=\sum_{\mathbf p}C\!\bigl(\mathbf p,d_{\mathbf p}\bigr)+\sum_{\mathbf p}\sum_{\mathbf q\in N_{\mathbf p}}R\!\bigl(d_{\mathbf p},d_{\mathbf q}\bigr),
\tag{SGM-1}
$$
- $C(\mathbf p,d)$＝像素 $\mathbf p$ 取视差 $d$ 的**匹配代价**；
- $R$＝邻域平滑（不连续）惩罚。直接对 2D 全局能量做精确优化是 NP 难，SGM 的核心是**用多条一维路径的代价聚合近似 2D 全局优化**。

## 7.2 平滑（正则）项（带边缘自适应 $P_2$）

$$
R(d_{\mathbf p},d_{\mathbf q})=\begin{cases}0,&d_{\mathbf p}=d_{\mathbf q},\\[1mm]P_1,&|d_{\mathbf p}-d_{\mathbf q}|=1,\\[1mm]P_2,&|d_{\mathbf p}-d_{\mathbf q}|>1,\end{cases}
\tag{SGM-2}
$$
其中 $P_1<P_2$：$P_1$ 罚"视差差 1"的小变化（允许斜面/曲面平滑过渡），$P_2$ 罚更大跳变（保深度不连续/物体边界）。**边缘自适应**：
$$
P_2=\max\!\left\{P_1,\ \frac{\hat P_2}{\,|I(\mathbf p)-I(\mathbf q)|\,}\right\},
\tag{SGM-3}
$$
即图像梯度大处（可能是真实边缘）减小 $P_2$，允许视差跳变。

## 7.3 匹配代价 $C(\mathbf p,d)$ 的选择

Hirschmüller 建议用**互信息（Mutual Information, MI）**或基于强度的绝对差；常见可选：绝对/平方强度差、Birchfield-Tomasi 不相似度、**Census 变换汉明距离**、归一化互相关（Pearson）、MI 近似。
**MI 代价（逐像素形式）**：基于两图联合熵的负、用 Taylor 展开成逐像素可加项
$$
C_{\mathrm{MI}}(\mathbf p,d)=-\,\mathrm{mi}_{\,I_1,\,I_2'}\!\bigl(I_1(\mathbf p),\,I_2'(\mathbf p,d)\bigr),
$$
其中 $I_2'$＝按视差 $d$ 形变到左图坐标的右图，$\mathrm{mi}(\cdot)$ 由联合概率 $P_{I_1,I_2}$（用 Parzen/高斯核估计、对当前视差图迭代更新）的逐像素互信息项给出。
> **⚠️ MI 的 Parzen 估计与 Taylor 展开精确式**：Wikipedia 未给逐字公式；需回 Hirschmüller (2005/2008) 原论文核对精确形式。综合作"简介"时给"MI/Census/AD 三选一 + MI 需对联合直方图迭代"即可。

## 7.4 路径代价递推（SGM 核心）

沿方向 $\mathbf r$ 的一维动态规划递推：
$$
\boxed{\
L_{\mathbf r}(\mathbf p,d)=C(\mathbf p,d)+\min\Bigl\{\,
L_{\mathbf r}(\mathbf p-\mathbf r,d),\
L_{\mathbf r}(\mathbf p-\mathbf r,d-1)+P_1,\
L_{\mathbf r}(\mathbf p-\mathbf r,d+1)+P_1,\
\min_i L_{\mathbf r}(\mathbf p-\mathbf r,i)+P_2\Bigr\}
-\min_k L_{\mathbf r}(\mathbf p-\mathbf r,k).\ }
\tag{SGM-4}
$$
末项 $-\min_k L_{\mathbf r}(\mathbf p-\mathbf r,k)$ 对当前像素所有视差是常数，**仅为数值稳定**（防 $L$ 沿路径无界增长），不改变 argmin。

## 7.5 代价聚合与视差选择

把所有方向路径代价相加：
$$
S(\mathbf p,d)=\sum_{\mathbf r}L_{\mathbf r}(\mathbf p,d).
\tag{SGM-5}
$$
路径数通常 **4 / 8 / 16**（Hirschmüller 建议至少 8、推荐 16；16 方向质量好，少则更快；8 方向常分前向/后向两遍计算）。逐像素取最小聚合代价：
$$
d^*(\mathbf p)=\arg\min_d S(\mathbf p,d).
\tag{SGM-6}
$$
- **亚像素精度**：对 $d^*$ 及左右邻代价拟合二次曲线取极小。
- **左右一致性检查**：交换左右图再算一次视差，两次不一致的像素判为无效（遮挡/误匹配）。

## 7.6 复杂度
$$
O(W\cdot H\cdot D),
$$
$W\times H$＝图像尺寸、$D$＝最大视差范围；每像素被访问 $R$（路径数）次。

> **与单目稠密的对照（综合可写）**：SGM 同样基于"匹配代价（块/MI/Census）"，但用**已校正双目 + 沿扫描线/多路径 DP** 取代单目的"极线搜索 + 多帧深度滤波"；SGM 单帧出稠密视差（不需多帧收敛），但需双目硬件与精确标定。

---

# 8 RGB-D 稠密建图总入口 + 点云地图实践（主源 §12.4–§12.4.1，行 731–911）

## 8.1 为何 RGB-D 是更好选择（§12.4，行 733）

RGB-D 相机的深度由传感器硬件直接测得，无须大量计算估计；**结构光/飞时（ToF）原理保证深度对纹理无关**——即使纯色物体，只要能反射光就能测深度。这是 RGB-D 一大优势。

**RGB-D 建图的主流形式（行 735）**：
- 最直观：按估算位姿把 RGB-D 转**点云**拼接，得离散点组成的**点云地图**（Point Cloud Map）。
- 要估物体表面：用**三角网格（Mesh）**、**面元（Surfel）**建图。
- 要障碍物信息并导航：用**体素（Voxel）**建**占据网格地图**（Occupancy Map）。

RGB-D 建图理论不多，下面几节直接以实践介绍；GPU 建图（TSDF）只讲原理不演示。

## 8.2 点云地图实践（§12.4.1，行 739–911）

最基本点含 $x,y,z$，也可带 $r,g,b$ 彩色。RGB-D 提供彩色图+深度图，易按内参算点云；有位姿则直接加和得全局点云（5.4.2 节曾给拼接例子）。实际建图还加滤波以获更好视觉效果，本程序用两种：**外点去除滤波器（统计滤波 SOR）** + **体素网格降采样滤波器（Voxel grid filter）**。

`slambook/ch12/dense_RGBD/pointcloud_mapping.cpp`（片段）：
```cpp
int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs; // 彩色图和深度图
    vector<Eigen::Isometry3d> poses;      // 相机位姿

    ifstream fin("./data/pose.txt");
    if (!fin) { cerr << "cannot find pose file" << endl; return 1; }

    for (int i = 0; i < 5; i++) {
        boost::format fmt("./data/%s/%d.%s"); // 图像文件格式
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "png").str(), -1)); // -1 读取原始图像
        double data[7] = {0};
        for (int i = 0; i < 7; i++) fin >> data[i];
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 计算点云并拼接 —— 相机内参
    double cx = 319.5, cy = 239.5, fx = 481.2, fy = -480.0;
    double depthScale = 5000.0;
    cout << "正在将图像转换为点云..." << endl;

    typedef pcl::PointXYZRGB PointT;          // 点云格式：XYZRGB
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
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;
                PointT p;
                p.x = pointWorld[0]; p.y = pointWorld[1]; p.z = pointWorld[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                current->points.push_back(p);
            }
        // depth filter and statistical removal —— 统计外点去除
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

    // voxel filter —— 体素网格降采样
    pcl::VoxelGrid<PointT> voxel_filter;
    double resolution = 0.03;
    voxel_filter.setLeafSize(resolution, resolution, resolution);
    PointCloud::Ptr tmp(new PointCloud);
    voxel_filter.setInputCloud(pointCloud);
    voxel_filter.filter(*tmp);
    tmp->swap(*pointCloud);
    cout << "滤波之后，点云共有" << pointCloud->size() << "个点." << endl;

    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
    return 0;
}
```
**点云投影公式（代码内嵌，即针孔反投影）**：$z=d/\text{depthScale}$，$x=(u-c_x)z/f_x$，$y=(v-c_y)z/f_y$，再 $P_W=T\,P_{cam}$。`depthScale=5000`（深度图单位→米）。

**安装与思路（行 854–870）**：`sudo apt-get install libpcl-dev pcl-tools`。三处不同于第 5 讲：
1. 去掉深度无效点（Kinect 有效量程外深度误差大或返回 0）。
2. **统计滤波器去孤立点**：统计每点与最近 N 个点的距离分布，去除距离均值过大的点（保留"粘在一起"的点，去孤立噪声点）。参数 `MeanK=50`、`StddevMulThresh=1.0`。
3. **体素网格滤波降采样**：多视角视野重叠区有大量位置相近点占内存；体素滤波保证每个一定大小立方体（体素）内仅一点（三维降采样）。

**数据集与数值（行 870）**：用 **ICL-NUIM** 仿真 RGB-D 数据集（无噪声深度），`data/` 存 5 张图+深度+位姿。体素分辨率 **0.03**（每 $0.03\times0.03\times0.03$ 格仅存一点）。输出点数从 **130 万**降到 **3 万**（只需 **2%** 存储）。

运行：`./build/pointcloud_mapping` → 得 `map.pcd`，用 `pcl_viewer` 打开（图 12-8）。

> **图 12-8**（行 882–890）ICL-NUIM 五张图重建结果（体素滤波后）。

**点云地图对地图需求的满足度（行 893–901，逐条全录）**：
1. **定位**：取决于前端 VO。基于特征点的 VO——点云无特征点信息→无法用于特征点定位；前端是点云 ICP——可将局部点云对全局点云 ICP 估位姿，但需全局点云精度好（本程序未优化点云本身，故不够）。
2. **导航与避障**：**无法直接用**。纯点云无法表"是否有障碍物"，无法做"任意空间点是否被占据"查询（导航避障基本需要）。可在点云基础上加工得更适合的地图。
3. **可视化与交互**：有基本能力（能看场景外观、漫游）。但点云只含离散点、无表面信息（如法线），不太符合可视化习惯（正反面看一样、能透过物体看背后）。

**小结（行 901–903）**：点云地图"基础/初级"，更接近传感器原始数据，常用于调试和基本显示，不便直接用于应用。可作出发点：针对导航构建**占据网格**；SfM 常用**泊松重建** [128] 从点云重建物体网格；**Surfel** [129] 以面元为基本单位建漂亮可视化地图。大部分由点云转换的地图形式 PCL 库都提供。

> **图 12-9**（行 905–909）泊松重建与 Surfel 重建样例，视觉效果明显优于纯点云，且都可由点云构建。

---

# 9 从点云重建网格（主源 §12.4.2，行 912–1033）+ surfel（辅源 5）

## 9.1 思路（行 914）
先计算点云**法线**，再从法线计算**网格**。

## 9.2 完整代码（行 923–1017）

`slambook2/ch12/dense_RGBD/surfel_mapping.cpp`：
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

// (1) MLS 计算法线/面元
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

// (2) 贪婪投影三角化成网格
pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr &surfels) {
    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);
    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);
    gp3.setSearchRadius(0.05);              // 连接点间最大距离（最大边长）
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    gp3.minimumAngle(M_PI / 18);           // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    gp3.setNormalConsistency(true);
    gp3.setInputCloud(surfels);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);
    return triangles;
}

int main(int argc, char **argv) {
    PointCloudPtr cloud(new PointCloud);
    if (argc == 0 || pcl::io::loadPCDFile(argv[1], *cloud)) {
        cout << "failed to load point cloud!"; return 1;
    }
    cout << "point cloud loaded, points: " << cloud->points.size() << endl;
    cout << "computing normals ... " << endl;
    double mls_radius = 0.05, polynomial_order = 2;
    auto surfels = reconstructSurface(cloud, mls_radius, polynomial_order);
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
**数值**：MLS 搜索半径 0.05、多项式阶 2；GP3 搜索半径 0.05、Mu=2.5、最大近邻 100、最大表面角 45°、最小角 10°、最大角 120°。运行 `./build/surfel_mapping map.pcd`（图 12-10）。

**算法出处（行 1027）**：MLS（Moving Least Square）见参考文献 [130]、贪婪投影三角化（Greedy Projection）见 [131]，均为经典重建算法。重建网格后，原本无表面信息的点云可构建法线、纹理等。

> **图 12-10**（行 1029–1033）从点云重建得到的表面和网格模型。

## 9.3 辅源补：surfel / mesh / Marching Cubes / 泊松 的权威定义（Handbook §5.3.2）

- **Surfel（面元，Handbook §5.3.2.2 行 153–157）**：点云只存测量、无表面朝向。surfel 给点加方向信息。常用**圆盘/椭圆盘**表示，或更一般用**椭球（高斯）**建模。一个圆形 surfel 由位置 $p\in\mathbb R^3$、法向 $n\in\mathbb R^3$、半径 $r\in\mathbb R$ 定义：
$$
\text{surfel}=(\,p\in\mathbb R^3,\ n\in\mathbb R^3,\ r\in\mathbb R\,).
$$
GPU 可高效渲染（splatting 融合重叠 surfel、整合纹理）。**KITTI Seq07 对比（图 5.7）**：全累积点云 2.95 GB，对应 SuMa surfel 图仅 160 MB。
- **NDT（正态分布变换，§5.3.2.2 行 159）**：把空间分体素、每体素内点用正态分布 $N(\mu,\Sigma)$ 近似。协方差特征值 $\lambda_1<\lambda_2<\lambda_3$ 与特征向量 $v_1,v_2,v_3$ 估表面属性：平面情形 $\lambda_1\ll\lambda_2$，最小特征值的 $v_1$＝表面法向。是"体素划分（显式）+ 体素内正态分布（隐式）"的混合表示。
- **Mesh（网格，§5.3.2.3）**：点 + 连接成多边形（多为三角形）的集合；能表水密表面、查询/插值新表面点、沿连通表面高效遍历。
- **Marching Cubes（§5.3.2.6 行 195，图 5.8）**：把隐式曲面（距离场）划分为定尺寸立方体网格，逐立方体独立处理；据 8 个角的隐式值生成三角形元素（共 **15 种配置**），顶点位置由**线性插值**精化。是距离场→网格的原始技术。
- **点云↔其它表示的转换（§5.3.2.6 行 193–195）**：点/网格等显式几何可经**最近点查找**转隐式曲面（按需计算，或在规则网格上用**Fast Marching 波前传播**）；隐式曲面经 Marching Cubes 转网格。**泊松曲面重建** [1131] 把点云转网格（14 讲 [128]），**Marching Cubes** [1132] 转 SDF——但仅在有额外数据（如点的测量视角）时可行。

---

# 10 八叉树地图（OctoMap）（主源 §12.4.3–§12.4.4，行 1036–1166）+ 辅源 5

## 10.1 点云的缺陷（行 1038–1043）
- **体积大、且"大"非必需**：640×480 图产生 30 万空间点，pcd 文件很大；提供很多不必要细节（地毯褶皱、阴影），浪费空间；除非降分辨率否则有限内存无法建大环境，降分辨率又降质量。
- **无法处理运动物体**：做法里只有"添加点"、无"点消失时移除"，运动物体普遍存在使点云图不实用。

→ 引入**灵活、压缩、能随时更新**的地图：**八叉树（Octree）** [132]。

## 10.2 八叉树结构与压缩性（行 1046–1059，图 12-11）
把三维空间建模为小方块（体素）：把一小方块每个面均分两片→变 8 个同样大小小方块；不断重复直到达最高精度。"一分为八"＝"从一节点展开为 8 子节点"，整个细分过程即一棵**八叉树**。大方块＝根节点，最小块＝叶子节点。**往上一层体积扩大 8 倍**。

**数值（行 1050）**：叶子方块 $1\,\mathrm{cm}^3$、限 10 层时总建模体积 $\approx 8^{10}\,\mathrm{cm}^3=1{,}073\,\mathrm{m}^3$（足够建一间屋）；体积与深度指数关系，更大深度时增长极快。

> **图 12-11**（行 1052–1056）八叉树示意。

**为何比点云省空间（行 1059）**：八叉树节点存"是否被占据"。**当某方块所有子节点全占据或全不占据时，无须展开该节点**（如空白地图只需一个根节点）。实际物体常连在一起、空白也常连在一起，故大多数节点无须展开到叶子层。

## 10.3 占据概率的 log-odds 表示（式 12.16–12.19）（行 1061–1089）

0-1 表示太简单（噪声使某点一会 0 一会 1、还有"未知"态）。改用**概率**表达：浮点 $x\in[0,1]$，初值 0.5，观测占据则增、观测空白则减。问题：$x$ 可能跑出 $[0,1]$。故改用**概率对数值（Log-odds）**。设 $y\in\mathbb R$ 为对数几率、$x$ 为概率，**logit 变换（式 12.16）**：
$$
y=\operatorname{logit}(x)=\log\!\left(\frac{x}{1-x}\right).
\tag{12.16}
$$
**反变换（式 12.17）**：
$$
x=\operatorname{logit}^{-1}(y)=\frac{\exp(y)}{\exp(y)+1}.
\tag{12.17}
$$
$y:-\infty\to+\infty$ 时 $x:0\to1$；$y=0$ 时 $x=0.5$。故存 $y$：观测"占据"则 $y$ 增、否则减；查询时用逆 logit 转回概率。

设节点 $n$、观测 $z$，到 $t$ 时刻对数几率 $L(n\mid z_{1:t})$，则 $t+1$ 时刻**更新（式 12.18）**：
$$
\boxed{\ L(n\mid z_{1:t+1})=L(n\mid z_{1:t})+L(n\mid z_{t+1}).\ }
\tag{12.18}
$$
> **⚠️ OCR 错误（式 12.18）**：原文右端写作 $L(n\mid z_{1:t-1})+L(n\mid z_t)$，下标 $t-1$ 与 $t$ 与左端 $t+1$ **不自洽**。**正确应为** $L(n\mid z_{1:t+1})=L(n\mid z_{1:t})+L(n\mid z_{t+1})$（"旧对数几率 + 本次观测的逆传感器模型"）。本抽取已更正（与 Handbook 式 5.6 一致：$l(m_k\mid z_{1:t})=l(m_k\mid z_{1:t-1})+l(m_k\mid z_t)$）。

写成概率形式（非对数）则较复杂（式 12.19）：
$$
P(n\mid z_{1:T})=\left[1+\frac{1-P(n\mid z_T)}{P(n\mid z_T)}\,\frac{1-P(n\mid z_{1:T-1})}{P(n\mid z_{1:T-1})}\,\frac{P(n)}{1-P(n)}\right]^{-1}.
\tag{12.19}
$$
（$P(n)$＝先验占据概率，常取 0.5；此式即 (12.18) 的概率域等价物。）

**RGB-D 更新八叉树（行 1089）**：观测到某像素带深度 $d$ → 深度对应空间点**观察到占据**，且**从光心到该点的线段上应无物体**（否则会被遮挡）。用此信息更新八叉树，并能处理运动结构。

## 10.4 八叉树地图实践（§12.4.4，行 1091–1166）

安装：`sudo apt-get install liboctomap-dev octovis`。代码 `slambook/ch13/dense_RGBD/octomap_mapping.cpp`（片段）：
```cpp
// octomap tree
octomap::OcTree tree(0.01); // 参数为分辨率

for (int i = 0; i < 5; i++) {
    cout << "转换图像中：" << i + 1 << endl;
    cv::Mat color = colorImgs[i];
    cv::Mat depth = depthImgs[i];
    Eigen::Isometry3d T = poses[i];
    octomap::Pointcloud cloud; // the point cloud in octomap
    for (int v = 0; v < color.rows; v++)
        for (int u = 0; u < color.cols; u++) {
            unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
            if (d == 0) continue; // 为0表示没有测量到
            Eigen::Vector3d point;
            point[2] = double(d) / depthScale;
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;
            Eigen::Vector3d pointWorld = T * point;
            cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]); // 世界坐标入点云
        }
    // 将点云存入八叉树地图，给定原点，这样可以计算投射线
    tree.insertPointCloud(cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
}
// 更新中间节点的占据信息并写入磁盘
tree.updateInnerOccupancy();
cout << "saving octomap ... " << endl;
tree.writeBinary("octomap.bt");
```
**说明（行 1140–1142）**：用最基本的不带颜色八叉树；`insertPointCloud` 传入**原点**（相机光心）以便计算投射线（自动对线段做"空闲"更新、端点做"占据"更新——即 §10.3 机理）；`updateInnerOccupancy` 更新中间节点；存为压缩 `octomap.bt`。可视化用 `octovis`。

> **图 12-12**（行 1146–1161）八叉树地图在 0.05 m / 0.1 m 分辨率下的显示（按"1"键按高度染色）。

**分辨率调节（行 1164）**：默认深度 16 层＝最高分辨率（边长 0.05 m）；减一层叶子上提、边长翻倍（0.1 m）。

**数值对比（行 1166）**：八叉树可查询任意点占据概率以设计导航 [133]。**点云地图磁盘约 6.9 MB，八叉树地图仅 56 KB（不到点云 1%）**，可有效建大场景。

## 10.5 辅源补：占据图/八叉树/体素数据结构的权威细节（Handbook §5.2.1、§5.3.3、§5.4.4）

- **占据图 log-odds（§5.2.1 行 94–103，式 5.6/5.7）**：给定测量 $z_{1:t}$ 与位姿 $x_{1:t}$，各栅格占据后验 $p(m\mid z_{1:t},x_{1:t})$。设各栅格 $m_k$ 独立、测量条件独立，**log-odds 更新**：
$$
l(m_k\mid \mathbf z_{1:t})=l(m_k\mid \mathbf z_{1:t-1})+l(m_k\mid z_t),
\tag{5.6}
$$
其中 odds $o(m_k\mid\mathbf z_{1:t})=\dfrac{p(m_k\mid\mathbf z_{1:t})}{1-p(m_k\mid\mathbf z_{1:t})}$（式 5.7），$l=\log o$。优点：只需旧值 + **逆传感器模型** $l(m_k\mid z_t)$，加法即可更新。缺点：假设栅格间独立（忽略空间相关）、分辨率须先验固定。
- **八叉树（octree）形式化（§5.3.3.3 行 231，图 5.10）**：每节点是一个 **octant（八分体）**＝子空间，由中心 $c\in\mathbb R^3$ 与范围 $e\in\mathbb R$（轴对齐包围盒）定义；每 octant 有最多 8 个范围 $\tfrac12 e$ 的子 octant。**构造（行 249）**：在含点云 $P$ 的轴对齐包围盒内迭代分 octant，每次把 $P$ 分成 $P_1,\dots,P_8$；非空子集形成子 octant，直到达指定 octant 尺寸或最小点数停止。新数据落在根 octant 外时，建新根、把新数据与旧根挂为其子（**树可向外扩展**）。八叉树**只表含数据的子空间**（省占据空间内存），但需树遍历访问叶子、且树结构本身有内存开销。
- **Octomap 剪枝与多分辨率（§5.4.4 行 329、345）**：Octomap [478] 首倡用 octree 存占据概率，多年事实标准。内节点存子节点的 max/平均占据；**递归剪掉与父节点值接近的叶子**→常数区域自动用更少、更低分辨率节点表示（环境多为自由空间，极有效）。优势：天然多分辨率、可低分辨率查询、利于层次化碰撞检测/探索规划。局限：**所有测量都在最高分辨率积分→计算复杂度仍是立方级**；编码树结构有显著内存开销、叶子查询时间正比于树高。
- **哈希体素块（§5.3.3.2、§5.4.4 行 327）**：Nießner 等 [808] 把体素分块（如 $8\times8\times8$）存哈希表，**O(1) 查询/动态插入**，被 TSDF 广泛采用；块粒度可调权衡哈希表大小与分配粒度。
- **VDB（§5.3.3.4 行 271）**：把空间分哈希块、每块内存层次树；兼得层次树（多分辨率、高效最近邻）与哈希（块定尺寸→树高恒定→常数时间查询/插入）之长。
- **测量积分器（§5.4.4 行 335–339）**：**光线追踪（ray-tracing）**——从传感器向点投射光线、更新沿途所有体素；通用（仅需传感器原点），但体素被多光线击中→重复劳动 + 并行竞争。**投影法（projection-based）**——直接遍历观测体素、把每体素投影到传感器坐标查所需光线；避免竞争、内存访问可预测、利于 GPU；但需完整位姿与投影模型、难用于无序点云。
- **ESDF（§5.2.2 行 115–117、§5.4.4 行 321）**：欧氏符号距离场，查询点到最近表面的距离（量值）+ 内外（符号）；用于碰撞检测、可导（高质量邻近梯度，利于优化运动规划）。已知表面用 Fast Marching 算；voxblox [822] 增量从 TSDF 建 ESDF（brushfire），FIESTA [426] 从占据图增量建 ESDF。

---

# 11 TSDF 地图与 Fusion 系列（主源 §12.5，行 1168–1255）+ 辅源 5/6

## 11.1 实时三维重建 vs SLAM（行 1170–1178）
本节是与 SLAM 相似但稍不同的方向：**实时三维重建**（涉 GPU 编程，无演示，可选阅读）。
- 之前地图模型**以定位为主体**，地图拼接作后续加工放进 SLAM 框架（定位轻量可实时、地图加工可在关键帧处理无须实时）。
- 现有做法**未对稠密地图优化**：两幅图都看到同一椅子时，只按两位姿叠加点云；位姿有误差→直接拼接不准（同一椅子点云无法完美叠加），出现**"鬼影"（重影）**。
- 实时重建**以建图为主体、定位次要**：把重建准确地图作主目标，需 GPU 甚至多 GPU 并行、较重设备；而 SLAM 朝轻量小型化发展（有的甚至只保留 VO）。实时重建朝大规模、大型动态场景重建发展。
- 自 RGB-D 出现，产生 **Kinect Fusion [134]、Dynamic Fusion [135]、Elastic Fusion [136]、Fusion4D [137]、Volume Deform [138]** 等。Kinect Fusion 完成基本模型重建（限小型场景），后续向大型/运动/变形场景拓展。

> **图 12-13**（行 1188–1227）各种实时三维重建模型：(a) Kinect Fusion；(b) Dynamic Fusion；(c) Volume Deform；(d) Fusion4D；(e) Elastic Fusion。

## 11.2 TSDF 定义（行 1180–1184，图 12-14）
**TSDF** = Truncated Signed Distance Function（截断符号距离函数）。与八叉树相似，也是网格式（方块式）地图。先选要建模的三维空间（如 $3\,\mathrm m\times3\,\mathrm m\times3\,\mathrm m$），按分辨率分许多小块，存每块信息。不同：**TSDF 整个存在显存（非内存）**，利用 GPU 并行对每体素计算更新。

每个 TSDF 体素存**该小块与距其最近物体表面的距离**：小块在表面**前方**→正值；表面**后方**→负值。物体表面通常很薄，故把太大/太小值都取成 **1 和 -1**（截断）。按定义 **TSDF 为 0 处即表面本身**；或因数值误差，**TSDF 由负变正处即表面**。

> **图 12-14**（行 1232–1255）TSDF 示意：$d_x=d_y=d_z=3\,[\mathrm{meters}]$；$V_x=V_y=V_z=\{32,64,128,256,512\}\,[\text{voxels}]$；相机观察物体表面时形成截断距离值。

## 11.3 TSDF 的"定位"与"建图"（行 1186、1230）
TSDF 也有定位与建图（似 SLAM 但形式不同）：
- **定位**：把当前 RGB-D 图与显存中 TSDF 地图比较、估相机位姿。传统还会对 RGB-D 先做一次**双边贝叶斯滤波**去深度噪声。
- **建图**：据估计位姿更新 TSDF。
- TSDF 定位**类似 ICP**，但因 GPU 并行可对**整张深度图与 TSDF 地图**做 ICP，无须先算特征点；TSDF 无颜色→**只用深度图、不用彩色图**即可估位姿，**摆脱了 VO 对光照纹理的依赖**，使 RGB-D 重建更稳健。建图部分是并行更新 TSDF 数值的过程，使表面更平滑可靠。

> **习题 5（行 1271）**：研究文献 [134]（KinectFusion），探讨 TSDF 如何位姿估计和更新，与之前定位建图算法的异同。

## 11.4 辅源补：TSDF 的精确融合公式（Curless-Levoy / KinectFusion，Handbook §5.2.2/§5.4.4 + 辅源 6）

14 讲只定性说 TSDF，**未给融合式**。补全：

**投影符号距离（projective SDF）**：给定一条穿过查询体素中心 $\mathbf p$ 的测量光线、深度读数 $\text{depth}$，体素沿光线的符号距离
$$
\eta=\lambda^{-1}\bigl(\text{depth}-z_{\text{voxel}}\bigr),\qquad \lambda=\|\,K^{-1}\mathbf x\,\|\ (\text{光线方向归一化因子}),
$$
（$\eta>0$＝体素在表面前方/相机侧，$\eta<0$＝表面后方。投影距离**高估**欧氏距离，但**零交叉（表面）位置正确**。）

**截断函数（TSDF 取值 $[-1,1]$ 或 $[-\mu_{tr},\mu_{tr}]$）**：
$$
\psi(\eta)=\operatorname{clamp}\!\left(\frac{\eta}{\mu_{tr}},\,-1,\,1\right)=
\begin{cases}-1,&\eta<-\mu_{tr},\\[1mm]\eta/\mu_{tr},&|\eta|\le\mu_{tr},\\[1mm]+1,&\eta>\mu_{tr},\end{cases}
$$
$\mu_{tr}$＝截断带（truncation band）。

**加权运行平均融合（Curless-Levoy 1996 / KinectFusion 2011）**：第 $k$ 帧用测量 $\psi_k(\mathbf p)$（权 $w_k(\mathbf p)$）更新体素：
$$
\boxed{\ F_k(\mathbf p)=\frac{W_{k-1}(\mathbf p)\,F_{k-1}(\mathbf p)+w_k(\mathbf p)\,\psi_k(\mathbf p)}{W_{k-1}(\mathbf p)+w_k(\mathbf p)},\qquad
W_k(\mathbf p)=\min\!\bigl(W_{k-1}(\mathbf p)+w_k(\mathbf p),\,W_{\max}\bigr).\ }
$$
（$F$＝融合后 TSDF 值、$W$＝累计权重；$W_{\max}$ 上限使旧测量缓慢遗忘、利于处理动态/漂移。权重常取与深度可信度相关，KinectFusion 简化为 $w_k=1$。）**表面提取**：对融合后 TSDF 用**光线投射（ray casting）**找零交叉，或用 **Marching Cubes** 抽网格。

> **Handbook §5.2.2/§5.4.4 关于 TSDF 的工程要点**：距离场须同时估**正距离**与表面后的**负距离**（表面＝零交叉）；为限制融合不完美正负距离的误差，更新**截断到表面附近的小截断带**。隐患：**薄物体从两侧观测时**，正（观测）与负（幻觉）距离平均会使零交叉翻转/消失（"擦除几何"）；可减轻不可消除。TSDF 在光滑表面优于占据法，但薄物体召回率低。**TSDF 严格高估欧氏距离**（安全隐患）→ voxblox 增量建 ESDF 补救。
>
> **⚠️ 上述 TSDF 公式为 Curless-Levoy/KinectFusion 公认标准形式**（经辅源 6 GitHub gist + Handbook §5.2.2 文字印证：Curless-Levoy 用"简单加权平均合并所有测量的投影符号距离"，KinectFusion 用"投影 TSDF + 加权运行平均"），**未逐字核到原 PDF 式号**。

---

# 12 工程权衡汇总（贯穿主源 + Handbook §5.5）

> 把散落各处的工程取舍集中，供综合写"工程实践/选型"节。

**单目稠密的固有难点（主源 §12.3.x）**：
1. **纹理依赖**：均匀区域（墙、光滑面）块匹配失效——立体视觉通病，理论性困难。
2. **梯度⊥极线**：梯度垂直极线时沿极线匹配度恒定，无有效匹配；应按"梯度-极线夹角"建更精确不确定度。
3. **旋转敏感**：块灰度不变假设在旋转时失效——块匹配前做仿射变换 (12.15)。
4. **效率**：几十万像素，CPU 难实时；像素间独立→GPU 并行（30 万线程≈算一像素）。
5. **外点**：简单 NCC 阈值法不显式处理外点；应用均匀-高斯混合滤波器（§5）显式建模内/外点。
6. **参数化**：正深度非对称、远点数值难→**逆深度**更稳（§6′）。

**地图形式的存储/能力对比（主源数值）**：
| 地图形式 | 典型存储（5 图/同场景） | 能定位 | 能导航避障 | 可视化 | 处理动态 |
| --- | --- | --- | --- | --- | --- |
| 点云图 | 6.9 MB（130 万点；体素 0.03 后 3 万点≈2%） | 仅点云 ICP | ✗（不能查占据） | 基础（无表面/法线） | ✗（只增不删） |
| 网格/surfel | （由点云转得） | — | — | 好（有表面/纹理/法线） | — |
| 八叉树 OctoMap | **56 KB（<点云 1%）** | 可查占据→定位辅助 | ✅（查任意点占据） | 一般（按高度染色） | ✅（光线更新可删） |
| TSDF | 显存、$V^3$ 体素（如 $512^3$） | 深度图↔TSDF 的 ICP（不需彩色） | ✅（隐式表面/距离） | 优（光滑、无鬼影） | 部分（Dynamic/Elastic Fusion） |

**Handbook §5.5 选型准则（行 426–434）**：
- **结构化 vs 非结构化环境**：受控空间（自动化工厂）用为任务/物体定制的地图更高效准；变/未知环境需区分**已观测自由空间 vs 未观测空间**（规划器避免穿未观测空间）——**占据法/隐式曲面法能区分**，而点/surfel/网格等**显式表面法不能**。
- **可扩展性**：显式表示（只描述表面）通常比隐式更省内存、保真度可按需调；需自由空间信息时多分辨率法优于固定分辨率体素（精度/内存/薄物体）。
- **任务匹配（表 5.2）**：操作沿**表面**还是在**笛卡尔空间**是最大区别。**隐式表示**便于高效滤波笛卡尔属性（如占据）、利于融合噪声深度（RGB-D）；**显式表示**便于滤波沿表面属性（如视觉纹理）、能造细腻可视化但对深度质量更敏感。

---

# 13 地图表示分类总表（Handbook §5.3.1 / 表 5.1 / 表 5.2）

**显式 / 隐式 × 表面 / 体积 四象限（§5.3.1 行 131–135，图 5.4）**：
| | 显式（explicit） | 隐式（implicit） |
| --- | --- | --- |
| **表面（surface）** | 点云、Mesh、Surfel | 隐式曲面（SDF/TSDF/ESDF 的零交叉） |
| **体积（volume）** | 占据体素、距离体素 | GP 图、Hilbert 图、神经隐式（SDF 网络） |

**Table 5.1（方法 vs 空间抽象，§5.4 行 263–267）**：§5.4.1 Points→Surface；§5.4.2 Surfels→Surface；§5.4.3 Mesh→Surface(connected)；§5.4.4 Voxels→Occupancy 或 Implicit surface；§5.4.5–5.4.6 Continuous function→Occupancy 或 Implicit surface。

**两大估计量（§5.2 Q1 行 68）**：**占据（occupancy）**——区分自由/占据空间，二分类预测每栅格占据概率；**距离（distance）**——更"机器人中心"的解读，测到最近表面/物体的距离（SDF/TSDF/ESDF）。

**占据 vs 距离场（§5.2.3 行 121–125）**：
- **建模直接性**：测量光线直接告知自由/占据/未观测→占据图用更少启发式更新；隐式曲面建表面距离，部分测量无法精确算→依赖 TSDF 等距离代理。
- **光滑性**：隐式曲面天然比占据图（二值属性）光滑→**可导**（邻近梯度有用）、减离散化误差、可亚像素插值；但不连续无法光滑表示→**易漏薄障碍**。

**Range 传感器测量模型（§5.1.1，式 5.1）**：飞行时间测距
$$
r=\frac{c\,(t_{\text{detect}}-t_{\text{emit}})}{2}.
\tag{5.1}
$$
**LiDAR→点云（§5.1.2，式 5.2–5.4）**：每束内参 $(\phi_{i,j},\theta_{i,j})$（方位角 $\phi\in[0,2\pi]$、极角 $\theta\in[-\pi,\pi]$），距离 $r_{i,j}$ 转点：
$$
x=r_{i,j}\cos\theta_{i,j}\cos\phi_{i,j},\quad
y=r_{i,j}\cos\theta_{i,j}\sin\phi_{i,j},\quad
z=r_{i,j}\sin\theta_{i,j}.
\tag{5.2–5.4}
$$
**深度图→点云（§5.1.2，式 5.5）**：$p=K^{\dagger}\,x$（$K^\dagger$＝内参伪逆/反投影）。

**连续函数表示（§5.2.x/§5.4.5–5.4.6）**：GP 图（GPOM [816]）、Hilbert 图 [904]——不离散化、能插值缺失数据、建模观测间相关、量化不确定度（GP 可对估计量线性运算仍得 GP→梯度概率化），但 GP 计算昂贵。神经隐式（iSDF/Sucar [1047] 等）用网络预测任意点 SDF，特征存体素/八叉树/点、小神经解码器出 SDF，利于大场景。

---

# 14 全章习题（主源 §习题，行 1261–1271，逐字全录）

1. **推导式 (12.6)。**（高斯乘积融合——完整证明见本抽取 §4.2 框注。）
2. **把本讲的稠密深度估计改成半稠密**，可先把梯度明显的地方筛选出来。（即"半稠密直接法"思路：只在高梯度像素估深度。）
3. **\*把本讲演示的单目稠密重建代码从正深度改成逆深度，并添加仿射变换。** 实验效果是否有改进？（逆深度见 §6′，仿射变换见式 12.15。）
4. **你能论证如何在八叉树中进行导航或路径规划吗？**（提示：八叉树可 O(1)~O(树高) 查询任意点占据概率，供 A\*/RRT 等查"可通过性"，见 §10.5。）
5. **\*研究参考文献 [134]（KinectFusion），探讨 TSDF 地图是如何进行位姿估计和更新的，它和我们之前讲过的定位建图算法有何异同。**（位姿＝深度图↔TSDF 的 GPU-ICP；更新＝加权运行平均，见 §11.4。）

---

# 15 OCR / 原文勘误汇总（综合时务必采纳更正）

| 处 | 原文（OCR） | 问题 | 更正 | 依据 |
| --- | --- | --- | --- | --- |
| 式 12.11 | $\bar{\boldsymbol A}(i,j),\bar{\boldsymbol B}(i,j)$ | 均值是标量、不应带 $(i,j)$ | $\bar A,\bar B$（块均值标量） | ZNCC 定义 |
| 式 12.14 末项 | $-\boldsymbol K\boldsymbol R_{CW}\boldsymbol R_{RW}^{\mathrm T}\boldsymbol K\boldsymbol t_{RW}$ | 中间 $\boldsymbol K$ 量纲错（应与首项 $K^{-1}$ 一致） | 联立 (12.12)(12.13) 重推得 $+\boldsymbol K(\boldsymbol t_{CW}-\boldsymbol R_{CW}\boldsymbol R_{RW}^{\mathrm T}\boldsymbol t_{RW})$ | 见 §6.10.4 框注 |
| **式 12.18** | $L(n\mid z_{1:t+1})=L(n\mid z_{1:t-1})+L(n\mid z_t)$ | 下标 $t-1,t$ 与左端 $t+1$ **不自洽**（严重） | $L(n\mid z_{1:t+1})=L(n\mid z_{1:t})+L(n\mid z_{t+1})$ | Handbook 式 5.6 |
| §12.4.4 代码路径 | `slambook/ch13/dense_RGBD/octomap_mapping.cpp` | 本讲是 ch12，路径写成 ch13 | 应为 `ch12`（与同节 surfel_mapping 一致）| 上下文 |
| 内参 `fy=-480.0` | 负号 | 非 OCR 错，是 **ICL-NUIM/REMODE 数据集约定**（图像 y 轴朝上 → fy 取负使 v 轴方向一致） | **保留**，但综合时加一句脚注解释，避免读者误删负号 | 数据集惯例 |
| §12.5 "Volumn Deform" | Volumn | 拼写（应 Volume）| Volume Deform | 论文名 |
| 行 280 等 `demoniator` | demoniator | 变量名拼写（denominator）| 保留原样（是源码变量名，改了会与原码不符）| 源码 |

> **附：术语标准化（综合时统一用词）**：SAD/SSD/NCC/ZNCC（去均值 NCC）、极线搜索（epipolar search）、块匹配（block matching）、深度滤波器（depth filter）、逆深度（inverse depth）、占据栅格（occupancy grid）、对数几率（log-odds）、八叉树（octree/OctoMap）、面元（surfel）、截断符号距离函数（TSDF）、欧氏符号距离场（ESDF）、加权运行平均（weighted running average）、Marching Cubes、泊松重建（Poisson reconstruction）、半全局匹配（SGM）。

---

## 附：参考文献条目（供综合 agent 合入 refs.bib，本抽取不写 refs.bib）

> 现有键见编排记忆；本主题**新增建议键**（综合 agent 核对后由主控集中合并，避免并发写 refs.bib）：

- `gaoxiang2019slam14`（**已存在**）——高翔《视觉SLAM十四讲》第 12 讲，主源。
- `carlone2026handbook`（**已存在**）——SLAM Handbook，其 Ch.5 Dense Map Representations 为辅源 5。
- 建议新增 `pizzoli2014remode` —— Pizzoli, Forster, Scaramuzza, "REMODE: Probabilistic, Monocular Dense Reconstruction in Real Time," ICRA 2014. https://rpg.ifi.uzh.ch/docs/ICRA14_Pizzoli.pdf
- 建议新增 `vogiatzis2011video` —— Vogiatzis & Hernández, "Video-based, real-time multi-view stereo," Image and Vision Computing 29(7):434–441, 2011.
- 建议新增 `forster2017svo` —— Forster, Zhang, Gassner, Werlberger, Scaramuzza, "SVO: Semidirect Visual Odometry for Monocular and Multicamera Systems," IEEE TRO 33(2):249–265, 2017.
- 建议新增 `hirschmuller2008sgm` —— Hirschmüller, "Stereo Processing by Semiglobal Matching and Mutual Information," IEEE TPAMI 30(2):328–341, 2008.
- 建议新增 `curless1996volumetric` —— Curless & Levoy, "A Volumetric Method for Building Complex Models from Range Images," SIGGRAPH 1996.
- 建议新增 `newcombe2011kinectfusion` —— Newcombe et al., "KinectFusion: Real-Time Dense Surface Mapping and Tracking," ISMAR 2011.
- 建议新增 `hornung2013octomap` —— Hornung, Wurm, Bennewitz, Stachniss, Burgard, "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees," Autonomous Robots 34(3):189–206, 2013.
- （主源引用编号映射，供综合追溯）：[121]=REMODE/极线块匹配；[124]=Vogiatzis-Hernández 均匀-高斯混合；[126,127]=逆深度（Civera 等）；[128]=Poisson 重建；[129]=Surfel；[130]=MLS；[131]=Greedy Projection；[132]=OctoMap；[134]=KinectFusion；[135]=DynamicFusion；[136]=ElasticFusion；[137]=Fusion4D；[138]=VolumeDeform；[75]=DTAM/几何+光度不确定度。
