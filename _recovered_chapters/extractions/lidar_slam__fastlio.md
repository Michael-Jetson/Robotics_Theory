# 抽取留痕：FAST-LIO / FAST-LIO2 紧耦合 LiDAR-惯性里程计（迭代 ESKF、ikd-Tree、运动补偿、完整状态方程）

> **源 1（FAST-LIO）**：Wei Xu, Fu Zhang, *FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter*, **IEEE Robotics and Automation Letters (RA-L)**, 2021；arXiv:**2010.08196**v3 [cs.RO], 14 Apr 2021。
> **PDF**：https://arxiv.org/pdf/2010.08196 ；**摘要页**：https://arxiv.org/abs/2010.08196 ；**HTML(ar5iv)**：https://ar5iv.labs.arxiv.org/html/2010.08196 ；**官方代码**：https://github.com/hku-mars/FAST_LIO
>
> **源 2（FAST-LIO2）**：Wei Xu*, Yixi Cai*, Dongjiao He, Jiarong Lin, Fu Zhang, *FAST-LIO2: Fast Direct LiDAR-inertial Odometry*, **IEEE Transactions on Robotics (T-RO)**, 2022；arXiv:**2107.06829**v1 [cs.RO], 14 Jul 2021。
> **PDF**：https://arxiv.org/pdf/2107.06829 ；**摘要页**：https://arxiv.org/abs/2107.06829 ；**官方代码（含 ikd-Tree）**：https://github.com/hku-mars/ikd-Tree
>
> **源 3（背景/底层理论，被两文引用）**：
> - **[Hertzberg 2013]** C. Hertzberg, R. Wagner, U. Frese, L. Schröder, *Integrating generic sensor fusion algorithms with sound state representations through encapsulation of manifolds*, Information Fusion 14(1):57–77, 2013（⊞/⊟ 流形封装算子的原始出处，两文中编号 [23]）。
> - **[IKFOM]** W. Xu et al.（FAST-LIO2 文中 [55]）即 IKFOM toolbox：*Kalman Filters on Differentiable Manifolds*，提供 ⊞/⊟ 上 KF 的"抽象推导"。FAST-LIO2 的具体 $F_{\tilde x},F_w$ 矩阵推导明确指向 FAST-LIO [22] 与 IKFOM [55]。
> - **[Bullo-Murray 1995]** F. Bullo, R. M. Murray, *Proportional derivative (PD) control on the Euclidean group*, ECC 1995（$A(\cdot)$ 函数的出处，FAST-LIO 文中 [25]）。
>
> **服务章节**：激光SLAM（激光里程计与建图；scan-to-scan/scan-to-map；松/紧耦合 LIO；FAST-LIO 迭代 ESKF 完整推导；回环与位姿图）。
> **抽取人备注**：本文件是【内部抽取留痕】，不是成书正文；遵循「禁摘要·全量保真」铁律。所有公式用 LaTeX 写全，保留全部式号与条件，并显式记录记号约定与本书统一约定的差异。FAST-LIO/FAST-LIO2 主体来自论文全文（含两附录、两算法、ikd-Tree 全部伪码与复杂度引理）；末尾 §X「本源未覆盖项」专门列出本主题中两文不含、需他源补齐的内容（尤其【回环与位姿图】——FAST-LIO/2 是纯里程计，不含回环）。

---

## §0 记号约定（本两源）与本书统一约定的差异

> 汇总自 FAST-LIO §III-A/B（Table I）与 FAST-LIO2 §III/IV。逐项对照本书统一约定（编写规范 §五：$\mathbf R\in\mathrm{SO}(3)$、Hamilton 四元数、**右扰动为主**、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前、协方差字母）。

| 项目 | FAST-LIO / FAST-LIO2 本源约定 | 本书统一约定 | 差异/转换说明 |
|---|---|---|---|
| 旋转表示 | **旋转矩阵** $\mathbf R\in\mathrm{SO}(3)$，记 ${}^G\mathbf R_I$（IMU 在全局系姿态）。**不用四元数**（状态直接定义在流形 $\mathrm{SO}(3)$ 上，靠 ⊞/⊟ 避免重归一化）。 | $\mathbf R\in\mathrm{SO}(3)$ | **一致**。本源完全用矩阵 + 流形封装，省去四元数 flavor 问题。 |
| 旋转作用 | ${}^G\mathbf p={}^G\mathbf R_I\,{}^I\mathbf p$（active，把体系坐标转到全局系） | 同 | 一致。 |
| 扰动方向（**关键**） | **左扰动 / 全局角误差**：误差态 $\delta\boldsymbol\theta=\mathrm{Log}({}^G\bar{\mathbf R}_I^\top\,{}^G\mathbf R_I)$，即 ${}^G\mathbf R_I={}^G\bar{\mathbf R}_I\,\mathrm{Exp}(\delta\boldsymbol\theta)$。**注意**：从 $\bar{\mathbf R}^\top\mathbf R$ 的写法看，这是把扰动放在**估计值右侧**的局部坐标定义，FAST-LIO 用 ⊟ 定义 $\mathbf R_1\ominus\mathbf R_2=\mathrm{Log}(\mathbf R_2^\top\mathbf R_1)$，对应 $\mathbf R_1=\mathbf R_2\,\mathrm{Exp}(\cdot)$ —— **即本源用的是右乘扰动（局部坐标）**，与本书"右扰动为主"一致。 | **右扰动为主** $\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$（局部坐标） | **一致**（本源 ⊞/⊟ 的 SO(3) 实现就是右乘：$\mathbf R\boxplus\mathbf r=\mathbf R\,\mathrm{Exp}(\mathbf r)$）。综合时无需翻转扰动侧。 |
| 角扰动符号 | $\delta\boldsymbol\theta\in\mathbb R^3$（FAST-LIO2 中具体写 $\delta{}^G\boldsymbol\theta_{I_k}$、$\delta{}^I\boldsymbol\theta_{L_k}$ 区分 IMU 姿态误差与外参旋转误差） | $\delta\boldsymbol\phi$（右扰动，局部坐标） | 仅符号差异：本源 $\delta\boldsymbol\theta$↔本书 $\delta\boldsymbol\phi$。 |
| ⊞/⊟ 算子 | $\mathbf R\boxplus\mathbf r=\mathbf R\,\mathrm{Exp}(\mathbf r)$（右），$\mathbf R_1\boxminus\mathbf R_2=\mathrm{Log}(\mathbf R_2^\top\mathbf R_1)$；$\mathbb R^n$ 上 $\mathbf a\boxplus\mathbf b=\mathbf a+\mathbf b$。出处 [Hertzberg 2013]。 | 同（右 ⊞，编写规范用 $\oplus/\ominus$ 或 $\boxplus/\boxminus$） | **一致**（这是本源最核心的记号，整套迭代 ESKF 都建在 ⊞/⊟ 上）。 |
| 指/对数映射 | 大写 $\mathrm{Exp}:\mathbb R^3\to\mathrm{SO}(3)$（Rodrigues），$\mathrm{Log}$ 为其逆。 | 同（Exp/Log 大写约定） | 一致。 |
| 反对称算子 | $\lfloor\mathbf a\rfloor_\wedge$（skew-symmetric / cross-product matrix，$\lfloor\mathbf a\rfloor_\wedge\mathbf b=\mathbf a\times\mathbf b$） | $[\cdot]_\times$ 或 $(\cdot)^\wedge$ | 一致；本源用 $\lfloor\cdot\rfloor_\wedge$ 记号。 |
| 状态排序（FAST-LIO，18 维） | $\mathbf x=[{}^G\mathbf R_I^\top,\;{}^G\mathbf p_I^\top,\;{}^G\mathbf v_I^\top,\;\mathbf b_\omega^\top,\;\mathbf b_a^\top,\;{}^G\mathbf g^\top]^\top$。**姿态在前、位置在中、速度、陀螺零偏、加速度计零偏、重力**。 | $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前（仅针对 $\mathfrak{se}(3)$ 单刚体位姿） | 这是**导航全状态向量**，与本书 $\mathfrak{se}(3)$ 的 $[\boldsymbol\rho;\boldsymbol\phi]$ 是不同层次；本源把**姿态(旋转)放在最前**（与 Solà 的位置在前不同）。综合若拼 $\mathfrak{se}(3)$ 子块需注意：本源是 $[\mathbf R;\mathbf p]$ 序。 |
| 状态排序（FAST-LIO2，24 维） | 在 18 维基础上**追加 LiDAR-IMU 外参** ${}^I\mathbf R_L,{}^I\mathbf p_L$：$\mathbf x=[{}^G\mathbf R_I^\top,{}^G\mathbf p_I^\top,{}^G\mathbf v_I^\top,\mathbf b_\omega^\top,\mathbf b_a^\top,{}^G\mathbf g^\top,{}^I\mathbf R_L^\top,{}^I\mathbf p_L^\top]^\top$，$\dim=24$，流形 $\mathcal M=\mathrm{SO}(3)\times\mathbb R^{15}\times\mathrm{SO}(3)\times\mathbb R^3$。 | — | FAST-LIO2 把外参纳入在线标定。 |
| 协方差字母 | $\mathbf P$（误差态协方差，切空间）；过程噪声协方差 $\mathbf Q$；测量噪声协方差 $\mathbf R$（**注意与旋转矩阵 $\mathbf R$ 同字母**，靠上下文/上标区分：测量噪声 $\mathbf R_j$、$\mathbf R=\mathrm{diag}(\dots)$ 无上标几何标记）；估计量上标 $\hat\cdot$（预测/传播）、$\bar\cdot$（更新后）、$\check\cdot$（反向传播相对量）、$\tilde\cdot$（误差）。 | $\mathbf P$ / $\mathbf R$(测量) / $\mathbf Q$(过程) | **几乎一致**；唯一坑：测量噪声 $\mathbf R$ 与旋转矩阵 $\mathbf R$ 撞字母（与 Solà 用 $\mathbf V$ 避让的处理相反）。综合写正文时建议测量噪声改记 $\mathbf R_{\text{meas}}$ 或 $\boldsymbol\Sigma_{\text{meas}}$ 以免歧义。 |
| 增益 / 测量雅可比 | $\mathbf K$（卡尔曼增益）；$\mathbf H$（测量雅可比，对**误差态** $\tilde{\mathbf x}$ 求导，在 $\tilde{\mathbf x}=0$ 处取值） | $\mathbf K$ / $\mathbf H$ | 一致；本源 $\mathbf H$ 是对误差态求导（与 ESKF 通例一致）。 |
| 迭代上标 | $\hat{\mathbf x}_k^\kappa$ = 迭代 ESKF 第 $\kappa$ 次迭代的状态估计；$\kappa=0$ 时 $\hat{\mathbf x}_k^0=\hat{\mathbf x}_k$（预测值）。 | — | 本源专有：iterated EKF 的内层迭代计数。 |
| 时间下标 | $t_k$=第 $k$ 个 LiDAR 扫描的扫描结束时刻；$\tau_i$=扫描内第 $i$ 个 IMU 采样时刻；$\rho_j$=扫描内第 $j$ 个特征点采样时刻；帧 $I_i,I_j,I_k$=对应时刻 IMU 体系，$L_j,L_k$=对应 LiDAR 体系。 | — | 本源专有（运动补偿核心记号）。 |

**本两源未使用/未涉及**（与本书其它源对照）：四元数运动学、信息滤波 vs 协方差滤波的对偶、RTS 平滑、批量优化/因子图（FAST-LIO/2 是**纯递归滤波里程计**）、**回环检测与位姿图优化**（两文均无）、可观测性分析（仅在被引文 [24] 中）。这些由综合 agent 从 Barfoot/其它 LIO-SAM 类源补齐。

---

# 第一部分：FAST-LIO（arXiv:2010.08196）全量抽取

## §1 引言与贡献 [源 FAST-LIO §I]

**问题背景**：SLAM 是移动机器人（如 UAV）的基础。视觉(惯性)里程计 VO（Stereo VO、Monocular VO）轻量低成本，但缺乏直接深度、重建 3D 环境耗算力、对光照敏感。LiDAR 可克服这些困难但此前对小型移动机器人太贵太大。固态 LiDAR（MEMS 扫描、旋转棱镜）近年兴起：成本接近全局快门相机、轻量（可上小 UAV）、高性能（长距离高精度的主动直接 3D 测量）。

**固态 LiDAR 给 SLAM 带来的新挑战**（源逐条）：
1. LiDAR 测量的特征点通常是几何结构（边、面）。在无强特征的杂乱环境中 LiDAR 方案易退化（degenerate），小 FoV 时尤甚。
2. 由于扫描方向高分辨率，一次 LiDAR 扫描通常含很多特征点（数千）。这些点在退化时不足以可靠定位，但把如此大量特征点紧耦合融合到 IMU 需要 UAV 板载计算机负担不起的巨大算力。
3. LiDAR 用少量激光/接收对**顺序采样**，一次扫描内的点总在不同时刻采样，导致**运动畸变（motion distortion）**，严重降低扫描配准质量。UAV 螺旋桨/电机的持续转动还给 IMU 带来显著噪声。

**贡献（源逐条，4 条）**：
1. 为应对快速运动、噪声或退化的杂乱环境，采用**紧耦合迭代卡尔曼滤波**融合 LiDAR 特征点与 IMU。提出一个**形式化的反向传播（back-propagation）过程**补偿运动畸变。
2. 为降低大量 LiDAR 特征点造成的计算负载，提出**计算卡尔曼增益的新公式**，并证明其与传统卡尔曼增益公式等价。新公式的计算复杂度依赖于**状态维数**而非**测量维数**。
3. 把上述公式实现为一个快速鲁棒的 LIO 软件包，可在小型四旋翼板载计算机上运行。
4. 在各种室内外环境与实际 UAV 飞行测试中验证系统在快速运动/强振动噪声下的鲁棒性。

## §2 相关工作 [源 FAST-LIO §II]

**A. LiDAR 里程计与建图**：Besl & McKay 的 **ICP**（迭代最近点）是扫描配准的基础，对稠密 3D 扫描效果好，但 LiDAR 稀疏点云的精确点匹配几乎不存在。Segal 等提出基于**点到面距离**的 generalized-ICP；Zhang & Singh 结合**点到边距离**发展出 **LOAM（LiDAR Odometry And Mapping）**。LOAM 变体：LeGO-LOAM、LOAM-Livox。这些方法在结构化环境与大 FoV LiDAR 上效果好，但对无特征环境或小 FoV LiDAR 很脆弱。

**B. 松耦合 LIO**：IMU 常用来缓解无特征环境的 LiDAR 退化。松耦合方法**分别处理** LiDAR 与 IMU 测量再融合结果。例：IMU-aided LOAM 用 IMU 积分位姿作扫描配准初值；Zhen 等用误差态 EKF 融合 IMU 与 LiDAR 的高斯粒子滤波输出；Balazadegan 等加 IMU-重力模型估 6-DOF 自运动辅助配准；Zuo 等用 **MSCKF（多状态约束卡尔曼滤波）**融合扫描配准结果与 IMU/视觉。松耦合通病：先配准新扫描得位姿测量、再与 IMU 融合，分离配准与融合降低算力，但**忽略了系统其它状态（如速度）与新扫描位姿的相关性**；且无特征时配准在某些方向退化，导致后续融合不可靠。

**C. 紧耦合 LIO**：不同于松耦合，紧耦合直接融合 LiDAR 的**原始特征点**（而非配准结果）与 IMU。两大类：**基于优化**与**基于滤波**。
- 优化类：Geneva 等用图优化 + IMU 预积分约束 + LiDAR 特征点的平面约束；Ye 等的 **LIOM** 用类似图优化但基于边/面特征。
- 滤波类：Bry 用**高斯粒子滤波 GPF** 融合 IMU 与 2D 平面 LiDAR（用于 Boston Dynamics Atlas 人形机器人）。但粒子滤波复杂度随特征点数与系统维数快速增长，故**卡尔曼滤波及其变体**更受偏好：EKF、UKF、迭代 KF。

**本文定位**：属紧耦合，采用类似 [21]（LINS）的**迭代扩展卡尔曼滤波**以减小线性化误差。卡尔曼滤波（及变体）时间复杂度为 $O(m^2)$（$m$ 为测量维数），处理大量 LiDAR 测量时计算负载极高。朴素降采样减少测量数但损失信息。LINS [21] 靠提取拟合地面平面减少测量数，但不适用于地面不总存在的空中应用。

## §3 方法：框架总览 [源 FAST-LIO §III-A]

工作流（Fig. 2(a)）：LiDAR 输入 → **特征提取模块**得平面与边特征 → 提取的特征 + IMU 测量 → **状态估计模块**在 10–50 Hz 做状态估计 → 估计的位姿把特征点配准到全局系并与至今所建特征点地图融合 → 更新的地图用于下一步配准新点。

### §3.1 流形与 ⊞/⊟ 算子 [源 FAST-LIO §III-B-1]

设 $\mathcal M$ 为所考虑的 $n$ 维流形（如 $\mathcal M=\mathrm{SO}(3)$）。由于流形局部同胚于 $\mathbb R^n$，可经两个**封装算子（encapsulation operators）** $\boxplus$、$\boxminus$（出处 [Hertzberg 2013]）建立 $\mathcal M$ 上局部邻域到其切空间 $\mathbb R^n$ 的双射：

$$\boxplus:\mathcal M\times\mathbb R^n\to\mathcal M;\qquad \boxminus:\mathcal M\times\mathcal M\to\mathbb R^n,$$

$$\mathcal M=\mathrm{SO}(3):\quad \mathbf R\boxplus\mathbf r=\mathbf R\,\mathrm{Exp}(\mathbf r),\qquad \mathbf R_1\boxminus\mathbf R_2=\mathrm{Log}(\mathbf R_2^\top\mathbf R_1),$$

$$\mathcal M=\mathbb R^n:\quad \mathbf a\boxplus\mathbf b=\mathbf a+\mathbf b,\qquad \mathbf a\boxminus\mathbf b=\mathbf a-\mathbf b,$$

其中指数映射（Rodrigues 公式，源原式）为

$$\mathrm{Exp}(\mathbf r)=\mathbf I+\frac{\mathbf r}{\lVert\mathbf r\rVert}\sin(\lVert\mathbf r\rVert)+\frac{\lfloor\mathbf r\rfloor_\wedge^2}{\lVert\mathbf r\rVert^2}\bigl(1-\cos(\lVert\mathbf r\rVert)\bigr),$$

> **抽取注**：源 pdftotext 把第二项写作 $\frac{\mathbf r}{\lVert\mathbf r\rVert}\sin$、第三项写作 $\frac{\mathbf r^2}{\lVert\mathbf r\rVert^2}(1-\cos)$。按标准 Rodrigues 与 ar5iv 渲染，第二项实际是 $\frac{\lfloor\mathbf r\rfloor_\wedge}{\lVert\mathbf r\rVert}\sin(\lVert\mathbf r\rVert)$、第三项是 $\frac{\lfloor\mathbf r\rfloor_\wedge^2}{\lVert\mathbf r\rVert^2}(1-\cos(\lVert\mathbf r\rVert))$（即 $\mathbf r/\lVert\mathbf r\rVert$ 应理解为 $\lfloor\mathbf r\rfloor_\wedge/\lVert\mathbf r\rVert$，因为只有反对称矩阵相加才与单位阵 $\mathbf I$ 同型）。上式已按标准形式写全。$\mathrm{Log}(\cdot)$ 是其逆映射。

复合流形 $\mathcal M=\mathrm{SO}(3)\times\mathbb R^n$ 的封装算子按分量定义：

$$\begin{bmatrix}\mathbf R\\\mathbf a\end{bmatrix}\boxplus\begin{bmatrix}\mathbf r\\\mathbf b\end{bmatrix}=\begin{bmatrix}\mathbf R\,\mathrm{Exp}(\mathbf r)\\\mathbf a+\mathbf b\end{bmatrix};\qquad \begin{bmatrix}\mathbf R_1\\\mathbf a\end{bmatrix}\boxminus\begin{bmatrix}\mathbf R_2\\\mathbf b\end{bmatrix}=\begin{bmatrix}\mathrm{Log}(\mathbf R_2^\top\mathbf R_1)\\\mathbf a-\mathbf b\end{bmatrix}.$$

**易验证的两个恒等式**（源给出，作为后续误差态推导的关键性质）：

$$(\mathbf x\boxplus\mathbf u)\boxminus\mathbf x=\mathbf u;\qquad \mathbf x\boxplus(\mathbf y\boxminus\mathbf x)=\mathbf y,\qquad \forall\,\mathbf x,\mathbf y\in\mathcal M,\;\forall\,\mathbf u\in\mathbb R^n.$$

### §3.2 连续时间模型 [源 FAST-LIO §III-B-2]

设 IMU 刚性固连于 LiDAR，外参 ${}^I\mathbf T_L=({}^I\mathbf R_L,{}^I\mathbf p_L)$ 已知。取 IMU 系（记 $I$）为参考体系，得运动学模型（源式 (1)）：

$$
{}^G\dot{\mathbf p}_I={}^G\mathbf v_I,\qquad
{}^G\dot{\mathbf v}_I={}^G\mathbf R_I(\mathbf a_m-\mathbf b_a-\mathbf n_a)+{}^G\mathbf g,\qquad
{}^G\dot{\mathbf g}=\mathbf 0,
$$
$$
{}^G\dot{\mathbf R}_I={}^G\mathbf R_I\,\lfloor\boldsymbol\omega_m-\mathbf b_\omega-\mathbf n_\omega\rfloor_\wedge,\qquad
\dot{\mathbf b}_\omega=\mathbf n_{b\omega},\qquad
\dot{\mathbf b}_a=\mathbf n_{ba}.
\tag{1}
$$

符号（源逐条）：${}^G\mathbf p_I,{}^G\mathbf R_I$ 为 IMU 在**全局系**（即第一个 IMU 帧，记 $G$）中的位置与姿态；${}^G\mathbf g$ 是全局系中**未知重力向量**；$\mathbf a_m,\boldsymbol\omega_m$ 是 IMU 测量（加速度计、陀螺）；$\mathbf n_a,\mathbf n_\omega$ 是 IMU 测量的**白噪声**；$\mathbf b_a,\mathbf b_\omega$ 是 IMU 零偏，建模为受高斯噪声 $\mathbf n_{ba},\mathbf n_{b\omega}$ 驱动的**随机游走过程**；$\lfloor\mathbf a\rfloor_\wedge$ 是向量 $\mathbf a\in\mathbb R^3$ 的反对称（叉积）矩阵。

> **关键建模点**：重力 ${}^G\mathbf g$ 被当作**待估状态**（$\dot{}^G\mathbf g=0$），而非固定常量。这避免了对初始姿态对齐重力方向的强假设，靠滤波在线估计重力向量。

### §3.3 离散模型（零阶保持）[源 FAST-LIO §III-B-3]

基于 ⊞ 运算，把连续模型 (1) 在 IMU 采样周期 $\Delta t$ 上用**零阶保持器**离散，得离散模型（源式 (2)）：

$$\mathbf x_{i+1}=\mathbf x_i\boxplus\bigl(\Delta t\,\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\bigr),\tag{2}$$

其中 $i$ 是 IMU 测量索引；函数 $\mathbf f$、状态 $\mathbf x$、输入 $\mathbf u$、噪声 $\mathbf w$ 定义如下（源式 (3)）：

$$\mathcal M=\mathrm{SO}(3)\times\mathbb R^{15},\qquad \dim(\mathcal M)=18,$$

$$\mathbf x\triangleq\begin{bmatrix}{}^G\mathbf R_I^\top & {}^G\mathbf p_I^\top & {}^G\mathbf v_I^\top & \mathbf b_\omega^\top & \mathbf b_a^\top & {}^G\mathbf g^\top\end{bmatrix}^\top\in\mathcal M,$$

$$\mathbf u\triangleq\begin{bmatrix}\boldsymbol\omega_m^\top & \mathbf a_m^\top\end{bmatrix}^\top,\qquad \mathbf w\triangleq\begin{bmatrix}\mathbf n_\omega^\top & \mathbf n_a^\top & \mathbf n_{b\omega}^\top & \mathbf n_{ba}^\top\end{bmatrix}^\top,$$

$$\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)=\begin{bmatrix}
\boldsymbol\omega_{m_i}-\mathbf b_{\omega_i}-\mathbf n_{\omega_i}\\[2pt]
{}^G\mathbf v_{I_i}\\[2pt]
{}^G\mathbf R_{I_i}(\mathbf a_{m_i}-\mathbf b_{a_i}-\mathbf n_{a_i})+{}^G\mathbf g_i\\[2pt]
\mathbf n_{b\omega_i}\\[2pt]
\mathbf n_{ba_i}\\[2pt]
\mathbf 0_{3\times1}
\end{bmatrix}.\tag{3}$$

> **结构解读**：$\mathbf f$ 的六个块依次对应状态六块 $({}^G\mathbf R_I,{}^G\mathbf p_I,{}^G\mathbf v_I,\mathbf b_\omega,\mathbf b_a,{}^G\mathbf g)$ 的"切空间增量率"。第一块（角速度去偏去噪）经 ⊞ 的 $\mathrm{Exp}$ 作用于旋转；第六块为 0 表示重力按常量传播。

### §3.4 LiDAR 测量预处理 [源 FAST-LIO §III-B-4]

LiDAR 测量是其局部体系中的点坐标。原始点采样率极高（如 200 kHz），不可能每收到一个点就处理一次。实用做法是**累积一段时间后一次性处理**。FAST-LIO 中最小累积间隔设为 **20 ms**，得最高 **50 Hz** 全状态估计（即里程计输出）与地图更新（Fig. 2(a)）。这样累积的一组点称为一个**扫描（scan）**，处理它的时刻记 $t_k$（Fig. 2(b)）。从原始点提取**高局部光滑度的平面点**（[8] LOAM）与**低局部光滑度的边点**（[10] LOAM-Livox）。设特征点数为 $m$，每个在时刻 $\rho_j\in(t_{k-1},t_k]$ 采样，记 ${}^{L_j}\mathbf p_{f_j}$（$L_j$ 为 $\rho_j$ 时刻的 LiDAR 局部系）。一次扫描内也有多个 IMU 测量，各在 $\tau_i\in[t_{k-1},t_k]$ 采样、对应状态 $\mathbf x_i$（见 (2)）。注意**最后一个 LiDAR 特征点是扫描末尾**（$\rho_m=t_k$），而 IMU 测量不一定与扫描起止对齐。

## §4 状态估计：迭代扩展卡尔曼滤波（迭代 ESKF）[源 FAST-LIO §III-C]

为估计 (2) 中状态，用**迭代扩展卡尔曼滤波**。把估计协方差刻画在**状态估计的切空间**中（[Hertzberg 2013]、[24]）。设上一 LiDAR 扫描（$t_{k-1}$ 时刻）的最优状态估计为 $\bar{\mathbf x}_{k-1}$、协方差矩阵 $\bar{\mathbf P}_{k-1}$。则 $\bar{\mathbf P}_{k-1}$ 表示如下**随机误差态向量**的协方差：

$$\tilde{\mathbf x}_{k-1}\triangleq\mathbf x_{k-1}\boxminus\bar{\mathbf x}_{k-1}=\begin{bmatrix}\delta\boldsymbol\theta^\top & {}^G\tilde{\mathbf p}_I^\top & {}^G\tilde{\mathbf v}_I^\top & \tilde{\mathbf b}_\omega^\top & \tilde{\mathbf b}_a^\top & {}^G\tilde{\mathbf g}^\top\end{bmatrix}^\top,$$

其中 $\delta\boldsymbol\theta=\mathrm{Log}({}^G\bar{\mathbf R}_I^\top\,{}^G\mathbf R_I)$ 是**姿态误差**，其余是标准加性误差（即量 $x$ 在估计 $\bar x$ 处的误差 $\tilde x=x-\bar x$）。直觉上 $\delta\boldsymbol\theta$ 描述真值姿态与估计姿态之间的（小）偏差。这种误差定义的主要优点是**可用 $3\times3$ 协方差矩阵 $\mathbb E[\delta\boldsymbol\theta\,\delta\boldsymbol\theta^\top]$ 表示姿态不确定性**。由于姿态有 3 DOF，这是**最小表示**（避免了 $\mathrm{SO}(3)$ 过参数化导致的奇异协方差）。

### §4.1 前向传播（Forward Propagation）[源 FAST-LIO §III-C-1]

每收到一个 IMU 输入就做一次前向传播（Fig. 2）。状态按 (2) 传播，过程噪声 $\mathbf w_i$ 置零（源式 (4)）：

$$\hat{\mathbf x}_{i+1}=\hat{\mathbf x}_i\boxplus\bigl(\Delta t\,\mathbf f(\hat{\mathbf x}_i,\mathbf u_i,\mathbf 0)\bigr);\qquad \hat{\mathbf x}_0=\bar{\mathbf x}_{k-1}.\tag{4}$$

其中 $\Delta t=\tau_{i+1}-\tau_i$。为传播协方差，用下面导出的**误差态动态模型**（源式 (5)）：

$$
\tilde{\mathbf x}_{i+1}=\mathbf x_{i+1}\boxminus\hat{\mathbf x}_{i+1}
=\bigl(\mathbf x_i\boxplus\Delta t\,\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\bigr)\boxminus\bigl(\hat{\mathbf x}_i\boxplus\Delta t\,\mathbf f(\hat{\mathbf x}_i,\mathbf u_i,\mathbf 0)\bigr)
$$
$$
\overset{[\text{Hertzberg}]}{\simeq}\mathbf F_{\tilde{\mathbf x}}\,\tilde{\mathbf x}_i+\mathbf F_{\mathbf w}\,\mathbf w_i.\tag{5}
$$

矩阵 $\mathbf F_{\tilde{\mathbf x}}$ 与 $\mathbf F_{\mathbf w}$ 按附录 A 计算（见 §6.1），结果列于源式 (7)。其中 $\hat{\boldsymbol\omega}_i=\boldsymbol\omega_{m_i}-\hat{\mathbf b}_{\omega_i}$、$\hat{\mathbf a}_i=\mathbf a_{m_i}-\hat{\mathbf b}_{a_i}$，函数 $\mathbf A(\mathbf u)^{-1}$ 与 [Bullo-Murray 1995, [25]] 一致定义（源式 (6)）：

$$\mathbf A(\mathbf u)^{-1}=\mathbf I-\tfrac12\lfloor\mathbf u\rfloor_\wedge+\bigl(1-\alpha(\lVert\mathbf u\rVert)\bigr)\frac{\lfloor\mathbf u\rfloor_\wedge^2}{\lVert\mathbf u\rVert^2},$$
$$\alpha(m)=\frac{m}{2}\cot\!\Bigl(\frac{m}{2}\Bigr)=\frac{m}{2}\cdot\frac{\cos(m/2)}{\sin(m/2)},\tag{6}$$

其中 ${}^I\mathbf T_L$ 是已知外参（见 §3.2）。

> **$\mathbf A(\mathbf u)$ 是什么**：它是 $\mathrm{SO}(3)$ 上 ⊞ 的右雅可比的逆（即 $\mathbf A(\mathbf u)^{-1}=\mathbf J_r(\mathbf u)^{-1}$ 的具体闭式）。在误差态传播中，旋转块的线性化会出现 $\mathrm{Exp}$ 复合的雅可比，故出现 $\mathbf A(\cdot)$。本书"右雅可比 $\mathbf J_r$"与此一致；$\mathbf A(\mathbf u)^{-1}$ 即标准 $\mathbf J_r(\mathbf u)^{-1}=\mathbf I-\frac12\lfloor\mathbf u\rfloor_\wedge+(\dots)\lfloor\mathbf u\rfloor_\wedge^2/\lVert\mathbf u\rVert^2$。

**误差态转移矩阵 $\mathbf F_{\tilde{\mathbf x}}$（$18\times18$，$6\times6$ 个 $3\times3$ 块）**（源式 (7)，按 ar5iv 渲染补全块位置）。行/列顺序对应误差态 $[\delta\boldsymbol\theta,{}^G\tilde{\mathbf p}_I,{}^G\tilde{\mathbf v}_I,\tilde{\mathbf b}_\omega,\tilde{\mathbf b}_a,{}^G\tilde{\mathbf g}]$：

$$\mathbf F_{\tilde{\mathbf x}}=\begin{bmatrix}
\mathrm{Exp}(-\hat{\boldsymbol\omega}_i\Delta t) & \mathbf 0 & \mathbf 0 & -\mathbf A(\hat{\boldsymbol\omega}_i\Delta t)^\top\Delta t & \mathbf 0 & \mathbf 0\\[3pt]
\mathbf 0 & \mathbf I & \mathbf I\,\Delta t & \mathbf 0 & \mathbf 0 & \mathbf 0\\[3pt]
-{}^G\hat{\mathbf R}_{I_i}\lfloor\hat{\mathbf a}_i\rfloor_\wedge\,\Delta t & \mathbf 0 & \mathbf I & \mathbf 0 & -{}^G\hat{\mathbf R}_{I_i}\,\Delta t & \mathbf I\,\Delta t\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I & \mathbf 0 & \mathbf 0\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I & \mathbf 0\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I
\end{bmatrix}.\tag{7a}$$

**噪声雅可比 $\mathbf F_{\mathbf w}$（$18\times12$，$6\times4$ 个 $3\times3$ 块）**。列顺序对应噪声 $[\mathbf n_\omega,\mathbf n_a,\mathbf n_{b\omega},\mathbf n_{ba}]$：

$$\mathbf F_{\mathbf w}=\begin{bmatrix}
-\mathbf A(\hat{\boldsymbol\omega}_i\Delta t)^\top\Delta t & \mathbf 0 & \mathbf 0 & \mathbf 0\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\\[3pt]
\mathbf 0 & -{}^G\hat{\mathbf R}_{I_i}\,\Delta t & \mathbf 0 & \mathbf 0\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf I\,\Delta t & \mathbf 0\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I\,\Delta t\\[3pt]
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0
\end{bmatrix}.\tag{7b}$$

> **抽取注（极重要）**：pdftotext 把 (7) 的块布局严重打乱（行列错位）。上式 (7a)(7b) 已用 ar5iv HTML 渲染逐块校核重排。关键非零块语义：
> - 旋转-旋转块 $\mathrm{Exp}(-\hat{\boldsymbol\omega}_i\Delta t)$（姿态误差的旋转传播）；旋转-陀螺零偏块 $-\mathbf A(\hat{\boldsymbol\omega}_i\Delta t)^\top\Delta t$（零偏经右雅可比影响姿态误差）。
> - 位置-速度块 $\mathbf I\Delta t$（位置误差受速度误差驱动）。
> - 速度-旋转块 $-{}^G\hat{\mathbf R}_{I_i}\lfloor\hat{\mathbf a}_i\rfloor_\wedge\Delta t$（姿态误差经比力转成速度误差）；速度-加计零偏块 $-{}^G\hat{\mathbf R}_{I_i}\Delta t$；速度-重力块 $\mathbf I\Delta t$。
> - 噪声块：陀螺白噪声经 $-\mathbf A^\top\Delta t$ 入姿态；加计白噪声经 $-{}^G\hat{\mathbf R}_{I_i}\Delta t$ 入速度；零偏随机游走噪声经 $\mathbf I\Delta t$ 入两零偏。
> - 零偏、重力的对角块为 $\mathbf I$（常量/随机游走）。

记白噪声 $\mathbf w$ 的协方差为 $\mathbf Q$，则传播协方差 $\hat{\mathbf P}_i$ 按下式迭代计算（源式 (8)）：

$$\hat{\mathbf P}_{i+1}=\mathbf F_{\tilde{\mathbf x}}\,\hat{\mathbf P}_i\,\mathbf F_{\tilde{\mathbf x}}^\top+\mathbf F_{\mathbf w}\,\mathbf Q\,\mathbf F_{\mathbf w}^\top;\qquad \hat{\mathbf P}_0=\bar{\mathbf P}_{k-1}.\tag{8}$$

传播持续到新扫描的结束时刻 $t_k$，此时传播的状态与协方差记 $\hat{\mathbf x}_k,\hat{\mathbf P}_k$。则 $\hat{\mathbf P}_k$ 表示真值状态 $\mathbf x_k$ 与传播状态 $\hat{\mathbf x}_k$ 之间误差（即 $\mathbf x_k\boxminus\hat{\mathbf x}_k$）的协方差。

### §4.2 反向传播与运动补偿（Backward Propagation & Motion Compensation）[源 FAST-LIO §III-C-2]

当点累积时间间隔在 $t_k$ 到达时，新扫描的特征点要与传播的状态 $\hat{\mathbf x}_k$、协方差 $\hat{\mathbf P}_k$ 融合产生最优更新。但虽然新扫描在 $t_k$，特征点是在各自采样时刻 $\rho_j\le t_k$ 测量的（§3.4、Fig. 2(b)），导致**参考体系不匹配**（每个点采样瞬间 LiDAR 位姿不同）。

为补偿 $\rho_j$ 与 $t_k$ 之间的相对运动（即运动畸变），把 (2) **向后传播**：$\check{\mathbf x}_{j-1}=\check{\mathbf x}_j\boxplus(-\Delta t\,\mathbf f(\check{\mathbf x}_j,\mathbf u_j,\mathbf 0))$，从**零位姿**及取自 $\hat{\mathbf x}_k$ 的其余状态（速度、零偏）开始。反向传播按**特征点频率**进行（通常远高于 IMU 率）。对两 IMU 测量之间采样的所有特征点，用**左侧 IMU 测量**作反向传播输入。注意 $\mathbf f(\mathbf x_j,\mathbf u_j,\mathbf 0)$ 的后三块（对应陀螺零偏、加计零偏、外参）为零（见 (3)），故反向传播可简化为（源式 (9)）：

$$
{}^{I_k}\check{\mathbf p}_{I_{j-1}}={}^{I_k}\check{\mathbf p}_{I_j}-{}^{I_k}\check{\mathbf v}_{I_j}\Delta t,\qquad\text{s.f. }{}^{I_k}\check{\mathbf p}_{I_m}=\mathbf 0;
$$
$$
{}^{I_k}\check{\mathbf v}_{I_{j-1}}={}^{I_k}\check{\mathbf v}_{I_j}-{}^{I_k}\check{\mathbf R}_{I_j}(\mathbf a_{m_{i-1}}-\hat{\mathbf b}_{a_k})\Delta t-{}^{I_k}\hat{\mathbf g}_k\Delta t,
$$
$$
\text{s.f. }{}^{I_k}\check{\mathbf v}_{I_m}={}^G\hat{\mathbf R}_{I_k}^\top\,{}^G\hat{\mathbf v}_{I_k},\quad {}^{I_k}\hat{\mathbf g}_k={}^G\hat{\mathbf R}_{I_k}^\top\,{}^G\hat{\mathbf g}_k;
$$
$$
{}^{I_k}\check{\mathbf R}_{I_{j-1}}={}^{I_k}\check{\mathbf R}_{I_j}\,\mathrm{Exp}\bigl((\hat{\mathbf b}_{\omega_k}-\boldsymbol\omega_{m_{i-1}})\Delta t\bigr),\qquad\text{s.f. }{}^{I_k}\mathbf R_{I_m}=\mathbf I.
\tag{9}
$$

其中 $\rho_{j-1}\in[\tau_{i-1},\tau_i)$，$\Delta t=\rho_j-\rho_{j-1}$，"s.f." 意为"starting from（始于）"。

> **解读**：反向传播是把扫描末尾 $t_k$ 时刻的状态，沿 IMU 运动学**反推**到每个特征点采样时刻 $\rho_j$，给出该点采样瞬间 IMU 相对扫描末尾的相对位姿 ${}^{I_k}\check{\mathbf T}_{I_j}=({}^{I_k}\check{\mathbf R}_{I_j},{}^{I_k}\check{\mathbf p}_{I_j})$。位置/旋转在 $I_k$ 系中从 0/$\mathbf I$ 反推；速度/重力初值由全局系量旋转到 $I_k$ 系。注意旋转用 $\mathrm{Exp}((\hat{\mathbf b}_{\omega}-\boldsymbol\omega_m)\Delta t)$（反向即角速度取负）。

该相对位姿使我们能把局部测量 ${}^{L_j}\mathbf p_{f_j}$ 投影到扫描末尾测量 ${}^{L_k}\mathbf p_{f_j}$（源式 (10)）：

$$
{}^{L_k}\mathbf p_{f_j}={}^I\mathbf T_L^{-1}\;{}^{I_k}\check{\mathbf T}_{I_j}\;{}^I\mathbf T_L\;{}^{L_j}\mathbf p_{f_j}.\tag{10}
$$

然后用投影点 ${}^{L_k}\mathbf p_{f_j}$ 构造残差（下节）。

> **链式解读**：${}^{L_j}\mathbf p_{f_j}$（LiDAR 系 $L_j$）$\xrightarrow{{}^I\mathbf T_L}$ IMU 系 $I_j$ $\xrightarrow{{}^{I_k}\check{\mathbf T}_{I_j}}$ IMU 系 $I_k$ $\xrightarrow{{}^I\mathbf T_L^{-1}}$ LiDAR 系 $L_k$。即"去外参→相对位姿补偿→回外参"。这就是 FAST-LIO 的**逐点精确运动补偿**（区别于松耦合的整扫描线性插值）。

### §4.3 残差计算（Residual Computation）[源 FAST-LIO §III-C-3]

有了 (10) 的运动补偿，可把整扫描特征点 $\{{}^{L_k}\mathbf p_{f_j}\}$ 视作**全在同一时刻 $t_k$ 采样**并构造残差。设迭代卡尔曼滤波当前迭代为 $\kappa$，对应状态估计 $\hat{\mathbf x}_k^\kappa$。当 $\kappa=0$ 时 $\hat{\mathbf x}_k^0=\hat{\mathbf x}_k$（来自 (4) 的传播预测）。则特征点 $\{{}^{L_k}\mathbf p_{f_j}\}$ 可变换到全局系（源式 (11)）：

$$
{}^G\hat{\mathbf p}_{f_j}^\kappa={}^G\hat{\mathbf T}_{I_k}^\kappa\,{}^I\mathbf T_L\,{}^{L_k}\mathbf p_{f_j};\qquad j=1,\cdots,m.\tag{11}
$$

对每个 LiDAR 特征点，假设其真正所属处是地图中由邻近特征点定义的**最近平面或边**。即残差定义为该特征点估计的全局系坐标 ${}^G\hat{\mathbf p}_{f_j}^\kappa$ 到地图中最近平面（或边）的**距离**。记 $\mathbf u_j$ 为对应平面（或边）的法向量（或边方向），其上有一点 ${}^G\mathbf q_j$，则残差 $\mathbf z_j^\kappa$ 为（源式 (12)）：

$$
\mathbf z_j^\kappa=\mathbf G_j\bigl({}^G\hat{\mathbf p}_{f_j}^\kappa-{}^G\mathbf q_j\bigr),\tag{12}
$$

其中**平面特征** $\mathbf G_j=\mathbf u_j^\top$，**边特征** $\mathbf G_j=\lfloor\mathbf u_j\rfloor_\wedge$。$\mathbf u_j$ 的计算与地图中邻近点搜索（定义对应平面或边）靠对**最近地图**中的点建 **KD-tree**（[10]）实现。仅考虑范数低于某阈值（如 0.5 m）的残差；超阈值者为外点或新观测点。

### §4.4 迭代状态更新（Iterated State Update）[源 FAST-LIO §III-C-4]

为把 (12) 的残差 $\mathbf z_j^\kappa$ 与 IMU 数据传播的状态预测 $\hat{\mathbf x}_k$、协方差 $\hat{\mathbf P}_k$ 融合，需**线性化测量模型**——它把残差 $\mathbf z_j^\kappa$ 关联到真值状态 $\mathbf x_k$ 与测量噪声。测量噪声源自测点 ${}^{L_j}\mathbf p_{f_j}$ 时 LiDAR 的**测距与束指向噪声** ${}^{L_j}\mathbf n_{f_j}$。去掉该噪声得真点位置（源式 (13)）：

$$
{}^{L_j}\mathbf p_{f_j}^{\,gt}={}^{L_j}\mathbf p_{f_j}-{}^{L_j}\mathbf n_{f_j}.\tag{13}
$$

此真点，经 (10) 投影到 $L_k$ 系、再经**真值状态 $\mathbf x_k$（即位姿）**投影到全局系后，应**恰好落在地图中的平面（或边）上**。即把 (13) 代入 (10)、再入 (11)、再入 (12) 应得零：

$$
\mathbf 0=\mathbf h_j(\mathbf x_k,{}^{L_j}\mathbf n_{f_j})=\mathbf G_j\,{}^G\mathbf T_{I_k}\,{}^{I_k}\check{\mathbf T}_{I_j}\,{}^I\mathbf T_L\bigl({}^{L_j}\mathbf p_{f_j}-{}^{L_j}\mathbf n_{f_j}\bigr)-{}^G\mathbf q_j.
$$

> **抽取注**：源 (13)→(14) 间的隐式测量方程 pdftotext 略有错位（出现 ${}^G\mathbf T_{I_k}{}^{I_k}\check{\mathbf T}_{I_j}$ 复合），其本质是"真点经真值位姿落在地图面上"。上式按语义补全。

在 $\hat{\mathbf x}_k^\kappa$ 处做一阶近似（源式 (14)）：

$$
\mathbf 0=\mathbf h_j(\mathbf x_k,{}^{L_j}\mathbf n_{f_j})\simeq\mathbf h_j(\hat{\mathbf x}_k^\kappa,\mathbf 0)+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa+\mathbf v_j=\mathbf z_j^\kappa+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa+\mathbf v_j,\tag{14}
$$

其中 $\tilde{\mathbf x}_k^\kappa=\mathbf x_k\boxminus\hat{\mathbf x}_k^\kappa$（等价地 $\mathbf x_k=\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa$）；$\mathbf H_j^\kappa$ 是 $\mathbf h_j(\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa,{}^{L_j}\mathbf n_{f_j})$ 关于 $\tilde{\mathbf x}_k^\kappa$ 的雅可比、在零处取值；$\mathbf v_j\in\mathcal N(\mathbf 0,\mathbf R_j)$ 来自原始测量噪声 ${}^{L_j}\mathbf n_{f_j}$。

**先验分布**：来自 §4.1 前向传播的 $\mathbf x_k$ 先验是关于 $\mathbf x_k\boxminus\hat{\mathbf x}_k$ 的。但在第 $\kappa$ 次迭代我们围绕 $\hat{\mathbf x}_k^\kappa$ 展开，故需把先验从 $\hat{\mathbf x}_k$ 改写到 $\hat{\mathbf x}_k^\kappa$ 的切空间（源式 (15)）：

$$
\mathbf x_k\boxminus\hat{\mathbf x}_k=(\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa)\boxminus\hat{\mathbf x}_k=\hat{\mathbf x}_k^\kappa\boxminus\hat{\mathbf x}_k+\mathbf J^\kappa\,\tilde{\mathbf x}_k^\kappa,\tag{15}
$$

其中 $\mathbf J^\kappa$ 是 $(\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa)\boxminus\hat{\mathbf x}_k$ 关于 $\tilde{\mathbf x}_k^\kappa$ 在零处的偏导（源式 (16)）：

$$
\mathbf J^\kappa=\begin{bmatrix}
\mathbf A\bigl({}^G\hat{\mathbf R}_{I_k}^\kappa\boxminus{}^G\hat{\mathbf R}_{I_k}\bigr)^{-\top} & \mathbf 0_{3\times15}\\[3pt]
\mathbf 0_{15\times3} & \mathbf I_{15\times15}
\end{bmatrix},\tag{16}
$$

其中 $\mathbf A(\cdot)^{-1}$ 定义在 (6)。**对第一次迭代（即退化为普通 EKF 的情形）**，$\hat{\mathbf x}_k^\kappa=\hat{\mathbf x}_k$，则 $\mathbf J^\kappa=\mathbf I$。

> **$\mathbf J^\kappa$ 的意义**：迭代 ESKF 每次迭代都更新工作点 $\hat{\mathbf x}_k^\kappa$，而先验协方差 $\hat{\mathbf P}_k$ 是在原始预测点 $\hat{\mathbf x}_k$ 的切空间定义的。$\mathbf J^\kappa$ 把先验从 $\hat{\mathbf x}_k$ 的切空间"搬运"到 $\hat{\mathbf x}_k^\kappa$ 的切空间（只有旋转块非平凡，因 $\mathbb R^n$ 部分 ⊞ 是平凡加法）。这是 on-manifold 迭代 ESKF 区别于普通 IEKF 的关键修正项。

**MAP 估计**：把 (15) 的先验与 (14) 的后验分布结合，得最大后验估计（源式 (17)）：

$$
\min_{\tilde{\mathbf x}_k^\kappa}\;\Bigl(\lVert\mathbf x_k\boxminus\hat{\mathbf x}_k\rVert_{\hat{\mathbf P}_k^{-1}}^2+\sum_{j=1}^m\lVert\mathbf z_j^\kappa+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa\rVert_{\mathbf R_j^{-1}}^2\Bigr),\tag{17}
$$

其中 $\lVert\mathbf x\rVert_{\mathbf M}^2=\mathbf x^\top\mathbf M\mathbf x$。把 (15) 的先验线性化代入 (17)、对得到的二次代价优化，导出**标准迭代卡尔曼滤波**（[21]）。为简化记号，令

$$\mathbf H=[\mathbf H_1^{\kappa\top},\cdots,\mathbf H_m^{\kappa\top}]^\top,\quad \mathbf R=\mathrm{diag}(\mathbf R_1,\cdots,\mathbf R_m),\quad \mathbf P=(\mathbf J^\kappa)^{-1}\hat{\mathbf P}_k(\mathbf J^\kappa)^{-\top},\quad \mathbf z_k^\kappa=[\mathbf z_1^{\kappa\top},\cdots,\mathbf z_m^{\kappa\top}]^\top,$$

则更新为（源式 (18)）：

$$
\mathbf K=\mathbf P\mathbf H^\top(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)^{-1},
$$
$$
\hat{\mathbf x}_k^{\kappa+1}=\hat{\mathbf x}_k^\kappa\boxplus\bigl(-\mathbf K\mathbf z_k^\kappa-(\mathbf I-\mathbf K\mathbf H)(\mathbf J^\kappa)^{-1}(\hat{\mathbf x}_k^\kappa\boxminus\hat{\mathbf x}_k)\bigr).\tag{18}
$$

更新的估计 $\hat{\mathbf x}_k^{\kappa+1}$ 用于按 §4.3 重算残差并重复，直至收敛（即 $\lVert\hat{\mathbf x}_k^{\kappa+1}\boxminus\hat{\mathbf x}_k^\kappa\rVert<\epsilon$）。收敛后最优状态估计与协方差为（源式 (19)）：

$$
\bar{\mathbf x}_k=\hat{\mathbf x}_k^{\kappa+1},\qquad \bar{\mathbf P}_k=(\mathbf I-\mathbf K\mathbf H)\,\mathbf P.\tag{19}
$$

**计算复杂度问题与新增益公式**：(18) 中常用卡尔曼增益形式需求逆 $\mathbf H\mathbf P\mathbf H^\top+\mathbf R$，其维数是**测量维数**。实际中 LiDAR 特征点数极大，求逆这么大的矩阵不可行（故 [21,26] 只用少量测量）。本文证明可避免此限制：直觉来自 (17) 的代价函数是**关于状态的**，故解应以**状态维数**复杂度算出。直接解 (17) 可得与 (18) 相同的解，但用新形式增益（源式 (20)）：

$$
\boxed{\;\mathbf K=(\mathbf H^\top\mathbf R^{-1}\mathbf H+\mathbf P^{-1})^{-1}\mathbf H^\top\mathbf R^{-1}.\;}\tag{20}
$$

附录 B（见 §6.2）基于**矩阵求逆引理**证明两形式等价。由于 LiDAR 测量独立，协方差 $\mathbf R$ 是（块）对角，新公式只需求逆**两个状态维数**的矩阵（而非测量维数）。新公式大幅节省计算，因状态维数（如 18）通常远低于测量数（如 10 Hz 扫描超过 1000 个有效特征点）。

**两增益公式运行时间对比（源 Table II）**：

| 特征点数 | 307 | 717 | 998 | 1243 | 1453 | 1802 |
|---|---|---|---|---|---|---|
| 旧公式 (18) (ms) | 7.1 | 23.4 | 109.3 | 251 | 1219 | 1621 |
| 新公式 (20) (ms) | 0.07 | 0.11 | 0.25 | 0.37 | 0.59 | 1.16 |

### §4.5 算法 1：状态估计 [源 FAST-LIO §III-C-5]

```
算法 1：State Estimation（状态估计）
输入：上一最优估计 x̄_{k-1} 与 P̄_{k-1}；当前扫描的 IMU 输入 (a_m, ω_m)；
      当前扫描的 LiDAR 特征点 {^{L_j} p_{f_j}}。
1   前向传播经 (4) 得状态预测 x̂_k、经 (8) 得协方差预测 P̂_k；
2   反向传播经 (9)(10) 得 ^{L_k} p_{f_j}（运动补偿）；
3   κ = -1, x̂_k^{κ=0} = x̂_k；
4   repeat
5       κ = κ + 1；
6       经 (16) 计算 J^κ，并令 P = (J^κ)^{-1} P̂_k (J^κ)^{-⊤}；
7       计算残差 z_j^κ（12）与雅可比 H_j^κ（14）；
8       经 (18) 用 (20) 的卡尔曼增益 K 计算状态更新 x̂_k^{κ+1}；
9   until ‖ x̂_k^{κ+1} ⊟ x̂_k^κ ‖ < ε；
10  x̄_k = x̂_k^{κ+1}；P̄_k = (I − KH) P。
输出：当前最优估计 x̄_k 与 P̄_k。
```

## §5 建图、初始化 [源 FAST-LIO §III-D, §III-E]

### §5.1 地图更新 [源 §III-D]

有了状态更新 $\bar{\mathbf x}_k$（即 ${}^G\bar{\mathbf T}_{I_k}=({}^G\bar{\mathbf R}_{I_k},{}^G\bar{\mathbf p}_{I_k})$），每个投影到体系 $L_k$ 的特征点 ${}^{L_k}\mathbf p_{f_j}$（见 (10)）再变换到全局系（源式 (21)）：

$$
{}^G\bar{\mathbf p}_{f_j}={}^G\bar{\mathbf T}_{I_k}\,{}^I\mathbf T_L\,{}^{L_k}\mathbf p_{f_j};\qquad j=1,\cdots,m.\tag{21}
$$

这些特征点最终**追加到含此前所有步特征点的现有地图**中。（FAST-LIO 用 KD-tree 维护最近地图；这正是 FAST-LIO2 用 ikd-Tree 替换的部分。）

### §5.2 初始化 [源 §III-E]

为获得系统状态（重力 ${}^G\mathbf g$、零偏、噪声协方差）的良好初值以加速状态估计器，需初始化。FAST-LIO 的初始化很简单：**保持 LiDAR 静止数秒**（本文所有实验为 2 秒），用采集数据初始化 IMU 零偏与重力向量。若 LiDAR 支持非重复扫描（如 Livox AVIA），保持静止还能让 LiDAR 捕获初始高分辨率地图，利于后续导航。

## §6 附录：两个完整证明 [源 FAST-LIO Appendix]

### §6.1 附录 A：$\mathbf F_{\tilde{\mathbf x}}$ 与 $\mathbf F_{\mathbf w}$ 的计算 [源 Appendix A]

回忆 $\mathbf x_i=\hat{\mathbf x}_i\boxplus\tilde{\mathbf x}_i$，记 $\mathbf g(\tilde{\mathbf x}_i,\mathbf w_i)=\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\Delta t=\mathbf f(\hat{\mathbf x}_i\boxplus\tilde{\mathbf x}_i,\mathbf u_i,\mathbf w_i)\Delta t$。则误差态模型 (5) 重写为（源式 (22)）：

$$
\tilde{\mathbf x}_{i+1}=\underbrace{\bigl((\hat{\mathbf x}_i\boxplus\tilde{\mathbf x}_i)\boxplus\mathbf g(\tilde{\mathbf x}_i,\mathbf w_i)\bigr)\boxminus\bigl(\hat{\mathbf x}_i\boxplus\mathbf g(\mathbf 0,\mathbf 0)\bigr)}_{\mathbf G(\tilde{\mathbf x}_i,\mathbf g(\tilde{\mathbf x}_i,\mathbf w_i))}.\tag{22}
$$

按偏微分链式法则，(5) 中矩阵 $\mathbf F_{\tilde{\mathbf x}}$ 与 $\mathbf F_{\mathbf w}$ 计算如下（源式 (23)）：

$$
\mathbf F_{\tilde{\mathbf x}}=\left.\left(\frac{\partial\mathbf G(\tilde{\mathbf x}_i,\mathbf g(\mathbf 0,\mathbf 0))}{\partial\tilde{\mathbf x}_i}+\frac{\partial\mathbf G(\mathbf 0,\mathbf g(\tilde{\mathbf x}_i,\mathbf 0))}{\partial\mathbf g(\tilde{\mathbf x}_i,\mathbf 0)}\cdot\frac{\partial\mathbf g(\tilde{\mathbf x}_i,\mathbf 0)}{\partial\tilde{\mathbf x}_i}\right)\right|_{\tilde{\mathbf x}_i=\mathbf 0},
$$
$$
\mathbf F_{\mathbf w}=\left.\left(\frac{\partial\mathbf G(\mathbf 0,\mathbf g(\mathbf 0,\mathbf w_i))}{\partial\mathbf g(\mathbf 0,\mathbf w_i)}\cdot\frac{\partial\mathbf g(\mathbf 0,\mathbf w_i)}{\partial\mathbf w_i}\right)\right|_{\mathbf w_i=\mathbf 0}.\tag{23}
$$

> **抽取注**：源把 $\mathbf F_{\tilde{\mathbf x}}$ 分解为两项之和——第一项是 $\mathbf G$ 对其第一参数 $\tilde{\mathbf x}_i$ 的偏导（保持 $\mathbf g$ 在 $\mathbf 0$），第二项是 $\mathbf G$ 对第二参数 $\mathbf g$ 的偏导乘以 $\mathbf g$ 对 $\tilde{\mathbf x}_i$ 的偏导。逐块代入 ⊞/⊟ 的雅可比（旋转块用 $\mathbf A(\cdot)$/$\mathrm{Exp}$ 雅可比）即得 (7a)(7b) 的具体闭式。源未在正文逐块展开中间代数，只给出链式法则框架 (23) 与最终结果 (7)；IKFOM [55] 给出更抽象的逐块推导。

### §6.2 附录 B：等价卡尔曼增益公式（完整证明）[源 FAST-LIO Appendix B]

基于**矩阵求逆引理（matrix inverse lemma）**[27]（Higham），可得（源给出的中间恒等式）：

$$
\bigl(\mathbf P^{-1}+\mathbf H^\top\mathbf R^{-1}\mathbf H\bigr)^{-1}=\mathbf P-\mathbf P\mathbf H^\top\bigl(\mathbf H\mathbf P\mathbf H^\top+\mathbf R\bigr)^{-1}\mathbf H\mathbf P.\tag{B.1}
$$

把 (B.1) 代入新公式 (20)，可得：

$$
\mathbf K=\bigl(\mathbf H^\top\mathbf R^{-1}\mathbf H+\mathbf P^{-1}\bigr)^{-1}\mathbf H^\top\mathbf R^{-1}=\mathbf P\mathbf H^\top\mathbf R^{-1}-\mathbf P\mathbf H^\top\bigl(\mathbf H\mathbf P\mathbf H^\top+\mathbf R\bigr)^{-1}\mathbf H\mathbf P\mathbf H^\top\mathbf R^{-1}.\tag{B.2}
$$

现注意到恒等式

$$
\mathbf H\mathbf P\mathbf H^\top\mathbf R^{-1}=\bigl(\mathbf H\mathbf P\mathbf H^\top+\mathbf R\bigr)\mathbf R^{-1}-\mathbf I.\tag{B.3}
$$

把 (B.3) 代入 (B.2)，得标准卡尔曼增益形式 (18)，证明如下：

$$
\mathbf K=\mathbf P\mathbf H^\top\mathbf R^{-1}-\mathbf P\mathbf H^\top\Bigl[\mathbf R^{-1}-\bigl(\mathbf H\mathbf P\mathbf H^\top+\mathbf R\bigr)^{-1}\Bigr]
$$
$$
=\mathbf P\mathbf H^\top\mathbf R^{-1}-\mathbf P\mathbf H^\top\mathbf R^{-1}+\mathbf P\mathbf H^\top\bigl(\mathbf H\mathbf P\mathbf H^\top+\mathbf R\bigr)^{-1}=\mathbf P\mathbf H^\top\bigl(\mathbf H\mathbf P\mathbf H^\top+\mathbf R\bigr)^{-1}.\tag{B.4}
$$

> **抽取注**：源把 (B.2)→(B.4) 写得很紧凑：先得 $\mathbf K=\mathbf P\mathbf H^\top\mathbf R^{-1}-\mathbf P\mathbf H^\top(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)^{-1}\mathbf H\mathbf P\mathbf H^\top\mathbf R^{-1}$，再用 (B.3) 代入 $\mathbf H\mathbf P\mathbf H^\top\mathbf R^{-1}=(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)\mathbf R^{-1}-\mathbf I$，使中括号 $(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)^{-1}[(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)\mathbf R^{-1}-\mathbf I]=\mathbf R^{-1}-(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)^{-1}$，两 $\mathbf P\mathbf H^\top\mathbf R^{-1}$ 抵消，剩 $\mathbf P\mathbf H^\top(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)^{-1}$。即 (18)。证毕。$\blacksquare$
>
> **矩阵求逆引理一般形式**（Woodbury，本书统一记法）：$(\mathbf A+\mathbf B\mathbf C\mathbf D)^{-1}=\mathbf A^{-1}-\mathbf A^{-1}\mathbf B(\mathbf C^{-1}+\mathbf D\mathbf A^{-1}\mathbf B)^{-1}\mathbf D\mathbf A^{-1}$。(B.1) 是其特例：取 $\mathbf A=\mathbf P^{-1},\mathbf B=\mathbf H^\top,\mathbf C=\mathbf R^{-1},\mathbf D=\mathbf H$。

## §7 FAST-LIO 实验要点（数值结果）[源 FAST-LIO §IV]

> 实验非理论核心，但保留关键数值以备综合时引用。

- **A. 计算复杂度实验**：用旧/新增益公式在同一管线、同特征点数下对比，结果见上 Table II。新公式复杂度远低于旧公式。
- **B. UAV 飞行实验**：小型四旋翼（280 mm 轴距）载 Livox Avia LiDAR（70° FoV）+ DJI Manifold 2-C 板载机（1.8 GHz 四核 Intel i7-8550U，8 GB RAM）。室内最高 50 Hz 实时里程计与建图。50 Hz 室内实验：平均有效特征点 270、运行时间 6.7 ms，漂移 < 0.3%（32 m 轨迹漂移 0.08 m）。
- **C. 室内实验**（大转速）：传感器手持快速摇动，角速度常超 100 deg/s。与 LOAM on Livox [10] 及 LOAM+IMU [8]（特征提取替换为 FAST-LIO 的）对比。FAST-LIO 输出更快更稳（Table III）。LOAM+IMU 是松耦合，建图不一致。
  - **Table III（10 Hz 单扫描处理时间对比）**：LOAM（1107 有效特征，59 ms）；LOAM+IMU（1107，44 ms）；FAST-LIO（1430，23 ms）。
- **D. 室外实验**：港大主楼建图，手持行走约 140 m 返回起点，漂移 < 0.05%（140 m 漂移 0.07 m）。10 Hz 扫描，平均处理 25 ms、平均 1497 有效特征点。与 LINS [21] 对比（用 LINS 的 Velodyne VLP-16 + Xsens MTiG-710 海港数据）：FAST-LIO 平均 7.3 ms（建图更准），LINS 平均 34.5 ms（因 EKF 增益复杂度高，降采样到平均 147 点/扫描，FAST-LIO 用 784 点/扫描），均 10 Hz。

---

# 第二部分：FAST-LIO2（arXiv:2107.06829）全量抽取（增量部分 + ikd-Tree）

> FAST-LIO2 在 FAST-LIO 基础上的**两大新意**：(1) **直接配准原始点到地图（direct registration，免特征提取）**；(2) **用增量 k-d 树 ikd-Tree 维护地图**。状态估计的迭代 ESKF 框架**继承 FAST-LIO**，但状态扩展为 24 维（含外参在线标定）。本部分只抽取 FAST-LIO2 相对 FAST-LIO 的**增量内容**（重复部分指回第一部分），并完整抽取 ikd-Tree（第一部分无）。

## §8 引言与贡献 [源 FAST-LIO2 §I]

**问题背景**：实时建稠密 3D 图并同时定位（SLAM）对自主机器人安全导航至关重要。视觉 SLAM 定位准但仅维护稀疏特征图、受光照变化与运动模糊影响；视觉稠密建图仍是难题。3D LiDAR 提供直接、稠密、主动、精确的深度测量。LiDAR 里程计与建图的挑战：(1) LiDAR 每秒产数十万到数百万点，实时处理需高效率；(2) 为减负载常按局部光滑度提**特征点**（边/面），但特征提取性能易受环境影响（无大平面/长边的结构缺失环境、小 FoV 固态 LiDAR 尤甚），且**因 LiDAR 而异**（扫描模式、点密度），需大量手工调参；(3) LiDAR 点顺序采样而传感器连续运动，产显著**运动畸变**，IMU 可缓解但引入额外状态（零偏、外参）；(4) LiDAR 测距远但扫描线间分辨率低，点稀疏分布于大 3D 空间，需大而稠密的地图配准这些稀疏点，地图须支持高效对应搜索同时实时更新——维护这种地图很有挑战（与高分辨率视觉测量很不同）。

**贡献（源逐条，4 条）**：
1. 开发增量 k-d 树数据结构 **ikd-Tree** 高效表示大稠密点云图。除高效最近邻搜索外，支持**增量地图更新**（点插入、树上降采样 on-tree downsampling、点删除）与**动态再平衡**，且计算代价极小。使**100 Hz 里程计与建图**可在算力受限平台（Intel i7 微 UAV 板载机、甚至 ARM 处理器）上运行。ikd-Tree 工具箱开源。
2. 借 ikd-Tree 增益的计算效率，**直接把原始点配准到地图**（类比视觉 SLAM 的 direct method），即使激进运动、极杂乱环境也能更准更可靠。免手工特征提取使系统**天然适配不同 LiDAR**。
3. 把上述两技术集成进此前开发的紧耦合 LIO 系统 **FAST-LIO [22]**。系统用 IMU 经严格反向传播补偿每点运动，经 **on-manifold 迭代卡尔曼滤波**估全状态。用**新等价卡尔曼增益公式**把复杂度降到状态维数。新系统称 FAST-LIO2，开源。
4. 各种实验评估 ikd-Tree、直接点配准、整体系统。18 序列证 ikd-Tree 优于现有动态数据结构（octree、R*-tree、nanoflann k-d tree）；19 序列穷举基准证 FAST-LIO2 在更低算力下持续取得更高精度；并在小 FoV 固态 LiDAR 的挑战性真实数据（旋转速度达 1000 deg/s、结构缺失环境）上验证。

## §9 相关工作 [源 FAST-LIO2 §II]

### §9.1 LiDAR(-惯性)里程计 [源 §II-A]

现有 3D LiDAR SLAM 多继承 **LOAM** [23] 结构，三大模块：**特征提取、里程计、建图**。新扫描先按局部光滑度提特征点（边/面）；**里程计（scan-to-scan）**匹配两连续扫描的特征点得粗糙但实时（如 10 Hz）的 LiDAR 位姿里程计；多扫描合成一个 sweep 再配准合并到全局图（**建图 mapping**）。建图中地图点建 **k-d 树**供高效 kNN 搜索，点云配准用 **ICP**（每次迭代取地图中若干最近点构成目标点所属的平面或边）。为降建树时间，地图点按设定分辨率降采样。优化的建图过程通常低频（1-2 Hz）。

后续工作框架类似 LOAM：**LeGO-LOAM** 加地面点分割降算力 + **回环模块**降长期漂移；**LOAM-Livox** 把 LOAM 适配固态 LiDAR，因小 FoV 与非重复扫描（两连续扫描特征点对应极少），其里程计靠**直接把新扫描配准到全局图**（scan-to-map 直接配准提升精度，代价是每步要为更新的地图点重建 k-d 树）。

加 IMU 可大增精度鲁棒（给 ICP 好初值、高频 IMU 补偿运动畸变）。**LION** 是松耦合 LIO（保留 LOAM 的 scan-to-scan，加可观测性感知检查降点数）。更紧耦合的 LiDAR-惯性融合 [17,29-31] 在**固定数量近期扫描（或关键帧）的小局部图**中做里程计；相比 scan-to-scan，**scan-to-local-map** 用更多近期信息通常更准：
- **LIOM** [29]：紧耦合，把 IMU 预积分引入里程计。
- **LILI-OM** [17]：为非重复扫描 LiDAR 开发新特征提取，在 20 个近期扫描的小图中配准。
- **LIO-SAM** [30]：里程计需 9 轴 IMU 产姿态测量作小局部图内配准先验；用**因子图平滑** [32]。
- **LINS** [31]：把**紧耦合迭代卡尔曼滤波**与**机器人中心（robocentric）公式**引入里程计的 LiDAR 位姿优化。

上述局部图通常小以保实时，里程计漂移快，需低频建图过程修正（LINS 的 map refining、LILI-OM/LIOM 的滑窗联合优化、LIO-SAM 的因子图平滑）。**FAST-LIO** [22] 相比引入**形式化反向传播**精确考虑每点采样时刻并经严格 IMU 运动学补偿畸变；新卡尔曼增益公式把复杂度从测量维降到状态维（证明数学等价但减计算几个数量级），允许里程计中**直接实时 scan-to-map 配准**并每步更新地图。但为防建图 k-d 树建树时间增长，系统只能在小环境（数百米）工作。

**FAST-LIO2 定位**：建于 FAST-LIO，继承紧耦合融合框架（尤其反向传播解畸变、快速卡尔曼增益）。为系统性解决计算增长问题，提出 **ikd-Tree**（每步增量更新地图 + 高效 kNN）。借大幅降低的负载，里程计**直接配准原始 LiDAR 点到地图**（提升精度鲁棒，尤其新扫描无显著特征时）。相比上述都用特征点的紧耦合 LiDAR-惯性方法，本法更轻量、建图率与里程计精度更高、免特征提取调参。直接配准原始点的思想在 LION（松耦合）探索过，也类似 generalized-ICP（G-ICP，点配准到地图小局部平面，假设环境局部光滑可视作平面，但 G-ICP 计算负载通常大）；NDT 类方法也配准原始点但稳定性低于 ICP、某些场景发散。

### §9.2 建图中的动态数据结构 [源 §II-B]

为实时建图需动态数据结构同时支持增量更新与高效 kNN。kNN 搜索可建空间索引解决，分两类：**划分数据**与**划分空间**。
- 划分数据：**R-tree** [37]（按空间邻近聚成可能重叠的轴对齐立方体，支持 kNN 与点级更新：插入、删除、重插入，及区域/条件搜索）；**R\*-tree** [38]（按最小重叠插入 + 强制重插入分裂，优于原版）。
- 划分空间：**octree** [39]（递归把空间均分八个轴对齐立方，遇空或满足停止规则（最小分辨率/最小点数）停；支持 kNN 与盒搜索）；**k-d tree** [40]（二叉树，节点表轴对齐超平面把空间二分；标准构造取最长维上的中位点作分裂节点以紧凑划分）。低维 + 主存场景下 k-d 树在 kNN 问题中性能最佳 [42,43]。但向 k-d 树**插入/删除点会破坏平衡**，需重建再平衡（ANN、libnabo、FLANN 全量重建，计算量大）。基于硬件的 k-d 树重建依赖算力（机器人板载机受限）。

部分再平衡方法：**scapegoat k-d tree** [50]（仅对不平衡子树部分重建以维护整树松平衡）；对数方法维护一组 k-d 树并选子集重建（**Bkd-tree** [53]：主存维护最大 $M$ 的 $T_0$，外存维护 $T_i$ 大小 $2^{(i-1)}M$，$T_0$ 满时把点抽到第一个空树）；**nanoflann** k-d 树用对数结构增量更新、lazy label 仅标删不真删 [54]。

**FAST-LIO2 提出 ikd-Tree**（基于 scapegoat k-d tree [50]）实现实时建图：支持**点级插入 + 树上降采样**（其它结构需插入前外部降采样）、按轴对齐立方体**直接盒删**（靠维护范围信息 + lazy label，R-tree/octree 需逐点删）、**并行重建**避免大量点重建时的卡顿（主线程实时性与精度有保证）。

## §10 系统总览 [源 FAST-LIO2 §III]

FAST-LIO2 管线（Fig. 1）：顺序采样的 LiDAR 原始点先在 10 ms（100 Hz 更新）到 100 ms（10 Hz 更新）间累积成一个**扫描**。新扫描的点经**紧耦合迭代卡尔曼滤波**配准到大局部图中的地图点（即里程计，§IV）。大局部图的全局地图点由增量 k-d 树 **ikd-Tree** 组织（§V）。若当前 LiDAR 的 FoV 范围越过地图边界，离 LiDAR 位姿最远的历史点从 ikd-Tree 删除。结果 ikd-Tree 跟踪一定边长（"地图尺寸 map size"）的大立方区内全部地图点，用于状态估计中算残差。优化的位姿最终把新扫描的点配准到全局系并按里程计率插入 ikd-Tree（即建图）。

## §11 状态估计：继承 + 24 维状态 [源 FAST-LIO2 §IV]

FAST-LIO2 的状态估计是继承自 FAST-LIO [22] 的紧耦合迭代卡尔曼滤波，但进一步**纳入 LiDAR-IMU 外参在线标定**。

### §11.1 运动学模型（状态转移）[源 §IV-A-1]

取第一个 IMU 帧（记 $I$）为全局系（记 $G$），记 ${}^I\mathbf T_L=({}^I\mathbf R_L,{}^I\mathbf p_L)$ 为**未知**的 LiDAR-IMU 外参，运动学模型（源式 (1)，相比 FAST-LIO 式(1) 多了外参导数）：

$$
{}^G\dot{\mathbf R}_I={}^G\mathbf R_I\,\lfloor\boldsymbol\omega_m-\mathbf b_\omega-\mathbf n_\omega\rfloor_\wedge,\qquad {}^G\dot{\mathbf p}_I={}^G\mathbf v_I,
$$
$$
{}^G\dot{\mathbf v}_I={}^G\mathbf R_I(\mathbf a_m-\mathbf b_a-\mathbf n_a)+{}^G\mathbf g,
$$
$$
\dot{\mathbf b}_\omega=\mathbf n_{b\omega},\qquad \dot{\mathbf b}_a=\mathbf n_{ba},
$$
$$
{}^G\dot{\mathbf g}=\mathbf 0,\qquad {}^I\dot{\mathbf R}_L=\mathbf 0,\qquad {}^I\dot{\mathbf p}_L=\mathbf 0.
\tag{F2-1}
$$

符号同 FAST-LIO（见 §3.2），新增**外参 ${}^I\mathbf R_L,{}^I\mathbf p_L$ 按常量传播**（在线标定时由滤波更新）。

记 $i$ 为 IMU 测量索引。基于 [22] 定义的 ⊞ 运算，连续模型 (F2-1) 在 IMU 采样周期 $\Delta t$ 上离散 [55]（源式 (2)，形式同 FAST-LIO 式(2)）：

$$\mathbf x_{i+1}=\mathbf x_i\boxplus\bigl(\Delta t\,\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\bigr),\tag{F2-2}$$

函数 $\mathbf f$、状态 $\mathbf x$、输入 $\mathbf u$、噪声 $\mathbf w$（源给出，24 维）：

$$\mathcal M\triangleq\mathrm{SO}(3)\times\mathbb R^{15}\times\mathrm{SO}(3)\times\mathbb R^3;\qquad \dim(\mathcal M)=24,$$

$$\mathbf x\triangleq\begin{bmatrix}{}^G\mathbf R_I^\top & {}^G\mathbf p_I^\top & {}^G\mathbf v_I^\top & \mathbf b_\omega^\top & \mathbf b_a^\top & {}^G\mathbf g^\top & {}^I\mathbf R_L^\top & {}^I\mathbf p_L^\top\end{bmatrix}^\top\in\mathcal M,$$

$$\mathbf u\triangleq\begin{bmatrix}\boldsymbol\omega_m^\top & \mathbf a_m^\top\end{bmatrix}^\top,\qquad \mathbf w\triangleq\begin{bmatrix}\mathbf n_\omega^\top & \mathbf n_a^\top & \mathbf n_{b\omega}^\top & \mathbf n_{ba}^\top\end{bmatrix}^\top,$$

$$\mathbf f(\mathbf x,\mathbf u,\mathbf w)=\begin{bmatrix}
\boldsymbol\omega_m-\mathbf b_\omega-\mathbf n_\omega\\[2pt]
{}^G\mathbf v_I+\tfrac12\bigl({}^G\mathbf R_I(\mathbf a_m-\mathbf b_a-\mathbf n_a)+{}^G\mathbf g\bigr)\Delta t\\[2pt]
{}^G\mathbf R_I(\mathbf a_m-\mathbf b_a-\mathbf n_a)+{}^G\mathbf g\\[2pt]
\mathbf n_{b\omega}\\[2pt]
\mathbf n_{ba}\\[2pt]
\mathbf 0_{3\times1}\\[2pt]
\mathbf 0_{3\times1}\\[2pt]
\mathbf 0_{3\times1}
\end{bmatrix}\in\mathbb R^{24}.\tag{F2-3}$$

> **抽取注（与 FAST-LIO 式(3) 的差异）**：
> 1. **位置块多了 $\tfrac12(\dots)\Delta t$ 二阶项**：FAST-LIO 式(3) 的位置块仅 ${}^G\mathbf v_I$，FAST-LIO2 式(3) 的位置块为 ${}^G\mathbf v_I+\tfrac12({}^G\mathbf R_I(\mathbf a_m-\mathbf b_a-\mathbf n_a)+{}^G\mathbf g)\Delta t$（更精确的二阶位置积分）。
> 2. **末尾多两个 $\mathbf 0_{3\times1}$ 块**对应外参 ${}^I\mathbf R_L,{}^I\mathbf p_L$ 的常量传播。
> 3. $\mathbf f\in\mathbb R^{24}$（FAST-LIO 是 $\in\mathbb R^{18}$ 隐含；FAST-LIO2 显式标 $\in\mathbb R^{24}$）。

### §11.2 测量模型（点到面，直接配准）[源 §IV-A-2]

LiDAR 逐点采样，连续运动下点在**不同位姿**采样。为校此扫描内运动，用 [22] 的**反向传播**估扫描内每点相对扫描末尾位姿的 LiDAR 位姿，把所有点按各自精确采样时刻投影到扫描末尾，从而点可视作**全在扫描末尾同时采样**（细节同 §4.2）。

记 $k$ 为 LiDAR 扫描索引，$\{{}^L\mathbf p_j,\;j=1,\cdots,m\}$ 为第 $k$ 扫描的点（在扫描末尾 LiDAR 局部系 $L$）。因 LiDAR 测量噪声，每个测点 ${}^L\mathbf p_j$ 被由测距与束指向噪声构成的噪声 ${}^L\mathbf n_j$ 污染。去噪得 LiDAR 局部系真点（源式 (3)）：

$$
{}^L\mathbf p_j^{\,gt}={}^L\mathbf p_j+{}^L\mathbf n_j.\tag{F2-3'}
$$

> **抽取注**：注意符号——FAST-LIO 式(13) 写 ${}^{L_j}\mathbf p_{f_j}^{gt}={}^{L_j}\mathbf p_{f_j}-{}^{L_j}\mathbf n_{f_j}$（减），FAST-LIO2 式(3) 写 ${}^L\mathbf p_j^{gt}={}^L\mathbf p_j+{}^L\mathbf n_j$（加）。二者只是噪声符号约定差异（噪声为零均值高斯，正负无本质影响）。

此真点经对应 LiDAR 位姿 ${}^G\mathbf T_{I_k}=({}^G\mathbf R_{I_k},{}^G\mathbf p_{I_k})$ 与外参 ${}^I\mathbf T_L$ 投影到全局系后，应**恰落在地图中的局部小平面片**上（源式 (4)）：

$$
\mathbf 0={}^G\mathbf u_j^\top\Bigl({}^G\mathbf T_{I_k}\,{}^I\mathbf T_L\bigl({}^L\mathbf p_j+{}^L\mathbf n_j\bigr)-{}^G\mathbf q_j\Bigr),\tag{F2-4}
$$

其中 ${}^G\mathbf u_j$ 是对应平面法向量，${}^G\mathbf q_j$ 是平面上一点（Fig. 2）。注意 ${}^G\mathbf T_{I_k}$ 与 ${}^I\mathbf T_{L_k}$（外参）**都含于状态向量 $\mathbf x_k$**。第 $j$ 个点测量贡献的测量可从 (4) 概括为更紧凑形式（源式 (5)）：

$$
\mathbf 0=\mathbf h_j\bigl(\mathbf x_k,{}^L\mathbf p_j+{}^L\mathbf n_j\bigr),\tag{F2-5}
$$

它定义了状态向量 $\mathbf x_k$ 的**隐式测量模型**。

> **scan-to-map 直接配准的关键**：FAST-LIO2 残差是**原始点到地图局部平面**的点到面距离（取最近 5 个地图点拟合局部小平面，法向 ${}^G\mathbf u_j$、质心 ${}^G\mathbf q_j$），**不提取边/面特征**。这与 FAST-LIO 的"提取平面/边特征 + (12) 残差"不同——FAST-LIO2 是 direct method。地图（ikd-Tree）提供 5-NN 搜索。

### §11.3 迭代卡尔曼滤波（继承 FAST-LIO）[源 §IV-B]

基于状态模型 (F2-2) 与测量模型 (F2-5)（建于流形 $\mathcal M\triangleq\mathrm{SO}(3)\times\mathbb R^{15}\times\mathrm{SO}(3)\times\mathbb R^3$），按 [55]、[22] 的流程在流形 $\mathcal M$ 上直接用迭代卡尔曼滤波。两关键步：**每 IMU 测量传播 + 每 LiDAR 扫描迭代更新**，两步都在流形 $\mathcal M$ 上**自然估计状态、避免任何重归一化**。IMU 频率通常高于 LiDAR 扫描（如 IMU 200 Hz、LiDAR 10–100 Hz），一次更新前通常已多次传播。

#### §11.3.1 传播（Propagation）[源 §IV-B-1]

设融合上一（$k-1$）扫描后最优状态估计为 $\bar{\mathbf x}_{k-1}$、协方差 $\bar{\mathbf P}_{k-1}$。每到一个 IMU 测量做前向传播，状态与协方差按 (F2-2) 传播、过程噪声 $\mathbf w_i$ 置零（源式 (6)）：

$$
\hat{\mathbf x}_{i+1}=\hat{\mathbf x}_i\boxplus\bigl(\Delta t\,\mathbf f(\hat{\mathbf x}_i,\mathbf u_i,\mathbf 0)\bigr);\qquad \hat{\mathbf x}_0=\bar{\mathbf x}_{k-1},
$$
$$
\hat{\mathbf P}_{i+1}=\mathbf F_{\tilde{\mathbf x}_i}\,\hat{\mathbf P}_i\,\mathbf F_{\tilde{\mathbf x}_i}^\top+\mathbf F_{\mathbf w_i}\,\mathbf Q_i\,\mathbf F_{\mathbf w_i}^\top;\qquad \hat{\mathbf P}_0=\bar{\mathbf P}_{k-1},
\tag{F2-6}
$$

其中 $\mathbf Q_i$ 是噪声 $\mathbf w_i$ 的协方差，矩阵 $\mathbf F_{\tilde{\mathbf x}_i}$ 与 $\mathbf F_{\mathbf w_i}$ 计算如下（源式 (7)，**更抽象的推导见 [55] IKFOM、更具体的推导见 [22] FAST-LIO**）：

$$
\mathbf F_{\tilde{\mathbf x}_i}=\left.\frac{\partial(\mathbf x_{i+1}\boxminus\hat{\mathbf x}_{i+1})}{\partial\tilde{\mathbf x}_i}\right|_{\tilde{\mathbf x}_i=\mathbf 0,\,\mathbf w_i=\mathbf 0},\qquad
\mathbf F_{\mathbf w_i}=\left.\frac{\partial(\mathbf x_{i+1}\boxminus\hat{\mathbf x}_{i+1})}{\partial\mathbf w_i}\right|_{\tilde{\mathbf x}_i=\mathbf 0,\,\mathbf w_i=\mathbf 0}.\tag{F2-7}
$$

> **抽取注（重要）**：FAST-LIO2 正文**只给出 $\mathbf F_{\tilde{\mathbf x}_i},\mathbf F_{\mathbf w_i}$ 的定义式 (7)（偏导形式），不再展开 24 维的具体块矩阵**，明确指回 FAST-LIO [22]（即本文件 §4.1 的 (7a)(7b)，18 维版）与 IKFOM [55]。综合时，24 维的 $\mathbf F_{\tilde{\mathbf x}}$ 在 18 维 (7a) 基础上对外参两块加单位对角块（外参常量传播 → 对角 $\mathbf I$，无耦合）。

前向传播持续到新（$k$）扫描结束时刻，传播状态与协方差记 $\hat{\mathbf x}_k,\hat{\mathbf P}_k$。

#### §11.3.2 残差计算 [源 §IV-B-2]

设状态 $\mathbf x_k$ 在当前迭代更新（见 §IV-B3）的估计为 $\hat{\mathbf x}_k^\kappa$，$\kappa=0$（首次迭代前）时 $\hat{\mathbf x}_k^0=\hat{\mathbf x}_k$（来自 (6) 的传播预测）。把每个测点 ${}^L\mathbf p_j$ 投影到全局系：

$$
{}^G\hat{\mathbf p}_j={}^G\hat{\mathbf T}_{I_k}^\kappa\,{}^I\hat{\mathbf T}_{L_k}^\kappa\,{}^L\mathbf p_j,
$$

并在 ikd-Tree 表示的地图中搜其**最近 5 个点**（§V-A）。找到的最近邻点用于拟合局部小平面片（法向 ${}^G\mathbf u_j$、质心 ${}^G\mathbf q_j$，用于测量模型 (4)(5)）。把测量方程 (5) 在 $\hat{\mathbf x}_k^\kappa$ 处一阶近似（源式 (8)）：

$$
\mathbf 0=\mathbf h_j(\mathbf x_k,{}^L\mathbf n_j)\simeq\mathbf h_j(\hat{\mathbf x}_k^\kappa,\mathbf 0)+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa+\mathbf v_j=\mathbf z_j^\kappa+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa+\mathbf v_j,\tag{F2-8}
$$

其中 $\tilde{\mathbf x}_k^\kappa=\mathbf x_k\boxminus\hat{\mathbf x}_k^\kappa$（等价 $\mathbf x_k=\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa$）；$\mathbf H_j^\kappa$ 是 $\mathbf h_j(\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa,{}^L\mathbf n_j)$ 关于 $\tilde{\mathbf x}_k^\kappa$ 在零处的雅可比；$\mathbf v_j\in\mathcal N(\mathbf 0,\mathbf R_j)$ 来自原始测量噪声 ${}^L\mathbf n_j$；$\mathbf z_j^\kappa$ 称为**残差**（源式 (9)）：

$$
\mathbf z_j^\kappa=\mathbf h_j(\hat{\mathbf x}_k^\kappa,\mathbf 0)={}^G\mathbf u_j^\top\bigl({}^G\hat{\mathbf T}_{I_k}^\kappa\,{}^I\hat{\mathbf T}_{L_k}^\kappa\,{}^L\mathbf p_j-{}^G\mathbf q_j\bigr).\tag{F2-9}
$$

> **抽取注**：(9) 即"投影点到拟合平面的有向距离 ${}^G\mathbf u_j^\top(\cdot)$"。注意外参 ${}^I\hat{\mathbf T}_{L_k}^\kappa$ 也带迭代上标 $\kappa$（在线标定）。

#### §11.3.3 迭代更新 [源 §IV-B-3]

传播的 $\hat{\mathbf x}_k,\hat{\mathbf P}_k$ 给未知状态 $\mathbf x_k$ 强加先验高斯分布。$\hat{\mathbf P}_k$ 表示如下误差态协方差（源式 (10)）：

$$
\mathbf x_k\boxminus\hat{\mathbf x}_k=(\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa)\boxminus\hat{\mathbf x}_k=\hat{\mathbf x}_k^\kappa\boxminus\hat{\mathbf x}_k+\mathbf J^\kappa\,\tilde{\mathbf x}_k^\kappa\sim\mathcal N(\mathbf 0,\hat{\mathbf P}_k),\tag{F2-10}
$$

其中 $\mathbf J^\kappa$ 是 $(\hat{\mathbf x}_k^\kappa\boxplus\tilde{\mathbf x}_k^\kappa)\boxminus\hat{\mathbf x}_k$ 关于 $\tilde{\mathbf x}_k^\kappa$ 在零处的偏导（源式 (11)，**24 维版，含两个 $\mathbf A(\cdot)$ 块**）：

$$
\mathbf J^\kappa=\begin{bmatrix}
\mathbf A(\delta{}^G\boldsymbol\theta_{I_k})^{-\top} & \mathbf 0_{3\times15} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\[3pt]
\mathbf 0_{15\times3} & \mathbf I_{15\times15} & \mathbf 0 & \mathbf 0\\[3pt]
\mathbf 0_{3\times3} & \mathbf 0_{3\times15} & \mathbf A(\delta{}^I\boldsymbol\theta_{L_k})^{-\top} & \mathbf 0_{3\times3}\\[3pt]
\mathbf 0_{3\times3} & \mathbf 0_{3\times15} & \mathbf 0_{3\times3} & \mathbf I_{3\times3}
\end{bmatrix},\tag{F2-11}
$$

其中 $\mathbf A(\cdot)^{-1}$ 定义见 [22,55]（即本文件 (6)），$\delta{}^G\boldsymbol\theta_{I_k}={}^G\hat{\mathbf R}_{I_k}^\kappa\boxminus{}^G\hat{\mathbf R}_{I_k}$、$\delta{}^I\boldsymbol\theta_{L_k}={}^I\hat{\mathbf R}_{L_k}^\kappa\boxminus{}^I\hat{\mathbf R}_{L_k}$ 分别是 **IMU 姿态误差**与**旋转外参误差**。首次迭代 $\hat{\mathbf x}_k^\kappa=\hat{\mathbf x}_k$ 时 $\mathbf J^\kappa=\mathbf I$。

> **抽取注**：FAST-LIO2 的 $\mathbf J^\kappa$ 有**两个非平凡 $\mathbf A(\cdot)^{-\top}$ 块**（IMU 姿态 + 旋转外参，因状态有两个 $\mathrm{SO}(3)$ 分量），而 FAST-LIO (16) 只有一个（仅 IMU 姿态）。其余 $\mathbb R^n$ 分量对应单位块。

除先验外，测量 (8) 给状态另一分布（源式 (12)）：

$$
-\mathbf v_j=\mathbf z_j^\kappa+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa\sim\mathcal N(\mathbf 0,\mathbf R_j).\tag{F2-12}
$$

把先验 (10) 与测量模型 (12) 结合，得状态 $\mathbf x_k$（由 $\tilde{\mathbf x}_k^\kappa$ 等价表示）的后验分布及其**最大后验估计 MAP**（源式 (13)）：

$$
\min_{\tilde{\mathbf x}_k^\kappa}\;\Bigl(\lVert\mathbf x_k\boxminus\hat{\mathbf x}_k\rVert_{\hat{\mathbf P}_k}^2+\sum_{j=1}^m\lVert\mathbf z_j^\kappa+\mathbf H_j^\kappa\,\tilde{\mathbf x}_k^\kappa\rVert_{\mathbf R_j}^2\Bigr),\tag{F2-13}
$$

其中 $\lVert\mathbf x\rVert_{\mathbf M}^2=\mathbf x^\top\mathbf M^{-1}\mathbf x$。

> **抽取注（与 FAST-LIO (17) 的记号差异）**：FAST-LIO (17) 把范数下标写成 $\hat{\mathbf P}_k^{-1}$、$\mathbf R_j^{-1}$ 且定义 $\lVert\mathbf x\rVert_{\mathbf M}^2=\mathbf x^\top\mathbf M\mathbf x$；FAST-LIO2 (13) 把下标写成 $\hat{\mathbf P}_k$、$\mathbf R_j$ 且定义 $\lVert\mathbf x\rVert_{\mathbf M}^2=\mathbf x^\top\mathbf M^{-1}\mathbf x$。**两者数学完全等价**（只是马氏范数下标用协方差还是其逆的写法约定不同）。

此 MAP 问题可由迭代卡尔曼滤波解（源式 (14)，记 $\mathbf H=[\mathbf H_1^{\kappa\top},\cdots,\mathbf H_m^{\kappa\top}]^\top$、$\mathbf R=\mathrm{diag}(\mathbf R_1,\cdots,\mathbf R_m)$、$\mathbf P=(\mathbf J^\kappa)^{-1}\hat{\mathbf P}_k(\mathbf J^\kappa)^{-\top}$、$\mathbf z_k^\kappa=[\mathbf z_1^{\kappa\top},\cdots,\mathbf z_m^{\kappa\top}]^\top$）：

$$
\mathbf K=(\mathbf H^\top\mathbf R^{-1}\mathbf H+\mathbf P^{-1})^{-1}\mathbf H^\top\mathbf R^{-1},
$$
$$
\hat{\mathbf x}_k^{\kappa+1}=\hat{\mathbf x}_k^\kappa\boxplus\bigl(-\mathbf K\mathbf z_k^\kappa-(\mathbf I-\mathbf K\mathbf H)(\mathbf J^\kappa)^{-1}(\hat{\mathbf x}_k^\kappa\boxminus\hat{\mathbf x}_k)\bigr).\tag{F2-14}
$$

注意卡尔曼增益 $\mathbf K$ 的计算需求逆**状态维数**的矩阵（而非此前工作用的测量维数）。上述过程重复至收敛（$\lVert\hat{\mathbf x}_k^{\kappa+1}\boxminus\hat{\mathbf x}_k^\kappa\rVert<\epsilon$）。收敛后最优状态与协方差（源式 (15)）：

$$
\bar{\mathbf x}_k=\hat{\mathbf x}_k^{\kappa+1},\qquad \bar{\mathbf P}_k=(\mathbf I-\mathbf K\mathbf H)\,\mathbf P.\tag{F2-15}
$$

有了 $\bar{\mathbf x}_k$，第 $k$ 扫描每个 LiDAR 点 ${}^L\mathbf p_j$ 变换到全局系（源式 (16)）：

$$
{}^G\bar{\mathbf p}_j={}^G\bar{\mathbf T}_{I_k}\,{}^I\bar{\mathbf T}_{L_k}\,{}^L\mathbf p_j;\qquad j=1,\cdots,m.\tag{F2-16}
$$

变换后的点 $\{{}^G\bar{\mathbf p}_j\}$ 插入 ikd-Tree 表示的地图（§V）。

### §11.4 算法 1：状态估计（FAST-LIO2 版）[源 §IV-B]

```
算法 1：State Estimation（FAST-LIO2）
输入：上一输出 x̄_{k-1} 与 P̄_{k-1}；当前扫描的 LiDAR 原始点；
      当前扫描期间的 IMU 输入 (a_m, ω_m)。
1   前向传播经 (6) 得状态预测 x̂_k 及其协方差 P̂_k；
2   反向传播补偿运动 [22]；
3   κ = -1, x̂_k^{κ=0} = x̂_k；
4   repeat
5       κ = κ + 1；
6       经 (11) 计算 J^κ，并令 P = (J^κ)^{-1} P̂_k (J^κ)^{-⊤}；
7       经 (8)(9) 计算残差 z_j^κ 与雅可比 H_j^κ；
8       经 (14) 计算状态更新 x̂_k^{κ+1}；
9   until ‖ x̂_k^{κ+1} ⊟ x̂_k^κ ‖ < ε；
10  x̄_k = x̂_k^{κ+1}；P̄_k = (I − KH) P；
11  经 (16) 得变换后的 LiDAR 点 {^G p̄_j}。
输出：当前最优估计 x̄_k 与 P̄_k；变换后的 LiDAR 点 {^G p̄_j}。
```

## §12 建图：ikd-Tree（增量 k-d 树）[源 FAST-LIO2 §V]

> 本节是 FAST-LIO2 相对 FAST-LIO 的最大增量（第一部分无），完整抽取数据结构、地图管理、增量更新、再平衡、kNN、复杂度分析（含两引理证明）。

### §12.1 地图管理 [源 §V-A]

地图点组织进 ikd-Tree，按里程计率合并新扫描点云动态增长。为防地图无界增长，**只维护 LiDAR 当前位置周围一个边长 $L$ 的大局部区域**内的地图点（Fig. 3 给 2D 示意）。地图区初始化为以初始 LiDAR 位置 $\mathbf p_0$ 为中心、边长 $L$ 的立方体。LiDAR 探测区设为以 LiDAR 当前位置（取自 (15)）为中心的**探测球**，半径 $r=\gamma R$（$R$ 为 LiDAR FoV 范围，$\gamma>1$ 为松弛参数）。当 LiDAR 移到新位置 $\mathbf p_0'$ 使探测球触碰地图边界时，地图区沿**增大 LiDAR 探测区与所触边界距离的方向**移动，移动距离设为常量 $d=(\gamma-1)R$。新旧地图区之间的**减法区（subtraction area）**内所有点经盒删（§V-C）从 ikd-Tree 删除。

### §12.2 树结构与构造 [源 §V-B]

#### §12.2.1 数据结构 [源 §V-B-1]

ikd-Tree 是**二叉搜索树**。不同于许多 k-d 树实现只在叶节点存"一桶点"，ikd-Tree **在叶节点与内部节点都存点**，以更好支持动态点插入与树再平衡（单 k-d 树时此存储模式在 kNN 搜索中也更高效 [41]）。因一个点对应树上单个节点，点与节点可互换称呼。

```
数据结构：树节点结构（Tree node structure）
1 Struct TreeNode:
2     PointType point;                              // 点信息（坐标、强度等）
3     TreeNode * leftchild, * rightchild;           // 左右子节点指针
4     int axis;                                     // 分裂空间的划分轴
5     int treesize, invalidnum;                     // 子树节点数（含有效+无效）、无效节点数
6     bool deleted, treedeleted;                    // lazy 标签：本点删除、整子树删除
7     CuboidVertices range;                         // 子树点的范围（外接轴对齐立方体）
8 end
```

属性语义（源逐条）：`point` 存点信息；`leftchild/rightchild` 指左右子节点；`axis` 记分裂轴；`treesize` 记以当前节点为根的(子)树节点数（含有效与无效）；点被移除时不立即删节点，只设 `deleted=true`（见 §V-C2），若整子树被删则 `treedeleted=true`；`invalidnum` 累加子树中已删点数；`range` 记子树点的范围（解释为含所有点的外接轴对齐立方体，由各维最小/最大坐标的两个对角顶点表示）。

#### §12.2.2 构造 [源 §V-B-2]

建 ikd-Tree 类似建静态 k-d 树 [40]：**递归在最长维上的中位点分裂空间**，直到子空间只剩一个点。构造时初始化数据结构中的属性（含计算子树的 treesize 与 range 信息）。

**新插入树节点的属性初始化（源 Table I）**：

| 属性 | 值 | 属性 | 值 |
|---|---|---|---|
| point | $\mathbf p$ | axis¹ | (father.axis + 1) mod $k$ |
| leftchild | NULL | rightchild | NULL |
| treesize | 1 | invalidnum | 0 |
| deleted | false | treedeleted | false |
| range² | $[\mathbf p,\mathbf p]$ | | |

¹ axis 用父节点的划分轴初始化（轮转）。² 立方体的最小/最大顶点都设为待插入点 $\mathbf p$。

### §12.3 增量更新 [源 §V-C]

增量更新指增量操作 + 动态再平衡（§V-D）。支持两类增量操作：**点级操作**（插入/删除/重插入单点）与**盒级操作**（对给定轴对齐立方体内所有点插入/删除/重插入）。两类操作中点插入都进一步集成**树上降采样**（按预设分辨率维护地图）。本文只讲 FAST-LIO2 地图管理所需的**点级插入**与**盒级删除**。

#### §12.3.1 带树上降采样的点插入 [源 §V-C-1]

考虑机器人应用，ikd-Tree 支持同时点插入与地图降采样（算法 2）。对状态估计模块（算法 1）给的 $\{{}^G\bar{\mathbf p}_j\}$ 中的点 $\mathbf p$ 与降采样分辨率 $l$：把空间均分为边长 $l$ 的立方，找含点 $\mathbf p$ 的立方 $C_D$（Line 2）；算法**只保留离 $C_D$ 中心 $\mathbf p_{center}$ 最近的点**（Line 3）——先盒搜 $C_D$ 内所有点存入点数组 $V$（连同新点 $\mathbf p$）（Line 4-5），比较 $V$ 中各点到 $\mathbf p_{center}$ 距离得最近点 $\mathbf p_{nearest}$（Line 6）；删 $C_D$ 内现有点（Line 7），再把 $\mathbf p_{nearest}$ 插入 k-d 树（Line 8）。盒搜实现类似盒删（§V-C2）。

点插入（Line 11-24）在 ikd-Tree 上**递归实现**：从根节点向下搜直到找到空节点附加新节点（Line 12-14），新叶节点属性按 Table I 初始化；在每个非空节点，新点沿划分轴与节点存储点比较以决定递归方向（Line 15-20）；访问节点的属性（如 treesize、range）用最新信息更新（Line 21，见 §V-C3）；对随新点更新的子树检查并维护平衡判据（Line 22，见 §V-D）。

```
算法 2：Point Insertion with On-tree Downsampling（带树上降采样的点插入）
输入：降采样分辨率 l，待插入新点 p，并行重建开关 SW
 1 Algorithm Start
 2     C_D ← FindCube(l, p)                         // 找含 p 的边长 l 立方
 3     p_center ← Center(C_D)
 4     V ← BoxwiseSearch(RootNode, C_D)            // 盒搜 C_D 内所有点
 5     V.push(p)
 6     p_nearest ← FindNearest(V, p_center)         // 取离中心最近点
 7     BoxwiseDelete(RootNode, C_D)                 // 删 C_D 内现有点
 8     Insert(RootNode, p_nearest, NULL, SW)        // 插入最近点
 9 Algorithm End
10
11   Function Insert(T, p, father, SW)
12      if T is empty then
13          Initialize(T, p, father)                // 按 Table I 初始化
14      else
15          ax ← T.axis
16          if p[ax] < T.point[ax] then
17              Insert(T.leftchild, p, T, SW)
18          else
19              Insert(T.rightchild, p, T, SW)
20          end
21          AttributeUpdate(T)                       // 更新 treesize/range 等
22          Rebalance(T, SW)                         // 检查并维护平衡
23      end
24   End Function
```

#### §12.3.2 用 lazy 标签的盒删 [源 §V-C-2]

删除用 **lazy delete 策略**：点不立即从树移除，只把属性 `deleted` 设 true（数据结构 Line 6）。若以节点 $T$ 为根的子树所有节点都已删，$T$ 的 `treedeleted` 设 true。因此 `deleted` 与 `treedeleted` 称为 **lazy 标签**。标 "deleted" 的点在重建过程中被移除（§V-D）。

盒删利用 `range` 属性中的范围信息与树节点上的 lazy 标签。`range` 用外接立方体 $C_T$ 表示。给定要从以 $T$ 为根的(子)树删除的点立方 $C_O$，算法递归向下并比较外接立方 $C_T$ 与给定立方 $C_O$：
- 若 $C_T$ 与 $C_O$ **无交**，递归直接返回不更新树（Line 3）。
- 若 $C_T$ **完全含于** $C_O$，盒删把 `deleted` 与 `treedeleted` 设 true（Line 5）；因子树全部点被删，`invalidnum` 等于 `treesize`（Line 6）。
- 若 $C_T$ 与 $C_O$ **相交但不被含**，当前点 $\mathbf p$ 若含于 $C_O$ 则先从树删（Line 9），再递归看子节点（Line 10-11）；之后做属性更新与平衡维护（Line 12-13）。

```
算法 3：Box-wise Delete（盒删）
输入：操作立方 C_O，k-d 树节点 T，并行重建开关 SW
 1 Function BoxwiseDelete(T, C_O, SW)
 2    C_T ← T.range
 3    if C_T ∩ C_O = ∅ then return                 // 无交，直接返回
 4    if C_T ⊆ C_O then                            // 完全包含
 5        T.treedelete, T.delete ← true
 6        T.invalidnum = T.treesize
 7    else                                          // 相交但不被含
 8        p ← T.point
 9        if p ⊂ C_O then T.treedelete = true       // 注：此处应为 T.delete（删本点）
10        BoxwiseDelete(T.leftchild, C_O, SW)
11        BoxwiseDelete(T.rightchild, C_O, SW)
12        AttributeUpdate(T)
13        Rebalance(T, SW)
14    end
15 End Function
```

> **抽取注**：算法 3 Line 9 源排版为 `T.treedelete = true`，但语义应是删**本节点点**（`T.delete = true`），因这里只判单点 $\mathbf p$ 是否在 $C_O$ 内。照录源文，并标注此处疑为 `T.delete`。

#### §12.3.3 属性更新 [源 §V-C-3]

每次增量操作后用函数 `AttributeUpdate` 更新访问节点的属性：`treesize` 与 `invalidnum` 由两子节点对应属性 + 自身点信息汇总；`range` 由两子节点 range 信息 + 自身点合并确定；`treedeleted` 当两子节点 treedeleted 都为 true 且本节点也已删时设 true。

### §12.4 再平衡（Re-balancing）[源 §V-D]

ikd-Tree 在每次增量操作后主动监测平衡性，仅重建相关子树以动态再平衡自身。

#### §12.4.1 平衡判据 [源 §V-D-1]

平衡判据由两个子判据组成：**$\alpha$-平衡判据**与 **$\alpha$-删除判据**。设 ikd-Tree 一子树以 $T$ 为根。子树 **$\alpha$-平衡**当且仅当满足（源式 (17)）：

$$
S(T.leftchild)<\alpha_{bal}\bigl(S(T)-1\bigr),\qquad S(T.rightchild)<\alpha_{bal}\bigl(S(T)-1\bigr),\tag{17}
$$

其中 $\alpha_{bal}\in(0.5,1)$，$S(T)$ 是节点 $T$ 的 treesize 属性。

子树 **$\alpha$-删除**判据（源式 (18)）：

$$
I(T)<\alpha_{del}\,S(T),\tag{18}
$$

其中 $\alpha_{del}\in(0,1)$，$I(T)$ 是子树上无效节点数（即节点 $T$ 的 invalidnum 属性）。

子树同时满足两判据则平衡；整树平衡当且仅当所有子树平衡。**违反任一判据触发重建以再平衡该子树**：$\alpha$-平衡判据维护树的最大高度（**易证 $\alpha$-平衡树的最大高度是 $\log_{1/\alpha_{bal}}(n)$**，$n$ 为树尺寸）；$\alpha$-删除判据确保子树上无效节点（标 "deleted"）被移除以减小树尺寸。减小 k-d 树高度与尺寸使未来增量操作与查询高效。

#### §12.4.2 重建与并行重建 [源 §V-D-2]

设子树 $\mathcal T$ 触发重建（Fig. 4）：子树先**展平（flatten）**进点存储数组 $V$，标 "deleted" 的节点在展平时丢弃；再用 $V$ 中所有点按 §V-B **建一个完美平衡 k-d 树**。重建大子树会有可观延迟、损害实时性。为保高实时性，设计**双线程重建法**：不简单地在第二线程重建，而用**操作记录器（operation logger）**避免两线程信息丢失与内存冲突，从而始终保持 kNN 搜索全精度。

重建法（算法 4）：平衡判据违反时，子树 treesize 小于预设值 $N_{max}$ 时在主线程重建；否则在第二线程重建。第二线程重建（函数 `ParRebuild`）：记要重建子树为 $\mathcal T$、根 $T$。第二线程锁定此子树上**所有增量更新**（点插入与删除）但**不锁查询**（Line 12）；然后把子树 $\mathcal T$ 内所有有效点拷进点数组 $V$（即展平），同时保留原子树不变供重建期间可能的查询（Line 13）；展平后解锁原子树供主线程接收后续增量更新请求（Line 14），这些请求同时记录进名为 operation logger 的队列；第二线程从 $V$ 建好新平衡 k-d 树 $\mathcal T'$ 后（Line 15），记录的更新请求经函数 `IncrementalUpdates` 在 $\mathcal T'$ 上重放（Line 16-18，注意并行重建开关设 false 因已在第二线程）；所有挂起请求处理后，原子树 $\mathcal T$ 点信息与新子树 $\mathcal T'$ 完全相同（只是新子树更平衡）；算法锁定节点 $T$ 的增量更新与查询并用新树 $\mathcal T'$ 替换（Line 20-22）；最后释放原子树内存（Line 23）。此设计确保第二线程重建期间主线程建图仍按里程计率无中断进行（虽因临时不平衡 k-d 树效率略低）。注意 `LockUpdates` 不阻塞查询（可在主线程并行进行）；`LockAll` 阻塞包括查询的所有访问但很快完成（仅一条指令），允许主线程及时查询。`LockUpdates` 与 `LockAll` 由互斥锁（mutex）实现。

```
算法 4：Rebuild (sub-)tree for re-balancing（重建子树以再平衡）
输入：待重建(子)树 𝒯 的根节点 T，重建开关 SW
 1 Function Rebalance(T, SW)
 2    if ViolateCriterion(T) then
 3        if T.treesize < N_max or Not SW then
 4            Rebuild(T)                            // 主线程重建
 5        else
 6            ThreadSpawn(ParRebuild, T)            // 派生第二线程重建
 7        end
 8    end
 9 End Function
10
11   Function ParRebuild(T)
12      LockUpdates(T)                              // 锁增量更新，不锁查询
13      V ← Flatten(T)                             // 展平（丢弃 deleted 节点）
14      Unlock(T)                                   // 解锁供主线程
15      𝒯' ← Build(V)                              // 第二线程建新平衡树
16      foreach op in OperationLogger do            // 重放期间累积的更新
17          IncrementalUpdates(𝒯', op, false)
18      end
19      T_temp ← T
20      LockAll(T)                                  // 锁所有访问（含查询，极快）
21      T ← 𝒯'                                      // O(1) 替换
22      Unlock(T)
23      Free(T_temp)                               // 释放原子树内存
24   End Function
```

### §12.5 k-最近邻搜索 [源 §V-E]

虽类似已知 k-d 树库 [43-45] 的实现，ikd-Tree 的最近搜索经彻底优化。树节点上的 `range` 信息用 "bounds-overlap-ball" 测试 [41] 加速最近邻搜索。维护一个**优先队列 $q$** 存至今遇到的 k 个最近邻及其到目标点距离。从根节点递归向下搜时，先算目标点到树节点立方 $C_T$ 的**最小距离 $d_{min}$**；若 $d_{min}$ 不小于 $q$ 中最大距离，则无需处理该节点及其后代节点。此外，FAST-LIO2（及许多 LiDAR 里程计）中只有目标点给定阈值内的邻点才视作内点用于状态估计，这天然给 k-最近邻**范围搜索**提供最大搜索距离 [43]。无论何种情形，范围搜索通过比较 $d_{min}$ 与最大距离剪枝，减少回溯以改善时间性能。ikd-Tree 支持多线程 kNN 搜索以适配并行计算架构。

### §12.6 时间复杂度分析 [源 §V-F]

ikd-Tree 时间复杂度分为增量操作（插入与删除）、重建、kNN 搜索三部分（均在**低维**假设下，如 FAST-LIO2 的三维）。

#### §12.6.1 增量操作 [源 §V-F-1]

因带树上降采样的插入依赖盒删与盒搜，先讨论盒操作。设 $n$ 为 ikd-Tree 尺寸。

**引理 1（盒操作复杂度）**：设 ikd-Tree 上点在 3-d 空间 $S_x\times S_y\times S_z$，操作立方 $C_D=L_x\times L_y\times L_z$。算法 3 用立方 $C_D$ 的盒删与盒搜的时间复杂度为（源式 (19)）：

$$
O(H(n))=\begin{cases}
O(\log n) & \text{if }\Delta_{min}>\alpha(\tfrac23)\quad(\ast)\\[4pt]
O(n^{1-a-b-c}) & \text{if }\Delta_{max}\le 1-\alpha(\tfrac13)\quad(\ast\ast)\\[4pt]
O(n^{\alpha(1/3)-\Delta_{min}-\Delta_{med}}) & \text{if }(\ast)\text{ 与 }(\ast\ast)\text{ 均不成立 }\\
& \quad\text{且 }\Delta_{med}<\alpha(\tfrac13)-\alpha(\tfrac23)\\[4pt]
O(n^{\alpha(2/3)-\Delta_{min}}) & \text{otherwise}
\end{cases}\tag{19}
$$

其中 $a=\log_n\frac{S_x}{L_x}$、$b=\log_n\frac{S_y}{L_y}$、$c=\log_n\frac{S_z}{L_z}$（$a,b,c>0$）；$\Delta_{min},\Delta_{med},\Delta_{max}$ 是 $a,b,c$ 中的最小、中位、最大值；$\alpha(u)$ 是 **flajolet-puech 函数**（$u\in[0,1]$），特定值：

$$
\alpha(\tfrac13)=0.7162,\qquad \alpha(\tfrac23)=0.3949.
$$

> **抽取注**：源式(19) pdftotext 排版极乱。已按 ar5iv 语义重排为四分支分段函数。$\Delta_{min}$ 用于条件 $(\ast)$，$\Delta_{max}$ 用于 $(\ast\ast)$；第三支额外条件 $\Delta_{med}<\alpha(1/3)-\alpha(2/3)$。

**证明（引理 1）**：k-d 树上轴对齐超立方范围搜索的渐近时间复杂度由 [56] 给出。盒删可视作范围搜索，只是树节点附 lazy 标签（$O(1)$）。故范围搜索结论可应用于 ikd-Tree 的盒删与盒搜，得 $O(H(n))$。$\blacksquare$

**引理 2（带树上降采样的插入复杂度）**：算法 2 中带树上降采样的点插入在 ikd-Tree 上的时间复杂度为 $O(\log n)$。

**证明（引理 2）**：ikd-Tree 上的降采样方法由盒搜与盒删 + 点插入组成。由引理 1，降采样时间复杂度为 $O(H(n))$。一般降采样立方 $C_D$ 相比整个空间很小，故归一化范围 $\Delta x,\Delta y,\Delta z$ 小、$\Delta_{min}$ 满足条件 $(\ast)$，得 $O(\log n)$。ikd-Tree 的最大高度由式(17) 易证为 $\log_{1/\alpha_{bal}}(n)$（静态 k-d 树为 $\log_2 n$）。故由 [40]（k-d 树点插入证明为 $O(\log n)$）直接得插入 $O(\log n)$。汇总降采样与插入的复杂度，得带树上降采样的插入复杂度为 $O(\log n)$。$\blacksquare$

#### §12.6.2 重建 [源 §V-F-2]

重建复杂度分两类：单线程重建与并行双线程重建。
- **单线程重建**：主线程递归进行，每层耗排序时间 $O(n)$，低维 $k$ 下 $\log n$ 层总时间 $O(n\log n)$ [40]。
- **并行重建**：主线程仅耗展平（暂停主线程进一步增量更新，算法 4 Line 12-14）与树更新（常量时间 $O(1)$，Line 20-22），**不含建树**（由第二线程并行，Line 15-18），故从主线程看复杂度 $O(n)$。

总结：重建 ikd-Tree 的时间复杂度为**双线程并行重建 $O(n)$**、**单线程重建 $O(n\log n)$**。

#### §12.6.3 kNN 搜索 [源 §V-F-3]

因 ikd-Tree 最大高度维护不超过 $\log_{1/\alpha_{bal}}(n)$，从根搜到叶的时间复杂度 $O(\log n)$。kNN 搜索过程中回溯次数正比于与树尺寸无关的常量 $\bar l$ [41]。故 ikd-Tree 上获 k-最近邻的**期望时间复杂度 $O(\log n)$**。

## §13 FAST-LIO2 实验与实现要点 [源 FAST-LIO2 §VI, §VII]

> 保留关键实现配置与数值（非理论核心）。

### §13.1 实现配置 [源 §VII-A]

C++ + ROS 实现。迭代卡尔曼滤波基于 IKFOM 工具箱 [55]。默认配置：
- **局部图尺寸 $L=1000$ m**；
- LiDAR 原始点经 **1:4 时间降采样**（每四个点取一个）直接送状态估计；
- **空间降采样分辨率 $l=0.5$ m**（所有实验）；
- **ikd-Tree 参数**：$\alpha_{bal}=0.6$、$\alpha_{del}=0.5$、$N_{max}=1500$；
- **kNN 找 5 个最近邻**。
- 基准平台：DJI Manifold 2-C（1.8 GHz 四核 Intel i7-8550U，8 GB RAM）；也测 ARM 平台 Khadas VIM3（低功耗 2.2 GHz 四核 Cortex-A73，4 GB RAM）。

### §13.2 数据结构基准（源 Table II 数据集 + Table III 时间）

**基准数据集（源 Table II）**：lili（固态 LiDAR Livox Horizon，非重复扫描，81.7°×25.1° FoV，10 Hz；6 轴 Xsens MTi-670 IMU 200 Hz）；utbm（旋转 Velodyne HDL-32E，32 线，10 Hz；6 轴 Xsens IMU 100 Hz）；ulhk（Velodyne HDL-32E，32 线，10 Hz；9 轴 Xsens MTi-10 IMU 100 Hz）；nclt（Velodyne HDL-32E，32 线，10 Hz；Microstrain MS25 IMU 50 Hz，为让 LIO-SAM 工作经零阶插值升到 100 Hz）；liosam（VLP-16，16 线，10 Hz；MicroStrain 3DM-GX5-25 9 轴 IMU 1000 Hz）。

**数据结构对比（源 Table III 摘录，单扫描平均时间 ms）**：ikd-Tree 在增量更新 + kNN 搜索的**总时间**上整体最优。例（utbm 1）：ikd-Tree 总 18.42 ms vs nanoflann 19.22 vs Octree 45.00 vs R*-tree 26.50。Octree 插入最快但 kNN 慢（树不平衡）；nanoflann 插入偶有巨峰（对数结构 + lazy 不真删致树尺寸暴涨，长序列尤甚：nanoflann 树尺寸 utbm 超 $6\times10^6$、nclt 超 $10^7$，而 ikd-Tree 最大 $2\times10^6$、$3.6\times10^6$；nanoflann 增量更新最大处理时间 7 个 utbm 超 3 s、3 个 nclt 超 7 s，ikd-Tree 全程最大 214.4 ms@nclt 2、其余 17 序列 <150 ms）。插入 + kNN 时间确实正比 $\log n$，与 §V-F 分析一致。

### §13.3 主要结论 [源 §VIII]

FAST-LIO2 计算高效（大室外环境 100 Hz 里程计与建图）、鲁棒（杂乱室内可靠位姿，旋转达 1000 deg/s）、通用（多线旋转 + 固态 LiDAR、UAV + 手持、Intel + ARM），且精度高于现有方法。两大新意（直接原始点配准、ikd-Tree）使之能在算力受限平台实时运行。

## §14 官方实现/使用要点 [源 GitHub hku-mars/FAST_LIO README]

> 供综合时落地参考（参数名照录）。

- **支持的 LiDAR**：旋转式 Velodyne、Ouster；固态 Livox Avia、Horizon、MID-70（可扩展）。
- **主要 launch 参数**（名照录）：`lid_topic`（LiDAR 点云话题）、`imu_topic`（IMU 话题）、`extrinsic_T`（平移外参，LiDAR 在 IMU 体系的位姿）、`extrinsic_R`（旋转外参，**仅旋转矩阵格式**）、`extrinsic_est_en`（开关外参估计，对应 FAST-LIO2 的在线标定）、`timestamp_unit`（PointCloud2 时间字段单位）、`scan_line`（LiDAR 线数，已测 16/32/64）、`time_sync_en`（软件时间同步开关）、`pcd_save_enable`（PCD 输出开关）。
- **IMU 要求**：IMU 与 LiDAR 必须**同步**（"that's important"）；**每点时间戳**对运动去畸变至关重要（Livox 需用 `livox_lidar_msg.launch` 产逐点时间戳）。
- **依赖与 ROS**：Ubuntu ≥ 16.04（18.04+ 默认 PCL/Eigen 即可）；ROS ≥ Melodic；PCL ≥ 1.8；Eigen ≥ 3.3.4；`livox_ros_driver` 须先装并 source。
- **FAST-LIO → FAST-LIO2 关系**：FAST-LIO2 改进为直接原始点里程计（特征提取可选）、ikd-Tree 增量建图、>100 Hz LiDAR 率处理。

---

## §X 本源未覆盖项（本主题中两文不含，需综合 agent 从他源补齐）

> 本章【聚焦】要求覆盖"激光里程计与建图；scan-to-scan/scan-to-map；松/紧耦合 LIO；FAST-LIO 迭代 ESKF 完整推导；**回环与位姿图**"。FAST-LIO/FAST-LIO2 对前四项做到全量保真，但下列项**两文确实没有**：

1. **回环检测（Loop Closure Detection）**：FAST-LIO/FAST-LIO2 是**纯递归滤波里程计**，**无回环检测**（不含 Scan Context、特征描述子、词袋 BoW 等回环识别）。两文只在相关工作里提及 LeGO-LOAM/LIO-SAM 有回环模块。**需从 LIO-SAM（Shan 2020, IROS）、SC-LIO-SAM、或回环检测专门源补齐**。
2. **位姿图优化（Pose Graph Optimization）/ 因子图平滑**：FAST-LIO/2 不做全局位姿图优化或因子图（GTSAM/iSAM2）。FAST-LIO2 §II 提到 LIO-SAM 用因子图平滑 [32]、LINS 用 map refining，但本身**不实现**。**需从 LIO-SAM、Barfoot 批量估计、或 GTSAM 文档补齐**位姿图节点/边、回环边、边缘化、增量平滑。
3. **scan-to-scan 配准的具体算法（ICP/GICP/NDT 的完整推导）**：两文只引用 ICP[6]、GICP[7]、NDT，**不给 ICP/GICP/NDT 的迭代推导与点到面/点到点最小二乘闭式**。**需从 Besl-McKay、Segal GICP、Biber NDT 原文补齐**。
4. **LOAM 的特征提取与点到线/点到面残差细节**：FAST-LIO 用 LOAM[8]/LOAM-Livox[10] 的特征提取与 KD-tree，但**不重新推导 LOAM 的曲率/光滑度特征、点到线点到面残差**。**需从 LOAM（Zhang-Singh 2014）原文补齐**。
5. **IMU 预积分（Preintegration）**：FAST-LIO/2 用**离散运动学 + ESKF 传播**（不是优化框架的预积分）。两文提到 LIOM/Geneva 用 IMU 预积分 [15]，但本身**不用预积分**。预积分推导（Forster 2017）属优化类 LIO，**需他源补齐**（与本章 FAST-LIO 的滤波路线对照）。
6. **可观测性分析**：FAST-LIO 引用 [24]（Xu et al. 基于统计运动模型的状态估计与可观测性分析），但本文**不含可观测性推导**。
7. **$\mathbf F_{\tilde{\mathbf x}},\mathbf F_{\mathbf w}$ 的逐块中间代数**：FAST-LIO 附录 A 只给链式法则框架 (23) + 最终结果 (7)，**未逐块展开 $\mathrm{Exp}/\mathbf A(\cdot)$ 雅可比的中间代数**；FAST-LIO2 更是只给定义式 (7) 指回 [22][55]。**逐块推导需从 IKFOM（Xu et al. *Kalman Filters on Differentiable Manifolds*）或本书 ESKF 章（Solà/Barfoot）的 $\mathfrak{so}(3)$ 误差传播雅可比补齐**。
8. **$\mathbf A(\mathbf u)$（右雅可比逆）的推导**：两文只给 $\mathbf A(\mathbf u)^{-1}$ 闭式 (6) 并引 Bullo-Murray[25]，**不推导**。本书 $\mathrm{SO}(3)$ 右雅可比 $\mathbf J_r$ 章已有，综合时指明 $\mathbf A(\mathbf u)^{-1}=\mathbf J_r(\mathbf u)^{-1}$ 即可。

**已全量覆盖**（本文件第一、二部分）：FAST-LIO 完整 18 维状态方程、连续/离散运动学、⊞/⊟ 流形封装、前向传播（含 $\mathbf F_{\tilde{\mathbf x}},\mathbf F_{\mathbf w}$ 全矩阵 + 协方差传播）、反向传播运动补偿、残差（点到面/点到线）、迭代 ESKF 更新（含 $\mathbf J^\kappa$ 切空间搬运）、两种卡尔曼增益 + 完整等价性证明、算法 1；FAST-LIO2 的 24 维状态（含外参在线标定）、二阶位置积分的 $\mathbf f$、scan-to-map 直接配准测量模型、迭代更新（双 $\mathbf A(\cdot)$ 块的 $\mathbf J^\kappa$）、算法 1；ikd-Tree 全部（数据结构、地图管理、点插入/盒删/属性更新算法、双线程重建、kNN、复杂度引理 1/2 含证明）。
