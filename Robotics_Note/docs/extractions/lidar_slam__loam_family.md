# 抽取留痕 — 激光SLAM：LOAM / LeGO-LOAM / LIO-SAM / FAST-LIO 家族（特征里程计、建图与回环）

> **本抽取服务章节**：`激光SLAM`（激光里程计与建图；scan-to-scan / scan-to-map；松/紧耦合 LIO；FAST-LIO 迭代 ESKF；回环与位姿图）。
>
> **来源（联网研究，权威原始文献）**：
> 1. **LOAM**：J. Zhang and S. Singh, *"LOAM: Lidar Odometry and Mapping in Real-time"*, Robotics: Science and Systems (RSS) X, 2014. PDF: <https://www.roboticsproceedings.org/rss10/p07.pdf>（已逐字读完，639 行抽取文本）。
> 2. **LeGO-LOAM**：T. Shan and B. Englot, *"LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain"*, IEEE/RSJ IROS 2018, pp. 4758–4765. 预印本 PDF: <https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/Shan_Englot_IROS_2018_Preprint.pdf>（已逐字读完，563 行）。
> 3. **LIO-SAM**：T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, D. Rus, *"LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping"*, IEEE/RSJ IROS 2020. arXiv:2007.00258v3 <https://arxiv.org/abs/2007.00258>（已逐字读完，546 行）。
> 4. **FAST-LIO**：W. Xu and F. Zhang, *"FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter"*, IEEE RA-L, vol. 6, no. 2, pp. 3317–3324, 2021. arXiv:2010.08196v3 <https://arxiv.org/abs/2010.08196>（已逐字读完，含两处附录证明，748 行）。
> 5. **（补充·使 IMU 预积分自包含）** C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, *"On-Manifold Preintegration for Real-Time Visual–Inertial Odometry"*, IEEE T-RO, vol. 33(1), 2016. arXiv:1512.02363 <https://arxiv.org/abs/1512.02363>（LIO-SAM 的预积分因子直接引用本文；抽其预积分测量/残差/偏置修正公式）。
> 6. **（补充·使 LeGO-LOAM 分割自包含）** I. Bogoslavskyi and C. Stachniss, *"Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation"*, IEEE/RSJ IROS 2016, pp. 163–169. PDF: <https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/bogoslavskyi16iros.pdf>（LeGO-LOAM 的图像分割[23]即本文；抽其角度 β 判据与算法）。
>
> **保真承诺**：本文件按各源论文小节顺序，逐条记录每一个定义、每一条公式（LaTeX 写全，保留原编号）、每一步推导（中间代数不跳）、每一道数值例/实验表、每一段算法伪码。不做摘要、不做凝练。FAST-LIO 的迭代 ESKF 完整推导（含 MAP→Kalman、新增益公式及其等价性证明）逐行展开。

---

## 0. 记号约定（各源）+ 与本书统一约定的差异

本章涉及多篇论文，记号体系各不相同。先列**与本书统一约定的全局对照**，再逐源列细节。

### 0.0 本书统一约定（编写规范 §五）

- 旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$（不用 $\mathbf C$）；位姿 $\mathbf T\in\mathrm{SE}(3)$。
- 四元数：Hamilton。
- **扰动以右扰动为主**：$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$，局部坐标，右雅可比 $\mathbf J_r$ 为主。
- $\mathfrak{se}(3)$ 排序 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）。
- 坐标系下标语义 $\mathbf T_{wc}$ = 相机/体到世界。
- 协方差用 $\boldsymbol\Sigma$，信息矩阵 $\boldsymbol\Sigma^{-1}$。

### 0.1 LOAM 记号（源 §III）

| 记号 | 含义（LOAM 用法） |
|---|---|
| $\{L\}$ | **Lidar 坐标系**：原点在 lidar 几何中心，x 轴向左、y 轴向上、z 轴向前（注意是右手系但**非常规** ENU/前左上）。 |
| $\{W\}$ | **World 坐标系**：与初始时刻的 $\{L\}$ 重合。 |
| 右上标 | 指示坐标系（如 $\mathbf X^L$ 表示在 $\{L\}$ 中的坐标）。 |
| 右下标 $k\in\mathbb Z^+$ | 指示第 $k$ 次 **sweep**（一次完整扫描覆盖）。 |
| $\mathcal P_k$ | sweep $k$ 期间感知到的点云。 |
| $\mathbf X^L_{(k,i)}$ | 点 $i$（$i\in\mathcal P_k$）在 $\{L_k\}$ 中的坐标。 |
| $\hat{\mathcal P}$ | 单次 laser scan 收到的点（一条扫描线）。 |
| $\bar{\mathcal P}_k$ | sweep $k$ 结束后重投影到 $t_{k+1}$ 的去畸变点云。 |
| $\mathcal E_{k+1},\mathcal H_{k+1}$ | 从 $\mathcal P_{k+1}$ 提取的**边缘点 / 平面点**集合。 |
| $\tilde{\mathcal E}_{k+1},\tilde{\mathcal H}_{k+1}$ | 重投影到 sweep 起点 $t_{k+1}$ 的边缘/平面点集合。 |
| $\mathbf T^L_{k+1}$ | $[t_{k+1},t]$ 间的 lidar 位姿变换，$\mathbf T^L_{k+1}=[t_x,t_y,t_z,\theta_x,\theta_y,\theta_z]^T$（6-DOF）。 |
| $\mathbf T^W_k$ | 建图算法在 sweep $k$ 末（$t_{k+1}$）给出的 lidar 在 map 上的位姿。 |
| $c$ | 局部曲面**光滑度 / 曲率**（smoothness）。 |
| $d_{\mathcal E},d_{\mathcal H}$ | 点到边缘线距离、点到平面距离。 |
| $\mathbf J$ | $\mathbf J=\partial\mathbf f/\partial\mathbf T^L_{k+1}$，Jacobian。 |

**LOAM 与本书的差异**：(1) LOAM 旋转用 Rodrigues 指数映射，本书右扰动主线兼容（LOAM 是 scan-to-scan 优化、非滤波，无左右扰动之分）。(2) LOAM 的 $\{L\}$ 轴向是"左-上-前"，本书常用"前-左-上"或相机系"右-下-前"，移植时须显式标轴。(3) LOAM 6-DOF 位姿排序是"平移在前、旋转在后"$[t_x,t_y,t_z,\theta_x,\theta_y,\theta_z]$，与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 排序一致（平移在前），但 LOAM 的旋转分量 $[\theta_x,\theta_y,\theta_z]$ 是 $\mathfrak{so}(3)$ 的指数坐标（轴角向量），且 LOAM 的平移分量直接是欧氏平移（**不是** $\mathfrak{se}(3)$ 的 $\boldsymbol\rho$，二者相差左雅可比 $\mathbf J$）。

### 0.2 LeGO-LOAM 记号（源 §III）

继承 LOAM 大部分记号，新增：

| 记号 | 含义 |
|---|---|
| $\mathcal P_t=\{p_1,\dots,p_n\}$ | 时刻 $t$ 获取的点云。 |
| $r_i$ | 点 $p_i$ 对应的距离值（range，到传感器的欧氏距离）。 |
| $\mathcal S$ | 与 $p_i$ 同一**行**（range image 同一 row）的连续点集合，$|\mathcal S|=10$（两侧各 5）。 |
| $c$ | 粗糙度 roughness（同 LOAM 的光滑度，见式 (1)）。 |
| $c_{th}$ | 区分边缘/平面特征的阈值。 |
| $\mathbb F_e,\mathbb F_p$ | lidar **odometry** 模块用的边缘/平面特征集合（粗，数量多）。 |
| $F_e,F_p$ | lidar **mapping** 模块用的边缘/平面特征集合（精，数量少），$F_e\subset\mathbb F_e$、$F_p\subset\mathbb F_p$。 |
| $n_{\mathbb F_e},n_{\mathbb F_p},n_{F_e},n_{F_p}$ | 各子图每行抽取的特征数 = 2, 4, 40, 80。 |
| $[t_z,\theta_{roll},\theta_{pitch}]$ | **两步优化第一步**（用地面平面特征求得）。 |
| $[t_x,t_y,\theta_{yaw}]$ | **两步优化第二步**（用边缘特征求得）。 |
| $\mathbb M^{t-1}$ | 保存所有历史特征集 $\{F^i_e,F^i_p\}$ 的集合。 |
| $Q^{t-1}$ | 周围点云地图（surrounding map）。 |

### 0.3 LIO-SAM 记号（源 §III-A）

| 记号 | 含义 |
|---|---|
| $\mathsf W$ | world frame；$\mathsf B$ | robot **body** frame（假设 IMU 系与 body 系重合）。 |
| $\mathbf x=[\mathbf R^T,\mathbf p^T,\mathbf v^T,\mathbf b^T]^T$ | 机器人状态：$\mathbf R\in\mathrm{SO}(3)$ 旋转、$\mathbf p\in\mathbb R^3$ 位置、$\mathbf v$ 速度、$\mathbf b$ IMU 偏置。 |
| $\mathbf T=[\mathbf R\mid\mathbf p]\in\mathrm{SE}(3)$ | $\mathsf B\to\mathsf W$ 变换。 |
| $\hat{\boldsymbol\omega}_t,\hat{\mathbf a}_t$ | IMU 原始角速度、加速度（在 $\mathsf B$）测量。 |
| $\mathbf b^\omega_t,\mathbf b^a_t$ | 陀螺/加表偏置；$\mathbf n^\omega_t,\mathbf n^a_t$ | 白噪声。 |
| $\mathbf R^{BW}_t$ | $\mathsf W\to\mathsf B$ 旋转矩阵；$\mathbf g$ | $\mathsf W$ 中常重力。 |
| $\Delta\mathbf v_{ij},\Delta\mathbf p_{ij},\Delta\mathbf R_{ij}$ | $i,j$ 间预积分测量。 |
| $\mathbb F^e_i,\mathbb F^p_i$ | 时刻 $i$ 的边缘/平面特征；$\mathbb F_i=\{\mathbb F^e_i,\mathbb F^p_i\}$ 一帧 lidar frame。 |
| $\mathbb M_i=\{\mathbb M^e_i,\mathbb M^p_i\}$ | 由 $n$ 个 sub-keyframe 合成的体素地图（边缘子图 + 平面子图）。 |
| $\mathbf T_i$ | 第 $i$ 个 keyframe 关联的位姿；$\Delta\mathbf T_{i,i+1}$ | lidar odometry 因子（相对变换）。 |

**LIO-SAM 与本书一致**：$\mathbf R\in\mathrm{SO}(3)$、$\mathbf T\in\mathrm{SE}(3)$ 与本书完全一致（LIO-SAM 是少数直接用 $\mathbf R$ 的论文）。预积分用 Forster 的 Hamilton-兼容 SO(3) 公式，与本书一致。

### 0.4 FAST-LIO 记号（源 §III-B，Table I）

| 记号 | 含义 |
|---|---|
| $t_k$ | 第 $k$ 个 LiDAR scan 的结束时刻。 |
| $\tau_i$ | 一个 scan 内第 $i$ 个 IMU 采样时刻。 |
| $\rho_j$ | 一个 scan 内第 $j$ 个特征点采样时刻。 |
| $I_i,I_j,I_k$ | $\tau_i,\rho_j,t_k$ 时刻的 **IMU body frame**。 |
| $L_j,L_k$ | $\rho_j,t_k$ 时刻的 **LiDAR body frame**。 |
| $\mathbf x,\hat{\mathbf x},\bar{\mathbf x}$ | $\mathbf x$ 的**真值、前向传播预测值、更新后值**。 |
| $\tilde{\mathbf x}$ | 真值与估计之间的**误差** $\tilde{\mathbf x}=\mathbf x\boxminus\bar{\mathbf x}$。 |
| $\hat{\mathbf x}^\kappa$ | 迭代 Kalman 滤波中 $\mathbf x$ 的第 $\kappa$ 次更新。 |
| $\mathbf x_i,\mathbf x_j,\mathbf x_k$ | $\tau_i,\rho_j,t_k$ 时刻的状态向量。 |
| $\check{\mathbf x}_j$ | 反向传播中 $\mathbf x_j$ 相对 $\mathbf x_k$ 的估计。 |
| $\boxplus,\boxminus$ | 流形封装算子（见 §FAST-LIO.B.1）。 |
| ${}^G\mathbf R_I,{}^G\mathbf p_I$ | IMU 在 global frame $G$（= 第一个 IMU frame）中的姿态、位置。 |
| ${}^G\mathbf g$ | global frame 中的未知重力向量。 |
| ${}^I\mathbf T_L=({}^I\mathbf R_L,{}^I\mathbf p_L)$ | LiDAR-IMU 已知外参。 |
| $\delta\boldsymbol\theta=\mathrm{Log}({}^G\bar{\mathbf R}_I^T\,{}^G\mathbf R_I)$ | **姿态误差**（注意：这是**左扰动**形式，见下方差异说明）。 |
| $\mathbf P$ | 状态误差协方差（在切空间）。 |
| $\mathbf K$ | Kalman 增益。 |
| $\mathbf H^\kappa_j$ | 测量模型对 $\tilde{\mathbf x}^\kappa_k$ 的 Jacobian。 |

**FAST-LIO 与本书的关键差异（必须转换）**：
- **左扰动**：FAST-LIO 的姿态误差定义为 $\delta\boldsymbol\theta=\mathrm{Log}({}^G\bar{\mathbf R}_I^T\,{}^G\mathbf R_I)$，即 ${}^G\mathbf R_I={}^G\bar{\mathbf R}_I\,\mathrm{Exp}(\delta\boldsymbol\theta)$。这是**右扰动**写法吗？注意：$\mathbf R=\bar{\mathbf R}\,\mathrm{Exp}(\delta\boldsymbol\theta)$ 把扰动 $\mathrm{Exp}(\delta\boldsymbol\theta)$ 乘在右边、且 $\delta\boldsymbol\theta$ 表达在 **body/local 系**，正是本书"右扰动为主"约定（$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$）。**因此 FAST-LIO 的姿态误差与本书右扰动一致**（$\delta\boldsymbol\theta\leftrightarrow\delta\boldsymbol\phi$），无须翻转扰动方向，仅符号字母由 $\delta\boldsymbol\theta$ 改为 $\delta\boldsymbol\phi$。（区别于 Barfoot 的左扰动 $\mathbf R\to\mathrm{Exp}(\delta\boldsymbol\phi)\mathbf R$。）
- **$\boxplus/\boxminus$ 流形封装**：FAST-LIO 用 Hertzberg 的 $\boxplus/\boxminus$ 算子统一处理 $\mathrm{SO}(3)\times\mathbb R^{15}$ 复合流形，本书在李群章已建立同款工具，记号可直接对接。
- **状态排序**：FAST-LIO 状态 $\mathbf x=[{}^G\mathbf R_I^T\;{}^G\mathbf p_I^T\;{}^G\mathbf v_I^T\;\mathbf b_\omega^T\;\mathbf b_a^T\;{}^G\mathbf g^T]^T$（**旋转在前**），与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）相反；移植时须重排状态向量与对应 Jacobian 行列块。
- **协方差字母**：FAST-LIO 用 $\mathbf P$（误差协方差）、$\mathbf Q$（过程噪声）、$\mathbf R$（测量噪声）——其中 $\mathbf R$ 与本书 $\mathbf R\in\mathrm{SO}(3)$ 同字母不同义（这里是测量噪声协方差），综合时须显式区分。

### 0.5 Forster 预积分记号（源补充）

| 记号 | 含义 |
|---|---|
| $\tilde\cdot$（tilde） | 带噪测量（如 $\tilde{\boldsymbol\omega}_k$ 是含偏置含噪的陀螺测量）。 |
| $\mathrm{Exp}(\cdot),\mathrm{Log}(\cdot)$ | $\mathfrak{so}(3)\leftrightarrow\mathrm{SO}(3)$ 指对数（Forster 用大写，区别于 $\exp/\log$ 的矩阵指对数）。 |
| $\mathbf J_r$ | $\mathrm{SO}(3)$ 右雅可比；$\mathbf J^k_r=\mathbf J_r((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)$。 |
| $\Delta\tilde{\mathbf R}_{ij},\Delta\tilde{\mathbf v}_{ij},\Delta\tilde{\mathbf p}_{ij}$ | 预积分**测量值**（带 tilde）。 |
| $\delta\boldsymbol\phi_{ij},\delta\mathbf v_{ij},\delta\mathbf p_{ij}$ | 对应噪声；$\boldsymbol\eta^\Delta_{ij}=[\delta\boldsymbol\phi_{ij}^T,\delta\mathbf v_{ij}^T,\delta\mathbf p_{ij}^T]^T\sim\mathcal N(\mathbf 0_{9\times1},\boldsymbol\Sigma_{ij})$。 |
| $\mathbf r_{\Delta\mathbf R_{ij}},\mathbf r_{\Delta\mathbf v_{ij}},\mathbf r_{\Delta\mathbf p_{ij}}$ | 三类残差。 |
| $\bar{\mathbf b}^a_i,\bar{\mathbf b}^g_i$ | 预积分时刻的偏置估计；$\delta\mathbf b$ 优化中的小偏置更新。 |

Forster 全程使用**右扰动 / 右雅可比**，与本书一致——这是本书"右扰动为主、对接因子图"决策的直接来源。

---

# 第一部分：LOAM（Lidar Odometry and Mapping in Real-time, RSS 2014）

> 这是激光 SLAM 特征法的开山之作。核心思想：把"同时估计运动 + 校正运动畸变"的耦合难题，**分解为两个并行算法**——高频低保真的 lidar **odometry**（估速度、去畸变）+ 低频高保真的 lidar **mapping**（精配准建图）。

## §LOAM.I-II 引入与动机（源 §I, §II）

**问题与动机**：3D 建图常用 lidar，因为 lidar 提供高频 range 测量、误差与距离无关。若 lidar 仅旋转激光束，点云配准很简单；但若 lidar 本身在 6-DOF 运动，精确建图需要知道连续激光测距期间的 lidar 位姿。常见解法：(1) 独立位置估计（GPS/INS）把激光点配准到固定系；(2) 用里程计（轮编/视觉里程计）配准。里程计对小增量运动积分必然漂移，故大量工作致力于减漂（如回环）。

**本文设定**：用 2 轴 lidar 在 6-DOF 运动建低漂移里程计地图。lidar 优点：对环境光照与光学纹理不敏感。本文目标是把里程计估计的漂移最小化，因此**不含回环**（当前版本）。

**关键性能来源（论文反复强调）**：把 SLAM 这个需同时优化大量变量的复杂问题，分解为两个算法——
- 一个算法以**高频低保真**做 odometry，估计 lidar 速度；
- 另一个算法以**低一个数量级的频率**做点云精配准与配准。
- 若有 IMU（非必需），可提供运动先验以处理高频运动。
- 两算法都提取位于**尖锐边缘**和**平面**上的特征点，分别匹配到边缘线段与平面片。odometry 中通过保证快速计算来找对应；mapping 中通过检查局部点簇的几何分布（关联特征值与特征向量）来确定对应。

**分解的合理性**（源 §I 末）：先解一个更易的在线运动估计问题；之后把 mapping 当作批优化（类似 ICP）以产生高精运动估计与地图。并行结构保证实时；由于运动估计在更高频进行，mapping 有充裕时间确保精度——低频运行时 mapping 能纳入大量特征点、用足够多迭代收敛。

## §LOAM.III 记号与任务描述（源 §III）

**假设**：(1) lidar 已预标定；(2) lidar 的角速度与线速度随时间**平滑连续、无突变**（第二个假设将在 §VII-B 用 IMU 放松）。

**坐标系定义**（见 §0.1）：
- **Lidar 系 $\{L\}$**：原点在 lidar 几何中心，x 向左、y 向上、z 向前。点 $i\in\mathcal P_k$ 在 $\{L_k\}$ 中的坐标记 $\mathbf X^L_{(k,i)}$。
- **World 系 $\{W\}$**：与初始 $\{L\}$ 重合。点 $i$ 在 $\{W_k\}$ 中坐标记 $\mathbf X^W_{(k,i)}$。

**问题（Problem）**：给定 lidar 点云序列 $\mathcal P_k,\ k\in\mathbb Z^+$，计算每个 sweep $k$ 期间 lidar 的自运动（ego-motion），并用 $\mathcal P_k$ 为遍历环境建图。

## §LOAM.IV 系统概览（源 §IV）

### A. Lidar 硬件
自制 3D lidar：Hokuyo UTM-30LX 激光扫描仪（视场 180°、0.25° 分辨率、40 lines/sec），连一电机以 180°/s 在 $-90^\circ$ 到 $90^\circ$ 间转动（水平方向为零）。一个 sweep = 从 $-90^\circ$ 到 $90^\circ$（或反向）的旋转，历时 1 s。连续旋转 lidar 的 sweep 即半球旋转。编码器以 0.25° 分辨率测电机转角，据此把激光点投影到 $\{L\}$。

### B. 软件系统概览（源 §IV-B，Fig.3）
- $\hat{\mathcal P}$ = 单次 laser scan 收到的点，在 $\{L\}$ 中配准；sweep $k$ 期间的组合点云形成 $\mathcal P_k$。
- $\mathcal P_k$ 送入两个算法。**Lidar odometry** 取点云、计算两连续 sweep 间 lidar 运动，用以校正 $\mathcal P_k$ 的畸变，约 **10 Hz**。
- 输出送 **lidar mapping**，把去畸变点云匹配配准到地图，**1 Hz**。
- 两算法发布的位姿变换被**整合**生成约 10 Hz 的变换输出（lidar 相对 map 的位姿）。

## §LOAM.V Lidar Odometry（源 §V）

### A. 特征点提取（源 §V-A）

lidar 自然产生不均匀点云。一条 scan 内分辨率 0.25°，但因扫描仪以 180°/s 转、40 Hz 出 scan，**垂直于 scan 平面的方向分辨率为 $180^\circ/40=4.5^\circ$**。考虑此，特征点**仅用单条 scan 内信息**（共面几何关系）提取。

选取位于尖锐边缘和平面片上的特征点。设 $i\in\mathcal P_k$，$\mathcal S$ 为激光扫描仪在**同一 scan** 内返回的 $i$ 的连续点集合（CW 或 CCW 顺序，$i$ 两侧各半、两点间 0.25° 间隔）。定义局部曲面**光滑度**：

$$c=\frac{1}{|\mathcal S|\cdot\|\mathbf X^L_{(k,i)}\|}\left\|\sum_{j\in\mathcal S,\,j\neq i}\big(\mathbf X^L_{(k,i)}-\mathbf X^L_{(k,j)}\big)\right\|.\tag{LOAM-1}$$

scan 内的点按 $c$ 值排序，选**最大 $c$** 者为**边缘点（edge points）**、**最小 $c$** 者为**平面点（planar points）**。

**为均匀分布特征点**：把一条 scan 分成**四个相同子区域**。每个子区域最多提供 **2 个边缘点、4 个平面点**。点 $i$ 被选为边缘/平面点的条件：其 $c$ 值大于/小于一个阈值，且已选点数未超上限。

**避免坏特征**（源 §V-A）：
1. 避免其周围点已被选中的点，或位于**与激光束大致平行**的局部平面上的点（Fig.4(a) 中的点 B）——这些通常不可靠。
2. 避免位于**遮挡区域边界**上的点（Fig.4(b)）。点 A 是边缘点，因其连接的曲面（虚线段）被另一物体挡住；但若 lidar 换视角，遮挡区可能变可见。为避免选中这些点，再次找点集 $\mathcal S$，**点 $i$ 可被选中当且仅当**：$\mathcal S$ 不构成与激光束大致平行的曲面片，且 $\mathcal S$ 中没有点在激光束方向上因间隙与 $i$ 断开、同时比 $i$ 更靠近 lidar（如 Fig.4(b) 的点 B）。

**总结（特征选取规则）**：边缘点从最大 $c$ 开始选、平面点从最小 $c$ 开始选；若一点被选中，则：
- 所选边缘点或平面点数**不超过子区域上限**，且
- 其**周围点尚未被选中**，且
- 它**不在与激光束大致平行的曲面片上**，也不在遮挡区域边界上。

### B. 寻找特征点对应（源 §V-B）

设 $t_k$ 为 sweep $k$ 起始时刻。每个 sweep 末，把该 sweep 感知的 $\mathcal P_k$ **重投影到时间戳 $t_{k+1}$**（见 Fig.6），记重投影点云为 $\bar{\mathcal P}_k$。下一 sweep $k+1$ 中，$\bar{\mathcal P}_k$ 与新收点云 $\mathcal P_{k+1}$ 一起估 lidar 运动。

从 $\mathcal P_{k+1}$ 用上节方法提边缘点集 $\mathcal E_{k+1}$、平面点集 $\mathcal H_{k+1}$。从 $\bar{\mathcal P}_k$ 找边缘线作为 $\mathcal E_{k+1}$ 的对应、平面片作为 $\mathcal H_{k+1}$ 的对应。

**递归估计**：sweep $k+1$ 开始时 $\mathcal P_{k+1}$ 为空集，随 sweep 进行而增长。odometry 递归估计 sweep 内 6-DOF 运动并逐渐纳入更多点。每次迭代，用当前估计变换把 $\mathcal E_{k+1},\mathcal H_{k+1}$ 重投影到 sweep 起点，得 $\tilde{\mathcal E}_{k+1},\tilde{\mathcal H}_{k+1}$。对其中每点，到 $\bar{\mathcal P}_k$ 找最近邻（$\bar{\mathcal P}_k$ 存于 3D KD-tree）。

**边缘线对应**（Fig.7(a)）：设 $i\in\tilde{\mathcal E}_{k+1}$。边缘线由两点表示。设 $j$ 为 $i$ 在 $\bar{\mathcal P}_k$ 中最近邻，$l$ 为 $i$ 在 $j$ 所在 scan 的**两条相邻 scan** 中的最近邻。$(j,l)$ 构成 $i$ 的对应。为验证 $j,l$ 是边缘点，基于 (LOAM-1) 检查局部光滑度。**特别要求 $j$ 与 $l$ 来自不同 scan**（单条 scan 不能含同一边缘线的多于一点）。唯一例外：边缘线在 scan 平面上——但此时边缘线退化为 scan 平面上的直线，其上的特征点本不该被提取。

**平面片对应**（Fig.7(b)）：设 $i\in\tilde{\mathcal H}_{k+1}$。平面片由三点表示。找 $i$ 在 $\bar{\mathcal P}_k$ 的最近邻 $j$；再找另两点 $l,m$：一个在 $j$ 的**同一 scan**、另一个在 $j$ 所在 scan 的**两条相邻 scan**——保证三点不共线。基于 (LOAM-1) 验证 $j,l,m$ 都是平面点。

**距离公式**。对边缘点 $i\in\tilde{\mathcal E}_{k+1}$，若 $(j,l)$ 为对应边缘线（$j,l\in\bar{\mathcal P}_k$），**点到线距离**：

$$d_{\mathcal E}=\frac{\left\|\big(\tilde{\mathbf X}^L_{(k+1,i)}-\bar{\mathbf X}^L_{(k,j)}\big)\times\big(\tilde{\mathbf X}^L_{(k+1,i)}-\bar{\mathbf X}^L_{(k,l)}\big)\right\|}{\left\|\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,l)}\right\|},\tag{LOAM-2}$$

其中 $\tilde{\mathbf X}^L_{(k+1,i)},\bar{\mathbf X}^L_{(k,j)},\bar{\mathbf X}^L_{(k,l)}$ 是点 $i,j,l$ 在 $\{L\}$ 中坐标。
（几何意义：分子是以 $i$ 与 $j,l$ 为顶点构成的平行四边形面积 = 底 $\|j-l\|$ × 高，故商即点 $i$ 到过 $j,l$ 直线的垂距。）

对平面点 $i\in\tilde{\mathcal H}_{k+1}$，若 $(j,l,m)$ 为对应平面片（$j,l,m\in\bar{\mathcal P}_k$），**点到面距离**：

$$d_{\mathcal H}=\frac{\left|\big(\tilde{\mathbf X}^L_{(k+1,i)}-\bar{\mathbf X}^L_{(k,j)}\big)\cdot\Big(\big(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,l)}\big)\times\big(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,m)}\big)\Big)\right|}{\left\|\big(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,l)}\big)\times\big(\bar{\mathbf X}^L_{(k,j)}-\bar{\mathbf X}^L_{(k,m)}\big)\right\|},\tag{LOAM-3}$$

其中 $\bar{\mathbf X}^L_{(k,m)}$ 是点 $m$ 在 $\{L\}$ 中坐标。
（几何意义：分母是 $\triangle jlm$ 张成平面的法向量模 = 平行四边形面积；分子是 $(i-j)$ 在该法向上的投影 × 面积；商即点 $i$ 到过 $j,l,m$ 平面的垂距。）

### C. 运动估计（源 §V-C）

lidar 运动建模为 sweep 内**常角速度、常线速度**，允许对不同时刻收到的点**线性插值**位姿变换。设 $t$ 为当前时间戳，$t_{k+1}$ 为 sweep $k+1$ 起始时刻，$\mathbf T^L_{k+1}$ 为 $[t_{k+1},t]$ 间 lidar 位姿变换，含 6-DOF 刚体运动：

$$\mathbf T^L_{k+1}=[t_x,t_y,t_z,\theta_x,\theta_y,\theta_z]^T,$$

其中 $t_x,t_y,t_z$ 是沿 $\{L\}$ 三轴平移，$\theta_x,\theta_y,\theta_z$ 是按右手定则的旋转角。

给定点 $i\in\mathcal P_{k+1}$，时间戳 $t_i$，令 $\mathbf T^L_{(k+1,i)}$ 为 $[t_{k+1},t_i]$ 间位姿变换，由 $\mathbf T^L_{k+1}$ **线性插值**：

$$\mathbf T^L_{(k+1,i)}=\frac{t_i-t_{k+1}}{t-t_{k+1}}\mathbf T^L_{k+1}.\tag{LOAM-4}$$

为解 lidar 运动，需建立 $\mathcal E_{k+1}$ 与 $\tilde{\mathcal E}_{k+1}$（或 $\mathcal H_{k+1}$ 与 $\tilde{\mathcal H}_{k+1}$）的几何关系。用 (LOAM-4)：

$$\mathbf X^L_{(k+1,i)}=\mathbf R\,\tilde{\mathbf X}^L_{(k+1,i)}+\mathbf T^L_{(k+1,i)}(1:3),\tag{LOAM-5}$$

其中 $\mathbf X^L_{(k+1,i)}$ 是 $\mathcal E_{k+1}$ 或 $\mathcal H_{k+1}$ 中点 $i$ 坐标，$\tilde{\mathbf X}^L_{(k+1,i)}$ 是 $\tilde{\mathcal E}_{k+1}$ 或 $\tilde{\mathcal H}_{k+1}$ 中对应点，$\mathbf T^L_{(k+1,i)}(a:b)$ 是其第 $a$ 到 $b$ 个元素，$\mathbf R$ 是由 Rodrigues 公式定义的旋转矩阵：

$$\mathbf R=e^{\hat{\boldsymbol\omega}\theta}=\mathbf I+\hat{\boldsymbol\omega}\sin\theta+\hat{\boldsymbol\omega}^2(1-\cos\theta).\tag{LOAM-6}$$

其中 $\theta$ 为旋转幅度

$$\theta=\big\|\mathbf T^L_{(k+1,i)}(4:6)\big\|,\tag{LOAM-7}$$

$\boldsymbol\omega$ 为旋转方向单位向量

$$\boldsymbol\omega=\mathbf T^L_{(k+1,i)}(4:6)\,\big/\,\big\|\mathbf T^L_{(k+1,i)}(4:6)\big\|,\tag{LOAM-8}$$

$\hat{\boldsymbol\omega}$ 为 $\boldsymbol\omega$ 的反对称矩阵。

**组合几何关系**。结合 (LOAM-2) 与 (LOAM-4)–(LOAM-8)，得边缘点与对应边缘线的几何关系：

$$f_{\mathcal E}\big(\mathbf X^L_{(k+1,i)},\mathbf T^L_{k+1}\big)=d_{\mathcal E},\quad i\in\mathcal E_{k+1}.\tag{LOAM-9}$$

类似地，结合 (LOAM-3) 与 (LOAM-4)–(LOAM-8)，得平面点与对应平面片的几何关系：

$$f_{\mathcal H}\big(\mathbf X^L_{(k+1,i)},\mathbf T^L_{k+1}\big)=d_{\mathcal H},\quad i\in\mathcal H_{k+1}.\tag{LOAM-10}$$

**Levenberg–Marquardt 求解**。把 (LOAM-9)、(LOAM-10) 对 $\mathcal E_{k+1},\mathcal H_{k+1}$ 中每个特征点堆叠，得非线性函数

$$\mathbf f(\mathbf T^L_{k+1})=\mathbf d,\tag{LOAM-11}$$

$\mathbf f$ 每行对应一个特征点，$\mathbf d$ 含对应距离。计算 Jacobian $\mathbf J=\partial\mathbf f/\partial\mathbf T^L_{k+1}$，则 (LOAM-11) 通过非线性迭代使 $\mathbf d\to\mathbf 0$ 求解：

$$\boxed{\ \mathbf T^L_{k+1}\leftarrow\mathbf T^L_{k+1}-\big(\mathbf J^T\mathbf J+\lambda\,\mathrm{diag}(\mathbf J^T\mathbf J)\big)^{-1}\mathbf J^T\mathbf d.\ }\tag{LOAM-12}$$

$\lambda$ 由 Levenberg–Marquardt 方法确定。

### D. Lidar Odometry 算法（源 Algorithm 1，逐行）

> **Algorithm 1: Lidar Odometry**
> **input**: $\bar{\mathcal P}_k$，$\mathcal P_{k+1}$，上一轮递归的 $\mathbf T^L_{k+1}$
> **output**: $\bar{\mathcal P}_{k+1}$，新计算的 $\mathbf T^L_{k+1}$
> 1. **begin**
> 2. &nbsp;&nbsp;**if** at the beginning of a sweep **then**
> 3. &nbsp;&nbsp;&nbsp;&nbsp;$\mathbf T^L_{k+1}\leftarrow\mathbf 0$;
> 4. &nbsp;&nbsp;**end**
> 5. &nbsp;&nbsp;Detect edge points and planar points in $\mathcal P_{k+1}$, put the points in $\mathcal E_{k+1}$ and $\mathcal H_{k+1}$, respectively;
> 6. &nbsp;&nbsp;**for** a number of iterations **do**
> 7. &nbsp;&nbsp;&nbsp;&nbsp;**for** each edge point in $\mathcal E_{k+1}$ **do**
> 8. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Find an edge line as the correspondence, then compute point to line distance based on (LOAM-9) and stack the equation to (LOAM-11);
> 9. &nbsp;&nbsp;&nbsp;&nbsp;**end**
> 10. &nbsp;&nbsp;&nbsp;**for** each planar point in $\mathcal H_{k+1}$ **do**
> 11. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Find a planar patch as the correspondence, then compute point to plane distance based on (LOAM-10) and stack the equation to (LOAM-11);
> 12. &nbsp;&nbsp;&nbsp;**end**
> 13. &nbsp;&nbsp;&nbsp;Compute a bisquare weight for each row of (LOAM-11);
> 14. &nbsp;&nbsp;&nbsp;Update $\mathbf T^L_{k+1}$ for a nonlinear iteration based on (LOAM-12);
> 15. &nbsp;&nbsp;&nbsp;**if** the nonlinear optimization converges **then** Break; **end**
> 16. &nbsp;&nbsp;**end**
> 17. &nbsp;&nbsp;**if** at the end of a sweep **then**
> 18. &nbsp;&nbsp;&nbsp;&nbsp;Reproject each point in $\mathcal P_{k+1}$ to $t_{k+2}$ and form $\bar{\mathcal P}_{k+1}$;
> 19. &nbsp;&nbsp;&nbsp;&nbsp;Return $\mathbf T^L_{k+1}$ and $\bar{\mathcal P}_{k+1}$;
> 20. &nbsp;&nbsp;**else** Return $\mathbf T^L_{k+1}$; **end**
> 21. **end**

**算法说明**（源 §V-D）：输入上一 sweep 点云 $\bar{\mathcal P}_k$、当前 sweep 增长点云 $\mathcal P_{k+1}$、上轮位姿 $\mathbf T^L_{k+1}$。新 sweep 开始时 $\mathbf T^L_{k+1}$ 置零（line 2-4）。提取特征点构建 $\mathcal E_{k+1},\mathcal H_{k+1}$（line 5）。对每特征点找 $\bar{\mathcal P}_k$ 中对应（line 7-12）。**运动估计适配鲁棒拟合 [27]**：line 13 给每特征点赋 **bisquare 权重**——距离对应越远权越小，距离大于阈值者视为外点、赋零权。line 14 更新位姿一次迭代。收敛或达最大迭代则终止。到 sweep 末则把 $\mathcal P_{k+1}$ 用估计运动重投影到 $t_{k+2}$；否则只返回 $\mathbf T^L_{k+1}$ 供下轮递归。

## §LOAM.VI Lidar Mapping（源 §VI）

mapping 比 odometry 低频，每 sweep 只调用一次。sweep $k+1$ 末，odometry 生成去畸变点云 $\bar{\mathcal P}_{k+1}$ 及位姿变换 $\mathbf T^L_{k+1}$（含 $[t_{k+1},t_{k+2}]$ 的 lidar 运动）。mapping 把 $\bar{\mathcal P}_{k+1}$ 配准到 world 系 $\{W\}$。

定义 $\mathcal Q_k$ 为地图上累积到 sweep $k$ 的点云，$\mathbf T^W_k$ 为 sweep $k$ 末（$t_{k+1}$）lidar 在地图上的位姿。用 odometry 输出，mapping 把 $\mathbf T^W_k$ 沿 sweep 从 $t_{k+1}$ 延伸到 $t_{k+2}$ 得 $\mathbf T^W_{k+1}$，并把 $\bar{\mathcal P}_{k+1}$ 投影到 $\{W\}$ 得 $\bar{\mathcal Q}_{k+1}$。再通过优化位姿 $\mathbf T^W_{k+1}$ 把 $\bar{\mathcal Q}_{k+1}$ 与 $\mathcal Q_k$ 匹配。

**与 odometry 的关键差异**（源 §VI）：
- 特征点提取方式同 §V-A，但用 **10 倍**于 odometry 的特征点。
- 为找对应，把地图点云 $\mathcal Q_k$ 存于 **10 m 立方区域**；与 $\bar{\mathcal Q}_{k+1}$ 相交的立方体内的点取出存入 3D KD-tree。
- 在特征点周围一定区域找 $\mathcal Q_k$ 中的点，记周围点集 $\mathcal S'$。边缘点只保留 $\mathcal S'$ 中在边缘线上的点；平面点只保留在平面片上的点。
- 计算 $\mathcal S'$ 的**协方差矩阵 $\mathbf M$** 及其特征值 $\mathbf V$、特征向量 $\mathbf E$：
  - 若 $\mathcal S'$ 分布在**边缘线**上，$\mathbf V$ 含**一个**显著大于其余两个的特征值，$\mathbf E$ 中关联最大特征值的特征向量表示边缘线方向；
  - 若 $\mathcal S'$ 分布在**平面片**上，$\mathbf V$ 含**两个**大特征值、第三个显著小，$\mathbf E$ 中关联最小特征值的特征向量表示平面片法向。
  - 边缘线/平面片位置由通过 $\mathcal S'$ 几何中心确定。
- 在边缘线上选两点、平面片上选三点，使距离能用与 (LOAM-2)、(LOAM-3) 相同的公式计算。每特征点导出 (LOAM-9) 或 (LOAM-10) 式，不同处在于 $\bar{\mathcal Q}_{k+1}$ 中所有点共享同一时间戳 $t_{k+2}$。
- 非线性优化再次由鲁棒拟合 [27] + Levenberg–Marquardt 求解，$\bar{\mathcal Q}_{k+1}$ 配准到地图。地图点云用 **5 cm 立方体素栅格滤波**降采样。

**位姿变换整合**（源 Fig.9）：mapping 输出 $\mathbf T^W_k$（每 sweep 一次）+ odometry 输出 $\mathbf T^L_{k+1}$（约 10 Hz）；lidar 相对 map 的位姿是两变换的组合，频率同 odometry。

## §LOAM.VII 实验（源 §VII）

**计算平台**：2.5 GHz 四核 + 6 GiB 内存笔记本，ROS/Linux。odometry 与 mapping 各占一核（共两核）。

### A. 室内外测试
室内（手推车载 lidar）：长走廊、大堂。室外（地面车前装 lidar）：植被路、果园。所有测试速度 **0.5 m/s**。局部精度用第二组静止采集的点云、point-to-plane ICP 匹配评估。室内匹配误差小于室外（人造环境特征匹配更精确）。

**TABLE I — 运动估计漂移相对误差**（源）：

| 环境 | Test 1 距离 | Test 1 误差 | Test 2 距离 | Test 2 误差 |
|---|---|---|---|---|
| Corridor（走廊，含闭环） | 58 m | 0.9% | 46 m | 1.1% |
| Orchard（果园） | 52 m | 2.3% | 67 m | 2.8% |

总体：室内相对精度 ~1%、室外 ~2.5%。

### B. IMU 辅助（源 §VII-B）
装 Xsens MTi-10 IMU 应对快速速度变化。点云送入本方法前预处理两步：(1) 用 IMU 朝向把一个 scan 内的点云旋转对齐到该 sweep lidar 初始朝向；(2) 用加速度测量，把运动畸变部分去除（视作 sweep 内匀速）。IMU 朝向由陀螺角速度 + 加表读数在 Kalman 滤波中积分得。

**TABLE II — 用/不用 IMU 的运动估计误差**（源）：人持 lidar 以 0.5 m/s 行走、上下幅度约 0.5 m，卷尺人工测真值。

| 环境 | 距离 | 仅 IMU 朝向（误差） | 仅本方法 Ours | Ours+IMU |
|---|---|---|---|---|
| Corridor | 32 m | 16.7% | 2.1% | 0.9% |
| Lobby | 27 m | 11.7% | 1.7% | 1.3% |
| Vegetated road | 43 m | 13.7% | 4.4% | 2.6% |
| Orchard | 51 m | 11.4% | 3.7% | 2.1% |

结论：IMU+本方法精度最高；仅用 IMU 朝向最差。IMU 有效抵消非线性运动，本方法处理线性运动。

### C. KITTI 数据集（源 §VII-C）
Velodyne 360° lidar，10 Hz 记录。覆盖 urban / country / highway 三类环境，总行驶 39.2 km。本方法在 KITTI 里程计 benchmark **排名 #1**（不论传感模态，含 SOTA 立体视觉里程计）。平均位置误差 **0.88%**（按 100 m, 200 m, …, 800 m 轨迹段、3D 坐标评估）。

## §LOAM.VIII 结论与未来工作（源 §VIII）
方法把"运动恢复 + 运动畸变校正"难题分解为两并行算法：odometry 粗处理高频估速度，mapping 精处理低频建图。未来：(1) 开发回环方法修正漂移；(2) 用 Kalman 滤波把输出与 IMU 融合进一步减漂。**（注：这两个"未来工作"正是 LeGO-LOAM / LIO-SAM / FAST-LIO 的切入点。）**

---

# 第二部分：LeGO-LOAM（Lightweight & Ground-Optimized LOAM, IROS 2018）

> 面向**地面车（UGV）**与**嵌入式系统**的轻量化 LOAM。两大改进：(1) **点云分割**（先分地面、再图像分割聚类、滤掉噪声小簇）以提速并提纯特征；(2) **两步 L-M 优化**（地面平面特征求 $[t_z,\theta_{roll},\theta_{pitch}]$、边缘特征求 $[t_x,t_y,\theta_{yaw}]$）。此外集成**回环 + 位姿图 SLAM**。

## §LeGO.I 引入与动机（源 §I）

**问题**：UGV 常无悬挂、算力弱；变地形上运动不平滑导致数据畸变；两连续 scan 间大运动、重叠少使对应难找；3D lidar 海量点对实时处理是挑战。

**LOAM 的不足（在 UGV 上）**：(1) 需对稠密 3D 点云每点算粗糙度，轻量嵌入式上特征提取频率跟不上传感器；(2) lidar 安装位置常靠近地面，地面噪声（如草地的 range 返回）产生高粗糙度→提出不可靠边缘特征；树叶同理。这些特征对 scan-matching 不可靠（同一草叶/树叶不会在两连续 scan 中再现），导致配准不准、大漂移。

**LeGO-LOAM 的两大设计**：
- **轻量**：点云分割丢弃地面分离后可能代表不可靠特征的点；可在嵌入式实时运行。
- **地面优化**：**两步优化**——第一步用**地面平面特征**求 $[t_z,\theta_{roll},\theta_{pitch}]$；第二步用**分割点云的边缘特征**求 $[t_x,t_y,\theta_{yaw}]$。并集成回环修正漂移。

## §LeGO.II 系统硬件（源 §II）
Velodyne VLP-16（量程 100 m、±3 cm、垂直 FOV 30°(±15°)、水平 360°、16 通道、垂直角分辨率 2°、水平 0.1°–0.4°；选 10 Hz 扫描率→水平分辨率 0.2°）与 HDL-64E（KITTI，64 通道、垂直 FOV 26.9°）。UGV：Clearpath Jackal（270 Wh 锂电、最大 2.0 m/s、载荷 20 kg、低成本 IMU CH Robotics UM6）。计算平台：Nvidia Jetson TX2（ARM Cortex-A57）+ 2.5 GHz i7-4710MQ 笔记本。

## §LeGO.III 方法（源 §III）

### A. 系统概览（源 §III-A）
五个模块：(1) **Segmentation** 把单 scan 点云投影到 range image 分割；(2) **Feature extraction**；(3) **Lidar odometry** 用特征找连续 scan 间变换；(4) **Lidar mapping** 把特征配准到全局地图；(5) **Transform integration** 融合 odometry 与 mapping 的位姿。

### B. 分割（Segmentation，源 §III-B）
设 $\mathcal P_t=\{p_1,\dots,p_n\}$ 为时刻 $t$ 点云，先投影到 **range image**（VLP-16 分辨率 **1800 × 16**）。每有效点 $p_i$ 对应唯一像素，关联 range 值 $r_i$（$p_i$ 到传感器欧氏距离）。

**地面提取**（不假设地面平坦，因坡地常见）：对 range image 做**逐列评估**（可视作地面平面估计 [22]），将可能代表地面的点标为**地面点**，不参与后续分割。

**图像分割聚类** [23]（即 Bogoslavskyi-Stachniss，见 §补充）：对 range image 分割成多个簇，同簇点赋唯一标签（地面点是特殊簇）。**省略点数少于 30 的簇**（如树叶——同一叶不会在两连续 scan 再现，构成不可靠琐碎特征）。

分割后只保留可能代表大物体（如树干）的点 + 地面点。每点记**三个属性**：(1) 地面点/分割点标签；(2) range image 中列、行索引；(3) range 值。

### C. 特征提取（源 §III-C）
类似 LOAM [20]，但从**地面点和分割点**而非原始点云提取。设 $\mathcal S$ 为 range image 同一行中 $p_i$ 的连续点集合，两侧各半，$|\mathcal S|=10$。用分割时算的 range 值评估 $p_i$ 的粗糙度：

$$c=\frac{1}{|\mathcal S|\cdot\|r_i\|}\left\|\sum_{j\in\mathcal S,\,j\neq i}(r_j-r_i)\right\|.\tag{LeGO-1}$$

（注意：LeGO-LOAM 用 **range 值 $r$** 而非 LOAM 的 3D 坐标 $\mathbf X$ 算粗糙度，是基于 range image 的简化。）

**均匀提取**：range image 水平分成若干等宽子图（本文 **6 个子图**，每子图 **300 × 16**）。每行按 $c$ 排序，用阈值 $c_{th}$ 区分：$c>c_{th}$ 为边缘特征、$c<c_{th}$ 为平面特征。

**两级特征集**（关键！）：
- **odometry 用（数量多）**：每子图每行选 $n_{\mathbb F_e}$ 个最大 $c$ 的**非地面**边缘特征、$n_{\mathbb F_p}$ 个最小 $c$ 的（地面或分割点）平面特征。全体记 $\mathbb F_e,\mathbb F_p$。
- **mapping 用（数量少、更精）**：每子图每行选 $n_{F_e}$ 个最大 $c$ 的**非地面**边缘特征、$n_{F_p}$ 个最小 $c$ 的**必为地面点**的平面特征。全体记 $F_e,F_p$，满足 $F_e\subset\mathbb F_e$、$F_p\subset\mathbb F_p$。
- 参数：$n_{F_e},n_{F_p},n_{\mathbb F_e},n_{\mathbb F_p}$ = **2, 4, 40, 80**（即 $F_e$=2、$F_p$=4、$\mathbb F_e$=40、$\mathbb F_p$=80）。

### D. Lidar Odometry（源 §III-D）
估计两连续 scan 间运动，做点到边、点到面 scan-matching，从上一 scan 的 $\mathbb F^{t-1}_e,\mathbb F^{t-1}_p$ 找 $\mathbb F^t_e,\mathbb F^t_p$ 的对应（细节同 LOAM [20]）。两点改进：

**1) 标签匹配（Label Matching）**：每特征经分割后带标签，只在**同标签**间找对应。平面特征 $\mathbb F^t_p$ 只用 $\mathbb F^{t-1}_p$ 中标为**地面点**者找平面片；边缘特征 $\mathbb F^t_e$ 的对应边缘线在 $\mathbb F^{t-1}_e$ 的**分割簇**中找。提高匹配精度（同物体的对应更易在两 scan 间找到），并缩小候选。

**2) 两步 L-M 优化（Two-step L-M Optimization）**：LOAM [20] 把边缘/平面距离编进单一综合距离向量、用 L-M 一次求最小距离变换。LeGO-LOAM 改为两步：
- **第一步**：用 $\mathbb F^t_p$ 的平面特征与其在 $\mathbb F^{t-1}_p$ 的对应，估 $[t_z,\theta_{roll},\theta_{pitch}]$；
- **第二步**：用 $\mathbb F^t_e$ 的边缘特征与其在 $\mathbb F^{t-1}_e$ 的对应，**以 $[t_z,\theta_{roll},\theta_{pitch}]$ 为约束**，估剩余 $[t_x,t_y,\theta_{yaw}]$。
- 注：$[t_x,t_y,\theta_{yaw}]$ 第一步也能得，但精度低、不用于第二步。最终 6D 变换由两步融合。**两步法精度相当、计算时间减约 35%**（Table III）。

### E. Lidar Mapping（源 §III-E）
把 $\{F^t_e,F^t_p\}$ 匹配到周围点云地图 $Q^{t-1}$ 精修位姿（低频），再用 L-M 求最终变换（细节同 LOAM [20]）。

**关键差异：地图存储**。不存单一点云地图，而是**保存每个特征集 $\{F^t_e,F^t_p\}$**。设 $\mathbb M^{t-1}=\{\{F^1_e,F^1_p\},\dots,\{F^{t-1}_e,F^{t-1}_p\}\}$ 为保存所有历史特征集的集合，每个特征集关联其 scan 拍摄时的传感器位姿。$Q^{t-1}$ 可从 $\mathbb M^{t-1}$ 两种方式得：

- **方式一（视场选取）**：选在传感器视场内的特征集（简化：选传感器位姿在当前位置 100 m 内者），变换并融合成单一周围地图 $Q^{t-1}$（同 LOAM [20]）。
- **方式二（位姿图 SLAM）**：每特征集的传感器位姿建模为**位姿图节点**，$\{F^t_e,F^t_p\}$ 视作该节点的传感器测量。因 mapping 漂移很低，可假设短期无漂移，故 $Q^{t-1}$ 由近期一组特征集构成：$Q^{t-1}=\{\{F^{t-k}_e,F^{t-k}_p\},\dots,\{F^{t-1}_e,F^{t-1}_p\}\}$，$k$ 定义 $Q^{t-1}$ 大小。新节点与所选节点间的**空间约束**用 L-M 优化后的变换添加。**进一步用回环检测消漂**：若当前特征集与某历史特征集用 **ICP** 匹配成功，则添加新约束；把位姿图送优化系统（如 iSAM2 [24]）更新传感器位姿。（注：仅 §IV-D 实验用此技术建周围地图。）

## §LeGO.IV 实验（源 §IV）

### A. 小型 UGV 测试（源 §IV-A）
室外植被环境。LeGO-LOAM 分割后特征数大减，树叶多被丢弃；草地噪声大、粗糙度高，若不分割会误提边缘特征。

### C. 基准结果（源 §IV-C）

**TABLE II — 特征提取后每 scan 平均特征数**（10 次均值，源；列为 LOAM / LeGO-LOAM）：

| 场景 | 边缘 $\mathbb F_e$ LOAM | $\mathbb F_e$ LeGO | 平面 $\mathbb F_p$ LOAM | $\mathbb F_p$ LeGO | 边缘 $F_e$ LOAM | $F_e$ LeGO | 平面 $F_p$ LOAM | $F_p$ LeGO |
|---|---|---|---|---|---|---|---|---|
| 1 | 157 | 102 | 323 | 152 | 878 | 253 | 4849 | 1319 |
| 2 | 145 | 102 | 331 | 154 | 798 | 254 | 4677 | 1227 |
| 3 | 174 | 101 | 172 | 103 | 819 | 163 | 6056 | 1146 |

分割后 LeGO-LOAM 需处理特征数对 $\mathbb F_e,\mathbb F_p,F_e,F_p$ 分别至少减 **29%, 40%, 68%, 72%**。

**TABLE III — LeGO-LOAM 两步优化迭代数比较**（源；处理一 scan 终止时平均迭代数）：

| 平台 | 场景 | 原始 Opt. 迭代数 | 原始 Opt. 时间(ms) | 两步 Step1 迭代数 | 两步 Step2 迭代数 |
|---|---|---|---|---|---|
| Jetson | 1 | 16.6 | 34.5 | 1.9 | 17.5 |
| Jetson | 2 | 15.7 | 32.9 | 1.7 | 16.7 |
| Jetson | 3 | 20.0 | 27.7 | 4.7 | 18.9 |
| i7 | 1 | 17.3 | 13.1 | 1.8 | 18.2 |
| i7 | 2 | 16.5 | 12.3 | 1.6 | 17.5 |
| i7 | 3 | 20.5 | 10.4 | 4.7 | 19.8 |

两步优化第一步在实验 1、2 中 **2 次迭代**完成；虽第二步迭代数与原始相近，但处理特征更少，故 lidar odometry 运行时间减 **34%–48%**。

**TABLE IV — 各模块处理一 scan 的运行时间（ms）**（源；列为 LOAM / LeGO-LOAM）：

| 平台 | 场景 | Seg. LOAM | Seg. LeGO | Extr. LOAM | Extr. LeGO | Odom. LOAM | Odom. LeGO | Map. LOAM | Map. LeGO |
|---|---|---|---|---|---|---|---|---|---|
| Jetson | 1 | N/A | 29.3 | 105.1 | 9.1 | 133.4 | 19.3 | 702.3 | 266.7 |
| Jetson | 2 | N/A | 29.9 | 106.7 | 9.9 | 124.5 | 18.6 | 793.6 | 278.2 |
| Jetson | 3 | N/A | 36.8 | 104.6 | 6.1 | 122.1 | 18.1 | 850.9 | 253.3 |
| i7 | 1 | N/A | 16.7 | 50.4 | 4.0 | 69.8 | 6.8 | 289.4 | 108.2 |
| i7 | 2 | N/A | 17.0 | 49.3 | 4.4 | 66.5 | 6.5 | 330.5 | 116.7 |
| i7 | 3 | N/A | 20.0 | 48.5 | 2.3 | 63.0 | 6.1 | 344.9 | 101.7 |

特征提取与 lidar odometry 运行时间在 LeGO-LOAM 中减**一个数量级**；LOAM 在 Jetson 上这两模块 >100 ms，导致跳过许多 scan（不实时）。lidar mapping 时间至少减 60%。

**TABLE I — 大规模室外数据集**（源）：

| 实验 | scan 数 | 高程变化(m) | 轨迹长(km) |
|---|---|---|---|
| 1 | 8077 | 11 | 1.09 |
| 2 | 8946 | 11 | 1.24 |
| 3 | 20834 | 19 | 2.71 |

**位姿误差**（源 §IV-A,B）：实验 3（林间步道，35 min、均速 1.3 m/s、高程变化 ~19 m），初始位姿置 $[0,0,0,0,0,0]$，比最终位姿与初始位姿求相对误差。LOAM 最终平移/旋转误差 = Jetson 上 69.40 m / 27.38°、laptop 上 62.11 m / 8.50°；**LeGO-LOAM** = Jetson 上 13.93 m / 7.73°、laptop 上 14.87 m / 7.96°。实验 2 中 LeGO-LOAM 精度比 LOAM 高一个数量级。

### D. KITTI 回环测试（源 §IV-D）
KITTI 序列 00，Jetson 上为实时把 HDL-64E 降采样到 VLP-16 同款 range image（**省略每 scan 75% 的点**）。用 **ICP** 在位姿图节点间加约束，**iSAM2 [24]** 优化图，用优化后的图校正传感器位姿与地图。

## §LeGO.V 结论（源 §V）
LeGO-LOAM 轻量（嵌入式实时）、地面优化（地面分离 + 点云分割 + 改进 L-M）。两步 L-M 分别计算位姿不同分量。可达相当或更优精度、更少计算时间。

---

# 第三部分：LIO-SAM（Tightly-coupled LIO via Smoothing and Mapping, IROS 2020）

> 把激光-惯性里程计建在**因子图**上，用 **iSAM2/Bayes 树**增量平滑。四类因子：**IMU 预积分因子、lidar 里程计因子、GPS 因子、回环因子**。用 IMU 预积分去畸变并给 scan-matching 初值；lidar 里程计反过来估 IMU 偏置。关键工程优化：**边缘化旧 scan、scan-to-局部滑窗 sub-keyframe** 而非 scan-to-全局地图——大幅提升实时性。

## §LIOSAM.I 引入与动机（源 §I）

**对 LOAM 的批判**（动机）：LOAM 把数据存全局体素地图，难做回环检测、难纳入 GPS 等绝对测量；体素地图在特征丰富环境变稠密后在线优化效率降低；LOAM 是 scan-matching 法、大规模测试有漂移。

**LIO-SAM 的应对**：紧耦合 LIO via smoothing and mapping。假设非线性运动模型用原始 IMU 测量做点云去畸变 + 给 lidar 里程计初值；lidar 里程计解再估 IMU 偏置。引入全局因子图：高效 lidar+IMU 融合、纳入位姿间地点识别、纳入 GPS/罗盘等绝对测量、联合优化。边缘化旧 scan（而非匹配全局地图）、scan-matching 在局部尺度——显著提升实时性；选择性关键帧 + 高效滑窗（新 keyframe 注册到固定大小的 prior "sub-keyframes"）。

**贡献**：(1) 建在因子图上的紧耦合 LIO 框架，适合多传感器融合与全局优化；(2) 高效局部滑窗 scan-matching（选择性 keyframe 注册到固定大小 prior sub-keyframes 实现实时）；(3) 跨尺度/车辆/环境广泛验证。

## §LIOSAM.III 方法（源 §III）

### A. 系统概览（源 §III-A）
world frame $\mathsf W$、robot body frame $\mathsf B$（假设 IMU 系与 body 系重合）。机器人状态：

$$\mathbf x=[\mathbf R^T,\mathbf p^T,\mathbf v^T,\mathbf b^T]^T,\tag{LIOSAM-1}$$

$\mathbf R\in\mathrm{SO}(3)$、$\mathbf p\in\mathbb R^3$、$\mathbf v$ 速度、$\mathbf b$ IMU 偏置。$\mathsf B\to\mathsf W$ 变换 $\mathbf T=[\mathbf R\mid\mathbf p]\in\mathrm{SE}(3)$。

**MAP 表述**：状态估计问题表述为**最大后验（MAP）**，用**因子图**建模（比 Bayes 网更适合推断）。高斯噪声假设下，MAP 推断**等价于求非线性最小二乘** [18]。可无损推广纳入其他绝对测量（高度计、罗盘）。

**四类因子 + 一种变量**：变量 = 特定时刻机器人状态（图节点）。四类因子：(a) IMU 预积分因子、(b) lidar 里程计因子、(c) GPS 因子、(d) 回环因子。当机器人位姿变化超用户阈值时，向图加新状态节点 $\mathbf x$。每插入新节点用 **iSAM2（incremental smoothing and mapping with the Bayes tree）[19]** 优化因子图。

### B. IMU 预积分因子（源 §III-B）

IMU 角速度、加速度测量：

$$\hat{\boldsymbol\omega}_t=\boldsymbol\omega_t+\mathbf b^\omega_t+\mathbf n^\omega_t,\tag{LIOSAM-2}$$
$$\hat{\mathbf a}_t=\mathbf R^{BW}_t(\mathbf a_t-\mathbf g)+\mathbf b^a_t+\mathbf n^a_t,\tag{LIOSAM-3}$$

$\hat{\boldsymbol\omega}_t,\hat{\mathbf a}_t$ 是 $\mathsf B$ 中时刻 $t$ 原始测量，受缓变偏置 $\mathbf b_t$ 与白噪 $\mathbf n_t$ 影响。$\mathbf R^{BW}_t$ 是 $\mathsf W\to\mathsf B$ 旋转，$\mathbf g$ 是 $\mathsf W$ 中常重力。

**用 IMU 推机器人运动**：$t+\Delta t$ 的速度、位置、旋转：

$$\mathbf v_{t+\Delta t}=\mathbf v_t+\mathbf g\Delta t+\mathbf R_t(\hat{\mathbf a}_t-\mathbf b^a_t-\mathbf n^a_t)\Delta t,\tag{LIOSAM-4}$$
$$\mathbf p_{t+\Delta t}=\mathbf p_t+\mathbf v_t\Delta t+\tfrac12\mathbf g\Delta t^2+\tfrac12\mathbf R_t(\hat{\mathbf a}_t-\mathbf b^a_t-\mathbf n^a_t)\Delta t^2,\tag{LIOSAM-5}$$
$$\mathbf R_{t+\Delta t}=\mathbf R_t\exp\big((\hat{\boldsymbol\omega}_t-\mathbf b^\omega_t-\mathbf n^\omega_t)\Delta t\big),\tag{LIOSAM-6}$$

其中 $\mathbf R_t=\mathbf R^{WB}_t={\mathbf R^{BW}_t}^T$。假设积分期间 $\mathsf B$ 的角速度与加速度恒定。

**预积分** [20]（Forster）：得两时刻间相对体运动。$i,j$ 间预积分测量 $\Delta\mathbf v_{ij},\Delta\mathbf p_{ij},\Delta\mathbf R_{ij}$：

$$\Delta\mathbf v_{ij}=\mathbf R_i^T(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij}),\tag{LIOSAM-7}$$
$$\Delta\mathbf p_{ij}=\mathbf R_i^T(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2),\tag{LIOSAM-8}$$
$$\Delta\mathbf R_{ij}=\mathbf R_i^T\mathbf R_j.\tag{LIOSAM-9}$$

（详细推导见 [20]，本抽取在 §补充·Forster 中逐式展开。）预积分自然给出**IMU 预积分因子**；IMU 偏置与 lidar 里程计因子在图中**联合优化**。

### C. Lidar 里程计因子（源 §III-C）

**特征提取**：评估点在局部区域的粗糙度，大粗糙度 = 边缘特征、小粗糙度 = 平面特征。时刻 $i$ 的边缘/平面特征记 $\mathbb F^e_i,\mathbb F^p_i$，一帧 $\mathbb F_i=\{\mathbb F^e_i,\mathbb F^p_i\}$（在 $\mathsf B$ 中）。细节见 LOAM [1]（用 range image 见 LeGO-LOAM [7]）。

**关键帧选取**：每帧都加因子计算不可行，故采用 keyframe。用简单启发式：当机器人位姿变化超用户阈值（**位置 1 m、旋转 10°**）时选 $\mathbb F_{i+1}$ 为 keyframe，关联新状态节点 $\mathbf x_{i+1}$；两 keyframe 间的帧丢弃。平衡地图密度与内存、维持稀疏因子图（适合实时非线性优化）。

**生成 lidar 里程计因子三步**：

**1) 体素地图 sub-keyframes**：滑窗法建含固定数量近期 scan 的点云地图。不优化两连续 scan 间变换，而是取**最近 $n$ 个 keyframe**（称 **sub-keyframes**）$\{\mathbb F_{i-n},\dots,\mathbb F_i\}$，用关联位姿 $\{\mathbf T_{i-n},\dots,\mathbf T_i\}$ 变换到 $\mathsf W$，合成体素地图 $\mathbb M_i$。$\mathbb M_i$ 由两子图组成：

$$\mathbb M_i=\{\mathbb M^e_i,\mathbb M^p_i\},$$
$$\mathbb M^e_i={}'\mathbb F^e_i\cup{}'\mathbb F^e_{i-1}\cup\dots\cup{}'\mathbb F^e_{i-n},$$
$$\mathbb M^p_i={}'\mathbb F^p_i\cup{}'\mathbb F^p_{i-1}\cup\dots\cup{}'\mathbb F^p_{i-n},$$

${}'\mathbb F^e_i,{}'\mathbb F^p_i$ 是变换到 $\mathsf W$ 的边缘/平面特征。$\mathbb M^e_i,\mathbb M^p_i$ 降采样去同体素重复特征。本文 $n=25$；$\mathbb M^e_i,\mathbb M^p_i$ 降采样分辨率 **0.2 m、0.4 m**。

**2) Scan-matching**：把新帧 $\mathbb F_{i+1}=\{\mathbb F^e_{i+1},\mathbb F^p_{i+1}\}$ 匹配到 $\mathbb M_i$。用 LOAM [1] 法（计算高效、鲁棒）。先用 IMU 预测运动 $\tilde{\mathbf T}_{i+1}$ 把 $\{\mathbb F^e_{i+1},\mathbb F^p_{i+1}\}$ 从 $\mathsf B$ 变到 $\mathsf W$ 得 $\{{}'\mathbb F^e_{i+1},{}'\mathbb F^p_{i+1}\}$；对每个特征在 $\mathbb M^e_i$ 或 $\mathbb M^p_i$ 找边缘/平面对应（细节见 [1]）。

**3) 相对变换**：特征到其边缘/平面对应的距离：

$$d_{e_k}=\frac{\big\|(\mathbf p^e_{i+1,k}-\mathbf p^e_{i,u})\times(\mathbf p^e_{i+1,k}-\mathbf p^e_{i,v})\big\|}{\big\|\mathbf p^e_{i,u}-\mathbf p^e_{i,v}\big\|},\tag{LIOSAM-10}$$

$$d_{p_k}=\frac{\Big|(\mathbf p^p_{i+1,k}-\mathbf p^p_{i,u})\cdot\big((\mathbf p^p_{i,u}-\mathbf p^p_{i,v})\times(\mathbf p^p_{i,u}-\mathbf p^p_{i,w})\big)\Big|}{\big\|(\mathbf p^p_{i,u}-\mathbf p^p_{i,v})\times(\mathbf p^p_{i,u}-\mathbf p^p_{i,w})\big\|},\tag{LIOSAM-11}$$

$k,u,v,w$ 是各自集合中特征索引。边缘特征 $\mathbf p^e_{i+1,k}\in{}'\mathbb F^e_{i+1}$ 的对应边缘线由 $\mathbb M^e_i$ 中 $\mathbf p^e_{i,u},\mathbf p^e_{i,v}$ 构成；平面特征 $\mathbf p^p_{i+1,k}\in{}'\mathbb F^p_{i+1}$ 的对应平面片由 $\mathbb M^p_i$ 中 $\mathbf p^p_{i,u},\mathbf p^p_{i,v},\mathbf p^p_{i,w}$ 构成。（这两式与 LOAM 的 (LOAM-2)、(LOAM-3) 同形。）

**Gauss-Newton** 求最优变换，最小化

$$\min_{\mathbf T_{i+1}}\ \Big\{\sum_{\mathbf p^e_{i+1,k}\in{}'\mathbb F^e_{i+1}}d_{e_k}+\sum_{\mathbf p^p_{i+1,k}\in{}'\mathbb F^p_{i+1}}d_{p_k}\Big\}.$$

得 $\mathbf x_i,\mathbf x_{i+1}$ 间**相对变换 = lidar 里程计因子**：

$$\Delta\mathbf T_{i,i+1}=\mathbf T_i^T\,\mathbf T_{i+1}.\tag{LIOSAM-12}$$

（注：另一等价做法是把 sub-keyframes 变换到 $\mathbf x_i$ 系直接得 $\Delta\mathbf T_{i,i+1}$；但因 ${}'\mathbb F^e_i,{}'\mathbb F^p_i$ 可复用，本文选 §III-C.1 法以提效。）

### D. GPS 因子（源 §III-D）
长时导航仍漂移，引入绝对测量传感器（高度计、罗盘、GPS）消漂。收到 GPS 测量先用 [21] 法转到本地笛卡尔系；向图加新节点时关联 GPS 因子。若 GPS 与 lidar 帧未硬件同步，按时间戳在 GPS 测量间**线性插值**。**只在估计位置协方差大于收到的 GPS 位置协方差时才加 GPS 因子**（lidar-惯性里程计漂移很慢，无需常加）。

### E. 回环因子（源 §III-E）
因用因子图，回环可无缝纳入（不像 LOAM/LIOM）。实现简单但有效的**欧氏距离回环检测**：新状态 $\mathbf x_{i+1}$ 加入图时，搜索图中欧氏空间靠近 $\mathbf x_{i+1}$ 的 prior 状态（如 Fig.1 中 $\mathbf x_3$ 是候选）；把 $\mathbb F_{i+1}$ 与 sub-keyframes $\{\mathbb F_{3-m},\dots,\mathbb F_3,\dots,\mathbb F_{3+m}\}$ scan-matching（先都变到 $\mathsf W$）；得相对变换 $\Delta\mathbf T_{3,i+1}$ 加为回环因子。本文 $m=12$，回环搜索距离设为离新状态 $\mathbf x_{i+1}$ **15 m**。框架兼容其他回环检测法（如点云描述子 [22],[23]）。

**实践发现**：当 GPS 是唯一绝对传感器时，回环因子对修正机器人**高度漂移**尤其有用——因 GPS 高程测量很不准（无回环时本文测试高度误差近 100 m）。

## §LIOSAM.IV 实验（源 §IV）
传感器：Velodyne VLP-16 + MicroStrain 3DM-GX5-25 IMU + Reach M GPS。5 个数据集：Rotation, Walking, Campus, Park, Amsterdam。LOAM、LIO-SAM 强制实时；LIOM 给无限时间。

**TABLE I — 数据集细节**（源）：

| 数据集 | scan 数 | 高程变化(m) | 轨迹长(m) | 最大旋转速度(°/s) |
|---|---|---|---|---|
| Rotation | 582 | 0 | 0 | 213.9 |
| Walking | 6502 | 0.3 | 801 | 133.7 |
| Campus | 9865 | 1.0 | 1437 | 124.8 |
| Park | 24691 | 19.0 | 2898 | 217.4 |
| Amsterdam | 107656 | 0 | 19065 | 17.2 |

**TABLE II — 端到端平移误差（米）**（源）：

| 数据集 | LOAM | LIOM | LIO-odom | LIO-GPS | LIO-SAM |
|---|---|---|---|---|---|
| Campus | 192.43 | Fail | 9.44 | 6.87 | 0.12 |
| Park | 121.74 | 34.60 | 36.36 | 2.93 | 0.04 |
| Amsterdam | Fail | Fail | Fail | 1.21 | 0.17 |

（LIO-odom = 仅 IMU 预积分+lidar 里程计因子；LIO-GPS = +GPS 因子；LIO-SAM = 全因子。）

**关键发现**（源 §IV-A,B）：Rotation 测试最大旋转 133.7°/s，LIO-SAM 保留更多结构细节（能在 SO(3) 精确注册每帧、即使快速旋转）；LIOM 因继承 VINS 初始化敏感性无法正确初始化。Walking 测试 LOAM 在快速旋转处发散；LIOM 仅 0.56× 实时；LIO-SAM 与 Google Earth 一致。

---

# 第四部分：FAST-LIO（Iterated ESKF LiDAR-Inertial Odometry, RA-L 2021）

> **本章核心**：紧耦合**迭代扩展 Kalman 滤波（iterated EKF / iterated ESKF）**融合 LiDAR 特征点与 IMU。两大贡献：(1) **前向 + 反向传播**预测状态、补偿一个 scan 内的运动畸变；(2) **新 Kalman 增益公式**——计算复杂度只依赖**状态维**（18）而非**测量维**（>1000），并证明与标准公式等价。下面把迭代 ESKF 完整推导（含 $\boxplus/\boxminus$ 流形、误差状态动力学、MAP→Kalman、两处附录证明）逐行展开。

## §FASTLIO.I-II 引入与相关工作（源 §I-II）

**动机**：固态 LiDAR（MEMS / 旋转棱镜）便宜轻量高性能、适合小型 UAV，但带来挑战：(1) 特征点是几何结构（边/面），杂乱环境无强特征时易退化（小 FoV 更明显）；(2) 一个 scan 含数千特征点，紧融合海量点到 IMU 需庞大算力（UAV 板载难承受）；(3) LiDAR 顺序采样、点在不同时刻采得→运动畸变；UAV 螺旋桨/电机持续旋转给 IMU 引入显著噪声。

**相关工作要点**（源 §II-C）：紧耦合 LIO 分优化法与滤波法。Kalman 滤波及变体时间复杂度 $O(m^2)$（$m$ = 测量维），海量 LiDAR 测量时算力高；朴素降采样减测量但损信息。FAST-LIO 用类似 [21] 的**迭代 EKF** 缓解线性化误差，但用**新增益公式**避免 $O(m^2)$ 限制。

## §FASTLIO.III 方法（源 §III）

### A. 框架概览（源 §III-A）
LiDAR 输入→特征提取（平面+边缘）→特征 + IMU 送状态估计模块（**10–50 Hz**）→估计位姿把特征点注册到全局系、与已建特征地图合并→更新地图用于下一步注册新点。

### B.1 $\boxplus/\boxminus$ 算子（源 §III-B.1）

设 $\mathcal M$ 为 $n$ 维流形（如 $\mathcal M=\mathrm{SO}(3)$）。因流形局部同胚于 $\mathbb R^n$，可经两个**封装算子** $\boxplus,\boxminus$ [23] 建立 $\mathcal M$ 局部邻域到切空间 $\mathbb R^n$ 的双射：

$$\boxplus:\mathcal M\times\mathbb R^n\to\mathcal M;\qquad\boxminus:\mathcal M\times\mathcal M\to\mathbb R^n.$$

对 $\mathcal M=\mathrm{SO}(3)$：

$$\mathbf R\boxplus\mathbf r=\mathbf R\,\mathrm{Exp}(\mathbf r);\qquad\mathbf R_1\boxminus\mathbf R_2=\mathrm{Log}(\mathbf R_2^T\mathbf R_1).$$

对 $\mathcal M=\mathbb R^n$：

$$\mathbf a\boxplus\mathbf b=\mathbf a+\mathbf b;\qquad\mathbf a\boxminus\mathbf b=\mathbf a-\mathbf b.$$

其中指数映射

$$\mathrm{Exp}(\mathbf r)=\mathbf I+\frac{\mathbf r}{\|\mathbf r\|}\sin(\|\mathbf r\|)+\frac{\mathbf r^2}{\|\mathbf r\|^2}\big(1-\cos(\|\mathbf r\|)\big)$$

（这里 $\mathbf r$ 视作其反对称矩阵 $\lfloor\mathbf r\rfloor_\wedge$），$\mathrm{Log}(\cdot)$ 是其逆。对复合流形 $\mathcal M=\mathrm{SO}(3)\times\mathbb R^n$：

$$\begin{bmatrix}\mathbf R\\\mathbf a\end{bmatrix}\boxplus\begin{bmatrix}\mathbf r\\\mathbf b\end{bmatrix}=\begin{bmatrix}\mathbf R\,\mathrm{Exp}(\mathbf r)\\\mathbf a+\mathbf b\end{bmatrix};\qquad\begin{bmatrix}\mathbf R_1\\\mathbf a\end{bmatrix}\boxminus\begin{bmatrix}\mathbf R_2\\\mathbf b\end{bmatrix}=\begin{bmatrix}\mathbf R_1\boxminus\mathbf R_2\\\mathbf a-\mathbf b\end{bmatrix}.$$

**封装算子恒等式**（易验证，源给出）：

$$(\mathbf x\boxplus\mathbf u)\boxminus\mathbf x=\mathbf u;\qquad\mathbf x\boxplus(\mathbf y\boxminus\mathbf x)=\mathbf y,\quad\forall\mathbf x,\mathbf y\in\mathcal M,\ \forall\mathbf u\in\mathbb R^n.$$

**【与本书对接】** $\mathbf R\boxplus\mathbf r=\mathbf R\,\mathrm{Exp}(\mathbf r)$ 正是本书**右扰动**约定；$\boxminus$ 用 $\mathbf R_2^T\mathbf R_1$ 也是右扰动局部坐标。本书李群章已建立同款 $\boxplus/\boxminus$，记号直接对接。

### B.2 连续模型（源 §III-B.2）

IMU 刚连 LiDAR，已知外参 ${}^I\mathbf T_L=({}^I\mathbf R_L,{}^I\mathbf p_L)$。取 IMU 系 $I$ 为体参考系，运动学模型：

$$\begin{aligned}
{}^G\dot{\mathbf p}_I&={}^G\mathbf v_I,\quad {}^G\dot{\mathbf v}_I={}^G\mathbf R_I(\mathbf a_m-\mathbf b_a-\mathbf n_a)+{}^G\mathbf g,\quad {}^G\dot{\mathbf g}=\mathbf 0,\\
{}^G\dot{\mathbf R}_I&={}^G\mathbf R_I\lfloor\boldsymbol\omega_m-\mathbf b_\omega-\mathbf n_\omega\rfloor_\wedge,\quad\dot{\mathbf b}_\omega=\mathbf n_{b\omega},\quad\dot{\mathbf b}_a=\mathbf n_{ba},
\end{aligned}\tag{FASTLIO-1}$$

其中 ${}^G\mathbf p_I,{}^G\mathbf R_I$ 是 IMU 在 global 系 $G$（第一个 IMU 系）中的位置、姿态；${}^G\mathbf g$ 是 global 系未知重力；$\mathbf a_m,\boldsymbol\omega_m$ 是 IMU 测量；$\mathbf n_a,\mathbf n_\omega$ 是 IMU 测量白噪；$\mathbf b_a,\mathbf b_\omega$ 是建模为带高斯噪 $\mathbf n_{ba},\mathbf n_{b\omega}$ 的**随机游走**偏置；$\lfloor\mathbf a\rfloor_\wedge$ 是 $\mathbf a\in\mathbb R^3$ 的反对称矩阵（叉乘算子）。

### B.3 离散模型（源 §III-B.3）

基于 $\boxplus$，用零阶保持以 IMU 采样周期 $\Delta t$ 离散化 (FASTLIO-1)：

$$\mathbf x_{i+1}=\mathbf x_i\boxplus\big(\Delta t\,\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\big),\tag{FASTLIO-2}$$

$i$ 是 IMU 测量索引，函数 $\mathbf f$、状态 $\mathbf x$、输入 $\mathbf u$、噪声 $\mathbf w$ 定义为：

$$\mathcal M=\mathrm{SO}(3)\times\mathbb R^{15},\quad\dim(\mathcal M)=18,$$
$$\mathbf x\doteq\begin{bmatrix}{}^G\mathbf R_I^T & {}^G\mathbf p_I^T & {}^G\mathbf v_I^T & \mathbf b_\omega^T & \mathbf b_a^T & {}^G\mathbf g^T\end{bmatrix}^T\in\mathcal M,$$
$$\mathbf u\doteq\begin{bmatrix}\boldsymbol\omega_m^T & \mathbf a_m^T\end{bmatrix}^T,\quad\mathbf w\doteq\begin{bmatrix}\mathbf n_\omega^T & \mathbf n_a^T & \mathbf n_{b\omega}^T & \mathbf n_{ba}^T\end{bmatrix}^T,$$
$$\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)=\begin{bmatrix}
\boldsymbol\omega_{m_i}-\mathbf b_{\omega_i}-\mathbf n_{\omega_i}\\
{}^G\mathbf v_{I_i}\\
{}^G\mathbf R_{I_i}(\mathbf a_{m_i}-\mathbf b_{a_i}-\mathbf n_{a_i})+{}^G\mathbf g_i\\
\mathbf n_{b\omega_i}\\
\mathbf n_{ba_i}\\
\mathbf 0_{3\times1}
\end{bmatrix}.\tag{FASTLIO-3}$$

### B.4 LiDAR 测量预处理（源 §III-B.4）
LiDAR 点是其局部体系坐标。原始点采样率极高（如 200 kHz），不可能逐点处理；实际累积一段时间一次处理。FAST-LIO 最小累积间隔 **20 ms**→最高 50 Hz 全状态估计与地图更新。这样一组累积点称一个 **scan**，处理它的时刻记 $t_k$。从原始点提取高局部光滑度的平面点 [8] 与低局部光滑度的边缘点 [10]。设特征点数 $m$，第 $j$ 个采样于 $\rho_j\in(t_{k-1},t_k]$，记 ${}^{L_j}\mathbf p_{f_j}$（$L_j$ 是 $\rho_j$ 时刻 LiDAR 局部系）。一个 scan 内也有多个 IMU 测量，第 $i$ 个采样于 $\tau_i\in[t_{k-1},t_k]$、对应状态 $\mathbf x_i$。注意最后一个 LiDAR 特征点是 scan 末（$\rho_m=t_k$），IMU 测量未必与 scan 始末对齐。

## §FASTLIO.III-C 状态估计：迭代扩展 Kalman 滤波（源 §III-C）

用迭代 EKF；在状态估计的**切空间**刻画估计协方差 [23,24]。设上一 scan（$t_{k-1}$）最优状态估计为 $\bar{\mathbf x}_{k-1}$、协方差 $\bar{\mathbf P}_{k-1}$。$\bar{\mathbf P}_{k-1}$ 表示如下**误差状态向量**的协方差：

$$\tilde{\mathbf x}_{k-1}\doteq\mathbf x_{k-1}\boxminus\bar{\mathbf x}_{k-1}=\begin{bmatrix}\delta\boldsymbol\theta^T & {}^G\tilde{\mathbf p}_I^T & {}^G\tilde{\mathbf v}_I^T & \tilde{\mathbf b}_\omega^T & \tilde{\mathbf b}_a^T & {}^G\tilde{\mathbf g}^T\end{bmatrix}^T,$$

其中 $\delta\boldsymbol\theta=\mathrm{Log}({}^G\bar{\mathbf R}_I^T\,{}^G\mathbf R_I)$ 是**姿态误差**，其余是标准加性误差（$\tilde{\mathbf x}=\mathbf x-\bar{\mathbf x}$）。姿态误差 $\delta\boldsymbol\theta$ 描述真姿态与估计姿态间的（小）偏差。**此误差定义的主要优点**：用 $3\times3$ 协方差 $\mathbb E[\delta\boldsymbol\theta\delta\boldsymbol\theta^T]$ 表示姿态不确定度——姿态 3 DOF，这是最小表示。

### C.1 前向传播（Forward Propagation，源）

收到 IMU 输入即做前向传播。把过程噪声 $\mathbf w_i$ 置零，按 (FASTLIO-2) 传播状态：

$$\hat{\mathbf x}_{i+1}=\hat{\mathbf x}_i\boxplus\big(\Delta t\,\mathbf f(\hat{\mathbf x}_i,\mathbf u_i,\mathbf 0)\big);\qquad\hat{\mathbf x}_0=\bar{\mathbf x}_{k-1},\tag{FASTLIO-4}$$

$\Delta t=\tau_{i+1}-\tau_i$。**传播协方差**，需误差状态动力学模型，推导如下：

$$\begin{aligned}
\tilde{\mathbf x}_{i+1}&=\mathbf x_{i+1}\boxminus\hat{\mathbf x}_{i+1}\\
&=\big(\mathbf x_i\boxplus\Delta t\,\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\big)\boxminus\big(\hat{\mathbf x}_i\boxplus\Delta t\,\mathbf f(\hat{\mathbf x}_i,\mathbf u_i,\mathbf 0)\big)\\
&\overset{(23)}{\simeq}\mathbf F_{\tilde{\mathbf x}}\,\tilde{\mathbf x}_i+\mathbf F_{\mathbf w}\,\mathbf w_i.
\end{aligned}\tag{FASTLIO-5}$$

矩阵 $\mathbf F_{\tilde{\mathbf x}}$、$\mathbf F_{\mathbf w}$ 按 Appendix A 计算，结果为（源式 7）：

$$\mathbf F_{\tilde{\mathbf x}}=\begin{bmatrix}
\mathrm{Exp}(-\hat{\boldsymbol\omega}_i\Delta t) & \mathbf 0 & \mathbf 0 & -\mathbf A(\hat{\boldsymbol\omega}_i\Delta t)^T\Delta t & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf I & \mathbf I\Delta t & \mathbf 0 & \mathbf 0 & \mathbf 0\\
-{}^G\hat{\mathbf R}_{I_i}\lfloor\hat{\mathbf a}_i\rfloor_\wedge\Delta t & \mathbf 0 & \mathbf I & \mathbf 0 & -{}^G\hat{\mathbf R}_{I_i}\Delta t & \mathbf I\Delta t\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I
\end{bmatrix},$$
$$\mathbf F_{\mathbf w}=\begin{bmatrix}
-\mathbf A(\hat{\boldsymbol\omega}_i\Delta t)^T\Delta t & \mathbf 0 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & -{}^G\hat{\mathbf R}_{I_i}\Delta t & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf I\Delta t & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I\Delta t\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0
\end{bmatrix},\tag{FASTLIO-7}$$

其中 $\hat{\boldsymbol\omega}_i=\boldsymbol\omega_{m_i}-\hat{\mathbf b}_\omega$、$\hat{\mathbf a}_i=\mathbf a_{m_i}-\hat{\mathbf b}_a$，$\mathbf A(\mathbf u)^{-1}$ 按 [25] 定义（源式 6）：

$$\mathbf A(\mathbf u)^{-1}=\mathbf I-\tfrac12\lfloor\mathbf u\rfloor_\wedge+\big(1-\alpha(\|\mathbf u\|)\big)\frac{\lfloor\mathbf u\rfloor_\wedge^2}{\|\mathbf u\|^2},\quad\alpha(m)=\frac{m}{2}\cot\frac{m}{2}=\frac{m\cos(m/2)}{2\sin(m/2)}.\tag{FASTLIO-6}$$

记白噪 $\mathbf w$ 的协方差为 $\mathbf Q$，则**传播协方差**迭代为：

$$\hat{\mathbf P}_{i+1}=\mathbf F_{\tilde{\mathbf x}}\,\hat{\mathbf P}_i\,\mathbf F_{\tilde{\mathbf x}}^T+\mathbf F_{\mathbf w}\,\mathbf Q\,\mathbf F_{\mathbf w}^T;\qquad\hat{\mathbf P}_0=\bar{\mathbf P}_{k-1}.\tag{FASTLIO-8}$$

传播持续到新 scan 结束时刻 $t_k$，得传播状态 $\hat{\mathbf x}_k$、协方差 $\hat{\mathbf P}_k$。$\hat{\mathbf P}_k$ 表示真值 $\mathbf x_k$ 与传播 $\hat{\mathbf x}_k$ 间误差（即 $\mathbf x_k\boxminus\hat{\mathbf x}_k$）的协方差。

### C.2 反向传播与运动补偿（Backward Propagation，源）

到 $t_k$ 累积间隔到时，新 scan 特征点应与传播 $\hat{\mathbf x}_k,\hat{\mathbf P}_k$ 融合产生最优更新。但虽新 scan 在 $t_k$，特征点在各自 $\rho_j\le t_k$ 测得，造成体参考系不匹配。

为补偿 $\rho_j$ 与 $t_k$ 间的相对运动（运动畸变），把 (FASTLIO-2) **反向传播**：$\check{\mathbf x}_{j-1}=\check{\mathbf x}_j\boxplus(-\Delta t\,\mathbf f(\check{\mathbf x}_j,\mathbf u_j,\mathbf 0))$，从**零位姿**、其余状态（速度、偏置）取自 $\hat{\mathbf x}_k$ 开始。反向传播在特征点频率（远高于 IMU 率）进行；两 IMU 测量间的所有特征点用**左侧 IMU 测量**作输入。注意到 $\mathbf f(\mathbf x_j,\mathbf u_j,\mathbf 0)$ 的最后三块（陀螺偏置、加表偏置、外参对应）为零，反向传播可化简为：

$$\begin{aligned}
{}^{I_k}\check{\mathbf p}_{I_{j-1}}&={}^{I_k}\check{\mathbf p}_{I_j}-{}^{I_k}\check{\mathbf v}_{I_j}\Delta t,&&\text{s.f. }{}^{I_k}\check{\mathbf p}_{I_m}=\mathbf 0;\\
{}^{I_k}\check{\mathbf v}_{I_{j-1}}&={}^{I_k}\check{\mathbf v}_{I_j}-{}^{I_k}\check{\mathbf R}_{I_j}(\mathbf a_{m_{i-1}}-\hat{\mathbf b}_{a_k})\Delta t-{}^{I_k}\hat{\mathbf g}_k\Delta t,&&\text{s.f. }{}^{I_k}\check{\mathbf v}_{I_m}={}^G\hat{\mathbf R}_{I_k}^T\,{}^G\hat{\mathbf v}_{I_k},\ {}^{I_k}\hat{\mathbf g}_k={}^G\hat{\mathbf R}_{I_k}^T\,{}^G\hat{\mathbf g}_k;\\
{}^{I_k}\check{\mathbf R}_{I_{j-1}}&={}^{I_k}\check{\mathbf R}_{I_j}\,\mathrm{Exp}\big((\hat{\mathbf b}_{\omega_k}-\boldsymbol\omega_{m_{i-1}})\Delta t\big),&&\text{s.f. }{}^{I_k}\check{\mathbf R}_{I_m}=\mathbf I.
\end{aligned}\tag{FASTLIO-9}$$

其中 $\rho_{j-1}\in[\tau_{i-1},\tau_i)$、$\Delta t=\rho_j-\rho_{j-1}$、"s.f." 意为 "starting from"（始于）。

反向传播产生 $\rho_j$ 与 scan 末 $t_k$ 间的相对位姿 ${}^{I_k}\check{\mathbf T}_{I_j}=({}^{I_k}\check{\mathbf R}_{I_j},{}^{I_k}\check{\mathbf p}_{I_j})$。用它把局部测量 ${}^{L_j}\mathbf p_{f_j}$ 投影到 scan 末测量 ${}^{L_k}\mathbf p_{f_j}$：

$$\boxed{\ {}^{L_k}\mathbf p_{f_j}={}^I\mathbf T_L^{-1}\,{}^{I_k}\check{\mathbf T}_{I_j}\,{}^I\mathbf T_L\,{}^{L_j}\mathbf p_{f_j}.\ }\tag{FASTLIO-10}$$

${}^I\mathbf T_L$ 是已知外参。投影后的点 ${}^{L_k}\mathbf p_{f_j}$ 用于构造残差。

### C.3 残差计算（Residual Computation，源）

经 (FASTLIO-10) 运动补偿，可视所有特征点 $\{{}^{L_k}\mathbf p_{f_j}\}$ 都在同一时刻 $t_k$ 采样、用以构残差。设当前迭代为 $\kappa$、状态估计为 $\hat{\mathbf x}^\kappa_k$（$\kappa=0$ 时 $\hat{\mathbf x}^\kappa_k=\hat{\mathbf x}_k$，即 (FASTLIO-4) 的传播预测）。特征点变换到全局系：

$$\hat{\mathbf p}^{G\,\kappa}_{f_j}={}^G\hat{\mathbf T}^\kappa_{I_k}\,{}^I\mathbf T_L\,{}^{L_k}\mathbf p_{f_j};\quad j=1,\dots,m.\tag{FASTLIO-11}$$

对每个 LiDAR 特征点，假设其真正所属是**地图中由近邻特征点定义的最近平面或边缘**。即残差 = 特征点估计全局坐标 $\hat{\mathbf p}^{G\,\kappa}_{f_j}$ 与地图中最近平面（或边缘）的距离。记 $\mathbf u_j$ 为对应平面（或边缘）的法向量（或边缘方向）、其上一点 ${}^G\mathbf q_j$，则残差 $\mathbf z^\kappa_j$：

$$\mathbf z^\kappa_j=\mathbf G_j\big(\hat{\mathbf p}^{G\,\kappa}_{f_j}-{}^G\mathbf q_j\big),\tag{FASTLIO-12}$$

其中**平面特征** $\mathbf G_j=\mathbf u_j^T$、**边缘特征** $\mathbf G_j=\lfloor\mathbf u_j\rfloor_\wedge$。$\mathbf u_j$ 计算与地图近邻搜索（定义对应平面/边缘）由地图点 KD-tree [10] 实现。只考虑范数低于阈值（如 0.5 m）的残差；超阈值者为外点或新观测点。

### C.4 迭代状态更新（Iterated State Update，源）

为把残差 $\mathbf z^\kappa_j$ 与传播 $\hat{\mathbf x}_k,\hat{\mathbf P}_k$ 融合，须线性化测量模型。测量噪来自测 ${}^{L_j}\mathbf p_{f_j}$ 时的 LiDAR 测距与束指向噪 ${}^{L_j}\mathbf n_{f_j}$。从点测量去噪得真点位置：

$$\mathbf p^{gt}_{f_j}={}^{L_j}\mathbf p_{f_j}-{}^{L_j}\mathbf n_{f_j}.\tag{FASTLIO-13}$$

此真点经 (FASTLIO-10) 投影到 $L_k$、再用真值状态 $\mathbf x_k$ 投影到全局系，**应精确落在地图平面（或边缘）上**。即把 (13) 代入 (10)、再 (11)、再 (12) 应得零：

$$0=\mathbf h_j\big(\mathbf x_k,{}^{L_j}\mathbf n_{f_j}\big)=\mathbf G_j\Big({}^G\mathbf T_{I_k}\,{}^{I_k}\check{\mathbf T}_{I_j}\,{}^I\mathbf T_L\big({}^{L_j}\mathbf p_{f_j}-{}^{L_j}\mathbf n_{f_j}\big)-{}^G\mathbf q_j\Big).$$

**一阶近似**（在 $\hat{\mathbf x}^\kappa_k$ 处）：

$$\begin{aligned}
0=\mathbf h_j\big(\mathbf x_k,{}^{L_j}\mathbf n_{f_j}\big)&\simeq\mathbf h_j(\hat{\mathbf x}^\kappa_k,\mathbf 0)+\mathbf H^\kappa_j\,\tilde{\mathbf x}^\kappa_k+\mathbf v_j\\
&=\mathbf z^\kappa_j+\mathbf H^\kappa_j\,\tilde{\mathbf x}^\kappa_k+\mathbf v_j,
\end{aligned}\tag{FASTLIO-14}$$

其中 $\tilde{\mathbf x}^\kappa_k=\mathbf x_k\boxminus\hat{\mathbf x}^\kappa_k$（等价 $\mathbf x_k=\hat{\mathbf x}^\kappa_k\boxplus\tilde{\mathbf x}^\kappa_k$），$\mathbf H^\kappa_j$ 是 $\mathbf h_j(\hat{\mathbf x}^\kappa_k\boxplus\tilde{\mathbf x}^\kappa_k,{}^{L_j}\mathbf n_{f_j})$ 对 $\tilde{\mathbf x}^\kappa_k$ 的 Jacobian（在零处求值），$\mathbf v_j\in\mathcal N(\mathbf 0,\mathbf R_j)$ 来自原始测量噪 ${}^{L_j}\mathbf n_{f_j}$。

**先验分布**（关键步骤）。前向传播得 $\mathbf x_k$ 的先验是对 $\mathbf x_k\boxminus\hat{\mathbf x}_k$ 而言的。但当前迭代点是 $\hat{\mathbf x}^\kappa_k$，须把先验转写到 $\hat{\mathbf x}^\kappa_k$ 处：

$$\mathbf x_k\boxminus\hat{\mathbf x}_k=(\hat{\mathbf x}^\kappa_k\boxplus\tilde{\mathbf x}^\kappa_k)\boxminus\hat{\mathbf x}_k=\hat{\mathbf x}^\kappa_k\boxminus\hat{\mathbf x}_k+\mathbf J^\kappa\tilde{\mathbf x}^\kappa_k,\tag{FASTLIO-15}$$

其中 $\mathbf J^\kappa$ 是 $(\hat{\mathbf x}^\kappa_k\boxplus\tilde{\mathbf x}^\kappa_k)\boxminus\hat{\mathbf x}_k$ 对 $\tilde{\mathbf x}^\kappa_k$ 的偏导（在零处求值）：

$$\mathbf J^\kappa=\begin{bmatrix}
\mathbf A\big({}^G\hat{\mathbf R}^\kappa_{I_k}\boxminus{}^G\hat{\mathbf R}_{I_k}\big)^{-T} & \mathbf 0_{3\times15}\\
\mathbf 0_{15\times3} & \mathbf I_{15\times15}
\end{bmatrix},\tag{FASTLIO-16}$$

其中 $\mathbf A(\cdot)^{-1}$ 由 (FASTLIO-6) 定义。**第一次迭代（即 EKF 情形）**：$\hat{\mathbf x}^\kappa_k=\hat{\mathbf x}_k$，故 $\mathbf J^\kappa=\mathbf I$。

**MAP 估计**。结合 (15) 的先验与 (14) 的后验分布，得**最大后验估计（MAP）**：

$$\boxed{\ \min_{\tilde{\mathbf x}^\kappa_k}\left(\big\|\mathbf x_k\boxminus\hat{\mathbf x}_k\big\|^2_{\hat{\mathbf P}_k^{-1}}+\sum_{j=1}^m\big\|\mathbf z^\kappa_j+\mathbf H^\kappa_j\tilde{\mathbf x}^\kappa_k\big\|^2_{\mathbf R_j^{-1}}\right)\ }\tag{FASTLIO-17}$$

其中 $\|\mathbf x\|^2_{\mathbf M}=\mathbf x^T\mathbf M\mathbf x$。把 (15) 的先验线性化代入 (17)、优化所得二次代价，导出**标准迭代 Kalman 滤波** [21]，计算如下（简记 $\mathbf H=[{\mathbf H^\kappa_1}^T,\dots,{\mathbf H^\kappa_m}^T]^T$、$\mathbf R=\mathrm{diag}(\mathbf R_1,\dots,\mathbf R_m)$、$\mathbf P=(\mathbf J^\kappa)^{-1}\hat{\mathbf P}_k(\mathbf J^\kappa)^{-T}$、$\mathbf z^\kappa=[{\mathbf z^\kappa_1}^T,\dots,{\mathbf z^\kappa_m}^T]^T$）：

$$\mathbf K=\mathbf P\mathbf H^T(\mathbf H\mathbf P\mathbf H^T+\mathbf R)^{-1},\tag{FASTLIO-18a}$$
$$\hat{\mathbf x}^{\kappa+1}_k=\hat{\mathbf x}^\kappa_k\boxplus\Big(-\mathbf K\mathbf z^\kappa_k-(\mathbf I-\mathbf K\mathbf H)(\mathbf J^\kappa)^{-1}\big(\hat{\mathbf x}^\kappa_k\boxminus\hat{\mathbf x}_k\big)\Big).\tag{FASTLIO-18b}$$

更新 $\hat{\mathbf x}^{\kappa+1}_k$ 用于重算 §C.3 残差、重复至**收敛**（$\|\hat{\mathbf x}^{\kappa+1}_k\boxminus\hat{\mathbf x}^\kappa_k\|<\epsilon$）。收敛后最优状态估计与协方差：

$$\bar{\mathbf x}_k=\hat{\mathbf x}^{\kappa+1}_k,\qquad\bar{\mathbf P}_k=(\mathbf I-\mathbf K\mathbf H)\mathbf P.\tag{FASTLIO-19}$$

### 新 Kalman 增益公式（FAST-LIO 核心贡献，源）

**问题**：(18a) 需求逆 $\mathbf H\mathbf P\mathbf H^T+\mathbf R$，维度 = **测量维**。LiDAR 特征点极多，求逆此矩阵代价高。既有工作 [21,26] 因此只用少量测量。

**直觉**：(17) 的代价是**对状态**的，故解的复杂度应依赖**状态维**。若直接解 (17)，可得与 (18) 同解、但增益新形：

$$\boxed{\ \mathbf K=(\mathbf H^T\mathbf R^{-1}\mathbf H+\mathbf P^{-1})^{-1}\mathbf H^T\mathbf R^{-1}.\ }\tag{FASTLIO-20}$$

附录 B 证明二者等价（基于矩阵求逆引理）。因 LiDAR 测量独立、$\mathbf R$ 是（块）对角，新公式只需求逆**两个状态维**矩阵（$\mathbf R^{-1}$ 块对角易求 + $(\mathbf H^T\mathbf R^{-1}\mathbf H+\mathbf P^{-1})$ 是状态维）。状态维（18）远小于测量维（>1000），新公式大省计算。

### C.5 算法（源 Algorithm 1，逐行）

> **Algorithm 1: State Estimation**
> **Input**: 上一最优估计 $\bar{\mathbf x}_{k-1},\bar{\mathbf P}_{k-1}$；当前 scan 的 IMU 输入 $(\mathbf a_m,\boldsymbol\omega_m)$；当前 scan 的 LiDAR 特征点 ${}^{L_j}\mathbf p_{f_j}$。
> 1. Forward propagation to obtain state prediction $\hat{\mathbf x}_k$ via (FASTLIO-4) and covariance prediction $\hat{\mathbf P}_k$ via (FASTLIO-8);
> 2. Backward propagation to obtain ${}^{L_k}\mathbf p_{f_j}$ via (FASTLIO-9), (FASTLIO-10);
> 3. $\kappa=-1$, $\hat{\mathbf x}^{\kappa=0}_k=\hat{\mathbf x}_k$;
> 4. **repeat**
> 5. &nbsp;&nbsp;$\kappa=\kappa+1$;
> 6. &nbsp;&nbsp;Compute $\mathbf J^\kappa$ via (FASTLIO-16) and $\mathbf P=(\mathbf J^\kappa)^{-1}\hat{\mathbf P}_k(\mathbf J^\kappa)^{-T}$;
> 7. &nbsp;&nbsp;Compute residual $\mathbf z^\kappa_j$ (FASTLIO-12) and Jacobian $\mathbf H^\kappa_j$ (FASTLIO-14);
> 8. &nbsp;&nbsp;Compute the state update $\hat{\mathbf x}^{\kappa+1}_k$ via (FASTLIO-18) with the Kalman gain $\mathbf K$ from (FASTLIO-20);
> 9. **until** $\|\hat{\mathbf x}^{\kappa+1}_k\boxminus\hat{\mathbf x}^\kappa_k\|<\epsilon$;
> 10. $\bar{\mathbf x}_k=\hat{\mathbf x}^{\kappa+1}_k$; $\bar{\mathbf P}_k=(\mathbf I-\mathbf K\mathbf H)\mathbf P$.
> **Output**: 当前最优估计 $\bar{\mathbf x}_k,\bar{\mathbf P}_k$。

### D. 地图更新（源 §III-D）
有了状态更新 $\bar{\mathbf x}_k$（即 ${}^G\bar{\mathbf T}_{I_k}=({}^G\bar{\mathbf R}_{I_k},{}^G\bar{\mathbf p}_{I_k})$），每个投影到体系 $L_k$ 的特征点（见 (10)）变换到全局系：

$${}^G\bar{\mathbf p}_{f_j}={}^G\bar{\mathbf T}_{I_k}\,{}^I\mathbf T_L\,{}^{L_k}\mathbf p_{f_j};\quad j=1,\dots,m.\tag{FASTLIO-21}$$

这些点最终追加到含所有历史步特征点的地图。

### E. 初始化（源 §III-E）
保持 LiDAR 静止数秒（本文所有实验 2 s），用采集数据初始化 IMU 偏置与重力向量。若 LiDAR 支持非重复扫描（如 Livox AVIA），静止还能捕获初始高分辨率地图。

## §FASTLIO.IV 实验（源 §IV）

### A. 计算复杂度实验（源 §IV-A）
**TABLE II — 两种 Kalman 增益公式运行时间**（源）：

| 特征数 | 307 | 717 | 998 | 1243 | 1453 | 1802 |
|---|---|---|---|---|---|---|
| Old Formula (ms) | 7.1 | 23.4 | 109.3 | 251 | 1219 | 1621 |
| New Formula (ms) | 0.07 | 0.11 | 0.25 | 0.37 | 0.59 | 1.16 |

新公式复杂度远低于旧公式。

### B. UAV 飞行实验（源 §IV-B）
小型四旋翼载 Livox Avia（70° FoV）+ DJI Manifold 2-C（1.8 GHz 四核 i7-8550U、8 GB RAM）、280 mm 轴距。室内最高 50 Hz 实时里程计与建图。50 Hz 室内：平均有效特征点 270、运行时间 6.7 ms、漂移 <0.3%（32 m 轨迹漂 0.08 m）。

### C. 室内实验（源 §IV-C）
挑战性室内大旋转（角速度常超 100 deg/s）。**TABLE III — 10 Hz 处理一 LiDAR scan 时间比较**（源）：

| 包 | 有效特征数 | 运行时间 |
|---|---|---|
| LOAM | 1107 | 59 ms |
| LOAM+IMU | 1107 | 44 ms |
| FAST-LIO | 1430 | 23 ms |

（LOAM+IMU 是松耦合、致映射不一致。）

### D. 室外实验（源 §IV-D）
港大 Main Building 手持 ~140 m 返回起点，漂移 <0.05%（140 m 漂 0.07 m），10 Hz、平均处理 25 ms、平均 1497 有效特征点。与 LINS [21] 比：FAST-LIO 平均 7.3 ms、LINS 34.5 ms（均 10 Hz）；LINS 因 EKF 高复杂度降采样到平均 147 点/scan（FAST-LIO 784 点/scan），致 LINS 映射精度降。

## §FASTLIO.V 结论（源 §V）
FAST-LIO：紧耦合迭代 Kalman 滤波的高效鲁棒 LIO。前向+反向传播预测状态、补偿 scan 内运动。证明并实现等价公式大降 Kalman 增益计算复杂度。

## §FASTLIO 附录 A：$\mathbf F_{\tilde{\mathbf x}}$ 与 $\mathbf F_{\mathbf w}$ 的计算（源 Appendix A，完整）

记 $\mathbf x_i=\hat{\mathbf x}_i\boxplus\tilde{\mathbf x}_i$，定义 $\mathbf g(\tilde{\mathbf x}_i,\mathbf w_i)=\mathbf f(\mathbf x_i,\mathbf u_i,\mathbf w_i)\Delta t=\mathbf f(\hat{\mathbf x}_i\boxplus\tilde{\mathbf x}_i,\mathbf u_i,\mathbf w_i)\Delta t$。则误差状态模型 (FASTLIO-5) 重写为：

$$\tilde{\mathbf x}_{i+1}=\underbrace{\big((\hat{\mathbf x}_i\boxplus\tilde{\mathbf x}_i)\boxplus\mathbf g(\tilde{\mathbf x}_i,\mathbf w_i)\big)\boxminus\big(\hat{\mathbf x}_i\boxplus\mathbf g(\mathbf 0,\mathbf 0)\big)}_{\mathbf G(\tilde{\mathbf x}_i,\mathbf g(\tilde{\mathbf x}_i,\mathbf w_i))}.\tag{FASTLIO-22}$$

按偏微分链式法则，(FASTLIO-5) 的矩阵 $\mathbf F_{\tilde{\mathbf x}}$、$\mathbf F_{\mathbf w}$ 计算为：

$$\mathbf F_{\tilde{\mathbf x}}=\left.\left(\frac{\partial\mathbf G(\tilde{\mathbf x}_i,\mathbf g(\mathbf 0,\mathbf 0))}{\partial\tilde{\mathbf x}_i}+\frac{\partial\mathbf G(\mathbf 0,\mathbf g(\tilde{\mathbf x}_i,\mathbf 0))}{\partial\mathbf g(\tilde{\mathbf x}_i,\mathbf 0)}\frac{\partial\mathbf g(\tilde{\mathbf x}_i,\mathbf 0)}{\partial\tilde{\mathbf x}_i}\right)\right|_{\tilde{\mathbf x}_i=\mathbf 0},\tag{FASTLIO-23a}$$
$$\mathbf F_{\mathbf w}=\left.\left(\frac{\partial\mathbf G(\mathbf 0,\mathbf g(\mathbf 0,\mathbf w_i))}{\partial\mathbf g(\mathbf 0,\mathbf w_i)}\frac{\partial\mathbf g(\mathbf 0,\mathbf w_i)}{\partial\mathbf w_i}\right)\right|_{\mathbf w_i=\mathbf 0}.\tag{FASTLIO-23b}$$

（代入 (FASTLIO-3) 的 $\mathbf f$ 与 $\boxplus/\boxminus$ 定义，逐块求导即得 (FASTLIO-7) 的闭式结果。其中姿态块的 $\mathbf A(\cdot)^{-T}$ 来自 $\mathrm{SO}(3)$ 上 $\boxminus$ 的右雅可比逆。）

## §FASTLIO 附录 B：等价 Kalman 增益公式（源 Appendix B，完整证明）

**目标**：证明 (FASTLIO-20) 与 (FASTLIO-18a) 等价。

基于**矩阵求逆引理**（Woodbury 恒等式）[27]：

$$\big(\mathbf P^{-1}+\mathbf H^T\mathbf R^{-1}\mathbf H\big)^{-1}=\mathbf P-\mathbf P\mathbf H^T\big(\mathbf H\mathbf P\mathbf H^T+\mathbf R\big)^{-1}\mathbf H\mathbf P.\tag{B-1}$$

把 (B-1) 代入新增益 (FASTLIO-20)：

$$\begin{aligned}
\mathbf K&=\big(\mathbf H^T\mathbf R^{-1}\mathbf H+\mathbf P^{-1}\big)^{-1}\mathbf H^T\mathbf R^{-1}\\
&=\Big(\mathbf P-\mathbf P\mathbf H^T\big(\mathbf H\mathbf P\mathbf H^T+\mathbf R\big)^{-1}\mathbf H\mathbf P\Big)\mathbf H^T\mathbf R^{-1}\\
&=\mathbf P\mathbf H^T\mathbf R^{-1}-\mathbf P\mathbf H^T\big(\mathbf H\mathbf P\mathbf H^T+\mathbf R\big)^{-1}\mathbf H\mathbf P\mathbf H^T\mathbf R^{-1}.
\end{aligned}$$

注意到 $\mathbf H\mathbf P\mathbf H^T\mathbf R^{-1}=(\mathbf H\mathbf P\mathbf H^T+\mathbf R)\mathbf R^{-1}-\mathbf I$。代入上式：

$$\begin{aligned}
\mathbf K&=\mathbf P\mathbf H^T\mathbf R^{-1}-\mathbf P\mathbf H^T\big(\mathbf H\mathbf P\mathbf H^T+\mathbf R\big)^{-1}\Big[(\mathbf H\mathbf P\mathbf H^T+\mathbf R)\mathbf R^{-1}-\mathbf I\Big]\\
&=\mathbf P\mathbf H^T\mathbf R^{-1}-\Big[\mathbf P\mathbf H^T\mathbf R^{-1}-\mathbf P\mathbf H^T\big(\mathbf H\mathbf P\mathbf H^T+\mathbf R\big)^{-1}\Big]\\
&=\mathbf P\mathbf H^T\big(\mathbf H\mathbf P\mathbf H^T+\mathbf R\big)^{-1}.
\end{aligned}$$

此即标准 Kalman 增益 (FASTLIO-18a)。**证毕。** ∎

（说明：原文证明把中间步写作
$\mathbf K=\mathbf P\mathbf H^T\mathbf R^{-1}-\mathbf P\mathbf H^T\mathbf R^{-1}+\mathbf P\mathbf H^T(\mathbf H\mathbf P\mathbf H^T+\mathbf R)^{-1}=\mathbf P\mathbf H^T(\mathbf H\mathbf P\mathbf H^T+\mathbf R)^{-1}$，
与上式逐项相消一致。）

---

# 第五部分（补充·使前文自包含）

## §补充A：Forster on-manifold IMU 预积分（LIO-SAM (LIOSAM-7~9) 的来源）

> LIO-SAM 的 (LIOSAM-7)~(LIOSAM-9) 与 IMU 预积分因子直接引用 Forster et al. (T-RO 2016)。为使本章自包含，抽其**预积分测量模型、噪声传播、偏置修正、残差**核心公式（保留原编号）。

### IMU 模型（源 §V）
带白噪 $\boldsymbol\eta$ 与缓变偏置 $\mathbf b$：陀螺与加表测量

$$\tilde{\boldsymbol\omega}(t)=\boldsymbol\omega(t)+\mathbf b^g(t)+\boldsymbol\eta^g(t),\qquad\tilde{\mathbf a}(t)=\mathbf R_{WB}^T(t)\big({}_W\mathbf a(t)-{}_W\mathbf g\big)+\mathbf b^a(t)+\boldsymbol\eta^a(t).\tag{F-27}$$

### 预积分测量（源式 33→38）

从两 keyframe $i,j$ 间的运动定义"delta"量。**真值定义**（源式 33）：

$$\Delta\mathbf R_{ij}=\mathbf R_i^T\mathbf R_j=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b^g_k-\boldsymbol\eta^{gd}_k)\Delta t\big),$$
$$\Delta\mathbf v_{ij}=\mathbf R_i^T(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})=\sum_{k=i}^{j-1}\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t,$$
$$\Delta\mathbf p_{ij}=\mathbf R_i^T\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)=\sum_{k=i}^{j-1}\Big[\Delta\mathbf v_{ik}\Delta t+\tfrac12\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t^2\Big].\tag{F-33}$$

**关键洞察**：$\Delta\mathbf R_{ij},\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$ 的右端**独立于 $i$ 时刻状态与重力**，故可直接从两 keyframe 间惯性测量算得（这正是"预积分"省重复积分的核心）。

经一阶近似把噪声移到末端，得**预积分测量模型**（源式 38；记 $\Delta\tilde{\mathbf R}_{ij}=\prod_{k=i}^{j-1}\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)$ 为预积分旋转测量、$\Delta\tilde{\mathbf v}_{ij}=\sum\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)\Delta t$、$\Delta\tilde{\mathbf p}_{ij}$ 同理）：

$$\Delta\tilde{\mathbf R}_{ij}=\mathbf R_i^T\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_{ij}),$$
$$\Delta\tilde{\mathbf v}_{ij}=\mathbf R_i^T(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})+\delta\mathbf v_{ij},$$
$$\Delta\tilde{\mathbf p}_{ij}=\mathbf R_i^T\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)+\delta\mathbf p_{ij}.\tag{F-38}$$

复合测量 = （待估状态的函数）"+"随机噪 $[\delta\boldsymbol\phi_{ij}^T,\delta\mathbf v_{ij}^T,\delta\mathbf p_{ij}^T]^T$。

### 噪声传播（源式 39-43）

$$\boldsymbol\eta^\Delta_{ij}\doteq[\delta\boldsymbol\phi_{ij}^T,\delta\mathbf v_{ij}^T,\delta\mathbf p_{ij}^T]^T\sim\mathcal N(\mathbf 0_{9\times1},\boldsymbol\Sigma_{ij}).\tag{F-39}$$

旋转噪声（源式 42，一阶）：

$$\delta\boldsymbol\phi_{ij}\simeq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}^T\,\mathbf J_r^k\,\boldsymbol\eta^{gd}_k\Delta t.\tag{F-42}$$

速度、位置噪声（源式 43，一阶）：

$$\delta\mathbf v_{ij}\simeq\sum_{k=i}^{j-1}\Big[-\Delta\tilde{\mathbf R}_{ik}\lfloor\tilde{\mathbf a}_k-\mathbf b^a_i\rfloor_\wedge\delta\boldsymbol\phi_{ik}\Delta t+\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t\Big],$$
$$\delta\mathbf p_{ij}\simeq\sum_{k=i}^{j-1}\Big[\delta\mathbf v_{ik}\Delta t-\tfrac12\Delta\tilde{\mathbf R}_{ik}\lfloor\tilde{\mathbf a}_k-\mathbf b^a_i\rfloor_\wedge\delta\boldsymbol\phi_{ik}\Delta t^2+\tfrac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t^2\Big].\tag{F-43}$$

均为零均值高斯（IMU 噪 $\boldsymbol\eta^{gd}_k,\boldsymbol\eta^{ad}_k$ 的线性组合），故 $\boldsymbol\Sigma_{ij}$ 可由 IMU 噪协方差线性传播（附录给出迭代式：新 IMU 测量到达只更新 $\boldsymbol\Sigma_{ij}$ 而非重算）。

### 偏置更新（一阶展开，源式 44）
偏置由 $\mathbf b\leftarrow\bar{\mathbf b}+\delta\mathbf b$ 更新时，用一阶展开更新 delta 测量（避免重积分）：

$$\Delta\tilde{\mathbf R}_{ij}(\mathbf b^g_i)\simeq\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathrm{Exp}\Big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g\Big),$$
$$\Delta\tilde{\mathbf v}_{ij}(\mathbf b^g_i,\mathbf b^a_i)\simeq\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a_i,$$
$$\Delta\tilde{\mathbf p}_{ij}(\mathbf b^g_i,\mathbf b^a_i)\simeq\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a_i.\tag{F-44}$$

Jacobian $\{\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g},\dots\}$ 在预积分时（$\bar{\mathbf b}_i$）预计算、保持常数。

### 预积分 IMU 因子（残差，源式 45）
残差 $\mathbf r_{\mathcal I_{ij}}=[\mathbf r_{\Delta\mathbf R_{ij}}^T,\mathbf r_{\Delta\mathbf v_{ij}}^T,\mathbf r_{\Delta\mathbf p_{ij}}^T]^T\in\mathbb R^9$：

$$\mathbf r_{\Delta\mathbf R_{ij}}=\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathrm{Exp}(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g)\big)^T\mathbf R_i^T\mathbf R_j\Big),$$
$$\mathbf r_{\Delta\mathbf v_{ij}}=\mathbf R_i^T(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-\Big[\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\Big],$$
$$\mathbf r_{\Delta\mathbf p_{ij}}=\mathbf R_i^T\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)-\Big[\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\Big].\tag{F-45}$$

**偏置模型**（源式 46-47）：偏置建模为布朗运动（积分白噪）：$\dot{\mathbf b}^g(t)=\boldsymbol\eta^{bg}$、$\dot{\mathbf b}^a(t)=\boldsymbol\eta^{ba}$；离散 $\mathbf b^g_j=\mathbf b^g_i+\boldsymbol\eta^{bgd}$、$\mathbf b^a_j=\mathbf b^a_i+\boldsymbol\eta^{bad}$。

**MAP 表述**（源式 26 附近）：零均值高斯噪假设下，负对数后验 = 残差平方和

$$\sum\Big(\|\mathbf r_0\|^2+\|\mathbf r_{\mathcal I_{ij}}\|^2_{\boldsymbol\Sigma_{ij}}+\|\mathbf r_{\mathcal C_{il}}\|^2_{\boldsymbol\Sigma_C}\Big),$$

$\mathbf r_0,\mathbf r_{\mathcal I_{ij}},\mathbf r_{\mathcal C_{il}}$ 分别是先验、IMU、相机（视觉）残差。**【与本书对接】** Forster 全程右雅可比 $\mathbf J_r$、$\mathrm{SO}(3)$ 上 $\mathrm{Exp/Log}$，与本书右扰动主线一致——本章 LIO-SAM 因子图即建在此基础上。

## §补充B：Bogoslavskyi-Stachniss range image 分割（LeGO-LOAM 分割[23]的来源）

> LeGO-LOAM 的图像分割 [23] 即本文。抽其角度 β 判据与算法（保留原编号）。

**核心思想**：先去地面，再在 range image 上基于角度 β 做连通域聚类，把同物体的点合为一簇。

**角度 β 定义**（源）。设两激光束端点 A、B，range 测量分别为 $d_1,d_2$，$\alpha$ 是两束间已知夹角（来自扫描仪文档）。定义 β 为激光束与**连接 A、B 的线段**间的夹角（在 x-y 平面投影；A 是离扫描仪较远者）：

$$\beta=\arctan\frac{\|BH\|}{\|HA\|}=\arctan\frac{d_2\sin\alpha}{d_1-d_2\cos\alpha},\tag{Bogo-β}$$

其中 $H$ 是 B 到 OA 的垂足。

**判据**：β 对大多数物体**较大**，仅当相邻点的深度差远大于其图像平面位移（由角分辨率定义）时才小。设阈值 θ：
- 若 $\beta>\theta$，两点视为**同一物体**（合并为一簇）；
- 若 $\beta<\theta$，深度变化太大，**分为不同簇**。

**失败情形**（源）：扫描仪靠近墙时，墙上远点的 β 小，可能把墙分成多段（即近似平行束方向的墙难判）；但实验表明此情形罕见、通常只导致倾斜平面的过分割。

**连通域算法**（源 Algorithm 1，逐行）：

> **Algorithm 1: Range Image Labeling**
> 1. **procedure** LabelRangeImage
> 2. &nbsp;&nbsp;$Label\leftarrow1$, $R\leftarrow$ range image
> 3. &nbsp;&nbsp;$L\leftarrow\mathrm{zeros}(R_{rows}\times R_{cols})$
> 4. &nbsp;&nbsp;**for** $r=1\dots R_{rows}$ **do**
> 5. &nbsp;&nbsp;&nbsp;&nbsp;**for** $c=1\dots R_{cols}$ **do**
> 6. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**if** $L(r,c)=0$ **then**
> 7. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;LabelComponentBFS($r,c,Label$);
> 8. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$Label\leftarrow Label+1$;
> 9. **procedure** LabelComponentBFS($r,c,Label$)
> 10. &nbsp;&nbsp;queue.push($\{r,c\}$)
> 11. &nbsp;&nbsp;**while** queue is not empty **do**
> 12. &nbsp;&nbsp;&nbsp;&nbsp;$\{r,c\}\leftarrow$ queue.top()
> 13. &nbsp;&nbsp;&nbsp;&nbsp;$L(r,c)\leftarrow Label$
> 14. &nbsp;&nbsp;&nbsp;&nbsp;**for** $\{r_n,c_n\}\in\mathrm{Neighborhood}\{r,c\}$ **do**
> 15. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$d_1\leftarrow\max(R(r,c),R(r_n,c_n))$
> 16. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;$d_2\leftarrow\min(R(r,c),R(r_n,c_n))$
> 17. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**if** $\arctan\dfrac{d_2\sin\alpha}{d_1-d_2\cos\alpha}>\theta$ **then**
> 18. &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;queue.push($\{r_n,c_n\}$)

复杂度 $O(N)$（$N$ = 像素/range 读数数），每像素最多访问两次。N4 邻域（左右上下）；处理顺序使上邻无需重复考虑。单参数 θ 有物理意义、调参少。**地面去除**（源）：用扫描仪在车上的已知安装高度与朝向快速估地面平面（作为 [13],[19] 地面估计的近似），从 range image 去地面后再分割。

---

# 附：家族脉络与本章组织建议（供综合 agent）

**演进逻辑（一条主线）**：
1. **LOAM (2014)**：奠基。曲率 (LOAM-1) 分边/面特征；点到线 (LOAM-2)、点到面 (LOAM-3) 距离；匀速运动 + 线性插值去畸变 (LOAM-4)；LM 求解 (LOAM-12)。**双算法**：odometry（10 Hz, scan-to-scan）+ mapping（1 Hz, scan-to-map）。无回环、IMU 仅松耦合预处理。
2. **LeGO-LOAM (2018)**：轻量化 + 地面优化。range image 分割（去地面 + 角度 β 聚类 + 滤小簇）；两级特征；**两步 LM**（地面平面→$[t_z,\theta_{roll},\theta_{pitch}]$、边缘→$[t_x,t_y,\theta_{yaw}]$）；引入**位姿图 + ICP 回环 + iSAM2**。
3. **LIO-SAM (2020)**：紧耦合 + 因子图。四类因子（IMU 预积分 / lidar 里程计 / GPS / 回环）；Forster 预积分；keyframe + sub-keyframe 滑窗 scan-to-局部地图；iSAM2 增量平滑；欧氏回环。
4. **FAST-LIO (2021)**：紧耦合 + 迭代 ESKF。$\boxplus/\boxminus$ 流形；前向 (FASTLIO-8) + 反向 (FASTLIO-9) 传播去畸变；迭代 EKF 的 MAP (FASTLIO-17)→更新 (FASTLIO-18)；**新增益公式 (FASTLIO-20)**（状态维复杂度，附录 B 证等价）。

**关键对比维度**（建议综合时做对照表）：
| 维度 | LOAM | LeGO-LOAM | LIO-SAM | FAST-LIO |
|---|---|---|---|---|
| 耦合 | 松（IMU 仅预处理） | 松 | **紧**（因子图） | **紧**（迭代 ESKF） |
| 后端 | 双 LM | 双 LM + 位姿图 | iSAM2 因子图 | 迭代 EKF |
| 去畸变 | 匀速线性插值 | 同 LOAM | IMU 预积分 | 前向+反向传播 |
| scan 匹配 | scan-to-scan + scan-to-map | + 标签匹配 + 两步 | scan-to-局部 sub-keyframe | scan-to-map（迭代） |
| 回环 | 无 | ICP + iSAM2 | 欧氏距离 + 因子图 | 无（里程计） |
| 绝对测量 | 无 | 无 | GPS 因子 | 无 |
| 特征 | 边/面（3D 坐标曲率） | 边/面（range 曲率, 分割） | 边/面 | 边/面 |

**与本书章节衔接**：
- FAST-LIO 的迭代 ESKF 直接复用本书"Kalman/ESKF"章的 ESKF 框架与"李群"章的 $\boxplus/\boxminus$、右扰动、$\mathbf A(\cdot)^{-1}$（即 $\mathbf J_r^{-1}$ 闭式）。
- LIO-SAM/LeGO-LOAM 的回环+位姿图直接复用"非线性优化/因子图"章的 Gauss-Newton、iSAM2/Bayes 树、Schur 边缘化。
- Forster 预积分可与本书"惯性导航/预积分"专题合并（VINS-Mono 同源）。
- **记号统一提醒**：移植 FAST-LIO 状态时把"旋转在前"重排为本书"平移在前"$\boxminus$ 顺序；姿态误差 $\delta\boldsymbol\theta$ 即本书右扰动 $\delta\boldsymbol\phi$（无需翻转方向）；测量噪声 $\mathbf R$ 与旋转 $\mathbf R\in\mathrm{SO}(3)$ 同字母须区分。
