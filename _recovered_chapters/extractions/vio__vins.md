# 抽取留痕：VINS-Mono(qin2018vins) 完整 + VIO 两条主线（优化/滤波）与可观性

> **主源**：Tong Qin, Peiliang Li, Shaojie Shen, *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*, IEEE Transactions on Robotics (TRO), vol. 34, no. 4, pp. 1004–1020, 2018；预印本 **arXiv:1708.03852v1 [cs.RO], 13 Aug 2017**（17 页）。键名 `qin2018vins`。
> **PDF**：https://arxiv.org/pdf/1708.03852 ；**摘要页**：https://arxiv.org/abs/1708.03852 ；**官方代码**：https://github.com/HKUST-Aerial-Robotics/VINS-Mono （PC/ROS），https://github.com/HKUST-Aerial-Robotics/VINS-Mobile （iOS）。
>
> **辅源（为满足本章「两条主线完整推导并对比 + 可观性 + Schur 边缘化」聚焦而引入，均权威）**：
> - **[MSCKF]** A. I. Mourikis, S. I. Roumeliotis, *A Multi-State Constraint Kalman Filter for Vision-Aided Inertial Navigation*, Proc. IEEE ICRA 2007, pp. 3565–3572. PDF: https://www-users.cse.umn.edu/~stergios/papers/ICRA07-MSCKF.pdf （滤波主线，VINS 论文文献 [13]）。
> - **[Forster]** C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, *IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation*, RSS 2015；扩展版 *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, IEEE TRO 2017, arXiv:1512.02363. PDF: https://www.roboticsproceedings.org/rss11/p06.pdf （VINS 论文文献 [23]，VINS 的「后验偏置校正」直接基于它；其 structureless 视觉因子的零空间消元与 MSCKF 零空间投影是同一思想）。
> - **[Huang-Rev]** G. Huang, *Visual-Inertial Navigation: A Concise Review*, ICRA 2019 / arXiv:1906.02650. PDF: https://arxiv.org/pdf/1906.02650 （VIO 可观性 4 个不可观方向、退化运动、松/紧耦合、滤波/优化、FEJ）。
> - **[Schur-Marg]** 标准 Schur 补边缘化代数（Sibley et al. 2010「Sliding window filter」= VINS 文献 [39]；Dong-Si & Mourikis ICRA 2012；OKVIS Leutenegger et al. 2014）。本文件 §VI.D' 给出自包含的完整推导。
>
> **服务章节**：视觉惯性里程计 VIO。**本章聚焦**（须重点覆盖）：① VIO 问题与可观性；② 松/紧耦合；③ 基于优化(VINS 滑窗+边缘化) 与 基于滤波(MSCKF) 两条主线完整推导并对比；④ 初始化。
>
> **抽取范围**：VINS-Mono 全文（正文 §I–§X + 全部公式/图/算法/实验）逐节、逐式、逐证保真抽取；并补抽 MSCKF 全推导、Forster 预积分理论全推导、可观性 4 方向 + 退化运动、Schur 边缘化完整代数。
>
> **抽取人备注**：本文件是【内部抽取留痕】，不是成书正文；遵循「禁摘要·全量保真」铁律——每步推导不跳、每式 LaTeX 写全、保留所有式号与条件，并显式记录记号约定与本书统一约定的差异（见 §0）。VINS 论文本身在若干处「不证已假定」（如视觉残差雅可比、IMU 误差态 F 阵全闭式只给框架、Schur 补只点名），这些缺口由辅源补齐并在 §0.3「源覆盖与缺口」逐条标注。

---

## §0 记号约定（各源）与本书统一约定的差异

### §0.1 VINS-Mono 记号（源 §III「OVERVIEW」末段 + 全文）

> 注意：arXiv v1 正文的章号与某些二级目录工具给出的不同。**以 PDF 正文为准**：§I Introduction、§II Related Work、§III Overview、§IV Measurement Preprocessing、§V Estimator Initialization、§VI Tightly-coupled Monocular VIO、§VII Relocalization、§VIII Global Pose Graph Optimization、§IX Experimental Results、§X Conclusion。本文件沿用 PDF 正文章号。

| 项目 | VINS-Mono 本源约定 | 本书统一约定 | 差异/转换说明 |
|---|---|---|---|
| 世界系 | $(\cdot)^w$，**重力方向对齐 $w$ 系的 $z$ 轴**，$\mathbf g^w=[0,0,g]^\top$ | $(\cdot)^w$ | 一致。 |
| 体/IMU 系 | $(\cdot)^b$，**body 系定义为与 IMU 系相同**；$b_k$=取第 $k$ 帧图像时的 body 系 | $(\cdot)^b$ | 一致。 |
| 相机系 | $(\cdot)^c$；$c_k$=取第 $k$ 帧图像时的相机系 | $(\cdot)^c$ | 一致。 |
| 旋转表示 | **同时用旋转矩阵 $\mathbf R$ 和 Hamilton 四元数 $\mathbf q$**；状态向量里主用四元数，旋转 3D 向量时用 $\mathbf R$ | $\mathbf R\in\mathrm{SO}(3)$，Hamilton 四元数 | **一致**（VINS 明确用 Hamilton，且 $\mathbf R$ 记号一致）。 |
| 位姿下标语义 | $\mathbf q^w_b,\ \mathbf p^w_b$ = body→world 的旋转与平移（即把 body 系坐标变到 world） | $\mathbf T_{wb}$ = body→world | **语义一致**：$\mathbf q^w_b$ 对应 $\mathbf R_{wb}$。 |
| 四元数乘 | $\otimes$（Hamilton 乘法） | $\otimes$ | 一致。 |
| 旋转误差（**关键**） | 四元数过参数化，误差定义为**右乘小扰动**：$\boldsymbol\gamma^{b_k}_t\approx\hat{\boldsymbol\gamma}^{b_k}_t\otimes\begin{bmatrix}1\\\tfrac12\delta\boldsymbol\theta^{b_k}_t\end{bmatrix}$（式 8），$\delta\boldsymbol\theta\in\mathbb R^3$ | **右扰动为主** $\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$ | **一致**：VINS 用四元数右乘小扰动 $\delta\boldsymbol\theta$，等价于本书右扰动 $\delta\boldsymbol\phi$（局部坐标）。符号差：VINS 用 $\delta\boldsymbol\theta$，本书 $\delta\boldsymbol\phi$。 |
| 预积分量 | $\boldsymbol\alpha^{b_k}_{b_{k+1}}$（位置增量）、$\boldsymbol\beta^{b_k}_{b_{k+1}}$（速度增量）、$\boldsymbol\gamma^{b_k}_{b_{k+1}}$（旋转增量，**四元数**）。带 $\hat{(\cdot)}$ 为估计/含噪测量值 | — | VINS 的旋转增量 $\boldsymbol\gamma$ 是**四元数**；Forster 用旋转矩阵 $\Delta\mathbf R$。转换见 §IV.B 与 §Forster。 |
| 噪声分布 | $\mathbf n_a\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_a^2)$、$\mathbf n_w\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_w^2)$；偏置随机游走 $\mathbf n_{b_a}\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_{b_a}^2)$、$\mathbf n_{b_w}\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_{b_w}^2)$ | $\mathcal N(\boldsymbol\mu,\boldsymbol\Sigma)$ | 一致。$b_a$=加速度计偏置，$b_w$=陀螺偏置。 |
| 协方差字母 | 预积分协方差 $\mathbf P^{b_k}_{b_{k+1}}$；噪声对角协方差 $\mathbf Q=\mathrm{diag}(\boldsymbol\sigma_a^2,\boldsymbol\sigma_w^2,\boldsymbol\sigma_{b_a}^2,\boldsymbol\sigma_{b_w}^2)$；雅可比 $\mathbf J$ | $\mathbf P$/$\mathbf Q$/$\mathbf R$ | VINS 用 $\mathbf P$ 表预积分协方差、$\mathbf Q$ 表 IMU 噪声谱，**不用 $\mathbf R$ 表测量噪声**（$\mathbf R$ 留给旋转矩阵）。视觉测量协方差直接记 $\mathbf P^{c_j}_l$。 |
| 逆深度 | 特征用**逆深度** $\lambda_l$ 参数化（第 $l$ 个特征在其首次观测帧的逆深度） | — | VINS 用逆深度（XYZ 替代）；本书因子图章可并列。 |
| 反对称算子 | $\lfloor\boldsymbol\omega\rfloor_\times$（式 4 写作 $\lfloor\cdot\rfloor_\times$） | $[\cdot]_\times$ 或 $(\cdot)^\wedge$ | 仅记号风格差异。 |
| 取四元数向量部 | $(\cdot)_{xyz}$：从四元数取虚部（3 维），用于误差态 | — | VINS 专用记号。 |

### §0.2 MSCKF 记号（辅源，源 §III）与差异

| 项目 | MSCKF 本源约定 | 本书统一约定 | 差异/转换 |
|---|---|---|---|
| 四元数 flavor | **JPL 约定**（Trawny-Roumeliotis），分量序虚部在前 $\bar{\mathbf q}=[q_x,q_y,q_z,q_w]^\top$，左手复合/被动 | **Hamilton**（虚部概念在前但实部存储，Eigen 兼容） | **重大差异**：MSCKF 用 JPL，四元数乘法顺序与 Hamilton 相反，$\mathbf R(\bar{\mathbf q})$ 是 Hamilton 的转置。引用时须做 JPL↔Hamilton 转换（见本书四元数章；规则：$\mathbf R_{JPL}(\bar{\mathbf q})=\mathbf R_{Ham}(\mathbf q)^\top$，且 $\otimes$ 顺序反转）。 |
| 旋转矩阵记号 | $C(\cdot)$（方向余弦矩阵），${}^I_G\bar{\mathbf q}$ = G→I 旋转，$C({}^I_G\bar{\mathbf q})$ 把 global 向量转到 IMU 系 | $\mathbf R$ | $C\to\mathbf R$。注意 MSCKF 的 ${}^I_G\bar{\mathbf q}$ 方向是 global→IMU（与 VINS 的 body→world 相反）。 |
| 姿态误差 | $\delta\bar{\mathbf q}\approx[\tfrac12\delta\boldsymbol\theta^\top,\ 1]^\top$（式 3），$\delta\boldsymbol\theta\in\mathbb R^3$ 最小表示 | $\delta\boldsymbol\phi$ | 概念一致（3 维角误差），但因 JPL 是**左乘/global 误差**性质，方向与本书右扰动相反。综合时按「左扰动并列」处理（同 Solà §7 的 global 误差）。 |
| IMU 误差态序 | $\tilde{\mathbf X}_{IMU}=[\delta\boldsymbol\theta_I^\top,\ \tilde{\mathbf b}_g^\top,\ {}^G\tilde{\mathbf v}_I^\top,\ \tilde{\mathbf b}_a^\top,\ {}^G\tilde{\mathbf p}_I^\top]^\top$（15 维，式 5） | — | 顺序 [姿态;陀螺偏置;速度;加计偏置;位置]，与 VINS 的 [位置;速度;姿态;$b_a$;$b_g$] 不同。综合时统一到本书序。 |
| 协方差 | $\mathbf P_{k|k}$（先验/后验），$\mathbf Q_{IMU}$（IMU 过程噪声），$\boldsymbol\Phi$（状态转移），测量噪声 $\sigma_{im}^2$ | $\mathbf P/\mathbf Q/\mathbf R$ | 一致风格；测量噪声标量 $\sigma_{im}^2$（像素噪声）。 |
| 重力 | ${}^G\mathbf g$（global 系重力，含进入加计测量） | $\mathbf g^w$ | 一致；MSCKF 还含地球自转 $\boldsymbol\omega_G$（VINS/Forster 忽略）。 |

### §0.3 Forster 记号（辅源）与差异

| 项目 | Forster 本源约定 | 本书统一约定 | 差异/转换 |
|---|---|---|---|
| 旋转 | $\mathbf R\in\mathrm{SO}(3)$，$\mathbf R_{WB}$ = body→world | $\mathbf R$ | 一致。 |
| 四元数 | **不用四元数，全程 $\mathrm{SO}(3)$ + 右雅可比 $\mathbf J_r$** | — | 与 VINS（四元数）互补；本书可两讲法并列。 |
| Exp/Log | $\mathrm{Exp}:\mathbb R^3\to\mathrm{SO}(3)$，$\mathrm{Log}:\mathrm{SO}(3)\to\mathbb R^3$（大写）；右雅可比 $\mathbf J_r(\boldsymbol\phi)$（式 7,8,9） | 同（Exp/Log 大写、$\mathbf J_r$ 为主） | **完全一致**（本书与 Forster 都右雅可比主线）。 |
| 旋转扰动 | **右乘** $\tilde{\mathbf R}=\mathbf R\,\mathrm{Exp}(\boldsymbol\epsilon)$，$\boldsymbol\epsilon\sim\mathcal N(\mathbf 0,\boldsymbol\Sigma)$（式 12） | **右扰动** | **一致**。 |
| 预积分量 | $\Delta\mathbf R_{ij},\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$（旋转用矩阵） | — | 对应 VINS 的 $\boldsymbol\gamma,\boldsymbol\beta,\boldsymbol\alpha$（旋转 VINS 用四元数）。 |
| 噪声 | $\boldsymbol\eta^{gd},\boldsymbol\eta^{ad}$（离散陀螺/加计噪声），$\boldsymbol\eta^{bg},\boldsymbol\eta^{ba}$（偏置随机游走） | $\mathbf n$ | 命名差异。离散↔连续：$\mathrm{Cov}(\boldsymbol\eta^{gd})=\tfrac1{\Delta t}\mathrm{Cov}(\boldsymbol\eta^g)$。 |

### §0.4 三源覆盖与缺口对照（综合 agent 写章时据此取材）

| 本章聚焦要素 | VINS-Mono 提供？ | 缺口由谁补 |
|---|---|---|
| IMU 预积分定义 $\alpha/\beta/\gamma$ | ✔ 完整（式 5–13） | — |
| 预积分协方差/雅可比递推 | ✔ 给框架（式 9–12），但 **F 阵的四元数误差块、bias 雅可比闭式未全展开** | **Forster** 给 $\mathrm{SO}(3)$ 上完整 A/B 阵、bias 雅可比解析式（§Forster.B,C） |
| 视觉惯性对齐与初始化 | ✔ 完整（式 14–20 + 算法 1） | — |
| 滑窗紧耦合 BA（IMU/视觉残差） | ✔ 完整（式 21–25） | 视觉残差**雅可比**未给（VINS 称「detailed 略」），可由 Forster 式 42–44 / 本书 BA 章补 |
| 边缘化 Schur 补 | ✘ **只点名 [39]**，无代数 | **§VI.D'**（本文件自含完整推导）+ Schur-Marg 辅源 |
| 回环 + 4-DOF 位姿图 | ✔ 完整（式 26–29） | — |
| **滤波主线 MSCKF** | ✘ 仅 §II 一句话提及 | **MSCKF 全推导**（§MSCKF，式 1–31） |
| **VIO 可观性（4 不可观方向 + 退化）** | ✘ 仅 §X 提「未来工作研究可观性」 | **Huang-Rev**（§Obs，式 18 + 退化运动表） |
| 松/紧耦合、滤波 vs 优化 对比 | ◑ §II 定性 | **Huang-Rev §3**（§Compare）补全 |

---

# 第一部分：VINS-Mono 主源逐节抽取

## §I 引言（源 §I）

**问题与动机**：单目视觉惯性系统（VINS）= 相机 + 低成本 IMU，是度量级 6-DOF 状态估计的**最小传感器套件**。优点：度量尺度、roll/pitch 角全部可观；IMU 可在视觉跟踪丢失（光照变化、无纹理、运动模糊）时桥接。代价/挑战（**反面，"不这样会怎样"**）：

- 单目 VINS 中，**尺度可观需要加速度激励** → 估计器**不能从静止起步**，而是从未知运动状态启动 → 初始化是最脆弱环节。
- 视觉惯性系统高度非线性 → 初始化困难。
- 两传感器 → 相机-IMU 外参标定关键。
- 为在可接受窗口内消除长期漂移 → 需完整系统：VIO + 回环检测 + 重定位 + 全局优化。

**贡献（源 §I 列表）**：
1. 鲁棒初始化，可从未知初始状态自举（bootstrap）。
2. 紧耦合、基于优化的单目 VIO，**在线**做相机-IMU 外参标定与 IMU 偏置估计。
3. 在线回环检测 + 紧耦合重定位。
4. 四自由度（4-DOF）全局位姿图优化。
5. 无人机导航 / 大尺度定位 / 移动 AR 的实时性能演示。
6. 开源（PC+ROS 与 iOS）。

**关键洞察（源 §I）**：因单目 VIO 使 roll/pitch 可观，**漂移仅发生在 4-DOF**（3D 平移 + 绕重力轴的偏航 yaw）→ 故位姿图只需在最小 4-DOF 下优化。

## §II 相关工作（源 §II，分类骨架——本章「松/紧耦合、滤波/优化」分类的源头）

**视觉惯性融合分类**（源 §II 第二段）：
- **松耦合（loosely-coupled）** [11],[12]：IMU 作为独立模块辅助由视觉 SfM 得到的纯视觉位姿。融合常用 EKF：IMU 做状态传播，纯视觉位姿做更新。
- **紧耦合（tightly-coupled）**：相机与 IMU 测量从**原始测量层**联合优化，又分两支：
  - **基于 EKF（滤波）** [13]–[15]：代表 **MSCKF** [13],[14]——在状态向量里维护若干历史相机位姿，用同一特征在多视图的观测构成多约束更新。**SR-ISWF** [18],[19] 是 MSCKF 的扩展，用平方根形式 [20] 实现单精度、避免数值病态，用逆滤波做迭代再线性化 → 等价于优化方法。
  - **基于图优化（优化）** [7],[8],[16],[17]：批量图优化/BA 维护并优化所有测量。为常数处理时间，主流图优化 VIO [8],[16],[17] 在**有界滑窗**上优化，**边缘化（marginalize）**过去状态与测量。

**视觉测量处理分类**（源 §II）：
- **直接法（direct）** [2],[3],[21]：最小化**光度误差**；需好初值（吸引域小）；易扩展稠密建图。
- **间接法（indirect）** [8],[14],[16]：最小化**几何位移**（重投影）；需额外算力提特征/匹配；工程部署更成熟鲁棒。VINS 属间接法。

**IMU 高频测量处理**（源 §II，预积分史）：
- 最直接：EKF 里用 IMU 做状态传播 [11],[13]。
- 图优化里：**IMU 预积分（pre-integration）** 避免重复重积分。首次提出 [22]（**用欧拉角参数化旋转误差**）。**流形上（on-manifold）旋转的预积分**由作者前作 [7] 提出（用连续时间 IMU 误差态动力学推协方差传播，**但忽略 IMU 偏置**）。预积分理论进一步被 [23]（**Forster**）改进——**加后验 IMU 偏置校正**。**本文（VINS）扩展 [7] 并纳入 IMU 偏置校正**。

**初始化史**（源 §II）：
- 线性初始化 [8],[24] 利用短期 IMU 预积分的相对旋转——但**不建模陀螺偏置**、不建模原始投影方程的传感器噪声 → 特征远时初始化不可靠。
- 闭式解 [25]（Martinelli）；加陀螺偏置标定的扩展 [26]——但靠长时间 IMU 二重积分，**不建模惯性积分不确定度**。
- 基于 SVO 的再初始化/失败恢复 [27]——松耦合，**需额外朝下测距传感器**恢复尺度。
- 基于 ORB-SLAM 的初始化 [17]——估尺度/重力方向/速度/IMU 偏置供视觉惯性全 BA，**但尺度收敛可能 >10 秒**。

**回环/位姿图史**（源 §II）：ORB-SLAM [4] 用 BoW [6] 闭环；闭环后做 **7-DOF**（位置+姿态+尺度）位姿图 [28]。**对单目 VINS，因 IMU 加入，漂移仅 4-DOF**（3D 平移 + 绕重力的 yaw）→ 本文选最小 4-DOF 位姿图。

## §III 系统概览（源 §III）

系统流水线（源 Fig. 2）：
1. **测量预处理（§IV）**：提取并跟踪特征；预积分两连续帧间 IMU。
2. **初始化（§V）**：提供位姿、速度、重力向量、陀螺偏置、3D 特征位置，自举后续非线性优化 VIO。
3. **VIO（§VI）+ 重定位（§VII）**：紧耦合融合预积分 IMU、特征观测、回环重检测特征。
4. **全局位姿图优化（§VIII）**：取几何验证的重定位结果，全局优化消漂移。

VIO、重定位、位姿图优化**并发多线程**运行，各模块不同运行率与实时保证。

**记号与坐标系定义**（源 §III 末，已并入 §0.1）。

## §IV 测量预处理（源 §IV）

### §IV.A 视觉前端（源 §IV.A）

- 每来一新图，用 **KLT 稀疏光流** [29] 跟踪已有特征。
- 同时检测新角点 [30]（Shi-Tomasi "Good features to track"），维持每图 **100–300** 个特征；检测器设最小像素间隔，强制特征均匀分布。
- 2D 特征**先去畸变，再投影到单位球**（outlier 剔除后）。
- Outlier 剔除：**RANSAC + 基础矩阵模型** [31]。
- **关键帧选择**两准则：
  1. **平均视差**：跟踪特征在当前帧与最新关键帧间的平均视差超阈值 → 新关键帧。**注意旋转也产生视差，但纯旋转下特征不能三角化** → 用**陀螺短期积分补偿旋转**后再算视差（此旋转补偿**仅用于关键帧选择，不进入 VINS 旋转计算**；故即使陀螺噪声大/有偏，只导致次优关键帧选择，不直接影响估计质量）。
  2. **跟踪质量**：跟踪特征数低于阈值 → 新关键帧（避免特征完全丢失）。

### §IV.B IMU 预积分（源 §IV.B）——本章 IMU 预积分核心

**IMU 测量模型**（式 1）。原始陀螺、加速度计测量 $\hat{\boldsymbol\omega}$、$\hat{\mathbf a}$：
$$
\hat{\mathbf a}_t = \mathbf a_t + \mathbf b_{a_t} + \mathbf R^w_t{}^\top\!\mathbf g^w + \mathbf n_a,\qquad
\hat{\boldsymbol\omega}_t = \boldsymbol\omega_t + \mathbf b_{w_t} + \mathbf n_w. \tag{1}
$$
（IMU 测量在 body 系，混合了"抵抗重力的力 + 平台动力学"，并受加计偏置 $\mathbf b_a$、陀螺偏置 $\mathbf b_w$、加性噪声影响。这里 $\mathbf R^w_t$ 是 body→world，故 $\mathbf R^w_t{}^\top\mathbf g^w$ 是 world 重力投到 body 系。）噪声假设高斯：$\mathbf n_a\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_a^2)$、$\mathbf n_w\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_w^2)$。偏置随机游走（式 2）：
$$
\dot{\mathbf b}_{a_t}=\mathbf n_{b_a},\qquad \dot{\mathbf b}_{w_t}=\mathbf n_{b_w},\qquad \mathbf n_{b_a}\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_{b_a}^2),\ \mathbf n_{b_w}\sim\mathcal N(\mathbf 0,\boldsymbol\sigma_{b_w}^2). \tag{2}
$$

**world 系状态传播**（式 3）。给两图像帧时刻 $b_k,b_{k+1}$，区间 $[t_k,t_{k+1}]$ 内位置、速度、姿态在 world 系传播：
$$
\begin{aligned}
\mathbf p^w_{b_{k+1}} &= \mathbf p^w_{b_k} + \mathbf v^w_{b_k}\Delta t_k + \iint_{t\in[t_k,t_{k+1}]}\!\big(\mathbf R^w_t(\hat{\mathbf a}_t-\mathbf b_{a_t}-\mathbf n_a)-\mathbf g^w\big)\,dt^2,\\
\mathbf v^w_{b_{k+1}} &= \mathbf v^w_{b_k} + \int_{t\in[t_k,t_{k+1}]}\!\big(\mathbf R^w_t(\hat{\mathbf a}_t-\mathbf b_{a_t}-\mathbf n_a)-\mathbf g^w\big)\,dt,\\
\mathbf q^w_{b_{k+1}} &= \mathbf q^w_{b_k}\otimes\int_{t\in[t_k,t_{k+1}]}\!\tfrac12\,\boldsymbol\Omega(\hat{\boldsymbol\omega}_t-\mathbf b_{w_t}-\mathbf n_w)\,\mathbf q^{b_k}_t\,dt,
\end{aligned}\tag{3}
$$
其中（式 4）
$$
\boldsymbol\Omega(\boldsymbol\omega)=\begin{bmatrix}-\lfloor\boldsymbol\omega\rfloor_\times & \boldsymbol\omega\\ -\boldsymbol\omega^\top & 0\end{bmatrix},\qquad
\lfloor\boldsymbol\omega\rfloor_\times=\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix}, \tag{4}
$$
$\Delta t_k$ 是区间 $[t_k,t_{k+1}]$ 时长。$\boldsymbol\Omega(\boldsymbol\omega)$ 是四元数运动学的右乘矩阵（$\dot{\mathbf q}=\tfrac12\boldsymbol\Omega(\boldsymbol\omega)\mathbf q$）。

**预积分的动机（反面）**（源 §IV.B）：式 3 的 IMU 状态传播需要帧 $b_k$ 的旋转/位置/速度。**当这些起始状态改变（优化中每次调位姿），就要重新传播 IMU** → 计算昂贵。为避免重传播 → 采用预积分。

**预积分：换参考系 world→local $b_k$**（式 5）。左乘 $\mathbf R^{b_k}_w=\mathbf R^w_{b_k}{}^\top$，把只与 $\hat{\mathbf a},\hat{\boldsymbol\omega}$ 有关的部分隔离：
$$
\begin{aligned}
\mathbf R^{b_k}_w\mathbf p^w_{b_{k+1}} &= \mathbf R^{b_k}_w\big(\mathbf p^w_{b_k}+\mathbf v^w_{b_k}\Delta t_k-\tfrac12\mathbf g^w\Delta t_k^2\big)+\boldsymbol\alpha^{b_k}_{b_{k+1}},\\
\mathbf R^{b_k}_w\mathbf v^w_{b_{k+1}} &= \mathbf R^{b_k}_w\big(\mathbf v^w_{b_k}-\mathbf g^w\Delta t_k\big)+\boldsymbol\beta^{b_k}_{b_{k+1}},\\
\mathbf q^{b_k}_w\otimes\mathbf q^w_{b_{k+1}} &= \boldsymbol\gamma^{b_k}_{b_{k+1}},
\end{aligned}\tag{5}
$$
其中预积分项定义（式 6）：
$$
\boxed{\;
\begin{aligned}
\boldsymbol\alpha^{b_k}_{b_{k+1}} &= \iint_{t\in[t_k,t_{k+1}]}\mathbf R^{b_k}_t(\hat{\mathbf a}_t-\mathbf b_{a_t}-\mathbf n_a)\,dt^2,\\
\boldsymbol\beta^{b_k}_{b_{k+1}} &= \int_{t\in[t_k,t_{k+1}]}\mathbf R^{b_k}_t(\hat{\mathbf a}_t-\mathbf b_{a_t}-\mathbf n_a)\,dt,\\
\boldsymbol\gamma^{b_k}_{b_{k+1}} &= \int_{t\in[t_k,t_{k+1}]}\tfrac12\,\boldsymbol\Omega(\hat{\boldsymbol\omega}_t-\mathbf b_{w_t}-\mathbf n_w)\,\boldsymbol\gamma^{b_k}_t\,dt.
\end{aligned}\;}\tag{6}
$$

**核心性质（源 §IV.B）**：预积分项 (6) **只用 IMU 测量、以 $b_k$ 为参考系即可得到**；$\boldsymbol\alpha^{b_k}_{b_{k+1}},\boldsymbol\beta^{b_k}_{b_{k+1}},\boldsymbol\gamma^{b_k}_{b_{k+1}}$ **只与 IMU 偏置有关，与 $b_k,b_{k+1}$ 的其他状态无关**。当偏置估计改变：若改变小，用**关于偏置的一阶近似**调整（见式 12），否则重新传播 → 为优化省大量算力。

**离散实现（欧拉积分，源用欧拉示范，代码用中点）**（式 7）。起始 $\boldsymbol\alpha^{b_k}_{b_k}=\mathbf 0,\boldsymbol\beta^{b_k}_{b_k}=\mathbf 0,\boldsymbol\gamma^{b_k}_{b_k}=$ 单位四元数。加性噪声 $\mathbf n_a,\mathbf n_w$ 未知、实现中置零 → 得估计值（标 $\hat{(\cdot)}$）：
$$
\begin{aligned}
\hat{\boldsymbol\alpha}^{b_k}_{i+1} &= \hat{\boldsymbol\alpha}^{b_k}_{i}+\hat{\boldsymbol\beta}^{b_k}_{i}\delta t+\tfrac12\,\mathbf R(\hat{\boldsymbol\gamma}^{b_k}_{i})(\hat{\mathbf a}_i-\mathbf b_{a_i})\,\delta t^2,\\
\hat{\boldsymbol\beta}^{b_k}_{i+1} &= \hat{\boldsymbol\beta}^{b_k}_{i}+\mathbf R(\hat{\boldsymbol\gamma}^{b_k}_{i})(\hat{\mathbf a}_i-\mathbf b_{a_i})\,\delta t,\\
\hat{\boldsymbol\gamma}^{b_k}_{i+1} &= \hat{\boldsymbol\gamma}^{b_k}_{i}\otimes\begin{bmatrix}1\\ \tfrac12(\hat{\boldsymbol\omega}_i-\mathbf b_{w_i})\,\delta t\end{bmatrix},
\end{aligned}\tag{7}
$$
$i$ 是 $[t_k,t_{k+1}]$ 内某 IMU 测量对应的离散时刻，$\delta t$ 是相邻 IMU 测量 $i,i+1$ 的时间间隔。

**协方差传播**（式 8–10）。四维旋转四元数 $\boldsymbol\gamma^{b_k}_t$ 过参数化，定义其误差为绕均值的扰动（式 8，**右乘小扰动——与本书右扰动一致**）：
$$
\boldsymbol\gamma^{b_k}_t\approx\hat{\boldsymbol\gamma}^{b_k}_t\otimes\begin{bmatrix}1\\ \tfrac12\delta\boldsymbol\theta^{b_k}_t\end{bmatrix},\tag{8}
$$
$\delta\boldsymbol\theta^{b_k}_t\in\mathbb R^3$ 是三维小扰动。**连续时间线性化误差态动力学**（对 (6) 求导，式 9）：
$$
\begin{bmatrix}\delta\dot{\boldsymbol\alpha}_t\\ \delta\dot{\boldsymbol\beta}_t\\ \delta\dot{\boldsymbol\theta}_t\\ \delta\dot{\mathbf b}_{a_t}\\ \delta\dot{\mathbf b}_{w_t}\end{bmatrix}^{b_k}
=\underbrace{\begin{bmatrix}
\mathbf 0 & \mathbf I & \mathbf 0 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & -\mathbf R^{b_k}_t\lfloor\hat{\mathbf a}_t-\mathbf b_{a_t}\rfloor_\times & -\mathbf R^{b_k}_t & \mathbf 0\\
\mathbf 0 & \mathbf 0 & -\lfloor\hat{\boldsymbol\omega}_t-\mathbf b_{w_t}\rfloor_\times & \mathbf 0 & -\mathbf I\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0
\end{bmatrix}}_{\mathbf F_t}
\begin{bmatrix}\delta\boldsymbol\alpha_t\\ \delta\boldsymbol\beta_t\\ \delta\boldsymbol\theta_t\\ \delta\mathbf b_{a_t}\\ \delta\mathbf b_{w_t}\end{bmatrix}^{b_k}
+\underbrace{\begin{bmatrix}
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\\
-\mathbf R^{b_k}_t & \mathbf 0 & \mathbf 0 & \mathbf 0\\
\mathbf 0 & -\mathbf I & \mathbf 0 & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf I & \mathbf 0\\
\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I
\end{bmatrix}}_{\mathbf G_t}
\begin{bmatrix}\mathbf n_a\\ \mathbf n_w\\ \mathbf n_{b_a}\\ \mathbf n_{b_w}\end{bmatrix}
=\mathbf F_t\,\delta\mathbf z_t + \mathbf G_t\,\mathbf n_t. \tag{9}
$$
**协方差递推**（一阶离散，初值 $\mathbf P^{b_k}_{b_k}=\mathbf 0$，式 10）：
$$
\mathbf P^{b_k}_{t+\delta t}=(\mathbf I+\mathbf F_t\delta t)\,\mathbf P^{b_k}_t\,(\mathbf I+\mathbf F_t\delta t)^\top+(\mathbf G_t\delta t)\,\mathbf Q\,(\mathbf G_t\delta t)^\top,\quad t\in[k,k+1], \tag{10}
$$
$\mathbf Q=\mathrm{diag}(\boldsymbol\sigma_a^2,\boldsymbol\sigma_w^2,\boldsymbol\sigma_{b_a}^2,\boldsymbol\sigma_{b_w}^2)$。

**一阶雅可比递推**（式 11，初值 $\mathbf J_{b_k}=\mathbf I$）：
$$
\mathbf J_{t+\delta t}=(\mathbf I+\mathbf F_t\delta t)\,\mathbf J_t,\quad t\in[k,k+1]. \tag{11}
$$

**偏置一阶校正**（式 12）。偏置改变小时，用雅可比近似校正预积分量，避免重传播：
$$
\begin{aligned}
\boldsymbol\alpha^{b_k}_{b_{k+1}} &\approx \hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}}+\mathbf J^{\alpha}_{b_a}\,\delta\mathbf b_{a_k}+\mathbf J^{\alpha}_{b_w}\,\delta\mathbf b_{w_k},\\
\boldsymbol\beta^{b_k}_{b_{k+1}} &\approx \hat{\boldsymbol\beta}^{b_k}_{b_{k+1}}+\mathbf J^{\beta}_{b_a}\,\delta\mathbf b_{a_k}+\mathbf J^{\beta}_{b_w}\,\delta\mathbf b_{w_k},\\
\boldsymbol\gamma^{b_k}_{b_{k+1}} &\approx \hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}\otimes\begin{bmatrix}1\\ \tfrac12\mathbf J^{\gamma}_{b_w}\,\delta\mathbf b_{w_k}\end{bmatrix},
\end{aligned}\tag{12}
$$
其中 $\mathbf J^{\alpha}_{b_a}$ 是 $\mathbf J_{b_{k+1}}$ 中位置对应 $\dfrac{\delta\boldsymbol\alpha^{b_k}_{b_{k+1}}}{\delta\mathbf b_{a_k}}$ 的子块，余 $\mathbf J^{\alpha}_{b_w},\mathbf J^{\beta}_{b_a},\mathbf J^{\beta}_{b_w},\mathbf J^{\gamma}_{b_w}$ 同义。偏置略变时用 (12) 近似校正而非重传播。

**IMU 测量模型 + 协方差**（式 13）。可写出 IMU 测量模型及其协方差 $\mathbf P^{b_k}_{b_{k+1}}$：
$$
\begin{bmatrix}\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}}\\ \hat{\boldsymbol\beta}^{b_k}_{b_{k+1}}\\ \hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}\\ \mathbf 0\\ \mathbf 0\end{bmatrix}
=\begin{bmatrix}
\mathbf R^{b_k}_w\big(\mathbf p^w_{b_{k+1}}-\mathbf p^w_{b_k}+\tfrac12\mathbf g^w\Delta t_k^2-\mathbf v^w_{b_k}\Delta t_k\big)\\[2pt]
\mathbf R^{b_k}_w\big(\mathbf v^w_{b_{k+1}}+\mathbf g^w\Delta t_k-\mathbf v^w_{b_k}\big)\\[2pt]
\mathbf q^{b_k}_w\otimes\mathbf q^w_{b_{k+1}}\\[2pt]
\mathbf b_{a_{b_{k+1}}}-\mathbf b_{a_{b_k}}\\[2pt]
\mathbf b_{w_{b_{k+1}}}-\mathbf b_{w_{b_k}}
\end{bmatrix}.\tag{13}
$$
（注：式 13 右侧第三行 $\mathbf q^{b_k}_w\otimes\mathbf q^w_{b_{k+1}}$ 应理解为 $(\mathbf q^w_{b_k})^{-1}\otimes\mathbf q^w_{b_{k+1}}$，对应 (5) 第三式。）

> **与 Forster 的关系（综合 agent 必读）**：VINS 的 $\boldsymbol\alpha,\boldsymbol\beta,\boldsymbol\gamma$ 对应 Forster 的 $\Delta\mathbf p,\Delta\mathbf v,\Delta\mathbf R$，但 (a) VINS 把 $\boldsymbol\alpha,\boldsymbol\beta$ 定义里**含 $\Delta t$ 加权但不减重力**（重力在残差里减），Forster 的 $\Delta\mathbf p,\Delta\mathbf v$ 也已扣 $\mathbf v_i,\mathbf g$ 项（见 Forster 式 26）；(b) VINS 旋转用四元数右乘扰动、Forster 用 $\mathrm{SO}(3)+\mathbf J_r$；(c) VINS 的 bias 雅可比由 (11) 数值递推、Forster 给**解析闭式**（§Forster.C）。两者在偏置一阶校正思想上一致（"测量 = 大值 + 小扰动"）。

## §V 估计器初始化（源 §V）——本章「初始化」核心

**为何需要松耦合初始化**（源 §V 引言）：单目紧耦合 VIO 高度非线性，尺度不直接可观，无好初值难直接融合两测量。假设静止起步不合适（实际常在运动中初始化）；IMU 大偏置时更复杂 → 初始化是单目 VINS 最脆弱步，需鲁棒过程。

**思路（源 §V）**：采用**松耦合**得初值。视觉 SLAM/SfM 有好初始化性质——纯视觉系统可由相对运动法（八点 [32]/五点 [33] 或单应）自举初值。**把度量 IMU 预积分与纯视觉 SfM 结果对齐** → 粗恢复尺度、重力、速度、甚至偏置，足以自举非线性单目 VINS 估计器（源 Fig. 4）。

**与 [17] 的差异**：[17] 在初始化阶段同时估陀螺与加计偏置；本文**初始步忽略加计偏置项**——因加计偏置与重力耦合，且重力幅值远大于平台动力学、初始化阶段又短 → 加计偏置难观测。详细加计偏置标定见前作 [34]。

### §V.A 滑窗纯视觉 SfM（源 §V.A）

维护有界滑窗。流程：
1. 检查最新帧与所有先前帧的特征对应。若找到**稳定跟踪（>30 跟踪特征）且足够视差（>20 旋转补偿像素）**，用**五点法** [33] 恢复这两帧间相对旋转与**上尺度**平移；否则保留最新帧、等新帧。
2. 五点法成功 → **任意设定尺度**并三角化这两帧观测的所有特征。
3. 基于三角化特征，用 **PnP** [35]（EPnP）估窗内其余所有帧位姿。
4. 最后**全局完整 BA** [36] 最小化所有特征观测的总重投影误差。

因尚无 world 系知识，**设第一相机系 $(\cdot)^{c_0}$ 为 SfM 参考系**；所有帧位姿 $(\bar{\mathbf p}^{c_0}_{c_k},\mathbf q^{c_0}_{c_k})$ 与特征位置都相对 $(\cdot)^{c_0}$ 表示。

给粗略外参 $(\mathbf p^b_c,\mathbf q^b_c)$（相机↔IMU），把位姿从相机系转到 body(IMU) 系（式 14）：
$$
\mathbf q^{c_0}_{b_k}=\mathbf q^{c_0}_{c_k}\otimes(\mathbf q^b_c)^{-1},\qquad
s\,\bar{\mathbf p}^{c_0}_{b_k}=s\,\bar{\mathbf p}^{c_0}_{c_k}-\mathbf R^{c_0}_{b_k}\mathbf p^b_c, \tag{14}
$$
$s$ 是把视觉结构对齐到度量尺度的尺度参数——**求 $s$ 是初始化成功的关键**。

### §V.B 视觉惯性对齐（源 §V.B）

#### §V.B.1 陀螺偏置标定（源 §V.B.1）

考虑窗内两连续帧 $b_k,b_{k+1}$，由视觉 SfM 得旋转 $\mathbf q^{c_0}_{b_k},\mathbf q^{c_0}_{b_{k+1}}$，由 IMU 预积分得相对约束 $\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}$。把预积分项对陀螺偏置线性化，最小化代价（式 15）：
$$
\min_{\delta\mathbf b_w}\ \sum_{k\in\mathcal B}\Big\|\,(\mathbf q^{c_0}_{b_{k+1}})^{-1}\otimes\mathbf q^{c_0}_{b_k}\otimes\boldsymbol\gamma^{b_k}_{b_{k+1}}\,\Big\|^2,\qquad
\boldsymbol\gamma^{b_k}_{b_{k+1}}\approx\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}\otimes\begin{bmatrix}1\\ \tfrac12\mathbf J^{\gamma}_{b_w}\delta\mathbf b_w\end{bmatrix}, \tag{15}
$$
$\mathcal B$ 索引窗内所有帧。用 §IV.B 推得的 bias 雅可比作 $\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}$ 对陀螺偏置的一阶近似。理想下 $(\mathbf q^{c_0}_{b_{k+1}})^{-1}\otimes\mathbf q^{c_0}_{b_k}\otimes\boldsymbol\gamma^{b_k}_{b_{k+1}}$ 应为单位四元数，取其虚部（向量部）= 残差 → 化为线性最小二乘解 $\delta\mathbf b_w$。得陀螺偏置初值后，**用新陀螺偏置重传播所有预积分量** $\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}},\hat{\boldsymbol\beta}^{b_k}_{b_{k+1}},\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}$。

> **重建说明（\rebuilt 提示）**：式 15 化为线性系统的标准做法（VINS 略）：令 $\mathbf r=2\big[(\mathbf q^{c_0}_{b_{k+1}})^{-1}\otimes\mathbf q^{c_0}_{b_k}\otimes\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}\big]_{xyz}$，对 $\delta\mathbf b_w$ 线性化得 $\mathbf J^{\gamma}_{b_w}{}^\top\mathbf J^{\gamma}_{b_w}\,\delta\mathbf b_w=\mathbf J^{\gamma}_{b_w}{}^\top\mathbf r$（正规方程，对所有 $k$ 累加）。综合时可补此步。

#### §V.B.2 速度、重力向量、度量尺度初始化（源 §V.B.2）

陀螺偏置初始化后，初始化其余导航关键状态——速度、重力向量、度量尺度。**待估状态向量**（式 16）：
$$
\mathcal X_I=\big[\mathbf v^{b_0}_{b_0},\,\mathbf v^{b_1}_{b_1},\,\cdots,\,\mathbf v^{b_n}_{b_n},\,\mathbf g^{c_0},\,s\big], \tag{16}
$$
$\mathbf v^{b_k}_{b_k}$ 是取第 $k$ 帧时 body 系下的速度，$\mathbf g^{c_0}$ 是 $c_0$ 系下重力向量，$s$ 把单目 SfM 缩放到度量单位。维数 $=3(n+1)+3+1$。

考虑窗内两连续帧 $b_k,b_{k+1}$，把 (5) 写成（用 $c_0$ 系视觉量，式 17）：
$$
\begin{aligned}
\boldsymbol\alpha^{b_k}_{b_{k+1}} &= \mathbf R^{b_k}_{c_0}\Big(s(\bar{\mathbf p}^{c_0}_{b_{k+1}}-\bar{\mathbf p}^{c_0}_{b_k})+\tfrac12\mathbf g^{c_0}\Delta t_k^2-\mathbf R^{c_0}_{b_k}\mathbf v^{b_k}_{b_k}\Delta t_k\Big),\\
\boldsymbol\beta^{b_k}_{b_{k+1}} &= \mathbf R^{b_k}_{c_0}\Big(\mathbf R^{c_0}_{b_{k+1}}\mathbf v^{b_{k+1}}_{b_{k+1}}+\mathbf g^{c_0}\Delta t_k-\mathbf R^{c_0}_{b_k}\mathbf v^{b_k}_{b_k}\Big).
\end{aligned}\tag{17}
$$
联立 (14) 与 (17) 得**线性测量模型**（式 18）：
$$
\hat{\mathbf z}^{b_k}_{b_{k+1}}=\begin{bmatrix}\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}}-\mathbf p^b_c+\mathbf R^{b_k}_{c_0}\mathbf R^{c_0}_{b_{k+1}}\mathbf p^b_c\\[3pt] \hat{\boldsymbol\beta}^{b_k}_{b_{k+1}}\end{bmatrix}=\mathbf H^{b_k}_{b_{k+1}}\,\mathcal X_I+\mathbf n^{b_k}_{b_{k+1}}, \tag{18}
$$
其中（式 19，$6\times(\text{对应两帧速度块}+\text{重力块}+\text{尺度块})$ 的块结构）：
$$
\mathbf H^{b_k}_{b_{k+1}}=\begin{bmatrix}
-\mathbf I\Delta t_k & \mathbf 0 & \tfrac12\mathbf R^{b_k}_{c_0}\Delta t_k^2 & \mathbf R^{b_k}_{c_0}(\bar{\mathbf p}^{c_0}_{c_{k+1}}-\bar{\mathbf p}^{c_0}_{c_k})\\[3pt]
-\mathbf I & \mathbf R^{b_k}_{c_0}\mathbf R^{c_0}_{b_{k+1}} & \mathbf R^{b_k}_{c_0}\Delta t_k & \mathbf 0
\end{bmatrix}. \tag{19}
$$
（4 列块对应 $[\mathbf v^{b_k}_{b_k},\ \mathbf v^{b_{k+1}}_{b_{k+1}},\ \mathbf g^{c_0},\ s]$。）$\mathbf R^{c_0}_{b_k},\mathbf R^{c_0}_{b_{k+1}},\bar{\mathbf p}^{c_0}_{c_k},\bar{\mathbf p}^{c_0}_{c_{k+1}}$ 由上尺度单目视觉 SfM 得到，$\Delta t_k$ 是两连续帧时间间隔。求解**线性最小二乘**（式 20）：
$$
\min_{\mathcal X_I}\ \sum_{k\in\mathcal B}\big\|\hat{\mathbf z}^{b_k}_{b_{k+1}}-\mathbf H^{b_k}_{b_{k+1}}\mathcal X_I\big\|^2, \tag{20}
$$
得每帧 body 系速度、视觉参考系 $(\cdot)^{c_0}$ 下重力向量、以及尺度 $s$。

#### §V.B.3 重力精化（源 §V.B.3）——含算法 1

前一线性步得到的重力向量可由**约束其幅值**精化。多数情况重力幅值已知 → 重力只剩 **2 DOF**。在切空间用两变量重参数化（源 Fig. 5）：
$$
\mathbf g = g\cdot\hat{\bar{\mathbf g}}+w_1\mathbf b_1+w_2\mathbf b_2, \tag{重力 2-DOF 参数化}
$$
$g$ 是已知重力幅值（$\approx 9.81\,\text{m/s}^2$），$\hat{\bar{\mathbf g}}$ 是表重力方向的单位向量，$\mathbf b_1,\mathbf b_2$ 是张成切平面的两正交基，$w_1,w_2$ 是朝 $\mathbf b_1,\mathbf b_2$ 的对应位移。用叉积找一组 $\mathbf b_1,\mathbf b_2$（算法 1）。把 (17) 里的 $\mathbf g$ 用 $g\cdot\hat{\bar{\mathbf g}}+w_1\mathbf b_1+w_2\mathbf b_2$ 替换，连同其他状态变量一起解 $w_1,w_2$。**此过程迭代直到 $\hat{\mathbf g}$ 收敛**。

> **算法 1（源）：求 $\mathbf b_1,\mathbf b_2$**
> ```
> if  ĝ̄ ≠ [1, 0, 0]  then
>     b1 ← normalize( ĝ̄ × [1, 0, 0] );
> else
>     b1 ← normalize( ĝ̄ × [0, 0, 1] );
> end
> b2 ← ĝ̄ × b1;
> ```

#### §V.B.4 完成初始化（源 §V.B.4）

精化重力后，通过**把重力旋到 $z$ 轴**得 world 系与相机系 $c_0$ 间旋转 $\mathbf q^w_{c_0}$。然后：
- 把所有变量从参考系 $(\cdot)^{c_0}$ 旋到 world 系 $(\cdot)^w$；
- body 系速度也旋到 world 系；
- 视觉 SfM 的平移分量按 $s$ 缩放到度量单位。

至此初始化完成，所有度量值喂给紧耦合单目 VIO。

## §VI 紧耦合单目 VIO（源 §VI）——本章「基于优化」主线核心

初始化后进入基于滑窗的紧耦合单目 VIO（源 Fig. 3）。

### §VI.A 问题表述（源 §VI.A）

**滑窗全状态向量**（式 21）：
$$
\begin{aligned}
\mathcal X &= [\mathbf x_0,\ \mathbf x_1,\ \cdots,\ \mathbf x_n,\ \mathbf x^b_c,\ \lambda_0,\ \lambda_1,\ \cdots,\ \lambda_m],\\
\mathbf x_k &= [\mathbf p^w_{b_k},\ \mathbf v^w_{b_k},\ \mathbf q^w_{b_k},\ \mathbf b_a,\ \mathbf b_g],\quad k\in[0,n],\\
\mathbf x^b_c &= [\mathbf p^b_c,\ \mathbf q^b_c],
\end{aligned}\tag{21}
$$
$\mathbf x_k$ 是取第 $k$ 帧时的 IMU 状态（world 系位置/速度/姿态 + body 系加计/陀螺偏置）；$n$=关键帧总数；$m$=滑窗内特征总数；$\lambda_l$=第 $l$ 个特征**从其首次观测起的逆深度**；$\mathbf x^b_c$=**相机-IMU 外参**（在线标定，纳入状态向量优化）。

**视觉惯性 BA（MAP 估计）**（式 22）：最小化先验 + 所有测量残差的马氏范数之和：
$$
\boxed{\;
\min_{\mathcal X}\ \Big\{\ \big\|\mathbf r_p-\mathbf H_p\mathcal X\big\|^2
+\sum_{k\in\mathcal B}\big\|\mathbf r_{\mathcal B}(\hat{\mathbf z}^{b_k}_{b_{k+1}},\mathcal X)\big\|^2_{\mathbf P^{b_k}_{b_{k+1}}}
+\sum_{(l,j)\in\mathcal C}\rho\big(\big\|\mathbf r_{\mathcal C}(\hat{\mathbf z}^{c_j}_l,\mathcal X)\big\|^2_{\mathbf P^{c_j}_l}\big)\ \Big\}\;}\tag{22}
$$
其中 **Huber 范数** [37]（式 23）：
$$
\rho(s)=\begin{cases}1, & s\ge 1,\\ 2\sqrt s-1, & s<1.\end{cases}\tag{23}
$$
$\mathbf r_{\mathcal B}(\hat{\mathbf z}^{b_k}_{b_{k+1}},\mathcal X)$、$\mathbf r_{\mathcal C}(\hat{\mathbf z}^{c_j}_l,\mathcal X)$ 分别是 IMU、视觉测量残差（详 §VI.B、§VI.C）；$\mathcal B$=所有 IMU 测量集；$\mathcal C$=当前滑窗内**至少被观测两次**的特征集；$\{\mathbf r_p,\mathbf H_p\}$=来自边缘化的先验信息。用 **Ceres Solver** [38] 求解此非线性问题。

### §VI.B IMU 测量残差（源 §VI.B）

按 (13) 的 IMU 测量模型，两连续帧 $b_k,b_{k+1}$ 间预积分 IMU 测量残差（**15 维**，式 24）：
$$
\mathbf r_{\mathcal B}(\hat{\mathbf z}^{b_k}_{b_{k+1}},\mathcal X)=\begin{bmatrix}\delta\boldsymbol\alpha^{b_k}_{b_{k+1}}\\ \delta\boldsymbol\beta^{b_k}_{b_{k+1}}\\ \delta\boldsymbol\theta^{b_k}_{b_{k+1}}\\ \delta\mathbf b_a\\ \delta\mathbf b_g\end{bmatrix}
=\begin{bmatrix}
\mathbf R^{b_k}_w\big(\mathbf p^w_{b_{k+1}}-\mathbf p^w_{b_k}+\tfrac12\mathbf g^w\Delta t_k^2-\mathbf v^w_{b_k}\Delta t_k\big)-\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}}\\[3pt]
\mathbf R^{b_k}_w\big(\mathbf v^w_{b_{k+1}}+\mathbf g^w\Delta t_k-\mathbf v^w_{b_k}\big)-\hat{\boldsymbol\beta}^{b_k}_{b_{k+1}}\\[3pt]
2\Big[(\mathbf q^w_{b_k})^{-1}\otimes\mathbf q^w_{b_{k+1}}\otimes(\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}})^{-1}\Big]_{xyz}\\[3pt]
\mathbf b_{a_{b_{k+1}}}-\mathbf b_{a_{b_k}}\\[3pt]
\mathbf b_{w_{b_{k+1}}}-\mathbf b_{w_{b_k}}
\end{bmatrix}.\tag{24}
$$
$[\cdot]_{xyz}$ 取四元数 $\mathbf q$ 的向量部作误差态表示；$\delta\boldsymbol\theta^{b_k}_{b_{k+1}}$ 是四元数的三维误差态表示。$[\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}},\hat{\boldsymbol\beta}^{b_k}_{b_{k+1}},\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}]$ 是两连续帧间仅用含噪加计/陀螺测量得到的预积分项；加计/陀螺偏置也含在残差里做**在线校正**。该残差以 $\mathbf P^{b_k}_{b_{k+1}}$（式 10 的预积分协方差）加权（马氏范数）。

### §VI.C 视觉测量残差（源 §VI.C）——单位球切平面残差

与传统针孔模型在广义像平面定义重投影误差不同，VINS **在单位球上定义相机测量残差**（适配广角/鱼眼/全向相机：光学可建模为连到单位球面的单位射线）。第 $l$ 个特征首次在第 $i$ 帧观测，其在第 $j$ 帧观测的残差（式 25）：
$$
\begin{aligned}
\mathbf r_{\mathcal C}(\hat{\mathbf z}^{c_j}_l,\mathcal X) &= \begin{bmatrix}\mathbf b_1 & \mathbf b_2\end{bmatrix}^\top\cdot\Big(\hat{\bar{\mathcal P}}^{c_j}_l-\frac{\mathcal P^{c_j}_l}{\|\mathcal P^{c_j}_l\|}\Big),\\
\hat{\bar{\mathcal P}}^{c_j}_l &= \pi_c^{-1}\!\begin{bmatrix}\hat u^{c_j}_l\\ \hat v^{c_j}_l\end{bmatrix},\\
\mathcal P^{c_j}_l &= \mathbf R^c_b\Big(\mathbf R^{b_j}_w\big(\mathbf R^w_{b_i}\big(\mathbf R^b_c\,\tfrac{1}{\lambda_l}\pi_c^{-1}\!\begin{bmatrix}u^{c_i}_l\\ v^{c_i}_l\end{bmatrix}+\mathbf p^b_c\big)+\mathbf p^w_{b_i}-\mathbf p^w_{b_j}\big)-\mathbf p^b_c\Big),
\end{aligned}\tag{25}
$$
其中 $[u^{c_i}_l,v^{c_i}_l]$ 是第 $l$ 特征在第 $i$ 帧的首次观测，$[\hat u^{c_j}_l,\hat v^{c_j}_l]$ 是在第 $j$ 帧的观测；$\pi_c^{-1}$ 是**反投影函数**（把像素位置经相机内参变为单位向量）。**视觉残差自由度为 2**，故把残差向量投到切平面：$\mathbf b_1,\mathbf b_2$ 是张成 $\hat{\bar{\mathcal P}}^{c_j}_l$ 切平面的两任意选正交基（用算法 1 找，源 Fig. 6）。$\mathbf P^{c_j}_l$（式 22 用）是切空间内固定长度的标准协方差。

> **VINS 未给视觉残差雅可比**（称"detailed 略"）。综合 agent 可由链式法则补 $\partial\mathbf r_{\mathcal C}/\partial\{\mathbf p^w_{b_i},\mathbf q^w_{b_i},\mathbf p^w_{b_j},\mathbf q^w_{b_j},\mathbf x^b_c,\lambda_l\}$，或引 Forster 式 42–44 / 本书 BA 章。

### §VI.D 边缘化（源 §VI.D，本源**只点名 Schur 补**，代数见 §VI.D'）

为界定优化 VIO 计算复杂度，引入边缘化：**选择性地把 IMU 状态 $\mathbf x_k$ 与特征 $\lambda_l$ 从滑窗中边缘化，同时把被边缘化状态对应的测量转成先验**。

**边缘化策略**（源 Fig. 7）：
- 若**第二新帧是关键帧** → 保留它，**边缘化最旧帧及其对应的视觉与惯性测量**；被边缘化测量转成先验，**新先验加到已有先验上**。
- 若**第二新帧不是关键帧** → 直接**丢弃该帧及其全部视觉测量**；但**保留连到该非关键帧的预积分惯性测量**，预积分过程继续向下一帧推进（不把非关键帧的所有测量都边缘化，以**保持系统稀疏性**）。

**策略目标（源）**：保持窗内关键帧**空间分散** → 保证足够视差以三角化特征，并最大化保留**大激励加计测量**的概率。

**边缘化用 Schur 补** [39] 执行：基于所有与被移除状态相关的边缘化测量构造新先验，新先验加到已有先验。

> **重要权衡（源 §VI.D）**：边缘化导致**线性化点的提前固定（early fix of linearization points）** → 可能次优。但因 VIO 可接受小漂移，作者认为边缘化的负面影响不致命。（这正是 FEJ/OC 要解决的一致性问题，见 §Obs。）

### §VI.D'（补：Schur 补边缘化完整代数——VINS 未给，由 Schur-Marg 辅源 + 标准推导补，自包含）

设当前 Gauss-Newton 一次迭代得到正规方程（信息形式）$\boldsymbol\Lambda\,\delta\mathbf x=\mathbf g$（$\boldsymbol\Lambda=\mathbf J^\top\boldsymbol\Sigma^{-1}\mathbf J$ 为 Hessian/信息矩阵，$\mathbf g=-\mathbf J^\top\boldsymbol\Sigma^{-1}\mathbf r$ 为梯度负值）。把状态分块为**待边缘化 $\delta\mathbf x_m$**（如最旧帧 + 仅其可见特征）与**保留 $\delta\mathbf x_r$**：
$$
\begin{bmatrix}\boldsymbol\Lambda_{mm} & \boldsymbol\Lambda_{mr}\\ \boldsymbol\Lambda_{rm} & \boldsymbol\Lambda_{rr}\end{bmatrix}
\begin{bmatrix}\delta\mathbf x_m\\ \delta\mathbf x_r\end{bmatrix}
=\begin{bmatrix}\mathbf g_m\\ \mathbf g_r\end{bmatrix}. \tag{M1}
$$
上式第一块行解出 $\delta\mathbf x_m=\boldsymbol\Lambda_{mm}^{-1}(\mathbf g_m-\boldsymbol\Lambda_{mr}\delta\mathbf x_r)$，代入第二块行消去 $\delta\mathbf x_m$，得**仅含保留状态**的方程：
$$
\underbrace{\big(\boldsymbol\Lambda_{rr}-\boldsymbol\Lambda_{rm}\boldsymbol\Lambda_{mm}^{-1}\boldsymbol\Lambda_{mr}\big)}_{\boldsymbol\Lambda_p}\,\delta\mathbf x_r=\underbrace{\mathbf g_r-\boldsymbol\Lambda_{rm}\boldsymbol\Lambda_{mm}^{-1}\mathbf g_m}_{\mathbf g_p}. \tag{M2}
$$
- **先验信息矩阵**（Schur 补）：$\boxed{\boldsymbol\Lambda_p=\boldsymbol\Lambda_{rr}-\boldsymbol\Lambda_{rm}\boldsymbol\Lambda_{mm}^{-1}\boldsymbol\Lambda_{mr}}$。
- **先验梯度**：$\boxed{\mathbf g_p=\mathbf g_r-\boldsymbol\Lambda_{rm}\boldsymbol\Lambda_{mm}^{-1}\mathbf g_m}$。

此 $\{\boldsymbol\Lambda_p,\mathbf g_p\}$ 即下一轮优化中加在保留状态上的**线性先验代价项**：$\tfrac12\delta\mathbf x_r^\top\boldsymbol\Lambda_p\delta\mathbf x_r-\mathbf g_p^\top\delta\mathbf x_r$。在残差形式下（VINS 式 22 写为 $\|\mathbf r_p-\mathbf H_p\mathcal X\|^2$），可对 $\boldsymbol\Lambda_p$ 做特征分解 $\boldsymbol\Lambda_p=\mathbf U\boldsymbol\Sigma\mathbf U^\top$，取 $\mathbf H_p=\boldsymbol\Sigma^{1/2}\mathbf U^\top$、$\mathbf r_p=\boldsymbol\Sigma^{-1/2}\mathbf U^\top\mathbf g_p$（即把信息先验"平方根化"为可塞进最小二乘的残差块；OKVIS/VINS 即如此存储）。

**FEJ（First-Estimate Jacobian）要点**（VINS 默认不做、Huang-Rev 指出其重要）：边缘化后 $\boldsymbol\Lambda_p,\mathbf g_p,\mathbf H_p$ 的雅可比在**边缘化时刻的线性化点**算得并**冻结**；后续若保留状态在新线性化点更新，先验雅可比与新因子雅可比的线性化点不一致 → 会沿不可观方向**注入虚假信息（spurious information）**破坏一致性。FEJ 的修法：**所有涉及被边缘化变量的因子，其雅可比一律在该变量的"首次估计"处求**，保证不可观子空间维数正确（见 §Obs）。

### §VI.E 仅运动的视觉惯性 BA（相机率状态估计，源 §VI.E）

对低算力设备（手机），完整紧耦合 VIO 因非线性优化重负达不到相机率输出。故用**轻量"仅运动"视觉惯性 BA** 把状态估计提到相机率（≈30 Hz）。

代价函数同 VIO 的 (22)，但**只优化固定数目最新 IMU 状态的位姿与速度**；把特征深度、外参、偏置、不想优化的旧 IMU 状态视为常量。仍用所有视觉与惯性测量。比单帧 PnP 平滑得多（源 Fig. 8）。完整紧耦合 VIO 在 SOTA 嵌入式上可能 >50 ms，而仅运动 BA 仅约 **5 ms** → 低延迟相机率位姿估计，利于无人机/AR。

### §VI.F IMU 前向传播（IMU 率状态估计，源 §VI.F）

IMU 率远高于视觉率。虽 VIO 频率受图像率限，仍可用最近 IMU 测量集**直接传播最新 VIO 估计**达 IMU 率。高频状态估计可作**闭环控制的状态反馈**（自主飞行实验 §IX-D 用之）。

### §VI.G 失败检测与恢复（源 §VI.G）

紧耦合 VIO 虽鲁棒，剧烈光照变化/激进运动仍会失败。失败检测（独立模块）准则：
- 最新帧跟踪特征数低于某阈值；
- 最近两次估计器输出间位置或旋转大不连续；
- 偏置或外参估计大变化。

一旦检测到失败 → 系统**切回初始化阶段**。VIO 成功重初始化后，**创建新的、独立的位姿图段**。

## §VII 重定位（源 §VII）——本章「回环」之一

滑窗 + 边缘化界定计算量，但引入累积漂移（具体：全局 3D 位置 $(x,y,z)$ + 绕重力方向 yaw 的漂移）。为消漂移，提出**与单目 VIO 无缝集成的紧耦合重定位**。流程（源 Fig. 9(a)）：回环检测识别已访问地点 → 在回环候选与当前帧间建**特征级**连接 → 这些特征对应被紧耦合进 VIO → 得几乎无漂移的状态估计、计算开销极小。多特征多观测直接用于重定位 → 更高精度、更好平滑。

### §VII.A 回环检测（源 §VII.A）

用 **DBoW2** [6]（SOTA 词袋地点识别）。除 VIO 用的角点外，**额外检测 500 角点并用 BRIEF 描述子** [40] 描述（额外角点提高回环召回率）。描述子作视觉单词查询视觉数据库。DBoW2 经**时间与几何一致性检查**后返回回环候选。保留所有 BRIEF 描述子供特征检索，但**丢弃原图省内存**。
**洞察**：因单目 VIO 使 roll/pitch 可观 → **无需依赖旋转不变特征**（如 ORB-SLAM 用的 ORB）。

### §VII.B 特征检索（源 §VII.B）——两步几何 outlier 剔除

检测到回环后，由 **BRIEF 描述子匹配**在局部滑窗与回环候选间建特征对应。直接描述子匹配会产生大量 outlier → 用**两步几何 outlier 剔除**（源 Fig. 10）：
- **2D-2D**：基础矩阵 + RANSAC [31]（用当前图与回环候选图中检索特征的 2D 观测）。
- **3D-2D**：PnP + RANSAC [35]（基于局部滑窗中特征的已知 3D 位置 + 回环候选图中 2D 观测）。

内点数超阈值 → 视为正确回环检测、执行重定位。

### §VII.C 紧耦合重定位（源 §VII.C）

重定位把单目 VIO 当前维护的滑窗对齐到过去位姿图。**重定位时把所有回环帧位姿当常量**。联合优化滑窗，用所有 IMU 测量、局部视觉测量、回环检索特征对应。回环帧观测的检索特征的视觉测量模型同 VIO 的 (25)，唯一差别：回环帧位姿 $(\hat{\mathbf q}^w_v,\hat{\mathbf p}^w_v)$（取自位姿图 §VIII，或首次重定位时直接取过去里程计输出）**当常量**。在 (22) 上加回环项（式 26）：
$$
\min_{\mathcal X}\ \Big\{\ \big\|\mathbf r_p-\mathbf H_p\mathcal X\big\|^2
+\sum_{k\in\mathcal B}\big\|\mathbf r_{\mathcal B}(\hat{\mathbf z}^{b_k}_{b_{k+1}},\mathcal X)\big\|^2_{\mathbf P^{b_k}_{b_{k+1}}}
+\sum_{(l,j)\in\mathcal C}\rho\big(\|\mathbf r_{\mathcal C}(\hat{\mathbf z}^{c_j}_l,\mathcal X)\|^2_{\mathbf P^{c_j}_l}\big)
+\sum_{(l,v)\in\mathcal L}\rho\big(\|\mathbf r_{\mathcal C}(\hat{\mathbf z}^v_l,\mathcal X,\hat{\mathbf q}^w_v,\hat{\mathbf p}^w_v)\|^2_{\mathbf P^{c_v}_l}\big)\ \Big\}, \tag{26}
$$
$\mathcal L$=回环帧中检索特征的观测集，$(l,v)$=第 $l$ 特征在回环帧 $v$ 中观测。注意：虽代价与 (22) 略不同，**待解状态维数不变**（回环帧位姿当常量）。多回环时同时用所有回环帧的所有特征对应 → 多视约束 → 更高精度更好平滑。过去位姿与回环帧的全局优化在重定位后进行（§VIII）。

## §VIII 全局位姿图优化（源 §VIII）——本章「回环」之二

重定位后局部滑窗移位、与过去位姿对齐。利用重定位结果，此**4-DOF 位姿图优化**步把过去位姿集注册到全局一致配置。因视觉惯性使 roll/pitch 完全可观 → 累积漂移仅 4-DOF $(x,y,z,\text{yaw})$ → 忽略估计无漂移的 roll/pitch，只做 4-DOF 位姿图优化。

### §VIII.A 关键帧加入位姿图（源 §VIII.A）

关键帧从滑窗边缘化时加入位姿图，作位姿图顶点，经两类边连其他顶点：
1. **序列边（Sequential Edge）**：关键帧与其前若干关键帧建序列边，表两关键帧在局部滑窗中的相对变换，值直接取自 VIO。新边缘化关键帧 $i$ 与其前一关键帧 $j$，序列边只含相对位置 $\hat{\mathbf p}^i_{ij}$ 与 yaw 角 $\hat\psi_{ij}$（式 27）：
$$
\hat{\mathbf p}^i_{ij}=\hat{\mathbf R}_i^{-1}(\hat{\mathbf p}^w_j-\hat{\mathbf p}^w_i),\qquad \hat\psi_{ij}=\hat\psi_j-\hat\psi_i. \tag{27}
$$
2. **回环边（Loop Closure Edge）**：若新边缘化关键帧有回环连接 → 由回环边连到回环帧，同样只含 4-DOF 相对位姿变换（定义同 (27)），值来自重定位结果。

### §VIII.B 4-DOF 位姿图优化（源 §VIII.B）

帧 $i,j$ 间边的残差**最小化定义**（式 28）：
$$
\mathbf r_{i,j}(\mathbf p^w_i,\psi_i,\mathbf p^w_j,\psi_j)=\begin{bmatrix}
\mathbf R(\hat\phi_i,\hat\theta_i,\psi_i)^{-1}(\mathbf p^w_j-\mathbf p^w_i)-\hat{\mathbf p}^i_{ij}\\[3pt]
\psi_j-\psi_i-\hat\psi_{ij}
\end{bmatrix}, \tag{28}
$$
$\hat\phi_i,\hat\theta_i$ 是估计的 roll、pitch（直接取自单目 VIO，**固定不优化**）。整个序列边 + 回环边图通过最小化（式 29）优化：
$$
\min_{\mathbf p,\psi}\ \Big\{\ \sum_{(i,j)\in\mathcal S}\|\mathbf r_{i,j}\|^2+\sum_{(i,j)\in\mathcal L}\rho(\|\mathbf r_{i,j}\|^2)\ \Big\}, \tag{29}
$$
$\mathcal S$=所有序列边集，$\mathcal L$=所有回环边集。虽紧耦合重定位已帮助消除错误回环，**回环边再加 Huber 范数 $\rho(\cdot)$** 进一步降低可能错误回环影响；**序列边不用鲁棒范数**（这些边从 VIO 提取，已含足够 outlier 剔除）。位姿图优化与重定位（§VII.C）**异步两线程**运行。

### §VIII.C 位姿图管理（源 §VIII.C）

travel 距离增长时位姿图可能无界增长 → 实现下采样维持数据库到有限大小。**所有带回环约束的关键帧都保留**；其他与邻居太近或朝向太相似的关键帧可移除。**某关键帧被移除的概率正比于其对邻居的空间密度**。

## §IX 实验结果（源 §IX，主要数值/系统结论）

**数据集对比（§IX.A）**：用 **EuRoC MAV** 视觉惯性数据集 [41]（机载微型飞行器；立体相机 Aptina MT9V034 全局快门 WVGA 单色 20 FPS；同步 IMU ADIS16448 200 Hz；真值 VICON + Leica MS50）。**只用左相机图**。数据集含大 IMU 偏置与光照变化。对比对象 **OKVIS** [16]（SOTA 单目/立体优化滑窗 VIO）。记号：VINS=仅单目 VIO；VINS_loop=完整版（重定位 + 位姿图）；OKVIS_mono / OKVIS_stereo。对齐方式：丢前 100 输出，用随后 150 输出对齐真值，比较其余。

**结论（源 §IX.A 文字）**：
- MH_03_median、MH_05_difficult 中，**VINS_loop 平移误差最小**；回环高效界定累积漂移。
- OKVIS 在 roll/pitch 角估计更好——可能因 VINS 用预积分（IMU 传播的一阶近似）省算力。
- VINS-Mono 在所有 EuRoC 数据集表现好，连最难的 V1_03_difficult（激进运动 + 无纹理 + 显著光照变化）都能因专门初始化过程**快速初始化**。
- **纯 VIO：VINS-Mono 与 OKVIS 精度相近，难分伯仲；但系统级 VINS-Mono 胜出**（完整系统含鲁棒初始化与回环）。

**应用**：
- 室内重复场景（§IX-C）：与 Google Tango 对比。
- 大尺度（§IX，2.5 km 户外手持，源 Fig. 1/20）。
- 无人机自主飞行（§IX-D，源 Fig. 21–23）：自研前视鱼眼相机（MatrixVision mvBlueFOX-MLC200w，190° FOV）+ DJI A3 飞控（ADXL278+ADXRS290，100 Hz）；图八轨迹四圈、**回环关闭**、VINS 估计作实时位置反馈；OptiTrack 真值；总长 **61.97 m，最终漂移 0.18 m**（源 Fig. 22）。
- iOS 移动设备（§IX）：iPhone7 Plus 跑 VINS-Mobile 对比 Google Tango；约 264 m，4-DOF 位姿图消除总漂移，VINS 返回起点，虚拟立方体注册回同处（源 Fig. 25）。Tango 局部更准，但 VINS 证明可在通用移动设备运行并有潜力媲美专用设备。

## §X 结论与未来工作（源 §X）

提出鲁棒通用单目视觉惯性估计器，特点：SOTA + 新颖的 IMU 预积分、初始化与失败恢复、在线外参标定、紧耦合 VIO、重定位、高效全局优化；超越 SOTA 开源实现与高度优化的工业方案；PC + iOS 开源。**未来方向**：① 在线评估单目 VINS **可观性**性质、在线生成运动规划恢复可观性（**本章可观性主题的源动机**——单目 VINS 在某些运动/环境下达到弱可观甚至退化）；② 大量消费设备上的部署（需几乎所有内外参在线标定 + 在线标定质量识别）；③ 单目 VINS 稠密建图 [44]。

---

# 第二部分：滤波主线 MSCKF 完整推导（辅源 [MSCKF]，本章「基于滤波」主线）

> 来源：Mourikis & Roumeliotis, *A Multi-State Constraint Kalman Filter for Vision-Aided Inertial Navigation*, ICRA 2007. **核心贡献**：一种测量模型，能表达"静止特征被多相机位姿观测"产生的几何约束，**而无需把特征位置放入滤波状态向量** → 计算复杂度**仅与特征数线性**（比 EKF-SLAM 快）。记号见 §0.2（JPL 四元数）。

## §MSCKF.A 滤波状态向量结构（源 §III.A）

目标：跟踪 IMU 固连系 $\{I\}$ 相对全局系 $\{G\}$ 的 3D 位姿。**演进 IMU 状态**（16 维表示，含单位四元数）：
$$
\mathbf X_{IMU}=\big[{}^I_G\bar{\mathbf q}^\top,\ \mathbf b_g^\top,\ {}^G\mathbf v_I^\top,\ \mathbf b_a^\top,\ {}^G\mathbf p_I^\top\big]^\top, \tag{1}
$$
${}^I_G\bar{\mathbf q}$=描述 $\{G\}\to\{I\}$ 旋转的单位四元数，${}^G\mathbf v_I,{}^G\mathbf p_I$=IMU 在 $\{G\}$ 中速度/位置，$\mathbf b_g,\mathbf b_a$=陀螺/加计偏置。**姿态误差用四元数误差**（式 3）：
$$
\delta\bar{\mathbf q}\approx\big[\tfrac12\delta\boldsymbol\theta^\top,\ 1\big]^\top, \tag{3}
$$
$\delta\boldsymbol\theta\in\mathbb R^3$ 是描述使真实姿态与估计姿态重合的小旋转（3-DOF 最小表示）。**IMU 误差态**（15 维）：
$$
\tilde{\mathbf X}_{IMU}=\big[\delta\boldsymbol\theta_I^\top,\ \tilde{\mathbf b}_g^\top,\ {}^G\tilde{\mathbf v}_I^\top,\ \tilde{\mathbf b}_a^\top,\ {}^G\tilde{\mathbf p}_I^\top\big]^\top. \tag{?}
$$

设时刻 $k$ 状态含 $N$ 个相机位姿，**完整 EKF 状态**（式 4）：
$$
\hat{\mathbf X}_k=\big[\hat{\mathbf X}_{IMU}^\top,\ {}^{C_1}_G\hat{\bar{\mathbf q}}^\top,\ {}^G\hat{\mathbf p}_{C_1}^\top,\ \cdots,\ {}^{C_N}_G\hat{\bar{\mathbf q}}^\top,\ {}^G\hat{\mathbf p}_{C_N}^\top\big]^\top, \tag{4}
$$
对应**完整误差态**（式 5）：
$$
\tilde{\mathbf X}_k=\big[\tilde{\mathbf X}_{IMU}^\top,\ \delta\boldsymbol\theta_{C_1}^\top,\ {}^G\tilde{\mathbf p}_{C_1}^\top,\ \cdots,\ \delta\boldsymbol\theta_{C_N}^\top,\ {}^G\tilde{\mathbf p}_{C_N}^\top\big]^\top. \tag{5}
$$
（每相机位姿 6 维，故 $\dim\tilde{\mathbf X}_k=15+6N$。）

## §MSCKF.B 传播（源 §III.B）

**连续时间 IMU 系统模型**（式 6）：
$$
{}^I_G\dot{\bar{\mathbf q}}(t)=\tfrac12\boldsymbol\Omega\big(\boldsymbol\omega(t)\big){}^I_G\bar{\mathbf q}(t),\quad \dot{\mathbf b}_g(t)=\mathbf n_{wg}(t),\quad {}^G\dot{\mathbf v}_I(t)={}^G\mathbf a(t),\quad \dot{\mathbf b}_a(t)=\mathbf n_{wa}(t),\quad {}^G\dot{\mathbf p}_I(t)={}^G\mathbf v_I(t), \tag{6}
$$
${}^G\mathbf a$=全局系体加速度，$\boldsymbol\omega=[\omega_x,\omega_y,\omega_z]^\top$=IMU 系角速度，$\boldsymbol\Omega(\boldsymbol\omega)=\begin{bmatrix}-\lfloor\boldsymbol\omega\rfloor_\times & \boldsymbol\omega\\ -\boldsymbol\omega^\top & 0\end{bmatrix}$。

**陀螺/加计测量模型**（式 7,8，**含地球自转 $\boldsymbol\omega_G$ 与重力 ${}^G\mathbf g$**）：
$$
\boldsymbol\omega_m=\boldsymbol\omega+\mathbf C({}^I_G\bar{\mathbf q})\boldsymbol\omega_G+\mathbf b_g+\mathbf n_g, \tag{7}
$$
$$
\mathbf a_m=\mathbf C({}^I_G\bar{\mathbf q})\big({}^G\mathbf a-{}^G\mathbf g+2\lfloor\boldsymbol\omega_G\rfloor_\times{}^G\mathbf v_I+\lfloor\boldsymbol\omega_G\rfloor_\times^2{}^G\mathbf p_I\big)+\mathbf b_a+\mathbf n_a, \tag{8}
$$
$\mathbf C(\cdot)$=旋转矩阵，$\mathbf n_g,\mathbf n_a$=零均值白高斯。

**状态估计传播方程**（对 (6) 取期望，式 9）：
$$
{}^I_G\dot{\hat{\bar{\mathbf q}}}=\tfrac12\boldsymbol\Omega(\hat{\boldsymbol\omega}){}^I_G\hat{\bar{\mathbf q}},\quad \dot{\hat{\mathbf b}}_g=\mathbf 0_{3\times1},\quad {}^G\dot{\hat{\mathbf v}}_I=\mathbf C_{\hat q}^\top\hat{\mathbf a}-2\lfloor\boldsymbol\omega_G\rfloor_\times{}^G\hat{\mathbf v}_I-\lfloor\boldsymbol\omega_G\rfloor_\times^2{}^G\hat{\mathbf p}_I+{}^G\mathbf g,\quad \dot{\hat{\mathbf b}}_a=\mathbf 0_{3\times1},\quad {}^G\dot{\hat{\mathbf p}}_I={}^G\hat{\mathbf v}_I, \tag{9}
$$
其中 $\mathbf C_{\hat q}=\mathbf C({}^I_G\hat{\bar{\mathbf q}})$、$\hat{\mathbf a}=\mathbf a_m-\hat{\mathbf b}_a$、$\hat{\boldsymbol\omega}=\boldsymbol\omega_m-\hat{\mathbf b}_g-\mathbf C_{\hat q}\boldsymbol\omega_G$。

**线性化连续时间 IMU 误差态模型**（式 10）：
$$
\dot{\tilde{\mathbf X}}_{IMU}=\mathbf F\,\tilde{\mathbf X}_{IMU}+\mathbf G\,\mathbf n_{IMU}, \tag{10}
$$
$\mathbf n_{IMU}=[\mathbf n_g^\top,\ \mathbf n_{wg}^\top,\ \mathbf n_a^\top,\ \mathbf n_{wa}^\top]^\top$=系统噪声，协方差 $\mathbf Q_{IMU}$（离线标定）。$\mathbf F,\mathbf G$（式 10 给出，按误差态序 $[\delta\boldsymbol\theta,\tilde{\mathbf b}_g,{}^G\tilde{\mathbf v},\tilde{\mathbf b}_a,{}^G\tilde{\mathbf p}]$）：
$$
\mathbf F=\begin{bmatrix}
-\lfloor\hat{\boldsymbol\omega}\rfloor_\times & -\mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
-\mathbf C_{\hat q}^\top\lfloor\hat{\mathbf a}\rfloor_\times & \mathbf 0_{3\times3} & -2\lfloor\boldsymbol\omega_G\rfloor_\times & -\mathbf C_{\hat q}^\top & -\lfloor\boldsymbol\omega_G\rfloor_\times^2\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}
\end{bmatrix},\qquad
\mathbf G=\begin{bmatrix}
-\mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & -\mathbf C_{\hat q}^\top & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf I_3\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}
\end{bmatrix}.
$$

**离散实现**（式 11–13）：IMU 以周期 $T$ 采样 $\boldsymbol\omega_m,\mathbf a_m$，用 **5 阶 RK** 数值积分 (9) 传播状态。协方差分块（式 11）：
$$
\mathbf P_{k|k}=\begin{bmatrix}\mathbf P_{II_{k|k}} & \mathbf P_{IC_{k|k}}\\ \mathbf P_{IC_{k|k}}^\top & \mathbf P_{CC_{k|k}}\end{bmatrix}, \tag{11}
$$
$\mathbf P_{II_{k|k}}$=$15\times15$ 演进 IMU 状态协方差，$\mathbf P_{CC_{k|k}}$=$6N\times6N$ 相机位姿协方差，$\mathbf P_{IC_{k|k}}$=两者相关。传播后协方差：
$$
\mathbf P_{k+1|k}=\begin{bmatrix}\mathbf P_{II_{k+1|k}} & \boldsymbol\Phi(t_k+T,t_k)\mathbf P_{IC_{k|k}}\\ \mathbf P_{IC_{k|k}}^\top\boldsymbol\Phi(t_k+T,t_k)^\top & \mathbf P_{CC_{k|k}}\end{bmatrix},
$$
$\mathbf P_{II_{k+1|k}}$ 由 **Lyapunov 方程**数值积分（式 12）：
$$
\dot{\mathbf P}_{II}=\mathbf F\mathbf P_{II}+\mathbf P_{II}\mathbf F^\top+\mathbf G\mathbf Q_{IMU}\mathbf G^\top, \tag{12}
$$
区间 $(t_k,t_k+T)$、初值 $\mathbf P_{II_{k|k}}$。状态转移矩阵 $\boldsymbol\Phi(t_k+T,t_k)$ 由微分方程数值积分（式 13）：
$$
\dot{\boldsymbol\Phi}(t_k+\tau,t_k)=\mathbf F\boldsymbol\Phi(t_k+\tau,t_k),\quad \tau\in[0,T],\quad \boldsymbol\Phi(t_k,t_k)=\mathbf I_{15}. \tag{13}
$$

## §MSCKF.C 状态增广（stochastic cloning，源 §III.C）

录新图时，由 IMU 位姿估计算相机位姿估计（式 14）：
$$
{}^C_G\hat{\bar{\mathbf q}}={}^C_I\bar{\mathbf q}\otimes{}^I_G\hat{\bar{\mathbf q}},\qquad {}^G\hat{\mathbf p}_C={}^G\hat{\mathbf p}_I+\mathbf C_{\hat q}^\top\,{}^I\mathbf p_C, \tag{14}
$$
${}^C_I\bar{\mathbf q}$=IMU↔相机旋转，${}^I\mathbf p_C$=相机原点相对 $\{I\}$ 位置（**外参，已知**）。此相机位姿估计**追加**到状态向量，协方差相应增广（式 15）：
$$
\mathbf P_{k|k}\leftarrow\begin{bmatrix}\mathbf I_{6N+15}\\ \mathbf J\end{bmatrix}\mathbf P_{k|k}\begin{bmatrix}\mathbf I_{6N+15}\\ \mathbf J\end{bmatrix}^\top, \tag{15}
$$
雅可比 $\mathbf J$（由 (14) 导出，式 16）：
$$
\mathbf J=\begin{bmatrix}
\mathbf C({}^C_I\bar{\mathbf q}) & \mathbf 0_{3\times9} & \mathbf 0_{3\times3} & \mathbf 0_{3\times6N}\\
\lfloor\mathbf C_{\hat q}^\top\,{}^I\mathbf p_C\rfloor_\times & \mathbf 0_{3\times9} & \mathbf I_3 & \mathbf 0_{3\times6N}
\end{bmatrix}. \tag{16}
$$
（这就是"stochastic cloning"：把当前相机位姿克隆进状态，并正确传递其与已有状态的相关性。）

## §MSCKF.D 测量模型（源 §III.D）——核心贡献

**动机（源）**：从多相机位姿观测静止特征会产生涉及所有这些位姿的约束。MSCKF **按跟踪特征分组**观测（而非按位姿对），用同一 3D 点的所有测量定义一个约束方程，关联所有观测它的相机位姿——**而不把特征位置放入滤波状态**。

考虑单特征 $f_j$ 被 $M_j$ 个相机位姿 $({}^{C_i}_G\bar{\mathbf q},{}^G\mathbf p_{C_i}),i\in\mathcal S_j$ 观测。每观测模型（式 18，归一化像素）：
$$
\mathbf z_i^{(j)}=\frac{1}{{}^{C_i}Z_j}\begin{bmatrix}{}^{C_i}X_j\\ {}^{C_i}Y_j\end{bmatrix}+\mathbf n_i^{(j)},\quad i\in\mathcal S_j, \tag{18}
$$
$\mathbf n_i^{(j)}$=$2\times1$ 图像噪声，协方差 $\mathbf R_i^{(j)}=\sigma_{im}^2\mathbf I_2$。特征在相机系位置（式 19）：
$$
{}^{C_i}\mathbf p_{f_j}=\begin{bmatrix}{}^{C_i}X_j\\ {}^{C_i}Y_j\\ {}^{C_i}Z_j\end{bmatrix}=\mathbf C({}^{C_i}_G\bar{\mathbf q})\big({}^G\mathbf p_{f_j}-{}^G\mathbf p_{C_i}\big), \tag{19}
$$
${}^G\mathbf p_{f_j}$=特征全局 3D 位置（未知）→ **第一步先用最小二乘三角化得估计 ${}^G\hat{\mathbf p}_{f_j}$**（用观测 $\mathbf z_i^{(j)}$ 与各时刻相机位姿估计，详见附录/§MSCKF.G）。

得特征位置估计后，计算测量残差（式 20）：
$$
\mathbf r_i^{(j)}=\mathbf z_i^{(j)}-\hat{\mathbf z}_i^{(j)},\quad \hat{\mathbf z}_i^{(j)}=\frac{1}{{}^{C_i}\hat Z_j}\begin{bmatrix}{}^{C_i}\hat X_j\\ {}^{C_i}\hat Y_j\end{bmatrix},\quad \begin{bmatrix}{}^{C_i}\hat X_j\\ {}^{C_i}\hat Y_j\\ {}^{C_i}\hat Z_j\end{bmatrix}=\mathbf C({}^{C_i}_G\hat{\bar{\mathbf q}})\big({}^G\hat{\mathbf p}_{f_j}-{}^G\hat{\mathbf p}_{C_i}\big). \tag{20}
$$
对相机位姿与特征位置线性化（式 21）：
$$
\mathbf r_i^{(j)}\simeq\mathbf H_{X_i}^{(j)}\tilde{\mathbf X}+\mathbf H_{f_i}^{(j)}\,{}^G\tilde{\mathbf p}_{f_j}+\mathbf n_i^{(j)}, \tag{21}
$$
$\mathbf H_{X_i}^{(j)},\mathbf H_{f_i}^{(j)}$=测量对状态、对特征位置的雅可比（精确值见 [21]），${}^G\tilde{\mathbf p}_{f_j}$=特征位置估计误差。堆叠该特征的全部 $M_j$ 个观测（式 22）：
$$
\mathbf r^{(j)}\simeq\mathbf H_X^{(j)}\tilde{\mathbf X}+\mathbf H_f^{(j)}\,{}^G\tilde{\mathbf p}_{f_j}+\mathbf n^{(j)}, \tag{22}
$$
$\mathbf r^{(j)},\mathbf H_X^{(j)},\mathbf H_f^{(j)},\mathbf n^{(j)}$ 为块向量/矩阵；不同图像观测独立 → $\mathbf R^{(j)}=\sigma_{im}^2\mathbf I_{2M_j}$。

## §MSCKF.E 零空间投影（源 §III.D 末）——MSCKF 灵魂

**问题**：因状态估计 $\tilde{\mathbf X}$ 被用于算特征位置估计 → (22) 中 ${}^G\tilde{\mathbf p}_{f_j}$ 与状态误差 $\tilde{\mathbf X}$ **相关** → $\mathbf r^{(j)}$ 不符合 EKF 标准形 $\mathbf r=\mathbf H\tilde{\mathbf X}+\text{noise}$，不能直接用于更新。

**解法（零空间投影）**：把 $\mathbf r^{(j)}$ 投到 $\mathbf H_f^{(j)}$ 的**左零空间**。设 $\mathbf A$ 是其列张成 $\mathbf H_f^{(j)}$ 左零空间的酉矩阵（即 $\mathbf A^\top\mathbf H_f^{(j)}=\mathbf 0$），则（式 23,24）：
$$
\mathbf r_o^{(j)}=\mathbf A^\top(\mathbf z^{(j)}-\hat{\mathbf z}^{(j)})\simeq\mathbf A^\top\mathbf H_X^{(j)}\tilde{\mathbf X}+\mathbf A^\top\mathbf n^{(j)}=\mathbf H_o^{(j)}\tilde{\mathbf X}+\mathbf n_o^{(j)}. \tag{23,24}
$$
因 $2M_j\times3$ 矩阵 $\mathbf H_f^{(j)}$ 满列秩 → 其左零空间维数 $2M_j-3$ → $\mathbf r_o^{(j)}$ 是 $(2M_j-3)\times1$ 向量。**此残差与特征坐标误差无关，故可做 EKF 更新**。(24) 定义了被 $f_j$ 观测的所有相机位姿间的线性约束，表达 $\mathbf z_i^{(j)}$ 提供的关于 $M_j$ 个状态的全部信息 → EKF 更新最优（除线性化误差）。

**实现优化（源）**：$\mathbf A$ 无需显式构造——$\mathbf r$ 与 $\mathbf H_X^{(j)}$ 在 $\mathbf H_f^{(j)}$ 零空间上的投影可用 **Givens 旋转**在 $O(M_j^2)$ 算得。又 $\mathbf A$ 酉 → 噪声协方差（式，$E\{\mathbf n_o^{(j)}\mathbf n_o^{(j)\top}\}=\sigma_{im}^2\mathbf A^\top\mathbf A=\sigma_{im}^2\mathbf I_{2M_j-3}$）。

> **替代（源讨论，反面）**：也可对 $M_j(M_j-1)/2$ 个图像对用对极约束——但仍只对应 $2M_j-3$ 个独立约束（每测量被多次用 → 统计相关）；实验表明对极约束线性化实现复杂得多、结果更差。

## §MSCKF.F EKF 更新（源 §III.E）——含 QR 压缩

**更新触发**（源）：(a) 一个跟踪多帧的特征不再被检测（特征移出视野，最常见）→ 处理其所有测量；(b) 每录新图克隆相机位姿，若达上限 $N_{max}$ 必删旧位姿——删前用其全部特征观测的约束做 EKF 更新（选 $N_{max}/3$ 个时间均匀分布、从次旧位姿起的位姿删除；**总保留最旧位姿**，因更早位姿涉及更大基线、定位信息更有价值）。

设处理 $L$ 个特征，按上述对每特征算 $\mathbf r_o^{(j)},\mathbf H_o^{(j)}$，堆叠为 $\mathbf r_o,\mathbf H_X$（式 25）。各特征测量独立 → $\mathbf n_o$ 不相关，$\mathbf R_o=\sigma_{im}^2\mathbf I_d,\ d=\sum_{j=1}^L(2M_j-3)$。

**问题**：$d$ 可能很大（如 10 特征各 10 位姿 → $d=170$）。**用 $\mathbf H_X$ 的 QR 分解**降复杂度（式）：
$$
\mathbf H_X=\begin{bmatrix}\mathbf Q_1 & \mathbf Q_2\end{bmatrix}\begin{bmatrix}\mathbf T_H\\ \mathbf 0\end{bmatrix},
$$
$\mathbf Q_1,\mathbf Q_2$ 酉（列张成 $\mathbf H_X$ 的值域、零空间），$\mathbf T_H$ 上三角。代入 (25) 得（式 26,27）：
$$
\mathbf r_o=\begin{bmatrix}\mathbf Q_1 & \mathbf Q_2\end{bmatrix}\begin{bmatrix}\mathbf T_H\\ \mathbf 0\end{bmatrix}\tilde{\mathbf X}+\mathbf n_o\ \Rightarrow\ \begin{bmatrix}\mathbf Q_1^\top\mathbf r_o\\ \mathbf Q_2^\top\mathbf r_o\end{bmatrix}=\begin{bmatrix}\mathbf T_H\\ \mathbf 0\end{bmatrix}\tilde{\mathbf X}+\begin{bmatrix}\mathbf Q_1^\top\mathbf n_o\\ \mathbf Q_2^\top\mathbf n_o\end{bmatrix}. \tag{26,27}
$$
$\mathbf Q_2^\top\mathbf r_o$ 全是噪声、可弃 → **用于 EKF 更新的残差**（式 28）：
$$
\mathbf r_n=\mathbf Q_1^\top\mathbf r_o=\mathbf T_H\tilde{\mathbf X}+\mathbf n_n, \tag{28}
$$
$\mathbf n_n=\mathbf Q_1^\top\mathbf n_o$，协方差 $\mathbf R_n=\mathbf Q_1^\top\mathbf R_o\mathbf Q_1=\sigma_{im}^2\mathbf I_r$（$r$=$\mathbf Q_1$ 列数）。**卡尔曼增益**（式 29）：
$$
\mathbf K=\mathbf P\mathbf T_H^\top\big(\mathbf T_H\mathbf P\mathbf T_H^\top+\mathbf R_n\big)^{-1}, \tag{29}
$$
**状态校正**（式 30）：$\Delta\mathbf X=\mathbf K\mathbf r_n$。**协方差更新（Joseph 形式）**（式 31）：
$$
\mathbf P_{k+1|k+1}=(\mathbf I_\xi-\mathbf K\mathbf T_H)\mathbf P_{k+1|k}(\mathbf I_\xi-\mathbf K\mathbf T_H)^\top+\mathbf K\mathbf R_n\mathbf K^\top, \tag{31}
$$
$\xi=6N+15$。复杂度：$\mathbf r_n,\mathbf T_H$ 用 Givens 旋转 $O(r^2d)$ 算（无需显式 $\mathbf Q_1$）；(31) 涉及 $\xi$ 维方阵乘。

## §MSCKF.G 特征三角化（源附录）

第一步用最小二乘求 ${}^G\hat{\mathbf p}_{f_j}$（**逆深度参数化**，避免远特征数值病态）：以首次观测帧 $C_n$ 为参考，设特征在 $C_n$ 系坐标 $(\alpha,\beta,\rho)=(X/Z,\ Y/Z,\ 1/Z)$，对每观测帧写重投影、组成非线性最小二乘 $\min_{\alpha,\beta,\rho}\sum_i\|\mathbf z_i^{(j)}-\hat{\mathbf z}_i^{(j)}(\alpha,\beta,\rho)\|^2$，高斯-牛顿求解，再恢复 ${}^G\hat{\mathbf p}_{f_j}={}^{C_n}_G\hat{\mathbf C}^\top\tfrac1\rho[\alpha,\beta,1]^\top+{}^G\hat{\mathbf p}_{C_n}$。

---

# 第三部分：Forster 流形预积分理论（辅源 [Forster]，补 VINS 预积分缺口）

> 来源：Forster et al., RSS 2015 / TRO 2017 (arXiv:1512.02363)。**VINS 的后验偏置校正直接基于本理论**（VINS 文献 [23]）。本部分给 $\mathrm{SO}(3)$ 上完整 A/B 协方差阵、bias 雅可比解析式、IMU 因子残差——补 VINS 式 9–12 未展开的闭式。记号见 §0.3（右扰动 + $\mathbf J_r$）。

## §Forster.0 预备：$\mathrm{SO}(3)$ 工具（源 §II）

**Exp/Log**（式 3,5,6）：$\exp(\boldsymbol\phi^\wedge)=\mathbf I+\tfrac{\sin\|\boldsymbol\phi\|}{\|\boldsymbol\phi\|}\boldsymbol\phi^\wedge+\tfrac{1-\cos\|\boldsymbol\phi\|}{\|\boldsymbol\phi\|^2}(\boldsymbol\phi^\wedge)^2$（Rodrigues）；一阶近似 $\exp(\boldsymbol\phi^\wedge)\approx\mathbf I+\boldsymbol\phi^\wedge$（式 4）。$\mathrm{Exp}:\mathbb R^3\to\mathrm{SO}(3)$、$\mathrm{Log}:\mathrm{SO}(3)\to\mathbb R^3$（式 6）。

**右雅可比 $\mathbf J_r$**（源 Fig. 2，式 7–9）——本书右扰动主线核心：
$$
\mathrm{Exp}(\boldsymbol\phi+\delta\boldsymbol\phi)\approx\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}\big(\mathbf J_r(\boldsymbol\phi)\,\delta\boldsymbol\phi\big), \tag{7}
$$
$$
\mathrm{Log}\big(\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\delta\boldsymbol\phi)\big)\approx\boldsymbol\phi+\mathbf J_r^{-1}(\boldsymbol\phi)\,\delta\boldsymbol\phi, \tag{9}
$$
$\mathbf J_r(\boldsymbol\phi)$ 闭式（[31] Chirikjian p.40，式 8）：$\mathbf J_r(\boldsymbol\phi)=\mathbf I-\tfrac{1-\cos\|\boldsymbol\phi\|}{\|\boldsymbol\phi\|^2}\boldsymbol\phi^\wedge+\tfrac{\|\boldsymbol\phi\|-\sin\|\boldsymbol\phi\|}{\|\boldsymbol\phi\|^3}(\boldsymbol\phi^\wedge)^2$。伴随性质（式 10,11）：$\mathbf R\,\mathrm{Exp}(\boldsymbol\phi)\,\mathbf R^\top=\mathrm{Exp}(\mathbf R\boldsymbol\phi)$，即 $\mathrm{Exp}(\boldsymbol\phi)\mathbf R=\mathbf R\,\mathrm{Exp}(\mathbf R^\top\boldsymbol\phi)$。

**$\mathrm{SO}(3)$ 不确定度**（式 12,13）：$\tilde{\mathbf R}=\mathbf R\,\mathrm{Exp}(\boldsymbol\epsilon),\ \boldsymbol\epsilon\sim\mathcal N(\mathbf 0,\boldsymbol\Sigma)$（**右乘扰动，与本书一致**）。负对数似然（式 14）：$\mathcal L(\mathbf R)=\tfrac12\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\|^2_{\boldsymbol\Sigma}+\text{const}$。

**$\mathrm{SE}(3)$ retraction（关键选择）**（源 §II.C，式 在 T=(R,p)）：$\mathcal R_{\mathbf T}(\delta\boldsymbol\phi,\delta\mathbf p)=(\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi),\ \mathbf p+\mathbf R\,\delta\mathbf p)$——**故只需 $\mathrm{SO}(3)$ 的 Exp，不必算 $\mathrm{SE}(3)$ 指数**。

## §Forster.A 运动积分与预积分定义（源 §IV–V）

**IMU 模型**（式 20,21）：
$$
{}_B\tilde{\boldsymbol\omega}_{WB}(t)={}_B\boldsymbol\omega_{WB}(t)+\mathbf b^g(t)+\boldsymbol\eta^g(t),\qquad
{}_B\tilde{\mathbf a}(t)=\mathbf R_{WB}^\top(t)\big({}_W\mathbf a(t)-{}_W\mathbf g\big)+\mathbf b^a(t)+\boldsymbol\eta^a(t). \tag{20,21}
$$
**运动学**（式 22）：$\dot{\mathbf R}_{WB}=\mathbf R_{WB}\,{}_B\boldsymbol\omega_{WB}^\wedge,\ {}_W\dot{\mathbf v}={}_W\mathbf a,\ {}_W\dot{\mathbf p}={}_W\mathbf v$。欧拉积分（式 23,24，假设区间内 $\mathbf a,\boldsymbol\omega$ 常）：
$$
\begin{aligned}
\mathbf R(t+\Delta t)&=\mathbf R(t)\,\mathrm{Exp}\big((\tilde{\boldsymbol\omega}(t)-\mathbf b^g(t)-\boldsymbol\eta^{gd}(t))\Delta t\big),\\
\mathbf v(t+\Delta t)&=\mathbf v(t)+\mathbf g\Delta t+\mathbf R(t)(\tilde{\mathbf a}(t)-\mathbf b^a(t)-\boldsymbol\eta^{ad}(t))\Delta t,\\
\mathbf p(t+\Delta t)&=\mathbf p(t)+\mathbf v(t)\Delta t+\tfrac12\mathbf g\Delta t^2+\tfrac12\mathbf R(t)(\tilde{\mathbf a}(t)-\mathbf b^a(t)-\boldsymbol\eta^{ad}(t))\Delta t^2.
\end{aligned}\tag{24}
$$
离散↔连续噪声：$\mathrm{Cov}(\boldsymbol\eta^{gd})=\tfrac1{\Delta t}\mathrm{Cov}(\boldsymbol\eta^g)$（$\boldsymbol\eta^{ad}$ 同）。

迭代 $i\to j$（式 25）后，**定义与 $t_i$ 处位姿/速度无关的相对运动增量**（式 26）——这是预积分核心：
$$
\boxed{\;
\begin{aligned}
\Delta\mathbf R_{ij}&\triangleq\mathbf R_i^\top\mathbf R_j=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b^g_k-\boldsymbol\eta^{gd}_k)\Delta t\big),\\
\Delta\mathbf v_{ij}&\triangleq\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})=\sum_{k=i}^{j-1}\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t,\\
\Delta\mathbf p_{ij}&\triangleq\mathbf R_i^\top\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)=\sum_{k=i}^{j-1}\big[\Delta\mathbf v_{ik}\Delta t+\tfrac12\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t^2\big].
\end{aligned}\;}\tag{26}
$$
$\Delta t_{ij}=\sum_{k=i}^{j-1}\Delta t$。**动机（反面，源 §IV）**：(25) 的积分在 $t_i$ 处线性化点（$\mathbf R_i$）改变时须重做（$\mathbf R_i$ 变 → 所有未来 $\mathbf R_k$ 变 → 重算所有求和/连乘）。(26) 的相对增量避免重复积分。

## §Forster.B 噪声分离与协方差传播（源 §V.A,B）——补 VINS 缺口

**假设两关键帧间偏置常**（式 27）：$\mathbf b^g_i=\cdots=\mathbf b^g_{j-1}$，$\mathbf b^a_i=\cdots=\mathbf b^a_{j-1}$。

**旋转增量分离噪声**（式 28，用 (7) 一阶近似 + (11) 把噪声"移到末尾"）：
$$
\Delta\mathbf R_{ij}\overset{(7)}{\approx}\prod_{k=i}^{j-1}\Big[\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t\big)\mathrm{Exp}\big(-\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t\big)\Big]\overset{(11)}{=}\Delta\tilde{\mathbf R}_{ij}\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t\big)\triangleq\Delta\tilde{\mathbf R}_{ij}\,\mathrm{Exp}(-\delta\boldsymbol\phi_{ij}), \tag{28}
$$
$\mathbf J_r^k=\mathbf J_r((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)$，**预积分旋转测量** $\Delta\tilde{\mathbf R}_{ij}\triangleq\prod_{k=i}^{j-1}\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)$。取 Log 并反复用 (9)（式 33,34）：
$$
\delta\boldsymbol\phi_{ij}\approx\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t. \tag{34}
$$
**速度、位置增量噪声**（式 29,30，代入 (28) + 一阶近似 + $(\mathbf a^\wedge)\delta\boldsymbol\phi=-(\delta\boldsymbol\phi)^\wedge\mathbf a$ 性质）：
$$
\Delta\mathbf v_{ij}\approx\Delta\tilde{\mathbf v}_{ij}-\delta\mathbf v_{ij},\quad \delta\mathbf v_{ij}=\sum_{k=i}^{j-1}\big[\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t+\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t\big], \tag{29}
$$
$$
\Delta\mathbf p_{ij}\approx\Delta\tilde{\mathbf p}_{ij}-\delta\mathbf p_{ij},\quad \delta\mathbf p_{ij}=\sum_{k=i}^{j-1}\big[\delta\mathbf v_{ik}\Delta t-\tfrac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2+\tfrac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t^2\big]. \tag{30}
$$
**预积分测量模型**（式 31，"真值 plus 噪声"）：
$$
\Delta\tilde{\mathbf R}_{ij}=\mathbf R_i^\top\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_{ij}),\quad \Delta\tilde{\mathbf v}_{ij}=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})+\delta\mathbf v_{ij},\quad \Delta\tilde{\mathbf p}_{ij}=\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)+\delta\mathbf p_{ij}. \tag{31}
$$
噪声 $[\delta\boldsymbol\phi_{ij}^\top,\delta\mathbf v_{ij}^\top,\delta\mathbf p_{ij}^\top]^\top\sim\mathcal N(\mathbf 0_{9\times1},\boldsymbol\Sigma_{ij})$（式 35）。**协方差 $\boldsymbol\Sigma_{ij}$ 可增量计算**（线性模型 $\boldsymbol\eta_{ij}=\mathbf A_{j-1}\boldsymbol\eta_{i\,j-1}+\mathbf B_{j-1}\boldsymbol\eta^d_{j-1}$ → $\boldsymbol\Sigma_{ij}=\mathbf A_{j-1}\boldsymbol\Sigma_{i\,j-1}\mathbf A_{j-1}^\top+\mathbf B_{j-1}\boldsymbol\Sigma^d\mathbf B_{j-1}^\top$，$\mathbf A,\mathbf B$ 闭式见补充材料 [29]，对应 VINS 式 10 的离散递推但在 $\mathrm{SO}(3)$ 解析）。

## §Forster.C 偏置更新一阶校正（源 §V.C）——VINS 后验 bias 校正之源

偏置在优化中变 $\mathbf b\leftarrow\bar{\mathbf b}+\delta\mathbf b$，用**一阶展开**更新预积分量（式 36，**避免重积分**）：
$$
\begin{aligned}
\Delta\tilde{\mathbf R}_{ij}(\mathbf b^g_i)&\approx\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathrm{Exp}\Big(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g\Big),\\
\Delta\tilde{\mathbf v}_{ij}(\mathbf b^g_i,\mathbf b^a_i)&\approx\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a,\\
\Delta\tilde{\mathbf p}_{ij}(\mathbf b^g_i,\mathbf b^a_i)&\approx\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a.
\end{aligned}\tag{36}
$$
（直接在 $\mathrm{SO}(3)$ 上操作，胜 [26]。雅可比在 $\bar{\mathbf b}_i$ 算、**预积分时即可预计算、保持常数**——对应 VINS 式 12 的 $\mathbf J^\alpha_{b_a}$ 等。）

**偏置随机游走因子**（式 38–40）：$\dot{\mathbf b}^g=\boldsymbol\eta^{bg},\dot{\mathbf b}^a=\boldsymbol\eta^{ba}$；积分得 $\mathbf b^g_j=\mathbf b^g_i+\boldsymbol\eta^{bgd},\mathbf b^a_j=\mathbf b^a_i+\boldsymbol\eta^{bad}$（式 39，$\boldsymbol\Sigma^{bgd}=\Delta t_{ij}\mathrm{Cov}(\boldsymbol\eta^{bg})$）；因子 $\|\mathbf r^b_{ij}\|^2=\|\mathbf b^g_j-\mathbf b^g_i\|^2_{\boldsymbol\Sigma^{bgd}}+\|\mathbf b^a_j-\mathbf b^a_i\|^2_{\boldsymbol\Sigma^{bad}}$（式 40）。

## §Forster.D IMU 因子残差（源 §V.D）

预积分 IMU 因子残差 $\mathbf r_{\mathcal I_{ij}}=[\mathbf r_{\Delta\mathbf R_{ij}}^\top,\mathbf r_{\Delta\mathbf v_{ij}}^\top,\mathbf r_{\Delta\mathbf p_{ij}}^\top]^\top\in\mathbb R^9$（式 37，含 (36) 偏置更新）：
$$
\begin{aligned}
\mathbf r_{\Delta\mathbf R_{ij}}&=\mathrm{Log}\Big(\big[\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\mathrm{Exp}(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g)\big]^\top\mathbf R_i^\top\mathbf R_j\Big),\\
\mathbf r_{\Delta\mathbf v_{ij}}&=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-\big[\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\big],\\
\mathbf r_{\Delta\mathbf p_{ij}}&=\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-\big[\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\big].
\end{aligned}\tag{37}
$$
（对照 VINS 式 24：结构同——旋转残差用 $\mathrm{Log}$（VINS 用四元数 $[\cdot]_{xyz}$）、速度/位置残差是真值减预积分测量。这是优化主线两套等价写法。）

## §Forster.E Structureless 视觉因子（源 §VI）——与 MSCKF 零空间投影同源！

视觉残差（式 42）：$\mathbf r_{C_{il}}=\mathbf z_{il}-\pi(\mathbf R_i,\mathbf p_i,\boldsymbol\rho_l)$，$\boldsymbol\rho_l$=路标位置。**直接用 (42) 须把所有路标 $\boldsymbol\rho_l$ 放入优化** → 计算大。故用**无结构（structureless）法消去路标**（保证仍得 MAP）：

每 GN 迭代线性化（式 44）：$\sum_l\sum_{i\in\mathcal X(l)}\|\mathbf F_{il}\delta\mathbf T_i+\mathbf E_{il}\delta\boldsymbol\rho_l-\mathbf b_{il}\|^2$。堆叠（式 45）后**把残差投到 $\mathbf E_l$ 的零空间消去 $\delta\boldsymbol\rho_l$**（式 46）：
$$
\sum_{l=1}^L\big\|\mathbf Q(\mathbf F_l\delta\mathbf T_{\mathcal X(l)}-\mathbf b_l)\big\|^2,\qquad \mathbf Q\triangleq\mathbf I-\mathbf E_l(\mathbf E_l^\top\mathbf E_l)^{-1}\mathbf E_l^\top, \tag{46}
$$
$\mathbf Q$=$\mathbf E_l$ 的正交投影补。**把含位姿+路标的大因子集 (43) 化为只含位姿的 $L$ 个小因子 (46)**。

> **关键洞察（连接两条主线！）**：Forster 优化主线的 structureless 视觉因子用的**零空间投影 $\mathbf Q=\mathbf I-\mathbf E(\mathbf E^\top\mathbf E)^{-1}\mathbf E^\top$，与 MSCKF 滤波主线的零空间投影 $\mathbf A^\top$（$\mathbf A^\top\mathbf H_f=0$）是同一线性代数操作**——都是"消去/边缘化路标，只保留位姿间约束"。差别仅在：MSCKF 一次性线性化后投影（不再 relinearize 特征 → 近似变差，Huang-Rev §3.1 指出此为滤波主线理论局限）；Forster **每次 GN 迭代都重做消元 → 仍得 MAP 最优**。这是"滤波 vs 优化"对比的数学本质（见 §Compare）。

---

# 第四部分：VIO 可观性、松/紧耦合、滤波 vs 优化（辅源 [Huang-Rev]，本章「VIO 问题与可观性」+「对比」核心）

> 来源：G. Huang, *Visual-Inertial Navigation: A Concise Review*, ICRA 2019 / arXiv:1906.02650.

## §Obs VINS 可观性分析（源 §5）

**可观性意义**：可观性检验"可用测量提供的信息是否足以无歧义估计状态"。系统可观 ⇔ 可观性矩阵可逆，与 **Fisher 信息（协方差）矩阵**密切相关。研究其零空间 → 洞察状态空间中估计器应/不应获取信息的方向。

**非线性可观性结论（源 §5）**：
- Martinelli [122] 用连续对称性导出 VINS 闭式解：**IMU 偏置、3D 速度、全局 roll/pitch 角可观**。
- Hesch et al. [51] 用李导数 + 可观性矩阵秩检验**解析证明单目 VINS 有 4 个不可观方向**：对应**全局 yaw + 外感受传感器的全局位置**。
- 外参（IMU↔相机变换）在一般运动下可观；RGBD（点+面）保持同样不可观方向。

**线性化 VINS 可观性矩阵与零空间**（实际更重要，因估计器建在线性化系统上，式 18）。区间 $[k_o,k]$ 上可观性矩阵 $\mathbf M$ 的零空间（不可观子空间）**理想张成 4 维**：
$$
\mathbf M=\begin{bmatrix}\mathbf H_{k_o}\\ \mathbf H_{k_o+1}\boldsymbol\Phi_{k_o}\\ \vdots\\ \mathbf H_k\boldsymbol\Phi_{k-1}\cdots\boldsymbol\Phi_{k_o}\end{bmatrix},\qquad
\mathbf M\,\mathbf N=\mathbf 0\ \Rightarrow\ \mathbf N=\begin{bmatrix}\mathbf 0_3 & \mathbf C({}^I_G\bar{\mathbf q}_k){}^G\mathbf g\\ \mathbf 0_3 & \mathbf 0_3\\ \mathbf 0_3 & -\lfloor{}^G\mathbf v_k\rfloor_\times{}^G\mathbf g\\ \mathbf 0_3 & \mathbf 0_3\\ \mathbf I_3 & -\lfloor{}^G\mathbf p_k\rfloor_\times{}^G\mathbf g\\ \mathbf I_3 & -\lfloor{}^G\mathbf p_f\rfloor_\times{}^G\mathbf g\end{bmatrix}. \tag{18}
$$
（行块按 $[\delta\boldsymbol\theta;\tilde{\mathbf b}_g;{}^G\tilde{\mathbf v};\tilde{\mathbf b}_a;{}^G\tilde{\mathbf p}_I;{}^G\tilde{\mathbf p}_f]$。）**$\mathbf N$ 第一列块对应全局平移（3 维）；第二列块对应绕重力向量的全局旋转（yaw，1 维）** → 共 4 维不可观。

**一致性问题（FEJ/OC，源 §5）**：设计 VINS 估计器时，**希望估计器所用系统模型的不可观子空间恰由这 4 方向张成**。但**标准 EKF 不满足**——标准 EKF 在当前状态估计处线性化，其线性化系统不可观子空间**只有 3 维（而非 4 维）** → 滤波器从可用测量获取**不存在的信息（spurious information）** → 不一致（协方差过小、误差变大、发散）。修法：
- **FEJ（First-Estimates Jacobian）** [47]：改善 MSCKF 一致性 [19,40]——所有涉及某变量的雅可比在该变量**首次估计**处求，保持不可观维数正确。
- **OC（Observability-Constrained）方法** [48]：开发 OC-VINS [38,39,50]，显式约束估计器线性化系统具正确可观性。
- **R-VIO（robocentric VIO）** [22,63]：机器人中心化表述，**不依赖线性化点即保持正确可观性**。
- **(右)不变卡尔曼滤波 [56]**、**迭代 EKF**：用于改善滤波一致性。

## §Obs.2 退化运动（degenerate motion）——本章可观性重点

源 §5 与 VINS-Mono §X 共同指出单目 VINS 在特殊运动下达到**弱可观甚至退化**，额外方向变不可观（汇总自 Martinelli [156]、Huang-Rev、社区共识）：

| 退化运动 | 额外不可观量 | 直觉 |
|---|---|---|
| **无任何旋转**（纯平移） | **三个全局旋转（roll/pitch/yaw）全不可观** | 无旋转激励 → 重力方向（roll/pitch）无法由 IMU 与视觉约束确定。 |
| **常加速度**（匀加速直线 / 加速度恒定） | **尺度不可观** | 尺度可观依赖加速度的"变化"激励；常加速度下视觉与 IMU 的尺度耦合退化。 |
| **匀速直线**（零加速度，无激励） | **尺度不可观**（且本就是常加速度的特例 $\mathbf a=0$） | 无加速度激励 → 度量尺度无法恢复（VINS-Mono §I 明言"加速度激励才能使尺度可观→不能静止起步"）。 |
| **单轴旋转** | 该轴外的部分姿态/外参方向可能弱可观 | 仅一个旋转轴被激励 → 另两轴信息不足。 |
| **静止** | 尺度 + 全局位置 + （视情）部分姿态 | 静止 = 零激励，单目 VINS 退化最严重——故须在运动中初始化。 |

> **本章「VIO 问题与可观性」可直接采用**：① 一般运动下 4 维恒不可观（全局位置 3 + yaw 1）→ 这是 VIO 作为里程计**长期漂移仅 4-DOF** 的根因，也解释 VINS-Mono 为何只做 4-DOF 位姿图（§VIII）；② 退化运动下额外维数不可观（尺度需加速度激励、姿态需旋转激励）→ 解释为何初始化最脆弱（§V）、为何静止不能起步。

## §Compare 松/紧耦合 与 滤波/优化（源 §3）——本章「对比」核心

### §Compare.1 滤波 vs 优化（源 §3.1）

| 维度 | 滤波（filtering，代表 MSCKF [18]） | 优化（optimization，代表 OKVIS [28]/VINS [31]） |
|---|---|---|
| 原理 | EKF：IMU 做状态传播，视觉测量做更新；MSCKF 把特征**零空间边缘化**（线性边缘化 [44]），只保留约束随机克隆的相机位姿 | 解非线性最小二乘（BA）；滑窗 + 边缘化过去状态/测量保常数处理时间 |
| 再线性化 | **测量只一次性线性化**（处理前）→ 大线性化误差风险、退化性能 | **可 relinearize** [79,80] → 误差更小，但计算更高 |
| 特征处理 | 不共估上千特征 → 计算省；但**阻止后续 relinearize 特征非线性测量 → 近似变差** | 可在窗内反复 relinearize |
| 复杂度 | 与特征数**线性**（MSCKF 核心优势） | 高（迭代解非线性系统）；少数图优化能在资源受限平台实时 |
| 一致性 | 标准 EKF 不可观维数错（3 而非 4）→ 需 FEJ/OC/不变滤波/SR-ISWF [6,53] 修 | 边缘化也致线性化点固定 → 需 FEJ 修；但活动窗内仍 relinearize |
| 代表扩展 | OC-MSCKF、SR-ISWF（平方根逆，单精度、移动端）、OSC-EKF、(右)不变 KF、迭代 EKF、R-VIO、线/面特征、滚动快门、RGBD、多相机/多 IMU | iSAM/iSAM2 增量平滑、OKVIS 关键帧、VINS-Mono（回环入局部窗+全局批优化）、Schmidt-MSCKF（单线程回环） |

**理论局限对比（源原话）**：滤波"非线性测量(12)须一次性线性化 → 可能引入大线性化误差、降性能"；优化"可通过 relinearize 减误差但计算高"。**这正是两条主线的根本权衡**。

### §Compare.2 紧耦合 vs 松耦合（源 §3.2）

- **松耦合**：在滤波或优化框架下，**分别处理**视觉与惯性测量推各自运动约束，再融合这些约束（[27,87–91]）。计算高效，但**解耦视觉与惯性约束 → 信息损失**。
- **紧耦合**：在单一过程内**直接融合**视觉与惯性原始测量（[18,28,34,40,85,92]）→ **更高精度**。MSCKF、OKVIS、VINS-Mono 皆紧耦合。

> VINS-Mono **初始化用松耦合**（§V：先纯视觉 SfM 再与 IMU 对齐）、**VIO 主体用紧耦合**（§VI：(22) 联合优化 IMU + 视觉残差）——是两者的实用组合。

### §Compare.3 VIO vs SLAM（源 §3.3）

- **VI-SLAM**（[28,33,85]）：**把特征位置与相机/IMU 位姿一起放入状态向量**联合估计 → 易纳回环 → 误差有界，但复杂度高。
- **VIO**（[18,22,39,40,52,95]，含 MSCKF / VINS-Mono 局部）：**不把特征放入状态**，但仍用视觉测量在位姿间施约束 → 本质是里程计（dead reckoning），**误差可能无界增长**，除非用全局信息（GPS/先验地图）或回环约束。
- 多数系统用**两线程**：局部窗优化少量"local"关键帧+特征（短期限漂移）+ 后台长期稀疏位姿图含回环约束（长期一致）[31,83,99,100]。**VINS-Mono [31,100] 在局部滑窗与全局批优化都用回环约束**：局部优化时关键帧的特征观测提供隐式回环约束，并**假设关键帧位姿完美（移出优化）以保问题规模小**。
- 回环是 VIO 与 SLAM 关键区别之一；难点：在不做不一致假设（如把关键帧位姿当真值、重用信息）下保持计算高效。混合估计器 [109]（MSCKF 实时局部 + 回环触发全局 BA）、大尺度地图基 VINS [110]、Cholesky-Schmidt EKF [102]、Schmidt-MSCKF [86] 皆尝试解决。

---

# §X 本抽取覆盖小结与未尽部分（comprehensiveness）

**已全量保真覆盖**：
1. **VINS-Mono 全文 §I–§X**：所有公式（式 1–29）逐式 LaTeX，含 IMU 预积分（1–13）、初始化（14–20 + 算法 1）、紧耦合 BA（21–25）、边缘化策略（§VI.D，Schur 补代数由 §VI.D' 自含补全）、相机率/IMU 率/失败恢复（§VI.E–G）、重定位（26 + 两步 outlier）、4-DOF 位姿图（27–29）、实验关键数值与系统结论。记号约定与本书差异逐项对照（§0.1）。
2. **MSCKF 全推导**（滤波主线）：状态/误差态（1–5）、连续/离散传播 + F/G 阵 + Lyapunov（6–13）、stochastic cloning 增广（14–16）、测量模型（18–22）、**零空间投影**（23–24）、QR 压缩 EKF 更新 + 增益 + Joseph 协方差（25–31）、逆深度三角化。JPL↔Hamilton 差异标注（§0.2）。
3. **Forster 流形预积分全推导**（补 VINS 缺口）：$\mathrm{SO}(3)$ 工具 + 右雅可比（3–14）、相对增量定义（26）、噪声分离与协方差 A/B 递推（28–35）、**bias 一阶校正解析雅可比**（36，VINS 后验校正之源）、IMU 因子残差（37）、**structureless 视觉因子零空间消元**（42–46，与 MSCKF 零空间投影同源——两条主线数学本质的桥）。右扰动主线与本书一致（§0.3）。
4. **可观性 + 对比**（本章聚焦）：VINS 4 维不可观方向 + 零空间矩阵闭式（式 18）、退化运动表（纯平移→姿态不可观、常加速度/匀速→尺度不可观等）、FEJ/OC/不变滤波一致性修法、滤波 vs 优化对比表、紧/松耦合、VIO vs SLAM。

**未尽部分 / 需综合时注意（缺口已在 §0.4 表与各处 \rebuilt 标注）**：
- VINS-Mono **视觉残差 (25) 的雅可比**、**IMU 残差 (24) 的雅可比**未在论文给出（论文称"detailed 略"）→ 须由 Forster 式 42–44 / 本书 BA 章 / VINS 代码（`vins_estimator/src/factor/`）补全闭式。
- VINS 式 9 的 **bias 雅可比 $\mathbf J^\alpha_{b_a}$ 等子块闭式**论文未展开（仅给数值递推 11）→ Forster §V.C 解析式（本文件 §Forster.C 给框架，完整闭式在 Forster 补充材料 [29]，本抽取未逐项展开 $\partial\Delta\bar{\mathbf p}_{ij}/\partial\mathbf b^a$ 的标量级表达式）。
- **Forster 协方差 A/B 阵的逐元素闭式**与三角化高斯-牛顿的逐步代数，源把它们放补充材料 [29]，本抽取给出递推结构与思想、未抄补充材料的每个矩阵元素（需要时综合 agent 可专门抽 [29]）。
- **退化运动的逐条解析证明**（Martinelli [156] 的连续对称性推导）本抽取只给结论表 + 直觉，未抄其完整李导数证明（超出 VINS 主源范围，属可观性专题论文）。
- MSCKF 测量雅可比 $\mathbf H_{X_i}^{(j)},\mathbf H_{f_i}^{(j)}$ 的逐元素值，源指向 [21]（Mourikis 技术报告），本抽取给链式结构、未抄 [21] 每元素。

> 总评：对四个本章聚焦要素（① VIO 问题与可观性、② 松/紧耦合、③ 优化(VINS)/滤波(MSCKF) 两主线完整推导并对比、④ 初始化）均做到**自包含、可独立复现级**的全量抽取；VINS-Mono 主源做到逐式逐策略保真；两条主线推导完整且给出数学本质对比（零空间投影同源、relinearize 差异、一致性/FEJ）。剩余缺口集中在"论文外推到补充材料/技术报告的逐元素闭式雅可比"，已逐条列明出处供综合 agent 按需补抽。
