# 抽取留痕：MSCKF 多状态约束卡尔曼滤波 VIO 完整推导（状态增广 · 零空间投影 · 滤波更新）

> **本文件性质**：项目内部「抽取留痕」，**非成书正文**。目标是把多个权威源材料【全量保真】地抽取下来（每一步推导/中间代数不跳、每一道例题与数值例、每一条定义/定理/引理 + 完整证明、每一张表/分类/算法伪码），供后续综合 agent 改写成自包含的书章。**禁摘要、禁凝练**。公式一律 LaTeX 写全，标注【源出处】，保留所有式号与条件。
>
> **服务章节**：视觉惯性里程计 VIO。本章【聚焦】须重点覆盖：(a) VIO 问题与可观性；(b) 松/紧耦合；(c) 基于优化(VINS 滑窗+边缘化) 与 基于滤波(MSCKF) 两条主线完整推导并对比；(d) 初始化。本抽取以 **MSCKF 主线为核心（§1–§7）**，并配齐 **可观性/一致性（§5）**、**初始化（§8 来自 VINS-Mono）**、**优化主线 VINS（§9 来自 VINS-Mono）** 与 **两主线对比（§10）**，使综合 agent 可一站式取材。

---

## 源材料清单与出处

| 代号 | 文献 | 出处（论文名 + 编号/URL） | 在本抽取中承担 |
|---|---|---|---|
| **[MR07]** | A. I. Mourikis, S. I. Roumeliotis, *A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation* | ICRA 2007, pp. 3565–3572, DOI 10.1109/ROBOT.2007.364024. PDF: `https://www-users.cse.umn.edu/~stergios/papers/ICRA07-MSCKF.pdf` | **MSCKF 主线骨架**：IMU 态/误差态、连续模型、$F/G$、状态增广 $J$、量测模型、**零空间投影（核心创新）**、QR 降维、EKF 更新、特征三角化（逆深度）。全文逐式抽取。 |
| **[Trawny05]** | N. Trawny, S. I. Roumeliotis, *Indirect Kalman Filter for 3D Attitude Estimation — A Tutorial for Quaternion Algebra* | Univ. of Minnesota MARS Lab Tech. Rep. TR-2005-002, Rev. 57. PDF: `https://mars.cs.umn.edu/tr/reports/Trawny05b.pdf` | **JPL 四元数误差态离散传播闭式**：状态转移阵 $\Phi=\begin{bmatrix}\Theta&\Psi\\0&I\end{bmatrix}$ 的闭式 $\Theta,\Psi$ 及离散噪声 $Q_d$ 闭式（[MR07] 只数值积分，此源给闭式）。 |
| **[Sun18]** | K. Sun, K. Mohta, B. Pfrommer, et al., *Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight* (S-MSCKF) | IEEE RA-L 2018; arXiv:1712.00036v3. PDF: `https://arxiv.org/pdf/1712.00036` | **MSCKF 的现代/含标定完整实现**：把相机外参 $({}^I_C\bar q,{}^I p_C)$ 纳入态、双目量测模型、**完整 $F,G,J$ 矩阵（附录 A/B）**、**完整量测雅可比 $H_{C_i},H_{f_i}$（附录 C）**、**可观性约束 OC-EKF（§III-C）**、**附录 D：单帧量测零空间退化证明**。 |
| **[OV]** | P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, G. Huang, *OpenVINS* 文档（官方）| `https://docs.openvins.com/`（页面：update-feat / update-null / update-compress / propagation / propagation_discrete / fej / update-delay）| **量测雅可比全链式分解**（畸变/投影/欧式变换/特征表示，含各 representation 的 $\partial f/\partial\lambda$）、**零空间投影与量测压缩的标准写法**、**FEJ 与 4 维不可观零空间 $\mathcal N$ 的闭式**、**延迟特征初始化（SLAM 特征）**。 |
| **[Qin18]** | T. Qin, P. Li, S. Shen, *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator* | IEEE T-RO 2018; arXiv:1708.03852. PDF: `https://arxiv.org/pdf/1708.03852` | **优化主线（对比用）**：IMU 预积分（连续误差动力学/协方差递推/bias 一阶修正）、滑窗紧耦合 BA、IMU/视觉残差、**边缘化（Schur 补）**、**初始化（vision-only SfM + 视觉惯性对齐：陀螺 bias、速度/重力/尺度、重力细化）**。 |
| **[Li13]** | M. Li, A. I. Mourikis, *High-Precision, Consistent EKF-based Visual-Inertial Odometry* (MSCKF 2.0) | IJRR 2013, 32(6):690–711. （PDF 镜像需授权；其可观性结论由 [OV]/[Sun18] 转述抽取）| **可观性/一致性理论来源**（OC-MSCKF / MSCKF 2.0），FEJ 思想的 VIO 落地。本抽取中其结论通过 [OV] §FEJ 与 [Sun18] §III-C 的等价表述给出。 |

> **抽取深度声明**：[MR07] 全文逐式（式 1–38）；[Trawny05] §2.4–2.5 全部闭式（式 158–192）；[Sun18] §III + 附录 A–D 全部；[OV] 7 个页面全部公式；[Qin18] §IV–VI（预积分/优化/边缘化）+ §V（初始化）。

---

## §0 记号约定（各源）与本书统一约定的差异

> 本节最重要：VIO 文献历史上分裂为 **JPL 四元数 + 左乘误差（Mourikis 系）** 与 **Hamilton 四元数 + 右扰动（Eigen/GTSAM/VINS 系）** 两大阵营。本书统一约定为 **$\mathbf R\in\mathrm{SO}(3)$、Hamilton 四元数、右扰动为主（$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$，局部坐标）、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$**。综合时必须按本表逐项转换，尤其 [MR07]/[Sun18]/[Trawny05]/[OV] 这条 MSCKF 主线全用 **JPL**。

### 0.1 总览对照表

| 项目 | [MR07]/[Sun18]/[OV]（MSCKF 主线，JPL） | [Trawny05]（JPL 姿态） | [Qin18]（VINS，Hamilton） | 本书统一约定 | 差异 / 转换 |
|---|---|---|---|---|---|
| 四元数 flavor | **JPL**（$ij=-k$，左手定则分量序，passive，global→local） | **JPL** | **Hamilton**（$ij=+k$） | **Hamilton** | JPL↔Hamilton：乘法顺序相反 $\otimes_{\rm JPL}$ 与 $\otimes_{\rm Ham}$ 互为转置矩阵；同一物理旋转下 ${}^I_G\bar q_{\rm JPL}\leftrightarrow ({}^G_I q_{\rm Ham})$。综合本书全部改写为 Hamilton。 |
| 旋转方向记号 | ${}^I_G\bar q$ = 从 $\{G\}$ 到 $\{I\}$ 的旋转；$\mathbf C({}^I_G\bar q)={}^I_G\mathbf R$ | ${}^L_G\bar q$ | $\mathbf q^w_{b_k}$ = 从 body 到 world | $\mathbf R_{wb}$（body→world） | MSCKF 主线把旋转写成 global→local（${}^I_G\mathbf R$），与本书 body→world 互为转置。 |
| 误差/扰动定义（**关键**） | **左乘小扰动**：$\bar q=\delta\bar q\otimes\hat{\bar q}$，$\delta\bar q\approx[\tfrac12\delta\boldsymbol\theta^\top,\;1]^\top$（[MR07] 式 3） | 同（式 137,141） | Hamilton 右乘：$\boldsymbol\gamma\approx\hat{\boldsymbol\gamma}\otimes[1,\tfrac12\delta\boldsymbol\theta]^\top$（[Qin18] 式 8） | **右扰动为主** | MSCKF 误差是**左乘 JPL** 小角 $\delta\boldsymbol\theta$，对应**左扰动**。综合到本书右扰动需把 $\delta\boldsymbol\theta_{\rm left}=-\mathbf R\,\delta\boldsymbol\phi_{\rm right}$ 之类做符号/侧别转换（见 §0.3）。 |
| 误差态排序 | $\tilde{\mathbf x}_I=[\delta\boldsymbol\theta_I,\;\tilde{\mathbf b}_g,\;{}^G\tilde{\mathbf v}_I,\;\tilde{\mathbf b}_a,\;{}^G\tilde{\mathbf p}_I]$（**姿态在前**，[MR07] 式 2） | $[\delta\boldsymbol\theta,\;\Delta\mathbf b]$ | $[\delta\mathbf p,\delta\mathbf v,\delta\boldsymbol\theta,\delta\mathbf b_a,\delta\mathbf b_g]$ | $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（限 $\mathfrak{se}(3)$ 层；IMU 全态另议） | MSCKF 把 $\delta\boldsymbol\theta$ 放最前、$\delta\mathbf p$ 放最后；[OV] 用 $[\delta\boldsymbol\theta,\delta\mathbf p,\delta\mathbf v,\mathbf b_g,\mathbf b_a]$。综合排序需重排 $F,G,\Phi,H$ 的分块。 |
| 反对称算子 | $\lfloor\mathbf a\times\rfloor$ 或 $[\mathbf a_\times]$（[Sun18]）、$\mathbf a^\times$ | $\lfloor\boldsymbol\omega\times\rfloor$ | $[\cdot]_\times$ | $[\cdot]_\times$ 或 $(\cdot)^\wedge$ | 一致（仅写法）。 |
| 协方差字母 | $\mathbf P$（态）、$\sigma_{\rm im}^2$（像素噪声）、$\mathbf Q_{\rm IMU}/\mathbf Q$（过程）、$\mathbf R^{(j)}$（量测）| $\mathbf P,\mathbf Q_d,\mathbf Q_c$ | $\mathbf P,\mathbf Q$ | $\mathbf P/\mathbf R/\mathbf Q$ | 基本一致；像素噪声 [MR07] 记 $\sigma_{\rm im}$。 |
| 增益/量测雅可比 | $\mathbf K$；$\mathbf H_{\mathbf X}$（对态）、$\mathbf H_f$（对特征）| — | $\mathbf K$；$\mathbf H$ | $\mathbf K,\mathbf H$ | 一致。MSCKF 关键是 $\mathbf H_f$ 的**左零空间**。 |
| 重力 | ${}^G\mathbf g$（[Sun18] 在 world 系；[MR07] 在 ECEF 还含地球自转 $\boldsymbol\omega_G$）| — | $\mathbf g^w$ | $\mathbf g$ | [MR07] 因选 ECEF 系，量测含地球自转项（式 7–8）；[Sun18]/[OV]/[Qin18] 选局部惯性系，忽略地球自转。本书默认局部系（忽略 $\boldsymbol\omega_G$）。 |
| 时间下标 | $k|k$（后验）、$k+1|k$（先验/预测）| 同 | $b_k$（关键帧）| check/hat（先验/后验）| 一致语义。 |

### 0.2 三种「零空间」的术语澄清（综合时极易混淆，务必区分）

MSCKF 全流程出现**三个不同的零空间/QR**，本抽取统一命名以防混淆：

1. **特征左零空间投影**（$\mathbf H_f$ 的左零空间 $\mathbf A$ / $\mathbf V$ / $\mathbf Q_2$）：把单特征堆叠残差中的特征误差 ${}^G\tilde{\mathbf p}_f$ 消去 —— **这是 MSCKF 的核心创新**（§3）。维度：$\mathbf H_f$ 为 $2M_j\times3$（单目）或 $4M_j\times3$（双目），左零空间维数 $2M_j-3$（或 $4M_j-3$）。
2. **量测压缩 QR**（大 $\mathbf H_{\mathbf X}$ 的列空间投影 $\mathbf Q_1$）：把堆叠后行数 $d$ 巨大的 $\mathbf r_o=\mathbf H_{\mathbf X}\tilde{\mathbf x}+\mathbf n_o$ 压成行数 $\le$ 态维 $r$，降更新成本（§4）。
3. **不可观零空间** $\mathcal N$（观测矩阵 $\mathcal O$ 的右零空间，4 维：3 平移 + 1 yaw）：可观性/一致性分析对象，FEJ/OC 用以保持其维数（§5）。

> **本质洞察（综合时写盒）**：1 是「把特征边缘化掉」的代数手段（Schur 补的对偶——见 §3.5）；3 是「VIO 物理上测不出的方向」。两者无关但都叫 nullspace，是 MSCKF 文献最大的术语陷阱。

### 0.3 左乘 JPL 误差 ↔ 本书右扰动 的转换要点（综合 agent 必读）

- [MR07]/[Sun18] 用 $\bar q=\delta\bar q\otimes\hat{\bar q}$（**全局/左乘**），其 $\delta\boldsymbol\theta$ 是在 **global 系**表达的小角；这与 [Trawny05] 式 137 的 ${}^L_G\bar q={}^L_{\hat L}\delta\bar q\otimes{}^{\hat L}_G\hat{\bar q}$（**局部/左乘**）侧别约定上一致（都左乘），区别仅在所旋转的对象。
- 本书右扰动 $\mathbf R_{wb}\leftarrow\mathbf R_{wb}\,\mathrm{Exp}(\delta\boldsymbol\phi)$ 是**局部右乘**。MSCKF 的全局左乘 $\delta\boldsymbol\theta$ 与本书局部右扰动 $\delta\boldsymbol\phi$ 通过 $\delta\boldsymbol\theta=\mathbf R_{wb}\,\delta\boldsymbol\phi$（小角下伴随关系）联系；旋转矩阵雅可比 $[\cdot\times]$ 项在转换时会左乘/右乘一个 $\mathbf R$。
- 因此综合到本书时：把所有 ${}^I_G\mathbf R$ 替换为 $\mathbf R_{wb}^\top$，把左乘 $\delta\boldsymbol\theta$ 改写为右扰动 $\delta\boldsymbol\phi$，相应地 $F$ 矩阵中 $-\lfloor\hat{\boldsymbol\omega}\times\rfloor$ 与 $-\mathbf C(\hat{\bar q})^\top\lfloor\hat{\mathbf a}\times\rfloor$ 等块需做侧别变换。**本抽取忠实保留各源原式，不预先转换**，转换工作交综合层并在此显式提示。

---

# 第一部分：MSCKF 主线（基于滤波）

## §1 EKF 状态向量结构 [源 [MR07] §III-A；[Sun18] §III]

### §1.1 演化 IMU 状态（[MR07] 式 1）

演化（evolving）IMU 状态由如下向量描述：
$$\mathbf X_{\rm IMU}=\begin{bmatrix}{}^I_G\bar q^\top & \mathbf b_g^\top & {}^G\mathbf v_I^\top & \mathbf b_a^\top & {}^G\mathbf p_I^\top\end{bmatrix}^\top,\tag{MR07-1}$$
其中：
- ${}^I_G\bar q$：单位四元数，描述从全局系 $\{G\}$ 到 IMU 系 $\{I\}$ 的旋转（JPL 约定，[MR07] 引 Breckenridge JPL 标准 [19]）；
- ${}^G\mathbf p_I,\ {}^G\mathbf v_I$：IMU 在 $\{G\}$ 中的位置与速度；
- $\mathbf b_g,\ \mathbf b_a\in\mathbb R^3$：陀螺与加速度计的偏置（bias），建模为**随机游走**，由白高斯噪声 $\mathbf n_{wg},\mathbf n_{wa}$ 驱动。

### §1.2 IMU 误差状态（[MR07] 式 2）

按式 1 定义 IMU 误差态为
$$\tilde{\mathbf X}_{\rm IMU}=\begin{bmatrix}\delta\boldsymbol\theta_I^\top & \tilde{\mathbf b}_g^\top & {}^G\tilde{\mathbf v}_I^\top & \tilde{\mathbf b}_a^\top & {}^G\tilde{\mathbf p}_I^\top\end{bmatrix}^\top.\tag{MR07-2}$$
- 对**位置、速度、偏置**用标准**加性误差**：量 $x$ 的估计 $\hat x$ 的误差定义为 $\tilde x=x-\hat x$。
- 对**四元数**用**不同的误差定义**：若 $\hat{\bar q}$ 是 $\bar q$ 的估计，则姿态误差由**误差四元数** $\delta\bar q$ 描述，定义关系为
$$\bar q=\delta\bar q\otimes\hat{\bar q},\tag{$\ast$}$$
$\otimes$ 为四元数乘法。误差四元数取小角近似（[MR07] 式 3）：
$$\delta\bar q\simeq\begin{bmatrix}\tfrac12\delta\boldsymbol\theta^\top & 1\end{bmatrix}^\top.\tag{MR07-3}$$

> **直觉（源原文）**：$\delta\bar q$ 描述使真实姿态与估计姿态重合的那个（小）旋转。姿态对应 3 自由度，故用 $\delta\boldsymbol\theta\in\mathbb R^3$ 描述姿态误差是**最小表示**（避免 4 维四元数协方差因单位约束而奇异——见 [Trawny05] §2.3）。

### §1.3 含 $N$ 个相机位姿的完整状态（[MR07] 式 4–5）

设 $k$ 时刻 EKF 状态含 $N$ 个相机位姿，则状态向量为
$$\hat{\mathbf X}_k=\begin{bmatrix}\hat{\mathbf X}_{\rm IMU}^\top & {}^{C_1}_G\hat{\bar q}^\top & {}^G\hat{\mathbf p}_{C_1}^\top & \cdots & {}^{C_N}_G\hat{\bar q}^\top & {}^G\hat{\mathbf p}_{C_N}^\top\end{bmatrix}^\top,\tag{MR07-4}$$
其中 ${}^{C_i}_G\hat{\bar q}$、${}^G\hat{\mathbf p}_{C_i}$（$i=1\dots N$）为相机姿态与位置的估计。对应 EKF 误差态向量：
$$\tilde{\mathbf X}_k=\begin{bmatrix}\tilde{\mathbf X}_{\rm IMU}^\top & \delta\boldsymbol\theta_{C_1}^\top & {}^G\tilde{\mathbf p}_{C_1}^\top & \cdots & \delta\boldsymbol\theta_{C_N}^\top & {}^G\tilde{\mathbf p}_{C_N}^\top\end{bmatrix}^\top.\tag{MR07-5}$$
IMU 误差态 15 维，每个相机位姿误差态 6 维，总维 $\xi=15+6N$。

### §1.4 [Sun18] 的扩展：把相机外参纳入态 [源 [Sun18] §III]

[Sun18] 把 IMU-相机外参也作为状态在线估计：
$$\mathbf x_I=\begin{bmatrix}{}^I_G q^\top & \mathbf b_g^\top & {}^G\mathbf v_I^\top & \mathbf b_a^\top & {}^G\mathbf p_I^\top & {}^I_C q^\top & {}^I\mathbf p_C^\top\end{bmatrix}^\top,$$
其中 ${}^I_C q,\ {}^I\mathbf p_C$ 为相机系相对 body(IMU) 系的外参（双目时取左相机，左右相机外参已标定）。对应误差态
$$\tilde{\mathbf x}_I=\begin{bmatrix}{}^I_G\tilde{\boldsymbol\theta}^\top & \tilde{\mathbf b}_g^\top & {}^G\tilde{\mathbf v}^\top & \tilde{\mathbf b}_a^\top & {}^G\tilde{\mathbf p}^\top & {}^I_C\tilde{\boldsymbol\theta}^\top & {}^I\tilde{\mathbf p}_C^\top\end{bmatrix}^\top,$$
误差四元数 $\delta q=q\otimes\hat q^{-1}$，$\delta q\approx[\tfrac12\,{}^I_G\tilde{\boldsymbol\theta}^\top,\ 1]^\top$。整个误差态（含 $N$ 个相机克隆）：
$$\tilde{\mathbf x}=\begin{bmatrix}\tilde{\mathbf x}_I^\top & \tilde{\mathbf x}_{C_1}^\top & \cdots & \tilde{\mathbf x}_{C_N}^\top\end{bmatrix}^\top,\qquad \tilde{\mathbf x}_{C_i}=\begin{bmatrix}{}^{C_i}_G\tilde{\boldsymbol\theta}^\top & {}^G\tilde{\mathbf p}_{C_i}^\top\end{bmatrix}^\top.$$
IMU 误差态此时为 21 维（15 + 6 外参），总维 $21+6N$。

> **OpenVINS 排序（[OV]）**：$\mathbf x_I=[{}^I_G\bar q,\ {}^G\mathbf p_I,\ {}^G\mathbf v_I,\ \mathbf b_g,\ \mathbf b_a]$，误差态 $\tilde{\mathbf x}_I=[{}^I_G\tilde{\boldsymbol\theta},\ {}^G\tilde{\mathbf p}_I,\ {}^G\tilde{\mathbf v}_I,\ \tilde{\mathbf b}_g,\ \tilde{\mathbf b}_a]$（姿态、位置、速度、两 bias）。综合时三家排序不同，重排 $F/G/\Phi/H$ 分块即可。

---

## §2 传播（Propagation）[源 [MR07] §III-B；[Sun18] §III-A；[Trawny05] §2.4–2.6]

滤波传播方程由对**连续时间 IMU 系统模型**离散化导出。

### §2.1 连续时间系统建模（[MR07] 式 6）

IMU 状态的时间演化（[MR07] 引 Chatfield [20]）：
$$
{}^I_G\dot{\bar q}(t)=\tfrac12\,\boldsymbol\Omega\big(\boldsymbol\omega(t)\big)\,{}^I_G\bar q(t),\quad
\dot{\mathbf b}_g(t)=\mathbf n_{wg}(t),\tag{MR07-6a}
$$
$$
{}^G\dot{\mathbf v}_I(t)={}^G\mathbf a(t),\quad
\dot{\mathbf b}_a(t)=\mathbf n_{wa}(t),\quad
{}^G\dot{\mathbf p}_I(t)={}^G\mathbf v_I(t).\tag{MR07-6b}
$$
其中 ${}^G\mathbf a$ 是 body 在全局系的加速度，$\boldsymbol\omega=[\omega_x,\omega_y,\omega_z]^\top$ 是在 IMU 系表达的角速度，且
$$
\boldsymbol\Omega(\boldsymbol\omega)=\begin{bmatrix}-\lfloor\boldsymbol\omega\times\rfloor & \boldsymbol\omega\\ -\boldsymbol\omega^\top & 0\end{bmatrix},\qquad
\lfloor\boldsymbol\omega\times\rfloor=\begin{bmatrix}0&-\omega_z&\omega_y\\ \omega_z&0&-\omega_x\\ -\omega_y&\omega_x&0\end{bmatrix}.\tag{MR07-6c}
$$

### §2.2 IMU 量测模型（[MR07] 式 7–8；[Sun18]）

陀螺与加速度计测量 $\boldsymbol\omega_m,\mathbf a_m$（[MR07] 选 ECEF 全局系，含地球自转 $\boldsymbol\omega_G$）：
$$
\boldsymbol\omega_m=\boldsymbol\omega+\mathbf C({}^I_G\bar q)\,\boldsymbol\omega_G+\mathbf b_g+\mathbf n_g,\tag{MR07-7}
$$
$$
\mathbf a_m=\mathbf C({}^I_G\bar q)\big({}^G\mathbf a-{}^G\mathbf g+2\lfloor\boldsymbol\omega_G\times\rfloor\,{}^G\mathbf v_I+\lfloor\boldsymbol\omega_G\times\rfloor^2\,{}^G\mathbf p_I\big)+\mathbf b_a+\mathbf n_a,\tag{MR07-8}
$$
$\mathbf C(\cdot)$ 为旋转矩阵，$\mathbf n_g,\mathbf n_a$ 为零均值白高斯量测噪声，${}^G\mathbf g$ 为重力。

> **简化版（[Sun18]/[OV]，忽略地球自转）**：$\boldsymbol\omega_m=\boldsymbol\omega+\mathbf b_g+\mathbf n_g$，$\mathbf a_m=\mathbf a+\mathbf C({}^I_G q)\,{}^G\mathbf g+\mathbf b_a+\mathbf n_a$（[OV]），或写 $\hat{\boldsymbol\omega}=\boldsymbol\omega_m-\hat{\mathbf b}_g,\ \hat{\mathbf a}=\mathbf a_m-\hat{\mathbf b}_a$（[Sun18]）。本书默认此简化版。

### §2.3 估计量传播（取期望，[MR07] 式 9）

对式 6 取期望算子，得演化 IMU 状态估计的传播方程：
$$
{}^I_G\dot{\hat{\bar q}}=\tfrac12\,\boldsymbol\Omega(\hat{\boldsymbol\omega})\,{}^I_G\hat{\bar q},\quad
\dot{\hat{\mathbf b}}_g=\mathbf 0_{3\times1},
$$
$$
{}^G\dot{\hat{\mathbf v}}_I=\mathbf C_{\hat q}^\top\hat{\mathbf a}-2\lfloor\boldsymbol\omega_G\times\rfloor\,{}^G\hat{\mathbf v}_I-\lfloor\boldsymbol\omega_G\times\rfloor^2\,{}^G\hat{\mathbf p}_I+{}^G\mathbf g,\quad
\dot{\hat{\mathbf b}}_a=\mathbf 0_{3\times1},\quad
{}^G\dot{\hat{\mathbf p}}_I={}^G\hat{\mathbf v}_I,\tag{MR07-9}
$$
其中简记 $\mathbf C_{\hat q}=\mathbf C({}^I_G\hat{\bar q})$，$\hat{\mathbf a}=\mathbf a_m-\hat{\mathbf b}_a$，$\hat{\boldsymbol\omega}=\boldsymbol\omega_m-\hat{\mathbf b}_g-\mathbf C_{\hat q}\boldsymbol\omega_G$。

[Sun18] 对应（忽略地球自转，含外参恒定）：
$$
{}^I_G\dot{\hat q}=\tfrac12\boldsymbol\Omega(\hat{\boldsymbol\omega})\,{}^I_G\hat q,\quad \dot{\hat{\mathbf b}}_g=\mathbf 0,\quad {}^G\dot{\hat{\mathbf v}}=\mathbf C({}^I_G\hat q)^\top\hat{\mathbf a}+{}^G\mathbf g,\quad \dot{\hat{\mathbf b}}_a=\mathbf 0,\quad {}^G\dot{\hat{\mathbf p}}_I={}^G\hat{\mathbf v},\quad {}^I_C\dot{\hat q}=\mathbf 0,\quad {}^I\dot{\hat{\mathbf p}}_C=\mathbf 0.\tag{Sun18-1}
$$

### §2.4 线性化连续误差态模型（[MR07] 式 10–式 F/G）

IMU 误差态的线性化连续模型：
$$\dot{\tilde{\mathbf X}}_{\rm IMU}=\mathbf F\,\tilde{\mathbf X}_{\rm IMU}+\mathbf G\,\mathbf n_{\rm IMU},\tag{MR07-10}$$
其中系统噪声 $\mathbf n_{\rm IMU}=[\mathbf n_g^\top,\ \mathbf n_{wg}^\top,\ \mathbf n_a^\top,\ \mathbf n_{wa}^\top]^\top$，其协方差 $\mathbf Q_{\rm IMU}$ 由 IMU 噪声特性决定（离线标定）。$\mathbf F$（$15\times15$，[MR07] 排序 $[\delta\boldsymbol\theta,\tilde{\mathbf b}_g,\tilde{\mathbf v},\tilde{\mathbf b}_a,\tilde{\mathbf p}]$）：
$$
\mathbf F=\begin{bmatrix}
-\lfloor\hat{\boldsymbol\omega}\times\rfloor & -\mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
-\mathbf C_{\hat q}^\top\lfloor\hat{\mathbf a}\times\rfloor & \mathbf 0_{3\times3} & -2\lfloor\boldsymbol\omega_G\times\rfloor & -\mathbf C_{\hat q}^\top & -\lfloor\boldsymbol\omega_G\times\rfloor^2\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}
\end{bmatrix},\tag{MR07-F}
$$
$\mathbf G$（$15\times12$）：
$$
\mathbf G=\begin{bmatrix}
-\mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf I_3 & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & -\mathbf C_{\hat q}^\top & \mathbf 0_{3\times3}\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf I_3\\
\mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3} & \mathbf 0_{3\times3}
\end{bmatrix}.\tag{MR07-G}
$$

### §2.5 [Sun18] 的完整 $F,G$（附录 A，含外参，忽略地球自转）[源 [Sun18] 附录 A]

误差态排序 $[{}^I_G\tilde{\boldsymbol\theta},\tilde{\mathbf b}_g,{}^G\tilde{\mathbf v},\tilde{\mathbf b}_a,{}^G\tilde{\mathbf p},{}^I_C\tilde{\boldsymbol\theta},{}^I\tilde{\mathbf p}_C]$（$21$ 维）。$\tilde{\mathbf x}_I$ 满足 $\dot{\tilde{\mathbf x}}_I=\mathbf F\tilde{\mathbf x}_I+\mathbf G\mathbf n_I$，$\mathbf n_I=[\mathbf n_g^\top,\mathbf n_{wg}^\top,\mathbf n_a^\top,\mathbf n_{wa}^\top]^\top$。
$$
\mathbf F=\begin{bmatrix}
-\lfloor\hat{\boldsymbol\omega}\times\rfloor & -\mathbf I_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
-\mathbf C({}^I_G\hat q)^\top\lfloor\hat{\mathbf a}\times\rfloor & \mathbf 0_3 & \mathbf 0_3 & -\mathbf C({}^I_G\hat q)^\top & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf I_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3
\end{bmatrix},\quad
\mathbf G=\begin{bmatrix}
-\mathbf I_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf I_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & -\mathbf C({}^I_G\hat q)^\top & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf I_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3\\
\mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3 & \mathbf 0_3
\end{bmatrix}.
$$
（外参 ${}^I_C\boldsymbol\theta,{}^I\mathbf p_C$ 行/列全为 0，因外参恒定无动力学。）

### §2.6 离散实现（[MR07] 式 11–13）

IMU 以周期 $T$ 采样 $\boldsymbol\omega_m,\mathbf a_m$。每收到新 IMU 量测，用 **5 阶 Runge-Kutta** 数值积分式 9 传播状态估计。协方差按下述分块传播。记
$$
\mathbf P_{k|k}=\begin{bmatrix}\mathbf P_{II_{k|k}} & \mathbf P_{IC_{k|k}}\\ \mathbf P_{IC_{k|k}}^\top & \mathbf P_{CC_{k|k}}\end{bmatrix},\tag{MR07-11}
$$
$\mathbf P_{II_{k|k}}$ 为 $15\times15$ IMU 协方差，$\mathbf P_{CC_{k|k}}$ 为 $6N\times6N$ 相机位姿协方差，$\mathbf P_{IC_{k|k}}$ 为二者相关。传播后协方差：
$$
\mathbf P_{k+1|k}=\begin{bmatrix}\mathbf P_{II_{k+1|k}} & \boldsymbol\Phi(t_k+T,t_k)\,\mathbf P_{IC_{k|k}}\\ \mathbf P_{IC_{k|k}}^\top\boldsymbol\Phi(t_k+T,t_k)^\top & \mathbf P_{CC_{k|k}}\end{bmatrix},
$$
其中 $\mathbf P_{II_{k+1|k}}$ 由 **Lyapunov 方程**数值积分得（区间 $(t_k,t_k+T)$，初值 $\mathbf P_{II_{k|k}}$）：
$$
\dot{\mathbf P}_{II}=\mathbf F\mathbf P_{II}+\mathbf P_{II}\mathbf F^\top+\mathbf G\mathbf Q_{\rm IMU}\mathbf G^\top.\tag{MR07-12}
$$
状态转移阵 $\boldsymbol\Phi(t_k+T,t_k)$ 由微分方程数值积分得（初值 $\boldsymbol\Phi(t_k,t_k)=\mathbf I_{15}$）：
$$
\dot{\boldsymbol\Phi}(t_k+\tau,t_k)=\mathbf F\,\boldsymbol\Phi(t_k+\tau,t_k),\quad\tau\in[0,T].\tag{MR07-13}
$$

[Sun18] 用 **4 阶 RK** 积分式 1 传播状态，离散转移阵与离散噪声协方差为
$$
\boldsymbol\Phi_k=\boldsymbol\Phi(t_{k+1},t_k)=\exp\!\Big(\int_{t_k}^{t_{k+1}}\mathbf F(\tau)\,d\tau\Big),\qquad
\mathbf Q_k=\int_{t_k}^{t_{k+1}}\boldsymbol\Phi(t_{k+1},\tau)\,\mathbf G\mathbf Q\mathbf G^\top\boldsymbol\Phi(t_{k+1},\tau)^\top\,d\tau,
$$
$\mathbf Q=\mathbb E[\mathbf n_I\mathbf n_I^\top]$ 为连续过程噪声协方差。则
$$
\mathbf P_{II_{k+1|k}}=\boldsymbol\Phi_k\mathbf P_{II_{k|k}}\boldsymbol\Phi_k^\top+\mathbf Q_k,\qquad
\mathbf P_{k+1|k}=\begin{bmatrix}\mathbf P_{II_{k+1|k}} & \boldsymbol\Phi_k\mathbf P_{IC_{k|k}}\\ \mathbf P_{IC_{k|k}}^\top\boldsymbol\Phi_k^\top & \mathbf P_{CC_{k|k}}\end{bmatrix}.
$$

### §2.7 闭式状态转移阵与离散噪声协方差（[Trawny05] §2.5，姿态 + 陀螺 bias 子系统）

> [MR07] 仅给数值积分；[Trawny05] 对「姿态误差 $\delta\boldsymbol\theta$ + 陀螺 bias 误差 $\Delta\mathbf b$」6 维子系统给出**闭式** $\Phi,Q_d$。这是 VIO 闭式离散化的经典出处，综合时可据此把 [MR07] 数值积分替换成解析式（其余 $\mathbf v,\mathbf p$ 块按 §2.8 [OV] 补）。

**连续误差态方程（[Trawny05] 式 158–160）**。由误差四元数定义 $\bar q=\delta\bar q\otimes\hat{\bar q}$ 求导（式 144–155 全过程）：
$$
\dot{\delta\boldsymbol\theta}=-\lfloor\hat{\boldsymbol\omega}\times\rfloor\,\delta\boldsymbol\theta-\Delta\mathbf b-\mathbf n_r,\qquad \dot{\Delta\mathbf b}=\mathbf n_w,\tag{Trawny-156/157}
$$
合写
$$
\begin{bmatrix}\dot{\delta\boldsymbol\theta}\\ \dot{\Delta\mathbf b}\end{bmatrix}=\underbrace{\begin{bmatrix}-\lfloor\hat{\boldsymbol\omega}\times\rfloor & -\mathbf I_3\\ \mathbf 0_3 & \mathbf 0_3\end{bmatrix}}_{\mathbf F_c}\begin{bmatrix}\delta\boldsymbol\theta\\ \Delta\mathbf b\end{bmatrix}+\underbrace{\begin{bmatrix}-\mathbf I_3 & \mathbf 0_3\\ \mathbf 0_3 & \mathbf I_3\end{bmatrix}}_{\mathbf G_c}\begin{bmatrix}\mathbf n_r\\ \mathbf n_w\end{bmatrix}.\tag{Trawny-158/160}
$$
连续噪声协方差 $\mathbf Q_c=\mathrm{diag}(\sigma_{r_c}^2\mathbf I_3,\ \sigma_{w_c}^2\mathbf I_3)$（式 162）。

**状态转移阵闭式（式 163–172）**。$\mathbf F_c$ 在步长 $\Delta t$ 内常值，故 $\boldsymbol\Phi=\exp(\mathbf F_c\Delta t)=\mathbf I+\mathbf F_c\Delta t+\tfrac1{2!}\mathbf F_c^2\Delta t^2+\cdots$。$\mathbf F_c$ 各幂（式 165）：
$$
\mathbf F_c^2=\begin{bmatrix}\lfloor\hat{\boldsymbol\omega}\times\rfloor^2 & \lfloor\hat{\boldsymbol\omega}\times\rfloor\\ \mathbf 0_3 & \mathbf 0_3\end{bmatrix},\quad
\mathbf F_c^3=\begin{bmatrix}-\lfloor\hat{\boldsymbol\omega}\times\rfloor^3 & -\lfloor\hat{\boldsymbol\omega}\times\rfloor^2\\ \mathbf 0_3 & \mathbf 0_3\end{bmatrix},\quad
\mathbf F_c^4=\begin{bmatrix}\lfloor\hat{\boldsymbol\omega}\times\rfloor^4 & \lfloor\hat{\boldsymbol\omega}\times\rfloor^3\\ \mathbf 0_3 & \mathbf 0_3\end{bmatrix}.
$$
转移阵分块结构（式 166）：
$$
\boldsymbol\Phi(t+\Delta t,t)=\begin{bmatrix}\boldsymbol\Theta & \boldsymbol\Psi\\ \mathbf 0_3 & \mathbf I_3\end{bmatrix}.
$$
$\boldsymbol\Theta$ 闭式（式 167→171，用 $\lfloor\hat{\boldsymbol\omega}\times\rfloor$ 幂级数重排 + 三角函数）：
$$
\boldsymbol\Theta=\cos(|\hat{\boldsymbol\omega}|\Delta t)\,\mathbf I_3-\sin(|\hat{\boldsymbol\omega}|\Delta t)\,\Big\lfloor\tfrac{\hat{\boldsymbol\omega}}{|\hat{\boldsymbol\omega}|}\times\Big\rfloor+\big(1-\cos(|\hat{\boldsymbol\omega}|\Delta t)\big)\,\tfrac{\hat{\boldsymbol\omega}}{|\hat{\boldsymbol\omega}|}\tfrac{\hat{\boldsymbol\omega}^\top}{|\hat{\boldsymbol\omega}|},\tag{Trawny-171}
$$
即 $\boldsymbol\Theta$ 恰为以 $\hat{\boldsymbol\omega}$ 为轴、$|\hat{\boldsymbol\omega}|\Delta t$ 为角的旋转矩阵。小 $|\hat{\boldsymbol\omega}|$ 取极限（L'Hôpital，式 172）：
$$
\lim_{|\boldsymbol\omega|\to0}\boldsymbol\Theta=\mathbf I_3-\Delta t\lfloor\hat{\boldsymbol\omega}\times\rfloor+\tfrac{\Delta t^2}{2}\lfloor\hat{\boldsymbol\omega}\times\rfloor^2.
$$
$\boldsymbol\Psi$ 闭式（式 173→176）：
$$
\boldsymbol\Psi=-\mathbf I_3\Delta t+\tfrac{1}{|\hat{\boldsymbol\omega}|^2}\big(1-\cos(|\hat{\boldsymbol\omega}|\Delta t)\big)\lfloor\hat{\boldsymbol\omega}\times\rfloor-\tfrac{1}{|\hat{\boldsymbol\omega}|^3}\big(|\hat{\boldsymbol\omega}|\Delta t-\sin(|\hat{\boldsymbol\omega}|\Delta t)\big)\lfloor\hat{\boldsymbol\omega}\times\rfloor^2,\tag{Trawny-176}
$$
小 $|\hat{\boldsymbol\omega}|$ 极限（式 180）：
$$
\lim_{|\boldsymbol\omega|\to0}\boldsymbol\Psi=-\mathbf I_3\Delta t+\tfrac{\Delta t^2}{2}\lfloor\hat{\boldsymbol\omega}\times\rfloor-\tfrac{\Delta t^3}{6}\lfloor\hat{\boldsymbol\omega}\times\rfloor^2.
$$
> **小 $\omega$ 近似误差注（式 105–107 段）**：若把未近似项除以 $|\hat{\boldsymbol\omega}|^n$，则小角近似误差约 $O(\Delta t^{n+2}|\hat{\boldsymbol\omega}|^2)$；100 Hz 采样时 $\boldsymbol\Theta$ 的误差约 $10^{-6}|\hat{\boldsymbol\omega}_{\rm thresh}|^2$。

**离散噪声协方差闭式（式 181–192）**：
$$
\mathbf Q_d=\int_{t_k}^{t_{k+1}}\boldsymbol\Phi(t_{k+1},\tau)\,\mathbf G_c\mathbf Q_c\mathbf G_c^\top\,\boldsymbol\Phi^\top(t_{k+1},\tau)\,d\tau
=\int_{t_k}^{t_{k+1}}\begin{bmatrix}\sigma_r^2\mathbf I_3+\sigma_w^2\,\boldsymbol\Psi\boldsymbol\Psi^\top & \sigma_w^2\,\boldsymbol\Psi\\ \sigma_w^2\,\boldsymbol\Psi^\top & \sigma_w^2\mathbf I_3\end{bmatrix}d\tau
$$
（用到 $\boldsymbol\Theta\boldsymbol\Theta^\top=\mathbf I$，式 186）。结构 $\mathbf Q_d=\begin{bmatrix}\mathbf Q_{11}&\mathbf Q_{12}\\ \mathbf Q_{12}^\top&\mathbf Q_{22}\end{bmatrix}$，各块（式 188–190，"considerable algebra" 后）：
$$
\mathbf Q_{11}=\sigma_r^2\Delta t\,\mathbf I_3+\sigma_w^2\Big[\tfrac{\Delta t^3}{3}\mathbf I_3+\Big(\tfrac{(|\hat{\boldsymbol\omega}|\Delta t)^3/3+2\sin(|\hat{\boldsymbol\omega}|\Delta t)-2|\hat{\boldsymbol\omega}|\Delta t}{|\hat{\boldsymbol\omega}|^5}\Big)\lfloor\hat{\boldsymbol\omega}\times\rfloor^2\Big],
$$
$$
\mathbf Q_{12}=-\sigma_w^2\Big[\tfrac{\Delta t^2}{2}\mathbf I_3-\tfrac{|\hat{\boldsymbol\omega}|\Delta t-\sin(|\hat{\boldsymbol\omega}|\Delta t)}{|\hat{\boldsymbol\omega}|^3}\lfloor\hat{\boldsymbol\omega}\times\rfloor+\tfrac{(|\hat{\boldsymbol\omega}|\Delta t)^2/2+\cos(|\hat{\boldsymbol\omega}|\Delta t)-1}{|\hat{\boldsymbol\omega}|^4}\lfloor\hat{\boldsymbol\omega}\times\rfloor^2\Big],
$$
$$
\mathbf Q_{22}=\sigma_w^2\Delta t\,\mathbf I_3.
$$
小 $|\hat{\boldsymbol\omega}|$ 极限（式 191–192）：
$$
\lim_{|\boldsymbol\omega|\to0}\mathbf Q_{11}=\sigma_r^2\Delta t\,\mathbf I_3+\sigma_w^2\Big(\tfrac{\Delta t^3}{3}\mathbf I_3+\tfrac{2\Delta t^5}{5!}\lfloor\hat{\boldsymbol\omega}\times\rfloor^2\Big),\quad
\lim_{|\boldsymbol\omega|\to0}\mathbf Q_{12}=-\sigma_w^2\Big(\tfrac{\Delta t^2}{2}\mathbf I_3-\tfrac{\Delta t^3}{3!}\lfloor\hat{\boldsymbol\omega}\times\rfloor+\tfrac{\Delta t^4}{4!}\lfloor\hat{\boldsymbol\omega}\times\rfloor^2\Big).
$$

### §2.8 [OV] 离散传播：完整 15×15 误差态 $F$ 与离散 $\Phi$ 块结构 [源 [OV] 离散传播页]

[OV] 误差态排序 $[{}^I_G\tilde{\boldsymbol\theta},{}^G\tilde{\mathbf p}_I,{}^G\tilde{\mathbf v}_I,\tilde{\mathbf b}_g,\tilde{\mathbf b}_a]$。量测模型
$$
{}^I\boldsymbol\omega_m={}^I\boldsymbol\omega+\mathbf b_g+\mathbf n_g,\qquad {}^I\mathbf a_m={}^I\mathbf a+{}^I_G\mathbf R\,{}^G\mathbf g+\mathbf b_a+\mathbf n_a.
$$
连续运动学：
$$
{}^I_G\dot{\bar q}=\tfrac12\boldsymbol\Omega({}^I\boldsymbol\omega)\,{}^I_G\bar q,\quad {}^G\dot{\mathbf p}_I={}^G\mathbf v_I,\quad {}^G\dot{\mathbf v}_I={}^I_G\mathbf R^\top\,{}^I\mathbf a,\quad \dot{\mathbf b}_g=\mathbf n_{wg},\quad \dot{\mathbf b}_a=\mathbf n_{wa}.
$$
[OV] 离散传播页给出离散一阶递推的 $\boldsymbol\Phi$（$5\times5$ 块，对 $[\boldsymbol\theta,\mathbf p,\mathbf v,\mathbf b_g,\mathbf b_a]$）与 $\mathbf G_k$（$5\times4$ 块），在采样周期 $\Delta t_k$、量测常值假设下评估。连续 $\mathbf F$ 的关键非零块（综合多源，[OV] + 上文）：
- $\partial\dot{\delta\boldsymbol\theta}/\partial\delta\boldsymbol\theta=-\lfloor\hat{\boldsymbol\omega}\times\rfloor$，$\partial\dot{\delta\boldsymbol\theta}/\partial\tilde{\mathbf b}_g=-\mathbf I_3$；
- $\partial{}^G\dot{\tilde{\mathbf p}}/\partial{}^G\tilde{\mathbf v}=\mathbf I_3$；
- $\partial{}^G\dot{\tilde{\mathbf v}}/\partial\delta\boldsymbol\theta=-{}^I_G\hat{\mathbf R}^\top\lfloor\hat{\mathbf a}\times\rfloor$，$\partial{}^G\dot{\tilde{\mathbf v}}/\partial\tilde{\mathbf b}_a=-{}^I_G\hat{\mathbf R}^\top$；
- 两 bias 行为零。

> **离散 $\Phi$ 的姿态块** $\boldsymbol\Phi_{\theta\theta}=\exp(-\lfloor\hat{\boldsymbol\omega}\times\rfloor\Delta t)=\boldsymbol\Theta$（即 §2.7 的 $\boldsymbol\Theta$）；速度/位置块由 $\mathbf F$ 幂展开得（含 $-{}^I_G\hat{\mathbf R}^\top\lfloor\hat{\mathbf a}\times\rfloor$ 与积分时间项）。完整解析式见 [OV] analytical propagation 页与 [Li13]；本书可取 [Trawny05] 闭式 + [Li13] 速度/位置块组合。

---

## §3 状态增广（State Augmentation）[源 [MR07] §III-C；[Sun18] §III-A 附录 B]

### §3.1 动机

每记录一张新图像，需把当前相机位姿**克隆**进状态向量（stochastic cloning）。这是处理特征量测的前提：EKF 更新时，每个被跟踪特征的量测用于约束**它被观测到的所有相机位姿**，故这些位姿必须在态内。

### §3.2 相机位姿计算（[MR07] 式 14）

由 IMU 位姿估计算出相机位姿：
$$
{}^C_G\hat{\bar q}={}^C_I\bar q\otimes{}^I_G\hat{\bar q},\qquad {}^G\hat{\mathbf p}_C={}^G\hat{\mathbf p}_I+\mathbf C_{\hat q}^\top\,{}^I\mathbf p_C,\tag{MR07-14}
$$
${}^C_I\bar q$ 为 IMU 与相机间旋转四元数，${}^I\mathbf p_C$ 为相机系原点相对 $\{I\}$ 的位置（二者已知/标定）。

[Sun18] 对应：${}^C_G\hat q={}^C_I\hat q\otimes{}^I_G\hat q$，${}^G\hat{\mathbf p}_C={}^G\hat{\mathbf p}_C+\mathbf C({}^I_G\hat q)\,{}^I\hat{\mathbf p}_C$（按其外参在态的写法）。

### §3.3 协方差增广（[MR07] 式 15–16）

把相机位姿估计附加到态后，EKF 协方差按下式增广：
$$
\mathbf P_{k|k}\leftarrow\begin{bmatrix}\mathbf I_{6N+15}\\ \mathbf J\end{bmatrix}\mathbf P_{k|k}\begin{bmatrix}\mathbf I_{6N+15}\\ \mathbf J\end{bmatrix}^\top,\tag{MR07-15}
$$
其中雅可比 $\mathbf J$ 由式 14 导出（[MR07] 式 16，$6\times(6N+15)$）：
$$
\mathbf J=\begin{bmatrix}\mathbf C({}^C_I\bar q) & \mathbf 0_{3\times9} & \mathbf 0_{3\times3} & \mathbf 0_{3\times6N}\\ \lfloor\mathbf C_{\hat q}^\top\,{}^I\mathbf p_C\times\rfloor & \mathbf 0_{3\times9} & \mathbf I_3 & \mathbf 0_{3\times6N}\end{bmatrix}.\tag{MR07-16}
$$

### §3.4 [Sun18] 修正版增广雅可比（附录 B，修正 [MR07] 式 16 笔误）[源 [Sun18] 附录 B]

[Sun18] 指出 [MR07] 式 16 有笔误，给出修正：增广雅可比 $\mathbf J=[\mathbf J_I\quad\mathbf 0_{6\times6N}]$，其中（误差态排序含外参）
$$
\mathbf J_I=\begin{bmatrix}\mathbf C({}^I_G\hat q) & \mathbf 0_{3\times9} & \mathbf 0_{3\times3} & \mathbf I_3 & \mathbf 0_{3\times3}\\ -\mathbf C({}^I_G\hat q)^\top\,\lfloor{}^I\hat{\mathbf p}_C\times\rfloor & \mathbf 0_{3\times9} & \mathbf I_3 & \mathbf 0_{3\times3} & \mathbf I_3\end{bmatrix}.
$$
增广协方差（[Sun18] 式 3）：
$$
\mathbf P_{k|k}\leftarrow\begin{bmatrix}\mathbf I_{21+6N}\\ \mathbf J\end{bmatrix}\mathbf P_{k|k}\begin{bmatrix}\mathbf I_{21+6N}\\ \mathbf J\end{bmatrix}^\top.\tag{Sun18-3}
$$
> 注：$\mathbf J_I$ 的最后两列（对外参 ${}^I_C\tilde{\boldsymbol\theta},{}^I\tilde{\mathbf p}_C$ 的偏导）正是 [MR07] 因把外参当常值而缺失、被 [Sun18] 补回的项。这体现「外参在线标定」需要的额外耦合。

> **本质洞察（综合时写盒）**：式 15 的 $[\mathbf I;\mathbf J]\mathbf P[\mathbf I;\mathbf J]^\top$ 即「保留原态、再追加一个由 $\mathbf J$ 线性映射出的新克隆位姿」，使新克隆与全态的互协方差被正确建立。这正是 **stochastic cloning**（Roumeliotis-Burdick）。它**不丢信息**（与边缘化对偶）。

---

## §4 量测模型（Measurement Model）—— MSCKF 核心 [源 [MR07] §III-D；[Sun18] §III-B 附录 C/D；[OV] 量测更新页]

### §4.1 一般残差形式（[MR07] 式 17）

EKF 需要一个**线性依赖误差态**的残差：
$$
\mathbf r=\mathbf H\tilde{\mathbf X}+\text{noise},\tag{MR07-17}
$$
$\mathbf H$ 为量测雅可比，噪声须零均值、白、且与误差态不相关，EKF 框架方可用。

### §4.2 单特征单目量测模型（[MR07] 式 18–19）

设静态特征 $f_j$ 被一组 $M_j$ 个相机位姿 $({}^{C_i}_G\bar q,{}^G\mathbf p_{C_i}),\ i\in\mathcal S_j$ 观测。每次观测：
$$
\mathbf z_i^{(j)}=\frac{1}{{}^{C_i}Z_j}\begin{bmatrix}{}^{C_i}X_j\\ {}^{C_i}Y_j\end{bmatrix}+\mathbf n_i^{(j)},\quad i\in\mathcal S_j,\tag{MR07-18}
$$
$\mathbf n_i^{(j)}$ 为 $2\times1$ 像素噪声，协方差 $\mathbf R_i^{(j)}=\sigma_{\rm im}^2\mathbf I_2$。特征在相机系坐标：
$$
{}^{C_i}\mathbf p_{f_j}=\begin{bmatrix}{}^{C_i}X_j\\ {}^{C_i}Y_j\\ {}^{C_i}Z_j\end{bmatrix}=\mathbf C({}^{C_i}_G\bar q)\big({}^G\mathbf p_{f_j}-{}^G\mathbf p_{C_i}\big),\tag{MR07-19}
$$
${}^G\mathbf p_{f_j}$ 为特征全局 3D 位置。它未知，第一步用**最小二乘**（见 §6 附录）估计 ${}^G\hat{\mathbf p}_{f_j}$（用量测 $\mathbf z_i^{(j)}$ 与对应相机位姿估计）。

### §4.3 残差线性化（[MR07] 式 20–22）

量测残差
$$
\mathbf r_i^{(j)}=\mathbf z_i^{(j)}-\hat{\mathbf z}_i^{(j)},\qquad \hat{\mathbf z}_i^{(j)}=\frac{1}{{}^{C_i}\hat Z_j}\begin{bmatrix}{}^{C_i}\hat X_j\\ {}^{C_i}\hat Y_j\end{bmatrix},\quad {}^{C_i}\hat{\mathbf p}_{f_j}=\mathbf C({}^{C_i}_G\hat{\bar q})({}^G\hat{\mathbf p}_{f_j}-{}^G\hat{\mathbf p}_{C_i}).\tag{MR07-20}
$$
对相机位姿与特征位置在估计处线性化：
$$
\mathbf r_i^{(j)}\simeq\mathbf H_{\mathbf X_i}^{(j)}\tilde{\mathbf X}+\mathbf H_{f_i}^{(j)}\,{}^G\tilde{\mathbf p}_{f_j}+\mathbf n_i^{(j)},\tag{MR07-21}
$$
$\mathbf H_{\mathbf X_i}^{(j)}$、$\mathbf H_{f_i}^{(j)}$ 分别是量测 $\mathbf z_i^{(j)}$ 对**态**与对**特征位置**的雅可比，${}^G\tilde{\mathbf p}_{f_j}$ 为特征位置估计误差。（[MR07] 称这两个雅可比的精确值见其 tech report [21]；本抽取在 §4.5/§4.6 用 [Sun18] 附录 C 与 [OV] 给出全式。）堆叠该特征全部 $M_j$ 个量测：
$$
\mathbf r^{(j)}\simeq\mathbf H_{\mathbf X}^{(j)}\tilde{\mathbf X}+\mathbf H_f^{(j)}\,{}^G\tilde{\mathbf p}_{f_j}+\mathbf n^{(j)},\tag{MR07-22}
$$
块向量/块矩阵，元素为 $\mathbf r_i^{(j)},\mathbf H_{\mathbf X_i}^{(j)},\mathbf H_{f_i}^{(j)},\mathbf n_i^{(j)}\ (i\in\mathcal S_j)$。不同图像量测独立，故 $\mathbf n^{(j)}$ 协方差 $\mathbf R^{(j)}=\sigma_{\rm im}^2\mathbf I_{2M_j}$。

### §4.4 零空间投影（消去特征误差）—— **MSCKF 第一核心创新**（[MR07] 式 23–24）

**问题**：因态 $\tilde{\mathbf X}$ 被用来算特征位置估计（见附录三角化），${}^G\tilde{\mathbf p}_{f_j}$ 与 $\tilde{\mathbf X}$ **相关**，故式 22 的 $\mathbf r^{(j)}$ 不符合式 17 形式，**不能直接做 EKF 更新**。

**解法**：把 $\mathbf r^{(j)}$ 投影到 $\mathbf H_f^{(j)}$ 的**左零空间**。设 $\mathbf A$ 为酉矩阵，其列张成 $\mathbf H_f^{(j)}$ 的左零空间，则
$$
\mathbf r_o^{(j)}=\mathbf A^\top(\mathbf z^{(j)}-\hat{\mathbf z}^{(j)})\simeq\mathbf A^\top\mathbf H_{\mathbf X}^{(j)}\tilde{\mathbf X}+\mathbf A^\top\mathbf n^{(j)},\tag{MR07-23}
$$
$$
=\mathbf H_o^{(j)}\tilde{\mathbf X}+\mathbf n_o^{(j)}.\tag{MR07-24}
$$
因 $2M_j\times3$ 的 $\mathbf H_f^{(j)}$ 满列秩，其左零空间维数 $2M_j-3$，故 $\mathbf r_o^{(j)}$ 是 $(2M_j-3)\times1$ 向量，**与特征坐标误差无关**，可做 EKF 更新。式 24 定义了「$f_j$ 被观测到的所有相机位姿之间的线性化约束」，是这些量测对 $M_j$ 个位姿提供的**全部可用信息**，故 EKF 更新**最优**（仅受线性化误差限制）。

**高效实现（[MR07] 关键工程注）**：$\mathbf A$ **无需显式求出**。$\mathbf r^{(j)}$ 与 $\mathbf H_{\mathbf X}^{(j)}$ 到 $\mathbf H_f^{(j)}$ 零空间的投影可用 **Givens 旋转**高效计算，仅 $O(M_j^2)$ 运算。又因 $\mathbf A$ 酉，投影后噪声协方差
$$
\mathbb E\{\mathbf n_o^{(j)}\mathbf n_o^{(j)\top}\}=\sigma_{\rm im}^2\,\mathbf A^\top\mathbf A=\sigma_{\rm im}^2\,\mathbf I_{2M_j-3}.\tag{MR07-之后}
$$

**[OV] 的等价 QR 写法**（零空间投影页）。线性化残差 $\tilde{\mathbf z}_{m,k}\simeq\mathbf H_x\tilde{\mathbf x}_k+\mathbf H_f\,{}^G\tilde{\mathbf p}_f+\mathbf n_k$。对 $\mathbf H_f$ 做 QR：
$$
\mathbf H_f=\begin{bmatrix}\mathbf Q_1 & \mathbf Q_2\end{bmatrix}\begin{bmatrix}\mathbf R_1\\ \mathbf 0\end{bmatrix}=\mathbf Q_1\mathbf R_1.
$$
左乘 $\mathbf Q_2^\top$（$\mathbf Q_2$ 列张成左零空间，$\mathbf Q_2^\top\mathbf Q_1=\mathbf 0$）：
$$
\mathbf Q_2^\top\tilde{\mathbf z}_m\simeq\mathbf Q_2^\top\mathbf H_x\tilde{\mathbf x}_k+\underbrace{\mathbf Q_2^\top\mathbf Q_1}_{=\mathbf 0}\mathbf R_1\,{}^G\tilde{\mathbf p}_f+\mathbf Q_2^\top\mathbf n_k\;\Rightarrow\;\tilde{\mathbf z}_{o,k}\simeq\mathbf H_{o,k}\tilde{\mathbf x}_k+\mathbf n_{o,k},
$$
其中 $\tilde{\mathbf z}_{o,k}=\mathbf Q_2^\top\tilde{\mathbf z}_m,\ \mathbf H_{o,k}=\mathbf Q_2^\top\mathbf H_x,\ \mathbf n_{o,k}=\mathbf Q_2^\top\mathbf n_k$，投影后噪声协方差 $\mathbf R_o=\mathbf Q_2^\top\mathbf R_d\mathbf Q_2$。维数关系：$\dim\mathbf H_f=2n\times3$，$\mathrm{rank}(\mathbf H_f)\le\min(2n,3)=3$，$\mathrm{nullity}(\mathbf H_f)=2n-3$；投影后量测维 $(2n-3)\times1$。

> **本质洞察**：左零空间投影把「未在态内的特征」从约束中代数地消去，等价于对该特征做 Schur 补边缘化（§3.5），但不必把特征加进态再删——这正是 MSCKF「复杂度只与特征数线性、不含特征于态」的来源。

### §4.5 单目量测雅可比 $\mathbf H_{\mathbf X},\mathbf H_f$ 的全链式分解 [源 [OV] 相机量测更新页]

[OV] 把量测 $\mathbf z=\mathbf h_d\circ\mathbf h_p\circ\mathbf h_t\circ\mathbf h_r$ 链式分解（$\mathbf h_r$ 特征表示、$\mathbf h_t$ 全局→相机欧式变换、$\mathbf h_p$ 透视投影、$\mathbf h_d$ 畸变）：
$$
\mathbf z_{m,k}=\mathbf h_d\big(\mathbf h_p(\mathbf h_t(\mathbf h_r(\boldsymbol\lambda,\cdots),{}^{C_k}_G\mathbf R,{}^G\mathbf p_{C_k})),\boldsymbol\zeta\big)+\mathbf n_k.
$$

**(a) 透视投影 $\mathbf h_p$**（相机系 3D 点 → 归一化平面）：${}^{C_k}\mathbf p_f=[{}^Cx,{}^Cy,{}^Cz]^\top$，
$$
\mathbf z_{n,k}=\mathbf h_p({}^{C_k}\mathbf p_f)=\begin{bmatrix}{}^Cx/{}^Cz\\ {}^Cy/{}^Cz\end{bmatrix},\qquad
\frac{\partial\mathbf h_p}{\partial{}^{C_k}\mathbf p_f}=\begin{bmatrix}\tfrac{1}{{}^Cz} & 0 & -\tfrac{{}^Cx}{({}^Cz)^2}\\ 0 & \tfrac{1}{{}^Cz} & -\tfrac{{}^Cy}{({}^Cz)^2}\end{bmatrix}.
$$

**(b) 欧式变换 $\mathbf h_t$**（全局特征 → 当前相机系）：
$$
{}^{C_k}\mathbf p_f={}^{C_k}_G\mathbf R({}^G\mathbf p_f-{}^G\mathbf p_{C_k}),\quad {}^G\mathbf p_{C_k}={}^G\mathbf p_{I_k}+{}^G_I\mathbf R\,{}^I\mathbf p_C,\quad {}^{C_k}_G\mathbf R={}^C_I\mathbf R\,{}^{I_k}_G\mathbf R,
$$
代入得 ${}^{C_k}\mathbf p_f={}^C_I\mathbf R\,{}^{I_k}_G\mathbf R({}^G\mathbf p_f-{}^G\mathbf p_{I_k})+{}^C\mathbf p_I$。各雅可比（用旋转的角向量扰动）：
$$
\frac{\partial\mathbf h_t}{\partial{}^G\mathbf p_f}={}^C_I\mathbf R\,{}^{I_k}_G\mathbf R,\quad
\frac{\partial\mathbf h_t}{\partial{}^{I_k}_G\mathbf R}={}^C_I\mathbf R\,\big\lfloor{}^{I_k}_G\mathbf R({}^G\mathbf p_f-{}^G\mathbf p_{I_k})\times\big\rfloor,\quad
\frac{\partial\mathbf h_t}{\partial{}^G\mathbf p_{I_k}}=-{}^C_I\mathbf R\,{}^{I_k}_G\mathbf R,
$$
外参雅可比（在线标定时）：
$$
\frac{\partial\mathbf h_t}{\partial{}^C_I\mathbf R}=\big\lfloor{}^C_I\mathbf R\,{}^{I_k}_G\mathbf R({}^G\mathbf p_f-{}^G\mathbf p_{I_k})\times\big\rfloor,\qquad \frac{\partial\mathbf h_t}{\partial{}^C\mathbf p_I}=\mathbf I_{3\times3}.
$$

**(c) 畸变 $\mathbf h_d$（径向-切向 OpenCV 模型）**：$\boldsymbol\zeta=[f_x,f_y,c_x,c_y,k_1,k_2,p_1,p_2]^\top$，$r^2=x_n^2+y_n^2$，
$$
\begin{bmatrix}u\\v\end{bmatrix}=\begin{bmatrix}f_x x+c_x\\ f_y y+c_y\end{bmatrix},\quad
\begin{aligned}x&=x_n(1+k_1r^2+k_2r^4)+2p_1x_ny_n+p_2(r^2+2x_n^2),\\ y&=y_n(1+k_1r^2+k_2r^4)+p_1(r^2+2y_n^2)+2p_2x_ny_n.\end{aligned}
$$
对归一化坐标的雅可比 $\partial\mathbf h_d/\partial\mathbf z_{n,k}$（[OV] 给出完整 $2\times2$ 式，含 $k_1,k_2,p_1,p_2$ 项；篇幅见原页，要点：对角主项 $f_x(1+k_1r^2+k_2r^4)+\cdots$）；对内参的雅可比 $\partial\mathbf h_d/\partial\boldsymbol\zeta$（$2\times8$）：
$$
\frac{\partial\mathbf h_d}{\partial\boldsymbol\zeta}=\begin{bmatrix}x & 0 & 1 & 0 & f_xx_nr^2 & f_xx_nr^4 & f_x\,2x_ny_n & f_x(r^2+2x_n^2)\\ 0 & y & 0 & 1 & f_yy_nr^2 & f_yy_nr^4 & f_y(r^2+2y_n^2) & f_y\,2x_ny_n\end{bmatrix}.
$$

**鱼眼（等距）模型**：$\theta=\mathrm{atan}(r),\ \theta_d=\theta(1+k_1\theta^2+k_2\theta^4+k_3\theta^6+k_4\theta^8)$，$x=\tfrac{x_n}{r}\theta_d,\ y=\tfrac{y_n}{r}\theta_d$；其 $\partial\mathbf h_d/\partial\mathbf z_{n,k}$ 按链式 $\tfrac{\partial uv}{\partial xy}\big(\tfrac{\partial xy}{\partial x_ny_n}+\tfrac{\partial xy}{\partial r}\tfrac{\partial r}{\partial x_ny_n}+\tfrac{\partial xy}{\partial\theta_d}\tfrac{\partial\theta_d}{\partial\theta}\tfrac{\partial\theta}{\partial r}\tfrac{\partial r}{\partial x_ny_n}\big)$ 组合，分量
$$
\tfrac{\partial uv}{\partial xy}=\mathrm{diag}(f_x,f_y),\ \tfrac{\partial xy}{\partial x_ny_n}=\tfrac{\theta_d}{r}\mathbf I_2,\ \tfrac{\partial xy}{\partial r}=\begin{bmatrix}-\tfrac{x_n}{r^2}\theta_d\\ -\tfrac{y_n}{r^2}\theta_d\end{bmatrix},\ \tfrac{\partial r}{\partial x_ny_n}=\big[\tfrac{x_n}{r}\ \tfrac{y_n}{r}\big],\ \tfrac{\partial\theta_d}{\partial\theta}=1+3k_1\theta^2+5k_2\theta^4+7k_3\theta^6+9k_4\theta^8,\ \tfrac{\partial\theta}{\partial r}=\tfrac{1}{r^2+1}.
$$

**(d) 链式合成**（[OV]）。对态 $\mathbf x$ 与对特征 $\boldsymbol\lambda$：
$$
\frac{\partial\mathbf z_k}{\partial\mathbf x}=\frac{\partial\mathbf h_d}{\partial\mathbf z_{n,k}}\frac{\partial\mathbf h_p}{\partial{}^{C_k}\mathbf p_f}\frac{\partial\mathbf h_t}{\partial\mathbf x}+\frac{\partial\mathbf h_d}{\partial\mathbf z_{n,k}}\frac{\partial\mathbf h_p}{\partial{}^{C_k}\mathbf p_f}\frac{\partial\mathbf h_t}{\partial{}^G\mathbf p_f}\frac{\partial\mathbf h_r}{\partial\mathbf x},
$$
$$
\mathbf H_x=\frac{\partial\mathbf h_d}{\partial\mathbf z_{n,k}}\frac{\partial\mathbf h_p}{\partial{}^{C_k}\mathbf p_f}\Big[\frac{\partial\mathbf h_t}{\partial\mathbf x}\Big]_{\text{pose,extrinsics}},\qquad
\mathbf H_f=\frac{\partial\mathbf h_d}{\partial\mathbf z_{n,k}}\frac{\partial\mathbf h_p}{\partial{}^{C_k}\mathbf p_f}\frac{\partial\mathbf h_t}{\partial{}^G\mathbf p_f}\frac{\partial\mathbf h_r}{\partial\boldsymbol\lambda}.
$$
对**全局表示**（$\mathbf h_r=\mathrm{id}$）第二项消失（$\partial\mathbf h_r/\partial\mathbf x=0$）；对**锚定（anchored）表示**两项都贡献，需对锚位姿与特征参数分别求雅可比。

### §4.6 特征表示函数 $\mathbf h_r$ 及其雅可比 $\partial f/\partial\boldsymbol\lambda$ [源 [OV] 相机量测更新页]

- **全局 XYZ**：${}^G\mathbf p_f=\boldsymbol\lambda=[{}^Gx,{}^Gy,{}^Gz]^\top$，$\partial f/\partial\boldsymbol\lambda=\mathbf I_3$。
- **全局逆深度（球坐标）**：${}^G\mathbf p_f=\tfrac1\rho[\cos\theta\sin\phi,\ \sin\theta\sin\phi,\ \cos\phi]^\top$，$\boldsymbol\lambda=[\theta,\phi,\rho]^\top$，
$$
\frac{\partial f}{\partial\boldsymbol\lambda}=\begin{bmatrix}-\tfrac1\rho s_\theta s_\phi & \tfrac1\rho c_\theta c_\phi & -\tfrac1{\rho^2}c_\theta s_\phi\\ \tfrac1\rho c_\theta s_\phi & \tfrac1\rho s_\theta c_\phi & -\tfrac1{\rho^2}s_\theta s_\phi\\ 0 & -\tfrac1\rho s_\phi & -\tfrac1{\rho^2}c_\phi\end{bmatrix}\quad(c_\bullet=\cos\bullet,\ s_\bullet=\sin\bullet).
$$
- **锚定 XYZ**（锚相机系 $\{C_a\}$，锚位姿 ${}^{I_a}_G\mathbf R,{}^G\mathbf p_{I_a}$）：${}^G\mathbf p_f={}^{I_a}_G\mathbf R^\top\,{}^C_I\mathbf R^\top(\boldsymbol\lambda-{}^C\mathbf p_I)+{}^G\mathbf p_{I_a}$，$\boldsymbol\lambda={}^{C_a}\mathbf p_f$，
$$
\tfrac{\partial f}{\partial\boldsymbol\lambda}={}^{I_a}_G\mathbf R^\top{}^C_I\mathbf R^\top,\ \tfrac{\partial f}{\partial{}^{I_a}_G\mathbf R}=-{}^{I_a}_G\mathbf R^\top\lfloor{}^C_I\mathbf R^\top({}^{C_a}\mathbf p_f-{}^C\mathbf p_I)\times\rfloor,\ \tfrac{\partial f}{\partial{}^G\mathbf p_{I_a}}=\mathbf I_3,
$$
$$
\tfrac{\partial f}{\partial{}^C_I\mathbf R}=-{}^{I_a}_G\mathbf R^\top{}^C_I\mathbf R^\top\lfloor({}^{C_a}\mathbf p_f-{}^C\mathbf p_I)\times\rfloor,\ \tfrac{\partial f}{\partial{}^C\mathbf p_I}=-{}^{I_a}_G\mathbf R^\top{}^C_I\mathbf R^\top.
$$
- **锚定逆深度（MSCKF 版，归一化方位）**：${}^G\mathbf p_f={}^{I_a}_G\mathbf R^\top{}^C_I\mathbf R^\top\big(\tfrac1\rho[\alpha,\beta,1]^\top-{}^C\mathbf p_I\big)+{}^G\mathbf p_{I_a}$，$\boldsymbol\lambda=[\alpha,\beta,\rho]^\top$，
$$
\frac{\partial f}{\partial\boldsymbol\lambda}={}^{I_a}_G\mathbf R^\top{}^C_I\mathbf R^\top\begin{bmatrix}\tfrac1\rho & 0 & -\tfrac1{\rho^2}\alpha\\ 0 & \tfrac1\rho & -\tfrac1{\rho^2}\beta\\ 0 & 0 & -\tfrac1{\rho^2}\end{bmatrix}.
$$
- **锚定逆深度（MSCKF 单深度，bearing $\hat{\mathbf b}$ 固定）**：${}^G\mathbf p_f={}^{I_a}_G\mathbf R^\top{}^C_I\mathbf R^\top\big(\tfrac1\rho\hat{\mathbf b}-{}^C\mathbf p_I\big)+{}^G\mathbf p_{I_a}$，$\boldsymbol\lambda=[\rho]$ 标量；更新时零空间投影（≥2 量测）消去对 $\hat{\mathbf b}$ 的依赖。

> **[OV] 特征表示说明**：OpenVINS 的 MSCKF 特征表示初始化时先 init 一个完整 3D 特征，再把 bearing 部分边缘化，bearing 在后续所有线性化中固定。

### §4.7 双目量测模型与雅可比（[Sun18] §III-B + 附录 C/D）

**双目量测**（左右相机位姿 $({}^{C_{i,1}}_G q,{}^G\mathbf p_{C_{i,1}}),({}^{C_{i,2}}_G q,{}^G\mathbf p_{C_{i,2}})$，态仅含左相机位姿，右相机由外参得；[Sun18] 式 4）：
$$
\mathbf z_i^j=\begin{bmatrix}u_{i,1}^j\\ v_{i,1}^j\\ u_{i,2}^j\\ v_{i,2}^j\end{bmatrix}=\begin{bmatrix}\tfrac{1}{{}^{C_{i,1}}Z_j}\mathbf I_2 & \mathbf 0_{2\times2}\\ \mathbf 0_{2\times2} & \tfrac{1}{{}^{C_{i,2}}Z_j}\mathbf I_2\end{bmatrix}\begin{bmatrix}{}^{C_{i,1}}X_j\\ {}^{C_{i,1}}Y_j\\ {}^{C_{i,2}}X_j\\ {}^{C_{i,2}}Y_j\end{bmatrix},
$$
即左右各取归一化像点（$\mathbb R^4$，无需立体校正，因不要求同特征在两图同一行）。特征在左/右相机系：
$$
{}^{C_{i,1}}\mathbf p_j=\mathbf C({}^{C_{i,1}}_G q)({}^G\mathbf p_j-{}^G\mathbf p_{C_{i,1}}),\quad
{}^{C_{i,2}}\mathbf p_j=\mathbf C({}^{C_{i,2}}_G q)({}^G\mathbf p_j-{}^G\mathbf p_{C_{i,2}})=\mathbf C({}^{C_{i,2}}_{C_{i,1}}q)\big({}^{C_{i,1}}\mathbf p_j-{}^{C_{i,1}}\mathbf p_{C_{i,2}}\big).
$$
残差线性化（[Sun18] 式 5）：$\mathbf r_i^j=\mathbf z_i^j-\hat{\mathbf z}_i^j=\mathbf H_{C_i}^j\tilde{\mathbf x}_{C_i}+\mathbf H_{f_i}^j\,{}^G\tilde{\mathbf p}_j+\mathbf n_i^j$。

**完整链式雅可比（[Sun18] 附录 C，式 7–8）**：
$$
\mathbf H_{C_i}^j=\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,1}}\mathbf p_j}\frac{\partial{}^{C_{i,1}}\mathbf p_j}{\partial\mathbf x_{C_{i,1}}}+\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,2}}\mathbf p_j}\frac{\partial{}^{C_{i,2}}\mathbf p_j}{\partial\mathbf x_{C_{i,1}}},\qquad
\mathbf H_{f_i}^j=\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,1}}\mathbf p_j}\frac{\partial{}^{C_{i,1}}\mathbf p_j}{\partial{}^G\mathbf p_j}+\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,2}}\mathbf p_j}\frac{\partial{}^{C_{i,2}}\mathbf p_j}{\partial{}^G\mathbf p_j},
$$
其中（投影雅可比，$4\times3$ 拼块）
$$
\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,1}}\mathbf p_j}=\frac{1}{{}^{C_{i,1}}\hat Z_j}\begin{bmatrix}1 & 0 & -\tfrac{{}^{C_{i,1}}\hat X_j}{{}^{C_{i,1}}\hat Z_j}\\ 0 & 1 & -\tfrac{{}^{C_{i,1}}\hat Y_j}{{}^{C_{i,1}}\hat Z_j}\\ 0 & 0 & 0\\ 0 & 0 & 0\end{bmatrix},\qquad
\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,2}}\mathbf p_j}=\frac{1}{{}^{C_{i,2}}\hat Z_j}\begin{bmatrix}0 & 0 & 0\\ 0 & 0 & 0\\ 1 & 0 & -\tfrac{{}^{C_{i,2}}\hat X_j}{{}^{C_{i,1}}\hat Z_j}\\ 0 & 1 & -\tfrac{{}^{C_{i,2}}\hat Y_j}{{}^{C_{i,1}}\hat Z_j}\end{bmatrix},
$$
$$
\frac{\partial{}^{C_{i,1}}\mathbf p_j}{\partial\mathbf x_{C_{i,1}}}=\Big[\lfloor{}^{C_{i,1}}\hat{\mathbf p}_j\times\rfloor\quad -\mathbf C({}^{C_{i,1}}_G\hat q)\Big],\qquad
\frac{\partial{}^{C_{i,1}}\mathbf p_j}{\partial{}^G\mathbf p_j}=\mathbf C({}^{C_{i,1}}_G\hat q),
$$
$$
\frac{\partial{}^{C_{i,2}}\mathbf p_j}{\partial\mathbf x_{C_{i,1}}}=\mathbf C({}^{C_{i,2}}_{C_{i,1}}q)\Big[\lfloor{}^{C_{i,1}}\hat{\mathbf p}_j\times\rfloor\quad -\mathbf C({}^{C_{i,1}}_G\hat q)\Big],\qquad
\frac{\partial{}^{C_{i,2}}\mathbf p_j}{\partial{}^G\mathbf p_j}=\mathbf C({}^{C_{i,2}}_{C_{i,1}}q)\,\mathbf C({}^{C_{i,1}}_G\hat q).
$$

**附录 D：单帧量测的零空间退化证明（[Sun18]）**。引入简记
$$
\frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,1}}\mathbf p_j}=\begin{bmatrix}\mathbf J_1\\ \mathbf 0\end{bmatrix},\quad \frac{\partial\mathbf z_i^j}{\partial{}^{C_{i,2}}\mathbf p_j}=\begin{bmatrix}\mathbf 0\\ \mathbf J_2\end{bmatrix},\quad \frac{\partial{}^{C_{i,1}}\mathbf p_j}{\partial\mathbf x_{C_{i,1}}}=\mathbf H_1,\quad \frac{\partial{}^{C_{i,1}}\mathbf p_j}{\partial{}^G\mathbf p_j}=\mathbf H_2,\quad \mathbf C({}^{C_{i,1}}_{C_{i,2}}q)=\mathbf R,
$$
则
$$
\mathbf H_{C_i}^j=\begin{bmatrix}\mathbf J_1\mathbf H_1\\ \mathbf J_2\mathbf R^\top\mathbf H_1\end{bmatrix},\qquad \mathbf H_{f_i}^j=\begin{bmatrix}\mathbf J_1\mathbf H_2\\ \mathbf J_2\mathbf R^\top\mathbf H_2\end{bmatrix}.
$$
设 $\mathbf v=[\mathbf v_1^\top,\mathbf v_2^\top]^\top\in\mathbb R^4$ 为 $\mathbf H_{f_i}^j$ 的左零空间，则 $\mathbf v^\top\mathbf H_{f_i}^j=(\mathbf v_1^\top\mathbf J_1+\mathbf v_2^\top\mathbf J_2\mathbf R^\top)\mathbf H_2=\mathbf 0$。因 $\mathbf H_2=\mathbf C({}^{C_{i,1}}_G\hat q)$ 是旋转阵、$\mathrm{rank}(\mathbf H_2)=3$，故 $\mathbf v_1^\top\mathbf J_1+\mathbf v_2^\top\mathbf J_2\mathbf R^\top=\mathbf 0$。由此立即得 $\mathbf v$ 也是 $\mathbf H_{C_i}^j$ 的左零空间：$\mathbf v^\top\mathbf H_{C_i}^j=(\mathbf v_1^\top\mathbf J_1+\mathbf v_2^\top\mathbf J_2\mathbf R^\top)\mathbf H_1=\mathbf 0$。**结论**：单帧双目量测投影到 $\mathbf H_f$ 零空间后 $\mathbf H_{C_i}$ 也被零化 ⇒ 单帧量测无法直接用于更新（其 $\mathbf H_{f_i}$ 的零空间是 $\mathbf H_{C_i}$ 零空间的子空间，得到平凡量测模型）。这解释了 MSCKF 为何必须跨多帧约束、为何 marginalize 单帧观测无意义。

---

## §5 可观性约束与一致性（FEJ / OC-EKF）[源 [Sun18] §III-C；[OV] FEJ 页；[Li13]]

### §5.1 VIO 的不可观方向（4 维）

如 [Li13]、[Huang10] 所证：6-DOF 运动的 EKF-VIO 有 **4 个不可观方向**——对应**全局位置（3 维平移）**与**绕重力轴的全局旋转（1 维 yaw）**。朴素 EKF-VIO 会在 yaw 上**虚假获得信息**（spurious information），根因是**过程步与量测步在同一时刻的线性化点不同**。

### §5.2 不可观零空间 $\mathcal N$ 的闭式（[OV] FEJ 页）

观测矩阵
$$
\mathcal O=\begin{bmatrix}\mathbf H_0\boldsymbol\Phi_{(0,0)}\\ \mathbf H_1\boldsymbol\Phi_{(1,0)}\\ \mathbf H_2\boldsymbol\Phi_{(2,0)}\\ \vdots\\ \mathbf H_k\boldsymbol\Phi_{(k,0)}\end{bmatrix}.
$$
其 4 维不可观零空间（对 $[{}^I_G\tilde{\boldsymbol\theta};{}^G\tilde{\mathbf p}_I;{}^G\tilde{\mathbf v}_I;{}^G\tilde{\mathbf p}_f]$，前列对应 yaw、后 3 列对应平移）：
$$
\mathcal N=\begin{bmatrix}{}^{I_0}_G\mathbf R\,{}^G\mathbf g & \mathbf 0_{3\times3}\\ -\lfloor{}^G\mathbf p_{I_0}\times\rfloor{}^G\mathbf g & \mathbf I_{3\times3}\\ -\lfloor{}^G\mathbf v_{I_0}\times\rfloor{}^G\mathbf g & \mathbf 0_{3\times3}\\ -\lfloor{}^G\mathbf p_f\times\rfloor{}^G\mathbf g & \mathbf I_{3\times3}\end{bmatrix},
$$
对应「绕重力(yaw)的全局旋转 + 全局平移」。理想系统满足 $\mathcal O\,\mathcal N=\mathbf 0$（4 维不可观）。

### §5.3 标准 EKF 为何不一致 & FEJ 的修复（[OV] FEJ 页）

**不一致根因**：标准 EKF 用**当前估计**算转移阵与量测雅可比，使各时刻行中特征估计不一致 ${}^G\hat{\mathbf p}_f(t_0)\ne\cdots\ne{}^G\hat{\mathbf p}_f(t_k)$，破坏 $\mathcal N$、把不可观维数从 4 错误降为 3 ⇒ 滤波沿 yaw 不可观方向**错误获得信息**，导致**误差偏大、协方差偏小、不一致（过自信）**。

**FEJ（First-Estimate Jacobians）**：把雅可比固定在**首次估计**线性化点（first estimate $\bar{\mathbf x}_k$）而非当前估计上评估，强制转移阵满足复合一致性：
$$
\boldsymbol\Phi_{(k+1,k-1)}(\mathbf x_{k+1|k},\mathbf x_{k-1|k-1})=\boldsymbol\Phi_{(k+1,k)}(\mathbf x_{k+1|k},\mathbf x_{k|k-1})\,\boldsymbol\Phi_{(k,k-1)}(\mathbf x_{k|k-1},\mathbf x_{k-1|k-1}),
$$
从而量测雅可比仅含首次估计点：
$$
\tilde{\mathbf z}_{k+1}=\mathbf z_{k+1}-\mathbf h(\hat{\mathbf x}_k)\simeq\bar{\mathbf H}_k(\mathbf x_k-\hat{\mathbf x}_k)+\mathbf n_k,
$$
$\bar{\mathbf H}_k$ 只由首次估计线性化点构成 ⇒ $\mathcal N$ 维数保持 4。[OV] 另提 **FEJ2**、**OC** 作为后续改进。

### §5.4 OC-EKF（[Sun18] §III-C 的选择与理由）

[Sun18] 实现采用 **OC-EKF（Observability-Constrained EKF）**，理由（引 [Huang10]/[Hesch12]）：(i) 不像 FEJ-EKF 那样重度依赖准确初值；(ii) 相比 Robocentric Mapping Filter，态内相机位姿可表示在惯性系（而非最新 IMU 系），使传播步中已有相机位姿的不确定度不受最新 IMU 态不确定度影响。维持一致性的几类方法：FEJ-EKF、OC-EKF、Robocentric Mapping Filter。

> **物理直觉（综合时写盒）**：全局位置与 yaw 测不出，是因为 VIO 只有「相对」量测（IMU 测加速度/角速度增量、相机测相对几何）。把这 4 个方向人为「测出来」就是过自信，长航时必发散/跳变。FEJ/OC 的作用是**保证线性化系统也保留这 4 维不可观**。

---

## §6 EKF 更新（含量测压缩 QR）[源 [MR07] §III-E；[OV] 量测压缩页]

### §6.1 更新触发条件（[MR07] §III-E）

EKF 更新由两类事件触发：
1. **特征跟踪结束**：被跟踪若干帧的特征不再被检测到，则用其全部量测做更新（最常见，特征移出视野）。
2. **状态满**：每记录新图像就克隆当前相机位姿；当达到最大位姿数 $N_{\max}$，须移除旧位姿前先用其量测。[MR07] 选 $N_{\max}/3$ 个**时间均匀分布**的位姿（从第二旧开始）删除，删前用这些位姿的公共特征约束做一次更新；**始终保留最旧位姿**（更大基线→更有价值的定位信息）。

### §6.2 堆叠所有特征 + 量测压缩 QR（[MR07] 式 25–28）

把 $L$ 个特征的 $\mathbf r_o^{(j)}$ 堆叠：
$$
\mathbf r_o=\mathbf H_{\mathbf X}\tilde{\mathbf X}+\mathbf n_o,\tag{MR07-25}
$$
噪声协方差 $\mathbf R_o=\sigma_{\rm im}^2\mathbf I_d$，$d=\sum_{j=1}^L(2M_j-3)$。$d$ 可能很大（例：10 特征 × 各 10 位姿 ⇒ $d=170$）。为降更新成本，对 $\mathbf H_{\mathbf X}$ 做 QR：
$$
\mathbf H_{\mathbf X}=\begin{bmatrix}\mathbf Q_1 & \mathbf Q_2\end{bmatrix}\begin{bmatrix}\mathbf T_H\\ \mathbf 0\end{bmatrix},
$$
$\mathbf Q_1,\mathbf Q_2$ 列分别张成 $\mathbf H_{\mathbf X}$ 的值域与零空间，$\mathbf T_H$ 上三角。代入式 25：
$$
\mathbf r_o=\begin{bmatrix}\mathbf Q_1 & \mathbf Q_2\end{bmatrix}\begin{bmatrix}\mathbf T_H\\ \mathbf 0\end{bmatrix}\tilde{\mathbf X}+\mathbf n_o\;\Rightarrow\;\begin{bmatrix}\mathbf Q_1^\top\mathbf r_o\\ \mathbf Q_2^\top\mathbf r_o\end{bmatrix}=\begin{bmatrix}\mathbf T_H\\ \mathbf 0\end{bmatrix}\tilde{\mathbf X}+\begin{bmatrix}\mathbf Q_1^\top\mathbf n_o\\ \mathbf Q_2^\top\mathbf n_o\end{bmatrix}.\tag{MR07-26/27}
$$
$\mathbf Q_2^\top\mathbf r_o$ 仅含噪声，可完全丢弃。故用压缩残差更新（[MR07] 式 28）：
$$
\mathbf r_n=\mathbf Q_1^\top\mathbf r_o=\mathbf T_H\tilde{\mathbf X}+\mathbf n_n,
$$
$\mathbf n_n=\mathbf Q_1^\top\mathbf n_o$，协方差 $\mathbf R_n=\mathbf Q_1^\top\mathbf R_o\mathbf Q_1=\sigma_{\rm im}^2\mathbf I_r$（$r$ 为 $\mathbf Q_1$ 列数 = 态维上界）。

### §6.3 卡尔曼增益与更新（[MR07] 式 29–31）

$$
\mathbf K=\mathbf P\,\mathbf T_H^\top\big(\mathbf T_H\mathbf P\,\mathbf T_H^\top+\mathbf R_n\big)^{-1},\tag{MR07-29}
$$
$$
\Delta\mathbf X=\mathbf K\,\mathbf r_n,\tag{MR07-30}
$$
协方差更新（**Joseph 形式**，[MR07] 式 31）：
$$
\mathbf P_{k+1|k+1}=(\mathbf I_\xi-\mathbf K\mathbf T_H)\,\mathbf P_{k+1|k}\,(\mathbf I_\xi-\mathbf K\mathbf T_H)^\top+\mathbf K\mathbf R_n\mathbf K^\top,
$$
$\xi=6N+15$ 为协方差维。

**复杂度（[MR07] §III-E 末）**：$\mathbf r_n,\mathbf T_H$ 可用 Givens 旋转以 $O(r^2d)$ 算（无需显式形成 $\mathbf Q_1$）；式 31 是 $O(\xi^3)$。故更新代价 $\max(O(r^2d),O(\xi^3))$。若不压缩直接用 $\mathbf r_o$，增益代价 $O(d^3)$；因常有 $d\gg\xi,r$，压缩节省巨大。

### §6.4 [OV] 的等价量测压缩与标准 EKF 更新

[OV] 量测压缩页：对 $\mathbf H_{o,k}$ 做薄 QR $\mathbf H_{o,k}=[\mathbf Q_1\ \mathbf Q_2][\mathbf R_1;\mathbf 0]=\mathbf Q_1\mathbf R_1$，左乘 $\mathbf Q_1^\top$：
$$
\mathbf Q_1^\top\tilde{\mathbf z}_{o,k}\simeq\mathbf R_1\tilde{\mathbf x}_k+\mathbf Q_1^\top\mathbf n_o\;\Rightarrow\;\tilde{\mathbf z}_{n,k}\simeq\mathbf H_{n,k}\tilde{\mathbf x}_k+\mathbf n_n\ (\mathbf H_{n,k}=\mathbf R_1).
$$
标准 EKF 更新（[OV] 量测更新页）：
$$
\mathbf K=\mathbf P_k^\ominus\mathbf H_k^\top(\mathbf H_k\mathbf P_k^\ominus\mathbf H_k^\top+\mathbf R_k)^{-1},\quad
\hat{\mathbf x}_k^\oplus=\hat{\mathbf x}_k^\ominus+\mathbf K(\mathbf z_{m,k}-\mathbf H_k\hat{\mathbf x}_k^\ominus),
$$
$$
\mathbf P_{xx}^\oplus=\mathbf P_k^\ominus-\mathbf P_k^\ominus\mathbf H_k^\top(\mathbf H_k\mathbf P_k^\ominus\mathbf H_k^\top+\mathbf R_k)^{-1}\mathbf H_k\mathbf P_k^\ominus.
$$
（对 MSCKF，$\mathbf H_k=\mathbf H_{n,k}$、$\tilde{\mathbf z}_k=\tilde{\mathbf z}_{n,k}$、$\mathbf R_k=\mathbf R_n$。）

### §6.5 误差态注入与重置

EKF 更新得 $\Delta\mathbf X$ 后，把姿态误差 $\delta\boldsymbol\theta$ 经误差四元数注入名义四元数：${}^I_G\bar q\leftarrow\delta\bar q(\Delta\boldsymbol\theta)\otimes{}^I_G\hat{\bar q}$（[MR07] 左乘约定），其余加性量直接相加；随后误差态重置为 0。（[MR07] 未单列此步，但属 ESKF 标准流程；详细注入/重置见 ESKF 抽取 `kalman_eskf__sola_eskf.md`。）

### §6.6 异常值剔除：Mahalanobis 距离（卡方）门限

[MR07] §IV 提到用「简单的 Mahalanobis 距离检验」剔除动态物体上的外点（多观测使非静态特征易于识别）。标准做法（综合常识，[OV] 该页未列式）：对候选量测 $\mathbf r$ 计算
$$
\gamma=\mathbf r^\top(\mathbf H\mathbf P\mathbf H^\top+\mathbf R)^{-1}\mathbf r,
$$
与卡方分布阈值 $\chi^2_{\alpha,\dim(\mathbf r)}$ 比较，超过则判外点丢弃。

---

## §7 特征三角化（逆深度最小二乘）[源 [MR07] 附录]

为算被跟踪特征 $f_j$ 的位置，[MR07] 用 **intersection（交会）**，并用**逆深度参数化**避免局部极小、提高数值稳定。设 $\{C_n\}$ 为首次观测该特征的相机系，则第 $i$ 时刻相机系下特征坐标（[MR07] 式 32）：
$$
{}^{C_i}\mathbf p_{f_j}=\mathbf C({}^{C_i}_{C_n}\bar q)\,{}^{C_n}\mathbf p_{f_j}+{}^{C_i}\mathbf p_{C_n},\quad i\in\mathcal S_j,
$$
$\mathbf C({}^{C_i}_{C_n}\bar q),{}^{C_i}\mathbf p_{C_n}$ 为时刻 $n,i$ 相机间旋转与平移。改写（式 33–34）：
$$
{}^{C_i}\mathbf p_{f_j}={}^{C_n}Z_j\Big(\mathbf C({}^{C_i}_{C_n}\bar q)\begin{bmatrix}\tfrac{{}^{C_n}X_j}{{}^{C_n}Z_j}\\ \tfrac{{}^{C_n}Y_j}{{}^{C_n}Z_j}\\ 1\end{bmatrix}+\tfrac{1}{{}^{C_n}Z_j}\,{}^{C_i}\mathbf p_{C_n}\Big)={}^{C_n}Z_j\Big(\mathbf C({}^{C_i}_{C_n}\bar q)\begin{bmatrix}\alpha_j\\ \beta_j\\ 1\end{bmatrix}+\rho_j\,{}^{C_i}\mathbf p_{C_n}\Big),
$$
其中**逆深度参数**（式 36）：
$$
\alpha_j=\frac{{}^{C_n}X_j}{{}^{C_n}Z_j},\qquad \beta_j=\frac{{}^{C_n}Y_j}{{}^{C_n}Z_j},\qquad \rho_j=\frac{1}{{}^{C_n}Z_j}.
$$
记 ${}^{C_i}\mathbf p_{f_j}={}^{C_n}Z_j\,[h_{i1},h_{i2},h_{i3}]^\top$（$h_{i1},h_{i2},h_{i3}$ 为 $\alpha_j,\beta_j,\rho_j$ 的标量函数，式 35）。代入量测式 18 得仅含 $\alpha_j,\beta_j,\rho_j$ 的量测方程（式 37）：
$$
\mathbf z_i^{(j)}=\frac{1}{h_{i3}(\alpha_j,\beta_j,\rho_j)}\begin{bmatrix}h_{i1}(\alpha_j,\beta_j,\rho_j)\\ h_{i2}(\alpha_j,\beta_j,\rho_j)\end{bmatrix}+\mathbf n_i^{(j)}.
$$
用 **Gauss-Newton 最小二乘**求 $\hat\alpha_j,\hat\beta_j,\hat\rho_j$；再算全局特征位置（式 38）：
$$
{}^G\hat{\mathbf p}_{f_j}=\frac{1}{\hat\rho_j}\,\mathbf C^\top({}^{C_n}_G\hat{\bar q})\begin{bmatrix}\hat\alpha_j\\ \hat\beta_j\\ 1\end{bmatrix}+{}^G\hat{\mathbf p}_{C_n}.
$$
> **优化中相机位姿当常量、其协方差被忽略**，故最小二乘很高效（牺牲特征位置最优性）。但一阶近似下，这些特征误差**不影响量测残差**（见式 23 的零空间投影），故性能无显著退化。

> **[OV] 延迟特征初始化（SLAM 特征，[OV] update-delay 页）**。当要把特征加进态（做 SLAM 特征而非 MSCKF 特征）时：堆叠残差 $\mathbf r=\mathbf H_x\tilde{\mathbf x}+\mathbf H_f\tilde{\mathbf f}+\mathbf n$，Givens 旋转把特征与态解耦：
> $$\begin{bmatrix}\mathbf r_1\\ \mathbf r_2\end{bmatrix}=\begin{bmatrix}\mathbf H_{x1}\\ \mathbf H_{x2}\end{bmatrix}\tilde{\mathbf x}+\begin{bmatrix}\mathbf H_{f1}\\ \mathbf 0\end{bmatrix}\tilde{\mathbf f}+\begin{bmatrix}\mathbf n_1\\ \mathbf n_2\end{bmatrix},$$
> 特征误差 $\tilde{\mathbf f}=\mathbf H_{f1}^{-1}(\mathbf r_1-\mathbf n_1-\mathbf H_{x1}\tilde{\mathbf x})$，特征协方差与互协方差
> $$\mathbf P_{ff}=\mathbf H_{f1}^{-1}(\mathbf H_{x1}\mathbf P_{xx}\mathbf H_{x1}^\top+\mathbf R_1)\mathbf H_{f1}^{-\top},\quad \mathbf P_{xf}=-\mathbf P_{xx}\mathbf H_{x1}^\top\mathbf H_{f1}^{-\top},$$
> 增广 $\mathbf P_{\rm aug}=\begin{bmatrix}\mathbf P_{xx}&\mathbf P_{xf}\\ \mathbf P_{xf}^\top&\mathbf P_{ff}\end{bmatrix}$，余下行 $\mathbf r_2=\mathbf H_{x2}\tilde{\mathbf x}+\mathbf n_2$ 作 MSCKF 更新。

### §7.1 MSCKF 完整算法伪码（[MR07] Algorithm 1）

```
Algorithm 1  Multi-State Constraint Filter（[MR07]）
─────────────────────────────────────────────────────
Propagation:  每收到一个 IMU 量测，
              传播滤波态与协方差（§2 / III-B）。

Image registration: 每记录一张新图像，
   • 用当前相机位姿估计的拷贝增广态与协方差（§3 / III-C）；
   • 启动图像处理模块（特征检测/跟踪）。

Update: 当某图像的特征量测可用时，做一次 EKF 更新
        （§4 量测模型 / §6 EKF 更新；III-D 与 III-E）。
─────────────────────────────────────────────────────
```
**逐特征更新流水线（综合 §4–§7）**：
1. 取一组待处理特征（跟踪结束 / 位姿将删）。
2. 对每特征：用逆深度 GN 最小二乘三角化 ${}^G\hat{\mathbf p}_{f_j}$（§7）。
3. 算 $\mathbf H_{\mathbf X}^{(j)},\mathbf H_f^{(j)}$ 与残差 $\mathbf r^{(j)}$（§4.5/§4.7）。
4. 左零空间投影消去特征：$\mathbf r_o^{(j)}=\mathbf A^\top\mathbf r^{(j)}$（Givens，§4.4）。
5. （可选）Mahalanobis 卡方门限剔外点（§6.6）。
6. 堆叠所有特征 $\mathbf r_o=\mathbf H_{\mathbf X}\tilde{\mathbf X}+\mathbf n_o$；QR 量测压缩得 $\mathbf r_n=\mathbf T_H\tilde{\mathbf X}+\mathbf n_n$（§6.2）。
7. 算增益、改正态、Joseph 更协方差（§6.3）。
8. 注入误差、重置（§6.5）；删除该删的相机克隆。

---

# 第二部分：服务本章的配套主线（初始化 + 优化主线 + 对比）

## §8 VIO 初始化（[源 [Qin18] §V VINS-Mono 松耦合初始化]）

> MSCKF 自身可在缓慢/已知初始条件下启动，但**鲁棒初始化**是 VINS 的关键。本章须覆盖初始化，取 VINS-Mono 经典「vision-only SfM + 视觉惯性对齐」方案（松耦合给初值，再喂紧耦合）。

### §8.1 动机

单目紧耦合 VIO 高度非线性，尺度对单目不直接可观，无好初值难直接融合。静止初始条件假设不当（实际常在运动中初始化），IMU 大 bias 时更复杂。VINS-Mono 用**松耦合**得初值：vision-only SfM/SLAM 易自举，再把度量 IMU 预积分与 vision-only SfM 对齐，粗恢复**尺度、重力、速度、bias**。

### §8.2 视觉 SfM（[Qin18] §V-A）

维护滑窗。检查最新帧与所有先前帧的特征对应；若稳定跟踪（>30 特征）且视差足够（>20 旋转补偿像素），用**五点法**恢复相对旋转与 up-to-scale 平移；否则保留最新帧等新帧。五点成功则任意设尺度、三角化这两帧共视特征；据此对窗内其余帧 **PnP** 求位姿；最后**全局 BA** 最小化所有特征观测重投影误差。设首相机系 $(\cdot)^{c_0}$ 为参考。由粗略外参 $(\mathbf p_b^c,\mathbf q_b^c)$ 把位姿从相机系转 body 系（式 14）：
$$
\mathbf q_{b_k}^{c_0}=\mathbf q_{c_k}^{c_0}\otimes(\mathbf q_b^c)^{-1},\qquad s\,\bar{\mathbf p}_{b_k}^{c_0}=s\,\bar{\mathbf p}_{c_k}^{c_0}-\mathbf R_{b_k}^{c_0}\mathbf p_b^c,
$$
$s$ 为对齐视觉结构到度量尺度的标量（求 $s$ 是初始化成功的关键）。

### §8.3 视觉惯性对齐（[Qin18] §V-B）

**① 陀螺 bias 标定**（式 15）。两连续帧 $b_k,b_{k+1}$，由 SfM 得 $\mathbf q_{b_k}^{c_0},\mathbf q_{b_{k+1}}^{c_0}$，由预积分得 $\hat{\boldsymbol\gamma}_{b_{k+1}}^{b_k}$。线性化预积分对陀螺 bias 并最小化：
$$
\min_{\delta\mathbf b_w}\sum_{k\in\mathcal B}\big\|\,\mathbf q_{b_{k+1}}^{c_0\,-1}\otimes\mathbf q_{b_k}^{c_0}\otimes\boldsymbol\gamma_{b_{k+1}}^{b_k}\,\big\|^2,\qquad \boldsymbol\gamma_{b_{k+1}}^{b_k}\approx\hat{\boldsymbol\gamma}_{b_{k+1}}^{b_k}\otimes\begin{bmatrix}1\\ \tfrac12\mathbf J^\gamma_{b_w}\delta\mathbf b_w\end{bmatrix},
$$
$\mathbf J^\gamma_{b_w}$ 为 §9.1 的 bias 雅可比。得 $\mathbf b_w$ 初值后用新陀螺 bias **重传播**所有预积分项 $\hat{\boldsymbol\alpha},\hat{\boldsymbol\beta},\hat{\boldsymbol\gamma}$。**（VINS 初始化阶段忽略加速度计 bias**——其与重力耦合、初始化短时难观测。）

**② 速度/重力/尺度初始化**（式 16–20）。待估
$$
\mathcal X_I=[\mathbf v_{b_0}^{b_0},\mathbf v_{b_1}^{b_1},\cdots,\mathbf v_{b_n}^{b_n},\mathbf g^{c_0},s],
$$
$\mathbf v_{b_k}^{b_k}$ 为第 $k$ 帧 body 系速度，$\mathbf g^{c_0}$ 为 $c_0$ 系重力，$s$ 尺度。两连续帧由预积分式（式 17）：
$$
\boldsymbol\alpha_{b_{k+1}}^{b_k}=\mathbf R_{c_0}^{b_k}\big(s(\bar{\mathbf p}_{b_{k+1}}^{c_0}-\bar{\mathbf p}_{b_k}^{c_0})+\tfrac12\mathbf g^{c_0}\Delta t_k^2-\mathbf R_{b_k}^{c_0}\mathbf v_{b_k}^{b_k}\Delta t_k\big),
$$
$$
\boldsymbol\beta_{b_{k+1}}^{b_k}=\mathbf R_{c_0}^{b_k}\big(\mathbf R_{b_{k+1}}^{c_0}\mathbf v_{b_{k+1}}^{b_{k+1}}+\mathbf g^{c_0}\Delta t_k-\mathbf R_{b_k}^{c_0}\mathbf v_{b_k}^{b_k}\big).
$$
合成线性量测模型（式 18–19）：
$$
\hat{\mathbf z}_{b_{k+1}}^{b_k}=\begin{bmatrix}\hat{\boldsymbol\alpha}_{b_{k+1}}^{b_k}-\mathbf p_b^c+\mathbf R_{c_0}^{b_k}\mathbf R_{b_{k+1}}^{c_0}\mathbf p_b^c\\ \hat{\boldsymbol\beta}_{b_{k+1}}^{b_k}\end{bmatrix}=\mathbf H_{b_{k+1}}^{b_k}\mathcal X_I+\mathbf n_{b_{k+1}}^{b_k},
$$
$$
\mathbf H_{b_{k+1}}^{b_k}=\begin{bmatrix}-\mathbf I\Delta t_k & \mathbf 0 & \tfrac12\mathbf R_{c_0}^{b_k}\Delta t_k^2 & \mathbf R_{c_0}^{b_k}(\bar{\mathbf p}_{c_{k+1}}^{c_0}-\bar{\mathbf p}_{c_k}^{c_0})\\ -\mathbf I & \mathbf R_{c_0}^{b_k}\mathbf R_{b_{k+1}}^{c_0} & \mathbf R_{c_0}^{b_k}\Delta t_k & \mathbf 0\end{bmatrix},
$$
解线性最小二乘（式 20）$\min_{\mathcal X_I}\sum_{k\in\mathcal B}\|\hat{\mathbf z}_{b_{k+1}}^{b_k}-\mathbf H_{b_{k+1}}^{b_k}\mathcal X_I\|^2$，得各帧速度、$c_0$ 系重力与尺度。

**③ 重力细化**（[Qin18] §V-B-3，图 5 + 算法 1）。重力模长已知（$g\approx9.81$），故重力只剩 **2 DOF**，在其切空间用两变量重参数化：$\mathbf g=g\cdot\hat{\bar{\mathbf g}}+w_1\mathbf b_1+w_2\mathbf b_2$，$\hat{\bar{\mathbf g}}$ 为当前重力方向单位向量，$\mathbf b_1,\mathbf b_2$ 为切平面正交基。把式 17 中 $\mathbf g$ 替换为 $g\cdot\hat{\bar{\mathbf g}}+w_1\mathbf b_1+w_2\mathbf b_2$，连同其余变量解 $w_1,w_2$，迭代至 $\hat{\bar{\mathbf g}}$ 收敛。
```
Algorithm 1  Finding b1 and b2（[Qin18]）
if ĝ ≠ [1,0,0] then  b1 ← normalize(ĝ × [1,0,0]);
else                 b1 ← normalize(ĝ × [0,0,1]);
end
b2 ← ĝ × b1;
```

**④ 完成初始化**（§V-B-4）。重力细化后，把重力转到 z 轴即得世界系与 $c_0$ 系的旋转 $\mathbf q_{c_0}^w$；把所有变量从 $c_0$ 系转世界系 $w$，body 速度也转世界系，视觉 SfM 平移分量按尺度转为度量单位。至此初始化完成，度量值喂入紧耦合单目 VIO。

> **与 MSCKF 初始化对比（综合用）**：MSCKF 类滤波常用**静止初始化**（估初始姿态/bias）或类似 vision-inertial 对齐；OpenVINS 提供动态初始化。VINS-Mono 这套「松耦合给初值」是优化主线的代表，思想可移植到滤波主线初始化。

---

## §9 优化主线：VINS-Mono 滑窗紧耦合 + 边缘化（对比用）[源 [Qin18] §IV、§VI]

> 本章须把「基于优化(VINS 滑窗+边缘化)」与「基于滤波(MSCKF)」**两主线完整推导并对比**。本节抽取优化主线核心：IMU 预积分、滑窗 BA 残差、边缘化（Schur 补）。

### §9.1 IMU 预积分（[Qin18] §IV-B）

预积分量（以 $b_k$ 为参考系，仅由 IMU 量测得，与 $b_k,b_{k+1}$ 其它态无关；式 6）：
$$
\boldsymbol\alpha_{b_{k+1}}^{b_k}=\iint_{t\in[t_k,t_{k+1}]}\mathbf R_t^{b_k}(\hat{\mathbf a}_t-\mathbf b_{a_t}-\mathbf n_a)\,dt^2,\quad
\boldsymbol\beta_{b_{k+1}}^{b_k}=\int_{t\in[t_k,t_{k+1}]}\mathbf R_t^{b_k}(\hat{\mathbf a}_t-\mathbf b_{a_t}-\mathbf n_a)\,dt,
$$
$$
\boldsymbol\gamma_{b_{k+1}}^{b_k}=\int_{t\in[t_k,t_{k+1}]}\tfrac12\boldsymbol\Omega(\hat{\boldsymbol\omega}_t-\mathbf b_{w_t}-\mathbf n_w)\,\boldsymbol\gamma_t^{b_k}\,dt.
$$
PVQ 传播关系（式 5）：
$$
\mathbf R_w^{b_k}\mathbf p_{b_{k+1}}^w=\mathbf R_w^{b_k}(\mathbf p_{b_k}^w+\mathbf v_{b_k}^w\Delta t_k-\tfrac12\mathbf g^w\Delta t_k^2)+\boldsymbol\alpha_{b_{k+1}}^{b_k},
$$
$$
\mathbf R_w^{b_k}\mathbf v_{b_{k+1}}^w=\mathbf R_w^{b_k}(\mathbf v_{b_k}^w-\mathbf g^w\Delta t_k)+\boldsymbol\beta_{b_{k+1}}^{b_k},\qquad \mathbf q_w^{b_k}\otimes\mathbf q_{b_{k+1}}^w=\boldsymbol\gamma_{b_{k+1}}^{b_k}.
$$

**连续误差动力学**（式 9，对 $\delta\mathbf z=[\delta\boldsymbol\alpha,\delta\boldsymbol\beta,\delta\boldsymbol\theta,\delta\mathbf b_a,\delta\mathbf b_w]$，四元数过参数化故误差用扰动 $\boldsymbol\gamma_t^{b_k}\approx\hat{\boldsymbol\gamma}_t^{b_k}\otimes[1,\tfrac12\delta\boldsymbol\theta_t^{b_k}]^\top$，式 8）：
$$
\dot{\delta\mathbf z}_t^{b_k}=\mathbf F_t\,\delta\mathbf z_t^{b_k}+\mathbf G_t\,\mathbf n_t,
$$
$$
\mathbf F_t=\begin{bmatrix}\mathbf 0 & \mathbf I & \mathbf 0 & \mathbf 0 & \mathbf 0\\ \mathbf 0 & \mathbf 0 & -\mathbf R_t^{b_k}\lfloor\hat{\mathbf a}_t-\mathbf b_{a_t}\rfloor_\times & -\mathbf R_t^{b_k} & \mathbf 0\\ \mathbf 0 & \mathbf 0 & -\lfloor\hat{\boldsymbol\omega}_t-\mathbf b_{w_t}\rfloor_\times & \mathbf 0 & -\mathbf I\\ \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\\ \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\end{bmatrix},\quad
\mathbf G_t=\begin{bmatrix}\mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf 0\\ -\mathbf R_t^{b_k} & \mathbf 0 & \mathbf 0 & \mathbf 0\\ \mathbf 0 & -\mathbf I & \mathbf 0 & \mathbf 0\\ \mathbf 0 & \mathbf 0 & \mathbf I & \mathbf 0\\ \mathbf 0 & \mathbf 0 & \mathbf 0 & \mathbf I\end{bmatrix}.
$$
**协方差递推**（一阶离散，初值 $\mathbf P_{b_k}^{b_k}=\mathbf 0$，式 10）：
$$
\mathbf P_{t+\delta t}^{b_k}=(\mathbf I+\mathbf F_t\delta t)\mathbf P_t^{b_k}(\mathbf I+\mathbf F_t\delta t)^\top+(\mathbf G_t\delta t)\mathbf Q(\mathbf G_t\delta t)^\top,\quad t\in[k,k+1],
$$
$\mathbf Q=\mathrm{diag}(\sigma_a^2,\sigma_w^2,\sigma_{b_a}^2,\sigma_{b_w}^2)$。**bias 一阶雅可比递推**（式 11）：$\mathbf J_{t+\delta t}=(\mathbf I+\mathbf F_t\delta t)\mathbf J_t$，初值 $\mathbf J_{b_k}=\mathbf I$。**bias 改变时一阶修正**（式 12，避免重传播）：
$$
\boldsymbol\alpha_{b_{k+1}}^{b_k}\approx\hat{\boldsymbol\alpha}_{b_{k+1}}^{b_k}+\mathbf J^\alpha_{b_a}\delta\mathbf b_a+\mathbf J^\alpha_{b_w}\delta\mathbf b_w,\quad
\boldsymbol\beta_{b_{k+1}}^{b_k}\approx\hat{\boldsymbol\beta}_{b_{k+1}}^{b_k}+\mathbf J^\beta_{b_a}\delta\mathbf b_a+\mathbf J^\beta_{b_w}\delta\mathbf b_w,\quad
\boldsymbol\gamma_{b_{k+1}}^{b_k}\approx\hat{\boldsymbol\gamma}_{b_{k+1}}^{b_k}\otimes\begin{bmatrix}1\\ \tfrac12\mathbf J^\gamma_{b_w}\delta\mathbf b_w\end{bmatrix}.
$$
**离散中点积分（实现用，式 7 给的是 Euler 版示意）**：
$$
\hat{\boldsymbol\alpha}_{i+1}^{b_k}=\hat{\boldsymbol\alpha}_i^{b_k}+\hat{\boldsymbol\beta}_i\delta t+\tfrac12\mathbf R(\hat{\boldsymbol\gamma}_i^{b_k})(\hat{\mathbf a}_i-\mathbf b_{a_i})\delta t^2,\quad
\hat{\boldsymbol\beta}_{i+1}^{b_k}=\hat{\boldsymbol\beta}_i^{b_k}+\mathbf R(\hat{\boldsymbol\gamma}_i^{b_k})(\hat{\mathbf a}_i-\mathbf b_{a_i})\delta t,\quad
\hat{\boldsymbol\gamma}_{i+1}^{b_k}=\hat{\boldsymbol\gamma}_i^{b_k}\otimes\begin{bmatrix}1\\ \tfrac12(\hat{\boldsymbol\omega}_i-\mathbf b_{w_i})\delta t\end{bmatrix}.
$$

### §9.2 滑窗紧耦合 BA 与残差（[Qin18] §VI-A/B/C）

**全状态**（式 21）：
$$
\mathcal X=[\mathbf x_0,\cdots,\mathbf x_n,\mathbf x_c^b,\lambda_0,\cdots,\lambda_m],\quad \mathbf x_k=[\mathbf p_{b_k}^w,\mathbf v_{b_k}^w,\mathbf q_{b_k}^w,\mathbf b_a,\mathbf b_g],\quad \mathbf x_c^b=[\mathbf p_c^b,\mathbf q_c^b],
$$
$\lambda_l$ 为第 $l$ 特征从首观测起的**逆深度**。**视觉惯性 BA（MAP）**（式 22）：
$$
\min_{\mathcal X}\Big\{\|\mathbf r_p-\mathbf H_p\mathcal X\|^2+\sum_{k\in\mathcal B}\|\mathbf r_{\mathcal B}(\hat{\mathbf z}_{b_{k+1}}^{b_k},\mathcal X)\|^2_{\mathbf P_{b_{k+1}}^{b_k}}+\sum_{(l,j)\in\mathcal C}\rho\big(\|\mathbf r_{\mathcal C}(\hat{\mathbf z}_l^{c_j},\mathcal X)\|^2_{\mathbf P_l^{c_j}}\big)\Big\},
$$
$\{\mathbf r_p,\mathbf H_p\}$ 为边缘化先验，$\rho$ 为 Huber 核（式 23：$\rho(s)=1$ 若 $s\ge1$，否则 $2\sqrt s-1$）。Ceres 求解。

**IMU 残差**（式 24）：
$$
\mathbf r_{\mathcal B}(\hat{\mathbf z}_{b_{k+1}}^{b_k},\mathcal X)=\begin{bmatrix}\delta\boldsymbol\alpha_{b_{k+1}}^{b_k}\\ \delta\boldsymbol\beta_{b_{k+1}}^{b_k}\\ \delta\boldsymbol\theta_{b_{k+1}}^{b_k}\\ \delta\mathbf b_a\\ \delta\mathbf b_g\end{bmatrix}=\begin{bmatrix}\mathbf R_w^{b_k}(\mathbf p_{b_{k+1}}^w-\mathbf p_{b_k}^w+\tfrac12\mathbf g^w\Delta t_k^2-\mathbf v_{b_k}^w\Delta t_k)-\hat{\boldsymbol\alpha}_{b_{k+1}}^{b_k}\\ \mathbf R_w^{b_k}(\mathbf v_{b_{k+1}}^w+\mathbf g^w\Delta t_k-\mathbf v_{b_k}^w)-\hat{\boldsymbol\beta}_{b_{k+1}}^{b_k}\\ 2\big[\mathbf q_{b_k}^{w\,-1}\otimes\mathbf q_{b_{k+1}}^w\otimes(\hat{\boldsymbol\gamma}_{b_{k+1}}^{b_k})^{-1}\big]_{xyz}\\ \mathbf b_{a_{b_{k+1}}}-\mathbf b_{a_{b_k}}\\ \mathbf b_{w_{b_{k+1}}}-\mathbf b_{w_{b_k}}\end{bmatrix},
$$
$[\cdot]_{xyz}$ 取四元数虚部作误差态表示。

**视觉残差（单位球面，式 25）**。第 $l$ 特征首见于第 $i$ 图、在第 $j$ 图的残差：
$$
\mathbf r_{\mathcal C}(\hat{\mathbf z}_l^{c_j},\mathcal X)=\begin{bmatrix}\mathbf b_1 & \mathbf b_2\end{bmatrix}^\top\cdot\Big(\hat{\bar{\mathbf P}}_l^{c_j}-\frac{\mathbf P_l^{c_j}}{\|\mathbf P_l^{c_j}\|}\Big),\quad \hat{\bar{\mathbf P}}_l^{c_j}=\pi_c^{-1}\!\Big(\begin{bmatrix}\hat u_l^{c_j}\\ \hat v_l^{c_j}\end{bmatrix}\Big),
$$
$$
\mathbf P_l^{c_j}=\mathbf R_b^c\Big(\mathbf R_w^{b_j}\big(\mathbf R_{b_i}^w(\mathbf R_c^b\tfrac{1}{\lambda_l}\pi_c^{-1}(\begin{bmatrix}u_l^{c_i}\\ v_l^{c_i}\end{bmatrix})+\mathbf p_c^b)+\mathbf p_{b_i}^w-\mathbf p_{b_j}^w\big)-\mathbf p_c^b\Big),
$$
$\pi_c^{-1}$ 为反投影（像素→单位向量），残差投影到 $\hat{\bar{\mathbf P}}_l^{c_j}$ 切平面（$\mathbf b_1,\mathbf b_2$ 为切平面正交基，自由度 2，由算法 1 求）。

### §9.3 边缘化（Schur 补）[源 [Qin18] §VI-D]

为限算复杂度，选择性边缘化滑窗中的 IMU 态 $\mathbf x_k$ 与特征 $\lambda_l$，把对应量测转为**先验**。**双向边缘化策略**（图 7）：
- 若**次新帧是关键帧**：保留它，**边缘化最旧帧**及其量测（转先验）；
- 若**次新帧非关键帧**：**丢弃其视觉量测，保留连到它的 IMU 量测**（不全删，以保系统稀疏性）。
关键帧选择确保窗内帧空间分离、保证三角化视差、最大化保留有大激励的加速度计量测。

**Schur 补构造先验**（[Qin18] §VI-D，引 [39]）。把边缘化变量记为 $\mathbf x_m$、保留变量记为 $\mathbf x_r$，相关 Hessian（信息阵）分块
$$
\begin{bmatrix}\boldsymbol\Lambda_{mm} & \boldsymbol\Lambda_{mr}\\ \boldsymbol\Lambda_{rm} & \boldsymbol\Lambda_{rr}\end{bmatrix}\begin{bmatrix}\delta\mathbf x_m\\ \delta\mathbf x_r\end{bmatrix}=\begin{bmatrix}\mathbf g_m\\ \mathbf g_r\end{bmatrix},
$$
消去 $\mathbf x_m$ 得保留变量的**先验信息阵与先验残差**（Schur 补）：
$$
\boldsymbol\Lambda_p=\boldsymbol\Lambda_{rr}-\boldsymbol\Lambda_{rm}\boldsymbol\Lambda_{mm}^{-1}\boldsymbol\Lambda_{mr},\qquad \mathbf g_p=\mathbf g_r-\boldsymbol\Lambda_{rm}\boldsymbol\Lambda_{mm}^{-1}\mathbf g_m,
$$
新先验叠加到既有先验上（即式 22 的 $\{\mathbf r_p,\mathbf H_p\}$，满足 $\boldsymbol\Lambda_p=\mathbf H_p^\top\mathbf H_p$）。
> **代价**：边缘化导致**线性化点提前固定**（early fix of linearization points），可能次优；VINS 认为 VIO 小漂移可接受。**这与 MSCKF 的 FEJ 问题同源**——固定线性化点既是边缘化的代价、又是保持一致性的手段（见 §10 对比）。

### §9.4 其它工程模块（简记）

- **Motion-only VI-BA**（§VI-E）：低算力设备只优化固定数目最新帧的位姿/速度（其余当常量），把状态估计提到相机率（~30 Hz）；代价函数同式 22。
- **IMU 前向传播**（§VI-F）：IMU 率远高于视觉，用最新 VIO 估计做 IMU 前向传播得 IMU 率输出。
- **失效检测与恢复**（§VI-G）：特征数过少 / 位姿/旋转大跳变 / bias 或外参大变 ⇒ 判失效，切回初始化。
- **重定位（§VII）+ 4-DOF 位姿图（§VIII）**：DBoW2 回环检测、紧耦合重定位、4-DOF（x,y,z,yaw）位姿图优化消漂移（yaw 与平移正是 §5 的 4 个不可观方向）。

---

## §10 两主线对比：MSCKF（滤波）vs VINS-Mono（优化）[综合 [MR07][Sun18][Li13][Qin18]]

> 本节为综合 agent 提供「两主线完整推导并对比」的对照骨架（结论性表格 + 关键对偶关系），所有条目均可回溯到上文各源。

### §10.1 总体对比表

| 维度 | MSCKF（基于滤波，[MR07][Sun18][Li13]）| VINS-Mono（基于优化，[Qin18]）|
|---|---|---|
| 估计框架 | EKF（递归一遍线性化）| 滑窗非线性最小二乘（迭代重线性化，Ceres）|
| 状态内容 | IMU 态 + 滑窗相机克隆（**不含特征**，§1）；可选含外参/SLAM 特征 | IMU 态 + 滑窗关键帧 + 相机外参 + **特征逆深度 $\lambda_l$**（§9.2）|
| 特征处理 | **零空间投影消去**（§4.4），复杂度对特征数线性 | 特征逆深度入态，参与 BA |
| IMU 处理 | 连续模型逐 IMU 传播 + 协方差 Lyapunov/转移阵（§2）| **预积分**（§9.1）合并区间内 IMU 为一个相对约束 |
| 信息利用 | 一次更新用一遍线性化（delayed linearization：多观测后再更新，§4 提升雅可比精度）| 迭代多遍线性化（每次优化重算雅可比，精度更高）|
| 旧态处理 | 删相机克隆（直接丢，因 §4.4 投影已用尽其约束信息）| **边缘化（Schur 补）**转先验（§9.3）|
| 一致性手段 | FEJ-EKF / **OC-EKF**（§5）保 4 维不可观 | 边缘化的固定线性化点（§9.3）；4-DOF 位姿图修漂移 |
| 计算成本 | 低（线性于特征数，$\max(O(r^2d),O(\xi^3))$，§6.3）；CPU 友好 | 高（迭代优化）；[Sun18] 实测 filter 比 optimization 省 CPU |
| 精度/鲁棒 | 高效、长航时一致（MSCKF 2.0 [Li13] 称优于固定滞后平滑器）| 高精度、回环后全局一致；初始化鲁棒（§8）|
| 典型实现 | [MR07] 原版、S-MSCKF [Sun18]、OpenVINS [OV]、MSCKF 2.0 [Li13] | VINS-Mono/VINS-Fusion [Qin18]、OKVIS |

### §10.2 关键对偶关系（综合时写「本质洞察」）

1. **零空间投影 ↔ Schur 补边缘化**：MSCKF 对特征做左零空间投影（§4.4）= 把特征从信息阵 Schur 补消去（§9.3）；二者在「i.i.d. 白高斯噪声 + 同线性化点」假设下**保留等量信息**（[OV]/Null-space-based marginalization 文献结论）。**这是两主线最深的连接点。**
2. **FEJ ↔ 固定线性化点**：滤波侧 FEJ 把雅可比钉在首次估计（§5.3）；优化侧边缘化把先验钉在边缘化时刻的线性化点（§9.3）。两者都是「为保一致性/稀疏性而牺牲在旧点重线性化」。
3. **不可观 4 维是共同物理约束**：MSCKF 的 $\mathcal N$（§5.2）与 VINS 的 4-DOF 位姿图（yaw + 平移，§9.4）面对的是同一组「VIO 测不出」的方向。
4. **delayed linearization（MSCKF）↔ 迭代优化（VINS）**：MSCKF 攒多观测后一次更新以改善雅可比精度，是优化「多遍重线性化」的廉价近似。

### §10.3 松/紧耦合（本章须覆盖）

- **松耦合（loosely-coupled）**：视觉与 IMU **分别**估计位姿/运动，再在外层融合（如 EKF 融合 vision-only VO 输出 + IMU）。优点简单、模块化；缺点丢失原始量测间的相关性、精度受限。[MR07] 相关工作中 [7][13][14] 的「pairwise 位移估计后融合」即松耦合思路；VINS-Mono **初始化阶段**（§8）用松耦合给初值。
- **紧耦合（tightly-coupled）**：把**原始视觉特征量测**与 IMU 量测放进**同一个估计器**联合优化/滤波。MSCKF（§4 特征量测直接进 EKF 残差）与 VINS-Mono（§9.2 视觉+IMU 残差同一 BA）**都是紧耦合**。优点：充分利用量测相关性、精度/鲁棒最好；缺点：实现复杂、计算重。本章两主线均属紧耦合。

---

## §X 本抽取覆盖范围与未尽事项（交综合 agent）

**已全量保真覆盖**：
- MSCKF 主线（[MR07] 式 1–38 逐式 + Algorithm 1）：IMU 态/误差态、连续模型、$F/G$、离散传播、状态增广 $J$、单目量测模型、**零空间投影（核心，式 23–24 + Givens + [OV] QR 写法）**、量测压缩 QR（式 25–28）、EKF 更新（式 29–31 Joseph）、逆深度三角化（式 32–38）。
- [Trawny05] 闭式离散传播（式 158–192）：$\boldsymbol\Theta,\boldsymbol\Psi$ 与 $\mathbf Q_d$ 全部闭式 + 小角极限。
- [Sun18] 全部：完整 $F,G$（附录 A）、修正增广 $\mathbf J_I$（附录 B）、完整双目量测雅可比（附录 C）、单帧零空间退化证明（附录 D）、OC-EKF（§III-C）。
- [OV] 7 页：相机量测全链式雅可比（畸变/投影/欧式/各特征表示）、零空间投影、量测压缩、FEJ 与 4 维 $\mathcal N$ 闭式、延迟特征初始化。
- 初始化（[Qin18] §V 全部，式 14–20 + 算法 1）；优化主线（[Qin18] §IV-B 预积分式 5–12、§VI-A/B/C 残差式 21–25、§VI-D 边缘化 Schur 补）；两主线对比表 + 对偶关系。

**未尽 / 需他源补齐（建议综合时补）**：
1. **[Li13] / [Hesch12] 的 OC-MSCKF 闭式状态转移阵速度/位置块**与「修改雅可比使 $\mathcal O\mathcal N=0$」的具体投影公式：本抽取只给了 [OV] FEJ 表述与 [Sun18] OC 选择理由，未抽到 [Li13] 原文逐式（其 PDF 镜像需授权）。建议综合时补 [Li13] §可观性分析的修改雅可比闭式。
2. **[OV] analytical propagation 页**的速度/位置块解析积分式（本抽取给了块结构与关键非零块，未抽到该子页逐式）。
3. **ESKF 误差注入/重置雅可比**（§6.5 仅指路）：详见本项目 `kalman_eskf__sola_eskf.md`，综合时交叉引用即可，无需在 VIO 章重证。
4. **Givens 旋转算左零空间/QR 的逐步算法**：[MR07] 仅给复杂度，未给伪码；可补 Golub & Van Loan 标准 Givens QR。
5. **VINS-Mono §VII 重定位 / §VIII 4-DOF 位姿图的残差与雅可比逐式**：本抽取只给模块功能与 4-DOF 含义；若本章要展开回环，需补 §VIII 式（位姿图边残差）。
6. **数值实验数据**：[MR07] 城市 3.2 km 实验（终点误差 ~10 m，0.31% 行程；姿态 3σ<1°、速度 3σ<0.35 m/s）、[Sun18] EuRoC/快飞 17.5 m/s 实验、[Qin18] EuRoC 对比——本抽取记录了关键数值结论，未逐图抽取（图为主，正文已含核心数字）。
