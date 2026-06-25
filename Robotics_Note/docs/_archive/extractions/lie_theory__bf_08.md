# 抽取留痕：Barfoot SER (2nd ed.) — 位姿估计问题（服务「李群与李代数」章）

> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/Barfoot_SER2_md/08_Pose_Estimation_Problems/08_Pose_Estimation_Problems.md`（1945 行，全文读毕）
> **源章名**：Pose Estimation Problems（《State Estimation for Robotics》2nd ed.）
> **⚠️ 编号说明**：目录文件夹称其为「第 8 章」，但**正文内部一律用「9.x」编号**（小节 9.1–9.4，公式 9.1–9.271，例 9.1）。本留痕一律沿用源内部的 **9.x** 小节/公式号，方便回查。
> **服务的成书章节**：李群与李代数（`parts/P0_math/lie_theory.tex`）。
>
> ⚠️ 本文件是「抽取留痕」，非成书正文。目标是【全量保真】：每一步推导（中间代数不跳）、每一道例题/数值例、每一条定义/定理/命题 + 完整证明、每一张表/分类/算法/伪码，全部完整记录。公式一律 LaTeX 写全；保留所有公式编号。**宁长勿略**。
>
> **本留痕额外承担两项任务**（见任务交接）：
> 1. **清偿 `lie_theory.tex:561` 的 Sim(3) 闭式 punt**（`J_s`/`W` 矩阵）。Barfoot ch8 **不含 Sim(3)**，故该闭式取自权威外源（Ethan Eade `lie_groups.pdf` + Strasdat 2010/2012 博论 §A.5 + Sophus `Sim3::exp` 的 `calcW`/`calcWInv`），**复现级完整闭式**，见本文件 **§S（Sim(3) 闭式专题）**。
> 2. 复审 ch7 的 SO(3)/SE(3) 推导/例在 ch8 中被引用处是否对得上（ch8 大量引用 ch7 的 wedge、odot、Ad、J、Q 等工具，本留痕在用到处把被引身份/编号点明）。

---

## 0. 记号约定（本源 Barfoot vs 本书统一约定）—— 综合时必读

### 0.1 本源核心记号

| 记号 | 含义（本源 Barfoot） |
|---|---|
| $\mathbf{C}$ / $\mathbf{C}_{v_k i}$ | **旋转矩阵**（DCM），$\mathbf{C}_{v_k i}$ 把系 $i$ 坐标变到系 $v_k$：$\mathbf r_{v_k}=\mathbf C_{v_k i}\mathbf r_i$。**字母用 C 不用 R**。双下标 $v_k i$ = 「$v_k$ from $i$」。 |
| $\mathbf{T}$ / $\mathbf{T}_{v_k i}$ | $4\times4$ **变换矩阵**，$SE(3)$ 元素。本源默认 $\mathbf T_{v_k i}=\begin{bmatrix}\mathbf C_{v_ki}&-\mathbf C_{v_ki}\mathbf r_i^{v_ki}\\\mathbf 0^T&1\end{bmatrix}$。 |
| $\mathbf r_i^{v_k i}$ | 从 $I$（系 $i$ 原点）到 $V_k$（系 $v_k$ 原点）的平移向量，**在系 $i$ 中表达**。上标 $v_ki$ 读「$v_k$ relative to $i$」，下标 $i$ = 表达系。 |
| $(\cdot)^{\wedge}$ | 反对称 / 李代数提升算子。对 $3\times1$：$\boldsymbol\phi^\wedge$ 是 $3\times3$ 反对称阵；对 $6\times1$ $\boldsymbol\xi^\wedge$ 是 $4\times4$；对 $9\times1$ $\boldsymbol\xi^\wedge$ 是 $5\times5$（$SE_2(3)$）。 |
| $(\cdot)^{\vee}$ | $\wedge$ 的逆（取出向量）。 |
| $(\cdot)^{\curlywedge}$（源里写 $(\cdot)^{\lambda}$）| **小伴随 / ad 算子**。对 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]\in\mathbb R^6$：$\boldsymbol\xi^{\curlywedge}=\begin{bmatrix}\boldsymbol\phi^\wedge&\boldsymbol\rho^\wedge\\\mathbf0&\boldsymbol\phi^\wedge\end{bmatrix}$（$6\times6$）。**源 OCR 常把这个上标渲染成 $\lambda$，本留痕统一写 $\curlywedge$。** |
| $(\cdot)^{\odot}$ | 把 $4\times1$（齐次点）转成 $4\times6$：满足 $\boldsymbol\xi^\wedge\,\mathbf p=\mathbf p^\odot\,\boldsymbol\xi$。源 7（即 ch7）§8.1.8 定义。对 $SE_2(3)$ 扩成 $5\times9$。 |
| $\mathrm{Ad}(\mathbf T)$ / $\boldsymbol{\mathcal T}$ | $\mathbf T$ 的**伴随**（$6\times6$，$SE_2(3)$ 时 $9\times9$）。本源 $\boldsymbol{\mathcal T}=\mathrm{Ad}(\mathbf T)$。 |
| $\mathbf J$ / $\mathbf J(\boldsymbol\phi)$ | $SO(3)$ 的**左雅可比**（$3\times3$）。 |
| $\boldsymbol{\mathcal J}$ / $\boldsymbol{\mathcal J}(\boldsymbol\xi)$ | $SE(3)$（或 $SE_2(3)$）的**左雅可比**（$6\times6$ / $9\times9$）。 |
| $\mathbf Q(\boldsymbol\phi,\boldsymbol\rho)$ | $SE(3)$ 左雅可比的耦合块（源式 8.91a，即本书 `lie_theory.tex` 附录的 $\mathbf Q_l$）。 |
| $\mathbf q=[\boldsymbol\varepsilon;\eta]$ | 四元数，$4\times1$，**矢部 $\boldsymbol\varepsilon$ 在前、标部 $\eta$ 在后**；Hamilton 惯例。$\mathbf q^{+},\mathbf q^{\oplus}$ 为左/右乘 $4\times4$ 算子（源式 7.17/7.19）。 |
| $\boldsymbol\varpi=[\boldsymbol\nu;\boldsymbol\omega]$ | $6\times1$ **广义速度**（平移速度在前、角速度在后），在车体系表达。 |
| $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ | $6\times1$ se(3) 向量，**平移分量 $\boldsymbol\rho$ 在前**。$SE_2(3)$ 时 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\nu;\boldsymbol\phi]$（位置、速度、转动）。 |
| $\check{(\cdot)},\hat{(\cdot)},\bar{(\cdot)}$ | 先验/预测、后验/校正、标称（nominal/mean）。 |
| $\check{\mathbf P},\hat{\mathbf P},\mathbf Q_k,\mathbf R_{jk},\boldsymbol\Sigma_{k\ell}$ | 协方差：预测/后验估计协方差、过程噪声、测量噪声、位姿图边协方差。 |
| $\mathbf 1$ | 单位阵；$\mathbf 0$ 零（列/阵），维数靠上下文。 |
| $w_j$ | 点对的**标量**权重（关键：点云对齐闭式解依赖权重是标量而非矩阵）。 |
| $\mathbf 1_i$ | $3\times3$ 单位阵的第 $i$ 列。 |

### 0.2 本源约定 vs 本书统一约定（差异表）

| 维度 | 本源 Barfoot ch8 | 本书统一约定 | 转换提示 |
|---|---|---|---|
| 旋转矩阵字母 | $\mathbf C$（$\mathbf C_{v_ki}$） | $R\in SO(3)$ | $\mathbf C_{v_ki}\leftrightarrow R$；下标 $v_ki$=「$v_k\leftarrow i$」。 |
| 反对称算子 | $(\cdot)^\wedge$ | $(\cdot)^\wedge$ | 一致。 |
| se(3) 向量排序 | $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（**平移在前**） | $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ | **一致**。 |
| **扰动方向（默认）** | **左扰动** $\mathbf T=\exp(\delta\boldsymbol\xi^\wedge)\bar{\mathbf T}$、$\mathbf C=\exp(\boldsymbol\phi^\wedge)\mathbf C_{\rm op}$（扰动乘在**左**，落在「车体」侧因为用 $\mathbf T_{vi}$）。源 §9.2.4(EKF)、§9.1.3/§9.1.4(对齐)、§9.3(位姿图)**全用左扰动**。 | **右扰动为主** $\mathbf T=\bar{\mathbf T}\exp(\boldsymbol\epsilon^\wedge)$ | ⚠️**核心差异**：综合到本书须把左扰动改写为右扰动。Barfoot 自述（源 §9.2.4「Relationship to Invariant EKF」）：左/右皆可，区别仅在 Jacobian 是否依赖状态；换成右扰动只是改符号（见源式 9.266、9.269–9.271 的 $\boldsymbol\epsilon_r=-\boldsymbol\epsilon_\ell\Rightarrow\mathbf G_r=-\mathbf G_\ell$）。 |
| **唯一的右扰动例外** | §9.4（惯性导航 / IMU 预积分）**改用右扰动** $\mathbf T=\bar{\mathbf T}\exp(\delta\boldsymbol\xi^\wedge)$、$\mathbf T_j=\mathbf T_{{\rm op},j}\exp(\boldsymbol\epsilon_j^\wedge)$，因为那里用 $\mathbf T_{iv}$（全局帧存平移/速度），且扰动仍在车体侧。 | 右扰动 | §9.4 **天然与本书一致**——直接可用，无需翻号。 |
| 四元数排序/惯例 | $[\boldsymbol\varepsilon;\eta]$ 矢部在前；Hamilton | Hamilton $[\eta;\boldsymbol\varepsilon]$ | 惯例同（Hamilton），仅排序需重排。 |
| 广义速度排序 | $\boldsymbol\varpi=[\boldsymbol\nu;\boldsymbol\omega]$（平移在前） | 与 $\boldsymbol\xi$ 一致 | 一致。 |
| 协方差字母 | $\mathbf P,\mathbf Q,\mathbf R,\boldsymbol\Sigma$ | 同 | 一致。 |
| 扩展位姿群 | 用 $SE_2(3)$（$5\times5$，含速度）、记 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\nu;\boldsymbol\phi]$ | 本书 `lie_theory.tex` **目前未含** $SE_2(3)$ 节 | ⚠️**缺口**：见 §9.4.2，可补入本书作 SE(3) 推广。 |
| Sim(3) | **ch8 不含** | `lie_theory.tex` §sim3 现为 punt | 见本留痕 §S（外源补全）。 |

### 0.3 关键被引身份（ch8 反复用到、定义在 ch7 / 第 8 章，本留痕在用处点明）
- $\boldsymbol\epsilon^\wedge\mathbf p=\mathbf p^\odot\boldsymbol\epsilon$（$\odot$ 算子，源 §8.1.8）。
- $\exp((\boldsymbol{\mathcal T}\boldsymbol\xi)^\wedge)=\mathbf T\exp(\boldsymbol\xi^\wedge)\mathbf T^{-1}$（伴随穿越指数，源式 7.19 类比 / §8.1）。
- BCH 一阶近似 $\ln(\exp(\boldsymbol\xi_1^\wedge)\exp(\boldsymbol\xi_2^\wedge))^\vee\approx\boldsymbol\xi_1+\boldsymbol{\mathcal J}(\boldsymbol\xi_1)^{-1}\boldsymbol\xi_2$（源式 8.105，下文 §9.3 用）。
- $SE(3)$ 左雅可比 $\boldsymbol{\mathcal J}$ 及其耦合块 $\mathbf Q$（源式 8.91）。
- 投影到 $SO(3)$：$\mathbf C=(\mathbf W\mathbf W^T)^{-1/2}\mathbf W$（源 §8.2.1，下文 §9.1.3「简化解」即此）。

---

## 9.（章引言）位姿估计问题

本部分把第 I 部分的经典状态估计与第 II 部分的三维机理结合。本章先讲**点云对齐**（最小二乘对齐两组三维点）；再回到 EKF 与批量估计器，调整它们适配旋转/位姿变量，背景是**已知世界几何下的车辆定位**。下一章处理世界几何未知（SLAM）。

本章四大主题：
- §9.1 点云对齐（Wahba 问题，三种参数化）
- §9.2 点云跟踪（运动/观测模型 + EKF + 批量 MAP）
- §9.3 位姿图松弛（pose-graph relaxation）
- §9.4 惯性导航（$SE_2(3)$ 扩展位姿、IMU 预积分）

---

## 9.1 点云对齐（Point-Cloud Alignment）

经典结果：对齐两组三维点（点云），最小化最小二乘代价。**前提**：每项权重必须是**标量**而非矩阵（即「ordinary least squares」，源脚注 1：源自航天姿态确定的著名 **Wahba 问题**，Wahba 1965）。

应用：ICP（Besl & McKay 1992）；RANSAC（Fischler & Bolles 1981，§5.4.1）里的快速位姿求解。

三种参数化（**均不近似/不线性化，全靠权重是标量**）：
1. **单位四元数** → 解一个**特征值问题**（Davenport 1965；Horn 1987b）。
2. **旋转矩阵** → 转成 **SVD**（Green 1952；Wahba 1965；Horn 1987a；Arun 1987；Umeyama 1991 考虑 $\det\mathbf C=1$；本源沿用 de Ruiter & Forbes 2013，覆盖所有可唯一确定 $\mathbf C$ 的情形）。
3. **变换矩阵**（迭代）→ 只需解**线性方程组**。

### 9.1.1 问题设置（源 §9.1.1）

两参考系：不动系 $\underline{\mathcal F}_i$，动系（车）$\underline{\mathcal F}_{v_k}$。有 $M$ 个测量 $\mathbf r_{v_k}^{p_jv_k}$（$j=1\dots M$），即点 $P_j$ 在车体系的位置（可能含噪）。已知 $\mathbf r_i^{p_ji}$（点 $P_j$ 在不动系的位置）。求最佳对齐两点云的平移+旋转（=动系相对不动系的位姿）。本问题只在**单一时刻 $k$** 对齐。

### 9.1.2 单位四元数解（源 §9.1.2）

> 用四元数比旋转矩阵简单，因为有效旋转的约束（单位长）更易处理。沿用 ch7 §7.2.3 四元数记号。

**齐次点（$4\times1$）**：
$$\mathbf{y}_{j} = \begin{bmatrix} \mathbf{r}_{v_{k}}^{p_{j}v_{k}} \\ 1 \end{bmatrix}, \quad \mathbf{p}_{j} = \begin{bmatrix} \mathbf{r}_{i}^{p_{j}i} \\ 1 \end{bmatrix}. \tag{9.1}$$
（除点索引 $j$ 外丢掉所有上下标。）

求平移 $\mathbf r$、旋转 $\mathbf q$。无噪几何关系：
$$\underbrace{\begin{bmatrix} \mathbf{r}_{v_k}^{p_j v_k} \\ 1 \end{bmatrix}}_{\mathbf{y}_j} = \underbrace{\begin{bmatrix} \mathbf{C}_{v_k i} & \mathbf{0} \\ \mathbf{0}^T & 1 \end{bmatrix}}_{\mathbf{q}^{-1+} \mathbf{q}^{\oplus}} \left( \begin{bmatrix} \mathbf{r}_i^{p_j i} \\ 1 \end{bmatrix} - \underbrace{\begin{bmatrix} \mathbf{r}_i^{v_k i} \\ 0 \end{bmatrix}}_{\mathbf{r}} \right). \tag{9.2}$$

用恒等式（ch7 式 7.19）$\begin{bmatrix}\mathbf C&\mathbf0\\\mathbf0^T&1\end{bmatrix}=\mathbf q^{-1+}\mathbf q^{\oplus}$，改写为
$$\mathbf{y}_{j} = \mathbf{q}^{-1+} (\mathbf{p}_{j} - \mathbf{r})^{+} \mathbf{q}. \tag{9.3}$$

**误差四元数**：
$$\mathbf{e}_j = \mathbf{y}_j - \mathbf{q}^{-1+} (\mathbf{p}_j - \mathbf{r})^+ \mathbf{q}. \tag{9.4}$$
左乘 $\mathbf q^{+}$ 得**对 $\mathbf q$ 线性**的误差：
$$\mathbf{e}_{j}' = \mathbf{q}^{+} \mathbf{e}_{j} = \left( \mathbf{y}_{j}^{\oplus} - \left( \mathbf{p}_{j} - \mathbf{r} \right)^{+} \right) \mathbf{q}. \tag{9.5}$$

**目标函数**（带 Lagrange 乘子保单位长）：
$$J(\mathbf{q}, \mathbf{r}, \lambda) = \frac{1}{2} \sum_{j=1}^{M} w_j \mathbf{e}_j^{\prime T} \mathbf{e}_j^{\prime} - \underbrace{\frac{1}{2} \lambda \left( \mathbf{q}^T \mathbf{q} - 1 \right)}_{\text{Lagrange}}. \tag{9.6}$$

用 $\mathbf e_j'$ 不影响目标（因 $\mathbf q^{+T}\mathbf q^{+}=(\mathbf q^{-1}\mathbf q)^{+}=\mathbf1$）：
$$\mathbf{e}_{j}^{\prime T}\mathbf{e}_{j}^{\prime} = \left(\mathbf{q}^{+}\mathbf{e}_{j}\right)^{T} \left(\mathbf{q}^{+}\mathbf{e}_{j}\right) = \mathbf{e}_{j}^{T}\mathbf{q}^{+T}\mathbf{q}^{+}\mathbf{e}_{j} = \mathbf{e}_{j}^{T} \left(\mathbf{q}^{-1}\mathbf{q}\right)^{+}\mathbf{e}_{j} = \mathbf{e}_{j}^{T}\mathbf{e}_{j}. \tag{9.7}$$

代入 $\mathbf e_j'$：
$$J(\mathbf{q}, \mathbf{r}, \lambda) = \frac{1}{2} \sum_{j=1}^{M} w_j \mathbf{q}^T \left( \mathbf{y}_j^{\oplus} - (\mathbf{p}_j - \mathbf{r})^+ \right)^{T} \left( \mathbf{y}_j^{\oplus} - (\mathbf{p}_j - \mathbf{r})^+ \right) \mathbf{q} - \frac{1}{2} \lambda \left( \mathbf{q}^T \mathbf{q} - 1 \right). \tag{9.8}$$

**对 $\mathbf q,\mathbf r,\lambda$ 求导**：
$$\frac{\partial J}{\partial \mathbf{q}^{T}} = \sum_{j=1}^{M} w_{j} \left( \mathbf{y}_{j}^{\oplus} - \left( \mathbf{p}_{j} - \mathbf{r} \right)^{+} \right)^{T} \left( \mathbf{y}_{j}^{\oplus} - \left( \mathbf{p}_{j} - \mathbf{r} \right)^{+} \right) \mathbf{q} - \lambda \mathbf{q}, \tag{9.9a}$$
$$\frac{\partial J}{\partial \mathbf{r}^{T}} = \mathbf{q}^{-1 \oplus} \sum_{j=1}^{M} w_{j} \left( \mathbf{y}_{j}^{\oplus} - \left( \mathbf{p}_{j} - \mathbf{r} \right)^{+} \right) \mathbf{q}, \tag{9.9b}$$
$$\frac{\partial J}{\partial \lambda} = -\frac{1}{2} \left( \mathbf{q}^T \mathbf{q} - 1 \right). \tag{9.9c}$$

**(9.9b)=0** 给出最优平移（=两点云质心之差，在不动系）：
$$\mathbf{r} = \mathbf{p} - \mathbf{q}^{+} \mathbf{y}^{+} \mathbf{q}^{-1}. \tag{9.10}$$

代回 (9.9a) 并令零，得**特征值问题**：
$$\mathbf{W}\mathbf{q} = \lambda \mathbf{q}, \tag{9.11}$$
其中
$$\mathbf{W} = \frac{1}{w} \sum_{j=1}^{M} w_j \left( (\mathbf{y}_j - \mathbf{y})^{\oplus} - (\mathbf{p}_j - \mathbf{p})^{+} \right)^{T} \left( (\mathbf{y}_j - \mathbf{y})^{\oplus} - (\mathbf{p}_j - \mathbf{p})^{+} \right), \tag{9.12a}$$
$$\mathbf{y} = \frac{1}{w} \sum_{j=1}^{M} w_j \mathbf{y}_j, \quad \mathbf{p} = \frac{1}{w} \sum_{j=1}^{M} w_j \mathbf{p}_j, \quad w = \sum_{j=1}^{M} w_j. \tag{9.12b}$$

> **源脚注 4（特征值问题定义）**：$N\times N$ 矩阵 $\mathbf A$ 的特征问题 $\mathbf A\mathbf x=\lambda\mathbf x$；$N$ 个（未必相异）特征值 $\lambda_i$ 由 $\det(\mathbf A-\lambda\mathbf1)=0$ 的根求得，再代回求各特征向量 $\mathbf x_i$（到一乘性常数）。重根情形需高级线代。

$\mathbf W$ **对称且半正定**（特征值 $\ge0$）。**取最小特征值**对应的特征向量得 $\mathbf q$（到乘性常数），单位长约束 $\mathbf q^T\mathbf q=1$ 定唯一。

**为何取最小特征值**：把 (9.9a)=0 改写出等价的
$$\mathbf{W} = \sum_{j=1}^{M} w_j \left( \mathbf{y}_j^{\oplus} - \left( \mathbf{p}_j - \mathbf{r} \right)^+ \right)^{T} \left( \mathbf{y}_j^{\oplus} - \left( \mathbf{p}_j - \mathbf{r} \right)^+ \right). \tag{9.13}$$
代入目标 (9.8)：
$$J(\mathbf{q}, \mathbf{r}, \lambda) = \frac{1}{2} \mathbf{q}^T \underbrace{\mathbf{W} \mathbf{q}}_{\lambda \mathbf{q}} - \frac{1}{2} \lambda \left( \mathbf{q}^T \mathbf{q} - 1 \right) = \frac{1}{2} \lambda. \tag{9.14}$$
故取最小 $\lambda$ 最小化目标。

**复杂情形**：若 $\mathbf W$ 奇异或最小特征值非相异，则解可能不唯一（需 Jordan 标准型等）；四元数法对此略过，留待旋转矩阵法详论。

> **注**：本技术无近似/线性化，但重度依赖**权重为标量**。

**重构结果**：
$$\begin{bmatrix} \hat{\mathbf{C}}_{v_k i} & \mathbf{0} \\ \mathbf{0}^T & 1 \end{bmatrix} = \mathbf{q}^{-1+} \mathbf{q}^{\oplus}, \qquad \begin{bmatrix} \hat{\mathbf{r}}_i^{v_k i} \\ 0 \end{bmatrix} = \mathbf{r}, \tag{9.15}$$
$$\hat{\mathbf{T}}_{v_k i} = \begin{bmatrix} \hat{\mathbf{C}}_{v_k i} & -\hat{\mathbf{C}}_{v_k i} \hat{\mathbf{r}}_i^{v_k i} \\ \mathbf{0}^T & 1 \end{bmatrix}, \tag{9.16}$$
$$\hat{\mathbf{T}}_{iv_k} = \hat{\mathbf{T}}_{v_k i}^{-1} = \begin{bmatrix} \hat{\mathbf{C}}_{iv_k} & \hat{\mathbf{r}}_i^{v_k i} \\ \mathbf{0}^T & 1 \end{bmatrix}. \tag{9.17}$$

### 9.1.3 旋转矩阵解（源 §9.1.3）

> 无平移时称 **Wahba 问题**；有平移称 **extended Wahba 问题**。沿用 de Ruiter & Forbes (2013)，覆盖所有可唯一确定 $\mathbf C$ 的情形，并指出无唯一全局解时有多少全局/局部解。

**简化记号**：
$$\mathbf{y}_j = \mathbf{r}_{v_k}^{p_j v_k}, \quad \mathbf{p}_j = \mathbf{r}_i^{p_j i}, \quad \mathbf{r} = \mathbf{r}_i^{v_k i}, \quad \mathbf{C} = \mathbf{C}_{v_k i}. \tag{9.18}$$
$$\mathbf{y} = \frac{1}{w} \sum_{j=1}^{M} w_j \mathbf{y}_j, \quad \mathbf{p} = \frac{1}{w} \sum_{j=1}^{M} w_j \mathbf{p}_j, \quad w = \sum_{j=1}^{M} w_j. \tag{9.19}$$

（注意：与四元数节相比，部分符号现为 $3\times1$ 而非 $4\times1$。）每点误差：
$$\mathbf{e}_j = \mathbf{y}_j - \mathbf{C}(\mathbf{p}_j - \mathbf{r}). \tag{9.20}$$

**代价**（约束 $\mathbf C\in SO(3)$，即 $\mathbf C\mathbf C^T=\mathbf1$ 且 $\det\mathbf C=1$）：
$$J(\mathbf{C}, \mathbf{r}) = \frac{1}{2} \sum_{j=1}^{M} w_j \mathbf{e}_j^T \mathbf{e}_j = \frac{1}{2} \sum_{j=1}^{M} w_j \left( \mathbf{y}_j - \mathbf{C}(\mathbf{p}_j - \mathbf{r}) \right)^T \left( \mathbf{y}_j - \mathbf{C}(\mathbf{p}_j - \mathbf{r}) \right). \tag{9.21}$$

**平移换元**：
$$\mathbf{d} = \mathbf{r} + \mathbf{C}^T \mathbf{y} - \mathbf{p}, \tag{9.22}$$
（源式 9.22 OCR 写 $\mathbf C^T\mathbf v$，应为 $\mathbf C^T\mathbf y$。）代价拆为只依赖 $\mathbf C$ 与只依赖 $\mathbf d$ 两半正定项：
$$J(\mathbf{C}, \mathbf{d}) = \underbrace{\frac{1}{2} \sum_{j=1}^{M} w_j \left( (\mathbf{y}_j - \mathbf{y}) - \mathbf{C}(\mathbf{p}_j - \mathbf{p}) \right)^T \left( (\mathbf{y}_j - \mathbf{y}) - \mathbf{C}(\mathbf{p}_j - \mathbf{p}) \right)}_{\text{只依赖 }\mathbf C} + \underbrace{\frac{1}{2} \mathbf{d}^T \mathbf{d}}_{\text{只依赖 }\mathbf d}. \tag{9.23}$$
取 $\mathbf d=\mathbf0$ 平凡最小化第二项 $\Rightarrow$
$$\mathbf{r} = \mathbf{p} - \mathbf{C}^T \mathbf{y}. \tag{9.24}$$
（即质心差，在不动系。）

**只剩对 $\mathbf C$ 最小化第一项**。展开各小项只有一项依赖 $\mathbf C$：
$$\left((\mathbf{y}_{j} - \mathbf{y}) - \mathbf{C}(\mathbf{p}_{j} - \mathbf{p})\right)^{T}\left(\cdots\right) = \underbrace{(\mathbf y_j-\mathbf y)^T(\mathbf y_j-\mathbf y)}_{\text{无关}} - 2 \underbrace{(\mathbf{y}_{j} - \mathbf{y})^{T} \mathbf{C}(\mathbf{p}_{j} - \mathbf{p})}_{\operatorname{tr}(\mathbf{C}(\mathbf{p}_{j} - \mathbf{p})(\mathbf{y}_{j} - \mathbf{y})^{T})} + \underbrace{(\mathbf{p}_{j} - \mathbf{p})^{T}(\mathbf{p}_{j} - \mathbf{p})}_{\text{无关}}. \tag{9.25}$$

中间项加权求和：
$$\frac{1}{w} \sum_{j=1}^{M} w_j (\mathbf{y}_j - \mathbf{y})^T \mathbf{C} (\mathbf{p}_j - \mathbf{p}) = \frac{1}{w} \sum_j w_j \operatorname{tr}\!\left( \mathbf{C} (\mathbf{p}_j - \mathbf{p}) (\mathbf{y}_j - \mathbf{y})^T \right) = \operatorname{tr}\!\left( \mathbf{C} \mathbf{W}^T \right), \tag{9.26}$$
其中
$$\mathbf{W} = \frac{1}{w} \sum_{j=1}^{M} w_j (\mathbf{y}_j - \mathbf{y}) (\mathbf{p}_j - \mathbf{p})^T. \tag{9.27}$$
（注意：此 $\mathbf W$ 与四元数节的 $\mathbf W$ **不同**；这里像「惯量矩阵」捕捉点散布。）

新代价（带 Lagrange 乘子 $\boldsymbol\Lambda$ 对称、$\gamma$）：
$$J(\mathbf{C}, \mathbf{\Lambda}, \gamma) = -\text{tr}(\mathbf{C}\mathbf{W}^T) + \underbrace{\text{tr}\left(\mathbf{\Lambda}(\mathbf{C}\mathbf{C}^T - \mathbf{1})\right) + \gamma(\det \mathbf{C} - 1)}_{\text{Lagrange}}. \tag{9.28}$$
（$\boldsymbol\Lambda$ 对称，因只需 6 个正交约束。）

**求导**（源脚注 5 用到：$\frac{\partial}{\partial\mathbf A}\det\mathbf A=\det(\mathbf A)\mathbf A^{-T}$，$\frac{\partial}{\partial\mathbf A}\operatorname{tr}(\mathbf A\mathbf B^T)=\mathbf B$，$\frac{\partial}{\partial\mathbf A}\operatorname{tr}(\mathbf B\mathbf A\mathbf A^T)=(\mathbf B+\mathbf B^T)\mathbf A$）：
$$\frac{\partial J}{\partial \mathbf{C}} = -\mathbf{W} + 2\mathbf{\Lambda}\mathbf{C} + \gamma \underbrace{\det \mathbf{C}}_{1} \underbrace{\mathbf{C}^{-T}}_{\mathbf C} = -\mathbf{W} + \mathbf{L}\mathbf{C}, \tag{9.29a}$$
$$\frac{\partial J}{\partial \mathbf{\Lambda}} = \mathbf{C}\mathbf{C}^T - \mathbf{1}, \tag{9.29b}$$
$$\frac{\partial J}{\partial \gamma} = \det \mathbf{C} - 1, \tag{9.29c}$$
其中 $\mathbf L=2\boldsymbol\Lambda+\gamma\mathbf1$。令 (9.29a)=0：
$$\mathbf{L}\mathbf{C} = \mathbf{W}. \tag{9.30}$$

#### 用李群工具无需 Lagrange 乘子也能得 (9.30)（源关键 Lie 推导）

取扰动 $\mathbf{C}' = \exp(\boldsymbol{\phi}^{\wedge})\mathbf{C}$（左扰动），对 $\boldsymbol\phi$ 第 $i$ 分量求导并令零：
$$\frac{\partial J}{\partial \phi_{i}} = \lim_{h \to 0} \frac{J(\mathbf{C}') - J(\mathbf{C})}{h} = \lim_{h \to 0} \frac{-\operatorname{tr}(\exp(h\mathbf{1}_{i}^{\wedge})\mathbf{C}\mathbf{W}^{T}) + \operatorname{tr}(\mathbf{C}\mathbf{W}^{T})}{h}$$
$$\approx \lim_{h \to 0} \frac{-\operatorname{tr}((\mathbf{1} + h\mathbf{1}_{i}^{\wedge})\mathbf{C}\mathbf{W}^{T}) + \operatorname{tr}(\mathbf{C}\mathbf{W}^{T})}{h} = -\operatorname{tr}(\mathbf{1}_{i}^{\wedge}\mathbf{C}\mathbf{W}^{T}). \tag{9.32}$$
令零：$(\forall i)\ \operatorname{tr}(\mathbf 1_i^\wedge\,\mathbf C\mathbf W^T)=0$ (9.33)。由 $\wedge$ 的反对称性，临界点处 $\mathbf L=\mathbf C\mathbf W^T$ **对称**；转置右乘 $\mathbf C$ 回到 (9.30)。

#### 简化解（Simplified Explanation）

若 $\det\mathbf W>0$：$\mathbf L\underbrace{\mathbf C\mathbf C^T}_{1}\mathbf L^T=\mathbf W\mathbf W^T$ (9.34)，$\mathbf L$ 对称故
$$\mathbf{L} = (\mathbf{W}\mathbf{W}^T)^{\frac{1}{2}}, \tag{9.35}$$
$$\mathbf{C} = (\mathbf{W}\mathbf{W}^T)^{-\frac{1}{2}}\mathbf{W}. \tag{9.36}$$
（与 §8.2.1 投影到 $SO(3)$ 同形。点多且非共面时通常好用；但难例需详解，如 RANSAC 中三点对齐。）

#### 详细解（Detailed Explanation，de Ruiter & Forbes 2013）

对实方阵 $\mathbf W$ 做 SVD：
$$\mathbf{W} = \mathbf{U}\mathbf{D}\mathbf{V}^T, \quad \mathbf{D}=\operatorname{diag}(d_1,d_2,d_3),\ d_1\ge d_2\ge d_3\ge0, \tag{9.37}$$
$\mathbf U,\mathbf V$ 方正交。代入 (9.30)：
$$\mathbf{L}^{2} = \mathbf{L}\mathbf{C}\mathbf{C}^{T}\mathbf{L}^{T} = \mathbf{W}\mathbf{W}^{T} = \mathbf{U}\mathbf{D}\underbrace{\mathbf{V}^{T}\mathbf{V}}_{1}\mathbf{D}^{T}\mathbf{U}^{T} = \mathbf{U}\mathbf{D}^{2}\mathbf{U}^{T}. \tag{9.38}$$
矩阵平方根：$\mathbf L=\mathbf U\mathbf M\mathbf U^T$ (9.39)，$\mathbf M^2=\mathbf D^2$ (9.40)。de Ruiter & Forbes 证：每个满足此的实对称 $\mathbf M$ 可写
$$\mathbf{M} = \mathbf{Y}\mathbf{D}\mathbf{S}\mathbf{Y}^T, \tag{9.41}$$
$\mathbf S=\operatorname{diag}(s_1,s_2,s_3)$，$s_i=\pm1$，$\mathbf Y$ 正交。**注意 $\mathbf Y$ 结构随重奇异值变复杂**（不能随便取）。例如 $d_1=d_2$ 时：
$$\mathbf{M} = \begin{bmatrix} d_{1} \cos \theta & d_{1} \sin \theta & 0 \\ d_{1} \sin \theta & -d_{1} \cos \theta & 0 \\ 0 & 0 & d_{3} \end{bmatrix} = \underbrace{\begin{bmatrix} \cos \frac{\theta}{2} & -\sin \frac{\theta}{2} & 0 \\ \sin \frac{\theta}{2} & \cos \frac{\theta}{2} & 0 \\ 0 & 0 & 1 \end{bmatrix}}_{\mathbf Y} \underbrace{\begin{bmatrix} d_{1} & 0 & 0 \\ 0 & -d_{1} & 0 \\ 0 & 0 & d_{3} \end{bmatrix}}_{\mathbf{DS}} \mathbf Y^T, \tag{9.42}$$
$\theta$ 任意。且总有
$$\mathbf{D} = \mathbf{Y}\mathbf{D}\mathbf{Y}^T \tag{9.43}$$
（$\mathbf Y$ 块结构与 $\mathbf D$ 奇异值重数对应）。

**目标函数化简**：
$$J = -\operatorname{tr}(\mathbf{C}\mathbf{W}^{T}) = -\operatorname{tr}(\mathbf L) = -\operatorname{tr}(\mathbf{U}\mathbf{Y}\mathbf{D}\mathbf{S}\mathbf{Y}^{T}\mathbf{U}^{T}) = -\operatorname{tr}(\mathbf{D}\mathbf{S}) = -(d_{1}s_{1} + d_{2}s_{2} + d_{3}s_{3}). \tag{9.44}$$

**Case (i): $\det\mathbf W\ne0$**（所有奇异值正）。
$$\det \mathbf{W} = \det \mathbf{L} = \underbrace{\det \mathbf{D}}_{>0} \det \mathbf{S}. \tag{9.45}$$
$$\det \mathbf{S} = \operatorname{sgn}(\det\mathbf W) = \det\mathbf U\det\mathbf V = \pm1. \tag{9.46}$$
（$\det\mathbf U=\pm1$ 因 $(\det\mathbf U)^2=\det(\mathbf U^T\mathbf U)=1$，$\mathbf V$ 同。）四个子情形：

- **(i-a) $\det\mathbf W>0$**：$\det\mathbf S=1$，唯一最小取 $s_1=s_2=s_3=1$。则
$$\mathbf{C} = \mathbf{L}^{-1}\mathbf{W} = \mathbf{U}\mathbf{Y}\mathbf{S}^{-1}\mathbf{D}^{-1}\mathbf{Y}^{T}\mathbf{D}\mathbf{V}^{T} = \mathbf{U}\mathbf{Y}\mathbf{S}\mathbf{Y}^{T}\mathbf{V}^{T} = \mathbf{U}\mathbf{S}\mathbf{V}^{T}, \tag{9.47}$$
$\mathbf S=\mathbf1$（即简化解）。
- **(i-b) $\det\mathbf W<0,\ d_1\ge d_2>d_3>0$**：$\det\mathbf S=-1$，恰一 $s_i$ 负；$d_3$ 相异故唯一取 $s_1=s_2=1,s_3=-1$。$\mathbf Y$ 为
$$\mathbf{Y} = \operatorname{diag}(\pm 1, \pm 1, \pm 1), \quad \text{或}\quad \mathbf{Y} = \begin{bmatrix} \pm \cos\frac{\theta}{2} & \mp \sin\frac{\theta}{2} & 0\\ \pm \sin\frac{\theta}{2} & \pm \cos\frac{\theta}{2} & 0\\ 0 & 0 & \pm 1 \end{bmatrix}, \tag{9.48}$$
因 $\mathbf Y\mathbf S\mathbf Y^T=\mathbf S$，得 $\mathbf C=\mathbf U\mathbf S\mathbf V^T$ (9.49)，$\mathbf S=\operatorname{diag}(1,1,-1)$。
- **(i-c) $\det\mathbf W<0,\ d_1>d_2=d_3>0$**：$\det\mathbf S=-1$；$d_2=d_3$ 故 $s_2=-1$ 或 $s_3=-1$ 同值。$\mathbf Y$:
$$\mathbf{Y} = \operatorname{diag}(\pm 1, \pm 1, \pm 1), \quad \text{或}\quad \mathbf{Y} = \begin{bmatrix} \pm 1 & 0 & 0 \\ 0 & \pm \cos\frac{\theta}{2} & \mp \sin\frac{\theta}{2} \\ 0 & \pm \sin\frac{\theta}{2} & \pm \cos\frac{\theta}{2} \end{bmatrix}, \tag{9.50}$$
$$\mathbf{C} = \mathbf{U}\mathbf{Y}\mathbf{S}\mathbf{Y}^T\mathbf{V}^T, \tag{9.51}$$
$\mathbf S=\operatorname{diag}(1,1,-1)$ 或 $\operatorname{diag}(1,-1,1)$。$\theta$ 任意 $\Rightarrow$ **无穷多解**。
- **(i-d) $\det\mathbf W<0,\ d_1=d_2=d_3>0$**：$s_1=-1$ 或 $s_2=-1$ 或 $s_3=-1$ 同值 $\Rightarrow$ **无穷多解**。

**Case (ii): $\det\mathbf W=0$**（按零奇异值个数）。
- **(ii-a) rank $\mathbf W=2$**（$d_1\ge d_2>d_3=0$）：唯一取 $s_1=s_2=1$，$s_3$ 自由。由 (9.30)：$(\mathbf U\mathbf Y\mathbf D\mathbf S\mathbf Y^T\mathbf U^T)\mathbf C=\mathbf U\mathbf D\mathbf V^T$ (9.52)，左乘 $\mathbf U^T$ 右乘 $\mathbf V$：
$$\mathbf{D} \underbrace{\mathbf{U}^T \mathbf{C} \mathbf{V}}_{\mathbf{Q}} = \mathbf{D} \tag{9.53}$$
（因 $\mathbf{DS}=\mathbf D$、$\mathbf Y\mathbf D\mathbf Y^T=\mathbf D$）。$\mathbf Q$ 正交，$\mathbf D=\operatorname{diag}(d_1,d_2,0)$ $\Rightarrow\mathbf Q=\operatorname{diag}(1,1,q_3)$，
$$q_3 = \det \mathbf{Q} = \det \mathbf{U}\det \mathbf{V} = \pm 1, \tag{9.54}$$
$$\mathbf{C} = \mathbf{U}\mathbf{S}\mathbf{V}^T,\quad \mathbf{S} = \operatorname{diag}(1, 1, \det \mathbf{U} \det \mathbf{V}). \tag{9.55}$$
- **(ii-b) rank $\mathbf W=1$**（$d_1>d_2=d_3=0$）：$s_1=1$，$s_2,s_3$ 自由。$\mathbf D\mathbf Q=\mathbf D$ (9.56)，$\mathbf D=\operatorname{diag}(d_1,0,0)$ $\Rightarrow\mathbf Q$ 为
$$\mathbf{Q} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & -\sin \theta \\ 0 & \sin \theta & \cos \theta \end{bmatrix} \quad \text{或} \quad \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & \sin \theta & -\cos \theta \end{bmatrix},\quad \theta\in\mathbb R, \tag{9.57}$$
**无穷多解**。$\det\mathbf Q=\det\mathbf U\det\mathbf V=\pm1$ (9.58)，$\mathbf C=\mathbf U\mathbf S\mathbf V^T$ (9.59)，
$$\mathbf{S} = \begin{cases} \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & -\sin \theta \\ 0 & \sin \theta & \cos \theta \end{bmatrix} & \det \mathbf{U} \det \mathbf{V} = 1 \\[12pt] \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & \sin \theta & -\cos \theta \end{bmatrix} & \det \mathbf{U} \det \mathbf{V} = -1 \end{cases} \tag{9.60}$$
物理：点共线（至少在一帧），绕该轴任意 $\theta$ 不改 $J$。
- **(ii-c) rank $\mathbf W=0$**：无点或点重合，任意 $\mathbf C\in SO(3)$ 同值。

**总结（唯一全局解判据）**：若唯一，则
$$\mathbf{C} = \mathbf{U}\mathbf{S}\mathbf{V}^T,\quad \mathbf{S} = \operatorname{diag}(1, 1, \det \mathbf{U} \det \mathbf{V}),\quad \mathbf{W} = \mathbf{U}\mathbf{D}\mathbf{V}^T. \tag{9.61}$$
**唯一全局解存在的充要条件**：(i) $\det\mathbf W>0$；或 (ii) $\det\mathbf W<0$ 且最小奇异值相异（$d_1\ge d_2>d_3>0$）；或 (iii) rank $\mathbf W=2$。否则无穷多解（病态，实际罕见）。

**最终估计**：$\hat{\mathbf C}_{v_ki}=\mathbf C$，
$$\hat{\mathbf{r}}_{i}^{v_{k}i} = \mathbf{p} - \hat{\mathbf{C}}_{v_{k}i}^{T} \mathbf{y}, \tag{9.62}$$
$$\hat{\mathbf{T}}_{v_k i} = \begin{bmatrix} \hat{\mathbf{C}}_{v_k i} & -\hat{\mathbf{C}}_{v_k i} \hat{\mathbf{r}}_i^{v_k i} \\ \mathbf{0}^T & 1 \end{bmatrix}, \tag{9.63} \qquad \hat{\mathbf{T}}_{iv_k} = \hat{\mathbf{T}}_{v_k i}^{-1} = \begin{bmatrix} \hat{\mathbf{C}}_{iv_k} & \hat{\mathbf{r}}_i^{v_k i} \\ \mathbf{0}^T & 1 \end{bmatrix}. \tag{9.64}$$

#### 例 9.1（源 Example 9.1，subcase (i-b) 数值例）

两点云各 6 点：
$$\mathbf p_1=3\mathbf 1_1,\ \mathbf p_2=2\mathbf 1_2,\ \mathbf p_3=\mathbf 1_3,\ \mathbf p_4=-3\mathbf 1_1,\ \mathbf p_5=-2\mathbf 1_2,\ \mathbf p_6=-\mathbf 1_3,$$
$$\mathbf y_1=-3\mathbf 1_1,\ \mathbf y_2=-2\mathbf 1_2,\ \mathbf y_3=-\mathbf 1_3,\ \mathbf y_4=3\mathbf 1_1,\ \mathbf y_5=2\mathbf 1_2,\ \mathbf y_6=\mathbf 1_3,$$
$\mathbf 1_i$ 是 $3\times3$ 单位阵第 $i$ 列。（第一云是长方体六面中心，每点配第二云对面的点。）
$$\mathbf{p} = \mathbf{0}, \quad \mathbf{y} = \mathbf{0}, \quad \mathbf{W} = \frac{1}{6} \operatorname{diag}(-18, -8, -2). \tag{9.65}$$
质心已重合，只需旋转。

**简化法**：
$$\mathbf{C} = (\mathbf{W}\mathbf{W}^T)^{-\frac{1}{2}}\mathbf{W} = \operatorname{diag}(-1, -1, -1). \tag{9.66}$$
但 $\det\mathbf C=-1\notin SO(3)$ —— **简化法失败**。

**严格法**：SVD
$$\mathbf{W} = \mathbf{U}\mathbf{D}\mathbf{V}^{T},\ \mathbf{U} = \operatorname{diag}(1, 1, 1),\ \mathbf{D} = \frac{1}{6}\operatorname{diag}(18, 8, 2),\ \mathbf{V} = \operatorname{diag}(-1, -1, -1). \tag{9.67}$$
$\det\mathbf W=-4/3<0$ 且最小奇异值相异 $\Rightarrow$ subcase (i-b)，$\mathbf S=\operatorname{diag}(1,1,-1)$：
$$\mathbf{C} = \operatorname{diag}(-1, -1, 1), \tag{9.68}$$
$\det\mathbf C=1$。这是绕 $\mathbf 1_3$ 轴转 $\pi$，使四点误差为零、两点非零，目标降至最小 $J=4$。（源脚注 7：想象六对点用橡皮筋连，找最小弹性势能的旋转。）

#### 测试局部极小（Testing for Local Minima）

(9.30) 是临界点条件，可能极小/极大/鞍点。取扰动 $\mathbf C'=\exp(\boldsymbol\phi^\wedge)\mathbf C$ (9.69)，$\boldsymbol\phi$ 任意方向但保持 $\mathbf C'\in SO(3)$。
$$\delta J = -\text{tr}\left((\mathbf{C}' - \mathbf{C})\mathbf{W}^T\right). \tag{9.70}$$
二阶近似：
$$\delta J \approx -\text{tr}(\boldsymbol{\phi}^{\wedge}\mathbf{C}\mathbf{W}^{T}) - \frac{1}{2}\text{tr}(\boldsymbol{\phi}^{\wedge}\boldsymbol{\phi}^{\wedge}\mathbf{C}\mathbf{W}^{T}). \tag{9.71}$$
代临界点 (9.30)：
$$\delta J = -\operatorname{tr}(\boldsymbol{\phi}^{\wedge} \mathbf{U} \mathbf{Y} \mathbf{D} \mathbf{S} \mathbf{Y}^{T} \mathbf{U}^{T}) - \frac{1}{2} \operatorname{tr}(\boldsymbol{\phi}^{\wedge} \boldsymbol{\phi}^{\wedge} \mathbf{U} \mathbf{Y} \mathbf{D} \mathbf{S} \mathbf{Y}^{T} \mathbf{U}^{T}). \tag{9.72}$$
第一项为零（临界点）：
$$\operatorname{tr}(\boldsymbol{\phi}^{\wedge}\mathbf{U}\mathbf{Y}\mathbf{D}\mathbf{S}\mathbf{Y}^{T}\mathbf{U}^{T}) = \operatorname{tr}((\mathbf{Y}^{T}\mathbf{U}^{T}\boldsymbol{\phi})^{\wedge}\mathbf{D}\mathbf{S}) = \operatorname{tr}(\boldsymbol{\varphi}^{\wedge}\mathbf{D}\mathbf{S}) = 0, \tag{9.73}$$
$$\boldsymbol{\varphi} = [\varphi_1;\varphi_2;\varphi_3]= \mathbf{Y}^T \mathbf{U}^T \boldsymbol{\phi} \tag{9.74}$$
（反对称阵对角为零）。第二项用 $\mathbf u^\wedge\mathbf u^\wedge=-\mathbf u^T\mathbf u\,\mathbf1+\mathbf u\mathbf u^T$：
$$\delta J = -\frac{1}{2} \operatorname{tr}\left( (-\boldsymbol{\varphi}^{2} \mathbf{1} + \boldsymbol{\varphi} \boldsymbol{\varphi}^{T}) \mathbf{D} \mathbf{S} \right),\quad \varphi^2=\varphi_1^2+\varphi_2^2+\varphi_3^2. \tag{9.75}$$
$$\delta J = \frac{1}{2} \varphi^2 \operatorname{tr}(\mathbf{DS}) - \frac{1}{2} \boldsymbol\varphi^T \mathbf{DS} \boldsymbol\varphi = \frac{1}{2} \left( \varphi_1^2 (d_2 s_2 + d_3 s_3) + \varphi_2^2 (d_1 s_1 + d_3 s_3) + \varphi_3^2 (d_1 s_1 + d_2 s_2) \right). \tag{9.76}$$
符号由 $\mathbf{DS}$ 决定。

**验证全局极小**：
- (i-a) $s_1=s_2=s_3$：$\delta J = \frac12(\varphi_1^2(d_2+d_3)+\varphi_2^2(d_1+d_3)+\varphi_3^2(d_1+d_2))>0$ (9.77)，确为极小。
- (i-b) $s_1=s_2=1,s_3=-1$：$\delta J=\frac12(\varphi_1^2\underbrace{(d_2-d_3)}_{>0}+\varphi_2^2\underbrace{(d_1-d_3)}_{>0}+\varphi_3^2(d_1+d_2))>0$ (9.78)，极小。
- (ii-a) $d_3=0,s_1=s_2=1,s_3=\pm1$：$\delta J=\frac12(\varphi_1^2 d_2+\varphi_2^2 d_1+\varphi_3^2(d_1+d_2))>0$ (9.79)，极小。

**有无其他局部极小**（关键于迭代法）：以 (i-a) 且 $d_1>d_2>d_3>0$ 为例：
- $s_1=s_2=-1,s_3=1$（$\det\mathbf S=1$）：$\delta J=\frac12(\varphi_1^2\underbrace{(d_3-d_2)}_{<0}+\varphi_2^2\underbrace{(d_3-d_1)}_{<0}+\varphi_3^2\underbrace{(-d_1-d_2)}_{<0})<0$ (9.80)，为**极大**。
- $\mathbf S=\operatorname{diag}(-1,1,-1)$ 与 $\operatorname{diag}(1,-1,-1)$：**鞍点**。
- 结论：无其他临界点 $\Rightarrow$ **除全局极小外无局部极小**。

(i-b) 同理：$\det\mathbf S=-1$，$\operatorname{diag}(-1,-1,-1)$ 极大，$\operatorname{diag}(-1,1,1)$、$\operatorname{diag}(1,-1,1)$ 鞍点；无额外局部极小。

(ii-a) 一般：
$$\delta J = \frac{1}{2} \left( \varphi_1^2 d_2 s_2 + \varphi_2^2 d_1 s_1 + \varphi_3^2 (d_1 s_1 + d_2 s_2) \right), \tag{9.81}$$
唯一造极小须 $s_1=s_2=1$（即全局极小）；无额外局部极小。

#### 迭代解（Iterative Approach，SO(3)-敏感、**无约束**）

> 优点：无约束，避开前两法的难处（源脚注 8：不需特征问题或 SVD）。局部有效（需初值），通常几次迭代收敛；由上节局部极小分析，凡有唯一全局极小处无额外局部极小可忧。

从消去平移的代价出发：
$$J(\mathbf{C}) = \frac{1}{2} \sum_{j=1}^{M} w_j \left( (\mathbf{y}_j - \mathbf{y}) - \mathbf{C}(\mathbf{p}_j - \mathbf{p}) \right)^T \left( (\mathbf{y}_j - \mathbf{y}) - \mathbf{C}(\mathbf{p}_j - \mathbf{p}) \right). \tag{9.82}$$
插入 SO(3)-敏感扰动（左扰动）：
$$\mathbf{C} = \exp(\boldsymbol{\psi}^{\wedge}) \mathbf{C}_{\text{op}} \approx (\mathbf{1} + \boldsymbol{\psi}^{\wedge}) \mathbf{C}_{\text{op}}, \tag{9.83}$$
代价变 $\boldsymbol\psi$ 的二次型，最优 $\boldsymbol\psi^*$ 满足：
$$\mathbf{C}_{\mathrm{op}} \underbrace{\left(-\frac{1}{w} \sum_{j=1}^{M} w_{j} (\mathbf{p}_{j} - \mathbf{p})^{\wedge} (\mathbf{p}_{j} - \mathbf{p})^{\wedge}\right)}_{\text{常量}} \mathbf{C}_{\mathrm{op}}^{T} \boldsymbol\psi^{*} = -\frac{1}{w} \sum_{j=1}^{M} w_{j} (\mathbf{y}_{j} - \mathbf{y})^{\wedge} \mathbf{C}_{\mathrm{op}} (\mathbf{p}_{j} - \mathbf{p}). \tag{9.84}$$

右端第 $i$ 行可化简（不需逐迭代用原始点）：
$$\mathbf{1}_{i}^{T} \left( -\frac{1}{w} \sum_j w_{j} (\mathbf{y}_{j} - \mathbf{y})^{\wedge} \mathbf{C}_{op} (\mathbf{p}_{j} - \mathbf{p}) \right) = \frac{1}{w} \sum_j w_{j} (\mathbf{y}_{j} - \mathbf{y})^{T} \mathbf{1}_{i}^{\wedge} \mathbf{C}_{op} (\mathbf{p}_{j} - \mathbf{p})$$
$$= \frac{1}{w} \sum_j w_{j} \operatorname{tr}\left( \mathbf{1}_{i}^{\wedge} \mathbf{C}_{op} (\mathbf{p}_{j} - \mathbf{p}) (\mathbf{y}_{j} - \mathbf{y})^{T} \right) = \operatorname{tr}\left( \mathbf{1}_{i}^{\wedge} \mathbf{C}_{op} \mathbf{W}^{T} \right), \tag{9.85}$$
$$\mathbf{W} = \frac{1}{w} \sum_{j=1}^{M} w_j (\mathbf{y}_j - \mathbf{y}) (\mathbf{p}_j - \mathbf{p})^T. \tag{9.86}$$
令
$$\mathbf{I} = -\frac{1}{w} \sum_{j=1}^{M} w_j (\mathbf{p}_j - \mathbf{p})^{\wedge} (\mathbf{p}_j - \mathbf{p})^{\wedge}, \tag{9.87a}\qquad \mathbf{b} = \left[ \operatorname{tr}\left( \mathbf{1}_{i}^{\wedge} \mathbf{C}_{\text{op}} \mathbf{W}^{T} \right) \right]_{i}, \tag{9.87b}$$
**闭式更新**：
$$\boldsymbol\psi^* = \mathbf{C}_{\text{op}} \mathbf{I}^{-1} \mathbf{C}_{\text{op}}^T \mathbf{b}, \tag{9.88}\qquad \mathbf{C}_{\mathrm{op}} \leftarrow \exp(\boldsymbol\psi^{*\wedge}) \mathbf{C}_{\mathrm{op}}, \tag{9.89}$$
迭代收敛取 $\hat{\mathbf C}_{v_ki}=\mathbf C_{\rm op}$，
$$\hat{\mathbf{r}}_i^{v_k i} = \mathbf{p} - \hat{\mathbf{C}}_{v_k i}^T \mathbf{y}. \tag{9.90}$$
（$\mathbf I,\mathbf W$ 可预先算好，执行时不需原始点。）

#### 需三个非共线点（Three Noncollinear Points Required）

唯一解 $\Leftrightarrow\det\mathbf I\ne0$。充分条件 $\mathbf I$ 正定：$\forall\mathbf x\ne0$，$\mathbf x^T\mathbf I\mathbf x>0$ (9.91)。
$$\mathbf{x}^{T}\mathbf{I}\,\mathbf{x} = \frac{1}{w}\sum_{j=1}^{M}w_{j}\underbrace{\left((\mathbf{p}_{j} - \mathbf{p})^{\wedge}\mathbf{x}\right)^{T}\left((\mathbf{p}_{j} - \mathbf{p})^{\wedge}\mathbf{x}\right)}_{\ge0} \geq 0. \tag{9.92}$$
为零须每项零：$(\forall j)\ (\mathbf p_j-\mathbf p)^\wedge\mathbf x=\mathbf0$ (9.93)，即 $\mathbf x=\mathbf0$（违设）、$\mathbf p_j=\mathbf p$、或 $\mathbf x\parallel\mathbf p_j-\mathbf p$。只要至少三点且非共线，后两者不成立。（注：三非共线点只是每迭代唯一解的充分条件，不决定全局解个数。）

### 9.1.4 变换矩阵解（源 §9.1.4，迭代，SE(3)）

> 用变换矩阵与指数映射（源脚注 9：用 §8.1.9 的优化法）。

**记号**（齐次点用不同字体 $\boldsymbol y_j,\boldsymbol p_j$）：
$$\boldsymbol{y}_{j} = \begin{bmatrix} \mathbf{y}_{j} \\ 1 \end{bmatrix}, \quad \boldsymbol{p}_{j} = \begin{bmatrix} \mathbf{p}_{j} \\ 1 \end{bmatrix}, \quad \mathbf{T} = \mathbf{T}_{v_{k}i} = \begin{bmatrix} \mathbf{C}_{v_{k}i} & -\mathbf{C}_{v_{k}i}\mathbf{r}_{i}^{v_{k}i} \\ \mathbf{0}^{T} & 1 \end{bmatrix}. \tag{9.94}$$
误差与目标：
$$\mathbf{e}_j = \boldsymbol{y}_j - \mathbf{T}\boldsymbol{p}_j, \tag{9.95}\qquad J(\mathbf{T}) = \frac{1}{2} \sum_{j=1}^{M} w_j (\boldsymbol{y}_j - \mathbf{T}\boldsymbol{p}_j)^T (\boldsymbol{y}_j - \mathbf{T}\boldsymbol{p}_j). \tag{9.96}$$
求 $\mathbf T\in SE(3)$。SE(3)-敏感扰动（左扰动）：
$$\mathbf{T} = \exp(\boldsymbol{\epsilon}^{\wedge}) \mathbf{T}_{op} \approx (\mathbf{1} + \boldsymbol{\epsilon}^{\wedge}) \mathbf{T}_{op}. \tag{9.97}$$
代入（$\boldsymbol z_j=\mathbf T_{\rm op}\boldsymbol p_j$，用 $\boldsymbol\epsilon^\wedge\boldsymbol z_j=\boldsymbol z_j^\odot\boldsymbol\epsilon$，ch7 §8.1.8）：
$$J(\mathbf{T}) \approx \frac{1}{2} \sum_{j=1}^{M} w_j \left( (\boldsymbol{y}_j - \boldsymbol{z}_j) - \boldsymbol{z}_j^{\odot} \boldsymbol{\epsilon} \right)^T \left( (\boldsymbol{y}_j - \boldsymbol{z}_j) - \boldsymbol{z}_j^{\odot} \boldsymbol{\epsilon} \right). \tag{9.98–9.99}$$
对 $\boldsymbol\epsilon$ 二次型，无约束。求导：
$$\frac{\partial J}{\partial \boldsymbol{\epsilon}^{T}} = -\sum_{j=1}^{M} w_{j} \boldsymbol{z}_{j}^{\odot T} \left( (\boldsymbol{y}_{j} - \boldsymbol{z}_{j}) - \boldsymbol{z}_{j}^{\odot} \boldsymbol{\epsilon} \right). \tag{9.100}$$
令零：
$$\left(\frac{1}{w}\sum_{j=1}^{M}w_{j}\boldsymbol{z}_{j}^{\odot T}\boldsymbol{z}_{j}^{\odot}\right)\boldsymbol{\epsilon}^{*} = \frac{1}{w}\sum_{j=1}^{M}w_{j}\boldsymbol{z}_{j}^{\odot T}(\boldsymbol{y}_{j} - \boldsymbol{z}_{j}). \tag{9.101}$$

**左端化简**（不需原始点）：
$$\frac{1}{w} \sum_j w_j \boldsymbol{z}_j^{\odot T} \boldsymbol{z}_j^{\odot} = \boldsymbol{\mathcal{T}}_{op}^{-T} \underbrace{\left(\frac{1}{w} \sum_j w_j \boldsymbol{p}_j^{\odot T} \boldsymbol{p}_j^{\odot}\right)}_{\boldsymbol{\mathcal M}} \boldsymbol{\mathcal{T}}_{op}^{-1}, \tag{9.102}$$
$$\boldsymbol{\mathcal{T}}_{\text{op}} = \operatorname{Ad}(\mathbf{T}_{\text{op}}),\quad \boldsymbol{\mathcal M} = \begin{bmatrix} \mathbf{1} & \mathbf{0} \\ \mathbf{p}^{\wedge} & \mathbf{1} \end{bmatrix} \begin{bmatrix} \mathbf{1} & \mathbf{0} \\ \mathbf{0} & \mathbf{I} \end{bmatrix} \begin{bmatrix} \mathbf{1} & -\mathbf{p}^{\wedge} \\ \mathbf{0} & \mathbf{1} \end{bmatrix}, \tag{9.103}$$
$$w = \sum_j w_j,\quad \mathbf{p} = \frac{1}{w} \sum_j w_j \mathbf{p}_j,\quad \mathbf{I} = -\frac{1}{w} \sum_j w_j (\mathbf{p}_j - \mathbf{p})^{\wedge} (\mathbf{p}_j - \mathbf{p})^{\wedge}.$$
$\boldsymbol{\mathcal M}$（$6\times6$）形如「广义质量矩阵」（Murray 1994，权重当质量），只依赖不动系点，是常量。

**右端化简**：
$$\mathbf{a} = \frac{1}{w} \sum_j w_j \boldsymbol{z}_j^{\odot T} (\boldsymbol{y}_j - \boldsymbol{z}_j) = \begin{bmatrix} \mathbf{y} - \mathbf{C}_{\text{op}} (\mathbf{p} - \mathbf{r}_{\text{op}}) \\ \mathbf{b} - \mathbf{y}^{\wedge} \mathbf{C}_{\text{op}} (\mathbf{p} - \mathbf{r}_{\text{op}}) \end{bmatrix}, \tag{9.104}$$
$$\mathbf{b} = \left[ \operatorname{tr}\left( \mathbf{1}_{i}^{\wedge} \mathbf{C}_{\text{op}} \mathbf{W}^{T} \right) \right]_{i},\quad \mathbf{T}_{\text{op}} = \begin{bmatrix} \mathbf{C}_{\text{op}} & -\mathbf{C}_{\text{op}} \mathbf{r}_{\text{op}} \\ \mathbf{0}^{T} & 1 \end{bmatrix}, \tag{9.105}$$
$$\mathbf{W} = \frac{1}{w} \sum_j w_j (\mathbf{y}_j - \mathbf{y}) (\mathbf{p}_j - \mathbf{p})^T,\quad \mathbf{y} = \frac{1}{w} \sum_j w_j \mathbf{y}_j. \tag{9.106}$$
（$\mathbf W,\mathbf y$ 可预先算。）**闭式更新**：
$$\boldsymbol\epsilon^* = \boldsymbol{\mathcal{T}}_{\text{op}} \boldsymbol{\mathcal M}^{-1} \boldsymbol{\mathcal{T}}_{\text{op}}^T \mathbf{a}, \tag{9.107}\qquad \mathbf{T}_{\mathrm{op}} \leftarrow \exp(\boldsymbol\epsilon^{*\wedge}) \mathbf{T}_{\mathrm{op}}, \tag{9.108}$$
迭代收敛取 $\hat{\mathbf T}_{v_ki}=\mathbf T_{\rm op}$（或 $\hat{\mathbf T}_{iv_k}=\hat{\mathbf T}_{v_ki}^{-1}$）。指数映射保 $\mathbf T_{\rm op}\in SE(3)$。**这正是 Gauss–Newton（§4.3.1）适配 SE(3)**。

**需三非共线点**：由 (9.103)
$$\det \boldsymbol{\mathcal M} = \det \mathbf{I}. \tag{9.109}$$
唯一解须 $\det\mathbf I\ne0$，充分条件 $\mathbf I$ 正定（同上节，至少三非共线点）。

---

## 9.2 点云跟踪（Point-Cloud Tracking）

与对齐相关，但现在要**随时间**估计物体位姿（测量 + 带输入的先验）。设运动/观测模型，给出递归（EKF）与批量（Gauss–Newton）两解。

### 9.2.1 问题设置（源 §9.2.1）

车辆状态（用图 9.1 设置）：$\mathbf r_i^{v_ki}$（$i$ 到 $V_k$，在 $\mathcal F_i$）与 $\mathbf C_{v_ki}$（$\mathcal F_i$ 到 $\mathcal F_{v_k}$），或合为
$$\mathbf{T}_{k} = \mathbf{T}_{v_{k}i} = \begin{bmatrix} \mathbf{C}_{v_{k}i} & -\mathbf{C}_{v_{k}i}\mathbf{r}_{i}^{v_{k}i} \\ \mathbf{0}^{T} & 1 \end{bmatrix}, \tag{9.110}$$
轨迹简记 $\mathbf x=\{\mathbf T_0,\mathbf T_1,\dots,\mathbf T_K\}$ (9.111)。

**(i) 运动先验/输入**：初始位姿（带不确定度）$\check{\mathbf T}_0$ (9.112)；平移速度 $\boldsymbol\nu_{v_k}^{iv_k}$、角速度 $\boldsymbol\omega_{v_k}^{iv_k}$（车体系表达），合为
$$\boldsymbol{\varpi}_{k} = \begin{bmatrix} \boldsymbol{\nu}_{v_{k}}^{iv_{k}} \\ \boldsymbol{\omega}_{v_{k}}^{iv_{k}} \end{bmatrix},\quad k=1\dots K, \tag{9.113}$$
（假设分段常值）。输入简记 $\mathbf v=\{\check{\mathbf T}_0,\boldsymbol\varpi_1,\dots,\boldsymbol\varpi_K\}$ (9.114)。

**(ii) 测量**：能测静止点 $P_j$ 在车体系位置 $\mathbf r_{v_k}^{p_jv_k}$，已知其不动系位置 $\mathbf r_i^{p_ji}$。记
$$\mathbf{y}_{jk} = \mathbf{r}_{v_k}^{p_j v_k}, \tag{9.115}$$
测量简记 $\mathbf y=\{\mathbf y_{11},\dots,\mathbf y_{M1},\dots,\mathbf y_{1K},\dots,\mathbf y_{MK}\}$ (9.116)。

### 9.2.2 运动先验（源 §9.2.2）

**连续时间** $SE(3)$ 运动学（源脚注 10：$\mathbf T=\mathbf T_{v_ki}$，$\boldsymbol\varpi$ 是 $\underline{\mathcal F}_{v_k}$ 表达的广义速度）：
$$\dot{\mathbf{T}} = \boldsymbol{\varpi}^{\wedge} \mathbf{T}, \tag{9.117}$$
含过程噪声扰动：
$$\mathbf{T} = \exp(\delta \boldsymbol{\xi}^{\wedge}) \bar{\mathbf{T}}, \tag{9.118a}\qquad \boldsymbol{\varpi} = \bar{\boldsymbol{\varpi}} + \delta \boldsymbol{\varpi}. \tag{9.118b}$$
分离（如源式 8.301）：
$$\text{标称：}\ \dot{\bar{\mathbf{T}}} = \bar{\boldsymbol{\varpi}}^{\wedge} \bar{\mathbf{T}}, \tag{9.119a}\qquad \text{扰动：}\ \delta \dot{\boldsymbol{\xi}} = \bar{\boldsymbol{\varpi}}^{\curlywedge} \, \delta \boldsymbol{\xi} + \delta \boldsymbol{\varpi}, \tag{9.119b}$$
（$\delta\boldsymbol\varpi(t)$ 视作过程噪声。注意 (9.119b) 用的是 $\bar{\boldsymbol\varpi}^{\curlywedge}$，即 $6\times6$ 小伴随；源 OCR 写 $\wedge$。）

**离散时间**（分段常值，用 §8.2.2）：
$$\text{标称：}\ \bar{\mathbf{T}}_k = \underbrace{\exp(\Delta t_k \bar{\boldsymbol{\varpi}}_k^{\wedge})}_{\boldsymbol{\Xi}_k} \bar{\mathbf{T}}_{k-1}, \tag{9.120a}$$
$$\text{扰动：}\ \delta \boldsymbol{\xi}_{k} = \underbrace{\exp(\Delta t_{k} \bar{\boldsymbol{\varpi}}_{k}^{\curlywedge})}_{\operatorname{Ad}(\boldsymbol{\Xi}_{k})} \delta \boldsymbol{\xi}_{k-1} + \mathbf{w}_{k}, \tag{9.120b}$$
$\Delta t_k=t_k-t_{k-1}$，$\mathbf w_k\sim\mathcal N(\mathbf0,\mathbf Q_k)$。（关键身份：$\exp(\Delta t\,\bar{\boldsymbol\varpi}^{\curlywedge})=\operatorname{Ad}(\exp(\Delta t\,\bar{\boldsymbol\varpi}^\wedge))=\operatorname{Ad}(\boldsymbol\Xi_k)$。）

### 9.2.3 测量模型（源 §9.2.3）

**非线性**（$3\times1$）：
$$\mathbf{y}_{jk} = \mathbf{D}^T \, \mathbf{T}_k \mathbf{p}_j + \mathbf{n}_{jk}, \tag{9.121}$$
$$\mathbf{p}_j = \begin{bmatrix} \mathbf{r}_i^{p_j i} \\ 1 \end{bmatrix}, \tag{9.122}\qquad \mathbf{D}^T = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix} \tag{9.123}$$
（$\mathbf D^T$ 去掉齐次坐标底行的 1），$\mathbf n_{jk}\sim\mathcal N(\mathbf0,\mathbf R_{jk})$。

**线性化**（扰动 $\mathbf T_k=\exp(\delta\boldsymbol\xi_k^\wedge)\bar{\mathbf T}_k$，$\mathbf y_{jk}=\bar{\mathbf y}_{jk}+\delta\mathbf y_{jk}$）：
$$\bar{\mathbf{y}}_{jk} + \delta \mathbf{y}_{jk} = \mathbf{D}^T (\exp(\delta \boldsymbol{\xi}_k^{\wedge}) \bar{\mathbf{T}}_k) \mathbf{p}_j + \mathbf{n}_{jk}, \tag{9.125}$$
减去标称 $\bar{\mathbf y}_{jk}=\mathbf D^T\bar{\mathbf T}_k\mathbf p_j$ (9.126)：
$$\delta \mathbf{y}_{jk} \approx \mathbf{D}^T (\bar{\mathbf{T}}_k \mathbf{p}_j)^{\odot} \delta \boldsymbol{\xi}_k + \mathbf{n}_{jk}, \tag{9.127}$$
一阶正确（用 $\delta\boldsymbol\xi_k^\wedge(\bar{\mathbf T}_k\mathbf p_j)=(\bar{\mathbf T}_k\mathbf p_j)^\odot\delta\boldsymbol\xi_k$）。

**Nomenclature**（与估计器记号对齐）：
$\hat{\mathbf T}_k$：$k$ 时刻 $4\times4$ 校正估计位姿；$\hat{\mathbf P}_k$：$6\times6$ 校正协方差（平移+转动）；$\check{\mathbf T}_k,\check{\mathbf P}_k$：预测位姿/协方差；$\check{\mathbf T}_0$：0 时刻先验位姿；$\boldsymbol\varpi_k$：$6\times1$ 输入广义速度；$\mathbf Q_k$：$6\times6$ 过程噪声协方差；$\mathbf y_{jk}$：$3\times1$ 测量；$\mathbf R_{jk}$：$3\times3$ 测量协方差。

### 9.2.4 EKF 解（源 §9.2.4）

**预测步**。均值前推：
$$\check{\mathbf{T}}_k = \underbrace{\exp(\Delta t_k \, \boldsymbol{\varpi}_k^{\wedge})}_{\boldsymbol{\Xi}_k} \hat{\mathbf{T}}_{k-1}. \tag{9.128}$$
协方差 $\check{\mathbf P}_k=E[\delta\check{\boldsymbol\xi}_k\delta\check{\boldsymbol\xi}_k^T]$ (9.129)，用扰动运动学 (9.120)：
$$\delta \check{\boldsymbol{\xi}}_{k} = \underbrace{\exp(\Delta t_{k} \boldsymbol{\varpi}_{k}^{\curlywedge})}_{\mathbf{F}_{k-1} = \operatorname{Ad}(\boldsymbol{\Xi}_{k})} \delta \hat{\boldsymbol{\xi}}_{k-1} + \mathbf{w}_{k}, \tag{9.130}$$
$$\mathbf{F}_{k-1} = \exp(\Delta t_k \,\boldsymbol{\varpi}_k^{\curlywedge}), \tag{9.131}$$
**只依赖输入不依赖状态**（指数表达不确定度的便利）。
$$\check{\mathbf{P}}_k = \mathbf{F}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{F}_{k-1}^T + \mathbf{Q}_k. \tag{9.132}$$

**校正步**（须特别处理位姿）。由 (9.127)：
$$\delta \mathbf{y}_{jk} = \underbrace{\mathbf{D}^{T} (\check{\mathbf{T}}_{k} \mathbf{p}_{j})^{\odot}}_{\mathbf{G}_{jk}} \delta \check{\boldsymbol{\xi}}_{k} + \mathbf{n}_{jk}, \tag{9.133}\qquad \mathbf{G}_{jk} = \mathbf{D}^T (\check{\mathbf{T}}_k \mathbf{p}_j)^{\odot}, \tag{9.134}$$
在预测均值 $\check{\mathbf T}_k$ 处求值。$M$ 个观测堆叠：
$$\mathbf{y}_{k} = \begin{bmatrix} \mathbf{y}_{1k} \\ \vdots \\ \mathbf{y}_{Mk} \end{bmatrix},\ \mathbf{G}_{k} = \begin{bmatrix} \mathbf{G}_{1k} \\ \vdots \\ \mathbf{G}_{Mk} \end{bmatrix},\ \mathbf{R}_{k} = \operatorname{diag}(\mathbf{R}_{1k}, \dots, \mathbf{R}_{Mk}). \tag{9.135}$$
Kalman 增益与协方差更新（与通用情形不变）：
$$\mathbf{K}_{k} = \check{\mathbf{P}}_{k} \mathbf{G}_{k}^{T} (\mathbf{G}_{k} \check{\mathbf{P}}_{k} \mathbf{G}_{k}^{T} + \mathbf{R}_{k})^{-1}, \tag{9.136a}\qquad \hat{\mathbf{P}}_k = (\mathbf{1} - \mathbf{K}_k \mathbf{G}_k) \check{\mathbf{P}}_k. \tag{9.136b}$$
注意 $\hat{\mathbf P}_k=E[\delta\hat{\boldsymbol\xi}_k\delta\hat{\boldsymbol\xi}_k^T]$ (9.137)。**均值更新须重排**：
$$\boldsymbol{\epsilon}_{k} = \underbrace{\ln(\hat{\mathbf{T}}_{k}\check{\mathbf{T}}_{k}^{-1})^{\vee}}_{\text{更新量}} = \mathbf{K}_{k}\underbrace{(\mathbf{y}_{k} - \check{\mathbf{y}}_{k})}_{\text{innovation}}, \tag{9.138}$$
$$\check{\mathbf{y}}_k = \begin{bmatrix} \check{\mathbf{y}}_{1k} \\ \vdots \\ \check{\mathbf{y}}_{Mk} \end{bmatrix},\quad \check{\mathbf{y}}_{jk} = \mathbf{D}^T \check{\mathbf{T}}_k \mathbf{p}_j, \tag{9.139}$$
$$\hat{\mathbf{T}}_k = \exp(\boldsymbol{\epsilon}_k^{\wedge}) \check{\mathbf{T}}_k. \tag{9.140}$$
（保均值在 $SE(3)$。）

**五条 EKF 方程汇总**：
$$\text{predictor:}\quad \check{\mathbf{P}}_{k} = \mathbf{F}_{k-1} \hat{\mathbf{P}}_{k-1} \mathbf{F}_{k-1}^{T} + \mathbf{Q}_{k}, \tag{9.141a}$$
$$\check{\mathbf{T}}_{k} = \boldsymbol{\Xi}_{k} \hat{\mathbf{T}}_{k-1}, \tag{9.141b}$$
$$\text{Kalman gain:}\quad \mathbf{K}_{k} = \check{\mathbf{P}}_{k} \mathbf{G}_{k}^{T} (\mathbf{G}_{k} \check{\mathbf{P}}_{k} \mathbf{G}_{k}^{T} + \mathbf{R}_{k})^{-1}, \tag{9.141c}$$
$$\text{corrector:}\quad \hat{\mathbf{P}}_{k} = (\mathbf{1} - \mathbf{K}_{k} \mathbf{G}_{k}) \check{\mathbf{P}}_{k}, \tag{9.141d}$$
$$\hat{\mathbf{T}}_{k} = \exp\left((\mathbf{K}_{k}(\mathbf{y}_{k} - \check{\mathbf{y}}_{k}))^{\wedge}\right) \check{\mathbf{T}}_{k}. \tag{9.141e}$$
**核心**：均值算在 $SE(3)$（李群），协方差算在 se(3)（李代数）。初始化用 $\check{\mathbf T}_0$。可改为迭代 EKF（围绕最新估计重线性化、迭代校正）。$\hat{\mathbf T}_{v_ki}=\hat{\mathbf T}_k$，需要时 $\hat{\mathbf T}_{iv_k}=\hat{\mathbf T}_k^{-1}$。

#### 与不变 EKF 的关系（源 §9.2.4「Relationship to Invariant EKF」，2ed new）

本书一般用**左扰动**定义李群上的分布与优化（通常是 $\mathbf T_{vi}$ 的「车体侧」扰动）。但左/右选择是任意的；某些情形选一更优。

**不变 EKF**（Barrau & Bonnabel 2017，源脚注 11 称其 IEKF，但本书 IEKF 已指迭代 EKF，故写「invariant」）：若运动/测量模型合特定形式，可在李群上建一个**误差动态酷似线性 KF** 的 EKF（回顾 §3.3.6）。特别地，状态 Jacobian $\mathbf F_{k-1},\mathbf G_k$ **不依赖当前状态估计**，使其可分析为**稳定观测器**。左/右选择取决于测量模型形式。

本书 (9.141) 不完全合模板：$\mathbf F_{k-1}$ 不依赖状态，但 $\mathbf G_k=\mathbf D^T(\check{\mathbf T}_k\mathbf p_j)^\odot$ **依赖状态**。然而此例可代数变形入不变 EKF 形式；或一开始就用右扰动可更直接达到。详见附录 C.4。

### 9.2.5 批量 MAP 解（源 §9.2.5，2ed update）

**误差项与目标**。输入误差（$\check{\mathbf T}_0,\boldsymbol\varpi_k$）：
$$\mathbf{e}_{v,k}(\mathbf{x}) = \begin{cases} \ln(\check{\mathbf{T}}_0 \mathbf{T}_0^{-1})^{\vee} & k = 0 \\ \ln(\boldsymbol{\Xi}_k \mathbf{T}_{k-1} \mathbf{T}_k^{-1})^{\vee} & k = 1 \dots K \end{cases}, \tag{9.142}$$
$\boldsymbol\Xi_k=\exp(\Delta t_k\boldsymbol\varpi_k^\wedge)$，$\mathbf x=\{\mathbf T_0,\dots,\mathbf T_K\}$。测量误差：
$$\mathbf{e}_{y,jk}(\mathbf{x}) = \mathbf{y}_{jk} - \mathbf{D}^T \mathbf{T}_k \mathbf{p}_j. \tag{9.143}$$

**噪声性质**（Bayesian：真位姿从先验抽样 $\mathbf T_k=\exp(\delta\boldsymbol\xi_k^\wedge)\check{\mathbf T}_k$，$\delta\boldsymbol\xi_k\sim\mathcal N(\mathbf0,\check{\mathbf P}_k)$ (9.144)）：
- 首个输入误差：
$$\mathbf{e}_{v,0}(\mathbf{x}) = \ln(\check{\mathbf{T}}_0 \mathbf{T}_0^{-1})^{\vee} = \ln(\check{\mathbf{T}}_0 \check{\mathbf{T}}_0^{-1} \exp(-\delta \boldsymbol{\xi}_0^{\wedge}))^{\vee} = -\delta \boldsymbol{\xi}_0, \tag{9.145}$$
$$\mathbf{e}_{v,0}(\mathbf{x}) \sim \mathcal{N}(\mathbf{0}, \check{\mathbf{P}}_{0}). \tag{9.146}$$
- 后续输入误差：
$$\mathbf{e}_{v,k}(\mathbf{x}) = \ln(\boldsymbol{\Xi}_{k} \exp(\delta \boldsymbol{\xi}_{k-1}^{\wedge}) \check{\mathbf{T}}_{k-1} \check{\mathbf{T}}_{k}^{-1} \exp(-\delta \boldsymbol{\xi}_{k}^{\wedge}))^{\vee}$$
$$= \ln(\underbrace{\boldsymbol{\Xi}_{k} \check{\mathbf{T}}_{k-1} \check{\mathbf{T}}_{k}^{-1}}_{1} \exp((\operatorname{Ad}(\boldsymbol{\Xi}_{k}) \delta \boldsymbol{\xi}_{k-1})^{\wedge}) \exp(-\delta \boldsymbol{\xi}_{k}^{\wedge}))^{\vee} \approx \operatorname{Ad}(\boldsymbol{\Xi}_{k}) \delta \boldsymbol{\xi}_{k-1} - \delta \boldsymbol{\xi}_{k} = -\mathbf{w}_{k}, \tag{9.147}$$
$$\mathbf{e}_{v,k}(\mathbf{x}) \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}_{k}). \tag{9.148}$$
- 测量误差：
$$\mathbf{e}_{y,jk}(\mathbf{x}) = \mathbf{y}_{jk} - \mathbf{D}^T \mathbf{T}_k \mathbf{p}_j = \mathbf{n}_{jk}, \tag{9.149}\qquad \mathbf{e}_{y,jk}(\mathbf{x}) \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_{jk}). \tag{9.150}$$

**目标函数**：
$$J_{v,k}(\mathbf{x}) = \begin{cases} \frac{1}{2} \mathbf{e}_{v,0}(\mathbf{x})^T \check{\mathbf{P}}_0^{-1} \mathbf{e}_{v,0}(\mathbf{x}) & k = 0\\ \frac{1}{2} \mathbf{e}_{v,k}(\mathbf{x})^T \mathbf{Q}_k^{-1} \mathbf{e}_{v,k}(\mathbf{x}) & k = 1 \dots K \end{cases}, \tag{9.151a}$$
$$J_{y,k}(\mathbf{x}) = \frac{1}{2} \mathbf{e}_{y,k}(\mathbf{x})^T \mathbf{R}_k^{-1} \mathbf{e}_{y,k}(\mathbf{x}), \tag{9.151b}$$
$$\mathbf{e}_{y,k}(\mathbf{x}) = \begin{bmatrix} \mathbf{e}_{y,1k}(\mathbf{x}) \\ \vdots \\ \mathbf{e}_{y,Mk}(\mathbf{x}) \end{bmatrix},\quad \mathbf{R}_k = \operatorname{diag}(\mathbf{R}_{1k}, \dots, \mathbf{R}_{Mk}), \tag{9.152}$$
$$J(\mathbf{x}) = \sum_{k=0}^{K} (J_{v,k}(\mathbf{x}) + J_{y,k}(\mathbf{x})). \tag{9.153}$$

**线性化误差项**（围绕 $\mathbf T_{{\rm op},k}$，$\mathbf T_k=\exp(\boldsymbol\epsilon_k^\wedge)\mathbf T_{{\rm op},k}$ (9.154)，$\mathbf x_{\rm op}=\{\mathbf T_{{\rm op},1},\dots\}$ (9.155)）：
- 首个：
$$\mathbf{e}_{v,0}(\mathbf{x}) = \ln(\underbrace{\check{\mathbf{T}}_0\mathbf{T}_{\mathrm{op},0}^{-1}}_{\exp(\mathbf{e}_{v,0}(\mathbf{x}_{\mathrm{op}})^{\wedge})} \exp(-\boldsymbol{\epsilon}_0^{\wedge}))^{\vee} \approx \mathbf{e}_{v,0}(\mathbf{x}_{\mathrm{op}}) - \underbrace{\boldsymbol{\mathcal J}(-\mathbf{e}_{v,0}(\mathbf{x}_{\mathrm{op}}))^{-1}}_{\mathbf{E}_0}\boldsymbol{\epsilon}_0, \tag{9.156}$$
$\mathbf e_{v,0}(\mathbf x_{\rm op})=\ln(\check{\mathbf T}_0\mathbf T_{{\rm op},0}^{-1})^\vee$。
- 后续：
$$\mathbf{e}_{v,k}(\mathbf{x}) = \ln(\underbrace{\boldsymbol{\Xi}_{k} \mathbf{T}_{\text{op},k-1} \mathbf{T}_{\text{op},k}^{-1}}_{\exp(\mathbf{e}_{v,k}(\mathbf{x}_{\text{op}})^{\wedge})} \exp((\operatorname{Ad}(\mathbf{T}_{\text{op},k} \mathbf{T}_{\text{op},k-1}^{-1}) \boldsymbol{\epsilon}_{k-1})^{\wedge}) \exp(-\boldsymbol{\epsilon}_{k}^{\wedge}))^{\vee}$$
$$\approx \mathbf{e}_{v,k}(\mathbf{x}_{\text{op}}) + \underbrace{\boldsymbol{\mathcal{J}}(-\mathbf{e}_{v,k}(\mathbf{x}_{\text{op}}))^{-1} \operatorname{Ad}(\mathbf{T}_{\text{op},k} \mathbf{T}_{\text{op},k-1}^{-1})}_{\mathbf{F}_{k-1}} \boldsymbol{\epsilon}_{k-1} - \underbrace{\boldsymbol{\mathcal{J}}(-\mathbf{e}_{v,k}(\mathbf{x}_{\text{op}}))^{-1}}_{\mathbf{E}_{k}} \boldsymbol{\epsilon}_{k}$$
$$= \mathbf{e}_{v,k}(\mathbf{x}_{\text{op}}) + \mathbf{F}_{k-1} \boldsymbol{\epsilon}_{k-1} - \mathbf{E}_{k} \boldsymbol{\epsilon}_{k}, \tag{9.157}$$
$\mathbf e_{v,k}(\mathbf x_{\rm op})=\ln(\boldsymbol\Xi_k\mathbf T_{{\rm op},k-1}\mathbf T_{{\rm op},k}^{-1})^\vee$。
- 测量：
$$\mathbf{e}_{y,jk}(\mathbf{x}) = \mathbf{y}_{jk} - \mathbf{D}^{T} \exp(\boldsymbol{\epsilon}_{k}^{\wedge}) \mathbf{T}_{\text{op},k} \mathbf{p}_{j} \approx \underbrace{\mathbf{y}_{jk} - \mathbf{D}^{T} \mathbf{T}_{\text{op},k} \mathbf{p}_{j}}_{\mathbf{e}_{y,jk}(\mathbf{x}_{\text{op}})} - \underbrace{\mathbf{D}^{T} (\mathbf{T}_{\text{op},k} \mathbf{p}_{j})^{\odot}}_{\mathbf{G}_{jk}} \boldsymbol{\epsilon}_{k}. \tag{9.158}$$
堆叠：
$$\mathbf{e}_{y,k}(\mathbf{x}) \approx \mathbf{e}_{y,k}(\mathbf{x}_{op}) - \mathbf{G}_k \boldsymbol{\epsilon}_k, \tag{9.159}\qquad \mathbf{G}_k = \begin{bmatrix} \mathbf{G}_{1k} \\ \vdots \\ \mathbf{G}_{Mk} \end{bmatrix}. \tag{9.160}$$

**Gauss–Newton 更新**。堆叠量（$\mathbf H$ 块双对角、$\mathbf A=\mathbf H^T\mathbf W^{-1}\mathbf H$ 块三对角）：
$$\delta \mathbf{x} = \begin{bmatrix} \boldsymbol{\epsilon}_0 \\ \boldsymbol{\epsilon}_1 \\ \vdots \\ \boldsymbol{\epsilon}_K \end{bmatrix},\quad \mathbf e(\mathbf x_{\rm op})=\begin{bmatrix}\mathbf e_{v,0}(\mathbf x_{\rm op})\\\vdots\\\mathbf e_{v,K}(\mathbf x_{\rm op})\\\mathbf e_{y,0}(\mathbf x_{\rm op})\\\vdots\\\mathbf e_{y,K}(\mathbf x_{\rm op})\end{bmatrix}, \tag{9.161}$$
（$\mathbf H$ 由运动块 $\mathbf E_0$ 及各 $-\mathbf F_{k-1},\mathbf E_k$ 与测量块 $\mathbf G_k$ 组成，源式 9.161 给出双对角结构。）
$$\mathbf{W} = \operatorname{diag}(\check{\mathbf{P}}_{0}, \mathbf{Q}_{1}, \dots, \mathbf{Q}_{K}, \mathbf{R}_{0}, \mathbf{R}_{1}, \dots, \mathbf{R}_{K}), \tag{9.162}$$
$$J(\mathbf{x}) \approx J(\mathbf{x}_{\text{op}}) - \mathbf{b}^T \delta \mathbf{x} + \frac{1}{2} \delta \mathbf{x}^T \mathbf{A} \delta \mathbf{x}, \tag{9.163}$$
$$\mathbf{A} = \underbrace{\mathbf{H}^T \mathbf{W}^{-1} \mathbf{H}}_{\text{block-tridiagonal}},\quad \mathbf{b} = \mathbf{H}^T \mathbf{W}^{-1} \mathbf{e}(\mathbf{x}_{\text{op}}). \tag{9.164}$$
$$\mathbf{A}\,\delta\mathbf{x}^{*} = \mathbf{b}, \tag{9.165}\qquad \delta \mathbf{x}^{*} = [\boldsymbol{\epsilon}_{0}^{*};\boldsymbol{\epsilon}_{1}^{*};\dots;\boldsymbol{\epsilon}_{K}^{*}], \tag{9.166}$$
$$\mathbf{T}_{\mathrm{op},k} \leftarrow \exp(\boldsymbol{\epsilon}_{k}^{*\wedge}) \mathbf{T}_{\mathrm{op},k}, \tag{9.167}$$
迭代收敛。$\hat{\mathbf T}_{v_ki}=\mathbf T_{{\rm op},k}$。**核心**：更新算在 se(3)，均值存在 $SE(3)$。

---

## 9.3 位姿图松弛（Pose-Graph Relaxation）

不直接测点，而从一组**相对位姿「测量」**（伪测量，来自 dead-reckoning）出发（图 9.3，每个白三角=三维参考系）。**位姿图**只含位姿无点。图可含闭环（及叶节点），但相对位姿测量不确定、绕环复合未必为单位。任务：相对某个（任选）privileged 位姿 0「松弛」位姿图，求各位姿相对 0 的最优估计。

### 9.3.1 问题设置（源 §9.3.1）

位姿 $k$ 处隐含参考系 $\underline{\mathcal F}_k$。$\mathbf T_k$：$\mathcal F_k$ 相对 $\mathcal F_0$ 的位姿。测量为节点间相对位姿变化，假设是 $SE(3)$ 上高斯，均值+协方差 $\{\bar{\mathbf T}_{k\ell},\boldsymbol\Sigma_{k\ell}\}$ (9.168)。随机样本：
$$\mathbf{T}_{k\ell} = \exp(\boldsymbol{\xi}_{k\ell}^{\wedge}) \bar{\mathbf{T}}_{k\ell}, \tag{9.169}\qquad \boldsymbol{\xi}_{k\ell} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_{k\ell}). \tag{9.170}$$
（来自轮里程、视觉里程、惯性等；不是所有位姿对都有测量，故实际稀疏。）

### 9.3.2 批量 ML 解（源 §9.3.2，类比 §8.3.6 位姿融合）

每测量误差：
$$\mathbf{e}_{k\ell}(\mathbf{x}) = \ln(\bar{\mathbf{T}}_{k\ell} (\mathbf{T}_k \mathbf{T}_{\ell}^{-1})^{-1})^{\vee} = \ln(\bar{\mathbf{T}}_{k\ell} \mathbf{T}_{\ell} \mathbf{T}_{k}^{-1})^{\vee}, \tag{9.171}$$
$\mathbf x=\{\mathbf T_1,\dots,\mathbf T_K\}$ (9.172)。$SE(3)$-敏感扰动 $\mathbf T_k=\exp(\boldsymbol\epsilon_k^\wedge)\mathbf T_{{\rm op},k}$ (9.173)：
$$\mathbf{e}_{k\ell}(\mathbf{x}) = \ln(\bar{\mathbf{T}}_{k\ell} \exp(\boldsymbol{\epsilon}_{\ell}^{\wedge}) \mathbf{T}_{\text{op},\ell} \mathbf{T}_{\text{op},k}^{-1} \exp(-\boldsymbol{\epsilon}_{k}^{\wedge}))^{\vee}. \tag{9.174}$$
把 $\boldsymbol\epsilon_\ell$ 移到右边（**无近似**，用伴随穿越）：
$$\mathbf{e}_{k\ell}(\mathbf{x}) = \ln(\underbrace{\bar{\mathbf{T}}_{k\ell}\mathbf{T}_{\mathrm{op},\ell}\mathbf{T}_{\mathrm{op},k}^{-1}}_{\text{small}} \exp((\boldsymbol{\mathcal{T}}_{\mathrm{op},k}\boldsymbol{\mathcal{T}}_{\mathrm{op},\ell}^{-1}\boldsymbol{\epsilon}_{\ell})^{\wedge}) \exp(-\boldsymbol{\epsilon}_{k}^{\wedge}))^{\vee}, \tag{9.175}$$
$\boldsymbol{\mathcal T}_{{\rm op},k}=\operatorname{Ad}(\mathbf T_{{\rm op},k})$。$\boldsymbol\epsilon_\ell,\boldsymbol\epsilon_k\to0$ 故合并：
$$\mathbf{e}_{k\ell}(\mathbf{x}) \approx \ln(\exp(\mathbf{e}_{k\ell}(\mathbf{x}_{\mathrm{op}})^{\wedge}) \exp((\boldsymbol{\mathcal{T}}_{\mathrm{op},k}\boldsymbol{\mathcal{T}}_{\mathrm{op},\ell}^{-1}\boldsymbol{\epsilon}_{\ell} - \boldsymbol{\epsilon}_{k})^{\wedge}))^{\vee}, \tag{9.176}$$
$$\mathbf{e}_{k\ell}(\mathbf{x}_{\text{op}}) = \ln(\bar{\mathbf{T}}_{k\ell} \mathbf{T}_{\text{op},\ell} \mathbf{T}_{\text{op},k}^{-1})^{\vee}, \tag{9.177a}\qquad \mathbf{x}_{\text{op}} = \{\mathbf{T}_{\text{op},1}, \dots, \mathbf{T}_{\text{op},K}\}. \tag{9.177b}$$
用 BCH 近似（源式 8.105）得线性化误差：
$$\mathbf{e}_{k\ell}(\mathbf{x}) \approx \mathbf{e}_{k\ell}(\mathbf{x}_{op}) - \mathbf{G}_{k\ell} \, \delta \mathbf{x}_{k\ell}, \tag{9.178}$$
$$\mathbf{G}_{k\ell} = \begin{bmatrix} -\boldsymbol{\mathcal{J}}(-\mathbf{e}_{k\ell}(\mathbf{x}_{\mathrm{op}}))^{-1} \boldsymbol{\mathcal{T}}_{\mathrm{op},k} \boldsymbol{\mathcal{T}}_{\mathrm{op},\ell}^{-1} & \boldsymbol{\mathcal{J}}(-\mathbf{e}_{k\ell}(\mathbf{x}_{\mathrm{op}}))^{-1} \end{bmatrix}, \tag{9.179a}\qquad \delta \mathbf{x}_{k\ell} = \begin{bmatrix} \boldsymbol{\epsilon}_{\ell} \\ \boldsymbol{\epsilon}_{k} \end{bmatrix}. \tag{9.179b}$$
可取 $\boldsymbol{\mathcal J}\approx\mathbf1$ 简化，但保全式有益（§8.3.6）：收敛后 $\mathbf e_{k\ell}(\mathbf x_{\rm op})\ne\mathbf0$（非零残差）。

**ML 目标**：
$$J(\mathbf{x}) = \frac{1}{2} \sum_{k,\ell} \mathbf{e}_{k\ell}(\mathbf{x})^T \boldsymbol{\Sigma}_{k\ell}^{-1} \mathbf{e}_{k\ell}(\mathbf{x}), \tag{9.180}$$
（每条相对位姿测量一项。）代入：
$$J(\mathbf{x}) \approx \frac{1}{2} \sum_{k,\ell} (\mathbf{e}_{k\ell}(\mathbf{x}_{op}) - \mathbf{G}_{k\ell} \mathbf{P}_{k\ell} \delta \mathbf{x})^{T} \boldsymbol{\Sigma}_{k\ell}^{-1} (\mathbf{e}_{k\ell}(\mathbf{x}_{op}) - \mathbf{G}_{k\ell} \mathbf{P}_{k\ell} \delta \mathbf{x}), \tag{9.181}$$
$$J(\mathbf{x}) \approx J(\mathbf{x}_{\text{op}}) - \mathbf{b}^T \delta \mathbf{x} + \frac{1}{2} \delta \mathbf{x}^T \mathbf{A} \delta \mathbf{x}, \tag{9.182}$$
$$\mathbf{b} = \sum_{k,\ell} \mathbf{P}_{k\ell}^T \mathbf{G}_{k\ell}^T \boldsymbol{\Sigma}_{k\ell}^{-1} \mathbf{e}_{k\ell}(\mathbf{x}_{op}), \tag{9.183a}$$
$$\mathbf{A} = \sum_{k,\ell} \mathbf{P}_{k\ell}^T \mathbf{G}_{k\ell}^T \boldsymbol{\Sigma}_{k\ell}^{-1} \mathbf{G}_{k\ell} \mathbf{P}_{k\ell}, \tag{9.183b}\qquad \delta \mathbf{x}_{k\ell} = \mathbf{P}_{k\ell} \delta \mathbf{x}, \tag{9.183c}$$
$\mathbf P_{k\ell}$ 是从全扰动 $\delta\mathbf x=[\boldsymbol\epsilon_1;\dots;\boldsymbol\epsilon_K]$ (9.184) 中挑出 $k\ell$ 变量的投影矩阵。
$$\frac{\partial J(\mathbf{x})}{\partial \delta \mathbf{x}^T} = -\mathbf{b} + \mathbf{A} \delta \mathbf{x}, \tag{9.185}\qquad \mathbf{A}\,\delta\mathbf{x}^{*} = \mathbf{b}, \tag{9.186}$$
$$\delta \mathbf{x}^* = [\boldsymbol{\epsilon}_1^*;\dots;\boldsymbol{\epsilon}_K^*], \tag{9.187}\qquad \mathbf{T}_{\mathrm{op},k} \leftarrow \exp(\boldsymbol{\epsilon}_{k}^{*\wedge}) \mathbf{T}_{\mathrm{op},k}, \tag{9.188}$$
迭代收敛取 $\hat{\mathbf T}_{k0}=\mathbf T_{{\rm op},k}$。

### 9.3.3 初始化（源 §9.3.3）

找**生成树**（图 9.4），从 privileged 节点 0 向外**复合**（一部分）相对位姿测量得各位姿初值。生成树不唯一；**浅树优于深树**（累积不确定度少）。

### 9.3.4 利用稀疏性（源 §9.3.4）

位姿图有固有稀疏性（源脚注 12：本节是上节暴力法的近似，因系统非线性）。图 9.5：有些节点（空三角）只有 1 或 2 条边，形成两类局部链：
- **(i) Constrained（受约束）**：链两端都接 junction（实三角）节点，其测量对图其余部分重要。
- **(ii) Cantilevered（悬臂）**：只一端接 junction，其测量不影响其余部分。

用 §8.3.3 的位姿复合法把 constrained 链的相对测量合成一条新相对测量替代其组分；再对只含 junction 节点的**约简图**做松弛。之后固定 junction，求局部链节点：悬臂链直接从其 junction 向外复合（代价线性于链长，无需迭代）；每条 constrained 链跑一个小型松弛（两端 junction 固定，沿链顺序排变量则 $\mathbf A$ 块三对角，每迭代代价线性于链长——稀疏 Cholesky + 前后向）。

> 注：两阶段法非唯一途径；好的稀疏求解器也能直接利用全系统 $\mathbf A$ 的稀疏性，免去识别/记账局部链。

### 9.3.5 链例（源 §9.3.5，图 9.6）

短 constrained 链：只解位姿 1,2,3,4（0 与 5 固定）。$\mathbf A$ 块三对角：
$$\mathbf{A} = \begin{bmatrix} \mathbf{A}_{11} & \mathbf{A}_{12} & & \\ \mathbf{A}_{12}^{T} & \mathbf{A}_{22} & \mathbf{A}_{23} & \\ & \mathbf{A}_{23}^{T} & \mathbf{A}_{33} & \mathbf{A}_{34} \\ & & \mathbf{A}_{34}^{T} & \mathbf{A}_{44} \end{bmatrix}$$
其分块（源式 9.189，OCR 后期错位，按结构整理）：
$$\mathbf A_{11}=\boldsymbol{\Sigma}_{10}^{\prime-1} + \boldsymbol{\mathcal T}_{21}^{T} \boldsymbol{\Sigma}_{21}^{\prime-1} \boldsymbol{\mathcal T}_{21},\quad \mathbf A_{12}=-\boldsymbol{\mathcal T}_{21}^{T} \boldsymbol{\Sigma}_{21}^{\prime-1},$$
$$\mathbf A_{22}=\boldsymbol{\Sigma}_{21}^{\prime-1} + \boldsymbol{\mathcal T}_{32}^{T} \boldsymbol{\Sigma}_{32}^{\prime-1} \boldsymbol{\mathcal T}_{32},\quad \mathbf A_{23}=-\boldsymbol{\mathcal T}_{32}^{T} \boldsymbol{\Sigma}_{32}^{\prime-1},$$
$$\mathbf A_{33}=\boldsymbol{\Sigma}_{32}^{\prime-1} + \boldsymbol{\mathcal T}_{43}^{T} \boldsymbol{\Sigma}_{43}^{\prime-1} \boldsymbol{\mathcal T}_{43},\quad \mathbf A_{34}=-\boldsymbol{\mathcal T}_{43}^{T} \boldsymbol{\Sigma}_{43}^{\prime-1},$$
$$\mathbf A_{44}=\boldsymbol{\Sigma}_{43}^{\prime-1} + \boldsymbol{\mathcal T}_{54}^{T} \boldsymbol{\Sigma}_{54}^{\prime-1} \boldsymbol{\mathcal T}_{54}, \tag{9.189}$$
其中
$$\boldsymbol{\Sigma}_{k\ell}^{\prime-1} = \boldsymbol{\mathcal{J}}_{k\ell}^{-T} \boldsymbol{\Sigma}_{k\ell}^{-1} \boldsymbol{\mathcal{J}}_{k\ell}^{-1}, \tag{9.190a}\qquad \boldsymbol{\mathcal T}_{k\ell} = \boldsymbol{\mathcal T}_{\text{op},k} \boldsymbol{\mathcal T}_{\text{op},\ell}^{-1}, \tag{9.190b}\qquad \boldsymbol{\mathcal{J}}_{k\ell} = \boldsymbol{\mathcal J}(-\mathbf{e}_{k\ell}(\mathbf{x}_{op})). \tag{9.190c}$$
$\mathbf b$：
$$\mathbf{b} = \begin{bmatrix} \mathbf{b}_{1} \\ \mathbf{b}_{2} \\ \mathbf{b}_{3} \\ \mathbf{b}_{4} \end{bmatrix} = \begin{bmatrix} \boldsymbol{\mathcal J}_{10}^{-T} \boldsymbol{\Sigma}_{10}^{-1} \mathbf{e}_{10}(\mathbf{x}_{\text{op}}) - \boldsymbol{\mathcal T}_{21}^{T} \boldsymbol{\mathcal J}_{21}^{-T} \boldsymbol{\Sigma}_{21}^{-1} \mathbf{e}_{21}(\mathbf{x}_{\text{op}}) \\ \boldsymbol{\mathcal J}_{21}^{-T} \boldsymbol{\Sigma}_{21}^{-1} \mathbf{e}_{21}(\mathbf{x}_{\text{op}}) - \boldsymbol{\mathcal T}_{32}^{T} \boldsymbol{\mathcal J}_{32}^{-T} \boldsymbol{\Sigma}_{32}^{-1} \mathbf{e}_{32}(\mathbf{x}_{\text{op}}) \\ \boldsymbol{\mathcal J}_{32}^{-T} \boldsymbol{\Sigma}_{32}^{-1} \mathbf{e}_{32}(\mathbf{x}_{\text{op}}) - \boldsymbol{\mathcal T}_{43}^{T} \boldsymbol{\mathcal J}_{43}^{-T} \boldsymbol{\Sigma}_{43}^{-1} \mathbf{e}_{43}(\mathbf{x}_{\text{op}}) \\ \boldsymbol{\mathcal J}_{43}^{-T} \boldsymbol{\Sigma}_{43}^{-1} \mathbf{e}_{43}(\mathbf{x}_{\text{op}}) - \boldsymbol{\mathcal T}_{54}^{T} \boldsymbol{\mathcal J}_{54}^{-T} \boldsymbol{\Sigma}_{54}^{-1} \mathbf{e}_{54}(\mathbf{x}_{\text{op}}) \end{bmatrix}. \tag{9.191}$$
（源式 9.191 末项 OCR 把 $\boldsymbol\Sigma_{43},\mathbf e_{43},\boldsymbol\Sigma_{54},\mathbf e_{54}$ 误渲染为 $\boldsymbol\Sigma_{21},\mathbf e_{21}$；按链结构应为 43、54 项，已修正。）

**稀疏 Cholesky** 解 $\mathbf A\delta\mathbf x^*=\mathbf b$：$\mathbf A=\mathbf U\mathbf U^T$ (9.192)，$\mathbf U$ 上三角块带状 (9.193)。**分块求解 $\mathbf U$**（自右下角向上）：
$$\mathbf{U}_{44}\mathbf{U}_{44}^T = \mathbf{A}_{44}:\ \text{Cholesky 求 }\mathbf U_{44},$$
$$\mathbf{U}_{34}\mathbf{U}_{44}^T = \mathbf{A}_{34}:\ \text{线代求 }\mathbf U_{34},$$
$$\mathbf{U}_{33}\mathbf{U}_{33}^T + \mathbf{U}_{34}\mathbf{U}_{34}^T = \mathbf{A}_{33}:\ \text{Cholesky 求 }\mathbf U_{33},$$
$$\mathbf{U}_{23}\mathbf{U}_{33}^T = \mathbf{A}_{23}:\ \text{线代求 }\mathbf U_{23},$$
$$\mathbf{U}_{22}\mathbf{U}_{22}^T + \mathbf{U}_{23}\mathbf{U}_{23}^T = \mathbf{A}_{22}:\ \text{Cholesky 求 }\mathbf U_{22},$$
$$\mathbf{U}_{12}\mathbf{U}_{22}^T = \mathbf{A}_{12}:\ \text{线代求 }\mathbf U_{12},$$
$$\mathbf{U}_{11}\mathbf{U}_{11}^T + \mathbf{U}_{12}\mathbf{U}_{12}^T = \mathbf{A}_{11}:\ \text{Cholesky 求 }\mathbf U_{11}.$$
然后**后向**解 $\mathbf c_4,\mathbf c_3,\mathbf c_2,\mathbf c_1$（$\mathbf U\mathbf c=\mathbf b$ 类），再**前向**解 $\boldsymbol\epsilon_1^*,\boldsymbol\epsilon_2^*,\boldsymbol\epsilon_3^*,\boldsymbol\epsilon_4^*$（$\mathbf U^T\delta\mathbf x^*=\mathbf c$）。各步代价线性于链长。更新
$$\mathbf{T}_{\mathrm{op},k} \leftarrow \exp(\boldsymbol{\epsilon}_{k}^{*\wedge}) \mathbf{T}_{\mathrm{op},k}, \tag{9.194}$$
迭代收敛。（短链不值得，长链收益明显。）

---

## 9.4 惯性导航（Inertial Navigation，2ed new）

> **本节是 ch8 中与「李群与李代数」最相关的部分**：引入扩展位姿群 $SE_2(3)$、其指数/伴随/雅可比/逆、time machine 辅助群 $\mathbb T$，并给出 IMU 预积分（filtering + batch）。**全节用右扰动**（与本书一致）。

惯性导航自 Apollo（1960s–70s）起关键（Apollo Guidance Computer 用 IMU 在 EKF 中估计指令舱/登月舱状态；'Doc' Draper 为「惯性导航之父」）。IMU 基本模型见 §7.4.4（即 ch7）。

### 9.4.1 问题设置（源 §9.4.1）

IMU 测三线加速度+三角速率。设 IMU 系=车体系 $\mathcal F_s=\mathcal F_v$，简化版（源式 7.154）：
$$\begin{bmatrix} \tilde{\mathbf{a}} \\ \tilde{\boldsymbol{\omega}} \end{bmatrix} = \begin{bmatrix} \mathbf{C}_{vi} (\mathbf{a}_i^{vi} - \mathbf{g}_i) \\ \boldsymbol{\omega}_v^{vi} \end{bmatrix}, \tag{9.195}$$
$\tilde{\mathbf a}$ 车体系测加速度，$\tilde{\boldsymbol\omega}$ 车体系测角速度，$\mathbf g_i$ 惯性系重力加速度，$\mathbf a_i^{vi}$ 惯性系加速度，$\mathbf C_{vi}$ 旋转，$\boldsymbol\omega_v^{vi}$ 车体系角速度。

含缓变偏置 $\mathbf b$ + 高斯过程噪声 $\mathbf w$：
$$\begin{bmatrix} \tilde{\mathbf{a}} \\ \tilde{\boldsymbol{\omega}} \end{bmatrix} = \begin{bmatrix} \mathbf{C}_{iv}^{T} (\mathbf{a}_{i}^{vi} - \mathbf{g}_{i}) \\ \boldsymbol{\omega}_{v}^{vi} \end{bmatrix} + \underbrace{\begin{bmatrix} \mathbf{b}_{a} \\ \mathbf{b}_{\omega} \end{bmatrix}}_{\mathbf{b}} + \underbrace{\begin{bmatrix} \mathbf{w}_{a} \\ \mathbf{w}_{\omega} \end{bmatrix}}_{\mathbf{w}}. \tag{9.196}$$
状态 $\mathbf x_k$ 含位姿、平移速度、IMU 偏置。能否估偏置是**可观测性**问题（§5.2）。

### 9.4.2 扩展位姿 $SE_2(3)$（源 §9.4.2）⭐

因测线加速度，须把线速度 $\mathbf v$ 纳入状态。最优雅做法：把 $SE(3)$ 扩为含速度的**扩展位姿**：
$$\mathbf{T} = \begin{bmatrix} \mathbf{C} & \mathbf{r} & \mathbf{v} \\ \mathbf{0}^T & 1 & 0 \\ \mathbf{0}^T & 0 & 1 \end{bmatrix}. \tag{9.197}$$
此类 $5\times5$ 矩阵构成矩阵李群 $SE_2(3)$（运作类似 $SE(3)$，不正式建立；详见 Barrau 2015；Barrau & Bonnabel 2018b；Brossard 2021）。结果摘要：

**指数映射**（$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\nu;\boldsymbol\phi]\in\mathfrak{se}_2(3)$，注意排序「位置、速度、转动」）：
$$\mathbf{T} = \exp(\boldsymbol{\xi}^{\wedge}) = \exp\left(\begin{bmatrix}\boldsymbol{\phi}^{\wedge} & \boldsymbol{\rho} & \boldsymbol{\nu}\\\mathbf{0}^{T} & 0 & 0\\\mathbf{0}^{T} & 0 & 0\end{bmatrix}\right) = \begin{bmatrix}\mathbf{C} & \mathbf{J}(\boldsymbol{\phi})\boldsymbol{\rho} & \mathbf{J}(\boldsymbol{\phi})\boldsymbol{\nu}\\\mathbf{0}^{T} & 1 & 0\\\mathbf{0}^{T} & 0 & 1\end{bmatrix} = \begin{bmatrix}\mathbf{C} & \mathbf{r} & \mathbf{v}\\\mathbf{0}^{T} & 1 & 0\\\mathbf{0}^{T} & 0 & 1\end{bmatrix}, \tag{9.198}$$
$\mathbf J(\cdot)$ 是 $SO(3)$ 左雅可比，$\wedge$ 已为 $\mathfrak{se}_2(3)$ 扩展。映射仅满射。

**$9\times9$ 伴随**：
$$\boldsymbol{\mathcal T} = \operatorname{Ad}(\mathbf{T}) = \exp\left(\begin{bmatrix} \boldsymbol{\phi}^{\wedge} & \mathbf{0} & \boldsymbol{\rho}^{\wedge} \\ \mathbf{0} & \boldsymbol{\phi}^{\wedge} & \boldsymbol{\nu}^{\wedge} \\ \mathbf{0} & \mathbf{0} & \boldsymbol{\phi}^{\wedge} \end{bmatrix}\right) = \begin{bmatrix} \mathbf{C} & \mathbf{0} & (\mathbf{J}(\boldsymbol{\phi})\boldsymbol{\rho})^{\wedge}\mathbf{C} \\ \mathbf{0} & \mathbf{C} & (\mathbf{J}(\boldsymbol{\phi})\boldsymbol{\nu})^{\wedge}\mathbf{C} \\ \mathbf{0} & \mathbf{0} & \mathbf{C} \end{bmatrix} = \begin{bmatrix} \mathbf{C} & \mathbf{0} & \mathbf{r}^{\wedge}\mathbf{C} \\ \mathbf{0} & \mathbf{C} & \mathbf{v}^{\wedge}\mathbf{C} \\ \mathbf{0} & \mathbf{0} & \mathbf{C} \end{bmatrix}. \tag{9.199}$$
（中间式是 $\operatorname{Ad}=\exp(\operatorname{ad})$，$\operatorname{ad}(\boldsymbol\xi)=\boldsymbol\xi^{\curlywedge}$，源 OCR 上标写 $\wedge/\lambda$。）

**身份**（$SE_2(3)$ 仍成立）：
$$(\boldsymbol{\mathcal T}\boldsymbol{\xi})^{\wedge} = \mathbf{T}\boldsymbol{\xi}^{\wedge}\mathbf{T}^{-1}, \tag{9.200a}\qquad (\boldsymbol{\mathcal T}\boldsymbol{\xi})^{\curlywedge} = \boldsymbol{\mathcal T}\boldsymbol{\xi}^{\curlywedge}\boldsymbol{\mathcal T}^{-1}, \tag{9.200b}$$
$$\exp((\boldsymbol{\mathcal T}\boldsymbol{\xi})^{\wedge}) = \mathbf{T}\exp(\boldsymbol{\xi}^{\wedge})\mathbf{T}^{-1}, \tag{9.200c}\qquad \exp((\boldsymbol{\mathcal T}\boldsymbol{\xi})^{\curlywedge}) = \boldsymbol{\mathcal T}\exp(\boldsymbol{\xi}^{\curlywedge})\boldsymbol{\mathcal T}^{-1}. \tag{9.200d}$$

**逆**：
$$\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{C}^T & -\mathbf{C}^T \mathbf{r} & -\mathbf{C}^T \mathbf{v} \\ \mathbf{0}^T & 1 & 0 \\ \mathbf{0}^T & 0 & 1 \end{bmatrix}, \quad \boldsymbol{\mathcal T}^{-1} = \begin{bmatrix} \mathbf{C}^T & \mathbf{0} & -\mathbf{C}^T \mathbf{r}^{\wedge} \\ \mathbf{0} & \mathbf{C}^T & -\mathbf{C}^T \mathbf{v}^{\wedge} \\ \mathbf{0} & \mathbf{0} & \mathbf{C}^T \end{bmatrix}. \tag{9.201}$$

**$SE_2(3)$ 左雅可比**：
$$\boldsymbol{\mathcal{J}}(\boldsymbol{\xi}) = \begin{bmatrix} \mathbf{J}(\boldsymbol\phi) & \mathbf{0} & \mathbf{Q}(\boldsymbol\phi, \boldsymbol\rho) \\ \mathbf{0} & \mathbf{J}(\boldsymbol\phi) & \mathbf{Q}(\boldsymbol\phi, \boldsymbol\nu) \\ \mathbf{0} & \mathbf{0} & \mathbf{J}(\boldsymbol\phi) \end{bmatrix}, \tag{9.202}$$
$\mathbf Q(\cdot,\cdot)$ 见源式 8.91a（即本书 `lie_theory.tex` 附录 $\mathbf Q_l$；勿与后文过程噪声 $\mathbf Q$ 混）。

### 9.4.3 运动模型（源 §9.4.3）

IMU 数据常作运动模型**输入**（Lupton & Sukkarieh 2012；Forster 2015）。平移/速度通常存全局帧。连续时间：
$$\dot{\mathbf{C}}_{iv} = \mathbf{C}_{iv} \boldsymbol{\omega}_{v}^{vi\wedge}, \tag{9.203a}\qquad \dot{\mathbf{r}}_{i}^{vi} = \mathbf{v}_{i}^{vi}, \tag{9.203b}\qquad \dot{\mathbf{v}}_{i}^{vi} = \mathbf{a}_{i}^{vi}. \tag{9.203c}$$
合为扩展位姿
$$\mathbf{T}_{iv} = \begin{bmatrix} \mathbf{C}_{iv} & \mathbf{r}_i^{vi} & \mathbf{v}_i^{vi} \\ \mathbf{0}^T & 1 & 0 \\ \mathbf{0}^T & 0 & 1 \end{bmatrix} \in SE_2(3). \tag{9.204}$$
（本书通常用其逆 $\mathbf T_{vi}$，后文调和。）代入 (9.196) 并丢下标：
$$\dot{\mathbf{C}} = \mathbf{C} (\tilde{\boldsymbol{\omega}} - \mathbf{b}_{\omega} - \mathbf{w}_{\omega})^{\wedge}, \tag{9.205a}\qquad \dot{\mathbf{r}} = \mathbf{v}, \tag{9.205b}$$
$$\dot{\mathbf{v}} = \mathbf{C} (\tilde{\mathbf{a}} - \mathbf{b}_a - \mathbf{w}_a) + \mathbf{g}, \tag{9.205c}\qquad \dot{\mathbf{b}}_{\omega} = \mathbf{w}_{b,\omega}, \tag{9.205d}\qquad \dot{\mathbf{b}}_a = \mathbf{w}_{ba}. \tag{9.205e}$$
（偏置建模为随机游走。）**右扰动**（扩展位姿）+ 偏置加性扰动：
$$\underbrace{\begin{bmatrix} \mathbf{C} & \mathbf{r} & \mathbf{v} \\ \mathbf{0}^{T} & 1 & 0 \\ \mathbf{0}^{T} & 0 & 1 \end{bmatrix}}_{\mathbf{T}} = \underbrace{\begin{bmatrix} \bar{\mathbf{C}} & \bar{\mathbf{r}} & \bar{\mathbf{v}} \\ \mathbf{0}^{T} & 1 & 0 \\ \mathbf{0}^{T} & 0 & 1 \end{bmatrix}}_{\bar{\mathbf{T}}} \underbrace{\begin{bmatrix} \exp(\delta\boldsymbol\phi^{\wedge}) & \mathbf{J}(\delta\boldsymbol\phi)\delta\boldsymbol\rho & \mathbf{J}(\delta\boldsymbol\phi)\delta\boldsymbol\nu \\ \mathbf{0}^{T} & 1 & 0 \\ \mathbf{0}^{T} & 0 & 1 \end{bmatrix}}_{\exp(\delta\boldsymbol\xi^{\wedge})}, \tag{9.206a}$$
$$\mathbf{b}_{\omega} = \bar{\mathbf{b}}_{\omega} + \delta\mathbf{b}_{\omega}, \quad \mathbf{b}_{a} = \bar{\mathbf{b}}_{a} + \delta\mathbf{b}_{a}. \tag{9.206b}$$
> **源明确：这是书中唯一的右乘扰动**，是右扰动的好例子；与本书一贯「扰动放车体侧」一致，因现在用 $\mathbf T_{iv}$ 而非 $\mathbf T_{vi}$。

**标称**（用 §8.2.3 方法分离）：
$$\underbrace{\begin{bmatrix} \dot{\bar{\mathbf{C}}} & \dot{\bar{\mathbf{r}}} & \dot{\bar{\mathbf{v}}} \\ \mathbf{0}^{T} & 0 & 0 \\ \mathbf{0}^{T} & 0 & 0 \end{bmatrix}}_{\dot{\bar{\mathbf{T}}}} = \bar{\mathbf{T}}\underbrace{\begin{bmatrix} \bar{\boldsymbol{\omega}}^{\wedge} & \mathbf{0} & \bar{\mathbf{a}} \\ \mathbf{0}^{T} & 0 & 0 \\ \mathbf{0}^{T} & 1 & 0 \end{bmatrix}}_{\boldsymbol{\Omega}} + \underbrace{\begin{bmatrix} \mathbf{0} & \mathbf{0} & \mathbf{g} \\ \mathbf{0}^{T} & 0 & 0 \\ \mathbf{0}^{T} & -1 & 0 \end{bmatrix}}_{\boldsymbol{\Gamma}}, \tag{9.207a}\qquad \dot{\bar{\mathbf{b}}}_{\omega} = \mathbf{0},\ \dot{\bar{\mathbf{b}}}_{a} = \mathbf{0}. \tag{9.207b}$$
**扰动**（LTV 系统）：
$$\underbrace{\begin{bmatrix} \delta \dot{\boldsymbol\rho} \\ \delta \dot{\boldsymbol\nu} \\ \delta \dot{\boldsymbol\phi} \\ \delta \dot{\mathbf{b}}_{\omega} \\ \delta \dot{\mathbf{b}}_{a} \end{bmatrix}}_{\delta \dot{\mathbf{x}}} = \underbrace{\begin{bmatrix} -\bar{\boldsymbol\omega}^{\wedge} & \mathbf{1} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & -\bar{\boldsymbol\omega}^{\wedge} & -\bar{\mathbf{a}}^{\wedge} & \mathbf{0} & -\mathbf{1} \\ \mathbf{0} & \mathbf{0} & -\bar{\boldsymbol\omega}^{\wedge} & -\mathbf{1} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \end{bmatrix}}_{\mathbf{A}} \underbrace{\begin{bmatrix} \delta \boldsymbol\rho \\ \delta \boldsymbol\nu \\ \delta \boldsymbol\phi \\ \delta \mathbf{b}_{\omega} \\ \delta \mathbf{b}_{a} \end{bmatrix}}_{\delta \mathbf{x}} + \underbrace{\begin{bmatrix} \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & -\mathbf{1} & \mathbf{0} & \mathbf{0} \\ -\mathbf{1} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{1} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{1} \end{bmatrix}}_{\mathbf{L}} \underbrace{\begin{bmatrix} \mathbf{w}_{a} \\ \mathbf{w}_{\omega} \\ \mathbf{w}_{b,\omega} \\ \mathbf{w}_{b,a} \end{bmatrix}}_{\mathbf{w}}, \tag{9.208}$$
$\bar{\boldsymbol\omega}=\tilde{\boldsymbol\omega}-\bar{\mathbf b}_\omega$，$\bar{\mathbf a}=\tilde{\mathbf a}-\bar{\mathbf b}_a$。标称看似 LTV 实为非线性（偏置藏在 $\bar{\boldsymbol\omega},\bar{\mathbf a}$）；扰动模型是 LTV。**标称传均值、扰动传不确定度**。

### 9.4.4 传播均值（源 §9.4.4）

两假设：(i) IMU 测量分段常值（区间 $(\tau_{j-1},\tau_j)$ 内常）；(ii) 偏置在同（或更大）区间常。目标：把 (9.207) 从 $\tau_{j-1}$ 积到 $\tau_j$。

偏置平凡：$\bar{\mathbf b}_\omega(\tau_j)=\bar{\mathbf b}_\omega(\tau_{j-1})$，$\bar{\mathbf b}_a(\tau_j)=\bar{\mathbf b}_a(\tau_{j-1})$。$\bar{\boldsymbol\omega},\bar{\mathbf a}$ 常 $\Rightarrow$ (9.207a) 为 LTI，解（见附录 C.3）：
$$\bar{\mathbf{T}}(\tau_j) = \bar{\mathbf{T}}(\tau_{j-1})\bar{\boldsymbol{\Phi}}(\tau_{j-1}, \tau_j) + \underbrace{\int_{\tau_{j-1}}^{\tau_j} \boldsymbol{\Gamma}\,\bar{\boldsymbol{\Phi}}(t, \tau_j)\,dt}_{\boldsymbol{\Lambda}(\Delta\tau_j)}, \tag{9.209}$$
（右乘 LTI）转移函数：
$$\bar{\boldsymbol{\Phi}}(\tau_{j-1}, \tau_j) = \exp(\boldsymbol{\Omega}(\tau_j)\,\Delta\tau_j) = \begin{bmatrix} \exp(\Delta\tau_j\,\bar{\boldsymbol{\omega}}_j^{\wedge}) & \frac{1}{2} \Delta\tau_j^2\,\mathbf{N}(\Delta\tau_j\,\bar{\boldsymbol{\omega}}_j)\,\bar{\mathbf{a}}_j & \Delta\tau_j\,\mathbf{J}(\Delta\tau_j\,\bar{\boldsymbol{\omega}}_j)\,\bar{\mathbf{a}}_j \\ \mathbf{0}^T & 1 & 0 \\ \mathbf{0}^T & \Delta\tau_j & 1 \end{bmatrix}, \tag{9.210}$$
$\Delta\tau_j=\tau_j-\tau_{j-1}$，$\bar{\boldsymbol\omega}_j=\tilde{\boldsymbol\omega}(\tau_j)-\mathbf b_\omega$，$\bar{\mathbf a}_j=\tilde{\mathbf a}(\tau_j)-\mathbf b_a$。

**新矩阵函数 $\mathbf N$**（$\mathbf J$ 是 $SO(3)$ 左雅可比）：
$$\mathbf{N}(\phi \mathbf{a}) = 2 \int_0^1 \alpha\, \mathbf{J}(\alpha\phi\mathbf{a})\, d\alpha = 2 \sum_{n=0}^\infty \frac{1}{(n+2)!} (\phi\mathbf{a}^\wedge)^n$$
$$= 2\frac{1 - \cos\phi}{\phi^2} \mathbf{1} + \left(1 - 2\frac{1 - \cos\phi}{\phi^2}\right) \mathbf{a}\mathbf{a}^T + 2\frac{\phi - \sin\phi}{\phi^2} \mathbf{a}^\wedge, \tag{9.211}$$
$\phi$ 角度，$\mathbf a$ 单位轴。

剩余积分：
$$\boldsymbol{\Lambda}(\Delta\tau_j) = \int_{\tau_{j-1}}^{\tau_j} \begin{bmatrix} \mathbf{0} & (\tau_j - t)\mathbf{g} & \mathbf{g} \\ \mathbf{0}^T & 0 & 0 \\ \mathbf{0}^T & -1 & 0 \end{bmatrix} dt = \begin{bmatrix} \mathbf{0} & \frac{1}{2}\Delta\tau_j^2\,\mathbf{g} & \Delta\tau_j\,\mathbf{g} \\ \mathbf{0}^T & 0 & 0 \\ \mathbf{0}^T & -\Delta\tau_j & 0 \end{bmatrix}. \tag{9.212}$$

**离散更新汇总**：
$$\bar{\mathbf{C}}(\tau_j) = \bar{\mathbf{C}}(\tau_{j-1}) \exp(\Delta\tau_j\,\bar{\boldsymbol{\omega}}_j^{\wedge}), \tag{9.213a}$$
$$\bar{\mathbf{r}}(\tau_j) = \bar{\mathbf{r}}(\tau_{j-1}) + \Delta\tau_j \bar{\mathbf{v}}(\tau_{j-1}) + \frac{1}{2}\Delta\tau_j^2 (\boldsymbol{\alpha}_r(\tau_j) + \mathbf{g}), \tag{9.213b}$$
$$\bar{\mathbf{v}}(\tau_j) = \bar{\mathbf{v}}(\tau_{j-1}) + \Delta\tau_j (\boldsymbol\alpha_v(\tau_j) + \mathbf{g}), \tag{9.213c}$$
$$\boldsymbol{\alpha}_r(\tau_j) = \bar{\mathbf{C}}(\tau_{j-1})\,\mathbf{N}(\Delta\tau_j\,\bar{\boldsymbol{\omega}}_j)\bar{\mathbf{a}}_j, \tag{9.214a}\qquad \boldsymbol{\alpha}_{v}(\tau_j) = \bar{\mathbf{C}}(\tau_{j-1})\mathbf{J}(\Delta\tau_j\,\bar{\boldsymbol{\omega}}_j)\bar{\mathbf{a}}_j. \tag{9.214b}$$

### 9.4.5 传播协方差（源 §9.4.5）

连续时间假设扰动是联合零均值高斯过程：
$$\delta \mathbf{x}(t) \sim \mathcal{GP}(\mathbf{0}, \check{\mathbf{P}}(t, t')), \tag{9.215}$$
$\check{\mathbf P}(t,t')=E[\delta\mathbf x(t)\delta\mathbf x(t')^T]$，由白噪声 $\mathbf w(t)\sim\mathcal{GP}(\mathbf0,\mathbf Q\delta(t-t'))$ 驱动（$\mathbf Q$ 功率谱密度，§3.4.2）。随机积分扰动模型：
$$\check{\mathbf{P}}(t,t') = \boldsymbol{\Phi}(t,t_0)\check{\mathbf{P}}(t_0,t_0)\boldsymbol{\Phi}(t',t_0)^T + \int_{t_0}^{\min(t,t')} \boldsymbol{\Phi}(t,s)\mathbf{L}\mathbf{Q}\mathbf{L}^T\boldsymbol{\Phi}(t',s)^T ds, \tag{9.216}$$
$\boldsymbol\Phi(t,t_0)$ 为（左乘）LTV 转移函数。$\tau_{j-1}$ 到 $\tau_j$（IMU/偏置常）为 LTI：
$$\boldsymbol{\Phi}(\tau_j, \tau_{j-1}) = \exp(\mathbf{A}_j\,\Delta\tau_j), \tag{9.217}$$
$\mathbf A_j=\mathbf A(\tau_j)$（$\bar{\boldsymbol\omega}_j=\tilde{\boldsymbol\omega}(\tau_j)-\bar{\mathbf b}_\omega$，$\bar{\mathbf a}_j=\tilde{\mathbf a}(\tau_j)-\bar{\mathbf b}_a$）。
$$\check{\mathbf{P}}(\tau_j, \tau_j) = \boldsymbol{\Phi}(\tau_j, \tau_{j-1}) \check{\mathbf{P}}(\tau_{j-1}, \tau_{j-1}) \boldsymbol{\Phi}(\tau_j, \tau_{j-1})^T + \mathbf{Q}(\tau_j, \tau_{j-1}), \tag{9.218}$$
$$\mathbf{Q}(\tau_j, \tau_{j-1}) = \int_{\tau_{j-1}}^{\tau_j} \boldsymbol{\Phi}(\tau_j, t) \mathbf{L}\mathbf{Q}\mathbf{L}^T \boldsymbol{\Phi}(\tau_j, t)^T dt. \tag{9.219}$$
数值近似（$\boldsymbol\Phi(\tau_j,t)\approx\mathbf1+\mathbf A_j(\tau_j-t)+\cdots$）：
$$\mathbf{Q}(\tau_j, \tau_{j-1}) \approx \Delta\tau_j\,\mathbf{L}\mathbf{Q}\mathbf{L}^T + \frac{1}{2}\Delta\tau_j^2 (\mathbf{A}_j\mathbf{L}\mathbf{Q}\mathbf{L}^T + \mathbf{L}\mathbf{Q}\mathbf{L}^T \mathbf{A}_j^T)$$
$$+ \frac{1}{6}\Delta\tau_j^3 (2\mathbf{A}_j\mathbf{L}\mathbf{Q}\mathbf{L}^T \mathbf{A}_j^T + \mathbf{A}_j^2\mathbf{L}\mathbf{Q}\mathbf{L}^T + \mathbf{L}\mathbf{Q}\mathbf{L}^T \mathbf{A}_j^{2T}), \tag{9.220}$$
$O(\Delta\tau_j^3)$（源脚注 13：可能需保这么多项以保 $\mathbf Q$ 正定）。又
$$E\left[(\boldsymbol{\Phi}(\tau_j, \tau_{j-1})\delta\mathbf{x}(\tau_{j-1}) - \delta\mathbf{x}(\tau_j))(\cdots)^{T}\right] = \mathbf{Q}(\tau_j, \tau_{j-1}), \tag{9.221}$$
后续批量估计用到。
