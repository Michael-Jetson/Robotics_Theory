# 抽取留痕：Forster 等《On-Manifold Preintegration for Real-Time Visual-Inertial Odometry》

> **源（主）**：C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, **IEEE Transactions on Robotics (TRO)**, vol. 33, no. 1, pp. 1–21, Feb. 2017。
> **arXiv**：arXiv:1512.02363（v3）。**PDF（作者站，含全部附录）**：https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf ；**arXiv PDF**：https://arxiv.org/pdf/1512.02363 ；**arXiv 摘要页**：https://arxiv.org/abs/1512.02363
> **源（副，RSS 会议版补充材料，含完整逐步证明）**：C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, *Supplementary Material to: IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation*, **Technical Report GT-IRIM-CP&R-2015-001**, in Robotics: Science and Systems (RSS), 2015。
> **PDF（补充材料）**：https://rpg.ifi.uzh.ch/docs/RSS15_Forster_Supplementary.pdf
> **服务章节**：IMU 模型与预积分。
> **抽取范围**：TRO 全文与 IMU 相关的 §III（预备）、§IV（MAP 框架与状态）、§V（IMU 模型与运动积分）、§VI（流形上 IMU 预积分）、§VII（无结构视觉因子，仅为接口）、附录 IX-A/B/C/D/E（迭代噪声传播、bias 一阶修正、残差雅可比、零空间投影、欧拉角积分）；以及 RSS 补充材料 §1–§4（噪声传播、bias 修正、残差雅可比、SO(3) 角速度与右雅可比）。逐节、逐式、逐证、逐表保真。
> **抽取人备注**：本文件是【内部抽取留痕】，不是成书正文；遵循「禁摘要·全量保真」铁律。公式用 LaTeX 写全，保留所有式号与条件。**两源式号体系不同**：TRO 版用 (1)–(85)；RSS 补充材料正文式用 “(x)” 指 RSS 主论文，附录式用 “(A.x)”。本文件以 **TRO 式号为主线**，并在对应处给出 RSS 补充材料的 (A.x) 式号（因补充材料给出更细的逐步证明，bias 修正与噪声传播以补充材料的 (A.x) 推导为权威，二者数学等价）。
>
> **重要范围说明（务必读）**：本论文是 IMU 预积分的**奠基论文**之一（在 Lupton & Sukkarieh 2012 用欧拉角的预积分基础上，给出**流形 SO(3) 上**的完整、解析、可在线增量的预积分理论）。本论文 IMU 部分**自包含**：测量模型→连续/离散运动学→相对增量定义→噪声分离→噪声协方差迭代传播→bias 一阶修正→残差与解析雅可比，全部齐备。本论文**未涉及**：四元数运动学细节（用旋转矩阵 R 与 SO(3) 指数映射，不用四元数）、SE(3) 指数映射（刻意回避，见 §III-A/§III-C 的 retraction 选择）、连续时间预积分（用离散欧拉积分）、左扰动（全程用**右扰动/右雅可比**，与本书主约定一致）。视觉部分（无结构因子、Schur 补、零空间投影、iSAM2）非本章重点，仅抽“与 VIO 的接口”所需骨架。

---

## §0 记号约定（本源）与本书统一约定的差异

> 本节汇总 Forster 等全文记号选择（主要来自 §III、§IV-A、§V、附录），并逐项对照本书统一约定（$\mathbf R\in\mathrm{SO}(3)$、Hamilton 四元数、**右扰动为主**、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前、协方差等）。

| 项目 | Forster 本源约定 | 本书统一约定 | 差异/转换说明 |
|---|---|---|---|
| 旋转表示 | **旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$**（不用四元数），$\mathrm{SO}(3)=\{\mathbf R\in\mathbb R^{3\times3}:\mathbf R^\top\mathbf R=\mathbf I,\det(\mathbf R)=1\}$（式 1 上下文） | $\mathbf R\in\mathrm{SO}(3)$ | **一致**。本源用 $\mathbf R$（非 Barfoot 的 $\mathbf C$）。本源完全不用四元数，故无 Hamilton/JPL 之分；综合时若需四元数实现，按 Hamilton 转换。 |
| 帧记号 | $\mathbf R_{\mathrm{WB}}$＝body(B)→world(W) 的旋转；前缀 $_{\mathrm B}(\cdot)$/$_{\mathrm W}(\cdot)$ 表所在坐标系；body 系 B 与 IMU 系重合（图 2） | 一般用 $\mathbf R_{wb}$ 同义 | **一致**（大小写记号差异）。$\mathbf R_{\mathrm{WB}}\mathbf x_{\mathrm B}=\mathbf x_{\mathrm W}$。 |
| 扰动方向（**关键**） | **右扰动**：$\tilde{\mathbf R}=\mathbf R\,\mathrm{Exp}(\boldsymbol\epsilon)$（式 12）；retraction $\mathcal R_{\mathbf R}(\delta\boldsymbol\phi)=\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$（式 20）；用**右雅可比 $\mathbf J_r$** | **右扰动为主**，右雅可比 $\mathbf J_r$ | **完全一致**。本源是右扰动文献的代表，无需翻转。 |
| 指数/对数 | 小写 $\exp:\mathfrak{so}(3)\to\mathrm{SO}(3)$ 作用于 $\boldsymbol\phi^\wedge$（式 3 Rodrigues）；大写 $\mathrm{Exp}:\mathbb R^3\to\mathrm{SO}(3)$，$\mathrm{Exp}(\boldsymbol\phi)=\exp(\boldsymbol\phi^\wedge)$（式 6）；$\mathrm{Log}$ 反之 | 同（Exp/Log 大写约定，$\mathbb R^3$ 旋转向量，无 1/2 半角因为不用四元数） | **一致**。注意本源 $\mathrm{Exp}$ 作用于旋转向量（满角），与四元数 $\mathrm{Exp}$ 的半角不同。 |
| hat/vee | hat $(\cdot)^\wedge:\mathbb R^3\to\mathfrak{so}(3)$（式 1）；vee $(\cdot)^\vee$ 反之 | $(\cdot)^\wedge$ 或 $[\cdot]_\times$；$(\cdot)^\vee$ | 一致。$\boldsymbol\omega^\wedge=[\boldsymbol\omega]_\times$。 |
| 右雅可比 | $\mathbf J_r(\boldsymbol\phi)$（式 8），逆 $\mathbf J_r^{-1}(\boldsymbol\phi)$（式 9 下），闭式见 §1 | $\mathbf J_r$ 为主 | **一致**，引 Chirikjian 2012 与 Barfoot–Furgale 2014。 |
| 状态向量 | $\mathbf x_i=[\mathbf R_i,\mathbf p_i,\mathbf v_i,\mathbf b_i]$（式 22），姿态在前、位置次之、速度、bias 末（$\mathbf b_i=[\mathbf b_i^g,\mathbf b_i^a]\in\mathbb R^6$） | $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（针对 $\mathfrak{se}(3)$ 切向量） | Forster 是 IMU 全状态列表，与 $\mathfrak{se}(3)$ 的 $[\boldsymbol\rho;\boldsymbol\phi]$ 是不同层次。**注意**：本源 SE(3) retraction（式 21）切向量排序为 $[\delta\boldsymbol\phi,\delta\mathbf p]$（**旋转在前**），与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）**相反**——综合时若拼装 $\mathfrak{se}(3)$ 协方差/雅可比需调换块顺序。 |
| bias | 陀螺 bias $\mathbf b^g$，加计 bias $\mathbf b^a$；随机游走（布朗运动）$\dot{\mathbf b}^g=\boldsymbol\eta^{bg}$，$\dot{\mathbf b}^a=\boldsymbol\eta^{ba}$（式 46） | $\mathbf b_g,\mathbf b_a$ 同义 | 一致（上下标位置差异）。 |
| 噪声 | 连续白噪声 $\boldsymbol\eta^g,\boldsymbol\eta^a$（式 27–28）；离散 $\boldsymbol\eta^{gd},\boldsymbol\eta^{ad}$（式 30 上下文），$\mathrm{Cov}(\boldsymbol\eta^{gd})=\tfrac1{\Delta t}\mathrm{Cov}(\boldsymbol\eta^g)$ | 过程噪声常记 $\mathbf w$ | 一致；注意离散/连续协方差的 $1/\Delta t$ 关系（见 §5）。 |
| 协方差字母 | 旋转“高斯”协方差 $\boldsymbol\Sigma$（式 12）；预积分噪声协方差 $\boldsymbol\Sigma_{ij}$（式 39）；原始 IMU 噪声协方差 $\boldsymbol\Sigma_\eta$（附录 A）；bias 离散噪声协方差 $\boldsymbol\Sigma_{bgd},\boldsymbol\Sigma_{bad}$（式 47 上下文）；视觉 $\boldsymbol\Sigma_C$ | $\mathbf P$/$\mathbf Q$/$\mathbf R$ | **差异**：本源用 $\boldsymbol\Sigma$ 系列（$\mathbf R$ 已被旋转占用），无 $\mathbf P/\mathbf Q/\mathbf R$ 命名。预积分协方差 $\boldsymbol\Sigma_{ij}$ 充当“测量信息”的逆权重，在 MAP 中以 $\|\cdot\|^2_{\boldsymbol\Sigma_{ij}}$ 形式出现。 |
| 预积分量（**核心记号**） | $\Delta\mathbf R_{ij},\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$＝真值相对增量（式 33）；$\Delta\tilde{\mathbf R}_{ij},\Delta\tilde{\mathbf v}_{ij},\Delta\tilde{\mathbf p}_{ij}$＝由含噪测量算出的**预积分测量**（式 35–38）；$\Delta\bar{\mathbf R}_{ij}$ 等＝在某 bias 估计 $\bar{\mathbf b}_i$ 处算出的预积分（式 64） | — | 记号严格区分：无波浪=真值；$\tilde{(\cdot)}$=含噪测量量；$\bar{(\cdot)}$=在给定 bias 估计处的值。综合务必沿用此三分。 |
| 误差/残差 | 预积分噪声 $\delta\boldsymbol\phi_{ij},\delta\mathbf v_{ij},\delta\mathbf p_{ij}$（式 35–37）；残差 $\mathbf r_{\Delta\mathbf R_{ij}},\mathbf r_{\Delta\mathbf v_{ij}},\mathbf r_{\Delta\mathbf p_{ij}}$（式 45） | $\delta$/残差 $\mathbf r$ | 一致。 |
| 测量集合 | $\mathcal I_{ij}$＝两关键帧 $i,j$ 间 IMU 测量集；$\mathcal C_i$＝帧 $i$ 图像测量；$\mathcal K_k$＝到 $k$ 的关键帧集；$\mathcal X_k$＝待估状态集（式 23）；$\mathcal Z_k$＝测量集（式 24） | — | 记号说明性，无冲突。 |
| ⊕/⊖ 与 retraction | 不显式用 ⊕/⊖；用 retraction $\mathcal R_{\mathbf x}$（式 18–21），“lift–solve–retract” | 本书右 ⊕ 等价于本源 retraction | 语义一致。本源强调 retraction 选择对 SE(3) 用 $(\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi),\mathbf p+\mathbf R\delta\mathbf p)$，从而**只需 SO(3) 指数映射**。 |
| 高斯记号 | $\mathcal N(\boldsymbol\mu,\boldsymbol\Sigma)$ | $\mathcal N(\boldsymbol\mu,\boldsymbol\Sigma)$ | 一致。 |
| 重力 | $\mathbf g$＝世界系重力向量（式 28），$_{\mathrm W}\mathbf g$ | $\mathbf g$ | 一致。**符号约定**：加计模型为 $_{\mathrm B}\tilde{\mathbf a}=\mathbf R_{\mathrm{WB}}^\top(_{\mathrm W}\mathbf a-_{\mathrm W}\mathbf g)+\mathbf b^a+\boldsymbol\eta^a$，故运动学里出现 $+\mathbf g\Delta t$（见式 31）。 |

**本源未使用/未涉及**：四元数、SE(3) 指数映射、左扰动/左雅可比 $\mathbf J_l$、连续时间预积分、信息滤波、卡尔曼滤波（本源是优化/因子图/MAP 框架，明确对比滤波的劣势，见 §II）。

---

## §1 SO(3)/SE(3) 预备：流形、指数映射、右雅可比 [TRO §III-A；RSS §4]

> 本节给出预积分所需的全部李群工具。**这是后续每一步推导反复调用的引理库**，务必完整。

### §1.1 SO(3) 群、hat/vee、指数映射 [TRO §III-A]

**SO(3) 定义**：3D 旋转矩阵群
$$\mathrm{SO}(3)=\{\mathbf R\in\mathbb R^{3\times3}:\mathbf R^\top\mathbf R=\mathbf I,\ \det(\mathbf R)=1\}.$$
群运算为矩阵乘法，逆为转置。SO(3) 是光滑流形，其在单位元处的切空间记 $\mathfrak{so}(3)$（李代数），即 $3\times3$ 反对称矩阵空间。

**hat 算子**（向量↔反对称矩阵）：
$$\boldsymbol\omega^\wedge=\begin{bmatrix}\omega_1\\\omega_2\\\omega_3\end{bmatrix}^\wedge=\begin{bmatrix}0&-\omega_3&\omega_2\\\omega_3&0&-\omega_1\\-\omega_2&\omega_1&0\end{bmatrix}\in\mathfrak{so}(3).\tag{1}$$
**vee 算子** $(\cdot)^\vee$：对反对称矩阵 $\mathbf S=\boldsymbol\omega^\wedge$，有 $\mathbf S^\vee=\boldsymbol\omega$。

**反对称矩阵有用性质**（后续“移项”反复用到）：
$$\mathbf a^\wedge\mathbf b=-\mathbf b^\wedge\mathbf a,\qquad\forall\mathbf a,\mathbf b\in\mathbb R^3.\tag{2}$$

**指数映射（Rodrigues 公式）** $\exp:\mathfrak{so}(3)\to\mathrm{SO}(3)$，与标准矩阵指数一致：
$$\exp(\boldsymbol\phi^\wedge)=\mathbf I+\frac{\sin(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|}\boldsymbol\phi^\wedge+\frac{1-\cos(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|^2}(\boldsymbol\phi^\wedge)^2.\tag{3}$$

**指数映射的一阶近似**（后续核心简化）：
$$\exp(\boldsymbol\phi^\wedge)\approx\mathbf I+\boldsymbol\phi^\wedge.\tag{4}$$

**对数映射** $\log:\mathrm{SO}(3)\to\mathfrak{so}(3)$（$\mathbf R\ne\mathbf I$）：
$$\log(\mathbf R)=\frac{\varphi\cdot(\mathbf R-\mathbf R^\top)}{2\sin(\varphi)}\quad\text{with}\quad\varphi=\cos^{-1}\!\left(\frac{\mathrm{tr}(\mathbf R)-1}{2}\right).\tag{5}$$
注：$\log(\mathbf R)^\vee=\mathbf a\varphi$，$\mathbf a,\varphi$ 分别为 $\mathbf R$ 的转轴与转角。若 $\mathbf R=\mathbf I$ 则 $\varphi=0$，$\mathbf a$ 不定可任取。指数映射在开球 $\|\boldsymbol\phi\|<\pi$ 上为双射，其逆即对数映射；不限定域时为满射（$\boldsymbol\phi=(\varphi+2k\pi)\mathbf a,k\in\mathbb Z$ 都是合法对数）。

**向量化指数/对数**（直接作用于向量而非反对称矩阵）：
$$\mathrm{Exp}:\mathbb R^3\to\mathrm{SO}(3);\ \boldsymbol\phi\mapsto\exp(\boldsymbol\phi^\wedge),\qquad\mathrm{Log}:\mathrm{SO}(3)\to\mathbb R^3;\ \mathbf R\mapsto\log(\mathbf R)^\vee.\tag{6}$$

### §1.2 右雅可比与一阶近似（BCH） [TRO §III-A；RSS §4]

**右雅可比下的加法↔右乘一阶近似**（图 1）：
$$\mathrm{Exp}(\boldsymbol\phi+\delta\boldsymbol\phi)\approx\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}\big(\mathbf J_r(\boldsymbol\phi)\,\delta\boldsymbol\phi\big).\tag{7}$$
$\mathbf J_r(\boldsymbol\phi)$ 是 SO(3) 的**右雅可比**[Chirikjian 2012, p.40]，把切空间的加法增量关联到右乘的乘法增量。闭式：
$$\mathbf J_r(\boldsymbol\phi)=\mathbf I-\frac{1-\cos(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|^2}\boldsymbol\phi^\wedge+\frac{\|\boldsymbol\phi\|-\sin(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|^3}(\boldsymbol\phi^\wedge)^2.\tag{8}$$

**对数的对应一阶近似**：
$$\mathrm{Log}\big(\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\delta\boldsymbol\phi)\big)\approx\boldsymbol\phi+\mathbf J_r^{-1}(\boldsymbol\phi)\,\delta\boldsymbol\phi.\tag{9}$$
**右雅可比之逆**闭式：
$$\mathbf J_r^{-1}(\boldsymbol\phi)=\mathbf I+\frac12\boldsymbol\phi^\wedge+\left(\frac{1}{\|\boldsymbol\phi\|^2}+\frac{1+\cos(\|\boldsymbol\phi\|)}{2\|\boldsymbol\phi\|\sin(\|\boldsymbol\phi\|)}\right)(\boldsymbol\phi^\wedge)^2.$$
$\mathbf J_r(\boldsymbol\phi)$ 与 $\mathbf J_r^{-1}(\boldsymbol\phi)$ 在 $\|\boldsymbol\phi\|=0$ 时退化为单位阵。

> **[RSS §4 补充：(7)(9) 的来源证明]** RSS 补充材料 §4 给出 (7)(9) 的**推导**（TRO 正文直接引用结果）。
> 右雅可比（亦称 body Jacobian）把参数向量的变化率 $\dot{\boldsymbol\phi}$ 关联到瞬时 body 角速度：
> $$_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}=\mathbf J_r(\boldsymbol\phi)\,\dot{\boldsymbol\phi}.\tag{A.41}$$
> 闭式 $\mathbf J_r(\boldsymbol\phi)$ 同式 (8)（RSS 记 (A.42)）。旋转矩阵 $\mathbf R_{\mathrm{WB}}(\boldsymbol\phi)$（body→world，由旋转向量 $\boldsymbol\phi$ 参数化）与角速度关系[Murray-Li-Sastry 1994]：
> $$\mathbf R_{\mathrm{WB}}^\top\dot{\mathbf R}_{\mathrm{WB}}={}_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}^\wedge.\tag{A.43}$$
> 故旋转矩阵在 $\boldsymbol\phi$ 处的导数：
> $$\dot{\mathbf R}_{\mathrm{WB}}(\boldsymbol\phi)=\mathbf R_{\mathrm{WB}}(\boldsymbol\phi)\big(\mathbf J_r(\boldsymbol\phi)\dot{\boldsymbol\phi}\big)^\wedge.\tag{A.44}$$
> 给定群元右乘的乘法扰动 $\mathrm{Exp}(\delta\boldsymbol\phi)$，问切空间中等价的加法扰动 $\boldsymbol\psi\in\mathfrak{so}(3)$（产生同一复合旋转）：
> $$\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\boldsymbol\psi)=\mathrm{Exp}(\boldsymbol\phi+\delta\boldsymbol\phi).\tag{A.45}$$
> 对两边增量求导，用 (A.44) 并设增量小，得
> $$\delta\boldsymbol\phi\approx\mathbf J_r(\boldsymbol\phi)\,\boldsymbol\psi,\tag{A.46}$$
> 即 $\boldsymbol\psi\approx\mathbf J_r^{-1}(\boldsymbol\phi)\delta\boldsymbol\phi$，代回得
> $$\mathrm{Exp}(\boldsymbol\phi+\delta\boldsymbol\phi)\approx\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}\big(\mathbf J_r(\boldsymbol\phi)\delta\boldsymbol\phi\big).\tag{A.47}$$
> 对数的对应近似（由 BCH 公式在 $\delta\boldsymbol\phi$ 小的假设下直接得到[Barfoot-Furgale 2014]）：
> $$\mathrm{Log}\big(\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\delta\boldsymbol\phi)\big)\approx\boldsymbol\phi+\mathbf J_r^{-1}(\boldsymbol\phi)\,\delta\boldsymbol\phi.\tag{A.48}$$

**指数映射的伴随性质**（“穿过旋转”，**预积分移项的关键**）：
$$\mathbf R\,\mathrm{Exp}(\boldsymbol\phi)\,\mathbf R^\top=\exp(\mathbf R\boldsymbol\phi^\wedge\mathbf R^\top)=\mathrm{Exp}(\mathbf R\boldsymbol\phi)\tag{10}$$
$$\Longleftrightarrow\quad\mathrm{Exp}(\boldsymbol\phi)\,\mathbf R=\mathbf R\,\mathrm{Exp}(\mathbf R^\top\boldsymbol\phi).\tag{11}$$
（即 $\mathrm{Ad}_{\mathbf R}\boldsymbol\phi=\mathbf R\boldsymbol\phi$；式 (11) 用于把一个右乘小旋转“移过”一个旋转矩阵，代价是把扰动用 $\mathbf R^\top$ 旋转。）

### §1.3 SE(3) 与 retraction 选择 [TRO §III-A/§III-C]

**SE(3) 定义**：3D 刚体运动群，$\mathrm{SO}(3)$ 与 $\mathbb R^3$ 的半直积：
$$\mathrm{SE}(3)=\{(\mathbf R,\mathbf p):\mathbf R\in\mathrm{SO}(3),\mathbf p\in\mathbb R^3\}.$$
给定 $\mathbf T_1,\mathbf T_2\in\mathrm{SE}(3)$，群运算 $\mathbf T_1\cdot\mathbf T_2=(\mathbf R_1\mathbf R_2,\mathbf p_1+\mathbf R_1\mathbf p_2)$，逆 $\mathbf T_1^{-1}=(\mathbf R_1^\top,-\mathbf R_1^\top\mathbf p_1)$。SE(3) 的指数/对数映射见[Murray-Li-Sastry]，但**本论文刻意不用**（见下 retraction）。

**SO(3) 上的不确定性建模**（切空间高斯经指数映射推前）：
$$\tilde{\mathbf R}=\mathbf R\,\mathrm{Exp}(\boldsymbol\epsilon),\qquad\boldsymbol\epsilon\sim\mathcal N(\mathbf 0,\boldsymbol\Sigma),\tag{12}$$
$\mathbf R$ 是无噪均值旋转，$\boldsymbol\epsilon$ 是零均值、协方差 $\boldsymbol\Sigma$ 的小扰动。由 $\mathbb R^3$ 高斯积分
$$\int_{\mathbb R^3}p(\boldsymbol\epsilon)\,d\boldsymbol\epsilon=\int_{\mathbb R^3}\alpha\,e^{-\frac12\|\boldsymbol\epsilon\|^2_{\boldsymbol\Sigma}}\,d\boldsymbol\epsilon=1,\tag{13}$$
其中 $\alpha=1/\sqrt{(2\pi)^3\det(\boldsymbol\Sigma)}$，$\|\boldsymbol\epsilon\|^2_{\boldsymbol\Sigma}=\boldsymbol\epsilon^\top\boldsymbol\Sigma^{-1}\boldsymbol\epsilon$ 为马氏距离平方。换元 $\boldsymbol\epsilon=\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})$（$\|\boldsymbol\epsilon\|<\pi$ 时为 (12) 之逆），(13) 变为
$$\int_{\mathrm{SO}(3)}\beta(\tilde{\mathbf R})\,e^{-\frac12\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\|^2_{\boldsymbol\Sigma}}\,d\tilde{\mathbf R}=1,\tag{14}$$
归一化因子 $\beta(\tilde{\mathbf R})=\alpha/|\det(\mathcal J(\tilde{\mathbf R}))|$，$\mathcal J(\tilde{\mathbf R})=\mathbf J_r(\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R}))$（换元的雅可比行列式副产物，见[Barfoot-Furgale]）。由 (14) 读出 SO(3) 上的“高斯”分布：
$$p(\tilde{\mathbf R})=\beta(\tilde{\mathbf R})\,e^{-\frac12\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\|^2_{\boldsymbol\Sigma}}.\tag{15}$$
小协方差时 $\beta\simeq\alpha$（$\mathbf J_r(\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R}))$ 近单位阵）。把 $\beta$ 视作常数，旋转 $\mathbf R$ 在测量 $\tilde{\mathbf R}\sim$ (15) 下的**负对数似然**：
$$\mathcal L(\mathbf R)=\frac12\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\|^2_{\boldsymbol\Sigma}+\text{const}=\frac12\|\mathrm{Log}(\tilde{\mathbf R}^{-1}\mathbf R)\|^2_{\boldsymbol\Sigma}+\text{const},\tag{16}$$
几何上即 $\tilde{\mathbf R}$ 与 $\mathbf R$ 间测地距离（夹角平方）按逆不确定性 $\boldsymbol\Sigma^{-1}$ 加权。

**流形上的高斯-牛顿（lift–solve–retract）** [TRO §III-C]：对 $\min_{\mathbf x\in\mathcal M}f(\mathbf x)$（式 17），不能直接对 $\mathbf x$ 做二次近似（过参数化使法方程欠定；解一般不落回 $\mathcal M$）。引入 **retraction** $\mathcal R_{\mathbf x}$（切空间元 $\delta\mathbf x$ 到 $\mathbf x$ 邻域的双射），重参数化
$$\min_{\mathbf x\in\mathcal M}f(\mathbf x)\ \Rightarrow\ \min_{\delta\mathbf x\in\mathbb R^n}f(\mathcal R_{\mathbf x}(\delta\mathbf x)).\tag{18}$$
GN 在切空间求二次近似得 $\delta\mathbf x^\star$，再 retract 更新：
$$\hat{\mathbf x}\leftarrow\mathcal R_{\hat{\mathbf x}}(\delta\mathbf x^\star).\tag{19}$$
本文采用的 retraction（**这解释了为何只需 SO(3) 指数映射、不需 SE(3) 指数映射**）：
$$\mathcal R_{\mathbf R}(\delta\boldsymbol\phi)=\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi),\quad\delta\boldsymbol\phi\in\mathbb R^3,\tag{20}$$
$$\mathcal R_{\mathbf T}(\delta\boldsymbol\phi,\delta\mathbf p)=(\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi),\ \mathbf p+\mathbf R\,\delta\mathbf p),\quad[\delta\boldsymbol\phi\ \delta\mathbf p]\in\mathbb R^6.\tag{21}$$
> **与本书约定差异（再强调）**：式 (21) 的 SE(3) 切向量排序为 $[\delta\boldsymbol\phi,\delta\mathbf p]$（旋转在前、平移在后），与本书 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）相反；且本源平移更新为 $\mathbf p+\mathbf R\delta\mathbf p$（“body 系平移增量”），而标准 SE(3) 指数 retraction 用 $\mathbf p+\mathbf R\mathbf J_l\delta\boldsymbol\rho$。本源这是“解耦”的 retraction（SO(3)×$\mathbb R^3$ 而非真 SE(3)），综合转换时注意。

---

## §2 MAP / 因子图框架与状态、测量 [TRO §IV]

> 本节给出预积分的“宿主”——MAP 优化框架，供“与 VIO 接口”使用。

### §2.1 状态 [TRO §IV-A]

系统在时刻 $i$ 的状态由 IMU 姿态、位置、速度、bias 描述：
$$\mathbf x_i\doteq[\mathbf R_i,\ \mathbf p_i,\ \mathbf v_i,\ \mathbf b_i].\tag{22}$$
位姿 $(\mathbf R_i,\mathbf p_i)\in\mathrm{SE}(3)$，速度 $\mathbf v_i\in\mathbb R^3$；IMU bias $\mathbf b_i=[\mathbf b_i^g\ \mathbf b_i^a]\in\mathbb R^6$，$\mathbf b_i^g,\mathbf b_i^a\in\mathbb R^3$ 为陀螺、加计 bias。设 $\mathcal K_k$ 为到时刻 $k$ 的全部关键帧集合，待估全状态：
$$\mathcal X_k\doteq\{\mathbf x_i\}_{i\in\mathcal K_k}.\tag{23}$$
本文用**无结构（structureless）**法（§VII），3D 路标不入状态（但框架可平凡推广到也估路标与相机内外参）。

### §2.2 测量 [TRO §IV-B]

相机在关键帧 $i$ 的图像测量记 $\mathcal C_i$（可含多路标 $l$ 的多观测 $\mathbf z_{il}$，记 $l\in\mathcal C_i$）。两连续关键帧 $i,j$ 间采集的 IMU 测量集记 $\mathcal I_{ij}$（视 IMU 率与关键帧频率，可含几个到上百个测量，见图 4）。到 $k$ 的全部测量：
$$\mathcal Z_k\doteq\{\mathcal C_i,\mathcal I_{ij}\}_{(i,j)\in\mathcal K_k}.\tag{24}$$

### §2.3 因子图与 MAP [TRO §IV-C]

变量 $\mathcal X_k$ 在视觉/惯性测量 $\mathcal Z_k$ 与先验 $p(\mathcal X_0)$ 下的后验：
$$p(\mathcal X_k|\mathcal Z_k)\overset{(a)}{\propto}p(\mathcal X_0)p(\mathcal Z_k|\mathcal X_k)=p(\mathcal X_0)\!\!\prod_{(i,j)\in\mathcal K_k}\!\!p(\mathcal C_i,\mathcal I_{ij}|\mathcal X_k)\overset{(b)}{=}p(\mathcal X_0)\!\!\prod_{(i,j)\in\mathcal K_k}\!\!p(\mathcal I_{ij}|\mathbf x_i,\mathbf x_j)\!\prod_{i\in\mathcal K_k}\prod_{l\in\mathcal C_i}\!p(\mathbf z_{il}|\mathbf x_i).\tag{25}$$
(a)(b) 由测量间独立性与马尔可夫性。MAP 估计 $\mathcal X_k^\star$ 即 (25) 之最大、等价于负对数后验之最小。零均值高斯噪声下，负对数后验＝残差平方和：
$$\mathcal X_k^\star=\arg\min_{\mathcal X_k}-\log_e p(\mathcal X_k|\mathcal Z_k)=\arg\min_{\mathcal X_k}\ \|\mathbf r_0\|^2_{\boldsymbol\Sigma_0}+\sum_{(i,j)\in\mathcal K_k}\|\mathbf r_{\mathcal I_{ij}}\|^2_{\boldsymbol\Sigma_{ij}}+\sum_{i\in\mathcal K_k}\sum_{l\in\mathcal C_i}\|\mathbf r_{\mathcal C_{il}}\|^2_{\boldsymbol\Sigma_C}.\tag{26}$$
$\mathbf r_0,\mathbf r_{\mathcal I_{ij}},\mathbf r_{\mathcal C_{il}}$ 为先验/IMU/视觉残差，$\boldsymbol\Sigma_0,\boldsymbol\Sigma_{ij},\boldsymbol\Sigma_C$ 为相应协方差。后续目标即给出这些残差与协方差的表达式。

---

## §3 IMU 测量模型与运动积分 [TRO §V]

### §3.1 IMU 测量模型与误差模型 [TRO §V 开头]

IMU 含 3 轴加速度计与 3 轴陀螺，测量传感器相对惯性系的角速率与加速度。测量 $_{\mathrm B}\tilde{\mathbf a}(t)$、$_{\mathrm B}\tilde{\boldsymbol\omega}_{\mathrm{WB}}(t)$ 受**加性白噪声 $\boldsymbol\eta$** 与**缓变 bias $\mathbf b$** 污染：

**陀螺测量模型**：
$$_{\mathrm B}\tilde{\boldsymbol\omega}_{\mathrm{WB}}(t)={}_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}(t)+\mathbf b^g(t)+\boldsymbol\eta^g(t).\tag{27}$$
**加速度计测量模型**：
$$_{\mathrm B}\tilde{\mathbf a}(t)=\mathbf R_{\mathrm{WB}}^\top(t)\big(_{\mathrm W}\mathbf a(t)-_{\mathrm W}\mathbf g\big)+\mathbf b^a(t)+\boldsymbol\eta^a(t).\tag{28}$$
记号：前缀 $_{\mathrm B}$ 表该量在 B 系表达（图 2）；IMU 位姿由 $\{\mathbf R_{\mathrm{WB}},_{\mathrm W}\mathbf p\}$ 描述（把点从 B 映到 W）；$_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}(t)\in\mathbb R^3$ 是 B 相对 W 的瞬时角速度在 B 系表达；$_{\mathrm W}\mathbf a(t)\in\mathbb R^3$ 是传感器加速度；$_{\mathrm W}\mathbf g$ 是世界系重力向量。**忽略地球自转**（即设 W 为惯性系）。

> **物理意义注解**：加计测的是“比力”——即真加速度减重力，再旋到 body 系。这正是为何运动学积分（式 31）里重力以 $+\mathbf g\Delta t$ / $+\tfrac12\mathbf g\Delta t^2$ 出现：从 (28) 解出 $_{\mathrm W}\mathbf a=\mathbf R_{\mathrm{WB}}(_{\mathrm B}\tilde{\mathbf a}-\mathbf b^a-\boldsymbol\eta^a)+_{\mathrm W}\mathbf g$。

### §3.2 连续时间运动学 [TRO §V，式 29]

刻画 B 位姿与速度演化的运动学模型[Murray-Li-Sastry; 49,53]：
$$\dot{\mathbf R}_{\mathrm{WB}}=\mathbf R_{\mathrm{WB}}\,{}_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}^\wedge,\qquad{}_{\mathrm W}\dot{\mathbf v}={}_{\mathrm W}\mathbf a,\qquad{}_{\mathrm W}\dot{\mathbf p}={}_{\mathrm W}\mathbf v.\tag{29}$$

### §3.3 离散时间（欧拉）积分 [TRO §V，式 30–31]

对 (29) 在 $[t,t+\Delta t]$ 积分，得状态更新：
$$\mathbf R_{\mathrm{WB}}(t+\Delta t)=\mathbf R_{\mathrm{WB}}(t)\,\mathrm{Exp}\!\left(\int_t^{t+\Delta t}{}_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}(\tau)\,d\tau\right),$$
$$_{\mathrm W}\mathbf v(t+\Delta t)={}_{\mathrm W}\mathbf v(t)+\int_t^{t+\Delta t}{}_{\mathrm W}\mathbf a(\tau)\,d\tau,$$
$$_{\mathrm W}\mathbf p(t+\Delta t)={}_{\mathrm W}\mathbf p(t)+\int_t^{t+\Delta t}{}_{\mathrm W}\mathbf v(\tau)\,d\tau+\iint_t^{t+\Delta t}{}_{\mathrm W}\mathbf a(\tau)\,d\tau^2.$$
设 $_{\mathrm W}\mathbf a$、$_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}$ 在 $[t,t+\Delta t]$ 内**恒定**（零阶保持）：
$$\mathbf R_{\mathrm{WB}}(t+\Delta t)=\mathbf R_{\mathrm{WB}}(t)\,\mathrm{Exp}\big({}_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}(t)\Delta t\big),$$
$$_{\mathrm W}\mathbf v(t+\Delta t)={}_{\mathrm W}\mathbf v(t)+{}_{\mathrm W}\mathbf a(t)\Delta t,$$
$$_{\mathrm W}\mathbf p(t+\Delta t)={}_{\mathrm W}\mathbf p(t)+{}_{\mathrm W}\mathbf v(t)\Delta t+\frac12{}_{\mathrm W}\mathbf a(t)\Delta t^2.\tag{30}$$
用 (27)–(28) 把 $_{\mathrm W}\mathbf a$、$_{\mathrm B}\boldsymbol\omega_{\mathrm{WB}}$ 写成测量的函数（**离散噪声** $\boldsymbol\eta^{gd},\boldsymbol\eta^{ad}$），(30) 成为（**丢掉坐标系下标以便阅读，下文同**）：
$$\mathbf R(t+\Delta t)=\mathbf R(t)\,\mathrm{Exp}\big((\tilde{\boldsymbol\omega}(t)-\mathbf b^g(t)-\boldsymbol\eta^{gd}(t))\Delta t\big),$$
$$\mathbf v(t+\Delta t)=\mathbf v(t)+\mathbf g\Delta t+\mathbf R(t)\big(\tilde{\mathbf a}(t)-\mathbf b^a(t)-\boldsymbol\eta^{ad}(t)\big)\Delta t,$$
$$\mathbf p(t+\Delta t)=\mathbf p(t)+\mathbf v(t)\Delta t+\frac12\mathbf g\Delta t^2+\frac12\mathbf R(t)\big(\tilde{\mathbf a}(t)-\mathbf b^a(t)-\boldsymbol\eta^{ad}(t)\big)\Delta t^2.\tag{31}$$

> **积分近似说明（源原文）**：(31) 的速度/位置数值积分假设积分区间内姿态 $\mathbf R(t)$ 恒定，对非零转速测量这**不是** (29) 的精确解；高率 IMU 缓解此误差。采用 (31) 因其简单、便于建模与不确定性传播；慢率 IMU 可考虑高阶数值积分[54–57]。

**离散↔连续噪声协方差关系**（关键，供协方差传播用）：离散时间噪声 $\boldsymbol\eta^{gd}$ 的协方差与采样率及连续时间谱噪声 $\boldsymbol\eta^g$ 的关系为
$$\mathrm{Cov}(\boldsymbol\eta^{gd}(t))=\frac{1}{\Delta t}\,\mathrm{Cov}(\boldsymbol\eta^g(t)),$$
$\boldsymbol\eta^{ad}$ 同理[58, Appendix]。

---

## §4 流形上 IMU 预积分 [TRO §VI；RSS §1]

> **本章核心**。从 (31) 单步递推到两关键帧间的相对增量，分离 bias 与噪声，导出预积分测量模型。

### §4.1 迭代积分（高率会暴增状态） [TRO §VI 开头，式 32]

(31) 关联 $t$ 与 $t+\Delta t$ 两态，若直接入因子图须在每个 IMU 测量处加状态[37]。对两连续关键帧 $k=i$ 到 $k=j$ 间所有 $\Delta t$ 区间迭代 (31)（引入简写 $\Delta t_{ij}\doteq\sum_{k=i}^{j-1}\Delta t$，$(\cdot)_i\doteq(\cdot)(t_i)$）：
$$\mathbf R_j=\mathbf R_i\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b_k^g-\boldsymbol\eta_k^{gd})\Delta t\big),$$
$$\mathbf v_j=\mathbf v_i+\mathbf g\Delta t_{ij}+\sum_{k=i}^{j-1}\mathbf R_k(\tilde{\mathbf a}_k-\mathbf b_k^a-\boldsymbol\eta_k^{ad})\Delta t,$$
$$\mathbf p_j=\mathbf p_i+\sum_{k=i}^{j-1}\Big[\mathbf v_k\Delta t+\frac12\mathbf g\Delta t^2+\frac12\mathbf R_k(\tilde{\mathbf a}_k-\mathbf b_k^a-\boldsymbol\eta_k^{ad})\Delta t^2\Big].\tag{32}$$
(32) 的缺点：积分须在时刻 $t_i$ 的线性化点（如 $\mathbf R_i$）改变时**整段重算**（$\mathbf R_i$ 变→所有未来 $\mathbf R_k$ 变→须重评求和与连乘）[24]。

### §4.2 相对运动增量定义（与 $i$ 态无关） [TRO §VI-A，式 33；RSS (28)-(29)-(30)]

为避免重算，跟随[Lupton-Sukkarieh 2012]定义**与 $t_i$ 处位姿、速度无关**的相对增量：
$$\Delta\mathbf R_{ij}\doteq\mathbf R_i^\top\mathbf R_j=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b_k^g-\boldsymbol\eta_k^{gd})\Delta t\big),$$
$$\Delta\mathbf v_{ij}\doteq\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})=\sum_{k=i}^{j-1}\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b_k^a-\boldsymbol\eta_k^{ad})\Delta t,$$
$$\Delta\mathbf p_{ij}\doteq\mathbf R_i^\top\Big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\frac12\sum_{k=i}^{j-1}\mathbf g\Delta t^2\Big)=\sum_{k=i}^{j-1}\Big[\Delta\mathbf v_{ik}\Delta t+\frac12\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b_k^a-\boldsymbol\eta_k^{ad})\Delta t^2\Big].\tag{33}$$
其中 $\Delta\mathbf R_{ik}\doteq\mathbf R_i^\top\mathbf R_k$，$\Delta\mathbf v_{ik}\doteq\mathbf R_i^\top(\mathbf v_k-\mathbf v_i-\mathbf g\Delta t_{ik})$。

> **重要语义说明（源原文）**：与“delta”旋转 $\Delta\mathbf R_{ij}$ 不同，$\Delta\mathbf v_{ij}$、$\Delta\mathbf p_{ij}$ **并非**真实物理上的速度/位置变化，而是被刻意定义为使 (33) 右端**与 $t_i$ 态及重力效应无关**的量。这样右端可直接由两关键帧间的惯性测量算出。

**恒定 bias 假设**（两关键帧间）：
$$\mathbf b_i^g=\mathbf b_{i+1}^g=\dots=\mathbf b_{j-1}^g,\qquad\mathbf b_i^a=\mathbf b_{i+1}^a=\dots=\mathbf b_{j-1}^a.\tag{34}$$
处理 bias 分两步：§VI-A 先设 $\mathbf b_i$ 已知；§VI-C 再处理 bias 估计变化时如何免重积分。

### §4.3 分离噪声：预积分测量模型 [TRO §VI-A，式 35–38]

**(i) 旋转增量 $\Delta\mathbf R_{ij}$**：用一阶近似 (7)（旋转噪声“小”）并用 (11) 把噪声“移到末尾”：
$$\Delta\mathbf R_{ij}\overset{(7)}{\simeq}\prod_{k=i}^{j-1}\Big[\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t\big)\,\mathrm{Exp}\big(-\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t\big)\Big]\overset{(11)}{=}\Delta\tilde{\mathbf R}_{ij}\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t\big)\doteq\Delta\tilde{\mathbf R}_{ij}\,\mathrm{Exp}(-\delta\boldsymbol\phi_{ij}),\tag{35}$$
其中 $\mathbf J_r^k\doteq\mathbf J_r((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t)$。定义**预积分旋转测量**
$$\boxed{\Delta\tilde{\mathbf R}_{ij}\doteq\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t\big)}$$
及其噪声 $\delta\boldsymbol\phi_{ij}$（下节分析）。
> 推导细节：从 (7) 拆出 $\mathrm{Exp}(\mathbf a\Delta t)\mathrm{Exp}(-\mathbf J_r\boldsymbol\eta\Delta t)$；用 (11) 把每个噪声项 $\mathrm{Exp}(-\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t)$ 依次穿过其右侧的 $\mathrm{Exp}((\tilde{\boldsymbol\omega}_{\cdot}-\mathbf b_i^g)\Delta t)$ 直到末尾，每穿过一项扰动被左乘 $\Delta\tilde{\mathbf R}_{k+1\,j}^\top$（即 $\mathbf R^\top\boldsymbol\phi$ 中的 $\mathbf R^\top$）。

**(ii) 速度增量 $\Delta\mathbf v_{ij}$**：把 (35) 代回 (33) 的 $\Delta\mathbf v_{ij}$，对 $\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})$ 用一阶近似 (4)，丢高阶噪声：
$$\Delta\mathbf v_{ij}\overset{(4)}{\simeq}\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\mathbf I-\delta\boldsymbol\phi_{ik}^\wedge)(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t-\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t\overset{(2)}{=}\Delta\tilde{\mathbf v}_{ij}+\sum_{k=i}^{j-1}\Big[\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t-\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t\Big]\doteq\Delta\tilde{\mathbf v}_{ij}-\delta\mathbf v_{ij},\tag{36}$$
（用了 (2)：$-(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}=+\delta\boldsymbol\phi_{ik}^\wedge(\tilde{\mathbf a}_k-\mathbf b_i^a)$ 反号互换）。定义**预积分速度测量**
$$\boxed{\Delta\tilde{\mathbf v}_{ij}\doteq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t}$$
及其噪声 $\delta\mathbf v_{ij}$。

**(iii) 位置增量 $\Delta\mathbf p_{ij}$**：把 (35)(36) 代回 (33) 的 $\Delta\mathbf p_{ij}$，用 (4)：
$$\Delta\mathbf p_{ij}\overset{(4)}{\simeq}\sum_{k=i}^{j-1}\Big[(\Delta\tilde{\mathbf v}_{ik}-\delta\mathbf v_{ik})\Delta t+\frac12\Delta\tilde{\mathbf R}_{ik}(\mathbf I-\delta\boldsymbol\phi_{ik}^\wedge)(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t^2-\frac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t^2\Big]$$
$$\overset{(2)}{=}\Delta\tilde{\mathbf p}_{ij}+\sum_{k=i}^{j-1}\Big[-\delta\mathbf v_{ik}\Delta t+\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2-\frac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t^2\Big]\doteq\Delta\tilde{\mathbf p}_{ij}-\delta\mathbf p_{ij},\tag{37}$$
定义**预积分位置测量** $\Delta\tilde{\mathbf p}_{ij}$ 及噪声 $\delta\mathbf p_{ij}$。其中
$$\boxed{\Delta\tilde{\mathbf p}_{ij}\doteq\sum_{k=i}^{j-1}\Big[\Delta\tilde{\mathbf v}_{ik}\Delta t+\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t^2\Big]}$$
（由 RSS (A.11) / TRO 附录给出的显式形式）。

**预积分测量模型**（把 (35)(36)(37) 代回 (33) 原定义；注意 $\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})^\top=\mathrm{Exp}(\delta\boldsymbol\phi_{ij})$）：
$$\boxed{\begin{aligned}\Delta\tilde{\mathbf R}_{ij}&=\mathbf R_i^\top\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_{ij}),\\\Delta\tilde{\mathbf v}_{ij}&=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})+\delta\mathbf v_{ij},\\\Delta\tilde{\mathbf p}_{ij}&=\mathbf R_i^\top\Big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\frac12\mathbf g\Delta t_{ij}^2\Big)+\delta\mathbf p_{ij}.\end{aligned}}\tag{38}$$
复合测量＝（待估）状态“加上”随机噪声 $[\delta\boldsymbol\phi_{ij}^\top,\delta\mathbf v_{ij}^\top,\delta\mathbf p_{ij}^\top]^\top$。
> **意义（源原文）**：(38) 的好处是，对合适的噪声分布，对数似然定义直截了当：末两行加性高斯噪声→二次型；若 $\delta\boldsymbol\phi_{ij}$ 零均值高斯，则可写 $\Delta\tilde{\mathbf R}_{ij}$ 的负对数似然（形如式 16）。

### §4.4 噪声传播：统计与协方差 [TRO §VI-B，附录 IX-A；RSS §1.1]

目标：求噪声向量
$$\boldsymbol\eta_{ij}^{\Delta}\doteq[\delta\boldsymbol\phi_{ij}^\top,\delta\mathbf v_{ij}^\top,\delta\mathbf p_{ij}^\top]^\top\sim\mathcal N(\mathbf 0_{9\times1},\boldsymbol\Sigma_{ij}).\tag{39}$$
协方差 $\boldsymbol\Sigma_{ij}$ 至关重要（其逆作 MAP 加权，式 26）。

**(i) 旋转噪声 $\delta\boldsymbol\phi_{ij}$**：由 (35)，
$$\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})=\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t\big).\tag{40}$$
两边取 Log 并变号：
$$\delta\boldsymbol\phi_{ij}=-\mathrm{Log}\!\left(\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t\big)\right).\tag{41}$$
反复用一阶近似 (9)（$\boldsymbol\eta_k^{gd}$ 与 $\delta\boldsymbol\phi_{ij}$ 均小旋转噪声，右雅可比近单位阵）：
$$\boxed{\delta\boldsymbol\phi_{ij}\simeq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t}\tag{42}$$
至一阶，$\delta\boldsymbol\phi_{ij}$ 零均值高斯（零均值噪声 $\boldsymbol\eta_k^{gd}$ 的线性组合），使旋转测量模型 (38) 恰为 (12) 形式。

**(ii) 速度/位置噪声**（$\boldsymbol\eta_k^{ad}$ 与 $\delta\boldsymbol\phi_{ij}$ 的线性组合，故亦零均值高斯）：
$$\delta\mathbf v_{ij}\simeq\sum_{k=i}^{j-1}\Big[-\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t+\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t\Big],\tag{43a}$$
$$\delta\mathbf p_{ij}\simeq\sum_{k=i}^{j-1}\Big[\delta\mathbf v_{ik}\Delta t-\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2+\frac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t^2\Big].\tag{43b}$$
（TRO 统编为式 43。）(42)-(43) 把预积分噪声 $\boldsymbol\eta_{ij}^\Delta$ 表为 IMU 测量噪声 $\boldsymbol\eta_k^d\doteq[\boldsymbol\eta_k^{gd},\boldsymbol\eta_k^{ad}],k=i,\dots,j-1$ 的线性函数；由 $\boldsymbol\eta_k^d$ 协方差（IMU 规格）经线性传播即得 $\boldsymbol\Sigma_{ij}$。

#### §4.4.1 迭代噪声传播（附录 IX-A / RSS §1.1，完整逐步证明）

> 直接代回 (42) 到 (43) 再传播很繁；改写为**迭代式**更简洁、利于在线。RSS (A.1)–(A.10) 给出完整推导。

把 (42)(43)（RSS (A.2)）的旋转噪声 $\delta\boldsymbol\phi_{ij}$ 拆出末项 $k=j-1$ 并重排（利用 $\Delta\tilde{\mathbf R}_{j-1\,j-1}=\mathbf I$，$\Delta\tilde{\mathbf R}_{k+1\,j}=\Delta\tilde{\mathbf R}_{k+1\,j-1}\Delta\tilde{\mathbf R}_{j-1\,j}$）：
$$\delta\boldsymbol\phi_{ij}\simeq\sum_{k=i}^{j-2}\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t+\Delta\tilde{\mathbf R}_{jj}^\top\mathbf J_r^{j-1}\boldsymbol\eta_{j-1}^{gd}\Delta t=\sum_{k=i}^{j-2}(\Delta\tilde{\mathbf R}_{k+1\,j-1}\Delta\tilde{\mathbf R}_{j-1\,j})^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t+\mathbf J_r^{j-1}\boldsymbol\eta_{j-1}^{gd}\Delta t$$
$$=\Delta\tilde{\mathbf R}_{j-1\,j}^\top\underbrace{\sum_{k=i}^{j-2}\Delta\tilde{\mathbf R}_{k+1\,j-1}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t}_{=\delta\boldsymbol\phi_{i\,j-1}}+\mathbf J_r^{j-1}\boldsymbol\eta_{j-1}^{gd}\Delta t=\Delta\tilde{\mathbf R}_{j-1\,j}^\top\,\delta\boldsymbol\phi_{i\,j-1}+\mathbf J_r^{j-1}\boldsymbol\eta_{j-1}^{gd}\Delta t.\tag{A.3}$$
同理对 $\delta\mathbf v_{ij}$ 拆末项：
$$\delta\mathbf v_{ij}=\delta\mathbf v_{i\,j-1}-\Delta\tilde{\mathbf R}_{i\,j-1}(\tilde{\mathbf a}_{j-1}-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{i\,j-1}\Delta t+\Delta\tilde{\mathbf R}_{i\,j-1}\boldsymbol\eta_{j-1}^{ad}\Delta t.\tag{A.4}$$
对 $\delta\mathbf p_{ij}$（注意 $\delta\mathbf p_{ij}$ 可写成 $\delta\mathbf v$ 的函数）：
$$\delta\mathbf p_{ij}=\delta\mathbf p_{i\,j-1}+\delta\mathbf v_{i\,j-1}\Delta t-\frac12\Delta\tilde{\mathbf R}_{i\,j-1}(\tilde{\mathbf a}_{j-1}-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{i\,j-1}\Delta t^2+\frac12\Delta\tilde{\mathbf R}_{i\,j-1}\boldsymbol\eta_{j-1}^{ad}\Delta t^2.\tag{A.5}$$
合并 (A.3)(A.4)(A.5)，**迭代形式**（$k=i,\dots,j$，初值 $\delta\boldsymbol\phi_{ii}=\delta\mathbf v_{ii}=\delta\mathbf p_{ii}=\mathbf 0_3$）：
$$\begin{aligned}\delta\boldsymbol\phi_{i\,k+1}&=\Delta\tilde{\mathbf R}_{k\,k+1}^\top\delta\boldsymbol\phi_{ik}+\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t,\\\delta\mathbf v_{i\,k+1}&=\delta\mathbf v_{ik}-\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t+\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t,\\\delta\mathbf p_{i\,k+1}&=\delta\mathbf p_{ik}+\delta\mathbf v_{ik}\Delta t-\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2+\frac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta_k^{ad}\Delta t^2.\end{aligned}\tag{A.6}$$
**矩阵形式**（记 $\boldsymbol\eta_{ik}^\Delta=[\delta\boldsymbol\phi_{ik},\delta\mathbf v_{ik},\delta\mathbf p_{ik}]$，$\boldsymbol\eta_k^d=[\boldsymbol\eta_k^{gd},\boldsymbol\eta_k^{ad}]$）：
$$\begin{bmatrix}\delta\boldsymbol\phi_{i\,k+1}\\\delta\mathbf v_{i\,k+1}\\\delta\mathbf p_{i\,k+1}\end{bmatrix}=\underbrace{\begin{bmatrix}\Delta\tilde{\mathbf R}_{k\,k+1}^\top&\mathbf 0_{3\times3}&\mathbf 0_{3\times3}\\-\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\Delta t&\mathbf I_{3\times3}&\mathbf 0_{3\times3}\\-\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\Delta t^2&\mathbf I_{3\times3}\Delta t&\mathbf I_{3\times3}\end{bmatrix}}_{\mathbf A}\begin{bmatrix}\delta\boldsymbol\phi_{ik}\\\delta\mathbf v_{ik}\\\delta\mathbf p_{ik}\end{bmatrix}+\underbrace{\begin{bmatrix}\mathbf J_r^k\Delta t&\mathbf 0_{3\times3}\\\mathbf 0_{3\times3}&\Delta\tilde{\mathbf R}_{ik}\Delta t\\\mathbf 0_{3\times3}&\frac12\Delta\tilde{\mathbf R}_{ik}\Delta t^2\end{bmatrix}}_{\mathbf B}\begin{bmatrix}\boldsymbol\eta_k^{gd}\\\boldsymbol\eta_k^{ad}\end{bmatrix}\tag{A.7}$$
简写：
$$\boldsymbol\eta_{i\,k+1}^\Delta=\mathbf A\,\boldsymbol\eta_{ik}^\Delta+\mathbf B\,\boldsymbol\eta_k^d.\tag{A.8 / 62}$$
（TRO 正文式 62 记作 $\boldsymbol\eta_{ij}^\Delta=\mathbf A_{j-1}\boldsymbol\eta_{i\,j-1}^\Delta+\mathbf B_{j-1}\boldsymbol\eta_{j-1}^d$。）由线性模型 (A.8) 与原始 IMU 噪声协方差 $\boldsymbol\Sigma_\eta\in\mathbb R^{6\times6}$，**协方差迭代传播**：
$$\boxed{\boldsymbol\Sigma_{i\,k+1}=\mathbf A\,\boldsymbol\Sigma_{ik}\,\mathbf A^\top+\mathbf B\,\boldsymbol\Sigma_\eta\,\mathbf B^\top}\tag{A.9 / 63}$$
初值 $\boldsymbol\Sigma_{ii}=\mathbf 0_{9\times9}$。（TRO 式 63：$\boldsymbol\Sigma_{ij}=\mathbf A_{j-1}\boldsymbol\Sigma_{i\,j-1}\mathbf A_{j-1}^\top+\mathbf B_{j-1}\boldsymbol\Sigma_\eta\mathbf B_{j-1}^\top$。）新测量到来时仅需更新 $\boldsymbol\Sigma_{ij}$，无须重算。

**预积分测量本身的迭代**（RSS (A.10)，由 (31) 的 (28)-(29)-(30) 直接得；对应 TRO 的 $\Delta\tilde{\mathbf R},\Delta\tilde{\mathbf v},\Delta\tilde{\mathbf p}$ 在线累加）：
$$\begin{aligned}\Delta\tilde{\mathbf R}_{i\,k+1}&=\Delta\tilde{\mathbf R}_{ik}\,\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t\big),\\\Delta\tilde{\mathbf v}_{i\,k+1}&=\Delta\tilde{\mathbf v}_{ik}+\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t,\\\Delta\tilde{\mathbf p}_{i\,k+1}&=\Delta\tilde{\mathbf p}_{ik}+\Delta\tilde{\mathbf v}_{ik}\Delta t+\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t^2.\end{aligned}\tag{A.10}$$

### §4.5 bias 更新的一阶修正 [TRO §VI-C，附录 IX-B；RSS §1.2，完整证明]

> §VI-A 假设积分用的 bias $\{\bar{\mathbf b}_i^a,\bar{\mathbf b}_i^g\}$ 正确且不变。实际优化中 bias 估计会变小量 $\delta\mathbf b$。重算 delta 代价高；改用**一阶展开**。

记在给定 bias 估计 $\bar{\mathbf b}_i=[\bar{\mathbf b}_i^g\ \bar{\mathbf b}_i^a]$ 处算出的预积分量为（式 64）
$$\Delta\bar{\mathbf R}_{ij}\doteq\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i),\quad\Delta\bar{\mathbf v}_{ij}\doteq\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}_i),\quad\Delta\bar{\mathbf p}_{ij}\doteq\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}_i).\tag{64}$$
**bias 更新公式**（给定 $\mathbf b\leftarrow\bar{\mathbf b}+\delta\mathbf b$，一阶展开；TRO 式 44）：
$$\boxed{\begin{aligned}\Delta\tilde{\mathbf R}_{ij}(\mathbf b_i^g)&\simeq\Delta\bar{\mathbf R}_{ij}\,\mathrm{Exp}\Big(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g\Big),\\\Delta\tilde{\mathbf v}_{ij}(\mathbf b_i^g,\mathbf b_i^a)&\simeq\Delta\bar{\mathbf v}_{ij}+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b_i^a,\\\Delta\tilde{\mathbf p}_{ij}(\mathbf b_i^g,\mathbf b_i^a)&\simeq\Delta\bar{\mathbf p}_{ij}+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b_i^a.\end{aligned}}\tag{44}$$
类似[Lupton-Sukkarieh 2012]的 bias 修正，但**直接在 SO(3) 上**操作。雅可比 $\{\partial\Delta\bar{\mathbf R}_{ij}/\partial\mathbf b^g,\dots\}$ 在积分时刻的 bias 估计 $\bar{\mathbf b}_i$ 处计算，**恒定、可在预积分时预计算并增量更新**。

#### §4.5.1 旋转 bias 修正的完整推导（RSS (A.13)-(A.14)）

新估计 $\hat{\mathbf b}_i\leftarrow\bar{\mathbf b}_i+\delta\mathbf b_i$。从（式 65 / RSS (A.12)）
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\hat{\mathbf b}_i^g)\Delta t\big)=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}_i^g-\delta\mathbf b_i^g)\Delta t\big).\tag{65}$$
代 $\hat{\mathbf b}_i=\bar{\mathbf b}_i+\delta\mathbf b_i$，对每因子用一阶近似 (7)（设 $\delta\mathbf b_i$ 小，$\mathbf J_r^k\doteq\mathbf J_r((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}_i^g)\Delta t)$）：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)\simeq\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}_i^g)\Delta t\big)\,\mathrm{Exp}\big(-\mathbf J_r^k\delta\mathbf b_i^g\Delta t\big).\tag{66}$$
用 (11) 把含 $\delta\mathbf b$ 项移到末尾（同 §4.3 (35) 套路）：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)=\Delta\bar{\mathbf R}_{ij}\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\mathbf J_r^k\delta\mathbf b_i^g\Delta t\big),\tag{67 / A.13}$$
（用了定义 $\Delta\bar{\mathbf R}_{ij}=\prod_{k=i}^{j-1}\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}_i^g)\Delta t)$）。反复用 (7)（$\delta\mathbf b_i^g$ 小，右雅可比近单位阵）：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)\simeq\Delta\bar{\mathbf R}_{ij}\,\mathrm{Exp}\Big(-\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\mathbf J_r^k\Delta t\,\delta\mathbf b_i^g\Big)=\Delta\bar{\mathbf R}_{ij}\,\mathrm{Exp}\Big(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g\Big).\tag{68 / A.14}$$
读出
$$\boxed{\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}=-\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\mathbf J_r^k\Delta t.}$$

#### §4.5.2 速度 bias 修正的完整推导（RSS (A.15)-(A.18)）

把 (68) 代回 $\Delta\tilde{\mathbf v}_{ij}(\hat{\mathbf b}_i)$（式 69）：
$$\Delta\tilde{\mathbf v}_{ij}(\hat{\mathbf b}_i)=\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\hat{\mathbf b}_i)(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a-\delta\mathbf b_i^a)\Delta t\overset{(68)}{\simeq}\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\,\mathrm{Exp}\Big(\frac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\delta\mathbf b_i^g\Big)(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a-\delta\mathbf b_i^a)\Delta t.\tag{69 / A.15}$$
用一阶近似 (4)（$\delta\mathbf b_i^g$ 小）：
$$\Delta\tilde{\mathbf v}_{ij}(\hat{\mathbf b}_i)\overset{(4)}{\simeq}\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Big(\mathbf I+\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\delta\mathbf b_i^g\big)^\wedge\Big)(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a-\delta\mathbf b_i^a)\Delta t.\tag{A.16}$$
展开、丢高阶项：
$$\overset{(a)}{\simeq}\Delta\bar{\mathbf v}_{ij}-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Delta t\,\delta\mathbf b_i^a+\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\delta\mathbf b_i^g\big)^\wedge(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)\Delta t\tag{A.17}$$
（(a) 用了 $\Delta\bar{\mathbf v}_{ij}=\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)\Delta t$）。用性质 (2) 把 $(\cdot)^\wedge(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)=-(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)^\wedge(\cdot)$：
$$\Delta\tilde{\mathbf v}_{ij}(\hat{\mathbf b}_i)=\Delta\bar{\mathbf v}_{ij}-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Delta t\,\delta\mathbf b_i^a-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)^\wedge\frac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\Delta t\,\delta\mathbf b_i^g=\Delta\bar{\mathbf v}_{ij}+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b_i^a+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g.\tag{A.18}$$

#### §4.5.3 位置 bias 修正（RSS (A.19)）

同法对 $\Delta\tilde{\mathbf p}_{ij}(\hat{\mathbf b}_i)$，用 (A.14)、(4)、(2)：
$$\Delta\tilde{\mathbf p}_{ij}(\hat{\mathbf b}_i)=\Delta\bar{\mathbf p}_{ij}+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b_i^a+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g.\tag{A.19}$$

#### §4.5.4 bias 修正雅可比汇总（TRO 附录 IX-B / RSS (A.20)）

$$\boxed{\begin{aligned}\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}&=-\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\,\mathbf J_r^k\,\Delta t,\\[4pt]\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}&=-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\,\Delta t,\\[4pt]\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}&=-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)^\wedge\,\frac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\,\Delta t,\\[4pt]\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}&=\sum_{k=i}^{j-1}\Big[\frac{\partial\Delta\bar{\mathbf v}_{ik}}{\partial\mathbf b^a}\Delta t-\frac12\Delta\bar{\mathbf R}_{ik}\Delta t^2\Big],\\[4pt]\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}&=\sum_{k=i}^{j-1}\Big[\frac{\partial\Delta\bar{\mathbf v}_{ik}}{\partial\mathbf b^g}\Delta t-\frac12\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a)^\wedge\frac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\Delta t^2\Big].\end{aligned}}\tag{IX-B / A.20}$$
其中 $\mathbf J_r^k=\mathbf J_r((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}_i^g)\Delta t)$。这些雅可比可随测量到来**增量计算**（同 §4.4.1 的迭代套路）。
> **实现注**：$\partial\Delta\bar{\mathbf R}_{ij}/\partial\mathbf b^g$ 的结构与 (A.2) 中乘噪声的项本质相同，故可用同样迭代法预计算。位置雅可比依赖速度雅可比（$\partial\Delta\bar{\mathbf v}_{ik}/\partial\mathbf b^a$、$\partial\Delta\bar{\mathbf v}_{ik}/\partial\mathbf b^g$），故按 $k$ 递推时一并维护。

---

## §5 IMU 预积分因子与残差 [TRO §VI-D；RSS §2]

### §5.1 预积分 IMU 残差 [TRO §VI-D，式 45；RSS (A.21)]

给定 (38) 测量模型与零均值高斯噪声（协方差 $\boldsymbol\Sigma_{ij}$，至一阶式 39），IMU 残差 $\mathbf r_{\mathcal I_{ij}}=[\mathbf r_{\Delta\mathbf R_{ij}}^\top,\mathbf r_{\Delta\mathbf v_{ij}}^\top,\mathbf r_{\Delta\mathbf p_{ij}}^\top]^\top\in\mathbb R^9$（已含式 44 的 bias 更新）：
$$\boxed{\begin{aligned}\mathbf r_{\Delta\mathbf R_{ij}}&=\mathrm{Log}\!\left(\Big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\,\mathrm{Exp}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g\big)\Big)^\top\mathbf R_i^\top\mathbf R_j\right),\\[4pt]\mathbf r_{\Delta\mathbf v_{ij}}&=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-\Big[\Delta\bar{\mathbf v}_{ij}(\bar{\mathbf b}_i^g,\bar{\mathbf b}_i^a)+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\Big],\\[4pt]\mathbf r_{\Delta\mathbf p_{ij}}&=\mathbf R_i^\top\Big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\Big)-\Big[\Delta\bar{\mathbf p}_{ij}(\bar{\mathbf b}_i^g,\bar{\mathbf b}_i^a)+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\Big].\end{aligned}}\tag{45 / A.21}$$
残差对应的协方差为 $\boldsymbol\Sigma_{ij}$（式 39），在 MAP (26) 中以 $\|\mathbf r_{\mathcal I_{ij}}\|^2_{\boldsymbol\Sigma_{ij}}$ 出现。
> 旋转残差几何意义：测量预测的相对旋转 $\Delta\tilde{\mathbf R}_{ij}$（含 bias 修正）与状态给出的 $\mathbf R_i^\top\mathbf R_j$ 之差，经 Log 映到 $\mathbb R^3$（夹角向量）。

### §5.2 残差的解析雅可比（lift–solve–retract） [TRO §VI-D + 附录 IX-C；RSS §2.1–2.3]

GN 每次迭代用 retraction (21) 重参数化 (45)，再线性化。提升（lifting）即代入 retraction（式 70 / RSS (A.22)-(A.23)）：
$$\begin{aligned}&\mathbf R_i\leftarrow\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i),&&\mathbf R_j\leftarrow\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_j),\\&\mathbf p_i\leftarrow\mathbf p_i+\mathbf R_i\delta\mathbf p_i,&&\mathbf p_j\leftarrow\mathbf p_j+\mathbf R_j\delta\mathbf p_j,\\&\mathbf v_i\leftarrow\mathbf v_i+\delta\mathbf v_i,&&\mathbf v_j\leftarrow\mathbf v_j+\delta\mathbf v_j,\\&\delta\mathbf b_i^g\leftarrow\delta\mathbf b_i^g+\tilde{\delta}\mathbf b_i^g,&&\delta\mathbf b_i^a\leftarrow\delta\mathbf b_i^a+\tilde{\delta}\mathbf b_i^a.\end{aligned}\tag{70}$$
（注：TRO 式 70 第三行 $\mathbf v_j\leftarrow\mathbf v_j+\delta\mathbf v_i$ 处疑为印刷，应为 $\mathbf v_j+\delta\mathbf v_j$，RSS (A.23) 写 $\mathbf v_j\leftarrow\mathbf v_j+\delta\mathbf v_j$。）下面对向量 $\delta\boldsymbol\phi_i,\delta\mathbf p_i,\delta\mathbf v_i,\delta\boldsymbol\phi_j,\delta\mathbf p_j,\delta\mathbf v_j,\tilde{\delta}\mathbf b_i^g,\tilde{\delta}\mathbf b_i^a$ 求雅可比。

#### §5.2.1 $\mathbf r_{\Delta\mathbf p_{ij}}$ 的雅可比（RSS §2.1，逐项）

$\mathbf r_{\Delta\mathbf p_{ij}}$ 关于 $\delta\mathbf b_i^g,\delta\mathbf b_i^a$ 线性、retraction 是向量和，故对 $\tilde{\delta}\mathbf b_i^g,\tilde{\delta}\mathbf b_i^a$ 的雅可比即 $\delta\mathbf b_i^g,\delta\mathbf b_i^a$ 的系数矩阵；$\mathbf R_j,\mathbf v_j$ 不出现，故对 $\delta\boldsymbol\phi_j,\delta\mathbf v_j$ 雅可比为零。其余（记 $C\doteq\Delta\tilde{\mathbf p}_{ij}+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b_i^a$）：
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_i+\mathbf R_i\delta\mathbf p_i)=\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf R_i\delta\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-C=\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_i)+(-\mathbf I_{3})\delta\mathbf p_i,\tag{71}$$
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_j+\mathbf R_j\delta\mathbf p_j)=\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_j)+(\mathbf R_i^\top\mathbf R_j)\delta\mathbf p_j,\tag{72}$$
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf v_i+\delta\mathbf v_i)=\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf v_i)+(-\mathbf R_i^\top\Delta t_{ij})\delta\mathbf v_i,\tag{73}$$
$$\begin{aligned}\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))&=(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-C\\&\overset{(4)}{\simeq}(\mathbf I-\delta\boldsymbol\phi_i^\wedge)\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-C\\&\overset{(2)}{=}\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf R_i)+\Big(\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)\Big)^\wedge\delta\boldsymbol\phi_i.\end{aligned}\tag{74}$$
**汇总**：
$$\begin{aligned}&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\boldsymbol\phi_i}=\Big(\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)\Big)^\wedge,&&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\boldsymbol\phi_j}=\mathbf 0,\\&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf p_i}=-\mathbf I_{3},&&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf p_j}=\mathbf R_i^\top\mathbf R_j,\\&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf v_i}=-\mathbf R_i^\top\Delta t_{ij},&&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf v_j}=\mathbf 0,\\&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\tilde{\delta}\mathbf b_i^a}=-\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a},&&\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\tilde{\delta}\mathbf b_i^g}=-\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}.\end{aligned}$$

#### §5.2.2 $\mathbf r_{\Delta\mathbf v_{ij}}$ 的雅可比（RSS §2.2，逐项）

$\mathbf R_j,\mathbf p_i,\mathbf p_j$ 不出现，故对 $\delta\boldsymbol\phi_j,\delta\mathbf p_i,\delta\mathbf p_j$ 雅可比零；对 bias 同样取系数。记 $D\doteq\Delta\tilde{\mathbf v}_{ij}+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b_i^a$：
$$\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_i+\delta\mathbf v_i)=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\delta\mathbf v_i-\mathbf g\Delta t_{ij})-D=\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_i)-\mathbf R_i^\top\delta\mathbf v_i,\tag{75}$$
$$\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_j+\delta\mathbf v_j)=\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_j)+\mathbf R_i^\top\delta\mathbf v_j,\tag{76}$$
$$\begin{aligned}\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))&=(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-D\\&\overset{(4)}{\simeq}(\mathbf I-\delta\boldsymbol\phi_i^\wedge)\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-D\overset{(2)}{=}\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf R_i)+\big(\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})\big)^\wedge\delta\boldsymbol\phi_i.\end{aligned}\tag{77}$$
**汇总**：
$$\begin{aligned}&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\boldsymbol\phi_i}=\big(\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})\big)^\wedge,&&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\boldsymbol\phi_j}=\mathbf 0,\\&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf p_i}=\mathbf 0,&&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf p_j}=\mathbf 0,\\&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf v_i}=-\mathbf R_i^\top,&&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf v_j}=\mathbf R_i^\top,\\&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\tilde{\delta}\mathbf b_i^a}=-\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a},&&\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\tilde{\delta}\mathbf b_i^g}=-\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}.\end{aligned}$$

#### §5.2.3 $\mathbf r_{\Delta\mathbf R_{ij}}$ 的雅可比（RSS §2.3，逐项，较繁）

$\mathbf p_i,\mathbf p_j,\mathbf v_i,\mathbf v_j,\delta\mathbf b_i^a$ 不出现，对应雅可比零。记 $\bar{\mathbf E}\doteq\mathrm{Exp}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g\big)$。

对 $\delta\boldsymbol\phi_i$：
$$\begin{aligned}\mathbf r_{\Delta\mathbf R_{ij}}(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))&=\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\bar{\mathbf E}\big)^\top(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))^\top\mathbf R_j\Big)\\&=\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\bar{\mathbf E}\big)^\top\mathrm{Exp}(-\delta\boldsymbol\phi_i)\mathbf R_i^\top\mathbf R_j\Big)\\&\overset{(11)}{=}\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\bar{\mathbf E}\big)^\top\mathbf R_i^\top\mathbf R_j\,\mathrm{Exp}(-\mathbf R_j^\top\mathbf R_i\delta\boldsymbol\phi_i)\Big)\\&\overset{(9)}{\simeq}\mathbf r_{\Delta\mathbf R_{ij}}(\mathbf R_i)-\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_i))\mathbf R_j^\top\mathbf R_i\,\delta\boldsymbol\phi_i.\end{aligned}\tag{78 / A.31}$$
对 $\delta\boldsymbol\phi_j$：
$$\mathbf r_{\Delta\mathbf R_{ij}}(\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_j))=\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\bar{\mathbf E}\big)^\top\mathbf R_i^\top(\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_j))\Big)\overset{(9)}{\simeq}\mathbf r_{\Delta\mathbf R_{ij}}(\mathbf R_j)+\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_j))\,\delta\boldsymbol\phi_j.\tag{79 / A.32}$$
对 $\tilde{\delta}\mathbf b_i^g$（最繁，记 $\mathbf E\doteq\mathrm{Exp}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g\big)$，$\hat{\mathbf J}_r^b\doteq\mathbf J_r\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b_i^g\big)$）：
$$\begin{aligned}\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g+\tilde{\delta}\mathbf b_i^g)&=\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\mathrm{Exp}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}(\delta\mathbf b_i^g+\tilde{\delta}\mathbf b_i^g)\big)\big)^\top\mathbf R_i^\top\mathbf R_j\Big)\\&\overset{(7)}{\simeq}\mathrm{Log}\Big(\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\mathbf E\,\mathrm{Exp}\big(\hat{\mathbf J}_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b_i^g\big)\big)^\top\mathbf R_i^\top\mathbf R_j\Big)\\&=\mathrm{Log}\Big(\mathrm{Exp}\big(-\hat{\mathbf J}_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b_i^g\big)\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i^g)\mathbf E\big)^\top\mathbf R_i^\top\mathbf R_j\Big)\\&=\mathrm{Log}\Big(\mathrm{Exp}\big(-\hat{\mathbf J}_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b_i^g\big)\mathrm{Exp}\big(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)\big)\Big)\\&\overset{(11)}{=}\mathrm{Log}\Big(\mathrm{Exp}\big(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)\big)\mathrm{Exp}\big(-\mathrm{Exp}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g))^\top\hat{\mathbf J}_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b_i^g\big)\Big)\\&\overset{(9)}{\simeq}\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)-\mathbf J_r^{-1}\big(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)\big)\mathrm{Exp}\big(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)\big)^\top\hat{\mathbf J}_r^b\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b_i^g.\end{aligned}\tag{80 / A.33}$$
**汇总**（记 $\boldsymbol\alpha$ 为对 $\tilde{\delta}\mathbf b_i^g$ 的系数）：
$$\begin{aligned}&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\boldsymbol\phi_i}=-\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_i))\,\mathbf R_j^\top\mathbf R_i,&&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf p_i}=\mathbf 0,\\&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf v_i}=\mathbf 0,&&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\boldsymbol\phi_j}=\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_j)),\\&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf p_j}=\mathbf 0,&&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf v_j}=\mathbf 0,\\&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\tilde{\delta}\mathbf b_i^a}=\mathbf 0,&&\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\tilde{\delta}\mathbf b_i^g}=\boldsymbol\alpha,\end{aligned}\tag{81}$$
$$\boldsymbol\alpha=-\mathbf J_r^{-1}\big(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)\big)\,\mathrm{Exp}\big(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b_i^g)\big)^\top\,\hat{\mathbf J}_r^b\,\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}.$$
> 注：$\mathbf r_{\Delta\mathbf R}(\mathbf R_i)$ 表残差在当前 $\mathbf R_i$（$\delta\boldsymbol\phi_i=0$）处之值，余同。

### §5.3 bias 残差（随机游走因子） [TRO §VI-E，式 46–48]

bias 缓变，用“布朗运动”（积分白噪声）建模：
$$\dot{\mathbf b}^g(t)=\boldsymbol\eta^{bg},\qquad\dot{\mathbf b}^a(t)=\boldsymbol\eta^{ba}.\tag{46}$$
在 $[t_i,t_j]$ 积分（简写 $\mathbf b_i^g=\mathbf b^g(t_i)$）：
$$\mathbf b_j^g=\mathbf b_i^g+\boldsymbol\eta^{bgd},\qquad\mathbf b_j^a=\mathbf b_i^a+\boldsymbol\eta^{bad},\tag{47}$$
离散噪声 $\boldsymbol\eta^{bgd},\boldsymbol\eta^{bad}$ 零均值，协方差 $\boldsymbol\Sigma_{bgd}\doteq\Delta t_{ij}\,\mathrm{Cov}(\boldsymbol\eta^{bg})$，$\boldsymbol\Sigma_{bad}\doteq\Delta t_{ij}\,\mathrm{Cov}(\boldsymbol\eta^{ba})$[58, Appendix]。(47) 作为额外加性项入 MAP (26)，对所有连续关键帧：
$$\|\mathbf r_{b_{ij}}\|^2\doteq\|\mathbf b_j^g-\mathbf b_i^g\|^2_{\boldsymbol\Sigma_{bgd}}+\|\mathbf b_j^a-\mathbf b_i^a\|^2_{\boldsymbol\Sigma_{bad}}.\tag{48}$$

---

## §6 与 VIO 的接口：无结构视觉因子 [TRO §VII，附录 IX-D；RSS §3]

> 本章重点是 IMU 预积分，视觉因子仅作“接口”骨架抽取（综合时若需视觉细节另见原文）。**关键接口**：预积分 IMU 因子约束 $(\mathbf x_i,\mathbf x_j)$，视觉因子约束观测同一路标的关键帧集，二者在因子图 (25)/(26) 中并列，由 iSAM2 增量平滑求 MAP。

**重投影残差**（单图像测量 $\mathbf z_{il}$）：
$$\mathbf r_{\mathcal C_{il}}=\mathbf z_{il}-\pi(\mathbf R_i,\mathbf p_i,\boldsymbol\rho_l),\tag{50}$$
$\boldsymbol\rho_l\in\mathbb R^3$ 路标位置，$\pi(\cdot)$ 标准透视投影（含已知 IMU-相机外参 $\mathbf T_{\mathrm{BC}}$）。视觉贡献到 (26)：
$$\sum_{i\in\mathcal K_k}\sum_{l\in\mathcal C_i}\|\mathbf r_{\mathcal C_{il}}\|^2_{\boldsymbol\Sigma_C}=\sum_{l=1}^{L}\sum_{i\in\mathcal X(l)}\|\mathbf r_{\mathcal C_{il}}\|^2_{\boldsymbol\Sigma_C},\tag{49}$$
$\mathcal X(l)$＝看到 $l$ 的关键帧子集。**无结构法**：用 Schur 补在每次 GN 迭代线性消去路标 $\boldsymbol\rho_l$（仍得最优 MAP）。提升后 (49) 成 (51)，线性化为 (52)：
$$\sum_{l=1}^{L}\sum_{i\in\mathcal X(l)}\|\mathbf F_{il}\delta\mathbf T_i+\mathbf E_{il}\delta\boldsymbol\rho_l-\mathbf b_{il}\|^2,\tag{52 / A.34}$$
$\delta\mathbf T_i=[\delta\boldsymbol\phi_i\ \delta\mathbf p_i]^\top$；$\mathbf F_{il}\in\mathbb R^{2\times6},\mathbf E_{il}\in\mathbb R^{2\times3}$ 与 $\mathbf b_{il}\in\mathbb R^2$（均被 $\boldsymbol\Sigma_C^{1/2}$ 归一化）由线性化得，$\mathbf b_{il}$ 是线性化点残差。堆叠为 (53)，最小化关于 $\delta\boldsymbol\rho_l$（每路标只出现一次）：
$$\delta\boldsymbol\rho_l=-(\mathbf E_l^\top\mathbf E_l)^{-1}\mathbf E_l^\top(\mathbf F_l\delta\mathbf T_{\mathcal X(l)}-\mathbf b_l),\tag{54 / A.37}$$
代回消元：
$$\sum_{l=1}^{L}\big\|(\mathbf I-\mathbf E_l(\mathbf E_l^\top\mathbf E_l)^{-1}\mathbf E_l^\top)(\mathbf F_l\delta\mathbf T_{\mathcal X(l)}-\mathbf b_l)\big\|^2,\tag{55 / A.38}$$
$\mathbf I-\mathbf E_l(\mathbf E_l^\top\mathbf E_l)^{-1}\mathbf E_l^\top$ 是 $\mathbf E_l$ 的正交投影（Schur 补/边缘化）。

**零空间投影实现**（附录 IX-D / RSS §3）：记 $\mathbf Q=\mathbf I-\mathbf E_l(\mathbf E_l^\top\mathbf E_l)^{-1}\mathbf E_l^\top\in\mathbb R^{2n_l\times2n_l}$（$n_l$＝观测 $l$ 的相机数）。$\mathbf E_l\in\mathbb R^{2n_l\times3}$ 秩 3，其零空间维 $2n_l-3$。任一零空间基 $\mathbf E_l^\perp\in\mathbb R^{2n_l\times(2n_l-3)}$ 满足
$$\mathbf E_l^\perp\big((\mathbf E_l^\perp)^\top\mathbf E_l^\perp\big)^{-1}(\mathbf E_l^\perp)^\top=\mathbf I-\mathbf E_l(\mathbf E_l^\top\mathbf E_l)^{-1}\mathbf E_l^\top.\tag{82 / A.39}$$
用 SVD 得酉基（$(\mathbf E_l^\perp)^\top\mathbf E_l^\perp=\mathbf I$）。代入 (55) 得等价、计算更省（无矩阵求逆）的形式：
$$\sum_{l=1}^{L}\|(\mathbf E_l^\perp)^\top(\mathbf F_l\delta\mathbf T_{\mathcal X(l)}-\mathbf b_l)\|^2.\tag{83 / A.40}$$
> **接口总结**：消元把含位姿+路标的因子 (51) 降为只含位姿的 $L$ 个因子 (55)/(83)，路标 $l$ 的因子只连接 $\mathcal X(l)$（图 3 连通性）。与 MSC-KF[5] 类似但允许多次重线性化与增量纳入新测量（iSAM2[3]）。IMU 预积分因子（§5）与这些视觉因子在同一因子图中由 iSAM2 求解。

---

## §7 附录 IX-E：用欧拉角积分角速率（对比/历史） [TRO 附录 IX-E]

> 本节给出[Lupton-Sukkarieh 2012]式的欧拉角参数化积分，用于与本文 SO(3) 法对比（实验 §VIII-A5）。本书主线用 SO(3) 法（§3–§5），此节仅备查。

设 $\tilde{\boldsymbol\omega}_k$ 为时刻 $k$ 角速率测量、$\boldsymbol\eta_k^g$ 对应噪声，欧拉角 $\boldsymbol\theta_k\in\mathbb R^3$，则
$$\boldsymbol\theta_{k+1}=\boldsymbol\theta_k+[\mathbf E'(\boldsymbol\theta_k)]^{-1}(\tilde{\boldsymbol\omega}_k-\boldsymbol\eta_k^g)\Delta t,\tag{84}$$
$\mathbf E'(\boldsymbol\theta_k)$ 为共轭欧拉角速率矩阵[72]。$\boldsymbol\theta_{k+1}$ 协方差一阶传播：
$$\boldsymbol\Sigma_{k+1}^{\mathrm{Euler}}=\mathbf A_k\,\boldsymbol\Sigma_k^{\mathrm{Euler}}\,\mathbf A_k^\top+\mathbf B_k\,\boldsymbol\Sigma_\eta\,\mathbf B_k^\top,\tag{85}$$
其中 $\mathbf A_k\doteq\mathbf I_{3}+\tfrac{\partial[\mathbf E'(\boldsymbol\theta_k)]^{-1}}{\partial\boldsymbol\theta_k}\Delta t$，$\mathbf B_k\doteq-[\mathbf E'(\boldsymbol\theta_k)]^{-1}\Delta t$，$\boldsymbol\Sigma_\eta$ 为测量噪声 $\boldsymbol\eta_k^{gd}$ 协方差。
> **对比要点（源 §VIII-A5）**：欧拉角法存在万向锁与非全局参数化问题；本文 SO(3) 法在大角速率/快速运动下更稳健、精度更高，且雅可比解析、可在线增量。

---

## §8 预积分计算流程（伪码化汇总，便于实现）

> 综合层可直接据此组织“在线预积分”实现。下列步骤完全由本论文 (A.10)、(A.7)/(A.9)、(A.20) 给出。

**初始化**（在关键帧 $i$，用当前 bias 估计 $\bar{\mathbf b}_i=[\bar{\mathbf b}_i^g,\bar{\mathbf b}_i^a]$）：
- $\Delta\tilde{\mathbf R}_{ii}\leftarrow\mathbf I$，$\Delta\tilde{\mathbf v}_{ii}\leftarrow\mathbf 0$，$\Delta\tilde{\mathbf p}_{ii}\leftarrow\mathbf 0$，$\Delta t_{ii}\leftarrow0$；
- $\boldsymbol\Sigma_{ii}\leftarrow\mathbf 0_{9\times9}$；
- 雅可比 $\partial\Delta\bar{\mathbf R}/\partial\mathbf b^g\leftarrow\mathbf 0$，$\partial\Delta\bar{\mathbf v}/\partial\mathbf b^g\leftarrow\mathbf 0$，$\partial\Delta\bar{\mathbf v}/\partial\mathbf b^a\leftarrow\mathbf 0$，$\partial\Delta\bar{\mathbf p}/\partial\mathbf b^g\leftarrow\mathbf 0$，$\partial\Delta\bar{\mathbf p}/\partial\mathbf b^a\leftarrow\mathbf 0$。

**对每个 IMU 测量 $(\tilde{\boldsymbol\omega}_k,\tilde{\mathbf a}_k)$，间隔 $\Delta t$**（$k=i,\dots,j-1$）：
1. 去 bias：$\hat{\boldsymbol\omega}\leftarrow\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}_i^g$，$\hat{\mathbf a}\leftarrow\tilde{\mathbf a}_k-\bar{\mathbf b}_i^a$；
2. 计算 $\mathbf J_r^k=\mathbf J_r(\hat{\boldsymbol\omega}\Delta t)$（式 8）；
3. **更新协方差**（式 A.7/A.9，用当前 $\Delta\tilde{\mathbf R}_{ik}=\Delta\tilde{\mathbf R}$、$\Delta\tilde{\mathbf R}_{k\,k+1}=\mathrm{Exp}(\hat{\boldsymbol\omega}\Delta t)$ 构造 $\mathbf A,\mathbf B$）：$\boldsymbol\Sigma\leftarrow\mathbf A\boldsymbol\Sigma\mathbf A^\top+\mathbf B\boldsymbol\Sigma_\eta\mathbf B^\top$；
4. **更新 bias 雅可比**（式 A.20 的迭代形式，须在更新预积分量**之前**用旧 $\Delta\tilde{\mathbf R}$）；
5. **更新预积分量**（式 A.10）：
   - $\Delta\tilde{\mathbf p}\leftarrow\Delta\tilde{\mathbf p}+\Delta\tilde{\mathbf v}\Delta t+\tfrac12\Delta\tilde{\mathbf R}\,\hat{\mathbf a}\Delta t^2$；
   - $\Delta\tilde{\mathbf v}\leftarrow\Delta\tilde{\mathbf v}+\Delta\tilde{\mathbf R}\,\hat{\mathbf a}\Delta t$；
   - $\Delta\tilde{\mathbf R}\leftarrow\Delta\tilde{\mathbf R}\,\mathrm{Exp}(\hat{\boldsymbol\omega}\Delta t)$；
   - $\Delta t_{ij}\leftarrow\Delta t_{ij}+\Delta t$。
   （注意位置/速度更新顺序：用更新前的 $\Delta\tilde{\mathbf v}$ 与 $\Delta\tilde{\mathbf R}$。）

**构造因子**（到达关键帧 $j$）：以 $\{\Delta\tilde{\mathbf R}_{ij},\Delta\tilde{\mathbf v}_{ij},\Delta\tilde{\mathbf p}_{ij},\boldsymbol\Sigma_{ij},\Delta t_{ij}\}$ 与 5 个 bias 雅可比构造预积分 IMU 因子，残差按式 (45)、雅可比按式 (71)–(81)。**bias 变化时**用式 (44)/(45) 的一阶修正免重积分。

---

## §9 本源关键结论速查表

| 量 | 表达式 | 出处 |
|---|---|---|
| 陀螺模型 | $\tilde{\boldsymbol\omega}=\boldsymbol\omega+\mathbf b^g+\boldsymbol\eta^g$ | (27) |
| 加计模型 | $\tilde{\mathbf a}=\mathbf R^\top(\mathbf a-\mathbf g)+\mathbf b^a+\boldsymbol\eta^a$ | (28) |
| 连续运动学 | $\dot{\mathbf R}=\mathbf R\boldsymbol\omega^\wedge,\ \dot{\mathbf v}=\mathbf a,\ \dot{\mathbf p}=\mathbf v$ | (29) |
| 离散积分 | $\mathbf R'=\mathbf R\mathrm{Exp}((\tilde{\boldsymbol\omega}-\mathbf b^g-\boldsymbol\eta^{gd})\Delta t)$ 等 | (31) |
| 预积分 $\Delta\tilde{\mathbf R}_{ij}$ | $\prod\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t)$ | (35) |
| 预积分 $\Delta\tilde{\mathbf v}_{ij}$ | $\sum\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t$ | (36) |
| 预积分 $\Delta\tilde{\mathbf p}_{ij}$ | $\sum[\Delta\tilde{\mathbf v}_{ik}\Delta t+\tfrac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t^2]$ | (37) |
| 测量模型 | $\Delta\tilde{\mathbf R}_{ij}=\mathbf R_i^\top\mathbf R_j\mathrm{Exp}(\delta\boldsymbol\phi_{ij})$ 等 | (38) |
| 旋转噪声 | $\delta\boldsymbol\phi_{ij}\simeq\sum\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t$ | (42) |
| 协方差传播 | $\boldsymbol\Sigma_{i\,k+1}=\mathbf A\boldsymbol\Sigma_{ik}\mathbf A^\top+\mathbf B\boldsymbol\Sigma_\eta\mathbf B^\top$ | (63)/(A.9) |
| bias 修正 | $\Delta\tilde{\mathbf R}_{ij}(\mathbf b)\simeq\Delta\bar{\mathbf R}_{ij}\mathrm{Exp}(\tfrac{\partial\Delta\bar{\mathbf R}}{\partial\mathbf b^g}\delta\mathbf b^g)$ 等 | (44) |
| bias 雅可比 | 见 §4.5.4 五式 | (A.20) |
| IMU 残差 | $\mathbf r_{\Delta\mathbf R},\mathbf r_{\Delta\mathbf v},\mathbf r_{\Delta\mathbf p}$ | (45) |
| 残差雅可比 | 见 §5.2 三组 | (71)–(81) |
| bias 随机游走 | $\mathbf b_j=\mathbf b_i+\boldsymbol\eta^{bd}$，$\boldsymbol\Sigma_{bd}=\Delta t_{ij}\mathrm{Cov}(\boldsymbol\eta^b)$ | (47) |

---

## §X 本源未覆盖项（需他源补齐）

本论文聚焦**优化/因子图/MAP 框架下的流形预积分**，以下本章可能涉及但本源**未给**，需综合 agent 从他源补：
1. **四元数实现细节**：本源全程用旋转矩阵 $\mathbf R$ 与 SO(3) 指数映射，不用四元数。若实现用 Hamilton 四元数（本书约定），姿态更新、$\mathbf J_r$ 的四元数等价、半角等需从 Solà《Quaternion kinematics for ESKF》或 Scommer/Barfoot 补。
2. **连续时间预积分 / 高阶积分**：本源用一阶欧拉（零阶保持），明确指出慢率 IMU 可用高阶[54–57] 但未展开。连续预积分（如 Eckenhoff 等闭式、RK4）需他源。
3. **ESKF / 卡尔曼滤波形式的 IMU 传播**：本源是优化框架，§II 仅对比滤波劣势，未给 ESKF 协方差传播。滤波侧 IMU 误差状态传播见 Solà ESKF / Sola 的 IMU 章 / MSCKF 原文。
4. **左扰动版本**：本源全程右扰动/右雅可比（与本书主约定一致），未给左扰动对偶（如需对接 Li-Mourikis/JPL 习惯需转换）。
5. **完整重力对齐/初始化、尺度可观性、IMU-相机时空标定推导**：本源引用 Kalibr[59]、Martinelli[1] 但未展开。
6. **iSAM2 / Bayes 树增量平滑算法细节**：本源引用[3]，未在文内推导。
7. **noise 项中 $\Delta\tilde{\mathbf R}_{ik}$ vs $\Delta\bar{\mathbf R}_{ik}$ 的精细区别**：本源在噪声传播用 $\Delta\tilde{\mathbf R}$、在 bias 雅可比用 $\Delta\bar{\mathbf R}$，实际实现常统一在 $\bar{\mathbf b}_i$ 处线性化（GTSAM 实现）；综合时可点明此实现一致性。

---

> **抽取完整性自评**：本文件覆盖 Forster TRO 论文与 IMU 直接相关的全部内容——§III 预备（式 1–21 全部，含 SO(3)/SE(3)、Rodrigues、右雅可比闭式及逆、伴随性质、retraction、SO(3) 高斯分布与负对数似然）、§IV MAP 框架（式 22–26）、§V IMU 模型与运动积分（式 27–31，含离散/连续运动学、离散噪声协方差关系）、§VI 预积分（式 32–48，含相对增量、噪声分离三式的完整代数、噪声向量、迭代噪声传播 A/B 矩阵与协方差递推、bias 一阶修正与五个雅可比、残差、bias 随机游走因子）、附录 IX-A/B/C/D/E（迭代噪声传播完整证明、bias 修正完整逐步证明、三组残差雅可比逐项推导、零空间投影、欧拉角积分）；并整合 RSS 补充材料 §1–§4 的逐步证明（A.1–A.48），其中 §4 角速度与右雅可比的来源证明（A.41–A.48）为 TRO 正文所无、本文件补入。视觉因子（§VII）按“VIO 接口”需要抽取骨架（式 49–55、82–83）。所有式给出 LaTeX、两源式号对照、记号约定差异表、实现伪码与速查表。**未尽部分**仅为本源刻意不涉及的他源内容（见 §X），已逐条列出。
