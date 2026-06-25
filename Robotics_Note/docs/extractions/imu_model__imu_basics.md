# 抽取留痕：IMU 测量模型基础 + 流形预积分（多源合并）

> **主题**：IMU 测量模型（加速度计/陀螺仪误差模型：bias / scale / 噪声 / 随机游走）、连续→离散、重力与坐标系；并向本章【IMU 模型与预积分】延伸覆盖：连续/离散运动学、SO(3)/SE(3) 流形预积分、bias 一阶修正、协方差传播、与 VIO 的接口。
>
> **本文件性质**：项目【内部抽取留痕】，**不是成书正文**。遵循「禁摘要·全量保真」铁律：每一步推导（中间代数不跳）、每一道例题/数值例、每一条定义/定理/引理 + 完整证明、每一张表/分类/伪码 完整记录；公式全部 LaTeX 写全，标注【源小节号】，保留所有式号与条件。宁长勿略。
>
> **服务章节**：`parts/P2_slam/imu_model`（`ch:imu`，IMU 模型与预积分），下接 `vio`（`ch:vio`）。
>
> **抽取人备注**：本主题没有单一"权威专著"，而是由若干权威论文/技术报告/官方文档拼合而成。下面**按源**组织（每源一节，标 §S1…），每节内再按源小节逐式抽取；最后 §M「跨源合并与记号统一」做约定转换、§X「本章未覆盖/需他源补齐」列缺口。

---

## §0 本抽取所用权威源清单（逐条出处）

| 标记 | 文献 | 出处（arXiv/URL/报告号） | 在本抽取中的角色 |
|---|---|---|---|
| **【F】** | C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* | IEEE T-RO 2017, DOI 10.1109/TRO.2016.2597321；**arXiv:1512.02363v3** [cs.RO]，PDF https://arxiv.org/pdf/1512.02363 | **预积分理论骨架**：IMU 测量模型、SO(3) 预积分、噪声传播（迭代式协方差）、bias 一阶修正、残差雅可比。本章核心源。 |
| **【K】** | ethz-asl/kalibr Wiki, *IMU Noise Model* | https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model | **连续→离散噪声转换的事实标准**：白噪声 / bias 随机游走的连续谱密度↔离散方差，单位约定。 |
| **【W】** | O. J. Woodman, *An Introduction to Inertial Navigation* | Univ. Cambridge Tech. Report **UCAM-CL-TR-696** (2007)，PDF https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf | **误差源分类 + 捷联机理 + 重力/坐标系 + Allan**：bias/白噪声/标定误差/温度，捷联姿态/位置积分，重力投影、漂移定量。 |
| **【V】** | T. Qin, P. Li, S. Shen, *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator* | IEEE T-RO 2018；**arXiv:1708.03852** [cs.RO]，PDF https://arxiv.org/pdf/1708.03852 | **四元数离散预积分（α/β/γ）+ 误差态连续动态 F/G + 离散协方差/雅可比递推**：与【F】的迭代式互为对照。 |
| **【O】** | Y. Yang, P. Geneva, X. Zuo, G. Huang, *Online Self-Calibration for VINS: Models, Analysis and Degeneracy* | **arXiv:2201.09170** [cs.RO]，PDF https://arxiv.org/pdf/2201.09170 | **完整 IMU 内参模型**：scale + 轴失准（上/下三角）+ g-sensitivity 矩阵，模型变体表（含 Kalibr/Rehder 模型）。 |
| **【A】** | H. Hou, *Modeling Inertial Sensors Errors Using Allan Variance* | M.Sc. thesis, Univ. of Calgary, UCGE Report 20201 (2004)，PDF https://www.ucalgary.ca/engo_webdocs/NES/04.20201.HaiyingHou.pdf；理论遵循 **IEEE Std 952-1997** | **Allan 方差五项噪声辨识**：定义、PSD↔AVAR、五种噪声幂律斜率与读数法（含 0.664 常数）。 |
| **【G】** | NIMA TR8350.2, *Department of Defense World Geodetic System 1984* | https://gis-lab.info/docs/nima-tr8350.2-wgs84fin.pdf；及 IEEE/INS 坐标系综述 | **重力模型（Somigliana 正常重力）+ 坐标系（ECI/ECEF/NED/ENU/body）**。 |

> **与本书统一约定的总体差异预告**（详见 §M）：
> - 旋转矩阵：【F】【V】用 $\mathbf R$（本书一致）；【W】用 $\mathbf C$（方向余弦阵，本书改记 $\mathbf R$）。
> - 扰动方向：【F】用**右扰动/右雅可比** $\mathbf J_r$（与本书主约定一致）；【V】姿态误差 $\delta\boldsymbol\theta$ 定义在四元数**右乘**（局部，亦与本书右扰动一致）。
> - 四元数：【V】Hamilton，实部在前 $\mathbf q=[q_w,\mathbf q_v]$ 但 VINS 代码存储顺序为 $[\mathbf q_v,q_w]$（仍 Hamilton），本书 Hamilton 一致。
> - $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 排序：诸源 IMU 状态向量自成排序（位置/姿态/速度），与 $\mathfrak{se}(3)$ 的 $[\boldsymbol\rho;\boldsymbol\phi]$ 不在同一层次，无冲突。
> - 协方差字母：【F】$\boldsymbol\Sigma$（预积分协方差）、$\boldsymbol\Sigma_\eta$（原始 IMU 噪声协方差）；【V】$\mathbf P$（预积分协方差）、$\mathbf Q=\mathrm{diag}(\sigma_a^2,\sigma_w^2,\sigma_{ba}^2,\sigma_{bw}^2)$。

---

# §S1 Forster 等《流形预积分》（arXiv:1512.02363）

> 本节为本章**理论骨架**。逐式抽取 §III（SO(3) 预备）、§V（IMU 模型与运动积分）、§VI（流形预积分）、附录 A（迭代噪声传播）、附录 B（bias 一阶修正完整推导）、附录 C（残差雅可比完整推导）。式号沿用原文。

## §S1.0 记号约定（Forster 本源）

- 旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$；位姿 $\mathbf T=(\mathbf R,\mathbf p)\in\mathrm{SE}(3)$。
- 前缀 $B$/$W$ 表所在坐标系：$B$=body/IMU 系，$W$=world 系。位姿 $\{\mathbf R_{WB},{}_W\mathbf p\}$ 把 $B$ 系点映到 $W$ 系。
- $(\cdot)^\wedge:\mathbb R^3\to\mathfrak{so}(3)$ hat 算子；$(\cdot)^\vee$ vee 逆算子。
- $\mathrm{Exp}/\mathrm{Log}$（大写，向量↔矩阵）；$\exp/\log$（小写，李代数↔群）。
- **右扰动 + 右雅可比 $\mathbf J_r$**（与本书主约定一致）。
- 测量带波浪号 $\tilde{(\cdot)}$（如 $\tilde{\boldsymbol\omega},\tilde{\mathbf a}$）；估计/标称带横杠 $\bar{(\cdot)}$（如 $\bar{\mathbf b}_i$）。
- 协方差：预积分噪声 $\boldsymbol\Sigma_{ij}$；原始 IMU 噪声 $\boldsymbol\Sigma_\eta\in\mathbb R^{6\times6}$；bias 随机游走 $\boldsymbol\Sigma^{bgd},\boldsymbol\Sigma^{bad}$。

## §S1.1 SO(3) 预备 [源 §III-A]

**SO(3) 定义**：
$$\mathrm{SO}(3)\triangleq\{\mathbf R\in\mathbb R^{3\times3}:\mathbf R^\top\mathbf R=\mathbf I,\ \det(\mathbf R)=1\}.$$
群运算为矩阵乘法，逆为转置。切空间（在单位元处）记 $\mathfrak{so}(3)$（李代数），即 $3\times3$ 反对称矩阵空间。

**hat 算子**（向量→反对称阵）：
$$\boldsymbol\omega^\wedge=\begin{bmatrix}\omega_1\\\omega_2\\\omega_3\end{bmatrix}^\wedge=\begin{bmatrix}0&-\omega_3&\omega_2\\\omega_3&0&-\omega_1\\-\omega_2&\omega_1&0\end{bmatrix}\in\mathfrak{so}(3).\tag{1}$$
**vee 算子** $(\cdot)^\vee$：对反对称阵 $\mathbf S=\boldsymbol\omega^\wedge$，$\mathbf S^\vee=\boldsymbol\omega$。

**反对称阵性质**（后续常用）：
$$\mathbf a^\wedge\mathbf b=-\mathbf b^\wedge\mathbf a,\quad\forall\,\mathbf a,\mathbf b\in\mathbb R^3.\tag{2}$$

**指数映射**（在单位元处）$\exp:\mathfrak{so}(3)\to\mathrm{SO}(3)$，即矩阵指数（Rodrigues 公式）：
$$\exp(\boldsymbol\phi^\wedge)=\mathbf I+\frac{\sin(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|}\boldsymbol\phi^\wedge+\frac{1-\cos(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|^2}(\boldsymbol\phi^\wedge)^2.\tag{3}$$
**一阶近似**：
$$\exp(\boldsymbol\phi^\wedge)\approx\mathbf I+\boldsymbol\phi^\wedge.\tag{4}$$

**对数映射**（在单位元处）：对 $\mathbf R\ne\mathbf I$，
$$\log(\mathbf R)=\frac{\varphi\cdot(\mathbf R-\mathbf R^\top)}{2\sin(\varphi)},\quad\varphi=\cos^{-1}\!\Big(\frac{\mathrm{tr}(\mathbf R)-1}{2}\Big).\tag{5}$$
有 $\log(\mathbf R)^\vee=\mathbf a\varphi$，$\mathbf a,\varphi$ 为 $\mathbf R$ 的转轴与转角。$\mathbf R=\mathbf I$ 时 $\varphi=0$，$\mathbf a$ 不定可任取。指数映射限制在开球 $\|\boldsymbol\phi\|<\pi$ 时为双射，逆即对数；不限定时为满射（$\boldsymbol\phi=(\varphi+2k\pi)\mathbf a$ 均为可行对数）。

**向量化 Exp/Log**：
$$\mathrm{Exp}:\mathbb R^3\to\mathrm{SO}(3);\ \boldsymbol\phi\mapsto\exp(\boldsymbol\phi^\wedge),\qquad \mathrm{Log}:\mathrm{SO}(3)\to\mathbb R^3;\ \mathbf R\mapsto\log(\mathbf R)^\vee.\tag{6}$$

**右雅可比关键一阶近似**（图 1 几何意义：切空间加性扰动 $\delta\boldsymbol\phi$ ↔ 流形右乘扰动）：
$$\mathrm{Exp}(\boldsymbol\phi+\delta\boldsymbol\phi)\approx\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}\big(\mathbf J_r(\boldsymbol\phi)\,\delta\boldsymbol\phi\big).\tag{7}$$
**右雅可比闭式**（引 [Chirikjian 2012, p.40]）：
$$\mathbf J_r(\boldsymbol\phi)=\mathbf I-\frac{1-\cos(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|^2}\boldsymbol\phi^\wedge+\frac{\|\boldsymbol\phi\|-\sin(\|\boldsymbol\phi\|)}{\|\boldsymbol\phi\|^3}(\boldsymbol\phi^\wedge)^2.\tag{8}$$
**对数的对偶一阶近似**：
$$\mathrm{Log}\big(\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\delta\boldsymbol\phi)\big)\approx\boldsymbol\phi+\mathbf J_r^{-1}(\boldsymbol\phi)\,\delta\boldsymbol\phi.\tag{9}$$
**右雅可比逆闭式**：
$$\mathbf J_r^{-1}(\boldsymbol\phi)=\mathbf I+\frac12\boldsymbol\phi^\wedge+\Big(\frac{1}{\|\boldsymbol\phi\|^2}+\frac{1+\cos(\|\boldsymbol\phi\|)}{2\|\boldsymbol\phi\|\sin(\|\boldsymbol\phi\|)}\Big)(\boldsymbol\phi^\wedge)^2.$$
$\|\boldsymbol\phi\|\to0$ 时 $\mathbf J_r,\mathbf J_r^{-1}\to\mathbf I$。

**伴随性质**（"穿过旋转"恒等式）：
$$\mathbf R\,\mathrm{Exp}(\boldsymbol\phi)\,\mathbf R^\top=\exp(\mathbf R\boldsymbol\phi^\wedge\mathbf R^\top)=\mathrm{Exp}(\mathbf R\boldsymbol\phi),\tag{10}$$
$$\Longleftrightarrow\quad\mathrm{Exp}(\boldsymbol\phi)\,\mathbf R=\mathbf R\,\mathrm{Exp}(\mathbf R^\top\boldsymbol\phi).\tag{11}$$

**SE(3) 预备** [源 §III-A b)]：$\mathrm{SE}(3)\triangleq\{(\mathbf R,\mathbf p):\mathbf R\in\mathrm{SO}(3),\mathbf p\in\mathbb R^3\}$，是 $\mathrm{SO}(3)$ 与 $\mathbb R^3$ 的半直积。群运算 $\mathbf T_1\cdot\mathbf T_2=(\mathbf R_1\mathbf R_2,\ \mathbf p_1+\mathbf R_1\mathbf p_2)$，逆 $\mathbf T_1^{-1}=(\mathbf R_1^\top,-\mathbf R_1^\top\mathbf p_1)$。本文不需要 SE(3) 的 Exp/Log（原因见 §III-C 的 retraction 选择）。

## §S1.2 SO(3) 上的不确定度描述 [源 §III-B]

切空间定义高斯、经指数映射推到 SO(3)：
$$\tilde{\mathbf R}=\mathbf R\,\mathrm{Exp}(\boldsymbol\epsilon),\qquad\boldsymbol\epsilon\sim\mathcal N(\mathbf 0,\boldsymbol\Sigma),\tag{12}$$
$\mathbf R$ 为无噪均值，$\boldsymbol\epsilon$ 为零均值小扰动、协方差 $\boldsymbol\Sigma$。

为得 $\tilde{\mathbf R}$ 的显式分布，从 $\mathbb R^3$ 高斯归一化积分出发：
$$\int_{\mathbb R^3}p(\boldsymbol\epsilon)\,d\boldsymbol\epsilon=\int_{\mathbb R^3}\alpha\,e^{-\frac12\|\boldsymbol\epsilon\|_{\boldsymbol\Sigma}^2}\,d\boldsymbol\epsilon=1,\tag{13}$$
其中 $\alpha=1/\sqrt{(2\pi)^3\det(\boldsymbol\Sigma)}$，$\|\boldsymbol\epsilon\|_{\boldsymbol\Sigma}^2\triangleq\boldsymbol\epsilon^\top\boldsymbol\Sigma^{-1}\boldsymbol\epsilon$ 为马氏距离平方。换元 $\boldsymbol\epsilon=\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})$（当 $\|\boldsymbol\epsilon\|<\pi$ 时为 (12) 逆），(13) 变为：
$$\int_{\mathrm{SO}(3)}\beta(\tilde{\mathbf R})\,e^{-\frac12\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\|_{\boldsymbol\Sigma}^2}\,d\tilde{\mathbf R}=1,\tag{14}$$
$\beta(\tilde{\mathbf R})$ 为归一化因子，$\beta(\tilde{\mathbf R})=\alpha/|\det(\mathcal J(\tilde{\mathbf R}))|$，$\mathcal J(\tilde{\mathbf R})\triangleq\mathbf J_r(\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R}))$（换元雅可比副产物）。**SO(3) 上的"高斯"分布**：
$$p(\tilde{\mathbf R})=\beta(\tilde{\mathbf R})\,e^{-\frac12\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\|_{\boldsymbol\Sigma}^2}.\tag{15}$$
小协方差时 $\beta\simeq\alpha$（$\mathbf J_r\approx\mathbf I$）。负对数似然：
$$\mathcal L(\mathbf R)=\frac12\big\|\mathrm{Log}(\mathbf R^{-1}\tilde{\mathbf R})\big\|_{\boldsymbol\Sigma}^2+\text{const}=\frac12\big\|\mathrm{Log}(\tilde{\mathbf R}^{-1}\mathbf R)\big\|_{\boldsymbol\Sigma}^2+\text{const},\tag{16}$$
几何上是 $\tilde{\mathbf R}$ 与 $\mathbf R$ 间测地角（平方）按 $\boldsymbol\Sigma^{-1}$ 加权。

## §S1.3 流形上的高斯-牛顿（retraction）[源 §III-C]

流形优化 $\min_{\mathbf x\in\mathcal M}f(\mathbf x)$ 不能直接对 $\mathbf x$ 做二次近似（过参数化 + 解不在 $\mathcal M$ 上）。引入 **retraction** $\mathcal R_{\mathbf x}$（切空间 $\delta\mathbf x$ ↔ $\mathbf x$ 邻域的双射），重参数化（lifting）：
$$\min_{\mathbf x\in\mathcal M}f(\mathbf x)\ \Rightarrow\ \min_{\delta\mathbf x\in\mathbb R^n}f(\mathcal R_{\mathbf x}(\delta\mathbf x)).\tag{18}$$
"lift-solve-retract"：在当前估计切空间求 $\delta\mathbf x^\star$，更新 $\hat{\mathbf x}\leftarrow\mathcal R_{\hat{\mathbf x}}(\delta\mathbf x^\star)$（式 19）。

**本文采用的 retraction**：
$$\mathcal R_{\mathbf R}(\delta\boldsymbol\phi)=\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi),\quad\delta\boldsymbol\phi\in\mathbb R^3,\tag{20}$$
$$\mathcal R_{\mathbf T}(\delta\boldsymbol\phi,\delta\mathbf p)=(\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi),\ \mathbf p+\mathbf R\,\delta\mathbf p),\quad[\delta\boldsymbol\phi\ \delta\mathbf p]\in\mathbb R^6.\tag{21}$$
> 该 retraction 即本书**右扰动**；式 (21) 的位置扰动写成 $\mathbf p+\mathbf R\delta\mathbf p$（即 body-frame 位置增量），故无需 SE(3) 的 Exp。这统一了航空滤波的 error-state 模型。

## §S1.4 IMU 测量模型与运动积分 [源 §V]

IMU = 三轴加速度计 + 三轴陀螺仪，测量传感器相对惯性系的转动率与加速度。测量 ${}_B\tilde{\mathbf a}(t)$、${}_B\tilde{\boldsymbol\omega}_{WB}(t)$ 受**加性白噪声 $\boldsymbol\eta$** 与**缓变 bias $\mathbf b$** 影响：

$$\boxed{\ {}_B\tilde{\boldsymbol\omega}_{WB}(t)={}_B\boldsymbol\omega_{WB}(t)+\mathbf b^g(t)+\boldsymbol\eta^g(t)\ }\tag{27}$$
$$\boxed{\ {}_B\tilde{\mathbf a}(t)=\mathbf R_{WB}^\top(t)\big({}_W\mathbf a(t)-{}_W\mathbf g\big)+\mathbf b^a(t)+\boldsymbol\eta^a(t)\ }\tag{28}$$

- ${}_B\boldsymbol\omega_{WB}(t)\in\mathbb R^3$：$B$ 相对 $W$ 的瞬时角速度（在 $B$ 系表达）。
- ${}_W\mathbf a(t)\in\mathbb R^3$：传感器加速度（$W$ 系）。${}_W\mathbf g$：重力向量（$W$ 系）。
- **加速度计读的是"比力"（specific force）**：真加速度减重力，再旋到 body 系。
- 忽略地球自转（即假设 $W$ 为惯性系）。

**连续运动学模型**（引 [49,53]）：
$$\dot{\mathbf R}_{WB}=\mathbf R_{WB}\,{}_B\boldsymbol\omega_{WB}^\wedge,\qquad {}_W\dot{\mathbf v}={}_W\mathbf a,\qquad{}_W\dot{\mathbf p}={}_W\mathbf v.\tag{29}$$

**积分到 $t+\Delta t$**：
$$\mathbf R_{WB}(t+\Delta t)=\mathbf R_{WB}(t)\,\mathrm{Exp}\!\Big(\int_t^{t+\Delta t}{}_B\boldsymbol\omega_{WB}(\tau)\,d\tau\Big),$$
$${}_W\mathbf v(t+\Delta t)={}_W\mathbf v(t)+\int_t^{t+\Delta t}{}_W\mathbf a(\tau)\,d\tau,$$
$${}_W\mathbf p(t+\Delta t)={}_W\mathbf p(t)+\int_t^{t+\Delta t}{}_W\mathbf v(\tau)\,d\tau+\iint_t^{t+\Delta t}{}_W\mathbf a(\tau)\,d\tau^2.$$

**假设 ${}_W\mathbf a$、${}_B\boldsymbol\omega_{WB}$ 在 $[t,t+\Delta t]$ 内恒定**（欧拉/分段常值）：
$$\mathbf R_{WB}(t+\Delta t)=\mathbf R_{WB}(t)\,\mathrm{Exp}({}_B\boldsymbol\omega_{WB}(t)\Delta t),$$
$${}_W\mathbf v(t+\Delta t)={}_W\mathbf v(t)+{}_W\mathbf a(t)\Delta t,$$
$${}_W\mathbf p(t+\Delta t)={}_W\mathbf p(t)+{}_W\mathbf v(t)\Delta t+\tfrac12{}_W\mathbf a(t)\Delta t^2.\tag{30}$$

**代入 (27)–(28) 用测量表示**（此后省坐标系下标，记号无歧义）：
$$\mathbf R(t+\Delta t)=\mathbf R(t)\,\mathrm{Exp}\big((\tilde{\boldsymbol\omega}(t)-\mathbf b^g(t)-\boldsymbol\eta^{gd}(t))\Delta t\big),$$
$$\mathbf v(t+\Delta t)=\mathbf v(t)+\mathbf g\Delta t+\mathbf R(t)(\tilde{\mathbf a}(t)-\mathbf b^a(t)-\boldsymbol\eta^{ad}(t))\Delta t,$$
$$\mathbf p(t+\Delta t)=\mathbf p(t)+\mathbf v(t)\Delta t+\tfrac12\mathbf g\Delta t^2+\tfrac12\mathbf R(t)(\tilde{\mathbf a}(t)-\mathbf b^a(t)-\boldsymbol\eta^{ad}(t))\Delta t^2.\tag{31}$$

> **近似说明（源原文）**：(31) 在两测量间假设姿态 $\mathbf R(t)$ 恒定，对非零转动率不是 (29) 的精确解；高频 IMU 缓解此误差。该方案简单、便于建模与不确定度传播；慢速率 IMU 可用更高阶数值积分 [54–57]。

**连续→离散噪声关系**（关键）：离散时间噪声 $\boldsymbol\eta^{gd}$ 协方差与连续谱噪声 $\boldsymbol\eta^g$ 的关系：
$$\boxed{\ \mathrm{Cov}(\boldsymbol\eta^{gd}(t))=\frac{1}{\Delta t}\,\mathrm{Cov}(\boldsymbol\eta^{g}(t))\ }$$
$\boldsymbol\eta^{ad}$ 同理（引 [58, Appendix]）。

> **本书统一约定差异**：与本书右扰动一致；上式 $1/\Delta t$ 即 Kalibr 的 $\sigma_d=\sigma_c/\sqrt{\Delta t}$（方差除 $\Delta t$）—— 见 §S2.4 逐式核对。

## §S1.5 IMU 预积分（流形）[源 §VI]

迭代 (31) 对两关键帧 $k=i$ 到 $k=j$ 间所有 $\Delta t$ 区间求积：
$$\mathbf R_j=\mathbf R_i\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b^g_k-\boldsymbol\eta^{gd}_k)\Delta t\big),$$
$$\mathbf v_j=\mathbf v_i+\mathbf g\Delta t_{ij}+\sum_{k=i}^{j-1}\mathbf R_k(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t,\tag{32}$$
$$\mathbf p_j=\mathbf p_i+\sum_{k=i}^{j-1}\Big[\mathbf v_k\Delta t+\tfrac12\mathbf g\Delta t^2+\tfrac12\mathbf R_k(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t^2\Big],$$
其中 $\Delta t_{ij}\triangleq\sum_{k=i}^{j-1}\Delta t$，$(\cdot)_i\triangleq(\cdot)(t_i)$。**缺点**：每当 $t_i$ 处线性化点（如 $\mathbf R_i$）改变，整段须重算。

**定义相对运动增量**（与 $t_i$ 的位姿/速度、重力**无关**，引 [2]）：
$$\Delta\mathbf R_{ij}\triangleq\mathbf R_i^\top\mathbf R_j=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\mathbf b^g_k-\boldsymbol\eta^{gd}_k)\Delta t\big),$$
$$\Delta\mathbf v_{ij}\triangleq\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})=\sum_{k=i}^{j-1}\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t,\tag{33}$$
$$\Delta\mathbf p_{ij}\triangleq\mathbf R_i^\top\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)=\sum_{k=i}^{j-1}\Big[\Delta\mathbf v_{ik}\Delta t+\tfrac12\Delta\mathbf R_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_k-\boldsymbol\eta^{ad}_k)\Delta t^2\Big],$$
其中 $\Delta\mathbf R_{ik}\triangleq\mathbf R_i^\top\mathbf R_k$，$\Delta\mathbf v_{ik}\triangleq\mathbf R_i^\top(\mathbf v_k-\mathbf v_i-\mathbf g\Delta t_{ik})$。

> **重要洞察（源强调）**：与 $\Delta\mathbf R_{ij}$ 不同，$\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$ **不是**真实物理速度/位置变化，而是被定义成使 (33) 右端独立于 $i$ 时刻状态与重力。右端可直接由两帧间惯性测量算出。

**bias 恒定假设**（两帧之间）：
$$\mathbf b^g_i=\mathbf b^g_{i+1}=\dots=\mathbf b^g_{j-1},\qquad\mathbf b^a_i=\mathbf b^a_{i+1}=\dots=\mathbf b^a_{j-1}.\tag{34}$$

### §S1.5.1 预积分测量模型（分离噪声）[源 §VI-A]

**旋转增量**：用一阶近似 (7)（旋转噪声"小"），并用 (11) 把噪声"移到末端"：
$$\Delta\mathbf R_{ij}\stackrel{(7)}{\simeq}\prod_{k=i}^{j-1}\Big[\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)\,\mathrm{Exp}(-\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t)\Big]\stackrel{(11)}{=}\Delta\tilde{\mathbf R}_{ij}\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t\big)\triangleq\Delta\tilde{\mathbf R}_{ij}\,\mathrm{Exp}(-\delta\boldsymbol\phi_{ij}),\tag{35}$$
其中 $\mathbf J_r^k\triangleq\mathbf J_r((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)$，**预积分旋转测量** $\Delta\tilde{\mathbf R}_{ij}\triangleq\prod_{k=i}^{j-1}\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b^g_i)\Delta t)$，其噪声 $\delta\boldsymbol\phi_{ij}$。

**速度增量**：把 (35) 代回 (33) 的 $\Delta\mathbf v_{ij}$，对 $\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})$ 用一阶近似 (4)、弃高阶噪声：
$$\Delta\mathbf v_{ij}\stackrel{(4)}{\simeq}\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\mathbf I-\delta\boldsymbol\phi_{ik}^\wedge)(\tilde{\mathbf a}_k-\mathbf b^a_i)\Delta t-\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t$$
$$\stackrel{(2)}{=}\Delta\tilde{\mathbf v}_{ij}+\sum_{k=i}^{j-1}\Big[\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t-\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t\Big]\triangleq\Delta\tilde{\mathbf v}_{ij}-\delta\mathbf v_{ij},\tag{36}$$
**预积分速度测量** $\Delta\tilde{\mathbf v}_{ij}\triangleq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)\Delta t$，噪声 $\delta\mathbf v_{ij}$。

**位置增量**：同样把 (35)(36) 代回，用一阶近似 (4)：
$$\Delta\mathbf p_{ij}\stackrel{(4)}{\simeq}\sum_{k=i}^{j-1}\Big[(\Delta\tilde{\mathbf v}_{ik}-\delta\mathbf v_{ik})\Delta t+\tfrac12\Delta\tilde{\mathbf R}_{ik}(\mathbf I-\delta\boldsymbol\phi_{ik}^\wedge)(\tilde{\mathbf a}_k-\mathbf b^a_i)\Delta t^2-\tfrac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t^2\Big]$$
$$\stackrel{(2)}{=}\Delta\tilde{\mathbf p}_{ij}+\sum_{k=i}^{j-1}\Big[-\delta\mathbf v_{ik}\Delta t+\tfrac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2-\tfrac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t^2\Big]\triangleq\Delta\tilde{\mathbf p}_{ij}-\delta\mathbf p_{ij},\tag{37}$$
**预积分位置测量** $\Delta\tilde{\mathbf p}_{ij}$、噪声 $\delta\mathbf p_{ij}$。

**预积分测量模型**（把 (35)(36)(37) 代回 (33)，并用 $\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})^\top=\mathrm{Exp}(\delta\boldsymbol\phi_{ij})$）：
$$\boxed{\begin{aligned}\Delta\tilde{\mathbf R}_{ij}&=\mathbf R_i^\top\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_{ij}),\\\Delta\tilde{\mathbf v}_{ij}&=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})+\delta\mathbf v_{ij},\\\Delta\tilde{\mathbf p}_{ij}&=\mathbf R_i^\top\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)+\delta\mathbf p_{ij}.\end{aligned}}\tag{38}$$
即"复合测量 = 待估状态 ⊕ 随机噪声 $[\delta\boldsymbol\phi_{ij}^\top,\delta\mathbf v_{ij}^\top,\delta\mathbf p_{ij}^\top]^\top$"。其优势：噪声合适分布时对数似然直接（后两行加性高斯→二次型；首行 $\delta\boldsymbol\phi_{ij}$ 高斯→对应 (12) 形式的 SO(3) 似然）。

### §S1.5.2 噪声传播 [源 §VI-B]

噪声向量
$$\boldsymbol\eta^\Delta_{ij}\triangleq[\delta\boldsymbol\phi_{ij}^\top,\delta\mathbf v_{ij}^\top,\delta\mathbf p_{ij}^\top]^\top\sim\mathcal N(\mathbf 0_{9\times1},\boldsymbol\Sigma_{ij}).\tag{39}$$
协方差 $\boldsymbol\Sigma_{ij}$ 极重要（其逆加权优化项）。

**旋转噪声**：由 (35)，
$$\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})\triangleq\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t\big).\tag{40}$$
两边取 Log、变号：
$$\delta\boldsymbol\phi_{ij}=-\mathrm{Log}\Big(\prod_{k=i}^{j-1}\mathrm{Exp}(-\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t)\Big).\tag{41}$$
反复用一阶近似 (9)（$\boldsymbol\eta^{gd}_k$ 与 $\delta\boldsymbol\phi_{ij}$ 皆小，右雅可比≈$\mathbf I$）：
$$\delta\boldsymbol\phi_{ij}\simeq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t.\tag{42}$$
一阶下 $\delta\boldsymbol\phi_{ij}$ 零均值高斯（零均值噪声线性组合），正好契合 (12)。

**速度/位置噪声**（$\boldsymbol\eta^{ad}_k$ 与 $\delta\boldsymbol\phi_{ik}$ 的线性组合，故零均值高斯）：
$$\delta\mathbf v_{ij}\simeq\sum_{k=i}^{j-1}\Big[-\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t+\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t\Big],\tag{43}$$
$$\delta\mathbf p_{ij}\simeq\sum_{k=i}^{j-1}\Big[\delta\mathbf v_{ik}\Delta t-\tfrac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2+\tfrac12\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t^2\Big].$$
(42)(43) 把 $\boldsymbol\eta^\Delta_{ij}$ 表为 IMU 测量噪声 $\boldsymbol\eta^d_k\triangleq[\boldsymbol\eta^{gd}_k,\boldsymbol\eta^{ad}_k]$ 的线性函数，故由 $\boldsymbol\eta^d_k$ 协方差（IMU 规格给出）线性传播即得 $\boldsymbol\Sigma_{ij}$。更聪明的迭代式见附录 A（§S1.7）。

### §S1.5.3 bias 更新一阶修正 [源 §VI-C]

预积分用的 bias 估计 $\{\bar{\mathbf b}^a_i,\bar{\mathbf b}^g_i\}$ 在优化中会小幅变 $\delta\mathbf b$。**一阶展开更新**（避免重积分）：给定 $\mathbf b\leftarrow\bar{\mathbf b}+\delta\mathbf b$，
$$\Delta\tilde{\mathbf R}_{ij}(\mathbf b^g_i)\simeq\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathrm{Exp}\Big(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g\Big),\tag{44}$$
$$\Delta\tilde{\mathbf v}_{ij}(\mathbf b^g_i,\mathbf b^a_i)\simeq\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a_i,$$
$$\Delta\tilde{\mathbf p}_{ij}(\mathbf b^g_i,\mathbf b^a_i)\simeq\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a_i.$$
旋转修正**直接作用在 SO(3)** 上（区别于 [2] 的欧拉角）。雅可比 $\{\partial\Delta\bar{\mathbf R}_{ij}/\partial\mathbf b^g,\dots\}$ 在 $\bar{\mathbf b}_i$ 处算，**恒定可预计算**（推导见附录 B，§S1.8）。

### §S1.5.4 预积分 IMU 因子（残差）[源 §VI-D]

残差 $\mathbf r_{\mathcal I_{ij}}\triangleq[\mathbf r_{\Delta\mathbf R_{ij}}^\top,\mathbf r_{\Delta\mathbf v_{ij}}^\top,\mathbf r_{\Delta\mathbf p_{ij}}^\top]^\top\in\mathbb R^9$，含 (44) 的 bias 更新：
$$\mathbf r_{\Delta\mathbf R_{ij}}\triangleq\mathrm{Log}\Big(\Big[\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathrm{Exp}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g\big)\Big]^\top\mathbf R_i^\top\mathbf R_j\Big),$$
$$\mathbf r_{\Delta\mathbf v_{ij}}\triangleq\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-\Big[\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\Big],\tag{45}$$
$$\mathbf r_{\Delta\mathbf p_{ij}}\triangleq\mathbf R_i^\top\big(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2\big)-\Big[\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}^g_i,\bar{\mathbf b}^a_i)+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g+\tfrac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a\Big].$$
高斯-牛顿每次迭代用 (21) retraction lifting，再线性化（残差雅可比见附录 C，§S1.9）。

### §S1.5.5 bias 模型（布朗运动）[源 §VI-E]

bias 缓变 → "布朗运动"（积分白噪声）：
$$\dot{\mathbf b}^g(t)=\boldsymbol\eta^{bg},\qquad\dot{\mathbf b}^a(t)=\boldsymbol\eta^{ba}.\tag{46}$$
在 $[t_i,t_j]$ 积分：
$$\mathbf b^g_j=\mathbf b^g_i+\boldsymbol\eta^{bgd},\qquad\mathbf b^a_j=\mathbf b^a_i+\boldsymbol\eta^{bad},\tag{47}$$
离散噪声 $\boldsymbol\eta^{bgd},\boldsymbol\eta^{bad}$ 零均值，协方差 $\boldsymbol\Sigma^{bgd}\triangleq\Delta t_{ij}\,\mathrm{Cov}(\boldsymbol\eta^{bg})$、$\boldsymbol\Sigma^{bad}\triangleq\Delta t_{ij}\,\mathrm{Cov}(\boldsymbol\eta^{ba})$（引 [58, Appendix]——即**随机游走方差正比 $\Delta t$**）。bias 因子加入因子图：
$$\|\mathbf r_{\mathbf b_{ij}}\|^2\triangleq\|\mathbf b^g_j-\mathbf b^g_i\|^2_{\boldsymbol\Sigma^{bgd}}+\|\mathbf b^a_j-\mathbf b^a_i\|^2_{\boldsymbol\Sigma^{bad}}.\tag{48}$$

## §S1.6 与 VIO 的接口：MAP / 因子图 [源 §IV]

> 本章重点之一"与 VIO 的接口"，Forster 给出预积分如何进因子图。

位姿 $\mathbf T_{WB}\triangleq(\mathbf R_{WB},{}_W\mathbf p)$ 是 body 相对 world 的位姿（假设 body=IMU 系）；相机外参 $\mathbf T_{BC}$ 标定已知（图 2）。预积分 IMU 测量 (38) 噪声零均值高斯（协方差 $\boldsymbol\Sigma_{ij}$，一阶下 (39)），故 IMU 因子 = 残差 (45) 的马氏二次型。整体 MAP（式 26）= 视觉重投影项 + IMU 预积分项 + bias 随机游走项 (48) 之和，经因子图增量平滑（iSAM2 等）求解；视觉用**无结构（structureless）**因子（消去 3D 点，见 §VII，本抽取略）。

## §S1.7 附录 A：迭代噪声传播 [源 Appendix A]

> 把 §S1.5.2 的整段求和改写为**逐测量递推**，便于在线（新测量到达只更新 $\boldsymbol\Sigma_{ij}$）。

**旋转噪声递推**：从 (42) 取出末项（$k=j-1$）重排：
$$\delta\boldsymbol\phi_{ij}\simeq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t\tag{59}$$
$$=\sum_{k=i}^{j-2}\Delta\tilde{\mathbf R}_{k+1\,j}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t+\underbrace{\Delta\tilde{\mathbf R}_{jj}^\top}_{=\mathbf I_{3\times3}}\mathbf J_r^{j-1}\boldsymbol\eta^{gd}_{j-1}\Delta t$$
$$=\sum_{k=i}^{j-2}(\underbrace{\Delta\tilde{\mathbf R}_{k+1\,j-1}\Delta\tilde{\mathbf R}_{j-1\,j}}_{=\Delta\tilde{\mathbf R}_{k+1\,j}})^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t+\mathbf J_r^{j-1}\boldsymbol\eta^{gd}_{j-1}\Delta t$$
$$=\Delta\tilde{\mathbf R}_{j-1\,j}^\top\sum_{k=i}^{j-2}\Delta\tilde{\mathbf R}_{k+1\,j-1}^\top\mathbf J_r^k\boldsymbol\eta^{gd}_k\Delta t+\mathbf J_r^{j-1}\boldsymbol\eta^{gd}_{j-1}\Delta t$$
$$=\Delta\tilde{\mathbf R}_{j-1\,j}^\top\,\delta\boldsymbol\phi_{ij-1}+\mathbf J_r^{j-1}\boldsymbol\eta^{gd}_{j-1}\Delta t.$$

**速度噪声递推**（对 (43)）：
$$\delta\mathbf v_{ij}=\sum_{k=i}^{j-1}\Big[-\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ik}\Delta t+\Delta\tilde{\mathbf R}_{ik}\boldsymbol\eta^{ad}_k\Delta t\Big]\tag{60}$$
$$=\delta\mathbf v_{ij-1}-\Delta\tilde{\mathbf R}_{ij-1}(\tilde{\mathbf a}_{j-1}-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ij-1}\Delta t+\Delta\tilde{\mathbf R}_{ij-1}\boldsymbol\eta^{ad}_{j-1}\Delta t.$$

**位置噪声递推**（对 (43)，注意 $\delta\mathbf p$ 可由 $\delta\mathbf v$ 表）：
$$\delta\mathbf p_{ij}=\delta\mathbf p_{ij-1}+\delta\mathbf v_{ij-1}\Delta t-\tfrac12\Delta\tilde{\mathbf R}_{ij-1}(\tilde{\mathbf a}_{j-1}-\mathbf b^a_i)^\wedge\delta\boldsymbol\phi_{ij-1}\Delta t^2+\tfrac12\Delta\tilde{\mathbf R}_{ij-1}\boldsymbol\eta^{ad}_{j-1}\Delta t^2.\tag{61}$$

**紧凑矩阵形式**（记 $\boldsymbol\eta^\Delta_{ik}\triangleq[\delta\boldsymbol\phi_{ik},\delta\mathbf v_{ik},\delta\mathbf p_{ik}]$，$\boldsymbol\eta^d_k\triangleq[\boldsymbol\eta^{gd}_k\ \boldsymbol\eta^{ad}_k]$）：
$$\boxed{\ \boldsymbol\eta^\Delta_{ij}=\mathbf A_{j-1}\boldsymbol\eta^\Delta_{ij-1}+\mathbf B_{j-1}\boldsymbol\eta^d_{j-1}\ }\tag{62}$$
**协方差迭代**（给定原始 IMU 噪声协方差 $\boldsymbol\Sigma_\eta\in\mathbb R^{6\times6}$）：
$$\boxed{\ \boldsymbol\Sigma_{ij}=\mathbf A_{j-1}\boldsymbol\Sigma_{ij-1}\mathbf A_{j-1}^\top+\mathbf B_{j-1}\boldsymbol\Sigma_\eta\mathbf B_{j-1}^\top\ }\tag{63}$$
初值 $\boldsymbol\Sigma_{ii}=\mathbf 0_{9\times9}$。

> **$\mathbf A,\mathbf B$ 块结构**（由 (59)–(61) 读出，$9\times9$ 与 $9\times6$，本书综合时可显式列出）：
> $$\mathbf A_{j-1}=\begin{bmatrix}\Delta\tilde{\mathbf R}_{j-1\,j}^\top&\mathbf 0&\mathbf 0\\-\Delta\tilde{\mathbf R}_{ij-1}(\tilde{\mathbf a}_{j-1}-\mathbf b^a_i)^\wedge\Delta t&\mathbf I&\mathbf 0\\-\tfrac12\Delta\tilde{\mathbf R}_{ij-1}(\tilde{\mathbf a}_{j-1}-\mathbf b^a_i)^\wedge\Delta t^2&\mathbf I\Delta t&\mathbf I\end{bmatrix},\quad \mathbf B_{j-1}=\begin{bmatrix}\mathbf J_r^{j-1}\Delta t&\mathbf 0\\\mathbf 0&\Delta\tilde{\mathbf R}_{ij-1}\Delta t\\\mathbf 0&\tfrac12\Delta\tilde{\mathbf R}_{ij-1}\Delta t^2\end{bmatrix}.$$
> （上块矩阵为本抽取据 (59)–(61) 系数整理，标 `\rebuilt` 待综合时核对；行列顺序 $[\delta\boldsymbol\phi;\delta\mathbf v;\delta\mathbf p]$，输入顺序 $[\boldsymbol\eta^{gd};\boldsymbol\eta^{ad}]$。）

## §S1.8 附录 B：bias 一阶修正完整推导 [源 Appendix B]

设旧 bias 估计 $\bar{\mathbf b}_i\triangleq[\bar{\mathbf b}^g_i\ \bar{\mathbf b}^a_i]$，对应预积分量
$$\Delta\bar{\mathbf R}_{ij}\triangleq\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}_i),\quad\Delta\bar{\mathbf v}_{ij}\triangleq\Delta\tilde{\mathbf v}_{ij}(\bar{\mathbf b}_i),\quad\Delta\bar{\mathbf p}_{ij}\triangleq\Delta\tilde{\mathbf p}_{ij}(\bar{\mathbf b}_i).\tag{64}$$
新估计 $\hat{\mathbf b}_i\leftarrow\bar{\mathbf b}_i+\delta\mathbf b_i$（小修正）。

**旋转 bias 修正**：把 $\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)$ 表为 $\Delta\bar{\mathbf R}_{ij}$ + 一阶修正。由 (35)：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\hat{\mathbf b}^g_i)\Delta t\big).\tag{65}$$
代入 $\hat{\mathbf b}_i=\bar{\mathbf b}_i+\delta\mathbf b_i$，每因子用一阶近似 (4)：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)=\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-(\bar{\mathbf b}^g_i+\delta\mathbf b^g_i))\Delta t\big)\simeq\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}^g_i)\Delta t\big)\,\mathrm{Exp}(-\mathbf J_r^k\delta\mathbf b^g_i\Delta t).\tag{66}$$
用 (11) 把含 $\delta\mathbf b^g_i$ 项移到末端：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)=\Delta\bar{\mathbf R}_{ij}\prod_{k=i}^{j-1}\mathrm{Exp}\big(-\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\mathbf J_r^k\delta\mathbf b^g_i\Delta t\big),\tag{67}$$
（用 $\Delta\bar{\mathbf R}_{ij}=\prod_{k=i}^{j-1}\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\bar{\mathbf b}^g_i)\Delta t)$）。反复用一阶近似 (7)：
$$\Delta\tilde{\mathbf R}_{ij}(\hat{\mathbf b}_i)\simeq\Delta\bar{\mathbf R}_{ij}\,\mathrm{Exp}\Big(\sum_{k=i}^{j-1}-\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\mathbf J_r^k\delta\mathbf b^g_i\Delta t\Big)=\Delta\bar{\mathbf R}_{ij}\,\mathrm{Exp}\Big(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i\Big).\tag{68}$$

**速度 bias 修正**：
$$\Delta\tilde{\mathbf v}_{ij}(\hat{\mathbf b}_i)=\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\hat{\mathbf b}_i)(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i-\delta\mathbf b^a_i)\Delta t\tag{69}$$
$$\stackrel{(68)}{\simeq}\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\,\mathrm{Exp}\Big(\tfrac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\delta\mathbf b^g_i\Big)(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i-\delta\mathbf b^a_i)\Delta t$$
$$\stackrel{(4)}{\simeq}\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Big(\mathbf I+\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\delta\mathbf b^g_i\big)^\wedge\Big)(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i-\delta\mathbf b^a_i)\Delta t$$
$$\stackrel{(a)}{\simeq}\Delta\bar{\mathbf v}_{ij}-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Delta t\,\delta\mathbf b^a_i+\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\delta\mathbf b^g_i\big)^\wedge(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i)\Delta t$$
$$\stackrel{(2)}{=}\Delta\bar{\mathbf v}_{ij}-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Delta t\,\delta\mathbf b^a_i-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i)^\wedge\tfrac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\Delta t\,\delta\mathbf b^g_i$$
$$=\Delta\bar{\mathbf v}_{ij}+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}\delta\mathbf b^a_i+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i.$$
其中 (a) 用 $\Delta\bar{\mathbf v}_{ij}=\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i)\Delta t$，并弃二阶 $\delta\mathbf b$ 项。位置 $\Delta\tilde{\mathbf p}_{ij}(\hat{\mathbf b}_i)$ 同法可得。

**bias 雅可比汇总**（式 44 用）：
$$\boxed{\begin{aligned}\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}&=-\sum_{k=i}^{j-1}\Big(\Delta\tilde{\mathbf R}_{k+1\,j}(\bar{\mathbf b}_i)^\top\mathbf J_r^k\Delta t\Big),\\\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a}&=-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}\Delta t,\\\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g}&=-\sum_{k=i}^{j-1}\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i)^\wedge\frac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\Delta t,\\\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a}&=\sum_{k=i}^{j-1}\Big[\frac{\partial\Delta\bar{\mathbf v}_{ik}}{\partial\mathbf b^a}\Delta t-\tfrac12\Delta\bar{\mathbf R}_{ik}\Delta t^2\Big],\\\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g}&=\sum_{k=i}^{j-1}\Big[\frac{\partial\Delta\bar{\mathbf v}_{ik}}{\partial\mathbf b^g}\Delta t-\tfrac12\Delta\bar{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\bar{\mathbf b}^a_i)^\wedge\frac{\partial\Delta\bar{\mathbf R}_{ik}}{\partial\mathbf b^g}\Delta t^2\Big].\end{aligned}}$$
均可随测量到达**增量计算**。

## §S1.9 附录 C：残差雅可比完整推导 [源 Appendix C]

**Lifting**（代入 retraction，使残差成为向量空间上的函数）：
$$\begin{aligned}&\mathbf R_i\leftarrow\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i),\quad\mathbf R_j\leftarrow\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_j),\\&\mathbf p_i\leftarrow\mathbf p_i+\mathbf R_i\delta\mathbf p_i,\quad\mathbf p_j\leftarrow\mathbf p_j+\mathbf R_j\delta\mathbf p_j,\\&\mathbf v_i\leftarrow\mathbf v_i+\delta\mathbf v_i,\quad\mathbf v_j\leftarrow\mathbf v_j+\delta\mathbf v_j,\\&\delta\mathbf b^g_i\leftarrow\delta\mathbf b^g_i+\tilde{\delta}\mathbf b^g_i,\quad\delta\mathbf b^a_i\leftarrow\delta\mathbf b^a_i+\tilde{\delta}\mathbf b^a_i.\end{aligned}\tag{70}$$

### §S1.9.1 $\mathbf r_{\Delta\mathbf p_{ij}}$ 雅可比

记 $\mathbf C\triangleq\Delta\tilde{\mathbf p}_{ij}+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g_i}\delta\mathbf b^g_i+\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a_i}\delta\mathbf b^a_i$。$\mathbf r_{\Delta\mathbf p_{ij}}$ 对 $\delta\mathbf b$ 线性、retraction 是向量和，故对 $\tilde{\delta}\mathbf b^g_i,\tilde{\delta}\mathbf b^a_i$ 雅可比即系数矩阵；$\mathbf R_j,\mathbf v_j$ 不出现 → 对 $\delta\boldsymbol\phi_j,\delta\mathbf v_j$ 雅可比为零。其余：
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_i+\mathbf R_i\delta\mathbf p_i)=\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf R_i\delta\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-\mathbf C=\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_i)+(-\mathbf I)\delta\mathbf p_i,\tag{71}$$
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_j+\mathbf R_j\delta\mathbf p_j)=\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf p_j)+(\mathbf R_i^\top\mathbf R_j)\delta\mathbf p_j,\tag{72}$$
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf v_i+\delta\mathbf v_i)=\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf v_i)+(-\mathbf R_i^\top\Delta t_{ij})\delta\mathbf v_i,\tag{73}$$
$$\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))\stackrel{(4)}{\simeq}(\mathbf I-\delta\boldsymbol\phi_i^\wedge)\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)-\mathbf C\stackrel{(2)}{=}\mathbf r_{\Delta\mathbf p_{ij}}(\mathbf R_i)+\big(\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)\big)^\wedge\delta\boldsymbol\phi_i.\tag{74}$$
**汇总**：
$$\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\boldsymbol\phi_i}=\big(\mathbf R_i^\top(\mathbf p_j-\mathbf p_i-\mathbf v_i\Delta t_{ij}-\tfrac12\mathbf g\Delta t_{ij}^2)\big)^\wedge,\quad\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\boldsymbol\phi_j}=\mathbf 0,$$
$$\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf p_i}=-\mathbf I,\quad\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf p_j}=\mathbf R_i^\top\mathbf R_j,\quad\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf v_i}=-\mathbf R_i^\top\Delta t_{ij},\quad\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\delta\mathbf v_j}=\mathbf 0,$$
$$\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\tilde{\delta}\mathbf b^a_i}=-\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^a_i},\quad\frac{\partial\mathbf r_{\Delta\mathbf p_{ij}}}{\partial\tilde{\delta}\mathbf b^g_i}=-\frac{\partial\Delta\bar{\mathbf p}_{ij}}{\partial\mathbf b^g_i}.$$

### §S1.9.2 $\mathbf r_{\Delta\mathbf v_{ij}}$ 雅可比

记 $\mathbf D\triangleq\big[\Delta\tilde{\mathbf v}_{ij}+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g_i}\delta\mathbf b^g_i+\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a_i}\delta\mathbf b^a_i\big]$。$\mathbf R_j,\mathbf p_i,\mathbf p_j$ 不出现 → 对 $\delta\boldsymbol\phi_j,\delta\mathbf p_i,\delta\mathbf p_j$ 雅可比为零。
$$\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_i+\delta\mathbf v_i)=\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\delta\mathbf v_i-\mathbf g\Delta t_{ij})-\mathbf D=\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_i)-\mathbf R_i^\top\delta\mathbf v_i,\tag{75}$$
$$\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_j+\delta\mathbf v_j)=\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf v_j)+\mathbf R_i^\top\delta\mathbf v_j,\tag{76}$$
$$\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))\stackrel{(4)}{\simeq}(\mathbf I-\delta\boldsymbol\phi_i^\wedge)\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})-\mathbf D\stackrel{(2)}{=}\mathbf r_{\Delta\mathbf v_{ij}}(\mathbf R_i)+\big(\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})\big)^\wedge\delta\boldsymbol\phi_i.\tag{77}$$
**汇总**：
$$\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\boldsymbol\phi_i}=\big(\mathbf R_i^\top(\mathbf v_j-\mathbf v_i-\mathbf g\Delta t_{ij})\big)^\wedge,\quad\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\boldsymbol\phi_j}=\mathbf 0,\quad\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf p_i}=\mathbf 0,\quad\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf p_j}=\mathbf 0,$$
$$\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf v_i}=-\mathbf R_i^\top,\quad\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\delta\mathbf v_j}=\mathbf R_i^\top,\quad\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\tilde{\delta}\mathbf b^a_i}=-\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^a_i},\quad\frac{\partial\mathbf r_{\Delta\mathbf v_{ij}}}{\partial\tilde{\delta}\mathbf b^g_i}=-\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g_i}.$$

### §S1.9.3 $\mathbf r_{\Delta\mathbf R_{ij}}$ 雅可比（较繁）

$\mathbf p_i,\mathbf p_j,\mathbf v_i,\mathbf v_j,\delta\mathbf b^a_i$ 不出现 → 对应雅可比为零。记 $\mathbf E\triangleq\mathrm{Exp}(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g)$，$\mathbf J_r^b\triangleq\mathbf J_r(\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\delta\mathbf b^g_i)$。

对 $\delta\boldsymbol\phi_i$：
$$\mathbf r_{\Delta\mathbf R_{ij}}(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))=\mathrm{Log}\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\mathbf E^\top(\mathbf R_i\,\mathrm{Exp}(\delta\boldsymbol\phi_i))^\top\mathbf R_j\big)=\mathrm{Log}\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\mathbf E^\top\mathrm{Exp}(-\delta\boldsymbol\phi_i)\mathbf R_i^\top\mathbf R_j\big)$$
$$\stackrel{(11)}{=}\mathrm{Log}\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\mathbf E^\top\mathbf R_i^\top\mathbf R_j\,\mathrm{Exp}(-\mathbf R_j^\top\mathbf R_i\delta\boldsymbol\phi_i)\big)\stackrel{(9)}{\simeq}\mathbf r_{\Delta\mathbf R}(\mathbf R_i)-\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_i))\mathbf R_j^\top\mathbf R_i\delta\boldsymbol\phi_i.\tag{78}$$

对 $\delta\boldsymbol\phi_j$：
$$\mathbf r_{\Delta\mathbf R_{ij}}(\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_j))=\mathrm{Log}\big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\mathbf E^\top\mathbf R_i^\top(\mathbf R_j\,\mathrm{Exp}(\delta\boldsymbol\phi_j))\big)\stackrel{(9)}{\simeq}\mathbf r_{\Delta\mathbf R}(\mathbf R_j)+\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_j))\delta\boldsymbol\phi_j.\tag{79}$$

对 $\tilde{\delta}\mathbf b^g_i$：
$$\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i+\tilde{\delta}\mathbf b^g_i)=\mathrm{Log}\Big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathrm{Exp}\big(\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}(\delta\mathbf b^g_i+\tilde{\delta}\mathbf b^g_i)\big)^\top\mathbf R_i^\top\mathbf R_j\Big)\tag{80}$$
$$\stackrel{(7)}{\simeq}\mathrm{Log}\Big(\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\,\mathbf E\,\mathrm{Exp}\big(\mathbf J_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b^g_i\big)^\top\mathbf R_i^\top\mathbf R_j\Big)$$
$$=\mathrm{Log}\Big(\mathrm{Exp}\big(-\mathbf J_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b^g_i\big)\Delta\tilde{\mathbf R}_{ij}(\bar{\mathbf b}^g_i)\mathbf E^\top\mathbf R_i^\top\mathbf R_j\Big)=\mathrm{Log}\Big(\mathrm{Exp}\big(-\mathbf J_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b^g_i\big)\mathrm{Exp}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))\Big)$$
$$\stackrel{(11)}{=}\mathrm{Log}\Big(\mathrm{Exp}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))\cdot\mathrm{Exp}\big(-\mathrm{Exp}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))^\top\mathbf J_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b^g_i\big)\Big)$$
$$\stackrel{(9)}{\simeq}\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i)-\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))\,\mathrm{Exp}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))^\top\mathbf J_r^b\tfrac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}\tilde{\delta}\mathbf b^g_i.$$

**汇总**：
$$\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\boldsymbol\phi_i}=-\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_i))\mathbf R_j^\top\mathbf R_i,\quad\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\boldsymbol\phi_j}=\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R}(\mathbf R_j)),$$
$$\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf p_i}=\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf p_j}=\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf v_i}=\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\delta\mathbf v_j}=\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\tilde{\delta}\mathbf b^a_i}=\mathbf 0,$$
$$\frac{\partial\mathbf r_{\Delta\mathbf R_{ij}}}{\partial\tilde{\delta}\mathbf b^g_i}=\boldsymbol\alpha,\quad\boldsymbol\alpha=-\mathbf J_r^{-1}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))\,\mathrm{Exp}(\mathbf r_{\Delta\mathbf R_{ij}}(\delta\mathbf b^g_i))^\top\mathbf J_r^b\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g}.\tag{81}$$

## §S1.10 Forster 实验中的 bias 一阶修正验证 [源 §VIII]

> 数值例（佐证 §S1.5.3 一阶修正有效）：作者做了额外实验，改变 bias 估计后用一阶修正 (44) 更新预积分，与重积分对比，证明小 bias 变化下一阶修正精度足够（避免重积分）。原文 §VIII-A.4 "First-Order Bias Correction"。具体数值图（位置/旋转误差随 bias change）见原文 Fig. 内；本抽取记其结论：在合理 bias 漂移幅度内，一阶修正与精确重积分误差可忽略。

---

# §S2 Kalibr IMU 噪声模型（连续↔离散）

> **本章重点之一**：连续→离散噪声转换的事实标准。Kalibr Wiki *IMU Noise Model*。

## §S2.1 连续时间测量模型 [源 Kalibr Wiki]

陀螺仪（逐轴独立同构）：
$$\tilde\omega(t)=\omega(t)+b(t)+n(t),$$
加速度计同结构。即"真值 + 缓变 bias $b(t)$ + 白噪声 $n(t)$"，与 Forster (27)(28) 一致（Kalibr 不显式写重力/旋转部分，那是运动学层）。

## §S2.2 四个噪声参数与连续单位 [源 Kalibr Wiki]

| 参数 | 符号 | 连续单位 | 说明 |
|---|---|---|---|
| 陀螺白噪声（角度随机游走 ARW） | $\sigma_g$ | $\frac{\mathrm{rad}}{s}\frac{1}{\sqrt{\mathrm{Hz}}}$ | 加性噪声强度 |
| 加速度计白噪声（速度随机游走 VRW） | $\sigma_a$ | $\frac{m}{s^2}\frac{1}{\sqrt{\mathrm{Hz}}}$ | 加性噪声强度 |
| 陀螺 bias 随机游走 | $\sigma_{b_g}$ | $\frac{\mathrm{rad}}{s^2}\frac{1}{\sqrt{\mathrm{Hz}}}$ | bias 变化强度 |
| 加速度计 bias 随机游走 | $\sigma_{b_a}$ | $\frac{m}{s^3}\frac{1}{\sqrt{\mathrm{Hz}}}$ | bias 变化强度 |

## §S2.3 自相关 / 功率谱密度 [源 Kalibr Wiki]

$$E[n(t)]\equiv0,\qquad E[n(t_1)n(t_2)]=\sigma_g^2\,\delta(t_1-t_2),$$
$\delta$ 为 Dirac delta。即连续白噪声功率谱密度为常数 $\sigma_g^2$（自相关为冲激）。

## §S2.4 连续→离散转换（核心）[源 Kalibr Wiki]

**白噪声**：
$$\boxed{\ \sigma_{g_d}=\sigma_g\frac{1}{\sqrt{\Delta t}}\ }$$
**bias 随机游走**：
$$\boxed{\ \sigma_{bg_d}=\sigma_{b_g}\sqrt{\Delta t}\ }$$
$\Delta t$ 为采样周期。

**离散模型**：
$$n_d[k]=\sigma_{g_d}\,w[k],\quad w[k]\sim\mathcal N(0,1),$$
$$b_d[k]=b_d[k-1]+\sigma_{bg_d}\,w[k],\quad w[k]\sim\mathcal N(0,1).$$

**直觉总结**（源原话）：白噪声方差随 $\Delta t$ **反比**缩放（$\propto1/\sqrt{\Delta t}$ on σ，即 $1/\Delta t$ on σ²）；bias 随机游走随 $\Delta t$ **正比**缩放（$\propto\sqrt{\Delta t}$ on σ）。

> **与 Forster 互证**：Forster $\mathrm{Cov}(\boldsymbol\eta^{gd})=\frac1{\Delta t}\mathrm{Cov}(\boldsymbol\eta^g)$ ↔ Kalibr $\sigma_{g_d}^2=\sigma_g^2/\Delta t$（一致）；Forster bias $\boldsymbol\Sigma^{bgd}=\Delta t\,\mathrm{Cov}(\boldsymbol\eta^{bg})$ ↔ Kalibr $\sigma_{bg_d}^2=\sigma_{b_g}^2\Delta t$（一致）。**这是连续↔离散两套写法的统一点，本书须显式给出。**
>
> **陷阱（综合 agent 注意）**：Kalibr 的 $\sigma_g$ 单位是 $\mathrm{rad}/s/\sqrt{\mathrm{Hz}}=\mathrm{rad}/\sqrt s$（连续谱密度的平方根），与数据手册的 ARW（$\mathrm{rad}/\sqrt h$ 或 $^\circ/\sqrt h$）相差单位换算；与 Allan 方差的 $N$ 系数关系见 §S6。

---

# §S3 Woodman《惯性导航导论》（UCAM-CL-TR-696）

> **本章重点**：误差源分类（bias/白噪声/标定/温度）、Allan、捷联机理（姿态/速度/位置积分）、**重力与坐标系**、漂移定量。本节用方向余弦阵 $\mathbf C$（本书改记 $\mathbf R$，见 §M）。

## §S3.0 记号约定（Woodman 本源）

- 旋转用**方向余弦阵 $\mathbf C$**（$3\times3$）；下标 $b$=body 系、$g$=global 系。
- $\mathbf v_g=\mathbf C\mathbf v_b$（body→global），逆 $\mathbf v_b=\mathbf C^\top\mathbf v_g$。
- $\boldsymbol\omega_b=(\omega_{bx},\omega_{by},\omega_{bz})^\top$ 角速度，$\mathbf a_b$ 加速度，$\mathbf g_g$ 重力（global）。

## §S3.1 惯性导航与系统配置 [源 §2]

惯导 = 用加速度计/陀螺测量跟踪物体相对已知起点的位置/姿态/速度，自包含。IMU 含三正交速率陀螺 + 三正交加速度计。两类配置：

**稳定平台（Stable Platform）** [§2.1.1]：传感器装在被陀螺反馈隔离外部转动的平台上（万向架），平台保持与 global 系对齐。姿态读万向架角；位置由平台加速度计**二次积分**（先减重力）。

**捷联（Strapdown）** [§2.1.2]：传感器刚性固连设备，输出 **body 系**量。姿态由陀螺信号"积分"得（§6）；位置先用姿态把加速度投到 global 系，减重力，再二次积分。捷联机械简单、体积小，代价是计算量大；随算力廉价成为主流。

## §S3.2 陀螺误差特性 [源 §3.2]

### §S3.2.1 常值 bias [§3.2.1]
bias = 无转动时陀螺平均输出（偏移），单位 $^\circ/h$。常值 bias $\epsilon$ 积分后**角度误差线性增长**：
$$\theta(t)=\epsilon\cdot t.$$
可由长时静止平均估计、相减补偿。

### §S3.2.2 热机械白噪声 / 角度随机游走 ARW [§3.2.2]
MEMS 陀螺输出受远高于采样率的热机械噪声 → 白噪声序列（零均值不相关、有限方差 $\sigma^2$）。设 $N_i$ 为白噪声序列第 $i$ 项，$E(N_i)=E(N)=0$，$\mathrm{Var}(N_i)=\sigma^2$，$\mathrm{Cov}(N_i,N_j)=0\ (i\ne j)$。**矩形法积分**白噪声 $\epsilon(t)$（$t=n\delta t$）：
$$\int_0^t\epsilon(\tau)d\tau=\delta t\sum_{i=1}^nN_i.\tag{2}$$
用 $E(aX+bY)=aE(X)+bE(Y)$、$\mathrm{Var}(aX+bY)=a^2\mathrm{Var}(X)+b^2\mathrm{Var}(Y)+2ab\,\mathrm{Cov}(X,Y)$：
$$E\Big[\int_0^t\epsilon\,d\tau\Big]=\delta t\cdot n\cdot E(N)=0,\tag{3}$$
$$\mathrm{Var}\Big[\int_0^t\epsilon\,d\tau\Big]=\delta t^2\cdot n\cdot\mathrm{Var}(N)=\delta t\cdot t\cdot\sigma^2.\tag{4}$$
即引入零均值随机游走，标准差
$$\boxed{\ \sigma_\theta(t)=\sigma\cdot\sqrt{\delta t\cdot t}\ }\tag{5}$$
**正比 $\sqrt t$ 增长**。厂商用 **ARW** 规定噪声：
$$\mathrm{ARW}=\sigma_\theta(1),\tag{6}$$
单位 $^\circ/\sqrt h$。**数值例**：Honeywell GG5300 的 ARW=$0.2^\circ/\sqrt h$ → 1h 后姿态误差标准差 $0.2^\circ$，2h 后 $\sqrt2\cdot0.2=0.28^\circ$。其他规格：PSD（单位 $(^\circ/h)^2/\mathrm{Hz}$）、FFT 噪声密度（单位 $^\circ/h/\sqrt{\mathrm{Hz}}$）。换算：
$$\mathrm{ARW}(^\circ/\sqrt h)=\frac{1}{60}\sqrt{\mathrm{PSD}((^\circ/h)^2/\mathrm{Hz})},\tag{7}$$
$$\mathrm{ARW}(^\circ/\sqrt h)=\frac{1}{60}\cdot\mathrm{FFT}(^\circ/h/\sqrt{\mathrm{Hz}}).\tag{8}$$

### §S3.2.3 闪烁噪声 / bias 稳定性 (Bias Stability) [§3.2.3]
MEMS 陀螺 bias 因电子学等闪烁噪声（$1/f$ 谱，低频显著）漂移；常建模为**随机游走**。bias 稳定性 = 规定时段（典型 ~100s）固定条件（恒温）下 bias 变化量，1σ，单位 $^\circ/h$（差器件 $^\circ/s$）。随机游走解释：$B_t$ 为 $t$ 时已知 bias，1σ 稳定性 $0.01^\circ/h$ over 100s → $t+100$s 时 bias 期望 $B_t$、标准差 $0.01^\circ/h$。随时间形成 bias 随机游走，标准差 $\propto\sqrt t$。故有时用 **bias 随机游走 BRW** 规定：
$$\boxed{\ \mathrm{BRW}(^\circ/\sqrt h)=\frac{\mathrm{BS}(^\circ/h)}{\sqrt{t(h)}}\ }\tag{9}$$
$t$ 为 bias 稳定性定义的时段。若 bias 服从随机游走模型，积分 bias 涨落 → **角度的二阶随机游走**。现实中 bias 涨落不真服从随机游走（否则不确定度无界增长），实际 bias 受限于某范围，故随机游走仅短时近似好。

### §S3.2.4 温度效应 [§3.2.4]
环境变化+自热 → bias 移动（不含于固定条件下的 bias 稳定性）。残余温度 bias → 姿态误差**线性增长**（§3.2.1）。bias-温度关系对 MEMS 常高度非线性；多数 IMU 含内置温度传感器可补偿（如 Xsens Mtx 内部补偿）。

### §S3.2.5 标定误差 (Calibration Errors) [§3.2.5]
统指 scale factor、轴对准（alignment）、线性度误差。这类误差**只在转动时显现**为 bias 误差，导致额外漂移，幅度正比转动率与持续时间 [4]。通常可测量/校正（如 Xsens Mtx 内部校正）。

### §S3.2.6 陀螺误差源汇总表 [源 Table 2]

| 误差类型 | 描述 | 积分结果 |
|---|---|---|
| Bias | 常值 bias $\epsilon$ | 稳增角度误差 $\theta(t)=\epsilon\cdot t$ |
| 白噪声 | 标准差 $\sigma$ 的白噪声 | 角度随机游走 $\sigma_\theta(t)=\sigma\sqrt{\delta t\cdot t}$，$\propto\sqrt t$ |
| 温度效应 | 温度相关残余 bias | 残余 bias 积分入姿态，误差线性增长 |
| 标定 | scale/对准/线性度的确定性误差 | 姿态漂移正比转动率与持续时间 |
| Bias 不稳定 | bias 涨落，常建模为 bias 随机游走 | 二阶随机游走 |

> **MEMS 主导误差（源结论）**：ARW（噪声）与未校正 bias（温度未补偿或初始 bias 估计误差）通常最重要。ARW 可作姿态不确定度下界。

## §S3.3 加速度计误差特性 [源 §4.2]

> 与陀螺类似，但**二次积分**（陀螺只积分一次）。

### §S3.3.1 常值 bias [§4.2.1]
bias = 输出偏移，单位 $m/s^2$。常值 bias $\epsilon$ **二次积分**后位置误差**二次增长**：
$$\boxed{\ s(t)=\epsilon\cdot\frac{t^2}{2}\ }\tag{10}$$
可由静止长时平均估 bias，但**受重力复杂化**（重力分量在加速度计上表现为 bias），须精确已知设备相对重力场的姿态（实践用转台校准）。

### §S3.3.2 热机械白噪声 / 速度随机游走 VRW [§4.2.2]
白噪声经**二次积分**。设 $N_i$ 白噪声序列，$E(N_i)=0$，$\mathrm{Var}(N_i)=\sigma^2$。二次积分 $\epsilon(t)$（$t=n\delta t$）：
$$\int_0^t\!\!\int_0^t\epsilon(\tau)d\tau d\tau=\delta t\sum_{i=1}^n\delta t\sum_{j=1}^iN_j\tag{11}=\delta t^2\sum_{i=1}^n(n-i+1)N_i.\tag{12}$$
期望：
$$E\Big[\int\!\!\int\epsilon\Big]=\delta t^2\sum_{i=1}^n(n-i+1)E(N_i)=0,\tag{13,14}$$
方差：
$$\mathrm{Var}\Big[\int\!\!\int\epsilon\Big]=\delta t^4\sum_{i=1}^n(n-i+1)^2\mathrm{Var}(N_i)=\frac{\delta t^4 n(n+1)(2n+1)}{6}\mathrm{Var}(N)\approx\frac13\delta t\cdot t^3\cdot\sigma^2,\tag{15,16,17}$$
（近似设 $\delta t$ 小/采样率高）。即**位置的二阶随机游走**，零均值，标准差
$$\boxed{\ \sigma_s(t)\approx\sigma\cdot t^{3/2}\sqrt{\frac{\delta t}{3}}\ }\tag{18}$$
$\propto t^{3/2}$。VRW 单位 $m/s/\sqrt h$。

### §S3.3.3 闪烁噪声 / bias 稳定性 [§4.2.3]
闪烁噪声→ bias 漂移，建模为 bias 随机游走（同 §3.2.3）。此模型下：**速度的二阶随机游走**（$\propto t^{3/2}$）+ **位置的三阶随机游走**（$\propto t^{5/2}$）。

### §S3.3.4 温度效应 [§4.2.4]
温度→ bias 涨落（常高度非线性）。残余 bias → 位置误差二次增长（§4.2.1）。有温度传感器可补偿。

### §S3.3.5 标定误差 [§4.2.5]
（源此处仅指与陀螺类比，scale/对准/线性度。）

> **加速度计误差汇总**（本抽取据 §4.2 整理，对应陀螺 Table 2 的加速度计版）：bias→位置二次增长 $\epsilon t^2/2$；白噪声→位置二阶随机游走 $\propto t^{3/2}$；bias 不稳定→速度二阶($t^{3/2}$)、位置三阶($t^{5/2}$)随机游走；温度/标定同陀螺。

## §S3.4 Allan 方差（Woodman 版）[源 §5]

> Woodman 的 Allan 部分较简，详尽五项理论见 §S6（Calgary/IEEE 952）。此处记其定义、读数法、Xsens Mtx 实测数值例。

**Allan 方差定义** [§5.1]：averaging time $t$，
1. 长序列分成长 $t$ 的 bin（≥9 个 bin）；
2. 各 bin 内平均 → $(a(t)_1,\dots,a(t)_n)$，$n$=bin 数；
3. 
$$\boxed{\ \mathrm{AVAR}(t)=\frac{1}{2(n-1)}\sum_i(a(t)_{i+1}-a(t)_i)^2\ }\tag{19}$$
**Allan deviation** $\mathrm{AD}(t)=\sqrt{\mathrm{AVAR}(t)}$（式 20），log-log 作图 vs $t$。不同随机过程产生不同斜率、出现在不同 $t$ 区域，便于辨识与读数。

**读数法**（MEMS 关注随机游走与 bias 不稳定）：
- **白噪声**：log-log 上斜率 $-0.5$；随机游走系数（陀螺 ARW / 加速度 VRW）= 拟合该斜率线在 $t=1$ 处的值。
- **Bias 不稳定**：图上**极小值附近的平坦区**；数值 = Allan deviation 曲线最小值。

**数值例（Xsens Mtx）** [§5.2]：12 小时静止、100Hz 采样。陀螺左侧斜率 ≈ $-0.5$（白噪声）。

陀螺噪声测量（Table 4）：

| 轴 | Bias 不稳定 | 角度随机游走 ARW |
|---|---|---|
| X | $0.010^\circ/s=36^\circ/h$（at 620s） | $0.075^\circ/\sqrt s=4.6^\circ/\sqrt h$ |
| Y | $0.009^\circ/s=32^\circ/h$（at 530s） | $0.078^\circ/\sqrt s=4.8^\circ/\sqrt h$ |
| Z | $0.012^\circ/s=43^\circ/h$（at 270s） | $0.079^\circ/\sqrt s=4.8^\circ/\sqrt h$ |

（加速度计 Allan 见原文 Fig.12 / Table 5，本抽取记其存在；数值随源未全列。）

## §S3.5 捷联机理：姿态跟踪 [源 §6.1]

> **本章重点**：连续运动学的方向余弦阵推导 + 离散更新闭式。

**理论** [§6.1.1]：用方向余弦阵 $\mathbf C$（每列=body 轴在 global 表达的单位向量）。$\mathbf v_g=\mathbf C\mathbf v_b$（21），$\mathbf v_b=\mathbf C^\top\mathbf v_g$（22）。$\mathbf C$ 随时间变化：
$$\dot{\mathbf C}(t)=\lim_{\delta t\to0}\frac{\mathbf C(t+\delta t)-\mathbf C(t)}{\delta t},\tag{23}$$
$\mathbf C(t+\delta t)=\mathbf C(t)\mathbf A(t)$（24），$\mathbf A(t)$ = body 系从 $t$ 到 $t+\delta t$ 的旋转。小角近似（附录 A）：
$$\mathbf A(t)=\mathbf I+\delta\boldsymbol\Psi,\tag{25}\qquad\delta\boldsymbol\Psi=\begin{bmatrix}0&-\delta\psi&\delta\theta\\\delta\psi&0&-\delta\phi\\-\delta\theta&\delta\phi&0\end{bmatrix},\tag{26}$$
$\delta\phi,\delta\theta,\delta\psi$ 为绕 body $x,y,z$ 轴的小转角。代入：
$$\dot{\mathbf C}(t)=\lim_{\delta t\to0}\frac{\mathbf C(t)(\mathbf I+\delta\boldsymbol\Psi)-\mathbf C(t)}{\delta t}=\mathbf C(t)\lim_{\delta t\to0}\frac{\delta\boldsymbol\Psi}{\delta t}.\tag{27-30}$$
极限下小角有效：
$$\lim_{\delta t\to0}\frac{\delta\boldsymbol\Psi}{\delta t}=\boldsymbol\Omega(t),\tag{31}\qquad\boldsymbol\Omega(t)=\begin{bmatrix}0&-\omega_{bz}(t)&\omega_{by}(t)\\\omega_{bz}(t)&0&-\omega_{bx}(t)\\-\omega_{by}(t)&\omega_{bx}(t)&0\end{bmatrix}\tag{32}$$
为角速度向量 $\boldsymbol\omega_b(t)$ 的反对称形式。故姿态微分方程：
$$\boxed{\ \dot{\mathbf C}(t)=\mathbf C(t)\boldsymbol\Omega(t)\ }\tag{33}$$
解：
$$\mathbf C(t)=\mathbf C(0)\cdot\exp\Big(\int_0^t\boldsymbol\Omega(t)\,dt\Big),\tag{34}$$
$\mathbf C(0)$ 初始姿态。

> **与 Forster (29) 一致**：$\dot{\mathbf R}_{WB}=\mathbf R_{WB}\boldsymbol\omega^\wedge$（$\mathbf C\equiv\mathbf R_{WB}$、$\boldsymbol\Omega\equiv\boldsymbol\omega^\wedge$）。

**实现（离散更新）** [§6.1.2]：IMU 给采样（周期 $\delta t$）。单周期 $[t,t+\delta t]$：
$$\mathbf C(t+\delta t)=\mathbf C(t)\cdot\exp\Big(\int_t^{t+\delta t}\boldsymbol\Omega(t)dt\Big),\tag{35}$$
矩形法 $\int_t^{t+\delta t}\boldsymbol\Omega dt=\mathbf B$（36），
$$\mathbf B=\begin{bmatrix}0&-\omega_{bz}\delta t&\omega_{by}\delta t\\\omega_{bz}\delta t&0&-\omega_{bx}\delta t\\-\omega_{by}\delta t&\omega_{bx}\delta t&0\end{bmatrix}.\tag{37}$$
令 $\sigma=|\boldsymbol\omega_b\delta t|$，代入 (35) 并 Taylor 展开指数：
$$\mathbf C(t+\delta t)=\mathbf C(t)\Big(\mathbf I+\mathbf B+\frac{\mathbf B^2}{2!}+\frac{\mathbf B^3}{3!}+\frac{\mathbf B^4}{4!}+\dots\Big)\tag{38}$$
$$=\mathbf C(t)\Big(\mathbf I+\mathbf B+\frac{\mathbf B^2}{2!}-\frac{\sigma^2\mathbf B}{3!}-\frac{\sigma^2\mathbf B^2}{4!}+\dots\Big)\tag{39}$$
（用 $\mathbf B^3=-\sigma^2\mathbf B$ 等反对称阵幂性质）
$$=\mathbf C(t)\Big(\mathbf I+\big(1-\tfrac{\sigma^2}{3!}+\tfrac{\sigma^4}{5!}\dots\big)\mathbf B+\big(\tfrac{1}{2!}-\tfrac{\sigma^2}{4!}+\tfrac{\sigma^4}{6!}\dots\big)\mathbf B^2\Big)\tag{40}$$
$$\boxed{\ \mathbf C(t+\delta t)=\mathbf C(t)\Big(\mathbf I+\frac{\sin\sigma}{\sigma}\mathbf B+\frac{1-\cos\sigma}{\sigma^2}\mathbf B^2\Big)\ }\tag{41}$$
即**姿态更新方程**（本质 = Rodrigues 公式，对照 Forster (3)）。

**误差传播** [§6.1.3]：陀螺误差经积分入姿态。MEMS 主要：白噪声→角度随机游走（$\propto\sqrt t$）、未校正 bias→线性增长。还有量化误差（角速度采样+积分方案量化）。

## §S3.6 捷联机理：位置跟踪 [源 §6.2]

**理论** [§6.2.1]：加速度投到 global：
$$\mathbf a_g(t)=\mathbf C(t)\mathbf a_b(t).\tag{42}$$
减重力，积分一次得速度、再积分得位移：
$$\mathbf v_g(t)=\mathbf v_g(0)+\int_0^t(\mathbf a_g(t)-\mathbf g_g)\,dt,\tag{43}$$
$$\mathbf s_g(t)=\mathbf s_g(0)+\int_0^t\mathbf v_g(t)\,dt,\tag{44}$$
$\mathbf g_g$ = global 系重力加速度。

**实现（矩形法）** [§6.2.2]：
$$\mathbf v_g(t+\delta t)=\mathbf v_g(t)+\delta t\cdot(\mathbf a_g(t+\delta t)-\mathbf g_g),\tag{45}$$
$$\mathbf s_g(t+\delta t)=\mathbf s_g(t)+\delta t\cdot\mathbf v_g(t+\delta t).\tag{46}$$

**误差传播 + 重力投影（重要）** [§6.2.3]：加速度计误差经二次积分 → 位置漂移主因。姿态误差也致位置漂移（$\mathbf C$ 用于投影加速度）：姿态错→ 加速度投影方向错；重力不能正确扣除。

> **重力-倾斜耦合（关键定量，源原文）**：捷联中从（global）竖直加速度减 $1g$。**倾斜误差 $\epsilon$ 会把幅度 $g\sin(\epsilon)$ 的重力分量投到水平轴**，在水平加速度留下幅度 $g\sin(\epsilon)$ 的残余 bias；竖直轴残余 bias 幅度 $g(1-\cos(\epsilon))$（小 $\epsilon$ 时小得多，$\cos\epsilon\to1$、$\sin\epsilon\to\epsilon$）。故小倾斜误差致的位置误差主要在 global xy 平面。

> **数值例（源 Abstract/§1）**：基于 Xsens Mtx 的简单惯导，**60s 后位置误差 >150m**；姿态误差（陀螺白噪声驱动）是漂移的关键主因——陀螺误差经姿态投影传到位置，是几乎所有 INS 的关键误差路径。

---

# §S4 VINS-Mono 四元数离散预积分（arXiv:1708.03852）

> **本章重点**：四元数版离散预积分（α/β/γ）+ 误差态连续动态 F/G + 离散协方差/雅可比递推。与 §S1（Forster 旋转矩阵 + 迭代式）互为对照——两者数学等价、实现风格不同。

## §S4.0 记号约定（VINS 本源）

- 旋转 $\mathbf R$ + Hamilton 四元数 $\mathbf q$（实部在前书写 $\mathbf q=[q_w,\mathbf q_v]$；代码存储 $[\mathbf q_v,q_w]$）。$\mathbf R(\gamma)$ = 四元数 $\gamma$ 对应旋转阵。
- $w$=world、$b_k$=第 $k$ 帧 body 系。${}^{b_k}_w\mathbf R$、${}^{b_k}_w\mathbf q$。
- 测量带帽 $\hat{\mathbf a},\hat{\boldsymbol\omega}$（VINS 用 $\hat{(\cdot)}$ 表"读数减去名义 bias 后"）。
- $\otimes$ 四元数乘；$\Omega(\boldsymbol\omega)$ 见下。
- bias $b_a,b_w$（加速度/陀螺）；加性白噪声 $n_a,n_w$；bias 随机游走驱动 $n_{ba},n_{bw}$。
- 协方差 $\mathbf P^{b_k}_{b_{k+1}}$；$\mathbf Q=\mathrm{diag}(\sigma_a^2,\sigma_w^2,\sigma_{ba}^2,\sigma_{bw}^2)$。

## §S4.1 IMU 动力学与预积分量定义 [源 §V (eqs 5-6)]

两帧间状态关系（重力 $\mathbf g^w$ 显式）：
$$\mathbf R^{b_k}_w\mathbf p^w_{b_{k+1}}=\mathbf R^{b_k}_w\big(\mathbf p^w_{b_k}+\mathbf v^w_{b_k}\Delta t_k-\tfrac12\mathbf g^w\Delta t_k^2\big)+\boldsymbol\alpha^{b_k}_{b_{k+1}},$$
$$\mathbf R^{b_k}_w\mathbf v^w_{b_{k+1}}=\mathbf R^{b_k}_w(\mathbf v^w_{b_k}-\mathbf g^w\Delta t_k)+\boldsymbol\beta^{b_k}_{b_{k+1}},$$
$$\mathbf q^{b_k}_w\otimes\mathbf q^w_{b_{k+1}}=\boldsymbol\gamma^{b_k}_{b_{k+1}}.\tag{5}$$
**预积分量定义**（以 $b_k$ 为参考系，仅依赖 IMU 读数与 bias）：
$$\boldsymbol\alpha^{b_k}_{b_{k+1}}=\iint_{t\in[t_k,t_{k+1}]}\mathbf R^{b_k}_t(\hat{\mathbf a}_t-b_{a_t}-n_a)\,dt^2,$$
$$\boldsymbol\beta^{b_k}_{b_{k+1}}=\int_{t\in[t_k,t_{k+1}]}\mathbf R^{b_k}_t(\hat{\mathbf a}_t-b_{a_t}-n_a)\,dt,$$
$$\boldsymbol\gamma^{b_k}_{b_{k+1}}=\int_{t\in[t_k,t_{k+1}]}\frac12\,\Omega(\hat{\boldsymbol\omega}_t-b_{w_t}-n_w)\,\boldsymbol\gamma^{b_k}_t\,dt.\tag{6}$$
$\boldsymbol\alpha,\boldsymbol\beta,\boldsymbol\gamma$ 仅与 IMU bias 相关（不依赖 $b_k,b_{k+1}$ 的其他状态）。bias 变化小时用一阶近似调整，否则重传播。

> **对照 Forster**：VINS $\boldsymbol\alpha\leftrightarrow\Delta\tilde{\mathbf p}_{ij}$、$\boldsymbol\beta\leftrightarrow\Delta\tilde{\mathbf v}_{ij}$、$\boldsymbol\gamma\leftrightarrow\Delta\tilde{\mathbf R}_{ij}$（四元数 vs 旋转阵）。差别：Forster 把重力放残差里、定义 $\Delta\mathbf v,\Delta\mathbf p$ 时已剔除重力与 $\mathbf v_i$；VINS (5) 把重力/初速放在状态关系两端，预积分量本身（6）不含重力（因以 $b_k$ 为参考、$\mathbf R^{b_k}_t$ 从单位起算）。**两者本质相同。**

## §S4.2 离散传播（欧拉，名义量）[源 §V (eq 7)]

不同数值法（Euler/mid-point/RK4）皆可；论文正文用 **Euler** 演示（代码实现用 **mid-point**）。初值 $\boldsymbol\alpha^{b_k}_{b_k}=\mathbf 0$、$\boldsymbol\beta^{b_k}_{b_k}=\mathbf 0$、$\boldsymbol\gamma^{b_k}_{b_k}$=单位四元数。加性噪声 $n_a,n_w$ 实现时置零（→估计值带帽）：
$$\hat{\boldsymbol\alpha}^{b_k}_{i+1}=\hat{\boldsymbol\alpha}^{b_k}_i+\hat{\boldsymbol\beta}^{b_k}_i\delta t+\tfrac12\mathbf R(\hat{\boldsymbol\gamma}^{b_k}_i)(\hat{\mathbf a}_i-b_{a_i})\delta t^2,$$
$$\hat{\boldsymbol\beta}^{b_k}_{i+1}=\hat{\boldsymbol\beta}^{b_k}_i+\mathbf R(\hat{\boldsymbol\gamma}^{b_k}_i)(\hat{\mathbf a}_i-b_{a_i})\delta t,$$
$$\hat{\boldsymbol\gamma}^{b_k}_{i+1}=\hat{\boldsymbol\gamma}^{b_k}_i\otimes\begin{bmatrix}1\\\tfrac12(\hat{\boldsymbol\omega}_i-b_{w_i})\delta t\end{bmatrix}.\tag{7}$$
$i$ 为 $[t_k,t_{k+1}]$ 内某 IMU 测量时刻，$\delta t$ = 相邻测量间隔。

## §S4.3 误差态连续动态 + 协方差/雅可比递推 [源 §V (eqs 8-12)]

四元数过参数化，误差用**右乘小扰动**定义：
$$\boldsymbol\gamma^{b_k}_t\approx\hat{\boldsymbol\gamma}^{b_k}_t\otimes\begin{bmatrix}1\\\tfrac12\delta\boldsymbol\theta^{b_k}_t\end{bmatrix},\tag{8}$$
$\delta\boldsymbol\theta^{b_k}_t\in\mathbb R^3$ 三维小扰动。

> **与本书约定一致**：四元数右乘小角扰动 = 本书右扰动（局部）。

**连续时间误差态线性动态**（误差态 $\delta\mathbf z=[\delta\boldsymbol\alpha,\delta\boldsymbol\beta,\delta\boldsymbol\theta,\delta b_a,\delta b_w]^\top\in\mathbb R^{15}$）：
$$\begin{bmatrix}\delta\dot{\boldsymbol\alpha}^{b_k}_t\\\delta\dot{\boldsymbol\beta}^{b_k}_t\\\delta\dot{\boldsymbol\theta}^{b_k}_t\\\delta\dot b_{a_t}\\\delta\dot b_{w_t}\end{bmatrix}=\underbrace{\begin{bmatrix}\mathbf 0&\mathbf I&\mathbf 0&\mathbf 0&\mathbf 0\\\mathbf 0&\mathbf 0&-\mathbf R^{b_k}_t\lfloor\hat{\mathbf a}_t-b_{a_t}\rfloor_\times&-\mathbf R^{b_k}_t&\mathbf 0\\\mathbf 0&\mathbf 0&-\lfloor\hat{\boldsymbol\omega}_t-b_{w_t}\rfloor_\times&\mathbf 0&-\mathbf I\\\mathbf 0&\mathbf 0&\mathbf 0&\mathbf 0&\mathbf 0\\\mathbf 0&\mathbf 0&\mathbf 0&\mathbf 0&\mathbf 0\end{bmatrix}}_{\mathbf F_t}\begin{bmatrix}\delta\boldsymbol\alpha^{b_k}_t\\\delta\boldsymbol\beta^{b_k}_t\\\delta\boldsymbol\theta^{b_k}_t\\\delta b_{a_t}\\\delta b_{w_t}\end{bmatrix}+\underbrace{\begin{bmatrix}\mathbf 0&\mathbf 0&\mathbf 0&\mathbf 0\\-\mathbf R^{b_k}_t&\mathbf 0&\mathbf 0&\mathbf 0\\\mathbf 0&-\mathbf I&\mathbf 0&\mathbf 0\\\mathbf 0&\mathbf 0&\mathbf I&\mathbf 0\\\mathbf 0&\mathbf 0&\mathbf 0&\mathbf I\end{bmatrix}}_{\mathbf G_t}\begin{bmatrix}n_a\\n_w\\n_{ba}\\n_{bw}\end{bmatrix}=\mathbf F_t\delta\mathbf z^{b_k}_t+\mathbf G_t\mathbf n_t.\tag{9}$$
（$\lfloor\cdot\rfloor_\times$ = 反对称阵，即本书 $(\cdot)^\wedge$。）

**离散协方差递推**（一阶离散，初值 $\mathbf P^{b_k}_{b_k}=\mathbf 0$）：
$$\boxed{\ \mathbf P^{b_k}_{t+\delta t}=(\mathbf I+\mathbf F_t\delta t)\mathbf P^{b_k}_t(\mathbf I+\mathbf F_t\delta t)^\top+(\mathbf G_t\delta t)\mathbf Q(\mathbf G_t\delta t)^\top,\quad t\in[k,k+1]\ }\tag{10}$$
$\mathbf Q=\mathrm{diag}(\sigma_a^2,\sigma_w^2,\sigma_{ba}^2,\sigma_{bw}^2)$ 为噪声对角协方差。

**一阶雅可比递推**（$\delta\mathbf z^{b_k}_{b_{k+1}}$ 对 $\delta\mathbf z^{b_k}_{b_k}$ 的雅可比，初值 $\mathbf J_{b_k}=\mathbf I$）：
$$\boxed{\ \mathbf J_{t+\delta t}=(\mathbf I+\mathbf F_t\delta t)\mathbf J_t,\quad t\in[k,k+1]\ }\tag{11}$$

**bias 一阶修正**（用 $\mathbf J_{b_{k+1}}$ 的子块）：
$$\boldsymbol\alpha^{b_k}_{b_{k+1}}\approx\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}}+\mathbf J^\alpha_{b_a}\delta b_{a_k}+\mathbf J^\alpha_{b_w}\delta b_{w_k},$$
$$\boldsymbol\beta^{b_k}_{b_{k+1}}\approx\hat{\boldsymbol\beta}^{b_k}_{b_{k+1}}+\mathbf J^\beta_{b_a}\delta b_{a_k}+\mathbf J^\beta_{b_w}\delta b_{w_k},$$
$$\boldsymbol\gamma^{b_k}_{b_{k+1}}\approx\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}\otimes\begin{bmatrix}1\\\tfrac12\mathbf J^\gamma_{b_w}\delta b_{w_k}\end{bmatrix},\tag{12}$$
$\mathbf J^\alpha_{b_a}$ = $\mathbf J_{b_{k+1}}$ 中 $\frac{\delta\boldsymbol\alpha^{b_k}_{b_{k+1}}}{\delta b_{a_k}}$ 对应子块（其余 $\mathbf J^\alpha_{b_w},\mathbf J^\beta_{b_a},\mathbf J^\beta_{b_w},\mathbf J^\gamma_{b_w}$ 同义）。bias 小变化用 (12) 近似修正，免重传播。

> **与 Forster 一阶修正等价**：VINS 用大 $15\times15$ 状态转移雅可比 $\mathbf J$ 的子块；Forster 用解析闭式 $\partial\Delta\bar{\mathbf R}/\partial\mathbf b^g$ 等（§S1.8）。两者数值等价。

## §S4.4 IMU 测量模型（与 VIO 接口）[源 §V]

预积分测量模型（带协方差 $\mathbf P^{b_k}_{b_{k+1}}$）写成 15 维测量（位置/速度/姿态预积分 + bias 不变项）：
$$\begin{bmatrix}\hat{\boldsymbol\alpha}^{b_k}_{b_{k+1}}\\\hat{\boldsymbol\beta}^{b_k}_{b_{k+1}}\\\hat{\boldsymbol\gamma}^{b_k}_{b_{k+1}}\\\mathbf 0\\\mathbf 0\end{bmatrix}=(\dots)$$
（残差项 = (5) 两端之差 + bias 差，进紧耦合滑窗 BA 优化；原文 §V-C/§VI 给 IMU 残差 $\mathbf r_{\mathcal B}$ 完整 15 维表达，含 $\mathbf q^{-1}\otimes$ 误差等，本抽取记其结构：IMU 残差 = 状态预测与预积分测量之差的马氏范数。）

> **本章接口要点**：VINS-Mono 把预积分 IMU 因子 + 视觉重投影因子 + （回环）合成紧耦合非线性优化（滑窗 BA），是"预积分→VIO"的代表实现；与 §S1.6 Forster 因子图 + iSAM2 + 无结构视觉因子为两条主流路线。

---

# §S5 完整 IMU 内参模型：scale + 轴失准 + g-sensitivity（arXiv:2201.09170）

> **本章重点**："scale/误差模型"的完整形式——前面诸源把 scale/失准并入"标定误差"一笔带过，此源给**完整矩阵模型**与变体分类（含 Kalibr/Rehder 模型）。

## §S5.0 记号约定（OpenVINS 本源）

- IMU 含两参考系：陀螺系 $\{w\}$、加速度计系 $\{a\}$；基"惯性"系 $\{I\}$ 须取与 $\{w\}$ 或 $\{a\}$ 之一重合。
- ${}^w_I\mathbf R,{}^a_I\mathbf R$：陀螺/加速度系到基惯性系 $\{I\}$ 的旋转。
- $\mathbf T_w,\mathbf T_a$：$3\times3$ 可逆阵，表 scale 缺陷 + 轴失准；$\mathbf D_w=\mathbf T_w^{-1},\mathbf D_a=\mathbf T_a^{-1}$。
- $\mathbf T_g$：g-sensitivity 矩阵（重力/加速度对陀螺读数的影响）。
- $b_g,b_a$ bias（随机游走）；$n_g,n_a$ 零均值高斯。

## §S5.1 一般 IMU 内参测量模型 [源 §4.1 (eqs 12-15)]

原始读数（陀螺 ${}^w\boldsymbol\omega_m$、加速度 ${}^a\mathbf a_m$）：
$$\boxed{\ {}^w\boldsymbol\omega_m=\mathbf T_w\,{}^w_I\mathbf R\,{}^I\boldsymbol\omega+\mathbf T_g\,{}^I\mathbf a+b_g+n_g\ }\tag{12}$$
$$\boxed{\ {}^a\mathbf a_m=\mathbf T_a\,{}^a_I\mathbf R\,{}^I\mathbf a+b_a+n_a\ }\tag{13}$$
- 若取 $\{I\}=\{w\}$，则 ${}^w_I\mathbf R=\mathbf I_3$；否则 ${}^a_I\mathbf R=\mathbf I_3$。
- $\mathbf T_g$ = **g-sensitivity**：重力对陀螺读数影响。
- 忽略陀螺-加速度计平移（多数 IMU 可忽略）。

**真值（校正后）**：
$$\boxed{\ {}^I\boldsymbol\omega={}^I_w\mathbf R\,\mathbf D_w\big({}^w\boldsymbol\omega_m-\mathbf T_g\,{}^I\mathbf a-b_g-n_g\big)\ }\tag{14}$$
$$\boxed{\ {}^I\mathbf a={}^I_a\mathbf R\,\mathbf D_a({}^a\mathbf a_m-b_a-n_a)\ }\tag{15}$$
$\mathbf D_w=\mathbf T_w^{-1},\mathbf D_a=\mathbf T_a^{-1}$。实践标定 $\mathbf D_a,\mathbf D_w,{}^I_a\mathbf R$（或 ${}^I_w\mathbf R$）、$\mathbf T_g$（避免在线求逆）。**只标 ${}^I_w\mathbf R$ 与 ${}^I_a\mathbf R$ 之一**（基惯性系与其一重合）；同时标二者会使 IMU-相机旋转因过参数化不可观。

## §S5.2 内参模型变体（含 Kalibr 模型）[源 §4.1.1, Table 1]

**上三角 scale-失准阵**（6 参数，imu1 等）：
$$\mathbf D_{*6}=\begin{bmatrix}d_{*1}&d_{*2}&d_{*4}\\0&d_{*3}&d_{*5}\\0&0&d_{*6}\end{bmatrix}.\tag{16}$$
**全阵**（9 参数，imu3/4 等，把旋转并入）：
$$\mathbf D_{*9}=\begin{bmatrix}d_{*1}&d_{*4}&d_{*7}\\d_{*2}&d_{*5}&d_{*8}\\d_{*3}&d_{*6}&d_{*9}\end{bmatrix}.\tag{17}$$
**6 参 g-sensitivity**（上三角）：
$$\mathbf T_{g6}=\begin{bmatrix}t_{g1}&t_{g2}&t_{g4}\\0&t_{g3}&t_{g5}\\0&0&t_{g6}\end{bmatrix}.\tag{18}$$
**9 参 g-sensitivity**（全阵）：
$$\mathbf T_{g9}=\begin{bmatrix}t_{g1}&t_{g4}&t_{g7}\\t_{g2}&t_{g5}&t_{g8}\\t_{g3}&t_{g6}&t_{g9}\end{bmatrix}.\tag{19}$$
**下三角 scale-失准阵**（6 参数，Kalibr/Rehder 模型用，imu6）：
$$\mathbf D'_{*6}=\begin{bmatrix}d_{*1}&0&0\\d_{*2}&d_{*4}&0\\d_{*3}&d_{*5}&d_{*6}\end{bmatrix}.\tag{20}$$

**模型变体表（Table 1，全量）**：

| 模型 | 维数 | $\mathbf D_w$ | $\mathbf D_a$ | ${}^I_w\mathbf R$ | ${}^I_a\mathbf R$ | $\mathbf T_g$ |
|---|---|---|---|---|---|---|
| imu0 | 0 | - | - | - | - | - |
| imu1 | 15 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | ${}^I_w\mathbf R$ | - | - |
| imu2 | 15 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | - | ${}^I_a\mathbf R$ | - |
| imu3 | 15 | $\mathbf D_{w9}$ | $\mathbf D_{a6}$ | - | - | - |
| imu4 | 15 | $\mathbf D_{w6}$ | $\mathbf D_{a9}$ | - | - | - |
| imu5 | 18 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | ${}^I_w\mathbf R$ | ${}^I_a\mathbf R$ | - |
| imu6 | 24 | $\mathbf D'_{w6}$ | $\mathbf D'_{a6}$ | ${}^I_w\mathbf R$ | - | $\mathbf T_{g9}$ |
| imu11 | 21 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | ${}^I_w\mathbf R$ | - | $\mathbf T_{g6}$ |
| imu12 | 21 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | - | ${}^I_a\mathbf R$ | $\mathbf T_{g6}$ |
| imu13 | 21 | $\mathbf D_{w9}$ | $\mathbf D_{a6}$ | - | - | $\mathbf T_{g6}$ |
| imu14 | 21 | $\mathbf D_{w6}$ | $\mathbf D_{a9}$ | - | - | $\mathbf T_{g6}$ |
| imu21 | 24 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | ${}^I_w\mathbf R$ | - | $\mathbf T_{g9}$ |
| imu22 | 24 | $\mathbf D_{w6}$ | $\mathbf D_{a6}$ | - | ${}^I_a\mathbf R$ | $\mathbf T_{g9}$ |
| imu23 | 24 | $\mathbf D_{w9}$ | $\mathbf D_{a6}$ | - | - | $\mathbf T_{g9}$ |
| imu24 | 24 | $\mathbf D_{w6}$ | $\mathbf D_{a9}$ | - | - | $\mathbf T_{g9}$ |
| imu31 | 9 | - | $\mathbf D_{a9}$ | - | - | - |
| imu32 | 9 | $\mathbf D_{w9}$ | - | - | - | - |
| imu33 | 6 | - | - | - | - | $\mathbf T_{g6}$ |
| imu34 | 9 | - | - | - | - | $\mathbf T_{g9}$ |

变体说明（源 §4.1.1 逐条）：
- **imu1**：含 ${}^I_w\mathbf R$ + 上三角 $\mathbf D_{w6},\mathbf D_{a6}$。
- **imu2**：改含 ${}^I_a\mathbf R$（Schneider et al. 2019 模型）。
- **imu3**：把 imu1 的 $\mathbf D_{w6}+{}^I_w\mathbf R$ 合为全阵 $\mathbf D_{w9}$（9 参），$\mathbf D_{a6}$ 仍上三角。
- **imu4**：imu2 的 $\mathbf D_{a6}+{}^I_a\mathbf R$ 合为 $\mathbf D_{a9}$，$\mathbf D_{w6}$ 上三角。
- **imuA（A=1..4）+ $\mathbf T_{g6}$ → imu1A**；**+ $\mathbf T_{g9}$ → imu2A**。
- **imu5**：含 $\mathbf D_{w6},\mathbf D_{a6},{}^I_w\mathbf R,{}^I_a\mathbf R$（冗余过参数化，用于验证 ${}^I_w\mathbf R,{}^I_a\mathbf R$ 不可同时标）。
- **imu6**：$\mathbf D'_{w6},\mathbf D'_{a6}$（下三角）+ ${}^I_w\mathbf R$ + $\mathbf T_{g9}$ = **Kalibr (Furgale 2013) / Rehder et al. 2016 的 scale-失准 IMU 内参模型**。

> **数值例/结论（源 §9）**：BMI160 的 g-sensitivity $\mathbf T_g$ 项一般比其他内参小 1–2 个数量级；含/不含 g-sensitivity 的估计误差相近 → 系统性能对 g-sensitivity 不敏感。IMU scale 0.003、skew 0.003 量级（评估设定）。
>
> **与本书统一约定关系**：此内参模型独立于扰动方向（scale/失准是确定性内参，bias 仍随机游走）。本书把 (12)(13) 作为**比 Forster (27)(28) 更完整的 IMU 测量模型**（Forster 隐含 $\mathbf T_a=\mathbf T_w=\mathbf I$、$\mathbf T_g=\mathbf 0$、$\{I\}=\{w\}=\{a\}$ 的理想情形）。

---

# §S6 Allan 方差五项噪声辨识（IEEE 952-1997 / Calgary thesis）

> **本章重点**：Allan 方差完整理论——定义、PSD↔AVAR 关系、五种噪声幂律斜率与读数法（含 0.664 常数）。源遵循 IEEE Std 952-1997。

## §S6.1 Allan 方差定义 [源 §4.1-4.2]

两样本（Allan）方差以 cluster/averaging time $T$ 估计。用输出角/速度 $\theta(t)=\int^t\Omega(t)dt$（式 4.6，下限不计，只用差），离散 $t=kt_0$、$\theta_k=\theta(kt_0)$。平均率
$$\bar\Omega_k(T)=\frac{\theta_{k+n}-\theta_k}{T},\tag{4.7}\qquad\bar\Omega_{k\,\text{next}}(T)=\frac{\theta_{k+2n}-\theta_{k+n}}{T}.\tag{4.8}$$
**Allan 方差估计**：
$$\boxed{\ \sigma^2(T)=\frac{1}{2(N-2n)}\sum_{k=1}^{N-2n}\big(\bar\Omega_{k+n}-\bar\Omega_k\big)^2\ }\tag{4.9}$$
（等价 Woodman (19) 的 bin 形式。）有限数据 → 估计量，质量取决于独立 cluster 数。

**AVAR 与 PSD 的关系**（IEEE 952，master 公式）：
$$\boxed{\ \sigma^2(T)=4\int_0^\infty S_\Omega(f)\frac{\sin^4(\pi fT)}{(\pi fT)^2}\,df\ }\tag{4.10}$$
$S_\Omega(f)$ = 率过程 $\Omega$ 的功率谱密度。
> **推导骨架**（源给出）：$\sigma^2(T)=\langle\frac12(\bar\Omega_{k\,\text{next}}-\bar\Omega_k)^2\rangle$，展开得三项（式 4.11）；$\bar\Omega_k=\frac1T\int_{t_k}^{t_k+T}\Omega(t)dt$（式 4.12）；平稳假设 $R(\tau)=\langle\Omega(t)\Omega(t-\tau)\rangle$（4.13），$S_\Omega(f)=\int_{-\infty}^\infty R(\tau)e^{-i2\pi f\tau}d\tau$（傅里叶），代入即得 (4.10)。**这是把任意噪声 PSD 映到 Allan 方差的通用桥梁**，下面五项皆由此积分得。

## §S6.2 五种噪声项（幂律斜率 + 读数法）[源 §4.3]

### §S6.2.1 量化噪声 (Quantization) [§4.3.1]
A/D 编码引起。角 PSD $S_\theta(f)\approx T_z Q_z^2$（小 $f$，式 4.26），$Q_z$ = 量化系数（理论极限 $S/\sqrt{12}$，$S$ 标度系数），$T_z$ 采样间隔。率 PSD $S_\Omega(f)=(2\pi f)^2S_\theta(f)\approx(2\pi f)^2 T_z Q_z^2$（4.27-4.28）。代入 (4.10) 积分：
$$\sigma^2(T)=\frac{3Q_z^2}{T^2}\ \Rightarrow\ \boxed{\ \sigma(T)=\frac{\sqrt3\,Q_z}{T}\ }\tag{4.29,4.30}$$
**log-log 斜率 $-1$**；读数：斜率线在 $T=\sqrt3$ 处。短相关/宽带，常可滤除，非主误差。

### §S6.2.2 角度（速度）随机游走 ARW/VRW [§4.3.2]
高频噪声（相关时间 ≪ 采样）。率 PSD：
$$S_\Omega(f)=Q^2,\tag{4.31}$$
$Q$ = 角（速度）随机游走系数。代入 (4.10)：
$$\sigma^2(T)=\frac{4Q^2}{T^2}\int_0^\infty\frac{\sin^4(\pi fT)}{(\pi f)^2}\,df,$$
换元 $u=\pi fT$：
$$\sigma^2(T)=\frac{4Q^2}{\pi^2 T}\int_0^\infty\frac{\sin^4 u}{u^2}\,du,\qquad\int_0^\infty\frac{\sin^4 u}{u^2}du=\frac{\pi}{4}\ \text{(Gradshteyn-Ryzhik)},$$
$$\sigma^2(T)=\frac{Q^2}{T}\ \Rightarrow\ \boxed{\ \sigma(T)=\frac{Q}{\sqrt T}\ }\tag{4.36,4.37}$$
**log-log 斜率 $-1/2$**；读数：斜率线在 $T=1$ 处直接读 $Q$。

### §S6.2.3 Bias 不稳定 (Bias Instability) [§4.3.3]
源于电子学闪烁（$1/f$）。率 PSD：
$$S_\Omega(f)=\begin{cases}\dfrac{B^2}{2\pi f}&f\le f_0\\[4pt]0&f>f_0\end{cases},\tag{4.38}$$
$B$ = bias 不稳定系数、$f_0$ 截止频率。代入 (4.10)（用 4.33 同型积分，4.40-4.42 含余弦积分 $\mathrm{Ci}$）：得 $\sigma^2(T)$ 闭式（式 4.48-4.50）。**极限**（$T\gg1/f_0$）：
$$\sigma(T)\to B\sqrt{\frac{2\ln2}{\pi}}\quad(T\gg1/f_0).\tag{4.51}$$
$\sqrt{2\ln2/\pi}\approx0.664$。**log-log**：$f_0\ll1/T$ 时斜率 $+1$，$T$ 远大于截止频率倒数时**渐近平台 $0.664B$**。读数：**平坦区高度 / 0.664 = $B$**（曲线极小值）。其上升段易被其他噪声掩盖。

### §S6.2.4 率随机游走 (Rate Random Walk) [§4.3.4]
不明起源（可能极长相关时间指数相关噪声极限）。率 PSD：
$$S_\Omega(f)=\Big(\frac{K}{2\pi}\Big)^2\frac{1}{f^2},\tag{4.52}$$
$K$ = 率随机游走系数。代入 (4.10)：
$$\sigma^2(T)=\frac{K^2 T}{3}\ \Rightarrow\ \boxed{\ \sigma(T)=K\sqrt{\frac{T}{3}}\ }\tag{4.53,4.54}$$
**log-log 斜率 $+1/2$**；读数：斜率线在 $T=3$ 处读 $K$。

### §S6.2.5 漂移率斜坡 (Drift Rate Ramp) [§4.3.5]
确定性误差 $\Omega=Rt$（式 4.55），$R$ = 斜坡系数。对含此输入的 cluster 操作：
$$\sigma^2(T)=\frac{R^2 T^2}{2}\ \Rightarrow\ \boxed{\ \sigma(T)=\frac{R\,T}{\sqrt2}\ }\tag{4.56,4.57}$$
率 PSD $S_\Omega(f)=R^2/(2\pi f)^3$（4.58）。**log-log 斜率 $+1$**；读数：斜率线在 $T=\sqrt2$ 处读 $R$。（注：$1/f^3$ 的 flicker 加速度噪声有相同 $T$ 依赖。）

### §S6.2.6 指数相关（马尔可夫）噪声 [§4.3.6]
有限相关时间 $T_c$、幅度 $q_c$。率 PSD：
$$S_\Omega(f)=\frac{(q_cT_c)^2}{1+(2\pi fT_c)^2},\tag{4.59}$$
代入 (4.10)：
$$\sigma^2(T)=\frac{(q_cT_c)^2}{T}\Big[1-\frac{T_c}{2T}\big(3-4e^{-T/T_c}+e^{-2T/T_c}\big)\Big].\tag{4.60}$$
**极限**：$T\gg T_c$ → $\sigma^2(T)\to(q_cT_c)^2/T$（ARW，$Q=q_cT_c$，式 4.61）；$T\ll T_c$ → $\sigma^2(T)\to q_c^2T/3$（率随机游走，式 4.62）。

### §S6.2.7 正弦噪声 [§4.3.7]
单频 PSD $S_\Omega(f)=\frac12\Omega_0^2[\delta(f-f_0)+\delta(f+f_0)]$（4.63）。
$$\sigma^2(T)=\Omega_0^2\Big(\frac{\sin^2(\pi f_0T)}{\pi f_0T}\Big)^2.\tag{4.64}$$
log-log 上呈正弦，峰按斜率 $-1$ 衰减；辨识需观察多峰，常被掩盖（此情形传统 PSD 更优）。

## §S6.3 组合 Allan 方差与估计质量 [源 §4.4-4.5]

**组合**：实际数据多噪声并存，总 Allan 方差 ≈ 各项之和（不同噪声出现在不同 $T$ 区域，便于辨识）。**典型组合形式**（IEEE 952 五项，本抽取据上式综合）：
$$\sigma^2(T)\approx\frac{3Q_z^2}{T^2}+\frac{Q^2}{T}+\frac{2\ln2}{\pi}B^2+\frac{K^2T}{3}+\frac{R^2T^2}{2}.$$
（依次：量化 / ARW / bias 不稳定 / 率随机游走 / 率斜坡。本书可直接用此式做最小二乘拟合提系数；标 `\rebuilt` 待核对系数。）

**估计质量** [§4.5]：百分误差
$$\delta_{AV}(\sigma)=\frac{1}{\sqrt{2(\frac{N}{n}-1)}},\tag{4.66}$$
$N$ 总点数、$n$ cluster 内点数。短 $T$（cluster 多）误差小、长 $T$ 误差大。**数值例**：20000 点、cluster 5000 → 误差 ≈40%；cluster 100 → ≈5%。

**数值例（设备级别，源 §5）**：
- Honeywell CIMU（导航级）：陀螺 in-run bias ≈$0.0022^\circ/h$、随机游走 ≈$0.0022^\circ/\sqrt h$；加速度计 in-run bias ≈$25\mu g$、噪声 ≈$0.00076\ m/s/\sqrt h$。
- 三档设备：CIMU（导航级）/ HG1700（战术级）/ MotionPak II（消费级 MEMS）。

> **与 Kalman/Kalibr 参数的桥（综合 agent 关键）**：Allan 的 $N$（=ARW 系数 $Q$）↔ Kalibr 连续白噪声 $\sigma_g$（同为连续白噪声谱密度平方根，仅单位 $\mathrm{rad}/\sqrt s$ vs $^\circ/\sqrt h$ 换算）；Allan 的 $K$（率随机游走系数）↔ Kalibr bias 随机游走 $\sigma_{b_g}$。bias 不稳定 $B$ 不直接等于随机游走（$1/f$ vs $1/f^2$），KF 常用一阶 Gauss-Markov 或随机游走近似 bias 漂移——本书须讲清"$B$（平坦区）≠ 随机游走系数 $K$"这一**常见混淆**。

---

# §S7 重力模型与坐标系（WGS84 / 惯性导航坐标系）

> **本章重点**："重力与坐标系"。综合 NIMA TR8350.2（WGS84）与惯性导航坐标系综述。

## §S7.1 坐标系定义 [源：惯性导航综述 / VectorNav / Trawny]

| 系 | 名称 | 原点 | 轴定义 | 用途 |
|---|---|---|---|---|
| **ECI** ($i$) | 地心惯性系 | 地球质心 | 相对恒星不旋转；$z$ 平行地球自转轴，$x$ 指平春分点，$y$ 右手补全 | 理想惯性系：理想加速度计/陀螺固连 $i$ 系输出为零。IMU 真正测量相对 $i$ 系的量。 |
| **ECEF** ($e$) | 地心地固系 | 地球质心 | 随地球旋转；$x$ 指格林尼治平子午线，$z$ 平行地球平自转轴，$y$ 右手补全 | GNSS、全球定位。相对 $i$ 系以 $\omega_E$ 旋转。 |
| **NED** ($n$) | 北-东-地 | 设备/传感器处（局部切平面） | $x$=真北，$z$=朝地心（向下），$y$=东（右手）；切于 WGS84 椭球 | 局部导航（航空常用）。 |
| **ENU** | 东-北-天 | 同 NED | $x$=东，$y$=北，$z$=朝外（向上） | 局部导航（机器人/SLAM 常用）。 |
| **body** ($b$) | 载体/IMU 系 | 传感器 | 固连载体三轴 | IMU 原始测量所在系。 |

- 地球自转率 $\omega_E\approx7.292115\times10^{-5}\ \mathrm{rad/s}$（≈$7.3\times10^{-5}$）。
> **导航系（n-frame）= 局部大地系**，原点与传感器重合，$x$ 指大地北、$z$ 垂直参考椭球向下 → 即 NED。

> **Forster/VINS 的简化**：取 world 系 $W$（或 $w$）为**惯性系**（忽略 $\omega_E$），即 §S1.4 (27)(28) 与 §S4 (5)(6) 的前提。高精度/长航时须补地球自转/输运率（见 §S1 Solà 脚注：高端 IMU $\boldsymbol\omega_m=\boldsymbol\omega_t+\mathbf R_t^\top\boldsymbol\omega_E+\dots$）。

## §S7.2 重力模型：WGS84 正常重力 [源：NIMA TR8350.2]

**Somigliana 闭式**（椭球面 $h=0$ 正常重力，纬度 $\phi$ = 大地纬度）：
$$\boxed{\ \gamma(\phi)=\gamma_e\,\frac{1+k\sin^2\phi}{\sqrt{1-e^2\sin^2\phi}}\ }$$
WGS84 常数：
- 赤道正常重力 $\gamma_e=9.7803253359\ \mathrm{m/s^2}$（部分版本 $9.7803267715$）；
- 极地正常重力 $\gamma_p=9.8321849378\ \mathrm{m/s^2}$（部分版本 $9.8321863685$）；
- Somigliana 常数 $k=\dfrac{b\gamma_p-a\gamma_e}{a\gamma_e}\approx0.00193185265241$；
- 第一偏心率平方 $e^2\approx0.00669437999014$；
- 长半轴 $a=6378137.0\ \mathrm m$、短半轴 $b\approx6356752.3142\ \mathrm m$、扁率 $f=1/298.257223563$。

**国际重力公式（简化级数）**：
$$\gamma(\phi)=9.780327\,(1+0.0053024\sin^2\phi-0.0000058\sin^2 2\phi)\ \mathrm{m/s^2}.$$

**高度（自由空气）修正**（一阶 + 二阶）：
$$\gamma(\phi,h)=\gamma(\phi)-(3.0877\times10^{-6}-4.4\times10^{-9}\sin^2\phi)\,h+0.72\times10^{-12}\,h^2\ \mathrm{m/s^2},$$
$h$ 单位米。一阶梯度约 $-3.086\times10^{-6}\ \mathrm{m/s^2/m}=-0.3086\ \mathrm{mGal/m}$（自由空气改正）。

> **IMU/VIO 用法（综合要点）**：
> 1. SLAM/VIO 常把重力当**待估状态**（方向 + 局部常值 $\|\mathbf g\|\approx9.81\ \mathrm{m/s^2}$），或固定为已知（如 ENU 下 $\mathbf g=[0,0,-9.81]$）。
> 2. 加速度计测**比力** $\mathbf f=\mathbf a-\mathbf g$（Forster (28) 的 $\mathbf R^\top(\mathbf a-\mathbf g)$）：静止时读 $-\mathbf R^\top\mathbf g$（指天，量值 $g$），这是**初始姿态对齐（重力对齐）**与加速度计 bias-重力耦合（§S3.3.1）的根源。
> 3. **纬度/高度依赖**：$\gamma$ 从赤道 $9.7803$ 到极地 $9.8322$（差约 $0.0519$，~0.5%）；高度每升 1km 降约 $3.086\times10^{-3}\ \mathrm{m/s^2}$。短时局部 VIO 可取常值；惯导/大范围须用模型。
> 4. **离心/科氏/输运项**：n 系导航严格式含离心加速度（并入正常重力 → 实际"重力" $\mathbf g$ 含地球离心）、科氏项 $2\boldsymbol\omega_{ie}\times\mathbf v$、输运率 $\boldsymbol\omega_{en}$；本书 IMU 章默认局部惯性近似（忽略），在惯导专题再补。

---

# §M 跨源合并与记号统一（供综合 agent 转换用）

## §M.1 统一记号对照表

| 概念 | Forster【F】 | VINS【V】 | Woodman【W】 | OpenVINS【O】 | Allan【A】 | **本书统一** |
|---|---|---|---|---|---|---|
| 旋转 | $\mathbf R\in\mathrm{SO}(3)$ | $\mathbf R$/$\mathbf q$ | $\mathbf C$（方向余弦） | $\mathbf R$ | — | $\mathbf R\in\mathrm{SO}(3)$ |
| 四元数 | —（用 $\mathbf R$） | Hamilton，实部前 | — | — | — | **Hamilton** |
| 扰动方向 | **右扰动 $\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$** | 右乘 $\delta\boldsymbol\theta$（局部） | 小角 $\delta\boldsymbol\Psi$ | — | — | **右扰动 $\delta\boldsymbol\phi$** |
| 右雅可比 | $\mathbf J_r$ | （隐于 $\frac12\delta\boldsymbol\theta$） | — | — | — | $\mathbf J_r$ |
| 角速度测量 | $\tilde{\boldsymbol\omega}$ | $\hat{\boldsymbol\omega}$ | $\boldsymbol\omega_b$ | ${}^w\boldsymbol\omega_m$ | $\Omega$ | $\tilde{\boldsymbol\omega}$ |
| 加速度测量(比力) | $\tilde{\mathbf a}$ | $\hat{\mathbf a}$ | $\mathbf a_b$ | ${}^a\mathbf a_m$ | — | $\tilde{\mathbf a}$ |
| 陀螺 bias | $\mathbf b^g$ | $b_w$ | bias | $b_g$ | — | $\mathbf b_g$ |
| 加速度计 bias | $\mathbf b^a$ | $b_a$ | bias | $b_a$ | — | $\mathbf b_a$ |
| 陀螺白噪声 | $\boldsymbol\eta^g$ | $n_w$ | $\sigma$（白噪声） | $n_g$ | $Q$/$N$ | $\boldsymbol\eta_g$（连续 $\sigma_g$） |
| 加速度白噪声 | $\boldsymbol\eta^a$ | $n_a$ | $\sigma$ | $n_a$ | $Q$/$N$ | $\boldsymbol\eta_a$（连续 $\sigma_a$） |
| 陀螺 bias 游走 | $\boldsymbol\eta^{bg}$ | $n_{bw}$ | BRW | （随机游走） | $K$ | $\boldsymbol\eta_{bg}$（连续 $\sigma_{bg}$） |
| 加速度 bias 游走 | $\boldsymbol\eta^{ba}$ | $n_{ba}$ | — | （随机游走） | $K$ | $\boldsymbol\eta_{ba}$（连续 $\sigma_{ba}$） |
| 预积分旋转 | $\Delta\tilde{\mathbf R}_{ij}$ | $\boldsymbol\gamma^{b_k}_{b_{k+1}}$ | — | — | — | $\Delta\mathbf R_{ij}$ |
| 预积分速度 | $\Delta\tilde{\mathbf v}_{ij}$ | $\boldsymbol\beta^{b_k}_{b_{k+1}}$ | — | — | — | $\Delta\mathbf v_{ij}$ |
| 预积分位置 | $\Delta\tilde{\mathbf p}_{ij}$ | $\boldsymbol\alpha^{b_k}_{b_{k+1}}$ | — | — | — | $\Delta\mathbf p_{ij}$ |
| 预积分协方差 | $\boldsymbol\Sigma_{ij}$ | $\mathbf P^{b_k}_{b_{k+1}}$ | — | — | — | $\boldsymbol\Sigma_{ij}$（或 $\mathbf P$） |
| 原始噪声协方差 | $\boldsymbol\Sigma_\eta$（$6\times6$） | $\mathbf Q$（$12\times12$ 含游走） | — | — | — | 据上下文 |
| 重力 | $\mathbf g$（world） | $\mathbf g^w$ | $\mathbf g_g$ | — | — | $\mathbf g$ |

## §M.2 两套预积分实现的等价性（综合要点）

- **【F】旋转矩阵 + 迭代式协方差 (63)** vs **【V】四元数 + 状态转移 (10)(11)**：数学等价。Forster 显式给 $\mathbf A,\mathbf B$ 与解析 bias 雅可比；VINS 用统一 $\mathbf F_t,\mathbf G_t$ 离散递推、bias 雅可比取大 $\mathbf J$ 子块。本书宜以 Forster 为**理论主线**（流形严谨、右雅可比），以 VINS 为**工程实现对照**（GTSAM vs Ceres）。
- **重力处理差异**：Forster 把 $\mathbf g$ 留在残差 (45)、预积分量剔除重力与 $\mathbf v_i$；VINS (5) 把重力放状态关系两端。本书统一用 Forster 式（预积分量与重力/初态解耦更清晰）。
- **bias 一阶修正**：两者皆"一阶展开避免重积分"，本书讲 Forster 解析式 (§S1.8) 为主，附 VINS 子块法。

## §M.3 连续↔离散噪声统一（本书须显式给出的转换）

$$\sigma_{g,d}^2=\frac{\sigma_g^2}{\Delta t}\ \text{(白噪声方差除}\Delta t),\qquad \sigma_{bg,d}^2=\sigma_{bg}^2\,\Delta t\ \text{(游走方差乘}\Delta t).$$
来源一致性：Forster $\mathrm{Cov}(\boldsymbol\eta^{gd})=\frac1{\Delta t}\mathrm{Cov}(\boldsymbol\eta^g)$、$\boldsymbol\Sigma^{bgd}=\Delta t\,\mathrm{Cov}(\boldsymbol\eta^{bg})$ ↔ Kalibr $\sigma_{g_d}=\sigma_g/\sqrt{\Delta t}$、$\sigma_{bg_d}=\sigma_{bg}\sqrt{\Delta t}$（§S2.4）。

---

# §X 本源未覆盖 / 需他源补齐（缺口清单）

> 诚实标注本抽取（基于上述 7 源）**未涵盖**、需综合 agent 从他源补的内容：

1. **SE(3)/SE_2(3) 群上的预积分**：本抽取以 SO(3)（旋转）为流形主线（Forster）；**矩阵李群 SE_2(3)（含位置/速度/姿态的"扩展位姿"）上的预积分**（如 Brossard/Barrau invariant、arXiv:2102.12897）未抽。本章【SE(3) 流形预积分】要点需补 SE_2(3) / 不变 EKF 视角。已抽的 Solà 提供 SO(3) error-state，可与本文件 §S1 合并。
2. **预积分残差完整 15 维表达（VINS）**：§S4.4 只记结构，VINS 原文 §VI 的 IMU 残差 $\mathbf r_{\mathcal B}$ 完整式（含 $\mathbf q^{-1}\otimes$、重力项）与视觉残差未逐式抽（属 VIO 章，本章只需接口）。
3. **mid-point / RK4 离散预积分显式公式**：VINS 正文用 Euler，代码用 mid-point；mid-point 的 $\boldsymbol\alpha/\boldsymbol\beta/\boldsymbol\gamma$ 显式式（Yibin Wu 推导 arXiv:1912.11986）未抽，可作进阶补充。
4. **Allan 方差与 KF Q 参数的精确换算表**：§S6.3 给了桥接思路，但"$B$（bias 不稳定）→ 一阶 Gauss-Markov 时间常数/驱动方差"的精确公式（El-Sheimy 2008 IEEE T-IM）未抽，本书若要从数据手册 Allan 反推 KF 参数需补。
5. **地球自转/科氏/输运率的严格惯导机理方程**：§S7 给了局部惯性近似与重力模型；n 系/e 系完整机理（$\dot{\mathbf v}^n=\mathbf C^n_b\mathbf f^b-(2\boldsymbol\omega_{ie}^n+\boldsymbol\omega_{en}^n)\times\mathbf v^n+\mathbf g^n$ 等）未逐式抽，属惯导专题。
6. **温度标定/确定性误差在线建模**：Woodman 定性，未给温度补偿模型式。
7. **数值积分阶数对预积分精度影响的定量分析**：Forster 提到 [54-57] 高阶法但未抽其式。

> 本抽取对**本章核心**（IMU 测量/误差模型、连续/离散运动学、SO(3) 预积分理论、bias 一阶修正、协方差传播、连续↔离散噪声、重力与坐标系、Allan 辨识、完整 IMU 内参模型、与 VIO 接口）做到**逐式逐证全量保真**；上列 1-7 为延展/进阶部分，留待综合或后续抽取补。

---

## 附：本抽取统计

- **定理/命题/引理 + 证明**：以 Forster §III SO(3) 性质（Rodrigues、右雅可比、伴随恒等式、SO(3) 高斯似然）+ Allan 五项 PSD→AVAR 推导 + Woodman 姿态/位置积分推导为主，约 18 条带完整推导的结论。
- **完整推导链**：Forster 预积分测量模型 (35)-(38)、噪声传播 (40)-(43)+迭代 (59)-(63)、bias 修正 (64)-(68)+雅可比、残差雅可比 (70)-(81)；Woodman 白噪声积分 (2)-(9)、加速度计二次积分 (11)-(18)、姿态更新 Taylor (38)-(41)；Allan (4.26)-(4.64) 五项；VINS (5)-(12)。≈ 30 条逐步推导。
- **例题/数值例**：GG5300 ARW 例、Xsens Mtx Allan 表、Xsens 60s>150m 漂移、CIMU/HG1700/MotionPak 三档、BMI160 g-sensitivity、WGS84 赤道/极地重力、估计误差 40%/5% 例 ≈ 8 例。
- **表/分类/伪码**：陀螺误差汇总表、加速度计误差汇总、Allan 陀螺数值表、OpenVINS IMU 变体表（18 行）、坐标系表、统一记号对照表、连续/离散单位表 ≈ 7 表。
