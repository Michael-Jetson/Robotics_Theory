# 抽取留痕：Barfoot《State Estimation for Robotics》第 7 章「矩阵李群」（Matrix Lie Groups）

> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/Barfoot_SER2_md/07_Matrix_Lie_Groups/07_Matrix_Lie_Groups.md`（3334 行，全篇读毕）
> **服务章节**：李群与李代数（`parts/P0_math/lie_theory.tex`）
> **抽取定位**：本章为黄金范本，目标「全量保真 + 加深补全」。两项专门任务：
> 1. 清偿 `lie_theory.tex:561` 处的 **Sim(3) $\mathbf{J}_s$ 闭式 punt**（Barfoot ch7 不含 Sim(3)，已从 Ethan Eade `lie_groups.pdf` + Strasdat 2010 thesis + Sophus `Sim3::exp`/`calcW` 取复现级闭式——见**附录 S**）。
> 2. 复审 Barfoot ch7 各 SO(3)/SE(3) 推导/例是否全录，缺则补。
>
> 注：源 md 用「8.x」公式/小节编号（书第二版整体编号沿用），章名为「第 7 章」。本留痕沿用源里的 8.x 公式号以便回溯。

---

## 0. 记号约定（务必先读）—— 本源 vs 本书统一约定

| 概念 | **Barfoot 本源记号** | **本书统一约定** | 差异/转换说明 |
|---|---|---|---|
| 旋转矩阵 | $\mathbf{C}$（direction Cosine matrix），$\mathbf{C}\in SO(3)$ | $\mathbf{R}\in SO(3)$ | **字母不同**：综合时 $\mathbf{C}\mapsto\mathbf{R}$。 |
| 旋转李代数向量 | $\boldsymbol{\phi}\in\mathbb{R}^3$，$\boldsymbol{\phi}=\phi\mathbf{a}$，$\phi=|\boldsymbol{\phi}|$ 角，$\mathbf{a}$ 单位轴 | $\boldsymbol{\phi}$ | 一致。 |
| $\mathfrak{so}(3)$ 元素 | $\boldsymbol{\Phi}=\boldsymbol{\phi}^\wedge\in\mathbb{R}^{3\times3}$ | 同 | 一致。 |
| hat/vee | $(\cdot)^\wedge,(\cdot)^\vee$ | 同 | 一致。 |
| 位姿/变换矩阵 | $\mathbf{T}=\begin{bmatrix}\mathbf{C}&\mathbf{r}\\\mathbf{0}^T&1\end{bmatrix}\in SE(3)$ | $\mathbf{T}=\begin{bmatrix}\mathbf{R}&\mathbf{t}\\\mathbf{0}^T&1\end{bmatrix}$ | 平移 $\mathbf{r}\mapsto\mathbf{t}$，旋转 $\mathbf{C}\mapsto\mathbf{R}$。 |
| $\mathfrak{se}(3)$ 排序 | $\boldsymbol{\xi}=[\boldsymbol{\rho};\boldsymbol{\phi}]\in\mathbb{R}^6$，**平移 $\boldsymbol{\rho}$ 在前、旋转 $\boldsymbol{\phi}$ 在后** | $\boldsymbol{\xi}=[\boldsymbol{\rho};\boldsymbol{\phi}]$ | **一致**（与本书、Sophus 同序）。注意李代数平移 $\boldsymbol{\rho}\neq$ 群平移 $\mathbf{r}=\mathbf{J}\boldsymbol{\rho}$。 |
| $\mathfrak{se}(3)$ 的 $\wedge$ | $\boldsymbol{\xi}^\wedge=\begin{bmatrix}\boldsymbol{\phi}^\wedge&\boldsymbol{\rho}\\\mathbf{0}^T&0\end{bmatrix}\in\mathbb{R}^{4\times4}$ | 同 | 一致。 |
| 群伴随 | $\mathcal{T}=\mathrm{Ad}(\mathbf{T})\in\mathbb{R}^{6\times6}$（花体 $\mathcal{T}$） | $\mathrm{Ad}(\mathbf{T})$ | 一致。 |
| 代数伴随 | $\mathrm{ad}(\boldsymbol{\xi}^\wedge)=\boldsymbol{\xi}^{\curlywedge}=\begin{bmatrix}\boldsymbol{\phi}^\wedge&\boldsymbol{\rho}^\wedge\\\mathbf{0}&\boldsymbol{\phi}^\wedge\end{bmatrix}$ | $\mathrm{ad}_{\boldsymbol{\xi}}$ | 源用 curlywedge（OCR 时成 `^`/`λ`/`⋏`），本留痕统一记 $\boldsymbol{\xi}^{\curlywedge}$（$6\times6$）；其逆记 $\curlyvee$。 |
| 左/右雅可比 | $\mathbf{J}_\ell,\mathbf{J}_r$（SO(3)）；$\mathcal{J}_\ell,\mathcal{J}_r$（SE(3) $6\times6$） | 本书**右扰动为主** | **关键差异**：Barfoot **默认 $\mathbf{J}=\mathbf{J}_\ell$**（§8.1.5 "Choosing the Left"，脚注14）。本书右扰动，用 $\mathbf{J}_r(\boldsymbol{\phi})=\mathbf{J}_\ell(-\boldsymbol{\phi})$、$\mathbf{J}_\ell=\mathbf{C}\mathbf{J}_r$ 转换。 |
| 扰动/噪声注入 | **左扰动**默认：$\mathbf{C}=\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{C}}$，$\mathbf{T}=\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{T}}$ | 本书**右扰动** $\mathbf{T}=\bar{\mathbf{T}}\exp(\boldsymbol{\epsilon}^\wedge)$ | **关键差异**。Barfoot 明言可任选，他选左；本书选右。 |
| 优化扰动 | 左乘 $\exp(\boldsymbol{\psi}^\wedge)\mathbf{C}_{\rm op}$（脚注19 右版亦可） | 右乘更新 | 同上。 |
| 协方差字母 | $\boldsymbol{\Sigma}$（切空间协方差），$\mathbf{Q}$（过程噪声），$\mathbf{R}$（测量噪声/也作投影旋转），$\boldsymbol{\Xi}$（pose+landmark $9\times9$） | $\boldsymbol{\Sigma},\mathbf{Q},\mathbf{R}$ | 一致。$\boldsymbol{\Xi}$ 别处也指 $\mathfrak{se}(3)$ 元素 $\boldsymbol{\xi}^\wedge$，靠上下文区分。 |
| 四元数 | 本章**不用**四元数 | Hamilton | 无冲突。 |
| 角速度 | $\boldsymbol{\omega}$（脚注22：相比第6章 (7.45) 符号**相反**，机器人学约定 $\mathbf{C}=\exp(\boldsymbol{\phi}^\wedge)$，$\dot{\mathbf{C}}=\boldsymbol{\omega}^\wedge\mathbf{C}$） | $\boldsymbol{\omega}$ | 跨章注意符号。 |
| 广义速度 | $\boldsymbol{\varpi}=[\boldsymbol{\nu};\boldsymbol{\omega}]\in\mathbb{R}^6$ | — | $\dot{\mathbf{T}}=\boldsymbol{\varpi}^\wedge\mathbf{T}$。 |
| 简化记号 | **不用** Solà 的 $\mathrm{Exp}/\mathrm{Ln}$（脚注8 提及但为与一版一致不采用） | 可用 $\mathrm{Exp}/\mathrm{Log}$ | Barfoot 写 $\exp(\boldsymbol{\phi}^\wedge)$、$\ln(\mathbf{C})^\vee$。 |

**curlyhat 约定**：$\boldsymbol{\xi}^\wedge$ = $4\times4$；$\boldsymbol{\xi}^{\curlywedge}=\mathrm{ad}(\boldsymbol{\xi}^\wedge)$ = $6\times6$。凡 $\mathcal{T}=\exp(\boldsymbol{\xi}^{\curlywedge})$、$\mathcal{J}=\sum\frac{1}{(n+1)!}(\boldsymbol{\xi}^{\curlywedge})^n$ 均指 $6\times6$ 版本。

---

## 1. §8.1 几何（Geometry）

### §8.1.1 特殊正交群与特殊欧氏群

引言：旋转**不是**向量空间，而是**非交换群（non-Abelian）**。聚焦 $SO(3)$（旋转）、$SE(3)$（位姿）。参考 Stillwell (2008)、Chirikjian (2009)。Marius Sophus Lie (1842–1899)。

**特殊正交群**（式 8.1）：$SO(3)=\{\mathbf{C}\in\mathbb{R}^{3\times3}\mid\mathbf{C}\mathbf{C}^T=\mathbf{1},\det\mathbf{C}=1\}$。
$\mathbf{C}\mathbf{C}^T=\mathbf{1}$ 给 6 约束→3 自由度。（式 8.2）$(\det\mathbf{C})^2=\det(\mathbf{C}\mathbf{C}^T)=1$，（式 8.3）$\det\mathbf{C}=\pm1$；取 $+1$ 为真旋转（脚注1：$-1$ 为 improper rotation/rotary reflection）。无 $\det=1$ 则为 $O(3)$。$SO(3)$ 非向量子空间（脚注2）：对加法不封闭（式 8.4）$\mathbf{C}_1+\mathbf{C}_2\notin SO(3)$，$\mathbf{0}\notin SO(3)$。

**特殊欧氏群**（式 8.5）：$SE(3)=\{\mathbf{T}=\begin{bmatrix}\mathbf{C}&\mathbf{r}\\\mathbf{0}^T&1\end{bmatrix}\mid\mathbf{C}\in SO(3),\mathbf{r}\in\mathbb{R}^3\}$，亦非向量子空间。

**群定义**：四公理（封闭、结合、单位元、可逆）。**李群**=群+微分流形+运算光滑（脚注4）。**矩阵李群**：元素为矩阵、合成=矩阵乘、逆=矩阵逆。脚注3：非 Abelian。

**表 8.1**：

| 性质 | $SO(3)$ | $SE(3)$ |
|---|---|---|
| 封闭 | $\mathbf{C}_1,\mathbf{C}_2\in SO(3)\Rightarrow\mathbf{C}_1\mathbf{C}_2\in SO(3)$ | $\mathbf{T}_1\mathbf{T}_2\in SE(3)$ |
| 结合 | $\mathbf{C}_1(\mathbf{C}_2\mathbf{C}_3)=(\mathbf{C}_1\mathbf{C}_2)\mathbf{C}_3$ | $\mathbf{T}_1(\mathbf{T}_2\mathbf{T}_3)=(\mathbf{T}_1\mathbf{T}_2)\mathbf{T}_3$ |
| 单位元 | $\mathbf{C}\mathbf{1}=\mathbf{1}\mathbf{C}=\mathbf{C}$ | $\mathbf{T}\mathbf{1}=\mathbf{1}\mathbf{T}=\mathbf{T}$ |
| 可逆 | $\mathbf{C}^{-1}\in SO(3)$ | $\mathbf{T}^{-1}\in SE(3)$ |

**封闭证明** $SO(3)$（式 8.6a,b）：$(\mathbf{C}_1\mathbf{C}_2)(\mathbf{C}_1\mathbf{C}_2)^T=\mathbf{C}_1\underbrace{\mathbf{C}_2\mathbf{C}_2^T}_{\mathbf{1}}\mathbf{C}_1^T=\mathbf{1}$，$\det(\mathbf{C}_1\mathbf{C}_2)=\det\mathbf{C}_1\det\mathbf{C}_2=1$。
$SE(3)$（式 8.7）：$\mathbf{T}_1\mathbf{T}_2=\begin{bmatrix}\mathbf{C}_1\mathbf{C}_2&\mathbf{C}_1\mathbf{r}_2+\mathbf{r}_1\\\mathbf{0}^T&1\end{bmatrix}\in SE(3)$。结合律由矩阵乘（脚注5），单位元 $\mathbf{1}$。
**可逆** $SO(3)$（式 8.8a,b）：$(\mathbf{C}^{-1})(\mathbf{C}^{-1})^T=\mathbf{C}^T\mathbf{C}=\mathbf{1}$，$\det(\mathbf{C}^{-1})=1$。
$SE(3)$（式 8.9）：$\mathbf{T}^{-1}=\begin{bmatrix}\mathbf{C}^T&-\mathbf{C}^T\mathbf{r}\\\mathbf{0}^T&1\end{bmatrix}\in SE(3)$。

### §8.1.2 李代数

**李代数**=向量空间 $\mathbb{V}$ over 域 $\mathbb{F}$（取 $\mathbb{R}$）+ 李括号 $[\cdot,\cdot]$，四性质：封闭 $[\mathbf{X},\mathbf{Y}]\in\mathbb{V}$；双线性 $[a\mathbf{X}+b\mathbf{Y},\mathbf{Z}]=a[\mathbf{X},\mathbf{Z}]+b[\mathbf{Y},\mathbf{Z}]$（对第二变元亦然）；交替 $[\mathbf{X},\mathbf{X}]=\mathbf{0}$；Jacobi $[\mathbf{X},[\mathbf{Y},\mathbf{Z}]]+[\mathbf{Y},[\mathbf{Z},\mathbf{X}]]+[\mathbf{Z},[\mathbf{X},\mathbf{Y}]]=\mathbf{0}$。李代数向量空间=群在单位元处切空间，刻画局部结构。Jacobi (1804–1851)。

**$\mathfrak{so}(3)$**：向量空间 $\{\boldsymbol{\Phi}=\boldsymbol{\phi}^\wedge\}$；域 $\mathbb{R}$；李括号 $[\boldsymbol{\Phi}_1,\boldsymbol{\Phi}_2]=\boldsymbol{\Phi}_1\boldsymbol{\Phi}_2-\boldsymbol{\Phi}_2\boldsymbol{\Phi}_1$。反对称（式 8.10）：
$$\boldsymbol{\phi}^\wedge=\begin{bmatrix}\phi_1\\\phi_2\\\phi_3\end{bmatrix}^\wedge=\begin{bmatrix}0&-\phi_3&\phi_2\\\phi_3&0&-\phi_1\\-\phi_2&\phi_1&0\end{bmatrix}.$$
逆（式 8.11）$\boldsymbol{\phi}=\boldsymbol{\Phi}^\vee$。四性质：封闭（式 8.12）$[\boldsymbol{\Phi}_1,\boldsymbol{\Phi}_2]=(\boldsymbol{\phi}_1^\wedge\boldsymbol{\phi}_2)^\wedge\in\mathfrak{so}(3)$；双线性（$\wedge$ 线性）；交替（式 8.13）；Jacobi（代入验）。

**$\mathfrak{se}(3)$**：向量空间 $\{\boldsymbol{\Xi}=\boldsymbol{\xi}^\wedge\in\mathbb{R}^{4\times4}\}$；域 $\mathbb{R}$；李括号同。$\wedge$（式 8.14）$\boldsymbol{\xi}^\wedge=\begin{bmatrix}\boldsymbol{\phi}^\wedge&\boldsymbol{\rho}\\\mathbf{0}^T&0\end{bmatrix}$（Murray et al. 1994 overloading）。逆（式 8.15）。封闭（式 8.16）$[\boldsymbol{\Xi}_1,\boldsymbol{\Xi}_2]=(\boldsymbol{\xi}_1^{\curlywedge}\boldsymbol{\xi}_2)^\wedge$，$6\times6$（式 8.17）$\boldsymbol{\xi}^{\curlywedge}=\begin{bmatrix}\boldsymbol{\phi}^\wedge&\boldsymbol{\rho}^\wedge\\\mathbf{0}&\boldsymbol{\phi}^\wedge\end{bmatrix}$；交替（式 8.18）；Jacobi 同验。
关系（待指数映射）：$SO(3)\leftrightarrow\mathfrak{so}(3)$，$SE(3)\leftrightarrow\mathfrak{se}(3)$。

### §8.1.3 指数映射

矩阵指数（式 8.19）$\exp(\mathbf{A})=\sum_{n=0}^{\infty}\frac{1}{n!}\mathbf{A}^n$；对数（式 8.20）$\ln(\mathbf{A})=\sum_{n=1}^{\infty}\frac{(-1)^{n-1}}{n}(\mathbf{A}-\mathbf{1})^n$。

**旋转**（式 8.21）$\mathbf{C}=\exp(\boldsymbol{\phi}^\wedge)=\sum_n\frac{1}{n!}(\boldsymbol{\phi}^\wedge)^n$；反向（式 8.22）$\boldsymbol{\phi}=\ln(\mathbf{C})^\vee$。脚注8：Solà 用 $\mathrm{Exp}/\mathrm{Ln}$。**满射非单射**（脚注9：奇异性/非唯一）。

前向闭式（式 8.23）令 $\boldsymbol{\phi}=\phi\mathbf{a}$：
$$\exp(\phi\mathbf{a}^\wedge)=\mathbf{a}\mathbf{a}^T+\underbrace{\left(\phi-\tfrac{\phi^3}{3!}+\cdots\right)}_{\sin\phi}\mathbf{a}^\wedge-\underbrace{\left(1-\tfrac{\phi^2}{2!}+\cdots\right)}_{\cos\phi}\underbrace{\mathbf{a}^\wedge\mathbf{a}^\wedge}_{-\mathbf{1}+\mathbf{a}\mathbf{a}^T}=\cos\phi\,\mathbf{1}+(1-\cos\phi)\mathbf{a}\mathbf{a}^T+\sin\phi\,\mathbf{a}^\wedge.$$
此即 Rodrigues 轴-角形式。恒等式（单位 $\mathbf{a}$）（式 8.24a,b）：$\mathbf{a}^\wedge\mathbf{a}^\wedge\equiv-\mathbf{1}+\mathbf{a}\mathbf{a}^T$，$\mathbf{a}^\wedge\mathbf{a}^\wedge\mathbf{a}^\wedge\equiv-\mathbf{a}^\wedge$（后者为 $\mathfrak{so}(3)$ 最小多项式，Appendix B.3.1）。加 $2\pi m$ 同 $\mathbf{C}$（式 8.25）$\mathbf{C}=\exp((\phi+2\pi m)\mathbf{a}^\wedge)$；限 $|\phi|<\pi$ 唯一。

逆向闭式：（式 8.26）$\mathbf{C}\mathbf{a}=\mathbf{a}$（$\mathbf{a}$ 是特征值 1 的特征向量，脚注10：$\mathbf{C}=\mathbf{1}$ 时 $\mathbf{a}$ 不唯一）。角度（式 8.27）$\mathrm{tr}(\mathbf{C})=2\cos\phi+1$，（式 8.28）$\phi=\cos^{-1}\!\left(\frac{\mathrm{tr}(\mathbf{C})-1}{2}\right)+2\pi m$（取 $|\phi|<\pi$；$\phi$ 符号歧义可代回验证）。**图 8.1**：平面旋转切线即一维李代数。

$\det\mathbf{C}=1$（Jacobi 公式，式 8.29）$\det(\exp\mathbf{A})=\exp(\mathrm{tr}\mathbf{A})$，（式 8.30）$\det\mathbf{C}=\exp(\mathrm{tr}(\boldsymbol{\phi}^\wedge))=\exp(0)=1$。

**位姿**（式 8.31）$\mathbf{T}=\exp(\boldsymbol{\xi}^\wedge)$；反向（式 8.32）$\boldsymbol{\xi}=\ln(\mathbf{T})^\vee$。满射非单射。前向（式 8.33）：
$$\exp(\boldsymbol{\xi}^\wedge)=\begin{bmatrix}\sum_n\frac{1}{n!}(\boldsymbol{\phi}^\wedge)^n&\left(\sum_n\frac{1}{(n+1)!}(\boldsymbol{\phi}^\wedge)^n\right)\boldsymbol{\rho}\\\mathbf{0}^T&1\end{bmatrix}=\begin{bmatrix}\mathbf{C}&\mathbf{r}\\\mathbf{0}^T&1\end{bmatrix},$$
（式 8.34）$\mathbf{r}=\mathbf{J}\boldsymbol{\rho}$，$\mathbf{J}=\sum_n\frac{1}{(n+1)!}(\boldsymbol{\phi}^\wedge)^n$。**图 8.2**：六分量变长方体位姿。逆向：$\mathbf{C}\to\boldsymbol{\phi}$，（式 8.35）$\boldsymbol{\rho}=\mathbf{J}^{-1}\mathbf{r}$。

**雅可比 $\mathbf{J}$（左雅可比 SO(3)）** 定义（式 8.36）。闭式（式 8.37a,b）：
$$\mathbf{J}=\tfrac{\sin\phi}{\phi}\mathbf{1}+\left(1-\tfrac{\sin\phi}{\phi}\right)\mathbf{a}\mathbf{a}^T+\tfrac{1-\cos\phi}{\phi}\mathbf{a}^\wedge,\quad \mathbf{J}^{-1}=\tfrac{\phi}{2}\cot\tfrac{\phi}{2}\mathbf{1}+\left(1-\tfrac{\phi}{2}\cot\tfrac{\phi}{2}\right)\mathbf{a}\mathbf{a}^T-\tfrac{\phi}{2}\mathbf{a}^\wedge.$$
奇异 $\phi=2\pi m$（$m\neq0$）。$\mathbf{J}\mathbf{J}^T$（式 8.38）$=\gamma\mathbf{1}+(1-\gamma)\mathbf{a}\mathbf{a}^T$，$(\mathbf{J}\mathbf{J}^T)^{-1}=\tfrac1\gamma\mathbf{1}+(1-\tfrac1\gamma)\mathbf{a}\mathbf{a}^T$，$\gamma=2\tfrac{1-\cos\phi}{\phi^2}$。
正定证（式 8.39）$\phi\neq0,\mathbf{x}\neq0$：$\mathbf{x}^T\mathbf{J}\mathbf{J}^T\mathbf{x}=\underbrace{(\mathbf{a}^T\mathbf{x})^2}_{\ge0}+\underbrace{2\tfrac{1-\cos\phi}{\phi^2}}_{>0}\underbrace{(\mathbf{a}^\wedge\mathbf{x})^T(\mathbf{a}^\wedge\mathbf{x})}_{\ge0}>0$（不能同时为零）。
积分形式（式 8.40）$\mathbf{J}=\int_0^1\mathbf{C}^\alpha d\alpha$，证（式 8.41）$\int_0^1\exp(\alpha\boldsymbol{\phi}^\wedge)d\alpha=\sum_n\frac{1}{(n+1)!}(\boldsymbol{\phi}^\wedge)^n$。关系（式 8.42）$\mathbf{C}=\mathbf{1}+\boldsymbol{\phi}^\wedge\mathbf{J}$（$\boldsymbol{\phi}^\wedge$ 不可逆故不能解 $\mathbf{J}$）。

**（2ed）$\mathbf{T}$ 直接级数**：$\mathfrak{se}(3)$ 最小多项式（Appendix B.3.2，式 8.43）$(\boldsymbol{\xi}^\wedge)^4+\phi^2(\boldsymbol{\xi}^\wedge)^2=\mathbf{0}$。（式 8.44）：
$$\mathbf{T}=\mathbf{1}+\boldsymbol{\xi}^\wedge+\left(\tfrac{1-\cos\phi}{\phi^2}\right)(\boldsymbol{\xi}^\wedge)^2+\left(\tfrac{\phi-\sin\phi}{\phi^3}\right)(\boldsymbol{\xi}^\wedge)^3.$$

### §8.1.4（2ed）伴随（Adjoints）

$SO(3)$ 伴随=群本身（略）。$SE(3)$ 伴随不同。
**群伴随**（共轭）（式 8.45）$\mathrm{Ad}_{\mathbf{T}}\mathbf{x}^\wedge=\mathbf{T}\mathbf{x}^\wedge\mathbf{T}^{-1}$，等价（式 8.46）$=(\mathcal{T}\mathbf{x})^\wedge$。$6\times6$（式 8.47）：
$$\mathcal{T}=\mathrm{Ad}(\mathbf{T})=\begin{bmatrix}\mathbf{C}&\mathbf{r}^\wedge\mathbf{C}\\\mathbf{0}&\mathbf{C}\end{bmatrix}.$$
（式 8.48）$\mathrm{Ad}(SE(3))$ 是矩阵李群。封闭（式 8.49，用式 8.50 $\mathbf{C}\mathbf{v}^\wedge\mathbf{C}^T=(\mathbf{C}\mathbf{v})^\wedge$）：
$$\mathcal{T}_1\mathcal{T}_2=\begin{bmatrix}\mathbf{C}_1\mathbf{C}_2&(\mathbf{C}_1\mathbf{r}_2+\mathbf{r}_1)^\wedge\mathbf{C}_1\mathbf{C}_2\\\mathbf{0}&\mathbf{C}_1\mathbf{C}_2\end{bmatrix}=\mathrm{Ad}(\mathbf{T}_1\mathbf{T}_2).$$
可逆（式 8.51）$\mathcal{T}^{-1}=\begin{bmatrix}\mathbf{C}^T&-\mathbf{C}^T\mathbf{r}^\wedge\\\mathbf{0}&\mathbf{C}^T\end{bmatrix}=\mathrm{Ad}(\mathbf{T}^{-1})$。

**代数伴随** $\mathrm{ad}$（式 8.52）$\mathrm{ad}_{\boldsymbol{\xi}^\wedge}\mathbf{x}^\wedge=[\boldsymbol{\xi}^\wedge,\mathbf{x}^\wedge]$。联系（式 8.53）$\frac{d}{dt}\mathrm{Ad}_{\exp(t\boldsymbol{\xi}^\wedge)}\mathbf{x}^\wedge|_{t=0}=\mathrm{ad}_{\boldsymbol{\xi}^\wedge}\mathbf{x}^\wedge$；等价（式 8.54）$=(\boldsymbol{\xi}^{\curlywedge}\mathbf{x})^\wedge$。（式 8.55,8.56）$\mathrm{ad}(\boldsymbol{\Xi})=\boldsymbol{\xi}^{\curlywedge}=\begin{bmatrix}\boldsymbol{\phi}^\wedge&\boldsymbol{\rho}^\wedge\\\mathbf{0}&\boldsymbol{\phi}^\wedge\end{bmatrix}$。（大写 $\mathrm{Ad}$=群，小写 $\mathrm{ad}$=代数。）

**$\mathrm{ad}(\mathfrak{se}(3))$**：李括号四性质同验（式 8.57,8.58）。

**伴随指数映射**（式 8.59）$\mathcal{T}=\exp(\boldsymbol{\xi}^{\curlywedge})$，反向（式 8.60）$\boldsymbol{\xi}=\ln(\mathcal{T})^{\curlyvee}$。**交换图**（式 8.61）$\mathrm{Ad}\circ\exp=\exp\circ\mathrm{ad}$。需证（式 8.62）$\mathrm{Ad}(\exp(\boldsymbol{\xi}^\wedge))=\exp(\mathrm{ad}(\boldsymbol{\xi}^\wedge))$。
右端（式 8.63）$\exp(\boldsymbol{\xi}^{\curlywedge})=\begin{bmatrix}\mathbf{C}&\mathbf{K}\\\mathbf{0}&\mathbf{C}\end{bmatrix}$，$\mathbf{K}=\sum_{n,m}\frac{1}{(n+m+1)!}(\boldsymbol{\phi}^\wedge)^n\boldsymbol{\rho}^\wedge(\boldsymbol{\phi}^\wedge)^m$。左端（式 8.64）$=\begin{bmatrix}\mathbf{C}&(\mathbf{J}\boldsymbol{\rho})^\wedge\mathbf{C}\\\mathbf{0}&\mathbf{C}\end{bmatrix}$。证 $\mathbf{K}=(\mathbf{J}\boldsymbol{\rho})^\wedge\mathbf{C}$（式 8.65）：
$$(\mathbf{J}\boldsymbol{\rho})^\wedge\mathbf{C}=\int_0^1\mathbf{C}^\alpha\boldsymbol{\rho}^\wedge\mathbf{C}^{1-\alpha}d\alpha=\sum_{n,m}\tfrac{1}{n!m!}\left(\int_0^1\alpha^n(1-\alpha)^m d\alpha\right)(\boldsymbol{\phi}^\wedge)^n\boldsymbol{\rho}^\wedge(\boldsymbol{\phi}^\wedge)^m,$$
用 **Beta 积分**（式 8.66）$\int_0^1\alpha^n(1-\alpha)^m d\alpha=\frac{n!m!}{(n+m+1)!}$，证毕。

**（2ed）$\mathcal{T}$ 直接级数**：$\mathrm{ad}(\mathfrak{se}(3))$ 最小多项式（Appendix B.3.3，式 8.67）$(\boldsymbol{\xi}^{\curlywedge})^5+2\phi^2(\boldsymbol{\xi}^{\curlywedge})^3+\phi^4\boldsymbol{\xi}^{\curlywedge}=\mathbf{0}$。（式 8.68）：
$$
\mathcal{T}=\mathbf{1}+\left(\tfrac{3\sin\phi-\phi\cos\phi}{2\phi}\right)\boldsymbol{\xi}^{\curlywedge}+\left(\tfrac{4-\phi\sin\phi-4\cos\phi}{2\phi^2}\right)(\boldsymbol{\xi}^{\curlywedge})^2+\left(\tfrac{\sin\phi-\phi\cos\phi}{2\phi^3}\right)(\boldsymbol{\xi}^{\curlywedge})^3+\left(\tfrac{2-\phi\sin\phi-2\cos\phi}{2\phi^4}\right)(\boldsymbol{\xi}^{\curlywedge})^4.
$$

### §8.1.5 Baker–Campbell–Hausdorff（BCH）

标量 $\exp(a)\exp(b)=\exp(a+b)$（式 8.69）。矩阵需 **BCH**（式 8.70）：
$$\ln(\exp\mathbf{A}\exp\mathbf{B})=\sum_{n=1}^{\infty}\frac{(-1)^{n-1}}{n}\sum_{\substack{r_i+s_i>0}}\frac{(\sum_i(r_i+s_i))^{-1}}{\prod_i r_i!s_i!}[\mathbf{A}^{r_1}\mathbf{B}^{s_1}\cdots\mathbf{A}^{r_n}\mathbf{B}^{s_n}],$$
嵌套括号（式 8.71）：$[\mathbf{A}^{r_1}\mathbf{B}^{s_1}\cdots]=[\underbrace{\mathbf{A},[\mathbf{A},\dots}_{r_1}[\underbrace{\mathbf{B},\dots}_{s_1}\dots[\underbrace{\mathbf{B},\dots[\mathbf{B}}_{s_n},\mathbf{B}]\dots]]$，若 $s_n>1$ 或（$s_n=0,r_n>1$）则零。$[\mathbf{A},\mathbf{B}]=\mathbf{A}\mathbf{B}-\mathbf{B}\mathbf{A}$（式 8.72）。$[\mathbf{A},\mathbf{B}]=\mathbf{0}$ 时（式 8.73）$=\mathbf{A}+\mathbf{B}$。人物：Baker (1866–1956)、Campbell (1862–1924)、Hausdorff (1868–1942)、Poincaré。

前几项（式 8.74）：
$$
\begin{aligned}
\ln(\exp\mathbf{A}\exp\mathbf{B})=&\,\mathbf{A}+\mathbf{B}+\tfrac12[\mathbf{A},\mathbf{B}]+\tfrac{1}{12}[\mathbf{A},[\mathbf{A},\mathbf{B}]]-\tfrac{1}{12}[\mathbf{B},[\mathbf{A},\mathbf{B}]]-\tfrac{1}{24}[\mathbf{B},[\mathbf{A},[\mathbf{A},\mathbf{B}]]]\\
&-\tfrac{1}{720}([[[[\mathbf{A},\mathbf{B}],\mathbf{B}],\mathbf{B}],\mathbf{B}]+[[[[\mathbf{B},\mathbf{A}],\mathbf{A}],\mathbf{A}],\mathbf{A}])\\
&+\tfrac{1}{360}([[[[\mathbf{A},\mathbf{B}],\mathbf{B}],\mathbf{B}],\mathbf{A}]+[[[[\mathbf{B},\mathbf{A}],\mathbf{A}],\mathbf{A}],\mathbf{B}])\\
&+\tfrac{1}{120}([[[[\mathbf{A},\mathbf{B}],\mathbf{A}],\mathbf{A}],\mathbf{A}]+[[[[\mathbf{B},\mathbf{A}],\mathbf{B}],\mathbf{B}],\mathbf{B}])+\cdots.
\end{aligned}
$$
仅 $\mathbf{A}$ 线性（Klarsfeld & Oteo 1989，式 8.75）$\approx\mathbf{B}+\sum_n\frac{B_n}{n!}\underbrace{[\mathbf{B},[\mathbf{B},\dots[\mathbf{B}}_n,\mathbf{A}]\dots]]$。仅 $\mathbf{B}$ 线性（式 8.76）$\approx\mathbf{A}+\sum_n(-1)^n\frac{B_n}{n!}\underbrace{[\mathbf{A},\dots[\mathbf{A}}_n,\mathbf{B}]\dots]$。

**Bernoulli 数**（式 8.77）：$B_0=1,B_1=-\tfrac12,B_2=\tfrac16,B_3=0,B_4=-\tfrac{1}{30},B_5=0,B_6=\tfrac{1}{42},B_7=0,B_8=-\tfrac{1}{30},B_9=0,B_{10}=\tfrac{5}{66},B_{11}=0,B_{12}=-\tfrac{691}{2730},B_{13}=0,B_{14}=\tfrac76,B_{15}=0,\dots$。奇 $n>1$ 时 $B_n=0$。（脚注12：第一序列；历史 Jakob Bernoulli、Seki Kōwa、Ada Lovelace Note G。）
**Lie 乘积**（式 8.78）$\exp(\mathbf{A}+\mathbf{B})=\lim_{\alpha\to\infty}(\exp(\mathbf{A}/\alpha)\exp(\mathbf{B}/\alpha))^\alpha$。

**旋转 BCH** 精确（式 8.79）$\ln(\mathbf{C}_1\mathbf{C}_2)^\vee=\boldsymbol{\phi}_1+\boldsymbol{\phi}_2+\tfrac12\boldsymbol{\phi}_1^\wedge\boldsymbol{\phi}_2+\tfrac{1}{12}\boldsymbol{\phi}_1^\wedge\boldsymbol{\phi}_1^\wedge\boldsymbol{\phi}_2+\tfrac{1}{12}\boldsymbol{\phi}_2^\wedge\boldsymbol{\phi}_2^\wedge\boldsymbol{\phi}_1+\cdots$。小角（式 8.80）$\approx\begin{cases}\mathbf{J}_\ell(\boldsymbol{\phi}_2)^{-1}\boldsymbol{\phi}_1+\boldsymbol{\phi}_2&\boldsymbol{\phi}_1\text{小}\\\boldsymbol{\phi}_1+\mathbf{J}_r(\boldsymbol{\phi}_1)^{-1}\boldsymbol{\phi}_2&\boldsymbol{\phi}_2\text{小}\end{cases}$。

右/左雅可比逆（式 8.81a,b）：
$$\mathbf{J}_r^{-1}=\sum_n\tfrac{B_n}{n!}(-\boldsymbol{\phi}^\wedge)^n=\tfrac{\phi}{2}\cot\tfrac{\phi}{2}\mathbf{1}+(1-\tfrac{\phi}{2}\cot\tfrac{\phi}{2})\mathbf{a}\mathbf{a}^T+\tfrac{\phi}{2}\mathbf{a}^\wedge,$$
$$\mathbf{J}_\ell^{-1}=\sum_n\tfrac{B_n}{n!}(\boldsymbol{\phi}^\wedge)^n=\tfrac{\phi}{2}\cot\tfrac{\phi}{2}\mathbf{1}+(1-\tfrac{\phi}{2}\cot\tfrac{\phi}{2})\mathbf{a}\mathbf{a}^T-\tfrac{\phi}{2}\mathbf{a}^\wedge.$$
右/左雅可比（式 8.82a,b）：
$$\mathbf{J}_r=\int_0^1\mathbf{C}^{-\alpha}d\alpha=\tfrac{\sin\phi}{\phi}\mathbf{1}+(1-\tfrac{\sin\phi}{\phi})\mathbf{a}\mathbf{a}^T-\tfrac{1-\cos\phi}{\phi}\mathbf{a}^\wedge,$$
$$\mathbf{J}_\ell=\int_0^1\mathbf{C}^{\alpha}d\alpha=\tfrac{\sin\phi}{\phi}\mathbf{1}+(1-\tfrac{\sin\phi}{\phi})\mathbf{a}\mathbf{a}^T+\tfrac{1-\cos\phi}{\phi}\mathbf{a}^\wedge.$$
**关系**（式 8.83）$\mathbf{J}_\ell=\mathbf{C}\mathbf{J}_r$，证（式 8.84）$\mathbf{C}\int_0^1\mathbf{C}^{-\alpha}d\alpha=\int_0^1\mathbf{C}^{1-\alpha}d\alpha=\mathbf{J}_\ell$。（式 8.85）$\mathbf{J}_\ell(-\boldsymbol{\phi})=\mathbf{J}_r(\boldsymbol{\phi})$，证（式 8.86）。

> **★ 本书右扰动**：用 $\mathbf{J}_r(\boldsymbol{\phi})=\mathbf{J}_\ell(-\boldsymbol{\phi})$、$\mathbf{J}_\ell=\mathbf{C}\mathbf{J}_r$ 转换。

**位姿 BCH** 精确（式 8.87a,b，$4\times4$ 与 $6\times6$ 同形）$\ln(\mathbf{T}_1\mathbf{T}_2)^\vee=\boldsymbol{\xi}_1+\boldsymbol{\xi}_2+\tfrac12\boldsymbol{\xi}_1^{\curlywedge}\boldsymbol{\xi}_2+\tfrac{1}{12}\boldsymbol{\xi}_1^{\curlywedge}\boldsymbol{\xi}_1^{\curlywedge}\boldsymbol{\xi}_2+\tfrac{1}{12}\boldsymbol{\xi}_2^{\curlywedge}\boldsymbol{\xi}_2^{\curlywedge}\boldsymbol{\xi}_1+\cdots$。小角（式 8.88a,b）$\approx\begin{cases}\mathcal{J}_\ell(\boldsymbol{\xi}_2)^{-1}\boldsymbol{\xi}_1+\boldsymbol{\xi}_2\\\boldsymbol{\xi}_1+\mathcal{J}_r(\boldsymbol{\xi}_1)^{-1}\boldsymbol{\xi}_2\end{cases}$。逆（式 8.89a,b）$\mathcal{J}_r^{-1}=\sum_n\frac{B_n}{n!}(-\boldsymbol{\xi}^{\curlywedge})^n$，$\mathcal{J}_\ell^{-1}=\sum_n\frac{B_n}{n!}(\boldsymbol{\xi}^{\curlywedge})^n$。

SE(3) 雅可比分块（式 8.90a,b）：$\mathcal{J}_r=\int_0^1\mathcal{T}^{-\alpha}d\alpha=\begin{bmatrix}\mathbf{J}_r&\mathbf{Q}_r\\\mathbf{0}&\mathbf{J}_r\end{bmatrix}$，$\mathcal{J}_\ell=\int_0^1\mathcal{T}^{\alpha}d\alpha=\begin{bmatrix}\mathbf{J}_\ell&\mathbf{Q}_\ell\\\mathbf{0}&\mathbf{J}_\ell\end{bmatrix}$。

**$\mathbf{Q}_\ell$ 闭式**（式 8.91a，脚注13：很长但精确）：
$$\mathbf{Q}_\ell=\sum_{n,m}\frac{(\boldsymbol{\phi}^\wedge)^n\boldsymbol{\rho}^\wedge(\boldsymbol{\phi}^\wedge)^m}{(n+m+2)!}=\tfrac12\boldsymbol{\rho}^\wedge+\left(\tfrac{\phi-\sin\phi}{\phi^3}\right)(\boldsymbol{\phi}^\wedge\boldsymbol{\rho}^\wedge+\boldsymbol{\rho}^\wedge\boldsymbol{\phi}^\wedge+\boldsymbol{\phi}^\wedge\boldsymbol{\rho}^\wedge\boldsymbol{\phi}^\wedge)$$
$$+\left(\tfrac{\phi^2+2\cos\phi-2}{2\phi^4}\right)(\boldsymbol{\phi}^\wedge\boldsymbol{\phi}^\wedge\boldsymbol{\rho}^\wedge+\boldsymbol{\rho}^\wedge\boldsymbol{\phi}^\wedge\boldsymbol{\phi}^\wedge-3\boldsymbol{\phi}^\wedge\boldsymbol{\rho}^\wedge\boldsymbol{\phi}^\wedge)+\left(\tfrac{2\phi-3\sin\phi+\phi\cos\phi}{2\phi^5}\right)(\boldsymbol{\phi}^\wedge\boldsymbol{\rho}^\wedge\boldsymbol{\phi}^\wedge\boldsymbol{\phi}^\wedge+\boldsymbol{\phi}^\wedge\boldsymbol{\phi}^\wedge\boldsymbol{\rho}^\wedge\boldsymbol{\phi}^\wedge).$$
$\mathbf{Q}_r$（式 8.91b）$=\mathbf{Q}_\ell(-\boldsymbol{\xi})=\mathbf{C}^T\mathbf{Q}_\ell-\mathbf{C}^T(\mathbf{J}_\ell\boldsymbol{\rho})^\wedge\mathbf{J}_\ell$。
关系（式 8.92）$\mathcal{J}_\ell=\mathcal{T}\mathcal{J}_r$，$\mathcal{J}_\ell(-\boldsymbol{\xi})=\mathcal{J}_r(\boldsymbol{\xi})$，证（式 8.93,8.94）。

**$\mathcal{J}_\ell$ 直接级数**（由 $\mathcal{T}=\mathbf{1}+\boldsymbol{\xi}^{\curlywedge}\mathcal{J}_\ell$，式 8.95–8.98）解系数得（式 8.99）：
$$\mathcal{J}_\ell=\mathbf{1}+\left(\tfrac{4-\phi\sin\phi-4\cos\phi}{2\phi^2}\right)\boldsymbol{\xi}^{\curlywedge}+\left(\tfrac{4\phi-5\sin\phi+\phi\cos\phi}{2\phi^3}\right)(\boldsymbol{\xi}^{\curlywedge})^2+\left(\tfrac{2-\phi\sin\phi-2\cos\phi}{2\phi^4}\right)(\boldsymbol{\xi}^{\curlywedge})^3+\left(\tfrac{2\phi-3\sin\phi+\phi\cos\phi}{2\phi^5}\right)(\boldsymbol{\xi}^{\curlywedge})^4.$$
逆分块（式 8.100a,b）$\mathcal{J}^{-1}=\begin{bmatrix}\mathbf{J}^{-1}&-\mathbf{J}^{-1}\mathbf{Q}\mathbf{J}^{-1}\\\mathbf{0}&\mathbf{J}^{-1}\end{bmatrix}$。奇异点相同（式 8.101）$\det\mathcal{J}_r=(\det\mathbf{J}_r)^2$。
$\rho$ 关系（式 8.102a,b）$\mathbf{T}=\begin{bmatrix}\mathbf{C}&\mathbf{J}_\ell\boldsymbol{\rho}\\\mathbf{0}^T&1\end{bmatrix}=\begin{bmatrix}\mathbf{C}&\mathbf{C}\mathbf{J}_r\boldsymbol{\rho}\\\mathbf{0}^T&1\end{bmatrix}$。
$\mathcal{J}\mathcal{J}^T>0$（式 8.103）三因子分解皆 $>0$。

**选定左**（脚注14）：$\mathbf{J}=\mathbf{J}_\ell$、$\mathcal{J}=\mathcal{J}_\ell$。SO(3)（式 8.104）$\approx\begin{cases}\mathbf{J}(\boldsymbol{\phi}_2)^{-1}\boldsymbol{\phi}_1+\boldsymbol{\phi}_2\\\boldsymbol{\phi}_1+\mathbf{J}(-\boldsymbol{\phi}_1)^{-1}\boldsymbol{\phi}_2\end{cases}$。SE(3)（式 8.105a,b）同形。

### §8.1.6（2ed）距离、体积、积分

**旋转**：两差（式 8.106a,b）$\boldsymbol{\phi}_{12}=\ln(\mathbf{C}_1^T\mathbf{C}_2)^\vee$（右），$\boldsymbol{\phi}_{21}=\ln(\mathbf{C}_2\mathbf{C}_1^T)^\vee$（左）。内积（式 8.107）$\langle\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge\rangle=\tfrac12\mathrm{tr}(\boldsymbol{\phi}_1^\wedge\boldsymbol{\phi}_2^{\wedge T})=\boldsymbol{\phi}_1^T\boldsymbol{\phi}_2$。距离（式 8.108a,b）$\phi_{12}=\sqrt{\boldsymbol{\phi}_{12}^T\boldsymbol{\phi}_{12}}=|\boldsymbol{\phi}_{12}|$。
扰动差（式 8.109a,b）$\ln(\delta\mathbf{C}_r)^\vee\approx\mathbf{J}_r\delta\boldsymbol{\phi}$，$\ln(\delta\mathbf{C}_\ell)^\vee\approx\mathbf{J}_\ell\delta\boldsymbol{\phi}$。体积元（式 8.110a,b）$d\mathbf{C}_r=|\det\mathbf{J}_r|d\boldsymbol{\phi}$。（式 8.111）$\det\mathbf{J}_\ell=\det(\mathbf{C}\mathbf{J}_r)=\det\mathbf{J}_r$（unimodular 群左右同），（式 8.112）$d\mathbf{C}=|\det\mathbf{J}|d\boldsymbol{\phi}$。
（式 8.113）$|\det\mathbf{J}|=2\tfrac{1-\cos\phi}{\phi^2}=1-\tfrac{1}{12}\phi^2+\tfrac{1}{360}\phi^4-\tfrac{1}{20160}\phi^6+\cdots$。积分（式 8.114）$\int_{SO(3)}f\,d\mathbf{C}\to\int_{|\boldsymbol{\phi}|<\pi}f|\det\mathbf{J}|d\boldsymbol{\phi}$。

**位姿**：距离（式 8.115a,b）$\boldsymbol{\xi}_{12}=\ln(\mathbf{T}_1^{-1}\mathbf{T}_2)^\vee$，$\boldsymbol{\xi}_{21}=\ln(\mathbf{T}_2\mathbf{T}_1^{-1})^\vee$。内积（式 8.116a,b）$\langle\boldsymbol{\xi}_1^\wedge,\boldsymbol{\xi}_2^\wedge\rangle=\mathrm{tr}(\boldsymbol{\xi}_1^\wedge\begin{bmatrix}\tfrac12\mathbf{1}&\mathbf{0}\\\mathbf{0}^T&1\end{bmatrix}\boldsymbol{\xi}_2^{\wedge T})=\boldsymbol{\xi}_1^T\boldsymbol{\xi}_2$（$6\times6$ 用 $\begin{bmatrix}\mathbf{0}&\mathbf{0}\\\mathbf{0}&\tfrac12\mathbf{1}\end{bmatrix}$）。距离（式 8.117a,b）$\xi_{12}=|\boldsymbol{\xi}_{12}|$。
扰动差（式 8.120a,b）$\ln(\delta\mathbf{T}_r)^\vee\approx\mathcal{J}_r\delta\boldsymbol{\xi}$，$\ln(\delta\mathbf{T}_\ell)^\vee\approx\mathcal{J}_\ell\delta\boldsymbol{\xi}$。体积元（式 8.121a,b）。（式 8.122）$\det\mathcal{J}_\ell=\det\mathcal{J}_r$（$\det\mathcal{T}=1$），（式 8.123）$d\mathbf{T}=|\det\mathcal{J}|d\boldsymbol{\xi}$。（式 8.124）$|\det\mathcal{J}|=|\det\mathbf{J}|^2=1-\tfrac16\phi^2+\tfrac{1}{80}\phi^4-\tfrac{17}{30240}\phi^6+\cdots$。积分（式 8.125）$\int_{SE(3)}f\,d\mathbf{T}=\int_{\mathbb{R}^3,|\boldsymbol{\phi}|<\pi}f|\det\mathcal{J}|d\boldsymbol{\xi}$。

### §8.1.7 插值（Interpolation）

线性插值（式 8.126）破坏封闭（式 8.127a,b）$(1-\alpha)\mathbf{C}_1+\alpha\mathbf{C}_2\notin SO(3)$。

**旋转**方案（式 8.128）$\mathbf{C}=(\mathbf{C}_2\mathbf{C}_1^T)^\alpha\mathbf{C}_1$。$\mathbf{C}_{21}=\exp(\boldsymbol{\phi}^\wedge)=\mathbf{C}_2\mathbf{C}_1^T$，（式 8.129）$\mathbf{C}_{21}^\alpha=\exp(\alpha\boldsymbol{\phi}^\wedge)\in SO(3)$。类比（式 8.130,8.131）。小 $\boldsymbol{\varphi}$（式 8.132）$\boldsymbol{\varphi}\approx\alpha\mathbf{J}(\boldsymbol{\phi}_1)^{-1}\boldsymbol{\phi}+\boldsymbol{\phi}_1$。$\mathbf{C}_1=\mathbf{1}$（式 8.133）$\boldsymbol{\varphi}=\alpha\boldsymbol{\phi}_2$。
**恒定角速度**（式 8.134–8.136）$\mathbf{C}(t)=\exp((t-t_1)\boldsymbol{\omega}^\wedge)\mathbf{C}(t_1)$，$\boldsymbol{\omega}=\tfrac{1}{t_2-t_1}\boldsymbol{\phi}$，恰为 Poisson 方程 (7.45) 解（式 8.137）。

**扰动旋转**：左差（式 8.138）。方案（式 8.139）。代入小扰动（式 8.140,8.141），保一阶（式 8.142），化简（式 8.143）：
$$\delta\boldsymbol{\varphi}=(\mathbf{1}-\mathbf{A}(\alpha,\boldsymbol{\phi}))\delta\boldsymbol{\phi}_1+\mathbf{A}(\alpha,\boldsymbol{\phi})\delta\boldsymbol{\phi}_2,\quad \mathbf{A}(\alpha,\boldsymbol{\phi})=\alpha\mathbf{J}(\alpha\boldsymbol{\phi})\mathbf{J}(\boldsymbol{\phi})^{-1}\ (\text{式 8.144}).$$
$\boldsymbol{\phi}$ 小时 $\mathbf{A}\approx\alpha\mathbf{1}$。级数（式 8.145,8.146，Cauchy 乘积）$\mathbf{A}=\sum_n\frac{F_n(\alpha)}{n!}(\boldsymbol{\phi}^\wedge)^n$。**Faulhaber**（式 8.147）$F_n(\alpha)=\frac{1}{n+1}\sum_{m=0}^n\binom{n+1}{m}B_m\alpha^{n+1-m}=\sum_{\beta=0}^{\alpha-1}\beta^n$。系数（式 8.148）$F_0=\alpha,F_1=\tfrac{\alpha(\alpha-1)}{2},F_2=\tfrac{\alpha(\alpha-1)(2\alpha-1)}{6},F_3=\tfrac{\alpha^2(\alpha-1)^2}{4}$。代回（式 8.149）$\mathbf{A}=\alpha\mathbf{1}+\tfrac{\alpha(\alpha-1)}{2}\boldsymbol{\phi}^\wedge+\tfrac{\alpha(\alpha-1)(2\alpha-1)}{12}\boldsymbol{\phi}^\wedge\boldsymbol{\phi}^\wedge+\tfrac{\alpha^2(\alpha-1)^2}{24}(\boldsymbol{\phi}^\wedge)^3+\cdots$。Cauchy (1789–1857)、Faulhaber (1580–1635)。

**另一解释**（$\alpha$ 正整数，式 8.150–8.153）$\mathbf{A}(\alpha,\boldsymbol{\phi})=\sum_{\beta=0}^{\alpha-1}\mathbf{C}^\beta=\sum_n\frac{F_n(\alpha)}{n!}(\boldsymbol{\phi}^\wedge)^n$。Faulhaber 例（式 8.154a–d）同上（$\alpha\in[0,1]$ 仍成立）。

**位姿**方案（式 8.155）$\mathbf{T}=(\mathbf{T}_2\mathbf{T}_1^{-1})^\alpha\mathbf{T}_1$，（式 8.156）$\boldsymbol{\zeta}\approx\alpha\mathcal{J}(\boldsymbol{\xi}_1)^{-1}\boldsymbol{\xi}+\boldsymbol{\xi}_1$。$\mathbf{T}_1=\mathbf{1}$（式 8.157）$\boldsymbol{\zeta}=\alpha\boldsymbol{\xi}_2$。扰动（式 8.158–8.160）$\delta\boldsymbol{\zeta}=(\mathbf{1}-\mathcal{A})\delta\boldsymbol{\xi}_1+\mathcal{A}\delta\boldsymbol{\xi}_2$，（式 8.161）$\mathcal{A}(\alpha,\boldsymbol{\xi})=\alpha\mathcal{J}(\alpha\boldsymbol{\xi})\mathcal{J}(\boldsymbol{\xi})^{-1}$（$6\times6$），级数（式 8.162）$\sum_n\frac{F_n(\alpha)}{n!}(\boldsymbol{\xi}^{\curlywedge})^n$。

### §8.1.8 齐次点

$\mathbf{p}=[sx,sy,sz,s]^T=[\boldsymbol{\varepsilon};\eta]$（Hartley & Zisserman 2000）；$s=0$ 表无穷远（无奇异/尺度，Triggs et al. 2000）；$\mathbf{p}_2=\mathbf{T}_{21}\mathbf{p}_1$。
两算子（脚注18：$\odot$ 类似 Furgale 2011 但带负号）（式 8.163）：
$$[\boldsymbol{\varepsilon};\eta]^\odot=\begin{bmatrix}\eta\mathbf{1}&-\boldsymbol{\varepsilon}^\wedge\\\mathbf{0}^T&\mathbf{0}^T\end{bmatrix}(4\times6),\qquad [\boldsymbol{\varepsilon};\eta]^{\circledcirc}=\begin{bmatrix}\mathbf{0}&\boldsymbol{\varepsilon}\\-\boldsymbol{\varepsilon}^\wedge&\mathbf{0}\end{bmatrix}(6\times4).$$
（源两算子 OCR 皆显示 $\odot$；第二个本留痕记 $\circledcirc$，为 $6\times4$。）恒等式（式 8.164）$\boldsymbol{\xi}^\wedge\mathbf{p}\equiv\mathbf{p}^\odot\boldsymbol{\xi}$，$\mathbf{p}^T\boldsymbol{\xi}^\wedge\equiv\boldsymbol{\xi}^T\mathbf{p}^\odot$。（式 8.165）$(\mathbf{T}\mathbf{p})^\odot\equiv\mathbf{T}\mathbf{p}^\odot\mathcal{T}^{-1}$。

### §8.1.9 微积分与优化

参考 Absil et al. (2009)、Boumal (2022)。

**旋转**：对 $\boldsymbol{\phi}$ 求导（式 8.166）。方向导数（式 8.167），BCH（式 8.168）$\exp((\boldsymbol{\phi}+h\mathbf{1}_i)^\wedge)\approx(\mathbf{1}+h(\mathbf{J}\mathbf{1}_i)^\wedge)\exp(\boldsymbol{\phi}^\wedge)$，得（式 8.169）$\frac{\partial(\mathbf{C}\mathbf{v})}{\partial\phi_i}=-(\mathbf{C}\mathbf{v})^\wedge\mathbf{J}\mathbf{1}_i$，堆叠（式 8.170）：
$$\frac{\partial(\mathbf{C}\mathbf{v})}{\partial\boldsymbol{\phi}}=-(\mathbf{C}\mathbf{v})^\wedge\mathbf{J}.$$
链式（式 8.171）$\frac{\partial u}{\partial\boldsymbol{\phi}}=-\frac{\partial u}{\partial\mathbf{x}}(\mathbf{C}\mathbf{v})^\wedge\mathbf{J}$。梯度下降（式 8.172,8.173）。
**左乘小旋转**（脚注19）（式 8.174）$\mathbf{C}=\exp(\boldsymbol{\psi}^\wedge)\mathbf{C}_{\rm op}$，BCH（式 8.175）$\boldsymbol{\psi}=-\alpha\mathbf{J}\mathbf{J}^T\boldsymbol{\delta}$；丢 $\mathbf{J}\mathbf{J}^T$（式 8.176）$\boldsymbol{\psi}=-\alpha\boldsymbol{\delta}$，仍减（式 8.177）。
**左 Lie 导数**（脚注20）（式 8.178）$\frac{\partial(\mathbf{C}\mathbf{v})}{\partial\psi_i}\approx-(\mathbf{C}\mathbf{v})^\wedge\mathbf{1}_i$，（式 8.179）$\frac{\partial(\mathbf{C}\mathbf{v})}{\partial\boldsymbol{\psi}}=-(\mathbf{C}\mathbf{v})^\wedge$（无 $\mathbf{J}$）。
**纯扰动**（式 8.180,8.181）$\mathbf{C}\mathbf{v}\approx\mathbf{C}_{\rm op}\mathbf{v}-(\mathbf{C}_{\rm op}\mathbf{v})^\wedge\boldsymbol{\psi}$（图 8.3），（式 8.182）$u\approx u(\mathbf{C}_{\rm op}\mathbf{v})+\boldsymbol{\delta}^T\boldsymbol{\psi}$，下降（式 8.183）$\boldsymbol{\psi}=-\alpha\mathbf{D}\boldsymbol{\delta}$，更新（式 8.184）。
**GN on SO(3)**（式 8.185）$J=\tfrac12\sum_m(u_m(\mathbf{C}\mathbf{v}_m))^2$，扰动（式 8.186,8.187），（式 8.188）$J\approx\tfrac12\sum_m(\boldsymbol{\delta}_m^T\boldsymbol{\psi}+\beta_m)^2$，（式 8.189,8.190）$(\sum_m\boldsymbol{\delta}_m\boldsymbol{\delta}_m^T)\boldsymbol{\psi}^\star=-\sum_m\beta_m\boldsymbol{\delta}_m$，更新（式 8.191）。

**位姿**：（式 8.192）$\frac{\partial(\mathbf{T}\mathbf{p})}{\partial\boldsymbol{\xi}}=(\mathbf{T}\mathbf{p})^\odot\mathcal{J}$。左扰动（式 8.193,8.194）$\frac{\partial(\mathbf{T}\mathbf{p})}{\partial\boldsymbol{\epsilon}}=(\mathbf{T}\mathbf{p})^\odot$（无 $\mathcal{J}$）。GN（式 8.195–8.201）$(\sum_m\boldsymbol{\delta}_m\boldsymbol{\delta}_m^T)\boldsymbol{\epsilon}^\star=-\sum_m\beta_m\boldsymbol{\delta}_m$，$\mathbf{T}_{\rm op}\leftarrow\exp(\boldsymbol{\epsilon}^{\star\wedge})\mathbf{T}_{\rm op}$。
**GN 三性质**：(i) 无奇异存储；(ii) 每步无约束；(iii) 矩阵级（避免标量三角函数求导）。可叠 line search、LM（§4.3.1）、鲁棒（§5.4.2）。

### §8.1.10（2ed）黎曼流形上的优化

参考 Absil et al. (2009)、Boumal (2022)、Ablin & Peyré (2021)。Bernhard Riemann (1826–1866)。
**旋转**：约束微分（式 8.202）$d\mathbf{C}\mathbf{C}^T+\mathbf{C}d\mathbf{C}^T=\mathbf{0}$，$d\mathbf{C}=\boldsymbol{\phi}^\wedge\mathbf{C}$ 满足（式 8.203）。加切向量（式 8.204）$(\mathbf{1}+\boldsymbol{\phi}^\wedge)\mathbf{C}$，**retraction**（式 8.205）$\mathbf{R}(\boldsymbol{\phi}^\wedge\mathbf{C},\mathbf{C})=\exp(\boldsymbol{\phi}^\wedge)\mathbf{C}$（无穷多 retraction，Bauchau & Trainelli 2003；SE(3) Barfoot et al. 2021）。
**黎曼梯度**（式 8.206）满足 $\lim_{h\to0}\frac{f(\mathbf{R}(h\boldsymbol{\phi}^\wedge\mathbf{C},\mathbf{C}))-f(\mathbf{C})}{h}=\langle\mathrm{grad}f,\boldsymbol{\phi}^\wedge\mathbf{C}\rangle_{\mathbf{C}}$，内积（式 8.207）。推导（式 8.208）得（式 8.209）$\mathrm{grad}f(\mathbf{C})=\boldsymbol{\psi}^\wedge\mathbf{C}$，$\boldsymbol{\psi}^\wedge=\frac{\partial f}{\partial\mathbf{C}}\mathbf{C}^T-\mathbf{C}\frac{\partial f}{\partial\mathbf{C}}^T$（投影到切空间）。更新（式 8.210）。
**例** $f=\mathbf{u}^T\mathbf{C}\mathbf{v}$（式 8.211–8.213）$\mathrm{grad}f=((\mathbf{C}\mathbf{v})^\wedge\mathbf{u})^\wedge\mathbf{C}$；扰动法（式 8.214–8.216）$\boldsymbol{\phi}=-\alpha(\mathbf{C}_{\rm op}\mathbf{v})^\wedge\mathbf{u}$，一致。
**位姿**：切向量 $\boldsymbol{\xi}^\wedge\mathbf{T}$（式 8.217），retraction（式 8.218）。黎曼梯度（式 8.219–8.224）$\mathrm{grad}f(\mathbf{T})=\boldsymbol{\epsilon}^\wedge\mathbf{T}$，
$$\boldsymbol{\epsilon}^\wedge=\begin{bmatrix}(\frac{\partial f}{\partial\mathbf{C}}\mathbf{C}^T-\mathbf{C}\frac{\partial f}{\partial\mathbf{C}}^T)+(\frac{\partial f}{\partial\mathbf{r}}\mathbf{r}^T-\mathbf{r}\frac{\partial f}{\partial\mathbf{r}}^T)&\frac{\partial f}{\partial\mathbf{r}}\\\mathbf{0}^T&0\end{bmatrix}$$
（脚注21：$\frac{\partial f}{\partial\mathbf{r}}$ 取列）。**例** $f=\mathbf{q}^T\mathbf{T}\mathbf{p}$（式 8.225–8.227）$\boldsymbol{\epsilon}=[\mathbf{u};(\mathbf{C}\mathbf{v}+\mathbf{r})^\wedge\mathbf{u}]$；扰动法（式 8.228,8.229）$\boldsymbol{\xi}=-\alpha[\mathbf{u};(\mathbf{C}_{\rm op}\mathbf{v}+\mathbf{r}_{\rm op})^\wedge\mathbf{u}]$。
**小结**：梯度下降下，扰动法即黎曼梯度下降，指数映射即 retraction。

### §8.1.11 恒等式汇总（Identities）

> 源两页表（SO(3)、SE(3) 各一页），OCR 较乱，据可读部分重构。域：$\alpha,\beta\in\mathbb{R}$；$\mathbf{u},\mathbf{v},\boldsymbol{\phi},\delta\boldsymbol{\phi}\in\mathbb{R}^3$；$\mathbf{p}\in\mathbb{R}^4$；$\mathbf{x},\mathbf{y},\boldsymbol{\xi},\delta\boldsymbol{\xi}\in\mathbb{R}^6$；$\mathbf{C}\in SO(3)$；$\mathbf{J},\mathbf{Q},\mathbf{W}\in\mathbb{R}^{3\times3}$；$\mathbf{T}\in SE(3)$；$\mathcal{T}\in\mathrm{Ad}(SE(3))$；$\mathcal{J},\mathcal{A}\in\mathbb{R}^{6\times6}$。

**SO(3)**：
- $(\alpha\mathbf{u}+\beta\mathbf{v})^\wedge\equiv\alpha\mathbf{u}^\wedge+\beta\mathbf{v}^\wedge$；$\mathbf{u}^\wedge\mathbf{v}\equiv-\mathbf{v}^\wedge\mathbf{u}$；$\mathbf{u}^\wedge\mathbf{u}\equiv\mathbf{0}$。
- $\mathbf{u}^\wedge\mathbf{v}^\wedge\equiv-(\mathbf{u}^T\mathbf{v})\mathbf{1}+\mathbf{v}\mathbf{u}^T$。
- $\mathbf{u}^\wedge\mathbf{W}\mathbf{u}^\wedge\equiv\mathbf{u}^\wedge(\mathrm{tr}(\mathbf{W})\mathbf{1}-\mathbf{W})-\mathbf{W}^T\mathbf{u}^\wedge$。
- 李括号 $[\mathbf{u}^\wedge,\mathbf{v}^\wedge]\equiv(\mathbf{u}^\wedge\mathbf{v})^\wedge$。
- $\mathbf{C}=\exp(\boldsymbol{\phi}^\wedge)\equiv\cos\phi\mathbf{1}+(1-\cos\phi)\mathbf{a}\mathbf{a}^T+\sin\phi\mathbf{a}^\wedge\approx\mathbf{1}+\boldsymbol{\phi}^\wedge$；$\mathbf{C}^{-1}\equiv\mathbf{C}^T\approx\mathbf{1}-\boldsymbol{\phi}^\wedge$。
- $(\mathbf{C}\mathbf{u})^\wedge\equiv\mathbf{C}\mathbf{u}^\wedge\mathbf{C}^T$；$\exp((\mathbf{C}\mathbf{u})^\wedge)\equiv\mathbf{C}\exp(\mathbf{u}^\wedge)\mathbf{C}^T$；$\mathbf{C}\mathbf{a}^\wedge\equiv\mathbf{a}^\wedge\mathbf{C}$。
- $\mathrm{tr}(\mathbf{C})\equiv2\cos\phi+1$；$\det\mathbf{C}\equiv1$；$\mathbf{C}\mathbf{a}\equiv\mathbf{a}$；$\mathbf{C}\boldsymbol{\phi}=\boldsymbol{\phi}$。
- $\mathbf{J}=\int_0^1\mathbf{C}^\alpha d\alpha\equiv\frac{\sin\phi}{\phi}\mathbf{1}+(1-\frac{\sin\phi}{\phi})\mathbf{a}\mathbf{a}^T+\frac{1-\cos\phi}{\phi}\mathbf{a}^\wedge\approx\mathbf{1}+\tfrac12\boldsymbol{\phi}^\wedge$；$\mathbf{J}^{-1}\equiv\frac{\phi}{2}\cot\frac{\phi}{2}\mathbf{1}+(1-\frac{\phi}{2}\cot\frac{\phi}{2})\mathbf{a}\mathbf{a}^T-\frac{\phi}{2}\mathbf{a}^\wedge\approx\mathbf{1}-\tfrac12\boldsymbol{\phi}^\wedge$。
- $\mathbf{C}\equiv\mathbf{1}+\boldsymbol{\phi}^\wedge\mathbf{J}$；$\mathbf{J}(\boldsymbol{\phi})\equiv\mathbf{C}\mathbf{J}(-\boldsymbol{\phi})$。
- $(\exp(\delta\boldsymbol{\phi}^\wedge)\mathbf{C})^\alpha\approx(\mathbf{1}+(\mathbf{A}(\alpha,\boldsymbol{\phi})\delta\boldsymbol{\phi})^\wedge)\mathbf{C}^\alpha$，$\mathbf{A}=\alpha\mathbf{J}(\alpha\boldsymbol{\phi})\mathbf{J}(\boldsymbol{\phi})^{-1}=\sum_n\frac{F_n(\alpha)}{n!}(\boldsymbol{\phi}^\wedge)^n$。

**SE(3)**：
- $\mathcal{T}=\exp(\boldsymbol{\xi}^{\curlywedge})\approx\mathbf{1}+\boldsymbol{\xi}^{\curlywedge}$（闭式见式 8.68）；$\mathcal{T}^{-1}\equiv\exp(-\boldsymbol{\xi}^{\curlywedge})\approx\mathbf{1}-\boldsymbol{\xi}^{\curlywedge}$。
- $\mathbf{T}=\exp(\boldsymbol{\xi}^\wedge)\approx\mathbf{1}+\boldsymbol{\xi}^\wedge$；$\mathbf{T}^{-1}\equiv\begin{bmatrix}\mathbf{C}^T&-\mathbf{C}^T\mathbf{r}\\\mathbf{0}^T&1\end{bmatrix}$。
- $(\mathcal{T}\mathbf{x})^\wedge\equiv\mathbf{T}\mathbf{x}^\wedge\mathbf{T}^{-1}$；$\exp((\mathcal{T}\mathbf{x})^\wedge)\equiv\mathbf{T}\exp(\mathbf{x}^\wedge)\mathbf{T}^{-1}$；$\exp((\mathcal{T}\mathbf{x})^{\curlywedge})\equiv\mathcal{T}\exp(\mathbf{x}^{\curlywedge})\mathcal{T}^{-1}$。
- $\mathcal{T}=\mathrm{Ad}(\mathbf{T})\equiv\begin{bmatrix}\mathbf{C}&(\mathbf{J}\boldsymbol{\rho})^\wedge\mathbf{C}\\\mathbf{0}&\mathbf{C}\end{bmatrix}$；$\mathrm{tr}(\mathbf{T})\equiv2\cos\phi+2$；$\det\mathbf{T}\equiv1$；$\mathrm{Ad}(\mathbf{T}_1\mathbf{T}_2)=\mathrm{Ad}(\mathbf{T}_1)\mathrm{Ad}(\mathbf{T}_2)$。
- $\mathcal{J}=\int_0^1\mathcal{T}^\alpha d\alpha\equiv\begin{bmatrix}\mathbf{J}&\mathbf{Q}\\\mathbf{0}&\mathbf{J}\end{bmatrix}\approx\mathbf{1}+\tfrac12\boldsymbol{\xi}^{\curlywedge}$；$\mathcal{J}^{-1}\equiv\begin{bmatrix}\mathbf{J}^{-1}&-\mathbf{J}^{-1}\mathbf{Q}\mathbf{J}^{-1}\\\mathbf{0}&\mathbf{J}^{-1}\end{bmatrix}\approx\mathbf{1}-\tfrac12\boldsymbol{\xi}^{\curlywedge}$。
- $\mathcal{T}\equiv\mathbf{1}+\boldsymbol{\xi}^{\curlywedge}\mathcal{J}$；$\mathcal{J}(\boldsymbol{\xi})\equiv\mathcal{T}\mathcal{J}(-\boldsymbol{\xi})$。
- $(\exp(\delta\boldsymbol{\xi}^\wedge)\mathbf{T})^\alpha\approx(\mathbf{1}+(\mathcal{A}(\alpha,\boldsymbol{\xi})\delta\boldsymbol{\xi})^\wedge)\mathbf{T}^\alpha$，$\mathcal{A}=\alpha\mathcal{J}(\alpha\boldsymbol{\xi})\mathcal{J}(\boldsymbol{\xi})^{-1}$。
- $(\mathbf{T}\mathbf{p})^\odot\equiv\mathbf{T}\mathbf{p}^\odot\mathcal{T}^{-1}$；$(\mathbf{T}\mathbf{p})^{\odot T}(\mathbf{T}\mathbf{p})^\odot\equiv\mathcal{T}^{-T}\mathbf{p}^{\odot T}\mathbf{p}^\odot\mathcal{T}^{-1}$。

> 注：含 $\mathbf{W}$（任意 $3\times3$）的 $\mathbf{u}^\wedge\mathbf{W}\mathbf{v}^\wedge$ 类几条在源 md 中 OCR 严重损坏，成书时务必回查 Barfoot 原书第 7 章恒等式两页。

---

## 2. §8.2 运动学（Kinematics）

### §8.2.1 旋转

**李群**（式 8.231）$\mathbf{C}=\exp(\boldsymbol{\phi}^\wedge)$。**Poisson 方程**（式 8.232）$\dot{\mathbf{C}}=\boldsymbol{\omega}^\wedge\mathbf{C}$ 或 $\boldsymbol{\omega}^\wedge=\dot{\mathbf{C}}\mathbf{C}^T$（脚注22：$\boldsymbol{\omega}$ 与 (7.45) 符号相反）。无奇异有约束。

**李代数**：矩阵指数导数一般式（式 8.234）$\frac{d}{dt}\exp(\mathbf{A}(t))=\int_0^1\exp(\alpha\mathbf{A})\dot{\mathbf{A}}\exp((1-\alpha)\mathbf{A})d\alpha$。（式 8.233,8.235）：
$$\dot{\mathbf{C}}\mathbf{C}^T=\int_0^1\mathbf{C}^\alpha\dot{\boldsymbol{\phi}}^\wedge\mathbf{C}^{-\alpha}d\alpha=\left(\int_0^1\mathbf{C}^\alpha d\alpha\,\dot{\boldsymbol{\phi}}\right)^\wedge=(\mathbf{J}\dot{\boldsymbol{\phi}})^\wedge.$$
（式 8.236,8.237）$\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$，$\dot{\boldsymbol{\phi}}=\mathbf{J}^{-1}\boldsymbol{\omega}$（$\mathbf{J}=\mathbf{J}_\ell$，$|\boldsymbol{\phi}|=2\pi m$ 奇异，无约束）。

**（2ed）数值积分**：
- **(A) 分段恒定 $\boldsymbol{\omega}$**（式 8.238）$\mathbf{C}(t_2)=\exp(\boldsymbol{\omega}^\wedge\Delta t)\mathbf{C}(t_1)$，（式 8.239）$\boldsymbol{\phi}=\boldsymbol{\omega}\Delta t$，（式 8.240）闭式 $\mathbf{C}_{21}$，（式 8.241）$\mathbf{C}(t_2)=\mathbf{C}_{21}\mathbf{C}(t_1)$。
- **(B) Magnus 展开**（$\boldsymbol{\omega}$ 线性，式 8.242）（Magnus 1954；Blanes et al. 2009）（式 8.243）。前三项（Huber & Wollherr 2020，式 8.244a–c）：$\boldsymbol{\phi}_1=\boldsymbol{\omega}_1\Delta t+\tfrac12\boldsymbol{\alpha}_1\Delta t^2$，$\boldsymbol{\phi}_2=\tfrac{1}{12}\boldsymbol{\alpha}_1^\wedge\boldsymbol{\omega}_1\Delta t^3$，$\boldsymbol{\phi}_3=\tfrac{1}{240}\boldsymbol{\alpha}_1^\wedge\boldsymbol{\alpha}_1^\wedge\boldsymbol{\omega}_1\Delta t^5$。（式 8.245）$\mathbf{C}(t_2)\approx\exp((\boldsymbol{\omega}_1\Delta t+\tfrac12\boldsymbol{\alpha}_1\Delta t^2+\tfrac{1}{12}\boldsymbol{\alpha}_1^\wedge\boldsymbol{\omega}_1\Delta t^3+\tfrac{1}{240}\boldsymbol{\alpha}_1^\wedge\boldsymbol{\alpha}_1^\wedge\boldsymbol{\omega}_1\Delta t^5)^\wedge)\mathbf{C}(t_1)$。
- **(C) 投影回 SO(3)**（Green 1952，式 8.246）$\arg\max_{\mathbf{R}}\mathrm{tr}(\mathbf{C}\mathbf{R}^T)-\tfrac12\sum_{ij}\lambda_{ij}(\mathbf{r}_i^T\mathbf{r}_j-\delta_{ij})$。求导置零（式 8.249,8.250）$\boldsymbol{\Lambda}\mathbf{R}=\mathbf{C}$，$\boldsymbol{\Lambda}=\boldsymbol{\Lambda}^T$。由 $\boldsymbol{\Lambda}^2=\mathbf{C}\mathbf{C}^T$ 得 $\boldsymbol{\Lambda}=(\mathbf{C}\mathbf{C}^T)^{1/2}$，最终 $\mathbf{R}=(\mathbf{C}\mathbf{C}^T)^{-1/2}\mathbf{C}$，$\mathbf{C}\leftarrow\mathbf{R}$（式 8.251）。脚注23：病态 $\mathbf{C}$ 可能得 $\det\mathbf{R}=-1$；严格法用 SVD（§9.1.3）；查 $\det\mathbf{C}>0$。

**（2ed）输运定理**（式 8.252–8.254）$\mathbf{v}_i=\mathbf{C}_{iv}\mathbf{v}_v$，代 $\dot{\mathbf{C}}_{iv}=-\mathbf{C}_{iv}\boldsymbol{\omega}_v^{iv\wedge}$：$\dot{\mathbf{v}}_i=\mathbf{C}_{iv}(\dot{\mathbf{v}}_v-\boldsymbol{\omega}_v^{iv\wedge}\mathbf{v}_v)$。

### §8.2.2 位姿

**李群**（式 8.255）$\mathbf{T}=\begin{bmatrix}\mathbf{C}&\mathbf{J}\boldsymbol{\rho}\\\mathbf{0}^T&1\end{bmatrix}=\exp(\boldsymbol{\xi}^\wedge)$。分离（式 8.256a,b）$\dot{\mathbf{r}}=\boldsymbol{\omega}^\wedge\mathbf{r}+\boldsymbol{\nu}$，$\dot{\mathbf{C}}=\boldsymbol{\omega}^\wedge\mathbf{C}$。等价（式 8.257）$\dot{\mathbf{T}}=\boldsymbol{\varpi}^\wedge\mathbf{T}$，$\boldsymbol{\varpi}=[\boldsymbol{\nu};\boldsymbol{\omega}]$。脚注24：$\dot{\mathcal{T}}=\boldsymbol{\varpi}^{\curlywedge}\mathcal{T}$。
**李代数**（式 8.258,8.259）$\dot{\mathbf{T}}\mathbf{T}^{-1}=(\mathcal{J}\dot{\boldsymbol{\xi}})^\wedge$，$\mathcal{J}=\int_0^1\mathcal{T}^\alpha d\alpha$。（式 8.260,8.261）$\boldsymbol{\varpi}=\mathcal{J}\dot{\boldsymbol{\xi}}$，$\dot{\boldsymbol{\xi}}=\mathcal{J}^{-1}\boldsymbol{\varpi}$。
**混合 Hybrid**（式 8.262）$\begin{bmatrix}\dot{\mathbf{r}}\\\dot{\boldsymbol{\phi}}\end{bmatrix}=\begin{bmatrix}\mathbf{1}&-\mathbf{r}^\wedge\\\mathbf{0}&\mathbf{J}^{-1}\end{bmatrix}\begin{bmatrix}\boldsymbol{\nu}\\\boldsymbol{\omega}\end{bmatrix}$（有 $\mathbf{J}^{-1}$ 奇异，无需 $\mathbf{Q}$，无约束）。
**（2ed）数值积分**：(A) 分段恒定 $\boldsymbol{\varpi}$（式 8.263–8.267）$\mathbf{T}(t_2)=\mathbf{T}_{21}\mathbf{T}(t_1)$，$\boldsymbol{\xi}=\boldsymbol{\varpi}\Delta t$。(B) Magnus（式 8.268,8.269）$\mathbf{T}(t_2)\approx\exp((\boldsymbol{\varpi}_1\Delta t+\tfrac12\boldsymbol{\gamma}_1\Delta t^2+\tfrac{1}{12}\boldsymbol{\gamma}_1^\wedge\boldsymbol{\varpi}_1\Delta t^3+\tfrac{1}{240}\boldsymbol{\gamma}_1^\wedge\boldsymbol{\gamma}_1^\wedge\boldsymbol{\varpi}_1\Delta t^5)^\wedge)\mathbf{T}(t_1)$。脚注25：投影旋转块、重置下块。
**含动力学**（D'Eleuterio 1985，式 8.270a,b）：
$$\dot{\mathbf{T}}=\boldsymbol{\varpi}^\wedge\mathbf{T},\qquad \dot{\boldsymbol{\varpi}}=-\mathcal{M}^{-1}\boldsymbol{\varpi}^{\curlywedge T}\mathcal{M}\boldsymbol{\varpi}+\mathbf{a},$$
广义质量（式 8.271）$\mathcal{M}=\begin{bmatrix}m\mathbf{1}&-m\mathbf{c}^\wedge\\m\mathbf{c}^\wedge&\mathbf{I}\end{bmatrix}$。
**（2ed）输运定理**（式 8.272–8.276）$\dot{\mathbf{p}}_i=\mathbf{T}_{iv}(\dot{\mathbf{p}}_v-\boldsymbol{\varpi}_v^{iv\wedge}\mathbf{p}_v)=\mathbf{T}_{iv}(\dot{\mathbf{p}}_v-\mathbf{p}_v^\odot\boldsymbol{\varpi}_v^{iv})$；静态点 $\dot{\mathbf{p}}_v=\mathbf{p}_v^\odot\boldsymbol{\varpi}_v^{iv}$。

### §8.2.3 线性化旋转

**李群** 扰动（式 8.277）$\mathbf{C}'=\exp(\delta\boldsymbol{\phi}^\wedge)\mathbf{C}$，分解（式 8.279a,b）：
$$\text{标称}:\dot{\mathbf{C}}=\boldsymbol{\omega}^\wedge\mathbf{C},\qquad \text{扰动}:\delta\dot{\boldsymbol{\phi}}=\boldsymbol{\omega}^\wedge\delta\boldsymbol{\phi}+\delta\boldsymbol{\omega}.$$
**李代数**（式 8.280）$\boldsymbol{\phi}'=\boldsymbol{\phi}+\mathbf{J}(\boldsymbol{\phi})^{-1}\delta\boldsymbol{\phi}$。$\delta\mathbf{J}$（式 8.282）$=\int_0^1\alpha(\mathbf{J}(\alpha\boldsymbol{\phi})\mathbf{J}(\boldsymbol{\phi})^{-1}\delta\boldsymbol{\phi})^\wedge\mathbf{C}^\alpha d\alpha$。用恒等式（式 8.285，Appendix B.2.1，脚注26 Hughes 1986）$\dot{\mathbf{J}}-\boldsymbol{\omega}^\wedge\mathbf{J}\equiv\frac{\partial\boldsymbol{\omega}}{\partial\boldsymbol{\phi}}$，得（式 8.286）额外项 $\frac{\partial\boldsymbol{\omega}}{\partial\boldsymbol{\phi}}\mathbf{J}^{-1}\delta\boldsymbol{\phi}-\delta\mathbf{J}\dot{\boldsymbol{\phi}}$，证为零（式 8.287）$=\delta\mathbf{J}\dot{\boldsymbol{\phi}}-\delta\mathbf{J}\dot{\boldsymbol{\phi}}=\mathbf{0}$。故（式 8.288a,b）与李群一致。
**解可交换**（式 8.289–8.291）。**积分解**：LTV（式 8.292,8.293），转移矩阵（式 8.295）$\boldsymbol{\Phi}(t,s)=\mathbf{C}(t)\mathbf{C}(s)^T$（脚注27），解（式 8.296）$\delta\boldsymbol{\phi}(t)=\mathbf{C}(t)\mathbf{C}(0)^T\delta\boldsymbol{\phi}(0)+\mathbf{C}(t)\int_0^t\mathbf{C}(s)^T\delta\boldsymbol{\omega}(s)ds$，验证（式 8.297,8.298a,b）。

### §8.2.4 线性化位姿

**李群**（式 8.299）$\mathbf{T}'=\exp(\delta\boldsymbol{\xi}^\wedge)\mathbf{T}$，分解（式 8.301a,b）：
$$\text{标称}:\dot{\mathbf{T}}=\boldsymbol{\varpi}^\wedge\mathbf{T},\qquad \text{扰动}:\delta\dot{\boldsymbol{\xi}}=\boldsymbol{\varpi}^{\curlywedge}\delta\boldsymbol{\xi}+\delta\boldsymbol{\varpi}.$$
**积分解**：转移矩阵（式 8.302）$\boldsymbol{\Phi}(t,s)=\mathcal{T}(t)\mathcal{T}(s)^{-1}$，解（式 8.303），用 $6\times6$ 标称（式 8.304）$\dot{\mathcal{T}}=\boldsymbol{\varpi}^{\curlywedge}\mathcal{T}$。
**含动力学**（式 8.305–8.308）扰动动力学 $\delta\dot{\boldsymbol{\varpi}}=\mathcal{M}^{-1}((\mathcal{M}\boldsymbol{\varpi})^{\curlywedge T}-\boldsymbol{\varpi}^{\curlywedge}\mathcal{M})\delta\boldsymbol{\varpi}+\delta\mathbf{a}$，组合（式 8.309）：
$$\begin{bmatrix}\delta\dot{\boldsymbol{\xi}}\\\delta\dot{\boldsymbol{\varpi}}\end{bmatrix}=\begin{bmatrix}\boldsymbol{\varpi}^{\curlywedge}&\mathbf{1}\\\mathbf{0}&\mathcal{M}^{-1}((\mathcal{M}\boldsymbol{\varpi})^{\curlywedge T}-\boldsymbol{\varpi}^{\curlywedge}\mathcal{M})\end{bmatrix}\begin{bmatrix}\delta\boldsymbol{\xi}\\\delta\boldsymbol{\varpi}\end{bmatrix}+\begin{bmatrix}\mathbf{0}\\\delta\mathbf{a}\end{bmatrix}.$$

---

## 3. §8.3 概率与统计（Probability and Statistics）

向量空间高斯（式 8.310）$\mathbf{x}\sim\mathcal{N}(\boldsymbol{\mu},\boldsymbol{\Sigma})$（式 8.311）$\mathbf{x}=\boldsymbol{\mu}+\boldsymbol{\epsilon}$。李群对加法不封闭，需另法。本源遵循 Barfoot & Furgale (2014)（小不确定度实用），溯源 Su & Lee、Chirikjian & Kyatkin (2001,2016)、Smith et al. (2003)、Wang & Chirikjian (2006,2008)、Wolfe et al. (2011)、Long et al. (2012)。

### §8.3.1 高斯随机变量与 PDF

李群无奇异有约束（现实需此）；李代数可当向量空间无约束有奇异。**在李代数定义随机变量**。SO(3) 三选项：

| | SO(3) | $\mathfrak{so}(3)$ |
|---|---|---|
| 左 | $\mathbf{C}=\exp(\boldsymbol{\epsilon}_\ell^\wedge)\bar{\mathbf{C}}$ | $\boldsymbol{\phi}\approx\boldsymbol{\mu}+\mathbf{J}_\ell^{-1}(\boldsymbol{\mu})\boldsymbol{\epsilon}_\ell$ |
| 中 | $\mathbf{C}=\exp((\boldsymbol{\mu}+\boldsymbol{\epsilon}_m)^\wedge)$ | $\boldsymbol{\phi}=\boldsymbol{\mu}+\boldsymbol{\epsilon}_m$ |
| 右 | $\mathbf{C}=\bar{\mathbf{C}}\exp(\boldsymbol{\epsilon}_r^\wedge)$ | $\boldsymbol{\phi}\approx\boldsymbol{\mu}+\mathbf{J}_r^{-1}(\boldsymbol{\mu})\boldsymbol{\epsilon}_r$ |

关系 $\boldsymbol{\epsilon}_m\approx\mathbf{J}_\ell^{-1}(\boldsymbol{\mu})\boldsymbol{\epsilon}_\ell\approx\mathbf{J}_r^{-1}(\boldsymbol{\mu})\boldsymbol{\epsilon}_r$。中间需标称入李代数（奇异）；左右可留李群。

> **★ 关键约定差异**：本源**约定左扰动**（脚注31 丢 $\ell$）。**本书右扰动**（上表 right 行）。脚注29：injecting 术语误导（满射非单射；$|\phi|<\pi$ 双射）；脚注30：仅适小扰动。

SO(3) 随机变量（式 8.312）$\mathbf{C}=\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{C}}$。诱导（式 8.313）$p(\boldsymbol{\epsilon})\to p(\mathbf{C})$。高斯（式 8.314）$p(\boldsymbol{\epsilon})=\frac{1}{\sqrt{(2\pi)^3\det\boldsymbol{\Sigma}}}\exp(-\tfrac12\boldsymbol{\epsilon}^T\boldsymbol{\Sigma}^{-1}\boldsymbol{\epsilon})$。归一（式 8.315），体积元（式 8.316）$d\mathbf{C}=|\det\mathbf{J}(\boldsymbol{\epsilon})|d\boldsymbol{\epsilon}$（左扰动 → $\mathbf{J}$ 在小 $\boldsymbol{\epsilon}$ 处 ≈ $\mathbf{1}$）。诱导 $p(\mathbf{C})$（式 8.317–8.319）：
$$p(\mathbf{C})=\frac{1}{\sqrt{(2\pi)^3\det\boldsymbol{\Sigma}}}\exp\!\left(-\tfrac12\ln(\mathbf{C}\bar{\mathbf{C}}^T)^{\vee T}\boldsymbol{\Sigma}^{-1}\ln(\mathbf{C}\bar{\mathbf{C}}^T)^\vee\right)\frac{1}{|\det\mathbf{J}|}.$$
**均值**（式 8.320）$\int\ln(\mathbf{C}\mathbf{M}^T)^\vee p(\mathbf{C})d\mathbf{C}=\mathbf{0}$，取 $\mathbf{M}=\bar{\mathbf{C}}$（式 8.321,8.322）$=E[\boldsymbol{\epsilon}]=\mathbf{0}$。**协方差**（式 8.323）$\boldsymbol{\Sigma}=E[\boldsymbol{\epsilon}\boldsymbol{\epsilon}^T]$。**图 8.4**。
**确定旋转作用**（式 8.324,8.325）$\mathbf{C}'=\mathbf{R}\mathbf{C}=\exp(\boldsymbol{\epsilon}'^\wedge)\bar{\mathbf{C}}'$，$\bar{\mathbf{C}}'=\mathbf{R}\bar{\mathbf{C}}$，$\boldsymbol{\epsilon}'=\mathbf{R}\boldsymbol{\epsilon}\sim\mathcal{N}(\mathbf{0},\mathbf{R}\boldsymbol{\Sigma}\mathbf{R}^T)$（无近似）。
**位姿**（式 8.326）$\mathbf{T}=\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{T}}$，均值（式 8.327,8.328），协方差（式 8.329）。确定变换（式 8.330,8.331）$\boldsymbol{\epsilon}'=\mathcal{R}\boldsymbol{\epsilon}\sim\mathcal{N}(\mathbf{0},\mathcal{R}\boldsymbol{\Sigma}\mathcal{R}^T)$，$\mathcal{R}=\mathrm{Ad}(\mathbf{R})$。**图 8.5**。

### §8.3.2 旋转向量的不确定度

（式 8.332）$\mathbf{y}=\mathbf{C}\mathbf{x}$，$\mathbf{C}=\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{C}}$（式 8.333）。样本在半径 $|\mathbf{x}|$ 球面。$E[\mathbf{y}]$ 三法：采样、sigmapoint、解析。
解析（式 8.334,8.335，奇次为零）$E[\mathbf{y}]=(\mathbf{1}+\tfrac12 E[\boldsymbol{\epsilon}^\wedge\boldsymbol{\epsilon}^\wedge]+\tfrac{1}{24}E[(\boldsymbol{\epsilon}^\wedge)^4]+\cdots)\bar{\mathbf{C}}\mathbf{x}$。
（式 8.336）$E[\boldsymbol{\epsilon}^\wedge\boldsymbol{\epsilon}^\wedge]=-\mathrm{tr}(\boldsymbol{\Sigma})\mathbf{1}+\boldsymbol{\Sigma}$；（式 8.337，Isserlis）$E[(\boldsymbol{\epsilon}^\wedge)^4]=((\mathrm{tr}\boldsymbol{\Sigma})^2+2\mathrm{tr}(\boldsymbol{\Sigma}^2))\mathbf{1}-\boldsymbol{\Sigma}(\mathrm{tr}(\boldsymbol{\Sigma})\mathbf{1}+2\boldsymbol{\Sigma})$。
到四阶（式 8.338）：
$$E[\mathbf{y}]\approx\left(\mathbf{1}+\tfrac12(-\mathrm{tr}(\boldsymbol{\Sigma})\mathbf{1}+\boldsymbol{\Sigma})+\tfrac{1}{24}(((\mathrm{tr}\boldsymbol{\Sigma})^2+2\mathrm{tr}(\boldsymbol{\Sigma}^2))\mathbf{1}-\boldsymbol{\Sigma}(\mathrm{tr}(\boldsymbol{\Sigma})\mathbf{1}+2\boldsymbol{\Sigma}))\right)\bar{\mathbf{C}}\mathbf{x}.$$
"二阶"=保 $\boldsymbol{\epsilon}^2$，"四阶"=保 $\boldsymbol{\epsilon}^4$（≈ sigmapoint ≈ 采样，图 8.5）。

### §8.3.3 复合位姿（Compounding Poses）

**图 8.6**。两噪声位姿 $\{\bar{\mathbf{T}}_1,\boldsymbol{\Sigma}_1\},\{\bar{\mathbf{T}}_2,\boldsymbol{\Sigma}_2\}$（式 8.339）。令（式 8.340）$\mathbf{T}=\mathbf{T}_1\mathbf{T}_2$。扰动（式 8.341）$\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{T}}=\exp(\boldsymbol{\epsilon}_1^\wedge)\bar{\mathbf{T}}_1\exp(\boldsymbol{\epsilon}_2^\wedge)\bar{\mathbf{T}}_2$。移左（式 8.342）$=\exp(\boldsymbol{\epsilon}_1^\wedge)\exp((\bar{\mathcal{T}}_1\boldsymbol{\epsilon}_2)^\wedge)\bar{\mathbf{T}}_1\bar{\mathbf{T}}_2$，$\bar{\mathcal{T}}_1=\mathrm{Ad}(\bar{\mathbf{T}}_1)$。取（式 8.343）$\bar{\mathbf{T}}=\bar{\mathbf{T}}_1\bar{\mathbf{T}}_2$，（式 8.344）$\exp(\boldsymbol{\epsilon}^\wedge)=\exp(\boldsymbol{\epsilon}_1^\wedge)\exp((\bar{\mathcal{T}}_1\boldsymbol{\epsilon}_2)^\wedge)$。令 $\boldsymbol{\epsilon}_2'=\bar{\mathcal{T}}_1\boldsymbol{\epsilon}_2$，BCH（式 8.345）：
$$\boldsymbol{\epsilon}=\boldsymbol{\epsilon}_1+\boldsymbol{\epsilon}_2'+\tfrac12\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_2'+\tfrac{1}{12}\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_2'+\tfrac{1}{12}\boldsymbol{\epsilon}_2'^{\curlywedge}\boldsymbol{\epsilon}_2'^{\curlywedge}\boldsymbol{\epsilon}_1-\tfrac{1}{24}\boldsymbol{\epsilon}_2'^{\curlywedge}\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_2'+\cdots.$$
$E[\boldsymbol{\epsilon}]$（$\boldsymbol{\epsilon}_1\sim\mathcal{N}(\mathbf{0},\boldsymbol{\Sigma}_1)$、$\boldsymbol{\epsilon}_2'\sim\mathcal{N}(\mathbf{0},\boldsymbol{\Sigma}_2')$ 不相关，式 8.346）$=-\tfrac{1}{24}E[\boldsymbol{\epsilon}_2'^{\curlywedge}\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_2']+O(\boldsymbol{\epsilon}^6)$，到三阶 $E[\boldsymbol{\epsilon}]=\mathbf{0}$，故式 8.343 合理（脚注33：若 $\boldsymbol{\Sigma}_1=\mathrm{diag}(\boldsymbol{\Sigma}_{1,\rho\rho},\sigma_{1,\phi\phi}^2\mathbf{1})$ 则四阶项也零，可达五阶——速度积分常见）。

**协方差** $\boldsymbol{\Sigma}=E[\boldsymbol{\epsilon}\boldsymbol{\epsilon}^T]$ 到四阶（式 8.347，省奇次）。两线性算子（式 8.348a,b）：
$$\langle\!\langle\mathbf{A}\rangle\!\rangle=-\mathrm{tr}(\mathbf{A})\mathbf{1}+\mathbf{A},\qquad \langle\!\langle\mathbf{A},\mathbf{B}\rangle\!\rangle=\langle\!\langle\mathbf{A}\rangle\!\rangle\langle\!\langle\mathbf{B}\rangle\!\rangle+\langle\!\langle\mathbf{B}\mathbf{A}\rangle\!\rangle,$$
恒等式（式 8.349）$-\mathbf{u}^\wedge\mathbf{A}\mathbf{v}^\wedge\equiv\langle\!\langle\mathbf{v}\mathbf{u}^T,\mathbf{A}^T\rangle\!\rangle$。逐项（式 8.350a–e）：$E[\boldsymbol{\epsilon}_1\boldsymbol{\epsilon}_1^T]=\boldsymbol{\Sigma}_1=\begin{bmatrix}\boldsymbol{\Sigma}_{1,\rho\rho}&\boldsymbol{\Sigma}_{1,\rho\phi}\\\boldsymbol{\Sigma}_{1,\rho\phi}^T&\boldsymbol{\Sigma}_{1,\phi\phi}\end{bmatrix}$；$E[\boldsymbol{\epsilon}_2'\boldsymbol{\epsilon}_2'^T]=\boldsymbol{\Sigma}_2'=\bar{\mathcal{T}}_1\boldsymbol{\Sigma}_2\bar{\mathcal{T}}_1^T$；$E[\boldsymbol{\epsilon}_1^{\curlywedge}\boldsymbol{\epsilon}_1^{\curlywedge}]=\mathcal{A}_1=\begin{bmatrix}\langle\!\langle\boldsymbol{\Sigma}_{1,\phi\phi}\rangle\!\rangle&\langle\!\langle\boldsymbol{\Sigma}_{1,\rho\phi}+\boldsymbol{\Sigma}_{1,\rho\phi}^T\rangle\!\rangle\\\mathbf{0}&\langle\!\langle\boldsymbol{\Sigma}_{1,\phi\phi}\rangle\!\rangle\end{bmatrix}$；$E[\boldsymbol{\epsilon}_2'^{\curlywedge}\boldsymbol{\epsilon}_2'^{\curlywedge}]=\mathcal{A}_2'$（同形）；$E[\boldsymbol{\epsilon}_1^{\curlywedge}(\boldsymbol{\epsilon}_2'\boldsymbol{\epsilon}_2'^T)\boldsymbol{\epsilon}_1^{\curlywedge T}]=\mathcal{B}=\begin{bmatrix}\mathbf{B}_{\rho\rho}&\mathbf{B}_{\rho\phi}\\\mathbf{B}_{\rho\phi}^T&\mathbf{B}_{\phi\phi}\end{bmatrix}$。
$\mathcal{B}$ 块（式 8.351a–c）：
$$\mathbf{B}_{\rho\rho}=\langle\!\langle\boldsymbol{\Sigma}_{1,\phi\phi},\boldsymbol{\Sigma}_{2,\rho\rho}'\rangle\!\rangle+\langle\!\langle\boldsymbol{\Sigma}_{1,\rho\phi}^T,\boldsymbol{\Sigma}_{2,\rho\phi}'\rangle\!\rangle+\langle\!\langle\boldsymbol{\Sigma}_{1,\rho\phi},\boldsymbol{\Sigma}_{2,\rho\phi}'^T\rangle\!\rangle+\langle\!\langle\boldsymbol{\Sigma}_{1,\rho\rho},\boldsymbol{\Sigma}_{2,\phi\phi}'\rangle\!\rangle,$$
$$\mathbf{B}_{\rho\phi}=\langle\!\langle\boldsymbol{\Sigma}_{1,\phi\phi},\boldsymbol{\Sigma}_{2,\rho\phi}'^T\rangle\!\rangle+\langle\!\langle\boldsymbol{\Sigma}_{1,\rho\phi}^T,\boldsymbol{\Sigma}_{2,\phi\phi}'\rangle\!\rangle,\qquad \mathbf{B}_{\phi\phi}=\langle\!\langle\boldsymbol{\Sigma}_{1,\phi\phi},\boldsymbol{\Sigma}_{2,\phi\phi}'\rangle\!\rangle.$$
**结果**（式 8.352，四阶精确，但协方差仅二阶——同 Wang & Chirikjian 2008）：
$$\boldsymbol{\Sigma}_{4\text{th}}\approx\underbrace{\boldsymbol{\Sigma}_1+\boldsymbol{\Sigma}_2'}_{\boldsymbol{\Sigma}_{2\text{nd}}}+\underbrace{\tfrac14\mathcal{B}+\tfrac{1}{12}(\mathcal{A}_1\boldsymbol{\Sigma}_2'+\boldsymbol{\Sigma}_2'\mathcal{A}_1^T+\mathcal{A}_2'\boldsymbol{\Sigma}_1+\boldsymbol{\Sigma}_1\mathcal{A}_2'^T)}_{\text{额外四阶项}}.$$
（脚注34：六阶项需 Isserlis。）总结：均值用式 8.343、协方差用式 8.352。

**Sigmapoint 法**（Julier & Uhlmann 1996；Hertzberg et al. 2013；Brookshire & Teller 2012）：Cholesky $\mathbf{L}\mathbf{L}^T=\mathrm{diag}(\boldsymbol{\Sigma}_1,\boldsymbol{\Sigma}_2)$，$\boldsymbol{\psi}_\ell=\sqrt{\lambda}\mathrm{col}_\ell\mathbf{L}$，$\boldsymbol{\psi}_{\ell+L}=-\sqrt{\lambda}\mathrm{col}_\ell\mathbf{L}$（$\ell=1\dots L$，$L=12$），$[\boldsymbol{\epsilon}_{1,\ell};\boldsymbol{\epsilon}_{2,\ell}]=\boldsymbol{\psi}_\ell$，$\mathbf{T}_{1,\ell}=\exp(\boldsymbol{\epsilon}_{1,\ell}^\wedge)\bar{\mathbf{T}}_1$，$\mathbf{T}_{2,\ell}=\exp(\boldsymbol{\epsilon}_{2,\ell}^\wedge)\bar{\mathbf{T}}_2$。（式 8.353）$\boldsymbol{\epsilon}_\ell=\ln(\mathbf{T}_{1,\ell}\mathbf{T}_{2,\ell}\bar{\mathbf{T}}^{-1})^\vee$，（式 8.354）$\boldsymbol{\Sigma}_{\rm sp}=\frac{1}{2\lambda}\sum_{\ell=1}^{2L}\boldsymbol{\epsilon}_\ell\boldsymbol{\epsilon}_\ell^T$（脚注35：$\lambda=1$，sigmapoint 旋转分量长 $<\pi$）。**与二阶法代数等价**（噪声不相关）。

**简单复合例**：$K$ 次连乘（式 8.355）$\exp(\boldsymbol{\epsilon}_K^\wedge)\bar{\mathbf{T}}_K=(\prod_{k=1}^K\exp(\boldsymbol{\epsilon}^\wedge)\bar{\mathbf{T}})\exp(\boldsymbol{\epsilon}_0^\wedge)\bar{\mathbf{T}}_0$（即 SE(3) 离散积分）。假设（式 8.356a–c）$\bar{\mathbf{T}}_0=\mathbf{1},\boldsymbol{\epsilon}_0\sim\mathcal{N}(\mathbf{0},\mathbf{0})$；$\bar{\mathbf{C}}=\mathbf{1},\bar{\mathbf{r}}=[r,0,0]^T$，$\boldsymbol{\Sigma}=\mathrm{diag}(0,0,0,0,0,\sigma^2)$（平面单轮车）。二阶（式 8.357a,b）$\bar{\mathbf{T}}_K=\begin{bmatrix}\mathbf{1}&[Kr,0,0]^T\\\mathbf{0}^T&1\end{bmatrix}$，$\boldsymbol{\Sigma}_K$ 的 $(2,2)$ 元 $=\tfrac{K(K-1)(2K-1)}{6}r^2\sigma^2$、$(2,6)$ 元 $=-\tfrac{K(K-1)}{2}r\sigma^2$，**$x$ 方向 $(1,1)$ 元为零**。四阶 $(1,1)$ 元非零（主要经 $\mathbf{B}_{\rho\rho}$）——"香蕉形"分布（Long et al. 2012）。**图 8.7**（$K=100$，$r=1$，$\sigma=0.03$）。

**复合实验**（式 8.358a,b）$\bar{\boldsymbol{\xi}}_1=[0,2,0,\pi/6,0,0]^T$，$\boldsymbol{\Sigma}_1=\alpha\mathrm{diag}\{10,5,5,\tfrac12,1,\tfrac12\}$；$\bar{\boldsymbol{\xi}}_2=[0,0,1,0,\pi/4,0]^T$，$\boldsymbol{\Sigma}_2=\alpha\mathrm{diag}\{5,10,5,\tfrac12,\tfrac12,1\}$。四法：Monte Carlo（$M=10^6$，$\boldsymbol{\Sigma}_{\rm mc}=\frac1M\sum\boldsymbol{\epsilon}_m\boldsymbol{\epsilon}_m^T$）、二阶、四阶、sigmapoint。Frobenius 范数 $\varepsilon=\sqrt{\mathrm{tr}((\boldsymbol{\Sigma}-\boldsymbol{\Sigma}_{\rm mc})^T(\boldsymbol{\Sigma}-\boldsymbol{\Sigma}_{\rm mc}))}$。**图 8.8**：四阶最优（约 7 倍）；二阶与 sigmapoint 等价。建议用相对位姿保持小不确定度。

### §8.3.4 求逆位姿的不确定度（隐含小节）

（式 8.359）$\mathbf{T}=\exp(\boldsymbol{\xi}^\wedge)\bar{\mathbf{T}}$，$\boldsymbol{\xi}\sim\mathcal{N}(\mathbf{0},\boldsymbol{\Sigma})$。逆（式 8.360）：
$$\mathbf{T}^{-1}=\bar{\mathbf{T}}^{-1}\exp(-\boldsymbol{\xi}^\wedge)=\exp((\underbrace{-\bar{\mathcal{T}}^{-1}\boldsymbol{\xi}}_{\boldsymbol{\xi}'})^\wedge)\bar{\mathbf{T}}^{-1},\quad \bar{\mathcal{T}}=\mathrm{Ad}(\bar{\mathbf{T}}).$$
统计（式 8.361a,b）$E[\boldsymbol{\xi}']=\mathbf{0}$，$E[\boldsymbol{\xi}'\boldsymbol{\xi}'^T]=\bar{\mathcal{T}}^{-1}\boldsymbol{\Sigma}\bar{\mathcal{T}}^{-T}$，故（式 8.362）$\mathbf{T}^{-1}=\exp(\boldsymbol{\xi}'^\wedge)\bar{\mathbf{T}}^{-1}$，$\boldsymbol{\xi}'\sim\mathcal{N}(\mathbf{0},\bar{\mathcal{T}}^{-1}\boldsymbol{\Sigma}\bar{\mathcal{T}}^{-T})$（**无近似**）。

### §8.3.5（2ed）相关位姿的复合与差分

独立时（式 8.363,8.364）$\mathbf{T}=\mathbf{T}_1\mathbf{T}_2$，$\boldsymbol{\Sigma}\approx\boldsymbol{\Sigma}_1+\bar{\mathcal{T}}_1\boldsymbol{\Sigma}_2\bar{\mathcal{T}}_1^T$（二阶）。相关（Mangelson et al. 2020，式 8.365）$\begin{bmatrix}\boldsymbol{\xi}_1\\\boldsymbol{\xi}_2\end{bmatrix}\sim\mathcal{N}(\mathbf{0},\begin{bmatrix}\boldsymbol{\Sigma}_1&\boldsymbol{\Sigma}_{12}\\\boldsymbol{\Sigma}_{12}^T&\boldsymbol{\Sigma}_2\end{bmatrix})$。复合协方差（式 8.366）：
$$\boldsymbol{\Sigma}\approx\boldsymbol{\Sigma}_1+\bar{\mathcal{T}}_1\boldsymbol{\Sigma}_2\bar{\mathcal{T}}_1^T+\boldsymbol{\Sigma}_{12}\bar{\mathcal{T}}_1^T+\bar{\mathcal{T}}_1\boldsymbol{\Sigma}_{12}^T\quad(\text{二阶}).$$
差分 $\mathbf{T}=\mathbf{T}_1\mathbf{T}_2^{-1}$（式 8.367,8.368）：
$$\boldsymbol{\Sigma}\approx\boldsymbol{\Sigma}_1+\bar{\mathcal{T}}\boldsymbol{\Sigma}_2\bar{\mathcal{T}}^T-\boldsymbol{\Sigma}_{12}\bar{\mathcal{T}}^T-\bar{\mathcal{T}}\boldsymbol{\Sigma}_{12}^T,\quad \mathcal{T}=\mathrm{Ad}(\bar{\mathbf{T}}_1\bar{\mathbf{T}}_2^{-1})\quad(\text{二阶}).$$
（推导留作习题 8.14, 8.15。）
