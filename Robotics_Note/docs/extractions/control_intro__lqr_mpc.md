# 抽取留痕：控制导论 —— LQR 完整推导(Riccati) · MPC 滚动时域优化完整表述

> **本文件性质**：项目内部「抽取留痕」，**非成书正文**。目标是把多个权威源材料【全量保真】地抽取下来（每一步推导/中间代数不跳、每一道例题与数值例、每一条定义/定理/引理/命题 + 完整证明、每一张表/分类/算法伪码），供后续综合 agent 改写成自包含的书章。**禁摘要、禁凝练**。公式一律 LaTeX 写全，标注【源出处】，保留所有式号与条件。
>
> **服务章节**：控制导论。本章【聚焦】须重点覆盖：(a) 反馈与稳定性；(b) PID；(c) 状态空间与 LQR（Riccati 完整推导）；(d) MPC（滚动时域优化）；(e) 面向机器人的控制（逆动力学/WBC 概览）。
>
> **个人笔记状态**：个人源 `SLAM理论.md` 未同步到本环境（见 [[project-relocation]]）。本抽取**主要据权威教材/讲义/官方文档**，凡非源材料直接给出、由抽取专员据控制理论标准结论补全或跨源综合者，标 `\rebuilt`。机器人控制（计算力矩/WBC）一节因主要服务"概览"且个人笔记缺失，整体标 `\rebuilt` 并注明权威出处。

---

## 源材料清单与出处

| 代号 | 文献 | 出处（标题 + URL/编号） | 在本抽取中承担 |
|---|---|---|---|
| **[Bansal]** | Somil Bansal, *EE 221A: Special Lecture on the Linear Quadratic Regulator* (UC Berkeley 讲义) | `https://smlbansal.github.io/Papers/lqrlecture.pdf` | **离散时间 LQR 经由动态规划的完整归纳证明**（定理 1 + 证明，逐步代数）：值函数二次型断言、Bellman 递推、梯度置零、Riccati 递推、无穷时域 ARE、仿射系统扩展。本讲义自陈基于 [Abbeel],[Boyd EE363],[Lancaster-Rodman],[Tomlin]。 |
| **[RMD]** | J. B. Rawlings, D. Q. Mayne, M. M. Diehl, *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill Publishing, 2020 | PDF（2nd ed. 1st printing）`https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-1st-printing.pdf`；亦有 2nd printing 镜像 | **MPC 权威主源**：§1.3（LQ 问题 + DP + 无穷时域 + 可控性 + LQR 收敛证明 Lemma 1.3 + DARE）；§2.2（非线性时不变 MPC 完整表述 P_N(x)、值函数、滚动时域律）；§2.3（DP 解）；§2.4（稳定性理论：Lyapunov 定义、基本稳定性假设 2.14、单调性 Prop 2.18、下降性、主定理 2.19、指数稳定）；§8.8（数值：LQP 的 Riccati 递推 (8.51)-(8.54)、condensing/凝聚 (8.55)、稀疏 KKT）；Example 1.1（二次型求和引理）；Example 2.5/2.6（线性二次 MPC 数值例）；Kalman 反例。 |
| **[Tedrake-LQR]** | Russ Tedrake, *Underactuated Robotics*, Ch. "Linear Quadratic Regulators" (LQR) | `https://underactuated.mit.edu/lqr.html` | **连续时间 LQR 经 HJB 的推导**：cost-to-go 二次型 ansatz $J^*=x^\top S x$、HJB、最优 $u^*=-R^{-1}B^\top S x$、连续微分 Riccati 方程 (CDRE)、终端条件、无穷时域 CARE、面向非线性系统的线性化/轨迹 LQR（时变）。 |
| **[Wiki-LQR]** | Wikipedia, *Linear–quadratic regulator* | `https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator` | **含交叉项 $N$ 的四种标准 LQR 公式速查**（连续/离散 × 有限/无穷时域），增益 $K$、Riccati（微分/代数）方程的统一记号版。 |
| **[Wiki-ARE]** | Wikipedia, *Algebraic Riccati equation* | `https://en.wikipedia.org/wiki/Algebraic_Riccati_equation` | **CARE/DARE 的 Hamiltonian/辛矩阵求解法**（稳定不变子空间 $P=U_{21}U_{11}^{-1}$）、迭代法、存在唯一性（可镇定/可检测）。 |
| **[Wiki-MPC]** | Wikipedia, *Model predictive control* | `https://en.wikipedia.org/wiki/Model_predictive_control` | **MPC 工程视角**：滚动/后退时域原理、线性 MPC 的跟踪型 QP 代价（$w_x,w_u,\Delta u$）、约束分类、线性/非线性 MPC 区别、与 LQR 的对比。 |
| **[Wiki-CTC]** | Wikipedia, *Computed torque control* | `https://en.wikipedia.org/wiki/Computed_torque_control` | **计算力矩/逆动力学控制律**（含积分项）、闭环误差动力学、反馈线性化结构。`\rebuilt` 服务机器人控制概览。 |
| **[WBC-refs]** | Sentis & Khatib（OSC/WBC）、Kuindersma et al.（Atlas QP-WBC）等综述/论文 | 见 §9 各条 URL | **整体控制 WBC 概览**：浮动基动力学、优先级任务、QP-WBC 标准形。`\rebuilt`。 |

> **抽取深度声明**：[Bansal] 全文（式 1–13 + 定理 1 完整证明）；[RMD] §1.3.1–1.3.6、§2.2、§2.4.1–2.4.4、§8.8.1–8.8.4、Ex 1.1、Ex 2.5/2.6 逐式；[Tedrake-LQR] LQR 章全部方程；Wikipedia 三页核心公式全部。MPC 书有 OCR 转写瑕疵（如 $P_f$、撇号代转置、$\Pi$/$P$ 混排），本抽取已据上下文恢复为规范 LaTeX，凡恢复处保留原式号便于回溯。

---

## §0 记号约定（各源）与本书统一约定的差异（务必先读）

> 控制理论文献的记号比 SLAM 更统一，但仍有若干分裂点：**Riccati 解矩阵**记 $P$（Boyd/Bansal/RMD 离散）还是 $S$（Tedrake/连续 HJB）；**代价权重终端项**记 $Q_f$（Bansal/RMD）、$P_f$（RMD §1.3 正文）、$F$（Wiki 连续）、$Q_{H_p}$（Wiki 离散）；**反馈律符号** $u=-Kx$（Boyd/Tedrake/Wiki）还是把符号吸入 $K$（Bansal 写 $u=-K_t z$、$K$ 不含负号）；**代价是否带 $\tfrac12$**（RMD/Tedrake 带 $\tfrac12$，Bansal/Wiki 不带）。综合时须统一。

### 0.1 总览对照表

| 项目 | [Bansal]（离散 DP） | [RMD]（MPC 书） | [Tedrake-LQR]（连续 HJB） | [Wiki-LQR/ARE] | **本书统一约定（建议）** | 差异 / 转换 |
|---|---|---|---|---|---|---|
| 状态/输入 | $x_t,u_t$（离散，$t$） | $x,u$，后继态 $x^+$；离散 $x_k,u_k$ | $\mathbf x,\mathbf u$（连续） | $x,u$ | $\mathbf x\in\mathbb R^n,\ \mathbf u\in\mathbb R^m$ | 一致。 |
| 连续动力学 | — | $\dot x=f(x,u)$；线性 $\dot x=Ax+Bu$ | $\dot{\mathbf x}=\mathbf A\mathbf x+\mathbf B\mathbf u$ | $\dot x=Ax+Bu$ | $\dot{\mathbf x}=\mathbf A\mathbf x+\mathbf B\mathbf u$ | 一致。 |
| 离散动力学 | $x_{t+1}=Ax_t+Bu_t$ | $x^+=Ax+Bu$，$x_{k+1}=Ax_k+Bu_k$ | $\mathbf x[n{+}1]=\mathbf A\mathbf x[n]+\mathbf B\mathbf u[n]$ | $x_{k+1}=Ax_k+Bu_k$ | $\mathbf x_{k+1}=\mathbf A\mathbf x_k+\mathbf B\mathbf u_k$ | 一致。 |
| Riccati 解矩阵 | $P_t$（离散，逐步） | $\Pi_k$ 或 $P$（§1.3 用 $\Pi$；§8.8 用 $P_i$） | $\mathbf S(t)$（连续），$\mathbf S$（稳态） | $P$（离散与连续皆 $P$） | **离散 $\mathbf P_k$、连续 $\mathbf S(t)$**（沿用各自传统，正文加一句"$\mathbf S$ 即连续版 $\mathbf P$"） | Tedrake $S\leftrightarrow$ Boyd/RMD $P$，同一对象（最优 cost-to-go 的二次型核）。 |
| 代价是否带 $\tfrac12$ | **不带**：$\sum x^\top Qx+u^\top Ru$ | **带**：$\sum\tfrac12(x^\top Qx+u^\top Ru)$ | **带**：$\ell=\mathbf x^\top\mathbf Q\mathbf x+\mathbf u^\top\mathbf R\mathbf u$（注：Tedrake 此处**不带** $\tfrac12$） | 不带 | **建议正文统一"不带 $\tfrac12$"**（与 Bansal/Wiki/Tedrake 一致；RMD 带 $\tfrac12$ 仅整体缩放，不影响最优解与 $K$，只把 $\mathbf P$ 整体缩放 $\tfrac12$） | $\tfrac12$ 只缩放代价与 $\mathbf P$，**不改变** $K$、不改变 Riccati 方程的解结构。抄录 RMD 公式时其 $\Pi$ 已含 $\tfrac12$ 约定。 |
| 终端权重记号 | $Q_f$ | $P_f$（§1.3 正文）；$V_f,\ \mathbf P_N$（终端代价/递推初值） | $\mathbf Q_f$ | $F$（连续）/$Q_{H_p}$（离散） | **$\mathbf Q_f$**（终端代价权重）；递推终端值 $\mathbf P_N=\mathbf Q_f$ | 同一物理量；MPC 中 $V_f(x)=\tfrac12 x^\top P_f x$ 是"终端代价函数"。 |
| 反馈律符号 | $u_t=-K_t z$，$K_t=(R+B^\top P_{t+1}B)^{-1}B^\top P_{t+1}A$（**$K$ 不含负号**） | $u^0(x)=K_k x$（**RMD 的 $K_k$ 已含负号**，$K_k=-(B^\top\Pi B+R)^{-1}B^\top\Pi A$） | $\mathbf u^*=-\mathbf K\mathbf x$，$\mathbf K=\mathbf R^{-1}\mathbf B^\top\mathbf S$（**$K$ 不含负号**） | $u=-Kx$，$K=(R+B^\top PB)^{-1}(B^\top PA+N^\top)$ | **$\mathbf u=-\mathbf K\mathbf x$，$\mathbf K$ 不含负号**（与 Bansal/Tedrake/Wiki 一致） | **RMD 的 $K_k$ 含负号**！抄 RMD 时注意 $\mathbf K_{\text{RMD}}=-\mathbf K_{\text{本书}}$。 |
| 交叉项 | 无（$N=0$） | §8.8 有 $S_i$（状态-输入耦合，记 $\mathbf S$） | 无 | 有，记 $N$（$2x^\top Nu$） | 默认 $\mathbf N=0$；需要时记 $\mathbf N$（耦合权重） | 三处交叉项符号 $N$/$S$ 撞名（$N$ 又是时域长度！）。**本书交叉项建议记 $\mathbf S_{xu}$ 以免与时域 $N$ 冲突**。 |
| 时域长度 | $N$ | $N$（控制时域） | $N$（步数） | $N$（预测时域）/$H_p,H_u$ | **预测时域 $N$（或 $H_p$）、控制时域 $H_u$** | 一致语义。 |
| 权重正定性 | $Q,R,Q_f\succeq0$（半正定；R 实际须 $\succ0$） | $Q,P_f\succeq0$，$R\succ0$ | $\mathbf Q\succeq0,\mathbf R\succ0$ | $Q\succeq0,R\succ0$ | $\mathbf Q\succeq0,\ \mathbf R\succ0,\ \mathbf Q_f\succeq0$ | 一致；$\mathbf R\succ0$ 保唯一性。 |
| 值函数/代价记号 | $J_t^*(x)$（cost-to-go） | $V_N^0(x)$（最优值函数）、$V_N(x,\mathbf u)$（代价） | $J^*(\mathbf x)$、$J^*(\mathbf x,t)$ | $J$ | **$V_N^0(\mathbf x)$（最优值/cost-to-go）、$J$（总代价）** | RMD 上标 $0$ 表"最优"，下标 $N$ 表时域。Bansal/Tedrake 用 $J^*$。 |

> **本书与 SLAM 主线约定的衔接**：本章不涉及 $SO(3)$/四元数/扰动方向（那是状态估计层）。但**机器人控制（§9）会用到 $\mathbf R\in SO(3)$、角速度、雅可比**，届时沿用本书 $\mathbf R_{wb}$（body→world）、右扰动约定。LQR/MPC 主体是 $\mathbb R^n$ 线性系统，无 Lie 群差异。

### 0.2 三处"$N$"撞名警示（综合 agent 必读）
控制文献中字母 $N$ 同时被用作：(1) **时域长度**（horizon，Bansal/RMD/Tedrake/Wiki 主用法）；(2) **LQR 交叉项权重**（Wiki-LQR 的 $2x^\top Nu$）；(3) **凝聚 QP 的预测矩阵之一**（[RMD] §8.8.4 用 $N_x$ 等）。本抽取在引用 Wiki 交叉项时保留 $N$ 但**显式注明"此 $N$ 是交叉权重非时域"**；综合到本书强烈建议交叉项改记 $\mathbf S_{xu}$。

---

# 第一部分：背景 —— 反馈、稳定性与 PID

> 本部分为本章 (a)(b) 聚焦点提供自包含基础。源材料主要为标准控制结论，凡非上列权威源逐字给出者标 `\rebuilt`（据控制理论标准教材通识，如 Åström-Murray *Feedback Systems*、Ogata *Modern Control Engineering* 之通用结论）。

## §B.1 状态空间模型与稳定性判据 `\rebuilt`

**连续线性时不变 (LTI) 系统**：
$$\dot{\mathbf x}(t)=\mathbf A\mathbf x(t)+\mathbf B\mathbf u(t),\qquad \mathbf y(t)=\mathbf C\mathbf x(t)+\mathbf D\mathbf u(t),$$
$\mathbf x\in\mathbb R^n$ 状态、$\mathbf u\in\mathbb R^m$ 输入、$\mathbf y\in\mathbb R^p$ 输出。**离散 LTI**：$\mathbf x_{k+1}=\mathbf A\mathbf x_k+\mathbf B\mathbf u_k$。

**内部稳定性（零输入）**：
- 连续 $\dot{\mathbf x}=\mathbf A\mathbf x$ **渐近稳定** $\iff$ $\mathbf A$ 的所有特征值 $\lambda_i$ 满足 $\operatorname{Re}\lambda_i<0$（$\mathbf A$ 为 Hurwitz）。
- 离散 $\mathbf x_{k+1}=\mathbf A\mathbf x_k$ **渐近稳定** $\iff$ $\mathbf A$ 的所有特征值满足 $|\lambda_i|<1$（谱半径 $\rho(\mathbf A)<1$，$\mathbf A$ 为 Schur 稳定）。

**Lyapunov 方程（线性系统的 Lyapunov 函数）**：取 $V(\mathbf x)=\mathbf x^\top\mathbf P\mathbf x$，$\mathbf P\succ0$。
- 连续：$\dot V=\mathbf x^\top(\mathbf A^\top\mathbf P+\mathbf P\mathbf A)\mathbf x$；若 $\exists\,\mathbf P\succ0$ 使 $\mathbf A^\top\mathbf P+\mathbf P\mathbf A=-\mathbf Q\prec0$，则系统渐近稳定。
- 离散：$\Delta V=\mathbf x^\top(\mathbf A^\top\mathbf P\mathbf A-\mathbf P)\mathbf x$；若 $\exists\,\mathbf P\succ0$ 使 $\mathbf A^\top\mathbf P\mathbf A-\mathbf P=-\mathbf Q\prec0$，则系统渐近稳定。

> **与 LQR/MPC 的钩子**：LQR 的最优代价 $V^*(\mathbf x)=\mathbf x^\top\mathbf P\mathbf x$ 恰是闭环系统的 Lyapunov 函数（见 §3.6、§6.x，[RMD] §1.3.6 末与 §2.4）；这是"最优即稳定（在无穷时域下）"的本质。

## §B.2 可控性 / 可镇定性 / 可观性 / 可检测性 [RMD §1.3.5；Hautus 引理逐字]

> 以下 [RMD] §1.3.5 逐字（OCR 已恢复）。

**可控性（controllable）定义**：系统 $x^+=Ax+Bu$ 可控，若存在有限时间 $N$ 与输入序列 $u_0,u_1,\dots,u_{N-1}$ 能把系统从任意 $x$ 转移到任意 $z$，其中
$$z=A^Nx+\begin{bmatrix}B&AB&\cdots&A^{N-1}B\end{bmatrix}\begin{bmatrix}u_{N-1}\\u_{N-2}\\\vdots\\u_0\end{bmatrix}.$$
由 Cayley–Hamilton 定理，$A^k\ (k\ge n)$ 可由 $A^0,\dots,A^{n-1}$ 线性表出，故 $\begin{bmatrix}B&AB&\cdots&A^{N-1}B\end{bmatrix}$（$N\ge n$）的值域与 $\begin{bmatrix}B&AB&\cdots&A^{n-1}B\end{bmatrix}$ 相同。**即对无约束线性系统，若 $n$ 步内不可达 $z$，则任意步数皆不可达。** 可控性 $\iff$ 线性方程组
$$\begin{bmatrix}B&AB&\cdots&A^{n-1}B\end{bmatrix}\begin{bmatrix}u_{n-1}\\\vdots\\u_0\end{bmatrix}=z-A^nx$$
对任意右端有解。其中
$$\boxed{\ \mathcal C=\begin{bmatrix}B&AB&\cdots&A^{n-1}B\end{bmatrix}\ }\tag{1.16}$$
称为**可控性矩阵**（$n\times nm$）。由线性代数基本定理，对任意右端有解 $\iff$ $\mathcal C$ 的行线性无关。故
$$\boxed{\ (A,B)\ \text{可控}\iff\operatorname{rank}\mathcal C=n.\ }$$

**Hautus 可控性判据（Lemma 1.2，[RMD] 引 Hautus 1972）**：系统可控 $\iff$
$$\operatorname{rank}\begin{bmatrix}\lambda I-A&B\end{bmatrix}=n\qquad\text{对所有}\ \lambda\in\mathbb C.\tag{1.17}$$
注：当 $\lambda$ 非 $A$ 的特征值时前 $n$ 列已满秩，故 (1.17) 等价于仅在 $A$ 的特征值处检验：
$$\operatorname{rank}\begin{bmatrix}\lambda I-A&B\end{bmatrix}=n\qquad\text{对所有}\ \lambda\in\operatorname{eig}(A).$$

**可镇定性 / 可检测性（[RMD] §1.3.6 末 + Lemma 1.12/1.13 表）`\rebuilt`（综合 Hautus 形式）**：
- **可镇定（stabilizable）**：仅要求**不稳定**模态可控，即 (1.17) 仅需对 $|\lambda|\ge1$（离散）或 $\operatorname{Re}\lambda\ge0$（连续）的 $\lambda$ 成立。
- **可观性（observable）**：$(A,C)$ 可观 $\iff$ 观测矩阵 $\mathcal O=[C;\,CA;\,\cdots;\,CA^{n-1}]$ 满列秩 $n$；Hautus：$\operatorname{rank}\big[\begin{smallmatrix}\lambda I-A\\C\end{smallmatrix}\big]=n,\ \forall\lambda$。
- **可检测（detectable）**：仅要求不稳定模态可观（对偶于可镇定）。

> **LQR 适用条件（[RMD] §1.3.6 总结）**：基本结论要求 $(A,B)$ **可控**且 $Q,R\succ0$；可放宽为 $(A,B)$ **可镇定** + $Q\succeq0$ 且 $(A,Q^{1/2})$（即 $(A,Q)$）**可检测** + $R\succ0$（保唯一）。

## §B.3 PID 控制 `\rebuilt`

> PID 为本章 (b) 聚焦点。个人笔记缺失，以下为标准定义（通识；可对照 Åström-Murray *Feedback Systems* §10、Ogata Ch.8）。

**连续 PID 控制律**（单输入单输出，误差 $e(t)=r(t)-y(t)$）：
$$\boxed{\ u(t)=K_p\,e(t)+K_i\int_0^t e(\tau)\,\mathrm d\tau+K_d\,\frac{\mathrm d e(t)}{\mathrm d t}\ }$$
等价的"积分时间/微分时间"形式：$u=K_p\big(e+\frac1{T_i}\int_0^t e\,\mathrm d\tau+T_d\,\dot e\big)$，其中 $T_i=K_p/K_i$（积分时间）、$T_d=K_d/K_p$（微分时间）。

**传递函数**：$C(s)=\dfrac{U(s)}{E(s)}=K_p+\dfrac{K_i}{s}+K_d s=K_p\Big(1+\dfrac1{T_i s}+T_d s\Big).$

**各项作用（标准结论）**：
- **比例 P**：$\propto$ 当前误差；增大 $K_p$ 提升响应速度、减小稳态误差，但过大致振荡/失稳。单纯 P 对 1 型以下系统有稳态误差。
- **积分 I**：$\propto$ 误差累积；消除**稳态误差**（对阶跃参考），但引入相位滞后、易致超调/积分饱和 (integral windup)。
- **微分 D**：$\propto$ 误差变化率；提供**预测/阻尼**，抑制超调、改善稳定裕度，但放大高频噪声（实际用带限微分 $\frac{K_d s}{1+s/N_d}$）。

**离散 PID（位置式）`\rebuilt`**：采样周期 $T_s$，
$$u_k=K_p e_k+K_i T_s\sum_{j=0}^k e_j+\frac{K_d}{T_s}(e_k-e_{k-1}).$$
**增量式**：$\Delta u_k=u_k-u_{k-1}=K_p(e_k-e_{k-1})+K_iT_s e_k+\frac{K_d}{T_s}(e_k-2e_{k-1}+e_{k-2})$（天然抗积分饱和、便于无扰切换）。

**整定（Ziegler–Nichols 临界比例度法，经典经验法）`\rebuilt`**：先纯 P 增益增至等幅振荡，记临界增益 $K_u$、振荡周期 $T_u$，则
| 控制器 | $K_p$ | $T_i$ | $T_d$ |
|---|---|---|---|
| P | $0.5K_u$ | — | — |
| PI | $0.45K_u$ | $T_u/1.2$ | — |
| PID | $0.6K_u$ | $T_u/2$ | $T_u/8$ |

> **与 LQR 的关系（综合时写盒）**：PID 是**输出反馈 + 固定结构**控制器（手工整定）；LQR 是**全状态反馈 + 最优**控制器（由 $Q,R$ 与 Riccati 自动给出 $\mathbf K$）。计算力矩控制（§9）的外环常用 PD/PID（见 §9.2 闭环误差动力学 $\ddot e+K_d\dot e+K_p e=0$）。

---

# 第二部分：状态空间最优控制 —— LQR 与 Riccati 完整推导

## §1 离散时间有限时域 LQR 经动态规划（[Bansal] 全文，含完整归纳证明）

> 本节为离散 LQR 的**复现级完整推导**，逐式逐步，**最重要**。源 [Bansal] §1–§2 与定理 1 证明。注意 [Bansal] 代价**不带 $\tfrac12$**、反馈增益 $K$ **不含负号**（$u=-Kz$）。

### §1.1 问题设置（[Bansal] 式 1–3）

**离散 LTI 系统**：
$$x_{t+1}=Ax_t+Bu_t,\quad t\in\{0,1,\dots,N\},\qquad x_0=x_{\text{init}}.\tag{1}$$
**二次代价泛函**：
$$J(U,x_0)=\sum_{\tau=0}^{N-1}\big(x_\tau^\top Q x_\tau+u_\tau^\top R u_\tau\big)+x_N^\top Q_f x_N,\tag{2}$$
其中 $Q,R,Q_f\succeq0$（半正定；实际 $R\succ0$ 保唯一），$U:=(u_0,u_1,\dots,u_{N-1})$。各项含义：
- $N$ 为**时域**（后取 $N=\infty$）；
- $x_t^\top Q x_t$ 度量 $\tau=t$ 处的**状态偏差代价**；
- $u_t^\top R u_t$ 度量 $\tau=t$ 处的**输入权限代价**；
- $x_N^\top Q_f x_N$ 度量**终端代价**。

**优化问题**：
$$\min_U\{J(U,x_0)\ \text{subject to}\ (1)\}.\tag{3}$$
本讲义用**动态规划 (DP)**（借助最优性原理）求解 (3)。

### §1.2 DP 求解：嵌入与 Bellman 递推（[Bansal] 式 4–5）

把问题嵌入更大类：从初始时刻 $t$ 起、在离散区间 $\{t:N\}$（$=\{t,t+1,\dots,N\}$ 的简记）上的代价：
$$J(U_t,x_t)=\sum_{\tau=t}^{N-1}\big(x_\tau^\top Q x_\tau+u_\tau^\top R u_\tau\big)+x_N^\top Q_f x_N,\tag{4}$$
其中 $U_t=(u_t,u_{t+1},\dots,u_{N-1})$。

**Bellman 最优性原理（原文）**：若已在区间 $\{0:N\}$ 上求得最优轨迹，则该轨迹在所有形如 $\{t:N\}\,(t>0)$ 的子区间上也最优，前提是 $t$ 时刻初值 $x_t$ 由从时刻 0 起沿最优轨迹前向运行得到。$J(U_t,x_t)$ 的最优值称为从状态 $x_t$、时刻 $t$ 起的"**cost-to-go**（待付代价）"，记 $J_t^*(x_t)$。目标是求 $J_0^*(x_0)$。

**递推构造**：设 $t$ 时刻已知对所有状态 $z$ 的 $J_{t+1}^*(z)$，求最优 $u_t$（从而 $J_t^*(x_t)$）。$u_t$ 影响：(i) 当前代价（经 $u_t^\top R u_t$）；(ii) 下一状态 $x_{t+1}$（从而从 $x_{t+1}$ 的最小 cost-to-go）。由最优性原理，最优 $u_t$ 是下式之解：
$$\boxed{\ J_t^*(x_t)=\min_{u_t}\Big[x_t^\top Q x_t+u_t^\top R u_t+J_{t+1}^*(Ax_t+Bu_t)\Big].\ }\tag{5}$$
其中 $x_t^\top Q x_t+u_t^\top R u_t$ 是 $t$ 时刻发生的代价，$J_{t+1}^*(Ax_t+Bu_t)$ 是在 $t+1$ 落点处的最小 cost-to-go。从 $t=N$ 反向求解 (5) 即得最优控制序列。

### §1.3 定理 1（最优 cost-to-go 与最优控制）及**完整归纳证明**（[Bansal] 定理 1）

> **定理 1（[Bansal]）.** 时刻 $t$ 的最优 cost-to-go 与最优控制为
> $$J_t^*(z)=z^\top P_t z,\qquad u_t^*=-K_t z,\tag{6}$$
> 其中 $t\in\{0,1,\dots,N-1\}$，且
> $$P_t=Q+K_t^\top R K_t+(A-BK_t)^\top P_{t+1}(A-BK_t),\qquad P_N=Q_f,\tag{7a}$$
> $$K_t=(R+B^\top P_{t+1}B)^{-1}B^\top P_{t+1}A.\tag{7b}$$

**证明（归纳，[Bansal] 逐步，全量保真）.**

*基例 $i=N$*：由 (4)，$i=N$ 时 LQR 代价与控制无关，最优 cost-to-go 平凡地为 $z^\top Q_f z$。故 $P_N=Q_f$。 ✓

*归纳步*：设定理对 $i=k$ 成立，证 $i=k-1$。由 (5)：
$$J_{k-1}^*(z)=\min_{u_{k-1}}\Big[z^\top Q z+u_{k-1}^\top R u_{k-1}+J_k^*(Az+Bu_{k-1})\Big].$$
由归纳假设 $J_k^*(x)=x^\top P_k x$，故
$$J_{k-1}^*(z)=\min_{u_{k-1}}\Big[z^\top Q z+u_{k-1}^\top R u_{k-1}+(Az+Bu_{k-1})^\top P_k(Az+Bu_{k-1})\Big].\tag{8}$$
**最优 $u_{k-1}$ 由 RHS 对 $u_{k-1}$ 求导置零得到**（梯度，逐项）：
$$2u_{k-1}^\top R+2(Az+Bu_{k-1})^\top P_k B=0.$$
（说明：$\partial(u^\top R u)/\partial u=2u^\top R$；$\partial\big[(Az+Bu)^\top P_k(Az+Bu)\big]/\partial u=2(Az+Bu)^\top P_k B$，用 $P_k=P_k^\top$。）转置并解出：
$$Ru_{k-1}+B^\top P_k(Az+Bu_{k-1})=0\ \Rightarrow\ (R+B^\top P_kB)u_{k-1}=-B^\top P_kAz,$$
$$\boxed{\ u_{k-1}^*=-(R+B^\top P_kB)^{-1}B^\top P_kAz=-K_{k-1}z.\ }\tag{9}$$
**最优 cost-to-go** 由把 (9) 代回 (8)：
$$J_{k-1}^*(z)=z^\top Q z+u_{k-1}^{*\top}Ru_{k-1}^*+(Az+Bu_{k-1}^*)^\top P_k(Az+Bu_{k-1}^*)\tag{10}$$
$$=z^\top\Big[Q+K_{k-1}^\top R K_{k-1}+(A-BK_{k-1})^\top P_k(A-BK_{k-1})\Big]z$$
$$=z^\top P_{k-1}z.$$
（中间：把 $u_{k-1}^*=-K_{k-1}z$ 代入，$Az+Bu_{k-1}^*=(A-BK_{k-1})z$；$u_{k-1}^{*\top}Ru_{k-1}^*=z^\top K_{k-1}^\top R K_{k-1}z$。）即定理对 $k-1$ 成立。$\square$

> **Remark 1（[Bansal]）**：最优控制是状态的**线性函数**（称**线性状态反馈**）。开环动力学被改成闭环 $x_{t+1}=Ax_t+Bu_t$ 经时变增益块 $K_t$ 反馈。最优 cost-to-go 是状态的二次函数。
>
> **Remark 2（[Bansal]）**：$P_t,K_t$ 可从 $t=N-1$ 起**反向递推**计算。

> **(7a) 与"标准 Riccati"形式的等价 `\rebuilt`（综合时用，证非凭记忆）**：把 (7b) 代入 (7a) 的"Joseph 稳定化形式"，经标准代数化简（即 §3.x 与 [RMD] 式 1.10 一致）得**紧凑 Riccati 递推**：
> $$P_t=Q+A^\top P_{t+1}A-A^\top P_{t+1}B(R+B^\top P_{t+1}B)^{-1}B^\top P_{t+1}A.$$
> 两形式代数恒等：(7a) 是"$(A-BK)^\top P(A-BK)+K^\top RK$"展开，紧凑式是其消去 $K$ 后的结果。Joseph 形式 (7a) **数值上更稳**（保对称半正定），紧凑式更省算。

### §1.4 闭环结构图（[Bansal] Figure 1，文字复述）
当前状态 $x_t$ 被测量，经时变增益块 $K_t$ 反馈进系统：$u_t=-K_tx_t\to x_{t+1}=Ax_t+Bu_t$。最优 cost-to-go 在最优策略下是状态的二次函数。

## §2 离散时间 LQR 的扩展（[Bansal] §3）

### §2.1 无穷时域 LQR 与离散代数 Riccati 方程 DARE（[Bansal] 式 11–12）

$N=\infty$ 时仍可用 DP，但递推 (7) 达到**稳态解**：
$$P_{ss}=Q+K_{ss}^\top R K_{ss}+(A-BK_{ss})^\top P_{ss}(A-BK_{ss}),\tag{11a}$$
$$K_{ss}=(R+B^\top P_{ss}B)^{-1}B^\top P_{ss}A.\tag{11b}$$
等价地可写为
$$\boxed{\ P_{ss}=Q+A^\top P_{ss}A-A^\top P_{ss}B(R+B^\top P_{ss}B)^{-1}B^\top P_{ss}A,\ }\tag{12}$$
称为**（离散时间）代数 Riccati 方程 (DARE)**。$P_{ss}$ 可由迭代 Riccati 递推或直接法求出。此时最优输入是**线性常增益状态反馈**：
$$u_t=-K_{ss}x_t,\qquad K_{ss}=(R+B^\top P_{ss}B)^{-1}B^\top P_{ss}A.$$

### §2.2 状态仿射系统（[Bansal] §3.2，式 13）

设动力学含常数项：$x_{t+1}=Ax_t+Bu_t+c$，$x_0=x_{\text{init}}$。最优 LQR 策略仍线性、最优 cost-to-go 仍二次。**技巧：增广状态** $z_t:=[x_t;\,1]$，则
$$z_{t+1}=\begin{bmatrix}x_{t+1}\\1\end{bmatrix}=\begin{bmatrix}A&c\\0&1\end{bmatrix}\begin{bmatrix}x_t\\1\end{bmatrix}+\begin{bmatrix}B\\0\end{bmatrix}u_t=A'z_t+B'u_t,\tag{13}$$
即标准 LQR 形式。最优策略 $u_t^*=-K_t z_t$。

> **[Bansal] 注**：LQR 易扩展到**时变系统、轨迹跟踪**等（见 §4 时变 LQR、§9 机器人控制）。本讲义 LQR 处理基于 [Abbeel 高级机器人课]、[Boyd EE363]、[Lancaster–Rodman *Algebraic Riccati Equations* 1995]、[Tomlin EE291E]。

## §3 离散 LQR 收敛性与稳定性（[RMD] §1.3.3–1.3.6，含 Kalman 反例与 Lemma 1.3 完整证明）

> 本节是离散 LQR 的**稳定性权威主源**。[RMD] §1.3 用 $\Pi$ 记 Riccati 矩阵（OCR 已恢复）、代价带 $\tfrac12$、$K_k$ **含负号**。下文保留 RMD 式号 (1.5)–(1.18)。

### §3.1 LQ 问题（[RMD] §1.3.1，式 1.5–1.6）

系统模型 $x^+=Ax+Bu,\ y=Cx$（先设 $C=I$，全状态可测）。目标函数（[RMD] 带 $\tfrac12$）：
$$V(x_0,\mathbf u)=\frac12\sum_{k=0}^{N-1}\big(x_k^\top Q x_k+u_k^\top R u_k\big)+\frac12 x_N^\top P_f x_N\quad\text{s.t.}\ x^+=Ax+Bu,\tag{1.5}$$
$\mathbf u=(u_0,u_1,\dots,u_{N-1})$。最优 LQ 控制问题：
$$\min_{\mathbf u}V(x_0,\mathbf u).\tag{1.6}$$
**假设**：$Q,P_f,R$ 实对称；$Q,P_f$ **半正定**；$R$ **正定**。这些保证最优控制问题解存在且唯一。

### §3.2 多阶段优化与"二次型求和引理"（[RMD] §1.3.2，Example 1.1）

**多阶段结构**：目标 $f(w,x)+g(x,y)+h(y,z)$ 各阶段只依赖相邻变量对。**反向 DP**：先对 $z$ 优化（$w$ 固定）：
$$h^0(y)=\min_z h(y,z),\quad z^0(y)=\arg\min_z h(y,z),$$
再对 $y$：$g^0(x)=\min_y\{g(x,y)+h^0(y)\}$，$y^0(x)=\arg\min_y\{\cdots\}$；最后对 $x$：$f^0(w)=\min_x\{f(w,x)+g^0(x)\}$，$x^0(w)=\arg\min_x\{\cdots\}$。归并：
$$\min_x\Big[f(w,x)+\underbrace{\min_y\big[g(x,y)+\underbrace{\min_z h(y,z)}_{h^0(y),\,z^0(y)}\big]}_{g^0(x),\,y^0(x)}\Big].$$
这是 Bellman 的**反向 DP**（变量逆序求出，得到的是"作为下一阶段待优化变量的函数"）。**前向 DP**（$z$ 固定、$w$ 待优化，[RMD] 式 1.7）用于状态估计。

> **Example 1.1（二次型求和，[RMD]，LQ 求解的核心引理，逐式）.** 设 $V_1(x)=\tfrac12(x-a)^\top A(x-a)$，$V_2(x)=\tfrac12(x-b)^\top B(x-b)$，$A,B\succ0$。
> **(a)** 和 $V(x)=V_1(x)+V_2(x)$ 仍二次：
> $$V(x)=\tfrac12(x-v)^\top H(x-v)+d,$$
> $$\boxed{\ H=A+B,\quad v=H^{-1}(Aa+Bb),\quad d=(Aa+Bb)^\top H^{-1}(Aa+Bb)\ \text{(写法待校)};\ }$$
> [RMD] 给的 $d$ 闭式为 $d=-\,(Aa+Bb)^\top H^{-1}(Aa+Bb)+a^\top Aa+b^\top Bb$（即 $d=v^\top Hv$ 形式经化简）。$H\succ0$（因 $A,B\succ0$）。
> **推导（[RMD]）**：展开两边 $x^\top Hx-2x^\top Hv+v^\top Hv+d=x^\top(A+B)x-2x^\top(Aa+Bb)+a^\top Aa+b^\top Bb$，逐阶比较：$H=A+B$；$v=H^{-1}(Aa+Bb)$；$d=v^\top Hv+a^\top Aa+b^\top Bb-?$（[RMD] 写 $d=-(Aa+Bb)^\top H^{-1}(Aa+Bb)+a^\top Aa+b^\top Bb$）。
> **(b)** 推广（$V_2(x)=\tfrac12(Cx-b)^\top B(Cx-b)$，对调控/估计皆有用）：
> $$H=A+C^\top BC,\quad v=H^{-1}(Aa+C^\top Bb),$$
> $$d=-(Aa+C^\top Bb)^\top H^{-1}(Aa+C^\top Bb)+a^\top Aa+b^\top Bb.\tag{1.8}$$
> $H\succ0$（$A\succ0$，$C^\top BC\succeq0$）。
> **(c)** 逆形式（用矩阵求逆引理，估计用）：$\tilde H=A^{-1}-A^{-1}C^\top(CA^{-1}C^\top+B^{-1})^{-1}CA^{-1}$，$v=a+A^{-1}C^\top(CA^{-1}C^\top+B^{-1})^{-1}(b-Ca)$，$V(x)=\tfrac12(x-v)^\top\tilde H^{-1}(x-v)+\text{const}$。

### §3.3 DP 解：反向 Riccati 迭代（[RMD] §1.3.3，式 1.9–1.14）

把 (1.6) 写为
$$V(x_0,\mathbf u)=\sum_{k=0}^{N-1}\ell(x_k,u_k)+\ell_N(x_N)\quad\text{s.t.}\ x^+=Ax+Bu,$$
**阶段代价** $\ell(x,u)=\tfrac12(x^\top Qx+u^\top Ru),\ k=0,\dots,N-1$，**终端阶段代价** $\ell_N(x)=\tfrac12 x^\top P_f x$。$x_0$ 已知，用反向 DP。

**最后一阶段（[RMD] 式 1.9）**：
$$\min_{u_{N-1},x_N}\big[\ell(x_{N-1},u_{N-1})+\ell_N(x_N)\big]\quad\text{s.t.}\ x_N=Ax_{N-1}+Bu_{N-1},$$
$x_{N-1}$ 作参数。代入状态方程消 $x_N$，并用 (1.8) 合并两二次型：
$$\ell(x_{N-1},u_{N-1})+\ell_N(x_N)=\tfrac12\Big(|x_{N-1}|_Q^2+|u_{N-1}|_R^2+|Ax_{N-1}+Bu_{N-1}|_{P_f}^2\Big)=\tfrac12\Big(|x_{N-1}|_Q^2+|u_{N-1}-v|_H^2\Big)+d,$$
其中（对照 (1.8)，取"$a$ 项"为 $u$、$C\to B$、$A\to R$、$B\to P_f$、被加二次中心化）：
$$H=R+B^\top P_f B,$$
$$v=-(B^\top P_f B+R)^{-1}B^\top P_f A\,x_{N-1},$$
$$d=\tfrac12\,x_{N-1}^\top\Big[A^\top P_f A-A^\top P_f B(B^\top P_f B+R)^{-1}B^\top P_f A\Big]x_{N-1}.$$
由此**直接看出**最优输入 $u_{N-1}=v$（线性于 $x_{N-1}$），最优终态也线性于 $x_{N-1}$，最优代价 $\tfrac12|x_{N-1}|_Q^2+d$ 是 $x_{N-1}$ 的二次型。总结：对所有 $x$，
$$u_{N-1}^0(x)=K_{N-1}x,\qquad x_N^0(x)=(A+BK_{N-1})x,\qquad V_{N-1}^0(x)=\tfrac12 x^\top\Pi_{N-1}x,$$
定义
$$\boxed{\ K_{N-1}:=-(B^\top P_f B+R)^{-1}B^\top P_f A,\ }$$
$$\boxed{\ \Pi_{N-1}:=Q+A^\top P_f A-A^\top P_f B(B^\top P_f B+R)^{-1}B^\top P_f A.\ }$$
（注：**RMD 的 $K_k$ 含负号**，与本书 $\mathbf u=-\mathbf K\mathbf x$ 约定相比 $K_{\text{RMD}}=-K_{\text{本书}}$。）

**下一阶段**结构相同，仅改名：
$$u_{N-2}^0(x)=K_{N-2}x,\quad x_{N-1}^0(x)=(A+BK_{N-2})x,\quad V_{N-2}^0(x)=\tfrac12 x^\top\Pi_{N-2}x,$$
$$K_{N-2}=-(B^\top\Pi_{N-1}B+R)^{-1}B^\top\Pi_{N-1}A,$$
$$\Pi_{N-2}=Q+A^\top\Pi_{N-1}A-A^\top\Pi_{N-1}B(B^\top\Pi_{N-1}B+R)^{-1}B^\top\Pi_{N-1}A.$$

**反向 Riccati 迭代（[RMD] 式 1.10–1.14，权威定义）**：
$$\boxed{\ \Pi_{k-1}=Q+A^\top\Pi_kA-A^\top\Pi_kB\big(B^\top\Pi_kB+R\big)^{-1}B^\top\Pi_kA,\quad k=N,N-1,\dots,1,\ }\tag{1.10}$$
终端条件（替代通常初值，因反向运行）
$$\Pi_N=P_f.\tag{1.11}$$
各阶段最优控制策略
$$u_k^0(x)=K_kx,\quad k=N-1,\dots,0,\tag{1.12}$$
$$\boxed{\ K_k=-\big(B^\top\Pi_{k+1}B+R\big)^{-1}B^\top\Pi_{k+1}A,\quad k=N-1,\dots,0,\ }\tag{1.13}$$
从 $k+1$ 到 $N$ 的最优 cost-to-go
$$V_k^0(x)=\tfrac12 x^\top\Pi_kx,\quad k=N,\dots,0.\tag{1.14}$$

### §3.4 无穷时域 LQ：Kalman 的"最优 ≠ 稳定"反例（[RMD] §1.3.4，逐字数值）

> **Kalman (1960b, p.113) 的警示（[RMD] 引原文）**："In the engineering literature it is often assumed (tacitly and incorrectly) that a system with optimal control law is necessarily stable."（工程文献常默认且错误地认为带最优控制律的系统必稳定。）

设用有限时域问题的**首个反馈增益** $K_0$ 作控制律 $u_k=K_0x_k$，则闭环稳定性由 $A+BK_0$ 的特征值决定。**[RMD] 构造的反例**：
$$A=\begin{bmatrix}4/3&-2/3\\1&0\end{bmatrix},\quad B=\begin{bmatrix}1\\0\end{bmatrix},\quad C=\begin{bmatrix}-2/3&1\end{bmatrix}.$$
该系统传递函数 $G(z)$ 在 $z=3/2$ 有**零点**（不稳定零点）。构造一个反演此零点、从而产生不稳定系统的 LQ 控制器：取 $Q=C^\top C$（罚 $y$，但仅半正定），加小正定项使 $Q\succ0$，取**很小**的 $R$（鼓励控制器"乱来"），$N=5$：
$$Q=C^\top C+0.001I=\begin{bmatrix}4/9+0.001&-2/3\\-2/3&1.001\end{bmatrix},\qquad R=0.001.$$
迭代 Riccati 四次（从 $\Pi=Q$ 起，$N=5$）算 $K_0$，再算 $A+BK_0$ 特征值：
$$\operatorname{eig}(A+BK_0^{(5)})=\{1.307,\ 0.001\}.$$
由于一个特征值 $>1$，闭环 $x_k=(A+BK_0^{(5)})^kx_0\to\infty$，**闭环不稳定**。继续迭代（增大时域），$N=7$：
$$\operatorname{eig}(A+BK_0^{(7)})=\{0.989,\ 0.001\},$$
控制器变稳定。继续迭代至收敛（无穷时域）：
$$\operatorname{eig}(A+BK_\infty^0)=\{0.664,\ 0.001\},$$
稳定且有合理稳定裕度。**结论：标称稳定是无穷时域控制器的保证性质**（下节证明）。

由此引出**无穷时域代价**（[RMD] 式 1.15）：
$$V(x_0,\mathbf u)=\frac12\sum_{k=0}^{\infty}\big(x_k^\top Q x_k+u_k^\top R u_k\big),\tag{1.15}$$
$x_k$ 为 $x^+=Ax+Bu$ 在初值 $x_0$、输入序列 $\mathbf u$ 下 $k$ 时刻解。**先限于存在使代价有界的输入序列的系统**：例 $A=I,B=0$ 则 (1.15) 对 $x_0\ne0$ 必无界（不可控系统，$A=I$ 不稳定而 $B=0$ 无输入）。需要可控/可镇定/可观/可检测概念（见 §B.2）。

### §3.5 LQR 收敛性主定理（[RMD] Lemma 1.3，含完整证明）

**无穷时域目标**（$Q,R\succ0$）：
$$V(x,\mathbf u)=\frac12\sum_{k=0}^\infty\big(x_k^\top Q x_k+u_k^\top R u_k\big)\quad\text{s.t.}\ x^+=Ax+Bu,\ x_0=x.$$
若 $(A,B)$ 可控，则 $\min_{\mathbf u}V(x,\mathbf u)$ 对所有 $x$ 存在且唯一。记最优解 $\mathbf u^0$、其首元 $u^0(x)$；**反馈控制律** $\kappa_\infty(x):=u^0(0;x)$，即 $u=\kappa_\infty(x)$。

> **Lemma 1.3（LQR 收敛，[RMD]）.** 对 $(A,B)$ **可控**、$Q,R\succ0$，无穷时域 LQR 给出**收敛**的闭环系统 $x^+=Ax+B\kappa_\infty(x)$。

**证明（[RMD] 逐字）.** 无穷时域目标对所有 $x_0$ 有上界，因 $(A,B)$ 可控：可控性蕴含存在 $n$ 个输入 $u_0,\dots,u_{n-1}$ 把状态从任意 $x_0$ 转到 $x_n=0$；$k\ge n$ 后取零控制（$u_n,u_{n+1},\dots$）使 $V$ 中 $k\ge n$ 各项为零，故此无穷控制序列的目标有限。代价对 $\mathbf u$ 严格凸（$R\succ0$），故最优解唯一。

沿闭环轨迹考察 cost-to-go 序列：
$$V_k=V_{k+1}+\frac12\big(x_k^\top Q x_k+u_k^\top R u_k\big),$$
其中 $V_k=V^0(x_k)$ 是 $x_k$ 处 $k$ 时刻代价、$u_k=u^0(x_k)$ 是 $x_k$ 的最优控制。沿闭环 cost-to-go **非增且有下界（由 0）**，故序列 $\{V_k\}$ 收敛，且
$$x_k^\top Q x_k\to0,\qquad u_k^\top R u_k\to0\qquad(k\to\infty).$$
因 $Q,R\succ0$，得 $x_k\to0,\ u_k\to0$（$k\to\infty$），闭环收敛性成立。$\square$

**更强结论（[RMD]）**：由前节，最优解由迭代 Riccati 方程给出，最优无穷时域控制律与最优代价为
$$u^0(x)=Kx,\qquad V^0(x)=\frac12 x^\top\Pi x,$$
$$K=-(B^\top\Pi B+R)^{-1}B^\top\Pi A,$$
$$\boxed{\ \Pi=Q+A^\top\Pi A-A^\top\Pi B(B^\top\Pi B+R)^{-1}B^\top\Pi A.\ }\tag{1.18}$$
**证 Lemma 1.3 同时已证**：对 $(A,B)$ 可控、$Q,R\succ0$，DARE (1.18) 的**正定解存在**，且对应该解的 $K$ 使 $A+BK$ 的特征值**渐近稳定**（[RMD] 引 Bertsekas 1987, pp.58–64）。

> **放宽条件（[RMD] §1.3.6 末）**：系统限制可由**可控**放宽为**可镇定**（Exercises 1.19–1.20）；状态罚限制可由 $Q\succ0$ 放宽为 $Q\succeq0$ 且 $(A,Q)$ **可检测**（Exercise 1.20）；$R\succ0$ 保留以保唯一。
>
> **与 Lyapunov 的钩子（[RMD]）**：对线性系统渐近收敛 $\equiv$ 渐近稳定；第 2 章证明**最优代价 $V^0$ 是闭环 Lyapunov 函数**，并可据其形式把稳定性从渐近强化到**指数稳定**。

## §4 连续时间 LQR 经 HJB（[Tedrake-LQR] 全部方程）

> 连续时间主推导用 Hamilton–Jacobi–Bellman (HJB)。源 [Tedrake-LQR]。注意 Tedrake 阶段代价 $\ell=\mathbf x^\top\mathbf Q\mathbf x+\mathbf u^\top\mathbf R\mathbf u$ **不带 $\tfrac12$**，Riccati 矩阵记 $\mathbf S$。

### §4.1 无穷时域连续 LQR（[Tedrake-LQR]）

**系统**：$\dot{\mathbf x}=\mathbf A\mathbf x+\mathbf B\mathbf u$。**无穷时域代价**：
$$J=\int_0^\infty\big[\mathbf x^\top\mathbf Q\mathbf x+\mathbf u^\top\mathbf R\mathbf u\big]\,\mathrm dt,\qquad \mathbf Q=\mathbf Q^\top\succeq0,\ \mathbf R=\mathbf R^\top\succ0.$$
**HJB 方程**（最优值 $J^*(\mathbf x)$）：
$$0=\min_{\mathbf u}\Big[\mathbf x^\top\mathbf Q\mathbf x+\mathbf u^\top\mathbf R\mathbf u+\frac{\partial J^*}{\partial\mathbf x}(\mathbf A\mathbf x+\mathbf B\mathbf u)\Big].$$
**cost-to-go ansatz**：$J^*(\mathbf x)=\mathbf x^\top\mathbf S\mathbf x$，$\mathbf S=\mathbf S^\top\succeq0$。则 $\dfrac{\partial J^*}{\partial\mathbf x}=2\mathbf x^\top\mathbf S$。

**对 $\mathbf u$ 最小化**（HJB 内层，置梯度零）`\rebuilt`（[Tedrake] 给结果，此处补中间步）：
$$\frac{\partial}{\partial\mathbf u}\Big[\mathbf u^\top\mathbf R\mathbf u+2\mathbf x^\top\mathbf S\mathbf B\mathbf u\Big]=2\mathbf R\mathbf u+2\mathbf B^\top\mathbf S\mathbf x=0\ \Rightarrow\ \boxed{\ \mathbf u^*=-\mathbf K\mathbf x=-\mathbf R^{-1}\mathbf B^\top\mathbf S\mathbf x.\ }$$
代回 HJB（$\mathbf u^*$ 处），$\mathbf u^{*\top}\mathbf R\mathbf u^*=\mathbf x^\top\mathbf S\mathbf B\mathbf R^{-1}\mathbf B^\top\mathbf S\mathbf x$，$2\mathbf x^\top\mathbf S\mathbf B\mathbf u^*=-2\mathbf x^\top\mathbf S\mathbf B\mathbf R^{-1}\mathbf B^\top\mathbf S\mathbf x$，整理得**连续时间代数 Riccati 方程 (CARE)**：
$$\boxed{\ 0=\mathbf S\mathbf A+\mathbf A^\top\mathbf S-\mathbf S\mathbf B\mathbf R^{-1}\mathbf B^\top\mathbf S+\mathbf Q.\ }$$
最优反馈增益 $\mathbf K=\mathbf R^{-1}\mathbf B^\top\mathbf S$，$\mathbf u=-\mathbf K\mathbf x$。

### §4.2 有限时域连续 LQR（[Tedrake-LQR]）

**代价**：$J=h(\mathbf x(t_f))+\int_0^{t_f}\ell(\mathbf x,\mathbf u)\,\mathrm dt$，其中 $h(\mathbf x)=\mathbf x^\top\mathbf Q_f\mathbf x$，$\ell(\mathbf x,\mathbf u)=\mathbf x^\top\mathbf Q\mathbf x+\mathbf u^\top\mathbf R\mathbf u$。
**时变 cost-to-go**：$J^*(\mathbf x,t)=\mathbf x^\top\mathbf S(t)\mathbf x$，$\mathbf S(t)=\mathbf S^\top(t)\succ0$。
**HJB（含 $\partial J^*/\partial t$）`\rebuilt`**：$-\dfrac{\partial J^*}{\partial t}=\min_{\mathbf u}[\ell+\dfrac{\partial J^*}{\partial\mathbf x}(\mathbf A\mathbf x+\mathbf B\mathbf u)]$，$-\partial J^*/\partial t=-\mathbf x^\top\dot{\mathbf S}\mathbf x$。
**最优策略**：
$$\mathbf u^*=\pi^*(\mathbf x,t)=-\mathbf R^{-1}\mathbf B^\top\mathbf S(t)\mathbf x.$$
**连续时间微分 Riccati 方程 (CDRE)**：
$$\boxed{\ -\dot{\mathbf S}(t)=\mathbf S(t)\mathbf A+\mathbf A^\top\mathbf S(t)-\mathbf S(t)\mathbf B\mathbf R^{-1}\mathbf B^\top\mathbf S(t)+\mathbf Q,\ }$$
**终端条件** $\mathbf S(t_f)=\mathbf Q_f$。（反向积分求 $\mathbf S(t)$；$t_f\to\infty$ 且系统可镇定时 $\mathbf S(t)\to\mathbf S_{ss}$ 满足 CARE。）

### §4.3 用 LQR 镇定非线性系统（[Tedrake-LQR]）

**定点线性化**：取定点 $(\mathbf x_0,\mathbf u_0)$（$f(\mathbf x_0,\mathbf u_0)=0$），局部坐标 $\bar{\mathbf x}=\mathbf x-\mathbf x_0,\ \bar{\mathbf u}=\mathbf u-\mathbf u_0$，
$$\dot{\bar{\mathbf x}}\approx\mathbf A\bar{\mathbf x}+\mathbf B\bar{\mathbf u},\qquad \mathbf A=\frac{\partial f}{\partial\mathbf x}\Big|_{\mathbf x_0,\mathbf u_0},\quad \mathbf B=\frac{\partial f}{\partial\mathbf u}\Big|_{\mathbf x_0,\mathbf u_0}.$$
对线性化系统设计 LQR，得控制器
$$\mathbf u^*=\mathbf u_0-\mathbf K(\mathbf x-\mathbf x_0).$$
**轨迹镇定（时变 LQR）**：对标称轨迹 $\mathbf x_0(t),\mathbf u_0(t)$ 沿轨迹线性化得 $\mathbf A(t),\mathbf B(t)$，**反向**解微分 Riccati 方程得 $\mathbf S(t),\mathbf K(t)$，控制器
$$\mathbf u^*=\mathbf u_0(t)-\mathbf K(t)\big(\mathbf x-\mathbf x_0(t)\big).$$

## §5 连续 LQR 经 Pontryagin / 协态（Hamiltonian 两点边值法）

> 此为连续 LQR 的**第二条主推导**（变分/极小值原理），与 HJB 互补。源 [Wiki-LQR]（Hamiltonian、协态）+ [Bansal] 提及 + 标准变分法 `\rebuilt`（[Wiki "Costate equation"]/[Wiki "Pontryagin's maximum principle"] 框架）。

### §5.1 Pontryagin 一阶必要条件 `\rebuilt`

最小化 $J=\tfrac12 x(t_f)^\top Q_f x(t_f)+\tfrac12\int_0^{t_f}(x^\top Qx+u^\top Ru)\,\mathrm dt$，s.t. $\dot x=Ax+Bu$。引入协态（Lagrange 乘子）$\lambda(t)$，**控制 Hamiltonian**：
$$H(x,u,\lambda)=\tfrac12(x^\top Qx+u^\top Ru)+\lambda^\top(Ax+Bu).$$
**Pontryagin 极小值原理一阶条件**：
- 状态方程：$\dot x=\partial H/\partial\lambda=Ax+Bu$；
- **协态方程**：$\dot\lambda=-\partial H/\partial x=-Qx-A^\top\lambda$；
- 平稳条件：$\partial H/\partial u=Ru+B^\top\lambda=0\ \Rightarrow\ u=-R^{-1}B^\top\lambda$；
- 横截/终端条件：$\lambda(t_f)=Q_f\,x(t_f)$。

### §5.2 扫描法（sweep）：协态 = $P(t)x$ ansatz 回到 Riccati `\rebuilt`

设 $\lambda(t)=P(t)\,x(t)$，$P(t_f)=Q_f$。则 $u=-R^{-1}B^\top P x$，$\dot x=(A-BR^{-1}B^\top P)x$。又
$$\dot\lambda=\dot P x+P\dot x=\dot P x+P(A-BR^{-1}B^\top P)x.$$
与协态方程 $\dot\lambda=-Qx-A^\top Px$ 联立、对所有 $x$ 成立，得
$$\dot P+PA+A^\top P-PBR^{-1}B^\top P+Q=0,$$
即 **CDRE** $-\dot P=A^\top P+PA-PBR^{-1}B^\top P+Q$，$P(t_f)=Q_f$。与 §4.2（$\mathbf S\leftrightarrow P$，差 $\tfrac12$ 缩放）一致。两点边值问题（TPBVP）由此化为对 $P$ 的初值问题。

### §5.3 Hamiltonian 矩阵法（[Wiki-ARE]，CARE/DARE 的求解）

> [Wiki-ARE] 逐字（OCR 已恢复符号）。

**CARE** $A^\top P+PA-PBR^{-1}B^\top P+Q=0$ 的求解：定义 **Hamiltonian 矩阵**
$$Z=\begin{bmatrix}A&-BR^{-1}B^\top\\-Q&-A^\top\end{bmatrix}\in\mathbb R^{2n\times2n}.$$
**镇定解** $P=U_{21}U_{11}^{-1}$，其中 $\begin{bmatrix}U_{11}\\U_{21}\end{bmatrix}$ 的列张成 $Z$ 的"**稳定不变子空间**"（实部为负的特征值对应的特征/广义特征向量张成的 $n$ 维子空间）。

**DARE** $P=A^\top PA-(A^\top PB)(R+B^\top PB)^{-1}(B^\top PA)+Q$（[Wiki-ARE] 形式，$N=0$）的求解：当 $A$ 可逆，定义**辛矩阵 (symplectic)** $Z$（类似分块结构），解 $P=U_{21}U_{11}^{-1}$，其列对应**严格在单位圆内**的特征值。

**迭代法（[Wiki-ARE]）**：离散动态 Riccati 递推
$$P_{t-1}=Q+A^\top P_tA-A^\top P_tB(B^\top P_tB+R)^{-1}B^\top P_tA,$$
从 $P_T=Q$ 起"无限反向迭代"收敛到代数解。

**存在唯一性（[Wiki-ARE]）**：求解器通常寻"唯一镇定解（若存在）"——使闭环稳定的那个解。（充分条件：$(A,B)$ 可镇定 + $(A,Q^{1/2})$ 可检测，见 §B.2、§3.5。）

## §6 含交叉项 $N$ 的四种标准 LQR 公式速查（[Wiki-LQR] 全四式，逐字）

> [Wiki-LQR] 给出含状态-输入交叉项 $2x^\top Nu$ 的统一记号四式。**警示：此 $N$ 是交叉权重，非时域长度！** 本书建议改记 $\mathbf S_{xu}$。

### §6.1 连续有限时域
动力学 $\dot x=Ax+Bu,\ t\in[t_0,t_1]$。代价
$$J=x^\top(t_1)F(t_1)x(t_1)+\int_{t_0}^{t_1}\big(x^\top Qx+u^\top Ru+2x^\top Nu\big)\,\mathrm dt.$$
反馈律 $u=-Kx$，增益
$$K=R^{-1}(B^\top P(t)+N^\top).$$
**连续微分 Riccati**：
$$A^\top P(t)+P(t)A-\big(P(t)B+N\big)R^{-1}\big(B^\top P(t)+N^\top\big)+Q=-\dot P(t),\qquad P(t_1)=F(t_1).$$

### §6.2 连续无穷时域
$J=\int_0^\infty(x^\top Qx+u^\top Ru+2x^\top Nu)\,\mathrm dt$，$u=-Kx$，$K=R^{-1}(B^\top P+N^\top)$，**CARE**：
$$A^\top P+PA-(PB+N)R^{-1}(B^\top P+N^\top)+Q=0.$$
**等价（消交叉项）形式**：$A^\top P+PA-PBR^{-1}B^\top P+Q=0$，其中 $A\leftarrow A-BR^{-1}N^\top,\ Q\leftarrow Q-NR^{-1}N^\top$。

### §6.3 离散有限时域
$x_{k+1}=Ax_k+Bu_k$，代价
$$J=x_{H_p}^\top Q_{H_p}x_{H_p}+\sum_{k=0}^{H_p-1}\big(x_k^\top Qx_k+u_k^\top Ru_k+2x_k^\top Nu_k\big).$$
$u_k=-F_kx_k$，增益
$$F_k=(R+B^\top P_{k+1}B)^{-1}(B^\top P_{k+1}A+N^\top).$$
**离散 Riccati 差分方程**：
$$P_{k-1}=A^\top P_kA-(A^\top P_kB+N)(R+B^\top P_kB)^{-1}(B^\top P_kA+N^\top)+Q,\qquad P_{H_p}=Q_{H_p}.$$

### §6.4 离散无穷时域
$J=\sum_{k=0}^\infty(x_k^\top Qx_k+u_k^\top Ru_k+2x_k^\top Nu_k)$，$u_k=-Fx_k$，$F=(R+B^\top PB)^{-1}(B^\top PA+N^\top)$，**DARE**：
$$P=A^\top PA-(A^\top PB+N)(R+B^\top PB)^{-1}(B^\top PA+N^\top)+Q.$$
**等价形式**：$P=A^\top PA-A^\top PB(R+B^\top PB)^{-1}B^\top PA+Q$，$A\leftarrow A-BR^{-1}N^\top,\ Q\leftarrow Q-NR^{-1}N^\top$。

**[Wiki-LQR] 记号约定表**：$Q$=状态权重；$R$=输入权重；$N$=交叉项权重；$F$（连续）/$Q_{H_p}$（离散）=终端权重。

> **四式与 §1–§5 的一致性核验（综合时写盒）**：取 $N=0$，则 §6.4 离散无穷式 $\to$ §2.1 DARE (12)/§3.5 (1.18)（增益差负号约定）；§6.2 连续无穷式 $\to$ §4.1 CARE；§6.3 离散有限 $\to$ §1.3 (7)/§3.3 (1.10)（注意 [Wiki] $P_{k-1}$ 递推与 [RMD] $\Pi_{k-1}$ 同）；§6.1 连续有限 $\to$ §4.2 CDRE。四源闭合。

---

# 第三部分：模型预测控制 MPC —— 滚动时域优化完整表述

## §7 MPC 的滚动/后退时域原理（[Wiki-MPC] + [RMD] §2.2 引言）

### §7.1 工程视角原理（[Wiki-MPC]，逐字要点）
MPC（亦称**滚动/后退时域控制 receding horizon control**）：
1. **有限时域优化**：在时刻 $t$ 采样当前状态，对相对短的未来时域 $[t,t+T]$ 用数值优化算出最小代价控制策略；
2. **实施 + 重优化**：**只实施控制策略的第一步**，然后重新采样状态、从新当前态重复计算，得到新控制与新预测路径；
3. **滚动时域**：预测时域不断前移，故称 receding horizon。

**三要素（[Wiki-MPC]）**：(i) 过程的内部动力学模型；(ii) 滚动时域上的代价函数 $J$；(iii) 在控制输入 $u$ 上极小化 $J$ 的优化算法。

**线性 MPC 的跟踪型二次代价（[Wiki-MPC]）**：
$$J=\sum_{i=1}^{N}w_{x_i}(r_i-x_i)^2+\sum_{i=1}^{M}w_{u_i}(\Delta u_i)^2,$$
$x_i$=第 $i$ 受控变量、$r_i$=设定点、$\Delta u_i$=控制增量、$w_{x_i}$=跟踪误差权重、$w_{u_i}$=控制增量权重、$N$=预测时域、$M$=控制时域。**离散线性模型** $x_{k+1}=Ax_k+Bu_k$，预测前向传播。

**约束分类（[Wiki-MPC]）**：(i) 输入约束（操纵变量限幅）；(ii) 状态约束（内部变量）；(iii) 输出约束（被测/受控变量上下限）。

**MPC vs LQR（[Wiki-MPC] 逐字）**："LQR optimizes across the entire time window (horizon) whereas MPC optimizes in a receding time window. With MPC a new solution is computed often whereas LQR uses the same single (optimal) solution for the whole time horizon."（LQR 在整个时域上优化一次得单一解全程使用；MPC 在滚动窗口上反复求解。）

**线性 vs 非线性 MPC（[Wiki-MPC]）**：NMPC 用非线性预测模型；线性 MPC 的优化问题**凸**（QP），NMPC **不必凸**。NMPC 数值法含直接单打靶 (single shooting)、直接多重打靶 (multiple shooting)、直接配点 (collocation)。

### §7.2 [RMD] §2.2 引言：从无穷时域理想问题到 MPC（逐字）

物理导出的非线性系统多为连续模型 $\mathrm dx/\mathrm dt=f(x,u)$。**闭环性质最佳的控制律**是如下无穷时域约束最优控制问题之解。代价
$$V_\infty(x,\mathbf u)=\int_0^\infty\ell(x(t),u(t))\,\mathrm dt,$$
$x,u$ 满足 $\dot x=f(x,u)$。问题 $\mathbb P_\infty(x)$：$\min_{\mathbf u}V_\infty(x,\mathbf u)$ s.t. $\dot x=f(x,u),\ x(0)=x_0,\ (x(t),u(t))\in Z\ \forall t$。$\ell$ 正定时调节器目标是把状态导到原点。

记解 $\mathbf u_\infty^0(\cdot;x)$、最优值 $V_\infty^0(x)$。闭环 $\mathrm dx(t)/\mathrm dt=f(x(t),u_\infty^0(t;x))$。若 $f,\ell,V_f$ 满足适当可微/增长假设、容许控制类足够丰富，则 $\mathbb P_\infty(x)$ 对所有 $x$ 有解且
$$\dot V_\infty^0(x)=-\ell(x,u_\infty^0(0;x));$$
据此 + $V_\infty^0$ 的上下界即可证原点**全局渐近稳定**。

**理想问题的障碍（[RMD]）**：(i) 解 $\mathbb P_\infty$ 给的是**开环序列**非反馈律（不确定性下需反馈）；(ii) 优化对象是无穷维时间函数；(iii) $[0,\infty)$ 半无穷区间；(iv) $V(x,\mathbf u)$ 一般非凸。**MPC 的任务**：限制系统与控制参数化使 $\mathbb P_\infty$ 可计算——通常 (a) 用离散差分方程替连续微分方程；(b) 用**有限**时域替半无穷区间并附加**终端域 + 终端代价**以逼近终端域内的 cost-to-go。

## §8 非线性时不变 MPC 的完整表述（[RMD] §2.2，逐式 2.1–2.8）

> 本节是 MPC **问题表述的权威主源**，逐式保真。

### §8.1 系统与约束（[RMD] 式 2.1–2.2）

非线性时不变系统由差分方程描述：
$$x^+=f(x,u),\tag{2.1}$$
$x\in\mathbb R^n$ 当前态、$u$ 当前控制、$x^+$ 后继态。$f$ 连续、$f(0,0)=0$（$(0,0)$ 为期望平衡对）。可推广到一般平衡对 $(x_s,u_s)$（$x_s=f(x_s,u_s)$）。

**记号（[RMD]）**：$\mathbb I=\{0,1,2,\dots\}$，$\mathbb I_{m:n}=\{m,m+1,\dots,n\}$。事件 $(x,i)$ 表"$i$ 时刻状态为 $x$"。控制序列 $\mathbf u=(u_0,u_1,\dots)$；MPC 中常为有限序列 $\mathbf u=(u_0,\dots,u_{N-1})$，$N$=**控制时域**。$\boldsymbol\phi(k;x,\mathbf u)$=初值 $x$、控制 $\mathbf u$ 下 (2.1) 在时刻 $k$ 的解（只依赖 $u_0,\dots,u_{k-1}$）；时不变 $\Rightarrow$ $\boldsymbol\phi(k;x,i,\mathbf u)=\boldsymbol\phi(k-i;x,\mathbf u)$。

**Proposition 2.1（系统解的连续性）.** $f$ 连续 $\Rightarrow$ 对每个 $k\in\mathbb I$，映射 $(x,\mathbf u)\mapsto\boldsymbol\phi(k;x,\mathbf u)$ 连续。
*证（归纳，[RMD]）*：$\boldsymbol\phi(1;x,u_0)=f(x,u_0)$ 连续；设 $\boldsymbol\phi(j;\cdot)$ 连续，则 $\boldsymbol\phi(j+1;x,\mathbf u_j)=f(\boldsymbol\phi(j;x,\mathbf u_{j-1}),u_j)$ 是连续函数复合，故连续。归纳得证。$\square$

**硬约束（[RMD] 式 2.2）**：
$$(x_k,u_k)\in Z\quad\forall k\in\mathbb I,\qquad Z=\{(x,u)\mid Fx+Eu\le e\}\ (\text{多面体}).$$
（例：速率约束 $|u_k-u_{k-1}|\le c$ 经增广态 $z^+=u$（$z_k=u_{k-1}$）化为 $|u_k-z_k|\le c$。）$(x,u)\in Z$ 蕴含状态相关控制约束集
$$u\in\mathbb U(x):=\{u\in\mathbb R^m\mid(x,u)\in Z\},$$
与状态约束 $x\in\mathbb X:=\{x\in\mathbb R^n\mid\mathbb U(x)\ne\varnothing\}$。无混合约束时 $Z=\mathbb X\times\mathbb U$，约束化为 $x_k\in\mathbb X,\ u_k\in\mathbb U$。本章设状态 $x$ 已知（若估计则需鲁棒 MPC，第 3 章）。

### §8.2 代价、终端约束、时不变化（[RMD] §2.2，式 2.3–2.6）

事件 $(x,i)$ 处的最优控制问题 $\mathbb P_N(x,i)$ 极小化区间 $[i,N+i]$ 上的代价
$$\sum_{k=i}^{i+N-1}\ell(x_k,u_k)+V_f(x_{i+N}),$$
对序列 $\mathbf x=(x_i,\dots,x_{i+N})$、$\mathbf u=(u_i,\dots,u_{i+N-1})$，s.t. (2.1)、$x_i=x$、约束 (2.2)。$\ell$ 连续，$\ell(0,0)=0$。

**时不变化（[RMD] 关键论证）**：因系统 $x^+=f(x,u)$、阶段代价 $\ell$、终端代价 $V_f$ 均时不变，$\mathbb P_N(x,i)$ 之解对任意 $i$ 与 $\mathbb P_N(x,0)$ 相同：
$$\mathbf u^0(x,i)=\mathbf u^0(x,0),\quad \mathbf x^0(x,i)=\mathbf x^0(x,0).$$
特别地 $u^0(i;x,i)=u^0(0;x,0)$——施加到对象的控制 $=\mathbf u^0(x,0)$ 的首元。故只需考虑 $\mathbb P_N(x,0)$，因初始时刻无关紧要，记 $\mathbb P_N(x)$；并简记 $\mathbf u^0(x),\mathbf x^0(x)$。

**约束化的代价（[RMD] 式 2.3）**：把状态序列 $\mathbf x$ 先验约束为 (2.1) 之解，则 $x_k=\boldsymbol\phi(k;x,\mathbf u)$ 是 $(x,\mathbf u)$ 的函数，代价纯为 $(x,\mathbf u)$ 之函数：
$$\boxed{\ V_N(x,\mathbf u):=\sum_{k=0}^{N-1}\ell(x_k,u_k)+V_f(x_N),\quad x_k=\boldsymbol\phi(k;x,\mathbf u).\ }\tag{2.3}$$
**终端约束**：$x_N\in X_f\subseteq\mathbb X$，对 $\mathbf u$ 施加隐式约束 $\mathbf u\in\mathcal U_N(x)$（式 2.4）：
$$\mathcal U_N(x):=\{\mathbf u\mid(x,\mathbf u)\in\mathcal Z_N\},\tag{2.5}$$
$$\mathcal Z_N:=\{(x,\mathbf u)\mid(\boldsymbol\phi(k;x,\mathbf u),u_k)\in Z,\ \forall k\in\mathbb I_{0:N-1};\ \boldsymbol\phi(N;x,\mathbf u)\in X_f\}.\tag{2.6}$$

### §8.3 最优控制问题 $\mathbb P_N(x)$、值函数、可行集（[RMD] 式 2.7–2.8）

**最优控制问题**：
$$\boxed{\ \mathbb P_N(x):\quad V_N^0(x):=\min_{\mathbf u}\{V_N(x,\mathbf u)\mid\mathbf u\in\mathcal U_N(x)\}.\ }\tag{2.7}$$
这是**参数优化问题**（决策变量 $\mathbf u$，代价与约束集都依赖参数 $x$）。$\mathcal Z_N$=容许 $(x,\mathbf u)$ 集。**可行集**（有解的状态集）：
$$X_N:=\{x\in\mathbb X\mid\mathcal U_N(x)\ne\varnothing\}=\{x\in\mathbb R^n\mid\exists\mathbf u\in\mathbb R^{Nm}\ \text{s.t.}\ (x,\mathbf u)\in\mathcal Z_N\},\tag{2.8}$$
即 $\mathcal Z_N\subseteq\mathbb R^n\times\mathbb R^{Nm}$ 到 $\mathbb R^n$ 的正交投影。$V_N^0(\cdot)$ 的定义域为 $X_N$。

**解的存在性（[RMD] Assumptions 2.2–2.3, Proposition 2.4）**：
- **Assumption 2.2（系统与代价连续）.** $f:Z\to\mathbb R^n$、$\ell:Z\to\mathbb R$、$V_f:X_f\to\mathbb R$ 连续，$f(0,0)=0,\ \ell(0,0)=0,\ V_f(0)=0$。
- **Assumption 2.3（约束集性质）.** $Z$ 闭。若有控制约束，$\mathbb U(x)$ 紧、在 $\mathbb X$ 上一致有界；$X_f\subseteq\mathbb X$ 紧；各集含原点。若无控制约束，则 $\mathbf u\mapsto V_N(x,\mathbf u)$ 强制 (coercive)（$|\mathbf u|\to\infty$ 时 $V_N\to\infty$）。
- **Proposition 2.4（解存在）.** 设 Assumptions 2.2–2.3 成立。则 (a) $V_N(\cdot)$ 在 $\mathcal Z_N$ 上连续；(b) 对每个 $x\in X_N$，控制约束集 $\mathcal U_N(x)$ 紧；(c) 对每个 $x\in X_N$，$\mathbb P_N(x)$ 有解。
  *证（[RMD]）*：(a) 由 $\ell,V_f$ 连续（Assn 2.2）+ $\boldsymbol\phi(j;\cdot)$ 连续（Prop 2.1）；(b) $\mathcal U_N(x)$ 由有限个连续约束界定、紧；(c) 代价连续 + 约束集紧 $\Rightarrow$ Weierstrass 定理给解。$\square$

### §8.4 滚动时域控制律（[RMD] §2.2 末）

MPC 把最优序列 $\mathbf u^0(x)$ 的**首元**施加到对象：
$$\boxed{\ \kappa_N(x):=u^0(0;x),\qquad x^+=f(x,\kappa_N(x)).\ }$$
（解非唯一时，$\mathbf u^0(x),\kappa_N(x)$ 可能集值；取最小范数元 + 任意选择映射保唯一。后文用 $\mathbf u^0(x)$ 记任一极小序列、$\kappa_N(x)=u^0(0;x)$ 记其首元。）

**值函数/控制律的连续性（[RMD] Theorem 2.7）`\rebuilt`（条目，未抄全证）**：附加适当假设下 $V_N^0$ 与 $\kappa_N$ 连续；但**一般 MPC 控制律可不连续**（Example 2.8 给出不连续 MPC 律的反例）。

## §9'（编号续）DP 解与 Riccati：MPC 的动态规划视角（[RMD] §2.3，式 2.9–2.11）

> [RMD] §2.3 用 DP 递推刻画 $V_j^0$，是 §10 稳定性证明的引擎。

**DP 递推（[RMD] 式 2.9，OCR 恢复）**：定义 $V_0^0(\cdot):=V_f(\cdot)$、$X_0:=X_f$，对 $j=1,2,\dots$：
$$V_j^0(x)=\min_{u\in\mathbb U(x)}\big\{\ell(x,u)+V_{j-1}^0(f(x,u))\mid f(x,u)\in X_{j-1}\big\},\tag{2.9}$$
最优控制 $\kappa_j(x)=\arg\min(\cdots)$，且**可行集递推**
$$X_j=\{x\in\mathbb X\mid\exists u\in\mathbb U(x)\ \text{s.t.}\ f(x,u)\in X_{j-1}\}.\tag{2.11}$$
**集合不变性定义（[RMD] Definition 2.9）.** (a) 集 $X$ 对 $x^+=f(x)$ **正不变 (positive invariant)** 若 $x\in X\Rightarrow f(x)\in X$。(b) 集 $X$ 对 $x^+=f(x,u),\ u\in\mathbb U$ **控制不变 (control invariant)** 若对所有 $x\in X$ 存在 $u\in\mathbb U(x)$ 使 $f(x,u)\in X$。
**Proposition 2.10（DP 递推解的存在）.** 若 $X_f$ 控制不变，则各 $X_j$ 控制不变，且 $X_j\supseteq X_{j-1}$（**嵌套**），$\{X_j\}$ 递增。（即终端域控制不变 $\Rightarrow$ $X_N$ 对闭环 $x^+=f(x,\kappa_N(x))$ 正不变。）

> **DP 与逐 horizon 的关系（[RMD]）**：$V_N^0$（式 2.7 的值函数）与 DP 递推 (2.9) 的 $V_N^0$ 一致（$N$ 次递推）；这把"对每个 $x$ 解 QP"与"反向 DP"统一——线性二次情形 DP 递推即 §3.3 的 Riccati 迭代 (1.10)。

## §10 MPC 稳定性理论（[RMD] §2.4，逐式 2.14–2.21，含完整证明链）

> 本节是 MPC **稳定性的权威主源**，从 Lyapunov 定义到主定理 2.19，逐式逐证。**全章核心。**

### §10.1 渐近稳定与 Lyapunov 函数定义（[RMD] §2.4.1，Def 2.11–2.12，Thm 2.13）

**函数类**：$\mathcal K$=连续、零点为零、严格增；$\mathcal K_\infty$=$\mathcal K$ 且无界；$\mathcal{KL}$=连续，对每个 $k\ge0$，$\beta(\cdot,k)\in\mathcal K$，对每个 $s\ge0$，$\beta(s,\cdot)$ 非增且 $\beta(s,i)\to0\,(i\to\infty)$。

**Definition 2.11（渐近稳定 AS 与 GAS）.** 设 $X$ 对 $x^+=f(x)$ 正不变。原点对 $x^+=f(x)$ 在 $X$ 中 **AS**，若存在 $\mathcal{KL}$ 函数 $\beta$ 使对每个 $x\in X$，
$$|\boldsymbol\phi(i;x)|\le\beta(|x|,i)\qquad\forall i\in\mathbb I.$$
若 $X=\mathbb R^n$，则原点对 $x^+=f(x)$ **GAS**。（$X$ 称**吸引域 region of attraction**。）

**Definition 2.12（Lyapunov 函数）.** 设 $X$ 对 $x^+=f(x)$ 正不变。$V:\mathbb R^n\to\mathbb R_{\ge0}$ 是 $X$ 中 $x^+=f(x)$ 的 Lyapunov 函数，若存在 $\alpha_1,\alpha_2\in\mathcal K_\infty$ 与连续正定 $\alpha_3$，使对任意 $x\in X$：
$$V(x)\ge\alpha_1(|x|),\tag{2.14}$$
$$V(x)\le\alpha_2(|x|),\tag{2.15}$$
$$V(f(x))-V(x)\le-\alpha_3(|x|).\tag{2.16}$$

**Theorem 2.13（Lyapunov 稳定性定理）.** 设 $X\subseteq\mathbb R^n$ 对 $x^+=f(x)$ 正不变。若存在 $X$ 中的 Lyapunov 函数，则原点在 $X$ 中 AS；若 $X=\mathbb R^n$ 则 GAS。若 $\alpha_i(|x|)=c_i|x|^a\ (a;c_i\in\mathbb R_{>0},i=1,2,3)$，则原点**指数稳定**。

> **思路（[RMD]）**：标准做法是用**无穷时域**问题的值函数作 Lyapunov 函数；$V_\infty^0$ 满足 $V_\infty^0(f(x,\kappa_\infty(x)))-V_\infty^0(x)\le-\ell(x,\kappa_\infty(x))$（性质 2.16）。但"**最优 ≠ 稳定**"（Kalman 反例 §3.4）——有限时域时此性质通常不成立。本章核心任务：若 $V_f,\ell,X_f$ 选得当，则
> $$V_N^0(f(x,\kappa_N(x)))\le V_N^0(x)-\ell(x,\kappa_N(x))\quad\forall x\in X_N,$$
> 从而得 (2.16)。下界 (2.14) 易得；上界 (2.15) 较难，但同样的"得当 ingredients"也保证 (2.15)。

### §10.2 基本稳定性假设（[RMD] §2.4.2，Assumption 2.14）

> **Assumption 2.14（基本稳定性假设，[RMD]，MPC 稳定性的核心条件）.** $V_f(\cdot),X_f,\ell$ 满足：
> **(a)（终端域控制不变 + 终端代价下降）** 对所有 $x\in X_f$，存在 $u$（使 $(x,u)\in Z$）满足
> $$f(x,u)\in X_f,\qquad\boxed{\ V_f(f(x,u))-V_f(x)\le-\ell(x,u).\ }$$
> **(b)（下界）** 存在 $\mathcal K_\infty$ 函数 $\alpha_1,\alpha_f$ 使
> $$\ell(x,u)\ge\alpha_1(|x|)\quad\forall x\in X_N,\ \forall u\ \text{s.t.}\ (x,u)\in Z;\qquad V_f(x)\le\alpha_f(|x|)\quad\forall x\in X_f.$$

> **直觉（综合时写盒）**：(a) 说在终端域内存在一个"局部控制器 $\kappa_f$"使 $V_f$ 像无穷时域 cost-to-go 一样**沿轨迹下降至少 $\ell$**——即 $V_f$ 是终端域内闭环的 control-Lyapunov 函数（CLF）。这正是把有限时域"接续"成无穷时域稳定性的关键。常见落地：(i) **终端等式约束** $X_f=\{0\},V_f=0$（$\kappa_f=0$ 在原点）；(ii) **终端代价 = 局部 LQR 的 cost-to-go**：$V_f(x)=\tfrac12 x^\top P_f x$（$P_f$ 解 DARE），$X_f$=该 LQR 在约束内的不变椭球，$\kappa_f$=LQR 律——则 (a) 取等号（[RMD] §2.5、Fig 2.11）。

### §10.3 下界 (2.14)、上界 (2.15)（[RMD] §2.4.2，Prop 2.15–2.16）

**下界 (2.14)（[RMD]）**：因 $V_N^0(x)\ge\ell(x,\kappa_N(x))$（首阶段代价非负且 $V$ 含它）对所有 $x\in X_N$，由 Assn 2.14(b)（$\ell(x,u)\ge\alpha_1(|x|)$）得 (2.14)。"通常选 $\ell(x,u)=\tfrac12(x^\top Qx+u^\top Ru)$、$Q,R\succ0$"满足此假设。

**上界 (2.15)（[RMD]，经两 Proposition）**：若 $X_f$ 含原点于内部，先证（Prop 2.18，下节）$V_j^0(x)\le V_f(x)\ \forall x\in X_f,\forall j\in\mathbb I$；又 Assn 下 $\exists\alpha_f\in\mathcal K_\infty$ 使 $V_f(x)\le\alpha_f(|x|)\ \forall x\in X_f$，故 $V_N^0$ 在 $X_f$ 上有同样上界 $\alpha_f(|x|)$。再把 $X_f$ 内的界扩到 $X_N$：
- **Proposition 2.15（$V_N^0$ 局部有界）.** 设 Assn 2.2–2.3。则 $V_N^0$ 在 $X_N$ 上局部有界。
  *证（[RMD]）*：取 $X_N$ 任意紧子集 $\mathcal X$。$V_N:\mathbb R^n\times\mathbb R^{Nm}\to\mathbb R$ 连续，在紧集 $\mathcal X\times\mathcal U_N$ 上有上界；因 $\mathcal U_N(x)\subseteq\mathcal U_N\ \forall x\in\mathcal X$，$V_N^0:X_N\to\mathbb R$ 在 $\mathcal X$ 上有同上界。$\mathcal X$ 任意，故 $V_N^0$ 在 $X_N$ 上局部有界。$\square$
- **Proposition 2.16（上界从 $X_f$ 扩到 $X_N$）.** 设 Assn 2.2–2.3，$X_f\subseteq\mathbb X$ 对 $x^+=f(x,u),u\in\mathbb U(x)$ 控制不变且含原点于内部；又设 $\exists\alpha_f\in\mathcal K_\infty$ 使 $V_f(x)\le\alpha_f(|x|)\ \forall x\in X_f$。则 $\exists\alpha_2\in\mathcal K_\infty$ 使
  $$V_N^0(x)\le\alpha_2(|x|)\qquad\forall x\in X_N.$$
  （证较长，[RMD] 用局部有界 + 控制不变的扩展构造，本抽取记结论。）

**Assumption 2.17（[RMD]，上界的弱可控性，条目）`\rebuilt`**：当原点在 $X_f$ 内部时 Assn 2.17 自动成立；终端等式约束 $X_f=\{0\}$ 时直接采纳之。

### §10.4 值函数单调性（[RMD] Proposition 2.18，含完整证明）

> 此命题是下降性的引擎，[RMD] 称"此有趣结果最早对无约束线性二次问题得到"。

**Proposition 2.18（值函数单调性）.** 设 Assn 2.2, 2.3, 2.14 成立。则
$$V_j^0(x)\le V_{j-1}^0(x)\quad\forall x\in X_{j-1},\ \forall j\in\mathbb I;\qquad V_j^0(x)\le V_f(x)\quad\forall x\in X_f,\ \forall j\in\mathbb I.$$

**证（[RMD] 归纳，逐式）.** 由 DP 递推 (2.9)：
$$V_1^0(x)=\min_{u\in\mathbb U(x)}\{\ell(x,u)+V_0^0(f(x,u))\mid f(x,u)\in X_0\}.$$
但 $V_0^0:=V_f$、$X_0:=X_f$。由 Assn 2.14(a)，
$$\min_{u\in\mathbb U(x)}\{\ell(x,u)+V_f(f(x,u))\mid f(x,u)\in X_f\}\le V_f(x)\qquad\forall x\in X_f,$$
故 $V_1^0(x)\le V_0^0(x)=V_f(x)\ \forall x\in X_1=X_f$（基例）。

设对某 $j-1$：$V_{j-1}^0(x)\le V_{j-2}^0(x)\ \forall x\in X_{j-2}$。由 DP (2.9)，对 $x\in X_{j-1}$，设 $\kappa_{j-1}(x)$ 为 $\mathbb P_{j-1}$ 的最优控制，则
$$V_j^0(x)\le\ell(x,\kappa_{j-1}(x))+V_{j-1}^0(f(x,\kappa_{j-1}(x))),$$
（因 $\kappa_{j-1}(x)$ 对 $\mathbb P_j$ 未必最优，给上界）。又
$$V_{j-1}^0(x)=\ell(x,\kappa_{j-1}(x))+V_{j-2}^0(f(x,\kappa_{j-1}(x))).$$
由 (2.11)，$x\in X_{j-1}\Rightarrow f(x,\kappa_{j-1}(x))\in X_{j-2}$，故由归纳假设 $V_{j-1}^0(f(x,\kappa_{j-1}(x)))\le V_{j-2}^0(f(x,\kappa_{j-1}(x)))$。两式相减：
$$V_j^0(x)\le V_{j-1}^0(x)\qquad\forall x\in X_{j-1}.$$
归纳得 $V_j^0(x)\le V_{j-1}^0(x)\ \forall x\in X_{j-1},\forall j$。又 $\{X_j\}$ 嵌套（$X_{j-1}\subseteq X_j$），逐次得 $V_j^0(x)\le V_f(x)\ \forall x\in X_f,\forall j$。$\square$
（[RMD] 注：即便 $\mathbb U(x)$ 非紧，只要 DP 递推极小元总存在，单调性仍成立——线性二次情形正是如此。）

### §10.5 下降性与主定理（[RMD]，式 2.17 + Theorem 2.19）

**下降性推导（[RMD]，用单调性）.** 由 DP 递推，
$$V_N^0(x)=\ell(x,\kappa_N(x))+V_{N-1}^0(f(x,\kappa_N(x)))\ge\ell(x,\kappa_N(x))+V_N^0(f(x,\kappa_N(x)))\ \text{?}$$
[RMD] 的精确链：
$$V_N^0(x)=\ell(x,\kappa_N(x))+V_{N-1}^0(f(x,\kappa_N(x))),$$
由单调性 $V_N^0(f(x,\kappa_N(x)))\le V_{N-1}^0(f(x,\kappa_N(x)))$，故
$$\boxed{\ V_N^0(f(x,\kappa_N(x)))\le V_N^0(x)-\ell(x,\kappa_N(x))\qquad\forall x\in X_N.\ }\tag{2.17}$$
（即 $V_N^0(f(x,\kappa_N(x)))-V_N^0(x)\le-\ell(x,\kappa_N(x))$。）因 $\ell(x,\kappa_N(x))\ge\alpha_1(|x|)\ \forall x\in\mathbb X$（Assn 2.14b），$V_N^0$ 有所需**下降性 (2.16)**。

> **[RMD] 等价表述（式 2.18）**：(2.17) 对所有 $x\in\mathbb R^n$ 成立的充分条件即"$V_f,X_f$ 满足：对所有 $x\in X_f$ 存在 $u$ 使 $(x,u)\in Z,\ V_f(f(x,u))\le V_f(x)-\ell(x,u),\ f(x,u)\in X_f$"——这正是 Assn 2.14(a)。

至此 (2.14)(2.15)(2.16) 全部满足，故：

> **Theorem 2.19（原点的渐近稳定性，MPC 主定理，[RMD]）.** 设 Assumptions 2.2, 2.3, 2.14, 2.17 满足。则
> **(a)** 存在 $\mathcal K_\infty$ 函数 $\alpha_1,\alpha_2$ 使
> $$\alpha_1(|x|)\le V_N^0(x)\le\alpha_2(|x|),\qquad V_N^0(f(x,\kappa_N(x)))-V_N^0(x)\le-\alpha_1(|x|)\qquad\forall x\in X_N.$$
> **(b)** 原点对闭环 $x^+=f(x,\kappa_N(x))$ 在 $X_N$ 中**渐近稳定**。

**证（[RMD]）.** (2.14)(2.15)(2.16) 由 §10.3–§10.5 全部建立，$X_N$ 对闭环正不变（$X_f$ 控制不变 $\Rightarrow$ Prop 2.10）。故 $V_N^0$ 是 $X_N$ 中闭环的 Lyapunov 函数，由 Theorem 2.13 得原点在 $X_N$ 中 AS。$\square$

> **[RMD] 注**：原点在 $X_f$ 内部时 Assn 2.17 立即成立；终端等式约束 $X_f=\{0\}$ 时 Assn 2.17 直接采纳（Prop 2.38 给更多成立情形）。

### §10.6 指数稳定（[RMD] §2.4.3，Def 2.20–Thm 2.21）

**Definition 2.20（指数稳定）.** 设 $X$ 对 $x^+=f(x)$ 正不变。原点指数稳定，若 $\exists c\in\mathbb R_{>0},\gamma\in(0,1)$ 使
$$|\boldsymbol\phi(i;x)|\le c\,|x|\,\gamma^i\qquad\forall x\in X,\forall i\in\mathbb I.$$

**Theorem 2.21（Lyapunov 函数与指数稳定）.** 设 $X\subseteq\mathbb R^n$ 对 $x^+=f(x)$ 正不变。若存在 $X$ 中 Lyapunov 函数且各界取幂律 $\alpha_i(s)=c_i s^a\ (a;c_i\in\mathbb R_{>0},i=1,2,3)$，则原点在 $X$ 中指数稳定。（证留作练习。）

> **线性二次 MPC 即满足**：$\ell,V_f$ 二次 $\Rightarrow$ $\alpha_i$ 可取 $a=2$ 幂律，故 LQ-MPC 在 $X_N$ 内**指数稳定**（[RMD] §1.3.6 末与 §2.4.3 呼应；与 Example 2.5 的数值观察"原点指数稳定"一致）。

### §10.7 可控性/可观性的隐含使用 + 非正定 $\ell$（[RMD] §2.4.4，Thm 2.24 条目）`\rebuilt`

[RMD] 指出：稳定性分析未显式假设可控/可观，因 (i) 基本稳定性假设 2.14 已**隐式局部要求**之；(ii) 限于 $X_N$（$N$ 步内可驱入 $X_f$ 且满足约束的状态集）。**阶段代价 $\ell$ 非正定**（如 $\ell(y,u)$ 经不可逆 $C$：$\ell=\tfrac12(x^\top Qx+u^\top Ru)$ 但 $Q=C^\top C$ 半正定）情形由 **Theorem 2.24（带 $\ell(y,u)$ 的渐近稳定）** 处理：仅需下界对 $y=h(x)$ 成立 + 可检测性，结论仍得 AS。（条目，未抄全证。）

## §11 线性二次 MPC 的具体 QP 与数值例（[RMD] Example 2.5, 2.6，逐式 + 数值）

> 把抽象 $\mathbb P_N(x)$ 落到最小可算例，给出**显式 QP、Hessian、闭环控制律、数值轨迹、稳定性验证**。

**Example 2.5（线性二次 MPC，[RMD]）.** 系统 $x^+=f(x,u):=x+u$，初值 $x_0$。阶段/终端代价
$$\ell(x,u):=\tfrac12(x^2+u^2),\qquad V_f(x):=\tfrac12 x^2.$$
控制约束 $u\in[-1,1]$，无状态/终端约束，时域 $N=2$。

**第一种（同时法，决策变量含 $\mathbf u,\mathbf x$）**：极小化
$$V_N(x_0,x_1,x_2,u_0,u_1)=\tfrac12\big(x_0^2+x_1^2+x_2^2+u_0^2+u_1^2\big)$$
对 $x_0,x_1,x_2,u_0,u_1$，s.t.
$$x_0=x,\quad x_1=x_0+u_0,\quad x_2=x_1+u_1,\quad u_0\in[-1,1],\ u_1\in[-1,1].$$
$u\in[-1,1]\equiv\{u\le1,\ -u\le1\}$（两线性不等式）；前三式为强制差分方程的等式约束。

**第二种（约化法/condensing，决策变量仅 $\mathbf u$）**：前三式自动满足（$\mathbf x$ 取差分方程解），问题化为对 $\mathbf u=(u_0,u_1)$ 极小化
$$V_N(x,\mathbf u)=\tfrac12\Big(x^2+(x+u_0)^2+(x+u_0+u_1)^2+u_0^2+u_1^2\Big)$$
$$=\tfrac32 x^2+\begin{bmatrix}2x&x\end{bmatrix}\mathbf u+\tfrac12\mathbf u^\top H\mathbf u,$$
其中
$$\boxed{\ H=\begin{bmatrix}3&1\\1&2\end{bmatrix},\ }$$
s.t. $\mathbf u\in\mathcal U_N(x)=\{\mathbf u\mid|u_k|\le1,\ k=0,1\}$。因无状态/终端约束，$\mathcal U_N(x)=\mathcal U_N$ 不依赖 $x$（一般会依赖）。**两种皆为 QP**。

**数值解（[RMD]）**：$x=10$ 时 $u^0(1;10)=u^0(2;10)=-1$（受限饱和），最优状态轨迹 $x^0(0;10)=10,\ x^0(1;10)=9,\ x^0(2;10)=8$，最优值 $V_N^0(10)=124$。对每个 $x\in[-10,10]$ 解 $\mathbb P_N(x)$ 得**隐式 MPC 控制律**（Fig 2.1a），时不变。闭环满足
$$x^+=x+\kappa_N(x),\qquad\boxed{\ \kappa_N(x)=\operatorname{sat}(3x/5)\ }$$
（$\operatorname{sat}$ 为 $[-1,1]$ 饱和）。$x_0=10$ 的轨迹见 Fig 2.1b。**此简单情形原点指数稳定**；但通常终端代价与终端约束集需精心选择以保稳定。

**Example 2.6（细看，[RMD]）.** 目标函数
$$V_N(x,\mathbf u)=\tfrac12\mathbf u^\top H\mathbf u+c(x)^\top\mathbf u+d(x),\quad c(x)=\begin{bmatrix}2\\1\end{bmatrix}x,\ d(x)=\tfrac32 x^2.$$
（无约束解 $\mathbf u^*=-H^{-1}c(x)$；叠加 $u\in[-1,1]$ 后即上述 $\kappa_N(x)=\operatorname{sat}(3x/5)$。）

> **MPC = 约束 LQR 的滚动实现（综合时写盒）**：去掉约束、$N\to\infty$，则 $\kappa_N\to$ 无约束 LQR 律 $Kx$（§3.5）。Example 2.5 中 $3/5$ 恰是该标量系统的无穷时域 LQR 增益（$A=B=Q=R=1$ 的 DARE 解 $\Pi=(1+\sqrt5)/2$，$K=-\Pi/(1+\Pi)=\dots$，给斜率 $3/5$ 的近似/有限时域版）。**这是"MPC 在内部、约束不激活时退化为 LQR"的最小可见证据。**

## §12 MPC 的数值结构：稀疏 KKT、Riccati 递推、Condensing（[RMD] §8.8，逐式 8.47–8.55）

> MPC 在线求解的 QP 有特殊块带结构。三条主线：simultaneous/稀疏、Riccati 递推、condensing/凝聚。

### §12.1 时变 LQ 问题 (LQP) 的稀疏 KKT（[RMD] §8.8.1–8.8.2，式 8.47–8.50）

**LQP（[RMD] 式 8.50，Newton 型法每步求解的子问题，或线性 MPC 本身）**：
$$\min_{\mathbf x,\mathbf u}\ \sum_{i=0}^{N-1}\begin{bmatrix}\bar q_i\\\bar r_i\end{bmatrix}^\top\begin{bmatrix}x_i\\u_i\end{bmatrix}+\tfrac12\begin{bmatrix}x_i\\u_i\end{bmatrix}^\top\begin{bmatrix}\bar Q_i&\bar S_i^\top\\\bar S_i&\bar R_i\end{bmatrix}\begin{bmatrix}x_i\\u_i\end{bmatrix}+\bar p_N^\top x_N+\tfrac12 x_N^\top\bar P_N x_N$$
$$\text{s.t.}\quad \bar x_0-x_0=0,\qquad \bar b_i+\bar A_ix_i+\bar B_iu_i-x_{i+1}=0,\ i=0,\dots,N-1.\tag{8.50}$$
（横杠表固定量；$\bar S_i$=状态-输入交叉项。）

**块带 KKT 系统（[RMD] 式 8.48–8.49）**：把原-对偶变量排成 $z=(\lambda_0,x_0,u_0,\dots,\lambda_N,x_N)$，则一步 LQ 的解对应块带线性系统
$$\bar M_{\text{KKT}}\,z=\bar r_{\text{KKT}},$$
$$\bar M_{\text{KKT}}=\begin{bmatrix}0&I&&&&&\\I&\bar Q_0&\bar S_0^\top&\bar A_0^\top&&&\\&\bar S_0&\bar R_0&\bar B_0^\top&&&\\&\bar A_0&\bar B_0&0&-I&&\\&&&-I&\ddots&&\\&&&&&\ddots&\\&&&&&&\bar P_N\end{bmatrix}.\tag{8.49}$$
忽略块结构，这是**对称带状矩阵**，带宽 $2n+m$、总尺寸 $N(2n+m)+2n$，可用带状 $LDL^\top$ 分解求解，**代价随时域 $N$ 线性、随 $(2n+m)$ 三次**。

### §12.2 LQP 经 Riccati 递推求解（[RMD] §8.8.3，式 8.51–8.54）

> 时变版 Riccati，三条递推（一矩阵 + 两向量）。

**第一条（反向矩阵递推）**：$P_N:=\bar P_N$，$i=N-1,\dots,0$：
$$\boxed{\ P_i:=\bar Q_i+\bar A_i^\top P_{i+1}\bar A_i-\big(\bar S_i^\top+\bar A_i^\top P_{i+1}\bar B_i\big)\big(\bar R_i+\bar B_i^\top P_{i+1}\bar B_i\big)^{-1}\big(\bar S_i+\bar B_i^\top P_{i+1}\bar A_i\big).\ }\tag{8.51}$$
良定条件：$\bar R_i+\bar B_i^\top P_{i+1}\bar B_i\succ0$（等价于优化问题良态，否则 (8.50) 下无界）。$P_i$ 对称（应利用）。

**第二条（反向向量递推）**：$p_N:=\bar p_N$，$i=N-1,\dots,0$：
$$p_i:=\bar q_i+\bar A_i^\top P_{i+1}\bar b_i+\bar A_i^\top p_{i+1}-\big(\bar S_i^\top+\bar A_i^\top P_{i+1}\bar B_i\big)\big(\bar R_i+\bar B_i^\top P_{i+1}\bar B_i\big)^{-1}\big(\bar s_i+\bar B_i^\top(P_{i+1}\bar b_i+p_{i+1})\big).\tag{8.52}$$
两条递推合起来给最优 cost-to-go $V_i^0(x_i)=c_i+p_i^\top x_i+\tfrac12 x_i^\top P_i x_i$，与最优反馈律 $u_i^0(x_i)=k_i+K_ix_i$：
$$K_i:=-\big(\bar R_i+\bar B_i^\top P_{i+1}\bar B_i\big)^{-1}\big(\bar S_i+\bar B_i^\top P_{i+1}\bar A_i\big),\tag{8.53a}$$
$$k_i:=-\big(\bar R_i+\bar B_i^\top P_{i+1}\bar B_i\big)^{-1}\big(\bar s_i+\bar B_i^\top(P_{i+1}\bar b_i+p_{i+1})\big).\tag{8.53b}$$

**第三条（前向递推 = 用最优律前向仿真）**：$x_0:=\bar x_0$，$i=0,\dots,N-1$：
$$u_i:=k_i+K_ix_i,\tag{8.54a}$$
$$x_{i+1}:=\bar b_i+\bar A_ix_i+\bar B_iu_i,\tag{8.54b}$$
（同时算 Lagrange 乘子）$\lambda_i:=p_i+P_ix_i,\ i=0,\dots,N.\tag{8.54c}$

**复杂度（[RMD]）**：矩阵递推 (8.51) 可解释为 $\bar M_{\text{KKT}}$ 的因子分解，约 $N(\tfrac73 n^3+4n^2m+2nm^2+\tfrac13 m^3)$ FLOPs，约为带状 $LDL^\top$ 的 1/3；向量递推约 $N(8n^2+8nm+2n)$ FLOPs。**只需 $\bar R_i+\bar B_i^\top P_{i+1}\bar B_i\succ0$。** 稍加改造可嵌入内点法处理不等式约束。

> **与 §3.3 的一致性**：取 $\bar A_i=A,\bar B_i=B,\bar Q_i=Q,\bar R_i=R,\bar S_i=0,\bar b_i=0$（时不变、无仿射、无交叉），(8.51) $\to$ 紧凑 Riccati (1.10)，(8.53a) $\to$ (1.13)。即 **LQR 是 LQP-Riccati 的时不变特例**。

### §12.3 Condensing（凝聚/批量消元，[RMD] §8.8.4，式 8.55）

**消去状态轨迹**：把 $\mathbf x=(x_0,\dots,x_N)$ 表为初值 $\bar x_0$ 与控制 $\mathbf u=(u_0,\dots,u_{N-1})$ 的函数。等式约束写为（略去横杠）
$$\underbrace{\begin{bmatrix}I&&&\\-A_0&I&&\\&\ddots&\ddots&\\&&-A_{N-1}&I\end{bmatrix}}_{\mathcal A}\mathbf x=\underbrace{\begin{bmatrix}0\\b_0\\\vdots\\b_{N-1}\end{bmatrix}}_{\mathbf b}+\underbrace{\begin{bmatrix}I\\0\\\vdots\\0\end{bmatrix}}_{\mathcal I}\bar x_0+\underbrace{\begin{bmatrix}0&&\\B_0&&\\&\ddots&\\&&B_{N-1}\end{bmatrix}}_{\mathcal B}\mathbf u.$$
$\mathcal A$ 的逆（下三角，[RMD] 给）
$$\mathcal A^{-1}=\begin{bmatrix}I&&&\\A_0&I&&\\A_1A_0&A_1&I&\\\vdots&\vdots&\ddots&\ddots\\A_{N-1}\cdots A_0&A_{N-1}\cdots A_1&\cdots&A_{N-1}&I\end{bmatrix},$$
**状态消元的仿射映射**：
$$\boxed{\ \mathbf x=\mathcal A^{-1}\mathbf b+\mathcal A^{-1}\mathcal I\,\bar x_0+\mathcal A^{-1}\mathcal B\,\mathbf u.\ }$$
代入目标消去全部状态，得**凝聚的无约束 QP**（[RMD] 式 8.55）
$$\min_{\mathbf u}\ c+\begin{bmatrix}\bar q\\\bar r\end{bmatrix}^\top\begin{bmatrix}\bar x\\\mathbf u\end{bmatrix}+\tfrac12\begin{bmatrix}\bar x\\\mathbf u\end{bmatrix}^\top\begin{bmatrix}\bar Q&\bar S^\top\\\bar S&\bar R\end{bmatrix}\begin{bmatrix}\bar x\\\mathbf u\end{bmatrix},\tag{8.55}$$
等价于原 (8.50)。**凝聚后无约束解**：
$$\boxed{\ \mathbf u^0=-\bar R^{-1}(\bar r+\bar S\bar x).\ }$$
Hessian $\bar R$ 稠密对称（通常正定），尺寸 $Nm$，Cholesky 分解约 $\tfrac13 N^3m^3$ FLOPs。**凝聚算法**把稀疏 (8.50) 处理成 (8.55)：经典变体约 $\tfrac13 N^3nm^2$ FLOPs；另一变体（对二次代价逆向 AD）约 $N^2(2n^2m+nm^2)$ FLOPs（[RMD] 引 Frison 2015）。

> **Riccati（稀疏）vs Condensing（稠密）选择（[RMD] §8.8.5）**：Riccati 递推（块带）随 $N$ **线性**，长时域首选、易嵌入带结构内点/正定 QP 解法；Condensing 把问题变量降到 $Nm$、Hessian 稠密，**短 $N$、$n\gg m$** 时更优。两者可组合（**partial condensing**，部分凝聚）兼得优点。Condensing 类同 sequential 法（先消状态）。

---

# 第四部分：面向机器人的控制（逆动力学 / WBC 概览）`\rebuilt`

> 本部分服务"概览"，个人笔记未同步，整体 `\rebuilt`，据权威源（[Wiki-CTC]、Sentis-Khatib OSC、Kuindersma et al. Atlas QP-WBC 等）抽取。机器人动力学的**完整推导**（Lagrange/Newton-Euler 得 $M,C,g$）属"动力学"章，本章只取其用于控制的形式。

## §13 机器人动力学方程（控制所需形式）`\rebuilt`

$n$ 自由度刚体机械臂的**关节空间动力学**（Lagrange 形式）：
$$\boxed{\ \mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\mathbf g(\mathbf q)=\boldsymbol\tau,\ }$$
$\mathbf q\in\mathbb R^n$ 关节位形、$\mathbf M(\mathbf q)\succ0$ 惯性矩阵（对称正定）、$\mathbf C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}$ 科氏/离心项、$\mathbf g(\mathbf q)$ 重力项、$\boldsymbol\tau$ 关节力矩输入。（[Wiki-CTC] 记 $\mathbf M(\vec\theta)\ddot{\vec\theta}+\mathbf C(\vec\theta,\dot{\vec\theta})\dot{\vec\theta}+\vec\tau_g(\vec\theta)=\vec\tau$，同一物理量。含接触/外力时右端加 $\mathbf J^\top\boldsymbol\lambda$。）

**重要性质（控制用）**：(i) $\mathbf M\succ0$ 故可逆；(ii) $\dot{\mathbf M}-2\mathbf C$ 反对称（用恰当 $\mathbf C$ 因子化时），常用于 Lyapunov 证稳；(iii) 线性于动力学参数（用于自适应控制）。

## §14 计算力矩控制 / 逆动力学控制（反馈线性化）[Wiki-CTC] + `\rebuilt`

### §14.1 控制律（[Wiki-CTC]，含积分项，逐字恢复）
**计算力矩控制律**（误差 $\vec\theta_e=\vec\theta_d-\vec\theta$，$\tilde\cdot$ 表模型估计）：
$$\boxed{\ \boldsymbol\tau=\tilde{\mathbf M}(\mathbf q)\Big(\ddot{\mathbf q}_d+K_p\,\mathbf e+K_i\!\int_0^t\!\mathbf e\,\mathrm dt'+K_d\,\dot{\mathbf e}\Big)+\tilde{\mathbf C}(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\tilde{\mathbf g}(\mathbf q),\ }$$
其中 $\mathbf e=\mathbf q_d-\mathbf q$。（[Wiki-CTC] 原式：$\vec\tau=\tilde{\mathbf M}(\vec\theta)(\ddot{\vec\theta}_d+K_p\vec\theta_e+K_i\int_0^t\ddot{\vec\theta}_e\,\mathrm dt'+K_d\dot{\vec\theta}_e)+\tilde{\mathbf C}(\vec\theta,\dot{\vec\theta})+\tilde\tau_g(\vec\theta)$；即**逆动力学前馈 + PID 误差反馈**。）

### §14.2 闭环误差动力学（[Wiki-CTC]，反馈线性化）
**理想（模型精确 $\tilde\cdot=\cdot$）时**，把控制律代入动力学、左乘 $\mathbf M^{-1}$，非线性项 $\mathbf C,\mathbf g$ **抵消**，得**线性闭环误差动力学**：
$$\boxed{\ \ddot{\mathbf e}+K_d\,\dot{\mathbf e}+K_p\,\mathbf e\ (+\,K_i\!\int\mathbf e)=0.\ }$$
（[Wiki-CTC]：$0=\ddot{\vec\theta}_e+K_p\vec\theta_e+K_i\int_0^t\ddot{\vec\theta}_e\,\mathrm dt'+K_d\dot{\vec\theta}_e$。）**稳定性**：选 $K_p,K_d(,K_i)$ 使该（线性常系数）误差方程 Hurwitz（如 $K_p,K_d\succ0$ 对角，等价 $n$ 个解耦二阶系统 $\ddot e_j+k_{d,j}\dot e_j+k_{p,j}e_j=0$，临界阻尼取 $k_d=2\sqrt{k_p}$），则 $\mathbf e\to0$ **指数收敛**。

> **"反馈线性化"之名（[Wiki-CTC]）**：内环用动力学模型**抵消机器人非线性动力学**，留下可由线性技术（PID）控制的残余**线性**误差动力学。即便关节角误差为零，计算力矩控制器也用规划加速度 $\ddot{\mathbf q}_d$ 产生非零前馈力矩。结构：**内环（逆动力学抵消）+ 外环（线性误差反馈）**。

> **与 LQR/MPC 衔接（综合写盒）**：反馈线性化把非线性机器人化为"双积分器" $\ddot{\mathbf e}=\mathbf v$（$\mathbf v$=外环虚拟输入），其上可设计 LQR（§4，连续双积分器的 LQR 给 PD 增益）或 MPC（§8，加关节限位/力矩约束）。这是把第二、三部分用于机器人的标准桥梁。

## §15 整体控制 WBC（Whole-Body Control）概览 `\rebuilt`

> 浮动基人形/腿足机器人的"力矩分解层"。源：Sentis & Khatib（操作空间 OSC）、Kuindersma et al. *Optimization-based locomotion planning, estimation, and control design for the Atlas humanoid robot*（Auton. Robots 2016）、Wensing 等综述。

### §15.1 浮动基动力学
浮动基系统位形 $\mathbf q=(\mathbf q_b,\mathbf q_j)$（基座 6 DoF + 关节 $n_j$），动力学
$$\mathbf M(\mathbf q)\ddot{\mathbf q}+\mathbf h(\mathbf q,\dot{\mathbf q})=\mathbf S^\top\boldsymbol\tau+\mathbf J_c^\top\boldsymbol\lambda,$$
$\mathbf h=\mathbf C\dot{\mathbf q}+\mathbf g$ 非线性项、$\mathbf S=[\mathbf 0\ \mathbf I_{n_j}]$ 选择矩阵（基座不可直接驱动，**欠驱动**前 6 行）、$\mathbf J_c$ 接触雅可比、$\boldsymbol\lambda$ 接触力。

### §15.2 优先级任务与 QP-WBC 标准形
**任务**：操作空间目标 $\ddot{\mathbf x}_i=\mathbf J_i\ddot{\mathbf q}+\dot{\mathbf J}_i\dot{\mathbf q}$（如质心 CoM 轨迹、足端位姿、末端执行器位姿、姿态）。期望任务加速度由各自 PD 给：$\ddot{\mathbf x}_i^{\text{des}}=\ddot{\mathbf x}_i^{\text{ref}}+K_{p,i}(\mathbf x_i^{\text{ref}}-\mathbf x_i)+K_{d,i}(\dot{\mathbf x}_i^{\text{ref}}-\dot{\mathbf x}_i)$。

**QP-WBC（决策变量 $\ddot{\mathbf q},\boldsymbol\tau,\boldsymbol\lambda$，标准形）**：
$$\min_{\ddot{\mathbf q},\boldsymbol\tau,\boldsymbol\lambda}\ \sum_i w_i\big\|\mathbf J_i\ddot{\mathbf q}+\dot{\mathbf J}_i\dot{\mathbf q}-\ddot{\mathbf x}_i^{\text{des}}\big\|^2\ (+\ \text{正则项})$$
$$\text{s.t.}\quad \mathbf M\ddot{\mathbf q}+\mathbf h=\mathbf S^\top\boldsymbol\tau+\mathbf J_c^\top\boldsymbol\lambda\ (\text{全身动力学}),$$
$$\mathbf J_c\ddot{\mathbf q}+\dot{\mathbf J}_c\dot{\mathbf q}=\mathbf 0\ (\text{接触不滑动}),\quad \boldsymbol\lambda\in\mathcal{FC}\ (\text{摩擦锥}),\quad \boldsymbol\tau_{\min}\le\boldsymbol\tau\le\boldsymbol\tau_{\max}.$$
即在**全身动力学 + 接触 + 摩擦锥 + 力矩限**约束下，同时（加权或严格优先级）满足平衡、足放置、末端执行器多目标的单一**QP**。摩擦锥常线性化为多面体（金字塔）使之成 QP。

**两种优先级实现**：(i) **加权（soft）**：上式权重 $w_i$ 软优先；(ii) **严格分层（strict hierarchy）**：逐级零空间投影或级联 QP（高优先任务的零空间内解低优先任务），保证高优先严格不被低优先牺牲。

> **WBC 与 MPC 的分工（综合写盒）**：腿足机器人典型双层——**上层 MPC**（§8，常用质心/单刚体或 LIP 简化模型 + 接触力规划，时域 ~1s）给出参考质心轨迹与接触力；**下层 WBC/QP**（本节，全身动力学，~1kHz）把上层参考分解为关节力矩。WBC 是"瞬时（1 步）逆动力学 QP"，MPC 是"多步预测 QP"——二者都是 §12 意义下的 QP，只是模型与时域不同。

---

## §16 给本书"控制导论"章的综合建议（清单）

1. **主线编排**：反馈/稳定性（§B.1–B.2）→ PID（§B.3）→ 状态空间 LQR（离散 DP §1 + 连续 HJB §4 + 协态 §5 + 无穷时域/DARE/收敛 §2–§3 + 四式速查 §6）→ MPC（原理 §7 + 表述 §8–§9' + 稳定性 §10 + 例 §11 + 数值 §12）→ 机器人控制（§13–§15）。
2. **LQR 推导取舍**：建议正文**主推离散 DP（§1 [Bansal] 含完整归纳证明，最干净）**，连续 HJB（§4）作平行小节，协态法（§5）作"另一视角"盒，Hamiltonian 矩阵求解（§5.3）入算法/附录。**务必收 Kalman 反例（§3.4）**——它是"为何要无穷时域/为何 MPC 需终端代价"的最佳动机，且带具体数值。
3. **统一记号**：按 §0.1 表——离散 $\mathbf P_k$、连续 $\mathbf S(t)$、$\mathbf u=-\mathbf K\mathbf x$（$\mathbf K$ 不含负号）、代价不带 $\tfrac12$、交叉项记 $\mathbf S_{xu}$（避与时域 $N$ 撞）。抄 [RMD] 公式时**注意其 $K_k$ 含负号、代价带 $\tfrac12$**。
4. **MPC 稳定性**：以 **Theorem 2.19** 为高潮，前置 Assumption 2.14（基本稳定性假设，含终端代价下降 CLF 条件）+ 单调性 Prop 2.18（完整证明已抄）+ 下降性 (2.17)。强调"**最优 ≠ 稳定 → 终端 ingredients 修复**"主线。指数稳定（§10.6）作推论。
5. **"LQR–MPC 同源"盒**：用 Example 2.5 的 $\kappa_N(x)=\operatorname{sat}(3x/5)$ 展示"约束不激活时 MPC=LQR"；用 §12.2 的 (8.51)$\to$(1.10) 展示"LQR=LQP-Riccati 时不变特例"；用终端代价=LQR cost-to-go 展示"MPC 的终端域内就是 LQR"。三处闭环。
6. **数值实现**：§12 的稀疏 KKT / Riccati 递推 / Condensing 三法对比表（复杂度 + 适用场景）入"工程实现"小节或附录；提 OSQP/qpOASES/acados/HPIPM 等求解器（综合时补当代工具引用）。
7. **机器人控制**：§14 计算力矩（反馈线性化 → 双积分器 → 上接 LQR/MPC）作主桥梁；§15 WBC 作"当代腿足/人形"概览，明确"MPC（上层、简化模型）+ WBC（下层、全身 QP）"分层。整段保留 `\rebuilt` 标记并补当代论文引用（Sentis-Khatib 2005；Kuindersma et al. 2016；Di Carlo et al. MIT Cheetah convex MPC 2018 等）。
8. **未尽/待综合二次核对项**：(i) Example 1.1(a) 常数 $d$ 的符号（[RMD] OCR 含糊，已标"写法待校"，综合时回原书 p.15–17 校）；(ii) §5.1–§5.2 协态法的 $\tfrac12$/因子-2 约定（本抽取按"代价带 $\tfrac12$、$\lambda=Px$"给，若正文不带 $\tfrac12$ 则 $\lambda=2Px$）；(iii) §6 [Wiki] 交叉项 $N$ 与时域 $N$ 撞名，正文务必改名；(iv) WBC 的严格分层投影公式（递归零空间）本抽取仅给条目，综合时若需可补 Sentis 论文闭式。
