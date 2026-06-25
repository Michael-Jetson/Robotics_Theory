# 抽取留痕 · 控制导论：反馈控制基础、PID、状态空间、稳定性(Lyapunov)、可控可观

> **本文件性质**：项目内部「抽取留痕」（非成书正文）。目标是把权威控制理论源材料**全量保真**抽取，供综合 agent 写成自包含书章《控制导论》。
>
> **\rebuilt 总声明**：本章对应的个人笔记（控制部分）**尚未同步**，故本抽取**完全基于权威公开教材/论文/官方百科**，逐条注明出处。综合写入正文时，所有源自此处的内容请标 `\rebuilt`（文案"重建·待核对："），并在用户回填个人笔记后二次核对。
>
> **铁律遵循**：每一步推导（中间代数不跳）、每一道例题与数值例、每一条定义/定理/引理 + 完整证明、每一张表/算法伪码均完整记录，绝不凝练。公式全部 LaTeX，标源小节号，保留编号与条件。

---

## §0 记号约定与本书统一约定的差异（务必先读）

本抽取跨多个源，控制论领域的记号与本书（SLAM/机器人状态估计向，约定 $R\in SO(3)$、右扰动为主、$\xi=[\rho;\phi]$、Hamilton 四元数、协方差用 $P$/$\Sigma$）有若干**冲突**，综合时必须翻译。下表逐项对照。

| 概念 | 控制文献常用记号（本抽取沿用） | 本书既有记号 | 冲突与转换提示 |
|---|---|---|---|
| 状态 | $\mathbf x\in\mathbb R^n$ | 同（状态估计章亦用 $\mathbf x$） | 一致。 |
| 控制输入 | $\mathbf u\in\mathbb R^m$ | — | 引入即可，无冲突。 |
| 系统矩阵 | $A$（连续 $\dot{\mathbf x}=A\mathbf x+B\mathbf u$；离散 $\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k$） | KF 章离散转移矩阵常记 $F$（或 $\Phi$），输入矩阵 $B$/$G$ | **冲突**：本书 KF 章用 $F=\partial f/\partial x$ 作离散状态转移；控制章用 $A$。综合时建议沿用控制惯例 $A,B,C,D$ 并在章首声明"$A$ 即 KF 章的 $F$（连续时为雅可比，离散时为转移矩阵）"。 |
| 输出/观测矩阵 | $C$（$\mathbf y=C\mathbf x+D\mathbf u$） | KF 章观测雅可比记 $H$ | **冲突**：控制 $C$ = KF 章 $H$。综合时声明对应关系。 |
| **协方差/Riccati 解矩阵** | LQR 的 Riccati 解记 $P$（Wikipedia/Hespanha）或 $S$（Tedrake）；MPC 终端权重记 $P$ | **本书 $P$ 是协方差矩阵！** | **最严重冲突**。LQR 的 $P$（代价矩阵，正定，满足 ARE）与 KF/ESKF 的协方差 $P$ 同字母但语义相反（一个是代价 Hessian，一个是不确定度）。**强烈建议**综合时 LQR Riccati 解改记 $S$（如 Tedrake），把字母 $P$ 留给协方差；或至少在 LQR 节显式声明"此 $P$ 非协方差"。本抽取在 LQR 主线统一用 $S$（Tedrake 风格），在直接引 Wikipedia/Riccati 公式处保留 $P$ 并注明 $P\equiv S$。 |
| 代价权重 | 状态权 $Q\succeq0$、输入权 $R\succ0$、交叉权 $N$、终端权 $Q_f$/$P$/$S(t_f)$ | KF 章过程噪声协方差 $Q$、量测噪声协方差 $R$ | **冲突**：LQR 的 $Q,R$（代价权重）与 KF 的 $Q,R$（噪声协方差）同字母异义。这是控制/估计对偶性的"巧合"（LQG 中确实对偶），但语义不同。综合时务必分节声明。 |
| Lyapunov 方程矩阵 | $A^\top M+MA=-Q$（$M$ 正定解，$V=\mathbf x^\top M\mathbf x$）；亦有人写 $A^\top P+PA=-Q$ | — | 引入即可。注意此 $Q$ 是给定正定阵，与上面代价权 $Q$、噪声 $Q$ 又是另一个 $Q$，需上下文区分。 |
| 误差 | 控制误差 $e=r-y$（参考减输出，Åström-Murray）；跟踪误差 $\boldsymbol\theta_e=\boldsymbol\theta_d-\boldsymbol\theta$ | 状态估计常用 $\delta\mathbf x=\hat{\mathbf x}\boxminus\mathbf x$ | 控制 $e$ 取"期望减实际"号；与本书估计误差号约定不同，转换时注意符号。 |
| 旋转/位姿 | 本章基本不涉及 $SO(3)$（标量/线性系统为主）；机器人控制节用关节坐标 $\mathbf q$ | $R\in SO(3)$、$T\in SE(3)$、$\xi=[\rho;\phi]$ | 机器人控制节（逆动力学/WBC）以广义坐标 $\mathbf q$（关节角）为状态，任务空间 $\mathbf x$ 经雅可比 $\dot{\mathbf x}=J\dot{\mathbf q}$ 关联；位姿层面与本书 $SE(3)$ 约定相容（综合时任务空间姿态误差可用本书右扰动/对数映射表达）。 |
| 四元数 | 本章不涉及 | Hamilton | 无冲突。 |

**源清单（每条结论的出处在正文逐处标注）**：
- **S1 = Åström & Murray, _Feedback Systems: An Introduction for Scientists and Engineers_, 2nd ed. (v3.1.5, 2020)，Princeton Univ. Press**。免费电子版 http://fbsbook.org 。本抽取精读其 **Chapter 11 "PID Control"（全 27 页，§11.1–§11.6 + 习题）**。下载：`https://www.cds.caltech.edu/~murray/books/AM08/pdf/fbs-pid_24Jul2020.pdf`。
- **S2 = Wikipedia, "Linear–quadratic regulator"**，https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator （LQR 有限/无限时域、连续/离散、CARE/DARE 全套公式）。
- **S3 = Wikipedia, "Lyapunov stability"**，https://en.wikipedia.org/wiki/Lyapunov_stability （稳定性定义、直接法、间接法、Lyapunov 方程、Barbalat）。
- **S4 = Wikipedia, "LaSalle's invariance principle"**，https://en.wikipedia.org/wiki/LaSalle%27s_invariance_principle 。
- **S5 = Wikipedia, "Controllability"** & **"Observability"**，https://en.wikipedia.org/wiki/Controllability ，https://en.wikipedia.org/wiki/Observability （Kalman 秩判据、Gramian、PBH、对偶）。
- **S6 = Wikipedia, "Algebraic Riccati equation"** & **"Riccati equation"**，https://en.wikipedia.org/wiki/Algebraic_Riccati_equation ，https://en.wikipedia.org/wiki/Riccati_equation （Hamilton 矩阵/辛矩阵求解、Riccati 降阶）。
- **S7 = Russ Tedrake, _Underactuated Robotics_ (MIT 6.832 课程讲义, online)**，https://underactuated.mit.edu/lqr.html ，https://underactuated.mit.edu/dp.html （LQR 经由 HJB 的复现级推导、动态规划 Bellman 递推）。
- **S8 = Wikipedia, "Hamilton–Jacobi–Bellman equation"**，https://en.wikipedia.org/wiki/Hamilton%E2%80%93Jacobi%E2%80%93Bellman_equation 。
- **S9 = Wikipedia, "Model predictive control"**，https://en.wikipedia.org/wiki/Model_predictive_control （MPC 问题形式、滚动时域）。
- **S10 = Borrelli, Bemporad, Morari, _Predictive Control for Linear and Hybrid Systems_, Cambridge 2017**，免费稿 `https://cse.lab.imtlucca.it/~bemporad/publications/papers/BBMbook.pdf` 。本抽取精读其 **Chapter 12 "Receding Horizon Control"**（持久可行性 Lemma 12.1/12.2、稳定性 **Theorem 12.2 + 完整证明**）。
- **S11 = Wikipedia, "Computed torque control"**，https://en.wikipedia.org/wiki/Computed_torque_control （计算力矩/逆动力学控制）。
- **S12 = Sentis & Khatib, "A Whole-Body Control Framework for Humanoids Operating in Human Environments", ICRA 2006**，https://khatib.stanford.edu/publications/pdfs/Sentis_2006_ICRA.pdf （操作空间公式、优先级零空间投影、WBC）。
- **S13 = Wikipedia, "Impedance control"**，https://en.wikipedia.org/wiki/Impedance_control 。
- 参考定位（未逐字抽取，仅作权威指引）：Khalil _Nonlinear Systems_ 3rd（Lyapunov 标准教材）；Mayne, Rawlings, Rao, Scokaert, "Constrained model predictive control: Stability and optimality", _Automatica_ 36(6):789–814, 2000（MPC 稳定性奠基）；Ziegler & Nichols, "Optimum settings for automatic controllers", _Trans. ASME_ 64:759–768, 1942（Z-N 原始论文，经 S1 §11.3 转引）。

---

# 第一部分 反馈控制基础

## §1.1 反馈控制的基本结构（综合 S1, S9 背景）

**对象 (plant)**：被控系统，传递函数 $P(s)$（SISO）或状态空间 $\dot{\mathbf x}=A\mathbf x+B\mathbf u,\ \mathbf y=C\mathbf x+D\mathbf u$。

**控制器 (controller)**：$C(s)$，由误差或测量生成控制信号 $u$。

**闭环 (closed loop)**：S1 §11.1 给出 PID 闭环框图（Fig. 11.1）。误差反馈型：单输入误差 $e=r-y$（$r$ 参考/设定点，$y$ 过程输出），控制 $u$ 全由 $e$ 生成。两自由度型：积分作用于误差 $e$，比例与微分仅作用于输出 $y$（避免参考阶跃对微分项造成的冲击）。

**参考-误差传递函数（S1 式 11.2）**：纯比例反馈 $C(s)=k_p$ 时
$$G_{er}(s)=\frac{1}{1+C(s)P(s)}=\frac{1}{1+k_pP(s)}.$$
单位阶跃稳态误差（设闭环稳定）：
$$G_{er}(0)=\frac{1}{1+C(0)P(0)}=\frac{1}{1+k_pP(0)}.$$
**数值例（S1 §11.1，对象 $P(s)=1/(s+1)^3$）**：$k_p=1,2,5$ 时稳态误差分别为 $0.5,\ 0.33,\ 0.17$；增益增大误差减小但系统更振荡，$k_p=8$ 时**失稳**。注意控制信号初值等于控制器增益。

## §1.2 反馈消除稳态误差与积分作用的"魔力"（S1 §11.1，含完整证明）

**前馈消差（S1 式 11.3）**：把比例项改为
$$u(t)=k_pe(t)+u_{\rm ff},$$
取 $u_{\rm ff}=r/P(0)=k_fr$，则常参考下稳态输出恰等于参考（需精确已知零频增益 $P(0)$，通常不可得）。这是前馈消差的局限。

**积分作用消差（"the magic of integral action"）——一般性证明（S1 §11.1）**：考虑含积分（$k_i\neq0$）的控制器
$$u=k_pe+k_i\int_0^t e(\tau)\,d\tau+k_d\frac{de}{dt}.$$
设 $u(t)\to u_0$、$e(t)\to e_0$ 收敛到稳态。由控制律取极限：
$$u_0=k_pe_0+k_i\lim_{t\to\infty}\int_0^t e(t)\,dt.$$
右端积分项除非 $e(t)\to0$ 否则发散为无穷，而 $u_0$ 有限，故必有 $e_0=0$。**结论**：只要存在稳态，积分控制使误差恒为零。**注**：此证明**未假设系统线性或时不变**，只假设存在平衡点。比前馈优越（不需精确模型参数）。

**频域视角（S1 式 11.4）**：PID 传递函数
$$C(s)=k_p+\frac{k_i}{s}+k_ds,$$
零频增益 $C(0)=\infty$，故由式 11.2 得 $G_{er}(0)=0$，阶跃无稳态误差。

**自动复位 (automatic reset)（S1 §11.1, Fig. 11.3a）**：积分作用可视为自动生成前馈项 $u_{\rm ff}$。把控制器输出经一阶低通正反馈：
$$G_{ue}=k_p\frac{1+sT_i}{sT_i}=k_p+\frac{k_p}{sT_i},$$
正是 PI 控制器的传递函数（历史上低通比积分器易实现）。

**积分增益与负载扰动抑制（S1 §11.1）**：稳定闭环初始静止，在过程输入加单位阶跃负载扰动，瞬态后 $e(t)\to0$，由控制律得
$$u(\infty)=k_i\int_0^\infty e(t)\,dt.$$
单位阶跃扰动的**积分误差** $\mathrm{IE}=\int_0^\infty e(t)\,dt$ 与 $k_i$ **成反比**，故 $k_i$ 是扰动抑制有效性的度量。$k_i$ 大则抑制好，但过大致振荡、鲁棒性差乃至失稳。

---

# 第二部分 PID 控制（主源 S1 = Åström–Murray Ch.11，全量）

## §2.1 PID 的三项与两种参数化（S1 §11.1, 式 11.1）

**理想 PID（误差反馈）输入输出关系（S1 式 11.1）**：
$$\boxed{u=k_pe+k_i\int_0^t e(\tau)\,d\tau+k_d\frac{de}{dt}=k_p\!\left(e+\frac{1}{T_i}\int_0^t e(\tau)\,d\tau+T_d\frac{de}{dt}\right).}$$
三项：比例反馈（现在的误差）、积分项（过去的误差）、微分项（预测未来误差）。历史上称"三项控制器 (three-term controller)"。

**参数**：比例增益 $k_p$、积分增益 $k_i$、微分增益 $k_d$。等价参数化：
- **比例带 (proportional band)**：$\mathrm{PB}=100/k_p$（%）。PB=10% 表示控制器仅在测量量程的 10% 内线性工作。
- **积分时间 (integral time)**：$T_i=k_p/k_i$。
- **微分时间 (derivative time)**：$T_d=k_d/k_p$。
$T_i,T_d$ 量纲为时间，可与控制器时间常数自然关联。

**各项直觉与定性影响（S1 §11.1 正文 + 通用控制结论，参 S11 背景）**：
- **P**：减小上升时间、减小稳态误差，但增大超调、过大致振荡/失稳。
- **I**：消除稳态误差（见 §1.2 证明），但增大超调与调节时间，过大致振荡。
- **D**：提供预测/阻尼，减小超调与调节时间、改善稳定性（小增益时），但放大高频测量噪声，对稳态误差无影响。

> **定性影响表（通用控制工程结论，源 S 综合；S1 以文字给出，Wikipedia PID 页给出表格形式）**：
>
> | 参数增大 | 上升时间 | 超调 | 调节时间 | 稳态误差 | 稳定性 |
> |---|---|---|---|---|---|
> | $k_p\uparrow$ | 减小 | 增大 | 小变化 | 减小 | 变差 |
> | $k_i\uparrow$ | 减小 | 增大 | 增大 | 消除 | 变差 |
> | $k_d\uparrow$ | 微变 | 减小 | 减小 | 无影响 | 改善（小增益时） |
>
> （此表为定性趋势，各参数耦合，非独立——综合时务必加此免责说明。）

## §2.2 微分作为预测；微分项的滤波实现（S1 §11.1, 式 11.5）

**微分=线性外推预测（S1 §11.1）**：比例+微分合写为
$$u=k_pe+k_d\frac{de}{dt}=k_p\!\left(e+T_d\frac{de}{dt}\right)=:k_pe_p,$$
其中 $e_p(t)$ 是误差在时刻 $t+T_d$ 的**线性外推预测**，预测时间 $T_d=k_d/k_p$ 即微分时间。

**滤波微分（S1 式 11.5, Fig. 11.3b）**：理想微分对高频增益无穷，放大噪声。用"信号减其低通版本"实现：
$$G_{ue}(s)=k_p\!\left(1-\frac{1}{1+sT_d}\right)=k_p\frac{sT_d}{1+sT_d}=\frac{k_ds}{1+sT_d}.$$
低频（$|s|\ll1/T_d$）时 $G(s)\approx k_pT_ds=k_ds$（近似微分），高频时为常增益 $k_p$（不放大噪声），故称**滤波微分**。

**数值例（S1 §11.1, Fig. 11.2, 对象 $P(s)=1/(s+1)^3$）**：PID 取 $k_p=2.5,\ k_i=1.5,\ k_d=0,1,2,4$。无微分时振荡，微分增大则阻尼增强。阶跃参考下微分项产生**冲击 (impulse)**（可由两自由度结构 Fig. 11.1b 避免）。

## §2.3 例题：视网膜中的 PD 作用（S1 Example 11.1，全量）

**Example 11.1（视网膜 PD 作用）**：视锥光感受器（cones, C）受光刺激，刺激水平细胞（horizontal cells, H），H 对 C 施加抑制性（负）反馈。微分方程模型（[Wil99]）：
$$\frac{dx_1}{dt}=\frac{1}{T_c}\left(-x_1-kx_2+u\right),\qquad \frac{dx_2}{dt}=\frac{1}{T_h}\left(x_1-x_2\right),$$
$u$ 为光强，$x_1,x_2$ 为视锥与水平细胞的平均脉冲率。阶跃响应呈现"大初始响应 + 较低恒定稳态响应"，即 PD 特征。仿真参数 $k=4,\ T_c=0.025,\ T_h=0.08$。$\nabla$

## §2.4 用低阶模型设计简单控制器（S1 §11.2，全量含数值例）

**模型阶数原则（S1 §11.2）**：PID 适用一阶/二阶模型。任意稳定系统在足够慢输入下可用静态模型近似；质量/动量/能量存储由单变量刻画则一阶（车速、刚性转动角速度、水位、良好混合体积浓度）；由两变量刻画则二阶（车的位置+速度、卫星姿态+角速度、双连通水位、双室浓度）。

**积分控制设计（S1 §11.2）**：以常数 $K=P(0)$ 近似对象，环路传递 $Kk_i/s$，闭环特征多项式 $s+Kk_i$。指定闭环时间常数 $T_{\rm cl}$，得
$$k_i=\frac{1}{T_{\rm cl}P(0)}.$$
要求 $T_{\rm cl}>T_{\rm ar}$，其中**平均驻留时间** $T_{\rm ar}=-P'(0)/P(0)$。

**一阶近似 + 积分控制（S1 式 11.6）**：以
$$P(s)\approx\frac{P(0)}{1+sT_{\rm ar}}$$
近似，取积分增益
$$\boxed{k_i=\frac{1}{2P(0)T_{\rm ar}}}\tag{11.6}$$
得环路传递
$$L(s)=P(s)C(s)\approx\frac{P(0)k_i}{1+sT_{\rm ar}}\cdot\frac{1}{s}=\frac{1}{2sT_{\rm ar}(1+sT_{\rm ar})},$$
闭环极点 $s=(-0.5\pm0.5i)/T_{\rm ar}$，$\omega_0=1/(T_{\rm ar}\sqrt2)$，上升时间 $3.1\,T_{\rm ar}$、调节时间 $7.9\,T_{\rm ar}$、超调 4%。

**Example 11.2（轻敲模式 AFM 的积分控制）**：原子力显微镜竖直运动传递函数
$$P(s)=\frac{a(1-e^{-s\tau})}{s\tau(s+a)},\quad a=\zeta\omega_0,\ \tau=2\pi n/\omega_0,\ \text{增益归一化为1}.$$
低频聚焦：$P(0)=1$，$P'(0)=-\tau/2-1/a=-(2+a\tau)/(2a)$。低频环路传递
$$L(s)\approx\frac{k_i(P(0)+sP'(0))}{s}=k_iP'(0)+\frac{k_iP(0)}{s}.$$
用式 11.6 设 $k_i=-1/(2P'(0))$，Nyquist/Bode（Fig. 11.5）显示低频性能好、稳定裕度好。即便含时延项，简单积分控制器即获良好性能。$\nabla$

**PI 极点配置（S1 §11.2, 式 11.7）**：一阶对象 $P(s)=b/(s+a)$，PI 控制下闭环特征多项式
$$s(s+a)+bk_ps+bk_i=s^2+(a+bk_p)s+bk_i.$$
要求 $p(s)=s^2+a_1s+a_2$，则
$$\boxed{k_p=\frac{a_1-a}{b},\qquad k_i=\frac{a_2}{b}.}\tag{11.7}$$
慢响应取 $a_1=a+\alpha,\ a_2=\alpha a$（$\alpha<a$）；快响应取 $a_1=2\zeta_c\omega_c,\ a_2=\omega_c^2$（$\omega_c,\zeta_c$ 为主模态固有频率与阻尼比）。$\omega_c$ 受模型有效频率上限与执行器饱和约束。

**Example 11.3（巡航控制 PI）**：车速线性模型（S1 式 11.8）
$$\frac{d(v-v_e)}{dt}=-a(v-v_e)-b_g(\theta-\theta_e)+b(u-u_e),$$
参数 $a=0.01,\ b=1.32,\ b_g=9.8,\ v_e=20,\ \theta_e=0,\ u_e=0.1687$。从油门到速度是一阶系统。开环慢（$1/a\approx100$ s），指定二阶闭环 $\zeta_c,\omega_c$，控制增益由式 11.7 给。Fig. 11.6：$t=5$ s 遇 $4^\circ$ 坡。取 $\zeta_c=1$ 无超调；$\omega_c$ 权衡响应速度与控制激烈程度，合理范围 $0.5$–$1.0$；$\omega_c=0.2$ 时最大速度误差仅约 1.3 m/s。$\nabla$

**高阶模型主导极点配置（S1 §11.2）**：积分控制器 $L(s)=k_iP(s)/s$，闭环特征 $s+k_iP(s)=0$。要 $s=-a$ 为根，取
$$k_i=\frac{a}{P(-a)},$$
若 $a$ 小于其他闭环极点模值则 $s=-a$ 为主导极点。PI/PID 同理（习题 11.3）。

## §2.5 PID 整定（S1 §11.3，全量含两张 Z-N 表）

### §2.5.1 Ziegler–Nichols 整定（S1 §11.3, Table 11.1, Fig. 11.7）

Z-N 是 1940 年代 [ZN42] 首套整定规则：做简单实验提取时/频域特征。

**(a) 阶跃响应法（时域，bump test）**：开环单位阶跃响应（Fig. 11.7a）由两参数刻画：$a$（最陡切线与纵轴交点）与 $\tau$（与时间轴交点，近似时延）。$a/\tau$ 是阶跃响应最陡斜率。无需等到稳态，到拐点即可。

**(b) 频率响应法（频域）**：连控制器、置 $k_i=k_d=0$，增大 $k_p$ 至系统起振。记临界增益 $k_c$ 与振荡周期 $T_c$。由 Nyquist 判据，$L=k_cP(s)$ 在 $\omega_c=2\pi/T_c$ 过临界点 $-1$，即得 $P(s)$ 上相位滞后 $180^\circ$ 的点（Fig. 11.7b）。

> **Table 11.1（原始 Ziegler–Nichols 整定规则，S1 §11.3，逐格抄录）**
>
> **(a) 阶跃响应法**（以 $a$、$\tau$ 表）：
>
> | 类型 | $k_p$ | $T_i$ | $T_d$ |
> |---|---|---|---|
> | P | $1/a$ | — | — |
> | PI | $0.9/a$ | $\tau/0.3$ | — |
> | PID | $1.2/a$ | $\tau/0.5$ | $0.5\tau$ |
>
> **(b) 频率响应法**（以临界增益 $k_c$、临界周期 $T_c$ 表）：
>
> | 类型 | $k_p$ | $T_i$ | $T_d$ |
> |---|---|---|---|
> | P | $0.5k_c$ | — | — |
> | PI | $0.45k_c$ | $T_c/1.2$ | — |
> | PID | $0.6k_c$ | $T_c/2$ | $T_c/8$ |

**Z-N 缺陷（S1 §11.3）**：用过程信息太少；所得闭环缺乏鲁棒性。

### §2.5.2 基于 FOTD 模型的整定（S1 §11.3, 式 11.9, 11.10）

**一阶 + 时延 (FOTD) 模型（S1 式 11.9）**：
$$\boxed{P(s)=\frac{K}{1+sT}e^{-\tau s},\qquad \tau_n=\frac{\tau}{T+\tau}.}\tag{11.9}$$
$\tau_n\in[0,1]$ 为**相对（归一化）时延**：$\tau_n\to0$ 滞后主导 (lag dominated)，$\tau_n\to1$ 时延主导 (delay dominated)，居中为平衡。

**参数辨识（S1 §11.3, Fig. 11.7a）**：$K$=阶跃响应稳态值；$\tau$=最陡切线与时间轴交点；$T_{63}$=输出达稳态 63% 的时间，$T=T_{63}-\tau$。（比 Z-N 慢，因测 $K$ 需等稳态。）

**改进 PI 整定规则（S1 式 11.10，基于 [ÅH06]；括号内为 Z-N 值对照）**：
$$k_p=\frac{0.15\tau+0.35T}{K\tau}\quad\left(\frac{0.9T}{K\tau}\right),\qquad k_i=\frac{0.46\tau+0.02T}{K\tau^2}\quad\left(\frac{0.27T}{K\tau^2}\right),\tag{11.10a}$$
$$k_p=0.16k_c\quad(0.45k_c),\qquad k_i=\frac{0.16k_c+0.72/K}{T_c}\quad\left(\frac{0.54k_c}{T_c}\right).\tag{11.10b}$$
改进式通常给出比原始 Z-N **更低**的增益。

**Example 11.4（轻敲模式 AFM 整定）**：归一化（以 $1/a$ 为时间单位）传递函数
$$P(s)=\frac{1-e^{-sT_n}}{sT_n(s+1)},\quad T_n=2n\pi a/\omega_0=2n\pi\zeta.$$
$\zeta=0.002,\ n=20$：首个实轴交点 $\mathrm{Re}\,s=-0.0461$（$\omega_c=13.1$），临界增益 $k_c=21.7$，临界周期 $T_c=0.48$。
- Z-N PI：$k_p=8.67,\ k_i=22.6$（$T_i=0.384$），稳定裕度 $s_m=0.31$（小），控制信号大超调。
- 改进 Z-N（式 11.10b）：$k_p=3.47,\ k_i=8.73$（$T_i=0.397$），$s_m=0.61$，超调降低，控制信号近乎瞬时到稳态。
- 纯积分控制器归一化增益 $k_i=1/(2+T_n)=0.44$，比 PI 积分增益小一个数量级以上。$\nabla$

**适用性（S1 §11.3）**：FOTD 整定对 PI 好用；微分对时延主导动态作用小，对滞后主导动态可显著提升性能，但滞后主导动态的 PID 整定不能仅基于 FOTD（见 [ÅH06]）。

### §2.5.3 继电反馈自整定 (Relay Feedback)（S1 §11.3, Fig. 11.9）

把过程接入与**继电非线性元件**的反馈环（Fig. 11.9a）。多数系统会起振（Fig. 11.9b）：继电输出 $u$ 为方波，过程输出 $y$ 近正弦，输入输出基波相差 $180^\circ$，即以临界周期 $T_c$ 振荡。

**临界增益推导（S1 §11.3，含完整代数）**：方波继电输出展开为 Fourier 级数；过程衰减高次谐波，故只看基波。设继电幅值 $d$，基波幅值 $4d/\pi$。设过程输出幅值 $a$，临界频率 $\omega_c=2\pi/T_c$ 处过程增益
$$|P(i\omega_c)|=\frac{\pi a}{4d}\quad\Rightarrow\quad \boxed{k_c=\frac{4d}{\pi a}.}$$
得 $k_c,T_c$ 后用 Z-N 规则定参数。可自动化（按钮触发，自动调幅值保持小振荡，整定毕换为 PID）；非对称继电可辨识更多参数 [BHÅ16]。

## §2.6 积分饱和 (Integral Windup) 与抗饱和（S1 §11.4，全量）

**饱和现象（S1 §11.4）**：执行器有限（电机限速、阀门全开/全闭）。控制变量达限时，反馈环断开、系统开环运行，误差非零故积分项继续累积，积分项与控制器输出变得很大；即便误差变号控制信号仍饱和，需很长时间才退出饱和，造成大瞬态。此即**积分饱和 (integrator windup)**。

**Example 11.5（巡航控制饱和）**：车遇 $6^\circ$ 陡坡使油门饱和（Fig. 11.10a）。$t=5$ 遇坡速度降、油门增至饱和；发动机力矩仅略大于重力分量，误差缓慢减小；积分持续累积至 $t=25$ 误差到零，但控制器输出仍超饱和限、执行器仍饱和；积分项开始减小，速度在 $t=40$ 才回到期望值，伴随大超调。$\nabla$

**抗饱和方案：执行器模型回算 (back-calculation)（S1 §11.4, Fig. 11.11）**：增设额外反馈路径，由饱和执行器的**数学模型**生成。设控制器输出 $u_a$、执行器模型输出 $u$，差
$$e_s=u-u_a,$$
经增益 $k_{aw}$ 馈入积分器输入。无饱和时 $e_s=0$，额外环无作用；饱和时 $e_s$ 反馈使 $e_s\to0$，从而控制器输出**保持在饱和限附近**，误差一变号即可退出饱和，避免 windup。复位速率由 $k_{aw}$ 控制（大则复位快），但过大会因测量噪声造成不良复位；合理取 $k_{aw}$ 为积分增益 $k_i$ 的若干倍。前馈信号 $u_{\rm ff}$ 按 Fig. 11.11 进入，基本抗饱和方案也处理前馈引起的饱和。

**Example 11.6（带抗饱和的巡航控制）**：Fig. 11.10b。因执行器模型反馈，积分器输出迅速复位使控制器输出处于饱和限，行为与 Fig. 11.10a 截然不同，大超调消失。仿真用跟踪增益 $k_{aw}=2$，比积分增益 $k_i=0.2$ 大一个数量级。$\nabla$

**抗饱和的稳定性分析（S1 §11.4, 式 11.11, 习题 11.12）**：重画框图隔离非线性，闭环 = 线性块 + 静态非线性。理想饱和是扇区有界非线性（式 10.17，$k_{\rm low}=0,\ k_{\rm high}=1$），线性部分
$$\boxed{H(s)=\frac{sP(s)C(s)-k_{aw}}{s+k_{aw}}.}\tag{11.11}$$
用**圆判据 (circle criterion)**（S1 §10.5）：此特殊非线性使圆退化为直线 $\mathrm{Re}\,s=-1$；若 $H(s)$ 的 Nyquist 曲线在直线 $\mathrm{Re}\,s=-1$ **右侧**则系统稳定。描述函数分析：若 $H(i\omega)$ 在临界点 $-1$ **左侧**与负实轴相交则可能振荡。习题 11.12(c)：$P(s)=k_v/s,\ C(s)=k_p+k_i/s$ 时，抗饱和系统稳定当 $k_{aw}>k_i/k_p$。

**手动控制与跟踪 (Manual Control and Tracking)（S1 §11.4, Fig. 11.11）**：自动 A / 手动 M 切换（开关）。手动模式下经 $k_{aw}$ 反馈调整积分器输入，使控制器输出 $u_a$ 跟踪手动输入 $u_m$，切回自动时无瞬态（无扰动切换 bumpless）。设积分器输出 $z$，控制器为
$$\frac{dx}{dt}=k_i(r-y_f)+k_{aw}(u-u_a),\qquad u_a=z-k_py_f-k_d\dot y_f,\qquad u=\begin{cases}F(u_a)&\text{自动},\\ F(u_m)&\text{手动},\end{cases}$$
$F(\cdot)$ 为执行器模型函数。$k_{aw}>k_i$ 时手动模式 $u$ 跟踪 $u_m$（若 $k_i(r-y_f)=0$ 则跟踪理想）。

**一般控制器的抗饱和（S1 §11.4, 式 11.12, Fig. 11.12）**：状态反馈 + 观测器结构（Fig. 8.11）中，饱和时若把指令输入（而非饱和输入）送观测器会出错；引入执行器模型，把其输出送观测器（Fig. 11.12）。稳定性：若观测器模型设计使过程执行器永不饱和，闭环 = 静态非线性块 $F(x)$ + 线性块（观测器+过程），线性块
$$\boxed{H(s)=K\big(sI-A+LC\big)^{-1}\big(B+LC[sI-A]^{-1}B\big),}\tag{11.12}$$
$A,B,C$ 为状态空间矩阵，$K$ 状态反馈增益，$L$ Kalman 滤波增益。简单饱和（扇区 $k_{\rm low}=0,k_{\rm high}=1$）下，圆判据：$L(i\omega)$ 的 Nyquist 在直线 $\mathrm{Re}\,z=-1/k_{\rm high}=-1$ 右侧且满足卷绕数条件则稳定。

## §2.7 PID 实现（S1 §11.5，全量含数字伪码）

### §2.7.1 微分滤波（S1 §11.5, 式 11.13, 11.14）

理想微分高频增益大。把 $k_ds$ 换为
$$\frac{k_ds}{1+sT_f},\qquad T_f=\frac{k_d/k_p}{N}=\frac{T_d}{N},\ N\in[5,20].$$
（可解释为低通滤波信号的理想微分。Fig. 11.3b 实现中 $T_f=T_d$，即 $N=1$。）

**对测量信号做二阶滤波（S1 式 11.13）**：用理想控制器 + 测量滤波：
$$\boxed{C(s)=k_p\!\left(1+\frac{1}{sT_i}+sT_d\right)\frac{1}{1+sT_f+(sT_f)^2/2}.}\tag{11.13}$$
Fig. 11.11 中滤波在 $G_f(s)$ 块，动态（S1 式 11.14）：
$$\frac{d}{dt}\begin{bmatrix}x_1\\x_2\end{bmatrix}=\begin{bmatrix}0&1\\-2T_f^{-2}&-2T_f^{-1}\end{bmatrix}\begin{bmatrix}x_1\\x_2\end{bmatrix}+\begin{bmatrix}0\\2T_f^{-2}\end{bmatrix}y,\tag{11.14}$$
状态 $x_1=y_f$（滤波测量）、$x_2=\dot y_f$。二阶滤波还提供高频滚降，改善鲁棒性。

### §2.7.2 设定点加权 (Setpoint Weighting)（S1 §11.5, 式 11.15）

误差反馈型（Fig. 11.1a）三项均作用于误差，阶跃参考下微分致控制信号大峰值。两自由度型（Fig. 11.1b）比例/微分仅作用于输出。中间形式（S1 式 11.15）：
$$\boxed{u=k_p(\beta r-y)+k_i\int_0^t\big(r(\tau)-y(\tau)\big)d\tau+k_d\!\left(\gamma\frac{dr}{dt}-\frac{dy}{dt}\right).}\tag{11.15}$$
比例/微分作用于参考的分数 $\beta,\gamma$（**参考权/设定点权**），积分必须作用于误差以保证稳态误差为零。不同 $\beta,\gamma$ 对负载扰动/测量噪声响应相同，对参考响应不同。$\beta=\gamma=0$ 称 **I-PD 控制器**。$\beta$ 典型 $0$–$1$，$\gamma$ 通常取 0（避免参考变化时控制信号大瞬态）。

**Example 11.7（带设定点加权的巡航控制）**：Fig. 11.13。$\beta=1$（误差反馈）速度有超调、油门近饱和；$\beta=0$ 无超调、控制信号小（驾乘更舒适）。增益 $k_p=0.74,\ k_i=0.19$；设定点权 $\beta=0,0.5,1$，$\gamma=0$。$\nabla$

### §2.7.3 运放实现（S1 §11.5, Fig. 11.14）

**PI 运放电路（S1 §11.5）**：阻抗
$$Z_1=R_1,\qquad Z_2=R_2+\frac{1}{sC_2}=\frac{1+R_2C_2s}{sC_2},\qquad \frac{Z_2}{Z_1}=\frac{1+R_2C_2s}{sR_1C_2}=\frac{R_2}{R_1}+\frac{1}{R_1C_2s},$$
传递函数 $-Z_2/Z_1$（运放增益为负）。即 PI 控制器，$k_p=R_2/R_1,\ k_i=1/(R_1C_2)$。

**PID 运放电路（S1 §11.5）**：
$$Z_1(s)=\frac{R_1}{1+R_1C_1s},\quad Z_2(s)=R_2+\frac{1}{C_2s},\quad \frac{Z_2}{Z_1}=\frac{(1+R_1C_1s)(1+R_2C_2s)}{R_1C_2s},$$
即 PID 控制器，参数
$$k_p=\frac{R_1C_1+R_2C_2}{R_1C_2},\qquad T_i=R_1C_1+R_2C_2,\qquad T_d=\frac{R_1R_2C_1C_2}{R_1C_1+R_2C_2}.$$

### §2.7.4 计算机（数字）实现（S1 §11.5, 式 11.16–11.18 + 伪码）

**采样控制时序（S1 §11.5）**：1. 等时钟中断；2. 读传感器；3. 算控制输出；4. 输出到执行器；5. 更新控制器状态；6. 重复。输出一就绪即送执行器（最小化延迟，把第 3 步算得尽量短，所有更新放输出之后）。

设采样时刻 $\{t_k\}$，采样周期 $h$（$t_{k+1}=t_k+h$）。离散化 Fig. 11.11 的 PID（含滤波微分、设定点加权、抗饱和），输出 $u=\mathrm{sat}(u_a)$，$u_a$=P+I+D。

**比例项（S1 式 11.16）**：
$$P(t_k)=k_p\big(\beta r(t_k)-y(t_k)\big).\tag{11.16}$$

**积分项（前向近似 + 抗饱和回算，S1 式 11.17）**：
$$I(t_{k+1})=I(t_k)+k_ih\,e(t_k)+\frac{h}{T_{aw}}\big(\mathrm{sat}(u_a)-u_a\big),\qquad T_{aw}=h/k_{aw}.\tag{11.17}$$

**滤波微分项（S1 式 11.18）**：连续 $T_f\dfrac{dD}{dt}+D=-k_d\dot y$，**后向差分**近似
$$T_f\frac{D(t_k)-D(t_{k-1})}{h}+D(t_k)=-k_d\frac{y(t_k)-y(t_{k-1})}{h},$$
整理：
$$\boxed{D(t_k)=\frac{T_f}{T_f+h}D(t_{k-1})-\frac{k_d}{T_f+h}\big(y(t_k)-y(t_{k-1})\big).}\tag{11.18}$$
后向差分的优点：系数 $T_f/(T_f+h)\in[0,1)$ 对所有 $h>0$ 成立，保证差分方程稳定。

> **PID 数字实现伪码（S1 §11.5，逐行抄录；ASCII 注释合规）**：
> ```
> % Precompute controller coefficients
> bi = ki*h
> ad = Tf/(Tf+h)
> bd = kd/(Tf+h)
> br = h/Taw
>
> % Initialize variables
> I = 0
> yold = adin(ch2)
>
> % Control algorithm - main loop
> while (running) {
>     r  = adin(ch1)                  % read setpoint from ch1
>     y  = adin(ch2)                  % read process variable from ch2
>     P  = kp*(b*r - y)               % compute proportional part  (b = beta)
>     D  = ad*D - bd*(y - yold)       % compute derivative part
>     ua = P + I + D                  % compute temporary output
>     u  = sat(ua, ulow, uhigh)       % simulate actuator saturation
>     daout(ch1)                      % set analog output ch1
>     I  = I + bi*(r - y) + br*(u-ua) % update integral state (anti-windup)
>     yold = y                        % update derivative state
>     sleep(h)                        % wait until next update interval
> }
> ```
> 三个状态：`yold`、`I`、`D`。一次输入到输出延迟 = 四乘四加 + 一次 `sat`。可定点实现于 PLC。注意此码把饱和放控制器内部（而非测执行器输出）。

## §2.8 PID 习题（S1 §11.6，全量，含可直接用作书章例题的解析式）

- **11.1（理想 PID）**：$P(s)=b/(s+a)$，$r\to y$ 传递函数：(a) 误差反馈 $G_{yr}(s)=\dfrac{bk_ds^2+bk_ps+bk_i}{(1+bk_d)s^2+(a+bk_p)s+bk_i}$；(b) 两自由度 $G_{yr}(s)=\dfrac{bk_i}{(1+bk_d)s^2+(a+bk_p)s+bk_i}$。
- **11.2**：二阶对象 $P(s)=b/(s^2+a_1s+a_2)$，PI 下闭环三阶，可配极点当极点和为 $-a_1$；目标 $(s+\alpha_c)(s^2+2\zeta_c\omega_cs+\omega_c^2)$。
- **11.3**：$P(s)=(s+1)^{-2}$，找使闭环极点 $s=-a$ 的积分控制器，定使 $k_i$ 最大的 $a$。
- **11.4/11.5（整定）**：对 $P_1=e^{-s}/s,\ P_2=e^{-s}/(s+1),\ P_3=e^{-s}$ 应用 Z-N 与改进规则设计 PI。
- **11.6（车辆转向）**：设计 PI 使闭环特征 $s^3+2\omega_cs^2+2\omega_c^2s+\omega_c^3$。
- **11.7（平均驻留时间）**：$T_{\rm ar}=\int_0^\infty th(t)\,dt=-P'(0)/P(0)$；PID（$k_i=k_p/T_i$）闭环平均驻留时间 $T_{\rm ar}=T_i/(P(0)k_p)$。
- **11.8（Web 服务器控制）**：队列 $dx/dt=\lambda-\mu$，PI $\mu=k_p(x-\beta x_r)+k_i\int_0^t(x-x_r)dt$；配特征 $s^2+1.6s+1$，调 $\beta$ 使阶跃超调 2%。
- **11.10/11.11/11.12（饱和/抗饱和）**：见 §2.6（11.12 的稳定性结论已抄）。11.11 条件积分（仅 $|e|<e_0$ 时更新积分）。
- **11.13**：二阶微分滤波换一阶的影响。

---

# 第三部分 稳定性理论（Lyapunov）（主源 S3, S4；参 Khalil 背景）

## §3.1 平衡点与稳定性定义（S3，含 $\epsilon$-$\delta$ 形式）

自治系统 $\dot x=f(x)$，**平衡点** $x_e$ 满足 $f(x_e)=0$。

**定义 3.1（Lyapunov 稳定, S3）**：平衡点 $x_e$ **Lyapunov 稳定 (stable i.s.L.)**，若
$$\forall\,\epsilon>0,\ \exists\,\delta>0\ \text{s.t.}\ \|x(0)-x_e\|<\delta\ \Rightarrow\ \forall t\ge0,\ \|x(t)-x_e\|<\epsilon.$$

**定义 3.2（渐近稳定, S3）**：$x_e$ **渐近稳定 (asymptotically stable)**，若它 Lyapunov 稳定，且
$$\exists\,\delta>0\ \text{s.t.}\ \|x(0)-x_e\|<\delta\ \Rightarrow\ \lim_{t\to\infty}\|x(t)-x_e\|=0.$$

**定义 3.3（指数稳定, S3）**：$x_e$ **指数稳定 (exponentially stable)**，若它渐近稳定，且
$$\exists\,\alpha,\beta,\delta>0\ \text{s.t.}\ \|x(0)-x_e\|<\delta\ \Rightarrow\ \|x(t)-x_e\|\le\alpha\|x(0)-x_e\|e^{-\beta t},\ \forall t\ge0.$$

**离散时间定义（S3）**：$x_{t+1}=f(x_t)$，$x$ Lyapunov 稳定若
$$\forall\epsilon>0\ \exists\delta>0\ \forall y\ \big[d(x,y)<\delta\Rightarrow\forall n\in\mathbb N,\ d(f^n(x),f^n(y))<\epsilon\big];$$
渐近稳定若 $\exists\delta>0$ s.t. $d(x,y)<\delta\Rightarrow\lim_{n\to\infty}d(f^n(x),f^n(y))=0$。

> **稳定性层级**：指数稳定 ⊂ 渐近稳定 ⊂ Lyapunov 稳定。前缀"全局 (global)"表示对**所有**初始状态成立（$\delta=\infty$）。

## §3.2 Lyapunov 第二（直接）法（S3, S4，含定理与判据）

**定理 3.1（Lyapunov 直接法, S3）**：考虑 $\dot x=f(x)$，平衡点 $x=0$。若存在 $V:\mathbb R^n\to\mathbb R$ 满足
1. $V(x)=0\iff x=0$；
2. $V(x)>0,\ \forall x\neq0$（**正定 positive definite**）；
3. $\dot V(x)=\dfrac{dV}{dt}=\sum_{i=1}^n\dfrac{\partial V}{\partial x_i}f_i(x)=\nabla V\cdot f(x)\le0,\ \forall x\neq0$（**负半定**），

则 $V$ 是 **Lyapunov 函数**，系统在 Lyapunov 意义下稳定。

**渐近稳定加强（S3）**：把条件 3 加强为 $\dot V(x)<0,\ \forall x\neq0$（**负定 negative definite**），则**渐近稳定**。

**全局渐近稳定 GAS（Barbashin–Krasovskii, S3）**：在渐近稳定条件外，再要求 **径向无界 (radially unbounded / proper)**：
$$\|x\|\to\infty\ \Rightarrow\ V(x)\to\infty,$$
则**全局渐近稳定**。（径向无界保证 $V$ 的水平集有界，避免轨迹逃逸到无穷。）

> **直觉**：$V$ 是"广义能量"，正定且沿轨迹不增（$\dot V\le0$）则系统不会远离平衡；严格递减（$\dot V<0$）则能量耗散到零，状态趋于平衡。

## §3.3 LaSalle 不变原理（S4，含全局/局部版与渐近稳定推论）

当 $\dot V$ 仅**负半定**（$\dot V\le0$ 而非 $<0$，常见于机械系统：$V=$ 能量，$\dot V=-$ 阻尼耗散，仅在速度非零时严格负）时，直接法只给稳定。LaSalle 原理把结论加强到渐近稳定。

**定理 3.2（LaSalle 不变原理，全局版，S4）**：自治系统 $\dot{\mathbf x}=f(\mathbf x)$，$f(\mathbf 0)=\mathbf 0$。若存在 $C^1$ 函数 $V(\mathbf x)$ 使
$$\dot V(\mathbf x)\le0,\quad\forall\mathbf x\quad(\text{负半定}),$$
则任意轨迹的**聚点 (accumulation points)** 包含于
$$\mathcal I=\{\text{完全包含于集合 }\{\mathbf x:\dot V(\mathbf x)=0\}\text{ 内的完整轨迹之并}\}.$$
即轨迹趋于 $\{\dot V=0\}$ 中的**最大不变集**。

**渐近稳定推论（Barbashin–Krasovskii–LaSalle, S4）**：若再有
- $V(\mathbf x)>0,\ \forall\mathbf x\neq\mathbf 0$，$V(\mathbf 0)=0$（正定）；
- $\mathcal I$ 除 $\mathbf x(t)\equiv\mathbf 0$ 外不含任何轨迹（即 $\{\dot V=0\}$ 的最大不变集仅为原点）；
- $V(\mathbf x)\to\infty$ 当 $\|\mathbf x\|\to\infty$（径向无界），

则**原点全局渐近稳定**。

**局部版（S4）**：若上述仅在原点的邻域 $D$ 内成立，且 $\{\dot V(\mathbf x)=0\}\cap D$ 除平凡解外不含轨迹，则**原点局部渐近稳定**。

> **注（S4）**：Wikipedia 该条未给证明。证明思路（Khalil 标准教材，\rebuilt 待核）：紧正不变集 $\Omega$ 上 $\dot V\le0\Rightarrow V$ 沿轨迹单调不增且有下界 $\Rightarrow V\to c$；正极限集 $\omega(x_0)$ 是不变的且 $V\equiv c$ 于其上 $\Rightarrow\dot V=0$ 于 $\omega(x_0)$，故 $\omega(x_0)\subseteq M$（$\{\dot V=0\}$ 中最大不变集）；轨迹趋于 $\omega(x_0)\subseteq M$。综合时建议补此证明骨架并引 Khalil。

## §3.4 线性系统与 Lyapunov 方程（S3，含等价条件）

**定理 3.3（线性系统稳定性, S3）**：线性系统 $\dot{\mathbf x}=A\mathbf x$ 渐近稳定（实为指数稳定）当且仅当 $A$ 的所有特征值实部为负（Hurwitz）。

**等价条件（Lyapunov 矩阵不等式, S3）**：存在正定对称阵 $M=M^\top\succ0$ 使
$$A^\top M+MA\prec0\quad(\text{负定}),$$
对应 Lyapunov 函数 $V(x)=x^\top Mx$。

**Lyapunov 方程形式（标准，\rebuilt 补全自 S3 + S10 §12.3.2 用法）**：给定任意 $Q=Q^\top\succ0$，方程
$$\boxed{A^\top M+MA=-Q}$$
有唯一正定解 $M\succ0$ 当且仅当 $A$ Hurwitz。证明骨架：$A$ 稳定时 $M=\int_0^\infty e^{A^\top t}Qe^{At}\,dt$ 收敛、正定、且满足方程（代入验证：$A^\top M+MA=\int_0^\infty\frac{d}{dt}(e^{A^\top t}Qe^{At})dt=[e^{A^\top t}Qe^{At}]_0^\infty=0-Q=-Q$）。综合时建议把此积分构造与代入验证完整写出。

**离散时间（S3 + 标准）**：$\mathbf x_{t+1}=A\mathbf x_t$ 指数稳定当且仅当 $A$ 所有特征值模 $<1$（Schur）。离散 Lyapunov 方程 $A^\top MA-M=-Q$。

## §3.5 Lyapunov 第一（间接）法（S3）

**定理 3.4（间接法/线性化, S3）**：平衡点 $x_e$ 处 Jacobian $J=\nabla f(x_e)$。
- 若 $J$ 是稳定矩阵（所有特征值实部 $<0$），则 $x_e$ **渐近稳定**。
- （标准补充，\rebuilt）若 $J$ 有特征值实部 $>0$，则 $x_e$ 不稳定；若有实部 $=0$（临界），间接法**失效**，须用直接法或 LaSalle。

## §3.6 Barbalat 引理（S3，自适应/非自治系统的关键工具）

**Barbalat 引理（S3）**：若 $f(t)$ 当 $t\to\infty$ 有有限极限，且 $\dot f$ 一致连续（充分条件：$\ddot f$ 有界），则
$$\dot f(t)\to0\quad(t\to\infty).$$

**积分版（S3）**：若 $f\in L^p(0,\infty)$ 且 $\dot f\in L^q(0,\infty)$，$p\in[1,\infty),\ q\in(1,\infty]$，则 $f(t)\to0$（$t\to\infty$）。

> **用途**：非自治系统 $\dot V\le0$ 但无 LaSalle（LaSalle 仅对自治系统）时，Barbalat 引理用来从 $\dot V\le-W(t)$ 推 $W(t)\to0$，是自适应控制收敛性证明的核心。综合时可作为 §3.3 LaSalle 的非自治补充。

---

# 第四部分 状态空间、可控性、可观性（主源 S5；参 S2, S6）

## §4.1 状态空间表示（S5）

线性时不变 (LTI) 系统：
$$\dot{\mathbf x}(t)=A\mathbf x(t)+B\mathbf u(t),\qquad \mathbf y(t)=C\mathbf x(t)+D\mathbf u(t),$$
$\mathbf x\in\mathbb R^n$ 状态，$\mathbf u\in\mathbb R^r$ 输入，$\mathbf y\in\mathbb R^m$（或 $p$）输出，$A\in\mathbb R^{n\times n}$，$B\in\mathbb R^{n\times r}$，$C\in\mathbb R^{m\times n}$，$D\in\mathbb R^{m\times r}$。离散：$\mathbf x(k+1)=A\mathbf x(k)+B\mathbf u(k),\ \mathbf y(k)=C\mathbf x(k)+D\mathbf u(k)$。

## §4.2 可控性 (Controllability)（S5，含定义、Kalman 秩、Gramian、PBH、证明链）

**定义 4.1（完全状态可控, S5）**：外部输入能在有限时间内把内部状态从任意初态移到任意终态。

**可控性矩阵与 Kalman 秩判据（S5）**：
$$\boxed{\mathcal C=\begin{bmatrix}B & AB & A^2B & \cdots & A^{n-1}B\end{bmatrix}\in\mathbb R^{n\times nr}.}$$
系统可控当且仅当 $\mathcal C$ 满行秩，即 $\operatorname{rank}(\mathcal C)=n$。

**离散时间可控性的代数（S5，含完整展开）**：由 $\mathbf x(k+1)=A\mathbf x(k)+B\mathbf u(k)$ 迭代，
$$\mathbf x(n)=A^n\mathbf x(0)+\sum_{i=0}^{n-1}A^{n-1-i}B\mathbf u(i),$$
即
$$\mathbf x(n)-A^n\mathbf x(0)=\begin{bmatrix}B & AB & \cdots & A^{n-1}B\end{bmatrix}\begin{bmatrix}\mathbf u^\top(n-1)\\ \mathbf u^\top(n-2)\\ \vdots\\ \mathbf u^\top(0)\end{bmatrix}.$$
若 $\mathcal C$ 有 $n$ 个线性无关列，则每个状态都可由适当输入到达。

**可控性 Gramian（S5）**：
- **有限时域积分形式**：
$$W(t_0,t_1)=\int_{t_0}^{t_1}\phi(t_0,t)B(t)B(t)^\top\phi(t_0,t)^\top\,dt,$$
$\phi(t_0,t)$ 为状态转移矩阵。存在从 $\mathbf x_0$ 到 $\mathbf x_1$ 的控制当且仅当 $\mathbf x_1-\phi(t_0,t_1)\mathbf x_0$ 在 $W(t_0,t_1)$ 的列空间中。
- **无限时域 Lyapunov 方程形式**（时不变稳态）：
$$\boxed{AW+WA^\top=-BB^\top,}$$
$W\in\mathbb R^{n\times n}$ 对称半正定。
- **Gramian 性质（S5）**：$W$ 对称；$t_1\ge t_0$ 时半正定；满足微分方程 $\dfrac{d}{dt}W(t,t_1)=A(t)W(t,t_1)+W(t,t_1)A(t)^\top-B(t)B(t)^\top$。

**Gramian↔秩判据的联系（Cayley–Hamilton, S5）**：系统可控当且仅当可控性 Gramian $W(t_0,t_1)$ 正定。其与秩判据的桥梁是 **Cayley–Hamilton 定理**：任意 $A^k$（$k\ge n$）可表为 $\{A^0,\dots,A^{n-1}\}$ 的线性组合，故 $n\times nr$ 可控性矩阵已捕获所有可达方向。

> **证明骨架（Gramian 正定 ⟺ 秩满，\rebuilt 标准补全）**：
> (⇐) 设 $W$ 非正定，则存在 $v\neq0$ 使 $v^\top Wv=\int_{t_0}^{t_1}\|B^\top\phi^\top v\|^2dt=0\Rightarrow B^\top\phi(t_0,t)^\top v\equiv0$。在 $t=t_0$ 处对 $t$ 反复求导并令 $t=t_0$（用 $\dot\phi=A\phi$）得 $v^\top B=v^\top AB=\dots=v^\top A^{n-1}B=0$，即 $v^\top\mathcal C=0$，$\mathcal C$ 不满秩。逆否即得满秩 $\Rightarrow W\succ0$。
> (⇒) $W\succ0$ 时取 $\mathbf u(t)=B^\top\phi(t_0,t)^\top W^{-1}(\mathbf x_1-\phi(t_0,t_1)\mathbf x_0)$ 可显式把 $\mathbf x_0$ 驱到 $\mathbf x_1$，故可控。
> 综合时建议把此双向证明完整写入。

**PBH（Popov–Belevitch–Hautus）判据（S5）**：系统可控当且仅当
$$\operatorname{rank}\begin{bmatrix}\lambda I-A & B\end{bmatrix}=n,\quad\forall\lambda\in\mathbb C$$
（实际只需对 $A$ 的特征值 $\lambda$ 验证）。

**可稳定性 (Stabilizability)（S5，对偶概念见下）**：若所有不可控模态都稳定，则系统**可稳定**。（PBH 形式：只对 $\mathrm{Re}\,\lambda\ge0$ 的 $\lambda$ 要求上式满秩。）

## §4.3 可观性 (Observability)（S5，含定义、秩、对偶、Gramian、PBH）

**定义 4.2（可观, S5）**：能由外部输出（与输入）唯一推断内部状态。

**可观性矩阵与秩判据（S5）**：
$$\boxed{\mathcal O=\begin{bmatrix}C\\ CA\\ CA^2\\ \vdots\\ CA^{n-1}\end{bmatrix}\in\mathbb R^{pn\times n},\qquad \text{可观}\iff\operatorname{rank}(\mathcal O)=n.}$$

**不可观子空间（S5）**：
$$N=\bigcap_{k=0}^{n-1}\ker(CA^k)=\ker(\mathcal O),$$
系统可观当且仅当 $N=\{0\}$。

**对偶性 (Duality)（S5）**：可观与可控互为对偶——系统 $(A,B)$ 可控当且仅当对偶系统 $(A^\top,B^\top)$ 可观。等价地，$(A,C)$ 可观 ⟺ $(A^\top,C^\top)$ 可控。故可观性 Gramian、PBH 由可控性结论按 $A\to A^\top,\ B\to C^\top$ 替换即得：

**可观性 Gramian（对偶得，S5 + 标准）**：
- 有限时域：$W_o(t_0,t_1)=\int_{t_0}^{t_1}\phi(t,t_0)^\top C^\top C\,\phi(t,t_0)\,dt$。
- 无限时域 Lyapunov 方程：$\boxed{A^\top W_o+W_oA=-C^\top C.}$
$A$ 稳定且该 Lyapunov 方程唯一解正定 ⟺ 系统可观。

**PBH 可观判据（对偶，S5 + 标准）**：
$$\operatorname{rank}\begin{bmatrix}\lambda I-A\\ C\end{bmatrix}=n,\quad\forall\lambda\in\mathbb C.$$

**可检测性 (Detectability)（S5）**：若所有不可观模态都稳定，则系统**可检测**（可观性的对偶松弛，对应可稳定性）。

**Kalman 分解（S5 背景）**：任意 LTI 系统可经相似变换分解为四块：可控可观、可控不可观、不可控可观、不可控不可观；传递函数仅反映可控可观部分。

## §4.4 全状态反馈与极点配置（综合 S2, S10 背景，\rebuilt）

可控系统 $(A,B)$ 可用状态反馈 $\mathbf u=-K\mathbf x$ 把闭环 $\dot{\mathbf x}=(A-BK)\mathbf x$ 的极点任意配置（极点配置定理；可控性是充要条件）。对偶地，可观系统可设计观测器（增益 $L$）任意配置观测误差动态 $(A-LC)$ 的极点。两者结合 = 输出反馈控制器（分离原理）。LQR（第五部分）给出 $K$ 的**最优**选择。

---

# 第五部分 状态空间最优控制：LQR 与 Riccati（主源 S7, S2；参 S6, S8）

## §5.1 动态规划与 Bellman 最优性（S7 dp.html, S8）

**离散 Bellman 最优性方程（cost-to-go 递推, S7）**：
$$\boxed{J^*(\mathbf x)=\min_{\mathbf u}\big[\ell(\mathbf x,\mathbf u)+J^*(f(\mathbf x,\mathbf u))\big].}$$
"任意状态的最优 cost-to-go = 一步代价 + 后继状态的最优 cost-to-go"。

**值迭代算法（S7）**：$\hat J^*(\mathbf x)\leftarrow\min_{\mathbf u}[\ell(\mathbf x,\mathbf u)+\hat J^*(f(\mathbf x,\mathbf u))]$，保证收敛到最优 cost-to-go。

**有限时域后向递推（S7）**：
$$J^*(\mathbf x,n-1)=\min_{\mathbf u}\big[\ell(\mathbf x,\mathbf u)+J^*(f(\mathbf x,\mathbf u),n)\big],\qquad \text{边界}\ J^*(\mathbf x,N)=\min_{\mathbf u}\ell(\mathbf x,\mathbf u).$$

**最优性原理 (Principle of Optimality, S7)**：最优策略的性质——无论初态与首步决策如何，余下决策对首步决策所致状态构成最优策略。

**最优策略提取（S7）**：$\pi^*(\mathbf x)=\arg\min_{\mathbf u}[\ell(\mathbf x,\mathbf u)+J^*(f(\mathbf x,\mathbf u))]$。

**连续时间 HJB 方程（S8）**：最优控制问题
$$V(x(0),0)=\min_u\left\{\int_0^TC[x(t),u(t)]\,dt+D[x(T)]\right\},\quad \dot x=F[x,u],$$
其值函数满足偏微分方程
$$\boxed{\frac{\partial V(x,t)}{\partial t}+\min_u\left\{\frac{\partial V(x,t)}{\partial x}\cdot F(x,u)+C(x,u)\right\}=0,}\qquad V(x,T)=D(x).$$
**推导（S8）**：对 $V(x(t+dt),t+dt)$ 作 Taylor 展开并令 $dt\to0$。**最优性条件（S8）**：在全状态空间上求解且 $V$ 连续可微时，HJB 是终端无约束最优的**必要且充分**条件。

## §5.2 连续时间无限时域 LQR：经 HJB 的复现级推导（S7 lqr.html，逐步不跳）

**问题（S7）**：LTI 系统
$$\dot{\mathbf x}=A\mathbf x+B\mathbf u,$$
无限时域二次代价
$$J=\int_0^\infty\big[\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u\big]dt,\qquad Q=Q^\top\succeq0,\ R=R^\top\succ0.$$
（**记号注**：Tedrake 用 $S$ 记 Riccati 解；本节沿用 $S$，与本书协方差 $P$ 区分。）

**步骤 1 — HJB（S7）**：最优 cost-to-go 满足
$$0=\min_{\mathbf u}\left[\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+\frac{\partial J^*}{\partial\mathbf x}(A\mathbf x+B\mathbf u)\right].$$

**步骤 2 — 二次型拟设 (ansatz)（S7）**：
$$J^*(\mathbf x)=\mathbf x^\top S\mathbf x,\quad S=S^\top\succeq0\quad\Rightarrow\quad\frac{\partial J^*}{\partial\mathbf x}=2\mathbf x^\top S.$$

**步骤 3 — 对 $\mathbf u$ 求最小（S7，含一阶条件代数）**：
$$\frac{\partial}{\partial\mathbf u}\big[\mathbf u^\top R\mathbf u+2\mathbf x^\top SB\mathbf u\big]=2\mathbf u^\top R+2\mathbf x^\top SB=0,$$
解出最优控制
$$\boxed{\mathbf u^*=-R^{-1}B^\top S\mathbf x=-K\mathbf x,\qquad K=R^{-1}B^\top S.}$$
（$R\succ0$ 保证 Hessian $2R\succ0$，确为极小。）

**步骤 4 — 回代得代数 Riccati 方程 (ARE)（S7，含化简）**：把 $\mathbf u^*$ 代回 HJB：
$$0=\mathbf x^\top Q\mathbf x+\mathbf x^\top SBR^{-1}B^\top S\mathbf x+2\mathbf x^\top S(A\mathbf x-BR^{-1}B^\top S\mathbf x).$$
（说明：$\mathbf u^{*\top}R\mathbf u^*=\mathbf x^\top SBR^{-1}RR^{-1}B^\top S\mathbf x=\mathbf x^\top SBR^{-1}B^\top S\mathbf x$；交叉项 $2\mathbf x^\top SB\mathbf u^*=-2\mathbf x^\top SBR^{-1}B^\top S\mathbf x$；合并 $\mathbf x^\top SBR^{-1}B^\top S\mathbf x-2\mathbf x^\top SBR^{-1}B^\top S\mathbf x=-\mathbf x^\top SBR^{-1}B^\top S\mathbf x$。）用对称性 $\mathbf x^\top SA\mathbf x=\mathbf x^\top A^\top S\mathbf x=\frac12\mathbf x^\top(SA+A^\top S)\mathbf x$，且上式对所有 $\mathbf x$ 成立，得
$$\boxed{SA+A^\top S-SBR^{-1}B^\top S+Q=0.}$$
（连续时间代数 Riccati 方程 CARE。正定解 $S\succ0$ 在系统**可稳定**时存在。）

**步骤 5 — 闭环稳定性（S7）**：闭环 $\dot{\mathbf x}=(A-BK)\mathbf x$，无限时域问题要求 $A-BK$ 所有特征值在左半平面。$S\succ0$（ARE 的正定解，可稳定时存在）保证之。**值函数 $J^*(\mathbf x)=\mathbf x^\top S\mathbf x$ 即 Lyapunov 函数**，证实稳定（$\dot J^*=-\mathbf x^\top Q\mathbf x-\mathbf u^{*\top}R\mathbf u^*\le0$，对 $Q\succ0$ 严格负定）。

## §5.3 连续时间有限时域 LQR 与微分 Riccati 方程（S7, S2）

**问题（S2 含交叉项的最一般形式）**：
$$\dot{\mathbf x}=A\mathbf x+B\mathbf u,\quad t\in[t_0,t_1],$$
$$J=\mathbf x^\top(t_1)F(t_1)\mathbf x(t_1)+\int_{t_0}^{t_1}\big(\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+2\mathbf x^\top N\mathbf u\big)dt,$$
$F(t_1)$ 终端代价阵，$N$ 交叉权。

**最优反馈（时变, S2）**：
$$\mathbf u=-K(t)\mathbf x,\qquad K(t)=R^{-1}\big(B^\top P(t)+N^\top\big).$$

**微分 Riccati 方程 (DRE, S2)**（含交叉项）：
$$A^\top P(t)+P(t)A-\big[P(t)B+N\big]R^{-1}\big[B^\top P(t)+N^\top\big]+Q=-\dot P(t),$$
**终端条件** $P(t_1)=F(t_1)$，后向积分。

**无交叉项的 DRE（S7 形式，配二次 cost-to-go $J^*(\mathbf x,t)=\mathbf x^\top S(t)\mathbf x$）**：由 $\partial J^*/\partial t=\mathbf x^\top\dot S(t)\mathbf x$ 代入 HJB，得
$$\boxed{-\dot S(t)=S(t)A+A^\top S(t)-S(t)BR^{-1}B^\top S(t)+Q,\qquad S(t_f)=Q_f.}$$
后向从 $t_f$ 积到 $t_0$。无限时域 ARE 即其稳态 $\dot S=0$ 解。

## §5.4 无限时域 LQR 的代数 Riccati（含交叉项的一般式, S2）

**代价**：$J=\int_0^\infty(\mathbf x^\top Q\mathbf x+\mathbf u^\top R\mathbf u+2\mathbf x^\top N\mathbf u)dt$。

**最优增益（S2）**：$\mathbf u=-K\mathbf x$，$K=R^{-1}(B^\top P+N^\top)$。

**CARE（一般式, S2）**：
$$A^\top P+PA-(PB+N)R^{-1}(B^\top P+N^\top)+Q=0.$$
**等价（消去交叉项, S2）**：令 $\mathcal A=A-BR^{-1}N^\top$，$\mathcal Q=Q-NR^{-1}N^\top$，则
$$\mathcal A^\top P+P\mathcal A-PBR^{-1}B^\top P+\mathcal Q=0.$$
（即把交叉项吸收进 $\mathcal A,\mathcal Q$，化为无交叉项的标准 CARE。）

## §5.5 离散时间 LQR（有限/无限时域, S2）

**有限时域问题（S2）**：$\mathbf x_{k+1}=A\mathbf x_k+B\mathbf u_k$，
$$J=\mathbf x_{H_p}^\top Q_{H_p}\mathbf x_{H_p}+\sum_{k=0}^{H_p-1}\big(\mathbf x_k^\top Q\mathbf x_k+\mathbf u_k^\top R\mathbf u_k+2\mathbf x_k^\top N\mathbf u_k\big),$$
$H_p$ 为时域。最优控制 $\mathbf u_k=-F_k\mathbf x_k$，时变增益
$$F_k=(R+B^\top P_{k+1}B)^{-1}(B^\top P_{k+1}A+N^\top),$$
**离散 Riccati 递推（后向）**：
$$\boxed{P_{k-1}=A^\top P_kA-(A^\top P_kB+N)(R+B^\top P_kB)^{-1}(B^\top P_kA+N^\top)+Q,}$$
终端 $P_{H_p}=Q_{H_p}$。

**无限时域（S2）**：$\mathbf u_k=-F\mathbf x_k$，常增益
$$F=(R+B^\top PB)^{-1}(B^\top PA+N^\top),$$
**离散代数 Riccati 方程 (DARE)**：
$$\boxed{P=A^\top PA-(A^\top PB+N)(R+B^\top PB)^{-1}(B^\top PA+N^\top)+Q.}$$
等价（$\mathcal A=A-BR^{-1}N^\top,\ \mathcal Q=Q-NR^{-1}N^\top$）：
$$P=\mathcal A^\top P\mathcal A-\mathcal A^\top PB(R+B^\top PB)^{-1}B^\top P\mathcal A+\mathcal Q.$$

**无交叉项 DARE（S6 简式, $N=0$）**：
$$P=A^\top PA-(A^\top PB)(R+B^\top PB)^{-1}(B^\top PA)+Q.$$

## §5.6 Riccati 方程的求解：Hamilton 矩阵 / 辛矩阵（S6）

**CARE 的 Hamilton 矩阵法（S6）**：标准 CARE $A^\top P+PA-PBR^{-1}B^\top P+Q=0$ 对应 $2n\times2n$ **Hamilton 矩阵**
$$\boxed{Z=\begin{pmatrix}A & -BR^{-1}B^\top\\ -Q & -A^\top\end{pmatrix}.}$$
取 $Z$ 的**稳定不变子空间**（特征值实部为负的特征向量张成）的一组基 $\begin{pmatrix}U_{1,1}\\ U_{2,1}\end{pmatrix}\in\mathbb R^{2n\times n}$，则**镇定解**
$$P=U_{2,1}U_{1,1}^{-1}.$$
闭环 $A-BR^{-1}B^\top P$ 所有特征值实部 $<0$ 时稳定。

**DARE 的辛矩阵法（S6，$A$ 可逆时）**：$2n\times2n$ **辛矩阵 (symplectic)**
$$Z=\begin{pmatrix}A+BR^{-1}B^\top(A^{-1})^\top Q & -BR^{-1}B^\top(A^{-1})^\top\\ -(A^{-1})^\top Q & (A^\top)^{-1}\end{pmatrix}.$$
同法 $P=U_{2,1}U_{1,1}^{-1}$，稳定子空间对应**模 $<1$**（单位圆内）的特征值。镇定要求 $A-B(R+B^\top PB)^{-1}B^\top PA$ 所有特征值在单位圆内。

**矩阵尺寸（S6）**：$P$ 为 $n\times n$ 对称未知阵；$A$ 为 $n\times n$；$B$ 为 $n\times k$；$Q$ 为 $n\times n$ 对称半正定（状态代价）；$R$ 为 $k\times k$ 对称正定（控制代价）。

**存在唯一性条件（S6 + 标准, \rebuilt）**：S6 原文未显式给出，但标准结论为——若 $(A,B)$ **可稳定**且 $(A,Q^{1/2})$（或 $(Q^{1/2},A)$）**可检测**，则 CARE/DARE 存在唯一对称半正定**镇定解**，对应闭环渐近稳定。综合时务必补此条件并引 Khalil/Anderson-Moore。

## §5.7 Riccati 方程的本质：标量 Riccati 与降阶（S6 Riccati 条目）

**标量 Riccati ODE（S6）**：$y'(x)=q_0(x)+q_1(x)y(x)+q_2(x)y^2(x)$（$q_0,q_2\neq0$）。$q_0=0$ 退化为 Bernoulli 方程；$q_2=0$ 退化为一阶线性 ODE。

**降阶为二阶线性 ODE（S6）**：先令 $v=yq_2$ 得 $v'=v^2+R(x)v+S(x)$，$S=q_0q_2$，$R=q_1+q_2'/q_2$；再用关键代换 $v=-u'/u$ 化为线性
$$u''-R(x)u'+S(x)u=0.$$
**矩阵 Riccati 微分方程一般形（S6）**：$\dfrac{dw(z)}{dz}=A_0(z)+A_1(z)w+A_2(z)w^2$，$A_i$ 为矩阵函数。

> **联系**：LQR 的矩阵 DRE 是上式的矩阵推广（$w\leftrightarrow P$，二次项 $-PBR^{-1}B^\top P$）；同样可经 $P=UV^{-1}$（$U,V$ 满足线性 Hamilton 系统 $\dot U=AU-BR^{-1}B^\top V$，$\dot V=-QU-A^\top V$）线性化，这正是 §5.6 Hamilton 矩阵法的微分版本。综合时可点出这一统一脉络。

---

# 第六部分 模型预测控制 MPC（主源 S10 = Borrelli-Bemporad-Morari Ch.12；参 S9）

## §6.1 MPC（滚动时域）问题形式（S9, S10）

**每步求解的有限时域最优控制问题 (S9)**：在每个时刻 $t$，以当前状态 $x_0=x(t)$ 为初值，解
$$\min_{u_0,\dots,u_{N-1}}\ J=\sum_{k=0}^{N-1}\ell(x_k,u_k)+V_f(x_N)$$
$$\text{s.t.}\quad x_{k+1}=f(x_k,u_k),\ k=0,\dots,N-1;\quad x_k\in\mathcal X;\quad u_k\in\mathcal U;\quad x_N\in\mathcal X_f;\quad x_0=x(t).$$
符号：$N$ 预测时域；$\ell(x_k,u_k)$ 阶段代价；$V_f(x_N)$ 终端代价；$\mathcal X$ 状态约束集；$\mathcal U$ 输入约束集；$\mathcal X_f$ 终端约束集。

**滚动时域原理 (Receding Horizon, S9)**：
1. 在 $t$ 解最优控制问题；2. **仅施加首步** $u^*(t)=u_0^*$；3. 采样新状态 $x(t+1)$；4. 回到 1。因预测窗向前滑动，故称"滚动时域控制"。

**线性 MPC 的二次代价（过程工业常用形, S9）**：
$$J=\sum_{i=1}^N w_{x_i}(r_i-x_i)^2+\sum_{i=1}^M w_{u_i}(\Delta u_i)^2,$$
$x_i$ 受控变量，$r_i$ 参考，$\Delta u_i$ 输入增量，$w_{x_i},w_{u_i}$ 权重。

## §6.2 MPC 的标准二次型问题与 QP（S10 §12.2, 含数值例）

**S10 标准形式（Ch.12 用）**：终端代价 $p(x_N)=x_N^\top Px_N$，阶段代价 $q(x_k,u_k)=x_k^\top Qx_k+u_k^\top Ru_k$。RHC 问题记 (12.6)，闭环系统记 $x(k+1)=f_{\rm cl}(x(k))$（12.10），$f_{\rm cl}(x)=Ax+Bu_0^*(x)$。

**Example 12.1（双积分器, S10 §12.2，数值全抄）**：$N=3$，$P=Q=\begin{bmatrix}1&0\\0&1\end{bmatrix}$，$R=10$，$\mathcal X_f=\mathbb R^2$，输入约束 $-0.5\le u(k)\le0.5$（12.12），状态约束 $\begin{bmatrix}-5\\-5\end{bmatrix}\le x(k)\le\begin{bmatrix}5\\5\end{bmatrix}$（12.13）。关联 QP（形如 11.31）的矩阵：
$$H=\begin{bmatrix}13.50&-10.00&-0.50\\-10.00&22.00&-10.00\\-0.50&-10.00&31.50\end{bmatrix},\quad F=\begin{bmatrix}-10.50&10.00&-0.50\\-20.50&10.00&9.50\end{bmatrix},\quad Y=\begin{bmatrix}14.50&23.50\\23.50&54.50\end{bmatrix}.$$
（$G_0,E_0,w_0$ 为约束矩阵，见 S10 式 12.15，此处从略；综合若需可回 S10。）

> **QP-based 在线 RHC 算法（S10 Algorithm 12.2，逐行）**：
> ```
> Input:  state x(t) at time t
> Output: receding horizon control input u(x(t))
> compute  F_tilde = 2 F' x(t)
> compute  w0_tilde = w0 + E0 x(t)
> [U0*, Flag] = QP(H, F_tilde, G0, w0_tilde)   % solve the QP
> if Flag == infeasible then stop
> return first element u0* of U0*
> ```
> Example 12.1 观察：$x(0)=[-4.5,2]$ 收敛且满足约束；$x(0)=[-4.5,3]$ 在 $x(2)=[1,2]$ 因**不可行**停止。闭环轨迹不同于开环预测（滚动时域本质）。

**Example 12.2（不稳定系统, S10 §12.2，数值全抄）**：
$$x(t+1)=\begin{bmatrix}2&1\\0&0.5\end{bmatrix}x(t)+\begin{bmatrix}1\\0\end{bmatrix}u(t),\quad -1\le u(k)\le1,\quad -10\le x_i(k)\le10.$$
取 $Q=I$，无终端约束/权（$\mathcal X_f=\mathbb R^2,\ P=0$）。三组设置：
- Setting 1: $N=2,\ R=10$ —— $O_\infty=\{0\}$，所有非零初态发散并最终不可行；
- Setting 2: $N=3,\ R=2$ —— 部分初态收敛；
- Setting 3: $N=4,\ R=1$ —— 收敛初态集更大。

**关键教训（S10）**：参数 $N,P,Q,R$ 以复杂方式影响闭环行为；$x(0)=[-4,8.5]$ 在**最大控制不变集** $C_\infty$ 之外，任何控制器都无法把它保持在有界集内。

## §6.3 持久可行性 (Persistent Feasibility)（S10 §12.3.1，含引理与证明）

**问题（S10）**：初始可行 $x(0)\in X_0$ 不必保证未来恒可行。"未来所有时刻都可行"称**持久可行性**。

集合定义（S10）：$C_\infty$ 最大控制不变集；$O_\infty$ 闭环 $x(k+1)=f_{\rm cl}(x(k))$ 的最大正不变集（依赖 $X,U,N,\mathcal X_f,P,Q,R$）；$X_0$ 初始可行集（不依赖 $P,Q,R$）。恒有 $O_\infty\subseteq X_0\subseteq C_\infty$。

**Lemma 12.1（持久可行性充要条件, S10）**：设 $O_\infty$ 为闭环（12.10）在约束（12.2）下的最大正不变集。RHC 问题持久可行**当且仅当** $X_0=O_\infty$。
**证明（S10，全抄）**：持久可行要求 $X_0$ 对闭环正不变。已论 $O_\infty\subseteq X_0$。正不变集 $X_0$ 不能大于最大正不变集 $O_\infty$，故 $X_0=O_\infty$。∎

**Lemma 12.2（持久可行性充分条件, S10）**：RHC（12.6）–（12.9），$N\ge1$。若 $X_1$ 是系统（12.1）–（12.2）的控制不变集，则 RHC 持久可行；且 $O_\infty=X_0$ 与 $P,Q,R$ 无关。
**证明（S10，全抄）**：$X_1$ 控制不变 $\Rightarrow X_1\subseteq\mathrm{Pre}(X_1)$。由可行集性质（11.20）$\mathrm{Pre}(X_1)=X_0$（控制不变下 $\mathrm{Pre}(X_1)\cap X=\mathrm{Pre}(X_1)$）。取任意 $x\in X_0$ 及其可行控制 $u$，定义 $x^+=Ax+Bu\in X_1$。则 $x^+\in X_1\subseteq\mathrm{Pre}(X_1)=X_0$。$u$ 任意（只要可行），故对所有可行 $u$ 有 $x^+\in X_0$。$X_0$ 正不变 $\Rightarrow X_0=O_\infty$（Lemma 12.1）。$X_0$ 对所有可行 $u$ 正不变，故 $O_\infty$ 不依赖 $P,Q,R$。∎（此性质称"对所有可行 $u$ 持久可行"。）

**Corollary 12.1（S10）**：RHC（$N\ge1$），若存在 $i\in[1,N]$ 使 $X_i$ 控制不变，则对**所有代价函数** RHC 持久可行。

**Corollary 12.2（S10）**：若 $N$ 大于 $K_\infty(\mathcal X_f)$ 的确定性指标 $\bar N$，则 RHC 持久可行。（$X_i=K_{N-i}(\mathcal X_f)$，最大可控集有限确定时 $X_i=K_\infty(\mathcal X_f)$ 控制不变。）

**Theorem 12.1（S10，由 Lemma 12.2 推论）**：若 $\mathcal X_f$ 控制不变，则 $X_{N-1},\dots,X_1$ 均控制不变，持久可行性成立。

## §6.4 MPC 稳定性主定理与完整证明（S10 §12.3.2, Theorem 12.2）

**核心思想（S10）**：寻找闭环 Lyapunov 函数。若终端代价与终端约束恰当选择，则**值函数 $J_0^*(\cdot)$ 即闭环 Lyapunov 函数**。

**Theorem 12.2（RHC 渐近稳定性, S10，全抄）**：考虑系统（12.1）–（12.2）、RHC 律（12.6）–（12.9）与闭环（12.10）。假设
- **(A0)** 阶段代价 $q(x,u)$ 与终端代价 $p(x)$ 连续且**正定**；
- **(A1)** 集合 $\mathcal X,\mathcal X_f,\mathcal U$ 在内部含原点且为**闭集**；
- **(A2)** $\mathcal X_f$ **控制不变**，$\mathcal X_f\subseteq\mathcal X$；
- **(A3)** $\displaystyle\min_{v\in\mathcal U,\ Ax+Bv\in\mathcal X_f}\big(-p(x)+q(x,v)+p(Ax+Bv)\big)\le0,\quad\forall x\in\mathcal X_f.$

则闭环（12.10）的**原点渐近稳定**，吸引域为 $X_0$。

**证明（S10，逐步全抄）**：

*持久可行性*：由 (A2)、Theorem 12.1、Lemma 12.1，$X_0=O_\infty$ 是闭环（12.10）对任意代价函数的正不变集，故 $X_0$ 内任意可行输入持久可行。

*收敛与稳定*：证 $J_0^*(\cdot)$（12.6）是闭环 Lyapunov 函数。因代价 $J_0$、系统、约束均时不变，只需研究 $J_0^*$ 在步 $k=0$ 与 $k+1=1$ 之间的性质。

设 $t=0$ 的问题（12.6），$x(0)\in X_0$，最优解 $U_0^*=\{u_0^*,\dots,u_{N-1}^*\}$，对应最优状态轨迹 $\mathbf x_0=\{x(0),x_1,\dots,x_N\}$。施加 $u_0^*$ 后 $x(1)=x_1=Ax(0)+Bu_0^*$。

考虑 $t=1$ 的问题，构造 $J_0^*(x(1))$ 的**上界**。取候选序列（**移位 + 终端控制**）
$$\tilde U_1=\{u_1^*,\dots,u_{N-1}^*,v\},$$
对应初态 $x(1)$ 的状态轨迹 $\tilde{\mathbf x}_1=\{x_1,\dots,x_N,Ax_N+Bv\}$。因 $x_N\in\mathcal X_f$ 且 (A2)，存在可行 $v$ 使 $x_{N+1}=Ax_N+Bv\in\mathcal X_f$，故 $\tilde U_1$ 可行。$\tilde U_1$ 非最优，故 $J_0(x(1),\tilde U_1)$ 是 $J_0^*(x(1))$ 的上界。

$U_0^*$ 与 $\tilde U_1$ 生成的轨迹除首末区间外重叠，故
$$J_0^*(x(1))\le J_0(x(1),\tilde U_1)=J_0^*(x(0))-q(x_0,u_0^*)-p(x_N)+\big(q(x_N,v)+p(Ax_N+Bv)\big).\tag{12.19}$$
令 $x=x_0=x(0)$，$u=u_0^*$。在 (A3) 下，（12.19）成为
$$\boxed{J_0^*(Ax+Bu)-J_0^*(x)\le-q(x,u),\quad\forall x\in X_0.}\tag{12.20}$$
（12.20）与 (A0)（$q(\cdot)$ 正定）保证 $J_0^*(x)$ 沿闭环（12.10）轨迹对任意 $x\in X_0,\ x\neq0$ **严格递减**。又 $J_0^*(x)$ 下界为零，且从任意 $x(0)\in X_0$ 出发的闭环轨迹对所有 $k\ge0$ 留在 $X_0$，故（12.20）足以保证闭环状态收敛到零（若初态在 $X_0$）。**(i) 收敛证毕。**

*稳定性（经 Theorem 7.2，即需证 $J_0^*$ 是 Lyapunov 函数）*：正定性由 (A0)；递减由（12.20）。**原点连续性**：证 $J_0^*(x)\le p(x),\ \forall x\in\mathcal X_f$，则因 $p(x)$ 在原点连续（A0），$J_0^*(x)$ 在原点连续。由 (A2)，$\mathcal X_f$ 控制不变，故对任意 $x\in\mathcal X_f$ 存在可行输入序列 $\{u_0,\dots,u_{N-1}\}$，其状态轨迹 $\{x_0,\dots,x_N\}$ 全留在 $\mathcal X_f$（$x_i\in\mathcal X_f,\ \forall i$）。在所有此类序列中取满足 (A3) 的 $u_i$（$\forall i$），给出 $J_0^*$ 的上界：
$$J_0^*(x_0)\le\sum_{i=0}^{N-1}q(x_i,u_i)+p(x_N),\quad x_i\in\mathcal X_f.\tag{12.21}$$
改写为
$$J_0^*(x_0)\le\sum_{i=0}^{N-1}q(x_i,u_i)+p(x_N)=p(x_0)+\sum_{i=0}^{N-1}\big(q(x_i,u_i)+p(x_{i+1})-p(x_i)\big),\quad x_i\in\mathcal X_f,\tag{12.22}$$
由 (A3)（每项 $q(x_i,u_i)+p(x_{i+1})-p(x_i)\le0$）得
$$\boxed{J_0^*(x)\le p(x),\quad\forall x\in\mathcal X_f.}\tag{12.23}$$
结论：任意 $x\in X_0$ 在**有限时间**被驱入 $\mathcal X_f$ 内某 $J_0^*$ 水平集，其后收敛到并稳定于原点。∎

**Remark 12.1（S10）**：(A0) 阶段代价正定性可松弛（如标准最优控制）。对 2-范数代价（12.8），可允许 $Q\succeq0$ 但要求 $(Q^{1/2},A)$ 可观。

**Remark 12.2（S10）**：Theorem 12.2 一般**保守**：需人工终端集 $\mathcal X_f$ 保证持久可行、终端代价保证稳定；要求 $x_N\in\mathcal X_f$ 通常缩小吸引域 $X_0=O_\infty$，且可能损性能。

**Remark 12.3（S10）**：满足 (A3) 的 $p(x)$ 常称**控制 Lyapunov 函数 (control Lyapunov function, CLF)**。

> **(A3) 的标准等价形（控制 Lyapunov 函数减量条件，综合常用写法，\rebuilt 由 S10 Remark 推广）**：存在局部终端控制律 $\kappa_f(x)$ 使 $\forall x\in\mathcal X_f$：
> $$p(f(x,\kappa_f(x)))-p(x)\le-q(x,\kappa_f(x)),\quad \kappa_f(x)\in\mathcal U,\quad f(x,\kappa_f(x))\in\mathcal X_f.$$
> 这就是 Mayne–Rawlings–Rao–Scokaert (2000) 经典四条件（终端集不变 + 终端集含于状态约束 + 终端控制满足输入约束 + 终端代价 CLF 减量）的紧凑版。综合写正文时建议同时给 S10 的 (A0)-(A3) 与此 $\kappa_f$ 版，并引 Mayne 2000。

## §6.5 终端代价/终端集的构造（S10 §12.3.2 续）

**线性 + 二次代价的常用构造（S10）**：取局部线性反馈 $u=F_\infty x$ 镇定 $(A,B)$，令 $\mathcal X_f$ 为闭环 $x(k+1)=(A+BF_\infty)x(k)$ 的（约束下）正不变集。此时 (A3) 变为 **Lyapunov 方程**：取 $p(x)=x^\top Px$，要求
$$(A+BF)^\top P(A+BF)-P=-(Q+F^\top RF),$$
即 $x^\top Px$ 是局部 LQR 无限时域代价时，(A3) 自动满足。**特别地**：若取 $F=F_\infty$（LQR 增益），$x^\top Px$（$P$ 为 DARE 解）恰为从 $\mathcal X_f$ 内起的无限时域最优代价，MPC 在终端集内退化为 LQR。**若开环系统已渐近稳定**，可取 $F=0$；$\mathcal X_f$ 取闭环（即开环）正不变集，则 $\mathcal X_f$ 内输入 $0$ 可行且满足 (A3)（1-范数/∞-范数有相应 Lyapunov 不等式）。

> **脉络小结（综合提示）**：MPC = "带约束、滚动重解的有限时域最优控制"，其无约束、无限时域、线性二次特例**就是 LQR**（S10 §12.3 明言："$N=\infty$ 时如 §8.5 LQR / §11.3.4 CLQR，闭环=开环预测，可行即恒可行、有限解即渐近收敛"）。终端代价取 LQR 的 Riccati 代价、终端集取 LQR 闭环不变集，是连接 MPC 与 LQR 的标准桥。

---

# 第七部分 面向机器人的控制（逆动力学 / 计算力矩 / WBC 概览）（主源 S11, S12, S13）

## §7.1 机器人动力学方程（S11, S12）

刚体机械臂（$N$ 自由度，关节坐标 $\mathbf q\in\mathbb R^N$）运动方程（S11，统一为本书惯用 $\mathbf q$）：
$$\boxed{M(\mathbf q)\ddot{\mathbf q}+C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\mathbf g(\mathbf q)=\boldsymbol\tau,}$$
- $M(\mathbf q)$ 惯性（质量）矩阵，对称正定；
- $C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}$ 科氏/离心力矩；
- $\mathbf g(\mathbf q)$ 重力力矩（S11 记 $\boldsymbol\tau_g$）；
- $\boldsymbol\tau$ 关节力矩输入。

（S12 含外力的更全形式：$\boldsymbol\tau=M(\mathbf q)\ddot{\mathbf q}+\mathbf b(\mathbf q,\dot{\mathbf q})+\mathbf g(\mathbf q)+\boldsymbol\tau_{\rm ext}$，$\mathbf b$ = 科氏/离心。）

## §7.2 计算力矩 / 逆动力学控制（S11，含误差动态推导）

**控制律（S11，统一 $\mathbf q$；误差 $\mathbf e=\mathbf q_d-\mathbf q$）**：
$$\boxed{\boldsymbol\tau=\hat M(\mathbf q)\big(\ddot{\mathbf q}_d+K_p\mathbf e+K_d\dot{\mathbf e}\big)+\hat C(\mathbf q,\dot{\mathbf q})\dot{\mathbf q}+\hat{\mathbf g}(\mathbf q),}$$
$\hat{(\cdot)}$ 为模型估计值，$\mathbf q_d$ 期望轨迹。

**误差动态（S11，含推导）**：若模型精确（$\hat M=M$ 等），代入动力学方程并左乘 $M^{-1}$：
$$\ddot{\mathbf q}=\ddot{\mathbf q}_d+K_p\mathbf e+K_d\dot{\mathbf e},$$
由 $\mathbf e=\mathbf q_d-\mathbf q\Rightarrow\ddot{\mathbf e}=\ddot{\mathbf q}_d-\ddot{\mathbf q}=-(K_p\mathbf e+K_d\dot{\mathbf e})$，得**线性解耦误差动态**
$$\boxed{\ddot{\mathbf e}+K_d\dot{\mathbf e}+K_p\mathbf e=\mathbf 0.}$$
即每个关节误差是二阶线性系统，可用标准 PD/PID 整定（$K_p,K_d$ 对角时完全解耦为 $N$ 个独立二阶环）。

**反馈线性化解释（S11）**：计算力矩 = 用机器人动力学模型做**反馈线性化**（抵消非线性 $C,\mathbf g$ 与惯性耦合 $M$），把非线性机器人动态变换为线性二阶误差系统，再叠加 PD/PID。模型不精确时只需 $M^{-1}\hat M\approx I$ 即近似线性化。

> **稳定性（综合提示，\rebuilt）**：线性误差动态 $\ddot{\mathbf e}+K_d\dot{\mathbf e}+K_p\mathbf e=0$ 当 $K_p,K_d\succ0$（对角正）时全局指数稳定（每个二阶子系统极点在左半平面）。Lyapunov 函数可取 $V=\frac12\dot{\mathbf e}^\top\dot{\mathbf e}+\frac12\mathbf e^\top K_p\mathbf e$（参第三部分）。模型误差下用鲁棒/自适应项 + Barbalat 引理（§3.6）证收敛。综合时可把此与 §3 的 Lyapunov 理论呼应。

## §7.3 操作（任务）空间控制（S12, S13）

**雅可比关系（S12, S13）**：任务空间坐标 $\mathbf x$ 与关节 $\mathbf q$ 经任务雅可比 $J$：
$$\dot{\mathbf x}=J(\mathbf q)\dot{\mathbf q},\qquad \mathbf x_{\rm task}=T(\mathbf x_{\rm base},\mathbf q)\ \text{（S12 式 1）}.$$

**力的对偶映射（S12 式 3）**：任务空间力 $F$ 经任务正运动学映为关节力矩
$$\boxed{\boldsymbol\Gamma=J^\top F.}$$

**操作空间惯性矩阵 (operational space inertia, S12 式 7)**：
$$\boxed{\Lambda=(J\,M^{-1}J^\top)^{-1}}$$
（S12 记 $A$ 为关节空间惯性矩阵，即本书 $M$；约束/优先级情形记 $\Lambda_{t|c}=(J_{t|c}A^{-1}J_{t|c}^\top)^{-1}$）。

**任务空间动力学（S12 式 9，含推导）**：以约束雅可比的动态一致广义逆 $\bar J_{t|c}^\top$ 左乘关节动力学：
$$\bar J_{t|c}^\top\big(A\ddot{\mathbf q}+\mathbf b+\mathbf g+J_{\rm int}^\top\mathbf f_{\rm int}\big)=\boldsymbol\Gamma\ \Rightarrow\ \Lambda_{t|c}\ddot{\mathbf x}_{\rm task}+\boldsymbol\mu_{t|c}+\mathbf p_{t|c}=\bar J_{t|c}^\top\boldsymbol\Gamma-\bar J_{t|c}^\top J_{\rm int}^\top\mathbf f_{\rm int},$$
$\boldsymbol\mu_{t|c}$ 任务级科氏/离心力，$\mathbf p_{t|c}$ 任务级重力，$\mathbf f_{\rm int}$ 交互力，$J_{\rm int}$ 其作用点雅可比。

**任务控制力矩（S12 式 10，补偿非线性）**：
$$\boldsymbol\Gamma_{\rm task}=J_{t|c}^\top\big(\hat\Lambda_{t|c}F_{\rm task}^*+\hat{\boldsymbol\mu}_{t|c}+\hat{\mathbf p}_{t|c}\big)+J_{t|c}^\top\bar J_{t|c}^\top J_{\rm int}^\top\hat{\mathbf f}_{\rm int},$$
$F_{\rm task}^*$ 为达成任务目标的力级反馈律，$\hat{(\cdot)}$ 估计值。若任务在约束下可行，此控制给出**解耦行为** $\ddot{\mathbf x}_{\rm task}=F_{\rm task}^*$。

## §7.4 阻抗控制 (Impedance Control)（S12 式 11–12, S13）

**任务空间阻抗目标动态（S13）**：期望机器人末端表现为质量-弹簧-阻尼
$$\Lambda\ddot{\mathbf e}_x+D_x\dot{\mathbf e}_x+K_x\mathbf e_x=\mathcal F_{\rm ext},\qquad \mathbf e_x=\mathbf x_d-\mathbf x,$$
$K_x$ 任务刚度，$D_x$ 任务阻尼，$\Lambda$ 任务惯性，$\mathcal F_{\rm ext}$ 外力。

**任务空间阻抗控制律（S13）**：
$$\mathcal F=K_x(\mathbf x_d-\mathbf x)+D_x(\dot{\mathbf x}_d-\dot{\mathbf x})+\hat\Lambda(\mathbf q)\ddot{\mathbf x}_d+\hat{\boldsymbol\mu}(\mathbf q,\dot{\mathbf q})+\hat{\boldsymbol\gamma}(\mathbf q)+\hat{\boldsymbol\eta}(\mathbf q,\dot{\mathbf q}),$$
$\hat\Lambda,\hat{\boldsymbol\mu},\hat{\boldsymbol\gamma},\hat{\boldsymbol\eta}$ 为内部模型估计（惯性、科氏、重力、其他）。

**S12 阻抗实现（式 11，含期望视在惯性/阻尼/刚度）**：
$$F_{\rm task}^*=\mathbf a_{\rm des}-M_{\rm des}^{-1}D_{\rm des}(\dot{\mathbf x}_{\rm task}-\mathbf v_{\rm des})-M_{\rm des}^{-1}K_{\rm des}(\mathbf x_{\rm task}-\mathbf x_{\rm des})+M_{\rm des}^{-1}\hat{\mathbf f}_{\rm int},$$
$\mathbf a_{\rm des},\mathbf v_{\rm des},\mathbf x_{\rm des}$ 期望加速度/速度/位置（来自跟踪律），$M_{\rm des},D_{\rm des},K_{\rm des}$ 期望视在惯性/阻尼/刚度（编程给定）。**结果阻抗行为（S12 式 12）**：
$$\boxed{M_{\rm des}(\ddot{\mathbf x}_{\rm task}-\mathbf a_{\rm des})+D_{\rm des}(\dot{\mathbf x}_{\rm task}-\mathbf v_{\rm des})+K_{\rm des}(\mathbf x_{\rm task}-\mathbf x_{\rm des})=\hat{\mathbf f}_{\rm int}.}$$

**混合位置/力控制（S12 式 13）**：$F_{\rm task}^*=\Omega_m F_m^*+\Omega_f F_f^*$，$\Omega_m,\Omega_f$ 为自由运动空间与接触空间的选择矩阵，$F_m^*,F_f^*$ 各自反馈律。

## §7.5 全身控制 WBC 与优先级零空间投影（S12，概览全量）

**控制类别层级（S12）**：复杂行为由三类控制基元 (control primitives) 分层组合：**约束 (constraints)**（最高优先，永不可违反，如平衡/关节限位/自碰撞）、**操作任务 (operational tasks)**（如手/视觉/定位）、**姿态 (postures)**（剩余冗余中的姿态优化）。

**优先级 = 低优先级雅可比向约束零空间的运动学投影（S12）**：定义投影后的虚拟雅可比：
$$J_{t|c}=J_{\rm tasks}N_{\rm constraints}\ \text{（S12 式 4）},\qquad J_{p|t|c}=J_{\rm postures}N_{\rm tasks}N_{\rm constraint}\ \text{（S12 式 5）}.$$
下标 $t|c$ 表"任务运行于约束点的零空间"；$p|t|c$ 表"姿态运行于约束与任务的零空间"。$N_{(\cdot)}$ 为高优先级控制层的**动态一致零空间**矩阵。多基元可合并同优先级，如双手操作任务（S12 式 6）$J_{\rm tasks}=[J_{\rm rightHand};J_{\rm leftHand}]$。

**力矩级层级方程（S12 式 2）**：
$$\boxed{\boldsymbol\Gamma=\boldsymbol\Gamma_{\rm constraints}+N_{\rm constraints}^\top\big(\boldsymbol\Gamma_{\rm tasks}+N_{\rm tasks}^\top\boldsymbol\Gamma_{\rm postures}\big).}$$

**操作空间形式（S12 式，约 §II.C 末）**：式 2 进一步写为
$$\boldsymbol\Gamma=J_{\rm constraints}^\top F_{\rm constraints}+J_{t|c}^\top F_{t|c}+J_{p|t|c}^\top F_{p|t|c},$$
$F_{(\cdot)}$ 为各基元的力向量：$F_{t|c}$ 控任务阻抗/力/轨迹，$F_{\rm constraints}$ 维持约束点的距离/位置/力，$F_{\rm postures}$ 优化期望准则同时提供柔顺交互。

**约束级惯性矩阵（S12 式 8）**：$\Lambda_{\rm constraints}=(J_{\rm constraints}A^{-1}J_{\rm constraints}^\top)^{-1}$（约束已最高优先，无需再约束）。

> **WBC 脉络小结（综合提示）**：WBC = "操作空间动力学（§7.3 的 $\Lambda,J^\top F$）+ 优先级零空间投影"的多任务推广；约束→任务→姿态分层，低优先级在高优先级零空间内"不打扰"地实现。本书若设 WBC 概览，建议以 §7.1 动力学→§7.2 计算力矩（单任务反馈线性化）→§7.3 操作空间（任务空间惯性 $\Lambda$）→§7.5 优先级投影（多任务）的递进组织，所有公式已在本部分备齐。机器人控制的稳定性可统一回挂第三部分 Lyapunov（误差能量 $V$）。

---

# §附 综合写作给本章《控制导论》的建议（清偿/组织清单）

1. **记号统一**：章首必须声明 §0 表中三处致命冲突的解决——(i) 控制 $A,B,C,D$ 与 KF 章 $F,H$ 的对应；(ii) **LQR/MPC 的 $P$（Riccati 代价阵）改记 $S$**，把 $P$ 留给协方差（本书既有）；(iii) LQR 代价权 $Q,R$ 与 KF 噪声协方差 $Q,R$ 同字母异义，分节声明。**这是本章最大踩坑点**。
2. **主线建议**：反馈与稳定性（第一、三部分）→ PID（第二部分，可作"经典控制"代表，例题丰富）→ 状态空间 + 可控可观（第四部分）→ LQR/Riccati（第五部分，给完整 HJB 推导 + Hamilton 矩阵求解）→ MPC（第六部分，给 Theorem 12.2 完整证明，并点明"无约束无限时域 LQR 是其特例"）→ 机器人控制（第七部分，逆动力学→操作空间→WBC，回挂 Lyapunov）。
3. **必给的完整证明（铁律要求，已在本抽取备齐）**：积分作用消差证明（§1.2）；Lyapunov 直接法 + LaSalle 推论（§3.2–3.3，LaSalle 证明骨架需综合时补全并引 Khalil）；Gramian↔秩判据双向证明（§4.2，需综合时补全）；LQR 经 HJB 的逐步推导含 ARE 化简（§5.2）；**MPC Theorem 12.2 完整 Lyapunov 证明（§6.4，已逐行抄录，含式 12.19–12.23）**；计算力矩误差动态推导（§7.2）。
4. **必给的表/伪码**：两张 Z-N 表（§2.5.1）；PID 定性影响表（§2.2，加耦合免责）；PID 数字实现伪码（§2.7.4）；QP-based RHC 算法（§6.2）。
5. **\rebuilt 标注**：全章源自权威教材而非个人笔记，整体标 \rebuilt；§0 已列出每个需"综合时补全证明/条件"的点（LaSalle 证明、Lyapunov 方程积分构造、Gramian 双向证明、Riccati 存在唯一性条件、机器人控制稳定性），这些是"重建·待核对"的重点。
6. **per-formula \cite**：S1（PID）→ astrom2008feedback（需加 refs.bib，返回给主 agent）；S10（MPC）→ borrelli2017predictive；S12（WBC）→ sentis2006wholebody；Mayne 2000 → mayne2000constrained；其余 Wikipedia/Tedrake 条目按编写规范处理（Tedrake → tedrake_underactuated）。**本 agent 不改 refs.bib**，建议的 bib 键名列此供主 agent 集中合并。
