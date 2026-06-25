# 复核报告：Barfoot 吸收 · 滤波族新内容（对抗式核验）

- **复核对象**：`parts/P1_estimation/kalman_eskf.tex`（AI 生成、未人工通读、未编译的滤波族新内容）
- **复核方式**：只读审计 + 独立重算（numpy/sympy 核验关键式），默认怀疑、能证伪则证伪
- **复核日期**：2026-06-18
- **总判**：**有 NEEDS-FIX**（1 项编译级硬伤：4 个参考文献键缺失；其余数学全部 PASS，含 2 项内容/措辞 MINOR）

数学独立重算结论先行：本批新内容的**数学正确性整体过硬**——UKF 矩匹配（含 κ=2 闭式）、N_eff 上下界、DARE 不动点与谱半径、Kalman–Bucy Riccati 化简、SPKF→EKF 交叉项消失、误差转移矩阵 A(I−KC)，逐一独立复现全部吻合。唯一真正会"炸"的是缺失的 bib 键（渲染成 `[?]`）。所谓"修了 Barfoot 一处笔误"的 Kalman–Bucy 段，**没有改错**，与标准 CARE 自洽。

---

## 逐项核验

### 1. ★InEKF（`paper:inekf`，行 969–985）—— PASS（含 2 处 MINOR）

**群仿射式** `f(X1X2)=f(X1)X2+X1f(X2)−X1f(I)X2`：与 Barrau–Bonnabel 2017 群仿射定义**逐字一致**。**右不变误差** `η=X̂X⁻¹`（左不变 `X⁻¹X̂`）：标准定义，正确。**SE₂(3) 5×5 结构**（R/v/p、"双平移"、普通 SE(3) 装不下速度故不满足群仿射）：正确，且"正式定义留 ch:lie"——已核 `ch:lie`（`parts/P0_math/lie_theory.tex`）确有 SE₂(3) 内容（27 处命中），承接成立。"与状态解耦⇒线性化更大范围精确⇒可观测性不被扭曲⇒一致性改善，比 FEJ 更具原理性"：论断准确。

- **MINOR-1（措辞欠精确）**：行 973 "**线性化后**的误差传播与状态轨迹无关（autonomous error / log-linear）"。Barrau–Bonnabel 的核心结论其实更强——**精确（非线性）误差动力学**在群仿射下就已是轨迹自治的（autonomous），"log-linear"指的是该精确误差经 Log 后其**线性化传播矩阵**恰好状态无关，是前者的推论。现写法把"autonomous error"与"log-linear"并列贴在"线性化后"一处，轻微低估了"精确自治"这一更强论断，且把两个概念糅在一句。**建议**：拆为"其**精确**误差动力学是轨迹自治的（autonomous error），故其对数线性化传播矩阵不依赖 X̂（log-linear property）"。不影响正确性，属表述精度。

- **MINOR-2（内容缺口，任务点名）**：任务要求确认"偏置破坏群仿射"的说明。**全文未提**（已 grep：`偏置.*群仿射` 无命中）。Barrau–Bonnabel 框架的著名局限正是——把 IMU 零偏并入状态会**破坏群仿射结构**，需走"imperfect InEKF"（偏置作为群仿射的扰动项近似处理）。本书 `ch:imu` 谈预积分偏置、`ch:vio` 谈 InEKF 去向，此处 `paper` 盒至少应一句点明"把陀螺/加计偏置增广进状态会破坏群仿射，实务用 imperfect InEKF 近似"，否则读者会误以为 InEKF 能无损吞偏置。**建议**：在 SE₂(3) 段末加一句此限制（盒子已自限"确定性动力学"，故只是 MINOR，但任务点名、且是真实概念缺口）。

- **`paper` 环境/`\cref`**：已核 `styles.tex:97` `\elegantnewtheorem{paper}{论文}{thmstyle}{paper}` —— 有独立计数器、cref 前缀为 `paper`，故 `\cref{paper:inekf}` 可解析、渲染为"论文 X.Y"，**合理**。这是全书首次用 `paper` 环境，定义与调用匹配，无忧。

- **引用年份/卷号**：正文（行 970、1236）述为"Barrau–Bonnabel 2017""The Invariant Extended Kalman Filter as a Stable Observer"——**正文叙述正确**（IEEE TAC vol 62, 2017；正文未写卷号，故"42 vs 62"在 prose 中不成问题）。**但**卷号正误的真正落点在 refs.bib，而该键缺失（见下 NEEDS-FIX），补录时务必填 `volume={62}, year={2017}`，勿误填 42。

### 2. SPKF/UKF/ISPKF（`algo:spkf-predict/-correct`、`algo:ispkf`、`prop:spkf-ekf`，行 526–624）—— PASS

- **sigma 点生成 / 权**（`eq:sigma-points`，行 527–533）：2L+1 点、`α₀=κ/(L+κ)`、`αᵢ=1/(2(L+κ))`、和为 1 —— 标准对称 sigma 点集，正确。
- **`ex:three-methods`（f(x)=x²）**：独立复算（mu/sig 多组）——UKF `μ_y=μ²+σ²`（与 κ 无关、恰真值）✓；`σ_y²=4μ²σ²+κσ⁴` 逐组吻合 ✓；**κ=2 时 σ_y²=4μ²σ²+2σ⁴ 与真值完全相同** ✓；"κ 只调四阶矩、令其匹配高斯峰度 3σ⁴ 解出 κ=2"——正确。线性化均值少 σ²、方差少 2σ⁴（过自信）——正确。
- **预测/校正矩匹配盒**：堆叠 [状态;噪声]、块对角 `Σzz`、Cholesky 生点、过 f/h、重组 `μ_y/Σyy/Σxy`、`K=ΣxyΣyy⁻¹`、`P̂=P̌−KΣxy^T`——与 `eq:ggf`（`prop:ggf` 行 483–491）的总接口逐式一致，正确。
- **`prop:spkf-ekf`"块对角⇒交叉项零⇒退回 EKF"**：独立数值复现（块对角 Σzz=diag(P,Σv)，N=3,m=2）——`Σᵢαᵢ(xᵢ−x̄)vᵢ'^T` 最大绝对值 = **0.0**，且重组 Σxx=P、Σvv=Σv 精确还原；于是 `Σyy≈HP̌H^T+Σv'`、`Σxy≈P̌H^T`、`K≈P̌H^T(HP̌H^T+Σv')⁻¹` 退回 EKF 增益。**证明成立**。
- **加性噪声 2N+1 点**（行 598）："加性观测噪声特例下只用 2N+1 个状态 sigma 点（N=dim x），观测噪声以 +Σv 解析补进 Σyy，并用 SMW 把 Σyy⁻¹ 化为对 (2N+1) 维矩阵求逆"——**正确**（加性噪声无需把噪声堆进增广态，故点数从 2(N+m)+1 降到 2N+1；SMW 那句也对）。
- **ISPKF（`algo:ispkf`）**：唯一差别是绕可更新工作点 `x_op` 生点、新息补"工作点不在预测均值"的修正项 `ΣyxΣxx⁻¹(x̌−x_op)`、`P̂=Σxx−KΣyx`——这正是 Sibley 迭代 sigma 点的正确形式（此处用重算的 `Σxx` 而非 `P̌`，因 x_op≠x̌，故与 SPKF 盒的 `P̌−KΣxy^T` 不同是**应然差异**，非笔误）。"首次迭代 x_op=x̌ 退回 SPKF""SPKF=ISPKF 首迭代"——正确。

### 3. 粒子滤波（`algo:pf`、`eq:pf-neff`，行 628–663）—— PASS

- **SIR 三步**（采样→加权→重采样）：bootstrap proposal = 运动模型先验、权退化为纯似然 `w∝p(z|x̌)`，正确。
- **重要性权更新一般式**（行 651）`w_{k,m}∝w_{k−1,m}·p(z|x̌)p(x̌|x̂)/q(x̌|·)`：标准序贯重要性采样权递推，正确；"bootstrap 取 q=运动先验⇒转移项约掉"——正确。
- **`eq:pf-neff` N_eff=1/Σwₖ²∈[1,M]**：独立复算——等权 w=1/M 时 N_eff=M（健康）、退化单粒子时 N_eff=1（最坏），上下界吻合。"按需重采样、阈值常取 M/2、每步必重采样加剧样本贫化、从不重采样任由退化"——全对（与速查表行 1139 一致）。
- **Madow 系统重采样**（行 662）：单随机起点 `ρ~U[0,1/M)`、等步长 `ρ+j/M` 扫累积权 bin、O(M) 一遍、"权>1/M 者至少被选一次、方差小于多项式抽样"——描述正确。
- **RBPF/FastSLAM**（行 665）："轨迹撒粒子、每粒子条件下路标用独立小 EKF 解析、降为路标数×粒子数线性代价"——正确。

### 4. Kalman–Bucy（`eq:kb-model/-mean/-cov`，行 396–409）—— PASS（"修笔误"未改错）

任务重点："该 agent 自称修了 Barfoot 源一处笔误，重点复核有没有改错。"

- **两条耦合 ODE + 连续增益 `K=P̂C^TR⁻¹`**：独立核验——以 `K=P̂C^TR⁻¹` 代入 `eq:kb-cov` 的 `−KRK^T`，得 `−P̂C^TR⁻¹CP̂`，即标准**连续代数 Riccati（CARE）**的二次项（`max|−KRK^T −(−P̂C^TR⁻¹CP̂)|=7e-18`）。即 `eq:kb-cov` 与"连续增益 P̂C^TR⁻¹"**自洽**，稳态 dP̂=0 给标准 CARE，无误。
- **均值式** `x̂˙=Ax̂+Bu+K(z−Cx̂)`：与离散校正同构（开环传播+增益乘新息），正确。注意连续时间增益是 `P̂C^TR⁻¹`（**非**离散的 `P̌C^T(CP̌C^T+R)⁻¹`）——本式用对了连续形式。
- **量纲**：`AP̂+P̂A^T`（A~1/时间，量纲 P/时间）、`+LQL^T`（Q 为功率谱密度 PSD，量纲对）、`−KRK^T`（同量纲），三项同量纲，`dP̂/dt` 量纲自洽。符号结构"+LQL^T 增长、−KRK^T 收缩"与正文叙述一致。
- **判定**：**未发现改错**。无论 agent 修的是哪处（最可能是把某版里写错的 Riccati 项/增益形式订正为标准 CARE 形式），结果与教科书标准 Kalman–Bucy 一致。**PASS**。

### 5. DARE / 稳态 KF（`eq:dare` + 四条件，行 370–384）—— PASS

- **`eq:dare` 不动点**：独立迭代标准预测协方差 Riccati 至稳态 P*，再代入书中"Joseph 形" RHS `A(I−KC)P(I−KC)^TA^T+AKΣvK^TA^T+Σw`——`max|P*−RHS|=5.5e-17`，**精确为不动点**；P* 对称正定。该式确为**预测协方差** P̌ 的稳态（与正文"协方差递推独立于均值先跑"一致），正确。
- **谱半径**：`A(I−KC)` 特征值 [0.683,0.495,0.402]，谱半径 0.683<1 ✓。
- **误差动力学 `E[ě_k]=A(I−KC)E[ě_{k-1}]`**：独立代数推导——ě_k=x_k−x̌_k=A(I−KC)ě_{k-1}−AKv_{k-1}+w_k，取期望即得，**转移矩阵 A(I−KC) 正确**。"对特征对取二次型证 |λ|²<1、可检测性保 Gramian 非退化"论证方向正确。
- **四条件**：(1) Σv>0、(2) Σw≥0、(3) (A,V) 可镇定（VV^T=Σw，Σw>0 时冗余）、(4) (A,C) 可检测——这正是 DARE 唯一半正定解存在的标准充要条件，**正确**。"可镇定/可检测比可控/可观更弱（只要坏模态可控可观）"——表述准确。

### 6. 直方图 / IF / 互补滤波 —— PASS

- **直方图滤波**（`eq:histogram-filter`，行 229–233）：预测 `p̌_i=Σⱼ p(Xᵢ|Xⱼ,u)p̂_{k-1,j}`、校正 `p̂_i=η p(z|Xᵢ)p̌_i`——离散贝叶斯的正确逐格形式；"非参数、能表多峰、维度灾难"定位正确。
- **信息滤波 IF**（`prop:info-filter`，行 334–350）：**预测两次逆**——`Λ̌=(AΛ̂⁻¹A^T+Σw)⁻¹`（内 Λ̂⁻¹ + 外层再逆，确是两次逆）、`η̌=Λ̌(AΛ̂⁻¹η̂+u)`；**校正信息相加**——`Λ̂=Λ̌+C^TΣv⁻¹C`、`η̂=η̌+C^TΣv⁻¹z`。与 KF 同解、代价对偶，全部正确；证明要点（校正即 `eq:kf-info-P/-x` 改记号；预测由 Λ̌=P̌⁻¹、x̌=Ax̂+u、x̂=Λ̂⁻¹η̂ 代入）成立。SEIF"主动置零弱元素换近常数时间、是近似、一致性争议促成转向因子图"——准确。
- **互补滤波**（`eq:complementary-filter`，行 956–963）：SO(3) 上 PI——`R̂˙=R̂(ω_m−b̂+k_P ω_corr)^∧`、`b̂˙=−k_I ω_corr`、`ω_corr=Σᵢ kᵢ v̂ᵢ×(R̂^T vᵢ^ref)`。这是 Mahony 非线性互补滤波的标准形式：叉积构造姿态误差、PI 反馈修正陀螺积分+在线估偏置，**正确**。"高通滤陀螺/低通滤加计、传函互补为 1""无协方差、增益靠整定、极轻量"取舍准确；Madgwick 为梯度下降变体——正确。

---

## 承接 / 记号 / 工程提示 核验

- **\cref 指向**：已逐一核验新内容引用的所有 label 目标**全部存在**——`prop:ggf`、`prop:ekf`、`def:nees-nis`、`thm:bias-overconfident`、`eq:eskf-reset`、`eq:eskf-Fx`、`eq:ggf`、`sec:eskf-appendix`、`sec:nlopt-solve`、`thm:eskf-error-kin`、`eq:eskf-err-theta`、`tab:eskf-pert`、`eq:intro-dw-update`（在 P0_intro）、`ch:lie/control/vio/imu`——无悬空 `\cref`。
- **右扰动主线一致**：InEKF 盒明确"同样右扰动 X Exp(δξ)"、与 ESKF 主线契合；速查表（行 1139/1140/1190+）、谱系表（`tab:filter-taxo` 行 671–687）、记号表（行 1162/1164/1185+）、延伸阅读（行 1236）、跨章去向表（行 1250–1252）均把新条目挂接到位，承接连贯。
- **谱系定位**：`tab:filter-taxo` 把 EKF/IEKF(众数)/UKF/ISPKF(均值)/粒子/直方图归位正确，与正文"IEKF→众数、ISPKF→均值"及数值证据（行 626：iekf=map=24.5694、mean=24.7770、ispkf=24.7414）自洽。
- **独立性**：grep 扫 `external_punt`（详见/参见原书 + 推导留给书）、`ventriloquize`（"Barfoot 指出/认为/写道"作论据）、`narration_dependence`（"按书第 X 节/书中第 X"）——**均无命中**。新内容用 `\cite` 作学术归属（允许），推导与论断自足，**未见独立性违规**。

---

## ★ NEEDS-FIX：4 个参考文献键缺失（编译级硬伤）

**文件**：`refs.bib`（全仓仅此一个 .bib）
**问题**：新内容引用的 **4 个 `\cite` 键在 refs.bib 中完全不存在**，将渲染为 `[?]` 并产生 `Citation undefined` 警告：

| 缺失键 | 引用处（行） | 应补文献 |
|---|---|---|
| `barrau2017invariant` | 970, 973, 1236, 及 `paper:inekf` 全盒 | Barrau & Bonnabel, *The Invariant Extended Kalman Filter as a Stable Observer*, **IEEE TAC vol. 62, no. 4, 2017**（卷号填 62，勿填 42） |
| `sarkka2013bayesian` | 654（`eq:pf-neff` N_eff 判据来源） | Särkkä, *Bayesian Filtering and Smoothing*, Cambridge, 2013 |
| `mahony2008nonlinear` | 956（互补滤波） | Mahony, Hamel & Pflimlin, *Nonlinear Complementary Filters on the SO(3) Group*, IEEE TAC vol. 53, 2008 |
| `madgwick2011estimation` | 963（Madgwick 滤波） | Madgwick, *An efficient orientation filter…*, 2010/2011 技术报告 |

**为何错**：`barfoot2024state/gaoxiang2019slam14/sola2017quaternion/thrun2005probabilistic/carlone2026handbook` 都在 bib 中，唯独这 4 个新引入的键漏录。未编译故未暴露。
**建议改法**：向 `refs.bib` 补这 4 条 BibTeX 条目（卷号/年份见上表，`barrau2017invariant` 务必 `volume={62}, year={2017}`）。这是唯一会导致"渲染出错"的问题，优先级最高。

---

## 最该修的 3–5 项（按优先级）

1. **【NEEDS-FIX，编译级】补 4 个缺失 bib 键**：`barrau2017invariant`（TAC vol **62** 2017）、`sarkka2013bayesian`、`mahony2008nonlinear`、`madgwick2011estimation`。不补则 4 处 `\cite` 全成 `[?]`。
2. **【MINOR，任务点名的内容缺口】`paper:inekf` 补"偏置破坏群仿射"一句**：在 SE₂(3) 段点明"把陀螺/加计零偏增广进状态会破坏群仿射结构，实务用 imperfect InEKF 近似处理"，避免读者误以为 InEKF 无损吞偏置。
3. **【MINOR，措辞】`paper:inekf` 行 973 区分"精确自治"与"log-linear"**：改为"其**精确**误差动力学轨迹自治（autonomous error），故对数线性化传播矩阵不依赖 X̂（log-linear）"，以不低估 Barrau–Bonnabel 的更强论断、并解开两概念的糅合。
4. **【可选，编译验证】补齐 bib 后跑一次编译**，确认 `paper` 环境（styles.tex:97，全书首次使用）渲染为"论文 X.Y"、`\cref{paper:inekf}` 正常解析，并扫 `Citation undefined`/`Reference undefined` 警告归零。

**数学层**：本批 6 大块的关键式经独立 numpy 复算**无一处需改**——UKF 矩匹配（κ=2 闭式）、N_eff 界、DARE 不动点+谱半径+误差转移、Kalman–Bucy CARE 化简、SPKF→EKF 交叉项归零，全部吻合。所谓"修 Barfoot 笔误"的 Kalman–Bucy 段**未改错**。承接、记号、\cref、独立性均 PASS。
