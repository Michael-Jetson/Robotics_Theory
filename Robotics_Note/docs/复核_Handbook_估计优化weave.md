# 复核报告：SLAM Handbook 吸收 —— 估计/优化两章 weave-in 新增部分

**审查对象**
- `parts/P1_estimation/nonlinear_optimization.tex`（2207 行）
- `parts/P1_estimation/slam_state_estimation.tex`（1835 行）

**审查方式**：完整通读两章新织入内容 + 缝合处；硬核数学逐式独立验算（sympy/numpy 数值与符号双核）；独立性/口吻/分工逐段核查。背景已静态干净，未查 label/cite 机械问题。

**严重度计数**：blocker **0** · major **0** · minor **5**（其中 1 条 out-of-scope，2 条为可议的措辞建议）

**一句话结论**：缝合**无缝**，硬核数学**全部验算通过**，独立性红线**已清干净**，两章分工**清晰互指、无重复无矛盾**。仅有少数措辞/定理条件归一化层面的 minor，均不影响正确性，可选择性修订。

---

## 总评（对四条红线的判定）

### 红线 1：无接缝 —— 通过
- **nlopt 三个新节均嵌进原有叙事，非堆章末**。章节顺序为：…位姿图→流形优化→**鲁棒化(§nlopt-robust)**→收敛后协方差→GVI→**可微优化(§nlopt-diffopt)**→**可证最优(§nlopt-certifiable)**→实践。鲁棒节落在「流形优化之后、协方差分析之前」的自然位置；diffopt/certifiable 与 GVI 一道构成「非线性优化的高阶视角」选读簇，每节均有定位 note 承上启下。
- **`sec:nlopt-certifiable` 真兑现了钩子**。`sec:nlopt-why` 的 pitfall（nonlinear_optimization.tex:254）明文埋钩：「可证最优（certifiably optimal）求解器（如位姿图的 SE-Sync，留作前沿）」；`sec:nlopt-robust` 末（:1155）再列其为鲁棒 SLAM 前沿出路。`sec:nlopt-certifiable` 开头（:1642）明文回收：「`\cref{sec:nlopt-why}` 说过…并埋下『可证最优求解器（SE-Sync）留作前沿』的钩子；`\cref{sec:nlopt-robust}` 末也把它列为…前沿出路。本节兑现该钩子」。**钩子—回收闭环成立**。
- **est 新节同样有钩子承接**。`sec:est-fim-laplacian` 承 `sec:est-crlb`（CRLB 一般形式）→ 追问「SLAM 的 FIM 长什么样」；GBP（`sec:est-gbp`）承 `thm:elimination`（精确串行消元）→ 对照「近似并行消息传递」；STEAM 样条（`sec:est-steam-spline`）回收 `sec:est-steam` 开头「连续时间有两条路」的悬念，补全样条一支。过渡自然。

### 红线 2：硬核数学（逐式独立验算）—— 全部通过
| 推导 | 位置 | 验算结果 |
|---|---|---|
| 隐式微分 $\partial\mathbf x^*/\partial\mathbf y=-\mathbf H_{xx}^{-1}\mathbf H_{xy}$ | nlopt:1603 | **数值通过**（与有限差分 Jacobian 差 ~1e-11） |
| 隐式微分链式总梯度 $\nabla_y\mathcal U=\partial_y\mathcal U-\mathbf q^\top\mathbf H_{xy}$, $\mathbf q=\mathbf H_{xx}^{-1}\mathbf v$ | nlopt:1614 | **数值通过**（~5e-10） |
| HVP $=$「梯度·向量积」之梯度 | nlopt:1622 | **符号通过**（H 对称下 $\mathbf H\mathbf q=\nabla_x((\nabla_x\mathcal L)\!\cdot\!\mathbf q)$） |
| FIM $=\boldsymbol{\mathcal L}_w\otimes\mathbf I_3$ 全证明链（旋转不变 (i) + Kron 混合积 (ii)） | est:1276–1283 | **数值通过**（随机 SO(3)，整链差 ~2e-15） |
| 矩阵树推论 $\det\boldsymbol{\mathcal I}=(\det\boldsymbol{\mathcal L}_w)^3$ | est:1299 | **数值通过**（约简拉普拉斯 ⊗ I₃ 行列式） |
| 数值消元例 fill-in 算术（配方、$\mathbf R_\ell=\sqrt2$、$\sigma_\ell^2=\tfrac12$、新因子 $\tau$ 信息 $\tfrac12$、合并得 $\tfrac32$） | est:1176–1187 | **符号通过**（配方恒等式与 $\tau$ 形式精确） |
| Black–Rangarajan：G-M 外点过程 $\Phi=\beta^2(\sqrt w-1)^2$、最优权 $w^*=(\beta^2/(r^2+\beta^2))^2$、回代还原 G-M | nlopt:1117 | **符号通过**（求导、回代精确还原 $\beta^2r^2/(\beta^2+r^2)$） |
| B-R：截断二次 $\Phi=(1-w)\beta^2$ 给 $\min\{r^2,\beta^2\}$ | nlopt:1106 | **通过**（线性于 w，边界取 min） |
| 逆 Wishart MAP 消去 $\mathbf M_i$ 得 Cauchy 核 | nlopt:1088–1099 | 矩阵导数三式标准、$\mathbf M_i$ 解形式与代回逻辑自洽 |
| Cauchy IRLS 权 `eq:nlopt-cauchy-Y` | nlopt:1072 | **符号通过**（$\mathbf Y^{-1}$ 标量因子 $=\alpha/(1+u^2)$） |
| RANSAC $k=\ln(1-p)/\ln(1-w^n)$，与「期望迭代 $1/w^n$」 | nlopt:970 | **数值通过**（n=5,w=0.7 期望 5.95<10 ✓；n=10,w=0.1 → 1e10 ✓） |
| STEAM 样条插值 $(\mathbf T_k\mathbf T_{k-1}^{-1})^\alpha\mathbf T_{k-1}$ | est:1559 | **数值通过**（随机 SE(3)，差 0.0） |
| STEAM 累积基尾和恒等式 | est:1553 | **数值通过**（差 ~1e-16） |
| STEAM 闭式 $\boldsymbol\Phi,\mathbf Q$（$\mathbf A^2=0$ 幂零、积分系数 $\tfrac13\Delta t^3$ 等） | est:1366–1376 | 推导完整、系数正确 |

**SE-Sync 全流程（QCQP→SDP→BM→Staircase）**（nlopt:1652–1737）逐步核查：
- Shor 松弛 `tr(M xx^T)` 提升 + 丢秩-1 → SDP，$d^*\le p^*$、秩-1 即全局最优：**陈述正确**。
- PGO MLE（:1690，旋转 Frobenius + 平移 L2，Langevin 噪声）→ 解析消平移 → 只含旋转 QCQP `min tr(Q̃ R^T R)`（:1696）：**结构正确**（消平移用加权拉普拉斯伪逆，与 est 章 Schur 一脉）。
- 精确性定理（`thm:nlopt-sesync-exact`，:1710）「噪声足够小 $\|\tilde{\mathbf Q}-\bar{\mathbf Q}\|<\beta$ ⇒ 唯一解且秩-d」、Staircase 充分条件（`thm:nlopt-staircase`，:1725）「行秩亏二阶临界点 ⇒ 全局最优」、$r\ge dn+1$ 必终止：**与 Rosen 2019 一致，陈述自包含**。
- Burer–Monteiro $\mathbf Z=\mathbf Y^\top\mathbf Y$、Stiefel 流形、维度 $\tfrac{dn(dn+1)}2\to rnd$、薄 SVD 舍入、证书 `eq:nlopt-sesync-cert`：**正确**。

### 红线 3：独立性 —— 已清干净
- **`sec:est-fg` 开头（est:1019）完全是本书口吻**，无「本节取自 Handbook」punt 语；以「前面把 SLAM 化为最小二乘…因子图给这套最小二乘一个图形语言」承接前文，`\cite{dellaert2017factor,carlone2026handbook}` 仅作出处标注。
- **GBP 的 Davison 愿景被规范框为「前瞻观点」**。`sec:est-gbp` 的第二个 insight 标题即「为什么有人关心 GBP：硬件与多机（**一种前瞻观点，选读**）」（est:1250），正文「近年**一种观点**把 GBP 视为…」+ `\cite{davison2019futuremapping2,ortiz2020bundleipu,murai2024robotweb}`，并「本书对此**点到为止**」。**非 ventriloquize**，是本书在转述并定位他人观点。
- GBP 正文（est:1220–1239）全程本书口吻，消息三式以本章已有工具（信息相加 `prop:info-add` + 舒尔补 `eq:info-marginal`）推出，收敛性结论带 `\cite{weiss2001correctness}`。
- nlopt 三新节均带定位 note 注明「方法用本书口吻陈述，库/系统仅作指引」「定理陈述+引用证明但陈述自包含」，正文为本书自有推导。**未见单书叙述依赖**。

### 红线 4：分工与记号 —— 清晰一致
- **FIM↔拉普拉斯 在 est、SE-Sync 在 nlopt：双向互指、无重复无矛盾**。
  - est `sec:est-fim-laplacian` 收尾 note（est:1303）：「半定松弛/SE-Sync 属**求解器**范畴…见 `\cref{ch:nlopt}`…`ch:slam_est` 讲『要解什么、解得多准』，`ch:nlopt` 讲『怎么解』」。
  - nlopt `sec:nlopt-certifiable` 开头 note（nlopt:1642）：「本节只讲**求解器**；…费舍尔信息与图拉普拉斯的联系属估计精度范畴，见 `\cref{ch:slam_est}`」。
  - 两节各自前置 insight 都重述了「关联矩阵/拉普拉斯/Fiedler 值」代数图论基础——这是**有意的局部自包含**（两章可独立阅读），非冗余堆叠：est 用于「精度极限」、nlopt 用于「消平移化简」，用途不同。判定为合理。
- **右扰动一致**。两章均声明右扰动主线；STEAM 样条 note（est:1564）、STEAM 因子 note（est:1398）显式把 Barfoot 左扰动/左雅可比改写为本书右扰动 $\mathbf T=\mathbf T_{\mathrm{op}}\mathrm{Exp}(\boldsymbol\epsilon)$，经伴随 $\mathcal T=\mathrm{Ad}(\mathbf T)$ 互换；diffopt pitfall（nlopt:1631）提醒隐式微分/HVP 在李群上按右扰动算。一致。
- **`\Lambda` 一致**。两章均以 $\boldsymbol\Lambda=\boldsymbol\Sigma^{-1}$ 为信息矩阵，并显式声明「Barfoot 的 $\mathbf I_k$ 在本书写 $\boldsymbol\Lambda$」（est:104, est:1399, nlopt:1399 附近）。GBP/FIM/SE-Sync 数据矩阵 $\tilde{\mathbf Q}$ 处均有「此 $\tilde{\mathbf Q}$ 非过程噪声 $\boldsymbol\Sigma_w$」之类的消歧（nlopt:1699）。
- **numerator-layout 一致**。GVI（nlopt:1276）明示「分子布局」；隐式微分 `eq:nlopt-implicit-grad`/`eq:nlopt-implicit-final` 的维数与之自洽（已数值验证 $\mathbf v=\partial\mathcal U/\partial\mathbf x^*$ 取列、$\mathbf H_{xy}$ 为 $n_x\times n_y$、总梯度为行）。一致。

---

## Minor 问题清单（按位置）

### M1（minor，可议）— Black–Rangarajan 定理的归一化条件与本书 $\tfrac12$-loss 约定不自洽
**位置**：`nonlinear_optimization.tex:1108–1109`（`thm:nlopt-br`）
**问题**：定理条件写 $\lim_{z\to0}\phi'(z)=1$（$\phi(z):=\rho(\sqrt z)$）。但本书 `tab:robust`/`eq:nlopt-mestim` 的 L2 核取 $\rho_{L2}=\tfrac12u^2$，故 $\phi_{L2}(z)=\tfrac12 z$、$\phi'(0)=\tfrac12$；本节的 Cauchy 运行例 $\rho=\tfrac12\ln(1+u^2)$ 亦 $\phi'(0)=\tfrac12\ne1$。**已验**：以该 $\tfrac12$ 约定，定理条件应为 $\phi'(0)=\tfrac12$（或声明「up to 正比例」）。如按字面 $\phi'(0)=1$，则与本书 Cauchy 例不符；定义里写的 G-M 例 $\rho=\beta^2r^2/(\beta^2+r^2)$（无 $\tfrac12$）反倒满足 $\phi'(0)=1$——即定理与其两个实例用了**两套归一化**。
**影响**：纯属条件陈述的归一化口径；外点过程、权更新机制（已逐一验算）全部正确，不影响任何结论。
**修复建议**：把条件改为 $\lim_{z\to0}\phi'(z)=c>0$（某正常数，由 $\rho$ 的内点二次系数定），或加一句「条件按内点二次系数归一化、本书 $\tfrac12$-loss 下对应 $\phi'(0)=\tfrac12$」。

### M2（minor）— FIM 证明步 (ii) 的混合积律「方向」与上一行表达式略错位
**位置**：`slam_state_estimation.tex:1282`
**问题**：上一行（:1280）的待化简式是 $(\mathbf A\otimes\mathbf I)^\top\,\mathbf R(\mathbf W\otimes\mathbf I)\mathbf R^\top(\mathbf A\otimes\mathbf I)$，即左端实际带 $\mathbf A^\top$；而步 (ii) 引用的恒等式写成 $(\mathbf A\otimes\mathbf I)(\mathbf W\otimes\mathbf I)(\mathbf A^\top\otimes\mathbf I)=(\mathbf A\mathbf W\mathbf A^\top)\otimes\mathbf I$（泛式形）。读者需自行把泛式中的 $\mathbf A$ 当作此处的 $\mathbf A^\top$ 代入才对得上。**已验**：以正确朝向 $(\mathbf A^\top\otimes\mathbf I)(\mathbf W\otimes\mathbf I)(\mathbf A\otimes\mathbf I)=(\mathbf A^\top\mathbf W\mathbf A)\otimes\mathbf I=\boldsymbol{\mathcal L}_w\otimes\mathbf I$ 数值精确成立。结论无误。
**影响**：极轻微的可读性错位，结果正确。
**修复建议**：步 (ii) 的恒等式直接按本例朝向写成 $(\mathbf A^\top\otimes\mathbf I)(\mathbf W\otimes\mathbf I)(\mathbf A\otimes\mathbf I)=(\mathbf A^\top\mathbf W\mathbf A)\otimes\mathbf I$，与上一行严格对齐。

### M3（minor，out-of-scope）— 残留单书叙述依赖语（在**非** weave-in 的旧节）
**位置**：`slam_state_estimation.tex:725`（`sec:est-normal`，线性高斯正规方程节）
**问题**：「本节主线**取自** Barfoot 第 3 章，逐式复现。」属独立性标准所禁的单书叙述依赖措辞。
**影响**：(a) 关于 Barfoot 而非 Handbook；(b) 位于**既有**章节，非本次审查的 weave-in 新增部分；(c) 与本任务四条红线无直接关系。**仅作顺带记录**，建议清理时一并处理（改为「本节沿用 Barfoot…的线性高斯框架，并以本书记号重述」之类）。同类 `sec:est-gauss` 开头（:163）「所有结论摘自 Barfoot 第 1 章…本节将其完整复现」亦同。
**修复建议**：全书独立性清理时统一软化此类「取自/摘自…逐式复现」措辞（已在十四讲吸收审计的「全书独立性待清理」范围内）。

### M4（minor，措辞）— `eq:fg-ls-early`「Handbook 视角」段落措辞偏「两书对照」而非纯本书口吻
**位置**：`slam_state_estimation.tex:686–692`（`thm:map-ls` 之后，**旧节**内）
**问题**：「Handbook 视角：因子图上同一结论」+「两书视角对照：Barfoot 从…（更严谨），Handbook 直接以因子图为对象（更直觉…）」。这是**平衡多书对照**（独立性标准明确鼓励），措辞得当、非依赖单书；但与 `sec:est-fg` 正式展开因子图略有内容预演重叠。
**影响**：无错，属风格观察。该段实为合理的「先给等价命题、再 `\cref{sec:est-fg}` 正式展开」过渡。
**修复建议**：可不改；若求紧凑，可把此段并入 `sec:est-fg` 或精简为一句指引。

### M5（minor，措辞精度）— GNC G-M 还原描述虽自洽但易让读者与标准文献对表混淆
**位置**：`nonlinear_optimization.tex:1141–1149`（`eq:nlopt-gnc-gm` 及算法步 (3)）
**问题**：文中 GNC G-M 取 $\rho_\mu=\mu\beta^2r^2/(\mu\beta^2+r^2)$，并述「$\mu\to\infty$ 时凸、$\mu=1$ 时还原 G-M」，算法步 (3) 对 G-M 取 $\mu\leftarrow\mu/\gamma$（$\mu$ 由大降到 1）。**已验**：$\mu\to\infty\Rightarrow\rho_\mu\to r^2$（凸）、$\mu=1\Rightarrow$ G-M，**内部完全自洽**。仅提示：部分文献（含 Yang 2020 原文的另一参数化）用相反的 $\mu$ 方向，读者跨文献对表时需注意本书约定。
**影响**：无错，纯跨文献对照提示。
**修复建议**：可加半句脚注「本书 $\mu$ 方向与某些文献相反，以『$\mu$ 大端凸、向原核收紧』为准」。

---

## 附：未发现的问题（已专门核查、确认无误）
- 隐式微分梯度符号 $-\mathbf H_{xx}^{-1}\mathbf H_{xy}$ 的正负与布局：**正确**。
- SE-Sync 精确性条件（小噪声、唯一解、秩-d）与 Staircase 终止性：**与原文一致、自包含**。
- 数值消元例两条对 $\ell$ 测量「信息翻倍、方差减半」与 `prop:info-add` 的接续：**正确**。
- GBP 有环图「收敛则均值精确、协方差偏乐观」+ `\cite{weiss2001correctness}`：**陈述正确**。
- 截断二次 ↔ 最大共识 ↔ RANSAC 的「鲁棒核↔共识最大化」桥（nlopt:1050,1043）：闭环正确。
- est ↔ nlopt 的 §nlopt-cov「逆海森三位一体（后验协方差/Fisher 逆/CRLB）」与 est CRLB 节：一致、互指。
- eskf 章确有 `eq:robust-kernels`/`thm:robust-iw`（kalman_eskf.tex:1061,1071），印证 `sec:nlopt-robust` 开头「要点版（eskf）+ 完整版（本节）、字母已统一」之说为真。
