# Barfoot 估计核心（ch1–ch5）吸收审计 —— 扫描式 gap 报告

> 审计对象：Barfoot《State Estimation for Robotics》2nd ed 的估计骨架（源目录 ch1 概率基元 / ch2 线性高斯 / ch3 非线性非高斯 / ch4 非理想 / ch5 变分推断；其内部式号分别为 2.x/3.x/4.x/5.x/6.x）。
> 落点：`parts/P1_estimation/{slam_state_estimation, kalman_eskf, nonlinear_optimization}.tex` + `parts/front/notation.tex` + `parts/appendix/matrix_calculus.tex`，并已对 `parts/` 全树做 grep 核查“是否挪到别章”。
> 标准 A（知识全量吸收：每步推导/每例/每定理证明/每表）、标准 C（独立性：external_punt / ventriloquize / narration_dependence）。只报 confirmed_missing 与 partial_thin；found_elsewhere 已剔除。

## 总体结论

**Barfoot 估计核心 ch1–ch4 吸收度很高（估计 ≳90% 的不可替代严谨内容已落地并自包含重写），ch5 变分推断几乎整章未吸收（confirmed_missing）。** 三章分工清晰且与 Barfoot 三条主线对齐：`slam_state_estimation` 全量复现 ch1（概率/高斯工具箱）+ ch2（线性高斯批量、正规方程、可观测性、批量↔递归等价）；`kalman_eskf` 复现 ch3（贝叶斯滤波、KF、EKF、广义高斯、IEKF、sigma-point/UKF、ISPKF、粒子滤波、滤波器谱系）+ ch4 的一致性/偏置/协方差估计/RANSAC；`nonlinear_optimization` 复现 ch3 批量 GN/LM + ch4 的鲁棒代价/IRLS/逆 Wishart/ML 偏差/NEES-NIS。记号映射（右扰动主线、$\boldsymbol\Sigma_{\mathbf w}/\boldsymbol\Sigma_{\mathbf v}$、$\boldsymbol\Lambda$ 信息阵、check/hat、把 $\mathbf R$ 留给旋转）处理得当，并在 `kalman_eskf` 显式给出与 Barfoot（$\mathbf v$=输入、$\mathbf n$=观测噪声、$\mathbf Q/\mathbf R$、$\mathbf C/\mathbf A$）的“跨书雷区”对照表。剩余 gap 集中在：**(1) 整章变分推断缺失；(2) 若干 ch3/ch4 二级方法只给叙述未给逐式算法盒（sigma-point/UKF/粒子滤波/ISPKF）；(3) 连续时间/GP 与 Kalman-Bucy 偏薄或缺。** C 独立性整体合格：核心推导自包含（“逐式复现，读者无需翻原书”），`\cite{barfoot2024state}` 多为合法出处标注与平衡对照；个别“Barfoot 得…/Barfoot 推测…”属轻度 ventriloquize，详见各节。

---

## ch1 — 概率基元（Barfoot 2.x）→ 主落点 `slam_state_estimation.tex §2`

**总判：A 近全量吸收，自包含重写质量高。** ch1 的 31 个知识单元中，核心严谨件（高斯联合/边缘/条件 via Schur、贝叶斯、CRLB/Fisher、Shannon/互信息/KL、SMW、信息形式、过非线性、Stein、Isserlis、样本协方差/Bessel、归一化积、卡方/马氏）逐条落地于 §2.1–2.9，多数带完整证明（Schur 条件化、SMW、卡方均值迹技巧均在正文或附录给出）。`matrix_calculus.tex` 附录补齐迹技巧/矩阵求导地基，与 ch1 的 SMW/熵求导对接。

### A 知识缺口
- **partial_thin — Q-Q（分位–分位）图与随机采样的逆变换细节**：Barfoot 2.1.6–2.1.7（Q-Q plot、quantile function、`x=Q(y)` 逆变换采样）。教材 §2.7 给了一维 `x=√2·erf⁻¹(2y-1)` 与多元 `μ+Vx_std` 采样，但 **Q-Q 图整体未提**（它是“比较近似 vs 真后验”的可视化工具）。影响小，属诊断性可视化，可在一致性/变分小节顺带补一句。
- **partial_thin — 高斯过非线性的“确定性变化变量例”（`y=exp(x)` 精确 PDF）**：Barfoot 2.2.7（eqs 2.76–2.82，先给确定性非线性的精确换元 PDF，再过渡到随机非线性线性化）。教材 §2.4 `prop:gauss-nonlinear` 直接给随机非线性的线性化结论（EKF 来源），**略去了“确定性非线性把高斯掰成非高斯”的那个精确例子**——而这正是后面 `kalman_eskf` 立体相机“后验偏斜”的概念前置。建议在 §2.4 或立体相机引例处补这条直觉桥。
- （注：MC/一阶/二阶/四阶/sigma-point 的完整不确定度传播——Barfoot 2.2.7 + §7.4 的延伸——**found_elsewhere**：已在 `camera_model.tex §1039-1069` 用 Barfoot 双目数值实验全量落地，含 `ex:cam-unc`；故不计为 ch1 缺口。）

### C 独立性问题
- 合格。§2 开篇明言“所有结论摘自 Barfoot 第 1 章…本节将其完整复现并合入十四讲的直觉”——属合法 narration（声明吸收来源 + 自包含重写），非依赖原书阅读。Schur/SMW 证明均自带，未 punt。

---

## ch2 — 线性高斯估计（Barfoot 3.x）→ 主落点 `slam_state_estimation.tex §4–7`

**总判：A 全量吸收，质量最高的一章。** 批量 MAP=稀疏最小二乘（`thm:normal-eq` 正规方程 $\mathbf H^\top\mathbf W^{-1}\mathbf H\hat{\mathbf x}=\mathbf H^\top\mathbf W^{-1}\mathbf z$）、贝叶斯=MAP=信息三路等价（`thm:map-bayes`，含 lifted 构造 + Schur 条件化 + SMW）、存在唯一性=可观测性（`eq:observability` 观测矩阵 + Cayley-Hamilton + 质点弹簧直觉）、MAP 协方差=逆 Hessian=Fisher=CRLB、块三对角稀疏、RTS/Cholesky 平滑“是批量 MAP 的稀疏递归”——全部落地。KF 作为“批量 MAP 前向递归”的缝合（`thm:kf-from-map`）在本章给定性、在 `kalman_eskf` 给逐式。

### A 知识缺口
- **partial_thin — Cholesky 平滑器的逐式块递归与平滑器协方差回收**：Barfoot 3.2.2（eqs 3.60–3.67 的 $\mathbf L$ 块前向/后向递归）、3.2.3（从 $\mathbf L$ 回收 $\hat{\mathbf P}_k$ 的后向递推）。教材 §6.1 把 Cholesky 平滑“结构”讲清（块下双对角、$O(N^3(K+1))$、前向=KF/前向+后向=RTS），但 **未给 Barfoot 那套 $\mathbf L_{k,k-1}/\mathbf L_k$ 的具体块递归式**；RTS 后向均值/协方差递推在 `kalman_eskf eq:rts/eq:rts-cov` + 附录补全，但 **Barfoot 3.2.3 的“从 Cholesky 因子直接回收边际协方差”那条单独算法未单列**。属“结构全、逐式算法盒缺”，对理解无碍，对实现略欠。
- **confirmed_missing — Kalman-Bucy 连续时间滤波器**：Barfoot 3.5.2（eqs 3.250–3.254，连续时间 LTV 的两条耦合 ODE $\dot{\hat{\mathbf x}},\dot{\hat{\mathbf P}}$ + 连续增益 $\mathbf K=\hat{\mathbf P}\mathbf C^\top\mathbf R^{-1}$）。全树 grep `Bucy/布西` **零命中**。属经典完备性条目，机器人实践少用，缺失影响低，但 Barfoot 主线有、教材无。
- **partial_thin — 稳态 KF 与离散代数 Riccati（DARE）的条件**：Barfoot 3.3.7（DARE、可镇定/可检测、误差动力学稳定性 $|\lambda(\mathbf A(\mathbf I-\mathbf K\mathbf C))|<1$）。教材 `kalman_eskf §321` 仅一句“时不变系统稳态协方差满足离散代数 Riccati 方程…可用常值增益”，**未给 DARE 方程本身与存在唯一性的四条件**。

### C 独立性问题
- 合格。§4–7 反复声明“本节主线取自 Barfoot 第 3 章，逐式复现”，证明自带。`\cite` 用于出处与并列，未见“在 Barfoot 中…”式叙事依赖。连续时间/GP 一瞥（§867）以 `\cite{barfoot2024state}` 收尾属合法点到为止。

---

## ch3 — 非线性非高斯估计（Barfoot 4.x）→ 主落点 `kalman_eskf.tex`（滤波族）+ `nonlinear_optimization.tex`（批量 GN/LM）

**总判：A 主干全量、二级方法 partial_thin（叙述充分但缺逐式算法盒）。** 贝叶斯滤波（`thm:bayes-filter`，四步推导 + “精确但不可算”双障碍）、EKF（`prop:ekf` 线性化预测-校正 + 三宗罪）、广义高斯滤波（`prop:ggf`，Barfoot 路线的联合高斯条件化）、IEKF→众数=MAP、ISPKF→均值、滤波器谱系表（`tab:filter-taxo`）、立体相机偏斜引例（含 Barfoot 的 $\check x=20$m 等参数与 $10^6$ 次试验 $e_{\rm mean}\approx-33.0$cm/$e_{\rm sq}\approx4.41$m² 数值）、批量非线性 GN（`thm:gn` 正规方程 + “EKF≈GN 一次迭代、IEKF=单时刻 GN”两伏笔的兑现）、Newton/LM/信赖域/Dogleg、白化/Cholesky/QR、稀疏/Schur/BA、流形右扰动优化——全部落地且多带完整推导。

### A 知识缺口
- **partial_thin — sigma-point/无迹变换、SPKF/UKF、ISPKF 的逐式算法**：Barfoot 4.2.7/4.2.9/4.2.10（sigmapoint 生成 eqs 4.47-4.49、SPKF 预测 eqs 4.71-4.76 与校正 eqs 4.79-4.89、ISPKF eqs 4.98-4.106，含与 EKF 等价性推导和 SMW 化简）。教材把这些放在 `kalman_eskf §5 (sec:nonlin-pdf)` 且标“次优”脉络层：**给了 sigma 点定义式 `eq:三点/2L+1`、$f(x)=x^2$ 对比例（`ex:three-methods`，含 κ=2 复现精确均值/方差）、谱系定位，但 SPKF/UKF/ISPKF 本身只作算法叙述，未给 Barfoot 那样的预测/校正逐步算法盒与 Σ_yy↔EKF 等价推导**。这是 ch3 最大的“广度有、深度按 A 标准偏薄”项——A 标准要求“每步推导不删减”，而 UKF 的矩匹配步、ISPKF 的迭代式被压缩。
- **partial_thin — 粒子滤波/序贯蒙特卡洛与重采样**：Barfoot 4.2.8（bootstrap/condensation 算法步、importance weighting eqs 4.65-4.69、Madow 系统重采样 eq 4.70）。教材 `kalman_eskf §478-479` 仅数句（加权样本、按权重 Madow 重采样、维度灾难/退化），**无算法盒、无重采样具体式**。属“点到为止”，但 Barfoot 给了完整算法，按 A 标准计 partial_thin。
- **partial_thin — 滑动窗滤波（SWF）的窗扩/窗缩边缘化逐式**：Barfoot 4.3.4（窗扩 eq 4.186、Schur 边缘化窗缩 eq 4.188、递归块式 eq 4.197）。教材 `kalman_eskf §498`（选读）+ `nonlinear_optimization 附录 §1403`（`derivation` 给 Schur 边缘化 $\mathbf x_0$ 的核心式）已覆盖原理与边缘化机理，**但窗扩/窗缩的完整递归块更新未逐式展开**。基本达标，按 A 严格计轻度 thin。
- （注：Box(1971) ML 偏差闭式——Barfoot 4.3.3——**found_elsewhere/达标**：`kalman_eskf 附录 §1083` 与 `nonlinear_optimization §1058+附录` 均给闭式 `eq:nlopt-mlbias` 与去偏；Laplace 协方差、连续时间 GP 批量也已落地，不计缺口。）

### C 独立性问题
- 基本合格，**轻度 ventriloquize 数处**，建议下一轮改写为本书口吻：
  - `kalman_eskf §1084` 附录：“…代入收敛最优条件并对噪声取期望，**Barfoot 得系统偏差**…”——把推导结论归于 Barfoot（narration 化）。结论已在本书附录复算，建议删“Barfoot 得”改“可得/由此得”。
  - `nonlinear_optimization §211/§343`：“**Barfoot 的立体相机例**…显示…”“**Barfoot §4.2.6 证明** IEKF 收敛到…众数即 MAP”——属合法出处指引，但“Barfoot §4.2.6 证明”是把关键定理的证明 punt 给原书（IEKF=MAP 的收敛性本书未自证）。**potential external_punt**：若要满足 A（每定理证明不删减），IEKF→MAP 的收敛论证宜在本书给出或在附录补证，而非仅引 §4.2.6。
  - 立体相机数值（$-33.0$cm 等）直接复用 Barfoot 实验值并标 `\cite`——属合法引用，但本书未独立重算/重绘直方图（Barfoot 4.1.2 的 Fig 4.4 histogram 未复现），按 A“每例不删减”计为 partial_thin 的例证缺图。

---

## ch4 — 非理想处理（Barfoot 5.x）→ 主落点 `kalman_eskf.tex §6,§8` + `nonlinear_optimization.tex §11–12`

**总判：A 高度吸收。** 无偏/一致定义、NEES/NIS（`def:nees-nis`，卡方检验）、未补偿偏置使 KF 过自信且无界增长（`thm:bias-overconfident`，含误差动力学证明）、偏置可观测性（状态增广 + 三例可观/不可观，含 SLAM 测量偏置不可观=全局平移零空间）、协方差估计（监督式样本协方差含 Bessel + 自适应拖尾窗反解）、数据关联（外部 vs 内部 + DARCES/星座）、RANSAC（五步 + 迭代次数 $k=\ln(1-p)/\ln(1-w^n)$）、M-估计/鲁棒核（L2/Huber/Cauchy/Geman-McClure 表 + 影响函数）、IRLS（完整推导 + Cauchy 重加权例）、鲁棒=逆 Wishart 先验 MAP（`thm:robust-iw` / `nonlinear_optimization §1001` 完整证明，等价 Barfoot 5.5.3 MAP 协方差估计）、GNC/自适应核族（Barron 2019、Yang 2020）——全部落地。

### A 知识缺口
- **partial_thin — Tukey（biweight）鲁棒核**：教材鲁棒核表只列 L2/Huber/Cauchy/Geman-McClure 四款。Barfoot 5.4.2 主举 quadratic/Cauchy/Geman-McClure（亦未列 Tukey），故对 Barfoot 而言 **不算缺**；但若以“鲁棒核全家福”为 A 目标，Tukey 缺位值得一句补全（影响极小，记此仅作完备性提示）。
- **partial_thin — MAP 协方差估计的 inverse-Wishart 推导“定位”问题**：Barfoot 5.5.3 把它作为 *covariance estimation* 的第三法（与监督/自适应并列，eqs 5.92-5.100），结论是“消去 $\mathbf M_i$ 后退化为加权 Cauchy”。教材把同一推导放进 `nonlinear_optimization §11 鲁棒核`（`eq:nlopt-robust-cauchy`），**逻辑完整且证明齐全**，但 **未在 `kalman_eskf` 协方差估计小节（监督/自适应两法处）回链“第三法=鲁棒核”**——读者可能感知为两处割裂而非 Barfoot 的统一三法。属编排/交叉引用 thin，非知识缺。
- **达标确认（非缺口，供放心）**：能控性/可观测性——Barfoot 5.x 仅线性可观测（秩判据）+ Ex 5.1 脚注一句 uncontrollable；教材 `slam_state_estimation §5.2`（可观测性矩阵）+ `imu_model §900`（AINS 四维不可观零空间）+ `control_intro §583`（可控性/Gramian/PBH/可镇定可检测）**覆盖度反超 Barfoot**。

### C 独立性问题
- 合格。NEES/NIS、RANSAC、IRLS、逆 Wishart 均自包含推导，`\cite{barfoot2024state}` 为出处。`nonlinear_optimization §1026` “**Barfoot 推测**所有鲁棒核都源自协方差先验某种选择（Black-Rangarajan 1996 起点）”——属对原作者观点的合法转述（明确是“推测/conjecture”，非把本书结论假托），可保留。

---

## ch5 — 变分推断（Barfoot 6.x）→ 落点：**几乎无**

**总判：confirmed_missing（整章未吸收），且未在任何前言/小节明确“定位为选读或刻意略去”。** Barfoot ch5 是 2nd ed 新增、132 个知识单元的独立章：ELBO/KL 反向散度的推断表述、Gaussian Variational Inference（GVI）、精确稀疏 ESGVI（factor-graph VI + Takahashi 协方差回收）、自然梯度、Stein 引理在 GVI 中的应用、EM 参数估计/系统辨识、GVI 在线性系统恢复批量解、GVI 滤波校正步（后验统计线性化，与 ISPKF 对照）、立体相机 GVI vs MAP vs ISPKF（GVI 偏差仅 0.28cm，远优于 MAP 的 -33cm）。

全树 grep `变分/variational/ELBO/证据下界/ESGVI/GVI/自然梯度`：仅三处擦边 —— `slam_state_estimation §428`（KL 散度“在比较近似 vs 真后验如变分推断时用作度量”一句带过）、`planning_intro §1001`（CHOMP 的 $\mathbf A^{-1}\nabla$ 类比“自然梯度”，与估计无关）、`control_intro §867`（变分/极小值原理推 LQR，与变分推断无关）。**ELBO、GVI、ESGVI、GVI 参数估计/系统辨识、GVI 滤波校正——零落地。**

### A 知识缺口（全部 confirmed_missing）
- **confirmed_missing — 整个 Gaussian Variational Inference 框架**：Barfoot 6.1–6.2（ELBO/负 ELBO 损失泛函 $V(q)$、KL(q‖p) 选择理由、对 $\boldsymbol\mu$/$\boldsymbol\Sigma^{-1}$ 的更新、Hessian-梯度关系、收敛保证）。
- **confirmed_missing — 精确稀疏 ESGVI**：Barfoot 6.3（因子化 $\phi=\sum\phi_k$、边际期望、Takahashi 稀疏协方差回收 eq 6.45/6.48、cubature/无迹积分实现、无导数推导）。这是 Barfoot 把变分推断接回因子图/稀疏后端的关键，与本书 `slam_state_estimation §7` 因子图主线本可强呼应。
- **confirmed_missing — 自然梯度、Stein 在 GVI、GVI 的 EM 参数估计与系统辨识**：Barfoot 6.2.3/6.2.4/6.4/6.5.2。
- **confirmed_missing — GVI 滤波校正步与立体相机 GVI 实验**：Barfoot 6.6（GVI 取后验协方差做统计线性化、与 ISPKF 的“先验 vs 后验线性化”对照、GVI 偏差 0.28cm 的数值）。这条与本书 `kalman_eskf` 立体相机贯穿例直接同源，最具“补一段就闭环”的价值。

### C 独立性问题
- 不适用（内容缺失）。**建议**：变分推断属 Barfoot 估计骨架的“顶层统一视角”（generalizes MAP+Laplace），但相对 SLAM 核心管线（批量 MAP/KF/EKF/BA）确为进阶/选读。下一轮至少应在 `nonlinear_optimization` 或 `kalman_eskf` 末尾以一节“变分推断：MAP/Laplace 的推广（选读）”给出 ELBO + GVI 骨架 + 与立体相机例的闭环，并显式标注“选读/前沿”定位——目前是“静默缺失”，连定位声明都没有。

---

## 最重要的 3–5 个 gap（按下一轮润色优先级）

1. **【最高 · ch5 整章】Gaussian Variational Inference 完全缺失且无定位声明。** 至少补一节（选读）：ELBO/$V(q)$ → GVI 更新（$\boldsymbol\mu,\boldsymbol\Sigma^{-1}$）→ ESGVI 稀疏（接 `§7` 因子图）→ GVI 滤波校正 + 立体相机 GVI vs MAP vs ISPKF 闭环（GVI 偏差 0.28cm 直接续用本书既有立体相机贯穿例）。落点建议：`kalman_eskf` 末或新增小节，明确标“前沿/选读”。

2. **【高 · ch3 深度】sigma-point/UKF · SPKF · ISPKF · 粒子滤波只有叙述，缺逐式算法盒（A 标准“每步推导不删减”未满足）。** 在 `kalman_eskf §5` 把 Barfoot 4.2.9/4.2.10/4.2.8 的预测/校正逐步算法、UKF 的 $\boldsymbol\Sigma_{yy}$↔EKF 等价、ISPKF 迭代式、粒子滤波 + Madow 重采样补成正式算法盒（目前是“次优”叙述层）。

3. **【中高 · ch3 独立性】IEKF→MAP 收敛性的证明 punt 给 Barfoot §4.2.6（potential external_punt）。** `nonlinear_optimization §343` 仅引“Barfoot §4.2.6 证明 IEKF 收敛到众数即 MAP”。按 C 独立性 + A“每定理证明”，宜在本书自证或附录补证；同时把附录 §1084“Barfoot 得系统偏差”改为本书口吻。

4. **【中 · ch2 完备性】Kalman-Bucy 连续时间滤波（confirmed_missing）+ 稳态 KF 的 DARE 方程与可镇定/可检测条件（partial_thin）。** Barfoot 3.5.2 / 3.3.7。在 `kalman_eskf` 稳态 KF 处补 DARE 与四条件，并加一段 Kalman-Bucy（哪怕选读）。

5. **【中 · ch1/ch3 例与图】两个“概念桥”性质的例缺失：** (a) Barfoot 2.2.7 的 `y=exp(x)` 确定性非线性精确 PDF（高斯被掰歪的最小例，做立体相机偏斜的前置直觉）；(b) Barfoot 4.1.2 立体相机 MAP 偏差直方图（Fig 4.4）本书复用了数值但未复现图/独立重算。按 A“每例不删减”补上，强化“mode≠mean”的视觉冲击。

---

*附：found_elsewhere（已核查、不计缺口）速记 —— MC/一/二/四阶/sigma-point 不确定度传播→`camera_model.tex §1039-1069`（Barfoot 双目数值实验全量）；连续时间 GP 回归/STEAM→`slam_state_estimation §867` + `nonlinear_optimization 附录 §1411`（点到为止+derivation）；Box 1971 ML 偏差→`kalman_eskf 附录` + `nonlinear_optimization §1058`；能控/可观→`control_intro §583` + `imu_model §900` + `slam_state_estimation §5.2`（覆盖反超 Barfoot）；SWF 边缘化→`nonlinear_optimization 附录 §1403`。*
