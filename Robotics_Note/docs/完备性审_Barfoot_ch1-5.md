# 完备性审：Barfoot《State Estimation for Robotics》第 1–5 章吸收审核

> 审核对象：Barfoot SER2 ch1（引论）/ ch1 概率基元（书内 2.x）/ ch2 线性高斯（3.x）/ ch3 非线性非高斯（4.x）/ ch4 非理想（5.x）/ ch5 变分推断（6.x）。
> 落点：`parts/P1_estimation/{slam_state_estimation, kalman_eskf, nonlinear_optimization}.tex` + `parts/front/notation.tex` + `parts/appendix/matrix_calculus.tex`，并 grep 全树核查是否挪到别章。
> **口径**：吸收 = 五维（(a) 知识 (b) 讲解过程 (c) 分析过程 (d) 脉络 (e) 思想/洞见）用本书口吻重写到位，不止公式落地，更看“让人真正理解的讲法/分析/脉络/思想有没有传达”。重点抓退化（结论在、推导链与洞见丢）。
> **方法**：完整通读源书 ch1–5 + 逐节核对教材落点 + 全树独立性扫描。本审为独立复核（换一批眼睛对抗式重核基线 `docs/Barfoot吸收审计.md` 等）。只读，不改任何 .tex/refs.bib。
> 日期 2026-06-18。

---

## 总体结论（与旧基线的重大出入）

**Barfoot ch1–5 现已被五维完整吸收，质量很高、自包含、独立性合格。** 这与旧基线 `docs/Barfoot吸收审计.md` 的结论（“ch1–4 高、**ch5 整章未吸收 confirmed_missing**；UKF/粒子/ISPKF/SWF 缺逐式算法盒；IEKF→MAP 证明 punt 给原书；Kalman–Bucy/DARE 缺；Q-Q 图缺；Tukey 缺”）**已显著不同**——旧基线列的 gap 在本会话此前的主吸收 + 复核轮里**几乎全部补齐**。本次独立复核确认：

- **ch5 变分推断已整章吸收**，落 `nonlinear_optimization.tex §高斯变分推断`（`sec:nlopt-gvi`，行 1223–1541 + 推导附录 `der:nlopt-gvi-deriv` 行 2110）：ELBO/负 ELBO 损失 `V(q)`、KL(q‖p) 选择理由（mode-seeking vs mass-covering，含脚注几何后果）、对 μ/Σ⁻¹ 的牛顿式更新 + 局部收敛证明（`tr(ABAB)≥0` vec/Kron 论证）、自然梯度等价、Stein 引理双向用法、**精确稀疏 ESGVI**（似然因式分解→边缘期望、Takahashi 协方差回收 + “盒子四角”规则、cubature 无导数实现）、ESGVI-GN/Jensen 保守变体、EM 参数估计/系统辨识、线性恢复批量解、**GVI 滤波校正步 + 与 ISPKF 的“先验 vs 后验统计线性化”对照**、立体相机 GVI vs MAP vs ISPKF 三方表（GVI 偏差 0.28cm，闭环全书贯穿例）。
- **ch3 二级方法已给逐式算法盒**：`kalman_eskf.tex` 有 `algo:spkf-predict/-correct`（SPKF/UKF 预测+校正逐步）、`prop:spkf-ekf`（轻非线性极限 Σyy↔EKF 增益等价，附证）、`algo:ispkf`（ISPKF 迭代）、`algo:pf`（SIR/bootstrap）+ `eq:pf-neff`（N_eff 退化判据）+ proposal/Madow 重采样。
- **IEKF→MAP 已自证**：`thm:nlopt-iekf-map`（行 1503）三步完整证明（GN 步→SMW 化卡尔曼增益→不动点=驻点=众数），不外引，原基线的 `external_punt` 已消除。
- **Kalman–Bucy（`eq:kb-*`）、DARE 四条件（`eq:dare`）、Q-Q 图（`slam_state_estimation §sampling`）、Tukey 核（`tab:robust` + redescending 讨论）、SWF 窗扩/窗缩逐式（`der:nlopt-swf-recursion`）、Cholesky 平滑器逐式 + Takahashi 协方差回收（`alg:est-cholesky-smoother`/`alg:est-cov-recovery`）** 均已补齐。
- **独立性扫描全树零命中**：P1 三章无 `external_punt`（证明从略/详见原书）、无 ventriloquize（“Barfoot 得/证明/坦言/指出/写道”）、无 narration_dependence。`\cite{barfoot2024state}` 均为合法出处标注与平衡对照。旧基线点名的“Barfoot 得系统偏差”“Barfoot §4.2.6 证明”句已不存在。

剩余仅 **2 项极轻微残留**（均不影响理解，下文 §残留）。下面给逐章五维覆盖表。

---

## ch1 — 引论（Barfoot 第 1 章）→ 散落 `intro_slam` / 各章动机 / notation

| 维度 | 源书内容 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 估计史（Gauss 最小二乘 1801、Kalman 1960 可观测性+KF、Apollo）；内/外感受传感器二分；估计问题定义；三书关系 | ✅ 估计史与传感器二分散入 `intro_slam` 与各章动机；KF/最小二乘谱系在 `slam_state_estimation`/`kalman_eskf` 开篇。属“引论性”内容，按需融入而非整段复刻——合理。 |
| (b) 讲解 | 从“航海定位”切入，把状态估计与控制并置、强调其被低估 | ✅ 教材以 SLAM 动机切入（本书定位不同），等价铺垫。 |
| (c) 分析 | 内/外感受互补（IMU+GPS、星敏+陀螺）为何要融合 | ✅ 在 `imu_model`/`vio` 融合动机处落地。 |
| (d) 脉络 | Part I 估计机器 / Part II 三维机器 / Part III 应用 的全书结构 | ✅ 教材自有五部结构，不照搬 Barfoot 三部——独立著作应然。 |
| (e) 思想 | “估计 = 用现有传感器做到最好 + 始终量化不确定度” | ✅ 贯穿 P1 主线（“不仅给点估计，还报协方差”）。 |

**判定 ✅**。引论按“独立著作”口吻重构，非缺。

---

## ch1 概率基元（Barfoot 2.x）→ `slam_state_estimation.tex §2`（`sec:est-gauss`）

| 维度 | 源书内容 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | PDF/CDF/边缘化/贝叶斯/期望矩；独立↔不相关；Shannon/互信息/KL；CRLB/Fisher；高斯联合/条件化(Schur)/边缘/信息形式；线性变换；过非线性(确定性 y=exp(x) 精确换元 + 线性化)；归一化乘积；卡方/马氏；高斯熵/互信息;SMW(四式);Stein(一/二阶);Isserlis(四阶 3σ⁴、`E[xxᵀxxᵀ]=Σ(tr Σ·I+2Σ)` 等);采样(逆变换 + μ+Vx_std);**Q-Q 图**;高斯过程(白噪声、GP回归伏笔) | ✅ 全量落地：`thm:schur-cond`/`prop:info-marg-cond`/`thm:gauss-indep`/`prop:lin-change`/`prop:gauss-nonlinear`/`ex:gauss-through-exp`(对数正态:众数 e^{−σ²}/中位 1/均值 e^{σ²/2} 互不重合)/`prop:info-add`/`prop:chi2`/`eq:isserlis`/`eq:stein`/`thm:crlb`/`eq:smw`/`eq:gauss-sample`/`sec:est-steam`(GP)。Schur、SMW、卡方均值迹技巧均自带证明。**Q-Q 图 `slam_state_estimation:504` 已补**（怎么画 + 偏离对角线的形状诊断 + 图，仿 Barfoot 重绘）——旧基线 partial_thin 已闭合。 |
| (b) 讲解 | 先一般 PDF 后专攻高斯；Schur 补“掰开联合密度”的换元路径 | ✅ §2 开篇明言“摘自 Barfoot 第 1 章…完整复现并合入十四讲直觉”，逐式复现 Schur 条件化。 |
| (c) 分析 | 确定性非线性(y=exp x)→随机非线性(线性化) 的过渡：高斯被掰歪是 EKF 偏斜之源 | ✅ `ex:gauss-through-exp` 完整给出对数正态精确 PDF + “必带雅可比/非高斯不可避免/众数偏均值”三点直觉，明确作立体相机偏斜的概念前置——旧基线“概念桥例缺失”已闭合。 |
| (d) 脉络 | 概率工具 → 为线性高斯估计铺地基 | ✅ §2 收尾导向 §4 正规方程，承接清晰。 |
| (e) 思想 | 信息形式能表“一无所知”(Σ⁻¹→0)；过非线性的“信息创造/毁灭” | ✅ `tab:cov-info`、信息形式洞见落地。 |

**判定 ✅**（近全量，自包含）。

---

## ch2 — 线性高斯估计（Barfoot 3.x）→ `slam_state_estimation.tex §4–7` + `kalman_eskf.tex`

| 维度 | 源书内容 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 批量 MAP=加权最小二乘(正规方程 `HᵀW⁻¹H x̂=HᵀW⁻¹z`);贝叶斯=MAP=信息三路等价(lifted+Schur+SMW);存在唯一=可观测性(观测矩阵+Cayley-Hamilton+质点弹簧);MAP 协方差=逆 Hessian=Fisher=CRLB;块三对角稀疏(马尔可夫);**Cholesky 平滑器逐式块递归 + 从 L 回收边际协方差**;RTS 平滑(前向KF+后向);KF 三推导(MAP/Bayes/增益优化MMSE);误差动力学(无偏/一致归纳证);**DARE + 可镇定/可检测四条件**;**Kalman–Bucy**;连续时间 GP 回归(WNOA Φ/Q、O(1)插值) | ✅ 全量、质量最高：`thm:normal-eq`/`thm:map-bayes`(三路等价含完整证明)/`eq:observability`/MAP协方差/`eq:tridiag`/`alg:est-cholesky-smoother`(块前向+后向逐式)/`alg:est-cov-recovery`(Takahashi 回收，旧基线 partial_thin 已闭合)/`eq:rts`+`eq:rts-cov`/`thm:kf`(MAP配方+高斯条件化两推导)+增益优化MMSE/BLUE/误差动力学/`eq:dare`(四条件，旧基线 partial_thin 已闭合)/`eq:kb-*`(Kalman–Bucy，旧基线 confirmed_missing 已补)/`sec:est-steam`(STEAM 整节)。 |
| (b) 讲解 | 先 MAP(易讲)后 Bayes;批量(易懂)→递归(高效);KF=RTS 前向 pass | ✅ §4 先 MAP、§5 三路等价、§6 批量↔递归。明言“批量更易讲清、理解批量也更易理解递归”，兑现伏笔。 |
| (c) 分析 | 为何 A⁻¹ 只两条对角(马尔可夫)→块三对角→O(K);为何滤波稠密化而平滑保稀疏(边缘化=Schur 引新边) | ✅ `insight[稀疏的物理根源:马尔可夫性]`、`insight[滤波=边缘化历史]`、`pitfall[以为滤波比平滑省所以更优]` 把分析链讲透——这是教材最强处。 |
| (d) 脉络 | 批量 MAP 为统一起点，KF/RTS/信息形式都是其稀疏递归特例 | ✅ `thm:kf-from-map` 缝合点 + `tab:est-batch-vs-recursive` 对照表。 |
| (e) 思想 | “Cholesky=矩阵分解语言、RTS=增益递推语言，同一稀疏批量解的两种写法”;BLUE(误差不必高斯) | ✅ `insight[前向pass=KF、前向+后向=RTS]` 一字传达此思想；增益优化视角点 BLUE/高斯-马尔可夫。 |

**判定 ✅**（全量吸收，五维俱到）。

---

## ch3 — 非线性非高斯（Barfoot 4.x）→ `kalman_eskf.tex`（滤波族）+ `nonlinear_optimization.tex`（批量）

| 维度 | 源书内容 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 立体相机偏斜引例(后验非高斯、MAP 有偏 −33cm/4.41m²、Fig4.4直方图);贝叶斯滤波(四步、精确不可算双障碍);EKF(线性化预测-校正);广义高斯滤波GGF;IEKF→众数;**IEKF=MAP 证明**;过非线性三法(MC/线性化/sigma点) + f(x)=x² 三法对比(κ=2 峰度匹配);**SPKF/UKF 预测+校正逐式 + Σyy↔EKF 等价**;**ISPKF 逐式**(趋均值,数值 24.7414);**粒子滤波 SIR + N_eff + proposal + Madow**;滤波器谱系表;批量 GN(Newton→GN→LM/线搜/Dogleg、Laplace协方差);**ML 偏差 Box1971 闭式**;**SWF 窗扩/窗缩边缘化逐式** | ✅ 主干全量 + 二级方法逐式齐备：`sec:ekf`(立体相机引例,数值齐)/`thm:bayes-filter`/`prop:ekf`/`prop:ggf`/`tab:filter-taxo`/`ex:three-methods`(κ=2)/`algo:spkf-predict`+`algo:spkf-correct`+`prop:spkf-ekf`/`algo:ispkf`/`algo:pf`+`eq:pf-neff`/`thm:nlopt-iekf-map`(自证,旧 external_punt 已消)/`eq:nlopt-mlbias`(Box1971,推导骨架+附录全步)/`der:nlopt-swf`+`der:nlopt-swf-recursion`(逐式,旧 partial_thin 已闭合)。**旧基线“UKF/PF/ISPKF/SWF 缺算法盒”全部补齐**。 |
| (b) 讲解 | 从 Bayes 滤波“精确但不可算”切入,逐个展示各滤波器是其哪种近似;历史(Schmidt-Kalman→EKF→RTS) | ✅ `thm:bayes-filter` 四步证明 + pitfall“两障碍→两近似轴”+ EKF 历史融入。 |
| (c) 分析 | Bayes 滤波第(3)步马尔可夫化简“微妙、是递归诸多局限之源”;线性化为何过自信(漏 σ² 与 2σ⁴);为何 SPKF 轻非线性退回 EKF | ✅ 证明里明标“此步微妙,是递归诸多局限之源”;`ex:three-methods` 逐项对比“线性化均值有偏、方差偏小(过自信)”;`prop:spkf-ekf` 块对角→交叉项零→退 EKF 附证。分析链完整。 |
| (d) 脉络 | **从贝叶斯滤波统一推出各估计器**(谱系表顶=Bayes 滤波);“批量优化是比 Bayes 滤波更优的起点”(§4.3 论点) | ✅ Barfoot 主线被精确传达：`tab:filter-taxo`(顶为不可实现的贝叶斯滤波) + 三处“批量优化是更优起点/马尔可夫内建后无法去除”(`kalman_eskf:224,500,697`)。`thm:nlopt-iekf-map` 后 `insight` 把“IEKF=单步MAP的GN=GVI均值退化”“滤波/优化/变分是同一座山三条路”缝起。 |
| (e) 思想 | 非线性分水岭:均值≠众数→找众数(MAP/IEKF)vs 找均值(MC/sigma点/ISPKF);EKF≈优化一次迭代;IEKF=单时刻GN | ✅ `insight[本质洞察]`(两条岔路)、`pitfall[EKF三宗罪]`(“EKF 大致只是优化里一次迭代”)、IEKF=单步GN 的 `thm` 自证。数值证据(24.5694/24.7770/24.7414)坐实“IEKF→众数、ISPKF→均值”。 |

**判定 ✅**（主干全量 + 二级逐式齐 + 脉络/思想到位）。

---

## ch4 — 非理想处理（Barfoot 5.x）→ `kalman_eskf.tex §consistency,§filter-vs-opt` + `nonlinear_optimization.tex §robust,§cov`

| 维度 | 源书内容 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 估计器性能(误差图±3σ、遍历假设);**NEES/NIS**(含运动 NIS、卡方检验);**偏置使 KF 过自信且无界增长**(误差动力学全证);偏置可观测性(状态增广+可观/不可观例);数据关联(外部/内部/DARCES/铁律);**RANSAC**(五步+`k=ln(1−p)/ln(1−wⁿ)`);**M-估计/IRLS**(L2/Huber/Cauchy/GM + 影响函数);协方差估计三法(监督样本协方差含Bessel/自适应拖尾窗/MAP);**鲁棒=逆Wishart先验MAP**(消 Mᵢ→加权Cauchy);GNC/Barron2019/Yang2020 | ✅ 高度吸收:`def:nees-nis`/`eq:nlopt-nees`+`eq:nlopt-nis`/`thm:bias-overconfident`(全证)/状态增广可观测性/`kalman_eskf:1044`(数据关联+DARCES+铁律)/`eq:nlopt-ransac`/`eq:robust-kernels`+`eq:irls`/监督+自适应协方差(`sec:eskf-appendix`)/`thm:robust-iw`(两文件,完整证)/GNC。**旧基线缺的 Tukey 核已补**(`tab:robust` 行1031 + redescending/c=4.685 效率讨论)。 |
| (b) 讲解 | 先“想要什么性质”(无偏/一致)再讲诊断;偏置从 KF 错在哪讲起 | ✅ `sec:consistency` 先无偏/一致定义再 NEES/NIS;`thm:bias-overconfident` 从误差动力学逐步推。 |
| (c) 分析 | 偏置使 KF 过自信“不论偏置符号、随 k 无界增长”;IRLS 为何收敛到同一极小(梯度同);自适应协方差与 NIS 的联系 | ✅ `thm:bias-overconfident` 证“不论符号、无界增长”;IRLS 梯度等价 + “自适应协方差=让滤波 NIS 一致”均落地。 |
| (d) 脉络 | 偏置/关联/外点/协方差四类非理想统一在“能否折进估计=可观测性” | ✅ 状态增广可观测性把偏置可估性串起;鲁棒核在 `kalman_eskf`(要点)与 `nonlinear_optimization`(完整)显式互链(“要点版+完整版”,字母已统一)——**旧基线“两处割裂”已修**。 |
| (e) 思想 | **鲁棒核不是临时补丁,是“噪声协方差本身不确定”的 MAP 解;所有鲁棒代价=对协方差某先验的特定选择**(Black-Rangarajan) | ✅ `kalman_eskf:1084` `insight[本质洞察]` 一字传达;`thm:robust-iw` 证等价加权 Cauchy + Black-Rangarajan 转述(明标“推测/对偶实例”,合法)。 |

**判定 ✅**（高度吸收,五维俱到）。

---

## ch5 — 变分推断（Barfoot 6.x）→ `nonlinear_optimization.tex §高斯变分推断`（`sec:nlopt-gvi`，行 1223–1541 + `der:nlopt-gvi-deriv`）

> **这是与旧基线最大的出入**：旧基线判 “confirmed_missing（整章未吸收、连定位声明都没有）”，**实际已整章完整吸收**。

| 维度 | 源书内容 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | KL(q‖p) 损失;负 ELBO `V(q)=E_q[φ]+½ln det Σ⁻¹`;对 μ/Σ⁻¹ 导数 + Hessian-梯度关系;牛顿式更新 + 局部收敛(`tr(ABAB)≥0`);NGD(块对角 Fisher diag(Σ⁻¹,½Σ⊗Σ));Stein 双向;**ESGVI 精确稀疏**(因式分解→边缘期望、Σ⁻¹稀疏);**Takahashi 协方差回收**(LDLᵀ + 盒子四角规则 + 后向递归);cubature/sigma点无导数;ESGVI-GN(Jensen 保守、统计雅可比);**EM 参数估计/系统辨识**(E步=ESGVI、M步闭式 W/A/B/C/Q/R);线性恢复批量解;**GVI 滤波校正步**;**立体相机 GVI vs MAP vs ISPKF**(GVI 0.28cm) | ✅ 全量:`eq:gvi-V`/`eq:gvi-derivs`/`eq:gvi-hessgrad`/`eq:gvi-cov-update`+`eq:gvi-mean-update`/`thm:nlopt-gvi-conv`(完整证)/`eq:gvi-ngd-components`/`eq:gvi-stein`+`eq:gvi-stein2`/`eq:gvi-esgvi-hess`/`eq:gvi-takahashi`+`eq:gvi-takahashi-rec`/`eq:gvi-cubature`/`eq:gvi-gn`/`eq:gvi-em-W`/`eq:gvi-linear`/`eq:gvi-filter`/`tab:nlopt-gvi-stereo`(0.28/−33.0/−3.84cm,基准 24.7770)。推导附录 `der:nlopt-gvi-deriv`(四式逐步,非stub)。**旧基线列的 GVI/ESGVI/自然梯度/参数估计/滤波校正/立体相机 全部落地**。 |
| (b) 讲解 | 自顶向下视角对照前面自底向上;为何选 KL(q‖p);历史(VI/Opper-Archambeau/ESGVI) | ✅ `note[本节定位:进阶/选读/前沿]` 明确定位(**旧基线“连定位声明都没有”已修**) + `动机[点估计丢掉后验形状]` + `反面[为何不干脆用更多 sigma点/粒子]` + `历史` 段。 |
| (c) 分析 | KL(q‖p) vs KL(p‖q):期望对谁取(q 可算/p 不可算)是全部理由;mode-seeking vs mass-covering 几何后果;先验 vs 后验统计线性化 | ✅ `sec:nlopt-gvi-elbo` 把“期望对 q（已知可采样）”讲成选 KL(q‖p) 的全部理由 + 脚注 zero-forcing/mass-covering;`insight[先验 vs 后验统计线性化]` 是 GVI≠ISPKF 的本质分析。 |
| (d) 脉络 | **VI 是统一框架,把前面各章(MAP/Laplace/KF/ISPKF/协方差估计/系统辨识)统一解释**;ESGVI 接回因子图稀疏后端 | ✅ Barfoot 顶层统一视角被精确传达:`insight[GVI⇒MAP+Laplace 是只用均值算期望的退化]`、`insight[ESGVI=GVI 接回 BA 稀疏后端]`、`insight[IEKF=单步MAP的GN=GVI均值退化]`(“滤波/优化/变分是同一座山三条路”)。与 `sec:nlopt-cov`(Laplace) 和 `sec:est-fg`(因子图) 显式接缝。 |
| (e) 思想 | GVI 比 MAP“少一层近似”(用整个高斯期望取代峰值求导,故看见展宽/偏移);GVI 几乎无偏(0.28cm)是“mode≠mean 有偏”pitfall 的彻底解药;鲁棒/协方差自适应/系统辨识是同一台机器的不同旋钮 | ✅ `insight`、`pitfall[GVI 只是迭代更多次的UKF/带sigma点的MAP]`(三者貌合神离)、立体相机表 caption“GVI 几乎无偏因显式最小化与真后验 KL + 按后验线性化”、`sec:nlopt-gvi-param` 脚注“鲁棒化/协方差自适应/系统辨识在 GVI 里是同一台机器不同旋钮”——五维全到位。 |

**判定 ✅**（整章完整吸收，五维俱到，是本批吸收质量最高的“补课”之一）。

---

## 独立性扫描（全树 P1 三章）—— ✅ 全部合格

- `external_punt`（证明从略 / 详见原书 / 参见 Barfoot 推导 / 留作不证）：**0 命中**。旧基线点名的 IEKF→MAP punt 已由 `thm:nlopt-iekf-map` 自证消除；ML 偏差由 `eq:nlopt-mlbias` + 附录全步自证。
- `ventriloquize`（“Barfoot 得 / 证明 / 坦言 / 承认 / 指出 / 写道”作论据）：**0 命中**。旧基线点名的“Barfoot 得系统偏差”“Barfoot §4.2.6 证明”句已不存在。
- `narration_dependence`（“在 Barfoot 中… / 书中第 X 节…”作叙述骨架）：**0 命中**。出现的 `\cite{barfoot2024state}` 均为合法出处标注与平衡对照（如“信息阵 Barfoot 记 I、本书记 Λ”“Barfoot 用左扰动、本书改右扰动”）。
- 记号一致性：右扰动主线、`Σ_w/Σ_v` 噪声协方差、`Λ` 信息阵、check/hat、把 `R` 留给旋转——抽查处贯穿一致，并在 `kalman_eskf §记号` 给跨书雷区对照（Barfoot 的 v=输入/n=观测/Q/R/C/A）。

---

## ❌ 高价值缺失 与 ⚠️ 讲解/脉络/思想退化（重点清单）

**❌ 高价值缺失：无。** 旧基线列的全部 confirmed_missing（ch5 整章、Kalman–Bucy）与 partial_thin（UKF/PF/ISPKF/SWF 算法盒、IEKF→MAP 证明、DARE、Q-Q 图、Tukey、Cholesky 逐式 + 协方差回收）经本次独立核对**均已闭合**。

**⚠️ 讲解/脉络/思想退化：无实质退化。** 五章的“让人真正理解的讲法/分析/脉络/思想”均以本书口吻传达到位，多处甚至超出 Barfoot 的展开深度（如把“滤波=边缘化历史→稠密化”讲成独立 insight + pitfall；把“IEKF=MAP=GVI 均值退化”缝成一条线）。

**仅 2 项极轻微残留（不影响理解，可选补）**：

1. **⚠️(极轻 · 例证缺图) Barfoot Fig 4.4 的 MAP 偏差直方图未独立重绘。** 教材复用了数值（−33.0cm/4.41m²）并入 `tab:nlopt-gvi-stereo`/`sec:ekf` 正文，但未复现“10⁶ 次试验估计值直方图”那张图。按 A 标准“每例不删减”属例证缺图；对“mode≠mean”的理解\emph{不}构成障碍（已有数值 + Q-Q 图 + 立体相机偏斜叙述）。落点建议：`sec:ekf` 立体相机引例处或 `tab:nlopt-gvi-stereo` 旁补一张直方图示意（可仿 Barfoot 重绘，与 GVI 0.28cm 同图对照更佳）。

2. **⚠️(极轻 · 表述精度,非缺口) InEKF “偏置破坏群仿射” 一句。** 这属 Barfoot 第 2 版新增材料/Barrau-Bonnabel 的边界，严格说不在 ch1–5 范围内（ch1–5 不含李群 InEKF）；`paper:inekf` 盒已在 `kalman_eskf`，复核_滤波族 已建议补“把零偏增广进状态会破坏群仿射、实务用 imperfect InEKF”一句。此项与本审 ch1–5 口径关系不大，仅记此以与既有复核报告对齐。

---

## 附：found_elsewhere / 跨章核查速记（不计缺口）

- MC/一/二/四阶/sigma点不确定度传播 → `camera_model.tex`（Barfoot 双目数值实验全量,含 `ex:cam-unc`）。
- 连续时间 GP 回归 / STEAM / WNOA Φ-Q / O(1) 插值 → `slam_state_estimation §sec:est-steam` 整节 + `nonlinear_optimization` 附录。
- 能控/可观 → `control_intro`(可控性/Gramian/PBH/可镇定可检测) + `imu_model`(AINS 不可观零空间) + `slam_state_estimation §observability`，覆盖反超 Barfoot。
- 平方根滤波(SRIF/UD/Bierman) → 优化侧 `nonlinear_optimization §nlopt-solve`(√SAM、κ(A)²) 完整,滤波侧 `kalman_eskf` Joseph-form pitfall 交叉引用——超 Barfoot ch1–5。
- 信息滤波 IF/SEIF、互补滤波(Mahony/Madgwick)、直方图滤波 → 均为对标 Thrun/Särkkä 的\emph{增补},超 Barfoot ch1–5 范围（Barfoot 未单列 IF 递归、无互补/直方图）。

---

*独立复核小结：Barfoot ch1–5 已五维完整吸收、自包含、独立性合格，质量高且多处超原书深度。旧基线 `docs/Barfoot吸收审计.md` 的 gap 清单已被本会话此前的主吸收 + 复核轮基本清零；本次对抗式重核仅发现 1 项例证缺图（Fig 4.4 直方图，极轻）。无需再动正文即达“完整吸收”标准。*
