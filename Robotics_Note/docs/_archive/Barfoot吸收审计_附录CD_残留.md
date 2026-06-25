# Barfoot《SER》2nd ed 吸收审计 —— 扫尾：附录 C / 附录 D + 前几轮残留项

> 只读审计，未改任何 .tex。本轮为「完全吸收」收口：核查从未审计的 **附录 C（杂项额外）**、**附录 D（习题解）**，并逐一 grep 全树复核前几轮（ch10/附录、状态估计完备性、独立性扫描）列出的残留项现状。
> 源：`Barfoot_SER2_md/{C_Appendix_C_Miscellaneous_Extras, D_Appendix_D_Solutions_to_Exercises}`；教材树：`/root/gpf/Robotics_Theory/Robotics_Note/parts/`。
> **重要快照说明**：`matrix_calculus.tex`、`slam_state_estimation.tex`、`nonlinear_optimization.tex`、`kalman_eskf.tex`、`lie_theory.tex` 这几轮正被其他 agent 大量追加内容。本轮读到的是 **2026-06-18 07:18 左右的快照**，且已能看到大量「前几轮报缺的内容现已落地」。凡涉及这些文件的判定，下文均标注实际命中行号为证。

---

## ① 总体结论

**附录 C / D 已无「有教学价值且不可替代、却完全未吸收」的内容单元。前几轮列为 major / partial 的残留项，在本轮快照里绝大多数已被补全或已有等价覆盖。**「完全吸收 Barfoot 2nd ed」在内容层面**实质达成**，只余 2 项「可选/装饰性」缺口与若干「编排/交叉引用」式收尾。

逐源结论：

| 源 | 本轮判定 | 说明 |
|---|---|---|
| **附录 C.1 多元高斯 FIM** | **found_elsewhere（已吸收其核心 + 归宿）** | 基本 CRLB/FIM 在 `slam_state_estimation.tex:561-572`；**vech/复制矩阵 D**、**FIM 重参数化（雅可比夹心）**、ML 估协方差的对称化求导已在 `matrix_calculus.tex:483-512,646-749`；C.1 的真正用途（自然梯度/变分推断）已由 `nonlinear_optimization.tex:1099-1344` 的 GVI/ESGVI 整节兑现（含 Amari 自然梯度、Stein 正/反用）。C.1.1–C.1.6 那套「同一高斯六种参数化、各自 $\mathcal I_\theta$」的**逐一枚举**未照搬——但其方法（链式法则作用于对数似然曲率）已被一般化收录，逐一枚举属可选。 |
| **附录 C.2 Stein 引理** | **found_elsewhere（已吸收）** | `slam_state_estimation.tex:480`（陈述 `eq:stein`）+ 二阶/分块版用于 sigma 点；`nonlinear_optimization.tex:1226-1344` 在 GVI 里正用 + 反用。完整。 |
| **附录 C.3 连续→离散 + Van Loan** | **found_elsewhere（已补全）** | **Van Loan 分块矩阵指数**现为 `matrix_calculus.tex:545-621` 完整一节（`thm:matderiv-vanloan`，含级数证明、$\boldsymbol\Phi$ 与 $\mathbf Q_k$ 一次算齐、$\big[\begin{smallmatrix}\mathbf A&\mathbf L\mathbf Q\mathbf L^\top\\\mathbf 0&-\mathbf A^\top\end{smallmatrix}\big]$ 读出法），并显式声明服务 `ch:eskf` 连续→离散与 `ch:slam_est` STEAM。前一轮的 C-3 缺口**已闭合**。 |
| **附录 C.4 不变 EKF（IEKF/InEKF）** | **found_elsewhere（已补全）** | `kalman_eskf.tex:969-984` 新增 `paper:inekf`（群仿射定义、右/左不变误差、log-linear 自治性、为何治好一致性、与 ESKF/FEJ 的关系、$\mathrm{SE}_2(3)$ 结构要点），`lie_theory.tex:981-1131` 给 $\mathrm{SE}_2(3)$ 权威定义 + 对称/不变/等变群论地基。前一轮「IEKF 一词双义、不变 EKF 全缺」**已彻底解决**（且已区分*迭代* EKF vs *不变* EKF）。 |
| **附录 D 习题解（ch2–8）** | **方法/思想已覆盖** | D 只含 ch2/3/4/5/7/8 题解（无 ch10、无附录题）。逐题抽查其**方法**（迹循环、完成平方、Stein 求协方差、Isserlis 算方差、高斯 KL 闭式、SMW、$(\mathbf{Cu})^\wedge=\mathbf{Cu}^\wedge\mathbf C^\top$、$\odot$ 算子、位姿复合协方差、SO(3) 星跟踪 ESKF）——**无一不在教材正文有对应工具或推导**。无逐题搬运义务，按任务要求只判方法覆盖：**通过**。 |

**一句话**：相较前一轮把 ch10 列为「唯一 major 缺口」，本轮发现 **ch10 已成完整正节** `sec:est-steam`（`slam_state_estimation.tex:1181-1392`，21 个 `eq:steam*` 标签，覆盖局部 GP 缝合→WNOA SE(3) 先验→块三对角稀疏→$O(1)$ 插值=三次 Hermite→外推→协方差恢复→WNOJ/Singer/连续体延伸）；前一轮的 B-2/B-3/B-5/B-6（李群运动学恒等式、Jordan 分解、最小多项式）也已在 `lie_theory.tex` 落地。**附录 C/D 本身没有再贡献任何新的 confirmed_missing。** 剩余只是「装饰性图」与「编排」级别。

---

## ② 逐项 gap 复核（confirmed_missing / partial_thin / found_elsewhere）

### 已闭合（found_elsewhere）——前几轮报缺、本轮快照已补，仅作存证

| 旧编号 | 内容 | Barfoot 出处 | 现落点（行号为证） |
|---|---|---|---|
| A-1～A-6 | SE(3) 连续时间 GP 先验 / WNOA / STEAM / 插值 / 外推 / WNOJ·Singer·连续体 | §11.1–11.4 | `slam_state_estimation.tex:1181-1392`（`sec:est-steam` 整节，`prop:steam-sparse`、`sec:est-steam-query`、`tang2019wnoj`/`wong2020singer`/`lilge2022continuum` 全部引入） |
| B-2/B-3 | $\dot{\mathbf J}-\boldsymbol\omega^\wedge\mathbf J=\partial\boldsymbol\omega/\partial\boldsymbol\phi$ 及 SE(3) 版 | §B.2.1/B.2.2 | `lie_theory.tex:1353-1363`（专门 derivation，含 so(3)+se(3) 两式与积分分部证明） |
| B-5 | SE(3) / Ad 的 Jordan 分解（$\lambda=0$ 代数重数 2、几何重数 1、不可对角化） | §B.3.2–B.3.3 | `lie_theory.tex:1269-1299`（`subsec:minimal-poly` 显式做 Jordan 分析） |
| B-6 | so(3)/se(3)/Ad 最小多项式 | §B.3.x | `lie_theory.tex:1287-1303`（`thm:minimal-poly` 给**三条**最小多项式 + insight「最小多项式是闭式总开关」） |
| C-1 | vech/复制矩阵、FIM 重参数化、自然参数信息形式 | §C.1.3-C.1.6 | `matrix_calculus.tex:487-512`（`def:matderiv-vech` 复制矩阵 D）、`:648-749`（`thm:matderiv-fim-reparam` 重参数化）；归宿 GVI 见 `nonlinear_optimization.tex:1099+` |
| C-2 | Stein 引理 | §C.2 | `slam_state_estimation.tex:480` + `nonlinear_optimization.tex:1226+` |
| C-3 | Van Loan 一次算齐 $\boldsymbol\Phi,\mathbf Q_k$ | §C.3 | `matrix_calculus.tex:545-621`（`thm:matderiv-vanloan`） |
| C-4 | 不变 EKF（Barrau–Bonnabel） | §C.4 | `kalman_eskf.tex:969-984`（`paper:inekf`）+ `lie_theory.tex:1050-1131` |
| —（状态估计完备性轮） | 立体相机 MAP/ML 有偏的闭式 | §4 正文/习题 | `kalman_eskf.tex:1328-1335`（`ML 偏差近似闭式` derivation，显式说「量化立体相机例 MAP/ML 有偏」）；GVI 把偏差从 −33cm 压到 0.28cm，`nonlinear_optimization.tex:211,1099+` |
| —（编排残留） | MAP 协方差「第三法=鲁棒核」回链 | §5 | **已回链**：`kalman_eskf.tex:769` 明写「自适应协方差估计与 `thm:robust-iw` 的 MAP 协方差估计是同一思想的两个落点」；`thm:robust-iw` 在 `:1064`。原报「编排问题」已不成立。 |

### partial_thin（已触及，可锦上添花，非硬缺口）

| # | 内容 | Barfoot 出处 | 现状 / 落点 | 建议 |
|---|---|---|---|---|
| P-1 | **C.1 的「同一高斯六种参数化各自 FIM 表」**（canonical / 对称化 / hybrid / natural，及各自 $\mathcal I^{-1}$ 的 $\tfrac12\boldsymbol\Sigma^{-1}\otimes\boldsymbol\Sigma^{-1}$、$\mathbf D^\top(\cdot)\mathbf D$ 等显式块） | §C.1.1-C.1.6 | 教材有**通用**重参数化定理（`thm:matderiv-fim-reparam`）+ vech/复制矩阵 + 自然梯度归宿，但**没有把这四套参数化的 $\mathcal I_\theta$ 当作一张对照表逐一写出**（尤其 natural 参数下 FIM 非块对角这一「unnatural」观察） | 低优先。若要「附录全量对照」，可在 `matrix_calculus.tex` FIM 小节加一个**选读表**列四参数化 + 一句「natural 参数 FIM 非块对角」。否则现有一般化 + GVI 已够，**不补亦达标**。 |
| P-2 | **附录 C.4 把 EKF 代数变换成不变 EKF 的逐步推导**（$\mathbf F'_{k-1}=\mathbf 1$、$\mathbf G'_k=\mathbf p^\odot$、左不变创新 $\check{\mathbf T}_k^{-1}(\mathbf y_k-\check{\mathbf y}_k)$、那一处协方差近似） | §C.4.1-C.4.4 | 教材 `paper:inekf` 给的是**概念+性质+去向**（群仿射⇒自治、为何改善一致性），**未复现** Barfoot 那套「把 (9.141) 一步步搬成不变形式」的代数细节，也未写出左不变创新的 $\mathbf G'=\mathbf p^\odot$ 闭式 | 低–中优先。教材右扰动主线下，左不变结构本属专题；`paper:inekf` 已达「读者懂其所以然」的标准 A。若追求与附录 C.4 等深，可在该 paper 框后加一个选读 derivation 写出 $\mathbf F'=\mathbf 1$、$\mathbf G'=\mathbf p^\odot$ 的代数。**非必需。** |

### confirmed_missing（全树 grep 零命中，且属可补内容）

| # | 内容 | Barfoot 出处 | 价值 | 应落何处 |
|---|---|---|---|---|
| **CM-1** | **立体相机 MAP 偏差直方图（数值实验图）** | Fig 4.4 | **低（装饰性）**。教材**复用了该实验的数值**（−33 cm MAP 偏差、GVI 0.28 cm，见 `nonlinear_optimization.tex:211`）并以文字 + `insight` 讲透「均值≠众数、线性化丢二阶曲率」的**机理**（`slam_state_estimation.tex:393`），还给了 ML 偏差闭式量化之。**唯独没有复现那张「N 次蒙特卡洛偏差直方图」**。机理与结论既已自包含，缺图不影响「内容完全吸收」，只影响「插图丰富度」。 | 若要补：`visual_odometry.tex` 或 `nonlinear_optimization.tex` GVI 节加一张直方图（横轴深度误差、纵轴频次，叠 MAP vs GVI 两簇），用 `绘图手册` 的 pgfplots 模板。**纯可选。** |

> 说明：任务点名的「Jordan 广义特征向量全套构造（Barfoot App B.30–B.76）」——**判定不值得照搬**。教材 `thm:minimal-poly` 走的是「Cayley–Hamilton + Jordan 块结构分析读出最小多项式」这条更经济的路（已含「$\lambda=0$ 代数重数 2/几何重数 1、需 Jordan 分解、$2\times2$ 零块平方为零」等关键代数事实），**正是 B.30–B.76 里有教学价值的那部分**；而 B.30–B.76 把每个广义特征向量逐一解出的冗长构造，对「理解为何 SE(3) 闭式有限项」并无额外不可替代价值。**故此项判为 found_elsewhere（核心思想已吸收），不列为缺口。**

---

## ③ 优先级清单（本轮收口建议）

按「对『完全吸收』的边际贡献 × 工作量」排序。注意：**前三优先级（ch10、李群恒等式/最小多项式、Van Loan、InEKF）在本轮快照里已基本完成，故下列只剩低优先收尾项。**

1. **【低 · 极小工程 · 编排】无须再补「MAP 协方差第三法回链」**——已确认 `kalman_eskf.tex:769` 完成回链，`thm:robust-iw` 与自适应/监督协方差估计已互指。**本项可从待办移除。** 仅建议复核一遍措辞是否够清晰即可。

2. **【低 · 极小工程 · 可选】FIM 四参数化对照表（P-1）**。在 `matrix_calculus.tex` 的 FIM 小节加一个**选读表**：列 canonical / 对称化（vech+D）/ hybrid（逆协方差）/ natural（信息形式）四参数化各自的 $\mathcal I_\theta$、$\mathcal I_\theta^{-1}$，并一句点明「natural 参数 FIM 非块对角，故名实不符」。完成「附录 C.1 全量对照」。**不补也达标。**

3. **【低–中 · 小工程 · 可选】不变 EKF 代数细节 derivation（P-2）**。在 `kalman_eskf.tex:984` 的 `paper:inekf` 之后加一个**选读 derivation**，把 Barfoot §C.4 的「(9.141) → 不变形式」代数走一遍：$\mathbf F'_{k-1}=\mathbf 1$、左不变创新 $\check{\mathbf T}_k^{-1}(\mathbf y_k-\check{\mathbf y}_k)$、$\mathbf G'_k=\mathbf p^\odot$、那处 $\check{\mathcal T}_k^{-1}\hat{\mathbf P}_k\check{\mathcal T}_k^{-\top}\approx\hat{\mathbf P}'_k$ 近似。让 InEKF 从「知其然」升到「与附录 C.4 等深」。

4. **【低 · 小工程 · 装饰】立体相机偏差直方图（CM-1）**。可选补一张蒙特卡洛偏差直方图（MAP vs GVI 两簇），复现 Barfoot Fig 4.4 的视觉。数值教材已有，纯属插图增色。

5. **【低 · 微 · 独立性收尾】连续时间/InEKF 段落的单书引用平衡**。前一轮独立性扫描提到「连续时间相关内容偏单书（Barfoot/Handbook）引用」。本轮看 `sec:est-steam` 与 `paper:inekf` 已大量引一手论文（`anderson2015steam`、`barfoot2014steam`、`tang2019wnoj`、`wong2020singer`、`barrau2017invariant`、`mahony2021equivariant` 等），单书依赖已显著降低。**仅建议**：在 STEAM 节末或 InEKF 处补一处与十四讲/Handbook 的**视角对照句**（如「Handbook 从连续时间 SLAM 综述角度…」），即彻底消除残留的 narration_dependence 顾虑。

---

### 附：本轮判定证据索引（便于复核）
- **附录 C.1/C.2**：`slam_state_estimation.tex:439-447`(KL/熵)、`:480`(Stein)、`:561-572`(CRLB/FIM)；`matrix_calculus.tex:483-512`(vech/复制矩阵/对称化求导)、`:646-749`(FIM 重参数化)；`nonlinear_optimization.tex:1099-1344`(GVI/ESGVI/自然梯度/Stein 正反用)。
- **附录 C.3 Van Loan**：`matrix_calculus.tex:545-621`（`thm:matderiv-vanloan`，`sec:matderiv-vanloan`）。
- **附录 C.4 不变 EKF**：`kalman_eskf.tex:969-984`(`paper:inekf`)、`:1140,1202,1236,1251-1252`(谱系/术语/引用/去向)；`lie_theory.tex:981-1054`($\mathrm{SE}_2(3)$+群仿射)、`:1066-1131`(对称/不变/等变)。
- **ch10 STEAM**（前一轮 major，今已成节）：`slam_state_estimation.tex:1181-1392`，21 个 `eq:steam*` 标签；`prop:steam-sparse:1312`、`sec:est-steam-query:1330`、WNOJ/Singer/连续体 `:1384`。
- **李群残留**：`lie_theory.tex:365`(`thm:so3-kinematics`)、`:409`(`thm:se3-kinematics`)、`:1353`(运动学雅可比恒等式 derivation)、`:1287`(`thm:minimal-poly`)、`:1269`(`subsec:minimal-poly` Jordan)。
- **立体相机偏差**：机理 `slam_state_estimation.tex:376,393`；数值 −33cm/0.28cm `nonlinear_optimization.tex:211`；ML 偏差闭式 `kalman_eskf.tex:1328`。直方图图：**全树 grep 零命中**（仅 loop_closure/camera_calibration 的「直方图」是 BoW/角点误差，无关）。
- **附录 D**：仅 ch2/3/4/5/7/8 题解（`D_…:5,147,249,333,361,451`），无 ch10/附录题；逐题方法均在正文有对应工具。
- **全树零命中确认**：`Van Loan` 仅 `matrix_calculus.tex` + 一处记号说明；`WNOJ/Singer/连续体` 仅 `slam_state_estimation.tex:1384`；FIM 四参数化「natural 非块对角」表述 0 处；立体偏差直方图 0 处。
