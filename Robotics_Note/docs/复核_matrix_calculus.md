# 独立复核报告：`parts/appendix/matrix_calculus.tex`（矩阵求导附录）

**总评：MINOR** — 公式与交叉引用全部正确、结构完整自洽、可作前置地基；唯一实质问题是"本书全程取分子布局"这句**口号与本章实际做法略有出入**（标量梯度实际取列向量 = 分母布局，向量雅可比才取分子布局，属业界通行的"混合约定"），建议把措辞精确化即可，不影响任何公式正确性。

> 复核员声明：fresh 独立复核，只读未改。已逐条核对全部 7 条求导公式与布局定义，对照 Matrix Cookbook (Petersen & Pedersen) 与 Wikipedia *Matrix calculus*；已验证全部 `\cref` 目标、`\cite` 键、`\label` 唯一性、表格列数、环境定义、`\input` 挂载。

---

## 维度一：完整性

**结论：PASS（覆盖齐全，作为"从零矩阵求导"地基充分）。**

逐项核对地基应覆盖的要素：

| 要素 | 位置 | 状态 |
|---|---|---|
| 标量对向量（梯度） | §matderiv-scalar / `def:matderiv-grad`（行 49） | ✅ 列/行两种写法都给 |
| 向量对向量（雅可比定义） | §matderiv-jacobian / `def:matderiv-jacobian`（行 85） | ✅ 含逐元 $(k,l)$ 归属 |
| 布局约定（分子 vs 分母） | §matderiv-layout / `def:matderiv-layout`（行 123）+ 对照表（行 137）+ 一图（行 433） | ✅ 定义、互为转置、表、图齐全 |
| 常用公式 | §matderiv-formulas（行 176）+ 速查表（行 309） | ✅ 线性型/内积/平方/二次型/复合，每条附证明 |
| 链式法则 | §matderiv-chain / `thm:matderiv-chain`（行 275）+ `cor:matderiv-Au` | ✅ 含证明与乘法方向说明 |
| 贯穿例（最小二乘正规方程） | §matderiv-normal / `derivation`（行 333） | ✅ 三步完整推导到 $\mathbf{A}^\top\mathbf{A}\mathbf{x}=\mathbf{A}^\top\mathbf{b}$，并扩到加权版 |
| 选读延伸（标量/矩阵对矩阵、迹技巧、vec/⊗） | §matderiv-matrix（行 381） | ✅ 标 $*$ 选读，最小够用 |
| 练习 | `practice`（行 472） | ✅ 5 题覆盖定义/公式/链式/贯穿例/迹 |

- `[info]` 行 12–14、§matderiv-motiv — 动机层（雅可比无处不在、不统一约定的转置 bug）写得到位，符合 v5.0「先动机后理论」。
- **超出十四讲原附录的增量均为正向**：原书附录仅 B.1–B.6（标量对向量、向量对向量、转置规则、简记），**不含**二次型求导、链式法则、正规方程贯穿例、迹技巧；本附录把这些补全，正是"全吸收 + 自包含"所要求的，且与正文各章（nlopt/lie）真正接得上。完整性无缺口。

---

## 维度二：正确性（对抗式逐条核验）

**结论：PASS（7 条公式全部正确；布局定义与互为转置关系正确；与 lie_theory 声明一致）。** 仅布局口号措辞有一处 MINOR（见维度三/下文"统一约定"条）。

### 逐条核对（对照 Matrix Cookbook / Wikipedia *Matrix calculus*）

| # | 公式 | 位置 | 核验 |
|---|---|---|---|
| 1 | $\partial(\mathbf{Ax})/\partial\mathbf{x}^\top=\mathbf{A}$ | `prop:matderiv-Ax` eq:matderiv-Ax（行 187） | ✅ 正确。证明按分量 $(\mathbf{Ax})_k=\sum_l A_{kl}x_l$、$\partial/\partial x_l=A_{kl}$，与 def 排布一致。Matrix Cookbook 分子布局同此。 |
| 2 | 简记 $\partial(\mathbf{Ax})/\partial\mathbf{x}=\mathbf{A}$ | eq:matderiv-Ax-short（行 199） | ✅ 与十四讲式 (B.6) 字面一致，明确标注"同义、仅省转置号"。 |
| 3 | 转置规则 $\partial\mathbf{F}^\top/\partial\mathbf{x}=(\partial\mathbf{F}/\partial\mathbf{x}^\top)^\top$ | `prop:matderiv-transpose` eq:matderiv-transpose（行 208） | ✅ 正确，与十四讲 (B.5) 一致；推论 $\partial(\mathbf{x}^\top\mathbf{A}^\top)/\partial\mathbf{x}=\mathbf{A}^\top$ 正确。 |
| 4 | 内积 $\partial(\mathbf{x}^\top\mathbf{a})/\partial\mathbf{x}=\mathbf{a}$（列）、$/\partial\mathbf{x}^\top=\mathbf{a}^\top$（行） | `prop:matderiv-xta` eq:matderiv-xta（行 226） | ✅ 正确。 |
| 5 | $\partial(\mathbf{x}^\top\mathbf{x})/\partial\mathbf{x}=2\mathbf{x}$ | `prop:matderiv-xtx` eq:matderiv-xtx（行 239） | ✅ 正确（列向量形）。Matrix Cookbook 分母布局 (eq.~阵列) 同为 $2\mathbf{x}$。 |
| 6 | $\partial(\mathbf{x}^\top\mathbf{A}\mathbf{x})/\partial\mathbf{x}=(\mathbf{A}+\mathbf{A}^\top)\mathbf{x}$；对称时 $2\mathbf{A}\mathbf{x}$ | `prop:matderiv-xtAx` eq:matderiv-xtAx（行 250） | ✅ 正确。逐分量证明严谨（$p=i$、$q=i$ 两类合并、$A_{ii}x_i^2$ 计数说明清楚）。Matrix Cookbook 列向量形即 $(\mathbf{A}+\mathbf{A}^\top)\mathbf{x}$。 |
| 7 | 链式（外到内左乘）$\partial\mathbf{F}/\partial\mathbf{x}^\top=(\partial\mathbf{h}/\partial\mathbf{u}^\top)(\partial\mathbf{g}/\partial\mathbf{x}^\top)$ | `thm:matderiv-chain` eq:matderiv-chain（行 278） | ✅ 正确，维度 $n\times p\cdot p\times m=n\times m$ 自洽；Wikipedia 确认分子布局外层在左。推论 `cor:matderiv-Au` 正确。 |

补充选读式核验（§matderiv-matrix）：
- `eq:matderiv-trace-formulas`（行 407）三式 $\partial\operatorname{tr}(\mathbf{AX})/\partial\mathbf{X}=\mathbf{A}^\top$、$\partial\operatorname{tr}(\mathbf{X}^\top\mathbf{AX})/\partial\mathbf{X}=(\mathbf{A}+\mathbf{A}^\top)\mathbf{X}$、$\partial\ln\det\mathbf{X}/\partial\mathbf{X}=\mathbf{X}^{-\top}$ — **均与 Matrix Cookbook 一致**，✅ 正确。识别定理 $\mathrm{d}f=\operatorname{tr}(\mathbf{G}^\top\mathrm{d}\mathbf{X})\Rightarrow\partial f/\partial\mathbf{X}=\mathbf{G}$ 表述正确。
- `eq:matderiv-vec`（行内，行 419）$\operatorname{vec}(\mathbf{AXB})=(\mathbf{B}^\top\otimes\mathbf{A})\operatorname{vec}(\mathbf{X})$ — ✅ 标准恒等式，正确。
- 贯穿例 `derivation`（行 333–374）三步：展开二次型（中间两项互为转置标量合并 $-2\mathbf{b}^\top\mathbf{A}\mathbf{x}$ ✅）、逐项求梯度（二次项用对称 $\mathbf{A}^\top\mathbf{A}$ 得 $2\mathbf{A}^\top\mathbf{A}\mathbf{x}$ ✅、线性项 $-2\mathbf{A}^\top\mathbf{b}$ ✅）、置零得正规方程 ✅。加权版 $\mathbf{A}^\top\boldsymbol{\Sigma}^{-1}\mathbf{A}\mathbf{x}=\mathbf{A}^\top\boldsymbol{\Sigma}^{-1}\mathbf{b}$ ✅。
- 练习 2（行 475）数值例自洽：$\mathbf{A}=\big[\begin{smallmatrix}1&2\\0&3\end{smallmatrix}\big]$，$(\mathbf{A}+\mathbf{A}^\top)\mathbf{x}=\big[\begin{smallmatrix}2&2\\2&6\end{smallmatrix}\big]\mathbf{x}$；对称化 $\tfrac12(\mathbf{A}+\mathbf{A}^\top)=\big[\begin{smallmatrix}1&1\\1&3\end{smallmatrix}\big]$，$2\hat{\mathbf{A}}\mathbf{x}$ 同值 ✅。

### 分子布局 vs 分母布局 — 定义与"互为转置"

- `def:matderiv-layout`（行 123–135）：分子布局 $n\times m$、$(k,l)=\partial F_k/\partial x_l$；分母布局 $m\times n$、为分子布局转置。**与 Wikipedia 标准定义完全一致** ✅。互为转置关系 `eq:matderiv-layout-rel` 正确 ✅。
- 对照表（行 137–150）把"标量对向量"的**列向量 $\partial f/\partial\mathbf{x}=\nabla f$ 归入分母布局**、行向量 $\partial f/\partial\mathbf{x}^\top$ 归入分子布局 — **这是标准且正确的归类**（Wikipedia：numerator layout 标量梯度为行向量，denominator layout 为列向量即 $\nabla f$）✅。

### 与 lie_theory「本书取分子布局」声明的一致性

- `parts/P0_math/lie_theory.tex:482` 原文："本书取分子布局 …（行随分子、列随分母，与十四讲一致）"，且据此解释 `eq:left-pert-se3` 的分块排布。
- 本附录 §matderiv-layout 的 `insight`（行 152）正声明自己是"lie_theory 那条声明的依据"，**对向量雅可比的排布二者完全一致** ✅。`insight`（行 422，与李群求导的关系）把 $\partial(\mathbf{Rp})/\partial\boldsymbol{\varphi}=-(\mathbf{Rp})^\wedge$ 归为标准"向量对向量"雅可比，逻辑自洽 ✅。

---

## 维度三：行文 / 脉络

**结论：PASS（循序渐进、自包含、可作前置附录被引用）。** 含 1 条 MINOR 措辞建议。

- `[ok]` 脉络：动机（雅可比无处不在 + 转置 bug 反面）→ 标量对向量 → 向量对向量 → 布局约定（钉死分子布局）→ 公式表（每条附证明）→ 贯穿例（正规方程）→ 选读延伸 → 小结速查 + 练习。严格"先动机后理论""由浅入深"，符合 v5.0 模板。
- `[ok]` 自包含：开篇明示"读这一章不需要任何前置"（行 14），仅用一元偏导 + 矩阵乘法/转置；公式每条带独立可复现证明，无外部 punt（见维度四）。
- `[ok]` 可被引用：`insight`（行 152、422）、`note`（行 68）、小结"后续关系"（行 469）反复把本附录定位为 rigid_body/lie/slam_est/nlopt 的统一地基，并精确指向各章既有标签，**前置附录定位达成**。

- **`[minor]` 行 152「本书全程取分子布局」措辞 — 与本章实际做法及 §6.3 记号约定的细微张力。**
  问题：本章在标量梯度（`def:matderiv-grad`、eq:matderiv-xtx、eq:matderiv-xtAx、正规方程推导、§7 记号表 $\nabla J$/$\mathbf{H}_J$）一律取**列向量** $\partial(\cdot)/\partial\mathbf{x}$。按本章自己的对照表（行 144）与 Wikipedia 标准分类，**列向量梯度属"分母布局"**；只有向量对向量的雅可比 $\partial\mathbf{F}/\partial\mathbf{x}^\top$ 才是分子布局。故本书实际是业界极常见的**混合约定**（标量梯度=分母/列，向量雅可比=分子），而非字面的"全程分子布局"。
  补充佐证：Wikipedia *Matrix calculus* 明确指出"some choose denominator layout for gradients (column vectors) but numerator layout for the vector-by-vector derivative ∂y/∂x"——这正是本书所为，是合法且主流的做法，**公式无一处错**。
  影响：纯措辞/口号层面；本章的表（行 144）、`pitfall` 第 2 条（行 169，已提醒"标量取列与向量简记不是一回事"）实际上已隐含正确区分，仅顶层那句"全程分子布局"读起来与之轻微抵触，严谨读者会察觉。
  建议（择一）：① 把行 152/463 的"全程取分子布局"细化为"**向量对向量的雅可比统一取分子布局；标量梯度按惯例取列向量（$\nabla f$，即分母布局），二者构成主流的混合约定**"；或 ② 保留口号但加一句脚注点明"此处'分子布局'特指向量雅可比的排布；标量梯度仍记为列向量 $\nabla f$"。lie_theory:482 的同句因只谈"分子 $[\mathbf{a};\mathbf{b}]$ 对分母"的**向量**情形，本身无歧义，可不动。

---

## 维度四：独立性 + LaTeX

**结论：PASS（纯 \cite 出处、引用/标签/表格/环境全部一致，无重复 label、无未定义引用风险）。**

- `[ok]` **零外部 punt**：通篇用 `\cite{petersen2012cookbook,magnus2019matrix,gaoxiang2019slam14}` 仅标出处，**无**"详见/见原书/此处略"等把内容推给原书的写法（符合 §1.3 铁律）。三个 `\cite` 键均存在于 `refs.bib`（petersen2012cookbook@1108、magnus2019matrix@1114、gaoxiang2019slam14@18）✅。
- `[ok]` **章 `\cref` 全部命中既有标签**：`ch:rigid_body`、`ch:lie`、`ch:slam_est`、`ch:nlopt`、`ch:camera`、`ch:eskf` 均在 parts/ 下唯一定义 ✅。
- `[ok]` **式/定理 `\cref` 全部命中**：`eq:gn-normal`（nlopt:321）、`eq:nlopt-gls`（nlopt:206）均存在 ✅。本附录**未**误引 `eq:lm`/`eq:schur`/`eq:retraction`/`thm:gn`（这些是 nlopt 内部标签，附录正文未引用，无悬空引用）✅。
- `[ok]` **无重复 label**：本章 51 个 `\label` 各唯一，且与 parts/ 其余文件**无一冲突**（逐一 grep 确认计数=1）。新标签一律 `app:` / `eq:matderiv-*` / `prop:matderiv-*` / `sec:matderiv-*` 前缀，符合开头注释（行 9）的命名纪律 ✅。
- `[ok]` **表格列数一致**：两张 `\begin{tabular}{lll}`（3 列）——对照表（行 137）与速查表（行 309）每行正好 2 个 `&`、`\toprule/\midrule/\bottomrule` 配齐（booktabs），无错列 ✅。两表均 `[H]`（float 包已在 preamble:29 载入）✅。
- `[ok]` **环境定义齐全**：`derivation`（styles:84）、`pitfall`（styles:101）、`practice`（styles:112）、`insight`（styles:96 `\elegantnewtheorem`，可 `\cref` 且接 `[标题]`）、`definition/theorem/proposition/corollary`（EB 主题，可 `\cref`）、`note`（被 kalman_eskf 等十余章正常使用，确为可用环境）全部存在 ✅。
- `[ok]` **宏与颜色**：`\SO`/`\SE`（preamble:37-38）、tikz 用色 `structurecolor`/`second`/`third`（preamble/styles 均有定义）✅。
- `[ok]` **挂载正确**：`book.tex:36` 在 `\appendix` 后 `\input{parts/appendix/matrix_calculus}`，作为附录章正确就位 ✅。
- `[note]` 行 153 称"Ceres、g2o、GTSAM 把残差雅可比按 $\partial\mathbf{e}/\partial\mathbf{x}$、行=残差维、列=状态维组织"——属实，与本书 §7 记号 `\mathbf{J}=\partial\mathbf{e}/\partial\mathbf{x}` 及 `\mathbf{H}=\mathbf{J}^\top\mathbf{W}^{-1}\mathbf{J}` 一致，无误。

> **LaTeX 编译风险评估**：未发现会导致编译失败或未定义引用的问题。所有 `\cref` 目标、`\cite` 键、环境、宏、颜色均已定义；标签无重复；表格结构合法。（本复核只读未编译，但静态核验覆盖了 §11 质量闸的"零未定义引用 / 标签无冲突 / 表格列数 / 环境签名"各项。）

---

## 汇总

- **正确性**：7 条核心公式 + 迹/vec 选读式 + 贯穿例 + 练习数值，**全部正确**，与 Matrix Cookbook、Wikipedia 一致；布局定义与"互为转置"正确；与 lie_theory:482 声明在向量雅可比排布上完全一致。
- **完整性 / 行文 / LaTeX**：均 PASS，无 punt、无重复 label、引用与表格环境全部一致，可作前置地基被各章引用。
- **唯一待办（MINOR，措辞）**：行 152/463「本书全程取分子布局」宜精确化为"向量雅可比取分子布局、标量梯度取列向量（分母布局），二者为主流混合约定"——纯表述清晰度问题，不涉及任何公式错误，不阻塞。

**建议状态：MINOR（可直接合入；建议顺手把布局口号一句话精确化）。**

---
### 参考
- The Matrix Cookbook (Petersen & Pedersen): https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
- Wikipedia, *Matrix calculus*（布局约定、混合约定的合法性）: https://en.wikipedia.org/wiki/Matrix_calculus
