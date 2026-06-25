# 独立复核报告：`parts/P0_math/lie_theory.tex`（李群与李代数）

> **总评：PASS（含 1 处 minor 建议）**——独立、对抗式复核（fresh，未参与之前润色）。
> **对抗式重点 Sim(3) 尺度雅可比 $\mathbf{J}_s$（eq:sim3-Js）经 Sophus `calcW` 逐项符号核对 + 数值核对，完全正确，两支退化分支亦正确。** 抽查的 Rodrigues / 左右雅可比及其逆 / SE(3) 指数 / BCH（含三阶符号与 SE(3) 6×6）/ 伴随 / $\odot$ / 右扰动 / $\mathbf{Q}_l$ 块 / 四阶协方差算子 全部独立验证通过。知识无新漏、本轮所补（Sim(3) $\mathbf{J}_s$、external_punt 清除、左扰动分块）已落实。行文连贯，独立性达标，LaTeX 健康（环境配平、`\cref`/`\label` 全闭合、零悬空引用）。
>
> 复核员：独立 fresh agent；日期 2026-06-18；方法：源对照 + SymPy 符号验证 + NumPy/SciPy 数值验证 + 权威实现（Sophus `sim_details.hpp`）对照 + grep 静态检查。

---

## 维度一：知识完整性 — **结论：PASS（无新漏；本轮所补已全部落实）**

### 十四讲 ch4 价值内容全覆盖（逐节核对）
| 十四讲 ch4 内容 | 本章对应 | 状态 |
|---|---|---|
| 群定义/封结幺逆/SO(3)/SE(3) (§4.1.1) | def:group, eq:so3/se3, prop:matrix-lie | ✅ 增厚（补群公理证明） |
| 从 $\mathbf{R}\mathbf{R}^\top=\mathbf{I}$ 求导引出 $\mathfrak{so}(3)$ (§4.1.2) | sec:lie-algebra（招牌推导全保留） | ✅ |
| 李代数定义/$\mathfrak{so}(3)$/$\mathfrak{se}(3)$ (§4.1.3-5) | def:lie-algebra, def:so3-se3 | ✅ |
| SO(3) 指数=Rodrigues + 级数推导 (§4.2.1) | thm:so3-exp + 完整证明 | ✅ 含奇偶项归并 |
| 对数映射级数 + 迹法 (§4.2.1) | eq:so3-log-series, eq:so3-log | ✅ |
| 满射非单射/周期性 (§4.2.1) | prop:surjective + pitfall | ✅ |
| SE(3) 指数 + 左雅可比 $\mathbf{J}$ + $\mathbf{t}=\mathbf{J}\boldsymbol\rho$ (§4.2.2) | thm:se3-exp + 证明 | ✅ |
| BCH + 线性近似 + $\mathbf{J}_l/\mathbf{J}_l^{-1}/\mathbf{J}_r$ (§4.3.1) | sec:bch, eq:bch, eq:bch-linear, eq:Jl..eq:Jr | ✅ 增厚 |
| 群乘↔代数加两条对照规则 (§4.3.1) | eq:bch-group-to-alg, eq:bch-alg-to-group | ✅ |
| SE(3) BCH (4.36/4.37) | eq:bch-se3-left/right | ✅ |
| 李代数求导（残留 $\mathbf{J}_l$）(§4.3.3) | eq:deriv-model | ✅ |
| 扰动模型 + SE(3) $\odot$ (§4.3.4-5) | sec:perturb, eq:right-pert-se3, eq:left-pert-se3 | ✅ 右扰动为主线 + 左扰动并列 |
| 向量对向量求导排布约定 (4.43) | line 482 note | ✅ |
| Sophus 实践 (§4.4.1) | sec:sophus codebox | ✅ |
| 轨迹误差 ATE/RPE + RMSE=2.207 (§4.4.2) | sec:traj-error, eq:ate-all..rpe-all | ✅ RMSE 2.207 已引 (line 754) |
| **Sim(3) 群/$\mathfrak{sim}(3)$/指数/$\mathbf{J}_s$/扰动 (§4.5)** | sec:sim3, eq:sim3..eq:sim3-pert | ✅ **本轮已补 $\mathbf{J}_s$ 闭式** |
| 习题（伴随性质等）(§4 习题) | 各 practice 盒 | ✅ |

### 本轮应修复项（来自审计「ch04」节）核验
- **`[major]` Sim(3) $\mathbf{J}_s$ 闭式缺失** → ✅ **已补**：derivation 盒（line 627-650）给出 eq:sim3-Js 完整闭式 $C_s,A_s,B_s$ + 两支退化。
- **`[major]` external_punt（旧 :619「闭式见十四讲§4.5」）** → ✅ **已消除**：punt grep 仅命中 line 400，且该句为「本书出于自包含…补全」的正向陈述，非 punt。
- `:400` ventriloquize「十四讲坦言…略去」 → ✅ 已改中立「部分教材（如十四讲）因计算用不到而略去；本书补全」。
- `:601/:655/:661` narration → ✅ 已清理（见维度四）。

### 增厚（超出十四讲，属 found_elsewhere/升格）
伴随 Ad/ad（sec:adjoint）、流形/⊞⊟（sec:manifold）、李群不确定度 + 复合协方差 + 四阶修正（sec:uncertainty）、SE(3) 6×6 雅可比 + $\mathbf{Q}_l$ 闭式（eq:se3-6x6 + 附录）。

**无新漏。** 唯一可议：见维度三的 minor。

---

## 维度二：正确性（对抗式）— **结论：PASS（零错误）**

### ★ 核心：Sim(3) 尺度雅可比 $\mathbf{J}_s$（eq:sim3-Js, line 629-638）

**逐项对照 Sophus `sim_details.hpp::calcW`（N=3，权威实现，verbatim 取得）：**

Sophus 用 `Omega = θ·a^`（全向量反对称），本章用单位轴基 $\mathbf{a}^\wedge,\mathbf{a}^\wedge\mathbf{a}^\wedge$。故须有
`chapter_As = Sophus_A·θ`、`chapter_Bs = Sophus_B·θ²`、`chapter_Cs = Sophus_C`。SymPy 符号化验证三者之差**恒等于 0**：

```
chapter_As - SophusA*theta   = 0
chapter_Bs - SophusB*theta^2 = 0
chapter_Cs - SophusC         = 0
```

逐系数：
- $C_s=\frac{e^\sigma-1}{\sigma}$ ←→ Sophus `C=(scale-1)/sigma` ✅
- $A_s=\frac{\sigma e^\sigma\sin\theta+(1-e^\sigma\cos\theta)\theta}{\sigma^2+\theta^2}$ ←→ Sophus `A·θ` ✅
- $B_s=C_s-\frac{(e^\sigma\cos\theta-1)\sigma+(e^\sigma\sin\theta)\theta}{\sigma^2+\theta^2}$ ←→ Sophus `B·θ²` ✅

**与十四讲 (4.51) 原式逐项一致**（十四讲用同一单位轴基，OCR 源 line 772-774 与本章字字相符）。
**与 Eade §6 / Strasdat 2012 同构**（基的标号不同，本章 line 628 已注明）。

**数值核对**（随机 σ=0.3, φ 任意）：`t = J_s ρ` 与 `expm(ζ^)` 右上块之差 = 4.4e-16；`sR` 块之差 = 2.2e-16。✅

**两支退化分支（line 641-648）独立验证（SymPy 极限）：**
- σ→0：`Cs→1`、`As→(1-cosθ)/θ`、`Bs→(θ-sinθ)/θ`——**符号极限精确等于本章所写**，且代回还原 SE(3) 左雅可比（eq:left-jacobian）✅。与 Sophus Case 2（σ≈0）换基后一致。
- θ→0：`As→0`、`Bs→0`，故 $\mathbf{J}_s→C_s\mathbf{I}=\frac{e^\sigma-1}{\sigma}\mathbf{I}$——**符号极限精确确认** ✅。与 Sophus Case 4（θ≈0）一致。

> 结论：**eq:sim3-Js 及其全部退化分支完全正确，已对齐 Sophus/十四讲/Eade。** 无需任何修改。
> （注：本章 σ→0 推导文字「令 $e^\sigma\to1$ 代入」略口语，但结论的严格极限已由 SymPy 证实无误——属行文而非正确性问题，见维度三。）

### 抽查其余公式（全部独立验证通过）
| 公式 | 验证方式 | 结果 |
|---|---|---|
| Rodrigues eq:rodrigues | expm vs 闭式 | err 1.1e-16 ✅ |
| 左雅可比 eq:Jl/left-jacobian | 级数 vs 闭式 | err 4.4e-16 ✅ |
| $\mathbf{J}_l^{-1}$ (line 377) | vs `inv(Jl)` | err 1.1e-16 ✅ |
| $\mathbf{J}_r=\mathbf{J}_l(-\phi)=\mathbf{J}_l^\top$ (eq:Jr, 框 line 381) | 三式互验 | err 0.0 ✅ |
| $\mathbf{t}=\mathbf{J}\boldsymbol\rho$ (thm:se3-exp) | expm vs $\mathbf{J}\rho$ | err 4.4e-16 ✅ |
| **BCH 三阶 eq:bch**（含 $+\frac1{12}[A,[A,B]]-\frac1{12}[B,[A,B]]$ 符号） | logm vs 截断 | 残差 1.9e-5 = O(4) ✅ 符号正确 |
| SE(3) BCH 左 eq:bch-se3-left | logm vs $\mathcal{J}_l^{-1}\Delta\xi+\xi$ | err 2.1e-9 ✅ |
| SE(3) BCH 右 eq:bch-se3-right | logm vs $\mathcal{J}_r^{-1}\Delta\xi+\xi$ | err 1.9e-9 ✅ |
| 伴随 Ad(T) eq:Ad + 移指数 eq:Ad-move | $T\mathrm{Exp}(\xi)=\mathrm{Exp}(\mathrm{Ad}\,\xi)T$ | err 1.1e-16 ✅ |
| 小伴随 eq:ad + $\mathrm{Ad}(\mathrm{Exp})=\exp(\mathrm{ad})$ | 6×6 验证 | err 2.8e-17 ✅ |
| 右扰动 SO(3) eq:right-pert-so3 $-\mathbf{R}\mathbf{p}^\wedge$ | 有限差分 | err 7.0e-7 ✅ |
| $\odot$ 恒等式 (line 466) $\xi^\wedge\tilde{\mathbf p}=\tilde{\mathbf p}^\odot\xi$ | 直接 | err 0.0 ✅ |
| 右扰动 SE(3) eq:right-pert-se3 $\mathbf{T}\tilde{\mathbf p}^\odot$ | 有限差分 | err 1.1e-6 ✅ |
| $\mathbf{Q}_l$ 块（Barfoot 8.91, 附录 line 768-772） | 6×6 级数 vs $[[\mathbf{J}_l,\mathbf{Q}_l],[0,\mathbf{J}_l]]$ | err 1.9e-16 ✅ |
| Sim(3) 扰动 eq:sim3-pert $[[\mathbf{I},-\mathbf{q}^\wedge,\mathbf{q}],[0]]$ | 有限差分（左扰动） | err 7.7e-7 ✅ |

**正确性维度无任何错误。**

---

## 维度三：行文/脉络 — **结论：PASS（1 处 minor 建议）**

通读全章：八节主线（为什么→群→李代数→指数对数→BCH→扰动→伴随→流形→不确定度→*Sim(3)→代码）脉络连贯，十四讲招牌钩子（$\mathbf{R}\mathbf{R}^\top=\mathbf{I}$「挤出」反对称、「重视这里的 $\mathbf{J}$」、RMSE 实例）全保留并讲透。Sim(3) 补全与独立性改写**自然融入，无突兀/重复/断裂**，仍像连贯独立教材。

### 逐项
- **`[minor]` line 641 σ→0 退化推导措辞略口语** — 位置：derivation 盒 σ→0 分支「令 $e^\sigma\to1$ 代入，并用 $\lim_{\sigma\to0}\frac{e^\sigma-1}{\sigma}=1$」。问题：$A_s,B_s$ 分子分母同含 $\sigma$，「令 $e^\sigma\to1$ 代入」不是严格的逐项代入（$A_s$ 分子 $\sigma e^\sigma\sin\theta$ 项随 $\sigma\to0$ 趋 0，须取整体极限）。**结论正确**（SymPy 已证极限恰为所写），仅过程叙述略松。
  - 建议（可选，纯行文）：改为「对 $A_s,B_s$ 取 $\sigma\to0$ 的整体极限（分子分母同阶，用 $e^\sigma=1+\sigma+O(\sigma^2)$ 展开）」，更贴合本书「复现级、每步可独立复现」标准。**不影响 PASS。**
- `[ok]` Sim(3) 引入段（line 608-625）从「单目尺度漂移→SE(3) 表达不出→显式 $s$」动机链顺畅；line 625 先以级数+换生成元解释 $\mathbf{J}_s$ 来历，再 derivation 给闭式，过渡自然。
- `[ok]` line 757「小结与过渡」明确「正文主干到此完整，Sim(3) 与附录为选读」，定位清楚。
- `[ok]` 左扰动并列段（sec:perturb, line 468-479）与右扰动主线呼应，用伴随互验（line 479），衔接好。

---

## 维度四：独立性 + LaTeX — **结论：PASS**

### 独立性（punt / ventriloquize / narration）
- **external_punt**：grep 全章——**零**。旧审计 :619 已消除（line 400 为「本书补全」正向陈述）。
- **ventriloquize**：零。旧 :400「十四讲坦言」已改中立。
- **narration_dependence**：仅 1 处边界——
  - **`[minor]` line 468** 段首「（含十四讲的分块推导）」+「为便于读者与原书对照」。位置：sec:perturb 左扰动对照段。问题：以「十四讲的分块推导」「与原书对照」作框架，轻微 narration。建议：「（左扰动分块推导，便于与采用左扰动的文献对照）」，去「十四讲的」「原书」。**程度轻微，不阻断。**
  - 其余十四讲/Barfoot 提及（line 86/88/90/174/482/688/870）均为**合法**：记号对齐（「与十四讲一致」）、中立多书对照（Barfoot 用 $\mathbf{C}$）、`\cite` 出处、延伸阅读——符合手册 §1.3「`\cite` 标出处 + 平衡多书对照合法」。

### 记号一致性（手册 §7）
- ✅ 右扰动为主 $\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\phi)$、$\mathbf{J}_r$ 主线（note line 90, sec:perturb, eq:right-update）。
- ✅ $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前（note line 88, def:so3-se3）；$\boldsymbol\rho\neq\mathbf{t}$、$\mathbf{t}=\mathbf{J}\boldsymbol\rho$（insight line 338, 误解表）。
- ✅ $\mathbf{R}$（非 Barfoot 的 $\mathbf{C}$）；$\mathrm{Exp}/\mathrm{Log}$ 算子；$(\cdot)^\wedge/(\cdot)^\vee$ 重载已声明（note line 244）。
- **`[minor]` 一致性观察**：eq:sim3-pert（line 653-657）复刻十四讲 (4.53)，而十四讲该式是在**左**扰动下导出（十四讲 line 784「设给予 Sp 左侧一个小扰动」）。本章 Sim(3) 节未显式标注此雅可比的扰动侧；全书主线为右扰动。数值上该 4×7 式确为**左**扰动结果（已验证）。建议：在 line 652-658 附近一句注明「（此处沿用左扰动以对接 ORB-SLAM/原文献；右扰动版相差一个 $\mathrm{Ad}$）」，与 sec:perturb 的右扰动主线对齐，免读者误用。**不阻断（Sim(3) 为选读 + 与十四讲一致）。**

### LaTeX 健康
- ✅ 环境 begin/end **全配平**（equation/align/align*/derivation/pitfall/practice/note/insight/theorem/definition/proposition/codebox/figure/table/tikzpicture 逐一核对，零失衡）。
- ✅ `\cref`/`\label`：所有 `\cref` 目标均有对应 `\label`（含外部章标签 ch:nlopt/ch:vo/ch:rigid_body/ch:slam_est/ch:vio）；**零悬空引用、零重复 label**。
- ✅ eq:sim3-Js 前向引用（line 625「见下面的推导盒」）规范。
- ✅ `\curlywedge`（amssymb）、`\odot` 标准符号；codebox 注释纯 ASCII（`^T`/`^-1`，符合 §7）。
- ✅ 图 tikz 重画（fig:lie-manifold）；fig:rodrigues 用 `figures/pdf/`。

---

## 附：本复核未发现需 NEEDS-FIX 的问题

四维全部 PASS。两处 `[minor]` 均为可选润色（σ→0 极限措辞、Sim(3) 扰动侧标注 + line 468 narration），**不影响出版正确性与独立性**。对抗式重点 eq:sim3-Js 经符号 + 数值 + 权威实现三重独立验证，结论：**正确，无需修改**。
