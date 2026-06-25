# 独立复核报告：`parts/P2_slam/visual_odometry.tex`

> 复核员：独立复核员（fresh，未参与之前润色）。模式：对抗式、只读。
> 复核对象：成稿章 `parts/P2_slam/visual_odometry.tex`（1787 行）。
> 对照：十四讲 `07_视觉里程计1.md` / `08_视觉里程计2.md`；审计 `docs/十四讲吸收审计.md`「ch07/ch08」；规范 `docs/项目交接手册.md` §1/§6/§7。
> 数值/符号验证：sympy + numpy 独立重算（右扰动 2×6 雅可比逐元素、左右伴随、ICP 左右雅可比、八点法行、ICP-SVD、金字塔自洽），见各节「证据」。
> 日期：2026-06-18。

## 总评

**MINOR** —— 知识完整、脉络连贯、四维核心结论全部对抗式验证通过（右扰动 2×6 逐元素雅可比、左右伴随关系、ICP 左右雅可比、金字塔内参缩放自洽性均经 sympy/numpy 独立重算确认正确）。仅 1 处 minor 代码瑕疵（多层光流 `calc` 中 `Eigen::Matrix2d H` 未零初始化，反向模式下首次累加进未定义内存）+ 数处可选的独立性微调（残留「Handbook 提醒/的关键图（图 7.4）」narration）。无 NEEDS-FIX 级问题。

---

## 维度一：知识完整（十四讲 ch7/ch8）

**结论：复核无误（全量覆盖）。**

逐主题核对，ch7/ch8 全部落地（节锚点均在）：

| 主题 | 教材锚点 | 状态 |
|---|---|---|
| ORB=Oriented FAST+Rotated BRIEF / 灰度质心 / FAST-12 预测试 / 金字塔尺度 | `sec:vo-orb-kp`,`sec:vo-brief` | ✅ 全（含手写质心 sin/cos、SSE popcount） |
| 特征匹配 / 汉明 / 暴力 / FLANN / 2×min+30 经验 | `sec:vo-match` | ✅ 全（含两版 OpenCV/手写代码 + 数值例 Max95/Min4） |
| 对极约束 / E / F / 八点法 / SVD 四解 / 奇异值投影 / 五点法 | `sec:vo-epipolar`,`sec:vo-eight-point` | ✅ 全（逐步推导不跳） |
| 单应 DLT / 退化（共面/纯旋转）/ F+H 选优 | `sec:vo-homography` | ✅ 全 |
| 三角化最小二乘 / 视差—深度矛盾 / 延迟三角化 / 深度滤波 | `sec:vo-triangulation` | ✅ 全 |
| PnP：DLT / P3P 余弦定理 / EPnP / BA 重投影 + 2×6 雅可比 | `sec:vo-pnp` | ✅ 全（含手写 GN + g2o 两版代码） |
| ICP：去质心 SVD / det<0 修正 / 非线性右扰动 / 全局最优定理 / 混合 PnP-ICP | `sec:vo-icp` | ✅ 全 |
| 光流 LK：约束方程 / 窗口最小二乘 / 反向 / 金字塔 / 单层+多层代码 | `sec:vo-flow` | ✅ 全 |
| 直接法：光度误差 / 链式雅可比 / 稀疏-半稠密-稠密 / 单层+多层代码 | `sec:vo-direct` | ✅ 全 |
| 前端架构（Handbook 增厚）：PTAM/关键帧/局部 BA/Schur/混合/学习型 | `sec:vo-frontend` | ✅ 全（超出十四讲，属规范鼓励的 Handbook 广度） |

**审计「应修复项」全部落地**（对照 `docs/十四讲吸收审计.md` ch07/ch08 节）：
- `[info]` ch07 右扰动 2×6 **逐元素显式矩阵**：已补 `eq:vo-jac-right-trans`(`:815`) + `eq:vo-jac-right-rot`(`:823`)，与左扰动 `eq:vo-jac-left`(`:834`) 并列，主线不再靠习题兜底。
- `[info]` ch08 多层金字塔 **可运行 driver 代码**：已补 `OpticalFlowMultiLevel`(`:1222`) 与 `DirectPoseEstimationMultiLayer`(`:1412`) 两段完整 driver。
- `[info]` ch08 习题：Baker--Matthews 四象限（`:1280` note + `:1292` 习题3）、中心差分梯度缺点（`:1477` 习题4）、单目逆深度联合优化（`:1450` insight + `:1478` 习题5）均已落地。

---

## 维度二：正确性（对抗式重点）

**结论：核心公式与代码自洽性全部独立验证通过（无错）。**

### 2.1 ★ BA 重投影误差对位姿的右扰动 2×6 雅可比逐元素式（`eq:vo-jac-right-trans`/`eq:vo-jac-right-rot`）

`[verified-correct]` 位置 `:813–829`。**独立 sympy + numpy 重算，全部一致。**

- **平移块** `-M R`（`eq:vo-jac-right-trans`）：与因子形 `de/dP' · [R, -RP^∧]` 的前 3 列 **逐元素相等**（sympy 化简差为零矩阵）。
- **旋转块** `M m^∧`（`eq:vo-jac-right-rot`，`m=RP=P'-t`）：sympy 验证 `M·hat(m)` 与章中所列 2×3 矩阵 **逐元素相等**（差为零矩阵）。
- **完整旋转块** `(M m^∧)R = M R P^∧`：成立，但**前提是 R∈SO(3)**（用到 `R P^∧ = (RP)^∧ R`，该恒等式需 R 正交）。用真旋转矩阵数值验证 `M R P^∧` 与 `(M m^∧) R` 差 ~5.7e-14。章中正文已正确指出该恒等式（`:821`「用恒等式 RP^∧=(RP)^∧R=(P'-t)^∧R」），逻辑闭合。
  - 提示（非错误）：恒等式仅对真旋转成立；若读者误用一般 3×3 矩阵会不等。正文措辞已默认 R 为旋转，无需改。
- **右扰动整体** vs **有限差分**（中心差分，eps=1e-6）：`[-MR, MRP^∧]` 与 `e(T·Exp(δξ))` 的数值雅可比差 ~8.3e-8（纯截断误差）。**确认正确。**
- **左扰动** `eq:vo-jac-left` vs 有限差分 `e(Exp(δξ)·T)`：差 ~8.9e-8。**与十四讲式(7.46) 同形，确认正确**（亦与代码 `:872` 手写 GN、`:921` g2o `linearizeOplus` 逐元素一致，符号取 `-[matrix]`）。
- **伴随一致性**（章中 `:840` 声称 `J_左 = J_右 · Ad(T)^{-1}`）：数值验证差 ~2.8e-13。**确认两版经伴随精确互转**（与规范 §7 右扰动主线 + 引用左扰动并列转换的要求一致）。

> 小结：右扰动雅可比是本次复核最关键、最易错处，逐元素 + 因子形 + 伴随 + 有限差分四路交叉验证全部通过。

### 2.2 ★ 多层金字塔 driver 代码自洽性

`[verified-correct]` 位置 直接法 driver `:1412–1442`、光流 driver `:1222–1259`。

- **直接法内参缩放自洽**（深度不缩放）：独立验证「`P = Z·K_lvl^{-1}·[p_lvl·s; 1]`，内参与参考像素同乘 s、深度 Z 固定 ⇒ 反投影 3D 点 P 在各层完全不变」（4 个尺度 1/0.5/0.25/0.125 下差 0.0）。前向投影 `u_lvl = s·u_full` 亦精确成立。章中 `:1443`「像素与内参都按本层缩放、互相抵消，故同一物理点 P 在各层一致——这正是深度不缩放仍自洽的原因」**论证正确**。代码 `fx=fxG*scales[level]`(`:1436`)、`px_ref·scales[level]`(`:1435`)、`depth_ref` 原样传入(`:1439`)、`T21` 跨层延续(`:1443`) 均与十四讲 `DirectPoseEstimationMultiLayer` 一致。
- **光流 driver 不缩内参**：章中 `:1261`「光流不含内参，故无内参缩放问题，这与下文直接法不同」**正确**（光流单层 `calc` 残差只用 `kp.pt±x+dx`，无 fx/fy）。
- **光流 driver 坐标缩放逻辑**：逐步追踪 `dx=kp2_p-kp1_p` 取上层结果作初值、`calc` 以 `kp1_p[i]` 为参考基、层间 `kp1_p,kp2_p /= scale` 同步放大 —— 逻辑与十四讲 `OpticalFlowMultiLevel`（经 `has_initial=true` 携带初值）**等价且正确**。

### 2.3 抽查：对极/八点法/ICP-SVD

- `[verified-correct]` 对极约束推导 `:387–423`：`x₂^T t^∧ R x₁=0`，`t^∧x₂⊥x₂` 消深度，与十四讲式(7.1)–(7.10) 逐步一致。
- `[verified-correct]` 八点法行 `eq:vo-eight-row`(`:451`)：numpy 验证 `[u₂u₁,…,1]·e == x₂^T E x₁`（差 0.0）。
- `[verified-correct]` E 的 SVD 恢复 R,t（`eq:vo-Rt1/Rt2`）+ 四解筛选 + 奇异值投影 `(σ₁+σ₂)/2`：与十四讲式(7.15)–(7.16) 一致。
- `[verified-correct]` ICP-SVD（`eq:vo-icp-R`）：numpy 验证 `W=∑q₁q₂^T`、`R=UV^T` 恢复真旋转（差 6.7e-16），`det<0` 取负 修正正确。
- `[verified-correct]` ICP 右扰动雅可比 `eq:vo-icp-jac-right`(`-[R, -Rp'^∧]`) vs 有限差分：差 2.3e-10；左扰动 `-[I, -(Tp')^∧]` vs 有限差分：差 1.8e-10。**两版均正确，伴随关系成立。**
- `[verified-correct]` P3P 变量定义 `:730`：`v=AB²/OC²`、`u·v=BC²/OC²`⇒`u=BC²/AB²`、`w·v=AC²/OC²`⇒`w=AC²/AB²`，最终二元二次方程组与十四讲一致。
- `[verified-correct]` 单应分解法向量：章 `:527` 写 `n=[0,0,1]^T`（沿光轴），**已纠正**十四讲 OCR 错字「1^T」。

### 2.4 唯一 minor 代码瑕疵

`[minor]` 位置 `:1187`（多层光流 `calc` 函数）—— **问题**：`Eigen::Matrix2d H;` 未零初始化；反向光流模式（`inverse=true`）下，`:1189` 仅当 `!inverse` 才 `H=Zero()`，而 `:1207` 在 `iter==0` 执行 `H += J*J.transpose()`，即**首次累加进未定义内存**（Eigen 默认构造不清零）。十四讲源（08 md `:225`）写的是 `Eigen::Matrix2d H = Eigen::Matrix2d::Zero();`，润色压缩时丢了 `= Zero()`。
**建议/正确形式**：`Eigen::Matrix2d H = Eigen::Matrix2d::Zero();`（与源一致）。属示意代码的内存初始化瑕疵，不影响公式正确性，但读者照抄会在反向模式得到随机 H。

---

## 维度三：行文/脉络（作者最看重）

**结论：复核无误（保留并显著提升，新增内容自然融入）。**

- **五法递进清晰**：知识导航 tikz（`:39`）按「2D-2D / 3D-2D / 3D-3D × 特征点/直接法」两轴展开；正文以「信息越少越难 → 信息越多越准」串起 对极→三角化→PnP→ICP，再「特征点太慢 → 省描述子(光流) → 边跟踪边调位姿(直接法)」过渡到 ch8。递进自然，无断裂。
- **右扰动 2×6 逐元素展开融入自然**：`derivation`(`:783`) 先给公共投影项 `eq:vo-de-dP`，再右扰动主线（因子形 → 逐元素 trans/rot 块），再并列左扰动闭式 + 伴随说明；结构「公共→主线→对照」清晰，不突兀。习题1（`:948`）顺势让读者用伴随验证两版一致，呼应正文。
- **多层 driver 融入自然**：两段 driver 各以 `practice` 承接单层代码，光流 driver 末点出「光流不缩内参」、直接法 driver 末点出「内参随层缩放、深度不缩放」，并各配 `pitfall`（`:1458` 缩图忘缩内参）双向呼应。
- **Baker--Matthews / 逆深度 新增融入自然**：Baker--Matthews 以 `note`(`:1280`) 承接「正向/反向光流」，升格为四象限统一框架，再入习题3；逆深度以 `insight`(`:1450`) 承接「单目直接法更难」，指向 LSD-SLAM/DSO，再入习题5。均为「正文 insight/note + 习题落地」的规范增量，未打断主线。
- **仍像连贯教材**：v5.0 教学层齐全（前置自测/目标/导航/桥接/跳过会怎样/各节动机-反面-历史-理论-陷阱-练习/常见误解/小结三表/累积项目/延伸阅读/后续关系/故障排查/API 速查），且技术内容全量（符合规范 §6.2「教学层叠加在全量内容上」）。

---

## 维度四：独立性 + LaTeX

**结论：基本干净；数处可选 narration 微调（minor）。**

### 4.1 独立性

- `[clean]` **零 external_punt**：grep「详见/参见/见十四讲/见原书/闭式…见/此处略/从略/不再赘述/略去」**全空**（符合规范 §1.3 铁律）。
- `[clean]` **零 meta-narration**：grep「worker/审稿/复核/TODO/待补充/占位/\pz/\rebuilt」**全空**。
- `[clean]` **代码文件名已降为脚注**：审计曾标 `:276/:280/:311/:530` 等「十四讲 orb_self.cpp / 数值例」，现已改为「代码改写自 `slambook2/ch7/...`\cite」脚注（`:246`,`:280`,`:530`,`:617`,`:854` 等），数值例改本书实验口吻直述（`:276`,`:311`）。审计 ch07-C 的系统性问题**已清理**。
- `[clean]` **源书式号锚点已清理**：审计曾标 `:807`「十四讲式(7.46)」，现正文已无该式号引用（改「与\cite 一致」式表述，`:831`）。
- `[minor]` 残留 narration（可选微调，均带 `\cite`，且本章自有完整推导/图）：
  - `:1537`「Handbook 的关键图（图 7.4）对比…」—— 以源书图号作叙述锚点（同审计对「式(7.46)」的顾虑）。**建议**：「现代 SLAM 中（\cite{carlone2026handbook}）EKF 边缘化导致稠密图、关键帧保持稀疏」直述，去「图 7.4」锚点。
  - `:1593`「Handbook 提醒：广角用…」—— 轻微 ventriloquize。**建议**：「相机模型选型上，广角用 Brown--Conrady、鱼眼用 KB…（\cite{carlone2026handbook}）」直述。
  - `:744`「Handbook\cite{} 从概率给出干净推导」/ `:1486`「本节按 Handbook\cite{} 的视角」—— 边界 narration，伴本章自有推导，可接受；纯净化可改本书直述 + \cite 移句尾。
  - 注：`:96/:781/:882/:936/:1011/:1054/:1336` 的「十四讲（左扰动）」均为**规范明确鼓励**的「本书右扰动 vs 十四讲左扰动」多书对照（规范 §7「Barfoot/十四讲左扰动处并列转换」），**合法，不必改**。

### 4.2 LaTeX / 记号 / 环境

- `[clean]` **`\cref` 全解析**：章内 `eq:vo-*`/`sec:vo-*`/`thm:vo-*`/`fig:vo-*` 引用全部有定义（comm 比对无悬空）；跨章 `eq:right-pert-se3`(lie)、`eq:gn-normal`/`eq:lm`/`eq:schur`/`eq:retraction`/`thm:gn`(nlopt)、`eq:pinhole`/`eq:intrinsic-scalar`(camera)、`ch:camera/imu/lie/nlopt/pointcloud/vio` **全部存在**。无硬编码章号/式号（符合 §7）。
- `[clean]` **bib 键全解析**：`baker2004lk/barfoot2024state/carlone2026handbook/engel2014lsd/engel2017dso/gaoxiang2019slam14/hartley2004multiview/nister2004fivepoint/sola2018micro` 均在 `refs.bib`。
- `[clean]` **代码注释纯 ASCII**：codebox 内 `//` 注释 grep 数学 unicode（ᵀ⁻¹ηΛΣδξ←→≈ 等）**全空**（符合 §7「注释禁 Unicode 数学符号」）；用 `^T`/`^-1`/`P^`/`m^` 等 ASCII，中文注释 OK。
- `[clean]` **记号一致**：右扰动 `T Exp(δξ)`、`δξ=[δρ;δφ]`（平移在前）、`Σ_v`/`Λ`、`R∈SO(3)`/`T∈SE(3)`、Hamilton 四元数（本章未涉），与 §7 锁定记号一致。`\simeq` 尺度相等、`E` 的 `Σ` 标注「非协方差」均规范。
- `[clean]` `codebox[语言]{标题}` 签名、`derivation`/`insight`/`pitfall`/`practice`/`note`/`theorem` 环境用法与 styles 一致；`\boxed` 用于关键结论（对极约束/ICP-R）；表格 `[H]` + `\toprule/\midrule/\bottomrule` 规范。
- 注：本环境无 Docker，未做实编译（与手册 §10 收尾「唯一剩余=有 Docker 处跑 compile.sh」一致）；静态层面 begin/end 配平、label 无重复（章内 `eq:vo-*` 无重名）。

---

## 附：复核方法留痕（可复现）

- 右扰动 2×6 逐元素：sympy 符号化 `M·hat(m)` 与 `-M R`，与章中矩阵逐元素相减化简为零；`M R P^∧` vs `(M m^∧)R` 用真旋转 numpy 验证（5.7e-14）。
- 左右 + 伴随：numpy 真旋转 + SE(3) 指数，`e(T Exp δξ)` / `e(Exp δξ T)` 中心差分 vs 解析（8e-8），`J_左 = J_右 Ad(T)^{-1}`（2.8e-13）。
- ICP 左右雅可比、八点法行、ICP-SVD：numpy 中心差分 / 直接代入（1e-10 ~ 1e-16）。
- 金字塔自洽：`P=Z·K_lvl^{-1}[p·s;1]` 跨尺度不变（0.0）、`u_lvl=s·u_full`（0.0）。
