# Handbook 吸收侦察报告 A：估计·优化内核组

> 范围：The SLAM Handbook 第 01/02/04/06/18 章（因子图 / 高级状态表示 / 可微优化 / 可证最优 / 空间 AI 计算结构）。
> 任务：只读侦察（recon），不改任何 .tex / refs.bib。目标是「现有那条已融十四讲+Barfoot 的叙事线长什么样、Handbook 这部分该如何**织进去**」，而非「内容堆到哪个文件」。
> 日期：2026-06-18。

---

## 0. 总体判断（先读这段）

**关键发现：现有教材的吸收程度远超本任务的初始假设。** 通读现有 6 章后，本组 5 个 HB 章里，**Ch1（因子图）与 Ch2（高级状态表示）已被大面积吸收**——不是"未吸收待织入"，而是"已织入、待补缺口 + 局部增量"。真正的大增量集中在 **Ch4（可微优化）、Ch6（可证最优）、Ch18（空间 AI）** 这三章前沿内容。

证据（现有 .tex 已在主线中、且已 `\cite{carlone2026handbook}`）：

| 现有落点 | 已吸收的 HB 内容 |
|---|---|
| `slam_state_estimation.tex` §`sec:est-fg`（因子图统一视角）| HB Ch1 §1.1–1.6 几乎全部：因子图定义、MAP=NLS、白化、稀疏雅可比/信息矩阵、COLAMD/fill-in（连 9399→4168 非零元的例子都搬了）、变量消元=矩阵分解 |
| `slam_state_estimation.tex` §`sec:est-smooth-filter`| HB Ch1 §1.7 的 Bayes 树/iSAM2：弦化贝叶斯网、团树、增量编辑、两条性质、constrained COLAMD（仅完整逐步算法留作去向）|
| `slam_state_estimation.tex` §`sec:est-steam`| HB Ch2 §2.2 连续时间：LTI SDE、闭式转移、WNOA 先验、块三对角、Schur vs Cholesky、iSAM 嵌入 |
| `lie_theory.tex`（整章）| HB Ch2 §2.1 全部：SO(d)/SE(d)、exp/log、伴随、左/右雅可比、`⊞/⊟`、retraction/黎曼优化、**`⊙` 齐次点算子 + 完整代数**（`subsec:odot-algebra`，连 `⊚`/`circledcirc` 配对与搬过位姿的伴随恒等式都有）|
| `nonlinear_optimization.tex` §`sec:nlopt-gn/lm/solve/sparse/ba/posegraph/manifold/robust`| HB Ch1 §1.3–1.4 求解器全家（GN/LM/Dogleg/信赖域/Cholesky/QR/√SAM/Schur）+ HB Ch4 §4.4.1 的 IRLS/鲁棒核 |
| `nonlinear_optimization.tex` §`sec:nlopt-gvi`（高斯变分推断）| 已是"MAP→Laplace→GVI"的进阶推广，是 HB Ch4 隐式微分 / HB Ch6 CRLB 的天然邻居 |
| `point_cloud_processing.tex`、`visual_odometry.tex`| 已 `\cite{carlone2026handbook}`：TEASER（截断最小二乘+半定松弛）、Power BA、VarPro 免初始化 |

主线记号已统一且与 HB 兼容：右扰动 `R·Exp(δφ)`、`SE(3) ξ=[ρ;φ]`、信息矩阵 `Λ`、numerator-layout、`⊞/⊟`（本书右乘=GTSAM 约定，HB 记 `⊕/⊖`）、`⊙` 算子直接采 HB 记号。`ch:slam_est` 前言已显式声明三书（Barfoot/十四讲/Handbook）记号映射。

**对融合的含义**：本组工作的主体不是"把 HB 内容搬进来"，而是
1. **Ch1/Ch2**：精修式补缺（少量真增量）+ 防止已有 `\cite` 沦为 ventriloquize；
2. **Ch6**：把现有多处"留作前沿/开放阅读"的 SE-Sync 钩子**兑现**为一节自包含论述（这是本组最大、最实在的一块新内容）；
3. **Ch4**：可微优化/双层优化是全书空白，需新增子节（自然落点 `nonlinear_optimization.tex` 末或 `slam_system.tex` 学习-几何接口处）；
4. **Ch18**：空间 AI / GBP / 硬件计算结构是全书空白，且**没有合适的现有章可挂**（P5 工程实践是 5 行 stub、未进 book.tex）——这是 slice 级结构决策的关键。

---

## 1. HB Ch01 — Factor Graphs for SLAM

**作者**：Dellaert, Kaess, Barfoot。**性质**：SLAM 后端的概率-优化骨架（因子图→MAP→最小二乘→稀疏分解→变量消元→Bayes 树/iSAM）。

### 覆盖度：**已有（约 85–90%）**

### 现有落点
- `parts/P1_estimation/slam_state_estimation.tex`
  - §`sec:est-fg`（因子图：现代统一视角）→ `sec:est-fg-def` / `sec:est-factors`（因子表 `tab:factors`）/ `sec:est-fg-linear`（白化+稀疏+COLAMD）/ `sec:est-elimination`（`thm:elimination` 变量消元=分解）/ `sec:est-smooth-filter`（平滑vs滤波 + Bayes树/iSAM2）
  - `thm:fg-map`（因子图 MAP = NLS）；`def:factor-graph`；§`sec:est-batch-recursive` 的块三对角 + Cholesky 平滑器（= HB §1.3 的稀疏分解的递归侧写）
- `parts/P1_estimation/nonlinear_optimization.tex`
  - §`sec:nlopt-gradient`（SD/牛顿）、`sec:nlopt-gn`（`thm:gn`）、`sec:nlopt-lm`（LM + ρ 增益比 + 信赖域）、`sec:nlopt-solve`（白化/Cholesky/QR/√SAM）、`sec:nlopt-sparse`、`sec:nlopt-ba`（箭头矩阵+Schur）、`sec:nlopt-posegraph`

### HB 增量清单（相对现有，真正多出来的）
1. **Powell's Dogleg 的几何图与 gain ratio ρ 的显式公式（HB eq 1.39）**：现有 `sec:nlopt-lm` 提了信赖域与 ρ，但 Dogleg 的"先分别算 GN/SD 步、被拒时只换组合方式、每步只一次分解"这一**省分解**论点（LM 被拒要重分解 vs PDL 不用）值得作为 LM 节的一个对照段补全。
2. **变量消元的 QR 变体逐步图解（HB §1.6.2，eq 1.48–1.49）**：现有 `thm:elimination` 给了结论（消元=分解、条件均值/协方差闭式），但 HB 把"消去 ℓ₁ → 部分 QR → 产生条件 p(xⱼ|sⱼ)+分隔集新因子 τ(sⱼ)"画成 5 步全过程（Fig 1.10–1.12）。现有是"定理+一段话"，可补一个**最小数值消元例**使其自包含、可独立读懂（满足全吸收标准）。
3. **Bayes 树更新的"受影响子树"图（HB Fig 1.14）+ 选择性重线性化的缓存细节**：现有 `sec:est-smooth-filter` 已讲两条性质与 constrained COLAMD，但"加一个 p₁–p₃ 因子只影响左支、绿支不动"这一**可视化的增量编辑**只是文字。属可选深化。
4. **历史注记**（SRIF/Mariner 10/Maybeck 引言、SAM 命名由来）：现有未收，属"叙事调味"，可选。

### 融合策略（重中之重）
- **不新增节**。本章定位是"已织入、精修补缺"。
- **Dogleg 段**：在 `sec:nlopt-lm` 讲完 LM 的"拒绝步→重分解最贵"之后，承接一句"若希望被拒时不重分解，可改用 Powell dogleg"，织入 Dogleg 几何（GN 步 + SD 步在信赖域边界的折线）与 ρ 公式。过渡保持现有"信赖域"主线，是**深化现有论述**。
- **消元数值例**：在 `thm:elimination` 之后补一个 `\begin{derivation}`（本书已有此环境），用现有那个小型因子图（`fig:factor-graph`）走一遍部分 QR 消元，落到上三角 R 的一两个块。**局部深化**，让"消元=分解"从断言变成可验算的演示。
- **Bayes 树增量编辑图**：若 `sec:est-smooth-filter` 要更自包含，补一段"受影响路径"的文字+（可选）TikZ；但优先级低于上面两项。

### 独立性风险
- **高风险**：§`sec:est-fg` 开头现写"本节取自 SLAM Handbook 第 1 章（Dellaert–Kaess–Barfoot），把…完整链条逐段记录"——这句"取自…逐段记录"接近 narration_dependence，**应改写**为本书口吻（如"因子图是当代 SLAM 后端的通用语言\cite{...}；本节建立…"）。这是清理重点。
- §`sec:est-smooth-filter` 多处 `\cite{carlone2026handbook}` 紧跟"Bayes 树把…团组织成树"是合法的（事实+引用），保留。
- 注意别在补 Dogleg/消元例时写成"Handbook 给出 Fig 1.x"——直接本书重绘 + `\cite`。

### 是否新建：**否**（精修现有 §`sec:est-fg`、`sec:est-elimination`、`sec:nlopt-lm`）。
### 工作量：**小**（增量本身小；独立性清理是主要动作）。

---

## 2. HB Ch02 — Advanced State Variable Representations

**作者**：Barfoot, Dellaert, Kaess, Blanco-Claraco。**性质**：把因子图里被刻意回避的"状态变量真实是什么"补上——(1) 流形/李群优化，(2) 连续时间轨迹（样条 + GP）。

### 覆盖度：**已有（约 85%）**

### 现有落点
- `parts/P0_math/lie_theory.tex`（对应 HB §2.1 全部）
  - §`sec:group`/`sec:lie-algebra`/`sec:exp-log`（`thm:so3-exp` Rodrigues、`thm:se3-exp` 左雅可比、`prop:surjective`）
  - §`sec:bch`（BCH + `eq:Jl`/`eq:Jr` 左右雅可比闭式 + 群↔代数翻译词典）
  - §`sec:perturb`（右扰动 `eq:right-pert-so3/se3`）、§`sec:adjoint`（`eq:Ad`、`Ad(T)=[[R,t^R],[0,R]]`）
  - §`sec:manifold`（retraction、`eq:boxplus` ⊞/⊟、黎曼优化框架）、§`sec:lie-unc`（李群上高斯、`|det J|` 体积元、协方差传播）
  - §`sec:lie-interp`（李群插值=匀速运动，STEAM 几何地基）、§`sec:sim3`、§`sec:se23`（SE₂(3) 扩展位姿）、§`sec:lie-symmetry`（对称/不变/等变→InEKF/EqF）
  - `subsec:odot-algebra`（`⊙`/`⊚` 齐次点算子完整代数 + 伴随恒等式 `(Tp)^⊙=T p^⊙ 𝒯⁻¹`）
- `parts/P1_estimation/slam_state_estimation.tex` §`sec:est-steam`（对应 HB §2.2 连续时间）
  - `sec:est-steam-prior`（LTI SDE、`eq:steam-Phi-Q` 闭式转移+累积噪声）、`sec:est-steam-factor`（右扰动线性化）、`sec:est-steam-map`（`prop:steam-sparse` 块三对角不稠密）
- `parts/appendix/matrix_calculus.tex` §`sec:matderiv-vanloan`（Van Loan：连续→离散 Φ,Qₖ，正是 HB §2.2.3 GP 核矩阵 Q 的算法）

### HB 增量清单
1. **样条（B-spline / 累积形式 / 李群上的累积样条，HB §2.2.1 + §2.2.4.1）**：这是相对现有 `sec:est-steam` 的**最大真增量**。现有连续时间是"GP/STEAM 单线"（非参数核方法）；HB 把连续时间讲成"**参数法（样条）vs 非参数法（GP）**"两支并列：
   - 参数样条：`p(t)=Σ Ψₖ(t)cₖ`、局部支撑→稀疏五元因子、可微取速度。
   - **累积样条**（`eq:2.54`）+ 李群上累积形式（`eq:2.61` `T(t)=Exp(ψᶜ Log(TₖTₖ₋₁⁻¹))·Tₖ₋₁`）+ 控制点扰动到插值位姿扰动的一阶关系（`eq:2.64–2.67`，含左雅可比 A(α)）。
   - 样条 vs GP 的本质区别：**GP 有运动先验因子做正则、样条没有**（HB §2.2.4.2 末）。
2. **GP-on-Lie-group 的局部变量构造（HB §2.2.4.2，eq 2.68–2.75）**：random-walk SDE 在控制点间的局部变量 ξₖ(t)、Φ⁻¹ 双对角、误差 `eₖ=Log(TₖTₖ₋₁⁻¹)` 的伴随线性化 `eq:2.75`。现有 `sec:est-steam` 用的是向量空间/右扰动 WNOA，**李群版的 GP 先验**这一层可补。
3. **"测量时刻 / 估计时刻 / 查询时刻三者可分离"的论点 + GP inducing points 减少控制点**（HB §2.2.3 末）：现有 STEAM 提了 GP 插值查询，但"一个 lidar scan 一个控制点、却用上每点时间戳"这一**去运动畸变**动机讲得更透。

### 融合策略
- **李群机理（§2.1）**：**几乎全跳过**（已在 `lie_theory.tex`）。唯一动作是独立性清理（见下）+ 确认 `⊙` 记号一致（已一致）。
- **样条（真增量）**：在 `slam_state_estimation.tex` §`sec:est-steam` 内部，于"局部 GP/LTI 先验"开讲之前或之后，**新增一个子节** `sec:est-steam-spline`（参数样条 + 累积样条 + 李群累积样条）。过渡句承接现有连续时间动机："连续时间轨迹有两条路——把位姿写成已知基函数的加权和（参数/样条），或写成核函数（非参数/GP）；本书 STEAM 主线用后者（运动先验自带正则），此处先补前者并点明二者取舍。" 这是**局部重构 §`sec:est-steam` 的引入**（从"GP 单线"升级为"样条/GP 双线"），属中等改动。须自包含：累积样条公式、局部支撑稀疏性、李群累积式 `eq:2.61`、控制点扰动关系 `eq:2.64` 都要落地可读。
- **GP-on-Lie**：在 `sec:est-steam-prior`/`sec:est-steam-factor` 里，把现有向量空间 WNOA 的扰动线性化补一段"状态在 SE(d) 上时，对局部变量 ξₖ(t) 用同样的右扰动机理，误差 `Log(TₖTₖ₋₁⁻¹)` 的雅可比经伴随给出（`eq:2.75`）"。**深化现有论述**，与本书右扰动主线一致（注意 HB 此处用左扰动，须转成本书右扰动并说明经伴随等价）。
- **三时刻分离 + inducing points**：织入 `sec:est-steam-map` 或小结，一两段即可。

### 独立性风险
- `lie_theory.tex` §`sec:manifold` 多处"（Handbook、Boumal）""Handbook 记作 ⊕/⊖""Handbook\cite{} 记号"——属合法的平衡对照（点名多书取法 + 本书统一），保留；但通读时确认没有滑成"Handbook 说…"。
- 补样条时，HB 原文左扰动（`eq:2.61` footnote 解释为何在 sensor 帧扰动）——**改写为本书右扰动主线**，并以本书口吻给出"部分文献在传感器帧用左扰动；本书统一右扰动，经伴随等价"的对照，避免照搬左扰动叙述。

### 是否新建：**子节**（`sec:est-steam` 内新增样条子节 + 深化 GP-on-Lie；李群部分不动）。
### 工作量：**中**（样条是实打实的自包含新内容，含李群累积式与扰动关系的推导）。

---

## 3. HB Ch04 — Differentiable Optimization

**作者**：Wang, Jatavallabhula, Mukadam。**性质**：把"学习前端 + 几何后端"缝成端到端可训练——双层优化（BLO），通过几何优化反向传播。这是**学习与几何的接口**，全书空白。

### 覆盖度：**几乎完全没有（约 10%，仅 Lie-group 求导地基已有）**

### 现有落点（仅地基，非主题本身）
- HB §4.1（NLS recap）：与 `slam_state_estimation.tex`/`nonlinear_optimization.tex` 完全重复，**跳过**。
- HB §4.3（流形求导）：`lie_theory.tex` §`sec:perturb`/`subsec:odot-algebra` 已有右雅可比、`⊙`、链式法则；`matrix_calculus.tex` 有 numerator-layout 链式法则。HB §4.3 的"右/左 ⊕⊖、`∂f/∂χ` 的极限定义、inversion/composition 雅可比闭式"在本书已有等价物——**大部分跳过/小补**。
- HB §4.4.1（IRLS/FastTriggs/鲁棒核）：`nonlinear_optimization.tex` §`sec:nlopt-robust` 已讲 M-估计/IRLS/Triggs；FastTriggs（只用一阶导、比 Triggs 稳）是**小增量**。
- HB §4.4.2（库 NumPy/Eigen/Ceres/g2o/GTSAM/PyTorch/Theseus/PyPose）：`nonlinear_optimization.tex` §`sec:nlopt-practice` 已对照 Ceres/g2o；可微优化库（Theseus/PyPose/CvxpyLayer）是增量。

### HB 增量清单（核心全新内容）
1. **双层优化（BLO）框架（HB §4.2，eq 4.2–4.3）**：上层 NN 损失 `U(y,x*)`、下层几何 NLS `x*=argmin L(y,x)`、关键梯度 `∂x*/∂y`。两个 SLAM 例（学习特征+BA、学习前端+PGO）。
2. **展开微分（Unrolled Diff，HB §4.2.1–4.2.2）**：把下层迭代展开成可 AutoDiff 的计算图；reverse/forward mode；截断展开；一步近似 + 有限差分绕过 Hessian（eq 4.8–4.10）。
3. **隐式微分（Implicit Diff，HB §4.2.3，eq 4.11–4.17）**：由下层最优性条件 `∂L/∂x*=0` 全微分得 `∂x*/∂y = -(H_xx)⁻¹ H_xy`；Hessian 太大→解线性系统 `Hq=v` + **快速 Hessian-向量积**（HVP，eq 4.15，"梯度的梯度"）；存储成本例（10⁶ 参数→4TB Hessian）。
4. **流形上的微分优化收尾（HB §4.3.2 例 4.4–4.5）**：SO(3) 上视觉-惯性旋转估计、机械臂复合雅可比——可作为现有 `⊙`/链式法则的**应用例**。
5. **数值陷阱（HB §4.4，例 4.6）**：四元数 Exp 的 `sin(ν)/ν` 在小角度的 Taylor 兜底（eq 4.41）——与本书 Hamilton 四元数主线相关，可作 pitfall。
6. **生态/趋势（HB §4.5）**：DROID-SLAM、BA-Net、DeepFactors、iSLAM（双向连接前后端）等。

### 融合策略
- **新增一个子节/小节**，自然落点二选一（建议放 `nonlinear_optimization.tex`，因 BLO 本质是"对优化解求导"，与该章 GVI/协方差回收同属"优化的高阶视角"）：
  - 落点 A（推荐）：`nonlinear_optimization.tex` 在 `sec:nlopt-gvi`（高斯变分推断）**之后**、`sec:nlopt-practice` 之前，新增 `sec:nlopt-diffopt`（可微优化：把后端嵌进端到端学习）。过渡句承接"前面把优化当作给定问题来解；本节反过来问：**如何对优化的解 `x*` 关于问题参数 `y` 求导**，从而让学习前端从几何后端反向受益（双层优化）"。这与 GVI 的"对优化的高阶处理"同调，是**深化全书优化论述的自然延伸**。
  - 落点 B（备选）：`slam_system.tex` 末（学习-几何混合架构处），但该章是工程 capstone、几乎不引入新数学，放纯理论的隐式微分会破坏其文本:代码比，故**不推荐主放**，可在其"延伸"提一句指向 A。
- **内容裁剪**：§4.1 全删（重复）。§4.3 的李群求导主体引用现有 `lie_theory.tex`（"流形上求导的机理见 \cref{ch:lie}"是合法的内部 cref，非 punt），只补 BLO 特有的 `∂x*/∂y`。子节自包含核心：BLO 定义、隐式微分公式 `eq:4.12` 的完整推导（由最优性条件全微分，本书读者已会矩阵求导，可独立读懂）、HVP 技巧、unrolled vs implicit 的取舍表。
- **接现有钩子**：`sec:nlopt-gvi-iekf`（IEKF→MAP 众数）、`sec:nlopt-cov`（Laplace 协方差回收）都涉及"最优解的二阶结构"，可在新子节开头一句话承接，强化"一个作者的连贯论述"。

### 独立性风险
- **中风险**：这是全新内容，最易写成"Wang 等提出 PyPose…""Theseus 库实现…"的库导览体（HB §4.4 本身就是库综述）。**对策**：把库放进"延伸/工程"小段 + `\cite`，正文只讲方法（unrolled/implicit/HVP）的本书口吻推导。FastTriggs 写成"一种只用一阶导的 IRLS 校正\cite{}"而非"PyPose 的 FastTriggs"。
- 避免"在 Handbook 中…"的叙述载体；BLO 的两个例子用本书口吻重述（学习特征+BA、学习前端+PGO）。

### 是否新建：**新子节**（`sec:nlopt-diffopt`，进阶/选读级，类比现有 `sec:nlopt-gvi` 的 `$\star$` 定位）。
### 工作量：**中**（隐式微分推导 + HVP + BLO 框架需自包含落地；与现有 GVI/协方差章承接需打磨）。

---

## 4. HB Ch06 — Certifiably Optimal Solvers and Theoretical Properties of SLAM

**作者**：Rosen, Khosoussi, Holmes, Dissanayake, Barfoot, Carlone。**性质**：两个根本问题——(1) 能否不靠初值、可证全局最优地解 SLAM（半定松弛/SE-Sync）？(2) 最优解离真值多近（CRLB + 图拉普拉斯）？**本组最大、最实在的新内容块。**

### 覆盖度：**部分有（约 20%：钩子已埋、CRLB 已有，但 SE-Sync 主体全空）**

### 现有落点（钩子 + 邻接）
- **已埋的"留作前沿"钩子**（多处显式承诺，现需兑现）：
  - `nonlinear_optimization.tex` §`sec:nlopt-why`：「目标非凸…这催生了**可证最优（certifiably optimal）**求解器（如位姿图的 SE-Sync，留作前沿）」
  - `nonlinear_optimization.tex` §`sec:nlopt-robust` 延伸：「可证最优位姿图（SE-Sync）——留作开放阅读」
  - `slam_state_estimation.tex` §`sec:est-fg`（`eq:fg-ls-early` 后）：「h_i 非线性使目标非凸…催生了好初值策略与'可证明最优'求解器（留 ch:nlopt）」
- **CRLB / Fisher 信息已有**：`slam_state_estimation.tex` §`sec:est-crlb`（`thm:crlb` CRLB + FIM）；`matrix_calculus.tex` §`sec:matderiv-fim`（FIM 重参数化 `I_φ=Jᵀ I_θ J`、CRLB 变换、流形上 CRLB 拉回李代数）——HB §6.2.1 的 CRLB/FIM 本书已有，**HB §6.2 的增量是"FIM↔图拉普拉斯"那一层**。
- **半定松弛的邻接**：`point_cloud_processing.tex` 已 `\cite` TEASER（截断最小二乘+半定松弛）——是"可证鲁棒配准"的同族，可作类比锚点。
- **Schur 补 / 旋转平均的邻接**：`nonlinear_optimization.tex` §`sec:nlopt-ba` 的 Schur 消元；`point_cloud_processing.tex` `thm:pc-svd`（Wahba/正交 Procrustes，旋转平均的单测量特例）。

### HB 增量清单（核心全新内容）
1. **Shor 松弛（HB §6.1.1，eq 6.1–6.6）**：QCQP → `xxᵀ=X`（秩1 PSD）→ 丢秩约束 → SDP；`d*≤p*` 给次优性界；若 SDP 解秩1 则恰为原问题全局最优。**这是整章的数学母机，必须自包含。**
2. **SE-Sync（HB §6.1.2，Problem 6.1–6.4 + Thm 6.1–6.2 + Alg 3）**：
   - PGO 的 MLE（各向同性 Langevin 旋转噪声 + 高斯平移噪声，eq 6.9）→ 化为只含旋转的 QCQP（解析消去平移 eq 6.16，连接拉普拉斯/incidence）。
   - Shor 松弛 → **精确性定理 Thm 6.1**（噪声足够小时松弛精确，可恢复全局最优）。
   - **Riemannian Staircase**（Burer-Monteiro 低秩分解 `Z=YᵀY` + Stiefel 流形优化 + Thm 6.2 秩亏二阶临界点=全局最优）。
   - **舍入**（thin SVD 投影回 SO(d)）+ **最优性证书**（eq 6.27 验证）。
   - 代数图论 Box 6.1（incidence A、Laplacian L=AᵀA、代数连通度/Fiedler 值）——本书需要这块作前置。
3. **扩展（HB §6.1.3–6.1.4）**：landmark-based（地标当纯平移变量、Schur 技巧 O(landmarks)）；range-aided（引入单位向量 bᵢⱼ 把非二次的距离项凑成 QCQP→CORA）；各向异性噪声/外点→**POP + Moment/Lasserre 松弛**（矩矩阵、冗余约束、松弛层级、退化 SDP 的求解难题）。
4. **FIM ↔ 图拉普拉斯（HB §6.2.2，eq 6.46–6.53）**：已知朝向的简化 PGO 里 `FIM = L_w ⊗ I₃`（加权拉普拉斯）；一般 PGO 的 FIM `eq:6.53`（含 `Ad(T⁻¹)`）；D/A/E-最优设计准则 = 拉普拉斯谱函数；Kirchhoff 矩阵树定理（D-最优 = 加权生成树数）；连接 active SLAM / 测量剪枝。"信息永不损害"原则、闭大环比小环更增信息。

### 融合策略
- **新增一节（章内大节），落点：`nonlinear_optimization.tex`**。这是兑现该章已埋的两处"留作前沿"钩子的最自然位置（SE-Sync 本就是"非凸最小二乘的全局求解器"，与 GN/LM 同章对照最连贯）。
  - 位置：放在 `sec:nlopt-gvi`（变分推断，进阶）**之后**或 `sec:nlopt-robust` 之后，新增 `sec:nlopt-certifiable`（可证最优：从局部到全局）。
  - 过渡句（直接缝合现有钩子）：「§`sec:nlopt-why` 说过 GN/LM 只保证局部最优、初值差会陷局部极小（\cref{fig:...} 停车场 PGO 的三个局部极小）。本节回答：能否**不靠初值、可证全局最优**地解 PGO？答案是半定松弛——把非凸 QCQP 松弛成凸 SDP，并在松弛精确时附带一张'最优性证书'。」——这是**把全书多处承诺兑现成一节**，强化连贯性。
- **结构建议**（自包含、由浅入深）：
  1. Shor 松弛（QCQP→SDP，秩1 lifting，次优性界）——通用，先讲透。
  2. 代数图论前置（incidence/Laplacian/Fiedler）——可压缩成一个 `\begin{insight}` 或小段，因 §6.2 也要用。
  3. SE-Sync 应用于 PGO：MLE→旋转 QCQP→SDP→精确性（Thm 6.1 陈述+直觉，证明引用）→Riemannian Staircase（Burer-Monteiro+Stiefel，Thm 6.2）→舍入+证书。
  4. 扩展（landmark/range/各向异性/外点→Moment 松弛）——**点到为止 + 关键式**，不必全展（POP/Lasserre 层级可作选读 `$\dagger$`）。
- **FIM↔拉普拉斯（§6.2）落点：`slam_state_estimation.tex` §`sec:est-crlb` 之后或同章新增 `sec:est-fim-laplacian`**。这与现有 CRLB/FIM 直接接续："§`sec:est-crlb` 给了 CRLB 的一般形式；SLAM 的 FIM 有额外结构——它由**问题的图（谁观测谁）**决定。" 推导 `FIM=L_w⊗I₃`（用现有 `matrix_calculus.tex` 的 Kronecker 积 `prop:matderiv-vec-kron`，**可复用本书已有工具**，自包含性强）。D/A/E-最优 + Kirchhoff + active SLAM 收尾。这是**深化现有 CRLB 论述**，把"信息矩阵"从代数对象升级为"图的拉普拉斯"。
  - 备选：FIM↔拉普拉斯也可与 SE-Sync 同节（都在 §6），但分置（求解器→`ch:nlopt`、精度极限→`ch:slam_est`）更贴合现有"slam_est 讲极限/normal eq、nlopt 讲求解"的分工。**建议分置**。
- **记号对齐**：HB 用 `SO(d)/SE(d)` 与本书一致；Stiefel/Burer-Monteiro/Langevin 是新记号需本书定义；伴随 `Ad(T)` 已在 `lie_theory.tex`（eq 6.53 可直接 cref）。

### 独立性风险
- **中-高风险**：易写成"SE-Sync 由 Rosen 等提出""CORA 是 Papalia 等的算法""[937] 证明…"。**对策**：方法用本书口吻陈述（"对 PGO 的旋转 QCQP 施 Shor 松弛，可得…"），具体算法/定理出处 `\cite`；Thm 6.1/6.2 的证明可"陈述 + 引用证明"（这是合法的，但**陈述要自包含、能看懂条件与结论**，不能 punt 成"证明见 [937]"而连定理内容都不写全）。
- 历史/谱系（§6.3 的一长串引用）放"延伸阅读"，正文不堆人名。

### 是否新建：**新大节 ×2**——`sec:nlopt-certifiable`（SE-Sync/Shor，在 `nonlinear_optimization.tex`）+ `sec:est-fim-laplacian`（FIM↔拉普拉斯，在 `slam_state_estimation.tex`）。
### 工作量：**大**（Shor 松弛 + SE-Sync 全链 + Riemannian Staircase + Moment 松弛 + FIM-拉普拉斯，全是自包含新推导；是本组最重的一章）。

---

## 5. HB Ch18 — The Computational Structure of Spatial AI Systems

**作者**：Andrew Davison。**性质**：愿景/前瞻章——SLAM→Spatial AI 的演化，计算结构（图处理器/近传感器处理/事件相机/GBP/持续学习）。**几乎无数学推导，是"系统观+前沿趋势"叙事。**

### 覆盖度：**完全没有（约 5%；仅 GBP 的因子图地基已有）**

### 现有落点（仅地基/邻接，主题本身全空）
- 因子图（`slam_state_estimation.tex` §`sec:est-fg`）= GBP 的载体，但 **GBP（高斯信念传播）本身全书未提**（grep 零命中）。
- `slam_system.tex` 的前后端/多线程架构 = HB §18.2 "实时闭环"的工程具象，可作类比锚点。
- **空间 AI / world model / 图处理器(IPU) / SCAMP 近传感器 / 事件相机计算结构 / 持续学习(GBP Learning) / 性能度量(bits×mm)**：全书空白。

### HB 增量清单（全新，但以"观点/结构"为主，非定理）
1. **SLAM→Spatial AI 的定义与演化**（§18.1）：稀疏/稠密/语义三级、world model = scene representation、闭环持续更新 vs 纯 VO/离线批处理。
2. **状态估计 vs 机器学习的融合谱系**（§18.3）：CodeSLAM/深度协方差/SuperPrimitives/DUSt3R/VGGT；为何混合法仍优（不确定度、增量融合）。
3. **硬件计算结构**（§18.4–18.5）：Dennard scaling 终结 → 并行/异构/专用；图处理器（Graphcore IPU）"存算一体、核间消息传递"；近传感器/在平面处理（SCAMP）；事件相机=只报变化→"模型预测事件相机"；把 Spatial AI 图映射到硬件（"Spatial AI brain" Fig 18.1）。
4. **GBP（高斯信念传播）作分布式收敛计算**（§18.6）：BP/GBP、loopy 图的收敛性、IPU 上 BA 比 CPU 快 30×、Robot Web 多机异步。**这是本章唯一接近"算法"的部分，且与本书因子图主线直接相关。**
5. **因子图内的持续学习**（§18.7）：Bayesian ML、GBP Learning（把 NN 结构嵌入因子图、变量=权重、非线性因子=神经元、训练=推断、无训练/测试之分）。
6. **性能度量**（§18.8）：超越定位精度的多目标度量（鲁棒/延迟/功耗/bits×mm）、Pareto 前沿、对 benchmark 的反思。

### 融合策略
- **核心难点（也是 slice 决策的关键）：HB Ch18 没有合适的现有章可挂。** 它既不是 P1 估计的推导、也不是 P2 SLAM 的系统实现，而是横跨"算法+硬件+学习+未来"的愿景。强行塞进 `slam_system.tex`（工程 capstone，文本:代码=60:40）会破坏其"精简 VO 实跑"的聚焦；塞进 `nonlinear_optimization.tex` 更不搭。
- **唯一可拆出来即时落地的硬核**：**GBP（§18.6）**。它是因子图上的**真算法**，与 `sec:est-fg`（因子图）+ `sec:est-elimination`（变量消元=精确推断）形成"精确 vs 近似"的天然对照。
  - 落点：`slam_state_estimation.tex` §`sec:est-elimination`/`sec:est-smooth-filter` 之后，可新增 `sec:est-gbp`（$\star$ 选读）："变量消元给出**精确** MAP，但需全局排序、本质串行；另一条路是**消息传递**——每个节点只与邻居通信，异步迭代收敛，天然分布式（GBP）。" 自包含核心：高斯因子图上的 belief/message 更新式、loopy 收敛性的一句话结论、与消元的对照。**这是深化现有"因子图推断"论述**，且填补全书 GBP 空白。
- **其余（§18.1–18.5, 18.7–18.8 的愿景/硬件/趋势）**：这些是**叙事性前瞻**，不适合塞进现有推导章。两个选项：
  - 选项 X（推荐，见 slice 建议）：聚成新"前沿专题"部分的一章/一节（如"空间 AI 与计算结构展望"），与 Ch4 可微优化、Ch6 可证最优的"前沿"气质聚类。
  - 选项 Y：分散为各章"延伸阅读/recent trends"的点缀（如 `slam_system.tex` 延伸加一段 Spatial AI 演化、`engineering_practice.tex`(P5) 若激活则讲硬件）。但分散会**削弱 Davison 这套"计算结构"叙事的整体性**，且 P5 现为 stub。
- **过渡/口吻**：愿景章最易写成 Davison 个人观点转述。融合时**升级为本书综述口吻**（"近年一种观点认为 SLAM 正演化为更广义的空间智能\cite{}…"），并与本书已有的"平滑 vs 滤波""学习 vs 几何"等张力呼应。

### 独立性风险
- **最高风险**：全章是 Davison 的 position paper，遍布"we believe""we hypothesize""we argue"。直接吸收极易变成"Davison 认为/Davison 预测…"（ventriloquize）或"FutureMapping 提出…"（narration_dependence）。**对策**：只提炼**可独立陈述的技术内核**（GBP 算法、IPU 存算一体原理、事件相机 bit-rate 论点、GBP Learning 结构），用本书综述口吻 + `\cite`；个人预测/愿景大幅压缩或转为"一种前瞻观点"。性能度量(bits×mm 等)可作客观清单收。

### 是否新建：
- **GBP 部分**：新子节 `sec:est-gbp`（$\star$ 选读，挂 `slam_state_estimation.tex`），可即时落地。
- **愿景/硬件/学习/度量部分**：**建议进新"前沿专题"部分**（见 slice 建议），不强挂现有章。
### 工作量：**中**（GBP 子节小-中；愿景部分若进新章则中等，主要是综述写作 + 独立性改写，数学少）。

---

## 6. Slice 级结构建议（本组：因子图/高级状态/可微优化/可证最优/空间AI计算结构）

### 判断：**混合策略——"大部分织进现有章 + 抽出一个新的'前沿专题'部分收纳无处可挂者"。**

理由分三层：

**(1) Ch1/Ch2 必须继续织进现有章（已成事实）。** 因子图、李群、连续时间已是 `slam_state_estimation.tex`/`lie_theory.tex` 的骨干，把它们的增量（Dogleg、消元例、样条、GP-on-Lie）抽出去会**割裂现有连贯叙事**、制造重复。这两章 100% 留在原位深化。

**(2) Ch4 可微优化、Ch6 可证最优应织进 `nonlinear_optimization.tex`（+ `slam_state_estimation.tex` 收 FIM-拉普拉斯）。** 关键论据：现有 `nonlinear_optimization.tex` **已经埋好了 SE-Sync 的"留作前沿"钩子**（§`sec:nlopt-why`、§`sec:nlopt-robust`），且已有 `sec:nlopt-gvi`（高斯变分推断）这一"优化的高阶视角"进阶节作为先例。可证最优（全局求解器）、可微优化（对解求导）都是"GN/LM 之上的高阶优化主题"，与 GVI 同章并列**最连贯**——读者读到的是"一个作者把非线性优化从基础讲到前沿"的单一论述，而非拼贴。这正中用户"融合而非斑斓颜料"的要求。

**(3) 只有 Ch18 的愿景/硬件/学习/度量部分真正"无处可挂"，需要新结构。** 它横跨算法+硬件+未来，不属任何现有 P1–P4 章的推导脉络；而 P5（工程实践）是 5 行 stub 且未进 book.tex。因此建议：

> **设立一个新的"前沿专题/展望"部分（暂名 P6 或并入激活后的 P5），收纳 Ch18 的 Spatial AI 计算结构展望（GBP 工程化、图处理器、近传感器处理、持续学习、性能度量）。** 这一部分天然可与本组另两块"前沿气质"内容呼应：虽然 Ch4/Ch6 主体织进 `ch:nlopt`，但若未来前沿内容增多（如本 slice 之外的深度学习 SLAM、开放世界 Spatial AI），此部分可成为统一的"前沿与展望"归宿。

**为何不把 Ch4/Ch6 也抽进新前沿部分？** 因为它们有**坚实的现有挂点和已埋钩子**，抽出去反而割裂"非线性优化"的完整论述、且与 GVI 重复邻接。可证最优/可微优化是"成熟的进阶方法"（有定理、有算法、可自包含推导），气质上属"深化的教材正文"；而 Ch18 是"前瞻愿景"（少数学、多观点），气质上属"展望"。**按气质分流**：进阶方法织进正文章，前瞻愿景聚成展望部分。

### 一句话 slice 结论
> Ch1/Ch2 留在 `slam_state_estimation`/`lie_theory`/`est-steam` 深化补缺；Ch4/Ch6 织进 `nonlinear_optimization`（兑现已埋的 SE-Sync 钩子，与 GVI 节并列）+ FIM-拉普拉斯进 `slam_state_estimation`；唯 Ch18 的 GBP 抽一子节进 `slam_state_estimation`、其愿景/硬件/学习/度量部分**新设一个"前沿展望"部分/章**收纳。整体上是"深化为主、单点新建"，不另起炉灶造前沿拼盘。

---

## 7. 工作量与优先级汇总

| HB 章 | 覆盖度 | 主落点 | 新建 | 工作量 | 优先级 |
|---|---|---|---|---|---|
| 01 因子图 | 已有 85–90% | `slam_state_estimation` §est-fg/elimination；`nonlinear_optimization` §lm | 否（精修）| 小 | 中（独立性清理优先）|
| 02 高级状态表示 | 已有 85% | `lie_theory`（不动）；`slam_state_estimation` §est-steam | 子节（样条）| 中 | 中 |
| 04 可微优化 | 几乎没有 10% | `nonlinear_optimization` §nlopt-diffopt（新）| 新子节 | 中 | 中-高 |
| 06 可证最优 | 部分 20% | `nonlinear_optimization` §certifiable（新）+ `slam_state_estimation` §fim-laplacian（新）| 新大节×2 | **大** | **高**（兑现已埋钩子）|
| 18 空间AI计算 | 没有 5% | GBP→`slam_state_estimation` §gbp（新）；愿景→**新前沿部分** | 子节 + 新部分 | 中 | 中 |

**共同独立性红线**（全组）：HB 这批前沿章作者鲜明（Carlone/Rosen/Davison/Wang），最易滑成 ventriloquize/narration。所有新内容一律本书口吻陈述方法 + `\cite` 出处；定理可"陈述+引用证明"但陈述须自包含；库/人名/谱系进延伸阅读。现有 §est-fg 开头"本节取自 Handbook 第1章…逐段记录"是已存在的 narration 隐患，应一并清理。
