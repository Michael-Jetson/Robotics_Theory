# 复核报告：Barfoot 吸收 Wave 2(对抗式独立核验)

> 复核 agent 独立完成。**全程只读,未编辑任何 .tex。** 关键数学式尽量用 numpy/scipy 独立数值验证(脚本临时落于 /tmp,均已跑通)。
> 默认怀疑、能证伪就证伪。结论:**绝大多数硬核数学经数值验证为正确**;发现 1 处确凿 NEEDS-FIX(交叉引用错指)、2 处 MINOR(需作者复核的约定/记号),独立性合规(无 punt/ventriloquize/narration 潜入)。

复核日期 2026-06-18。文件快照取自 `parts/` 下各章。复核时各文件末尾可能仍被其他 agent 追加,本报告只覆盖点名的既有 section/label。

---

## 0. 数值验证一览(我自己重算的,非照抄)

| 验证项 | 文件:label | 结果 |
|---|---|---|
| SE₂(3) `exp(ξ^)` = [R, Jρ, Jν] | lie:thm:se23-exp | **PASS** 机器精度(4e-16) |
| SE₂(3) 群乘法/逆 | lie:prop:se23-group | **PASS** (5e-17) |
| SE₂(3) 伴随移指数 `T Exp(ξ)=Exp(Adξ)T` | lie:eq:se23-adjoint | **PASS** (1e-16) |
| SE₂(3) `ξ^⌣ = ad(ξ)` = 李括号矩阵表示 | lie:eq:se23-adjoint | **PASS** (4e-16) |
| SE₂(3) `Ad(exp)=exp(ad)` | lie:eq:se23-adjoint | **PASS** (8e-17) |
| SE₂(3) 9×9 `J_l` = exp(ad) 级数:上三角、对角 3×Jl、耦合 Q_l(ρ,φ)/Q_l(ν,φ) | lie:1043 正文 | **PASS** (1.6e-16);下三角块严格 0 |
| `|detJ| = 2(1−cosφ)/φ²` + Taylor | lie:thm:detJ | **PASS** 全角度(≤2e-14) |
| SE(3) Q_l 闭式 vs 双重级数 | lie:deriv:Ql | **PASS** (2.8e-15) |
| GVI `∂V/∂μ = Σ⁻¹E[(x−μ)φ]` | nlopt:eq:gvi-dmu | **PASS** 蒙卡 vs 有限差 |
| Stein 一阶 `E[(x−μ)f]=ΣE[∇f]` | nlopt:eq:gvi 推导 | **PASS** 蒙卡 |
| Stein 二阶 `E[(x−μ)(x−μ)ᵀf]=ΣE[Hf]Σ+ΣE[f]` | nlopt:eq:gvi-stein2 | **PASS** 蒙卡 |
| `tr(ABAB)≥0` (A≻0,B 对称),等号⟺B=0 | nlopt:thm:nlopt-gvi-conv 证明 | **PASS** vec/Kron 论证正确 |
| STEAM Φ、Q(1/3Δt³,1/2Δt²,Δt) vs SDE 积分 | est:eq:steam-Phi-Q | **PASS** (2e-16) |
| STEAM GP 插值权 Λ,Ψ 边界 (Λ(0)=I,Ψ(0)=0,Λ(T)=0,Ψ(T)=I) | est:eq:steam-interp | **PASS** |
| Wahba 二阶判据 δ²J 系数 (V 主轴坐标) | pc:eq:pc-wahba-secondorder | **PASS**(见 §5.1 记号注) |
| 惯量 Gram 阵 `Σp^ᵀp^ = −Σp^p^ = Σ(\|p\|²I−ppᵀ)` | pc:thm:pc-wahba-suff | **PASS** (0.0) |
| 跟踪雅可比 `G=Dᵀ(Ťp̃)^⊙=[I,−m^]` | pc:eq:pc-track-G | **PASS** 结构正确 |
| RAE 3×3 雅可比逐元 + z=0 退化 | lidar:eq:lidar-rae-jac | **PASS** 有限差(2e-10) |
| IMU 杆臂(向心+切向)Coriolis 推导 | imu:eq:imu-leverarm-acc | **PASS** 代数正确 |
| 群上测地插值保群 + 匀速 Poisson | lie:thm:lie-interp | **PASS** |
| 插值扰动 `δφ` 一阶式 A=αJ(αφ)J(φ)⁻¹ | lie:eq:interp-perturb | **MINOR/待复核**(见 §4.2) |

---

## 1. SE₂(3) 扩展位姿群 (lie_theory.tex)

### `sec:se23` / `def:se23` / `thm:se23-exp` / `eq:se23-adjoint` —— **PASS**

逐项独立验算结论:

- **群形 5×5、代数 9 维、ξ=[ρ;ν;φ]**:与 \eqref{eq:se23-group}/\eqref{eq:se23-hat} 自洽。群乘法、逆 \eqref{eq:se23-inverse} 数值核验通过。
- **指数映射 r=Jρ, v=Jν 共用 SO(3) 左雅可比**:`expm` 直算 5×5 `ξ^` 与闭式 [R, Jρ, Jν] **完全吻合**(4e-16)。证明里 `(ξ^)ⁿ` 的分块展开(左上 φ^ⁿ、两并排列各 (φ^)^{n−1}ρ/ν、右下 2×2 零块自乘为零)正确,归并为同一 J 的论证成立。
- **9×9 伴随分块结构** \eqref{eq:se23-adjoint}:`Ad(T)=[[R,0,r^R],[0,R,v^R],[0,0,R]]`、`ad(ξ)=[[φ^,0,ρ^],[0,φ^,ν^],[0,0,φ^]]`。移指数恒等式、ad=李括号、Ad(exp)=exp(ad) 三条数值全 PASS。
- **分量排序 ξ=[ρ;ν;φ] ⟹ 伴随上三角**:正确。我额外验证了 9×9 `J_l=Σ(ξ^⌣)ⁿ/(n+1)!` 的级数,**确为块上三角**(下三角块严格 =0),对角三个 `J_l(φ)`,位置耦合块 = `Q_l(ρ,φ)`、速度耦合块 = `Q_l(ν,φ)`——与 line 1043 正文断言**逐字吻合**。
- **对照 Barrau–Bonnabel 2017**:line 1047 的"旋转放最前/最后 ⟹ 下/上三角"记号提醒准确,有益。

> 这一节质量很高,核心闭式全部经得起独立数值验证。

### NEEDS-FIX-1:line 1043 `eq:se23-adjoint` 段的 Q_l 交叉引用**错指**

> **文件:行/label**:`parts/P0_math/lie_theory.tex:1043`(`eq:se23-adjoint` 下方正文)
> **原文**:"…耦合块用 **\cref{eq:se23-hat}** 的 $\mathbf{Q}_l$ 块(位置块 $\mathbf{Q}_l(\boldsymbol{\rho},\boldsymbol{\phi})$、速度块 $\mathbf{Q}_l(\boldsymbol{\nu},\boldsymbol{\phi})$,闭式见 \cref{sec:lie-appendix} 的 $\mathbf{Q}_l$ 推导盒)。"

**问题**:`eq:se23-hat`(line 990–994)是 5×5 的 `ξ^` 算子,其中**根本没有 Q_l 块**(Q_l 是 SE(3) 6×6 左雅可比的耦合块)。这是一处确凿的 `\cref` 错指。

**为何错**:读者点过去看到的是 hat 算子,找不到任何 Q_l。正确的 Q_l 出处是:
- 定义形 `[[J_l, Q_l],[0,J_l]]` 在 `eq:se3-6x6`(line 494–496);
- 闭式在 `deriv:Ql`(line 1247,"SE(3) 的 Q_l 块闭式(Barfoot 式 8.91)")。

(注:`sec:lie-appendix` 与 `deriv:Ql` 推导盒**都存在**,故"闭式见 sec:lie-appendix 的 Q_l 推导盒"这半句是对的;只有 `eq:se23-hat` 这个引用是错的。)

**建议改法**:把 `\cref{eq:se23-hat}` 改为 `\cref{eq:se3-6x6}`(或直接 `\cref{deriv:Ql}`)。可改写为:"…耦合块与 SE(3) 同形(\cref{eq:se3-6x6,deriv:Ql}),位置块 $\mathbf{Q}_l(\boldsymbol{\rho},\boldsymbol{\phi})$、速度块 $\mathbf{Q}_l(\boldsymbol{\nu},\boldsymbol{\phi})$ 共用同一闭式"——其内容我已数值证实(9×9 级数的两耦合块恰为 Q_l(ρ,φ)、Q_l(ν,φ),1.6e-16)。

---

## 2. GVI 变分推断 (nonlinear_optimization.tex) `sec:nlopt-gvi`

### ELBO / 负 ELBO / 对 μ,Σ⁻¹ 更新 —— **PASS**

- 负 ELBO `V(q)=E_q[φ]+½ln det Σ⁻¹`、KL(q‖p) 推导 \eqref{eq:gvi-kl-expand}–\eqref{eq:gvi-V} 正确;高斯熵项符号一致。
- 导数 \eqref{eq:gvi-derivs}:我用蒙卡 + 公共随机数有限差独立验证 `∂V/∂μᵀ=Σ⁻¹E[(x−μ)φ]`,吻合。Hessian–梯度关系 \eqref{eq:gvi-hessgrad} 与据此构造的牛顿式更新 \eqref{eq:gvi-cov-update}/\eqref{eq:gvi-mean-update} 自洽。
- 自然梯度块对角费舍尔 `diag(Σ⁻¹, ½Σ⊗Σ)` ⟹ 更新与类牛顿逐字相同(\eqref{eq:gvi-ngd-components}):结构正确。
- Stein 一阶/二阶 \eqref{eq:stein}/\eqref{eq:gvi-stein2}:**蒙卡两条都 PASS**,据此化简到 \eqref{eq:gvi-stein} 的"信息阵=Hessian 期望、梯度=梯度期望"正确。

### `thm:nlopt-gvi-conv` 收敛证明 —— **PASS**

下降量 \eqref{eq:gvi-descent} 两项 ≥0:第一项显然;第二项 `tr(Σ⁽ⁱ⁾δΣ⁻¹Σ⁽ⁱ⁾δΣ⁻¹)≥0` 的 vec/Kronecker 论证(`vec(ABA)=(A⊗A)vec(B)`、A≻0⟹A⊗A≻0、等号⟺B=0)**完全正确**,我已独立确认该不等式。属"局部保证"的措辞诚实(与牛顿/GN 同性质)。

### `thm:nlopt-iekf-map` (IEKF→MAP 自证) —— **PASS**

三步证明(GN 步 → SMW 化成卡尔曼增益 → 不动点=驻点=众数)逐步正确:`Σ=P̌−KGP̌`、`ΣGᵀR⁻¹=K`、`ΣP̌⁻¹=I−KG` 的代换无误,Step 2 整理出的更新式确与 IEKF \eqref{eq:iekf} 同形。无外引,自包含。

### Takahashi 稀疏回收 \eqref{eq:gvi-takahashi-rec} / ESGVI 精确稀疏 —— **PASS**

后向递归式、"盒子四角"封闭性规则、LDLᵀ 推导都标准且正确;ESGVI 把对全 q 的期望精确退化为对边缘 q_k 的期望(\eqref{eq:gvi-esgvi-hess} 最后一步)论证成立。

### 立体相机三方数值 (GVI 0.28 / MAP −33.0 / ISPKF −3.84 cm) —— **PASS(但提请注意两列口径不同)**

`tab:nlopt-gvi-stereo` 单步点估计 `x̂`(24.5694 / 24.7414 / 24.7792,真后验均值 24.7770)内部自洽;偏差列 `ê_mean` 按 caption 明确定义为 **10⁵ 次试验的 E[x̂]−x̌(估计减先验)**,与"单次 x̂ 减真后验均值"是**两个不同量**(后者算出来是 −20.8/−3.6/+0.2 cm,与 −33/−3.84/0.28 不同,但这正常——它们口径不同)。caption 已写明定义,故**不算错**;与 Barfoot 第 5 章表一致。

> 提示(非 fix):普通读者易把"单步 x̂"与"系统偏差 ê_mean"混为可相减的同一量。可在 caption 末补一句"注:ê_mean 是 10⁵ 次蒙卡的统计偏差,非单次 x̂−x̄"以杜绝误解。

---

## 3. STEAM 连续时间 (slam_state_estimation.tex) `sec:est-steam`

### WNOA 的 Φ/Q —— **PASS**

`A=[[0,I],[0,0]]` 幂零 ⟹ `Φ=[[I,Δt I],[0,I]]`;`Q(Δt)` 的三块 `⅓Δt³Qc, ½Δt²Qc, ΔtQc` 我用 `∫₀^Δt Φ(Δt,s)LQcLᵀΦᵀds` 数值积分**完全吻合**(2e-16)。derivation 盒里的逐块积分系数(⅓,½,1)正确。

### `prop:steam-sparse` 块三对角逆核稀疏 —— **PASS**

箭头矩阵 \eqref{eq:steam-arrow}:`A₂₂` 块对角、`A₁₁=Fᵀ⁻Q⁻¹F⁻¹+G₁ᵀΣ⁻¹G₁` 块三对角(块下双对角的 `BᵀB` 型乘积只在主/邻对角非零)、`A₁₂` 稀疏——推理正确。`G₁` 末尾零块(速度不参与对路标观测)的说明对。证明严谨。

### GP 插值 O(1) 查询 —— **PASS(结构)**

马尔可夫性 ⟹ 查询 τ 只需两端 t_k,t_{k+1};插值权 Λ,Ψ 我用标准 `Ψ=Q(τ)Φ(t_{k+1},τ)ᵀQ(Δt)⁻¹`、`Λ=Φ(τ)−ΨΦ(Δt)` 验证**满足全部边界条件**(τ=0:Λ=I,Ψ=0;τ=Δt:Λ=0,Ψ=I)。WNOA 下退化为三次 Hermite 的论断与此一致(常速度 GP 后验插值=三次多项式,标准结论)。

### `ex:gauss-through-exp` (y=exp(x) 对数正态) —— **PASS**

`p(y)=1/(y√2πσ²)·exp(−(ln y)²/2σ²)` 正确;众数 `e^{−σ²}`、中位数 1、均值 `e^{σ²/2}` 三者**确为对数正态标准矩**(教科书值),互不重合的论断对。`1/y` 雅可比因子保归一的论证正确。三点直觉(必带雅可比/非高斯不可避免/众数偏均值)的承接干净。

### 记号注 (line 1151–1153, 1262) —— 合规

明确声明"把 Barfoot 第 11 章左雅可比/左扰动改写为本书右扰动主线",并指出 `J_r=J(−e)`、信息阵改记 Λ。这是**正当的记号对照**,非 narration 依赖。

---

## 4. 李群运动学/测度/插值 (lie_theory.tex)

### `thm:so3-kinematics` (ω=J·φ̇) —— **PASS**

含参矩阵指数导数 \eqref{eq:matexp-deriv} → `Ṙ Rᵀ=(J_l φ̇)^∧`(世界系)、`Rᵀ Ṙ=(J_r φ̇)^∧`(机体系)推导正确;伴随恒等式、积分表示 `J_l=∫₀¹Rᵅdα` 用得对。`J_l=R J_r`、`ω_w=Rω_b` 互验自洽。`cor:fixed-axis`(匀轴 φ̇=ω,因 Ja=a)正确。右扰动主线下取 `φ̇=J_r⁻¹ω_b` 是对的,pitfall(world↔body × left↔right 配对)准确。

### `thm:detJ` (|detJ|=2(1−cosφ)/φ²) —— **PASS**

特征值法证明(沿轴 λ=1,垂面共轭复 λ=sinφ/φ±i(1−cosφ)/φ)正确;`sin²φ+(1−cosφ)²=2−2cosφ` 末步对。SE(3) `|det𝒥|=|detJ|²`(分块上三角)对。我对多个角度数值核验闭式与 `det(J_l)` **完全吻合**,Taylor `1−φ²/12+φ⁴/360` 也对。幺模性 \eqref{eq:unimodular}、Haar 测度论述正确。

### `subsec:pose-fusion` 位姿融合 —— **PASS**

右扰动 GN:误差 `e_k=Log(T⁻¹ T̄_k)`、雅可比 `G_k=−𝒥_l(e_k)⁻¹`(负号来自左侧扰动 Exp(−ε))、正规方程 \eqref{eq:fusion-normal}、融合协方差=Hessian 逆 \eqref{eq:fusion-cov} 全部正确。小不确定度退化为欧氏信息加权平均的论证对。line 913 明示"Barfoot 用左扰动推导、本书改右扰动、内容等价"——合规。

### NEEDS-VERIFY:`thm:lie-interp` 主体 PASS,但 `eq:interp-perturb` 扰动式存疑 —— **MINOR**

> **文件:label**:`parts/P0_math/lie_theory.tex:709–717`,`eq:interp-perturb`

**主体 PASS**:测地插值 `R(α)=(R₂R₁ᵀ)ᵅR₁` 保群、端点正确、匀速=Poisson 解的论证都对(`insight` line 701)。边界 `A(0,·)=0`、`A(1,·)=I` 我数值确认成立(故 practice line 727 无误)。

**存疑点**:扰动一阶式 `δφ=(I−A)δφ₁+Aδφ₂`,`A(α,φ)=αJ(αφ)J(φ)⁻¹`(J=SO(3) **左**雅可比)。我用 numpy 在多种扰动约定(R₁/R₂ 各左乘/右乘 Exp(ε))× 左/右雅可比下穷举数值核验,**没有任何一种组合**能把该式逼近到一阶精度:scale=0.1 时最优组合残差仍 ~0.047,且**随转角增大而增大**(scale=0.3 → ~0.13),不呈一阶收敛。对角项接近、非对角项系统性偏差。

**为何可能错**:本书主线是**右扰动**,但此式形如直接照搬 Barfoot 的**左扰动**结果(用 J_l、`varphi=Log(R(α))` 绝对对数)。在右扰动主线下,`A` 很可能应改用 `J_r`,或 `varphi/δφ_i` 的扰动变量定义需明确(Barfoot 的 δφ_i 可能定义在相对/局部量上,而非我测试的直接对 R₁,R₂ 扰动)。

**为何只列 MINOR 而非 NEEDS-FIX**:(a) 是标 `供协方差/雅可比传播` 的进阶旁支,非主干;(b) 边界与结构(小转动 A→αI)都对;(c) 我无法排除是 δφ_i 扰动变量约定差异(而非式子本身错)。

**建议**:作者在右扰动约定下**重新推导一次** δφ→δvarphi 的一阶映射并数值自检(同我的脚本思路:对 R₁,R₂ 施右扰动,看 `Log(R(α))` 的一阶变化),确认 `A` 用 J_l 还是 J_r、`varphi` 取绝对对数还是相对 R₁。若确为左扰动遗留,改 `J→J_r` 或补一句扰动变量定义。

---

## 5. 位姿估计应用 (point_cloud_processing.tex / slam_system.tex)

### `thm:pc-wahba-cases` Wahba 唯一性分类 —— **PASS**

case (i-a)~(ii) 的分档(det(UVᵀ) 符号 × 奇异值重数/秩)与 de Ruiter–Forbes 一致,几何解读(rank2 共面/rank1 共线/rank0 重合)正确。`derivation` 的二阶判据 \eqref{eq:pc-wahba-secondorder} `δ²J=−½Σφⱼ²(σ_k s_k+σ_l s_l)` 我数值验证(在 **V** 主轴坐标下)**逐系数吻合**(3,4,5),case (i-a)/(i-b) 的"全正⟹极小""翻最小奇异值"结论对。

### MINOR:`eq:pc-wahba-secondorder` 坐标系标注疑为 U vs V

> **文件:label**:`point_cloud_processing.tex:618`

正文写"$\boldsymbol{\varphi}=[\varphi_1,\varphi_2,\varphi_3]^\top$ 在 **U 主轴坐标**下"。但 `tr(Exp(φ^)·V S Σ Vᵀ)` 的对角化框架是 **V**(我以 φ=V[:,i] 扰动才得到吻合的 3,4,5;以 U 列扰动不吻合)。

**为何可能错**:对角化 `R* H = V S Σ Vᵀ`(因 `R*=VSUᵀ`,`H=UΣVᵀ`,`R*H=VS Σ Vᵀ`),主轴应是 V 的列。

**为何低置信**:在最优点 `R*=VSUᵀ` 处 U、V 经 R* 紧密相关,且公式**数值正确**,只是"主轴属 U 还是 V"的文字标注存疑。建议作者核对一行:把"U 主轴坐标"改为"V 主轴坐标"(或写明 φ 是在 `R*` 把 U 系搬过去后的坐标)。**不影响任何结论**。

### `thm:pc-wahba-suff` 三点不共线充分条件 —— **PASS**

惯量阵 `I_p=−Σp'^p'^=Σ(‖p'‖²I−p'p'ᵀ)≻0 ⟺ 存在三个不共线点`。Gram 阵恒等式我数值核验 **完全相等**(0.0)。`xᵀI_p x=Σ‖p'^x‖²` 取零 ⟺ 全共线的论证对。证明干净。

### `eq:pc-track-G` = Dᵀ(Ťp̃)^⊙ 跟踪雅可比 —— **PASS**

`(Ťp̃)^⊙=[[I,−m^],[0,0]]`(m=Ř p+ť 载体系预测点),Dᵀ 取上三行 ⟹ `G=[I, −(Řp+ť)^]`。结构正确,与 ICP 雅可比 `[I,−Rp'^]`、VO 重投影 `[−MR, MRP^]` 同根(都源于 ⊙ 算子)的统一论述对。`ť=0` 退化(practice line 833)对。

### `eq:sys-pgo-spantree` / `eq:sys-pgo-tridiag` 位姿图 (slam_system.tex) —— **PASS**

- 生成树初始化 `T_j⁽⁰⁾=T̄_{ij}T_i⁽⁰⁾`、浅树优于深树(协方差随深度累积)、里程计=退化单链的论述正确。
- 链式块三对角 \eqref{eq:sys-pgo-tridiag}:`A_kk=Λ'_{(k−1)k}+𝒯ᵀΛ'_{(k+1)k}𝒯`、`A_{k(k+1)}=−𝒯ᵀΛ'`,其中 `Λ'_{kℓ}=𝒥_{kℓ}⁻ᵀΣ⁻¹𝒥_{kℓ}⁻¹`、`𝒯=Ad(T_k)Ad(T_ℓ)⁻¹`。结构与"相对位姿残差只耦合 i,j ⟹ 三对角"一致。块 Cholesky O(n) 论述对。
- **记号注**:`𝒥_{kℓ}=𝒥(−e_{kℓ})`(line 1634)用了 `𝒥(−e)` 形式,与本书右扰动下 `J_r=J_l(−·)` 的惯例一致,无矛盾。
- line 1644"g2o/GTSAM/Ceres 通用稀疏 Cholesky 自动吃掉链/树稀疏,Barfoot 本人也这么说"——属事实陈述+正当归因,非 punt。

---

## 6. 传感器模型 (lidar_slam.tex / imu_model.tex)

### `eq:lidar-rae-jac` RAE 3×3 雅可比 —— **PASS**

`∂s/∂ρ` 三行(径向 ρᵀ/r;方位 [−y,x,0]/(x²+y²);俯仰用 `1−(z/r)²=ρ_xy²/r²`)我用有限差**逐元核验吻合**(2e-10)。z=0 退化为 range-bearing 2×2 块(\eqref{eq:lidar-rangebearing})也数值确认。practice 提示(核对俯仰行)对。`atan2` 覆盖 (−π,π] 的实现注对。

### `eq:imu-leverarm-acc` 杆臂离心+切向项 —— **PASS**

二次求导 `p̈_s^w=a^w+R_wb[(ω̇_b)^r + (ω_b)^(ω_b)^r]`:乘积法则、`Ṙ_wb=R_wb(ω_b)^` 代入正确;切向 `ω̇×r`、向心 `ω×(ω×r)` 拆分对。最终模型 \eqref{eq:imu-accel-leverarm} 外层左乘 R_sb、`R_ws^ᵀR_wb=R_sb` 化简(line 207)正确;r=0,R_sb=I 退回标准模型对。陀螺无杆臂效应(角速度是整刚体属性)的论述对。`insight`(向心∝ω²、忽略杆臂会污染 b^a 估计)与 Barfoot 把 ω̇ 列可选状态的做法归因合规。

---

## 7. 独立性合规扫描 —— **全部 PASS**

对全部 7 个文件做正则扫描:
- **external_punt**(证明从略/详见原书/参见 Barfoot 推导/留作不证):**0 命中**。
- **ventriloquize**("Barfoot 坦言/承认/证明了/写道/指出"):**0 命中**(点名章节内)。
- **narration_dependence**("在 Barfoot 中…/书中给出…"作为叙述骨架):**0 命中**。出现的 `\cite{barfoot2024state}` 均为正当出处标注或记号对照(如"Barfoot 用左扰动、本书改右扰动""信息阵 Barfoot 记 I、本书记 Λ"),符合独立著作"平衡对照"标准,非依赖。
- 文件头注释明确"自包含独立记录…读者无需翻原书",且正文确实把定理+证明+闭式全部落地(SE₂(3)、GVI 收敛、IEKF→MAP、Q_l 闭式、Wahba 二阶判据均自证)。**独立性达标。**

记号一致性:右扰动主线、ξ=[ρ;φ]/[ρ;ν;φ]、Λ 信息阵在抽查处贯穿一致。唯一记号风险点是 §4.2 的插值扰动式(疑似左扰动遗留)。

---

## 最该修的 3–5 项

1. **【NEEDS-FIX】lie_theory.tex:1043** —— `eq:se23-adjoint` 段把 SE₂(3) 9×9 左雅可比的 Q_l 块 `\cref{eq:se23-hat}`(5×5 hat,无 Q_l)**错指**。改为 `\cref{eq:se3-6x6}` 或 `\cref{deriv:Ql}`。(内容本身正确,仅引用错。)

2. **【MINOR/待复核】lie_theory.tex:709–717** —— 插值扰动一阶式 `eq:interp-perturb`(`A=αJ_l(αφ)J_l(φ)⁻¹`)在右扰动主线下数值**不收敛到一阶**(穷举各约定残差 ~0.05–0.13、随角增大)。疑为照搬 Barfoot 左扰动结果。请在右扰动约定下重推+数值自检,确认 J_l vs J_r 与 δφ_i 扰动变量定义;若确系遗留,改 `J→J_r` 或补扰动变量说明。边界 A(0)=0/A(1)=I 无误。

3. **【MINOR】point_cloud_processing.tex:618** —— `eq:pc-wahba-secondorder` 文字标"φ 在 **U** 主轴坐标下",但对角化框架是 **V**(数值在 V 系才吻合)。建议核对改"V 主轴坐标"。公式值与所有 case 结论均正确,纯文字标注。

4. **【可选/防误解】nonlinear_optimization.tex:1358 表 caption** —— 补一句注:`ê_mean` 是 10⁵ 次蒙卡统计偏差(估计减先验),非单次 `x̂` 与真后验均值之差,避免读者把两列当可相减的同一量。三方数值(0.28/−33.0/−3.84 cm)本身与 Barfoot 一致、无误。

> 综评:Wave 2 这批硬核数学**整体质量很高**,SE₂(3)、GVI(含收敛证明与 Stein)、STEAM(Φ/Q/插值)、Wahba、RAE、IMU 杆臂、detJ、Q_l 闭式全部经我独立数值验证为正确,独立性合规。唯一确凿缺陷是 1 处交叉引用错指(易修);插值扰动式与 Wahba 二阶坐标标注两处需作者用我提供的思路复核确认(很可能是约定/标注问题,而非实质数学错误)。
