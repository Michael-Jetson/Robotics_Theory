# Barfoot《SER》2nd ed 吸收审计 —— 第 10 章（连续时间估计）+ 附录 A/B/C/D

> 只读审计，未改任何 .tex。对照源：`Barfoot_SER2_md/{10_Continuous-Time_Estimation, A_Appendix_A_Matrix_Primer, B_Appendix_B_Rotation_and_Pose_Extras, C_Appendix_C_Miscellaneous_Extras, D_Appendix_D_Solutions_to_Exercises}`。
> 教材树：`/root/gpf/Robotics_Theory/Robotics_Note/parts/`。判定按内容匹配，不因左/右扰动、C↔R、布局差异报缺。
> 注：ch1–5 已在 `docs/Barfoot吸收审计.md` 另审；本报告只管 ch10 + 附录。其中 Stein 引理(C.2)、Fisher/CRLB(C.1 的基本版)、不变 EKF(C.4) 等与 ch1–5 审计有边界重叠，本报告只就「附录额外内容」判定，不重复 ch1–5 结论。

---

## ① 总体结论

| Barfoot 单元 | 教材吸收度 | 落点 | 是否达标准 A |
|---|---|---|---|
| **ch10 连续时间估计**（GP 运动先验 / WNOA / SE(3) 局部 SDE / STEAM / 插值外推） | **~15%，点到为止** | `slam_state_estimation.tex` §867（1 段「前沿一瞥」）+ `nonlinear_optimization.tex` §1411（1 个 `derivation` 环境，向量空间） | ✗ **远未达标** |
| **附录 A 矩阵基元** | 矩阵*微积分*达标；线性代数*基元*散落、部分缺 | `appendix/matrix_calculus.tex`（求导）；Cholesky/SVD/Schur/特征值散在各估计与视觉章 | △ 主线够用，作为「附录 A 对照」有缺口 |
| **附录 B 旋转/位姿补充** | 约 40% | `lie_theory.tex` / `rigid_body_motion.tex` | △ 两条重要恒等式 + Jordan 分解 + 最小多项式缺失 |
| **附录 C 杂项** | 约 50% | 散在 `imu_model.tex` / `slam_state_estimation.tex` / `kalman_eskf.tex` / `nonlinear_optimization.tex` | △ 离散化的 Van Loan、FIM 多参数化、不变 EKF 几何版缺失 |
| **附录 D 习题解答** | 仅覆盖 ch2–8 题 | —— | ✓ 无额外义务（D 不含 ch10/附录题） |

**一句话**：附录 A/B/C 的「主线必需」部分基本吸收，缺的是「专题深化」恒等式；**真正的大缺口是 ch10——整章连续时间 SE(3) 轨迹估计（STEAM）在教材里只有两段「指路」文字，没有任何一节自包含展开**。这与项目「完全吸收、独立重写」的目标差距最大。已知的 §867 与 nonlinear_optimization 附录推导，经逐字核对，**确属「点到为止 + 向量空间一段 derivation」，不满足标准 A**，需补成完整一节（见 ④）。

---

## ② 逐节 gap 清单

### A. ch10 连续时间估计（核心缺口区）

教材现状（已逐字核对）：
- `slam_state_estimation.tex:867`「前沿一瞥：连续时间与高斯过程回归」——**1 段**：把估计看作 GP 回归、WNOA「常速度」$\ddot{\mathbf p}=\mathbf w$、离散化得块三对角、$O(1)$ 插值（点明常速度下为三次 Hermite）、点名 STEAM——随即「完整展开属专题，此处给出去向」。
- `nonlinear_optimization.tex:1411`「$\star$ 连续时间批量估计（高斯过程回归）」——**1 个 derivation 环境**：连续 SDE $\dot{\mathbf x}=f(\cdot)$ 线性化为 LTV、lifted 先验 $\mathcal N(\mathbf F\boldsymbol\nu,\mathbf F\mathbf Q'\mathbf F^\top)$、与离散批量同形的 GN、转移函数 $\boldsymbol\Phi(t,s)=\boldsymbol\Upsilon(t)\boldsymbol\Upsilon(s)^{-1}$ 由归一化基本阵积分。**全程在向量空间 $\mathbb R^n$，无 SE(3)**。
- `slam_state_estimation.tex:961` 表格一行「STEAM（连续时间）」；`imu_model.tex:1034`「进阶」节提到连续加速度/旋转预积分用 GP（squared-exp 核）、常 jerk 模型——但那是预积分语境，非连续时间轨迹估计框架。

| # | 缺什么（标准 A 视角的不可替代内容） | Barfoot 出处 | 为何重要 | 应落教材何处 |
|---|---|---|---|---|
| A-1 | **SE(3) 上的连续时间运动先验**：用一串*局部* GP 变量 $\boldsymbol\xi_k(t)\in\mathfrak{se}(3)$（$\mathbf T(t)=\exp(\boldsymbol\xi_k(t)^\wedge)\mathbf T(t_k)$）缝合全局位姿，把非线性 SDE $\dot{\mathbf T}=\boldsymbol\varpi^\wedge\mathbf T,\ \dot{\boldsymbol\varpi}=\mathbf w'$ 化为局部 LTI 可闭式随机积分 | §11.1.1–11.1.2，式(11.1)–(11.7) | 这是把向量空间 GP 回归真正搬上李群的关键技巧；教材两段都停在 $\mathbb R^n$，**完全没有 $\boldsymbol\xi_k$ 局部变量这一步**，正是 §867「专题」被略去的核心 | `slam_state_estimation.tex` 连续时间新一节，或 `nonlinear_optimization.tex` 附录扩成正节 |
| A-2 | **WNOA 误差项与代价（SE(3) 形式）**：二元因子误差(11.13) 含 $\ln(\mathbf T_k\mathbf T_{k-1}^{-1})^\vee$、$\mathcal J^{-1}\boldsymbol\varpi$；线性化得 $\mathbf F_{k-1},\mathbf E_k$ 块(11.20)(11.21)；堆叠成 $\mathbf F^{-1}$ 块下双对角(11.23) | §11.1.3–11.1.4 | 这是「白噪声加速度先验」在李群上的*可计算*落地；教材只写标量 $\ddot{\mathbf p}=\mathbf w$，没有 SE(3) 因子与雅可比 | 同 A-1 |
| A-3 | **STEAM 问题与稀疏结构**：状态含 $\{\mathbf T_k,\boldsymbol\varpi_k,\mathbf p_j\}$；测量块 $\mathbf G_{1,jk}=[\mathbf S(\cdot)^\odot\ \mathbf 0]$（速度列补零）；$\mathbf A=\mathbf H^\top\mathbf W^{-1}\mathbf H$ 仍是箭头 + $\mathbf A_{11}$ 块三对角，可 Schur/Cholesky | §11.2.1–11.2.4，式(11.25)–(11.38) | 「Simultaneous Trajectory Estimation And Mapping」的完整 setup。教材只在变体表里列了一行 STEAM 名字，**无问题构造、无测量模型、无稀疏性证明** | `slam_state_estimation.tex` 因子图节之后，作 SLAM 的连续时间变体专节 |
| A-4 | **GP 插值查询任意时刻**：$t_k\le\tau<t_{k+1}$ 时只用两端，均值(11.43)–(11.45)（$\boldsymbol\Lambda(\tau),\boldsymbol\Psi(\tau)$ 由 $\boldsymbol\Phi,\mathbf Q$ 组成）+ **协方差插值**(11.49)–(11.50)（用三时刻 $36\times36$ 信息阵边缘化）| §11.3.1–11.3.2 | 连续时间方法相对离散的*核心卖点*。教材点了「$O(1)$ 插值/三次 Hermite」一句话，**但没有 $\boldsymbol\Lambda/\boldsymbol\Psi$ 公式、更没有协方差插值** | 同 A-3，插值子节 |
| A-5 | **外推（预测未来态）**：$\tau>t_K$ 时 $\hat{\mathbf T}(\tau)=\exp((\tau-t_K)\hat{\boldsymbol\varpi}^\wedge)\hat{\mathbf T}_K$，速度恒定；协方差(11.52)–(11.55) 经 SMW 化为类 EKF 预测 $\hat{\mathbf P}(\tau,\tau)=\mathbf E^{-1}(\mathbf F\hat{\mathbf P}\mathbf F^\top+\mathbf Q)\mathbf E^{-\top}$ | §11.3.3 | 与滤波预测打通的漂亮收尾；教材完全没有 | 同 A-3，外推子节 |
| A-6 | **讨论中的延伸点**：① 测量时刻/估计时刻/查询时刻*三套时间戳解耦*（可减少主解状态数）；② WNOJ（白噪声加加速度，Tang 2019）、Singer 模型（Wong 2020）作为高阶 SDE；③ 连续体机器人（以弧长代时间，Cosserat ≈ WNOA） | §11.4 | 这些是把 ch10 与教材已有的 LQR/规划/连续体话题串起来的「视野」；WNOJ/Singer 教材完全未提名 | 连续时间节末「延伸/视野」段 + 习题 |
| A-7 | **Takahashi/选择性稀疏协方差回收在连续时间下的应用** | §11.2.4 末引 Yan 2014（sparse-GP 融入 iSAM）；协方差插值(11.48) 需「按箭头结构高效抽取边缘」| 连续时间下查询协方差离不开*只回收所需块*的稀疏逆 | 教材有「回代 $\boldsymbol\Lambda^{-1}$ 对应块、勿整体求逆」(`nonlinear_optimization.tex:544,1052`)的*雏形*，但**未点名 Takahashi/selected inverse、未与连续时间协方差插值挂钩**。建议在协方差恢复节补一句 selected-inverse 命名 + 连续时间引用 | `nonlinear_optimization.tex` §nlopt-cov |

> 判定：A-1～A-5 是 ch10 的不可替代骨架，**当前教材几乎全缺**，仅有的两段是「向量空间预览 + 指路」。这是本次审计发现的**唯一 major 级缺口**，与「完全吸收」目标冲突最大。

### B. 附录 B 旋转与位姿补充

| # | 缺什么 | Barfoot 出处 | 教材现状 / 落点 | 重要性 |
|---|---|---|---|---|
| B-1 | **$\mathrm{Ad}(SE(3))$ 作用的导数** $\partial(\boldsymbol{\mathcal T}(\boldsymbol\xi)\mathbf x)/\partial\boldsymbol\xi=-(\boldsymbol{\mathcal T}\mathbf x)^\odot\boldsymbol{\mathcal J}(\boldsymbol\xi)$ | §B.1.1 | ✓ **已吸收**（同类 $\odot$ 算子与 $\partial(\mathbf T\tilde{\mathbf p})/\partial\delta\boldsymbol\xi=(\mathbf T\tilde{\mathbf p})^\odot$ 在 `lie_theory.tex:460-479` 有完整推导） | 低（已覆盖等价内容） |
| B-2 | **SO(3) 雅可比运动学恒等式** $\dot{\mathbf J}(\boldsymbol\phi)-\boldsymbol\omega^\wedge\mathbf J(\boldsymbol\phi)\equiv\partial\boldsymbol\omega/\partial\boldsymbol\phi$（$\boldsymbol\omega=\mathbf J\dot{\boldsymbol\phi}$）| §B.2.1，式(B.5) | ✗ **缺失**（全树无此恒等式） | **中–高**：连续时间/滤波里把「左雅可比时间导数」与角速度联系起来的基本工具，A-1/A-2 推导会用到 |
| B-3 | **SE(3) 雅可比运动学恒等式** $\dot{\boldsymbol{\mathcal J}}(\boldsymbol\xi)-\boldsymbol\varpi^\wedge\boldsymbol{\mathcal J}(\boldsymbol\xi)\equiv\partial\boldsymbol\varpi/\partial\boldsymbol\xi$（$\boldsymbol\varpi=\boldsymbol{\mathcal J}\dot{\boldsymbol\xi}$）| §B.2.2，式(B.10) | ✗ **缺失** | **中–高**：同上，且正是 ch10 式(11.12) $\boldsymbol\psi_k=\mathcal J(\boldsymbol\xi_k)^{-1}\boldsymbol\varpi$ 的运动学根基 |
| B-4 | **SO(3) 特征分解** → 由 $\boldsymbol\phi^\wedge=\mathbf V\mathbf D\mathbf V^H$（$\mathbf D=\mathrm{diag}(i\theta,-i\theta,0)$）导出 Euler/Rodrigues $\mathbf C=\cos\theta\,\mathbf 1+(1-\cos\theta)\mathbf a\mathbf a^\top+\sin\theta\,\mathbf a^\wedge$ | §B.3.1 | ✓ **已吸收**（`lie_theory.tex:305-307` 用迹/特征向量恢复转角转轴；`rigid_body_motion.tex` Rodrigues 多证） | 低 |
| B-5 | **SE(3) Jordan 分解** + **$\mathrm{Ad}(SE(3))$ Jordan 分解**：位姿一般*不可对角化*（$\lambda=0$ 代数重数 2、几何重数 1），需广义特征向量，得 Jordan 块 | §B.3.2–B.3.3 | ✗ **缺失**（仅 `rigid_body_motion.tex` 总览表出现「Jordan」字样，无展开） | 中：是「为什么 SE(3) 的指数/对数要用左雅可比而非简单对角化」的深层解释；属专题深化 |
| B-6 | **so(3)/se(3) 最小多项式**：$\boldsymbol\phi^{\wedge3}+\phi^2\boldsymbol\phi^\wedge=\mathbf 0$（即 $(\boldsymbol\phi^\wedge)^3=-\phi^2\boldsymbol\phi^\wedge$）、$\boldsymbol\xi^{\wedge4}+\phi^2\boldsymbol\xi^{\wedge2}=\mathbf 0$、$\mathbf C^3-(\mathrm{tr}\,\mathbf C)\mathbf C^2+(\mathrm{tr}\,\mathbf C)\mathbf C-\mathbf 1=\mathbf 0$ | §B.3.1/B.3.2 末 | △ **部分**：`slam_state_estimation.tex:737` 只把 Cayley–Hamilton 用于*可观测性*；**so(3) 的 $(\boldsymbol\phi^\wedge)^3=-\phi^2\boldsymbol\phi^\wedge$ 这一具体恒等式未单列**（它是 Rodrigues 级数闭合、指数/对数有限项的代数根据） | 中：教材 Rodrigues 推导其实隐含用到，但未把恒等式提炼为可复用工具 |

### C. 附录 C 杂项

| # | 缺什么 | Barfoot 出处 | 教材现状 / 落点 | 重要性 |
|---|---|---|---|---|
| C-1 | **多元高斯 FIM 的多种参数化**：canonical(均值+vec协方差)、对称化(vech+复制矩阵 $\mathbf D$)、hybrid(逆协方差)、natural(信息形式) 各自的 $\mathcal I_\theta$ 与 $\mathcal I_\theta^{-1}$（块 $\tfrac12\boldsymbol\Sigma^{-1}\otimes\boldsymbol\Sigma^{-1}$ 等） | §C.1.1–C.1.6 | △ 教材有 **基本 CRLB + Fisher 信息**（`slam_state_estimation.tex:506-517`），但**没有「多参数化 FIM」「复制矩阵 $\mathbf D$ 处理对称」「natural 参数 FIM 非块对角」**这些 Magnus–Neudecker 细节 | 中：与「自然梯度/变分推断」专题挂钩；主线只需基本 CRLB，故优先级中 |
| C-2 | **Stein 引理**（$E[(\mathbf z-\boldsymbol\mu)\mathbf f(\mathbf z)^\top]=\boldsymbol\Sigma\,E[\partial\mathbf f/\partial\mathbf z]$，及二次/分块版） | §C.2 | ✓ **已吸收**（`slam_state_estimation.tex:458-464` 完整陈述，用于 sigma-point/UKF；与 ch1–5 审计重叠） | 低 |
| C-3 | **连续→离散运动模型的系统离散化**：LTV/LTI SDE 通解、$\boldsymbol\Phi(t,s)$ 三性质、离散过程噪声 $\mathbf Q_k=\int\boldsymbol\Phi\mathbf L\mathbf Q\mathbf L^\top\boldsymbol\Phi^\top ds$、**Van Loan 矩阵指数技巧**同时算 $\mathbf A_{k-1},\mathbf B_k,\mathbf Q_k$（式 C.67a/b）| §C.3 全节 | △ **部分**：白噪声/随机游走的 $\sigma_d=\sigma/\sqrt{\Delta t}$ 标量版在 `imu_model.tex:323-345` 有；$\boldsymbol\Phi$ 由 $\dot{\boldsymbol\Phi}=\mathbf F\boldsymbol\Phi$ 积分在 `nonlinear_optimization.tex:1412`、`vio.tex` 有。**但通用 $\mathbf Q_k$ 积分式、尤其 Van Loan 的「分块矩阵指数一次算齐 $\mathbf A,\mathbf B,\mathbf Q$」技巧全树缺** | **中–高**：这是 ch10/IMU/EKF 共用的离散化引擎；Van Loan 技巧实用且 Barfoot 单列，值得补 |
| C-4 | **不变 EKF（IEKF, Barrau–Bonnabel）**：左不变误差动态使 $\mathbf F_{k-1}',\mathbf G_k'$ 不依赖状态估计、左不变创新 $\check{\mathbf T}_k^{-1}(\mathbf y_k-\check{\mathbf y}_k)$ | §C.4 | △ **名称冲突的缺失**：教材的「IEKF」(`kalman_eskf.tex`) 指*迭代* EKF（趋 MAP 众数），**不是*不变* EKF**；Barrau–Bonnabel 的群论不变性、左不变创新、$\mathbf F'=\mathbf 1$ 全树未提（`imu_model.tex:1052` 仅以 $\mathrm{SE}_2(3)$ 一句带过并注明「属不变滤波专题，本章未展开」） | 中：教材右扰动主线下，左不变结构天然次要；属专题，但「IEKF 一词双义」易混，建议至少加一条术语澄清 |

### D. 附录 D 习题解答

- D 只含 **ch2（概率）、ch3（线性高斯）、ch4（非线性非高斯）、ch5（非理想）、ch7（三维几何）、ch8（矩阵李群）** 的习题解，**不含 ch10 与任何附录的题**。
- 故对本次审计范围（ch10+附录）而言，**D 不构成额外的「习题覆盖」义务**。教材是否覆盖 ch2–8 方法的习题，属 ch1–5 审计范畴，此处不重复。

---

## ③ 独立性问题（external_punt / ventriloquize / narration_dependence）

| 位置 | 类型 | 说明 | 处置建议 |
|---|---|---|---|
| `slam_state_estimation.tex:868`「完整展开属专题，**此处给出去向**」 | **external_punt（轻度）** | 用「指路」替代展开。在「前沿一瞥」框里作为视野是*可接受*的；但若把它当作 ch10 的全部吸收，则是把一整章 punt 给读者/原书 | 不算违规*前提是*另起一节真正展开（见 ④）；否则即构成对 ch10 的整章 punt |
| `nonlinear_optimization.tex:1412` derivation 全程 `\cite{barfoot2024state}`，且收在「转移函数…数值积分得到」 | narration_dependence（轻度） | 内容本身是本书口吻、独立推导，引用合规；但**只引 Barfoot 单书**，未与十四讲/Handbook 的连续时间视角对照 | 补一处 Handbook（Furgale 综述/连续时间 SLAM）平衡引用即可 |
| `matrix_calculus.tex:419-420`「本节仅作索引，不展开；需要时可查 Magnus–Neudecker 与 Matrix Cookbook」 | **external_punt（合理）** | 矩阵对矩阵求导明确 punt 给外部书。鉴于本书已论证「主线用迹技巧/李群扰动回避」，**这是合理的范围裁剪，非违规** | 保留；这是 deliberate scope，不需补 |
| `imu_model.tex:1034+` 进阶节连续/GP 预积分**全段** `\cite{carlone2026handbook}` | narration_dependence（中度） | 连续加速度/旋转预积分、$\mathrm{SE}_2(3)$、学习方法等几乎逐句挂在 Handbook 上，且多处「属…专题，本章未展开」 | 这是「视野综述」段，可接受；但密集单书依赖偏多，宜挑 1–2 个点（如常 jerk = 梯形法则、GP 预积分核）做独立小推导以降依赖 |

> 总体：本范围内**无严重 ventriloquize**；主要独立性风险是「连续时间相关内容普遍以*指路/综述*形式存在、且偏单书引用」。一旦补全 ch10 正文（④），这些 punt 自然降级为正常的延伸引用。

---

## ④ 最该补的 3–5 项（按 贡献 × 工作量 排序）

1. **【最高优先 · 大工程】补「连续时间轨迹估计（SE(3) GP / STEAM）」完整一节**（覆盖 A-1～A-5）。
   - 内容：局部变量 $\boldsymbol\xi_k(t)$ 缝合 → 局部 LTI SDE 闭式积分($\boldsymbol\Phi,\mathbf Q$) → WNOA 二元因子(11.13) 与线性化块 $\mathbf F_{k-1},\mathbf E_k$ → STEAM 问题 setup + 箭头/块三对角稀疏 → GP 均值插值($\boldsymbol\Lambda,\boldsymbol\Psi$) + 协方差插值/外推。
   - 落点：`slam_state_estimation.tex` 因子图节后新增 §「连续时间与 STEAM」，把 `nonlinear_optimization.tex:1411` 的向量空间 derivation 升级为 SE(3) 正文；§867 的「前沿一瞥」改为指向该节。
   - 贡献：**填掉本次审计唯一的 major 缺口**，直接兑现「完全吸收 ch10」。工作量大（需新写 ~15 式 + 2 图），但价值最高。

2. **【高性价比 · 小工程】补两条李群运动学恒等式 B-2/B-3 + so(3) 最小多项式 B-6**。
   - 内容：$\dot{\mathbf J}-\boldsymbol\omega^\wedge\mathbf J=\partial\boldsymbol\omega/\partial\boldsymbol\phi$、SE(3) 对应版、$(\boldsymbol\phi^\wedge)^3=-\phi^2\boldsymbol\phi^\wedge$。各一个 `derivation`/`proposition`。
   - 落点：`lie_theory.tex`（雅可比/指数节）。
   - 贡献：高——它们是 ① 推导的*前置工具*，先补好 ② 再写 ① 顺理成章；工作量小（每条 5–8 行）。

3. **【中高性价比 · 小工程】补「连续→离散」的通用 $\mathbf Q_k$ 积分 + Van Loan 分块矩阵指数技巧（C-3）**。
   - 内容：$\mathbf Q_k=\int\boldsymbol\Phi\mathbf L\mathbf Q\mathbf L^\top\boldsymbol\Phi^\top ds$；式(C.67a/b) 用 $\exp\big(\big[\begin{smallmatrix}\mathbf A&\mathbf L\mathbf Q\mathbf L^\top\\\mathbf 0&-\mathbf A^\top\end{smallmatrix}\big]\Delta t\big)$ 一次算齐。
   - 落点：`imu_model.tex` §连续↔离散噪声 之后，或 `kalman_eskf.tex` 预测节附录。
   - 贡献：中高——一个公用工具同时服务 IMU/EKF/ch10；Barfoot 单列说明其实用性。工作量小（~10 行 + 一段级数证明可选）。

4. **【中等 · 极小工程】术语澄清「IEKF 双义」（C-4）**。
   - 在 `kalman_eskf.tex` IEKF 处加 1 句脚注/note：本书 IEKF=*迭代* EKF（趋众数）；Barrau–Bonnabel 的*不变* EKF（左不变误差/创新、$\mathbf F'=\mathbf 1$）是另一回事，见 $\mathrm{SE}_2(3)$ 不变滤波专题。
   - 贡献：消除读者混淆；工作量极小。完整展开不变 EKF 可留作专题（优先级低于上三项）。

5. **【可选 · 低优先】FIM 多参数化（C-1）与 SE(3) Jordan 分解（B-5）**作为「选读/延伸」补充。
   - 二者均属专题深化（变分推断 / 李群结构解释），主线非必需。若追求「附录全量对照」可补，否则以一处引用 + 一句话点明即可，不必展开。

---

### 附：判定证据索引（便于复核）
- ch10 教材全部落点：`slam_state_estimation.tex:867-868, 961`；`nonlinear_optimization.tex:1411-1413`；`imu_model.tex:1034-1050`。全树 `STEAM` 仅 3 处、`Takahashi` 0 处、`WNOJ/Singer/白噪声加加速度` 0 处。
- 附录 A：`matrix_calculus.tex`（求导，含迹技巧/vec/Kronecker 索引式 §381-420）；Cholesky/SVD/Schur/特征值/行列式散落于 P1/P2 各章（grep 命中文件见正文）；`vech/复制矩阵/duplication` 全树 0 处。
- 附录 B：B-1 `lie_theory.tex:460-479` ✓；B-4 `lie_theory.tex:305-307`+`rigid_body_motion.tex` ✓；B-2/B-3/B-5 全树缺；B-6 仅 `slam_state_estimation.tex:737`（可观测性语境）。
- 附录 C：C-2 `slam_state_estimation.tex:458-464` ✓；C-1 基本版 `:506-517`（多参数化缺）；C-3 `imu_model.tex:323-345`+`nonlinear_optimization.tex:1412`（Van Loan 缺）；C-4 `kalman_eskf.tex`（=迭代EKF）+`imu_model.tex:1052`（不变滤波明确未展开）。
- 附录 D：仅 ch2/3/4/5/7/8 习题解（`D_Appendix_D…:5,147,249,333,361,451`），无 ch10/附录题。
