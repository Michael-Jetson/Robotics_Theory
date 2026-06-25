# Barfoot《State Estimation for Robotics》2nd ed. 第 7 章「矩阵李群」吸收审计

> 范围：Barfoot ch7（内部式号 8.x，全 3334 行）+ Appendix B 李群部分（Lie Group Tools / Kinematics / Decompositions）。
> 对照：`parts/P0_math/lie_theory.tex`（主，898 行）、`parts/P0_math/rigid_body_motion.tex`（次），并 grep 全 `parts/` 树。
> 记号对照已校准：教材主线 = **右扰动 $\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\phi})$、$\boldsymbol{\xi}=[\boldsymbol{\rho};\boldsymbol{\phi}]$、$\mathbf{R}$ 记旋转、Hamilton 四元数**；Barfoot = 左扰动为主、$\mathbf{C}$ 记旋转、$\mathbf{T}$ 记位姿、$\varpi$ 广义速度、$\curlywedge/\barwedge$ 记号。凡"教材给右扰动版而 Barfoot 给左扰动版"一律判为合法吸收，不算缺。
> 审计日期 2026-06-18。ch1–5 已另审（`docs/Barfoot吸收审计.md`），本报告只管 ch7 + 相关 Appendix B。

---

## ① 总体结论

**吸收度估计：李群章（ch7 §8.1 几何 + §8.3 概率）核心约 80%；但 ch7 的两大"招牌硬核块"——§8.1.6 距离/体积/积分、§8.3.6 位姿融合、Appendix B 运动学雅可比恒等式与 Jordan/eigen 分解（最小多项式来源）——基本缺失或仅一句带过。综合（含 §8.2 运动学、§8.4 对称性）约 65–70%。**

`lie_theory.tex` 是一篇**质量很高、独立性优秀**的李群教程：群/李代数定义、$\mathfrak{so}(3)/\mathfrak{se}(3)$、Rodrigues、$\mathrm{SE}(3)$ 指数与左雅可比 $\mathbf{J}$、BCH 线性近似与 $\mathbf{J}_l/\mathbf{J}_r$ 闭式、$6\times6$ 雅可比 $\boldsymbol{\mathcal{J}}_l$ 与 $\mathbf{Q}_l$ 块、伴随 $\mathrm{Ad}/\mathrm{ad}$ 与"移指数"恒等式、右扰动求导、流形/$\boxplus\boxminus$、切空间高斯 + 一阶/四阶位姿复合协方差 + 香蕉分布、$\mathrm{Sim}(3)$——**这些都已全量、自包含地吸收**，且 cite 平衡（Barfoot/十四讲/Solà/Handbook/Eade/Chirikjian/Hertzberg），无 punt/ventriloquize。Barfoot §8.3.7（非线性相机模型不确定度一/二/四阶 + sigmapoint + 立体相机数值实验）已**完整落在** `parts/P2_slam/camera_model.tex`（§"不确定度传播"，1059–1076 行 + 延伸阅读指 §8.3.7）。Barfoot §8.2.1 旋转运动学的 Poisson 方程 $\dot{\mathbf{R}}=\boldsymbol{\omega}^\wedge\mathbf{R}$、§8.2.4 位姿运动学 $\dot{\mathbf{T}}=\varpi^\wedge\mathbf{T}$ 与广义速度 $\varpi=[\boldsymbol{\nu};\boldsymbol{\omega}]$ 已落在 `rigid_body_motion.tex` §"旋转与位姿运动学"（710–765 行）。

**主要落点一览：**
| Barfoot ch7 主题 | 教材落点 | 状态 |
|---|---|---|
| §8.1.1–8.1.2 群/李代数/$\mathfrak{so}(3),\mathfrak{se}(3)$ | lie_theory §群/§李代数 | ✅ 全量 |
| §8.1.3 指数/对数（Rodrigues、$\mathrm{SE}(3)$ 指数、$\mathbf{J}$、直接级数式 8.44） | lie_theory thm:so3-exp / thm:se3-exp | ✅（缺 8.44 的"$\mathbf{T}$ 直接级数式"，见②） |
| §8.1.4 伴随 $\mathrm{Ad}/\mathrm{ad}$、$\mathcal{T}$ 闭式、$\mathrm{Ad}=\exp(\mathrm{ad})$、$\mathcal{T}$ 直接级数 8.68 | lie_theory sec:adjoint | ✅ 主体（缺 8.68 级数式） |
| §8.1.5 BCH、$\mathbf{J}_l/\mathbf{J}_r$ 级数+闭式、$\boldsymbol{\mathcal{J}}_l/\mathbf{Q}_l$、Bernoulli/$\int\mathbf{C}^\alpha$ | lie_theory sec:bch + 附录 Q_l | ✅ 强 |
| §8.1.6 距离/内积/体积元 $\lvert\det\mathbf{J}\rvert$/李群积分 | — | ❌ **缺**（见②-A） |
| §8.1.7 插值（SO(3)/SE(3)）+ 扰动插值 + Faulhaber | — | ❌ **缺**（见②-B） |
| §8.1.8 齐次点 $\odot/\circledcirc$ 算子与恒等式 | lie_theory eq:right-pert-se3（仅 $\odot$ 的 $4\times6$） | ◑ 部分（缺 $6\times4$ 的 $\circledcirc$、$(\mathbf{Tp})^\odot$ 恒等式） |
| §8.1.9 微积分与优化（GN on SO(3)/SE(3)） | lie_theory sec:perturb + sec:manifold；nonlinear_optimization | ✅ 等价吸收 |
| §8.1.10 黎曼流形优化（Riemannian grad/retraction） | lie_theory sec:manifold | ◑ 概念有，缺黎曼梯度闭式推导（见③非缺口） |
| §8.2.1–8.2.2 SO(3) 运动学 $\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$、数值积分 | rigid_body §运动学（仅 Poisson）；vio/imu（$\dot{\mathbf{r}}=\mathbf{J}_r^{-1}\boldsymbol{\omega}$ 一句） | ◑ 部分（见②-C） |
| §8.2.3–8.2.4 线性化（扰动）运动学 + LTV 状态转移矩阵 | vio/imu（误差态 LTV，右扰动版） | ◑ 见③ |
| §8.3.1 李群高斯/PDF 严格定义（mean 方程、$\det\mathbf{J}$ 诱导密度、左/中/右三选项） | lie_theory sec:uncertainty | ◑ 有切空间高斯，缺 PDF 严格定义（见②-D） |
| §8.3.2 旋转矢量不确定度（$E[\mathbf{y}]$ 四阶、Isserlis） | camera_model（同款四阶 + Isserlis） | ✅ 等价落点 |
| §8.3.3 位姿复合（二阶/四阶/sigmapoint/banana） | lie_theory sec:uncertainty + 附录四阶算子 | ✅ 主体（偏薄，见③） |
| §8.3.4 位姿求逆不确定度（精确 $\bar{\mathcal{T}}^{-1}\Sigma\bar{\mathcal{T}}^{-T}$） | — | ◑ 仅 insight 一句"位姿求逆获干净闭式"，无式 |
| §8.3.5 相关位姿复合/差分（$\Sigma_{12}$ 交叉项） | — | ❌ **缺**（见②-E） |
| §8.3.6 位姿融合（$K$ 估计 GN 融合、$\mathbf{G}_k=\boldsymbol{\mathcal{J}}^{-1}$） | — | ❌ **缺**（见②-E，重要） |
| §8.3.7 非线性相机模型不确定度 | camera_model.tex | ✅ 全量（在相机章） |
| §8.4 对称/不变/等变、不变误差、InEKF/EqF | vio（不变 KF 一句）、lidar（不变性 vs 觉察性）、camera_calib | ◑ 散点提及，无 ch7 式 8.406–8.416（见②-F） |
| App B.1 $\mathrm{Ad}(SE(3))$ 导数 | — | ❌ 缺（小，见③） |
| App B.2 运动学雅可比恒等式 $\dot{\mathbf{J}}-\boldsymbol{\omega}^\wedge\mathbf{J}=\partial\boldsymbol{\omega}/\partial\boldsymbol{\phi}$ | — | ❌ **缺**（见②-C） |
| App B.3 eigen/Jordan 分解 + **三个最小多项式** | — | ❌ **缺**（关键，见②-G） |

---

## ② 逐节 gap 清单（缺什么 / Barfoot 式号 / 为何重要 / 应落何处）

### A.【缺，中-高】李群上的距离、体积元与积分 —— Barfoot §8.1.6（式 8.106–8.125）
- **缺什么**：(1) $\mathfrak{so}(3)$ 内积 $\langle\boldsymbol{\phi}_1^\wedge,\boldsymbol{\phi}_2^\wedge\rangle=\tfrac12\mathrm{tr}(\boldsymbol{\phi}_1^\wedge\boldsymbol{\phi}_2^{\wedge T})=\boldsymbol{\phi}_1^T\boldsymbol{\phi}_2$（8.107）与 $\mathfrak{se}(3)$ 加权内积（8.116）；(2) 两旋转/位姿的**左/右距离度量** $\phi_{12}=\ln(\mathbf{C}_1^T\mathbf{C}_2)^\vee$、$\xi_{12}=\ln(\mathbf{T}_1^{-1}\mathbf{T}_2)^\vee$（8.106/8.115/8.108/8.117）；(3) **无穷小体积元** $d\mathbf{C}=\lvert\det\mathbf{J}\rvert\,d\boldsymbol{\phi}$，$d\mathbf{T}=\lvert\det\boldsymbol{\mathcal{J}}\rvert\,d\boldsymbol{\xi}$，及关键结论 $\det\mathbf{J}_l=\det\mathbf{J}_r$（unimodular），$\lvert\det\mathbf{J}\rvert=2(1-\cos\phi)/\phi^2$（8.110–8.113、8.121–8.124）；(4) 李群上积分 $\int_{SO(3)}f\,d\mathbf{C}\to\int_{\lvert\phi\rvert<\pi}f\lvert\det\mathbf{J}\rvert d\boldsymbol{\phi}$（8.114/8.125）。
- **为何重要**：$\lvert\det\mathbf{J}\rvert$ 是 §8.3.1 把切空间高斯密度 $p(\boldsymbol{\epsilon})$ 诱导到 $p(\mathbf{C})$ 的**雅可比因子**——没有它，李群上的 PDF 归一化、期望与"为何均值≠众数"都讲不严格。这是 Barfoot 概率框架的数学地基，且全 grep 在 `parts/` 树**完全无落点**。
- **应落何处**：`lie_theory.tex` 现 sec:uncertainty 之前新增一小节"李群上的距离、体积与积分"，或并入 sec:uncertainty 开头（密度归一化必需）。难度中，篇幅约 1 页。

### B.【缺，中】矩阵李群上的插值 —— Barfoot §8.1.7（式 8.126–8.162）
- **缺什么**：(1) 闭式保群插值 $\mathbf{C}=(\mathbf{C}_2\mathbf{C}_1^T)^\alpha\mathbf{C}_1$、$\mathbf{T}=(\mathbf{T}_2\mathbf{T}_1^{-1})^\alpha\mathbf{T}_1$（8.128/8.155），及"为何线性插值 $(1-\alpha)\mathbf{C}_1+\alpha\mathbf{C}_2\notin SO(3)$"（8.127）；(2) 其**匀角速度物理诠释**（8.134–8.137，与 Poisson 方程的精确联系）；(3) 扰动插值 $\delta\boldsymbol{\varphi}=(\mathbf{1}-\mathbf{A})\delta\boldsymbol{\phi}_1+\mathbf{A}\delta\boldsymbol{\phi}_2$、$\mathbf{A}(\alpha,\boldsymbol{\phi})=\alpha\mathbf{J}(\alpha\boldsymbol{\phi})\mathbf{J}(\boldsymbol{\phi})^{-1}$ 与 Faulhaber 系数级数（8.143–8.162）。
- **为何重要**：SE(3)/SO(3) 插值是连续时间 SLAM、轨迹平滑、关键帧间位姿生成、IMU/相机时间对齐的基础。教材现状：`lidar_slam`/`point_cloud_processing` 用的是 LOAM "匀速**线性**插值"（一阶近似），与 Barfoot 的**保群指数插值**不是一回事；`slam_state_estimation` 连续时间 GP 一节也只字未提群上插值闭式。属真缺口。
- **应落何处**：`lie_theory.tex` 在 sec:manifold 后新增"李群上的插值"小节（可标 $*$ 选读），与 §8.2 运动学呼应。难度中。

### C.【缺，中-高】运动学的李代数形式 $\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$ + 雅可比恒等式 —— Barfoot §8.2.1（8.236）& App B.2（B.5/B.10）
- **缺什么**：(1) 关键结论 $\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$（左雅可比把李代数速率映成角速度）及其逆 $\dot{\boldsymbol{\phi}}=\mathbf{J}^{-1}\boldsymbol{\omega}$（8.236/8.237），$\mathrm{SE}(3)$ 版 $\varpi=\boldsymbol{\mathcal{J}}\dot{\boldsymbol{\xi}}$；(2) **运动学雅可比恒等式** $\dot{\mathbf{J}}(\boldsymbol{\phi})-\boldsymbol{\omega}^\wedge\mathbf{J}(\boldsymbol{\phi})\equiv\partial\boldsymbol{\omega}/\partial\boldsymbol{\phi}$（B.5）及 SE(3) 版 $\dot{\boldsymbol{\mathcal{J}}}-\varpi^\barwedge\boldsymbol{\mathcal{J}}\equiv\partial\varpi/\partial\boldsymbol{\xi}$（B.10）。
- **为何重要**：$\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$ 把"$\mathbf{J}$"从指数映射里的代数对象升格为**有物理意义的运动学映射**——它正是连续旋转预积分 $\dot{\mathbf{r}}=\mathbf{J}_r^{-1}\boldsymbol{\omega}$（`vio.tex` 1144 行、`imu_model.tex` 1042 行已**孤立地引用**该式却未在李群章建立！）的来源。B.5/B.10 恒等式则是连续时间状态估计、IMU 预积分协方差推导的工具。教材现状：`rigid_body_motion` 只给了 Poisson 方程 $\dot{\mathbf{R}}=\boldsymbol{\omega}^\wedge\mathbf{R}$，没有李代数侧 $\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$，导致 vio/imu 引用 $\mathbf{J}_r^{-1}\boldsymbol{\omega}$ 时**前置定理悬空**。
- **应落何处**：$\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$ 应进 `lie_theory.tex`（紧接 thm:se3-exp 或 sec:bch 之后，作为"$\mathbf{J}$ 的运动学含义"insight，右扰动版给 $\boldsymbol{\omega}=\mathbf{J}_r\dot{\boldsymbol{\phi}}$ 或对应式），让 vio/imu 有处可引；B.5/B.10 可放 sec:lie-appendix 推导附录（标进阶）。难度中。

### D.【部分薄，中】李群高斯/PDF 的严格定义 —— Barfoot §8.3.1（8.312–8.331）
- **缺什么**：教材 sec:uncertainty 直接写了 $\mathbf{T}=\bar{\mathbf{T}}\,\mathrm{Exp}(\boldsymbol{\epsilon})$、$\boldsymbol{\epsilon}\sim\mathcal{N}(0,\Sigma)$，但**缺**：(1) 左/中/右三种扰动定义的对照表与"为何选左（Barfoot）/右（本书）、为何不选 middle（要进李代数会碰奇异）"的讨论（8.319 表）；(2) **诱导密度** $p(\mathbf{C})$ 含 $1/\lvert\det\mathbf{J}\rvert$ 因子的推导（8.319，依赖 ②-A）；(3) 均值的**隐式定义方程** $\int\ln(\mathbf{C}\mathbf{M}^T)^\vee p\,d\mathbf{C}=0$ 与协方差定义（8.320/8.323/8.327/8.329）；(4) 确定性变换下分布精确变换 $\mathbf{C}'=\mathbf{R}\mathbf{C}\Rightarrow\boldsymbol{\epsilon}'=\mathbf{R}\boldsymbol{\epsilon}\sim\mathcal{N}(0,\mathbf{R}\Sigma\mathbf{R}^T)$（8.324/8.330，无近似）。
- **为何重要**：这是 Barfoot 不确定度框架的**定义层**——"切空间高斯"为什么是合理的均值/协方差定义、它诱导的群上密度长什么样，全靠这套。教材现状只有"操作层"（怎么传播），缺"定义层"（为什么这么定义合法）。
- **应落何处**：扩充 `lie_theory.tex` sec:uncertainty 的"切空间高斯"段，补三选项对照 + 均值隐式方程 + 确定变换精确传播式。难度中。

### E.【缺，高】位姿融合 + 相关位姿复合/差分 —— Barfoot §8.3.5（8.363–8.368）& §8.3.6（8.369–8.377）
- **缺什么**：(1) **位姿融合**（§8.3.6）：把 $K$ 个带协方差的位姿估计 $\{\bar{\mathbf{T}}_k,\Sigma_k\}$ 用李群上 Gauss–Newton **迭代融合**为单一 $\{\bar{\mathbf{T}},\Sigma\}$；误差 $\mathbf{e}_k=\ln(\bar{\mathbf{T}}_k\mathbf{T}^{-1})^\vee$、雅可比 $\mathbf{G}_k=\boldsymbol{\mathcal{J}}(-\mathbf{e}_k)^{-1}$、正规方程 $(\sum\mathbf{G}_k^T\Sigma_k^{-1}\mathbf{G}_k)\boldsymbol{\epsilon}^\star=\sum\mathbf{G}_k^T\Sigma_k^{-1}\mathbf{e}_k$、$\Sigma=(\sum\mathbf{G}_k^T\Sigma_k^{-1}\mathbf{G}_k)^{-1}$（8.371–8.377）。(2) **相关位姿**的复合 $\Sigma\approx\Sigma_1+\bar{\mathcal{T}}_1\Sigma_2\bar{\mathcal{T}}_1^T+\Sigma_{12}\bar{\mathcal{T}}_1^T+\bar{\mathcal{T}}_1\Sigma_{12}^T$ 与差分 $\mathbf{T}_1\mathbf{T}_2^{-1}$ 的协方差（8.366/8.368），以及 §8.3.4 位姿求逆的精确协方差 $\bar{\mathcal{T}}^{-1}\Sigma\bar{\mathcal{T}}^{-T}$（8.362）。
- **为何重要**：位姿融合是**多传感器位姿对齐、位姿图节点合并、relative-pose 因子构造**的核心；相关位姿复合/差分是位姿图边、回环约束协方差**正确性**的关键（教材 loop_closure/PGO 现按"独立"处理，遇相关会低估/高估协方差）。这是 Barfoot 把"李群优化 + 不确定度"合流的标志性应用，全 grep 树内**完全无落点**（`dense_mapping` 的"高斯融合"是标量深度融合，不是 SE(3) 融合）。属高价值真缺口。
- **应落何处**：`lie_theory.tex` sec:uncertainty 末新增"位姿融合（李群上的 Gauss–Newton）"+"位姿求逆与相关位姿复合/差分"两小节；也可放 `slam_state_estimation.tex` 或 `loop_closure.tex`。难度中-高（融合需结合 sec:perturb 的右扰动 GN）。

### F.【部分散，中】对称性、不变性、等变性、不变误差 —— Barfoot §8.4（8.406–8.416）
- **缺什么**：ch7 §8.4 的系统论述：(1) 不变性 $f(\mathbf{Tp})=f(\mathbf{p})$ 与"$SE(3)$ 是 $f$ 的对称"（8.406–8.410，以定位代价 $J^\star$ 为例）；(2) 等变性 $\mathbf{T}^\star(\mathbf{T}_{12}\mathbf{p})=\mathbf{T}^\star(\mathbf{p})\mathbf{T}_{12}$、左/右等变定义（8.411–8.414）；(3) **不变误差** $\mathbf{e}_v=\ln(\mathbf{T}_{vi}\tilde{\mathbf{T}}_{vi}^{-1})^\vee$（动系/静系不变误差），及 InEKF（Barrau–Bonnabel）/EqF（Mahony–Trumpf）的动机（8.415/8.416）。
- **为何重要**：等变/不变是现代李群滤波（InEKF、EqF）与"为何状态估计问题不依赖参考系选择"的理论根；也是 $\mathrm{SE}_2(3)$ 不变滤波（vio/imu 已多次提及却"未展开"）的入口。教材现状：`vio`（"(右)不变卡尔曼滤波"一句、Martinelli 连续对称性一句）、`lidar`（"不变性 vs 觉察性"是 place-recognition 语境）、`camera_calib`（IAC 不变性）——都是**散点提及**，无 ch7 的对称性正式定义与不变误差。
- **应落何处**：`lie_theory.tex` 末新增 $*$ 选读小节"对称、不变与等变（通往 InEKF/EqF）"，或落 `kalman_eskf.tex`（InEKF 自然归宿）。难度中。注：这属"完整性"而非"独立性必需"，优先级低于 A/C/E/G。

### G.【缺，高（理论硬核）】eigen/Jordan 分解与三个最小多项式 —— Barfoot App B.3（B.15–B.76）
- **缺什么**：(1) $\mathfrak{so}(3)$ 的 eigen 分解 $\boldsymbol{\phi}^\wedge=\mathbf{V}\mathbf{D}\mathbf{V}^H$、$\mathbf{C}=\mathbf{V}\exp(\mathbf{D})\mathbf{V}^H$ 及由此重得 Rodrigues/$\mathrm{tr}\,\mathbf{C}=2\cos\phi+1$/$\det\mathbf{C}=1$；(2) $\mathfrak{se}(3)$ 与 $\mathrm{ad}(\mathfrak{se}(3))$ 的 **Jordan 分解**（不可对角化、广义特征向量）；(3) **三个最小多项式**：$(\boldsymbol{\phi}^\wedge)^3+\phi^2\boldsymbol{\phi}^\wedge=0$（B.24）、$(\boldsymbol{\xi}^\wedge)^4+\phi^2(\boldsymbol{\xi}^\wedge)^2=0$（B.49）、$(\boldsymbol{\xi}^\barwedge)^5+2\phi^2(\boldsymbol{\xi}^\barwedge)^3+\phi^4\boldsymbol{\xi}^\barwedge=0$（B.75）。
- **为何重要**：**这三个最小多项式正是教材正文已经在用、却未证明出处的降阶引理**。`lie_theory.tex`：thm:so3-exp/thm:se3-exp 的证明用 $(\mathbf{a}^\wedge)^3=-\mathbf{a}^\wedge$ 降阶（= B.24）；sec:bch 的"$\mathbf{T}$ 直接级数式"概念、$\boldsymbol{\mathcal{J}}_l$ 只到四次幂的事实，依赖 B.49（教材正文 8.43–8.44 的"直接级数表达式"本身也缺，见下）；$\boldsymbol{\mathcal{J}}_l$/$\mathcal{T}$ 的级数截断到四次依赖 B.75。当前教材把 $(\mathbf{a}^\wedge)^3=-\mathbf{a}^\wedge$ 当"单位向量恒等式"直接给（eq:a-identities），**未点明它就是 $\mathfrak{so}(3)$ 的最小多项式**，也未给 $\mathfrak{se}(3)/\mathrm{ad}$ 的对应多项式——这是"自包含独立著作"标准下的一处理论缺环。
- **应落何处**：`lie_theory.tex` sec:lie-appendix 推导附录新增"特征/Jordan 分解与最小多项式"（标进阶），把三个最小多项式作为正文降阶的统一出处；至少应补 $\mathfrak{se}(3)$、$\mathrm{ad}(\mathfrak{se}(3))$ 两个最小多项式（8.43/8.67 正文已用其结论）。难度中-高（Jordan 分解推导冗长，但最小多项式本身可经 Cayley–Hamilton 简述）。

### 附带小缺（低优先，归并提及）
- **直接级数式**：$\mathbf{T}=\exp(\boldsymbol{\xi}^\wedge)=\mathbf{1}+\boldsymbol{\xi}^\wedge+\tfrac{1-\cos\phi}{\phi^2}(\boldsymbol{\xi}^\wedge)^2+\tfrac{\phi-\sin\phi}{\phi^3}(\boldsymbol{\xi}^\wedge)^3$（8.44）与 $\mathcal{T}$ 的五项级数（8.68）、$\boldsymbol{\mathcal{J}}_l$ 的直接四次式（8.99）——教材给了分块式但缺"避免分块、直接整体级数"的等价闭式。便于实现，属可选补全。
- **齐次点 $\circledcirc$ 算子**（$6\times4$，8.163）与恒等式 $\boldsymbol{\xi}^\wedge\mathbf{p}=\mathbf{p}^\odot\boldsymbol{\xi}$、$(\mathbf{Tp})^\odot=\mathbf{Tp}^\odot\mathcal{T}^{-1}$（8.164/8.165）：教材只用了 $\odot$（$4\times6$），$\circledcirc$ 与几条恒等式缺。
- **App B.1** $\mathrm{Ad}(SE(3))$ 导数 $\partial(\mathcal{T}\mathbf{x})/\partial\boldsymbol{\xi}=-(\mathcal{T}\mathbf{x})^\barwedge\boldsymbol{\mathcal{J}}$（B.4）：缺，小。

---

## ③ 独立性问题（external_punt / ventriloquize / narration_dependence）

**结论：`lie_theory.tex` 独立性优秀，无违规。** 全章 cite 平衡引用 Barfoot/十四讲/Solà/Handbook/Eade/Strasdat/Chirikjian/Hertzberg，关键定理（$\mathfrak{so}(3)$ 引出、Rodrigues、$\mathbf{J}$、左右雅可比、右扰动求导、$\mathrm{Sim}(3)$ 雅可比）均**本书口吻自证**、记号统一为右扰动。记号对照处（如"部分文献如 Solà/十四讲用右扰动，Barfoot 兼论左右；本书统一右扰动"，note 见 86–90 行、468–479 行、688 行）正是健康的多书平衡，**不是** punt。grep"详见/略去/参见原书/超出本书"在 lie_theory.tex **零命中**。

**唯一一处轻度 narration（在他章，非李群章本身）**：`imu_model.tex` 1045 行 `\rebuilt{$\mathrm{SE}_2(3)$ 预积分细节属不变滤波专题，本章未展开}` 与 `vio.tex` 1146 行同义。这本身合法（标注了去向、未冒充原创），但叠加 ②-F（对称/等变在李群章缺）与 ②-G（$\mathrm{SE}_2(3)$ 在 ch7 习题 8.21 出现），说明**"$\mathrm{SE}_2(3)$ + 不变滤波"这条线在全书是悬空的**——若不在 lie_theory 或 eskf 补 §8.4 对称性 + $\mathrm{SE}_2(3)$ 群定义，多处"未展开"就缺了统一锚点。建议补 ②-F 时一并给 $\mathrm{SE}_2(3)$ 群定义（Barfoot 习题 8.21）。

**跨章一致性提示（非独立性违规，但需修）**：vio/imu 引用的 $\dot{\mathbf{r}}=\mathbf{J}_r^{-1}\boldsymbol{\omega}$ 与连续旋转预积分依赖 ②-C 的 $\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$，而该前置式在 `lie_theory.tex` 缺席——属"被引定理无出处"，应在李群章补齐使引用闭合。

---

## ④ 最该补的 3–5 项（按 贡献×不可替代性 / 工作量 排序）

1. **【最高】运动学的李代数形式 $\boldsymbol{\omega}=\mathbf{J}\dot{\boldsymbol{\phi}}$（②-C 核心式 8.236/8.237）**。
   理由：贡献大、工作量小（半页 insight + 一个证明），且能**立即闭合** vio/imu 对 $\mathbf{J}_r^{-1}\boldsymbol{\omega}$ 的悬空引用。落 `lie_theory.tex` thm:se3-exp 后。

2. **【高】位姿融合（李群 GN）+ 位姿求逆/相关位姿复合差分（②-E，式 8.362/8.366/8.368/8.371–8.377）**。
   理由：Barfoot 把"李群优化 + 不确定度"合流的招牌应用，全树无落点；直接支撑位姿图、回环、多传感器对齐的协方差正确性。工作量中-高，但与现有 sec:perturb/sec:uncertainty 衔接顺滑。落 `lie_theory.tex` sec:uncertainty 末（或 slam_state_estimation/loop_closure）。

3. **【高】李群上的距离/体积元 $\lvert\det\mathbf{J}\rvert$ 与积分 + 高斯 PDF 严格定义（②-A + ②-D，式 8.107–8.125 + 8.312–8.331）**。
   理由：这是 §8.3 概率框架的**定义地基**（密度归一化、均值隐式方程都依赖 $\det\mathbf{J}$ 诱导因子），缺它则"切空间高斯"只有操作没有定义。两项天然连写，工作量中。落 `lie_theory.tex` sec:uncertainty 开头。

4. **【中-高】三个最小多项式 + eigen/Jordan 分解（②-G，式 B.24/B.49/B.75）**。
   理由：是教材正文降阶（Rodrigues、$\mathbf{T}/\mathcal{T}/\boldsymbol{\mathcal{J}}$ 级数截断）已在用却未给出处的统一引理，关乎"自包含独立著作"成色。至少补 $\mathfrak{se}(3)/\mathrm{ad}$ 两式（正文 8.43/8.67 已用其结论）。工作量中（最小多项式可经 Cayley–Hamilton 简证，Jordan 完整推导可标进阶/选做）。落 sec:lie-appendix。

5. **【中】矩阵李群上的插值（②-B，式 8.128/8.155 + 匀角速度诠释 8.134–8.137）**。
   理由：连续时间 SLAM、关键帧间位姿、时间对齐的基础，且与现有 LOAM"线性插值"形成对照（澄清"线性插值不保群、指数插值保群"）。工作量中，可标 $*$ 选读。落 sec:manifold 后。

> 已充分吸收、**无需再补**：群/李代数定义、Rodrigues、$\mathrm{SE}(3)$ 指数与 $\mathbf{J}$、BCH + $\mathbf{J}_l/\mathbf{J}_r$ + $\boldsymbol{\mathcal{J}}_l/\mathbf{Q}_l$、伴随 $\mathrm{Ad}/\mathrm{ad}$ + 移指数恒等式、右扰动求导、流形/$\boxplus\boxminus$、GN on SO(3)/SE(3)、一阶/四阶位姿复合 + banana 分布、非线性相机模型不确定度（在 camera_model.tex）、Poisson 方程与广义速度（在 rigid_body_motion.tex）、$\mathrm{Sim}(3)$。
