# 完备性审核 — Barfoot《State Estimation for Robotics》2nd ed. ch6–10 + 附录 A–D

> **审核员**：完备性审核员（独立复核，只读，未改任何 .tex / refs.bib）。
> **口径**：吸收 = 五维（知识 / 讲解过程 / 分析过程 / 脉络 / 思想洞见）全量用本书口吻重写。重点抓“公式在、但 Barfoot 推导链/几何直觉/洞见丢了”的**退化**，而非查抄。
> **方法**：通读源书 ch6（1341 行）、ch7（3334 行全读）、ch8（点云/Wahba）、ch9（BA）、ch10（STEAM）+ 附录（经残留基线 + 直接 grep 复核）；逐章提炼五类清单；grep/read 教材落点逐一定位并判定；独立复核基线报告的当前态。
> **关键发现（务必先读）**：**六份逐章基线报告（ch6/ch7/ch8-9/ch10/附录CD/Wave2）写于教材大规模扩写之前，其列出的几乎全部缺口现已被填平。** 本次独立复核确认：`lie_theory.tex` 已从 ~898 行扩到 **1553 行**，新增了基线所缺的全部小节；`slam_state_estimation.tex` 新增完整 STEAM 节；`kalman_eskf.tex` 新增 InEKF 专题；`matrix_calculus.tex` 新增 Van Loan 与 FIM 重参数化；`point_cloud_processing.tex` 新增 Wahba 完整 case 分析 + SE(3) 跟踪范式；`imu_model.tex` 删除 SE₂(3) 的 `\rebuilt` punt。Wave2 已用 numpy/scipy 独立数值验证这批新内容在机器精度上正确。
> **审核日期**：2026-06-18（晚于全部逐章基线，本报告为最新态）。

---

## ① 总体结论

**Barfoot ch6–10 + 附录 A–D 的吸收，在内容层面已实质达成（估计 ≥ 95% 的不可替代严谨内容已自包含落地）。** 五维全部到位：不仅公式/定理/证明在，Barfoot 的**脉络主线**（“右扰动 + 矩阵李群统一处理旋转/位姿”“把估计搬到流形上”）被本书用右扰动主线**系统性重述并升级**，且每处都附 Barfoot 左扰动↔本书右扰动的精确换算 note。**独立性优秀**：全树对 ch6–10 相关文件扫描 external_punt / ventriloquize / narration_dependence **零命中**（Wave2 已确认），`\cite{barfoot2024state}` 均为合法出处标注或多书平衡对照。

**逐章五维覆盖表（当前态）：**

| Barfoot 单元 | 知识(a) | 讲解(b) | 分析(c) | 脉络(d) | 思想(e) | 落点 | 判定 |
|---|---|---|---|---|---|---|---|
| **ch6 §7.1–7.3** 向量/旋转表示/运动学/位姿/变换矩阵/Frenet–Serret | ✅ | ✅ | ✅ | ✅ | ✅ | `rigid_body_motion.tex` | ✅ 全量（多处补全 Barfoot“without proof”的证明） |
| **ch6 §7.2.5** 扰动旋转 + Example 7.1 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` §扰动 | ✅ 升级为右扰动李代数版 |
| **ch6 §7.4.1–7.4.2** 透视/双目相机（本质/基础/单应/视差，含全证明） | ✅ | ✅ | ✅ | ✅ | ✅ | `camera_model.tex` | ✅ 全量 |
| **ch6 §7.4.3** RAE/range-bearing 激光观测模型 | ✅ | ✅ | ✅ | ✅ | ✅ | `lidar_slam.tex` `eq:lidar-rae-jac` | ✅ **已补**（基线 A 缺口，今含正/逆 + 3×3 雅可比 + 平面退化，Wave2 有限差验证 2e-10） |
| **ch6 §7.4.4** IMU + 杆臂加速度计模型 | ✅ | ✅ | ✅ | ✅ | ✅ | `imu_model.tex` `eq:imu-leverarm-acc` | ✅ **已补**（基线 B 缺口，今含向心/切向/Coriolis 二次求导，Wave2 代数验证通过） |
| **ch7 §8.1.1–8.1.5** 群/李代数/指数对数/Ad·ad/BCH/Jl·Jr·Ql | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` | ✅ 全量 |
| **ch7 §8.1.6** 距离/内积/体积元 \|detJ\|/积分/幺模性 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `thm:detJ` | ✅ **已补**（基线 A 缺口，含闭式证明 + Haar 测度 + “众数≠均值”洞见） |
| **ch7 §8.1.7** 保群（测地）插值 + 扰动插值 + Faulhaber | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `thm:lie-interp` | ✅ **已补**（基线 B 缺口，含匀速=Poisson 解的洞见、与 LOAM 线性插值对照） |
| **ch7 §8.1.8** 齐次点 ⊙/⊛ 算子与恒等式 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `subsec:odot-algebra` | ✅ **已补**（含 ⊛、两条换位式、(Tp)⊙=Tp⊙𝒯⁻¹） |
| **ch7 §8.1.9–8.1.10** SO(3)/SE(3) 上 GN + 黎曼流形优化 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` §扰动/§流形 + `nonlinear_optimization.tex` | ✅ 等价吸收 |
| **ch7 §8.1.11** 恒等式表 | ✅ | — | — | — | — | `lie_theory.tex` 速查表 | ✅ |
| **ch7 §8.2** 旋转/位姿运动学 ω=Jφ̇、数值积分、Magnus、transport | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `thm:so3-kinematics`/`thm:se3-kinematics` + `rigid_body_motion.tex` | ✅ **已补**（基线 C 缺口，今有完整证明 + 连续预积分前置定理 insight） |
| **ch7 §8.3.1** 李群高斯/诱导密度/左中右扰动/均值隐式方程 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `subsec:tangent-gaussian` | ✅ **已补**（基线 D 缺口，含 p(T)=p(ε)/\|det𝒥\| 推导） |
| **ch7 §8.3.2** 旋转矢量不确定度（四阶 + Isserlis） | ✅ | ✅ | ✅ | ✅ | ✅ | `camera_model.tex` + `lie_theory.tex` | ✅ |
| **ch7 §8.3.3** 位姿复合（二阶/四阶/sigmapoint/banana） | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `eq:compound-cov`/`eq:fourth-order` | ✅ |
| **ch7 §8.3.4** 位姿求逆精确协方差 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `eq:pose-inverse-cov` | ✅ **已补** |
| **ch7 §8.3.5** 相关位姿复合/差分（Σ₁₂ 交叉项） | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `eq:corr-compound`/`eq:corr-diff` | ✅ **已补**（基线 E，含“相关当独立会失真”陷阱） |
| **ch7 §8.3.6** 位姿融合（李群 GN，K 估计） | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `subsec:pose-fusion` | ✅ **已补**（基线 E，含“融合=李群上信息相加”洞见，Wave2 验证） |
| **ch7 §8.3.7** 非线性相机模型不确定度（二阶 + sigmapoint + 立体实验） | ✅ | ✅ | ✅ | ✅ | ✅ | `camera_model.tex` | ✅ 全量 |
| **ch7 §8.4** 对称/不变/等变、不变误差、InEKF/EqF 动机 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `sec:lie-symmetry` | ✅ **已补**（基线 F，含定位代价不变/最优位姿等变两例） |
| **附录 B.1** Ad(SE(3)) 导数 | ✅ | ✅ | ✅ | — | — | `lie_theory.tex` `deriv:Ad-deriv` | ✅ **已补** |
| **附录 B.2** 运动学雅可比恒等式 J̇−ω∧J=∂ω/∂φ | ✅ | ✅ | ✅ | — | ✅ | `lie_theory.tex` 运动学恒等式 derivation | ✅ **已补**（基线 B-2/B-3） |
| **附录 B.3** eigen/Jordan 分解 + 三个最小多项式 + 直接级数式 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory.tex` `thm:minimal-poly`/`deriv:direct-series` | ✅ **已补**（基线 G/B-5/B-6，含“最小多项式=闭式总开关”洞见 + SE(3)/Ad 直接整体级数 8.44/8.68） |
| **ch8(§9.1)** 点云配准/Wahba（四元数特征值 + SVD + 迭代 GN + de Ruiter–Forbes 完整 case） | ✅ | ✅ | ✅ | ✅ | ✅ | `point_cloud_processing.tex` | ✅ 全量（`thm:pc-wahba-cases` 已补完整 case + 二阶判据 + 三点不共线充分条件） |
| **ch8(§9.2)** 点云跟踪（SE(3)-EKF + 批量） | ✅ | ✅ | ✅ | ✅ | ✅ | `point_cloud_processing.tex` `alg:pc-track-ekf` | ✅ **已补**（基线 partial，今有 G=Dᵀ(Ťp)⊙ + SE(3)-EKF 五式最小范式） |
| **ch8(§9.3)** 位姿图松弛 + 生成树初始化 + 链式稀疏 | ✅ | ✅ | ✅ | ✅ | ✅ | `nonlinear_optimization.tex` + `slam_system.tex` `eq:sys-pgo-spantree`/`eq:sys-pgo-tridiag` | ✅ **已补**（基线 partial，今有生成树初始化 + 链式块三对角，Wave2 验证） |
| **ch8(§9.4)** 惯导 / IMU 预积分 / SE₂(3) | ✅ | ✅ | ✅ | ✅ | ✅ | `imu_model.tex` + `vio.tex` + `lie_theory.tex` `sec:se23` | ✅ **已补**（基线 major #1，SE₂(3) 群结构整支落地于李群章，含群仿射 + 不变滤波动机，Wave2 全 PASS） |
| **ch9(§10.1)** 束调整 BA（⊙ 雅可比/箭头矩阵/Schur/协方差恢复/二阶修正） | ✅ | ✅ | ✅ | ✅ | ✅ | `camera_model.tex` + `nonlinear_optimization.tex` | ✅ 全量 |
| **ch9(§10.2)** SLAM = BA + 运动先验（MAP=ML+先验/因子图/可观测性） | ✅ | ✅ | ✅ | ✅ | ✅ | `slam_state_estimation.tex` | ✅ 全量 |
| **ch10** 连续时间 SE(3) 轨迹估计 STEAM（局部 GP/WNOA/插值/外推/WNOJ·Singer·连续体） | ✅ | ✅ | ✅ | ✅ | ✅ | `slam_state_estimation.tex` `sec:est-steam` | ✅ **已补**（基线唯一 major 缺口，今为完整一节 ~270 行，Wave2 验证 Φ/Q/插值权机器精度） |
| **附录 C.1** 多元高斯 FIM 重参数化 + vech/复制矩阵 | ✅ | ✅ | ✅ | — | — | `matrix_calculus.tex` `thm:matderiv-fim-reparam`/`def:matderiv-vech` | ✅ 通用化吸收（四参数化逐一对照表未照搬，见 ②-P1） |
| **附录 C.2** Stein 引理 | ✅ | ✅ | ✅ | — | — | `slam_state_estimation.tex` + `nonlinear_optimization.tex` GVI | ✅ |
| **附录 C.3** 连续→离散 + Van Loan 分块矩阵指数 | ✅ | ✅ | ✅ | ✅ | ✅ | `matrix_calculus.tex` `thm:matderiv-vanloan` | ✅ **已补**（基线 C-3） |
| **附录 C.4** 不变 EKF（Barrau–Bonnabel） | ✅ | ✅ | ✅ | ✅ | ✅ | `kalman_eskf.tex` `paper:inekf` | ✅ **已补**（基线 major #2，含群仿射/log-linear/为何治一致性，且显式区分迭代 EKF；代数细节见 ②-P2） |
| **附录 D** 习题解（ch2–8） | ✅ | — | — | — | — | 方法散于各章正文 | ✅ 方法全覆盖（无逐题搬运义务） |

---

## ② ❌ 高价值缺失 与 ⚠️ 讲解/脉络/思想退化 清单（重点）

> **结论先行：本次独立复核未发现任何 ❌ confirmed_missing（高价值缺失），也未发现 ⚠️ 退化（公式在而推导链/直觉/洞见丢失）。** 恰恰相反，新增内容普遍在 Barfoot 之上**补强了洞见层**（如“最小多项式=闭式总开关”“融合=李群上信息相加”“测地插值=匀速=Poisson 解”“\|detJ\| 把密度往大角度压、故众数≠均值”——这些 insight 盒是 Barfoot 原文没有、本书自加的“把人讲懂”增量）。下列仅剩 **3 项可选/装饰性**残留，均不影响“完全吸收”判定。

### 可选残留（低优先，补不补都达标）

- **P-1 ⚪（C.1 装饰性，低）**：Barfoot C.1.1–C.1.6 那张“同一高斯六种参数化各自 FIM/逆 FIM”逐一对照表（canonical / 对称化 vech+D / hybrid / natural，及各自 ½Σ⁻¹⊗Σ⁻¹、Dᵀ(·)D 块，与“natural 参数 FIM 非块对角”这一名实不符观察）未照搬。
  - **判定**：**非缺口**。其**方法**（链式法则作用于对数似然曲率）已被一般化为 `thm:matderiv-fim-reparam`（雅可比夹心 ℐ_φ=JᵀℐθJ），且“协方差↔信息形式”“流形 CRLB 拉回李代数”“量纲/条件数”三处应用已写出；真正用途（自然梯度/变分推断）由 `nonlinear_optimization.tex` GVI/ESGVI 整节兑现。逐一枚举属“附录全量对照”的可选增益。
  - **落点（若补）**：`matrix_calculus.tex` FIM 小节加一张选读表。

- **P-2 ⚪（C.4 深度，低–中）**：`paper:inekf` 给的是 InEKF 的概念 + 性质 + 去向（群仿射⇒误差自治、为何改善一致性、与 ESKF/FEJ 关系），**未复现** Barfoot §C.4 把标准 EKF 五式一步步代数变换成不变形式的细节（F′=1、左不变创新 Ť⁻¹(y−y̌)、G′=p⊙、那处协方差近似 𝒯̌⁻¹P̂𝒯̌⁻ᵀ≈P̂′）。
  - **判定**：**非缺口**。本书右扰动主线下左不变结构本属专题，`paper:inekf` 已达“读者懂其所以然”的标准。若要与附录 C.4 等深可加一个选读 derivation。
  - **落点（若补）**：`kalman_eskf.tex` `paper:inekf` 后加选读 derivation。

- **CM-1 ⚪（装饰图，低）**：立体相机 MAP 偏差的蒙特卡洛直方图（Barfoot Fig 4.4）未复现。
  - **判定**：**非缺口**。该实验的**数值**已复用（MAP −33 cm、GVI 0.28 cm，`nonlinear_optimization.tex:211`），机理（均值≠众数、线性化丢二阶曲率）已用文字 + insight 讲透并给 ML 偏差闭式量化。仅缺那张直方图，影响插图丰富度而非内容吸收。

### Wave2 复核遗留的两处“待作者复核”（非缺口、非退化，属记号/约定核对）

- **lie_theory.tex:1043（NEEDS-FIX，已知）**：`eq:se23-adjoint` 段把 SE₂(3) 9×9 左雅可比的 Q_l 块 `\cref{eq:se23-hat}` **错指**（5×5 hat 无 Q_l）。内容本身正确（Wave2 数值证实两耦合块恰为 Q_l(ρ,φ)/Q_l(ν,φ)），仅引用应改为 `\cref{eq:se3-6x6}` 或 `\cref{deriv:Ql}`。**属编排级笔误，非吸收缺口。**
- **lie_theory.tex:709–717（MINOR，待复核）**：插值扰动一阶式 `eq:interp-perturb`（A=αJ_l(αφ)J_l(φ)⁻¹）疑为照搬 Barfoot 左扰动结果，Wave2 在右扰动约定下穷举数值未收敛到一阶（边界 A(0)=0/A(1)=I 无误）。建议作者在右扰动下重推、核对 J_l vs J_r 与扰动变量定义。**属进阶旁支的约定核对，非主干内容缺失。**
- **point_cloud_processing.tex:618（MINOR）**：Wahba 二阶判据 `eq:pc-wahba-secondorder` 文字标“U 主轴坐标”疑应为“V 主轴坐标”（公式值与所有 case 结论 Wave2 已验证正确）。**纯文字标注。**

---

## ③ 与逐章基线的差异说明（为何本报告与旧基线结论不同）

六份逐章基线（`Barfoot吸收审计_ch6/ch7/ch8-9/ch10_*.md`、`复核_Barfoot吸收_Wave2.md`、`Barfoot吸收审计_附录CD_残留.md`）写于教材**正在被多个 agent 大量追加内容**的不同时间点。其列为 major/partial 的缺口，在当前快照（2026-06-18 晚）几乎全部已填平：

- **ch6 基线**两缺口（RAE、IMU 杆臂）→ 今在 `lidar_slam.tex`/`imu_model.tex` 落地（Wave2 数值验证）。
- **ch7 基线**七缺口（A 距离/体积/积分、B 插值、C ω=Jφ̇、D PDF 定义、E 位姿融合/相关、F 对称性、G 最小多项式/Jordan）→ 今全在 `lie_theory.tex`（已扩到 1553 行）落地。
- **ch8-9 基线**两 major（SE₂(3)、InEKF）+ 三 partial（SE(3) 跟踪、生成树、Wahba 完整 case）→ 今全部落地。
- **ch10 基线**唯一 major（STEAM）+ B-2/B-3/B-6 + C-3（Van Loan）→ 今 `sec:est-steam` 成完整节、李群恒等式/最小多项式/Van Loan 全落地。
- **附录CD残留基线**（最新一份）已先行确认“内容层面实质达成，只余 P-1/P-2/CM-1 可选项”——本报告独立复核与之一致。

**本报告为最新态：Barfoot ch6–10 + 附录的吸收已实质完成，无高价值缺失、无退化，仅余三项可选装饰性增益 + 三处记号/引用待作者顺手核对。**

---

### 附：本次独立复核证据索引
- ch7 落点（`lie_theory.tex`，1553 行全读）：`thm:so3-kinematics:365`、`thm:se3-kinematics:409`、`thm:lie-interp:678`、`subsec:lie-measure:739`（`thm:detJ:771`）、`subsec:tangent-gaussian:801`、`subsec:correlated-pose:838`、`subsec:pose-fusion:877`、`sec:se23:977`、`sec:lie-symmetry:1066`、`thm:minimal-poly:1287`、`deriv:Ql:1247`、`deriv:direct-series:1306`、`deriv:Ad-deriv:1335`、运动学恒等式 derivation:1353、`subsec:odot-algebra:1369`。
- ch10 STEAM（`slam_state_estimation.tex`）：`sec:est-steam:1316`、`prop:steam-sparse:1449`、`sec:est-steam-query:1466`、外推:1498、样条对照:1535、WNOJ/Singer/连续体:1521。
- InEKF（`kalman_eskf.tex`）：`paper:inekf:979`、SE₂(3) 结构要点:984、谱系/术语/去向:1147/1209/1259。
- SE₂(3) 在 `imu_model.tex:1234`（无 `\rebuilt`，回指 `sec:se23`）、`vio.tex:1147`。
- Wahba 完整 case（`point_cloud_processing.tex`）：`thm:pc-wahba-cases:601`、二阶判据:621、三点不共线:635；SE(3) 跟踪 `sec:pc-track-point:733`、`alg:pc-track-ekf:799`。
- Van Loan（`matrix_calculus.tex`）：`sec:matderiv-vanloan:553`、`thm:matderiv-vanloan`；FIM 重参数化:651（`thm:matderiv-fim-reparam`）、vech/复制矩阵:491（`def:matderiv-vech`）。
- RAE/IMU 杆臂：`lidar_slam.tex` `eq:lidar-rae-jac`、`imu_model.tex` `eq:imu-leverarm-acc`（均 Wave2 数值验证）。
- 独立性：Wave2 对 7 文件正则扫描 external_punt/ventriloquize/narration_dependence **0 命中**；本报告抽查一致。
