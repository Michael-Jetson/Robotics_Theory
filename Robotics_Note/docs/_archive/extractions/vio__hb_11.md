# 抽取留痕：SLAM Handbook ch11《Inertial Odometry for SLAM》（惯性里程计）

> **本文件性质**：项目内部「抽取留痕」（非成书正文）。目标是把源材料【全量保真】抽取下来，供综合 agent 写成自包含书章。服务章节：**视觉惯性里程计 VIO**。
>
> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/11_Inertial_Odometry_for_SLAM/11_Inertial_Odometry_for_SLAM.md`（共 561 行，已逐页完整读毕：1–306 + 307–561）。
>
> **作者**：Guoquan (Paul) Huang, Cédric Le Gentil, Teresa Vidal-Calleja, Davide Scaramuzza, Frank Dellaert, Luca Carlone。
>
> **抽取范围说明**：下面【按源小节号】组织，把每一步推导、每一道例、每一条定义/命题、每一张表/分类、每一个算法逻辑完整记录，公式用 LaTeX 写全并保留源中编号（11.1–11.51 + 表 11.1）。凡源中只给文字而未展开的内容，原样记录并标注「源仅文字、未展开」。源中含若干 OCR 噪声（错排公式、错引式号），我在相应处加【OCR 勘误】标注，给出物理上正确的版本，但同时保留原样供核对。

---

## ★★ 极重要：本源对【VIO 章聚焦清单】的实际覆盖情况（综合 agent 必读）★★

任务给定的「本章聚焦」要求**重点覆盖**：① VIO 问题与可观性；② 松/紧耦合；③ 基于优化（VINS 滑窗 + 边缘化）与基于滤波（MSCKF）**两条主线完整推导并对比**；④ 初始化。

**但本源（Handbook ch11）的真实重心并不在 VIO 求解器，而在 IMU 本身**：它是一篇以 **IMU 预积分（on-manifold preintegration，沿用 Forster et al. [334,335]）** 为核心、加上 **AINS（aided INS）可观性分析** 的章节。VIO 求解器（VINS 滑窗、MSCKF、初始化、松/紧耦合）在本源中**几乎只是被点名提及，没有逐步推导**。为避免综合时误以为本源涵盖全部，先在此明确盘点（这是本次抽取的头号发现）：

| VIO 聚焦点 | 本源覆盖程度 | 位置 / 说明 |
|---|---|---|
| **IMU 测量模型**（含 misalignment/scale/g-sensitivity 扩展） | **完整覆盖** | §11.1.1，式 (11.1)–(11.5) |
| **初始对齐 / 重力对齐**（静止初始化 Gram-Schmidt；高端 IMU 用地球自转解析对齐） | **完整覆盖**（这是本源唯一系统讲的"初始化"，但**不是 VIO 的视觉惯性联合初始化**） | §11.1.2，式 (11.6)–(11.7) |
| **IMU 预积分**（运动积分→流形预积分→噪声传播→偏置更新→因子与偏置模型） | **完整、逐步、最重头** | §11.2，式 (11.8)–(11.34) |
| **先进预积分**（数值积分误差、连续加速度预积分、连续旋转预积分 GP） | **覆盖（以综述口吻，推导多指向引文，但给了关键式 11.35/11.36）** | §11.2.3，式 (11.35)–(11.36) |
| **VIO 可观性 / 4 维不可观零空间（全局位置 + yaw）** | **完整覆盖**（这是本源对"可观性"的核心贡献，含点/线/面特征线性化模型 + 零空间显式给出 + 退化运动表） | §11.3，式 (11.37)–(11.51) + 表 11.1 |
| **退化运动**（纯平移→全局朝向不可观；常加速度→系统尺度；纯旋转/朝特征移动→特征尺度） | **完整覆盖**（表 11.1 + 文字推理） | §11.3.3，表 11.1 |
| **VINS 滑窗优化（fixed-lag smoother）+ 边缘化** | **仅文字、未展开**：源只说"典型实现为 fixed-lag smoother / sliding-window；horizon 外的变量逐步 **marginalize**；常用 **Schur complement** 消去视觉路标"。**没有给边缘化的舒尔补公式、没有 FEJ 一致性推导、没有 VINS-Mono 的具体残差/雅可比**。VINS-Mono、OpenVINS、Kimera、BASALT、DM-VIO 等只作为系统名列出 | §11.4.1（重点段落见下） |
| **MSCKF（基于滤波主线）** | **几乎未出现**：MSCKF 这个名字**在正文中未作为方法展开**；"基于滤波"路线只在可观性小节以"AINS estimator 常以离散时间实现 / EKF 风格"隐含出现，并通过引文 [456,653,1234] 提及一致性（FEJ）问题。**没有 MSCKF 的 null-space projection、no-feature-in-state、measurement compression(QR) 推导** | 无专门小节 |
| **松耦合 vs 紧耦合（loosely/tightly coupled）** | **未出现该术语对**：源未明确区分 loosely/tightly coupled。其讲法是统一的紧耦合 factor-graph（预积分因子 + 视觉因子 + 偏置因子同图联合优化），属"紧耦合优化"，但**没有把它和松耦合对比** | 全章默认紧耦合 |
| **VIO 因子图结构图**（预积分因子/偏置因子/视觉因子/先验） | **覆盖（图 11.4 文字说明）** | §11.4.1，图 11.4 |
| **滤波 vs 优化对比** | **仅一句**：fixed-lag 滑窗 vs 增量求解器 iSAM2 的取舍（延迟保证 vs 运行时尖峰）。**没有 MSCKF vs VINS 的系统化对比表** | §11.4.1 末段 |
| **外参标定 / 时间同步** | **覆盖（文字、分类）** | §11.4.2、§11.4.3 |
| **数值例 / 性能数字** | **有少量**：FEJ-WBA-VINS 在 KAIST seq 38（11.42 km）ATE≈2.05°/21.2 m（0.18%）；好的 VIO 漂移 <1%（可低至 0.1%）；连续局部加速度模型在 EuRoC 上较标准预积分 +5%；连续旋转预积分较离散预积分精度 ≥1 个数量级；扩展位姿海洋导航 1.8 km 后平移误差≈5 m | §11.4.1、§11.2.3、§11.5 |

> **结论先行（给综合 agent）**：写"VIO"章时，本源能提供**权威而完整**的两块：(A) **IMU 预积分全套推导**（运动积分→流形预积分测量模型→噪声协方差线性传播→偏置一阶修正→预积分因子残差 + 偏置随机游走因子），(B) **VIO/AINS 可观性**（点/线/面线性化测量模型 + 4 维零空间显式结构 + 退化运动分类表）。
>
> **但 "VINS 滑窗 + 边缘化(舒尔补) 完整推导"、"MSCKF 完整推导(零空间投影 + 量测压缩)"、"松/紧耦合对比"、"视觉惯性联合初始化(陀螺偏置/重力/尺度/速度恢复)" 这四块本源几乎没有可抽取的推导，必须从其它源补**（典型如 Qin et al. VINS-Mono、Mourikis & Roumeliotis MSCKF 原文、Sola/Barfoot 的 ESKF 与边缘化章节，以及本项目已有的 `slam_state_estimation__*`、`kalman_eskf__*`、`nonlinear_optimization__*` 抽取）。本源把这些当作"已知系统"或"留给引文"。

---

## 记号约定（本源 vs 本书统一约定）

本源在旋转/扰动/坐标系记法上与本书统一约定有**若干关键差异**，综合时务必逐条转换：

| 量 | 本源记号 | 含义 | 与本书统一约定（R∈SO(3)、右扰动为主、ξ=[ρ;φ]、Hamilton 四元数）的差异 |
|---|---|---|---|
| 旋转矩阵 | **`R`**（如 `R_b^w`、`R_w^b`） | 旋转矩阵 ∈ SO(3) | **与本书一致**（用 R，非 C）。下标/上标语义见下行。 |
| 坐标系上下标方向 | **`R_b^w`** = body→world，且 `R_w^b = (R_b^w)^T` | 上标=目标系，下标=源系 | 与本书"`R_{wb}` 把 b 系坐标映到 w 系"语义一致（`R_b^w` ↔ `R_{wb}`）。注意源里 `R_w^b(t)` 出现在测量模型里把世界量旋到 body。 |
| 帧符号 | `F^w`（world，近似惯性系）、`F^b`（body=IMU）、`F^c`（camera/sensor） | 各坐标系 | 一致。源假设 sensor frame 与 body frame 重合（§11.1.1）。 |
| 旋转扰动（预积分推导） | **右扰动**：`R = \hat R · Exp(δφ)`，等价于脚注8 `R_b^w = \hat R_b^w R_b^w(\tilde θ) ≃ \hat R_b^w (I + \tilde θ^∧)` | tangent-space 小角 | **与本书一致（右扰动为主）**。预积分中 `Δ\tilde R_{ij} = R_i^T R_j Exp(δφ_{ij})` 也是右乘小扰动。 |
| tangent/误差旋转 | 预积分小节用 **`δφ`**；可观性小节用 **`\tilde θ`** | so(3) 向量 | 两处符号不同但都是 3 维旋转误差向量；本书统一可记 δφ（或 ξ 的 φ 分量）。 |
| Exp / Log | `Exp(·): R^3→SO(3)`，`Log(·)` 反之；`(·)^∧` hat 算子（向量→反对称阵） | 群指数/对数、hat | 与本书一致。源还用 `exp(φ^∧) ≈ I + φ^∧`（式 11.19）。 |
| 右雅可比 | **`J_r(φ)`**，逆为 `J_r^{-1}(φ)`；式中 `J_r^k ≐ J_r((\tildeω_k - b_i^g)Δt)` | SO(3) 右雅可比 | 与本书一致（右扰动配右雅可比）。 |
| 加速度计测量 | **`a(t)`**（源用 `a` 表"测量值"），真值加速度 `a^w`；后文用 `\tilde a` 表测量 | 比力测量 | ⚠ **同字母两义**：式 (11.1) 左边 `a(t)` 是测量；式 (11.12) 起改用带 tilde 的 `\tilde a` 表测量、不带 tilde 的 `a^w` 表真加速度。转写须按上下文区分。 |
| 陀螺测量 | `ω(t)`（测量），真值角速度 `ω_b^b`；后文 `\tildeω` 表测量 | 角速度测量 | 同上，注意 tilde 区分测量/真值。 |
| 加速度计偏置 | **`b^a`** | accelerometer bias（**上标 a 指传感器，非坐标系**） | 本书常记 `b_a`；上标↔下标差异，语义一致。源明确"上标 a/g 指传感器而非帧"。 |
| 陀螺偏置 | **`b^g`** | gyroscope bias | 本书常记 `b_g`。 |
| 加速度计白噪声 | `η^a`（连续）；`η^{ad}`（离散） | 量测白噪声 | 离散/连续转换见式 (11.12) 后：`Cov(η^{gd}(t)) = (1/Δt)Cov(η^g(t))`，η^{ad} 同理。 |
| 陀螺白噪声 | `η^g`（连续）；`η^{gd}`（离散） | 量测白噪声 | 同上。 |
| 偏置驱动噪声 | 连续 `η^{bg}, η^{ba}`；离散 `η^{bgd}, η^{bad}` | 偏置随机游走的白噪声 | 离散协方差 `Σ^{bgd} ≐ Δt_{ij} Cov(η^{bg})`，`Σ^{bad} ≐ Δt_{ij} Cov(η^{ba})`。 |
| 重力 | **`g` / `g^w`**（world 系重力向量）；body 系 `g^b` | gravity | 一致。世界系选为 gravity-aligned（§11.1.2）。 |
| 速度 | `v` / `v^w` | world 系速度 | 一致。 |
| 位置 | `p` / `p^w` | world 系位置 | 一致。 |
| 预积分增量 | `Δ R_{ij}`, `Δ v_{ij}`, `Δ p_{ij}`（真值）；带 tilde `Δ\tilde R_{ij}`, `Δ\tilde v_{ij}`, `Δ\tilde p_{ij}`（预积分测量） | 相对运动增量 | ⚠ `Δv, Δp` **不是真实的速度/位置物理变化**，是为消去 i 时刻状态与重力而人为定义的量（源原话）。 |
| 预积分噪声 | `η^Δ_{ij} ≐ [δφ_{ij}^T, δv_{ij}^T, δp_{ij}^T]^T ~ N(0_{9×1}, Σ_{ij})` | 9 维预积分测量噪声 | 协方差字母用 **`Σ_{ij}`**（本书一致用 Σ）。 |
| shape 矩阵 | `T_a`（加速度计 misalignment+scale），`T_g`（陀螺），`T_s`（g-sensitivity） | 标定矩阵 | — |
| 时间间隔简写 | `Δt`（IMU 采样周期）；`Δt_{ij} ≐ Σ_{k=i}^{j-1} Δt`；`(·)_i ≐ (·)(t_i)` | shorthand | — |
| 误差状态（可观性） | `\tilde x = {\tildeθ, \tilde b^g, \tilde v^w, \tilde b^a, \tilde p^w, \tilde x_f^w}` | 15+n_f 维误差态 | ⚠ **状态分量排序为 [θ, b^g, v, b^a, p, x_f]**（旋转→陀螺偏置→速度→加速度计偏置→位置→特征），与本书常见 [p,v,φ,b_a,b_g] 不同，转写时务必重排。 |
| 状态转移阵 | 连续 `F(t)=blkdiag(F_c, 0)`，噪声雅可比 `G(t)`；离散 `Φ_{(k+1,k)}` | 线性化系统 | — |
| 可观性矩阵 | **`M(\hat x)`** | observability matrix（依赖线性化点 \hat x） | 与 Fisher 信息/协方差紧密相关（源指向 ch6）。 |
| 不可观零空间 | **`U` = span([... u_i ...])**，`M u_i = 0` | unobservable subspace | — |
| 特征 | 点 `p_f^w`；线 Plücker `l^w=[n_ℓ^w; v_ℓ^w]`；面 `π^w=[n_π^w; d_π^w]` | 几何特征 | — |
| 测量雅可比 | `H_x`（对状态）、`H_f`（对特征位置）、`H_r/H_b`（range/bearing）、`H_ℓ`、`H_π` | Jacobians | ⚠ 本源 `H` 一律指测量雅可比，与别处 Hessian/Householder 无关。 |
| 四元数 | **本源全程用旋转矩阵 R，未使用四元数** | — | 无 Hamilton/JPL 之争；综合时若改用四元数须自行选 Hamilton（本书约定）。 |

---

# 11. Inertial Odometry for SLAM（章引言）

**核心叙述（§开篇，源行 5–7）**：IMU 已成为机器人 SLAM 最普遍的里程计来源之一。IMU 测量其所附着刚体的**线加速度**和**旋转速率**。形态/成本/性能跨度极大：从飞机上又大又准的光学传感器，到手机等消费设备里又小又噪的 MEMS。MEMS IMU 的低 SWAP（Size, Weight and Power）和低价使其成为机器人极佳传感器，二十多年来在 SLAM 中被广泛研究。

**章节安排**（源行 7）：
- §11.1 IMU 基本事实与测量模型；
- §11.2 IMU 预积分（把高频 IMU 数据加入因子图优化框架）；
- §11.3 引入 IMU 带来额外变量（如偏置），讨论 IMU 与外感受传感器（相机/LiDAR）结合后系统的**可观性**（脚注1：可观性确立估计问题在什么条件下是适定的，即给定测量是否有可能算出接近真值的估计）；
- §11.4 现代 IMU-centric SLAM 系统能做到什么（示例）；
- §11.5 近期趋势。

---

## 11.1 Basics of Inertial Sensing and Navigation（惯性感知与导航基础）

**6 轴 IMU** = 加速度计（测量传感器相对惯性系的线加速度）+ 陀螺仪（测量角速度/旋转速率）。（脚注2：IMU 通常还含测磁北方向的罗盘，但在 SLAM 中较少用，因室内/城市环境磁偏差大——大金属结构、电子设备造成局部磁扰动。）

**INS（惯性导航系统）**：传统由航空航天研究，目标是从初始状态 + IMU 测量历史，估计平台当前状态（位姿、速度）。分两类（源行 17）：
- **捷联系统（strapdown）**：IMU 刚固连在平台框架上；
- **稳定系统（stabilized）**：IMU 装在内常平架/多常平架/浮球上，保持相对惯性系朝向恒定。

机器人 INS 多属前者（捷联），即依赖一个刚连平台、测量本地线加速度与角速度的 IMU。机器人里 **inertial odometry（惯性里程计）** 常作为 inertial navigation 同义词，强调其里程计本质。

**漂移与 AINS**（源行 19）：INS 里程估计随时间漂移，故多数应用还依赖其它传感器（GPS、相机、LiDAR），此时称 **aided inertial navigation systems (AINS)**。机器人里常直接按所用传感器组合命名：相机 + IMU 做 3D 运动跟踪 = **visual-inertial odometry (VIO)**；再加回环 = **visual-inertial SLAM**。

### 11.1.1 Sensing Principles and Measurement Models（感知原理与测量模型）

**原理**（源行 23）：陀螺仪设计基本原理是**角动量守恒**；加速度计利用**质量的惯性**测量「相对惯性系的运动学加速度」与「重力加速度」之差。加速度计可有多种设计：把速率陀螺当作摆质量；基于低摩擦壳内检验质量的惯性；基于壳内悬挂检验质量两侧两条薄金属带的振动差。

**测量模型（Measurement Model）**（源行 25）：为简化，假设传感器系与机器人 body 系 `F^b` 重合，世界系 `F^w` 为惯性系（脚注3：航空航天会区分非惯性导航系 ECEF、LGV 与惯性系 ECI；机器人近地小尺度应用常弱化此区分，因地球自转影响相对噪声可忽略，故固定在地球上的 `F^w` 近似当作惯性系）。t 时刻 IMU 测量 `a(t)`、`ω(t)` 通常假设被**加性白高斯噪声 η** 与**缓变偏置 b** 污染：

$$\mathbf{a}(t) = \mathbf{R}_w^b(t) \left( \mathbf{a}^w(t) - \mathbf{g}^w \right) + \mathbf{b}^a(t) + \boldsymbol{\eta}^a(t), \tag{11.1}$$

$$\boldsymbol{\omega}(t) = \boldsymbol{\omega}_b^b(t) + \mathbf{b}^g(t) + \boldsymbol{\eta}^g(t). \tag{11.2}$$

记号解释（源行 35，逐条）：
- 上标 `b` 表该量在 body(IMU)系 `F^b` 中表达。
- t 时刻 IMU 位姿由变换 `{R_b^w(t), p^w(t)}` 描述，把点从 `F^b` 映到 `F^w`（注意 `R_w^b(t) = (R_b^w(t))^T`）。
- `a^w(t) ∈ R^3`：传感器在世界系的加速度。
- `g^w`：世界系重力向量。
- 因此 `R_w^b(t)(a^w(t) - g^w)` 是 IMU 在 body/IMU 系中**实际感受到的加速度（比力）**。
- `ω_b^b(t) ∈ R^3`：`F^b` 相对 `F^w` 的瞬时角速度，在 `F^b` 中表达。
- 噪声 `η^g(t)`、`η^a(t)` 为零均值高斯随机变量。
- 待估偏置 `b^a(t)`、`b^g(t)` 服从随机游走。
- ⚠ 噪声与偏置的上标指**传感器（accelerometer / gyroscope）而非坐标系**；如 `b^a(t)` 是加速度计偏置。

**扩展模型（Extended Models）**（源行 37）：标准模型 (11.1)–(11.2) 在机器人中常够用，但（重）标定时需更精细模型。因制造缺陷，加速度计有 misalignment 与 scale 误差，(11.2) 可扩展为【注：源此处把式号写成"(11.2) 可扩展"，但给出的是加速度计式，应理解为对 (11.1) 加速度计模型的扩展】：

$$\mathbf{a}(t) = \mathbf{T}_a\, \mathbf{R}_w^b(t) \left( \mathbf{a}^w(t) - \mathbf{g}^w \right) + \mathbf{b}^a(t) + \boldsymbol{\eta}^a(t), \tag{11.3}$$

其中 `T_a` 是同时建模加速度计 misalignment 与 scale 误差的 **shape 矩阵**。Scale 误差可含静态或与温度相关的分量，可在（内参）标定中确定。类似地，陀螺测量模型可扩展以捕捉 misalignment 与 scale 误差：

$$\boldsymbol{\omega}(t) = \mathbf{T}_g\, \boldsymbol{\omega}_b^b(t) + \mathbf{b}^g(t) + \boldsymbol{\eta}^g(t), \tag{11.4}$$

【OCR 勘误】源式 (11.4) 印作 `ω(t)=T_a ω_b^b(t)+...`，但正文随即称该 shape 矩阵为 `T_g`（"where T_g is the shape matrix ... in the gyroscope measurements"）。物理上陀螺式应乘 `T_g`，故上面已订正为 `T_g`；源原样为 `T_a`。

`T_g` 是建模陀螺 misalignment 与 scale 误差的 shape 矩阵。陀螺测量常受加速度影响，称 **g-sensitivity**。若其量级在加性白噪声 `η^g(t)` 范围内则视为可忽略；某些 MEMS 硬件上更显著，可建模为：

$$\boldsymbol{\omega}(t) = \mathbf{T}_g\, \boldsymbol{\omega}_b^b(t) + \mathbf{T}_s\, \mathbf{R}_w^b(t)\, (\mathbf{a}^w(t) - \mathbf{g}^w) + \mathbf{b}^g(t) + \boldsymbol{\eta}^g(t), \tag{11.5}$$

【OCR 勘误】源式 (11.5) 首项印作 `T_a ω_b^b`、g-sensitivity 项印作 `R_c^b`；按上下文（陀螺式 + 重力比力项需 body 系）应为 `T_g ω_b^b` 与 `R_w^b`，已订正。其中 `T_s` 是 g-sensitivity 矩阵，可在标定中估计。

### 11.1.2 Initial Alignment（初始对齐）

**动机**（源行 53）：SLAM 中习惯把全局坐标系设为轨迹起始位姿，即令初始位姿 `{R_b^w(0), p^w(0)}` 为单位位姿。但 INS 中 IMU 测量涉及重力（参见 (11.1)），故通常选世界系为 **gravity-aligned**，从而需要把初始位姿与重力方向对齐。换言之，由于 IMU 测量依赖重力方向，机器人朝向不再是任意选择，必须与重力方向一致。

具体需计算把 body(IMU)系对齐到世界系的旋转 `R_b^w(0)`。为简化，假设机器人初始静止，即部署之初无比力施加，常用低价 MEMS IMU 只测重力。显然，仅凭本地重力测量 `g^b` 无法恢复绕重力的旋转（即 yaw），yaw 因应用而自由选取。但可由如下**静止初始化**确定 roll 与 pitch 对应的旋转：

$$\begin{cases}
\boldsymbol{z}_{w}^{b} = \dfrac{\mathbf{g}^{b}}{\lVert\mathbf{g}^{b}\rVert} \\[2mm]
\boldsymbol{x}_{w}^{b} = \dfrac{\mathbf{e}_{1} - \boldsymbol{z}_{w}^{b}\, \mathbf{e}_{1}^{\top} \boldsymbol{z}_{w}^{b}}{\lVert\mathbf{e}_{1} - \boldsymbol{z}_{w}^{b}\, \mathbf{e}_{1}^{\top} \boldsymbol{z}_{w}^{b}\rVert} \quad \Rightarrow\ \boldsymbol{R}_{w}^{b} = \begin{bmatrix} \boldsymbol{x}_{w}^{b} & \boldsymbol{y}_{w}^{b} & \boldsymbol{z}_{w}^{b} \end{bmatrix} \\[2mm]
\boldsymbol{y}_{w}^{b} = \boldsymbol{z}_{w}^{b} \times \boldsymbol{x}_{w}^{b}
\end{cases} \tag{11.6}$$

其中对向量 `e_1 = [1 0 0]^T` 与 `g^b` 执行 **Gram-Schmidt 正交归一化**，`×` 是叉乘。

直观解释（源行 62）：旋转矩阵 `R_w^b` 的最后一列 `z_w^b` 是世界系 z 轴相对 body 系的方向。由于世界系 z 轴与重力对齐，(11.6) 从 body 系重力向量 `g^b` 的测量计算出 `z_w^b`。然后正交归一化过程计算正交向量 `x_w^b`、`y_w^b`，对任意选取的 yaw 补全旋转矩阵 `R_w^b` 的各列。

**高端 IMU 对齐（Alignment with High-end IMUs）**（源行 64）：用高端 IMU 时，陀螺灵敏到能测地球自转率 `ω_{ie}`。此时假设所选世界系为惯性系（如 ECI），可用 body 系重力向量 `g^b` 与地球自转率 `ω_{ie}` 的测量做**解析对齐**：

$$\begin{cases}
\mathbf{g}^{b} = \mathbf{R}_{w}^{b}\, \mathbf{g}^{w} \\[1mm]
\boldsymbol{\omega}_{ie}^{b} = \mathbf{R}_{w}^{b}\, \boldsymbol{\omega}_{ie}^{w} \\[1mm]
\mathbf{g}^{b} \times \boldsymbol{\omega}_{ie}^{b} = \mathbf{R}_{w}^{b}\, (\mathbf{g}^{w} \times \boldsymbol{\omega}_{ie}^{w})
\end{cases}
\ \Rightarrow\
\mathbf{R}_{b}^{w} = \begin{bmatrix} \mathbf{g}^{w\top} \\ \boldsymbol{\omega}_{ie}^{w\top} \\ (\mathbf{g}^{w} \times \boldsymbol{\omega}_{ie}^{w})^{\top} \end{bmatrix}^{-1} \begin{bmatrix} \mathbf{g}^{b\top} \\ \boldsymbol{\omega}_{ie}^{b\top} \\ (\mathbf{g}^{b} \times \boldsymbol{\omega}_{ie}^{b})^{\top} \end{bmatrix} \tag{11.7}$$

【OCR 勘误】源式 (11.7) 排版严重错位：把三条约束（`g^b=R_w^b g^w`、`ω_{ie}^b=R_w^b ω_{ie}^w`、`g^b×ω_{ie}^b=R_w^b(g^w×ω_{ie}^w)`）与最终解 `R_b^w = [...]^{-1}[...]` 混排，且第二个矩阵末行印作 `(g^b×ω_{ie}^w)^T`（混用 w/b）。按物理（三对方向向量对 + 叉乘对，构造可逆矩阵求 `R_b^w`），订正如上：第一矩阵为 world 系三向量的行堆叠（取逆），第二矩阵为 body 系对应三向量的行堆叠，末行应为 `(g^b×ω_{ie}^b)^T`。源原样保留供核对。

结果旋转矩阵 `R_b^w` 通常需**投影回 SO(3)**，以缓解测量噪声的影响。

---

## 11.2 IMU Preintegration and Factor Graphs（IMU 预积分与因子图）

**动机**（源行 74）：测量模型 (11.1)–(11.2) 关联 IMU 测量与机器人状态（尤其位姿、速度）及偏置。原则上可据此导出 ch1 那样的 MAP 估计器，但会导致**不切实际地巨大的因子图**：典型 IMU 高频输出（如 200–1000 Hz），测量模型要求在每个 IMU 采样时刻向因子图加状态，所得因子图很快无法求解。更精明的读者或许注意到连续时间表述能绕开高频加变量，但连续时间惯性导航仍需高频加因子（每个测量一个），同样导致难以处理的因子图。

**预积分核心思想**（源行 76）：IMU **预积分（preintegration）** 给出一条避免以 IMU 频率向因子图加状态/测量的途径。基本想法是把 IMU 测量随时间积分，得到**相对运动测量**，从而把这些（更少的）运动测量加入因子图。但朴素积分（§11.2.1）仍需在因子图求解器每次迭代时重复积分（因积分初始条件可能变）。预积分通过**分离依赖状态变量的项与测量项**来避免此问题。预积分原始思想可追溯到 [709]，被扩展到流形上 [334,335]；§11.2.2 紧随 [334,335] 的表述；§11.2.3 讨论更先进技术；近期工作留到 §11.5。

### 11.2.1 Motion Integration（运动积分）

**运动学模型**（源行 80，引自 [790,318]）：

$$\dot{\mathbf{R}}_b^w = \mathbf{R}_b^w\, (\boldsymbol{\omega}_b^b)^{\wedge}, \qquad \dot{\boldsymbol{v}}^w = \mathbf{a}^w, \qquad \dot{\boldsymbol{p}}^w = \boldsymbol{v}^w, \tag{11.8}$$

描述 body 系 `F^b` 相对世界系 `F^w` 的旋转 `R_b^w`、平移 `p^w`、速度 `v^w` 的演化。

**积分得到 t+Δt 状态**（Δt 为 IMU 采样周期，对 (11.8) 积分，源行 86–95，源标号 (11.10)）：

$$\mathbf{R}_{b}^{w}(t + \Delta t) = \mathbf{R}_{b}^{w}(t)\, \mathrm{Exp}\!\left(\int_{t}^{t + \Delta t} \boldsymbol{\omega}_{b}^{b}(\tau)\, d\tau\right)$$

$$\mathbf{v}^{w}(t + \Delta t) = \mathbf{v}^{w}(t) + \int_{t}^{t + \Delta t} \mathbf{a}^{w}(\tau)\, d\tau$$

$$\mathbf{p}^{w}(t + \Delta t) = \mathbf{p}^{w}(t) + \int_{t}^{t + \Delta t} \mathbf{v}^{w}(\tau)\, d\tau \tag{11.10}$$

（第一式假设角速度 `ω_b^b` 的**方向**在区间 `[t, t+Δt]` 内不变；脚注4：更一般的旋转积分表达式见后文 (11.35)。）

**进一步假设 `a^w` 与 `ω_b^b` 在 `[t,t+Δt]` 内恒定**（源行 99–104，源标号 (11.11)）：

$$\mathbf{R}_{b}^{w}(t+\Delta t) = \mathbf{R}_{b}^{w}(t)\, \mathrm{Exp}\!\left(\boldsymbol{\omega}_{b}^{b}(t)\,\Delta t\right)$$

$$\mathbf{v}^{w}(t+\Delta t) = \mathbf{v}^{w}(t) + \mathbf{a}^{w}(t)\,\Delta t$$

$$\mathbf{p}^{w}(t+\Delta t) = \mathbf{p}^{w}(t) + \mathbf{v}^{w}(t)\,\Delta t + \tfrac{1}{2}\mathbf{a}^{w}(t)\,\Delta t^{2}. \tag{11.11}$$

（(11.11) 可理解为对 (11.9) 中积分作 **Euler 积分**数值求解。）

**代入测量模型 (11.1)–(11.2) 把 `a^w`、`ω_b^b` 写成 IMU 测量的函数**（源行 108–112，源标号 (11.12)，此后省略坐标系下标）：

$$\mathbf{R}(t + \Delta t) = \mathbf{R}(t)\, \mathrm{Exp}\!\left(\left(\tilde{\boldsymbol{\omega}}(t) - \mathbf{b}^{g}(t) - \boldsymbol{\eta}^{gd}(t)\right) \Delta t\right)$$

$$\mathbf{v}(t + \Delta t) = \mathbf{v}(t) + \mathbf{g}\,\Delta t + \mathbf{R}(t) \left(\tilde{\mathbf{a}}(t) - \mathbf{b}^{a}(t) - \boldsymbol{\eta}^{ad}(t)\right) \Delta t$$

$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}(t)\,\Delta t + \tfrac{1}{2}\mathbf{g}\,\Delta t^{2} + \tfrac{1}{2}\mathbf{R}(t) \left(\tilde{\mathbf{a}}(t) - \mathbf{b}^{a}(t) - \boldsymbol{\eta}^{ad}(t)\right) \Delta t^{2}, \tag{11.12}$$

说明（源行 114）：此速度与位置的数值积分假设积分期间朝向 `R(t)` 恒定，对非零旋转率的测量并非 (11.8) 微分方程的精确解；实践中高频 IMU 缓解此近似。采用 (11.12) 因其简单且便于建模与不确定度传播。**离散时间噪声 `η^{gd}` 协方差与采样率有关**，与连续时间噪声 `η^g` 关系为 `Cov(η^{gd}(t)) = (1/Δt) Cov(η^g(t))`；`η^{ad}` 同理（参见 [232, Appendix]）。

**问题**（源行 116）：(11.12) 关联 t 与 t+Δt 状态（Δt = IMU 采样周期），直接作因子图约束会要求每个新 IMU 测量都加新状态 [503]。

**尝试在更长区间积分**（源行 124）：若已有建模其它传感器测量（如 ch7 视觉）的因子图，可用 (11.12) 在因子图中两个**时间相邻状态**之间积分 IMU 测量，称这些状态为 **"keyframe states"**（脚注5：因许多 IMU+相机应用中状态加在相机帧子集即 keyframes 处；此处不失一般性，可任意实例化 keyframe）。对两连续 keyframe（时刻 `t_i`、`t_j`）间所有 Δt 区间迭代 IMU 积分 (11.12)（参见图 11.1）（源行 126–131，源标号 (11.13)）：

$$\mathbf{R}_{j} = \mathbf{R}_{i} \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( \left( \tilde{\boldsymbol{\omega}}_{k} - \mathbf{b}_{k}^{g} - \boldsymbol{\eta}_{k}^{gd} \right) \Delta t \right),$$

$$\mathbf{v}_{j} = \mathbf{v}_{i} + \mathbf{g}\, \Delta t_{ij} + \sum_{k=i}^{j-1} \mathbf{R}_{k} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t$$

$$\mathbf{p}_{j} = \mathbf{p}_{i} + \sum_{k=i}^{j-1} \left[ \mathbf{v}_{k}\, \Delta t + \tfrac{1}{2} \mathbf{g}\, \Delta t^{2} + \tfrac{1}{2} \mathbf{R}_{k} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t^{2} \right] \tag{11.13}$$

shorthand：`Δt_{ij} ≐ Σ_{k=i}^{j-1} Δt`，`(·)_i ≐ (·)(t_i)`。

**(11.13) 的缺点**（源行 133）：(11.13) 的积分在时刻 `t_i` 线性化点变化时须重复 [645]（如 Gauss-Newton 每次迭代）。例如 `R_i` 变会使所有未来旋转 `R_k`(k=i,…,j-1) 变，须重算 (11.13) 的求和与连乘。

（脚注6/同步说明，源行 137）：为简化假设 IMU 与其它传感器同步，IMU 测量在 `t_i`、`t_j` 采样；实践中可插值近似 IMU 恰在 `t_i`、`t_j` 采样的情形，时间同步见 §11.4.3。

**图 11.1**（源行 120–122）：IMU 与相机不同速率示意。来自 [335]（©2016 IEEE）。

### 11.2.2 IMU Preintegration on Manifold（流形上的 IMU 预积分）

**关键洞察**（源行 141）：对运动积分结果 (11.13) 做小变形，可计算 `t_i`、`t_j` 状态间的**相对测量**，使其在线性化点变化时无需重算。关键是把测量表达在**局部帧**（使其不随机器人全局状态估计变化）并**隔离重力贡献**（重力携带全局帧信息）。此过程得到所谓**预积分 IMU 测量**，约束因子图中连续状态间的运动。

**定义相对运动增量**（重排 (11.13)，独立于 `t_i` 处位姿与速度，源行 145–151，源标号 (11.14)）：

$$\Delta \mathbf{R}_{ij} \doteq \mathbf{R}_{i}^{\mathsf{T}} \mathbf{R}_{j} = \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( \left( \tilde{\boldsymbol{\omega}}_{k} - \mathbf{b}_{k}^{g} - \boldsymbol{\eta}_{k}^{gd} \right) \Delta t \right)$$

$$\Delta \boldsymbol{v}_{ij} \doteq \mathbf{R}_{i}^{\mathsf{T}} \left( \boldsymbol{v}_{j} - \boldsymbol{v}_{i} - \mathbf{g}\, \Delta t_{ij} \right) = \sum_{k=i}^{j-1} \Delta \mathbf{R}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t$$

$$\Delta \boldsymbol{p}_{ij} \doteq \mathbf{R}_{i}^{\mathsf{T}} \left( \boldsymbol{p}_{j} - \boldsymbol{p}_{i} - \boldsymbol{v}_{i}\, \Delta t_{ij} - \tfrac{1}{2} \mathbf{g}\, \Delta t_{ij}^{2} \right) = \sum_{k=i}^{j-1} \left[ \Delta \boldsymbol{v}_{ik}\, \Delta t + \tfrac{1}{2} \Delta \boldsymbol{R}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t^{2} \right] \tag{11.14}$$

其中 `ΔR_{ik} ≐ R_i^T R_k`，`Δv_{ik} ≐ R_i^T (v_k - v_i - g Δt_{ik})`。

**重要说明**（源行 153）：与"delta"旋转 `ΔR_{ij}` 不同，`Δv_{ij}` 与 `Δp_{ij}` 都**不对应真实物理的速度/位置变化**，而是被刻意定义为使 (11.14) 右端独立于 i 时刻状态及重力效应。事实上 (11.14) 右端可直接由两 keyframe 间惯性测量算出。

**残留问题**（源行 155）：(11.14) 的求和与连乘仍是偏置估计的函数。分两步处理：§11.2.2.1 假设 `b_i` 已知；§11.2.2.3 给出偏置估计变化时避免重积分的方法。两种情形均假设偏置在 `t_i`、`t_j` 间恒定：

$$\mathbf{b}_{i}^{g} = \mathbf{b}_{i+1}^{g} = \dots = \mathbf{b}_{j-1}^{g}, \quad \mathbf{b}_{i}^{a} = \mathbf{b}_{i+1}^{a} = \dots = \mathbf{b}_{j-1}^{a}. \tag{11.15}$$

#### 11.2.2.1 Preintegrated IMU Measurements（预积分 IMU 测量）

**目标**（源行 162）：(11.14) 关联 keyframe i、j 状态（左端）与测量（右端），已可视为测量模型，但其对测量噪声的依赖颇为复杂，妨碍直接 MAP 估计。本节变形 (11.14)，隔离各惯性测量的噪声项，便于推导测量对数似然。本节假设 `t_i` 处偏置已知。

**所用 SO(3) 指数映射性质**（源行 164–169，源标号 11.16、11.17）：

$$\mathrm{Exp}(\boldsymbol{\phi} + \delta \boldsymbol{\phi}) \approx \mathrm{Exp}(\boldsymbol{\phi})\, \mathrm{Exp}(\mathbf{J}_r(\boldsymbol{\phi})\,\delta \boldsymbol{\phi}), \tag{11.16}$$

$$\mathrm{Exp}(\boldsymbol{\phi})\, \mathbf{R} = \mathbf{R}\, \mathrm{Exp}(\mathbf{R}^{\mathsf{T}} \boldsymbol{\phi}). \tag{11.17}$$

（第一是指数对"和"的一阶近似；第二可由群的伴随表示导出。）

**旋转增量 `ΔR_{ij}` 的变形**（"把噪声移到末尾"，源行 175–179，源标号 11.18）：

$$\Delta \mathbf{R}_{ij} \overset{(11.16)}{\simeq} \prod_{k=i}^{j-1} \left[ \mathrm{Exp}\!\left( \left( \tilde{\boldsymbol{\omega}}_{k} - \mathbf{b}_{i}^{g} \right) \Delta t \right) \mathrm{Exp}\!\left( -\mathbf{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t \right) \right]$$

$$\overset{(11.17)}{=} \Delta \tilde{\mathbf{R}}_{ij} \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( -\Delta \tilde{\mathbf{R}}_{k+1\,j}^{\mathsf{T}}\, \mathbf{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t \right)$$

$$\doteq \Delta \tilde{\mathbf{R}}_{ij}\, \mathrm{Exp}\!\left( -\delta \boldsymbol{\phi}_{ij} \right) \tag{11.18}$$

其中 `J_r^k ≐ J_r^k((\tildeω_k - b_i^g)Δt)`。最后一行定义**预积分旋转测量** `ΔR̃_{ij} ≐ Π_{k=i}^{j-1} Exp((\tildeω_k - b_i^g)Δt)`，及其噪声 `δφ_{ij}`（下节进一步分析）。

**速度/位置变形所用关系**（源行 185–188，源标号 11.19、11.20）：

$$\mathrm{exp}(\boldsymbol{\phi}^{\wedge}) \approx \mathbf{I} + \boldsymbol{\phi}^{\wedge}, \tag{11.19}$$

$$\mathbf{a}^{\wedge} \mathbf{b} = -\mathbf{b}^{\wedge} \mathbf{a}, \quad \forall\, \mathbf{a}, \mathbf{b} \in \mathbb{R}^3, \tag{11.20}$$

（第一是指数映射在原点的一阶近似；第二是向量 wedge 算子的性质。）

**速度增量 `Δv_{ij}` 的变形**（把 (11.18) 代回 (11.14) 中 `Δv_{ij}`，用 (11.19) 近似 `Exp(-δφ_{ij})`，丢弃高阶噪声，源行 194–200，源标号 11.21）：

$$\Delta \boldsymbol{v}_{ij} \overset{(11.19)}{\simeq} \sum_{k=i}^{j-1} \Delta \tilde{\boldsymbol{R}}_{ik} (\mathbf{I} - \delta \boldsymbol{\phi}_{ik}^{\wedge}) \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right) \Delta t - \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t$$

$$\overset{(11.20)}{=} \Delta \tilde{\boldsymbol{v}}_{ij} + \sum_{k=i}^{j-1} \left[ \Delta \tilde{\boldsymbol{R}}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t - \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t \right]$$

$$\doteq \Delta \tilde{\boldsymbol{v}}_{ij} - \delta \boldsymbol{v}_{ij} \tag{11.21}$$

定义**预积分速度测量** `Δṽ_{ij} ≐ Σ_{k=i}^{j-1} ΔR̃_{ik}(\tilde a_k - b_i^a)Δt` 及其噪声 `δv_{ij}`。

**位置增量 `Δp_{ij}` 的变形**（把 (11.18)、(11.21) 代入 (11.14) 中 `Δp_{ij}`，用 (11.19)，源行 206–214，源标号 11.22）：

$$\Delta \boldsymbol{p}_{ij} \overset{(11.19)}{\simeq} \sum_{k=i}^{j-1} \left[ (\Delta \tilde{\boldsymbol{v}}_{ik} - \delta \boldsymbol{v}_{ik}) \Delta t + \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik} (\mathbf{I} - \delta \boldsymbol{\phi}_{ik}^{\wedge}) \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right) \Delta t^{2} - \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t^{2} \right]$$

$$\overset{(11.20)}{=} \Delta \tilde{\boldsymbol{p}}_{ij} + \sum_{k=i}^{j-1} \left[ -\delta \boldsymbol{v}_{ik}\, \Delta t + \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t^{2} - \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t^{2} \right]$$

$$\doteq \Delta \tilde{\boldsymbol{p}}_{ij} - \delta \boldsymbol{p}_{ij}, \tag{11.22}$$

定义**预积分位置测量** `Δp̃_{ij}` 及其噪声 `δp_{ij}`。

**最终预积分测量模型**（把 (11.18)、(11.21)、(11.22) 代回 (11.14) 中 `ΔR_{ij}, Δv_{ij}, Δp_{ij}` 的原始定义；用 `Exp(-δφ_{ij})^T = Exp(δφ_{ij})`，源行 218–223，源标号 11.23）：

$$\Delta \tilde{\mathbf{R}}_{ij} = \mathbf{R}_{i}^{\mathsf{T}} \mathbf{R}_{j}\, \mathrm{Exp}\!\left( \delta \boldsymbol{\phi}_{ij} \right)$$

$$\Delta \tilde{\mathbf{v}}_{ij} = \mathbf{R}_{i}^{\mathsf{T}} \left( \mathbf{v}_{j} - \mathbf{v}_{i} - \mathbf{g}\, \Delta t_{ij} \right) + \delta \mathbf{v}_{ij}$$

$$\Delta \tilde{\mathbf{p}}_{ij} = \mathbf{R}_{i}^{\mathsf{T}} \left( \mathbf{p}_{j} - \mathbf{p}_{i} - \mathbf{v}_{i}\, \Delta t_{ij} - \tfrac{1}{2} \mathbf{g}\, \Delta t_{ij}^{2} \right) + \delta \mathbf{p}_{ij} \tag{11.23}$$

复合测量写成（待估）状态"加"一个随机噪声 `[δφ_{ij}^T, δv_{ij}^T, δp_{ij}^T]^T`。

**小结**（源行 227）：本节把 (11.14) 变形为 (11.23)。(11.23) 的优势是：在合适噪声分布下，可直接实例化 `t_i`–`t_j` 间因子。噪声性质见下节。

#### 11.2.2.2 Noise Propagation（噪声传播）

**目标**（源行 231）：推导噪声向量 `[δφ_{ij}^T, δv_{ij}^T, δp_{ij}^T]^T` 的统计量。虽便利地近似为零均值正态，但准确建模噪声协方差至关重要。本节给出预积分测量协方差 `Σ_{ij}` 的推导：

$$\boldsymbol{\eta}_{ij}^{\Delta} \doteq [\delta \boldsymbol{\phi}_{ij}^{\mathsf{T}}, \delta \boldsymbol{v}_{ij}^{\mathsf{T}}, \delta \boldsymbol{p}_{ij}^{\mathsf{T}}]^{\mathsf{T}} \sim \mathcal{N}(\mathbf{0}_{9 \times 1}, \boldsymbol{\Sigma}_{ij}). \tag{11.24}$$

**旋转噪声 `δφ_{ij}`**（由 (11.18)，源行 236–238，源标号 11.25）：

$$\mathrm{Exp}\!\left( -\delta \boldsymbol{\phi}_{ij} \right) \doteq \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( -\Delta \tilde{\boldsymbol{R}}_{k+1\,j}^{\mathsf{T}}\, \mathbf{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t \right). \tag{11.25}$$

两侧取 Log 并变号（源行 242，源标号 11.26）：

$$\delta \boldsymbol{\phi}_{ij} = -\mathrm{Log}\!\left( \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( -\Delta \tilde{\boldsymbol{R}}_{k+1\,j}^{\mathsf{T}}\, \mathbf{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t \right) \right). \tag{11.26}$$

**SO(3) 对数的一阶近似**（源行 246，源标号 11.27）：

$$\mathrm{Log}(\mathrm{Exp}(\boldsymbol{\phi})\, \mathrm{Exp}(\delta \boldsymbol{\phi})) \approx \boldsymbol{\phi} + \mathbf{J}_r^{-1}(\boldsymbol{\phi})\, \delta \boldsymbol{\phi}. \tag{11.27}$$

（`J_r^{-1}(φ)` 是右雅可比的逆。）反复应用 (11.27)（因 `η_k^{gd}` 与 `δφ_{ij}` 都是小旋转噪声，右雅可比接近单位）得（源行 251，源标号 11.28）：

$$\delta \boldsymbol{\phi}_{ij} \simeq \sum_{k=i}^{j-1} \Delta \tilde{\boldsymbol{R}}_{k+1\,j}^{\mathsf{T}}\, \mathbf{J}_r^k\, \boldsymbol{\eta}_k^{gd}\, \Delta t \tag{11.28}$$

至一阶，`δφ_{ij}` 零均值高斯（零均值噪声 `η_k^{gd}` 的线性组合）。

**速度/位置噪声**（`δv_{ij}, δp_{ij}` 是加速度噪声 `η_k^{ad}` 与旋转噪声 `δφ` 的线性组合，故亦零均值高斯，源行 257–261，源标号 11.29）：

$$\delta \mathbf{v}_{ij} \simeq \sum_{k=i}^{j-1} \left[ -\Delta \tilde{\mathbf{R}}_{ik} \left( \tilde{\mathbf{a}}_k - \mathbf{b}_i^a \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t + \Delta \tilde{\mathbf{R}}_{ik}\, \boldsymbol{\eta}_k^{ad}\, \Delta t \right]$$

$$\delta \mathbf{p}_{ij} \simeq \sum_{k=i}^{j-1} \left[ \delta \mathbf{v}_{ik}\, \Delta t - \tfrac{1}{2} \Delta \tilde{\mathbf{R}}_{ik} \left( \tilde{\mathbf{a}}_k - \mathbf{b}_i^a \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t^2 + \tfrac{1}{2} \Delta \tilde{\mathbf{R}}_{ik}\, \boldsymbol{\eta}_k^{ad}\, \Delta t^2 \right] \tag{11.29}$$

（关系均至一阶有效。）

**结论**（源行 265）：(11.28)–(11.29) 把预积分噪声 `η^Δ_{ij}` 表为 IMU 测量噪声 `η_k^d ≐ [η_k^{gd}, η_k^{ad}]`(k=1,…,j-1) 的线性函数。故由 `η_k^d` 的协方差（IMU 规格给出），可经**简单线性传播**算出 `η^Δ_{ij}` 的协方差 `Σ_{ij}`。扩展推导见 [335]，其还给出**迭代式**随新测量增量计算协方差（更简洁、更适合在线推断）。

#### 11.2.2.3 Incorporating Bias Updates（纳入偏置更新）

**问题**（源行 271）：前节假设预积分（k=i 到 k=j）所用偏置 `{b̄_i^a, b̄_i^g}` 正确且不变。但优化中偏置估计常变小量 `δb`。重算 delta 测量代价高，故给定偏置更新 `b ← b̄ + δb`，用一阶展开更新 delta 测量（源行 273–277，源标号 11.30）：

$$\Delta \tilde{\mathbf{R}}_{ij}(\mathbf{b}_{i}^{g}) \simeq \Delta \tilde{\mathbf{R}}_{ij}(\bar{\mathbf{b}}_{i}^{g})\, \mathrm{Exp}\!\left( \frac{\partial \Delta \bar{\mathbf{R}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g} \right)$$

$$\Delta \tilde{\mathbf{v}}_{ij}(\mathbf{b}_{i}^{g}, \mathbf{b}_{i}^{a}) \simeq \Delta \tilde{\mathbf{v}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}_{i}^{g} + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}_{i}^{a}$$

$$\Delta \tilde{\mathbf{p}}_{ij}(\mathbf{b}_{i}^{g}, \mathbf{b}_{i}^{a}) \simeq \Delta \tilde{\mathbf{p}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}_{i}^{g} + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}_{i}^{a} \tag{11.30}$$

说明（源行 281）：类似 [709] 的偏置修正，但**直接在 SO(3) 上操作**。雅可比 `{∂ΔR̄_{ij}/∂b^g, ∂Δv̄_{ij}/∂b^g, …}`（在积分时偏置估计 `b̄_i` 处计算）描述测量随偏置估计变化的方式。这些雅可比**保持恒定，可在预积分时预计算**。雅可比推导与 §11.2.2.1 把测量写成"大值 + 小扰动"的方法很相似，见 [335]。

#### 11.2.2.4 Preintegrated IMU Factors and Bias Models（预积分 IMU 因子与偏置模型）

**残差**（由预积分测量模型 (11.23) + 噪声一阶零均值高斯 (协方差 Σ_{ij}) (11.24)，定义将出现在因子图优化中的残差 `r_{I_{ij}} ≐ [r_{ΔR_{ij}}^T, r_{Δv_{ij}}^T, r_{Δp_{ij}}^T]^T ∈ R^9`，并含 (11.30) 偏置更新，源行 287–295，源标号 11.31）：

$$r_{\Delta \mathbf{R}_{ij}} \doteq \mathrm{Log}\!\left( \left( \Delta \tilde{\mathbf{R}}_{ij}(\bar{\mathbf{b}}_{i}^{g})\, \mathrm{Exp}\!\left( \frac{\partial \Delta \bar{\mathbf{R}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g} \right) \right)^{\mathsf{T}} \mathbf{R}_{i}^{\mathsf{T}} \mathbf{R}_{j} \right)$$

$$r_{\Delta \mathbf{v}_{ij}} \doteq \mathbf{R}_{i}^{\mathsf{T}}\left( \mathbf{v}_{j} - \mathbf{v}_{i} - \mathbf{g}\, \Delta t_{ij} \right) - \left[ \Delta \tilde{\mathbf{v}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g} + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}^{a} \right]$$

$$r_{\Delta \mathbf{p}_{ij}} \doteq \mathbf{R}_{i}^{\mathsf{T}}\left( \mathbf{p}_{j} - \mathbf{p}_{i} - \mathbf{v}_{i}\, \Delta t_{ij} - \tfrac{1}{2}\mathbf{g}\, \Delta t_{ij}^{2} \right) - \left[ \Delta \tilde{\mathbf{p}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g} + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}^{a} \right] \tag{11.31}$$

这些项可通过把 `‖r_{I_{ij}}‖²_{Σ_{ij}}`（马氏范数平方）加入最小化目标，直接加入因子图。

**偏置模型（Bias Models）**（源行 299）：偏置是缓变量，用 **"Brownian motion"（积分白噪声）** 建模：

$$\dot{\mathbf{b}}^g(t) = \boldsymbol{\eta}^{bg}, \qquad \dot{\mathbf{b}}^a(t) = \boldsymbol{\eta}^{ba}. \tag{11.32}$$

对 (11.32) 在两连续 keyframe `[t_i, t_j]` 区间积分得（源行 305，源标号 11.33）：

$$\mathbf{b}_{j}^{g} = \mathbf{b}_{i}^{g} + \boldsymbol{\eta}^{bgd}, \qquad \mathbf{b}_{j}^{a} = \mathbf{b}_{i}^{a} + \boldsymbol{\eta}^{bad}, \tag{11.33}$$

shorthand `b_i^g ≐ b^g(t_i)`；离散噪声 `η^{bgd}`、`η^{bad}` 零均值，协方差 `Σ^{bgd} ≐ Δt_{ij} Cov(η^{bg})`、`Σ^{bad} ≐ Δt_{ij} Cov(η^{ba})`（参见 [232, Appendix]）。

(11.33) 可作为目标函数的进一步加性项，对所有连续 keyframe 加入因子图（源行 313，源标号 11.34）：

$$\|\mathbf{r}_{\mathbf{b}_{ij}}\|^2 \doteq \|\mathbf{b}_j^g - \mathbf{b}_i^g\|_{\boldsymbol{\Sigma}^{bgd}}^2 + \|\mathbf{b}_j^a - \mathbf{b}_i^a\|_{\boldsymbol{\Sigma}^{bad}}^2 \tag{11.34}$$

【OCR 勘误】源式 (11.34) 第二项印作 `‖b_i^a - b_i^a‖`（同为 i，恒等于 0），按 (11.33) 物理意义应为 `‖b_j^a - b_i^a‖`，已订正；源原样保留供核对。

### 11.2.3 Advanced Preintegration Techniques（先进预积分技术）

**总览**（源行 318）：本节看标准预积分的局限并探索更新的替代。先看 (11.14) 隐含的信号与运动假设，再过各种放松这些假设、得到更精确预积分测量（从而提升 AINS 的定位建图精度）的工作。**注意：源不详述各方法推导，建议读者参原文。** —— 故本小节多为综述性文字，关键式仅 (11.35)、(11.36)。

#### 11.2.3.1 Numerical Integration Accuracy（数值积分精度）

（源行 322）标准预积分用 **Euler 法**把惯性信号积分为离散时刻的旋转/速度/位置伪测量。快而高效，但引入积分误差（从而漂移）。Euler 法即对信号用矩形法则数值求积分（图 11.2 左：信号被以给定频率采样的分段常值块近似；惯性系统中样本对应加速度计/陀螺测量）。

（源行 324）低采样频率下分段常值假设不准，双重积分迅速累积误差（图 11.2 右上）。提高采样频率可缓解（图 11.2 下），但真实问题中采样频率受硬件限制。

（源行 326）[631] 提出用 **GP 回归**（脚注7：GP 回归是非参数概率插值法，参见 [909]）虚拟上采样陀螺与加速度计输入信号到任意时间戳。虽优于标准预积分，但仍基于分段常值假设做数值积分，未充分利用 GP 模型连续性。

**图 11.2**（源行 330–332）：用低（上行）/高（下行）采样频率的 Euler 积分（已知初始条件）示例。

#### 11.2.3.2 Continuous Acceleration Preintegration（连续加速度预积分）

（源行 336）另一减小积分误差的途径是用**连续时间表示**（不限离散时间戳）更好逼近真实惯性信号并做解析积分。除精度增益外，连续时间表示允许**异步查询**预积分测量——对与非硬件同步或采样完全异步的传感器（如事件相机）做惯性辅助状态估计尤为有用。

（源行 338）预积分难点是处理旋转空间。旋转操作的非交换性阻止使用众多经典 Riemann 积分工具。故若干工作**分离预积分的旋转与平移部分**。本小节先在假设旋转积分已解的前提下用连续时间表示探讨平移部分；连续旋转积分见下小节。

（源行 340）[300] 用零阶积分器 [1105] 积分陀螺后，给出速度/位置预积分测量的连续表述，通过求解连续时间微分方程系统（LTV），假设**常加速度计测量**或**常局部加速度**（两模型见 [300]）。相比假设**常全局加速度**的标准预积分 [335]，[300] 证明常局部加速度假设更代表真实场景，在 EuRoC 数据集 [131] 上较标准预积分与常加速度计测量模型**整体 VIO 精度提升约 5%**。

（源行 348）为放松常加速度运动模型假设，可用**可解析积分函数**逼近输入数据。假设旋转部分已解，[632] 把**旋转修正加速度计测量** `â_k`（定义 `â_k = ΔR_{ik} \tilde a_k`）连续表示。图 11.3 显示分段线性与 GP 连续表示相比 Euler 法（图 11.2）的精度增益：
- **分段线性近似**：第一重积分（从 `â_k` 到 `Δv_{ik}`）对应经典**梯形法则**；可解释为**常 jerk 运动模型**，已较 Euler 法显著增益。
- **GP 回归（无模型）**：用 `â ~ GP(0, k_a(t,t')I)`，`k_a(t,t')` 为平方指数协方差核函数；经对 GP 施加线性算子 [969]，可解析推断 `â` 的积分与双重积分。因平方指数核无限可微，方法不依赖任何显式运动模型。图 11.3 底行示 GP 模型较分段线性法的精度提升。核超参控制信号平滑度，可从数据学习或经验设定。

**图 11.3**（源行 344–346）：上行——分段线性近似的连续积分（对应常 jerk 运动假设）；下行——GP 回归的无模型积分。

#### 11.2.3.3 Continuous Rotation Preintegration（连续旋转预积分）

（源行 352）想把连续表示扩到旋转部分，但旋转 R 属 SO(3) Lie 群非欧空间，群操作交换律不成立。求解 (11.8) 的乘积积分（product integral）（源行 354，源标号 11.35）：

$$\mathbf{R}_{b}^{w}(t+\Delta t) = \mathbf{R}_{b}^{w}(t) \prod_{\tau=t}^{t+\Delta t} \mathrm{Exp}\!\left(\boldsymbol{\omega}_{b}^{b}(\tau)\right)^{d\tau} \tag{11.35}$$

**没有已知通解** [109]，需新方法做连续无模型旋转积分。

（源行 359）[630] 提出用 Lie 代数中**旋转向量表示 `r(t)`**（`R(t)=Exp(r(t))`）作线性向量空间，用线性工具做连续积分。该空间动力学为（源行 361，源标号 11.36）：

$$\dot{\mathbf{r}} = (\mathbf{J}_r(\mathbf{r}))^{-1}\, \boldsymbol{\omega}_b^b, \tag{11.36}$$

`J_r(r)` 是 SO(3) 右雅可比在 r 处求值。问题：`r` 与 `ṙ` 都不被 IMU 直接观测。[630] 的关键想法是用 **GP** 与一组**虚拟观测 `ṙ_{t•}`** 建模 `ṙ`，经线性算子表示连续旋转向量函数 r。直观上虚拟观测可解释为连续旋转动力学的**控制点**，通过以 (11.36) 为残差、陀螺测量为 `ω_b^b` 观测的非线性最小二乘优化估计。这是连续旋转预积分的无模型方法，较标准离散预积分**精度提升至少一个数量级**。

（源行 365）该连续法与 STEAM 连续时间状态估计 [47]（ch2 提及）有诸多相似，都在 Lie 代数中做 GP 插值。主要区别：用平方指数核致**稠密**线性系统，对比 STEAM 用稀疏 Markov 法。但因 IMU 预积分窗口一般够短，解稠密系统不成问题。[376] 把优化 inducing values 概念扩展到同时估计旋转修正加速度与旋转向量，从而**关联预积分测量协方差矩阵的旋转与平移部分**。

---

## 11.3 Observability of Aided Inertial Navigation（辅助惯性导航的可观性）

**动机**（源行 369）：因测量噪声、偏置、数值积分误差，纯惯性里程计可能快速漂移（尤其低保真传感器）。常用降漂办法是把 IMU 与外感受传感器（相机/LiDAR）配对成 AINS。引入外感受传感器常增大待估状态（如加外部路标变量），故自然问：传感器数据是否**足以**无歧义估计系统 SLAM 状态——这是**可观性分析**的目标，确定可用测量提供的信息是否足以无歧义估计状态/参数 [117,453]。

**方法**（源行 371）：可观性分析通常通过导出线性化测量模型并计算**可观性矩阵**完成，该矩阵与状态估计的 **Fisher 信息（及协方差）矩阵** 紧密相关 [487,485]（参见 ch6）。系统可观时可观性矩阵满秩；不满秩时，研究其零空间可洞察状态空间中估计器信息不足的方向。可观性分析结果可用于：改进估计**一致性** [1234,456,653]；确定初始化估计器所需的**最小测量** [456,737]；识别造成额外不可观方向、应避免或预警的**退化运动** [1235]。故 AINS 可观性分析（尤其视觉惯性系统 [457,654,1235]）受到大量研究。

**本节范围**（源行 373）：讨论辅助 IMU 的传感器产生**几何特征（点、线、面）**时的可观性。此一般处理可涵盖相机、LiDAR 等广泛传感器并理解退化构型。§11.3.1 引入假设外感受测量几何路标的线性化模型；§11.3.2 用这些模型做可观性分析；§11.3.3 讨论退化构型。

### 11.3.1 Linearized Measurement Models（线性化测量模型）

**AINS 状态**（源行 377）：聚焦产生 landmark-based 表示的 SLAM/里程计前端。多数 AINS 用点特征（尤其相机 [456,653,645,896,375,335]），可用时也用线/面特征 [599,455,414,1236]，此时须用各几何特征增广状态向量。AINS 待估状态（每时刻）含机器人状态 `x_b` 与外部特征状态 `x_f^w`（世界系表达）（源行 379，源标号 11.37）：

$$\boldsymbol{x} = \{\boldsymbol{R}_b^w,\ \mathbf{b}^g,\ \boldsymbol{v}^w,\ \mathbf{b}^a,\ \boldsymbol{p}^w,\ \boldsymbol{x}_f^w\} \tag{11.37}$$

其中 `R_b^w` 为 body 系相对世界系的旋转；`p^w, v^w` 为机器人世界系位置与速度；`b^g, b^a` 为 body 系陀螺与加速度计偏置；特征 `x_f^w` 可为点/线/面（或组合），世界系表达。

（源行 386）后续可观性分析需系统动力学模型（与 IMU 加速度、角速率测量有关）+ 外感受测量模型。下面先回顾 INS 运动学模型，再考虑外感受测量方程。

#### 11.3.1.1 Linearized IMU Kinematic Model（线性化 IMU 运动学模型）

**IMU 运动学模型**（参见 (11.8)、(11.32)，源行 392–397，源标号 11.38/11.39）：

$$\dot{\mathbf{R}}_b^w = \mathbf{R}_b^w (\boldsymbol{\omega}_b^b)^{\wedge}, \quad \dot{\mathbf{v}}^w = \mathbf{a}^w, \quad \dot{\mathbf{p}}^w = \mathbf{v}^w, \tag{11.38}$$

$$\dot{\mathbf{b}}^g(t) = \boldsymbol{\eta}^{bg}, \quad \dot{\mathbf{b}}^a(t) = \boldsymbol{\eta}^{ba} \tag{11.39}$$

（源行 397 处 (11.38)–(11.39) 在原文有一处重复印刷偏置随机游走方程，此处合并。）`η^{bg}, η^{ba}` 是驱动陀螺与加速度计偏置（建模为随机游走）的零均值高斯噪声。

**线性化误差态连续时间动力系统**（源行 401，源标号 11.40）：

$$\dot{\tilde{\boldsymbol{x}}}(t) \simeq \begin{bmatrix} \mathbf{F}_c(t) & \mathbf{0}_{15 \times n_f} \\ \mathbf{0}_{n_f \times 15} & \mathbf{0}_{n_f} \end{bmatrix} \tilde{\boldsymbol{x}}(t) + \begin{bmatrix} \mathbf{G}_c(t) \\ \mathbf{0}_{n_f \times 12} \end{bmatrix} \boldsymbol{\eta}(t) =: \mathbf{F}(t)\, \tilde{\boldsymbol{x}}(t) + \mathbf{G}(t)\, \boldsymbol{\eta}(t) \tag{11.40}$$

其中误差态向量 `\tilde x = {\tildeθ, \tilde b^g, \tilde v^w, \tilde b^a, \tilde p^w, \tilde x_f^w}`（列向量）表示对线性化点的偏离（如 `\tilde b^g` 是偏置相对线性化点的变化）；旋转分量用线性化点处的 **tangent-space 表示 `\tildeθ`**（脚注8：把任意旋转写成线性化点旋转的扰动 `R_b^w = \hat R_b^w R_b^w(\tildeθ)`，`\tildeθ` 是合适 tangent-space 向量，再用小角近似 `R_b^w = \hat R_b^w R_b^w(\tildeθ) ≃ \hat R_b^w(I + \tildeθ^∧)`）。`n_f` 是 `\tilde x_f^w` 维数；`F_c(t)`、`G_c(t)` 是 IMU 状态的连续时间线性化转移矩阵与噪声雅可比矩阵；`η(t)` 是堆叠噪声，含 `η^{bg}`、`η^{ba}` 及把 (11.38) 实际加速度/旋转率换成测量时产生的 IMU 噪声（参见 §11.2.1 推导）。

**离散时间转移矩阵**（源行 405）：AINS 估计器实践中常以离散时间实现，离散动力模型由计算状态转移矩阵 `Φ_{(k+1,k)}`（从 `t_k` 到 `t_{k+1}`）得到，基于 `Φ̇_{(k+1,k)} = F(t_k)Φ_{(k+1,k)}`，初始条件为单位阵：

【OCR 勘误】源行 405 印作 `Φ_{(k+1,k)} = F(t_k)Φ_{(k+1,k)}`，左端应为导数 `Φ̇_{(k+1,k)}`（状态转移矩阵的微分方程），已订正。

$$\boldsymbol{\Phi}_{(k+1,k)} = \begin{bmatrix}
\boldsymbol{\Phi}_{11} & \boldsymbol{\Phi}_{12} & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_{3 \times n_f} \\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_{3 \times n_f} \\
\boldsymbol{\Phi}_{31} & \boldsymbol{\Phi}_{32} & \mathbf{I}_3 & \boldsymbol{\Phi}_{34} & \mathbf{0}_3 & \mathbf{0}_{3 \times n_f} \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_{3 \times n_f} \\
\boldsymbol{\Phi}_{51} & \boldsymbol{\Phi}_{52} & \boldsymbol{\Phi}_{53} & \boldsymbol{\Phi}_{54} & \mathbf{I}_3 & \mathbf{0}_{3 \times n_f} \\
\mathbf{0}_{n_f \times 3} & \mathbf{0}_{n_f \times 3} & \mathbf{0}_{n_f \times 3} & \mathbf{0}_{n_f \times 3} & \mathbf{0}_{n_f \times 3} & \mathbf{I}_{n_f}
\end{bmatrix} \tag{11.41}$$

【OCR 勘误】源式 (11.41) 把最后一行/列的零块维数印作 `0_{n_f×3}`（在前几列）与 `0_{3×n_f}`（在前几行），存在转置混排；按分块矩阵维数自洽（行块顺序 θ,b^g,v,b^a,p,x_f 各 3 维，末块 n_f 维），上面已统一为前 5 行末列 `0_{3×n_f}`、末行前 5 列 `0_{n_f×3}`、末行末列 `I_{n_f}`。源原样含混排，供核对。其中 (i,j) 块 `Φ_{ij}` 可解析或数值求得 [456]。

状态转移矩阵结构解读（按状态序 θ, b^g, v, b^a, p, x_f）：
- 第 2 行（陀螺偏置 b^g）：除自身 `I_3` 外全 0 → 偏置随机游走，转移为单位。
- 第 4 行（加速度计偏置 b^a）：同上，仅 `I_3`。
- 第 6 行（特征 x_f）：仅 `I_{n_f}` → 特征在世界系静止，不随 IMU 动力学演化。
- 旋转(1)、速度(3)、位置(5)行含非平凡耦合块 `Φ_{11},Φ_{12}`（旋转受陀螺偏置影响）、`Φ_{31},Φ_{32},Φ_{34}`（速度受旋转、陀螺偏置、加速度计偏置影响）、`Φ_{51}..Φ_{54}`（位置受前述各项影响）。

#### 11.3.1.2 Exteroceptive Measurement Models（外感受测量模型）

**点特征（Point Features）**（源行 427）：外感受传感器（单目/双目相机、声呐、LiDAR）的点特征测量一般建模为 range 和/或 bearing 观测，是特征在传感器系 `F^c` 中相对位置的函数（源行 429，源标号 11.42）：

$$\boldsymbol{z}_{p} = \underbrace{\begin{bmatrix} \lambda_{r} & \mathbf{0}_{1 \times 2} \\ \mathbf{0}_{2 \times 1} & \lambda_{b} \mathbf{I}_{2} \end{bmatrix}}_{\boldsymbol{\Lambda}} \begin{bmatrix} z_{r} \\ \boldsymbol{z}_{b} \end{bmatrix} = \boldsymbol{\Lambda} \begin{bmatrix} \|\boldsymbol{p}_{f}^{c}\| + \eta^{r} \\ h_{b}\!\left(\boldsymbol{p}_{f}^{c}\right) + \boldsymbol{\eta}^{b} \end{bmatrix} \tag{11.42}$$

其中 `p_f^c = R_w^c(p_f^w - p_c^w)` 是特征在传感器系的位置；`z_r, z_b` 分别为 range、bearing 测量；`h_b(·)` 是通用 bearing 测量函数（具体形式取决于传感器）；`Λ` 是测量选择矩阵，二值元素 `λ_r, λ_b`（如 `λ_b=1, λ_r=1` 则 `z_p` 含 range 与 bearing 两者）；`η^r, η^b` 是测量噪声（简化为加性）。链式法则在当前状态估计处线性化 (11.42)（源行 434，源标号 11.43）：

$$\tilde{\boldsymbol{z}}_{p} = \boldsymbol{z}_{p} - \hat{\boldsymbol{z}}_{p} \simeq \boldsymbol{\Lambda} \begin{bmatrix} \left.\frac{\partial z_{r}}{\partial \boldsymbol{p}_{f}^{c}} \frac{\partial \boldsymbol{p}_{f}^{c}}{\partial \boldsymbol{x}}\right|_{\hat{\boldsymbol{x}}} \tilde{\boldsymbol{x}} + \eta^{r} \\ \left.\frac{\partial \boldsymbol{z}_{b}}{\partial \boldsymbol{p}_{f}^{c}} \frac{\partial \boldsymbol{p}_{f}^{c}}{\partial \boldsymbol{x}}\right|_{\hat{\boldsymbol{x}}} \tilde{\boldsymbol{x}} + \boldsymbol{\eta}^{b} \end{bmatrix} =: \boldsymbol{\Lambda} \begin{bmatrix} \mathbf{H}_{r} \\ \mathbf{H}_{b} \end{bmatrix} \mathbf{H}_{f}\, \tilde{\boldsymbol{x}} + \boldsymbol{\Lambda} \begin{bmatrix} \eta^{r} \\ \boldsymbol{\eta}^{b} \end{bmatrix} =: \mathbf{H}_{x}\, \tilde{\boldsymbol{x}} + \boldsymbol{\eta}^{p} \tag{11.43}$$

`\hat z_p` 是线性化点测量。依选择矩阵 `Λ`，雅可比 `H_x` 可含 range-only 雅可比 `H_r`(λ_r=1,λ_b=0)、bearing-only 雅可比 `H_b`(λ_r=0,λ_b=1) 或两者。

**线特征（Line Features）**（源行 440）：给定两 3D 点 `p_1^w, p_2^w`，过两点的线用 **Plücker 坐标**表示（源行 442，源标号 11.44）：

$$\mathbf{l}^w = \begin{bmatrix} \mathbf{n}_{\ell}^w \\ \mathbf{v}_{\ell}^w \end{bmatrix} = \begin{bmatrix} \mathbf{p}_1^w \times \mathbf{p}_2^w \\ \mathbf{p}_2^w - \mathbf{p}_1^w \end{bmatrix} \tag{11.44}$$

`n_ℓ^w` 是**线矩（line moment）**，编码两点与原点定义的平面的法向；`v_ℓ^w` 是**线方向向量**（需要时可归一化为单位向量）。原点到线的距离 `d_ℓ^w = ‖n_ℓ^w‖/‖v_ℓ^w‖`。世界系 Plücker 坐标可变换到相机系 [1021]（源行 447，源标号 11.45）：

$$\begin{bmatrix} \mathbf{n}_{\ell}^{c} \\ \mathbf{v}_{\ell}^{c} \end{bmatrix} = \begin{bmatrix} \mathbf{R}_{c}^{w\top} & -\mathbf{R}_{c}^{w\top} (\mathbf{p}_{c}^{w})^{\wedge} \\ \mathbf{0} & \mathbf{R}_{c}^{w\top} \end{bmatrix} \begin{bmatrix} \mathbf{n}_{\ell}^{w} \\ \mathbf{v}_{\ell}^{w} \end{bmatrix} \tag{11.45}$$

考虑 3D 线在 2D 图像中被观测。给定图像中线段两端点 `q_1 := [u_1,v_1,1]^T`、`q_2 := [u_2,v_2,1]^T`，2D 视觉线测量模型为这两端点到「反投影 3D Plücker 线投影到图像平面」的距离 [1315]。经 (11.45) 把 3D 线从世界系变到当前相机系，再用已知内参投影到图像 [1021]（源行 452，源标号 11.46）：

$$\boldsymbol{\ell} = \underbrace{\begin{bmatrix} f_2 & 0 & 0 \\ 0 & f_1 & 0 \\ -f_2 c_1 & -f_1 c_2 & f_1 f_2 \end{bmatrix}}_{\mathbf{K}} \begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_3 \end{bmatrix} \begin{bmatrix} \mathbf{n}_{\ell}^c \\ \mathbf{v}_{\ell}^c \end{bmatrix} =: \begin{bmatrix} \ell_1 \\ \ell_2 \\ \ell_3 \end{bmatrix} \tag{11.46}$$

`K` 是**典范投影 Plücker 矩阵**；`f_1, f_2, c_1, c_2` 是标准相机内参。注意只有矩向量 `n_ℓ^c` 参与投影，意味着 `v_ℓ^c` 含的**线 range 与朝向不可测**。故线段两端点到投影线 `ℓ` 的距离作为线特征测量（源行 457，源标号 11.47）：

$$\boldsymbol{z}_{\ell} = \begin{bmatrix} \dfrac{\mathbf{q}_{1}^{\top} \boldsymbol{\ell}}{\sqrt{\ell_{1}^{2} + \ell_{2}^{2}}} \\[3mm] \dfrac{\mathbf{q}_{2}^{\top} \boldsymbol{\ell}}{\sqrt{\ell_{1}^{2} + \ell_{2}^{2}}} \end{bmatrix} + \boldsymbol{\eta}^{\ell} \tag{11.47}$$

`η^ℓ` 是测量噪声。链式法则线性化得线测量雅可比 `H_x = (∂z_ℓ/∂ℓ)(∂ℓ/∂x)`。

**面特征（Plane Features）**（源行 462）：3D 平面用世界系中到原点距离与法向参数化 `π^w = [n_π^w; d_π^w]`，可变换到通常检测平面的局部传感器系（源行 464，源标号 11.48）：

$$\begin{bmatrix} \boldsymbol{n}_{\pi}^{c} \\ d_{\pi}^{c} \end{bmatrix} = \begin{bmatrix} \boldsymbol{R}_{w}^{c} & \mathbf{0}_{3\times1} \\ -(\boldsymbol{p}_{c}^{w})^{\mathsf{T}} & 1 \end{bmatrix} \begin{bmatrix} \boldsymbol{n}_{\pi}^{w} \\ d_{\pi}^{w} \end{bmatrix} \tag{11.48}$$

不失一般性，考虑从点云（LiDAR 或深度传感器）提取的平面特征 `(n_π^c, d_π^c)`，用**最近点（closest point）** `p_π^c = d_π^c n_π^c`（平面到原点）作为 AINS 状态向量中的平面表示 [374]（源行 471，源标号 11.49）：

$$\boldsymbol{z}_{\pi} = d_{\pi}^{c}\, \boldsymbol{n}_{\pi}^{c} + \boldsymbol{\eta}^{\pi} = \boldsymbol{p}_{\pi}^{c} + \boldsymbol{\eta}^{\pi} \tag{11.49}$$

`η^π` 是平面测量噪声。线性化 (11.49) 得平面测量雅可比 `H_x = (∂z_π/∂p_π^c)(∂p_π^c/∂x)`。

### 11.3.2 Observability Analysis（可观性分析）

**可观性矩阵**（源行 478–480，源标号 11.50，参见 [486]）：

$$\mathbf{M}(\hat{\boldsymbol{x}}) = \begin{bmatrix} \mathbf{H}_{x_1} \boldsymbol{\Phi}_{(1,1)} \\ \mathbf{H}_{x_2} \boldsymbol{\Phi}_{(2,1)} \\ \vdots \\ \mathbf{H}_{x_k} \boldsymbol{\Phi}_{(k,1)} \end{bmatrix} \tag{11.50}$$

`H_{x_k}` 堆叠 k 时刻所有测量（点/线/面）的雅可比；记号 `M(\hat x)` 强调可观性矩阵依赖线性化点 `\hat x`。其**零空间 `U`** 即满足 `M(x)u_i = 0` 的零向量张成 `span([... u_i ...]) = U`，描述 AINS 的不可观子空间。零空间为空则系统完全可观。

**核心结论**（源行 483，引 [1234]）：**AINS 一般有 4 个不可观方向**（即零空间 `U` 有 4 个独立向量），描述了**全局 3D 位置**与**全局 yaw**从 IMU 测量与对先前未知路标的局部观测中不可观这一事实。

**4 维零空间的结构**（考虑状态向量含点、线、面各一：`x_f^w = {p_f^w, l^w, π^w}`，外感受测量 `z = {z_p, z_ℓ, z_π}`，参见 (11.42)、(11.47)、(11.49)；算相关系统与测量雅可比 `H_{x_i}`、`Φ_{(i,1)}` 代入 (11.50) 建 `M`，算零空间 null(M)，得 4 个零向量，源行 487，源标号 11.51，引 [1234]）：

$$\mathrm{null}(\mathbf{M}) = \mathrm{span}[\boldsymbol{u}_{1}\ \boldsymbol{u}_{2:4}] = \mathrm{span}\begin{bmatrix} \boldsymbol{u}_{g} & \mathbf{0}_{12\times3} \\ -\boldsymbol{p}_{1}^{w} \times \mathbf{g}^{w} & \mathbf{I}_{3} \\ -\boldsymbol{p}_{f}^{w} \times \mathbf{g}^{w} & \mathbf{I}_{3} \\ -\boldsymbol{g}^{w} & \dfrac{\mathbf{v}_{\ell}^{w}}{d_{\ell}^{w} \|\mathbf{v}_{\ell}^{w}\|} (\boldsymbol{R}_{\ell}^{w} \mathbf{e}_{1})^{\mathsf{T}} \\ 0 & -(\boldsymbol{R}_{\ell}^{w} \mathbf{e}_{3})^{\mathsf{T}} \\ -d_{\pi}^{w} \mathbf{n}_{\pi}^{w} \times \mathbf{g}^{w} & \mathbf{n}_{\pi}^{w} (\boldsymbol{R}_{\pi}^{w} \mathbf{e}_{3})^{\mathsf{T}} \end{bmatrix} \tag{11.51}$$

其中
$$\boldsymbol{u}_g = \begin{bmatrix} (\boldsymbol{R}_w^{c_1} \mathbf{g}^w)^\mathsf{T} & \mathbf{0}_{1 \times 3} & -(\mathbf{v}_1^w \times \mathbf{g}^w)^\mathsf{T} & \mathbf{0}_{1 \times 3} \end{bmatrix}^\mathsf{T},$$

`p_1^w` 指 k=1 时刻传感器位置，`R_w^{c_1}` 是 k=1 时刻传感器系 `C_1` 到世界系 W 的旋转矩阵；`R_π^w` 是用平面法向 `n_π^w` 经 Gram-Schmidt 正交归一化（参见 (11.6)）构造的旋转矩阵；`R_ℓ^w = [ n_ℓ^w/‖n_ℓ^w‖, v_ℓ^w/‖v_ℓ^w‖, (n_ℓ^w×v_ℓ^w)/‖n_ℓ^w×v_ℓ^w‖ ]` 是用线法向与线方向构造的旋转矩阵。

【OCR 勘误】源行 490 把 `R_ℓ^w` 的列向量印作 `[n_ℓ^w/‖n_ℓ^w‖, v_ℓ^w/‖v_ℓ^w‖, n_ℓ^w·v_ℓ^w/...]`（第三列含混排重复），按标准 Plücker 正交基应为第三列 = 前两列叉乘归一化 `(n_ℓ^w×v_ℓ^w)/‖n_ℓ^w×v_ℓ^w‖`，已订正示意；源原样含混排，供核对。

第一个零向量 `u_1` 与**绕重力的旋转（即 yaw）**有关，`u_{2:4}` 与**机器人平移运动**有关。更多分析见 [1234,1233]。

**小结**（源行 492）：可观性矩阵存在 4 维零空间（(11.51)）正确描述了系统**全局位置与 yaw 不可观**。直观上，没有任何测量（IMU 数据、未知点/线/面路标观测）携带全局帧信息，**除 roll 与 pitch 外**——后者可从加速度计对重力方向的测量观测到。此不可观性在 SLAM 中常见（脚注9：不用 IMU 时，landmark-based SLAM 问题的零空间**至少 6 维**，反映无 IMU 时系统**整个 3D 旋转**[除 3D 位置外]都不可观），**且非病态**：只意味着可任意设世界帧的 yaw 与 3D 原点（因这些变量只有相对测量）。加绝对测量传感器（如 GPS）即可消除此不可观性。更需关切的是：某些运动（与线性化点）下，可观性矩阵零空间可**变大**，制造额外不可观维度。

### 11.3.3 Degenerate Motions（退化运动）

**总览**（源行 496）：某些运动会为 AINS 引入额外不可观方向（在上述 4 个预期之外），实践重要因这些退化运动可能导致状态空间某些方向大误差、导致导航失败。AINS 退化运动汇总于**表 11.1**（完整推导见 [1234]）。具体：
- **纯平移（pure translation）**：对**所有特征类型**退化，致**全局旋转**不可观。直观：若系统不旋转，可能混淆重力测量与加速度计偏置，使 roll 与 pitch 不再可观。
- 另三种退化运动——**常加速度（constant acceleration，含常速度即加速度为零的情形）**、**纯旋转（pure rotation）**、**朝特征方向运动（motion in the direction of the feature，单点特征情形）**——对**单目相机（bearing-only 测量）**致**尺度**不可观。其中：
  - **常加速度**致**整个系统（位置、速度、加速度计偏置、特征）尺度**不可观；
  - **纯旋转**与**朝特征移动**仅致**特征尺度**不可观。

（源行 500）注意：后三种退化运动仅在**传感器到特征距离远大于传感器与机器人 body 间外参平移**时成立，即 `‖p_f^c‖ >> ‖p_b^c‖`（实践中通常成立）。

【表 11.1，源行 502–509】AINS 退化运动：

| Motion（运动） | Sensor（传感器） | Unobservable（不可观量） |
|---|---|---|
| 1. Pure translation（纯平移） | General（通用） | Global orientation（全局朝向） |
| 2. Constant acceleration（常加速度） | Mono cam（单目） | System scale（系统尺度） |
| 3. Pure rotation（纯旋转） | Mono cam（单目） | Feature scale（特征尺度） |
| 4. Moving toward point feature（朝点特征移动） | Mono cam（单目） | Feature scale（特征尺度） |

---

## 11.4 Visual-Inertial Odometry and Practical Considerations（视觉惯性里程计与实践考量）

**引言**（源行 511）：惯性测量常与其它传感器数据融合以缓解漂移。本节聚焦相机视觉测量与 IMU 测量用**因子图**融合（脚注10：机器人中也常把惯性数据与 LiDAR、radar 等结合，见 ch8、ch9）。相机与 IMU 是流行组合，因二者都便宜、轻、低功耗，且**互补**：IMU 能捕捉快速加速与旋转，相机能提供丰富环境观测。一方面相机大幅降低纯惯性里程计漂移；另一方面 IMU 能观测原本无法估计的量——尤其单目相机 SLAM 无先验时**无法估计场景尺度**（尺度不可观），加 IMU 可恢复尺度（只要机器人运动非退化，§11.3.3）。含一或多相机 + IMU 的系统通常称 **VIO**，加回环后成 **visual-inertial SLAM**。

### 11.4.1 Visual-Inertial Odometry（视觉惯性里程计）

**应用与延迟要求**（源行 517）：VIO 常作里程计源，常用于闭合轨迹跟踪与控制回路。其它应用如 VR 中用 VIO 补偿用户运动。两种情形 VIO 都须产生**极低延迟**估计（典型 10–50ms）。例如 Meta Quest 3 刷新率 72–120 Hz [761]，VIO 延迟直接影响 VR 体验、是缓解晕动症的关键；轨迹跟踪也须保持低延迟，否则大延迟会致跟踪控制器不稳定与发散。

**★ 滑窗优化 / 固定滞后平滑器 / 边缘化（VIO 求解器核心段，源行 527）★**（综合 agent 重点）：

基于上述考量，**基于因子图的 VIO 系统通常实现为固定滞后平滑器（fixed-lag smoother，亦称 sliding-window optimization 滑窗优化）**，只尝试估计**后退视界（receding horizon）**内的状态（如最近 5–10 秒）。结果因子图示例见图 11.4（预积分 IMU 因子=紫，偏置因子=蓝，视觉因子=橙，先验=黑）。

- **视界选择**权衡计算与精度：视界越长，估计状态空间越大。
- **边缘化（marginalization）**：随时间推移，落出后退视界的因子与变量被**逐步边缘化（gradually marginalized）**。
- **Schur 补消元路标**：许多优化实现还用 **Schur complement（舒尔补）** 把视觉路标从优化中消去，进一步缩小状态空间（见 [335]）。
- **替代方案——增量求解器**：用 iSAM2（§1.7）等增量求解器，在算当前估计时复用之前优化的计算。实践中很准 [335]，但缺点是**不提供延迟保证**、可能致运行时尖峰（spikes），对某些应用成问题。

> ⚠ 抽取层提醒：以上即本源对"VINS 滑窗 + 边缘化"的**全部**内容——**纯文字、无公式**。源**没有给出**：边缘化的舒尔补显式公式、先验信息矩阵/残差的构造、First-Estimate Jacobian (FEJ) 的一致性推导、VINS-Mono 具体残差与雅可比、滑窗内变量的具体管理流程。综合时须从专门文献补这些推导。

**图 11.4**（源行 521–523）：用预积分 IMU 因子的 VIO 因子图示例 [335]。因子图显示：预积分 IMU 因子（紫，约束连续位姿、速度、偏置）；偏置因子（蓝，约束 IMU 偏置随时间演化）；视觉因子（橙，关联相机位姿与外部路标位置）；先验（黑）。

**VIO 系统与性能（VIO Systems and Performance）**（源行 529）：过去十年涌现大量 VIO/SLAM 系统，多有开源实现。流行方法包括：
- 视觉惯性版 **ORB-SLAM** [786]
- **Direct Sparse Visual-Inertial Odometry** [1116]
- **VINS-Mono** [896]
- **OpenVINS** [375]
- **Kimera** [3,942]
- **BASALT** [1118]
- **DM-VIO** [1043]

好的 VIO 系统漂移**低于行进距离的 1%**（如 100m 轨迹后累积误差 <1m），有些低至 **0.1%**。

> ⚠ 抽取层提醒：源把 VINS-Mono、OpenVINS、MSCKF 类（OpenVINS 即 MSCKF 风格滤波器）等**仅作为系统名罗列**，**未展开任一方法的算法推导**。MSCKF 这一名字在正文中**未单独成节**，其"滑窗滤波、不在状态中保留特征、零空间投影(null-space projection)、量测压缩"等标志性技术本源**没有**。

**数值例（FEJ-WBA-VINS on KAIST）**（源行 537）：滑窗优化（如 VINS-Mono [896]）实践中极成功。这里展示较新的滑窗法 **First-Estimate Jacobian (FEJ)-based Window Bundle Adjustment (WBA)-VINS** [180,181] 在 **KAIST Urban Dataset** [515] 上的表现。KAIST 城市数据集聚焦自动驾驶与复杂城市环境定位，韩国采集，车辆装：双目相机对、2D/3D LiDAR、Xsens IMU、光纤陀螺(FoG)、轮编码器、RTK GPS。相机 10Hz，IMU 100Hz。真值轨迹由 FoG、RTK GPS、轮编码器融合得到。图 11.5 示 sequence 38（**11.42 km，36 分钟**）的 FEJ-WBA-VINS (VIO) 估计轨迹叠加 Google 地图。**最终绝对轨迹误差 ATE ≈ 2.05 度 与 21.2 米（轨迹长的 0.18%）**。注意：这是**纯在线 VIO、无回环**的结果。

**图 11.5**（源行 531–533）：[180] 最近的滑窗优化 VIO 算法在 KAIST 城市自动驾驶数据集 seq 38（36 分钟、11.42 km）上运行示意。VIO 估计与真值叠加在 Google 地图上；底部为两张样本图像。VIO（无回环）最终 ATE = 2.05 度 / 21.2 米（0.18%）。

（源行 539）VIO 在自动驾驶等自主系统的应用见 [3,4]，其还讨论特征跟踪、关键帧选择、不同传感模态（单目/双目/RGB-D 图像、轮里程计）融合的挑战。

### 11.4.2 Extrinsic Calibration（外参标定）

（源行 543）准确 AINS 需做传感器**外参标定**，即估计不同传感器间相对位姿（如相机相对 IMU 的位姿）。文献方法主要分两类：
- **离线（offline）标定**：部署前执行标定流程，常用标定靶 [350]、已知运动模式 [686] 或环境先验 [631,714]。或多或少耗时，可能需专门设备与受训操作员；通常更准，但繁琐，在期望由非专家大规模使用的场景中不理想。
- **在线（online）标定**：不需专门流程 [301,633,1211,1237]，外参作为状态估计问题一部分估计。优势是能适应系统变化（如传感器位移）而无需重新标定。但可能不如离线准，且可能使状态估计问题更复杂甚至病态(ill-posed) [1235]。

### 11.4.3 Temporal Synchronization（时间同步）

（源行 547）惯性辅助系统另一关键是传感器数据的**时间同步**。未被处理的错误同步会致轨迹估计显著误差，和/或在 benchmark 指标中引入偏差。同步可在硬件或软件做：
- **硬件低层**：常用专门硬件经特定同步输入引脚、基于公共时钟信号触发各传感器数据采集。但并不总可行（尤其传感器经不同通信协议连接计算机时）。
- **传感器内置同步**：某些传感器有内置同步机制，无需专门硬件输入即可同步不同传感器时钟。**PTP（Precision Time Protocol）** 是经以太网软件同步的例子，许多 LiDAR、radar、INS 方案可用此协议同步。
- **时间戳对齐**：在传感器层给数据打时间戳，后处理对齐。通常不如前述方法准且鲁棒。
- **时间偏移作状态**：若无法同步且后处理不可行（如在线应用），某些状态估计算法把**时间偏移作为状态变量**纳入估计问题 [301,376,1237]。

---

## 11.5 Further Readings & Recent Trends（延伸阅读与近期趋势）

（源行 551）惯性里程计进展正稳步转化为工业产品，但辅助惯性导航仍是研究热点。

**扩展位姿预积分（Extended Pose Preintegration）**（源行 553）：近期趋势包括用**扩展位姿流形（extended-pose manifolds）**与**高阶噪声传播** [120] 改进 IMU 预积分的不确定度建模。Brossard et al. [120] 扩展预积分理论以考虑地球自转的 **Coriolis 与离心力**。Vial et al. [1126] 给出用线速度传感器 + 导航级 IMU 的扩展位姿预积分例子：海洋导航一小时、1.8 km 轨迹后，报告平移误差约 **5m**。

**连续时间状态表示（Continuous-time State Representations）**（源行 555）：除预积分（减少离散状态变量数）外，连续时间状态表示也能容纳众多 IMU 测量而不增估计状态维度。例子：[349] 用 **B-spline 基函数**；[47] 用 **GP 先验**。两者都允许在固定状态变量集间用插值动力学、把高频 IMU 测量用于残差。[130] 比较"把 IMU 测量作为输入直接放入连续时间 GP 先验" vs "把 IMU 测量直接用于残差"，结论：用 LiDAR-惯性传感器套件时，把惯性信息作为**状态的测量**得到更好里程计精度。[659] 比较 [47] 的 GP 状态表示 vs [376] 的连续 GP 预积分（本章前面介绍）：在 event-based VIO 情境下，后者在精度与计算效率上略优于前者。

**仅本体感受里程计（Proprioception-only Odometry）**（源行 557）：近期工作用本体感受传感器做辅助惯性导航。里程计方面：[441]（腿式机器人）与 [813]（轮装 IMU）展示如何用系统运动学知识提供有竞争力的 IMU 里程计估计（**亚百分比位置误差**）。[441] 关键信息是机器人足-地接触；[813] 用单平面旋转运动约束 IMU 偏置、限制航位推算漂移。[813] 被扩展为完整 SLAM 系统 [1200]——通过识别道路倾斜角(road bank angle)随时间的模式检测回环，是 IMU 本体感受系统能做回环检测与校正的有趣例子。**注意**：虽惯性传感器通常提供更好性能与鲁棒性，但 IMU 的 dropout 或饱和可对系统整体性能有灾难性影响。[266] 研究陀螺饱和时用加速度计数据估计角速度，提升下游 SLAM 鲁棒性。

**仅惯性里程计（Inertial-only Odometry, IOO）**（源行 559）：无视觉等辅助源时朴素积分 IMU 测量通常致里程估计快速发散。即便辅助惯性里程计中、辅助源不可用时也是隐患。例如移动 AR/VR 手部跟踪中高动态手易移出相机 FOV、只剩 IMU 数据维持跟踪；或无纹理场景阻止特征检测跟踪、致 VIO 只能靠 IMU。故近期工作研究用**学习与神经网络**降低仅惯性里程计漂移 [1215,179,1055,451,452,220,898]：
- 用神经网络以数据驱动方式建模 IMU 偏置 [225]；
- 直接从噪声 IMU 测量序列预测位移 [682]；
- 用可微积分模块积分去除预测偏置后的 IMU 读数 [1276,898]；
- 用真值偏置监督 [123]；
- 用条件扩散模型(conditional diffusion model)近似建模为概率分布的偏置 [1298]。

这些方法证明可大幅降低仅惯性里程计漂移，但目前**泛化有限**（如对不同传感器或训练时未见运动的泛化）。

---

## 附：综合 agent 转换与补全清单（基于本源）

1. **记号重排**：本源误差态序为 **[θ, b^g, v, b^a, p, x_f]**（式 11.40）；本书若用 [p, v, φ, b_a, b_g] 须整体重排 F、Φ、M 各分块。旋转用右扰动 `R=\hat R Exp(δφ)`（与本书一致）；偏置上标 a/g 对应本书下标 a/g。
2. **OCR 勘误已标注 7 处**（式 11.4 的 `T_a→T_g`、式 11.5 的 `T_a→T_g`/`R_c^b→R_w^b`、式 11.7 排版与末行 b/w 混用、式 11.34 第二项 `b_i^a→b_j^a`、式 11.40 后 `Φ=FΦ→Φ̇=FΦ`、式 11.41 零块维数转置混排、式 11.51 中 `R_ℓ^w` 第三列）。综合时采用订正版。
3. **本源可直接成文的两大块**：① §11.2 IMU 预积分全套（11.8–11.34，含噪声协方差线性传播与偏置一阶修正）；② §11.3 可观性（11.37–11.51 + 表 11.1，4 维零空间 + 退化运动）。这两块推导**完整无跳步**，可原样进书。
4. **必须从其它源补的 VIO 内容**（本源仅文字或缺失）：
   - **VINS 滑窗 + 边缘化舒尔补的显式推导**（本源仅 §11.4.1 文字）；
   - **MSCKF 完整推导**（零空间投影、不在状态中保留特征、量测压缩 QR、OC/FEJ 一致性）——本源**无**；
   - **松耦合 vs 紧耦合的对比**——本源**无该术语对**，默认紧耦合因子图；
   - **视觉惯性联合初始化**（陀螺偏置标定、重力方向与尺度恢复、速度初始化、SfM bootstrap）——本源只有**纯 IMU 静止/地球自转初始对齐**（11.6–11.7），**不是 VIO 初始化**；
   - **滤波 vs 优化系统化对比表**——本源仅一句（fixed-lag vs iSAM2 的延迟保证权衡）。
5. **可直接引用的数值/性能锚点**：FEJ-WBA-VINS KAIST seq38 = 2.05°/21.2 m（0.18%，11.42 km 无回环）；好 VIO 漂移 <1%（可达 0.1%）；常局部加速度模型 EuRoC +5%；连续旋转预积分较离散 ≥1 数量级；扩展位姿海洋导航 1.8 km 后 ≈5 m；VR 延迟要求 10–50 ms、Quest 3 刷新 72–120 Hz；IMU 典型 200–1000 Hz。
