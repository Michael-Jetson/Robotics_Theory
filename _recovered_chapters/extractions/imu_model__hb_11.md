# 抽取留痕：SLAM Handbook 第 11 章《Inertial Odometry for SLAM（惯性里程计）》

> 本文件是【抽取层 / 全量保真留痕】，不是成书正文。目标：把 SLAM Handbook ch11 的每一步推导、每一道例题/数值例、每一条定义/定理/命题+证明、每一张表/分类/伪码，连同记号约定，全部完整记录，供后续综合 agent 改写为自包含书章。严格禁止摘要/凝练。
>
> 源文件：`/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/11_Inertial_Odometry_for_SLAM/11_Inertial_Odometry_for_SLAM.md`（共 561 行，已完整读取）
> 作者：Guoquan (Paul) Huang, Cédric Le Gentil, Teresa Vidal-Calleja, Davide Scaramuzza, Frank Dellaert, Luca Carlone
> 本抽取服务的章节：**IMU 模型与预积分**。重点覆盖：IMU 测量模型与误差模型；连续/离散运动学；预积分理论（SO(3)/SE(3) 流形）；bias 建模与一阶修正；协方差传播；与 VIO 的接口。

---

## 0. 记号约定（本源 vs 本书统一约定）

### 0.1 本源（Handbook ch11）使用的记号

| 记号 | 含义（源中定义） |
|---|---|
| $\mathcal{F}^b$ | 体（IMU）坐标系 body frame。源中假设传感器系与体系重合。 |
| $\mathcal{F}^w$ | 世界坐标系 world frame，在机器人学中固定于地球某处，**近似视为惯性系**（见源脚注 3：航空中区分 ECEF/LGV 非惯性系与 ECI 惯性系；机器人近地小尺度场景中地球自转影响相对噪声可忽略）。 |
| $\mathcal{F}^c$ | 相机/外感受传感器坐标系 sensor frame。 |
| $\mathbf{R}_b^w(t)$ | 将点从体系 $\mathcal{F}^b$ 映射到世界系 $\mathcal{F}^w$ 的旋转矩阵（即 body-to-world）。 |
| $\mathbf{R}_w^b(t)$ | 世界-to-体系旋转，满足 $\mathbf{R}_w^b(t)=(\mathbf{R}_b^w(t))^{\mathsf T}$。 |
| $\mathbf{p}^w(t)$, $\boldsymbol v^w(t)$ | IMU 在世界系下的位置、速度。 |
| $\{\mathbf{R}_b^w(t),\mathbf{p}^w(t)\}$ | 时刻 $t$ 的 IMU 位姿。 |
| $\mathbf{a}^w(t)\in\mathbb R^3$ | 传感器在世界系下的（运动学）加速度。 |
| $\mathbf{g}^w$ | 世界系下的重力向量。 |
| $\boldsymbol\omega_b^b(t)\in\mathbb R^3$ | $\mathcal{F}^b$ 相对 $\mathcal{F}^w$ 的瞬时角速度，在 $\mathcal{F}^b$ 中表达。 |
| $\mathbf{a}(t)$, $\boldsymbol\omega(t)$ | 加速度计 / 陀螺仪的**原始测量值**（在体系）。注意：在 11.1 节用 $\mathbf{a}(t),\boldsymbol\omega(t)$ 表测量；在 11.2 节预积分中**改用带 tilde 的 $\tilde{\mathbf a}_k,\tilde{\boldsymbol\omega}_k$** 表测量。 |
| $\mathbf{b}^a(t)$, $\mathbf{b}^g(t)$ | 加速度计偏置、陀螺仪偏置（上标 $a$/$g$ 指**传感器类型**，不是坐标系！源中特别强调这一点）。建模为随机游走（random walk）。 |
| $\boldsymbol\eta^a(t)$, $\boldsymbol\eta^g(t)$ | 加速度计、陀螺仪的连续时间零均值高斯白噪声。 |
| $\boldsymbol\eta^{ad}$, $\boldsymbol\eta^{gd}$ | 对应的**离散时间**噪声（上标 $d$ = discrete）。 |
| $\boldsymbol\eta^{ba}$, $\boldsymbol\eta^{bg}$ | 驱动偏置随机游走的连续时间白噪声。 |
| $\boldsymbol\eta^{bad}$, $\boldsymbol\eta^{bgd}$ | 偏置随机游走的离散时间噪声。 |
| $\mathbf T_a$ | 加速度计形状矩阵（shape matrix），建模失准（misalignment）与尺度（scale）误差。 |
| $\mathbf T_g$ | 陀螺仪形状矩阵（失准+尺度）。 |
| $\mathbf T_s$ | g-sensitivity 矩阵（陀螺仪对加速度的敏感）。 |
| $(\cdot)^\wedge$ | wedge 算子：把 $\mathbb R^3$ 向量映射到 $\mathfrak{so}(3)$ 反对称矩阵。 |
| $\mathrm{Exp}(\cdot)$, $\mathrm{Log}(\cdot)$ | SO(3) 的指数/对数映射（大写，从 $\mathbb R^3$ 到 SO(3) 及其逆）。 |
| $\mathbf J_r(\boldsymbol\phi)$, $\mathbf J_r^{-1}(\boldsymbol\phi)$ | SO(3) 右雅可比及其逆。 |
| $\Delta t$ | IMU 采样周期。 |
| $\Delta t_{ij}\doteq\sum_{k=i}^{j-1}\Delta t$ | 关键帧 $i$ 到 $j$ 的总时间。 |
| $(\cdot)_i\doteq(\cdot)(t_i)$ | 时刻 $t_i$ 的量。 |
| $\Delta\mathbf R_{ij},\Delta\boldsymbol v_{ij},\Delta\boldsymbol p_{ij}$ | 相对运动增量（含噪、含状态），下标 $ij$ 表 $i\to j$。 |
| $\Delta\tilde{\mathbf R}_{ij},\Delta\tilde{\boldsymbol v}_{ij},\Delta\tilde{\boldsymbol p}_{ij}$ | **预积分测量**（带 tilde，是从测量直接算出、与 $i$ 时刻状态及重力无关的量）。 |
| $\delta\boldsymbol\phi_{ij},\delta\boldsymbol v_{ij},\delta\boldsymbol p_{ij}$ | 预积分测量的噪声。 |
| $\boldsymbol\eta_{ij}^\Delta\doteq[\delta\boldsymbol\phi_{ij}^{\mathsf T},\delta\boldsymbol v_{ij}^{\mathsf T},\delta\boldsymbol p_{ij}^{\mathsf T}]^{\mathsf T}$ | 9 维预积分噪声向量。 |
| $\boldsymbol\Sigma_{ij}$ | 预积分测量协方差（$9\times9$）。 |
| $\bar{\mathbf b}_i^a,\bar{\mathbf b}_i^g$ | 预积分时所用的偏置估计（bar = 线性化点/积分时刻的值）。 |
| $\delta\mathbf b$ | 优化中偏置的小增量更新。 |
| $\mathbf r_{\mathcal I_{ij}}=[\mathbf r_{\Delta R_{ij}}^{\mathsf T},\mathbf r_{\Delta v_{ij}}^{\mathsf T},\mathbf r_{\Delta p_{ij}}^{\mathsf T}]^{\mathsf T}\in\mathbb R^9$ | 预积分 IMU 因子的残差。 |
| $\tilde{\boldsymbol x}$ | 误差状态（error-state），相对线性化点的偏差。 |
| $\tilde{\boldsymbol\theta}$ | 旋转的切空间误差表示。 |
| $\mathbf F_c(t),\mathbf G_c(t)$ | 连续时间线性化转移矩阵、噪声雅可比（IMU 状态部分）。 |
| $\boldsymbol\Phi_{(k+1,k)}$ | 离散时间状态转移矩阵 $t_k\to t_{k+1}$。 |
| $\mathbf M(\hat{\boldsymbol x})$ | 可观测性矩阵。 |
| $\mathcal U$ | 可观测性矩阵零空间（不可观子空间）。 |
| $\boldsymbol\omega_{ie}$ | 地球自转角速率。 |

### 0.2 与本书统一约定的差异（供综合时转换）

- **旋转记号**：本源用 **R**（与本书统一的 $R\in SO(3)$ 一致）。源用 $\mathbf R_b^w$ 表 body-to-world（主用此方向）。本书统一也用 R，无需改字母。
- **扰动方向（左/右扰动）**：本源的 SO(3) 一阶展开主要用**右扰动/右雅可比** $\mathbf J_r$。关键证据：
  - 公式 (11.16) $\mathrm{Exp}(\boldsymbol\phi+\delta\boldsymbol\phi)\approx\mathrm{Exp}(\boldsymbol\phi)\mathrm{Exp}(\mathbf J_r(\boldsymbol\phi)\delta\boldsymbol\phi)$（右乘扰动）。
  - 预积分把噪声"移到末尾"成 $\Delta\tilde{\mathbf R}_{ij}\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})$（右乘）。
  - **例外**：在 11.3 可观测性分析中（脚注 8）用的是**左扰动**：$\mathbf R_b^w=\hat{\mathbf R}_b^w\,\mathbf R_b^w(\tilde{\boldsymbol\theta})\simeq\hat{\mathbf R}_b^w(\mathbf I+\tilde{\boldsymbol\theta}^\wedge)$。这里 $\hat{\mathbf R}_b^w$ 是 body-to-world，$\hat{\mathbf R}_b^w(\mathbf I+\tilde\theta^\wedge)$ 形式上是在 body-to-world 矩阵右侧乘 $(\mathbf I+\tilde\theta^\wedge)$，即体系内的扰动（**body-frame/right perturbation on body-to-world**）。综合时需注意：预积分章主用右扰动，与本书一致。
- **$\xi=[\rho;\phi]$ 排序**：本源**不使用** SE(3) 的 6 维 $\xi$ 堆叠记号；预积分在 SO(3)×$\mathbb R^3$×$\mathbb R^3$ 上分块进行（rotation, velocity, position 三块），9 维噪声排序为 $[\delta\boldsymbol\phi;\delta\boldsymbol v;\delta\boldsymbol p]$（先旋转后平移类）。误差状态排序为 $\{\tilde{\boldsymbol\theta},\tilde{\mathbf b}^g,\tilde{\boldsymbol v}^w,\tilde{\mathbf b}^a,\tilde{\boldsymbol p}^w,\tilde{\boldsymbol x}_f^w\}$（注意 bias 交错排在 rotation 后、velocity 前等，见 (11.40) 附近）。综合时若要套本书 $\xi=[\rho;\phi]$ 顺序需重排。
- **四元数**：本源**全程不用四元数**，旋转一律用旋转矩阵 R + SO(3) 指数映射。故无 Hamilton/JPL 之分。本书统一 Hamilton 四元数，转换时与本源无冲突（本源未涉及）。
- **协方差字母**：本源用 $\boldsymbol\Sigma$（如 $\boldsymbol\Sigma_{ij}$、$\boldsymbol\Sigma^{bgd}$、$\boldsymbol\Sigma^{bad}$）。与本书一致。
- **重力符号**：源中加速度计模型为 $\mathbf a=\mathbf R_w^b(\mathbf a^w-\mathbf g^w)+\dots$，即测量 = 旋转·(运动加速度 − 重力)。注意这里 $\mathbf g^w$ 是重力向量本身（不是 −9.81 那种带符号约定的差异，综合时需核对 g 的正负号与"specific force"定义）。
- **离散噪声-连续噪声关系**：源给出 $\mathrm{Cov}(\boldsymbol\eta^{gd}(t))=\frac{1}{\Delta t}\mathrm{Cov}(\boldsymbol\eta^g(t))$（同理 $\eta^{ad}$）。偏置随机游走则 $\boldsymbol\Sigma^{bgd}=\Delta t_{ij}\mathrm{Cov}(\boldsymbol\eta^{bg})$。注意白噪声离散化是除以 $\Delta t$，随机游走是乘以 $\Delta t$，方向相反（综合时勿混）。

---

## 11.（章首）引言

IMU 已成为机器人 SLAM 中最普遍的里程计来源之一。IMU 测量其所附着体的**线加速度**与**旋转速率**。IMU 形态、成本、性能跨度极大：从飞机上又大又准的光学传感器，到手机等消费设备里又小又噪的 MEMS（微机电系统）。MEMS IMU 低 SWAP（尺寸/重量/功耗）且廉价，是机器人传感器的优良候选，在 SLAM 中已被研究二十余年。

本章结构：
- 11.1 IMU 基础与测量模型；
- 11.2 IMU 预积分（把高频 IMU 数据加入因子图优化框架）；
- 11.3 使用 IMU 引入额外变量（如偏置）的可观测性分析（IMU + 外感受传感器如相机/LiDAR）；
- 11.4 现代 IMU 为核心的 SLAM 系统示例；
- 11.5 近期趋势。

> 源脚注 1：可观测性（Observability）确立估计问题在何条件下良态，即给定测量是否有可能算出接近真值的估计。

---

## 11.1 惯性感知与导航基础（Basics of Inertial Sensing and Navigation）

6 轴 IMU = 加速度计（测传感器相对惯性系的线加速度）+ 陀螺仪（测传感器的角速度/旋转速率）。

> 源脚注 2：IMU 通常还含罗盘（测磁北方向），但在 SLAM 中较少用——许多机器人应用（含室内、城市环境）中罗盘有大偏置，由局部磁干扰（如大金属结构、电子设备）引起。

惯性导航系统（INS）传统上在航空航天研究，目标是从初始状态和 IMU 测量历史估计平台当前状态（位姿、速度）。INS 分类：
- **捷联系统（strapdown systems）**：IMU 刚性安装在平台框架上。
- **稳定系统（stabilized systems）**：IMU 装在内万向节/多万向节结构/浮动球上，设计用以相对惯性系保持其朝向恒定。

机器人中的 INS 多属前者（捷联），即依赖刚连平台、测局部线加速度和角速度的 IMU。机器人中"**inertial odometry（惯性里程计）**"常作为惯性导航的同义词，强调估计的里程计性质。

INS 的里程计估计随时间漂移，故多数应用还依赖其他传感器（GPS、相机、LiDAR），此时称**辅助惯性导航系统（AINS, aided INS）**。机器人中常直接指明 IMU 搭配的传感器组合：相机+IMU 做 3D 运动跟踪 = **视觉惯性里程计（VIO）**；若再加闭环 = **视觉惯性 SLAM**。

### 11.1.1 感知原理与测量模型（Sensing Principles and Measurement Models）

IMU 通常含 3 轴加速度计 + 3 轴陀螺仪。
- 陀螺仪设计基本原理：**角动量守恒**。
- 加速度计：用质量块的惯性测量"相对惯性系的运动学加速度"与"重力加速度"之差。设计原理多样：摆式质量上的速率陀螺、低摩擦壳体内检验质量的惯性、或壳内两条悬挂金属薄带（间挂检验质量）的振动差。

#### 测量模型（Measurement Model）【源 11.1.1】

简化假设：传感器系与体系 $\mathcal{F}^b$ 重合，世界系 $\mathcal{F}^w$ 为惯性系。时刻 $t$ 的 IMU 测量 $\mathbf a(t),\boldsymbol\omega(t)$ 被假设受**加性高斯白噪声 $\boldsymbol\eta$** 和**缓变偏置 b** 污染：

$$\mathbf{a}(t) = \mathbf{R}_w^b(t) \left( \mathbf{a}^w(t) - \mathbf{g}^w \right) + \mathbf{b}^a(t) + \boldsymbol{\eta}^a(t), \tag{11.1}$$

$$\boldsymbol\omega(t) = \boldsymbol\omega_b^b(t) + \mathbf{b}^g(t) + \boldsymbol\eta^g(t). \tag{11.2}$$

说明（逐项）：
- 上标 $b$ 表该量在体系 $\mathcal{F}^b$ 中表达。
- IMU 时刻 $t$ 位姿 $\{\mathbf R_b^w(t),\mathbf p^w(t)\}$ 把点从 $\mathcal{F}^b$ 映到 $\mathcal{F}^w$；$\mathbf R_w^b(t)=(\mathbf R_b^w(t))^{\mathsf T}$。
- $\mathbf a^w(t)\in\mathbb R^3$ 是传感器在世界系下的加速度；$\mathbf g^w$ 是世界系重力向量。
- 因此 $\mathbf R_w^b(t)(\mathbf a^w(t)-\mathbf g^w)$ 是 IMU 在体/IMU 系中**实际感受到的加速度**（specific force）。
- $\boldsymbol\omega_b^b(t)\in\mathbb R^3$ 是 $\mathcal{F}^b$ 相对 $\mathcal{F}^w$ 的瞬时角速度，在 $\mathcal{F}^b$ 中表达。
- $\boldsymbol\eta^g(t),\boldsymbol\eta^a(t)$ 为零均值高斯随机变量；待估偏置 $\mathbf b^a(t),\mathbf b^g(t)$ 服从随机游走。
- **重要记号提醒**：噪声/偏置上标指**传感器**（加速度计 a、陀螺 g），不是坐标系；如 $\mathbf b^a(t)$ 是加速度计偏置。

> 源脚注 3（已并入记号约定 0.1）：航空区分非惯性系（ECEF、LGV）与惯性系（ECI）；机器人近地小尺度常用噪声大的低成本 IMU，地球自转影响相对测量噪声可忽略，故世界系 $\mathcal{F}^w$（固定于地球某处）近似当作惯性系处理。

#### 扩展模型（Extended Models）【源 11.1.1】

标准模型 (11.1)-(11.2) 在机器人中常已够用；但 (重)标定传感器平台时需更精细模型。

**加速度计失准+尺度误差**（制造缺陷导致），扩展为：

$$\mathbf{a}(t) = \mathbf{T}_a \ \mathbf{R}_w^b(t) \left( \mathbf{a}^w(t) - \mathbf{g}^w \right) + \mathbf{b}^a(t) + \boldsymbol{\eta}^a(t), \tag{11.3}$$

其中 $\mathbf T_a$ 是形状矩阵，建模加速度计测量的失准与尺度误差。尺度误差可由静态分量或温度相关分量组成，可在传感器（内参）标定中确定。

**陀螺仪失准+尺度误差**：

$$\boldsymbol\omega(t) = \mathbf{T}_g \, \boldsymbol\omega_b^b(t) + \mathbf{b}^g(t) + \boldsymbol\eta^g(t), \tag{11.4}$$

其中 $\mathbf T_g$ 是陀螺仪形状矩阵（失准+尺度）。

> 抽取注：源 (11.4) 处的 OCR 写成 "$\mathbf T_a$"，但正文明确说"$\mathbf T_g$ is the shape matrix ... gyroscope"，应为 $\mathbf T_g$，此处已按正文订正。

**g-sensitivity（陀螺受加速度影响）**：若该影响幅度在加性白噪声 $\boldsymbol\eta^g(t)$ 范围内则视为可忽略；某些 MEMS 硬件中更显著，建模为：

$$\boldsymbol\omega(t) = \mathbf{T}_g \, \boldsymbol\omega_b^b(t) + \mathbf{T}_s \, \mathbf{R}_w^b(t) \, (\mathbf{a}^w(t) - \mathbf{g}^w) + \mathbf{b}^g(t) + \boldsymbol\eta^g(t), \tag{11.5}$$

其中 $\mathbf T_s$ 是 g-sensitivity 矩阵，可在标定中估计。

> 抽取注：源 (11.5) OCR 写 "$\mathbf T_a$ $\boldsymbol\omega_b^b$" 和 "$\mathbf R_c^b$"，按正文语义应分别为 $\mathbf T_g\boldsymbol\omega_b^b$ 与 $\mathbf R_w^b$（与 (11.3) 的 specific-force 项一致），已订正。

### 11.1.2 初始对准（Initial Alignment）【源 11.1.2】

SLAM 中习惯把全局坐标系设为轨迹起始位姿，即初始位姿 $\{\mathbf R_b^w(0),\mathbf p^w(0)\}$ 设为单位位姿。但在 INS 中，因 IMU 测量涉及重力（见 (11.1)），通常选**重力对齐**的世界系，故需把初始位姿与重力方向对齐。即：IMU 测量依赖重力方向，机器人朝向不再是任意选择，必须与重力方向一致。具体需算出把体（IMU）系对齐到世界系的旋转 $\mathbf R_b^w(0)$。

**静态初始化假设**：部署之初机器人静止，无 specific force 作用，常用低成本 MEMS IMU 只测得重力。仅给定局部重力测量 $\mathbf g^b$，**无法恢复绕重力的旋转（即 yaw 偏航）**，yaw 可按应用自由选取。但可由如下静态初始化确定 roll 与 pitch 对应的旋转：

$$\begin{cases}
\boldsymbol{z}_{w}^{b} = \dfrac{\mathbf{g}^{b}}{\lVert\mathbf{g}^{b}\rVert} \\[2mm]
\boldsymbol{x}_{w}^{b} = \dfrac{\mathbf{e}_{1} - \boldsymbol{z}_{w}^{b}\, \mathbf{e}_{1}^{\top} \boldsymbol{z}_{w}^{b}}{\lVert\mathbf{e}_{1} - \boldsymbol{z}_{w}^{b}\, \mathbf{e}_{1}^{\top} \boldsymbol{z}_{w}^{b}\rVert} \\[2mm]
\boldsymbol{y}_{w}^{b} = \boldsymbol{z}_{w}^{b} \times \boldsymbol{x}_{w}^{b}
\end{cases}
\quad\Rightarrow\quad
\boldsymbol{R}_{w}^{b} = \begin{bmatrix} \boldsymbol{x}_{w}^{b} & \boldsymbol{y}_{w}^{b} & \boldsymbol{z}_{w}^{b} \end{bmatrix}
\tag{11.6}$$

这里对向量 $\mathbf e_1=[1\ 0\ 0]^{\top}$ 与 $\mathbf g^b$ 做 **Gram-Schmidt 正交归一化**，$\times$ 是叉积。

直观解释：旋转矩阵 $\mathbf R_w^b$ 的最后一列 $\mathbf z_w^b$ 是世界系 z 轴相对体系的方向。世界系 z 轴与重力对齐，故 (11.6) 从体系重力向量 $\mathbf g^b$ 的测量算出 $\mathbf z_w^b$；正交归一化过程算出正交单位向量 $\mathbf x_w^b,\mathbf y_w^b$ 以补全旋转矩阵 $\mathbf R_w^b$ 的列，yaw 任意选取。

#### 高端 IMU 的对准（Alignment with High-end IMUs）

高端 IMU 的陀螺足够灵敏，能测出地球自转速率 $\boldsymbol\omega_{ie}$。此时假设所选世界系是惯性系（如 ECI），可用体系重力向量 $\mathbf g^b$ 与地球自转速率 $\boldsymbol\omega_{ie}$ 做解析对准：

$$\begin{cases}
\mathbf{g}^{b} = \mathbf{R}_{w}^{b}\, \mathbf{g}^{w} \\
\boldsymbol{\omega}_{ie}^{b} = \mathbf{R}_{w}^{b}\, \boldsymbol{\omega}_{ie}^{w} \\
\mathbf{g}^{b} \times \boldsymbol{\omega}_{ie}^{b} = \mathbf{R}_{w}^{b} (\mathbf{g}^{w} \times \boldsymbol{\omega}_{ie}^{w})
\end{cases}
\Rightarrow\ 
\mathbf{R}_{b}^{w} = \begin{bmatrix} \mathbf{g}^{w\top} \\ \boldsymbol{\omega}_{ie}^{w\top} \\ (\mathbf{g}^{w} \times \boldsymbol{\omega}_{ie}^{w})^{\top} \end{bmatrix}^{-1} \begin{bmatrix} \mathbf{g}^{b\top} \\ \boldsymbol{\omega}_{ie}^{b\top} \\ (\mathbf{g}^{b} \times \boldsymbol{\omega}_{ie}^{b})^{\top} \end{bmatrix}
\tag{11.7}$$

其中所得旋转矩阵 $\mathbf R_b^w$ 通常需投影回 SO(3) 以减弱测量噪声影响。

> 抽取注：源 (11.7) 的 OCR 把右侧第三行写成 $(\mathbf g^b\times\boldsymbol\omega_{ie}^w)^{\top}$，与左侧约束 $\mathbf g^b\times\boldsymbol\omega_{ie}^b$ 不一致，按三条约束的自洽性应为 $(\mathbf g^b\times\boldsymbol\omega_{ie}^b)^{\top}$，已订正。原理：三组对应向量 $\{\mathbf g,\boldsymbol\omega_{ie},\mathbf g\times\boldsymbol\omega_{ie}\}$ 在两系下由同一 $\mathbf R_w^b$ 关联，堆叠求逆即得旋转。

---

## 11.2 IMU 预积分与因子图（IMU Preintegration and Factor Graphs）【源 11.2】

动机：(11.1)-(11.2) 把 IMU 测量与机器人状态（位姿、速度）及偏置关联。原则上可据此推一个第 1 章式的 MAP 估计器，但会导致**不实用的巨大因子图**：典型 IMU 高频（200–1000 Hz）输出，测量模型要求每个 IMU 采样时刻向因子图加状态，因子图迅速变得无法求解。

更敏锐的读者会想：连续时间表述能否绕过高频加变量？答案：连续时间惯性导航表述中仍需高频加因子（每个测量一个），同样导致笨重因子图。

**IMU 预积分（IMU preintegration）核心思想**：避免以 IMU 频率向因子图加状态或测量。基本想法：把 IMU 测量随时间积分得到**相对运动测量**，这样（更少的）运动测量可加入因子图。但朴素积分（11.2.1 复习）仍需在求解器每次迭代时重做积分（因积分初条件可能变）。预积分通过**分离依赖状态变量的项与测量项**来避免此问题。

- 预积分原始思想：源参考 [709]；
- 流形上的扩展：源参考 [334, 335]；
- 11.2.2 紧随 [334, 335] 的表述；11.2.3 讨论更高级技术；近期工作放在 11.5。

### 11.2.1 运动积分（Motion Integration）【源 11.2.1】

从 IMU 测量推断机器人运动。引入运动学模型（源参考 [790, 318]）：

$$\dot{\mathbf{R}}_b^w = \mathbf{R}_b^w \, (\boldsymbol{\omega}_b^b)^{\wedge}, \qquad \dot{\boldsymbol{v}}^w = \mathbf{a}^w, \qquad \dot{\boldsymbol{p}}^w = \boldsymbol{v}^w, \tag{11.8}$$

描述体系 $\mathcal{F}^b$ 相对世界系 $\mathcal{F}^w$ 的旋转 $\mathbf R_b^w$、平移 $\mathbf p^w$、速度 $\mathbf v^w$ 的演化。

时刻 $t+\Delta t$（$\Delta t$ 为 IMU 采样周期）的状态由对 (11.8) 积分得到：

$$\mathbf{R}_{b}^{w}(t + \Delta t) = \mathbf{R}_{b}^{w}(t)\, \mathrm{Exp}\!\left(\int_{t}^{t + \Delta t} \boldsymbol{\omega}_{b}^{b}(\tau)\, d\tau\right)$$

$$\mathbf{v}^{w}(t + \Delta t) = \mathbf{v}^{w}(t) + \int_{t}^{t + \Delta t} \mathbf{a}^{w}(\tau)\, d\tau$$

$$\mathbf{p}^{w}(t + \Delta t) = \mathbf{p}^{w}(t) + \int_{t}^{t + \Delta t} \mathbf{v}^{w}(\tau)\, d\tau
\tag{11.9--11.10}$$

其中第一式假设角速度 $\boldsymbol\omega_b^b$ 的**方向**在区间 $[t,t+\Delta t]$ 内不变（源脚注 4：更一般的旋转积分见 (11.35)）。

进一步假设 $\mathbf a^w$ 与 $\boldsymbol\omega_b^b$ 在 $[t,t+\Delta t]$ 内**保持常值**，可写：

$$\mathbf{R}_{b}^{w}(t+\Delta t) = \mathbf{R}_{b}^{w}(t)\, \mathrm{Exp}\!\left(\boldsymbol{\omega}_{b}^{b}(t)\,\Delta t\right)$$

$$\mathbf{v}^{w}(t+\Delta t) = \mathbf{v}^{w}(t) + \mathbf{a}^{w}(t)\,\Delta t$$

$$\mathbf{p}^{w}(t+\Delta t) = \mathbf{p}^{w}(t) + \mathbf{v}^{w}(t)\,\Delta t + \tfrac{1}{2}\mathbf{a}^{w}(t)\,\Delta t^{2}.
\tag{11.11}$$

更一般地，(11.11) 可理解为对 (11.9) 的积分施加**欧拉积分**数值求解。

用 (11.1)-(11.2) 把 $\mathbf a^w$ 与 $\boldsymbol\omega_b^b$ 写成 IMU 测量的函数，(11.11) 变为（下文起省略坐标系下标）：

$$\mathbf{R}(t + \Delta t) = \mathbf{R}(t)\, \mathrm{Exp}\!\left(\left(\tilde{\boldsymbol{\omega}}(t) - \mathbf{b}^{g}(t) - \boldsymbol{\eta}^{gd}(t)\right) \Delta t\right)$$

$$\mathbf{v}(t + \Delta t) = \mathbf{v}(t) + \mathbf{g}\,\Delta t + \mathbf{R}(t) \left(\tilde{\mathbf{a}}(t) - \mathbf{b}^{a}(t) - \boldsymbol{\eta}^{ad}(t)\right) \Delta t$$

$$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}(t)\,\Delta t + \tfrac{1}{2}\mathbf{g}\,\Delta t^{2} + \tfrac{1}{2}\mathbf{R}(t) \left(\tilde{\mathbf{a}}(t) - \mathbf{b}^{a}(t) - \boldsymbol{\eta}^{ad}(t)\right) \Delta t^{2}
\tag{11.12}$$

说明：
- 此处 $\tilde{\boldsymbol\omega}(t),\tilde{\mathbf a}(t)$ 为 IMU 原始测量（带 tilde），与 (11.1)-(11.2) 中 $\boldsymbol\omega,\mathbf a$ 同义但改用 tilde 表"测量"。从 (11.1) 解出 $\mathbf a^w=\mathbf R(\tilde{\mathbf a}-\mathbf b^a-\boldsymbol\eta^{ad})+\mathbf g$，从 (11.2) 解出 $\boldsymbol\omega_b^b=\tilde{\boldsymbol\omega}-\mathbf b^g-\boldsymbol\eta^{gd}$，代入即得。
- 此速度、位置的数值积分在两测量间假设**常值朝向 $\mathbf R(t)$**，对非零旋转速率的测量，这不是 (11.8) 的精确解；实践中高频 IMU 缓解此近似的影响。
- 采用 (11.12) 因其简单、便于建模与不确定性传播；更高级积分见 11.2.3。
- **离散-连续噪声协方差关系**：$\mathrm{Cov}(\boldsymbol\eta^{gd}(t))=\frac{1}{\Delta t}\mathrm{Cov}(\boldsymbol\eta^{g}(t))$，$\eta^{ad}$ 同理（源参考 [232, Appendix]）。

(11.12) 虽可直接看作因子图中的概率约束，但需高频加状态：它关联 $t$ 与 $t+\Delta t$ 的状态，$\Delta t$ 是 IMU 采样周期，故每来一个新 IMU 测量就要加新状态（源参考 [503]）。

> 源脚注 4：旋转积分的更一般表达见 (11.35)。

#### 跨关键帧积分（避免高频加状态的第一步尝试）

**图 11.1**（源 `_page_6_Figure_2.jpeg`）：IMU 与相机的不同频率示意（来自 [335], ©2016 IEEE）。

设已有一因子图建模问题中其他传感器测量（如第 7 章视觉测量），可用 (11.12) 在因子图中两个**时间上相邻的状态**之间积分 IMU 测量。这些状态称"**关键帧状态（keyframe states）**"。对关键帧时刻 $t_i,t_j$ 之间所有 $\Delta t$ 区间迭代 IMU 积分 (11.12)，得：

$$\mathbf{R}_{j} = \mathbf{R}_{i} \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( \left( \tilde{\boldsymbol{\omega}}_{k} - \mathbf{b}_{k}^{g} - \boldsymbol{\eta}_{k}^{gd} \right) \Delta t \right),$$

$$\mathbf{v}_{j} = \mathbf{v}_{i} + \mathbf{g}\, \Delta t_{ij} + \sum_{k=i}^{j-1} \mathbf{R}_{k} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t$$

$$\mathbf{p}_{j} = \mathbf{p}_{i} + \sum_{k=i}^{j-1} \left[ \mathbf{v}_{k}\, \Delta t + \tfrac{1}{2} \mathbf{g}\, \Delta t^{2} + \tfrac{1}{2} \mathbf{R}_{k} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t^{2} \right]
\tag{11.13}$$

其中引入简写 $\Delta t_{ij}\doteq\sum_{k=i}^{j-1}\Delta t$ 与 $(\cdot)_i\doteq(\cdot)(t_i)$。

(11.13) 已给出 $t_i,t_j$ 间运动估计，但缺点是：当 $t_i$ 处线性化点改变（如 Gauss-Newton 求解器每次迭代）时，(11.13) 的积分必须重做（源参考 [645]）。例如 $\mathbf R_i$ 一变，所有未来旋转 $\mathbf R_k,k=i,\dots,j-1$ 都变，必须重算 (11.13) 中的求和与连乘。

> 源脚注 5：用"keyframe（关键帧）"术语，因为在很多 IMU+相机应用中，因子图的状态加在相机帧的子集（即关键帧，见第 7 章）。但此术语此处不失一般性，可任意决定何时实例化关键帧状态（如每个相机帧、每个 LiDAR 扫描、每 n 个 IMU 测量……）。
>
> 源脚注 6：为简化，假设 IMU 与其他传感器同步，IMU 测量恰在 $t_i,t_j$ 采样。实践中可插值近似该情形；时间同步进一步讨论见 11.4.3。

### 11.2.2 流形上的 IMU 预积分（IMU Preintegration on Manifold）【源 11.2.2】

核心洞见：对 (11.13) 做小变形，可算出 $t_i,t_j$ 间的相对测量，使其在线性化点改变时**无需重算**。关键：把测量表达在**局部系**（使其不随机器人全局状态估计改变而变），并**隔离重力贡献**（重力携带全局系信息）。这样得到所谓**预积分 IMU 测量**，它约束因子图中相邻状态间的运动。

为此，重排 (11.13) 的项，定义如下**与 $t_i$ 处位姿、速度无关**的相对运动增量：

$$\Delta \mathbf{R}_{ij} \doteq \mathbf{R}_{i}^{\mathsf{T}} \mathbf{R}_{j} = \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( \left( \tilde{\boldsymbol{\omega}}_{k} - \mathbf{b}_{k}^{g} - \boldsymbol{\eta}_{k}^{gd} \right) \Delta t \right)$$

$$\Delta \boldsymbol{v}_{ij} \doteq \mathbf{R}_{i}^{\mathsf{T}} \left( \boldsymbol{v}_{j} - \boldsymbol{v}_{i} - \mathbf{g}\, \Delta t_{ij} \right) = \sum_{k=i}^{j-1} \Delta \mathbf{R}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t$$

$$\Delta \boldsymbol{p}_{ij} \doteq \mathbf{R}_{i}^{\mathsf{T}} \left( \boldsymbol{p}_{j} - \boldsymbol{p}_{i} - \boldsymbol{v}_{i}\, \Delta t_{ij} - \tfrac{1}{2} \mathbf{g}\, \Delta t_{ij}^{2} \right)
= \sum_{k=i}^{j-1} \left[ \Delta \boldsymbol{v}_{ik}\, \Delta t + \tfrac{1}{2} \Delta \mathbf{R}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{k}^{a} - \boldsymbol{\eta}_{k}^{ad} \right) \Delta t^{2} \right]
\tag{11.14}$$

其中 $\Delta\mathbf R_{ik}\doteq\mathbf R_i^{\mathsf T}\mathbf R_k$，$\Delta\mathbf v_{ik}\doteq\mathbf R_i^{\mathsf T}(\mathbf v_k-\mathbf v_i-\mathbf g\Delta t_{ik})$。

**关键强调**：与"delta"旋转 $\Delta\mathbf R_{ij}$ 不同，$\Delta\mathbf v_{ij}$ 和 $\Delta\mathbf p_{ij}$ **都不对应速度/位置的真实物理变化**，而是定义成使 (11.14) 右端独立于 $i$ 时刻状态以及重力效应。事实上，可直接从两关键帧间的惯性测量算出 (11.14) 右端。

遗留问题：(11.14) 的求和与连乘仍是**偏置估计的函数**。分两步解决：
- 11.2.2.1：假设 $\mathbf b_i$ 已知；
- 11.2.2.3：展示偏置估计变化时如何避免重做积分。

两种情形都假设偏置在 $t_i,t_j$ 间**保持常值**：

$$\mathbf{b}_{i}^{g} = \mathbf{b}_{i+1}^{g} = \dots = \mathbf{b}_{j-1}^{g}, \quad \mathbf{b}_{i}^{a} = \mathbf{b}_{i+1}^{a} = \dots = \mathbf{b}_{j-1}^{a}.
\tag{11.15}$$

#### 11.2.2.1 预积分 IMU 测量（Preintegrated IMU Measurements）【源 11.2.2.1】

(11.14) 把关键帧 $i,j$ 状态（左端）与测量（右端）关联，已可看作测量模型。但它对测量噪声的依赖相当复杂，直接做 MAP 估计有困难（MAP 需清楚定义测量密度及其对数似然）。本节变形 (11.14) 以便推导测量对数似然——具体是**隔离各惯性测量的噪声项**。本节内假设 $t_i$ 处偏置已知。

**先处理旋转增量 $\Delta\mathbf R_{ij}$。** 用如下 SO(3) 指数映射性质（见第 2 章）：

$$\mathrm{Exp}(\boldsymbol\phi + \delta\boldsymbol\phi) \approx \mathrm{Exp}(\boldsymbol\phi)\, \mathrm{Exp}(\mathbf J_r(\boldsymbol\phi)\,\delta\boldsymbol\phi), \tag{11.16}$$

$$\mathrm{Exp}(\boldsymbol\phi)\, \mathbf{R} = \mathbf{R}\, \mathrm{Exp}(\mathbf{R}^{\mathsf{T}} \boldsymbol\phi). \tag{11.17}$$

其中 (11.16) 是指数之和的一阶近似，(11.17) 由群的伴随表示（adjoint）导出。

用 (11.16)、(11.17) 重排 $\Delta\mathbf R_{ij}$，把噪声"移到末尾"：

$$\Delta \mathbf{R}_{ij} \stackrel{(11.16)}{\simeq} \prod_{k=i}^{j-1} \left[ \mathrm{Exp}\!\left( \left( \tilde{\boldsymbol{\omega}}_{k} - \mathbf{b}_{i}^{g} \right) \Delta t \right) \mathrm{Exp}\!\left( -\mathbf{J}_{r}^{k} \boldsymbol{\eta}_{k}^{gd}\, \Delta t \right) \right]$$

$$\stackrel{(11.17)}{=} \Delta \tilde{\mathbf{R}}_{ij} \prod_{k=i}^{j-1} \mathrm{Exp}\!\left( -\Delta \tilde{\mathbf{R}}_{k+1\,j}^{\mathsf{T}}\, \mathbf{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t \right)$$

$$\stackrel{\dot{=}}{=} \Delta \tilde{\mathbf{R}}_{ij}\, \mathrm{Exp}\!\left( -\delta \boldsymbol{\phi}_{ij} \right)
\tag{11.18}$$

其中 $\mathbf J_r^k\doteq\mathbf J_r^k((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t)$。

推导细节（逐步）：
- 第一步用 (11.16)：把 $\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g-\boldsymbol\eta_k^{gd})\Delta t)$ 中的小噪声 $-\boldsymbol\eta_k^{gd}\Delta t$ 视为 $\delta\boldsymbol\phi$，分裂为 $\mathrm{Exp}((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t)\,\mathrm{Exp}(-\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t)$（注意此处把 (11.14) 里随 $k$ 变的 $\mathbf b_k^g$ 用常值 $\mathbf b_i^g$ 替换，依 (11.15)）。
- 第二步用 (11.17) 把所有噪声项 $\mathrm{Exp}(-\mathbf J_r^k\boldsymbol\eta_k^{gd}\Delta t)$ 逐一"穿过"右侧的无噪旋转连乘，移到整个连乘末尾，每穿过一次产生 $\Delta\tilde{\mathbf R}_{k+1\,j}^{\mathsf T}$ 的伴随旋转。
- 最后一行**定义预积分旋转测量**：$\displaystyle\Delta\tilde{\mathbf R}_{ij}\doteq\prod_{k=i}^{j-1}\mathrm{Exp}\!\big((\tilde{\boldsymbol\omega}_k-\mathbf b_i^g)\Delta t\big)$ 及其噪声 $\delta\boldsymbol\phi_{ij}$（下节分析）。

**再处理速度、位置。** 用如下关系：

$$\exp(\boldsymbol\phi^{\wedge}) \approx \mathbf{I} + \boldsymbol\phi^{\wedge}, \tag{11.19}$$

$$\mathbf{a}^{\wedge} \mathbf{b} = -\mathbf{b}^{\wedge} \mathbf{a}, \quad \forall\, \mathbf{a}, \mathbf{b} \in \mathbb{R}^3, \tag{11.20}$$

(11.19) 是指数映射在原点的一阶近似，(11.20) 是向量 wedge 算子的性质（反对称交换）。

把 (11.18) 代回 (11.14) 中 $\Delta\mathbf v_{ij}$ 的表达，对 $\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})$ 用一阶近似 (11.19)，丢弃高阶噪声项：

$$\Delta \boldsymbol{v}_{ij} \stackrel{(11.19)}{\simeq} \sum_{k=i}^{j-1} \Delta \tilde{\boldsymbol{R}}_{ik} (\mathbf{I} - \delta \boldsymbol{\phi}_{ik}^{\wedge}) \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right) \Delta t \;-\; \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t$$

$$\stackrel{(11.20)}{=} \Delta \tilde{\boldsymbol{v}}_{ij} + \sum_{k=i}^{j-1} \left[ \Delta \tilde{\boldsymbol{R}}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t - \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t \right]$$

$$\stackrel{\dot{=}}{=} \Delta \tilde{\boldsymbol{v}}_{ij} - \delta \boldsymbol{v}_{ij}
\tag{11.21}$$

其中**定义预积分速度测量** $\displaystyle\Delta\tilde{\mathbf v}_{ij}\doteq\sum_{k=i}^{j-1}\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t$ 及其噪声 $\delta\mathbf v_{ij}$。

推导细节：第二步用 (11.20) 把 $-\delta\boldsymbol\phi_{ik}^\wedge(\tilde{\mathbf a}_k-\mathbf b_i^a)$ 改写为 $+(\tilde{\mathbf a}_k-\mathbf b_i^a)^\wedge\delta\boldsymbol\phi_{ik}$（即 $\boldsymbol a^\wedge\boldsymbol b=-\boldsymbol b^\wedge\boldsymbol a$，把对 $\delta\boldsymbol\phi$ 线性的项提出）。

类似地，把 (11.18) 和 (11.21) 代入 (11.14) 中 $\Delta\mathbf p_{ij}$ 的表达，用一阶近似 (11.19)：

$$\Delta \boldsymbol{p}_{ij} \stackrel{(11.19)}{\simeq} \sum_{k=i}^{j-1} \left[ (\Delta \tilde{\boldsymbol{v}}_{ik} - \delta \boldsymbol{v}_{ik}) \Delta t + \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik} (\mathbf{I} - \delta \boldsymbol{\phi}_{ik}^{\wedge}) \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right) \Delta t^{2} - \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t^{2} \right]$$

$$\stackrel{(11.20)}{=} \Delta \tilde{\boldsymbol{p}}_{ij} + \sum_{k=i}^{j-1} \left[ -\delta \boldsymbol{v}_{ik}\, \Delta t + \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik} \left( \tilde{\mathbf{a}}_{k} - \mathbf{b}_{i}^{a} \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t^{2} - \tfrac{1}{2} \Delta \tilde{\boldsymbol{R}}_{ik}\, \boldsymbol{\eta}_{k}^{ad}\, \Delta t^{2} \right]$$

$$\stackrel{\dot{=}}{=} \Delta \tilde{\boldsymbol{p}}_{ij} - \delta \boldsymbol{p}_{ij}
\tag{11.22}$$

其中**定义预积分位置测量** $\Delta\tilde{\mathbf p}_{ij}$ 及其噪声 $\delta\mathbf p_{ij}$。（$\Delta\tilde{\mathbf p}_{ij}=\sum_{k=i}^{j-1}[\Delta\tilde{\mathbf v}_{ik}\Delta t+\frac12\Delta\tilde{\mathbf R}_{ik}(\tilde{\mathbf a}_k-\mathbf b_i^a)\Delta t^2]$。）

**预积分测量模型（合成结果）。** 把 (11.18)、(11.21)、(11.22) 代回 (11.14) 中 $\Delta\mathbf R_{ij},\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$ 的原始定义（记住 $\mathrm{Exp}(-\delta\boldsymbol\phi_{ij})^{\mathsf T}=\mathrm{Exp}(\delta\boldsymbol\phi_{ij})$），最终得：

$$\Delta \tilde{\mathbf{R}}_{ij} = \mathbf{R}_{i}^{\mathsf{T}} \mathbf{R}_{j}\, \mathrm{Exp}\!\left( \delta \boldsymbol\phi_{ij} \right)$$

$$\Delta \tilde{\mathbf{v}}_{ij} = \mathbf{R}_{i}^{\mathsf{T}} \left( \mathbf{v}_{j} - \mathbf{v}_{i} - \mathbf{g}\, \Delta t_{ij} \right) + \delta \mathbf{v}_{ij}$$

$$\Delta \tilde{\mathbf{p}}_{ij} = \mathbf{R}_{i}^{\mathsf{T}} \left( \mathbf{p}_{j} - \mathbf{p}_{i} - \mathbf{v}_{i}\, \Delta t_{ij} - \tfrac{1}{2} \mathbf{g}\, \Delta t_{ij}^{2} \right) + \delta \mathbf{p}_{ij}
\tag{11.23}$$

复合测量写成（待估）状态 "加" 随机噪声的形式，噪声由随机向量 $[\delta\boldsymbol\phi_{ij}^{\mathsf T},\delta\mathbf v_{ij}^{\mathsf T},\delta\mathbf p_{ij}^{\mathsf T}]^{\mathsf T}$ 描述。

小结：本节把测量模型 (11.14) 变形重写为 (11.23)。(11.23) 的优点：对合适的噪声分布，可直接用于在因子图中实例化 $t_i,t_j$ 间的因子。噪声项性质见下节。

#### 11.2.2.2 噪声传播（Noise Propagation）【源 11.2.2.2】

推导噪声向量 $[\delta\boldsymbol\phi_{ij}^{\mathsf T},\delta\mathbf v_{ij}^{\mathsf T},\delta\mathbf p_{ij}^{\mathsf T}]^{\mathsf T}$ 的统计量。把噪声近似为零均值正态分布很方便，但准确建模噪声协方差至关重要。本节给出预积分测量协方差 $\boldsymbol\Sigma_{ij}$ 的推导：

$$\boldsymbol{\eta}_{ij}^{\Delta} \doteq [\delta \boldsymbol{\phi}_{ij}^{\mathsf{T}},\ \delta \boldsymbol{v}_{ij}^{\mathsf{T}},\ \delta \boldsymbol{p}_{ij}^{\mathsf{T}}]^{\mathsf{T}} \sim \mathcal{N}(\mathbf{0}_{9 \times 1},\ \boldsymbol{\Sigma}_{ij}).
\tag{11.24}$$

**预积分旋转噪声 $\delta\boldsymbol\phi_{ij}$。** 由 (11.18)：

$$\mathrm{Exp}\!\left(-\delta \boldsymbol{\phi}_{ij}\right) \doteq \prod_{k=i}^{j-1} \mathrm{Exp}\!\left(-\Delta \tilde{\boldsymbol{R}}_{k+1\,j}^{\mathsf{T}}\, \mathbf{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t\right). \tag{11.25}$$

两边取 Log 并变号：

$$\delta \boldsymbol\phi_{ij} = -\mathrm{Log}\!\left(\prod_{k=i}^{j-1} \mathrm{Exp}\!\left(-\Delta \tilde{\boldsymbol{R}}_{k+1\,j}^{\mathsf{T}}\, \boldsymbol{J}_{r}^{k}\, \boldsymbol{\eta}_{k}^{gd}\, \Delta t\right)\right). \tag{11.26}$$

用 SO(3) 对数的一阶近似：

$$\mathrm{Log}(\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\delta\boldsymbol\phi)) \approx \boldsymbol\phi + \mathbf J_r^{-1}(\boldsymbol\phi)\,\delta\boldsymbol\phi. \tag{11.27}$$

其中 $\mathbf J_r^{-1}(\boldsymbol\phi)$ 是右雅可比之逆。重复应用 (11.27)（注意 $\boldsymbol\eta_k^{gd}$ 及 $\delta\boldsymbol\phi_{ij}$ 都是小旋转噪声，故右雅可比近似单位阵）得：

$$\delta \boldsymbol\phi_{ij} \simeq \sum_{k=i}^{j-1} \Delta \tilde{\boldsymbol R}_{k+1\,j}^{\mathsf{T}}\, \mathbf J_r^k\, \boldsymbol{\eta}_k^{gd}\, \Delta t \tag{11.28}$$

至一阶，$\delta\boldsymbol\phi_{ij}$ 是零均值高斯（零均值噪声 $\boldsymbol\eta_k^{gd}$ 的线性组合）。

**速度、位置噪声 $\delta\mathbf v_{ij},\delta\mathbf p_{ij}$。** 它们是加速度噪声 $\boldsymbol\eta_k^{ad}$ 与预积分旋转噪声 $\delta\boldsymbol\phi_{ij}$ 的线性组合，故也零均值高斯。简单整理得：

$$\delta \mathbf{v}_{ij} \simeq \sum_{k=i}^{j-1} \left[ -\Delta \tilde{\mathbf{R}}_{ik} \left( \tilde{\mathbf{a}}_k - \mathbf{b}_i^a \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t + \Delta \tilde{\mathbf{R}}_{ik}\, \boldsymbol{\eta}_k^{ad}\, \Delta t \right]$$

$$\delta \mathbf{p}_{ij} \simeq \sum_{k=i}^{j-1} \left[ \delta \mathbf{v}_{ik}\, \Delta t - \tfrac{1}{2} \Delta \tilde{\mathbf{R}}_{ik} \left( \tilde{\mathbf{a}}_k - \mathbf{b}_i^a \right)^{\wedge} \delta \boldsymbol{\phi}_{ik}\, \Delta t^2 + \tfrac{1}{2} \Delta \tilde{\mathbf{R}}_{ik}\, \boldsymbol{\eta}_k^{ad}\, \Delta t^2 \right]
\tag{11.29}$$

（关系均至一阶有效。）

(11.28)-(11.29) 把预积分噪声 $\boldsymbol\eta_{ij}^\Delta$ 表为 IMU 测量噪声 $\boldsymbol\eta_k^d\doteq[\boldsymbol\eta_k^{gd},\boldsymbol\eta_k^{ad}],\,k=1,\dots,j-1$ 的线性函数。因此，由 $\boldsymbol\eta_k^d$ 的协方差（IMU 规格书给出），可通过简单的**线性传播**计算 $\boldsymbol\eta_{ij}^\Delta$ 的协方差 $\boldsymbol\Sigma_{ij}$。

噪声传播的扩展推导见 [335]，其中还给出**迭代式**：随新测量到来增量计算协方差。迭代计算表达更简单、更便于在线推断。

#### 11.2.2.3 纳入偏置更新（Incorporating Bias Updates）【源 11.2.2.3】

上一节假设预积分（$k=i$ 到 $k=j$）所用偏置 $\{\bar{\mathbf b}_i^a,\bar{\mathbf b}_i^g\}$ 正确且不变。但更可能的是优化中偏置估计变化一个小量 $\delta\mathbf b$。一种方案是偏置变时重算 delta 测量，但计算昂贵。改为：给定偏置更新 $\mathbf b\leftarrow\bar{\mathbf b}+\delta\mathbf b$，用**一阶展开**更新 delta 测量：

$$\Delta \tilde{\mathbf{R}}_{ij}(\mathbf{b}_{i}^{g}) \simeq \Delta \tilde{\mathbf{R}}_{ij}(\bar{\mathbf{b}}_{i}^{g})\, \mathrm{Exp}\!\left(\frac{\partial \Delta \bar{\mathbf{R}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g}\right)$$

$$\Delta \tilde{\mathbf{v}}_{ij}(\mathbf{b}_{i}^{g}, \mathbf{b}_{i}^{a}) \simeq \Delta \tilde{\mathbf{v}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}_{i}^{g} + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}_{i}^{a}$$

$$\Delta \tilde{\mathbf{p}}_{ij}(\mathbf{b}_{i}^{g}, \mathbf{b}_{i}^{a}) \simeq \Delta \tilde{\mathbf{p}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}_{i}^{g} + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}_{i}^{a}
\tag{11.30}$$

这与 [709] 的偏置修正类似，但**直接在 SO(3) 上操作**（旋转用右乘 $\mathrm{Exp}$ 修正，速度/位置用加法修正）。

雅可比 $\left\{\frac{\partial\Delta\bar{\mathbf R}_{ij}}{\partial\mathbf b^g},\frac{\partial\Delta\bar{\mathbf v}_{ij}}{\partial\mathbf b^g},\dots\right\}$（在积分时刻偏置估计 $\bar{\mathbf b}_i$ 处计算）描述测量随偏置估计变化的方式。这些雅可比**保持常值，可在预积分时预计算**。其推导与 11.2.2.1 中"把测量表为大值加小扰动"的方法非常类似，详见 [335]。

> 注：(11.30) 中 $\frac{\partial\Delta\bar{(\cdot)}}{\partial\mathbf b}$ 的 bar 表示在 $\bar{\mathbf b}_i$ 处求值的预积分量对偏置的偏导（5 个独立雅可比：旋转对 $\mathbf b^g$ 一个；速度对 $\mathbf b^g,\mathbf b^a$ 各一；位置对 $\mathbf b^g,\mathbf b^a$ 各一。旋转不依赖 $\mathbf b^a$，因 (11.14) 旋转项只含陀螺）。

#### 11.2.2.4 预积分 IMU 因子与偏置模型（Preintegrated IMU Factors and Bias Models）【源 11.2.2.4】

由预积分测量模型 (11.23) 和（一阶下）零均值高斯噪声（协方差 $\boldsymbol\Sigma_{ij}$，见 (11.24)），易写出将出现在因子图优化中的残差 $\mathbf r_{\mathcal I_{ij}}\doteq[\mathbf r_{\Delta R_{ij}}^{\mathsf T},\mathbf r_{\Delta v_{ij}}^{\mathsf T},\mathbf r_{\Delta p_{ij}}^{\mathsf T}]^{\mathsf T}\in\mathbb R^9$：

$$\mathbf r_{\Delta \mathbf{R}_{ij}} \doteq \mathrm{Log}\!\left(\left(\Delta \tilde{\mathbf{R}}_{ij}(\bar{\mathbf{b}}_{i}^{g})\, \mathrm{Exp}\!\left(\frac{\partial \Delta \bar{\mathbf{R}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g}\right)\right)^{\mathsf{T}} \mathbf{R}_{i}^{\mathsf{T}} \mathbf{R}_{j}\right)$$

$$\mathbf r_{\Delta \mathbf{v}_{ij}} \doteq \mathbf{R}_{i}^{\mathsf{T}}\!\left(\mathbf{v}_{j} - \mathbf{v}_{i} - \mathbf{g}\,\Delta t_{ij}\right) - \left[\Delta \tilde{\mathbf{v}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g} + \frac{\partial \Delta \bar{\mathbf{v}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}^{a}\right]$$

$$\mathbf r_{\Delta \mathbf{p}_{ij}} \doteq \mathbf{R}_{i}^{\mathsf{T}}\!\left(\mathbf{p}_{j} - \mathbf{p}_{i} - \mathbf{v}_{i}\,\Delta t_{ij} - \tfrac{1}{2}\mathbf{g}\,\Delta t_{ij}^{2}\right) - \left[\Delta \tilde{\mathbf{p}}_{ij}(\bar{\mathbf{b}}_{i}^{g}, \bar{\mathbf{b}}_{i}^{a}) + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{g}}\, \delta \mathbf{b}^{g} + \frac{\partial \Delta \bar{\mathbf{p}}_{ij}}{\partial \mathbf{b}^{a}}\, \delta \mathbf{b}^{a}\right]
\tag{11.31}$$

其中也已纳入 (11.30) 的偏置更新。这些项可直接加入因子图：向最小化目标加 $\lVert\mathbf r_{\mathcal I_{ij}}\rVert_{\boldsymbol\Sigma_{ij}}^2$（马氏范数）。

**偏置模型。** 偏置是缓变量，用"**布朗运动（Brownian motion）**"建模，即积分白噪声：

$$\dot{\mathbf{b}}^g(t) = \boldsymbol{\eta}^{bg}, \qquad \dot{\mathbf{b}}^a(t) = \boldsymbol{\eta}^{ba}. \tag{11.32}$$

在相邻关键帧 $[t_i,t_j]$ 上积分 (11.32)：

$$\mathbf{b}_{j}^{g} = \mathbf{b}_{i}^{g} + \boldsymbol{\eta}^{bgd}, \qquad \mathbf{b}_{j}^{a} = \mathbf{b}_{i}^{a} + \boldsymbol{\eta}^{bad}, \tag{11.33}$$

其中沿用简写 $\mathbf b_i^g\doteq\mathbf b^g(t_i)$，离散噪声 $\boldsymbol\eta^{bgd},\boldsymbol\eta^{bad}$ 零均值，协方差分别为：

$$\boldsymbol\Sigma^{bgd}\doteq\Delta t_{ij}\,\mathrm{Cov}(\boldsymbol\eta^{bg}), \qquad \boldsymbol\Sigma^{bad}\doteq\Delta t_{ij}\,\mathrm{Cov}(\boldsymbol\eta^{ba})$$

（源参考 [232, Appendix]）。

模型 (11.33) 可直接加入因子图，作为目标函数对所有相邻关键帧的额外加性项：

$$\lVert\mathbf{r}_{\mathbf{b}_{ij}}\rVert^2 \doteq \lVert\mathbf{b}_j^g - \mathbf{b}_i^g\rVert_{\boldsymbol{\Sigma}^{bgd}}^2 + \lVert\mathbf{b}_j^a - \mathbf{b}_i^a\rVert_{\boldsymbol{\Sigma}^{bad}}^2
\tag{11.34}$$

> 抽取注：源 (11.34) OCR 第二项写成 $\lVert\mathbf b_i^a-\mathbf b_i^a\rVert$（两项都是 $i$），按物理意义与第一项对称，应为 $\lVert\mathbf b_j^a-\mathbf b_i^a\rVert$，已订正。

### 11.2.3 高级预积分技术（Advanced Preintegration Techniques）【源 11.2.3】

本节看标准预积分的局限并探索更新的替代方案。先看 (11.14) 中隐含的信号与运动假设，再过一遍各种放松这些假设、得到更精确预积分测量（从而提升辅助惯性导航定位建图精度）的工作。注意：不详述各方法推导，请读者参阅相应论文。

#### 11.2.3.1 数值积分精度（Numerical Integration Accuracy）【源 11.2.3.1】

标准预积分用**欧拉法**把惯性信号在离散时刻积分为旋转、速度、位置伪测量。该法快、高效，但引入积分误差（即漂移）。欧拉法本质是对信号施加**矩形法则**数值求积分；如**图 11.2（左）**所示，信号被以给定频率采样的**分段常值块**近似（在惯性系统中样本即加速度计/陀螺测量）。

采样频率较低时，分段常值假设不能准确表示输入信号，双重积分迅速累积误差（图 11.2 右上）。一个变通是提高信号采样频率（图 11.2 下），但实际惯性导航问题中采样频率受惯性传感器硬件特性限制。

[631] 提出用 **GP（高斯过程）回归**虚拟上采样输入信号至任意时间戳（陀螺、加速度计都可）。这改进了标准预积分，但仍基于分段常值假设做数值积分，未充分利用 GP 模型的连续性。

> 源脚注 7：GP 回归是非参数概率插值方法，深入理解见 [909]。

**图 11.2**（源 `_page_13_Figure_2.jpeg`）：已知初条件下，低（上行）与高（下行）采样频率的欧拉积分示例。

#### 11.2.3.2 连续加速度预积分（Continuous Acceleration Preintegration）【源 11.2.3.2】

另一减小积分误差的途径：利用**连续时间表示**（不限于离散时间戳），更好近似真实惯性信号并做**解析积分**。除精度提升外，连续时间表示还允许**异步查询**预积分测量——这在与非硬件同步或完全异步采样的其他传感器（如事件相机）做惯性辅助状态估计时尤其有用。

预积分的难点是处理旋转空间：旋转操作的**非交换性**使许多经典 Riemann 积分工具不可用。多项工作将预积分的旋转部分与平移部分**解耦**。本小节先探讨平移部分（假设旋转积分已解决），旋转的连续积分留待下一小节。

[300]：先用零阶积分器 [1105] 积分陀螺测量，再给出速度、位置预积分测量的连续表述——通过求解连续时间微分方程组（LTV，线性时变），假设**加速度计测量常值**或**局部加速度常值**（两种模型见 [300]）。对比假设**全局加速度常值**的标准预积分 [335]，[300] 证明**局部加速度常值假设更贴近真实场景**，在 EuRoC 数据集 [131] 上 VIO 整体精度比标准预积分和常值加速度计测量模型都改善约 **5%**。

为放松常值加速度运动模型假设，可用**可解析积分的函数**近似输入数据。假设旋转部分已解决，[632] 把"**旋转修正后的加速度计测量** $\hat{\mathbf a}_k$"（定义 $\hat{\mathbf a}_k=\Delta\mathbf R_{ik}\tilde{\mathbf a}_k$）以连续方式表示。

**图 11.3**（源 `_page_14_Figure_2.jpeg`）：上行——分段线性近似的连续积分（对应**常值 jerk（加加速度）运动假设**）；下行——用 GP 回归的无模型积分。

- **分段线性近似**：第一重积分（从 $\hat{\mathbf a}_k$ 到 $\Delta\mathbf v_{ik}$）对应经典**梯形法则**数值积分。该模型可解读为**常值 jerk 运动模型**，相比欧拉法已有显著精度增益。
- **GP 回归（无模型积分）**：用 $\hat{\mathbf a}\sim\mathcal{GP}(0,k_a(t,t')\mathbf I)$，$k_a(t,t')$ 为**平方指数协方差核**；通过对 GP 施加**线性算子** [969] 解析推断 $\hat{\mathbf a}$ 的积分（及双重积分）。由于平方指数核无限可微，该法不依赖任何显式运动模型。图 11.3 下行显示非参数 GP 模型相比分段线性法的积分精度改善。核的超参控制信号平滑度，可从数据学习或凭经验设定。

#### 11.2.3.3 连续旋转预积分（Continuous Rotation Preintegration）【源 11.2.3.3】

看到连续表示对平移、速度预积分的精度增益，自然想扩展到旋转。但在旋转空间积分有挑战：旋转 $\mathbf R$ 属 SO(3) 李群，非欧氏空间，群运算交换性不成立。实际上，求解运动学模型 (11.8) 的**乘积积分（product integral）**：

$$\mathbf{R}_{b}^{w}(t+\Delta t) = \mathbf{R}_{b}^{w}(t) \prod_{\tau=t}^{t+\Delta t} \mathrm{Exp}\!\left(\boldsymbol{\omega}_{b}^{b}(\tau)\right)^{d\tau}
\tag{11.35}$$

**没有已知的通用解** [109]，需新方法做连续无模型旋转积分。

[630] 提出：用李代数中的**旋转向量表示** $\mathbf r(t)$（满足 $\mathbf R(t)=\mathrm{Exp}(\mathbf r(t))$）作为线性向量空间，用线性工具做连续积分。该空间中系统动力学为：

$$\dot{\mathbf{r}} = (\mathbf{J}_r(\mathbf{r}))^{-1}\, \boldsymbol{\omega}_b^b, \tag{11.36}$$

其中 $\mathbf J_r(\mathbf r)$ 是 SO(3) 在 $\mathbf r$ 处求值的右雅可比。

困难：$\mathbf r$ 与 $\dot{\mathbf r}$ 都不被 IMU 直接观测。[630] 的关键思想：用 GP 建模 $\dot{\mathbf r}$，并用一组**虚拟观测** $\dot{\mathbf r}_{t_\bullet}$（可解读为连续旋转动力学的**控制点**）通过对 GP 施加线性算子表示连续旋转向量函数 $\mathbf r$。这些虚拟观测由非线性最小二乘优化估计——残差基于 (11.36)、以陀螺测量作为 $\boldsymbol\omega_b^b$ 的观测。结果是连续旋转预积分的无模型方法，精度比标准离散预积分至少改善**一个数量级**。

该连续方法与 [47] 的 **STEAM 连续时间状态估计**（第 2 章提及）有许多相似：两者都在李代数中做 GP 插值。主要差别：本法用平方指数核导致**稠密线性系统**，而 STEAM 用**稀疏 Markov** 方法。不过对 IMU 预积分而言，积分窗口一般足够短，解稠密系统不成问题。[376] 把"优化诱导值（optimized inducing values）"概念扩展到同时估计旋转修正加速度与旋转向量，从而能**关联预积分测量协方差矩阵的旋转与平移部分**。

---

## 11.3 辅助惯性导航的可观测性（Observability of Aided Inertial Navigation）【源 11.3】

由于测量噪声、偏置、数值积分误差，纯惯性里程计可能快速漂移（尤其用低保真惯性传感器）。常用降漂法：把 IMU 与外感受传感器（相机、LiDAR）配对，得 AINS。引入外感受传感器常增大待估状态（如加外部地标变量），故自然要问：**传感器数据是否足以无歧义估计系统 SLAM 状态？** 这是**可观测性分析（observability analysis）**的目标——确定可用测量提供的信息是否足以无歧义估计状态/参数（源参考 [117, 453]）。

可观测性分析通常做法：推导线性化测量模型、计算**可观测性矩阵**，它与状态估计的 **Fisher 信息（及协方差）矩阵**密切相关（源参考 [487, 485]，见第 6 章）。系统可观测 ⟺ 可观测性矩阵满秩；若不满秩，因该矩阵描述测量中可用的信息，研究其**零空间**可洞察状态空间中估计器信息不足的方向。

可观测性分析结果用途：
- 改善估计一致性（consistency）[1234, 456, 653]；
- 确定初始化估计器所需的最少测量 [456, 737]；
- 识别导致额外不可观方向、应避免或（若可能）告警的**退化运动** [1235]。

为此大量研究投入 AINS 可观测性分析 [1234]，尤其视觉惯性系统（如 [457, 654, 1235]）。

本节讨论用以辅助 IMU 的传感器产生**几何特征**（点、线、面）时的可观测性。这一般化处理覆盖广泛传感器（相机、LiDAR），并理解退化配置。结构：11.3.1 线性化模型（假设外感受测量几何地标）；11.3.2 可观测性分析；11.3.3 退化配置。

### 11.3.1 线性化测量模型（Linearized Measurement Models）【源 11.3.1】

假设辅助 IMU 的传感器（相机、LiDAR）产生几何特征，即聚焦产生基于地标表示的 SLAM/里程计前端。多数 AINS 用点特征（尤其相机，如 [456, 653, 645, 896, 375, 335]），有线、面特征时也可用（如 [599, 455, 414, 1236]）。此时可能需用所有这些几何特征扩充待估状态向量。

AINS 待估状态（每时间步）含机器人状态 $\boldsymbol x_b$ 与外部特征状态 $\boldsymbol x_f^w$（世界系表达）：

$$\boldsymbol{x} = \{\boldsymbol{R}_b^w,\ \mathbf{b}^g,\ \boldsymbol{v}^w,\ \mathbf{b}^a,\ \boldsymbol{p}^w,\ \boldsymbol{x}_f^w\}
\tag{11.37}$$

其中 $\mathbf R_b^w$ 是体系相对世界系的旋转，$\mathbf p^w,\mathbf v^w$ 是机器人在世界系的位置、速度，$\mathbf b^g,\mathbf b^a$ 是体系中的陀螺、加速度计偏置；特征 $\boldsymbol x_f^w$ 可为点/线/面（或组合），世界系表达。

可观测性分析既需系统动力学模型（关联 IMU 加速度、角速率测量），也需外感受测量模型。下面先复习 INS 运动学模型（基于前节 IMU 方程），再考虑外感受测量方程。

#### 11.3.1.1 线性化 IMU 运动学模型（Linearized IMU Kinematic Model）【源 11.3.1.1】

IMU 运动学模型（基于 (11.8) 与 (11.32)）：

$$\dot{\mathbf{R}}_b^w = \mathbf{R}_b^w (\boldsymbol{\omega}_b^b)^{\wedge}, \quad \dot{\mathbf{v}}^w = \mathbf{a}^w, \quad \dot{\mathbf{p}}^w = \mathbf{v}^w,$$

$$\dot{\mathbf{b}}^g(t) = \boldsymbol{\eta}^{bg}, \quad \dot{\mathbf{b}}^a(t) = \boldsymbol{\eta}^{ba}
\tag{11.38--11.39}$$

其中 $\boldsymbol\eta^{bg},\boldsymbol\eta^{ba}$ 是驱动陀螺、加速度计偏置（建模为随机游走）的零均值高斯噪声。

为做可观测性分析，线性化上述非线性系统，得**连续时间线性化误差状态动力学系统**：

$$\dot{\tilde{\boldsymbol{x}}}(t) \simeq \begin{bmatrix} \mathbf{F}_c(t) & \mathbf{0}_{15 \times n_f} \\ \mathbf{0}_{n_f \times 15} & \mathbf{0}_{n_f} \end{bmatrix} \tilde{\boldsymbol{x}}(t) + \begin{bmatrix} \mathbf{G}_c(t) \\ \mathbf{0}_{n_f \times 12} \end{bmatrix} \boldsymbol{\eta}(t) =:\ \mathbf{F}(t)\, \tilde{\boldsymbol{x}}(t) + \mathbf{G}(t)\, \boldsymbol{\eta}(t)
\tag{11.40}$$

其中误差状态向量 $\tilde{\boldsymbol x}=\{\tilde{\boldsymbol\theta},\tilde{\mathbf b}^g,\tilde{\boldsymbol v}^w,\tilde{\mathbf b}^a,\tilde{\boldsymbol p}^w,\tilde{\boldsymbol x}_f^w\}$（写成列向量）表相对线性化点的偏差（如 $\tilde{\mathbf b}^g$ 是偏置相对线性化点的变化）；旋转分量用线性化点处的切空间表示 $\tilde{\boldsymbol\theta}$。$n_f$ 是 $\tilde{\boldsymbol x}_f^w$ 的维数；$\mathbf F_c(t),\mathbf G_c(t)$ 是 IMU 状态的连续时间线性化转移矩阵与噪声雅可比；$\boldsymbol\eta(t)$ 是堆叠噪声，含 $\boldsymbol\eta^{bg},\boldsymbol\eta^{ba}$ 以及把 (11.38) 中实际加速度/旋转率替换为加速度计/陀螺测量时产生的 IMU 噪声（见 11.2.1 推导）。

> 源脚注 8（旋转线性化，**左扰动**）：把任意旋转 $\mathbf R_b^w$ 写成线性化点旋转 $\hat{\mathbf R}_b^w$ 的扰动：$\mathbf R_b^w=\hat{\mathbf R}_b^w\,\mathbf R_b^w(\tilde{\boldsymbol\theta})$，$\tilde{\boldsymbol\theta}$ 是合适的切空间向量。再用小角近似线性化：$\mathbf R_b^w=\hat{\mathbf R}_b^w\,\mathbf R_b^w(\tilde{\boldsymbol\theta})\simeq\hat{\mathbf R}_b^w(\mathbf I+\tilde{\boldsymbol\theta}^\wedge)$。

AINS 估计器实际多在离散时间实现，需离散时间动力学模型：由其状态转移矩阵 $\boldsymbol\Phi_{(k+1,k)}$（$t_k\to t_{k+1}$）给出，满足 $\dot{\boldsymbol\Phi}_{(k+1,k)}=\mathbf F(t_k)\boldsymbol\Phi_{(k+1,k)}$，初条件为单位阵：

$$\boldsymbol\Phi_{(k+1,k)} = \begin{bmatrix}
\boldsymbol\Phi_{11} & \boldsymbol\Phi_{12} & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_{3\times n_f} \\
\mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_{3\times n_f} \\
\boldsymbol\Phi_{31} & \boldsymbol\Phi_{32} & \mathbf{I}_3 & \boldsymbol\Phi_{34} & \mathbf{0}_3 & \mathbf{0}_{3\times n_f} \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_{3\times n_f} \\
\boldsymbol\Phi_{51} & \boldsymbol\Phi_{52} & \boldsymbol\Phi_{53} & \boldsymbol\Phi_{54} & \mathbf{I}_3 & \mathbf{0}_{3\times n_f} \\
\mathbf{0}_{n_f\times 3} & \mathbf{0}_{n_f\times 3} & \mathbf{0}_{n_f\times 3} & \mathbf{0}_{n_f\times 3} & \mathbf{0}_{n_f\times 3} & \mathbf{I}_{n_f}
\end{bmatrix}
\tag{11.41}$$

其中 $(i,j)$ 块 $\boldsymbol\Phi_{ij}$ 可解析或数值求得（源参考 [456]）。

> 抽取注：源 (11.41) OCR 把最后一行块与 $n_f$ 维块的行列标注略有错位（如把若干 $\mathbf 0_{n_f\times3}$ 写在上方块行的最后一列），上面已按"状态排序 $\{\tilde\theta,\tilde{\mathbf b}^g,\tilde{\mathbf v},\tilde{\mathbf b}^a,\tilde{\mathbf p},\tilde{\mathbf x}_f\}$、特征块为 $\mathbf I_{n_f}$、各 IMU 块对特征的耦合为 0"的结构整理。块结构解读：行/列 1=旋转误差 $\tilde\theta$，2=$\tilde{\mathbf b}^g$，3=$\tilde{\mathbf v}$，4=$\tilde{\mathbf b}^a$，5=$\tilde{\mathbf p}$，6=特征。偏置行（2、4）只有对角 $\mathbf I_3$（随机游走，转移为单位）；特征行只有对角 $\mathbf I_{n_f}$（静态地标，$\mathbf F$ 中对应块为 0）。$\dot{\boldsymbol\Phi}=\mathbf F\boldsymbol\Phi$ 应理解为转移矩阵满足的微分方程（源 OCR 此处写成 $\boldsymbol\Phi=\mathbf F\boldsymbol\Phi$，缺了点号/积分语义）。

#### 11.3.1.2 外感受测量模型（Exteroceptive Measurement Models）【源 11.3.1.2】

给出不同几何特征的测量模型及其线性化，是 AINS 可观测性分析所必需。

**点特征（Point Features）。** 外感受传感器（单目/双目相机、声呐、LiDAR）提供的点特征测量，一般建模为**距离（range）和/或方位（bearing）**观测，是特征在传感器系 $\mathcal{F}^c$ 中相对位置的函数：

$$\boldsymbol{z}_{p} = \underbrace{\begin{bmatrix} \lambda_{r} & \mathbf{0}_{1 \times 2} \\ \mathbf{0}_{2 \times 1} & \lambda_{b}\, \mathbf{I}_{2} \end{bmatrix}}_{\boldsymbol{\Lambda}} \begin{bmatrix} z_{r} \\ \boldsymbol{z}_{b} \end{bmatrix} = \boldsymbol{\Lambda} \begin{bmatrix} \lVert\boldsymbol{p}_{f}^{c}\rVert + \eta^{r} \\ h_{b}\!\left(\boldsymbol{p}_{f}^{c}\right) + \boldsymbol{\eta}^{b} \end{bmatrix}
\tag{11.42}$$

其中 $\boldsymbol p_f^c=\mathbf R_w^c(\boldsymbol p_f^w-\boldsymbol p_c^w)$ 是特征在传感器系的位置；$z_r,z_b$ 分别是距离、方位测量；$h_b(\cdot)$ 是通用方位测量函数，具体形式依传感器；$\boldsymbol\Lambda$ 是测量选择矩阵，二元项 $\lambda_r,\lambda_b$（如 $\lambda_b=1,\lambda_r=1$ 则 $\boldsymbol z_p$ 含距离+方位）；$\eta^r,\boldsymbol\eta^b$ 是测量噪声（简化为加性）。

用链式法则在当前状态估计处线性化 (11.42)，得测量误差方程：

$$\tilde{\boldsymbol{z}}_{p} = \boldsymbol{z}_{p} - \hat{\boldsymbol{z}}_{p} \simeq \boldsymbol{\Lambda} \begin{bmatrix} \frac{\partial z_{r}}{\partial \boldsymbol{p}_{f}^{c}} \frac{\partial \boldsymbol p_{f}^{c}}{\partial \boldsymbol{x}} \big|_{\hat{\boldsymbol{x}}}\, \tilde{\boldsymbol{x}} + \eta^{r} \\ \frac{\partial \boldsymbol{z}_{b}}{\partial \boldsymbol{p}_{f}^{c}} \frac{\partial \boldsymbol p_{f}^{c}}{\partial \boldsymbol{x}} \big|_{\hat{\boldsymbol{x}}}\, \tilde{\boldsymbol{x}} + \boldsymbol{\eta}^{b} \end{bmatrix} =:\ \boldsymbol{\Lambda} \begin{bmatrix} \mathbf{H}_{r} \\ \mathbf{H}_{b} \end{bmatrix} \mathbf{H}_{f}\, \tilde{\boldsymbol{x}} + \boldsymbol{\Lambda} \begin{bmatrix} \eta^{r} \\ \boldsymbol{\eta}^{b} \end{bmatrix} =:\ \mathbf{H}_{x}\, \tilde{\boldsymbol{x}} + \boldsymbol{\eta}^{p}
\tag{11.43}$$

其中 $\hat{\boldsymbol z}_p$ 是线性化点测量。依选择矩阵 $\boldsymbol\Lambda$，雅可比 $\mathbf H_x$ 可含：仅距离雅可比 $\mathbf H_r$（$\lambda_r=1,\lambda_b=0$）、仅方位雅可比 $\mathbf H_b$（$\lambda_r=0,\lambda_b=1$）、或两者。

**线特征（Line Features）。** 给定两 3D 点 $\mathbf p_1^w,\mathbf p_2^w$，用 **Plücker 坐标**表示过两点的直线：

$$\mathbf{l}^w = \begin{bmatrix} \mathbf{n}_{\ell}^w \\ \mathbf{v}_{\ell}^w \end{bmatrix} = \begin{bmatrix} \mathbf{p}_1^w \times \mathbf{p}_2^w \\ \mathbf{p}_2^w - \mathbf{p}_1^w \end{bmatrix}
\tag{11.44}$$

其中 $\mathbf n_\ell^w$ 是**线矩（line moment）**，编码由两点与原点定义的平面的法向；$\mathbf v_\ell^w$ 是**线方向向量**（需要时可归一化为单位向量）。原点到直线的距离 $d_\ell^w=\frac{\lVert\mathbf n_\ell^w\rVert}{\lVert\mathbf v_\ell^w\rVert}$。世界系 Plücker 坐标变换到相机系（源参考 [1021]）：

$$\begin{bmatrix} \mathbf{n}_{\ell}^{c} \\ \mathbf{v}_{\ell}^{c} \end{bmatrix} = \begin{bmatrix} \mathbf{R}_{c}^{w\top} & -\mathbf{R}_{c}^{w\top} (\mathbf{p}_{c}^{w})^{\wedge} \\ \mathbf{0} & \mathbf{R}_{c}^{w\top} \end{bmatrix} \begin{bmatrix} \mathbf{n}_{\ell}^{w} \\ \mathbf{v}_{\ell}^{w} \end{bmatrix}
\tag{11.45}$$

考虑 3D 线在 2D 图像中被观测：给定图像中线段两端点 $\mathbf q_1:=[u_1,v_1,1]^{\mathsf T}$，$\mathbf q_2:=[u_2,v_2,1]^{\mathsf T}$，把 2D 视觉线测量建模为这两端点到反投影 3D Plücker 线在图像平面投影的**距离**（源参考 [1315]）。先用 (11.45) 把 3D 线从世界系变到当前相机系，再用已知相机内参投影到图像（源参考 [1021]）：

$$\boldsymbol\ell = \underbrace{\begin{bmatrix} f_2 & 0 & 0 \\ 0 & f_1 & 0 \\ -f_2 c_1 & -f_1 c_2 & f_1 f_2 \end{bmatrix}}_{\mathbf K} \begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_3 \end{bmatrix} \begin{bmatrix} \mathbf{n}_{\ell}^c \\ \mathbf{v}_{\ell}^c \end{bmatrix} =:\ \begin{bmatrix} \ell_1 \\ \ell_2 \\ \ell_3 \end{bmatrix}
\tag{11.46}$$

其中 $\mathbf K$ 是规范投影 Plücker 矩阵，$f_1,f_2,c_1,c_2$ 是标准相机内参。注意：投影只涉及 Plücker 坐标中的**矩向量 $\mathbf n_\ell^c$**，这意味着 $\mathbf v_\ell^c$ 中含的线距离与朝向**不可测**。故可算线段两端点到图像中投影线 $\boldsymbol\ell$ 的距离，作为线特征测量：

$$\boldsymbol{z}_{\ell} = \begin{bmatrix} \dfrac{\mathbf{q}_{1}^{\top} \boldsymbol{\ell}}{\sqrt{\ell_{1}^{2} + \ell_{2}^{2}}} \\[3mm] \dfrac{\mathbf{q}_{2}^{\top} \boldsymbol{\ell}}{\sqrt{\ell_{1}^{2} + \ell_{2}^{2}}} \end{bmatrix} + \boldsymbol{\eta}^{\ell}
\tag{11.47}$$

其中 $\boldsymbol\eta^\ell$ 是测量噪声。用链式法则对状态线性化 (11.47)，得测量雅可比 $\mathbf H_x=\frac{\partial\boldsymbol z_\ell}{\partial\boldsymbol\ell}\frac{\partial\boldsymbol\ell}{\partial\boldsymbol x}$。

**面特征（Plane Features）。** 3D 平面用世界系下到原点的距离和法向参数化：$\boldsymbol\pi^w=\begin{bmatrix}\mathbf n_\pi^w\\ d_\pi^w\end{bmatrix}$，可变换到（通常检测平面的）局部传感器系：

$$\begin{bmatrix} \boldsymbol{n}_{\pi}^{c} \\ d_{\pi}^{c} \end{bmatrix} = \begin{bmatrix} \boldsymbol{R}_{w}^{c} & \mathbf{0}_{3\times1} \\ -(\boldsymbol{p}_{c}^{w})^{\mathsf{T}} & 1 \end{bmatrix} \begin{bmatrix} \boldsymbol{n}_{\pi}^{w} \\ d_{\pi}^{w} \end{bmatrix}
\tag{11.48}$$

不失一般性，考虑从点云（LiDAR、深度传感器）提取的平面特征 $(\mathbf n_\pi^c,d_\pi^c)$，采用**最近点（closest point）** $\boldsymbol p_\pi^c\doteq d_\pi^c\mathbf n_\pi^c$（平面到原点的最近点）作为 AINS 状态向量中的平面表示（源参考 [374]）：

$$\boldsymbol z_{\pi} = d_{\pi}^{c}\, \mathbf n_{\pi}^{c} + \boldsymbol\eta^{\pi} = \boldsymbol p_{\pi}^{c} + \boldsymbol\eta^{\pi}
\tag{11.49}$$

其中 $\boldsymbol\eta^\pi$ 是平面测量噪声。线性化 (11.49) 得平面测量雅可比 $\mathbf H_x=\frac{\partial\boldsymbol z_\pi}{\partial\boldsymbol p_\pi^c}\frac{\partial\boldsymbol p_\pi^c}{\partial\boldsymbol x}$。

### 11.3.2 可观测性分析（Observability Analysis）【源 11.3.2】

基于前节线性化系统与测量模型，做可观测性分析。分析依赖如下可观测性矩阵 $\mathbf M(\hat{\boldsymbol x})$（源参考 [486]）：

$$\mathbf{M}(\hat{\boldsymbol{x}}) = \begin{bmatrix} \mathbf{H}_{x_1}\, \boldsymbol{\Phi}_{(1,1)} \\ \mathbf{H}_{x_2}\, \boldsymbol{\Phi}_{(2,1)} \\ \vdots \\ \mathbf{H}_{x_k}\, \boldsymbol{\Phi}_{(k,1)} \end{bmatrix}
\tag{11.50}$$

其中 $\mathbf H_{x_k}$ 堆叠离散时刻 $k$ 采集的所有测量（点、线、面）的雅可比；记号 $\mathbf M(\hat{\boldsymbol x})$ 强调可观测性矩阵依赖线性化点 $\hat{\boldsymbol x}$。该矩阵的**零空间 $\mathcal U$**，即满足 $\mathbf M(\boldsymbol x)\boldsymbol u_i=\mathbf 0$ 的零向量张成 $\mathrm{span}([\cdots\boldsymbol u_i\cdots])=\mathcal U$，描述 AINS 的不可观子空间。零空间为空 ⟹ 系统完全可观。

**关键结论**（源参考 [1234]）：AINS 一般有 **4 个不可观方向**（零空间 $\mathcal U$ 有四个独立向量），描述：从 IMU 测量和对先验未知地标的局部观测，**全局 3D 位置和全局偏航（yaw）不可观**。

为理解 4 维零空间的结构，考虑状态向量中含全部三类几何特征（单个点、线、面）：$\mathbf x_f^w=\{\mathbf p_f^w,\mathbf l^w,\boldsymbol\pi^w\}$，外感受测量含 $\mathbf z=\{\mathbf z_p,\mathbf z_\ell,\mathbf z_\pi\}$（见 (11.42)、(11.47)、(11.49)）。计算相关系统与测量雅可比（$\mathbf H_{x_i}$ 和 $\boldsymbol\Phi_{(i,1)}$）并代入 (11.50)，构建线性化 AINS 可观测性矩阵 $\mathbf M$。数学求其零空间 $\mathrm{null}(\mathbf M)$，可得如下四个零向量（源参考 [1234]）：

$$\mathrm{null}(\mathbf{M}) = \mathrm{span}[\boldsymbol{u}_{1}\ \ \boldsymbol{u}_{2:4}] = \mathrm{span}\begin{bmatrix} \boldsymbol{u}_{g} & \mathbf{0}_{12\times3} \\ -\boldsymbol{p}_{1}^{w} \times \mathbf{g}^{w} & \mathbf{I}_{3} \\ -\boldsymbol{p}_{f}^{w} \times \mathbf{g}^{w} & \mathbf{I}_{3} \\ -\boldsymbol{g}^{w} & \dfrac{\mathbf{v}_{\ell}^{w}}{d_{\ell}^{w}\, \lVert\mathbf{v}_{\ell}^{w}\rVert} (\boldsymbol{R}_{\ell}^{w} \mathbf{e}_{1})^{\mathsf{T}} \\ 0 & -(\boldsymbol{R}_{\ell}^{w} \mathbf{e}_{3})^{\mathsf{T}} \\ -d_{\pi}^{w}\, \mathbf{n}_{\pi}^{w} \times \mathbf{g}^{w} & \mathbf{n}_{\pi}^{w} (\boldsymbol{R}_{\pi}^{w} \mathbf{e}_{3})^{\mathsf{T}} \end{bmatrix}
\tag{11.51}$$

其中：
- $\boldsymbol u_g=\big[(\boldsymbol R_w^{c_1}\mathbf g^w)^{\mathsf T}\quad\mathbf 0_{1\times3}\quad-(\mathbf v_1^w\times\mathbf g^w)^{\mathsf T}\quad\mathbf 0_{1\times3}\big]^{\mathsf T}$；
- $\boldsymbol p_1^w$ 指 $k=1$ 时刻传感器位置；
- $\boldsymbol R_w^{c_1}$ 是 $k=1$ 时刻传感器系 $C_1$ 到世界系 $W$ 的旋转矩阵；
- $\boldsymbol R_\pi^w$ 是用平面法向 $\mathbf n_\pi^w$ 经 Gram-Schmidt 正交归一化（见 (11.6)）构造的旋转矩阵；
- $\boldsymbol R_\ell^w=\Big[\frac{\mathbf n_\ell^w}{\lVert\mathbf n_\ell^w\rVert}\ \ \frac{\mathbf v_\ell^w}{\lVert\mathbf v_\ell^w\rVert}\ \ \frac{\mathbf n_\ell^w\times\mathbf v_\ell^w}{\lVert\mathbf n_\ell^w\times\mathbf v_\ell^w\rVert}\Big]$ 是用线法向与线方向构造的旋转矩阵。

可以看出：第一个零向量 $\boldsymbol u_1$ 关联**绕重力的旋转（即 yaw 偏航）**，$\boldsymbol u_{2:4}$ 关联**机器人的平移运动**。更详细分析见 [1234, 1233]。

> 抽取注：源 (11.51) 的 $\boldsymbol R_\ell^w$ 第三列 OCR 重复/残缺（写成 $\frac{\mathbf n_\ell^w}{\lVert\mathbf n_\ell^w\rVert}\frac{\mathbf v_\ell^w}{\lVert\mathbf v_\ell^w\rVert}$），按构造正交旋转矩阵的标准做法，第三列应为前两列叉积归一化 $\frac{\mathbf n_\ell^w\times\mathbf v_\ell^w}{\lVert\mathbf n_\ell^w\times\mathbf v_\ell^w\rVert}$，已据此整理（与 (11.6) 的 Gram-Schmidt 思想一致）。

**小结**：可观测性矩阵有 4 维零空间（(11.51)），正确描述系统全局位置和 yaw 不可观。直观：任何测量（IMU 数据、未知点/线/面地标观测）都不携带全局系信息，唯一例外是 **roll 和 pitch**——它们可从加速度计对重力方向的测量观测到。这种不可观在 SLAM 中常见、非病态：只意味着可任意设定世界系的 yaw 和 3D 原点（因为对这些变量只有相对测量）。若加提供绝对测量的传感器（如 GPS），此不可观消失。更令人担忧的是：对某些运动（和线性化点），可观测性矩阵零空间会变大，产生额外不可观维度（见下节）。

> 源脚注 9：不用 IMU 时，基于地标的 SLAM 问题零空间至少 **6 维**，反映无 IMU 时系统的整个 3D 旋转（外加 3D 位置）都不可观。

### 11.3.3 退化运动（Degenerate Motions）【源 11.3.3】

某些运动可能为 AINS 引入额外不可观方向（除上文 4 个预期方向外）。这有实际重要性：退化运动可能导致状态空间某些方向大误差、致导航失败。AINS 退化运动总结于**表 11.1**（完整推导见 [1234]）。

具体：
- **纯平移（pure translation）**对所有特征类型都退化，使**整个全局旋转不可观**。直观：系统不旋转时，可能混淆重力测量与加速度计偏置，故 roll、pitch 不再可观。
- 另外三种退化运动——**常值加速度（constant acceleration，含常值速度即加速度为零的情形）**、**纯旋转（pure rotation）**、**朝特征方向运动（motion toward feature，单点特征情形）**——对**单目相机（即仅方位测量）**导致**尺度（scale）不可观**。其中：
  - **常值加速度**导致**整个系统**（位置、速度、加速度偏置、特征）的尺度不可观；
  - **纯旋转**和**朝特征运动**只使**特征尺度**不可观。

注意：后三种退化运动仅当传感器到特征的距离远大于传感器与机器人体之间的外参平移（若两者不重合）时成立，即 $\lVert\boldsymbol p_f^c\rVert\gg\lVert\boldsymbol p_b^c\rVert$，实践中通常如此。

#### 表 11.1 AINS 的退化运动（Degenerate motions of AINS）

| Motion（运动） | Sensor（传感器） | Unobservable（不可观量） |
|---|---|---|
| 1. Pure translation（纯平移） | General（通用） | Global orientation（全局朝向） |
| 2. Constant acceleration（常值加速度） | Mono cam（单目相机） | System scale（系统尺度） |
| 3. Pure rotation（纯旋转） | Mono cam（单目相机） | Feature scale（特征尺度） |
| 4. Moving toward point feature（朝点特征运动） | Mono cam（单目相机） | Feature scale（特征尺度） |

---

## 11.4 视觉惯性里程计与实际考量（Visual-Inertial Odometry and Practical Considerations）【源 11.4】

惯性测量通常与其他传感器融合以缓解里程计漂移。本节聚焦相机视觉测量与 IMU 测量用**因子图**融合的情形。相机+IMU 是流行组合：都廉价、轻、低功耗，且互补——IMU 能捕捉快速加速度与旋转，相机能提供环境丰富观测。一方面相机大幅降低（相对纯惯性里程计的）漂移；另一方面 IMU 能让某些原本无法估计的量变可观。特别地：用单目相机做 SLAM 时无先验信息无法估计场景尺度（尺度不可观），加 IMU 后只要机器人运动非退化（11.3.3）即可恢复尺度。含一或多相机+IMU 的系统称 **VIO**，加闭环成视觉惯性 SLAM。

> 源脚注 10：机器人中也常把惯性数据与其他传感器（LiDAR、雷达）结合，见第 8、9 章。

### 11.4.1 视觉惯性里程计（Visual-Inertial Odometry）【源 11.4.1】

VIO 常用作里程计来源，常用于闭合轨迹跟踪与控制的控制回路。其他应用如虚拟现实中，VIO 用以补偿用户在虚拟环境中的运动。两种情形都要求 VIO 产生**极低延迟**估计，典型 10–50ms。例如 Meta Quest 3 刷新率 72–120Hz（源参考 [761]），VIO 延迟直接影响 VR 体验、是缓解晕动症的关键。类似地，轨迹跟踪中保持低延迟很重要，大延迟可能致跟踪控制器不稳定与发散。

**图 11.4**（源 `_page_23_Figure_2.jpeg`）：用预积分 IMU 因子的 VIO 因子图示例（来自 [335]）。图中：
- **预积分 IMU 因子（紫色 violet）**：约束相邻位姿、速度、偏置；
- **偏置因子（蓝色 blue）**：约束 IMU 偏置随时间的演化；
- **视觉因子（橙色 orange）**：关联相机位姿与外部地标位置；
- **先验（黑色 black）**。

基于这些考量，基于因子图的 VIO 系统通常实现**固定滞后平滑器（fixed-lag smoother，又称滑动窗口优化 sliding-window optimization）**，只估计后退视界（如最近 5–10 秒）内的状态。视界选择权衡计算与精度：视界越长，估计状态空间越大。落出后退视界的因子和变量随时间推进逐步**边缘化（marginalize）**。许多优化实现还用 **Schur 补（Schur complement）** 从优化中消去视觉地标，进一步减小状态空间（如 [335]）。

固定滞后平滑器的替代是用**增量求解器如 iSAM2**（1.7 节），它在计算当前估计时复用先前优化的计算。该法实践中很准（如 [335]），但缺点是不提供延迟保证、可能导致运行时尖峰，对某些应用成问题。

#### VIO 系统与性能（VIO Systems and Performance）

过去十年涌现大量 VIO/SLAM 系统，许多有开源实现。流行方法包括：
- ORB-SLAM 的视觉惯性版本 [786]；
- Direct Sparse Visual-Inertial Odometry [1116]；
- VINS-Mono [896]；
- OpenVINS [375]；
- Kimera [3, 942]；
- BASALT [1118]；
- DM-VIO [1043]。

好的 VIO 系统**漂移低于行进距离的 1%**（如行 100m 累积误差小于 1m），某些情形漂移可低至 **0.1%**。

#### 数值例：FEJ-WBA-VINS 在 KAIST 城市数据集上的表现

滑动窗口优化方法如 VINS-Mono [896] 在实践中极成功。这里展示一个较新的滑动窗口方法 **First-Estimate Jacobian (FEJ)-based Window Bundle Adjustment (WBA)-VINS**（源参考 [180, 181]）在 **KAIST Urban Dataset**（源参考 [515]）上的表现。

**图 11.5**（源 `_page_24_Figure_2.jpeg`）：FEJ-WBA-VINS [180] 在 KAIST 城市自动驾驶数据集 sequence 38 上运行的图示。该序列总时长 **36 分钟**、长 **11.42 km**。VIO 估计与真值叠加在 Google 地图上。底部为两张样本图像。VIO（即无闭环）最终 ATE 为 **2.05 度和 21.2 米（0.18%）**。

KAIST 城市数据集详情：聚焦自动驾驶与复杂城市环境定位，在韩国用配备**双目相机对、2D/3D LiDAR、Xsens IMU、光纤陀螺（FoG）、轮编码器、RTK GPS** 的车辆采集。相机 **10Hz**，IMU 感知率 **100Hz**。真值轨迹由 **FoG + RTK GPS + 轮编码器**融合得到。对 sequence 38（11.42 公里路径），FEJ-WBA-VINS（VIO）最终**绝对轨迹误差（ATE）约 2.05 度和 21.2 米（行进轨迹的 0.18%）**。这些结果来自**纯在线 VIO、无闭环**。

VIO 在自动驾驶等自主系统的应用见 [3, 4]，其中也讨论特征跟踪、关键帧选择、不同传感器模态（单目、双目、RGB-D 相机图像及轮里程计）融合的挑战。

### 11.4.2 外参标定（Extrinsic Calibration）【源 11.4.2】

准确的辅助惯性导航需做传感器**外参标定**——估计不同传感器间的相对位姿（如相机相对 IMU 的位姿）。方法主要两类：
- **离线（offline）标定**：部署前执行标定程序，常需用标定靶 [350]、已知运动模式 [686] 或环境先验 [631, 714]。这些程序或多或少耗时，可能需专门设备和受训操作员。一般更准，但繁琐、在期望非专家大规模使用最终系统的场景中不理想。
- **在线（online）标定**：不需专门程序 [301, 633, 1211, 1237]，把外参标定参数作为状态估计问题的一部分估计。优点：能适应系统变化（如传感器位移）而无需新标定程序。但在线法可能不如离线法准，且可能使状态估计问题更复杂甚至病态 [1235]。

### 11.4.3 时间同步（Temporal Synchronization）【源 11.4.3】

惯性辅助系统的另一关键是传感器数据的时间同步。未被考虑的错误同步会导致轨迹估计显著误差和/或在基准度量中引入偏差。同步可在硬件或软件层做：
- **底层硬件**：常依赖专门硬件，基于公共时钟信号经专门同步输入引脚触发各传感器数据采集。但并非总可行，尤其传感器经不同通信协议连到计算机时。某些传感器有内建同步机制，可用于同步各传感器时钟而无需专门硬件输入。
- **软件同步**：**PTP（精确时间协议，Precision Time Protocol）** 是经以太网软件同步的一例，许多 LiDAR、雷达和 INS 方案可用此协议同步。
- 另一方案：在传感器层给数据**打时间戳**，再在**后处理**步骤对齐时间戳。此法一般不如前述方法准和鲁棒。

若系统无法同步且后处理不可行（如在线应用），某些状态估计算法把**时间偏移作为状态变量**纳入估计问题 [301, 376, 1237]。

---

## 11.5 延伸阅读与近期趋势（Further Readings & Recent Trends）【源 11.5】

惯性里程计进展正稳步转化为工业产品，但辅助惯性导航仍是研究热点。

**扩展位姿预积分（Extended Pose Preintegration）。** 近期趋势包括用**扩展位姿流形（extended-pose manifolds）**和**高阶噪声传播** [120] 改进 IMU 预积分的不确定性建模。Brossard 等 [120] 扩展预积分理论以考虑**地球自转的科氏力（Coriolis）与离心力（centrifugal）**。Vial 等 [1126] 给出扩展位姿预积分的例子，结合线速度传感器与导航级 IMU；海上导航一小时、1.8km 轨迹后，作者报告平移误差约 **5m**。

**连续时间状态表示（Continuous-time State Representations）。** 本章主要把 IMU 用于预积分以减少因子图离散状态变量数。但基于连续时间状态表示的其他方法也能在不增加估计状态维度下考虑众多 IMU 测量。例子：[349] 用 **B 样条基函数**，[47] 用 **GP 先验**。两种表述都允许在固定状态变量集间用插值动力学的残差中使用高频 IMU 测量。
- [130] 比较"把 IMU 测量直接作为连续时间 GP 先验的输入"与"把 IMU 测量直接用于残差"，结论：用 LiDAR-惯性传感器套件时，**把惯性信息作为状态的测量**得到更好的里程计精度。
- [659] 比较 [47] 的 GP 状态表示与 [376] 的连续 GP 预积分（本章前面介绍过）；在事件相机 VIO 上下文，作者证明后者（[376]）在精度和计算效率上都比前者略有优势。

**仅本体感知里程计（Proprioception-only Odometry）。** 近期工作用本体感知传感器做辅助惯性导航：
- [441]（足式机器人）和 [813]（轮装 IMU）展示用系统运动学知识提供有竞争力的 IMU 里程计估计，亚百分比位置误差。[441] 的关键信息是机器人足与地接触的知识；[813] 用单平面旋转运动约束 IMU 偏置、从而限制航位推算漂移。
- [813] 被扩展为完整 SLAM 系统 [1200]：通过识别道路侧倾角（road bank angle）随时间的模式检测闭环，是 IMU 本体感知系统能做闭环检测与校正的有趣例子。
- 注意：虽然惯性传感器一般提供更好性能与鲁棒性，但 IMU 传感器的丢失或饱和可对整体系统性能有灾难性影响。[266] 研究陀螺饱和时用加速度计数据估计角速度，提升下游 SLAM 算法鲁棒性。

**仅惯性里程计（Inertial-only Odometry, IOO）。** 无视觉等辅助源时朴素积分 IMU 测量通常快速发散。即使在辅助惯性里程计中，辅助源不可用时这也是隐忧。例如：移动 AR/VR 手部跟踪中高动态手部易移出跟踪相机视场（FOV），只剩 IMU 数据维持运动跟踪；或无纹理场景阻止特征检测跟踪，致 VIO 只能依赖 IMU 数据。为此近期工作研究用**学习和神经网络**减小仅惯性里程计的漂移 [1215, 179, 1055, 451, 452, 220, 898]，包括：
- 用神经网络以数据驱动方式建模 IMU 偏置 [225]；
- 从噪声 IMU 测量序列直接预测位移 [682]；
- 用**可微积分模块**积分去除预测偏置后的 IMU 读数 [1276, 898]；
- 或直接用真值偏置做监督 [123]；
- 或用**条件扩散模型（conditional diffusion model）**近似偏置（偏置建模为概率分布）[1298]。

这些方法已证明能大幅减小仅惯性里程计漂移，但目前泛化能力有限（如对不同传感器或训练时未见运动）。

---

## 附录 A：本章公式编号总览（便于综合时检索）

| 编号 | 内容 |
|---|---|
| (11.1)–(11.2) | IMU 测量模型（加速度计/陀螺，含偏置+白噪声） |
| (11.3) | 加速度计扩展模型（形状矩阵 $\mathbf T_a$） |
| (11.4) | 陀螺扩展模型（形状矩阵 $\mathbf T_g$） |
| (11.5) | 陀螺 g-sensitivity 模型（$\mathbf T_s$） |
| (11.6) | 静态初始对准（Gram-Schmidt，roll/pitch） |
| (11.7) | 高端 IMU 解析对准（用地球自转 $\boldsymbol\omega_{ie}$） |
| (11.8) | 连续运动学模型 $\dot{\mathbf R},\dot{\mathbf v},\dot{\mathbf p}$ |
| (11.9)–(11.10) | 积分形式状态更新 |
| (11.11) | 常值假设下的状态更新（欧拉积分） |
| (11.12) | 用 IMU 测量表达的离散状态更新 |
| (11.13) | 跨关键帧迭代积分 |
| (11.14) | 相对运动增量定义 $\Delta\mathbf R_{ij},\Delta\mathbf v_{ij},\Delta\mathbf p_{ij}$ |
| (11.15) | 偏置常值假设 |
| (11.16)–(11.17) | SO(3) 指数性质（一阶展开、伴随） |
| (11.18) | 旋转增量噪声分离 → $\Delta\tilde{\mathbf R}_{ij}\mathrm{Exp}(-\delta\phi_{ij})$ |
| (11.19)–(11.20) | 指数一阶近似、wedge 反对称性 |
| (11.21) | 速度增量 → $\Delta\tilde{\mathbf v}_{ij}-\delta\mathbf v_{ij}$ |
| (11.22) | 位置增量 → $\Delta\tilde{\mathbf p}_{ij}-\delta\mathbf p_{ij}$ |
| (11.23) | 预积分测量模型（状态+噪声） |
| (11.24) | 预积分噪声高斯分布 $\mathcal N(0,\Sigma_{ij})$ |
| (11.25)–(11.28) | 旋转噪声 $\delta\phi_{ij}$ 的一阶传播 |
| (11.29) | 速度/位置噪声 $\delta\mathbf v_{ij},\delta\mathbf p_{ij}$ |
| (11.30) | 偏置更新的一阶修正 |
| (11.31) | 预积分 IMU 因子残差 $\mathbf r_{\mathcal I_{ij}}$ |
| (11.32) | 偏置布朗运动（连续） |
| (11.33) | 偏置随机游走（离散） |
| (11.34) | 偏置因子残差 |
| (11.35) | 旋转乘积积分（无通解） |
| (11.36) | 旋转向量动力学 $\dot{\mathbf r}=\mathbf J_r^{-1}\boldsymbol\omega$ |
| (11.37) | AINS 状态向量定义 |
| (11.38)–(11.39) | IMU 运动学+偏置随机游走 |
| (11.40) | 连续时间线性化误差状态系统 |
| (11.41) | 离散状态转移矩阵 $\boldsymbol\Phi_{(k+1,k)}$ |
| (11.42)–(11.43) | 点特征测量模型及线性化 |
| (11.44)–(11.47) | 线特征（Plücker）测量模型及线性化 |
| (11.48)–(11.49) | 面特征测量模型及线性化 |
| (11.50) | 可观测性矩阵 $\mathbf M(\hat{\boldsymbol x})$ |
| (11.51) | AINS 4 维零空间结构 |

## 附录 B：图表清单

- **图 11.1**（`_page_6_Figure_2.jpeg`）：IMU 与相机的不同频率（[335]，©2016 IEEE）。
- **图 11.2**（`_page_13_Figure_2.jpeg`）：低/高采样频率的欧拉积分示例。
- **图 11.3**（`_page_14_Figure_2.jpeg`）：分段线性（常值 jerk）连续积分 vs GP 无模型积分。
- **图 11.4**（`_page_23_Figure_2.jpeg`）：VIO 因子图（预积分 IMU 因子=紫，偏置因子=蓝，视觉因子=橙，先验=黑）。
- **图 11.5**（`_page_24_Figure_2.jpeg`）：FEJ-WBA-VINS 在 KAIST sequence 38 上的轨迹（36 分钟/11.42km，ATE 2.05°/21.2m/0.18%）。
- **表 11.1**：AINS 退化运动（4 行）。

## 附录 C：抽取层 OCR 订正汇总（综合时请注意）

源 markdown 由 OCR 生成，以下处按物理/数学自洽性做了订正，综合时应采用订正版：
1. **(11.4)**：陀螺形状矩阵 OCR 写 $\mathbf T_a$ → 订正为 $\mathbf T_g$（正文明确）。
2. **(11.5)**：OCR 写 $\mathbf T_a\boldsymbol\omega_b^b$ 和 $\mathbf R_c^b$ → 订正为 $\mathbf T_g\boldsymbol\omega_b^b$ 和 $\mathbf R_w^b$（与 specific-force 项一致）。
3. **(11.7)**：右侧第三行 OCR 写 $(\mathbf g^b\times\boldsymbol\omega_{ie}^w)$ → 订正为 $(\mathbf g^b\times\boldsymbol\omega_{ie}^b)$（与左侧约束自洽）。
4. **(11.34)**：第二项 OCR 写 $\lVert\mathbf b_i^a-\mathbf b_i^a\rVert$ → 订正为 $\lVert\mathbf b_j^a-\mathbf b_i^a\rVert$。
5. **(11.41)**：块矩阵行列标注 OCR 错位 → 按状态排序与随机游走/静态地标结构整理；$\dot{\boldsymbol\Phi}=\mathbf F\boldsymbol\Phi$ 的微分方程语义补全（OCR 缺点号）。
6. **(11.51)**：$\boldsymbol R_\ell^w$ 第三列 OCR 残缺/重复 → 订正为 $\frac{\mathbf n_\ell^w\times\mathbf v_\ell^w}{\lVert\mathbf n_\ell^w\times\mathbf v_\ell^w\rVert}$。
7. 全文 $\Delta\tilde{\mathbf R}_{k+1\,j}$ 在 OCR 中偶写作 $\Delta\tilde{\mathbf R}_{k+1j}$，含义同（$k{+}1$ 到 $j$ 的预积分旋转），已统一加空格便于阅读。
