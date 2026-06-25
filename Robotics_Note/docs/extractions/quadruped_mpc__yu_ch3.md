# 全量保真抽取：单刚体模型 MPC（四足机器人运动控制范本章种子）

> **源文件**：`MinerU_markdown_SY1807103-于宪元-基于稳定性的仿生四足机器人控制系统设计_2069372853834055680.md`（于宪元硕士学位论文）
> **抽取范围**：主体 = 第三章「仿生四足机器人模型预测控制（MPC）」(源 line 1313–1691)；前置 = 第二章 §2.2「坐标系及刚体的位姿描述」(源 line 425–703) 中 MPC 推导所依赖的部分；并补录被 MPC 引用的关键跨章公式 (2.34/2.35, 2.50, 2.76, 2.77, 2.85, 2.86)。
> **抽取原则**：完全吸收 · 禁摘要。每一步推导、每个公式、每条约束、每个定义、每张表、每个数值例全量原样录入，标注源节号。公式用 LaTeX。
> **本书写章用途**：本文件是「四足机器人运动控制」部 → 范本章「单刚体模型 MPC」的写作种子（§9 流水线）。文末附「于宪元记号 → 本书记号」映射建议表，以及 OCR 乱码存疑清单。

---

## 0. 章引言与全局控制方案（源 §第三章引言, line 1315）

> 机器人控制系统，就是要解决机器人**在哪儿、去哪儿、怎么去**的问题，分别对应了机器人控制系统的**状态估计器、轨迹规划器和控制器**。

本文采用 **MPC + WBC 相结合**的控制方案：
- **MPC**：使用**简化的单刚体模型**，在**较长时间范围**内求解**最优的足底反力**。
- **WBC**（全身控制）：采用**多刚体模型**，将整体控制任务按优先级分为 4 个小控制任务，以规划好的轨迹为跟踪目标，在每个控制周期解算电机指令。

**控制频率（来自源 line 378 控制系统框图）**：
- MPC 控制周期 $\Delta T = 0.01\,\mathrm{s}$（即 100 Hz）。
- 其他算法控制周期 $\Delta t = 0.002\,\mathrm{s}$（即 500 Hz）。

---

## 1. 前置：坐标系与位姿描述（源 §2.2，MPC 依赖部分）

> 仅录入 MPC 推导（§3）实际依赖的前置：旋转矩阵定义与性质、基本旋转、连续旋转左乘、ZYX 欧拉角、角速度张量/反对称算子、角速度的坐标变换性质。

### 1.1 记号约定（源 §2.2.2(1), line 475）

> 通常使用**加粗小写字母**表示向量，**不加粗大写字母**表示点，诸如 $\mathcal{A}$ 字符表示笛卡尔坐标系。
> - **左上角标**：向量所在的坐标系；
> - **右下角标**：向量所表示的具体物理量；
> - **右上角标与左下角标**：标识向量的其他信息。

四足机器人坐标系体系（源 §2.2.1, line 429–450）：
- **世界系** $\mathcal{O}$：刚接到外部物理环境。开机瞬间 $z$ 轴垂直向上且过躯干几何中心，$x$ 轴水平向前，$y$ 轴水平向左，原点在 $z$ 轴与地面交点。
- **本体系** $\mathcal{B}$（$=\mathcal{B}_1$）：刚接于躯干，原点为躯干几何中心；启动瞬间与世界系朝向相同；近似认为躯干质心与本体系原点重合。
- **刚体坐标系** $\mathcal{B}_i$：整机含躯干共 13 个刚体（躯干编号 1，4 条腿各 3 连杆，腿顺序为右前、左前、右后、左后）。
- **足底坐标系** $\mathcal{C}_1\sim\mathcal{C}_4$：原点与足底接触点重合，方向同 3 号连杆刚体系。
- **定向本体系** $\mathcal{P}$（§3.2.2 引入，line 1407）：方向与世界系相同，原点与本体系重合（即与本体系仅差一个旋转，与世界系仅差一个平移）。

机器人物理尺寸（源 (2.2), line 455，§3 落足点等会用到）：
$$
\left[ h_x \; h_y \; l_1 \; l_2 \; l_3 \right]^T = \left[ 0.275\,\mathrm{m} \; 0.0625\,\mathrm{m} \; 0.088\,\mathrm{m} \; 0.27\,\mathrm{m} \; 0.27\,\mathrm{m} \right]^T
\tag{2.2}
$$

### 1.2 旋转矩阵及其性质（源 §2.2.3(1)）

坐标向量在两坐标系间的映射（旋转矩阵定义，源 (2.9)）：
$$
{}^{\mathcal{A}}\mathbf{r}_{AP} = \left[ {}^{\mathcal{A}}\mathbf{e}_x^{\mathcal{B}} \;\; {}^{\mathcal{A}}\mathbf{e}_y^{\mathcal{B}} \;\; {}^{\mathcal{A}}\mathbf{e}_z^{\mathcal{B}} \right] {}^{\mathcal{B}}\mathbf{r}_{AP} = {}^{\mathcal{A}}R_{\mathcal{B}}\,{}^{\mathcal{B}}\mathbf{r}_{AP}
\tag{2.9}
$$
其中 ${}^{\mathcal{A}}R_{\mathcal{B}}$ 是 $3\times3$ 旋转矩阵，列向量为一组单位正交向量，故为正交矩阵：
$$
{}^{\mathcal{A}}R_{\mathcal{B}}^{T}\,{}^{\mathcal{A}}R_{\mathcal{B}} = \mathbb{I}_3
\tag{2.10}
$$
$$
{}^{\mathcal{B}}R_{\mathcal{A}} = {}^{\mathcal{A}}R_{\mathcal{B}}^{-1} = {}^{\mathcal{A}}R_{\mathcal{B}}^{T}
\tag{2.11}
$$

三个基本旋转（源 (2.12)，$c_\varphi=\cos\varphi$、$s_\varphi=\sin\varphi$ 的简写）：
$$
R_x(\varphi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\varphi & -s_\varphi \\ 0 & s_\varphi & c_\varphi \end{bmatrix},\quad
R_y(\varphi) = \begin{bmatrix} c_\varphi & 0 & s_\varphi \\ 0 & 1 & 0 \\ -s_\varphi & 0 & c_\varphi \end{bmatrix},\quad
R_z(\varphi) = \begin{bmatrix} c_\varphi & -s_\varphi & 0 \\ s_\varphi & c_\varphi & 0 \\ 0 & 0 & 1 \end{bmatrix}
\tag{2.12}
$$

连续旋转 = 旋转矩阵连续左乘（源 (2.15)(2.16)）：
$$
{}^{\mathcal{A}}\mathbf{r} = {}^{\mathcal{A}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}R_{\mathcal{C}}\cdot{}^{\mathcal{C}}\mathbf{r} = {}^{\mathcal{A}}R_{\mathcal{C}}\cdot{}^{\mathcal{C}}\mathbf{r}
\tag{2.15}
$$
$$
{}^{\mathcal{A}}R_{\mathcal{C}} = {}^{\mathcal{A}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}R_{\mathcal{C}}
\tag{2.16}
$$

### 1.3 ZYX 欧拉角（源 §2.2.3(1), line 584–629）

> 本文采用机器人领域常用的 **ZYX 欧拉角**（Tait-Bryan 角），坐标轴旋转次序为 ZYX。

欧拉角向量（源 (2.17)）：
$$
\boldsymbol{\Theta} = \left[ \phi \; \theta \; \psi \right]^T
\tag{2.17}
$$
$\phi$ 为横滚角（roll），$\theta$ 为俯仰角（pitch），$\psi$ 为偏航角（yaw）。

ZYX 欧拉角 → 旋转矩阵（源 (2.20)）：
$$
\begin{aligned}
R(\boldsymbol{\Theta}) &= R_z(\psi) R_y(\theta) R_x(\phi) \\
&= \begin{bmatrix} c_\psi & -s_\psi & 0 \\ s_\psi & c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix}
\begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\phi & -s_\phi \\ 0 & s_\phi & c_\phi \end{bmatrix} \\
&= \begin{bmatrix}
c_\theta c_\psi & c_\psi s_\phi s_\theta - c_\phi s_\psi & s_\phi s_\psi + c_\phi c_\psi s_\theta \\
c_\theta s_\psi & c_\phi c_\psi + s_\phi s_\theta s_\psi & c_\phi s_\theta s_\psi - c_\psi s_\phi \\
-s_\theta & c_\theta s_\phi & c_\phi c_\theta
\end{bmatrix}
\end{aligned}
\tag{2.20}
$$

逆变换（给定 $R=[c_{ij}]$，源 (2.21)(2.22)）：
$$
\boldsymbol{\Theta} = \begin{bmatrix} \phi \\ \theta \\ \psi \end{bmatrix} = \begin{bmatrix}
\operatorname{atan}(c_{32}/c_{33}) \\
-\operatorname{atan}\left(c_{31}/\sqrt{c_{32}^2 + c_{33}^2}\right) \\
\operatorname{atan}(c_{21}/c_{11})
\end{bmatrix}
\tag{2.22}
$$
> 该逆变换非唯一；实际使用时需对分母极小/为 0 的情况做特殊处理。

### 1.4 角速度张量与反对称算子（源 §2.2.3(2), line 644–702）

> **关键动机**（line 633）：不能将角速度简单定义为欧拉角对时间的一阶微分。欧拉角三个分量旋转轴随姿态变化、不正交，若以其为基描述角速度则违背牛顿第一定律。角速度表征**刚体上点的位置矢量 → 线速度矢量**的映射。

对 (2.9) 两边微分（$P$ 为刚体上一点，$\mathcal{B}$ 为刚体系，源 (2.23)）：
$$
\begin{aligned}
{}^{\mathcal{A}}\dot{\mathbf{r}}_{AP}
&= \frac{d({}^{\mathcal{A}}\mathbf{r}_{AP})}{dt} \\
&= {}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{B}}\mathbf{r}_{AP} + {}^{\mathcal{A}}R_{\mathcal{B}}\frac{d({}^{\mathcal{B}}\mathbf{r}_{AP})}{dt} \\
&= {}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{B}}\mathbf{r}_{AP} + {}^{\mathcal{A}}R_{\mathcal{B}}\cdot\mathbf{0} \\
&= {}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T}\,{}^{\mathcal{A}}\mathbf{r}_{AP} \\
&= {}^{\mathcal{A}}\Omega_{\mathcal{B}}\,{}^{\mathcal{A}}\mathbf{r}_{AP}
\end{aligned}
\tag{2.23}
$$
> 注：刚体上点在刚体系下描述为常值，故 $d({}^{\mathcal{B}}\mathbf{r}_{AP})/dt=\mathbf{0}$；图中 $\mathcal{A}$ 为不动系，故 ${}^{\mathcal{A}}\dot{\mathbf{r}}_{AP}=d({}^{\mathcal{A}}\mathbf{r}_{AP})/dt$。

**角速度张量** ${}^{\mathcal{A}}\Omega_{\mathcal{B}} = {}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T}$ 为反对称（源 (2.24) 证明）：
$$
\begin{aligned}
{}^{\mathcal{A}}\Omega_{\mathcal{B}} + {}^{\mathcal{A}}\Omega_{\mathcal{B}}^{T}
&= {}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T} + \left({}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T}\right)^{T} \\
&= {}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T} + {}^{\mathcal{A}}R_{\mathcal{B}}\,{}^{\mathcal{A}}\dot{R}_{\mathcal{B}}^{T} \\
&= \frac{d}{dt}\left({}^{\mathcal{A}}R_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T}\right) = \frac{d}{dt}(\mathbb{I}_3) = \mathbf{0}
\end{aligned}
\tag{2.24}
$$
故 ${}^{\mathcal{A}}\Omega_{\mathcal{B}} = -{}^{\mathcal{A}}\Omega_{\mathcal{B}}^{T}$，可写为（源 (2.25)）：
$$
{}^{\mathcal{A}}\Omega_{\mathcal{B}} = \begin{bmatrix} 0 & -\omega_z & \omega_y \\ \omega_z & 0 & -\omega_x \\ -\omega_y & \omega_x & 0 \end{bmatrix}
\tag{2.25}
$$
角速度矢量（源 (2.26)）：
$$
{}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}} = \left[ \omega_x \; \omega_y \; \omega_z \right]^T
\tag{2.26}
$$

**反对称矩阵运算符**（源 (2.27)，下标 $\times$）：
$$
\boldsymbol{a}_\times = \begin{bmatrix} a_1 \\ a_2 \\ a_3 \end{bmatrix}_\times = \begin{bmatrix} 0 & -a_3 & a_2 \\ a_3 & 0 & -a_1 \\ -a_2 & a_1 & 0 \end{bmatrix}
\tag{2.27}
$$
于是（源 (2.28)，注意于宪元用 ${}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}\times}$ 记号表示「角速度矢量的反对称矩阵」，即 $=\Omega$）：
$$
{}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}\times} = {}^{\mathcal{A}}\Omega_{\mathcal{B}}
\tag{2.28}
$$
> 验证关系：${}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}} \times {}^{\mathcal{A}}\mathbf{r}_{AP} = {}^{\mathcal{A}}\Omega_{\mathcal{B}}\,{}^{\mathcal{A}}\mathbf{r}_{AP}$。

角速度的坐标变换性质（§3.2.5 会用，源 (2.31)(2.32)）：
$$
{}^{\mathcal{B}}\boldsymbol{\omega}_{\mathcal{AB}\times} = {}^{\mathcal{B}}R_{\mathcal{A}}\cdot{}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}\times}\cdot{}^{\mathcal{A}}R_{\mathcal{B}}
\tag{2.31}
$$
$$
{}^{\mathcal{B}}\boldsymbol{\omega}_{\mathcal{AB}\times} = {}^{\mathcal{B}}R_{\mathcal{A}}\cdot\left({}^{\mathcal{A}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{A}}R_{\mathcal{B}}^{T}\right)\cdot{}^{\mathcal{A}}R_{\mathcal{B}} = {}^{\mathcal{B}}R_{\mathcal{A}}\cdot{}^{\mathcal{A}}\dot{R}_{\mathcal{B}}
\tag{2.32}
$$

> **§2.2.3 给出的核心关系（§3.2.4 微分会用）**：${}^{\mathcal{O}}\dot{R}_{\mathcal{B}} = {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{O}}R_{\mathcal{B}}$（由 (2.23) 中 $\Omega={}^{\mathcal{A}}\dot R_{\mathcal B}{}^{\mathcal A}R_{\mathcal B}^T$ 右乘 $R$ 得到，即式 (3.17) 的等价形式）。

---

## 2. §3.1 模型预测控制在间歇欠驱系统中的必要性（源 line 1317–1359）

### 2.1 间歇欠驱系统概念（运动员越障类比，图29）

运动员跃过障碍分三阶段：**起跳、飞行（腾空）、落地**。忽略空气阻力，系统外力仅含足底反力与重力。
- **腾空阶段**：系统仅受重力，质心轨迹是抛物线，无论肌肉如何发力都无法改变质心轨迹 → **典型欠驱系统**。
- **起跳/落地阶段**：可通过控制肌肉发力控制足底反力 → 质心位置**可控**。

> **定义（间歇欠驱系统）**：某些自由度间歇性进入欠驱状态的系统。

**对预测特性的需求**：跳得多高/多远完全取决于起跳阶段地面作用力大小，要求运动员在起跳阶段就考虑好未来一段时间质心位置变化；腾空阶段虽不能改变质心轨迹，但可调整四肢为落地做准备。

> 机器人控制器同样应有此效果（line 1330）：当前控制周期求控制量时，不仅要跟踪当前期望位置，还要求出未来一段时间的多个控制量以跟踪未来轨迹 → 要求控制器**根据控制量和系统模型预测未来一段时间的状态**。

### 2.2 四足对角步态的欠驱性（图30）

以最常见的**对角步态**为例：右前腿+左后腿为摆动腿（足端与地面无相互作用），两个支撑足简化为球，球心连线称为**支撑线**。
- 足底反力作用点都在支撑线上 → **无论足底反力多大，都不可能产生绕支撑线的扭矩** → 机器人绕支撑线旋转方向是欠驱的。
- 半个迈步周期后支撑腿与摆动腿切换，支撑线变为另一对足端连线，绕新支撑线方向仍欠驱。

**结论（line 1359）**：为保证欠驱方向不失稳，控制器应在某方向成为欠驱方向**之前**提前为即将到来的欠驱状态做准备；在即将脱离欠驱状态之前为可驱状态做准备。因此四足对角步态控制器**需要预测特性**。

---

## 3. §3.2 单刚体动力学建模（源 line 1361–1584）

### 3.0 建模动机（源 line 1363）

MPC 每周期对未来预测并选最优控制序列，相当于多次仿真，计算量大。为简化：**忽略关节位置变化导致的腿部质量分布变化，将四足机器人建模为单刚体**。考虑单刚体空间 6 自由度的位置与速度，**仅用 12 个变量**即可完整描述四足机器人状态。相关研究 [44] 表明此简化足以将 MPC 计算量降到可接受水平，同时不损失太多控制精度。

### 3.1 §3.2.1 平动动力学与转动动力学（源 line 1365–1383）

**平动动力学**（牛顿第二定律，世界系下刚体平动加速度，源 (3.1)）：
$$
{}^{\mathcal{O}}\ddot{\boldsymbol{p}}_{com} = \frac{\sum_{i=1}^{4}{}^{\mathcal{O}}\boldsymbol{f}_i}{m} + {}^{\mathcal{O}}\boldsymbol{g}
\tag{3.1}
$$

**转动动力学**（角动量定理：质心处角动量对时间的微分 = 作用于刚体所有外力在质心处产生力矩的矢量和，源 (3.2)）：
$$
\frac{d}{dt}\left({}^{\mathcal{P}}I \cdot {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}}\right) = \sum_{i=1}^{4}{}^{\mathcal{O}}\boldsymbol{r}_i \times {}^{\mathcal{O}}\boldsymbol{f}_i + \boldsymbol{\theta}_{3\times1} \times (m\cdot{}^{\mathcal{O}}\boldsymbol{g})
\tag{3.2}
$$
> 其中 ${}^{\mathcal{P}}I$ 为刚体惯性张量在**定向本体系** $\mathcal{P}$ 下的描述。
> **⚠OCR 存疑**：(3.2) 右端第二项 $\boldsymbol{\theta}_{3\times1}\times(m\cdot{}^{\mathcal{O}}\boldsymbol{g})$ 中的 $\boldsymbol{\theta}_{3\times1}$ 据物理含义应为**质心到质心的零向量** $\mathbf{0}_{3\times1}$（重力作用于质心，对质心无力矩，故该项 $=\mathbf{0}$）。疑为 MinerU 把 $\mathbf{0}$（粗体零）OCR 成了 $\boldsymbol{\theta}$。本书采用 $\mathbf{0}_{3\times1}$。

**受力分析（图31）**：黑色实心点为足底位置，黑色箭头为质心指向足底位置的向量，红色箭头为足底反力，品红色箭头为刚体所受重力。

### 3.2 §3.2.2 惯性张量的转动变换（源 line 1386–1429）

刚体由体积元组成，惯性张量在坐标系 $\mathcal{A}$ 下定义为（源 (3.3)）：
$$
{}^{\mathcal{A}}I = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix}
\tag{3.3}
$$
各元素（源 (3.4)，$\rho$ 为密度，$dv$ 为体积元）：
$$
\begin{cases}
I_{xx} = \iiint_V (y^2 + z^2)\rho\, dv \\
I_{yy} = \iiint_V (x^2 + z^2)\rho\, dv \\
I_{zz} = \iiint_V (x^2 + y^2)\rho\, dv
\end{cases}
\qquad
\begin{cases}
I_{xy} = \iiint_V (xy)\rho\, dv \\
I_{xz} = \iiint_V (xz)\rho\, dv \\
I_{yz} = \iiint_V (yz)\rho\, dv
\end{cases}
\tag{3.4}
$$
> 惯性张量描述质量在刚体中的分布。在定向本体系下惯性张量是姿态的函数，而本体系下惯性张量是常值，故需推导本体系→定向本体系的映射。

**惯性张量的转动变换推导**（设角动量 $\boldsymbol{l}$、角速度 $\boldsymbol{\omega}_{\mathcal{OB}}$，定向本体系下角动量，源 (3.5)）：
$$
\begin{aligned}
{}^{\mathcal{P}}\boldsymbol{l}
&= {}^{\mathcal{P}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}\boldsymbol{l} \\
&= {}^{\mathcal{P}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{B}}\boldsymbol{\omega}_{\mathcal{OB}} \\
&= {}^{\mathcal{P}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{P}}R_{\mathcal{B}}^{T}\cdot{}^{\mathcal{P}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}\boldsymbol{\omega}_{\mathcal{OB}} \\
&= \left({}^{\mathcal{P}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{P}}R_{\mathcal{B}}^{T}\right)\left({}^{\mathcal{P}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}\boldsymbol{\omega}_{\mathcal{OB}}\right) \\
&= {}^{\mathcal{P}}I\cdot{}^{\mathcal{P}}\boldsymbol{\omega}_{\mathcal{OB}}
\end{aligned}
\tag{3.5}
$$
由于定向本体系与世界系平行，即 ${}^{\mathcal{P}}R_{\mathcal{B}} = {}^{\mathcal{O}}R_{\mathcal{B}}$，从而（源 (3.6)）：
$$
{}^{\mathcal{P}}I = {}^{\mathcal{O}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{O}}R_{\mathcal{B}}^{T}
\tag{3.6}
$$

**数值例（本文四足机器人本体系惯性张量，三维建模软件导出，源 (3.7)）**：
$$
{}^{\mathcal{B}}I = \begin{bmatrix} 0.31 & 0 & -0.05 \\ 0 & 1.09 & 0 \\ -0.05 & 0 & 1.12 \end{bmatrix}\,\mathrm{kg\cdot m^2}
\tag{3.7}
$$

**质心指向足底向量**（(3.2) 中 ${}^{\mathcal{O}}\boldsymbol{r}_i$，由状态估计器的质心位置与 (2.50) 足底坐标做差，源 (3.8)）：
$$
{}^{\mathcal{O}}\boldsymbol{r}_i = {}_i^{\mathcal{O}}\boldsymbol{p} - {}_i^{\mathcal{O}}\boldsymbol{p}_{com}
\tag{3.8}
$$
> **⚠OCR 存疑**：(3.8) 右端第二项原文作 ${}_i^{\mathcal{O}}\boldsymbol{p}_{com}$，但质心位置不应带腿编号下标 $i$，应为 ${}^{\mathcal{O}}\boldsymbol{p}_{com}$。本书写作时去掉下标 $i$。
> **被引用前置 (2.50)（足底坐标本体系→世界系，源 line 902）**：
> $$ {}_i^{\mathcal{O}}\boldsymbol{p} = {}^{\mathcal{O}}\boldsymbol{p}_{com} + {}^{\mathcal{O}}R_{\mathcal{B}}\cdot{}_i^{\mathcal{B}}\boldsymbol{p} \tag{2.50} $$

### 3.3 §3.2.3 足底反力约束（源 line 1431–1470）

**摆动腿足底反力为 0**（源 (3.9)；$_i s_\Phi$ 为第 $i$ 腿支撑相布尔量，见下方 (2.35)）：
$$
{}^{\mathcal{O}}\boldsymbol{f}_i = \mathbf{0}_{3\times1},\quad \forall i\; s_\Phi = 0
\tag{3.9}
$$
> **被引用前置 (2.35)（支撑相布尔量，源 line 772）**：$_i s_\Phi = 1$（$\tilde t_\Phi \le \boldsymbol G_d(i)$，支撑相）；$_i s_\Phi = 0$（$\tilde t_\Phi > \boldsymbol G_d(i)$，摆动相）。

**竖直方向幅值上限**（源 (3.10)）：
$$
{}^{\mathcal{O}}f_i^{z} \leqslant f_{\max}
\tag{3.10}
$$

**摩擦锥条件**（保证足底与地面不滑动，水平分量 $\le$ 竖直分量 $\times$ 滑动摩擦系数 $\mu$，图33，源 (3.11)）：
$$
\mu\cdot{}^{\mathcal{O}}f_i^{z} \geqslant \sqrt{\left({}^{\mathcal{O}}f_i^{x}\right)^2 + \left({}^{\mathcal{O}}f_i^{y}\right)^2}
\tag{3.11}
$$

> (3.11) 是非线性约束，不利于计算。在满足使用的情况下拆分为 **4 个线性约束**（金字塔近似），结合 (3.10) 整合到一个矩阵（源 (3.12)）：
$$
\underbrace{\begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}}_{\underline{c}_i}
\leqslant
\underbrace{\begin{bmatrix} -1 & 0 & \mu \\ 0 & -1 & \mu \\ 1 & 0 & \mu \\ 0 & 1 & \mu \\ 0 & 0 & 1 \end{bmatrix}}_{C_i}
\begin{bmatrix} {}^{\mathcal{O}}f_i^{x} \\ {}^{\mathcal{O}}f_i^{y} \\ {}^{\mathcal{O}}f_i^{z} \end{bmatrix}
\leqslant
\underbrace{\begin{bmatrix} +\infty \\ +\infty \\ +\infty \\ +\infty \\ f_{\max} \end{bmatrix}}_{\bar{c}_i}
\tag{3.12}
$$
紧凑形式（源 (3.13)）：
$$
\underline{c}_i \leqslant C_i\cdot{}^{\mathcal{O}}\boldsymbol{f}_i \leqslant \overline{c}_i
\tag{3.13}
$$
> 注（line 1470）：$\underline{c}_i$、$C_i$、$\overline{c}_i$ 中具体内容与腿编号 $i$ 无关（即四条腿同形）。

### 3.4 §3.2.4 近似欧拉方程（源 line 1472–1486）

由于 ${}^{\mathcal{B}}I$ 是常数，结合 (2.23)（即 ${}^{\mathcal{O}}\dot R_{\mathcal B}={}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}\times}{}^{\mathcal O}R_{\mathcal B}$）对 (3.6) 两边微分（源 (3.14)）：
$$
\begin{aligned}
{}^{\mathcal{P}}\dot{I}
&= {}^{\mathcal{O}}\dot{R}_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{O}}R_{\mathcal{B}}^{T} + {}^{\mathcal{O}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{O}}\dot{R}_{\mathcal{B}}^{T} \\
&= \left({}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{O}}R_{\mathcal{B}}\right)\cdot{}^{\mathcal{B}}I\cdot{}^{\mathcal{O}}R_{\mathcal{B}}^{T} + {}^{\mathcal{O}}R_{\mathcal{B}}\cdot{}^{\mathcal{B}}I\cdot\left({}^{\mathcal{O}}R_{\mathcal{B}}^{T}\cdot\left({}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\right)^{T}\right) \\
&= {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{P}}I + {}^{\mathcal{P}}I\cdot\left({}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\right)^{T} \\
&= {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{P}}I - {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}
\end{aligned}
\tag{3.14}
$$
> 末步用了反对称性 $\left({}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\right)^{T} = -{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}$。

因此 (3.2) 左端角动量对时间的微分为（源 (3.15)）：
$$
\begin{aligned}
\frac{d}{dt}\left({}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}}\right)
&= {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\dot{\boldsymbol{\omega}}_{\mathcal{OB}} + {}^{\mathcal{P}}\dot{I}\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\
&= {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\dot{\boldsymbol{\omega}}_{\mathcal{OB}} + \left({}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{P}}I - {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\right)\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\
&= {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\dot{\boldsymbol{\omega}}_{\mathcal{OB}} + {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} - {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\
&= {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\dot{\boldsymbol{\omega}}_{\mathcal{OB}} + {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\
&\approx {}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\dot{\boldsymbol{\omega}}_{\mathcal{OB}}
\end{aligned}
\tag{3.15}
$$
> 第三步→第四步：$-{}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} = \mathbf{0}$，因为 $\boldsymbol{\omega}_\times\boldsymbol{\omega} = \boldsymbol{\omega}\times\boldsymbol{\omega} = \mathbf{0}$。
> 最后一步**近似**：在刚体角速度很小的情况下，$ {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}\cdot{}^{\mathcal{P}}I\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}}$ 项（陀螺项）可忽略，得到**线性化的欧拉方程**。实践 [40, 45] 表明此近似足以满足控制需求。

### 3.5 §3.2.5 角速度到 ZYX 欧拉角微分的映射（源 line 1488–1538）

定义 $\boldsymbol{e}_1$、$\boldsymbol{e}_2$、$\boldsymbol{e}_3$ 为 ${}^{\mathcal{O}}R_{\mathcal{B}}$ 的三列，结合 (2.20)（源 (3.16)）：
$$
{}^{\mathcal{O}}R_{\mathcal{B}}(\boldsymbol{\Theta}) = R_z(\psi) R_y(\theta) R_x(\phi) = \begin{bmatrix} \boldsymbol{e}_1 & \boldsymbol{e}_2 & \boldsymbol{e}_3 \end{bmatrix}
\tag{3.16}
$$
角速度与旋转矩阵关系（源 (3.17)，即 (2.23) 等价形式）：
$$
{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times} = {}^{\mathcal{O}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{O}}R_{\mathcal{B}}^{T}
\tag{3.17}
$$
将 (3.16) 代入 (3.17)（源 (3.18)，$R_z,R_y,R_x$ 为 $R_z(\psi),R_y(\theta),R_x(\phi)$ 的简写）：
$$
\begin{aligned}
{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}
&= {}^{\mathcal{O}}\dot{R}_{\mathcal{B}}\,{}^{\mathcal{O}}R_{\mathcal{B}}^{T} \\
&= \left(\dot{R}_z R_y R_x + R_z \dot{R}_y R_x + R_z R_y \dot{R}_x\right)\left(R_z R_y R_x\right)^{T} \\
&= \left(\dot{R}_z R_y R_x + R_z \dot{R}_y R_x + R_z R_y \dot{R}_x\right) R_x^{T} R_y^{T} R_z^{T} \\
&= \dot{R}_z R_y R_x R_x^{T} R_y^{T} R_z^{T} + R_z \dot{R}_y R_x R_x^{T} R_y^{T} R_z^{T} + R_z R_y \dot{R}_x R_x^{T} R_y^{T} R_z^{T} \\
&= \dot{R}_z R_z^{T} + R_z \dot{R}_y R_y^{T} R_z^{T} + R_z R_y \dot{R}_x R_x^{T} R_y^{T} R_z^{T} \\
&= (\boldsymbol{e}_3\dot{\psi})_\times + R_z(\boldsymbol{e}_2\dot{\theta})_\times R_z^{T} + R_z R_y(\boldsymbol{e}_1\dot{\phi})_\times R_y^{T} R_z^{T}
\end{aligned}
\tag{3.18}
$$
> 关键：单轴旋转的 $\dot R R^T$ 是以「该轴单位向量 $\times$ 角速率」为轴的反对称矩阵，即 $\dot R_z R_z^T = (\boldsymbol e_3\dot\psi)_\times$ 等（$\boldsymbol e_1=\hat x,\boldsymbol e_2=\hat y,\boldsymbol e_3=\hat z$ 为基本轴）。

结合 (2.31)（角速度反对称矩阵的相似变换 $R\,\boldsymbol a_\times R^T = (R\boldsymbol a)_\times$），(3.18) 继续推（源 (3.19)）：
$$
\begin{aligned}
{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}\times}
&= (\boldsymbol{e}_3\dot{\psi})_\times + R_z(\boldsymbol{e}_2\dot{\theta})_\times R_z^{T} + R_z R_y(\boldsymbol{e}_1\dot{\phi})_\times R_y^{T} R_z^{T} \\
&= (\boldsymbol{e}_3\dot{\psi})_\times + (R_z\boldsymbol{e}_2\dot{\theta})_\times + (R_z R_y\boldsymbol{e}_1\dot{\phi})_\times \\
&= (\boldsymbol{e}_3\dot{\psi} + R_z\boldsymbol{e}_2\dot{\theta} + R_z R_y\boldsymbol{e}_1\dot{\phi})_\times
\end{aligned}
\tag{3.19}
$$
因此 **ZYX 欧拉角微分 → 角速度**（去掉反对称算子，源 (3.20)）：
$$
\begin{aligned}
{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}}
&= \boldsymbol{e}_3\dot{\psi} + R_z\boldsymbol{e}_2\dot{\theta} + R_z R_y\boldsymbol{e}_1\dot{\phi} \\
&= \begin{bmatrix} R_z R_y\boldsymbol{e}_1 & R_z\boldsymbol{e}_2 & \boldsymbol{e}_3 \end{bmatrix}\begin{bmatrix} \dot{\phi} & \dot{\theta} & \dot{\psi} \end{bmatrix}^{T} \\
&= \begin{bmatrix} c_\theta c_\psi & -s_\psi & 0 \\ c_\theta s_\psi & c_\psi & 0 \\ -s_\theta & 0 & 1 \end{bmatrix}\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
\end{aligned}
\tag{3.20}
$$
由于（源 (3.21)，给出逆矩阵）：
$$
\begin{bmatrix} c_\psi/c_\theta & s_\psi/c_\theta & 0 \\ -s_\psi & c_\psi & 0 \\ c_\psi s_\theta/c_\theta & s_\psi s_\theta/c_\theta & 1 \end{bmatrix}\begin{bmatrix} c_\theta c_\psi & -s_\psi & 0 \\ c_\theta s_\psi & c_\psi & 0 \\ -s_\theta & 0 & 1 \end{bmatrix} = \mathbb{I}_3
\tag{3.21}
$$
因此**角速度 → ZYX 欧拉角微分**的映射（源 (3.22)）：
$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} c_\psi/c_\theta & s_\psi/c_\theta & 0 \\ -s_\psi & c_\psi & 0 \\ c_\psi s_\theta/c_\theta & s_\psi s_\theta/c_\theta & 1 \end{bmatrix}{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}}
\tag{3.22}
$$

当横滚角 $\phi$ 与俯仰角 $\theta$ 很小时（$c_\theta\to1,\,s_\theta\to0$），(3.22) 近似为（源 (3.23)）：
$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} c_\psi & s_\psi & 0 \\ -s_\psi & c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} = R_z^{T}(\psi)\cdot{}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}}
\tag{3.23}
$$
> 实践 [15, 16] 表明此近似足以满足控制需求。

### 3.6 §3.2.6 单刚体动力学模型及其离散化（源 line 1540–1584）

将 (3.23)、(3.15)、(3.2)、(3.1) 整合到一个方程（源 (3.24)，状态序 $[\boldsymbol\Theta;\,{}^{\mathcal O}\boldsymbol p_{com};\,{}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}};\,{}^{\mathcal O}\dot{\boldsymbol p}_{com}]$）：
$$
\frac{d}{dt}\begin{bmatrix} \boldsymbol{\Theta} \\ {}^{\mathcal{O}}\boldsymbol{p}_{com} \\ {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\ {}^{\mathcal{O}}\dot{\boldsymbol{p}}_{com} \end{bmatrix}
=
\begin{bmatrix}
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & R_z^{T}(\psi) & \mathbf{0}_{3\times3} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbb{I}_3 \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3}
\end{bmatrix}
\begin{bmatrix} \boldsymbol{\Theta} \\ {}^{\mathcal{O}}\boldsymbol{p}_{com} \\ {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\ {}^{\mathcal{O}}\dot{\boldsymbol{p}}_{com} \end{bmatrix}
+
\begin{bmatrix}
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
{}^{\mathcal{P}}I^{-1}r_{1\times} & {}^{\mathcal{P}}I^{-1}r_{2\times} & {}^{\mathcal{P}}I^{-1}r_{3\times} & {}^{\mathcal{P}}I^{-1}r_{4\times} \\
\mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m
\end{bmatrix}
\begin{bmatrix} {}^{\mathcal{O}}\boldsymbol{f}_1 \\ {}^{\mathcal{O}}\boldsymbol{f}_2 \\ {}^{\mathcal{O}}\boldsymbol{f}_3 \\ {}^{\mathcal{O}}\boldsymbol{f}_4 \end{bmatrix}
+
\begin{bmatrix} \mathbf{0}_{3\times1} \\ \mathbf{0}_{3\times1} \\ \mathbf{0}_{3\times1} \\ {}^{\mathcal{O}}\boldsymbol{g} \end{bmatrix}
\tag{3.24}
$$
> 其中 ${}^{\mathcal{P}}I^{-1}$ 为 ${}^{\mathcal{P}}I$ 的逆矩阵；$r_{i\times}$ 为 ${}^{\mathcal{O}}\boldsymbol r_i$ 的反对称矩阵（来自 (3.2) 的叉乘 ${}^{\mathcal{O}}\boldsymbol r_i\times{}^{\mathcal{O}}\boldsymbol f_i = r_{i\times}{}^{\mathcal{O}}\boldsymbol f_i$）。

由于 (3.24) 不能直接作为标准状态方程（含常数重力项），将**第三项重力加速度项整合到状态中**（把 ${}^{\mathcal{O}}\boldsymbol g(3)=-9.8$ 作为第 13 个状态分量），得增广连续状态方程（源 (3.25)）：
$$
\underbrace{\frac{d}{dt}\begin{bmatrix} \boldsymbol{\Theta} \\ {}^{\mathcal{O}}\boldsymbol{p}_{com} \\ {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\ {}^{\mathcal{O}}\dot{\boldsymbol{p}}_{com} \\ {}^{\mathcal{O}}\boldsymbol{g}(3) \end{bmatrix}}_{\dot{\boldsymbol{x}}}
=
\underbrace{\begin{bmatrix}
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & R_z^{T}(\psi) & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbb{I}_3 & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \boldsymbol{1}_{3\times1} \\
\mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & 0
\end{bmatrix}}_{A_c}
\underbrace{\begin{bmatrix} \boldsymbol{\Theta} \\ {}^{\mathcal{O}}\boldsymbol{p}_{com} \\ {}^{\mathcal{O}}\boldsymbol{\omega}_{\mathcal{OB}} \\ {}^{\mathcal{O}}\dot{\boldsymbol{p}}_{com} \\ {}^{\mathcal{O}}\boldsymbol{g}(3) \end{bmatrix}}_{\boldsymbol{x}}
+
\underbrace{\begin{bmatrix}
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
{}^{\mathcal{P}}I^{-1}r_{1\times} & {}^{\mathcal{P}}I^{-1}r_{2\times} & {}^{\mathcal{P}}I^{-1}r_{3\times} & {}^{\mathcal{P}}I^{-1}r_{4\times} \\
\mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m \\
\mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3}
\end{bmatrix}}_{B_c}
\underbrace{\begin{bmatrix} {}^{\mathcal{O}}\boldsymbol{f}_1 \\ {}^{\mathcal{O}}\boldsymbol{f}_2 \\ {}^{\mathcal{O}}\boldsymbol{f}_3 \\ {}^{\mathcal{O}}\boldsymbol{f}_4 \end{bmatrix}}_{\boldsymbol{u}}
\tag{3.25}
$$
> **⚠OCR 存疑（A_c 第 5 列与末行）**：源 (3.25) 中 $A_c$ 第 4 块行第 5 列、第 5 行整行的 MinerU 渲染含糊（出现 `0`、`1`、`\vdots`、`\cdots` 混排）。从 (3.24)→(3.25) 的逻辑（把 ${}^{\mathcal{O}}\dot{\boldsymbol p}_{com}$ 行的 ${}^{\mathcal O}\boldsymbol g$ 改为乘状态分量 ${}^{\mathcal O}\boldsymbol g(3)$，且 ${}^{\mathcal O}\boldsymbol g$ 只有第 3 分量非零）可推断：第 4 块行第 5 列应是把重力作用到 $\dot z$ 方向的选择列 $[0,0,1]^T$，其余为 0；末行（${}^{\mathcal O}\boldsymbol g(3)$ 的导数 = 0）全为 0。上式按此物理含义重排，本书写作时采用此规整形式并显式说明。

紧凑形式 = 单刚体系统**连续状态方程**（源 (3.26)）：
$$
\dot{\boldsymbol{x}}(t) = A_c(\psi)\boldsymbol{x}(t) + B_c(\boldsymbol{r}_1, \boldsymbol{r}_2, \boldsymbol{r}_3, \boldsymbol{r}_4, \psi)\boldsymbol{u}(t)
\tag{3.26}
$$

**离散化**：用 $\Delta T$ 表 MPC 周期，一个 $\Delta T$ 内近似认为 $\dot{\boldsymbol x}$ 未变（源 (3.27)）：
$$
\dot{\boldsymbol{x}} = \frac{\boldsymbol{x}(k+1) - \boldsymbol{x}(k)}{\Delta T} = A\boldsymbol{x}(k) + B\boldsymbol{u}(k)
\tag{3.27}
$$
前向欧拉离散化（源 (3.28)）：
$$
\boldsymbol{x}(k+1) = (\mathbb{I} + \Delta T\cdot A)\boldsymbol{x}(k) + \Delta T\cdot B\boldsymbol{u}(k) = A_k\boldsymbol{x}(k) + B_k\boldsymbol{u}(k)
\tag{3.28}
$$
其中 $A_k$、$B_k$（源 (3.29)）：
$$
A_k = \begin{bmatrix}
\mathbb{I}_3 & \mathbf{0}_{3\times3} & R_z^{T}({}_d\psi^k)\Delta T & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbb{I}_3 & \mathbf{0}_{3\times3} & \mathbb{I}_3\Delta T & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbb{I}_3 & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times1} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbb{I}_3 & \boldsymbol{1}_{3\times1}\Delta T \\
\mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & 1
\end{bmatrix},\quad
B_k = \begin{bmatrix}
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
\mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} & \mathbf{0}_{3\times3} \\
{}^{\mathcal{P}}I_k^{-1}\cdot r_{1\times}^{k}\Delta T & {}^{\mathcal{P}}I_k^{-1}\cdot r_{2\times}^{k}\Delta T & {}^{\mathcal{P}}I_k^{-1}\cdot r_{3\times}^{k}\Delta T & {}^{\mathcal{P}}I_k^{-1}\cdot r_{4\times}^{k}\Delta T \\
\mathbb{I}_3\Delta T/m & \mathbb{I}_3\Delta T/m & \mathbb{I}_3\Delta T/m & \mathbb{I}_3\Delta T/m \\
\mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3} & \mathbf{0}_{1\times3}
\end{bmatrix}
\tag{3.29}
$$
> **维度**：$A_k$ 是 13×13 矩阵，$B_k$ 是 13×12 矩阵。
> **${}_d\psi^k$**：质心轨迹规划器生成的第 $k$ 控制周期期望偏航角。
> **第 $k$ 周期惯性张量**：若忽略俯仰角与横滚角，将第 $k$ 周期旋转矩阵近似为 ${}^{\mathcal{O}}R_{\mathcal{B}}^{k} = R_z({}_d\psi^k)$，代入 (3.6) 求 ${}^{\mathcal{P}}I_k$。
> **${}_d\psi^k$ 上的 OCR**：源把它写成 ${}_d\psi^k$（"$_{d}\psi^{k}$"），与 (2.76) 期望偏航一致。

**名义落足点 $r_{i\times}^k$**（第 $k$ 周期第 $i$ 腿；支撑腿取当前足底坐标，摆动腿取 (2.85)(2.86) 落足点，源 (3.30)）：
$$
\boldsymbol{r}_{i\times}^{k} = \begin{cases} {}_i^{\mathcal{O}}\boldsymbol{p}^{t}, & {}_i s_\Phi^{t} = 1 \\ {}_i^{\mathcal{O}}\boldsymbol{p}_{sw\cdot end}^{t}, & {}_i s_\Phi^{t} = 0 \end{cases}
\tag{3.30}
$$
> **预测时域约束（line 1584，重要假设）**：假设预测时域内每条腿作为支撑腿期间足底坐标不变。因此**预测时域长度不能超过半个迈步周期**，否则同一腿可能在时域内经历「支撑→摆动→支撑」，第二次支撑沿用第一次足底坐标会产生较大预测误差。
> **被引用前置 (2.85)（摆动腿落足点，源 line 1187）**：${}_i^{\mathcal{O}}\boldsymbol{p}_{sw\cdot end} = {}_i^{\mathcal{O}}\boldsymbol{p}_{hip}^{touch} + \Delta\boldsymbol{p}_1 + \Delta\boldsymbol{p}_2 + \Delta\boldsymbol{p}_3 + \Delta\boldsymbol{p}_4$；**(2.86)** 给落足点 $z$ 坐标（地面坡度平面）：${}_i^{\mathcal{O}}\boldsymbol{p}_{sw\cdot end}(3) = a + b\cdot{}_i^{\mathcal{O}}\boldsymbol{p}_{sw\cdot end}(1) + c\cdot{}_i^{\mathcal{O}}\boldsymbol{p}_{sw\cdot end}(2)$。

---

## 4. §3.3 预测方程（源 line 1586–1612）

由离散状态方程 (3.28)，$A_k$、$B_k$ 仅和期望轨迹、落足点坐标、当前系统状态有关，是**已知量**；给定 $\boldsymbol u(k)$ 可由 $\boldsymbol x(k)$ 递推 $\boldsymbol x(k+1)$，依次类推。

利用 (3.28) 递推未来 $h$ 个控制周期的状态（源 (3.31)）：
$$
\begin{aligned}
\boldsymbol{x}(1) &= A_0\boldsymbol{x}(0) + B_0\boldsymbol{u}(0) \\
\boldsymbol{x}(2) &= A_1 A_0\boldsymbol{x}(0) + A_1 B_0\boldsymbol{u}(0) + B_1\boldsymbol{u}(1) \\
\boldsymbol{x}(3) &= A_2 A_1 A_0\boldsymbol{x}(0) + A_2 A_1 B_0\boldsymbol{u}(0) + A_2 B_1\boldsymbol{u}(1) + B_2\boldsymbol{u}(2) \\
\boldsymbol{x}(4) &= A_3 A_2 A_1 A_0\boldsymbol{x}(0) + A_3 A_2 A_1 B_0\boldsymbol{u}(0) + A_3 A_2 B_1\boldsymbol{u}(1) + A_3 B_2\boldsymbol{u}(2) + B_3\boldsymbol{u}(3) \\
&\;\;\vdots \\
\boldsymbol{x}(h) &= \left(\prod_{i=h-1}^{0}A_i\right)\boldsymbol{x}(0) + \sum_{i=0}^{h-2}\left[\left(\prod_{j=h-1}^{i+1}A_j\right)B_i\boldsymbol{u}(i)\right] + B_{h-1}\boldsymbol{u}(h-1)
\end{aligned}
\tag{3.31}
$$
> 连乘记号 $\prod_{i=h-1}^{0}A_i = A_{h-1}A_{h-2}\cdots A_0$（**下标递减**，即按时间逆序左乘）。

未来 $h$ 个控制量组合成向量（源 (3.32)）：
$$
U = \begin{bmatrix} \boldsymbol{u}^{T}(0) & \boldsymbol{u}^{T}(1) & \dots & \boldsymbol{u}^{T}(h-1) \end{bmatrix}^{T}
\tag{3.32}
$$
**预测方程**（紧凑矩阵形式，源 (3.33)）：
$$
\mathrm{X} = A_{qp}\boldsymbol{x}(0) + B_{qp}\mathrm{U}
\tag{3.33}
$$
> $\boldsymbol x(0)$ 为当前时刻状态（来自状态估计器）；$\mathrm{U}$ 为预测时域 $h$ 内控制量，$12h$ 行 1 列；$\mathrm{X}$ 为未来 $h$ 周期状态，$13h$ 行 1 列；$A_{qp}$ 状态系数矩阵 $13h$ 行 13 列；$B_{qp}$ 控制系数矩阵 $13h$ 行 $12h$ 列。
> **⚠OCR 注**：源正文 (3.33) 段落中所有维度数字（"$\square$ 行 $\square$ 列"）均被 MinerU 丢失，上述维度由本人据 13 维状态/12 维控制/$h$ 步预测补全。

各量展开（源 (3.34)）：
$$
\mathrm{X} = \begin{bmatrix} \boldsymbol{x}(1) \\ \boldsymbol{x}(2) \\ \vdots \\ \boldsymbol{x}(h) \end{bmatrix},\quad
A_{qp} = \begin{bmatrix} A_0 \\ A_1 A_0 \\ A_2 A_1 A_0 \\ \vdots \\ \prod_{i=h-1}^{0}A_i \end{bmatrix},\quad
B_{qp} = \begin{bmatrix}
B_0 & \mathbf{0} & \dots & \mathbf{0} \\
A_1 B_0 & B_1 & \dots & \mathbf{0} \\
\vdots & \vdots & \ddots & \mathbf{0} \\
\left(\prod_{j=h-1}^{1}A_j\right)B_0 & \left(\prod_{j=h-1}^{2}A_j\right)B_1 & \dots & B_{h-1}
\end{bmatrix}
\tag{3.34}
$$

---

## 5. §3.4 带约束的二次凸优化（源 line 1614–1686）

### 5.1 优化问题构建（源 line 1616–1620）

由 (2.77) 已得未来 $h$ 周期期望轨迹 $D$，而 (3.33) 中未来状态 $\mathrm X$ 仅取决于未来控制量 $\mathrm U$。目标：找 $\mathrm U$ 使 $\mathrm X$ 与 $D$ 的差尽可能为 0，则 $\mathrm U$ 第一组控制量 $\boldsymbol u(0)$ 作为当前控制周期控制量。结合 (3.9)(3.13) 构建优化问题（源 (3.35)）：
$$
\begin{aligned}
\min_{U}\; & J(U) = (X - D)^{T}Q(X - D) + U^{T}R U \\
\text{s.t.}\quad & {}^{\mathcal{O}}\boldsymbol{f}_i = \mathbf{0}_{3\times1},\quad \forall i\; s_\Phi = 0 \\
& \underline{c}_i \leqslant C_i\cdot{}^{\mathcal{O}}\boldsymbol{f}_i \leqslant \overline{c}_i
\end{aligned}
\tag{3.35}
$$
> **被引用前置 (2.76)(2.77)（期望状态/轨迹，源 line 1096/1102）**：
> 第 $k$ 周期期望状态 ${}_d\boldsymbol{x}^k = \left[{}_d\boldsymbol{\Theta}^k;\ {}_d\boldsymbol{p}_{com}^k;\ {}_d\boldsymbol{\omega}^k;\ {}_d\boldsymbol{v}_{com}^k;\ {}^{\mathcal{O}}\mathbf{g}(3)\right]$（13 维，末位常数 ${}^{\mathcal O}\mathbf g(3)=-9.8\,\mathrm{m/s^2}$）；
> 期望轨迹 $D = \left[({}_d\boldsymbol{x}^1)^{T}\ ({}_d\boldsymbol{x}^2)^{T}\ \dots\ ({}_d\boldsymbol{x}^h)^{T}\right]^{T}$，$13h$ 行 1 列。

### 5.2 权重矩阵（源 line 1622–1638）

**$J$ 第一项 $(X-D)^TQ(X-D)$**：让 $X$ 与 $D$ 的差尽可能为 0。$Q$ 为反馈权重矩阵，$13h\times13h$ 对角矩阵，$Q(i,i)$ 越大则保证 $X$ 与 $D$ 第 $i$ 分量差距越小。工程实现上不同控制周期同一控制闭环权重相等（源 (3.36)，`%` 为取余）：
$$
Q(i,i) = Q(i\%13,\ i\%13)
\tag{3.36}
$$
$Q$ 前 13 个对角元素（源 (3.37)，对应状态序 $[\phi,\theta,\psi,\,p_x,p_y,p_z,\,\omega_x,\omega_y,\omega_z,\,v_x,v_y,v_z,\,g]$）：
$$
Q.\mathrm{diagonal}(0,13) = \begin{bmatrix} 25 & 25 & 10 & 1 & 1 & 100 & 0 & 0 & 0.3 & 0.2 & 0.2 & 20 & 0 \end{bmatrix}^{T}
\tag{3.37}
$$

**$J$ 第二项 $U^TRU$**：保证控制量 $U$ 不能太大（否则持续高扭矩导致电机发热、精度下降、机构寿命缩短、噪音大）。$R$ 为控制量权重矩阵，$12h\times12h$ 对角矩阵（源 (3.38)）：
$$
R = 0.00005\cdot\mathbb{I}_{12h\times12h}
\tag{3.38}
$$

### 5.3 化为二次规划一般形式（源 line 1640–1656）

将 (3.33) 代入 (3.35) 的 $J(U)$（源 (3.39)）：
$$
\begin{aligned}
J(U) &= (X - D)^{T}Q(X - D) + U^{T}R U \\
&= \left(\underbrace{A_{qp}\boldsymbol{x}_0 - D}_{E} + B_{qp}\mathrm{U}\right)^{T}Q\left(A_{qp}\boldsymbol{x}_0 - D + B_{qp}\mathrm{U}\right) + U^{T}R U \\
&= (E + B_{qp}\mathrm{U})^{T}Q(E + B_{qp}\mathrm{U}) + U^{T}R U \\
&= E^{T}Q E + (B_{qp}\mathrm{U})^{T}Q(B_{qp}\mathrm{U}) + 2E^{T}Q(B_{qp}\mathrm{U}) + U^{T}R U \\
&= \mathrm{U}^{T}B_{qp}^{T}Q B_{qp}\mathrm{U} + U^{T}R U + 2E^{T}Q B_{qp}\mathrm{U} + E^{T}Q E \\
&= \mathrm{U}^{T}\left(B_{qp}^{T}Q B_{qp} + R\right)\mathrm{U} + U^{T}\left(2E^{T}Q B_{qp}\right)^{T} + \underbrace{E^{T}Q E}_{\text{const}}
\end{aligned}
\tag{3.39}
$$
> 其中 $E := A_{qp}\boldsymbol{x}_0 - D$（$\boldsymbol{x}_0$ 是 $\boldsymbol x(0)$ 的简写）；用了 $Q=Q^T$（对角阵）。

令（源 (3.40)）：
$$
\begin{cases}
H = 2\left(B_{qp}^{T}Q B_{qp} + R\right) \\
g = \left(2E^{T}Q B_{qp}\right)^{T} = 2B_{qp}^{T}Q\left(A_{qp}\boldsymbol{x}_0 - D\right)
\end{cases}
\tag{3.40}
$$
$E^TQE$ 不是 $U$ 的函数，对「$U$ 取何值时 $J$ 最小」无影响，直接舍去。则（源 (3.41)）：
$$
J(U) = \frac{1}{2}U^{T}H U + U^{T}g
\tag{3.41}
$$

### 5.4 约束整合与降维技巧（源 line 1658–1686）

将 (3.13) 对足底力约束整合成对 $U$ 的约束（源 (3.42)）：
$$
\underline{c} \leqslant CU \leqslant \overline{c}
\tag{3.42}
$$
其中（源 (3.43)，块对角，$h$ 个周期 × 4 腿，每腿 5 行约束）：
$$
\underline{c} = \mathbf{0}_{20h\times1},\quad
C = \begin{bmatrix} C_i & \dots & \mathbf{0} \\ \vdots & \ddots & \vdots \\ \mathbf{0} & \dots & C_i \end{bmatrix},\quad
\bar{c} = \begin{bmatrix} \overline{c}_i \\ \overline{c}_i \\ \vdots \\ \overline{c}_i \end{bmatrix}
\tag{3.43}
$$
> 约束行数 $20h$ = 5(每腿)×4(腿)×$h$(周期)。

**降维技巧（line 1670）**：由 (3.9) 摆动腿足底力为 0，对角步态每时刻摆动/支撑各 2 腿，故 $U$ 有一半元素为 0，若参与优化会降低效率。工程做法：若第 $k$ 周期第 $i$ 腿为摆动腿，则
- 将 $H$ 和 $g$ 中第 $n_{ki}\sim n_{ki}+2$ 个元素剔除；
- 将 $H$ 中第 $n_{ki}\sim n_{ki}+2$ 行和列剔除；
- 将 $C$ 中第 $n_{ki}\sim n_{ki}+2$ 列剔除；
- 再将 $C$ 中全为 0 的行以及 $\underline{c}$、$\overline c$ 中对应元素剔除。

其中（源 (3.44)）：
$$
n_{ki} = 12(k-1) + 3(i-1) + 1
\tag{3.44}
$$
> 本文不再另设符号表示剔除后的矩阵。

**二次规划一般形式**（源 (3.45)）：
$$
\begin{aligned}
\min_{U}\; & J(U) = \frac{1}{2}U^{T}H U + U^{T}g \\
\text{s.t.}\quad & \underline{c} \leqslant CU \leqslant \overline{c}
\end{aligned}
\tag{3.45}
$$
> **求解器**：工程实现使用基于 C++ 的 **qpOASES** 库。

**MPC 输出**（源 line 1682–1686）：求解出的 $U$ 第一组控制量 $\boldsymbol u(0)$ 作为当前控制周期控制量。因用了剔除元素降维，$\boldsymbol u(0)$ 大小不再是 12 行 1 列，而是 $3n_{st}$ 行 1 列（$n_{st}$ 为此时支撑腿数量）。后文用更直观符号表示 MPC 求解的控制量（源 (3.46)）：
$$
{}^{\mathcal{O}}\boldsymbol{f}^{MPC} = \boldsymbol{u}(0)
\tag{3.46}
$$

### 5.5 本章小结（源 line 1688）

> 本章首先分析 MPC 在间歇欠驱系统中的必要性；然后将四足机器人简化为单刚体模型并分析受力、列出状态方程；随后据状态方程搭建预测方程，利用预测方程将控制问题转化为优化问题，最终求出足底反力。

---

## 6. 于宪元记号 → 本书记号 映射建议表

> 本书主线：旋转 $\mathbf R\in SO(3)$、**右扰动**为主线、Hamilton 四元数、$\mathfrak{se}(3)$ 序 $[\boldsymbol\rho;\boldsymbol\phi]$。下表给出于宪元（YU）记号到本书（BOOK）记号的迁移建议；MPC 状态/控制/足底力采用清晰记号。

### 6.1 坐标系与基础几何

| 概念 | 于宪元记号 (YU) | 本书建议记号 (BOOK) | 说明 |
|---|---|---|---|
| 世界系 | $\mathcal{O}$ | $\mathcal{W}$（或保留 $\mathcal{O}$） | 本书惯例可用 $\mathcal{W}$（world）；若沿用 $\mathcal{O}$ 需在符号表声明 |
| 本体系 | $\mathcal{B}=\mathcal{B}_1$ | $\mathcal{B}$ | 一致 |
| 定向本体系 | $\mathcal{P}$ | $\mathcal{B}^{\circ}$（orientation-aligned body）或保留 $\mathcal{P}$ | 方向同世界、原点同本体；建议显式命名"定向本体系" |
| 足底系 | $\mathcal{C}_i$ | $\mathcal{C}_i$ | 一致 |
| 旋转矩阵（B→A） | ${}^{\mathcal{A}}R_{\mathcal{B}}$ | $\mathbf{R}_{\mathcal{AB}}$ 或 ${}_{\mathcal A}\mathbf R_{\mathcal B}$ | 本书用粗体 $\mathbf R\in SO(3)$；建议保留左上/右下角标结构，仅旋转字母加粗 |
| 旋转群 | （未显式） | $\mathbf{R}\in SO(3)$ | 本书强调 $SO(3)$ 流形 |
| 反对称算子 | $\boldsymbol{a}_\times$（下标 $\times$） | $\boldsymbol{a}^{\wedge}$ 或 $[\boldsymbol a]_\times$ | 本书 $\mathfrak{so}(3)$ 主线建议用 hat $(\cdot)^\wedge$；与 $\mathfrak{se}(3)$ 序 $[\rho;\phi]$ 协调 |
| 角速度张量 | ${}^{\mathcal{A}}\Omega_{\mathcal{B}}$ | $\boldsymbol{\omega}^{\wedge}$ | 即 $\boldsymbol\omega^\wedge=\dot{\mathbf R}\mathbf R^T$ |
| 角速度矢量 | ${}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}}$ | ${}_{\mathcal A}\boldsymbol{\omega}_{\mathcal{AB}}$ | 一致结构 |
| 角速度的反对称写法 | ${}^{\mathcal{A}}\boldsymbol{\omega}_{\mathcal{AB}\times}$ | $\boldsymbol{\omega}_{\mathcal{AB}}^{\wedge}$ | YU 的 "$\boldsymbol\omega_{\times}$" 等价于本书 hat |
| ZYX 欧拉角 | $\boldsymbol{\Theta}=[\phi,\theta,\psi]^T$ | $\boldsymbol{\Theta}=[\phi,\theta,\psi]^T$ | 一致（roll/pitch/yaw） |

> **右扰动一致性提示**：YU 全程用 $\dot{\mathbf R}\mathbf R^T=\boldsymbol\omega^\wedge$（**世界系/左扰动**形式的角速度，body-to-world 的空间角速度）。本书主线为**右扰动**（$\dot{\mathbf R}=\mathbf R\,\boldsymbol\omega_b^\wedge$，body 角速度）。写章时需注意：YU 的 ${}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}}$ 是**空间角速度**（spatial，世界系下表达）。若要切到本书右扰动主线，应明确 ${}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}} = \mathbf R\,{}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}}$，并在欧拉方程/映射处统一。建议在范本章里**保留 YU 的空间角速度表述**（因 MPC 状态在世界系最自然），但加一个 remark 指出与右扰动主线的换算关系。

### 6.2 MPC 专有量

| 概念 | 于宪元记号 (YU) | 本书建议记号 (BOOK) | 说明 |
|---|---|---|---|
| 质心位置（世界系） | ${}^{\mathcal{O}}\boldsymbol{p}_{com}$ | ${}_{\mathcal W}\boldsymbol{p}_{c}$ | $c$=CoM |
| 质心速度 | ${}^{\mathcal{O}}\dot{\boldsymbol{p}}_{com}$ | ${}_{\mathcal W}\boldsymbol{v}_{c}$ | 与 (2.76) 的 ${}_d\boldsymbol v_{com}$ 协调 |
| 足底反力（第 $i$ 腿，世界系） | ${}^{\mathcal{O}}\boldsymbol{f}_i$ | ${}_{\mathcal W}\boldsymbol{f}_i$ | 接触力（GRF） |
| 质心→足底向量 | ${}^{\mathcal{O}}\boldsymbol{r}_i$ | ${}_{\mathcal W}\boldsymbol{r}_i$ | $=\boldsymbol p_i-\boldsymbol p_c$ |
| 惯性张量（本体系/定向本体系） | ${}^{\mathcal{B}}I$ / ${}^{\mathcal{P}}I$ | ${}_{\mathcal B}\mathbf{I}$ / ${}_{\mathcal B^\circ}\mathbf{I}$ | 粗体 $\mathbf I$ 区别于单位阵 $\mathbb I$ |
| MPC 状态向量（13 维） | $\boldsymbol{x}$ | $\boldsymbol{x}$ | 序 $[\boldsymbol\Theta;\boldsymbol p_c;\boldsymbol\omega;\boldsymbol v_c;g]$ |
| MPC 控制向量（12 维） | $\boldsymbol{u}=[{}^{\mathcal O}\boldsymbol f_1;\dots;{}^{\mathcal O}\boldsymbol f_4]$ | $\boldsymbol{u}=[\boldsymbol f_1;\dots;\boldsymbol f_4]$ | 4 腿 GRF 堆叠 |
| 连续/离散系统矩阵 | $A_c,B_c$ / $A_k,B_k$ | $\mathbf A_c,\mathbf B_c$ / $\mathbf A_k,\mathbf B_k$ | 粗体 |
| 预测堆叠 | $\mathrm{X},\mathrm{U},A_{qp},B_{qp},D$ | $\mathbf X,\mathbf U,\mathbf A_{qp},\mathbf B_{qp},\mathbf D$ | QP 堆叠量 |
| QP 海森/梯度 | $H,g$ | $\mathbf H,\boldsymbol g$ | |
| 权重 | $Q,R$ | $\mathbf Q,\mathbf R$ | $\mathbf R$ 与旋转矩阵 $\mathbf R$ 冲突！**本书须改名**，建议控制权重用 $\mathbf W_u$、状态权重用 $\mathbf W_x$（或 $\mathbf Q,\mathbf R_u$） |
| 摩擦约束矩阵 | $C_i,\underline c_i,\overline c_i$ | $\mathbf C_i,\underline{\mathbf c}_i,\overline{\mathbf c}_i$ | |
| 支撑相布尔量 | $_i s_\Phi$ | $s_i\in\{0,1\}$ 或 $c_i$（contact） | 1=支撑(stance)，0=摆动(swing) |
| 摩擦系数/力上限 | $\mu,f_{\max}$ | $\mu,f_{\max}$ | 一致 |
| MPC 周期 | $\Delta T$ | $\Delta T$ | $=0.01$ s |
| 预测时域步数 | $h$ | $N$ 或 $h$ | 本书 MPC 惯例常用 $N$（horizon） |
| 期望偏航 | ${}_d\psi^k$ | $\psi_d^k$ | 左下标 $d$→下标 $d$ |
| MPC 输出力 | ${}^{\mathcal O}\boldsymbol f^{MPC}$ | ${}_{\mathcal W}\boldsymbol f^{\mathrm{MPC}}$ | |

> **关键命名冲突警示**：YU 的控制权重矩阵 $R$ 与旋转矩阵 $R$ 同字母。本书旋转一律粗体 $\mathbf R\in SO(3)$，故控制权重**必须改名**（建议 $\mathbf R_u$ 或 $\mathbf W_u$），避免读者混淆。

---

## 7. OCR / 渲染存疑清单（写章时需复核或修正）

1. **(3.2) 重力力矩项**：源作 $\boldsymbol{\theta}_{3\times1}\times(m{}^{\mathcal O}\boldsymbol g)$，物理上应为 $\mathbf{0}_{3\times1}\times(m{}^{\mathcal O}\boldsymbol g)=\mathbf 0$（重力过质心、对质心无矩）。疑 MinerU 把粗体 $\mathbf 0$ 误识为 $\boldsymbol\theta$。**本书取 $\mathbf 0$，该项整体为零**，写章时可直接说明"重力对质心无力矩"并略去。
2. **(3.8) 质心下标**：源右端 ${}_i^{\mathcal O}\boldsymbol p_{com}$ 带腿编号 $i$，应为 ${}^{\mathcal O}\boldsymbol p_{com}$（质心无腿编号）。本书去掉 $i$。
3. **(3.25) 的 $A_c$ 第 5 列 / 末行**：MinerU 渲染出现 `0/1/\vdots/\cdots` 混排，结构不清。已据 (3.24)→(3.25) 的增广逻辑（${}^{\mathcal O}\boldsymbol g(3)$ 作第 13 状态、仅注入 $\dot z$ 通道）重排为规整形式（第 4 块行第 5 列 = $[0,0,1]^T$，末行全 0）。写章须显式给出规整后的 $13\times13$ 矩阵。
4. **(3.29) 的 $A_k$**：同理，重力注入列（第 5 列）在 $\dot v$ 块应为 $[0,0,\Delta T]^T$，末行 $[0,\dots,0,1]$。源渲染含 `\vdots`，已规整。
5. **(3.33) 段维度**：源正文所有"□ 行 □ 列"维度数字被 MinerU 丢失。已据 13/12/$h$ 补全（$\mathrm X:13h\times1$，$\mathrm U:12h\times1$，$A_{qp}:13h\times13$，$B_{qp}:13h\times12h$，$D:13h\times1$）。
6. **§3.4 正文符号丢失**：(3.35)(3.36) 周围多处行内变量名（$X$、$D$、$U$、$Q$、$Q(i,i)$ 等）在 MinerU 输出中被渲染为空/乱码（如 "$^{66}U$"、"$|g\rrangle$"），已据上下文逻辑还原。写章时以本文件还原版为准。
7. **(3.18) 末项基向量**：$\boldsymbol e_1,\boldsymbol e_2,\boldsymbol e_3$ 在 (3.16) 定义为 ${}^{\mathcal O}R_{\mathcal B}$ 的三列，但 (3.18)(3.19)(3.20) 中 $(\boldsymbol e_1\dot\phi),(\boldsymbol e_2\dot\theta),(\boldsymbol e_3\dot\psi)$ 的 $\boldsymbol e_k$ 实为**基本轴单位向量** $\hat x,\hat y,\hat z$（因 $\dot R_z R_z^T=(\hat z\dot\psi)_\times$）。源此处记号一符两义（既是 $R$ 的列、又是基本轴），写章须澄清：在 $\dot R R^T$ 推导里 $\boldsymbol e_k$ 取标准基 $\hat x,\hat y,\hat z$。
8. **图片**：图29–33 均为 MinerU 截图链接（受力分析图31、摩擦锥图33、欠驱步态图30），本文件未内嵌；写章配图建议用本书 figs 子系统（TikZ）重绘单刚体受力图与摩擦锥金字塔近似图。
9. **${}^{\mathcal P}I$ 微分 (3.14) 第二步**：源把 ${}^{\mathcal O}\dot R_{\mathcal B}^T$ 写为 ${}^{\mathcal O}R_{\mathcal B}^T({}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}\times})^T$，是对 ${}^{\mathcal O}\dot R_{\mathcal B}={}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}\times}{}^{\mathcal O}R_{\mathcal B}$ 取转置的结果，推导正确，无误，仅记号密集。

---

## 8. 写章要点速记（给 §9 流水线下游）

- **逻辑主线**：间歇欠驱 → 需预测 → 单刚体简化(12态) → 平动/转动动力学 → 惯性张量转动变换(常值 $\to$ 姿态相关) → 摩擦锥线性化(非线性→4线性) → 陀螺项小角度忽略得线性欧拉方程 → 角速度↔欧拉角微分映射(小角度近似 $R_z^T(\psi)$) → 整合连续状态方程(增广重力为第13态) → 前向欧拉离散 → 递推得预测方程(QP堆叠) → 跟踪+力正则的带约束 QP → qpOASES 求解 → 取 $\boldsymbol u(0)$。
- **三处关键近似**（写章须各加 remark 与适用条件）：①陀螺项 $\boldsymbol\omega_\times I\boldsymbol\omega\approx0$（小角速度）；②欧拉角映射 $\approx R_z^T(\psi)$（小 roll/pitch）；③每腿支撑期足底坐标不变 ⇒ 预测时域 ≤ 半迈步周期。
- **数值例可直接复现**：惯性张量 (3.7)、$Q$ 对角 (3.37)、$R=5\times10^{-5}\mathbb I$ (3.38)、$\Delta T=0.01$ s、${}^{\mathcal O}g(3)=-9.8$、尺寸 (2.2)。
- **复现级关键**：维度 $A_k\,13\times13$、$B_k\,13\times12$、约束 $20h$ 行、降维索引 $n_{ki}=12(k-1)+3(i-1)+1$。
- **本书须改名项**：控制权重 $R\to\mathbf R_u$（避撞旋转 $\mathbf R$）；视情况世界系 $\mathcal O\to\mathcal W$、horizon $h\to N$。
