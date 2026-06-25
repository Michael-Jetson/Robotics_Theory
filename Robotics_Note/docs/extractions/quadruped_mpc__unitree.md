# 抽取：四足机器人 单刚体模型 / 平衡控制 / 足底力 —— 宇树《四足机器人控制算法》

> **源文件**：`MinerU_markdown_四足机器人控制算法_(杭州宇树科技有限公司组编 卞泽坤,王兴兴编著)_..._2069364125957193728.md`（5878 行，11 章）
> **角色定位**：本部「四足机器人运动控制」范本章=单刚体模型 MPC 的 **实践 / 对照源**（§9 流水线）。工程向、教材级、配套 `unitree_guide` 开源代码。
> **抽取日期**：2026-06-23
> **抽取原则**：MPC / 平衡控制 / 单刚体动力学 / 足底力相关 = 全量保真（每式每步每图含义全录，标源章节）；纯工程内容（电机 FOC / 配置 / 报文、ROS/Gazebo/编译/网络/OOP/绘图）= 一句话登记备用。

---

## 0. 全书章目辨认（MinerU 漏识修正）+ 相关性裁定

MinerU 把若干章标题拆成多个 `##` 行（如「第5章」与「单腿的运动学与静力学」分两行），下表据正文 + 目录（源 84–176 行）还原，并标注与 **单刚体动力学/MPC/平衡控制/足底力** 的相关性。

| 章 | 标题 | 源行 | 相关性 | 处置 |
|---|---|---|---|---|
| 第1章 | 四足机器人概述及实践准备 | 180 | 无 | 工程登记 |
| 第2章 | 关节电机（FOC/编码器/混合控制/配置/报文） | 247 | 无 | 工程登记 |
| 第3章 | 机器人仿真与控制框架（ROS/Gazebo/OOP/FSM/编译/网络） | 645 | 无（FSM 站立=位置控制背景） | 工程登记 |
| 第4章 | 刚体运动学（旋转矩阵/指数坐标/齐次变换/欧拉角） | 1467 | **基础**（被 ch6/ch8 引用：式 4.33/4.36/4.39/4.71/4.91） | 数学前置，已在李群/运动学章覆盖 |
| 第5章 | 单腿的运动学与静力学 | 2529 | **强相关**（§5.3.2 单腿静力学 = ch8 简化逆动力学的核心；§5.4 摆动腿力控） | **抽**（§3 录） |
| 第6章 | 四足机器人的运动学与动力学 | 2994 | **核心**（§6.2 单刚体动力学、§6.3 惯性张量、§6.4 动力学简化方程=足底力↔运动） | **全量抽**（§2 录） |
| 第7章 | 四足机器人的状态估计器（IMU/KF/QP/最小二乘/概率） | 3382 | **部分相关**（§7.4 二次规划=ch8 QP 前置；状态估计为控制器提供 R_sb, v_s, ω_s） | QP 抽（§4），KF 仅登记 |
| 第8章 | 平衡控制器 | 4220 | **核心**（机身位姿 PD 反馈→足底力 QP 分配→简化逆动力学→质量属性整定） | **全量抽**（§5 录） |
| 第9章 | 四足机器人的步态与轨迹规划（步态/落脚点/摆线轨迹） | 4664 | 弱相关（为支撑/摆动腿划分提供上下文） | 工程登记 + 一句话 |
| 第10章 | 四足机器人的行走控制器 | 4970 | **关键对照**（§10.7 改进方向明确点出 QP→MPC、静力学→WBC） | **抽改进方向**（§6 录） |
| 第11章 | 四足机器人的感知与导航（距离传感器/ROS nav） | 5263 | 无 | 工程登记 |

> **⚠ 全书最重要的判定**：**宇树这本书没有实现 MPC**。它的平衡控制器是 **单刚体动力学（SRBD）+ 单步（瞬时）QP 足底力分配**，只考虑当前时刻、不做未来预测。MPC 仅在 §10.7.4（源 5258 行）作为「改进方向」被一句话提及。因此本书对范本章的价值是：
> 1. **SRBD 建模的完整一手推导**（ch6，比多数 MPC 论文详尽）；
> 2. **QP 足底力分配 + 摩擦锥约束**（ch8）——这正是凸 MPC 每个时间步内核的「N=1 退化版」，是 MPC 的直接前身/对照；
> 3. **工程落地细节**（简化逆动力学、质量属性实验整定）——MPC 论文通常略过。

---

## 1. 建模符号与坐标系约定（源 §6.1.1，行 3004–3049）

- **世界（惯性）坐标系 $\{s\}$**：地面固定点 $X$ 为原点。一般取 0 号足端 $P_0$ 为 $X$，坐标轴与初始机身坐标系平行，故初始 $R_{sb}=I$。
- **机身坐标系 $\{b\}$**：原点在机身形心，$x$ 朝头部，$y$ 朝左，$z=x\times y$ 朝上。**关键约定**：$\{b\}$ 被定义为一个**静止的惯性系**，只是「在当前时刻」与机身位姿重合（牵连速度与动力学推导需要惯性系，牛顿第二定律只在惯性系成立）。
- 四足端 $P_0\dots P_3$，腿从 **0 号**开始编号（故动力学求和式用 $(i-1)$）。
- $R_{sb}$：机身姿态旋转矩阵；$R_{bs}=R_{sb}^{\mathrm T}$。

**足端在 $\{s\}$ 下的常量坐标**（足不滑移 → $p_{si}$ 不随机身位姿变，源式 6.1）：
$$\boldsymbol p_{si}=\boldsymbol R_{sb}\big[\boldsymbol p_{bi}(0)-\boldsymbol p_{b0}(0)\big]=\boldsymbol p_{bi}(0)-\boldsymbol p_{b0}(0),\quad i=0,1,2,3$$
其中 $p_{bi}(0)$ 为初始时刻足端 $i$ 在 $\{b\}$ 中坐标；该式仅在初始（$\{s\}\parallel\{b\}$）时成立。

**机身位姿齐次变换**（源式 6.2–6.3）：初始 $T_{sb}(0)=\begin{bmatrix}I_{3\times3}&-p_{b0}(0)\\0&1\end{bmatrix}$；一般姿态（roll/pitch/yaw $=\gamma,\beta,\alpha$、平移 $p_d$）：
$$\boldsymbol T_{sb}=\begin{bmatrix}\boldsymbol R_z(\alpha)\boldsymbol R_y(\beta)\boldsymbol R_x(\gamma)&-\boldsymbol p_{b0}(0)+\boldsymbol p_d\\0&1\end{bmatrix}$$
再由 $\boldsymbol p_{bi}=\boldsymbol T_{sb}^{-1}\boldsymbol p_{si}$（式 6.4）算出各足端在 $\{b\}$ 下目标坐标 → 单腿逆运动学 → 关节角（原地姿态控制 = 纯位置控制，FreeStand 状态）。

---

## 2. 单刚体动力学（SRBD）—— 全量推导（源 §6.2，行 3115–3329）

### 2.1 简化假设（源 §6.2.1）
- 忽略部件形变 → 机器人=多刚体系统；严格多刚体须用旋量理论 + Featherstone 算法（RBDL 库），超本书范围。
- **腿质量占比低** → 整机近似为**单刚体**，忽略四条腿对动力学的影响。

### 2.2 质量与重心（源式 6.14–6.15）
密度场 $\rho(x,y,z)$，质量 $m=\int_B\rho\,\mathrm dV$。
重心定义：若 $\{b\}$ 原点取在使 $\int_B\rho\,\boldsymbol p_b\,\mathrm dV=\boldsymbol 0_{3\times1}$ 之点，则该点即重心。动力学默认 $\{b\}$ 建在重心（大幅简化）。推论：$\int_B\rho[\boldsymbol p_b]_\times\mathrm dV$、$\int_B\rho\boldsymbol p_b^{\mathrm T}\mathrm dV$ 也为零。

### 2.3 刚体上一点的速度/加速度（源 §6.1.2，行 3051–3081）
$\{b\}$ 下机身线速度 $v_b$、角速度 $\omega_b$。点 $P$（坐标 $p_P$）速度=平移+绕轴转动叠加，又因当前时刻 $O'$ 与 $\{b\}$ 原点重合（$p_{O'}=0$）：
$$\dot{\boldsymbol p}_P=\boldsymbol v_b+[\boldsymbol\omega_b]_\times\boldsymbol p_P\tag{6.8}$$
$$\ddot{\boldsymbol p}_P=\dot{\boldsymbol v}_b+[\dot{\boldsymbol\omega}_b]_\times\boldsymbol p_P+[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times\boldsymbol p_P\tag{6.9}$$
（推导链：6.5 叠加→6.6 求导→6.7 代入 $\dot p_{O'}=v_b$→6.8/6.9 令 $p_{O'}=0$。）

### 2.4 合外力 → 牛顿方程（源式 6.16–6.18）
$\boldsymbol f_b=\int_B\rho\,\ddot{\boldsymbol p}_b\,\mathrm dV$，把 $v_b,\omega_b$ 及其导数作为积分常量提出，代入质量/重心性质：
$$\boldsymbol f_b=\dot{\boldsymbol v}_b\!\int_B\!\rho\,\mathrm dV+[\dot{\boldsymbol\omega}_b]_\times\!\int_B\!\rho\boldsymbol p_b\,\mathrm dV+[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times\!\int_B\!\rho\boldsymbol p_b\,\mathrm dV\ \Rightarrow\ \boxed{\boldsymbol f_b=m\dot{\boldsymbol v}_b}\tag{6.18}$$
（后两项含 $\int\rho p_b\,\mathrm dV=0$ 而消失。）

### 2.5 合外力矩 → 欧拉方程（源式 6.19–6.22）
$\boldsymbol m_b=\int_B[\boldsymbol p_b]_\times\rho\ddot{\boldsymbol p}_b\,\mathrm dV$ 展开三项：
- 第 1 项 $\propto\int\rho[p_b]_\times\mathrm dV=0$（重心定义）。
- 第 2 项用向量积反交换律（式 4.33）→ 定义**惯性张量** $\boldsymbol I_b=-\int_B\rho[\boldsymbol p_b]_\times[\boldsymbol p_b]_\times\mathrm dV$，得 $\boldsymbol I_b\dot{\boldsymbol\omega}_b$（式 6.20）。
- 第 3 项用二重向量积展开（式 4.36）逐步化简（源式 6.21 共 9 步，标量点积 $\omega_b^{\mathrm T}p_b,\omega_b^{\mathrm T}\omega_b$ 可左右移位）→ $[\boldsymbol\omega_b]_\times\boldsymbol I_b\boldsymbol\omega_b$。

$$\boxed{\boldsymbol m_b=\boldsymbol I_b\dot{\boldsymbol\omega}_b+[\boldsymbol\omega_b]_\times\boldsymbol I_b\boldsymbol\omega_b}\tag{6.22}\quad\text{（旋转刚体的欧拉方程）}$$

### 2.6 单刚体动能（源 §6.2.3，行 3189–3229）
微元动能 $\mathrm dK=\tfrac12\rho\,\boldsymbol v^{\mathrm T}\boldsymbol v\,\mathrm dV$。代入 6.8 积分，中间交叉项含 $\int\rho p_b\,\mathrm dV=0$ 消失，分解为：
- **平移动能** $K_m=\tfrac12 m\,\boldsymbol v_b^{\mathrm T}\boldsymbol v_b$（式 6.25，对应 $\tfrac12 mv^2$）。
- **转动动能** $K_t=\tfrac12\boldsymbol\omega_b^{\mathrm T}\boldsymbol I_b\boldsymbol\omega_b$（式 6.28，二次型，由二重向量积法则化简）。
> 意义：把二次型视为向量的「二次方」，后续 QP 代价函数反复使用此结构。

### 2.7 惯性张量（源 §6.3，行 3231–3280）
分量展开（式 6.29–6.30）：
$$\boldsymbol I_b=\int_B\rho\begin{bmatrix}p_y^2+p_z^2&-p_xp_y&-p_xp_z\\-p_xp_y&p_x^2+p_z^2&-p_yp_z\\-p_xp_z&-p_yp_z&p_x^2+p_y^2\end{bmatrix}\mathrm dV=\begin{bmatrix}I_{xx}&I_{xy}&I_{xz}\\I_{xy}&I_{yy}&I_{yz}\\I_{xz}&I_{yz}&I_{zz}\end{bmatrix}$$
**长方体（形心对齐）简化**（图 6.2，式 6.31）：$\boldsymbol I_b=\frac{1}{12}\mathrm{diag}\big(m(w^2+h^2),\,m(l^2+h^2),\,m(l^2+w^2)\big)$ —— 对角阵。
**工程取值**：单刚体已是简化模型，无需高精度惯性张量；可把机器人等效为匀质长方体，或由三维软件导出后**将非对角元强制置 0**。A1 实例（式 6.32）：$\boldsymbol I_b=\mathrm{diag}(0.1320,\,0.3475,\,0.3775)$。
**坐标变换**（转动动能守恒 $K_{st}=K_{bt}$ 推得，式 6.33–6.34）：
$$\boxed{\boldsymbol I_s=\boldsymbol R_{sb}\,\boldsymbol I_b\,\boldsymbol R_{sb}^{\mathrm T}}\tag{6.34}$$
（$\{b\}$ 下 $I_b$ 是常量；世界系下随姿态变。）

### 2.8 动力学简化方程（足底力 ↔ 机身运动）—— **MPC 的预测模型核心**（源 §6.4，行 3282–3329）

精确 SRBD（式 6.35）含非线性项 $[\omega_b]_\times I_b\omega_b$。**线性化手段**：可用基于变分法的线性化；但四足正常运动 $\omega_b$ 很小，**直接忽略**该项（式 6.36）：
$$\boldsymbol f_b=m\dot{\boldsymbol v}_b,\qquad \boldsymbol m_b=\boldsymbol I_b\dot{\boldsymbol\omega}_b$$
转到世界系（式 6.37）：$\boldsymbol f_s=m\dot{\boldsymbol v}_s$，$\boldsymbol m_s=\boldsymbol I_s\dot{\boldsymbol\omega}_s=\boldsymbol R_{sb}\boldsymbol I_b\boldsymbol R_{sb}^{\mathrm T}\dot{\boldsymbol\omega}_s$。

引入 $n$ 个触地足端力 $\boldsymbol f_{is}$（地面支撑力+摩擦力的合力，三维向量）、重心→足端向量 $\boldsymbol p_{gi}$、重力 $\boldsymbol g=[0,0,-9.81]^{\mathrm T}$（作用线过重心、无力矩）：
$$\left\{\begin{array}{l}m\boldsymbol g+\sum_{i=1}^{n}\boldsymbol f_{(i-1)s}=m\dot{\boldsymbol v}_s\\[4pt]\sum_{i=1}^{n}[\boldsymbol p_{g(i-1)}]_\times\boldsymbol f_{(i-1)s}=\boldsymbol R_{sb}\boldsymbol I_b\boldsymbol R_{sb}^{\mathrm T}\dot{\boldsymbol\omega}_s\end{array}\right.\tag{6.38}$$

**重心→足端向量**（重心未必在机身中心，图 6.3，式 6.39）：
$$\boldsymbol p_{g(i-1)}=\boldsymbol R_{sb}\{\overrightarrow{P_bP_{(i-1)}}\}_b-\boldsymbol R_{sb}\{\overrightarrow{P_bP_g}\}_b$$
（$\{\overrightarrow{P_bP_{(i-1)}}\}_b$ 由单腿正运动学算得；$\{\overrightarrow{P_bP_g}\}_b$ 为常量。）

**矩阵形式（→ 直接喂给 QP/MPC，式 6.40）**：
$$\begin{bmatrix}\boldsymbol I&\boldsymbol I&\boldsymbol I&\boldsymbol I\\ [\boldsymbol p_{g0}]_\times&[\boldsymbol p_{g1}]_\times&[\boldsymbol p_{g2}]_\times&[\boldsymbol p_{g3}]_\times\end{bmatrix}\begin{bmatrix}\boldsymbol f_{0s}\\\boldsymbol f_{1s}\\\boldsymbol f_{2s}\\\boldsymbol f_{3s}\end{bmatrix}=\begin{bmatrix}m(\dot{\boldsymbol v}_s-\boldsymbol g)\\\boldsymbol R_{sb}\boldsymbol I_b\boldsymbol R_{sb}^{\mathrm T}\dot{\boldsymbol\omega}_s\end{bmatrix}\tag{6.40}$$
- 腾空腿对应足端力置零向量。
- **维度矛盾**：左侧 6 行（3 力 + 3 力矩）方程，但 4 足端力 = 12 未知数 → **欠定，解不唯一** → 必须用 **QP** 选最优解（见 §5）。
- 缺口：式 6.40 需要 $R_{sb}$、$v_s$、$\omega_s$，由 ch7 状态估计器提供。

---

## 3. 单腿静力学 / 雅可比（= ch8 简化逆动力学的内核，源 §5.3，行 2844–2925）

**雅可比矩阵**（式 5.42）：三自由度单腿，足端线速度与关节角速度线性关系
$$[\dot x_P,\dot y_P,\dot z_P]^{\mathrm T}=\boldsymbol J\,[\dot\theta_1,\dot\theta_2,\dot\theta_3]^{\mathrm T}$$
$\boldsymbol J$ 为 $3\times3$（由正运动学式 5.11 对 $t$ 求导得到，式 5.38–5.41）；A1/Go1 关节限位内 $\boldsymbol J$ 恒可逆 → 逆微分 $\dot\theta=\boldsymbol J^{-1}\dot p$（式 5.43）。

**单腿静力学**（源 §5.3.2，行 2883–2913）：腿静止 → 总功率 0（流入功率=流出功率）：
$$\boldsymbol\tau^{\mathrm T}\dot{\boldsymbol\theta}=\boldsymbol F^{\mathrm T}\boldsymbol J\dot{\boldsymbol\theta}\ \Rightarrow\ \boxed{\boldsymbol\tau=\boldsymbol J^{\mathrm T}\boldsymbol F}\tag{5.45}$$
- 正/逆：$\boldsymbol\tau=\boldsymbol J^{\mathrm T}\boldsymbol F$（式 5.46），$\boldsymbol F=\boldsymbol J^{-\mathrm T}\boldsymbol\tau$（式 5.47）。
- **符号约定（易错点）**：$\boldsymbol F$=足端**对地面**的作用力（通常向下）；若用地面**对足端**的反作用力 $\boldsymbol F'$，则 $\boldsymbol\tau=-\boldsymbol J^{\mathrm T}\boldsymbol F'$。

**摆动腿力控**（源 §5.4，行 2915–2925）：足端 PD 修正力 $\boldsymbol f_d=\boldsymbol K_p(\boldsymbol p_{0d}-\boldsymbol p_{0f})+\boldsymbol K_d(\dot{\boldsymbol p}_{0d}-\dot{\boldsymbol p}_{0f})$（式 5.48），再 $\boldsymbol\tau=\boldsymbol J^{\mathrm T}\boldsymbol f_d$。

**足端速度合成**（源 §6.1.3，式 6.10–6.13）：牵连速度 $\boldsymbol v_{be}=\boldsymbol v_b+[\boldsymbol\omega_b]_\times\boldsymbol p_{bfB}$ + 关节速度 $\boldsymbol v_{bj}=\boldsymbol J\dot{\boldsymbol\theta}$，旋到世界系 $\boldsymbol v_{sf}=\boldsymbol v_s+\boldsymbol R_{sb}([\boldsymbol\omega_b]_\times\boldsymbol p_{bfB}+\boldsymbol J\dot{\boldsymbol\theta})$；因 $v_s$ 不可直接测，实用量为相对机身速度 $\boldsymbol v_{sfB}=\boldsymbol R_{sb}([\boldsymbol\omega_b]_\times\boldsymbol p_b+\boldsymbol J\dot{\boldsymbol\theta})$（式 6.13，供状态估计器用）。

---

## 4. 二次规划前置（源 §7.4，行 3569–3601）

无约束标量 QP：$y=ax^2+bx+c$（$a>0$）极小在 $x=-b/2a$（式 7.24）。
向量代价函数 $J=\boldsymbol x^{\mathrm T}\boldsymbol A\boldsymbol x+\boldsymbol b\boldsymbol x+c$（$\boldsymbol A$ 对称正定），$\partial J/\partial\boldsymbol x=2\boldsymbol A\boldsymbol x+\boldsymbol b=0\Rightarrow\boldsymbol x=-\tfrac12\boldsymbol A^{-1}\boldsymbol b$（式 7.25–7.27，无约束理论解，KF 推导用）。带约束数值 QP 见 §5。

---

## 5. 平衡控制器（核心）—— 机身位姿 PD → 足底力 QP → 简化逆动力学（源 §8，行 4220–4583）

### 5.0 动机（源章首，行 4222）
固定站立（§3.4）与原地姿态控制（§6.5）都是**足端位置控制**——「四条腿的桌子」「能伸缩桌腿的桌子」，在崎岖地形站不稳。故需建立基于**足端力控制**而非位置控制的平衡控制器。

### 5.1 机身位置反馈控制（源 §8.1.1，行 4230–4369）
思路：把 $\dot{\boldsymbol v}_s,\dot{\boldsymbol\omega}_s$ 视为可控输入（由足底力经式 6.40 生成），先设计期望加速度。
离散运动学（dt 内，式 8.1–8.2）→ 以位置偏差 $\Delta p_k=r_k-p_k$、速度偏差 $\Delta\dot p_k$ 为状态，建离散状态空间（式 8.3）；三轴解耦，取 $x$ 轴（式 8.4）：
$$\begin{bmatrix}\Delta x_{k+1}\\\Delta\dot x_{k+1}\end{bmatrix}=\begin{bmatrix}1&\mathrm dt\\0&1\end{bmatrix}\begin{bmatrix}\Delta x_k\\\Delta\dot x_k\end{bmatrix}-\begin{bmatrix}(\mathrm dt)^2/2\\\mathrm dt\end{bmatrix}\ddot x_k$$
**PD 控制律**：$\ddot x_k=k_p\Delta x_k+k_d\Delta\dot x_k$（式 8.5，$k_p,k_d>0$）→ 闭环 $\boldsymbol x_{k+1}=\boldsymbol A\boldsymbol x_k$（式 8.6）。

**稳定性分析（全量保真）**：$\boldsymbol x_k=\boldsymbol A^k\boldsymbol x_0=\boldsymbol P\,\mathrm{diag}(\lambda_1^k,\lambda_2^k)\,\boldsymbol P^{-1}\boldsymbol x_0$（对角化，式 8.7）。
- 收敛要求 $|\lambda_1|,|\lambda_2|<1$；四足平衡**不希望振荡** → 要求两特征值均为 $(0,1)$ 区间**实数**（笔记，行 4283）。
- 特征方程 $a\lambda^2+b\lambda+c=0$（式 8.10）：$a=1,\ b=-2+\tfrac{(\mathrm dt)^2}{2}k_p+\mathrm dt\,k_d,\ c=1+\tfrac{(\mathrm dt)^2}{2}k_p-\mathrm dt\,k_d$。
- 实根且最快收敛 → 令判别式 $b^2-4ac=0$（式 8.12 因式分解），解得**临界阻尼整定**：
$$\boxed{k_d=2\sqrt{k_p}-\frac{\mathrm dt}{2}k_p}\tag{8.13}$$
- 此时重根 $\lambda_1=\lambda_2=1-\mathrm dt\sqrt{k_p}$（式 8.15）；稳定另一条件 $\lambda>0\Rightarrow k_p<1/(\mathrm dt)^2$（式 8.16）。控制器 500 Hz（dt=0.002 s）→ $k_p<250000$，实用值恒满足；$k_p$ 越大收敛越快，但过大致超关节力矩/高频振荡 → 实验定 $k_p$，再由 8.13 算 $k_d$。

**三轴合并**（式 8.17）：$\ddot{\boldsymbol p}=\boldsymbol K_p\Delta\boldsymbol p+\boldsymbol K_d\Delta\dot{\boldsymbol p}$（$\boldsymbol K_p,\boldsymbol K_d$ 对角阵）。

### 5.2 机身姿态反馈控制（源 §8.1.2，行 4371–4413）
姿态 $R_{sb}$ 不可解耦三轴，须整体处理（用李群/李代数，本书只给结论）。
当前→目标姿态差：$\boldsymbol R=\boldsymbol R_d\boldsymbol R_{sb}^{\mathrm T}$（式 8.19），取其**指数坐标 $\hat{\boldsymbol\omega}\theta$**（用 §4.3.5 矩阵对数）作为姿态偏差。
- P 控制：$\dot{\boldsymbol\omega}=k_p\hat{\boldsymbol\omega}\theta=k_p\theta\hat{\boldsymbol\omega}$（式 8.20，$k_p$ 标量；几何意义=沿 $\hat\omega$ 轴产生角加速度驱使 $R_{sb}\to R_d$）。
- D 控制：$\dot{\boldsymbol\omega}=\boldsymbol K_d(\boldsymbol\omega_d-\boldsymbol\omega)$（式 8.21，$\boldsymbol K_d$ 对角阵）。
- **PD 合并**（式 8.22）：
$$\boxed{\dot{\boldsymbol\omega}=k_p\hat{\boldsymbol\omega}\theta+\boldsymbol K_d(\boldsymbol\omega_d-\boldsymbol\omega)}$$
> 小结（行 4413）：位置控制需机身产生期望 $\ddot{\boldsymbol p}$，姿态控制需期望 $\dot{\boldsymbol\omega}$；二者皆通过**控制各足底力**实现。

### 5.3 足端力控制 + 摩擦锥约束（源 §8.2，行 4415–4446）—— **MPC 不等式约束的来源**
触地足端受力 $\boldsymbol f_{is}=[F_x,F_y,F_z]^{\mathrm T}$（$F_z$ 竖直支撑、$F_x,F_y$ 摩擦）。静摩擦系数 $\mu$、不滑移、不腾空 → 约束（式 8.23）：$-\mu F_z<F_x<\mu F_z,\ -\mu F_z<F_y<\mu F_z,\ 0<F_z$。
矩阵形式（式 8.24）：
$$\begin{bmatrix}1&0&\mu\\-1&0&\mu\\0&1&\mu\\0&-1&\mu\\0&0&1\end{bmatrix}\boldsymbol f_{is}\geqslant\boldsymbol 0\quad(\text{记 }\boldsymbol F_\mu\boldsymbol f_{is}\geqslant 0)$$
- 可行域=**摩擦四棱锥**（图 8.2）。真实摩擦锥是圆锥，此处**线性化为四棱锥**便于 QP（凸 MPC 同款近似）。
- 腾空足端等式约束 $\boldsymbol I\boldsymbol f_{is}=\boldsymbol 0$（式 8.25）。

### 5.4 QP 求解足底力分配（源 §8.3，行 4448–4536）—— **本书 = MPC 的「单步退化」内核**
式 6.40 欠定（6 方程 12 未知）→ 带约束 QP 选最优解。一般约束问题（式 8.26–8.28）：求解 $\boldsymbol A\boldsymbol x-\boldsymbol b=0$，受 $\boldsymbol C_e\boldsymbol x+\boldsymbol c_e=0$、$\boldsymbol C_i\boldsymbol x+\boldsymbol c_i\geqslant0$。
代价函数（式 8.29）：$J=(\boldsymbol A\boldsymbol x-\boldsymbol b)^{\mathrm T}\boldsymbol S(\boldsymbol A\boldsymbol x-\boldsymbol b)+\boldsymbol x^{\mathrm T}\boldsymbol W\boldsymbol x$（$\boldsymbol S,\boldsymbol W$ 正定对角；$\boldsymbol S$ 权方程满足度，$\boldsymbol W$ 权解的幅值；两目标矛盾，靠权重折中）。
化为 QuadProg++ 标准形（式 8.30–8.33）：$\boldsymbol G=\boldsymbol A^{\mathrm T}\boldsymbol S\boldsymbol A+\boldsymbol W,\ \boldsymbol g_0=-\boldsymbol b^{\mathrm T}\boldsymbol S\boldsymbol A$。
> 易错点（行 4502）：线性方程组形式上同等式约束，但**等式约束数不能超未知数**；四腿全腾空有 12 等式约束=未知数上限，故式 6.40 只能放进代价函数、不能当等式约束。

**机器人动力学 QP（式 8.34–8.36）**：$\boldsymbol A,\boldsymbol b_d$ 同式 6.40，缩写 $\boldsymbol A\boldsymbol f-\boldsymbol b_d=0$。代价函数加入**足底力平滑项**：
$$J=(\boldsymbol A\boldsymbol f-\boldsymbol b_d)^{\mathrm T}\boldsymbol S(\boldsymbol A\boldsymbol f-\boldsymbol b_d)+\boldsymbol f^{\mathrm T}\alpha\boldsymbol W\boldsymbol f+(\boldsymbol f-\boldsymbol f_{\text{prev}})^{\mathrm T}\beta\boldsymbol U(\boldsymbol f-\boldsymbol f_{\text{prev}})\tag{8.35}$$
三项含义：① 满足动力学；② 足底力尽量小（$\alpha\boldsymbol W$）；③ 与上步足底力差异小、抑制突变（$\beta\boldsymbol U$，$\boldsymbol f_{\text{prev}}$=上轮解）。等效标准形（式 8.36）：$\boldsymbol G=\boldsymbol A^{\mathrm T}\boldsymbol S\boldsymbol A+\alpha\boldsymbol W+\beta\boldsymbol U$。
**约束装配**：$n$ 腾空腿 → $3n$ 等式约束（式 8.37，对应足端力块=0）；$m$ 触地腿 → $5m$ 不等式约束（式 8.38，块对角 $\boldsymbol F_\mu$）。调 QuadProg++ 解出 $\boldsymbol f$。

> **对照 MPC 的关键**：式 8.35 的代价 = MPC 在单一时间步（N=1）的 stage cost（含力幅值正则 + 平滑项），约束 8.37/8.38 = MPC 的等式/摩擦锥不等式约束。把 6.40 沿预测时域堆叠、加 SRBD 离散状态转移，即得凸 MPC（Di Carlo/Convex MPC）。**本书缺的正是「预测时域」这一维。**

### 5.5 简化逆向动力学（足底力 → 关节力矩，源 §8.4，行 4538–4552）
单腿正/逆动力学超本书范围 → 用 §5.3.2 单腿静力学近似（触地腿速度变化小 + 腿质量小 → 总功率≈0）：
$$\boxed{\boldsymbol\tau_i=-\boldsymbol J_i^{\mathrm T}\boldsymbol f_{ib}=-\boldsymbol J_i^{\mathrm T}\boldsymbol R_{sb}^{\mathrm T}\boldsymbol f_{is}}\tag{8.39}$$
（负号：$\boldsymbol f_{is}$=地面对足端力，与足端对地面力互为反作用。）

### 5.6 简化模型质量属性的实验整定（源 §8.5，行 4554–4583）—— **MPC 论文通常略过的工程关键**
已做简化：单刚体 + 忽略欧拉非线性项 + 静力学近似（忽略腿惯性与重力）→ 需在质量属性上「修修补补」。
- **质量 $m$ 不等于整机/躯干质量**（图 8.3 论证）：足端附加质量 $m_A$ 由地面承担（不计入）；腿上 $m_B$ 需关节克服 → 计入 $rm_B$（$r\in(0,1)$，且与关节角相关）。结论：$m$ 介于躯干质量与整机质量之间，**靠实验定**：减小 $\boldsymbol K_p$ 后站立，站太高→ $m$ 偏大需减小，站太低→增大。
- **重心位置**：减小姿态 $k_p$ 后站立，低头→后腿力大、重心偏后需前移；抬头→后移。
- **转动惯量**：与重力无关、不能靠实验调 → 由三维软件得整机 $I_w$、称重得 $m_w$，按质量比缩放（式 8.40）：$\boldsymbol I_b=\boldsymbol I_w\,\dfrac{m}{m_w}$。
> 收尾（行 4583）：理论上简化的，需在调试中补偿；进阶可了解 **WBC（全身控制）**——不用单刚体简化、考虑所有构件质量属性、免去繁琐参数整定。

---

## 6. QP → MPC / WBC 的对照（源 §10.7 改进方向，行 5245–5258）—— **范本章「对照」骨架**

§10「行走控制器」不增新理论，仅组装：支撑腿用 ch8 平衡控制器维持平衡+驱动机身，摆动腿沿 ch9 摆线轨迹运动；四腿交替（trot 对角步态，State_Trotting）。其 §10.7「改进方向」给出与 MPC/WBC 的明确对照（**直接作为范本章引言/对照素材**）：

1. **完整动力学缺失**（行 5248–5252）：通用关节空间动力学 $\boldsymbol\tau=\boldsymbol M(\boldsymbol\theta)\ddot{\boldsymbol\theta}+\boldsymbol h(\boldsymbol\theta,\dot{\boldsymbol\theta})$（式 10.1，$\boldsymbol M$ 对称正定质量阵，$\boldsymbol h$ 含向心力/科氏力/重力/摩擦）。本书未推导 → 支撑腿即便运动也只能用静力学近似、摆动腿无前馈力矩；**高速时误差大到无法接受**。求 $\boldsymbol M,\boldsymbol h$ 的两法：拉格朗日法（低自由度简便）、牛顿-欧拉法（Featherstone 高效算法 + RBDL，实用首选）。
2. **WBC 路线**（行 5254）：Khatib 提出 WBC；Kim 在其上提出 kinWBC + WBIC（全身脉冲控制），使 Mini-Cheetah 获良好高速性能。
3. **MPC 路线（§10.7.4，行 5258，核心对照句）**：
   > 「本书用 QP 计算各足端期望足底力来控制机身运动与平衡。**不足是控制器只能考虑当前状态，不能预测未来时刻运动的影响**。为解决此问题需用 **MPC**。MPC 通过对未来一段时间运动的预测，用二次规划等优化算法计算下一瞬间的目标位姿和足底力。」
4. **学习方法**（同段）：近年大量基于机器学习的控制，如 Lee 等用强化学习在 ANYmal 上取得很好效果。

---

## 7. 与于宪元（理论/推导源）的异同点

> 范本章主推导源为于宪元（推导向）；宇树为实践/对照源。下列对照供「§9 流水线」融合时取舍。

**相同（可互证）**：
- 单刚体动力学骨架一致：牛顿方程 $\boldsymbol f=m\dot{\boldsymbol v}$ + 欧拉方程含 $[\boldsymbol\omega]_\times\boldsymbol I\boldsymbol\omega$ 陀螺项。
- 惯性张量世界系变换 $\boldsymbol I_s=\boldsymbol R\boldsymbol I_b\boldsymbol R^{\mathrm T}$。
- 足底力→机身合力/合力矩的映射结构（力求和 + $[\boldsymbol p_{gi}]_\times$ 力矩）。
- 摩擦锥线性化为多面锥、足底力 QP 分配的代价+约束范式。

**不同（互补）**：
| 维度 | 宇树（本源） | 于宪元（对照） |
|---|---|---|
| 是否实现 MPC | **否**，仅瞬时单步 QP；MPC 只在 §10.7 提一句 | （范本章主体，含预测时域 SRBD-MPC 完整推导） |
| 非线性项处理 | 直接忽略 $[\omega_b]_\times I_b\omega_b$（小角速度假设） | 预期含变分/绕参考轨迹线性化（待对照确认） |
| 时间维 | 单时刻（瞬时平衡），无预测 horizon | 多步预测 + 滚动优化 |
| 逆动力学 | 单腿静力学近似 $\boldsymbol\tau=-\boldsymbol J^{\mathrm T}\boldsymbol R_{sb}^{\mathrm T}\boldsymbol f$（忽略腿惯性） | 视范本章而定 |
| 模型参数 | 强调**实验整定** $m$/重心/$\boldsymbol I_b$（式 8.40 缩放） | 偏理论给定 |
| 求解器 | QuadProg++（C++ 工程） | 偏理论/通用 QP |
| 状态来源 | KF 状态估计器（ch7）显式接入 $R_{sb},v_s,\omega_s$ | 多假设已知 |
| 写法风格 | 逐式逐步、配代码类（落地） | 推导严谨、定理化 |

**融合建议**：以于宪元的 SRBD-MPC 推导为理论主线；用宇树式 6.14–6.40 作 SRBD 从积分到矩阵形式的**全量补全**（密度积分起手，比多数论文详尽）；用宇树式 8.23–8.40 作 **QP/摩擦锥/逆动力学/参数整定的实践落地**；用 §10.7.4 的 QP↔MPC 对照句作范本章引言的动机锚点。

---

## 8. 可作对照 / 实践的素材清单

**作「对照」（理论锚点）**：
- 式 6.40 矩阵形（足底力↔机身运动）= MPC 预测模型每步内核；可直接对齐 MPC 状态方程。
- 式 8.35 QP 代价（动力学项 + 力幅值正则 $\alpha W$ + 平滑项 $\beta U$）= MPC stage cost 的 N=1 退化。
- 式 8.24/8.37/8.38 摩擦锥与触地/腾空约束 = MPC 不等式/等式约束。
- §10.7.4 「只能考虑当前、不能预测未来」= 引出 MPC 必要性的最佳对照句（行 5258）。

**作「实践」（落地素材）**：
- 简化逆动力学式 8.39（足底力→关节力矩，$\boldsymbol\tau=-\boldsymbol J^{\mathrm T}\boldsymbol R_{sb}^{\mathrm T}\boldsymbol f$）。
- 质量属性实验整定流程 + 缩放式 8.40（$m$/重心/$\boldsymbol I_b$ 怎么调）。
- PD 临界阻尼整定式 8.13、稳定上界式 8.16（500 Hz、dt=0.002 s 具体数值）。
- 惯性张量工程取值（非对角强制置 0、A1 实例 diag(0.1320,0.3475,0.3775)，式 6.32）。
- 代码类映射：`BalanceCtrl`（calF 解 QP，成员 _S/_W/_alpha/_U/_beta/_Fprev/_fricMat ↔ 式 8.35/8.24，§8.6.1）、`State_BalanceTest`（calcTau 五步流程，§8.6.2）、`QuadrupedRobot`（getQ/getQd/getTau/getJaco，§6.5.1）、`State_Trotting`（_gait/_est/_robModel/_balCtrl 四组件，§10.2）、求解器 QuadProg++、绘图 PyPlot/matplotlib-cpp。

---

## 9. 纯工程内容登记（属实践节素材，未深抽）

> 以下仅一句话登记，供后续「实践节」取用，不含理论推导。

- **第1章**（行 180）：四足发展现状、组成、硬件/控制器/示例代码准备——**实践节背景素材**。
- **第2章 关节电机**（行 247）：永磁同步电机 FOC 简介、关节电机结构/单圈绝对编码器/混合控制(位置-速度-力矩-阻尼-零力矩-混合 6 模式)/线路连接/ID 配置(ChangeID)/Python 范例/通信报文格式——**纯电机工程，实践节素材**。
- **第3章 仿真与控制框架**（行 645）：ROS+Gazebo、OOP 三思想(数据抽象/继承/动态绑定)、有限状态机(FSM)、关节控制/阻尼模式/坐标系零角度点/固定站立、条件编译/依赖/Gazebo 验证、真机网络配置(局域网/ping/ssh/scp/接互联网/切真机模式)——**纯框架与工程，实践节素材**（FSM 与固定站立=位置控制，可作平衡控制器的「反面动机」引子）。
- **第7章 状态估计器**（行 3382，除 §7.4 QP 已抽）：IMU 质量-弹簧模型(式 7.1–7.7，加速度计读数 $a_{out}=a-R_{bs}g$)、连续/离散状态空间、矩阵微积分、概率(期望/方差/协方差)、加权/递推最小二乘、离散卡尔曼滤波器(理论推导+计算流程+参数调试 Q/R/协方差比值/足端腾空触地)、Estimator/AvgCov 类——**KF 状态估计，为控制器供 $R_{sb},v_s,\omega_s$；本身偏估计理论，登记备用**（若范本章需「状态从何来」可引 §7.10）。
- **第9章 步态与轨迹规划**（行 4664）：常用步态(对角/踱步等)、落脚点规划(平移式 9.x/转动)、摆动腿摆线轨迹、WaveGenerator/FeetEndCal/GaitGenerator 类——**弱相关，提供支撑/摆动腿划分上下文，实践节素材**。
- **第10章 行走控制器**（行 4970，除 §10.7 对照已抽）：控制逻辑、State_Trotting 类、实时进程、运行流程/调试/真机编译——**纯工程组装，实践节素材**。
- **第11章 感知与导航**（行 5263）：距离传感器、ROS 导航包、控制程序接口、仿真/实机导航——**与运动控制无关，实践节素材**。
- **绘图子系统**：PyPlot 类 / matplotlib-cpp（§8.6.3，addPlot/addFrame/showPlot，调试折线图，阻塞需注意安全）——**调试工具，实践节素材**。
