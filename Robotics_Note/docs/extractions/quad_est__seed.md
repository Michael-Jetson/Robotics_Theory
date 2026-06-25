# 抽取留痕（写作种子）：四足机器人「足式状态估计」章（tag = est）

> **本文件性质**：项目内部「抽取留痕 / 写作种子」，**非成书正文**。三源中文页（个人笔记 + 于宪元硕论 §2.4 + 宇树书 §7）为内容权威，本种子**忠实吸收**其推导/记号/结论，统一到本书全局约定（右扰动主线、Hamilton 四元数、$\mathfrak{se}(3)$ 平移在前），换算在 `note` 盒标注。
>
> **抽取原则**：完全吸收·禁摘要·禁凝练。逐式录入、逐步推导、保留中间步骤；纯工程（ROS/Gazebo/代码类名/电机配置）凝练进 `practice`。
>
> **本章定位**：足式状态估计章 = 「腿式里程计 + IMU」融合主线。聚焦四足特有的**触地检测**与**腿式运动学约束**作观测；通用 KF/EKF/ESKF 推导**引用** `\cref{ch:eskf}`、`\cref{ch:invariant-filter}`、`\cref{ch:imu}`、`\cref{ch:leg}` 而不重复。本章是这些通用滤波理论在四足上的**落地实例**。
>
> **服务部**：四足运动控制部，与范本章「单刚体 MPC」(`\cref{ch:quad_mpc}`) 并列。范本量级目标 25–45 页。
>
> **作者洞见标记法**：凡作者自述理解/动机/"为什么"，正文内用 `〔作者洞见〕` 前缀就地标出，文末另设「作者洞见清单」编号汇总，成章时一一做 `insight` 盒。

---

## 源材料清单与出处

| 代号 | 来源 | 行号 | 承担内容 |
|---|---|---|---|
| **[笔记·触地]** | 个人笔记 `足式机器人控制.md` §足端触地检测 / §概率融合触地估计 | 1255–1316 | 触地检测动机（两类误判后果）、为何不用力传感器、广义动量法（GM）接触力观测器全推导。出处 Bledt ICRA2018（`\cite{bledt2018cheetah3}` 同组）、De Luca IROS2006。**含原创洞见 ≥3 条。** |
| **[笔记·状态估计]** | 个人笔记 §状态估计 | 1435–1438 | 主线串联：躯干旋转由 IMU 直读+坐标变换；躯干平动由 KF 足底里程计估计；与控制框架（MPC+WBC）的接口。**全章骨架来源。** |
| **[于宪元·2.4]** | 于宪元硕论 §2.4 状态估计器（2.4.1 状态方程 / 2.4.2 观测方程） | 789–956 | IMU 坐标系定义与原始数据→躯干姿态的旋转换算；KF 连续/离散状态方程（18 维）；观测方程（28 维，足底位置/速度/高度）；不打滑约束推导；摆动足协方差膨胀技巧。`\cite{yu2021quadruped}` |
| **[宇树·ch7]** | 宇树《四足机器人控制算法》§7 状态估计器 | 3382–3940 | IMU 质量-弹簧模型（读数=$a-Rg$）；连续/离散状态空间（18 维 $x$、28 维 $y$）；矩阵微积分；二次规划；概率基础（期望/方差/协方差/独立/白噪声）；加权最小二乘→递推最小二乘→离散 KF 全推导；协方差递推测量。`\cite{unitree2023quadruped}` |

> **外部出处（源自标）**：
> - **Bledt et al., ICRA 2018**, *Contact model fusion for event-based locomotion in unstructured terrains*（MIT Cheetah 3 概率融合触地，99.3% 准确率/毫秒级延时）→ 与已有 `\cite{bledt2018cheetah3}`（Cheetah3 控制系统）同组，成章主文献之一。
> - **De Luca et al., IROS 2006**, *Collision detection and safe reaction with the DLR-III lightweight manipulator arm*（广义动量碰撞观测器原始出处）。
> - **Bloesch et al.**, *State Estimation for Legged Robots*（`\cite{bloesch2013state}`，腿式 KF 里程计经典，成章互补对照）。
> - **Di Carlo et al., IROS 2018**（`\cite{dicarlo2018cheetah}`，凸 MPC，状态估计是其反馈来源）。

> **可用编译环境（已 Read `styles.tex`/`preamble.tex` 确认）**：`derivation`（推导盒）、`insight`（洞见，可 `\cref`）、`paper`（论文精读）、`pitfall`（陷阱）、`practice`（练习/实践）、`algo`（算法，可 `\cref`）、ElegantBook 自带 `definition`/`theorem`/`example`/`note`/`remark`；`finenote`（小字说明环境）；`\cnum{}` 全局带圈数字（**勿重定义**）；中文裸入 math mode 须 `\text{}`；表格 `\centering`。

---

## 一、本章拟定节结构（对应教学规范 v5.0：动机→反面→历史→理论→陷阱→练习）

```
\chapter{足式状态估计}\label{ch:quad_est}

[前置自测 practice] + [本章目标]

§est.1 动机：机器人"在哪儿"——为何四足必须做状态估计   \label{sec:est-why}
   - [笔记·状态估计 1435] 状态估计=反馈源头，无准确状态则无稳定运动
   - 关节角可由编码器直读；但世界系下机身位置/速度无法直接测量（无 GPS/动捕）
   - 解决路径：IMU(惯性) + 足端运动学(腿式里程计) 融合 → 引出本章主线
   - 与 \cref{ch:quad_mpc} 的接口：MPC/WBC 需要 (p_b, v_b, R, ω) 作反馈

§est.2 反面：朴素方案为何不够                          \label{sec:est-naive}
   - 纯 IMU 积分：加速度二次积分→漂移发散（位置漂移）
   - 纯运动学里程计：依赖"支撑足不打滑"假设，打滑/触地误判即崩
   - 触地误判的两类灾难后果（[笔记·触地 1265-1277]）→ 逼出触地检测
   - 为何不用足端力传感器（成本/贴合/噪声/线路）→ 逼出本体感知估计

§est.3 历史与谱系                                       \label{sec:est-history}
   - Bloesch 2013 腿式 KF 里程计；Bledt 2018 概率融合触地（Cheetah3）
   - 宇树 unitree_guide 的 18 维线性 KF（工程落地范式）
   - 不变卡尔曼滤波(InEKF, \cref{ch:invariant-filter}) 作为现代演进指针

§est.4 IMU 测量模型                                     \label{sec:est-imu}
   - [宇树 7.1] 质量-弹簧模型 → 读数 a_out = a - R g（含重力耦合）
   - [于宪元 2.4] IMU 坐标系定义(B_imu/B_imu0)与原始数据→躯干姿态换算
   - note 盒：与 \cref{ch:imu} 通用 IMU 噪声/bias 模型的接口；记号统一

§est.5 足端触地检测                                     \label{sec:est-contact}
   §est.5.1 力阈值法（最朴素）与其缺陷
   §est.5.2 广义动量接触力观测器（GM）[笔记·触地 1293-1315] 全推导
   §est.5.3 概率融合触地估计（Bledt 2018）——相位先验×测量似然
   - insight：为何绕过加速度（GM 的核心动机）

§est.6 腿式里程计的状态方程                              \label{sec:est-state}
   - [于宪元 2.44-2.46] / [宇树 7.10-7.11] 连续状态方程（足端静止假设）
   - 18 维状态 x = [p_b, v_b, p_foot(4)]；输入 u = R a + g
   - 离散化（前向欧拉 / ZOH）[于宪元 2.45][宇树 7.16-7.18]

§est.7 腿式里程计的观测方程                              \label{sec:est-obs}
   - 正向运动学得本体系足端位置 [于宪元 2.47]
   - 雅可比得本体系足端速度 [于宪元 2.49]；不打滑约束 [于宪元 2.50-2.52]
   - 28 维观测 y = [足端相对位置×4, 足端相对速度×4, 足端高度×4]
   - [宇树 7.13-7.14] 同构观测，C 矩阵显式
   - 摆动足协方差膨胀 [于宪元 2.58]（支撑/摆动差异化信任）

§est.8 卡尔曼滤波融合                                   \label{sec:est-kf}
   - 引用 \cref{ch:eskf} 通用 KF；本节只给四足实例的预测/更新两步
   - [于宪元 2.57] 标准递归式；[宇树 7.64-7.68] 先验/后验形式
   - 数值例：单步预测-更新走一遍
   - 指针：右扰动 ESKF / InEKF \cref{ch:invariant-filter} 处理 SO(3) 一致性

§est.9 陷阱速查 [pitfall]                               \label{sec:est-pitfall}

§est.10 实践与部署 [practice]                           \label{sec:est-practice}
   - Q/R 整定、摆动足 big_number、高度漂移与零高度假设的代价
   - unitree_guide 接口（getRotMat 等）凝练，不展开代码

[本章符号表] + [速查卡] + [练习题]
```

---

## 二、逐条关键公式收录（LaTeX + 来由 + 推导步骤 + 出处）

### A. IMU 测量模型（§est.4，源 [宇树 7.1] line 3392–3447）

**A1 一维质量-弹簧（牛二）** —— 来由：加速度计简化为质量块+弹簧，失重下加速度 $a$ 致弹簧压缩 $s$，弹力反向。出处 [宇树式7.1]。
$$ ma = -ks $$
**A2 读数定义**（[宇树式7.2]）：由形变反推读数 $ a_{\text{out}} = -\frac{k}{m}s $。
**A3 含重力一维**（[宇树式7.3]）：地球惯性系须计重力，$g=-9.81\,\text{m/s}^2$：
$$ ma = -ks + mg \quad\Rightarrow\quad s = \frac{m}{k}(g-a) $$
**A4 读数受重力耦合**（[宇树式7.4]）：$ a_{\text{out}} = -\tfrac{k}{m}s = a - g $。静止时 $a_{\text{out}}=-g=9.81$。
**A5 三维 IMU（局部系 $\{b\}$）**（[宇树式7.5–7.6]）：重力为世界系常量，须左乘 $R_{bs}$ 转入 $\{b\}$：
$$ m\boldsymbol a = -k\boldsymbol s + m R_{bs}\boldsymbol g,\qquad \boldsymbol s = \frac{m}{k}(R_{bs}\boldsymbol g - \boldsymbol a) $$
$$ \boldsymbol a_{\text{out}} = -\frac{k}{m}\boldsymbol s = \boldsymbol a - R_{bs}\boldsymbol g = \boldsymbol a - R_{sb}^{\mathrm T}\boldsymbol g,\qquad \boldsymbol g=[0,0,-9.81]^{\mathrm T} $$
**A6 世界系加速度**（[宇树式7.7]，关键，状态方程输入）：
$$ \boldsymbol a_s = R_{sb}\boldsymbol a = R_{sb}\boldsymbol a_{\text{out}} + R_{sb}R_{sb}^{\mathrm T}\boldsymbol g = R_{sb}\boldsymbol a_{\text{out}} + \boldsymbol g $$
> 推导步骤：将 A5 的 $\boldsymbol a = \boldsymbol a_{\text{out}} + R_{sb}^{\mathrm T}\boldsymbol g$ 代入 $\boldsymbol a_s=R_{sb}\boldsymbol a$，用 $R_{sb}R_{sb}^{\mathrm T}=I$ 得净世界加速度 = 读数旋转回世界系 + 重力补偿。**这是 §est.6 状态方程里 $\dot v_b = R a_b + g$ 的物理来源。**

> `note` 盒（记号统一）：宇树用 $R_{sb}$（世界←机体），于宪元用 $^{\mathcal O}R_{\mathcal B}$（同义）。本书全局右扰动主线下，姿态用 $R\in SO(3)$，世界系记 $\{w\}$/$\mathcal O$，机体系 $\{b\}$/$\mathcal B$。统一取 $R \equiv {}^{w}R_b = R_{sb}$。

### B. IMU 坐标系换算（§est.4，源 [于宪元 2.4] line 806–838）

**B1 IMU 本体系↔机体系常值旋转**（[于式2.38]）：$ {}^{imu}R_{\mathcal B} = R_x(\pi) $。
**B2 世界系↔IMU 零系**（[于式2.39]）：$ {}^{\mathcal O}R_{imu0} = R_z(\psi_{init})R_x(\pi) $（$\psi_{init}$=启动朝向不确定角）。
**B3 启动角标定**（[于式2.40]）：首周期读 ${}^{imu0}R_{imu}^{t=0}$ 反解 $ \psi_{init} = -R_z^{-1}({}^{imu0}R_{imu}^{t=0}) $。
**B4 躯干姿态合成**（[于式2.41]，组合旋转）：
$$ {}^{\mathcal O}R_{\mathcal B} = {}^{\mathcal O}R_{imu0}\,{}^{imu0}R_{imu}\,{}^{imu}R_{\mathcal B} $$
**B5 加速度/角速度仅需 $B_{imu}\to\mathcal B$**（[于式2.42]）：
$$ {}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}} = {}^{\mathcal B}R_{imu}\,\boldsymbol\omega_{original},\qquad {}^{\mathcal B}\boldsymbol a_{com} = {}^{\mathcal B}R_{imu}\,\boldsymbol a_{original} $$
> 来由：IMU 可装机身任意位置，原始数据非躯干状态，须经标定的常值旋转链对齐。**这是于宪元相对宇树的互补点：宇树假设 IMU 即机体系直接给 $R_{sb}$，于宪元显式处理 IMU 安装偏置标定。**

### C. 触地检测——广义动量接触力观测器 GM（§est.5.2，源 [笔记·触地] line 1291–1315）

**C0 动机（为何不用力传感器 / 不用加速度法）**：足端无力传感器（成本/贴合球状足底/薄片精度噪声）→ 仅靠本体感知（关节扭矩/位置/速度）估计接触力。朴素法比较"模型预期扭矩"与"实际扭矩"，但模型扭矩需要 $\ddot q$，数值微分放大噪声且引入延迟。
**C1 广义动量定义**（line 1293）：$ p = M(q)\dot q $。
**C2 拉格朗日动力学**（line 1297）：
$$ M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau + \tau_K $$
（$g$=重力扭矩，$\tau$=电机扭矩，$\tau_K$=未知外部碰撞扭矩）。
**C3 外力扭矩（含加速度，须消去）**（line 1301）：
$$ \tau_K = [M(q)\ddot q + C(q,\dot q)\dot q + g(q)] - \tau $$
**C4 动量求导（仍含 $\ddot q$）**（line 1305）：
$$ \dot p = \frac{d}{dt}(M(q)\dot q) = \dot M(q)\dot q + M(q)\ddot q $$
**C5 联立消加速度**（line 1309）：将 C2 解出的 $M\ddot q = \tau+\tau_K - C\dot q - g$ 代入 C4：
$$ \dot p = \dot M(q)\dot q + [\tau + \tau_K - C(q,\dot q)\dot q - g(q)] $$
**C6 用斜对称性质** $\dot M - 2C$ 斜对称 $\Rightarrow \dot M = C + C^{\mathrm T}$（line 1311），代入：
$$ \dot p = \tau + \tau_K + C^{\mathrm T}(q,\dot q)\dot q - g(q) $$
> **关键结果（line 1315）：右端不含加速度，仅依赖总扭矩、位置、速度。** 成章时补全观测器形式：$\hat\tau_K = K_I\!\left(p - \int(\tau + C^{\mathrm T}\dot q - g + \hat\tau_K)\,dt\right)$（一阶低通残差动量观测器，De Luca 形式），由 $\hat\tau_K$ 经足端雅可比转置反推接触力 $\hat f = (J^{\mathrm T})^{+}\hat\tau_K$，再阈值/概率判触地。

**C7 概率融合触地（Bledt 2018，§est.5.3）**：成章须补——以步态调度相位给**接触先验** $P(s)$，以 (a) GM 估计的法向力、(b) 足高、(c) 足端纵向速度 三路测量似然，贝叶斯融合得接触概率 $P(\text{contact}\mid z)$。源笔记给出动机（平地固定调度够用，非结构化地形必须检测器）与指标（99.3%/毫秒级）。

### D. 腿式里程计 KF 状态方程（§est.6）

**D1 本体量旋到世界系**（[于式2.43]）：
$$ {}^{\mathcal O}\boldsymbol a_{com} = {}^{\mathcal O}R_{\mathcal B}\cdot{}^{\mathcal B}\boldsymbol a_{com},\qquad {}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}} = {}^{\mathcal O}R_{\mathcal B}\cdot{}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}} $$
**D2 连续状态方程（足端静止假设）**（[于式2.44] / [宇树式7.10]）：
$$ {}^{\mathcal O}\dot{\boldsymbol p}_{com} = {}^{\mathcal O}\boldsymbol v_{com},\quad {}^{\mathcal O}\dot{\boldsymbol v}_{com} = {}^{\mathcal O}\boldsymbol a_{com} + {}^{\mathcal O}\boldsymbol g,\quad {}^{\mathcal O}_i\dot{\boldsymbol p}=\boldsymbol 0\ \ \forall i\in\{1,2,3,4\} $$
（${}^{\mathcal O}\boldsymbol g=[0,0,-9.8]^{\mathrm T}$；支撑足世界系速度为 0。）
**D3 离散状态方程（18 维，于宪元形式）**（[于式2.45–2.46]）：状态 $\boldsymbol x = [{}^{\mathcal O}\boldsymbol p_{com};\,{}^{\mathcal O}\boldsymbol v_{com};\,{}^{\mathcal O}\boldsymbol p_{foot}]$，
$$
\boldsymbol x_{k+1} =
\underbrace{\begin{bmatrix} \mathbb I_3 & \Delta t\,\mathbb I_3 & \boldsymbol 0_{3\times12}\\ \boldsymbol 0_{3\times3} & \mathbb I_3 & \boldsymbol 0_{3\times12}\\ \boldsymbol 0_{12\times3} & \boldsymbol 0_{12\times3} & \mathbb I_{12}\end{bmatrix}}_{A}\boldsymbol x_k
+ \underbrace{\begin{bmatrix}\boldsymbol 0_{3\times3}\\ \Delta t\,\mathbb I_3\\ \boldsymbol 0_{12\times3}\end{bmatrix}}_{B}\underbrace{{}^{\mathcal O}\boldsymbol g}_{u_k},\qquad \boldsymbol x_{k+1}=A\boldsymbol x_k + B\boldsymbol u_k
$$
**D4 宇树等价形（输入=机身加速度）**（[宇树式7.10–7.11]）：$\boldsymbol x=[p_b;v_b;p_0;p_1;p_2;p_3]$（18 维），
$$
\dot{\boldsymbol x} = \begin{bmatrix} \boldsymbol v_b\\ R_{sb}\boldsymbol a_b + \boldsymbol g\\ \boldsymbol 0_{3\times1}\,(\times4)\end{bmatrix}
=\underbrace{\begin{bmatrix}\boldsymbol 0_{3\times3} & I_3 & \boldsymbol 0_{3\times12}\\ \boldsymbol 0_{3\times18}\\ \boldsymbol 0_{12\times18}\end{bmatrix}}_{A_c}\boldsymbol x + \underbrace{\begin{bmatrix}\boldsymbol 0_{3\times3}\\ I_3\\ \boldsymbol 0_{12\times3}\end{bmatrix}}_{B_c}[R_{sb}\boldsymbol a_b + \boldsymbol g]
$$
**D5 离散化（前向欧拉/ZOH）**（[宇树式7.16–7.18]）：
$$ \boldsymbol x(k) = \boldsymbol x(k-1) + \mathrm dt\,\dot{\boldsymbol x}(k-1) = (I+\mathrm dt\,A_c)\boldsymbol x(k-1) + \mathrm dt\,B_c\boldsymbol u(k-1) $$
$$ A = I + \mathrm dt\,A_c,\qquad B = \mathrm dt\,B_c,\qquad C = C_c $$
> **三源融合点**：于宪元把输入设为重力 $g$（加速度已并入 $a_{com}$ 项的世界系表达），宇树把输入设为整个机身加速度 $R a_b + g$。本质同一 18 维线性系统，区别仅在 $u$ 的取法。成章统一取宇树形（输入=世界系净加速度），并在 `note` 标注两种等价写法。

### E. 腿式里程计 KF 观测方程（§est.7）

**E1 正向运动学：本体系足端位置**（[于式2.47]，串联腿闭式）：
$$
{}^{\mathcal B}_i\boldsymbol p = \begin{bmatrix}-l_2 s_2 - l_3 s_{23} + \delta h_x\\ \zeta l_1 c_1 + l_3 s_1 c_{23} + l_2 c_2 s_1 + \zeta h_y\\ \zeta l_1 s_1 - l_3 c_1 c_{23} - l_2 c_1 c_2\end{bmatrix}
$$
其中 $c_j=\cos({}_iq^j)$，$s_j=\sin({}_iq^j)$，$c_{jk}=\cos({}_iq^j+{}_iq^k)$，$s_{jk}=\sin({}_iq^j+{}_iq^k)$；符号变量（[于式2.48]）$\zeta=+1$ 左腿/$-1$ 右腿，$\delta=+1$ 前腿/$-1$ 后腿。
**E2 足端雅可比（本体系足端速度）**（[于式2.49]）：对 E1 求偏微分得 $3\times3$ 雅可比 $J_i$，$ {}^{\mathcal B}_i\dot{\boldsymbol p} = J_i\,\dot{\boldsymbol q}_i $（完整矩阵见源，成章全录）。
**E3 足端世界系位置**（[于式2.50]）：$ {}^{\mathcal O}_i\boldsymbol p = {}^{\mathcal O}\boldsymbol p_{com} + {}^{\mathcal O}R_{\mathcal B}\cdot{}^{\mathcal B}_i\boldsymbol p $。
**E4 足端世界系速度（微分 E3）**（[于式2.51]，用 $\dot R = R\,[\omega]_\times$）：
$$
{}^{\mathcal O}_i\dot{\boldsymbol p} = {}^{\mathcal O}\boldsymbol v_{com} + {}^{\mathcal O}R_{\mathcal B}\big([{}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}}]_\times\,{}^{\mathcal B}_i\boldsymbol p + {}^{\mathcal B}_i\dot{\boldsymbol p}\big)
$$
**E5 不打滑约束 → 质心速度观测**（[于式2.52]，令 ${}^{\mathcal O}_i\dot{\boldsymbol p}=\boldsymbol 0$）：
$$
{}^{\mathcal O}\boldsymbol v_{com} = -{}^{\mathcal O}R_{\mathcal B}\big([{}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}}]_\times\,{}^{\mathcal B}_i\boldsymbol p + {}^{\mathcal B}_i\dot{\boldsymbol p}\big)
$$
**E6 足高观测**（[于式2.53/2.56]）：$ {}^{\mathcal O}_ip^z = [0\ 0\ 1]\cdot{}^{\mathcal O}_i\boldsymbol p $，名义零高度假设 $ {}^{\mathcal O}_ip^z=0 $。
**E7 整合观测方程 $\boldsymbol z_k = H\boldsymbol x_k$**（[于式2.54–2.55]）：28 维观测 = 4×足端相对位置(12) + 4×足端相对速度(12) + 4×足高(4)。
**E8 宇树等价 28 维观测**（[宇树式7.12–7.14]）：
$$
\boldsymbol y = \big[\,R_{sb}p_{bfBi}\ (\times4);\ R_{sb}([\omega_b]_\times p_{bfBi} + J_i\dot\theta_i)\ (\times4);\ p_{szi}\ (\times4)\,\big]
= C\boldsymbol x
$$
其中 $C$ 把"足端相对机身位置"对应 $p_i - p_b$、"足端相对速度"对应 $-v_b$、"足高"对应 $p_i(2)$（z 轴，0-index）。
> **三源融合点**：于宪元与宇树观测方程**结构完全同构**（28 维，位置/速度/高度三段），互为独立佐证。差异：于宪元显式给出串联腿运动学闭式（E1/E2 可整章作运动学例），宇树用通用雅可比 $J_i$ 与角速度叉乘项 $[\omega_b]_\times$。成章主推于宪元闭式 + 宇树雅可比通式并列。

### F. 摆动足协方差膨胀（§est.7 收尾，源 [于式2.58] line 949–955）

支撑/摆动差异化信任：若第 $i$ 腿摆动，则在 $Q$（过程）与 $R$（测量）中把对应块设为大数 $n_{big} = 100$：
$$
Q_{block}(7+3(i-1),\,\cdot,\,3,3)=n_{big}\mathbb I_3,\quad R_{block}(1+3(i-1),\cdot)=R_{block}(13+3(i-1),\cdot)=n_{big}\mathbb I_3,\quad R_{block}(25+(i-1),\cdot)=n_{big}
$$
> 来由：不打滑假设只对支撑足成立；摆动足运动学信息须降权，靠膨胀协方差实现"软关闭"。**这是触地检测(§est.5)与里程计(§est.6-7)的耦合点：触地状态 → 决定哪条腿的观测可信。**

### G. 概率与最小二乘基础（§est.8 引子，源 [宇树 7.3/7.5/7.6] line 3535–3782）

**G1 二次型偏导**（[宇树式7.19–7.21]）：$ \frac{\partial x^{\mathrm T}Ax}{\partial x}=x^{\mathrm T}A^{\mathrm T}+x^{\mathrm T}A $；对称时 $=2x^{\mathrm T}A$；$\frac{\partial Ax}{\partial x}=A$，$\frac{\partial x^{\mathrm T}A}{\partial x}=A^{\mathrm T}$。
**G2 迹偏导**（[宇树式7.22–7.23]）：$ \frac{\partial\,\mathrm{Tr}(ABA^{\mathrm T})}{\partial A}=AB^{\mathrm T}+AB $；$B$ 对称 $\Rightarrow 2AB$。
**G3 标量 QP 解**（[宇树式7.25–7.27]）：$J=x^{\mathrm T}Ax+bx+c$，$A$ 对称正定，$\frac{\partial J}{\partial x}=2Ax+b=0\Rightarrow x=-\tfrac12A^{-1}b$。
**G4 期望/方差/协方差**（[宇树式7.28–7.34]）：$E(x+y)=E(x)+E(y)$；$\sigma_x^2=E[(x-\bar x)^2]$；$C_{xy}=E[(x-\bar x)(y-\bar y)^{\mathrm T}]$；独立 $\Rightarrow C_{xy}=0$；白噪声协方差对角 $C_v=E(vv^{\mathrm T})=\mathrm{diag}(\sigma_i^2)$。
**G5 加权最小二乘**（[宇树式7.39–7.41]）：$J=(y-C\hat x)^{\mathrm T}R^{-1}(y-C\hat x)$，求导得
$$ \hat x = (C^{\mathrm T}R^{-1}C)^{-1}C^{\mathrm T}R^{-1}y $$
**G6 递推最小二乘 / 增益**（[宇树式7.42–7.49]）：$\hat x_k=\hat x_{k-1}+K_k(y_k-C\hat x_{k-1})$，误差递推 $\omega_k=(I-K_kC)\omega_{k-1}-K_kv_k$，协方差递推
$$ P_k=(I-K_kC)P_{k-1}(I-K_kC)^{\mathrm T}+K_kRK_k^{\mathrm T} $$
对 $K_k$ 求迹偏导置零得最优增益
$$ K_k = P_{k-1}C^{\mathrm T}(R+CP_{k-1}C^{\mathrm T})^{-1} $$
**G7 时变状态协方差递推**（[宇树式7.53–7.57]）：含过程噪声 $w\sim(0,Q)$，$\bar x_k=A\bar x_{k-1}+Bu_{k-1}$，$ P_k=AP_{k-1}A^{\mathrm T}+Q $。

> **教学层处理**：G1–G7 是通用 KF 推导。**本章不全推**，凝练为 `derivation` 盒 + 主体 `\cref{ch:eskf}`。只保留四足落地必需的最终递归式（H）。宇树这套"加权最小二乘→递推最小二乘→离散 KF"链条作为 `\cref{ch:eskf}` 的补充阅读指针。

### H. 离散卡尔曼滤波四足实例（§est.8 主体）

**H1 系统-观测组合**（[宇树式7.58–7.59]）：$x_k=Ax_{k-1}+Bu_{k-1}+w_{k-1}$，$y_k=Cx_k+v_k$，$w\sim(0,Q)$，$v\sim(0,R)$。
**H2 先验/后验定义**（[宇树式7.60–7.61]）：$\hat x_k^-=E\{x_k\mid y_{1:k-1}\}$，$\hat x_k^+=E\{x_k\mid y_{1:k}\}$；$P_k^\mp$ 同理。
**H3 时间更新（预测）**（[宇树式7.62–7.63]）：
$$ \hat x_k^- = A\hat x_{k-1}^+ + Bu_{k-1},\qquad P_k^- = AP_{k-1}^+A^{\mathrm T} + Q $$
**H4 测量更新（校正）**（[宇树式7.64]）：
$$ K_k = P_k^-C^{\mathrm T}(R + CP_k^-C^{\mathrm T})^{-1} $$
$$ \hat x_k^+ = \hat x_k^- + K_k(y_k - C\hat x_k^-) $$
$$ P_k^+ = (I-K_kC)P_k^-(I-K_kC)^{\mathrm T} + K_kRK_k^{\mathrm T}\quad\text{(Joseph 形)} $$
**H5 于宪元紧凑等价形**（[于式2.57]，与 H3/H4 同一滤波器，记号差异）：
$$
\underline{\hat x}_k = A\hat x_{k-1}+Bu_{k-1},\ \ \underline P_k=AP_{k-1}A^{\mathrm T}+Q,\ \ K_k=\frac{\underline P_k H^{\mathrm T}}{H\underline P_k H^{\mathrm T}+R},\ \ \hat x_k=\underline{\hat x}_k+K_k(z_k-H\underline{\hat x}_k),\ \ P_k=(\mathbb I-K_kH)\underline P_k
$$
> `note` 盒（记号统一）：于宪元用 $H$/$z$，宇树用 $C$/$y$（观测矩阵/观测量）。本书统一 $H$（观测矩阵）、$\boldsymbol z$（观测量），与 `\cref{ch:eskf}` 一致。于宪元用简化协方差更新 $(I-KH)P^-$，宇树用 Joseph 形 $(I-KC)P^-(I-KC)^{\mathrm T}+KRK^{\mathrm T}$（数值更稳、保对称半正定）——成章推荐 Joseph 形，`pitfall` 标注简化形的数值风险。

---

## 三、个人笔记中的「作者思路/洞见」点（主线 + insight 盒）

> 笔记是主线骨架来源，承载"为什么这样做"。以下编号供成章一一做 `insight` 盒（可标"作者洞见"）。

- **〔洞见①〕状态估计 = 控制的根**（line 1435）："只有估计出一个准确的状态，机器人才可以获得准确的反馈信息从而进行更准确的控制，否则就无法进行稳定的运动。" → 全章动机句，置 §est.1 开篇。
- **〔洞见②〕旋转直读、平动滤波的二分主线**（line 1437）："对躯干旋转的估计由 IMU 实现，只需对 IMU 原始数据稍作旋转坐标变换；对躯干平动，利用卡尔曼滤波器设计了一个足底里程计。" → **本章组织主线**：姿态(易，直读)与位置/速度(难，须融合)分治。
- **〔洞见③〕触地检测是混合控制的开关**（line 1257, 1443）：支撑相=力/阻抗控制，摆动相=位置/速度控制，两者控制律完全不同；触地状态决定切换 → 触地误判 = 控制逻辑崩溃。串联 §est.2 反面与 §est.5。
- **〔洞见④〕两类触地误判的灾难后果**（line 1265–1277）：已触地未检出→巨大关节力矩损坏电机/被撑起失衡/能耗；空中误判触地→无法到达落足点/支撑腿数减少摔倒。→ `pitfall` 盒素材。
- **〔洞见⑤〕为何不用足端力传感器**（line 1279）：薄片传感器难贴合球状足底、精度不足带噪、高精度则成本剧增、额外线路。→ 逼出"本体感知估计"路线，且兼得动力学模型效率。这是 GM 法的根本动机。
- **〔洞见⑥〕GM 绕过加速度的核心动机**（line 1291）：模型预期扭矩需 $\ddot q$，而 $\ddot q$ 由速度数值微分得到→放大噪声+延迟→任何依赖加速度的力观测器信号嘈杂。GM 通过广义动量求导 + 斜对称性质消去 $\ddot q$。→ §est.5.2 最关键洞见。
- **〔洞见⑦〕概率融合触地的适用边界**（line 1283）：平地固定时间调度器即可（运动模式固定，时间到即触地）；非结构化地形必出现时间偏移（高地早触、凹地晚触），必须检测器。→ 解释"为什么平地够用、复杂地形必须概率融合"。
- **〔洞见⑧〕高度漂移的特殊危害与零高度假设**（[于式2.56] 935–941，笔记同源思路）：x/y 累积误差不影响控制，但 z 方向误差直接体现在质心高度与抬腿高度控制 → 故假设支撑足名义零高度反求质心高度，代价是无法感知所踩平台高度但不显著影响控制。→ `insight`+`pitfall` 双标。

---

## 四、OCR 可疑处清单（原印 vs 推断订正）

> MinerU 源含 OCR 错（下标/分数/矩阵/符号）。成章遇可疑处据上下文与物理意义订正，用 `note` 盒标"OCR 订正"。

1. **[于宪元 line 795]** 原印 `$_{i}q^{j}$` 与 `$i\dot q^j$` 腿/关节序号下标混乱，传感器名"IMU"被 OCR 吞成空白（"测量…的 □"）。**订正**：${}_iq^j$=第 $i$ 腿第 $j$ 关节角，${}_i\dot q^j$=对应转速；缺失器件 = IMU。
2. **[于宪元 line 818/821]** `$\psi_{imit}$`、`$\psi_{imi t}$` —— **订正**：应为 $\psi_{init}$（初始/启动朝向角，init 被 OCR 成 imit）。式2.40 $\psi_{init}=-R_z^{-1}(\cdot)$ 中 $R_z^{-1}$ 指"从 $R_z$ 旋转矩阵反解偏航角"的算子（非矩阵逆），成章须文字澄清。
3. **[于宪元 line 877]** 式2.47 第二、三分量首项印作 `$\zeta l_1 c_1$`/`$\zeta l_1 s_1$`，但第二分量末项印 `$\zeta h_y$` 而第一分量末项印 `$\delta h_x$`。**疑点**：$h_x,h_y$ 为髋偏置，符号变量 $\delta$（前后）配 $x$、$\zeta$（左右）配 $y$ 物理上自洽，**暂判无误**，但 $l_2c_2s_1$ 项与 E2 雅可比对照核验。
4. **[于宪元 line 908/914]** 式2.51/2.52 中 `${}^{\mathcal B}\boldsymbol\omega_{\mathcal{OBX}}$` 的下标 `OBX` —— **订正**：$X$ 实为叉乘/反对称记号，应读作 $[{}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}}]_\times$（角速度的反对称矩阵）。源式2.32 即 $\dot R = R[\omega]_\times$ 的依据。
5. **[于宪元 line 926]** 式2.54 的 $H$ 矩阵被 OCR 渲染成超宽畸形阵（大量空列），结构不可读。**订正**：据式2.50/2.52/2.53 重建为 28×18 块矩阵——位置段 $[\mathbb I_3,\,0,\,-\mathbb I_{12}\text{块选择}]$、速度段 $[0,\,\mathbb I_3$ 重复$]$、高度段 $[0_{1\times6},0,0,1,\dots]$。成章按宇树式7.14 的 $C$ 矩阵对照重绘。
6. **[于宪元 line 944]** 式2.57 出现 `$\underline{{\hat x}}_k$`、`$\underline{P}_k$` 下划线记号 = 先验量（$\hat x_k^-$/$P_k^-$）。**订正**：统一为上标减号 $\hat x_k^-$，与宇树/本书一致。
7. **[宇树 line 3474]** 式7.10 状态导数向量第二行印 `$R_{sb}a_b+g$` 正确；但式7.11 的 $A_c$ 矩阵中间被 OCR 压成 `$0_{3\times18}$`/`$0_{12\times18}$` 两整行——**订正**：应为 $\begin{bmatrix}0_{3\times3}&I_3&0_{3\times12}\\0_{3\times18}\\0_{12\times18}\end{bmatrix}$，即仅第一块行 $[0,I_3,0]$ 非零（$\dot p_b=v_b$），其余行全 0（$\dot v_b$ 进 $B$、足端 $\dot p=0$）。
8. **[宇树 line 3500]** 式7.14 的 $C$ 矩阵同样 OCR 成超宽畸形阵（数十空列、块错位）。**订正**：按式右侧已给的 $[p_i-p_b;\,-v_b;\,p_i(2)]$ 语义重建 28×18 块矩阵。
9. **[宇树 line 3503]** "$p_0(2)$ 代表 $p_0$ 向量的第三个元素" —— 非 OCR 错，是 0-index 约定（程序对齐），**成章 `note` 显式提醒** $(2)=$ z 轴，避免读者误为第二元素。
10. **[宇树 line 3745]** 式7.44 $J_k=E(\omega_{k1}^2+\dots)$ 中 $\omega$ 既表角速度又表估计误差 —— **订正/警示**：此处 $\omega_k$ = 状态估计误差（非角速度），与 IMU 角速度 $\omega_b$ 同符号冲突。成章须改估计误差为 $\boldsymbol e_k$ 或 $\tilde{\boldsymbol x}_k$，`note` 标注换符。
11. **[宇树 line 3413]** 式（7.3 上一式）`$ma=-ks+mg$` 无编号、与7.3 连排，OCR 漏号。**订正**：补为式7.3 前半。
12. **[宇树 line 3910/3924]** "(7.66)"/"(7.69)" 孤立编号悬空（对应公式被并入上一 `\left\{`），**订正**：编号错位，成章合并。

---

## 五、三源对同一主题的差异 / 互补点（供融合）

| 主题 | [笔记] | [于宪元 §2.4] | [宇树 §7] | 融合策略 |
|---|---|---|---|---|
| **姿态估计** | 主线：IMU 直读+坐标变换（洞见②） | 显式 IMU 安装偏置标定链 B1–B5（启动角 $\psi_{init}$） | IMU 质量-弹簧物理模型 A1–A6（读数=$a-Rg$）；getRotMat 直读 | 物理(宇树)→标定(于宪元)→主线归纳(笔记)，三段递进 |
| **触地检测** | **独家**：GM 接触力观测器全推 C1–C6 + 概率融合动机 | 无（仅靠协方差膨胀间接区分） | 无（假设四足全触地） | 触地检测**只在笔记**，是本章相对两书的增量；成章主推笔记 GM + Bledt2018 |
| **状态方程** | 框架性描述 | 18 维，输入=重力 $g$（[于2.44-46]） | 18 维，输入=机身加速度 $Ra_b+g$（[宇7.10-11]） | 同一系统两种输入取法；统一取宇树形 |
| **观测方程** | — | 串联腿运动学闭式 E1+雅可比 E2（具体腿型） | 通用雅可比 $J_i$+角速度叉乘 E8（与机型无关） | 闭式(于,作运动学例)+通式(宇树,主体公式)并列 |
| **不打滑约束** | 思路点 | 显式微分推导 E3–E5（$\dot R=R[\omega]_\times$） | 隐含在 $C$ 矩阵 $-v_b$ 项 | 主推于宪元微分推导 |
| **支撑/摆动差异** | 触地切换控制律（洞见③） | 协方差膨胀 $n_{big}=100$（[于2.58]） | 仅理论假设全触地 | 于宪元的协方差膨胀 = 触地检测的滤波器落地，耦合 §5↔§7 |
| **KF 推导** | — | 直接给递归式（[于2.57]，简化协方差更新） | **全推**：WLS→RLS→KF，Joseph 形（[宇7.39-7.68]） | 全推引用 `\cref{ch:eskf}`；本章只留 H1–H5；Joseph 形优先 |
| **高度漂移** | — | 零高度假设+代价分析（[于2.56]，洞见⑧） | 同（足高观测=0） | 共识，作 `insight`+`pitfall` |
| **概率基础** | — | — | 期望/方差/协方差/独立/白噪声 G4（[宇7.5]） | 凝练为 `derivation` 盒或引 `\cref{ch:eskf}` |

> **融合主线一句话**：以笔记的"旋转直读 + 平动滤波"二分（洞见②）为骨架；姿态段用宇树物理模型+于宪元标定；触地段用笔记 GM（两书皆无，本章增量）；里程计 KF 段用于宪元的运动学观测推导 + 宇树的概率/KF 全推（降权引用 `\cref{ch:eskf}`），以于宪元协方差膨胀把触地检测与滤波器缝合。通用 KF/ESKF/InEKF 一律 `\cref` 不重推。

---

## 六、关键公式计数与引用账

- 收录关键公式（编号 A1–A6, B1–B5, C1–C7, D1–D5, E1–E8, F, G1–G7, H1–H5）：**约 43 条**。
- 主 `\cite`：`yu2021quadruped`（§est.4/6/7 推导主源）、`unitree2023quadruped`（§est.4/8 物理与 KF 主源）、`bledt2018cheetah3`（§est.5 触地）、`bloesch2013state`（§est.3/8 对照）、`dicarlo2018cheetah`（控制接口）。
- 内部互引：`\cref{ch:eskf}`（通用 KF/ESKF）、`\cref{ch:invariant-filter}`（InEKF/SO(3) 一致性）、`\cref{ch:imu}`（IMU bias/噪声通用模型）、`\cref{ch:leg}`（腿式里程计通用）、`\cref{ch:quad_mpc}`（反馈消费端）。
