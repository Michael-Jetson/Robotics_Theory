# 抽取留痕：四足机器人运动控制 —— 单刚体模型 MPC（作者个人笔记，含原创思路）

> **本文件性质**：项目内部「抽取留痕」，**非成书正文**。本抽取的源是**作者本人的个人笔记**（`/home/gpf/Note/SimpleSLAM_Theory/足式机器人控制.md`），价值最高——其中混有作者自己的理解、动机解释与"为什么这样做"的洞见，这些是教科书不会直述的内容，须**逐条标记保留**供成章时做 `insight` 盒。
>
> **抽取原则**：**完全吸收·禁摘要·禁凝练**。每一道公式、每一步推导、每一段文字解释全量录入，公式一律改写为规范 LaTeX，保留作者的所有中间步骤与口语化解释（口语化解释在成书时再润色，此处先保真）。
>
> **服务章节**：新部「四足机器人运动控制」之范本章 = **单刚体模型 MPC**（§9 流水线）。本章须覆盖：(a) 单刚体/质心动力学建模；(b) 足端力约束（幅度 + 摩擦锥线性化）；(c) 角速度↔欧拉角微分映射；(d) 连续→离散状态空间；(e) 动力学预测方程；(f) 最优控制/MPC 理论基础（代价函数、预测/控制时域、滚动优化、QP 化简）；(g) 四足中的 MPC（凸 MPC 标准形）。
>
> **作者洞见标记法**：凡作者自述的理解/动机/"为什么"，正文内用 `〔作者洞见〕` 前缀就地标出，文末另设「作者洞见清单」汇总编号，供成章时一一对应做 `insight`/`remark` 盒。

---

## 源材料清单与出处

| 代号 | 来源 | 行号 | 在本抽取中承担 |
|---|---|---|---|
| **[个人笔记·动力学]** | 作者个人笔记 `足式机器人控制.md` §机器人动力学 | line 459–653 | 质心/单刚体模型、动力学三方程、足端力约束（幅度+摩擦锥线性化）、角速度↔欧拉角映射、连续/离散状态空间、动力学预测方程。**含核心原创洞见 2 条（line 461、474）。** |
| **[个人笔记·控制理论]** | 作者个人笔记 §控制理论基础 / 最优控制理论 / 模型预测控制理论 | line 654–1027 | 系统线性化、最优控制概念与代价函数、MPC 核心思想、预测/控制时域、逐步预测方程递推、扩充系统方程、代价函数（误差项+惩罚项）、求导得 QP 闭式解。**含多条对"为什么"的作者解释。** |
| **[个人笔记·四足MPC]** | 作者个人笔记 §四足中的MPC | line 1193–1218 | WBC 的局限（无预测性）、凸 MPC 标准形（出处 MIT Cheetah 3）、约束矩阵 $C_i$/选择矩阵 $D_i$ 含义。 |

> **交叉引用（作者自标的外部出处）**：
> - **于宪元硕士论文 §3.2.5**（line 527）：角速度↔ZYX 欧拉角近似映射的完整推导。作者自陈"经过一系列复杂推导（这里可以看于宪元论文的3.2.5节）"，即此映射式的推导细节作者引到于宪元。**全文末（line 2567、2571）作者再次致谢于宪元**："非常感谢于宪元的硕士论文，其中的插图和理论推导非常齐全，本人方便起见直接截图粘贴" → 文献：**于宪元，《基于稳定性的仿生四足机器人控制系统设计》[D]，北京航空航天大学，2021**。成章时角速度映射一节须 cite 此论文。
> - **视觉 SLAM 十四讲**（line 527）：李群求导 $\dot R=\omega_\times R$ 的依据，作者引到十四讲（本书已有李群章，可内部 `\cref`）。
> - **MIT Cheetah 3 凸 MPC**（line 1197）：四足 MPC 标准形的出处 —— *Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control*（Di Carlo, Wensing, Katz, Bledt, Kim, IROS 2018）。成章时此章主文献。

> **OCR/转写说明**：源笔记公式存在少量手写/转写瑕疵（如 line 527 `R,\omega_\times^b` 的逗号实为乘号；line 1209 代价范数 $\|\cdot\|_{Q_i}$ 实际应为加权二次型 $\|\cdot\|_{Q_i}^2$；line 999/1004 起 $\underline{u}$ 与 $u$、$\mathbf s$ 与 $\underline{\mathbf s}$ 混排）。本抽取已据上下文恢复为规范记号，凡恢复处在脚注式括注中说明，便于回溯。

---

# 第一部分：机器人动力学（单刚体/质心模型）

> 源：[个人笔记·动力学] line 459–653。本部分是单刚体 MPC 的物理建模基础。

## §1.1 质心模型（CoM 模型）的引入与简化动机

作者原文（line 461）：在 MPC 中，一般认为四足机器人的物理模型为**质心模型（CoM 模型）**，也就是机器人所有的质量集中在机器人的躯体中心位置，以此来简化计算。

〔作者洞见 ①〕**为何要用质心模型简化**（line 461，原创动机解释，非教科书直述）：
- "如果不进行简化，则每次 MPC 的计算都相当于对系统做多次仿真来计算最优控制量序列，会导致计算量暴增。"
- "同时为了接近这种近似模型，串联腿的四足机器人模型一般会将膝关节的驱动电机放置在髋关节处，搭配连杆来驱动膝关节，从而降低腿部的转动惯量。"
- "同时进行实时反馈和计算，不会损失过多的控制精度。"

> 即：简化（质心模型）是为求解速度；硬件设计（膝电机上移至髋部 + 连杆传动）是为让真实机器人**逼近**这个被简化的模型（降低腿部转动惯量 → 腿质量分布对单刚体假设的破坏更小）；实时反馈则补偿简化带来的误差。这是"建模简化—硬件配合—反馈补偿"三位一体的工程闭环思路。

## §1.2 动力学三方程

记号（line 465）：四足机器人质心位置 $r$，速度 $\dot r$，角动量 $L$；$\mathbf I$ 是机器人的惯性张量，描述物体的质量分布（**忽略关节变化导致的质量分布变化**）；$\omega$（原文记 $w$）是机器人的角速度矢量；触地点位置和触地力为 $p_i$ 和 $f_i$。据此构建动力学方程（line 467–470）：

$$
m \ddot{\mathbf r}=\sum_{i=1}^n f_i + m\mathbf g
\qquad
\dot L =\frac{d}{dt}(\mathbf I\omega)= \sum_{i=1}^n(p_i-r)\times f_i \approx \mathbf I\dot{\boldsymbol\omega}
\qquad
\dot{\mathbf{R}} = [\boldsymbol{\omega}]^\wedge \mathbf{R}
$$

三个方程的物理含义（line 472）：
1. 第一个方程就是**牛顿第二定律**（合力 = 各足端力之和 + 重力，等于质量×质心加速度）。
2. 第二个方程表示**角动量的变化率等于合外力矩**（各足端力相对质心的力矩之和）。
3. 第三个公式描述了机器人**旋转矩阵的变化率与其角速度之间的关系**。

## §1.3 ⭐ 忽略陀螺项（角动量叉乘项）的线性化简化

作者原文（line 474）：这里存在一个模型简化。根据理论力学的理论，当外部有一个合力矩 $\tau$ 的时候，按理说应该有动力学方程（**完整欧拉方程**）：

$$
\dot L =\tau =\frac{d}{dt}(I_G \omega)=I_G \dot\omega +\omega \times (I_G \omega)
$$

但是在论文中明显忽略了**角动量的叉乘项** $\omega \times (I_G \omega)$。

〔作者洞见 ②〕**为何 MPC 可以忽略陀螺项**（line 474，原创理解，本章最核心的洞见之一）：
- "因为完整的欧拉方程是一个**非线性耦合微分方程**，在 MPC 这类需要**滚动在线求解**的问题中，忽略角动量叉乘项（也称为**陀螺项**）是一个将模型**线性化和解耦**的好方法。"
- "可以有效**降低求解难度和提高求解速度**。"
- "并且在刚体**角速度很小**的情况下该项可忽略。"（$\omega\times(I_G\omega)$ 是 $\omega$ 的二次项，$\omega$ 小则该项为高阶小量）
- "同时通过**反馈控制**可以有效避免模型失效。"（即便单步预测有模型误差，滚动 + 反馈逐步校正）

> 即对应上式的近似 $\dot L \approx \mathbf I\dot{\boldsymbol\omega}$：丢掉 $\omega\times(I_G\omega)$ 后，角动量微分对 $\omega$ 是线性的，配合后面欧拉角小角度近似，整个动力学才能写成 $\dot x = A_c x + B_c u$ 的线性时变形式，使每步 MPC 退化为 QP。这是"为追求凸 QP 而主动牺牲陀螺耦合"的设计取舍。

惯性张量获取的工程提示（line 476）：注意，在三维建模软件中，设置好装配体各个零部件的密度，可直接导出其惯性张量矩阵。

## §1.4 足端作用力约束

足端作用力约束有两种（line 480）：第一种是**幅度约束**，第二种是**摩擦锥约束**。

### 1.4.1 幅度约束（两部分）

**第一部分——摆动腿零力约束**（line 482）：认为摆动腿的足端作用力为 0。

〔作者洞见 ③〕**摆动腿零力约束的物理依据**（line 482）："这种很好理解，因为腿在空中不会与地面接触，自然不会有作用力产生并影响机身运动状态；至于摆动腿本身运动导致的对机身的作用力也认为是可以忽略的，故可以施加一个 0 约束。"（即两层近似：无接触⇒无地面反力；且摆动腿摆动惯性力对机身的反作用也忽略不计。）

**第二部分——垂直方向幅度限制**（line 484）：因为腿部电机是有物理极限的，其对腿部施加的力矩有限制，对应地，足端产生的对地作用力也有限制，因此需要在垂直方向上进行限幅：

$$
f_i^z\leq f_{\max}
$$

作者补充（line 484）："当然，有的地方也会加一个最小值约束。"（即 $f_{\min}\le f_i^z \le f_{\max}$，最小值保证支撑腿不脱离地面。）

### 1.4.2 摩擦锥约束及其线性化

**非线性摩擦锥**（line 486–488）：为保证足底与地面不发生相对滑动，足底反力的水平分量不能大于其竖直分量与滑动摩擦系数 $\mu$ 的乘积，即满足摩擦锥条件：

$$
\mu \cdot f^z_i \geqslant \sqrt{(f^x_i)^2+(f^y_i)^2}
$$

〔作者洞见 ④〕**为何要把摩擦锥线性化为摩擦金字塔**（line 492）："但是这是一个非线性的约束，不利于计算，所以考虑转化为线性约束，比如说我在摩擦圆锥中选取一个**摩擦金字塔**，这样子就可以构成四个线性约束并且其可以**严格满足**摩擦锥约束。"

> 关键洞见点：选内接金字塔（而非外切）⇒ 四个线性面构成的可行域是圆锥的**子集**，因此线性化后的解**严格满足**原非线性摩擦锥（保守但安全），不会出现"线性约束满足但实际打滑"。这是凸 QP 化的代价（可行域略缩）与收益（线性约束）的权衡。

**线性化后的矩阵形式**（line 494–523）：

$$
\underbrace{
\begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}}_{\underline{c}_i}
\leq
\underbrace{\begin{bmatrix}
-1 & 0 & \mu \\
0 & -1 & \mu \\
1 & 0 & \mu \\
0 & 1 & \mu \\
0 & 0 & 1
\end{bmatrix}}_{C_i}
\begin{bmatrix} f_{i}^{x} \\ f_{i}^{y} \\ f_{i}^{z} \end{bmatrix}
\leq
\underbrace{\begin{bmatrix} +\infty \\ +\infty \\ +\infty \\ +\infty \\ f_{\max} \end{bmatrix}}_{\bar{c}_i}
$$

> 解读（抽取专员注，便于成章）：前四行成对给出 $\mu f_i^z \pm f_i^{x} \ge 0$、$\mu f_i^z \pm f_i^{y}\ge 0$，即 $|f_i^{x}|\le\mu f_i^z$、$|f_i^{y}|\le\mu f_i^z$（金字塔四面）；第五行 $0\le f_i^z\le f_{\max}$ 同时编码了垂直幅度上限（与最小值 0，对应支撑相非负法向力）。约束矩阵记 $C_i$，上下界记 $\underline c_i,\bar c_i$，这正是后面四足 MPC 标准形（§4）里 $\underline c_i\le C_i u_i\le\bar c_i$ 的来源。

## §1.5 角速度与欧拉角微分映射

作者原文（line 527）：根据李群求微分（"大概可能叫这个，具体可以参考视觉 SLAM 十四讲"）的内容，可以认为有 $\dot R=\omega_\times R$，其中 $\omega$ 是**世界系**下的角速度；如果是**机身坐标系**下的，那么公式为 $\dot{R} = R\,\omega_\times^b$（原文逗号为乘号笔误）。也就是旋转矩阵的导是可以求出的。

〔作者洞见 ⑤〕**推导外包给于宪元**（line 527，方法论性的自述）："经过一系列复杂推导（这里可以看于宪元论文的 3.2.5 节），最终得到角速度与 ZYX 欧拉角之间的**近似映射**关系。" → 即作者明确把欧拉角速率映射的完整推导**引用**到于宪元硕士论文 §3.2.5，自己只取结论。成章时该推导须补全或显式 cite 于宪元 2021。

最终近似映射（line 529–544）：

$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}
=
\begin{bmatrix}
c_{\psi} & s_{\psi} & 0 \\
-s_{\psi} & c_{\psi} & 0 \\
0 & 0 & 1
\end{bmatrix}
{}^{\mathcal{O}}\omega_{OB}
=
R_z^{T}(\psi)\cdot {}^{\mathcal{O}}\omega_{OB}
$$

其中 $c_\psi = \cos\psi$，$s_\psi = \sin\psi$（$\psi$ 为偏航角）（line 546；原文记 $c_w,s_w$，下标 $w$ 即 $\psi$）。

> 这是一个**近似**映射：精确的 ZYX 欧拉角速率—机体角速度映射矩阵依赖 $\phi,\theta,\psi$ 三个角且含 $\sec\theta$ 等项；此处近似为只依赖偏航 $\psi$ 的 $R_z^T(\psi)$，对应小俯仰/横滚角假设——与 §1.3 "角速度小"假设同源，共同服务于线性化。

## §1.6 连续时间状态空间模型

作者原文（line 550）：首先可以通过状态空间模型的方法求出来连续时间下的动力学模型，也就是把之前的**角速度微分映射、角动量近似微分、牛二定律下的加速度、力矩作用下的角动量变化率**等公式合并为一个矩阵方程的形式：

$$
\frac{d}{dt}
\begin{bmatrix}
\Theta \\ {}^{o}p_{com} \\ {}^{o}\omega_{OB} \\ {}^{o}\dot{p}_{com}
\end{bmatrix}
=
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & R_z^{T}(\psi) & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & \mathbb{I}_3 \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3}
\end{bmatrix}
\begin{bmatrix}
\Theta \\ {}^{o}p_{com} \\ {}^{o}\omega_{OB} \\ {}^{o}\dot{p}_{com}
\end{bmatrix}
+
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
{}^{p}I^{-1}r_{1\times} & {}^{p}I^{-1}r_{2\times} & {}^{p}I^{-1}r_{3\times} & {}^{p}I^{-1}r_{4\times} \\
\mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m
\end{bmatrix}
\begin{bmatrix}
f_1 \\ f_2 \\ f_3 \\ f_4
\end{bmatrix}
+
\begin{bmatrix}
0_{3\times1} \\ 0_{3\times1} \\ 0_{3\times1} \\ {}^{o}g
\end{bmatrix}
$$

> 状态量解读：$\Theta$ = 姿态（欧拉角）、${}^{o}p_{com}$ = 质心位置、${}^{o}\omega_{OB}$ = 机体角速度、${}^{o}\dot p_{com}$ = 质心速度（均在世界/惯性系 $o$ 下）。状态矩阵第一行块 $R_z^T(\psi)$ 即 §1.5 的欧拉角速率映射；第二行块 $\mathbb I_3$ 表示 $\dot p_{com}$ 积分得 $p_{com}$；输入矩阵第三行块 ${}^{p}I^{-1}r_{i\times}$ 来自 $\dot\omega=I^{-1}\sum (p_i-r)\times f_i$（$r_{i\times}$ 为 $(p_i-r)$ 的反对称矩阵，${}^pI^{-1}$ 为机体惯量逆）；第四行块 $\mathbb I_3/m$ 来自牛顿第二定律 $\ddot p=\frac1m\sum f_i$。

〔作者洞见 ⑥〕**为何把重力并入状态（增广技巧）**（line 593）："因为带有一个重力常量影响，无法直接作为**标准状态方程**，所以需要稍微改动，将重力项整合到状态中。"

> 即标准线性系统形如 $\dot x=Ax+Bu$ 无常数偏置项，而重力 $g$ 是常数项 $\dot x=Ax+Bu+\mathbf g_0$。作者用**状态增广**把重力分量 ${}^{o}g(3)$ 作为一个额外状态（其导数恒 0），从而把仿射项吸收进 $A$ 矩阵，恢复成无偏置的标准形——这是为了直接套用标准 MPC/QP 框架的实用技巧。

重力增广后的状态方程（line 595–631）：

$$
\frac{d}{dt}
\begin{bmatrix}
\Theta \\ {}^{o}p_{com} \\ {}^{o}\omega_{OB} \\ {}^{o}\dot{p}_{com} \\ {}^{o}g(3)
\end{bmatrix}
=
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & R_z^{T}(\psi) & 0_{3\times3} & 0 \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & \mathbb{I}_3 & \vdots \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0 \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 1 \\
0 & \cdots & 0 & \cdots & 0
\end{bmatrix}
\begin{bmatrix}
\Theta \\ {}^{o}p_{com} \\ {}^{o}\omega_{OB} \\ {}^{o}\dot{p}_{com} \\ {}^{o}g(3)
\end{bmatrix}
+
\begin{bmatrix}
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & 0_{3\times3} \\
{}^{p}I^{-1}r_{1\times} & {}^{p}I^{-1}r_{2\times} & {}^{p}I^{-1}r_{3\times} & {}^{p}I^{-1}r_{4\times} \\
\mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m & \mathbb{I}_3/m \\
0_{1\times3} & 0_{1\times3} & 0_{1\times3} & 0_{1\times3}
\end{bmatrix}
\begin{bmatrix}
f_1 \\ f_2 \\ f_3 \\ f_4
\end{bmatrix}
$$

> 增广列中 $\dot{p}_{com}$ 那一行的 "$1$" 即重力作用：$\ddot p$ 的第 3 分量（z）由重力状态 $g(3)$ 驱动；末行全 0 表示 $\dot g=0$（重力为常数）。注意此时偏置项已消失，方程回到 $\dot x = Ax + Bu$ 的纯标准形。

紧凑形式（line 633–635）：以一种紧凑的方式重写即得连续时间状态方程

$$
\dot{\boldsymbol x}(t)=A_c(\psi)\,\boldsymbol{x}(t)+B_c(r_1,r_2,r_3,r_4,\psi)\,\boldsymbol{u}(t)
$$

> 注意作者明确标出：$A_c$ 仅依赖偏航 $\psi$；$B_c$ 依赖四个落足点矢量 $r_1\!\sim\!r_4$ 与偏航 $\psi$。这是"线性时变（LTV）"——每个 MPC 时刻按当前 $\psi$ 和落足点重新装配 $A_c,B_c$。

## §1.7 离散化

作者原文（line 637）：因为计算机只能离散化地处理数据，因此需要将其转化为离散模型，这里使用 $\Delta T$ 来表示 MPC 的周期。

〔作者洞见 ⑦〕**单周期内微分不变的近似**（line 637）："在一个周期中，可以近似认为系统状态的微分未发生变化。" → 即零阶保持（ZOH）/前向欧拉式离散化的依据：$\Delta T$ 内 $\dot x$ 视为常量，故 $x_{k+1}\approx x_k+\dot x\,\Delta T=(I+A_c\Delta T)x_k+B_c\Delta T\,u_k$。

离散状态矩阵（line 641–648，作者只给出 $A_k$，$B_k$ 同理由 $B_c\Delta T$ 得到）：

$$
A_k =
\begin{bmatrix}
I_3 & 0_{3\times3} & R_z^{T}(d\psi^k)\,\Delta T & 0_{3\times3} & 0 \\
0_{3\times3} & I_3 & 0_{3\times3} & I_3\,\Delta T & \vdots \\
0_{3\times3} & 0_{3\times3} & I_3 & 0_{3\times3} & 0 \\
0_{3\times3} & 0_{3\times3} & 0_{3\times3} & I_3 & \Delta T \\
0 & \cdots & 0 & \cdots & 1
\end{bmatrix}
$$

> 对比 §1.6 连续 $A_c$：对角线全部由 $0$ 变为 $I_3$（$I+A_c\Delta T$ 的 $I$ 项），耦合块乘上 $\Delta T$（$R_z^T\Delta T$、$I_3\Delta T$、重力列的 $\Delta T$）。$d\psi^k$ 即第 $k$ 步的期望偏航角。

## §1.8 动力学预测方程

作者原文（line 652）：因为模型预测控制的核心就是**预测系统未来一段时间的状态**然后求解最优控制序列，所以预测系统未来状态也是非常重要的一步。在上一步中我们推导出了单刚体质心模型的离散化状态方程，其中 $A_k$ 和 $B_k$ 仅和**期望轨迹、落足点坐标以及当前系统状态**有关，是**已知量**，那么在后面每个时刻的控制量都确定的情况下，就可以根据某一时刻的状态推导出下一时刻的状态。

〔作者洞见 ⑧〕**$A_k,B_k$ 是已知量这一观察的意义**（line 652）：作者强调 $A_k,B_k$ 仅依赖期望轨迹/落足点/当前状态（都是 MPC 求解前已知的量），因此整个预测序列对**控制量 $u$ 是线性的**——这正是后面能把 MPC 写成关于 $u$ 的 QP（§3.10）的前提。把 LTV 的"时变"参数都归到"已知系数"，剩下的优化变量只有力序列。

---

# 第二部分：控制理论基础与最优控制

> 源：[个人笔记·控制理论] line 654–696。本部分为 MPC 提供线性化与最优控制（代价函数）的基础。作者以**倒立摆**为引子讲线性化。

## §2.1 系统线性化（雅可比法）

作者原文（line 656）：这个系统是个非线性系统，为了实现对非线性系统的控制，需要在**平衡点附近**对系统进行线性化，线性化后的系统就可以应用状态空间控制理论实现控制了。

〔作者洞见 ⑨〕**倒立摆双平衡点与控制目标**（line 656，以具体例子建立直觉）："对于倒立摆模型来说，系统有两个平衡点，分别是角度为 0 和 180 度时，而系统并不关心小车的水平位置如何，并且角度为 180 度时为**稳定平衡点**，角度为 0 度时为**不稳定点**，而倒立摆系统的目标就是在不稳定点处实现稳定控制，因此主要关注此点。"

线性化的本质（line 658）："系统线性化的本质是在平衡点附近的系统状态变量微分方程采用**一阶方程**的方式来表达，类似于将方程采用**泰勒公式**展开后，忽略高阶项，只保留一次项的方式，从而完成线性化，使非线性系统变得线性。系统状态越靠近平衡点处，系统的线性化精度越高。系统线性化常用的方法为**雅可比矩阵**。"

非线性系统模型（line 660–662）：假设有一个非线性控制系统

$$
\dot x=f(x,u)
$$

平衡点 $x=x^*$，则系统的线性化公式如下（line 664–666）：

$$
\dot x=J(x^*)\,x+J(u)\,u
$$

（$J(x^*)$ 为 $f$ 对 $x$ 的雅可比在平衡点取值，$J(u)$ 为 $f$ 对 $u$ 的雅可比。）即系统最终化为一个线性系统（line 668–670）：

$$
\dot X=AX+BU
$$

## §2.2 最优控制的概念与代价函数

作者原文（line 676）：首先就是**最优控制的概念**，其研究动机是在**约束条件下达到最优的系统表现**。

〔作者洞见 ⑩〕**最优控制动机的双重含义**（line 676）："比如说物理被控对象存在各种物理极限，电机有转速和力矩极限，方向盘有转动极限，而最优系统表现也可以定义为**误差均方的积分**和**控制输入均方的积分**，前者代表**追踪性能良好**，后者代表**输入小、能量损耗小**。" → 作者点出代价函数两项的物理意义对偶：误差项=性能，输入项=能耗/省力，最优即两者权衡。

SISO 系统代价函数（line 676–678）：引入代价函数（性能指标）概念

$$
J=\int_0^t \big(qe^2+ru^2\big)\,dt
$$

其中 $r$（与 $q$）是权重系数，目标是寻找最优的控制输入 $u$ 使得代价函数最小（line 680）。

基于状态空间的代价函数（line 682–687）：

$$
\frac{dX}{dt}=AX+Bu,\qquad
Y=Cx,\qquad
J=\int_0^t \big(E^{T}QE+u^{T}Ru\big)\,dt
$$

作者补充（line 689）："如果是离散域的控制模型，那么上式就会转化为累加形式。"

〔作者洞见 ⑪〕**为何引入半正定权重矩阵 $Q$**（line 691）："在实际意义中，每一个状态变量的重要性不同，比如说有时候更看重速度，有时候更看重力矩，所以就需要引入**半正定矩阵**来修改权重系数，也就是从原先的 $E^{T}E$ 变成了 $E^{T}QE$，其中的矩阵 $Q$ 一般是一个**对角矩阵**。" → 权重矩阵是把"哪个状态更重要"这一工程意图编码进优化目标的手段。

〔作者洞见 ⑫〕**硬约束 vs 软约束的区分**（line 693）："当然实际应用中还需要考虑一些物理约束，比如说车辆的速度和加速度有极限，机械臂中的电机力矩有极限，这些约束都是**不可突破的，都属于硬约束的范畴**；与之相对的就是**软约束**，通过**性能指标**来约束某个变量不能太大（太大的话代价会过高）。" → 硬约束=不等式约束（QP 的 $C u\le c$），软约束=进代价函数惩罚项。这一区分直接对应后面 §3.9 惩罚项（软）与 §4 约束矩阵（硬）。

> 注：源笔记 §动态规划（line 695）为空标题（作者未展开），成章时可由抽取专员据 Bellman 最优性原理补全（标 `\rebuilt`），或与控制导论章的 DP/LQR 内容打通 `\cref`。

---

# 第三部分：模型预测控制理论（MPC）

> 源：[个人笔记·控制理论] line 697–1027。这是本章最完整的一段原创推导：从逐步预测 → 扩充系统方程 → 代价函数（误差+惩罚）→ 求导 → QP 闭式解，全程作者手推。

## §3.1 MPC 核心思想

作者原文（line 699）：模型预测控制，即 Model Predictive Control，其核心思想是通过模型来**预测系统在未来某一段时间内的行为**，然后通过**在线计算**获取最优控制输入，以此在满足约束条件的情况下**最小化代价函数**实现最优。

〔作者洞见 ⑬〕**MPC 与 PID 的本质区别**（line 699）："这种方法与 PID 只考虑**当前误差**的控制方法有较大区别。" → 一句点出 MPC 的独特价值：有预测/前瞻性，而 PID 是纯反应式。这条与 §4 "WBC 无预测性"的论述呼应，构成全章的主旨线索。

系统模型表达（line 699–702）：因为一般用于数位控制，所以使用离散型状态空间表达。其中 $X$ 是状态量（可以使用最小的一组量来准确描述系统），$u$ 是控制量/控制输入，$Z$ 是系统的输出（不一定是可观测的输出），$A,B,C$ 三矩阵是系统矩阵，这里认为状态是已知的（或者可以使用观测器观测估计的）：

$$
X_{k+1}=AX_k+Bu_k,\qquad Z_k=CX_k
$$

## §3.2 预测时域与控制时域

作者原文（line 704）：MPC 的核心思想如下图所示，**预测时域记为 $f$，控制时域记为 $v$**，时域的概念就是范围的长度，在实际工程中时域也是可以调节的参数。其中蓝线就是系统的状态或者控制输出，在当前时刻 $k$ 以后的就是预测出的系统状态，而我们需要做的事情就是计算出一组控制输入来引导输出或者状态到我们所期望的轨迹。

〔作者洞见 ⑭〕**为何"预测时域 > 控制时域"且控制时域后控制量恒定**（line 706，重要的工程取舍解释）："MPC 中，一般默认**预测时域大于控制时域**，且**控制时域之后的控制输入恒定**（等于控制时域最后的控制量）。这是因为 MPC 的核心是**在线实时优化**，所以需要在**短时间内快速求解**，如果控制时域过大会导致**优化变量过多**从而无法快速求解，所以就假定在控制时域之后的控制输入是固定值并且等于控制时域最后的控制量。"

> 即：预测要看得远（$f$ 大才能为未来做准备），但优化变量（控制量）要少（$v$ 小才能快速求解）⇒ 用"控制时域后控制量冻结"来解耦这对矛盾。这是 MPC 实时性的关键工程妥协，也是后面 §3.6 推导"脱离控制时域后系数合并"的动机。

记法约定（line 708）：使用 $u_{k+i|i}$（应为 $u_{k+i|k}$）来描述在 $k$ 时刻计算出的 $k+i$ 时刻的控制输入，其他系统状态和观测值也采用类似记法。以**最小化期望轨迹和预测轨迹之间的误差**为优化目标。

## §3.3 逐步预测（递推找规律）

作者原文（line 712）：设计 MPC 的第一步就是构建系统的方程，也就是预测状态和输出的方程。首先在 $k$ 时刻，开始预测下一时刻的状态和输出（line 714–716）：

$$
\mathbf{x}_{k+1|k} = A\mathbf{x}_k + B\mathbf{u}_{k|k},\qquad
\mathbf{z}_{k+1|k} = C\mathbf{x}_{k+1|k} = CA\mathbf{x}_k + CB\mathbf{u}_{k|k}
$$

继续预测再下一时刻（line 720–724）：

$$
\mathbf{x}_{k+2|k} = A\mathbf{x}_{k+1|k} + B\mathbf{u}_{k+1|k} = A^2\mathbf{x}_k + AB\mathbf{u}_{k|k} + B\mathbf{u}_{k+1|k}
$$
$$
\mathbf{z}_{k+2|k} = C\mathbf{x}_{k+2|k} = CA^2\mathbf{x}_k + CAB\mathbf{u}_{k|k} + CB\mathbf{u}_{k+1|k}
$$

再进一步预测，发现规律（line 732–734）：

$$
\mathbf{x}_{k+3|k} = A\mathbf{x}_{k+2|k} + B\mathbf{u}_{k+2|k} = A^3\mathbf{x}_k + A^2B\mathbf{u}_{k|k} + AB\mathbf{u}_{k+1|k} + B\mathbf{u}_{k+2|k}
$$
$$
\mathbf{z}_{k+3|k} = C\mathbf{x}_{k+3|k} = CA^3\mathbf{x}_k + CA^2B\mathbf{u}_{k|k} + CAB\mathbf{u}_{k+1|k} + CB\mathbf{u}_{k+2|k}
$$

## §3.4 控制时域内的一般预测式

作者原文（line 736）：将其拓展到控制时域内任意时刻，得到（line 738–739）：

$$
\mathbf{x}_{k+v|k} = A^v \mathbf{x}_k + \sum_{j=0}^{v-1} A^{j} B \mathbf{u}_{k+j|k},\qquad
\mathbf{z}_{k+v|k} = C A^v \mathbf{x}_k + \sum_{j=0}^{v-1} C A^{j} B \mathbf{u}_{k+j|k}
$$

转化为矩阵形式（line 743–762）：

$$
\mathbf{x}_{k+v|k} = A^v\mathbf{x}_k +
\begin{bmatrix} A^{v-1}B & A^{v-2}B & \cdots & AB & B \end{bmatrix}
\begin{bmatrix}
\mathbf{u}_{k|k} \\ \mathbf{u}_{k+1|k} \\ \mathbf{u}_{k+2|k} \\ \vdots \\ \mathbf{u}_{k+v-2|k} \\ \mathbf{u}_{k+v-1|k}
\end{bmatrix}
$$
$$
\mathbf{z}_{k+v|k} = CA^v\mathbf{x}_k +
\begin{bmatrix} CA^{v-1}B & CA^{v-2}B & \cdots & CAB & CB \end{bmatrix}
\begin{bmatrix}
\mathbf{u}_{k|k} \\ \mathbf{u}_{k+1|k} \\ \mathbf{u}_{k+2|k} \\ \vdots \\ \mathbf{u}_{k+v-2|k} \\ \mathbf{u}_{k+v-1|k}
\end{bmatrix}
$$

## §3.5 脱离控制时域后的预测（控制量冻结）

作者原文（line 764）：但这只是控制时域内的方程，一般情况下预测时域要大于控制时域，因为系统要看得更远然后进行当下决策，所以在控制时域之后和预测时域之间的区域也需要考虑进去。在这个范围内，控制输入是恒定的（line 766）：

$$
u_{k+v-1|k} = u_{k+v|k} = u_{k+v+1|k} = \cdots = u_{k+f-1|k}
$$

〔作者洞见 ⑮〕**控制量冻结后只改系数不扩维**（line 768）："当我们脱离了控制时域之后，系统的方程如下，这里因为后续的控制量是固定值了，所以**无需在控制量向量上进行拓展了，只需要在前面的矩阵中修改系数即可**。" → 这是 §3.2 工程取舍的数学落地：冻结控制量 ⇒ 把多个相同 $u$ 的系数 $A^j B$ 相加合并成一个块，优化变量维度锁死在 $v$ 个，与预测时域 $f$ 无关。

控制时域后一步（line 770–778）：

$$
\mathbf{z}_{k+v+1|k} = CA^{v+1}\mathbf{x}_k +
\begin{bmatrix} CA^vB & CA^{v-1}B & \cdots & CA^2B & C(A+I)B \end{bmatrix}
\begin{bmatrix}
\mathbf{u}_{k|k} \\ \mathbf{u}_{k+1|k} \\ \mathbf{u}_{k+2|k} \\ \vdots \\ \mathbf{u}_{k+v-2|k} \\ \mathbf{u}_{k+v-1|k}
\end{bmatrix}
$$

> 注意末块由 $CAB$ 变为 $C(A+I)B$：因为 $u_{k+v|k}=u_{k+v-1|k}$ 被合并，其系数 $CAB$ 与原 $u_{k+v-1|k}$ 的系数 $CB$ 之外的项叠加，得 $C(A+I)B$（即两步对同一冻结控制量的累积响应）。

进一步归纳，预测时域内任意时刻（line 782–790）：

$$
\mathbf{z}_{k+f|k} = CA^f\mathbf{x}_k +
\begin{bmatrix} CA^{f-1}B & \cdots & CA^{f-v+1}B & C(A^{f-v} + A^{f-v-1} + \cdots + A + I)B \end{bmatrix}
\begin{bmatrix}
u_{k|k} \\ u_{k+1|k} \\ u_{k+2|k} \\ \vdots \\ u_{k+v-2|k} \\ u_{k+v-1|k}
\end{bmatrix}
$$

化简记号（line 794–803）：定义累加项

$$
\bar{A}_{f,v} = A^{f-v} + A^{f-v-1} + \cdots + A + I
$$
$$
\mathbf{z}_{k+f|k} = CA^f \mathbf{x}_k +
\begin{bmatrix} CA^{f-1}B & CA^{f-2}B & \cdots & CA^{f-v+1}B & C\bar{A}_{f,v}B \end{bmatrix}
\begin{bmatrix}
u_{k|k} \\ u_{k+1|k} \\ u_{k+2|k} \\ \vdots \\ u_{k+v-2|k} \\ u_{k+v-1|k}
\end{bmatrix}
$$

一般形式（line 805–807）：不局限于控制视域之后、预测视域之内，则

$$
\bar{A}_{i,v} = A^{i-v} + A^{i-v-1} + \cdots + A + I
$$

## §3.6 扩充系统方程（堆叠全预测时域）

作者原文（line 809）：上面的方程只能逐步预测不同时刻的系统输出，所以进一步整理，得到**扩充的系统方程**——给定 $k$ 时刻初始状态和控制时域内的控制量，就可以预测出预测时域内的所有系统输出（line 811–849）：

$$
\begin{bmatrix}
z_{k+1|k} \\ z_{k+2|k} \\ z_{k+3|k} \\ \vdots \\ z_{k+v|k} \\ z_{k+v+1|k} \\ \vdots \\ z_{k+f|k}
\end{bmatrix}
=
\begin{bmatrix}
CA \\ CA^2 \\ CA^3 \\ \vdots \\ CA^v \\ CA^{v+1} \\ \vdots \\ CA^f
\end{bmatrix} x_k
+
\begin{bmatrix}
CB & 0 & 0 & 0 & \cdots & 0 \\
CAB & CB & 0 & 0 & \cdots & 0 \\
CA^2B & CAB & CB & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \vdots & \ddots & \vdots \\
CA^{v-1}B & CA^{v-2}B & CA^{v-3}B & \cdots & CAB & CB \\
CA^vB & CA^{v-1}B & CA^{v-2}B & \cdots & CA^2B & C\bar{A}_{v+1,v}B \\
\vdots & \vdots & \vdots & \vdots & \ddots & \vdots \\
CA^{f-1}B & CA^{f-2}B & CA^{f-3}B & \cdots & CA^{f-v+1}B & C\bar{A}_{f,v}B
\end{bmatrix}
\begin{bmatrix}
u_{k|k} \\ u_{k+1|k} \\ u_{k+2|k} \\ \vdots \\ u_{k+v-2|k} \\ u_{k+v-1|k}
\end{bmatrix}
$$

简写（line 851–900）：

$$
\underline{\mathbf{z}} = O\,\mathbf{x} + M\,\underline{\mathbf{u}}
$$

其中各块定义为

$$
\underline{\mathbf{z}} =
\begin{bmatrix} \mathbf{z}_{k+1|k} \\ \mathbf{z}_{k+2|k} \\ \mathbf{z}_{k+3|k} \\ \vdots \\ \mathbf{z}_{k+v|k} \\ \mathbf{z}_{k+v+1|k} \\ \vdots \\ \mathbf{z}_{k+f|k} \end{bmatrix},
\quad
O =
\begin{bmatrix} CA \\ CA^2 \\ CA^3 \\ \vdots \\ CA^v \\ CA^{v+1} \\ \vdots \\ CA^f \end{bmatrix},
\quad
M =
\begin{bmatrix}
CB & 0 & \cdots & 0 \\
CAB & CB & \cdots & 0 \\
CA^2B & CAB & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
CA^{v-1}B & CA^{v-2}B & \cdots & CB \\
CA^vB & CA^{v-1}B & \cdots & C\bar{A}_{v+1,v}B \\
\vdots & \vdots & \ddots & \vdots \\
CA^{f-1}B & CA^{f-2}B & \cdots & C\bar{A}_{f,v}B
\end{bmatrix},
\quad
\underline{\mathbf{u}} =
\begin{bmatrix} \mathbf{u}_{k|k} \\ \mathbf{u}_{k+1|k} \\ \mathbf{u}_{k+2|k} \\ \vdots \\ \mathbf{u}_{k+v-2|k} \\ \mathbf{u}_{k+v-1|k} \end{bmatrix}
$$

> 这就是 MPC 的"凝聚（condensed）"预测模型：所有预测输出 $\underline{\mathbf z}$ 用当前状态 $\mathbf x$（经 $O$）与待优化的控制序列 $\underline{\mathbf u}$（经 $M$）线性表出。$O$ 即自由响应矩阵，$M$ 即强迫响应（脉冲响应堆叠）矩阵。

## §3.7 跟踪目标与朴素代价（无约束的问题）

作者原文（line 902）：实际上我们的目标是跟踪目标输出，将目标输出/期望输出记为（line 904）：

$$
z_{k+1}^d,\ z_{k+2}^d,\ z_{k+3}^d,\ \cdots,\ z_{k+f}^d
$$

将期望系统输出记为 $z^d=[z_{k+1}^d,z_{k+2}^d,z_{k+3}^d,\cdots,z_{k+f}^d]^T$（line 906），那么可定义一种 MPC 表述：确定一个控制输入向量 $u$，使代价函数达到最小值（line 908）：

$$
\min_{\underline{\mathbf{u}}} \left\| \underline{\mathbf{z}}^d - \underline{\mathbf{z}} \right\|_2
= \min_{\underline{\mathbf{u}}} (\underline{\mathbf{z}}^d - \underline{\mathbf{z}})^{T} (\underline{\mathbf{z}}^d - \underline{\mathbf{z}})
$$

〔作者洞见 ⑯〕**朴素代价为何不够——必须惩罚控制输入**（line 910，引出惩罚项的关键动机）："但是这种方法的问题在于**无法约束控制输入的幅度**，控制输入非常大以至于越过执行器的执行范围，导致实际应用不可行，或者可能引发**执行器饱和，进而对反馈控制回路造成灾难性后果**。因此，我们需要在代价函数中对控制输入增加**惩罚函数**。此外，还需在代价函数中引入误差函数，来使得系统尽可能收敛。"

## §3.8 控制输入惩罚项 $J_u$

作者原文（line 912）：惩罚项的定义如下，首先是最初输入的幅度，然后是不同输入之间的变化或者差异，同时带有权重矩阵来修改惩罚项的系数。

〔作者洞见 ⑰〕**惩罚项软约束幅度与变化率**（line 912）："这种惩罚会对输入的**幅度和变化率**进行**软约束**，使其尽可能幅度小和变化速度慢。" → 即 $J_u$ 含两类项：$u_{k|k}$ 的幅度项（防过大）、相邻 $\Delta u$ 的差分项（防突变/抖振）。对应 §2.2 软约束概念。

惩罚项展开（line 914–940）：

$$
\begin{aligned}
J_u &= u_{k|k}^{T} Q_0 u_{k|k} + \sum_{j=1}^{v-1} (u_{k+j|k} - u_{k+j-1|k})^{T} Q_j (u_{k+j|k} - u_{k+j-1|k})\\
&=
\begin{bmatrix} u_{k|k} \\ u_{k+1|k} - u_{k|k} \\ u_{k+2|k} - u_{k+1|k} \\ \vdots \\ u_{k+v-1|k} - u_{k+v-2|k} \end{bmatrix}^{T}
\begin{bmatrix}
Q_0 & 0 & 0 & \cdots & 0 \\
0 & Q_1 & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & Q_{v-1}
\end{bmatrix}
\begin{bmatrix} u_{k|k} \\ u_{k+1|k} - u_{k|k} \\ u_{k+2|k} - u_{k+1|k} \\ \vdots \\ u_{k+v-1|k} - u_{k+v-2|k} \end{bmatrix}
\end{aligned}
$$

控制差分部分的简化（line 942–966）：定义差分矩阵 $W_1$

$$
\underbrace{\begin{bmatrix}
I & 0 & 0 & 0 & \cdots & 0 \\
-I & I & 0 & 0 & \cdots & 0 \\
0 & -I & I & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & 0 & -I & I
\end{bmatrix}}_{W_1}
\begin{bmatrix} u_{k|k} \\ u_{k+1|k} \\ u_{k+2|k} \\ \vdots \\ u_{k+v-2|k} \\ u_{k+v-1|k} \end{bmatrix}
= W_1\,\underline u
=
\begin{bmatrix} u_{k|k} \\ u_{k+1|k} - u_{k|k} \\ u_{k+2|k} - u_{k+1|k} \\ \vdots \\ u_{k+v-1|k} - u_{k+v-2|k} \end{bmatrix}
$$

则惩罚项化简（line 968–970）：

$$
J_u=(W_1 \underline u)^{T} Q (W_1 \underline u)=\underline u^{T} W_1^{T} Q W_1 \underline u=\underline u^{T} W \underline u,
\qquad W := W_1^{T} Q W_1
$$

## §3.9 跟踪误差项 $J_z$

作者原文（line 972）：对于跟踪误差部分也有类似的函数来描述差异，这个误差项描述了期望轨迹（目标轨迹）与实际轨迹之间的差别，是总代价函数的一部分（line 974–985）：

$$
\begin{aligned}
J_z &= (z^d_{k+1}-z_{k+1|k})^{T} P_1 (z^d_{k+1}-z_{k+1|k})+\cdots+ (z^d_{k+f}-z_{k+f|k})^{T} P_f (z^d_{k+f}-z_{k+f|k})\\
&= (z^d-z)^{T}P(z^d-z)
\end{aligned}
$$

其中权重

$$
P=
\begin{bmatrix}
P_1 & 0 & 0 & \cdots & 0 \\
0 & P_2 & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & P_{f}
\end{bmatrix}
$$

代入预测方程 $\underline{\mathbf z}=O\mathbf x_k+M\underline{\mathbf u}$（line 987–995）：

$$
\begin{aligned}
J_{\underline{\mathbf{z}}} &= (\underline{\mathbf{z}}^{d} - \underline{\mathbf{z}})^{T}P(\underline{\mathbf{z}}^{d} - \underline{\mathbf{z}}) \\
&= (\underline{\mathbf{z}}^{d} - O\mathbf{x}_{k} - M\underline{\mathbf{u}})^{T}P(\underline{\mathbf{z}}^{d} - O\mathbf{x}_{k} - M\underline{\mathbf{u}}) \\
&= (\underline{\mathbf{s}} - M\underline{\mathbf u})^{T}P(\underline{\mathbf{s}} - M\underline{\mathbf{u}})
\end{aligned}
\qquad\text{其中 } \underline{\mathbf s}=\underline{\mathbf z}^d-O\mathbf x_k
$$

> 关键代换：令 $\underline{\mathbf s}=\underline{\mathbf z}^d-O\mathbf x_k$（期望输出减去自由响应），把跟踪误差变成只关于待优化 $\underline{\mathbf u}$ 的二次型，为下一步求导铺路。

## §3.10 完整代价函数与 QP 闭式解

作者原文（line 997）：将两部分惩罚项叠加，就是完整代价函数（line 999–1000）：

$$
\min_{\underline{\mathbf{u}}} J=\min_{\underline{\mathbf{u}}} \big(J_{\underline{\mathbf z}}+J_{\underline{\mathbf u}}\big)=
\min_{\underline{\mathbf{u}}} \Big( (\mathbf{s} - M\underline{\mathbf{u}})^{T} P (\mathbf{s} - M\underline{\mathbf{u}}) + \underline{\mathbf{u}}^{T} W \underline{\mathbf{u}} \Big)
$$

拆解（line 1002–1004）：

$$
J=\mathbf{s}^{T} P \mathbf{s} - \mathbf{s}^{T} P M \underline{\mathbf u} - \underline{\mathbf u}^{T} M^{T} P \mathbf{s} + \underline{\mathbf u}^{T} M^{T} P M \underline{\mathbf u} + \underline{\mathbf u}^{T} W \underline{\mathbf u}
$$

求偏导（line 1006，作者提示：MPC 的目标是计算最优控制输入，因此**控制向量是自变量**，其中 $P$ 为对角矩阵，转置前后不变）（line 1008–1016）：

$$
\frac{\partial\, \mathbf s^{T} P \mathbf s}{\partial \underline{\mathbf u}} = 0,\qquad
\frac{\partial\, \mathbf s^{T} P M \underline{\mathbf u}}{\partial \underline{\mathbf u}} = M^{T} P \mathbf s,\qquad
\frac{\partial\, \underline{\mathbf u}^{T} M^{T} P \mathbf s}{\partial \underline{\mathbf u}} = M^{T} P \mathbf s,
$$
$$
\frac{\partial\, \underline{\mathbf u}^{T} M^{T} P M \underline{\mathbf u}}{\partial \underline{\mathbf u}} = 2M^{T} P M \underline{\mathbf u},\qquad
\frac{\partial\, \underline{\mathbf u}^{T} W \underline{\mathbf u}}{\partial \underline{\mathbf u}} = 2W \underline{\mathbf u}
$$

代价函数的导数（line 1018–1020）：

$$
\frac{\partial J}{\partial \underline{\mathbf{u}}} = -2 M^{T} P \mathbf{s} + 2 (M^{T} P M + W) \underline{\mathbf{u}}
$$

〔作者洞见 ⑱〕**二次型 ⇒ 令导数为零即得唯一最优**（line 1022）："因为代价函数是**二次型**，所以可以很容易地找出最优的向量，也就是导数为零时的变量。" → 强调凸二次型的全局最优性：一阶必要条件即充分条件，无需迭代搜索。令导数为零（line 1022–1024）：

$$
(M^{T} P M + W) \underline{\mathbf{u}} = M^{T} P \mathbf{s}
\;\Longrightarrow\;
\hat{\underline{\mathbf u}} = (M^{T} P M + W)^{-1} M^{T} P \mathbf{s}
$$

〔作者洞见 ⑲〕**MPC 本质 = 把控制问题转化为 QP**（line 1026，全章的方法论总结）："上述的过程，实际上就是根据**系统状态方程和矩阵方法，将控制问题转化为一个 QP 问题**然后使用各种 QP 求解器进行求解。" → 这句把前面所有矩阵推导收束到一个判断：MPC（带约束时）= QP；无约束时有上面的闭式解，有约束（幅度/摩擦锥）时则交给 QP 求解器。承上启下到 §4 的四足 QP 标准形。

---

# 第四部分：四足中的 MPC

> 源：[个人笔记·四足MPC] line 1193–1218。把前三部分（单刚体动力学 + MPC 理论）落地到四足，给出凸 MPC 标准形（出处 MIT Cheetah 3）。

## §4.1 为何需要 MPC——WBC 的局限

〔作者洞见 ⑳〕**WBC 无预测性，故需 MPC**（line 1195，本章立论的核心，与 §3.1 MPC vs PID 呼应）："虽然 WBC 使用了浮动基逆动力学，考虑了完整的机器人动力系统模型，但它**只找到瞬时的关节力矩和力**。这意味着 WBC **无法理解和考虑系统未来的状态**，也就是无法提前为未来的系统控制做一些提前的准备。如在机器人的**飞行相**中，单纯的 WBC 并不理想：在**支撑相的后半段**，单纯地使用 WBC 和其他无预测特性的控制算法，控制系统**不会为没有地面反作用力的飞行相做准备**；而在**飞行相后半程**，控制系统也**不会为后续的地面冲击做准备**。"

> 这是全章最重要的"为什么用 MPC"论证：动态步态（含飞行相）必须有前瞻性才能在相位切换处（起跳前、落地前）预先调整力分配，纯反应式控制（WBC/PID）做不到。MPC 的预测时域正好覆盖未来若干相位。

出处（line 1197）：作者引用 *Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control*（即四足凸 MPC 的奠基性工作）。

## §4.2 四足凸 MPC 标准形

作者原文（line 1201）：相关的动力学理论依然是同一套框架（即第一部分的单刚体质心模型），因此 MPC 优化问题为（line 1203–1215）：

$$
\min_{x, u} \sum_{i=0}^{k-1} \big\| x_{i+1} - x_{i+1, \text{ref}} \big\|_{Q_i}^{2} + \big\| u_i \big\|_{R_i}^{2}
$$
$$
\begin{aligned}
\text{subject to:}\quad
& x_{i+1} = A_i x_i + B_i u_i, & i &= 0,\ldots, k - 1\\
& \underline{c}_i \leq C_i u_i \leq \overline{c}_i, & i &= 0,\ldots, k - 1\\
& D_i u_i = 0, & i &= 0,\ldots, k - 1
\end{aligned}
$$

（原文范数记 $\|\cdot\|_{Q_i}$，按加权二次型惯例应为 $\|\cdot\|_{Q_i}^2 = (\cdot)^T Q_i(\cdot)$，已补正。）

各量含义（line 1217）：
- $Q_i / R_i$ 为**对角正权重的半正定矩阵**（$Q_i$ 惩罚状态跟踪误差，$R_i$ 惩罚控制量）。
- **等式约束** $x_{i+1}=A_ix_i+B_iu_i$：即第一部分推出的离散单刚体动力学（§1.7）。
- **不等式约束** $\underline c_i\le C_iu_i\le\bar c_i$：$C_i$ 表示**约束矩阵**，这个矩阵**包括了作用力范围限制和摩擦锥约束**，用于限制机器人足端作用力不至于超限导致失控（即 §1.4 的 $C_i$、$\underline c_i$、$\bar c_i$）。
- **选择约束** $D_iu_i=0$：$D_i$ 表示**选择矩阵**。

〔作者洞见 ㉑〕**选择矩阵 $D_i$ 与相位变量的联系**（line 1217）："$D_i$ 表示选择矩阵，这个矩阵会**选择哪些足端作用力为 0**（这个与之前提到的**足端相位变量**有关，因为**摆动腿不受力**所以需要作用力约束为 0，只进行位置/速度控制）。" → 把 §1.4.1 的"摆动腿零力约束"用步态相位（接触序列）动态地编码进 MPC：每个预测步按该步的接触状态装配 $D_i$，强制摆动腿对应的 $u$ 分量为 0。这是步态规划（相位）与 MPC（力优化）的接口。

〔作者洞见 ㉒〕**MPC 输出虚拟力、再经逆运动学转关节力矩**（line 1217，端到端的控制链条）："$u_i$ 是控制量或者说力，因为四足或者其他的机器人是一个物理系统，**施加力才可以改变其运动状态**。MPC 的作用就是考虑四足未来一段时间的运动情况计算出一系列合理的足端作用力来实现运动控制，然后**逆运动学等方法会将计算出的虚拟力转换为腿部电机的真实力矩**实现驱动。" → 明确 MPC 在控制栈中的位置：MPC 算"足端力"（任务空间/笛卡尔），下游用雅可比/逆运动学映射到"关节力矩"（执行器空间），与 VMC（§源笔记 line 1032，本抽取未含）的"虚拟力→雅可比→关节力矩"思路一致。

---

# 作者洞见清单（供成章做 `insight`/`remark` 盒）

> 共 **22 条**作者原创理解/动机/"为什么"。每条注明源行号、所在本抽取小节、建议盒类型。强烈建议成章时**逐条**转为 `insight`（洞见）或 `remark`（注记）环境，这是本章相对教科书的最大增量价值。

| 编号 | 源行 | 所在节 | 一句话要点 | 建议盒 |
|---|---|---|---|---|
| ① | 461 | §1.1 | 质心模型简化动机：不简化则每次 MPC≈多次仿真致计算量暴增；硬件上膝电机移髋部+连杆传动降腿部转动惯量以逼近单刚体假设；反馈补偿误差 | insight（核心）|
| ② | 474 | §1.3 | ⭐忽略陀螺项 $\omega\times(I\omega)$ 的理由：完整欧拉方程是非线性耦合微分方程，滚动在线求解中丢陀螺项=线性化解耦的好方法；角速度小时该项可忽略；反馈控制避免模型失效 | insight（核心）|
| ③ | 482 | §1.4.1 | 摆动腿零力约束的双层近似：空中无接触⇒无地面反力；摆动惯性力对机身的反作用也忽略 | remark |
| ④ | 492 | §1.4.2 | 摩擦锥线性化为内接金字塔，4 个线性约束且**严格满足**原非线性锥（保守安全） | insight |
| ⑤ | 527 | §1.5 | 欧拉角速率映射的完整推导外包给于宪元论文 §3.2.5，自己只取近似结论 | remark（注引用）|
| ⑥ | 593 | §1.6 | 重力是常数偏置破坏标准状态方程形式，用状态增广把 $g$ 作额外状态（$\dot g=0$）吸收仿射项 | insight |
| ⑦ | 637 | §1.7 | 离散化依据：一个 MPC 周期 $\Delta T$ 内近似认为状态微分不变（ZOH/前向欧拉） | remark |
| ⑧ | 652 | §1.8 | $A_k,B_k$ 仅依赖期望轨迹/落足点/当前状态=已知量 ⇒ 预测对 $u$ 线性 ⇒ 可化 QP | insight |
| ⑨ | 656 | §2.1 | 倒立摆双平衡点（0°不稳定、180°稳定），控制目标=不稳定点处镇定 | remark（例）|
| ⑩ | 676 | §2.2 | 代价函数两项物理对偶：误差均方积分=追踪性能；输入均方积分=省力/低能耗 | insight |
| ⑪ | 691 | §2.2 | 引入半正定（对角）权重 $Q$ 的动机：各状态重要性不同，编码"更看重谁" | remark |
| ⑫ | 693 | §2.2 | 硬约束（不可突破，物理极限⇒不等式约束）vs 软约束（进代价惩罚） | insight |
| ⑬ | 699 | §3.1 | MPC vs PID 本质区别：MPC 有预测/前瞻，PID 只看当前误差 | insight（主旨线索）|
| ⑭ | 706 | §3.2 | 预测时域>控制时域+控制时域后控制量冻结的理由：看得远 vs 优化变量少，解耦实时性矛盾 | insight（核心）|
| ⑮ | 768 | §3.5 | 控制量冻结后只需合并系数（$\to C(A+I)B$ 等），优化变量维度锁死=$v$，与 $f$ 无关 | insight |
| ⑯ | 910 | §3.7 | 朴素跟踪代价的缺陷：无法约束控制幅度⇒执行器饱和⇒反馈回路灾难 ⇒ 必须加输入惩罚 | insight（核心）|
| ⑰ | 912 | §3.8 | 惩罚项软约束输入的"幅度"与"变化率"（差分项防抖振） | remark |
| ⑱ | 1022 | §3.10 | 代价是二次型⇒导数为零即唯一全局最优（一阶条件=充分条件） | remark |
| ⑲ | 1026 | §3.10 | 方法论总结：MPC 本质=用状态方程+矩阵法把控制问题化成 QP，交 QP 求解器 | insight（核心）|
| ⑳ | 1195 | §4.1 | ⭐为何要 MPC：WBC 只解瞬时力、无预测性，飞行相/相位切换（起跳前、落地前）无法预先准备 | insight（核心立论）|
| ㉑ | 1217 | §4.2 | 选择矩阵 $D_i$ 与步态相位变量挂钩：按接触序列动态强制摆动腿足端力=0 | insight |
| ㉒ | 1217 | §4.2 | 控制栈定位：MPC 算足端虚拟力，下游逆运动学/雅可比转关节力矩；与 VMC 思路一致 | insight |

> **盒化优先级建议**：核心 insight（①②⑬⑭⑯⑲⑳）务必单独成盒且醒目；②（忽略陀螺项）与⑳（WBC 无预测）是全章两大"为什么"，可考虑各配一个推导/示意图。其余可酌情合并为节末 remark。

---

# 成章衔接备注（供 §9 综合 agent）

1. **章定位**：本章是新部「四足机器人运动控制」的**范本章**，单刚体模型 MPC。上游可承接控制导论章（LQR/MPC 一般理论，见 `docs/_archive/extractions/control_intro__lqr_mpc.md`，可 `\cref` 复用预测/控制时域、QP 化简的通用部分，避免重复；本章侧重"四足专属"的单刚体建模与足端力约束）。
2. **必引文献**：(a) 于宪元，《基于稳定性的仿生四足机器人控制系统设计》[D]，北京航空航天大学，2021（角速度↔欧拉角映射 §3.2.5；作者全文多处截图致谢）；(b) Di Carlo et al., *Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control*, IROS 2018（四足凸 MPC 标准形 §4.2）。
3. **与本书既有约定衔接**：旋转用 $R\in SO(3)$、$\dot R=[\omega]^\wedge R$（世界系）/ $\dot R=R[\omega^b]^\wedge$（机体系），可 `\cref` 李群章；右扰动约定本章主体（$\mathbb R^n$ 线性 MPC）不涉及，仅 §1.2、§1.5 的旋转微分处需与李群章记号统一。
4. **记号需统一项**：源笔记中 $f$=预测时域、$v$=控制时域；而控制导论抽取用 $N$=时域。成章须二选一并全书统一（注意 $f$ 在四足里也可能与"力"撞名 —— 建议预测时域改 $N_p$、控制时域改 $N_c$，力保留 $f_i$）。
5. **可补图**：①摩擦锥/金字塔内接示意（§1.4.2）；②MPC 预测/控制时域滚动示意（§3.2，源 line 710 有原图链接，已失效需重绘）；③步态相位与 $D_i$ 选择矩阵示意（§4.2）；④WBC vs MPC 在飞行相的对比（§4.1）。源笔记的图均为飞书内网链接（已失效），须用 figs 子系统（TikZ/Asymptote）重绘。
6. **`\rebuilt` 提示**：§2.2 末"动态规划"(line 695)在源中为空标题，若本章需要可由综合 agent 据 Bellman 原理补全并标 `\rebuilt`，或直接 `\cref` 到 RL/控制导论章。
