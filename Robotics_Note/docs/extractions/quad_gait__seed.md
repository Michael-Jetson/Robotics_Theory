# 抽取留痕：四足运动控制 —— 步态生成与轨迹规划（tag=gait）

> **本文件性质**：项目内部「抽取留痕」，**非成书正文**。服务新部「四足机器人运动控制」之 **步态生成与轨迹规划章（`\label{ch:quad_gait}`）**，为 `\cref{ch:quad_mpc}`（单刚体凸 MPC）**供给接触序列/相位时序与期望质心轨迹**。
>
> **抽取原则**：**完全吸收·禁摘要·禁凝练**。三源中文页 = 内容权威。每一道公式改写为规范 LaTeX，逐步推导写全，逐重要式标源（`\cite{yu2021quadruped}` / `\cite{unitree2023quadruped}` / 个人笔记）。目标成章 25–45 页。
>
> **记号统一（本书全局约定）**：右扰动主线、Hamilton 四元数、$\mathfrak{se}(3)$ 平移在前。三源记号冲突在 `note` 盒标换算。本章主要涉及欧拉角 ZYX（$\phi$ 横滚、$\theta$ 俯仰、$\psi$ 偏航）与世界系 $\mathcal{O}$ / 本体系 $\mathcal{B}$ 上下标，沿用于宪元记号 ${}^{\mathcal{O}}(\cdot)$、${}^{\mathcal{B}}(\cdot)$，左下标 $d$ 表期望值、左下标 $i$ 表第 $i$ 号腿。
>
> **作者洞见标记法**：凡作者个人笔记自述的理解/动机/"为什么"，正文内用 `〔作者洞见〕` 前缀就地标出，文末另设「作者洞见清单」汇总编号，供成章一一对应做 `insight` 盒（可标"作者洞见"）。

---

## 源材料清单与出处

| 代号 | 来源 | 行号 | 承担内容 |
|---|---|---|---|
| **[笔记·步态]** | 个人笔记 `足式机器人控制.md` §步态解析 / 落足点规划 / 落足点主动搜索 / 地形坡度估计 | line 1317–1434 | 步态定义与直觉（踏步类比）、占空比/偏移量、相位钟表类比、G(trot) 描述法、Raibert 启发式（中性点+速度补偿+旋转补偿）三项物理意义、捕获点、落足点修正流程、Raibert 缺陷（纯旋转切向外推）。**全章主线骨架与多条原创洞见。** |
| **[于·调度]** | 于宪元硕论 §2.3 步态调度器 | line 704–788 | 时间切换 vs 事件切换、足底传感器五大工程难题、占空比/偏移量定义、G(trot) (2.33)、常用步态表、相位进度 $\tilde t_{ng}$、相位变量 ${}_i\tilde t_\Phi$ (2.34)、支撑布尔 ${}_iS_\Phi$ (2.35)、支撑/摆动相位进度 (2.36)(2.37)。 |
| **[于·质心]** | 于宪元硕论 §2.5 质心轨迹生成器 | line 957–1106 | 12 自由度状态、速度命令→偏航递推 (2.59)、本体/世界系期望速度 (2.60)(2.61)、堵转保护质心位置 (2.62)(2.63)(2.64)、地形坡度平面方程 (2.66)、支撑足坐标递推 (2.67)、最小二乘平面拟合 (2.68)–(2.71)、法向量→期望俯仰/横滚 (2.72)–(2.76)、期望轨迹堆叠 (2.77)。 |
| **[于·摆动]** | 于宪元硕论 §2.6 摆动相规划 | line 1107–1308 | 落足点规划（对称点 (2.78)、近似触地位姿 (2.79)(2.80)、平动/转动项 (2.81)、捕获点速度修正 (2.82)、向心补偿 (2.83)(2.84)、落足点合成 (2.85)(2.86)）、点到点最优轨迹（变分法 (2.87)–(2.95)）、足底轨迹分段 (2.96)–(2.99)。 |
| **[宇·摆动]** | 宇树书 §摆动腿的控制 | line 2915–2935 | 足端 PD 修正力 (5.48)、$\tau=J^\top f_d$ 力矩映射。**凝练入实践节，不全量推导。** |

> **外部出处（作者/源自标）**：
> - **Raibert《Legged Robots that Balance》(1986) §2.2.3** —— 落足启发式来源（笔记 line 1382 自标）；成章 `\cite{raibert1986legged}`（已在 sota 调研待新增）。
> - **捕获点理论（Capture Point）** —— 速度反馈校正项 $\sqrt{z_0/\|g\|}$ 来源（笔记 line 1405）；可 `\cite{pratt2006capture}`（成章按 sota 调研补 bib）。
> - **推广 Euler–Lagrange 方程** —— 二阶导变分（于 line 1229 引文献 [43]）。
> - **倒立摆/LIPM** —— 向心补偿 (2.83) 与捕获点同源；可内部 `\cref` 至 MPC 章 LIPM 对照（`\cite{kajita2001lipm}`）。

> **OCR/转写说明**：MinerU 源公式存在下标丢失/上下标错排/矩阵分隔符瑕疵（详见文末「OCR 可疑处清单」）。本抽取已据上下文与物理意义订正，凡订正处在 `note` 盒标"OCR订正"（原印 vs 订正）。

---

# 本章拟定节结构（对应规范 v5.0：动机→反面→历史→理论→陷阱→练习）

| 节 | 标题 | 主源 | 主要盒子 |
|---|---|---|---|
| §G.1 | **动机**：步态相位是 MPC 已知的未来 | 笔记+于·调度 | `insight`（洞见⑬'⑭'）：相位序列在每步 MPC 求解前已知，是预测时域的接触模式输入 |
| §G.2 | **反面教材**：纯时间切换在崎岖地面的冲击 / 朴素零高度落足在斜坡的"踩坑踩石" | 于·调度 line 708 + 笔记 line 1319 | `pitfall` 两则 |
| §G.3 | **历史/直觉**：步态即周期协调模式（踏步类比）、Raibert 1986 谱系 | 笔记 line 1331–1335 | `insight`（洞见①步态≠移动） |
| §G.4 | **步态调度器**：周期/占空比/偏移量、相位变量、支撑布尔、相位进度 | 于·调度 (2.33)–(2.37) + 笔记 | `definition`+`example`（trot/walk 时序表） |
| §G.5 | **基于速度命令的质心轨迹** | 于·质心 (2.59)–(2.64) | `insight`（洞见④堵转保护）+`example`（数值递推） |
| §G.6 | **地形坡度估计** | 于·质心 (2.66)–(2.76) | `theorem`(最小二乘)+`note`(法向量→欧拉角) |
| §G.7 | **落足点规划**：Raibert 启发式（中性点+速度+旋转）+ 捕获点 + 向心补偿 | 笔记 1394–1411 + 于·摆动 (2.78)–(2.86) | `keyidea`（三项分解）+`insight`（洞见②半支撑相中点、③捕获点刹车） |
| §G.8 | **摆动足底轨迹**：点到点最优（变分→三次多项式）+ xyz 分段（摆线式 z） | 于·摆动 (2.87)–(2.99) | `theorem`（变分推导）+`example`（边界条件解） |
| §G.9 | **陷阱**：Raibert 纯旋转切向外推 / 时间切换时差 / 近似触地位姿误差 | 笔记 1427 + 于 1130/1136 | `pitfall` 三则 |
| §G.10 | **实践**：摆动腿 PD 跟踪与 $\tau=J^\top f$、参数整定 | 宇·摆动 (5.48) | `practice`（不全量推导） |
| §G.x | 练习 | — | `exercise` 4–6 题 |

---

# 第一部分：步态调度器（§G.2–§G.4）

> 主源：[于·调度] line 704–788 + [笔记·步态] line 1329–1376。

## §1.1 步态的定义与直觉（历史/动机）

[笔记 line 1331]：**步态（Gait）** 是在一个周期内各条腿的接触/摆动**时序与相位关系的模式**，即腿部的协调运动模式；这种模式是**周期性**的，实现机器人整体的运动与平衡。

[笔记 line 1333]：**步态周期** = 一个完整循环 = 从一条腿的支撑开始到同一腿下一次支撑（某接触事件开始到下一次相同事件发生所经历的时间）；用时间或相位（$0$ 到 $2\pi$）表示，**步频 = 步态周期的倒数**。

〔作者洞见 ①〕**步态 ≠ 移动运动**（笔记 line 1335，原创直觉，教科书不直述）："步态只是一种协调运动的模式，而不是必须产生移动运动的模式，就好比人可以原地踏步……人可以选择正常踏步、高抬腿踏步、行军式踏步等不同的腿部运动模式（这就是步态的概念）。" → 成章作 `insight`：解耦"步态时序"与"质心运动"两个层次——步态调度器只管腿何时支撑/摆动，质心轨迹（§G.5）单独由速度命令生成。这是后面 MPC 把"接触序列"当**已知外生输入**的概念前提。

## §1.2 ⭐ 切换方式：时间 vs 事件（反面教材）

[于 line 706–730]：支撑/摆动相切换有两种方式。

**基于时间的切换**：无内外传感器辅助，仅靠系统时钟强行切换。问题（line 708）：控制程序内部切换与物理实际切换有时差，尤其崎岖路面——
- 踩到**坑里**：足底未真实触地，程序已切到支撑相 → 足底有较大垂直向下加速度（冲击）。
- 踩到**石头上**：足底已真实触地，程序仍在摆动相 → 足底被迫紧急减速。

**基于事件的切换**：足底加触碰/压力传感器，真实触碰时信号回传。理论上优于时间切换，稳定性上限更高，是未来方向。但五大工程难题（line 712–722）：
1. 足底传感器**加大腿部惯量**（转动惯量 $\propto$ 距离平方，末端加质量急剧增惯量，且末端是全机最快部位）。
2. **寿命**：反复承受落地冲击 + 耐磨 + 防水防酸碱。
3. **可靠性**：漏检触碰会导致足底一直向下探直至腿伸到最长 → 危险。
4. **布线**：信号/供电线从足底经膝、髋回躯干，需耐反复弯折且不被夹伤。
5. **测量角度**：一维应变片压力传感器不适合高速——触地瞬间小腿与地面角度极小，地面反力在传感器方向分量很小，难测（图17）。

**结论**（line 730–731）：于宪元控制系统采用**较保守的基于时间的步态调度器**。

> 〔抽取专员注·成章 `pitfall`〕本节天然是 §G.2 的"反面教材"：先展示纯时间切换在崎岖地面的冲击，引出为何仍需要（且如何用）相位时序，并铺垫 §G.6 地形坡度估计作为"无足底力传感时的几何补偿"。

## §1.3 占空比、偏移量与"占空比-偏移量"描述法

[于 line 733–735 / 笔记 line 1337–1345] 定义：

- **占空比** $\beta$（向量 $\mathbf G_d$）：每条腿支撑相时间占（支撑+摆动）总时间的比例，$\beta = T_{stance}/T_{period}$。
- **偏移量**（向量 $\mathbf G_o$）：一个步态周期中，每条腿支撑相开始时刻占总时间的比例。

[笔记 line 1339–1343] 占空比的步态分类直觉：
- $\beta > 0.5$：**静态步态**（稳定，适合粗糙地形）。
- $\beta \approx 0.5$：**动态步态**（速度高，需平衡控制）。
- $\beta < 0.5$：**跳跃步态**（有飞行相，所有腿离地）。

**"占空比-偏移量"描述法**（于 (2.33)，line 750；笔记 line 1353 同式，腿序 LF-RF-LH-RH）：对角步态 trot

$$
\mathbf{G}(\text{trot}) =
\begin{bmatrix} \mathbf{G}_d \\ \mathbf{G}_o \end{bmatrix}
=
\begin{bmatrix}
\begin{bmatrix} 0.5 & 0.5 & 0.5 & 0.5 \end{bmatrix}^\top \\[2pt]
\begin{bmatrix} 0.5 & 0 & 0 & 0.5 \end{bmatrix}^\top
\end{bmatrix}
\tag{G.1}
$$

[于 line 753] 解读：trot 占空比均 $0.5$；偏移量表示左前(LF)、右后(RH) 先进支撑相，半个周期后切到右前(RF)、左后(LH)。

> 〔记号统一 note〕笔记用 FR/FL/BR/BL（右前/左前/右后/左后）顺序排时序图；于宪元用 LF-RF-LH-RH。**成章统一采用 LF-RF-LH-RH**（与 §G.4 公式索引 $i=1,2,3,4$ 对齐），并在脚注给中文腿名对照。

[于 line 755] **频率与类型解耦**：步态频率通过改变周期长度改变；因步态由"占空比-偏移量"定义，**即使步频变化，步态类型不变**。→ 这是 §G.1 洞见①（时序模式与运动解耦）的又一层。

**常用步态表**（于表 2，line 761）—— 成章直接做 `table`（`\centering`）：

| 步态名称 | 英文 | 占空比 $\mathbf G_d$ | 偏移量 $\mathbf G_o$ |
|---|---|---|---|
| 爬行步态 | walk | $[0.75,0.75,0.75,0.75]^\top$ | $[0.25,0.75,0.5,0]^\top$ |
| 对角步态 | trot | $[0.5,0.5,0.5,0.5]^\top$ | $[0.5,0,0,0.5]^\top$ |
| 跳跃步态 | pronk | $[0.5,0.5,0.5,0.5]^\top$ | $[0,0,0,0]^\top$ |
| 踱步步态 | pace | $[0.5,0.5,0.5,0.5]^\top$ | $[0,0.5,0,0.5]^\top$ |
| 奔跑步态 | bound | $[0.5,0.5,0.5,0.5]^\top$ | $[0,0,0.5,0.5]^\top$ |

> 〔差异/互补〕笔记的步态分类表（line 1365–1369）只列 stance/trot/pace 三类且不全（多行空白），于宪元表 2 五类完整且带数值偏移量。**成章以于宪元表 2 为准**，笔记的 stance（静止站立，所有腿着地）作为补充行。`\cite{yu2021quadruped}` 标表。

## §1.4 ⭐ 相位变量与支撑/摆动判定（调度器核心公式）

[于 line 763] 设 $t$ 为从开始踏步计时的系统时间，$t_{sw}$ 名义摆动相时间，$t_{st}$ 名义支撑相时间，

$$
T_{gait} = t_{sw} + t_{st} \tag{G.2}
$$

设 $n$ 为满足 $t > (n-1)T_{gait}$ 的最大正整数（当前为第 $n$ 个迈步周期），当前周期内时间进度与**相位进度**（归一化）：

$$
t_{ng} = t - (n-1)T_{gait}, \qquad
\tilde t_{ng} = \frac{t_{ng}}{T_{gait}} \in [0,1) \tag{G.3}
$$

**每条腿的相位变量**（于 (2.34)，line 766）——把全局相位平移到该腿自己的支撑起点：

$$
{}_i\tilde t_\Phi =
\begin{cases}
\tilde t_{ng} - \mathbf G_o(i), & \tilde t_{ng} \ge \mathbf G_o(i) \\[2pt]
\tilde t_{ng} - \mathbf G_o(i) + 1, & \tilde t_{ng} < \mathbf G_o(i)
\end{cases}
\tag{G.4}
$$

> 来由：第 $i$ 腿的支撑相在全局相位 $\mathbf G_o(i)$ 处开始；减去偏移量即得该腿的"本地相位"，加 $1$ 是把负值绕回 $[0,1)$（模 1 运算）。

**支撑布尔**（于 (2.35)，line 772）——本地相位落在占空比内即支撑：

$$
{}_iS_\Phi =
\begin{cases}
1, & {}_i\tilde t_\Phi \le \mathbf G_d(i) \quad(\text{支撑相}) \\
0, & {}_i\tilde t_\Phi > \mathbf G_d(i) \quad(\text{摆动相})
\end{cases}
\tag{G.5}
$$

**支撑相相位进度**（于 (2.36)，line 778）——支撑段内归一化到 $[0,1]$：

$$
\tilde t_{st} =
\begin{cases}
{}_i\tilde t_\Phi / \mathbf G_d(i), & {}_iS_\Phi = 1 \\
0, & {}_iS_\Phi = 0
\end{cases}
\tag{G.6}
$$

**摆动相相位进度**（于 (2.37)，line 784）——摆动段内归一化到 $[0,1]$：

$$
\tilde t_{sw} =
\begin{cases}
\dfrac{{}_i\tilde t_\Phi - \mathbf G_d(i)}{1 - \mathbf G_d(i)}, & {}_iS_\Phi = 0 \\[6pt]
0, & {}_iS_\Phi = 1
\end{cases}
\tag{G.7}
$$

[于 line 787] 约定：带上波浪 $\tilde{(\cdot)}$ 表归一化后某过程的进度，取值 $[0,1]$。

> 〔抽取专员注·这是全章的"齿轮箱"〕(G.4)–(G.7) 把单一系统时钟 $t$ 翻译为四条腿各自的"我现在支撑还是摆动、进行到几成"。`${}_iS_\Phi$` 序列即 §G.5 质心轨迹与 MPC 接触约束的接触模式；`$\tilde t_{sw}$` 即 §G.8 足底轨迹的归一化时间参数。**成章把它做成主图（TikZ 时序条 + 相位指针）**。

〔作者洞见 ②〕**钟表指针类比**（笔记 line 1347）："因为步态是周期性循环的，可以使用类似于钟表指针的相位概念来准确描述腿部的支撑和摆动时机，便于生成轨迹和多腿同步。" → 成章 `insight`：相位 = 指针角度，占空比 = 表盘上支撑扇区弧长，偏移量 = 各腿指针的初相位差。

---

# 第二部分：基于速度命令的质心轨迹（§G.5）

> 主源：[于·质心] line 957–1013。

## §2.1 状态维数与上层/下层分工

[于 line 959–961]：完整控制系统 = 上层**运动规划** + 下层**运动跟随**。单刚体模型有 6 自由度，每自由度需位置 + 速度 = **12 个变量**描述某时刻期望状态。本节求这 12 个变量（实际成 13 维，见 (2.76) 含重力常量）。

## §2.2 速度指令 → 偏航角/角速度递推

[于 line 965]：手柄（Logitech F710）给本体系期望速度

$$
{}_d^{\mathcal B}\boldsymbol\nu_{handle} =
\begin{bmatrix} {}_d^{\mathcal B}\nu_x & {}_d^{\mathcal B}\nu_y & 0 & 0 & 0 & {}_d^{\mathcal B}\omega_z \end{bmatrix}^\top
\tag{G.8}
$$

（有头模式更符合人类思维习惯）。设当前为第 0 控制周期，偏航角/角速度递推（于 (2.59)，line 968）：

$$
\begin{cases}
{}_d\psi^{k} = {}_d\psi^{k-1} + {}_d^{\mathcal O}\omega_z^{k-1}\cdot \Delta T \\
{}_d^{\mathcal O}\omega_z^{k} = {}_d^{\mathcal B}\omega_z \\
{}_d\psi^{0} = \psi_0 \\
{}_d^{\mathcal O}\omega_z^{0} = {}_d^{\mathcal B}\omega_z
\end{cases}
\tag{G.9}
$$

[于 line 971]：仅考虑水平移动，忽略俯仰/横滚，故本体系偏航角速度 $\approx$ 世界系偏航角速度；$\Delta T$ 为控制周期。

## §2.3 期望质心速度（本体系 → 世界系）

[于 (2.60)(2.61)，line 976/982]：

$$
{}_d^{\mathcal B}\boldsymbol v_{com}^{k} =
\begin{bmatrix} {}_d^{\mathcal B}v_x & {}_d^{\mathcal B}v_y & 0 \end{bmatrix}^\top,
\qquad
{}_d^{\mathcal O}\boldsymbol v_{com}^{k} = R_z\!\big({}_d\psi^{k}\big)\cdot {}_d^{\mathcal B}\boldsymbol v_{com}^{k}
\tag{G.10}
$$

## §2.4 ⭐ 期望质心位置：堵转保护 + 递推

[于 (2.62)，line 995] 第 1 周期期望位置（含"堵转保护"系数 $\varpi$）：

$$
{}_d^{\mathcal O}\boldsymbol p_{com}^{1} =
\varpi \cdot {}_d^{\mathcal O}\boldsymbol p_{com}^{0}
+ (1-\varpi)\cdot {}^{\mathcal O}\boldsymbol p_{com}^{0}
+ {}_d^{\mathcal O}\boldsymbol v_{com}^{0}\cdot \Delta T
\tag{G.11}
$$

〔作者洞见 ③ —— 实为于宪元原文设计动机，成章作 `insight` 标"于宪元"〕**堵转保护**（于 line 998）："设想若操作机器人撞向一堵墙，期望位置应该在一定程度上'跟随'实际位置，否则机器人会逐步加大推墙的力度，直到失控。$\varpi$ 越大，质心抵抗外力改变其位置的能力越强。" → 即 (G.11) 把期望位置在"纯积分理想位置 ${}_d^{\mathcal O}p_{com}^0$"与"实际位置 ${}^{\mathcal O}p_{com}^0$"之间做凸组合（权 $\varpi$），防止期望与实际偏差无界累积导致力饱和。这是一种**抗积分饱和（anti-windup）**思想在位置规划层的体现。

后续周期纯递推（于 (2.63)，line 1003）：

$$
{}_d^{\mathcal O}\boldsymbol p_{com}^{k} = {}_d^{\mathcal O}\boldsymbol p_{com}^{k-1} + {}_d^{\mathcal O}\boldsymbol v_{com}^{k-1}\cdot \Delta T
\tag{G.12}
$$

高度方向不积分速度，用期望机身高度填充（于 (2.64)，line 1009）：

$$
{}_d^{\mathcal O}\boldsymbol p_{com}^{k}(3) = {}_d h_{body}
\tag{G.13}
$$

> 〔成章 `example`〕给 $\Delta T = 0.002\,\text{s}$、$\varpi = 0.99$、手柄 $v_x = 0.5\,\text{m/s}$、$\omega_z = 0.2\,\text{rad/s}$，递推 3–5 步演示偏航角增长、世界系速度旋转、位置积分与堵转保护融合（数值例须自洽自算，标 `\rebuilt`，不臆造源未给的数）。

---

# 第三部分：地形坡度估计（§G.6）

> 主源：[于·质心 §2.5.2] line 1014–1097 + [笔记] line 1317–1325。

## §3.1 动机（反面教材）

[于 line 1024 / 笔记 line 1319]：斜坡上行走时摆动相落足点 z 坐标**不能简单规划为 0**，否则坡下腿像"踩进坑里"、坡上腿像"踩到石头上"（图22），严重干扰机身控制。故须估算地面坡度合理规划落足高度。

## §3.2 平面方程与支撑足坐标采集

[于 (2.66)，line 1036 / 笔记 line 1325] 地面平面方程（机器人不会在垂直墙面行走）：

$$
z = a + bx + cy = \begin{bmatrix} 1 & x & y \end{bmatrix} A_{pla},
\qquad A_{pla} = \begin{bmatrix} a & b & c \end{bmatrix}^\top
\tag{G.14}
$$

每条腿交替为支撑/摆动。记 ${}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}$ 为第 $i$ 腿**最后一次成为支撑腿时的足底坐标**（支撑则为当前足底坐标，摆动则为最后一次支撑→摆动瞬间的足底坐标）。递推（于 (2.67)，line 1042）：

$$
\begin{cases}
{}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}^{0} = {}_i^{\mathcal O}\boldsymbol p^{\,t=0} \\
{}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}^{k} = {}_i^{\mathcal O}\boldsymbol p, & {}_iS_\Phi = 1 \\
{}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}^{k} = {}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}^{k-1}, & {}_iS_\Phi = 0
\end{cases}
\tag{G.15}
$$

## §3.3 ⭐ 最小二乘平面拟合

[于 (2.68)(2.69)，line 1048/1052] 四足 z 坐标向量与回归设计矩阵：

$$
{}^{\mathcal O}\boldsymbol z_f =
\begin{bmatrix}
{}_1^{\mathcal O}\boldsymbol p_{st\text{-}end}(3) & {}_2^{\mathcal O}\boldsymbol p_{st\text{-}end}(3) & {}_3^{\mathcal O}\boldsymbol p_{st\text{-}end}(3) & {}_4^{\mathcal O}\boldsymbol p_{st\text{-}end}(3)
\end{bmatrix}^\top
\tag{G.16}
$$

$$
W_{pla} =
\begin{bmatrix}
1 & {}_1^{\mathcal O}\boldsymbol p_{st\text{-}end}(1) & {}_1^{\mathcal O}\boldsymbol p_{st\text{-}end}(2) \\
1 & {}_2^{\mathcal O}\boldsymbol p_{st\text{-}end}(1) & {}_2^{\mathcal O}\boldsymbol p_{st\text{-}end}(2) \\
1 & {}_3^{\mathcal O}\boldsymbol p_{st\text{-}end}(1) & {}_3^{\mathcal O}\boldsymbol p_{st\text{-}end}(2) \\
1 & {}_4^{\mathcal O}\boldsymbol p_{st\text{-}end}(1) & {}_4^{\mathcal O}\boldsymbol p_{st\text{-}end}(2)
\end{bmatrix}
\tag{G.17}
$$

伪逆解 + 低通滤波（于 (2.70)(2.71)，line 1058/1064）：

$$
A_{pla} = W_{pla}^{+}\cdot {}^{\mathcal O}\boldsymbol z_f,
\qquad
\hat A_{pla}^{k} = \begin{bmatrix}\hat a & \hat b & \hat c\end{bmatrix}^\top = \eta\, A_{pla} + (1-\eta)\,\hat A_{pla}^{k-1}
\tag{G.18}
$$

其中 $W_{pla}^{+}$ 为伪逆，$\eta = 0.2$（低通系数）。

> 〔成章 `theorem`/`derivation`〕(G.18) 是超定线性回归 $W_{pla}A_{pla} = {}^{\mathcal O}z_f$（4 方程 3 未知）的最小二乘正规解 $A_{pla} = (W_{pla}^\top W_{pla})^{-1}W_{pla}^\top {}^{\mathcal O}z_f = W_{pla}^+ {}^{\mathcal O}z_f$。须补全正规方程来由（标 `\rebuilt`），并解释低通滤波抑制单步触地噪声。

## §3.4 法向量 → 期望俯仰/横滚角

[于 (2.72)，line 1072] 平面法向量及单位化：

$$
\boldsymbol n = \begin{bmatrix} -\hat b & -\hat c & 1 \end{bmatrix}^\top,
\qquad
\boldsymbol n_e = \frac{\boldsymbol n}{\|\boldsymbol n\|}
\tag{G.19}
$$

$\boldsymbol n_e$ 为机体期望旋转矩阵 ${}_d^{\mathcal O}R_{\mathcal B}$ 第三列。结合姿态约定（于 (2.73)，line 1078）：

$$
\boldsymbol n_e =
\begin{bmatrix}
s_\phi s_\psi + c_\phi c_\psi s_\theta \\
c_\phi s_\theta s_\psi - c_\psi s_\phi \\
c_\phi c_\theta
\end{bmatrix}
=
\underbrace{\begin{bmatrix} c_\psi & s_\psi & 0 \\ s_\psi & -c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}}_{Y}
\underbrace{\begin{bmatrix} c_\phi s_\theta \\ s_\phi \\ c_\phi c_\theta \end{bmatrix}}_{\boldsymbol\xi}
= Y\boldsymbol\xi
\tag{G.20}
$$

偏航 $\psi$ 已知 ⇒ $Y$ 已知，解（于 (2.74)，line 1084）：

$$
\boldsymbol\xi = Y^{+}\boldsymbol n_e
\tag{G.21}
$$

由 $\boldsymbol\xi$ 定义反解期望俯仰/横滚（于 (2.75)，line 1090）：

$$
\begin{cases}
{}_d\phi^{1} = \arcsin\!\big(\xi(2)\big) \\
{}_d\theta^{1} = \arctan\!\big(\xi(1)/\xi(3)\big)
\end{cases}
\tag{G.22}
$$

> 来由：$\boldsymbol\xi = [c_\phi s_\theta,\ s_\phi,\ c_\phi c_\theta]^\top$，故 $\xi(2)=s_\phi \Rightarrow \phi=\arcsin\xi(2)$；$\xi(1)/\xi(3)=s_\theta/c_\theta=\tan\theta \Rightarrow \theta=\arctan(\xi(1)/\xi(3))$。期望俯仰/横滚不时变。

## §3.5 期望状态向量堆叠

[于 (2.76)，line 1096] 整合期望欧拉角 ${}_d\Theta^k = [{}_d\phi^1,\ {}_d\theta^1,\ {}_d\psi^k]^\top$，第 $k$ 周期 13 维期望状态：

$$
{}_d\boldsymbol x^{k} =
\begin{bmatrix}
{}_d\boldsymbol\Theta^{k} \\
{}_d\boldsymbol p_{com}^{k} \\
{}_d\boldsymbol\omega^{k} \\
{}_d\boldsymbol v_{com}^{k} \\
{}^{\mathcal O}\mathbf g(3)
\end{bmatrix},
\qquad {}^{\mathcal O}\mathbf g(3) = -9.8\ \text{m/s}^2
\tag{G.23}
$$

未来 $h$ 周期堆叠为期望轨迹（于 (2.77)，line 1102），大小 $13h \times 1$：

$$
D = \begin{bmatrix} ({}_d\boldsymbol x^1)^\top & ({}_d\boldsymbol x^2)^\top & \cdots & ({}_d\boldsymbol x^h)^\top \end{bmatrix}^\top
\tag{G.24}
$$

〔抽取专员注·**全章主线收口**〕(G.23)(G.24) 正是 `\cref{ch:quad_mpc}` 凸 MPC 的**期望轨迹输入** $D$，而第 13 维重力常量 ${}^{\mathcal O}g(3)$ 即 MPC 章重力增广状态（对应 MPC 个人笔记洞见⑥）。此处 `\cref` 到 MPC 章；成章在 §G.1 与本节首尾呼应"步态/轨迹章为 MPC 供料"主旨。

---

# 第四部分：落足点规划（§G.7）

> 主源：[笔记] line 1378–1429（Raibert 直觉与三项分解）+ [于·摆动 §2.6.1] line 1111–1194（工程实现公式）。**两源互补：笔记给物理直觉与三项分解，于给可落地的对称点/触地预测/四项叠加。**

## §4.1 动机与根本目标（直觉）

〔作者洞见 ④〕**落足点的人类跑步类比**（笔记 line 1380，原创引入）："想象人类跑步时脚并不是随意落下的。为向前跑，脚自然落在重心前方；为刹车，落在重心后方；为转弯，落点偏向内侧。这个过程是下意识的，但背后遵循精确物理规律。" → 成章作 `insight` 开篇。

**根本目标**（笔记 line 1382，加粗）：**通过主动选择落足点位置，产生期望地面反作用力，从而控制机身运动并维持动态平衡。**

## §4.2 Raibert 启发式（笔记版三项分解）

[笔记 line 1382] 最简方法 = **Raibert 启发式**（平面行走、落足点总在地面，机身系下计算后转世界系；推理见 Raibert《Legged Robots that Balance》§2.2.3）。预知参数表（笔记 line 1386）：控制指令 $\mathbf v_d,\boldsymbol\omega_d$；当前状态 $\mathbf v_{mes}$；步态参数 $T_{period},T_{stance},T_{swing}$。

**(1) 中性落足点** ${}^{B}\mathbf p_{f,neutral}$（笔记 line 1394）：静止站立时每条腿的默认落足点，通常在对应髋关节正下方地面，机身系描述。

**(2) 速度补偿项**（笔记 line 1400）——支撑机身在未来支撑相内运动，落足点提前到未来支撑中心：

$$
\Delta\mathbf p_{vel} =
\underbrace{\frac{T_{stance}}{2}\cdot {}^{B}\mathbf v_{curr}}_{\text{前馈项}}
+ \underbrace{\sqrt{\frac{z_0}{\|g\|}}\,\big({}^{B}\mathbf v_{curr} - {}^{B}\mathbf v_{des}\big)}_{\text{反馈校正项}}
\tag{G.25}
$$

〔作者洞见 ⑤〕**为何是半个支撑相**（笔记 line 1403）："前馈项预测半个支撑相内机身移动距离。为什么是半个？因为将脚落在未来支撑阶段的**中点**，可使整个支撑阶段的力最均匀，机身姿态最稳定。"

〔作者洞见 ⑥〕**反馈校正项 = 捕获点刹车**（笔记 line 1405）："若当前速度比期望快，差值为正，落足点更靠前，起'刹车'作用；反之靠后产生更大前推力。该公式来源于**捕获点理论**，$z_0$ 是行走名义高度。" → 成章 `insight` 标"捕获点"；$\sqrt{z_0/\|g\|}$ 即 LIPM 时间常数 $1/\omega_n$，可 `\cref` MPC 章 LIPM。

**(3) 旋转补偿项**（笔记 line 1409）——转弯时外侧腿迈得更远：

$$
\Delta\mathbf p_{rot} = \frac{T_{stance}}{2}\cdot\big({}^{B}\boldsymbol\omega_{des}\times {}^{B}\mathbf p_{f,neutral}\big)
\tag{G.26}
$$

[笔记 line 1411]：叉乘计算中性点处由期望角速度导致的足端切向速度，乘半支撑相得额外位移以平稳支撑旋转。

三项叠加得机身系完整落足点，再转世界系供摆动相轨迹生成（笔记 line 1413）。

〔作者洞见 ⑦〕**Raibert 公式的本质与定位**（笔记 line 1425）："这个公式实际上是对复杂动力学模型进行简化和线性化的结果，结合了物理直觉、控制经验，定义的落脚点**并不是最优的，但非常鲁棒且易于计算**；想找严格最优落脚点需求解复杂非线性动力学方程。" → 成章 `insight`：鲁棒性 vs 最优性的工程取舍，呼应 §G.8 才用最优变分。

## §4.3 落足点修正（结合感知）

[笔记 line 1415–1423] 平面假设不成立时（石块/水坑/台阶边缘），加**落足点修正**流程：
1. **生成地形图**：LiDAR/深度相机实时生成 3D 高度图（Height Map）或代价图（Cost Map）。
2. **验证理想落足点**：检查 Raibert 理想点在地形图上是否安全（平坦、无障碍）。
3. **搜索安全区域**：不安全则在周围小范围（如半径 5–10 cm 圆）搜代价最低（最安全/最平坦）替代点。
4. **更新目标**：以修正后安全点为最终结果。

[笔记 line 1431]：不规则地面须依赖主动感知（LiDAR、相机）实现 3D 环境感知与规划，主动寻找落足点。

## §4.4 ⭐ 于宪元版工程落足点（四项叠加）

[于 §2.6.1] 与笔记三项分解互补，给出可计算的对称点 + 四个修正项。

**对称点（机身系）**（于 (2.78)，line 1118）：

$$
{}_i^{\mathcal B}\boldsymbol p_{hip} =
\begin{bmatrix} \delta h_x \\ \zeta(h_y + l_1) \\ 0 \end{bmatrix}
+ \begin{bmatrix} p_{offset}^{x} \\ 0 \\ 0 \end{bmatrix}
\tag{G.27}
$$

[于 line 1121] 第一项为关节角 0 时第二连杆坐标原点在本体系的水平坐标；第二项 $p_{offset}^x = -0.018\,\text{m}$（全肘式结构质心偏后，落足点稍靠后，否则踏步时机器人在重力下有后退趋势）。$\delta,\zeta$ 为腿位符号（前后/左右）。

**近似触地位姿**（于 (2.79)，line 1133）——避免 (G.12) 大量矩阵递推：

$$
\begin{cases}
{}^{\mathcal O}\boldsymbol p_{com}^{touch} = {}^{\mathcal O}\boldsymbol p_{com}^{t} + {}^{\mathcal O}R_{\mathcal B}^{t}\cdot {}_d^{\mathcal B}\boldsymbol v_{com}^{t}\cdot (1 - {}_i\tilde t_{sw})\,t_{sw} \\
\psi^{touch} = \psi^{t} + {}_d\omega_z^{t}\cdot (1 - {}_i\tilde t_{sw})\,t_{sw}
\end{cases}
\tag{G.28}
$$

其中名义触地时刻 ${}_it_{touch} = t + (1 - {}_i\tilde t_{sw})\,t_{sw}$（于 line 1130）。

〔作者洞见 ⑧ —— 于宪元设计自述，成章 `insight`〕**近似误差自消失**（于 line 1136）："尽管这是近似方法，但随 ${}_i\tilde t_{sw}$ 逐渐逼近 1，${}^{\mathcal O}p_{com}^t$ 也逐渐逼近 ${}^{\mathcal O}p_{com}^{touch}$，误差将逐渐减少并在名义触地时刻消失。" → 在线滚动重算使近似在临界处自动收敛——与 MPC 章"反馈补偿模型误差"同一哲学。

名义触地对称点（世界系）（于 (2.80)，line 1139）：

$$
{}_i^{\mathcal O}\boldsymbol p_{hip}^{touch} = {}^{\mathcal O}\boldsymbol p_{com}^{touch} + R_z(\psi^{touch})\,{}_i^{\mathcal B}\boldsymbol p_{hip}
\tag{G.29}
$$

**平动项 $\Delta p_1$ 与转动项 $\Delta p_2$**（于 (2.81)，line 1145）——使支撑足起止坐标均值落在对称点（落足点设在实际速度方向、距对称点半步长）：

$$
\begin{cases}
\Delta\boldsymbol p_1 = {}_d^{\mathcal O}\boldsymbol v_{com}^{t}\cdot t_{st}/2 \\
\Delta\boldsymbol p_2 = R_z(\psi^{touch})\Big[ R_z\big({}_d\omega_z^{t}\cdot t_{st}/2\big)\cdot {}_i^{\mathcal B}\boldsymbol p_{hip} - {}_i^{\mathcal B}\boldsymbol p_{hip} \Big]
\end{cases}
\tag{G.30}
$$

**捕获点速度修正 $\Delta p_3$**（于 (2.82)，line 1153）——在一方向增加落足距离将引发反方向加速度（足式系统内在特性，文献 [8]），修正质心速度跟踪误差：

$$
\Delta\boldsymbol p_3 = k_p\big({}^{\mathcal O}\boldsymbol v_{com}^{t} - {}_d^{\mathcal O}\boldsymbol v_{com}^{t}\big),
\qquad k_p = 0.15
\tag{G.31}
$$

> 〔差异/互补 note〕(G.31) 与笔记 (G.25) 反馈校正项同理（速度误差 → 落足偏移），但于用比例系数 $k_p$，笔记用捕获点系数 $\sqrt{z_0/\|g\|}$。成章并列两式，note 标二者皆"速度误差→落足前后偏移"的捕获点思想，系数取法不同。

**向心补偿 $\Delta p_4$**（于 (2.83)，line 1168）——匀速圆周运动倒立摆，支撑点相对质心总在远离圆心方向：

$$
\Delta\boldsymbol p_4 = \frac{{}^{\mathcal O}\boldsymbol p_{com}^{t}(3)}{-{}^{\mathcal O}\boldsymbol g(3)}\,{}^{\mathcal O}\boldsymbol v_{com}^{t}\times {}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}}^{t}
\tag{G.32}
$$

来由（于 (2.84)，line 1174）——对位置约束 ${}^A\dot{\mathbf r}_{AP}$ 微分、忽略角加速度得向心加速度 $\mathbf a = -{}^{\mathcal O}\boldsymbol v_{com}^t\times{}^{\mathcal O}\boldsymbol\omega_{\mathcal{OB}}^t$：

$$
{}^A\ddot{\mathbf r}_{AP}
= {}^A\dot{\boldsymbol\Omega}_B\,{}^A\mathbf r_{AP} + {}^A\boldsymbol\Omega_B\,{}^A\dot{\mathbf r}_{AP}
= \mathbf 0 + {}^A\boldsymbol\omega_{AB}\times {}^A\dot{\mathbf r}_{AP}
= -{}^A\dot{\mathbf r}_{AP}\times {}^A\boldsymbol\omega_{AB}
\tag{G.33}
$$

**落足点合成**（于 (2.85)，line 1187）——x、y 方向：

$$
{}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end} = {}_i^{\mathcal O}\boldsymbol p_{hip}^{touch} + \Delta\boldsymbol p_1 + \Delta\boldsymbol p_2 + \Delta\boldsymbol p_3 + \Delta\boldsymbol p_4
\tag{G.34}
$$

z 坐标代入坡度平面 (G.14)（于 (2.86)，line 1193）：

$$
{}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end}(3) = a + b\cdot {}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end}(1) + c\cdot {}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end}(2)
\tag{G.35}
$$

> 〔抽取专员注·闭环〕(G.35) 用 §G.6 坡度估计的 $(a,b,c)$ 给落足点 z 坐标 → 正是 §G.3 动机（斜坡不能简单置 0）的兑现。坡度估计与落足点规划在此闭环。

## §4.5 ⭐ Raibert 的纯旋转缺陷（陷阱）

〔作者洞见 ⑨〕**Raibert 纯旋转切向外推缺陷**（笔记 line 1427，原创批判，重要陷阱）："Raibert 方法有一个很严重的缺点：在只有角速度指令不为零时，落足点补偿项只有一个切向补偿，根据叉乘物理意义，会将落足点沿相对机身的环形轨道切向**向外推动**，导致期望落足点相对机身中心距离增加（而理想情况下仅旋转时落足点应在某圆上，圆心是质心在地面的投影）；若增幅过大会使期望落足点**超出足端可达区域**，因此需限幅约束。" → 成章 `pitfall`：纯转动指令下 (G.26) 只给切向位移、缺径向收敛，落足点半径漂移；改进可加径向修正或限幅。[笔记 line 1429]：Raibert 是很早期方法，后续有更优方法及结合感知的落足搜索（呼应 §4.3）。

---

# 第五部分：摆动足底轨迹（§G.8）

> 主源：[于·摆动 §2.6.2] line 1196–1308。

## §5.1 点到点最优轨迹（变分法推导）

[于 line 1198]：${}_i^{\mathcal O}p_{st\text{-}end}$（${}_i\tilde t_{sw}=0$ 起点）与落足点 ${}_i^{\mathcal O}p_{sw\text{-}end}$（终点）已知 ⇒ 足底轨迹是**点到点规划问题**。

**最优性问题**（于 line 1202）：单位质量滑块在光滑地面，水平力 $u$，初位置/速度为 0，要求单位时间内走单位距离并停止。无穷多轨迹，求最优。动力学（于 (2.87)，line 1214）：

$$
\dot x = v, \qquad \ddot x = \dot v = u
\tag{G.36}
$$

**评价指标**（最小控制力平方积分，于 (2.88)，line 1220）：

$$
J = \int_0^1 u^2\,dt = \int_0^1 \ddot x^2\,dt
\tag{G.37}
$$

变分法，构造被积函数（于 (2.89)，line 1226）：

$$
\mathcal L(t,x,\dot x,\ddot x) = \ddot x^2
\tag{G.38}
$$

含二阶导，用**推广 Euler–Lagrange 方程**（于 (2.90)，line 1232，文献 [43]）：

$$
\frac{\partial\mathcal L}{\partial x}
- \frac{d}{dt}\frac{\partial\mathcal L}{\partial\dot x}
+ \frac{d^2}{dt^2}\frac{\partial\mathcal L}{\partial\ddot x}
= 0
\tag{G.39}
$$

> 〔OCR订正 note〕于 (2.90) 印为 `- d²/dt²(∂L/∂ẍ)`（第三项取负号）。**推广 E–L 方程标准形第三项应为 `+`**：$\frac{\partial\mathcal L}{\partial x} - \frac{d}{dt}\frac{\partial\mathcal L}{\partial\dot x} + \frac{d^2}{dt^2}\frac{\partial\mathcal L}{\partial\ddot x}=0$。但因前两项为 0、$\partial\mathcal L/\partial\ddot x = 2\ddot x$，无论第三项正负，结论 $\frac{d^2}{dt^2}(2\ddot x)=0$ 不变，故不影响最终结果。成章用标准 `+` 号并 note 标原印取负。

代入 $\partial\mathcal L/\partial x=0$、$\partial\mathcal L/\partial\dot x=0$、$\partial\mathcal L/\partial\ddot x=2\ddot x$（于 (2.91)(2.92)，line 1240/1246）：

$$
\frac{d^2}{dt^2}(2\ddot x) = 0 \;\Longrightarrow\; \frac{d^4}{dt^4}x = 0
\tag{G.40}
$$

故 $x(t)$ 是 $t$ 的三次函数（于 (2.93)，line 1252）：

$$
\begin{cases}
x(t) = a t^3 + b t^2 + c t + d \\
v(t) = 3a t^2 + 2b t + c
\end{cases}
\tag{G.41}
$$

边界条件（于 (2.94)，line 1258）：$x(0)=0,\ v(0)=0,\ x(1)=1,\ v(1)=0$，解得（于 (2.95)，line 1264）：

$$
\boxed{\;x(t) = 3t^2 - 2t^3, \qquad v(t) = 6t - 6t^2\;}
\tag{G.42}
$$

> 〔成章 `theorem`〕(G.42) 即归一化**三次/五次样条最小加加速度轨迹**的最简形（实为最小 jerk 的特例：$d^4x/dt^4=0$）。须补全四元一次方程组解 $a=-2,b=3,c=0,d=0$ 的过程（标 `\rebuilt`）。物理意义：起止速度 0、加速度光滑，控制力平方积分最小。

## §5.2 足底轨迹合成（xy 三次、z 摆线式分段）

起止向量（于 (2.96)，line 1272）：

$$
{}_i^{\mathcal O}\boldsymbol p_{st\text{-}sw} = {}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end} - {}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}
\tag{G.43}
$$

**x、y 方向**用 (G.42) 整形（于 (2.97)，line 1285）：

$$
\begin{cases}
{}_d x({}_i\tilde t_{sw}) = {}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}(1) + {}_i^{\mathcal O}\boldsymbol p_{st\text{-}sw}(1)\cdot\big(3\,{}_i\tilde t_{sw}^2 - 2\,{}_i\tilde t_{sw}^3\big) \\
{}_d y({}_i\tilde t_{sw}) = {}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}(2) + {}_i^{\mathcal O}\boldsymbol p_{st\text{-}sw}(2)\cdot\big(3\,{}_i\tilde t_{sw}^2 - 2\,{}_i\tilde t_{sw}^3\big) \\
{}_d\dot x({}_i\tilde t_{sw}) = {}_i^{\mathcal O}\boldsymbol p_{st\text{-}sw}(1)\cdot\big(6\,{}_i\tilde t_{sw} - 6\,{}_i\tilde t_{sw}^2\big) \\
{}_d\dot y({}_i\tilde t_{sw}) = {}_i^{\mathcal O}\boldsymbol p_{st\text{-}sw}(2)\cdot\big(6\,{}_i\tilde t_{sw} - 6\,{}_i\tilde t_{sw}^2\big)
\end{cases}
\tag{G.44}
$$

**z 方向**分抬腿/落腿两段（于 (2.98)，line 1291）——先升到抬腿高度 ${}_dh_{foot}$ 再落到终点高度：

$$
\begin{cases}
{}_d z = {}_i^{\mathcal O}\boldsymbol p_{st\text{-}end}(3) + {}_d h_{foot}\cdot(3\sigma^2 - 2\sigma^3), & \sigma = 2\,{}_i\tilde t_{sw} \le 1 \\[4pt]
{}_d z = {}_d z(0.5) + \big[{}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end}(3) - {}_d z(0.5)\big](3\sigma^2 - 2\sigma^3), & \sigma = 2({}_i\tilde t_{sw}-0.5) > 0 \\[4pt]
{}_d\dot z = {}_d h_{foot}\cdot(6\sigma - 6\sigma^2), & \sigma = 2\,{}_i\tilde t_{sw} \le 1 \\[4pt]
{}_d\dot z = \big[{}_i^{\mathcal O}\boldsymbol p_{sw\text{-}end}(3) - {}_d z(0.5)\big](6\sigma - 6\sigma^2), & \sigma = 2({}_i\tilde t_{sw}-0.5) > 0
\end{cases}
\tag{G.45}
$$

其中 ${}_d h_{foot}$ 为期望抬腿高度。合成期望足底轨迹（于 (2.99)，line 1297）：

$$
{}_i^{\mathcal O}\boldsymbol p^{d} = \begin{bmatrix} {}_d x & {}_d y & {}_d z \end{bmatrix}^\top,
\qquad
{}_i^{\mathcal O}\dot{\boldsymbol p}^{d} = \begin{bmatrix} {}_d\dot x & {}_d\dot y & {}_d\dot z \end{bmatrix}^\top
\tag{G.46}
$$

> 〔抽取专员注〕z 分段中令 $\sigma=2\tilde t_{sw}$（前半 $\tilde t_{sw}\in[0,0.5]$ 抬腿到峰、后半落到终点），每段仍用三次形 $3\sigma^2-2\sigma^3$，保证抬腿过程通过性 + 峰点处速度连续（峰点 $\sigma=1$ 时 $\dot z\propto 6-6=0$）。这就是 §G.8 "摆线式 z"的实质——分段三次而非真正摆线，但形态相近。成章 TikZ 重绘图28（xyz 三曲线）。

> 〔差异/互补〕笔记 §摆动相规划未展开足底轨迹（只到落足点修正止），**于宪元独家**给出点到点最优 + 分段实现；宇树书只给足端 PD 跟踪（见 §G.10）。三源在此互补：笔记=落足直觉、于=轨迹生成、宇树=轨迹跟踪。

---

# 第六部分：实践——摆动腿轨迹跟踪（§G.10，凝练入 practice 盒）

> 主源：[宇·摆动] line 2915–2925。**纯跟踪控制，凝练不全量推导。**

[宇 line 2917–2919]：摆动腿控制 = 足端腾空、跟随目标轨迹位置/速度、落到期望落脚点。已知基座系 $\{0\}$ 下足端目标位置 $p_{0d}$、目标速度 $\dot p_{0d}$ → 逆运动学求关节目标角 $q_d$、一阶微分逆运动学求目标角速度 $\dot q_d$ → 直接发关节位置/速度命令。但纯关节控制下足端误差难修正，故引入足端**修正力 $f_d$**（PD，于 (5.48)，line 2922）：

$$
\boldsymbol f_d = \boldsymbol K_p(\boldsymbol p_{0d} - \boldsymbol p_{0f}) + \boldsymbol K_d(\dot{\boldsymbol p}_{0d} - \dot{\boldsymbol p}_{0f})
\tag{G.47}
$$

$\boldsymbol K_p,\boldsymbol K_d$ 对角正定。修正力经雅可比转关节力矩（于 line 2925）：

$$
\boldsymbol\tau = \boldsymbol J^\top \boldsymbol f_d
\tag{G.48}
$$

[宇 line 2925]：足端 x 位置小于目标时产生 +x 修正力驱足端加速缩小偏差；腿速不快时单腿静力学近似成立。整定关节位置刚度 $k_p$、速度刚度 $k_d$ 与 $\boldsymbol K_p,\boldsymbol K_d$ 使足端良好跟随。

> 〔成章 `practice`〕跳过宇树书 §5.5「实践：让腿动起来」的 FSM/SwingTest/C++ 代码细节（src/FSM/State_SwingTest.cpp、unitreeLeg.h 等，line 2929–2939），仅保留 (G.47)(G.48) 物理与整定要点。`\cref` 至运动学/雅可比章。$\boldsymbol\tau=\boldsymbol J^\top\boldsymbol f$ 力-力矩对偶可 `\cref` 静力学/雅可比转置章。

---

# 作者洞见清单（成章 `insight` 盒索引）

| 编号 | 洞见 | 源行 | 成章用途 |
|---|---|---|---|
| ① | 步态 ≠ 移动运动（踏步类比，时序与质心运动解耦） | 笔记 1335 | §G.1/G.3 开篇 insight |
| ② | 相位 = 钟表指针（占空比=支撑扇区、偏移量=初相位差） | 笔记 1347 | §G.4 insight |
| ③ | 堵转保护：期望位置凸组合实际位置防力饱和（anti-windup） | 于 998 | §G.5 insight（标"于宪元"） |
| ④ | 落足点的人类跑步类比（前/后/内侧落点） | 笔记 1380 | §G.7 开篇 insight |
| ⑤ | 速度前馈落在半支撑相中点使支撑力最均匀 | 笔记 1403 | §G.7 insight |
| ⑥ | 反馈校正=捕获点刹车，$\sqrt{z_0/\|g\|}$=LIPM 时间常数 | 笔记 1405 | §G.7 insight（cref LIPM） |
| ⑦ | Raibert 公式鲁棒但非最优（vs §G.8 最优变分） | 笔记 1425 | §G.7 insight |
| ⑧ | 近似触地位姿误差随相位逼近 1 自动消失 | 于 1136 | §G.7 insight（标"于宪元"） |
| ⑨ | Raibert 纯旋转切向外推缺陷（落足半径漂移、超可达域） | 笔记 1427 | §G.9 pitfall（核心陷阱） |
| ⑬'/⑭' | 步态相位序列是 MPC 已知的未来（接触模式外生输入） | 抽取专员据 G.5/G.23 综合 | §G.1 主旨 insight（cref MPC 章） |

---

# OCR 可疑处清单（原印 vs 推断订正）

| # | 出处 | 原印（MinerU/笔记） | 推断订正 | 依据 |
|---|---|---|---|---|
| O1 | 于 (2.90) line 1232 | 推广 E–L 第三项 `- d²/dt²(∂L/∂ẍ)`（负号） | 标准形第三项应为 `+ d²/dt²(∂L/∂ẍ)` | 推广 Euler–Lagrange 标准式；本例两端为 0 故结论不变（已 note 说明） |
| O2 | 于 (2.33) line 750 | 矩阵分隔符错排 `[[0.5 & 0.5 & 0.5 & 0.5]^T \\ [0.5 & 0 & 0 & 0.5]^T]`（两行混入列分隔） | $\mathbf G_d=[0.5,0.5,0.5,0.5]^\top$、$\mathbf G_o=[0.5,0,0,0.5]^\top$ 两个列向量堆叠 | 与笔记 (G.1)、表 2 一致 |
| O3 | 于 (2.36)(2.37) line 778/784 | 下标 ${}_iS_\Phi$ 排作 `_{iS_Φ}`、$s_\phi$ 与 $S_\Phi$ 混用大小写 | 统一 ${}_iS_\Phi$（布尔，(2.35)） | (2.35) 定义；摆动条件应为 ${}_iS_\Phi=0$ |
| O4 | 于 (2.59)(2.62) 等 | 大量左下标 $d$（期望）、左上标 $\mathcal O/\mathcal B$ 被 MinerU 拆成 `_{d}^{\mathcal O}` 散乱字符、`\varpi`/`\eta` 旁注丢失符号名 | 据上下文恢复 ${}_d^{\mathcal O}(\cdot)$、$\varpi$（堵转系数）、$\eta=0.2$ | (2.62) 文字解释、(2.71) $\eta=0.2$ |
| O5 | 于 (2.65) line 1019 | 期望角速度向量印作 `_d^O ω^k = [0 0 _d^O ω_z^k]^T` 前缀符号散 | ${}_d^{\mathcal O}\boldsymbol\omega^k=[0,0,{}_d^{\mathcal O}\omega_z^k]^\top$ | 文字"期望俯仰/横滚速度设 0" |
| O6 | 于 (2.73) line 1078 | $Y$ 矩阵第二行 `[s_ψ -c_ψ 0]`（对称矩阵但非旋转阵） | 保留原印（$Y$ 非旋转阵，是 $n_e$ 分量重排矩阵，对称） | 与 (2.74) $Y^+$ 伪逆解一致；物理上 $Y$ 仅依赖 $\psi$ 已知 |
| O7 | 笔记 line 1325 / 于 (2.66) | 平面方程笔记只给 $z=a+bx+cy$，缺矩阵形 | 补 $=[1\ x\ y]A_{pla}$（于 (2.66)） | 两源融合，落足 z 解算 (G.35) 需矩阵形 |
| O8 | 于 (2.78) line 1118 | $\delta h_x$、$\zeta(h_y+l_1)$ 中 $\delta,\zeta$ 未定义（疑为腿位符号 ±1） | 据"前后/左右腿"推断 $\delta,\zeta\in\{+1,-1\}$ 选位符号 | 全肘式 4 腿对称布局；成章 note 标存疑 `\pz` |
| O9 | 于 (2.83) line 1168 | 向心补偿分母 `-^O g(3)`、叉乘顺序 `v×ω` | 保留 $\dfrac{p_{com}(3)}{-g(3)}v\times\omega$（$-g(3)>0$，得正高度因子） | (2.84) 推导 $a=-\dot r\times\omega$ 一致 |

> **OCR 总体说明**：MinerU 对左上/左下标（$\mathcal O/\mathcal B$、$d$、$i$）与多行向量分隔符识别最差，本章绝大多数公式均经手工恢复记号；矩阵结构与系数数值（$\eta=0.2$、$k_p=0.15$、$p_{offset}^x=-0.018$、$g=-9.8$）均与源文字描述交叉核对无误。

---

# 三源差异/互补总表（供成章融合）

| 主题 | 个人笔记 | 于宪元硕论 | 宇树书 | 融合策略 |
|---|---|---|---|---|
| 步态定义 | 直觉强（踏步类比、步态≠移动） | 工程定义（占空比/偏移量精确） | — | 笔记开篇直觉 → 于公式化 |
| 步态时序 | trot/walk 时序图、3 类表（不全） | 5 步态完整表 + 相位变量 (2.34)–(2.37) | — | 以于表 2 + 相位公式为主体 |
| 质心轨迹 | — | 完整 (2.59)–(2.77) | — | 于独家，全量收录 |
| 坡度估计 | 只给平面方程 $z=a+bx+cy$ | 完整最小二乘 + 法向→欧拉角 | — | 笔记引入动机，于给算法 |
| 落足点 | 三项分解 + 物理意义 + Raibert 缺陷 | 四项叠加 + 触地预测工程式 | — | 笔记讲"为什么"，于讲"怎么算"，并列两套 |
| 足底轨迹 | 未展开 | 变分最优 + xyz 分段 (2.87)–(2.99) | — | 于独家，全量收录 |
| 轨迹跟踪 | — | — | 足端 PD + $\tau=J^\top f$ | 宇树入 practice 盒 |

---

# 收录统计

- **关键公式数**：G.1–G.48 共 **48 条**编号公式（含三源核心式与必要推导中间式）。
- **作者洞见点**：10 条（①–⑨ + MPC 主旨 ⑬'/⑭'）。
- **OCR 可疑处**：9 条（O1–O9）。
- **citation 落点**：步态/质心/坡度/落足/足底轨迹核心式 `\cite{yu2021quadruped}`；步态分类直觉/Raibert 三项/捕获点 `\cite{unitree2023quadruped}` 或个人笔记 + Raibert/捕获点外部 bib（按 sota 调研补）；足端 PD 跟踪 `\cite{unitree2023quadruped}`。
