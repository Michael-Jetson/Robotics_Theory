# 富种子：浮动基座建模与运动学（tag=kin · 四足运动控制部 · 地基章）

> **本文件性质**：项目内部「写作种子」，**非成书正文**。服务于出版级中文教材新部「四足机器人运动控制」之 **第「浮动基座建模与运动学」章**（章 label 拟 `ch:quad_kin`）。
>
> **本章定位**：运动学地基章。给出浮动基座（floating-base）系统的位姿描述与坐标系约定、单腿正逆运动学、足端雅可比、一阶微分运动学与静力学对偶；用空间向量/Plücker 代数与运动学树推导递推正运动学与全身雅可比；重点把"奇异性与零空间理论"作为 WBC（后续章）的数学地基讲透（含零空间投影证明）。与本书 `\cref{ch:lie}`/`\cref{ch:rigid_body}` 衔接但聚焦四足。
>
> **抽取原则**：完全吸收 · 禁摘要。逐式逐推导录入、标源节号/页。三源记号冲突时以本书全局约定统一（右扰动主线、Hamilton 四元数、se(3) 平移在前），并在 `note` 盒标换算。OCR 可疑处据上下文订正并在 `note` 盒标"OCR 订正"。

> **本批次源映射**：
> | 代号 | 来源 | 行号 | 承担内容 |
> |---|---|---|---|
> | **[笔记]** | 作者个人笔记 `足式机器人控制.md` | 99–458 + 1221–1254 | 坐标变换总论、FK/IK 概念、DH、一阶微分运动学与静力学、**奇异性与零空间理论（含零空间投影/MP 伪逆的拉格朗日证明）**、四足坐标系/关节命名/电机零点。**本章思路主线与作者洞见的主要承载者。** |
> | **[宇树]** | 《四足机器人控制算法》(卞泽坤·王兴兴) | 1467–2528 + 2529–2993 + 2994–3114 | 2D/3D 旋转矩阵（坐标变换视角）、轴角/指数坐标、矩阵对数、齐次变换、欧拉角；**单腿 FK（齐次变换连乘）、单腿 IK（解耦/atan2/余弦定理）、单腿雅可比（直接求导）、单腿静力学 $\tau=J^TF$、摆动腿控制；6.1 四足姿态↔足端位置、牵连速度+关节速度→足端速度。** |
> | **[于]** | 于宪元硕士论文《基于稳定性的仿生四足机器人控制系统设计》(北航,2021) | 425–703 + 2168–3898 | §2.2 坐标系与刚体位姿（旋转矩阵性质、ZYX 欧拉角及逆解、**角速度张量与反对称算子的严格推导**）；**第五章 Plücker 六维空间向量（运动/力向量、坐标变换 $X$/$X^*$、空间叉乘 $v\times$/$v\times^*$、空间加速度、空间惯量、运动方程）、运动学树、关节模型 $S_i$、足底接触点、递推正运动学算法、全身控制雅可比 $J_1\dots J_4$ 与 $\dot J\dot q$ 项。** |

> **引用标记法**：逐重要公式标源。成章 BibTeX 键拟：宇树书 = `\cite{unitree2023quadruped}`；于宪元 = `\cite{yu2021quadruped}`。

---

## A. 本章拟定节结构（对应教材 v5.0 流水线：动机 → 反面 → 历史 → 理论 → 陷阱 → 练习）

| 节 | 标题（拟） | 主源 | 盒子/形式 |
|---|---|---|---|
| §0 | **动机**：从"控关节"到"控足端、控机身" | [笔记]2533/[宇树]6.1 | `motivation` 引入：12 个电机是唯一可直接控的量，运动学是关节↔足端↔机身的桥 |
| §1 | **反面教材**：欧拉角直接做姿态长期描述 / 朴素求逆遇奇异 | [笔记]112,380 | `pitfall`：万向锁、$JJ^T$ 病态数值溢出 |
| §2 | **历史脉络**：DH→旋量/指数坐标→Featherstone 空间向量 | [笔记]212/[宇树]4.3/[于]§5.1 | `history`：SDH/MDH、Plücker 代数为现代多刚体动力学统一框架 |
| §3 | **坐标系与位姿描述**：四足坐标系体系、记号约定、旋转矩阵、欧拉角、角速度张量 | [于]§2.2/[宇树]§4 | 理论主体；`def` 盒 |
| §4 | **浮动基座与运动学树**：13 刚体、父节点集、虚拟 6-DoF 关节 | [于]§5.2.1 | `def`+TikZ 树图 |
| §5 | **单腿正运动学**：齐次变换连乘、三维单腿 FK 闭式 | [宇树]§5.1 | 数值例 |
| §6 | **单腿逆运动学**：解耦、atan2/余弦定理三步解 | [宇树]§5.2 | 例题+`pitfall`（多解/限位） |
| §7 | **一阶微分运动学与静力学对偶**：足端雅可比、$\tau=J^TF$ | [宇树]§5.3/[笔记]286 | `theorem` 对偶 |
| §8 | **空间向量（Plücker）与递推正运动学**：六维运动/力向量、$X/X^*$、空间叉乘、空间惯量、运动方程、关节模型 $S_i$、递推 FK 算法 | [于]§5.1–5.2 | 理论主体；`algorithm` 盒 |
| §9 | **全身控制雅可比**：刚体雅可比 ${}_bJ_i$、足底系投影、$J_1\dots J_4$、$\dot J\dot q$ | [于]§5.2.5 | `algorithm`，衔接 WBC 章 |
| §10 | **奇异性与零空间理论（WBC 数学地基）**：冗余、零空间投影、MP 伪逆、拉格朗日证明、阻尼最小二乘 | [笔记]332–458 | 理论高潮；`theorem`+`proof`+`insight` |
| §11 | **四足整机运动学**：姿态↔足端位置、牵连速度+关节速度 | [宇树]§6.1 | 整合 |
| §12 | **陷阱与练习** | 全源 | `pitfall`/习题 |

---

## B. 思路主线与作者洞见（[笔记] 承载；成章用 `insight` 盒，可标"作者洞见"）

> 个人笔记是把三源融为一体的骨架。以下洞见点贯穿全章。

- **〔洞见 K1〕坐标系即一切**（[笔记]1223）："有运动学必先建立坐标系"。四足坐标系按 Body/Joint/Link 分层，机身系 $xyz$ = 前/左/上。作主线开篇。
- **〔洞见 K2〕旋转矩阵三用途**（[笔记]136）：旋转矩阵有且仅有三大用途——**描述姿态、向量坐标变换、向量旋转**；变换矩阵亦有"坐标系映射"与"对向量操作（移动/转动）"两重作用，且对向量操作时移动/转动顺序影响结果，对坐标系变换则不然。这是统一三源记号的认知锚点。
- **〔洞见 K3〕欧拉角只配人机交互**（[笔记]112）：欧拉角直观但有奇异性（万向锁），"不适用于机器人姿态的长期描述……会丢失自由度且无法插值迭代"，长期描述用四元数或旋转矩阵。→ §1 反面教材。
- **〔洞见 K4〕FK 易、IK 难**（[笔记]206,208）：正运动学由因推果、唯一解、计算直接；逆运动学由果推因，存在**多解/无解/奇异解**，需可达工作空间（Reachable）与灵巧工作空间（Dexterous）判断先验可解性。
- **〔洞见 K5〕一阶微分把非线性变线性**（[笔记]288,320 / [宇树]2881）：FK 中足端位置与关节角是**非线性**关系不能写成矩阵乘；而**速度层面是线性**的 $\dot p=J\dot q$，故可用矩阵求逆实现正逆切换。"线性关系是非常有用的属性"。这是全章方法论核心。
- **〔洞见 K6〕雅可比的运动学/静力学对偶**（[笔记]322 / [宇树]2885）：静止时整条腿总功率为 0（流入功率=流出功率），由虚功/功率守恒直接得 $\tau=J^TF$——同一个 $J$ 同时贯通速度映射与力映射。
- **〔洞见 K7〕零空间 = 冗余的"免费内部运动"**（[笔记]358,366）：通解 $\dot q=\dot q_p+\dot q_0$，$\dot q_0\in N(J)$ 是"关节动而末端不动"的内部运动，是 WBC 多任务优先级的本质。**本章把它讲透是为后续 WBC 章铺垫。**
- **〔洞见 K8〕MP 伪逆 = 能量最优**（[笔记]394）：无穷多右逆中选 MP 伪逆，物理上对应"最小关节速度模长"（最节能/最平滑），由拉格朗日乘子法严格导出，是全身控制"真正基石"。
- **〔洞见 K9〕奇异点要阻尼**（[笔记]380）：近奇异时 $JJ^T$ 理论可逆但行列式趋零成病态，求逆数值溢出，用阻尼最小二乘 $J^+=J^T(JJ^T+\lambda^2\mathbb I)^{-1}$ 保证始终可逆。→ §1 反面 + §10 陷阱。
- **〔洞见 K10〕浮动基座 = 虚拟 6-DoF 关节**（[于]2561）：固定基座（编号 0，虚拟）→ 浮动基座（机身，编号 1）之间用一个**虚拟 6 自由度关节**连接，$q_1=[\Theta^T\ {}^{\mathcal O}p_{com}^T]^T$，$S_1=\mathbb I_{6\times6}$。这是"浮动基座"与定基机械臂的唯一结构差别，全章统一记号的关键。
- **〔洞见 K11〕机身系是"瞬时重合的静止惯性系"**（[宇树]3010,3093）：机身系 $\{b\}$ **不**随机身运动，只是当前时刻与机身位姿重合——否则关节静止时足端在该系速度恒为 0，无法定义牵连速度。这是理解整机足端速度公式的关键认知。
- **〔洞见 K12〕电机零点≠模型零点**（[笔记]1243）：$\theta_{model}=\theta_{raw}+\theta_{offset}$ 连接实物与模型。→ 实践盒（`practice`）。

---

## C. 逐条关键公式收录（LaTeX · 来由 · 推导要点 · 出处）

> 编号 [Kxx] 为本种子内部编号；括号内为源式号。统一记号已按本书约定处理，换算见 D 节。

### C-1 坐标系与旋转矩阵（[于]§2.2 / [宇树]§4）

**[K01] 旋转矩阵定义（坐标映射）** — 源 [于](2.9) / [宇树](4.5,4.8)
$$
{}^{\mathcal A}\mathbf r_{AP}=\big[\,{}^{\mathcal A}\mathbf e_x^{\mathcal B}\ \ {}^{\mathcal A}\mathbf e_y^{\mathcal B}\ \ {}^{\mathcal A}\mathbf e_z^{\mathcal B}\,\big]\,{}^{\mathcal B}\mathbf r_{AP}={}^{\mathcal A}R_{\mathcal B}\,{}^{\mathcal B}\mathbf r_{AP}
$$
来由：把 $\{\mathcal B\}$ 的基向量在 $\{\mathcal A\}$ 中的坐标列成列向量。宇树以 $R_{sb}=[\hat x_b\ \hat y_b\ \hat z_b]$ 表述（"列即 b 系坐标轴在 s 系的坐标"），并给"下标消去原则" $R_{sa}R_{ab}=R_{sb}$（源(4.19,4.20)）。

**[K02] 正交性与逆=转置** — 源 [于](2.10,2.11)/[宇树](4.11,4.14)
$$
{}^{\mathcal A}R_{\mathcal B}^{T}\,{}^{\mathcal A}R_{\mathcal B}=\mathbb I_3,\qquad
{}^{\mathcal B}R_{\mathcal A}={}^{\mathcal A}R_{\mathcal B}^{-1}={}^{\mathcal A}R_{\mathcal B}^{T},\qquad \det R=1\ (\text{右手系})
$$
宇树洞见（源2662）：计算机求逆耗时易错，旋转矩阵务必用转置。

**[K03] 基本旋转矩阵** — 源 [于](2.12)/[宇树](4.9,4.10)
$$
R_x(\varphi)=\begin{bmatrix}1&0&0\\0&c_\varphi&-s_\varphi\\0&s_\varphi&c_\varphi\end{bmatrix},\
R_y(\varphi)=\begin{bmatrix}c_\varphi&0&s_\varphi\\0&1&0\\-s_\varphi&0&c_\varphi\end{bmatrix},\
R_z(\varphi)=\begin{bmatrix}c_\varphi&-s_\varphi&0\\s_\varphi&c_\varphi&0\\0&0&1\end{bmatrix}
$$
（$c_\varphi=\cos\varphi,\ s_\varphi=\sin\varphi$）

**[K04] 连续旋转左乘** — 源 [于](2.16)/[宇树](4.19,4.22,4.23)
$$
{}^{\mathcal A}R_{\mathcal C}={}^{\mathcal A}R_{\mathcal B}\,{}^{\mathcal B}R_{\mathcal C}
$$
满足结合律、不满足交换律。[笔记]136 补：绕固定轴转动→左乘新矩阵；绕动轴转动→右乘。

**[K05] 罗德里格斯公式（轴角→旋转矩阵）** — 源 [笔记]164–172
$$
\mathbf R=\mathbf I+(\sin\theta)\,\hat{\mathbf u}_\times+(1-\cos\theta)\,\hat{\mathbf u}_\times^2,\quad
\hat{\mathbf u}_\times=\begin{bmatrix}0&-u_z&u_y\\u_z&0&-u_x\\-u_y&u_x&0\end{bmatrix}
$$
绕单位轴 $\hat u$ 逆时针转 $\theta$。宇树以**指数坐标** $R=e^{[\hat u]_\times\theta}$ 表述（源§4.3，矩阵指数/对数），本书以 `\cref{ch:lie}` 衔接，此处给闭式。

**[K06] ZYX 欧拉角→旋转矩阵** — 源 [于](2.17,2.19,2.20)
$$
\boldsymbol\Theta=[\phi\ \theta\ \psi]^T\ (\text{横滚/俯仰/偏航}),\qquad
R(\boldsymbol\Theta)=R_z(\psi)R_y(\theta)R_x(\phi)
$$
展开（源(2.20)）：
$$
R=\begin{bmatrix}
c_\theta c_\psi & c_\psi s_\phi s_\theta-c_\phi s_\psi & s_\phi s_\psi+c_\phi c_\psi s_\theta\\
c_\theta s_\psi & c_\phi c_\psi+s_\phi s_\theta s_\psi & c_\phi s_\theta s_\psi-c_\psi s_\phi\\
-s_\theta & c_\theta s_\phi & c_\phi c_\theta
\end{bmatrix}
$$

**[K07] 旋转矩阵→ZYX 欧拉角（逆解）** — 源 [于](2.21,2.22)
$$
\boldsymbol\Theta=\begin{bmatrix}\phi\\\theta\\\psi\end{bmatrix}
=\begin{bmatrix}
\operatorname{atan}(c_{32}/c_{33})\\
-\operatorname{atan}\!\big(c_{31}/\sqrt{c_{32}^2+c_{33}^2}\big)\\
\operatorname{atan}(c_{21}/c_{11})
\end{bmatrix},\quad R=[c_{ij}]
$$
源注（2629）：逆解不唯一；分母极小/为零（即 $\theta\to\pm90°$）需特殊处理 → **万向锁**，对应 §1 反面。

**[K08] 齐次变换矩阵** — 源 [笔记]176/[宇树]§4.5
$$
T=\begin{bmatrix}R&\mathbf t\\\mathbf 0^T&1\end{bmatrix}\in SE(3),\quad
T^{-1}=\begin{bmatrix}R^T&-R^T\mathbf t\\\mathbf 0^T&1\end{bmatrix}
$$
向量需齐次扩展（末位补 1）；可像旋转矩阵一样连乘。**本书约定 se(3) 平移在前**，李代数坐标 $\xi=[\rho^T\ \phi^T]^T$，此处 $T$ 本身的块结构不变，仅 `note` 盒标 twist 排序换算。

### C-2 角速度张量（[于]§2.2.3(2) — 本章严格推导亮点）

**[K09] 反对称算子** — 源 [于](2.27)/[宇树](4.32)
$$
\boldsymbol a_\times=[\,a_1\ a_2\ a_3\,]^T_\times=\begin{bmatrix}0&-a_3&a_2\\a_3&0&-a_1\\-a_2&a_1&0\end{bmatrix},\qquad \boldsymbol a\times\boldsymbol b=\boldsymbol a_\times\boldsymbol b
$$

**[K10] 角速度张量定义与反对称性证明** — 源 [于](2.23,2.24,2.25)
对 ${}^{\mathcal A}\mathbf r_{AP}={}^{\mathcal A}R_{\mathcal B}\,{}^{\mathcal B}\mathbf r_{AP}$ 求导（$P$ 刚接刚体，${}^{\mathcal B}\mathbf r_{AP}$ 为常数）：
$$
{}^{\mathcal A}\dot{\mathbf r}_{AP}={}^{\mathcal A}\dot R_{\mathcal B}\,{}^{\mathcal B}\mathbf r_{AP}
=\underbrace{{}^{\mathcal A}\dot R_{\mathcal B}\,{}^{\mathcal A}R_{\mathcal B}^{T}}_{{}^{\mathcal A}\Omega_{\mathcal B}}\,{}^{\mathcal A}\mathbf r_{AP}
$$
由 $\frac{d}{dt}(R R^T)=\dot R R^T+R\dot R^T=\frac{d}{dt}\mathbb I=0$ 得 ${}^{\mathcal A}\Omega_{\mathcal B}=-{}^{\mathcal A}\Omega_{\mathcal B}^T$（反对称），故
$$
{}^{\mathcal A}\Omega_{\mathcal B}=\begin{bmatrix}0&-\omega_z&\omega_y\\\omega_z&0&-\omega_x\\-\omega_y&\omega_x&0\end{bmatrix}={}^{\mathcal A}\boldsymbol\omega_{\mathcal{AB}\times},\qquad
\dot R=\boldsymbol\omega_\times R
$$
**本书右扰动主线**：$\dot R=R\,{}^{\mathcal B}\boldsymbol\omega_\times$（机体系角速度，源(2.32) ${}^{\mathcal B}\boldsymbol\omega_{\mathcal{AB}\times}={}^{\mathcal B}R_{\mathcal A}{}^{\mathcal A}\dot R_{\mathcal B}$）。`note` 盒标世界系/机体系换算。

### C-3 单腿正运动学（[宇树]§5.1）

**[K11] 2D 二连杆 FK（引子）** — 源 [宇树](5.1)
$$
x_P=l_1\cos\theta_1+l_2\cos(\theta_1+\theta_2),\quad y_P=l_1\sin\theta_1+l_2\sin(\theta_1+\theta_2)
$$

**[K12] 三维单腿齐次变换** — 源 [宇树](5.6,5.7,5.8)
机身关节绕 $x$、大腿/小腿关节绕 $y$；坐标系 $\{0\}\{1\}\{2\}$ 原点重合（简化）：
$$
T_{01}=\begin{bmatrix}1&0&0&0\\0&c_1&-s_1&0\\0&s_1&c_1&0\\0&0&0&1\end{bmatrix},\quad
T_{12}=\begin{bmatrix}c_2&0&s_2&0\\0&1&0&0\\-s_2&0&c_2&0\\0&0&0&1\end{bmatrix},\quad
T_{23}=\begin{bmatrix}c_3&0&s_3&0\\0&1&0&l_1\\-s_3&0&c_3&l_2\\0&0&0&1\end{bmatrix}
$$
其中 $l_1=\mp l_{abad}$（右/左腿）、$l_2=-l_{hip}$（源(5.9)）；足端 $p_3=[0,0,l_3]^T,\ l_3=-l_{knee}$（源(5.10)）。

**[K13] 单腿 FK 闭式** — 源 [宇树](5.11)
$$
\begin{bmatrix}x_P\\y_P\\z_P\end{bmatrix}=
\begin{bmatrix}
l_3\sin(\theta_2+\theta_3)+l_2\sin\theta_2\\
-l_3\sin\theta_1\cos(\theta_2+\theta_3)+l_1\cos\theta_1-l_2\cos\theta_2\sin\theta_1\\
l_3\cos\theta_1\cos(\theta_2+\theta_3)+l_1\sin\theta_1+l_2\cos\theta_1\cos\theta_2
\end{bmatrix}
$$
来由：$[p_0;1]=T_{01}T_{12}T_{23}[p_3;1]$。**数值例可由此装配**（如 A1 腿长代入）。

### C-4 单腿逆运动学（[宇树]§5.2 — 解耦三步）

**[K14] 机身关节 $\theta_1$（yz 投影 + atan2）** — 源 [宇树](5.12)–(5.19)
$$
\theta_1=\operatorname{atan2}(z_P l_1+y_P L,\ y_P l_1-z_P L),\qquad L=\sqrt{y_P^2+z_P^2-l_1^2}
$$
来由：将腿投影到 yz 平面，$[y_P;z_P]=R(\theta_1)[l_1;-L]$，两式相除消 $\cos\theta_1$ 得 $\tan\theta_1$。$\theta_1$ 理论两解，另一解超 A1 限位故舍（源2717 → `pitfall`）。

**[K15] 小腿关节 $\theta_3$（余弦定理）** — 源 [宇树](5.23,5.24)
$$
\theta_3=-\pi+\arccos\!\Big(\tfrac{|\overrightarrow{O_3A}|^2+|\overrightarrow{O_3P}|^2-|\overrightarrow{AP}|^2}{2|\overrightarrow{O_3A}||\overrightarrow{O_3P}|}\Big),\quad
|\overrightarrow{AP}|=\sqrt{x_P^2+y_P^2+z_P^2-l_{abad}^2}
$$
$|\overrightarrow{O_3A}|=l_{hip},\ |\overrightarrow{O_3P}|=l_{knee}$。先算 $\theta_3$（不依赖 $\theta_2$）。

**[K16] 大腿关节 $\theta_2$（整理 tan 分式 + atan2）** — 源 [宇树](5.34)–(5.37)
$$
\theta_2=\operatorname{atan2}(a_1 m_1+a_2 m_2,\ a_2 m_1-a_1 m_2),\quad
\begin{cases}a_1=y_P\sin\theta_1-z_P\cos\theta_1\\a_2=x_P\\ m_1=l_3\sin\theta_3\\ m_2=l_3\cos\theta_3+l_2\end{cases}
$$
来由：FK 式中仅 $\theta_2$ 未知，由 $x_P$ 式与 $(y_P\sin\theta_1-z_P\cos\theta_1)$ 式相除得 $\tan\theta_2$。源证 $a_1,a_2$ 均非零（5.31–5.33）。

### C-5 单腿一阶微分运动学与静力学（[宇树]§5.3 / [笔记])

**[K17] 足端雅可比（FK 直接求导）** — 源 [宇树](5.39)–(5.42) / [笔记](290–318)
$$
\dot{\mathbf p}=\begin{bmatrix}\dot x_P\\\dot y_P\\\dot z_P\end{bmatrix}
=\frac{\partial\mathbf f}{\partial\mathbf q}\dot{\mathbf q}=J(\mathbf q)\,\dot{\mathbf q},\qquad J=\frac{\partial\mathbf f}{\partial\mathbf q}\ (3\times3)
$$
分量（源5.39 例）：$\dot x_P=0\cdot\dot\theta_1+(l_3\cos(\theta_2+\theta_3)+l_2\cos\theta_2)\dot\theta_2+l_3\cos(\theta_2+\theta_3)\dot\theta_3$，故 $J_{11}=0,\ J_{12}=l_3\cos(\theta_2+\theta_3)+l_2\cos\theta_2,\ J_{13}=l_3\cos(\theta_2+\theta_3)$。A1/Go1 限位下 $J$ 可逆，故 $\dot{\mathbf q}=J^{-1}\dot{\mathbf p}$（源5.43）。

**[K18] 单腿静力学（功率守恒 → 对偶）** — 源 [宇树](5.44)–(5.47)/[笔记](322–328)
$$
\tau_1\dot\theta_1+\tau_2\dot\theta_2+\tau_3\dot\theta_3=F_x\dot x_P+F_y\dot y_P+F_z\dot z_P
\ \Rightarrow\ \boldsymbol\tau^T\dot{\mathbf q}=\mathbf F^T J\dot{\mathbf q}\ \Rightarrow\
\boxed{\boldsymbol\tau=J^T\mathbf F}
$$
正逆：$\boldsymbol\tau=J^T\mathbf F$，$\mathbf F=J^{-T}\boldsymbol\tau$。注（源2913）：$\mathbf F$ 为足端**对地**力；地对足反力 $\mathbf F'$ 时 $\boldsymbol\tau=-J^T\mathbf F'$。

**[K19] 摆动腿修正力（笛卡尔 PD）** — 源 [宇树](5.48)
$$
\mathbf f_d=K_p(\mathbf p_{0d}-\mathbf p_{0f})+K_d(\dot{\mathbf p}_{0d}-\dot{\mathbf p}_{0f}),\qquad \boldsymbol\tau=J^T\mathbf f_d
$$
$K_p,K_d$ 正定对角。→ `practice` 盒（工程，凝练不全推）。

### C-6 浮动基座、运动学树与 Plücker 空间向量（[于]§5.1–5.2 — 本章理论主体）

**[K20] 六维空间速度（运动向量 $\mathbf M^6$）** — 源 [于](5.5)
$$
\mathbf v=[\,\omega_x\ \omega_y\ \omega_z\ v_{Ox}\ v_{Oy}\ v_{Oz}\,]^T=[\,\boldsymbol\omega^T\ \mathbf v_O^T\,]^T
$$
来由：刚体速度场由纯转动 $\boldsymbol\omega$ + 过 $O$ 点纯平动 $\mathbf v_O$ 完整描述（Plücker 坐标）。**本书 se(3) 平移在前约定**：本书 twist 写 $[\mathbf v_O^T\ \boldsymbol\omega^T]^T$（平移在前），于宪元为角速度在前；`note` 盒标"换算：交换上下 3 块 → 对应 $X$ 矩阵分块转置"。

**[K21] 六维空间力（动力向量 $\mathbf F^6$）** — 源 [于](5.10)
$$
\hat{\mathbf f}=[\,n_{Ox}\ n_{Oy}\ n_{Oz}\ f_x\ f_y\ f_z\,]^T=[\,\mathbf n_O^T\ \mathbf f^T\,]^T,\qquad \mathbf n_P=\mathbf n_O+\mathbf f\times\overrightarrow{OP}
$$

**[K22] Plücker 坐标变换（运动/力对偶）** — 源 [于](5.11,5.14,5.22)
$$
{}^{\mathcal B}\mathbf m={}^{\mathcal B}X_{\mathcal A}\,{}^{\mathcal A}\mathbf m,\qquad
{}^{\mathcal B}\mathbf f={}^{\mathcal B}X_{\mathcal A}^{*}\,{}^{\mathcal A}\mathbf f,\qquad
{}^{\mathcal B}X_{\mathcal A}^{*}={}^{\mathcal B}X_{\mathcal A}^{-T}
$$
来由：保功率不变 $\mathbf m^T\mathbf f$ 在两系相等（源5.13）。组合变换（$E={}^{\mathcal B}R_{\mathcal A}$，$\mathbf r=\overrightarrow{AB}$ 在 $\mathcal A$ 系）：
$$
{}^{\mathcal B}X_{\mathcal A}=\begin{bmatrix}E&\mathbf 0\\-E\,\mathbf r_\times&E\end{bmatrix},\qquad
{}^{\mathcal B}X_{\mathcal A}^{*}=\begin{bmatrix}E&-E\,\mathbf r_\times\\\mathbf 0&E\end{bmatrix}
$$
分解：纯旋转 $\operatorname{diag}(E,E)$；纯平移 $\begin{bmatrix}\mathbb I&\mathbf 0\\-\mathbf r_\times&\mathbb I\end{bmatrix}$（源5.16,5.20）。

**[K23] 空间叉乘（运动/力两套规则）** — 源 [于](5.27,5.28)
$$
\mathbf v\times=\begin{bmatrix}\boldsymbol\omega_\times&\mathbf 0\\\mathbf v_{O\times}&\boldsymbol\omega_\times\end{bmatrix},\qquad
\mathbf v\times^{*}=\begin{bmatrix}\boldsymbol\omega_\times&\mathbf v_{O\times}\\\mathbf 0&\boldsymbol\omega_\times\end{bmatrix}
$$
来由：常数空间向量随坐标系以速度 $\mathbf v$ 运动时 ${}^{\mathcal O}\dot{\mathbf m}=\mathbf v\times{}^{\mathcal O}\mathbf m$，${}^{\mathcal O}\dot{\mathbf f}=\mathbf v\times^{*}{}^{\mathcal O}\mathbf f$；72 个单位微分排列组合而得（源5.24–5.26）。

**[K24] 空间加速度** — 源 [于](5.29,5.32)
$$
\mathbf a=\frac{d\mathbf v}{dt}=\begin{bmatrix}\dot{\boldsymbol\omega}\\\dot{\mathbf v}_O\end{bmatrix}
=\begin{bmatrix}\dot{\boldsymbol\omega}\\\ddot{\mathbf r}-\boldsymbol\omega\times\dot{\mathbf r}\end{bmatrix}
$$
要点（源5.30,5.31）：$O$ 是不动点，$t,t+\delta t$ 与之重合的刚体点不同，故 $\dot{\mathbf v}_O=\ddot{\mathbf r}-\boldsymbol\omega\times\dot{\mathbf r}$（区别于传统加速度）。

**[K25] 空间惯量** — 源 [于](5.38,5.40)
$$
\hat I_C=\begin{bmatrix}\bar I_C&\mathbf 0\\\mathbf 0&m\mathbb I\end{bmatrix},\qquad
\hat I_O=\begin{bmatrix}\bar I_C+m\,\mathbf c_\times\mathbf c_\times^T&m\,\mathbf c_\times\\ m\,\mathbf c_\times^T&m\mathbb I\end{bmatrix}\ (\mathbf c=\overrightarrow{OC})
$$
$\hat{\mathbf h}=\hat I\hat{\mathbf v}$（空间动量，源5.36）。

**[K26] 运动方程（牛顿-欧拉合一）** — 源 [于](5.41)
$$
\mathbf f=\frac{d}{dt}(I\mathbf v)=I\mathbf a+\mathbf v\times^{*}I\mathbf v
$$
注：此式含陀螺项 $\mathbf v\times^{*}I\mathbf v$；与单刚体 MPC 章（tag=单刚体）忽略陀螺项 $\dot L\approx\mathbf I\dot{\boldsymbol\omega}$ 形成对照，`insight` 盒可交叉引用。

**[K27] 运动学树父节点集** — 源 [于](5.42,5.43)
$$
\lambda=\{\lambda(1),\dots,\lambda(N)\},\quad
\lambda=\{0,1,2,3,1,5,6,1,8,9,1,11,12\}\ (N=13)
$$
固定基座=0（虚拟）；浮动基座=1；腿序右前/左前/右后/左后。

**[K28] 关节坐标变换与关节运动子空间** — 源 [于](5.44,5.45,5.47,5.49,5.52,5.54)
$$
{}^iX_{\lambda(i)}=X_J(i)X_T(i),\qquad \mathbf v_{J,i}=S_i\dot{\mathbf q}_i
$$
浮动基座虚拟 6-DoF 关节（源5.44,5.54）：
$$
X_J(1)=\begin{bmatrix}{}^{\mathcal B}R_{\mathcal O}&\mathbf 0\\-{}^{\mathcal B}R_{\mathcal O}\,{}^{\mathcal O}\mathbf p_{com\times}&{}^{\mathcal B}R_{\mathcal O}\end{bmatrix},\quad
{}^1S_1=\mathbb I_{6\times6}
$$
单自由度绕 $x$ 关节 $X_J(i)=\operatorname{diag}(R_x^T(q_i),R_x^T(q_i))$（绕 $y$ 同理，源5.45,5.46）；${}^iS_i=[1\,0\,0\,0\,0\,0]^T$（绕 $x$，源5.52）。

**[K29] 递推正运动学算法** — 源 [于](5.50,5.55,5.58)
速度/加速度递推：
$$
\mathbf v_i=\mathbf v_{\lambda(i)}+S_i\dot{\mathbf q}_i,\qquad
\mathbf a_i=\mathbf a_{\lambda(i)}+S_i\ddot{\mathbf q}_i+\mathbf v_i\times S_i\dot{\mathbf q}_i
$$
算法（`algorithm` 盒，源5.58）：
```
v0=0, a0=0, ^0X_0 = I6
for i = 1 to N:
    ^iX_{λ(i)} = X_J(i) X_T(i)
    ^iX_0      = ^iX_{λ(i)} · ^{λ(i)}X_0
    ^iv_{J,i}  = ^iS_i · q̇_i
    ^iv_i      = ^iX_{λ(i)} · ^{λ(i)}v_{λ(i)} + ^iv_{J,i}
    ^ia_i      = ^iX_{λ(i)} · ^{λ(i)}a_{λ(i)} + ^iS_i·q̈_i + ^iv_i × ^iS_i·q̇_i
```

**[K30] 足底接触点变换** — 源 [于](5.56,5.57)
$$
\{\rho(1),\rho(2),\rho(3),\rho(4)\}=\{4,7,10,13\},\quad
X_C(i)=\begin{bmatrix}\mathbb I_3&\mathbf 0\\-([0,0,-l_3]^T)_\times&\mathbb I_3\end{bmatrix}
$$

### C-7 全身控制雅可比（[于]§5.2.5 — 衔接 WBC）

**[K31] 刚体雅可比** — 源 [于](5.59,5.61,5.62)
$$
\mathbf v_i=\sum_{j\in\kappa(i)}S_j\dot{\mathbf q}_j=\underbrace{[\epsilon_{i1}S_1\ \cdots\ \epsilon_{iN}S_N]}_{{}_bJ_i}\dot{\mathbf q},\quad
\epsilon_{ij}=\begin{cases}1&j\in\kappa(i)\\0&\text{else}\end{cases}
$$
$\kappa(i)$ = 支撑 $i$ 的节点集。伪代码沿运动学树回溯装配（源5.63）。

**[K32] 足底系投影与任务雅可比** — 源 [于](5.64,5.65,5.66,5.67)
$$
X_Q(i)=\operatorname{diag}({}^{\rho(i)}R_0^T,{}^{\rho(i)}R_0^T),\qquad
{}_b^{\mathcal Q}J_i=X_Q(i)X_C(i)\,{}_bJ_{\rho(i)}
$$
取后 3 行（平动）${}_b^{\mathcal Q}J_i'$；$J_1$=支撑足按列堆叠、$J_4$=摆动足按列堆叠。

**[K33] $\dot J\dot q$ 项（VP 加速度）** — 源 [于](5.68,5.69,5.70,5.74)
$$
\ddot{\mathbf x}=J\ddot{\mathbf q}+\dot J\dot{\mathbf q},\quad
\mathbf a_i^{vp}=\mathbf a_{\lambda(i)}^{vp}+\mathbf v_i\times S_i\dot{\mathbf q}_i,\quad \mathbf a_1^{vp}=\mathbf 0
$$
足底世界系加速度 ${}^{\mathcal O}\mathbf a_{C,i}=\mathbf a_{\rho(i)}^{C,i}+\boldsymbol\omega_{\rho(i)}\times\mathbf v_{\rho(i)}^{C,i}$；任务 2/3 的 $\dot J\dot q=\mathbf 0$（源5.71）。

**[K34] 浮动基座动力学标准形（衔接预告）** — 源 [于](5.76)
$$
M(\mathbf q)\ddot{\mathbf q}+C(\mathbf q,\dot{\mathbf q})=S_j\boldsymbol\tau+J_c^T\,{}^{\mathcal O}\mathbf f_c
$$
→ 留给动力学/WBC 章，本章只引出。

### C-8 奇异性与零空间理论（[笔记]332–458 — 本章理论高潮）

**[K35] 一阶微分运动学方程与冗余** — 源 [笔记](334–360)
$$
\dot{\mathbf x}=J\dot{\mathbf q},\qquad N(J)=\{\mathbf w\in\mathbb R^N\mid J\mathbf w=\mathbf 0\}
$$
冗余：关节数 $N$ > 任务维 $M$（如 7-DoF 抓 6-DoF 目标）。$J\cdot N(J)=0$。

**[K36] 通解结构** — 源 [笔记](362–384)
$$
\dot{\mathbf q}=\underbrace{J^{+}\dot{\mathbf x}}_{\dot{\mathbf q}_p\ \text{特解}}+\underbrace{N\dot{\mathbf q}_{\forall}}_{\dot{\mathbf q}_0\in N(J)\ \text{内部运动}},\qquad
J^{+}=J^T(JJ^T)^{-1},\quad N(J)=\mathbb I-J^{+}J
$$
来由：$0=J-JJ^{+}J=J(\mathbb I-J^{+}J)$。$\dot{\mathbf q}_0$ 满足 $J\dot{\mathbf q}_0=0$（末端静止）。

**[K37] ⭐ MP 伪逆 = 最小范数解（拉格朗日乘子法证明）** — 源 [笔记](394–435)
目标 $\min_{\dot{\mathbf q}}\frac12\|\dot{\mathbf q}\|^2$ s.t. $J\dot{\mathbf q}=\dot{\mathbf x}$。构造
$$
\mathcal L(\dot{\mathbf q},\boldsymbol\lambda)=\tfrac12\dot{\mathbf q}^T\dot{\mathbf q}-\boldsymbol\lambda^T(J\dot{\mathbf q}-\dot{\mathbf x})
$$
平稳点：
$$
\nabla_{\dot{\mathbf q}}\mathcal L=\dot{\mathbf q}-J^T\boldsymbol\lambda=0\Rightarrow\dot{\mathbf q}=J^T\boldsymbol\lambda;\qquad
\nabla_{\boldsymbol\lambda}\mathcal L=-(J\dot{\mathbf q}-\dot{\mathbf x})=0\Rightarrow J\dot{\mathbf q}=\dot{\mathbf x}
$$
代入：$JJ^T\boldsymbol\lambda=\dot{\mathbf x}\Rightarrow\boldsymbol\lambda=(JJ^T)^{-1}\dot{\mathbf x}$（非奇异时 $JJ^T$ 可逆），故
$$
\dot{\mathbf q}=J^T(JJ^T)^{-1}\dot{\mathbf x}=J^{+}\dot{\mathbf x}
$$
即 MP 伪逆给出唯一最小范数解，位于 $J$ 行空间（与零空间正交）→ WBC "真正基石"（`proof` 盒 + `insight` K8）。

**[K38] 阻尼最小二乘（近奇异稳健化）** — 源 [笔记](380)
$$
J^{+}=J^T(JJ^T+\lambda^2\mathbb I)^{-1}
$$
近奇异时 $\det(JJ^T)\to0$ 病态；加阻尼保证始终可逆。→ §1 反面 + §12 陷阱。

**[K39] 秩-零度定理（冗余维数）** — 源 [笔记](437)
输入空间维 = 值域维 + 零空间维：$N=\operatorname{rank}(J)+\dim N(J)$。非奇异下 $\dim N(J)=N-M>0$，故必存在可用内部运动。

**[K40] 零空间投影矩阵唯一性** — 源 [笔记](441–457)
$$
\dot{\mathbf q}_0=N\dot{\mathbf q}_{\forall},\quad J N=0\ \Rightarrow\ N=\mathbb I-J^{+}J
$$
要点：$N$ 是把任意 $\dot{\mathbf q}_{\forall}$ 正交投影到 $N(J)$ 的唯一正交投影矩阵（误差与零空间正交）——"在完美执行主任务前提下尽力近似次要任务"。

### C-9 四足整机运动学（[宇树]§6.1）

**[K41] 姿态↔足端位置（不滑移约束）** — 源 [宇树](6.1,6.3,6.4)
$$
\mathbf p_{si}=\mathbf p_{bi}(0)-\mathbf p_{b0}(0),\quad
T_{sb}=\begin{bmatrix}R_z(\alpha)R_y(\beta)R_x(\gamma)&-\mathbf p_{b0}(0)+\mathbf p_d\\\mathbf 0&1\end{bmatrix},\quad
\mathbf p_{bi}=T_{sb}^{-1}\mathbf p_{si}
$$
机身位姿变 → 各足端在 $\{s\}$ 系不动（常量）→ 反解机身系足端坐标 → 单腿 IK 得关节角。

**[K42] 刚体上一点速度/加速度** — 源 [宇树](6.8,6.9)
$$
\dot{\mathbf p}_P=\mathbf v_b+[\boldsymbol\omega_b]_\times\mathbf p_P,\qquad
\ddot{\mathbf p}_P=\dot{\mathbf v}_b+[\dot{\boldsymbol\omega}_b]_\times\mathbf p_P+[\boldsymbol\omega_b]_\times[\boldsymbol\omega_b]_\times\mathbf p_P
$$
（$\{b\}$ 为瞬时重合的静止惯性系，洞见 K11）

**[K43] 整机足端速度（牵连+关节）** — 源 [宇树](6.10,6.11,6.12,6.13)
$$
\mathbf v_{be}=\mathbf v_b+[\boldsymbol\omega_b]_\times\mathbf p_{bfB},\quad \mathbf v_{bj}=J\dot{\boldsymbol\theta}
$$
$$
\mathbf v_{sf}=\mathbf v_s+R_{sb}([\boldsymbol\omega_b]_\times\mathbf p_{bfB}+J\dot{\boldsymbol\theta}),\qquad
\mathbf v_{sfB}=\mathbf v_{sf}-\mathbf v_s=R_{sb}([\boldsymbol\omega_b]_\times\mathbf p_b+J\dot{\boldsymbol\theta})
$$
$\mathbf v_s$ 不可直接测，实用量为足端相对机身速度 $\mathbf v_{sfB}$（衔接状态估计章）。

---

## D. 记号统一（本书全局约定 → 三源换算，成章 `note` 盒）

| 主题 | 本书约定 | [于] | [宇树] | [笔记] | 换算 note |
|---|---|---|---|---|---|
| 旋转表示 | Hamilton 四元数 + 旋转矩阵主线 | 旋转矩阵/ZYX 欧拉角 | 旋转矩阵/指数坐标/欧拉角 | 旋转矩阵/欧拉角/罗德里格斯 | 欧拉角仅人机交互 |
| 旋转矩阵下标 | ${}^{\mathcal A}R_{\mathcal B}$（A 系看 B 系） | 同 | $R_{sb}$（s 系看 b 系） | 混用 | 统一为左上下标式 |
| $\dot R$ | **右扰动** $\dot R=R\,{}^{\mathcal B}\boldsymbol\omega_\times$ | 给世界系 $\dot R=\Omega R$ 与机体系两式 | — | $\dot R=\omega_\times R$（世界系） | 标世界/机体系换算 |
| twist 排序 | **se(3) 平移在前** $[\rho^T,\phi^T]^T$ | 角速度在前 $[\omega^T,v_O^T]^T$ | — | — | $X$ 矩阵分块需对应交换 |
| 反对称算子 | $[\cdot]_\times$ / $\cdot_\times$ | $\cdot_\times$ | $[\cdot]_\times$ | $\hat u_\times$/$[\cdot]_\times$ | 统一 $[\cdot]_\times$ |
| 欧拉角符号 | $\phi,\theta,\psi$=roll,pitch,yaw | $\phi,\theta,\psi$ | $\gamma,\beta,\alpha$=roll,pitch,yaw | $\phi,\theta,\psi$ | 宇树 $\alpha\!=\!\psi$ 等需对齐 |
| 关节角 | $q_i$ | $q_i$ | $\theta_i$ | $q$/$\theta$ | 统一 $q_i$ |
| 浮动基座编号 | 机身=1，虚拟固定基=0 | 同 | s/b 系（无树编号） | Body 系 | 以于宪元树编号为准 |

---

## E. OCR 可疑处清单（原印 vs 推断订正，成章 `note` 标"OCR 订正"）

1. **[于](5.39 旁)/[笔记]527 类比**：MinerU 矩阵中大量 `\overrightarrow{m_A - AB}` 错位（如源2329 ${}^B\hat{\mathbf m}$ 第二块写成 `\overrightarrow{\mathbf m_A - AB}\times\mathbf m`），应为 $\mathbf m_B=\mathbf m_A-\overrightarrow{AB}\times\mathbf m$。订正见 [K22]。
2. **[于](5.41)**：原印 `f = I a + (v×* I − I v×)v = I a + v×* I v`。中间步 $(\mathbf v\times^{*}I-I\mathbf v\times)\mathbf v$ 的第二项 $I\mathbf v\times\mathbf v$ 因 $\mathbf v\times\mathbf v$ 涉空间叉乘自身需谨慎；最终 $\mathbf f=I\mathbf a+\mathbf v\times^{*}I\mathbf v$ 是 Featherstone 标准式，订正采纳标准式 [K26]。
3. **[宇树](5.39)**：原印 `ẋ_P = l_3 sin(θ2+θ3)+l_2 sinθ2`（第一行漏求导，实为 FK 的 $x_P$ 本身），第二行才是求导结果 $\dot x_P=l_3\cos(\theta_2+\theta_3)(\dot\theta_2+\dot\theta_3)+l_2\cos\theta_2\dot\theta_2$。OCR 把 FK 式与其导数混排，订正见 [K17]。
4. **[宇树](5.40)整段**：源2865 后半完全乱码（大量 `J_{2 1}`、`-1,-1,...`、`| |.` 噪声），$\dot y_P,\dot z_P$ 及 $J_{2j},J_{3j}$ 各项**无法从该页恢复**，须据 [K13] FK 式重新逐项求导补全（标 `\rebuilt`）。**列为最高优先 OCR 风险。**
5. **[于](5.44)**：${}^{\mathcal O}\mathbf p_{com\times}$ 的反对称下标在 MinerU 中时隐时现（`p_{c o m\times}`），确认为 $[{}^{\mathcal O}\mathbf p_{com}]_\times$。
6. **[于](2.22)**：`atan` 应为 `atan2`（需象限信息，否则逆解错误），订正见 [K07]；源(5.18,5.37) 宇树明确用 atan2，可佐证。
7. **[于](5.57)**：$X_C(i)$ 左上块印作空白 + `\mathbb I_{3\times3}`，结合(5.20)平移变换结构订正为 $\begin{bmatrix}\mathbb I_3&\mathbf 0\\-([0,0,-l_3]^T)_\times&\mathbb I_3\end{bmatrix}$。
8. **[笔记]166**：罗德里格斯 $\hat{\mathbf u}_\times^2$（叉乘矩阵平方）排版为 `\mathbf{\hat u}_\times^2`，正确，无需订正——但成章须显式说明是矩阵平方非分量平方。
9. **[宇树](5.9)**：$l_1=\mp l_{abad}$ 的左右腿符号（右前/右后取负、左前/左后取正）易随 OCR 丢符号，须保留原表（源2641）。

---

## F. 三源差异/互补点（供融合，避免拼接）

| 主题 | [笔记] | [宇树] | [于] | 融合策略 |
|---|---|---|---|---|
| 旋转矩阵切入 | 基向量投影 + 三用途口语 | 坐标变换 + 下标消去（最直观） | 单位正交基严格定义 | 以宇树"下标消去"建直觉，于宪元给严格性，笔记给"三用途"洞见 |
| 建模范式 | DH 参数法（机械臂传统） | 齐次变换连乘（工程实用） | Plücker 空间向量 + 运动学树（现代统一） | 历史线：DH→齐次变换→空间向量；四足主用后两者，DH 作历史 `history` 盒 |
| 单腿 FK | 概念（FK 易/IK 难） | 完整闭式 [K13] | 递推算法 [K29] | 宇树给单腿闭式（可手算/数值例），于宪元给递推（可扩展 N 刚体）；二者互验 |
| 单腿 IK | 概念（多解/工作空间） | 完整三步解耦 [K14-16] | 不展开（树算法不需逐腿 IK） | 宇树独家，全量录入 |
| 雅可比 | 一般定义 + 静力学对偶 | 单腿直接求导 $3\times3$ | 全身刚体雅可比（Plücker） | 笔记给"为何对偶"洞见，宇树给单腿可逆 $J$，于宪元给整机 $J_1\dots J_4$ |
| 角速度↔姿态 | 引于宪元 | 牵连速度 $[\omega]_\times p$ | 角速度张量严格推导 [K10] | 于宪元的张量推导作理论；宇树的牵连速度作整机应用 |
| 奇异/零空间 | **独家完整证明** [K35-40] | 仅"$J$ 可逆"一句 | 用 MP 伪逆但不证 | **笔记独家，作 §10 高潮**；与于宪元 WBC 任务雅可比衔接 |
| 整机运动学 | 四足坐标系命名 | 姿态↔足端 + 足端速度 [K41-43] | 运动学树 + 浮动基座 | 宇树给"控姿态"应用直觉，于宪元给浮动基座严格框架 |
| 静止惯性系 $\{b\}$ | — | **独家强调** K11 | 用刚体系 $B_i$ | 宇树的"瞬时重合静止系"洞见是理解整机速度的钥匙 |

---

## G. 数值例/表格建议（成章充实，读者不翻原书）

- **表**：四足坐标系一览（世界/机身/关节/连杆/足底系 + 定向足底系 $\mathcal Q_i$），源 [笔记]105 表 + [于]§2.2.1。
- **表**：机器人物理尺寸 $[h_x,h_y,l_1,l_2,l_3]=[0.275,0.0625,0.088,0.27,0.27]$ m（[于](2.2)）；A1 腿长 $l_{abad},l_{hip},l_{knee}$（[宇树]§5）。
- **数值例 1**：给定 $(\theta_1,\theta_2,\theta_3)$ 代入 [K13] 算足端位置（FK 正算）。
- **数值例 2**：给定足端 $(x_P,y_P,z_P)$ 走 [K14-16] 三步反解关节角（IK），并用 FK 回代验证（闭环自洽）。
- **数值例 3**：在某构型下装配 $3\times3$ 雅可比 [K17]，计算 $\det J$ 演示远奇异/近奇异，配阻尼 [K38] 对比。
- **TikZ 图建议**：(a) 四足坐标系总览；(b) 单腿三关节坐标系 $\{0\}\{1\}\{2\}\{3\}$ 与 $l_{abad},l_{hip},l_{knee}$；(c) IK 的 yz 投影几何（$\theta_1$）与大腿小腿平面（$\theta_3$ 余弦定理）；(d) 运动学树（固定基座 0 + 浮动基座 1 + 12 连杆 + 虚拟 6-DoF 关节）；(e) 零空间投影示意（任意 $\dot q_\forall$ 正交投影到 $N(J)$）。

---

## H. 编译安全自查（成章遵守）

- 中文一律 `\text{}` 入 math；表格 `\centering`；`\cnum` 已全局定义勿重定义。
- 盒子环境仅用范本出现过的：`def`/`theorem`/`proof`/`insight`/`note`/`pitfall`/`practice`/`algorithm`/`motivation`/`history`。
- 图优先 TikZ 重绘（本批次数据图可省略或占位）。
- `\cref{ch:lie}`（指数坐标/矩阵对数/$\dot R$ 推导）、`\cref{ch:rigid_body}`（SE(3)）衔接；动力学标准形 [K34]、陀螺项对照 [K26] 预告后续动力学/WBC 章。
