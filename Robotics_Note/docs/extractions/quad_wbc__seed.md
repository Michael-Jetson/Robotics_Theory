# 抽取留痕：四足机器人运动控制 —— 全身控制 WBC（tag=wbc）

> **文件性质**：项目内部「抽取留痕」，**非成书正文**。服务于新部「四足机器人运动控制」之 **全身控制（WBC）章**（`ch:quad_wbc`）。
>
> **抽取原则**：**完全吸收·禁摘要·禁凝练**。每一道公式、每一步推导、每一段动机解释全量录入，公式一律改写为规范 LaTeX，保留作者所有中间步骤。目标成章 25–45 页。
>
> **本章定位**：从 VMC/ZMP 朴素思想起步 → 零空间数学地基（引 `ch:quad_kin`） → 任务分层（带优先级双/多任务、位置/速度/加速度级）→ 浮动基座逆动力学（把 `ch:quad_mpc` 输出的虚拟足端力/质心指令转成可执行关节力矩与接触力）→ 子任务划分 → 硬优先级松弛为加权 QP（松弛优化）。与 `ch:quad_mpc` 形成"MPC 算力 → WBC 转矩"控制链闭环。
>
> **作者洞见标记法**：凡作者自述的理解/动机/"为什么"，正文内用 `〔作者洞见〕` 前缀就地标出，文末另设清单汇总编号，供成章时一一对应做 `insight`/`remark` 盒。

---

## 源材料清单与出处

| 代号 | 来源 | 行号 | 在本抽取中承担 |
|---|---|---|---|
| **[个人笔记·零空间]** | 作者个人笔记 `足式机器人控制.md` §奇异性问题与零空间理论 | line 332–458 | WBC 数学地基：冗余/零空间定义、Moore-Penrose 伪逆、阻尼最小二乘、通解结构、零空间投影矩阵的拉格朗日乘子法**完整证明**、秩-零度定理、正交投影解释。**含作者原创动机解释多条。** |
| **[个人笔记·VMC/ZMP]** | 作者个人笔记 §VMC原理 / §零力矩点ZMP | line 1030–1061 | VMC 直觉控制思想、虚拟力→关节力矩映射、二自由度单腿例、ZMP 动态稳定性判据、支撑多边形。 |
| **[个人笔记·WBC]** | 作者个人笔记 §任务分层的WBC原理 / §优化的WBC原理 / §四足中的WBC应用 | line 1062–1192 | 任务分解动机（望远镜例）、双任务/多任务递推、NSP→WQP→HQP 三层方法谱系、浮动基座 18 维关节空间、四子任务优先级表、四足递推 WBC 伪代码（加速度级）。 |
| **[于宪元 §4]** | 于宪元硕论 §4 全身控制 | line 1692–2167 | §4.1 带优先级 WBC（双/多任务/位置级/加速度级，逐式编号 4.1–4.31）；§4.2 四足应用（浮动基座动力学 4.32–4.40、子任务雅可比）；§4.3 松弛优化（4.41–4.62，QP 标准化 → 关节扭矩）。**最系统、含全部接触/选择矩阵与 QP 推导。** |

> **本书全局记号统一**（三源冲突时以此为准，并在 `note` 盒标换算）：
> - 四元数=Hamilton；`se(3)` 平移在前；扰动主线=右扰动。本章涉及旋转矩阵微分 $\dot{\mathbf R}=[\boldsymbol\omega]^\wedge\mathbf R$（与 `ch:quad_mpc` 一致），引 `ch:lie_group`。
> - 于宪元用 $N_j$ 记关节数、$\mathbb I$ 记单位阵、$J^+$ 记右伪逆；个人笔记用 $N$ 记关节数。成书统一：关节数 $n$（或 $N_j$ 在浮动基段），自由度维 $M$，伪逆 $J^+$，零空间投影 $N(J)$ 或 $N$。
> - 浮动基广义坐标 $\boldsymbol q=[\boldsymbol q_b^T,\ \boldsymbol q_j^T]^T$，$\boldsymbol q_b=[\boldsymbol\Theta^T,\ \boldsymbol p_{com}^T]^T$（欧拉角+质心位置）。注意于宪元广义速度 $\dot{\boldsymbol q}$ 是**重定义**（角速度+本体平动速度+关节速度），**非** $\boldsymbol q$ 的逐元素微分 → 须在 `note` 盒强调。

> **`\cite` 键**：本章主线引 `\cite{yu2021quadruped}`（于宪元硕论，BUAA 2021）；个人笔记作者思路做 `insight` 不单独 cite，但零空间投影化简引 `\cite{maciejewski1985obstacle}`（Maciejewski & Klein, IJRR 1985, 4(3):109-117）。MPC 控制链对侧引 `ch:quad_mpc`/`\cite{dicarlo2018cheetah}`。

---

# 拟定节结构（对应教材 v5.0：动机→反面→历史→理论→陷阱→练习）

> 难度星：本章理论密度高，建议 ★★★★（与 MPC 章同级）。

1. **§1 动机：为什么需要 WBC**（motivation 盒）
   - 一条腿 12 关节、一个浮动躯干 6 自由度，一个关节服务多任务、一个任务用多关节 → 需要"全身协调"框架。
   - 控制链定位：`ch:quad_mpc` 给出**质心层**最优足端力/质心指令（侧重"未来一段时间"），但它不管单腿各关节如何分配、不管摆动腿轨迹、不管严格优先级 → WBC 负责把质心层指令"翻译"成 12 个关节的可执行**力矩**（侧重"此刻任务优先级"）。

2. **§2 反面教材：朴素直觉控制 VMC 与静态判据 ZMP 的边界**（pitfall/motivation 盒）
   - VMC：虚拟弹簧阻尼 + 雅可比转置映射，简单直观、免逆运动学，但无优先级、无约束、无预测 → 作为"WBC 的史前形态"引入。
   - ZMP：动态稳定判据（合力矩水平分量为零点落在支撑多边形内），比 CoG 静态判据进步，但仍是判据而非控制器。

3. **§3 历史与方法谱系**（history/insight 盒）
   - 零空间投影（NSP，硬优先级、解析、快，但不处理不等式约束）→ 加权二次规划（WQP，软优先级、可加不等式约束，但优先级不严格）→ 层级二次规划（HQP，级联 QP，严格优先级+不等式，但慢）。本章主线落在 NSP 递推 + 末端松弛 QP（于宪元方案）。

4. **§4 数学地基：冗余、零空间与伪逆**（theory，引 `ch:quad_kin`）
   - §4.1 正/逆速度映射与冗余；§4.2 零空间定义与通解结构；§4.3 Moore-Penrose 伪逆（最小范数解的拉格朗日乘子法**完整证明**）；§4.4 阻尼最小二乘（奇异点数值稳定）；§4.5 零空间投影矩阵的正交投影解释；§4.6 秩-零度定理。

5. **§5 带优先级的任务分层 WBC**（theory）
   - §5.1 双任务递推；§5.2 多任务递推（累加链 + 起始条件）；§5.3 位置级/速度级/加速度级三种映射。

6. **§6 浮动基座 WBC**（theory）
   - §6.1 浮动基座机器人与 6 自由度虚拟关节；§6.2 浮动基座动力学标准方程（质量阵/偏置力/接触雅可比/选择矩阵）；§6.3 四子任务划分与优先级；§6.4 各子任务雅可比与误差（含转动误差的基变换）；§6.5 递推 WBC 伪代码（加速度级）。

7. **§7 松弛优化：硬优先级软化为加权 QP**（theory，闭环 MPC）
   - §7.1 MPC 足端力与 WBC 加速度的不一致性；§7.2 松弛变量与带约束优化问题；§7.3 化为 QuadProg++ 标准 QP（目标/等式/不等式）；§7.4 反解关节扭矩 $\boldsymbol\tau_j$ 与最终力矩指令（前馈+PD）。

8. **§8 陷阱与数值实务**（pitfall 盒）
   - 奇异点伪逆病态；优先级用尽导致低优先级失效；WQP 权重污染；欧拉角误差需变基；广义速度非逐元素微分；接触切换时 $J_c$ 维度突变。

9. **§9 练习与实践**（exercise / practice 盒）
   - 习题：验证 $JN=0$、推导双任务化简、二自由度单腿 VMC 数值例。
   - practice 盒：QuadProg++/qpOASES 选型、$Q_1=\mathbb I,Q_2=0.005\mathbb I$ 的工程含义、关节级 PD 增益整定（跳过代码细节）。

---

# 第一部分：动机与反面教材（VMC / ZMP）

> 源：[个人笔记·VMC/ZMP] line 1030–1061。

## §VMC 原理 —— WBC 的"史前形态"

**核心思想**（line 1032）：虚拟模型控制（Virtual Model Control, VMC）是一种**直觉控制方法**，用虚拟机械部件（弹簧、阻尼器、轴承等）连接机器人内部作用点或作用点与环境，产生**虚拟力**驱动机器人运动。虚拟力非实际执行力，需通过雅可比矩阵映射到执行机构得到期望关节力矩。

〔作者洞见 ①〕**VMC 为何直观**（line 1032–1034）："用户可以直观地设计虚拟力，而不需要深入处理底层动态方程。"例：敲门任务难以数学描述，但用 VMC 只需在手上附加虚拟弹簧+阻尼+给定动能的虚拟质量，手在虚拟组件作用下移动、撞击、回弹。
> 成书 `insight`：VMC 把"控制律设计"降维成"想象一组机械元件"，这是它在早期四足（如 MIT Leg Lab）流行的原因，也是 WBC"任务=期望力/加速度"思想的直觉前身。

**虚拟力→关节力矩的映射**（line 1046–1048）：先由正运动学建立空间映射
$$
\mathbf x=f(\mathbf q),\quad \mathbf x=[x_1,\cdots,x_m]^T,\ \mathbf q=[q_1,\cdots,q_n]^T
$$
其中 $\mathbf x$ 是机器人系相对地面系的 $m$ 自由度位姿向量，$\mathbf q$ 为 $n$ 个关节变量。求导得雅可比（即一阶微分运动学/静力学，引 `ch:quad_kin`）。由静力学对偶 $\boldsymbol\tau=J^T\boldsymbol F_{\text{virtual}}$（虚拟力经 $J^T$ 映射为关节力矩）。

〔作者洞见 ②〕**VMC 应用两关键**（line 1044）："一是在每个需控制自由度上构造恰当虚拟构件产生合适虚拟力；二是在不同相位状态利用相应雅可比计算期望关节力矩。"并强调（line 1054）雅可比**随关节角度变化**，非定值。

> **二自由度单腿例**（line 1036）：A、B 两转动副电机驱动，连杆长 $L_1,L_2$，关注足端 C 点位置（直角或极坐标）。成书可做 TikZ 重绘示意 + 数值例：给定期望足端力，算两关节力矩。

## §零力矩点 ZMP —— 动态稳定性判据

**定义**（line 1058）：ZMP（零力矩点 / 零倾覆力矩点）是地面上一点，机器人重力与惯性力所产生合力矩在该点的**水平分量为零**。若运动时 ZMP 位于**支撑多边形**内，则保证运动过程稳定（动态稳定性判据）。

〔作者洞见 ③〕**ZMP 相对 CoG 的进步**（line 1058）：ZMP 相对传统静态稳定判据（重心投影法 / CoG 判据）"充分考虑了机器人重力和惯性力的合力的影响"。支撑多边形（line 1060）= 四足足底与地面所有接触点构成的多边形；ZMP 越界 → 倾倒。

> 成书 `pitfall`/`remark`：VMC 与 ZMP 都是"无优先级、无显式约束优化"的早期方案 → 引出后文 WBC 的必要性。

---

# 第二部分：数学地基 —— 冗余、零空间与伪逆

> 源：[个人笔记·零空间] line 332–458 与 [于宪元 §4 引言+4.1] line 1694–1778。两源高度一致；个人笔记更详（含完整拉格朗日证明与正交投影解释），于宪元给出规范编号公式 (4.1)–(4.9)。成书以个人笔记证明为主体，编号对齐于宪元。

## §正/逆速度映射与冗余

**正运动学与雅可比**（于宪元 4.1–4.2）：
$$
\boldsymbol x=f(\boldsymbol q)\tag{4.1}
$$
$$
\dot{\boldsymbol x}=J(\boldsymbol q)\,\dot{\boldsymbol q}\tag{4.2}
$$
$J(\boldsymbol q)$ 为 $M\times N_j$ 矩阵（行=工作空间维 $M$，列=关节数 $N_j$），是 $\boldsymbol q$ 的函数。

〔作者洞见 ④〕**正向唯一、逆向不唯一**（个人笔记 line 334–336）："从关节到工作空间的正向速度映射很容易实现……明确的 $\dot{\boldsymbol q}$ 必然对应明确且唯一的 $\dot{\boldsymbol x}$。但反过来不一定。"定义 $R(J)$ 为可达工作空间速度集合（$R^M$ 子集）。**冗余**（line 338）：自由度 $N>$ 任务维 $M$（例 7-DOF 臂做 6-DOF 抓取）。

**于宪元的冗余反例**（4.3）：平面四自由度机器人（$q_1,q_2,q_3$ 转动、$q_4$ 直线，连杆长均 $l$），当 $q_1=q_3$ 且
$$
\sin(\pi-\dot q_2)=-\frac{\dot q_4}{l^2}\tag{4.3}
$$
时末端速度为 0（即存在非零关节运动而末端静止——零空间运动的具体实例）。
> **OCR 疑点 A**：式(4.3) 量纲存疑——左边无量纲，右边 $\dot q_4/l^2$ 量纲为 [1/(长度·时间)]，不自洽。疑原印为 $\sin(\pi-q_2)=-\dot q_4/(l\,\dot q_2)$ 之类的零末端速度条件，MinerU 转写漏项。成书**仅作定性反例引用**，不照搬此式量纲；`note` 盒标"OCR 疑点，原式量纲不自洽，按定性零空间运动理解"。

## §零空间定义与通解结构

**零空间投影矩阵**（个人笔记 line 342–348 / 于宪元 4.4）：定义 $N(J)$ 为 $J$ 的零空间投影矩阵，满足
$$
J\cdot N(J)=0\tag{4.4}
$$
零空间数学定义：
$$
N(A)=\{\,\boldsymbol w\in\mathbb R^N \mid A\boldsymbol w=\boldsymbol 0\,\}
$$
其秩 = 机器人冗余程度。

**通解结构**（个人笔记 line 350–366）：若 $A\boldsymbol w=\boldsymbol b$ 有特解 $\boldsymbol w_p$，则所有解
$$
\boldsymbol w_{\text{general}}=\boldsymbol w_p+\boldsymbol w_0,\quad \boldsymbol w_0\in N(A)
$$
对应到 $\dot{\boldsymbol x}=J\dot{\boldsymbol q}$：通解 $\dot{\boldsymbol q}_{\text{general}}=\dot{\boldsymbol q}_p+\dot{\boldsymbol q}_0$，$\dot{\boldsymbol q}_0\in N(J)$。

〔作者洞见 ⑤〕**零空间运动的物理意义**（line 366）：$\dot{\boldsymbol q}_0$ 是"内部运动"——关节运动但**末端在工作空间完全静止**（$\dot{\boldsymbol x}=\boldsymbol 0$）。这是 WBC 让低优先级任务"免费搭车"的物理基础。

## §Moore-Penrose 伪逆（右逆）与零空间矩阵

**右逆与零空间矩阵**（个人笔记 line 368–376 / 于宪元 4.5–4.7）：定义 $N\times M$ 右逆 $J^+$，满足 $JJ^+=\mathbb I$。由
$$
\boldsymbol 0=J-\mathbb I J=J-JJ^+J=J(\mathbb I-J^+J)\tag{4.5}
$$
得
$$
N=\mathbb I-J^+J\tag{4.6}\qquad\qquad J^+=J^T(JJ^T)^{-1}\tag{4.7}
$$
通解（个人笔记 line 384 / 于宪元 4.8）：
$$
\dot{\boldsymbol q}=J^+\dot{\boldsymbol x}+N\dot{\boldsymbol q}_{\forall}\tag{4.8}
$$
$\dot{\boldsymbol q}_{\forall}\in R^{N}$ 任意。验证（于宪元 4.9）：左乘 $J$，$J\dot{\boldsymbol q}=JJ^+\dot{\boldsymbol x}+JN\dot{\boldsymbol q}_{\forall}=\mathbb I\dot{\boldsymbol x}+\boldsymbol 0=\dot{\boldsymbol x}$。

〔作者洞见 ⑥〕**WBC 的核心一句话**（个人笔记 line 388）："在尽可能保证工作空间任务 $\dot{\boldsymbol x}$ 完成的前提下，关节可以按 $\dot{\boldsymbol q}_{\forall}$ 任意运动来实现其他额外任务（如关节避障）——这种充分利用关节冗余、同时保证多任务的方法称为 WBC。"**此句是全章主线锚点**，成书放章首 `insight`。

## §⭐ 伪逆为最小范数解的完整证明（拉格朗日乘子法）

> 源：个人笔记 line 390–435。**于宪元未给此证明，个人笔记独有**，是本章理论深度的关键，须全量保留。

**问题**：右逆有无穷多个（$JG=\mathbb I$ 的 $G$ 不唯一，line 392），$J^+$ 是其中"能量最优"特例。优化目标——最小关节速度模长（line 396）：
$$
\min_{\dot{\boldsymbol q}}\left(\tfrac12\|\dot{\boldsymbol q}\|^2\right)=\min_{\dot{\boldsymbol q}}\left(\tfrac12\dot{\boldsymbol q}^T\dot{\boldsymbol q}\right)
$$
约束（line 400）：$J\dot{\boldsymbol q}=\dot{\boldsymbol x}\Rightarrow J\dot{\boldsymbol q}-\dot{\boldsymbol x}=\boldsymbol 0$。引入 $M\times1$ 乘子 $\boldsymbol\lambda$，拉格朗日函数（line 404）：
$$
\mathcal L(\dot{\boldsymbol q},\boldsymbol\lambda)=\tfrac12\dot{\boldsymbol q}^T\dot{\boldsymbol q}-\boldsymbol\lambda^T(J\dot{\boldsymbol q}-\dot{\boldsymbol x})
$$
平稳点条件（line 409–416）：
$$
\boldsymbol 0=\nabla_{\dot{\boldsymbol q}}\mathcal L=\dot{\boldsymbol q}-J^T\boldsymbol\lambda\ \Longrightarrow\ \dot{\boldsymbol q}=J^T\boldsymbol\lambda
$$
$$
\boldsymbol 0=\nabla_{\boldsymbol\lambda}\mathcal L=-(J\dot{\boldsymbol q}-\dot{\boldsymbol x})\ \Longrightarrow\ J\dot{\boldsymbol q}=\dot{\boldsymbol x}
$$
代入（line 422–427，假设非奇异 $J$ 行满秩 $\Rightarrow JJ^T$ 可逆）：
$$
J(J^T\boldsymbol\lambda)=\dot{\boldsymbol x}\Rightarrow(JJ^T)\boldsymbol\lambda=\dot{\boldsymbol x}\Rightarrow\boldsymbol\lambda=(JJ^T)^{-1}\dot{\boldsymbol x}
$$
回代得最小范数解（line 431–433）：
$$
\dot{\boldsymbol q}=J^T(JJ^T)^{-1}\dot{\boldsymbol x}=J^+\dot{\boldsymbol x},\qquad J^+=J^T(JJ^T)^{-1}
$$

〔作者洞见 ⑦〕**伪逆的深层几何**（line 435）："$J^+\dot{\boldsymbol x}$ 位于 $J$ 的**行空间**（与零空间正交），这是更深刻的数学原因，也是后续零空间/多任务/全身控制的**真正基石**。"成书做 `insight`：特解在行空间、零空间补偿在零空间，二者正交分解 → 优先级控制天然成立。

## §阻尼最小二乘（奇异点数值稳定）

〔作者洞见 ⑧〕（line 380）：非奇异构型下 $J$ 行满秩、$JJ^T$ 可逆；但**接近奇异点**时 $JJ^T$ 行列式趋零成病态矩阵，求逆数值溢出。故用**阻尼最小二乘**：
$$
J^+=J^T(JJ^T+\lambda^2\mathbb I)^{-1}
$$
微小阻尼项 $\lambda^2\mathbb I$ 保证恒可逆（即使奇异点），代价是精度略损。成书 `pitfall` 盒。

## §零空间投影矩阵的正交投影解释 + 秩-零度定理

**秩-零度定理**（line 437）：输入空间维 = 值域维 + 零空间维（机器人学中：关节空间维 = 工作空间维 + 零空间维）。非奇异冗余 → 零空间维 $>0$，必存在内部运动 $\dot{\boldsymbol q}_0$，$J\dot{\boldsymbol q}_0=\boldsymbol 0$（4.39 个人笔记）。

**投影矩阵作为正交投影**（line 441–457）：次要任务想执行任意 $\dot{\boldsymbol q}_{\forall}$，但会干扰主任务，需"过滤器"投影矩阵 $N$：$\dot{\boldsymbol q}_0=N\dot{\boldsymbol q}_{\forall}$。

〔作者洞见 ⑨〕**优先级控制的本质**（line 447）："零空间投影矩阵的意义**不是**完美执行次要任务，而是**在完美执行主任务的前提下，尽最大努力近似次要任务**。"作为正交投影，它在 $N(J)$ 中找唯一向量使其与原始意图欧氏距离最小（误差 $\dot{\boldsymbol q}_{\forall}-\dot{\boldsymbol q}_0$ 与零空间正交）。由 $JN\dot{\boldsymbol q}_{\forall}=\boldsymbol 0$ 对任意 $\dot{\boldsymbol q}_{\forall}$ 成立 $\Rightarrow J\cdot N=0$，故 $N$ 必是 $J$ 的零空间矩阵，且为唯一正交投影矩阵。
> 成书 `insight`：把"硬优先级"翻译成"正交投影"，几何上一目了然，是 NSP 方法的灵魂。

---

# 第三部分：带优先级的任务分层 WBC

> 源：[个人笔记·WBC] line 1062–1141 与 [于宪元 §4.1.1–4.1.4] line 1780–1934。两源公式一致，于宪元有编号 4.10–4.31。

## §任务分解动机（望远镜例）

〔作者洞见 ⑩〕（个人笔记 line 1066 / 于宪元 line 1782）：综合控制任务可按意义分解为子任务，可同优先级加权平均，也可按重要程度分优先级（低优先级不应影响高优先级）。**望远镜例**：机械臂操作望远镜观星，几十米位置误差不影响观测，但 <1° 角度误差导致巨大视野变化 → **姿态任务优先级高于位置任务**。这是"为什么需要优先级"的最佳直觉例，成书 `motivation`/`insight`。

## §双任务全身控制（于宪元 4.10–4.14）

两任务 $\dot{\boldsymbol x}_1,\dot{\boldsymbol x}_2$，雅可比 $J_1,J_2$，零空间 $N_1,N_2$，任务 1 优先级高。由 (4.8)：
$$
\dot{\boldsymbol q}=\underbrace{J_1^+\dot{\boldsymbol x}_1}_{\text{特解 (完成任务1)}}+\underbrace{N_1\dot{\boldsymbol q}_{\forall}}_{\text{齐次解 (不影响任务1)}}\tag{4.10}
$$
代入 $\dot{\boldsymbol x}_2=J_2\dot{\boldsymbol q}$（于宪元 4.11 / 个人笔记 line 1082–1086）：
$$
\dot{\boldsymbol x}_2=J_2(J_1^+\dot{\boldsymbol x}_1+N_1\dot{\boldsymbol q}_{\forall})=\underbrace{J_2J_1^+\dot{\boldsymbol x}_1}_{\text{A.\ 任务1的"副作用"}}+\underbrace{J_2N_1\dot{\boldsymbol q}_{\forall}}_{\text{B.\ 可控部分}}
$$
解出（4.12）——调 $\dot{\boldsymbol q}_{\forall}$ 让 B 补偿 A 并等于 $\dot{\boldsymbol x}_2$：
$$
\dot{\boldsymbol q}_{\forall}=(J_2N_1)^+(\dot{\boldsymbol x}_2-J_2J_1^+\dot{\boldsymbol x}_1)\tag{4.12}
$$
代回 (4.10) 得双任务解（4.13）：
$$
\dot{\boldsymbol q}=J_1^+\dot{\boldsymbol x}_1+N_1(J_2N_1)^+(\dot{\boldsymbol x}_2-J_2J_1^+\dot{\boldsymbol x}_1)\tag{4.13}
$$
**化简**（个人笔记 line 1096 / 于宪元 4.14，引 `\cite{maciejewski1985obstacle}` 证 $N_1(J_2N_1)^+=(J_2N_1)^+$）：
$$
\dot{\boldsymbol q}=J_1^+\dot{\boldsymbol x}_1+(J_2N_1)^+(\dot{\boldsymbol x}_2-J_2J_1^+\dot{\boldsymbol x}_1)\tag{4.14}
$$

〔作者洞见 ⑪〕（个人笔记 line 1088）"A 部分是任务 1 给任务 2 带来的'副作用'，B 是我们能控制的部分——必须用 B 去补偿 A 才能逼近 $\dot{\boldsymbol x}_2$。"成书 `insight`：把代数项赋予"副作用/补偿"语义，是理解递推的钥匙。

## §多任务递推（于宪元 4.15–4.21 / 个人笔记 line 1106–1141）

$n$ 个任务，$i$ 越小优先级越高。前 $i-1$ 个任务堆叠成大任务 $A_{i-1}$（4.15–4.16）：
$$
\dot{\boldsymbol x}_{i-1}^A=\begin{bmatrix}\dot{\boldsymbol x}_1\\\vdots\\\dot{\boldsymbol x}_{i-1}\end{bmatrix},\quad
J_{i-1}^A=\begin{bmatrix}J_1\\\vdots\\J_{i-1}\end{bmatrix},\quad
\dot{\boldsymbol x}_{i-1}^A=J_{i-1}^A\dot{\boldsymbol q}_{i-1}
$$
其零空间（4.17）：$N_{i-1}^A=\mathbb I-(J_{i-1}^A)^+J_{i-1}^A$。设 $\dot{\boldsymbol q}_{i-1}$ 已知（4.18）：$\dot{\boldsymbol q}_i=\dot{\boldsymbol q}_{i-1}+N_{i-1}^A\dot{\boldsymbol q}_{\forall}$。代入 $\dot{\boldsymbol x}_i=J_i\dot{\boldsymbol q}_i$（4.19）解出（4.20）：
$$
\dot{\boldsymbol q}_{\forall}=(J_iN_{i-1}^A)^+(\dot{\boldsymbol x}_i-J_i\dot{\boldsymbol q}_{i-1})\tag{4.20}
$$
回代得**递推公式**（4.21，同样用 Maciejewski 化简）：
$$
\dot{\boldsymbol q}_i=\dot{\boldsymbol q}_{i-1}+(J_iN_{i-1}^A)^+(\dot{\boldsymbol x}_i-J_i\dot{\boldsymbol q}_{i-1})\tag{4.21}
$$
起始条件 $\dot{\boldsymbol q}_1=J_1^+\dot{\boldsymbol x}_1$，迭代累加至 $\dot{\boldsymbol q}_n$（最终关节速度指令）。

〔作者洞见 ⑫〕（个人笔记 line 1141）"$\dot{\boldsymbol q}_n$ 是递推链的最终输出——发送给机器人的那个'总的'关节速度指令。"

## §位置级与加速度级（于宪元 4.22–4.31）

**位置级**（4.22–4.25）：定义工作空间误差 $\boldsymbol e_i=\boldsymbol x_i^d-\boldsymbol x_i$、关节误差 $\Delta\boldsymbol q_i=\boldsymbol q_i^d-\boldsymbol q_i$。以 $\boldsymbol e_i/T$、$\Delta\boldsymbol q_i/T$ 近似为速度代入 (4.21)，两边乘 $T$：
$$
\Delta\boldsymbol q_i=\Delta\boldsymbol q_{i-1}+(J_iN_{i-1}^A)^+(\boldsymbol e_i-J_i\Delta\boldsymbol q_{i-1})\tag{4.25}
$$
**加速度级**（4.26–4.31）：微分 (4.2) 得 $\ddot{\boldsymbol x}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}$（4.26）；多任务 $\ddot{\boldsymbol x}_i=\dot J_i\dot{\boldsymbol q}+J_i\ddot{\boldsymbol q}_i$（4.27，$\dot{\boldsymbol q}$ 从传感器读取为已知）。同理递推（4.31）：
$$
\ddot{\boldsymbol q}_i=\ddot{\boldsymbol q}_{i-1}+(J_iN_{i-1}^A)^+(\ddot{\boldsymbol x}_i-\dot J_i\dot{\boldsymbol q}-J_i\ddot{\boldsymbol q}_{i-1})\tag{4.31}
$$
起始 $\ddot{\boldsymbol q}_1=J_1^+(\ddot{\boldsymbol x}_1-\dot J_1\dot{\boldsymbol q})$。
> **OCR 疑点 B**：于宪元 (4.34) 后正文写 $\dot{\boldsymbol q}_1=J_1^+(\ddot{\boldsymbol x}_1-\dot J_1\dot{\boldsymbol q})$，左边记号印作 $\dot{\boldsymbol q}_1$ 但语境是加速度级递推**起始**，应为 $\ddot{\boldsymbol q}_1$。成书订正为 $\ddot{\boldsymbol q}_1$，`note` 盒标"OCR 订正：原印 $\dot q_1$ → $\ddot q_1$（加速度级起始）"。

## §三种方法谱系：NSP → WQP → HQP

> 源：个人笔记 line 1145–1160。于宪元未展开，个人笔记独有方法学综述，成书 `history`/`insight` 盒主线。

**NSP（零空间投影 / 任务分层）**：解析、计算快（有闭式解），但（line 1147）"过于依赖预定义优先级，不同优先级任务不会相互妥协；高优先级用尽自由度时低优先级完全失败；无法处理不等式约束。"

**WQP（加权二次规划，软优先级）**（line 1149–1155）：把 $N$ 个任务误差 $\boldsymbol e_i=\mathbf A_i\boldsymbol x-\mathbf b_i$ 组合为单一加权最小二乘：
$$
\min_{\boldsymbol x}\ \sum_{i=1}^N w_i\|\mathbf A_i\boldsymbol x-\mathbf b_i\|^2
$$
优点：可调权重让任务相互协调（高优先级让出小偏差换低优先级大改善）、可加不等式约束。缺点：软优先级**不保证严格优先级**，低权重任务若 $\|J\|$ 大或瞬时误差大仍可能污染高优先级。

**HQP（层级二次规划，集大成）**（line 1157–1159）：级联 QP，按优先级依次求解，低优先级 QP 附带"严格保持高优先级最优解"的等式约束（即只在高优先级 QP 的零空间内优化）。优点：严格优先级 + 不等式约束；缺点：需连续求解多个 QP，实现与耗时要求高。

〔作者洞见 ⑬〕：NSP 是几何（零空间），WQP 是软优化，HQP 是"NSP 的严格优先级 + QP 的不等式处理"的统一。于宪元实际方案 = NSP 递推（§5）+ 末端单个松弛 QP（§7），介于三者之间。

---

# 第四部分：浮动基座 WBC 与四足应用

> 源：[个人笔记·WBC] line 1161–1192 与 [于宪元 §4.2] line 1936–2022。

## §浮动基座与 6 自由度虚拟关节

〔作者洞见 ⑭〕（个人笔记 line 1163 / 于宪元 line 1940）：四足可等效为"躯干上连 4 个 3-DOF 臂"，但躯干本身是世界系下 6-DOF 刚体（**浮动基座**），仅 12 个关节参数无法求解刚体世界位姿。解法（于宪元 line 1949）：在世界系与本体系间加一个**无实体的 6-DOF 虚拟关节**，共 13 关节、18 维关节空间：
$$
\boldsymbol q=\begin{bmatrix}\boldsymbol q_b\\\boldsymbol q_j\end{bmatrix}\tag{4.32}
$$
$\boldsymbol q_b=[\boldsymbol\Theta^T,\ \boldsymbol p_{com}^T]^T$（欧拉角+质心位置），$\boldsymbol q_j$ 为 12 实体关节。

**广义速度/加速度（重定义）**（于宪元 4.33）：
$$
\dot{\boldsymbol q}=\begin{bmatrix}{}^{\mathcal B}\boldsymbol\omega_{\mathcal{OB}}\\{}^{\mathcal B}\dot{\boldsymbol p}_{com}\\\dot{\boldsymbol q}_j\end{bmatrix},\qquad
\ddot{\boldsymbol q}=\begin{bmatrix}{}^{\mathcal B}\dot{\boldsymbol\omega}_{\mathcal{OB}}\\{}^{\mathcal B}\ddot{\boldsymbol p}_{com}\\\ddot{\boldsymbol q}_j\end{bmatrix}\tag{4.33}
$$
> **关键 `note` 盒**（于宪元 line 1970）：(4.33) 的 $\dot{\boldsymbol q},\ddot{\boldsymbol q}$ **不是** $\boldsymbol q$ 的一阶/二阶微分，而是重新定义的符号（角速度而非欧拉角速率，本体系而非世界系）。陷阱来源，须强调。

## §浮动基座动力学标准方程（于宪元 4.34–4.35）

$$
M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})=S_j\boldsymbol\tau+J_c^T\,{}^{\mathcal O}\boldsymbol f_c\tag{4.34}
$$
$M(\boldsymbol q)$=关节空间质量矩阵；$C(\boldsymbol q,\dot{\boldsymbol q})$=偏置力（含重力、科氏力等与加速度无关项）；$J_c,\boldsymbol f_c$=接触雅可比与接触力。虚拟 6-DOF 关节无法产生扭矩，用**选择矩阵** $S_j$ 过滤前 6 元素（4.35）：
$$
S_j=\begin{bmatrix}\mathbf 0_{6\times6}&\mathbf 0\\\mathbf 0&\mathbb I\end{bmatrix}\tag{4.35}
$$
物理含义（line 1982）：右边=系统受力（外部接触力+内部关节力），左边=当前状态及受力下的运动。

## §四子任务划分与优先级（个人笔记 line 1180 表 / 于宪元 line 1986）

| 任务 | 优先级 | 内容 |
|---|---|---|
| 支撑腿轨迹跟随 | 1 | 已踩地的足底在世界系**绝对静止**（$\dot{\boldsymbol x}_{sup}=\boldsymbol 0$）；失败则摔倒 |
| 机身转动控制 | 2 | 控制浮动基姿态（Roll/Pitch/Yaw）保持水平或跟随期望 |
| 机身平动控制 | 3 | 控制浮动基位置 $(x,y,z)$，最常见控身高 |
| 摆动腿足底轨迹跟随 | 4 | 控制空中摆动足跟随规划轨迹（抬起/前伸/落下） |

〔作者洞见 ⑮〕**优先级排序的逻辑**（个人笔记 line 1178 / 于宪元 line 1986）："四足靠支撑腿与地面稳定接触实现运动，对支撑腿的良好控制是其他算法的前提 → 支撑腿最高；机身平稳重要 → 转动、平动排 2、3；与外界接触主要靠支撑腿，摆动腿轨迹不太重要 → 最低。"成书 `insight`：优先级=物理因果链（接触→机身→摆动）。

## §各子任务雅可比与误差（于宪元 4.37–4.40）

**任务 1（支撑腿）**：$J_1=J_c$（支撑腿即接触腿），纯运动学逆解，无 PD 控制器，$\boldsymbol e_i,\dot{\boldsymbol x}_i^d,\ddot{\boldsymbol x}_i^{cmd}$ 视作 0（line 2008）。$\ddot{\boldsymbol x}_1^d=0$（足底世界系不动，无加速度规划，line 1990）。

**任务 2（机身转动）误差需变基**（4.37–4.38）：欧拉角的基随欧拉角变化，须把欧拉角误差变到世界系角速度的基下。对欧拉角速率映射 (3.20) 在 $\boldsymbol\Theta$ 附近线性化：
$$
\boldsymbol e_2=\begin{bmatrix}c_\theta c_\psi&-s_\psi&0\\c_\theta s_\psi&c_\psi&0\\-s_\theta&0&1\end{bmatrix}(\boldsymbol\Theta_d-\boldsymbol\Theta)\tag{4.38}
$$
任务 2 雅可比（工作空间在世界系，$\dot{\boldsymbol q}$ 角速度在本体系，4.39）：
$$
J_2=\begin{bmatrix}{}^{\mathcal O}R_{\mathcal B}&\mathbf 0_{3\times15}\end{bmatrix}\tag{4.39}
$$
**任务 3（机身平动）**（4.40）：$\boldsymbol x_3,\dot{\boldsymbol x}_3$ = 质心世界系位置 ${}^{\mathcal O}\boldsymbol p_{com}$、速度 ${}^{\mathcal O}\boldsymbol v_{com}$：
$$
J_3=\begin{bmatrix}\mathbf 0_{3\times3}&{}^{\mathcal O}R_{\mathcal B}&\mathbf 0_{3\times12}\end{bmatrix}\tag{4.40}
$$
**任务 4（摆动腿）**：世界系各摆动足位置/期望/速度/期望速度，取自状态估计器与足底轨迹规划器。
> **OCR 疑点 C**：于宪元 (4.38) 矩阵第三行末元素印作 `1`，第二列第三行 `0`——此为 ZYX 欧拉角速率→角速度映射 $E(\boldsymbol\Theta)$ 的标准形式，与本书 `ch:quad_mpc` 的角速度映射应一致。须与 MPC 章 (3.20) 逐元素核对基/转置约定（本体系 vs 世界系），`note` 盒标换算。
> **OCR 疑点 D**：(4.39)/(4.40) 中 ${}^{\mathcal O}R_{\mathcal B}$ 的下标在 MinerU 中花体 $\mathcal O/\mathcal B$ 偶有混排（${}^{?}p_{com}$ 等问号），成书统一为世界系 $\mathcal O$、本体系 $\mathcal B$。

## §递推 WBC 伪代码（加速度级，于宪元 4.36 / 个人笔记 line 1191）

$$
\begin{aligned}
&\Delta\boldsymbol q_1^{cmd}=\boldsymbol 0,\quad \dot{\boldsymbol q}_1^{cmd}=\boldsymbol 0\\
&\ddot{\boldsymbol q}_1^{cmd}=J_1^+(-\dot J_1\dot{\boldsymbol q})\\
&\textbf{for } i=2\textbf{ to }4\textbf{ do}\\
&\quad \boldsymbol e_i=\boldsymbol x_i^d-\boldsymbol x_i\\
&\quad \ddot{\boldsymbol x}_i^{cmd}=\ddot{\boldsymbol x}_i^d+K_p^i(\boldsymbol x_i^d-\boldsymbol x_i)+K_d^i(\dot{\boldsymbol x}_i^d-\dot{\boldsymbol x}_i)\\
&\quad J_{i-1}^A=[J_1^T\cdots J_{i-1}^T]^T,\quad N_{i-1}^A=\mathbb I-(J_{i-1}^A)^+J_{i-1}^A\\
&\quad \Delta\boldsymbol q_i^{cmd}=\Delta\boldsymbol q_{i-1}^{cmd}+(J_iN_{i-1}^A)^+(\boldsymbol e_i-J_i\Delta\boldsymbol q_{i-1}^{cmd})\\
&\quad \dot{\boldsymbol q}_i^{cmd}=\dot{\boldsymbol q}_{i-1}^{cmd}+(J_iN_{i-1}^A)^+(\dot{\boldsymbol x}_i^d-J_i\dot{\boldsymbol q}_{i-1}^{cmd})\\
&\quad \ddot{\boldsymbol q}_i^{cmd}=\ddot{\boldsymbol q}_{i-1}^{cmd}+(J_iN_{i-1}^A)^+(\ddot{\boldsymbol x}_i^{cmd}-\dot J_i\dot{\boldsymbol q}-J_i\ddot{\boldsymbol q}_{i-1}^{cmd})\\
&\textbf{end for}\\
&\boldsymbol q^{cmd}=\boldsymbol q+\Delta\boldsymbol q_4^{cmd},\quad \dot{\boldsymbol q}^{cmd}=\dot{\boldsymbol q}_4^{cmd},\quad \ddot{\boldsymbol q}^{cmd}=\ddot{\boldsymbol q}_4^{cmd}
\end{aligned}\tag{4.36}
$$
> **OCR 疑点 E**：个人笔记第 4 行印作 $\ddot q_1^{cmd}=(J_1)^+(-\dot J_1\dot q)$（含 $\dot J_1$），于宪元 (4.36) 印作 $J_1^+(-J_1\dot q)$（漏点，应为 $\dot J_1$）。由加速度映射 $\ddot{\boldsymbol x}=\dot J\dot{\boldsymbol q}+J\ddot{\boldsymbol q}$ 且支撑腿 $\ddot{\boldsymbol x}_1=0$ 推得 $\ddot{\boldsymbol q}_1=J_1^+(-\dot J_1\dot{\boldsymbol q})$，**个人笔记正确，于宪元漏 dot**。成书取 $\dot J_1$，`note` 盒标"OCR 订正：于宪元原印 $J_1$ → $\dot J_1$"。
> **OCR 疑点 F**：个人笔记第 9 行 `$\ddot{x}_i^{cmd}=\ddot x_i^d+K_p^i(e_i)+K_d^i(\dot x_i^d-\dot x_i)\dot x_i^{cmd}$` 末尾多出 `\dot x_i^{cmd}`（粘连），应删；以于宪元 (4.36) 该行为准（无末项）。

待求量（line 2022）：任务 1、4 的雅可比 $J_1,J_4$，所有任务的 $\dot J_i\dot{\boldsymbol q}$ 项，以及 (4.34) 中接触力 $\boldsymbol f_c$（下一章/MPC 提供）。

---

# 第五部分：松弛优化 —— 闭合 MPC↔WBC 控制链

> 源：[于宪元 §4.3] line 2024–2162。个人笔记无此节，于宪元独有，是"MPC 足端力 + WBC 加速度 → 关节扭矩"的闭环关键，须全量。

## §问题：MPC 足端力与 WBC 加速度不一致

只取 (4.34) 前 6 行（浮动基体），定义浮动基选择矩阵（4.41）：
$$
S_f=\begin{bmatrix}\mathbb I_{6\times6}&\mathbf 0\\\mathbf 0&\mathbf 0\end{bmatrix}\tag{4.41}
$$
左乘 $S_f$ 得躯干动力学（4.42）：
$$
S_fM\ddot{\boldsymbol q}+S_fC=S_fJ_c^T\,{}^{\mathcal O}\boldsymbol f_c\tag{4.42}
$$
〔作者洞见 ⑯〕（line 2040）：MPC 已得足端力 ${}^{\mathcal O}\boldsymbol f^{MPC}$，WBC 已得广义加速度 $\ddot{\boldsymbol q}^{cmd}$，但**直接代入 (4.42) 左右不相等**——因为 $\boldsymbol f^{MPC}$ 侧重"未来一段时间"，$\ddot{\boldsymbol q}^{cmd}$ 侧重"任务优先级"。两者口径不同 → 需协调。成书 `insight`：这正是控制链两环（MPC=预测层、WBC=瞬时层）目标不一致的本质，松弛 QP 是粘合剂。

## §松弛变量与带约束优化（于宪元 4.43–4.44）

给两者各加松弛变量（4.43）：
$$
\begin{cases}\ddot{\boldsymbol q}=\ddot{\boldsymbol q}^{cmd}+\begin{bmatrix}\boldsymbol\delta_{\ddot q}\\\boldsymbol 0\end{bmatrix}\\[2mm]{}^{\mathcal O}\boldsymbol f_c={}^{\mathcal O}\boldsymbol f^{MPC}+\boldsymbol\delta_f\end{cases}\tag{4.43}
$$
> **OCR 疑点 G**：(4.43) 上式 $\ddot{\boldsymbol q}^{cmd}$ 加的松弛仅作用于前 6 维（浮动基），故写成 $[\boldsymbol\delta_{\ddot q};\ \boldsymbol 0]$，$\boldsymbol\delta_{\ddot q}\in\mathbb R^6$。MinerU 中 $\boldsymbol\delta_{\ddot q}$ 与 $\boldsymbol\delta_{\dot q}$ 混排（目标函数 4.44/4.46 印作 $\delta_{\dot q}^T$），应统一为 $\boldsymbol\delta_{\ddot q}$（加速度松弛）。成书订正，`note` 盒标。

松弛越小越好，构造带约束优化（4.44）：
$$
\begin{aligned}
\min_{\boldsymbol\delta_{\ddot q},\boldsymbol\delta_f}\ &J=\boldsymbol\delta_{\ddot q}^TQ_1\boldsymbol\delta_{\ddot q}+\boldsymbol\delta_f^TQ_2\boldsymbol\delta_f\\
\text{s.t.}\ &S_fM\ddot{\boldsymbol q}+S_fC=S_fJ_c^T\,{}^{\mathcal O}\boldsymbol f_c\\
&\ddot{\boldsymbol q}=\ddot{\boldsymbol q}^{cmd}+[\boldsymbol\delta_{\ddot q};\boldsymbol 0]\\
&{}^{\mathcal O}\boldsymbol f_c={}^{\mathcal O}\boldsymbol f^{MPC}+\boldsymbol\delta_f\\
&\underline c_i\le C_i\cdot{}^{\mathcal O}\boldsymbol f_c^i\le\overline c_i
\end{aligned}\tag{4.44}
$$
〔作者洞见 ⑰〕（line 2052）权重矩阵 $Q_1,Q_2$ 含义="控制系统信任 WBC 多一些还是信任 MPC 多一些"；本文 $Q_1=\mathbb I$、$Q_2=0.005\,\mathbb I$（即更信任 MPC 足端力、允许 WBC 加速度多让步）。最后一行 $\underline c_i\le C_i\,{}^{\mathcal O}\boldsymbol f_c^i\le\overline c_i$ 即第 $i$ 支撑腿的摩擦锥/幅度线性约束（$C_i$ 复用 `ch:quad_mpc` 的摩擦金字塔约束矩阵）。

## §化为 QuadProg++ 标准 QP（4.45–4.58）

优化变量（4.45）$\boldsymbol{\mathcal X}=[\boldsymbol\delta_{\ddot q};\ \boldsymbol\delta_f]$。目标（4.46）：
$$
J=\boldsymbol{\mathcal X}^T\underbrace{\begin{bmatrix}Q_1&\mathbf 0\\\mathbf 0&Q_2\end{bmatrix}}_{0.5G}\boldsymbol{\mathcal X}=0.5\,\boldsymbol{\mathcal X}^TG\boldsymbol{\mathcal X}
$$
**等式约束**（4.47–4.50）：将 (4.43) 代入 (4.42)，取前 6 行（$M_f,\ddot{\boldsymbol q}_f^{cmd},C_f,J_{cf}^T$ 为各量前 6 行）：
$$
M_f(\ddot{\boldsymbol q}_f^{cmd}+\boldsymbol\delta_{\ddot q})+C_f=J_{cf}^T({}^{\mathcal O}\boldsymbol f^{MPC}+\boldsymbol\delta_f)\tag{4.48}
$$
整理（4.49–4.50）：
$$
\underbrace{[M_f\ \ -J_{cf}^T]}_{C_E^T}\underbrace{\begin{bmatrix}\boldsymbol\delta_{\ddot q}\\\boldsymbol\delta_f\end{bmatrix}}_{\boldsymbol{\mathcal X}}+\underbrace{(-J_{cf}^T\,{}^{\mathcal O}\boldsymbol f^{MPC}+C_f+M_f\ddot{\boldsymbol q}_f^{cmd})}_{c_e}=\boldsymbol 0\ \Rightarrow\ C_E^T\boldsymbol{\mathcal X}+c_e=\boldsymbol 0
$$
**不等式约束**（4.51–4.57）：$n_c$ 条支撑腿，${}^{\mathcal O}\boldsymbol f_c=[{}^{\mathcal O}\boldsymbol f_c^1;\cdots;{}^{\mathcal O}\boldsymbol f_c^{n_c}]\in\mathbb R^{3n_c}$，块对角约束 $\underline c_A\le C_A\,{}^{\mathcal O}\boldsymbol f_c\le\overline c_A$（4.53）。代入 ${}^{\mathcal O}\boldsymbol f_c={}^{\mathcal O}\boldsymbol f^{MPC}+\boldsymbol\delta_f$ 拆成两不等式（4.55）→ 整理（4.56–4.57）：
$$
\underbrace{\begin{bmatrix}\mathbf 0&-C_A\\\mathbf 0&C_A\end{bmatrix}}_{C_I^T}\boldsymbol{\mathcal X}+\underbrace{\begin{bmatrix}\overline c_A-C_A{}^{\mathcal O}\boldsymbol f^{MPC}\\C_A{}^{\mathcal O}\boldsymbol f^{MPC}-\underline c_A\end{bmatrix}}_{c_i}\ge\boldsymbol 0\ \Rightarrow\ C_I^T\boldsymbol{\mathcal X}+c_i\ge\boldsymbol 0
$$
**标准 QP**（4.58）：
$$
\min_{\boldsymbol{\mathcal X}}\ 0.5\,\boldsymbol{\mathcal X}^TG\boldsymbol{\mathcal X}\quad\text{s.t.}\ C_E^T\boldsymbol{\mathcal X}+c_e=\boldsymbol 0,\ \ C_I^T\boldsymbol{\mathcal X}+c_i\ge\boldsymbol 0
$$
> practice 盒：QuadProg++（C++ 二次型库），问题规模小故选它；与 `ch:quad_mpc` 的 qpOASES 对比。

## §反解关节扭矩（4.59–4.62）

QP 解出 $\boldsymbol\delta_{\ddot q},\boldsymbol\delta_f$ → 代 (4.43) 得 $\ddot{\boldsymbol q},{}^{\mathcal O}\boldsymbol f_c$。由 (4.34) 变形（虚拟关节扭矩 $\boldsymbol\tau_f=0$，4.59–4.61）：
$$
\begin{bmatrix}\mathbf 0_{6\times1}\\\boldsymbol\tau_j\end{bmatrix}=M(\boldsymbol q)\ddot{\boldsymbol q}+C(\boldsymbol q,\dot{\boldsymbol q})-J_c^T\,{}^{\mathcal O}\boldsymbol f_c\tag{4.61}
$$
最终电机扭矩指令（前馈 $\boldsymbol\tau_j$ + 关节级 PD，4.62）：
$$
\boldsymbol\tau^{cmd}=\boldsymbol\tau_j+k_p(\boldsymbol q_j^{cmd}-\boldsymbol q_j)+k_d(\dot{\boldsymbol q}_j^{cmd}-\dot{\boldsymbol q}_j)\tag{4.62}
$$
其中 $\boldsymbol q_j^{cmd},\dot{\boldsymbol q}_j^{cmd}$ 由 $\boldsymbol q^{cmd},\dot{\boldsymbol q}^{cmd}$ 剔除前 6 行得到。

〔作者洞见 ⑱〕这一步**闭合控制链**：MPC 给 $\boldsymbol f^{MPC}$ → WBC 给 $\ddot{\boldsymbol q}^{cmd}$/$\boldsymbol q^{cmd}$ → 松弛 QP 协调 → 逆动力学反解 $\boldsymbol\tau_j$ → 前馈+PD 成 $\boldsymbol\tau^{cmd}$ 发电机。成书章末 `insight` 总结全链。

---

# 三源差异 / 互补点（供融合）

1. **零空间数学地基**：个人笔记**独有完整拉格朗日乘子法证明**（伪逆为最小范数解）+ 阻尼最小二乘 + 正交投影几何解释；于宪元仅给结论式 (4.5)–(4.9)。→ 成书证明用个人笔记，编号对齐于宪元。
2. **方法谱系（NSP/WQP/HQP）**：个人笔记**独有**三方法综述（line 1145–1160）；于宪元只实现 NSP+松弛 QP。→ 成书 `history` 盒用个人笔记。
3. **VMC/ZMP**：个人笔记**独有**（line 1030–1061）；于宪元 §4 不含。→ 作为本章 `motivation`/反面教材。
4. **松弛优化 QP 全推导**：于宪元**独有**（4.41–4.62，含等式/不等式约束化简、QuadProg++ 标准化、扭矩反解）；个人笔记仅在四足应用节点到。→ 成书 §7 用于宪元，逐式 `\cite{yu2021quadruped}`。
5. **递推伪代码**：两源都有 (4.36)/line 1191，**互校 OCR**（疑点 E/F）——个人笔记的 $\dot J_1$ 正确补了于宪元的漏 dot；于宪元的 PD 行无粘连项更干净。→ 成书取两者各自正确部分。
6. **望远镜优先级例**：两源都有（个人笔记 line 1066 / 于宪元 line 1782），表述一致 → 直接用作 `insight`。
7. **子任务雅可比与转动误差变基**：于宪元**独有**(4.37)–(4.40)；个人笔记只给优先级表 → 成书 §6.4 用于宪元，与 `ch:quad_mpc` 角速度映射核对。

---

# OCR 可疑处清单（汇总，原印 vs 推断订正）

- **A**（于宪元 4.3）：$\sin(\pi-\dot q_2)=-\dot q_4/l^2$ 量纲不自洽；疑 MinerU 转写漏项。订正：仅作定性零空间运动反例，不照搬。
- **B**（于宪元 4.31 后）：加速度级递推起始印作 $\dot q_1$，应为 $\ddot q_1=J_1^+(\ddot x_1-\dot J_1\dot q)$。
- **C**（于宪元 4.37/4.38）：欧拉角速率→角速度映射矩阵 $E(\boldsymbol\Theta)$，须与 `ch:quad_mpc` 的 (3.20) 逐元素核对基/转置约定。
- **D**（于宪元 4.39/4.40）：花体坐标系下标 $\mathcal O/\mathcal B$ 混排、出现 ${}^{?}p_{com}$ 问号；统一世界系 $\mathcal O$/本体系 $\mathcal B$。
- **E**（两源 4.36 第 4 行）：于宪元印 $J_1^+(-J_1\dot q)$ 漏 dot，应 $-\dot J_1\dot q$（个人笔记正确）。
- **F**（个人笔记 line 1191 第 9 行）：PD 行末尾粘连多余 $\dot x_i^{cmd}$，应删（以于宪元 4.36 该行为准）。
- **G**（于宪元 4.44/4.46）：松弛变量目标函数印作 $\delta_{\dot q}^TQ_1\delta_{\ddot q}$，前一 $\delta_{\dot q}$ 应为 $\delta_{\ddot q}$（加速度松弛，二次型须同变量）。
- **H**（个人笔记 line 376 / 于宪元 4.7）：$J^+=J^T(JJ^T)^{-1}$ 为右伪逆（适用行满秩冗余 $N>M$）；与左伪逆 $J^+=(J^TJ)^{-1}J^T$ 区分，`note` 盒澄清避免读者误用。

---

# 作者洞见清单（供 `insight`/`remark` 盒，编号对应正文）

①VMC 把控制律设计降维成"想象机械元件"（敲门例）。②VMC 应用两关键 + 雅可比非定值。③ZMP 相对 CoG 判据考虑了惯性力合力。④正向唯一/逆向不唯一是冗余的起点。⑤零空间运动=关节动而末端静（低优先级免费搭车的物理基础）。⑥WBC 一句话定义（章首主线锚点）。⑦伪逆解在行空间、与零空间正交——后续一切的真正基石。⑧奇异点病态 → 阻尼最小二乘。⑨优先级本质=正交投影"尽力近似"。⑩望远镜例（为什么要优先级）。⑪"副作用/补偿"语义化代数项。⑫递推链最终输出=总关节指令。⑬NSP/WQP/HQP 谱系与本方案定位。⑭浮动基座为何需 6-DOF 虚拟关节。⑮优先级排序=物理因果链（接触→机身→摆动）。⑯MPC 与 WBC 目标口径不一致是松弛 QP 的动机。⑰$Q_1,Q_2$="信任 WBC 还是 MPC"（$Q_1=\mathbb I,Q_2=0.005\mathbb I$）。⑱松弛 QP+逆动力学闭合"MPC→WBC→扭矩"全链。
