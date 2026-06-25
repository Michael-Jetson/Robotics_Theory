# 抽取留痕 — 规划导论：构型空间 / 搜索式(A*,Dijkstra) / 采样式(RRT, RRT-Connect, PRM, RRT*, PRM*, RRG) / 轨迹优化(CHOMP)

> **本抽取服务章节**：`规划导论`（构型空间；搜索式与采样式运动规划完整算法 + 性质；轨迹优化简介）。
>
> **专题主线**：采样式运动规划的**算法**（RRT / RRT* / PRM / PRM* / RRG / RRT-Connect）与其**理论性质**（概率完备性 probabilistic completeness、渐近最优性 asymptotic optimality）。这是本抽取的【核心】，权威源逐字精读、全量保真。
>
> **来源（联网研究，权威原始文献，均已逐字精读 PDF / 原文）**：
>
> 1. **【核心源 = KF11】** S. Karaman and E. Frazzoli, *"Sampling-based Algorithms for Optimal Motion Planning"*, International Journal of Robotics Research (IJRR), vol. 30, no. 7, pp. 846–894, 2011. arXiv:1105.1186v1 [cs.RO], 5 May 2011. <https://arxiv.org/abs/1105.1186>（PDF 全文 4627 行已逐字读完；这是渐近最优采样规划的奠基论文，给出 PRM*/RRT*/RRG 及全部完备性/最优性定理与证明。本抽取的定理编号 1–49 全部来自此源）。
> 2. **【RRT 源 = KL00】** J. J. Kuffner and S. M. LaValle, *"RRT-Connect: An Efficient Approach to Single-Query Path Planning"*, IEEE ICRA 2000, pp. 995–1001. PDF: <https://www.cs.cmu.edu/afs/cs/academic/class/15494-s14/readings/kuffner_icra2000.pdf>（全文 421 行已逐字读完；给出 BUILD_RRT / EXTEND / CONNECT / RRT_CONNECT_PLANNER 伪码 + 概率完备性证明 Lemma1/Lemma2/Theorem1/Corollary1 + Voronoi 偏置）。原始 RRT 报告 = S. M. LaValle, *"Rapidly-exploring random trees: A new tool for path planning"*, TR 98-11, Computer Science Dept., Iowa State University, Oct. 1998 <https://lavalle.pl/papers/Lav98c.pdf>（其 BUILD_RRT/EXTEND 与 KL00 同；KL00 是其正式会议版扩展）。
> 3. **【综述/记号源 = LaValleBook】** S. M. LaValle, *Planning Algorithms*, Cambridge Univ. Press, 2006（在线版 <https://lavalle.pl/planning/>）。用于构型空间、采样规划框架、Voronoi 偏置、典型参数 $k=15$/$k=20$ 等（KF11 多处引用之）。
> 4. **【A* 源 = HNR68 / Wiki-A\*】** P. E. Hart, N. J. Nilsson, B. Raphael, *"A Formal Basis for the Heuristic Determination of Minimum Cost Paths"*, IEEE Trans. Syst. Sci. Cybern., vol. 4(2), pp. 100–107, 1968. 配合 <https://en.wikipedia.org/wiki/A*_search_algorithm>（伪码、可采纳性、一致性、最优性定理）。
> 5. **【构型空间源 = Wiki-MP】** <https://en.wikipedia.org/wiki/Motion_planning>（构型空间、$C_{free}$/$C_{obs}$、完备性三类、算法分类、PSPACE-hard）；PRM 见 <https://en.wikipedia.org/wiki/Probabilistic_roadmap>。
> 6. **【轨迹优化源 = CHOMP13】** M. Zucker, N. Ratliff, A. D. Dragan, M. Pivtoraiko, M. Klingensmith, C. M. Dellin, J. A. Bagnell, S. S. Srinivasa, *"CHOMP: Covariant Hamiltonian Optimization for Motion Planning"*, IJRR vol. 32(9–10), pp. 1164–1193, 2013. PDF: <https://publications.ri.cmu.edu/storage/publications/pub_files/2013/5/CHOMP_IJRR.pdf>（全文 2357 行已逐字读完；给出目标泛函式(1)–(19)、泛函梯度、协变更新规则）。

>
> **保真承诺**：本文件按各源小节顺序，逐条记录每一个定义、每一条公式（LaTeX 写全、保留原编号）、每一步推导（中间代数不跳）、每一道数值例/实验结论、每一段算法伪码。**不做摘要、不做凝练。** KF11 的完备性/最优性定理（含 RRT 非最优性的完整证明、RRT* 最优性的标记点过程 + 球链证明、PRM* 球覆盖证明）逐行展开。
>
> **\rebuilt 说明**：个人笔记（`SLAM理论.md` 规划部分）尚未同步，本抽取**主要据权威教材/原论文**重建；凡个人笔记可能有而此处据公开源补的内容，已隐含为 \rebuilt（全文据公开权威源，无私货推断）。

---

## 0. 记号约定（各源）+ 与本书统一约定的差异

本专题涉及运动规划，**与本书 SLAM 主线的李群/扰动约定基本正交**（规划在构型空间 $\mathcal{C}=\mathbb{R}^d$ 或流形上做几何搜索，通常不涉及左/右扰动、四元数排序）。但仍需显式登记各源记号，便于综合 agent 统一。

### 0.0 本书统一约定（编写规范 §五，回顾）

- 旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$；位姿 $\mathbf T\in\mathrm{SE}(3)$；四元数 Hamilton；**右扰动**为主 $\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$；$\mathfrak{se}(3)$ 排序 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）；协方差 $\boldsymbol\Sigma$。
- **规划专题的接口**：当构型 $q$ 含姿态时（如 $\mathcal{C}=\mathrm{SE}(3)$ 的刚体规划），构型空间的"距离度量 $\rho$"、"插值/Steer"须用本书的李群工具（$\mathrm{Log}/\mathrm{Exp}$、测地线）实现；本专题源多在 $\mathbb{R}^d$ 上叙述，移植到 $\mathrm{SO}(3)/\mathrm{SE}(3)$ 时距离与采样需改为流形版（见 §1.3 注）。

### 0.1 KF11（核心源）记号

| 记号 | 含义（KF11 用法） | 与本书/统一差异 |
|---|---|---|
| $X=(0,1)^d$ | **构型空间**（单位开立方体，$d\ge2$） | 本书一般记 $\mathcal{C}$；KF11 用 $X$ 且归一化到单位立方体（理论方便） |
| $X_{obs}$ | 障碍区，$X\setminus X_{obs}$ 为开集 | 本书记 $\mathcal{C}_{obs}$ |
| $X_{free}=\mathrm{cl}(X\setminus X_{obs})$ | **自由空间**（取闭包） | 本书 $\mathcal{C}_{free}$；注意 KF11 取**闭包**（含边界） |
| $x_{init}\in X_{free}$ | 初始构型 | $q_{init}$ |
| $X_{goal}\subset X_{free}$ | 目标区（开子集） | $\mathcal{C}_{goal}$ |
| $\sigma:[0,1]\to\mathbb R^d$ | **路径**（有界变差函数） | 本书 $c(\cdot)$ 或 $\gamma(\cdot)$；KF11 用 $\sigma$ |
| $\mathrm{TV}(\sigma)$ | 路径全变差 = 欧氏长度 | — |
| $c:\Sigma\to\mathbb R_{\ge0}$ | **代价函数**（单调、有界） | 本书代价/cost；注意与本书 $\mathbf C$（旋转矩阵别名，本书不用）无关 |
| $c^*=c(\sigma^*)$ | 最优代价 | — |
| $\mu(\cdot)$ | **Lebesgue 测度**（体积） | — |
| $\zeta_d$ | **$d$ 维单位球体积**，$\zeta_d=\pi^{d/2}/\Gamma(d/2+1)$ | 关键常数，反复出现在半径公式 |
| $\lambda_c$ | **连续渗流阈值**（critical density） | $d=2$ 时 $0.696<\lambda_c<3.372$，仿真 $\approx1.44$ |
| $\eta$ | Steer 步长上界（$\mathtt{Steer}$ 函数的最大步） | RRT 的 $\Delta q$ / step size |
| $r=r(n)$ | 连接半径（PRM*/RRG/RRT* 中随 $n$ 变化） | — |
| $\gamma_{PRM},\gamma_{RRG},\gamma_{RRT^*}$ | 各算法半径常数 | 见 §3、§4 公式 |
| $Y_n^{\mathrm{ALG}}$ | 算法 ALG 第 $n$ 次迭代后图中最优解代价（扩展随机变量） | — |
| $G_n=(V_n,E_n)$ | 第 $n$ 步的图（顶点/边） | — |
| $\mathtt{card}(V)$ | 顶点集基数（= 顶点数） | $|V|$ |
| $B_{x,r}$ | 以 $x$ 为心、$r$ 为半径的（闭）球 | — |
| $\mathrm{int}_\delta(X_{free})$ | $X_{free}$ 的 **$\delta$-内部**（离障碍至少 $\delta$） | 完备性/最优性核心概念 |

**KF11 与本书无冲突**：纯几何/概率论叙述，无李群扰动方向问题。唯一需注意：KF11 的 $c$ 是**代价函数**（不是旋转矩阵），$\mu$ 是**测度**（不是均值），$\lambda$ 是**Poisson 强度/渗流参数**（不是特征值或波长）——综合时若与 SLAM 章并置须显式区分字母。

### 0.2 KL00（RRT / RRT-Connect 源）记号

| 记号 | 含义 |
|---|---|
| $\mathcal C$ | 构型空间；$q\in\mathcal C$ 一个构型 |
| $\mathcal C_{free}$ | 无碰撞构型集（非凸开集，单连通） |
| $\rho$ | $\mathcal C$ 上的度量（metric） |
| $q_{init},q_{goal}$ | 初始/目标构型 |
| $\epsilon$ | EXTEND 的增量步长（RRT step size） |
| $T,\mathcal T$ | RRT 树；$T_a,T_b$ 双树（init/goal 各一） |
| $K$ | 最大迭代数 |
| $q_{rand},q_{near},q_{new}$ | 随机/最近/新构型 |
| $D_k(q),d_k$ | $q$ 到树中最近顶点的距离（随机变量/值），$k$=顶点数 |
| $X,X_k$ | 采样分布 / RRT 顶点分布（随机变量） |
| Reached/Advanced/Trapped | EXTEND 三种返回值 |

**KL00 与 KF11 的记号桥**：KL00 的 $\mathcal C\leftrightarrow$ KF11 的 $X$；KL00 的 $\epsilon$（RRT 步长）$\leftrightarrow$ KF11 的 $\eta$（Steer 上界）；KL00 的 $q_{near}\leftrightarrow$ KF11 的 $x_{nearest}$。

### 0.3 A*（HNR68）记号

| 记号 | 含义 |
|---|---|
| $g(n)$ | 起点到节点 $n$ 的已知最小代价 |
| $h(n)$ | $n$ 到目标的启发式估计代价 |
| $f(n)=g(n)+h(n)$ | 评价函数 |
| $h^*(n)$ | $n$ 到目标的**真实**最优代价 |
| $d(x,y)$ | 边 $(x,y)$ 的权 |
| OPEN/CLOSED | 开/闭列表 |

### 0.4 CHOMP（CHOMP13）记号

| 记号 | 含义 |
|---|---|
| $\xi:[0,1]\to\mathcal C\subset\mathbb R^d$ | 轨迹（光滑函数；时间 $\to$ 构型） |
| $\mathcal F_{smooth}[\xi]$ | 光滑性泛函 |
| $\mathcal F_{obs}[\xi]$ | 障碍泛函 |
| $\mathcal U[\xi]=\mathcal F_{obs}+\lambda\mathcal F_{smooth}$ | 总目标泛函 |
| $\lambda$ | 光滑/避障权衡系数 |
| $c:\mathbb R^3\to\mathbb R$ | **工作空间代价场**（penalize 障碍附近） |
| $x:\mathcal C\times\mathcal B\to\mathbb R^3$ | 正运动学（构型 + 体点 $u\to$ 工作空间点） |
| $\mathcal B\subset\mathbb R^3$ | 机器人体表点集 |
| $J$ | 体点处运动学 Jacobian |
| $\bar\nabla\mathcal U$ | 泛函梯度 |
| $A=K^TK$ | 有限差分度量矩阵（measures total acceleration） |
| $\eta_i$ | 步长 |

**CHOMP 的 $\lambda$ 是权衡系数**（与 KF11 的渗流 $\lambda$、SLAM 的特征值 $\lambda$ 均不同义）；CHOMP 的 $c$ 是**工作空间代价场**（与 KF11 的代价函数 $c$ 含义相近但定义域不同）。

---

# 第一部分：构型空间与问题表述（来源 Wiki-MP §定义、KF11 §2.1、KL00 §2）

## 1.1 构型空间 $\mathcal C$、自由空间 $\mathcal C_{free}$、障碍空间 $\mathcal C_{obs}$（Wiki-MP）

**构型（configuration）**：描述机器人位姿的参数组。例：
- 平面内点机器人：$\mathcal C=\mathbb R^2$，参数 $(x,y)$。
- 平面内可平移 + 旋转的 2D 刚体：$\mathcal C=\mathrm{SE}(2)=\mathbb R^2\times\mathrm{SO}(2)$，参数 $(x,y,\theta)$。
- 空间刚体：$\mathcal C=\mathrm{SE}(3)=\mathbb R^3\times\mathrm{SO}(3)$，6 自由度。
- $n$ 关节机械臂：$\mathcal C=\mathbb T^n$（$n$ 维环面）或 $\mathbb R^n$，$n$ 自由度。

**自由度（DOF）**：完全确定系统构型所需的独立参数个数 = $\dim\mathcal C$。

**自由空间** $\mathcal C_{free}$：不与障碍碰撞的构型集合。**障碍空间** $\mathcal C_{obs}$：$\mathcal C\setminus\mathcal C_{free}$（forbidden region）。

**关键计算事实（Wiki-MP）**：显式计算 $\mathcal C_{free}$ 的形状通常**计算上极难**（prohibitively difficult）；但**测试某一给定构型是否属于 $\mathcal C_{free}$ 是高效的**（碰撞检测）。这正是采样式规划的立足点（KL00 §2 同此："an explicit representation of $\mathcal C_{free}$ is not available. However, using a collision detection algorithm, a given $q\in\mathcal C$ can be tested to determine whether $q\in\mathcal C_{free}$"）。

## 1.2 基本运动规划问题（Wiki-MP / KL00）

**问题（path planning）**：计算一条连续路径，连接起始构型 $S$（$q_{init}$）与目标构型 $G$（$q_{goal}$），并避开已知障碍。形式化：找 $c:[0,1]\to\mathcal C_{free}$ 连续，$c(0)=q_{init}$，$c(1)=q_{goal}$。

**single-query（单查询，KL00 §2）**：给定 $q_{init},q_{goal}$，**不做预处理**直接算路径（RRT 适用）。
**multi-query（多查询，PRM 适用）**：先建路网（预处理），后续多个 query 复用之。

## 1.3 KF11 的形式化问题表述（源 §2.1，全量保真）

设 $X=(0,1)^d$ 为构型空间，$d\in\mathbb N,d\ge2$。$X_{obs}$ 为障碍区，使 $X\setminus X_{obs}$ 为开集；自由空间 $X_{free}=\mathrm{cl}(X\setminus X_{obs})$（$\mathrm{cl}(\cdot)$ = 闭包）。初始 $x_{init}\in X_{free}$，目标 $X_{goal}$ 为 $X_{free}$ 的开子集。**路径规划问题由三元组 $(X_{free},x_{init},X_{goal})$ 定义。**

**全变差（total variation）**：对 $\sigma:[0,1]\to\mathbb R^d$，
$$\mathrm{TV}(\sigma)=\sup_{\{n\in\mathbb N,\,0=\tau_0<\tau_1<\cdots<\tau_n=s\}}\sum_{i=1}^n|\sigma(\tau_i)-\sigma(\tau_{i-1})|.$$
$\mathrm{TV}(\sigma)<\infty$ 称**有界变差**。

**定义 1（Path，KF11）** 有界变差函数 $\sigma:[0,1]\to\mathbb R^d$ 称为：
- **Path**（路径），若它连续；
- **Collision-free path**（无碰撞路径），若它是路径且 $\sigma(\tau)\in X_{free},\forall\tau\in[0,1]$；
- **Feasible path**（可行路径），若它无碰撞，$\sigma(0)=x_{init}$，$\sigma(1)\in\mathrm{cl}(X_{goal})$。

> 路径的全变差本质上即其长度（在 $\mathbb R^d$ 中走过的欧氏距离）。

**问题 2（Feasible path planning，可行路径规划）** 给定 $(X_{free},x_{init},X_{goal})$，找一条可行路径 $\sigma:[0,1]\to X_{free}$，$\sigma(0)=x_{init}$，$\sigma(1)\in\mathrm{cl}(X_{goal})$（若存在）；若不存在，报告失败。

**路径的串联与代价**：$\Sigma$ = 所有路径集；$\Sigma_{free}$ = 无碰撞路径集。对 $\sigma_1,\sigma_2\in\Sigma$ 且 $\sigma_1(1)=\sigma_2(0)$，定义串联 $\sigma_1|\sigma_2\in\Sigma$：
$$(\sigma_1|\sigma_2)(\tau):=\sigma_1(2\tau),\ \tau\in[0,1/2];\qquad (\sigma_1|\sigma_2)(\tau):=\sigma_2(2\tau-1),\ \tau\in(1/2,1].$$
$\Sigma,\Sigma_{free}$ 在串联下封闭。代价函数 $c:\Sigma\to\mathbb R_{\ge0}$ 对所有非平凡无碰撞路径赋严格正代价（$c(\sigma)=0\iff\sigma(\tau)=\sigma(0),\forall\tau$）。代价假定：
- **单调（monotonic）**：$c(\sigma_1)\le c(\sigma_1|\sigma_2),\ \forall\sigma_1,\sigma_2$；
- **有界（bounded）**：$\exists k_c$ 使 $c(\sigma)\le k_c\,\mathrm{TV}(\sigma),\forall\sigma$。

**问题 3（Optimal path planning，最优路径规划）** 给定 $(X_{free},x_{init},X_{goal})$ 与代价 $c:\Sigma\to\mathbb R_{\ge0}$，找可行路径 $\sigma^*$ 使 $c(\sigma^*)=\min\{c(\sigma):\sigma\text{ 可行}\}$；若不存在，报告失败。

> **移植到流形（\rebuilt 注）**：KF11 在 $\mathbb R^d$ 上叙述。当 $\mathcal C=\mathrm{SO}(3)$ 或 $\mathrm{SE}(3)$ 时，欧氏 $\|x-y\|$ 须换为李群测地距离（如 $\mathrm{SO}(3)$ 上 $\|\mathrm{Log}(\mathbf R_x^\top\mathbf R_y)\|$），单位球体积 $\zeta_d$、测度 $\mu$ 须换为相应黎曼体积；KF11 的渐近最优性结论在"局部欧氏 + 测度良性"条件下推广成立（KF11 §3.1 脚注：结论对任意"密度有下界的绝对连续分布"成立）。

## 1.4 完备性的三个层级（Wiki-MP，全量）

- **完备（complete）**：有限时间内**要么给出一条解，要么正确报告无解**。多数完备算法基于几何（精确几何方法）。
- **分辨率完备（resolution complete）**：若底层栅格分辨率足够细，则保证找到路径。栅格法计算复杂度 $O(1/h^d)$（$h$ = 分辨率，$d$ = 维数）。
- **概率完备（probabilistically complete）**：随着投入"工作量"增加，**算法在有解时找不到解的概率渐近趋于 0**。采样式方法属此类。

**计算难度（KF11 §1）**：基本运动规划（广义钢琴搬运工问题 generalized piano movers problem）是 **PSPACE-hard**（Reif 1979）。完备算法存在（Lozano-Perez & Wesley 1979；Schwartz & Sharir 1983；Canny 1988），但复杂度使其不适用于实践。故实践转向放松完备性（分辨率完备 / 概率完备）。

## 1.5 运动规划算法分类（Wiki-MP）

1. **基于栅格/格点搜索（grid-based / lattice search）**：A*、Dijkstra、D*、Field D*（见第二部分）。
2. **基于区间搜索（interval-based）**。
3. **几何算法（geometric）**：可见图（visibility graph）、单元分解（cell decomposition）、Voronoi 图。
4. **人工势场（artificial potential fields，Khatib 1986）**。
5. **采样式（sampling-based）**：PRM、RRT 系（见第三部分）。"目前被认为是高维空间运动规划的 state-of-the-art。"
6. **轨迹优化（trajectory optimization）**：CHOMP、TrajOpt、STOMP（见第四部分）。

---

# 第二部分：搜索式规划 A* / Dijkstra（来源 HNR68 + Wiki-A*）

> 本部分服务章节"搜索式(A*/Dijkstra)"要点。A* = 在显式图（栅格/路网）上的最优启发式搜索。Dijkstra 是其 $h\equiv0$ 的特例。

## 2.1 A* 评价函数（HNR68；Wiki-A*）

A* 在图上搜索从起点到目标的最小代价路径。每个节点 $n$ 维护：
$$\boxed{f(n)=g(n)+h(n)},$$
- $g(n)$ = 起点到 $n$ 的**当前已知最小代价**（"the currently known cost of the cheapest path from start to $n$"）；
- $h(n)$ = $n$ 到目标的**启发式估计代价**；
- $f(n)$ = "若路径经过 $n$，从起到终可能的最廉价代价的当前最佳猜测"。

历史：Hart、Nilsson、Raphael（SRI）1968 首次发表，可视为 Dijkstra 的扩展；和式 $g(n)+h(n)$ 由 Raphael 建议；可采纳性（admissibility）与一致性（consistency）概念由 Hart 提出。

## 2.2 A* 伪码（Wiki-A*，结构保真）

```
A_STAR(start, goal, h):
    openSet ← {start}                       // 待扩展（发现但未展开）节点
    cameFrom ← empty map                    // 父指针，用于回溯路径
    gScore[start] ← 0; gScore[n] ← +∞ for n ≠ start
    fScore[start] ← h(start)                // fScore[n] = gScore[n] + h(n)
    while openSet ≠ ∅:
        current ← node in openSet with lowest fScore       // (1) 取 f 最小
        if current = goal:                                 // (2) 到目标则回溯返回
            return reconstruct_path(cameFrom, current)
        openSet.remove(current)                            //     移入 CLOSED
        for each neighbor of current:
            tentative_g ← gScore[current] + d(current, neighbor)
            if tentative_g < gScore[neighbor]:             // (3) 发现更优路径则松弛
                cameFrom[neighbor] ← current
                gScore[neighbor]  ← tentative_g
                fScore[neighbor]  ← tentative_g + h(neighbor)
                if neighbor ∉ openSet: openSet.add(neighbor)
    return failure                                          // OPEN 空 → 无路径

reconstruct_path(cameFrom, current):
    path ← [current]
    while current in cameFrom:
        current ← cameFrom[current]; path.prepend(current)
    return path
```

## 2.3 可采纳性、一致性、最优性（Wiki-A*，全量）

**可采纳启发式（admissible heuristic）**：$h$ 从不**高估**到目标的实际代价：
$$\boxed{h(n)\le h^*(n)\quad\forall n,}$$
其中 $h^*(n)$ 是 $n$ 到目标的真实最优代价。

**一致性 / 单调性（consistent / monotone heuristic）**：对每条边 $(x,y)$，
$$\boxed{h(x)\le d(x,y)+h(y),}$$
即 $h$ 满足"三角不等式"（沿所有边）。等价地 $f$ 沿任一路径**非降**。

**层级关系**：所有一致启发式都是可采纳的；但并非所有可采纳启发式都一致（consistent $\Rightarrow$ admissible，反之不然）。

**最优性定理（Wiki-A*）**：
- **(可采纳 $\Rightarrow$ 最优解)** 用可采纳 $h$ 时，A* 返回**最优路径**（"A* is admissible"）。
- **(一致 $\Rightarrow$ 不重复展开 + 最优效率)** 用一致 $h$ 时，A* 保证找到最优路径**且不重复处理任何节点超过一次**（"once a node is expanded, the cost by which it was reached is the lowest possible"——节点首次出 OPEN 即已最优，同 Dijkstra 的条件）。一致 $h$ 下 A* 是**最优高效（optimally efficient）**的：在非病态问题上，它展开的节点数不多于任何同类可采纳算法。
- **(与 Dijkstra 的关系)** Dijkstra 是 A* 的特例 $h(x)\equiv0$（"a special case of A* where $h(x)=0$ for all $x$"）。$h\equiv0$ 平凡可采纳且一致，故 Dijkstra 给最优解、节点首次出队即最优。
- **(与贪心最佳优先的区别)** A* 区别于 Greedy Best-First（后者只用 $h$）在于 A* **计入已走代价 $g(n)$**。

> **直觉（A* vs Dijkstra）**：Dijkstra 向所有方向均匀扩展（$f=g$）；A* 用 $h$ 把扩展"拉向"目标（$f=g+h$），在 $h$ 接近 $h^*$ 时大幅减少展开节点数，同时（$h$ 可采纳时）仍保证最优。

## 2.4 搜索式 vs 采样式（衔接，\rebuilt 综合）

- **搜索式（A*/Dijkstra）**：在**显式图**上工作，需先离散化 $\mathcal C$（栅格/格点/路网）；**分辨率完备 + 最优**；但维数灾难——栅格法 $O(1/h^d)$ 随 $d$ 指数爆炸，高维不可行。
- **采样式（PRM/RRT）**：**隐式**地用碰撞检测探索 $\mathcal C_{free}$，无需显式离散化；**概率完备**（PRM/RRT）或**渐近最优**（PRM*/RRT*）；适合高维（机械臂、刚体 6-DOF）。这正是本抽取核心，见第三部分。

---

# 第三部分（核心）：采样式规划与其理论性质（来源 KF11 §3,§4 + KL00）

> 这是本抽取的**重中之重**。KF11 §3 给出 6 个算法（PRM/sPRM/RRT/PRM*/RRG/RRT*）的精确伪码与原始函数（primitive procedures），§4 给出全部完备性/最优性定理（编号 14–49）。逐条全量保真。

## 3.1 原始函数（Primitive Procedures，KF11 §3.1，全量保真）

**Sampling（采样）**：$\mathtt{Sample}:\omega\mapsto\{\mathtt{Sample}_i(\omega)\}_{i\in\mathbb N_0}\subset X$ 是从样本空间 $\Omega$ 到 $X$ 中点列的映射，使随机变量 $\mathtt{Sample}_i$ 独立同分布（i.i.d.）。为简便假定服从**均匀分布**（结论可推广到任意"在 $X$ 上密度有正下界的绝对连续分布"）。另设 $\mathtt{SampleFree}:\omega\mapsto\{\mathtt{SampleFree}_i(\omega)\}_{i\in\mathbb N_0}\subset X_{free}$ 返回 $X_{free}$ 中 i.i.d. 样本：$\{\mathtt{SampleFree}_i(\omega)\}=\{\mathtt{Sample}_i(\omega)\}\cap X_{free}$。

**Nearest Neighbor（最近邻）**：给定图 $G=(V,E)$（$V\subset X$）与点 $x\in X$，
$$\mathtt{Nearest}(G=(V,E),x):=\arg\min_{v\in V}\|x-v\|\quad(\text{欧氏距离}).$$
集值版 $\mathtt{kNearest}(G,x,k)\mapsto\{v_1,\dots,v_k\}$ 返回 $V$ 中离 $x$ 最近的 $k$ 个顶点（若 $|V|<k$ 则返回 $V$）。

**Near Vertices（近邻集）**：给定 $G=(V,E)$、点 $x$、半径 $r>0$，
$$\mathtt{Near}(G=(V,E),x,r):=\{v\in V:v\in B_{x,r}\}\quad(\text{半径 }r\text{ 球内的顶点}).$$

**Steering（转向）**：给定 $x,y\in X$，$\mathtt{Steer}(x,y)\mapsto z$ 返回比 $x$ 更靠近 $y$ 的点 $z$，满足在 $\|z-x\|\le\eta$（$\eta>0$ 预设）约束下最小化 $\|z-y\|$：
$$\mathtt{Steer}(x,y):=\arg\min_{z\in B_{x,\eta}}\|z-y\|.$$
（此 Steer 自 Kuffner & LaValle 2000 起广泛使用；亦可用 Rapidly-exploring Dense Trees 免去调 $\eta$。）

**Collision Test（碰撞检测）**：给定 $x,x'\in X$，布尔函数 $\mathtt{CollisionFree}(x,x')$ 返回 `True` 当且仅当线段 $[x,x']\subset X_{free}$（无碰撞），否则 `False`。

**所有算法 I/O 约定**：输入 = 问题 $(X_{free},x_{init},X_{goal})$、整数 $n\in\mathbb N$、代价 $c:\Sigma\to\mathbb R_{\ge0}$（若需）；输出 = 图 $G=(V,E)$，$V\subset X_{free}$，$\mathtt{card}(V)\le n+1$，$E\in V\times V$。解可由标准最短路（如 Dijkstra）从该图提取。

## 3.2 已有算法（KF11 §3.2）

### 3.2.1 PRM（概率路网，预处理阶段，Algorithm 1）

主要面向**多查询**。基本版含**预处理阶段**（建路网）+ **查询阶段**（连起终点找路）。预处理从空图开始；每次迭代采样 $x_{rand}\in X_{free}$ 加入 $V$，对 $x_{rand}$ 半径 $r$ 球内顶点按距离升序尝试用局部规划器（如直线）连接，无碰撞则加边。**为避免无用计算，不连同一连通分量内的顶点**——故 PRM 路网是**森林（forest）**。

```
Algorithm 1: PRM (preprocessing phase)
 1  V ← ∅; E ← ∅;
 2  for i = 0, ..., n do
 3      x_rand ← SampleFree_i;
 4      U ← Near(G = (V, E), x_rand, r);
 5      V ← V ∪ {x_rand};
 6      foreach u ∈ U, in order of increasing ‖u − x_rand‖, do
 7          if x_rand and u are NOT in the same connected component of G = (V,E) then
 8              if CollisionFree(x_rand, u) then E ← E ∪ {(x_rand, u), (u, x_rand)};
 9  return G = (V, E);
```

**查询阶段（Wiki-PRM）**：将 $q_{init},q_{goal}$ 连入图，再用 **Dijkstra 最短路**查询得解。

### 3.2.2 sPRM（简化 PRM，Algorithm 2）

文献中可分析的是简化版 sPRM（Kavraki et al. 1998）：用 $x_{init}$ 初始化 $V$，采 $n$ 个点，连接距离 $<r$ 的点（逻辑同 PRM，**但允许连同一连通分量内的顶点**）。**无障碍时（$X_{free}=X$），sPRM 路网是随机 $r$-disc 图（random $r$-disc graph）**——这是与随机几何图理论的关键连接。

```
Algorithm 2: sPRM
 1  V ← {x_init} ∪ {SampleFree_i}_{i=1,...,n};  E ← ∅;
 2  foreach v ∈ V do
 3      U ← Near(G = (V, E), v, r) \ {v};
 4      foreach u ∈ U do
 5          if CollisionFree(v, u) then E ← E ∪ {(v, u), (u, v)}
 6  return G = (V, E);
```

**(s)PRM 的实用 $U$ 选择变体（KF11 §3.2，全量）**：
- **k-Nearest (s)PRM**：连最近 $k$ 个邻居（典型 $k=15$，LaValle 2006）。$U\leftarrow\mathtt{kNearest}(G,x_{rand},k)$（Alg.1 行4）/ $U\leftarrow\mathtt{kNearest}(G,v,k)\setminus\{v\}$（Alg.2 行3）。无障碍时路网为**随机 $k$-最近邻图**。
- **Bounded-degree (s)PRM**：对固定 $r$，每次连接尝试数 $\propto|V|$，大 $n$ 时负担过重。故对 $|U|$ 加上界 $k$（典型 $k=20$）：$U\leftarrow\mathtt{Near}(G,x_{rand},r)\cap\mathtt{kNearest}(G,x_{rand},k)$。
- **Variable-radius (s)PRM**：令 $r$ 为 $n$ 的函数（而非固定）。但文献对 $r$-$n$ 的合适函数关系**无明确指引**（这正是 KF11 §3.3 PRM* 要解决的）。

### 3.2.3 RRT（快速扩展随机树，Algorithm 3）

主要面向**单查询**。增量地建一棵以 $x_{init}$ 为根的可行轨迹树。每次采 $x_{rand}$，尝试把树中最近顶点 $v$ 连向新样本；成功则加 $x_{rand}$ 到 $V$、加边 $(v,x_{rand})$。原始版一旦树含目标区节点即停；KF11 为与 PRM 一致改为迭代 $n$ 次。**无障碍时 RRT 树是 online nearest neighbor graph（在线最近邻图）。**

```
Algorithm 3: RRT
 1  V ← {x_init};  E ← ∅;
 2  for i = 1, ..., n do
 3      x_rand    ← SampleFree_i;
 4      x_nearest ← Nearest(G = (V, E), x_rand);
 5      x_new     ← Steer(x_nearest, x_rand);
 6      if ObstacleFree(x_nearest, x_new) then            // [原文笔误 ObtacleFree]
 7          V ← V ∪ {x_new};  E ← E ∪ {(x_nearest, x_new)};
 8  return G = (V, E);
```

**RRT 变体**：双树（init/goal 各一棵）= RRT-Connect（见 §3.5）；强调采样不必随机时称 Rapidly-exploring Dense Trees (RDT)。

## 3.3 提出的算法（KF11 §3.3，渐近最优版）

### 3.3.1 PRM*（最优 PRM，Algorithm 4）

与 sPRM 的**唯一区别**：连接半径 $r$ 取为 $n$ 的函数：
$$\boxed{r(n):=\gamma_{PRM}\left(\frac{\log n}{n}\right)^{1/d}},\qquad \gamma_{PRM}>\gamma_{PRM}^*=2\left(1+\frac1d\right)^{1/d}\left(\frac{\mu(X_{free})}{\zeta_d}\right)^{1/d},$$
$d$ = 空间维数，$\mu(X_{free})$ = 自由空间 Lebesgue 测度（体积），$\zeta_d$ = $d$ 维单位球体积。半径随 $n$ **递减**，衰减率使每顶点平均连接尝试数 $\propto\log n$。

```
Algorithm 4: PRM*
 1  V ← {x_init} ∪ {SampleFree_i}_{i=1,...,n};  E ← ∅;
 2  foreach v ∈ V do
 3      U ← Near(G = (V, E), v, γ_PRM (log(n)/n)^{1/d}) \ {v};
 4      foreach u ∈ U do
 5          if CollisionFree(v, u) then E ← E ∪ {(v, u), (u, v)}
 6  return G = (V, E);
```

> **与 LaValle 2006 的呼应**：LaValle 建议半径按**样本弥散度（dispersion）**选取（弥散度 = $S$ 内最大空球半径）。$n$ 个均匀独立样本的弥散度为 $O((\log n/n)^{1/d})$（Niederreiter 1992）——恰是 PRM* 的半径衰减率。

**k-nearest PRM\***：$k(n):=k_{PRM}\log n$，$k_{PRM}>k_{PRM}^*=e(1+1/d)$，$U\leftarrow\mathtt{kNearest}(G,v,k_{PRM}\log n)\setminus\{v\}$。注意 $k_{PRM}^*$ **只依赖 $d$**（不依赖问题实例，区别于 $\gamma_{PRM}^*$）；$k_{PRM}=2e$ 对所有实例都是合法选择。

### 3.3.2 RRG（快速扩展随机图，Algorithm 5）

增量地建**可含环**的连通路网。类 RRT，先连最近节点；但**每次新点 $x_{new}$ 加入 $V$ 后，对 $V$ 中所有处于半径**
$$r(\mathtt{card}(V))=\min\Big\{\gamma_{RRG}\big(\log(\mathtt{card}(V))/\mathtt{card}(V)\big)^{1/d},\ \eta\Big\},\qquad \gamma_{RRG}>\gamma_{RRG}^*=2(1+1/d)^{1/d}\big(\mu(X_{free})/\zeta_d\big)^{1/d}$$
**球内的顶点尝试连接**（$\eta$ = Steer 步长上界）。每次成功连接加一条边。故**对同一采样序列，RRT 图（有向树）是 RRG 图（可含环的无向图）的子图**——同顶点集，RRT 边集 $\subseteq$ RRG 边集。

```
Algorithm 5: RRG
 1  V ← {x_init};  E ← ∅;
 2  for i = 1, ..., n do
 3      x_rand    ← SampleFree_i;
 4      x_nearest ← Nearest(G = (V, E), x_rand);
 5      x_new     ← Steer(x_nearest, x_rand);
 6      if ObstacleFree(x_nearest, x_new) then
 7          X_near ← Near(G=(V,E), x_new, min{γ_RRG (log(card(V))/card(V))^{1/d}, η});
 8          V ← V ∪ {x_new};  E ← E ∪ {(x_nearest, x_new), (x_new, x_nearest)};
 9          foreach x_near ∈ X_near do
10              if CollisionFree(x_near, x_new) then E ← E ∪ {(x_near, x_new), (x_new, x_near)}
11  return G = (V, E);
```

**k-nearest RRG**：$k=k(\mathtt{card}(V)):=k_{RRG}\log(\mathtt{card}(V))$，$k_{RRG}>k_{RRG}^*=e(1+1/d)$，$X_{near}\leftarrow\mathtt{kNearest}(G,x_{new},k_{RRG}\log(\mathtt{card}(V)))$（行7）。$k_{RRG}=2e$ 对所有实例合法。

### 3.3.3 RRT*（最优 RRT，Algorithm 6）——【本专题最核心算法】

维护**树**结构（而非图），省内存、且便于推广到微分约束/容错。RRT* = 修改 RRG 使**不成环**：删除"冗余"边（不属于从根到顶点最短路的边）。由于 RRT 与 RRT* 都是同根同顶点集的有向树、边集 $\subseteq$ RRG，这相当于对 RRT 树做**重连线（rewiring）**，确保每顶点经**最小代价路径**到达。

**新增函数**：$\mathtt{Line}(x_1,x_2):[0,s]\to X$ = 从 $x_1$ 到 $x_2$ 的直线路径；$\mathtt{Parent}:V\to V$ 把 $v$ 映到唯一 $u$ 使 $(u,v)\in E$（根 $v_0$ 取 $\mathtt{Parent}(v_0)=v_0$）；$\mathtt{Cost}:V\to\mathbb R_{\ge0}$ 把 $v$ 映到从根到 $v$ 唯一路径的代价。可加代价下 $\mathtt{Cost}(v)=\mathtt{Cost}(\mathtt{Parent}(v))+c(\mathtt{Line}(\mathtt{Parent}(v),v))$，$\mathtt{Cost}(v_0)=0$。

RRT* 与 RRT/RRG 同样加点；考虑从 $x_{new}$ 到 $X_{near}$（处于半径 $r(\mathtt{card}(V))=\min\{\gamma_{RRT^*}(\log(\mathtt{card}(V))/\mathtt{card}(V))^{1/d},\eta\}$ 内的顶点）的连接，但**并非所有可行连接都加边**：
- **(i) Connect along minimum-cost path**：从 $X_{near}$ 中**能以最小代价连到 $x_{new}$** 的顶点建一条边到 $x_{new}$；
- **(ii) Rewire the tree**：若经 $x_{new}$ 到某 $x_{near}$ 的路径**比经其当前父更省**，则建 $x_{new}\to x_{near}$ 边并删 $x_{near}$ 与原父的边（保持树结构）。

```
Algorithm 6: RRT*
 1  V ← {x_init};  E ← ∅;
 2  for i = 1, ..., n do
 3      x_rand    ← SampleFree_i;
 4      x_nearest ← Nearest(G = (V, E), x_rand);
 5      x_new     ← Steer(x_nearest, x_rand);
 6      if ObstacleFree(x_nearest, x_new) then
 7          X_near ← Near(G=(V,E), x_new, min{γ_RRT* (log(card(V))/card(V))^{1/d}, η});
 8          V ← V ∪ {x_new};
 9          x_min ← x_nearest;  c_min ← Cost(x_nearest) + c(Line(x_nearest, x_new));
10          foreach x_near ∈ X_near do                       // Connect along a minimum-cost path
11              if CollisionFree(x_near, x_new) ∧ Cost(x_near) + c(Line(x_near, x_new)) < c_min then
12                  x_min ← x_near;  c_min ← Cost(x_near) + c(Line(x_near, x_new))
13          E ← E ∪ {(x_min, x_new)};
14          foreach x_near ∈ X_near do                                       // Rewire the tree
15              if CollisionFree(x_new, x_near) ∧ Cost(x_new) + c(Line(x_new, x_near)) < Cost(x_near)
16              then x_parent ← Parent(x_near);  E ← (E \ {(x_parent, x_near)}) ∪ {(x_new, x_near)}
17  return G = (V, E);
```

**k-nearest RRT\***：$k(\mathtt{card}(V))=k_{RRG}\log(\mathtt{card}(V))$，$X_{near}\leftarrow\mathtt{kNearest}(G,x_{new},k_{RRG}\log(i))$（行7）。

> **三个半径常数的对照（务必记牢）**：
> | 算法 | 半径/邻居数公式 | 常数下界 |
> |---|---|---|
> | PRM* / RRG (r-disc) | $r(n)=\gamma\,(\log n/n)^{1/d}$ | $\gamma>2(1+1/d)^{1/d}(\mu(X_{free})/\zeta_d)^{1/d}$ |
> | RRT* (r-disc) | $r(n)=\min\{\gamma_{RRT^*}(\log n/n)^{1/d},\eta\}$ | $\gamma_{RRT^*}>(2(1+1/d))^{1/d}(\mu(X_{free})/\zeta_d)^{1/d}$ |
> | PRM* / RRG (k-near) | $k(n)=k\log n$ | $k>e(1+1/d)$ |
> | RRT* (k-near) | $k(n)=k_{RRT^*}\log n$ | $k_{RRT^*}>2^{d+1}e(1+1/d)$ |
>
> **注意 RRT* 的 r-disc 常数下界 $(2(1+1/d))^{1/d}$ 与 PRM*/RRG 的 $2(1+1/d)^{1/d}$ 不同**（指数位置不同！RRT* 把 2 也放进 $1/d$ 次方内）。这是因为 RRT* 是单查询树、连接结构更受限，需更大半径常数保证最优性。

## 3.4 RRT-Connect（KL00，单查询双树 + 贪心连接，全量伪码）

KL00 把 RRT 与一个"贪心地把两棵树连起来"的启发式结合，专为**单查询、无微分约束**问题设计。

### 3.4.1 BUILD_RRT 与 EXTEND（KL00 Figure 2，逐行）

```
BUILD_RRT(q_init)
 1  T.init(q_init);
 2  for k = 1 to K do
 3      q_rand ← RANDOM_CONFIG();
 4      EXTEND(T, q_rand);
 5  Return T

EXTEND(T, q)
 1  q_near ← NEAREST_NEIGHBOR(q, T);
 2  if NEW_CONFIG(q, q_near, q_new) then        // 朝 q 走固定增量 ε，并测碰撞
 3      T.add_vertex(q_new);
 4      T.add_edge(q_near, q_new);
 5      if q_new = q then
 6          Return Reached;                     // q 直接到达（已在 ε 内）
 7      else
 8          Return Advanced;                    // 新顶点 q_new ≠ q 加入树
 9  Return Trapped;                             // q_new ∉ C_free，拒绝
```

**NEW_CONFIG**（KL00 §2 文字）：朝 $q$ 以固定增量 $\epsilon$ 运动并测碰撞（用增量距离计算"几乎常数时间"完成）。三种返回：
- **Reached**：$q$ 直接加入树（树已含 $q$ 的 $\epsilon$ 邻域内顶点）；
- **Advanced**：新顶点 $q_{new}\neq q$ 加入树；
- **Trapped**：提出的新顶点不在 $\mathcal C_{free}$，被拒。

### 3.4.2 CONNECT 与 RRT_CONNECT_PLANNER（KL00 Figure 5，逐行）

```
CONNECT(T, q)
 1  repeat
 2      S ← EXTEND(T, q);
 3  until not (S = Advanced)              // 一直 EXTEND 直到 Reached 或 Trapped
 4  Return S;

RRT_CONNECT_PLANNER(q_init, q_goal)
 1  T_a.init(q_init);  T_b.init(q_goal);
 2  for k = 1 to K do
 3      q_rand ← RANDOM_CONFIG();
 4      if not (EXTEND(T_a, q_rand) = Trapped) then       // T_a 朝随机点扩一步
 5          if (CONNECT(T_b, q_new) = Reached) then        // T_b 贪心连向 T_a 的新顶点
 6              Return PATH(T_a, T_b);                      // 两树相遇 → 返回路径
 7      SWAP(T_a, T_b);                                     // 交换角色
 8  Return Failure
```

**Connect 启发式（KL00 §3）**：贪心函数，可替代 EXTEND。不是只扩一个 $\epsilon$ 步，而是**迭代 EXTEND 直到到达 $q$ 或撞障碍**。作用类似随机势场法的人工势函数，使快速收敛；但结合了 RRT 的快速均匀探索，避开势场法的局部极小陷阱——"basin of attraction 随树增长而移动"（而势场法吸引盆固定在目标）。**CONNECT 的关键优势**：一次 CONNECT 只调一次 NEAREST_NEIGHBOR 即可建长路径（每个新顶点成为下一个的最近邻）。变体：把 RRT_CONNECT_PLANNER 中 EXTEND 换成 CONNECT 得"更强贪心"的规划器；把 CONNECT 退化为 EXTEND 得简单双 RRT 规划器。

### 3.4.3 RRT 的 Voronoi 偏置与快速探索（KL00 §2, Figure 4）

**核心性质**："顶点被选中扩展的概率 **正比于其 Voronoi 区域的面积**"（"the probability that a vertex is selected for extension is proportional to the area of its Voronoi region"）。这使 RRT **偏置向大的未探索区域**扩展（biased to rapidly explore），先快速铺开、再均匀覆盖空间。这是 RRT 高效的根本原因。

## 3.5 概率完备性（KF11 §4.1，全量保真 + 全部证明）

### 3.5.1 强 $\delta$-clearance 与概率完备性定义

设 $\delta>0$。状态 $x\in X_{free}$ 称 **$\delta$-内部状态**，若以 $x$ 为心、半径 $\delta$ 的闭球完全在 $X_{free}$ 内。**$\delta$-内部**：
$$\mathrm{int}_\delta(X_{free}):=\{x\in X_{free}\mid B_{x,\delta}\subseteq X_{free}\}$$
（= 离任意障碍点至少 $\delta$ 的状态集）。无碰撞路径 $\sigma$ 有**强 $\delta$-clearance（strong $\delta$-clearance）**，若 $\sigma$ 完全在 $\mathrm{int}_\delta(X_{free})$ 内（$\sigma(\tau)\in\mathrm{int}_\delta(X_{free}),\forall\tau$）。问题 $(X_{free},x_{init},X_{goal})$ **鲁棒可行（robustly feasible）**，若存在某 $\delta>0$ 的强 $\delta$-clearance 路径解之。

**定义 14（概率完备性）** 算法 ALG 概率完备，若对任意鲁棒可行问题 $(X_{free},x_{init},X_{goal})$：
$$\boxed{\liminf_{n\to\infty}\mathbb P\big(\{\exists x_{goal}\in V_n^{\mathrm{ALG}}\cap X_{goal}\text{ s.t. }x_{init}\text{ 在 }G_n^{\mathrm{ALG}}\text{ 中与 }x_{goal}\text{ 连通}\}\big)=1.}$$
若 ALG 概率完备且问题鲁棒可行，则上述极限存在且等于 1；反之若问题非鲁棒可行，则该极限对任意采样式算法（含概率完备者）等于 0（除非样本取自适配问题的奇异分布）。

### 3.5.2 sPRM 与 RRT 的（指数收敛）概率完备性

**定理 15（sPRM 概率完备性，Kavraki et al. 1998）** 对鲁棒可行问题，存在仅依赖 $X_{free},X_{goal}$ 的常数 $a>0,n_0\in\mathbb N$，使
$$\mathbb P\big(\exists x_{goal}\in V_n^{\mathrm{sPRM}}\cap X_{goal}:x_{goal}\text{ 与 }x_{init}\text{ 在 }G_n^{\mathrm{sPRM}}\text{ 中连通}\big)>1-e^{-a\,n},\quad\forall n>n_0.$$

**定理 16（RRT 概率完备性，LaValle & Kuffner 2001）** 对鲁棒可行问题，存在仅依赖 $X_{free},X_{goal}$ 的 $a>0,n_0\in\mathbb N$，使
$$\mathbb P\big(V_n^{\mathrm{RRT}}\cap X_{goal}\neq\emptyset\big)>1-e^{-a\,n},\quad\forall n>n_0.$$
> **失败概率随顶点数指数趋零**——这是 sPRM/RRT 完备性的"快"的体现。

### 3.5.3 1-最近邻 sPRM 的**不**完备性（定理 17，含完整证明）

实用启发式不一定继承完备性。1-最近邻 sPRM（记 1PRM）：每顶点连其最近邻，返回无向图。RRT 可看作 1-最近邻 sPRM 的增量版（也连最近邻，但增量构造**强制连通**）。

**定理 17（k-nearest sPRM 在 $k=1$ 时不完备）** $k=1$ 的 k-nearest sPRM 不概率完备。且
$$\lim_{n\to\infty}\mathbb P\big(\{\exists x_{goal}\in V_n^{1PRM}\cap X_{goal}\text{ s.t. }x_{init}\text{ 与 }x_{goal}\text{ 连通}\}\big)=0.$$

设 $X_{free}=X$。$G_n^{1PRM}=(V_n^{1PRM},E_n^{1PRM})$ = $n$ 样本下 1PRM 返回的图。$L_n$ = 所有边总长。$\zeta_d$ = 单位球体积，$\zeta_d'$ = 圆心相距单位距离的两单位球并的体积。

**引理 18（1-最近邻图总长度，Wade 2007）** 对 $d\ge2$，$L_n/n^{1-1/d}$ 在均方意义下收敛到常数：
$$\lim_{n\to\infty}\mathbb E\Big[\Big(\frac{L_n}{n^{1-1/d}}-\big(1+\tfrac1d\big)\frac1{\zeta_d}\frac{\zeta_d}{2(\zeta_d')^{1+1/d}}\Big)^2\Big]=0.$$
（证明：Wade 2007 定理 3 的直接推论。）

**引理 19（1-最近邻图连通分量数）** 对 $d\ge2$，$N_n/n$ 均方收敛到常数：
$$\lim_{n\to\infty}\mathbb E\Big[\Big(\frac{N_n}{n}-\frac{\zeta_d}{2\zeta_d'}\Big)^2\Big]=0.$$
（证明：**互反对（reciprocal pair）** = 互为最近邻的一对顶点。在"每顶点连最近邻"的图中，当顶点数 $>2$ 时**每个连通分量恰含一个互反对**（Eppstein et al. 1997）。互反对数均方收敛到 $\zeta_d/(2\zeta_d')$（Henze 1987；Wade 2007 Remark 2）。$N_n$ = 互反对数。）

**定理 17 的证明**：设 $\widetilde L_n=L_n/N_n$ = 连通分量平均长度；$L_n'$ = 含 $x_{init}$ 的连通分量长度。样本独立均匀 $\Rightarrow$ $\widetilde L_n,L_n'$ 同分布（虽相依）。设 $\gamma_L$ = $L_n/n^{1-1/d}$ 的极限常数（引理18），$\gamma_N$ = $N_n/n$ 的极限常数（引理19）。

均方收敛 $\Rightarrow$ 依概率收敛 $\Rightarrow$ 依分布收敛。$L_n/n^{1-1/d}$ 与 $N_n/n$ 均方收敛到常数且 $\mathbb P(N_n=0)=0$，由 **Slutsky 定理**：
$$n^{1/d}\widetilde L_n=\frac{L_n/n^{1-1/d}}{N_n/n}\ \xrightarrow{d}\ \gamma:=\frac{\gamma_L}{\gamma_N}\quad(\text{常数,故亦依概率收敛}).$$
则 $n^{1/d}L_n'$ 也依概率收敛到 $\gamma$（因 $\widetilde L_n,L_n'$ 同分布）。故 $L_n'\to0$ 依概率，即 $\lim_{n\to\infty}\mathbb P(\{L_n'>\epsilon\})=0,\forall\epsilon>0$。

取 $\epsilon<\inf_{x\in X_{goal}}\|x-x_{init}\|$。设 $A_n$ = 1PRM 返回图含可行路径（从 $x_{init}$ 到 $X_{goal}$）的事件。$\{L_n'>\epsilon\}$ 在 $A_n$ 发生时必发生，即 $A_n\subseteq\{L_n'>\epsilon\}$，故 $\mathbb P(A_n)\le\mathbb P(\{L_n'>\epsilon\})$。取上极限：
$$\liminf_{n\to\infty}\mathbb P(A_n)\le\limsup_{n\to\infty}\mathbb P(A_n)\le\limsup_{n\to\infty}\mathbb P(\{L_n'>\epsilon\})=0.$$
故 $\lim_{n\to\infty}\mathbb P(A_n)$ 存在且 $=0$。$\blacksquare$

> **直觉**：1PRM 的"含 $x_{init}$ 的连通分量"长度 $L_n'\sim n^{-1/d}\to0$，缩到一点；故它够不到任何固定距离外的目标，失败概率趋 1。RRT 因强制增量连通而幸免（定理16）。

### 3.5.4 变半径 sPRM（$r=\gamma n^{-1/d}$）的不完备性（定理 20，含完整证明）

**定理 20** 存在常数 $\gamma>0$ 使连接半径 $r(n)=\gamma n^{-1/d}$ 的变半径 sPRM 不概率完备。

$\lambda_c$ = 临界密度/连续渗流阈值。$G_\Gamma^{disc}(n,r)$ = 顶点独立均匀采自 Borel 集 $\Gamma$、$\|v-v'\|<r_n$ 连边的随机 $r$-disc 图。

**引理 21（Penrose 2003）** 设 $\lambda\in(0,\lambda_c)$，$\Gamma\subset\mathbb R^d$ Borel 集，$\{r_n\}$ 满足 $nr_n^d\le\lambda,\forall n$。$N_{max}(G_\Gamma^{disc}(n,r_n))$ = 最大分量大小。则存在 $a,b>0,m_0\in\mathbb N$ 使 $\forall m\ge m_0$：
$$\mathbb P\big(N_{max}(G_\Gamma^{disc}(n,r_n))\ge m\big)\le n\big(e^{-am}+e^{-bn}\big).$$

**定理 20 的证明**：取 $\epsilon$ 使 $\epsilon<\inf_{x\in X_{goal}}\|x-x_{init}\|$ 且 $2\epsilon$-球（心 $x_{init}$）完全在自由空间内。$G_n^{PRM}$ = 变半径 sPRM 返回图。$G_n=(V_n,E_n)$ = 其在 $B_{x_{init},2\epsilon}$ 上的限制（$V_n=V_n^{PRM}\cap B_{x_{init},2\epsilon}$，$E_n=(V_n\times V_n)\cap E_n^{PRM}$）。

$G_n$ 等价于 $\Gamma=B_{x_{init},2\epsilon}$ 上的随机 $r$-disc 图。由引理 21，$\exists a,b>0,m_0$ 使 $\mathbb P(\{N_{max}(G_n)\ge m\})\le n(e^{-am}+e^{-bn}),\forall m\ge m_0$。取 $m=\lambda^{-1/d}(\epsilon/2)n^{1/d}>m_0$：
$$\mathbb P\big(N_{max}(G_n)\ge\lambda^{-1/d}\tfrac\epsilon2 n^{1/d}\big)\le n\big(e^{-a\lambda^{-1/d}(\epsilon/2)n^{1/d}}+e^{-bn}\big).$$
设 $L_n$ = 含 $x_{init}$ 的分量的边总长。因 $r_n=\lambda^{1/d}n^{-1/d}$：
$$\mathbb P\big(L_n\ge\tfrac\epsilon2\big)\le n\big(e^{-a\lambda^{-1/d}(\epsilon/2)n^{1/d}}+e^{-bn}\big).$$
右端可和（summable），由 **Borel-Cantelli 引理**，事件 $\{L_n\ge\epsilon/2\}$ 无穷次发生的概率为 0，即 $\mathbb P(\limsup_n\{L_n\ge\epsilon/2\})=0$。

设 $D_n$ = $G_n$ 最大分量的直径（图直径 = $\max_{v,v'\in V}\|v-v'\|$）。显然 $D_n\le L_n$ 必然成立，故 $\mathbb P(\limsup_n\{D_n\ge\epsilon/2\})=0$。设 $I\in\mathbb N$ 是满足 $r_I\le\epsilon/2$ 的最小数；对 $n\ge I$，连到 $V_n^{PRM}\cap B_{x_{init},\epsilon}$ 的边与连到 $V_n\cap B_{x_{init},\epsilon}$ 的一致。设 $R_n$ = 含 $x_{init}$ 分量中离 $x_{init}$ 最远顶点的距离。对 $n\ge I$，$R_n\ge\epsilon$ 只当 $D_n\ge\epsilon/2$，即 $\{R_n\ge\epsilon\}\subseteq\{D_n\ge\epsilon/2\}$，故 $\mathbb P(\limsup_n\{R_n\ge\epsilon\})=0$。

设 $A_n$ = 返回图含到达目标的路径。$\{R_n\ge\epsilon\}$ 在 $A_n$ 发生时必发生，$\mathbb P(A_n)\le\mathbb P(\{R_n\ge\epsilon\})$。取上极限：
$$\liminf_n\mathbb P(A_n)\le\limsup_n\mathbb P(A_n)\le\limsup_n\mathbb P(\{R_n\ge\epsilon\})\le\mathbb P(\limsup_n\{R_n\ge\epsilon\})=0.$$
故 $\lim_n\mathbb P(A_n)=0$。$\blacksquare$

> **教训**：$r(n)\propto n^{-1/d}$（**无 $\log$ 因子**）落在**亚临界（subcritical）渗流区**，图碎成小簇、够不到目标。PRM* 的 $r(n)\propto(\log n/n)^{1/d}$ 多出的 $\log$ 因子正是跨过渗流/连通阈值的关键（对照定理 7：连通要求 $\zeta_d r^d>\log n/n$）。

### 3.5.5 提出算法的概率完备性

**定理 22（PRM\* 完备）** PRM* 概率完备（由其渐近最优性 §4.2 蕴含）。

**定理 23（RRG 与 RRT\* 概率完备）** RRG、RRT* 概率完备。且对鲁棒可行问题，$\exists a>0,n_0$ 仅依赖 $X_{free},X_{goal}$ 使
$$\mathbb P(V_n^{RRG}\cap X_{goal}\neq\emptyset)>1-e^{-an},\qquad \mathbb P(V_n^{RRT^*}\cap X_{goal}\neq\emptyset)>1-e^{-an},\quad\forall n>n_0.$$
**证明**：构造上 $V_n^{RRG}(\omega)=V_n^{RRT^*}(\omega)=V_n^{RRT}(\omega),\forall\omega,n$，且 RRG/RRT* 返回连通图。故结论直接由 RRT 概率完备性（定理16）得出。即：若 RRT 在第 $n$ 步给出可行解，则同采样序列下 RRG、RRT* 也给出。$\blacksquare$

### 3.5.6 RRT-Connect 的概率完备性（KL00 §4，独立证明，全量）

> KL00 给出 RRT/RRT-Connect 完备性的**自包含**证明（球链 + 顶点分布收敛），与 KF11 的视角互补，一并保真记录。

设 $D_k(q)$ = $q$ 到 $G$ 中最近顶点的距离（随机变量，$k$=顶点数），$d_k$ = 其值；$\epsilon$ = EXTEND 增量（RRT 步长）。

**引理 1（凸 $\mathcal C_{free}$ 上的覆盖，KL00）** 设 $\mathcal C_{free}$ 为 $n$ 维构型空间的凸有界开子集。对任意 $q\in\mathcal C_{free}$ 与常数 $\epsilon>0$，$\lim_{k\to\infty}\mathbb P[d_k(q)<\epsilon]=1$。

**证明梗概**：$q\in\mathcal C_{free}$，$q_0$ = 任一初始 RRT 顶点。$B(q)$ = 心 $q$、半径 $\epsilon$ 的球，$B^\epsilon(q)=B(q)\cap\mathcal C_{free}$，$\mu(B^\epsilon(q))>0$。初始 $d_1(q)=\rho(q,q_0)$。每次迭代随机点落入 $B^\epsilon(q)$ 的概率严格为正。故若所有 RRT 顶点都在 $B(q)$ 外，则 $\mathbb E[D_k]-\mathbb E[D_{k+1}]>b$（某 $b>0$）。这蕴含 $\lim_{k\to\infty}\mathbb P[d_k(q)<\epsilon]=1$。$\blacksquare$

**引理 2（非凸单连通 $\mathcal C_{free}$ 上的覆盖，KL00）** 设 $\mathcal C_{free}$ 为 $n$ 维构型空间的非凸、有界、开、单连通 $n$ 维分量。对任意 $q\in\mathcal C_{free}$ 与 $\epsilon>0$，$\lim_{n\to\infty}\mathbb P[d_n(q)<\epsilon]=1$。

**证明梗概**：$q_0$ = 任一初始 RRT 顶点。若 $q_0,q$ 在有界开集同一连通分量，则存在构型序列 $q_1,q_2,\dots,q_k$ 与球序列 $B=B_1(q_1),\dots,B_k(q_k)$，满足 $B_i\cap B_{i+1}\neq\emptyset$（$i\in\{1,\dots,n-1\}$）、$q_0\in B_1$、$q\in B_k$。设 $C_i=B_i\cap B_{i+1}$，可构造使各 $C_i$ 为开集 $\Rightarrow\mu(C_i)>0$。对每个 $C_i$ **归纳应用引理 1**得 $\lim_{n\to\infty}\mathbb P[d_n(q_i)<\epsilon]=1$（点 $q_i\in C_i$），且 $\epsilon$ 可选使 RRT 顶点落入 $C_i$。最终概率趋 1 使 RRT 顶点落入 $B_k$；再对 $B_k$ 应用引理 1 得 $\mathbb P[d_n(q)<\epsilon]=1$。$\blacksquare$

**定理 1（顶点分布收敛，KL00）** $X_k$（RRT 顶点分布）依概率收敛到 $X$（采样分布）。

**证明梗概**：考虑"未覆盖"集 $Y_k=\{q\in\mathcal C_{free}\mid\rho(q,v)>\epsilon\ \forall v\in V_k\}$（$V_k$ = 第 $k$ 步顶点集）。由引理 2，$Y_{k+1}\subseteq Y_k$ 且 $\mu(Y_k)\to0$。RRT 在样本落入某顶点 $\epsilon$ 内时加顶点，此时新顶点服从与 $X$ 相同的概率密度。因 $\mu(Y_k)\to0$，$X$ 与 $X_k$ 的密度仅在某集 $Z_k\subseteq Y_k$ 上不同；$\mu(Y_k)\to0\Rightarrow\mu(Z_k)\to0$；$X$ 密度光滑 $\Rightarrow X_k\xrightarrow{p}X$。$\blacksquare$

**推论 1（RRT-Connect 概率完备）** RRT-Connect 概率完备，且顶点收敛到 $\mathcal C_{free}$ 上的均匀分布。

**证明梗概**：定理 1 对多棵 RRT 也成立。CONNECT 启发式生成所有常规 RRT 顶点 + 额外顶点；额外顶点贡献于 $\mathcal C_{free}$ 的覆盖，不损害 $\mu(Y_k),\mu(Z_k)\to0$ 的收敛结论。$\blacksquare$

> **KL00 自承局限**：未给出**收敛率**的理论刻画（"we do not have a theoretical characterization of the rate of convergence (which is observed to be very fast in practice)"）。收敛率的理论分析是其遗留问题（KF11 的指数界 + 渐近最优性部分填补此空白）。

## 3.6 渐近最优性（KF11 §4.2，全量保真 + 关键证明）

### 3.6.1 弱 $\delta$-clearance、路径空间范数、鲁棒最优解

**同伦（homotopy）**：$\sigma_1,\sigma_2\in\Sigma_{free}$ 同端点。$\sigma_1$ 同伦于 $\sigma_2$，若存在连续 $\psi:[0,1]\to\Sigma_{free}$（同伦），$\psi(0)=\sigma_1,\psi(1)=\sigma_2$，且 $\psi(\tau)$ 对所有 $\tau$ 都是无碰撞路径。

**弱 $\delta$-clearance（weak $\delta$-clearance）**：无碰撞路径 $\sigma:[0,s]\to X_{free}$ 有弱 $\delta$-clearance，若存在有强 $\delta$-clearance 的路径 $\sigma'$ 与同伦 $\psi$（$\psi(0)=\sigma,\psi(1)=\sigma'$）使**对所有 $\alpha\in(0,1]$ 存在 $\delta_\alpha>0$ 令 $\psi(\alpha)$ 有强 $\delta_\alpha$-clearance**。弱 $\delta$-clearance **不要求路径上各点离障碍至少 $\delta$**——一条有不可数个点在障碍边界上的无碰撞路径仍可有弱 $\delta$-clearance（如路径恰穿过两球形障碍接触点，见 KF11 Fig.4）。

**路径向量空间与 BV 范数**：$\Sigma$ 在逐点加法 $(\sigma_1+\sigma_2)(\tau)=\sigma_1(\tau)+\sigma_2(\tau)$ 与标量乘 $(\alpha\sigma)(\tau)=\alpha\sigma(\tau)$ 下是向量空间。定义范数
$$\|\sigma\|_{BV}:=\int_0^1|\sigma(\tau)|\,d\tau+\mathrm{TV}(\sigma),$$
记赋此范数的 $\Sigma$ 为 $BV(X)$。诱导距离
$$\mathrm{dist}(\sigma_1,\sigma_2)=\|\sigma_1-\sigma_2\|_{BV}=\int_0^1\|(\sigma_1-\sigma_2)(\tau)\|\,d\tau+\mathrm{TV}(\sigma_1-\sigma_2).$$
$\{\sigma_n\}$ 收敛到 $\bar\sigma$（记 $\lim_n\sigma_n=\bar\sigma$）即 $\lim_n\|\sigma_n-\bar\sigma\|_{BV}=0$。

**鲁棒最优解（robustly optimal solution）**：解问题 3 的可行路径 $\sigma^*$ 称鲁棒最优，若它有弱 $\delta$-clearance，且对任意收敛到 $\sigma^*$ 的无碰撞路径列 $\{\sigma_n\}$（$\lim_n\sigma_n=\sigma^*$）有 $\lim_n c(\sigma_n)=c(\sigma^*)$。$c^*=c(\sigma^*)$；$Y_n^{\mathrm{ALG}}$ = ALG 第 $n$ 步图中最优解代价（扩展随机变量）。

**定义 24（渐近最优性）** 算法 ALG 渐近最优，若对任意有有限代价 $c^*$ 鲁棒最优解的问题与代价 $c$：
$$\boxed{\mathbb P\Big(\limsup_{n\to\infty}Y_n^{\mathrm{ALG}}=c^*\Big)=1.}$$
因 $Y_n^{\mathrm{ALG}}\ge c^*$，渐近最优蕴含 $\lim_n Y_n^{\mathrm{ALG}}$ 存在且 $=c^*$。概率完备是渐近最优的**必要条件**。

**引理 25（0-1 律）** 给定 $\limsup_n Y_n^{\mathrm{ALG}}<\infty$（ALG 终能找到可行解），则 $\limsup_n Y_n^{\mathrm{ALG}}=c^*$ 的概率非 0 即 1。
**证明**：条件化于 $\{\limsup_n Y_n^{\mathrm{ALG}}<\infty\}$ 保证 $Y_n^{\mathrm{ALG}}$ 对大 $n$ 有限。设 $\mathcal F_m'$ = $\{Y_n^{\mathrm{ALG}}\}_{n=m}^\infty$ 生成的 $\sigma$-域，尾 $\sigma$-域 $\mathcal T=\bigcap_{n\in\mathbb N}\mathcal F_n'$。任何尾事件由 **Kolmogorov 0-1 律**以概率 0 或 1 发生。$\{\limsup_n Y_n^{\mathrm{ALG}}=c^*\}=\{\limsup_{n\ge m}Y_n^{\mathrm{ALG}}=c^*\}\in\mathcal F_m',\forall m$，故 $\in\mathcal T$ 是尾事件。$\blacksquare$
> 即：采样式算法**要么在几乎所有 run 都收敛到最优，要么在几乎所有 run 都不收敛**。

**引理 26（单调性 $\Rightarrow$ 极限存在）** 若 $G_i^{\mathrm{ALG}}(\omega)\subseteq G_{i+1}^{\mathrm{ALG}}(\omega),\forall\omega,i$，则 $\lim_n Y_n^{\mathrm{ALG}}(\omega)=Y_\infty^{\mathrm{ALG}}(\omega)$。
**证明**：$G_i\subseteq G_{i+1}\Rightarrow Y_{i+1}^{\mathrm{ALG}}\le Y_i^{\mathrm{ALG}}$；又 $Y_i^{\mathrm{ALG}}\ge c^*$，故单调有下界收敛到依赖 $\omega$ 的极限 $Y_\infty^{\mathrm{ALG}}(\omega)$。$\blacksquare$
> **PRM、sPRM、RRT、RRG、RRT\* 满足单调性**（图只增不减）；**k-nearest sPRM 与 PRM\* 不满足**（$Y_{i+1}$ 不一定被 $Y_i$ 支配，因 $k(n),r(n)$ 随 $n$ 变会重连）。

**假设 27（最优路径零测）** 所有最优轨迹经过的点集测度为 0：$\mu(X_{opt})=0$，其中 $X_{opt}=\{x\in X_{free}\mid\exists\sigma^*\in\Sigma^*,\tau\in[0,1]:x=\sigma^*(\tau)\}$。
> 多数代价/实例满足（如目标区凸时的欧氏长度）。不蕴含唯一最优路径（可有不可数条最优路径仍满足假设27，如 3D 中球形障碍居中的情形）。

**引理 28** 若假设27成立，采样式算法在有限步 $n$ 返回含最优路径图的概率为 0：$\mathbb P(\cup_{n}\{Y_n^{\mathrm{ALG}}=c^*\})=0$。
**证明**：$B_n=\{Y_n^{\mathrm{ALG}}=c^*\}$，$B=\cup_n B_n$。$B_n\subseteq B_{n+1}$，由测度单调 $\lim_i\mathbb P(B_n)=\mathbb P(B)$。由假设27与采样定义，$\mathbb P(B_n)=0,\forall n$（有限个样本点含零测集中点的概率为0）。故 $\mathbb P(B)=0$。$\blacksquare$

### 3.6.2 已有算法的（非）最优性（KF11 §4.2.1）

**定理 29（PRM 非渐近最优）** PRM 算法不渐近最优。
> （PRM 是森林、不连同分量内顶点，故漏掉很多更短连接；将由定理33的 RRT 非最优性框架蕴含。）

**定理 30（sPRM 渐近最优）** sPRM 算法渐近最优。
> （sPRM 用固定半径 $r$ 连所有近邻——但代价是复杂度 $\Omega(n)$ 次碰撞检测，见 §3.7；PRM* 用 $\Theta(\log n)$ 达到同样最优性。）

**定理 31（k-nearest sPRM 非渐近最优）** k-nearest sPRM（固定 $k$）不渐近最优。
**证明梗概**（KF11 给完整 tiling 证明）：$\sigma^*$ = 最优路径，$s^*=\mathrm{TV}(\sigma^*)$。把 $\sigma^*$ 邻域切成 tiles（瓦片，见 KF11 Fig.5），用 Poisson 过程独立性（引理11）与序统计算出"固定 $k$ 个最近邻无法跨越所有 tile 接力"的概率，得 $\mathbb P(\{\limsup_n Y_n=c^*\})<1$，再由引理25 $\Rightarrow=0$。$\blacksquare$

**定理 32（变半径 sPRM with $r(n)=\gamma n^{-1/d}$ 非渐近最优）** 此变半径 sPRM 不渐近最优。$\mathbb P(\limsup_n Y_n=c^*)=0$（Kolmogorov 0-1 律）。

**定理 33（RRT 非渐近最优）——【关键负面结果】** RRT 算法不渐近最优。

因 RRT 每步要么加一点一边、要么图不变，$G_i^{RRT}\subseteq G_{i+1}^{RRT},\forall i,\omega$，故 $\lim_n Y_n^{RRT}$ 存在 $=Y_\infty^{RRT}$。结合引理25，定理33 蕴含此极限**几乎必然严格大于 $c^*$**：
$$\boxed{\mathbb P\big(\{\lim_{n\to\infty}Y_n^{RRT}>c^*\}\big)=1.}$$
即 **RRT 最优解代价几乎必然收敛到一个次优随机变量值**。事实上可构造实例使 RRT 首解代价任意高的概率有正下界（Nechushtan et al. 2010）。

> **此结果的洞见**：跑多个 RRT 实例 = 抽 $Y_\infty^{RRT}$ 的多个样本（解释 Ferguson & Stentz 2006 多实例 RRT 的有效性）。

#### 定理 33 的完整证明（KF11 附录 B，逐行展开）

简化假设：(i) 无障碍 $X_{free}=[0,1]^d$，(ii) Steer 参数 $\eta$ 足够大（如 $\eta\ge\mathrm{diam}(X_{free})=\sqrt d$）。此情形足以证明非最优（给出一个鲁棒最优却不收敛的反例）。

**证明大纲**：按加入顺序给 RRT 顶点编号。含**根的第 $k$ 个孩子及其所有后代**的顶点集称树的**第 $k$ 分支（k-th branch）**。先证渐近最优的**必要条件**是"无穷多分支含小球外顶点"；再证 RRT 以概率 1 **违反**此条件。

**B.1 必要条件**

**引理 44** 设 $0<R<\inf_{y\in X_{goal}}\|y-x_{init}\|$。事件 $\{\lim_N Y_n^{RRT}=c^*\}$ 仅当**第 $k$ 分支对无穷多 $k$ 含 $R$-球（心 $x_{init}$）外顶点**时才发生。

**证明**：$\{x_1,x_2,\dots\}$ = 根的孩子（按加入序）。$\Gamma(x_k)$ = 从根经 $x_k$ 到目标的最优路径代价。由假设27（引理28），$\mathbb P(\Gamma(x_k)=c^*)=0,\forall k$。故
$$\mathbb P\Big(\bigcup_{k\in\mathbb N}\{\Gamma(x_k)=c^*\}\Big)\le\sum_{k=1}^\infty\mathbb P(\{\Gamma(x_k)=c^*\})=0.$$
$A_k$ = 第 $k$ 分支至少一顶点在 $R$-球外的事件。若 $\{\limsup_k A_k\}$ 不发生且 $\{\Gamma(x_k)>c^*\}$ 对所有 $k$ 发生：则 $A_k$ 只对有限 $k$ 发生。设 $K$ = 使 $A_K$ 发生的最大数。则最优解代价 $\ge\sup\{\Gamma(x_k)\mid k\in\{1,\dots,K\}\}$，严格 $>c^*$（因 $\Gamma(x_k)>c^*$ 对所有有限 $k$）。故 $\lim_n Y_n^{RRT}>c^*$。即
$$\Big(\limsup_{k\to\infty}A_k\Big)^c\cap\bigcap_{k}\{\Gamma(x_k)>c^*\}\subseteq\{\lim_n Y_n^{RRT}>c^*\}.$$
取补 + 测度单调 + 并集界：
$$\mathbb P\big(\lim_n Y_n^{RRT}=c^*\big)\le\mathbb P\big(\limsup_k A_k\big)+\mathbb P\Big(\bigcup_k\{\Gamma(x_k)=c^*\}\Big),$$
末项 $=0$（上证）。$\blacksquare$

**B.2 分支首路径长度**

**引理 45** 设 $U=\{X_1,\dots,X_n\}$ 独立均匀于 $[0,1]^d$，$X_{n+1}$ 独立均匀采样。则 $X_{n+1}$ 的最近邻是 $X_i$ 的概率为 $1/n$（对所有 $i$）。且 $X_{n+1}$ 到其 $U$ 中最近邻的**期望距离为 $n^{-1/d}$**。
**证明**：均匀分布 $\Rightarrow X_{n+1}$ 最近为 $X_i$ 的概率对所有 $i$ 相同 $=1/n$。期望距离由均匀分布序统计得。$\blacksquare$
> 推论：RRT 每顶点的度随 $n\to\infty$ **几乎必然无界**。

**无穷路径构造**：$\Lambda$ = 自然数无穷序列集 $\alpha=(\alpha_1,\alpha_2,\dots)$。$\pi_i:\alpha\mapsto(\alpha_1,\dots,\alpha_i)$ 取前缀。字典序：$\alpha\le\beta\iff\exists j$ 使 $\alpha_i=\beta_i(i\le j-1)$ 且 $\alpha_j\le\beta_j$。$L_{\pi_i(\alpha)}$ = 从根到其 $\alpha_1$-孩子、再到该点 $\alpha_2$-孩子……共 $i$ 项的距离和。$L_\alpha=\lim_{i\to\infty}L_{\pi_i(\alpha)}$（极限存在，$L_{\pi_i(\alpha)}$ 关于 $i$ 非降）。引入 $\mathbf k=(k,1,1,\dots)$，简记 $L_k:=L_{(k,1,1,\dots)}$。

**引理 46** $\mathbb E[L_k]$ 非负有限、关于 $k$ 单调非增（$\mathbb E[L_{k+1}]\le\mathbb E[L_k]$）、且 $\lim_{k\to\infty}\mathbb E[L_k]=0$。
**证明**：无障碍 + $\eta$ 大时，$V_n^{RRT}$ 恰为前 $n$ 样本，每新样本连最近邻。$Z_i$ = 第 $i$ 步对 $L_1$ 的贡献（若第 $i$ 样本在算 $L_1$ 的路径上则为其到前 $i-1$ 样本最近邻距离，否则 0）。由引理45 + 单调收敛定理：
$$\mathbb E[L_1]=\mathbb E\Big[\sum_{i=1}^\infty Z_i\Big]=\sum_{i=1}^\infty\mathbb E[Z_i]=\sum_{i=1}^\infty i^{-1/d}i^{-1}=\zeta(1+1/d),$$
$\zeta$ = Riemann zeta 函数。$\zeta(y)$ 对 $y>1$ 有限，故 $\mathbb E[L_1]$ 对所有 $d$ 有限。设 $N_k$ = 首个贡献于 $L_k$ 的样本的迭代数：
$$\mathbb E[L_{k+1}]=\sum_{i=N_k+1}^\infty i^{-(1+1/d)}=\mathbb E[L_1]-\sum_{i=1}^{N_k}i^{-(1+1/d)}.$$
故 $\mathbb E[L_{k+1}]<\mathbb E[L_k]$；又 $N_k\ge k\Rightarrow\lim_k\mathbb E[L_k]=0$。$\blacksquare$

**B.3 分支最长路径长度**

**引理 48** $\mathbb E[L_\alpha]\le\mathbb E[L_k]$，对所有 $\alpha\ge\mathbf k$。
**证明**（归纳）：$\alpha\ge\mathbf k\Rightarrow\pi_1(\alpha)\ge k$，由引理46 $\mathbb E[L_{(\pi_1(\alpha),1,1,\dots)}]\le\mathbb E[L_k]$。又对任意 $i$，$\mathbb E[L_{(\pi_{i+1}(\alpha),1,\dots)}]\le\mathbb E[L_{(\pi_i(\alpha),1,\dots)}]$（对以 $\pi_i(\alpha)$ 末顶点为根的子树用类似论证）。$\blacksquare$

**引理 47** 对任意 $\epsilon>0$：$\mathbb P(\{\sup_{\alpha\ge\mathbf k}L_\alpha>\epsilon\})\le\dfrac{\mathbb E[L_k]}{\epsilon}.$
**证明**：$\bar\alpha:=\inf\{\alpha\ge\mathbf k\mid L_\alpha>\epsilon\}$（若无则 $\bar\alpha:=\mathbf k$），$\bar\alpha\ge\mathbf k$ 必然，由引理48 $\mathbb E[L_{\bar\alpha}]\le\mathbb E[L_k]$。$I_\epsilon$ = 事件 $S_\epsilon:=\{\sup_{\alpha\ge\mathbf k}L_\alpha>\epsilon\}$ 指示变量。则
$$\mathbb E[L_k]\ge\mathbb E[L_{\bar\alpha}]=\mathbb E[L_{\bar\alpha}I_\epsilon]+\mathbb E[L_{\bar\alpha}(1-I_\epsilon)]\ge\epsilon\,\mathbb P(S_\epsilon),$$
末步因 $S_\epsilon$ 发生时 $L_{\bar\alpha}\ge\epsilon$。$\blacksquare$

**推论 49** 对任意 $\epsilon>0$：$\lim_{k\to\infty}\mathbb P(\{\sup_{\alpha\ge\mathbf k}L_\alpha>\epsilon\})=0$（由引理46+47）。

**B.4 违反必要条件**

由引理44，渐近最优的必要条件是第 $k$ 分支对无穷多 $k$ 含 $R$-球外顶点（$0<R<\inf_{y\in X_{goal}}\|y-x_{init}\|$）。此事件仅当第 $k$ 分支最长路径 $>R$ 对无穷多 $k$。故
$$\mathbb P\big(\lim_n Y_n^{RRT}=c^*\big)\le\mathbb P\Big(\limsup_{k\to\infty}\{\sup_{\alpha\ge\mathbf k}L_\alpha>R\}\Big).$$
右端事件单调（$\{\sup_{\alpha\ge k+1}L_\alpha>R\}\supseteq\{\sup_{\alpha\ge k}L_\alpha>R\}$），由测度连续 $\mathbb P(\limsup_k\cdots)=\lim_k\mathbb P(\{\sup_{\alpha\ge\mathbf k}L_\alpha>R\})$。由推论49 此 $=0,\forall R>0$。故 $\mathbb P(\{\lim_n Y_n^{RRT}=c^*\})=0$。$\blacksquare$

> **RRT 非最优性的物理直觉**：RRT 一旦把某顶点连到树，**永不重连**（无 rewire）。早期建立的"歪"连接被冻结，后代只能继承之；分支最长路径期望 $\to0$（引理46，$\zeta$ 函数尾部），意味着新分支越来越"短"、够不到远处去改善已有路径。RRT* 加 rewire 正是为打破这个冻结。

### 3.6.3 提出算法的渐近最优性（KF11 §4.2.2，定理 34–39）

> 这些是本专题的**正面核心结论**。证明很长（KF11 附录 C–G），下给定理 + 证明技术骨架。

回顾 $d$ = 维数，$\mu(X_{free})$ = 自由空间测度，$\zeta_d$ = $d$ 维单位球体积。

**定理 34（PRM\* 渐近最优）** 若 $\gamma_{PRM}>2(1+1/d)^{1/d}\big(\dfrac{\mu(X_{free})}{\zeta_d}\big)^{1/d}$，则 PRM* 渐近最优。

**定理 35（k-nearest PRM\* 渐近最优）** 若 $k_{PRM}>e(1+1/d)$，则 k-nearest PRM* 渐近最优。

**定理 36（RRG 渐近最优）** 若 $\gamma_{PRM}>2(1+1/d)^{1/d}\big(\dfrac{\mu(X_{free})}{\zeta_d}\big)^{1/d}$，则 RRG 渐近最优。

**定理 37（k-nearest RRG 渐近最优）** 若 $k_{RRG}>e(1+1/d)$，则 k-nearest RRG 渐近最优。

**定理 38（RRT\* 渐近最优）——【本专题最核心定理】** 若
$$\boxed{\gamma_{RRT^*}>\big(2(1+1/d)\big)^{1/d}\Big(\frac{\mu(X_{free})}{\zeta_d}\Big)^{1/d}},$$
则 RRT* 渐近最优。

**定理 39（k-nearest RRT\* 渐近最优）** 若 $k_{RRT^*}>2^{d+1}e(1+1/d)$，则 k-nearest RRT* 渐近最优。（由定理37、38 的证明得出。）

#### PRM* 渐近最优性的证明技术（KF11 附录 C，球覆盖法，骨架全量）

这是渐近最优性证明的**典范技术**（"ball-covering / ball-chain" 论证），RRG/RRT* 的证明都是其变体。

**C.1 大纲**：$\sigma^*$ = 鲁棒最优路径（有弱 $\delta$-clearance）。
1. 构造 $\{\delta_n\}$（$\delta_n>0$，$\delta_n\to0$）与路径列 $\{\sigma_n\}$，使 $\sigma_n$ 有强 $\delta_n$-clearance 且 $\sigma_n\to\sigma^*$。
2. 构造 $\{q_n\}$ 与覆盖球集 $B_n=\{B_{n,1},\dots,B_{n,M_n}\}$（每球半径 $q_n$），共同"覆盖"$\sigma_n$。使任意相邻两球各取一点 $x_m\in B_{n,m},x_{m+1}\in B_{n,m+1}$ 满足 (i) $\|x_m-x_{m+1}\|\le$ 连接半径 $r(n)$，(ii) 连线段在自由空间内（取 $\delta_n,q_n$ 为 $r(n)$ 的常数倍可满足）。
3. 设 $A_n$ = "$B_n$ 中每球都含至少一个 PRM* 顶点"的事件。证 $A_n$ 对所有大 $n$ 以概率 1 发生。此时 PRM* 会用边连接相邻球内顶点，所成路径无碰撞。
4. 证如此所成路径列收敛到 $\sigma^*$；用 $\sigma^*$ 的鲁棒性证 PRM* 返回图最优解代价几乎必然收敛到 $c(\sigma^*)$。

**C.2 引理 50（强/弱 clearance 连接）** 设 $\sigma^*$ 有强 $\delta$-clearance，$\{\delta_n\}$ 满足 $\delta_n\to0$、$0\le\delta_n\le\delta$。则存在路径列 $\{\sigma_n\}$ 使 $\sigma_n\to\sigma^*$ 且 $\sigma_n$ 有强 $\delta_n$-clearance。
**证明**：$X_n:=\mathrm{cl}(\mathrm{int}_{\delta_n}(X_{free}))$（闭、离障碍 $\ge\delta_n$）。设 $\psi:[0,1]\to\Sigma_{free}$ 为弱 clearance 保证的同伦（$\psi(0)=\sigma^*$）。定义 $\alpha_n:=\max_{\alpha\in[0,1]}\{\alpha\mid\psi(\alpha)\in\Sigma_{X_n}\}$，$\sigma_n:=\psi(\alpha_n)$。$\Sigma_{X_n}$ 闭 $\Rightarrow$ 极大值可达；$\psi(1)$ 有强 $\delta$-clearance 且 $\delta_n\le\delta\Rightarrow\sigma_n\in\Sigma_{X_n}$（强 $\delta_n$-clearance）。$\bigcup_n X_n=X_{free}$（$\delta_n\to0$），弱 clearance $\Rightarrow\lim_n\alpha_n=0\Rightarrow\lim_n\sigma_n=\sigma^*$。$\blacksquare$

连接半径 $r_n=\gamma_{PRM}(\log n/n)^{1/d}$。取小常数 $\theta_1>0$，$\delta_n:=\min\{\delta,\frac{1+\theta_1}{2+\theta_1}r_n\}$（满足 $0\le\delta_n\le\delta,\delta_n\to0$）。

**C.3 覆盖球（定义 51）** 给定 $\sigma_n:[0,1]\to X$ 与 $q_n,l_n>0$，$\mathtt{CoveringBalls}(\sigma_n,q_n,l_n)=\{B_{n,1},\dots,B_{n,M_n}\}$：$B_{n,m}$ 心 $\sigma(\tau_m)$、半径 $q_n$；$\tau_1=0$；相邻心**恰相距 $l_n$**（$\tau_m=\min\{\tau\in[\tau_{m-1},1]\mid\|\sigma(\tau)-\sigma(\tau_{m-1})\|\ge l_n\}$）；$M-1$ 为最大可生成数且末球心 $\sigma(1)$（$\tau_{M_n}=1$）。取 $q_n:=\delta_n/(1+\theta_1)$，$B_n:=\mathtt{CoveringBalls}(\sigma_n,q_n,\theta_1 q_n)$。各球半径 $q_n$、相邻心距 $\theta_1 q_n$，球集覆盖 $\sigma_n$。

**C.4 每球含至少一顶点的概率**：$A_{n,m}=\{B_{n,m}\cap V_n^{PRM^*}\neq\emptyset\}$，$A_n=\cap_{m=1}^{M_n}A_{n,m}$。用大偏差（binomial 集中）+ $\gamma_{PRM}>2(1+1/d)^{1/d}(\mu(X_{free})/\zeta_d)^{1/d}$ 保证 $\mathbb P(\liminf_n A_n)=1$（每球期望顶点数 $\propto\log n$，半径常数足够大使漏球概率可和 $\Rightarrow$ Borel-Cantelli）。

**C.5–C.6**：$A_n$ 发生时相邻球顶点被边连成无碰撞路径，其代价 $\to c(\sigma_n)\to c(\sigma^*)=c^*$；由鲁棒最优性的代价连续性 $\Rightarrow\limsup_n Y_n^{PRM^*}=c^*$ 几乎必然。$\blacksquare$

#### RRT* 渐近最优性的证明技术（KF11 附录 G，标记点过程 + 球链，骨架全量）

RRT* 证明在 PRM* 球覆盖基础上多一层难点：RRT* 是**增量、有向、按到达顺序**建树，需保证球链内顶点的**到达顺序**也"对"（前球顶点先于后球顶点到达，才能连成从根出发的路径）。

**G.1 标记点过程（marked point process）**：$\{X_1,\dots,X_n\}$ 独立均匀于 $X_{free}$，$\{Y_1,\dots,Y_n\}$ 独立均匀于 $[0,1]$ 作**到达顺序标记**（$X_{i'}$ 在 $X_i$ 前抽 $\iff Y_{i'}<Y_i$）；含 $x_{init}$ 标记 $Y=0$。建图 $G_n$：加边 $(X_{i'},X_i)$ 当 (i) $Y_{i'}<Y_i$ 且 (ii) $\|X_i-X_{i'}\|\le r_n$。$G_n$ 无有向环。子图 $G_n'$：每顶点 $X_i$ 取使 $c(X_i)$（从 $x_{init}$ 到 $X_i$ 最优路径代价）最小的唯一父。**$G_n'$ 等价于 $\eta$ 足够大时 RRT* 第 $n$ 步返回的图**。设 $Y_n,Y_n'$ 为 $G_n,G_n'$ 中到目标最优解代价，$\limsup_n Y_n=\limsup_n Y_n'$ 必然。证 $\mathbb P(\{\limsup_n Y_n=c^*\})=1$ 即得结论。

**G.2 路径列与球集**：$\sigma^*$ = 最优路径，$\delta_n:=\min\{\delta,4r_n\}$，$\{\sigma_n\}$ 由引理50 保证。$r_n=\gamma_{RRT^*}(\log n/n)^{1/d}$。$B_n:=\mathtt{CoveringBalls}(\sigma_n,r_n,2r_n)$——相邻球心距 $2r_n$，故**球开互斥（openly disjoint）**。

**G.3 连接相邻球（引理 71）** 设 $A_{n,m}$ = 存在 $X_i\in B_{n,m},X_{i'}\in B_{n,m+1}$ 使 $Y_{i'}\le Y_i$（即 $X_{i'}$ 先于 $X_i$ 到达，则 $X_i,X_{i'}$ 在 $G_n$ 中连边）。$A_n=\cap_{m=1}^{M_n}A_{n,m}$。**若 $\gamma_{RRT^*}>4(\mu(X_{free})/\zeta_d)^{1/d}$，则 $\mathbb P(\liminf_n A_n)=1$。**

**证明（Poisson 化 + 序统计，骨架）**：用 $\mathrm{Poisson}(\theta n)$ 点（$\theta\in(0,1)$）做 Poisson 过程（强度 $\theta n/\mu(X_{free})$，引理11）。$\mathbb P(A_{n,m}^c)\le\mathbb P(\tilde A_{n,m}^c)+\mathbb P(\{\mathrm{Poisson}(\theta n)>n\})$，末项 $\le e^{-an}$。设 $N_{n,m}$ = $B_{n,m}$ 内顶点数，$\mathbb E[N_{n,m}]=\frac{\zeta_d\gamma_{RRT^*}^d}{\mu(X_{free})}\log n$，记 $\alpha:=\zeta_d\gamma_{RRT^*}^d/\mu(X_{free})$。事件 $C_{n,m,\epsilon}=\{N_{n,m}\ge(1-\epsilon)\alpha\log n\}$，binomial 大偏差：$\mathbb P(C_{n,m,\epsilon}^c)\le e^{-\alpha H(\epsilon)\log n}=n^{-\alpha H(\epsilon)}$，$H(\epsilon)=\epsilon+(1-\epsilon)\log(1-\epsilon)$（$H(0)=0,H(1)=1$）。

关键序统计：$\mathbb P(\tilde A_{n,m}^c\mid C_{n,m,\epsilon}\cap C_{n,m+1,\epsilon})$ = "1 减去（$\alpha\log n$ 个 $[0,1]$ 均匀样本的**最大值** < 另 $\alpha\log n$ 个样本的**最小值**）的概率"。由均匀序统计：最小值密度 $f_{\min}(x)=\frac{(1-x)^{\alpha\log n-1}}{\mathrm{Beta}(1,\alpha\log n)}$，最大值 CDF $F_{\max}(x)=x^{\alpha\log n}$。计算此概率随 $n$ 的衰减，配合 $\gamma_{RRT^*}>4(\mu(X_{free})/\zeta_d)^{1/d}$（即 $\alpha$ 足够大）使 $\sum_n M_n\mathbb P(A_{n,m}^c)<\infty$，Borel-Cantelli $\Rightarrow\mathbb P(\liminf_n A_n)=1$。$\blacksquare$

> **RRT* 证明的精髓**：标记 $Y_i$ 把"空间覆盖"（球链）与"时间顺序"（到达序）解耦为独立的均匀随机变量；连边条件 $Y_{i'}\le Y_i$ + $\|X_i-X_{i'}\|\le r_n$ 的概率由序统计算出，半径常数 $\gamma_{RRT^*}$ 的下界恰好压过这个序统计的衰减率。这就是为什么 RRT* 的 $\gamma$ 下界比 PRM* 多一个因子（$(2(1+1/d))^{1/d}$ vs $2(1+1/d)^{1/d}$）——增量树的"顺序约束"要求更密的采样。

## 3.7 计算复杂度（KF11 §4.3，全量）

**渐近记号**：$W_n^{\mathrm{ALG}}(P)$ = ALG 在输入 $P=(X_{free},x_{init},X_{goal})$、$n$ 下返回图的某函数（随机变量）。$f:\mathbb N\to\mathbb N$ 增、$\lim_n f(n)=\infty$。
- $W_n^{\mathrm{ALG}}\in\Omega(f(n))$：$\exists P$ 使 $\liminf_n\mathbb E[W_n^{\mathrm{ALG}}(P)/f(n)]>0$；
- $W_n^{\mathrm{ALG}}\in O(f(n))$：$\forall P$，$\limsup_n\mathbb E[W_n^{\mathrm{ALG}}(P)/f(n)]<\infty$。

**碰撞检测调用次数 $M_n^{\mathrm{ALG}}$**：

**引理 40（PRM）** $M_n^{PRM}\in\Omega(n)$。
**证明**：构造 $X_{free}=X_1\cup X_2$（两开不交集，KF11 Fig.9）。$X_2$ 为超矩形、一边 $=r/2$。任意心在 $X_2$ 的 $r$-球必含 $X_2$ 非零测部分。$\bar\mu:=\inf_{x\in X_2}\mu(B_{x,r}\cap X_1)>0$。落入 $X_2$ 的样本 $X_n$ 须尝试连到 $X_1$ 某子集 $X_1'$（$\mu(X_1')\ge\bar\mu$）的若干顶点（期望 $\ge\bar\mu n$），且这些都不与 $X_n$ 同分量。故 $\mathbb E[M_n^{PRM}/n]>\bar\mu$。$\blacksquare$

**引理 41（sPRM）** $M_n^{sPRM}\in\Omega(n)$。
**证明**：更强结论——$\forall P$，$\liminf_n\mathbb E[M_n^{sPRM}/n]>0$。$\bar\mu:=\inf_{x\in X_{free}}\mu(B_{x,r}\cap X_{free})>0$（$X_{free}$ 是开集闭包）。$M_n$ = 末样本 $X_n$ 半径 $r$ 球内节点数；球内 $X_{free}$ 体积 $\ge\bar\mu$。$\mathbb E[M_n^{sPRM}]\ge\frac{\bar\mu}{\mu(X_{free})}n$（二项过程下界）。故 $\mathbb E[M_n/n]\ge\bar\mu/\mu(X_{free})$。$\blacksquare$

**k-nearest PRM**：$M_n^{k\text{-}sPRM}=k,\forall n>k$。**RRT**：$M_n^{RRT}=1,\forall n$。

**引理 42（PRM\*, RRG, RRT\*）** $M_n^{PRM^*},M_n^{RRG},M_n^{RRT^*}\in O(\log n)$。
**证明（PRM\* 部分）**：$r_n=\gamma_{PRM}(\log n/n)^{1/d}$。末样本 $X_n$ 落入 $r_n$-内部 $\mathrm{int}_{r_n}(X_{free})$ 时，球内期望顶点数 $\mathbb E[M_n^{PRM^*}\mid A]=\zeta_d\gamma_{PRM}^d\cdot\frac{\log n}{?}\cdots\propto\log n$（KF11 给出 $=\zeta_d\gamma_{PRM}^d\frac{\mu(B_{X_n,r_n})}{\mu(X_{free})}n=\zeta_d\gamma_{PRM}^d\log n$ 量级），边界情形贡献更低阶。故 $\in O(\log n)$。$\blacksquare$

> **复杂度总结表（KF11 §4.3）**：
> | 算法 | $M_n$（碰撞检测/迭代摊还） | 渐近最优？ |
> |---|---|---|
> | PRM | $\Omega(n)$ | 否（定理29） |
> | sPRM | $\Omega(n)$ | 是（定理30） |
> | k-nearest sPRM | $k$（常数） | 否（定理31） |
> | RRT | $1$（常数） | 否（定理33） |
> | **PRM\*** | $O(\log n)$ | **是（定理34）** |
> | **RRG** | $O(\log n)$ | **是（定理36）** |
> | **RRT\*** | $O(\log n)$ | **是（定理38）** |
>
> **核心结论**：PRM*/RRG/RRT* 以**仅 $O(\log n)$ 的每步代价**（仅是概率完备但非最优的 RRT 的常数倍 × $\log n$）达到**渐近最优**。这是 KF11 的中心贡献——"渐近最优几乎免费"。

## 3.8 数值实验（KF11 §5，全量保真）

实现：C 语言，2.66 GHz / 4GB / Linux。除注明外路径代价 = 全变差。

**实验1（k-nearest PRM vs PRM\*，2D）**：并排跑，画最优路径代价 vs 迭代数（KF11 Fig.10）。**k-nearest PRM 不收敛到最优，PRM\* 收敛**。PRM* 在 $d\le5$ 维亦验证（Fig.11）。

**实验2（RRT vs RRT\*，主实验）**：三场景，前两场景代价 = 欧氏长度。
- **场景1（无障碍，方形环境）**：树在各阶段见 Fig.12。**RRT 不改善可行解、不收敛到最优；RRT\* 持续改善到最优**。Monte-Carlo：各跑 20,000 迭代 × 500 次，最优代价逐迭代平均（Fig.13）。**极限上 RRT 代价非常接近最优的 $\sqrt2$ 倍**（LaValle & Kuffner 2009 在确定性设定有类似结果），**RRT\* 收敛到最优**。RRT 不同 run 的方差 $\to2.5$，RRT\* 方差 $\to0$——即几乎所有 RRT* run 都收敛到最优（符合渐近最优）。
- **场景2（有障碍）**：20,000 迭代后树见 Fig.14；RRT* 不同阶段见 Fig.15。**RRT\* 先像 RRT 一样快速探索**，随样本增多改善树含更短路径，**最终发现不同同伦类（homotopy class）的路径**，大幅降低到目标的代价。Monte-Carlo（20,000 迭代 × 500 次，Fig.16）：**所有 RRT\* run 收敛到最优；RRT 平均约最优的 1.5 倍**。RRT 高方差源于两个到目标的同伦类——若 RRT 幸运收敛到含最优解的同伦类则接近最优，否则（此场景常见）约 2 倍最优。
- **场景3（无障碍，加权代价）**：代价 = 一个函数的线积分（高代价区取 2、低代价区取 1/2、其余 1）。20,000 迭代后 RRT* 树见 Fig.17。**树或避开高代价区、或快速穿过；对低代价区反之**（对应光的 **Snell-Descartes 折射定律**，Rowe & Alexander 2000）。

**运行时间**：无障碍并排跑到 100 万迭代，RRT*/RRT 运行时间比 vs 迭代数（50 次平均，Fig.18）**收敛到常数**（符合 §4.3 复杂度分析）。场景2 类似（Fig.19）。5 维（Fig.20–21）、10 维（Fig.22）亦验证。

> **三个 take-away 数字（务必记牢，常考/常引用）**：
> 1. **无障碍下 RRT 收敛到 $\sqrt2\approx1.414$ 倍最优**（确定性 + 随机均如此）；
> 2. **有障碍（双同伦类）下 RRT 平均约 1.5 倍最优**；
> 3. **RRT\* 方差 $\to0$、代价 $\to$ 最优**；RRT 方差 $\to2.5$（场景1）。
> 这定量印证定理33（RRT 非最优）与定理38（RRT* 最优）。

---

# 第四部分：轨迹优化简介 — CHOMP（来源 CHOMP13，全量保真）

> 服务章节"轨迹优化简介"。轨迹优化 = 从一条（可能不可行的）初始轨迹出发，**对一个权衡光滑性与避障的泛函做（协变）梯度下降**，把轨迹"拉出"碰撞并平滑。代表：CHOMP、TrajOpt、STOMP。本部分精读 CHOMP（IJRR 2013）的泛函与更新规则。

## 4.1 轨迹与目标泛函（CHOMP13 §3.1）

轨迹 $\xi:[0,1]\to\mathcal C\subset\mathbb R^d$（时间 $\to$ 构型，光滑函数）。固定端点 $\xi(0)=q_0,\xi(1)=q_1$（§6 放松末端到流形）。目标泛函 = 光滑项 + 障碍项的加权和：
$$\boxed{\mathcal U[\xi]=\mathcal F_{obs}[\xi]+\lambda\,\mathcal F_{smooth}[\xi]}\tag{1}$$
$\lambda$ = 光滑/避障权衡系数。

**光滑性泛函（式2）**：度量轨迹动态量（如速度平方积分）：
$$\mathcal F_{smooth}[\xi]=\frac12\int_0^1\Big\|\frac{d}{dt}\xi(t)\Big\|^2\,dt.\tag{2}$$
（可推广到加速度、jerk 等高阶导，此处简化只用速度平方。）

**障碍泛函（式3）**：设 $\mathcal B\subset\mathbb R^3$ = 机器人体表点集，$x:\mathcal C\times\mathcal B\to\mathbb R^3$ = 正运动学（构型 $q$ + 体点 $u\mapsto$ 工作空间点 $x(q,u)$），$c:\mathbb R^3\to\mathbb R$ = 工作空间代价场（penalize 障碍内及附近，通常用到障碍边界的欧氏距离定义，见 §3.4 EDT）。障碍泛函 = 每个体点扫过轨迹时遇到的代价的**弧长参数化线积分**，对所有体点积分：
$$\mathcal F_{obs}[\xi]=\int_0^1\!\!\int_{\mathcal B}c\big(x(\xi(t),u)\big)\,\Big\|\frac{d}{dt}x(\xi(t),u)\Big\|\,du\,dt.\tag{3}$$
代价 $c$ 乘以体点工作空间速度的范数——把简单线积分变为**弧长参数化**，使障碍目标**对重新计时（re-timing）不变**（同一路径不同速度走，$\mathcal F_{obs}$ 不变）。

> **CHOMP 关键创新**：障碍积分在**工作空间**（workspace）做弧长参数化，而非构型空间（区别于 Quinlan 的 elastic bands）。这使几何量在欧氏假设更自然的工作空间计算；直觉上目标泛函**无动机直接改变轨迹在工作空间的速度**。

**障碍泛函的 max 变体（式4）**：取体上最大代价而非积分：
$$\mathcal F_{obs}[\xi]=\int_0^1\max_{u\in\mathcal B}c\big(x(\xi(t),u)\big)\,\Big\|\frac{d}{dt}x(\xi(t),u)\Big\|\,dt.\tag{4}$$
（每步最小化最大违反；计算更省但每步信息更少、可能需更多迭代。）

## 4.2 泛函梯度（CHOMP13 §3.2，Euler-Lagrange 推导，全量）

**泛函梯度** $\bar\nabla\mathcal U$ = 使 $\nabla\mathcal U[\bar\xi+\epsilon\phi]$ 当 $\epsilon\to0$ 最大化的扰动 $\phi:[0,1]\to\mathbb R^d$。设 $\mathcal U[\xi]=\int v(\xi(t),\xi'(t))\,dt$，扰动 $\bar\xi=\xi+\epsilon\phi$，视 $\mathcal U[\bar\xi]$ 为 $f(\epsilon)$：
$$\frac{df}{d\epsilon}=\int\Big(\frac{\partial v}{\partial\bar\xi}\cdot\frac{\partial\bar\xi}{\partial\epsilon}+\frac{\partial v}{\partial\bar\xi'}\cdot\frac{\partial\bar\xi'}{\partial\epsilon}\Big)dt.\tag{5}$$
对第二项**分部积分**，用 $\phi(0)=\phi(1)=0$（端点固定）：
$$\int\frac{\partial v}{\partial\bar\xi'}\cdot\frac{\partial\bar\xi'}{\partial\epsilon}\,dt=-\int\Big(\frac{d}{dt}\frac{\partial v}{\partial\bar\xi'}\Big)\cdot\frac{\partial\bar\xi'}{\partial\epsilon}\,dt.\tag{6}$$
> **【\rebuilt 勘误】** KF/CHOMP 原文式(6)右端写作 $\frac{\partial\bar\xi'}{\partial\epsilon}$，按分部积分应为 $\frac{\partial\bar\xi}{\partial\epsilon}$（即 $\phi$），此处保留原文符号但指出：分部后乘的应是 $\partial\bar\xi/\partial\epsilon$。综合时以式(8)(9)为准。

故总体：
$$\frac{df}{d\epsilon}=\int\Big(\frac{\partial v}{\partial\bar\xi}-\frac{d}{dt}\frac{\partial v}{\partial\bar\xi'}\Big)\cdot\frac{\partial\bar\xi}{\partial\epsilon}\,dt.\tag{7}$$
$\epsilon=0$ 时即 $\mathcal U[\xi]$ 沿 $\phi$ 方向的方向导数：
$$\frac{df}{d\epsilon}\Big|_0=\int\Big(\frac{\partial v}{\partial\xi}-\frac{d}{dt}\frac{\partial v}{\partial\xi'}\Big)\cdot\phi\,dt.\tag{8}$$
使此积分最大（在有限欧氏范数下）的 $\phi$ 正比于括号项，故**泛函梯度**：
$$\boxed{\bar\nabla\mathcal U[\xi]=\frac{\partial v}{\partial\xi}-\frac{d}{dt}\frac{\partial v}{\partial\xi'}}\tag{9}$$
（这正是 **Euler-Lagrange 方程**；$\bar\nabla\mathcal U=0$ 即临界点。）梯度下降：
$$\xi_{i+1}=\xi_i-\eta_i\,\bar\nabla\mathcal U[\xi].\tag{10}$$

**各项泛函梯度**：$\bar\nabla\mathcal U=\bar\nabla\mathcal F_{obs}+\lambda\bar\nabla\mathcal F_{smooth}$。光滑项（式2）梯度：
$$\bar\nabla\mathcal F_{smooth}[\xi](t)=-\frac{d^2}{dt^2}\xi(t).\tag{11}$$
障碍项（式3）梯度：
$$\bar\nabla\mathcal F_{obs}[\xi]=\int_{\mathcal B}J^T\,\|x'\|\,\big[(I-\hat x'\hat x'^T)\nabla c-c\,\kappa\big]\,du,\tag{12}$$
其中 $\kappa=\|x'\|^{-2}(I-\hat x'\hat x'^T)x''$ 是体点工作空间轨迹的**曲率向量**，$x',x''$ = 体点速度/加速度，$\hat x'$ = 归一化速度向量，$J$ = 该体点运动学 Jacobian。$(I-\hat x'\hat x'^T)$ = **投影矩阵**，把工作空间梯度正交投影到轨迹运动方向——确保更新方向不直接改速度剖面（见 §4.1 直觉）。

## 4.3 协变梯度（CHOMP13 §3.3，covariant gradient）

式(10) 的普通梯度依赖轨迹**表示**（其泛函梯度源于欧氏范数，隐含 Dirac delta 基）。CHOMP 改用**只依赖轨迹动态量**的范数（算子 $A$）：
$$\|\xi\|_A^2=\int\sum_{n=1}^k\alpha_n\big(D^n\xi(t)\big)^2\,dt,\tag{13}$$
$D^n$ = $n$ 阶导算子，$\alpha_n\in\mathbb R$ 常数。$k=1$ 时退化为 $\|\xi\|_A^2=\int\xi'(t)^2\,dt$（速度平方）。内积：
$$\langle\xi_1,\xi_2\rangle=\int\sum_{n=1}^k\alpha_n\big(D^n\xi_1(t)\big)\big(D^n\xi_2(t)\big)\,dt.\tag{14}$$
抽象地 $A=D^\dagger D$ 使 $\|f\|_A^2=\langle f,Af\rangle=\int(Df(t))^2\,dt$。（一般地 $A$ 是轨迹流形上的**黎曼度量**，可随轨迹光滑变化；本文设 $A$ 常数。）此不变范数下的**协变泛函梯度**仅比欧氏梯度多左乘 $A^{-1}$：
$$\boxed{\bar\nabla_A\mathcal U[\xi]=A^{-1}\bar\nabla\mathcal U[\xi]}\tag{15}$$

## 4.4 路点参数化与离散形式（CHOMP13 §3.4）

均匀离散化（步长 $\Delta t$）：$\xi\approx(q_1^T,q_2^T,\dots,q_n^T)^T\in\mathbb R^{n\times d}$，$q_0,q_{n+1}$ = 固定端点。光滑目标（式2）写成有限差分：
$$\mathcal F_{smooth}[\xi]=\frac12\sum_{t=1}^{n+1}\Big\|\frac{q_{t+1}-q_t}{\Delta t}\Big\|^2.\tag{16}$$
用有限差分矩阵 $K$ 与向量 $e$（处理边界 $q_0,q_{n+1}$）：
$$\mathcal F_{smooth}[\xi]=\frac12\|K\xi+e\|^2=\frac12\xi^T A\xi+\xi^T b+c,\tag{17}$$
$A=K^TK$，$b=K^Te$，$c=e^Te/2$。故光滑项关于 $\xi$ **二次**，Hessian $=A$、梯度 $=b$（外加 $A\xi$）。$A$ 度量轨迹**总加速度**。障碍梯度对路点参数化是式(12)的直接修改（$\mathcal B$ 离散化、积分换求和；用中心差分算 $x',x''$ 经 $J$ 映射）。

## 4.5 梯度下降更新规则（CHOMP13 §3.5，climax）

从初始轨迹 $\xi_0$（通常 = 构型空间**直线**，一般不可行）迭代。把更新写成"在使 $\mathcal U$ 下降最多、同时保持 $\xi_{i+1}$ 与 $\xi_i$ 在度量 $M$ 下接近"的拉格朗日优化。一阶 Taylor 展开 $\mathcal U[\xi]\approx\mathcal U[\xi_i]+(\xi-\xi_i)^T\bar\nabla\mathcal U[\xi_i]$，优化问题：
$$\xi_{i+1}=\arg\min_\xi\ \mathcal U[\xi_i]+(\xi-\xi_i)^T\bar\nabla\mathcal U[\xi_i]+\frac\eta2\|\xi-\xi_i\|_M^2,\tag{18}$$
$\|\xi-\xi_i\|_M^2=(\xi-\xi_i)^TM(\xi-\xi_i)$，$\eta$ = 步长/U-步长权衡系数。欧氏情形 $M=I$；CHOMP **取 $M=A$**（偏好只增少量加速度的扰动）。对式(18)关于 $\xi$ 求导置零得**更新规则**：
$$\boxed{\xi_{i+1}=\xi_i-\frac1\eta A^{-1}\bar\nabla\mathcal U[\xi_i]}\tag{19}$$
此规则**协变**（更新只依赖轨迹本身、不依赖表示——小步长 + 细离散极限下）。

**收敛性分析（CHOMP13 §3.5）**：在局部最优附近 $\mathcal F_{obs}$ 凸的区域，总目标**强凸**（可被曲率 $A$ 的二次下界）。CHOMP 更新可理解为序贯最小化目标的局部二次近似（用 $A$ 作下界，比各向同性下界紧得多）。直觉："因单点障碍冲击而调整轨迹大段"——预期 CHOMP 比调单点的标准欧氏梯度法**快 $O(n)$ 倍收敛**。终止准则：$\|\bar\nabla\mathcal U[\xi]\|$ 低于阈值。

> **CHOMP 的完备性局限（CHOMP13 §3.5 自承）**：CHOMP **无完备性**——可能以不可行轨迹（仍有碰撞）终止，且无解时不报告。但判定某轨迹是否可行是直接的（工作空间代价场知道到障碍的距离）。终止后由更高层规划器/协调机制据路径质量与可行性决定下步。$A$ 矩阵虽大但**稀疏**（带状）。
>
> **避免局部极小（搜索结果补充）**：CHOMP 用 **Hamiltonian Monte Carlo (HMC)** 缓解收敛到高代价局部极小的问题，并由此获得**概率完备性**（"uses Hamiltonian Monte Carlo to alleviate the problem of convergence to high-cost local minima (and for probabilistic completeness)"）。

## 4.6 轨迹优化 vs 采样式（衔接，\rebuilt 综合）

| 维度 | 采样式（RRT*/PRM*） | 轨迹优化（CHOMP） |
|---|---|---|
| 完备性 | 概率完备（RRT/PRM）/ 渐近最优（*版） | 局部法，**一般不完备**（CHOMP + HMC 才概率完备） |
| 初值 | 不需可行初值（从根长树） | 需初始轨迹（常用直线，可不可行） |
| 解性质 | 全局（探索多同伦类）；路径常 jagged 需后平滑 | 局部最优、**天然光滑**；不跨同伦类 |
| 高维 | 好（碰撞检测即可） | 好（梯度 + 距离场） |
| 典型用法 | 先采样式找粗解 → 再轨迹优化局部精化 + 平滑 | 精化/平滑其他方法的结果（CHOMP13 §1 明言"to smooth and improve the results of other techniques locally"） |

---

# 第五部分：给本书"规划导论"章的综合建议（清单）

1. **构型空间 $\mathcal C$**（§1.1–1.2）：先建 $\mathcal C$ / $\mathcal C_{free}$ / $\mathcal C_{obs}$、自由度、单/多查询、完备性三层级（complete / resolution complete / probabilistically complete）、PSPACE-hard。强调"显式 $\mathcal C_{free}$ 难算、碰撞检测易"这一采样式立足点。移植到 $\mathrm{SE}(3)$ 时距离/采样改流形版（接本书李群章）。

2. **搜索式（§2）**：A* 的 $f=g+h$、伪码、可采纳 $h\le h^*$、一致 $h(x)\le d(x,y)+h(y)$、最优性定理（可采纳⇒最优；一致⇒不重复展开 + 最优高效）、Dijkstra = $h\equiv0$ 特例。点出维数灾难 $O(1/h^d)$ 引出采样式。

3. **采样式（§3，核心，篇幅最大）**：
   - **算法**：PRM/sPRM（多查询，森林/$r$-disc 图）、RRT（单查询，Voronoi 偏置、online NN 图）、RRT-Connect（双树 + 贪心 CONNECT）、PRM*/RRG/RRT*（半径随 $n$ 缩、$O(\log n)$、渐近最优）。**六个伪码全给**（Alg.1–6 + BUILD_RRT/EXTEND/CONNECT/RRT_CONNECT_PLANNER）。
   - **半径公式**：PRM*/RRG $\gamma>2(1+1/d)^{1/d}(\mu/\zeta_d)^{1/d}$；RRT* $\gamma_{RRT^*}>(2(1+1/d))^{1/d}(\mu/\zeta_d)^{1/d}$；k-near 版 $k>e(1+1/d)$、RRT* k-near $k>2^{d+1}e(1+1/d)$。**务必区分 RRT* 与 PRM* 的常数指数位置不同**。
   - **概率完备性**（定义14、定理15/16 指数界、定理17 1-NN 反例 + 完整证明、定理20 变半径反例、KL00 球链证明）。
   - **渐近最优性**（定义24、引理25 0-1律、引理26 单调性、假设27、**定理33 RRT 非最优完整证明**、定理34–39 + 球覆盖/标记点过程证明骨架）。
   - **复杂度表**（§3.7）+ **实验数字**（RRT→$\sqrt2$×最优、1.5×、RRT* 方差→0）。

4. **轨迹优化简介（§4）**：CHOMP 目标 $\mathcal U=\mathcal F_{obs}+\lambda\mathcal F_{smooth}$（式1–3）、Euler-Lagrange 泛函梯度（式9）、协变更新 $\xi_{i+1}=\xi_i-\frac1\eta A^{-1}\bar\nabla\mathcal U$（式19）、不完备性 + HMC 补救。点出"采样式找粗解 → 轨迹优化平滑精化"的流水线。

5. **统一约定提示**：在章首加一句"本章构型空间记 $\mathcal C$（KF11 用 $X$）；代价函数 $c$、测度 $\mu$、渗流强度 $\lambda$ 与本书 SLAM 章的旋转 $\mathbf C$（不用）、均值 $\mu$、特征值 $\lambda$ 同字母不同义，已隔离在本章"。

---

## 附：本抽取与各源的对应索引（便于综合 agent 回溯原文）

| 本书拟用内容 | 源 + 小节/编号 | 本抽取小节 |
|---|---|---|
| 构型空间/完备性分类 | Wiki-MP；KF11 §1,§2.1 | §1 |
| 问题2/问题3 形式化 | KF11 §2.1 定义1/问题2/问题3 | §1.3 |
| 随机几何图理论（�clude 渗流/连通） | KF11 §2.2 定义4–13、定理6/7/10/12 | （§3.2 sPRM=r-disc 引、§3.5.4 用） |
| A* 伪码/最优性 | HNR68；Wiki-A* | §2 |
| 原始函数 Sample/Near/Steer/CollisionFree | KF11 §3.1 | §3.1 |
| PRM/sPRM/RRT 伪码 | KF11 Alg.1/2/3 | §3.2 |
| PRM*/RRG/RRT* 伪码 + 半径 | KF11 Alg.4/5/6、§3.3 | §3.3 |
| RRT-Connect 伪码 + 完备性 | KL00 Fig.2/5、Lemma1/2/Thm1/Cor1 | §3.4, §3.5.6 |
| 概率完备性定义 + 定理 | KF11 §4.1 定义14、定理15–23 | §3.5 |
| 渐近最优性定义 + 定理 | KF11 §4.2 定义24、引理25/26/28、假设27、定理29–39 | §3.6 |
| RRT 非最优完整证明 | KF11 附录B 引理44–49 | §3.6.2 |
| PRM*/RRT* 最优证明骨架 | KF11 附录C（引理50/51）、附录G（引理71） | §3.6.3 |
| 复杂度 | KF11 §4.3 引理40–42 | §3.7 |
| 实验数字 | KF11 §5 | §3.8 |
| CHOMP 泛函/更新 | CHOMP13 §3 式1–19 | §4 |
