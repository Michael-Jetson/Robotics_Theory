# 抽取留痕 · 规划导论：图搜索规划（A*/启发式/最优性证明、栅格地图）+ 构型空间 + 采样式规划（RRT/PRM/RRT*）+ 轨迹优化简介

> **本文件性质**：项目内部「抽取留痕」，**非成书正文**。目标是把"图搜索规划 A*/启发式/最优性证明、栅格地图"主题及本章（规划导论）须覆盖的全部要素，从**权威来源（原论文/官方书 HTML 与 PDF/权威教程）联网检索并精读**后，**全量保真**抽下来，供综合 agent 写成自包含书章。遵循铁律：禁摘要、每步推导不跳、每条定义/定理/引理 + 完整证明、每张表/算法伪码完整记录、公式用 LaTeX 写全、标源、显式记号约定。
>
> **个人笔记状态**：规划部分的个人笔记**未同步**（见 MEMORY 的 project-relocation / SLAM理论.md 待同步）。故本抽取**主要据权威教材/论文**重建；凡据网络源重述、或我据标准结论补全而无逐字原文者，标 `\rebuilt` 待综合 agent 二次核。

---

## 源清单与精读出处（每条结果标明出处）

本抽取交叉使用以下权威源（均经 WebSearch 检索 + WebFetch/本地 `pdftotext` 精读原文）：

- **S1 = LaValle, *Planning Algorithms*, Cambridge Univ. Press, 2006**（作者官方全文）。
  - Ch.2 离散规划（FORWARD_SEARCH、Dijkstra、A*）：PDF `https://lavalle.pl/planning/ch2.pdf`（本地精读），HTML `https://lavalle.pl/planning/node40.html` 起。
  - Ch.4 构型空间（C、C_obs、C_free、Piano Mover's Problem）：PDF `https://lavalle.pl/planning/ch4.pdf`，HTML `node123.html`/`node156.html`。
  - Ch.5 采样式运动规划（RDT/RRT、PRM）：PDF `https://lavalle.pl/planning/ch5.pdf`，HTML `node230.html`/`node239.html`。
- **S2 = Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning", IJRR 30(7):846–894, 2011**，arXiv:1105.1186（本地 `pdftotext` 精读全文）。PRM*/RRG/RRT* 的最优性定理、常数、概率完备性、渐近最优性、复杂度。
- **S3 = Hart, Nilsson, Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", IEEE Trans. SSC 4(2):100–107, 1968**（A* 原论文，g+h、admissible/consistent 概念）。经 Wikipedia A* 条目 `https://en.wikipedia.org/wiki/A*_search_algorithm` 转述其结论；原文结论与 LaValle/标准教材一致。
- **S4 = Wikipedia 权威条目**（作核对与补全，含证明）：
  - A* `https://en.wikipedia.org/wiki/A*_search_algorithm`
  - Admissible heuristic `https://en.wikipedia.org/wiki/Admissible_heuristic`
  - Consistent heuristic `https://en.wikipedia.org/wiki/Consistent_heuristic`（含 consistency⇒admissibility 归纳证明、f 单调证明）
  - Dijkstra `https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm`
  - RRT `https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree`，PRM `https://en.wikipedia.org/wiki/Probabilistic_roadmap`
- **S5 = Amit Patel (Stanford), "Heuristics" (Red Blob Games / Stanford theory)** `https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html`：栅格地图启发式（Manhattan/对角/octile/Chebyshev/Euclidean）公式、tie-breaking。
- **S6 = Solovey, Janson, Schmerling, Frazzoli, Pavone, "Revisiting the Asymptotic Optimality of RRT*", ICRA 2020**，arXiv:1909.09688：指出 S2 的 RRT* 最优性证明逻辑缺口，给出修正半径条件。

---

## §0 记号约定（务必先读：本主题 vs 本书统一约定的差异）

规划属于**几何/图论**主题，**不涉及李群扰动/四元数**，故本书"右扰动为主、ξ=[ρ;φ]、Hamilton 四元数、R∈SO(3)"等约定**在此基本不触发**。需统一的是**规划自身的符号**，各源差异如下表。

| 概念 | LaValle (S1) | Karaman-Frazzoli (S2) | A*/AI 传统 (S3,S4) | 本书建议统一 | 说明/转换 |
|---|---|---|---|---|---|
| 构型空间 | $\mathcal C$（manifold）| $X=(0,1)^d$ | （图 $G=(V,E)$，状态 $X$）| $\mathcal C$（连续）/ $X$（离散状态/图）| S2 把 C 直接取成单位立方体内的 $\mathbb R^d$ 子集；本书连续规划用 $\mathcal C$，栅格/图用 $X$ 或 $V$。 |
| 障碍/自由空间 | $\mathcal C_{obs}$, $\mathcal C_{free}=\mathcal C\setminus\mathcal C_{obs}$ | $X_{obs}$, $X_{free}=\mathrm{cl}(X\setminus X_{obs})$ | — | $\mathcal C_{obs}$, $\mathcal C_{free}$ | **注意闭/开差异**：LaValle $\mathcal C_{free}$ 是**开集**（$\mathcal C_{obs}$ 闭）；S2 把 $X_{free}$ 取成**闭包** $\mathrm{cl}(X\setminus X_{obs})$（为了能谈"最短路径"这种在开集上无法取到的优化，见 S1 §4.3.1 也指出须用 $\mathrm{cl}(\mathcal C_{free})$）。综合时统一说明：可达性分析用开集，最优化用闭包。 |
| 起点/终点 | $q_I$, $q_G$（query pair）| $x_{init}$, $X_{goal}$（目标是**区域**）| $s$/start, goal | $q_I$/$x_{init}$, $q_G$/$X_{goal}$ | S2 目标是开子集 $X_{goal}$；离散 A* 目标常是单点或集合 $X_G$。 |
| 单步/边代价 | $l(e)=l(x,u)\ge 0$ | $c(\sigma)$（路径泛函，可加）| $d(x,y)$ / $c(N,P)$ / $w(u,v)$ | $c(\cdot)$ / $l(e)$ | 均要求**非负**（Dijkstra/A* 最优性前提）。 |
| 到达代价（cost-to-come）| $C(x)$，最优 $C^*(x)$ | $\mathrm{Cost}(v)$ | $g(n)$ | $g(n)$ / $C(x)$ | **同一对象**：起点到 $x$ 的已知/最优累计代价。 |
| 到目标代价（cost-to-go）启发式 | $\hat G^*(x)$（$G^*$ 的下估计）| — | $h(n)$（$h^*(n)$ 为真值）| $h(n)$，真值 $h^*(n)$ | **同一对象**。LaValle 记 $\hat G^*$、AI 记 $h$。 |
| 评价函数 | $C^*(x)+\hat G^*(x)$ | — | $f(n)=g(n)+h(n)$ | $f(n)=g(n)+h(n)$ | 完全等价；本书用 $f=g+h$ 主记号。 |
| 启发式可采纳条件 | $\hat G^*(x)\le G^*(x)$（underestimate）| — | $h(n)\le h^*(n)$（admissible）| $h(n)\le h^*(n)$ | 同一条件，"绝不高估"。 |
| 步长 / 局部转向 | $\Delta q$ / `stopping-configuration` | $\eta$（`Steer` 半径）| — | $\eta$（或 $\Delta q$）| RRT 一步前进距离上限。 |
| 连接半径 | $r$（PRM 邻域）| $r(n)=\gamma(\log n/n)^{1/d}$ | — | $r(n)$ 同 S2 | 渐近最优的关键尺度。 |
| 维数 | $\dim(\mathcal C)$ | $d$ | — | $d$ | C-空间维数。 |
| 路径/轨迹 | path $\tau:[0,1]\to\mathcal C$ | $\sigma:[0,1]\to X_{free}$，有界变差 BV | — | $\sigma$ / $\tau$ | 见 S2 Def 1。 |

> **本质洞察（跨源统一）**：离散 A*/Dijkstra 的 "状态图 $X$/边代价 $l$" 与连续运动规划的 "构型空间 $\mathcal C$/路径代价 $c$" 是**同一框架在两种粒度下的实例**：栅格地图就是把 $\mathcal C_{free}$ 离散成图后跑 A*；采样式（PRM/RRT）则是把 $\mathcal C_{free}$ 随机离散成图后再跑 A*/Dijkstra（PRM 的 query phase 明确"用 §2.2 的任一搜索算法"，见 S1 §5.6.1）。综合 agent 应把"构型空间 → 图离散化 → 图搜索"作为本章主线。

---

# 第一部分　构型空间（Configuration Space）【S1 Ch.4】

## §1.1 构型空间的基本定义【S1 §4.1–4.2】

**构型（configuration）** $q$：完整确定机器人 $\mathcal A$ 中所有点在世界 $\mathcal W$（$\mathcal W=\mathbb R^2$ 或 $\mathbb R^3$）中位置所需的最少参数。

**构型空间 $\mathcal C$**：所有构型构成的集合。LaValle 强调 $\mathcal C$ 一般是**流形（manifold）**：

> **流形定义（S1 §4.1.2，逐字）**：拓扑空间 $M\subseteq\mathbb R^m$ 是一个 **流形**，若对每个 $x\in M$，存在开集 $O\subset M$ 使得：1) $x\in O$，2) $O$ 同胚于 $\mathbb R^n$，3) $n$ 对所有 $x$ 相同。此 $n$ 称为流形的**维数** $\dim(M)=n$。

典型 C-空间（S1 §4.2，本书可作表）：

| 机器人 | $\mathcal W$ | 构型 $q$ | $\mathcal C$ | $\dim\mathcal C$ |
|---|---|---|---|---|
| 平面平移点 | $\mathbb R^2$ | $(x_t,y_t)$ | $\mathbb R^2$ | 2 |
| 平面刚体 | $\mathbb R^2$ | $(x_t,y_t,\theta)$ | $\mathbb R^2\times S^1=SE(2)$ | 3 |
| 空间刚体 | $\mathbb R^3$ | $(x_t,y_t,z_t,h)$，$h$ 单位四元数 | $\mathbb R^3\times\mathbb{RP}^3=SE(3)$ | 6 |
| $n$ 关节机械臂（旋转关节）| — | $(\theta_1,\dots,\theta_n)$ | $T^n=(S^1)^n$（$n$-环面）| $n$ |

> **本质洞察（S1 §4 引言）**：C-空间的威力在于——无论机器人多复杂（多连杆、多自由度），一旦写成 $\mathcal C$，规划问题就统一成"**在 $\mathcal C_{free}$ 中找一条从 $q_I$ 到 $q_G$ 的路径**"，机器人退化为 $\mathcal C$ 中的**一个点**。这是把"刚体避障"几何问题代数化的关键抽象（Lozano-Pérez 提出，Latombe 著作统一）。

## §1.2 构型空间障碍 $\mathcal C_{obs}$ 与自由空间 $\mathcal C_{free}$【S1 §4.3.1，逐字保真】

设世界 $\mathcal W=\mathbb R^2$ 或 $\mathbb R^3$ 含**障碍区域** $\mathcal O\subset\mathcal W$。设刚体机器人 $\mathcal A\subset\mathcal W$，$\mathcal A$ 与 $\mathcal O$ 均用半代数模型（含多边形/多面体）表示。设 $q\in\mathcal C$ 为 $\mathcal A$ 的构型，$\mathcal A(q)$ 表示按 $q$ 变换后的机器人。

> **定义（C-空间障碍区域，S1 式 4.34）**：
> $$\boxed{\;\mathcal C_{obs}=\{q\in\mathcal C\mid \mathcal A(q)\cap\mathcal O\neq\varnothing\}\;}$$
> 即所有使变换后机器人 $\mathcal A(q)$ 与障碍 $\mathcal O$ 相交的构型之集。**由于 $\mathcal O$ 与 $\mathcal A(q)$ 在 $\mathcal W$ 中均为闭集，$\mathcal C_{obs}$ 是 $\mathcal C$ 中的闭集。**

> **定义（自由空间，S1 §4.3.1）**：
> $$\boxed{\;\mathcal C_{free}=\mathcal C\setminus\mathcal C_{obs}\;}$$
> 由于 $\mathcal C$ 是拓扑空间、$\mathcal C_{obs}$ 闭，**$\mathcal C_{free}$ 必为开集**。这意味着机器人可以**任意接近**障碍而仍在 $\mathcal C_{free}$ 内。

**"接触"边界条件（S1 式 4.35）**：若
$$\mathrm{int}(\mathcal O)\cap\mathrm{int}(\mathcal A(q))=\varnothing\quad\text{且}\quad \mathcal O\cap\mathcal A(q)\neq\varnothing,$$
则 $q\in\mathcal C_{obs}$（$\mathrm{int}$ 表内部）。即仅边界相交也算碰撞。

> **关键陷阱（S1 §4.3.1，本书须保留）**：因 $\mathcal C_{free}$ 是**开集**，某些优化问题（如**最短路径**）**无法良定义**（开集上下确界取不到）。此时须改用闭包 $\mathrm{cl}(\mathcal C_{free})$（见 S1 §7.7）。**——这正是 S2 把 $X_{free}=\mathrm{cl}(X\setminus X_{obs})$ 直接取闭包的原因。** 综合 agent 应在"最优规划"小节显式提醒此开/闭区别。

**多连杆机器人的 $\mathcal C_{obs}$（S1 式 4.36，含自碰撞）**：设机器人为 $m$ 个连杆 $\{\mathcal A_1,\dots,\mathcal A_m\}$，单一构型向量 $q$ 描述全体，$\mathcal A_i(q)$ 为第 $i$ 连杆。设 $P$ 为**碰撞对集合**，$(i,j)\in P$（$i\neq j$）表示不允许 $\mathcal A_i(q)\cap\mathcal A_j(q)\neq\varnothing$。则
$$\boxed{\;\mathcal C_{obs}=\left(\bigcup_{i=1}^{m}\{q\in\mathcal C\mid \mathcal A_i(q)\cap\mathcal O\neq\varnothing\}\right)\cup\left(\bigcup_{[i,j]\in P}\{q\in\mathcal C\mid \mathcal A_i(q)\cap\mathcal A_j(q)\neq\varnothing\}\right)\;}$$
即：某连杆撞障碍，或 $P$ 指定的某对连杆互撞，则 $q\in\mathcal C_{obs}$。$P$ 一般大小 $O(m^2)$，但常可几何分析剔除大量对（相邻连杆恒接触者不入 $P$）。

## §1.3 基本运动规划问题（Piano Mover's Problem）【S1 Formulation 4.1，逐字】

> **Formulation 4.1（钢琴搬运工问题 / 基本运动规划）**：
> 1. 世界 $\mathcal W$，$\mathcal W=\mathbb R^2$ 或 $\mathbb R^3$。
> 2. 世界中半代数障碍区域 $\mathcal O\subset\mathcal W$。
> 3. 半代数机器人，定义于 $\mathcal W$：可为刚体 $\mathcal A$ 或 $m$ 连杆 $\mathcal A_1,\dots,\mathcal A_m$。
> 4. 由"机器人可施加的全部变换"决定的构型空间 $\mathcal C$；由此导出 $\mathcal C_{obs}$ 与 $\mathcal C_{free}$。
> 5. 初始构型 $q_I\in\mathcal C_{free}$。
> 6. 目标构型 $q_G\in\mathcal C_{free}$。$(q_I,q_G)$ 合称 **query（查询对）**。
>
> **任务**：计算 $\mathcal C_{free}$ 中一条**连续路径** $\tau:[0,1]\to\mathcal C_{free}$，满足 $\tau(0)=q_I$ 且 $\tau(1)=q_G$；若不存在则报告失败。

> **核心难点（S1 §4.3.1）**：构造 $\mathcal C_{free}$ 或 $\mathcal C_{obs}$ 的**显式边界/实体表示既不直接也不高效**（尤其高维）。这正是采样式方法（第三部分）回避显式 $\mathcal C_{obs}$、只用碰撞检测的根本动机。

---

# 第二部分　搜索式规划：Dijkstra 与 A*【S1 Ch.2 + S2,S3,S4】

## §2.1 离散规划与通用前向搜索 FORWARD_SEARCH【S1 §2.2.1，逐字伪码】

离散规划问题（状态-空间表示）：状态集 $X$，每状态 $x$ 有动作集 $U(x)$，状态转移 $x'=f(x,u)$，初始 $x_I$，目标集 $X_G$。

搜索中任一时刻，状态分三类（S1 §2.2.1）：
1. **Unvisited（未访问）**：尚未遇到的状态；初始时除 $x_I$ 外全部。
2. **Dead（死）**：已访问、且其**每个可能后继都已访问**的状态——"在此意义上无更多可做"。（S1 §2.4 讨论死状态复活的变体。）
3. **Alive（活）**：已遇到但**可能仍有未访问后继**的状态；初始时唯一的活状态是 $x_I$。活状态存于优先队列 $Q$。

> **算法（S1 图 2.4，FORWARD_SEARCH，逐行）**：
> ```
> FORWARD_SEARCH
>  1  Q.Insert(x_I) and mark x_I as visited
>  2  while Q not empty do
>  3      x ← Q.GetFirst()
>  4      if x ∈ X_G
>  5          return SUCCESS
>  6      forall u ∈ U(x)
>  7          x' ← f(x, u)
>  8          if x' not visited
>  9              Mark x' as visited
> 10              Q.Insert(x')
> 11          else
> 12              Resolve duplicate x'
> 13  return FAILURE
> ```
> **S1 关键论断**："各搜索算法之间唯一实质的差别，是用来对 $Q$ 排序的那个函数。"（The only significant difference between various search algorithms is the particular function used to sort $Q$.）BFS=FIFO，DFS=LIFO，Dijkstra/A*=按代价排序。

## §2.2 Dijkstra 算法【S1 §2.2.2 + S4】

设每条边 $e\in E$ 有非负代价 $l(e)\ge 0$（state-space 记 $l(x,u)$），计划总代价 = 起点到目标路径上边代价之和。

$Q$ 按 **cost-to-come（到达代价）** $C:X\to[0,\infty]$ 排序。$C^*(x)$ 为从 $x_I$ 到 $x$ 的**最优到达代价**（对所有 $x_I\to x$ 路径取边代价累加之最小）；未知最优时记 $C(x)$。

**增量计算（S1）**：初始 $C^*(x_I)=0$。每当生成 $x'$，令
$$C(x')=C^*(x)+l(e),\qquad e\text{ 为 }x\to x'\text{ 的边}\quad(\text{等价 }C(x')=C^*(x)+l(x,u)).$$
$C(x')$ 是"目前已知最佳到达代价"，未必最优（故不写 $C^*$）。FORWARD_SEARCH 第 12 行（resolve duplicate）：若 $x'$ 已在 $Q$ 中，且新路径更优，则**下调** $C(x')$ 并对 $Q$ 重排。

> **定理（Dijkstra 最优性，S1 §2.2.2 归纳论证，逐字重述）**：当 $x$ 经 `Q.GetFirst()` 移出 $Q$（变 dead）时，$C(x)=C^*(x)$（已最优，不可能以更低代价到达）。
>
> **证明（归纳，S1 原文）**：对初始状态 $C^*(x_I)=0$ 已知，为基例。归纳假设：每个 dead 状态的最优到达代价已正确确定（其 $C$ 值不再变）。对 $Q$ 的首元素 $x$，其 $C(x)$ 必最优——因为任何**总代价更低**的路径都必须**经过 $Q$ 中另一状态**，而这些状态的 $C$ 值都**不低于** $x$（$x$ 是队首/最小）；而所有**只经过 dead 状态**的路径在产生 $C(x)$ 时已被考虑。$x$ 的所有出边探索完后，$x$ 宣告 dead，归纳继续。∎
> （S1 注：这不是完整证明，更严格论证见 §2.3.3 与 Dijkstra/CLRS。）

> **关键前提：边代价非负（S4 Dijkstra 条目）**。证明用到"经过 $Q$ 中其它状态的路径代价不低于队首"，依赖非负性；**负权会破坏该不变量**（负环可无限降代价）。负权须用 Bellman-Ford。

**复杂度（S1 + S4）**：
- S1：$O(|V|\lg|V|+|E|)$，前提是 $Q$ 用 **Fibonacci 堆**、其余操作（是否访问等）$O(1)$。
- S4（更细，按堆实现）：
  - 二叉堆：$O((|E|+|V|)\log|V|)$（每次 decrease-key 与 extract-min 各 $O(\log|V|)$，分别执行 $|E|$、$|V|$ 次）。
  - Fibonacci 堆：$O(|E|+|V|\log|V|)$（decrease-key 摊还 $O(1)$）——"任意非负权有向图单源最短路的渐近最快已知算法"。

> **Dijkstra 伪码（S4，优先队列版，逐字）**：
> ```
> function Dijkstra(Graph, source):
>     Q ← min-priority queue
>     dist[source] ← 0
>     for each vertex v in Graph.Vertices:
>         if v ≠ source:
>             dist[v] ← INFINITY
>             prev[v] ← UNDEFINED
>         Q.add_with_priority(v, dist[v])
>     while Q is not empty:
>         u ← Q.extract_min()
>         for each neighbor v of u still in Q:
>             alt ← dist[u] + Graph.Edge_Distance(u, v)        // 松弛
>             if alt < dist[v]:
>                 dist[v] ← alt
>                 prev[v] ← u
>                 Q.decrease_priority(v, alt)
>     return dist, prev
> ```
> **松弛（relaxation）核心**：`alt ← dist[u] + length(u,v); if alt < dist[v] then update`。

## §2.3 A* 搜索：起源、定义与启发式【S1 §2.2.2 + S3 + S4】

**历史（S3）**：A* 由 Peter Hart、Nils Nilsson、Bertram Raphael（Stanford Research Institute）于 **1968** 年发表（论文：*A Formal Basis for the Heuristic Determination of Minimum Cost Paths*），是 **Shakey** 移动机器人项目的产物。Raphael 提议用和 $g(n)+h(n)$（$g$=起点到 $n$ 的距离，$h$=$n$ 到目标的估计）；Hart 发明了今称 **admissibility（可采纳性）** 与 **consistency（一致性）** 的概念。

**A* 思想（S1 §2.2.2，逐字）**："A* 搜索是 Dijkstra 的扩展，通过引入到目标代价的**启发式估计**来减少所探索的状态总数。" 设 $C(x)$ = 从 $x_I$ 到 $x$ 的 cost-to-come，$G(x)$ = 从 $x$ 到 $X_G$ 中某状态的 cost-to-go。$C^*(x)$ 可由动态规划增量算出，但**真实最优 cost-to-go $G^*$ 无法预先知道**。幸而许多应用可构造其**合理下估计**。记此估计为 $\hat G^*(x)$。

**A* = Dijkstra + 改排序函数（S1 逐字）**："A* 的工作方式与 Dijkstra **完全相同，唯一区别是排序 $Q$ 的函数**。" A* 用和
$$\boxed{\;C^*(x')+\hat G^*(x')\;}\quad\Longleftrightarrow\quad \boxed{\;f(n)=g(n)+h(n)\;}$$
即 $Q$ 按"$x_I$ 经 $x'$ 到 $X_G$ 的最优代价估计"排序。当 $\hat G^*(x)=0\ \forall x$ 时，**A* 退化为 Dijkstra**。

**可采纳性条件（underestimate，S1 + S4）**：
$$\boxed{\;\hat G^*(x)\le G^*(x)\quad\forall x\in X\;}\quad\Longleftrightarrow\quad \boxed{\;h(n)\le h^*(n)\quad\text{（admissible：绝不高估）}\;}$$
其中 $h^*(n)$ 为从 $n$ 到目标的**真实最优代价**。

> **定理（A* 最优性，S1 §2.2.2 逐字）**："若 $\hat G^*(x)$ 对所有 $x\in X$ 都是真实最优 cost-to-go 的**下估计**，则 A* 算法**保证找到最优计划**。" 随 $\hat G^*\to G^*$，相比 Dijkstra 探索的顶点数趋少。任何情况下搜索都仍是 systematic（系统的）。

**LaValle 的栅格 Manhattan 下估计例（S1 §2.2.2 逐字）**：迷宫栅格，代价=步数。状态坐标 $(i,j)$ 与 $(i',j')$，则
$$|i'-i|+|j'-j|$$
是下估计，因为它是"忽略障碍的直接计划长度"；一旦加入障碍，绕行只会使代价**增加**（甚至不可达）。$0$ 也是下估计但无信息（退化为 Dijkstra）。目标是估计**尽量接近** $G^*$ 又**保证不超过**。

> **陷阱（S1）：best-first（贪心最佳优先）不保最优**。best-first 仅按 cost-to-go 估计排序，不在意是否高估，故解不必最优；常太贪心、被早期"看起来好"的状态误导（S1 图 2.5 螺旋管反例：可构造浪费至少 $k$ 步的实例）。best-first 不是 systematic，最坏比 A*/DP 更差。

## §2.4 可采纳启发式（Admissible Heuristic）【S4 Admissible heuristic】

> **定义（admissible）**：启发式 $h$ **可采纳**，当
> $$\boxed{\;h(n)\le h^*(n)\quad\forall n\;}$$
> 其中 $h(n)$ 为 $h$ 给出的从 $n$ 到目标的代价，$h^*(n)$ 为从 $n$ 到目标的**最优代价**。即 $h$ **从不高估**到达目标的代价。

**例（S4，15-数码 / 8-puzzle）**：
- **Hamming / 错位块数**：错位棋子总数。可采纳，因每个不在位的棋子至少要移动一次。
- **Manhattan 距离**：各棋子到其目标位置的（横+纵）距离之和。可采纳。S4 例给 $h(n)=36$。

**构造可采纳启发式的方法（S4）**：
- **松弛问题（relaxed problem）**：去掉部分约束后的精确解代价 ≤ 原问题（如允许棋子重叠/穿墙），故为下估计。
- **模式数据库（pattern databases）**：存子问题精确解。
- **归纳学习方法**。

> **定理（A* + 可采纳 ⇒ 最优，tree-search，S4）**：用可采纳启发式引导、"只推进评价最低的路径"、并在终止前关闭所有最优路径的算法，"只能在最优路径上终止"。

**A* 最优性的标准反证法（Russell–Norvig 经典证明，`\rebuilt` 据 S4 思路 + 标准 AI 教材补全完整链条）**：
> **命题**：A*（tree-search，$h$ 可采纳）返回的解是最优解。
> **证明（反证）**：设 $C^*$ 为最优解代价。设 A* 选中一个**次优目标** $G_2$（即 $g(G_2)>C^*$，因目标处 $h(G_2)=0$，故 $f(G_2)=g(G_2)>C^*$）。在 A* 选中 $G_2$ 的那一刻，最优路径上必有某节点 $n$ 仍在 open（fringe）中（因起点在最优路径上、目标 $G_2$ 不在最优路径上，最优路径未被完全展开）。对该 $n$：
> $$f(n)=g(n)+h(n)\le g(n)+h^*(n)=C^*\quad(\text{因 }h(n)\le h^*(n)\text{ 可采纳，且 }g(n)+h^*(n)\text{ 是过 }n\text{ 的最优路径代价}=C^*).$$
> 于是 $f(n)\le C^*<f(G_2)$。但 A* 总是从 open 中取 **$f$ 最小** 者，故应先扩展 $n$（$f(n)\le C^*<f(G_2)$），**与 A* 选中 $G_2$ 矛盾**。∴ A* 不会返回次优解。∎
> （来源说明：S4 仅给"直觉证明"，此处补全的不等式链 $f(n)=g(n)+h(n)\le g(n)+h^*(n)=C^*$ 是 Russell & Norvig *AIMA* §3.5.2 的标准证明，标 `\rebuilt` 待综合 agent 对 AIMA 原文核字句。）

> **可采纳但 graph-search 须谨慎（S4）**：在 **graph search**（关闭重复状态）中，仅 admissible **不足**以保最优——若 $h$ 可采纳但**不一致**，则一个节点每次以更低的"目前最佳"代价被发现时**需重新扩展**（reopen）。要在 graph-search 中无重复地保最优，需 **consistent**（见 §2.5）。

## §2.5 一致（单调）启发式 Consistent / Monotone Heuristic【S4 Consistent heuristic，含完整证明】

> **定义（consistent / monotone，S4 逐字）**：启发式 $h$ **一致**，当满足
> $$\boxed{\;h(N)\le c(N,P)+h(P)\quad\text{对每条边 }(N,P)\;}\qquad\text{且}\qquad \boxed{\;h(G)=0\ \text{对每个目标 }G\;}$$
> 其中 $N$ 为任一节点，$P$ 为其后继，$c(N,P)$ 为边代价。（直觉：启发式满足"三角不等式"——从 $N$ 的估计 ≤ 走一步到邻居 $P$ 的实际代价 + 从 $P$ 的估计。）

> **定理 1（一致 ⇒ 可采纳，S4 归纳证明，逐字保真）**：
> 设目标节点 $h(N_0)=0$。**基例**：$0\le 0$ 平凡成立。**归纳**：由一致性反复应用，
> $$h(N_{i+1})\le c(N_{i+1},N_i)+h(N_i)\le c(N_{i+1},N_i)+c(N_i,N_{i-1})+\cdots+c(N_1,N_0)+h(N_0).$$
> 右端 = 从 $N_{i+1}$ 沿该路径到目标 $N_0$ 的实际代价（$h(N_0)=0$）。对**任意**到目标的路径如此，故 $h(N_{i+1})\le h^*(N_{i+1})$（取最优路径），即可采纳。∎

> **定理 2（一致 ⇒ $f$ 沿任一路径单调非减，S4 逐字）**：
> 设 $P$ 为 $N$ 的后继。则
> $$\boxed{\;f(P)=g(P)+h(P)=g(N)+c(N,P)+h(P)\ \ge\ g(N)+h(N)=f(N)\;}$$
> （第一不等号正是一致性 $h(P)\ge h(N)-c(N,P)$，即 $c(N,P)+h(P)\ge h(N)$）。故 $f$ 沿任一路径**单调非减**。∎

> **推论（一致 ⇒ A* 弹出即最优，S4）**：一致启发式下，$f$ 值单调非减，**当 A* 扩展（弹出）一个节点时，到该节点的最优路径已被找到**（其 $g$ 值已最优）。故 A* **每个节点至多处理一次**（无需 reopen），且保证最优。
>
> **等价视角（S4 / A* 条目）**：一致启发式下，A* **等价于在"约化代价（reduced cost）" $c'(N,P)=c(N,P)-h(N)+h(P)\ge 0$ 上跑 Dijkstra**。（约化代价非负正是一致性；这把 A* 严格归约为 Dijkstra。）

**一致 vs 可采纳的关系（S4）**：**所有一致启发式都可采纳；反之不真**——存在可采纳但不一致的启发式。实践中由松弛问题/三角不等式得到的启发式（Manhattan、Euclidean、对角）通常**既可采纳又一致**。

## §2.6 A* 的复杂度与最优效率【S4 A* 条目】

**展开节点的刻画（S4）**：A* 展开**所有满足 $f(n)=g(n)+h(n)\le C^*$ 的节点**（$C^*$ 为最优目标代价），加上部分 $f=C^*$ 的节点（取决于 tie-breaking）。
**上界（S4）**：设 $\varepsilon$ 为不同节点间最小的 $f$ 值差，则 A* 展开至多 $O(C^*/\varepsilon)$ 个节点（粗略）。
**最坏复杂度（S4）**：时间/空间 $O(b^d)$，$b$ = 分支因子，$d$ = 最浅解深度（与 BFS 同量级；启发式好时远低于此）。
**与 Dijkstra/DP 的关系（S4）**：Dijkstra 是 A* 在 $h\equiv 0$ 的特例；A* 与 Dijkstra 都是动态规划的特例。
**最优效率（S4，Dechter & Pearl）**："给定一致启发式，A* 对所有 'non-pathological'（仅 tie-breaking 不同）的搜索问题，在所有可采纳的 A*-like 搜索算法中是**最优高效**的"——即任何能保证最优的同类算法都不会比 A* 少扩展节点（除 tie-breaking 外）。

## §2.7 栅格地图（Grid Maps）的启发式【S5 Amit Patel（Stanford）】

栅格地图把 $\mathcal C_{free}$ 离散为方格图（cells），相邻格之间连边后跑 A*。设 $D$ = 相邻格（正交）移动的最小代价，$\mathrm{dx}=|x_{goal}-x_n|$，$\mathrm{dy}=|y_{goal}-y_n|$。下列启发式公式逐字保真（S5）：

> **Manhattan 距离（4-邻接，只许上下左右）**：
> $$\boxed{\;h(n)=D\cdot(\mathrm{dx}+\mathrm{dy})\;}$$

> **对角距离（8-邻接，许斜走）**：设斜走一步代价 $D_2$，则
> $$\boxed{\;h(n)=D\cdot(\mathrm{dx}+\mathrm{dy})+(D_2-2D)\cdot\min(\mathrm{dx},\mathrm{dy})\;}$$
> 等价写法 $h(n)=D\cdot\max(\mathrm{dx},\mathrm{dy})+(D_2-D)\cdot\min(\mathrm{dx},\mathrm{dy})$。特例：
> - **Chebyshev 距离**：$D=1,\ D_2=1$ ⇒ $h=\max(\mathrm{dx},\mathrm{dy})$。
> - **Octile 距离**：$D=1,\ D_2=\sqrt2$ ⇒ $h=\max(\mathrm{dx},\mathrm{dy})+(\sqrt2-1)\min(\mathrm{dx},\mathrm{dy})$。

> **Euclidean 距离（任意角度移动）**：
> $$\boxed{\;h(n)=D\cdot\sqrt{\mathrm{dx}^2+\mathrm{dy}^2}\;}$$

> **关键陷阱（S5 逐字）**：**不要用平方欧氏距离** $D\cdot(\mathrm{dx}^2+\mathrm{dy}^2)$！"当 A* 计算 $f=g+h$ 时，距离的平方会远大于代价 $g$，导致 **高估** 启发式" → 失最优、且行为怪异（量纲不匹配）。

> **量纲匹配（S5）**：启发式返回的单位（米、分钟……）必须与代价函数 $g$ 的单位一致。

> **欠估/高估的权衡（S5）**：
> - $h$ 欠估（含 $h\equiv 0$=Dijkstra）：保证最短路，但**扩展更多节点**（更慢）。
> - $h$ **恰等** $h^*$：A* 只沿最优路径扩展（最快），但一般不可得。
> - $h$ 高估：放弃最优性换速度（探索更少节点）。

> **Tie-breaking（打破平局，S5）**：当多节点 $f$ 相等时 A* 路径可能"摊开"成方块状。可对启发式乘一极小因子偏向目标方向：
> $$h\ \mathrel{*}=\ (1.0+p),\qquad p\approx\frac{1}{\text{预期最大路径长度}}$$
> （$p$ 足够小则仍近似可采纳，能显著减少等价 $f$ 的探索。）

---

# 第三部分　采样式规划：RRT / PRM / RRT*【S1 Ch.5 + S2 + S4】

> **动机（S1 §4.3.1 + §5）**：高维 C-空间中显式构造 $\mathcal C_{obs}$ 边界既难又指数爆炸。采样式方法**回避显式 $\mathcal C_{obs}$**，仅依赖一个**碰撞检测模块**（布尔判断某构型/某条线段是否在 $\mathcal C_{free}$），把 $\mathcal C_{free}$ 随机/确定性地离散成图，再用第二部分的图搜索取路径。

## §3.1 通用原语（Primitive Procedures）【S2 §3.1，逐字保真】

S2 给出所有采样算法共用的原语（C-空间 $X=(0,1)^d$，$d\ge 2$）：

- **采样 Sampling**：$\mathrm{Sample}:\omega\mapsto\{\mathrm{Sample}_i(\omega)\}_{i\in\mathbb N_0}\subset X$，$\mathrm{Sample}_i$ 独立同分布（i.i.d.，默认均匀分布，结论可推广到密度在 $X$ 上有正下界的任意绝对连续分布）。$\mathrm{SampleFree}$ 返回 $X_{free}$ 中 i.i.d. 样本子列。
- **最近邻 Nearest**（欧氏度量）：
  $$\mathrm{Nearest}(G=(V,E),x):=\arg\min_{v\in V}\|x-v\|.$$
  集合版 $\mathrm{kNearest}(G,x,k)\mapsto\{v_1,\dots,v_k\}$ 返回 $V$ 中离 $x$ 最近的 $k$ 个（$|V|<k$ 时返回 $V$）。
- **邻域顶点 Near**：给定半径 $r>0$，
  $$\mathrm{Near}(G=(V,E),x,r):=\{v\in V:v\in \mathcal B_{x,r}\}\quad(\mathcal B_{x,r}=\text{以 }x\text{ 为心、}r\text{ 为半径的球}).$$
- **转向 Steering**：给定 $x,y\in X$，$\mathrm{Steer}(x,y)\mapsto z$ 返回比 $x$ "更靠近" $y$ 的点 $z$，约束 $\|z-x\|\le\eta$（$\eta>0$ 预设）：
  $$\boxed{\;\mathrm{Steer}(x,y):=\arg\min_{z\in\mathcal B_{x,\eta}}\|z-y\|\;}$$
- **碰撞测试 CollisionFree**：$\mathrm{CollisionFree}(x,x')$ 返回 True 当且仅当线段 $[x,x']\subset X_{free}$。

## §3.2 RRT（快速探索随机树）【S1 §5.5 + S2 Alg.3 + S4】

### §3.2.1 LaValle 的 RDT/RRT 探索算法【S1 §5.5.1，逐字伪码】

LaValle 的 RRT 属"**快速探索稠密树（RDT）**"族（序列随机⇒RRT；确定性⇒RDT）。设 $\alpha$ 为 $\mathcal C$ 中**无穷稠密序列**（均匀随机序列以概率 1 稠密），$\alpha(i)$ 为第 $i$ 样本。RDT 是拓扑图 $G(V,E)$，其**swath（覆盖集）** $S=\bigcup_{e\in E}e([0,1])\subset\mathcal C_{free}$（图上所有点）。

> **算法（S1 图 5.16，无障碍版 SIMPLE_RDT，逐行）**：
> ```
> SIMPLE_RDT(q0)
>  1  G.init(q0);
>  2  for i = 1 to k do
>  3      G.add_vertex(α(i));
>  4      q_n ← nearest(S(G), α(i));      // 注意 nearest 取自整个 swath S，可在边内部
>  5      G.add_edge(q_n, α(i));
> ```
> **边分裂细节（S1 §5.5.1）**：若最近点 $q_n$ 是顶点，则连边 $q_n\to\alpha(i)$；若 $q_n$ 落在某条边**内部**，则该边在 $q_n$ 处**分裂**，$q_n$ 成为新顶点，再连 $q_n\to\alpha(i)$。每次迭代边数可增 1 或 2。

> **算法（S1 图 5.21，有障碍版 RDT，逐行）**：
> ```
> RDT(q0)
>  1  G.init(q0);
>  2  for i = 1 to k do
>  3      q_n ← nearest(S, α(i));
>  4      q_s ← stopping-configuration(q_n, α(i));
>  5      if q_s ≠ q_n then
>  6          G.add_vertex(q_s);
>  7          G.add_edge(q_n, q_s);
> ```
> `stopping-configuration(q_n, α(i))`：沿 $q_n\to\alpha(i)$ 方向，给出在撞 $\mathcal C_{obs}$ 边界前**所能到达的最靠近边界的构型** $q_s$（碰撞检测决定能多近）。若 $q_s\neq q_n$ 才加点加边。

> **本质洞察：Voronoi 偏置（S1 §5.5.1 + S4，逐字综合）**：RDT/RRT 的"快速探索"源于——`nearest` 把新样本连到树上最近点，**等价于"被选中扩展的顶点 $x$ 的概率正比于其 Voronoi 区域 $\mathrm{Vor}(x)$ 的体积"**（S4："The probability of expanding an existing state is proportional to the size of its Voronoi region... the tree preferentially expands towards large unsearched areas"）。**大 Voronoi 区域 = 大片未探索空间**，故树被"拉"向边界/未探索区，呈分形式快速铺满（S1 图 5.19：45 次迭代即触达四角，2345 次渐稠密）。**极限以概率 1 稠密**（dense in the limit），即任意构型可被任意接近。

### §3.2.2 Karaman-Frazzoli 的 RRT【S2 Alg.3，逐字伪码】

> **算法（S2 Algorithm 3，RRT，逐行）**：
> ```
> RRT
>  1  V ← {x_init};  E ← ∅;
>  2  for i = 1, …, n do
>  3      x_rand   ← SampleFree_i;
>  4      x_nearest ← Nearest(G=(V,E), x_rand);
>  5      x_new    ← Steer(x_nearest, x_rand);          // 前进至多 η
>  6      if ObstacleFree(x_nearest, x_new) then
>  7          V ← V ∪ {x_new};  E ← E ∪ {(x_nearest, x_new)};
>  8  return G = (V, E);
> ```
> （S2 注：原版一旦树触达目标即停；此处为与 PRM 一致改为固定迭代 $n$ 次。无障碍时构造出的是 online nearest neighbor graph。双树变体 = 分别从起点、目标长树，亦称 RDT。）

### §3.2.3 Wikipedia 的 RRT 与 goal-bias【S4】

> **算法（S4 BuildRRT，逐字）**：
> ```
> Algorithm BuildRRT
>   Input:  q_init, K, Δq
>   Output: RRT graph G
>   G.init(q_init)
>   for k = 1 to K do
>       q_rand ← RAND_CONF()
>       q_near ← NEAREST_VERTEX(q_rand, G)
>       q_new  ← NEW_CONF(q_near, q_rand, Δq)    // 从 q_near 朝 q_rand 前进增量 Δq
>       G.add_vertex(q_new)
>       G.add_edge(q_near, q_new)
>   return G
> ```
> - **RAND_CONF()**：随机构型（碰撞检测版 RAND_FREE_CONF 只取 $\mathcal C_{free}$）。
> - **NEAREST_VERTEX()**：$G$ 中离 $q_{rand}$ 最近顶点。
> - **NEW_CONF()**：从 $q_{near}$ 朝 $q_{rand}$ 移动增量 $\Delta q$。
>
> **目标偏置（goal bias，S4）**：实践中以**小概率 $p$ 直接采样目标**（把 $q_{rand}$ 设为目标），引导树朝目标生长。

**RRT 变体（S4，一句话）**：RRT*（趋最优）、LQR-RRT（kinodynamic）、RRT*-Smart（加速收敛）、Informed RRT*（启发式约束采样椭球）、RT-RRT*（实时动态环境）、RRTX/RRT#（动态再优化）、RRT-Connect（双向）。

## §3.3 PRM（概率路标图 / 采样式路标）【S1 §5.6 + S2 Alg.1,2 + S4】

PRM 面向**多查询**（multiple-query）：一次预处理建路标图，反复回答不同 $(q_I,q_G)$。两阶段（S1 §5.6.1）：
- **预处理阶段（Preprocessing）**：投入大量计算建图 $G$，使其"可从 $\mathcal C_{free}$ 各处可达"（roadmap 之名由来）。
- **查询阶段（Query）**：给定 $(q_I,q_G)$，用局部规划器（LPM）把二者各自接入 $G$，再用 **§2.2 的任一离散搜索（如 Dijkstra）** 取从 $q_I$ 到 $q_G$ 的边序列。

> **算法（S1 图 5.25，BUILD_ROADMAP，逐行）**：
> ```
> BUILD_ROADMAP
>  1  G.init();  i ← 0;
>  2  while i < N
>  3      if α(i) ∈ C_free then
>  4          G.add_vertex(α(i));  i ← i + 1;
>  5          for each q ∈ neighborhood(α(i), G)
>  6              if ((not G.same_component(α(i), q)) and connect(α(i), q)) then
>  7                  G.add_edge(α(i), q);
> ```
> （S1：$\alpha(i)\in\mathcal C_{obs}$ 时**不**自增 $i$，使 $i$ 正确计顶点数。`connect` 是 LPM——通常测 $\alpha(i)$ 与 $q$ 间最短直线段是否无碰；第 6 行 `not same_component` 是为效率，避免在同一连通分量内重复连边，使 PRM 成森林。）

> **邻域选取（S1 §5.6.1）**：`neighborhood` 常取 **最近 $K$ 个**（典型 $K=20$）或 **半径 $r$ 球内全部**；样本规则（如栅格）时二者近似等价，样本不规则（随机）时取最近 $K$ 更佳。半径可随点数增多自适应缩小（按 dispersion/discrepancy）。

> **S2 的 sPRM（简化 PRM，Algorithm 2，逐行）**——理论分析对象（允许同分量内连接）：
> ```
> sPRM
>  1  V ← {x_init} ∪ {SampleFree_i}_{i=1,…,n};   E ← ∅;
>  2  foreach v ∈ V do
>  3      U ← Near(G=(V,E), v, r) \ {v};
>  4      foreach u ∈ U do
>  5          if CollisionFree(v, u) then  E ← E ∪ {(v,u), (u,v)};
>  6  return G = (V, E);
> ```
> （无障碍时 sPRM 建出的就是 random $r$-disc graph。）

> **S2 的 PRM 预处理（Algorithm 1，含 same-component 剪枝、按距离递增尝试，逐行）**：
> ```
> PRM (preprocessing)
>  1  V ← ∅;  E ← ∅;
>  2  for i = 0, …, n do
>  3      x_rand ← SampleFree_i;
>  4      U ← Near(G=(V,E), x_rand, r);
>  5      V ← V ∪ {x_rand};
>  6      foreach u ∈ U, in order of increasing ‖u − x_rand‖, do
>  7          if x_rand and u are not in the same connected component of G then
>  8              if CollisionFree(x_rand, u) then  E ← E ∪ {(x_rand,u), (u,x_rand)};
>  9  return G = (V, E);
> ```

**(s)PRM 的实用变体（S2 §3.2）**：
- **k-Nearest (s)PRM**：连最近 $k$ 邻（典型 $k=15$）。无障碍时为 random $k$-nearest graph。
- **Bounded-degree (s)PRM**：$U\leftarrow\mathrm{Near}(G,x,r)\cap\mathrm{kNearest}(G,x,k)$（典型 $k=20$），限制度数。
- **Variable-radius (s)PRM**：半径 $r$ 随 $n$ 变（但 S2 指出文献无明确 $r$–$n$ 关系——这正是 PRM* 要补的）。

**完备性（S4）**：PRM "可证概率完备：采样点数无界增长时，若有解则找到解的概率趋 1"；性能依赖**可见性**（点能'看见'大片空间、子集间互见性好则成功）。

## §3.4 最优采样规划：PRM*、RRG、RRT*【S2，全量定理保真】

S2 的核心贡献：标准 PRM/RRT **不渐近最优**；提出 PRM*/RRG/RRT*，在与原算法同阶复杂度下**可证渐近最优**。

### §3.4.1 关键尺度：连接半径与 k 的公式【S2 §3.3，逐字常数】

> **PRM\* 连接半径（S2 Algorithm 4 / Theorem 34）**：
> $$\boxed{\;r(n)=\gamma_{PRM}\left(\frac{\log n}{n}\right)^{1/d},\qquad \gamma_{PRM}>\gamma_{PRM}^*=2\left(1+\frac1d\right)^{1/d}\left(\frac{\mu(X_{free})}{\zeta_d}\right)^{1/d}\;}$$
> 其中 $d$=维数，$\mu(X_{free})$=自由空间的勒贝格测度（体积），$\zeta_d$=$d$ 维单位球体积。半径随 $n$ 减小，使每顶点平均连接数正比于 $\log n$。
> （S2 指出：$n$ 个均匀随机点的 **dispersion 是 $O((\log n/n)^{1/d})$**，恰是该半径的衰减率——这是公式的几何来由。）

> **k-nearest PRM\*（S2 Theorem 35）**：
> $$\boxed{\;k(n)=k_{PRM}\log n,\qquad k_{PRM}>k_{PRM}^*=e\left(1+\frac1d\right)\;}$$
> （$k_{PRM}^*$ **只依赖 $d$**，不依赖问题实例（与 $\gamma_{PRM}^*$ 不同）；$k_{PRM}=2e$ 对所有实例都有效。）

> **RRG 连接半径（S2 §3.3）**：
> $$r(\mathrm{card}(V))=\min\left\{\gamma_{RRG}\left(\frac{\log(\mathrm{card}\,V)}{\mathrm{card}\,V}\right)^{1/d},\ \eta\right\},\qquad \gamma_{RRG}>\gamma_{RRG}^*=2\left(1+\frac1d\right)^{1/d}\left(\frac{\mu(X_{free})}{\zeta_d}\right)^{1/d}$$
> （$\eta$=Steer 步长上限。）k-nearest RRG：$k=k_{RRG}\log(\mathrm{card}\,V)$，$k_{RRG}>k_{RRG}^*=e(1+1/d)$，$k_{RRG}=2e$ 通用。

> **RRT\* 重连半径（S2 §3.3 / Theorem 38）**：
> $$\boxed{\;r(\mathrm{card}(V))=\min\left\{\gamma_{RRT^*}\left(\frac{\log(\mathrm{card}\,V)}{\mathrm{card}\,V}\right)^{1/d},\ \eta\right\},\qquad \gamma_{RRT^*}>\left(2\left(1+\frac1d\right)\right)^{1/d}\left(\frac{\mu(X_{free})}{\zeta_d}\right)^{1/d}\;}$$
> k-nearest RRT*：$k(\mathrm{card}\,V)=k_{RRG}\log(\mathrm{card}\,V)$，Theorem 39 要 $k_{RRT^*}>2^{d+1}e(1+1/d)$。

### §3.4.2 PRM*、RRG、RRT* 算法【S2 Alg.4,5,6，逐字伪码】

> **算法（S2 Algorithm 4，PRM\*）**：同 sPRM，仅把固定 $r$ 换成 $r(n)=\gamma_{PRM}(\log n/n)^{1/d}$：
> ```
> PRM*
>  1  V ← {x_init} ∪ {SampleFree_i}_{i=1,…,n};   E ← ∅;
>  2  foreach v ∈ V do
>  3      U ← Near(G=(V,E), v, γ_PRM (log(n)/n)^{1/d}) \ {v};
>  4      foreach u ∈ U do
>  5          if CollisionFree(v, u) then  E ← E ∪ {(v,u), (u,v)};
>  6  return G = (V, E);
> ```

> **算法（S2 Algorithm 5，RRG）**——RRT 的基础上，每加新点就向半径球内**所有**可达顶点连边（成图，可含环）：
> ```
> RRG
>  1  V ← {x_init};  E ← ∅;
>  2  for i = 1, …, n do
>  3      x_rand   ← SampleFree_i;
>  4      x_nearest ← Nearest(G=(V,E), x_rand);
>  5      x_new    ← Steer(x_nearest, x_rand);
>  6      if ObstacleFree(x_nearest, x_new) then
>  7          X_near ← Near(G=(V,E), x_new, min{γ_RRG (log(card V)/card V)^{1/d}, η});
>  8          V ← V ∪ {x_new};  E ← E ∪ {(x_nearest, x_new), (x_new, x_nearest)};
>  9          foreach x_near ∈ X_near do
> 10              if CollisionFree(x_near, x_new) then  E ← E ∪ {(x_near,x_new),(x_new,x_near)};
> 11  return G = (V, E);
> ```
> （S2：同采样序列下，RRT 图（有向树）是 RRG 图（无向、可含环）的**子图**；二者顶点集相同，RRT 边集 ⊆ RRG 边集。）

> **算法（S2 Algorithm 6，RRT\*）**——把 RRG 改成**无环树**：去冗余边，使每顶点经**最小代价路径**到达（即对 RRT 树做"rewiring"）。需先定义：$\mathrm{Line}(x_1,x_2):[0,s]\to X$ 为 $x_1\to x_2$ 直线路径；$\mathrm{Parent}:V\to V$ 取父顶点（根的父是自身）；$\mathrm{Cost}:V\to\mathbb R_{\ge0}$ 为根到该顶点唯一路径的代价（可加代价时 $\mathrm{Cost}(v)=\mathrm{Cost}(\mathrm{Parent}(v))+c(\mathrm{Line}(\mathrm{Parent}(v),v))$，根 $\mathrm{Cost}=0$）。
> ```
> RRT*
>  1  V ← {x_init};  E ← ∅;
>  2  for i = 1, …, n do
>  3      x_rand   ← SampleFree_i;
>  4      x_nearest ← Nearest(G=(V,E), x_rand);
>  5      x_new    ← Steer(x_nearest, x_rand);
>  6      if ObstacleFree(x_nearest, x_new) then
>  7          X_near ← Near(G=(V,E), x_new, min{γ_RRT* (log(card V)/card V)^{1/d}, η});
>  8          V ← V ∪ {x_new};
>  9          x_min ← x_nearest;  c_min ← Cost(x_nearest) + c(Line(x_nearest, x_new));
> 10          foreach x_near ∈ X_near do          // —— 沿最小代价路径连接（choose parent）
> 11              if CollisionFree(x_near, x_new) ∧ Cost(x_near)+c(Line(x_near,x_new)) < c_min then
> 12                  x_min ← x_near;  c_min ← Cost(x_near) + c(Line(x_near, x_new));
> 13          E ← E ∪ {(x_min, x_new)};
> 14          foreach x_near ∈ X_near do          // —— 重连树（rewire）
> 15              if CollisionFree(x_new, x_near) ∧ Cost(x_new)+c(Line(x_new,x_near)) < Cost(x_near) then
> 16                  x_parent ← Parent(x_near);
> 17                  E ← (E \ {(x_parent, x_near)}) ∪ {(x_new, x_near)};
> 18  return G = (V, E);
> ```
> **RRT* 两步精髓（S2 §3.3）**：(i) **choose parent**——从 $X_{near}$ 中选能以**最小代价**连到 $x_{new}$ 的顶点做父；(ii) **rewire**——若经 $x_{new}$ 到某 $x_{near}$ 比其当前路径更省，则把 $x_{near}$ 的父改为 $x_{new}$（删旧边、加新边），保持树结构。

## §3.5 性质：概率完备性、渐近最优性、复杂度【S2，全部定理逐字保真】

### §3.5.1 概率完备性【S2 §4.1】

**鲁棒可行（robustly feasible，S2）**：路径 $\sigma$ 有 **strong $\delta$-clearance**，若其整条都在 $X_{free}$ 的 $\delta$-内部 $\mathrm{int}_\delta(X_{free})$（每点离障碍 $\ge\delta$）。问题 $(X_{free},x_{init},X_{goal})$ **鲁棒可行**，若存在某 $\delta>0$ 使有 strong $\delta$-clearance 的解。

> **定义 14（概率完备性 Probabilistic Completeness，S2 逐字）**：算法 ALG 概率完备，若对任意鲁棒可行的问题，
> $$\boxed{\;\liminf_{n\to\infty}\mathbb P\{\exists\,x_{goal}\in V_n^{ALG}\cap X_{goal}\text{ s.t. }x_{init}\text{ connected to }x_{goal}\text{ in }G_n^{ALG}\}=1\;}$$
> （问题不鲁棒可行时，除非用奇异分布适配问题，否则任何采样算法该极限为 0。）

> **定理 15（sPRM 概率完备，Kavraki et al. 1998；S2 逐字）**：对鲁棒可行问题，存在仅依赖 $X_{free},X_{goal}$ 的常数 $a>0,\ n_0\in\mathbb N$ 使
> $$\mathbb P\{\exists x_{goal}\in V_n^{sPRM}\cap X_{goal}:x_{goal}\text{ connected to }x_{init}\text{ in }G_n^{sPRM}\}>1-e^{-an},\quad\forall n>n_0.$$
> （即找到解的概率随顶点数**指数快**趋 1。）

> **定理 16（RRT 概率完备，LaValle & Kuffner 2001；S2 逐字）**：存在仅依赖 $X_{free},X_{goal}$ 的 $a>0,n_0$ 使
> $$\mathbb P\{V_n^{RRT}\cap X_{goal}\neq\varnothing\}>1-e^{-an},\quad\forall n>n_0.$$

> **定理 17（k=1 时 k-nearest sPRM 不完备，S2）**：1-nearest sPRM（每点只连最近邻）**不**概率完备；且其找不到解的概率随 $n\to\infty$ 趋 1。（对比：RRT 是 1-nearest sPRM 的增量版但强制连通，故完备——见定理 16。）

> **定理 20（变半径 sPRM with $r(n)=\gamma n^{-1/d}$ 不完备，S2）**：存在使其不完备的实例。（说明 $n^{-1/d}$ 衰减**太快**；正确尺度须带 $\log$，即 $(\log n/n)^{1/d}$。）

> **定理 22（PRM\* 概率完备，S2）**：PRM* 概率完备。

> **定理 23（RRG 与 RRT\* 概率完备，S2 逐字）**：RRG 与 RRT* 概率完备；对鲁棒可行问题，存在 $a>0,n_0$ 使
> $$\mathbb P\{V_n^{RRG}\cap X_{goal}\neq\varnothing\}>1-e^{-an},\quad \mathbb P\{V_n^{RRT^*}\cap X_{goal}\neq\varnothing\}>1-e^{-an},\quad\forall n>n_0.$$

### §3.5.2 渐近最优性【S2 §4.2】

**弱 $\delta$-clearance 与鲁棒最优解（S2，渐近最优的技术前提）**：路径 $\sigma$ 有 **weak $\delta$-clearance**，若存在有 strong $\delta$-clearance 的路径 $\sigma'$ 及同伦 $\psi$（$\psi(0)=\sigma,\psi(1)=\sigma'$）使每个 $\alpha\in(0,1]$ 对应的 $\psi(\alpha)$ 有 strong $\delta_\alpha$-clearance（$\delta_\alpha>0$）。可行解 $\sigma^*$ 是**鲁棒最优解**，若它有 weak $\delta$-clearance 且对任意 $\sigma_n\to\sigma^*$（BV 范数）有 $c(\sigma_n)\to c(\sigma^*)$。记 $c^*=c(\sigma^*)$，$Y_n^{ALG}$ = ALG 第 $n$ 次迭代图中最小代价解的代价。

（路径空间范数：$\|\sigma\|_{BV}=\int_0^1|\sigma(\tau)|\,d\tau+\mathrm{TV}(\sigma)$，$\mathrm{TV}$ 为全变差≈长度；$\mathrm{dist}(\sigma_1,\sigma_2)=\|\sigma_1-\sigma_2\|_{BV}$。）

> **定义 24（渐近最优性 Asymptotic Optimality，S2 逐字）**：算法 ALG 渐近最优，若对任意有有限代价鲁棒最优解 $c^*$ 的问题与代价函数，
> $$\boxed{\;\mathbb P\Big\{\limsup_{n\to\infty}Y_n^{ALG}=c^*\Big\}=1\;}$$
> （因 $Y_n^{ALG}\ge c^*$，故等价于 $\lim_{n\to\infty}Y_n^{ALG}=c^*$ a.s.）**概率完备是渐近最优的必要条件**。且采样算法收敛到最优解的概率非 0 即 1（**0–1 律**：要么几乎所有运行都收敛到最优，要么几乎都不）。

> **定理 29（PRM 不渐近最优，S2）**：标准 PRM 不渐近最优。
> **定理 31（k-nearest sPRM 不渐近最优，任意常数 $k$，S2）**。
> **定理 32（变半径 sPRM with $r(n)=\gamma n^{-1/d}$ 不渐近最优，S2）**。

> **定理 33（RRT 不渐近最优，S2 逐字，本主题关键负面结果）**：RRT 算法**不**渐近最优。证明见 S2 附录 B。因每次迭代 RRT 要么加一点一边、要么不变，故 $G_i^{RRT}(\omega)\subseteq G_{i+1}^{RRT}(\omega)$，$\lim_{n\to\infty}Y_n^{RRT}$ 存在 = 随机变量 $Y_\infty^{RRT}$。结合 Lemma 25，定理 33 蕴含此极限**几乎必然严格大于 $c^*$**：
> $$\boxed{\;\mathbb P\{\lim_{n\to\infty}Y_n^{RRT}>c^*\}=1\;}$$
> **即 RRT 返回的最佳解代价以概率 1 收敛到一个次优值。** 更甚：可构造实例使 RRT 首解代价任意高的概率有正下界（Nechushtan et al. 2010）。
> （这解释了为何"多次重启 RRT"有效——相当于对随机变量 $Y_\infty^{RRT}$ 多次采样取最好。）

> **定理 34（PRM\* 渐近最优，S2 逐字）**：若 $\gamma_{PRM}>2(1+1/d)^{1/d}\big(\frac{\mu(X_{free})}{\zeta_d}\big)^{1/d}$，则 PRM* 渐近最优。
> **定理 35（k-nearest PRM\* 渐近最优）**：若 $k_{PRM}>e(1+1/d)$，则成立。
> **定理 36（RRG 渐近最优）**：若 $\gamma_{RRG}>2(1+1/d)^{1/d}\big(\frac{\mu(X_{free})}{\zeta_d}\big)^{1/d}$，则成立。
> **定理 37（k-nearest RRG 渐近最优）**：若 $k_{RRG}>e(1+1/d)$，则成立。
> **定理 38（RRT\* 渐近最优，S2 逐字）**：若 $\gamma_{RRT^*}>\big(2(1+1/d)\big)^{1/d}\big(\frac{\mu(X_{free})}{\zeta_d}\big)^{1/d}$，则 RRT* 渐近最优。
> **定理 39（k-nearest RRT\* 渐近最优）**：若 $k_{RRT^*}>2^{d+1}e(1+1/d)$，则成立（证明由定理 37、38 得）。

> **证明思路（S2 §4.2，渐近最优性核心机制，`\rebuilt` 提炼自 Lemma 50–56 等）**：取有 strong $\delta_n$-clearance 的路径列 $\sigma_n\to\sigma^*$；沿 $\sigma_n$ 铺一串相互重叠的"覆盖球（covering balls，Def 51）"，球半径 $\sim r(n)$；证明半径取 $\gamma(\log n/n)^{1/d}$（$\gamma>\gamma^*$）时，**每个球以概率 1（最终）含至少一个图顶点**（这要求半径**够大以保证连通**，故 $\log$ 因子不可少；又**够小以保证收敛到 $\sigma^*$**）；于是图中存在一条贴着 $\sigma_n$ 的路径，其 BV 距离 $\|\sigma_n'-\sigma_n\|_{BV}\to0$ a.s.（Lemma 55/61/...），由代价的连续性 $c\to c^*$。RRG/RRT* 因保留/重连了到每点的最小代价路径，继承此最优路径。∎（梗概）

### §3.5.3 计算复杂度【S2 §4.3，逐字定理与证明】

记 $M_n^{ALG}$ = ALG 第 $n$ 次迭代调用 CollisionFree 的次数。$W_n^{ALG}\in\Omega(f(n))$ 指存在实例使 $\liminf_n\mathbb E[W_n/f(n)]>0$；$\in O(f(n))$ 类似。

> **引理 40（PRM）**：$M_n^{PRM}\in\Omega(n)$。
> **证明（S2 逐字）**：取 $X_{free}=X_1\cup X_2$（开不交并，图 9），$X_2$ 为一边长 $r/2$ 的超矩形（$r$=连接半径）。任何以 $X_2$ 中点为心的 $r$-球必含 $X_2$ 的非零测度部分。令 $\bar\mu:=\inf_{x\in X_2}\mu(\mathcal B_{x,r}\cap X_1)>0$。则落入 $X_2$ 的样本 $X_n$ 会尝试连到 $X_1$ 中子集 $X_1'$（$\mu(X_1')\ge\bar\mu$）的若干顶点，其期望数 $\ge\bar\mu n$，且这些顶点都不与 $X_n$ 同分量。故 $\mathbb E[M_n^{PRM}/n]>\bar\mu$，取下极限得证。∎

> **引理 41（sPRM）**：$M_n^{sPRM}\in\Omega(n)$。
> **证明（S2 逐字，更强结论）**：对**所有**实例 $\liminf_n\mathbb E[M_n^{sPRM}/n]>0$。令 $\bar\mu:=\inf_{x\in X_{free}}\mu(\mathcal B_{x,r}\cap X_{free})>0$（$X_{free}$ 是开集闭包）。第 $n$ 次 CollisionFree 调用数 = 以最后样本 $X_n$ 为心 $r$-球内的节点数；球内 $X_{free}$ 体积 $\ge\bar\mu$，故 $\mathbb E[M_n^{sPRM}]\ge\frac{\bar\mu}{\mu(X_{free})}n$，即 $\mathbb E[M_n/n]\ge\bar\mu/\mu(X_{free})\ \forall n$，取下极限得证。∎
> （另：k-nearest PRM 有 $M_n^{k\text{-}sPRM}=k\ (\forall n>k)$；RRT 有 $M_n^{RRT}=1\ \forall n$。）

> **引理 42（PRM\*、RRG、RRT\*）**：$M_n^{PRM^*},\ M_n^{RRG},\ M_n^{RRT^*}\in O(\log n)$。
> **证明（S2 梗概，逐字起首）**：以 PRM* 为例，连接半径 $r_n=\gamma_{PRM}(\log n/n)^{1/d}$。设 $A$ = 末次样本 $X_n$ 落入 $r_n$-内部 $\mathrm{int}_{r_n}(X_{free})$ 的事件，则 $\mathbb E[M_n^{PRM^*}]=\mathbb E[M_n^{PRM^*}\mid A]\mathbb P(A)+\mathbb E[M_n^{PRM^*}\mid A^c]\mathbb P(A^c)$。对 $n\ge n_0$（使 $\mu(\mathrm{int}_{r_n}(X_{free}))>0$），$\mathbb E[M_n^{PRM^*}\mid A]=\zeta_d\gamma_{PRM}^d\cdots$（正比于球体积 $\times$ 密度 $\sim \zeta_d r_n^d\cdot n=\zeta_d\gamma_{PRM}^d\log n$），即 $O(\log n)$。∎（完整见 S2。）

> **复杂度总结（S2 §4.3 + S4）**：PRM*/RRT* 每次迭代仅 $O(\log n)$ 次碰撞检测（vs PRM/sPRM 的 $\Omega(n)$），故 $n$ 次总计 $O(n\log n)$ 量级——**与概率完备的原版同阶**，却换来渐近最优。

## §3.6 RRT* 最优性证明的逻辑缺口与修正【S6，Solovey et al. 2020】

> **重要勘误（S6 = arXiv:1909.09688，*Revisiting the Asymptotic Optimality of RRT\**, ICRA 2020）**：
> - **缺口**：Solovey、Janson、Schmerling、Frazzoli、Pavone 指出 **S2（Karaman-Frazzoli 2011）对 RRT\* 渐近最优性的原始证明存在一个逻辑缺口**（与 RRT* 中样本的**增量/时序依赖**有关——覆盖球论证未充分处理"样本到达顺序"这一额外维度）。
> - **修正**：他们给出严格的替代证明。其关键修正是把连接半径从 $\gamma(\log n/n)^{1/d}$ 调整为
> $$\boxed{\;r(n)=\gamma'\left(\frac{\log n}{n}\right)^{1/(d+1)}\;}\quad(\text{指数 }1/(d+1)\text{，非 }1/d)$$
> 以"计入决定样本排序的额外时间维度"。
> - **结论不变**：修正后 **RRT\* 仍渐近最优**——S6 目的是补严格性，非推翻结论，但**参数要求（半径指数 $1/(d+1)$）更严**。
>
> **给综合 agent 的提示**：成书写 RRT* 渐近最优时，**正文用 S2 定理 38 的经典表述（$1/d$）**，但**须加脚注/note 指出 S6 的缺口与 $1/(d+1)$ 修正**（标 `\cite{solovey2020revisiting}`），以保严谨。这是本主题近年最重要的理论更正。

---

# 第四部分　轨迹优化简介（Trajectory Optimization）【综述，`\rebuilt`】

> **范围说明**：本章（规划导论）对轨迹优化只需**简介**。个人笔记未同步，以下据采样/搜索式与优化式规划的标准对比综述给出，**标 `\rebuilt` 待综合 agent 据专门文献（CHOMP/STOMP/TrajOpt/iLQR 原论文）扩充**。本抽取仅给定位与最小要素，供综合 agent 衔接，**不展开各算法完整推导**（属后续"轨迹优化"专章）。

**定位（与前两部分对比）**：
- **搜索式（A*/Dijkstra）**：在**离散图**上找最优路径；分辨率受栅格/图离散限制；保证图意义下最优。
- **采样式（PRM/RRT*）**：随机离散 $\mathcal C_{free}$；高维有效；渐近最优但收敛慢、解常不光滑（需后处理平滑）。
- **轨迹优化式**：把规划写成**连续最优化问题**——给定一条初始轨迹 $\xi$（可由前两类给出），通过梯度/数值优化**直接在轨迹空间**极小化代价泛函（含光滑度 + 避障 + 动力学），得到**光滑、动力学可行**的局部最优轨迹。

**一般形式（`\rebuilt`，标准轨迹优化）**：求轨迹 $\xi:[0,T]\to\mathcal C$ 极小化
$$\min_{\xi}\ \underbrace{\mathcal U_{smooth}[\xi]}_{\text{光滑/能量项}}+\lambda\,\underbrace{\mathcal U_{obs}[\xi]}_{\text{避障代价}}\quad\text{s.t.}\ \xi(0)=q_I,\ \xi(T)=q_G,\ (\text{动力学/约束}).$$
- 光滑项常取 $\mathcal U_{smooth}=\frac12\int_0^T\|\dot\xi(t)\|^2dt$（或加加速度/jerk 项），离散后为 $\frac12\xi^\top A\xi$（$A$ 为有限差分构成的对称正定矩阵）。
- 避障项 $\mathcal U_{obs}$ 由到 $\mathcal C_{obs}$ 的距离场（signed distance field）构造，障碍内/近障碍惩罚大。

**代表方法（`\rebuilt`，一句话定位，供综合 agent 展开）**：
- **CHOMP**（Covariant Hamiltonian Optimization for Motion Planning，Ratliff et al. 2009 / Zucker et al. 2013）：协变梯度下降，用 $A$ 诱导的黎曼度量，对障碍距离场做泛函梯度。
- **STOMP**（Stochastic Trajectory Optimization，Kalakrishnan et al. 2011）：无梯度，采样噪声轨迹加权更新（适合不可微代价）。
- **TrajOpt**（Schulman et al. 2013/2014）：序列凸优化（SQP）+ 连续碰撞凸约束。
- **iLQR / DDP**：基于动力学线性化的最优控制式轨迹优化（kinodynamic）。

> **本质洞察（`\rebuilt`）**：搜索/采样式是**全局**方法（找到"哪条路"），轨迹优化是**局部**精化方法（把一条路调"多好"）。实际系统常**级联**：A*/RRT* 给初值 → 轨迹优化平滑成可执行轨迹。综合 agent 应把此"全局 + 局部"流水线作为本章收尾的过渡桥（接后续轨迹优化专章）。

---

# 附：给综合 agent 的整合要点与待核清单

1. **主线建议**：构型空间（第一部分）→ 图离散化 →〔栅格地图 + 图搜索 A*/Dijkstra（第二部分，本主题重心，最优性证明齐全）〕→ 高维则采样离散化（第三部分 PRM/RRT/RRT*，完整算法 + 概率完备/渐近最优定理）→ 轨迹优化精化（第四部分简介）。这条线把"搜索式 vs 采样式 vs 优化式"三大范式串成一个 $\mathcal C_{free}$ 离散化粒度谱。
2. **记号统一**：全书用 $f(n)=g(n)+h(n)$（而非 LaValle 的 $C^*+\hat G^*$）；连续 C-空间用 $\mathcal C,\mathcal C_{free},\mathcal C_{obs}$；离散图用 $G=(V,E)$、状态 $X$。**显式提醒 $\mathcal C_{free}$ 开/闭区别**（最优化须用 $\mathrm{cl}(\mathcal C_{free})$）。
3. **证明可直接内联正文**（符合编写规范"证明优先内联"）：Dijkstra 最优性归纳（§2.2）、A* 反证最优性（§2.4）、一致⇒可采纳归纳 + $f$ 单调（§2.5）、RRT 不渐近最优（定理 33）、PRM*/RRT* 半径公式的几何来由（dispersion）。复杂度引理 40–42 的证明亦可内联（短）。S2 渐近最优性完整证明（Lemma 50–72，数十页）建议入附录或仅给梗概（§3.5.2）。
4. **图（tikz 重画，勿搬原图）**：C-space 障碍映射（点机器人）、栅格 4/8 邻接 + 三种距离、Dijkstra/A* 波前对比、RRT 的 Voronoi 偏置分形生长（S1 图 5.19 风格）、RRT* 的 choose-parent/rewire 两步、PRM 两阶段。
5. **待核清单（务必二次核对）**：
   - **A* 反证证明字句**：本抽取据 S4 直觉证明 + AIMA 标准链条补全（标 `\rebuilt`），综合 agent 应对 Russell & Norvig *AIMA* §3.5.2 原文核句（`\cite{russell2020aima}`）。
   - **RRT\* 半径修正**：必须并入 S6（$1/(d+1)$）的勘误（§3.6），否则不严谨。
   - **S2 各常数**：$\gamma_{PRM}^*,\gamma_{RRG}^*,\gamma_{RRT^*}^*,k_{PRM}^*,k_{RRG}^*,k_{RRT^*}^*$ 已逐字抄录（注意 $\gamma_{RRT^*}^*$ 用 $(2(1+1/d))^{1/d}$，与 $\gamma_{PRM}^*$ 的 $2(1+1/d)^{1/d}$ **底数位置不同**，勿混）。
   - **轨迹优化（第四部分）**：全部 `\rebuilt`，须据 CHOMP/STOMP/TrajOpt 原论文扩充，本抽取仅定位。
   - **LaValle 复杂度 $O(|V|\lg|V|+|E|)$** 与 S4 的 $O((|E|+|V|)\log|V|)$ / $O(|E|+|V|\log|V|)$（Fib 堆）并列给出，注明堆实现差异。
