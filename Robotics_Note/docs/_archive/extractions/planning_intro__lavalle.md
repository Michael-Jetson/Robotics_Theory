# 抽取留痕：运动规划导论 —— 构型空间 C-space · 搜索式规划 (Dijkstra/A*/D*) · 采样式规划 (PRM/RRT/RRT*) · 轨迹优化简介

> **文件性质**：项目内部「抽取留痕」（非成书正文）。目标是把权威源材料【全量保真】抽取下来，供后续综合 agent 改写为自包含书章「规划导论」。
> **铁律**：禁摘要、禁凝练；每一步推导/每一道例题/每一条定义-定理-引理-命题+证明/每一张表/每一段算法伪码都完整记录；公式用 LaTeX 写全；标注源小节号；显式记录记号约定及其与本书统一约定的差异。宁长勿略。
>
> **个人笔记未同步**：本抽取主要据权威教材/原论文。凡本书目标章需要但源材料未直接给出、由抽取员据权威源补全或推断之处，统一标 `\rebuilt` 并注明依据。

---

## §0 源清单与权威出处（每条结果均注出处）

| 代号 | 文献 | 覆盖主题 | 出处 URL / 编号 | 抽取方式 |
|---|---|---|---|---|
| **L** | S. M. LaValle, *Planning Algorithms*, Cambridge University Press, 2006 | C-space（第4章）、离散规划/Dijkstra/A*（第2章）、采样式 PRM/RRT/RDT（第5章） | 官方全文 https://lavalle.pl/planning/ ；分章 PDF `ch2.pdf` `ch4.pdf` `ch5.pdf`（旧域名 http://planning.cs.uiuc.edu/） | 下载 PDF→`pdftotext -layout` 本地精读全章 |
| **KF** | S. Karaman, E. Frazzoli, *Sampling-based Algorithms for Optimal Motion Planning*, Int. J. Robotics Research (IJRR) 30(7):846–894, 2011 | RRT*、RRG、PRM*、渐近最优性理论、RRT 非最优性 | arXiv:1105.1186 https://arxiv.org/abs/1105.1186 （PDF https://arxiv.org/pdf/1105.1186） | 下载 PDF→`pdftotext -layout` 本地精读关键节（算法1–6、定理） |
| **KF10** | S. Karaman, E. Frazzoli, *Incremental Sampling-based Algorithms for Optimal Motion Planning*, RSS 2010 | RRT*/RRG 会议初版（与 KF 同源，定理编号略异） | arXiv:1005.0416 https://arxiv.org/abs/1005.0416 | 检索核对 |
| **St** | A. Stentz, *Optimal and Efficient Path Planning for Partially-Known Environments*, Proc. IEEE ICRA 1994, vol.4, pp.3310–3317 | D*（动态 A*）原始算法 | CMU RI 仓库 https://www.ri.cmu.edu/publications/optimal-and-efficient-path-planning-for-partially-known-environments/ ；技报版 *…for Unknown and Dynamic Environments* CMU-RI | 原 PDF 为 CCITT 扫描件无法 OCR；D* 内部伪码据论文文字描述 + 二级权威源复建，标 `\rebuilt` |
| **St-wiki** | Wikipedia, *D\**（条目，述 Stentz D*、Focussed D*、D* Lite） | D* 状态机/RAISE-LOWER 概念、与 A* 关系、变体 | https://en.wikipedia.org/wiki/D* | 概念核对 |

> **抽取员注（PDF→文本可靠性）**：L / KF 的分章 PDF 含可提取文本层，`pdftotext` 抽取干净，公式与伪码逐行可读，故 L、KF 部分为**复现级**。Stentz 1994 原 PDF 为传真扫描位图（CCITT Fax），无文本层，故 **D* 的逐行编号伪码（L1–L33）无法逐字复现**；本抽取的 D* `PROCESS-STATE`/`MODIFY-COST` 伪码按论文正文描述 + 后续标准教材（含 LaValle §12.3.2 对 D* 的转述、St-wiki）**复建**，逻辑等价但行号/具体写法可能与原文有出入，整段标 `\rebuilt`。综合时如需逐字原文请回原始 IEEE/ICRA 版。

---

## §0.1 记号约定（务必先读：源 vs 本书统一约定）

本抽取涉及"规划"领域，与本书 SLAM/李群主线的记号体系不同。下表列出**各源记号**及其与**本书统一约定**（旋转 $\mathbf R\in SO(3)$、右扰动为主、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 平移在前、Hamilton 四元数、协方差 $\boldsymbol\Sigma$）的对照与差异。

| 项目 | LaValle (L) | Karaman-Frazzoli (KF) | Stentz (St, D*) | 本书统一约定 | 差异/转换说明 |
|---|---|---|---|---|---|
| 构型 / 状态 | 离散：状态 $x\in X$；连续：构型 $q\in\mathcal C$ | 构型 $x\in\mathcal X$（直接用 $x$，连续） | 状态 $X$（栅格节点，大写） | 状态 $\mathbf x$ / 构型 $\mathbf q$ | **L 用 $q$ 表连续构型、$x$ 表离散状态；KF 把连续构型也记 $x$**。本书连续构型建议统一记 $\mathbf q$，离散状态记 $\mathbf x$，避免 KF 的 $x$ 与本书状态 $\mathbf x$ 混淆。 |
| 构型空间 | $\mathcal C$（calligraphic C） | $\mathcal X$（含 $\mathcal X_{free},\mathcal X_{obs},\mathcal X_{goal}$） | 二维栅格（隐式） | $\mathcal C$（C-space） | **KF 用 $\mathcal X$ 指代 L 的 $\mathcal C$**。本书沿用 L 的 $\mathcal C/\mathcal C_{free}/\mathcal C_{obs}$；引用 KF 定理时把其 $\mathcal X_{free}\to\mathcal C_{free}$、$\mu(\mathcal X_{free})\to\mu(\mathcal C_{free})$。 |
| 障碍/自由空间 | $\mathcal C_{obs}$, $\mathcal C_{free}=\mathcal C\setminus\mathcal C_{obs}$ | $\mathcal X_{obs}$, $\mathcal X_{free}$ | 障碍=不可通行栅格 | $\mathcal C_{obs}$, $\mathcal C_{free}$ | 一致（仅 $\mathcal C$ vs $\mathcal X$ 之差）。 |
| 旋转表示 | $SO(2)\cong S^1$；$SO(3)\cong\mathbb{RP}^3$；用旋转矩阵 $A$/$R$、单位四元数 $h=a+bi+cj+dk$ | 不涉具体 $SO(3)$（默认 $\mathcal X\subset\mathbb R^d$ 或一般度量空间） | 不涉旋转 | $\mathbf R\in SO(3)$，四元数 $\mathbf q=[w,\mathbf v]$ Hamilton | **L 四元数写作 $h=a+bi+cj+dk$，标量 $a$ 在前**，与本书 Hamilton 标量在前一致（本书记 $w$）。L 用 $h$，本书用 $\mathbf q$；注意 L 同时用 $q$ 表构型，故 L 的"$q$"与"四元数 $h$"是两码事。 |
| 路径 | $\tau:[0,1]\to\mathcal C_{free}$，$\tau(0)=q_I,\tau(1)=q_G$ | 路径/轨迹 $\sigma\in\Sigma$，代价 $c(\sigma)$ | 反向指针链 $b(\cdot)$ | $\tau:[0,1]\to\mathcal C_{free}$ | 一致；KF 另记 $\Sigma$ 为所有路径之集。 |
| 起点/终点 | 初始 $q_I$、目标 $q_G$（或目标集 $X_G\subset X$） | $x_{init}$、目标区 $\mathcal X_{goal}$ | start / goal（D* 反向从 goal 搜） | $\mathbf q_I$、$\mathbf q_G$ | 一致。**注意 D* 反向搜索：树根在 goal。** |
| 代价 / 损失 | 边代价 $l(e)=l(x,u)$；阶段可加损失泛函 $L$ | 代价函数 $c:\Sigma\to\mathbb R_{\ge0}$ | 弧代价 $c(X,Y)$；路径代价估计 $h(X)$ | 代价 $l(\cdot)$ / 残差能量 | L 的 $l$ 与 St 的 $c$ 同义（边/弧代价）；**St 的 $h(X)$ 是"到目标的代价"（cost-to-goal），与 A* 文献里 $h$=启发值含义不同——见 §D* 注**。 |
| 代价到达 / 代价到去 | cost-to-come $C(x)$、$C^*$；cost-to-go $G(x)$、$G^*$；A* 启发 $\hat G^*(x)$ | $\mathrm{Cost}(v)$=根到 $v$ 的树路径代价 | $h(X)$=到 goal 代价；key $k(X)$ | $g$（已走代价）、$h$（启发）、$f=g+h$（A* 经典记号） | **记号冲突警告**：经典 A* 文献 $g$=cost-to-come、$h$=启发(cost-to-go下界)、$f=g+h$；**L 用 $C$=cost-to-come、$\hat G^*$=启发**；**St 用 $h$ 表 cost-to-goal（非启发！）、$k$ 表优先级 key**。本书写 A* 建议用经典 $g/h/f$；转 D* 时务必把 St 的 $h\to$ "到目标实际/估计代价"、$k\to$ 优先队列键。 |
| 维数 | $n$=链节数/自由度；$\dim\mathcal C$ | $d$=$\mathcal X$ 维数 | 2（平面） | $n$ 或 $d$ | KF 全程用 $d$，L 用 $n$。本书引 KF 公式时 $d=\dim\mathcal C$。 |
| 步长 / 邻域半径 | RDT 步长 $\Delta q$（可消去）；栅格分辨率 $k_i$ | 转向上限 $\eta$；Near 半径 $r(n)$ | 栅格步 | $\Delta q$、$r$ | KF 的 $\eta$ 是 `Steer` 的最大步长。 |
| 度量 | $\rho:\mathcal C\times\mathcal C\to\mathbb R$（$L_p$ 度量为主） | $\|\cdot\|$ 欧氏 | 栅格邻接 | $\rho$ / $\|\cdot\|$ | 一致。 |
| 测度 / 体积 | 测度 $\mu$；单位球体积 $\zeta_d=\pi^{d/2}/\Gamma(d/2+1)$ | $\mu$（Lebesgue）；$\zeta_d$ 同 | — | $\mu$、$\zeta_d$ | 一致。 |
| 弥散 / 偏差 | 弥散 $\delta(P)$；偏差 $D(P,\mathcal R)$ | — | — | $\delta$、$D$ | L 专有。 |

> **本书选型建议（综合时）**：①C-space 记号沿用 LaValle（$\mathcal C,\mathcal C_{free},\mathcal C_{obs},q,\tau,q_I,q_G$）；②A* 写经典 $g/h/f$ 而非 L 的 $C/\hat G^*$（更通行、与 D* 衔接顺）；③RRT*/PRM* 公式照搬 KF 但把 $\mathcal X_{free}\to\mathcal C_{free}$、把构型 $x\to q$、维数 $d=\dim\mathcal C$；④D* 把 St 的 $h$ 显式改称"到目标代价 $g_{\text{goal}}$"以免与启发 $h$ 撞名。

---

---

# 第一部分　构型空间 C-space（源：LaValle 第4章）

> 源章：L Ch.4 *The Configuration Space*。L 把"机器人所有可施加的变换之集"称为**构型空间**（configuration space, C-space），溯源 Lozano-Pérez 的开创工作、Latombe 教材的统一。C-space 的核心价值：几何/运动学看似迥异的运动规划问题，一旦表达到 C-space 上就能被同一套规划算法求解。

## §C1 基本拓扑概念（L §4.1.1）

规划要在连续空间上"连通一点到另一点"，需要拓扑语言。以下定义全部来自 L §4.1.1。

### §C1.1 拓扑空间与开/闭集

**开区间 vs 闭区间（动机）**：$(0,1)$ 含 0 与 1 之间所有实数但不含端点；可构造收敛到端点的序列（如 $1/2,1/4,\dots,1/2^i\to0$），即可任意逼近边界却取不到边界。闭区间 $[0,1]$ 含端点。

**拓扑空间定义（L §4.1.1）**：集合 $X$ 配一族子集（称**开集**），满足三公理：
1. 任意多个（含无穷多）开集之并仍是开集；
2. **有限**个开集之交仍是开集；
3. $X$ 与 $\varnothing$ 都是开集。

注意：无穷多开集之**交**未必开。

例（L 式4.1）：$\bigcup_{i=1}^{\infty}\left(\dfrac{1}{3^i},\dfrac{2}{3^i}\right)$ 是无穷多两两不交开区间之并，是开集。

**闭集**：设 $X$ 拓扑空间，$C\subset X$ 称**闭集** $\iff$ $X\setminus C$ 是开集。即开集之补为闭、闭集之补为开。$[0,1]$ 是闭集（补 $(-\infty,0)\cup(1,\infty)$ 开）。

**关键反直觉点**：并非每个子集非开即闭。$X=\mathbb R$ 上 $[0,2\pi)$ 既不开也不闭。又：任何拓扑空间里 $X$ 与 $\varnothing$ **既开又闭**。

### §C1.2 特殊点（L §4.1.1，配 L 图4.1）

设 $X$ 拓扑空间，$U\subseteq X$，$x\in X$：
- **内点 (interior point)**：存在开集 $O_1$，$x\in O_1\subseteq U$。$U$ 全体内点之集称**内部** $\mathrm{int}(U)$。
- **外点 (exterior point)**：存在开集 $O_2$，$x\in O_2\subseteq X\setminus U$。
- **边界点 (boundary point)**：既非内点也非外点。$U$ 全体边界点之集称**边界** $\partial U$。
- **极限点 (limit point / accumulation point)**：内点或边界点（冗余术语）。$U$ 全体极限点之集是闭集，称**闭包** $\mathrm{cl}(U)$，且
$$\mathrm{cl}(U)=\mathrm{int}(U)\cup\partial U.$$

**推论（对运动规划很关键）**：闭集 $C$ 含其任一子序列的极限点，故含全部边界点；闭包 $\mathrm{cl}$ 总产生闭集（把边界点都加进去）；开集不含任一自身边界点。

### §C1.3 例子（L §4.1.1）

**例 4.1（$\mathbb R^n$ 的拓扑）**：开球
$$B(x,\rho)=\{x'\in\mathbb R^n\mid\|x'-x\|<\rho\}\tag{L 4.2}$$
（$\|\cdot\|$=欧氏范数）是 $\mathbb R^n$ 的开集；其他开集都能写成可数个开球之并（开球族称一组**基**）。任何代数原语 $H=\{x\in\mathbb R^n\mid f(x)\le0\}$ 产生闭集；只用 $<$ 的原语产生开集。

**例 4.2（子空间拓扑）**：$Y\subset X$，其开集定义为所有可写成 $U\cap Y$（$U$ 是 $X$ 的开集）的子集。

**例 4.3（平凡拓扑）**：只取 $X,\varnothing$ 为开集即满足公理。

**例 4.4（怪拓扑）**：$X=\{cat,dog,tree,house\}$，取 $\varnothing,X,\{cat\},\{dog\}$ 为开集，则 $\{cat,dog\}$ 也须是开集。说明拓扑空间可极一般。

**Hausdorff 公理（L §4.1.1）**：对任意不同 $x_1,x_2\in X$，存在开集 $O_1,O_2$ 使 $x_1\in O_1,x_2\in O_2,O_1\cap O_2=\varnothing$。满足者称 **Hausdorff 空间**。$\mathbb R^n$ 及本书一切 C-space 都是 Hausdorff（小开球即可分离）。

### §C1.4 连续函数与同胚（L §4.1.1）

**预像**：$f:X\to Y$，$B\subseteq Y$，
$$f^{-1}(B)=\{x\in X\mid f(x)\in B\}.\tag{L 4.4}$$

**连续**：$f$ 连续 $\iff$ 对 $Y$ 每个开集 $O$，$f^{-1}(O)$ 是 $X$ 的开集。（此定义比经典 $\delta\text-\epsilon$ 简洁：连续函数之复合连续，只需"开集的预像的预像仍开"一行论证。）

**同胚 (homeomorphism)**：$f:X\to Y$ 双射，若 $f$ 与 $f^{-1}$ 都连续，则 $f$ 为同胚；$X,Y$ 称**同胚**，记 $X\cong Y$。同胚是拓扑里最基本的等价关系（自反、对称、传递）。"甜甜圈与单柄咖啡杯同胚"即指其表面都恰一个洞。

**例 4.5（区间同胚）**：任意开区间彼此同胚（$(0,1)\to(0,5)$ 用 $x\mapsto5x$）。一般地，$X\subset\mathbb R^n$ 经非奇异线性变换到 $Y\subset\mathbb R^n$ 则二者同胚——故刚体平移/旋转不改变机器人自身拓扑。**警告**：$[0,1]$ 与 $(0,1)$ 不同胚，$(0,1)$ 与 $[0,1)$ 也不同胚（端点作梗）。有界与无界可同胚：$x\mapsto1/x$ 使 $(0,1)\cong(1,\infty)$；$x\mapsto\frac{2}{\pi}\tan^{-1}(x)$ 使 $(-1,1)\cong\mathbb R$。

**例 4.6（拓扑图）**：每顶点对应 $X$ 中一点、每边对应连续单射 $\tau:[0,1]\to X$（边像除顶点外不相交）。两图（标准图论意义下）同构 $\Rightarrow$ 对应拓扑图同胚；不同构的图，去掉"无用的二度顶点"后，若仍不同构则拓扑图不同胚。

## §C2 流形（L §4.1.2）

**流形定义（L §4.1.2，脚注4）**：拓扑空间 $M\subseteq\mathbb R^m$ 是**流形** $\iff$ 对每个 $x\in M$ 存在开集 $O\subset M$ 使：① $x\in O$；② $O$ 同胚于 $\mathbb R^n$；③ $n$ 对所有 $x\in M$ 固定。该固定 $n$ 称流形**维数**。第②条最关键：任一点邻域行为与 $\mathbb R^n$ 中一点邻域相同。
（更一般的流形不必是 $\mathbb R^m$ 子集，只需 Hausdorff + 第二可数；取 $M\subseteq\mathbb R^m$ 自动满足，已够本书用。）

**Whitney 嵌入定理（L 引）**：$m\ge n$ 必然；且 $m\le2n+1$，即 $\mathbb R^{2n+1}$ "足够大"装下任何 $n$ 维流形。称 $M$ **嵌入**于 $\mathbb R^m$（存在 $M\to\mathbb R^m$ 单射）。

**带边流形**：边界点邻域同胚于 $n$ 维半空间、内点邻域同胚于 $\mathbb R^n$。

### §C2.1 笛卡尔积构造流形（L §4.1.2）

$X,Y$ 拓扑空间，**笛卡尔积** $X\times Y$：点为 $(x,y)$，开集由"一个 $X$ 开集 × 一个 $Y$ 开集"生成（再取并/有限交）。$\mathbb R\times\mathbb R=\mathbb R^2$，一般 $\mathbb R^n=\mathbb R\times\cdots\times\mathbb R$。

### §C2.2 1维流形（L §4.1.2）

- $\mathbb R$（可用 $(0,1)$ 代替，同胚）。
- 圆 $S^1$（与 $(0,1)$ 不同胚）：
$$S^1=\{(x,y)\in\mathbb R^2\mid x^2+y^2=1\}.\tag{L 4.5}$$

**等同 (identification)**：声明某些点等价（即便原本不同），记 $X/\!\sim$。$S^1$ 可定义为 $[0,1]/\!\sim$ 且 $0\sim1$（把区间两端"粘"成闭环）；同胚由极坐标 $\theta\mapsto(\cos2\pi\theta,\sin2\pi\theta)$ 给出。这是**商拓扑**。等同只是减少可视化所需维数，不违反流形定义（Whitney 保证可嵌入）。

### §C2.3 2维流形（L §4.1.2，配 L 图4.5）

由 1 维流形作笛卡尔积或在方块 $(0,1)^2$ 上作边界等同得到：
- 平面 $\mathbb R^2=\mathbb R\times\mathbb R$。
- 无限柱面 $\mathbb R\times S^1$。
- 环面 (torus) $T^2=S^1\times S^1$（甜甜圈表面）。
- 由 $M=(0,1)^2$ 作等同：
  - **柱面**：$(0,y)\sim(1,y)$。
  - **Möbius 带**：$(0,y)\sim(1,1-y)$（侧边加 180° 扭转）；单侧、不可定向。
  - **环面 $T^2$**：$(0,y)\sim(1,y)$ 且 $(x,0)\sim(x,1)$（无扭转）；点 $(0,0)$ 与另三点等同。
  - **Klein 瓶**：侧边像 Möbius 那样加扭、上下像环面那样等同；可嵌入 $\mathbb R^4$，内外面同一。
  - **实射影平面 $\mathbb{RP}^2$**：侧边与上下都加扭。等价于 $\mathbb R^3$ 中过原点的所有直线之集。
- 二维球面：
$$S^2=\{(x,y,z)\in\mathbb R^3\mid x^2+y^2+z^2=1\}.\tag{L 4.6}$$
  注意 $S^1\times S^1\ne S^2$（故环面记 $T^n=(S^1)^n$）。
- 双环面（双柄甜甜圈表面）。

### §C2.4 高维流形与射影空间（L §4.1.2）

- $n$ 维环面 $T^n=(S^1)^n$；混合柱面 $\mathbb R^i\times T^j$（$i+j=n$）。
- $n$ 维球面：
$$S^n=\{x\in\mathbb R^{n+1}\mid\|x\|=1\}.\tag{L 4.7}$$
- **实射影空间 $\mathbb{RP}^n$**：$\mathbb R^{n+1}$ 中过原点的所有直线之集。每条这样的直线与 $S^n$ 恰交于两个**对径点 (antipodal)**；把 $S^n$ 上所有对径点等同，得
$$\mathbb{RP}^n\cong S^n/\!\sim.$$
  解读：$\mathbb{RP}^n$ 是 $S^n$ 上半部分但赤道上每点与其对径点等同。**$\mathbb{RP}^3$ 是运动规划最重要的流形之一（=$SO(3)$，见 §C4）**。

## §C3 路径与连通性（L §4.1.3）

**路径**：$X$ 拓扑空间（本书取流形），**路径**是连续函数 $\tau:[0,1]\to X$（路径是函数不是点集；$\tau(s)$ 是 $s$ 处的点）。这是离散规划"访问的状态序列 $x_1,x_2,\dots$"的连续推广（$s$ 代替阶段下标）。

**连通 vs 路径连通（L §4.1.3）**：
- $X$ **连通**：不能写成两个不交非空开集之并。
- $X$ **路径连通**：对任意 $x_1,x_2\in X$ 存在路径 $\tau$ 使 $\tau(0)=x_1,\tau(1)=x_2$。
- 路径连通 $\Rightarrow$ 连通；反之未必（反例：**拓扑学家的正弦曲线** $X=\{(x,y)\in\mathbb R^2\mid x=0\text{ 或 }y=\sin(1/x)\}$（L 4.8），它连通但 y 轴与正弦曲线间无路径）。
- **本书约定（L）**：因只考虑流形，连通与路径连通重合，统称"连通"；但提醒严格应说"路径连通"。

**同伦 (homotopy)（L §4.1.3，配 L 图4.6）**：两路径 $\tau_1,\tau_2$ 称**同伦**（端点固定）$\iff$ 存在连续 $h:[0,1]\times[0,1]\to X$ 满足：
1. $h(s,0)=\tau_1(s)\ \forall s$（起于第一条路径）；
2. $h(s,1)=\tau_2(s)\ \forall s$（终于第二条路径）；
3. $h(0,t)=h(0,0)\ \forall t$（起点不动）；
4. $h(1,t)=h(1,0)\ \forall t$（终点不动）。
$t$ 像旋钮，连续地把 $\tau_1$ 形变到 $\tau_2$，形变中路径像不可跳过洞（$\mathbb R^2$ 中过不去洞）。

**单连通 vs 多连通**：$X$ 路径连通时，若所有路径同属一个同伦等价类则 $X$ **单连通**，否则**多连通**。

**群 (group)（L §4.1.3）**：集合 $G$ 配二元运算 $\circ$ 满足：①闭合 $a\circ b\in G$；②结合律 $(a\circ b)\circ c=a\circ(b\circ c)$；③单位元 $e$：$e\circ a=a\circ e=a$；④逆元 $a^{-1}$：$a\circ a^{-1}=a^{-1}\circ a=e$。可交换者称 **Abelian/交换群**。例 4.7：$(\mathbb Z,+)$（单位0、逆 $-i$）、$(\mathbb Q\setminus0,\times)$（单位1、逆 $1/q$）。**3D 旋转之集是非交换群**。

**基本群 $\pi_1(X)$（第一同伦群，L §4.1.3）**：取基点 $x_b$，考虑所有 $f(0)=f(1)=x_b$ 的**环路 (loop)**；定义环路乘积
$$\tau(t)=\begin{cases}\tau_1(2t)&t\in[0,1/2)\\\tau_2(2t-1)&t\in[1/2,1]\end{cases}\tag{L 4.9}$$
（首尾相接）。在同伦等价类上此乘积构成群，即 $\pi_1(X)$，刻画 $X$ 的连通性。例 4.8：$X$ 单连通则所有环路同伦，$\pi_1(X)=\mathbf 1_G$（平凡群）。

## §C4 定义 C-space：刚体与链（L §4.2）

### §C4.1 二维刚体 SE(2)（L §4.2.1）

**矩阵群法推导 $SO(2)$（L 例4.12 转述）**：从 $2\times2$ 实矩阵 $\mathbb R^4$ 出发，加正交约束 $AA^T=I$，展开得（L 4.12–4.15）：
$$a^2+b^2=1,\quad ac+bd=0,\quad ca+db=0\ (\text{冗余}),\quad c^2+d^2=1.$$
有 $\binom n2=1$ 个列正交方程（$n=2$）、2 个单位列方程，故从 $\mathbb R^4$ 减 3 维得 1 维流形。对每个 $(a,b)$（在 $S^1$ 上）有两组 $(c,d)$：$(c,d)=(b,-a)$ 或 $(-b,a)$，是两个圆 $S^1\sqcup S^1$（不连通），即 $O(2)$。再加 $\det A=ad-bc=1$ 选其一，得
$$\boxed{SO(2)\cong S^1},$$
可用极坐标参数化为标准 2D 旋转矩阵（L 式3.31）。

**用单位复数表示 $SO(2)$（L §4.2.1）**：单位复数 $a+bi$（$a^2+b^2=1$）之集在乘法下成群，与 $SO(2)$ **同构**（$e^{i\theta}\leftrightarrow$ 旋转矩阵）。两旋转复合 = 复数相乘：$e^{i\theta_1}e^{i\theta_2}=e^{i(\theta_1+\theta_2)}$；笛卡尔形式 $(a_1+b_1i)(a_2+b_2i)=(a_1a_2-b_1b_2)+(a_1b_2+a_2b_1)i$。转矩阵：
$$R(a,b)=\begin{bmatrix}a&-b\\b&a\end{bmatrix}.\tag{L 4.18}$$
故单位复数集与 $SO(2)$ 同为 $S^1$。

**SE(2) 的 C-space（L 式4.17）**：平面可平移+旋转的刚体
$$\boxed{\mathcal C=\mathbb R^2\times S^1}\tag{L 4.17}$$
（更精确应用 $\cong$）。$S^1$ 多连通故 $\mathbb R\times S^1,\mathbb R^2\times S^1$ 多连通。可视化：取 $(0,1)^3$，对 $z$ 方向作 $(x,y,0)\sim(x,y,1)$ 等同（$x,y$ 方向有"边界"、$z$ 方向环绕）。**规划算法必须知道这个环绕**（L 图4.8 例：柱面上中间有障碍，若不识别上下等同就连不通 $q_I\to q_G$）。

### §C4.2 三维刚体 SE(3)（L §4.2.2）——核心：$SO(3)$ 的拓扑

**结论**：6 维流形
$$\boxed{\mathcal C=\mathbb R^3\times\mathbb{RP}^3}\tag{L 4.31}$$
（3 平移 + 3 旋转）。

**为何不用欧拉角（yaw-pitch-roll $\alpha,\beta,\gamma$）**：会破坏拓扑——存在非零角给出单位旋转（等价 $\alpha=\beta=\gamma=0$）、存在连续多组角给同一旋转矩阵（万向锁），造成理论与实践困难。

**矩阵群法（L §4.2.2）**：$GL(3)\cong\mathbb R^9$；$O(3)$ 由 $AA^T=I$ 定（$\binom32=3$ 个列正交方程 + 3 个单位列方程），3 维。$O(3)$ 是两个三维球面 $S^3\sqcup S^3$，$\det A=1$ 选其一得 $SO(3)$；但**对径点给同一旋转矩阵**（下用四元数说明）。

**四元数（L §4.2.2）**：$\mathbb H$ 之元 $h=a+bi+cj+dk$，$a,b,c,d\in\mathbb R$；
$$i^2=j^2=k^2=ijk=-1\Rightarrow ij=k,\ jk=i,\ ki=j.$$
两四元数 $h_1=a_1+b_1i+c_1j+d_1k$、$h_2=\dots$ 之积 $h_1\cdot h_2=a_3+b_3i+c_3j+d_3k$（L 式4.19）：
$$\begin{aligned}
a_3&=a_1a_2-b_1b_2-c_1c_2-d_1d_2\\
b_3&=a_1b_2+a_2b_1+c_1d_2-c_2d_1\\
c_3&=a_1c_2+a_2c_1+b_2d_1-b_1d_2\\
d_3&=a_1d_2+a_2d_1+b_1c_2-b_2c_1.
\end{aligned}\tag{L 4.19}$$
$\mathbb H$ 在四元数乘法下成群，**不可交换**（与 3D 旋转一致）。向量形式：令 $\mathbf v=[b\ c\ d]$，则 $h_1\cdot h_2$ 的标量部 $=a_1a_2-\mathbf v_1\cdot\mathbf v_2$，向量部 $=a_1\mathbf v_2+a_2\mathbf v_1+\mathbf v_1\times\mathbf v_2$。
> **与本书 Hamilton 约定对照**：L 标量 $a$ 在前、虚部 $bi+cj+dk$，与本书 Hamilton（标量在前 $\mathbf q=[w,\mathbf v]$）**一致**；L 的乘法即 Hamilton 积。本书记 $w=a,\mathbf v=[b,c,d]$。

**单位四元数 → $SO(3)$（L 式4.20）**：约束 $a^2+b^2+c^2+d^2=1$（构成子群）。映射
$$R(h)=\begin{bmatrix}2(a^2+b^2)-1&2(bc-ad)&2(bd+ac)\\2(bc+ad)&2(a^2+c^2)-1&2(cd-ab)\\2(bd-ac)&2(cd+ab)&2(a^2+d^2)-1\end{bmatrix}\tag{L 4.20}$$
正交且 $\det R(h)=1$，属 $SO(3)$。轴-角解读（L 图4.9、式4.21）：绕单位轴 $\mathbf v=[v_1,v_2,v_3]$ 转 $\theta$：
$$h=\cos\tfrac\theta2+\Big(v_1\sin\tfrac\theta2\Big)i+\Big(v_2\sin\tfrac\theta2\Big)j+\Big(v_3\sin\tfrac\theta2\Big)k.\tag{L 4.21}$$

**二对一与 $\mathbb{RP}^3$（L §4.2.2）**：$R(h)=R(-h)$（绕 $\mathbf v$ 转 $\theta$ = 绕 $-\mathbf v$ 转 $2\pi-\theta$；验证：实部 $\cos\frac{2\pi-\theta}2=-\cos\frac\theta2=-a$（L 4.22），虚部 $-\mathbf v\sin\frac{2\pi-\theta}2=[-b,-c,-d]$（L 4.23）），故映射是从单位四元数到 $SO(3)$ 的**二对一**。单位四元数集 $\cong S^3$（$a^2+b^2+c^2+d^2=1$）；对径等同 $h\sim-h$ 给
$$\boxed{SO(3)\cong\mathbb{RP}^3=S^3/\!\sim}.$$
唯一化办法：取"上半 $S^3$"（要求 $a\ge0$；$a=0$ 时要求 $b\ge0$；$a=b=0$ 时 $c\ge0$；$a=b=c=0$ 时 $d=1$），但路径越过赤道时须跳到对径点。

**用四元数乘法做旋转（L §4.2.2）**：共轭 $h^*=a-bi-cj-dk$；点 $(x,y,z)$ 记 $p=0+xi+yj+zk$，则旋转后点 = $h\cdot p\cdot h^*$ 的 $i,j,k$ 分量（与 $R(h)$ 作用等价）。优势：四元数乘法比矩阵乘法运算少。

**从旋转矩阵反求四元数（L 式4.24–4.30）**：给定 $R=[r_{ij}]$，
$$a=\tfrac12\sqrt{r_{11}+r_{22}+r_{33}+1};\tag{L 4.24}$$
若 $a\ne0$：
$$b=\frac{r_{32}-r_{23}}{4a},\quad c=\frac{r_{13}-r_{31}}{4a},\quad d=\frac{r_{21}-r_{12}}{4a}.\tag{L 4.25–4.27}$$
若 $a=0$（赤道）：
$$b=\frac{r_{13}r_{12}}{\sqrt{r_{12}^2r_{13}^2+r_{12}^2r_{23}^2+r_{13}^2r_{23}^2}},\ c=\frac{r_{12}r_{23}}{\sqrt{\cdots}},\ d=\frac{r_{13}r_{23}}{\sqrt{\cdots}}.\tag{L 4.28–4.30}$$
（当 $R$ 退化为纯 yaw/pitch/roll 时此式失效，需预先检测。）
> **与本书对照**：本书旋转矩阵→四元数采用 Sheppard/Shuster 式分支选最大对角元的稳健版；L 4.24–4.30 是 $a$-主分支版，数值上 $a$ 接近 0 时退化。综合若用 L 版须补稳健分支，或引本书既有公式。

**SE(3)（L §4.2.2）**：一般形 $\big[\begin{smallmatrix}R&v\\0&1\end{smallmatrix}\big]$，$R\in SO(3),v\in\mathbb R^3$；因 $SO(3)\cong\mathbb{RP}^3$ 且平移独立，得 $\mathcal C=\mathbb R^3\times\mathbb{RP}^3$（6 维，恰等自由飞行刚体自由度）。

### §C4.3 多体、链与树（L §4.2.3）

**$n$ 个自由漂浮刚体**（$\mathcal W=\mathbb R^2$ 或 $\mathbb R^3$）：
$$\mathcal C=\mathcal C_1\times\mathcal C_2\times\cdots\times\mathcal C_n.\tag{L 4.32}$$

**运动学链/树**：须逐情形分析，无统一简化规则。注意：①关节常无法全程转动（旋转关节若不能绕满 $S^1$，则该关节 C-space 同胚于 $\mathbb R$ 而非 $S^1$；球关节常因机械限位非 $\mathbb{RP}^3$）；②DH 参数化便于变换计算但忽略拓扑（如用三个零长连杆模拟球腕会重蹈欧拉角覆辙）。

**例（L §4.2.3，$\mathcal W=\mathbb R^2$，$n$ 旋转关节链）**：
- 第一关节绕定点、各关节满程 $\theta_i\in[0,2\pi)$：
$$\mathcal C=S^1\times\cdots\times S^1=T^n.\tag{L 4.33}$$
- 各关节受限 $\theta_i\in(-\pi/2,\pi/2)$：$\mathcal C=\mathbb R^n$。
- 若 $A_1$ 可施加任意 $SE(2)$：满程时 $\mathcal C=\mathbb R^2\times T^n$；受限时 $\mathbb R^{n+2}$。棱柱关节每个贡献 $\mathbb R$。

**例（$\mathcal W=\mathbb R^3$，六类关节，L §4.2.3 + L 图3.12）**：旋转/棱柱关节同 $\mathbb R^2$ 情形；螺旋关节贡献 $\mathbb R$；圆柱关节贡献 $\mathbb R\times S^1$（旋转不受限时）；平面关节贡献 $\mathbb R^2\times S^1$（任意 $SE(2)$），受限则 $\mathbb R^3$；球关节理论上贡献 $\mathbb{RP}^3$，实际常因限位降为 $\mathbb R^2\times S^1$ 或 $\mathbb R^3$；第一关节若是自由漂浮体则贡献 $\mathbb R^3\times\mathbb{RP}^3$。

**运动学树**：同链处理；连杆间可能碰撞（自碰），用 §C5 的方法在 $\mathcal W$ 中加障碍处理。

## §C5 构型空间障碍（L §4.3）

### §C5.1 障碍区与自由空间（L §4.3.1）

**刚体的障碍区**：世界 $\mathcal W=\mathbb R^2$ 或 $\mathbb R^3$ 含障碍区 $\mathcal O\subset\mathcal W$，刚体机器人 $\mathcal A\subset\mathcal W$，$\mathcal A,\mathcal O$ 均为半代数模型。构型 $q\in\mathcal C$（$\mathcal W=\mathbb R^2$ 时 $q=(x_t,y_t,\theta)$；$\mathcal W=\mathbb R^3$ 时 $q=(x_t,y_t,z_t,h)$，$h$=单位四元数）。$\mathcal A(q)$=变换后机器人。**障碍区**
$$\boxed{\mathcal C_{obs}=\{q\in\mathcal C\mid\mathcal A(q)\cap\mathcal O\ne\varnothing\}}\tag{L 4.34}$$
即机器人与障碍相交的所有构型。因 $\mathcal O,\mathcal A(q)$ 在 $\mathcal W$ 中是闭集，$\mathcal C_{obs}$ 是 $\mathcal C$ 中**闭集**。

**自由空间**：
$$\boxed{\mathcal C_{free}=\mathcal C\setminus\mathcal C_{obs}}$$
是**开集**（$\mathcal C_{obs}$ 闭）。故机器人可在 $\mathcal C_{free}$ 内任意逼近障碍。"接触"条件（$q\in\mathcal C_{obs}$）：
$$\mathrm{int}(\mathcal O)\cap\mathrm{int}(\mathcal A(q))=\varnothing\ \text{且}\ \mathcal O\cap\mathcal A(q)\ne\varnothing.\tag{L 4.35}$$
（$\mathcal C_{free}$ 开 $\Rightarrow$ 某些优化如"最短路"无定义，须改用闭包 $\mathrm{cl}(\mathcal C_{free})$，见 L §7.7。）

**多体障碍区（自碰，L §4.3.1）**：机器人是 $m$ 个连杆 $\{\mathcal A_1,\dots,\mathcal A_m\}$，单一构型向量 $q$，写 $\mathcal A_i(q)$。设**碰撞对集** $P$：$(i,j)\in P$（$i\ne j$）表示 $\mathcal A_i,\mathcal A_j$ 不许相交。常定义：每连杆须避开非关节相连的连杆。$|P|=O(m^2)$，但几何分析常可剔除大量不可能碰撞的对。含自碰的障碍区：
$$\boxed{\mathcal C_{obs}=\Big(\bigcup_{i=1}^m\{q\mid\mathcal A_i(q)\cap\mathcal O\ne\varnothing\}\Big)\cup\Big(\bigcup_{[i,j]\in P}\{q\mid\mathcal A_i(q)\cap\mathcal A_j(q)\ne\varnothing\}\Big).}\tag{L 4.36}$$

### §C5.2 基本运动规划问题（钢琴搬运工问题，L §4.3.1 Formulation 4.1）

> 本书目标章「规划导论」的核心问题陈述。配 L 图4.11（$\mathcal C=\mathcal C_{free}\cup\mathcal C_{obs}$，任务=在 $\mathcal C_{free}$ 内连 $q_I\to q_G$）。

**Formulation 4.1（The Piano Mover's Problem，L §4.3.1）**：
1. 世界 $\mathcal W=\mathbb R^2$ 或 $\mathbb R^3$；
2. 世界中半代数障碍区 $\mathcal O\subset\mathcal W$；
3. $\mathcal W$ 中半代数机器人：刚体 $\mathcal A$ 或多连杆 $\mathcal A_1,\dots,\mathcal A_m$；
4. 由"可施加的全部变换"确定的构型空间 $\mathcal C$，并据之导出 $\mathcal C_{obs},\mathcal C_{free}$；
5. 初始构型 $q_I\in\mathcal C_{free}$；
6. 目标构型 $q_G\in\mathcal C_{free}$；$(q_I,q_G)$ 称**查询对 (query)**；
7. **完备算法**须计算连续路径 $\tau:[0,1]\to\mathcal C_{free}$ 使 $\tau(0)=q_I,\tau(1)=q_G$，或正确报告不存在此路径。

**复杂度（L §4.3.1，引 Reif）**：该问题 **PSPACE-hard**（蕴含 NP-hard）；难点在 $\dim\mathcal C$ 无界。
> **抽取员注**：这是采样式规划存在的根本动因——精确构造 $\mathcal C_{obs}$ 不可行（见下），故转而"用采样探测 + 把碰撞检测当黑箱"（见第三部分）。

### §C5.3 显式构造 $\mathcal C_{obs}$：纯平移情形（L §4.3.2）

**Minkowski 和/差（L 式4.37）**：$X,Y\subset\mathbb R^n$，
$$X\ominus Y=\{x-y\in\mathbb R^n\mid x\in X,y\in Y\}.\tag{L 4.37}$$
Minkowski 差 = $X$ 与 $-Y$ 的 Minkowski 和 $X\oplus(-Y)$（$-Y$ 即每个 $y$ 取 $-y$）。纯平移机器人（$\mathcal C=\mathbb R^n$）：
$$\boxed{\mathcal C_{obs}=\mathcal O\ominus\mathcal A(0).}$$

**例 4.13（一维 C-障碍，L）**：$\mathcal W=\mathbb R$，机器人 $\mathcal A=[-1,2]$，障碍 $\mathcal O=[0,4]$，$-\mathcal A=[-2,1]$，则 $\mathcal C_{obs}=\mathcal O\oplus(-\mathcal A)=[-2,5]$。

**卷积视角（L 式4.38）**：$f(x)=\mathbf 1[x\in\mathcal O]$，$g(x)=\mathbf 1[x\in\mathcal A]$，
$$h(x)=\int_{-\infty}^{\infty}f(\tau)g(x-\tau)\,d\tau,\tag{L 4.38}$$
则 $h(x)>0\iff x\in\mathrm{int}(\mathcal C_{obs})$。

**多边形 C-障碍（star algorithm，L §4.3.2）**：凸多边形障碍 $\mathcal O$ + 凸多边形机器人 $\mathcal A$（纯平移），$\mathcal C_{obs}$ 也是凸多边形。关键：$\mathcal C_{obs}$ 每条边都是 $\mathcal A$ 或 $\mathcal O$ 的某条边的平移，且 $\mathcal O,\mathcal A$ 各边恰用一次。算法=按内/外法向角排序：设 $\mathcal A$ 各内向边法向角 $\alpha_1,\dots,\alpha_n$（逆时针），$\mathcal O$ 各外向边法向角 $\beta_1,\dots,\beta_m$，在 $S^1$ 上圆序归并即得 $\mathcal C_{obs}$ 边序。运行 $O(n+m)$（角已有序，仅需归并）。
- **两类接触（L 图4.16）**：Type EV（$\mathcal A$ 的边 vs $\mathcal O$ 的顶点）与 Type VE（$\mathcal A$ 的顶点 vs $\mathcal O$ 的边）。Type EV 出现在遇到 $\mathcal A$ 的边法向时，Type VE 出现在遇到 $\mathcal O$ 的边法向时。
- **半平面表示（L §4.3.2）**：每条 $\mathcal C_{obs}$ 边对应一支撑半平面；接触条件为法向 $n$ 与向量 $v$ 垂直 $n\cdot v=0$（L 图4.17）。平移下 $n$ 不依赖构型、$v$ 依赖平移 $(x_t,y_t)$，故 $n\cdot v(x_t,y_t)=0$ 是 $\mathcal C$ 中一条直线方程。令 $f(x_t,y_t)=n\cdot v(x_t,y_t)$，$H=\{(x_t,y_t)\mid f\le0\}$，交所有 EV/VE 半平面得 $\mathcal C_{obs}$（$n+m$ 条边的凸多边形）。
- 例 4.14（三角形机器人 + 矩形障碍）：绕障碍滑动机器人使其始终接触，原点描出 $\partial\mathcal C_{obs}$ 共七条边，每条对应 $\mathcal A$ 或 $\mathcal O$ 一边（L 图4.13–4.15）。

> **抽取员注（旋转情形）**：L §4.3.3 进一步讨论含旋转的 $\mathcal C_{obs}$（须在 $\mathbb R^2\times S^1$ 中按 $\theta$ 切片构造，每片是平移情形），代数迅速复杂化——这正再次印证"高维显式 $\mathcal C_{obs}$ 不可行"。本抽取按目标章聚焦，旋转情形从略；若综合需要可回 L §4.3.3。

---

---

# 第二部分　搜索式（组合/图）规划：离散规划框架、Dijkstra、A*、D*

## §S1 离散可行规划：问题表述（源：LaValle §2.1）

> L 把搜索算法先在**离散状态空间**讲清，再（§2.4 / §5.4）推广到连续 C-space 的栅格/采样图。本书目标章的 A*/Dijkstra 即此框架。

**Formulation 2.1（离散可行规划，L §2.1.1）**：
1. 非空状态空间 $X$（有限或可数无穷）；
2. 每个 $x\in X$ 有有限**动作空间** $U(x)$；
3. **状态转移函数** $f$：对每个 $x\in X,u\in U(x)$ 产生 $f(x,u)\in X$；**状态转移方程**
$$x'=f(x,u);\tag{L 2.1}$$
4. 初始状态 $x_I\in X$；
5. 目标集 $X_G\subset X$。

全体动作 $U=\bigcup_{x\in X}U(x)$（L 2.2）。任务：找有限动作序列把 $x_I$ 变换到某 $X_G$ 中状态。

**有向状态转移图表示（L §2.1.1）**：顶点 = $X$；有向边 $x\to x'$ 当且仅当存在 $u\in U(x)$ 使 $x'=f(x,u)$。$x_I,X_G$ 标为特殊顶点。

**例 2.1（2D 栅格行走，L）**：$X=\{(i,j)\mid i,j\in\mathbb Z\}$，$U=\{(0,1),(0,-1),(1,0),(-1,0)\}$，$U(x)=U$，$f(x,u)=x+u$（如 $x=(3,4),u=(0,1)\Rightarrow(3,5)$），$x_I=(0,0)$，$X_G=\{(100,100)\}$。涂黑格=障碍（删对应顶点与边）。

**例 2.2（魔方，L）**：状态=魔方所有构型（整体朝向无关），每态 12 个动作，目标=六面纯色。说明状态转移图通常**不显式给出**，而在搜索中增量揭示（魔方规则即隐式编码无穷/巨大图）。

## §S2 通用前向搜索框架（L §2.2.1）

**系统性 (systematic)（L §2.2）**：有限图时算法须访问每个可达状态（才能有限时间正确判定有无解）；无穷图时弱化为"迭代趋于无穷时每个可达顶点终被探索"。须标记已访问状态防止无限循环。

**三类状态（L §2.2.1）**：
1. **未访问 (Unvisited)**：尚未访问；初始为除 $x_I$ 外所有状态。
2. **死 (Dead)**：已访问且其所有后继也已访问；对搜索无新贡献。
3. **活 (Alive)**：已遇到但可能仍有未访问后继；初始唯一活态是 $x_I$。

活态存于优先队列 $Q$，**唯一区分各搜索算法的就是 $Q$ 的排序函数**。

**通用前向搜索伪码（L 图2.4 FORWARD SEARCH）**：
```
FORWARD_SEARCH
 1  Q.Insert(x_I) and mark x_I as visited
 2  while Q not empty do
 3      x ← Q.GetFirst()
 4      if x ∈ X_G
 5          return SUCCESS
 6      forall u ∈ U(x)
 7          x' ← f(x, u)
 8          if x' not visited
 9              Mark x' as visited
 10             Q.Insert(x')
 11         else
 12             Resolve duplicate x'
 13 return FAILURE
```
**实现要点（L §2.2.1）**：①恢复路径：在第7行后记 $x'$ 的父指针 $x$，终态回溯到 $x_I$（并可存所用动作）；②判重（第8行）：树形图无需判重；栅格用查找表 $O(1)$；一般情形 $x'$ 须与 $Q$ 中及死态全部比较，代价高，用哈希/精巧数据结构缓解；③某些算法须对每态算并存代价（第12行更新），代价可用于排序 $Q$ 或恢复路径（存"返回 $x_I$ 的最优代价"即足以定出到任一访问态的动作序列，前提是代价满足单调性——由 Dijkstra/A* 保证，更一般须构成导航函数）。

## §S3 具体前向搜索方法（L §2.2.2）

每法都是图2.4 的特例，仅 $Q$ 排序不同。

### §S3.1 广度优先 (Breadth first)（L §2.2.2）
$Q$ = FIFO（先进先出）。前沿均匀扩张；所有 $k$ 步计划在 $k+1$ 步之前穷尽，故首个解步数最少。判重时第12行无事。系统性（即使不判重也系统，只是浪费在无关环上）。运行 $O(|V|+|E|)$（假定基本操作 $O(1)$）。$|V|=|X|$；若各态同动作集 $|E|=|U||X|$；若各态动作集两两不交 $|E|=|U|$。

### §S3.2 深度优先 (Depth first)（L §2.2.2）
$Q$ = 栈（LIFO）。激进深入；偏好早期长计划（但所选长计划是任意的——`forall` 顺序随定义）。判重时第12行无事。有限 $X$ 系统、无穷 $X$ 不系统（可能只盯一个方向）。运行 $O(|V|+|E|)$。

### §S3.3 Dijkstra 算法（L §2.2.2）—— 单源最短路 / 动态规划特例

每边 $e\in E$ 有非负代价 $l(e)$，可写 $l(x,u)$（从 $x$ 施 $u$ 的代价）。计划总代价 = 路径边代价之和。$Q$ 按**到达代价 (cost-to-come)** $C:X\to[0,\infty]$ 排序。$C^*(x)$ = 从 $x_I$ 到 $x$ 的**最优**到达代价（所有 $x_I\to x$ 路径上累计代价之最小）；未知最优时写 $C(x)$。

**增量计算**：初始 $C^*(x_I)=0$。每生成 $x'$，算
$$C(x')=C^*(x)+l(e)=C^*(x)+l(x,u),$$
$e$ 是 $x\to x'$ 的边。$C(x')$ 是目前已知最优到达代价（尚不确定是否真最优故不写 $C^*$）。第12行：若 $x'$ 已在 $Q$ 中且新路径更优，则降低 $C(x')$ 并重排 $Q$。

**何时 $C(x)$ 变成 $C^*(x)$？**（L §2.2.2 归纳论证）当 $x$ 经 `Q.GetFirst()` 被取出变死态时，$x$ 不可能再以更低代价到达。归纳：base $C^*(x_I)$ 已知；设所有死态最优到达代价已正确（不再变）；$Q$ 首元 $x$ 的值必最优，因任何更低总代价的路径都得穿过 $Q$ 中另一态而那些态代价更高，且只经死态的路径已在算 $C(x)$ 时考虑过。取出 $x$、探索其所有出边后 $x$ 变死，归纳继续。运行 $O(|V|\lg|V|+|E|)$（Fibonacci 堆实现 $Q$、其余 $O(1)$）。

### §S3.4 A* 搜索（L §2.2.2）

**思想**：在 Dijkstra 上加入对"从某态到目标代价"的启发下界，减少探索态数。记 $C(x)$ = cost-to-come，$G(x)$ = 从 $x$ 到某 $X_G$ 态的 **cost-to-go**。$C^*$ 可由动态规划增量算，但 $G^*$（真最优 cost-to-go）事先无法知。许多问题可构造其**低估 (underestimate)**，记 $\hat G^*(x)$。
- 例（L 图2.2 迷宫，代价=步数）：态 $(i,j)$ 到 $(i',j')$ 的 $|i'-i|+|j'-j|$（曼哈顿距离）是低估（忽略障碍的直达计划长度；含障碍后代价只增）。零也是低估但无信息。目标：尽量贴近 $G^*$ 又保证不超过。

**A* 与 Dijkstra 的唯一差别（L §2.2.2）**：$Q$ 排序函数。A* 用
$$\boxed{\text{priority}(x')=C^*(x')+\hat G^*(x')}$$
即按"从 $x_I$ 经 $x'$ 到 $X_G$ 的最优总代价之估计"排序。
- **可采纳性 (admissibility)**：若 $\hat G^*(x)$ 对所有 $x\in X$ 都低估真最优 cost-to-go，则 **A* 保证找到最优计划**（L 引 [13,29]）。
- $\hat G^*$ 越贴近 $G^*$，相比 Dijkstra 探索顶点越少。$\hat G^*(x)\equiv0$ 时 A* **退化为 Dijkstra**。任何情形 A* 都系统。

> **本书记号转换提醒**：L 的 $C^*$=cost-to-come 对应经典 A* 的 $g$；L 的 $\hat G^*$=启发下界对应经典 $h$；优先级 $C^*+\hat G^*$ 对应经典 $f=g+h$。本书写 A* 建议用 $g/h/f$。可采纳性即 $h(x)\le h^*(x)$（$h^*$=真 cost-to-go）。
> **\rebuilt（一致性 consistency / 单调性）**：L 正文未单列"consistency（一致性，$h(x)\le l(x,u)+h(x')$）"。经典结果：可采纳保证最优；**一致**（更强）则保证 A* 每态只需扩展一次（闭表无需重开）。综合时建议补一句一致性定义，依据 Hart-Nilsson-Raphael 1968 与 Russell-Norvig 教材。

### §S3.5 最佳优先 (Best first)（L §2.2.2）
$Q$ 仅按 cost-to-go 估计排序。**解不保证最优**（故估计是否超真值无所谓）；常探索远少顶点、更快，但无保证、最坏比 A*/DP 差，因过于贪心（早期偏好"看起来好"的态）。**不系统**。L 图2.5 给反例：3D 螺旋管，从管口进入会绕螺旋而非直奔目标。

### §S3.6 迭代加深 (Iterative deepening) 与 IDA*（L §2.2.2）
分支因子大时优选。用深度优先找距 $x_I$ 不超过 $i$ 的所有态；未找到目标则**弃前功**，再找不超过 $i+1$ 的；从 $i=1$ 起。把深度优先变系统。弃前功合理：第 $i+1$ 层态数常远超第 $i$ 层（如10倍），故前功可忽略。比广度优先最坏更好且省空间（DFS 队列远小于 BFS）。**IDA***：把 $i$ 换成 $C^*(x')+\hat G^*(x')$，每轮逐步放宽允许总代价（L 引 [29]）。

## §S4 其他通用搜索模板（L §2.2.3）

### §S4.1 后向搜索 (Backward search)（L §2.2.3）
从 $x_G$ 起反向搜（分支因子在 $x_I$ 端大时更高效）。需"反向状态转移"：
$$U^{-1}=\{(x,u)\in X\times U\mid x\in X,u\in U(x)\},$$
$$U^{-1}(x')=\{(x,u)\in U^{-1}\mid x'=f(x,u)\}\tag{L 2.3}$$
（$x'$ 的**反向动作空间**）。记 $u^{-1}=(x,u)\in U^{-1}(x')$，反向转移方程
$$x=f^{-1}(x',u^{-1}).$$
图解：反转每条边方向；反图上的后向搜索 = 原图上的前向搜索。

**后向搜索伪码（L 图2.6 BACKWARD SEARCH）**：
```
BACKWARD_SEARCH
 1  Q.Insert(x_G) and mark x_G as visited
 2  while Q not empty do
 3      x' ← Q.GetFirst()
 4      if x = x_I               (注：判 x'是否已抵 x_I)
 5          return SUCCESS
 6      forall u^{-1} ∈ U^{-1}(x)
 7          x ← f^{-1}(x', u^{-1})
 8          if x not visited
 9              Mark x as visited
 10             Q.Insert(x)
 11         else
 12             Resolve duplicate x
 13 return FAILURE
```
（处理目标区 $X_G$：第1行把所有 $x_G\in X_G$ 入队并标已访问。）

### §S4.2 双向搜索 (Bidirectional search)（L §2.2.3，图2.7）
一树从 $x_I$ 生长、一树从 $x_G$（设 $X_G=\{x_G\}$）生长，两树相遇即成功；任一队列空则失败。常大幅减少探索。Dijkstra/A* 有双向最优变体；best-first 等变体难保两树相遇（可能近而不连）。可扩展到任意多树，但连接更复杂昂贵。

**双向搜索伪码（L 图2.7 BIDIRECTIONAL SEARCH）**：
```
BIDIRECTIONAL_SEARCH
 1  Q_I.Insert(x_I) and mark x_I as visited
 2  Q_G.Insert(x_G) and mark x_G as visited
 3  while Q_I not empty and Q_G not empty do
 4      if Q_I not empty
 5          x ← Q_I.GetFirst()
 6          if x already visited from x_G
 7              return SUCCESS
 8          forall u ∈ U(x)
 9              x' ← f(x, u)
 10             if x' not visited
 11                 Mark x' as visited
 12                 Q_I.Insert(x')
 13             else
 14                 Resolve duplicate x'
 15     if Q_G not empty
 16         x' ← Q_G.GetFirst()
 17         if x' already visited from x_I
 18             return SUCCESS
 19         forall u^{-1} ∈ U^{-1}(x')
 20             x ← f^{-1}(x', u^{-1})
 21             if x not visited
 22                 Mark x as visited
 23                 Q_G.Insert(x)
 24             else
 25                 Resolve duplicate x
 26 return FAILURE
```

### §S4.3 搜索方法的统一视角（L §2.2.4）—— 与采样式规划的桥
所有方法共享六步模板（L §5.4 把采样式规划视为其向连续空间的推广）：
1. **初始化**：搜索图 $G(V,E)$，$E$ 空，$V$ 含起始态（前向 $V=\{x_I\}$、后向 $V=\{x_G\}$、双向 $V=\{x_I,x_G\}$）；
2. **选顶点**：用优先队列选 $n_{cur}\in V$ 扩展（对应态 $x_{cur}$）；
3. **施动作**：前向 $x_{new}=f(x,u)$ 或后向 $x=f(x_{new},u)$ 得新态；
4. **插边**：通过算法特定测试则加边（前向 $x\to x_{new}$、后向 $x_{new}\to x$），$x_{new}\notin V$ 则插入 $V$；
5. **查解**：判 $G$ 是否含 $x_I\to x_G$ 路径（单树平凡、多树昂贵）；
6. **回到第2步**：除非找到解或满足早停。

## §S5 离散最优规划：值迭代/动态规划（L §2.3）

> 本节是 A*/Dijkstra/D* 背后的动态规划原理。本书目标章 A* 部分可用作"为何 A* 最优"的理论支撑。

### §S5.1 定长最优计划与（后向）值迭代（L §2.3.1）

**Formulation 2.2（定长最优规划，L）**：继承 Formulation 2.1（$X$ 有限），加：阶段数 $K$（计划恰 $K$ 个动作）；**阶段可加损失泛函** $L$：
$$L(\pi_K)=\sum_{k=1}^{K}l(x_k,u_k)+l_F(x_F),\quad F=K+1.\tag{L 2.4}$$
$l_F(x_F)=0$ 若 $x_F\in X_G$，否则 $=\infty$（把可行性约束转成优化：$L=\infty$ 即不可行）。设 $l\equiv0$ 退化为可行规划；$l\equiv1$ 即最小化步数。

**最优 cost-to-go（L 式2.5–2.6）**：
$$G_k^*(x_k)=\min_{u_k,\dots,u_K}\Big\{\sum_{i=k}^{K}l(x_i,u_i)+l_F(x_F)\Big\},\qquad G_F^*(x_F)=l_F(x_F).\tag{L 2.5, 2.6}$$

**最优性原理推导递推（L 式2.7–2.11）**：先算第二轮
$$G_K^*(x_K)=\min_{u_K}\{l(x_K,u_K)+l_F(x_F)\}=\min_{u_K}\{l(x_K,u_K)+G_F^*(f(x_K,u_K))\}.\tag{L 2.7, 2.8}$$
把 (2.5) 拆首项、分离 $u_k$ 的 min：
$$G_k^*(x_k)=\min_{u_k}\Big\{l(x_k,u_k)+\min_{u_{k+1},\dots,u_K}\big[\sum_{i=k+1}^{K}l(x_i,u_i)+l_F(x_F)\big]\Big\},\tag{L 2.9–2.10}$$
内 min 即 $G_{k+1}^*$，得**动态规划递推（Bellman 形式）**：
$$\boxed{G_k^*(x_k)=\min_{u_k}\big\{l(x_k,u_k)+G_{k+1}^*(x_{k+1})\big\},\quad x_{k+1}=f(x_k,u_k).}\tag{L 2.11}$$
每次值迭代 $O(|X||U|)$。值迭代序列
$$G_F^*\to G_K^*\to G_{K-1}^*\to\cdots\to G_1^*\tag{L 2.12}$$
共 $O(K|X||U|)$；$G_1^*(x_I)$ 即从 $x_I$ 到目标最优代价。

**例 2.3（五态定长，L）**：$X=\{a,b,c,d,e\}$，$K=4,x_I=a,X_G=\{d\}$，边代价见 L 图2.8。后向值迭代结果（L 图2.9）：
| | a | b | c | d | e |
|---|---|---|---|---|---|
| $G_5^*$ | ∞ | ∞ | ∞ | 0 | ∞ |
| $G_4^*$ | ∞ | 4 | 1 | ∞ | ∞ |
| $G_3^*$ | 6 | 2 | ∞ | 2 | ∞ |
| $G_2^*$ | 4 | 6 | 3 | ∞ | ∞ |
| $G_1^*$ | 6 | 4 | 5 | 4 | ∞ |

### §S5.2 前向值迭代（L §2.3.1）
对偶：算从初始阶段起的最优 **cost-to-come** $C_k^*$。
$$C_1^*(x_1)=l_I(x_1)\ (\,l_I(x_I)=0,\ l_I(x\ne x_I)=\infty\,);\tag{L 2.13}$$
中间阶段 $k\in\{2,\dots,K\}$：
$$C_k^*(x_k)=\min_{u_1,\dots,u_{k-1}}\Big\{l_I(x_1)+\sum_{i=1}^{k-1}l(x_i,u_i)\Big\};\tag{L 2.14}$$
末阶段
$$C_F^*(x_F)=\min_{u_1,\dots,u_K}\Big\{l_I(x_1)+\sum_{i=1}^K l(x_i,u_i)\Big\};\tag{L 2.15}$$
递推（用反向转移 $x_{k-1}=f^{-1}(x_k,u_k^{-1})$）：
$$\boxed{C_k^*(x_k)=\min_{u_k^{-1}\in U^{-1}(x_k)}\big\{C_{k-1}^*(x_{k-1})+l(x_{k-1},u_{k-1})\big\}.}\tag{L 2.16}$$
$O(K|X||U|)$。例 2.4（L 图2.12）前向 cost-to-come 表：
| | a | b | c | d | e |
|---|---|---|---|---|---|
| $C_1^*$ | 0 | ∞ | ∞ | ∞ | ∞ |
| $C_2^*$ | 2 | 2 | ∞ | ∞ | ∞ |
| $C_3^*$ | 4 | 4 | 3 | 6 | ∞ |
| $C_4^*$ | 4 | 6 | 5 | 4 | 7 |
| $C_5^*$ | 6 | 6 | 5 | 6 | 5 |

### §S5.3 变长计划与终止动作（L §2.3.2）
**Formulation 2.3（离散最优规划，L）**：继承 2.1 + 阶段概念；损失泛函 $L(\pi_K)=\sum_{k=1}^K l(x_k,u_k)+l_F(x_F)$（L 2.17，$K$ 不再预定）；每 $U(x)$ 含**终止动作** $u_T$：施 $u_T$ 后永远重复、状态不变、不再累代价（$\forall i\ge k:u_i=u_T,x_i=x_k,l(x_i,u_T)=0$）。

**无界后向值迭代→稳态（L §2.3.2）**：因 $x_I$ 不参与后向值迭代，可一直算到 $G_0^*,G_{-1}^*,\dots$。当 $l\ge0$ 时，存在阶段使 cost-to-go **稳态**：$\forall i\le k:G_{i-1}^*(x)=G_i^*(x)$，此时去掉阶段下标得递推
$$\boxed{G^*(x)=\min_u\{l(x,u)+G^*(f(x,u))\}}\tag{L 2.18}$$
（即**Bellman 方程**稳态形式）。$l$ 非负 $\Rightarrow$ 必稳态（上界 = 所有可达 $X_G$ 的态中最优计划阶段数之最大值）。**负环**则代价可 $\to-\infty$，须禁（可预先检测）。

**恢复最优动作（L 式2.19）**：从 $G^*$ 直接得
$$\boxed{u^*=\arg\min_{u\in U(x)}\{l(x,u)+G^*(f(x,u))\}}\tag{L 2.19}$$
逐步 $x'=f(x,u^*)$ 直到 $X_G$。$G^*$ 是从任意态把系统最优引向目标的"导航函数"。前向对偶（L 2.20）：
$$\arg\min_{u^{-1}\in U^{-1}}\{C^*(f^{-1}(x,u^{-1}))+l(f^{-1}(x,u^{-1}),u')\}.\tag{L 2.20}$$
例 2.5（L 图2.14）变长后向 cost-to-go 稳态 $G^*$：$a{=}4,b{=}2,c{=}1,d{=}0,e{=}\infty$（$d$ 从 $e$ 不可达故 $\infty$）。

### §S5.4 Dijkstra 再访 + 标签校正（L §2.3.3）
**两类动态规划对比**：值迭代每轮遍历整个 $X$；Dijkstra 只流过 $X$ 一次但需维护"哪些态活"。Dijkstra 可由前向值迭代聚焦"有趣变化"导出：不可达态恒 $\infty$（=未访问）；最优 cost-to-come 已稳的态=死态；其余态先得初值再被降若干次到最优。

**标签校正算法（L §2.3.3，图2.16 FORWARD LABEL CORRECTING）**：Dijkstra 属更广的**标签校正**族——允许死态在发现更优 cost-to-come 时复活：
```
FORWARD_LABEL_CORRECTING(x_G)
 1  Set C(x) = ∞ for all x ≠ x_I, and set C(x_I) = 0
 2  Q.Insert(x_I)
 3  while Q not empty do
 4      x ← Q.GetFirst()
 5      forall u ∈ U(x)
 6          x' ← f(x, u)
 7          if C(x) + l(x, u) < min{C(x'), C(x_G)} then
 8              C(x') ← C(x) + l(x, u)
 9              if x' ≠ x_G then
 10                 Q.Insert(x')
```
**与 Dijkstra/A* 关系**：Dijkstra、A* 每态只访问一次，故图2.4 与图2.16 在它们身上本质相同；但标签校正对**任意** $Q$ 排序（FIFO/LIFO 皆可）都产生最优解（只要 $X$ 有限）。差异：第7行用目标代价 $C(x_G)$ 剪枝（仅适单目标）；$x_G$ 不入队（第9–10行，无须以 $x_G$ 为中间态）。

> **抽取员注**：D*（下节）正是把"标签校正 + 增量重规划"做到极致——边代价在运行中改变时，只把受影响的代价变化以波形局部传播，而非整图重算。

## §S6 D*（动态 A*）：部分已知/动态环境的增量最优规划（源：Stentz 1994）`\rebuilt`

> **源**：A. Stentz, *Optimal and Efficient Path Planning for Partially-Known Environments*, ICRA 1994（技报名 *…for Unknown and Dynamic Environments*, CMU-RI）。
> **抽取可靠性声明**：Stentz 1994 原 PDF 为 CCITT 传真扫描位图，**无文本层，无法 OCR 复现其逐行编号伪码（原文 PROCESS-STATE/MODIFY-COST 约 L1–L33）**。以下设定、key 定义、RAISE/LOWER 机制、PROCESS-STATE/MODIFY-COST 的**逻辑结构**据论文正文描述 + LaValle §12.3.2 对 D* 的转述 + Wikipedia *D\** 条目复建，**逻辑等价但行号与具体写法可能与原文有出入，整节标 `\rebuilt`**。逐字原文请回 IEEE Xplore / ICRA 1994 原版。

### §S6.1 动机与定位（St）
A* 假设环境模型完整且静止。**D* 之名 = Dynamic A***：弧代价可在算法运行中改变。典型场景：移动机器人先按已知（或乐观假设）地图规划，行进中传感器发现新障碍/代价变化，需**高效**地修正（而非整路重规划，也不牺牲最优）。D* 初次规划方式类似 A*，但具备代价变化时的局部重规划能力。**D* 反向搜索：树根在 goal**，对每个可能的 start 节点都算出到 goal 的最优路径（故机器人移动后无需重启）。

### §S6.2 形式设定（St）`\rebuilt`
- **状态** $X$：环境离散化的节点（如栅格）。
- **弧代价** $c(X,Y)$：从状态 $X$ 移到邻接 $Y$ 的代价（$c$ 可在运行中改变；$c=\infty$ 表不可通行）。
- **反向指针** $b(X)=Y$：表示沿当前最优路径，$X$ 的下一个（朝 goal 的）状态是 $Y$。沿 $b(\cdot)$ 链即得路径。
- **路径代价估计** $h(X)$：当前估计的从 $X$ 到 goal 的路径代价（**注意：St 的 $h$ 是"到目标代价"cost-to-goal，不是 A* 文献里的启发下界**；见 §0.1 记号警告）。
- **标签** $t(X)\in\{\textsf{NEW},\textsf{OPEN},\textsf{CLOSED}\}$：`NEW`=从未入过 OPEN 表；`OPEN`=当前在 OPEN 表（待处理）；`CLOSED`=已移出 OPEN 表。
- **OPEN 表**：待处理状态的优先队列（按 key 排序），与 Dijkstra/A* 同。

### §S6.3 关键函数 key $k(X)$ 与 RAISE/LOWER（St）`\rebuilt`
**key** $k(X)$ 定义为：自 $X$ 上次被放入 OPEN 表以来，$h(X)$ 取过的所有值（含当前 $h(X)$）之**最小**：
$$\boxed{k(X)=\min\big(h(X),\ \text{$X$ 入 OPEN 以来 $h(X)$ 的历史值}\big).}$$
OPEN 表按 $k$ 升序取出（最小 $k$ 优先）。记当前 OPEN 表最小 key 为 $k_{\min}=\min_{X\in\text{OPEN}}k(X)$（若 OPEN 空则 $k_{\min}=-1$ 表示终止）；处理时常记 $k_{old}=k_{\min}$。

key 把 OPEN 上的状态分两类：
- **LOWER 状态**：$k(X)=h(X)$。表示自上次入 OPEN 后 $h(X)$ 未升（代价下降或不变），该状态可向邻居**传播更低代价**。
- **RAISE 状态**：$k(X)<h(X)$。表示 $h(X)$ 曾被抬高（某弧代价增大使路径变贵），该状态需向邻居**传播代价上升**、并寻找可能的更优新父。
（RAISE/LOWER 也常被当作状态标记名，随代价升降的"波"在图上传播。）

### §S6.4 PROCESS-STATE（D* 核心，处理 OPEN 表一个状态）`\rebuilt`
> 逻辑等价复建（非原文逐字）。返回 $k_{\min}$（OPEN 处理后的最小 key），$=-1$ 表 OPEN 空。

```
PROCESS-STATE()                                        // 处理 OPEN 表上 key 最小的状态
 1   X ← MIN-STATE()                                   // 取 OPEN 中 k 最小者
 2   if X = NULL then return -1                        // OPEN 空
 3   k_old ← GET-KMIN(); DELETE(X)                     // k_old = 处理前最小 key；X 移出 OPEN

 4   if k_old < h(X) then                              // —— RAISE 处理：先尝试经邻居降低自身 h ——
 5       for each neighbor Y of X:
 6           if t(Y) ≠ NEW and h(Y) ≤ k_old and h(X) > h(Y) + c(Y, X) then
 7               b(X) ← Y;  h(X) ← h(Y) + c(Y, X)      // 找到更优父，修正自身

 8   if k_old = h(X) then                              // —— LOWER 状态：向邻居传播更低代价 ——
 9       for each neighbor Y of X:
10           if t(Y) = NEW
11              or (b(Y) = X and h(Y) ≠ h(X) + c(X, Y))
12              or (b(Y) ≠ X and h(Y) > h(X) + c(X, Y)) then
13                  b(Y) ← X;  INSERT(Y, h(X) + c(X, Y))   // 经 X 给 Y 更优代价，重排
14   else                                              // —— RAISE 状态：k_old < h(X) ——
15       for each neighbor Y of X:
16           if t(Y) = NEW
17              or (b(Y) = X and h(Y) ≠ h(X) + c(X, Y)) then
18                  b(Y) ← X;  INSERT(Y, h(X) + c(X, Y))   // 经 X 的子节点须随 X 抬高
19           else
20               if b(Y) ≠ X and h(Y) > h(X) + c(X, Y) then
21                   INSERT(X, h(X))                    // X 可经 Y 改善 → 重置 X 待再处理
22               else
23                   if b(Y) ≠ X and h(X) > h(Y) + c(Y, X)
24                      and t(Y) = CLOSED and h(Y) > k_old then
25                       INSERT(Y, h(Y))                // 受影响的次优 CLOSED 邻居须重新评估

26   return GET-KMIN()
```

### §S6.5 MODIFY-COST（弧代价改变时调用）`\rebuilt`
> 当机器人发现 $c(X,Y)$ 变化（如新障碍使 $c=\infty$），更新代价并把 $X$ 放回 OPEN 以触发传播。

```
MODIFY-COST(X, Y, c_new)
 1   c(X, Y) ← c_new
 2   if t(X) = CLOSED then
 3       INSERT(X, h(X))            // 把 X 重新放回 OPEN，h 不变 → 通常成为 RAISE 源
 4   return GET-KMIN()
```
**用法（St）**：机器人沿 $b(\cdot)$ 链朝 goal 移动；每检测到弧代价变化就调 `MODIFY-COST`，随后反复调 `PROCESS-STATE` 直到 $k_{\min}\ge h(\text{机器人当前态})$（即与机器人相关的代价波已稳定），再继续移动。如此只重算受影响区域。

### §S6.6 辅助函数（St）`\rebuilt`
- **MIN-STATE()**：返回 OPEN 表中 $k$ 最小的状态（空则 NULL）。
- **GET-KMIN()**：返回 OPEN 表当前最小 $k$（空则 $-1$）。
- **DELETE(X)**：把 $X$ 移出 OPEN，置 $t(X)=\textsf{CLOSED}$。
- **INSERT(X, h_new)**：设 $h(X)\leftarrow h_{new}$；若 $X$ 是 `NEW` 置 $k(X)=h_{new}$；若已在 `OPEN` 置 $k(X)=\min(k(X),h_{new})$；若 `CLOSED` 置 $k(X)=\min(h(X),h_{new})$ 并重新入 OPEN、置 $t(X)=\textsf{OPEN}$。

### §S6.7 关键性质 / 定理（St）`\rebuilt`
> 复建陈述（论文原定理编号未能从扫描件取得，按内容表述）。
- **(D*-P1，最优性)** 当状态 $X$ 经 `PROCESS-STATE` 被置为 `CLOSED` 且 $k_{\min}\ge h(X)$ 时，$h(X)$ 等于从 $X$ 到 goal 的**最优**路径代价（在当前已知代价下）。反复 `PROCESS-STATE` 直到目标态相关代价稳定，沿 $b(\cdot)$ 链即得最优路径——此性质与 Dijkstra"出队即定最优"同理。
- **(D*-P2，与 A* 一致)** 初次规划（无代价变化）时，D* 的 `PROCESS-STATE` 序列产生与（反向）A*/Dijkstra 相同的最优路径与代价。
- **(D*-P3，增量等价)** 弧代价改变后，D* 经 `MODIFY-COST` + 若干 `PROCESS-STATE` 得到的解，与"用新代价从头跑一遍最优规划"得到的解**相同**（最优性保持），但通常只触及受影响子集，故远快于整图重算。
- **(D*-P4，完备/终止)** 代价非负且图有限时，OPEN 表终将清空（$k_{\min}=-1$），算法终止；若存在解必返回最优解。

### §S6.8 变体（St-wiki）
- **Focussed D*（Stentz 1995）**：把 A* 启发引导融入 D*，用启发把代价传播波**聚焦**于机器人附近，进一步减少处理状态数。
- **D* Lite（Koenig & Likhachev 2002）**：不基于原 D* 而基于 LPA*（Lifelong Planning A*）重新设计，逻辑更简、性能相当或更优，**现今实践首选**。`\rebuilt`：D* Lite 维护 $g$（cost-to-goal，反向）与 rhs（一步前瞻值），用两元素 key $[\min(g,rhs)+h(s_{start},s); \min(g,rhs)]$ 排序 OPEN，靠 rhs≠g 判"局部不一致"驱动重算。详见 Koenig & Likhachev, *D\* Lite*, AAAI 2002（本抽取未取原文，标 `\rebuilt`，综合若深入需回原论文）。

> **抽取员小结（搜索式）**：Dijkstra/A* 是静态单查询最优图搜索（A*=Dijkstra+可采纳启发）；D* 系列是其在**动态/部分已知**环境下的增量化，核心是 key 的"历史最小"定义 + RAISE/LOWER 双波传播，把"边代价改变"的影响局部化。三者都建立在 §S5 动态规划/Bellman 递推之上。

---

---

# 第三部分　采样式运动规划：PRM、RRT/RDT、RRT*（源：LaValle 第5章 + Karaman-Frazzoli 2011）

> **动因（L §5，§C5.2）**：Piano Mover's Problem 是 PSPACE-hard，高维 $\mathcal C_{obs}$ 显式构造不可行。采样式规划的总思想（L 图5.1）：**避免显式构造 $\mathcal C_{obs}$**，改用采样探测 $\mathcal C$，并把**碰撞检测当黑箱**——规划算法只问"此构型/此线段是否无碰撞"，由独立的碰撞检测模块回答（半代数集/三角网/凸多面体等几何细节对规划器透明）。

## §P1 完备性的弱化概念（L §5 引言）

- **完备 (complete)**：对任意输入，有限时间内正确报告有无解（组合方法可达，L 第6章）。采样式达不到。
- **稠密 (dense)**：采样随迭代趋于无穷而任意逼近任一构型（关键弱化概念）。
- **分辨率完备 (resolution complete)**：确定性稠密采样的算法——有解则有限时间找到；无解可能永远运行。
- **概率完备 (probabilistically complete)**：基于随机采样（以概率1稠密）——样本足够多时，找到存在解的概率收敛到1。
- 最相关却最难建立的是**收敛速率**。

## §P2 C-space 上的距离与体积（L §5.1）

几乎所有采样式算法都需 $\mathcal C$ 上的距离函数（度量空间）与（有时）子集体积（测度空间）。

### §P2.1 度量空间（L §5.1.1）
**度量空间** $(X,\rho)$：拓扑空间 $X$ + 函数 $\rho:X\times X\to\mathbb R$ 满足（任意 $a,b,c\in X$）：①非负 $\rho(a,b)\ge0$；②自反 $\rho(a,b)=0\iff a=b$；③对称 $\rho(a,b)=\rho(b,a)$；④三角不等式 $\rho(a,b)+\rho(b,c)\ge\rho(a,c)$。

**$L_p$ 度量（L §5.1.1）**：$\mathbb R^n$ 上 $L_p$ 范数（$p\ge1$）
$$\|x\|_p=\Big(\sum_{i=1}^n|x_i|^p\Big)^{1/p};\tag{L 5.3}$$
$L_2$=欧氏度量（$\rho(x,y)=\|x-y\|$）；$L_1$=曼哈顿度量；$L_\infty$：
$$L_\infty(x,x')=\max_i\{|x_i-x_i'|\}.\tag{L 5.2}$$

**度量子空间（L §5.1.1）**：$(X,\rho)$ 的子集 $Y$ 限制 $\rho$ 到 $Y\times Y$ 仍是度量空间——故第4章任一流形/簇可借其嵌入空间 $\mathbb R^m$ 的 $L_p$ 度量。

**度量空间的笛卡尔积（L 式5.4–5.5）**：$(X,\rho_x),(Y,\rho_y)$，$Z=X\times Y$ 上（$z=(x,y)$，两正常数 $c_1,c_2>0$）：
$$\rho_z(z,z')=c_1\rho_x(x,x')+c_2\rho_y(y,y');\tag{L 5.4}$$
或 $L_p$ 式
$$\rho_z(z,z')=\big(c_1\rho_x(x,x')^p+c_2\rho_y(y,y')^p\big)^{1/p}.\tag{L 5.5}$$

### §P2.2 运动规划常用度量（L §5.1.2）
- **例 5.1（$SO(2)$ 复数法）**：$SO(2)$ 表为 $\{(a,b)\mid a^2+b^2=1\}$，用 $\mathbb R^2$ 的 $L_2$：$\rho=\sqrt{(a_1-a_2)^2+(b_1-b_2)^2}$。
- **例 5.2（$SO(2)$ 比角法）**：直接比 $\theta_1,\theta_2$，须取沿 $S^1$ 的最短弧 $\rho=\min(|\theta_1-\theta_2|,2\pi-|\theta_1-\theta_2|)$。
- **例 5.3（$SE(2)$）**：用复数表示嵌入 $\mathbb R^4$，任意 $L_p$ 即得度量。
- **例 5.4（$SO(3)$ 四元数法，L 式5.9）**：$S^3$ 上 $L_p$ 不是 $SO(3)$ 度量（须对径等同）；用内积，$h_1,h_2\in\mathbb R^4$：
$$\rho(h_1,h_2)=\min\big(\cos^{-1}|h_1\cdot h_2|,\ \cos^{-1}|h_1\cdot(-h_2)|\big)$$
（两参数对应到 $h_2$ 与 $-h_2$ 的距离，取小者，尊重对径等同）。
- **例 5.5（$SE(2)$ 另一度量）**：须比较平面距离与角量；$\mathbb R^2$ 与 $S^1$ 的单位不可通约——同一常数 $c_2$ 用弧度 vs 度会得两种很不同的度量。
- **例 5.6（机器人位移度量）**：$\rho(q_1,q_2)=\max_{a\in\mathcal A}\|a(q_1)-a(q_2)\|$（机器人上点的最大位移），物理意义最佳但难高效计算。
- **例 5.7（$T^n$ 度量）**：笛卡尔积规则推广到环面每个 $S^1$ 分量。
- **例 5.8（$SE(3)$ 度量）**：对 $\mathbb R^3$ 度量与 $SO(3)$ 度量用笛卡尔积规则组合。

**伪度量 (pseudometric)（L §5.1.2）**：满足部分度量公理（如不对称）的"类距离"函数。常见来源：取某准则的最优 cost-to-go（如车辆耗能不对称）；又如势函数（估计到目标的距离，见 §P5.2）。

### §P2.3 测度理论与不变测度（L §5.1.3–5.1.4，扼要）
- **σ-代数 / Borel 集 / Lebesgue 积分**：定义体积所需（L §5.1.3）。
- **Haar 测度（L §5.1.4）**：群上的不变测度，应尽量用之以避免参数化引入的偏置。$SO(3)$ 的 Haar 测度 = $S^3$ 上均匀（用四元数；欧拉角均匀采样**不**给 $SO(3)$ 均匀）。
> **抽取员注**：本节对本书目标章"导论"层级可压缩为一句"度量给最近邻/连接、测度给均匀采样与体积，$SO(3)$ 须用四元数/Haar"。完整定义见 L §5.1.3–5.1.4。

## §P3 采样理论：稠密、弥散、偏差（L §5.2）

### §P3.1 稠密与随机采样（L §5.2.1–5.2.2）
**稠密 (dense)（L §5.2.1）**：$U$ 在 $V$ 中稠密 $\iff\mathrm{cl}(U)=V$。采样方法底线=产生稠密序列。$\mathbb Q$ 在 $\mathbb R$ 中可数且稠密。

**随机以概率1稠密（L §5.2.1）**：$\mathcal C=[0,1]$ 上随机取点，"前 $k$ 点未落入某固定子区间"的概率随 $k\to\infty$ 趋0，故随机序列**以概率1稠密**（非确定稠密）。

**均匀随机采样（L §5.2.2）**：概率密度在 $\mathcal C$ 上均匀且与 Haar 测度一致。优势：C-space 由笛卡尔积构成，独立随机样本沿积自然延拓——$X=X_1\times X_2$，$x_1,x_2$ 各自均匀则 $(x_1,x_2)$ 在 $X$ 均匀。
- **均匀随机 $SO(3)$（L 式5.15）**：取 $u_1,u_2,u_3\in[0,1]$ 均匀，
$$h=\Big(\sqrt{1-u_1}\sin2\pi u_2,\ \sqrt{1-u_1}\cos2\pi u_2,\ \sqrt{u_1}\sin2\pi u_3,\ \sqrt{u_1}\cos2\pi u_3\Big).\tag{L 5.15}$$
落在下半球（$a<0$）的 $h$ 取 $-h$ 以尊重对径等同。
- **随机方向（L §5.2.2）**：在 $S^n$ 上采。$n+1$ 个坐标各取零均值同方差高斯 $u_i$（由中心极限：$k\ge12$ 个 $[-1,1]$ 均匀样本求和近似），归一化 $u_i/\|u\|$——高斯保证球对称（均匀立方采样会偏向角）。

**伪随机数（L §5.2.2）**：计算机数非真随机。线性同余 $y_{i+1}=ay_i+c\bmod M$（L 5.16），$x_i=y_i/M$（L 5.17），周期 $M$（如 $2^{31}-1$）。**Mersenne Twister** 是避免笛卡尔积上确定性依赖的好选择。卡方检验（L 5.18）$e(P)=\sum_{i=1}^{100}(b_i-k/100)^2$ 测均匀性（反直觉：$e$ 太小说明"太均匀而不像随机"）。

### §P3.2 弥散 (Dispersion)（L §5.2.3）—— "分辨率"的推广
**van der Corput 序列（L §5.2.1，表5.2）**：$[0,1]$ 上反转二进制位得到的低弥散序列。朴素计数 0,1,2,…的二进制反位→序列在 $[0,1]/\!\sim$ 上漂亮跳跃；$i$ 为2的幂时点完美等距，其他 $i$ 覆盖仍好（长 $l$ 区间约含 $il$ 点）。记 $\nu(i)$ 为第 $i$ 点。每点远离前一点，前 $i$ 点对任意 $i$ 都较均匀。

**弥散定义（L 式5.19）**：度量空间 $(X,\rho)$ 中有限点集 $P$ 的弥散
$$\boxed{\delta(P)=\sup_{x\in X}\Big(\min_{p\in P}\rho(x,p)\Big).}\tag{L 5.19}$$
即**最大空球半径**（$L_\infty$ 下空球是立方体）。Voronoi 视角：Voronoi 顶点是"离最近样本最远"之点，在各 Voronoi 顶点放空球，最大者半径=弥散。（**反直觉**：弥散越**低**越好，即点分布越均匀。）

**Sukharev 栅格（L §5.2.3，式5.20）**：$L_\infty$、$X=[0,1]^n$、$k$ 点时，最优弥散=把 $[0,1]^n$ 划成立方栅格、每立方中心放一点（每轴 $\lfloor k^{1/n}\rfloor$ 点）。下界（对任意 $k$ 点集 $P$，L 5.20）：
$$\delta(P)\ge\frac{1}{2\lfloor k^{1/d}\rfloor}\quad(\text{原文写 }2k^{1/d}\text{ 分母}).\tag{L 5.20}$$
即固定弥散需点数随维数指数增长。$L_\infty$ 而非 $L_2$ 因 $L_2$ 极难优化（仅 $\mathbb R^2$ 用等边三角形铺砌可解）。
- 例 5.15（$n=2,k=9$）：$[0,1]^2$ 上各坐标取 $1/6,1/2,5/6$，$L_\infty$ 弥散 $1/6$，点距 $1/3$=2倍弥散；环面 $[0,1]^2/\!\sim$ 则可平移为各坐标 $0,1/3,2/3$ 标准栅格。

**格 (lattice)（L 式5.21）**：生成元 $g_j$（$n$ 个独立向量），格点
$$x=\sum_{j=1}^n k_j g_j.\tag{L 5.21}$$
一般生成元不必正交（L 图5.5b）。

**多分辨率栅格序列（L §5.2.3）**：van der Corput 推广到 $n$ 维——每次用 $n$ 个二进制位选 orthant，$k=2^{ni}$ 后得每轴 $2^i$ 点的完整栅格，任意 $k$ 都是部分栅格、$L_\infty$ 最优弥散。Sukharev 多分辨率较难（底变3）。

**弥散渐近界（L §5.2.3）**：$X=[0,1]^n$、任意 $L_p$，最优渐近弥散 $O(k^{-1/n})$（$k$ 为变量、$n$ 为常数；任意 $f(n)$ 当常数）。van der Corput 弥散 $\le1/k$。

### §P3.3 偏差 (Discrepancy)（L §5.2.4）
**偏差定义（L 式5.22）**：测度空间 $X$、值域空间 $\mathcal R$（通常取所有轴对齐矩形）、点集 $P$（$k$ 点）：
$$\boxed{D(P,\mathcal R)=\sup_{R\in\mathcal R}\left|\frac{|P\cap R|}{k}-\frac{\mu(R)}{\mu(X)}\right|.}\tag{L 5.22}$$
即"点数比例"与"体积比例"的最大偏差（衡量用 $P$ 估 $R$ 体积的最坏误差，与卡方检验相关但对所有盒取 sup）。

**渐近界（L §5.2.4）**：轴对齐盒、$[0,1]^n$：单一序列最优偏差 $O(k^{-1}\log^n k)$（$k$ 不预定）；每 $k$ 可换点集时 $O(k^{-1}\log^{n-1}k)$。

**弥散-偏差关系（L 式5.23）**：对任意 $P\subset[0,1]^n$：
$$\delta(P,L_\infty)\le D(P,\mathcal R)^{1/d}.\tag{L 5.23}$$
即低偏差 $\Rightarrow$ 低弥散（逆不真：轴对齐栅格偏差高但弥散低）。

**低偏差采样三类（L §5.2.4）**：
1. **Halton/Hammersley**：van der Corput 的高维推广。Halton（L 5.24–5.25）：取 $n$ 个互素整数 $p_1,\dots,p_n$（通常前 $n$ 素数），第 $i$ 样本第 $j$ 坐标为基-$p_j$ 反位
$$r(i,p)=\frac{a_0}{p}+\frac{a_1}{p^2}+\frac{a_2}{p^3}+\cdots\ \ (i=a_0+pa_1+p^2a_2+\cdots),\tag{L 5.24}$$
$$\text{Halton 第 }i\text{ 样本}=\big(r(i,p_1),\dots,r(i,p_n)\big).\tag{L 5.25}$$
Hammersley（$k$ 已知，L 5.26）：用 $n-1$ 个素数
$$\big(i/k,\ r(i,p_1),\dots,r(i,p_{n-1})\big).\tag{L 5.26}$$
二者渐近最优偏差，但常数随维数超指数增长（高于10维性能显著退化）。
2. **(t,s)-序列与 (t,m,s)-网**：在 canonical 矩形上强制零偏差。著名者 Sobol'、Faure；Niederreiter-Xing 渐近常数最佳 $(a/n)^n$。
3. **格**：栅格的非正交推广。例（L §5.2.4）：$\alpha$ 为正无理数，第 $i$ 点 $(i/k,\{i\alpha\})$（$\{\cdot\}$=小数部分），$\alpha=(\sqrt5+1)/2$（黄金比）。

## §P4 碰撞检测（L §5.3，扼要）

**逻辑谓词（L §5.3.1）**：$\phi:\mathcal C\to\{\text{true},\text{false}\}$，$q\in\mathcal C_{obs}\Rightarrow\phi(q)=\text{true}$。比显式构造 $\mathcal C_{obs}$ 更高效地判单构型是否碰撞。**两集间距离**函数提供"机器人离障碍多远"（规划很需要）。

**路径段无碰检验（L §5.3.4）——采样式规划的关键子程序**：
- **实用法**：固定 C-space 步长 $\Delta q>0$，沿 $\tau$ 取 $t_1,t_2$ 使 $\rho(\tau(t_1),\tau(t_2))\le\Delta q$；$\Delta q$ 经验定（太小费时、太大可能穿薄障碍）。
- **保证法（Lipschitz 界）**：设碰撞检测报 $\mathcal A(q)$ 离障碍至少 $d$；若 $q\to q'$ 沿 $\tau$ 机器人无点移动超 $d$，则 $q,q'$ 及其间全无碰。机器人位移界：
$$\|a(q)-a(q')\|<c\|q-q'\|\ (\text{Lipschitz},\ \text{L 5.32});\quad \|a(q)-a(q')\|<\sum_{i=1}^n c_i|q_i-q_i'|\ (\text{逐参数},\ \text{L 5.34}).$$
逐参数界对长链更优（如50连杆，$\theta_1$ 的步长须远小于 $\theta_{50}$）。$SO(3)$ 用四元数差 $\|h-h'\|$ 表界。
- **检查顺序（L §5.3.4）**：实验表明**递归二分**最佳（无碰时无差别，碰撞时常省时）——这正是 [0,1] 上的采样问题，用 van der Corput 序列 $\nu$：先查 $\tau(1)$，再 $\tau(0),\tau(1/2),\tau(1/4),\tau(3/4),\tau(1/8),\dots$；弥散降到 $\Delta q$ 以下时停。

## §P5 增量采样与搜索的统一框架（L §5.4）

**单查询模型（L §5.4.1）**：$(q_I,q_G)$ 只给一次，无预计算优势。**采样式规划与 §S4.3 的离散搜索六步几乎同构**，唯一大差别：第3步"施动作 $u$"换成"生成路径段 $\tau_s$"；搜索图 $G$ 无向（边=路径）。

**单查询采样式规划通用模板（L §5.4.1）**：
1. **初始化**：无向搜索图 $G(V,E)$，$V$ 至少含一顶点（通常含 $q_I,q_G$ 或两者），$E$ 空；
2. **顶点选择法 (VSM)**：选 $q_{cur}\in V$ 扩展（类比 §2.2 的优先队列 $Q$）；
3. **局部规划法 (LPM)**：对某 $q_{new}\in\mathcal C_{free}$（未必是 $V$ 中顶点），尝试构造 $\tau_s:[0,1]\to\mathcal C_{free}$ 使 $\tau_s(0)=q_{cur},\tau_s(1)=q_{new}$，并用 §5.3.4 检验无碰；失败则回第2步；
4. **插边**：把 $\tau_s$ 作为 $q_{cur}\to q_{new}$ 的边插入 $E$，$q_{new}\notin V$ 则插入 $V$；
5. **查解**：判 $G$ 是否编码解路径（单树平凡、多树昂贵）；
6. **回到第2步**：除非找到解或满足终止条件（报失败）。

$G$ 是拓扑图（顶点=构型，边=路径）。VSM 类比优先队列、LPM 计算可加入图的无碰局部路径段（"局部"=路径段简单/短，**LPM 常失败**是正常的）。

**为何不能简单用高分辨率栅格 + §S2 搜索（L §5.4.1，图5.13）**：高维栅格搜索易困在局部极小（如 $\mathcal C_{obs}$ "碗"，$n$ 维约 $100^n$ 个栅格点会先填满碗）。单树法（≈前向搜索，对"bug trap"困难）；**双向法**（两波前从 $q_I,q_G$ 相遇，覆盖面积小，对 bug trap 好）；多向法（双 bug trap 用，但连接复杂）。

**适配离散搜索（栅格化，L §5.4.2）**：$\mathcal C=[0,1]^n/\!\sim$，分辨率 $k_1,\dots,k_n$，
$$\Delta q_i=[0\cdots0\ 1/k_i\ 0\cdots0]\ (\text{第 }i\text{ 位}),\quad\text{栅格点 }q=\sum_{i=1}^n j_i\Delta q_i\ (j_i\in\{0,\dots,k_i\}).\tag{L 5.35–5.36}$$
**邻域**：1-邻域
$$N_1(q)=\{q\pm\Delta q_i\mid i=1,\dots,n\}\ (\le2n\text{ 个});\tag{L 5.37}$$
2-邻域
$$N_2(q)=\{q\pm\Delta q_i\pm\Delta q_j\mid 1\le i,j\le n,i\ne j\}\cup N_1(q);\tag{L 5.38}$$
$n$-邻域至多 $3^n-1$ 个。得离散规划问题后用 §S2 任一搜索（best-first/A*）；边碰撞在搜索中按需检验（不预构 $\mathcal C_{obs}$）。
**分辨率难题（L §5.4.2）**：①迭代细化（类比迭代加深：每轮每轴 $i$ 个点，$2^n,3^n,4^n,\dots$，弃前功）；②直接对连续问题设计算法（→§P6 RRT）。**union-find** 算法可近 $O(1)$ 维护连通分量，$q_I,q_G$ 同分量即有解（紧密交织采样与搜索）。

## §P5.2 随机势场法（L §5.4.3，扼要）
> 早期采样式方法（曾解31自由度问题），但参数调校繁重，已被 RRT/PRM 取代。三态机（best-first / random-walk / backtrack，L 图5.15）：势函数 $g(q)$（吸引项+排斥项，不要求最优/下界）；best-first 沿 $-\nabla g$ 下降，困住则随机游走逃局部极小（每坐标 $\pm\Delta q_i$ 掷币），$K$ 次（典型 $K=20$）后 backtrack。路径常需平滑（L 式5.39 随机取 $t_1,t_2$ 用直线段 $\tau'(t)=a\tau(t_1)+(1-a)\tau(t_2)$，$a=(t_2-t)/(t_2-t_1)$ 替换并检无碰）。
> **抽取员注**：本书目标章可仅一句带过随机势场为历史方法，重点放 RRT/PRM。

## §P6 RRT / 快速探索稠密树 RDT（L §5.5）

> RRT (Rapidly-exploring Random Tree) / RDT (Rapidly-exploring Dense Tree)：增量采样+搜索，**无参数调校**即有好表现。思想：增量建搜索树，分辨率渐增却不显式设分辨率参数；极限下树稠密覆盖空间。$\alpha$=无限稠密样本序列，$\alpha(i)$=第 $i$ 样本（随机则是 RRT，确定/随机统称 RDT）。

**swath（树已达点集，L 式5.40）**：RDT 是拓扑图 $G(V,E)$，$S\subset\mathcal C_{free}$=$G$ 所有可达点：
$$S=\bigcup_{e\in E}e([0,1]).\tag{L 5.40}$$

### §P6.1 探索算法（无障碍，L 图5.16 SIMPLE RDT）
```
SIMPLE_RDT(q_0)
 1  G.init(q_0);
 2  for i = 1 to k do
 3      G.add_vertex(α(i));
 4      q_n ← nearest(S(G), α(i));
 5      G.add_edge(q_n, α(i));
```
每轮：$\alpha(i)$ 成顶点，连到 swath $S$ 中最近点 $q_n$（沿最短路）。若 $q_n$ 是顶点（L 图5.17）直接连；若 $q_n$ 落在某边内部（L 图5.18）则**裂边**，$q_n$ 成新顶点再连。每轮边数增1或2。**结果树稠密**（每个 $\alpha(i)$ 都入树）。L 图5.19：早期迅速触及远角，后逐渐填充，呈分形（$\alpha$ 均匀随机则为随机分形）。
> **VSM 推广**：RDT 的 `nearest` 充当 VSM，但可从 swath 任意处（含边内）选点，故 VSM 推广为 **swath-point selection method (SSM)**；LPM 沿最短路把 $\alpha(i)$ 连到 $q_n$。

### §P6.2 含障碍的 RDT（L 图5.21）
```
RDT(q_0)
 1  G.init(q_0);
 2  for i = 1 to k do
 3      q_n ← nearest(S, α(i));
 4      q_s ← stopping-configuration(q_n, α(i));
 5      if q_s ≠ q_n then
 6          G.add_vertex(q_s);
 7          G.add_edge(q_n, q_s);
```
`stopping-configuration`（L 图5.20）：沿 $q_n\to\alpha(i)$ 方向，返回**碰到 $\mathcal C_{free}$ 边界前最后可达构型** $q_s$（边可能到不了 $\alpha(i)$）。能多近取决于 §5.3.4 碰撞检测法。若 $q_n$ 已是该方向最近边界则此轮不加点/边。
> **与旧 RRT 步长（L 脚注14）**：原始 RRT [Kuffner-LaValle] 含步长参数 $\epsilon$（即 KF 的 $\eta$）：每次只朝 $\alpha(i)$ 走固定 $\epsilon$ 而非走到 $q_s$；现版消去之。实现上仍可保留步长版（更易写）。

### §P6.3 高效最近点查找（L §5.5.2）
- **精确法**：边为 $\mathbb R^m$ 线段时，把多次裂分的边当**supersegment** 整体处理，用"点到线段距离"原语 $O(1)$，遍历所有 supersegment 取最小。$SO(3)$ 边是 $S^3$ 上圆弧，可用"点到圆弧距离"原语（或映射到 4D 立方面线性插值，仅4个面够，但有失真；映射到 $[0,1]^3/\!\sim$ 失真更大且不合 Haar 测度引入偏置）。
- **近似法 + Kd-树（L §5.5.2，图5.22–5.23）**：沿长边插中间顶点（参数 $\Delta q$，使相邻顶点距 $\le\Delta q$），忽略边内部、只在顶点上找最近——最简单、且恰合 §5.4.1 框架。代价：顶点数剧增。**Kd-树**（多维二叉搜索树推广）：$\mathbb R^2$ 中先按 $x$ 排序取中位点作竖线分两半、再按 $y$ 取中位作横线、交替循环 $n$ 坐标；$k$ 点建树 $O(nk\lg k)$，查最近 $O(\lg k)$（常数随维数指数增），实用约到20维。经验：点数 $>2^n$ 时 Kd-树优于朴素。

### §P6.4 用 RDT 求解查询（L §5.5.3）
- **单树**：从 $q_I$ 长树，周期性试连 $q_G$。RRT 实现：每轮掷偏置币（99/100 取 $\alpha$ 下一样本、1/100 置 $\alpha(i)=q_G$）——偶尔强制朝 $q_G$ 连（1/100 经验值；偏置太强像贪心势场、太弱无动力连目标）。也可让概率密度对 $q_G$ 轻微偏置（难调，慎用）。
- **平衡双向搜索（L §5.5.3，图5.24 RDT_BALANCED_BIDIRECTIONAL）**：两树 $T_a$(从 $q_I$)、$T_b$(从 $q_G$)，性能常远好（尤其逃 bug trap）：
```
RDT_BALANCED_BIDIRECTIONAL(q_I, q_G)
 1   T_a.init(q_I); T_b.init(q_G);
 2   for i = 1 to K do
 3       q_n ← nearest(S_a, α(i));
 4       q_s ← stopping-configuration(q_n, α(i));
 5       if q_s ≠ q_n then
 6           T_a.add_vertex(q_s);
 7           T_a.add_edge(q_n, q_s);
 8           q_n' ← nearest(S_b, q_s);
 9           q_s' ← stopping-configuration(q_n', q_s);
 10          if q_s' ≠ q_n' then
 11              T_b.add_vertex(q_s');
 12              T_b.add_edge(q_n', q_s');
 13          if q_s' = q_s then return SOLUTION;
 14      if |T_b| > |T_a| then SWAP(T_a, T_b);
```
关键：$T_b$ 不用 $\alpha(i)$ 而用 $T_a$ 新顶点 $q_s$ 去试连（$T_b$ 朝 $T_a$ 生长，相遇即解）；第14行**平衡**——总让较小树生长（"小"按顶点数或总边长）。确定性 $\alpha$ 须给两树各自的确定序列以保各自稠密；伪随机则两树可共用同序列。
- **多树（L §5.5.3）**：双 bug trap 等用；连接哪两树、何时连是难题。极限情形=从每个样本起新树并随时连邻近树→覆盖空间且与查询无关，引出下节 PRM。

## §P7 路标图法（多查询）：PRM / 概率路标（L §5.6）

> 多查询：机器人模型与障碍固定、多个 $(q_I,q_G)$ 查询，值得大力**预处理**建路标图 (roadmap)。L 称此族为**采样式路标 (sampling-based roadmaps)**（L 指出框架主由 Kavraki 等以 **PRM, probabilistic roadmaps** 引入；"概率"非本质，故 L 用更中性的名）。

**两阶段（L §5.6.1）**：
- **预处理阶段**：大力建 $G$（路标），应从 $\mathcal C_{free}$ 各处都易达。
- **查询阶段**：给 $(q_I,q_G)$，用局部规划器各自连到 $G$，再用 §S2 任一离散搜索在 $G$ 上找 $q_I\to q_G$ 路径。

### §P7.1 PRM 预处理伪码（L 图5.25 BUILD_ROADMAP）
```
BUILD_ROADMAP
 1  G.init(); i ← 0;
 2  while i < N
 3      if α(i) ∈ C_free then
 4          G.add_vertex(α(i)); i ← i + 1;
 5          for each q ∈ neighborhood(α(i), G)
 6              if ((not G.same_component(α(i), q)) and connect(α(i), q)) then
 7                  G.add_edge(α(i), q);
```
（$\alpha$=均匀稠密序列；$\alpha(i)\in\mathcal C_{obs}$ 则 $i$ 不增，确保 $i$ 正确计顶点数。）`connect`=LPM（通常试 $\alpha(i)$ 与 $q$ 间最短路；实验最优为 §5.3.4 的多分辨率 van der Corput 二分检验）。`same_component` 用 union-find 近 $O(1)$ 检查——确保每次连接都减少连通分量数（若要多解/不同同伦类，把第6行条件 `not same_component` 换成 `G.vertex_degree(q) < K`，如 $K=15$）。

### §P7.2 邻域选择实现（L §5.6.1）
连接前按到 $\alpha(i)$ 距离升序排候选顶点（短路径更易无碰、检验更便宜）。`neighborhood` 几种实现：
1. **Nearest K**：最近 $K$ 个（典型 $K=15$；不确定就用这个）。
2. **Component K**：每连通分量取至多 $K$ 个最近（合理 $K=1$）。
3. **Radius**：半径 $r$ 球内全部，设上限 $K$ 防过多；$r$ 可随点数自适应缩小（按弥散/偏差）。
4. **Visibility**：§P7.4 变体中尝试连 $G$ 所有顶点。

均需 $\mathcal C$ 为度量空间。

### §P7.3 查询阶段与分析（L §5.6.1）
查询：把 $q_I,q_G$ 当作 $\alpha$ 中样本各跑一轮图5.25 连入 $G$，再搜 $q_I\to q_G$ 路径。失败不能断言无解；若已知 $\alpha$ 弥散，至少能断"该分辨率下无解"（即任何解都须穿过比最大空球半径更窄的走廊）。

**$\epsilon$-goodness 分析（L §5.6.1，式5.41，引 Kavraki 等）**：$V(q)$=能用 `connect` 连到 $q$ 的所有构型（"可见"集，L 图5.28a）。$\mathcal C_{free}$ 的 $\epsilon$-goodness：
$$\epsilon(\mathcal C_{free})=\min_{q\in\mathcal C_{free}}\left(\frac{\mu(V(q))}{\mu(\mathcal C_{free})}\right).\tag{L 5.41}$$
表示从任一点可见的 $\mathcal C_{free}$ 最小比例。由 $\epsilon$ 与顶点数可给"找到解的概率"界。难点：$\epsilon$-goodness 极保守（最坏情形）且依赖 $\mathcal C_{free}$ 结构（不可高效算）。L 图5.27 难例：细管，须有样本落在管内多点。

### §P7.4 可见性路标 (Visibility Roadmap)（L §5.6.2）
力求路标小而覆盖好（引 Siméon 等）。两类顶点：
- **Guard（守卫）**：其可见区 $V(q)$ 内无其他 guard。
- **Connector（连接子）**：能见 $\ge2$ 个 guard（存在 $q_1,q_2$ 使 $q\in V(q_1)\cap V(q_2)$）。

构建（`neighborhood` 返回所有顶点）：每新样本 $\alpha(i)$ 三种情形：①连不到任何 guard→自身成 guard 入图；②能连到 $\ge2$ 个不同连通分量的 guard→成 connector 入图（带边）；③只能连到同一分量的 guard→**丢弃**（大幅减顶点数）。缺点：不允许删旧 guard 换更优；依赖样本顺序；仍概率完备/分辨率完备。

### §P7.5 路标改进启发（L §5.6.3）
- **顶点增强 (Vertex enhancement，引 Kavraki 等)**：对难连顶点加力。概率分布 $P(v)$（推荐统计量 $n_f/(n_t+1)$，$n_t$=尝试连接总数、$n_f$=失败数）采样 $v$，从 $v$ 作随机运动得新顶点再试连。
- **边界采样 (L 图5.29a)**：从 $\mathcal C_{obs}$ 中样本沿随机方向二分到 $\mathcal C_{free}$ 中尽量贴 $\mathcal C_{obs}$ 之点。$\tau(0)\in\mathcal C_{obs},\tau(1)\in\mathcal C_{free}$，二分：测 $\tau(1/2)$，据其在 free/obs 决定 $\partial\mathcal C_{free}$ 在哪半段，递归。
- **Gaussian 采样（引 Boor 等）**：$q_1\in\mathcal C$ 均匀；$q_2$ 按以 $q_1$ 为均值的高斯（方差控偏置强度）；若 $q_1,q_2$ 一在 free 一在 obs，则留 free 那个作顶点——偏置样本贴近 $\partial\mathcal C_{free}$。
- **桥测试 (Bridge test，L 图5.29b)**：取沿一直线的三连点找窄走廊（中点在 free、两端在 obs 时留中点）。`\rebuilt`：L 仅图示提及桥测试细节，完整见 Hsu 等原文；综合若深入需回原论文。

## §P8 渐近最优采样式规划：RRT*、RRG、PRM*（源：Karaman-Frazzoli 2011，arXiv:1105.1186）

> **背景**：LaValle 的 PRM/RRT **概率完备但非渐近最优**（解会收敛到次优）。KF 提出 PRM*、RRG、RRT*，证明渐近最优（解代价以概率1收敛到最优 $c^*$），关键是**连接半径随样本数缩放** $\sim(\log n/n)^{1/d}$。
> **抽取可靠性**：KF arXiv PDF 含文本层，`pdftotext` 抽取干净，算法1–6 与定理公式**复现级可靠**。下用 KF 记号（构型 $x\in\mathcal X$，$\mathcal X_{free}$，维数 $d$），转本书时按 §0.1 把 $\mathcal X\to\mathcal C$、$x\to q$、$d=\dim\mathcal C$。

### §P8.1 基元过程（KF §3.1）
- **SampleFree**：返回 $\mathcal X_{free}$ 上 i.i.d. 均匀样本序列（结果可推广到任何密度在 $\mathcal X$ 上有正下界的绝对连续分布）。
- **Nearest(G,x)**：图 $G=(V,E)$ 中按欧氏距离离 $x$ 最近的顶点：
$$\mathrm{Nearest}(G,x)=\arg\min_{v\in V}\|x-v\|.$$
集值版 **kNearest(G,x,k)**：最近 $k$ 个（$|V|<k$ 时返回 $V$）。
- **Near(G,x,r)**：半径 $r$ 球 $B_{x,r}$ 内顶点：
$$\mathrm{Near}(G=(V,E),x,r)=\{v\in V:v\in B_{x,r}\}.$$
- **Steer(x,y)**：返回 $z$ 使 $z$ 在 $\|z-x\|\le\eta$ 约束下最小化 $\|z-y\|$（$\eta>0$ 预定步长）：
$$\mathrm{Steer}(x,y)=\arg\min_{z\in B_{x,\eta}}\|z-y\|.$$
- **CollisionFree(x,x')**：线段 $[x,x']\subset\mathcal X_{free}$ 则 true。
- **Line(x1,x2)**：$x_1\to x_2$ 直线路径 $[0,s]\to\mathcal X$。
- **Parent(v)**：树中 $v$ 的唯一父顶点（根 $v_0$ 取 $\mathrm{Parent}(v_0)=v_0$）。
- **Cost(v)**：根到 $v$ 的唯一树路径代价。可加代价时 $\mathrm{Cost}(v)=\mathrm{Cost}(\mathrm{Parent}(v))+c(\mathrm{Line}(\mathrm{Parent}(v),v))$，$\mathrm{Cost}(v_0)=0$。

输入：规划问题 $(\mathcal X_{free},x_{init},\mathcal X_{goal})$、整数 $n$、代价 $c:\Sigma\to\mathbb R_{\ge0}$。输出：图 $G=(V,E)$，$V\subset\mathcal X_{free}$，$|V|\le n+1$。

### §P8.2 PRM 与 sPRM（KF §3.2，算法1–2）
**算法1 PRM（预处理）**：
```
1  V ← ∅; E ← ∅;
2  for i = 0, ..., n do
3      x_rand ← SampleFree_i;
4      U ← Near(G=(V,E), x_rand, r);
5      V ← V ∪ {x_rand};
6      foreach u ∈ U, in order of increasing ||u − x_rand||, do
7          if x_rand and u are not in the same connected component of G then
8              if CollisionFree(x_rand, u) then E ← E ∪ {(x_rand,u),(u,x_rand)};
9  return G=(V,E);
```
（避免同分量内连接→PRM 路标是森林。）

**算法2 sPRM（简化 PRM，文献可分析版）**：
```
1  V ← {x_init} ∪ {SampleFree_i}_{i=1,...,n}; E ← ∅;
2  foreach v ∈ V do
3      U ← Near(G=(V,E), v, r) \ {v};
4      foreach u ∈ U do
5          if CollisionFree(v, u) then E ← E ∪ {(v,u),(u,v)}
6  return G=(V,E);
```
（与 PRM 差别：允许同分量连接。无障碍时路标是随机 $r$-disc 图。）连接集 $U$ 的实用变体：kNearest、Near∩kNearest 等（KF §3.2）。

### §P8.3 PRM*（KF §3.3，算法4 概念）
**变半径**：$r(n)=\gamma_{\mathrm{PRM}}(\log n/n)^{1/d}$，要求
$$\boxed{\gamma_{\mathrm{PRM}}>\gamma_{\mathrm{PRM}}^*=2\Big(1+\tfrac1d\Big)^{1/d}\Big(\frac{\mu(\mathcal X_{free})}{\zeta_d}\Big)^{1/d}}$$
$d$=维数，$\mu(\mathcal X_{free})$=自由空间 Lebesgue 测度，$\zeta_d=\pi^{d/2}/\Gamma(d/2+1)$=$\mathbb R^d$ 单位球体积。**k-nearest PRM***：$k_{\mathrm{PRM}}(n)=k_{\mathrm{PRM}}\log n$，$k_{\mathrm{PRM}}>k_{\mathrm{PRM}}^*=e(1+1/d)$；$k_{\mathrm{PRM}}=2e$ 对一切问题有效（不依赖具体实例，优于 $\gamma$ 版）。
sPRM 第3行半径换成 $\gamma_{\mathrm{PRM}}(\log n/n)^{1/d}$。

### §P8.4 RRG（Rapidly-exploring Random Graph，KF §3.3，算法5）
```
Algorithm 5: RRG
 1   V ← {x_init}; E ← ∅;
 2   for i = 1, ..., n do
 3       x_rand ← SampleFree_i;
 4       x_nearest ← Nearest(G=(V,E), x_rand);
 5       x_new ← Steer(x_nearest, x_rand);
 6       if ObstacleFree(x_nearest, x_new) then
 7           X_near ← Near(G=(V,E), x_new, min{γ_RRG (log(|V|)/|V|)^{1/d}, η});
 8           V ← V ∪ {x_new}; E ← E ∪ {(x_nearest,x_new),(x_new,x_nearest)};
 9           foreach x_near ∈ X_near do
10               if CollisionFree(x_near, x_new) then E ← E ∪ {(x_near,x_new),(x_new,x_near)}
11   return G=(V,E);
```
**半径**：$r(|V|)=\min\{\gamma_{\mathrm{RRG}}(\log|V|/|V|)^{1/d},\eta\}$（$\eta$=Steer 步长），
$$\boxed{\gamma_{\mathrm{RRG}}>\gamma_{\mathrm{RRG}}^*=2\Big(1+\tfrac1d\Big)^{1/d}\Big(\frac{\mu(\mathcal X_{free})}{\zeta_d}\Big)^{1/d}}$$
（与 $\gamma_{\mathrm{PRM}}^*$ 同式）。k-nearest RRG：$k(|V|)=k_{\mathrm{RRG}}\log|V|$，$k_{\mathrm{RRG}}>k_{\mathrm{RRG}}^*=e(1+1/d)$；$k_{\mathrm{RRG}}=2e$ 通用。RRG 在 RRT 基础上对 $x_{new}$ 额外连接半径内**所有**无碰邻居（成图非树）。

### §P8.5 RRT*（KF §3.3，算法6）—— 核心
RRT* = RRG 去环：删除"非根到顶点最短路一部分"的冗余边，即对 RRT 树**重接线 (rewiring)** 保证每顶点经最小代价路径到达。
```
Algorithm 6: RRT*
 1   V ← {x_init}; E ← ∅;
 2   for i = 1, ..., n do
 3       x_rand ← SampleFree_i;
 4       x_nearest ← Nearest(G=(V,E), x_rand);
 5       x_new ← Steer(x_nearest, x_rand);
 6       if ObstacleFree(x_nearest, x_new) then
 7           X_near ← Near(G=(V,E), x_new, min{γ_RRT* (log(|V|)/|V|)^{1/d}, η});
 8           V ← V ∪ {x_new};
 9           x_min ← x_nearest;  c_min ← Cost(x_nearest) + c(Line(x_nearest, x_new));
10           foreach x_near ∈ X_near do        // —— 沿最小代价路径连接（选最优父）——
11               if CollisionFree(x_near, x_new) ∧ Cost(x_near)+c(Line(x_near,x_new)) < c_min then
12                   x_min ← x_near;  c_min ← Cost(x_near) + c(Line(x_near, x_new))
13           E ← E ∪ {(x_min, x_new)};
14           foreach x_near ∈ X_near do        // —— 重接线（rewire the tree）——
15               if CollisionFree(x_new,x_near) ∧ Cost(x_new)+c(Line(x_new,x_near)) < Cost(x_near)
16               then  x_parent ← Parent(x_near);
17                     E ← (E \ {(x_parent, x_near)}) ∪ {(x_new, x_near)}
18   return G=(V,E);
```
> **抽取员注（伪码行号）**：KF 原算法6 把第16–17行的 `x_parent←Parent(x_near)` 与边替换合并表述（原文行15的 `then` 后接行16的边替换）；上方按可执行逻辑展开为"取旧父→删旧边加新边"，逻辑与原文等价。
**两关键步**：①**选最优父**（行10–13）：在 $X_{near}$ 中选能以最小代价 $c_{min}$ 连到 $x_{new}$ 的顶点 $x_{min}$ 作父；②**重接线**（行14–17）：若经 $x_{new}$ 到某 $x_{near}$ 比其当前路径更优，则改 $x_{near}$ 的父为 $x_{new}$（删旧父边、加新边，维持树结构）。
**半径**：$r(|V|)=\min\{\gamma_{\mathrm{RRT}^*}(\log|V|/|V|)^{1/d},\eta\}$，$\gamma_{\mathrm{RRT}^*}>\gamma_{\mathrm{RRG}}^*$（同式）。**k-nearest RRT***：第7行 $X_{near}\leftarrow$ kNearest$(G,x_{new},k_{\mathrm{RRG}}\log|V|)$。
RRT* 优势：树结构省内存、易扩展到含微分约束问题、**anytime**（随时给可行解且渐近收敛到最优）、计算/内存开销与 RRT 同阶。

### §P8.6 性质与定理（KF §4）
**关键定义（KF §4）**：
- **概率完备**：有解的问题，算法失败概率随样本数 $\to0$。
- **渐近最优 (asymptotic optimality)**：返回解代价以概率1收敛到最优代价 $c^*$。

**主要定理（KF arXiv:1105.1186，编号按该版）**：
- **Theorem 20（变半径 sPRM 以 $r(n)=\gamma n^{-1/d}$ 的不完备性）**：存在常数 $\gamma>0$ 使半径 $r(n)=\gamma n^{-1/d}$ 的变半径 sPRM **不**概率完备。——说明半径衰减太快（$n^{-1/d}$ 而非 $(\log n/n)^{1/d}$）会丧失完备。
- **Theorem 32（同上设定的非最优性）**：半径 $r(n)\le\gamma n^{-1/d}$ 的变半径 sPRM **非**渐近最优。
- **(RRT 非最优，KF 正文/会议版定理)** `\rebuilt`（KF10/期刊版编号）：标准 RRT 概率完备，但其最优路径代价**以概率1收敛到一个严格大于 $c^*$ 的随机值**（即非渐近最优）。直觉：RRT 一旦给某顶点定父就不再改，无法吸收后来更优连接。
- **(PRM*/RRG/RRT* 渐近最优，KF 主结果)** `\rebuilt`：在上述 $\gamma>\gamma^*$（或 $k>k^*$）的连接缩放下，PRM*、RRG、RRT* 均渐近最优——返回解代价以概率1收敛到 $c^*$。
> **抽取员注（定理编号）**：KF 期刊定稿与 arXiv/会议版定理编号不同（如某些版用 Theorem 33/34/38 述 RRT 非最优、PRM*/RRT* 最优）。上方 Theorem 20、32 是从 arXiv:1105.1186 PDF 文本**直接核到的原文编号**（复现级）；RRT 非最优与 PRM*/RRG/RRT* 最优的"主定理"因未逐一核到该 PDF 对应行的编号，标 `\rebuilt`，综合引用时请以 arXiv:1105.1186 定稿编号为准。

**为何 $(\log n/n)^{1/d}$ 是临界缩放（KF §4 直觉）**：
- **随机 $r$-disc 图连通性（Theorem 7，引 Penrose 2003）**：$d$ 维随机 $r$-disc 图 $G^{disc}(n,r)$，
$$\lim_{n\to\infty}\mathbb P\{G^{disc}(n,r)\text{ 连通}\}=\begin{cases}1,&\zeta_d r^d>\log n/n\\0,&\zeta_d r^d<\log n/n\end{cases}$$
$\zeta_d$=$d$ 维单位球体积。即连通的临界半径恰是 $r^d\sim\log n/(n\zeta_d)$——这正是 $\gamma^*$ 公式里 $(\mu(\mathcal X_{free})/\zeta_d)^{1/d}$ 与 $(\log n/n)^{1/d}$ 的来源。半径再小则图不连通（丢完备），故 $(\log n/n)^{1/d}$ 是兼顾连通/最优与计算开销的最小缩放。
- **k-近邻图**：临界 $k\sim\log n$（故 $k(n)=k_{\mathrm{RRG}}\log n$）。

> **抽取员小结（采样式）**：PRM=多查询、建路标、查询时图搜索；RRT/RDT=单查询、增量长树、无参数、双向加速；RRT* /PRM* /RRG=在前者上引入 $(\log n/n)^{1/d}$ 变半径 + 选最优父/重接线，换来渐近最优。RRT*=树+anytime（实践最常用最优规划器之一）。

---

---

# 第四部分　轨迹优化简介 `\rebuilt`

> **说明**：LaValle 2006 第5章不专门讲"轨迹优化 (trajectory optimization)"这一现代分支（CHOMP/STOMP/TrajOpt 等均在2009年后）。本书目标章「规划导论」要求"轨迹优化简介"，本部分据该领域权威源**复建**，整部分标 `\rebuilt`；用于"导论"层级定位，深入需回各原论文。出处随条标注。

## §T1 定位：规划 vs 优化（与前三部分的关系）`\rebuilt`
- 搜索式（A*/D*）与采样式（PRM/RRT*）解决**可行性/全局最优结构**（在复杂 $\mathcal C_{free}$ 中找一条无碰拓扑路径）。
- **轨迹优化**解决**局部精化**：给定一条初始轨迹（可由 RRT* 给出，或简单直线初始化），通过最小化一个**代价泛函**（含平滑度 + 障碍代价 + 约束）把它优化成光滑、低代价、动力学可行的轨迹。
- 二者常**组合**：采样式做全局、优化做局部（如 RRT* 初始化 + CHOMP/TrajOpt 精化）。

## §T2 一般问题形式 `\rebuilt`
轨迹 $\xi:[0,1]\to\mathcal C$（或离散化为 waypoint 序列 $q_0,q_1,\dots,q_T$），最小化
$$\min_{\xi}\ \mathcal U[\xi]=\underbrace{\mathcal U_{smooth}[\xi]}_{\text{平滑/动力学}}+\lambda\,\underbrace{\mathcal U_{obs}[\xi]}_{\text{障碍代价}}\quad\text{s.t. 约束（起终点、关节限、动力学）}.$$
- **平滑项**常取导数平方积分 $\mathcal U_{smooth}=\frac12\int_0^1\|\tfrac{d^k}{dt^k}\xi(t)\|^2dt$（$k=1$ 速度、$k=2$ 加速度/急动度 jerk），离散化为 $\frac12\xi^\top A\xi$（$A$=有限差分算子，半正定）。
- **障碍项**常用到障碍的距离场（signed distance field, SDF）$c(\cdot)$ 构造，使轨迹被推离障碍。

## §T3 代表方法（简介）`\rebuilt`
- **CHOMP**（Covariant Hamiltonian Optimization for Motion Planning，Ratliff/Zucker/Bagnell/Srinivasa, ICRA 2009 / IJRR 2013）：用**协变梯度下降**（用 Riemannian 度量 $A$ 而非欧氏梯度）优化 $\mathcal U_{smooth}+\mathcal U_{obs}$；障碍项基于工作空间 SDF；更新 $\xi\leftarrow\xi-\eta A^{-1}\nabla\mathcal U$。出处：arXiv/IJRR；本书可引为"基于梯度的轨迹优化"代表。
- **STOMP**（Stochastic Trajectory Optimization for Motion Planning，Kalakrishnan 等, ICRA 2011）：**无梯度**，采样一批噪声轨迹按代价加权更新（适合不可微代价/硬约束）；与 CHOMP 互补。
- **TrajOpt**（Schulman 等, RSS 2013 / IJRR 2014）：把轨迹优化写成**序列凸规划 (SQP)**，碰撞约束用凸近似（连续碰撞检测 + 支撑面/距离约束），处理硬约束更稳健。
- **KOMO / 因子图轨迹优化** `\rebuilt`：把轨迹优化表为非线性最小二乘（与本书 SLAM 主线的因子图/高斯牛顿同框架），$\min\sum_k\|f_k(\xi)\|^2$，用 GN/LM 求解——与本书 §（后端优化）天然衔接。

## §T4 与本书主线的接口 `\rebuilt`
- 轨迹优化的"最小二乘 + GN/LM"形式与本书 SLAM 后端**同数学框架**（信息矩阵 $A=J^\top J$、协变更新 $A^{-1}\nabla$），可在本书统一记号下复用；综合时可指出：CHOMP 的 $A$ 即平滑先验的信息矩阵、$A^{-1}\nabla$ 即一步高斯牛顿/自然梯度。
- 本书统一约定（$\mathbf R\in SO(3)$、右扰动、$[\boldsymbol\rho;\boldsymbol\phi]$）下，若轨迹在 $SE(3)$ 上优化，平滑项与梯度须用李群导数（右雅可比 $\mathbf J_r$）——与本书李理论章一致；上述源多在 $\mathbb R^n$ 关节空间表述，转 $SE(3)$ 需补李群梯度（标 `\rebuilt`，依据本书李理论章）。

---

---

# 附录A　全文公式/算法/例/表/定理清单（便于综合核对）

**算法伪码（逐行抽取，共12段）**：
1. FORWARD_SEARCH（L 图2.4）§S2
2. BACKWARD_SEARCH（L 图2.6）§S4.1
3. BIDIRECTIONAL_SEARCH（L 图2.7）§S4.2
4. FORWARD_LABEL_CORRECTING（L 图2.16）§S5.4
5. PROCESS-STATE（St，`\rebuilt`）§S6.4
6. MODIFY-COST（St，`\rebuilt`）§S6.5
7. SIMPLE_RDT（L 图5.16）§P6.1
8. RDT 含障碍（L 图5.21）§P6.2
9. RDT_BALANCED_BIDIRECTIONAL（L 图5.24）§P6.4
10. BUILD_ROADMAP / PRM（L 图5.25）§P7.1
11. PRM / sPRM（KF 算法1–2）§P8.2；RRG（KF 算法5）§P8.4
12. RRT*（KF 算法6）§P8.5

**核心定义（编号/出处）**：拓扑空间/开闭集（L§4.1.1）；内/外/边界/极限点、闭包（L§4.1.1）；流形（L§4.1.2）；同胚（L§4.1.1）；同伦/基本群/群（L§4.1.3）；$SO(2)\cong S^1$、$SO(3)\cong\mathbb{RP}^3$（L§4.2）；$\mathcal C_{obs}$（L4.34）、$\mathcal C_{free}$；Piano Mover's Problem（L Formulation 4.1）；Minkowski 差（L4.37）；离散可行规划（L Formulation 2.1）；定长/变长最优规划（L Formulation 2.2/2.3）；度量空间（L§5.1.1）；弥散（L5.19）、偏差（L5.22）；稠密/分辨率完备/概率完备（L§5）；渐近最优（KF§4）；D* 的 key/RAISE/LOWER（St，`\rebuilt`）。

**关键公式（编号）**：状态转移 L2.1；cost-to-come 增量 §S3.3；A* 优先级 $C^*+\hat G^*$ §S3.4；Bellman 递推 L2.11/L2.18；最优动作 L2.19；$L_p$ L5.3、$L_\infty$ L5.2；笛卡尔积度量 L5.4–5.5；四元数乘法 L4.19；$R(h)$ L4.20；轴角四元数 L4.21；矩阵→四元数 L4.24–4.30；均匀 $SO(3)$ L5.15；弥散下界 L5.20；Halton L5.24–5.25、Hammersley L5.26；弥散-偏差 L5.23；$\epsilon$-goodness L5.41；RRT* 路径平滑（势场）L5.39；$\gamma^*_{\mathrm{PRM/RRG/RRT^*}}=2(1+1/d)^{1/d}(\mu(\mathcal X_{free})/\zeta_d)^{1/d}$、$\zeta_d=\pi^{d/2}/\Gamma(d/2+1)$、$k^*=e(1+1/d)$（KF§3.3）；连通临界 Penrose Theorem 7（KF§4）。

**例题/数值例**：L 例2.1（2D栅格）、2.2（魔方）、2.3（五态定长值迭代，表）、2.4（前向值迭代，表）、2.5（变长，表）；L 例4.1–4.8（拓扑）、4.12（$SO(2)$ 两圆）、4.13（一维 C-障碍）、4.14（三角机器人+矩形障碍）；L 例5.1–5.8（各度量）、5.15（Sukharev 栅格 $n=2,k=9$）。

**表**：L 图2.9/2.12/2.14/2.15（值迭代 cost-to-go/cost-to-come 表，已抄录）；本抽取 §0/§0.1 的源清单表与记号对照表。

**定理/命题**：A* 可采纳性最优（L§2.2.2，引 Hart-Nilsson-Raphael）；Dijkstra 出队即最优（L§2.2.2 归纳）；最优性原理/Bellman（L§2.3.1）；Reif：Piano Mover's PSPACE-hard（L§4.3.1）；Whitney 嵌入（L§4.1.2）；D* 最优性/增量等价（St，`\rebuilt` D*-P1..P4）；Penrose 随机 $r$-disc 连通（KF Theorem 7）；变半径 sPRM 不完备/非最优（KF Theorem 20/32）；RRT 非最优、PRM*/RRG/RRT* 渐近最优（KF，`\rebuilt` 编号待定稿核对）。

---

# 附录B　覆盖度自评与未尽部分

**已全量覆盖（复现级，源含文本层、本地 pdftotext 精读）**：
- C-space（L 第4章）：拓扑基础、流形、$SO(2)/SO(3)/SE(2)/SE(3)$、四元数全套公式、链/树、$\mathcal C_{obs}/\mathcal C_{free}$、Piano Mover's Problem、Minkowski 构造——**全部定义/公式/例题逐条抄录**。
- 搜索式（L 第2章）：前向/后向/双向搜索、BFS/DFS、Dijkstra、A*、best-first、迭代加深/IDA*、值迭代（前向/后向/变长）、标签校正——**全部伪码逐行、全部递推公式、全部例题表格抄录**。
- 采样式（L 第5章 + KF）：完备性概念、度量/测度、弥散/偏差/Halton-Hammersley/van der Corput/Sukharev、碰撞检测、单查询框架、RDT/RRT（含障碍、双向平衡）、Kd-树、PRM/sPRM、可见性路标、改进启发、RRT*/RRG/PRM* 全套伪码与 $\gamma^*/k^*$ 公式、渐近最优理论——**复现级**。

**复建部分（标 `\rebuilt`，源不可逐字提取或源材料本身不含）**：
- **D*（§S6 全节）**：Stentz 1994 原 PDF 为传真扫描位图无文本层，无法 OCR；PROCESS-STATE/MODIFY-COST 伪码与定理按论文文字 + LaValle§12.3.2 转述 + Wikipedia 复建，**逻辑等价但行号/写法可能与原文有出入**；D* Lite 细节仅纲要。逐字原文需回 IEEE/ICRA 1994。
- **KF 主定理编号**：arXiv:1105.1186 核到 Theorem 7/20/32 原文编号；"RRT 非最优"与"PRM*/RRG/RRT* 最优"主定理的精确编号标 `\rebuilt`，以定稿为准。
- **轨迹优化（第四部分全节）**：LaValle 2006 不含此现代分支；据 CHOMP/STOMP/TrajOpt/KOMO 权威源复建为"导论简介"层级；$SE(3)$ 上的李群梯度形式需结合本书李理论章补全。
- **A* 一致性(consistency)**：L 正文未单列，按经典结果补一句（依据 Hart-Nilsson-Raphael 1968 / Russell-Norvig）。
- **C-障碍旋转情形（L§4.3.3）**：按"导论"聚焦从略（纯平移 star algorithm 已全量抄录）；如综合需要可回 L§4.3.3。

**个人笔记同步状态**：未同步，本抽取全部据权威教材/原论文（见 §0 源清单）。

**记号转换要点（再次提醒综合 agent）**：①KF 的 $\mathcal X_{free}\to$ 本书 $\mathcal C_{free}$、构型 $x\to q$、维数 $d=\dim\mathcal C$；②A* 写经典 $g/h/f$（非 L 的 $C/\hat G^*$）；③D* 的 $h(X)$ 是 cost-to-goal（非 A* 启发），建议改称 $g_{\text{goal}}$ 以免撞名；④四元数 L 用 $h=a+bi+cj+dk$ 标量在前，与本书 Hamilton 一致（本书记 $w$）；⑤L 同时用 $q$ 表构型、$h$ 表四元数，勿混。
