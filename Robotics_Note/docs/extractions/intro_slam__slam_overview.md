# 抽取留痕：初识 SLAM —— SLAM 概览（定义·框架·数学表述·分类·工程基础·全书路线图）

> 本文件是项目内部「抽取留痕」（非成书正文）。目标：把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章《初识 SLAM》。
> **铁律·禁摘要·全量保真**：源里每一步推导、每例与数值、每条定义/定理+完整证明、每张表/分类/算法伪码、每段实践代码完整记录。公式 LaTeX 写全、标源小节号。宁长勿略。
>
> **主源（成书骨架）**：高翔《视觉SLAM十四讲》第 2 讲《初识 SLAM》
> `/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/02_初识SLAM.md`（共 720 行，已完整读取，§2.1–§2.4 + 习题）。
>
> **权威联网补强源（注明出处，用于"是什么/为什么/历史/分类/数学/现代系统"的自包含与权威化）**：
> - **[Cadena2016]** C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. Reid, J. J. Leonard, *"Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age,"* IEEE T-RO 32(6):1309–1332, 2016. arXiv:1606.05830。（PDF 全文 pdftotext 抽取，引文标行）
> - **[DW2006]** H. Durrant-Whyte, T. Bailey, *"Simultaneous Localization and Mapping (SLAM): Part I The Essential Algorithms,"* IEEE Robotics & Automation Magazine, June 2006, pp. 99–110。（PDF 全文 pdftotext 抽取，引文标行）
> - **[Handbook-VSLAM]** J. Engel, J. D. Tardós, J. Civera, M. Chli, S. Leutenegger, F. Dellaert, D. Cremers, *"Visual SLAM,"* in *The SLAM Handbook*, ch. 7。本地：`/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/07_Visual_SLAM/07_Visual_SLAM.md`。
> - **[Handbook-FG]** F. Dellaert, M. Kaess, T. Barfoot, *"Factor Graphs for SLAM,"* *The SLAM Handbook*, ch. 1。本地：`/home/ziren2/pengfei/Robotics_Theory/SLAM_Handbook_md/01_Factor_Graphs_for_SLAM/01_Factor_Graphs_for_SLAM.md`。
> - **现代系统原文**：ORB-SLAM（Mur-Artal 2015, T-RO, arXiv:1502.00956）、ORB-SLAM2（2017, T-RO, arXiv:1610.06475）、ORB-SLAM3（Campos 2021, T-RO, arXiv:2007.11898）、VINS-Mono（Qin, Li, Shen 2018, T-RO, arXiv:1708.03852）、LIO-SAM（Shan et al. 2020, IROS, arXiv:2007.00258）。

---

## 0. 记号约定（本源 vs 本书统一约定）

> 本节由抽取专员根据主源 + 权威源用法显式整理，方便综合时统一记号。本章是**入门概览章**，记号大多停留在抽象层面（用一般函数 $f,h$），与本书统一记号差异很小。

| 量 | 主源（十四讲第2讲） | 权威源记号 | 本书统一约定 | 差异/转换说明 |
|---|---|---|---|---|
| 离散时刻 | $t=1,\cdots,K$（位置下标用 $k$） | [DW2006] 用 $k$；[Cadena2016] 用 $k$ | $k$ | 主源正文混用 $t/k$：时刻集合记 $t=1,\dots,K$，但位置/方程下标用 $x_k$。本书统一 $k$。 |
| 机器人位姿/状态 | $\boldsymbol x_k$（平面例：$[x_1,x_2,\theta]_k^\mathrm T$） | [DW2006] $x_k$（vehicle state）；[Cadena2016] $X$（含整条轨迹） | $x_k\in\mathrm{SE}(3)$（或其参数化） | 一致。三维时位姿 $\in\mathrm{SE}(3)$，参数化见第3/4讲。 |
| 路标/地图点 | $\boldsymbol y_j$，$j=1,\dots,N$ | [DW2006] $m_i$（landmark），$m=\{m_1,\dots,m_n\}$；[Cadena2016] 记 $l_i$ | $y_j$（路标）/ $m$（地图） | **记号差异**：主源路标用 $y_j$，DW/Cadena 用 $m_i/l_i$。本书沿用主源 $y_j$，地图整体记 $m$。综合时注明。 |
| 运动输入/控制 | $\boldsymbol u_k$ | [DW2006] $u_k$（control，$k-1$ 时刻施加驱动到 $x_k$） | $u_k$ | 一致。 |
| 观测数据 | $\boldsymbol z_{k,j}$（$k$ 时刻对 $y_j$ 的观测） | [DW2006] $z_k$ 或 $z_{k}^i$；[Cadena2016] $z_k$ | $z_{k,j}$ | 一致（主源带双下标更精确）。 |
| 运动噪声 | $\boldsymbol w_k$ | [DW2006] 加性高斯（隐含）；[Cadena2016] $\epsilon_k$ | $\boldsymbol w_k\sim\mathcal N(0,\Sigma_w)$ | **本书统一**：运动噪声协方差 $\Sigma_w$。主源仅写 $w_k$ 未指定分布；状态估计章设高斯。 |
| 观测噪声 | $\boldsymbol v_{k,j}$ | [Cadena2016] $\epsilon_k$，信息矩阵 $\Omega_k$ | $\boldsymbol v_{k,j}\sim\mathcal N(0,\Sigma_v)$ | **本书统一**：观测噪声协方差 $\Sigma_v$。 |
| 运动方程 | $\boldsymbol x_k=f(\boldsymbol x_{k-1},\boldsymbol u_k,\boldsymbol w_k)$ 式(2.1) | [DW2006] $P(x_k\mid x_{k-1},u_k)$ 式(3)（概率形式） | 同 | 主源为函数形式，DW 为概率密度形式——两者等价（噪声边缘化）。综合时并列呈现。 |
| 观测方程 | $\boldsymbol z_{k,j}=h(\boldsymbol y_j,\boldsymbol x_k,\boldsymbol v_{k,j})$ 式(2.2) | [DW2006] $P(z_k\mid x_k,m)$ 式(2)；[Cadena2016] $z_k=h_k(X_k)+\epsilon_k$ | 同 | 同上。 |
| 观测集合（哪时刻看到哪路标） | $\mathcal O$（"集合，记录在哪个时刻观察到哪个路标"，式(2.5)） | — | $\mathcal O$ | 一致。 |
| 信息矩阵 | （本章未出现，§后端只定性说"方差/不确定性"） | [Cadena2016] $\Omega_k=\Sigma_k^{-1}$ | $\Omega$（信息）/ $\Sigma$（协方差） | 本章定性；MAP 公式由 Cadena 补入，用 $\Omega_k$。 |
| 旋转/四元数 | 本章不涉及（推迟到第3/4讲） | — | $R\in\mathrm{SO}(3)$、Hamilton 四元数、右扰动、$\xi=[\rho;\phi]$ | 本章无差异（概览不展开）。 |
| 加权范数 | （未出现） | [Cadena2016] $\|e\|_\Omega^2=e^\mathrm T\Omega e$ 式(3) 后 | $\|e\|_\Sigma^2=e^\mathrm T\Sigma^{-1}e$（马氏） | 注意 Cadena 用信息矩阵 $\Omega$ 直接加权，本书马氏范数下标常写协方差 $\Sigma$，二者 $\Omega=\Sigma^{-1}$。 |

**术语中英对照（贯穿本章）**：定位 Localization；建图 Mapping；视觉里程计 Visual Odometry (VO)；前端 Front End；后端 Back End；回环检测 Loop Closure Detection（又称闭环检测）；累积漂移 Accumulating Drift；尺度不确定性 Scale Ambiguity；视差 Disparity；基线 Baseline；路标 Landmark；度量地图 Metric Map；拓扑地图 Topological Map；稀疏 Sparse；稠密 Dense；占据栅格 Grid；体素 Voxel；最大后验估计 Maximum-a-Posteriori (MAP)；最大似然估计 Maximum Likelihood Estimation (MLE)；卡尔曼滤波 Kalman Filter (KF)；扩展卡尔曼滤波 Extended Kalman Filter (EKF)；粒子滤波 Particle Filter；图优化 Graph Optimization；因子图 Factor Graph；线性高斯 Linear Gaussian (LG)；非线性非高斯 Non-Linear Non-Gaussian (NLNG)；惯性测量单元 Inertial Measurement Unit (IMU)。

---

## 1. SLAM 是什么、为什么需要它（"是什么/为什么"——主源 §2.1 引子 + [Cadena2016] + [DW2006]）

### 1.1 主源 §2.1：小萝卜的例子（定位与建图的引入）

> 主源用"小萝卜"机器人引入 SLAM。完整保留其论证链。

**设定**：组装一台叫"小萝卜"的机器人（图2-1，左正视图、右侧视图；设备有相机、轮子、笔记本，手是装饰品；笔记本塞进后备箱方便随时调试）。希望它具有**自主运动能力**——这是许多高级功能（扫地、搬东西）的前提。

**论证链（主源原文要点）**：
- 要移动 → 需轮子和电机（足式步态复杂，故装轮子）。
- 有了轮子若不加规划和控制，机器人不知行动目标，会四处乱走甚至撞墙损毁。
- 要规划和控制 → 首先需**感知周边环境** → 在脑袋上装相机。
- 机器人和人类相似：有眼睛、大脑和四肢的人能在任意环境轻松行走探索。

**为探索一个房间，机器人至少需要知道两件事（主源原文）**：
1. **我在什么地方？——定位。**
2. **周围环境是什么样？——建图。**

> 主源原文："'定位'和'建图'，可以看成感知的'内外之分'。作为一个'内外兼修'的小萝卜，一方面要明白自身的状态（即位置），另一方面也要了解外在的环境（即地图）。"

### 1.2 主源 §2.1：传感器两分类（为什么用携带式传感器 → 为什么需要 SLAM）

解决定位的方法很多：地板铺导引线、墙上贴二维码、桌上放无线电定位设备（很多仓储物流机器人的做法）、室外装 GPS 接收器。主源把传感器（图2-2）分为**两类**：

| 传感器类别 | 例子 | 特点 | 局限 |
|---|---|---|---|
| **安装于环境中**（外部） | 导轨、二维码标志、（房间里的）无线电定位、GPS | 通常能**直接测量机器人位置**，简单有效解决定位 | **要求环境必须由人工布置**，限制使用范围（室内无 GPS、园区无法铺导轨） |
| **携带于机器人本体上**（内部） | 轮式编码器、相机、激光传感器、IMU | **不对环境提任何要求**，适用于未知环境 | 测量的是**间接物理量**（编码器测轮转角度、IMU 测角速度/加速度、相机/激光读外部环境观测），只能间接推算位置 |

图2-2 子图：(a) 用二维码定位的增强现实软件；(b) GPS 定位装置；(c) 铺设导轨的小车；(d) 激光雷达；(e) IMU 单元；(f) 双目相机。

**关键结论（主源原文）**：环境类传感器"约束了外部环境……虽然简单可靠，但无法提供普遍的、通用的解决方案"。携带式传感器"没有对环境提出任何要求，从而使得这种定位方案可适用于未知环境"。

> **SLAM 中强调未知环境**：主源原文——"我们在 SLAM 中非常强调未知环境。所以理论上，我们不限制小萝卜的使用环境，这意味着我们没法假设像 GPS 或导轨这样的外部传感器都能顺利工作。因此，使用携带式的传感器来完成 SLAM 是我们重点关心的问题。"
> - **视觉 SLAM**：传感器主要是**相机**，指如何用相机解决定位和建图问题（本书主题）。
> - **激光 SLAM**：传感器主要是**激光**，相对成熟（2005《概率机器人》[5] 介绍许多激光 SLAM 知识，ROS 里有许多现成软件）。

### 1.3 权威定义（[Cadena2016] + [DW2006]，用于成书的权威化）

> 本书《初识 SLAM》宜在小萝卜直觉之后给出一句**权威定义**。以下逐字引（≤125 字符片段）。

- **[Cadena2016] 引言定义（cadena.txt L89–97）**："SLAM comprises the simultaneous estimation of the state of a robot equipped with on-board sensors, and the construction of a model (the map) of the environment that the sensors are perceiving." 状态在简单情形即**位姿（position and orientation）**，也可含速度、传感器偏置、标定参数；地图是对环境中关心方面（路标、障碍）的表示。
- **[Cadena2016] 摘要定义**："Simultaneous Localization and Mapping (SLAM) consists in the concurrent construction of a model of the environment (the map), and the estimation of the state of the robot moving within it."
- **[DW2006] 定义（dw_tutorial.txt L153–159）**："SLAM is a process by which a mobile robot can build a map of an environment and at the same time use this map to deduce its location. In SLAM, both the trajectory of the platform and the location of all landmarks are estimated **online without the need for any a priori knowledge of location**."
- **[Handbook-VSLAM] 视觉 SLAM 定义（07_Visual_SLAM.md L31）**："Visual SLAM is a computational technique that enables a system to simultaneously localize itself within an unknown environment and build a map of that environment in real time."

### 1.4 为什么需要地图 / 为什么需要 SLAM（[Cadena2016] cadena.txt L98–122, L166–250）

**用地图的两个理由（[Cadena2016] L98–108）**：
1. 地图常用于支持其他任务（路径规划、给操作员直观可视化）。
2. **地图限制状态估计误差**："In the absence of a map, dead-reckoning would quickly drift over time; on the other hand, using a map ... the robot can 'reset' its localization error by re-visiting known areas (so-called **loop closure**)."

**何时不需要 SLAM（[Cadena2016] L109–116）**：若一组路标位置**先验已知**（工厂地面人工布设信标地图、或机器人有 GPS——可把 GPS 卫星看作已知位置的移动信标），且能可靠地相对已知路标定位，则可能不需要 SLAM。SLAM 的流行与**室内移动机器人应用**的兴起相关：室内排除 GPS，SLAM 给出无须人工定位基础设施即可运行的方案（L117–122）。

**"自主机器人真的需要 SLAM 吗？"（[Cadena2016] L166–250，三重回答）**：
> 关键词是 **loop closure（回环）**："SLAM aims at building a globally consistent representation of the environment, leveraging both ego-motion measurements and loop closures. The keyword here is 'loop closure': **if we sacrifice loop closures, SLAM reduces to odometry.**"（L168–171）
1. **SLAM 研究本身产出了当前最先进的视觉-惯性里程计**（如 [163,175]）；从这个意义上 **Visual-Inertial Navigation (VIN) 就是 SLAM**——VIN 可看作禁用了回环/地点识别模块的简化 SLAM（L182–187）。现代里程计漂移已很小（视觉-惯性 < 0.5% 轨迹长度，L177–179）。
2. **环境的真实拓扑**：只做里程计而忽略回环，机器人把世界理解成"无限长走廊"（图1 左，从起点 A 到终点 B 不断探索新区域）；回环事件告诉机器人这条"走廊"自相交（图1 右），从而**发现地图中的捷径**（如 B、C 两点）。为什么不干脆丢掉度量信息只做地点识别？因为**度量信息使地点识别更简单更鲁棒**，度量重建能预测回环机会并丢弃虚假回环——SLAM 是抵御**错误数据关联与感知混叠（perceptual aliasing，外观相似但实为不同地点）**的天然防线（L221–241）。
3. 许多应用（军事/民用探索并向操作员汇报地图、结构巡检桥梁/建筑）**显式或隐式要求全局一致地图**（L242–250）。

**"SLAM 解决了吗？"（[Cadena2016] L251–294）**：此问只对给定的 **机器人/环境/性能** 三元组才良定义。评判需指定：robot（运动类型、可用传感器、算力）、environment（平面/三维、自然/人工路标、动态元素、对称性与感知混叠风险）、performance（精度、地图类型、成功率、延迟、运行时长、地图规模）。例如：2D 室内 + 轮式编码器 + 激光、精度 < 10cm、低失败率 → **基本已解决**（工业例：Kuka Navigation Solution）。慢速视觉 SLAM（火星车、家用机器人）与视觉-惯性里程计 → **成熟研究领域**。而高速动态、高度动态环境、高频闭环控制需求 → 仍需大量基础研究。

### 1.5 SLAM 的"鸡生蛋"本质与关键洞见（[DW2006] dw_tutorial.txt L232–373）

> 这是《初识 SLAM》"为什么定位与建图互相依赖"必须讲清的核心，[DW2006] 给出**最经典、最权威**的表述。逐字+逐式保真。

**两个退化子问题（L232–239）**：
- **纯建图问题** = 计算条件密度 $P(m\mid X_{0:k},Z_{0:k},U_{0:k})$，**假设车辆位置 $x_k$ 在所有时刻已知（或确定）**，由不同位置的观测融合出地图 $m$。
- **纯定位问题** = 计算 $P(x_k\mid Z_{0:k},U_{0:k},m)$，**假设路标位置确定已知**，目标是估计车辆相对这些路标的位置。

→ 二者各自假设了对方已知，但在 SLAM 中**两者都未知**——这正是"鸡生蛋"（chicken-and-egg）：要定位需要地图，要建图需要位姿。

**联合后验不可天真分解（L295–303，关键式）**：观测模型 $P(z_k\mid x_k,m)$ 同时依赖车辆和路标位置，因此联合后验**不能**按下式分解：
$$
P(x_k, m\mid z_k) = P(x_k\mid z_k)\,P(m\mid z_k). \tag{DW-partition}
$$
> "indeed it is well known from the early papers on consistent mapping [17],[39] that **a partition such as this leads to inconsistent estimates**."（强行如此分解会得到不一致估计）

**误差高度相关（L304–317）**：估计与真实路标位置之间的误差**在路标之间是共同的**，源于**单一来源——观测路标时机器人在哪里的认知误差**。因此路标位置估计**高度相关**：任意两路标的**相对位置 $m_i-m_j$ 可被高精度获知**，即使单个路标的**绝对位置 $m_i$ 很不确定**（概率上：联合密度 $P(m_i,m_j)$ 很尖锐，而边缘密度 $P(m_i)$ 很弥散）。

**最重要的洞见——相关性单调增、地图单调收敛（L318–325，"定理"级结论）**：
> "**The most important insight in SLAM was to realize that the correlations between landmark estimates increase monotonically as more and more observations are made.**"（这些结果**仅在线性高斯情形被证明** [14]；一般概率情形的正式证明仍是 open problem。）
- 实践含义：**路标相对位置的认知总在改善、永不发散，与机器人运动无关**。
- 概率含义：所有路标的联合密度 $P(m)$ 随观测增多**单调变尖锐**。

**收敛机制（L326–373）——弹簧网络类比（图2 Spring network analogy）**：观测可视为对路标间相对位置的"近独立"测量。机器人在 $x_k$ 同时观测 $m_i,m_j$，相对位置与车辆坐标系无关 → 独立测量相对关系。机器人移到 $x_{k+1}$ 再观测 $m_j$，更新它相对 $x_k$ 的位置 → 又因 $m_i,m_j$ 高度相关而**传播回更新 $m_i$（即使 $m_i$ 在新位置看不见）**。所有路标最终连成由相对位置/相关性连接的网络，每次观测都增其精度。**弹簧/橡皮膜类比**：路标由弹簧相连（弹簧刚度=相关性），机器人来回穿行使弹簧（单调）变硬，极限下得到**刚性路标地图 / 精确相对地图**；此时机器人相对地图的定位精度**仅受地图与相对测量传感器质量限制**，理论极限下等于"给定该地图所能达到的定位精度"。

> **抽取专员注（综合提示）**：上面的"误差相关 / 单调收敛 / 弹簧类比"是 EKF-SLAM 时代的核心理论，应作为《初识 SLAM》"为何鸡生蛋问题可解"的权威支撑。注意正式收敛性证明只在**线性高斯**下成立（[14]=Dissanayake et al. 2001 的经典结果）。

### 1.6 SLAM 简史（三个时代 + 关键年代；[Cadena2016] L123–146, L295–314 + [DW2006] L53–146 + 联网核验）

> 主源第2讲未系统讲历史（仅 §2.2 提一句"现在称为后端优化的部分，在很长一段时间内直接被称为'SLAM研究'……最早提出 SLAM 的论文称它为'空间状态不确定性的估计'(Spatial Uncertainty)[4,11]"）。下面用 [Cadena2016]/[DW2006] 补全权威简史。

**起源（[DW2006] L53–117）**：
- 概率 SLAM 问题的**起源 = 1986 年旧金山 IEEE 机器人与自动化会议（ICRA）**。当时概率方法刚被引入机器人与 AI。Peter Cheeseman、Jim Crowley、Hugh Durrant-Whyte 等在会上（"napkins/table cloths"上）讨论一致性建图；Raja Chatila、Olivier Faugeras、Randal Smith 等亦有贡献。
- 随后几年关键论文：**Smith & Cheeseman [39]**（"On the representation and estimation of spatial uncertainty", IJRR 1986）与 **Durrant-Whyte [17]** 建立了描述路标间关系、操纵几何不确定性的**统计基础**，并指出不同路标位置估计之间必有**高度相关**、且相关性随观测增长。
- **Smith, Self & Cheeseman [40]**（landmark 论文）证明：移动机器人取相对路标观测时，所有路标估计因**车辆位置的共同误差**而必然相互相关 → 一致的完整解需要一个由**车辆位姿 + 每个路标位置**组成的联合状态，每次观测后整体更新，状态向量巨大、计算量随路标数**平方**增长。
- **概念突破 + 命名（[DW2006] L107–116）**："**the convergence result and the coining of the acronym SLAM was first presented in a mobile robotics survey paper presented at the 1995 International Symposium on Robotics Research [16].**" 即 **1995 年 ISRR**（Durrant-Whyte, Rye, Nebot 等）首次系统提出 SLAM 结构、收敛性结论并**提出 "SLAM" 缩写**（彼时也称 concurrent mapping and localization, CML）。收敛性基础理论由 **Csorba [10,11]** 发展。MIT、Zaragoza、Sydney ACFR 等组开始投入。1999 ISRR'99 首次 SLAM 专题；2002 ICRA SLAM workshop 吸引 150 人；2002 KTH SLAM summer school。

**三个时代（[Cadena2016] L123–146, L295–314）**——逐字保真：

| 时代 | 年代 | 特征（[Cadena2016] 原文要点） |
|---|---|---|
| **古典时代 Classical age** | **1986–2004** | "saw the introduction of the main probabilistic formulations for SLAM, including approaches based on **Extended Kalman Filters, Rao-Blackwellised Particle Filters, and maximum likelihood estimation**; moreover, it delineated the basic challenges connected to efficiency and robust data association." 主要被 [DW2006] 两篇 tutorial 覆盖；另见 Thrun-Burgard-Fox《Probabilistic Robotics》[240]。 |
| **算法分析时代 Algorithmic-analysis age** | **2004–2015** | "saw the study of fundamental properties of SLAM, including **observability, convergence, and consistency**. In this period, the key role of **sparsity** towards efficient SLAM solvers was also understood, and the main **open-source SLAM libraries** were developed." 部分被 Dissanayake et al. [64] 覆盖。 |
| **鲁棒感知时代 Robust-perception age** | **2015–至今** | 本文提出"我们正进入第三个时代"，关键要求：(1) **robust performance**（低失败率、长时段、广环境、含 fail-safe 与自调参，对抗"manual tuning 的诅咒"）；(2) **high-level understanding**（超越基础几何，理解高层几何/语义/物理/可供性 affordances）；(3) 资源感知（resource awareness）；(4) 任务驱动的感知（task-driven perception）。 |

> **SLAM 综述一览表（[Cadena2016] Table I, cadena.txt L189–217）**：2006 概率方法与数据关联（Durrant-Whyte & Bailey [7,69]）；2008 滤波方法（Aulinas et al. [6]）；2011 SLAM 后端（Grisetti et al. [97]）；2011 可观性/一致性/收敛（Dissanayake et al. [64]）；2012 视觉里程计（Scaramuzza & Fraundorfer [85,218]）；2016 多机器人 SLAM（Saeedi et al. [216]）；2016 视觉地点识别（Lowry et al. [160]）；2016 Handbook of Robotics ch.46（Stachniss et al. [234]）；2016 理论方面（Huang & Dissanayake [109]）。

---

## 2. 各类相机（主源 §2.1 续：单目/双目/RGB-D）

> 主源在 §2.1 末尾详述视觉 SLAM 用的相机三大类。全量保留（含每条特点与权衡）。

**相机与单反不同**：SLAM 用的相机更简单，通常不带昂贵镜头，以一定速率拍摄环境形成连续视频流（普通摄像头约 30 fps，高速相机更快）。按工作方式分**单目 Monocular / 双目 Stereo / 深度 RGB-D** 三大类（图2-3）。此外还有**全景相机[7]、Event 相机[8]** 等特殊/新兴种类（尚未成主流）。

### 2.1 单目相机（Monocular）

- 只用一个摄像头做 SLAM = **单目 SLAM**。结构特别简单、成本特别低。数据=照片。
- 照片本质：拍摄某场景（Scene）在相机成像平面上的**投影**，以二维记录三维世界 → **丢掉了一个维度，即深度（距离）**。
- **无法从单张图片计算物体与相机的距离**（图2-4：手掌上的人是真人还是模型，无法仅凭单图判断）。换言之，单张图像里**无法确定物体真实大小**——可能是"大而远"或"小而近"，因近大远小透视而在图像中同样大小。
- **恢复三维结构必须改变视角（移动相机）**：移动相机才能估计**运动（Motion）**，同时估计场景物体远近和大小即**结构（Structure）**。机制：相机移动时物体在图像上的运动形成**视差（Disparity）**——近处物体移动快、远处慢、极远处（太阳月亮）看上去不动；通过视差定量判断远近。
- **尺度不确定性（Scale Ambiguity）**：即便知道远近也只是相对值。把相机运动和场景大小**同时放大任意倍**，单目相机看到的像是一样的 → 单目 SLAM 估计的轨迹和地图与真实相差一个**尺度（Scale）因子**，且**无法仅凭图像确定真实尺度**。
- **两大麻烦**：必须平移后才能算深度；无法确定真实尺度。根本原因：单张图像无法确定深度 → 引出双目/深度相机。

### 2.2 双目相机（Stereo）与深度相机（RGB-D）

**双目相机**：
- 由两个单目相机组成，两相机间距离=**基线（Baseline）已知**。通过基线估计每个像素的空间位置（类比人眼左右图像差异判断远近）。可拓展为**多目相机**（本质相同）。
- 计算机双目需**大量计算**才能（不太可靠地）估计每像素深度。测量深度范围与基线相关：**基线越大、能测越远**（无人车双目通常很大）。
- 优点：不依赖其他传感设备，**室内外皆可**。
- 缺点：配置与标定复杂；深度量程和精度受基线与分辨率所限；视差计算**极耗算力**，需 GPU/FPGA 加速才能实时输出整张图距离 → **计算量是双目主要问题之一**。

**深度相机（RGB-D，2010 年前后兴起）**：
- 最大特点：通过**红外结构光**或 **Time-of-Flight（ToF）** 原理，像激光那样**主动发射光并接收返回光**测距，是**物理测量手段**而非软件计算 → 相比双目**节省大量算力**。
- 常用型号：Kinect/Kinect V2、Xtion Pro Live、RealSense 等（手机也用于人脸识别）。
- 缺点：测量范围窄、噪声大、视野小、易受日光干扰、无法测透射材质 → **主要用于室内，室外较难**。

> **视觉 SLAM 的目标（主源 §2.1 末）**："想象相机在场景中运动……得到一系列连续变化的图像。视觉 SLAM 的目标，是通过这样的一些图像，进行定位和地图构建。" 这不是单一算法（输入数据就吐定位+地图），而需要**一套完善的算法框架**——经研究者长期努力已较成熟。

---

## 3. 经典视觉 SLAM 框架（"五模块/职责/输入输出/数据流"——主源 §2.2，配框架图）

> 这是本章最核心的工程图景。主源 §2.2 给出**五模块**结构（图2-7），并在 §2.2.1–§2.2.4 逐一展开。全量保真，并用 [Cadena2016]/[Handbook-VSLAM] 补充权威的"前端/后端"二分视角。

### 3.1 整体流程（主源 §2.2，图2-7 经典视觉 SLAM 框架）

整个视觉 SLAM 流程包括以下步骤（主源原文逐条）：

1. **传感器信息读取（Sensor data）**。视觉 SLAM 中主要为相机图像信息的读取和预处理；机器人中还可能有码盘、惯性传感器等信息的读取与同步。
2. **前端视觉里程计（Visual Odometry, VO）**。任务是估算**相邻图像间相机的运动**，以及局部地图的样子。VO 又称**前端（Front End）**。
3. **后端（非线性）优化（Optimization）**。接受不同时刻 VO 测量的相机位姿，以及回环检测的信息，对它们进行优化，得到**全局一致的轨迹和地图**。因接在 VO 之后，又称**后端（Back End）**。
4. **回环检测（Loop Closure Detection）**。判断机器人**是否到达过先前的位置**。若检测到回环，把信息提供给后端处理。
5. **建图（Mapping）**。根据估计的轨迹，建立**与任务要求对应的地图**。

> **框架成熟度声明（主源原文）**："经典的视觉 SLAM 框架是过去十几年的研究成果……如果把工作环境限定在**静态、刚体、光照变化不明显、没有人为干扰**的场景，那么这种场景下的 SLAM 技术已经相当成熟[9]。"

**【框架图说明（供综合 agent 绘制 TikZ；主源图2-7 数据流）】**：

```
传感器数据          前端 VO              后端优化            建图
(Sensor data) ──▶ (Visual Odometry)──▶ (Optimization) ──▶ (Mapping)
 相机图像/码盘/IMU   估相邻帧间相机运动     接收VO位姿+回环信息    据轨迹建任务对应地图
 读取与预处理        +局部地图/局部结构     输出全局一致轨迹+地图
                         │                    ▲
                         │                    │ 回环约束(A与B同一点)
                         ▼                    │
                    回环检测 (Loop Closure Detection)
                    判断是否回到先前位置 → 提供回环信息给后端
```
数据流要点：
- 传感器数据 → VO（逐帧/相邻帧）；
- VO 输出**相机位姿初值 + 局部地图/局部结构**给后端，同时图像流送回环检测；
- 回环检测把"A 与 B 是同一个点"的**回环约束**送后端；
- 后端融合 VO 位姿 + 回环约束做**全局优化** → 输出全局一致轨迹 + 地图；
- 建图据优化后的轨迹生成最终地图。
- （[Cadena2016] 强调：**后端会反馈信息给前端**，支持回环检测与验证——见 §3.6。）

### 3.2 §2.2.1 视觉里程计 VO（职责/输入输出/累积漂移）

- **关心相邻图像之间的相机运动**，最简单是两张图像之间的运动关系（图2-8：右图是左图向左旋转一定角度的结果；人能定性判断"向左旋转"，但难给出精确度数/厘米）。
- 计算机困境：图像在计算机里只是**数值矩阵**，矩阵里表达什么计算机毫无概念。视觉 SLAM 中只能看到一个个像素，知道它们是空间点在成像平面上投影的结果 → 为定量估计相机运动，**必须先了解相机与空间点的几何关系**（铺垫见第5讲相机模型、第7/8讲 VO 实现）。
- **VO 能力**：通过相邻帧间图像**估计相机运动**并**恢复场景空间结构**。称"里程计"因它只计算相邻时刻运动、与过去信息无关（像只有短时记忆的物种；可不限两帧，数量可更多，如 5~10 帧）。
- **VO 的输入/输出**：
  - 输入：相邻（或近邻 5~10）帧图像。
  - 输出：相邻时刻相机运动（位姿增量）+ 局部场景结构。
- **串起轨迹解决定位**：把相邻时刻运动"串"起来构成机器人运动轨迹（定位）；据每时刻相机位置算各像素对应空间点位置 → 得地图。
- **致命缺陷——累积漂移（Accumulating Drift）**：VO 最简单情况只估两帧间运动，每次估计带误差，**先前时刻误差传递到下一时刻** → 一段时间后轨迹不再准确（图2-9）。例：先左转 90° 再右转 90°，若第一个 90° 估成 89°，右转后估计位置回不到原点，且之后即便完全准确也都带这 $-1°$ 误差。
- **漂移（Drift）后果**：无法建立一致地图（直走廊变斜、90° 直角不再是直角）。
- **解决漂移需两种技术**：**后端优化** + **回环检测**。回环检测负责"机器人回到原始位置"的检测，后端优化据此**校正整个轨迹形状**。

### 3.3 §2.2.2 后端优化（职责：噪声处理 / 状态估计 / MAP）

- **后端优化主要指处理 SLAM 过程中的噪声问题**。再精确的传感器也带噪声（便宜的误差大、贵的小、有的受磁场温度影响）。
- 除"如何从图像估计相机运动"，还要关心：这个估计**带多大噪声**、噪声**如何从上一时刻传递到下一时刻**、对当前估计**有多大自信**。
- **后端要解决的问题**：如何从带噪数据中**估计整个系统的状态**及该估计的**不确定性**——这称为**最大后验概率估计（Maximum-a-Posteriori, MAP）**。状态**既包括机器人轨迹，也包含地图**。
- **前端/后端分工（主源原文）**：前端给后端提供**待优化的数据及其初始值**；后端负责整体优化，**往往只面对数据、不必关心数据来自什么传感器**。视觉 SLAM 中前端与计算机视觉更相关（特征提取与匹配），后端主要是**滤波与非线性优化算法**。
- **历史意义（主源原文）**："现在我们称为后端优化的部分，在很长一段时间内直接被称为'SLAM研究'。早期的 SLAM 问题是一个**状态估计问题**……最早提出 SLAM 的一系列论文中，当时的人们称它为'空间状态不确定性的估计'（Spatial Uncertainty）[4,11]……反映出 SLAM 问题的本质：对运动主体自身和周围环境**空间不确定性的估计**。" 为解决 SLAM 需**状态估计理论**，用**滤波器或非线性优化**估计状态的均值和不确定性（方差）。（内容见第6/9/10讲。）

### 3.4 §2.2.3 回环检测（职责：消除漂移 / 图像相似性）

- **又称闭环检测，主要解决位置估计随时间漂移的问题**。
- 思路：机器人运动一段时间后回到原点，但因漂移位置估计没回原点 → 若有手段让机器人知道"回到了原点"或识别出"原点"，再把位置估计"拉"过去，即可消除漂移。
- **与定位、建图都密切相关**："地图存在的主要意义是让机器人知晓自己到过的地方。" 实现回环检测需让机器人**识别到过的场景**。
- 实现手段：可在机器人下方设标志物（如二维码图片），看到即知回原点——但这是**环境中的传感器**，对环境做了限制。更希望用**携带的传感器（图像本身）**：判断**图像间相似性**完成回环检测（类比人看到两张相似图片易辨认来自同一地方）。**回环检测成功可显著减小累积误差**。→ 视觉回环检测实质是**计算图像数据相似性的算法**；图像信息丰富使正确检测回环的难度降低。
- **数据流**：检测到回环后，把"**A 与 B 是同一个点**"的信息告诉后端优化；后端据此把轨迹和地图调整到符合回环结果的样子 → 充分而正确的回环检测可**消除累积误差、得到全局一致的轨迹和地图**。

### 3.5 §2.2.4 建图（职责：按应用建地图 / 度量 vs 拓扑）

- **建图 = 构建地图的过程**。地图（图2-10）是对环境的描述，但**描述不固定，视 SLAM 应用而定**。
- 不同需求：扫地机器人（低矮平面）只需**二维地图**标记可通过/障碍；相机 6 自由度运动至少需**三维地图**；有时要带纹理三角面片的漂亮重建；有时只需"A 到 B 可通过、B 到 C 不行"；甚至不需要地图（或地图由他人提供，如行驶车辆用已绘当地地图）。
- 因此**建图没有固定形式和算法**（不像 VO/后端/回环）。一组空间点、漂亮 3D 模型、标着城市村庄铁路河道的图片都可叫地图。大体分**度量地图**与**拓扑地图**两种。图2-10 示例：2D 栅格地图、2D 拓扑地图、3D 点云地图、3D 网格地图。

**度量地图（Metric Map）**：
- 强调**精确表示物体位置关系**，常用**稀疏（Sparse）/稠密（Dense）**分类。
- **稀疏地图**：做了一定抽象，不需表达所有物体；选一部分有代表意义的东西=**路标（Landmark）**，稀疏地图由路标组成（非路标部分忽略）。**定位用稀疏路标地图就足够**。
- **稠密地图**：着重建模所有看到的东西。**导航往往需稠密地图**（否则撞上两路标间的墙）。稠密地图按某分辨率由许多小块组成：二维=许多**小格子（Grid）**，三维=许多**小方块（Voxel）**。每个小块通常含**占据、空闲、未知**三种状态，查询某空间位置可知是否可通过 → 用于 A*、D* 等导航算法。
- 稠密度量地图缺点：(1) 需存每个格点状态，**耗大量存储**，多数细节无用；(2) 大规模时有**一致性问题**——很小的转向误差可能导致两间屋子的墙重叠使地图失效。

**拓扑地图（Topological Map）**：
- 相比度量地图的精确性，**更强调地图元素之间的关系**。是一个**图（Graph），由节点和边组成**，只考虑节点间**连通性**（只关注 A、B 连通，不考虑如何从 A 到 B）。
- 放松了对精确位置的需要、去掉细节，是**更紧凑的表达**。
- 缺点：不擅长表达复杂结构的地图；如何分割形成节点与边、如何用拓扑地图导航与路径规划仍是待研究问题。

> **抽取专员注（前瞻）**：建图详见第12讲《建图》（稠密重建、八叉树 OctoMap、TSDF 等）。本章只给度量/拓扑、稀疏/稠密的入门分类。

### 3.6 权威补强：前端/后端二分与数据流（[Cadena2016] cadena.txt L350–599 + [Handbook-VSLAM]）

> 主源把流程拆成五模块；现代综述则统一为**前端(front-end)/后端(back-end)二分**。两种视角等价：VO+回环检测 ⊂ 前端，优化 ⊂ 后端，建图依赖后端输出。给出权威表述以便成书自洽。

**[Cadena2016] 标准架构（L350–355, 图2 caption L311–312）**：
> "...two components: the front-end and the back-end. The front-end abstracts sensor data into models that are amenable for estimation, while the back-end performs inference on the abstracted data produced by the front-end."
> 图2 caption："Front-end and back-end in a typical SLAM system. **The back-end can provide feedback to the front-end for loop closure detection and verification.**"

**前端职责（[Cadena2016] L559–599）**：原始传感器数据常难直接写成状态的解析函数（如图像像素强度、单束激光），故在后端前设**前端**：(1) **特征提取**（视觉中提取少量可区分点的像素位置，使后端易建模）；(2) **数据关联（data association）**——把每个观测 $z_k$ 关联到一个路标子集 $X_k$ 使 $z_k=h_k(X_k)+\epsilon_k$；(3) 提供**非线性优化(4)式的初值**（如特征法单目中靠多视图三角化初始化路标）。数据关联又分：
- **短期数据关联（short-term）**：关联**连续测量**中对应特征（如跟踪连续两帧的同一 3D 点的两个像素）。
- **长期数据关联（long-term，即回环 loop closure）**：把**新测量关联到更早的路标**。
- 后端通常**反馈信息给前端**支持回环检测与验证。

**[Handbook-VSLAM] 视觉 SLAM 流水线三大子功能（07_Visual_SLAM.md L35–55）**：现代完整视觉 SLAM 典型含三个互补子功能：**里程计前端（odometry front-end）+ 建图后端（mapping back-end）+ 回环与重定位（loop closure and re-localization）**。
- **前端 VO（L37–41）**：估连续相机帧间相对运动，给位姿提供初值。两条路线：(i) **特征法（feature-based）** 三阶段=检测/提取特征点 → 跨图像求对应 → 最小化重投影误差求相对运动+3D 点（末阶段类似经典 BA）；(ii) **直接法（direct）** 一步到位，直接对**光度损失**关于相机运动+结构优化（与光流、photometric BA 相关）。特征法因显式数据关联有**更大收敛域**；直接法常用 coarse-to-fine（先对齐降采样图像）。
- **后端（L43–45）**：用 BA 或位姿图优化（pose-graph optimization）全局优化轨迹与地图，精化前端估计、整合（含地点识别）观测 → 长期一致、减小长程畸变。
- **回环与重定位（L49）**：VO 因相机跟踪误差累积而漂移；在无 GPS 等绝对定位时，靠把当前图像对齐到先前图像消除漂移、强制全局一致；检测重访已建图区域（回环检测）校正漂移，跟踪失败时重建定位。
- **流水线并行（L55）**：tracking/mapping/optimization 等组件常**并行运行**以提效（PTAM/ORB-SLAM 多线程思想，见 §6）。

---

## 4. SLAM 问题的数学表述（主源 §2.3 + [DW2006] + [Cadena2016]）

> 本章最重要的"理性层次"。主源 §2.3 给出**离散运动方程/观测方程**与参数化例子，并由此**引出状态估计=滤波 or 优化**。全量+逐式保真，并用 [DW2006]（概率/滤波视角）与 [Cadena2016]（MAP/因子图视角）补全。

### 4.1 主源 §2.3：离散化与变量定义

- 相机在某些时刻采集数据 → 只关心这些时刻的位置和地图，把连续运动变成**离散时刻 $t=1,\cdots,K$** 的事情。
- 各时刻位置 $x_1,\cdots,x_K$ 构成小萝卜的**轨迹**。
- 地图由许多路标组成，共 $N$ 个，记 $y_1,\cdots,y_N$；每时刻传感器测到一部分路标的观测数据。
- "小萝卜携带传感器在环境中运动"由两件事描述：
  1. **什么是运动？** 考察从 $k-1$ 到 $k$ 时刻，位置 $x$ 如何变化。
  2. **什么是观测？** 假设 $k$ 时刻在 $x_k$ 处探测到路标 $y_j$，如何用数学描述。

### 4.2 主源 §2.3：运动方程（式 2.1）

携带测自身运动的传感器（码盘/惯性传感器），读数不一定是位置之差，还可能是加速度、角速度；也可能给指令（"前进1米""左转90°""油门踩到底""刹车"）。**通用抽象数学模型——运动方程**：
$$
\boldsymbol{x}_{k} = f\left(\boldsymbol{x}_{k-1}, \boldsymbol{u}_{k}, \boldsymbol{w}_{k}\right). \tag{2.1}
$$
- $\boldsymbol u_k$：运动传感器的读数或输入；
- $\boldsymbol w_k$：该过程中加入的**噪声**；
- $f$：一般函数，**不指明具体作用方式** → 可指代任意运动传感器/输入，成为通用方程。

**噪声 → 随机模型（主源原文）**："即使下达'前进1米'命令，也不代表小萝卜真前进了1米……可能某次只前进0.9米、另一次1.1米、再一次轮胎打滑没前进。每次运动噪声是随机的。若不理会噪声，只根据指令确定的位置可能与实际相差十万八千里。"

### 4.3 主源 §2.3：观测方程（式 2.2）

观测方程描述：小萝卜在 $x_k$ 看到路标 $y_j$ 时产生观测数据 $z_{k,j}$。抽象函数 $h$：
$$
\boldsymbol{z}_{k, j} = h\left(\boldsymbol{y}_{j}, \boldsymbol{x}_{k}, \boldsymbol{v}_{k, j}\right). \tag{2.2}
$$
- $\boldsymbol v_{k,j}$：这次观测里的噪声；
- 观测传感器形式多 → $z$ 与 $h$ 也有许多不同形式。

### 4.4 主源 §2.3：两个参数化具体例子（数值/代数全保留）

**例 1：平面运动 + 位移角度增量输入（式 2.3）**。平面中运动，位姿由两位置+一转角描述 $\boldsymbol x_k=[x_1,x_2,\theta]_k^\mathrm T$；输入为两时间间隔的位置和转角变化量 $\boldsymbol u_k=[\Delta x_1,\Delta x_2,\Delta\theta]_k^\mathrm T$。则运动方程具体化为：
$$
\left[\begin{array}{l} x_{1} \\ x_{2} \\ \theta \end{array}\right]_{k}
=\left[\begin{array}{l} x_{1} \\ x_{2} \\ \theta \end{array}\right]_{k-1}
+\left[\begin{array}{l} \Delta x_{1} \\ \Delta x_{2} \\ \Delta \theta \end{array}\right]_{k}
+\boldsymbol{w}_{k}. \tag{2.3}
$$
> 这是**简单的线性关系**。但并非所有输入都是位移/角度变化量——"油门""控制杆"输入是速度或加速度量，故存在更复杂的运动方程（需动力学分析）。

**例 2：二维激光观测（式 2.4）**。2D 激光观测一个 2D 路标点时测到两个量：路标点与本体的**距离 $r$** 和**夹角 $\phi$**。记路标 $\boldsymbol y_j=[y_1,y_2]_j^\mathrm T$，位姿 $\boldsymbol x_k=[x_1,x_2]_k^\mathrm T$，观测 $\boldsymbol z_{k,j}=[r_{k,j},\phi_{k,j}]^\mathrm T$，则观测方程：
$$
\left[\begin{array}{c} r_{k, j} \\ \phi_{k, j} \end{array}\right]
=\left[\begin{array}{c}
\sqrt{\left(y_{1, j}-x_{1, k}\right)^{2}+\left(y_{2, j}-x_{2, k}\right)^{2}} \\[2pt]
\arctan\left(\dfrac{y_{2, j}-x_{2, k}}{y_{1, j}-x_{1, k}}\right)
\end{array}\right]+\boldsymbol{v}. \tag{2.4}
$$
> **视觉 SLAM 的观测方程** = "对路标点拍摄后，得到图像中的像素"的过程，牵涉相机模型（第5讲），此处略。

### 4.5 主源 §2.3：通用抽象形式（式 2.5）与状态估计问题

保持通用性，取抽象形式，SLAM 过程可总结为**两个基本方程**：
$$
\left\{
\begin{array}{ll}
\boldsymbol{x}_{k} = f\left(\boldsymbol{x}_{k-1}, \boldsymbol{u}_{k}, \boldsymbol{w}_{k}\right), & k = 1, \dots, K \\[3pt]
\boldsymbol{z}_{k, j} = h\left(\boldsymbol{y}_{j}, \boldsymbol{x}_{k}, \boldsymbol{v}_{k, j}\right), & (k, j) \in \mathcal{O}
\end{array}
\right. \tag{2.5}
$$
- $\mathcal O$：集合，**记录在哪个时刻观察到了哪个路标**（通常不是每个路标每时刻都能看到——单个时刻往往只看到一小部分）。

**SLAM = 状态估计问题（主源原文）**："这两个方程描述了最基本的 SLAM 问题：当知道运动测量读数 $u$ 及传感器读数 $z$ 时，如何求解**定位（估计 $x$）和建图（估计 $y$）**？这时我们就把 SLAM 问题建模成一个**状态估计问题**：如何通过带有噪声的测量数据，估计内部的、隐藏着的状态变量？"

### 4.6 主源 §2.3：按线性/高斯分类 → 滤波 vs 优化（关键引出）

> 这是本章"由数学表述引出后续方法论"的枢纽。逐字保真。

求解依赖两方程的**具体形式**与**噪声分布**。按运动/观测方程**是否线性**、噪声**是否服从高斯分布**分类：

| 系统类型 | 方程线性性 | 噪声分布 | 求解方法 |
|---|---|---|---|
| **线性高斯系统（Linear Gaussian, LG）** | 线性 | 高斯 | **最简单**；其**无偏的最优估计**由**卡尔曼滤波器（Kalman Filter, KF）** 给出 |
| **非线性非高斯系统（Non-Linear Non-Gaussian, NLNG）** | 非线性 | 非高斯 | 用**扩展卡尔曼滤波器（EKF）** 和**非线性优化**两大类方法 |

**历史脉络（主源原文）**：
- 直至 **21 世纪早期**，以 **EKF** 为主的滤波器方法在 SLAM 中**占主导**。在工作点处把系统线性化，以**预测—更新**两步求解（第10讲）。**最早的实时视觉 SLAM 系统就基于 EKF[2]** 开发。
- 随后为克服 EKF 缺点（**线性化误差**和**噪声高斯分布假设**），人们用**粒子滤波器（Particle Filter）** 等其他滤波器，乃至**非线性优化**方法。
- **时至今日，主流视觉 SLAM 使用以图优化（Graph Optimization）为代表的优化技术进行状态估计[13]。** 作者观点：**优化技术已明显优于滤波器技术，只要计算资源允许，通常都偏向优化方法**（第10/11讲）。

**仍需澄清的问题（主源原文，引出后续各讲）**：
1. 机器人位置 $\boldsymbol x$ 是什么？平面可用两坐标+一转角，但小萝卜更多时候是**三维空间机器人**：3 轴平移 + 绕 3 轴旋转 = **6 自由度**。是否随便用 $\mathbb R^6$ 向量就能描述？**没那么简单**——6 自由度位姿如何表达、如何优化，是**第3讲、第4讲**内容（李群与李代数）。
2. 视觉 SLAM 中**观测方程如何参数化**（空间路标点如何投影到照片）→ 相机成像模型，**第5讲**。
3. 知道这些后**怎么求解上述方程** → 非线性优化，**第6讲**。

> 主源原文："可以看到，本讲介绍的内容构成了本书的提要。"（见 §7 全书路线图）

### 4.7 权威补强：递归贝叶斯（滤波视角，[DW2006]）

> 主源 §2.3 把方程写成**函数形式** $x_k=f(\cdot)$；[DW2006] 写成**概率密度形式**并给出**递归贝叶斯滤波**的完整两步。两者等价，综合时并列呈现最佳。逐式保真（含定义、式 1–5）。

**变量定义（dw_tutorial.txt L194–215）**——在时刻 $k$：
- $x_k$：描述车辆位置与朝向的**状态向量**；
- $u_k$：**控制向量**，在 $k-1$ 时刻施加以把车辆驱动到 $k$ 时刻状态 $x_k$；
- $m_i$：描述第 $i$ 个路标位置的向量，其**真实位置假设时不变**；
- $z_{ik}$（或简记 $z_k$）：$k$ 时刻从车上对第 $i$ 个路标位置的**观测**。
- 集合：$X_{0:k}=\{x_0,\dots,x_k\}$ 车辆位置历史；$U_{0:k}=\{u_1,\dots,u_k\}$ 控制历史；$m=\{m_1,\dots,m_n\}$ 全部路标；$Z_{0:k}=\{z_1,\dots,z_k\}$ 全部路标观测。

**概率 SLAM 要求（式 1，L243–245）**：对所有时刻 $k$ 计算联合后验密度
$$
P\!\left(x_k, m \mid Z_{0:k}, U_{0:k}, x_0\right). \tag{DW-1}
$$
描述：给定到 $k$ 为止的全部观测和控制及车辆初始状态，路标位置与车辆状态（$k$ 时刻）的**联合后验密度**。

**观测模型（式 2，L257–261）**：
$$
P\!\left(z_k \mid x_k, m\right). \tag{DW-2}
$$
"once the vehicle location and map are defined, observations are conditionally independent given the map and the current vehicle state."（车辆位置与地图给定后，观测条件独立。）

**运动模型（式 3，L266–271）**：
$$
P\!\left(x_k \mid x_{k-1}, u_k\right). \tag{DW-3}
$$
"the state transition is assumed to be a **Markov process** in which the next state $x_k$ depends only on the immediately preceding state $x_{k-1}$ and the applied control $u_k$ and is independent of both the observations and the map."（一阶马尔可夫。）

**递归两步（式 4、5，L219–288）**——SLAM 以标准**预测（time-update）—校正（measurement-update）**递归实现：

*时间更新 / 预测（Time-update，式 4）*：
$$
P\!\left(x_k, m \mid Z_{0:k-1}, U_{0:k}, x_0\right)
= \int P\!\left(x_k \mid x_{k-1}, u_k\right)\,
P\!\left(x_{k-1}, m \mid Z_{0:k-1}, U_{0:k-1}, x_0\right)\, \mathrm d x_{k-1}. \tag{DW-4}
$$

*测量更新 / 校正（Measurement Update，式 5）*：
$$
P\!\left(x_k, m \mid Z_{0:k}, U_{0:k}, x_0\right)
= \frac{P\!\left(z_k \mid x_k, m\right)\,
P\!\left(x_k, m \mid Z_{0:k-1}, U_{0:k}, x_0\right)}
{P\!\left(z_k \mid Z_{0:k-1}, U_{0:k}\right)}. \tag{DW-5}
$$
> "Equations (4) and (5) provide a recursive procedure for calculating the joint posterior $P(x_k,m\mid Z_{0:k},U_{0:k},x_0)$ ... a function of a vehicle model $P(x_k\mid x_{k-1},u_k)$ and an observation model $P(z_k\mid x_k,m)$."（L228–231）

**解法（[DW2006] L375–388）**："Solutions to the probabilistic SLAM problem involve finding an appropriate representation for both the observation model (2) and motion model (3) ... By far, the most common representation is in the form of a **state-space model with additive Gaussian noise**, leading to the use of the **extended Kalman filter (EKF)**." 另一重要替代：把运动模型(3)描述为更一般非高斯分布的**样本** → **Rao-Blackwellized 粒子滤波（FastSLAM）**。EKF-SLAM 与 FastSLAM 是两大经典解法；更新的有**信息形式（information-state form）**。

> **EKF-SLAM 收敛定理（[DW2006] dw_tutorial.txt L421–430）**："In the EKF-SLAM problem, convergence of the map is manifest in the **monotonic convergence of the determinant** of the map covariance matrix ... toward a lower bound determined by **initial uncertainties in robot position and observations**."（地图协方差行列式单调收敛到由初始不确定性决定的下界——见图3，地标位置方差标准差随时间单调减小。这与 §1.5 的单调收敛洞见对应。）

### 4.8 权威补强：MAP / 因子图（优化视角，[Cadena2016] + [Handbook-FG]）

> 主源 §2.2.2 提到 MAP 但未给公式；现代主流（图优化）以 MAP/因子图为标准。给出 [Cadena2016] 的完整 MAP 推导（式 1–4），供成书"优化视角"自洽。逐式保真。

**变量与测量（cadena.txt L369–377）**：估计未知变量 $X$（SLAM 中含机器人轨迹的离散位姿集合 + 环境路标位置）；给定测量集 $Z=\{z_k: k=1,\dots,m\}$，每个测量可写成 $X$ 的函数：
$$
z_k = h_k(X_k) + \epsilon_k,
$$
其中 $X_k\subseteq X$ 是变量子集，$h_k(\cdot)$ 是已知函数（**测量/观测模型**），$\epsilon_k$ 是随机测量噪声。

**MAP 估计（式 1，L378–401）**：
$$
X^{\star} = \arg\max_{X} p(X \mid Z) = \arg\max_{X} p(Z \mid X)\, p(X), \tag{Cadena-1}
$$
等号由 Bayes 定理。$p(Z\mid X)$ 是似然，$p(X)$ 是先验；无先验知识时 $p(X)$ 为常数（均匀分布）可丢弃，此时 **MAP 退化为最大似然估计（MLE）**。注："unlike Kalman filtering, MAP estimation does not require an explicit distinction between motion and observation model: both models are treated as factors"；且**线性高斯情形下卡尔曼滤波与 MAP 给出相同估计**，一般情形则不同。

**因子分解（式 2，L402–424）**：假设测量 $Z$ 独立（对应噪声不相关），(1) 分解为
$$
X^{\star} = \arg\max_{X} p(X) \prod_{k=1}^{m} p(z_k \mid X) = \arg\max_{X} p(X) \prod_{k=1}^{m} p(z_k \mid X_k), \tag{Cadena-2}
$$
右端用了 $z_k$ 只依赖变量子集 $X_k$。这可解释为**因子图（factor graph）** 上的推断：变量=节点，$p(z_k\mid X_k)$ 与先验 $p(X)$ 称**因子**（编码变量子集上的概率约束）。

> **因子图 SLAM（[Cadena2016] 图3 caption, L434–440）**：蓝圆=连续时刻机器人位姿 $x_1,x_2,\dots$；绿圆=路标位置 $l_1,l_2,\dots$；红圆=内参标定参数 $K$；黑方块=因子（"u"=里程计约束、"v"=相机观测、"c"=回环、"p"=先验）。优点：(1) 直观可视化；(2) 通用性（异构变量/因子、任意互连）；(3) 因子图连通性决定 SLAM 问题的**稀疏性**。

**高斯噪声 → 非线性最小二乘（式 3、4，L452–490）**：设测量噪声 $\epsilon_k$ 为零均值高斯、信息矩阵 $\Omega_k$（协方差的逆），则测量似然
$$
p(z_k \mid X_k) \propto \exp\!\left(-\tfrac{1}{2}\,\|h_k(X_k) - z_k\|_{\Omega_k}^{2}\right), \tag{Cadena-3}
$$
记号 $\|e\|_\Omega^2 = e^\mathrm T \Omega e$。先验同样设 $p(X)\propto\exp(-\tfrac12\|h_0(X)-z_0\|_{\Omega_0}^2)$。最大化后验=最小化负对数后验，故 MAP 估计 (2) 变为
$$
X^{\star} = \arg\min_{X}\left(-\log p(X)\prod_{k=1}^{m}p(z_k\mid X_k)\right) = \arg\min_{X} \sum_{k=0}^{m} \|h_k(X_k) - z_k\|_{\Omega_k}^{2}, \tag{Cadena-4}
$$
这是一个**非线性最小二乘问题**（$h_k$ 多为非线性）。注：(4) 来自正态噪声假设；Laplace 噪声 → $\ell_1$ 范数；为抗外点常把平方 $\ell_2$ 换成**鲁棒核（Huber/Tukey）**。

**与 BA 的关系（L498–514）**：(4) 与 Structure-from-Motion 的 **Bundle Adjustment (BA)** 相似（都源于 MAP），但 SLAM 有两点独特：(1) 因子不限于投影几何，可含惯性、轮速、GPS 等多种传感器模型（激光约束相对位姿、直接法惩罚像素强度差）；(2) SLAM 需**增量求解**（机器人移动时新测量逐步到来）。

**求解与稀疏性（L515–541）**：(4) 常用**逐次线性化（Gauss-Newton / Levenberg-Marquardt）** 解：从初值 $\hat X$ 起，用二次近似代价、解线性**正规方程（normal equations）**；可推广到流形（如旋转）。**现代 SLAM 求解器的关键洞见：正规方程的矩阵是稀疏的，稀疏结构由因子图拓扑决定** → 用快速线性求解器 + 增量/在线求解。库：GTSAM、g2o、Ceres、iSAM、SLAM++（万级变量数秒内解）。该框架又称 **MAP 估计 / 因子图优化 / graph-SLAM / full smoothing / smoothing and mapping (SAM)**；流行变体 **位姿图优化（pose graph optimization）**——变量为轨迹上采样的位姿，每因子约束一对位姿。

**MAP vs 滤波（L542–558）**："MAP estimation has been proven to be more accurate and efficient than original approaches for SLAM based on nonlinear filtering." 但某些 EKF 系统也达 SOTA（如 MSCKF [175]、Kottas [139]、Hesch [105] 的 VIN 系统）；当 EKF 线性化点准确（如视觉-惯性导航）、用滑窗滤波、处理好不一致性来源时，滤波与 MAP 的性能差距缩小。

> **[Handbook-FG] 因子图玩具例（01_Factor_Graphs_for_SLAM.md L21–80，供成书举例）**：3 个位姿 $p_1,p_2,p_3$ + 2 个路标 $\ell_1,\ell_2$ + 对 $p_1$ 的绝对测量。状态 $x=[p_1;p_2;p_3;\ell_1;\ell_2]$（式1.2）。后验 $p(x\mid z)\propto p(z\mid x)p(x)$（Bayes），按马尔可夫生成模型分解（式1.4a–d）：
> $$p(x\mid z) \propto p(p_1)p(p_2\mid p_1)p(p_3\mid p_2)\cdot p(\ell_1)p(\ell_2)\cdot p(z_1\mid p_1)\cdot p(z_2\mid p_1,\ell_1)p(z_3\mid p_2,\ell_1)p(z_4\mid p_3,\ell_2).$$
> 每个因子 $\phi$ 只连其依赖的变量节点（如 $\phi_9(p_3,\ell_2)$ 只连 $p_3,\ell_2$）。这正是 §2.2.2 后端"在因子图/图优化上做 MAP"的最小可视化。

---

## 5. SLAM 分类（按传感器 / 按方法；主源散点 + [Cadena2016] + [Handbook-VSLAM] + Strasdat）

> 本章【聚焦】要求"按传感器/方法分类"。主源把分类散在 §2.1（相机类型）、§2.3（线性/高斯→滤波/优化）；下面系统整理为成书可用的分类表，并标权威出处。

### 5.1 按传感器分类

| 维度 | 类别 | 说明 | 出处 |
|---|---|---|---|
| 传感器主体 | **视觉 SLAM** | 主要用相机解决定位建图（本书主题）。又按相机分单目/双目/RGB-D | 主源 §2.1 |
| | **激光 SLAM** | 主要用激光（2D/3D LiDAR）；相对成熟（《概率机器人》[5]、ROS 现成软件） | 主源 §2.1 |
| | **视觉-惯性 SLAM（VI-SLAM / VIN）** | 相机 + IMU；IMU 给高频运动+尺度。VIN 可视为禁用回环的简化 SLAM；"VIN 就是 SLAM" | [Cadena2016] L182–187 |
| 相机类型 | **单目 Monocular** | 单摄像头；结构简单成本低；**尺度不确定性**、需平移才能算深度 | 主源 §2.1 |
| | **双目 Stereo / 多目** | 已知基线估每像素深度；室内外皆可；计算量大需 GPU/FPGA | 主源 §2.1 |
| | **深度 RGB-D** | 结构光/ToF 主动测距；省算力；主要室内 | 主源 §2.1 |
| | 特殊/新兴 | 全景相机[7]、Event 相机[8]（尚非主流） | 主源 §2.1 |

> **现代系统补充（[Handbook] 各章 + 总大纲）**：还有**激光-惯性 SLAM (LIO)**、**激光-视觉-惯性 SLAM (LVI)**、Radar SLAM、Event-based SLAM、Leg odometry 等（详见本书后续相应章/SLAM Handbook ch.8–12）。

### 5.2 按方法（后端状态估计）分类

| 维度 | 类别 | 说明 | 出处 |
|---|---|---|---|
| **后端范式** | **滤波（Filtering）** | EKF/UKF/粒子滤波等；边缘化过去位姿、用概率分布概括历史信息；21 世纪早期主导 | 主源 §2.3；[Cadena2016] L542–558 |
| | **优化 / 平滑（Optimization / Smoothing）** | 图优化/因子图/BA/位姿图；保留（部分）历史位姿做全局优化；当代主流 | 主源 §2.3；[Cadena2016] L535–544 |
| **系统线性/噪声** | **线性高斯 (LG)** | KF 给无偏最优估计 | 主源 §2.3 |
| | **非线性非高斯 (NLNG)** | EKF / 非线性优化 | 主源 §2.3 |
| **视觉前端范式** | **特征点法（feature-based / indirect）** | 提取-匹配特征点，最小化**重投影误差**；显式数据关联、收敛域大、对光照/动态较稳健、成熟 | [Handbook-VSLAM] L39–41；十四讲 §7.1 |
| | **直接法（direct）** | 直接最小化**光度误差**，不需特征匹配；与光流/photometric BA 相关；常 coarse-to-fine | [Handbook-VSLAM] L39–41；十四讲第8讲 |
| | 半直接/混合 | 介于二者（如 SVO） | [Handbook-VSLAM] |

> **滤波 vs 优化的权威结论（Strasdat et al., "Visual SLAM: Why Filter?", IVC 2012 / ICRA 2010；联网核验）**："while filtering may have a niche in systems with low processing resources, in most modern applications **keyframe optimisation gives the most accuracy per unit of computing time**." 滤波边缘化过去位姿、用概率分布概括历史；关键帧法保留全局 BA 的优化结构、但只选少量过去帧。**路标数很大时基于关键帧 BA 的方法优于滤波法**。→ 与主源 §2.3"优化技术已明显优于滤波器技术"一致。

> **历史里程碑系统（联网核验，供成书"分类配实例"）**：
> - **MonoSLAM**（Davison 2007）：**首个实时单目视觉 SLAM**，用 **EKF** + Shi-Tomasi 点；缺点：复杂度 $O(N^3)$（$N$=路标数），仅适合几百点的小场景。
> - **PTAM**（Klein & Murray 2007）：首次用**非线性优化**替代滤波做后端，并首次**并行化 tracking 与 mapping**（双线程）；自 PTAM 起，主流方法把 SfM 的 BA 引入优化型视觉 SLAM。

---

## 6. 现代代表系统概览（联网精读：ORB-SLAM2/3、VINS-Mono、LIO-SAM）

> 本章【聚焦】要求"现代代表系统概览"。主源第2讲未涉及（属第14讲与本书后续）。下面据**原论文 + 官方仓库**抽取，供成书在概览章末尾给"现代系统坐标系"。

### 6.1 ORB-SLAM 家族（特征点法视觉 SLAM 标杆）

| 系统 | 年/出处 | 传感器 | 核心特征 |
|---|---|---|---|
| **ORB-SLAM** | Mur-Artal, Montiel, Tardós 2015, T-RO 31:1147–1163 (arXiv:1502.00956)；获 **2015 T-RO Best Paper** | 单目 | 特征点法；**三线程并行：tracking / local mapping / loop closing**；**所有任务共用同一 ORB 特征**（tracking/mapping/relocalization/loop closing）；用 **DBoW2（二进制词袋）** 做快速地点识别；covisibility graph + essential graph；回环用 **Sim3**（修正单目尺度漂移）；全自动初始化；宽基线回环与重定位 |
| **ORB-SLAM2** | Mur-Artal, Tardós 2017, T-RO (arXiv:1610.06475)；开源 | 单目 / 双目 / RGB-D | 首个**统一**支持三种相机的开源系统；**后端基于 Bundle Adjustment**（单目+双目观测），双目/RGB-D 给**真实尺度（metric）**轨迹；含地图复用、回环、重定位；标准 CPU 实时（手持室内→工业无人机→城市行车） |
| **ORB-SLAM3** | Campos, Elvira, Rodríguez, Montiel, Tardós 2021, T-RO (arXiv:2007.11898)；开源 | 单目/双目/RGB-D + IMU；pin-hole & 鱼眼 | 首个做**视觉、视觉-惯性、多地图 SLAM** 的系统。两大创新：(1) **完全基于 MAP 估计的紧耦合视觉-惯性 SLAM**（连 IMU 初始化阶段也用 MAP）→ 实时鲁棒，比前法**精度高 2~5 倍**；(2) **多地图系统 ATLAS**（基于改进召回率的新地点识别方法），跟踪丢失时新建子地图、后续与旧图合并 |

### 6.2 VINS-Mono（单目视觉-惯性，优化型 VIO 标杆）

- **出处**：Qin, Li, Shen, *"VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator,"* IEEE T-RO 34(4):1004–1020, 2018（arXiv:1708.03852）；官方仓库 HKUST-Aerial-Robotics/VINS-Mono。
- **传感器**：**单目相机 + 低成本 IMU**——度量级 6-DOF 状态估计的**最小传感器组合**。
- **核心方法**：
  - **紧耦合（tightly-coupled）非线性优化**：在**滑动窗口**中融合**预积分 IMU 测量** + 特征观测，得高精度视觉-惯性里程计。
  - **鲁棒的初始化**：估计器初始化 + 失败恢复（failure recovery）。
  - **回环 + 重定位**：回环检测模块 + 紧耦合 relocalization（最小计算）。
  - **全局一致**：执行 **4-DOF 位姿图优化**（重力使 roll/pitch 可观，只需优化 x/y/z/yaw 四自由度）强制全局一致。

### 6.3 LIO-SAM（激光-惯性，因子图平滑型 LIO 标杆）

- **出处**：Shan, Englot, Meyers, Wang, Ratti, Rus, *"LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping,"* IEEE/RSJ IROS 2020（arXiv:2007.00258）；官方仓库 TixiaoShan/LIO-SAM。
- **核心方法**：
  - 把**激光-惯性里程计建立在因子图（factor graph）之上**，允许把不同来源的相对/绝对测量（含回环）作为**因子**纳入系统。
  - **四类因子**：(a) **IMU 预积分因子**；(b) **激光里程计因子**；(c) **GPS 因子**；(d) **回环因子**。
  - **IMU 预积分**估计的运动用于**点云去畸变（de-skew）** + 给激光里程计优化提供初值；激光里程计解又用于**估计 IMU 偏置（bias）**。
  - **实时性**：对位姿优化**边缘化（marginalize）旧的激光扫描**，而非把扫描匹配到全局地图（局部滑窗 scan-matching）。

> **抽取专员注（系统映射）**：三系统恰对应三种现代主流：ORB-SLAM=纯视觉（特征点法+优化+多地图）；VINS-Mono=视觉惯性（紧耦合滑窗优化）；LIO-SAM=激光惯性（因子图平滑）。详见本书后续 VO/VIO/LIO/回环/后端各章及 SLAM Handbook ch.7/8/11。

---

## 7. 实践：编程基础与 "Hello SLAM"（主源 §2.4，全量含全部代码）

> 本章【聚焦】要求"编程环境与 'Hello SLAM' 工程基础（cmake/库链接的概念）"。主源 §2.4 完整保留（含每段代码、终端输出、cmake 概念）。

### 7.1 §2.4.1 安装 Linux 操作系统

- 程序以 **Linux 上的 C++ 程序**为主；用大量程序库，多数只对 Linux 支持好（Windows 配置麻烦）。
- 选 **Ubuntu** 作开发环境（对新手友好；开源；清华/中科大有软件源）。第1版用 Ubuntu 14.04，第2版默认更新至 **Ubuntu 18.04**（图2-11 虚拟机运行的 Ubuntu 18.04）。其他可选：Ubuntu Kylin、Debian、Deepin、Linux Mint。讲解以 Ubuntu 18.04 + `apt-get` 命令为例。
- 安装方式：**虚拟机**（最简单，但需 4GB 以上内存+CPU 才流畅；对外部硬件支持差）或**双系统**（更快；需空白 U 盘做启动盘；用实际传感器如双目/Kinect 建议双系统）。
- 安装小提示：(1) 安装时断网、不选"安装中下载更新"以提速（SSD 约 15 分钟），装完再更新；(2) 把软件源设到就近服务器（清华源约 10MB/s）。
- 代码根目录：放在家目录 `/home` 下的 `slambook2`；按章节划分（第2讲在 `slambook2/ch2`、第3讲在 `slambook2/ch3`）。

### 7.2 §2.4.2 Hello SLAM

**程序是什么（Linux）**：一个**具有执行权限的文件**（脚本或二进制，不限后缀名）。`cd`、`ls` 等是 `/bin` 下的可执行文件；任何有可执行权限的程序，在终端输入程序名即可运行。

**源代码 `slambook2/ch2/helloSLAM.cpp`**：
```cpp
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
    cout << "Hello SLAM!" << endl;
    return 0;
}
```

**用 g++ 编译**（编译器把文本文件编译成可执行程序）。终端输入：
```txt
g++ helloSLAM.cpp
```
- 顺利则无任何输出。若 "command not found" → 未装 g++：`sudo apt-get install g++`。若别的错误，检查程序是否输入正确。
- 编译产生有执行权限的 `a.out`（g++ 默认输出名）。运行：
```txt
% ./a.out
Hello SLAM!
```
回顾：编辑器写源码 → g++ 编译 → 得可执行文件（默认 `a.out`，可指定输出名，留作习题）。下面用 cmake 编译。

### 7.3 §2.4.3 使用 cmake

**动机**：工程规模大时（许多文件夹/源文件、十几个类、复杂依赖、部分编可执行/部分编库），仅靠 g++ 命令极烦琐。历史上用 **makefile**，但 **cmake 更方便**且工程上广泛使用（后面大多数库都用 cmake 管理）。

**cmake 工作流**：用 `cmake` 命令生成 `makefile`，再用 `make` 根据 makefile 编译整个工程。

**`slambook2/ch2/CMakeLists.txt`**（最基本工程：指定工程名 + 一个可执行程序）：
```txt
# 声明要求的cmake最低版本
cmake_minimum_required( VERSION 2.8 )
# 声明一个cmake工程
project( HelloSLAM )
# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 )
add_executable( helloSLAM helloSLAM.cpp )
```

**编译（在 `slambook2/ch2/` 下）**：
```txt
cmake .
```
cmake 输出编译信息并在当前目录生成中间文件（最重要的是 **MakeFile**，自动生成、不必修改）。再 `make`：
```txt
% make
Scanning dependencies of target helloSLAM
[100%] Building CXX object CMakeFiles/helloSLAM.dir/helloSLAM.cpp.o
Linking CXX executable helloSLAM
[100%] Built target helloSLAM
```
得到可执行程序 `helloSLAM`，执行：
```txt
% ./helloSLAM
Hello SLAM!
```
> **cmake vs g++ 区别（主源原文）**：cmake 处理工程文件间关系，make 实际调用 g++ 编译。多了 cmake/make 步骤，但把"输入一串 g++ 命令"变成"维护若干直观的 CMakeLists.txt"，明显降低维护难度（新增可执行文件只需加一行 `add_executable`）。

**out-of-source 构建（更常见做法，把中间文件隔离到 build 目录）**：
```batch
mkdir build
cd build
cmake ..
make
```
新建 `build` 文件夹，进入后用 `cmake ..` 对上一层（源代码所在文件夹）编译 → 中间文件生成在 `build` 内、与源码分开；发布时删 `build` 即可。

### 7.4 §2.4.4 使用库（库的概念 + 静态/共享库 + 头文件 + 链接）

**库（Library）**：只有带 `main` 函数的文件才编成可执行程序；其他代码打包成**库**供其他程序调用。库是许多算法/程序的集合（如 OpenCV 提供计算机视觉算法、Eigen 提供矩阵代数计算）。

**库源文件 `slambook2/ch2/libHelloSLAM.cpp`**（无 main，故库里无可执行文件）：
```cpp
//这是一个库文件
#include <iostream>
using namespace std;

void printHello() {
    cout << "Hello SLAM" << endl;
}
```

**在 CMakeLists.txt 中编译成库**：
```txt
add_library( hello libHelloSLAM.cpp )
```
编译（`cd build; cmake ..; make`）后在 `build` 中生成 **`libhello.a`**（静态库）。

**静态库 vs 共享库（主源原文）**：Linux 库分两种——**静态库以 `.a` 结尾**，**共享库以 `.so` 结尾**。都是函数打包集合，差别：**静态库每次被调用都生成一个副本；共享库只有一个副本，更省空间**。生成共享库：
```cmake
add_library( hello_shared SHARED libHelloSLAM.cpp )
```
得 **`libhello_shared.so`**。

**头文件（说明库里有什么）**：仅有 `.a`/`.so` 不知里面函数及调用形式 → 需提供头文件。使用者拿到**头文件 + 库文件**即可调用。

**头文件 `slambook2/ch2/libHelloSLAM.h`**：
```c
#ifndef LIBHELLOSLAM_H_
#define LIBHELLOSLAM_H_
// 上面的宏定义是为了防止重复引用这个头文件而引起的重定义错误
// 打印一句Hello的函数
void printHello();
#endif
```

**调用库的可执行程序 `slambook2/ch2/useHello.cpp`**：
```c
#include "libHelloSLAM.h"
// 使用libHelloSLAM.h中的printHello()函数
int main(int argc, char **argv) {
    printHello();
    return 0;
}
```

**在 CMakeLists.txt 中生成可执行程序并链接到库**：
```txt
add_executable( useHello useHello.cpp )
target_link_libraries( useHello hello_shared )
```
两行使 `useHello` 顺利使用 `hello_shared` 库的代码。对他人提供的库也用同样方式调用、整合到自己程序。

**回顾（主源原文，库使用三要点）**：
1. 程序代码由**头文件和源文件**组成。
2. 带 `main` 的源文件编成**可执行程序**，其他编成**库文件**。
3. 可执行程序调库需**参考库的头文件**（明白调用格式），并**把可执行程序链接到库文件上**。
> 思考题：若引用了库函数但忘了链接到库会怎样？（去掉链接部分看 cmake 报错——见习题6）

### 7.5 §2.4.5 使用 IDE（KDevelop / Clion + 断点调试）

- **为什么用 IDE**：文本编辑器够用，但文件多时跳转、查函数声明/实现烦琐；IDE 提供跳转、补全、断点调试。
- Linux 下 C++ IDE：Eclipse、Qt Creator、Code::Blocks、Clion、VS Code 等。主源用 **KDevelop（免费，在 Ubuntu 仓库，可 apt-get 装）** 与 **Clion（收费，学生邮箱免费一年；对主机要求更高）**。两者优点：支持 cmake 工程；对 C++（含 11+）支持好（高亮/跳转/补全/排版）；方便看文件目录树；一键编译+断点调试。
- **KDevelop 用法**：工程→打开/导入工程→打开 CMakeLists.txt，软件默认建 build 文件夹并调用 cmake/make（快捷键 F8）。
- **断点调试（底层是 gdb）三步**：(1) CMakeLists.txt 设为 Debug 模式且不开优化（默认不开）：
  ```cmake
  set( CMAKE_BUILD_TYPE "Debug" )
  ```
  （Debug 模式运行较慢但可断点调试；Release 模式快但无调试信息。）
  (2) "运行→配置启动器"→"Add New→应用程序"，告诉 KDevelop 启动哪个程序（建议直接指向二进制文件，问题更少；可设运行参数和工作目录）。(3) 进断点调试界面单步运行：单步运行 F10、单步跟进 F11、单步跳出 F12；展开左侧看局部变量值。程序崩溃时可用断点调试定位出错位置。
- **Clion 用法**：打开 CMakeLists.txt 或指定目录，自动完成 cmake-make；右上角选运行/调试的程序、调启动参数与工作目录；小甲虫按钮启动断点调试。还有自动创建类、函数改名、自动调整编码风格等。
- 入门章到此为止；今后实践不再重复"新建 build、调用 cmake/make"的步骤。

---

## 8. ★全书路线图——本章如何引出后续每一章（主源 §2.3 末 + 全书结构 + 总大纲）

> 本章【聚焦】明确要求"★全书路线图——本章如何引出后续每一章"。下面综合主源 §2.3 末尾"仍需澄清的问题"、十四讲全书章节结构、以及本项目《SLAM理论_总大纲》的纵切主线，给出**本章作为全书提要**如何牵引各后续章。

### 8.1 主源 §2.3 末尾显式埋下的伏笔（逐条对应章）

主源原文明确点名后续讲：
- **6 自由度位姿如何表达、如何优化** → **第3讲《三维空间刚体运动》**（旋转矩阵 $R\in\mathrm{SO}(3)$、变换矩阵 $T\in\mathrm{SE}(3)$、四元数、旋转向量、欧拉角）+ **第4讲《李群与李代数》**（$\mathfrak{so}(3)/\mathfrak{se}(3)$、$\exp/\log$、BCH、左右扰动求导、$\xi=[\rho;\phi]$）。
- **观测方程如何参数化（空间路标点如何投影到照片）** → **第5讲《相机与图像》**（针孔模型、内参 $K$、畸变、双目/RGB-D 模型、图像数据）。
- **怎么求解上述方程**（带噪测量下估隐藏状态） → **第6讲《非线性优化》**（最小二乘、Gauss-Newton、Levenberg-Marquardt、状态估计=MLE/MAP、Ceres/g2o）。

### 8.2 框架五模块 → 后续各章（本章 §3 框架是全书目录）

| 本章模块（§3） | 引出的章 | 内容 |
|---|---|---|
| 前端 VO（§3.2） | **第7讲《视觉里程计1》** | 特征点法：ORB 特征、对极几何(E/F/H)、PnP、ICP、三角化 |
| 前端 VO（§3.2） | **第8讲《视觉里程计2》** | 直接法/光流（LK）：光度误差、稀疏/半稠密直接法 |
| 后端优化（§3.3） | **第9讲《后端1》** | 状态估计概论、BA 与图优化、批量/增量、稀疏性与 Schur 消元 |
| 后端优化（§3.3） | **第10讲《后端2》** | 位姿图、滑动窗口、滤波（EKF）回顾、因子图/iSAM |
| 回环检测（§3.4） | **第11讲《回环检测》** | 词袋模型 DBoW、相似性评分、感知混叠、回环验证 |
| 建图（§3.5） | **第12讲《建图》** | 单目稠密重建、RGB-D 稠密建图、八叉树 OctoMap、TSDF |
| 整体框架（§3.1） | **第13讲《实践：设计 SLAM 系统》** | 搭建一个完整的双目/RGB-D VO 系统（前端+局部地图） |
| SLAM 全景（§1） | **第14讲《SLAM：现在与未来》** | 现状、开放问题、未来方向（与 [Cadena2016] 鲁棒感知时代呼应） |
| 数学预备 | **第1讲《预备知识》** + **附录/参考文献（第15讲）** | 课程介绍、Linux/IDE/数学软件、Eigen 等 |

### 8.3 由本章数学表述 → 状态估计两条主线（§4.6/§4.7/§4.8）

- 本章 §4.6 由"线性/高斯分类"引出**滤波（KF/EKF/PF）**与**优化（图优化）**两大求解范式 → 贯穿第9/10讲。
- §4.7（[DW2006] 递归贝叶斯：预测-校正）= **滤波主线**的母方程（第10讲 EKF/滑窗滤波）。
- §4.8（[Cadena2016] MAP/因子图：非线性最小二乘）= **优化主线**的母方程（第6/9讲 BA/图优化、第11讲回环作为长期数据关联因子）。
- 本项目《SLAM理论_总大纲》的纵切主线可与本章对接：`状态估计 → Lie与优化 → 三模块测量模型(视觉/激光/惯性) → VIO/LIO → LVI → 标定/可观性/退化/鲁棒 → 前沿`（见总大纲 §A/§D）。本章 §5（分类）与 §6（现代系统 ORB-SLAM/VINS/LIO-SAM）正是该主线在"模块/融合"层的预告。

### 8.4 本章在全书中的定位（主源原文收束）

> 主源原文："本讲概括地介绍了一个视觉 SLAM 系统的结构，作为后续内容的大纲。" / "可以看到，本讲介绍的内容构成了本书的提要。如果读者还没有很好地理解上面的概念，不妨回过头再阅读一遍。"
>
> 即：**《初识 SLAM》= 全书地图**。框架五模块=后续 §7–§12 各章；数学表述=后续 §3–§6 数学预备 + §9–§10 状态估计；实践 Hello SLAM=后续所有章的 cmake/库工程基础；SLAM 全景与现代系统=第14讲的伏笔。

---

## 9. 习题（主源 §习题，全量保留）

1. 阅读文献 [1] 和 [14]，你能看懂其中的内容吗？
2.\* 阅读 SLAM 的综述文献，例如 [9, 15–18] 等。这些文献中关于 SLAM 的看法与本书有何异同？
3. g++ 命令有哪些参数？怎么填写参数可以更改生成的程序文件名？
4. 使用 build 文件夹来编译你的 cmake 工程，然后在 KDevelop 中试试。
5. 刻意在代码中添加一些语法错误，看看编译会生成什么样的信息。你能看懂 g++ 的错误信息吗？
6. 如果忘了把库链接到可执行程序上，编译会报错吗？报什么样的错？
7.\* 阅读《cmake 实践》，了解 cmake 的其他语法。
8.\* 完善 Hello SLAM 小程序，把它做成一个小程序库，安装到本地硬盘中。然后，新建一个工程，使用 `find_package` 找这个库并调用。
9.\* 阅读其他 cmake 教学材料，例如 https://github.com/TheErk/CMake-tutorial 。
10. 找到 KDevelop 的官方网站，看看它还有哪些特性。你都用上了吗？
11. 如果在第 1 讲学习了 Vim，那么请试试 KDevelop 的 Vim 编辑功能。

> 主源开头"主要目标"4 条：(1) 理解视觉 SLAM 框架由哪几个模块组成、各模块任务；(2) 搭建编程环境；(3) 理解 Linux 下编译运行程序、如何调试；(4) 掌握 cmake 基本使用。

---

## 10. OCR 修正说明

> 主源 `02_初识SLAM.md` 由 MinerU OCR 自 PDF 生成，少量 OCR 噪声与图片占位。本抽取已做如下处理（不改变技术内容）：

1. **OCR 错字（图标题/正文）**：原文件第15行 "Visnel Odometry 视觉里程计" → 应为 "Visual Odometry 视觉里程计"（已在抽取中用正确拼写）。第20–22行散落的 "圆环检测 / Loop closure地图构建 / Mayang" 是图2-7 框架图中模块标签的 OCR 碎片（"圆环检测"应为"回环检测"，"Mayang"=Mapping 的误识），已据上下文还原为正确的五模块标签（见 §3.1 框架图）。
2. **图片占位**：源含大量 `![image](https://cdn-mineru.openxlab.org.cn/...jpg)` 外链图（图2-1 小萝卜设计图、2-2 各类传感器、2-3 三类相机、2-4 单目深度尴尬、2-5 双目数据、2-6 RGB-D 数据、2-7 经典框架、2-8 相机运动、2-9 累积误差与回环校正、2-10 各类地图、2-11 Ubuntu、2-12~2-15 IDE 界面）。抽取保留**图编号 + 图题 + 内容描述**，未嵌外链（成书需重绘 TikZ，§3.1 已给框架图绘制说明）。
3. **数学符号渲染**：源 LaTeX 多处用 `$^{①}$` 等带圈数字脚注、`$\pmb{}$`/`$\boldsymbol{}$` 混用、`Make-File`/`MakeFile` 断词。抽取统一为标准 LaTeX（粗体向量 `\boldsymbol`），公式编号沿用源 (2.1)–(2.5)。脚注内容（如尺度脚注、Ubuntu 源速度脚注、A*/D* 脚注）已融入正文相应位置。
4. **权威源公式**：[DW2006]/[Cadena2016] 的公式由本地 PDF 经 `pdftotext` 抽取后，由抽取专员转写为规范 LaTeX 并标注原文行号（dw_tutorial.txt / cadena.txt），编号加前缀 (DW-x)/(Cadena-x) 以区别于主源 (2.x)。pdftotext 对上下标偶有错位（如 $z_k^i$ 被压成 $z_{ik}$、$X_{0:k}$ 的冒号），已据论文语义校正。
5. **未抽取的纯叙事**：主源中纯过渡性、与技术无关的口语段落（如"令人兴奋的实践环节""笔者有些话唠"）做了压缩，但所有**技术陈述、定义、权衡、代码、命令、数值**均全量保留。
