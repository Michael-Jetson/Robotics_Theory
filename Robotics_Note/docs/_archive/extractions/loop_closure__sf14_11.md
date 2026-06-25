# 抽取留痕：《视觉SLAM十四讲》第11讲 回环检测

> 本文件是项目内部「抽取留痕」，目标是把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章。**禁摘要、禁凝练**：每一步推导、每一道例题/数值例、每一条定义、每一段代码/伪码、每一张表均完整记录。
>
> **源文件**：`/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/11_回环检测.md`（共 665 行，已完整读取）
> **源章节**：《视觉SLAM十四讲》（第二版，高翔等）第 11 讲「回环检测」（Loop Closure / Loop Closing）
> **⚠ OCR 注意**：本源由图像 OCR 得到，必有识别错。抽取时已用领域知识重建正确公式/文字并交叉核对；所有发现并修正的 OCR 错列于文末「OCR 修正说明」，正文中关键修正处就地用 `[OCR?]` 或 `[OCR 修正：…]` 标注。

---

## 0. 本章定位与范围说明（给综合 agent 的提示）

本第 11 讲是《十四讲》中**回环检测**模块的完整章节，覆盖了【本章聚焦】清单中的几乎全部主题：

| 【本章聚焦】要点 | 源对应小节 | 本文是否覆盖 |
|---|---|---|
| 回环检测的作用（消累积漂移、给位姿图加约束） | §11.1.1 | ✅ 完整 |
| 视觉词袋完整（词典构建、TF-IDF、相似度评分） | §11.2 / §11.3 / §11.4 | ✅ 完整（含 K-means、k 叉树字典、TF-IDF 三式、L1 评分式） |
| 关键帧/相似度处理与时间一致性 | §11.5.2 / §11.5.3 | ✅ 完整 |
| 几何验证 | §11.5.4 | ✅ 完整（时间一致性 + 空间一致性两法） |
| 感知混叠与准确率/召回率 | §11.1.2 末 / §11.1.3 | ✅ 完整（感知偏差/感知变异、P-R 定义、P-R 曲线、TP/FP/FN/TN 表） |
| 与位姿图后端（ch:nlopt）接口 | §11.1.1 / §11.5.4 | ✅ 概念层面（"回环边给位姿图加约束"、空间一致性验证把运动放回位姿图）。**注意**：本源不含位姿图优化的数学推导，那属于《十四讲》第 10 讲「后端 2 / 位姿图」。本书位姿图后端（ch:nlopt）的数学需另从第 10 讲或非线性优化章抽取。 |
| 深度学习地点识别简介 | §11.5.5 | ✅ 完整（VLAD/CNN、PoseNet 类回归位姿、参考文献 [113-117]） |

**实践代码**：本源含两段实践程序——`feature_training.cpp`（训练 ORB 字典）与 `loop_closure.cpp`（计算相似度/数据库查询），均基于 **DBoW3** 库。本文逐字记录（含 OCR 修正后的可编译版本）。

**本源未含、需别处补的内容**（给综合 agent）：
- 位姿图（Pose Graph）的图优化数学（节点=位姿、边=相对位姿约束、g2o/Ceres 求解）——属第 10 讲；
- 回环成功后的**全局 BA / 位姿图优化的具体目标函数与求解**——本章只到"加一条边"的概念为止；
- FabMap / Chow-Liu tree 的具体概率推导——本源仅提及名字与参考文献，留作习题（习题 5）。

---

## 记号约定（本源《十四讲》第 11 讲 vs 本书统一约定）

| 项目 | 本源记号 | 含义 | 本书统一约定 | 差异/转换提示 |
|---|---|---|---|---|
| 图像/帧 | $A,B$；关键帧 $x_1,x_2,\dots$ | 待比较的两幅图像；位姿图节点 | 同 | 一致 |
| 位姿节点 | $x_1,\dots,x_{100}$ | 位姿图（pose graph）中的位姿顶点 | 本书位姿用 $T\in SE(3)$ | 本源用 $x_i$ 抽象指代位姿节点，未写成 $SE(3)$ 元素；综合到位姿图章时对应 $T_i\in SE(3)$ |
| 相似度评分 | $s(A,B)$、$s(\boldsymbol a,\boldsymbol b)$、$s(\boldsymbol v_A,\boldsymbol v_B)$ | 两图像/两描述向量的相似性评分 | 同 | 一致 |
| 描述向量 | $\boldsymbol a,\boldsymbol b\in\mathbb R^{W}$；$\boldsymbol v_A$ | 词袋（BoW）描述向量 | 同 | $W$=字典单词数（向量维度）；$\boldsymbol v_A$ 为稀疏向量 |
| 单词 | $w_1,w_2,\dots,w_W$ / $w_i$ / $w_j$ | 字典中的单词（Word） | 同 | 一致 |
| 字典容量 | $W$（单词总数）；$k$（树分叉）；$d$ 或 $L$（树深度） | k 叉树字典参数 | 同 | ⚠ 源中树深度同时用 $d$（§11.3.1 正文）与 $L$（DBoW3 输出、§11.3.2）两个符号，二者同义。DBoW3 输出用 `L`。 |
| 聚类中心 | $c_1,\dots,c_k$ | K-means 的 $k$ 个中心点 | 同 | 一致 |
| 特征数 | $N$（图像特征总数 / 数据点数）；$n$（总特征数）；$n_i$（落入单词 $w_i$ 的特征数） | 统计 TF-IDF 用 | 同 | ⚠ $N$ 在不同处含义略变（一会儿是"提取的特征点数"、一会儿是"BoW 中单词条目数"、一会儿是可能回环数）；TF-IDF 公式里区分 $n$（总数）与 $n_i$（某单词的数）。详见 §11.4.1 逐式注解。 |
| 权重 | $\eta_i$ | 单词 $w_i$ 的 TF-IDF 权重 | 同 | 一致；$\eta_i=\mathrm{TF}_i\times\mathrm{IDF}_i$ |
| 准确率/召回率 | Precision / Recall；TP/FP/FN/TN | 评价指标 | 同 | 一致 |
| 时间下标 | $\boldsymbol v_t,\ \boldsymbol v_{t-\Delta t},\ \boldsymbol v_{t_j}$ | 当前帧、上一关键帧、历史关键帧的 BoW | 同 | 一致 |
| 范数 | $\|\cdot\|$、$\|\cdot\|_1$ | 通用范数 / $L_1$ 范数 | 同 | ⚠ OCR 多处把范数双竖线 $\|\ \|$ 丢成单竖线或漏掉，已重建；§11.2 式(11.4) 明确取 $L_1$ |
| 旋转/四元数/协方差 | 本章未使用 $R\in SO(3)$、四元数、$\Sigma_w/\Sigma_v$ 等 | — | — | 本章为外观式回环检测，**不涉及**李群/李代数/四元数/概率协方差，无需做右扰动/Hamilton 对齐 |

**术语对照（中英）**：
- 回环检测 = Loop Closure / Loop Closing / Loop Detection；重定位 = Relocalization。
- 词袋 = Bag-of-Words (BoW)；单词 = Word；字典 = Dictionary / Vocabulary。
- 聚类 = Clustering；无监督机器学习 = Unsupervised Machine Learning；K 均值 = K-means。
- TF-IDF = Term Frequency–Inverse Document Frequency（词频–逆文档频率）。
- 准确率 = Precision；召回率 = Recall；P-R 曲线 = Precision-Recall curve。
- 真/假阳性 = True/False Positive (TP/FP)；真/假阴性 = True/False Negative (TN/FN)。
- 感知偏差 = Perceptual Aliasing（即假阳性 FP；又译"感知混叠"）；感知变异 = Perceptual Variability（即假阴性 FN）。
- 基于里程计 = Odometry based；基于外观 = Appearance based。
- 几何验证 = Geometric verification；时间一致性 = temporal consistency；空间一致性 = spatial consistency。
- VLAD = Vector of Locally Aggregated Descriptors；CNN = 卷积神经网络。

---

# 第 11 讲 回环检测（正文逐节抽取）

## 主要目标（源 §"主要目标"）

1. 理解回环检测的必要性。
2. 掌握基于词袋（Bag-of-Words）的外观式回环检测。
3. 通过 DBoW3 的实验，学习词袋模型的实际用途。

**本讲引言（源开篇）**：回环检测是 SLAM 中另一个主要模块。SLAM 主体（前端、后端）的主要目的在于估计相机运动，而回环检测模块无论目标还是方法都与前面内容相差较大，所以通常被认为是一个**独立的模块**。本讲介绍主流视觉 SLAM 中检测回环的方式——**词袋模型**——并通过 DBoW 库上的程序实验得到更直观的理解。

---

## 11.1 概述

### 11.1.1 回环检测的意义（源 §11.1.1）

**问题背景**：前端提供特征点的提取和轨迹、地图的初值；后端负责对这些数据进行优化。但若像视觉里程计那样**仅考虑相邻时间上的关键帧**，则之前产生的误差不可避免地累积到下一时刻，使整个 SLAM 出现**累积误差（accumulated drift）**，长期估计结果不可靠——无法构建**全局一致（globally consistent）**的轨迹和地图。

**自动驾驶建图例子（源逐字）**：在自动驾驶建图阶段，通常指定采集车在某给定区域绕若干圈以覆盖所有采集范围。假设前端提取了特征，然后忽略特征点，在后端使用**位姿图（pose graph）优化**整个轨迹（图 11-1(a)）。前端给出的只是**局部的位姿间约束**，例如可能是
$$
x_1 - x_2,\quad x_2 - x_3,\quad \dots
$$
（即相邻关键帧之间的相对位姿约束）。但是：
- $x_1$ 的估计存在误差；
- $x_2$ 是根据 $x_1$ 决定的；
- $x_3$ 又是由 $x_2$ 决定的；
- 依此类推 —— **误差被逐级累积**，使后端优化结果（图 11-1(b)）慢慢趋向不准确。

在这种应用场景下，应保证：优化的轨迹和实际地点一致；当实际经过同一地点时，估计轨迹也必定经过同一点。

**图 11-1（漂移示意图，源图注逐字）**：
- (a) 真实轨迹；
- (b) 由于前端只给出相邻帧间的估计，优化后的位姿图出现**漂移**；
- (c) 添加回环检测后的位姿图**可以消除累积误差**（见彩插）。

**回环约束的来源与本质（源逐字）**：虽然后端能够估计最大后验误差，但"好模型架不住烂数据"——只有相邻关键帧数据时能做的事并不多，无从消除累积误差。而回环检测模块能给出**除相邻帧外、时隔更久远的约束**，例如 $x_1\sim x_{100}$ 之间的位姿变换。之所以它们之间会有约束，是因为我们**察觉到相机经过了同一个地方，采集到了相似的数据**。

> **回环检测的关键**：如何有效地检测出"相机经过同一个地方"这件事。若能成功检测，就可以为后端位姿图提供更多有效数据，得到更好的、**全局一致**的估计。

**质点–弹簧系统类比（源逐字，重要直观）**：由于位姿图可以看成一个**质点–弹簧系统**，所以回环检测相当于在图中加入了**额外的弹簧**，提高了系统稳定性。可直观想象成：**回环边把带有累积误差的边"拉"到了正确的位置**——前提是回环本身正确。

**回环检测的两大意义（源逐字）**：
1. 关系到估计的轨迹和地图在**长时间下的正确性**（全局一致性）。
2. 提供了**当前数据与所有历史数据的关联**，因此可用于**重定位（relocalization）**。重定位用处更多：例如事先对某场景录制一条轨迹并建立地图，之后在该场景中可一直跟随这条轨迹导航，重定位帮助确定自身在轨迹上的位置。

> **VO 与 SLAM 的划界（源逐字）**：甚至在某些时候，我们把**仅有前端和局部后端**的系统称为**视觉里程计（VO）**，而把**带有回环检测和全局后端**的系统称为 **SLAM**。

---

### 11.1.2 回环检测的方法（源 §11.1.2）

**朴素思路 1：暴力两两匹配**。对任意两幅图像都做一遍特征匹配，根据正确匹配数量确定哪两幅图像存在关联。
- 优点：朴素且有效。
- 缺点：盲目假设"任意两幅图像都可能存在回环"，检测数量太大。对 $N$ 个可能的回环要检测
$$
C_N^2 = \binom{N}{2} = \frac{N(N-1)}{2}
$$
那么多次，这是 $O(N^2)$ 的复杂度，随轨迹变长增长太快，在大多数实时系统中**不实用**。
> [OCR 修正：源作 $C_{N}^{2}$，即组合数 $\binom{N}{2}$，复杂度 $O(N^2)$，OCR 正确。]

**朴素思路 2：随机抽取**。随机抽取历史数据进行回环检测，例如在 $n$ 帧中随机抽 5 帧与当前帧比较。
- 优点：能维持**常数时间**的运算量。
- 缺点：盲目试探，在帧数 $N$ 增长时**抽到回环的概率大幅下降**，检测效率不高。（注：源指出随机检测在某些实现中确实有用，参考文献 [97]。）

> 上述朴素思路都过于粗糙。我们至少希望有一个"哪处可能出现回环"的预计，才好不那么盲目地检测。这样的方式大体有**两种思路**：

**思路 A：基于里程计（Odometry based）的几何关系**（参考文献 [98]）。当发现当前相机运动到了之前某个位置附近时，检测它们有没有回环关系。
- 直观，但有**逻辑缺陷**：由于累积误差的存在，往往没法正确发现"运动到了之前某个位置附近"这件事实，回环检测也就无从谈起。
- 这是**倒果为因**：回环检测的目标本是发现"相机回到之前位置"从而消除累积误差；而基于里程计的做法却**假设了**"相机回到之前位置附近"才能检测回环。因而在累积误差较大时无法工作（参考文献 [12]）。

**思路 B：基于外观（Appearance based）的几何关系**。与前端、后端的估计**都无关**，仅根据两幅图像的**相似性**确定回环关系。
- 优点：**摆脱了累积误差**，使回环检测成为 SLAM 系统中一个相对独立的模块（前端可为它提供特征点）。
- 自 21 世纪初被提出以来，基于外观的回环检测能有效地在不同场景下工作，成了视觉 SLAM 的**主流做法**，并被应用于实际系统（参考文献 [88, 95, 99]）。

**工程角度的补充思路**：室外无人车通常配备 **GPS**，可提供全局位置信息，很容易判断汽车是否回到某经过的点；但这类方法在**室内**不好用。

**核心问题（源逐字）**：在基于外观的回环检测算法中，核心问题是**如何计算图像间的相似性**。对图像 $A$ 和图像 $B$，设计一种方法计算相似性评分 $s(A,B)$。评分在某区间内取值，大于一定量后认为出现回环。

**为什么不能直接图像相减？（源 式 11.1）**

一个直观但错误的想法：图像可表示成矩阵，直接两幅图像相减取范数：
$$
s(\boldsymbol A,\boldsymbol B)=\|\boldsymbol A-\boldsymbol B\|. \tag{11.1}
$$
> [OCR 修正：源式范数双竖线被 OCR 部分丢失，已恢复为 $\|\boldsymbol A-\boldsymbol B\|$。]

**为什么不这样做（两条原因，源逐字）**：
1. **像素灰度是不稳定的测量值**，严重受环境光照和相机曝光影响。假设相机未动、打开一支电灯，图像会整体变亮；即使对同样的数据也会得到很大差异值。
2. 当相机**视角发生少量变化**时，即使每个物体的光度不变，它们的像素也会在图像中发生位移，造成很大的差异值。

由于这两种情况，实际中即使对非常相似的图像，$\boldsymbol A-\boldsymbol B$ 也经常得到一个（不符合实际的）很大的值。所以这个函数不能很好地反映图像间的相似关系。由此引出两个概念：**感知偏差（Perceptual Aliasing）**和**感知变异（Perceptual Variability）**。

---

### 11.1.3 准确率和召回率（源 §11.1.3）

**判断框架（源逐字）**：从人类角度，我们能以很高精确度感觉到"两幅图像是否相似""这两张照片是否从同一地方拍摄"，但尚不掌握人脑工作原理。从程序角度，希望算法得出与人类（或事实）一致的判断：事实是回环时算法应给"是回环"；事实不是回环时算法应给"不是回环"。但程序判断并不总与人类一致，于是出现 4 种情况（表 11-1）。

#### 表 11-1 回环检测的结果分类（源逐字重建）

| 算法\事实 | **是回环**（事实为回环） | **不是回环**（事实非回环） |
|---|---|---|
| **是回环**（算法判为回环） | 真阳性 True Positive (TP) | 假阳性 False Positive (FP) |
| **不是回环**（算法判为非回环） | 假阴性 False Negative (FN) | 真阴性 True Negative (TN) |

> 表头与体内文字源逐字为：行头"是回环 / 不是回环"，列头"是回环 / 不是回环"，四格依次为"真阳性 / 假阳性 / 假阴性 / 真阴性"。表布局对应：**行=算法判断，列=事实**。

**阴性/阳性术语（源逐字）**：借用医学说法。
- **假阳性（False Positive, FP）= 感知偏差（Perceptual Aliasing）**。
- **假阴性（False Negative, FN）= 感知变异（Perceptual Variability）**。
- 缩写：TP=True Positive（真阳性），TN=True Negative（真阴性），余类推。
- 我们希望算法与人类判断一致，故希望 **TP、TN 尽量高，FP、FN 尽可能低**。

可统计某算法在某数据集上 TP、TN、FP、FN 的出现次数，并计算两个统计量——**准确率（Precision）**和**召回率（Recall）**：

$$
\boxed{\ \text{Precision}=\dfrac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FP}},\qquad \text{Recall}=\dfrac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FN}}.\ } \tag{11.2}
$$

**图 11-2（假阳性与假阴性例子，源图注逐字）**：左侧为假阳性——两幅图像看起来很像，但**并非同一走廊**；右侧为假阴性——由于光照变化，**同一地点不同时刻**的照片看起来很不一样。

**字面意义（源逐字）**：
- **准确率**：算法提取的所有回环中**确实是真实回环的概率**。
- **召回率**：在所有真实回环中**被正确检测出来的概率**。

**为什么取这两个量**：它们有代表性，并且**通常是一对矛盾**。一个算法往往有许多设置参数：
- 提高某阈值→算法更"严格"→检出更少回环→**准确率提高**，但漏掉原本是回环的→**召回率下降**。
- 更宽松配置→检出更多回环→**召回率提高**，但混杂非回环→**准确率下降**。

**P-R 曲线（源逐字）**：测试算法在各种配置下的 P、R 值，做 **Precision-Recall 曲线**（图 11-3）。以召回率为横轴、准确率为纵轴，关心：
- 整条曲线**偏向右上方**的程度；
- **100% 准确率下的召回率**；
- **50% 召回率时的准确率**。
作为评价指标。注意：除"天壤之别"的算法外，通常不能一概而论说算法 A 优于 B（可能 A 在准确率较高时仍有很好召回率，而 B 在 70% 召回率时还能保证较好准确率，诸如此类）。

**图 11-3 图注（源逐字，参考文献 [100]）**：准确率-召回率曲线的例子。随着召回率上升，检测条件变宽松，准确率随之下降。好的算法在较高召回率情况下仍能保证较好的准确率。

> **SLAM 中的取舍（源逐字，极重要的工程结论）**：在 SLAM 中，**对准确率的要求更高，对召回率相对宽容**。原因：
> - **假阳性**回环将在后端位姿图中添加**根本错误的边**，有时导致优化给出完全错误结果。（设想 SLAM 错误地把所有办公桌当成同一张：走廊不直了、墙壁交错、整个地图失效。）
> - **召回率低**顶多有部分回环没被检测到，地图受一些累积误差影响——**而仅需一两次回环就可以完全消除它们**。
>
> 所以选择回环检测算法时，倾向于**把参数设置得更严格**，或在检测之后**再加上回环验证步骤**（见 §11.5.4）。

**回到开头的问题**：为什么不用 $\boldsymbol A-\boldsymbol B$？因为其准确率和召回率都很差，可能出现大量假阳性或假阴性，所以"不好"。下面引出更好的方法——词袋模型。

---

## 11.2 词袋模型（源 §11.2）

**引入（源逐字）**：直接图像相减不够好。一种思路：像视觉里程计那样**使用特征点**做回环检测——对两幅图像的特征点匹配，只要匹配数量大于一定值就认为出现回环；据匹配还能算出两图像间运动关系。但此法有问题：特征匹配费时、光照变化时特征描述可能不稳定等。词袋模型与此很相近。

**词袋（Bag-of-Words, BoW）定义（源逐字）**：目的是用"**图像上有哪几种特征**"来描述一幅图像。例如某照片中有一个人、一辆车；另一张中有两个人、一只狗。据此度量两图像相似性。具体三步：

1. 确定"人""车""狗"等概念——对应 BoW 中的**单词（Word）**；许多单词放在一起组成**字典（Dictionary）**。
2. 确定一幅图像中出现了哪些字典中定义的概念——用单词出现的情况（**直方图**）描述整幅图像，把图像转换成一个**向量**。
3. 比较上一步描述的相似程度。

**字典向量化例子（源 式 11.3）**：设字典记录单词 $w_1,w_2,w_3$（"人""车""狗"）。对任意图像 $A$，按其含有的单词记为
$$
A = 1\cdot w_1 + 1\cdot w_2 + 0\cdot w_3. \tag{11.3}
$$
字典固定，所以只要用向量 $[1,1,0]^{\mathrm T}$ 就可表达 $A$ 的意义。

**BoW 关键性质（源逐字）**：该向量描述"图像是否含有某类特征"，比单纯灰度值更稳定。又因为它说的是"**是否出现**"而不管"在哪儿出现"，所以**与物体的空间位置和排列顺序无关**；相机发生少量运动时，只要物体仍在视野中，描述向量就不变。正因强调 Words 的**有无**而无关其**顺序**，才称 **Bag**-of-Words 而非 List-of-Words；字典类似单词的一个**集合**。

**相似度计算（源 式 11.4）**：同理用 $[2,0,1]^{\mathrm T}$ 描述图像 $B$（若只考虑"是否出现"则为二值向量 $[1,0,1]^{\mathrm T}$）。对两个向量求差有多种做法。对 $\boldsymbol a,\boldsymbol b\in\mathbb R^{W}$ 可计算：

$$
\boxed{\ s(\boldsymbol a,\boldsymbol b)=1-\frac{1}{W}\,\|\boldsymbol a-\boldsymbol b\|_1.\ } \tag{11.4}
$$

其中范数取 **$L_1$ 范数**（各元素绝对值之和），$W$ 为字典单词数（向量维度）。
> [OCR 修正：源式范数被 OCR 写作 $\| \cdot \|_1$ 但双竖线/下标在多处丢失，已恢复。]

**式(11.4) 边界值（源逐字）**：两向量完全一样时得 $1$；完全相反（$\boldsymbol a$ 为 0 处 $\boldsymbol b$ 为 1）时得 $0$。这样就定义了两描述向量的相似性，也即图像间的相似程度。

**接下来两个问题（源逐字）**：
1. 字典的定义方式清楚了，但它**到底怎么来的**？（→ §11.3）
2. 若能算两图像间相似程度评分，**是否就足够判断回环**？（→ §11.4、§11.5）

---

## 11.3 字典（源 §11.3）

### 11.3.1 字典的结构（源 §11.3.1）

**字典生成 = 聚类问题（源逐字）**：字典由很多单词组成，每个单词代表一个概念。一个单词与单个特征点不同——它不是从单幅图像提取的，而是**某一类特征的组合**。所以字典生成问题类似一个**聚类（Clustering）问题**。

聚类在**无监督机器学习（Unsupervised ML）**中常见。设对大量图像提取了 $N$ 个特征点，想找一个有 $k$ 个单词的字典（每个单词看作局部相邻特征点的集合），可用经典的 **K-means（K 均值）算法**（参考文献 [101]）。

#### 算法 11.A：K-means（K 均值聚类，源逐字四步）

> 输入：$N$ 个数据点；目标类数 $k$。

1. **随机选取 $k$ 个中心点**：$c_1,\dots,c_k$。
2. 对每一个样本，**计算它与每个中心点之间的距离**，取最小的作为它的归类。
3. **重新计算每个类的中心点**。
4. 若每个中心点都**变化很小**，则算法**收敛、退出**；否则**返回第 2 步**。

**K-means 的不足（源逐字）**：朴素简单有效，但存在问题——需指定聚类数量、随机选取中心点使每次结果都不同、以及一些效率问题。研究者开发了**层次聚类法、K-means++**（参考文献 [102]）等以弥补。

**从 O(n) 查找到对数级查找（源逐字）**：据 K-means 把大量特征点聚成含 $k$ 个单词的字典后，问题变成：如何根据图像中某特征点**查找字典中相应的单词**？
- 朴素思想：与每个单词比对取最相似——简单有效。
- 但考虑**字典通用性**，通常用大规模字典（保证当前环境图像特征都曾在字典出现或有相近表达）；对一万、十万个单词逐一比较，$O(n)$ 查找不可取。
- 若字典排过序，**二分查找**可达**对数级别**复杂度。
- 实践中可用更复杂数据结构，如 **FabMap**（参考文献 [103-105]）中的 **Chow-Liu tree**（参考文献 [106]）等。
- 本书介绍另一种较简单实用的**树结构**（参考文献 [107]）。
> [OCR 修正：源把 "Chow-Liu tree" OCR 成 "Chou-Liu tree"，已修正为 Chow-Liu（习题 5 中源又作 "Chow-Liu"，前后印证）。"Fabmap" 通行写法为 FabMap。]

#### 算法 11.B：k 叉树字典（层次 K-means，源逐字三步，参考文献 [107]）

用一种 **$k$ 叉树**表达字典，思路类似层次聚类，是 K-means 的直接扩展（图 11-4）。假定有 $N$ 个特征点，希望构建一个**深度为 $d$、每次分叉为 $k$** 的树，做法如下：

1. 在**根节点**，用 K-means 把所有样本聚成 $k$ 类（实际中为保证聚类均匀性会使用 **K-means++**）。这样得到**第一层**。
2. 对**第一层每个节点**，把属于该节点的样本**再聚成 $k$ 类**，得到下一层。
3. 依此类推，最后得到**叶子层**。**叶子层即为所谓的 Words（单词）**。

**图 11-4 图注（源逐字）**：K 叉树字典示意图。训练字典时，逐层使用 K-means 聚类；根据已知特征查找单词时，可逐层比对，找到对应单词（见彩插）。

**容量与查找复杂度（源逐字，重要结论）**：
- 最终在**叶子层**构建单词，树中的**中间节点仅供快速查找**使用。
- 一个 **$k$ 分支、深度为 $d$** 的树，可容纳
$$
\boxed{\ k^{d}\ \text{个单词}.\ }
$$
- 查找某给定特征对应单词时，只需将它与每个中间节点的聚类中心比较（一共 $d$ 次），即可找到最后的单词，保证了**对数级别（$O(\log_k W)=O(d)$）的查找效率**。

---

### 11.3.2 实践：创建字典（源 §11.3.2）

**实验设置（源逐字）**：演示如何生成及使用 **ORB 字典**（前面 VO 部分大量使用 ORB 特征描述）。选取 **TUM 数据集中的 10 幅图像**（位于 `slambook2/ch11/data` 中，图 11-5），来自一组实际相机运动轨迹。**第一幅与最后一幅图像明显采自同一地方**——看算法能否检测到这个回环。

**图 11-5 图注（源逐字）**：演示实验中使用的 10 幅图像，采集自不同时刻的轨迹。

**字典规模说明（源逐字）**：实用字典往往在更大数据集上训练，数据应来自与目标环境类似的地方。通常用较大规模字典——越大单词量越丰富、越易找到对应单词，但不能大到超过计算能力和内存。本例暂从 10 幅图像训练一个小字典；追求更好效果应下载更多数据训练更大字典；也可用别人训练好的字典，但**注意字典使用的特征类型是否一致**。

**BoW 库（源逐字）**：使用 **DBoW3**：https://github.com/rmsalinas/DBow3 。也可从本书代码 `3rdparty` 文件夹找到，是一个 cmake 工程，按 cmake 流程编译安装。

#### 代码 11.1：`slambook2/ch11/feature_training.cpp`（训练字典）

> **⚠ OCR 严重错**：源把这段代码分裂成两个代码块（行 232–242 与 244–266），且首块 `main` 提前闭合、`detectAndCompute` 后出现乱码 `descriptors.push_backubes。`。下面给出**修复后可编译的完整版本**（按 DBoW3 官方示例与上下文重建），原始 OCR 乱码逐处标注。

```cpp
// slambook2/ch11/feature_training.cpp
int main(int argc, char **argv) {
    // read the image
    cout << "reading images... " << endl;
    vector<Mat> images;
    for (int i = 0; i < 10; i++) {
        string path = "./data/" + to_string(i + 1) + ".png";
        images.push_back(imread(path));
    }
    // detect ORB features
    cout << "detecting ORB features ... " << endl;
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for (Mat &image : images) {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);   // [OCR 原文乱码: "descriptors.push_backubes。"]
    }
    // create vocabulary
    cout << "creating vocabulary ... " << endl;
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);
    cout << "vocabulary info: " << vocab << endl;
    vocab.save("vocabulary.yml.gz");
    cout << "done" << endl;

    return 0;
}
```

> [OCR 修正记录（本段）：
> 1. 源行 241 处 `}` 提前关闭了 `main`，把单个函数错误地拆成两个代码块（一个标 `txt`、一个标 `cpp`），且第二块开头多出一个孤立 `}`。已合并为单一函数。
> 2. 源行 254 `descriptors.push_backubes。`（含中文句号）→ 应为 `descriptors.push_back(descriptor);`。]

**DBoW3 使用说明（源逐字）**：对 10 张目标图像提取 ORB 特征并存入 `vector` 容器，然后调用 DBoW3 的字典生成接口即可。`DBoW3::Vocabulary` 构造函数中能指定树的**分叉数量及深度**，这里用默认构造函数，即 **$k=10,\ d=5$**。这是一个小规模字典，**最大能容纳 $10^5 = 100{,}000$ 个单词**（$k^d=10^5$）。图像特征亦用默认参数，即**每幅图像 500 个特征点**。最后把字典存为压缩文件。

#### 终端输出 11.1（运行 `feature_training`，源逐字）

```text
$ build/feature_training
reading images...
detecting ORB features ...
creating vocabulary ...
vocabulary info: Vocabulary: k = 10, L = 5, Weighting = tf-idf, Scoring = L1-norm, Number of words = 4983
done
```

**输出解读（源逐字）**：分支数量 $k=10$，深度 $L=5$，单词数量 **4983**（未达最大容量 $10^5$）。剩下的 `Weighting`（权重）= **tf-idf**，`Scoring`（评分）= **L1-norm**——但评分如何计算？引出 §11.4。
> [注：源正文用 $d$ 表示树深度，DBoW3 输出用 `L`；二者同义，此处 $L=5=d$。]

---

## 11.4 相似度计算（源 §11.4）

### 11.4.1 理论部分（源 §11.4.1）

**从特征到直方图（源逐字）**：有了字典后，给定任意特征 $f_i$，在字典树中逐层查找即可找到对应单词 $w_j$——字典足够大时可认为 $f_i$ 和 $w_j$ 来自同一类物体（无理论保证，仅聚类意义下如此）。从一幅图像提取 $N$ 个特征、找到对应单词后，就拥有了该图像在单词列表中的**分布（直方图）**。理想情况相当于"这幅图里有一个人和一辆汽车"，即一个 **Bag**。

**为什么要加权（TF-IDF 动机，源逐字）**：上述做法对所有单词"一视同仁"（有就是有、没有就是没有）。但**部分单词具有更强区分性**：如"的""是"在许多句子中出现，无法据其判别句子类型；而"文档""足球"对判别作用更大、提供更多信息。所以希望对单词的**区分性/重要性**加以评估，给不同权值。

**TF-IDF 定义（源逐字，参考文献 [109, 110]）**：TF-IDF（Term Frequency–Inverse Document Frequency，词频–逆文档频率）是文本检索常用加权方式，也用于 BoW。
- **TF（词频）思想**：某单词在一幅图像中经常出现，则其区分度高。
- **IDF（逆文档频率）思想**：某单词在字典中出现的频率越低，分类图像时区分度越高。

**IDF（建立字典时计算，源 式 11.5）**：统计某叶子节点 $w_i$ 中的特征数量相对所有特征数量的比例。设**所有特征数量为 $n$，$w_i$ 中特征数量为 $n_i$**，则该单词的 IDF 为
$$
\boxed{\ \mathrm{IDF}_i = \log\frac{n}{n_i}.\ } \tag{11.5}
$$
> [逐符号注解：此处 $n$=训练字典时落入整棵树的特征总数，$n_i$=落入叶子 $w_i$ 的特征数；某单词越"罕见"（$n_i$ 越小），$\log(n/n_i)$ 越大，IDF 权重越高。对数底数源未明写，DBoW3 实现用自然对数 $\ln$。]

**TF（单幅图像内计算，源 式 11.6）**：某特征在单幅图像中出现的频率。设图像 $A$ 中单词 $w_i$ 出现 $n_i$ 次、图像中一共出现的单词次数为 $n$，则
$$
\boxed{\ \mathrm{TF}_i = \frac{n_i}{n}.\ } \tag{11.6}
$$
> [⚠ 符号复用警告：式(11.6) 的 $n$、$n_i$ 与式(11.5) 的**含义不同**！式(11.5) 中 $n,n_i$ 是**训练字典时的全局统计量**；式(11.6) 中 $n,n_i$ 是**针对当前单幅图像 $A$ 的局部统计量**（$n$=该图所有单词出现总次数，$n_i$=单词 $w_i$ 在该图出现次数）。源沿用同字母，综合写书时建议改名以免混淆，例如 IDF 用 $N,N_i$、TF 用 $n_A,n_{A,i}$。OCR 未引入额外错误，但符号本身是源的歧义点。]

**权重（源 式 11.7）**：$w_i$ 的权重等于 TF 乘 IDF 之积：
$$
\boxed{\ \eta_i = \mathrm{TF}_i \times \mathrm{IDF}_i.\ } \tag{11.7}
$$

**图像的 BoW 向量（源 式 11.8）**：考虑权重后，图像 $A$ 的特征点对应到许多单词，组成它的 BoW：
$$
\boxed{\ A = \{(w_1,\eta_1),(w_2,\eta_2),\dots,(w_N,\eta_N)\}\ \stackrel{\mathrm{def}}{=}\ \boldsymbol v_A.\ } \tag{11.8}
$$
**稀疏性（源逐字）**：相似特征可能落到同一类，故实际 $\boldsymbol v_A$ 中存在**大量的零**。$\boldsymbol v_A$ 是一个**稀疏向量**，非零部分指示图像 $A$ 含有哪些单词，其值为 TF-IDF 值。

**相似度评分（源 式 11.9，⚠ OCR 关键错误）**：给定 $\boldsymbol v_A$ 和 $\boldsymbol v_B$，如何计算差异？与范数定义一样有多种方式。参考文献 [111] 的 $L_1$ 范数形式：

源 OCR 原样（**有误**）：
$$
s(\boldsymbol v_A - \boldsymbol v_B) = 2\sum_{i=1}^{N} |\boldsymbol v_{Ai}| + |\boldsymbol v_{Bi}| - |\boldsymbol v_{Ai}-\boldsymbol v_{Bi}|. \tag{11.9-OCR}
$$

**领域知识重建（正确形式）**——这是 DBoW2/DBoW3 论文（Gálvez-López & Tardós, 参考文献 [111]）中的标准 $L_1$ 评分公式，正确写法为：
$$
\boxed{\ s(\boldsymbol v_A,\boldsymbol v_B)=1-\frac{1}{2}\,\big\|\,\widehat{\boldsymbol v}_A-\widehat{\boldsymbol v}_B\,\big\|_1\ }
$$
其展开等价形式（仅对两向量都非零的分量 $i$ 求和才有贡献）为：
$$
s(\boldsymbol v_A,\boldsymbol v_B)=\frac{1}{2}\sum_{i}\Big(\,|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|\,\Big).
$$
> [OCR 修正（极重要）：源式(11.9) 至少有三处错误：
> (1) 左端写成 $s(\boldsymbol v_A-\boldsymbol v_B)$（把两个参数误并成一个差），应为二元函数 $s(\boldsymbol v_A,\boldsymbol v_B)$；
> (2) 系数 OCR 作 "$2\sum\dots$" 实应为 "$\tfrac12\sum\dots$"（DBoW2 论文式中分数为 $1/2$，OCR 把 $\frac12$ 误认成 $2$ 或漏掉分母）；
> (3) 求和号内缺少把三项括在一起的括号，导致 $|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|$ 的结合关系被破坏。
> 重建依据：Gálvez-López & Tardós, "Bags of Binary Words for Fast Place Recognition in Image Sequences", IEEE T-RO 2012 中给出 $s(v_1,v_2)=1-\tfrac12\big|\tfrac{v_1}{|v_1|}-\tfrac{v_2}{|v_2|}\big|$，展开即上式。注意该评分在两向量都已 $L_1$ 归一化（$\|\widehat{\boldsymbol v}\|_1=1$）时，结果落在 $[0,1]$，完全相同得 1。**综合写书时务必采用重建后的正确式，切勿照抄 (11.9-OCR)。**]

至此说明了如何通过词袋模型计算任意图像间的相似度。

---

### 11.4.2 实践：相似度的计算（源 §11.4.2）

**任务（源逐字）**：用 §11.3 生成的字典生成词袋并比较差异。

#### 代码 11.2：`slambook2/ch11/loop_closure.cpp`（计算相似度 / 数据库查询）

> [OCR 修正：源文件路径标作 `slambook/ch12/loop_closure.cpp`（行 334），与全书一致的正确路径应为 **`slambook2/ch11/loop_closure.cpp`**（第二版、第 11 章）。源又把代码分裂成三个块（337–356、358–398），且 `vector(Mat>` 括号错、`descriptors.push_backubesor);` 乱码。下面给出修复后完整版本。]

```cpp
// slambook2/ch11/loop_closure.cpp
int main(int argc, char **argv) {
    // read the images and database
    cout << "reading database" << endl;
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
    // DBoW3::Vocabulary vocab("./vocab_larger.yml.gz");  // use large vocab if you want
    if (vocab.empty()) {
        cerr << "Vocabulary does not exist." << endl;
        return 1;
    }
    cout << "reading images... " << endl;
    vector<Mat> images;
    for (int i = 0; i < 10; i++) {
        string path = "./data/" + to_string(i + 1) + ".png";
        images.push_back(imread(path));
    }

    // NOTE: in this case we are comparing images with a vocabulary generated by themselves,
    // this may lead to overfit.

    // detect ORB features
    cout << "detecting ORB features ... " << endl;
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;                       // [OCR 原文: "vector(Mat> descriptors;"]
    for (Mat &image : images) {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);         // [OCR 原文: "descriptors.push_backubesor);"]
    }

    // we can compare the images directly or we can compare one image to a database
    // images :
    cout << "comparing images with images " << endl;
    for (int i = 0; i < images.size(); i++) {
        DBoW3::BowVector v1;
        vocab.transform(descriptors[i], v1);
        for (int j = i; j < images.size(); j++) {
            DBoW3::BowVector v2;
            vocab.transform(descriptors[j], v2);
            double score = vocab.score(v1, v2);
            cout << "image " << i << " vs image " << j << " : " << score << endl;
        }
        cout << endl;
    }

    // or with database
    cout << "comparing images with database " << endl;
    DBoW3::Database db(vocab, false, 0);
    for (int i = 0; i < descriptors.size(); i++)
        db.add(descriptors[i]);
    cout << "database info: " << db << endl;
    for (int i = 0; i < descriptors.size(); i++) {
        DBoW3::QueryResults ret;
        db.query(descriptors[i], ret, 4);   // max result = 4
        cout << "searching for image " << i << " returns " << ret << endl << endl;
    }
    cout << "done." << endl;
}
```

> [OCR 修正记录（本段）：
> 1. 行 362 `vector(Mat> descriptors;` → `vector<Mat> descriptors;`（左尖括号被 OCR 成左圆括号）。
> 2. 行 367 `descriptors.push_backubesor);` → `descriptors.push_back(descriptor);`。
> 3. 行 354–355 注释 `// this may lead to overfit.` 跨行，已并入注释。]

**两种比对方式（源逐字）**：(1) 图像之间直接比较；(2) 图像与数据库之间比较——二者大同小异。

#### 终端输出 11.2a（BoW 描述向量，源逐字）

> [OCR 修正：终端首行源作 `$ build/feature_training`，但本程序是 `loop_closure`，应为 `$ build/loop_closure`。]

```text
$ build/loop_closure
reading database
reading images...
detecting ORB features ...
comparing images with images
desp 0 size: 500
transform image 0 into BoW vector: size = 455
key value pair = <1, 0.00155622>, <3, 0.00222645>, <12, 0.00222645>, <13, 0.00222645>,
<14, 0.00222645>, <22, 0.00222645>, <33, 0.00222645>, <37, 0.00155622>, <38, 0.00222645>,
<39, 0.00222645>, <43, 0.00222645>, <57, 0.00155622> .....
```

**解读（源逐字）**：BoW 描述向量含每个单词的 **ID 和权重**，构成稀疏向量。比较两向量时 DBoW3 计算一个分数，方式由构造字典时定义（即 tf-idf 加权 + L1-norm 评分）。

#### 终端输出 11.2b（image 0 与各图相似度，源逐字）

```text
image 0 vs image 0 : 1
image 0 vs image 1 : 0.0234552
image 0 vs image 2 : 0.0225237
image 0 vs image 3 : 0.0254611
image 0 vs image 4 : 0.0253451
image 0 vs image 5 : 0.0272257
image 0 vs image 6 : 0.0217745
image 0 vs image 7 : 0.0231948
image 0 vs image 8 : 0.0311284
image 0 vs image 9 : 0.0525447
```

#### 终端输出 11.2c（数据库查询，排序结果，源逐字全量）

```text
searching for image 0 returns 4 results:
<EntryId: 0, Score: 1>
<EntryId: 9, Score: 0.0525447>
<EntryId: 8, Score: 0.0311284>
<EntryId: 5, Score: 0.0272257>

searching for image 1 returns 4 results:
<EntryId: 1, Score: 1>
<EntryId: 2, Score: 0.0339641>
<EntryId: 8, Score: 0.0299387>
<EntryId: 3, Score: 0.0256668>

searching for image 2 returns 4 results:
<EntryId: 2, Score: 1>
<EntryId: 7, Score: 0.036092>
<EntryId: 9, Score: 0.0348702>
<EntryId: 1, Score: 0.0339641>

searching for image 3 returns 4 results:
<EntryId: 3, Score: 1>
<EntryId: 9, Score: 0.0357317>
<EntryId: 8, Score: 0.0278496>
<EntryId: 5, Score: 0.0270168>

searching for image 4 returns 4 results:
<EntryId: 4, Score: 1>
<EntryId: 5, Score: 0.0493492>
<EntryId: 0, Score: 0.0253451>
<EntryId: 6, Score: 0.0253017>

searching for image 5 returns 4 results:
<EntryId: 5, Score: 1>
<EntryId: 4, Score: 0.0493492>
<EntryId: 9, Score: 0.028996>
<EntryId: 6, Score: 0.0277584>

searching for image 6 returns 4 results:
<EntryId: 6, Score: 1>
<EntryId: 8, Score: 0.0306241>
<EntryId: 5, Score: 0.0277584>
<EntryId: 3, Score: 0.0267135>

searching for image 7 returns 4 results:
<EntryId: 7, Score: 1>
<EntryId: 2, Score: 0.036092>
<EntryId: 1, Score: 0.0239091>
<EntryId: 0, Score: 0.0231948>

searching for image 8 returns 4 results:
<EntryId: 8, Score: 1>
<EntryId: 9, Score: 0.0329149>
<EntryId: 0, Score: 0.0311284>
<EntryId: 6, Score: 0.0306241>

searching for image 9 returns 4 results:
<EntryId: 9, Score: 1>
<EntryId: 0, Score: 0.0525447>
<EntryId: 3, Score: 0.0357317>
<EntryId: 2, Score: 0.0348702>
```

**实验结论（源逐字）**：明显相似的图 1 和图 10（C++ 下标分别为 **0 和 9**），相似度评分约 **0.0525**；其他图像约 **0.02**。
- 从人类角度，我们认为图 1 和图 10 至少有百分之七八十相似度，其他图可能百分之二三十。
- 但实验结果是：无关图像相似度约 **2%**，相似图像约 **5%**——**没有想象的那么明显**。这是否是想要的结果？（→ §11.5 分析）

---

## 11.5 实验分析与评述（源 §11.5）

### 11.5.1 增加字典规模（源 §11.5.1）

**动机（源逐字）**：在机器学习领域，代码没错而结果不满意，首先怀疑"网络结构是否够大、层数是否够深、数据样本是否够多"——出于"好模型敌不过烂数据"的原则。这里首先怀疑：**字典是不是太小了**？（毕竟只从 10 幅图生成。）

**更大字典（源逐字）**：`slambook2/ch11/vocab_larger.yml.gz` 是一个稍大的字典——实际是对**同一数据序列的所有图像**（约 **2,900 幅**）生成的。规模仍取 **$k=10,\ d=5$**（最多一万个单词 [OCR? 见下注]）。可用同目录 `gen_vocab_large.cpp` 自行训练；训练大字典可能需要内存较大的机器并耐心等待。对 §11.4 程序稍加修改使用更大字典。
> [OCR? / 内部矛盾：源此处说"最多一万个单词"，但 $k=10,d=5\Rightarrow k^d=10^5=100{,}000$，与 §11.3.2 的"最大 100,000"一致，且下方终端输出实测 **Number of words = 99566**（≈10 万）。故"最多一万个单词"应为"最多十万个单词"，疑为源笔误或 OCR 把"十万"误作"一万"。综合写书时按 **$10^5=100{,}000$** 为准。]
> [OCR 修正：源行 519 引用"我们对 **10.4 节** 的程序稍加修改"，正确应为 **§11.4 节**（章号错印成 10）。]

#### 终端输出 11.3（更大字典的数据库查询，源逐字全量）

```text
comparing images with database
database info: Database: Entries = 10, Using direct index = no. Vocabulary: k = 10, L = 5,
Weighting = tf-idf, Scoring = L1-norm, Number of words = 99566
searching for image 0 returns 4 results:
<EntryId: 0, Score: 1>
<EntryId: 9, Score: 0.0320906>
<EntryId: 8, Score: 0.0103268>
<EntryId: 4, Score: 0.0066729>

searching for image 1 returns 4 results:
<EntryId: 1, Score: 1>
<EntryId: 2, Score: 0.0238409>
<EntryId: 8, Score: 0.00814409>
<EntryId: 3, Score: 0.00697527>

searching for image 2 returns 4 results:
<EntryId: 2, Score: 1>
<EntryId: 1, Score: 0.0238409>
<EntryId: 5, Score: 0.00897928>
<EntryId: 8, Score: 0.00893477>

searching for image 3 returns 4 results:
<EntryId: 3, Score: 1>
<EntryId: 5, Score: 0.0107005>
<EntryId: 8, Score: 0.00870392>
<EntryId: 6, Score: 0.00720695>

searching for image 4 returns 4 results:
<EntryId: 4, Score: 1>
<EntryId: 6, Score: 0.0069998>
<EntryId: 0, Score: 0.0066729>
<EntryId: 5, Score: 0.0062834>

searching for image 5 returns 4 results:
<EntryId: 5, Score: 1>
<EntryId: 3, Score: 0.0107005>
<EntryId: 2, Score: 0.00897928>
<EntryId: 4, Score: 0.0062834>

searching for image 6 returns 4 results:
<EntryId: 6, Score: 1>
<EntryId: 7, Score: 0.00915307>
<EntryId: 3, Score: 0.00720695>
<EntryId: 4, Score: 0.0069998>

searching for image 7 returns 4 results:
<EntryId: 7, Score: 1>
<EntryId: 6, Score: 0.00915307>
<EntryId: 8, Score: 0.00814517>
<EntryId: 1, Score: 0.00538609>

searching for image 8 returns 4 results:
<EntryId: 8, Score: 1>
<EntryId: 0, Score: 0.0103268>
<EntryId: 2, Score: 0.00893477>
<EntryId: 3, Score: 0.00870392>

searching for image 9 returns 4 results:
<EntryId: 9, Score: 1>
<EntryId: 0, Score: 0.0320906>
<EntryId: 8, Score: 0.00636511>
<EntryId: 1, Score: 0.00587605>
```

**结论（源逐字）**：增加字典规模时，**无关图像的相似性明显变小**（从约 0.02 降到约 0.006–0.01）。而相似图像（如图 1 和 10，下标 0 与 9）虽分值也略降（0.0525→0.0321），但**相对于其他图像的评分却更显著了**。这说明**增加字典训练样本是有益的**。可尝试更大字典。

> **数值对照表（综合 agent 参考，小字典 vs 大字典，image 0 的 top 结果）**：

| 查询 image 0 的 top 匹配 | 小字典(4983 词) Score | 大字典(99566 词) Score |
|---|---|---|
| 自身 (EntryId 0) | 1 | 1 |
| **回环帧 (EntryId 9)** | **0.0525447** | **0.0320906** |
| EntryId 8 | 0.0311284 | 0.0103268 |
| 第 4 名 | 0.0272257 (Id 5) | 0.0066729 (Id 4) |

可见回环帧(9)与"次相似帧"的差距，在大字典下被显著拉大（小字典 0.0525 vs 0.0311，比值≈1.7；大字典 0.0321 vs 0.0103，比值≈3.1）。

---

### 11.5.2 相似性评分的处理（源 §11.5.2）

**动机（源逐字）**：只用分值的**绝对大小**不一定有好帮助——有些环境外观本就相似（办公室有很多同款桌椅），另一些环境各处差异很大。

**先验相似度归一化（源 式 11.10）**：取一个**先验相似度** $s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})$——表示某时刻关键帧图像与**上一时刻关键帧**的相似性。其他分值都参照它**归一化**：

$$
\boxed{\ s\!\left(\boldsymbol v_t,\boldsymbol v_{t_j}\right)' = \dfrac{s\!\left(\boldsymbol v_t,\boldsymbol v_{t_j}\right)}{\,s\!\left(\boldsymbol v_t,\boldsymbol v_{t-\Delta t}\right)\,}.\ } \tag{11.10}
$$
> [符号说明：$\boldsymbol v_t$=当前帧 BoW；$\boldsymbol v_{t-\Delta t}$=上一关键帧 BoW；$\boldsymbol v_{t_j}$=历史第 $j$ 个关键帧 BoW；右上撇 $'$ 表示归一化后的分值。OCR 中下标 $t_j$、$t-\Delta t$ 基本正确，仅个别撇号/下标层次需还原。]

**判据（源逐字）**：如果当前帧与之前某关键帧的相似度**超过当前帧与上一个关键帧相似度的 3 倍**，就认为可能存在回环。此步骤**避免引入绝对的相似性阈值**，使算法能适应更多环境。

---

### 11.5.3 关键帧的处理（源 §11.5.3）

**关键帧选取不能太近（源逐字）**：若关键帧选得太近，两关键帧间相似性过高，相比之下不易检测出历史数据中的回环。例如检测结果经常是第 $n$ 帧和第 $n-2$、$n-3$ 帧最相似——太平凡、意义不大。所以实践上，**用于回环检测的帧最好稀疏一些**，彼此不太相同，又能涵盖整个环境。

**回环聚类避免重复（源逐字）**：若成功检测到回环（如出现在第 1 帧和第 $n$ 帧），那么很可能第 $n+1$、$n+2$ 帧也会和第 1 帧构成回环。确认第 1 帧和第 $n$ 帧的回环对轨迹优化有帮助；但接下去的 $n+1$、$n+2$ 帧与第 1 帧的回环帮助就没那么大了——因为已用之前信息消除了累积误差，**更多回环并不带来更多信息**。所以把"相近"的回环**聚成一类**，使算法不要反复检测同一类回环。

---

### 11.5.4 检测之后的验证（源 §11.5.4，对应【本章聚焦】之"几何验证"）

**为什么需要验证（源逐字）**：词袋回环检测**完全依赖外观、未利用任何几何信息**，导致外观相似的图像容易被当成回环。且词袋**不在乎单词顺序、只在意有无**，更易引发**感知偏差（Perceptual Aliasing）**。所以检测之后通常还有一个**验证步骤**（参考文献 [95, 112]）。

**两类验证方法（源逐字）**：
1. **时间上的一致性检测（temporal consistency）**：设立回环的**缓存机制**——认为单次检测到的回环不足以构成良好约束，而**在一段时间中一直检测到的回环**才是正确回环。
2. **空间上的一致性检测（spatial consistency / 几何验证）**：对回环检测到的两个帧进行**特征匹配，估计相机的运动**；然后把运动放到之前的**位姿图**中，检查与之前的估计是否有很大出入。

> **与后端位姿图（ch:nlopt）接口（综合 agent 重点）**：空间一致性验证正是回环模块与后端位姿图的**衔接点**——验证通过的回环会作为一条**相对位姿约束边**加入位姿图（即 §11.1.1 的"额外弹簧"）。本源只到"检查是否有大出入"为止，**位姿图优化的具体目标函数/求解需从第 10 讲补**。

> 验证部分通常是必需的，但如何实现见仁见智。

---

### 11.5.5 与机器学习的关系（源 §11.5.5，对应【本章聚焦】之"深度学习地点识别简介"）

**回环检测像分类问题（源逐字）**：回环检测本身非常像一个**分类问题**。与传统模式识别的区别：回环中**类别数量很大、每类样本很少**——极端情况下，机器人运动后图像变化即产生新类别，甚至可把类别当成**连续变量**而非离散；而回环（两图像落入同一类）很少出现。另一角度看，回环检测也相当于对"图像间相似性"概念的一个**学习**。

**词袋是无监督学习（源逐字）**：构建词典相当于对特征描述子**聚类**，树只是对所聚类的快速查找数据结构。既是聚类，至少可问：
1. 能否对**机器学习的图像特征**聚类，而非对 SURF、ORB 这样的**人工设计特征**聚类？
2. 是否有比"树结构 + K-means"更好的聚类方式？

**深度学习展望（源逐字，参考文献 [113–117]）**：
- 二进制描述子的学习、无监督聚类，都有望在**深度学习框架**中解决。
- 已陆续看到利用机器学习进行回环检测的工作。尽管目前**词袋仍是主流**，但作者相信未来深度学习方法很有希望打败这些人工设计特征的"传统"方法（参考文献 [113, 114]）——毕竟词袋在物体识别上已明显不如神经网络，而回环检测是非常相似的问题。
- 例子：BoW 的改进形式 **VLAD（Vector of Locally Aggregated Descriptors）** 就有**基于 CNN 的实现**（参考文献 [115, 116]）；也有一些**网络在训练后可从图像直接计算采集时刻相机的位姿**（参考文献 [117]，即 PoseNet 类的位姿回归）。这些都可能成为新的回环检测算法。
> [OCR 修正：源行 652 "也有一些**网格**在训练之后" 中"网格"应为"**网络**"（OCR 形近字错）。]

---

## 习题（源 §"习题"，逐字）

1. 请书写计算 PR 曲线的小程序。用 MATLAB 或 Python 可能更简便，因为它们擅长作图。
2. 验证回环检测算法需要有人工标记回环的数据集（例如参考文献 [103]）。然而人工标记很不方便，可考虑**根据标准轨迹计算回环**：若轨迹中有两个帧的位姿非常相近，就认为它们是回环。请根据 TUM 数据集给出的标准轨迹，计算出一个数据集中的回环。这些回环的图像真的相似吗？
3. 学习 DBoW3 或 DBoW2 库，自己寻找几张图片，看能否正确检测出回环。
4. 调研相似性评分的常用度量方式，哪些比较常用？
5. **Chow-Liu 树**是什么原理？它是如何被用于构建字典和回环检测的？
6. 阅读参考文献 [118]，除了词袋模型，还有哪些用于回环检测的方法？

---

## 参考文献编号（源中出现的引用，供综合 agent 溯源）

源正文以方括号数字引用，未给出文献全名。出现的编号：
- [12]（基于里程计回环检测的局限）；[88, 95, 99]（基于外观回环检测的实际系统）；[95, 112]（检测后验证）；
- [97]（随机检测有用的实现）；[98]（基于里程计的几何关系）；[100]（图 11-3 P-R 曲线来源）；
- [101]（K-means）；[102]（K-means++）；[103]（人工标记回环数据集 / FabMap 相关）；[103–105]（FabMap）；[106]（Chow-Liu tree）；[107]（k 叉树字典 / 层次 K-means，对应 Nister-Stewenius 词汇树类工作）；
- [109, 110]（TF-IDF）；[111]（DBoW2 的 $L_1$ 评分，即 Gálvez-López & Tardós 2012）；
- [113, 114]（深度学习回环检测）；[115, 116]（VLAD 的 CNN 实现，即 NetVLAD 类）；[117]（从图像回归相机位姿，PoseNet 类）；[118]（习题 6，其他回环检测方法综述）。

---

## 附录 A：本章公式总表（编号对齐源）

| 编号 | 公式 | 含义 | OCR 状态 |
|---|---|---|---|
| (11.1) | $s(\boldsymbol A,\boldsymbol B)=\|\boldsymbol A-\boldsymbol B\|$ | 朴素图像相减相似度（不好） | 范数符号已补 |
| (11.2) | $\text{Precision}=\frac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FP}},\ \text{Recall}=\frac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FN}}$ | 准确率/召回率 | 正确 |
| (11.3) | $A=1\cdot w_1+1\cdot w_2+0\cdot w_3$ | 图像的单词线性组合 | 正确 |
| (11.4) | $s(\boldsymbol a,\boldsymbol b)=1-\frac1W\|\boldsymbol a-\boldsymbol b\|_1$ | 二值/计数向量 $L_1$ 相似度 | 范数下标已补 |
| (11.5) | $\mathrm{IDF}_i=\log\frac{n}{n_i}$ | 逆文档频率（全局统计） | 正确 |
| (11.6) | $\mathrm{TF}_i=\frac{n_i}{n}$ | 词频（单图统计；$n,n_i$ 含义与 11.5 不同） | 正确（符号歧义为源固有） |
| (11.7) | $\eta_i=\mathrm{TF}_i\times\mathrm{IDF}_i$ | 单词权重 | 正确 |
| (11.8) | $A=\{(w_1,\eta_1),\dots,(w_N,\eta_N)\}\stackrel{\mathrm{def}}{=}\boldsymbol v_A$ | 图像的 BoW 向量（稀疏） | 正确 |
| (11.9) | $s(\boldsymbol v_A,\boldsymbol v_B)=1-\frac12\|\widehat{\boldsymbol v}_A-\widehat{\boldsymbol v}_B\|_1=\frac12\sum_i(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|)$ | DBoW2 的 $L_1$ 评分 | **OCR 重大错误，已重建** |
| (11.10) | $s(\boldsymbol v_t,\boldsymbol v_{t_j})'=\frac{s(\boldsymbol v_t,\boldsymbol v_{t_j})}{s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})}$ | 先验相似度归一化 | 正确 |

辅助公式（非编号）：暴力匹配复杂度 $C_N^2=\binom N2=O(N^2)$；k 叉树容量 $k^d$；查找复杂度 $O(d)=O(\log_k W)$；默认字典 $k=10,d=5\Rightarrow k^d=10^5$。

---

## 附录 B：OCR 修正说明（全量清单）

> 本源为图像 OCR，下列为抽取时发现并已修正的错误（按源行号）。原则：用领域知识/官方代码/文献重建，正文已就地标注。

**一、代码类（影响可编译性，已重建）**
1. 行 232–266：`feature_training.cpp` 被错误拆成两个代码块，`main` 在行 241 提前 `}` 闭合，第二块多出孤立 `}`。→ 合并为单一函数。
2. 行 254：`descriptors.push_backubes。`（且含中文句号）→ `descriptors.push_back(descriptor);`。
3. 行 334：文件路径 `slambook/ch12/loop_closure.cpp` → `slambook2/ch11/loop_closure.cpp`（第二版第 11 章）。
4. 行 337–398：`loop_closure.cpp` 被拆成三块。→ 合并。
5. 行 362：`vector(Mat> descriptors;` → `vector<Mat> descriptors;`（`<` 误成 `(`）。
6. 行 367：`descriptors.push_backubesor);` → `descriptors.push_back(descriptor);`。
7. 行 407：终端提示 `$ build/feature_training` → 应为 `$ build/loop_closure`（此处运行的是 loop_closure 程序）。

**二、公式类（影响数学正确性，已重建）**
8. 行 67–69 式(11.1)：范数双竖线 $\|\ \|$ 被 OCR 部分丢失 → 恢复 $\|\boldsymbol A-\boldsymbol B\|$。
9. 行 152–154 式(11.4)：范数下标 $_1$ 与双竖线层次有缺失 → 恢复 $\|\boldsymbol a-\boldsymbol b\|_1$。
10. **行 323–325 式(11.9)（最严重）**：OCR 作 $s(\boldsymbol v_A-\boldsymbol v_B)=2\sum_{i=1}^N|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|$。三处错：(a) 单参数 $s(\cdot-\cdot)$→二元 $s(\cdot,\cdot)$；(b) 系数 $2$→$\frac12$；(c) 求和项缺括号。→ 按 DBoW2 论文（参考文献 [111]，Gálvez-López & Tardós 2012）重建为 $s(\boldsymbol v_A,\boldsymbol v_B)=1-\frac12\|\widehat{\boldsymbol v}_A-\widehat{\boldsymbol v}_B\|_1=\frac12\sum_i(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|)$。
11. 行 622–626 式(11.10)：下标 $t_j$、$t-\Delta t$ 与撇号层次需还原（OCR 基本可读）→ 已规范化。

**三、文字/术语/章节号类**
12. 行 188 & 全章："Fabmap" → FabMap；"Chou-Liu tree" → **Chow-Liu tree**（习题 5 行 664 源本身作 "Chow-Liu"，前后印证）。
13. 行 519："对 **10.4 节** 的程序稍加修改" → **§11.4 节**（章号错印成 10，本章是第 11 讲）。
14. 行 519 / 行 525："最多一万个单词" 与 $k=10,d=5\Rightarrow10^5$ 及实测 `Number of words = 99566`（≈10 万）矛盾 → 应为"最多十万个单词"（疑源笔误或"十万"被 OCR 成"一万"）。已标 [OCR?] 并按 $10^5$ 为准。
15. 行 652："也有一些**网格**在训练之后" → "**网络**"（形近字错）。
16. 行 281："深度 $L$ 为 $5$"——$d$ 与 $L$ 两符号同义混用（非错，已注明）。
17. 行 277/525：DBoW3 输出 `k = 10, L = 5, Weighting = tf-idf, Scoring = L1-norm`——逐字保留，与正文 $k,d$ 对齐。

**四、未引入修改、但标注的源固有歧义**
18. 式(11.5) 与式(11.6) 复用同字母 $n,n_i$ 但含义不同（全局 vs 单图）；式(11.5)(11.6)(11.8) 与 $N$ 的含义在全章漂移——为源叙述固有，非 OCR 错，已在 §11.4.1 与记号表显式警告，建议综合写书时改名消歧。

---

*（抽取留痕结束。本文件按源小节组织，已全量记录 §11.1–§11.5 全部正文、表 11-1、式 (11.1)–(11.10)、算法 K-means/k 叉树、两段实践代码（OCR 修复版）、全部终端输出（小字典 + 大字典）、习题 1–6、参考文献编号；并附公式总表与 OCR 修正全清单。）*
