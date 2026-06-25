# 抽取留痕：回环检测与视觉词袋 Bag-of-Words / DBoW2

> 本文件为《机器人学笔记》项目内部「抽取留痕」，目标是把源材料【全量保真】抽取，供后续综合 agent 写成自包含书章。**禁摘要、禁凝练**。公式 LaTeX 写全、标源小节号，宁长勿略。
>
> **服务章节**：回环检测（Loop Closure / Place Recognition）。
>
> **本章聚焦**（综合 agent 必须覆盖）：回环检测的作用（消累积漂移、给位姿图加约束）；视觉词袋完整（词典构建、TF-IDF、相似度评分）；关键帧/相似度处理与时间一致性；几何验证；感知混叠与准确率/召回率；与位姿图后端（`ch:nlopt`）接口；深度学习地点识别简介。

---

## 0. 本抽取的源清单（多源融合）

本任务【源·联网研究】指定主题为 **视觉词袋 Bag-of-Words / DBoW2（Galvez-Lopez & Tardos TRO2012）** 与 **Nister-Stewenius 词汇树**。经检索，本环境内还**存在一份高度对口的本地中文教材源**（高翔《视觉SLAM十四讲》第 11 讲 回环检测），它正是基于 DBoW 库讲解回环检测的，与本书记号最接近，故一并全量抽取并作为"骨架源"。综合 agent 应以本地教材为叙事骨架，用两篇原始论文补全全部公式与算法细节。

| 编号 | 源 | 类型 | 物理位置 / URL | 抽取状态 |
| --- | --- | --- | --- | --- |
| **S1** | 高翔《视觉SLAM十四讲（第2版）》第 11 讲 回环检测 | 本地教材 MD（666 行，已完整读取） | `/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/11_回环检测.md` | ✅ 全量 |
| **S2** | Gálvez-López & Tardós, **"Real-Time Loop Detection with Bags of Binary Words"**, IEEE/RSJ IROS 2011, pp. 51–58 | 论文 PDF（作者自托管，597 行 pdftotext，已完整读取） | http://webdiis.unizar.es/~jdtardos/papers/2011_IEEE_IROS_Galvez.pdf | ✅ 全量 |
| **S2′** | Gálvez-López & Tardós, **"Bags of Binary Words for Fast Place Recognition in Image Sequences"**, **IEEE Trans. on Robotics (TRO), vol. 28, no. 5, Oct. 2012, pp. 1188–1197**, DOI 10.1109/TRO.2012.2197158 | 期刊版（任务指定的核心论文；与 S2 为同一方法的期刊扩展版） | IEEE Xplore 06202705 / DOI 上 | 通过 S2（同方法、同公式、作者自托管会议版）+ DBoW2 官方实现 S4 全量抽取；期刊版 PDF 因 cert 过期 / 403 无法直接抓取，已注明 |
| **S3** | Nistér & Stewénius, **"Scalable Recognition with a Vocabulary Tree"**, CVPR 2006, vol. 2, pp. 2161–2168 | 论文 PDF（Berkeley 课程托管，已完整读取 510 行 + 幻灯片确认公式） | https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/nister_stewenius_cvpr2006.pdf | ✅ 全量 |
| **S4** | DBoW2 官方实现（Dorian Galvez-López） | C++ 源码 + README | https://github.com/dorian3d/DBoW2 ；`src/ScoringObject.cpp`、`include/DBoW2/BowVector.h` | ✅ 抽取了 6 种评分公式、权重/评分枚举、API |
| **S5** | Arandjelović et al., **"NetVLAD: CNN architecture for weakly supervised place recognition"**, CVPR 2016, arXiv:1511.07247 | 论文 PDF（已读取 §3.1） | https://arxiv.org/abs/1511.07247 | ✅ 抽取 VLAD/NetVLAD 公式（深度学习地点识别简介用） |
| — | 旁证：ORB-SLAM / ORB-SLAM2（Mur-Artal & Tardós）；FAB-MAP（Cummins & Newman）；Sivic & Zisserman "Video Google" | 二手综述检索 | 见正文引用 | 用于"与位姿图后端接口"和历史脉络 |

> **S2 vs S2′ 说明（重要）**：任务点名的核心论文是 **TRO2012**（S2′）。TRO2012 是 IROS2011（S2）的**期刊扩展版**，由同两位作者撰写，方法、记号、核心公式 (1)–(6) 完全一致；TRO 版增加了更多实验（含 NewCollege、City Centre、Ford 等数据集）与更详细的几何验证（DI 层级、direct index 在第几层取节点）。由于 TRO 版 PDF 的两个权威镜像在抓取时分别返回 "certificate has expired"（doriangalvez.com）与 HTTP 403（IEEE/researchgate），**本文件以作者自托管的 IROS2011 会议版（S2）+ DBoW2 官方代码（S4）逐式抽取**，这两者共同覆盖了 TRO2012 的全部技术内容。凡 TRO 版相对会议版的增量（如默认 `k=10, L=6 → 1M words`、`α`、`k=3` 等参数、direct index 取倒数第 2 层等），均在正文显式标注来源。

---

## ⚠️ 抽取专员的覆盖度说明（综合 agent 务必先读）

| 聚焦主题 | 主源 | 覆盖深度 |
| --- | --- | --- |
| 回环检测作用（消漂移、加位姿图约束） | S1 §11.1.1 | ✅ 完整（含质点-弹簧系统类比、重定位） |
| 词袋完整：词典构建 | S1 §11.3 + S3 §3 + S2 §IV | ✅ 完整（K-means / K-means++ / 层次 k 叉树 / 二进制空间离散化 / k^d 容量 / 对数查找） |
| 词袋完整：TF-IDF | S1 §11.4.1 (式 11.5–11.8) + S3 式(4) + S2 式(2)(3) | ✅ 完整（两源 IDF 定义一致、TF 定义、tf-idf 权重、BoW 向量） |
| 词袋完整：相似度评分（L1 范数） | S1 式(11.9) + S2 式(4) + S3 式(3)(5)(6) + S4 源码 | ✅ 完整（L1/L2/χ²/KL/Bhatt/dot 全部公式 + Nister L_p 范数推导 + 归一化推导） |
| 关键帧 / 相似度处理 / 归一化 | S1 §11.5.1–11.5.3 (式 11.10) + S2 §V 式(5) | ✅ 完整（先验归一化、3 倍阈值、关键帧稀疏、回环聚类） |
| 时间一致性 | S2 §V（islands + k 连续匹配 + 式(6) H 评分） | ✅ 完整 |
| 几何验证 | S1 §11.5.4 + S2 §V（RANSAC 基础矩阵 + direct index + 12 对应） | ✅ 完整 |
| 感知混叠 / 准召率 | S1 §11.1.2–11.1.3 (式 11.1, 11.2) + S2 多处 | ✅ 完整（混淆矩阵、PR 曲线、SLAM 偏重准确率） |
| 与位姿图后端（`ch:nlopt`）接口 | S1 §11.1.1 + ORB-SLAM 旁证 | ✅ 概念完整（回环边、Sim3、essential graph、covisibility graph）；**注意**：位姿图优化本身的数学（信息矩阵、Schur 等）属 `ch:nlopt` 章，本文件只给"接口"层。 |
| 深度学习地点识别 | S1 §11.5.5 + S5 NetVLAD 式(1)(2) | ✅ 完整（VLAD / NetVLAD 软分配公式 / 弱监督 triplet 思想） |
| 实践代码 | S1 §11.3.2、§11.4.2、§11.5.1（DBoW3 C++）+ S4 API | ✅ 全量保留 |

---

## 1. 记号约定与本书统一约定的对齐（含差异说明）

> 本书统一约定：旋转 $R\in SO(3)$；右扰动为主；李代数切向量 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前、旋转在后）；Hamilton 四元数；过程噪声协方差 $\Sigma_w$、观测噪声协方差 $\Sigma_v$。

回环检测主题基本不涉及流形/扰动/四元数（它是"外观匹配"模块），但与几个记号会冲突，列明如下：

| 概念 | 本源记号 | 本书/统一记号 | 差异与处理 |
| --- | --- | --- | --- |
| 单词 (word) | $w_i$ (S1/S2)；node $i$ (S3) | 沿用 $w_i$ | 一致。**注意 $w_i$ 在 S3 还兼指"节点权重"**（S3 式(1)(4) 的 $w_i$ 是权重标量），需按上下文区分：S1/S2 中 $w_i$=单词，S3 中 $w_i$=节点权重。本文件统一：**单词写 $w_i$，权重写 $\eta_i$（跟 S1）或在 S3 语境保留 $w_i$ 并显式说明**。 |
| 词典/词汇表大小 | $W$（叶子数，S2）；leaf nodes $k^L$（S3） | $W$ | 一致。 |
| 分支因子 | $k$（S3 branch factor）；$k_w$（S2） | $k$ | 一致。S2 用 $k_w=10$。**注意 $k$ 在 S2 §V 还表示"时间一致性要求的连续匹配数"**（$k=3$），是完全不同的量；本文件保留两处 $k$ 并显式区分。 |
| 树深度 | $d$（S1）；$L$（S3, S2 写 $L_w$） | $L$ | S1 用 $d$，S2/S3 用 $L$。DBoW3 输出里写 `L=5`。**统一用 $L$（深度/层数），并注 S1 的 $d=L$。** |
| BoW 向量 | $\boldsymbol v_A$ (S1)、$\boldsymbol v_t$ (S2)、$\boldsymbol q,\boldsymbol d$ (S3) | $\boldsymbol v$ | S3 用 $\boldsymbol q$=query、$\boldsymbol d$=database 区分查询/库向量；S1/S2 统一用 $\boldsymbol v$。注意 **S3 的 $\boldsymbol d$ 是"数据库向量"，不是本书里 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ 的位姿/平移**。 |
| 权重 | $\eta_i$ (S1 式 11.7)；$w_i$ (S3) | $\eta_i$ | tf-idf 权重。 |
| 相似度评分 | $s(\cdot,\cdot)$（所有源） | $s(\cdot,\cdot)$ | 一致。S2/S4 约定 $s\in[0,1]$（1=最相似）；S3 的 $s$ 是"归一化差"（0=最相似，因为是范数差）。**两种约定方向相反，务必区分**：见 §6.4。 |
| 归一化后评分 | $\eta(\boldsymbol v_t,\boldsymbol v_{t_j})$ (S2 式5)；$s'(\cdot)$ (S1 式11.10) | — | **冲突警告**：S2 式(5) 把"先验归一化后的评分"也记作 $\eta$，与"tf-idf 权重 $\eta_i$"撞名。本文件在归一化评分处一律写 $\eta(\boldsymbol v_t,\boldsymbol v_{t_j})$（带括号双参数）以区别于权重标量 $\eta_i$（带单下标）。 |
| 描述子 | BRIEF $B(p)$、SIFT、ORB、SURF | — | 二进制描述子用 Hamming 距离；浮点描述子用欧氏距离。 |
| 协方差 | 本主题基本不出现 | $\Sigma_w/\Sigma_v$ | 回环边送入位姿图后才有信息矩阵，属 `ch:nlopt`。 |

**与本书差异小结**：最大陷阱是 (a) **相似度方向**——S3 是"范数差"（越小越像），S1/S2/S4 是"评分"（越大越像，归一化到 [0,1]）；(b) **$\eta$ 撞名**——既是 tf-idf 权重又是归一化评分；(c) **$k$ 撞名**——既是分支因子又是时间一致性匹配数。综合成书时建议：权重用 $\eta_i$、归一化评分另起符号（如 $s'$ 或 $\hat s$）、时间一致性匹配数另用符号（如 $\kappa$）。

---

# 第一部分（S1）高翔《视觉SLAM十四讲》第 11 讲 回环检测 —— 全量抽取

> 源：`视觉SLAM十四讲_md/11_回环检测.md`。本部分作为叙事骨架，逐小节保真。

## 11.0 本讲主要目标（S1 源行 1–11）

1. 理解回环检测的必要性。
2. 掌握基于词袋的外观式回环检测。
3. 通过 **DBoW3** 的实验，学习词袋模型的实际用途。

本讲介绍 SLAM 中另一个主要模块：回环检测。SLAM 主体（前端、后端）主要目的在于估计相机运动，而回环检测模块，无论目标上还是方法上，都与前面相差较大，通常被认为是一个**独立的模块**。主流视觉 SLAM 检测回环的方式是**词袋模型**。

## 11.1 概述

### 11.1.1 回环检测的意义（S1 §11.1.1，源行 18–51）

前端提供特征点提取和轨迹、地图的初值，后端负责对所有数据进行优化。然而，如果像视觉里程计那样**仅考虑相邻时间上的关键帧**，那么之前产生的误差将不可避免地**累积**到下一时刻，使整个 SLAM 出现**累积误差（累积漂移）**，长期估计结果不可靠——无法构建全局一致的轨迹和地图。

**例子（自动驾驶建图，源行 22）**：采集车在某给定区域绕若干圈。前端只给出局部位姿间约束，例如 $x_1-x_2,\ x_2-x_3,\dots$。但由于 $x_1$ 的估计存在误差，而 $x_2$ 根据 $x_1$ 决定，$x_3$ 又由 $x_2$ 决定，依此类推，**误差被累积**，后端优化结果慢慢趋向不准确（图 11-1(b)）。在这种场景下应保证：优化的轨迹和实际地点一致；当实际经过同一地点时，估计轨迹也必定经过同一点。

> **图 11-1 漂移示意图**（源行 46）：(a) 真实轨迹；(b) 由于前端只给出相邻帧间的估计，优化后的位姿图出现漂移；(c) 添加回环检测后的位姿图可以消除累积误差。

虽然后端能估计最大后验误差，但"好模型架不住烂数据"，只有相邻关键帧数据时无从消除累积误差。**回环检测模块能给出除相邻帧之外的、时隔更久远的约束**：例如 $x_1\sim x_{100}$ 之间的位姿变换。为什么它们之间会有约束？因为**察觉到相机经过了同一个地方，采集到了相似的数据**。回环检测的关键，就是如何**有效地检测出相机经过同一个地方**这件事。成功检测后，就可为后端的位姿图提供更多有效数据，得到更好的、特别是**全局一致**的估计。

> **质点-弹簧系统类比（源行 49）**：位姿图可看成一个质点—弹簧系统，回环检测相当于在图中加入了额外的弹簧，提高系统稳定性。可直观想象成**回环边把带有累积误差的边"拉"到了正确的位置**——如果回环本身正确的话。

回环检测意义重大：(1) 关系到估计的轨迹和地图在长时间下的正确性；(2) 提供了**当前数据与所有历史数据的关联**，还可用于**重定位**。重定位用处更多：例如事先对某场景录制一条轨迹并建立地图，之后可一直跟随这条轨迹导航，重定位帮助确定自身在轨迹上的位置。

> **术语界定（源行 51）**：有时把**仅有前端和局部后端的系统称为视觉里程计（VO）**，而把**带有回环检测和全局后端的系统称为 SLAM**。

### 11.1.2 回环检测的方法（S1 §11.1.2，源行 53–77）

存在若干思路（理论与工程上）：

1. **暴力两两匹配**：对任意两幅图像都做一遍特征匹配，按正确匹配数量确定关联。朴素而有效，但盲目假设"任意两幅图像都可能存在回环"，检测数量太大：对 $N$ 个可能回环要检测 $C_N^2$ 次，是 $O(N^2)$ 复杂度，随轨迹变长增长太快，实时系统不实用。
2. **随机抽取**：随机抽历史数据做回环检测（如在 $n$ 帧中随机抽 5 帧与当前帧比较）。能维持常数时间运算量，但帧数 $N$ 增长时抽到回环的概率大幅下降，检测效率不高。随机检测在有些实现中确实有用 [97]。

上面朴素思路过于粗糙。至少希望有"哪处可能出现回环"的预计。大体两种思路：

- **基于里程计（Odometry based）的几何关系** [98]：当发现当前相机运动到了之前某位置附近时，检测它们有没有回环关系。但由于**累积误差**，往往没法正确发现"运动到了之前某位置附近"，回环检测也无从谈起。逻辑上有"倒果为因"嫌疑（回环检测目标本是发现"相机回到之前位置"以消除累积误差，而此法却假设了"相机回到之前位置附近"才能检测），因而在累积误差较大时无法工作 [12]。
- **基于外观（Appearance based）**：和前端、后端的估计都无关，**仅根据两幅图像的相似性**确定回环关系。摆脱了累积误差，使回环检测模块成为相对独立的模块（前端可为它提供特征点）。自 21 世纪初被提出以来，能有效在不同场景下工作，**成了视觉 SLAM 中主流做法** [88,95,99]。

工程上还有别的办法：室外无人车配 GPS 提供全局位置信息，可轻松判断是否回到经过的点，但室内不好用。

**核心问题**：在基于外观的回环检测中，如何计算图像间相似性 $s(A,B)$。当它大于一定量后认为出现回环。最朴素的想法——直接相减取范数：

$$
s(\boldsymbol A,\boldsymbol B)=\|\boldsymbol A-\boldsymbol B\|.\tag{11.1}
$$

**为什么不这样做**（源行 73–77）：

1. **像素灰度是不稳定的测量值**，严重受环境光照和相机曝光影响。相机未动、打开一支电灯，图像整体变亮，即使同样数据也得到很大差异值。
2. **相机视角少量变化时**，即使每个物体光度不变，像素也会在图像中位移，造成很大差异值。

由于这两种情况，实际中即使非常相似的图像，$\boldsymbol A-\boldsymbol B$ 也常得到不符合实际的很大值。所以式(11.1)不能很好反映相似关系。由此引出**感知偏差（Perceptual Aliasing）**和**感知变异（Perceptual Variability）**两个概念。

### 11.1.3 准确率和召回率（S1 §11.1.3，源行 79–128）

希望程序算法得出和人类（或事实）一致的判断。程序判断不总与人类一致，可能出现 4 种情况：

> **表 11-1 回环检测的结果分类**（源行 84–87）：

| 算法 ＼ 事实 | 是回环 | 不是回环 |
| --- | --- | --- |
| **是回环** | 真阳性 (True Positive, TP) | 假阳性 (False Positive, FP) |
| **不是回环** | 假阴性 (False Negative, FN) | 真阴性 (True Negative, TN) |

阴性/阳性借用医学说法。**假阳性（False Positive）又称感知偏差**，**假阴性（False Negative）称感知变异**（图 11-2）。希望 TP、TN 尽量高，FP、FN 尽可能低。统计某算法在某数据集上的 TP、TN、FP、FN 次数，计算两个统计量——**准确率和召回率（Precision & Recall）**：

$$
\text{Precision}=\frac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FP}},\qquad \text{Recall}=\frac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FN}}.\tag{11.2}
$$

> **图 11-2**（源行 110）：左侧假阳性——两幅图像看起来很像但并非同一走廊；右侧假阴性——由于光照变化，同一地点不同时刻照片看起来很不一样。

**字面意义**：准确率描述算法提取的所有回环中**确实是真实回环的概率**；召回率指在所有真实回环中**被正确检测出来的概率**。取这两个统计量因为它们有代表性，且**通常是一对矛盾**：

- 提高某阈值 → 算法更"严格" → 检出更少回环 → 准确率提高，但漏掉原本是回环的地方 → 召回率下降。
- 更宽松配置 → 检出更多 → 召回率更高，但混杂非回环 → 准确率下降。

**Precision-Recall 曲线**（图 11-3）：以召回率为横轴、准确率为纵轴，关心整条曲线偏向右上方的程度、**100% 准确率下的召回率**、或 **50% 召回率时的准确率**作为评价指标。除"天壤之别"的算法外，通常不能一概而论说 A 优于 B。

> **SLAM 中的取舍（源行 119，关键）**：在 SLAM 中**对准确率要求更高，对召回率相对宽容**。因为假阳性回环将在后端位姿图中**添加根本错误的边**，有时导致优化给出完全错误的结果（"如果 SLAM 把所有办公桌当成同一张，走廊不直了、墙壁交错、整个地图失效"）。相比之下，召回率低些顶多有部分回环没检出，地图受些累积误差影响——而**仅需一两次回环就可完全消除**。所以倾向于把参数设得更严格，或在检测后再加**回环验证**步骤。

> **图 11-3**（源行 125）：随召回率上升，检测条件变宽松，准确率随之下降；好算法在较高召回率下仍能保证较好准确率。引自 [100]。

回到式(11.1)：用 $\boldsymbol A-\boldsymbol B$ 算相似性，准确率和召回率都很差，可能大量假阳性或假阴性，所以"不好"。

## 11.2 词袋模型（S1 §11.2，源行 130–164）

直接相减不够好，需要更可靠的方式。一种思路：像 VO 那样用特征点做回环检测——对两幅图像特征点匹配，匹配数量大于一定值就认为回环；还能算出运动关系。但存在问题（匹配费时、光照变化时描述不稳定），不过离词袋模型已很相近。

**词袋（Bag-of-Words, BoW）**目的是用"图像上有哪几种特征"来描述一幅图像。三步：

1. 确定"人""车""狗"等概念——对应 BoW 中的**单词（Word）**，许多单词组成**字典（Dictionary）**。
2. 确定一幅图像中出现了哪些字典中定义的概念——用单词出现情况（或直方图）描述整幅图像，把图像转换成一个**向量**。
3. 比较描述的相似程度。

记单词 $w_1,w_2,w_3$（如"人""车""狗"）。对图像 $A$，根据含有的单词记为

$$
A=1\cdot w_1+1\cdot w_2+0\cdot w_3.\tag{11.3}
$$

字典固定，故只用 $[1,1,0]^{\mathrm T}$ 即可表达 $A$。该向量描述"图像是否含有某类特征"，比单纯灰度值更稳定；又因为说的是"是否出现"而不管"在哪儿出现"，所以**与物体的空间位置和排列顺序无关**——相机少量运动时只要物体仍在视野中，描述向量就不变。强调 **Words 的有无而无关其顺序**，故称 Bag-of-Words 而非 List-of-Words。字典类似单词的一个集合。

图像 $B$ 用 $[2,0,1]^{\mathrm T}$ 描述；若只考虑"是否出现"不考虑数量，可为 $[1,0,1]^{\mathrm T}$（二值）。对两个向量 $\boldsymbol a,\boldsymbol b\in\mathbb R^W$ 求差的一种做法：

$$
s(\boldsymbol a,\boldsymbol b)=1-\frac{1}{W}\|\boldsymbol a-\boldsymbol b\|_1.\tag{11.4}
$$

其中范数取 $L_1$ 范数（各元素绝对值之和）。**两向量完全一样时得 1；完全相反时（$\boldsymbol a$ 为 0 的地方 $\boldsymbol b$ 为 1）得 0**。这就定义了描述向量的相似性，也即图像间的相似程度。

接下来两个问题：(1) 字典怎么来的？(2) 能算相似度评分就足够判断回环了吗？

## 11.3 字典（S1 §11.3）

### 11.3.1 字典的结构（S1 §11.3.1，源行 168–207）

字典由很多单词组成，每个单词代表一个概念。单词不是从单幅图像提取的，而是**某一类特征的组合**。所以字典生成问题类似**聚类（Clustering）问题**——属无监督机器学习（Unsupervised ML）。

设对大量图像提取了 $N$ 个特征点，想找含 $k$ 个单词的字典，每个单词看作局部相邻特征点的集合。用经典 **K-means（K 均值）算法** [101]。

**K-means 步骤**（把 $N$ 个数据归成 $k$ 类，源行 174–183）：

1. 随机选取 $k$ 个中心点 $c_1,\dots,c_k$。
2. 对每个样本，计算它与每个中心点的距离，取最小的作为它的归类。
3. 重新计算每个类的中心点。
4. 若每个中心点都变化很小则收敛、退出；否则返回第 2 步。

K-means 朴素简单有效，但有问题：需指定聚类数量、随机选中心使每次结果不同、效率问题。研究者开发了**层次聚类法、K-means++ [102]** 等弥补不足。

**查找问题**：如何根据图像中某特征点查找字典中相应单词？朴素思想是和每个单词比对取最相似——$O(n)$ 查找，对一万、十万个单词显然太慢。字典排序后二分查找可达对数级别。实践中可用更复杂数据结构，如 FAB-MAP [103-105] 中的 **Chow-Liu tree** [106]。本书介绍另一种简单实用的树结构 [107]（即 Nister-Stewenius 词汇树，本文件 S3）。

**k 叉树字典**（参考文献 [107]，源行 190–207）：类似层次聚类，是 K-means 的直接扩展。设有 $N$ 个特征点，要建深度为 $d$、每次分叉为 $k$ 的树：

1. 在根节点，用 K-means 把所有样本聚成 $k$ 类（实际中为保证均匀性用 **K-means++**），得到第一层。
2. 对第一层每个节点，把属于该节点的样本再聚成 $k$ 类，得到下一层。
3. 依此类推，最后得到叶子层。**叶子层即为所谓的 Words**。

> **图 11-4**（源行 204）：训练字典时逐层用 K-means 聚类；查找单词时逐层比对找到对应单词。

最终在叶子层构建单词，**中间节点仅供快速查找使用**。一个 $k$ 分支、深度 $d$ 的树可以容纳

$$
\boxed{k^d}\ \text{个单词}.
$$

查找某给定特征对应单词时，只需将它与每个中间节点的聚类中心比较（共 $d$ 次），即可找到最后单词，保证**对数级别的查找效率**（$O(\log W)=O(d)$）。

### 11.3.2 实践：创建字典（S1 §11.3.2，源行 209–283）

演示如何生成及使用 **ORB 字典**。选取 TUM 数据集中 10 幅图像（`slambook2/ch11/data`，图 11-5）——来自一组实际相机运动轨迹，**第一幅与最后一幅明显采自同一地方**，看算法能否检测这个回环。

> 实用字典往往在更大数据集上训练，数据应来自与目标环境类似的地方。字典越大单词量越丰富、越容易找到对应单词，但不能大到超过计算能力和内存。这里暂从 10 幅图像训练小字典。可用别人训练好的字典，但**注意特征类型是否一致**。

使用的 BoW 库：**DBoW3**（https://github.com/rmsalinas/DBow3），cmake 工程。

**代码 `slambook2/ch11/feature_training.cpp`**（源行 232–266，全量保留，含 OCR 修正见 §OCR）：

```cpp
int main(int argc, char **argv) {
    // read the image
    cout << "reading images..." << endl;
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
        descriptors.push_back(descriptor);   // [OCR修正: 源OCR作 "descriptors.push_backubes。"]
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

**说明（源行 268）**：对 10 张图像提取 ORB 特征存入 `vector`，调用 DBoW3 字典生成接口。`DBoW3::Vocabulary` 构造函数可指定树的分叉数量及深度，这里用默认构造函数即 **$k=10,\ d=5$**——小规模字典，**最大容纳 $k^d=10^5=100{,}000$ 个单词**。图像特征用默认每幅 500 个特征点。字典存为压缩文件。

**运行输出**（源行 272–278）：

```text
$ build/feature_training
reading images...
detecting ORB features ...
creating vocabulary ...
vocabulary info: Vocabulary: k = 10, L = 5, Weighting = tf-idf, Scoring = L1-norm, Number of words = 4983
done
```

分支数 $k=10$，深度 $L=5$，单词数 **4983**（未达最大容量）。`Weighting = tf-idf`（权重）、`Scoring = L1-norm`（评分）——评分如何计算见 §11.4。

## 11.4 相似度计算（S1 §11.4）

### 11.4.1 理论部分（S1 §11.4.1，源行 287–327）

有了字典后，给定任意特征 $f_i$，在字典树中逐层查找，最后找到对应单词 $w_j$——字典足够大时认为 $f_i$ 和 $w_j$ 来自同一类物体（聚类意义下，无理论保证）。从一幅图像提取 $N$ 个特征、找到对应单词后，就拥有该图像在单词列表中的**分布/直方图**——一个 Bag。

**为何加权（源行 291）**：朴素做法对所有单词"一视同仁"（有就是有）。但**部分单词具有更强区分性**：如"的""是"在许多句子出现、无法判别句子类型；"文档""足球"区分作用大。希望对单词的区分性/重要性加以评估，给不同权值。

**TF-IDF（Term Frequency-Inverse Document Frequency，频率—逆文档频率）** [109,110]：文本检索常用加权方式，也用于 BoW。

- **TF 思想**：某单词在一幅图像中**经常出现**，区分度就高。
- **IDF 思想**：某单词在字典中**出现频率越低**，分类图像时区分度越高。

**IDF（建立字典时计算）**：统计某叶子节点 $w_i$ 中的特征数量相对于所有特征数量的比例。设所有特征数量为 $n$、$w_i$ 数量为 $n_i$，则

$$
\mathrm{IDF}_i=\log\frac{n}{n_i}.\tag{11.5}
$$

**TF（某特征在单幅图像中出现的频率）**：设图像 $A$ 中单词 $w_i$ 出现 $n_i$ 次、一共出现单词次数为 $n$，则

$$
\mathrm{TF}_i=\frac{n_i}{n}.\tag{11.6}
$$

> ⚠️ **OCR/记号注意**：S1 式(11.5) 与 (11.6) 复用了符号 $n,n_i$，但**含义不同**——(11.5) 的 $n,n_i$ 是"训练语料全体的特征计数"，(11.6) 的 $n,n_i$ 是"单幅图像内的单词计数"。S2 的式(2)(3) 用了不同符号区分（见 §第二部分），更清楚。综合成书建议采用 S2 记号。

$w_i$ 的权重 = TF × IDF：

$$
\eta_i=\mathrm{TF}_i\times\mathrm{IDF}_i.\tag{11.7}
$$

考虑权重后，图像 $A$ 的特征点对应到许多单词，组成它的 BoW：

$$
A=\{(w_1,\eta_1),(w_2,\eta_2),\dots,(w_N,\eta_N)\}\stackrel{\mathrm{def}}{=}\boldsymbol v_A.\tag{11.8}
$$

相似特征可能落到同一类，故 $\boldsymbol v_A$ 中有大量零——是**稀疏向量**，非零部分指示图像 $A$ 含哪些单词、值为 TF-IDF。

**差异计算（$L_1$ 范数形式，引自参考文献 [111]，源行 321–325）**：

$$
s(\boldsymbol v_A-\boldsymbol v_B)=2\sum_{i=1}^{N}\bigl(|\boldsymbol v_{Ai}|+|\boldsymbol v_{Bi}|-|\boldsymbol v_{Ai}-\boldsymbol v_{Bi}|\bigr).\tag{11.9}
$$

> ⚠️ **OCR 与一致性注意（重要）**：式(11.9) 的 OCR 缺了求和号对整个括号的作用域（源里写成 `2 \sum |v_Ai| + |v_Bi| - |v_Ai - v_Bi|`，求和号只罩住第一项）。**正确形式应为上式（求和罩住整个 $|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|$）**。此式即 Nister-Stewenius 论文式(5) 在 $L_1$ 下的 per-term 形式（见 S3 §6.4）：因为对归一化向量，$\sum_i(|q_i|+|d_i|-|q_i-d_i|)$ 只在 $q_i,d_i$ 都非零处非零，$\|q-d\|_1=2+\sum_{q_i\ne0,d_i\ne0}(|q_i-d_i|-|q_i|-|d_i|)=2-\sum_{q_i\ne0,d_i\ne0}(|q_i|+|d_i|-|q_i-d_i|)$。S1 引文 [111] 即 Nister-Stewenius，故 (11.9) 给出的是**相似度的"重叠量"**（越大越像）。参见 §6.4 的完整推导与 §第二部分 S2 式(4) 给出的等价归一化形式 $s(\boldsymbol v_1,\boldsymbol v_2)=1-\tfrac12\bigl\|\frac{\boldsymbol v_1}{|\boldsymbol v_1|}-\frac{\boldsymbol v_2}{|\boldsymbol v_2|}\bigr\|_1$。

### 11.4.2 实践：相似度的计算（S1 §11.4.2，源行 329–511）

用 11.3 节生成的字典生成词袋并比较差异。

**代码 `slambook2/ch11/loop_closure.cpp`**（源行 337–397，全量保留，OCR 修正见 §OCR）：

```cpp
int main(int argc, char **argv) {
    // read the images and database
    cout << "reading database" << endl;
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
    // DBoW3::Vocabulary vocab("./vocab_larger.yml.gz"); // use large vocab if you want:
    if (vocab.empty()) {
        cerr << "Vocabulary does not exist." << endl;
        return 1;
    }
    cout << "reading images..." << endl;
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
    vector<Mat> descriptors;                       // [OCR修正: 源作 "vector(Mat>"]
    for (Mat &image : images) {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);         // [OCR修正: 源作 "push_backubesor)"]
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
        db.query(descriptors[i], ret, 4); // max result=4
        cout << "searching for image " << i << " returns " << ret << endl << endl;
    }
    cout << "done." << endl;
}
```

演示两种比对：**图像之间直接比较**与**图像与数据库比较**（大同小异）。

**终端输出（BoW 向量，源行 406–414）**：

```text
desp 0 size: 500
transform image 0 into BoW vector: size = 455
key value pair = <1, 0.00155622>, <3, 0.00222645>, <12, 0.00222645>, <13, 0.00222645>,
<14, 0.00222645>, <22, 0.00222645>, <33, 0.00222645>, <37, 0.00155622>, <38, 0.00222645>,
<39, 0.00222645>, <43, 0.00222645>, <57, 0.00155622> .....
```

BoW 向量含每个单词的 **ID 和权重**（构成整个稀疏向量）。比较两向量时 DBoW3 计算一个分数，方式由构造字典时定义（`L1-norm`）。

**终端输出（图 0 与各图相似度，源行 424–433）**：

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

**数据库查询（排序给出最相似，源行 443–507，全量保留）**：

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

**结果分析（源行 509–511）**：明显相似的图 1 和图 10（C++ 下标 0 和 9）相似度评分约 **0.0525**，其他图约 **0.02**。然而单从数值看并没有想象的那么明显：无关图像相似度约 2%、相似图像约 5%。引出后续"为什么不明显"与"如何处理"的讨论。

## 11.5 实验分析与评述（S1 §11.5）

### 11.5.1 增加字典规模（S1 §11.5.1，源行 515–618）

机器学习里结果不满意先怀疑数据/规模。这里先怀疑：**字典是不是太小**（仅从 10 幅图生成）。

`slambook2/ch11/vocab_larger.yml.gz`：对同一数据序列**所有图像（约 2,900 幅）**生成的稍大字典，规模仍取 $k=10,\ d=5$（最多一万单词）。可用 `gen_vocab_large.cpp` 自训。**训练大字典需大内存机器并耐心等待**。

**终端输出（用大字典，源行 523–615，全量保留关键部分）**：

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

> 注：该大字典输出里 `Number of words = 99566`（接近 $10^5$ 上限），而正文叙述训练自约 2,900 幅图。

**结论（源行 618）**：增加字典规模时，**无关图像的相似性明显变小**；相似图像（图 1、10）分值虽略降，但**相对于其他图像变得更显著**。说明**增加字典训练样本是有益的**。

### 11.5.2 相似性评分的处理（S1 §11.5.2，源行 620–628）

只利用分值绝对大小不一定有好帮助：有些环境本来就很相似（办公室同款桌椅），另一些各处差别很大。取一个**先验相似度** $s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})$——某时刻关键帧与上一时刻关键帧的相似性，其他分值参照它**归一化**：

$$
s(\boldsymbol v_t,\boldsymbol v_{t_j})'=\frac{s(\boldsymbol v_t,\boldsymbol v_{t_j})}{s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})}.\tag{11.10}
$$

> 此即 S2 式(5) 的归一化评分 $\eta$（见 §第二部分）。

**判据**：如果当前帧与之前某关键帧的相似度**超过当前帧与上一关键帧相似度的 3 倍**，就认为可能存在回环。此步**避免引入绝对的相似性阈值**，使算法适应更多环境。

### 11.5.3 关键帧的处理（S1 §11.5.3，源行 630–634）

- **关键帧不能太近**：太近则两关键帧相似性过高，不易检出历史回环（检测结果常是第 $n$ 帧与第 $n-2,n-3$ 帧最相似，太平凡）。用于回环检测的帧**最好稀疏**，彼此不太相同又涵盖整个环境。
- **回环聚类**：若成功检测到回环（如第 1 帧与第 $n$ 帧），则第 $n+1,n+2$ 帧很可能也与第 1 帧构成回环。但确认第 1–$n$ 帧回环后，更多回环并不带来更多信息（累积误差已被消除）。所以把"相近"的回环**聚成一类**，使算法不反复检测同一类回环。

> 此即 S2 §V 的 **islands（岛）**机制（见 §第二部分），高翔在此用文字描述，S2 给出形式化定义与岛评分 $H$。

### 11.5.4 检测之后的验证（S1 §11.5.4，源行 636–640）

词袋回环检测**完全依赖外观、不用任何几何信息**，外观相似的图像容易被当成回环；且词袋**不在乎单词顺序、只在意有无**，更易引发**感知偏差**。故回环检测后通常加**验证步骤** [95,112]：

1. **时间一致性检测（回环缓存机制）**：单次检测到的回环不足以构成良好约束；**在一段时间中一直检测到的回环才是正确的回环**。
2. **空间一致性检测**：对回环检测到的两帧做**特征匹配、估计相机运动**，把运动放到位姿图中，检查与之前估计是否有大出入。

验证部分通常必需，但实现见仁见智。

### 11.5.5 与机器学习的关系（S1 §11.5.5，源行 642–652）

回环检测**很像分类问题**，但类别数量很大、每类样本很少——极端情况下机器人运动后图像变化就产生新类别，甚至可把类别当连续变量；回环检测相当于"两幅图像落入同一类"，很少出现。也可看作对"图像间相似性"概念的学习。

词袋本身是**非监督机器学习**：构建词典 = 对特征描述子聚类，树只是快速查找的数据结构。可问：

1. 能否对**机器学习的图像特征**聚类，而非对 SURF、ORB 人工特征聚类？
2. 是否有更好的聚类方式（而非树 + K-means 这种朴素方式）？

二进制描述子的学习和无监督聚类都有望在深度学习框架中解决。词袋在物体识别上已明显不如神经网络，而回环检测是非常相似的问题。例如 **BoW 的改进形式 VLAD 有基于 CNN 的实现** [115,116]（即 NetVLAD，本文件 S5），也有网络训练后可从图像直接计算采集时刻相机位姿 [117]。

### 11.5.6 习题（S1，源行 654–666，全量保留）

1. 书写计算 PR 曲线的小程序（MATLAB 或 Python）。
2. 验证回环检测需人工标记回环数据集（如 [103]）。人工标记不便，可**根据标准轨迹计算回环**：若轨迹中两帧位姿非常相近就认为回环。请根据 TUM 数据集标准轨迹计算回环。这些回环的图像真的相似吗？
3. 学习 DBoW3 或 DBoW2 库，自己找几张图片，看能否正确检测回环。
4. 调研相似性评分的常用度量方式，哪些常用？
5. Chow-Liu 树是什么原理？如何用于构建字典和回环检测？
6. 阅读参考文献 [118]，除词袋外还有哪些回环检测方法？

---

# 第二部分（S2/S2′）Gálvez-López & Tardós — Bags of Binary Words（DBoW2，IROS2011/TRO2012）

> 源：`2011_IEEE_IROS_Galvez.pdf`（作者自托管会议版，与 TRO2012 同方法）。**这是任务点名的核心论文 DBoW2 的权威原始描述。**逐节、逐式全量抽取。

## D-0. 摘要与三大创新（S2 Abstract + §I）

提出一种用**高效特征**在图像序列中**实时**检测重访地点的方法。对"词袋 + 几何检验"框架引入三大创新：

1. **二进制特征**：用 **FAST 关键点** + **BRIEF 描述子**（二进制、计算极快，每点 <20 µs）。
2. **二进制词袋 + 逆向索引（inverse index）**：用一个**离散化二进制描述子空间**的词袋做图像比较。
3. **正向索引（direct index）**：利用词袋高效获得两图间对应点，**避免 $\Theta(n^2)$ 的匹配**。

为检测回环候选，提出**分组管理匹配**（islands）提高词袋返回候选的可靠性。在三个真实公开数据集（0.7–1.7 km 轨迹）上获高准召率，对 19000 图序列平均每图仅 **16 ms**（特征计算 + 整个回环检测），比同类技术快**一个数量级**。

> **TRO2012 增量（S2′）**：期刊版把方法推广到一般描述子（实现库 DBoW2 同时支持 ORB / BRIEF），增加 NewCollege、City Centre、Ford、Malaga 等数据集，并更细化几何验证（direct index 取词汇树倒数第 2 层节点做对应、DI levels 参数）。核心公式 (1)–(6) 与会议版相同。

**问题定义（§I）**：移动机器人遍历环境时，发觉某地是否已访问过称**回环闭合检测（loop closure detection）**，是 SLAM 产生一致地图必须解决的任务。基本技术：把机器人在线采集的图像加入数据库；新图像采集时检索最相似的库图像，足够相似则检测到回环。词袋使"与数千图像比较"可在数十毫秒完成 [7]。但词袋因**感知混叠（perceptual aliasing）** [5] 不是完美方案，故后接**几何一致性验证**（需特征对应）。

**SLAM 双线程结构（§I）**：回环算法瓶颈通常是**特征提取**（约比其余步骤贵 10 倍计算周期），且描述子维度高会拖慢匹配。这导致 SLAM 常跑两个解耦线程：一个做主 SLAM 功能，另一个仅做回环检测和数据关联（如 [4]）。

## D-1. 二进制特征（S2 §III）

提取局部特征（关键点 + 描述子）通常计算极贵、是瓶颈。本文用 **FAST 关键点** [9] + **BRIEF 描述子** [8]。

**FAST**：角点状点，通过比较半径为 3 的 Bresenham 圆上若干像素的灰度强度检测。只检查少量像素，故极快，适合实时。

**BRIEF**：对每个 FAST 关键点画方形 patch 并算 BRIEF 描述子——一个二进制向量，每位是 patch 内两像素的**强度测试**结果。patch 先用高斯核平滑降噪。给定 patch 大小 $S_b$，测试像素对在**离线阶段随机选定**。还需设 $L_b$：测试次数（即描述子长度）。给定图像中点 $p$，其 BRIEF 描述子向量 $B(p)$ 的第 $i$ 位为

$$
B_i(p)=\begin{cases}1 & \text{if } p+x_i < p+y_i\\[2pt] 0 & \text{otherwise}\end{cases}\qquad \forall i\in[1..L_b]\tag{1}
$$

其中 $B_i(p)$ 是向量第 $i$ 位，$x_i,y_i$ 是测试点偏移（事先随机选定），其值须落在 $\bigl[-\tfrac{S_b}{2}..\tfrac{S_b}{2}\bigr]\times\bigl[-\tfrac{S_b}{2}..\tfrac{S_b}{2}\bigr]$。**该描述子不需训练，只需离线随机选点（几乎不耗时）**。

**测试点选取（关键改进）**：原 BRIEF [8] 按正态分布 $N(0,\tfrac{1}{25}S_b^2)$ 选 $x,y$。本文发现**用接近的（close）测试对效果更好**（§VI-A），通过采样

$$
x=N\!\left(0,\tfrac{1}{25}S_b^2\right),\qquad y=N\!\left(x,\tfrac{4}{625}S_b^2\right)
$$

来选这些对（[8] 也报告过此法但未在最终实验用）。**最终选 $L_b=256$、$S_b=48$**（distinctiveness 与计算时间的良好折中）。

**优势**：BRIEF 计算极快（Calonder 等报告 $L_b=256$ 时每点 17.3 µs），比较也快——描述子是位向量，**两向量距离 = 不同位数（Hamming 距离）**，用 **xor** 实现。比 SIFT/SURF 的欧氏距离更适合本场景。

> **本书记号对齐**：二进制描述子用 Hamming 距离；本书统一对 ORB（同为二进制）亦用 Hamming。FAST/BRIEF/ORB 的细节属视觉里程计章（`ch:vo`，本仓库 `visual_odometry__*`），此处仅给回环检测所需的"二进制词袋"接口。

## D-2. 图像数据库：层次词袋 + 正向索引 + 逆向索引（S2 §IV，核心）

> **图 2**（源行 145）：图像数据库由 **bag of words（词袋树）** + **direct index（正向索引）** + **inverse index（逆向索引）** 三部分组成。

### D-2.1 词袋（视觉词汇表）的构建

词袋用视觉词汇表把一幅图像的局部特征集**转成稀疏数值向量**。词汇表**离线**创建，把描述子空间离散化为 $W$ 个**视觉单词**。与 SIFT/SURF 不同，本文**离散化二进制描述子空间**，创建更紧凑的词汇表。

**层次词袋（树结构）构建步骤（§IV，逐字）**：

1. 从一些训练图像（与在线处理的图像独立）提取丰富特征集。
2. 用 **k-means++ 算法** [19] 把描述子离散化为 $k_w$ 个二进制簇。由于 k-means++ 需计算若干描述子向量的**质心（centroid）**，**对质心的值取整（round）以获得二进制簇**。这些簇构成词汇树的第一层节点。
3. 对每个节点关联的描述子**重复此操作**生成后续层，直到 $L_w$ 层。
4. 最终得到含 **$W$ 个叶子**的树，叶子即词汇表的**单词**。

> **关键**："To the best of our knowledge, this is the first time a binary vocabulary is used for loop detection." 这是首次将二进制词汇表用于回环检测。

**权重（idf）**：训练时给每个单词 $w_i$ 赋**逆文档频率（inverse document frequency, idf）**——降低很频繁、因而判别性弱的单词的权重：

$$
\mathrm{idf}(i)=\log\frac{N}{n_i}\tag{2}
$$

其中 $N$ 是训练图像数，$n_i$ 是单词 $w_i$ 在这些图像中的出现次数。

> **与 S1 式(11.5) 一致**：$\log(N/n_i)$。S2 把"训练语料计数"显式写作 $N,n_i$，与下面式(3) 的"单幅图像计数"区分清楚。

### D-2.2 图像 → BoW 向量；TF-IDF

把时刻 $t$ 拍摄的图像 $I_t$ 转成 BoW 向量 $\boldsymbol v_t\in\mathbb R^W$：其特征的二进制描述子从**根到叶**遍历树，每层选**最小化 Hamming 距离**的中间节点。这样可计算该图像中每个单词的**词频（term frequency, tf）**：

$$
\mathrm{tf}(i,I_t)=\frac{n_{iI_t}}{n_{I_t}}\tag{3}
$$

其中 $n_{iI_t}$ 是单词 $w_i$ 在图像 $I_t$ 中的出现次数，$n_{I_t}$ 是 $I_t$ 中的单词总数。$\boldsymbol v_t$ 的第 $i$ 项最终取值

$$
\boxed{v_{ti}=\mathrm{tf}(i,I_t)\times\mathrm{idf}(i)}
$$

即 **tf-idf 权重**（如 [6, Sivic-Zisserman 提出]）。

> **与 S1 式(11.6)(11.7) 一致**：$v_{ti}=\mathrm{TF}_i\cdot\mathrm{IDF}_i=\eta_i$。

### D-2.3 相似度评分（L1 score）

衡量两 BoW 向量 $\boldsymbol v_1,\boldsymbol v_2$ 的相似度，用 **$L_1$-score** $s(\boldsymbol v_1,\boldsymbol v_2)$，值落在 $[0..1]$：

$$
\boxed{\,s(\boldsymbol v_1,\boldsymbol v_2)=1-\frac{1}{2}\left\|\frac{\boldsymbol v_1}{|\boldsymbol v_1|}-\frac{\boldsymbol v_2}{|\boldsymbol v_2|}\right\|_1\,}\tag{4}
$$

数据库被查询时，所有匹配按其评分**排序**。

> **方向**：S2 式(4) 中 $s=1$ 表示最相似（两归一化向量相等时 $\|\cdot\|_1=0$，$s=1$），$s=0$ 表示最不相似。**与 S1 的 DBoW3 输出方向一致**（自比为 1）。
>
> **与 S1 式(11.9) 的关系**：S1 式(11.9) 给的是"重叠量" $\sum(|v_{1i}|+|v_{2i}|-|v_{1i}-v_{2i}|)$；对归一化向量，$\|v_1/|v_1|-v_2/|v_2|\|_1=2-\sum_{i}(|\hat v_{1i}|+|\hat v_{2i}|-|\hat v_{1i}-\hat v_{2i}|)$，代入式(4) 得 $s=1-\tfrac12\bigl(2-\sum(\cdot)\bigr)=\tfrac12\sum(\cdot)$。故 S1 式(11.9) 的 $\tfrac12\times$ 即 S2 式(4)。两式等价。详见 §6.4 的 Nister-Stewenius 推导。

### D-2.4 逆向索引（inverse index）

与词袋一起维护一个**逆向索引（inverse index）**：为词汇表中每个单词 $w_i$ 存一个**包含该单词的图像 $I_t$ 列表**。查询数据库时极有用——**只需与"和查询图像有共同单词"的图像比较**。本文**扩充逆向索引以存储二元组 $\langle I_t, v_{ti}\rangle$**，以快速访问单词在该图像中的权重。逆向索引在新图像 $I_t$ 加入数据库时更新、在数据库被检索时访问。

### D-2.5 正向索引（direct index，本文创新）

词袋 + 逆向索引常是词袋方法仅用的两个结构。本文**新增正向索引（direct index）**：为每个图像 $I_t$ 存它**包含的单词 $w_i$ 列表**，以及**关联到每个单词的局部特征 $f_{tj}$**。

利用词袋树**作为 BRIEF 描述子空间的近似最近邻（ANN）手段**：要计算查询图像与库中任一图像的对应，**只需比较属于同一单词的特征**，避免匹配两图所有特征（$\Theta(n^2)$ 复杂度）。正向索引在新图像加入时更新、在获得候选匹配且需几何检验时访问。

> **TRO2012 增量**：DI 不一定取叶子层单词，而是取词汇树**某个较高层（如倒数第 $l$ 层，论文记 direct index level）**的节点来聚合特征——层越高聚到一起的特征越多（找到更多对应但更慢），层越低越快但可能漏对应。会议版 §VII 也讨论了"可放宽到共享父节点"的方向。

## D-3. 回环检测算法（S2 §V，核心：归一化、islands、时间一致性、几何验证）

四步（§V）：

1. **检索**：在数据库中搜索当前图像，检索那些匹配评分达到阈值的场景。
2. **分组（islands）**：把"时间上相近采集"的图像的匹配**分组为单个匹配**。
3. **时间一致性**：对评分最高的匹配做与之前场景的**时间一致性检验**，得到回环候选。
4. **几何验证**：若最佳候选通过几何验证，接受回环。

### D-3.1 检索与先验归一化（式 5）

最后图像 $I_t$ 采集后转成 $\boldsymbol v_t$，搜索数据库得匹配列表 $\langle\boldsymbol v_t,\boldsymbol v_{t_1}\rangle,\langle\boldsymbol v_t,\boldsymbol v_{t_2}\rangle,\dots$ 及其评分 $s(\boldsymbol v_t,\boldsymbol v_{t_j})$。评分范围取决于 $\boldsymbol v_t$ 中单词的分布。用"该序列中对 $\boldsymbol v_t$ **期望得到的最佳评分**"归一化，得新评分 $\eta$：

$$
\boxed{\,\eta(\boldsymbol v_t,\boldsymbol v_{t_j})=\frac{s(\boldsymbol v_t,\boldsymbol v_{t_j})}{s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})}\,}\tag{5}
$$

这里用**前一图像的 BoW 向量** $\boldsymbol v_{t-\Delta t}$ 来近似 $\boldsymbol v_t$ 的期望评分。**$s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})$ 小的情况（如机器人转弯时）会错误地造成高评分**，故**跳过**那些 $s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})$ 达不到最小值、或特征数达不到要求的图像。然后**拒绝**那些 $\eta(\boldsymbol v_t,\boldsymbol v_{t_j})$ 达不到最小值 $\alpha$ 的匹配。

> **与 S1 式(11.10) 完全对应**：S1 的 $s'$ 即此处 $\eta$；S1 的"超过 3 倍"即取 $\alpha=3$ 的一种设定（但 S2 在带几何验证时用 $\alpha=0.3$，不带时用 $\alpha=0.6$，见 §D-4 实验——注意 S1 与 S2 的 $\alpha$ 数值口径不同：S1 的 3 倍是相对前一帧的倍数，S2 的 $\alpha$ 是归一化评分阈值，量纲一致只是取值不同）。

### D-3.2 Islands（岛）—— 防止时间相近图像互相竞争（式 6）

**创新**：防止时间相近的图像在查询时互相竞争，把它们**分组为 islands** 当作一个匹配。记号：

- $T_i$：由时间戳 $t_{n_i},\dots,t_{m_i}$ 组成的**区间（interval）**。
- $V_{T_i}$：把条目 $\boldsymbol v_{t_{n_i}},\dots,\boldsymbol v_{t_{m_i}}$ 分组在一起的**岛（island）**。

若 $t_{n_i},\dots,t_{m_i}$ 中连续时间戳的间隔很小，则多个匹配 $\langle\boldsymbol v_t,\boldsymbol v_{t_{n_i}}\rangle,\dots,\langle\boldsymbol v_t,\boldsymbol v_{t_{m_i}}\rangle$ 被合并为单个匹配 $\langle\boldsymbol v_t, V_{T_i}\rangle$。

**岛评分 $H$**：

$$
\boxed{\,H(\boldsymbol v_t,V_{T_i})=\sum_{j=n_i}^{m_i}\eta(\boldsymbol v_t,\boldsymbol v_{t_j})\,}\tag{6}
$$

**评分最高的岛被选为匹配组**。Islands 不仅避免连续图像间的冲突，还能帮助建立正确匹配：若 $I_t$ 和 $I_{t'}$ 是真实回环，则 $I_t$ 很可能也相似于 $I_{t'\pm\Delta t}, I_{t'\pm2\Delta t},\dots$，产生**长岛**；由于 $H$ 定义为 $\eta$ 之和，$H$ **偏好长岛**。

> 此即 S1 §11.5.3 "回环聚类"的形式化。

### D-3.3 时间一致性检验（temporal consistency）

得到匹配岛 $V_{T'}$ 后，接受为有效前先施加**时间约束**（本文把 [4][5] 的时间约束扩展到支持 islands）。匹配 $\langle\boldsymbol v_t, V_{T'}\rangle$ 必须与 **$k$ 个之前的匹配** $\langle\boldsymbol v_{t-\Delta t},V_{T_1}\rangle,\dots,\langle\boldsymbol v_{t-k\Delta t},V_{T_k}\rangle$ **一致**，使得区间 $T_i$ 与 $T_{i+1}$ **接近重叠（close to overlap）**。若岛通过时间约束，则**只保留**匹配 $\langle\boldsymbol v_t,\boldsymbol v_{t'}\rangle$（取使评分 $\eta$ 最大的 $t'\in T'$），并视为**回环候选**。

> **关键参数 $k$**：这里的 $k$ 是"时间一致性要求的连续匹配数"，**与分支因子 $k_w$ 不同**！实验选 $k=3$（§D-4）。

### D-3.4 几何验证（geometrical checking）

对候选匹配场景做**几何检验**：用 **RANSAC** [20] 在 $I_t$ 与 $I_{t'}$ 之间找一个**基础矩阵（fundamental matrix）**，要求至少 **12 个对应** [21, Hartley-Zisserman] 支持。

**计算对应的三种方式**：

1. **线性搜索（linear search）**：度量 $I_t$ 每个特征到 $I_{t'}$ 各特征在描述子空间的距离，取最小距离的对。是特征数的 $\Theta(n^2)$ 操作（最简单最慢）。
2. **k-means / k-d 树 ANN** [22]：把描述子排进 k-means 或 k-d 树算近似最近邻。
3. **复用词袋词汇表（本文采用）**：加图像入库时把"单词—特征对"列表存进**正向索引**；获取 $I_t$ 与 $I_{t'}$ 对应时，在正向索引查 $I_{t'}$，**只对"关联到同一单词"的特征做线性搜索**。

只需基础矩阵做验证，但算出后**可把匹配场景间的数据关联以零额外成本提供给底层 SLAM 算法**。

> **与 S1 §11.5.4 对应**：S1 的"空间一致性检测（特征匹配、估计运动、检查与之前估计出入）"即此处 RANSAC 基础矩阵；S1 的"时间一致性（回环缓存）"即 §D-3.3。
>
> **与位姿图后端接口**：算出的基础矩阵给出 $I_t,I_{t'}$ 间相对位姿（分解 $F$ 或对应点重新三角化/PnP），这就是送入位姿图（`ch:nlopt`）的**回环边**（约束）。详见 §第六部分。

## D-4. 实验评估（S2 §VI，全量保留参数、表与结论）

### D-4.1 选择 BRIEF 参数（§VI-A）

变 $L_b$（描述子长度）、$S_b$（patch 大小）、测试对选法。两种选 $x,y$ 法：(1) $x,y$ 都从 $N(0,\tfrac{1}{25}S_b^2)$；(2) close pairs：$x=N(0,\tfrac1{25}S_b^2),\ y=N(x,\tfrac4{625}S_b^2)$。

取 3 个不同场景各若干图（14–23 张），用**光束法平差（bundle adjustment）** [23] 重建 3D 并获每对图的真值对应。对 $S_b=24,48,64,80$ 像素的 patch 在每图密集 FAST 关键点上算两版 BRIEF 并匹配。匹配判据：两描述子距离最小，且"最近/次近距离比"低于阈值；因描述子随机性，对 **5 个不同测试对模式重复**。

> **图 3**：3 测试场景下 BRIEF 与 BRIEF-close-pairs 随 $L_b$ 的平均准召率（$S_b=48$）。**close pairs 的准确率在相同召回率下总更高**（局部性提供更多判别性）；准召率随对数增加而提升直到某长度。最终选 **BRIEF close pairs，$L_b=256$，$S_b=48$**。

### D-4.2 回环检测器评估（§VI-B）

三个公开真实数据集（European Rawseeds 项目 [10]）：**Bicocca 2009-02-25b（室内 Indoor）、Bovisa 2008-10-04（室外 Outdoor）、Bovisa 2008-10-06（混合 Mixed）**。静态室内、静态室外、动态混合室内外。两个大学校园不同日期采集。用单相机，26K–35K 张 640×480 黑白图，15 Hz 采集，0.7–1.7 km 轨迹。

**词汇表参数（关键）**：$k_w=10$ 分支、$L_w=6$ 深度，**产生 $W=10^6$（一百万）单词**。用约 **10K 图、9M 特征**（来自第四个 Rawseeds 数据集）训练。实验时每图特征限 **300 个**，FAST 角点响应阈值 **10**。查询时只保留 **50f** 个返回结果（$f$ 为处理频率），忽略接近当前时间戳的结果。

> **TRO2012 默认即此**：$k=10, L=6 \Rightarrow 10^6$ words，DBoW2 库内置训练好的 ORB/BRIEF 词典也采用这一量级。

**消融实验**：先 $f=1$ Hz、无几何约束。

> **图 4**：各数据集变 $\alpha$ 的 PR 曲线，并展示要求 $k$ 个时间一致匹配的影响。$k=0$（禁用时间一致性）与 $k>0$ 间有**大改进**——时间一致性是避免误匹配的宝贵机制。$k$ 增大时 100% 准确率下召回率更高，但**很高的 $k$ 不成立**（只有很长的回环才被找到）。**选 $k=3$**（三数据集准召平衡好）。$f=1$ Hz 时这意味着回环须**至少持续 3 秒**。

> **图 5(a)**：$k=3$、变 $\alpha$ 的 PR——系统表现很好，即使无几何验证也获高召回（除室外数据集）且无假阳性。与之前 SURF 系统 [5]（图 5(b)）相近。**室外数据集 BRIEF 比 SURF 稍差**（深度大导致图像很相似），但 100% 准确率工作点 BRIEF 召回略高（4.1%）。

> **TABLE I — 系统的准确率与召回率**（源行 421–428，全量）：
>
> | Dataset | Length (m) | Precision (%) | Recall (%) | # Images |
> | --- | --- | --- | --- | --- |
> | Indoor | 760 | **100** | 57.86 | 2723 |
> | Outdoor | 1365 | **100** | 5.83 | 2345 |
> | Mixed | 1750 | **100** | 28.08 | 2247 |
>
> 用 **$\alpha=0.3$ + 基础矩阵约束**（图 6 最终检出回环，无假阳性）。室外召回低因场景深度大、图像很相似（[5] 报告之前技术最佳无假阳性仅 1.9% 召回；[4] 显示 FAB-MAP [24] 在此数据集达不到 100% 准确率）。证明 **BRIEF 虽缺尺度/旋转不变性，但对平面内相机运动的回环检测和 SURF 一样可靠且快得多**。

**$\alpha$ 选择策略**：可选限制性 $\alpha$（如 0.6，同 [5]）得 100% 准确率，但依赖数据集；或设较低 $\alpha$ 并用几何约束验证以最大化准确率、提升召回——本文取后者，**$\alpha=0.3$ + 基础矩阵**。

### D-4.3 性能与执行时间（§VI-C）

Intel Core @ 2.67 GHz。

> **TABLE II — 2723 图执行时间 (ms)**（源行 466–481，全量）：
>
> | 阶段 | 子步 | Mean | Std | Min | Max |
> | --- | --- | --- | --- | --- | --- |
> | Features | FAST | 2.39 | 1.07 | 1.37 | 7.84 |
> | Features | Smoothing | 1.27 | 0.05 | 1.21 | 1.43 |
> | Features | BRIEF | 1.44 | 0.37 | 0.11 | 1.74 |
> | Bag of words | Conversion | 3.00 | 0.78 | 0.21 | 3.66 |
> | Bag of words | Query | 0.17 | 0.13 | 0.00 | 0.62 |
> | Bag of words | Islands | 0.06 | 0.02 | 0.01 | 0.11 |
> | Bag of words | Insertion | 0.02 | 0.04 | 0.00 | 0.13 |
> | Verification | Correspondences and RANSAC | 0.82 | 1.29 | 0.25 | 5.79 |
> | **Whole system** | | **6.15** | 4.03 | 1.37 | 16.34 |
>
> 整系统 **6 ms/图**（峰值 <17 ms），远低于 30 Hz 视频的 33 ms 预算。

> **TABLE III — 几何检验执行时间 (ms)**（源行 483–488，全量）：
>
> | | Mean | Std | Min | Max |
> | --- | --- | --- | --- | --- |
> | **With direct index** | 0.82 | 1.29 | 0.25 | 5.79 |
> | **Without direct index** | 17.82 | 9.67 | 0.14 | 57.07 |
>
> **正向索引使几何验证提速约 90%**。

> **TABLE IV — 19344 图执行时间 (ms)**（源行 466–481，全量）：
>
> | 阶段 | 子步 | Mean | Std | Min | Max |
> | --- | --- | --- | --- | --- | --- |
> | Features | FAST | 4.52 | 2.16 | 1.39 | 16.66 |
> | Features | Smoothing | 1.65 | 0.75 | 1.21 | 6.90 |
> | Features | BRIEF | 1.54 | 0.54 | 0.10 | 4.77 |
> | Bag of words | Conversion | 2.98 | 0.78 | 0.21 | 7.00 |
> | Bag of words | Query | 4.95 | 5.44 | 0.00 | 36.38 |
> | Bag of words | Islands | 0.07 | 0.02 | 0.01 | 0.18 |
> | Bag of words | Insertion | 0.02 | 0.01 | 0.00 | 0.15 |
> | Verification | Correspondences and RANSAC | 1.03 | 1.59 | 0.02 | 6.12 |
> | **Whole system** | | **15.69** | 6.88 | 1.39 | 48.91 |
>
> 19K 图库整系统 **16 ms/图**（<49 ms）。查询随库增大变慢但仍很低（5–37 ms），说明**该阶段对数万图扩展良好**。

**大词汇表的反直觉好处（§VI-C 关键洞见）**：图像特征转 BoW 是第二耗时阶段，依赖特征数与词汇表大小。**可用更小词汇表减少转换时间**，但发现**大词汇表（1M 单词，而非 [1][4] 的 10–60K）产生更稀疏的逆向索引**——查询时**需遍历的库条目更少**，大幅降低查询时间，**远超转换时多花的时间**。结论：**大词汇表在大图像集上可改善计算时间**。

### D-4.4 结论与未来工作（§VII）

实时检测单目序列回环，对"词袋 + 几何检验"引入数个创新：(1) **BRIEF 离散为层次二进制词袋**提速 >1 数量级；(2) BRIEF close pairs 对平面内运动和 SURF 一样可靠；(3) **islands** 防相近图像冲突；(4) 基于 [4][5] 的**岛时间约束**；(5) **正向索引优化几何验证**（对应点提速约 90%）。

**正向索引的局限**：可能"内禀地禁止某些不属于同一视觉单词的特征间的对应"。可放宽为"**对其单词有某共同父节点的特征**计算对应"——上溯词汇树的层数权衡速度与找到更多正确对应的机会。

**大词汇表降低大库执行时间**。2.7K 图 16 ms（均 6 ms）、19K 图 49 ms（均 16 ms），比 SIFT/SURF 的 100–700 ms 改善 >1 数量级，也胜 [2]（用紧凑随机树签名 [17][18]，对 2700 图约需 200 ms 完整回环检测）。**对感知混叠场景有效**。未来：自动学习系统参数、测试更多挑战环境（含车/人等高动态城市区）。

> **S2 参考文献关键映射**（本书引用时用）：[6]=Sivic & Zisserman "Video Google" ICCV2003（tf-idf+inverse index 之源）；[7]/[此处 [7] 在 IROS 版指 Nister-Stewenius]，会议版 ref [7]=Nister & Stewenius CVPR2006（即本文件 S3，层次词汇树）；[8]=Calonder BRIEF ECCV2010；[9]=Rosten & Drummond FAST ECCV2006；[19]=Arthur & Vassilvitskii k-means++ SODA2007；[20]=Fischler & Bolles RANSAC 1981；[21]=Hartley & Zisserman MVG（基础矩阵/12 对应）；[22]=Muja & Lowe FLANN；[24]=Cummins & Newman FAB-MAP IJRR2008。

---

# 第三部分（S3）Nistér & Stewénius — Scalable Recognition with a Vocabulary Tree（CVPR 2006）

> 源：`nister_stewenius_cvpr2006.pdf`（Berkeley 课程托管，全文 510 行 + 幻灯片确认）。**这是层次词汇树（hierarchical k-means + TF-IDF + inverted file）的奠基论文**，S1 的参考文献 [107]、S2 的参考文献 [7] 均指它。逐节、逐式全量抽取。

## N-0. 摘要与贡献（§Abstract + §1）

提出**高效扩展到大量物体**的识别方案：现场演示从 **40000** 张流行音乐 CD 封面库识别 CD 封面。建立在"索引局部区域描述子"的流行技术上，对背景杂乱和遮挡鲁棒。局部区域描述子在**词汇树（vocabulary tree）**中**层次量化**，允许用更大、更具判别性的词汇表，**显著提升检索质量**。最重要的性质：**树直接定义了量化**——量化与索引完全整合、本质上是同一件事。在带真值的库上评估，规模高达 **100 万** 图像。

**最重要贡献（§1）**：一种使检索极高效的**索引机制**。当前实现：640×480 帧特征提取约 0.2 s，对 50000 图库查询 25 ms。

## N-1. 方法与相关工作（§2，含关键技术陈述）

受 **Sivic & Zisserman [17]（Video Google）**启发：他们用文本检索方法检索电影镜头，描述子由 k-means 量化成**视觉单词**，用 **TF-IDF** 对图像相关性评分、用**倒排文件（inverted files）**实现。

本文提出**层次 TF-IDF 评分**，用层次定义的视觉单词构成**词汇树**，允许更高效查找视觉单词、用更大词汇表（显著提升检索质量）。关键陈述：

- 用更大词汇表**释放倒排文件方法的真正威力**：减少须显式考虑的库图像比例。[17] 用约 10000 视觉单词、每帧约 1000 单词，则查询遍历约**库的 1/10**（即使占用均匀）。本文证明**更大词汇表（甚至 16M 叶子）检索质量更好**。
- 词汇树用**层次 k-means** 提供更高效训练：[17] 用 400 训练帧，本文高达 35000。
- **量化一次定义、永久使用**，允许**在线即时插入新图像**（约 5 Hz @ 640×480），用于视觉 SLAM——"new locations need to be added on-the-fly"。

**特征提取**：本文用自实现的 **MSER**（最大稳定极值区域）[10]，把每个 MSER 椭圆 patch warp 成圆 patch，其余按 Lowe SIFT 流水线 [9]（方向直方图找标准方向、提 SIFT 描述子）。归一化 SIFT 描述子再由词汇树量化，最后用层次评分检索。

## N-2. 构建与使用词汇树（§3，核心算法）

词汇树定义一个由**层次 k-means 聚类**建立的层次量化。用一大组代表性描述子做**无监督训练**。

**关键：$k$ 不是最终簇/量化单元数，而是树的分支因子（branch factor，每节点子节点数）**：

**词汇树构建算法（§3，逐字）**：

1. 先在训练数据上跑一次 **k-means**，定义 $k$ 个簇中心。
2. 训练数据**划分为 $k$ 组**，每组由"最接近某簇中心"的描述子构成。
3. **递归地**对每组描述子应用同样过程，把每个量化单元再**分裂为 $k$ 个新部分**。
4. 逐层确定树，直到**最大层数 $L$**；每次"分裂为 $k$ 部分"只由属于父量化单元的描述子分布决定。

> **图 2**：层次量化在每层由 $k$ 个中心（此处 $k=3$）及其 Voronoi 区域定义。

**在线阶段（查找单词）**：每个描述子向量**沿树向下传播**——每层将其与 $k$ 个候选簇中心（$k$ 个子节点）比较、选最近者。这是每层 $k$ 个点积，共 **$kL$ 个点积**（$k$ 不太大时很高效）。向下路径可用**单个整数编码**，供评分使用。

> **关键陈述**：树**直接定义视觉词汇表 + 高效搜索过程**（整合方式）。这不同于"非层次定义词汇表再设计 ANN 搜索"。层次法对后续评分更灵活。

**复杂度**：

- 非层次增大词汇表计算代价**很高**；层次法的计算代价是叶子数的**对数**。
- 内存随叶子数 $k^L$ **线性**。须表示的描述子向量总数：

$$
\sum_{i=1}^{L}k^{i}=\frac{k^{L+1}-k}{k-1}\approx k^{L}.
$$

- $D$ 维描述子用 char 表示时，树大小约 $D k^{L}$ 字节。**当前实现：$D=128,\ L=6,\ k=10\Rightarrow 1\text{M}$ 叶子，用 143 MB 内存**。

> **与 S1 式"$k^d$ 容量"一致**；与 S2 的 $k_w=10,L_w=6\Rightarrow 10^6$ 一致。

## N-3. 评分定义（§4，核心：权重、查询/库向量、归一化差、L_p 推导）

量化定义后，根据"库图像与查询图像的描述子在词汇树中的**路径有多相似**"确定库图像与查询的相关性。

> **图 3**：分支因子 10 的词汇树 3 层，填充以表示一幅含 400 特征的图像。

**通用框架**：给每个节点 $i$ 赋权重 $w_i$（通常基于熵 entropy），按权重定义**查询向量 $q_i$** 与**库向量 $d_i$**：

$$
q_i = n_i w_i\tag{1}
$$
$$
d_i = m_i w_i\tag{2}
$$

其中 $n_i,m_i$ 分别是**查询图像、库图像**通过节点 $i$ 的描述子向量数。库图像的**相关性评分 $s$**基于查询与库向量的**归一化差**：

$$
s(q,d)=\left\|\frac{q}{\|q\|}-\frac{d}{\|d\|}\right\|.\tag{3}
$$

> ⚠️ **方向**：S3 式(3) 是**范数差**——**越小越相似**（与 S2 式(4) 的 $s$ 越大越相似**方向相反**）。归一化是为"少描述子与多描述子的库图像之间的公平"。**发现 $L_1$ 范数比标准 $L_2$ 范数结果更好。**

**熵权重（TF-IDF）**：最简单情形 $w_i$ 设为常数，但**熵权重通常改善检索**：

$$
\boxed{\,w_i=\ln\frac{N}{N_i}\,}\tag{4}
$$

其中 $N$ 是库中图像数，$N_i$ 是库中"至少有一个描述子路径通过节点 $i$"的图像数。**这给出 TF-IDF 方案**。（也试过用"节点 $i$ 出现频率"代替 $N_i$，差别不大。）

> **与 S1 式(11.5)、S2 式(2) 一致**：$\ln(N/N_i)=\log(N/n_i)$（IDF）。$q_i=n_i w_i$ 即 $\mathrm{tf}\cdot\mathrm{idf}$（$n_i$ 是查询内词频，$w_i$ 是 idf 熵权重）。

**层级权重处理**：直觉上应"赋予节点相对其路径上方节点的熵"，但**令人意外地发现**：用"相对树根的熵、忽略路径内依赖"更好。也可令某些层权重为零、只用最接近叶子的层。

**最重要的发现（§4）**：对检索质量最重要的是**大词汇表（大量叶子）**且**不要给内节点过强权重**。原则上词汇表最终会太大（描述子的变异和噪声频繁把描述子在量化单元间移动）。权衡：判别性（要小量化单元、深树）vs 可重复性（要大量化单元）。但**层次评分降低了"词汇表过大"的风险**。在很大范围内（1–16M 叶子）检索性能随叶子数增加。**叶子节点远比内节点强大**——这也解释了为何"相对根节点赋熵"更好。

**停用表（stop lists）**：也可对最频繁/最不频繁的符号令 $w_i=0$。用倒排文件时**屏蔽过长的列表**（密集列表的符号贡献熵少），主要为效率，有时也改善检索。[17] 用停用表移除误匹配，但本文多数情况未能靠停用表改善检索质量。

## N-4. 评分实现（§5，核心：倒排文件 + L_p 范数推导）

用**倒排文件（inverted files）**对大库高效评分。词汇树每个节点关联一个倒排文件，存"该节点出现的图像 id"以及"每图的词频 $m_i$"。**正向文件（forward files）**可作补充以查"某图含哪些视觉单词"。

**实现细节**：本文实现中**只显式表示叶子节点**，内节点的倒排文件是其下叶子倒排文件的**串接**（图 4）。倒排文件长度（即决定节点熵的文档频率）存在词汇树每个节点中；**超过一定长度的倒排文件被屏蔽**不参与评分。

> **图 4**：两层、分支 2 的数据库结构。叶子有显式倒排文件，内节点有"虚拟倒排文件"（叶子倒排文件串接而成）。

**$L_p$ 范数归一化差的高效计算（核心推导，§5 式 5）**：设各节点熵固定已知（可预计算）。库图像向量可预计算并**归一化到单位模**（入库时）；查询向量也归一化到单位模。计算 $L_p$ 范数归一化差可用：

$$
\begin{aligned}
\|q-d\|_p^p &= \sum_i |q_i-d_i|^p\\
&= \sum_{i\mid d_i=0}|q_i|^p+\sum_{i\mid q_i=0}|d_i|^p+\sum_{i\mid q_i\ne0,d_i\ne0}|q_i-d_i|^p\\
&= \|q\|_p^p+\|d\|_p^p+\sum_{i\mid q_i\ne0,d_i\ne0}\bigl(|q_i-d_i|^p-|q_i|^p-|d_i|^p\bigr)\\
&= 2+\sum_{i\mid q_i\ne0,d_i\ne0}\bigl(|q_i-d_i|^p-|q_i|^p-|d_i|^p\bigr),
\end{aligned}\tag{5}
$$

因为查询与库向量已归一化（$\|q\|_p^p=\|d\|_p^p=1$）。

> **此即 S1 式(11.9) 的来源**！取 $p=1$：$\|q-d\|_1=2+\sum_{q_i\ne0,d_i\ne0}(|q_i-d_i|-|q_i|-|d_i|)=2-\sum_{q_i\ne0,d_i\ne0}(|q_i|+|d_i|-|q_i-d_i|)$。S1 式(11.9) 给的 $\tfrac12\sum(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|)$ 就是 S2 式(4) 的 $s=1-\tfrac12\|q-d\|_1$ 中的"重叠量"。三源在此完全自洽。

这使**倒排文件可用**：对每个非零查询维 $q_i\ne0$，用倒排文件遍历对应的非零库项 $d_i\ne0$ 累加求和。查询通过"填充表示查询的树"实现（既算 $q_i$ 又排序）。用虚拟倒排文件时库维 $d_i$ 进一步碎片化为多个独立部分 $d_{ij}$，$d_i=\sum_j d_{ij}$（$d_{ij}$ 来自叶子倒排文件）。标量积对 $d_i$ 线性、易划分；其他范数更复杂——最佳做法是先合成 $d_i$（为每个库图像记住"最后触及的节点 $i$ 及累积的 $d_i$"），再用于式(5)。

**$L_2$ 范数的进一步简化（式 6）**：

$$
\boxed{\,\|q-d\|_2^2 = 2 - 2\sum_{i\mid q_i\ne0,d_i\ne0} q_i d_i\,}\tag{6}
$$

> 对应 S4 DBoW2 L2Scoring 源码注释 `||v - w||_{L2} = sqrt( 2 - 2 * Sum(v_i * w_i) )`。

## N-5. 结果（§6，核心结论 + Table 1 全量）

库由"已知关系的图像子集"组成。**带真值的图像集含 6376 图，每 4 个一组**（图 5）；用每图查询，质量度量基于"同组其余 3 图"的表现。也用约 **1400** 图子集对比非层次方案。

**度量哲学（§6 关键）**：自然要用匹配关键点的几何做**后验证（post-verification）**提升质量。但对极大库（如 20 亿图），后验证须从磁盘 $n$ 个随机位置读 top-$n$ 图，磁盘寻道约 10 ms，每盘每秒仅约 100 图——故**初始查询必须把正确图像放到最顶部**。本文**聚焦初始查询结果**，默认（很不宽容的）度量：**"每组其余 3 图被完美找到的百分比"**。

> **图 6**：1400 图库的检索曲线（top $x$% vs 命中 $y$%，到库 5%）。**关键是曲线与 $y$ 轴相交处**（验证只对极小部分库可行）。结论：(a) **更大词汇表改善检索**；(b) **$L_1$ 范数优于 $L_2$**；(c) **熵权重重要**（至少对小词汇表）。最佳是 **method A**，远胜 [17] 的设定 T。

> **TABLE 1 — 各设定下"完美检索"百分比**（即图 6 各方法与 $y$ 轴交点；全量保留）：
>
> 列含义：**Me**=评分方法 A–W；**En**=查询/库是否用熵权重（y/n）；**No**=式(3) 归一化差用的范数；**S%**=停用表上的视觉单词百分比（m=最频繁、l=最不频繁）；**Voc-Tree**=词汇树形状（层数 $L$ × 分支 $k$ = 叶子数）；**Le**=层次评分用的层数（从叶子起）；**Eb**=节点赋熵方法（i=图像频率/v=视觉单词频率定义 $N_i$；r=相对根节点/p=相对父节点）；**Perf**=性能 %。
>
> | Me | En | No | S% | Voc-Tree | Le | Eb | Perf |
> | --- | --- | --- | --- | --- | --- | --- | --- |
> | **A** | y/y | L1 | 0 | 6×10=1M | 1 | ir | **90.6** |
> | B | y/y | L1 | 0 | 6×10=1M | 1 | vr | 90.6 |
> | C | y/y | L1 | 0 | 6×10=1M | 2 | ir | 90.4 |
> | D | n/y | L1 | 0 | 6×10=1M | 2 | ir | 90.4 |
> | E | y/n | L1 | 0 | 6×10=1M | 2 | ir | 90.4 |
> | F | n/n | L1 | 0 | 6×10=1M | 2 | ir | 90.4 |
> | G | n/n | L1 | 0 | 6×10=1M | 1 | ir | 90.2 |
> | H | y/y | L1 | m2 | 6×10=1M | 1 | ir | 90.0 |
> | I | y/y | L1 | 0 | 6×10=1M | 3 | ir | 89.9 |
> | J | y/y | L1 | 0 | 6×10=1M | 4 | ir | 89.9 |
> | K | y/y | L1 | 0 | 6×10=1M | 2 | vr | 89.8 |
> | L | y/y | L1 | 0 | 6×10=1M | 2 | ip | 89.0 |
> | M | y/y | L1 | m5 | 6×10=1M | 1 | ir | 89.1 |
> | N | y/y | **L2** | 0 | 6×10=1M | 1 | ir | 87.9 |
> | O | y/y | L2 | 0 | 6×10=1M | 2 | ir | 86.6 |
> | P | y/y | L1 | l10 | 6×10=1M | 2 | ir | 86.5 |
> | Q | y/y | L1 | 0 | 1×10K=10K | 1 | - | 86.0 |
> | R | y/y | L1 | 0 | 4×10=10K | 2 | ir | 81.3 |
> | S | y/y | L1 | 0 | 4×10=10K | 1 | ir | 80.9 |
> | **T** | y/y | **L2** | 0 | 1×10K=10K | 1 | - | **76.0** |
> | U | y/y | L2 | 0 | 4×10=10K | 1 | ir | 74.4 |
> | V | y/y | L2 | 0 | 4×10=10K | 2 | ir | 72.5 |
> | W | n/n | L2 | 0 | 1×10K=10K | 1 | - | 70.1 |
>
> 设定 T 即 [17, Sivic-Zisserman] 用的（10K 单词、L2、无层次），最佳 A（1M 单词、L1、熵权重）显著更好（90.6 vs 76.0）。

> **图 7**：词汇树形状对 6376 真值集的影响。左：性能 vs 叶子数（分支 $k=8,10,16$）；右：性能 vs $k$（1M 叶子）。**性能随叶子数显著上升，随分支因子上升但不剧烈**。

> **图 8**：无监督训练的影响。左：性能 vs 训练数据量（720×480 帧数，20 训练周期）；右：性能 vs 训练周期数（7K 帧）。训练用的视频与库**完全分离**，6×10 词汇树在 6376 真值集测试。

> **图 9**：性能 vs 库大小（到 1M 图）。词汇树用与库分离的视频定义。展示两种熵权重定义方式（最重要的是"用与库独立的视频定义熵"）。**分数随库增大而下降**（更多图混淆）。

**大规模演示**：40000 图 CD 封面库实时识别（图 10），测试到 **1M 图**库（图 9）——比已知任何同类工作大 >1 数量级。6376 真值集嵌入含 7 部电影（Bourne Identity、Matrix、Braveheart、Collateral、Resident Evil、Almost Famous、Monsters Inc）所有帧的库。8 GB 机器 RAM 查询约 1 s/次，建库（主要特征提取）2.5 天。

**结论（§7）**：索引方案远强于当时 SOTA，建于"层次量化描述子的词汇树"；检索质量随更大词汇表、$L_1$ 范数改善；做了 40K CD 封面实时演示、1M 图库查询计时。有望成为互联网规模的内容图像搜索引擎。

---

# 第四部分（S4）DBoW2 官方实现 —— 评分公式与 API（权威工程对照）

> 源：https://github.com/dorian3d/DBoW2 （`include/DBoW2/BowVector.h`、`src/ScoringObject.cpp`、README）。这是 Galvez-Lopez 本人维护的 TRO2012 论文实现，可作为公式落地的权威对照。

## DB-1. 库结构与 API（README）

- **两个核心模板类**（README 原文）：
  ```cpp
  template<class TDescriptor, class F> class TemplatedVocabulary { ... };
  template<class TDescriptor, class F> class TemplatedDatabase  { ... };
  ```
  `TDescriptor` 是描述子数据类型，`F` 是派生自 `FClass` 的操作类。
- **TemplatedVocabulary**：把图像转成 bag-of-words 向量；**TemplatedDatabase**：索引图像。
- **两种索引**：
  - **Direct File（正向索引）**：库内**快速特征比较**（即论文 §IV-E direct index）。
  - **Inverted Index（逆向索引）**：**快速查询**、与已索引图像匹配。
- **支持的描述子**：
  - **ORB**：`cv::Mat`（CV_8UC1），描述子类 `FORB`。
  - **BRIEF**：`boost::dynamic_bitset<>`，描述子类 `FBrief`。
  - 预定义类型：`OrbVocabulary, OrbDatabase, BriefVocabulary, BriefDatabase`（`include/DBoW2/DBoW2.h` 中的 typedef）：
    ```cpp
    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>   OrbVocabulary;
    typedef DBoW2::TemplatedDatabase  <DBoW2::FORB::TDescriptor, DBoW2::FORB>   OrbDatabase;
    typedef DBoW2::TemplatedVocabulary<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefVocabulary;
    typedef DBoW2::TemplatedDatabase  <DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefDatabase;
    ```
- **评分缩放**：所有评分缩放到 **[0..1]**（README："scales all the scores to [0..1]"）。
- **性能（README）**："3 ms to convert the BRIEF features of an image into a bag-of-words vector and 5 ms to look for image matches in a database with more than 19000 images."

**BibTeX（README）**：
```bibtex
@ARTICLE{GalvezTRO12,
  author={Gálvez-López, Dorian and Tardós, J. D.},
  journal={IEEE Transactions on Robotics},
  title={Bags of Binary Words for Fast Place Recognition in Image Sequences},
  year={2012}, month={October}, volume={28}, number={5},
  pages={1188--1197}, doi={10.1109/TRO.2012.2197158}
}
```

## DB-2. 权重与评分枚举（`include/DBoW2/BowVector.h`，逐字）

```cpp
/// Weighting type
enum WeightingType { TF_IDF, TF, IDF, BINARY };

/// Scoring type
enum ScoringType  { L1_NORM, L2_NORM, CHI_SQUARE, KL, BHATTACHARYYA, DOT_PRODUCT };

/// Vector of words to represent images
class BowVector : public std::map<WordId, WordValue> { ... };
// WordId = unsigned int, WordValue = double
```

`BowVector` 是从单词 ID 到单词权值（double）的稀疏映射（`std::map`），即论文中的稀疏 BoW 向量 $\boldsymbol v$。

## DB-3. 六种评分函数的精确实现（`src/ScoringObject.cpp`，逐式还原）

> 全部对归一化（或等价处理的）BoW 向量计算，结果缩放到 [0..1]（KL、DotProduct 除外，源注 "cannot be scaled" / "cannot scale"）。下式按源码逻辑写出（$v_i,w_i$ 为两 BoW 向量在共同非零维上的权值）。

**1) L1_NORM（论文式 4 的实现）**：源码累加 `score += fabs(vi - wi) - fabs(vi) - fabs(wi)`（仅共同非零维），末尾 `score = -score/2.0`：

$$
s_{L1}(\boldsymbol v,\boldsymbol w)=-\frac12\sum_{i\mid v_i\ne0,w_i\ne0}\bigl(|v_i-w_i|-|v_i|-|w_i|\bigr)=1-\frac12\left\|\frac{\boldsymbol v}{\|\boldsymbol v\|}-\frac{\boldsymbol w}{\|\boldsymbol w\|}\right\|_1\in[0,1].
$$

（与 S2 式(4)、S1 式(11.9)、S3 式(5) at $p{=}1$ 完全一致。）

**2) L2_NORM（论文/ S3 式 6 的实现）**：源码 `score += vi * wi`（共同非零维），末尾 `score = 1.0 - sqrt(1.0 - score)`，源注 `||v - w||_{L2} = sqrt( 2 - 2 * Sum(v_i * w_i) )`：

$$
s_{L2}(\boldsymbol v,\boldsymbol w)=1-\sqrt{1-\sum_{i\mid v_i\ne0,w_i\ne0}v_i w_i}\ \ \left(=1-\tfrac12\|q-d\|_2,\ \text{其中}\ \|q-d\|_2^2=2-2\textstyle\sum v_iw_i\right)\in[0,1].
$$

**3) CHI_SQUARE（卡方）**：源码 `if(vi+wi!=0) score += vi*wi/(vi+wi)`，末尾 `score = 2.*score`：

$$
s_{\chi^2}(\boldsymbol v,\boldsymbol w)=2\sum_{i\mid v_i+w_i\ne0}\frac{v_i w_i}{v_i+w_i}\in[0,1].
$$

> 这是 $\chi^2$ 相似度形式（与 $\chi^2$ 距离 $\sum\frac{(v_i-w_i)^2}{v_i+w_i}$ 互补：$\sum\frac{(v_i-w_i)^2}{v_i+w_i}=\sum(v_i+w_i)-4\sum\frac{v_iw_i}{v_i+w_i}=2-2s_{\chi^2}$ 当两向量 L1 归一化）。

**4) KL（KL 散度，越小越像，源注 "cannot be scaled"）**：`LOG_EPS = log(DBL_EPSILON)`；共同非零维 `score += vi*log(vi/wi)`；仅 $v$ 非零（$w_i\approx0$）维 `score += vi*(log(vi) - LOG_EPS)`：

$$
d_{KL}(\boldsymbol v\,\|\,\boldsymbol w)=\sum_{i\mid v_i\ne0,w_i\ne0}v_i\log\frac{v_i}{w_i}+\sum_{i\mid v_i\ne0,w_i=0}v_i\bigl(\log v_i-\log\epsilon\bigr).
$$

> ⚠️ KL 是**距离/散度（越小越相似）**，不缩放到 [0,1]，方向与其余相似度相反。

**5) BHATTACHARYYA（巴氏系数，已天然缩放）**：`score += sqrt(vi*wi)`：

$$
s_{BC}(\boldsymbol v,\boldsymbol w)=\sum_{i\mid v_i\ne0,w_i\ne0}\sqrt{v_i w_i}\in[0,1].
$$

**6) DOT_PRODUCT（点积，源注 "cannot scale"）**：`score += vi*wi`：

$$
s_{\cdot}(\boldsymbol v,\boldsymbol w)=\sum_{i\mid v_i\ne0,w_i\ne0}v_i w_i.
$$

> **DBoW2 默认**：`L1_NORM` 评分 + `TF_IDF` 权重（与 S1 的 DBoW3 输出 `Scoring = L1-norm, Weighting = tf-idf` 一致）。论文（S3 Table 1）证明 L1 优于 L2，故默认 L1。

---

# 第五部分（S5）深度学习地点识别简介 —— VLAD / NetVLAD

> 源：Arandjelović et al. "NetVLAD" CVPR 2016（arXiv:1511.07247），§3.1。对应 S1 §11.5.5 提到的"BoW 改进形式 VLAD 的 CNN 实现 [115,116]"。供"深度学习地点识别简介"小节用。

## V-1. 任务设定（§2 overview）

地点识别建模为**图像检索**：用函数 $f$ 把库图像 $\{I_i\}$ 离线表示为 $\{f(I_i)\}$，查询 $q$ 在线表示 $f(q)$；测试时按 $f(q)$ 与 $f(I_i)$ 的**欧氏距离 $d(q,I_i)$** 排序找最近库图像（精确或快速 ANN）。NetVLAD **端到端学习表示** $f_\theta(I)$（参数 $\theta$），欧氏距离 $d_\theta(I_i,I_j)=\|f_\theta(I_i)-f_\theta(I_j)\|$ 也依赖 $\theta$。固定距离为欧氏、求显式特征映射 $f_\theta$。

## V-2. VLAD（Vector of Locally Aggregated Descriptors）

VLAD 是流行的描述子池化法。**词袋只保留视觉单词计数，VLAD 存每个视觉单词的"残差之和"（描述子与其对应簇心之差向量）**。

给定 $N$ 个 $D$ 维局部描述子 $\{x_i\}$ 与 $K$ 个簇心（视觉单词）$\{c_k\}$，VLAD 表示 $V$ 是 $K\times D$ 维（写成 $K\times D$ 矩阵，归一化后转向量）。$V$ 的 $(j,k)$ 元素：

$$
\boxed{\,V(j,k)=\sum_{i=1}^{N} a_k(x_i)\,\bigl(x_i(j)-c_k(j)\bigr)\,}\tag{1}
$$

其中 $x_i(j),c_k(j)$ 是第 $i$ 描述子、第 $k$ 簇心的第 $j$ 维；**$a_k(x_i)$ 是 $x_i$ 对第 $k$ 视觉单词的隶属度**——$c_k$ 是离 $x_i$ 最近簇时为 1，否则为 0（**硬分配 hard assignment**）。即 $V$ 的第 $k$ 列记录"分配到 $c_k$ 的描述子的残差 $(x_i-c_k)$ 之和"。$V$ 先**列内 L2 归一化（intra-normalization）**，转向量，最后**整体 L2 归一化**。

## V-3. NetVLAD：可微的广义 VLAD 层

VLAD 的不连续源于**硬分配** $a_k(x_i)$。为使可微，替换为**软分配（soft assignment）**：

$$
\boxed{\,\bar a_k(x_i)=\frac{e^{-\alpha\|x_i-c_k\|^2}}{\sum_{k'}e^{-\alpha\|x_i-c_{k'}\|^2}}\,}\tag{2}
$$

按"$x_i$ 与各簇心的接近程度"分配权重，$\bar a_k(x_i)\in(0,1)$，最近簇心权重最高。**$\alpha$ 是正常数**，控制响应随距离衰减的快慢；$\alpha\to+\infty$ 时退化为硬分配（原 VLAD）。

> **论文后续（§3.1，本抽取据 §3.1 文与图 2）**：展开 $\|x_i-c_k\|^2$ 中与 $k$ 无关项相消，软分配可写成 $\bar a_k(x_i)=\dfrac{e^{w_k^\top x_i+b_k}}{\sum_{k'}e^{w_{k'}^\top x_i+b_{k'}}}$（$w_k=2\alpha c_k$，$b_k=-\alpha\|c_k\|^2$），即一个 **1×1 卷积 + softmax**。代入式(1) 得 NetVLAD 池化（论文式 4）：$V(j,k)=\sum_i \bar a_k(x_i)\,(x_i(j)-c_k(j))$，其中 $\{w_k\},\{b_k\},\{c_k\}$ 是**可学习参数**（这是 NetVLAD 相对 VLAD 的关键解耦——分配的 $\{w_k,b_k\}$ 与残差中心 $\{c_k\}$ 独立学习）。

**架构（图 2）**：CNN 截到最后卷积层得 $H\times W\times D$ 图，视为 $N=H\times W$ 个 $D$ 维局部描述子 $x$；接 NetVLAD 层（1×1×D×K 卷积 → softmax 软分配 $s$，与 VLAD core $c$ 聚合得 $V$，列内归一化 + 整体 L2 归一化）输出 $(K\times D)\times1$ 的 VLAD 向量。整个图是有向无环图，可标准 CNN 层实现、端到端反向传播。

**弱监督训练（§4 概述）**：用 Google Street View Time Machine 弱标注影像，**弱监督排序损失（triplet 思想）**——同地点（近 GPS）图像比不同地点图像距离更近。最终表示对视角和光照变化鲁棒（图 1：夜/昼、人车遮挡仍正确识别）。

> **与本书衔接**：NetVLAD 输出全局描述子 $f(I)\in\mathbb R^{KD}$，可直接替换 BoW 向量做地点识别/回环检索（欧氏距离或 ANN）。这呼应 S1 §11.5.5 的预言"深度学习方法有望打败传统人工特征词袋"。其他深度地点识别（综述层面）：基于 CNN 全局描述子（NetVLAD、AP-GeM）、序列法（SeqSLAM）、近年 Transformer/foundation 特征（如 AnyLoc，2023+）。本抽取仅给 VLAD/NetVLAD 公式骨架，综合 agent 可据本书定位补充简述。

---

# 第六部分 综合：回环检测的完整流水线 + 与位姿图后端（`ch:nlopt`）的接口

> 本部分整合各源，给综合 agent 一个"自包含书章"的结构骨架。所有公式均已在前文给出。

## 6.1 回环检测在 SLAM 中的位置与作用

- **作用一：消累积漂移**（S1 §11.1.1）。前端只给相邻帧约束 $x_i-x_{i+1}$，误差逐帧累积；回环边 $x_i-x_j$（$|i-j|$ 大）把漂移"拉回"正确位置。
- **作用二：给位姿图加约束**（S1 §11.1.1 + ORB-SLAM）。回环边是位姿图（pose graph）中的一条额外边/弹簧，提供全局一致性。**位姿图优化的数学（误差项、信息矩阵 $\Sigma$、高斯-牛顿/LM、Schur）属本书 `ch:nlopt` 章**，本章只负责"产生回环约束"。
- **作用三：重定位**（S1 §11.1.1）。当前帧与历史地图关联，可在已建图中定位。

## 6.2 外观式回环检测完整流水线（融合 S1 + S2）

1. **关键帧选取**（S1 §11.5.3）：稀疏、互不太相同、涵盖环境。
2. **特征提取**：ORB / BRIEF+FAST（二进制，Hamming 距离）或 SIFT/SURF（欧氏）。
3. **离线词典训练**（S1 §11.3 + S3 §3 + S2 §IV）：层次 k-means（++）建 $k$ 叉、$L$ 层词汇树（$W=k^L$ 叶子=单词）；二进制描述子取整质心。赋 idf 权重 $w_i=\log(N/n_i)$（式 11.5 / S2(2) / S3(4)）。
4. **图像→BoW 向量**（S1 §11.4 + S2 §IV）：描述子从根到叶（每层选最小 Hamming/最近簇）→ tf-idf 权重 $v_{ti}=\mathrm{tf}\cdot\mathrm{idf}$（式 11.6–11.8 / S2(3)）→ 稀疏向量 $\boldsymbol v_t$。
5. **数据库检索**（S2 §IV-D）：用**逆向索引**只比较有共同单词的图像；L1 评分 $s(\boldsymbol v_1,\boldsymbol v_2)=1-\tfrac12\|\hat v_1-\hat v_2\|_1$（式 11.9 / S2(4) / S3(5)）排序。
6. **先验归一化**（S1 §11.5.2 + S2 §V）：$\eta=s(\boldsymbol v_t,\boldsymbol v_{t_j})/s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})$（式 11.10 / S2(5)），阈值 $\alpha$（或"3 倍"判据）；跳过转弯等 $s(\boldsymbol v_t,\boldsymbol v_{t-\Delta t})$ 小的帧。
7. **岛分组**（S1 §11.5.3 + S2 §V）：把时间相近匹配合并为 island，岛评分 $H=\sum_j\eta$（式 6），选最高岛。
8. **时间一致性**（S1 §11.5.4 + S2 §V）：要求与 $k$（如 3）个先前匹配的区间接近重叠；通过则取岛内 $\eta$ 最大者为回环候选。
9. **几何验证**（S1 §11.5.4 + S2 §V）：用**正向索引**只对同单词特征做对应（避免 $\Theta(n^2)$），RANSAC 求基础矩阵 $F$（≥12 对应支持）；通过则接受回环。
10. **输出回环约束**：由 $F$（或对应点重三角化/PnP）得相对位姿 $T_{ij}$，作为回环边送入位姿图后端（`ch:nlopt`）。

## 6.3 感知混叠与准召率（融合 S1 §11.1.2–11.1.3 + S2）

- **感知混叠（perceptual aliasing）= 假阳性**：不同地点外观相似（S2 多处强调；S1 图 11-2 左）。词袋"不在意单词顺序"加剧之（S1 §11.5.4）。
- **感知变异 = 假阴性**：同地点外观因光照/视角差异很大（S1 图 11-2 右）。
- **准召率**（式 11.2）、PR 曲线（图 11-3 / S2 图 4–5）。**SLAM 偏重高准确率**（假阳性会毁掉位姿图，S1 §11.1.3）；故严格阈值 + 几何/时间验证。S2 在三数据集均达 **100% 准确率**（Table I）。

## 6.4 L1 评分 / 范数差的三源统一推导（关键，集中放此处）

设两图归一化 BoW 向量 $\hat q=q/\|q\|_1,\ \hat d=d/\|d\|_1$（$\|\hat q\|_1=\|\hat d\|_1=1$，各分量 $\ge0$，因 tf-idf 权重非负）。由 S3 式(5)（$p=1$）：

$$
\|\hat q-\hat d\|_1=\sum_i|\hat q_i-\hat d_i|=2+\sum_{i:\hat q_i\ne0,\hat d_i\ne0}\bigl(|\hat q_i-\hat d_i|-\hat q_i-\hat d_i\bigr)=2-\sum_{i:\hat q_i\ne0,\hat d_i\ne0}\bigl(\hat q_i+\hat d_i-|\hat q_i-\hat d_i|\bigr).
$$

故 S2 式(4) 的 L1 评分

$$
s(q,d)=1-\tfrac12\|\hat q-\hat d\|_1=\tfrac12\sum_{i:\hat q_i\ne0,\hat d_i\ne0}\bigl(\hat q_i+\hat d_i-|\hat q_i-\hat d_i|\bigr),
$$

括号内即 S1 式(11.9) 的被求和项（"重叠量"）。三源（S1 式 11.9、S2 式 4、S3 式 5、S4 L1Scoring 源码）在此**完全自洽**：S3 给"范数差"（越小越像），S1/S2/S4 给"1 − 半范数差 = 评分"（越大越像）。**只有 $\hat q_i,\hat d_i$ 都非零的维（即两图共有的单词）才贡献评分**——这正是逆向索引高效的原因。

## 6.5 与 ORB-SLAM / 位姿图后端的接口（旁证综述）

> 来源：ORB-SLAM / ORB-SLAM2（Mur-Artal & Tardós, TRO2015 / 2017）综述检索；本书后端章 `ch:nlopt`。

- ORB-SLAM 的**地点识别器基于 DBoW2 + ORB**，实时回环闭合（即本文件 S2/S4 的直接应用）。
- **三线程**：tracking（运动 BA 定位）/ local mapping（局部 BA）/ **loop closing（检测大回环、做位姿图优化纠正漂移）**。
- **共视图（covisibility graph）**：连接观测共同地图点的关键帧；**本质图（essential graph）**：共视图的稀疏子图（高共视边 + 生成树 + 回环边），位姿图优化在其上进行。
- 回环闭合先算候选关键帧间的 **Sim(3)**（单目尺度漂移）或 SE(3)（双目/RGB-D）相似变换做 3D 对齐，搜索"welding window"共视关键帧强化关联，再在本质图上做**位姿图优化（PGO）**。
- **接口数据**：回环检测向后端提供 (a) 回环帧对 $(i,j)$；(b) 相对位姿约束 $T_{ij}$（含尺度，Sim3）；(c) 内点对应（供后续全局 BA）。**PGO/全局 BA 的目标函数、信息矩阵 $\Sigma$、求解（Gauss-Newton/LM、稀疏 Cholesky/Schur）见 `ch:nlopt`。**

---

# OCR 修正说明（本抽取发现并修正的错误）

> 主源 S1 是 MinerU OCR 转的中文 MD，存在若干 OCR 错误；S2/S3 是 pdftotext 抽取，公式排版有断行但内容完整。逐条记录修正（凡涉及公式/代码的均已在正文修正并标注）。

**S1（高翔 ch11）OCR 错误**：

1. **式(11.9) 求和作用域错误（数学性，重要）**：源 OCR 作 `s(v_A - v_B) = 2 \sum_{i=1}^N |v_Ai| + |v_Bi| - |v_Ai - v_Bi|`，求和号 $\sum$ 视觉上只罩住第一项 $|v_{Ai}|$。依据 Nister-Stewenius 式(5)（本文件 S3）与 DBoW2 实现（S4），**正确形式为求和罩住整个 $(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|)$**。已在 §11.4.1 修正并给出与 S2 式(4) 的等价推导。
2. **代码 `feature_training.cpp`**：源 OCR `descriptors.push_backubes。`（混入中文句号与乱码）→ 修正为 `descriptors.push_back(descriptor);`。
3. **代码 `loop_closure.cpp`**：源 OCR `vector(Mat> descriptors;` → `vector<Mat> descriptors;`；`detector->detectAndCompute(...descriptor); descriptors.push_backubesor)` → `descriptors.push_back(descriptor);`。
4. **代码注释断行**：源把 `// NOTE: ... this may lead to overfit.` 拆到代码块外，已并回注释行。
5. **路径不一致（保留原样并标注）**：源 §11.4.2 代码头注释写 `slambook/ch12/loop_closure.cpp`，但正文与目录均为 `slambook2/ch11/`。这是源自身的笔误（章号 11 vs 12、slambook vs slambook2），非 OCR；正文保留原文并在此标注，正确应为 `slambook2/ch11/loop_closure.cpp`。
6. **`$5^{\text{①}}$` 等脚注上标混入正文**：源行 281 "深度 $L$ 为 $5$" 后混入脚注标记，已清理为"深度 $L$ 为 5"。
7. **TF/IDF 符号复用（非错误但需提示）**：式(11.5)(11.6) 复用 $n,n_i$ 但含义不同（语料级 vs 图像级），已在正文加注并对照 S2 式(2)(3) 的清晰记号。

**S2（IROS2011 Galvez）pdftotext 断行修正**：

8. **式(1) BRIEF**：pdftotext 把 cases 环境与 $\forall i\in[1..L_b]$ 拆行、把约束区间 $\bigl[-\tfrac{S_b}{2}..\tfrac{S_b}{2}\bigr]^2$ 的负号/分式打散，已据上下文重排为完整 LaTeX。
9. **式(2)(3)(4)(5)(6)** 的分式（$\log\frac{N}{n_i}$、$\frac{n_{iI_t}}{n_{I_t}}$、$1-\frac12\|\cdot\|$、$\frac{s(\cdot)}{s(\cdot)}$、$\sum_{j=n_i}^{m_i}$）均有上下标错位，已逐式校正（与 DBoW2 源码 S4 交叉验证一致）。
10. **测试对分布** $N(0,\tfrac1{25}S_b^2)$、$N(x,\tfrac4{625}S_b^2)$ 的分数被 pdftotext 拆成多行（`1 2 / 25 Sb`），已重组。

**S3（Nister-Stewenius）pdftotext 断行修正**：

11. **描述子总数公式** $\sum_{i=1}^L k^i=\frac{k^{L+1}-k}{k-1}\approx k^L$ 被 pdftotext 拆成 `L ∑ i / i=1 k = / k^{L+1}-k / k-1`，已重组为完整分式。
12. **式(1)(2)(3)(4)** 的 $q_i=n_iw_i$、$d_i=m_iw_i$、$\|q/\|q\|-d/\|d\|\|$、$w_i=\ln\frac{N}{N_i}$ 排版错位，已校正（幻灯片源二次确认 $w_i=\ln(N/N_i)$、式(5)(6) 形式）。
13. **式(5)(6)** 多行求和下标 $i\mid q_i\ne0,d_i\ne0$ 被打散，已重组；Table 1 的列经幻灯片+正文交叉核对后逐行还原。

**S4/S5**：源码与 arXiv PDF 抽取干净，无实质 OCR 错误；S5 软分配式(2) 的指数 $e^{-\alpha\|x_i-c_k\|^2}$ 上标被 pdftotext 降行，已校正。

---

# 参考文献（本抽取实际引用的权威出处）

- **[S1]** 高翔、张涛 等《视觉SLAM十四讲：从理论到实践（第2版）》第 11 讲「回环检测」。本地：`视觉SLAM十四讲_md/11_回环检测.md`。
- **[S2]** D. Gálvez-López, J. D. Tardós, "Real-Time Loop Detection with Bags of Binary Words," *IEEE/RSJ IROS 2011*, pp. 51–58. http://webdiis.unizar.es/~jdtardos/papers/2011_IEEE_IROS_Galvez.pdf
- **[S2′]** D. Gálvez-López, J. D. Tardós, "Bags of Binary Words for Fast Place Recognition in Image Sequences," *IEEE Trans. on Robotics*, vol. 28, no. 5, pp. 1188–1197, Oct. 2012. DOI: 10.1109/TRO.2012.2197158.
- **[S3]** D. Nistér, H. Stewénius, "Scalable Recognition with a Vocabulary Tree," *IEEE CVPR 2006*, vol. 2, pp. 2161–2168. https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/nister_stewenius_cvpr2006.pdf
- **[S4]** DBoW2 库（Dorian Galvez-López）。https://github.com/dorian3d/DBoW2 （及 DBoW3：https://github.com/rmsalinas/DBow3）
- **[S5]** R. Arandjelović, P. Gronat, A. Torii, T. Pajdla, J. Sivic, "NetVLAD: CNN architecture for weakly supervised place recognition," *IEEE CVPR 2016*. arXiv:1511.07247. https://arxiv.org/abs/1511.07247
- **[旁证]** J. Sivic, A. Zisserman, "Video Google: A Text Retrieval Approach to Object Matching in Videos," *ICCV 2003*（tf-idf + inverted index 之源，S2 ref [6]、S3 ref [17]）。
- **[旁证]** M. Cummins, P. Newman, "FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance," *IJRR* 27(6):647–665, 2008（S2 ref [24]；S1 ref [103-105]，含 Chow-Liu tree）。
- **[旁证]** R. Mur-Artal, J. D. Tardós, "ORB-SLAM2," *IEEE T-RO* 2017（DBoW2 在完整 SLAM 系统中的应用：covisibility/essential graph、Sim3、PGO）。
- **[旁证]** M. Calonder et al., "BRIEF," *ECCV 2010*（S2 ref [8]）；E. Rosten, T. Drummond, "FAST," *ECCV 2006*（S2 ref [9]）；D. Arthur, S. Vassilvitskii, "k-means++," *SODA 2007*（S2 ref [19]，S1 ref [102]）。
