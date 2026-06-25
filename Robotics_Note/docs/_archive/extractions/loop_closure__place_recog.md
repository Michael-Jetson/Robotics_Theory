# 抽取留痕 — 回环检测 / 地点识别（视觉词袋 DBoW2 · 相似度评分 · 时间一致性 · 几何验证 · 感知混叠 · 准确率/召回率 · NetVLAD）

> **本抽取服务章节**：`回环检测`（`\label{ch:loop}`，`parts/P2_slam/loop_closure.tex`，当前为占位章，待综合）。
>
> **本章聚焦（来自任务书）须重点覆盖**：回环检测的作用（消累积漂移、给位姿图加约束、重定位）；视觉词袋完整（词典构建、TF-IDF、相似度评分）；关键帧/相似度处理与时间一致性；几何验证（RANSAC + 对极/PnP）；感知混叠（perceptual aliasing）与准确率/召回率权衡；与位姿图后端（`\cref{ch:nlopt}` 之 `sec:nlopt-posegraph`）的接口；深度学习地点识别（NetVLAD）简介。
>
> **来源（联网研究，权威原始文献，均已逐行读取）**：
> 1. **DBoW2（主源，视觉词袋回环检测的奠基工作）**：D. Gálvez-López and J. D. Tardós, *"Bags of Binary Words for Fast Place Recognition in Image Sequences"*, **IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188–1197, Oct. 2012**. PDF: <http://doriangalvez.com/papers/GalvezTRO12.pdf>（已下载并逐字读完，654 行抽取文本；含全部式 (1)–(4)、词汇树参数、时间一致性 k、几何验证 RANSAC 与直接索引）。该文即 DBoW2 库，被 ORB-SLAM/ORB-SLAM2/3、VINS-Mono、LeGO-LOAM 用作回环前端。
> 2. **NetVLAD（深度学习地点识别，主源）**：R. Arandjelović, P. Gronat, A. Torii, T. Pajdla, J. Sivic, *"NetVLAD: CNN architecture for weakly supervised place recognition"*, **CVPR 2016**（扩展版 IEEE TPAMI 2018）. arXiv:1511.07247 <https://arxiv.org/abs/1511.07247>（已下载 PDF 并 pdftotext 逐字读完 1116 行；含 VLAD 式 (1)、软分配式 (2)(3)、NetVLAD 层式 (4)、弱监督三元组排序损失式 (5)(6)(7)）。
> 3. **视觉SLAM十四讲（第二版）第 11 讲 回环检测（教学化二级源，本章 Plan B 的教材底本）**：高翔、张涛 等，《视觉SLAM十四讲：从理论到实践》第二版，电子工业出版社。键名 `gaoxiang2019slam14`。本讲给出准确率/召回率、混淆矩阵（真阳性/假阳性=感知偏差/真阴性/假阴性=感知变异）、TF-IDF、L1 相似度评分、先验相似度归一化（3 倍阈值）、关键帧处理、一致性检测。CSDN/Zhihu 直连被反爬（521/403），公式经多处镜像（cnblogs CV-life 综述、CSDN warningm_dm/weixin_46034116、知乎 p/420617414）交叉确认，已逐字对齐；下文凡标“十四讲 §11.x”均来自此书正文。
> 4. **FAB-MAP（感知混叠的概率建模奠基，补充源）**：M. Cummins and P. Newman, *"FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance"*, **IJRR 2008**（cnblogs CV-life 综述逐字引其摘要：显式建模 perceptual aliasing）。
> 5. **VPR 综述（补充源，给地点识别问题定义与流水线分层）**：P. Yin et al., *"General Place Recognition Survey: Towards Real-World Autonomy"*, arXiv:2209.04497（已下载 PDF 并 pdftotext 通读 1478 行）。
> 6. **ORB-SLAM（回环工程实现，补充）**：R. Mur-Artal, J. M. M. Montiel, J. D. Tardós, *"ORB-SLAM: a Versatile and Accurate Monocular SLAM System"*, IEEE T-RO 2015, arXiv:1502.00956（VINS/loam_family 抽取已引；本文取其“3 次连续共视关键帧”一致性与“回环后 7-DOF / SE(3) 位姿图 + 全局 BA”）。
>
> **保真承诺**：本文件按各源小节顺序，逐条记录每一个定义、每一条公式（LaTeX 写全、保留原编号）、每一步推导（中间代数不跳）、每一道数值例/实验表、每一段算法伪码。不做摘要、不做凝练。DBoW2 的式 (1)–(4) 与全部参数（kw=10、Lw=6、1M 单词、时间一致性 k=3、几何验证 ≥12 对应点、直接索引层 DI0–DI3）逐项展开；NetVLAD 的软分配从式 (2) 展开平方到式 (3) 的代数逐行给出。
>
> **与本书已有抽取的去重说明**：`vio__vins.md` §VII 已抽 VINS-Mono 用 DBoW2 + 2 步几何验证（2D-2D 基础矩阵 RANSAC、3D-2D PnP RANSAC）+ 4-DOF 位姿图；`lidar_slam__loam_family.md` 已抽 LeGO-LOAM（ICP 回环 + iSAM2）与 LIO-SAM（欧氏距离回环因子）。**本抽取聚焦“视觉地点识别流水线本身的完整推导”**（词袋全链路、相似度归一化、岛屿/组匹配、时间一致性、感知混叠、P-R 权衡、NetVLAD），与上两者互补；引用处会明确指向。

---

## 0. 记号约定（各源）+ 与本书统一约定的差异

本章涉及多篇论文与一本教材，记号体系不同。先列**与本书统一约定的全局对照**，再逐源列细节。

### 0.0 本书统一约定（`parts/front/notation.tex`，编写规范 §五）

- 旋转矩阵 $\mathbf R\in\mathrm{SO}(3)$；位姿 $\mathbf T\in\mathrm{SE}(3)$；$\mathbf R_{ab},\mathbf t_{ab}$ 把 $b$ 系坐标变到 $a$ 系（$\mathbf p_a=\mathbf R_{ab}\mathbf p_b+\mathbf t_{ab}$）。
- 四元数：Hamilton，实部在前 $\mathbf q=[w,\mathbf v]$。
- **扰动以右扰动为主**：$\mathbf R\,\mathrm{Exp}(\delta\boldsymbol\phi)$。
- $\mathfrak{se}(3)$ 排序 $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$（平移在前）。
- 协方差 $\boldsymbol\Sigma$，信息矩阵 $\boldsymbol\Lambda=\boldsymbol\Sigma^{-1}$；过程/观测噪声 $\boldsymbol\Sigma_{\mathbf w}/\boldsymbol\Sigma_{\mathbf v}$。
- 马氏平方范数 $\lVert\mathbf e\rVert_{\boldsymbol\Sigma}^2=\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf e$。

**全局提醒（本章语义）**：回环检测的“输出”是一条**位姿图回环边**（相对位姿约束 $\mathbf T_{ij}$ + 信息矩阵 $\boldsymbol\Lambda_{ij}$），它直接喂给 `\cref{sec:nlopt-posegraph}` 的位姿图目标函数（本书 `eq:nlopt-pg-cost`，残差 $\mathbf e_{ij}=\mathrm{Log}(\mathbf T_{ij}^{-1}\mathbf T_i^{-1}\mathbf T_j)$）。本章前 90% 讲的是“怎样**正确地**检出 $(i,j)$ 这一对回环”；几何验证产出的相对位姿就是这条边的测量值。**§7 接口** 专门讲这条对接。

### 0.1 DBoW2 记号（源 §III–§V，Gálvez-López & Tardós 2012）

| 记号 | 含义（DBoW2 用法） |
|---|---|
| $\mathbf B(p)$ | 点 $p$ 的 BRIEF 二进制描述子（向量），$B^i(p)$ 为第 $i$ 位 |
| $L_b,\ S_b$ | BRIEF 描述子长度（测试次数）/ patch 尺寸 |
| $a_i,b_i$ | 第 $i$ 个测试的两个像素相对 patch 中心的 2D 偏移 |
| $k_w$ | 词汇树每个节点的分支数（branches） |
| $L_w$ | 词汇树的深度（层数，depth levels） |
| $W$ | 词汇表的单词总数（= 叶子数 = $k_w^{L_w}$ 量级） |
| $w_i$ | 第 $i$ 个**单词**（visual word，词汇树的叶节点） |
| tf-idf | 单词权重方案（term frequency–inverse document frequency） |
| $I_t,\ v_t$ | $t$ 时刻图像、其词袋向量 $v_t\in\mathbb R^W$ |
| $s(v_1,v_2)$ | 两词袋向量的 $L_1$-相似度评分，$\in[0,1]$（式 (2)） |
| $\eta(v_t,v_{t_j})$ | 归一化相似度评分（式 (3)） |
| $\Delta t$ | 相邻图像（上一帧）的时间间隔 |
| $\alpha$ | 归一化评分的接受阈值 |
| $T_i,\ V_{T_i}$ | 时间区间 $t_{n_i},\dots,t_{m_i}$ / 把这些匹配聚成一组的**岛屿**（island） |
| $H(v_t,V_{T_i})$ | 岛屿评分（式 (4），岛内 $\eta$ 之和） |
| $k$ | **时间一致性**所需的连续匹配数（本文取 $k=3$） |
| $l$ | 词汇树层级（叶=0，根=$L_w$）；几何验证用直接索引在第 $l$ 层取祖先节点 |
| $f_{tj}$ | 图像 $I_t$ 第 $j$ 个局部特征（存于直接索引） |

**DBoW2 与本书差异**：(1) DBoW2 的 $w$ 指**单词**，与本书 $w$（权重矩阵/像素列）冲突，本抽取保留 DBoW2 原义并在每处标“单词 $w_i$”。(2) DBoW2 的 $v$ 指**词袋向量**（稀疏 $\mathbb R^W$），与本书 $\mathbf v$（速度）冲突，本抽取标“词袋向量 $v_t$”。(3) DBoW2 几何验证求**基础矩阵 $\mathbf F$**（对极几何，见本书 `\cref{ch:vo}`），与本书 $\mathbf F$（无歧义）一致。

### 0.2 NetVLAD 记号（源 §3–§4，Arandjelović et al. 2016）

| 记号 | 含义（NetVLAD 用法） |
|---|---|
| $f_\theta(I)$ | 图像 $I$ 的（可学习参数 $\theta$ 的）全局描述子（特征图） |
| $d_\theta(I_i,I_j)=\lVert f_\theta(I_i)-f_\theta(I_j)\rVert$ | 两图欧氏距离（地点识别检索度量） |
| $\{x_i\}$ | $N$ 个 $D$ 维局部描述子（CNN 最后卷积层输出，$H\times W\times D$ 视作 $N\times D$） |
| $\{c_k\}$ | $K$ 个聚类中心（“visual words”），VLAD 参数 |
| $V$ | VLAD/NetVLAD 输出，$K\times D$ 矩阵（拉直为向量） |
| $V(j,k)$ | $V$ 的 $(j,k)$ 元（第 $k$ 簇、第 $j$ 维） |
| $a_k(x_i)$ | 硬分配（$x_i$ 属第 $k$ 簇为 1，否则 0） |
| $\bar a_k(x_i)$ | 软分配（式 (2)(3)） |
| $\alpha$ | 软分配温度（正常数，$\alpha\to+\infty$ 退化为硬分配） |
| $w_k,b_k$ | NetVLAD 可学习卷积权重/偏置（$w_k=2\alpha c_k$、$b_k=-\alpha\lVert c_k\rVert^2$） |
| $q,\ p_i^q,\ n_j^q$ | 训练查询 / 潜在正样本（地理近）/ 确定负样本（地理远） |
| $p_{i^\ast}^q$ | 最佳潜在正样本（式 (5)） |
| $L_\theta$ | 弱监督三元组排序损失（式 (7)）；$m$ 间隔（margin）；$l(x)=\max(x,0)$ 铰链 |

**NetVLAD 与本书差异**：(1) NetVLAD 的 $\alpha$ 是 softmax 温度，与本书 $\alpha$（步长/无固定义）无关，仅本节内有效。(2) NetVLAD 的 $c_k$（聚类中心）对应词袋的“单词”，但是**连续向量**而非离散叶子。(3) NetVLAD 的 $w_k,b_k$ 是卷积参数，再次与本书权重 $w$、DBoW2 单词 $w_i$ 同形不同义——本抽取每处标注。

### 0.3 十四讲第 11 讲记号（教材底本）

| 记号 | 含义 |
|---|---|
| $n$ | （IDF 语境）训练集所有特征总数；（TF 语境）单幅图像出现的单词总次数 |
| $n_i$ | 单词 $w_i$ 对应的特征数 / 出现次数 |
| $\mathrm{IDF}_i,\ \mathrm{TF}_i$ | 逆文档频率 / 词频 |
| $\eta_i$ | 单词 $w_i$ 的权重 $=\mathrm{TF}_i\cdot\mathrm{IDF}_i$ |
| $v_A$ | 图像 $A$ 的词袋向量 $\{(w_1,\eta_1),\dots,(w_N,\eta_N)\}$（稀疏） |
| $s(v_A,v_B)$ | 相似度评分（$L_1$ 形式，式见 §3） |
| $s(v_t,v_{t-\Delta t})$ | 先验相似度（当前关键帧与上一关键帧） |
| TP/FP/TN/FN | 真阳性/假阳性（感知偏差）/真阴性/假阴性（感知变异） |

**十四讲与本书差异**：十四讲用 $w_i$ 指单词、$\eta_i$ 指权重（与 DBoW2 一致），与本书 $\boldsymbol\eta$（信息向量）冲突，本抽取保留教材原义并标注。十四讲的 $s$ 评分写成“差异性求和”形式（$s$ 越大越相似），与 DBoW2 式 (2) 的“$1-\tfrac12\lVert\cdot\rVert_1$”是同一 $L_1$ 思想的两种书写，**§3.3 专门辨析二者的代数关系**（本书建议统一采用 DBoW2 的归一化 $[0,1]$ 形式）。

---

# 第一部分 · 回环检测的作用与问题定义

## 1. 为什么需要回环检测（十四讲 §11.1；VPR 综述 §I）

**累积漂移问题**：前端里程计（VO/LIO/VIO）对相邻帧相对运动逐帧积分，误差**单调累积**，长时间后轨迹与地图发生显著漂移；后端（BA/位姿图，本书 `\cref{ch:nlopt}`）只在**已有约束**下做最优估计，无法凭空消除“整段轨迹一致偏移”这种漂移——因为它没有“此处=彼处”的信息。

**回环检测的三大作用**（十四讲 §11.1）：
1. **消除累积漂移**：当机器人重访已到过的地点，检出“当前帧 $I_t$ 与历史帧 $I_{t'}$ 是同一地点”，给后端一条跨越长时间跨度的约束，把漂移“拉回”。
2. **给位姿图/后端加约束**：回环边连接图中相距很远的两个位姿节点，使位姿图从“带状（里程计链）”变成含长程边的图，全局优化后轨迹与地图**全局一致**（globally consistent）。这正是本书 `\cref{sec:nlopt-posegraph}` 中“球”位姿图里**层间回环边**的来源（`nonlinear_optimization.tex` §10.3 实践：2500 位姿、9799 边 = 里程计边 + 回环边）。
3. **重定位（relocalization）**：跟踪丢失后，用同一套地点识别机制把当前帧匹配到地图中的历史关键帧，恢复位姿。回环与重定位**共用地点识别引擎**，区别仅在“回环=已知大致在哪、找闭环”vs“重定位=完全不知在哪、全局查”。

**与前端/后端的关系**（十四讲 §11.1）：回环检测通常被视为**独立的第三个模块**（前端 / 后端 / 回环），自 PTAM 起前后端分线程，回环作为“很少运行的线程”（本书 `visual_odometry.tex` 四线程图：跟踪→局部建图→**回环闭合（每关键帧检测）**→完整 BA）。**“VO 与 SLAM 的关键差别就在有没有回环/全局地图”**（本书 `visual_odometry.tex` §1367 已立此论）。

## 2. 基于外观的地点识别：问题与三类方法（十四讲 §11.2；VPR 综述）

**核心问题**：如何判断“两幅图像来自同一地点”？检测回环的关键是**有效地检测出相机经过同一个地方**。三类思路（十四讲 §11.2）：

- **基于里程计几何关系（odometry-based）**：当估计的位姿距某历史位姿很近时，认为可能回环。**致命缺陷**：正因为漂移，估计位姿本身就不准——漂移越大越测不到回环，**而回环恰恰是为了消漂移**，逻辑上自相矛盾（在大漂移时失效）。（注：LeGO-LOAM/LIO-SAM 的“欧氏距离回环”属此类，靠低漂移 LIO 才可用，见 `lidar_slam__loam_family.md`。）
- **基于外观（appearance-based）**：**只根据两幅图像本身的相似性**判断回环，与前端/后端位姿估计**彻底解耦**。这是视觉 SLAM 回环检测的**主流**（十四讲 §11.2 明言“外观法已是主流”）。本章主体即此类。
- **基于概率/生成模型**：FAB-MAP（Cummins & Newman 2008）把词袋扩展为外观空间的概率 SLAM，**显式建模感知混叠**（见 §6.1），用贝叶斯滤波估回环概率。

**地点识别的输出与下游**（VPR 综述 §I）：VPR 方法**对感知混叠不鲁棒**，因此**下游需要几何验证才能产出准确回环**（"VPR methods … require downstream geometric verification to produce accurate loop closures"）。这奠定了本章流水线：**外观检索（召回候选）→ 一致性筛选 → 几何验证（保准确率）→ 输出位姿图边**。

---

# 第二部分 · 准确率/召回率与感知混叠（评价与权衡）

## 3'. 准确率、召回率与混淆矩阵（十四讲 §11.2）

> 本节是回环检测的**评价基石**，决定后面所有阈值（$\alpha$、$k$、几何验证内点数）的取舍方向。十四讲在讲算法前先立此评价框架。

**混淆矩阵**（confusion matrix）。把“算法判定”与“真值”交叉，得四格：

| | **真值：是回环** | **真值：不是回环** |
|---|---|---|
| **算法：是回环** | 真阳性 TP（true positive） | 假阳性 FP（false positive）= **感知偏差** |
| **算法：不是回环** | 假阴性 FN（false negative）= **感知变异** | 真阴性 TN（true negative） |

- **感知偏差 / 感知混叠（perceptual aliasing → 假阳性 FP）**：算法**误把不同地点判为同一地点**。两个外观相似但实为不同的地方（走廊、相似立面、重复纹理）被当作回环。
- **感知变异（perceptual variability → 假阴性 FN）**：算法**漏检真实回环**。同一地点因光照/视角/季节/动态物变化太大而未被识别。

**准确率（Precision）与召回率（Recall）定义**（十四讲 §11.2）：
$$
\boxed{\ \mathrm{Precision}=\frac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FP}}\ },\qquad
\boxed{\ \mathrm{Recall}=\frac{\mathrm{TP}}{\mathrm{TP}+\mathrm{FN}}\ }.
$$
- **准确率**：算法检出的“是回环”里，**真的是回环的比例**（检出的对不对）。
- **召回率**：所有真实回环里，**被算法检出的比例**（漏没漏）。

**SLAM 对二者的态度（关键工程结论，十四讲 §11.2）**：
**SLAM 中对准确率的要求远高于召回率**。理由：
- 一个**假阳性（FP）**会往位姿图里加一条**根本错误的回环边**，后端把整段轨迹/地图“掰”到错误位置，**灾难性**（本书 `nonlinear_optimization.tex` 故障表已记“回环处轨迹被拉飞=外点/误匹配主导二次代价”）。
- 一个**假阴性（FN，漏检）**只是“少消了一次漂移”，损失有限——后续重访还有机会再检出。

因此回环检测**宁可牺牲召回率，也要追求接近 100% 的准确率**（DBoW2 实验即报告“no false positives”，并以“100% 准确率下的召回率”作为核心指标，见 §5.4）。这也是 DBoW2 设计三道关卡（归一化阈值 $\alpha$ → 时间一致性 $k$ → 几何验证）层层把关、只保高置信回环的根本动机。

**准确率-召回率曲线（P-R curve）与 F 值**（十四讲 §11.2；DBoW2 §VI-B）：调节判定阈值（如 $\alpha$）可得一系列 $(\mathrm{Recall},\mathrm{Precision})$ 点，连成 **P-R 曲线**。
- 阈值严 → 准确率↑、召回率↓；阈值松 → 召回率↑、准确率↓（**此消彼长**）。
- 比较两算法：曲线**整体更靠右上**者更优（DBoW2 §VI-B：“SURF64 的曲线在两数据集上都压制 U-SURF128”）。
- 综合指标 **F 值（$F_1$ score）**= 准确率与召回率的调和平均：
$$
F_1=\frac{2\,\mathrm{Precision}\cdot\mathrm{Recall}}{\mathrm{Precision}+\mathrm{Recall}}.
$$
- SLAM 常用的实用单点指标：**最高准确率（100%）下的召回率**（recall at 100% precision）——因 SLAM 容不得 FP，故只关心“在保证零误检的前提下能检出多少回环”。

## 4'. DBoW2 的评价方法学（源 §VI-A，逐字）

DBoW2 §VI-A 明确写出常被忽略的评测细节（“little detail is given in the literature”），全量记录：
1. **数据集**（Table I，五个公开集，平面内运动）：
   | 数据集 | 相机 | 描述 | 总长 (m) | 重访长 (m) | 均速 (m/s) | 图像尺寸 (px) |
   |---|---|---|---|---|---|---|
   | New College | Frontal | 室外、动态 | 2260 | 1570 | 1.5 | 512×384 |
   | Bicocca 2009-02-25b | Frontal | 室内、静态 | 760 | 113 | 0.5 | 640×480 |
   | Ford Campus 2 | Frontal | 城市、微动态 | 4004 | 280 | 6.9 | 600×1600 |
   | Malaga 2009 Parking 6L | Frontal | 室外、微动态 | 1192 | 162 | 2.8 | 1024×768 |
   | City Centre | Lateral | 城市、动态 | 2025 | 801 | – | 640×480 |
2. **真值（ground truth）**：多数数据集不直接给回环，**手工创建实际回环列表**，每条是“查询区间↔匹配区间”的时间区间对。
3. **正确性度量**：用 precision/recall。“准确率=正确检出/所有触发的检出；召回率=正确检出/真值中所有回环事件”。一次触发=一对(查询时间戳, 匹配时间戳)；判 TP 时在真值里找包含这两时间戳的区间。真值中回环事件总数=所有查询区间长度 × 处理帧率。**一查询时间戳因多次重访对应多个匹配时，只算一个**。
4. **参数选取原则**：用**训练集**（NewCollege、Bicocca25b、Ford2）调参，用**独立评估集**（CityCentre、Malaga6L）当黑盒验证——**用不同数据调参与评估以证明鲁棒性**。
5. **统一设置（源 §VI-A-5，关键数值）**：所有实验**同一词汇树**：$k_w=10$ 分支、$L_w=6$ 层 → **一百万单词**；用独立数据集 Bovisa 的 **9M 特征 / 10K 图像**训练。FAST 响应阈值 10、SURF Hessian 阈值 500；每图只保留**响应最高的 300 个特征**。

---

# 第三部分 · 视觉词袋完整流程（DBoW2 主源 + 十四讲）

## 5. 二进制特征：FAST + BRIEF（DBoW2 §III，逐字）

DBoW2 用 **FAST 关键点 + BRIEF 描述子**（为实时、省内存而选二进制特征）。

- **FAST 关键点**：在半径 3 的 Bresenham 圆上比较若干像素灰度的角点；只查少数像素，故极快，适合实时。
- **BRIEF 描述子**：对每个 FAST 关键点取方形 patch，描述子每一位是 patch 内**两像素的灰度比较**结果；patch 先高斯平滑去噪。给定 patch 尺寸 $S_b$，测试像素对在离线阶段随机选定；另设 $L_b$=测试次数（=描述子长度）。点 $p$ 的 BRIEF 描述子第 $i$ 位（**源式 (1)**）：
$$
B^i(p)=
\begin{cases}
1 & \text{if } I(p+a_i) < I(p+b_i)\\[2pt]
0 & \text{otherwise}
\end{cases}
\qquad \forall i\in[1..L_b],
\tag{DBoW2-1}
$$
其中 $B^i(p)$ 为描述子第 $i$ 位，$I(\cdot)$ 为平滑图像中像素灰度，$a_i,b_i$ 为第 $i$ 个测试点相对 patch 中心的 2D 偏移，取值在 $\left[-\tfrac{S_b}{2}..\tfrac{S_b}{2}\right]\times\left[-\tfrac{S_b}{2}..\tfrac{S_b}{2}\right]$，预先随机选定。**该描述子不需训练**，只需离线随机选点（几乎不耗时）。二进制描述子用 **Hamming 距离**比较（对位异或后计 1 的个数），远快于 SIFT/SURF 的浮点欧氏距离——这是 DBoW2 实时性的根基。

> **记号对齐**：本书相机/特征章（`\cref{ch:vo}`）以 ORB（FAST + 带方向的 BRIEF）为主特征；DBoW2 原版用原始 BRIEF（无旋转/尺度不变），ORB-SLAM 的 DBoW2 词典则建在 ORB 上。综合时统一表述为“二进制描述子（ORB/BRIEF）+ Hamming 距离”。

## 6. 词典（视觉词汇）的构建（DBoW2 §IV + 十四讲 §11.3）

### 6.1 字典-单词-描述子的层级关系（十四讲 §11.3）

$$
\text{字典}\ \supset\ \text{单词}\ \supset\ \text{差距较小的描述子的集合}.
$$
**单词**（word）= 对大量描述子聚类后得到的一类（一个聚类中心代表的 Voronoi 胞），每个单词由“距离相近的描述子”组成。**字典**（dictionary/vocabulary）= 所有单词的集合。词袋的核心思想：**用“图像上有哪些单词”来描述一幅图像**，**只在乎单词有无、不在乎单词的排列顺序**（无序 bag）——这既带来对视角/平移的鲁棒，也埋下感知混叠的隐患（§6'）。

### 6.2 离线构建：层次 k-means / k-medians + 词汇树（DBoW2 §IV + 十四讲 §11.3）

**为什么要树**：字典动辄百万单词，一一线性查找匹配单词计算量巨大，故用 **$k$ 叉树**组织（十四讲 §11.3）。

**DBoW2 的离线构建（源 §IV，逐字）**：从一批**独立于在线数据**的训练图像提取丰富特征，然后：
1. 把所有描述子用 **k-medians 聚类（k-means++ 播种）**离散为 $k_w$ 个**二进制**簇；结果中**非二进制的中位数截断为 0**（保持单词是合法二进制描述子）。这些簇构成树的**第一层节点**。
2. 对每个节点关联的描述子**重复**上述操作，共做 $L_w$ 层。
3. 最终得到含 $W$ 个**叶子**的树，叶子即词汇表的**单词**。

**十四讲 §11.3 的等价表述（k 叉树流程，深度 $d$）**：
- 在根节点对所有描述子用 k-means++ 聚成 $k$ 类（第 1 层 $k$ 个节点）；
- 对每个节点的描述子再聚 $k$ 类（第 2 层 $k^2$ 个节点）；…
- 共 $d$ 层，得 $k^d$ 个叶子（单词）。
- **复杂度**：$d$ 层、每层 $k$ 类的树，单次查找单词只需 **$d$ 次、每次 $k$ 比较**（$O(d\,k)$），把“在 $k^d$ 个单词里线性查”降到对数级——**这是词袋能实时的关键**。

> **注（与本书 BA/点云章节呼应）**：层次 k-means 与本书 `\cref{ch:pointcloud}` 的 k-d 树近邻、`\cref{ch:vo}` 的特征匹配同源；DBoW2 的“词汇树 = 近似最近邻索引”这一观点（§9.2 直接索引）正是把同一棵树**复用于检索与几何验证**。

### 6.3 单词权重：TF-IDF（DBoW2 §IV + 十四讲 §11.3，逐字推导）

**动机**：有些单词对“判同一地点”更有用，有些贡献小；**常见单词区分度低、罕见单词区分度高**。DBoW2 §IV：“decreasing the weight of those words which are very frequent and thus less discriminative”，用 **tf-idf**（源引 [7] = Nister & Stewenius 2006 / Sivic & Zisserman）。

**IDF（逆文档频率，离线、建字典时定）**（十四讲 §11.3）：设训练集所有特征总数为 $n$、叶节点单词 $w_i$ 对应的特征数为 $n_i$，则
$$
\boxed{\ \mathrm{IDF}_i=\log\frac{n}{n_i}\ }.
$$
含义：**某单词在字典中出现频率越低（$n_i$ 越小），IDF 越大，区分度越高**。（十四讲原话：“某单词在字典中出现的频率越低，则区分度越低”指的是“高频 → 低区分”，与 $\log(n/n_i)$ 一致——$n_i$ 大则 IDF 小。）

**TF（词频，在线、对单幅图像定）**（十四讲 §11.3）：设单幅图像中一共出现的单词次数为 $n$、某叶节点单词 $w_i$ 出现 $n_i$ 次，则
$$
\boxed{\ \mathrm{TF}_i=\frac{n_i}{n}\ }.
$$
含义：**某单词在一幅图像里出现越频繁，TF 越大**（对这幅图越重要）。

> **OCR/记号修正（重要）**：十四讲 IDF 与 TF 复用同一对符号 $n,n_i$ 但语义不同（IDF 的 $n,n_i$ 是**训练集全局**统计；TF 的 $n,n_i$ 是**单幅图像局部**统计）。镜像笔记里二者混排极易误读。**综合写章时务必显式区分**，建议记为 $\mathrm{IDF}_i=\log(N_{\text{tot}}/N_i)$、$\mathrm{TF}_i=n_i^{(I)}/n^{(I)}$。本抽取此处忠实保留教材原符号并加注。

**单词权重 = TF·IDF**（十四讲 §11.3）：
$$
\boxed{\ \eta_i=\mathrm{TF}_i\cdot\mathrm{IDF}_i\ }.
$$

**词袋向量**（十四讲 §11.3）：图像 $A$ 的特征点对应许多单词，组成它的词袋：
$$
A=\{(w_1,\eta_1),(w_2,\eta_2),\dots,(w_N,\eta_N)\}\ \triangleq\ v_A.
$$
$v_A$ 是**稀疏向量**（$\in\mathbb R^W$），非零部分代表图像含哪些单词，非零值即对应的 TF-IDF 权重 $\eta_i$。把图像 $I_t$ 转为 $v_t$ 时（DBoW2 §IV），每个二进制描述子**从根到叶遍历树**，每层选 **Hamming 距离最小**的中间节点，到达叶子即得其单词。

> **其它权重方案（十四讲 §11.3 提及）**：除 tf-idf 外还有局部方案（Squared TF、Frequency logarithm、Binary、BM25 TF 等）与全局方案（Probabilistic IDF、Squared IDF 等），可按需替换。

## 7'. 相似度评分（DBoW2 式 (2) + 十四讲，含两式辨析）

### 7'.1 DBoW2 的 $L_1$-评分（源式 (2)，逐字）

两词袋向量 $v_1,v_2$ 的相似度用 **$L_1$-评分** $s(v_1,v_2)\in[0,1]$（**源式 (2)**）：
$$
\boxed{\ s(v_1,v_2)=1-\frac{1}{2}\left\lVert\frac{v_1}{|v_1|}-\frac{v_2}{|v_2|}\right\rVert_1\ }.
\tag{DBoW2-2}
$$
即：先把两向量按 $L_1$ 范数归一化（$v/|v|$），再取归一化向量之差的 $L_1$ 范数、除以 2、用 1 减。**$s=1$ 完全相同，$s=0$ 完全不同**。除以 2 是因为两个 $L_1$ 单位向量之差的 $L_1$ 范数最大为 2。

### 7'.2 十四讲的 $L_1$-评分（差异性求和形式）

十四讲 §11.4 给出等价的 $L_1$ 评分（写成“差异性”求和，**$s$ 越大越相似**）：
$$
s(v_A,v_B)=2\sum_{i=1}^{N}\Big(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|\Big),
$$
其中 $v_{Ai}$ 表示只在 $A$ 中有的单词、$v_{Bi}$ 表示只在 $B$ 中有的单词、$v_{Ai}-v_{Bi}$ 表示在 $A,B$ 中都有的单词的差；$s$ 越大相似性越大。

### 7'.3 两式的代数关系（本抽取补的辨析，供综合统一记号）

对任意标量 $a,b$ 有恒等式 $|a|+|b|-|a-b|=2\min(|a|,|b|)\cdot\mathbb 1[\text{同号}]$（同号时 $=2\min(|a|,|b|)$；异号时 $=0$）。而 $|a-b|=|a|+|b|-(|a|+|b|-|a-b|)$。由此对 $L_1$-归一化向量（$\sum|v_{Ai}|=\sum|v_{Bi}|=1$）：
$$
\tfrac12\lVert v_A/|v_A| - v_B/|v_B|\rVert_1=1-\sum_i\min\big(\hat v_{Ai},\hat v_{Bi}\big),\quad \hat v=v/|v|\ (\text{且 }v\ge 0).
$$
故 DBoW2 式 (2) 等价于 $s=\sum_i\min(\hat v_{Ai},\hat v_{Bi})$（直方图交，$\in[0,1]$）。十四讲的“$2\sum(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|)$”是同一 $L_1$ 思想的**未归一化、未除常数**书写（量纲与 $[0,1]$ 不同，但单调一致）。**综合写章建议统一采用 DBoW2 式 (2) 的 $[0,1]$ 归一化形式**，并在脚注给十四讲的等价直方图交解读，避免读者把两式当成两个不同方法。

> **OCR 修正**：十四讲该式在镜像笔记里常脱落求和上下标或写成 `s(v_A-v_B)`（应为 `s(v_A,v_B)`，是“$A$ 与 $B$ 的相似度”而非“差向量的范数”）。本抽取已订正。

## 8. 倒排索引与直接索引（DBoW2 §IV，逐字）

DBoW2 的图像数据库 = **层次词袋 + 倒排索引 + 直接索引**（源 Fig.1）。

- **倒排索引（inverse index）**：对词汇表每个单词 $w_i$，存一份“**含该单词的图像 $I_t$ 列表**”，并把 $\langle I_t, v_{ti}\rangle$ 配对存储以快速取“该单词在该图的权重”。
  - **作用**：查询时**只与“与查询图有共同单词”的图像比较**（而非全库），大幅提速；新图入库时更新，检索时访问。
  - 十四讲 §11.4 同述：“根据 word 查关键帧时，不用遍历所有关键帧，只要找查询帧描述符映射的那些 words 索引的关键帧即可。”
- **直接索引（direct index，DBoW2 的创新）**：对每幅图像 $I_t$，按词汇树**第 $l$ 层**（叶=0、根=$L_w$），存“该图中各单词在第 $l$ 层的**祖先节点**”及挂在这些节点上的**局部特征列表 $f_{tj}$**。
  - **作用**：把词汇树**复用为 BRIEF 空间的近似最近邻索引**，几何验证时**只在“属于同一单词/同一第 $l$ 层祖先节点”的特征之间**算对应，避免 $O(n^2)$ 穷举（详见 §10）。新图入库时更新，得到候选匹配、需几何验证时访问。

> **本书呼应**：倒排索引=经典文本检索的索引结构；直接索引是 DBoW2 把“同一棵词汇树既当检索器又当 ANN 匹配器”的工程巧思，对应本书工程实践章（`\cref{ch:engineering}`）“一份数据结构多处复用”的思想。

---

# 第四部分 · 回环检测算法四阶段（DBoW2 §V，逐字全展开）

> DBoW2 §V 把回环检测分四阶段（沿用其前作 [5,6]）：**A 数据库查询 → B 匹配分组（岛屿）→ C 时间一致性 → D 几何一致性**。逐阶段全量记录。

## 9. 阶段 A：数据库查询与相似度归一化（DBoW2 §V-A，逐字）

新图 $I_t$ 到来 → 转为词袋向量 $v_t$ → 用数据库（倒排索引）检索 → 得一串候选匹配 $\langle v_t,v_{t_1}\rangle,\langle v_t,v_{t_2}\rangle,\dots$ 及其评分 $s(v_t,v_{t_j})$。

**问题**：评分 $s$ 的取值范围**强依赖查询图本身与其单词分布**（不同查询图的 $s$ 不可直接比、不可设统一阈值）。

**解法：归一化相似度评分（源式 (3)）**。用“本序列中 $v_t$ 可期望得到的最佳评分”去归一：
$$
\boxed{\ \eta(v_t,v_{t_j})=\frac{s(v_t,v_{t_j})}{s(v_t,v_{t-\Delta t})}\ }.
\tag{DBoW2-3}
$$
其中用**上一帧** $v_{t-\Delta t}$ 的评分 $s(v_t,v_{t-\Delta t})$ 近似 $v_t$ 的期望最佳评分（相邻帧外观应当最像，是“满分参照”）。

**十四讲 §11.4 的等价表述（先验相似度 + 3 倍阈值）**：取**先验相似度** $s(v_t,v_{t-\Delta t})$（当前关键帧与上一关键帧的相似性），其它分值都参照它归一化；并给出实用判据——
> **若当前帧与之前某关键帧的相似度，超过当前帧与上一关键帧相似度的 3 倍，就认为可能存在回环**（即 $\eta\ge 3$ 量级作回环候选门限）。

**退化处理（源 §V-A，逐字）**：当 $s(v_t,v_{t-\Delta t})$ 很小（如机器人**转弯**时相邻帧差异大）会错误地把 $\eta$ 抬高 → 故**跳过**那些 $s(v_t,v_{t-\Delta t})$ 达不到最小值、或特征数不足的图像。此最小评分在“可用于检测回环的图像数”与“评分 $\eta$ 的正确性”之间权衡，**取较小值以免丢弃有效图像**。最后，**$\eta(v_t,v_{t_j})$ 达不到阈值 $\alpha$ 的匹配一律拒绝**。

> **本书阈值统一**：$\alpha$ 即上面的“3 倍”量级门限；综合时建议写“归一化评分门限 $\alpha$（典型取使 P-R 曲线达 100% 准确率的值）”，并指明 DBoW2 用训练集 P-R 曲线选 $\alpha$（§5.4、§11）。

## 10. 阶段 B：匹配分组成岛屿（DBoW2 §V-B，逐字）

**动机**：时间上相近的图像（同一段重访内的连续帧）会**互相竞争**得分，把票分散。故把它们**聚成“岛屿”当一个匹配处理**。

**定义**：记 $T_i$ 为时间戳区间 $t_{n_i},\dots,t_{m_i}$，$V_{T_i}$ 为把条目 $v_{t_{n_i}},\dots,v_{t_{m_i}}$ 聚到一起的**岛屿**。若连续时间戳间隙小，则多条匹配 $\langle v_t,v_{t_{n_i}}\rangle,\dots,\langle v_t,v_{t_{m_i}}\rangle$ 合并为单条 $\langle v_t,V_{T_i}\rangle$。

**岛屿评分（源式 (4)）**：
$$
\boxed{\ H(v_t,V_{T_i})=\sum_{j=n_i}^{m_i}\eta(v_t,v_{t_j})\ }.
\tag{DBoW2-4}
$$
即岛内各帧归一化评分之和。**取 $H$ 最高的岛屿**作为匹配组，进入时间一致性阶段。

**为什么岛屿有用（源 §V-B，逐字）**：除避免连续图像互撞外，岛屿还帮助建立正确匹配——若 $I_t$ 与 $I_{t'}$ 是真实回环，则 $I_t$ 很可能也与 $I_{t'\pm\Delta t},I_{t'\pm2\Delta t},\dots$ 相似，**产生长岛屿**；因 $H$ 定义为 $\eta$ 之和，**$H$ 偏好长岛屿**（真回环更易形成长岛、得分更高）。

## 11. 阶段 C：时间一致性（DBoW2 §V-C + 十四讲 + ORB-SLAM，逐字）

得到最佳岛屿 $V_{T'}$ 后，检查它与**前若干次查询**的**时间一致性**（DBoW2 把前作 [5,6] 的时间约束扩展到支持岛屿）：

**判据（源 §V-C，逐字）**：匹配 $\langle v_t,V_{T'}\rangle$ 必须与 **$k$ 个先前匹配** $\langle v_{t-\Delta t},V_{T_1}\rangle,\dots,\langle v_{t-k\Delta t},V_{T_k}\rangle$ **一致**，且要求相邻区间 $T_j$ 与 $T_{j+1}$ **接近重叠**（即历史几次回环都指向同一片历史区域、且随时间平滑移动）。

- 若岛屿通过时间约束 → 只保留岛内**使 $\eta$ 最大**的那条匹配 $\langle v_t,v_{t'}\rangle$（$t'\in T'$），作为**回环候选**，交给几何验证最终裁决。
- **参数 $k$（源 §VI-C 实验确定）**：测 $k\in[0,4]$。$k=0$（关闭时间一致性）到 $k>0$ 在所有工作频率下都**大幅改善**；$k$ 越大、100% 准确率下召回率越高，但**过大只能找到很长的闭环**（召回反降）。**最终取 $k=3$**（三训练集 P-R 平衡好）；并在 Bicocca25b 用 $f=1,2,3$ Hz 验证 $k=3$ **对帧率稳定**。

**十四讲 / ORB-SLAM 的等价表述**：
- 十四讲 §11.5：“某一位姿附近**连续多次**（**ORB-SLAM 中为 3 次**）与历史中某一位姿附近出现回环，才判断为回环”——与 DBoW2 $k=3$ 一致。
- ORB-SLAM 进一步要求**共视一致性**（covisibility consistency）：候选须在连续 3 个**共视关键帧组**中都出现。

> **结构/空间一致性（十四讲 §11.5 列为验证三点之一，与时间一致性并列）**：词袋只看“有无单词、不看排列”，故还须空间结构验证——这与下阶段几何验证衔接。

## 12. 阶段 D：高效几何一致性验证（DBoW2 §V-D，逐字 + 十四讲 + VINS）

**回环候选最终须过几何验证**（DBoW2 §V-D）：在候选回环的任意一对图像间做几何检查。

**DBoW2 的几何验证（源 §V-D，逐字）**：
- **用 RANSAC 在 $I_t$ 与 $I_{t'}$ 间求一个基础矩阵 $\mathbf F$**（fundamental matrix，对极几何，见本书 `\cref{ch:vo}`），要求**至少 12 个对应点**支持。
- 求对应点的三种办法：
  1. **穷举搜索**：把 $I_t$ 每个特征到 $I_{t'}$ 全部特征算描述子距离，按**最近邻距离比（NNDR）**[15] 选对应——$\Theta(n^2)$，最慢。
  2. **k-d 树近似最近邻**[27]。
  3. **DBoW2 法（复用词袋 + 直接索引）**：入库时存“节点-特征对”到直接索引；查 $I_{t'}$ 的直接索引，**只在“第 $l$ 层属于同一节点”的特征间**算对应。
- **参数 $l$ 的权衡（源 §V-D，逐字 + Table II 数据）**：$l$ 固定，权衡“对应点数 vs 耗时”。
  - $l=0$：只比同一**单词**的特征 → 最快、但对应点少 → 因缺对应点而拒掉部分正确回环、**降召回**。
  - $l=L_w$：召回不受影响、但**也不提速**。
  - **Table II（NewCollege，不同取对应法的召回与耗时）**：

    | 技术 | 召回 (%) | 中位耗时 (ms/query) | 最小 | 最大 |
    |---|---|---|---|---|
    | DI0（直接索引 $l=0$） | 38.3 | 0.43 | 0.25 | 16.50 |
    | DI1 | 48.5 | 0.70 | 0.44 | 17.14 |
    | DI2 | 56.1 | 0.78 | 0.50 | 19.26 |
    | DI3 | 57.0 | 0.80 | 0.48 | 19.34 |
    | Flann（k-d 树近似） | 53.6 | 14.09 | 13.79 | 25.07 |
    | Exhaustive（穷举） | 61.2 | 14.17 | 13.65 | 24.68 |

    解读：直接索引（DI2/DI3）以**约 0.8 ms**逼近穷举的召回（57% vs 61.2%），而穷举/Flann 要 **14 ms**——**提速约 18 倍**、召回仅略降。
- **额外收益（源 §V-D，逐字）**：验证只需基础矩阵，但算完 $\mathbf F$ 后**顺带得到图像间数据关联**，可零成本提供给底层 SLAM 算法（即回环边的特征对应，供后端构建约束）。

**十四讲 §11.5 的验证三点（与 DBoW2 互补）**：
1. **不与过近的帧闭环**：关键帧选太近 → 相似度过高、回环无意义；用于回环检测的帧应**稀疏**、彼此不太像又能覆盖全环境。
2. **连续多次一致**（ORB-SLAM 3 次，见 §11）。
3. **回环候选仍要做特征匹配，匹配点足够才算回环**（即几何验证；可用 PnP 后验校正、或条件随机场剔除不符几何一致性的图像）。

**VINS-Mono 的两步几何外点剔除（`vio__vins.md` §VII.B，与 DBoW2 对应、本章应引用而非重抽）**：DBoW2 返回候选后，BRIEF 描述子匹配会产大量外点，VINS 用两步 RANSAC：
- **2D-2D**：基础矩阵 + RANSAC（当前图与候选图检索特征的 2D 观测）；
- **3D-2D**：PnP + RANSAC（滑窗特征的已知 3D 位置 + 候选图 2D 观测）；
- 内点数超阈值才视为正确回环、执行重定位。

> **本章对“几何验证”的统一叙述建议**：纯单目无尺度/无 3D → 用**对极几何（基础/本质矩阵）+ RANSAC**（DBoW2、VINS 第一步）；有 3D 路标（双目/RGB-D/已三角化）→ 用 **PnP + RANSAC**（VINS 第二步、十四讲）。两者都引本书 `\cref{ch:vo}` 的对极几何/PnP 推导，不在本章重推。

---

# 第五部分 · 感知混叠的本质与应对

## 6'. 感知混叠（perceptual aliasing）（FAB-MAP + 十四讲 + VPR 综述，逐字）

**定义（VPR 综述）**：感知混叠 = **不同地点产生相似视觉结构**（走廊、建筑立面、重复纹理），导致**错误匹配（假阳性）**。"different locations with similar visual structures … lead to false matches"。

**词袋为何天然易感知混叠（十四讲 §11.5）**：基于词袋的回环**只在乎单词有无、不在乎单词排列顺序** → 丢失了空间结构信息；且**完全依赖外观、不用任何几何信息** → 外观相似的图像极易被误判为回环。**这正是必须加几何验证步骤的根本原因**。

**FAB-MAP 的概率应对（Cummins & Newman 2008，cnblogs CV-life 综述逐字引其摘要）**：
> “……是一种基于外观识别场所问题的概率方法。……可以确定新观察来自以前看不见的地方，从而增加其地图。实际上这是一个外观空间的 SLAM 系统。**我们的概率方法允许我们明确地考虑环境中的感知混叠——相同但不明显的观察结果来自同一地点的可能性很小**。我们通过学习地方外观的生成模型来实现……算法复杂度在地图中的位置数是线性的，特别适用于移动机器人中的在线环闭合检测。”

要点：FAB-MAP 把词袋扩展为**外观空间的生成模型 + 贝叶斯滤波估回环概率**，显式区分“两观测相似是因为同一地点”还是“因为感知混叠（不同地点恰好长得像）”，从而抑制 FP。

**应对感知混叠的四道防线（综合本章）**：
1. **TF-IDF 降权高频单词**（§6.3）：罕见单词区分度高，削弱“到处都有的常见结构”的影响。
2. **相似度归一化 $\eta$ + 阈值 $\alpha$**（§9）：用先验相似度归一，滤掉“环境本来就相似”造成的虚高分。
3. **时间一致性 $k$**（§11）：要求连续多帧都指向同一历史区域——偶发的感知混叠很难连续 $k$ 次自洽。
4. **几何验证（RANSAC + 对极/PnP）**（§12）：外观相似但几何不一致（无法用同一 $\mathbf F$/位姿解释足够多对应点）的候选被剔除——**这是对感知混叠最强的一道关**。

**与 P-R 权衡的闭环逻辑**：感知混叠 = FP 的主因；SLAM 重准确率（§3'），故宁愿调严上述四道防线、牺牲召回率，也要把感知混叠造成的 FP 压到接近 0。DBoW2 正因这四道防线而在五个数据集上报告 **no false positives**。

---

# 第六部分 · 深度学习地点识别简介（NetVLAD）

## 13. NetVLAD：可端到端训练的广义 VLAD 层（arXiv:1511.07247，§3，逐字全展开）

### 13.1 任务设定（源 §2 概览）

大尺度视觉地点识别 = 给定查询照片 $q$，**快速准确地识别其拍摄地点**。做法：把表示参数化为 $f_\theta(I)$（CNN，参数 $\theta$），用**欧氏距离**
$$
d_\theta(I_i,I_j)=\lVert f_\theta(I_i)-f_\theta(I_j)\rVert
$$
检索——**对查询 $q$，按 $d(q,I_i)$ 排序数据库图像**（可用快速近似最近邻），取最近者为同一地点。**固定距离为欧氏距离**，把问题转化为“学一个在欧氏距离下好用的特征映射 $f_\theta$”。检索流水线两步：(i) 提局部描述子；(ii) **无序池化**（orderless pooling，对平移/遮挡鲁棒）。NetVLAD 把第 (ii) 步设计成**可学习层**。

### 13.2 原始 VLAD（源 §3.1，逐字）

VLAD（Vector of Locally Aggregated Descriptors）：与**词袋只数单词计数**不同，VLAD **存每个单词的残差和**（描述子与其聚类中心之差）。给定 $N$ 个 $D$ 维局部描述子 $\{x_i\}$ 与 $K$ 个聚类中心 $\{c_k\}$，输出 $V$ 为 $K\times D$ 矩阵，其 $(j,k)$ 元（**源式 (1)**）：
$$
\boxed{\ V(j,k)=\sum_{i=1}^{N} a_k(x_i)\,\big(x_i(j)-c_k(j)\big)\ },
\tag{NetVLAD-1}
$$
其中 $x_i(j),c_k(j)$ 为第 $i$ 描述子、第 $k$ 中心的第 $j$ 维；$a_k(x_i)$ 为**硬分配**（$x_i$ 离 $c_k$ 最近则为 1、否则 0）。直观：$V$ 的第 $k$ 列记录“分到 $c_k$ 的描述子的残差 $(x_i-c_k)$ 之和”。$V$ 随后**逐列 $L_2$ 归一化（intra-normalization）→ 拉直为向量 → 整体 $L_2$ 归一化**。

> **与词袋的关系（本章关键串联）**：词袋（§6）= 数“每个单词出现几次”（0 阶统计）；VLAD = 存“每个单词的残差和”（1 阶统计），信息更丰富。NetVLAD = 让 VLAD 的“单词 $c_k$ 与软分配”全部**可学习、端到端训练**。

### 13.3 从硬分配到软分配（源 §3.1，逐字 + 平方展开推导）

VLAD 不可微的根源是**硬分配 $a_k(x_i)$**。NetVLAD 换成**软分配**（**源式 (2)**）：
$$
\boxed{\ \bar a_k(x_i)=\frac{e^{-\alpha\lVert x_i-c_k\rVert^2}}{\sum_{k'} e^{-\alpha\lVert x_i-c_{k'}\rVert^2}}\ },
\tag{NetVLAD-2}
$$
按“与各中心的接近程度（相对）”分配权重，$\bar a_k(x_i)\in[0,1]$，最近中心权重最大；$\alpha$ 为正常数，控制响应随距离衰减的快慢。**$\alpha\to+\infty$ 时退化为原始 VLAD**（最近簇权重→1、其余→0）。

**平方展开（源 §3.1，逐字“易见”推导，本抽取补足中间代数）**：展开式 (2) 指数里的平方
$$
-\alpha\lVert x_i-c_k\rVert^2=-\alpha\big(\lVert x_i\rVert^2-2c_k^\top x_i+\lVert c_k\rVert^2\big).
$$
分子分母都含因子 $e^{-\alpha\lVert x_i\rVert^2}$（与 $k$ 无关），**约去**，得
$$
\bar a_k(x_i)=\frac{e^{\,2\alpha c_k^\top x_i-\alpha\lVert c_k\rVert^2}}{\sum_{k'}e^{\,2\alpha c_{k'}^\top x_i-\alpha\lVert c_{k'}\rVert^2}}
=\frac{e^{\,w_k^\top x_i+b_k}}{\sum_{k'}e^{\,w_{k'}^\top x_i+b_{k'}}},
$$
即**源式 (3)**：
$$
\boxed{\ \bar a_k(x_i)=\frac{e^{\,w_k^\top x_i+b_k}}{\sum_{k'} e^{\,w_{k'}^\top x_i+b_{k'}}}\ },\qquad
w_k=2\alpha c_k,\quad b_k=-\alpha\lVert c_k\rVert^2.
\tag{NetVLAD-3}
$$
这正是一个 **softmax（$1\times1$ 卷积 $w_k$ + 偏置 $b_k$，再 soft-max）**。

### 13.4 NetVLAD 层（源式 (4)，逐字）

把软分配 (3) 代入 VLAD (1)，得 **NetVLAD 层（源式 (4)）**：
$$
\boxed{\ V(j,k)=\sum_{i=1}^{N}\frac{e^{\,w_k^\top x_i+b_k}}{\sum_{k'} e^{\,w_{k'}^\top x_i+b_{k'}}}\,\big(x_i(j)-c_k(j)\big)\ }.
\tag{NetVLAD-4}
$$
**三组独立可学习参数 $\{w_k\},\{b_k\},\{c_k\}$**（原始 VLAD 只有 $\{c_k\}$）——**解耦 $\{w_k,b_k\}$ 与 $\{c_k\}$** 带来更大灵活性（源 Fig.3：在监督下可学到比聚类中心更好的“锚点”，使不该匹配的两图残差点积更小）。输出维度：归一化后 $(K\times D)\times 1$。**实现**：标准 CNN 层即可（卷积 $w_k,b_k$ → softmax → VLAD core 聚合（式 4）→ intra-normalization（逐列 $L_2$）→ 整体 $L_2$），组成有向无环图，可反向传播。

**实现细节（源 §4 + 附录）**：基网 **AlexNet / VGG-16**（裁到最后卷积层 conv5、ReLU 前）；NetVLAD 取 **$K=64$**，得 **16k / 32k 维**图像表示；初始化 $\{c_k\}$ 来自对训练描述子的 k-means。$\{w_k,b_k\}$ 的解耦初始化方式同 [3]。

### 13.5 弱监督三元组排序损失（源 §4，逐字全展开）

**监督来源**：Google Street View **Time Machine**——同一地点不同时间/季节的全景图（带 GPS 标签）。**弱监督**：GPS 只给“地理近/远”，**不给图像间对应关系**；同一全景采样为多张不同朝向/俯仰的透视图，每张标该全景的 GPS。故对查询 $q$，GPS 只能给：**潜在正样本 $\{p_i^q\}$**（地理近，但不一定真看同物——可能朝向不同/拐角/遮挡）与**确定负样本 $\{n_j^q\}$**（地理远）。

**目标**：学 $f_\theta$ 使“查询 $q$ 到地理近图 $I_{i^\ast}$ 的距离 < 到所有远图 $I_i$ 的距离”，即 $d_\theta(q,I_{i^\ast})<d_\theta(q,I_i)$。

**步骤 1：选最佳潜在正样本（源式 (5)）**。因不知哪张潜在正样本真匹配，取距离最小者：
$$
\boxed{\ p_{i^\ast}^q=\operatorname*{arg\,min}_{p_i^q}\, d_\theta(q,p_i^q)\ }.
\tag{NetVLAD-5}
$$

**步骤 2：排序约束（源式 (6)）**。要求 $q$ 到最佳正样本的距离小于到**每个**负样本的距离：
$$
\boxed{\ d_\theta(q,p_{i^\ast}^q) < d_\theta(q,n_j^q),\quad \forall j\ }.
\tag{NetVLAD-6}
$$

**步骤 3：弱监督三元组排序损失（源式 (7)）**。对训练元组 $(q,\{p_i^q\},\{n_j^q\})$：
$$
\boxed{\ L_\theta=\sum_j l\!\left(\min_i d_\theta^2(q,p_i^q)+m-d_\theta^2(q,n_j^q)\right)\ },
\tag{NetVLAD-7}
$$
其中 $l(x)=\max(x,0)$ 是**铰链损失（hinge）**、$m$ 是**间隔（margin）常数**。解读：对每个负样本 $n_j^q$，若“查询到该负样本的距离”比“查询到最佳正样本的距离”**大出至少 $m$**，则该项损失为 0；否则损失正比于违反量。这是把常用三元组损失**适配到弱监督场景**（用式 (5) 的 $\min$ 做最佳正样本选择，类似多示例学习 MIL）。

**实现数值（源附录）**：margin **$m=0.1$**，学习率 0.001 或 0.0001；负样本用**随机化困难负样本挖掘（hard negative mining）**并缓存表示一段时间。

### 13.6 评测协议与对接 SLAM（源 §5 + 本章串联）

**地点识别评测（源 §5）**：标准协议——查询图被判“正确定位”当**前 $N$ 个检索结果中至少一个在真值 $d=25$ m 内**；画 **Recall@Top-$N$** 曲线。

**与本章前文的串联（综合写章用）**：
- NetVLAD 产出**单一全局描述子向量**，用**欧氏距离 / 余弦相似度**检索（替代词袋的 $L_1$ 评分 $s$，但用途相同：召回回环候选）。
- VPR 综述明言：**深度全局描述子（NetVLAD/GeM/PointNetVLAD）也对感知混叠不鲁棒，仍需下游几何验证**（"require downstream geometric verification"）。即 **NetVLAD 替换的是流水线的“阶段 A 检索”，时间一致性（C）与几何验证（D）依旧不可省**。
- NetVLAD 是后续大量工作的基线：PointNetVLAD（点云版，`lidar_slam` 可呼应）、Multires-NetVLAD、可微分 NetVLAD + CNN 区域注意力等（VPR 综述列举）。

> **OCR 修正（NetVLAD）**：pdftotext 把式 (2)(3) 的分母 $\sum_{k'}$ 误拆成两行、把上标 $w_k^\top x_i+b_k$ 的转置丢失；本抽取据 CVPR 开放版与公式语义订正为 $w_k^\top x_i+b_k$、$\sum_{k'}e^{w_{k'}^\top x_i+b_{k'}}$。式 (7) 的 $\min_i d_\theta^2$ 与 $+m-d_\theta^2$ 的括号层次亦据原文校正。

---

# 第七部分 · 与位姿图后端的接口（`\cref{ch:nlopt}` 之 `sec:nlopt-posegraph`）

## 7. 回环边如何喂进位姿图（本抽取整合，串联本书已有内容）

**回环检测的最终产物 = 一条位姿图回环边**：$(i,j)$ 一对回环关键帧 + 几何验证得到的**相对位姿测量** $\hat{\mathbf T}_{ij}\in\mathrm{SE}(3)$ + 信息矩阵 $\boldsymbol\Lambda_{ij}=\boldsymbol\Sigma_{ij}^{-1}$。

**对接本书位姿图（`nonlinear_optimization.tex` §793, `eq:nlopt-pg-cost`/`eq:nlopt-pg-jac`）**：位姿图残差（本书右扰动主线）
$$
\mathbf e_{ij}=\mathrm{Log}\!\big(\hat{\mathbf T}_{ij}^{-1}\,\mathbf T_i^{-1}\mathbf T_j\big)\in\mathbb R^6,\qquad
\min_{\{\mathbf T_i\}}\ \tfrac12\sum_{(i,j)\in\mathcal E}\lVert\mathbf e_{ij}\rVert_{\boldsymbol\Sigma_{ij}}^2,
$$
其中 $\mathcal E$ = **里程计边 ∪ 回环边**。里程计边来自前端相邻关键帧、$\hat{\mathbf T}_{ij}$ 是 VO/LIO 输出；**回环边** $\hat{\mathbf T}_{ij}$ 就是本章 §12 几何验证（对极/PnP）解出的相对位姿。这正是本书 §10.3“球”位姿图里**层间回环边**的物理来源（2500 位姿、9799 边）。

**关键工程点（串联本书）**：
1. **回环边是稀疏图变稠密图的唯一原因**：本书 §836 已指出“直线前进得带状（稀疏）位姿图、来回环绕（loopy）得稠密位姿图”——回环边就是制造“loopy”的那些长程边。
2. **FP 回环边 = 灾难**：本书 §1331 故障表“回环处轨迹被拉飞 = 外点/误匹配主导二次代价 → RANSAC 预剔 + 鲁棒核（Huber/Cauchy），非凸核用 GNC”。这与本章 §3'（重准确率）、§12（RANSAC 几何验证）首尾呼应：**几何验证是 RANSAC 预剔，鲁棒核是后端的第二道保险**。
3. **回环触发全局优化的时机**：前端每关键帧检测回环（`visual_odometry.tex` 四线程图），**检出并验证通过后**才触发一次全局位姿图优化 / 回环后完整 BA（ORB-SLAM：回环后做 7-DOF 位姿图 + 全局 BA；VINS-Mono：4-DOF 位姿图，见 `vio__vins.md` §VIII）。
4. **增量后端**：LIO-SAM/LeGO-LOAM 用 **iSAM2/Bayes 树**增量地把回环因子并入因子图（`lidar_slam__loam_family.md`），避免每次回环全量重解。
5. **尺度自由度**：单目回环后位姿图是 **7-DOF**（含尺度，ORB-SLAM）；VIO 因 IMU 固定尺度与重力，漂移仅 **4-DOF**（3D 平移 + yaw），故 VINS-Mono 只优化 4-DOF 位姿图（`vio__vins.md` §123）。

---

# 附录 A · 全章公式索引（供综合 agent 速查）

| 编号 | 公式 | 出处 |
|---|---|---|
| DBoW2-1 | BRIEF 位 $B^i(p)=\mathbb 1[I(p+a_i)<I(p+b_i)]$ | DBoW2 §III 式 (1) |
| DBoW2-2 | $L_1$ 相似度 $s(v_1,v_2)=1-\tfrac12\lVert v_1/|v_1|-v_2/|v_2|\rVert_1$ | DBoW2 §IV 式 (2) |
| DBoW2-3 | 归一化评分 $\eta(v_t,v_{t_j})=s(v_t,v_{t_j})/s(v_t,v_{t-\Delta t})$ | DBoW2 §V-A 式 (3) |
| DBoW2-4 | 岛屿评分 $H(v_t,V_{T_i})=\sum_{j=n_i}^{m_i}\eta(v_t,v_{t_j})$ | DBoW2 §V-B 式 (4) |
| 十四讲-IDF | $\mathrm{IDF}_i=\log(n/n_i)$ | 十四讲 §11.3 |
| 十四讲-TF | $\mathrm{TF}_i=n_i/n$ | 十四讲 §11.3 |
| 十四讲-W | 权重 $\eta_i=\mathrm{TF}_i\cdot\mathrm{IDF}_i$；词袋 $v_A=\{(w_i,\eta_i)\}$ | 十四讲 §11.3 |
| 十四讲-S | $s(v_A,v_B)=2\sum_i(|v_{Ai}|+|v_{Bi}|-|v_{Ai}-v_{Bi}|)$ | 十四讲 §11.4 |
| 十四讲-P | $\mathrm{Precision}=\mathrm{TP}/(\mathrm{TP}+\mathrm{FP})$ | 十四讲 §11.2 |
| 十四讲-R | $\mathrm{Recall}=\mathrm{TP}/(\mathrm{TP}+\mathrm{FN})$ | 十四讲 §11.2 |
| 十四讲-F1 | $F_1=2PR/(P+R)$ | 十四讲 §11.2 |
| NetVLAD-1 | $V(j,k)=\sum_i a_k(x_i)(x_i(j)-c_k(j))$ | NetVLAD §3.1 式 (1) |
| NetVLAD-2 | $\bar a_k=e^{-\alpha\lVert x_i-c_k\rVert^2}/\sum_{k'}(\cdot)$ | NetVLAD §3.1 式 (2) |
| NetVLAD-3 | $\bar a_k=e^{w_k^\top x_i+b_k}/\sum_{k'}(\cdot)$，$w_k=2\alpha c_k,b_k=-\alpha\lVert c_k\rVert^2$ | NetVLAD §3.1 式 (3) |
| NetVLAD-4 | NetVLAD 层（式 (3) 代入 (1)） | NetVLAD §3.1 式 (4) |
| NetVLAD-5 | $p_{i^\ast}^q=\arg\min_{p_i^q}d_\theta(q,p_i^q)$ | NetVLAD §4 式 (5) |
| NetVLAD-6 | $d_\theta(q,p_{i^\ast}^q)<d_\theta(q,n_j^q),\forall j$ | NetVLAD §4 式 (6) |
| NetVLAD-7 | $L_\theta=\sum_j l(\min_i d^2_\theta(q,p^q_i)+m-d^2_\theta(q,n^q_j))$ | NetVLAD §4 式 (7) |
| 位姿图边 | $\mathbf e_{ij}=\mathrm{Log}(\hat{\mathbf T}_{ij}^{-1}\mathbf T_i^{-1}\mathbf T_j)$ | 本书 `eq:nlopt-pg-cost` |

# 附录 B · 关键参数与数值汇总

| 参数 | 值 | 出处 |
|---|---|---|
| 词汇树分支 $k_w$ | 10 | DBoW2 §VI-A-5 |
| 词汇树层 $L_w$ | 6 | DBoW2 §VI-A-5 |
| 单词总数 $W$ | 1,000,000（$\approx 10^6$） | DBoW2 §VI-A-5 |
| 训练特征 / 图像 | 9M 特征 / 10K 图像（Bovisa） | DBoW2 §VI-A-5 |
| FAST 响应阈值 | 10 | DBoW2 §VI-A-5 |
| 每图保留特征数 | 300（响应最高） | DBoW2 §VI-A-5 |
| 时间一致性 $k$ | 3 | DBoW2 §VI-C；十四讲/ORB-SLAM |
| 几何验证最少对应点 | 12（RANSAC 基础矩阵） | DBoW2 §V-D |
| 归一化评分阈值 | “超过先验相似度 3 倍” | 十四讲 §11.4 |
| BRIEF 计算耗时 | 13 ms/图（vs SURF 100–400 ms） | DBoW2 §VI-B |
| 1M 词典内存 | 32 MB（BRIEF）vs 256 MB（SURF） | DBoW2 §VI-B |
| 直接索引几何验证 | DI2/DI3 ≈ 0.8 ms（vs 穷举 14 ms） | DBoW2 Table II |
| NetVLAD 簇数 $K$ | 64 | NetVLAD §4 |
| NetVLAD 维度 | 16k(AlexNet)/32k(VGG-16) | NetVLAD §4 |
| NetVLAD margin $m$ | 0.1 | NetVLAD 附录 |
| NetVLAD 定位判定半径 | 25 m | NetVLAD §5 |

---

# 附录 C · OCR / 抽取修正说明（综合 agent 务必先读）

1. **十四讲 TF-IDF 符号复用 $n,n_i$**：IDF 的 $n,n_i$ 是**训练集全局**统计，TF 的 $n,n_i$ 是**单幅图像局部**统计——镜像笔记常混排致误读。已加注，建议综合时用 $N_{\text{tot}}/N_i$ 与 $n^{(I)}_i/n^{(I)}$ 区分。（§6.3）
2. **十四讲相似度式写成 `s(v_A-v_B)`**：应为 $s(v_A,v_B)$（$A$ 与 $B$ 的相似度，非“差向量的范数”）。已订正。（§7'.2）
3. **十四讲 $s$ 式与 DBoW2 式 (2) 的关系**：二者是同一 $L_1$ 思想的“未归一化求和”vs“$[0,1]$ 归一化”两种书写，**不是两个方法**。已给代数辨析（直方图交 $s=\sum\min(\hat v_{Ai},\hat v_{Bi})$），建议统一用 DBoW2 式 (2)。（§7'.3）
4. **NetVLAD pdftotext 公式损坏**：式 (2)(3) 分母 $\sum_{k'}$ 被拆行、转置 $(\cdot)^\top$ 丢失、$e^{w_k^\top x_i+b_k}$ 上标错位；式 (7) 括号层次错位。已据 CVPR 开放版与语义全部订正。（§13.3–13.5）
5. **DBoW2 式 (2) 的 layout 渲染**：pdftotext 把 $1-\tfrac12\lVert\tfrac{v_1}{|v_1|}-\tfrac{v_2}{|v_2|}\rVert$ 的分式拆散为多片，已据公式语义重组为标准 $L_1$ 形式（与 DBoW2/DBoW3 源码注释一致）。（§7'.1）
6. **“感知偏差/感知变异”术语**：十四讲用“感知偏差”=假阳性(FP)=perceptual aliasing、“感知变异”=假阴性(FN)。英文文献统称 perceptual aliasing 偏指 FP。已对齐并在 §3'、§6' 双语标注。
7. **DBoW2 期刊卷期页**：源 PDF 页眉为占位“VOL. , NO. , MONTH, YEAR”，正式出版信息为 IEEE T-RO **vol. 28, no. 5, pp. 1188–1197, Oct. 2012**（据 IEEE Xplore / 综述引文 [78] 核实）。已在头部标注。
8. **DBoW2 §V vs §IV 编号**：源 PDF 中“四阶段”位于 §V（Loop detection algorithm），数据库结构在 §IV（Image database），二进制特征在 §III。本抽取 §8–§12 的“DBoW2 §V-x”指向正确；§5–§8 的“§III/§IV”亦核对无误。
9. **未能直连的镜像**：CSDN（521）、freesion（403）、bilibili（验证码）、ResearchGate（403）对 WebFetch 反爬；十四讲公式经 cnblogs CV-life 综述（curl 成功，HTTP 200）+ 两次 WebSearch 摘要交叉确认，关键式（P/R、TF-IDF、$s$、先验归一化、3 倍阈值、ORB-SLAM 3 次一致性）三处以上一致，可信。DBoW2、NetVLAD 两主源为**官方 PDF 下载后逐字读取**，无此风险。

---

**抽取完成说明**：本文件已按“作用→评价(P-R/感知混叠)→词袋全链路(特征/词典/TF-IDF/相似度/索引)→四阶段算法(查询/岛屿/时间一致性/几何验证)→感知混叠应对→NetVLAD→位姿图接口”组织，全量抽取 **DBoW2（式 1–4 + 全部参数 + Table I/II + 评测方法学）** 与 **NetVLAD（式 1–7 + 软分配平方展开 + 三元组损失 + 实现数值）** 两主源的每一条公式、每一步推导、每一个数值，并以**十四讲第 11 讲**补足 P-R/混淆矩阵/TF-IDF 教材化表述、以 **FAB-MAP/VPR 综述**补足感知混叠的概率与综述视角。记号差异（DBoW2 的 $w_i$=单词、$v_t$=词袋向量；NetVLAD 的 $\alpha$=softmax 温度、$w_k$=卷积权重；十四讲 $\eta_i$=权重）已逐处对齐本书约定并标注。与 `vio__vins.md`（VINS 回环/重定位/4-DOF 位姿图）、`lidar_slam__loam_family.md`（ICP/欧氏回环 + iSAM2）的去重与引用关系已在头部与 §12、§7 明确——**本章应引用而非重抽**那两处的几何验证与后端细节。
