# Handbook 吸收侦察报告 D —— 学习 / 动态 / 语义 / 开放世界前沿（ch13/15/16/17 + 结语 + 00_Notation）

> 只读侦察（recon），唯一产物是本报告。未改动任何 `.tex` / `refs.bib`。
> 负责章：Handbook **ch13 学习式 SLAM、ch15 动态与可变形、ch16 度量-语义、ch17 开放世界空间AI、ch19 结语**；外加 **00_Notation 对照**。
> 最高准则：**融合而非拼贴**——侦察目标是"现有叙事线长什么样、Handbook 这部分如何织进去、读者读到一个作者的连贯论述、读不出接缝"，不是"内容堆哪"。
> 方法：主侦察 agent 通读 4 章框架/关键节 + Epilogue + 两份 notation + 3 个现有教材章（intro_slam/loop_closure/dense_mapping）+ preface + book.tex；并行 4 路后台 agent 逐章详读 ch13/15/16/17（其详单已并入本报告）。

---

## 0. 总判断（先给结论，细节见各章节）

**这组五章是全书"经典 → 前沿"演进的最后一跃，且本书自己早已为它铺好路。** 关键证据：

1. **preface 已写明全书弧线**：「沿"经典模型 → 学习方法 → 具身智能"的脉络，架起一座由传统机器人学通向现代具身智能的桥梁」，并点名「强化学习、视觉–语言–动作（VLA）模型、大模型驱动的策略与操作」。**这正是 ch13（学习式）+ ch16/17（语义/开放世界/VLA）要兑现的承诺**——前沿这组不是"另起炉灶"，而是 preface 立的旗终于落地。
2. **intro_slam 已有 `\section{前沿与展望}`（`sec:intro-frontier`）**，明确自述是"展望性概览，本书不会逐一展开其全部细节"，已预告**动态/可变形 SLAM**（DynaSLAM/Co-Fusion/DynamicFusion）与**多机器人协作 SLAM**。这是一根现成的"预告→兑现"主线索。
3. **现有三章已埋好接口**：`loop_closure` 已有 `sec:loop-dl`（NetVLAD+PoseNet，且立场"深度描述子仍需几何验证"）；`dense_mapping` 已引入 metric-semantic 术语 + 四类稠密表示 + Kimera-Multi 一句 + 已 `\cite` Handbook ch5；`vio` 的 `sec:vio-frontier` 已有"学习增强的惯性里程计"段。前沿织入有大量现成锚点。
4. **数学接口天然吻合**：ch13 的可微分 BA（式13.10）、ch16 的物体/语义因子（式16.1/16.3/16.27）**都直接建立在本书 `eq:intro-nls`/`ch:nlopt` 的 MAP/因子图母方程 $J(\mathbf{x})=\sum\|\mathbf{e}_i\|^2_{\boldsymbol\Sigma_i}$ 之上**——ch13 §13.3 原文就说"recall in Chapter 1 that a factor graph gives rise to a cost function"。这意味着前沿能以"同一条母方程的新因子/新残差/可学习权重"的姿态接续，而非平行宇宙。

**slice 级结构推荐（详见 §7）**：**新建一个 part「P6 学习时代的 SLAM 与空间 AI」（暂名），把 ch13+ch16+ch17 聚成 3 个新章，ch15 拆为"织入 + 新章"混合，ch19 化为全书结语。** 理由：这组体量大、主题自洽（统一在"学习/语义/开放世界"旗下）、且正是 preface 承诺的"经典→学习→具身"桥梁；分散硬塞进 P2 各章会撑爆且打断经典叙事。但**入口与收尾必须由 intro_slam 的前沿节与各章前沿段"牵引过去"**，做到读者顺着经典主线自然走入前沿，读不出接缝。

---

## 1. ch13 《Boosting SLAM with Deep Learning》（学习式 SLAM）

**源**：`SLAM_Handbook_md/13_.../13_....md`（488 行）。作者：Teed, Deng, Chidlovskii, Revaud, Wimbauer, Cremers（即 DROID-SLAM/DPVO + DUSt3R/MASt3R + Cremers 组）。

### 覆盖度：**部分有（点状）→ 大部分全新**
现有教材只有"点状"覆盖：`vo` 章导航图有"混合/学习 DSO/SuperGlue"格、带 $*$ 的"学习型特征"选读；`loop_closure` 的 `sec:loop-dl` 有 NetVLAD + PoseNet 位姿回归；`vio` 前沿段有"学习增强惯性里程计"。但**学习式 SLAM 的体系**（可微分 BA / 端到端 / pointmap 范式）教材**完全没有**。

### 现有落点（可挂接处）
- `parts/P2_slam/visual_odometry.tex`：特征点法/直接法/三角化主线；`sec:vo-flow`/`sec:vo-direct` 是"学习式光流 RAFT"的天然上文。
- `parts/P1_estimation/nonlinear_optimization.tex` + `parts/P2_slam/slam_system.tex`（`ch:nlopt`）：因子图/GN/BA 母方程，是可微分 BA 的直接前置。
- `parts/P2_slam/loop_closure.tex` `sec:loop-dl`：位姿回归（PoseNet/DeepVO/TartanVO）与之有交集，但本书是"位姿回归用于重定位"，ch13 是"用于 VO"，互补不冲突。

### Handbook 增量清单（按吸收价值排序）
1. **可微分 BA（核心思想，式13.9→13.10）**：把优化层（BA）当可反传的网络层，用网络产出残差 $\mathbf{e}_\theta(\mathbf{z},\mathbf{x})$ 与协方差 $\boldsymbol\Sigma_\theta$「塑造」代价函数 $J(\mathbf{x})=\|\mathbf{e}_\theta(\mathbf{z},\mathbf{x})\|^2_{\boldsymbol\Sigma_\theta(\mathbf{z},\mathbf{x})}$。**这是全章最该补、教材最缺的增量**。前置：BANet。
2. **DROID-SLAM（式13.7/13.8/13.11）**：光流作中间表示；帧图 G + 每边相关金字塔 + Conv-GRU 更新算子输出 $\delta\mathbf f_{ij}$ 与置信 $w_{ij}$；可微分 BA 代价 $J=\sum_{(i,j)\in\mathcal G}\|\delta\mathbf f_{ij}-[h_{ij}(\mathbf x_{ij})-h_{ij}(\mathbf x_{ij}^0)]\|^2_{\boldsymbol\Sigma_{ij}}$，$\boldsymbol\Sigma_{ij}=\mathrm{diag}\,w_{ij}$；展开 GN 反传端到端训练；运动滤波选关键帧；中距离回环（全帧对光流距离<阈值加长程边，**无重定位**，大回环关不上）；TartanAir 训练（位姿损失+光流损失）。免重训推广到 RGB-D（式13.12 加深度因子）/立体/VI/事件。
3. **DPVO/DPVS**：稀疏 patch 版（patch graph、单逆深度/patch、式13.13 重投影、时间卷积、展开 2 步 GN）；DPVS 加全局优化+回环、单 GPU 实时。
4. **RAFT（式13.4/13.5）**：4D 全对相关体 + 相关金字塔 + 查找算子 + Conv-GRU 迭代 $\mathbf f^{k+1}=\delta\mathbf f^k+\mathbf f^k$。DROID 的基础，应作"学习式光流"小节。
5. **学习式深度/位姿/自监督（式13.1）**：单图深度网（Eigen、ViT backbone）对抗单目尺度漂移；位姿回归（PoseNet/TartanVO/DeepVO，**本章自评不如经典 SfM**）；视图合成自监督（光度 warp 损失训深度+位姿）；DVSO（虚拟立体）、D3VO（深度+位姿+**不确定性**三网络喂直接法 DSO，对非朗伯面降权光度残差）；MonoRec（代价体+动态掩膜稠密重建）、BTS（体积密度场 $\phi(\mathbf x):\mathbb R^3\to\mathbb R^+$ + 体渲染）。
6. **特征/匹配网络**：SuperPoint（学习关键点+描述子）、SuperGlue/LightGlue（自+交叉注意力出分配矩阵）、LoFTR（无检测器稠密匹配）。
7. **Pointmap 范式（式13.14–13.18）**：**DUSt3R**——Siamese ViT 编码器 + 双交叉注意力解码器 + 回归头，从**无标定无位姿**图像对直接回归两张"同坐标系" pointmap $X\in\mathbb R^{W\times H\times3}$ + 置信图；置信度加权损失（式13.17，$C=1+\exp c>0$）；下游从 pointmap 解内参（Weiszfeld）、相对位姿（Procrustes/PnP+RANSAC）；多图全局对齐（式13.18，约束 $\prod_e\sigma_e=1$）。
8. **MASt3R / MASt3R-SLAM 等（式13.19–13.23）**：DUSt3R + 匹配头 + **InfoNCE 匹配损失**（式13.21）；MASt3R-SfM（冻结编码器检索、去 RANSAC）、MUSt3R（对称架构+工作记忆、统一坐标系）、MASt3R-SLAM（实时稠密单目、免相机模型）；VGGT/VGGT-SLAM、MegaSAM/ViPE（13.7 趋势）。

### 融合策略（重中之重）
**单立新章「学习式 SLAM」，挂在新 part 的开头**，用"经典→前沿"的明确过渡牵引，分三段对应本章三条主线（替换组件 → 端到端可微分 BA → pointmap 全栈）：

- **过渡接口（关键）**：本章开篇应**直接接住** `eq:intro-nls`/`ch:nlopt` 的母方程 $J(\mathbf x)=\sum\|\mathbf e_i(\mathbf x_i)\|^2_{\boldsymbol\Sigma_i}$，一句话点明"经典后端手工设计残差 $\mathbf e_i$ 与权重 $\boldsymbol\Sigma_i$；本章把这两者交给网络学（式13.10），优化层本身可反传——这就是可微分 BA"。如此可微分 BA 不是空降，而是"同一条母方程，残差/权重换成可学的"，与经典后端浑然一体。
- **与 `vo` 章对接**：学习式光流（RAFT）、学习式特征（SuperPoint/SuperGlue）应明确"复用并升级 `ch:vo` 的特征/光流"，用前后引用（`\cref{ch:vo}`），而非重述对极几何。
- **与 `loop_closure` `sec:loop-dl` 对接**：位姿回归与 NetVLAD 那条线交叉引用，避免重复——本书已有"位姿回归 PoseNet"，这里只补"位姿回归用于 VO 的累积误差问题 + RNN/序列版 DeepVO"。
- **与 `dense_mapping` 对接**：MonoRec/BTS 的多视深度与体渲染应交叉引用本书已有的单目深度滤波（`sec:dense-depth-filter`）与（若吸收 Handbook ch14）NeRF 章；点明"学习式稠密 = 概率深度滤波的网络化"。
- **主线记号**：ch13 用 $\exp(\boldsymbol\xi^\wedge)\mathbf T$ 局部参数化 + 逆深度——**与本书右扰动、$\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$、逆深度 $\rho_{\mathrm{inv}}$（dense 章已定）完全吻合**，可直接复用，无需改写。位姿损失式13.1 的 $\|(\mathbf T_1^2)^{-1}\mathbf T_1^{2*}\|$ 应改写为本书的 $\mathrm{Log}(\cdot)$ + 右扰动度量。

### 独立性风险
- **§13.6（MASt3R-SfM/MUSt3R/MASt3R-SLAM）与 §13.7（VGGT/MegaSAM/ViPE）几乎纯系统罗列**（系统名+一句卖点+©IEEE 图，零公式）。照搬必成 ventriloquize。**对策**：压成"前沿趋势"一节、显式标注为综述指针；或只精选 MASt3R-SLAM 补其前后端机制。
- **跨章 punt**：§13.3 反复"using the techniques outlined in Chapter 4"来反传 GN——**本书没有 Handbook ch4**，必须自补"如何对 GN/优化层求导（隐函数定理/展开反传）"，否则可微分 BA 核心增量悬空。这是本章吸收**最大的自包含缺口**。
- **BTS 体渲染**："through the established volume rendering technique"是典型 punt；若讲 BTS/NeRF 式表示须自补体渲染积分。
- SuperGlue 最优传输/Sinkhorn、Procrustes 闭式、Weiszfeld 迭代、InfoNCE 均只给名字/一行式——要自包含须各补小推导（RANSAC/PnP/SE(3) 闭式对齐本书他章已有，内部 `\cref` 即可）。
- 大量 ©IEEE 图须重绘（Fig 13.10 DROID 架构、Fig 13.14 DUSt3R 架构最值得本书自绘）。

### 是否新建子节/新章/新 part：**新章**（属新 part；见 §7）。工作量：**大**（可微分 BA 推导 + DROID + pointmap 三大块，且需补 ch4 缺的反传推导）。

---

## 2. ch15 《Dynamic and Deformable SLAM》（动态与可变形）

**源**：`SLAM_Handbook_md/15_.../15_....md`（421 行）。作者：Schmid, Montiel, Huang, Cremers, Neira, Civera（DefSLAM 系 + Khronos 系核心团队）。

### 覆盖度：**部分有（概览级预告）→ 大部分全新（推导级）**
`intro_slam` 的 `sec:intro-frontier`（`sec:intro-dynamic`）已**概览预告** DynaSLAM / Co-Fusion / DynamicFusion，并明确说"展望性、不展开"。`dense_mapping` 有 TSDF/surfel 完整推导（ch15 的稠密重建直接复用）。但 ch15 的**问题刻画框架、动态物体跟踪的因子图公式、长期/change-aware SLAM、可变形 SLAM（NRSfM/医学）**全部超出预告，是推导级全新。

### 现有落点
- `parts/P0_intro/intro_slam.tex` `sec:intro-dynamic`：已预告 3 个系统，是"概览→深化"的牵引点。
- `parts/P2_slam/dense_mapping.tex`：TSDF（`sec:dense-tsdf`）、surfel（`sec:dense-surfel`）、octree（`sec:dense-octomap`）——ch15 的 canonical map/SurfelWarp/体素差分直接复用，且 dense 章 TSDF 的 $W_\eta$ 移动平均已埋"支持动态场景"伏笔。
- `parts/P1_estimation/nonlinear_optimization.tex`（`ch:nlopt`）+ 因子图/SE(3)：动态物体因子、change-aware 因子图、边缘化的直接前置。
- `parts/P2_slam/loop_closure.tex`：长期定位/外观不变特征与回环/地点识别同源。

### Handbook 增量清单（按推导深度排序）
1. **动态物体跟踪的因子图公式（式15.1–15.4，本章唯一成体系数学段）**：相机 $\mathbf T_i^w$ + 静态点 $\mathbf x_l^w$ + 动态物体位姿 $\mathbf T_{k,i}^w$ + 物体系点 $\mathbf x_j^k$ + 线/角速度 $\mathbf v_i^k,\boldsymbol\omega_i^k$；改进重投影 $\mathbf e_{\text{reproj}}=\mathbf z_j^i-\pi((\mathbf T_i^w)^{-1}\mathbf T_{k,i}^w\tilde{\mathbf x}_j^k)$；恒速模型残差（式15.2）；速度-位姿耦合（式15.3）；恒速位姿增量 $\Delta\mathbf T=\begin{bmatrix}\exp(\boldsymbol\omega_i^k\Delta t)&\mathbf v_i^k\Delta t\\\mathbf 0&1\end{bmatrix}$（式15.4）。代表：DynaSLAM II、VDO-SLAM、MVO、AirDOS（铰接/人体骨架=多连刚体）。**最高优先增量**，能与本书因子图/SE(3) 无缝衔接。
2. **可变形 SLAM 整块（§15.4，体量最大、最全新）**：NRSfM vs Deformable SLAM 区分；SfT（模板法，等距/能量两类形变模型）；正统 NRSfM（低维形状基 + 时空/拓扑正则）；**Floating Map Ambiguity**（刚/非刚分离病态）；RGB-D 系（DynamicFusion=canonical map + **Embedded Deformation graph (ED)** + **ARAP** 正则；KillingFusion=killing 正则处理拓扑变化；SurfelWarp；MIS-SLAM；EMDQ 医学立体）；单目流水线（DefSLAM=等距 NRSfM；NR-SLAM=dynamic deformable graph + visco-elastic + deformable BA；LK 光流替代 ORB）；医学内窥镜应用。
3. **长期动态 / change-aware SLAM（§15.3，全新概念框架）**：短期 vs 长期动态的"相对率"定义（变化率/观测率之比，非绝对时长）；**absence of evidence vs evidence of absence**；experiences/sessions + 跨经历约束 + 离线汇总；**边缘化的"baking in"陷阱**（被边缘化节点拓扑不可改、错误回环冻结）；地图清理（体素一致性 vs 可见性）；变化检测（体素差分 scene differencing + 语义）；物体实例重定位（**SE(3)-invariant/equivariant 描述子**）；change-aware SLAM 的核心可识别性论证 **local consistency**；Panoptic Multi-TSDF/POCD/NeuSE/POV-SLAM/Khronos。
4. **时序场景理解（§15.3.4）**：Maps of Dynamics、周期事件的**频率图（Fourier 谱）**、scene-graph 学习预测。
5. **问题刻画三轴框架（§15.1）**：动态效应类型 / 理解细节 / 处理时机（在线 vs 离线）；State estimation vs scene representation；这套分类法是组织全章（也是教材本章导言）的骨架。

### 融合策略
**拆为"织入 + 新章"混合**：

- **短期动态（剔除/跟踪）织入 + 升级 `intro_slam` 预告并新立一章**：`intro_slam` 的 `sec:intro-dynamic` 现有的"展望式 punt"（"本书不展开"）应改写为"指向后文专章"。动态物体跟踪因子图（式15.1–15.4）值得在新 part 单立一章「动态与可变形 SLAM」，**以本书已有因子图/SE(3)/恒速运动模型为底座**，把动态物体当"额外的刚体状态 + 恒速因子"接进既有母方程——这是"经典 SLAM 同时估位姿+静态地图 → 动态下还要判每个观测属静/动"的自然加难（呼应 `sec:intro-dynamic` 已写的"鸡生蛋又拧紧一圈"洞见）。
- **稠密动态/可变形复用 `dense_mapping`**：canonical map/ED/SurfelWarp 应明确"复用 `ch:dense` 的 TSDF/surfel，把静态体素推广为随时间形变的体素"（dense 章 DynamicFusion 一句已埋此线），交叉引用而非重证融合公式。
- **长期/change-aware 与 `loop_closure` 对接**：长期定位的外观不变特征、物体重定位与回环/地点识别同源，交叉引用 `ch:loop`。
- **主线记号**：ch15 用 $\exp(\boldsymbol\omega(t_{i+1}-t_i))$ 恒速积分——须对齐本书 $\mathrm{Exp}$/右扰动约定；SE(3)-equivariant 描述子可与 Barfoot 已吸收的"对称不变/等变"对照。

### 独立性风险
- **本章本质是综述（作者=各系统原作者），系统罗列密度极高**。§15.2.1 六类剔除方法、§15.3 各系统（Panoptic/POCD/NeuSE/POV-SLAM/Khronos）逐段介绍——照搬即 ventriloquize 流水账。**对策**：抽统一原理（动态外点判别 / local consistency 可识别性 / EM 交替）+ 一两个自洽推导实例，系统名仅作引文代表。
- **形变能量泛函全部 punt 到原论文**（最高风险）：ED 图、ARAP、Killing、等距、visco-elastic **只给名字+一句直觉，零数学式**。本书要自包含**必须自补** ARAP 能量 $\sum_i\sum_{j\in\mathcal N(i)}w_{ij}\|(\mathbf p_i'-\mathbf p_j')-\mathbf R_i(\mathbf p_i-\mathbf p_j)\|^2$ 类标准式、等距约束、ED 图蒙皮权重插值——Handbook 在此完全不可依赖。
- §15.1 短/长期定义引单一来源 [693]——应升级为本书自己的形式化定义。
- 16 张图全 ©IEEE/Springer，须自绘（Fig 15.6 动态物体因子图、Fig 15.14 NRSfM vs Deformable 三帧示意最值得重画）。

### 是否新建子节/新章/新 part：**新章**（属新 part；短期动态部分同时升级 intro 预告）。工作量：**大**（因子图公式可全吸收，但可变形能量泛函 + 长期 SLAM 原理两块须自补推导）。

---

## 3. ch16 《Metric-Semantic SLAM》（度量-语义）

**源**：`SLAM_Handbook_md/16_.../16_....md`（525 行）。作者：Asgharivaskasi, Doherty, Behley, Hughes, Chang, Leonard, Christensen, Carlone, Atanasov（该领域顶级团队，Kimera/Hydra 体系）。**三本书里唯一系统讲语义 SLAM 的章。**

### 覆盖度：**部分有（仅术语锚点）→ 几乎全新（方法/推导）**
`dense_mapping` 已**引入 metric-semantic 术语**（`tab:dense-maptypes`：度量-语义=稠密语义网格、物体级地图、用于任务级规划/人机交互）并 `\cite` Handbook ch5；`intro_slam` 提 Kimera-Multi 一句。但语义 SLAM 的**任何具体方法/推导**（物体级、稠密语义融合、场景图、语义因子）教材**全无**。

### 现有落点
- `parts/P2_slam/dense_mapping.tex`：`tab:dense-maptypes`（度量-语义术语锚点）、TSDF/surfel/octree/点云/mesh 几何推导——ch16 §16.3 是"这些表示之上加语义层"，几何部分 `\cref` 回 dense 章即可（本书有自己的"第5章"，独立性优势）。
- `parts/P1_estimation/nonlinear_optimization.tex` + 因子图/SE(3)：ch16 §16.2 物体级/语义因子（式16.1/16.3/16.27）直接建在本书已有因子图上。
- `parts/P2_slam/loop_closure.tex`：语义辅助回环、语义 ScanContext、场景图回环——与本书 NetVLAD/回环交叉引用。
- `parts/P0_intro/intro_slam.tex`：Kimera-Multi 一句可升级为有出处的定位。

### Handbook 增量清单（四块可独立推导的硬核 + 一块理论框架）
1. **稀疏物体级 SLAM（§16.2，全新）**：物体地标 Def 16.1 $\ell=(\sigma,\mathbf q,\lambda,\mathbf s)$（类别/位姿/尺度/形状）；标准残差式16.1 $r_i=\|\mathbf z_i-\mathbf h_i(\mathbf T_i,\ell_i)\|_{\boldsymbol\Sigma_i}$（**就是本书 MAP 母方程**）。四种形状表示：
   - **语义关键点**（式16.2 主动形状模型 $\mathbf s_j=\mathbf b_{j,0}+\sum_k c_k\mathbf b_{j,k}$；式16.3 透视投影残差）；
   - **二次曲面/椭球**（式16.4–16.11，对偶二次曲面解析投影成对偶圆锥，与 bbox 拟合椭圆比较；向量化 $\mathbf A=\mathbf D(\mathbf P\otimes\mathbf P)\mathbf E$）——**数学最完整，最适合作"语义因子推导"样板**；
   - **mesh**（可微渲染，式16.12 IoU 倒数残差）；
   - **隐式 SDF**（式16.13–16.15，神经解码器 $d_\theta(\mathbf x,\mathbf s)$ + shape code；DeepSDF/ELLIPSDF）。
2. **混合离散-连续因子图与求解器（§16.2.2，理论增量）**：MAP 式16.16（位姿+地标含离散语义+数据关联联合）；混合因子图分解（式16.17–16.19，因子分纯离散/纯连续/离散-连续三类）；条件独立分解（式16.20–16.21）；求解策略——交替最小化/DC-SAM（式16.22）、EM（Bowman，E 步软关联可由**矩阵 permanent** 精确恢复）、multi-hypothesis、连续松弛（单纯形）。
3. **多类贝叶斯体素建图（§16.3.2，语义融合核心，教材最该补）**：range-category 观测 $\mathbf z=(r,y)$；因子化 PMF；**多类 log-odds 向量** $\mathbf h_{t,i}\in\mathbb R^{K+1}$（free 为 pivot）+ softmax 恢复；**加法贝叶斯更新** $\mathbf h_{t+1,i}=\mathbf h_{t,i}+\sum_z\mathbf l_i(\mathbf z)$（式16.24）；分段常数观测模型（式16.26）；多类 OctoMap（节点存类别分布 log-odds + ray-cast + pruning）；分布式 ROAM。**这是本书二值 log-odds 占据（`sec:dense-octomap` 式 `eq:dense-octo-logodds`）的直接多类推广**。
4. **mesh 语义 + PGMO（§16.3.3）**：embedded deformation graph 把回环形变化简为 PGO（式16.27→16.31，三项：PGO + mesh-mesh 刚性 + pose-mesh 刚性；式16.32–16.33 顶点插值更新）。
5. **分层 / 3D 场景图（§16.4，理论性强）**：symbol grounding；内存复杂度三级（式16.34→16.36，分层无损/有损）；分层图 Def 16.2（single parent + locality + disjoint children）+ **treewidth 界**（式16.37，小 treewidth → 多项式时间推断）；3D scene graph 谱系（Armeni/Kim/Wald/Rosinol/**Hughes-Hydra 首个在线全分层+全局一致**）；场景图与因子图联合优化。

### 融合策略
**单立新章「度量-语义 SLAM」，挂在新 part 中段**，承接 `dense_mapping` 的 metric-semantic 锚点：

- **过渡接口（关键）**：本章开篇**直接接住** `dense_mapping` 的 `tab:dense-maptypes`——"前面（`ch:dense`）已把地图分为度量/拓扑/度量-语义/混合，并说度量-语义=几何+'这是什么'；本章兑现：如何把语义织进几何"。如此语义不是空降而是 dense 章分类表的展开。
- **物体级 = 因子图母方程的新残差**：式16.1 与本书 `eq:intro-nls` 同形，应一句话点明"物体级 SLAM = 把点路标 $\mathbf y_j$ 推广为物体 $\ell=(\sigma,\mathbf q,\lambda,\mathbf s)$，残差仍喂同一后端"，与 `ch:nlopt` 无缝。二次曲面物体因子作样板推导。
- **稠密语义 = 复用 + 加语义层**：§16.3 明确"复用 `ch:dense` 的 TSDF/surfel/octree，只补语义"；多类 log-odds 作为本书二值 log-odds（`eq:dense-octo-logodds`）的推广来讲——"二值占据是 $K=1$ 的特例"，浑然一体。
- **与 `loop_closure` 对接**：语义辅助回环、语义 ScanContext 交叉引用 `ch:loop`（本书已有 ScanContext 提及）。
- **闭集 vs 开放集分界**：ch16 自己明确把 open-set/语言嵌入 **punt 给 ch17**——本书应同样处理：本章只讲闭集语义，开放词汇留给"开放世界"章，形成 ch16→ch17 的自然递进（闭集→开集）。
- **主线记号**：ch16 用 $\mathbf q\in SE(3)$ 表物体位姿、$\mathbf T\in SE(3)$ 表相机、$\mathrm{SIM}(3)$ 表带尺度、$\boldsymbol\Sigma$ 协方差——**须把残差对 SE(3)/SIM(3) 的求导改写为本书右扰动**，否则与他章左/右扰动混用。

### 独立性风险
- **§16.2.2 求解器段、§16.3.1 点/surfel 系统、§16.4.2 场景图谱系、§16.5 全节**是高风险综述罗列（"X 做 A、Y 做 B"）。**对策**：抽统一框架（离散-连续 MAP / per-element 语义置信融合 / 分层图定义演化）+ 一两个自洽推导实例，系统名仅作引文。
- **§16.5 全节**风险最高（VLM/CLIP/ATLAS/M3/Kimera-Multi/多机场景图 trends）。且 ch16 自己已把 open-set punt 给 ch17——本书的 §16.5 等价物应**直接并入"开放世界"章、只留指针**，不在本章展开成清单。
- **中风险（有公式但出自单一作者实现）**：§16.3.2 观测模型（式16.26）+ 工程技巧（log-odds 上下限、K̄+others）出自 [37]（作者自己）；PGMO（式16.27）出自 [943]——应从"为什么这样设计"自洽推导，把具体参数化当实例，与本书二值 log-odds / PGO 对照。
- **低风险（数学自洽，最该硬核吸收）**：二次曲面物体因子、隐式 SDF 物体、多类贝叶斯 log-odds 融合、treewidth 界——四块照公式即可独立成立，唯须**记号对齐右扰动**。
- 18 张图全 ©，须自绘。
- **源文件 OCR typo（勿照抄）**：L323 多类 log-odds 首项印成 $\log(p_t(m_i{=}0)/p_t(m_i{=}0))$（应为 $m_i{=}1$ 起）；正文交叉引用 (16.16)↔(16.18) 互错；"ROAM [996]" 应为 [38]。

### 是否新建子节/新章/新 part：**新章**（属新 part）。工作量：**大**（四块硬核推导 + 离散-连续 MAP 框架；记号对齐工作量不小）。

---

## 4. ch17 《Towards Open-World Spatial AI》（开放世界空间 AI）

**源**：`SLAM_Handbook_md/17_.../17_....md`（397 行）。作者：Paull, Morin, Maggio, Büchner, Cadena, Valada, Carlone。**全书压轴展望章，但 §17.3 是有公式可硬核吸收的技术节，非纯随笔。**

### 覆盖度：**几乎全新**（教材几乎空白）。preface 已点"VLA/大模型策略"是承诺、`dense_mapping` 有 metric-semantic 术语、`loop_closure` 有深度地点识别——但开放词汇/基础模型/空间 AI 体系全无。

### 现有落点
- `parts/front/preface.tex`：唯一已含"具身智能/VLA/大模型策略"的承诺，是本章的"旗"。
- `parts/P2_slam/dense_mapping.tex`（metric-semantic）、`parts/P2_slam/loop_closure.tex`（深度地点识别）：开放词汇是其延伸。
- `parts/P0_intro/intro_slam.tex` `sec:intro-need`：本书"是否真需要 SLAM？"（Cadena 2016）与 ch17 §17.4.2"重新追问是否需要地图"**直接呼应**——是绝佳的"首尾相扣"结构机会。

### 增量清单（分三段，价值不均）
1. **§17.3 开放世界建图（有公式，可硬核吸收）**：地图元素 $m_k=(\mathbf p_k,\mathbf f_k)$（几何+语义特征）；range-feature 观测（类比 ch16 range-category）；局部/全局特征融合（ConceptFusion，式17.2/17.3 置信加权 $\alpha=e^{-\gamma^2/2\sigma^2}$）；零样本分割/查询（式17.1/17.4 $y_k=\arg\max_C\langle\mathbf f_k,\mathbf f_C\rangle$，类别集建图时无需已知）；多模态查询（文本/点击/图像/音频）；物体地图（每物体一向量降内存、几何+语义相似度关联融合）；物体语言描述（VLM 生成+LLM 检索，处理否定/affordance 查询）；物体/层次场景图（式17.5/17.6，HOV-SG/ConceptGraphs）；隐式（LERF NeRF 式17.7、LangSplat 3DGS）；**任务驱动表示（Clio + 信息瓶颈，式17.8 $\min I(X;\tilde X)-\beta I(\tilde X;Y)$ + Agglomerative IB 式17.9）**。
2. **§17.1–17.2 基础模型背景（可压缩为最小必要背景）**：基础模型定义；DL 简史（AlexNet→ResNet→BERT/Transformer→对比学习 SimCLR→ViT→DINO/DINOv2→CLIP）；特征型（CLIP 共享嵌入空间 + 余弦查询，式17.1 是后文支点）；生成型（LLM next-token、VLM/LLaVA）；类无关分割（**SAM**，§17.3 公共底座）。**两种开放世界范式区分**（开放词汇前端≠开放世界地图：条件化在建图时→仍闭世界 vs 嵌入地图用图时查询→真开放）是本章最有价值的概念区分。
3. **§17.4 展望（只作全书结语素材，风险最高）**：Map Prompting / Map API；**重新追问是否需要地图**（长上下文 VLM：OpenEQA 50 帧直喂 GPT-4V 胜场景图 prompt；Mobility VLA map-free 变体无效）；机器人基础模型（数据集 RT1/Open-X-Embodiment；**VLA** 模型 GATO/PaLM-E/RT2/OpenVLA/π0、ACT、扩散策略、Octo、RL）；结语（显式结构与通用策略**互补**）。

### 融合策略
**拆成两半 + 一个收尾**：

- **§17.3 → 新章「开放世界空间 AI」的技术主体**，挂在新 part **ch16 之后**（闭集→开集自然递进）。开篇**直接接住 ch16 的闭集 punt**："ch16 只处理闭集（词典先验固定）；本章问：如何让地图回答任何概念？"——用本书口吻把"地图元素 $(\mathbf p_k,\mathbf f_k)$ + range-feature 观测 + 零样本查询"讲成**本书自己的统一框架**（ch16 已铺 range-category，这里平行推广为 range-feature，浑然一体）。系统名（ConceptFusion/ConceptGraphs/HOV-SG/Clio/LERF/LangSplat）当引文代表，**只提炼共性 + 关键差异（平均 vs medoid vs 聚类；稠密 vs 物体 vs 隐式 vs 任务驱动）**，不逐系统复述 pipeline。Clio 信息瓶颈（式17.8）数学含金量最高，作"任务驱动地图"小专题严肃讲授（信息瓶颈是经典理论，可独立推导）。
- **§17.1–17.2 → 最小必要背景小节**（或附录）：浓缩成"读懂开放词汇 SLAM 所需三块"——(i) 特征/嵌入+余弦相似度，(ii) 多模态对齐（CLIP），(iii) 类无关分割（SAM）。**CLIP 共享嵌入空间必须讲透**（后文一切支点）；ViT/对比学习/LLM 点到为止 + 引外部 DL 教材。**不照搬全史**（否则单书叙事+与无数 DL 教材重复）。
- **§17.4 → 全书结语章（ch19）的素材库**，与 intro `sec:intro-need` 首尾呼应：把"是否还需要地图"作为本书自己的开放问题（用 OpenEQA/Mobility VLA 的**实验事实**支撑，立场本书自己下）；VLA/机器人基础模型那段（事实性强）可作结语一个独立小节。**剥离作者主观立场表态**（"we argue…/it remains to be seen…"），只留事实+本书重述的论点。

### 独立性风险（全章最高，但分区不均）
- **§17.4 是 ventriloquize 重灾区**：满是 "we argue that…"、"it seems conceivable…" 主观 projection。**绝不能**写成"Handbook 展望：未来会…"。**对策**：剥立场留事实——实验事实（OpenEQA/Mobility VLA 数据）可引并明确归属，立场（显式结构与学习互补）转化为本书自己的立场（本书持"SLAM 仍有价值"会很自然）。
- **§17.3 逐系统流水账风险（中等）**：提共性弃 pipeline。
- **§17.1–17.2 与通用 DL 教材重复风险**：压成最小背景。
- **"空间 AI"无正式定义**：ch17 把 Spatial AI 当既有概念用（"超越几何、编码语义、高层推理"），未单独立定义。若本书想立此旗统领前沿，**本章不能提供权威定义**，只能提供内涵——本书须自己给一个工作定义。
- **本章不碰李群/后端/四元数**，与本书右扰动主线**零冲突也零交集**（纯表示/学习章）；与本书"多机器人/动态前沿节"**几乎不重叠**（另一条前沿线，且 ch17 只在静态场景假设下工作）。
- 8 张图多为复制自 SimCLR/ViT/CLIP/LERF/Clio/RT2 原文，须自绘或重述。
- **ch17 未引 Cadena 2016**——本书"三个时代"叙事来源不在本章（来自 intro 已引的 Cadena 2016），两者可并立。

### 是否新建子节/新章/新 part：**新章（§17.3 技术主体）+ 结语素材（§17.4）**（属新 part）。工作量：**中-大**（§17.3 可吸收但需提炼去罗列；§17.1-2 压缩；§17.4 须最克制重写）。

---

## 5. ch19 《Epilogue》（结语）

**源**：`SLAM_Handbook_md/19_.../19_....md`（34 行）。

### 覆盖度：N/A（不是技术内容）
内容是 Handbook 作者们的**格言集**："Trust the math."、"SLAM is not solved, work 2-3 steps ahead."、"Always start with a toy problem."、"SLAM is about demos!"、"Multimodality"、"Clear notation is the best tool to spot modeling errors."、"Extract truth from conflicting sensors."、"Robot perception is a mirror to look into our mind."等。

### 融合策略
**不照搬（照搬必是 ventriloquize 集锦）。** 作为**本书结语章（ch19，属新 part 收尾）**的**精神素材**——把其中与本书主线契合的几条（"trust the math"对应本书逐推导自包含；"clear notation"对应本书统一记号约定的良苦用心；"SLAM is not solved"对应前沿这组的存在理由；"multimodality / extract truth from conflicting sensors"对应本书多传感器融合主线）**用本书自己的口吻、结合本书已建立的内容**重写为收尾寄语，**不得**写成"Handbook 作者说…"。可与 ch17 §17.4 的"是否需要地图"开放问题、intro `sec:intro-need` 首尾合流，构成全书"经典→前沿→展望"的闭环收束。

### 工作量：**小**（融入结语章，作精神素材）。

---

## 6. 00_Notation 对照（Handbook 记号 vs 本书 `parts/front/notation.tex`）

逐项对照 Handbook `00_Notation/00_Notation.md` 与本书 `notation.tex`，列出差异/冲突供吸收时统一。**结论：本组五章几乎不涉及底层李群记号（ch13 唯一相关且与本书吻合；ch15/16 涉及 SE(3) 求导须统一为右扰动；ch17 零交集），整体记号冲突风险低，但有若干须统一项。**

| 维度 | Handbook 00 约定 | 本书 `notation.tex` 约定 | 差异/冲突 → 吸收时处理 |
|---|---|---|---|
| **帧/位姿下标** | 上下标式 $\mathbf R_a^b$：把 $\mathcal F^a$ 坐标重表为 $\mathcal F^b$，$\mathbf v^b=\mathbf R_a^b\mathbf v^a$ | 下标式 $\mathbf R_{ab}$：$b$→$a$，$\mathbf p_a=\mathbf R_{ab}\mathbf p_b+\mathbf t_{ab}$ | **方向相反**（$\mathbf R_a^b=\mathbf R_{ba}$）。本书 notation 已明确登记此差异。**吸收 ch13/15/16 时务必把所有 $\mathbf R_a^b/\mathbf T_a^b/\mathbf T_i^w$ 改写为本书下标式**（如 ch13 $\mathbf T_1^2$、ch16 $\mathbf q,\mathbf T_i$、ch15 $\mathbf T_{k,i}^w$），否则与全书冲突。 |
| **扰动约定** | Handbook 00 只给 $\wedge/\vee$、$\exp/\log$、$\mathfrak{so}(3)/\mathfrak{se}(3)$，**未规定左/右扰动**（各章自定） | **右扰动为主线** $\mathbf R\cdot\mathrm{Exp}(\delta\boldsymbol\phi)$；左扰动文献结论并列对照 | ch13（$\exp(\boldsymbol\xi^\wedge)\mathbf T$ 局部参数化）**与本书右扰动吻合**；ch16 物体/相机位姿求导、ch15 动态物体位姿求导**须统一改写为右扰动**。这是本组最需统一的一项。 |
| **$\mathfrak{se}(3)$ 坐标序** | 00 未明确给 $\boldsymbol\xi$ 分量顺序（只给 $\mathbf T_a^b=\begin{bmatrix}\mathbf R&\mathbf t\\0&1\end{bmatrix}$） | "平移在前" $\boldsymbol\xi=[\boldsymbol\rho;\boldsymbol\phi]$ | 本组各章很少显式写 $\boldsymbol\xi$ 分量；ch13 用逆深度+位姿局部参数化，不涉序。吸收时按本书"平移在前"统一即可。 |
| **后验/先验记号** | $\hat{(\cdot)}$ 后验、$\check{(\cdot)}$ 先验 | $\check{\mathbf x}$ 先验、$\hat{\mathbf x}$ 后验 | **一致**（本书 notation 与 Barfoot/Handbook 同源）。 |
| **协方差/信息** | $\mathcal N(\boldsymbol\mu,\boldsymbol\Sigma)$、$\boldsymbol\Sigma$ 协方差 | $\mathcal N(\boldsymbol\mu,\boldsymbol\Sigma)$、$\boldsymbol\Lambda/\boldsymbol\Omega$ 信息矩阵 | **一致**。ch13 学习式 $\boldsymbol\Sigma_\theta$、ch16 $\boldsymbol\Sigma_i$、ch15 各残差权重均可纳入本书 $\boldsymbol\Sigma/\boldsymbol\Omega=\boldsymbol\Sigma^{-1}$ 体系。 |
| **马氏范数** | 00 给 $L_1/L_2$ 范数；各章用 $\|\cdot\|_{\boldsymbol\Sigma}$（如 ch16 式16.1、ch13 式13.9） | $\|\mathbf e\|^2_{\boldsymbol\Sigma}=\mathbf e^\top\boldsymbol\Sigma^{-1}\mathbf e$ | **一致**（本书加权范数定义与 ch13/16 残差完全同形——这正是因子图能无缝接续的记号基础）。 |
| **高斯过程** | $\mathcal{GP}(\boldsymbol\mu(t),\boldsymbol{\mathcal K}(t,t'))$ | 本书 vio 章已用 GP（连续时间）；notation 未单列 | 一致，本组少用。 |
| **$\wedge/\vee$ 算子** | $\mathbb R^3$(resp.$\mathbb R^6$)↔李代数，$\mathbf u^\wedge\mathbf v=\mathbf u\times\mathbf v$ | $(\cdot)^\wedge,(\cdot)^\vee$（$\mathfrak{so}(3)$ 反对称、$\mathfrak{se}(3)$ 重载）+ 本书另有 $(\cdot)^\odot$ 齐次点扰动 | **一致**；本书更细（$\odot$）。ch15 式15.3 的 $h^{-1}$ 齐次去除算子需与本书 $\tilde{(\cdot)}$ 齐次约定对齐。 |
| **新增记号（本组特有，本书 notation 暂无，吸收时须新登记）** | — | — | **学习/语义专属**：网络 $f_\theta$、嵌入/特征 $\mathbf f$（CLIP/DINO）、pointmap $X\in\mathbb R^{W\times H\times3}$、shape code $\mathbf s$、物体地标 $\ell=(\sigma,\mathbf q,\lambda,\mathbf s)$、二次曲面 $\mathbf Q_s$/对偶 $\mathbf Q_s^*$、多类 log-odds $\mathbf h\in\mathbb R^{K+1}$、语义类别 $\sigma\in\mathbb N$、数据关联 $D$、形变图节点/权重、互信息 $I(\cdot;\cdot)$。这些须在新 part 各章首"本章记号"框登记，并**避让本书已占用符号**：⚠️ ch17 用 $\mathbf f$ 表"特征/编码器"——本书 dense 章已用 $\mathbf f$ 表"单位视线方向"，**须区分**（如语义特征用 $\boldsymbol\psi$ 或 $\mathbf f^{\mathrm{sem}}$）；ch16 用 $\mathbf s$ 表 shape，与本书 $s$（尺度因子/视差）潜在冲突，须区分。 |

**一句话**：底层李群/概率记号本书已与 Handbook 同源（马氏范数、$\hat{}/\check{}$、$\wedge/\vee$、$\mathcal N$ 全一致），**唯两件须统一**：(1) **帧下标方向**（$\mathbf R_a^b$→本书 $\mathbf R_{ab}$）；(2) **SE(3) 求导统一右扰动**。**新增的是"学习/语义层"记号**（$f_\theta$/特征/pointmap/物体地标/多类 log-odds/互信息），须新登记并避让 $\mathbf f$、$s$ 等已占符号。

---

## 7. slice 级结构建议（核心决策：是否新建 part）

### 明确推荐：**新建一个 part「P? 学习时代的 SLAM 与空间 AI」（暂名，下称"前沿 part"）**，承接现 P2（感知与 SLAM）之后。

**这组（学习/动态/语义/开放世界 + 结语）是全书最可能、也最应该触发"新建 part"的结构决策。** 推荐结构：

```
P2  感知与 SLAM（现有，经典主线，10 章，不动其骨架）
       └─ slam_system「设计一个视觉SLAM系统」= 经典 SLAM 的 capstone（现有收尾章）
P?  学习时代的 SLAM 与空间 AI（新 part）★
    ├─ ch  学习式 SLAM（ch13：可微分 BA / DROID / pointmap）
    ├─ ch  动态与可变形 SLAM（ch15：动态物体因子图 / 可变形 / change-aware）
    ├─ ch  度量-语义 SLAM（ch16：物体级 / 稠密语义 / 3D 场景图）
    ├─ ch  开放世界空间 AI（ch17 §17.3 主体 + §17.1-2 背景）
    └─ ch  结语与展望（ch17 §17.4 + ch19 + 呼应 intro sec:intro-need）
```

### 理由（为什么新建 part 而非分散织入）

1. **体量**：ch13/15/16 各含大段独立推导（可微分 BA、动态因子图+可变形能量、物体因子+多类贝叶斯+场景图），单章皆达本书理论章规模（数百行 LaTeX 量级）。硬塞进 P2 现有章会各自撑成"章中章"，打断经典叙事节奏。
2. **主题自洽**：四章统一在一条清晰主线下——**"当 SLAM 遇上学习与语义，世界从静态几何走向动态、语义、开放"**。它们彼此衔接（ch13 学习式特征/深度 → ch16 闭集语义 → ch17 开集/开放世界；ch15 动态/可变形是"非静态世界"的另一维），聚成一 part 才能讲出这条演进线，分散则线断。
3. **preface 已承诺这条弧线**："经典模型 → 学习方法 → 具身智能"，点名 VLA/大模型策略。**新 part 正是 preface 旗帜的落地**；不建则 preface 承诺悬空。
4. **"经典→前沿"演进读感**：P2 以 `slam_system`（搭一个完整经典视觉 SLAM 系统）收尾，是经典的高峰；紧接新 part「学习时代…」，读者从"我会搭经典 SLAM"自然走向"前沿如何超越它"——这是最顺的叙事坡度。反之若把学习式 BA 硬塞进 `ch:nlopt`、语义塞进 `ch:dense`，会让经典章变成"经典+前沿"的颜料混合（正是用户最忌的"斑斓"）。
5. **独立性可控**：前沿这组综述味重、ventriloquize 风险高（尤其 ch17 §17.4、各章系统罗列）。**聚成独立 part 便于统一施加"剥立场留事实、提共性弃 pipeline、自补 punt 推导"的改写策略**，并在 part 引言统一声明"本 part 是前沿综述性质、以本书口吻重组"，比分散在经典章里逐处补救更可控、读感更一致。

### 与"分散织入"的取舍——**混合方案（推荐的精髓）**

不是"全聚 part、零织入"，而是**新 part 承载主体 + 现有章做牵引锚点**，确保读不出接缝：

- **intro_slam 是总牵引**：`sec:intro-frontier` 现有的动态/多机器人预告，应升级为"指向新 part 各专章"的路标（把"本书不展开"改为"详见第 P? 部"）；并在 `tab:intro-roadmap` 全书路线图**新增几行**指向新 part 各章——让开篇地图就把前沿这组"点到名"，读者一开始就知道前沿归属何处。
- **现有章埋的接口做"过门"**：`dense_mapping` 的 metric-semantic 术语锚点（`tab:dense-maptypes`）→ 语义章过门；`dense_mapping` TSDF 的 $W_\eta$"支持动态"伏笔 + DynamicFusion 一句 → 动态/可变形章过门；`vo` 的"混合/学习"格 + `loop_closure` 的 `sec:loop-dl` → 学习式 SLAM 章过门；`vio` 的"学习增强惯性"段 → 学习式章过门。**每个新章开篇都"接住"一个现有锚点**，而非空降。
- **新 part 内部递进**：学习式（怎么把网络织进因子图）→ 动态/可变形（非静态世界）→ 度量-语义（闭集语义）→ 开放世界（开集/基础模型/VLA）→ 结语（是否还需要地图，呼应 intro）。**ch16→ch17 的闭集→开集是 Handbook 自己就设好的递进，直接沿用**。
- **数学贯穿做"缝合线"**：四章都反复回扣本书母方程 $J(\mathbf x)=\sum\|\mathbf e_i\|^2_{\boldsymbol\Sigma_i}$（ch13 学残差/权重、ch15 加动态因子、ch16 加物体/语义因子、ch17 加 range-feature）与右扰动/SE(3)/log-odds（ch16 多类 = 本书二值的推广）。**用同一套数学语言把前沿与经典缝成一体**——这是"浑然一体"最硬的保证。

### 备选与不推荐

- **备选 A（把 P5_engineering 复活并扩为前沿 part）**：现 `P5_engineering` 已存在但**未装配进 book.tex**（无 part.tex、未 `\input`）。可考虑把前沿 part 与工程实践合并或并列，但二者主题不同（工程 vs 前沿），不宜混。建议前沿**单立新 part**，P5 另议。
- **不推荐：分散硬塞进 P2 各章**。理由见上（撑爆经典章、断主线、独立性难统一管控、违"浑然一体"）。
- **不推荐：把 ch17 §17.4/ch19 单独成 part**。结语体量小，作前沿 part 的收尾章即可，单立 part 头重脚轻。

---

## 8. 工作量与优先级汇总

| 章 | 覆盖度 | 落点 | 新建 | 工作量 | 独立性最大风险 |
|---|---|---|---|---|---|
| ch13 学习式 | 部分有→大部分全新 | 新 part 首章；锚 `ch:vo`/`ch:nlopt`/`sec:loop-dl` | **新章** | **大** | 可微分 BA 反传跨章 punt 到 Handbook ch4，须自补 |
| ch15 动态可变形 | 概览预告→推导全新 | 新 part；升级 `sec:intro-dynamic`；锚 `ch:dense` | **新章** | **大** | 形变能量泛函（ARAP/ED/Killing）全 punt，须自补 |
| ch16 度量-语义 | 仅术语→方法全新 | 新 part；锚 `tab:dense-maptypes`/`ch:nlopt` | **新章** | **大** | 求解器/场景图谱系/§16.5 综述罗列；记号须对齐右扰动 |
| ch17 开放世界 | 几乎全新 | 新 part；§17.3 技术主体 + §17.4 入结语；锚 preface/`sec:intro-need` | **新章+结语** | **中-大** | §17.4 ventriloquize 重灾区，须剥立场留事实 |
| ch19 结语 | N/A 格言 | 结语章精神素材 | 融入结语 | **小** | 照搬即格言集锦 ventriloquize |
| 00 Notation | — | `front/notation.tex` 新增"学习/语义层"记号 | 登记新符号 | **小** | 帧下标方向 + 右扰动统一；避让 $\mathbf f$/$s$ |

**建议吸收顺序**（先地基后前沿、先经典接口后纯展望）：ch13（可微分 BA 接母方程，最承上启下）→ ch16（语义因子接母方程，闭集）→ ch17 §17.3（开集，接 ch16）→ ch15（动态/可变形，相对独立）→ ch17 §17.4 + ch19（结语收尾，最后写、最克制）。refs.bib 增补条目极多（四章合计上百篇，尤以 ch13 pointmap 系、ch16 物体/场景图系、ch17 基础模型/VLA 系为最），应集中由专人合并，勿多 agent 同改 refs.bib。
