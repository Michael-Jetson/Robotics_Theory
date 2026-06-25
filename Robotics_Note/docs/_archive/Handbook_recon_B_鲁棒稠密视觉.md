# Handbook 吸收侦察报告 B：鲁棒 / 稠密地图 / 视觉SLAM / 可微体渲染

> 只读侦察（recon），不改任何 `.tex`、不碰 `refs.bib`。
> 负责 HB 章：03 Robustness / 05 Dense Map Representations / 07 Visual SLAM / 14 Differentiable Volume Rendering Maps。
> 对应现有教材章：`P1_estimation/nonlinear_optimization.tex`、`P2_slam/{dense_mapping,visual_odometry,vio,camera_model}.tex`。
> 日期：2026-06-18。最高准则：**融合而非拼贴**——目标是把 HB 织进“已融十四讲+Barfoot”的那条单一叙事线，让读者读到一个作者的连贯论述。

---

## 总体判断（先给结论，细节见各节）

侦察后最重要的发现：**我这一组的现有教材已经吸收了 HB 这四章的相当一部分**，而且大多以本书口吻+`\cite{carlone2026handbook}` 落地。具体：

- **HB07 视觉SLAM**：现有 VO/VIO/camera_model 三件套已覆盖约 **85%**。尤其 `camera_model.tex` 的 `\sec:cam-modern` 几乎是 HB §7.3.1 的完整本土化（针孔/rad-tan/BC/KB/DS 谱系、抽象 $\pi/\pi^{-1}$、FOV 选型表、双球 insight、卷帘、光度模型全有）。真增量是窄而具体的若干点。
- **HB05 稠密地图**：`dense_mapping.tex` 覆盖经典表示（占据/TSDF/surfel/mesh/点云）很完整，且已吸收 ch5 的“度量/拓扑/语义”分类轴；但**显式地把 ch5 的另一条核心组织线——“显式/隐式 × 表面/体积”四象限 + ESDF + 连续函数（GP/Hilbert）表示——推给了“延伸阅读”，没进正文**。这是 ch5 真正的、可定位的正文缺口。
- **HB03 鲁棒**：`nonlinear_optimization.tex` 的 `\sec:nlopt-robust` 已有 M-估计五核谱系、IRLS 完整推导、影响函数、甚至“鲁棒=逆 Wishart 先验 MAP”的完整证明（这其实就是 Black–Rangarajan 对偶的一个特例！），RANSAC、GNC 也有提及。真缺口集中在**前端几何外点剔除（RANSAC 在 SLAM 里的角色、PCM/最大团）** 与 **GNC/B-R 对偶的完整算法化**。
- **HB14 NeRF/3DGS**：**全书零覆盖**（仅 point_cloud 表格里一格“SDF/NeRF”字样、surfel 节里“splatting”指面元渲染）。这是唯一真正“全新前沿”的一章。

**slice 级结构建议（详见文末第 5 节）**：**不**聚成独立“前沿专题 part”。03/05/07 应继续织进现有章（深化+局部补子节）；唯独 14（NeRF/3DGS）单立一节，挂在 `dense_mapping.tex` 末尾、作为“经典稠密表示 → 可微渲染地图”的自然延伸。理由：本书叙事线是按“问题/传感器”组织的（估计→SLAM→稠密→…），HB 这四章恰好分别落在已有的“非线性优化”“稠密建图”“视觉SLAM”三条线上，拆开织入维持单一叙事；只有可微渲染地图是一个新表示家族，值得一个有标题的落点，但仍属“稠密地图”大类，故为节、不为章、更不为 part。

---

## HB03 — Robustness to Incorrect Data Association and Outliers

### 覆盖度：**部分有**（后端 M-估计已深，前端几何剔除几乎没有）

### 现有落点
- 主落点：`parts/P1_estimation/nonlinear_optimization.tex` → `\section{鲁棒核函数与 M-估计}`（`\label{sec:nlopt-robust}`，约 937–1048 行）。
- 配套：附录推导 `\label{der:nlopt-irls}`（IRLS 论证）、`\label{der:nlopt-robust-iw}`（鲁棒=逆 Wishart MAP 完整证明，用 Sherman–Morrison 收拢）。
- 滤波侧也已埋伏：`kalman_eskf.tex` 提过 Huber/Cauchy/GM 核与 IW 先验结论（nlopt 节自称是“完整版”）。
- RANSAC 公式 `\label{eq:nlopt-ransac}`（迭代次数 $k=\ln(1-p)/\ln(1-w^n)$）在本节内，但只作为“硬剔除基线”一笔带过。
- 现有主线记号：标量马氏范数 $u_i=\sqrt{\mathbf{e}_i^\top\boldsymbol{\Sigma}_i^{-1}\mathbf{e}_i}$、影响函数 $\psi(u)=\rho'(u)$、IRLS 权 $w(u)\propto\psi/u$、膨胀协方差 $\mathbf{Y}_i$、信息矩阵 $\boldsymbol{\Lambda}=\mathbf{J}^\top\mathbf{W}^{-1}\mathbf{J}$、右扰动 $\boxplus$、numerator-layout。与 HB 完全一致（HB 的 $\rho(r_i)$、$\psi$、$w_i$ 与本书 $u/\psi/w$ 一一对应）。

### HB 增量清单（HB 比现有多讲了什么）
1. **问题来源的系统论述**：HB §3.1 把外点拆成两类成因——(a) 数据关联错误（前端误匹配/感知混叠 perceptual aliasing，举了地标 SLAM 与位姿图回环两例）；(b) 模型假设违背（动态物体、传感器退化）。现有教材只在 pitfall 里一句“误匹配/动态物体/GPS 多径”。**这是把“为什么有外点”讲透的整段叙事，本书缺。**
2. **前端 vs 后端的二分框架**：HB 的骨架是“前端剔除（§3.2）→ 后端鲁棒化（§3.3）”。本书只有后端，前端被压成 RANSAC 一句话。
3. **RANSAC 在 SLAM 里的真正角色**：HB 给出 consensus maximization 的优化式 (3.4)、最小集/最小求解器（Nister 五点法）、$1/\omega^n$ 期望迭代数、以及**何时失效**（最小集大、外点多 → 期望迭代爆炸，例 $n{=}10,\omega{=}0.1\Rightarrow10^{10}$ 次）。本书只有迭代次数公式，无“几何约束 $C(z_i,x)\le\gamma$ + 共识集”的优化视角。
4. **图论外点剔除 / 成对一致性最大化（PCM）**——**全新**：一致性函数 $F(z_i,z_j)\le\gamma$（与状态无关、可预计算），3D-3D 例 (3.5) 与位姿图回环“回路复合到单位”例 (3.6)；一致性图 → **最大团（maximum clique）**等价 (3.8)；与 RANSAC 的细致权衡。本书完全没有。
5. **Black–Rangarajan 对偶的一般形式（定理 3.4 + 外点过程 $\Phi_\rho(w_i)$）**：本书有“鲁棒=IW 先验 MAP”的 Cauchy 特例证明，但**没有把它点明为 B-R 对偶的一般定理**，也没有 truncated quadratic 的 $\Phi_\rho=(1-w_i)\beta^2$、G-M 的 $\Phi_\rho=\beta^2(\sqrt{w_i}-1)^2$ 这套“外点过程”语言，以及由此自然导出 IRLS 交替最小化、并落到 **dynamic covariance scaling** 的事实。
6. **截断二次损失 / 最大共识损失**：本书五核里没有 truncated quadratic（只有 Tukey 这个 redescending 代表），也没把“最大共识损失 ↔ consensus maximization”这条线点出来。
7. **GNC（图渐进非凸）完整算法**：HB §3.3.4 给了 GNC truncated quadratic (3.24)、GNC G-M (3.25) 的显式光滑族、$\mu$ 控制凸性、三步算法（变量更新/权更新/$\mu$ 更新），并指出 B-R 对偶对光滑核仍成立。本书只在 insight/pitfall 里一句话提 GNC（Yang 2020）。
8. **实验直觉与失败案例**：M3500/SubT/Victoria Park 三数据集，对比 LS/GD/IRLS/GNC/PCM+GNC 的轨迹；以及“无里程计骨架时连 GNC 也崩、PCM 前端+GNC 后端最稳、M3500 仍全崩”的诚实结论。本书无此类整体性直觉。
9. **可证最优（certifiable）求解器**：SDP 松弛 + 事后最优性证书，旋转/3D-3D/位姿图三处。本书 nlopt 仅一句“SE-Sync 留作前沿”。

### 融合策略（重中之重）
**判断：局部重构 + 新增一个前端子节**，把现有“后端 M-估计”升级为“前端剔除 + 后端鲁棒化”的完整二分叙事。具体：

- **(a) 在 `\sec:nlopt-robust` 开头插入一段“外点从哪来 + 前端/后端二分”过渡**（深化现有论述）。承接句可从现有节首“真实数据有外点（误匹配、动态物体、GPS 多径）”自然展开成 HB §3.1 的两类成因，并预告“先在前端用几何剔除（本节新增），再在后端用鲁棒核（原有内容）”。保持主线：仍以 $u_i/\psi/w$ 和 MAP→LS 为锚。
- **(b) 新增子节 `\subsection{前端外点剔除：RANSAC 与成对一致性最大化（PCM）}`**，置于节首之后、现有“M-估计/核函数表”之前。内容织入增量 3+4：先把现有 RANSAC 公式提升为 consensus maximization 优化视角（最小集/最小求解器/期望迭代/失效条件），再**新增 PCM 全段**（一致性函数、3D-3D 与位姿图两例、一致性图→最大团等价、与 RANSAC 权衡）。这一子节是真增量，自包含可独立读懂。过渡到后端：用“前端剔除是必要非充分条件，残余外点仍会毒化二次代价”承上启下（这正是 HB §3.3 的开场逻辑）。
- **(c) 深化现有 IRLS/IW 证明为“B-R 对偶一般定理”**（局部重构，不重写）：在现有“鲁棒=逆 Wishart MAP（Cauchy 特例）”证明**之后**加一小段，点明“此即 Black–Rangarajan 对偶的一个实例”，给出一般定理 3.4 的外点过程 $\Phi_\rho(w_i)$ 语言，并补 truncated quadratic / G-M 两个 $\Phi_\rho$ 实例 + 交替最小化两步 + “G-M 实例即 dynamic covariance scaling”。**重复部分（Cauchy 的 IRLS 权、IW 收拢）跳过，只补“一般化”的壳。**
- **(d) 在核函数表里补 truncated quadratic 一行 + “最大共识损失↔consensus maximization”一句**（深化），与 (b) 的前端 RANSAC 呼应闭环。
- **(e) 把现有一句话的 GNC 扩成一个小子节或 box `\subsection{图渐进非凸（GNC）}`**：给 GNC truncated quadratic / G-M 光滑族 + 三步算法 + “B-R 对偶仍成立”。接在 IRLS 之后最自然（HB 也是此序）。
- **(f) 实验直觉 + certifiable**：以 1–2 段“工程直觉”形式收尾（增量 8+9），不必搬全部图；certifiable 仍可保持“前沿指引”力度但补 1–2 句 SDP 松弛 + 最优性证书的实质，避免停留在书名。

> 跳过/不织：HB §3.1.2 用平方放大外点影响的论证（本书已有“二次型对外点极敏感”）；Huber/Cauchy/GM/Tukey 核定义与影响函数（本书表已全）。

### 独立性风险
- 中。**严禁**写“HB 第 3 章指出/坦言”。PCM、GNC、B-R 对偶定理、certifiable 都要 `\cite` + 本书口吻平衡转述。
- HB 大量实验依赖 GTSAM 的 `GNCOptimizer`/`NonlinearConjugateGradientOptimizer`——本书若引用要么自包含描述算法、要么落到本书既有的 Ceres/g2o 口径，不要变成“GTSAM 这么做”的叙述依赖。
- B-R 对偶定理 3.4 的条件（$\phi(z):=\rho(\sqrt z)$，$\lim_{z\to0}\phi'=1$、$\lim_{z\to\infty}\phi'=0$、$\phi''<0$）须自包含写出，能独立读懂。

### 是否需要新建子节 / 新章 / 新 part
**新增 2 个子节（前端 PCM/RANSAC；GNC），局部重构 B-R 对偶段，深化节首与核表。不新章、不新 part。**

### 工作量：**中**（PCM 与 GNC 是真增量需新写并配图/例；其余是深化与点题）

---

## HB05 — Dense Map Representations

### 覆盖度：**部分有**（经典显式表示很全；隐式/连续函数家族与“四象限”组织线缺）

### 现有落点
- 主落点：`parts/P2_slam/dense_mapping.tex`（`\label{ch:dense}`）。
- 关键现有节（带 label）：
  - `\sec:dense-intro` 地图用途与分类——**已吸收 ch5 的度量/拓扑/语义分类轴**（`\label{tab:dense-maptypes}` 明确 `\cite{carlone2026handbook}`，insight“表示应由用例决定”也引 ch5）。
  - `\sec:dense-octomap` 占据/OctoMap/八叉树（log-odds、clamping、八叉树压缩）——**全治**。
  - `\sec:dense-tsdf` TSDF/KinectFusion（投影 SDF、加权运行平均、frame-to-model）——**全治**。
  - `\sec:dense-surfel` surfel/ElasticFusion（含 surfel splatting 渲染、deformation graph、point-to-plane+光度跟踪）——**全治**。
  - `\sec:dense-mesh` 网格（Marching Cubes 全 + 泊松一段 + MLS/GP3 代码）——**全治**。
  - `\sec:dense-rgbd-pc` 点云地图（反投影、SOR、体素降采样）——一节。
  - `\sec:dense-engineering` 工程选型总表（`\label{tab:dense-engineering}`）。
- 现有主线记号：$\mathbf{R}\in SO(3)$、$\mathbf{T}\in SE(3)$（如 $\mathbf{T}_{wc}$）、Hamilton 四元数、相机内参 $\mathbf{K}(f_x,f_y,c_x,c_y)$、深度 $d$=沿射线 range、逆深度 $\rho_{\mathrm{inv}}=1/d$（注意已为避让 $\xi=[\rho;\phi]$ 而不裸用 $\rho$）、占据 $p$/对数几率 $\ell$、TSDF 值 $F$/权 $W$/截断 $\mu_{\mathrm{tr}}$。与 HB 记号兼容（HB 的 range image $\mathbf{R}$、点云 $\mathsf P$、ESDF、log-odds 都能对接本书记号）。

### HB 增量清单
1. **“显式/隐式 × 表面/体积”四象限组织线 + 显隐式互转**——HB §5.3.1/5.3.2/5.5：把所有表示统一进四象限（显式表面=点/surfel/mesh；隐式表面=SDF/TSDF/GP；显式体积=占据/距离体素；隐式体积=GP/Hilbert），并系统讲互转（点/mesh↔隐式 via closest-point/fast marching；隐式→mesh via Marching Cubes；离散→连续 via 优化拟合）。**本书把这条线推给了“延伸阅读”，没进正文**——这是 ch5 最值得补的“骨架”。
2. **ESDF（欧氏符号距离场）与 TSDF 的区别 + voxblox/FIESTA**——HB §5.2.2/5.4.4.1：ESDF（查询点到最近面的真欧氏距离，用于碰撞检测与基于优化的规划的高质量梯度）vs 投影 SDF/TSDF（过估欧氏距离、需截断带）；voxblox 从 TSDF 增量建 ESDF（brushfire），FIESTA 从占据增量建 ESDF。**本书有 TSDF 无 ESDF**，而 ESDF 对规划极重要。
3. **范围传感前置（§5.1）**：LiDAR 测距方程 $r=c(t_{\text{detect}}-t_{\text{emit}})/2$、range image $\mathbf{R}\in\mathbb{R}^{B\times M}$、球坐标转点云、organized/unorganized 点云、**运动畸变去畸变**。本书 RGB-D 反投影有，LiDAR 这套主要在 `lidar_slam.tex`/`point_cloud_processing.tex`（侦察确认 dense_mapping 不重复）。
4. **数据结构与存储（§5.3.3）系统化**：naive array / **hash map**（分片/哈希函数/碰撞）/ **tree（kD-tree、BVH、octree 八分构造）** / **hybrid（voxel-block hashing、VDB=块哈希+树）**。本书八叉树有，但 hash map、voxel hashing、VDB、kD-tree 这套“数据结构维度”散在别处或缺，未在 dense_mapping 系统化。
5. **测量整合算法（§5.4.4.3）**：ray-tracing vs projection-based 整合器的权衡（竞态、并行/GPU、对无序点云的适配）。本书 TSDF 节有 frame-to-model 但无此“整合器”维度。
6. **可扩展性（§5.4.4.4）**：多分辨率 ray-tracing/投影、Supereight2 按测量熵调分辨率、**wavelet/wavemap 粗到细**。本书无。
7. **连续函数 / 概率表示家族**——**本书几乎全缺**：
   - **高斯过程地图 GPOM**（§5.4.5）：GP 回归式 (5.8)–(5.11)、squashing 成占据、$O(J^3)$ 复杂度；
   - **GPIS**（GP 隐式曲面，§5.4.5.1）：联合 GP 估距离场+梯度 (5.12)、不确定度；
   - **Hilbert 地图**（§5.4.6）：投影特征空间+logistic 回归+SGD、稀疏核。
   这条线给出“概率、连续、可量化不确定度、可外插未观测区”的表示，与本书既有的“贝叶斯滤波建图”精神一致，但本书没把它作为地图表示讲。
8. **深度学习建图过渡（§5.4.7）**：神经隐式 SDF（iMAP 系）、特征网格/octree/点解耦神经解码器——这是通向 ch14 的桥，HB 在此明确转场 ch14。

### 融合策略（重中之重）
**判断：新增 1 个“隐式与连续函数表示”子节（把已推给延伸阅读的 ch5 骨架收回正文）+ 在既有节里深化（ESDF/数据结构/整合器）+ 把第 8 点作为通向 HB14 节的过渡桥。**

- **(a) 把四象限组织线收回正文**（局部重构 `\sec:dense-intro` 或新增过渡）：现有 `\sec:dense-intro` 已讲度量/拓扑/语义，可在其后补一小段“另一条正交分类：显式/隐式 × 表面/体积”，并用它**重新串起本章后续各节**（点/surfel/mesh=显式表面，OctoMap=显式体积，TSDF/SDF=隐式表面…）。这样本章读起来是“一个分类框架贯穿到底”，而非表示罗列。过渡句：承现有“表示应由用例决定”insight，转入“而从空间抽象看，所有这些表示落在四个象限……”。
- **(b) 新增子节 `\subsection{隐式表面、ESDF 与连续函数地图}`**，置于 `\sec:dense-tsdf` 之后（隐式表面承 TSDF）或 `\sec:dense-mesh` 之后、`\sec:dense-engineering` 之前：
  - 隐式曲面定义（零交叉=表面、符号判内外）+ ESDF vs TSDF（增量 2，含 voxblox/FIESTA 一句），深化现有 TSDF（本书已讲 TSDF 是“投影 SDF 加权平均”，这里只需补“它过估欧氏距离、ESDF 才是真距离场、规划要 ESDF 梯度”）；
  - **GP 地图 + GPIS + Hilbert 地图**（增量 7）作为“连续/概率表示”小段。自包含写出 GP 预测式 (5.10)(5.11) 与 GPIS 联合式 (5.12)，与本书既有“贝叶斯滤波”记号呼应（GP 的不确定度↔本书一贯强调的协方差/信息）。这是真增量，需独立可读。
- **(c) 数据结构维度**（深化）：在 `\sec:dense-octomap` 既有八叉树之后或工程选型表附近，补一小段把 hash map / voxel-block hashing / VDB 串起来（增量 4）。本书已多处提“TSDF 大场景需体素哈希”，此处只需把哈希/VDB 讲清，避免停在一句话。**kD-tree 在 point_cloud 已有，dense_mapping 只引不重复。**
- **(d) 整合器与可扩展性**（轻量深化）：ray-tracing vs projection-based、多分辨率/wavemap 可作为 1 段“工程注记”或 insight，挂在 TSDF/选型表附近（增量 5+6）。不必展开成大节。
- **(e) 第 8 点作为过渡桥**：在本章末把“神经隐式 SDF（iMAP）/特征网格解码器”作为 1–2 句过渡，**直接引出 HB14 的可微体渲染地图节**（见下）。这样 ch5→ch14 的转场与 HB 原书一致，叙事连贯。

> 跳过/不织（已重复）：占据 log-odds、TSDF 加权平均、Marching Cubes、surfel/deformation graph、泊松——本书已全治。LiDAR 测距/去畸变交给 `lidar_slam.tex`，dense_mapping 用 `\cref` 指过去即可。

### 独立性风险
- 中。`\sec:dense-intro` 已正确用“综合十四讲与 Handbook”的平衡口吻 + `\cite`，新增内容延续此风格即可。
- GPOM/GPIS/Hilbert 的公式须自包含（GP 预测式、squashing、稀疏核思想），不能写“详见 ch5”。
- 注意 `\cite{carlone2026handbook}` 在本章已高频出现，新增段落继续 `\cite` 但避免“Handbook 把成像分两分量”式的叙述依赖措辞（camera_model 节已有此倾向，见下 07 风险）。

### 是否需要新建子节 / 新章 / 新 part
**新增 1 个核心子节（隐式/ESDF/连续函数）+ 数据结构与整合器的深化段。不新章、不新 part。**

### 工作量：**中**（四象限收编是重构性的；GP/GPIS/Hilbert 是真增量需新写公式；ESDF/数据结构是深化）

---

## HB07 — Visual SLAM

### 覆盖度：**已有（约 85%）**——这是覆盖最充分的一章，真增量窄而具体

### 现有落点（横跨三件套，侦察已确认各自边界、互不重复且彼此 `\cref`）
- `parts/P2_slam/camera_model.tex`（`\label{ch:camera}`）：
  - `\sec:cam-modern` 现代相机模型谱系——**几乎=HB §7.3.1 的完整本土化**：抽象 $\pi/\pi^{-1}$（`\eq:abstract-pi`）、针孔/rad-tan/BC/KB（`\eq:kb`）/双球 DS（`\eq:ds`）全有、FOV 选型表 `\tab:lens`、insight“为何偏爱双球（6 参=8 参精度、5× 快、闭式可逆）”、卷帘快门段、光度模型 `\eq:photometric`、用错模型 pitfall。
  - `\sec:cam-obs` 相机作为可微观测模型与投影雅可比。
  - `\sec:cam-epipolar` 本质/基础/单应。
- `parts/P2_slam/visual_odometry.tex`（`\label{ch:vo}`）：
  - `\sec:vo-feature`（ORB/FAST/BRIEF，含历史 Harris/SIFT/SURF/BRISK 提及）、`\sec:vo-epipolar`（对极/八点/单应）、`\sec:vo-triangulation`、`\sec:vo-pnp`（DLT/P3P/重投影 BA `\eq:vo-reproj-cost`，含 MLE→加权重投影推导）、`\sec:vo-icp`、`\sec:vo-flow`（LK 光流）、`\sec:vo-direct`（直接法/光度误差，含 DSO 辐照度恒定+完整光度标定一段）、`\sec:vo-frontend`（`\sec:vo-ptam` PTAM 并行跟踪建图+关键帧、`\sec:vo-schur` Schur 补消元、`\sec:vo-systems` 完整系统/混合/新趋势）。
- `parts/P2_slam/vio.tex`（`\label{ch:vio}`）：IMU 预积分、松/紧耦合、VINS-Mono 优化主线、MSCKF 滤波主线、可观性 4-DOF、初始化、外参/时间同步。回环/词袋在此仅一句（`DBoW2`），**全治在 `loop_closure.tex` / `slam_system.tex`**（VO/VIO 都正确 defer）。
- 现有主线记号：$\mathbf{T}\in SE(3)$ 右扰动 $\mathbf{T}\,\mathrm{Exp}(\delta\boldsymbol\xi)$、$\xi=[\rho;\phi]$、投影 $\pi$、重投影残差 $\mathbf{e}=\mathbf{u}_i-\pi(\mathbf{T}\mathbf{P}_i)$、本质矩阵 $\mathbf{E}=\mathbf{t}^\wedge\mathbf{R}$、Schur $\mathbf{H}_{cc}^{\mathrm{red}}=\mathbf{H}_{cc}-\mathbf{H}_{cp}\mathbf{H}_{pp}^{-1}\mathbf{H}_{cp}^\top$、Hamilton 四元数（VIO）。与 HB §7 记号高度一致（HB 用 $\pi(x^c,\xi)$、$e_{\mathrm{reproj}}=z_j-\pi$、同样的 Schur 块式）。

### HB 增量清单（窄而具体）
1. **历史/术语谱系**：HB §7.1 摄影测量→BA→SfM→VO→VSLAM 的源流（von Gruber 形式化 BA、Schmid/Brown 上计算机、Tomasi-Kanade 正交分解、Building Rome in a Day）与 5 个术语的精确区分（photogrammetry/BA/SfM/VO/VSLAM）。本书散见一句话，无成段谱系。
2. **学习型关键点的成段论述**：SuperPoint/LIFT/HF-Net/D2-Net（describe-and-detect 反转检测顺序）。本书目前只在时间线图/表里列名，无 1–2 段实质介绍。
3. **直接法系统的更完整对照**：HB §7.4.2 把 LSD-SLAM（交替跟踪+建图+PGO）、DSO（单次 GN 联合光度 BA+滑窗边缘化）、PGBA（Pose Graph Bundle Adjustment，只更新位姿却纳入 BA 全光度不确定度）讲成一条线。本书 `\sec:vo-direct` 有直接法与 DSO，但 **PGBA 缺**，LSD vs DSO 的“交替 vs 联合”对照可深化。
4. **Power Bundle Adjustment + 免初始化 BA（变量投影）**——**真增量**：HB §7.4.4 用矩阵幂级数近似 reduced camera system 之逆 (7.8)(7.9)、variable projection 解耦相机-路标 chicken-and-egg、二者结合做大规模免初始化 BA。本书 `\sec:vo-schur` 末仅一句提“Power BA”。
5. **滤波 vs 关键帧的稀疏性论证（图 7.4）**：EKF-SLAM 边缘化历史位姿→稠密图（限百级特征、且不重线性化），关键帧法保稀疏。本书 vio 有 MSCKF vs VINS 对比，但此“边缘化毁稀疏”的可视化论证可强化（VO 侧）。
6. **实时单目稠密重建（§7.6）变分法**：TV 正则 (7.10)(7.11)、soap-film fill-in。本书 dense_mapping 走的是块匹配+高斯滤波路线，此变分/TV 路线缺（可放 dense_mapping 或 VO 直接法附近）。
7. **协视图/本质图（covisibility/essential graph）**：HB 图 7.5/7.6 的局部 BA 协视准则。本书 `\sec:vo-ptam` 提“共视准则”一句，essential graph 缺（但回环/位姿图主战场在 `slam_system.tex`）。
8. **学习型匹配（§7.9）**：SuperGlue（自/交叉注意力+最优匹配层）、MASt3R/MASt3R-SLAM（3D-aware 匹配）。本书表里列名，无实质。
9. **GPS/WiFi 全局定位、IMU 减 gauge 自由度（7→4）**：HB §7.8 把多模态融合（含 RTK-GPS）与“IMU 把 7 自由度规范降到 4（x,y,z,yaw）”讲清。本书 vio 有紧/松耦合与初始化，gauge 自由度计数可补一句。

### 融合策略（重中之重）
**判断：以“深化 + 点状补子节”为主，绝不另起炉灶。三件套已是骨架，HB07 增量逐条贴到既有 label 旁。**

- **(a) camera_model 几乎不动**：`\sec:cam-modern` 已是 HB §7.3.1 的完整版。仅做独立性清理（见风险），不再吸收。
- **(b) VO 历史/术语**（深化）：在 `\sec:vo-feature` 之前或 `ch:vo` 开篇补一小段摄影测量→BA→SfM→VO→VSLAM 谱系 + 术语精确区分（增量 1）。承接句可从本书既有“视觉 SLAM 三大子功能”自然引出。
- **(c) 学习型关键点 + 学习型匹配**（新增 1 子节）：在 `\sec:vo-feature`（ORB 之后、`\sec:vo-epipolar` 之前）新增 `\subsection{学习型关键点与匹配}`，把 SuperPoint/D2-Net（describe-and-detect）/SuperGlue/MASt3R 讲成 1–2 段（增量 2+8），**聚焦“在 SLAM 流水线里替换了哪一环”而非训练细节**，与本书既有“经典手工特征”形成对照闭环。
- **(d) Power BA + 免初始化 BA**（深化 `\sec:vo-schur`）：现有 Schur 节末已提“Power BA”，把它扩成 1 段——矩阵幂级数近似 reduced system 逆（自包含写 (7.9)）+ variable projection 解耦 + 二者结合免初始化（增量 4）。这是 Schur 主线的自然延伸，主线（reduced camera system）不变。
- **(e) PGBA + LSD/DSO 对照**（深化 `\sec:vo-direct`/`\sec:vo-systems`）：把 PGBA 作为“位姿图 BA：只更新位姿但保留 BA 全光度不确定度”补进直接法全局一致性那段（增量 3），并把 LSD（交替）vs DSO（联合）的对照点透。
- **(f) 滤波毁稀疏论证 + gauge 自由度**（深化 vio）：在 vio 的“两条主线对比”里补 HB 图 7.4 的“EKF 边缘化历史位姿→稠密图、不重线性化”论证（增量 5）与“IMU 把 gauge 7→4”一句（增量 9）。
- **(g) 变分稠密重建（§7.6）**：放 dense_mapping 单目稠密节或 VO 直接法附近，作为“另一条单目稠密路线（变分+TV）”补 1 段（增量 6）。优先 dense_mapping（与块匹配/高斯滤波并列）。

> 跳过/不织（已重复）：针孔/rad-tan/KB/DS/卷帘/光度模型（camera_model 全有）、重投影误差 MLE 推导（`\eq:vo-reproj-cost` 已有）、full/pose-only/local BA + PTAM 三线程（`\sec:vo-ptam` 已有）、Schur 块消元（`\sec:vo-schur` 已有）、IMU 预积分/紧松耦合/MSCKF/VINS（vio 全有）、回环/词袋（`loop_closure.tex` 主场）。

### 独立性风险
- **较高**（因为 camera_model/VO 已大量 `\cite{carlone2026handbook}`，且 `\sec:cam-modern` 多处用“Handbook 把成像分两分量”“Handbook 由此给出一条谱系”这类**叙述依赖**措辞——这是已存在的待清理点，新增内容不要再叠加）。建议趁此侦察标记：把“Handbook 把…”改写成本书口吻 + `\cite`。
- 学习型方法（SuperPoint/SuperGlue/MASt3R/Power BA）要 `\cite` 原文，不写“HB 第 7 章介绍/强调”。
- 系统名（ORB-SLAM/DSO/LSD/OKVIS）已大量出现，继续保持“平衡对照”而非“某系统坦言”。

### 是否需要新建子节 / 新章 / 新 part
**新增 1 个子节（学习型关键点与匹配）+ 多处深化（历史/Power BA/PGBA/滤波稀疏/变分稠密）。不新章、不新 part。**

### 工作量：**小–中**（多为深化与点状补写；唯一较实的新增是学习型关键点子节与 Power BA 段）

---

## HB14 — Map Representations with Differentiable Volume Rendering（NeRF / 3DGS）

### 覆盖度：**完全没有**（全书唯一真正零覆盖的前沿章）

### 现有落点
- **无正文落点。** 全书仅两处影子：`point_cloud_processing.tex:163` 表格一格“隐式表示 …（如 SDF/NeRF）”；`dense_mapping.tex` surfel 节里“surfel splatting”指面元渲染（非 3DGS）。`dense_mapping.tex` 延伸阅读引了 ch5 但未引 ch14。
- 可挂接的现有锚点：`dense_mapping.tex` 的隐式曲面/TSDF/surfel 叙事线（HB14 §14.2.4 明确说神经场相对 TSDF-SLAM 的优势是“端到端可微、省去手工融合步骤”；§14.3 的 3DGS 与 surfel 同源——HB05/HB14 都点明 3DGS≈带不透明度的 surfel）。

### HB 增量清单（整章皆新）
1. **可学习 3D 表示 → 可微渲染（§14.1）**：DeepSDF/ONet（MLP 回归 SDF/占据，需 3D 监督）→ 可微渲染（让梯度从 2D 图回流 3D 参数）；表面渲染（光栅化）vs 体渲染（ray marching）二分。
2. **NeRF（§14.2）**：$f_\Theta(\mathbf{x},\mathbf{d})\to(\mathbf{c},\sigma)$、体渲染积分 (14.1) + 求积离散 (14.2)（透射率 $T_i$、不透明度 $\alpha_i=1-e^{-\sigma_i\delta_i}$、alpha 合成）、光度损失 (14.3)、位置编码；数据结构加速（Plenoxel/DirectVoxGo 体素特征格、Instant-NGP 多分辨率哈希编码——天→秒）；点/surfel NeRF（Point-NeRF）。
3. **Neural Fields 泛化（§14.2.3）**：SDF/占据场（NeuS/VolSDF/UNISURF，可 Marching Cubes 出网格）、语义场。
4. **NeRF/Neural Field for SLAM（§14.2.4）**：iMAP（首个，纯 MLP，平滑先验→补洞/压缩）、NICE-SLAM（多分辨率体素特征格+小 MLP 混合）、NeRF-SLAM（借 DROID-SLAM 深度加速）；数据结构 trade-off 表（MLP/体素哈希/点 在压缩/推理速度/补洞/动态分配/遗忘问题上的取舍）；**遗忘问题（forgetting problem）**、回环时局部子图 vs 点表示无缝重排。
5. **3D Gaussian Splatting（§14.3）**：每高斯 $(\boldsymbol\mu,\boldsymbol\Sigma,o,\mathrm{SH}\,\mathbf{c})$、$\boldsymbol\Sigma=\mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$、投影协方差 $\boldsymbol\Sigma'=\mathbf{J}\mathbf{W}\boldsymbol\Sigma\mathbf{W}^\top\mathbf{J}^\top$、alpha 合成 (14.6)、L1+SSIM 损失 (14.8)、自适应稠密化/剪枝、tile 化 radix sort、frustum culling；**可微光栅化 vs ray marching 的本质区别**（遍历图元 vs 沿射线行进，利用 3D 稀疏性 → 快）。
6. **3DGS for SLAM（§14.3.3）**：RGB-D（ToF 深度初始化高斯）、纯单目 MonoGS（随机初始化+多视优化）、LoopSplat（离散高斯作回环配准特征）、I²-SLAM（物理成像/运动模糊/HDR）、与点云方法融合（Photo-SLAM=ORB-SLAM+3DGS、GS-ICP、RTG-SLAM）、LiDAR+3DGS。
7. **前沿与局限（§14.4）**：实时性瓶颈（多数 <5fps，连 4090 都吃力）、前馈预测高斯（PixelSplat/MVSplat/No-PoSplat，免测试时优化、免位姿）、跟踪精度落后经典稀疏法、4D/动态、大尺度/室外。

### 融合策略（重中之重）
**判断：单立 1 个有标题的节，挂在 `dense_mapping.tex` 末尾（`\sec:dense-mesh` 之后、`\sec:dense-engineering` 之前/或选型表之后作为“前沿表示”），而非织进既有任一节、也不新开 part。**

理由（直接回答任务里点名的问题）：
- **为何织进 dense_mapping 而非单立新章/新 part**：NeRF/3DGS 在 HB 自身的定位就是“**一种稠密地图表示**”（HB14 标题即 *Map Representations with Differentiable Volume Rendering*，开篇就接 ch5）。它与本书 dense_mapping 的隐式曲面/surfel 叙事同源（3DGS≈可微 surfel；NeRF 的 SDF 场≈神经化的隐式曲面/TSDF）。放进稠密地图章，读者得到的是“经典稠密表示（点/面元/TSDF/GP）→ 可微渲染地图（NeRF/3DGS）”的**单一演进叙事**，正合“融合而非拼贴”。单开新 part 会把它从稠密地图叙事里割裂出去，反而像“另一桶颜料”。
- **为何单立一节而非织进 surfel/隐式子节**：它是一个**完整的新表示家族 + 新优化范式（可微渲染/端到端）**，体量与自包含度都够一节；塞进 surfel 节会撑爆且打断既有叙事。故“为节、不为章、不为 part”。

具体落地：
- **(a) 过渡桥（与 HB05 §5.4.7 一致）**：上承 dense_mapping 新增的“隐式/连续函数”子节末尾“神经隐式 SDF（iMAP）”那 1–2 句（见 HB05 融合 (e)），自然引出本节。过渡句立意：本书前面所有稠密表示都是“先重建几何、再（可选）渲染”；可微渲染地图反过来——“**用渲染损失直接优化 3D 表示**”，让 2D 像素的梯度回流 3D。
- **(b) 节内结构**（建议三段，全部自包含、能独立读懂）：
  1. `可微渲染与 NeRF`：可学习 3D 表示→可微渲染（表面/光栅化 vs 体积/ray marching 二分）；NeRF 体渲染积分 (14.1) 与离散求积 (14.2)（自包含写出 $T_i$、$\alpha_i$、alpha 合成）+ 光度损失；加速数据结构（Instant-NGP 哈希、体素特征格）一句带过。
  2. `3D 高斯泼溅（3DGS）`：高斯参数化 + $\boldsymbol\Sigma=\mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$ + 投影协方差 (14.5) + alpha 合成 (14.6) + L1/SSIM 损失 + 稠密化/剪枝；点透“可微光栅化 vs ray marching”的快在何处。**记号注意**：HB 此处 $\boldsymbol\Sigma$ 是高斯协方差、$\mathbf{R}\in SO(3)$ 是高斯朝向、$\mathbf{J}/\mathbf{W}$ 是投影/视变换——与本书 $\mathbf{R}$ 专留旋转、$\boldsymbol\Lambda$ 信息矩阵、$\mathbf{J}$ 雅可比的约定**有局部冲突**，须在节内显式声明“此处 $\boldsymbol\Sigma$ 为高斯协方差而非噪声协方差”等，避免读者混淆（独立性/一致性风险点）。
  3. `用于 SLAM`：NeRF-SLAM（iMAP/NICE-SLAM，含 MLP/体素/点的 trade-off 表 + 遗忘问题 + 回环）与 3DGS-SLAM（MonoGS 单目、与 ORB/ICP 融合的 Photo-SLAM/RTG-SLAM、LoopSplat 回环）；收尾用 §14.4 的局限（实时性 <5fps、跟踪精度、4D/室外）+ 前馈预测（No-PoSplat）作“前沿展望”，与本书既有“前沿指引”笔法一致。
- **(c) 呼应既有叙事**：明确点出 3DGS 与本书 `\sec:dense-surfel` 的 surfel 同源（HB05/14 都讲）、NeRF 的 SDF 场与本书隐式曲面/TSDF 的关系（NeuS 可 Marching Cubes 出网格——直接复用本书 `\sec:dense-mesh` 的 MC）、以及“端到端可微 vs TSDF 手工融合”的对照（呼应本书 TSDF 节）。这些呼应是“融合”的关键缝合点。

### 独立性风险
- **较高**（整章新、且记号冲突）。
  - 记号：高斯 $\boldsymbol\Sigma$/朝向 $\mathbf{R}$/投影 $\mathbf{J},\mathbf{W}$ 与本书主线（$\mathbf{R}$ 专留旋转、$\boldsymbol\Lambda$ 信息、numerator-layout $\mathbf{J}$）局部冲突——**必须节内声明**。NeRF 的 $\mathbf{d}=(\theta,\phi)$ 视向与本书 $\xi=[\rho;\phi]$ 的 $\phi$、$\rho$ 也可能撞符号，需避让或声明。
  - 全章须 `\cite` 原文（NeRF/3DGS/iMAP/NICE-SLAM/MonoGS/…），**严禁**“HB 第 14 章指出/Matsuki-Davison 坦言”式叙述依赖。
  - 体渲染积分、alpha 合成、$\boldsymbol\Sigma=\mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$、投影协方差都要自包含推/写，能独立读懂（全吸收标准）。
- 需新增 refs.bib 条目（NeRF、3DGS、iMAP、NICE-SLAM、Instant-NGP、MonoGS、DeepSDF、NeuS 等）——**本次只读侦察不动 bib，仅在此记录“吸收时需补引”**。

### 是否需要新建子节 / 新章 / 新 part
**新增 1 个独立节（三子段）于 `dense_mapping.tex` 末尾。不新章、不新 part。**

### 工作量：**大**（整章全新、含两套渲染范式的自包含数学、SLAM 应用谱系、记号避让；是这一组里最重的一块）

---

## 5. slice 级结构建议（整组：鲁棒 / 稠密地图 / 视觉SLAM / 可微体渲染地图）

### 结论：**整体织进现有章，不聚成独立“前沿专题”part。唯一“单立”是 HB14 的可微渲染地图节（仍挂在稠密建图章内）。**

### 理由
1. **本书叙事线是“按问题/传感器组织”**（P1 估计：非线性优化/滤波；P2 SLAM：相机/VO/VIO/稠密/点云/激光/回环/系统）。我这四个 HB 章恰好分别落在三条已有线上：
   - HB03 → `nonlinear_optimization.tex`（鲁棒估计本就是非线性优化的一节）；
   - HB07 → `camera_model + visual_odometry + vio`（视觉 SLAM 三件套）；
   - HB05 + HB14 → `dense_mapping.tex`（稠密地图表示，HB 自身也把 14 接在 5 之后）。
   把它们拆开织入，正好维持“一个作者按问题展开”的单一叙事；聚成“前沿专题 part”反而会把鲁棒/稠密/视觉从它们的自然语境里割裂，违背“融合而非拼贴”。
2. **覆盖度梯度决定策略，而非“前沿”标签**：HB07≈85% 已有（深化为主）、HB05/HB03 部分有（补关键子节）、HB14 零覆盖（单立节）。这是一条从“深化”到“新增”的连续谱，不存在一个统一的“全是新东西的前沿块”可供聚合——把 85% 已有的 HB07 也塞进“前沿 part”显然荒谬。
3. **HB14 单立但不离家**：它体量/新颖度够一节，但本质仍是“一种稠密地图表示”，留在稠密建图章内能与 surfel/隐式曲面/TSDF 形成“经典→可微渲染”的演进叙事。这是“单立”与“融合”的最佳折中。
4. **跨章缝合点已天然存在**：camera_model 的光度模型/直接法、dense_mapping 的 surfel/隐式曲面、nlopt 的鲁棒核——HB 这几章彼此引用的关系（ch3↔ch7 RANSAC、ch5↔ch14 表示、ch7↔ch5 地图）在本书既有 `\cref` 网里都能落位，无需新建结构容器。

### 落点一览（吸收时的施工顺序建议：先易后难）
| HB 章 | 落点文件 | 动作 | 工作量 |
|---|---|---|---|
| 07 视觉SLAM | camera_model / visual_odometry / vio | 深化 + 1 子节（学习型关键点）；独立性清理 | 小–中 |
| 03 鲁棒 | nonlinear_optimization | 2 子节（前端 PCM/RANSAC、GNC）+ B-R 对偶点题 + 核表深化 | 中 |
| 05 稠密地图 | dense_mapping | 1 子节（隐式/ESDF/连续函数）+ 四象限收编 + 数据结构深化 | 中 |
| 14 NeRF/3DGS | dense_mapping（末尾） | 1 独立节（NeRF/3DGS/SLAM 三段）+ 记号避让 + 补 bib | 大 |

### 配套提醒（吸收阶段，非本次侦察范围）
- 独立性专项清理：`camera_model.tex \sec:cam-modern` 现有“Handbook 把成像分两分量/由此给出谱系”等叙述依赖措辞，宜在吸收 HB07 时一并改成本书口吻 + `\cite`。
- refs.bib：HB14 需新增 NeRF/3DGS/iMAP/NICE-SLAM/Instant-NGP/MonoGS/DeepSDF/NeuS 等；HB03 需 PCM(Mangelson)/GNC(Yang)/Black-Rangarajan/certifiable 系列；HB05 需 GPOM/GPIS/Hilbert/voxblox/VDB 等。
- 记号一致性：HB14 高斯 $\boldsymbol\Sigma,\mathbf{R}$ 与本书主线冲突，须节内声明；HB05 GP 记号 $\mathcal{K}/K$ 与本书信息矩阵 $\boldsymbol\Lambda$ 不冲突可直接用。
