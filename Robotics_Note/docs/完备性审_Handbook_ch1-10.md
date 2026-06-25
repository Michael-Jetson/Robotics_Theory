# 完备性审：The SLAM Handbook 第 1–10 章吸收

> **审核员**：完备性审核员（独立复核）。**日期**：2026-06-18。**口径**：吸收 = 把源书一切有价值内容用本书口吻重写进教材——不止知识点，而是五类：(a) 知识、(b) 讲解过程、(c) 分析过程、(d) 脉络、(e) 思想/洞见。重点抓两类问题：① 知识点在、但讲解/脉络/思想丢了（退化）；② Handbook 该章有价值内容被整段跳过（漏吸收）。
> **方法**：完整通读 HB ch1–10 源（`SLAM_Handbook_md/01–10`）→ 逐章提炼五维清单 → 教材落点 grep/read 核查（`P1_estimation/{slam_state_estimation,nonlinear_optimization,kalman_eskf}.tex`、`P2_slam/{dense_mapping,visual_odometry,camera_model,vio,imu_model,lidar_slam,point_cloud_processing,radar_slam,event_camera_slam}.tex`、`P5_frontier/learning_slam.tex`）。基线侦察/复核报告（`Handbook_recon_*`、`复核_Handbook_*`）作参考，但完备性独立判定。
> **约束**：只读，未改任何 .tex/refs.bib，只写本报告。

---

## 一句话总判

**ch1–10 的吸收完备度极高，五维（知识/讲解/分析/脉络/思想）整体到位，无 major 漏吸收、无 major 讲解退化。** 各章不仅搬全了知识点，更把 HB「把人讲懂」的展开路径、权衡分析、动机脉络与深层洞见用本书口吻重写并接进单一叙事线（因子图 MAP 母方程 + 右扰动 + `Λ` 信息矩阵）。仅余 **3 处 minor 漏点**（均为"叙事调味/单一公式形态"层面，不伤理解）与 **若干 minor 提示**，详见末节清单。

下表为逐章五维覆盖；✅=完整吸收（含讲解/脉络/思想）、⚠️=知识在但某维偏薄、❌=高价值整段漏吸收。

| HB 章 | (a)知识 | (b)讲解过程 | (c)分析过程 | (d)脉络 | (e)思想/洞见 | 主落点 |
|---|---|---|---|---|---|---|
| 01 因子图 | ✅ | ✅ | ✅ | ✅ | ✅ | `slam_state_estimation`§est-fg/elimination；`nlopt`§lm |
| 02 高级状态 | ✅ | ✅ | ✅ | ✅ | ✅ | `lie_theory`（机理）；`slam_state_estimation`§est-steam |
| 03 鲁棒 | ✅ | ✅ | ✅ | ✅ | ✅ | `nlopt`§nlopt-robust |
| 04 可微优化 | ✅ | ✅ | ✅ | ✅ | ✅ | `nlopt`§nlopt-diffopt |
| 05 稠密地图 | ✅ | ✅ | ✅ | ✅ | ✅ | `dense_mapping`（全章 + §dense-implicit） |
| 06 可证最优 | ✅ | ✅ | ✅ | ✅ | ✅ | `nlopt`§certifiable + `slam_state_estimation`§fim-laplacian |
| 07 视觉 SLAM | ✅ | ✅ | ✅ | ✅ | ✅ | `camera_model`+`visual_odometry`+`vio`+`learning_slam` |
| 08 激光 SLAM | ✅ | ✅ | ✅ | ✅ | ✅ | `lidar_slam`+`point_cloud_processing` |
| 09 雷达 SLAM | ✅ | ✅ | ✅ | ✅ | ✅ | `radar_slam`（专章 677 行） |
| 10 事件 SLAM | ✅ | ✅ | ✅ | ✅ | ✅ | `event_camera_slam`（专章） |

---

## 逐章五维覆盖详表

### HB Ch01 — Factor Graphs for SLAM ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 因子图定义、Bayes 网/MRF、MAP=NLS、白化、矩阵分解(Cholesky/QR/Householder)、稀疏雅可比/信息矩阵、变量消元、Bayes 树/iSAM2、Powell Dogleg、增益比 ρ | **✅ 全在**。`est-fg`/`thm:fg-map`/`tab:factors`/`est-fg-linear`(含 COLAMD、9399→4168 实测)/`thm:elimination`(消元=分解)/`est-smooth-filter`(Bayes 树/iSAM2)；Dogleg 在 `nlopt:468`(`Powell 狗腿：省一次分解`) |
| (b) 讲解过程 | 玩具例→大例可视化→MAP→NLS→线性化→矩阵分解→稀疏→消元逐步图解(消 ℓ₁ 5 步) | **✅**。`der:est-elim-numeric`(`一个可手算的标量消元例`)把"消元=分解"从断言落成可验算演示，逐式重写 HB Fig 1.10–1.12 的精神 |
| (c) 分析过程 | Cholesky vs QR(快 2×但更稳)、平滑 vs 滤波(稀疏 vs 稠密)、消元序决定 fill-in(NP-hard)、Dogleg vs LM(省分解) | **✅**。`nlopt` Cholesky/QR 对照；`est:1198` 平滑/滤波稀疏性分析；`ins:`消元顺序决定 fill-in、为何路标先消(连 Schur-in-BA 同根) |
| (d) 脉络 | √SAM 命名→稀疏→消元→Bayes 树→iSAM 增量 演进 | **✅**。完整复现，且接全书因子图主线 |
| (e) 思想/洞见 | "因子图=稀疏性的图形化身"、"Bayes 树=Cholesky 因子的图模型"、增量编辑=编辑 Bayes 树(只顶部受影响,两条性质)、SRIF/Mariner 10 历史 | **✅**。`est:1201` Bayes 树增量编辑+两条性质+受影响子树+orphan 全有；`est:1198` 收了 SRIF/Mariner 10(1969)历史注记(本侦察列为可选,已吸收) |

**附加价值**：GBP(高斯置信传播)作"精确串行消元 vs 近似并行消息传递"对照(`est:1220`),是 HB Ch18 的提前兑现,超出 Ch1 本身。

### HB Ch02 — Advanced State Variable Representations ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | SO(d)/SE(d)、exp/log、⊕/⊖、伴随、左/右雅可比、李群上高斯、⊙ 齐次点算子；连续时间(样条+GP)、累积样条、李群累积式、GP-on-Lie、块三对角 K⁻¹ | **✅**。李群机理在 `lie_theory`(整章,含 `subsec:odot-algebra`);连续时间在 `est-steam`,样条在 `est-steam-spline`(`另一条路：参数样条与 GP 的取舍`) |
| (b) 讲解过程 | 流形优化为何/怎么、PnP 例线性化、参数→非参数动机、核技巧、SDE→稀疏 K⁻¹ | **✅**。`est-steam` 完整落地 GP 主线;样条节给参数样条→累积形式→李群累积式逐步推 |
| (c) 分析过程 | 样条 vs GP(过拟合/正则)、参数 vs 非参数权衡、左扰动 vs 右扰动 | **✅**。`ins:`样条 vs GP 唯一本质区别是"有没有运动先验因子";HB 左扰动显式改写为本书右扰动经伴随等价 |
| (d) 脉络 | "连续时间有两条路"→GP 主线→样条补全 | **✅**。`est:1322` 明开"两条路",`est:1535` 补全样条一支,悬念回收 |
| (e) 思想/洞见 | "测量/估计/查询三时刻解耦"、GP inducing points 减控制点、WNOA 退化为三次 Hermite | **✅**。三时刻解耦(`est:1322`)、`est:1488` WNOA→三次 Hermite 插值的先验层面解释 |

### HB Ch03 — Robustness to Outliers ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 外点两类成因、前端剔除(RANSAC/共识最大化/最小集/Nister 五点)、PCM/一致性图/最大团、M-估计五核+截断二次+最大共识、IRLS、B-R 对偶+外点过程、GNC | **✅**。`nlopt-robust` 全有:`nlopt-robust-frontend`(RANSAC 共识最大化视角 + PCM 最大团);`tab:robust` 含截断二次/最大共识;`thm:nlopt-br` B-R 对偶;GNC |
| (b) 讲解过程 | 为何有外点(两例)→前/后端二分→RANSAC 两洞见→PCM 一致性函数→图论→GNC 三步 | **✅**。`nlopt:942` 起"外点从哪来"+两道防线;RANSAC 两洞见、PCM 两例(3D-3D/位姿图回环)逐步 |
| (c) 分析过程 | RANSAC 何时失效(n=10,w=0.1→10¹⁰)、PCM vs RANSAC 权衡、GNC 失败案例、前端硬剔+后端软降权互补 | **✅**。`nlopt:973` 失效定量;`nlopt:1001` PCM↔RANSAC 权衡;`nlopt:1155` M3500/SubT/Victoria Park 三诚实结论(无骨架时 GNC 崩、PCM+GNC 最稳、M3500 全崩) |
| (d) 脉络 | 共识最大化↔截断二次↔最大共识↔RANSAC 桥;鲁棒=逆 Wishart MAP 即 B-R 特例 | **✅**。桥闭环;Cauchy 逆 Wishart 证明点明为 B-R 对偶实例 |
| (e) 思想/洞见 | "影响函数"、Tukey 在软降权与硬剔除之间、动态协方差缩放=G-M IRLS、certifiable 用 SDP+证书 | **✅**。Tukey 重下降型定位(`nlopt:1040`);certifiable 接 §certifiable |

### HB Ch04 — Differentiable Optimization ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | BLO 框架、展开微分(reverse/forward/截断/一步+有限差分)、隐式微分(最优性条件全微分)、HVP、FastTriggs、库(Theseus/PyPose/CvxpyLayer)、系统(BA-Net/DROID/DeepFactors/iSLAM) | **✅**。`nlopt-diffopt`(`可微优化：把后端嵌进端到端学习`)全有:`eq:nlopt-blo`、展开/隐式微分、`eq:nlopt-hvp`、4TB 海森例、库与系统 |
| (b) 讲解过程 | 单向→双向信息流动机、隐函数求导类比、HVP="梯度·向量积"的梯度、两例(学习特征+BA、学习前端+PGO) | **✅**。`nlopt:1589` 用 x+y+5=0 类比隐函数求导;`nlopt:1618` HVP 自包含落成可执行算法 |
| (c) 分析过程 | 展开 vs 隐式取舍(内存/路径无关/收敛要求)、忽略间接项的近似误差 | **✅**。`nlopt:1627` 两途径取舍表 + 直接梯度近似误差界 |
| (d) 脉络 | 接 GVI(升级输出)→本节(升级接口)→接协方差/IEKF 的二阶结构 | **✅**。`nlopt:1561` 明示与 GVI/协方差/IEKF 的承接关系 |
| (e) 思想/洞见 | "BA 当一层只单向反传 vs 双层让前端学几何"、流形上扰动须右扰动 | **✅**。`nlopt:1908` 误解表点破单/双向之别;流形右扰动 pitfall |

### HB Ch05 — Dense Map Representations ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 占据/TSDF/surfel/mesh/点云、ESDF vs 投影 SDF、四象限、数据结构(数组/hash/voxel-block/octree/VDB/kD)、整合器(ray-trace vs projection)、可扩展性(多分辨率/wavemap)、GPOM/GPIS/Hilbert、神经隐式 | **✅**。`dense_mapping` 全有:`tab:dense-quadrant` 四象限;`dense-implicit`(`隐式曲面、ESDF 与连续函数地图`)含 ESDF/voxblox/FIESTA/brushfire、GPOM/GPIS/Hilbert 预测式;`dense:992` 数据结构;`dense:1000` 整合器+wavemap |
| (b) 讲解过程 | 隐式曲面定义(零交叉/符号)、显隐互转机制、GP 回归当建图、KinectFusion 闭环 | **✅**。`dense:1145` 隐式曲面;`dense:1153` 互转机制;`dense:1164` GP 当建图(承 `est-steam` 时间→空间) |
| (c) 分析过程 | 占据 vs 距离场(直接性/光滑性)、显式 vs 隐式(沿表面 vs Cartesian)、投影 SDF 过估、薄物体擦除几何、ray vs projection 竞态 | **✅**。`dense:982` 投影 SDF 过估+ESDF;`dense:989` 薄物体擦除;`dense:1000` 竞态/GPU 友好对照 |
| (d) 脉络 | 四象限骨架贯穿全章、每种表示先定位;经典→可微渲染(ch14)桥 | **✅**。四象限在 `dense:131,147,948,1139` 等反复引用,真"一个框架贯穿到底";神经隐式 SDF→`dense-radiance`(NeRF/3DGS) |
| (e) 思想/洞见 | "重建看零交叉、规划看 ESDF 梯度"、TSDF clamping 与 OctoMap 同精神、GP 连续概率表示给不确定度可外插 | **✅**。`dense:983` 一句洞见;GP 与贝叶斯精神同源 |

### HB Ch06 — Certifiably Optimal Solvers ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | Shor 松弛(QCQP→SDP、秩-1 lifting、次优界)、SE-Sync(PGO MLE/Langevin、旋转 QCQP、精确性定理、Burer-Monteiro/Stiefel/Riemannian Staircase、舍入/证书)、地标扩展+Schur、测距辅助/CORA、各向异性/外点→矩(Lasserre)松弛、退化 SDP;FIM↔图拉普拉斯、D/A/E-最优、Kirchhoff 矩阵树、Fiedler、信息永不损害 | **✅**。求解器全在 `nlopt`§certifiable(`可证最优`):`Shor 松弛`/`thm:nlopt-sesync-exact`/`thm:nlopt-staircase`/`扩展：地标、测距、各向异性与外点`;精度极限在 `slam_state_estimation`§fim-laplacian(`信息矩阵就是图的拉普拉斯`):`thm:fim-laplacian`/D-A-E/矩阵树/Fiedler |
| (b) 讲解过程 | Shor 三步代数、PGO 三阶段(化简→松弛→Staircase)、矩矩阵+冗余约束、FIM=L_w⊗I₃ 全推导 | **✅**。`nlopt:1659` Shor 三步;`nlopt:1686` PGO 三步;`est:1277` FIM=拉普拉斯⊗I₃ 逐式 |
| (c) 分析过程 | "提升"代价(维度涨)、精确性条件(小噪声)、Staircase 为何有限终止、测距松弛不一概精确、矩松弛退化致 Staircase 失效、闭大环 vs 小环 | **✅**。`nlopt:1674` 提升代价;`nlopt:1753` 测距连通度;`nlopt:1756` 退化 SDP 致 Staircase 失效;`est:1299` 闭大环更增信息 |
| (d) 脉络 | 兑现 §nlopt-why/§nlopt-robust 埋的 SE-Sync 钩子;est↔nlopt 分工(精度极限 vs 求解器) | **✅**。`nlopt:1642` 明文回收钩子;`est:1303` 明分工 |
| (e) 思想/洞见 | "给不出证书本身就是预警"、"信息矩阵=可从图读出的量"、D-最优⇔生成树最多⇔连得最牢、主动 SLAM/测量剪枝用拉普拉斯谱代理 | **✅**。`nlopt:1646` 证书即预警;`est:1255` 信息矩阵升级为图量;`est:1299` 矩阵树精确化"连通度决定精度" |

### HB Ch07 — Visual SLAM ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 摄影测量/BA/SfM/VO/VSLAM 史与术语、相机模型谱系(针孔/rad-tan/BC/KB/DS)、光度模型/卷帘、关键点(Harris/SIFT/SURF/FAST/ORB/BRISK + 学习型 SuperPoint/LIFT/D2-Net/HF-Net)、重投影误差、四种优化法、直接法/光度误差(LSD/DSO)、协视/本质图、Schur/Power BA/VarPro、PGBA、变分稠密、IMU 减 gauge、SuperGlue/MASt3R | **✅**。`camera_model`§cam-modern(相机谱系完整本土化);`visual_odometry`(特征/对极/三角化/PnP/光流/直接法/PTAM/Schur/Power BA/PGBA);学习型特征/匹配集中在 `learning_slam`(SuperPoint/SuperGlue/LightGlue/LoFTR/MASt3R,**有意去重不两处展开**) |
| (b) 讲解过程 | 史的源流、双球为何快、describe-and-detect 反转、滤波毁稀疏可视化、Power BA 幂级数 | **✅**。`vo:101` 史;`vo:1610` 学习特征反转;`vo:1548` 四法 + EKF 毁稀疏;`vo:1587` Power BA 幂级数自包含 |
| (c) 分析过程 | 特征 vs 直接(收敛域/数据关联)、滤波 vs 关键帧(稀疏)、LSD(交替) vs DSO(联合)、局部 BA 稠密 vs 完整 BA 稀疏 | **✅**。`vo:1314` 特征 vs 直接本质差;`vo:1505` 两类前端;`vo:1584` 局部/完整 BA 稀疏度 |
| (d) 脉络 | PTAM→ORB-SLAM→LSD/DSO→VI/学习 里程碑链 | **✅**。`vo:109` 里程碑链;学习时代→`ch:learning` |
| (e) 思想/洞见 | "能跟踪和是角点是同一件事两面"、辐照度恒定替代亮度恒定、逆深度三大理由、几何被学习包裹而非取代 | **✅**。`vo:122` 角点↔可跟踪;`vo:1458` 辐照度恒定;`vo:1462` 逆深度三理由;`learning:133` 几何被包裹 |

**说明**：HB §7.6 变分单目稠密(TV 正则/soap-film)在教材以 REMODE 的"深度不确定度加权总变分(Huber)正则"形式吸收(`dense:651`)——思想(TV 正则致平滑/保边)已到位,唯未单列 HB (7.10)(7.11) 的纯变分泛函形态(见 minor m1)。

### HB Ch08 — LiDAR SLAM ✅✅✅✅✅

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | LiDAR 类型/测距(ToF/AMCW/FMCW/Risley)、ICP 距离度量/对应搜索、LOAM 特征、去畸变四法、Point-LIO/CT-ICP/KISS-ICP、scan-to-scan/map、位置识别(ScanContext/RING++/OverlapNet/SegMap/BTC、局部/全局/BEV/range image)、PGO/鲁棒/锚节点/多会话、多机器人/MAGIC→SubT/异构识别 | **✅**。`lidar_slam`+`point_cloud_processing` 全有;测距三机制(`par:lidar-ranging`);CT-ICP/Point-LIO 两支线(`par:lidar-ct-pointwise`);MAGIC→SubT+异构(`lidar:1150` 子节) |
| (b) 讲解过程 | ICP 两核心设计(距离度量/对应搜索)、运动畸变成因、特征曲率、scan-to-map 为何更准 | **✅**。`lidar` ICP 三距离度量;去畸变四法表(含逐点更新);FAST-LIO 逐点反向传播补偿 |
| (c) 分析过程 | 特征法 vs 直接逐点(丢孤立点/调参)、四去畸变法适用边界、集中 vs 去中心 vs 分布式、稀疏/结构混淆挑战 | **✅**。`tab:lidar-deskew` 四法权衡;`lidar:1154` 半去中心式 + 通信瓶颈 |
| (d) 脉络 | Lu-Milios→LOAM→FAST-LIO 主线 + CT-ICP/Point-LIO 两并行支线 | **✅**。`lidar:1209` 主脉络 + 两支线并列点出 |
| (e) 思想/洞见 | "单机单会话成熟→多机协同→跨模态→终身演化"三方向、KISS-ICP 极简也能稳健泛化、后端语言始终是 PGO+鲁棒+位置识别 | **✅**。`lidar:1157` 三方向脉络;`lidar:1209` KISS-ICP"少即是多" |

### HB Ch09 — Radar SLAM ✅✅✅✅✅（全新专章）

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 旋转 vs SoC 雷达、RCS、FMCW 测距/锯齿测速/三角解耦/相控阵测角、雷达噪声(斑点/多路径/饱和)、CFAR/BFAR/k-最强、多普勒里程计/直接(相位相关/Masking-by-Moving)/特征(FSCD/BASD)/配准(CFEAR/APDGICP)、位置识别(雷达化 ScanContext/RING/M2DP)、系统(TBV/RadarSLAM/4DRadarSLAM/4D iRIOM)、地图(landmark/occupancy/heatmap/NDT/神经场/PHD)、多模态(雷达-激光/相机/热像/OSM/RSL-Net/UWB)、数据集 | **✅**。`radar_slam`(677 行)全有,FMCW/多普勒/测角自包含推导;配准复用 `ch:pointcloud`;位置识别迁移 `sec:lidar-place` |
| (b) 讲解过程 | FMCW IF 信号→距离/速度、单帧 ego-velocity 线性最小二乘、多普勒因子进图 | **✅**。`radar:318` 单帧本体速度逐步;多普勒残差+信息矩阵进因子图 |
| (c) 分析过程 | 多普勒不需对应 vs 配准难、PCM/CFEAR 多关键帧抗稀疏、三角 vs 锯齿调制取舍、窄 FoV 横向速度不可观 | **✅**。`radar:117` 速度从配角变主心骨;`radar:235` 三角 vs 锯齿;`radar:348` 窄扇区退化 |
| (d) 脉络 | 承接 `sec:lidar-rae` RAE→多普勒第四维引出;与激光同构(测/滤波/里程计/识别/SLAM) | **✅**。`radar:104` RAE 同义 + 第四维;`radar:45` 与激光同构主线 |
| (e) 思想/洞见 | "多一维径向速度、少一维高度"、**单雷达角速度不可观的秩论证**(视线方向须张成 R³)、雷达穿透性改造占据更新 | **✅**。`radar:117` 根本差异;`radar:348` 不可观秩论证(列满秩 3);`radar:499` 穿透性占据更新 |

### HB Ch10 — Event-based SLAM ✅✅✅✅✅（全新专章）

| 维度 | 源书要点 | 教材落点与判定 |
|---|---|---|
| (a) 知识 | 事件相机原理/事件生成模型 ΔL=pC、优势(HDR/低延迟/无模糊)、生物启发、器件(DVS/DAVIS/ATIS)、表示(帧/SAE/体素/IWE)、方法分类(逐事件/批、模型/学习、间接/直接、几何/光度)、CMax/聚焦框架、后端(间接/直接、起步阶段)、系统(EVO/ESVO/USLAM/EDS/CMax-SLAM/DEVO)、模拟器(ESIM/Vid2E/V2E)、数据集(ECDS/MVSEC/DSEC)、指标(ATE/RPE) | **✅**。`event_camera_slam`(专章)全有;`der:`事件生成模型;CMax 全推导;表示/分类表;系统/模拟器/数据集综述 |
| (b) 讲解过程 | 事件何时触发、CMax 几何直觉(运动对→锐利、错→散开)、IWE 方差度量、弯曲映射雅可比 | **✅**。`event:135` 生成模型;`event:223` CMax 锐利度直觉;`thm:cmax-jac` 雅可比结构(右扰动) |
| (c) 分析过程 | 逐事件 vs 批(延迟/信息)、间接 vs 直接(成熟几何 vs 满信息非凸)、各表示牺牲什么、单目事件尺度不可观 | **✅**。`tab:event-taxonomy` 四轴取舍;`event:191` 各表示牺牲;`event:370` 尺度不可观→融 IMU |
| (d) 脉络 | 复杂度沿运动维度/场景/传感器递增、模型法为主学习渐起;CMax 内蕴连续时间→接 STEAM | **✅**。`event:390` 探索期脉络;`event:360` CMax→连续时间 BA(CMax-SLAM)接 `est-steam` |
| (e) 思想/洞见 | "CMax=为只有微秒事件、无灰度的相机量身定做的直接法"(对齐量/失配度/时间模型三差别)、神经形态终极愿景、IWE 把运动估计嵌进表示构造 | **✅**。`ins:`CMax↔直接法同一思想两副面孔(`event:326`);`event:414` 神经形态愿景;`event:198` IWE 嵌运动估计 |

---

## ❌ 高价值缺失清单（重点）

**无。** 通读 HB ch1–10 五维清单逐条比对教材落点,未发现任何"HB 该章有价值内容被整段跳过"的 major 漏吸收。包括最易被略过的项也已落地:
- ch1 的 SRIF/Mariner 10 历史注记(侦察曾标"可选调味")——已收(`est:1198`);
- ch1 的可手算消元数值例 + Bayes 树增量编辑两性质 + 受影响子树/orphan——已收;
- ch2 的样条全套(累积形式 + 李群累积式 + 控制点扰动一阶关系) + GP-on-Lie + 三时刻解耦——已收;
- ch6 的测距辅助/CORA(单位向量凑 QCQP) + 各向异性/外点矩松弛 + 退化 SDP 难题——已收;
- ch6 的 D/A/E-最优 + Kirchhoff 矩阵树 + 主动 SLAM/测量剪枝——已收;
- ch9 的单雷达角速度不可观秩论证、热图/PHD/神经场地图、RSL-Net/OSM/UWB——已收;
- ch10 的事件表示牺牲了什么、后端起步现状、神经形态愿景——已收。

---

## ⚠️ 讲解/脉络/思想退化清单（重点）

**无 major 退化。** 所有章不仅知识点在,讲解路径、权衡分析、动机脉络、深层洞见均以本书口吻传达到位,且普遍**强于**一般教材吸收——多处把 HB 的零散论述提炼成显式 `insight`/`pitfall`(如"消元顺序决定 fill-in""CMax↔直接法两副面孔""证书即预警""多一维速度少一维高度"),并接进单一叙事线。下列 minor 仅为"单一公式形态/叙事调味"层面的可选增厚,**不构成理解层面的退化**：

### m1（minor，单一公式形态）— HB §7.6 纯变分单目稠密泛函未单列
**位置**：`dense_mapping.tex`(单目稠密节)
**现状**：HB (7.10)(7.11) 的"数据项 + TV 正则 + soap-film fill-in"变分泛函,教材以 REMODE 的"深度不确定度加权总变分(Huber)正则"形式吸收(`dense:651`)——**思想(TV 正则致平滑保边、填补未观测)已到位**,且 REMODE 是更现代的概率化实例。唯未把 HB 那条"连续 h:Ω→ℝ 上最小化 ∫ρ+λ∫|∇h|"的纯变分泛函单列为一段。
**判定**：思想未退化(TV 正则的动机与效果都讲了),仅"变分法作为一条独立单目稠密路线"的泛函形态未显式写出。属可选增厚,非退化。

### m2（minor，措辞）— vo:770 残留一处 narration_dependence
**位置**：`visual_odometry.tex:770`
**问题**：把本书自推的标准式(`eq:vo-pnp-ba`)说成"这正是 Handbook 的 pose-only BA / PTAM 跟踪线程所解的式子",且无 `\cite`。
**判定**：复核_Handbook_感知weave.md 已记此条(其 m6);属独立性措辞残留,非完备性问题,顺带登记。建议去掉"Handbook 的"所属化措辞(pose-only BA 已是通用术语)。

### m3（minor，跨章一致性）— 若干"取自/摘自…逐式复现"旧措辞
**位置**：`slam_state_estimation.tex:163`(`sec:est-gauss`,"所有结论摘自 Barfoot 第 1 章…完整复现")等
**问题**：关于 Barfoot(非 HB)、且在既有节;属全书独立性待清理项,与 HB ch1–10 完备性无直接关系。
**判定**：顺带登记,不影响 HB 吸收完备性。已在十四讲吸收审计的"全书独立性待清理"范围内。

---

## 附：交叉确认（与基线侦察/复核一致性）

- 本审独立判定的"无 major 漏吸收/无 major 退化"与 `复核_Handbook_估计优化weave.md`(blocker 0/major 0)、`复核_Handbook_新传感器.md`(blocker 0,2 major 均为内容内的数学/缝合而非漏吸收)、`复核_Handbook_感知weave.md`(1 blocker B1 为 PGBA 引用键错配 √BA 论文——属文献题录错,非吸收完备性)总体一致。
- 复核_Handbook_感知weave.md 报告的 B1(PGBA 引用键 `demmel2021pgba` 指向 √BA 论文)、M1(测距物理三机制跨 lidar/point_cloud 近重复)、M2/M3(标准预积分/LOAM 两章重复)属**引用题录 / 跨章去重**问题,**不影响 ch1–10 内容是否被完整吸收**;本完备性审不重复处置,仅指明它们与"漏吸收/退化"两类完备性问题正交。
- 三份 recon 报告(A/B/C)规划的所有"真增量"(ch1 Dogleg/消元例、ch2 样条/GP-on-Lie、ch4 全章、ch6 全章、ch5 四象限/ESDF/GP-Hilbert/数据结构、ch7 学习特征/Power BA/PGBA、ch8 CT-ICP/Point-LIO/多机器人、ch9/ch10 全章)经本审 grep/read 核查**已全部落地**。

---

## 结论

The SLAM Handbook 第 1–10 章在《机器人学笔记》中的吸收**完备度极高**：知识、讲解过程、分析过程、脉络、思想/洞见五维整体到位,**无 high-value 缺失、无讲解/脉络/思想退化**。仅余 3 处 minor(1 处纯变分泛函形态可选增厚、1 处独立性措辞残留、1 处旧节单书措辞),均不伤理解。本审视为"完整吸收"目标在 ch1–10 上**已达成**。
