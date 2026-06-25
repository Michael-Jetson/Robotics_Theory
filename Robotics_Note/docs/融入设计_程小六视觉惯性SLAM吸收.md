# 融入设计：程小六《视觉惯性 SLAM：理论与源码解析》精华吸收

> 本文是**映射设计稿**，只定「取什么 / 落哪里 / 要不要新章 / refs 键 / 分工」，**不写正文**。
> 作者：程小六（电子工业出版社，2023-01，ISBN 978-7-121-44812-6，约 29 印张 / 64.7 万字，20 章）。
> GitHub 配套：`electech6/ORB_SLAM2_detailed_comments`、`electech6/ORB_SLAM3_detailed_comments`。

---

## 0. 总评与核心纪律

这本书是 **ORB-SLAM2 + ORB-SLAM3 的逐行源码导读**，定位是「带初学者跑通第一个 SLAM 工程」。全书结构：

- **第一部分（Ch1–6，SLAM 基础）**：SLAM 概览、C++ / CMake、齐次坐标与旋转表达、针孔成像与畸变、对极几何、g2o。→ **本书（《机器人学笔记》）已全覆盖且更深**（十四讲 + Barfoot + Handbook 已消化），**全略**。
- **第二部分（Ch7–14，ORB-SLAM2）**：ORB 特征、特征匹配、地图点/关键帧/图结构、地图初始化、跟踪线程、局部建图线程、闭环线程、优化方法。→ **混合**：方法/架构精华可取，逐行源码解析略。
- **第三部分（Ch15–20，ORB-SLAM3）**：IMU 预积分、多地图系统、跟踪线程、局部建图线程（含 IMU 初始化）、闭环+地图融合、视觉 SLAM 现在与未来。→ **本书吸收重心**，但**预积分一章已被 `vio.tex` 完全覆盖，须避免重复**。

**纪律落实——本设计的三条硬约束：**

1. **不搬逐行源码。** 此书约 60% 篇幅是 `Tracking.cc / LocalMapping.cc / LoopClosing.cc / Atlas.cc` 的中文注释式贴码。本书是「教材」，只取**方法、架构图、判定准则、几何/优化推导**，代码最多保留**伪码或 1–2 行点睛**。
2. **已被 `vio.tex` 覆盖的不重吸。** 经核查（见 §3），`vio.tex` 已含 **Forster + VINS 双形式 IMU 预积分全推导**（ΔR/Δv/Δp、噪声传播、零偏一阶雅可比、残差）+ **VIO 三段式初始化**（视觉 SfM → 陀螺零偏 → 速度/重力/尺度线性解 → 重力精化）。因此本书 **Ch15 整章 + Ch18 的通用初始化理论 = 已有**，**只取 ORB-SLAM3 特有的「MAP 三阶段在局部建图线程里的工程编排」差异**。
3. **有机结合，落到现有章。** 绝大多数精华挂到 `slam_system.tex`（ORB-SLAM2 架构）与 `vio.tex`（ORB-SLAM3 惯性增量），**新增一章只在「ORB-SLAM3 系统级整合」确有独立价值时设**——结论见 §2.4：**建议新增一节群而非整章**，挂在 `slam_system.tex` 末尾的 ORB-SLAM 对照节之后，或视体量升格为一章 `orbslam3_architecture.tex`。

**一句话结论：取约「3 主题 + 6 个 ORB-SLAM2 方法点」，主要落 `slam_system.tex` 与 `vio.tex`，新增 1 节群（《ORB-SLAM3 系统剖析》）可选升格为章；预积分零吸收。**

---

## 1. 全书目录 + 每章「源码解析(略) vs 方法理论(吸)」标注

### 第一部分 SLAM 基础（Ch1–6）—— 整体「已覆盖，全略」

| 章 | 标题 | 判定 | 理由 |
|---|---|---|---|
| 1 | SLAM 概览 | 略 | 入门概念，本书 SLAM 卷已覆盖 |
| 2 | 编程及编译工具（C++11 / CMake） | 略 | 工程预备，与教材定位无关 |
| 3 | SLAM 常用数学基础（齐次坐标 / 旋转表达 / Eigen） | 略 | 李群李代数本书远更深（Barfoot ch6-7） |
| 4 | 相机成像模型（针孔 / 畸变） | 略 | `camera_model.tex` / `camera_calibration.tex` 已覆盖 |
| 5 | 对极几何 | 略 | `visual_odometry.tex` §对极几何 已含全推导 |
| 6 | 图优化库 g2o | 略 | 本书有专门优化章（`ch:nlopt`），且不搬库 API |

### 第二部分 ORB-SLAM2 理论与实践（Ch7–14）—— 「方法吸，源码略」

| 章 | 标题 | 判定 | 取什么（精华） | 已有覆盖 |
|---|---|---|---|---|
| 7 | ORB 特征提取 | **半吸** | **四叉树特征点均匀化分布**（7.2.3）、图像金字塔分层配额（7.2.2）——这是 ORB-SLAM 独有、本书**缺**的工程点 | `sec:vo-orb-kp` 已讲 Oriented FAST + BRIEF，但**无四叉树均匀化** |
| 8 | ORB-SLAM2 中的特征匹配 | **少量吸** | 四类匹配的**适用场景对比**（单目初始化匹配 / 词袋匹配 / 投影匹配 / Sim(3) 投影匹配）；方向一致性检验思想 | 投影匹配/词袋本书有，但**缺 Sim(3) 相互投影匹配**与四者对照 |
| 9 | 地图点、关键帧、图结构 | **吸（重点）** | **共视图 / 本质图 / 生成树**三大数据结构定义；关键帧选取与**剔除准则**；地图点**剔除准则**（创建后观测比、可观测金字塔尺度）；地图点融合 | 本书仅启发式（inlier 计数选关键帧、零观测删点），**缺图结构与 ORB-SLAM 剔除逻辑** |
| 10 | ORB-SLAM2 中的地图初始化 | 略（已覆盖） | 单目 F/H 模型选择、双目初始化 | `sec:vo-epipolar`+`sec:vo-homography`+`sec:sys-init` **已全覆盖** |
| 11 | ORB-SLAM2 中的跟踪线程 | **少量吸** | 三段式跟踪的**判定准则**（参考关键帧/恒速模型/重定位）、倒排索引加速重定位、**EPnP 求位姿**（11.3.4，选学）、局部地图跟踪的局部关键帧/局部点定义 | 恒速模型+pose-only BA 已有；**缺 EPnP 细节、倒排索引、局部地图二级共视展开** |
| 12 | ORB-SLAM2 中的局部建图线程 | **吸（流程）** | 局部建图五步流程（处理新关键帧→剔除点→生成点→融合→剔除关键帧）；冗余关键帧剔除 | `sec:sys-backend` 有局部 BA/滑窗，但**缺 ORB-SLAM 风格的关键帧/点维护流水线** |
| 13 | ORB-SLAM2 中的闭环线程 | **吸（重点）** | **Sim(3) 变换计算与原理推导**（13.3，含 *7-DOF 推导）；闭环候选验证（时间一致性）；**Sim(3) 闭环矫正**（位姿传播 + 地图点传播）；闭环全局 BA | 本书 `loop_closure.tex` 有 DBoW2 四阶段流水线 + 几何验证，但 **Sim(3) 仅一句带过**、**无闭环矫正** |
| 14 | ORB-SLAM2 中的优化方法 | **吸（提纲）** | 四类优化的**角色划分**：跟踪线程仅优化位姿 / 局部建图局部 BA / 闭环 Sim(3) 位姿优化 / 闭环本质图优化 + 全局 BA | 本书优化章有 BA/位姿图，但**缺「四线程各跑什么优化」的 ORB-SLAM 全景对照** |

### 第三部分 ORB-SLAM3 理论与实践（Ch15–20）—— 「本书吸收重心」

| 章 | 标题 | 判定 | 取什么 | 已有覆盖 |
|---|---|---|---|---|
| 15 | ORB-SLAM3 中的 IMU 预积分 | **几乎全略（已覆盖）** | 仅可校核：紧/松耦合互补性的「5 点论证」措辞可参考 | ⚠ **`vio.tex` `sec:vio-preint` 已含全套**：基础公式(Exp/Log/右雅可比/伴随)、IMU 模型与运动积分、为何预积分、噪声分离、噪声递推协方差、零偏更新一阶修正、残差雅可比，**Forster + VINS 双形式**。**零吸收** |
| 16 | ORB-SLAM3 中的多地图系统（**Atlas**） | **吸（主题①·重点）** | **地图集 Atlas 概念**（活跃/非活跃地图、共享 DBoW2 关键帧库）；多地图量化收益（表 16-1/16-2，ATE+覆盖率，vs VINS-Mono/ORB-SLAM2）；**宽基线匹配** vs 窄基线（地图融合的关键使能）；何时创建新地图（时间戳颠倒/跳变、跟踪丢失「断臂求生」）；地图融合 5 步概述 | **本书完全无**多地图/Atlas |
| 17 | ORB-SLAM3 中的跟踪线程 | **少量吸（差异）** | 新跟踪状态 `RECENTLY_LOST`（短期丢失用 IMU 预测位姿找回 vs 长期丢失新建地图）；IMU 模式下「降低成功跟踪门槛」；恒速模型用 **IMU 积分代替位姿差**；重定位 **EPnP→MLPnP**（解耦相机模型、≥6 点） | 本书跟踪是纯视觉，**缺 IMU 模式状态机与 MLPnP** |
| 18 | ORB-SLAM3 中的局部建图线程（含 **IMU 初始化**） | **吸（主题②·重点，但只取工程编排）** | **MAP-based IMU 三阶段初始化在局部建图子线程的编排**：①纯视觉最大后验→②纯惯性最大后验（固定轨迹、尺度 s + 零偏 b + 重力方向 R_wg + 速度，因子图 18-2）→③视觉惯性联合最大后验→④（单目）尺度/重力精化（10s/100KF/75s 收尾）；**不会阻塞跟踪实时性** 的设计；阶段标志 `mbImuInitialized/BA1/BA2` | ⚠ **`sec:vio-init` 已含等价的「松→紧」三段式初始化理论**（视觉 SfM→陀螺零偏→速度/重力/尺度线性解→重力精化）。**只吸 ORB-SLAM3 把它放进「局部建图线程、不阻塞跟踪、用因子图分四阶段递进精化」的系统级差异**，理论本体不重吸 |
| 19 | ORB-SLAM3 中的闭环及地图融合线程 | **吸（主题③·重点）** | **检测共同区域**（vs ORB-SLAM2 闭环候选：先几何一致性后时序一致性的「集卡式」验证 → 召回率↑）；闭环 vs 地图融合的**分流逻辑**（活跃地图内→闭环；跨地图→融合，同检则优先融合）；**Sim(3)/SE(3) 求位姿 + 引导匹配**；**纯视觉地图融合**（局部窗口、Sim(3) 位姿传播矫正、生成树融合的父子换向、熔接 Welding BA、本质图、全局 BA）；**视觉惯性地图融合**（MergeLocal2，固定尺度=1、IMU 快速优化、视觉惯性熔接 BA 因子图 19-9）；线程间配合（融合时停局部建图/全局 BA） | **本书完全无**地图融合；闭环矫正本身也只在 ORB-SLAM2 Ch13 取 |
| 20 | 视觉 SLAM 的现在与未来 | **半吸（综述价值）** | **视觉 SLAM 发展三阶段史**（早期 EKF→PTAM/直接法快速期→视觉惯性成熟期）；**主流框架对比表 20-1**（MSCKF/ROVIO/OKVIS/ICE-BA/Kimera/BASALT/VINS-Fusion/ORB-SLAM3 × 前端/后端/闭环/耦合/多地图/传感器）；**数据集对比表 20-2**（EuRoC/TUM-VI/Zurich Urban MAV/Canoe/PennCOSYVIO）；未来趋势（与深度学习结合 / 动态环境 / 软硬一体 / 多智能体协作） | 本书各章散见前沿，但**缺一张统一的 VI-SLAM 框架横向对比表 + 标准数据集表**——可作为 `vio.tex` `sec:vio-frontier` 或综述章的收尾增强 |

---

## 2. 取哪些精华 → 落点设计（节级要点）

> 原则：**同一 .tex 内插入新节**优先于新章；只有「ORB-SLAM3 系统级整合（Atlas+融合+IMU 编排）」自成一体且体量大，才考虑升格。

### 2.A 落 `parts/P2_slam/visual_odometry.tex`（`ch:vo`）——ORB 特征工程

**新增 1 子节**，挂在 `sec:vo-orb-kp`（Oriented FAST）之后：

- `\subsection{ORB 特征点的均匀化分布：四叉树法}`（建议 label `sec:vo-orb-quadtree`）
  - **要点**：为何要均匀化（特征扎堆→匹配/三角化退化、局部过约束）；图像金字塔逐层**配额分配**（按面积比例分特征数）；**四叉树递归剖分**直到节点数达配额，每节点保留响应最强者。
  - **形式**：一张四叉树剖分示意 + 配额公式；**不贴 `ComputeKeyPointsOctTree` 源码**，最多伪码 4–5 行。
  - 交叉引用 ORB-SLAM2 论文 `murartal2017orbslam2`。

### 2.B 落 `parts/P2_slam/slam_system.tex`（`ch:slam_system`）——ORB-SLAM2 架构精华

`slam_system.tex` 末尾已有 `sec:sys-orbslam`（与 ORB-SLAM2 架构对照）。**在其前后扩成一个「ORB-SLAM2 方法精读」节群**（4 个新节）：

- `\subsection{图结构：共视图、本质图与生成树}`（`sec:sys-graph`）
  - 共视图（边权=共视地图点数）、本质图（高权重共视边 + 生成树 + 闭环边，用于位姿图优化降复杂度）、生成树（关键帧父子关系，建图与融合时维护）。一张三图对照示意。落 ORB-SLAM2 Ch9。
- `\subsection{关键帧与地图点的剔除准则}`（`sec:sys-culling`）
  - 关键帧剔除：冗余判定（90% 地图点被其它≥3 关键帧观测则删）；地图点剔除：创建后观测比阈值、可见帧/匹配帧比。落 ORB-SLAM2 Ch9 + Ch12。补本书当前「启发式删点」之不足。
- `\subsection{三段式跟踪与重定位（含 EPnP）}`（`sec:sys-tracking-orb`）
  - 参考关键帧跟踪 / 恒速模型跟踪 / 重定位跟踪三者**触发条件与回退链**；倒排索引（Inverted Index）加速重定位候选检索；**EPnP** 求位姿原理（4 控制点线性化，≥4 点）作为选学盒子。落 ORB-SLAM2 Ch11。
- `\subsection{局部建图流水线与四线程优化全景}`（`sec:sys-localmapping-orb`）
  - 局部建图五步（处理新 KF→剔除点→三角化生成点→相邻融合→剔除冗余 KF）；**「四线程各跑什么优化」总表**：跟踪=仅位姿 BA / 局部建图=局部 BA / 闭环=Sim(3) 位姿优化 + 本质图优化 / 全局=全局 BA。落 ORB-SLAM2 Ch12+Ch14。

**另在 `loop_closure.tex`（`ch:loop`）补 Sim(3) 闭环**（与几何验证 `sec:loop-stageD` 衔接）：

- `\subsection{单目闭环的 Sim(3) 计算与矫正}`（`sec:loop-sim3`）
  - 为何单目闭环要 Sim(3)（7-DOF，含尺度，吸收累计尺度漂移）；Sim(3) 求解（候选匹配→RANSAC→引导匹配优化）；**闭环矫正**（沿生成树/共视图传播 Sim(3) 位姿 + 地图点坐标传播）；闭环本质图优化 + 全局 BA。落 ORB-SLAM2 Ch13。补本书「Sim(3) 一句带过」之不足。
  - 与本书已有 `\cref{sec:loop-backend}`（位姿图后端接口）自然衔接。

### 2.C 落 `parts/P2_slam/vio.tex`（`ch:vio`）——ORB-SLAM3 惯性增量

`vio.tex` 已是「VIO 教材主章」（含预积分 + 初始化 + 可观性 + VINS/MSCKF 双主线 + 边缘化 + FEJ）。**ORB-SLAM3 的惯性特有内容挂这里**：

- `\subsection{ORB-SLAM3 的 MAP-based IMU 初始化：三阶段工程编排}`（`sec:vio-orbslam3-init`），挂在 `sec:vio-init` 之后。
  - **明确写「与 \cref{sec:vio-init} 的通用三段式初始化互为印证」**，只讲 ORB-SLAM3 的**系统化差异**：
    - 放在**局部建图子线程**完成，**不阻塞跟踪实时性**（关键工程设计）。
    - **因子图四阶段递进**（图 18-2 a/b/c/d）：纯视觉 MAP（尺度未知，固定不优化部分）→纯惯性 MAP（固定轨迹，优化 {s, R_wg, b, v}，尺度/零偏显式独立变量、收敛快）→视觉惯性联合 MAP（5~15s 联合 BA，误差收敛到 1%）→（单目）尺度精化（每 10s 跑、至 100KF/75s 收尾）。
    - 阶段标志 `mbImuInitialized / mbIMU_BA1 / mbIMU_BA2`；双目惯性=固定 s=1 加速收敛。
  - **不重推预积分残差/雅可比**（已在 `sec:vio-bias`/`sec:vio-imu-factor`）；只标注「ORB-SLAM3 用 \cref{thm:vio-imu-residual} 的残差构造惯性因子」。
  - 引用 `campos2021orbslam3`、`campos2020inertialinit`（IMU 初始化原论文，**新键**，见 §3）。

- `\subsection{ORB-SLAM3 的 IMU 模式跟踪状态机}`（`sec:vio-orbslam3-track`），挂在初始化节之后或并入上节。
  - `RECENTLY_LOST` 短/长丢失二态；IMU 预测位姿找回；恒速模型用 IMU 积分代替；重定位 EPnP→**MLPnP**（解耦相机模型）。要点级，**不贴 `Track()` 源码**。

### 2.D 新增内容：Atlas 多地图 + 地图融合 → **节群 or 章？**

这是本书唯一**完全无覆盖**、且自成体系的块（ORB-SLAM3 主题①③）。两种落法：

**方案 A（推荐，先做）：在 `vio.tex` 末尾新增「ORB-SLAM3 系统级整合」节群**（约 3 节）：
- `\section{ORB-SLAM3：多地图 Atlas 与地图融合}`（`sec:vio-orbslam3`）
  - `\subsection{多地图系统 Atlas}`（`sec:vio-atlas`）：地图集结构（活跃/非活跃、共享 DBoW2 库）；量化收益（表 16-1/16-2 改绘）；何时创建新地图（断臂求生、时间戳异常）；宽基线 vs 窄基线匹配。
  - `\subsection{检测共同区域与闭环/融合分流}`（`sec:vio-merge-detect`）：集卡式验证（共视几何 + 时序几何，召回率↑）；活跃地图内→闭环 vs 跨地图→融合分流。
  - `\subsection{地图融合：纯视觉与视觉惯性}`（`sec:vio-merge`）：局部窗口、Sim(3) 位姿传播矫正、生成树父子换向、熔接 Welding BA、本质图、全局 BA；视觉惯性融合的固定尺度+IMU 快优+视觉惯性熔接 BA（因子图 19-9）。

**方案 B（若节群体量超 ~600 行 / 想要独立章）：升格为新章 `parts/P2_slam/orbslam3_architecture.tex`**
- `\chapter{ORB-SLAM3 系统剖析：多地图与地图融合}`（`ch:orbslam3`）
- 在 `part.tex` 中**插在 `vio` 之后**（第 9 行 `\input{...vio}` 之后加 `\input{...orbslam3_architecture}`）。
- 章内含上述 3 节 + IMU 三阶段初始化（从 §2.C 移入），形成「ORB-SLAM3 = 视觉惯性紧耦合 + Atlas + 融合」的完整系统级章。

> **决策建议**：**先按方案 A 落节群**（保证「有机结合、不另起炉灶」）；写作过程中若 Atlas+融合+IMU 编排合计内容确实撑得起一章（生成树换向、Sim(3) 传播、熔接 BA 因子图都值得展开），**再升格为方案 B 的 `ch:orbslam3`**。两方案的**节级要点完全复用**，切换零返工。

### 2.E 综述增强 → `vio.tex` `sec:vio-frontier`（或本书综述章）

- 把 **表 20-1 主流 VI-SLAM 框架横向对比** + **表 20-2 标准数据集对比** 改绘融入 `sec:vio-frontier`（本书已有「前沿与延伸」节）。
- 视觉 SLAM 三阶段发展史（图 20-1 时间线）可作该节引子；未来趋势四条与本书事件相机/语义等章交叉引用。
- **注意**：表中框架 MSCKF/OKVIS/VINS/Kimera/BASALT 多数在 `vio.tex` 正文已述，**只补一张统一对比表**，不重述各框架。

---

## 3. refs.bib 键

### 已存在（直接 \cite，勿重复加）

| 键 | 内容 |
|---|---|
| `murartal2015orbslam` | ORB-SLAM（单目） |
| `murartal2017orbslam2` | ORB-SLAM2（单目/双目/RGB-D） |
| `forster2017preintegration` | On-Manifold Preintegration（预积分，已用于 `vio.tex`） |
| `qin2018vins` | VINS-Mono |
| `galvez2012bags` | DBoW2 词袋 |
| `mourikis2007msckf` | MSCKF |
| `leutenegger2015okvis` | OKVIS |
| `forster2017svo` | SVO |
| `engel2017dso` | DSO |
| `engelXXXX`（`LSD-SLAM`，行 957） | LSD-SLAM |
| `rosinol2021kimera` | Kimera |
| `huang2019vins` | VINS-Fusion（核对：行 478，疑为 VINS-Fusion 或多地图扩展，写作前确认） |

### 需新增（写作前补入 `refs.bib`）

| 建议键 | 文献 | 用途 |
|---|---|---|
| `campos2021orbslam3` | Campos C, Elvira R, Rodríguez J J G, et al. **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM**. IEEE T-RO, 2021, 37(6): 1874-1890. | 全部 ORB-SLAM3 内容的主引用（Ch15-20 反复出现） |
| `campos2020inertialinit` | Campos C, Montiel J M M, Tardós J D. **Inertial-Only Optimization for Visual-Inertial Initialization**. ICRA 2020: 51-57. | ORB-SLAM3 MAP-based IMU 三阶段初始化原论文（Ch18 [1]） |
| `elvira2019atlas` | Elvira R, Tardós J D, Montiel J M M. **ORBSLAM-Atlas: A Robust Multi-Map System**. IROS 2019: 6253-6259. | 多地图 Atlas 系统原论文（Ch16 [1]） |

> 另：BASALT / ROVIO / ICE-BA / Maplab / MonoSLAM / PTAM / RTAB-MAP / ElasticFusion 等综述表所列框架，**若仅出现在对比表则可不单独建键**（表内引文献编号即可）；若正文展开再按需补。

---

## 4. 写作分工建议（同 .tex 不并行）

> 硬规则：**同一 `.tex` 文件只能由一个 agent 写**（避免 Edit 冲突）。下列分工保证各 agent 文件不重叠。

| 批次 | Agent | 负责文件 | 任务 | 依赖 |
|---|---|---|---|---|
| **W1** | A | `refs.bib` | 先补 3 个新键（`campos2021orbslam3` / `campos2020inertialinit` / `elvira2019atlas`），核对 `huang2019vins` 实指 | 无（最先做，其余批次依赖键存在） |
| **W2** | B | `visual_odometry.tex` | §2.A：四叉树均匀化子节 | W1 |
| **W2** | C | `loop_closure.tex` | §2.B 末：Sim(3) 闭环计算与矫正子节 | W1 |
| **W2** | D | `slam_system.tex` | §2.B：ORB-SLAM2 方法精读节群（图结构 / 剔除准则 / 三段式跟踪+EPnP / 局部建图+四线程优化全景） | W1 |
| **W3** | E | `vio.tex` | §2.C + §2.D-方案A + §2.E：ORB-SLAM3 IMU 三阶段初始化编排 + IMU 模式跟踪状态机 + Atlas/融合节群 + 综述对比表 | W1；**与 B/C/D 文件不同，可与 W2 并行** |

**关键避坑（给写作 agent 的纪律）：**
- **预积分零吸收**：`vio.tex` 的 `sec:vio-preint` 已完整，agent E **严禁**重写 ΔR/Δv/Δp/噪声/零偏雅可比/残差；ORB-SLAM3 初始化节须用 `\cref{sec:vio-init}`/`\cref{thm:vio-imu-residual}` **回指**，只写系统级差异。
- **不搬源码**：全部以方法图 / 判定准则 / 因子图 / 伪码呈现；保留的代码 ≤ 5 行且必须是「点睛」（如 `CreateNewMap()` 的三步、阶段标志位）。
- **交叉引用既有锚点**：复用 `sec:vio-init`、`sec:vio-imu-factor`、`sec:loop-stageD`、`sec:loop-backend`、`sec:sys-orbslam`、`sec:sys-backend`、`sec:vo-orb-kp` 等，做到「有机结合」而非平行另写。
- **方案 A→B 升格**（若 W3 内容超 ~600 行）：agent E 把 §2.C 初始化 + §2.D Atlas/融合合并为新文件 `orbslam3_architecture.tex`，并在 `part.tex` 第 9 行后加 `\input`；其余节级要点零返工迁移。

---

## 附：本设计依据的关键核查结论

1. **`vio.tex` 已覆盖 IMU 预积分全套**（Forster 矩阵形式 + VINS 四元数形式、噪声分离与递推协方差、零偏一阶修正雅可比、9 维残差），故书 **Ch15 整章不吸收**。
2. **`vio.tex` `sec:vio-init` 已覆盖 VIO 三段式初始化理论**（视觉 SfM→陀螺零偏→速度/重力/尺度线性解→重力精化），故书 **Ch18 的通用初始化理论不重吸**，**只吸 ORB-SLAM3 把它放进局部建图线程、不阻塞跟踪、因子图四阶段递进** 的系统级差异。
3. **本书已覆盖**：ORB 特征（Oriented FAST+BRIEF，缺四叉树）、对极几何 F/H 单目初始化、三角化、PnP（缺 EPnP）、DBoW2 四阶段闭环+几何验证（缺 Sim(3)/闭环矫正）、恒速模型+pose-only-BA 跟踪、双线程系统架构、双目初始化。
4. **本书完全无、必吸**：ORB 四叉树均匀化、共视图/本质图/生成树、ORB-SLAM 关键帧/点剔除准则、Sim(3) 闭环矫正、EPnP/MLPnP、**Atlas 多地图、地图融合、ORB-SLAM3 IMU 三阶段编排、VI-SLAM 框架对比表/数据集表**。
