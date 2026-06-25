# 融入设计：高翔《自动驾驶与机器人中的 SLAM 技术：从理论到实践》整本对齐吸收 → P2

> 落盘 2026-06-25。本文件是**映射设计稿**（只出"书→章映射"，**不写正文**）。
> 目标书：高翔（半闲居士）著，电子工业出版社 2023.8，ISBN 978-7-121-45878-1，《视觉SLAM十四讲》同作者的**激光/惯性续作**。代码仓库 `github.com/gaoxiang12/slam_in_autonomous_driving`（C++17，ROS 存储，NCLT/UTBM/UrbanLoco 数据集）。
> 核心纪律：**有机结合**——重叠处=深化/校验，独特处=新增章/节；绝不 verbatim 堆砌。
> 适用范围：本书 P2「感知与 SLAM」部（`parts/P2_slam/`，现 13 章）+ 少量回引 P1「状态估计与滤波」部。

---

## 0. 总评（一句话结论）

高翔此书 = **十四讲未覆盖的激光/2D/组合导航半壁江山的工程化教科书**。它与本书 P2 的重叠面**比预期小**：本书在 **3D 点云配准（ICP 全家族/GICP/NDT）、LOAM/LeGO-LOAM/LIO-SAM/FAST-LIO、IMU 预积分（Forster+VINS 双主线）、ESKF/IEKF（P1）** 上已**深于高翔**（本书走严格右扰动推导 + Handbook/Barfoot 骨架）。高翔的**真正增量集中在三块本书几乎为零的空白**：

1. **2D 概率激光 SLAM 全家**（占据栅格 log-odds、似然场扫描匹配、Bresenham、子地图、多分辨率回环）——本书 grep 零命中；
2. **组合导航 GINS / RTK-GNSS / UTM 世界坐标系**——本书 grep 零命中（有 ESKF 引擎、有 IMU 模型，但从未组装成 GINS）；
3. **从零到一的完整激光定位建图系统**（离线高精地图两轮 PGO 流水线 + 实时点云融合定位 + 紧耦合 LIO 的 SMW 高维降维理论）——本书有零件无整机。

**建议：新增 4 章 + 深化 5 处 weave-in**（详见 §3、§6）。这与 Handbook 吸收用过的"经典织进 + 少量就近新建"混合方案一脉相承。

---

## 1. 高翔全书章节目录（完整列）

> 全书 3 部分 10 章，585 千字，24.25 印张。下列页码为**书内页码**。

### 第一部分　基础数学知识（书 1–133）
- **第 1 章　自动驾驶**（书 3）
  - 1.1 自动驾驶技术（1.1.1 自动驾驶能力与分级；1.1.2 L4 的典型业务）
  - 1.2 自动驾驶中的定位与地图（1.2.1 为什么 L4 需要定位与地图；1.2.2 高精地图的内容与生产）
  - 1.3 本书内容的介绍顺序
- **第 2 章　基础数学知识回顾**（书 17）
  - 2.1 几何学（2.1.1 坐标系；2.1.2 李群与李代数；2.1.3 SO(3) 上的 BCH 线性近似式）
  - 2.2 运动学（2.2.1 李群视角下的运动学；2.2.2 四元数视角下的运动学；2.2.3 四元数的李代数与旋转矢量间的转换；2.2.4 其他几种运动学表达方式；2.2.5 线速度与加速度；2.2.6 扰动模型与雅可比矩阵）
  - 2.3 运动学演示案例：圆周运动
  - 2.4 滤波器与最优化理论（2.4.1 状态估计问题与最小二乘；2.4.2 卡尔曼滤波器；2.4.3 非线性系统的处理方法；2.4.4 最优化方法与图优化）
  - 2.5 本章小结 + 习题
- **第 3 章　惯性导航与组合导航**（书 47）
  - 3.1 IMU 系统的运动学（3.1.1 关于 IMU 测量值的解释；3.1.2 IMU 测量方程中的噪声模型；3.1.3 IMU 的离散时间噪声模型；3.1.4 现实中的 IMU）
  - 3.2 使用 IMU 进行航迹推算（3.2.1 利用 IMU 数据进行短时间航迹推算；3.2.2 IMU 递推的代码实验）
  - 3.3 卫星导航（3.3.1 GNSS 的分类与供应商；3.3.2 实际的 RTK 安装与接收数据；3.3.3 常见的世界坐标系；3.3.4 RTK 读数的显示）
  - 3.4 使用误差状态卡尔曼滤波器（ESKF）实现组合导航（3.4.1 ESKF 的数学推导；3.4.2 离散时间的 ESKF 运动方程；3.4.3 ESKF 的运动过程；3.4.4 ESKF 的更新过程；3.4.5 ESKF 的误差状态后续处理）
  - 3.5 实现 ESKF 的组合导航（3.5.1 ESKF 的实现；3.5.2 实现预测过程；3.5.3 实现 RTK 观测过程；3.5.4 ESKF 系统的初始化；3.5.5 运行 ESKF；3.5.6 速度观测量）
  - 3.6 本章小结 + 习题
- **第 4 章　预积分学**（书 99）
  - 4.1 IMU 状态的预积分学（4.1.1 预积分的定义；4.1.2 预积分测量模型；4.1.3 预积分噪声模型；4.1.4 零偏的更新；4.1.5 预积分模型归结至图优化；4.1.6 预积分的雅可比矩阵；4.1.7 小结）
  - 4.2 实践：预积分的程序实现（4.2.1 实现预积分类；4.2.2 预积分的图优化顶点；4.2.3 预积分方案的图优化边；4.2.4 实现基于预积分和图优化的 GINS）
  - 4.3 本章小结 + 习题

### 第二部分　激光雷达的定位与建图（书 135–304）
- **第 5 章　基础点云处理**（书 137）
  - 5.1 激光雷达传感器与点云的数学模型（5.1.1 激光雷达传感器的数学模型 [RAE 极坐标]；5.1.2 点云的表达；5.1.3 Packet 的表达 [Velodyne HDL-64S3]；5.1.4 俯视图和距离图 [BEV / Range Image]；5.1.5 其他表达形式 [Surfel]）
  - 5.2 最近邻问题（5.2.1 暴力最近邻法；5.2.2 栅格与体素方法 [空间哈希]；5.2.3 二分树与 K-d 树；5.2.4 四叉树与八叉树；5.2.5 其他树类方法；5.2.6 小结）
  - 5.3 拟合问题（5.3.1 平面拟合；5.3.2 平面拟合的实现；5.3.3 直线拟合；5.3.4 直线拟合的实现）
  - 5.4 本章小结 + 习题
- **第 6 章　2D SLAM**（书 191）
  - 6.1 2D SLAM 的基本原理
  - 6.2 扫描匹配算法（6.2.1 点到点的扫描匹配；6.2.2 点到点 ICP 的实现 [高斯-牛顿]；6.2.3 点到线的扫描匹配算法 [PL-ICP]；6.2.4 点到线 ICP 的实现 [高斯-牛顿]；6.2.5 似然场法；6.2.6 似然场法的实现 [高斯-牛顿]；6.2.7 似然场法的实现 [g2o]）
  - 6.3 占据栅格地图（6.3.1 占据栅格地图的原理 [Ray Casting + log-odds]；6.3.2 基于 Bresenham 算法的地图生成；6.3.3 基于模板的地图生成）
  - 6.4 子地图（6.4.1 子地图的原理 [T_WS·T_SC]；6.4.2 子地图的实现）
  - 6.5 回环检测与闭环（6.5.1 多分辨率的回环检测 [金字塔 / BAB 对比]；6.5.2 基于子地图的回环修正 [SE2 位姿图]；6.5.3 讨论 [运动畸变 / 退化 / Gauge Freedom]）
  - 6.6 本章小结 + 习题
- **第 7 章　3D SLAM**（书 243）
  - 7.1 多线激光雷达的工作原理（7.1.1 机械旋转式激光雷达；7.1.2 固态激光雷达 [Livox]）
  - 7.2 多线激光雷达的扫描匹配（7.2.1 点到点 ICP；7.2.2 点到线、点到面 ICP；7.2.3 NDT 方法；7.2.4 本节各种配准方法与 PCL 内置方法的对比）
  - 7.3 直接法激光雷达里程计（7.3.1 使用 NDT 构建激光雷达里程计；7.3.2 增量 NDT 里程计 [LRU 缓存 + 在线高斯合并]）
  - 7.4 特征法激光雷达里程计（7.4.1 特征的提取；7.4.2 基于激光雷达线束的特征提取；7.4.3 特征提取部分的实现；7.4.4 特征法激光雷达里程计的实现 [仿 LOAM]）
  - 7.5 松耦合 LIO 系统（7.5.1 坐标系说明 [W/I/L + 外参 T_IL]；7.5.2 运动与观测方程；7.5.3 数据准备 [CloudConvert / MessageSync]；7.5.4 主要流程 [ESKF + 增量 NDT]；7.5.5 配准部分 [去畸变 + ObserveSE3 回灌]）
  - 7.6 本章小结 + 习题

### 第三部分　应用实例（书 305–359）
- **第 8 章　紧耦合 LIO 系统**（书 307）
  - 8.1 紧耦合的原理和优点
  - 8.2 基于 IEKF 的 LIO 系统（8.2.1 IEKF 状态变量与运动方程 [18 维误差态]；8.2.2 观测方程中的迭代过程 [ICP/NDT 残差作观测]；8.2.3 高维观测的等效处理 [**SMW 恒等式降到 18×18**]）
  - 8.3 实现基于 IEKF 的 LIO 系统
  - 8.4 基于预积分的 LIO 系统（8.4.1 预积分 LIO 系统的原理；8.4.2 代码实现 [g2o + 手动边缘化先验]）
  - 8.5 本章小结 + 习题
- **第 9 章　自动驾驶车辆的离线地图构建**（书 329）
  - 9.1 点云建图的流程
  - 9.2 前端实现 [复用第 8 章 IEKF LIO + 抽关键帧 + RTK 位姿]
  - 9.3 后端位姿图优化与异常值检验 [RTK 因子 / 相对运动因子 / 回环因子 + 鲁棒核两阶段]
  - 9.4 回环检测 [Scan-to-Map + 多分辨率 NDT 10/5/4/3 m]
  - 9.5 地图的导出 [100 m 网格切分 + 索引]
  - 9.6 本章小结 + 习题
- **第 10 章　自动驾驶车辆的实时定位**（书 349）
  - 10.1 点云融合定位的设计方案 [ESKF vs 位姿图 + 初始化网格搜索]
  - 10.2 算法实现（10.2.1 RTK 初始搜索 [360°/10° 步长扫朝向]；10.2.2 外围测试代码 [九宫格地图动态加卸载]）
  - 10.3 本章小结 + 习题

参考文献（书 360+）。

---

## 2. 逐章映射表（书 → P2 落点 + 重叠/独特判定）

> 图例：**【深化】**=本书已有同主题且更深/相当，吸收=对照校验+少量补点 weave-in；**【新增】**=本书空白，需新建章/节；**【已超】**=本书已显著深于高翔，仅作引用/不动。
> 关键事实：本书 ESKF/IEKF 在 **P1 `kalman_eskf.tex`**（`\label{ch:eskf}`），不在 P2；本书预积分在 **P2 `imu_model.tex`**（`\label{ch:imu}`，Forster+VINS 双主线）。

| 高翔章 | 主题 | 本书现状 | 映射判定 | 落点 |
|---|---|---|---|---|
| **Ch1 自动驾驶** | L4 业务 / 定位与地图 / 高精地图生产 | P0_intro `intro_slam.tex` 有 SLAM 总览，无自动驾驶/高精地图专述 | **【新增·轻】** 半页背景 | 织入新章「组合导航」引言，或 P0_intro 补一小节"自动驾驶定位栈与高精地图"。**低优先**（属背景非算法）。 |
| **Ch2 数学回顾** | 坐标系 / 李群李代数 / BCH / 四元数运动学 / 扰动雅可比 / KF / 图优化 | P0_math `lie_theory`+`rigid_body_motion`、P1 全部 | **【已超】** | **不吸收**（本书 P0/P1 已系统覆盖且更深，右扰动主线一致）。高翔此章是十四讲前 6 章的浓缩复述。 |
| **Ch3 §3.1 IMU 运动学/噪声** | 测量值解释 / 噪声模型 / 连续→离散噪声 / 现实 IMU | P2 `imu_model.tex` §2,§3.3,§7（极深，Allan 方差/OpenVINS 内参谱系） | **【已超】** | 不动。 |
| **Ch3 §3.2 IMU 航迹推算** | 短时 dead-reckoning + 代码实验 | `imu_model.tex` §3.1–3.2 仅作"漂移反面分析"，**无独立可运行递推里程计** | **【深化·补点】** | weave 进 `imu_model.tex`：补一小节"纯惯性递推作为里程计模块（及其漂移定量）"，或在新章"组合导航"用作 ESKF 预测的引子。 |
| **Ch3 §3.3 卫星导航 GNSS/RTK** | GNSS 分类供应商 / RTK 安装接收 / **世界坐标系 UTM/经纬度** / RTK 读数 | **零命中**（仅 ECI/ECEF/NED/ENU 简述于 imu §7.3，无 UTM 投影、无经纬度↔局部系） | **【新增·核心】** | → 新章「组合导航：GINS 与 RTK-GNSS」§1–2。 |
| **Ch3 §3.4–3.5 ESKF 组合导航** | ESKF 数学推导/运动方程/更新/误差后续 + GINS 实现 + 速度观测 | ESKF **数学**在 P1 `kalman_eskf.tex` §"ESKF"（真态/名义态/预测/校正/注入复位/右扰动，极深）；**但组合导航应用零命中** | **【深化(数学,引P1) + 新增(应用)】** | ESKF 公式**引用 P1**；**GINS 组合导航管线（IMU 预测 + RTK 位姿观测 + 速度观测 + 初始化）= 新章核心**。 |
| **Ch4 预积分学** | 预积分定义/测量模型/噪声/零偏更新/归图/雅可比 + **GINS（预积分+图优化）** | `imu_model.tex` §4–6 完整（ΔR/Δv/Δp、协方差 Σ←AΣAᵀ+BΣ_ηBᵀ、5 个偏置雅可比、9 维残差）；vio §2 复述 | **【已超(预积分) + 新增(预积分版GINS应用)】** | 预积分理论不动（本书更系统）；**"预积分+图优化做 GINS"这一应用落点**进新章「组合导航」§3（与 ESKF 版 GINS 并列两条路线）。 |
| **Ch5 §5.1 点云数学模型/表达** | RAE / Packet / **BEV 俯视图 / Range 距离图** / Surfel | `point_cloud_processing` §131 表列 BEV/range-image/surfel；`lidar_slam` §159 RAE 深；**但 Packet/BEV/Range 的构造细节浅** | **【深化·补点】** | weave 进 `point_cloud_processing.tex`：补"Packet 解析 + BEV/Range Image 投影构造"一小节（工程表达，当前仅表格一行）。 |
| **Ch5 §5.2 最近邻** | 暴力 / 栅格体素哈希 / K-d / 八叉树 / 其他树 | `point_cloud_processing` K-d 树**深**、体素**深**、暴力**中**；**八叉树仅表格一行（浅）** | **【已超(K-d/voxel) + 深化(octree)】** | 大体不动；可 weave 八叉树最近邻算法细节（Box3D 包围盒剪枝）入 `point_cloud_processing` 或回指 `dense_mapping` 的 OctoMap。 |
| **Ch5 §5.3 拟合** | 平面拟合 / 直线拟合（SVD/特征值） | 本书把拟合**融进 PCA 特征值框架**（法向估计/LOAM/GICP），**无独立"ax+by+cz+d=0 最小二乘"小节** | **【深化·补点】** | weave 进 `point_cloud_processing.tex`：补独立"平面/直线最小二乘拟合（超定方程 SVD 解）"小节，对齐高翔的工具化讲法（也服务于后面似然场/特征法）。 |
| **Ch6 2D SLAM（整章）** | 点点/点线 ICP 扫描匹配 / **似然场** / **占据栅格(log-odds/Bresenham/模板)** / **子地图** / **多分辨率回环 + SE2 PGO** | **几乎全零命中**：似然场=0、占据栅格 2D=0（dense_mapping 只有 3D OctoMap）、Bresenham 仅 1 处提及、2D 相关性扫描匹配=0、SE2 子地图=0 | **【新增·核心整章】** | → **新章「2D 激光 SLAM 与占据栅格建图」**（完整一章）。本书与高翔重叠最低、增量最大的一块。 |
| **Ch7 §7.1 多线雷达原理** | 机械旋转式 / 固态 Livox | `lidar_slam` §148 传感器分类（机械/MEMS/flash/Risley）已有 | **【深化·相当】** | 基本覆盖；可 weave Livox 花瓣式扫描 + N 线垂直排布成像几何细节（当前浅）入 `lidar_slam`。 |
| **Ch7 §7.2 3D 扫描匹配** | 点点/点线/点面 ICP / NDT / 与 PCL 对比 | `point_cloud_processing` 全套**深**（SVD/四元数/Low 线性化/GICP/NDT 梯度海森） | **【已超】** | 不动。高翔的"手写 vs PCL 横评"可作旁注引用。 |
| **Ch7 §7.3 直接法 NDT 里程计 + 增量 NDT** | NDT 构建 LO / **增量 NDT（LRU + 在线高斯合并 式7.17–7.25）** | NDT **配准算法**深；**但"NDT 作里程计系统 + 增量体素地图在线更新"=浅/无** | **【深化·补节(系统化)】** | weave 进 `lidar_slam.tex`：补"直接法 NDT 激光里程计 + 增量 NDT 体素地图（在线高斯合并/LRU 缓存）"小节——把已讲透的 NDT *算法* 提升为 *里程计/建图系统*。 |
| **Ch7 §7.4 特征法 LOAM 里程计** | 线束曲率提特征 / 角点点线 + 平面点点面 | `point_cloud_processing` §1226 + `lidar_slam` §369 LOAM **深** | **【已超】** | 不动。 |
| **Ch7 §7.5 松耦合 LIO** | W/I/L 坐标系 / 运动观测方程 / 时间同步 / **ESKF + 增量 NDT 装机** / 去畸变回灌 | `lidar_slam` §52 区分松/紧、§633 指 LOAM 的 IMU=松耦合；**但无"ESKF + 增量 NDT 的松耦合 LIO 完整装机流程"** | **【深化·补节(系统装机)】** | weave 进 `lidar_slam.tex`：补"松耦合 LIO 系统装机（ESKF 预测 + 增量 NDT 观测 + MessageSync 同步 + ESKF 预测位姿去畸变）"——本书有全部零件（ESKF/NDT/预积分/去畸变四法），缺这条**装配主线**。 |
| **Ch8 §8.1–8.3 紧耦合 LIO (IEKF)** | IEKF 状态/迭代观测 / **SMW 高维观测降到 18×18** / 实现 | IEKF **数学**在 P1（Bell-Cathey/IKFoM/FAST-LIO，极深）；`lidar_slam` FAST-LIO 逐行 IEKF **深** | **【已超(IEKF) + 深化·补点(SMW 视角)】** | IEKF 不重写；**weave 高翔独到的"SMW 恒等式把高维 NDT/ICP 观测求逆等效降到 18×18、揭示紧耦合 LIO = 带 IMU 预测的高维 NDT"** 入 `lidar_slam.tex`（或 P1 IEKF 节的应用旁注）——这是高翔相对本书 FAST-LIO 推导的**增量视角**。 |
| **Ch8 §8.4 预积分 LIO** | 预积分因子 + NDT 位姿观测 + 手动边缘化先验 | `lidar_slam` LIO-SAM 因子图**深**；`imu_model` 预积分+边缘化**深** | **【已超】** | 不动（LIO-SAM 已是同型紧耦合因子图 LIO）。可一句话点出"NDT 位姿观测替代 GPS 约束"的差异。 |
| **Ch9 离线地图构建（整章）** | 流程 / 前端抽关键帧 / **两轮 PGO（RTK 绝对 + LIO 相对 + 鲁棒核两阶段去异常）** / **Scan-to-Map 多分辨率 NDT 回环** / **100 m 网格切分导出** | **无对应系统**：本书有 PGO（`lidar_slam` §1100、P1 isam2）、有回环零件，**但无"数据包进→分块高精地图出"的离线建图流水线** | **【新增·核心整章(系统)】** | → **新章「自动驾驶离线高精地图构建」**（完整一章，系统组织 + 两轮优化 + 回环 + 地图切分）。 |
| **Ch10 实时定位（整章）** | 点云融合定位设计 / **RTK 引导初始化网格搜索（360°/10°）** / **九宫格地图动态加卸载** / ESKF 地图匹配定位 | **无对应**：本书无"在先验点云地图上实时定位"的系统（VO/VIO 是里程计非地图定位） | **【新增·核心整章(系统)】** | → **新章「自动驾驶实时点云融合定位」**（完整一章，冷启动网格搜索 + 地图分块加载 + ESKF + NDT ObserveSE3）。 |

---

## 3. 建议新增章/节（章序 + 节级大纲）

> P2「感知与 SLAM」当前章序（`part.tex`）：camera_model → camera_calibration → imu_model → point_cloud_processing → visual_odometry → loop_closure → lidar_slam → vio →（新型传感器群）radar_slam → event_camera_slam → leg_odometry → dense_mapping → slam_system。
>
> **新增 4 章统一插在激光线主体之后、新型传感器群之前**（即紧接 `lidar_slam.tex`，因它们都建立在 lidar_slam/point_cloud/imu/ESKF 之上）。建议章序与 label：

```
... imu_model → point_cloud_processing → visual_odometry → loop_closure → lidar_slam
  → 【新】gins_navigation     （组合导航：GINS 与 RTK-GNSS）        \label{ch:gins}
  → 【新】slam_2d             （2D 激光 SLAM 与占据栅格建图）        \label{ch:slam2d}
  → 【新】lidar_mapping       （自动驾驶离线高精地图构建）           \label{ch:lidar_mapping}
  → 【新】lidar_localization  （自动驾驶实时点云融合定位）           \label{ch:lidar_loc}
  → vio → radar_slam → event_camera_slam → leg_odometry → dense_mapping → slam_system
```

理由：① GINS 接 `imu_model`（预积分/IMU）+ P1 ESKF，是激光定位建图的惯性/卫星地基；② 2D SLAM 接 `point_cloud_processing`（扫描匹配/拟合）；③④ 建图/定位是**应用顶层**，依赖前面所有（lidar_slam 的 NDT/LIO + GINS 的 RTK + 2D 的子地图回环思想）。四章顺序即"地基→2D→建图→定位"的依赖链。

> **注（与"GINS 放哪"的替代方案）**：GINS 也可考虑放 P1（紧贴 `kalman_eskf` 作为 ESKF/IEKF 的旗舰应用）。**本设计选 P2**，理由是高翔把 GINS 与激光定位建图绑成一条自动驾驶主线，且 GINS 的 RTK/UTM/世界坐标系更贴"感知-定位"语境；P1 保持"通用估计理论"纯度。ESKF/IEKF 公式仍 `\cref{ch:eskf}` 回引 P1，不复述。

### 新章 A：`gins_navigation` —「组合导航：GINS 与 RTK-GNSS」（`\label{ch:gins}`）
> 定位：把本书已有的 ESKF（P1）+ IMU 模型/预积分（`ch:imu`）**组装**成可运行的 GNSS-惯性组合导航；填补 RTK/GNSS/世界坐标系空白。
- §1 为什么需要组合导航：IMU 漂移 vs GNSS 慢而绝对的互补性；松耦合数据流；与自动驾驶定位栈的关系（含 Ch1 高精地图背景半页）。
- §2 卫星导航与世界坐标系：GNSS 分类与供应商（GPS/北斗/Galileo/GLONASS）；差分与 **RTK** 原理（载波相位、基准站、固定解/浮点解）；**世界坐标系**（WGS84 经纬度、**UTM 横轴墨卡托投影**、ECEF/ENU/局部系转换）；单/双天线与 RTK 读数（位置+可选航向）。【对齐 Gao §3.3】
- §3 ESKF 组合导航（GINS）：回引 `\cref{ch:eskf}` 的 ESKF 公式（真态/名义态/预测/校正/注入复位，**不复述推导**）；**应用层**：IMU 高频预测 + **RTK 位姿观测**更新 + **速度观测**；系统初始化（静止估零偏 + RTK 定位姿）；单天线无航向时的处理。【对齐 Gao §3.4–3.5】
- §4 预积分 + 图优化的 GINS：用 `\cref{ch:imu}` 的预积分因子 + RTK 绝对位姿因子 + 零偏随机游走因子构图；与 ESKF 版对照（滤波 vs 优化，回引 P1 滤波-优化谱）。【对齐 Gao §4.2】
- §5 实践与对比：ESKF-GINS vs 预积分-GINS 精度/复杂度；为后续离线建图（RTK 作绝对约束）与实时定位（RTK 作初始化）埋接口。
- §6 小结 + 习题。

### 新章 B：`slam_2d` —「2D 激光 SLAM 与占据栅格建图」（`\label{ch:slam2d}`）
> 定位：本书最大空白，经典 2D 概率栅格 SLAM 全家（gmapping/Hector/Cartographer 谱系的数学内核）。
- §1 2D SLAM 总览：单线激光的 (ρ,θ) 观测；2D SLAM 架构（扫描匹配 + 占据栅格 + 子地图 + 回环）；与 3D LIO 的异同。
- §2 2D 扫描匹配：点到点 ICP（2D 位姿 [x,y,θ]，SE2/SO2 更新，高斯-牛顿雅可比）；**点到线 ICP / PL-ICP**（拟合直线 + 残差雅可比）；与 `\cref{ch:pointcloud}` 3D ICP 的降维关系（避免重复，只讲 2D 特有）。【对齐 Gao §6.2.1–6.2.4】
- §3 **似然场扫描匹配**（likelihood field）：距离变换图"磁场吸引"思想；目标函数 + π 函数对位姿雅可比；高斯-牛顿实现 + g2o 一元边实现两版；多分辨率似然场金字塔（接 §5 回环）。【对齐 Gao §6.2.5–6.2.7】——**本书完全空白**。
- §4 **占据栅格地图**（occupancy grid）：占据概率 / **log-odds 二值贝叶斯滤波** / 逆传感器模型；**Ray Casting + Bresenham 整数画线**栅格化；模板法生成；动态物体过滤。与 `dense_mapping` 的 **3D OctoMap 对照**（2D 平面栅格 vs 3D 八叉树，回指不重复 log-odds 推导）。【对齐 Gao §6.3】——**本书 2D 部分空白**。
- §5 子地图与多分辨率回环：**SE2 子地图**（T_WS·T_SC，子地图为回环基本单元）；**多分辨率（由粗至精）回环检测** + SE2 位姿图闭环；与 **Cartographer 的分支定界 BnB** 对照（高翔用金字塔近似，点明 BnB 是精确替代——补 `loop_closure`/此处 BnB 空白）。【对齐 Gao §6.4–6.5】
- §6 讨论（运动畸变 / 退化与 Gauge Freedom / 多传感器融合）+ 小结 + 习题。

### 新章 C：`lidar_mapping` —「自动驾驶离线高精地图构建」（`\label{ch:lidar_mapping}`）
> 定位：系统集成章——把前面所有零件拼成"数据包进→分块高精地图出"的离线流水线。
- §1 离线建图流程总览：数据包 → IMU/RTK/Lidar → LIO 前端 → 关键帧 → 第一轮 PGO → 回环 → 第二轮 PGO → 地图导出；离线 vs 在线（确定性/可调度）。
- §2 前端：复用 `\cref{ch:lidar_slam}` 的 LIO（IEKF 或松耦合）跑里程计；按距离/角度**抽关键帧**（点云存盘省内存）；关键帧赋 **RTK 位姿**（时间插值 SLERP+线性）。
- §3 第一轮后端 PGO：三类因子——**RTK 绝对位姿因子**（单天线只平移）/ **LIO 相对运动因子** / 回环因子；**鲁棒核两阶段**（带核优化 → 去核再优化）剔除 RTK 异常值；RTK 无姿态时用轨迹 ICP 估整体旋转。回引 P1 `\cref{ch:isam2}`/PGO。
- §4 回环检测：空间近时间远的检查点对；**Scan-to-Map** 配准（关键帧邻域抽子地图）；**多分辨率 NDT（10/5/4/3 m）由粗至精** + NDT 分值判据；并发执行。【可回引新章 B 的多分辨率思想 + `ch:lidar_slam` 的 NDT】
- §5 第二轮 PGO（纳入回环消重影）+ **地图导出**（100 m 网格切分 + 体素滤波 + 索引文件）。
- §6 小结 + 习题。【对齐 Gao Ch9 整章】

### 新章 D：`lidar_localization` —「自动驾驶实时点云融合定位」（`\label{ch:lidar_loc}`）
> 定位：在新章 C 产出的先验点云地图上做实时定位（区别于里程计）。
- §1 点云融合定位设计：与组合导航的异同（多了激光-地图匹配输入）；融合手段（**ESKF** 平滑 vs 位姿图鲁棒）；点云定位需初值引导 NDT 收敛（不像 RTK 直接给坐标）。
- §2 **冷启动初始化**：首个有效 RTK 控制搜索范围；RTK 无航向时 **360° 按 10° 步长并发扫朝向**，多分辨率 NDT 取最高分；成功置 WORKING。【对齐 Gao §10.2.1】——**本书空白**。
- §3 **地图动态加卸载**：按当前位姿加载车辆周边**九宫格 100 m 地图块**，卸载范围略大于加载范围防边界抖动；地图变更时重建 NDT target（K-d 树重建/单独加载线程）。
- §4 实时融合主循环：ESKF 预测 → 去畸变 → `LoadMap(pred)` → NDT 与地图配准 → `ObserveSE3` 回灌滤波器；失效重定位（NDT 分值判据）。
- §5 工程要点与泛化：跨月份数据验证（建图/定位不同时段）；点云定位对天气/遮挡的鲁棒性；大场景压成 2D/2.5D 地图。+ 小结 + 习题。【对齐 Gao Ch10 整章】

---

## 4. refs.bib：需并入的文献键

> 现状：`refs.bib`（~324 条）已含 `gaoxiang2019slam14`(十四讲)、`zhang2014loam`、`shan2018legoloam`、`shan2020liosam`、`xu2021fastlio`/`xu2022fastlio2`、`magnusson2009ndt`、`biber2003ndt`、`forster2017preintegration`、`sola2017quaternion`/`sola2018micro`、`thrun2005probabilistic`、`kim2018scancontext`、`lu2023ringpp`、`kim2020mulran`、`burnett2023boreas`、IKFoM note 等。**激光/预积分/ESKF/NDT/ScanContext 文献齐全。**

**必须新增（本书 grep 零命中）**：

| 建议 bib 键 | 文献 | 用途 |
|---|---|---|
| `gao2023slamad` | 高翔《自动驾驶与机器人中的 SLAM 技术：从理论到实践》, 电子工业出版社 2023, ISBN 978-7-121-45878-1 | **目标书本身**，所有新章吸收来源主引；代码仓 `gaoxiang12/slam_in_autonomous_driving` |
| `hess2016cartographer` | Hess, Kohler, Rapp, Andor. *Real-Time Loop Closure in 2D LIDAR SLAM* (Cartographer), ICRA 2016 | 新章 B §5（**分支定界 BnB** + 2D 子地图回环）；补 `loop_closure` BnB 空白 |
| `grisetti2007gmapping` | Grisetti, Stachniss, Burgard. *Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters* (GMapping), T-RO 2007 | 新章 B §4（占据栅格 2D SLAM 代表系统） |
| `kohlbrecher2011hector` | Kohlbrecher et al. *A Flexible and Scalable SLAM System with Full 3D Motion Estimation* (Hector SLAM), SSRR 2011 | 新章 B §3（似然场扫描匹配代表系统） |
| `censi2008plicp` | Censi. *An ICP variant using a point-to-line metric* (PL-ICP), ICRA 2008 | 新章 B §2（点到线 2D 扫描匹配） |
| `carlevaris2016nclt` | Carlevaris-Bianco, Ushani, Eustice. *University of Michigan North Campus Long-Term Vision and LIDAR Dataset* (NCLT), IJRR 2016 | 新章 C/D 实测数据集（高翔建图/定位主用） |
| `yan2020utbm` | Yan et al. *EU Long-term Dataset with Multiple Sensors for Autonomous Driving* (UTBM), IROS 2020 | 新章 C/D 数据集（松耦合 LIO） |
| `wen2020urbanloco` | Wen et al. *UrbanLoco: A Full Sensor Suite Dataset for Mapping and Localization in Urban Scenes* (ULHK), ICRA 2020 | 新章 C/D + `lidar_slam` 直接法 LO 数据集 |

**可选（若 weave 涉及）**：
- `solà2017quaternion` 已有 → ESKF 公式回引，无需新增。
- Bresenham 画线、UTM 投影、SMW(Sherman–Morrison–Woodbury) 恒等式：经典数学，正文给出即可，**通常不单列文献**（SMW 可引任一数值线代教材，本书 P1/附录或已有）。
- Livox 花瓣式扫描：可引 Livox 白皮书或 `lin2020loamlivox`（若 weave §Ch7.1）。

> 建议把上述新增键先汇总到 `docs/高翔SLAM_新增文献_待并.bib`，按既有流程中心化并入 `refs.bib`（仿 Handbook/Tutorial 的"待并 bib → 集中合并"）。

---

## 5. 写作分工建议（按章拆 agent，同一 .tex 不并行写）

> 纪律：一章一 owner；新章独立文件天然不冲突；**weave-in 深化点按目标 .tex 串行**（同一 .tex 的多个 weave 由同一 agent 顺序做，禁并行写同文件）。refs.bib 中心化合并（单独一个收口步骤，不让各 writer 并发改 bib）。

**A. 新章 writers（4 个独立 .tex，可全并行）**
| Agent | 产出文件 | 依赖/回引 |
|---|---|---|
| W-gins | `parts/P2_slam/gins_navigation.tex` | 回引 `ch:eskf`(P1 ESKF/IEKF)、`ch:imu`(预积分)；不复述推导 |
| W-2d | `parts/P2_slam/slam_2d.tex` | 回引 `ch:pointcloud`(3D ICP 降维)、`ch:dense`(3D OctoMap 对照)、`ch:loop`(BnB) |
| W-map | `parts/P2_slam/lidar_mapping.tex` | 回引 `ch:lidar_slam`(LIO/NDT)、`ch:gins`(RTK 约束)、`ch:isam2`/PGO、`ch:slam2d`(多分辨率回环) |
| W-loc | `parts/P2_slam/lidar_localization.tex` | 回引 `ch:lidar_mapping`(地图)、`ch:eskf`、`ch:lidar_slam`(NDT) |

**B. 深化 weave-in writers（按目标 .tex 分组，组内串行）**
| Agent | 目标 .tex（串行处理该文件内所有 weave） | weave 点 |
|---|---|---|
| D-imu | `imu_model.tex` | ① 纯惯性递推作里程计模块（Gao §3.2） |
| D-pc | `point_cloud_processing.tex` | ① 平面/直线最小二乘独立小节（§5.3）；② Packet/BEV/Range Image 构造（§5.1）；③（可选）八叉树最近邻细节（§5.2.4） |
| D-lidar | `lidar_slam.tex` | ① 直接法 NDT 里程计 + 增量 NDT 体素地图（§7.3）；② 松耦合 LIO 装机流程（§7.5）；③ 紧耦合 LIO 的 SMW 高维降维视角（§8.2.3）；④（可选）Livox 扫描几何（§7.1） |

> **排期建议**：先并发跑 A 的 4 个新章 + B 的 3 个 weave 组（共 7 个 writer，互不写同文件）；全部回来后，单独一步**收口**：refs.bib 合并 + `part.tex` 插入 4 行 `\input` + 全树 label/cref/cite 静态体检 + 跨章脉络一致性（仿 Handbook 收尾）。**编译为唯一硬验收（需 docker/gpu 机）**。

---

## 6. 全书吸收规模估计（供主控排期）

| 类别 | 数量 | 说明 |
|---|---|---|
| **新增章** | **4 章** | gins_navigation / slam_2d / lidar_mapping / lidar_localization。对齐 Gao Ch3(部分)+Ch6+Ch9+Ch10。预计每章 ~700–1000 行（仿 P2 现有章密度），合计 **~3000–3800 行**。 |
| **深化 weave-in** | **3 个 .tex / 约 7 个 weave 点** | imu_model(1) + point_cloud_processing(2–3) + lidar_slam(3–4)。每点 ~80–200 行，合计 **~800–1200 行**。 |
| **不吸收（已超/已覆盖）** | Gao Ch2 全、Ch5 主体、Ch7 §7.2/§7.4、Ch8 §8.4 | 本书 P0/P1/P2 已系统覆盖且更深；仅作引用/旁注，**零新写**。 |
| **refs.bib** | **+8 条**（+可选 1–2） | 目标书 + Cartographer/GMapping/Hector/PL-ICP + NCLT/UTBM/UrbanLoco。 |
| **基建改动** | `part.tex` +4 行 `\input` | 章序插入 lidar_slam 之后、vio 之前。 |

**总规模**：约 **4 新章 + 3 文件 7 处深化**，新写 **~3800–5000 行**，新增文献 **~8 条**。相对 Gao 全书 585 千字，本书**吸收率约 40%**（Part 1 大半已超、Part 2 Ch5/Ch7 主体已超；真增量 = Ch3 卫星/GINS 应用 + Ch6 整章 + Ch9 + Ch10 + 若干系统化 weave）。

**关键决策小结**：
1. **新增 4 章**（组合导航 / 2D 激光 SLAM / 离线建图 / 实时定位），插在 `lidar_slam` 之后。
2. **深化 3 文件**（imu_model / point_cloud_processing / lidar_slam）共 ~7 weave 点。
3. **ESKF/IEKF/预积分/ICP/NDT/LOAM/LIO 不重写**——本书 P0/P1/P2 已深于高翔，一律 `\cref` 回引。
4. **高翔最独到、值得显式吸收的两点理论**：① **SMW 把高维点云观测降到 18×18、揭示"紧耦合 LIO = 带 IMU 预测的高维 NDT"**（weave 进 lidar_slam）；② **2D 似然场 + 占据栅格 log-odds + 多分辨率回环**（新章 B 主体）。
5. **GINS 放 P2 不放 P1**（贴自动驾驶定位语境；P1 保持通用估计理论纯度，ESKF 公式回引）。
