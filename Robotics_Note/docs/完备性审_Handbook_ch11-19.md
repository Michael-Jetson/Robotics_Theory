# 完备性审核报告：SLAM Handbook 第 11–19 章 + 00_Notation 的吸收

> 审核员视角：**完备性**（五维吸收口径），而非正确性/接缝。
> 口径：吸收 = 把源书一切有价值内容用本书口吻重写进教材，覆盖五类——
> (a) 知识；(b) 讲解过程；(c) 分析过程；(d) 脉络；(e) 思想/洞见。
> 重点抓两类退化：① 知识点在但讲解/脉络/思想退化；② Handbook 有价值内容被整段跳过。
> 只读审核，未改任何 `.tex`/`refs.bib`。日期：2026-06-18。
>
> **与既有报告的分工**：`复核_Handbook_新部.md`/`复核_Handbook_新传感器.md` 审的是
> **正确性/接缝/独立性**（B/M/N 级问题）；本报告审的是**完整吸收度**（高价值是否缺失、
> 讲解/思想是否退化）。两者互补，不重复其 blocker/major 清单（仅在影响"完备性观感"时引用）。

---

## 0. 一页总判

| HB 章 | 落点 | 五维覆盖 | 判定 |
|---|---|---|---|
| ch11 惯性里程计 | `imu_model.tex`(进阶/可观/对准) + `vio.tex`(前沿) | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 高度完整 |
| ch12 腿式里程计 | `leg_odometry.tex`（新章） | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 高度完整（含 M1 速度公式错，属正确性） |
| ch13 学习式 SLAM | `learning_slam.tex`（新章） | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 高度完整 |
| ch14 可微体渲染地图 | `dense_mapping.tex` §可微渲染地图 | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 高度完整（**两份 recon 均未规划，却吸收到位**） |
| ch15 动态与可变形 | `dynamic_deformable_slam.tex`（新章） | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 范例级（§15.3 框架吸收超源书） |
| ch16 度量-语义 | `metric_semantic_slam.tex`（新章） | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 高度完整（含 B1 右扰动错，属正确性） |
| ch17 开放世界空间 AI | `open_world_slam.tex`（新章） | 知识✓ 讲解✓ 分析✓ 脉络✓ 思想✓ | ✅ 高度完整（§17.3/17.4 前瞻思想到位） |
| **ch18 空间 AI 计算结构** | **仅 §18.6 GBP → `slam_state_estimation.tex`** | 知识⚠️ 讲解❌ 分析❌ 脉络❌ 思想❌ | **❌ 重大缺口（全章思想未吸收）** |
| ch19 结语 | `epilogue.tex`（新章） | 精神素材✓ | ✅ 到位（未照搬格言，熔成本书收束） |
| 00_Notation | `front/notation.tex` + 各章首记号 note | — | ✅ 已统一（右扰动/下标式/学习层新符号） |

**一句话**：ch11–17、ch19 与记号**吸收得极好**，多处达到"用自己口吻把人讲懂 + 还做了源书没做的分析/连接"的范例水准；**唯一的重大完备性缺口是 ch18**——Davison《空间 AI 的计算结构》整章的**思想几乎全未吸收**，只有其 §18.6 高斯置信传播（GBP）的技术内核被吸收进 `slam_state_estimation.tex`（且吸收得很好），但该章作为一篇"少数学、多观点"的前瞻总纲，其**核心论点、脉络与设计哲学被整段跳过**。这恰恰命中任务预警的"前瞻章最容易被压得过薄而丢思想"。

---

## 1. ch18《The Computational Structure of Spatial AI Systems》(Davison) —— ❌ 重大缺口（本报告最重要发现）

### 1.1 为何这是缺口（且此前未被发现）
- **两份 recon 都没覆盖 ch18**：`Handbook_recon_C` 覆盖 ch08–12，`Handbook_recon_D` 明写覆盖 "ch13/15/16/17 + Epilogue + 00_Notation"。**ch14 与 ch18 都落在两份 recon 的分工缝隙里**。ch14 侥幸被 `dense_mapping.tex` 吸收到位（见 §5），**ch18 则成了真空**。
- **现状实测**（grep 全书 `parts/`）：ch18 的招牌概念命中数——世界模型 0、图处理器/IPU/Graphcore 0、近传感器处理/SCAMP/in-plane 0、Dennard/功耗墙/Moore 0、持续学习(continual learning) 0、性能度量/SLAMBench/Pareto 0、"空间 AI/Spatial AI" 仅 4（且全是 ch17 语境的工作定义，非 ch18）、"Davison" 5（均为系统署名或 GBP 引用）。
- **唯一被吸收的部分（且很好）**：ch18 §18.6 的**高斯置信传播（GBP）** 落在 `slam_state_estimation.tex` §`sec:est-gbp`（行 1217–1252），引 `davison2019futuremapping2`/`ortiz2021gbp`（即 FutureMapping2 原文，**非** Handbook ch18 本身）。该节质量很高：两类消息 + 信息形式相加/舒尔补的完整推导（`eq:gbp-belief`/`eq:gbp-factor-msg`）、树 vs 有环的精确/近似辨析、收敛定理（均值→MAP、协方差过自信，引 `weiss2001correctness`）、并附一个**前瞻 insight 盒**点到"存算一体（IPU 30× BA，引 `ortiz2020bundleipu`）+ 异步多机（Robot Web，引 `murai2024robotweb`）"。**ch18 §18.6 的技术内核因此可判为已吸收**。

### 1.2 ch18 被整段跳过的高价值内容（五维清单）

ch18 是全书唯一一篇**系统论述"SLAM 如何演进成 Spatial AI、其计算结构会长什么样"** 的总纲性前瞻章，含大量**思想/脉络/分析**（少公式但极有洞见）。以下是教材**完全没有**吸收的高价值内容：

**(e) 思想/洞见层（最该吸收、最该痛惜的丢失）**
1. **Spatial AI 的定义与两条假设（§18.1.2）**：① "应建一个**接近度量 3D 几何、局部一致、人类可理解**的通用持久场景表示"（排除纯 task-specific 表示与纯黑盒 world model）；② "Spatial AI 的有用性可由**少数性能指标**刻画"。——这两条是 Davison 给"空间 AI"下的**工程化定义**，与本书 ch17（`open_world_slam.tex`）给的"超越几何、编码语义、高层推理"的工作定义**互补且更偏系统/硬件视角**。ch17 那个定义 note 本可与 ch18 这两条假设**对照**，现完全缺失。
2. **"world model = scene representation"的论断（§18.1.2）**：ML 界新热的"世界模型"与机器人学几十年的"场景表示"**是一回事**；显式几何表示在效率/可组合/可解释/通用性上的优势论证。这是连接"SLAM 地图"与"具身智能世界模型"的**关键脉络洞见**，本书 `dynamic_deformable_slam.tex:501` 仅一句"推向世界模型"擦边，未展开。
3. **"每换一次表示，整个系统要重新设计"（§18.1.3）**：SLAM 不是往可用系统上**叠层**，而是**闭环重设计**（稠密图一旦有了就能反过来做直接跟踪、语义能改善稠密重建）。这是实时系统研究的**核心设计哲学**，全书无。
4. **状态估计 vs 机器学习的此消彼长 + 混合方法为何长期占优（§18.3）**：那个标志性论证——"若网络能从 100 张图出 3D 模型，第 101 张来了怎么办？总不能重跑整网；一旦承认需要长期表示与融合，就需要概率状态估计的工具"。这与本书 `learning_slam.tex`/`epilogue.tex` 的"几何不被取代、被包裹"立场**高度同源、可互证**，但 ch18 这个"增量融合"角度的犀利论证未被吸收。
5. **Sutton "Bitter Lesson" 用到 Spatial AI（§18.6 开头）**："押注能随算力扩展的通用方法"——为何该把**因子图本身**（而非由它导出的分布）当作主表示、就地消息传递。这是 GBP 节背后的**思想动机**，本书 `sec:est-gbp` 只讲了 GBP 的机理与硬件用途，**未吸收这层"赌算力/赌通用方法"的哲学动机**。
6. **生物视觉的类比克制（§18.2 末、§18.5.2）**：大脑 <10W 做全嵌入式语义几何视觉；但作者明确"不强求复刻大脑"，纯工程视角——这种**方法论自觉**有教育价值。

**(d) 脉络层**
7. **Slamcore 的 Levels 1/2/3（稀疏定位 / 稠密建图 / 语义标注）+ 终点是带物理仿真实例的场景图（§18.1.3）**：一条清晰的"SLAM→Spatial AI 能力阶梯"脉络，正好可串起本书 `dense_mapping`（L1/L2）→`metric_semantic_slam`（L3）→`open_world_slam`（场景图）的章序，**惜未用作贯穿脉络**。
8. **整体计算结构 6 条（§18.2）**：闭环、持久增量世界模型、"what is where"、局部度量、**聚焦式分层质量（focused max-quality + 残差质量层级）**、**前向预测式"普遍警觉"**（每个新像素都对照模型预测）。这套"闭环 vs 纯 VO/纯离线批处理"的对比是 Spatial AI 计算观的骨架。

**(c) 分析层 + (a) 知识层**
9. **处理器硬件全景（§18.4.1）**：Dennard Scaling 失效 → 功耗墙 → 为何嵌入式 Spatial AI 必须拥抱并行/异构/专用；SIMT/GPU 的来历与局限；Movidius Myriad / HoloLens HPU / Apple R1 / Meta ARIA 定制芯片；SpiNNaker / Graphcore IPU / Tenstorrent / Cerebras。
10. **传感器硬件全景（§18.4.2）**：事件相机作"去冗余"、**SCAMP 在平面内处理（in-plane / image-plane processing）**、3D 堆叠 CNN、云端算力"近乎无限免费、但通信/延迟不免费"。
11. **把 Spatial AI 图映射到硬件（§18.5）**：世界模型的**图结构**（近邻强边、远邻弱边、剪弱边得稀疏图——这其实是 `slam_state_estimation.tex` 信息矩阵稀疏性的**硬件对偶视角**！）；地图存储分布到核、维护处理（聚类/回环/正则/无监督发现新类）内建进图；实时环（标注/渲染/跟踪/融合/自监督）；**接口节点（类海马体）**；**"模型化事件相机"**（相机只回报与预测之差，是事件相机概念的极限推广）。
12. **持续学习在因子图内（§18.7）**：贝叶斯 ML = 状态估计同一套推断律；**GBP Learning**（把网络结构搭进因子图、非线性因子当神经元软开关、变量推断 = 权重学习、训练/测试无界）。这与本书 `learning_slam.tex` 的"把网络框进母方程"立场**惊人一致**，是该立场的**激进终点**，惜未吸收。
13. **性能度量（§18.8）**：为何 benchmark 对 SLAM 不充分、demo 文化、SLAMBench、Pareto 前沿、12 项度量（含 `On-device data movement, bits×millimetres` 这种极具洞见的指标）。

### 1.3 吸收建议（不在本报告动手，仅给落点）
- **不必新建一章**（ch18 偏前瞻随笔、与本书"可实现"基调略异，单立易显空泛）。**推荐两路织入**：
  1. **`epilogue.tex` 增一节"空间 AI 的计算结构（前瞻）"**：吸收 §18.1（Spatial AI 定义/两假设/world model 论断/每换表示重设计）、§18.2（计算结构 6 条）、§18.3（状态估计 vs ML、增量融合论证）、§18.7（持续学习于因子图）、§18.8（度量与 demo 文化）。这些是**思想/脉络/分析**，与结语的反思基调天然契合，且能与现有 §`sec:epilogue-open`（开放问题）、`epilogue` 的"几何被包裹"立场互证。**Levels 1/2/3 阶梯**可直接用作回望本部章序的脉络线。
  2. **`slam_state_estimation.tex` §`sec:est-gbp` 的硬件 insight 盒就地加厚**：补 §18.4（功耗墙/Dennard/异构专用芯片）、§18.5（世界模型图 = 信息矩阵稀疏性的硬件对偶；近传感器/SCAMP；模型化事件相机）、§18.6 的 Bitter-Lesson 动机。把现有"点到为止"升级为"有动机、有硬件全景"的一节。并把 GBP 节的引用从 `davison2019futuremapping2` 补挂一处 `\cite{carlone2026handbook}`（ch18），明确其前瞻论述的源。
- **记号无冲突**：ch18 不碰李群/四元数，纯系统/表示论述，织入零记号摩擦。
- **独立性提醒**：§18 满是 "we believe / we argue / we hypothesize"（Davison 第一人称立场），吸收时须**剥立场留事实/论点**，转成本书自己的综述口吻——这与 ch17 §17.4、ch19 的处理同法（本书在那两处做得很好，可照搬经验）。

---

## 2. ch11《Inertial Odometry for SLAM》—— ✅ 高度完整

**落点**：`imu_model.tex`（主干 + §`sec:imu-advanced` 进阶 + §`sec:imu-obs` 可观测性 + 初始对准）、`vio.tex`（§`sec:vio-frontier` 前沿、§VIO 可观性、滑窗/MSCKF/初始化/标定/同步）。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 测量/扩展模型（杆臂/scale/misalign/g-敏感）、运动学+捷联、连续↔离散噪声、Forster 预积分全套（增量/噪声分离/协方差递推/偏置一阶修正/残差）、Allan、4 维可观零空间、退化运动表、解析对准(11.7)、线/面 Plücker 进可观、扩展位姿/SE₂(3)/地球自转、连续/GP 预积分、学习 IO（IONet/RoNIN/TLIO/扩散）。**§11.2.3、§11.3.1.2、§11.5 三块"真增量"全部落地**。 |
| (b) 讲解 | ✓ | §`sec:imu-advanced` 的"欧拉把信号当台阶"动机 + 三种插值（矩形/梯形/GP）配重绘图 11.2–11.3；"先把旋转解耦出去"的分治讲法。 |
| (c) 分析 | ✓ | `ins:advanced-orthogonal`"重组与积分正交"；`ins:preint-steam`**连续旋转预积分与 STEAM 的同与异**（马尔可夫稀疏核 vs 光滑稠密核、由窗口长短决定）——此分析**比源书更深**；`ins:beyond-euler`"何时需要超越欧拉"。 |
| (d) 脉络 | ✓ | 骨干→进阶的"磨锐而非推翻"脉络；学习 IO 三类被替换零件（去噪/直接回归/概率偏置）排进"框进母方程"统一坐标。 |
| (e) 思想 | ✓ | "积分是线性算子，GP 经线性算子仍是 GP"的洞见；学习方法"泛化有限、过拟合训练分布"的批评与 `ch:learning` 同源；本体感受里程计前指 `ch:leg`。 |

**完备性结论**：无高价值缺失，无讲解/思想退化。`复核_新传感器` 指出的问题（线/面统一零空间显式式延后到 vio）属编排选择，非缺失。

---

## 3. ch12《Leg Odometry for SLAM》—— ✅ 高度完整

**落点**：`leg_odometry.tex`（新章，`ch:leg`），结构完整：为何需要→参考系/状态→腿运动学(fk/J 自包含)→关节传感→相对位姿(接触系静止)→速度估计(零速度约束)→接触估计(摩擦圆锥/CoP/GRF 反演)→因子图平滑(编码器噪声传播 + 正运动学因子 + 接触预积分因子 + 速度预积分因子)→接触辅助 InEKF(接触点装进状态群)→外感受融合→开放问题→数值例。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 式 12.1–12.38 几乎全吸收（fk/J、相对位姿、零速度、摩擦圆锥/CoP、GRF 反演、三类预积分因子、InEKF 增广群）。 |
| (b) 讲解 | ✓ | "把腿读成里程计测量"的递进；fk/J 标注"自包含"独立可读。 |
| (c) 分析 | ✓ | `ins:leg-preint-iso`**接触/速度预积分 = IMU 预积分换种高频测量**的同构表（全书缝合范例）；InEKF"群仿射⇒误差自治"论证。 |
| (d) 脉络 | ✓ | `ins:leg-three-lines`**汇 InEKF/SE₂(3)/预积分三线**；接 `paper:inekf` 的前向承诺、接 P4 足式控制 fk/J 互为表里。 |
| (e) 思想 | ✓ | 本体感受"亚百分比误差"为何可能；刚性假设失效/学习接触估计前沿。 |

**完备性结论**：无高价值缺失。`复核_新传感器` 的 **M1**（`eq:leg-vel-meas` 世界系帧不一致，且被本章下游公式反证）与 **M2**（雷达"三问入图"未被回扣）是**正确性/接缝**问题，不影响完备性判定，但 M1 踩中本章招牌结论，宜尽快修。m3（GRF 反演偏黑盒、punt 给 Featherstone）是**唯一轻微的自包含让步**，属选学小节、可接受。

---

## 4. ch13《Boosting SLAM with Deep Learning》—— ✅ 高度完整

**落点**：`learning_slam.tex`（新章，`ch:learning`）。三段对应源章三主线：替换组件 → 可微分 BA/DROID → pointmap 全栈。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 学习式深度/位姿回归、MonoRec/BTS（密化/体密度场）、SuperPoint/SuperGlue/LoFTR、RAFT、可微分 BA（`eq:learn-diffba-cost` 网络出 e_θ+Σ_θ）、DROID（相关金字塔→Conv-GRU→δf+w→展开 BA）、DPVO（选读）、DUSt3R（Siamese ViT+置信加权回归+全局对齐）、MASt3R（选读）。 |
| (b) 讲解 | ✓ | "把范式倒过来"无；但有"光流即测量→与重投影 BA 同构"的递进；前置自测以"那个 BA 怎么会可微"切入。 |
| (c) 分析 | ✓ | "RAFT 是从数据学到下降方向的迭代优化器"；位姿回归累积漂移与经典 VO"一度之差"同源、且系统性不如几何深植；D3VO 光度不确定性 = 给 Σ_i 注入学习式权重"已一只脚踏进可微 BA"。 |
| (d) 脉络 | ✓ | "替换组件→端到端→pointmap"按**介入深度**递增；桥头堡定位、扣 `sec:intro-frontier` 弧线。 |
| (e) 思想 | ✓ | **统一立场"把学习框进母方程"**（残差/权重/状态表示逐个交给网络，优化即推断不变）；"让网络喂养几何而非替代几何"。 |

**关键完备性亮点**：recon D 点名的**最大自包含缺口——§13.3 反复 punt 到"Handbook ch4"求 GN 的导数**，本书**已彻底补齐**：可微优化（双层优化/隐式微分/HVP/展开式反传）在 `nonlinear_optimization.tex` §`sec:nlopt-diffopt` **自包含建好**，本章 `\cref` 过去、只用结论。这是把跨章 punt 转成本书内部前向引用的范例，**完备性无悬空**。recon 担心的 §13.6/13.7 系统罗列（ventriloquize 风险），本章压成"选读 + 前沿趋势"处理得当。

---

## 5. ch14《Map Representations with Differentiable Volume Rendering》—— ✅ 高度完整（两份 recon 均未规划，却吸收到位）

**重要发现**：ch14（NeRF/3DGS，Matsuki & Davison）**与 ch18 一样落在两份 recon 的分工缝隙**（recon C 止于 ch12，recon D 始于 ch13 但跳过 ch14、直接到 ch15）。**但与 ch18 不同，ch14 被 `dense_mapping.tex` 主动吸收且质量极高**——这说明生产阶段在 recon 之外补足了 ch14，却漏了 ch18。

**落点**：`dense_mapping.tex` §`sec:dense-radiance`（行 1240–1364）："可微渲染与 NeRF → 3DGS → 用于 SLAM"。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 体渲染积分 `eq:dense-nerf-integral` + 求积离散 `eq:dense-nerf-quad`（透射率/不透明度/alpha 合成）、光度损失、位置编码、显式数据结构加速（Plenoxel/DirectVoxGo/Instant-NGP）、神经场(NeuS/VolSDF/UNISURF)、3DGS（协方差因子化 `eq:dense-gs-cov`=RSS^T R^T、投影 `eq:dense-gs-proj`、深度排序 alpha 合成、SH、致密化/剪枝、tile/基数排序/视锥剔除）、用于 SLAM（iMAP/NICE-SLAM/NeRF-SLAM/MonoGS/LoopSplat/Photo-SLAM/RTG-SLAM）、数据结构取舍表、遗忘问题。 |
| (b) 讲解 | ✓ | "把范式倒过来：用渲染损失优化 3D 表示"的统领动机；局部记号 note 钉死 σ（体密度 vs 标准差）、V_cw（避让 TSDF 权重 W）。 |
| (c) 分析 | ✓ | insight**"alpha 合成与 OctoMap 对数几率同根"**（占据=吸收概率）；insight**"遍历图元为何比沿射线行进快"**（= surfel 稀疏性在渲染维度的翻版）；pitfall**"以为可微渲染地图已能取代经典稠密 SLAM"**（实时性/跟踪精度/静态房间级三硬伤 + 前馈预测新方向）。 |
| (d) 脉络 | ✓ | "经典稠密表示 → 可微渲染地图"的**单一自然演进**收束；NeRF 接回隐式曲面/TSDF/MC、3DGS 略属性退回点云，"没跳出本章四象限、只装上可微渲染引擎"。 |
| (e) 思想 | ✓ | 可微渲染的设计哲学（梯度从 2D 回流 3D、未定表面即可给梯度）；与经典法互补而非替代。 |

**完备性结论**：无高价值缺失，**反而是把"前沿稠密表示"无缝缝进经典稠密章的范例**。`复核_感知weave.md` 若已审此节，本报告与之一致。

---

## 6. ch15《Dynamic and Deformable SLAM》—— ✅ 范例级（§15.3 框架吸收超源书）

**落点**：`dynamic_deformable_slam.tex`（新章，`ch:dynamic`）。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 三轴刻画框架(§15.1)、动态物体因子图(式 15.1–15.4 完整，`eq:dyn-reproj/couple/increment`)、稠密动态/运动分割、**自补形变能量泛函**(ED 蒙皮/`def:dyn-arap` ARAP/Killing/等距/visco)、浮动地图歧义、SfT/NRSfM、DynamicFusion 式非刚性融合、长期/终身/change-aware(§15.3)、时序场景理解(MoD/频率图/学习式)。 |
| (b) 讲解 | ✓ | "动态 = 经典鸡生蛋再拧紧一圈（还要判每个观测属静/动）"的动机递进。 |
| (c) 分析 | ✓ | **大量超源书的 insight**：`浮动地图歧义=尺度歧义的近亲`（同病同药）；`absence of evidence ≠ evidence of absence` insight 把它连到占据栅格**占据/空闲/未知三态**（"未知=无证据、空闲=有缺席证据"，**源书未点破这层**）；`局部一致性可识别性原理` 连到回环几何验证与动态外点多视一致；两个 pitfall（边缘化"烤进"错误、"半张桌子"语义一致性）。 |
| (d) 脉络 | ✓ | 四条线（剔除/跟踪/非刚性/change-aware）统一为"回答同一个数据关联问题的不同侧面"；推向"世界模型"前指 `ch:openworld`/`ch:epilogue`。 |
| (e) 思想 | ✓ | `长期 change-aware 仍是母方程+物体级因子`（从短期跟踪到长期变化一以贯之）；SE(3)-等变描述子 = 鲁棒数据关联的现代答案、接 `ch:loop`。 |

**关键完备性亮点**：recon D 点名的**最高风险——形变能量泛函(ARAP/ED/Killing)Handbook 全 punt、零数学式**，本书**已自补**（`sec:dynamic-deformable` 专设"自补"小节 + `def:dyn-arap` 完整能量定义）。§15.3 这个"全新概念框架"不仅没被压薄，**吸收得比源书更透**（占据三态的连接是本书自己的增值）。`复核_新部` 的 **M1**（`eq:dyn-couple` 去齐次 h^{-1} 除零）、**M2**（两处"Handbook 提出/给出"单书叙述依赖）是正确性/独立性问题，不影响完备性，但 M2 是本部唯一的独立性瑕疵，宜改本书直述。

---

## 7. ch16《Metric-Semantic SLAM》—— ✅ 高度完整

**落点**：`metric_semantic_slam.tex`（新章，`ch:semantic`）。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 物体路标 `def:semantic-object` ℓ=(σ,q,λ,s)、四种形状表示（语义关键点/二次曲面椭球/网格/隐式 SDF）各自观测因子、对偶二次曲面解析投影成对偶圆锥、离散-连续混合 MAP（DC-SAM/EM 软关联/连续松弛）、多类贝叶斯 log-odds 体素(`h∈R^{K+1}`+softmax+加性更新)、多类 OctoMap、网格语义+PGMO、分层 3D 场景图 + treewidth 界。**四块硬核 + 离散-连续 MAP 框架全落地**。 |
| (b) 讲解 | ✓ | 前置自测五问把"点路标→物体路标""二值→多类 log-odds""离散+连续组合爆炸""分层为何更快"逐一逼出。 |
| (c) 分析 | ✓ | 多类 log-odds **= 二值 `eq:dense-octo-logodds` 的多类推广（K=1 退化为 sigmoid）**；treewidth 接 `thm:elimination`；用本书右扰动对 T、q 求导。 |
| (d) 脉络 | ✓ | 统一立场"经典 SLAM 两端各升一维"；接 `tab:dense-maptypes` 锚点；闭集→开集 punt 给 `ch:openworld`（沿用源书递进）。 |
| (e) 思想 | ✓ | "物体级 = 同一后端的新残差"；`metric_semantic:640` 明写"把离散语义整体换成连续语言特征，闭集 SLAM 就过渡到开放世界"——为 ch17 铺脉络。 |

**完备性结论**：无高价值缺失。`复核_新部` 的 **B1**（`:287` 物体位姿右扰动雅可比误写成左扰动、漏 q(·)q^{-1} 共轭，且踩中本章"用本书右扰动统一"卖点）是**必修的正确性 blocker**，但不属完备性范畴（知识点在、只是推导一步算错）。N2/N4（log-odds 维度冗余表述、mesh IoU 残差最小值非 0）属表述精度。

---

## 8. ch17《Towards Open-World Spatial AI》—— ✅ 高度完整（前瞻思想到位）

**落点**：`open_world_slam.tex`（新章，`ch:openworld`）。

| 维度 | 覆盖 | 证据 |
|---|---|---|
| (a) 知识 | ✓ | 闭集 vs 开集两范式区分、基础模型最小背景(特征/嵌入/余弦、CLIP 共享空间、SAM)、开放词汇建图(地图元素 m_k=(p_k,f_k)、range-feature 观测、ConceptFusion 全局/局部特征加权融合、零样本查询/多模态查询)、物体级地图与场景图(关联/融合、语言描述、HOV-SG 分层)、隐式(LERF/LangSplat 选读)、**任务驱动表示 Clio 信息瓶颈**(式 17.8/17.9 + Agglomerative IB)、§17.4(Map Prompting/Map API、是否需要地图、VLA 谱系)。 |
| (b) 讲解 | ✓ | "从闭集到开集最后一步"立锚 + 立"空间 AI 工作定义"；基础模型背景"只取读懂后文所需最小必要内容"。 |
| (c) 分析 | ✓ | **range-feature 接 range-category**（ch16 平行推广）；**信息瓶颈接本书互信息 `eq:mutual-info`**；insight**JS 散度 = `eq:gauss-kl` 的 KL 散度对称化**；pitfall"信息瓶颈两处误差源"。 |
| (d) 脉络 | ✓ | 闭集(ch16)→开集递进；§17.4 接 preface 具身承诺 + `sec:intro-need`"是否需要 SLAM"首尾呼应。 |
| (e) 思想 | ✓ | **§17.4 前瞻思想吸收到位**（任务预警的重灾区）：长上下文 VLM/OpenEQA/Mobility VLA 的**事实**陈述（剥立场）、VLA 谱系(GATO/PaLM-E/RT/ACT/OpenVLA/π0/扩散/Octo/RL)、note"VLA 学的是策略不是地图"、**收束以本书口吻明确给出"显式结构与学习策略互补"的立场**（区别于文献事实）。 |

**关键完备性亮点**：recon D 点名 §17.4 是"ventriloquize 重灾区（满是 we argue）"，本书**stance-stripping 做得很好**——只陈述可复核事实、立场只在节末以本书口吻明标。Clio 信息瓶颈这个"数学含金量最高"的点被当**全章理论核心**严肃讲授并接回本书信息论语言。**§17.3/17.4 的前瞻思想未被压薄**。

---

## 9. ch19《Epilogue》—— ✅ 到位

**落点**：`epilogue.tex`（新章，`ch:epilogue`）。

- ch19 源是资深学者**格言集**（"Trust the math"/"SLAM is not solved, 2-3 steps ahead"/"clear notation is the best tool"/"extract truth from conflicting sensors"/"robot perception is a mirror"等）。
- 本书**未照搬、未逐条"某人说"**，而是择其与主线契合者熔成**本书自己的五条贯穿思想**（一张因子图一条母方程 / 几何是免费精确先验 / 对不确定性诚实 / 清晰记号是发现错误最好工具 / 从矛盾测量萃取真相）+ 四个开放问题（还需不需要地图 / 终身一致性 / 不确定性贯彻到学习时代 / 鲁棒性效率保证）。与 `sec:intro-need`/`sec:intro-frontier`/`tab:intro-eras` 首尾扣合。
- **完备性结论**：ch19 的**精神**吸收到位，是处理"格言类源"的范例。`复核_新部` 的 **N5**（延伸阅读把"结语之源"挂 `\cite{teed2026boosting}`=ch13 键、疑似张冠李戴 ch19）是**引用键正确性**问题，需人工核 bib；**与完备性无关，但顺带提示**：若 `refs.bib` 无独立 ch19 条目，应补或改述。

---

## 10. 00_Notation —— ✅ 已统一

- Handbook 00 仅给底层李群/概率记号（$\wedge/\vee$、$\exp/\log$、$\hat{}/\check{}$、$\mathcal{N}$、$\mathcal{GP}$、马氏范数、帧上下标式 $\mathbf{R}_a^b$）。
- `front/notation.tex` 已登记**两处关键统一**：① 帧下标方向（本书 $\mathbf{R}_{ab}$ vs Handbook $\mathbf{R}_a^b$，notation 明写"此约定与部分文献相反"）；② **右扰动为主线**。底层记号本书与 Handbook 同源（$\hat{}/\check{}$、$\wedge/\vee$、$\mathcal{N}$、马氏范数全一致）。
- **学习/语义层新符号**（$f_\theta$/特征 ψ/pointmap/物体路标 ℓ/多类 log-odds h/互信息 I）已在 P5 各章首"本章记号"note 登记并显式避让已占符号（open_world 用 ψ 避让 dense 章 `f`；dense 章 §可微渲染用 V_cw 避让 TSDF 权重 W；semantic 用 s^sh 避让尺度 s）。
- **完备性结论**：记号统一无缺口。

---

## 11. 总结：完备性结论与待办

### 11.1 五维覆盖总判
- **ch11/12/13/14/15/16/17/19 + 00_Notation：吸收完整，五维到位**，多处达到"用自己口吻把人讲懂 + 做了源书没做的分析/连接"的范例水准（尤以 ch15 §15.3 占据三态连接、ch11 STEAM 对照、ch14 alpha-合成↔log-odds、ch16 多类⊃二值、ch17 JS↔KL 为最）。**无"知识点在但讲解/脉络/思想退化"的章**（ch11–17、19 的讲解与思想反而常**超出**源书）。

### 11.2 ❌ 唯一高价值缺失：ch18
- **ch18《空间 AI 的计算结构》整章思想未吸收**，仅 §18.6 GBP 内核落在 `slam_state_estimation.tex`（且很好）。被整段跳过的高价值内容见 §1.2 共 13 条，核心痛失：
  1. **Spatial AI 的工程化定义 + 两条假设**（§18.1.2）——与 ch17 工作定义互补的系统/硬件视角；
  2. **"world model = scene representation"论断**与显式表示的优势论证（§18.1.2）；
  3. **状态估计 vs ML、"第 101 张图怎么办"的增量融合论证**（§18.3）——与本书"几何被包裹"立场互证；
  4. **持续学习于因子图 / GBP Learning**（§18.7）——本书"把学习框进母方程"立场的激进终点；
  5. **处理器/传感器硬件全景 + 世界模型图=信息矩阵稀疏性的硬件对偶**（§18.4/18.5）；
  6. **Levels 1/2/3 能力阶梯**（§18.1.3）——本可用作贯穿本部章序的脉络线；
  7. **性能度量与 demo 文化、bits×mm 指标**（§18.8）。
- **建议**（§1.3）：不新建章；织入 `epilogue.tex`（思想/脉络/分析层）+ 加厚 `slam_state_estimation.tex` §`sec:est-gbp` 硬件 insight（硬件全景 + Bitter-Lesson 动机 + 补挂 ch18 引用）。剥 Davison 第一人称立场、转本书综述口吻（照搬 ch17/ch19 的 stance-stripping 经验）。

### 11.3 ⚠️ 顺带提示（属正确性/独立性，非完备性，但影响"完整著作"观感）
（详见 `复核_Handbook_新部.md`/`复核_新传感器.md`，本报告不重复，仅汇总以免遗漏）
- **必修正确性**：ch16 B1（`metric_semantic_slam.tex:287` 物体右扰动雅可比误成左扰动，踩中卖点）；ch12 M1（`leg_odometry.tex:362` `eq:leg-vel-meas` 世界系帧不一致，被本章下游反证）；ch15 M1（`eq:dyn-couple` h^{-1} 除零）。
- **宜修独立性**：ch15 M2（`dynamic_deformable_slam.tex:128/145` 两处"Handbook 提出/给出"单书叙述依赖——本部唯一独立性瑕疵）。
- **接缝**：ch12/event M2（雷达 `ins:radar-factor-recipe`"三问入图"统一范式未被 event/leg 回扣）。
- **引用键**：ch19 N5（epilogue 延伸阅读"结语之源"疑似误挂 ch13 键 `teed2026boosting`，需人工核 bib 是否有独立 ch19/Handbook-epilogue 条目）。
- **缺口顺带提示**：ch18 的 GBP 节当前引 `davison2019futuremapping2` 而非 `carlone2026handbook`(ch18)——若按 §1.3 织入 ch18 论述，须补挂 ch18 引用键（确认 `refs.bib` 有 `davison2026computational` 一类键，否则用统一 Handbook 键并注明涵盖 ch18）。

### 11.4 一句话交代
**本批 9 章 + 记号，8 章（ch11–17、19）与记号吸收完整且常超源书；唯 ch18 整章思想被跳过（仅 GBP 内核落地），是本次审核发现的唯一重大完备性缺口，建议织入结语 + 加厚 GBP 节的硬件/动机论述。** ch14 与 ch18 同处两份 recon 的分工缝隙，ch14 已被 `dense_mapping` 补足、ch18 被遗漏——这条"分工缝隙"是缺口的成因，提示后续若再分派 recon 须显式点名每一章。
