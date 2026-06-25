# 复核报告：SLAM Handbook 感知类 weave-in 的质量与缝合

> 对象：`parts/P2_slam/` 七章中本轮把 The SLAM Handbook 内容**织进**现有感知章的新增部分——
> `dense_mapping.tex`、`lidar_slam.tex`、`point_cloud_processing.tex`、`imu_model.tex`、`vio.tex`、`camera_model.tex`、`visual_odometry.tex`。
> 审查四维：① 无接缝（织入 vs bolt-on）② 无重复（跨章去重，重点）③ 硬核数学正确 ④ 独立性 + 记号 + 保护标签。
> 方法：主审独读 dense_mapping 全章 + 三路并行 agent 读其余六章；主审独立验算 NeRF/3DGS/PGBA 级数/Schur 等数学，并核查所有「真重复 vs 既有 vs 本轮引入」的归属与保护标签语义。
> 背景：全树静态已干净（label/cite/环境配平），本报告**不查机械问题**，专注内容质量、缝合、去重。
> 约束：只读，未改任何 `.tex` / `refs.bib`。日期：2026-06-18。

---

## 严重度计数

| 级别 | 数量 | 条目 |
|---|---|---|
| **blocker** | 1 | B1（PGBA 引用错配到 √BA 论文） |
| **major** | 3 | M1（三种测距物理跨文件近重复，本轮引入）、M2（vo 标准预积分整段重推，既有）、M3（LOAM 残差两章重复，既有） |
| **minor** | 8 | m1–m8（见下） |

**缝合**：基本到位，四象限骨架真正贯穿全章，NeRF/3DGS 被正确框为「经典→可微渲染」自然演进。
**去重（本轮新内容）**：到位——先进预积分（vio→imu 指针）、学习型特征（→`ch:learning`）、FMCW 推导（→`ch:radar`）三处委派**都真做了、未就地重复**。
**数学**：本轮新织入内容**无错**（NeRF alpha 合成、3DGS 协方差/投影、Power BA 级数、Schur、FMCW 差频/多普勒、ToF、GP 残差均经独立验算）。
**遗留 major 中 2 条（M2/M3）属原始吸收阶段的「自包含」旧设计，非本轮 Handbook 织入引入**——列出供全书层面统一处置，不应记在本轮头上。

---

## BLOCKER

### B1. PGBA 的引用键指向了错误的论文（√BA），语义错配
`visual_odometry.tex:1604`
> 位姿图束调整（Pose Graph Bundle Adjustment, PGBA）`\cite{demmel2021pgba}`——PGO 的一种自适应加强版……却在约束里完整保留 BA 的光度不确定度。

但 `refs.bib:2099-2102` 的 `demmel2021pgba` 实际题录是 **"Square Root Bundle Adjustment for Large-Scale Reconstruction"（Demmel, Sommer, Cremers, Usenko, CVPR 2021, pp. 11723–11732）**——这是 **√BA / QR 边缘化数值稳定性**论文，**不是** Pose-Graph-BA 论文。正文描述的 PGBA 概念（只更新位姿、约束里保留全 BA 光度不确定度、用以统一 LSD/DSO）来自 Demmel/Usenko 系的**另一篇**工作，与该 √BA 论文主题不符。
- **独立验证**：`grep` 确认 refs.bib:2101 标题为 "Square Root Bundle Adjustment"；`weber2023powerba`（refs.bib:2104，Power BA，正文 :1587 引）题录**正确无需动**。
- **旁证**：暂存库 `docs/Handbook_新增文献_待并.bib:693` 自带批注「demmel2021pgba 题录待按 HB [1135] 核验」——团队已知此条未核实。
- **影响**：读者按引用查到主题不符的文章，PGBA 这一织入点可追溯性断裂；正属独立性标准要避免的「引用幌子」。
- **修复**：核对 Handbook [1135] 的真实出处，把 `demmel2021pgba` 题录改为真正的 PGBA 论文；或新增独立 key（如 `demmelXX pgba`）指向正确文献，√BA 那条另留它用或删。同时把正文「PGO 的一种自适应加强版」的「自适应」改成更准确的机理描述（见 m7）。

---

## MAJOR

### M1.【本轮引入】三种测距物理（ToF/AMCW/FMCW）在两文件跨章近重复
`lidar_slam.tex:152`（新增 `\paragraph{三种测距物理：ToF、AMCW、FMCW}`，带 `\label{par:lidar-ranging}`）
`point_cloud_processing.tex:110`（`\paragraph{点从哪来：LiDAR 的测距原理}`）

两处讲**同一套**三机制，且重合到修辞层：
- 同一 ToF 公式 `$d=\tfrac12 c\Delta t$`、同一「对环境光敏感→压信噪比→限精度与频率」论述；
- 同一「AMCW 相位 / FMCW 频移测速」二机制；
- 同一「FMCW 在毫米波雷达里**反客为主/主心骨**、完整推导见 `\cref{ch:radar}`」修辞与转引；
- 连「四类传感机构（机械式/扫描式固态/闪光式/Risley）」都各列一遍（lidar:149 vs point_cloud:110）。

两章都是点云感知前端、读者通常连读，第二遍会明显察觉。**这是本轮织入引入的真接缝/去重问题**（与既有 LOAM 重复不同，见 M3）。
- **修复**：保留 `lidar_slam.tex:151-152`（有专门小节标题、与 RAE/退化场景叙事咬合更紧），把 `point_cloud_processing.tex:110` 的三机制压成一句指引——「LiDAR 测距机制 ToF/AMCW/FMCW 见 `\cref{par:lidar-ranging}`；本章只把（FMCW 的）速度当作点的可选属性字段」，并删去 point_cloud 侧的四类机构复述（已在 lidar:149）。

### M2.【既有，非本轮】vio 把标准预积分整段重推，与 imu 近乎逐字重复
`visual_odometry.tex:232-369`（`sec:vio-preint`：`sec:vio-preint-delta` / `sec:vio-imu-factor`）
vs `imu_model.tex:470-560, 720-744`（`sec:imu-preint`）

证据：两处各有同名 derivation「旋转/速度/位置增量的噪声分离」（vio:266,286 ≈ imu:503,520）、同一 boxed 测量模型（`thm:vio-preint-model` vio:305 ≡ `eq:imu-preint-model` imu:537）、同一组残差（`thm:vio-imu-residual` vio:354 ≡ `eq:imu-residual` imu:732），连两条 SO(3) 恒等式都各抄一遍。vio 虽在 :179/:209 引 `\cref{ch:imu}` 取测量模型，但预积分主体是**独立重推**而非指针。
- **归属**：vio 头注释（vio:5-9）确认 VINS-Mono「全文」+ Forster 预积分是**原始吸收阶段**写进 vio 的；**非本轮 Handbook 织入引入**。与本轮「先进预积分用指针指向 imu」形成鲜明反差——先进预积分去了重，标准预积分却整段重复，是旧「每章自包含」设计的产物。
- **修复（需主审定调，不建议本轮强改）**：
  - (a) 若坚持 vio 自包含：现状可接受，但应在 `sec:vio-preint` 开头加一句「本节为自包含重述 `\cref{sec:imu-preint}`，IMU 章首推；熟练者可跳读」，把「重复」显式标注为「有意复述」，与 frontier 指针风格统一。
  - (b) 若彻底去重：把 vio 两个 derivation 降为指针 + 只保留 VINS 四元数形式（vio 独有价值），正文引 `eq:imu-preint-model` / `eq:imu-residual`。改动大、超「审新织入」范围，宜记入审计待办。

### M3.【既有，非本轮】LOAM 边/面残差闭式在两文件完整重复
`lidar_slam.tex:425-433`（`eq:lidar-loam-de` / `eq:lidar-loam-dh`）
vs `point_cloud_processing.tex:1257-1263`（`eq:pc-loam-dE` / `eq:pc-loam-dH`）

逐字相同的点到线/点到面闭式，连「分子=平行四边形面积、商=垂距」的几何解说都几乎一字不差；曲率定义、LM 更新、PCA 边/面判别同样重复。
- **归属（已独立核查头注释）**：`point_cloud_processing.tex:6-10` 头注释明列「Zhang LOAM」为**原始吸收阶段**的经典文献；`lidar_slam.tex:5-7` 亦在原始阶段吸收 LOAM。本轮织入（测距/CT-ICP/多机器人）**未触碰 LOAM 数学**。故此条对本轮「去重是否到位」无关，但红线 2 点名 LOAM，须上报全书层面统一处置。
- **修复（全书层面）**：以 `point_cloud_processing.tex` 的 `sec:pc-loam` 为 LOAM 公式正本（在「点云配准算法族」语境最自洽），`lidar_slam.tex` 的 `sec:lidar-loam-dist` 改为 `\cref{eq:pc-loam-dE,eq:pc-loam-dH}` 引用 + 只保留「LOAM 把它放进双算法/去畸变」的系统级增量。**勿在本轮单方面改。**

---

## MINOR

### m1.【本轮】`lidar_slam.tex` 内 ToF 公式 `$d=\tfrac12 c\Delta t$` 出现两次
`lidar_slam.tex:111`（动机段）与 `:152`（新增三机制段）相隔 ~40 行各写一遍同一公式。属本轮新段与既有动机段的章内小重叠。
- 修复：L152 改为「飞行时间（TOF，见 `\cref{sec:lidar-why}` 的 `$d=\tfrac12 c\Delta t$`）」回指，不再重列。（与 M1 一并处理。）

### m2.【本轮，措辞】CT-ICP 的缝合可再咬紧一句（但非 bolt-on）
`lidar_slam.tex:545, 1208`
说明：CT-ICP **已织入**——L545 显式接 `sec:est-steam`（STEAM 连续时间主线）与 `sec:imu-advanced`，L1208 又把「连续时间配准/逐点流式」两支线对照「本章 LOAM→FAST-LIO 主线」。**这是真缝合，不是 bolt-on**；「推导属广度，留给所引文献」是合理的广度取舍。唯一可加强：把 CT-ICP 与本章已完整推导的 LOAM 匀速插值（`eq:lidar-loam-interp`）显式对照一句——「CT-ICP = 把 LOAM 单一 `$\mathbf{T}^L_{k+1}$` 匀速插值升级为帧始/帧末两位姿插值并与配准联合优化」，让它长在已有公式上。优先级低。

### m3.【既有/越界，仅提示】FAST-LIO 状态向量疑似转置笔误
`lidar_slam.tex:765`：状态 `$\mathbf{x}=[{}^G\mathbf{R}_I^\top\ {}^G\mathbf{p}_I^\top\ \dots]^\top$`，第一块 `${}^G\mathbf{R}_I^\top$` 的转置可疑——流形 `$\SO(3)\times\mathbb{R}^{15}$` 应存旋转 `${}^G\mathbf{R}_I$` 本身（其余块 `$^\top$` 是为列排版，旋转块不应再带 `$^\top$`）。属既有 FAST-LIO 段、非本轮织入，越出范围，顺手提示主审核对。

### m4.【跨章一致性】Schur 补推导在 camera/vo 间近重复
`camera_model.tex:893-923`（`thm:ba-schur` + insight）vs `visual_odometry.tex:1555-1585`（`thm:vo-schur` / `thm:vo-schur-ba`）。同一 Schur 三步公式 + 同一「local BA 稠密 / full BA 稀疏」权衡（cam:922 ≈ vo:1584），标签不同但内容等价；`ch:nlopt` 还有第三处母本。**主审判断**：cam 的 `thm:ba-schur` 作「相机封装成观测模型→对接 BA」的收束保留其**存在**有理由，但完整再推一遍代数 + 重复稠密/稀疏权衡与 vo 冗余。涉及 camera 是否本轮重写过，倾向归入「可精简」而非强制。
- 修复（可选）：cam 一侧 Schur 只留「BA 法方程因 `$\mathbf{G}=[\mathbf{G}_1\,\mathbf{G}_2]$` 呈箭头结构、可 Schur 边缘化、复杂度 `$O(K^3+K^2M)$`」结论 + `\cref{sec:vo-schur}` 指引，删逐步证明（cam:912-919）与重复权衡（cam:922）。vo:1553 标注「独立记录完整代数」，更像指定母本。

### m5.【跨章一致性】「BA 起源 / von Gruber / 摄影测量」历史叙事在 camera/vo 重复
`camera_model.tex:120` vs `visual_odometry.tex:101`。同一史实（von Gruber + BA 起源 + 多视图重建结构与运动）两处叙述、措辞高度同构。vo 一侧更完整（加 Schmid、Building Rome、SfM 谱系），是「视觉运动估计简史」正题；cam 一侧是「从摄影术到针孔模型」引子。属可接受的轻度重叠，但 von Gruber+BA 一句几乎重复。
- 修复（可选）：cam:120 落点收紧到摄影测量与针孔模型谱系，BA 起源改为「BA 数学框架同期成形（`\cref{ch:vo}` 简史详述）」，避免与 vo:101 撞史。

### m6.【本轮残留独立性】vo 一处 narration_dependence
`visual_odometry.tex:770`：「……=重投影误差。**这正是 Handbook 的 pose-only BA / PTAM 跟踪线程所解的式子。**」——把本书自推的标准式（`eq:vo-pnp-ba`）说成「Handbook 的」，且**无 `\cite`**，是单书叙述依赖残留。
- 修复：改为「这正是**仅位姿 BA**（pose-only BA）/ PTAM 跟踪线程所解的式子」，去掉「Handbook 的」这一所属化措辞（pose-only BA 已是通用术语）。
- 注：其余含「Handbook」处（cam:104/106/204/268/325、cam:1114 符号表、vo:93 等）均为合法跨书记号对照（带 `\cite{carlone2026handbook}` 或仅作记号注），**不构成** punt/ventriloquize/narration；`sec:cam-modern` 全节自包含、无「详见 Handbook」。叙述依赖清理**基本到位**，仅此一处残留。

### m7.【措辞，伴随 B1】PGBA「自适应」一词含混
`visual_odometry.tex:1604`：「PGO 的一种**自适应**加强版」——「自适应」所指未点明（自适应权重？保留哪些约束？）。修复 B1 时一并把「自适应」替换为准确机理（如「在相对位姿约束上挂载由 BA 边缘化得到的完整信息矩阵」）。

### m8.【跨章记号一致性，不影响正确性】若干局部记号撞字母 / 多写法
逐条登记，均不影响正确性，多数属「合理的局部呼应/各章已局部声明」：
- **imu `$\hat{\mathbf a}$`**：`imu_model.tex:1133`（`eq:rot-corrected-acc`）把「旋转修正后的加速度」记 `$\hat{\mathbf a}_k$`，而 vio 章 `$\hat{\mathbf a}$`（vio:209,377,466）表「去偏置测量」、imu 本章 `$\hat{\,}$` 又用于「新估计偏置 `$\hat{\mathbf b}$`」。建议在 `eq:rot-corrected-acc` 加脚注澄清此处专指「旋转修正后」，避免跨章读者绊。
- **相机系点三写法**：camera 用 `$\mathbf{x}^c$`/`$\mathbf{z}_c$`，vo 用 `$\mathbf{P}'$`/`$\mathbf{P}^c$`；各章已在 note 声明且 cam:104 列「Handbook `$\mathbf{x}^c$`」对照，非错误，仅跨章摩擦。可在桥接处加半句对照消解。
- **lidar `$\rho_j$`（采样时刻）**：CT-ICP 段（lidar:545）用 `$\rho_j$` 表点采样时刻，与同章 `$\boldsymbol\rho$`（RAE 坐标/李代数平移块）、`$\rho_{xy}$`（径向距离）同字母；但 `$\rho_j$` 作采样时刻是**沿用本章既有 FAST-LIO 约定**（lidar:529,784,838,847），本章内自洽，属合理呼应，仅提示字母复用偏密。

---

## 正面确认（缝合 / 去重 / 数学 / 独立性 / 保护标签）

### 缝合到位
- **dense 四象限骨架真正贯穿全章**：`tab:dense-quadrant`（dense:135）在 dense:131,140-141,147,155,707,803,948,1010,1138,1180,1199,1364 被反复引用——**每一种主力表示（点云/OctoMap/TSDF/surfel/网格/GP/Hilbert/NeRF/3DGS）入场时都先在四象限里定位**，确属「一个框架贯穿到底」而非孤立装饰。ESDF 亦正确织进 TSDF 节（dense:982-983「投影式 SDF 不是真距离场」）与隐式节。
- **NeRF/3DGS 被正确框为「经典→可微渲染地图」自然演进**：`sec:dense-radiance`（dense:1240）开篇即点明「把流水线倒过来」，并显式接 surfel/TSDF/log-odds——insight「alpha 合成与 OctoMap 对数几率同根」（dense:1285）、「遍历图元 = surfel 省内存思路在渲染维度的翻版」（dense:1325）、小结「NeRF 把隐式表面神经化、3DGS 把面元升级成可微高斯、没跳出四象限」（dense:1364）。这是教科书级的承接，非另起新主题。
- **dense 神经隐式 SDF 桥**（dense:1202-1203）从 GPOM/GPIS/Hilbert「无学习连续场」过渡到 DeepSDF/iMAP「学习隐式场」再到可微渲染，链条干净。
- imu `sec:imu-advanced` 经 `ins:advanced-orthogonal`（imu:1123）把「变量重组 vs 怎么积分」讲成正交两件事、自然从离散欧拉演进到梯形/GP；`ins:preint-steam`（imu:1224）接 STEAM 论证经验算正确。
- lidar 多机器人/SubT/跨模态（lidar:1150-1157）作为 `sec:lidar-place` 子节自然加厚，MAGIC(2010)→SubT(2021) 二维→三维演进与 WildCat surfel 压缩、`ch:radar` 跨模态咬合良好。
- camera `sec:cam-modern`/`sec:cam-obs`、vo 视觉运动估计简史、PGBA 阶梯均织进原叙事，非 bolt-on。

### 去重（本轮新内容）到位
- **先进预积分**：vio `sec:vio-frontier`（vio:1143-1144）用指针 `\cref{sec:imu-advanced}` + 明文「此处不重复」，GP/平方指数核/旋转向量/jerk 推导**无一泄漏**到 vio。
- **学习型特征**：真委派——`ch:learning`（`P5_frontier/learning_slam.tex`:194,197,200）确实展开 SuperPoint/SuperGlue/LightGlue/LoFTR，camera/vo 只在 vo:106,1610,1745 与 cam 图节点**点去向、不展开**。
- **FMCW 推导**：lidar/point_cloud 把完整差频/多普勒推导转引 `\cref{ch:radar}`（`radar_slam.tex:170-262` 自包含全推导），**避免了 FMCW 数学在感知章重复**——是好架构。

### 数学（本轮新织入）独立验算无错
主审用 numpy 独立验算：
- **NeRF 体渲染求积**（`eq:dense-nerf-quad`，dense:1276-1281）：`$T_i=\prod_{j<i}(1-\alpha_j)=\exp(-\sum_{j<i}\sigma_j\delta_j)$`，`$\alpha_i=1-e^{-\sigma_i\delta_i}$`——两种 `$T_i$` 写法数值完全一致（验算 `np.allclose`=True），权重和 ≤1（能量守恒）。正确。
- **3DGS 协方差**（`eq:dense-gs-cov`，dense:1306）：`$\boldsymbol\Sigma=\mathbf{R}\mathbf{S}\mathbf{S}^\top\mathbf{R}^\top$` 对任意 `$\mathbf R\in\SO(3)$` 半正定，特征值=缩放平方（验算吻合）；**投影**（`eq:dense-gs-proj`，dense:1313）`$\boldsymbol\Sigma'=\mathbf{J}\mathbf{V}_{cw}\boldsymbol\Sigma\mathbf{V}_{cw}^\top\mathbf{J}^\top$` 取左上 2×2 仍 SPD。正确。
- **Power BA 级数**（vo:1587）：Neumann 级数 `$(\mathbf{H}^{red}_{cc})^{-1}=\sum_i(\mathbf{H}_{cc}^{-1}\mathbf{H}_{cp}\mathbf{H}_{pp}^{-1}\mathbf{H}_{cp}^\top)^i\mathbf{H}_{cc}^{-1}$` 与 `$(\mathbf{I}-\mathbf{X})^{-1}\mathbf{H}_{cc}^{-1}$` 一致。正确。
- **Schur**（cam/vo）：三步 + 复杂度 `$O(K^3+K^2M)$` 正确。
- **FMCW 差频/多普勒、ToF**（lidar/radar）：`$d=\tfrac12 c\Delta t$`、`$f_0=2Sd/c\Rightarrow d=cf_0/(2S)$`、`$v=\lambda\Delta\phi/(4\pi T)$`、`$v_{\max}=\lambda/(4T)$`（agent 验算）均正确。
- **GP 残差**（imu:1219）：`$\dot{\mathbf r}=\mathbf J_r(\mathbf r)^{-1}\boldsymbol\omega\Rightarrow\mathbf e_k=\tilde{\boldsymbol\omega}_k-\mathbf J_r(\mathbf r(t_k))\dot{\mathbf r}(t_k)$`（测得−预测陀螺）符号/形式/量纲正确。
- **GP 预测式**（`eq:dense-gp-mean/cov`，dense:1174-1175）：标准条件高斯后验，正确。
- CT-ICP 连续时间插值式**不存在**（lidar:545 明文 punt 为广度内容），故无式可验——是合理的广度取舍。

### 独立性 + 记号 + 保护标签
- **独立性基本干净**：dense/lidar/point_cloud/imu/vio 新织入内容**无** external_punt / ventriloquize / narration_dependence，对 Handbook 一律 `\cite{carlone2026handbook}`。唯一残留是 m6（vo:770）。`point_cloud_processing.tex:1230`「SLAM Handbook 只给定性描述……本节补全精确公式」是**正面的独立性姿态**（声明本书超越源），非违规。dense 多处 insight/note 以本书口吻直述。
- **记号避让正确**：dense 的 `$\mathbf G$`（GP Gram 矩阵）在 dense:1168 **显式声明「特意不写 `$\mathbf K$`，以免与相机内参混淆」**，且 `$\mathbf K$` 确在 dense:576,963 用作内参——避让生效；体密度 `$\sigma$`、不透明度 `$\alpha_i$` 在 dense:1249 局部声明避让深度滤波 `$\sigma$` 与极平面角 `$\alpha$`；视向 `$\mathbf d_{view}$` 避让深度 `$d$`；3DGS 视变换 `$\mathbf V_{cw}$` **不沿用源文献 `$\mathbf W$`**（因 `$W$` 已是 TSDF 权重）。dense 记号工程极为自觉。imu/vio 重力 `$\mathbf g$`、偏置 `$\mathbf b^g/\mathbf b^a$`、预积分量 `$\Delta\tilde{\mathbf R}/\Delta\tilde{\mathbf v}/\Delta\tilde{\mathbf p}$` 两章一致（唯一瑕疵 m8 的 `$\hat{\mathbf a}$`）。
- **保护标签全部在且语义未破坏**（独立 grep 确认）：
  - `sec:lidar-rae`（`lidar_slam.tex:160`）——RAE 球坐标观测模型，被 radar/rigid_body 引用，语义完好；
  - `eq:imu-preint-model`（`imu_model.tex:545`）——boxed 三式，被本章及 `leg_odometry.tex` 当同构母式引用，语义完好；
  - `eq:vo-direct-J`（`visual_odometry.tex:1361`）——直接法总雅可比 `$\mathbf J=-\partial\mathbf I_2/\partial\mathbf u\cdot\partial\mathbf u/\partial\delta\boldsymbol\xi$`，负号溯源正确，被 vo:1367,1380,1697 引用，语义完好。

---

## 给主审的处置建议（优先级）
1. **立即修 B1**（blocker，纯文献错配，低风险）：核对并改 `demmel2021pgba` 题录 + 顺手处理 m7 的「自适应」措辞。
2. **本轮宜修的 minor**：M1 + m1（测距物理跨章去重，本轮引入的真接缝）；m6（vo:770 narration 残留，一句改写）。
3. **全书层面统一决策，勿在本轮单方改**：M2（vo 标准预积分重述）、M3（LOAM 两章重复）——均属原始「自包含」旧设计，需作者拍板「自包含 vs 去重」总方针；m4/m5（camera↔vo 的 Schur/BA 史重叠）可一并归入。
4. **可选润色**：m2（CT-ICP 接 LOAM 插值一句）、m3（FAST-LIO 转置笔误，越界仅提示）、m8（`$\hat{\mathbf a}$` 等记号脚注）。
