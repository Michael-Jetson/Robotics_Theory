# 独立复核报告：parts/P2_slam/dense_mapping.tex

> **总评：PASS**（小幅修饰可选）。独立、对抗式复核（fresh，未参与之前润色），只读不改。
> 十四讲 ch12 全量吸收且大幅增厚；本轮新增的「点云三大需求·定位讨论」与「MLS+GP3 点云→网格代码及参数说明」**经 PCL 官方文档逐条核对，正确无误**；行文自然融入、无与三角化/深度滤波章重复或冲突；LaTeX/独立性干净，无残留 narration。唯二需留意者：(1) 两处 `\rebuilt{}` 批注会在终稿 PDF 中以「[重建待核对：…]」可见渲染，需确认是否保留；(2) 审计文档列的两条 minor 缺口（点云定位讨论、MLS+GP3 代码）**本稿已补齐**，建议回写审计表状态。

复核范围：成稿章 dense_mapping.tex（1239 行）；对照十四讲源 12_建图.md；审计项 docs/十四讲吸收审计.md「ch12」节；规范 §1/§6/§7。PCL API 经官方文档（pointclouds.org）与源码核验。

---

## 维度 1：知识完整性（十四讲 ch12 是否全在）

**结论：完整且大幅增厚。十四讲 ch12 五大块（极线搜索块匹配 / 深度滤波 / 八叉树 log-odds / TSDF / 点云）全部覆盖，且补 REMODE/SGM/KinectFusion/ElasticFusion/MC/泊松等多源。无 confirmed_missing。**

逐块核对（十四讲源 → 本稿）：

- `[ok]` 极线搜索 + 块匹配（§12.2.2）— SAD/SSD/NCC 三式 + ZNCC 去均值变体，方向相反约定，非凸分布 → `eq:dense-sad/ssd/ncc/zncc`、`sec:dense-epipolar` 全在，且补「梯度⊥极线」pitfall。
- `[ok]` 高斯深度滤波（§12.2.3）— 高斯归一化积 `eq:dense-fuse`、几何不确定度正弦定理 `eq:dense-unc-pprime`、四步算法 `alg:dense-gauss-filter`、深度≠z值 insight 全在；并补完整配方推导 `derivation`。
- `[ok]` 单目实践代码（§12.3）— main/update/epipolarSearch/NCC/updateDepthFilter 五函数全录（`fy=-480` 负号约定、0.85 阈、0.7 步长、`min_cov/max_cov` 等细节悉数保留）。
- `[ok]` 实验分析五小节（§12.3.1–12.3.6）— 像素梯度/逆深度/仿射变换/并行化/外点对策全在（`sec:dense-mono-pitfalls`、仿射式 `eq:dense-affine-rel` 含「t_RW 前无多余 K」量纲陷阱 note）。
- `[ok]` 点云地图（§12.4.1）— 反投影拼接 + SOR + 体素滤波代码、130万→3万/2% 数值、三大需求讨论全在（见维度 2）。
- `[ok]` 点云→网格 MLS+GP3（§12.4.2）— 本稿新增完整代码 + 参数解读（见维度 2）。
- `[ok]` 八叉树 log-odds（§12.4.3–12.4.4）— logit 变换 `eq:dense-octo-logit`、加性更新 `eq:dense-octo-logodds`、连分式贝叶斯 `eq:dense-octo-bayes`、8^10≈1073m³、6.9MB→56KB/<1% 全在；并补二值贝叶斯 derivation、clamping、逆传感器模型、剪枝、两套参数表。
- `[ok]` TSDF/Fusion 系列（§12.5）— 鬼影动机、符号约定、截断、投影 SDF、加权融合、frame-to-model、Fusion 谱系全在；并补 Curless-Levoy 凸去噪、光线投射提面、内存立方增长 pitfall。

`[note]` 增厚部分（贝叶斯混合滤波 `sec:dense-bayes-filter`、SGM `sec:dense-sgm`、surfel/ElasticFusion `sec:dense-surfel`、MC/泊松 `sec:dense-mesh`、工程选型 `sec:dense-engineering`）均超出十四讲 ch12，符合规范 §1.8 自包含 + §1.9 网络增补，非缺口。

---

## 维度 2：正确性（对抗式重点核对本轮新增）

### 2a. 点云「三大需求」中定位讨论 — **结论：正确**

位置：`dense_mapping.tex:749-757`（`\paragraph{点云能满足地图的哪些需求？}` + 定位 itemize）。

对照十四讲源 §12.4.1（md:895）逐句核对：

- `[ok]` **特征点法 VO 不能用纯点云定位** — 本稿:754「点云里只存了离散的三维坐标，并没有保存特征点的描述子，无从做特征匹配」⟷ 源「点云中没有存储特征点信息，则无法用于基于特征点的定位方法」。逻辑正确：点云退化为 `XYZRGB`，无描述子，特征匹配无从谈起。
- `[ok]` **ICP 法可但需全局精度** — 本稿:755「可以把当前帧局部点云对全局点云做配准…但要求全局点云本身精度足够好；而拼接流程只是叠加、未联合优化（无去重影、无 BA），精度通常不够，直接拿来 ICP 定位不可靠」⟷ 源「可以考虑将局部点云对全局点云进行 ICP…然而这要求全局点云具有较好的精度。我们处理点云的方式并没有对点云本身进行优化，所以是不够的」。**忠实且更具体**（点名「无去重影、无 BA」是合理增厚，不引入错误）。
- `[ok]` 导航/避障无法直接完成、可视化交互具基本能力但无表面/法线/可透视 — 与源 §12.4.1 第 2、3 条一致，`insight`「点云是一切高级地图的出发点」对应源「点云是一个不错的出发点」。

**对抗性追问**：ICP 定位说法是否过强？—— 不过强。本稿措辞为「直接拿来 ICP 定位是不可靠的」，限定语「直接/未优化」准确，与 KinectFusion 等 frame-to-model（对融合后 TSDF/全局模型配准）的成功案例不矛盾，且本章后文 TSDF 节正补足了「联合优化后的全局模型可支撑跟踪」，前后自洽。

### 2b. MLS + GP3 点云→网格代码与参数说明 — **结论：正确，且为对源码的合理「现代化」改进**

位置：代码 `dense_mapping.tex:1030-1082`（`reconstructSurface` / `triangulateMesh` / `main`）；参数解读 `:1084-1085`；选型 insight `:1087-1089`。

**API 调用核对（逐条对 PCL 官方文档）：**

- `[ok]` `MovingLeastSquares::setSearchRadius(radius)` — 官方：「Set the sphere radius… for k-nearest neighbors used for fitting」；本稿注「local fitting neighborhood radius」正确。
- `[ok][强正确性]` `setSqrGaussParam(radius*radius)` 与本稿:1085「PCL 在 setSearchRadius 时即默认如此置位」— **官方文档原文确认**：`setSearchRadius` 的说明含「**Calling this method resets the squared Gaussian parameter to radius \* radius !**」，且 `setSqrGaussParam` 说明「the square of the search radius works best in general」。本稿表述与官方**逐字吻合**。
- `[ok]` `setComputeNormals(true)` — 官方方法存在，「store the normals computed」；注「also output a normal per point」正确。
- `[ok][改进]` `setPolynomialOrder(2)` + 注「设 1 即退化为平面拟合」— 官方：「Setting order > 1 indicates using a polynomial fit」（order 1 = 平面）。**关键发现**：十四讲源（md:953）用 `setPolynomialFit(polynomial_order > 1)`，该方法已于 2017-09 **deprecated**（PCL PR#1960，1.12 移除，提示「use setPolynomialOrder() instead」）。本稿**正确删去 deprecated 调用、仅留 `setPolynomialOrder`**，是对源码的合规现代化，且 `:1085` 的「设 1 即退化为平面拟合」恰好等价于被弃用 flag 的语义——**改对了，不是回归**。
- `[ok]` `GreedyProjectionTriangulation::setSearchRadius(0.05)` 注「caps the maximum triangle edge length」— 官方：「practically the maximum edge length for every triangle」。正确。
- `[ok]` `setMu(2.5)` 注「search radius = mu × nn-distance（density-adaptive）」— 官方：「maximum acceptable distance… relative to the distance of the nearest point… to adjust to changing densities」，典型 2.5–3。正确。
- `[ok]` `setMaximumNearestNeighbors(100)` 注「neighbor budget per point」— 官方典型 50–100。正确。
- `[ok][重点核对]` `setMaximumSurfaceAngle(M_PI/4)` 注「45 deg: do not connect across sharp folds」+ 解读「拒绝法线偏差过大的邻点，避免跨越尖锐折边误连」— 官方原文：「points are not connected to the current point if their **normals deviate more than the specified angle**」。本稿「法线偏差过大」表述**精确正确**。
- `[ok]` `setMinimumAngle(M_PI/18)`（10°，「not guaranteed」）、`setMaximumAngle(2π/3)`（120°，「guaranteed」）— 与官方典型值及 guaranteed/not 语义一致。本稿注「preferred min」恰当反映了「not guaranteed」。
- `[ok]` `setNormalConsistency(true)` 注「input normals are consistently oriented…依赖 MLS 已给出的法线」— 正确，且点出与 MLS 上游的依赖关系，是合理增厚。

`[note][与源差异，合理]` 源 main 用 `vis.addPolylineFromPolygonMesh` + `addPolygonMesh`；本稿仅 `addPolygonMesh` + `vis.spin()`。简化展示不影响重建正确性，属合法精简。

**GP3 算法描述正确性**（`:1028`「不解全局隐式场，从种子三角形出发、沿边界贪婪地把邻近点投影到局部切平面并连接扩张，快、适合大而带噪点云，但不保证水密」）—— 与 GP3 原理及官方教程描述一致，「不保证水密」准确（区别于泊松的 watertight）。MLS/泊松/MC 三路线选型 insight（`:1087`）按「手里已有什么/要不要水密/能否承受全局求解」分类，准确无误。

---

## 维度 3：行文 / 脉络（新增是否自然融入、是否重复或冲突）

**结论：自然融入，无重复无冲突。**

- `[ok]` **定位讨论的位置**：插在点云代码 + 数值 + 「直接当导航地图」pitfall 之后、OctoMap 节之前（`:749`），承「点云初级在哪」的设问，逐条回扣 `sec:dense-intro` 五大用途，再以 insight「点云是一切高级地图的出发点」过渡到 OctoMap/泊松/surfel——脉络顺承，符合 v5.0 R2/R5/R14。
- `[ok]` **MLS+GP3 代码的位置**：置于「网格重建简介」节（`sec:dense-mesh`），紧接泊松重建之后，明确定位为「泊松那条路线工程上更轻的近亲」（`:1027`），并承接 `sec:dense-rgbd-pc` 存出的 `map.pcd`（`:1028`「承接…存出的 map.pcd」）——与点云节、TSDF 节、泊松段三方衔接自然。
- `[ok]` **与三角化/深度滤波章无重复**：MLS+GP3 是「点云→网格」后处理，与单目深度滤波（`sec:dense-depth-filter`）的「像素→深度」是不同管线阶段，无内容重叠。定位讨论引用 ICP（`\cref{ch:pointcloud}`）而非重述，符合规范 §7 纪律「同一成果只深讲一次」。
- `[ok]` **与 TSDF 融合 / surfel 融合无冲突**：MLS 平滑重采样、加权运行平均（TSDF `eq:dense-tsdf-fuse`、surfel `eq:dense-surfel-fuse`）三者各司其职，insight「clamping 与 W_η 是同一种智慧」横向打通，无矛盾。
- `[ok]` 选型总表 `tab:dense-engineering`、常见误解汇总、符号/公式/知识点速查、故障排查手册俱全，符合理论教学章模板（规范 §四 / v5.0）。

---

## 维度 4：独立性 + LaTeX

**结论：独立性干净，LaTeX 静态检查全过。**

### 独立性（规范 §1.8 铁律）

- `[ok]` 全文**无 narration_dependence / ventriloquize**：grep「十四讲(坦言/强调/指出/把/用/在/原书/原文)」「主源」「见原书/详见十四讲」**零命中**。
- `[ok]` 唯一含「主源」字样在 `:1209` 延伸阅读「（本章代码主源）」——属合法**出处致谢**（规范 §1.4/§7 允许 `\cite` 标源），非以源书作叙述载体，**合规**。
- `[ok]` 无 external_punt：所有内容自包含写入，`\cite` 仅标出处。点云定位、MLS+GP3 均独立复述，无「详见十四讲」。
- `[ok]` 无 meta-narration 禁词：grep「TODO/待补充/审稿/复检/worker/FIXME」零命中（规范 §0 G5）。

### LaTeX

- `[ok]` **环境配平**：codebox 11/11、insight 8/8、note 4/4、pitfall 4/4、derivation 3/3、algo 1/1、practice 3/3、definition 1/1、figure 1/1、equation 35/35、align 1/1、table 9/9——全部 begin=end。
- `[ok]` **无重复 label**；63 个 `\label` 唯一。
- `[ok]` **`\cref` 全解析**：48 个 cref 目标，章内 44 个均有定义；4 个跨章（`ch:camera`→camera_model.tex、`ch:lie`→lie_theory.tex、`ch:pointcloud`→point_cloud_processing.tex、`ch:vo`→visual_odometry.tex）**经核存在**。无悬空 cref、无硬编码章号/式号（规范 §2.3）。
- `[ok]` **代码注释全 ASCII**：codebox 内 `//` 注释 grep CJK 零命中；CJK 仅出现在 codebox **标题参数**（caption，LaTeX 正文，正确）。
- `[ok]` `\rebuilt` / `pitfall` 宏均已定义（common/preamble.tex:21、styles.tex:101）。

### 需留意项（非阻断）

- `[minor]` `:617`、`:980` 两处 `\rebuilt{...}` 批注（混合滤波式号、surfel 半径/权重闭式）。规范 §6 确实要求重建/存疑公式打 `\rebuilt`，**用法合规**；但其展开为「[重建待核对：…]」会在**终稿 PDF 可见渲染**（second 色脚注）。若本章定位为已成稿交付，建议：要么完成核对（对 Vogiatzis-Hernández/SVO 补充材料、Keller 原文）后摘除批注，要么确认保留为「诚实标注未尽事项」。属编辑决策，不影响正确性。
- `[minor][行文]` `:683`、`:702` 等处 `fy=-480.0` 在 RGB-D 反投影代码中沿用单目数据集的负 fy 约定。十四讲源码 pointcloud_mapping 同样如此（md:780），属忠实复刻 ICL-NUIM 约定，非错误；惟读者易困惑，章首记号 note 已统一说明（`:69`「fy 取负是合成数据集约定」），可接受。

---

## 给作者的回写建议（可选，非阻断）

1. **回写审计表**：`docs/十四讲吸收审计.md` ch12 节列的两条 minor partial_thin（点云定位讨论、MLS+GP3 代码）**本稿已补齐**，建议将状态由「建议补」更新为「已补」。
2. **`\rebuilt` 决策**：确认 `:617`/`:980` 两处批注是终稿保留还是待摘除。
3. （编译验证）本环境无 Docker，未跑 `compile.sh`；静态检查（环境配平/label/cref/ASCII）已全过，建议在有 Docker 处做一次实编译确认零未定义引用。

---

**复核签名**：独立复核员（fresh，未参与 dense_mapping.tex 之前润色）。方法：通读成稿 + 十四讲源全文逐块比对 + PCL 官方文档（setSqrGaussParam/setMaximumSurfaceAngle/setPolynomialFit 弃用）逐 API 核验 + grep 独立性/LaTeX 静态扫描。

**PCL 文档来源**：
- MovingLeastSquares: http://pointclouds.org/documentation/classpcl_1_1_moving_least_squares.html
- GreedyProjectionTriangulation 教程: https://pointclouds.org/documentation/tutorials/greedy_projection.html
- setPolynomialFit 弃用: https://github.com/PointCloudLibrary/pcl/pull/1960
