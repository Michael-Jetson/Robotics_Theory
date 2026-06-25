# 独立复核报告 — `parts/P2_slam/slam_system.tex`

**复核员**：独立复核员（fresh，未参与此前润色）
**对象**：`parts/P2_slam/slam_system.tex`（第「设计一个视觉 SLAM 系统」章，1734 行）
**对照**：十四讲源 `13_实践设计SLAM系统.md`、`docs/十四讲吸收审计.md`「ch13」节 + ch10 note(:144)、规范 §1/§6/§7
**日期**：2026-06-18

---

## 总评：NEEDS-FIX

**核心结论**：知识完整性、正确性、行文脉络三维度均为 PASS（实质全量吸收、与 ORB-SLAM2 对照清晰准确、代码出处声明合规）。**但本轮指派的独立性改写并未实际落地**——审计 `docs/十四讲吸收审计.md` ch13 节(:163)与 ch10 note(:144) 明确列出的 5 处 narration（`:101` 我的世界比喻、`:1514` 回答习题3、`:1528/:1579/:1595` "ch13 系统"）在成稿文件中**仍逐字存在、一字未改**。本任务的「本章重点 = 独立性改写后通读确认」前提不成立：改写尚未执行。判 NEEDS-FIX——非质量崩坏，而是约定动作缺失，5 处一句改写即可清零。

---

## 维度一：知识完整性 — PASS

十四讲 ch13 的全部知识点均在，且大幅增厚（用 slambook2/ch13 源码补全了原书"留给读者自行阅读"的全部函数）。逐项核验：

- `[PASS]` 双目 VO 框架（为何搭系统 / 程序=数据结构+算法 / 前端快后端准 / 地图中枢 / 目录约定）— §sec:sys-why、§sec:sys-arch，与源 §13.1/§13.2 逐点对应，*我的世界*比喻、雪人比喻、选双目理由（单帧初始化 + 3D 观测）全保留。
- `[PASS]` 数据结构 Frame/Feature/MapPoint/Map — §sec:sys-data，四类代码骨架与源 §13.3.1 完全一致（含 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`、weak_ptr 断循环引用、`num_active_keyframes_=7`、unordered_map 双容器）。
- `[PASS]` 前端光流 — §sec:sys-frontend，源只印了 `AddFrame`/`Track`/`TrackLastFrame` 三函数并"建议读者自行阅读"，成稿补全了 `StereoInit`/`DetectFeatures`(带掩膜)/`FindFeaturesInRight`/`BuildInitMap`/`triangulation`/`InsertKeyframe`/`SetObservationsForKeyFrame`/`TriangulateNewPoints`/`EstimateCurrentPose`/`Reset` 全部 8+ 函数。属超额吸收。
- `[PASS]` 后端局部 BA — §sec:sys-backend，`Optimize`/`BackendLoop`/构造/`Stop` 与源 §13.3.3 一致，并补全 `RemoveOldKeyframe`/`CleanMap`/`InsertKeyFrame` 滑窗逻辑（源未印）。
- `[PASS]` 系统总成 — §sec:sys-arch 的 `VisualOdometry::Init/Run/Step` 依赖注入装配（源散文描述，成稿落为代码）。
- `[PASS]` 实验（KITTI 运行步骤、16ms 非关键帧耗时、内存线性增长）— §sec:sys-experiment，与源 §13.4 一致。
- `[PASS]` 习题 3（回环）— 落入 §sec:sys-orbslam 的回环线程详述。

**补充观察（非缺口，属增厚）**：g2o 顶点/边与雅可比（§sec:sys-jacobian）、ORB-SLAM2 三+一线程对照（§sec:sys-orbslam）均为源外补入，与审计「无 confirmed_missing、无 partial_thin」判断一致。**知识维度复核无误。**

---

## 维度二：正确性 — PASS（抽查）

- `[PASS]` 代码骨架对照源：`:1305` 起的 `RemoveOldKeyframe`（se3 模长距离 + min_dis_th=0.2 删冗余近帧 / 否则删最远）、`:1159` 起 `Optimize`（id 偏移 `landmark_id+max_kf_id+1`、`setMarginalized(true)`、左右目外参选边、chi2_th=5.991 翻倍自适应）、`:648` 起 `EstimateCurrentPose`（4 轮迭代、`iteration==2` 去核精修）均与源/slambook2 语义吻合。
- `[PASS]` `triangulation` 返回值反向 bug — `:879` pitfall 准确指出官方仓库该函数"质量差却 return true"的真实 bug 并给正确修法。这是高质量的源码级正确性洞察，非杜撰（多 issue/fork 佐证）。
- `[PASS]` ORB-SLAM2 架构对照（`:1517` 三+一线程、`:1553` 三坐标关键点 + 虚拟右坐标 `u_R=u_L-f_x b/d`、`:1562` 共视图/生成树/本质图阈值 θ=15/100、`:1572` local BA 的 K1 优化/K2 固定、`:1568` 关键帧四插入条件 + 90% 冗余剔除）与 ORB-SLAM2/ORB-SLAM 论文一致、表述准确。`eq:sys-orb-motiononly`/`eq:sys-orb-localba` 与论文目标函数同形。
- `[PASS]` 数学：重投影误差 `e=z-π(·)`（观测减预测）、pose-only/local BA 目标、Schur 补 `eq:sys-schur`、Huber 与 χ²(2,0.05)=5.991 来历、左/右扰动雅可比及伴随关系——内部自洽，约定声明清楚（`:91` note 声明全书右扰动主约定 + 复用代码左扰动并显式标转换）。

- `[minor][fidelity]` `:620, :794` — 两处 `cv::calcOpticalFlowPyrLK` 窗口写 `cv::Size(11, 11)`，而十四讲源 `TrackLastFrame`(md:426) 印的是 `cv::Size(21, 21)`。— 窗口大小不影响教学点正确性，但与"取自 slambook2/ch13"的源印值有出入。— 建议：统一为源值 21×21，或不动（slambook2 repo 不同函数本就用不同窗口，11×11 亦属合理工程值）。仅记录，非阻断。

**正确性维度：抽查无实质错误。**

---

## 维度三：行文 / 脉络 — PASS（本章重点之一）

通读确认行文顺畅、脉络完整、与 ORB-SLAM2 对照清晰：

- `[PASS]` 脉络：前置自测→目标→知识导航(含 tikz 架构图)→桥接→「为何搭系统→架构→数据结构→前端→g2o 类型→后端→多线程→实验→ORB 对照」各节[动机→反面→理论→陷阱→练习]齐整→常见误解(7)→小结(三表)→延伸阅读→后续关系→故障排查(8)。v5.0 教学层完整，无断裂。
- `[PASS]` 与 ORB-SLAM2 对照清晰：§sec:sys-orbslam 逐条把精简 VO 缺失件（回环/重定位/共视图/本质图/冗余剔除/K2 固定帧）补齐，`tab:sys-orb-compare` 12 行对照表 + `:1613` insight"VO 与完整 SLAM 的分界"收束，对照充分且有洞察。
- `[PASS]` 代码出处声明合规（详见维度四）。
- `[PASS]` 雪人/传送带（`:432` insight）/砌砖盖房等直觉钩子全保留并升格为 insight/pitfall，过渡句自然。

- `[note] :101` — 我的世界比喻段落本身行文流畅（属合法直觉钩子，应保留比喻），唯叙述主语"十四讲用一个生动的比喻"是 narration（见维度四）。比喻去留 ≠ 独立性问题：审计建议是**淡化叙述主语**（改本书直述 + `\cite`），而非删比喻。

**脉络维度：行文顺、对照清晰，复核无误。**（唯叙述主语问题归入维度四。）

---

## 维度四：独立性 + LaTeX — NEEDS-FIX（独立性未改）；LaTeX PASS

### 4a. 独立性 — NEEDS-FIX（5 处 audit-prescribed narration 一字未改）

经 grep 全文核验，`docs/十四讲吸收审计.md`(:163 ch13 节 + :144 ch10 note) 明确列出待清理的 5 处，在成稿中**全部仍逐字存在**：

- `[NEEDS-FIX][narration_dependence] :101` — "十四讲用一个生动的比喻\cite{...}：在《我的世界》里…" — 以"十四讲"为叙述载体。— **建议**：删转述主语，本书直述 +句尾 `\cite`：「有一个广为流传的比喻：在《我的世界》（Minecraft）里…\cite{gaoxiang2019slam14}」（保留比喻，去依赖式主语）。
- `[NEEDS-FIX][narration_dependence] :1514` — "这同时也回答了十四讲第 13 讲的习题 3——"加回环线程"。" — 用源书习题号作叙述锚点。— **建议**：改本章自有口吻「这也正是把精简 VO 升级为完整 SLAM 的最后一块拼图」，习题指涉淡化或移脚注。（`:1581` "回环线程：习题 3 的完整答案" 标题与 `:1606` 表内"习题 3 建议"同源，建议一并改为"回环线程：从 VO 到完整 SLAM"/"（本章未实现，见延伸）"。）
- `[NEEDS-FIX][narration_dependence] :1528` — "我们的 ch13 系统只对应其中的 Tracking…" — "ch13 系统"指代本章产物。— **建议**：→「本章的精简双目 VO 只对应其中的 Tracking（前端）+ Local Mapping（后端局部 BA）」。
- `[NEEDS-FIX][narration_dependence] :1579` — "这是与本章 ch13 后端的核心差异：ch13 激活窗口内的 7 帧…" — 同上。— **建议**：→「这是与本章后端的核心差异：本章激活窗口内的 7 帧全部被优化…」。
- `[NEEDS-FIX][narration_dependence] :1595` — 表头 "本章精简双目 VO（ch13）" — 列名带"（ch13）"源书锚点。— **建议**：→「本章精简双目 VO」（删"（ch13）"）。

> 说明：审计原文(:163)将本节判为"轻微 narration"，且代码骨架"取自 slambook2/ch13"属**合法出处**。问题不在严重度，而在**约定动作未执行**——本任务被指派为"独立性改写后通读确认"，但改写尚未发生。这 5 处是教材内唯一一处仍以源书为叙述主体的残留，清理后本章独立性即达全书干净标准。

**其余 narration 扫描结果（均合法，无需改）**：
- `[PASS] :92, :93, :977, :986, :1701` — "本章复用的十四讲原始代码用左扰动"/"源代码骨架取自《视觉SLAM十四讲》\cite + slambook2/ch13"/"left perturbation (slambook2 original)"/延伸阅读列源 — 均为**合法代码出处声明 + \cite**，符合规范 §7（出处声明 vs punt：此为合法出处，非 punt）。
- `[PASS] :110, :213` — "代码组织对应官方仓库 slambook2/ch13"/codebox 标题"（slambook2/ch13）" — 合法出处标注。
- `[PASS] :880` — `triangulation` bug 处"slambook2 仓库里一个长期存在的 bug" — 合法的源码事实陈述，非依赖式叙述。
- `[PASS]` 无 `external_punt`（无"详见十四讲§x"式外推）、无 `ventriloquize`（无"十四讲坦言/特意强调")。grep "主源/原书/原文/坦言/强调" 零命中。

### 4b. LaTeX — PASS

- `[PASS]` 环境 begin/end 全配平：codebox(25)、insight(9)、pitfall(9)、practice(7)、algo(1)、note(1)、figure(4)、table(9)、align(1)、equation(12)、tikzpicture(5)。
- `[PASS]` 零重复 label（45 个 label，uniq -d 空）。
- `[PASS]` 零悬空 `\cref`：本章 36 个 `\cref` 目标全部解析——34 个章内 label + 6 个跨章 `ch:camera/lie/vo/nlopt/loop/dense`（均在对应章 :2 定义）。
- `[PASS]` 4 个 `\cite` key（carlone2026handbook/gaoxiang2019slam14/murartal2015orbslam/murartal2017orbslam2）全部存在于 `refs.bib`。
- `[PASS]` 自定义环境 algo/insight/pitfall/practice/codebox 均在 `styles.tex` 定义；`note` 由全书 20 文件共用（preamble/类提供），无未定义风险。
- `[PASS]` `sec:sys-orbslam` 与 `sec:sys-loop` 双 label 指向同一节标题(:1510-1511) — 经核验是**有意的别名**（两者都被 :60/:69 `\cref` 引用，解析到同一节号），非重复 label bug，符合"带回环的 §sec:sys-loop 是对精简 VO 的自然延伸"的导航设计。
- `[PASS]` 记号 note(:91) 约定齐全（Tcw 主约定、右扰动主 + 左扰动转换声明、Σ_v=I、ASCII 注释规范）。

---

## 整改清单（按优先级）

| # | 位置 | 问题 | 建议 |
|---|---|---|---|
| 1 | `:101` | narration "十四讲用一个生动的比喻" | 删主语，本书直述 + 句尾 `\cite`（保留比喻本身） |
| 2 | `:1514`（+ `:1581` 标题、`:1606` 表项） | narration "回答十四讲第 13 讲习题 3" | 改本章自有口吻，习题指涉淡化/移脚注 |
| 3 | `:1528` | "ch13 系统" | → "本章的精简双目 VO" |
| 4 | `:1579` | "本章 ch13 后端 / ch13 激活窗口" | → "本章后端 / 本章激活窗口" |
| 5 | `:1595` | 表头 "（ch13）" | 删"（ch13）" |
| 6（可选） | `:620, :794` | LK 窗口 11×11 vs 源 21×21 | 统一为源值或保留（非阻断） |

整改 1–5（独立性，约 5 句改写）后，本章四维全 PASS。第 6 项为 minor fidelity，可选。

---

*复核方法：通读全章 1734 行 + 比对十四讲源全章；grep 核验 narration 残留 / label / cref / cite / 环境配平；对照审计文档 ch13+ch10 prescribed 项逐条验证落地与否。只读未改。*
