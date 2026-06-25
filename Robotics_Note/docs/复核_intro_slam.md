# 独立复核报告：parts/P0_intro/intro_slam.tex（初识 SLAM）

**总评：PASS** —— 四维全部通过。十四讲 ch2 全量吸收、ch14 展望承接到位；新增「前沿与展望」节六个代表工作 `\cite` 与方向描述经 WebSearch 逐一核对**全部准确**；`find_package`/IDE 调试 practice 表述正确；审计「ch02/ch14」节所列应修复项（5 处 narration、动态/多机器人方向偏薄）**已全部落实**；残留 narration 为零，LaTeX 标签/`\cref`/环境配平零问题。仅 2 条 info 级观察（非缺陷）。

> 复核员：fresh / 未参与润色。方法：通读成稿章（1022 行）+ 十四讲源 ch02/ch14 + 审计 ch02/ch14 节 + 规范 §1/§6/§7；refs.bib 逐 key 核对；6 个前沿工作 + ORB-SLAM3/VINS-Mono 经 WebSearch 独立查证；narration/label/cref/env 全文 grep。

---

## 维度 1：知识完整（ch2 框架/方程/模块 + ch14 展望承接）—— 复核无误（PASS）

**结论：十四讲 ch2 全量吸收且增厚，ch14 展望承接到位。** 与审计「ch02」总判（全量吸收且大幅提升）一致；审计「ch14」原列的两条 partial_thin（动态/多机器人方向偏薄）已在本章新增 `sec:intro-frontier` 正面回填。

ch2 核心知识逐项落位（已核对）：
- 两个基本问题（定位/建图）+ 内外之分 → L80–88；SLAM 定义（未知/在线/同时/实时四源互证）→ `def:intro-slam`(L90)。
- 两类传感器对比（环境/携带式）+「未知环境」命门 → `tab:intro-sensors`(L102) / L115–116，与源 L48–102 一致。
- 鸡生蛋（纯建图/纯定位两退化子问题、联合后验不可拆、误差纠缠与可解性）→ `sec:intro-chicken-egg`，且补 Durrant-Whyte 单调收敛定理 `thm:intro-convergence`（超原书）。
- 三类相机（单目尺度不确定性 `thm:intro-scale`、双目基线/计算量、RGB-D 结构光/ToF）→ `sec:intro-camera`，与源 L115–156 逐点对应。
- 五模块（传感器/VO/后端/回环/建图）+ 数据流图 + 输入输出职责表 → `fig:intro-pipeline`/`tab:intro-modules`；累积漂移 89° 例 → `ex:intro-drift`（源 L204 一度之差完整保留）。
- 后端=状态估计/MAP、前后端分工、"空间状态不确定性"史 → `sec:intro-backend`，与源 L215–221 一致。
- 回环（图像相似性）、建图（度量/拓扑、稀疏/稠密、栅格/体素三状态）→ `sec:intro-loop`/`sec:intro-mapping`/`tab:intro-maps`，与源 L223–277 一致。
- 离散时间 + 运动方程 `eq:intro-motion` + 观测方程 `eq:intro-obs` + 平面/二维激光两参数化 `ex:intro-planar`/`ex:intro-laser` + 通用形式 `eq:intro-slam-general` + 四象限 LG/NLNG 分类 + 优化胜出史 → `sec:intro-math`，与源 §2.3 全量对应，并补严格性注（SE(2)/SE(3)、atan2）。
- Hello SLAM / g++ / CMake / 库·头文件·链接 → `sec:intro-code`，与源 §2.4.1–2.4.4 一致。

ch14 承接（审计两条 partial_thin 已回填）：
- 奠基系统 MonoSLAM(EKF/O(N³))、PTAM(双线程/首次 BA 替滤波/关键帧) → L657；现代 ORB-SLAM/VINS-Mono/LIO-SAM → `tab:intro-modern`。与十四讲 §14.1.1–14.1.3 + Cadena 一致。
- **动态/可变形 SLAM** 与 **多机器人 SLAM** → 新增 `sec:intro-dynamic`/`sec:intro-multirobot`，正面成节（原审计标"承接偏薄"，现已超额回填）。
- VIO、语义/深度学习方向 → 本章 L683 明示落 `ch:vio`/`ch:dense`/`ch:loop`（与补充审计 ch14 found_elsewhere 一致）。

- `[info]` 审计「ch02」原列 2 条 minor 知识缺口（find_package 工程概念、IDE 断点调试手把手）现已落实：`find_package` 复用见 `sec:intro-lib` 末 + `tab:intro-findpkg` + practice 第 6 题；最小调试 4 步见 `prac:intro-debug` + practice 第 5 题。两条 minor 缺口已闭合。

---

## 维度 2：正确性（前沿方向/代表工作 \cite + find_package/IDE practice）—— 复核无误（PASS）

**结论：六个前沿工作的方向描述与 `\cite` 全部准确（WebSearch 逐一核对）；工程 practice 表述正确。** refs.bib 中 14 个相关 key 全部存在、venue/year 准确。

前沿工作核对（位置 `sec:intro-frontier`，逐条已查证）：
- `[ok] L689` **DynaSLAM**\cite{bescos2018dynaslam}：建于 ORB-SLAM2、Mask R-CNN 语义分割 + 多视图几何识别动态物、剔除其特征、inpaint 背景 → 与原文（RA-L 2018, Bescós/Fácil/Civera/Neira）**完全一致**。
- `[ok] L691` **Co-Fusion**\cite{runz2017cofusion}：ICRA 2017 Rünz & Agapito，实时 RGB-D 分割成多物体、各维护 3D 模型分别跟踪融合 → **准确**（bib `R{\"u}nz` 拼写正确）。
- `[ok] L691` **DynamicFusion**\cite{newcombe2015dynamicfusion}：CVPR 2015，首个实时非刚性重建、canonical model + warp field（体素形变场）、template-free → **完全一致**。
- `[ok] L701` **CCM-SLAM**\cite{schmuck2019ccmslam}：集中式协作单目、agent 跑 VO + 中央服务器合并优化、容忍网络延迟/丢包（JFR 2019 Schmuck & Chli）→ **准确**。
- `[ok] L701` **DOOR-SLAM**\cite{lajoie2020doorslam}：分布式/点对点（无中心）、PCM（pairwise consistent measurement set）筛除离群机器人间回环（RA-L 2020 Lajoie 等）→ **完全一致**（PCM 正确归属 DOOR-SLAM）。
- `[ok] L701` **Kimera-Multi**\cite{tian2022kimeramulti}：分布式/仅点对点、鲁棒位姿图优化、全局一致带语义标注 3D 网格（TRO 2022 Tian/Carlone 等）→ **准确**。注：原文用 incremental maximum clique 做外点剔除，本章未把 PCM 误安到 Kimera-Multi 头上，归属精准。
- 现代系统表附带核对：ORB-SLAM3「完全基于 MAP 的紧耦合 VI + 多地图 ATLAS」、VINS-Mono「4-DOF 位姿图（重力使 roll/pitch 可观）」→ 均与原文**一致**。

工程 practice 核对：
- `[ok]` `find_package` 机制（`xxxConfig.cmake` 配置文件、按名查找、`target_include_directories(... ${Xxx_INCLUDE_DIRS})`、`target_link_libraries(... ${Xxx_LIBRARIES})`，`tab:intro-findpkg` L843）→ CMake 标准做法，**表述正确**。
- `[ok]` IDE 调试 4 步（`set(CMAKE_BUILD_TYPE "Debug")` 等价 -g/关 -O2、断点、Debug 启动停在断点、单步 F10/进入 F11、查变量；`prac:intro-debug` L861）→ **正确**，且明确"与 IDE 品牌无关"，符合审计"带新手跑通调试"诉求。

- `[info]` `sec:intro-frontier` 两处方向性 `\cite{cadena2016past}`（"思路大体分两路" L687 / 多机器人框架 L699）：Cadena2016 综述确有动态-SLAM、多机器人的"新前沿"论述，作为"此为活跃方向"的框架性出处可立；但"剔除 vs 正面建模"二分、集中式/分布式二分属本章作者综合，非 Cadena 原文逐字。此为 topic-first 合法综合（规范 §7），非缺陷，记录备查。

---

## 维度 3：行文/脉络（展望节融入 + \cref 呼应 + 直述立场连贯）—— 复核无误（PASS）

**结论：新增展望节自然融入、与专章 `\cref` 呼应充分；"主源→本书直述"改写后立场连贯。**

- `[ok]` `sec:intro-frontier` 动机段（L682）以"经典框架隐含前提：静态/刚体/光照稳/无人干扰"开篇，直接回扣 L316 框架适用边界与 `tab:intro-eras` 鲁棒感知时代——**承接自然不突兀**，且明示"展望性概览，不逐一展开细节，指明问题 + 代表工作 + 后续专章"，定位清楚。
- `[ok]` 与正文专章呼应密：动态节 `\cref{ch:dense}`（语义/稠密）、多机器人节 `\cref{sec:intro-loop}`（回环跨机器人版）/`\cref{sec:intro-need}`（感知混叠）/`\cref{eq:intro-nls}`（优化母方程）。两个 `insight` 盒把动态/多机器人都收束回本章"鸡生蛋"（L693）与"因子图优化母方程"（L703），与全书主线焊死，**脉络闭环**。
- `[ok]` 立场叙述连贯：审计原标 4 处"主源"narration 处现读来均为本书第一人称直述 + `\cite`——例如 `sec:intro-est` L574"本书的立场是：优化技术已明显优于滤波器技术…\cite{gaoxiang2019slam14}"，并以 Strasdat "Why Filter?" 独立佐证（L574/`insight` L646），立场陈述自然、无转述腔。
- `[ok]` 现代系统节（L656）"先看两个奠基者，再看三类现代主流"过渡顺，`tab:intro-modern` 后一段（L676）"区别只在喂进去哪种因子"再次印证后端统一性，与 `sec:intro-backend` 呼应。

无脉络断裂。

---

## 维度 4：独立性 + LaTeX —— 复核无误（PASS）

**结论：narration 残留为零；LaTeX 标签/cref/环境全部自洽。**

独立性（全文 grep 核对）：
- `[ok]` "主源/与主源一致/正如主源所言/主源把/主源强调/主源坦言" → **零命中**。审计「ch02」C 节列的 5 处（`:380`/`:574`/`:646`/`:828`/`:70`）均已清理：
  - 原 `:380`"主源把流程拆成五个模块" → 现 `sec:intro-frontback` L379"按五模块划分（`\cref{sec:intro-pipeline}`）…现代综述则常统一为前端/后端二分\cite{cadena2016past}"（本书直述）。
  - 原 `:574`"本书的立场（与主源一致）" → 现 L574 直述"本书的立场是…\cite"（已删"与主源一致"）。
  - 原 `:646`"这与主源'优化优于滤波'一致" → 现 `insight` L646"这与本书 `\cref{sec:intro-est}` 的结论…一致"。
  - 原 `:828`"正如主源所言，本讲'构成本书提要'" → 现 L888"本章是全书的提要：它概括地介绍了一个视觉 SLAM 系统的结构…\cite{gaoxiang2019slam14}"（删转述与引号，本书直述）。
  - 原 `:70`（记号 note）→ 现 L70 已改中立多书对照（"部分教材…只写抽象噪声项\cite{gaoxiang2019slam14}…严谨状态估计文献…设零均值高斯\cite{barfoot2024state,thrun2005probabilistic}…本书统一取零均值高斯"）。
- `[ok]` "十四讲"在正文 → 仅 2 处：L5（注释，不入正文）、L1003 延伸阅读"视觉 SLAM 十四讲\cite{gaoxiang2019slam14} 第 2 章"（合法出处）。无 `narration_dependence`/`ventriloquize`/`external_punt`。与补充审计 ch14 C 节"无违规"一致。

LaTeX：
- `[ok]` 全部 `\cref`/`\nameref` 目标（含 16 个跨章 `ch:*`：calib/camera/dense/eskf/imu/lidar_slam/lie/loop/nlopt/notebook... 等）均在全书 `\label{ch:*}` 集合内解析，零悬空。`ch:notation` 用 `\nameref`（因其为 front matter 无章号）—合理。
- `[ok]` 本章 73 个 `\label` 与正文 `\cref` 自洽；新增节标签 `sec:intro-frontier`/`sec:intro-dynamic`/`sec:intro-multirobot`、practice 标签 `prac:intro-debug`、表标签 `tab:intro-findpkg`/`tab:intro-libtype` 均被正确引用。
- `[ok]` 环境配平：practice 6/6、insight 9/9、pitfall 3/3、note 3/3、definition 2/2、theorem 2/2、example 3/3、codebox 9/9；全文 `\begin` 108 = `\end` 108。
- `[ok]` refs.bib：本章 14 个相关 `\cite` key 全部存在；frontier 6 key + ORB/VINS/LIO + 四基础书 venue/year 经核对准确。

---

## 复核员附注（非阻断）

1. `[info]` `sec:intro-frontier` 两处 `\cite{cadena2016past}` 框定"活跃方向 + 二分法"——综述支持方向存在，二分系本书综合，属规范 §7 topic-first 合法做法。若追求极致，可在二分处补一条更专的综述/教材 cite（如多机器人 SLAM 专门综述），但当前不构成缺陷。
2. `[info]` 全书级编译验证（`./compile.sh`，XeLaTeX+biber）本环境无 Docker 未跑；本复核为静态核对（label/cref/env/bib 全自洽），与收尾备忘 §187 所述"唯一剩余=实编译"状态一致，非本章问题。

**复核结论：四维全部 PASS，无 MINOR/NEEDS-FIX 级问题。** intro_slam.tex 作为全书开篇章，知识全量、前沿准确、脉络闭环、独立性与 LaTeX 干净，达到出版级。
