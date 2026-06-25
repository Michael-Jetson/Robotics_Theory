# Tutorial 融合规划（范围A · 主题对齐填空）

> 把**同作者** Robotics_Tutorial（CC-BY 4.0 成品课程，428 文件 / 73.6 万行 md）按"理论对齐"融入《机器人学笔记》。落盘 2026-06-18。范围由用户定为 **A**。

## 0. 性质与纪律（每章必守）
- Tutorial 是成品（多轮审核闭环），但**由 AI(Claude/Codex)撰写、可能有错/幻觉**（错公式、张冠李戴的论文归属、编造数值）。故融合 = 选材 + md→ElegantBook 重渲染 + **联网对抗复核**：每章 synth 后必经 WebSearch/WebFetch 核实非平凡断言，错则就地改、联网仍存疑则打 `\rebuilt`/`\pz`。
- **记号归一化**到本书：R∈SO(3)/T∈SE(3)、**右扰动** R·Exp(δφ)（J_r 主）、**Hamilton** 四元数、se(3) 排序 **ξ=[ρ;φ]**（ρ≠t）。源若用左扰动/C 记号 → 转换并 note 注明。
- 逐公式 `\cite`；全 `\label`+`\cref`（禁手写号）；图一律 **TikZ 内联重画**；理论章 text:code≥85:15。
- **跳过纯工程**：框架剖析(Apollo/Autoware/PX4)、库 API(Pinocchio/Crocoddyl/OCS2/Ceres 内部)、ROS2/MoveIt2、仿真平台、RL 训练部署、Mini-* 实战、调研/审计/导读/附录/修复记录。
- bib **中心化合并**（主控做，agent 只返回 bibEntries/bibCorrections）；章节备份 `../_recovered_chapters/`；**每波后用户跑 `./compile.sh` 验**再下一波。
- 黄金范本 = `parts/P0_math/lie_theory.tex`；规范 = `docs/编写规范.md`（其上位 = Tutorial 自身 v5.0 教学规范）；避坑 = `docs/LaTeX易错手册.md`。

## 1. 规模与卷结构
| 部分 | 新章 | 理论源行 | 内容 |
|---|---|---|---|
| P1 估计补点 | ~3 | ~12k | iSAM2/Bayes树、不变滤波InEKF、非线性滤波族+平滑器 |
| P4 控制部 | 17 | ~71k | 最优控制9 + 刚体动力学3 + 浮动基座/接触/力控5 |
| P3 规划部 | 16 | ~92k | 时空4 + 采样MPC2 + 不确定2 + 博弈3 + 多机4 + TAMP1-3 |
| 合计 | ~36 | ~175k | 现书 28 章 / 33k 行 / 484 页 |

→ **多卷本**，契合 `book.tex` 既有分册设计：
- **vol_slam**（现状）：绪论+数学+估计+SLAM+前沿（P1 补点并入）
- **vol_control**：P4 控制 17 章
- **vol_planning**：P3 规划 16 章
- 备注：控制部"刚体动力学 3 章"为规划/控制共用地基，亦可单列「动力学」短部（执行时定）。

## 2. P1 估计补点（3）— 源 `01_数学/60_概率与估计/`
- **增量平滑与 Bayes 树（iSAM/iSAM2）** [NEW `ch:isam2`] ← `60_iSAM2与Bayes树` —— 接因子图章
- **不变卡尔曼滤波与流形滤波族（InEKF）** [NEW `ch:invariant-filter`] ← `30_流形滤波族` + `70_Barrau_Bonnabel精读`（群仿射/对数线性/稳定性）
- **(深化 `kalman_eskf`)** sigma 点族 UKF/CKF/GHKF + 平滑器(RTS)/平方根/信息形式/迭代变体/滤波族全景表 ← `20_经典非线性滤波族` + `40_Kalman族全景收口`
- 已覆盖→跳过：因子图/最小二乘(50)、可证最优(80)、鲁棒GNC(90)。

## 3. P4 控制部（17, A–Q）— 源 40=`01_数学/40_控制理论`, 50d=`01_数学/50_刚体动力学`, 5x=`05_运动控制`
最优控制理论 9：
- A 变分法/EL/PMP ← 40/10+20 [NEW]
- B 动态规划/Bellman/HJB/黏性解 ← 40/30+40 [DEEPENS LQR 基础]
- C LQR/LQG/Riccati 深化(存在唯一/分离原理/H∞) ← 40/50 [DEEPENS]
- D Lyapunov 稳定性/LaSalle/ISS/反步 ← 40/70 [NEW]
- E 辨识·鲁棒·频域(H∞/μ/灵敏度) ← 40/60+150 [NEW]
- F CLF/CBF/QP 安全控制 ← 40/80 [NEW]
- G MPC 深化(稳定性/数值/鲁棒随机) ← 40/110+120+130 [DEEPENS MPC]
- H DDP/iLQR(+约束) ← 40/90+100(剔 Crocoddyl API) [NEW]
- I HJ 可达性 ← 40/160 [NEW]
刚体动力学 3：
- J 空间向量代数/RNEA/ABA(O(n)) ← 50d/10+30 [NEW]
- K Lagrange/Hamilton/SE(3)几何力学/辛/Noether ← 50d/20+40+70 [NEW]
- L 约束动力学(DAE/Baumgarte)+解析微分 ← 50d/50+60 [NEW]
浮动基座/接触/力控 5：
- M 浮动基座动力学+腿足简化模型(LIPM/SRBD/CMM) ← 5x/足式50+70 [DEEPENS 足式概览]
- N 接触力学(摩擦锥/GIWC/LCP)+约束优化 ← 5x/足式80+60 [NEW]
- O 操作空间动力学+阻抗/导纳+无源性 ← 5x/机械臂F02+F01+F06 [NEW]
- P 力位混合/笛卡尔阻抗/WBC/TSID(HQP) ← 5x/机械臂F03+F07+足式90 [DEEPENS 机器人控制]
- Q 复合系统统一动力学+多模态MPC ← 5x/复合20+30 [NEW]

## 4. P3 规划部（16, P1–P16）— 源 10=时空, 20=采样MPC, 30=不确定, 40g=博弈, 50m=多机, 60t=TAMP
- P1 Frenet/ST图/路径-速度解耦 ← 10/20 [DEEPENS 轨迹优化入门]
- P2 时空走廊构建与凸分解(SFC/IRIS/FIRI) ← 10/30 [DEEPENS 安全走廊]
- P3 时空轨迹优化(CILQR/TEB/OBCA/MINCO) ← 10/40 [DEEPENS]
- P4 路径积分理论与 MPPI ← 20/20+30+50 [NEW]
- P5 MPPI 变体/扩散增强/TD-MPC ← 20/40+60+70 [NEW]
- P6 不确定性:分支场景/Tube-MPC/CBF安全滤波 ← 30/20+30 [NEW]
- P7 机会约束/POMDP/CVaR ← 30/40+50+60 [NEW]
- P8 微分博弈/HJI/可达 ← 40g/20 [NEW]
- P9 实时博弈(iLQGames/ALGAMES)/逆博弈 ← 40g/30+40 [NEW]
- P10 安全证书/博弈-CBF/MARL基础 ← 40g/50 + 50m/110 [NEW]
- P11 共识/分布式优化(Laplacian/ADMM/编队) ← 50m/20+30 [NEW]
- P12 MAPF/任务分配(CBS/LaCAM/ORCA/EGO-Swarm) ← 50m/40 + 10/60 [NEW]
- P13 分布式MPC/协同力控(Grasp matrix/异构) ← 50m/50+60+70+80 [NEW, 可拆2]
- P14 MARL 多机协调/安全学习控制 ← 50m/120+130 [NEW]
- P15 TAMP 符号-几何(PDDLStream/LGP/BT/信念TAMP) ← 60t/20+30+40+50+60+70+90 [NEW, 拆2-3]
- P16 扩散式/端到端/大模型规划 ← 10/70 + 60t/80 [NEW]

## 5. 跳过清单（纯工程/超范围 · 摘要）
控制：40/140(C++)、5x 机械臂 M01-M15/P01-P02/D01-D10、F05(ROS2)、足式 30/40/100/110/120-260、复合 RL/轮足/调研/附录、仿真全部、_调研_2026Q2。
规划：10/50(Apollo)、20/80/90/100/110、50m/90/100/140、70_无人机全目录、80_综述全目录、各"总论/附录/实战"stub。

## 6. 分波计划
- **Phase 0（本波）**：P1 三章 —— 验证整条 Tutorial→教材 流水线（含联网复核 + 深化模式）。
- **Phase 1–3 控制部**：最优控制(A–I,分2波) → 动力学(J–L) → 力控/WBC(M–Q)，每波≤6章。
- **Phase 4–6 规划部**：时空+采样(P1–P5) → 不确定+博弈(P6–P10) → 多机+TAMP+扩散(P11–P16)，每波≤6章。
- 每波闭环：synth(归一化) → 联网对抗复核(改错/标疑/修 cite) → 主控合并 bib + 结构QC(label/begin-end/坑扫) + 备份 → **用户编译**。

## 7. 状态
- 2026-06-18：Phase 0 启动（workflow `tutorial-fuse-p1`）。大卷顺序（control vs planning 先）待 Phase 0 验收后定。
- 2026-06-22：波1 已完成 P11（共识/分布式优化）+ P12–P13（MAPF / 分布式MPC / 协同力控，5 章，全书 1227pp / 0 `??`）。审核留痕：`docs/_archive/审核报告_P11共识分布式优化.md`；波1_P12-P13 报告 findings 已整合至 §8、原件已删。

## 8. bib 遗留待办（2026-06-23 整合自波1审核报告）
- `verginis2019cooperative` arXiv 号待最终核定（1905.01498 vs 1911.01297，现留带“待核验”一条）。
- 键名年份/venue 与实际不符（键名保留以匹配 `\cite`、字段已填正确值；如需可后续重命名键 + 改 `\cite`）：`fawcett2024datadriven`→实 2023 ICRA、`michael2014aerial`→2011、`aghs2024survey`→2026、`ravichandar2019strata`→2020 (JAAMAS)、`pandit2025biped`→CoRL 2024。
