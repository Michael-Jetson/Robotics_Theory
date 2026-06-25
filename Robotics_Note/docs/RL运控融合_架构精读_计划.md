# RL运控部「架构精读 · 补充」计划

## 为什么做
官方文档（尤其 Isaac Lab）篇幅很大，但只是**技术参考**，不是**教学文档**——讲清了"有哪些 API"，讲不清"**为什么这么组织、运行时数据怎么流、资产从哪来、怎么从零搭一个项目**"。本书的差异化价值就在这里：**从框架源码拆解架构，用教学语言讲清运行逻辑与组织逻辑**，并在 mjlab / Isaac Lab / UniLab 三框架间对比同一架构概念的不同组织方式。

## 素材就绪（gpufree 上三框架源码 + 可实跑印证）
- Isaac Lab 源码：`/root/isaaclab/source/{isaaclab, isaaclab_tasks, isaaclab_assets, isaaclab_rl, isaaclab_mimic, ...}`；Isaac Sim：`/root/isaacsim`（`_isaac_sim→/isaac-sim`）。
- mjlab 源码：mjlab 仓库（conda env `/root/gpufree-data/envs/mjlab`；如需源码再 clone）。
- UniLab 源码：`/root/gpufree-data/src/UniLab`（github.com/unilabsim/UniLab）。
- 方法：agent **读源码（不只读文档）** + 在 gpufree **实跑印证** → 产出教学段落 + 插图建议（数据流图 / 包结构树 / 时序图）。

## 要拆解讲清的内容（→ 目标章）
1. **运行逻辑**（→ Ch04 Manager 架构 prac_manager_arch）：`env.step` 全流程逐环节；Manager 系统（Observation/Reward/Termination/Event/Curriculum/Action Manager）如何被编排调用；GPU 张量数据流（SoA、零拷贝、CUDA-graph）。配数据流时序图。
2. **组织逻辑**（→ Ch01 生态 / Ch02 搭建）：包/项目结构（`isaaclab` 内核 vs `isaaclab_tasks` 任务 vs `isaaclab_assets` 资产 vs `isaaclab_rl` 训练接口）、`@configclass` 配置系统、task 注册机制（gym registry）。配包结构树。
3. **资产管线**（→ Ch11 资产链路 prac_assets）：资产**从哪来**（USD/MJCF、Nucleus/本地、官方资产库）；**如何导入新资产**（URDF→USD/MJCF 转换工具链、`ArticulationCfg`/`spawn` 配置、碰撞/惯量/驱动器设置）；常见坑。
4. **从零搭项目**（→ Ch22 DIY prac_diy）：用官方 template 起一个新 task 工程的**完整步骤**（目录、配置、env、注册、训练入口），一步步可复现。
5. **三框架对比**（贯穿）：同一概念（env 抽象、manager、资产、配置）在 mjlab / Isaac Lab / UniLab 的组织差异——这是本部"三框架对比"主线在架构层面的体现。

## 合成原则（用户定，最重要）
- **以算法为线串联三框架 API**：所有挖掘出的 API/架构材料，最终按**算法主线**组织——每个算法概念下**并列对比 mjlab / Isaac Lab / UniLab 的对应 API/写法**（如"观测"概念 → 三框架各自如何定义观测项、归一化、特权观测），**而非按框架各自堆叠**。
- **最终写入 LaTeX**：参考文档（UniLab参考 / 架构精读_运行与组织 / 资产与建项）均为**中间产物**；补充阶段统一**并入对应 `prac_*.tex`**，成书形态是 LaTeX 章节本身。

## 执行（统帅编排，控制并发）
- **时机**：迁移两波完成 + 框架就绪后启动；并发 1–2 个 agent（沿用低并发原则）。
- **产出**：架构讲解段落/图 → 并入对应 `prac_*.tex`（或在该章新增 `\section{架构精读：…}`）；源码与 md/官方文档不一致 → 回写 `RL运控融合_问题与解决记录.md`；可复用的高级技巧 → `RL运控融合_进阶技巧建议.md`。
- **复核**：这些新增内容同样纳入"每章 3 次复核"。
