# RL运控 融合：进阶技巧建议（活文档）

> 来源：在 gpufree 对 **mjlab 1.4.0** 与 **UniLab**(`github.com/unilabsim/UniLab`) 的实跑 + 源码挖掘（2026-06-24）。
> 用途：本文档列出**源 md 章节欠覆盖、但框架真实提供且高价值**的进阶用法/工程技巧，每条**绑定目标章节**供成稿时补入。API 名一律以安装源实测为准（mjlab 在 `/root/gpufree-data/envs/.../site-packages/mjlab`；UniLab 在 `/root/gpufree-data/src/UniLab/src/unilab`）。与"问题与解决记录" B.1/B.2 配套。
> 标注：⭐ = 强烈建议补；○ = 锦上添花。实跑证据见本文末"实跑记录"。

---

## 一、mjlab 进阶技巧（每条→目标章）

### ⭐ 1. `expand_model_fields` + `@requires_model_fields` + `RecomputeLevel`（域随机化的内存/重算机制）→ Ch08（域随机化）
- **真实机制**（修正 md Ch08 的猜测写法）：DR 函数用**装饰器**声明它要改的模型字段，
  ```python
  from mjlab.managers.event_manager import requires_model_fields, RecomputeLevel
  @requires_model_fields("body_mass", recompute=RecomputeLevel.set_const)
  def body_mass(env, env_ids, asset_cfg, ...): ...
  ```
  装饰器把字段挂到 `func.model_fields`；env 构造时 `ManagerBasedRlEnv` 统一调
  `self.sim.expand_model_fields(self.event_manager.domain_randomization_fields)`
  （`manager_based_rl_env.py:309`）一次性把这些**本来跨 world 共享**的模型数组展开成 **per-world** 数组，否则 CUDA-graph 会从旧的共享数组读、静默忽略新值。
- **要点**：`RecomputeLevel`（枚举，如 `set_const`）控制改了 `qpos0/body_inertia/dof_armature` 后是否重算派生量；派生字段会自动追加进展开列表。**用户不手动碰 `_graph_captured`**。
- **教学价值**：这是"为什么 GPU 批量仿真里 DR 不能简单赋值"的核心工程点，比 md 现有的伪代码准确得多。建议在 Ch08 用真实装饰器范式重写，并解释"共享字段 vs per-world 字段 / CUDA-graph 重捕获"。

### ⭐ 2. 模块化 DR 包 `mjlab/envs/mdp/dr/` 全景（含 `pseudo_inertia`、`encoder_bias`、相机 DR）→ Ch08；相机部分→Ch18（视觉感知运动控制）
- mjlab 把 DR 拆成 per-物理量模块（`body.py/joint.py/geom.py/pair.py/camera.py/tendon.py/site.py/light.py/material.py/actuator.py`），函数名精简：`body_mass / body_com_offset / pd_gains / joint_stiffness / joint_damping / joint_armature / joint_friction / effort_limits / geom_friction / pair_friction / encoder_bias / pseudo_inertia / tendon_* / cam_fovy / cam_intrinsic / cam_pos / cam_quat / light_* / mat_rgba ...`。
- **`pseudo_inertia`（mjlab 独有，物理上合法的惯量随机化）**→ Ch08 + Ch12（Actuator建模与系统辨识，惯量部分）：在 4×4 伪惯量矩阵上扰动并经 **Cholesky** 重构，保证扰动后惯量张量仍**正定/物理可实现**（`_cholesky_4x4 / _decompose_pseudo_inertia_J / _reconstruct_pseudo_inertia_J`，含平行轴定理把 COM 惯量搬到 body 原点）。比"直接乘质量"高级，是 sim2real 惯量鲁棒性的正确做法。
- **`encoder_bias`（关节编码器零位偏置 DR，mode=startup）**→ Ch08 + Ch23（Sim2Real）：每 episode 给关节读数加常值偏置 `bias_range=(-0.015,0.015)`，模拟真实编码器装配零位误差——md 几乎没讲这类"传感器 DR"。
- **相机 DR（`cam_fovy/cam_intrinsic/cam_pos/cam_quat`）**→ Ch18：视觉策略 sim2real 必备，建议补入 Ch18 的视觉域随机化小节。
- **参数范式**：`func=dr.geom_friction, params={"operation":"abs"|"add"|"scale", "ranges":..., "shared_random":True}`；`shared_random=True` 让一组 geom 共享同一采样值（如四只脚同摩擦）。

### ⭐ 3. 多 GPU 训练 `--gpu-ids` + torchrunx → Ch24（大规模训练）/ Ch07（训练管线）
- 单文件无缝多卡：`train <TASK> --gpu-ids "[0,1,2,3]"`（或 `--gpu-ids all`）。`select_gpus()` 解析后，`num_gpus<=1` 直跑、`>1` 用 **torchrunx.Launcher**（`workers_per_host=num_gpus`，`copy_env_vars` 带上 `MUJOCO*`），每 worker 设 `MUJOCO_EGL_DEVICE_ID=local_rank`、`device=cuda:local_rank`、`seed=base+rank`，rsl_rl 自己起 process group。**只在 rank 0 录视频/写 yaml**。这是 md Ch24 该有的"如何扩到多卡"的真实落地，比泛泛而谈强。

### ⭐ 4. CUDA-graph capture（性能内核）→ Ch24（性能优化）/ Ch03（物理引擎）
- mjlab/MuJoCo-Warp 在**启用 memory pool 的 CUDA 设备**上自动 capture CUDA graph（`sim.py`，`wp.is_mempool_enabled(wp_device)` 门控），把整条物理步固化成一张图、消除 per-step kernel launch 开销——这正是它 64 envs 也能 2600+ steps/s 的原因之一。要讲清"graph 一旦捕获就锁定数组指针 → 故 DR 必须先 `expand_model_fields` 再让它重捕获"（与技巧 1 呼应）。建议 Ch24 增"CUDA-graph 与批量仿真吞吐"小节。

### ⭐ 5. NaN 守卫 + `viz-nan` 离线复现 → Ch24（NaN 排查）
- `--enable-nan-guard True` 打开 `NanGuard`（`mjlab/utils/nan_guard.py`，`sim.nan_guard.watch(data)` 上下文管理器逐步监控），命中 NaN 自动 dump（`sim.nan_guard.output_dir`）；事后 `viz-nan <dump.npz>` 可视化复现出事那一帧。md Ch24 讲了 NaN 现象，但**这套"自动 dump + 离线可视化"的工具链值得专门补**（这是 mjlab 相对手搓 `torch.isnan` 的工程优势）。

### ⭐ 6. Viser 远程可视化 + 检查点热切换 → Ch07（训练管线）/ Ch23（部署）/ Ch02（环境搭建）
- `play <TASK> --viewer {auto,native,viser}`：`auto` 有显示器用 `NativeMujocoViewer`、无显示器（服务器）用 **`ViserPlayViewer`**（浏览器远程看，跨境/无 X 都行）。viser viewer 内置 **`CheckpointManager`** 可在不重启的情况下**热切换/对比不同 checkpoint**（边训边看新模型）。`demo` 命令默认就是 `viewer="viser", num_envs=8` 跑预训练 tracking 策略。md 对"如何在远程 GPU 上看策略"覆盖弱，建议 Ch07/Ch23 补 viser 工作流。

### ○ 7. 策略导出与部署产物（自动 ONNX）→ Ch23（Sim2Real 部署）
- 训练**自动导出 ONNX**：实跑后 log 目录同时有 `model_N.pt` 与 `<timestamp>.onnx`（`mjlab/rl/exporter_utils.py` + `runner.save_model`）。`export-scene <TASK> --output-dir ...` 还能导出场景。Ch23 讲部署时应指明"mjlab 训练即产出 ONNX，可直接喂 onnxruntime/TensorRT"。

### ○ 8. 完整 Actuator 模型族（含 learned actuator）→ Ch12（Actuator建模与系统辨识）
- mjlab `actuator/` 不止一个 `ActuatorCfg`：基类 `ActuatorCfg`(字段 `armature/frictionloss/viscous_damping`) + `BuiltinPositionActuatorCfg` + `BuiltinPdActuatorCfg`(stiffness=kp / damping=kd / effort_limit) + `dc_actuator.py`(直流电机模型) + **`learned_actuator.py`(学习型执行器网络)** + `pd_actuator.py` + `xml_actuator.py`。还有反射惯量工具 `reflected_inertia()` / `reflected_inertia_from_two_stage_planetary()`（两级行星齿轮）。md Ch12 的执行器辨识应引用这套真实分层（尤其 learned actuator 与两级行星齿轮反射惯量，是 Unitree 类电机建模的关键）。

### ○ 9. 课程项与地形等级 → Ch06（奖励课程与终止）/ Ch13（四足粗糙地形）
- 真实课程函数：`mjlab/tasks/velocity/mdp/curriculums.py` 的 `terrain_levels_vel`（按跟踪表现升/降地形难度，配 `TerrainGenerator` 的 `max_init_terrain_level`）与 `commands_vel`（逐步放宽指令速度范围，含 `VelocityStage` TypedDict 分阶段）。运行日志确有 `Curriculum/command_vel/lin_vel_x_max` 这类曲线。Ch06/Ch13 补"地形等级课程 + 指令课程"的真实实现。

### ○ 10. 腱驱/位点力动作（灵巧手）→ Ch17（机械臂与灵巧手操作）
- 除关节动作外，mjlab 有 `TendonLengthActionCfg / TendonVelocityActionCfg / TendonEffortActionCfg / SiteEffortActionCfg`（腱长/腱速/腱力、位点力）。腱驱是欠驱动灵巧手（如肌腱手）的标准建模，Ch17 可补。

---

## 二、UniLab — 真实 API 基线 + 进阶特性（→ 各章"UniLab 实现"占位）

> UniLab 不在源 md；本节是**章节"UniLab 实现"小节的事实来源**。核心卖点：**异构 CPU 仿真 + GPU 策略**，绕开"GPU 仿真主导"范式。论文 arXiv:2605.30313，文档 unilabsim.github.io/UniLab-doc。

### A. 架构主线（→ Ch01 生态系统 / Ch24 大规模训练 的对照小节）⭐
```
┌── CPU Physics Sim ──┐   Unified Shared Memory   ┌── GPU Policy Training ──┐
│   MuJoCo / Motrix   │ ───────────────────────▶  │   PPO/SAC/TD3/APPO...    │
│  Multithread Step   │     SharedReplayBuffer     │  CUDA / MPS / ROCm / XPU │
└─────────────────────┘                           └──────────────────────────┘
```
- **CPU 多线程并行仿真** 产 transition，经**共享内存**流给 **GPU 策略学习**。与 mjlab/Isaac（GPU 仿真）形成对比：UniLab 适合 CPU 核多/无强 GPU 仿真后端、或物理需要 CPU 精度的场景。
- **IPC 层（`src/unilab/ipc/`，UniLab 的工程精华）**：`SharedBufferBase`(设备自适应共享内存缓冲) / `replay_buffer.py`(SharedReplayBuffer) / `rollout_ring_buffer.py`(on-policy 环形缓冲) / **`weight_sync.py` `SharedWeightSync`**(learner→collector 的 actor 权重共享，带 `version()` / `write_weights` / `read_weights_into`，避免每步序列化) / `shared_obs_stats.py`(共享 obs 归一化统计) / `async_runner.py`(用 `multiprocessing` **spawn** context 起 collector 子进程 + 错误管道 `collector_error_guard` 把子进程异常回传父进程) / `memory_budget.py` / `replay_pipelines`。
- **教学落点**：在讲"大规模/异构训练"或"PPO 数据流"时，用 UniLab 的 collector-learner 解耦 + 共享内存权重同步作为"非 GPU-sim 范式"的范例（Ch24，或 Ch01 框架对比）。

### B. CLI / 配置（→ 所有"UniLab 实现"小节的通用模板）⭐
- 入口（`pyproject.toml [project.scripts]`）：`train / eval / demo / unilab-complete / unilab-viz-nan / unilab-export-scene / unilab-import-robot / unilab-pull-assets / unilab-render-teaser`。
- **训练**：`uv run train --algo {ppo,mlx_ppo,appo,sac,td3,flashsac} --task <name> --sim {mujoco,motrix} [--render-mode {auto,interactive,record,none}] [--device cuda] [--profile] [Hydra覆盖...]`
- **评估**：`uv run eval --algo ... --task ... --sim ... --load-run {-1|run目录名} [--render-mode record]`（`record` = headless 录视频，服务器用）。
- **演示**：`uv run demo <name>`（teaser/dance/wallflip/wallflip2/boxtracking/locomani/inhandgrasp；首跑从 HF 拉权重）。
- **关键**：env 数/迭代数**走 Hydra 覆盖**（不是 flag）：`algo.num_envs=4096 algo.max_iterations=300`、`env.sim_dt=0.015`、`reward.scales.tracking_lin_vel=1.0`。路由键 `algo/task/training.sim_backend/training.play_only` 是 RESERVED，必须用 flag。后端切换语义 = owner 配置 `task=<task>/<backend>`。
- **配置文件**：`conf/<algo>/task/<task>/<backend>.yaml`（Hydra `# @package _global_`），结构化 dataclass 在 `src/unilab/structured_configs.py`（`PPOConfig/APPOConfig/SACConfig/TD3Config/FlashSACConfig`，各含 policy/algorithm 子配置）。
- 国内：首跑资产从 HF 拉，须 `export HF_ENDPOINT=https://hf-mirror.com`。

### C. 环境/任务 API（→ Ch22 DIY实战 / 各机器人章的 UniLab 实现）⭐
- 基类（`src/unilab/base/`）：`ABEnv`(抽象基) → **`NpEnv` / `NpEnvState`**（NumPy CPU 环境核，异构设计的 CPU 侧实现）；配置 `EnvCfg / SceneCfg / TerrainSceneCfg`；契约 `TerminalObservationContract`(终止观测处理) / `TransitionBootstrapContract`(bootstrap 价值)；增强 `SymmetryAugmentation`(对称性数据增强) / `PenaltyCurriculum`(惩罚课程) / `EpisodeLengthTracker`。
- 任务按机器人组织：`src/unilab/envs/locomotion/{g1,go1,go2,go2_arm,go2w}` + `manipulation/` + `motion_tracking/`。UniLab 自带 `dr/`(manager/provider/types) 与 `terrains/`（与 mjlab 各搞一套 DR/地形）。
- **奖励是 Hydra 标量字典**（`reward.scales{...}`，如 `tracking_lin_vel/tracking_ang_vel/lin_vel_z/base_height/action_rate/similar_to_default/contact/swing_feet_z`）+ `tracking_sigma` + `base_height_target`——与 mjlab 的 Python `RewardTermCfg(func=...)` 范式不同，成稿"UniLab 实现"小节须如实对照这种差异。

### D. 进阶算法路径（→ Ch07 训练管线 / Ch09 蒸馏 / Ch17 灵巧手）○
- **APPO**（异步 PPO，最能体现异构 collector-learner 解耦）；**FlashSAC / SAC / TD3**（off-policy，配 SharedReplayBuffer）；**MLX PPO**（Apple Silicon）；脚本级 **HORA**（`conf/hora_distill`，in-hand 操作课程蒸馏）与 **HIM-PPO**。in-hand 任务 `allegro_inhand / sharpa_inhand`（→ Ch17 灵巧手可作 UniLab 实现样例）。

---

## 实跑记录（证据）
- **mjlab PASS**：`MUJOCO_GL=egl train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 64 --agent.max-iterations 5 --agent.logger tensorboard --agent.upload-model False` → 跑满 5 iter，Total steps 7680，2638 steps/s，value/surrogate/entropy loss 正常更新，自动落 `model_0.pt/model_4.pt/<ts>.onnx`，无 error/NaN。
- **UniLab PASS（双后端）**：`uv run --no-sync train --algo ppo --task go2_joystick_flat --sim {motrix|mujoco} --render-mode none algo.num_envs=64 algo.max_iterations=3` → 各跑满 3 iter，~8900 steps/s，reward 各项正常，无 error/NaN。`--sim mujoco` 经普通 mujoco 3.8.0 跑通（`mujoco_uni` 未 sync 也行）。
- 磁盘：跑前/跑后 `/root/gpufree-data` 均 36G 可用（产物全清到 `/root/gpufree-data/tmp` 并删除）。
