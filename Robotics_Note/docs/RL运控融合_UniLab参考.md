# UniLab 深度参考（架构 + 编程模型 + 从零建任务 + 逐章占位填充）

> 用途：(a) 填充 `parts/P8_rl_motion_control/*.tex` 各章的 `\pz{UniLab 实现待补}` 占位；(b) 架构深读教材中心素材。
> 事实来源：**直接读源码 + 实跑** gpufree `/root/gpufree-data/src/UniLab`（commit `99936e08`, branch main；Python 3.13, torch 2.7.0+cu126, RTX 4090 + 64 核 Xeon）。日期 2026-06-24。
> 本文是「进阶技巧建议.md §二 UniLab」与「问题与解决记录.md §B.2」的**深层延伸**——基础 API/CLI 见那两处，本文给架构内部 + 全编程模型 + 逐章可落地代码块。
> 论文 arXiv:2605.30313；文档 unilabsim.github.io/UniLab-doc；README 副标题 *"A Heterogeneous Architecture for Robot RL Beyond GPU-Dominant Paradigms"*。

---

## 0. 一句话定位与本文导读

UniLab 是**异构 RL 运行时**：物理仿真在 **CPU**（MuJoCoUni / MotrixSim，多线程批量步进），策略学习在 **GPU**（PPO/APPO/SAC/TD3/FlashSAC，CUDA/MPS/ROCm/XPU）；二者经 **共享内存 IPC** 解耦（transition 单向流 collector→learner、actor 权重单向流 learner→collector）。这与 Isaac Lab / mjlab 的「GPU 仿真主导」范式正交——后者把物理和策略都钉在 GPU 上、靠 CUDA-graph 消除 launch 开销；UniLab 把物理留在 CPU、用多进程 + 共享内存把 CPU 核吞吐喂给 GPU 学习器。

- §1 架构内部（教学中心）：IPC / 共享内存 / 权重同步 / 进程拓扑 / 数据流 / 与 GPU-sim 范式的对比与权衡 + 建议教学图。
- §2 全编程模型：env 基类、Hydra dataclass 配置、reward 标量字典、obs/action 规格、后端选择、任务注册、CLI、算法集。
- §3 从零建一个 UniLab 任务：步骤 + 最小骨架（已实跑验证范式）。
- §4 逐章占位填充：每章给可直接放进 `\pz` 槽的 codebox（镜像 mjlab/Isaac Lab 的呈现方式）。
- §5 实跑验证记录 + §6 UniLab 真实欠缺的特性（不编造）。

---

## 1. 架构内部（异构 CPU-sim ↔ GPU-policy）

### 1.1 进程拓扑（spawn-based collector/learner 解耦）

```
   ┌─────────────────────── 父进程 = Learner (GPU) ───────────────────────┐
   │  - 持 actor/critic 网络在 GPU                                          │
   │  - 从 IPC 缓冲取 transition，做 PPO/SAC 更新                           │
   │  - 把新 actor 权重写进 SharedWeightSync（版本号 +1）                    │
   │  - 监听 collector 死亡（error pipe + exitcode）                        │
   └───────────────┬──────────────────────────────▲───────────────────────┘
       spawn 子进程 │ Process(daemon=True)          │ error pipe(ExceptionWrapper)
                    ▼                               │
   ┌─────────────────────── 子进程 = Collector (CPU) ──────────────────────┐
   │  - NpEnv 批量 CPU 仿真（MuJoCoUni / MotrixSim，多线程 step）            │
   │  - 周期 read_weights_into(actor) 拉最新权重（按 version 判断是否更新）   │
   │  - 把 (obs, act, rew, next_obs, done, trunc) 写进共享缓冲             │
   └───────────────────────────────────────────────────────────────────────┘

         ┌──────────────── Unified Shared Memory（/dev/shm）────────────────┐
   on-policy(APPO):  RolloutRingBuffer  (N-slot 环形, 原始 rollout payload)
   off-policy(SAC):  ReplayBuffer       (packed CPU 权威存储) + ReplayPipeline(双缓冲 H2D)
   权重:             SharedWeightSync    (float32 扁平 buffer + int64 version)
   obs 归一化:       SharedObsNormStats  (Queue 传 (mean,std))
   ```

关键源码事实（全部 `src/unilab/ipc/`）：
- **所有跨进程用 `multiprocessing.get_context("spawn")`**（`_SPAWN_CTX`，见 `weight_sync.py:13`、`async_runner.py:18`、`collector_error.py`、`rollout_ring_buffer.py:229`）。spawn（非 fork）保证子进程是干净解释器——避免 CUDA/原生句柄被 fork 复制导致的 SIGBUS/段错误。代价：子进程要重新 import + 重建注册表（见 §2.6 `ensure_registries` 的 `UNILAB_EXTRA_REGISTRY_PACKAGES` 注释专为 spawn 子进程设计）。
- **collector 子进程由 `AsyncRunner._start_collector` 起**（`async_runner.py`）：`_SPAWN_CTX.Process(target=_collector_entry_wrapper, args=(target_fn, error_send, kwargs), daemon=True)`。`_collector_entry_wrapper` 用 `collector_error_guard` 上下文管理器包住真正的 collector 函数。
- **错误传播 = Pipe + ExceptionWrapper**（`collector_error.py`，注释明说仿 PyTorch DataLoader / CPython ProcessPoolExecutor）：子进程崩溃时把可 pickle 的 `ExceptionWrapper`（含预格式化 traceback，故意不存 `exc_info` 以免引用环阻碍 GC）经单向 `Pipe(duplex=False)` 回传父进程；父进程 `_check_collector_alive()` → `_read_collector_error()` → `format_collector_death(exitcode, traceback)`。`format_collector_death` 还把负 exitcode 翻译成信号名 + 排障提示（SIGBUS=7→「native mmap/共享内存/CUDA pinned」、SIGKILL=9→「OOM killer，查 dmesg」、SIGSEGV=11→「C++/CUDA 段错误」）。**这套机制是 UniLab 工程成熟度的体现**：多进程 RL 最难调的就是 collector 静默死亡，UniLab 把它做成了带信号语义的死亡报告。

### 1.2 共享内存权重同步（learner→collector，版本化，零序列化）

`SharedWeightSync`（`ipc/weight_sync.py`）是异构设计的核心——learner 在 GPU 上更新完 actor，要让 CPU collector 用上新策略，**不能每步 pickle 一整个 state_dict 走队列**（太慢）。做法：

- 构造时按 `param_shapes` 算出 `total_numel`，开一块 `shared_memory.SharedMemory`，布局 = `[float32 扁平权重 buffer | int64 version]`（`data_bytes` 后紧跟 `meta_bytes=8`）。
- `write_weights(state_dict)`（learner 调）：拿 lock，逐参数 `param.detach().cpu().numpy().ravel()` 拷进 buffer，最后 `version += 1`。
- `read_weights_into(state_dict)`（collector 调）：逐参数 `copy_` 回来，返回当前 version。collector 据 version 判断「权重是否变了」决定是否真拷。
- `from_state_dict(state_dict, **kw)` 类方法一步建 + 写。
- 父进程建（`create=True`，自带 `_SPAWN_CTX.Lock()`）；子进程 attach（`create=False, shm_name=..., lock=`从父进程传入）。
- 内置 `trace_recorder` 钩子记 `weight_sync/write_weights_d2h` / `read_weights_into_cpu_actor` 时间片（profiling 用，见 §1.6）。

> **教学洞察**：这是「learner 与 collector 用不同设备、却要频繁交换大张量」的标准工业解法——共享内存 + 版本号 + 单写多读，避开序列化与拷贝。和 mjlab/Isaac（同进程同设备、权重就在 GPU 上无需同步）形成鲜明对比。

### 1.3 数据流：obs/action/reward 在哪跨 CPU↔GPU 边界

两条路径，取决于 on-policy（PPO/APPO）还是 off-policy（SAC/TD3/FlashSAC）：

**on-policy（`RolloutRingBuffer`，`ipc/rollout_ring_buffer.py`）**：
- N-slot 环形共享内存（默认 `num_slots=4`），字段 `obs/critic/actions/log_probs/rewards/dones/truncated/last_obs/last_critic`，每字段一块独立 SharedMemory（np.float32），形状如 `obs=(num_slots, num_envs, num_steps, obs_dim)`。
- collector 写满一个 slot 调 `signal_write_done()`（`_write_ptr += 1`）；learner `wait_for_data()`→`read_torch(device)` 把整 slot 搬到 GPU→`advance_read()`。
- `available()` / `_clamp_read_ptr_to_valid_window()` 处理「learner 跟不上、被覆盖」的窗口语义——这就是 **A**PPO 的 A：collector 不等 learner，旧 rollout 被覆盖（off-policy 校正用 V-trace，见 §2.5 `APPOAlgorithmConfig.vtrace_clip_rho/c`）。

**off-policy（`ReplayBuffer` + `ReplayPipeline`，`ipc/replay_buffer.py` + `ipc/replay_pipelines/`）**：
- `ReplayBuffer(SharedBufferBase)`：**packed CPU 权威存储**——一个 `torch.zeros(capacity, total_dim).share_memory_()`，`total_dim = 2*obs_dim + action_dim + 3 + 2*critic_dim`（obs|next_obs|act|rew|done|trunc|[critic|next_critic]），列切片 `_obs_sl/_nobs_sl/_act_sl/_rew_col/...` 固定。`add()` 由 collector 调（写 CPU 共享内存），`sample()` 由 learner 调（`_storage[indices].to(device)`）。
- `add()` 注释点出 UniLab 的生命周期契约：`done = terminated | truncated`，learner 必须把 `done` 与 `truncated` 配对算 bootstrap mask（呼应 `base/final_observation.py` 的 `TransitionBootstrapContract`）。终止帧的 `next_obs` 经 `_patch_terminal_next_observations` 用 `terminal_next_obs` 打补丁（保证 bootstrap 用的是真终止观测、不是 autoreset 后的新 obs）。
- **设备传输被独立成「replay pipeline」**：`CPUPinnedDoubleBufferReplayPipeline`（CUDA 走 pinned-host→GPU 异步 H2D 快路径 + 可选原生扩展 `native_h2d_ext.cpp`，否则回退 `dst.copy_(src, non_blocking=True)`），`MultiGPUCPUPinnedReplayPipeline`（每 rank 独立 host slot + H2D stream，不经 rank0 漏斗）。双缓冲 hot/cold slot + 后台 `_collector_h2d_worker` 线程异步预取下一批——把「CPU 采样打包 + H2D 拷贝」与「GPU 训练」流水线重叠。

> **本质**：UniLab 把「CPU 产数据 → GPU 用数据」这条边界做成了**显式、可计量、可预取**的管线（packed storage + pinned + 双缓冲 + 原生 H2D），这是它在 CPU-sim 前提下还能保持 GPU 训练吞吐的工程根。mjlab/Isaac 没有这条边界（数据本就在 GPU）。

### 1.4 内存预算守卫（`ipc/memory_budget.py`）

异构设计的副作用：共享内存（`/dev/shm`）可能被大 replay buffer 撑爆。UniLab 在分配前算预算：
- `estimate_offpolicy_bytes(...)` / `estimate_appo_bytes(...)` 估 replay + 双缓冲 / ring buffer 字节数（含 breakdown 字符串）。
- `raise_if_shared_memory_over_budget(...)` 超 `/dev/shm` 80% 直接 `MemoryError`（提示「减小 algo.num_envs 或 algo.replay_buffer_n，或加大 /dev/shm」）；`warn_if_over_budget(...)` 超系统内存阈值打警告（`UNILAB_SKIP_MEMORY_CHECK=1` 关）。
- **教学点**：这是「CPU 共享内存 RL」独有的失败模式（GPU-sim 没有），值得在大规模训练章作对比。

### 1.5 与 GPU-dominant（Isaac Lab / mjlab）的对比与权衡

| 维度 | UniLab（异构 CPU-sim + GPU-policy） | Isaac Lab / mjlab（GPU-sim 主导） |
|---|---|---|
| 物理仿真位置 | CPU 多线程（MuJoCoUni / MotrixSim） | GPU（PhysX / MuJoCo-Warp） |
| 策略学习位置 | GPU（独立进程） | GPU（同进程） |
| 解耦方式 | spawn 多进程 + 共享内存 IPC | 同进程，无 IPC |
| 数据跨边界 | 显式 CPU→GPU H2D 管线（pinned/双缓冲/原生） | 无（数据在 GPU） |
| 权重同步 | `SharedWeightSync`（版本化共享内存） | 无（权重就在 GPU） |
| 关键加速内核 | 多进程并行 + H2D 流水线重叠 | CUDA-graph capture（锁数组指针） |
| 适用场景 | CPU 核多/无强 GPU-sim 后端/物理需 CPU 精度/异步 off-policy | 有大显存 + GPU-sim 后端、追求单卡极致并行 |
| 典型规模 | 实跑 ~4600–8900 env-steps/s @ 32–64 CPU envs（小规模冒烟） | mjlab 64 envs ~2600 steps/s（GPU-graph） |
| 算法重心 | on+off-policy 并重（APPO/SAC/TD3 一等公民） | PPO（RSL-RL）一等公民，off-policy 需外接 |
| DR 内存机制 | 后端 per-world 模型（`materialize()`） | mjlab `expand_model_fields` + CUDA-graph 重捕获 |

权衡总结：UniLab 用「多进程 + 共享内存」换来了 **(1)** 不依赖 GPU-sim 后端（任何能跑 MuJoCo/Motrix 的机器都行）、**(2)** collector/learner 异步（APPO/off-policy 天然契合）、**(3)** CPU 物理精度（接触求解在 CPU 全精度）；代价是 **(1)** 失去 CUDA-graph 那种把整条物理步固化的极致并行、**(2)** 多了 CPU↔GPU 数据搬运（靠 H2D 流水线缓解）、**(3)** 共享内存预算成为新失败面。

### 1.6 诊断 / profiling / NaN（异构特供）

- **NaN 守卫**：`NpEnv` 内建 `_nan_guard`（`utils/nan_guard.py`，`NanGuard`）。`step()` 中 `check_ctrl` 查动作 NaN、`check` 查 obs/reward NaN、`capture` 存物理快照（若后端支持 `supports_physics_state_playback`）；命中即 `dump`。CLI/Hydra：`training.nan_guard.{enabled,buffer_size,max_envs_to_dump,output_dir}`（默认 `enabled: true`）。事后 `unilab-viz-nan <dump>` 离线复现。注意 `step()` 末尾还有兜底 `np.nan_to_num(reward, ...)`。
- **trace/profiling**：`--profile` flag + `training.trace_enabled/trace_output_dir/trace_thread_time/trace_cuda_events`（off-policy config 有这些字段）。IPC 各类内建 `trace_recorder.add_slice(...)`（`weight_sync/*`、`replay/*`、`replay_pipeline/*`），产 chrome-trace 风格时间片——可视化 collector/learner/H2D 重叠。
- **建议教学图**：(1) 进程拓扑图（§1.1 的两框 + IPC 三条边）；(2) 数据流时序图（collector step → 写 ring/replay → learner read/sample → H2D → update → write_weights → collector read_weights）；(3) on-policy ring buffer 的 write/read 指针窗口图；(4) off-policy 双缓冲 hot/cold + 后台 H2D 线程重叠图。

---

## 2. 全编程模型

### 2.1 env 基类层级（`src/unilab/base/`）

```
ABEnv(abc)                         # base/base.py —— 抽象基，定义 num_envs/cfg/observation_space/
  │                                #   action_space/obs_groups_spec/state/init_state/step/close
  │                                #   + 播放/渲染契约(resolve_play_render_plan/run_playback/...)
  └── NpEnv(ABEnv)                 # base/np_env.py —— NumPy CPU 环境核（异构的 CPU 侧实现）
        │                          #   持 SimBackend、num_envs、NpEnvState、DR manager、NanGuard
        │                          #   step() 模板: apply_action→backend.step→update_state→
        │                          #   compute_truncated→autoreset done envs
        │                          #   抽象方法: apply_action(actions,state)、update_state(state)
        └── LocomotionBaseEnv(NpEnv)   # envs/locomotion/common/base.py —— 四足/人形共用基
              ├── Go2BaseEnv → Go2WalkTask / Go2FootStandTask / Go2JoystickRoughEnv
              ├── Go1BaseEnv → ...
              ├── G1BaseEnv  → ...（人形）
              └── Go2W / Go2Arm ...
```

- **`NpEnvState`**（`@dataclass`，`np_env.py`）：`obs: dict[str,np.ndarray]`、`reward`、`terminated`、`truncated`、`info: dict`、`final_observation`。有 `.replace(**updates)`（基于 `dataclasses.replace`）。这是 env 每步返回的不可变快照。
- **`EnvCfg`**（`base/base.py`，`@dataclass`）：`scene: SceneCfg|None`、`sim_dt=0.01`、`ctrl_dt=0.01`、`max_episode_seconds`、`render_spacing`、`render_offset_mode="grid"`。派生属性 `max_episode_steps = max_episode_seconds/ctrl_dt`、`sim_substeps = round(ctrl_dt/sim_dt)`。`validate()` 强制 `sim_dt <= ctrl_dt`。
- **`SceneCfg`**（`base/scene.py`）：`model_file: str`（MJCF/Motrix XML 路径）、`fragment_files: list[str]`（冷路径拼装片段，如把 locomotion 任务 XML 拼进机器人 XML）、`terrain: TerrainSceneCfg|None`、`visual_model_file`（仅渲染用的视觉孪生）。`TerrainSceneCfg`：`generator: TerrainGeneratorCfg|None`、`hfield_name`、`geom_name`。
- **契约类**（`base/final_observation.py`）：`TransitionBootstrapContract`（`actor_next_obs/transition_next_obs/terminal_mask/timeout_terminal_mask/...`）与 `TerminalObservationContract`——把「自举该用哪个 next_obs」「超时 vs 失败」的语义固化成数据结构。这是 UniLab 处理 autoreset + PPO bootstrap 正确性的核心（对应 mjlab/Isaac 的 `time_out` 标记，但 UniLab 做成了显式契约）。
- **增强/课程**（`base/augmentation.py` + `base/curriculum.py`）：`SymmetryAugmentation`(Protocol，`augment_obs_and_actions`/`mirror_obs`，对称性数据增强；env 经 `build_symmetry_augmentation(device=)` 提供)；`EpisodeLengthTracker`(滑窗 episode 长度)；`PenaltyCurriculum`(按 episode 长度自适应缩放**负权重** reward——识别 `scales` 里 `<0` 的项，在 `[min_scale,max_scale]` 间按 `level_up/down_threshold` 调，原地改 `cfg.reward_config.scales`)。

### 2.2 obs / action 规格

- **观测组**：env 必须实现 `obs_groups_spec -> dict[str,int]`（组名→维度），如 Go2 joystick 的 `{"obs": 49, "critic": 52}`。`observation_space` 默认 = 所有组维度之和的 Box。组的语义由 `base/observations.py` 定：`split_obs_dict` 把 dict 拆成 `(actor=obs["obs"], critic=obs.get("critic", obs["obs"]))`——即 **`obs` 组喂 actor、`critic` 组喂 critic**（非对称 actor-critic 的实现，见 §4.5 特权学习）。`get_obs_dims` / `get_critic_base_dim` 由此取维度。
- **Hydra `algo.obs_groups`**：路由 RL 库（rsl-rl）的 obs 组。go2 flat 用 `obs_groups: {actor: [actor]}`，go2 rough 用 `{actor: [actor], critic: [critic]}`。**注意**：YAML 里的组名（`actor`/`critic`）是给 RL runner 的 obs-group 路由键，env 侧 `obs_groups_spec` 的键（`obs`/`critic`）是 env 内部组名——两层经 wrapper 对齐（与 mjlab `obs_groups={"actor":["policy"],"critic":[...]}` 同思想，但 UniLab env 侧固定用 `obs`/`critic` 两个组）。
- **动作空间**：`LocomotionBaseEnv._init_action_space` 从后端读 `get_actuator_ctrl_range()` + `num_actuators` 建 Box。动作经 `apply_action` 转 control：`ctrl = exec_actions * action_scale + default_angles`（位置式 PD 目标，`action_scale` 默认 0.25，`default_angles` 来自 keyframe `home` 的 qpos 尾段）。`simulate_action_latency=True` 时用上一步动作（模拟一帧延迟）。**这是关节位置目标动作（残差到默认姿态），不是力矩动作**。

### 2.3 reward = Hydra 标量字典（与 mjlab/Isaac 的 Python term cfg 范式不同）

UniLab 的 reward **不是** `RewardTermCfg(func=...)`，而是 **`reward.scales` 标量字典 + env 内 `_reward_fns` 派发表**：
- YAML 侧：`reward.scales: {tracking_lin_vel: 1.0, lin_vel_z: -5.0, ...}` + `reward.tracking_sigma` + `reward.base_height_target`（+ 任务特有阈值）。这些被 Hydra 注入 env 的 `RewardConfig`（`@dataclass`，字段 `scales: dict[str,float]`, `tracking_sigma`, `base_height_target`, ...）。
- env 侧：`_init_reward_functions()` 建 `_reward_fns: dict[str, Callable[[RewardContext], np.ndarray]]`，把 scale 名映射到函数（共享函数在 `envs/locomotion/common/rewards.py`，robot 特有的是 env 方法）。
- 每步 `_compute_reward` 建一个 `RewardContext`（`rewards.py`，bundle 了 `info/linvel/gyro/dof_pos/default_angles/tracking_sigma/base_height/...`），调 `run_reward_dispatch(scales=, fns=, ctx=, info=, enable_log=, ctrl_dt=)`。
- **`run_reward_dispatch`（`rewards.py:339`）核心逻辑**：`reward = Σ scales[name] * fns[name](ctx)`，跳过 `scale==0` 或 fns 里没有的项；每 `log_every_n_steps=4` 步把 `reward/{name}` 均值写进 `info["log"]`；**返回 `reward * ctrl_dt`**（即按控制周期缩放——等价于 mjlab/Isaac 的 `scale_by_dt`，但这里是 `ctrl_dt` 而非 `physics_dt*decimation`，注意 `ctrl_dt` 本身已是控制周期）；`only_positive=True` 时正裁剪。

> **逐章成稿要点**：UniLab 的 reward 章对照必须如实写「标量字典 + 派发表 + RewardContext」范式，与 mjlab 的类式 `RewardTermCfg`/Isaac 的函数式 `@configclass` 并列。共享 reward 函数全集（`common/rewards.py`，~40 个）：`tracking_lin_vel / tracking_ang_vel / forward_progress / under_speed / lin_vel_z / ang_vel_xy / orientation / roll / upright / base_height / similar_to_default / weighted_pose / action_rate / action_smooth / torques / energy / dof_acc / alive / dof_torques_l2 / dof_acc_l2 / joint_pos_limits / joint_power / stand_still / joint_pos_penalty / upward / track_lin_vel_xy_yaw_frame_exp / track_ang_vel_z_world_exp / feet_air_time_positive_biped / joint_deviation_l1`，外加 robot 方法 `_reward_swing_feet_z / _reward_contact / _reward_foot_drag`（go2）。

### 2.4 后端选择（mujoco-uni vs motrixsim）—— 差异

- **抽象**：`base/backend/base.py` 定 `SimBackend` ABC；`create_backend(backend_type, scene, num_envs, sim_dt, ...)`（`backend/__init__.py`）按 `"mujoco"`/`"motrix"` 实例化。后端负责：模型加载/编译、批量 step、传感器读取（`get_sensor_data`）、状态读写（`get_dof_pos/vel`、`get_base_pos`、`set_state`）、执行器 ctrl range、keyframe qpos、DR 能力声明（`get_dr_capabilities`）、per-world 模型 `materialize()`、播放/渲染。
- **抽象方法集（`SimBackend` ABC）**：属性 `num_envs/model/num_actuators/num_dof_vel`；`get_actuator_ctrl_range()`、`get_keyframe_qpos(name)`、`get_init_qvel()`、`get_body_ids(names)`、`get_joint_range()`、`step(ctrl, nsteps)`、`set_state(env_idx, qpos, qvel, randomization=)`、`get_dr_capabilities()`、`apply_interval_randomization(plan)`；运动学（世界系）`get_base_pos/quat/lin_vel/ang_vel`、`get_dof_pos/vel`、`get_body_{pos,quat,lin_vel,ang_vel}_w`；运动学（机体系）`get_body_{pos,quat,lin_vel,ang_vel}_b`；`get_sensor_data(name)`。**机体系速度直接可得**（`get_body_lin_vel_b`，呼应 mjlab 的便利）。许多默认方法（`get_geom_friction/get_body_mass/get_dof_armature/init_renderer/materialize/...`）非抽象、未覆盖即 `NotImplementedError`。`create_backend` 懒加载后端，motrix 缺失时 `ImportError("MotrixSim not available")`。
- **DR 能力差异（精确，已读两后端源码）**：每后端返回 `DomainRandomizationCapabilities`（`dr/types.py:37`，字段 `supported_reset_terms: frozenset / supports_interval_push / supports_interval_body_velocity_delta / supports_interval_body_force`）：
  - **MuJoCo 后端**（`backend/mujoco/backend.py:889`，**静态全集，11 项**）：`supported_reset_terms = {base_mass_delta, base_com_offset, gravity, body_iquat, body_inertia, body_ipos, body_mass, dof_armature, geom_friction, kp, kd}`；`supports_interval_push = (push_body_id>=0)`，`supports_interval_body_force=True`，`supports_interval_body_velocity_delta=False`。
  - **Motrix 后端**（`backend/motrix/backend.py:614`，**动态特性门控，基线仅 4 项**）：基线 `{base_mass_delta, base_com_offset, body_mass, body_ipos}`，按能力追加 `+{kp,kd}`（若支持位置执行器增益）`+geom_friction`（若支持摩擦覆写）`+gravity`（若支持重力覆写）；`supports_interval_push=True`，`supports_interval_body_velocity_delta=False`，`supports_interval_body_force` 受 `_supports_external_force` 门控。**Motrix 不支持 `body_iquat / body_inertia / dof_armature` reset 项**（MuJoCo 支持）。
  - `DomainRandomizationManager.reset` 经 `filter_reset_payload` 自动剔除后端不支持的项（打 warning 跳过、不报错）——优雅降级。这解释了 go2 flat 的 motrix.yaml 显式 `randomize_kp/kd: false`。
- **其他差异**：mujoco 后端经 `mujoco-uni==3.8.0`（本机用普通 `mujoco 3.8.0` 也跑通）；motrix 经 `motrixsim-core==0.8.2`。motrix 的交互渲染走原生 renderer（macOS 需 `mxpython`），且 **motrix 在 tracking 任务上「仅用于 sim2sim 评估/回放」注册**（见 §2.8 sim2sim）。`EnvCfg.motrix_max_iterations` 是 motrix 特有的求解器迭代上限（`create_backend` 映射为 `max_iterations`）。`post_step_forward_sensor` 控制传感器在 step 后是否再 forward 一次。

### 2.5 算法集（`structured_configs.py` + `algos/`）

入口经 CLI `--algo`，配置是 **typed dataclass**（`structured_configs.py`，注释说替代 ml_collections，用 OmegaConf/Hydra 组合）：

| algo | flag | 配置类 | 性质 | 关键字段（默认） | 脚本 |
|---|---|---|---|---|---|
| PPO | `ppo` | `PPOConfig` | on-policy（rsl-rl） | `num_envs=4096, num_steps_per_env=24, max_iterations=101`；`policy.actor/critic_hidden_dims=[512,256,128], activation=elu`；`algorithm`: `clip_param=0.2, entropy_coef=0.01, num_learning_epochs=5, num_mini_batches=4, learning_rate=1e-3, schedule=adaptive, gamma=0.99, lam=0.95, desired_kl=0.01`；`algorithm.class_name="unilab.algos.torch.rsl_rl_ppo:FinalObservationAwarePPO"` | `scripts/train_rsl_rl.py` |
| APPO | `appo` | `APPOConfig` | **异步 PPO**（最能体现异构解耦） | `num_envs=2048, steps_per_env=24, max_iterations=150`；`algorithm`: 同 PPO + **V-trace** `vtrace_clip_rho=1.0, vtrace_clip_c=1.0`（off-policy 校正） | `scripts/train_appo.py` |
| SAC | `sac` | `SACConfig` | off-policy（SharedReplayBuffer） | `num_envs=4096, batch_size=8192, replay_buffer_n=512, updates_per_step=4, gamma=0.97, tau=0.125, num_atoms=101`（分布式 critic）, `obs_normalization=True, use_layer_norm=True` | `scripts/train_offpolicy.py` |
| TD3 | `td3` | `TD3Config` | off-policy | `replay_buffer_n=1000, policy_noise=0.2, noise_clip=0.5, use_cdq=True`(clipped double Q) | `scripts/train_offpolicy.py` |
| FlashSAC | `flashsac` | `FlashSACConfig` | off-policy（快速 SAC） | `num_envs=1024, actor_num_blocks=2, learning_rate warmup/decay` | `scripts/train_offpolicy.py` |
| MLX PPO | `mlx_ppo` | （`config_mlx`） | Apple Silicon（仅 macOS） | 同 PPO | `scripts/train_mlx_ppo.py` |

脚本级路径（非 `--algo` flag）：**HORA**（`conf/hora_distill/`，in-hand 操作课程蒸馏，`scripts/` 有 hora 入口）、**HIM-PPO**（`conf/ppo_him/`，`scripts/train_him_ppo.py`）。
- PPO 用 rsl-rl 的 `OnPolicyRunner`，UniLab 自定义 `FinalObservationAwarePPO`（处理终止观测 bootstrap，呼应 §2.1 契约）。

### 2.6 任务注册（`base/registry.py`）

装饰器双注册（config + 每后端 env 类）：
```python
@registry.envcfg("Go2JoystickFlat")          # 注册配置类（CamelCase 名 = training.task_name）
@dataclass
class Go2JoystickCfg(Go2BaseCfg): ...

@registry.env("Go2JoystickFlat", sim_backend="mujoco")   # 注册 env 类 + 后端
@registry.env("Go2JoystickFlat", sim_backend="motrix")   # 同名可注册多后端
class Go2WalkTask(Go2BaseEnv): ...
```
- `register_env_config(name, cls)` / `register_env(name, cls, sim_backend)`：sim_backend 只接受 `"mujoco"`/`"motrix"`。`make(name, sim_backend, env_cfg_override, num_envs)` 建实例：建 cfg → `apply_cfg_overrides`（**深合并** Hydra 风格嵌套覆盖，如 `env.scene.terrain.generator.num_rows=4` 保留其他默认）→ `cfg.validate()` → `env_cls(env_cfg, num_envs=, backend_type=)`。
- `ensure_registries(packages=)`：import `unilab.envs.{locomotion,manipulation,motion_tracking}` 触发注册（每包 `__unilab_registry_modules__` 列要 import 的模块）。`UNILAB_EXTRA_REGISTRY_PACKAGES` 环境变量给 spawn 子进程注入额外注册包（spawn 是干净解释器，不继承 conftest）。
- **注意命名两套**：**注册名**（`training.task_name`）是 CamelCase（`Go2JoystickFlat`，实测共 **26** 个，见 §5 列表）；**CLI `--task` slug** 是小写（`go2_joystick_flat`），对应 `conf/<algo>/task/<slug>/<backend>.yaml` 目录。二者经 owner YAML 的 `training.task_name` 字段绑定。

### 2.7 CLI + Hydra（`cli.py` + `conf/`）

- 入口（`pyproject.toml [project.scripts]`）：`train / eval / demo / unilab-complete / unilab-viz-nan / unilab-export-scene / unilab-import-robot / unilab-pull-assets / unilab-render-teaser`。
- **`cli.py` 是 argparse 薄壳 → 路由到 `scripts/train_*.py` + 注入 Hydra 覆盖**。`build_route(algo, task, sim, profile)`：
  - on-policy：`scripts/train_rsl_rl.py`（ppo）/`train_appo.py`（appo），config_group=`ppo`/`appo`，owner=`<task>/<sim>.yaml`，生成覆盖 `task=<task>/<sim>`。
  - off-policy：`scripts/train_offpolicy.py`，config_group=`offpolicy`，owner=`<algo>/<task>/<sim>.yaml`，生成 `algo=<algo> task=<algo>/<task>/<sim>`。
  - `--profile hora` → owner 变 `<sim>_hora`（如 `conf/ppo/task/sharpa_inhand/mujoco_hora.yaml`）。
- **RESERVED 路由键**（`RESERVED_OVERRIDE_KEYS`）：`algo / task / training.sim_backend / training.play_only` **必须用 flag，不能 Hydra 透传**（透传会 `SystemExit`）。
- **env 数/迭代数走 Hydra 覆盖**（非 flag）：`algo.num_envs=4096 algo.max_iterations=300 env.sim_dt=0.002 reward.scales.tracking_lin_vel=1.0`。
- 训练：`train --algo ppo --task go2_joystick_flat --sim mujoco [--render-mode {auto,interactive,record,none}] [--device cuda] [--profile hora] [Hydra覆盖...]`。
- 评估：`eval --algo ... --task ... --sim ... --load-run {-1|run目录名} [--render-mode record]`。
- 演示：`demo <name>`（teaser/dance/wallflip/wallflip2/boxtracking/locomani/inhandgrasp；首跑从 HF 拉权重，国内须 `HF_ENDPOINT=https://hf-mirror.com`）。
- **直接调脚本**（绕过 cli 薄壳，本机验证用过）：`python scripts/train_rsl_rl.py task=go2_joystick_flat/motrix training.sim_backend=motrix algo.num_envs=32 algo.max_iterations=2 training.no_play=true training.log_root=/tmp/...`。
- 配置文件：`conf/<algo>/config.yaml`（根，含 `defaults: [_self_, task: <default>]` + `algo/training/interactive/viser/env/hydra` 段）+ `conf/<algo>/task/<task>/<backend>.yaml`（owner，头 `# @package _global_`，只覆盖差异）。

### 2.8 sim2sim 跨后端契约校验（`training/sim2sim.py`）

UniLab 内建**跨后端 sim2sim 一致性校验**（GPU-sim 框架少见的一等公民）——把一份后端训出的策略拿到另一后端回放/评估前，先比对「决定策略行为的字段」是否一致：
- `resolve_sim2sim_config(source_run_dir, target_cfg, *, algo_name=, strict=True)`：读源 run 的 `run_config.json` 里 `contract_snapshot`，与目标 play 配置逐字段比对。
- `DENYLIST`/`ENV_STRUCTURAL_DENYLIST` 里的字段若不一致（或一侧有一侧无）→「denial」；**`strict=True`（默认）抛 `CrossBackendIncompatibleError`**，否则打 `[sim2sim] WARNING (non-strict)`。非 denylist 差异只警告。
- `policy_load_dim_guard` 把张量 shape 不匹配重抛成 sim2sim 诊断（指向 `scripts/audit_sim2sim_contracts.py`）。
- CLI/Hydra：`training.sim2sim_strict`（默认 `true`）控制是否致命。motrix 在 tracking 任务上正是「仅用于 sim2sim 评估/回放」。
- **教学点**：这是异构多后端框架特有的正确性关卡——同一策略在 MuJoCo↔Motrix 间迁移，DR 能力/接触模型不同可能让策略行为漂移，sim2sim 契约把不兼容**前置**为编译期错误。

---

## 3. 从零建一个 UniLab 任务（已实跑验证范式）

### 3.1 步骤

1. **准备场景 XML**：MJCF（mujoco 后端）放 `src/unilab/assets/robots/<robot>/scene_*.xml`，含 keyframe `home`（提供 `default_angles`）+ 传感器（`local_linvel`、`gyro`、`upvector`、足端接触/位置 sensor）。
2. **写 env 类**：继承 `LocomotionBaseEnv`（或直接 `NpEnv`），实现 `obs_groups_spec`、`apply_action`（基类已给位置式 PD）、`update_state`（算 obs/reward/terminated）、`_init_reward_functions`、`_compute_obs`、`_compute_reward`。
3. **写 cfg dataclass**：继承 `EnvCfg`/`LocomotionBaseCfg`，定 `scene`、`max_episode_seconds`、`reward_config`、`domain_rand`、`commands` 等。
4. **注册**：`@registry.envcfg("MyTask")` 标 cfg、`@registry.env("MyTask", sim_backend="mujoco")` 标 env（多后端多标几次）。把模块加进所属包的 `__unilab_registry_modules__`。
5. **写 owner YAML**：`conf/ppo/task/my_task/mujoco.yaml`（`# @package _global_`），设 `training.task_name: MyTask`、`training.sim_backend: mujoco`、`algo.*`、`reward.scales.*`。
6. **训练**：`train --algo ppo --task my_task --sim mujoco algo.num_envs=4096 algo.max_iterations=300`。

### 3.2 最小骨架（基于实跑过的 Go2 范式精简）

```python
# src/unilab/envs/locomotion/mybot/joystick.py
from dataclasses import dataclass, field
import numpy as np
from unilab.base import registry
from unilab.base.backend import create_backend
from unilab.base.np_env import NpEnvState
from unilab.base.scene import SceneCfg
from unilab.envs.locomotion.common import rewards
from unilab.envs.locomotion.common.base import LocomotionBaseCfg, LocomotionBaseEnv
from unilab.envs.locomotion.common.commands import Commands
from unilab.envs.locomotion.common.rewards import RewardContext

@dataclass
class RewardConfig:
    scales: dict[str, float]
    tracking_sigma: float
    base_height_target: float

@registry.envcfg("MyBotJoystick")
@dataclass
class MyBotJoystickCfg(LocomotionBaseCfg):
    scene: SceneCfg = field(default_factory=lambda: SceneCfg(
        model_file="src/unilab/assets/robots/mybot/scene_flat.xml"))
    max_episode_seconds: float = 20.0
    commands: Commands = field(default_factory=Commands)
    reward_config: RewardConfig | None = None     # 必须经 Hydra 注入
    sim_dt: float = 0.01
    ctrl_dt: float = 0.02

@registry.env("MyBotJoystick", sim_backend="mujoco")
@registry.env("MyBotJoystick", sim_backend="motrix")
class MyBotWalkTask(LocomotionBaseEnv):
    _cfg: MyBotJoystickCfg
    def __init__(self, cfg, num_envs=1, backend_type="mujoco"):
        if cfg.reward_config is None:
            raise ValueError("reward_config must be provided via Hydra")
        backend = create_backend(backend_type, cfg.scene, num_envs, cfg.sim_dt,
                                 base_name="base",
                                 position_actuator_gains={"kp": cfg.control_config.Kp,
                                                          "kd": cfg.control_config.Kd})
        super().__init__(cfg, backend, num_envs)
        self._reward_cfg = cfg.reward_config
        self._init_reward_functions()
        # self._init_domain_randomization(MyDRProvider())  # 需要 DR 时

    @property
    def obs_groups_spec(self) -> dict[str, int]:
        return {"obs": 45, "critic": 48}           # actor / critic（critic 多特权量）

    def _init_reward_functions(self):
        self._reward_fns = {
            "tracking_lin_vel": rewards.tracking_lin_vel,
            "tracking_ang_vel": rewards.tracking_ang_vel,
            "lin_vel_z": rewards.lin_vel_z,
            "action_rate": rewards.action_rate,
            "similar_to_default": rewards.similar_to_default,
            "alive": rewards.alive,
        }

    def update_state(self, state: NpEnvState) -> NpEnvState:
        linvel = self.get_local_linvel(); gyro = self.get_gyro()
        gravity = self._backend.get_sensor_data("upvector")
        dof_pos = self.get_dof_pos(); dof_vel = self.get_dof_vel()
        terminated = gravity[:, 2] <= 0.5                       # 摔倒终止
        reward = self._compute_reward(state.info, linvel, gyro, dof_pos)
        obs = self._compute_obs(state.info, gyro, gravity, dof_pos, dof_vel)
        return state.replace(obs=obs, reward=reward, terminated=terminated)

    def _compute_obs(self, info, gyro, gravity, dof_pos, dof_vel):
        diff = dof_pos - self.default_angles
        cmd = info["commands"]; last = info.get("current_actions", np.zeros_like(diff))
        obs = np.concatenate([gyro, -gravity, diff, dof_vel, last, cmd], axis=1)
        critic = np.concatenate([obs, self.get_local_linvel()], axis=1)  # critic 加特权 linvel
        return {"obs": obs, "critic": critic}

    def _compute_reward(self, info, linvel, gyro, dof_pos):
        cfg = self._reward_cfg
        ctx = RewardContext(info=info, linvel=linvel, gyro=gyro, dof_pos=dof_pos,
                            num_envs=self._num_envs, default_angles=self.default_angles,
                            tracking_sigma=cfg.tracking_sigma)
        return rewards.run_reward_dispatch(scales=cfg.scales, fns=self._reward_fns,
                                           ctx=ctx, info=info, enable_log=True,
                                           ctrl_dt=self._cfg.ctrl_dt)
```
```yaml
# conf/ppo/task/mybot_joystick/mujoco.yaml
# @package _global_
training: {task_name: MyBotJoystick, sim_backend: mujoco}
algo:
  num_envs: 4096
  max_iterations: 300
  obs_groups: {actor: [actor], critic: [critic]}
reward:
  scales: {tracking_lin_vel: 1.0, tracking_ang_vel: 0.2, lin_vel_z: -2.0,
           action_rate: -0.01, similar_to_default: -0.1, alive: 0.5}
  tracking_sigma: 0.25
  base_height_target: 0.3
```
```bash
train --algo ppo --task mybot_joystick --sim mujoco algo.num_envs=4096 algo.max_iterations=300
```

---

## 4. 逐章 UniLab 占位填充（可直接放进 `\pz` 槽）

> 每块镜像各章 mjlab/Isaac Lab 的呈现方式。codebox 标题里 `_` 已转义（`\_`）。凡 UniLab 真欠缺的特性，§6 明列、对应章如实标注。

### 4.1 环境配置 / 生态系统 / Quick Start（prac_setup / prac_ecosystem / prac_manager_arch / prac_quad_loco Quick Start）

```text
% UniLab 安装与系统要求（对照 mjlab/Isaac Lab 的安装列）
推荐 uv 工作流（Conda/pip 用户也走 uv）：
  git clone https://github.com/unilabsim/UniLab && cd UniLab
  uv sync --extra mujoco          # MuJoCoUni 后端（mujoco-uni==3.8.0）
  uv sync --extra motrix          # 或 MotrixSim 后端（motrixsim-core==0.8.2）
系统要求：Linux CUDA / Linux ROCm / Linux XPU / Apple Silicon(macOS) 多套路径（README）。
  物理仿真跑 CPU（多核越多越好），策略学习需 GPU（CUDA/MPS/ROCm/XPU 之一）。
  与 Isaac Lab 不同：不依赖 Isaac Sim / NVIDIA 驱动绑定；与 mjlab 不同：不依赖 MuJoCo-Warp/GPU 仿真。
国内首跑拉 HF 资产：export HF_ENDPOINT=https://hf-mirror.com
```

```bash
# UniLab Quick Start（5 分钟最小可跑，对照 mjlab uv run train / Isaac Lab python scripts/.../train.py）
uv run train --algo ppo --task go2_joystick_flat --sim mujoco \
    algo.num_envs=4096 algo.max_iterations=300
# zero/short 冒烟：env 数与迭代数走 Hydra 覆盖（不是 flag）
uv run train --algo ppo --task go2_joystick_flat --sim motrix \
    algo.num_envs=64 algo.max_iterations=3 training.no_play=true
# 回放预训练 demo（首跑从 HF 拉权重）
uv run demo locomani      # Go2 loco-manipulation；其他：teaser/dance/wallflip/boxtracking/inhandgrasp
```

生态系统三框架对比表第三列（prac_ecosystem.tex:239-241）：

```text
物理世界  : CPU 多线程 MuJoCoUni / MotrixSim（接触求解在 CPU 全精度；非 GPU-sim）
MDP 机器  : 非 Manager-Based —— NpEnv 子类直接实现 update_state（obs/reward/done 一处算），
            reward 用 scales 标量字典 + RewardContext 派发表（不是 RewardTermCfg 类树）；
            DR/课程/对称性用独立 manager/contract（DomainRandomizationManager / PenaltyCurriculum /
            SymmetryAugmentation / TransitionBootstrapContract）
调试仪器  : Viser（web 3D，--render-mode interactive）+ 直接读 NpEnvState；
            NaN 守卫（NanGuard，dump+unilab-viz-nan 离线复现）；--profile chrome-trace
```

> 框架定位（放选型决策树 UniLab 分支）：**当 CPU 核多、无强 GPU-sim 后端、或要异步 off-policy（APPO/SAC）训练时选 UniLab**；它把物理留在 CPU、用共享内存把 collector/learner 解耦，是「GPU-sim 主导」之外的第三条路。

### 4.2 Manager 架构对照（prac_manager_arch.tex 各 `\pz`）

```text
% UniLab 不是 Manager-Based —— 逐项对照 mjlab/Isaac Lab 的 manager
env.step() 内部时序（np_env.py NpEnv.step）：
  apply_action(actions,state)            # 动作→ctrl（位置式 PD：act*action_scale+default_angles）
  → dr_manager.apply_interval_randomization_if_due(step)   # interval DR（等价 EventMode.interval）
  → backend.step(ctrl, sim_substeps)     # CPU 批量物理步（sim_substeps=ctrl_dt/sim_dt）
  → update_state(state)                  # 一处算 obs+reward+terminated（替代 Obs/Reward/Term 三 manager）
  → _compute_truncated(state)            # 超时截断（max_episode_steps）
  → autoreset done envs（_reset_done_envs，含 final_observation 打补丁）
  → NaN 守卫 check + np.nan_to_num(reward)
任务注册 : @registry.envcfg("Name") 标 cfg + @registry.env("Name", sim_backend="mujoco") 标 env
           （多后端多标）；make(name, sim_backend, env_cfg_override, num_envs)；
           CLI 覆盖经 Hydra（apply_cfg_overrides 深合并嵌套键）。
观测组   : env.obs_groups_spec -> {"obs": <actor 维>, "critic": <critic 维>}；
           默认组名固定 obs/critic（split_obs_dict：obs→actor、critic→critic，无 critic 则复用 obs）。
reward 项: 无 RewardTermCfg 类；scales 标量字典 + _reward_fns 派发表；
           run_reward_dispatch 返回 reward*ctrl_dt（等价 scale_by_dt，按控制周期缩放）。
动作项   : 无 JointPositionActionCfg 类；动作=关节位置目标残差，
           ctrl = act*action_scale(默认0.25) + default_angles（apply_action）；
           actuator 经后端位置执行器 gains {kp,kd}（create_backend 的 position_actuator_gains）。
终止项   : 无 DoneTerm/TermTerm 类；terminated 在 update_state 里直接算（如 gravity[:,2]<=0.5 摔倒）；
           超时 truncated 由基类 _compute_truncated 按 max_episode_steps 自动算；
           done = terminated | truncated（生命周期契约，bootstrap 用 TransitionBootstrapContract）。
event 模式: DR 经 DomainRandomizationManager 分发——init（≈startup，apply_init_randomization）/
           reset（per-episode，build_reset_plan）/ interval（apply_interval_randomization_if_due）。
命令/课程: Commands（commands.py，速度命令采样+heading 反馈）；课程有 PenaltyCurriculum（按 episode
           长度缩放负权重）+ TerrainCurriculumCfg/TerrainSpawnManager（地形等级）。
状态数据源: NpEnvState + 后端 getter（get_local_linvel 直接给机体系速度、get_gyro、get_dof_pos/vel、
           get_base_pos、get_sensor_data(name)）——机体系速度可直接得（类似 mjlab 的便利）。
调试/NaN : Viser web 可视化 + NanGuard（自动 dump + unilab-viz-nan 离线复现）+ --profile trace。
PPO 后端 : 复用 rsl-rl（OnPolicyRunner），自定义 FinalObservationAwarePPO；配置类 PPOConfig
           （structured_configs.py），CLI 入口 train --algo ppo。
```

### 4.3 观测与动作（prac_obs_action / prac_quad_loco:469）

```python
# UniLab：actor/critic 两组观测（非对称）+ 噪声 + 特权 term（go2_joystick 实例）
# env 侧固定两组：obs(喂 actor) / critic(喂 critic，含特权量)
@property
def obs_groups_spec(self) -> dict[str, int]:
    return {"obs": 49, "critic": 52}          # critic 比 obs 多 3 维 = 特权 linvel

def _compute_obs(self, info, linvel, gyro, gravity, dof_pos, dof_vel, feet_phase):
    nc = self._cfg.noise_config
    diff = dof_pos - self.default_angles
    obs = np.concatenate([                     # actor：只含真机可得量 + 噪声
        self._obs_noise(gyro, nc.scale_gyro),          # 角速度(3)
        -self._obs_noise(gravity, nc.scale_gravity),   # 投影重力(3)
        self._obs_noise(diff, nc.scale_joint_angle),   # 关节角差(12)
        self._obs_noise(dof_vel, nc.scale_joint_vel),  # 关节速(12)
        info.get("current_actions", np.zeros_like(diff)),  # 上一动作(12)
        info["commands"],                              # 速度命令(3)
        feet_phase,                                    # 步态相位(4)
    ], axis=1)
    critic = np.concatenate([                  # critic：无噪声 + 追加特权 base linvel
        gyro, -gravity, diff, dof_vel, info["current_actions"],
        info["commands"], feet_phase, linvel], axis=1)   # +linvel(3) = 特权
    return {"obs": obs, "critic": critic}
# 噪声经 _obs_noise(data, scale)：仅 actor 加，level<=0 关；幅度 = level*scale*U(-1,1)。
# Hydra 路由：algo.obs_groups = {actor: [actor], critic: [critic]}
# 维度自检：env.obs_groups_spec → {"obs":49,"critic":52}；observation_space = Box(sum=101)
```

### 4.4 奖励课程与终止（prac_reward 各 `\pz` + prac_quad_loco:609）

```python
# UniLab：reward = scales 标量字典 + RewardContext 派发表（对照 mjlab 类式 / Isaac 函数式）
# (1) tracking 项（共享函数，common/rewards.py）—— 指数核，σ 经 reward.tracking_sigma 配
def tracking_lin_vel(ctx):                      # = exp(-||cmd_xy - v_xy||² / σ²)
    e = np.sum(np.square(ctx.info["commands"][:, :2] - ctx.linvel[:, :2]), axis=1)
    return np.exp(-e / ctx.tracking_sigma)
# (2) 派发 + dt 缩放（run_reward_dispatch，rewards.py:339）
reward = run_reward_dispatch(scales=cfg.scales, fns=self._reward_fns, ctx=ctx,
                             info=info, enable_log=True, ctrl_dt=self._cfg.ctrl_dt)
#   reward = Σ scales[name]*fns[name](ctx)，跳过 scale==0；每 4 步写 info["log"]["reward/<name>"]；
#   返回 reward*ctrl_dt（等价 mjlab/Isaac 的 scale_by_dt，按控制周期缩放）。
```
```python
# UniLab：自定义「左右脚 air time 差异」惩罚（返回 [num_envs]，值域 [0,1]，对照练习要求）
def feet_air_time_balance(ctx) -> np.ndarray:   # 放进 env._reward_fns
    at = ctx.info["feet_air_time"]              # (N,4)，env 自维护
    front = at[:, :2].mean(axis=1); rear = at[:, 2:].mean(axis=1)
    return np.tanh(np.abs(front - rear))        # 差异越大惩罚越大
# 在 scales 里给负权重即生效：reward.scales.feet_air_time_balance: -0.05
```
```yaml
# UniLab：四层奖励（tracking 主导 / regularization / style / contact），σ 默认 0.25（go2 flat）
reward:
  scales:
    tracking_lin_vel: 1.0          # tracking 层（主导梯度）
    tracking_ang_vel: 0.2
    lin_vel_z: -5.0                # regularization 层（penalty，负权重）
    ang_vel_xy: -0.1
    action_rate: -0.005
    similar_to_default: -0.1
    base_height: -100.0
    swing_feet_z: 4.0             # style 层（步态外观）
    contact: 0.24                  # contact 层（相位-接触一致）
  tracking_sigma: 0.25
  base_height_target: 0.3
# σ 课程：UniLab 无现成「按步收紧 σ」课程函数（见 §6）；可在 owner YAML 分阶段手调 tracking_sigma，
#   或自定义回调改 cfg.reward_config.tracking_sigma。PenaltyCurriculum 自带「按 episode 长度缩放负权重」。
```
- 终止条件命名：UniLab **无独立 TerminationTermCfg 类**；terminated 在 `update_state` 里算（go2 = `gravity[:,2] <= 0.5` 即摔倒）；超时 `truncated` 由基类 `_compute_truncated` 按 `max_episode_steps` 自动算；`done = terminated | truncated`，PPO 自举经 `TransitionBootstrapContract`（区分 `timeout_terminal_mask` 与失败终止，等价 mjlab/Isaac 的 `time_out` 语义）。
- 改单项权重：`train ... reward.scales.tracking_lin_vel=2.0`（Hydra 覆盖）。

### 4.5 特权学习 / 蒸馏（prac_privileged 各 `\pz` + prac_visuomotor:727）

```python
# UniLab：特权 critic —— 特权量只进 critic 组、不进 actor 组（非对称 actor-critic）
# 机制：env.obs_groups_spec 声明 {"obs": A, "critic": C}（C>A，多出的就是特权维）；
#   base/observations.py split_obs_dict 把 obs["obs"]→actor、obs["critic"]→critic；
#   _compute_obs 里 actor 组只放真机可得量(+噪声)，critic 组追加特权量(base linvel 等，无噪声)。
@property
def obs_groups_spec(self): return {"obs": 49, "critic": 52}   # go2：多 3 维 = 特权 base linvel
# g1（人形，g1/joystick.py）：return {"obs": 98, "critic": 101}  # 多 3 维 = 特权 base linvel
# Hydra 路由：algo.obs_groups = {actor: [actor], critic: [critic]}（rough 任务用两组）；
#   flat 任务可只 {actor: [actor]}（对称）。维度自检入口：env.obs_groups_spec / get_critic_base_dim。
# 与 Isaac Lab ObsGroup 类比：obs↔policy 组、critic↔privileged+policy 组；
#   UniLab 无独立命名的 "privileged" 组——非对称性就是 critic 组多带 linvel（及 critic-only 缩放）。
```

```python
# UniLab：teacher→student 蒸馏 —— HORA 原生支持（in-hand 操作的 RMA 式时序自适应蒸馏）
# 这是 UniLab 真实的蒸馏机制（对照 RSL-RL DistillationRunner 的离线 BC）：
# 入口脚本 scripts/train_hora_distill.py + conf/hora_distill/，算法在 algos/torch/hora/distill.py。
# teacher 经 owner YAML 声明（conf/hora_distill/task/sharpa_inhand/mujoco.yaml）：
teacher:
  algo_family: ppo
  task: sharpa_inhand/mujoco_hora       # teacher = 在 HORA-特权任务上训的 PPO；checkpoint 路径运行时解析
# HoraDistillationTrainer（distill.py）核心：
#   - build_student_actor_and_normalizer(env, cfg)；_load_teacher_checkpoint→load_teacher_actor_weights
#   - 关键：只训练时序自适应模块 —— _trainable_parameters 仅对名字含 "adapt_tconv" 的参数置
#     requires_grad=True（TemporalAdaptationEncoder，models.py）；teacher 主干 + obs_normalizer 冻结(eval())。
#   - 即 stage-1 PPO teacher 用特权 priv_info 训练 → stage-2 student 从本体感知历史学一个时序卷积
#     适应编码器（Rapid Motor Adaptation 范式）。priv_info 必需（observations.py）。
# 运行：uv run train --algo ppo --task sharpa_inhand --sim mujoco --profile hora   # 训 teacher
#       python scripts/train_hora_distill.py task=sharpa_inhand/mujoco              # 蒸馏 student
# 交互回放：play_interactive.py 的 hora_distill 路径（create_hora_distill_playback_session）。
# 对照 RSL-RL DistillationRunner：UniLab 的 HORA 是「特权→适应模块」蒸馏（非通用 DAgger 在线交替）；
#   通用 teacher-student DAgger / camera-conditioned student 非内置（见 §6）。
```

### 4.6 域随机化（prac_domain_rand 各 `\pz` + prac_quad_loco:1050）

```python
# UniLab：DR = 任务 DomainRandConfig（dataclass）+ DomainRandomizationProvider/Manager + 后端能力声明
# (1) 配置（go2 rough 实例，conf YAML）—— startup 个体差异 + interval 动态扰动 + scale 模式
env:
  domain_rand:
    randomize_base_mass: true          # reset：基座质量 +U(added_mass_range)
    added_mass_range: [-1.0, 3.0]
    random_com: true                   # reset：质心 x 偏移
    randomize_kp: true                 # reset：PD kp 乘 U(kp_multiplier_range)
    kp_multiplier_range: [0.5, 2.0]
    randomize_kd: true
    kd_multiplier_range: [0.5, 2.0]
    push_robots: true                  # interval：每 push_interval 步施加速度扰动
    push_interval: 625
    max_force: [1.0, 1.0, 0.5]
# (2) 机制（dr/manager.py）：reset 时 provider.build_reset_plan→后端 set_state(randomization=payload)；
#   interval 时 apply_interval_randomization_if_due(step)→后端 apply_interval_randomization。
#   后端经 get_dr_capabilities() 声明支持的项（DomainRandomizationCapabilities）；
#   不支持的 reset 项被 filter_reset_payload 自动跳过(打 warning，不报错)——这是「两后端 DR 能力不同」的降级。
# (3) 摩擦随机化（对照 mjlab geom_friction / Isaac randomize_rigid_body_material）：
#   geom_friction 是 reset 项之一（dr/types.py RESET_TERM_GEOM_FRICTION）；
#   在 DomainRandConfig 加 randomize_ground_friction + ground_friction_multiplier_range（scale 模式保物理一致）。
# 验证 per-env 摩擦不同：reset 后读后端 per-world geom_friction，跨 env std>1e-6 即生效。
```
- **scale 模式保物理一致**：DR 用乘性（`*_multiplier_range`）而非绝对值，类似 mjlab 的 `operation="scale"`。`pseudo_inertia`（mjlab 独有的正定惯量随机化）UniLab **无对应**（reset 项只到 `body_inertia/body_ipos/body_iquat` 级，不保证正定，见 §6）。

### 4.7 四足 / 人形 locomotion + 资产（prac_quad_loco / prac_humanoid_loco / prac_assets / prac_humanoid_wbc）

```bash
# UniLab：四足模型来源 + 站立/短跑冒烟（对照 MuJoCo Menagerie / USD）
# 模型来源：MJCF（mujoco 后端，src/unilab/assets/robots/go2/scene_flat.xml）或 Motrix XML；
#   不用 USD。资产经 unilab-pull-assets 从 HF 拉，或 unilab-import-robot 从 URDF 导入。
# 站立冒烟（zero 动作不直接支持，用极短训练代替）：
uv run train --algo ppo --task go2_joystick_flat --sim mujoco \
    algo.num_envs=64 algo.max_iterations=3 training.no_play=true
# 短跑 + 回放：
uv run train --algo ppo --task go2_joystick_flat --sim mujoco \
    algo.num_envs=2048 algo.max_iterations=300
uv run eval --algo ppo --task go2_joystick_flat --sim mujoco --load-run -1 --render-mode record
```
```text
% UniLab 四足/人形任务清单（实测注册，26 个）
四足  : Go2JoystickFlat/Rough、Go1JoystickFlat/Rough、Go2WJoystickFlat/Rough（轮足）、
        Go2FootStand（直立）、Go2ArmManipLoco（带臂 loco-manipulation）
人形  : G1WalkFlat/Rough、G1MotionTracking(+SAC/Deploy)、G1BoxTracking、G1ClimbTracking、
        G1FlipTracking(+SAC)、G1WallFlipTracking(+SAC)、G1WBTObs、X2WallFlipTracking
% rough 起步、flat 做减法：rough owner 含 terrain.generator + terrain_curriculum + 更全 DR + 更多
%   reward 项（joint_torques_l2/feet_air_time/feet_gait/...）；flat owner 砍掉地形与部分 reward。
% 资产组合：SceneCfg.model_file（机器人 XML）+ fragment_files（拼任务/地形片段）+ terrain.generator
%   （TerrainGeneratorCfg，size/num_rows/num_cols/border_width，等价共享 base + robot override）。
% 人形 WBC（上下身解耦/接触分解/mask 多模态）：UniLab 有 G1 motion-tracking 系列任务可作骨架，
%   但 ExBody2/WoCoCo/HOVER 的具体 reward 结构需在 G1 任务里自定义（见 §6 上身 landmark 等需自实现）。
```

### 4.8 模仿 / 动作跟踪 / 多模态（prac_imitation / prac_motion_imitation / prac_multimodal）

```text
% UniLab：动作跟踪（motion tracking）—— G1 / X2 原生支持，对照 BeyondMimic / ProtoMotions
任务名/数据格式/回放（实测注册，envs/motion_tracking/{g1,x2}）：
  G1MotionTracking   : 通用动作跟踪；类 G1MotionTrackingEnv(G1BaseEnv)，cfg G1MotionTracking{,Env,Deploy}Cfg
  G1FlipTracking / G1WallFlipTracking / G1BoxTracking / G1ClimbTracking : 特技/交互跟踪
  G1MotionTrackingSAC / G1FlipTrackingSAC / ... : off-policy(SAC) 变体
  G1WBTObs           : 部署导向变体（去掉真机不可得通道 base_lin_vel/motion_anchor_pos_b，加本体感知历史）
  demo dance / boxtracking / wallflip : 对应任务的预训练回放（uv run demo <name>）
数据格式 = NPZ 参考运动（g1/motion_loader.py，class MotionLoader）：
  键 fps/joint_pos/joint_vel/body_pos_w/body_quat_w/body_lin_vel_w/body_ang_vel_w（→ @dataclass MotionData）；
  默认 assets/motions/g1/dance1_subject2_part.npz（注释含 LAFAN/gangnam_style 备选）；
  MotionSampler 采帧，reset 经 _build_motion_reference_state 建参考态。
跟踪奖励（G1MotionTrackingCfg.reward.scales，确实跟参考轨迹）：
  motion_global_root_pos(0.5) / motion_global_root_ori(0.5) / motion_body_pos(1.0) /
  motion_body_ori(1.0) / motion_body_lin_vel(1.0) / motion_body_ang_vel(1.0) /
  motion_ee_body_pos_z / motion_joint_pos / motion_joint_vel
跟踪观测（actor，_compute_obs）：command(2n) + motion_anchor_pos_b(3) + motion_anchor_ori_b(6)
  （motion anchor = 目标躯干位姿相对机器人，_write_motion_anchor_transform 写入）。
切换对抗算法（AMP 类）：UniLab 主线是显式跟踪 + RL（PPO/APPO/SAC），
  对抗判别器（AMP/ASE）非内置一等公民（见 §6）——CALM 文本条件 / MaskedMimic inpainting 需自实现。
部署导出（deploy 闭环，scripts/deploy/）：export_motion_bin.py 把参考运动导成 .bin；
  export_deploy_config.py 导 deploy_config.yaml；sim_prototype.py 用 ONNX 策略在 MuJoCo 逐段
  校验 obs 装配 == 训练侧（写 C++ State_WBT 前的验证，G1WBTObs 任务专为此设计）。
```

### 4.9 操作 / 灵巧手（prac_manipulation）

```bash
# UniLab：in-hand 操作（对照 Isaac Lab manipulation）—— Allegro / Sharpa 灵巧手
# 任务（实测注册）：AllegroInhandRotation(+Grasp)、SharpaInhandRotation(+Grasp)
uv run train --algo ppo --task sharpa_inhand --sim mujoco
uv run demo inhandgrasp                       # Sharpa in-hand 预训练回放
# HORA 课程蒸馏（in-hand 旋转，--profile hora 走 mujoco_hora.yaml owner）：
uv run train --algo ppo --task sharpa_inhand --sim mujoco --profile hora
# 配置位置：conf/ppo/task/{allegro,sharpa}_inhand/{mujoco,motrix}.yaml；
#   HORA 蒸馏 conf/hora_distill/task/sharpa_inhand/mujoco.yaml。
# 类名：AllegroRotationPPO(AllegroBaseEnv) / SharpaInhandRotationEnv(SharpaInhandBaseEnv)（各 +Grasp 变体）。
# 物体位姿观测（确认）：actor "policy frame" 含 object_pos + object_quat（sharpa 拼 [object_pos_f, object_quat]
#   + object_pos_anchor，跟踪 prev_object_pos/quat）；critic 追加特权 priv_info（friction_scale /
#   gravity_direction / object_pos_delta）。sharpa obs_groups_spec = {"obs": policy_dim,
#   "critic": policy_dim + critic_info_dim}；allegro = {"obs": NUM_OBS_PER_STEP * NUM_LAG_STEPS}（单组）。
#   allegro reward fns: rotate / obj_linvel / pose_diff / torque / work / drop。
```

### 4.10 Sim2Real / 部署（prac_training_pipeline deploy + prac_quad_loco:1444）

```bash
# UniLab：训练→部署闭环（对照 mjlab/Isaac Lab 共用 RSL-RL ONNX 导出）—— UniLab 确实导出可部署 ONNX
# (1) 各算法都导 policy.onnx 并用 onnxruntime 自验：
#   PPO(play 路径)  : eval --algo ppo ... --load-run -1 → runner.export_policy_to_onnx + export_policy_to_jit
#   APPO            : train_appo.py:284 包 _DeterministicAPPOActor(actor.mlp)，torch.onnx.export 后
#                     起 ort.InferenceSession(输入名 "obs") 比对 max/mean diff，打印 "ONNX export verified OK"
#   off-policy      : train_offpolicy.py:557，由 training.export_onnx=true(默认) 控；actor.as_export_module()
#   HIM-PPO/MLX     : 同样导 policy.onnx + onnxruntime 自验
uv run eval --algo ppo --task go2_joystick_flat --sim mujoco --load-run -1   # → log 目录出 policy.onnx
# 注意时机差异：与 mjlab（训练后自动导）不同，UniLab PPO 在 play/eval 阶段导（故 training.no_play=true 时不导）；
#   off-policy 由 export_onnx flag 控、训练流程内导。
# (2) 部署工具链（scripts/deploy/，UniLab 比 mjlab 更完整的一处）：
#   export_deploy_config.py  —— 导 deploy_config.yaml（obs_layout/obs_dim/default_angles/action scale）
#   export_motion_bin.py     —— 参考运动导成 .bin（动作跟踪部署）
#   sim_prototype.py         —— 用 ONNX 在 MuJoCo 跑，逐段校验 obs 装配 == 训练侧（写 C++ State_WBT 前的验证）
#   prepend_warmup.py / append_cooldown.py —— 给部署轨迹加 warmup/cooldown
# 归一化：empirical_normalization=true 时归一化层经 rsl-rl 写进 ONNX（与 mjlab 一致）。
# joint ordering：部署侧由 deploy_config.yaml 显式记录（sim_prototype.py 按 cfg["obs_layout"] 装配）。
# sim2sim 关卡：迁移到另一后端回放前，training.sim2sim_strict=true 会强制契约一致（见 §2.8）。
```

### 4.11 大规模训练 / 诊断（prac_training_pipeline 诊断 + prac_physics 排查 + 架构对照章）

```bash
# UniLab：大规模 + 诊断（对照 mjlab --gpu-ids torchrunx / NaN guard / viser）
# (1) 多 GPU（off-policy 原生 rank-local 管线）：
uv run train --algo sac --task g1_walk_flat --sim mujoco training.num_gpus=4 \
    training.multi_gpu_sync_mode=local_sgd
#   → MultiGPUCPUPinnedReplayPipeline：每 rank 独立 host slot + H2D stream，不经 rank0 漏斗。
# (2) 异步 collector-learner（APPO，最能体现异构）：
uv run train --algo appo --task go2_joystick_flat --sim mujoco algo.num_envs=2048
#   仪表盘有 Weight Sync / H2D Copy / Rollouts Read / Staging Pool / Sync Collect 面板（实测）。
# (3) NaN 守卫（默认开）+ 离线复现：
uv run train ... training.nan_guard.enabled=true training.nan_guard.output_dir=/tmp/nan
#   NanGuard(utils/nan_guard.py)：capture(physics_state) 环缓存最近 buffer_size 帧；
#   check(obs,reward,step)/check_ctrl(ctrl) 返回非有限值的 env ids；
#   dump(...) 写 nan_dump_<ts>_step<N>.npz（含 states 环缓冲 + meta_* + 复制 model_file + latest 软链）。
unilab-viz-nan /tmp/nan/<dump>          # replay_dump：用 mujoco.viewer 回放出事前那几帧（复现）
# (4) profiling（Perfetto/Chrome-trace JSON，看 collector/learner/H2D 重叠）——off-policy 路径特性：
uv run train --algo sac ... training.trace_enabled=true training.trace_output_dir=/tmp/trace \
    training.trace_cuda_events=true
#   → TraceRecorder(logging/trace_event.py) 写 perfetto_offpolicy_timeline.json（{"traceEvents":[...]}）；
#     用 scripts/analyze_offpolicy_trace.py 解析（看 learner/training_e2e 等时长/间隙）。
#   注意：无字面 --profile flag（走 Hydra training.trace_enabled）；APPO 路径未接 TraceRecorder（trace 是 off-policy 特性）。
# (5) 内存预算守卫（共享内存特有失败面）：超 /dev/shm 80% 直接 MemoryError；
#   提示减小 algo.num_envs 或 algo.replay_buffer_n；UNILAB_SKIP_MEMORY_CHECK=1 关警告。
# 显存/headless：CPU-sim 不占 GPU 仿真显存（GPU 只放策略）；--render-mode none 纯 headless。
```
```text
% UniLab 数据流（对照 mjlab WarpBridge / Isaac Lab wp.array 管线）—— prac_physics
描述→编译→CPU→桥接：
  SceneCfg.model_file(MJCF/Motrix XML) → 后端编译（mujoco-uni / motrixsim）
  → CPU 多线程批量 step（NpEnv，每 env 独立物理状态）
  → 共享内存 IPC（packed ReplayBuffer / RolloutRingBuffer，np.float32）
  → CPU→GPU H2D 管线（pinned-host + 双缓冲 + 可选原生 native_h2d_ext.cpp）
  → GPU 策略学习（torch）
  与 GPU-sim 的根本不同：物理数据原生在 CPU，跨边界搬到 GPU 是显式可计量步骤
  （mjlab/Isaac 数据本就在 GPU、无此边界）。
% 模型格式（prac_physics）：MJCF（mujoco 后端）/ Motrix XML（motrix 后端）；不用 USD；
%   URDF 经 unilab-import-robot 导入；无 MJCF↔USD 互转（Isaac Lab 才有 convert_mjcf/urdf）。
% 排查工具（prac_physics）：--render-mode none headless；--profile chrome-trace；
%   NanGuard + unilab-viz-nan；run_summary.json/run_config.json（记硬件/seed/git/任务）；
%   内存预算守卫。无 GPU 显存查询专用入口（CPU-sim 不占仿真显存）。
```

### 4.12 视觉运动控制（prac_visuomotor）

> 见 §6：UniLab **当前任务集无相机/视觉策略任务**（locomotion/manipulation/motion_tracking 均本体感知）。视觉运动控制章的 UniLab 列**如实标注「当前无视觉任务，待官方补」**，不编造相机 DR / depth obs 接口。Motrix 有渲染后端（teaser 用 MotrixSim 渲染）但用于回放可视化，非训练时相机观测。

### 4.13 Actuator 建模（prac_actuator 各 `\pz` + prac_manager_arch:801）

```python
# UniLab：actuator 配置（对照 mjlab <position>/<dcmotor> / Isaac ImplicitActuatorCfg/DCMotorCfg/ActuatorNetMLPCfg）
# UniLab 的 actuator 模型 = 后端位置执行器 + PD 增益（Ideal PD 层级），不在 task 代码里配 actuator 类树：
# (1) PD 增益经 create_backend 传给后端（locomotion/common/base.py PdControlConfig）：
control_config: PdControlConfig  # 字段 Kp(默认 35.0), Kd(默认 0.5), action_scale(0.25), simulate_action_latency
backend = create_backend(backend_type, cfg.scene, num_envs, cfg.sim_dt,
                         position_actuator_gains={"kp": cfg.control_config.Kp,
                                                  "kd": cfg.control_config.Kd})
# (2) 动作→力矩：apply_action 给位置目标 ctrl = act*action_scale + default_angles，
#     后端位置执行器按 kp/kd 算力矩（= Ideal PD 层级）。力矩/速度饱和由 MJCF/Motrix 模型本身的
#     actuator 定义（ctrlrange/forcerange/gear）承担，不在 Python 配 DCMotorCfg 等价类。
# (3) Hydra 改增益：env.control_config.Kp=40 env.control_config.Kd=1.0（go2 rough 还有 hip/non_hip 分项 action_scale）。
# 层级对照：
#   Ideal PD       : UniLab 原生（PdControlConfig + 后端位置执行器）✓
#   DC Motor 饱和  : 经 MJCF <general>/forcerange 在模型层表达（非 Python actuator 类）；部分由后端
#   Actuator Network: UniLab 无 learned actuator 网络等价物（见 §6）—— 需自实现或靠 DR 覆盖
# kp/kd 域随机化（sim2real 主力，替代精确建模）：DomainRandConfig.randomize_kp/kd + *_multiplier_range
#   （reset 项，MuJoCo 后端支持 kp/kd；Motrix 经能力门控，见 §2.4）。
```

---

## 5. 实跑验证记录（本轮，gpufree，2026-06-24）

全部产物写 `/root/gpufree-data/tmp/unilab_verify` 并删除，跑前后盘 36G 可用。

- **PPO PASS（motrix 后端）**：`python scripts/train_rsl_rl.py task=go2_joystick_flat/motrix training.sim_backend=motrix algo.num_envs=32 algo.max_iterations=2 training.no_play=true` → 跑满 2 iter，~4643 steps/s，9 项 reward 正常（`reward/tracking_lin_vel/swing_feet_z/...`），value/surrogate/entropy loss 正常，EXIT=0，产 `model_0/1.pt + events.tfevents + run_config.json + run_summary.json + git/UniLab.diff`。**无 ONNX**（no_play 跳过 play→不导 ONNX，证实 ONNX 在 play 阶段才导）。
- **APPO PASS（mujoco 后端，异步 collector-learner）**：`python scripts/train_appo.py task=go2_joystick_flat/mujoco training.sim_backend=mujoco algo.num_envs=32 algo.max_iterations=3 training.no_play=true` → 跑满 3 iter，13120 env steps，EXIT=0。**仪表盘直接证实 IPC 架构**：面板含 `Weight Sync 0.9ms`、`H2D Copy 1.4ms`、`Rollouts Read 2.0`、`Staging Pool 3/3`、`Sync Collect ✓`、`Available On Arrive 2.0`、`Vtrace/Rho Clip Fraction`——即 SharedWeightSync + RolloutRingBuffer + replay pipeline H2D + V-trace 全部在跑。产 `model_3.pt`。
- **任务枚举 PASS**：`registry.ensure_registries(); list_registered_envs()` → **26** 个注册任务（见 §4.7 清单），均双后端（个别 Go2FootStand/X2WallFlipTracking 仅 mujoco）。
- **CLI 后端切换语义确认**：owner YAML `training.task_name` 绑定 CamelCase 注册名；`--task`/`task=` 用小写 slug 对应 conf 目录。
- 已验证的 API：`registry.envcfg/env/make/ensure_registries`、`NpEnv.step` 模板、`run_reward_dispatch`（reward/{name} log 键实测出现）、`obs_groups_spec`、Hydra 覆盖 `algo.num_envs/max_iterations/training.no_play/training.log_root`、双后端 `--sim {mujoco,motrix}`。

---

## 6. UniLab 真实欠缺 / 需谨慎的特性（不编造，对应章如实标注）

1. **视觉 / 相机策略**：当前 26 个注册任务**全是本体感知**（无 RGB/depth 观测任务）。Motrix 有渲染后端但用于回放可视化。→ prac_visuomotor 的 UniLab 列标「当前无视觉运动控制任务，相机 DR / depth obs 接口待官方提供」。
2. **对抗模仿（AMP/ASE/判别器）+ 通用蒸馏**：主线是**显式动作跟踪 + RL**（G1MotionTracking 等）。无内置 AMP 判别器 reward 一等公民。→ prac_imitation/prac_motion_imitation 的对抗/CALM/MaskedMimic inpainting 列标「需自实现」。**注意：teacher→student 蒸馏并非完全缺失**——UniLab 有 **HORA** 原生蒸馏（特权 teacher → 时序自适应 `adapt_tconv` student，RMA 范式，见 §4.5），但**通用 DAgger 在线交替 / camera-conditioned student** 不内置（→ prac_privileged:954、prac_visuomotor:727 标「HORA 式适应蒸馏 ✓，通用 DAgger/视觉 student 需自实现」）。
3. **`pseudo_inertia` 正定惯量随机化**（mjlab 独有）：UniLab reset DR 到 `body_inertia/body_ipos/body_iquat` 级，**不保证扰动后惯量正定**。→ prac_domain_rand / prac_actuator 惯量章标「UniLab 无 Cholesky 重构的物理合法惯量 DR」。
4. **σ（tracking_sigma）课程**：无现成「按步收紧 reward std」课程函数。有 `PenaltyCurriculum`（按 episode 长度缩放**负权重**）+ `TerrainCurriculumCfg`（地形等级）。→ prac_reward σ 课程列标「需手动分阶段或自定义回调」。
5. **learned actuator / 两级行星齿轮反射惯量**（mjlab 有）：UniLab 用后端位置执行器（`{kp,kd}` PD），无学习型执行器网络 / 反射惯量工具族。→ prac_actuator 标「UniLab 用 builtin 位置 PD 执行器，无 actuator network」。
6. **Manager 体系**：UniLab **不是** Manager-Based——无 `ObservationManager/RewardManager/EventManager/CurriculumManager/MetricsManager/RecorderManager` 类。功能分散在 `update_state`（obs/reward/done）+ `DomainRandomizationManager`（DR）+ `PenaltyCurriculum`（课程）+ contract 类。→ prac_manager_arch 必须如实写「第三种非 Manager-Based 架构」，不能套 manager 类名。
7. **ONNX 导出时机（非缺失，仅时机不同）**：UniLab **确实导出可部署 ONNX 并用 onnxruntime 自验**（APPO/off-policy/HIM/MLX 都导 `policy.onnx`）。差异在时机：PPO 在 **play/eval 阶段**导（`export_policy_to_onnx`，故 `training.no_play=true` 时不导）；off-policy 由 `export_onnx` flag 控、训练内导——与 mjlab「训练后自动导」不同。→ prac_training_pipeline 导出列如实写「支持 ONNX 导出 + onnxruntime 自验，时机随算法（PPO 在 play 阶段）」。
8. **腱驱 / 位点力动作**（mjlab 有 TendonLength/Effort/SiteEffort）：未见 UniLab 等价动作族（动作=关节位置目标）。→ prac_manipulation 灵巧手腱驱列谨慎，标「未确认」。

> §B.2 已追加 source-vs-docs 差异条目（见「问题与解决记录.md」）。
