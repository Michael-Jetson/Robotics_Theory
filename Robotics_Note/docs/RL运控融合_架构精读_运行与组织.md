# RL 运控部 · 架构精读：运行逻辑与组织逻辑（Isaac Lab × mjlab，源码级）

> 教学定位：官方文档是**技术参考**（讲"有哪些 API"），本材料是**架构教学**（讲"为什么这么组织、运行时数据怎么流、字符串任务名如何变成跑起来的 env"）。所有结论**逐行读源**得来，关键处标 `file:line`；用 import-introspection 印证的另标注。
>
> 素材来源（gpufree，2026-06-24）：
> - Isaac Lab：`/root/isaaclab/source/{isaaclab, isaaclab_tasks, isaaclab_assets, isaaclab_rl}`（`/root/isaaclab` 是 `/workspace/isaaclab` 的 symlink，故 traceback 显示 `/workspace/...`，同一份源）。版本 repo=2.3.2 / 内部包号=0.54.2。
> - mjlab：`/root/gpufree-data/envs/mjlab/lib/python3.12/site-packages/mjlab`（pip 1.4.0）。
> - 印证手段：mjlab `import`-introspection **完全可用**（已核多处签名）；Isaac Lab **仅顶包**可 import，`isaaclab.managers` 因间接 import USD(`pxr`) 在本机失败（见问题记录 C.2），故 Isaac Lab manager 事实以读源 + 官方文档为准。
> - UniLab（第三框架）由另一 agent 覆盖；本文在对比处留"UniLab 钩子"标记 `【UniLab 对比钩子】`，待并。

---

## 第 0 章 速读：两个 `env.step` 端到端（→ Ch04）

### 0.1 Isaac Lab `ManagerBasedRLEnv.step()`（`isaaclab/envs/manager_based_rl_env.py`）

一个**环境步 = 1 次策略决策 = decimation 个物理子步**。完整控制流：

```text
step(action):
 1. action_manager.process_action(action.to(device))   # 策略率，做一次：切片+预处理(缩放/偏置/IK)
 2. recorder_manager.record_pre_step()
 3. for _ in range(decimation):                          # 物理率，做 decimation 次
       _sim_step_counter += 1
       action_manager.apply_action()                     #   把已算好的目标写进各 term 缓冲
       scene.write_data_to_sim()                         #   缓冲 → 仿真器(PhysX)
       sim.step(render=False)                            #   推进一个物理子步
       (按 render_interval 决定是否 sim.render())
       scene.update(dt=physics_dt)                       #   刷新资产/传感器缓冲
 4. episode_length_buf += 1; common_step_counter += 1    # 计数器
 5. reset_buf = termination_manager.compute()            # 终止：terminated | time_outs（reset 前状态算）
    reset_terminated = termination_manager.terminated
    reset_time_outs  = termination_manager.time_outs
 6. reward_buf = reward_manager.compute(dt=step_dt)      # 奖励：Σ weight·func·dt（reset 前状态算）
 7. (若有 recorder active) obs=obs_manager.compute(); recorder.record_post_step()
 8. reset_env_ids = reset_buf.nonzero()
    if 有终止 env:  _reset_idx(reset_env_ids)            # 仅复位这些 env
 9. command_manager.compute(dt=step_dt)                  # 刷新指令（如速度目标）
10. if "interval" in event modes: event_manager.apply("interval", dt=step_dt)
11. obs_buf = observation_manager.compute(update_history=True)  # 注意：在 reset 之后取 → reset env 拿到新 obs
return obs_buf, reward_buf, reset_terminated, reset_time_outs, extras
```

**教学要点**：
- **奖惩在 reset 前算、obs 在 reset 后取**。终止/奖励看的是"撞墙那一刻"的状态；observation 故意放在 `_reset_idx` 之后，让被复位的 env 返回**新一回合初始 obs**（源注释明说 "done after reset to get the correct observations for reset envs"）。时序图务必画对这个先后。
- **返回值是 5 元组**：`(obs, reward, terminated, time_outs, extras)`——terminated 与 time_outs **分开返回**（Gymnasium 语义，供 RL 库正确做 value bootstrap）。
- `step_dt = sim.dt × decimation`（控制周期），`physics_dt = sim.dt`（物理周期）。`render_interval` 默认应 ≥ decimation，否则每步多次渲染（源里有 warning）。

### 0.2 mjlab `ManagerBasedRlEnv.step()`（`mjlab/envs/manager_based_rl_env.py`）

```text
step(action):
 0. (若 auto_reset=False 且有未复位的 pending env → 抛 RuntimeError)
 1. extras["log"] = {}; action_manager.process_action(action.to(device))
 2. for _ in range(decimation):
       _sim_step_counter += 1
       action_manager.apply_action()
       scene.write_data_to_sim()
       sim.step()                                        # mjwarp.step（GPU，CUDA-graph 重放）
       scene.update(dt=physics_dt)
       metrics_manager.compute_substep()                 # ← mjlab 独有：子步级 metric 累加
 3. episode_length_buf += 1; common_step_counter += 1
 4. reset_buf = termination_manager.compute()            # （派生量此时滞后一个子步，见下）
    reset_terminated / reset_time_outs
 5. reward_buf = reward_manager.compute(dt=step_dt); metrics_manager.compute()
 6. reset_env_ids = reset_buf.nonzero()
    if auto_reset and 有终止: _reset_idx(...); scene.write_data_to_sim()
 7. sim.forward()                                        # ★ 全程只调一次 forward（见下）
 8. command_manager.compute(dt=step_dt)
 9. if "step"     in modes: event_manager.apply("step", dt)
    if "interval" in modes: event_manager.apply("interval", dt)
10. sim.sense()                                          # ← BVH/相机/raycast 感知（独立 CUDA-graph）
11. obs_buf = observation_manager.compute(update_history=True)
12. (auto_reset 收尾 / 否则 _manual_reset_pending[reset]=True); recorder.record_post_step()
return obs_buf, reward_buf, reset_terminated, reset_time_outs, extras
```

**mjlab 独有的两点教学价值**（源 docstring ~60 行专门讲）：
1. **"单次 forward" 优化**。MuJoCo 的 `mj_step` 内部"先 forward-kinematics 再积分"，所以**步后**派生量（`xpos/xquat/site_xpos/cvel/sensordata`）滞后 `qpos/qvel` 一个子步。mjlab **不**在 decimation 后 + reset 后各调一次 `forward()`，而是**只在算 obs 前调一次** `sim.forward()`：一次刷新所有 env——非 reset env 拿 post-decimation 运动学，reset env 拿 post-reset 运动学。代价是 termination/reward manager 看到的派生量**滞后一个子步**，但"滞后一致"（每 env 每步都一样）⇒ MDP 良定义，价值函数能学到正确映射。这是 **MuJoCo-warp 后端特有取舍**，Isaac Lab（PhysX 步后状态即时一致）没有。
2. **`auto_reset` 可关**（`cfg.auto_reset:bool=True`）。关掉后 `step()` 返回**真终止 obs**，调用方须自己 `reset(env_ids=...)`，否则下次 `step()` 触发 `_manual_reset_pending` 守卫抛错。给自定义训练循环用；自带 train.py 走 rsl_rl 不驱动手动 reset 故默认 True。Isaac Lab 无此开关，永远 auto-reset。

### 0.3 两个 step 的对照（建议做"并排时序图"）

| 维度 | Isaac Lab | mjlab |
|---|---|---|
| 后端 | Omniverse / PhysX（GPU TensorAPI） | `mujoco_warp` + warp + torch |
| 物理推进 | `sim.step(render=False)` | `sim.step()`（CUDA-graph 重放 `mjwarp.step`） |
| 步后状态一致性 | 即时一致 | 派生量滞后一子步 → "单次 forward" 补偿 |
| 感知阶段 | 渲染并入物理循环（按 render_interval） | 独立 `sim.sense()`（专属 sense_graph） |
| 子步级 metric | 无 | `metrics_manager.compute_substep()` |
| auto-reset | 强制 | 可关（`auto_reset`） |
| 奖励 ×dt | 永远 ×dt | 由 `scale_rewards_by_dt` 开关控 |
| 返回 | `(obs,rew,terminated,time_outs,extras)` | 同（结构一致） |

> 【UniLab 对比钩子】UniLab 是"CPU(NumPy)仿真 + GPU 策略"异构，env 核是 `NpEnv/NpEnvState`，step 语义由 UniLab agent 给出，此处留并表行。

---

## 第 1 章 Manager 系统运行时（→ Ch04 主体）

### 1.1 一个 Manager 是怎么"从 Cfg 长出来 + 被调用"的

两框架共享同一抽象：**Manager 持有一组 Term，每个 Term 由一个 TermCfg 描述**。但实例化与调用顺序由 env 的 `load_managers()` 严格编排。

**Isaac Lab 构建顺序**（`ManagerBasedEnv.load_managers` + `ManagerBasedRLEnv.load_managers`，顺序有注释强调"order is important"）：
```text
command_manager → (父类) event_manager 已建 + recorder_manager + action_manager + observation_manager
→ termination_manager → reward_manager → curriculum_manager → _configure_gym_env_spaces → (startup 事件)
```
顺序理由：observation 需要先知道 command 与 action；reward 需要先知道 termination。

**mjlab 构建顺序**（`load_managers`，注释 "Order is important!"）：
```text
event_manager（最先！）→ sim.expand_model_fields(event.domain_randomization_fields)
→ command(或 NullCommandManager) → action → observation
→ termination → reward(scale_by_dt=...) → curriculum/metrics/recorder（空则 Null*）
→ _configure_gym_env_spaces → (startup 事件)
```
mjlab **把 event_manager 放在最前**，是因为紧接着要用它收集的 `domain_randomization_fields` 去 `sim.expand_model_fields(...)` 预分配 per-world 内存（见 1.6 皇冠明珠）。

### 1.2 Term 的两大家族（关键教学概念，Isaac Lab 尤其清晰）

- **函数式 Term**（obs / reward / termination / curriculum / event）：TermCfg 持有 `func`（字符串路径**或**可调用），加 `params: dict`。多为无状态 MDP 函数；也可是继承 term 基类的有状态类。构建时走 `_resolve_common_term_cfg`：
  - 字符串 → 可调用：`string_to_callable`（dotted-path import）。
  - 静态签名校验：`inspect.signature` 比对，要求除前 `min_argc` 个位置参（Isaac Lab：obs/reward/termination `min_argc=1` 即 `func(env, **params)`；event/curriculum `min_argc=2` 即 `func(env, env_ids, **params)`）外，其余形参名 = `params` 键 ∪ 带默认值的参数。**不匹配在构建期就报错**（早于任何 sim step）。
  - `SceneEntityCfg.resolve(scene)`：把"实体名 + joint/body 正则"解析成**具体整数索引张量**（须等仿真 playing，因要查活的 articulation）。Isaac Lab 用 timeline PLAY 回调延迟到 play 时做（`manager_base.py` order=20 回调）。
- **类式 Term**（action / command）：TermCfg 持有 `class_type`（**不是** `func`），由 manager **实例化** `class_type(cfg, env)`。永远是有状态对象。**绕过** `_resolve_common_term_cfg`。

> Isaac Lab term 基类真名（import-rename 坑，见问题记录 B）：`ObservationTermCfg/RewardTermCfg/TerminationTermCfg/EventTermCfg/CurriculumTermCfg/ActionTermCfg/CommandTermCfg/ObservationGroupCfg`；用户常 import 重命名为 `ObsTerm/RewTerm/DoneTerm/EventTerm/CurrTerm/ObsGroup`。**mjlab 不提供这些短别名**，只导出全名；mjlab **唯一**的 term 基类是 `ManagerTermBase`。

### 1.3 数据布局：SoA + 名↔索引（两框架共性）

每个 manager 内部是 **index-aligned 平行列表**（或按 group/mode 分桶的列表字典）：`_term_names[i]`、`_term_cfgs[i]`、以及张量某一列 `[:, i]` **共享同一索引**。所有状态张量形状 `(num_envs, ·)`、驻留 `env.device`（GPU）。
- **Term 顺序 = 配置声明顺序**（Python dict/类体属性插入序）。这对 **obs 拼接顺序**和**动作向量切片顺序**是**承重**的——改配置字段顺序会悄悄改变 obs/action 向量布局。
- 终止/动作另维护 `name→idx` 映射（O(1) 查列）。

### 1.4 Observation Manager：group → term → 拼接张量

两框架都是"分组、组内拼接"。

**逐 term 处理流水线**（顺序重要）：
- Isaac Lab（`observation_manager.py:compute_group` ~393-407）：`compute → modifiers → noise → clip → scale`，输出 `.clone()`（防止别名到资产内部张量）。源注释解释 **noise 在 clip/scale 之前**：保持噪声对真实信号幅度的忠实，避免被裁剪/缩放人为压缩或放大。
- mjlab（`observation_manager.py` docstring + `compute_group`）：`compute → noise → clip → scale → (逐 term NaN 检查) → delay → history`。

**拼接与维度**：`concatenate_terms=True` 时 `torch.cat(组内各 term, dim=concat_dim)` → `(num_envs, group_dim)`；维度核算在**构建期**完成（`group_obs_dim/group_obs_concatenate`）。group 级开关覆盖 term 级：`enable_corruption=False` 把组内所有 noise 置空（eval 用干净 obs）；group `history_length` 覆盖 term 的。

**mjlab 比 Isaac Lab 多的两件**（真实工程价值）：
- **延迟建模 `DelayBuffer`**：`delay_min_lag/delay_max_lag/delay_hold_prob/delay_update_period/delay_per_env/delay_per_env_phase`——逐 env 采样滞后步数、保持概率、更新周期，模拟真实传感器/通信延迟。Isaac Lab base 无此延迟模型。
- **NaN 策略**：group 级 `nan_policy ∈ {disabled,warn,sanitize,error}` + `nan_check_per_term`，比 Isaac Lab 更细。
- **compute 缓存**：`update_history=False` 且已算过则返回 `_obs_buffer`，防止同一步两次调用把 delay/history 缓冲重复推进。

**obs 分组的 actor/critic 语义**（非对称 actor-critic 的关键）：env 提供若干 obs group（如 `policy`/`privileged`/`images`），RL 侧 `obs_groups: dict[str,list[str]]` 把它们映射到算法预定义的 obs-set。Isaac Lab 文档示例用 `{"policy":[...], "critic":[...]}`（`rl_cfg.py:170`），locomotion 实配用 `{"actor":[...], "critic":[...]}`——**两种 key 都合法**，取决于底层算法的 set 名（rsl-rl actor-critic 用 actor/critic）。mjlab 配置里 obs group 直接命名 `actor`/`critic`（见第 2 章示例）。

### 1.5 Reward / Termination Manager：加权和、episodic 日志、terminated/time_out 拆分

**Reward**（两框架同构）：
- `compute(dt)`：`value = func(env,**params) × weight × dt`；累加进 `_reward_buf`（净奖励）与 `_episode_sums[name]`（整回合累加器）；weight=0 的 term 直接跳过（微优化）；mjlab 额外 `nan_to_num`（坏物理保护）。返回 `(num_envs,)`。
- **dt 缩放的 WHY**：每项 ×dt 使奖励幅度对控制周期不变——改 decimation/dt 时权重仍可迁移。Isaac Lab **永远** ×dt；mjlab 由 `RewardManager(scale_by_dt=...)` 控（env 透传 `cfg.scale_rewards_by_dt`，实测签名 `RewardManager.__init__(self, cfg, env, *, scale_by_dt=True)`）。
- **episodic-sum 日志机制**（per-term 奖励曲线之源）：`reset(env_ids)` 时 `Episode_Reward/<term> = mean(episode_sum[env_ids]) / max_episode_length_s`，写进 `extras` → env 并入 `extras["log"]` → TensorBoard；随后把这些 env 的累加器清零。注意是**每秒平均率**（除以 `max_episode_length_s`）。

**Termination**（两框架同语义同属性名）：
- `TerminationTermCfg.time_out: bool` 决定该项 OR 进 `_truncated_buf`（截断/超时）还是 `_terminated_buf`（真终止：成功/失败/摔倒）。
- 属性：`dones = truncated | terminated`、`time_outs = truncated`、`terminated = terminated`。**净 done = 所有 term 逻辑或**，按 `time_out` 旗分两桶。RL 库据此区分 bootstrap（真终止不 bootstrap、截断 bootstrap）。
- 日志：`Episode_Termination/<term> = 因该项而结束的 env 比例`。

### 1.6 Action Manager 的两阶段设计（decimation 的使能器，承重教学点）

两框架都把动作处理拆成 `process_action` / `apply_action` 两阶段：
- `process_action(action)`——**每环境步一次**：校验 `total_action_dim == action.shape[1]`；移动历史 `prev ← current ← new`；按 `idx` 游标把扁平 `(num_envs, total_action_dim)` 切成各 term 连续块（宽 `term.action_dim`，按插入序），调 `term.process_actions(slice)`（**昂贵的策略率工作**：IK、缩放/偏置、算目标关节位置）。
- `apply_action()`——**每物理子步一次**：循环各 term 调 `apply_actions()`（**廉价的物理率工作**：把已算好的目标写进 PhysX/MuJoCo，如 `set_joint_position_target`）。

**WHY**：RL 用 decimation（1 策略步 = N 物理子步）。把昂贵解释只做 1 次、把"写目标"做 N 次，让 PD 控制器在高物理率动作、策略在低率决策——这正是 decimation 能工作、且不会把 IK 重算 N 次的原因。env 循环即：`process_action(a)` 一次 → 循环 N×：`apply_action()` + `sim.step()`。

**mjlab 的关键差异——按"执行器"而非"关节"选择**（见问题记录 B.1）：
- Isaac Lab `JointPositionActionCfg(asset_name="robot", joint_names=[...], scale, use_default_offset)`。
- mjlab `BaseActionCfg(actuator_names=(...), scale, offset, preserve_order)`——**用 `actuator_names`**，内部 `entity.find_joints_by_actuator_names()`（`entity/entity.py`）把执行器名解析到它们驱动的关节索引。即 mjlab 的动作空间由**执行器**定义。另有 `JointEffort/RelativeJointPosition/TendonLength|Velocity|Effort/SiteEffort` 等动作族（腱驱/位点力，灵巧手用）。`JointPositionAction.apply_actions` 还会先减 `encoder_bias` 再写目标。`use_default_offset=True` 仅 Position/RelativePosition/Velocity 有。

### 1.7 Event Manager 的模式与 mjlab 的 DR/CUDA-graph 皇冠明珠

**模式（两框架）**：`startup`（仿真起一次）、`reset`（每次复位）、`interval`（定时，manager 自己计时分发）。Isaac Lab 另有 `prestartup`（USD 级随机化，仿真前）；mjlab 另有 `step`（每步、所有 env 无条件）。`interval` 逐 env 独立计时器（或 `is_global_time` 共享），`time_left -= dt`，`< 1e-6` 时重采样并对触发 env 调 `func`。

**mjlab 独有的 DR → CUDA-graph 流水线**（Isaac Lab 无对应；Isaac Lab DR 直接写 PhysX view）：

1. **声明**：DR 函数上挂装饰器 `@requires_model_fields("body_mass", recompute=RecomputeLevel.set_const)`（`event_manager.py:73`），给函数挂 `func.model_fields`（请求字段 + 该 recompute 等级的派生字段）与 `func.recompute`。
2. **重算等级** `RecomputeLevel`（`IntEnum`）：`none/set_const_fixed/set_const_0/set_const`，分别决定 DR 改完后要重算哪些 MuJoCo 派生常量（如 `body_subtreemass`、`dof_invweight0`…）。`IntEnum` 故可 `max(...)` 复合。
3. **收集**：`EventManager._prepare_terms` 把所有带 `model_fields` 的 term 的字段去重收进 `_domain_randomization_fields`，经 `domain_randomization_fields` 属性暴露。
4. **预分配**：env `load_managers` 里 `sim.expand_model_fields(self.event_manager.domain_randomization_fields)`（`manager_based_rl_env.py:309`）。`expand_model_fields`（`sim/sim.py`）对 leading-dim 为 1 的"全 world 共享"字段 `tile()` 成 `(num_envs, ...)` per-world 副本（`randomization.py` 的 `repeat_array_kernel`，每 world 一行），让每个 world 有自己可随机化的拷贝；`nworld==1` 时直接返回不扩。
5. **CUDA-graph 重捕**：`expand_model_fields` 末尾**必须** `create_graph()`。因为捕获的 CUDA graph 锁死了捕获时的 GPU 数组**地址**，而 `expand_model_fields` 用 `setattr` 换了数组——不重捕则 graph 会静默读旧数组、忽略新 per-world 值。
6. **运行期重算**：`EventManager.apply(mode,...)` 里跟踪本次触发 term 的 `strongest_fired = max(recompute)`，循环后若非 `none` 则**一次性** `sim.recompute_constants(strongest_fired)`（内部 `getattr(mjwarp, level.name)(...)`）。"多个 DR term 触发、最后按最强等级只重算一次派生常量"是 mjlab 的效率设计。

**4 张 CUDA graph**（`sim/sim.py:create_graph`）：`step_graph / forward_graph / reset_graph / sense_graph`，仅当 `_should_use_cuda_graph()` 为真才捕获——门控是 `wp.is_mempool_enabled(device)` ∧ CUDA 设备 ∧ 驱动 ≥ 12.4，否则回退到 eager `mjwarp.*` 直调。捕获时 `_suspend_gc()` 关 GC，防陈旧 warp Graph 的析构被录进新 graph 而损坏重放。

### 1.8 mjlab 独有：MetricsManager（Isaac Lab 无）

**用途**：累加逐步 metric 值并报告**真·逐步回合平均**——与 reward 不同，metric **无 weight、不 ×dt、不除 max_episode_length**，所以 `[0,1]` 的 metric 在日志里仍是 `[0,1]`（这是与 RewardManager 的明确设计对比）。
- `MetricsTermCfg(per_substep:bool=False, reduce:Literal["mean","last"])`。`per_substep` term 在 decimation 循环内 `compute_substep()` 累加（注意子步内只有 `qpos/qvel/act` 是新的，派生量陈旧）；`reduce="last"` 给"成功标志"等不该时间平均的二值量。
- `reset` 写 `Episode_Metrics/<term>`。

### 1.9 mjlab 独有：Null-manager 零开销模式

对 4 个**可选** manager（command/curriculum/metrics/recorder），cfg dict 为空时 env 用 `Null*` 占位（`NullCommandManager/NullCurriculumManager/NullMetricsManager/NullRecorderManager`）：
- `Null*` **不是 `ManagerBase` 子类**（零开销，源注释明说），不建缓冲、`cfg=None`、`active_terms=[]`。
- **鸭子类型实现全部公开接口为 no-op**（返回空/None），含 viser GUI 钩子。
- 结果：env 热循环里 `metrics_manager.compute_substep()`（每子步）、`command_manager.compute(dt)`（每步）**无条件调用、无 `if is not None` 分支**——非激活时是平凡 Python no-op。用极小常数调用成本换"无分支热循环 + 统一接口"。
- **无** `NullEventManager/NullObservation.../NullAction.../NullReward.../NullTermination.`——这五个永远是真的（每个 RL env 都需要）。

---

## 第 2 章 包/项目组织与配置系统（→ Ch01 生态、Ch02 搭建）

### 2.1 Isaac Lab 多包架构（"内核 vs 任务 vs 资产 vs 训练胶水"）

`source/` 下是**独立可安装的扩展包**（各有 `pyproject.toml`/`setup.py`/`config/extension.toml`）：

| 包 | 职责（FOR 什么） | 关键内容 |
|---|---|---|
| **`isaaclab`**（内核） | 与任务无关的仿真抽象层 | 子包：`app/ sim/ scene/ assets/ actuators/ sensors/ controllers/ terrains/ markers/ managers/ envs/ devices/ ui/ utils`。`envs/` 含 `ManagerBasedRLEnv`、`ManagerBasedEnv`、`DirectRLEnv`（两套工作流，见 2.4）。`managers/` 是第 1 章主角。 |
| **`isaaclab_tasks`**（任务） | 具体 env 定义 + gym 注册 | `manager_based/{locomotion,manipulation,navigation,classic,...}` 与 `direct/{...}`；每任务有 `<task>_env_cfg.py` + `config/<robot>/__init__.py`(注册) + `agents/`(各 RL 库超参)。`utils/`：`import_packages`、`parse_cfg`、`hydra`。 |
| **`isaaclab_assets`**（资产） | 预定义机器人/传感器 cfg | `robots/`（unitree/anymal/franka/spot/g1/h1/…）、`sensors/`。是一堆 `ArticulationCfg` 常量，被任务 cfg `import` 复用。 |
| **`isaaclab_rl`**（训练胶水） | 把 env 适配到各 RL 库 | `rsl_rl/ rl_games/ skrl/ sb3.py`；核心是 `RslRlVecEnvWrapper`（见 2.5）+ 各库的 `*RunnerCfg`。 |
| `isaaclab_mimic` / `isaaclab_contrib` | 模仿学习 / 社区贡献 | 旁支，本部不展开。 |

**依赖方向**：`isaaclab_tasks` → 依赖 `isaaclab`(内核) + `isaaclab_assets`(资产)；`isaaclab_rl` → 依赖 `isaaclab`(env 类型) + 外部 RL 库；训练脚本 → 同时用 `isaaclab_tasks`(注册/cfg) + `isaaclab_rl`(wrapper)。建议画**包依赖树**。

### 2.2 mjlab 单包架构（扁平模块）

mjlab 是**单一 `mjlab` 包**，按职责分模块（非多扩展包）：

```text
mjlab/
  sim/        Simulation + SimulationCfg + randomization（mujoco_warp 封装、CUDA-graph、expand_model_fields）
  scene/      Scene + SceneCfg（持有 entities + sensors）
  entity/     Entity + EntityCfg + variants（资产抽象 = mujoco_warp 实体；含 find_joints_by_actuator_names）
  actuator/   执行器模型（BuiltinActuatorGroup / XmlActuator / 传动类型）
  managers/   第 1 章主角（含 MetricsManager + Null*；唯一 term 基类 ManagerTermBase）
  envs/       ManagerBasedRlEnv + ManagerBasedRlEnvCfg + mdp/（通用 obs/reward/event/dr 函数库）
  tasks/      cartpole / manipulation / tracking / velocity（每任务 config/<robot> + registry 注册）
  rl/         RslRl*Cfg + RslRlVecEnvWrapper + MjlabOnPolicyRunner
  sensor/ terrains/ viewer/ utils/ asset_zoo/ scripts/
```

**对比**：Isaac Lab = 多扩展包（内核/任务/资产/训练分离，可独立发版）；mjlab = 单包扁平模块（更轻、依赖更少）。Isaac Lab 的"资产"是独立包 `isaaclab_assets`，mjlab 的资产抽象是 `entity/` + `asset_zoo/`。
> 【UniLab 对比钩子】UniLab 是 `src/unilab/{base,envs,dr,terrains,...}` 单包 + Hydra `conf/` YAML，env 基类 `ABEnv/NpEnv`，由 UniLab agent 给出，此处留并表行。

### 2.3 配置系统：`@configclass`（Isaac Lab） vs 原生 `dataclass`（mjlab）

**Isaac Lab `@configclass`**（`isaaclab/utils/configclass.py`）：自研装饰器，包 `dataclass` 并：
- 自动给无注解的类成员补类型注解（`_add_annotation_types`）；
- 自动把可变默认转成 `field(default_factory=...)`（`_process_mutable_types`），用户可直接写 `eye: list = [7.5,7.5,7.5]`；
- 注入 `to_dict / from_dict / replace / copy / validate` 方法。

配置写成**嵌套 `@configclass` 类**，term 是**类属性**（声明顺序 = obs 拼接/动作切片顺序，靠类体属性顺序保证）。看 `velocity_env_cfg.py` 实例：
```python
@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(-0.1,0.1))   # 顺序承重
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(-0.2,0.2))
        ...
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
    policy: PolicyCfg = PolicyCfg()
```
顶层 env cfg 把 `MySceneCfg / CommandsCfg / ActionsCfg / ObservationsCfg / EventCfg / RewardsCfg / TerminationsCfg / CurriculumCfg` 组合进 `ManagerBasedRLEnvCfg` 的字段。机器人专属 cfg（如 `G1RoughEnvCfg`）继承基类、`__post_init__` 里改字段（换 robot、调权重）。

**mjlab 原生 `@dataclass(kw_only=True)`**（无自研装饰器）：配置写成**工厂函数返回 dataclass 实例**，manager 配置是**普通 `dict[str, TermCfg]`**。看 `velocity_env_cfg.py`（`make_velocity_env_cfg()`）：
```python
actor_terms = {
    "base_lin_vel": ObservationTermCfg(func=mdp.builtin_sensor, params={"sensor_name":"robot/imu_lin_vel"}, noise=Unoise(-0.5,0.5)),
    "joint_pos":    ObservationTermCfg(func=mdp.joint_pos_rel, noise=Unoise(-0.01,0.01)),
    ...
}
critic_terms = {**actor_terms, "foot_height": ObservationTermCfg(...), ...}   # 特权 obs 用 dict 解包扩展
observations = {
    "actor":  ObservationGroupCfg(terms=actor_terms,  concatenate_terms=True),
    "critic": ObservationGroupCfg(terms=critic_terms, concatenate_terms=True),
}
```
顺序靠 dict 插入序。机器人专属用 `dataclasses.replace(...)` 或工厂参数（如 `unitree_g1_rough_env_cfg(play=True)`）定制。

**核心对比表**：

| 维度 | Isaac Lab | mjlab |
|---|---|---|
| 配置装饰器 | 自研 `@configclass`（补注解+可变默认+to_dict/validate） | 原生 `@dataclass(kw_only=True)` |
| 配置组织 | 嵌套 `@configclass` 类 | 工厂函数返回 dataclass 实例 |
| manager 项容器 | 类属性 | 普通 `dict[str,TermCfg]` |
| 顺序保证 | 类体属性顺序 | dict 插入顺序 |
| 特权/非对称 obs | 多定义一个 `CriticCfg` group | `critic_terms = {**actor_terms, ...}` 解包 |
| 定制方式 | 继承 + `__post_init__` 改字段 | `replace()` / 工厂参数 |

> 【UniLab 对比钩子】UniLab 是 Hydra dataclass + `conf/<algo>/task/<task>/<backend>.yaml`，`reward.scales{...}` 是标量字典权重——第三种配置范式，由 UniLab agent 给出。

### 2.4 两套工作流：Manager-based vs Direct（Isaac Lab 特有的二分）

Isaac Lab `envs/` 同时有 `manager_based_rl_env.py`（声明式：拼 manager + term cfg）与 `direct_rl_env.py`（命令式：把 obs/reward/reset 写进一个 env 类的方法里）。**本部聚焦 manager-based**（这是"Manager 架构"章的主题）。mjlab **只有 manager-based 一套**（无 Direct 对应）。教学上可点出："Isaac Lab 给两种风格，mjlab 押注 manager-based"。

### 2.5 任务注册：字符串任务名 → 跑起来的 env+cfg（两框架机制根本不同）

**Isaac Lab —— 裸 gym registry + 字符串 entry_point**：
- 注册（`config/g1/__init__.py`）：
  ```python
  gym.register(
      id="Isaac-Velocity-Rough-G1-v0",
      entry_point="isaaclab.envs:ManagerBasedRLEnv",
      disable_env_checker=True,
      kwargs={
          "env_cfg_entry_point": f"{__name__}.rough_env_cfg:G1RoughEnvCfg",   # 字符串路径！
          "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:G1RoughPPORunnerCfg",
          "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
      },
  )
  ```
  - 同一任务常注册 4 个 id：`-v0`（训练）+ `-Play-v0`（播放/评估），rough/flat 各一对。
  - cfg 以**字符串路径**存 kwargs，**多 RL 库各一条**（`rsl_rl_cfg_entry_point`/`skrl_cfg_entry_point`/...）。
- 任务发现：`isaaclab_tasks/__init__.py` 调 `import_packages(__name__, _BLACKLIST_PKGS)`（`utils/importer.py`）**递归 walk 整个包**触发各 `__init__.py` 里的 `gym.register`。
- 字符串 → 类解析：训练时 `load_cfg_from_registry(task, "env_cfg_entry_point")`（`utils/parse_cfg.py`）：取 `gym.spec(task).kwargs[key]`；若是 `.yaml` 路径→`yaml.full_load`；若是 `"mod:Attr"` 字符串→`importlib.import_module(mod)` + `getattr(mod,Attr)` 得类，再**实例化** `cfg_cls()`。
- 解析链总览：**任务字符串 → gym.spec → kwargs 里的 cfg 路径字符串 → importlib+getattr → cfg 类 → 实例化 → 传给 `ManagerBasedRLEnv(cfg)`**。

**mjlab —— 自定义 registry + 已实例化 cfg 对象**：
- 注册（`config/g1/__init__.py`）用 `register_mjlab_task`（`tasks/registry.py`）：
  ```python
  register_mjlab_task(
      task_id="Mjlab-Velocity-Rough-Unitree-G1",
      env_cfg=unitree_g1_rough_env_cfg(),          # 已实例化的 cfg 对象！
      play_env_cfg=unitree_g1_rough_env_cfg(play=True),
      rl_cfg=unitree_g1_ppo_runner_cfg(),
      runner_cls=VelocityOnPolicyRunner,
  )
  ```
  - 存**已实例化 cfg 对象**到私有 dict `_REGISTRY`；train/play 用 `load_env_cfg(task, play=...)` 返回 `deepcopy`（防注册态被改）。
- 任务发现：靠 **Python entry_points（group=`mjlab.tasks`）**——`mjlab/__init__.py` 的 `_import_registered_packages()` 用 `importlib.metadata.entry_points().select(group="mjlab.tasks")` 逐个 `entry_point.load()`，触发各包注册。**不是** gym registry，故 `list-envs` 读的是 `_REGISTRY` 不是 `gym.envs.registry`。
- CLI 用 `tyro`（`mjlab/__init__.py` 配 `TYRO_FLAGS`：禁联合类型切换、关 `--no-flag` 自动转换故布尔须 `--flag False`、集合用 Python 语法）。

**对比表**：

| 维度 | Isaac Lab | mjlab |
|---|---|---|
| 注册 API | 裸 `gym.register` | 自定义 `register_mjlab_task` |
| cfg 存储 | 字符串路径（kwargs） | 已实例化对象（`_REGISTRY`） |
| 解析 | `importlib`+`getattr`+实例化 | `deepcopy(_REGISTRY[id])` |
| 任务发现 | `import_packages` 递归 walk | entry_points `mjlab.tasks` + load |
| 注册表 | `gym.envs.registry` | 私有 `_REGISTRY` |
| 多 RL 库 | kwargs 多条 entry_point | 单 `rl_cfg` + `runner_cls` |
| CLI | argparse/hydra | tyro |

### 2.6 RL 库胶水：两个 `RslRlVecEnvWrapper`（趋同设计，好教学点）

两框架的 rsl_rl 适配器**几乎一模一样**（都继承 rsl_rl 的 `VecEnv`、都返回 `TensorDict`）：
- ctor 里 `self.env.reset()`（因 rsl_rl runner **不**自己 reset）；`num_actions = action_manager.total_action_dim`；`clip_actions` 改写 action space。
- `step(actions)`：可选 `torch.clamp(±clip_actions)` → `env.step` → `dones = (terminated|truncated).long()` → **若非 finite-horizon 把 `time_outs` 搬进 `extras`**（无限 horizon 任务靠它做 value bootstrap）→ 返回 `(TensorDict(obs), rew, dones, extras)`。
- Isaac Lab 版多支持 `DirectRLEnv`（`isinstance` 检查二选一）；mjlab 版只认 `ManagerBasedRlEnv`。

**教学价值**：两个独立项目在 RL 胶水层收敛到几乎相同的接口——说明"`VecEnv` 抽象 + TensorDict + time_outs 入 extras"已是 GPU 并行 RL env 的事实标准。可并排展示两段 `step` 源码佐证。

mjlab `RslRlPpoAlgorithmCfg` 默认值（实测）：`num_learning_epochs=5, num_mini_batches=4, learning_rate=1e-3, gamma=0.99, lam=0.95, entropy_coef=0.005, desired_kl=0.01, clip_param=0.2`；`RslRlModelCfg` actor/critic 分开（`hidden_dims, activation, obs_normalization, cnn_cfg/rnn_type/distribution_cfg`）。Isaac Lab `RslRlOnPolicyRunnerCfg` 字段含 `num_steps_per_env, max_iterations, obs_groups, clip_actions, experiment_name` 等（`actor_obs_normalization/critic_obs_normalization` 取代旧 `empirical_normalization`）。

### 2.7 CPU↔GPU 边界（数据流图关键，两框架可见性不同）

- **mjlab：边界显式可读**（`sim/sim.py`）。三层：
  1. `mujoco.MjModel/MjData`（host/CPU/NumPy，C 对象，单模板）；
  2. `mjwarp.put_model(mj_model)` + `mjwarp.put_data(..., nworld=num_envs)` 抬到 warp 数组（GPU，`wp_device`），**按 nworld 批化**（SoA）；
  3. `WarpBridge` 把 warp 数组**零拷贝**暴露成 torch 张量（`TorchArray`），manager term 函数即读写这些张量。
  - 热循环全在 GPU（`step/forward/reset/sense` 是 warp kernel，理想下各 1 次 `wp.capture_launch`）；CPU 越界只在日志/检视（`.cpu().tolist()/.item()`，每步或每 reset 一次）。
  - 变体（per-world mesh）场景：`_mj_model` 仍是单 host 模板，变体相关字段已 per-world 在 wp_model 里；CPU 消费者渲染某 env 前须先同步该 env 的 per-world 字段（显式边界告诫）。
- **Isaac Lab：边界藏在 Isaac Sim 内**。`sim.reset()` 那一下"activate physics handles that expose TensorAPIs"（`manager_based_env.py` 注释）——之后 PhysX 状态经 Fabric/TensorAPI 暴露成 torch 张量，用户读不到 `put_model` 那层。教学上 mjlab 适合讲清 "NumPy→warp→torch" 全链，Isaac Lab 只能讲 "reset 激活 TensorAPI 后一切皆 `(num_envs,·)` GPU 张量"。

---

## 第 3 章 建议的教学插图（4 张）

1. **env.step 时序图（并排 Isaac Lab vs mjlab）**：纵轴时间，泳道 = `Policy(env-rate)` / `process_action` / `decimation 循环(apply_action→write→sim.step→update)` / `termination` / `reward` / `reset` / `command` / `event(interval)` / `observation`。**重点标注**：① 奖惩在 reset 前、obs 在 reset 后；② mjlab 的"单次 `sim.forward()`"位置 + 独立 `sim.sense()`；③ mjlab 子步内 `metrics.compute_substep()`。
2. **Manager 编排图**：env 中心，8/9 个 manager 环绕；箭头标 `load_managers` **构建顺序**（Isaac: command→event→recorder→action→obs→termination→reward→curriculum；mjlab: event→command→action→obs→termination→reward→curriculum→metrics→recorder）+ 每步**调用顺序**。标 mjlab 独有 MetricsManager 与 Null* 占位。
3. **包依赖树**：Isaac Lab 四包（`isaaclab` ← `isaaclab_tasks`/`isaaclab_rl`；`isaaclab_assets` ← `isaaclab_tasks`；外部 RL 库 ← `isaaclab_rl`）；mjlab 单包内模块依赖（`sim`←`scene`/`envs`；`managers`←`envs`；`tasks`→`envs`/`managers`/`entity`；`rl`→`envs`）。
4. **CPU↔GPU 数据流图**：mjlab 三层（MjModel/MjData[CPU NumPy] →`put_model/put_data nworld`→ wp arrays[GPU SoA] →`WarpBridge`→ torch[zero-copy]），叠加 4 张 CUDA graph（step/forward/reset/sense）与 DR 的 `expand_model_fields`(tile per-world) + `create_graph` 重捕；Isaac Lab 对照画"reset 激活 TensorAPI"那道隐藏边界。
5.（可选）**任务字符串解析流水线**：`"Isaac-Velocity-Rough-G1-v0"` → `gym.spec.kwargs["env_cfg_entry_point"]="...:G1RoughEnvCfg"` → `importlib+getattr` → `cfg=G1RoughEnvCfg()` → `ManagerBasedRLEnv(cfg)`；并排 mjlab `"Mjlab-Velocity-Rough-Unitree-G1"` → `_REGISTRY[id].env_cfg` → `deepcopy` → `ManagerBasedRlEnv(cfg)`。

---

## 第 4 章 每章映射（这些素材该并入哪一章）

### → Ch01 生态（prac 生态总览）
- **2.1/2.2 包架构对比**（Isaac Lab 多扩展包 vs mjlab 单包）：讲清"内核/任务/资产/训练胶水"分工——生态结构是认识框架的第一步。
- **2.4 两套工作流**（manager-based vs direct；mjlab 只 manager-based）：定位本部为何聚焦 manager-based。
- **2.6 RL 库胶水趋同**（两个 `RslRlVecEnvWrapper` 几乎相同）：说明 GPU 并行 RL env 的事实标准接口；引出"为何都接 rsl-rl"。
- 框架定位一句话：Isaac Lab = Omniverse/PhysX、重、生态全、双工作流；mjlab = mujoco_warp、轻、单包、押注 manager-based + CUDA-graph。

### → Ch02 搭建（prac 搭建 + 从零起项目）
- **2.5 任务注册机制对比**（gym registry+字符串 entry_point+import_packages vs register_mjlab_task+entry_points+_REGISTRY）：用户"为什么 train 脚本给个任务名就能跑"的答案——搭建/起新任务必讲。
- **2.3 配置系统对比**（@configclass vs 原生 dataclass+dict；嵌套类 vs 工厂函数）：起项目就是写 cfg，必须讲清两种范式 + 顺序承重 + 定制方式（继承+`__post_init__` vs `replace`/工厂参数）。
- **2.7 CPU↔GPU 边界** 的"何处装东西"在搭建时也相关（device 选择、num_envs 在 scene cfg）。
- 配 **插图 3（包依赖树）+ 插图 5（任务字符串解析）**。

### → Ch04 Manager 架构（prac Manager 架构，本部核心章）
- **第 0 章 两个 env.step 端到端** + **插图 1（时序图）**：章节主轴。
- **1.1–1.3 Manager 构建/调用编排 + Term 两家族 + SoA 数据布局**：架构骨架。
- **1.4 Observation**（流水线顺序、拼接、actor/critic obs_groups、mjlab delay/NaN/缓存）。
- **1.5 Reward/Termination**（加权和 + episodic 日志机制 + terminated/time_out 拆分 + dt 缩放的 WHY）。
- **1.6 Action 两阶段设计**（decimation 使能器；Isaac joint vs mjlab actuator 解析）——最承重的单点教学。
- **1.7 Event + mjlab DR/CUDA-graph 皇冠明珠**（`@requires_model_fields → expand_model_fields → create_graph 重捕 → recompute_constants`）。
- **1.8 MetricsManager**（mjlab 独有）、**1.9 Null-manager 零开销模式**（mjlab 独有）。
- **2.7 CPU↔GPU 边界** + **插图 2（编排图）+ 插图 4（数据流图）**。

> 注：本文是**参考材料**，待并入对应 `prac_*.tex`；不直接编辑 `.tex`（迁移 agent 在写，避免竞争）。源码-文档不一致已回写 `RL运控融合_问题与解决记录.md`（B.3 / C.2 节）。

---

## 附：核心 `file:line` 锚点速查

- Isaac Lab env.step：`isaaclab/envs/manager_based_rl_env.py`（`step` / `_reset_idx` / `load_managers`）；基类 `manager_based_env.py`（`__init__` 里 `sim.reset()` 激活 TensorAPI 注释、`load_managers`）。
- Isaac Lab managers：`managers/manager_base.py`（`ManagerTermBase:31`、`ManagerBase:131`、`_resolve_common_term_cfg:301`、`_process_term_cfg_at_play:381`）；`manager_term_cfg.py`（各 TermCfg）；`observation_manager.py:compute_group ~393`；`reward_manager.py:compute ~129 / reset ~101`；`termination_manager.py:compute ~154`；`action_manager.py:process_action:372 / apply_action:395 / total_action_dim:248`；`event_manager.py:apply ~153`。
- Isaac Lab 组织：`utils/configclass.py:configclass`；`isaaclab_tasks/__init__.py`(import_packages)、`utils/importer.py`、`utils/parse_cfg.py:load_cfg_from_registry:20`；注册示例 `manager_based/locomotion/velocity/config/g1/__init__.py`；cfg 组合 `velocity/velocity_env_cfg.py`；RL 胶水 `isaaclab_rl/rsl_rl/vecenv_wrapper.py:step:151`、`rl_cfg.py:obs_groups:159`。
- mjlab env.step：`envs/manager_based_rl_env.py`（`step`、`load_managers:300+`、`expand_model_fields 调用:309`、`_reset_idx`、`ManagerBasedRlEnvCfg`）。
- mjlab managers：`managers/manager_base.py`（`ManagerTermBase:64`、`_resolve_common_term_cfg:137`）；`observation_manager.py`（流水线 docstring、`compute_group`、`__init__:130`）；`reward_manager.py:compute:117 / reset:103 / __init__:53`；`termination_manager.py:compute:108`；`action_manager.py:process_action:165 / apply_action:191 / total_action_dim:106`；`event_manager.py`（`requires_model_fields:73`、`RecomputeLevel:25`、`domain_randomization_fields:213`、`apply:226`）；`metrics_manager.py`（`compute_substep:130 / compute:143`）；各 `Null*` 在对应 manager 文件尾。
- mjlab 组织：`__init__.py`（`_import_registered_packages` entry_points、`TYRO_FLAGS`）；`tasks/registry.py:register_mjlab_task`；`tasks/velocity/velocity_env_cfg.py:make_velocity_env_cfg`；`tasks/velocity/config/g1/__init__.py`(注册)；`rl/vecenv_wrapper.py:step`、`rl/config.py`(RslRl*Cfg)。
- mjlab sim：`sim/sim.py`（`Simulation:161`、`_init_with_model/_finish_init`、`create_graph:218`、`_should_use_cuda_graph:577`、`expand_model_fields:386`、`recompute_constants:425`、`step/forward/reset/sense`）；`sim/randomization.py`（`expand_model_fields` + `repeat_array_kernel`）；`scene/scene.py`（`write_data_to_sim:202`、`update:196`）；`entity/entity.py`（`find_joints_by_actuator_names:525`）。
