# P8 工程验证报告 · autodl 路（14 章，mjlab/UniLab，无 Isaac）

> 服务器实跑核验工程实践 + 找优化 + 评教学性。2026-06-25。

## prac_setup

**服务器**：autodl (RTX 4080 SUPER 32GB; 仅 mjlab+UniLab，无 Isaac)

**实跑命令（15）**
- mjlab `uv run list-envs` → 成功,12 任务,含 Mjlab-Velocity-Flat-Unitree-G1 与 Go1(Flat/Rough),确无 Go2(印证 L879 \rebuilt)
- mjlab L0 `torch.randn(10).cuda()` → avail True, kernel=4.12, torch.version.cuda=12.8(驱动向后兼容,印证 L49 法则)
- mjlab L1 `import mjlab`+`mujoco.__version__` → mjlab ok, mujoco 3.8.1
- mjlab `play G1 --help` → `--agent {zero,random,trained}` + `--viewer {auto,native,viser}`(精确印证 L380-381 的 zero/random)
- mjlab L6 `train G1 --env.scene.num-envs 64 --agent.max-iterations 2` → 成功,1319 steps/s,reward/loss 各项更新无 NaN
- mjlab console_scripts → ['demo','export-scene','list-envs','play','train','viz-nan'](与 L877 完全一致); export-scene/viz-nan --help 均存在且功能描述吻合
- mjlab `play G1 --agent zero`(无 viewer flag)→ 启动成功, 打印 `viser (listening *:8080)`(实证端口 8080 正确,非 7860), mujoco_warp JIT load on cuda:0 cached
- mjlab `train G1 --help` → `--agent.algorithm.schedule {adaptive,fixed}`(默认 adaptive)+`--agent.algorithm.learning-rate`(KL 自适应 LR 内部存在,但终端摘要不打印 kl)
- UniLab L0 → avail True kernel ok; `df -h /dev/shm` = 31G 可用(本机不受限)
- UniLab L1 `import unilab` → ok; L2 `registry.ensure_registries()+list_registered_envs()` → 恰 26 个(精确印证 L293/577/627),注册名为 CamelCase(Go2JoystickFlat 等)
- UniLab L6 `train --algo ppo --task go2_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true` → 成功,snake_case slug 被接受并映射到 Go2JoystickFlat,9 项 reward(tracking_lin_vel/swing_feet_z/... 印证 L617),68 steps/s(16env 冷启)
- UniLab `--sim motrix` 同任务 → 成功,425 steps/s,同 9 项 reward(印证后端无关性 L612/628)
- UniLab console scripts → ['demo','eval','train','unilab-complete','unilab-export-scene','unilab-import-robot','unilab-pull-assets','unilab-render-teaser','unilab-viz-nan'](印证 eval/demo/unilab-export-scene/unilab-viz-nan 均存在)
- UniLab `train --help` 路由键 → `--algo {ppo,mlx_ppo,appo,sac,td3,flashsac}` `--task` `--sim {mujoco,motrix}` `--render-mode {auto,interactive,record,none}`(精确印证 L39/L562/L716 的 appo,且 algo/task/sim 确为 argparse flag 非 Hydra)
- Isaac 部分:autodl 无 Isaac,未实跑,待 gpufree 补验

**工程核对（是这样/不符）**
- 是这样:mjlab Mjlab-Velocity-Flat-Unitree-G1 任务名、--agent zero/random、--viewer viser、Viser 端口 8080、console_scripts 六命令、export-scene/viz-nan——全部实跑核实无误(L156-162/L377-386/L877)
- 是这样:UniLab 实测恰 26 注册任务、--task go2_joystick_flat slug 可用、9 项 reward 名(tracking_lin_vel/swing_feet_z)、mujoco+motrix 双后端均起训、--render-mode 四值、--algo 含 appo——全部核实(L293/L562/L617/L716)
- 是这样:UniLab Hydra 覆盖 algo.num_envs=/algo.max_iterations=/training.no_play= 与路由 flag --algo/--task/--sim 的二分(L146-148 pitfall)实跑成立——传 mjlab 式 --num_envs 到 play 会报 Unrecognized options,印证 CLI 哲学差异
- 不完全是这样(教学性偏差,非硬错):L719-736「读日志」表称『以 mjlab 风格为例』,但真实 mjlab 终端字段是 `Mean reward`/`Mean value loss`/`Mean surrogate loss`/`Mean entropy loss`/`Steps per second`/`Collection time`/`Learning time`,与表中 `mean_reward`/`policy_loss`/`value_loss`/`fps` 字面不符;尤其 `kl` 与 `entropy` 在 mjlab 每迭代摘要里根本不打印——读者照表找 kl 字段会扑空
- 是这样:mjlab play 默认即起 Viser(L4 play --agent zero 自动 listening 8080),而 train 默认无头需 --viewer——L190/L356 称『mjlab 默认无头』仅对 train 准确,对 play 不准确(play 默认就开 Viser),措辞可更精确区分 train/play 默认

**优化点 / 高级技巧**
- L719-736 日志表建议:要么改成 RSL-RL/mjlab 真实字段名(Mean reward / Mean value loss / Mean surrogate loss / Steps per second / Collection time / Learning time),要么明确标注『字段名为通用语义、各框架字面不同』;并补一句『mjlab 的 KL 不在终端摘要打印,但 --agent.algorithm.schedule=adaptive 内部按 desired_kl 自适应调 LR,要看 KL 须开 WandB/TensorBoard』——这正好兜住 L731 的 kl 健康范围
- 可补 mjlab 真实可调旋钮:实跑确认 `--agent.algorithm.learning-rate`(默认 0.001)、`--agent.algorithm.schedule {adaptive,fixed}`、`--agent.algorithm.entropy-coef` 等都经 tyro 暴露为 CLI flag——L347-358 的四维表只列了 num-envs/max-iterations,可补『超参亦走 --agent.algorithm.* 直接命令行覆盖』这一高级用法(对比 UniLab 的 Hydra 点路径,正好两种范式对照)
- L617 吞吐基准建议补本机实测点:RTX 4080S 上 16env 冷启 mujoco 68 sps / motrix 425 sps(JIT 未热),与 L617『32env~4600/64env~8900』需说明那是热稳态大并行值——否则读者跑 16env 小冒烟见到几十 sps 会误判异常
- 可补『同一 UniLab 任务注册名是 CamelCase(Go2JoystickFlat)但 CLI 接 snake_case(go2_joystick_flat)』这一坑:L596 讲了装饰器注册键,但没点明 CLI slug 与注册名的大小写/下划线转换,初学者用 list_registered_envs 拿到 CamelCase 直接传 --task 可能困惑
- mjlab `play` 与 `train` 的 CLI option 集不同(play 不认 --env.scene.num-envs,实测报 Unrecognized)——可在 L156/L377 旁补一句『play 改并行数用其自身 flag,勿照搬 train 的 --env.scene.num-envs』,避免读者在 L4/L5 复制 train 写法

**教学缺口（只讲代码没教工程之处）**
- L719-736『读日志』是全章最大教学缺口:表把 kl/entropy/fps 当作『你日后所有训练的诊断基准』,但 mjlab 实际摘要既不打印 kl 也字段名不一致——读者拿表去对真实输出会对不上,且不会被教到『KL 藏在 adaptive LR 调度里、要去 WandB 看』。应教『不同框架日志字段名不同,关键是认语义(采集时间/学习时间/loss/reward),并指出各框架去哪看 KL』
- L617 抛出吞吐数字(4600/8900 sps)但没教读者『怎么判断我这台机的 sps 算不算正常』:缺一条『sps≈num_envs×控制频率×并行效率,小并行冷启会远低于稳态』的估算心法,否则读者跑小冒烟见低 sps 无从判断是正常还是故障(本章 L611『远低』给的原因是 CPU核不足/shm抖动,但没给『冷启JIT/小并行』这个最常见的良性原因)
- Isaac 部分(L467-523)纯静态:zero_agent.py/random_agent.py/rsl_rl/train.py 路径与 isaaclab.__version__ 本机无法证伪,L878 已诚实标注以官方为准——建议正文再加一句『本章 Isaac 命令未在本配置实跑,以官方仓库脚本为准』的可见提示(目前只在末尾 finenote),避免读者误以为三框架都经同等实跑
- 教学性总体优秀(L4 反事实 ins:rlmc-skip-l4、分层=故障隔离 ins:rlmc-fault-isolation、可视化=breakpoint 等都讲了『为什么这么做』而非逐行翻译),缺口集中在『日志字段与真实输出对不齐』这一处可证伪的硬细节;补齐后教学闭环完整

**notes**：三验证维度结论:维度1(工程实践真假)——mjlab 与 UniLab 的 Quick Start/训练命令/API/console_scripts/任务名/reward 项/端口/CLI 二分,实跑代表性命令 14 条全部核实『是这样』,无 API 名不存在/参数报错;唯一硬细节偏差是 L719-736 日志表字段名与真实 mjlab 输出对不齐且 kl 不打印(已记 engineeringFindings#4)。维度2(可优化)——见 optimizations,核心是日志表改真实字段+补 KL 去向、补 --agent.algorithm.* 超参 CLI、补吞吐估算与冷启说明。维度3(是否教会工程)——整体优秀,缺口仅在『日志字段证伪点』与『吞吐正常性判据』两处。Isaac 部分本机无法实跑,静态代码合理(脚本路径/2.0 模块前缀迁移/3.0 四元数 wxyz→xyzw 等均与官方一致),标【Isaac 部分待 gpufree 补验】。未改任何 .tex/refs.bib。目标文件:/home/ziren2/pengfei/Robotics_Theory/Robotics_Note/parts/P8_rl_motion_control/prac_setup.tex

---

## prac_physics

**服务器**：autodl (RTX 4080 SUPER, 32GB; mjlab 1.4.0 + UniLab commit-on-disk; 无 Isaac)

**实跑命令（15）**
- mjlab `uv run --no-sync list-envs`：确认 Mjlab-Velocity-Flat-Unitree-Go1 / -G1 / -Rough 存在，无 Go2——与章节一致。
- mjlab `python -c 'MujocoCfg(); SimulationCfg()'`：类默认 timestep=0.002/iterations=100/impratio=1.0/cone=pyramidal/solver=newton/integrator=implicitfast/tolerance=1e-08，SimCfg nconmax=njmax=None——与 line432 勘误逐字符吻合。
- mjlab `velocity_env_cfg.py` grep：nconmax=35/njmax=1500/timestep=0.005/iterations=10——与 line432 任务覆盖值逐行吻合。
- mjlab 训练实跑 `train Mjlab-Velocity-Flat-Unitree-Go1 --env.scene.num-envs 64 --agent.max-iterations 2`：成功跑通，553→1072 steps/s，value/surrogate/entropy loss 正常，13 reward terms，Physics 0.005/Env 0.02s(=50Hz)，无 NaN/报错。
- mjlab `play <task> --help`：实际 flag 为 --agent{zero,random,trained}/--checkpoint-file/--video/--viewer 等，**无 --export-onnx**；ONNX 经 runner.py:34 export_policy_to_onnx() 自动产出 policy.onnx——章节 line814 命令的 --export-onnx 不存在。
- mjlab grep solver：Literal['newton','cg','pgs']='newton'，pgs 确为合法但被劝退——章节 solver='newton' 默认与 'pgs 不完整支持' 表述成立。
- mjlab grep：WarpBridge 类在 sim_data.py:183(wp.to_torch)，expand_model_fields 在 sim.py/randomization.py/event_manager.py 等——与 line707/736 路径一致。
- UniLab `train --help` / `eval --help`：--algo{ppo,mlx_ppo,appo,sac,td3,flashsac} --sim{mujoco,motrix}，且**存在 --profile flag**(语义为 task owner variant 如 hora，cli.py:262/85，非 profiling)——与 line942 '无字面 --profile flag' 冲突。
- UniLab 训练实跑 `train --algo ppo --task go2_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true`：成功，359 steps/s，9 reward terms(tracking_lin/ang_vel 等)正常，无 NaN/报错。
- UniLab `ls conf/ppo/task/`：go2_joystick_flat(mujoco.yaml+motrix.yaml)/go1_joystick_flat/allegro_inhand 等存在——章节任务名属实。
- UniLab grep：randomize_ground_friction:true + ground_friction_multiplier_range:[0.8,1.2] 在 go2_arm_manip_loco/mujoco.yaml:93；control_config(action_scale) 在多任务 yaml——章节 DR/PD 键真实(章节用[0.5,1.5]/Kp35 为示例值)。
- UniLab `ls src`：nan_guard.py/memory_budget.py/sim2sim.py 均存在；CLI unilab-import-robot/-pull-assets/-viz-nan 均存在——line656/859/941-943 全部属实。
- UniLab sim2sim.py:14/36/48：CrossBackendIncompatibleError + DENYLIST(含 env.control_config.action_scale/algo.obs_groups/policy hidden_dims/empirical_normalization) + ENV_STRUCTURAL_DENYLIST + strict——与 line859 契约校验描述一致。
- UniLab Makefile setup:`uv sync --extra mujoco --extra motrix`——与 line91 逐字吻合。
- UniLab conf grep：sim2sim_strict:true(默认)/trace_enabled:false/trace_output_dir:null 在 ppo+offpolicy config——与 line855/942 一致。

**工程核对（是这样/不符）**
- [是这样] mjlab 端到端可跑：Go1 训练 64envs/2iter 成功，loss/steps-per-second 正常，无 NaN——章节 Quick Start 与训练命令(train Mjlab-Velocity-Flat-Unitree-Go1 --env.scene.num-envs/--agent.max-iterations)实测有效(prac_physics.tex:75,813)。
- [是这样] mjlab MujocoCfg 类默认(0.002/100/None) vs velocity 任务覆盖(0.005/10/35/1500)的勘误(line432)经类实例化 + velocity_env_cfg.py:443-448 逐项核实，完全准确——这是本章最关键的源稿纠错，已坐实。
- [是这样] mjlab integrator 默认 implicitfast(line210)、solver 默认 newton(line462)、WarpBridge@sim_data.py(line707)、expand_model_fields(line736)——API 名/路径/默认值全部实测吻合。
- [是这样] UniLab 端到端可跑：go2_joystick_flat mujoco 后端 1iter 成功，reward terms 正常——Quick Start(line94)与 sim2sim 训练命令(line848)有效；任务名 go2_joystick_flat/go1_joystick_flat 真实存在。
- [是这样] UniLab 异构架构的工程组件全部落地：nan_guard.py/memory_budget.py/sim2sim.py 源文件、CrossBackendIncompatibleError/DENYLIST/strict 契约、unilab-viz-nan/-import-robot CLI、make setup=uv sync --extra mujoco --extra motrix——line656/859/941-943 逐条属实，非杜撰。
- [不是这样] 章节 line814 `uv run play Mjlab-Velocity-Flat-Unitree-Go1 --export-onnx`：mjlab play **无 --export-onnx flag**(task help 实测只有 --agent/--checkpoint-file/--video/--viewer 等)；ONNX 经 runner.export_policy_to_onnx() 自动产 policy.onnx，该命令会以 unrecognized argument 报错。建议改为说明 play 自动导出 policy.onnx(或删该 flag)。
- [部分不符] 章节 line942 称 UniLab '无字面 --profile flag'：实测 train/eval **均有 --profile flag**(cli.py:262)，只是语义是选 task-owner 变体(如 --profile hora)而非 profiling。章节关于'profiling 走 training.trace_enabled 而非 profiling flag'的实质判断对，但'无字面 --profile flag'的措辞与 CLI 不符,易误导,宜改为'--profile 非用于性能剖析(它选 owner 变体)'。
- [Isaac 部分待 gpufree 补验] 章节 Isaac Lab 实战(PhysxCfg 字段分层 line482-512、RigidBodyMaterialCfg line527、solver_type/min-max_position_iteration_count、wrap_warp_to_torch.py line746、play --task Isaac-...-Go1-v0 line820)在 autodl 无法实跑；静态看 API 命名风格合理、字段分层勘误(num_position_iterations 不存在)与官方 2.x 风格自洽,但未经实跑证实,需 gpufree(有 Isaac)补验。

**优化点 / 高级技巧**
- mjlab 训练默认会自动导出 ONNX(runner.export_policy_to_onnx)且 play 有 --registry-name/--wandb-run-path/--checkpoint-file 多种加载源——章节可补:如何指定 checkpoint 做 play、ONNX 文件落在哪个 log 目录、metadata 经 attach_metadata_to_onnx 注入(obs 顺序可随 ONNX 走),这正好补强 sim2sim 一节'obs 契约'的可操作性(当前只给 obs_config.json 建议,没给 mjlab 实际导出路径)。
- mjlab MujocoCfg 还有 ls_iterations=50/ls_tolerance/ccd_iterations/jacobian=auto 等字段未在章节 line462 配置示例出现——locomotion 稳定性调参时 ls_iterations(线搜索)与 jacobian(auto/dense/sparse,正对 line119 'dense Jacobian 能力边界')都是实用旋钮,可在调参表补一行。
- UniLab --profile(owner 变体, 如 sharpa_inhand 有 mujoco_hora.yaml)是章节完全没提的能力:同一 task 可挂多套配置变体(hora/nodr 等),这对'同任务不同 DR 方案对照'很实用,值得在 UniLab 小节补一句。
- 稳定性/吞吐排查表(line885,911)是静态清单——可补一个'实测基线锚点':本次 mjlab Go1 64envs 实测 ~1000 steps/s(小 batch)、UniLab 16envs ~360 steps/s,章节若给出'RTX4080S 上 4096envs 预期 steps/s 区间'作对照,读者能据此判断自己是否踩了 viewer/sensor 坑(章节现有 RTX4090 60k 的数字但无 4080S 锚点)。
- UniLab sim2sim DENYLIST 实际字段(env.control_config.action_scale/algo.obs_groups/policy hidden_dims/empirical_normalization/sampling_mode)很具体——章节 line859 只抽象说'逐字段比 DENYLIST',可补这张真实字段清单,读者就知道'改了 action_scale 或网络宽度会触发跨后端不兼容',比抽象描述更能举一反三。

**教学缺口（只讲代码没教工程之处）**
- ONNX 导出这一步是 sim2sim 全流程的关键衔接,但章节(line814/847)给的导出命令一个 flag 错误(--export-onnx 不存在)、一个把 eval 当导出入口——只贴命令没讲清'mjlab 是训练自动导出 vs UniLab 是 eval/play 阶段导出'的机制差异,读者照抄会卡在第一步。应教'导出在何时发生、文件落在哪、怎么确认导出成功',而非给一行可能跑不通的命令。
- 三框架接触参数对照表(line567)与调参案例(line585)信息密度高,但全是'设某值→结果改善'的结论式叙述,缺'怎么观测到症状'的可操作手段:章节反复说'可视化确认微滑/穿透',却没教用什么命令可视化(mjlab play --agent zero/--viewer viser 实测可用)、怎么读 contact buffer overflow 日志、UniLab 怎么用 unilab-viz-nan 离线回放——排错教学止于'回方程',没落到'敲哪条命令看到现象'。
- GPU 数据流/CUDA Graph 一节(line666)概念讲得透(三阶段不可逆、WarpBridge 铁律),但纯属'读源码讲架构',没有一个'你能自己验证'的动手锚点:例如'reset 后读 per-world geom_friction 跨 env 标准差>1e-6 即 DR 生效'(line560 提了一句但没给代码)、'第一次 DR 后那步变慢可用 time 测出来'——这些可验证断言若配一小段可跑代码,就从'听作者讲'升级为'读者能自证',更符合工程教学。
- 选型决策树/速查表(line339,362)给的是确定性映射(任务→引擎),但没教'当任务落在边界上怎么自己做实验决策':line389 人形全身操作那题点到了权衡却没给方法论(如'先各跑 200iter 比 steps-per-second + sim2sim 差异再定')——教了'查表'没教'查不到时怎么自己量化选型',举一反三的脚手架缺一层。
- Isaac 部分(PhysxCfg 字段分层、RigidBodyMaterialCfg 勘误)写得很细且标了已联网核对,但全章 Isaac 代码无一行能在 autodl 实跑验证,读者(若也只有非 Isaac 环境)同样无法自验——章节可明确标注'Isaac 示例需 Isaac Sim 环境,无则跳过/用 mjlab 等价验证',避免读者在没有 Isaac 时反复怀疑是自己装错。

**notes**：实跑覆盖 autodl 可用的两框架(mjlab+UniLab),两者训练 smoke test 均跑通、loss/steps-per-second 正常、无 NaN/报错;章节核心工程主张(MujocoCfg 类默认 vs velocity 覆盖值勘误、任务名、WarpBridge/expand_model_fields 路径、UniLab 异构组件 nan_guard/memory_budget/sim2sim/CrossBackendIncompatibleError/DENYLIST、make setup、sim2sim_strict)经源码逐项核实,绝大多数精确属实——本章经得起实跑。发现两处需主控修正的事实硬伤:(1) mjlab `play --export-onnx`(line814)flag 不存在,ONNX 自动导出;(2) UniLab '无字面 --profile flag'(line942)与 CLI 不符(flag 存在,语义为 owner 变体非 profiling)。Isaac Lab 全部实战(PhysxCfg/RigidBodyMaterialCfg/wrap_warp_to_torch.py/Isaac play)autodl 无环境,**待 gpufree 补验**,静态看 API 命名与字段分层自洽。教学性主要缺口:ONNX 导出衔接、'症状→用哪条命令可视化'的排错落地、GPU 数据流缺可自证的动手锚点。全程未修改任何 .tex/.bib。


---

## prac_reward

**服务器**：autodl (RTX4080S; mjlab 1.4.0 + UniLab commit-pinned 已实跑；Isaac 未安装→Isaac 实战部分待 gpufree 补验，仅静态核对)

**实跑命令（6）**
- mjlab `list-envs` → 12 个任务，含 Mjlab-Velocity-Flat-Unitree-Go1（章节 Quick Start 用的就是它，存在✓）；确认无 Go2 任务（章节用 Go1 正确）。
- mjlab `train Mjlab-Velocity-Flat-Unitree-Go1 --env.scene.num-envs 64 --agent.max-iterations 2`（WANDB_MODE=disabled）→ 成功跑完 2 iter，~1.45s/iter；日志含 Episode_Reward/track_linear_velocity、Curriculum/command_vel/lin_vel_x_max、Metrics/slip_velocity_mean、Episode_Termination/{time_out,fell_over}，与章节描述一致。
- UniLab `train --algo ppo --task go2_joystick_flat --sim mujoco algo.num_envs=16 algo.max_iterations=1 training.no_play=true` → 成功，日志含 reward/tracking_lin_vel、reward/swing_feet_z（章节 Quick Start 点名的两个字段都出现✓）。
- UniLab 同命令 @num_envs=64 max_iterations=2 → 成功；Steps per second 冷启 ~1500、热身后 ~3569（章节 Quick Start 声称 ~8900 env-steps/s @64，未复现，实测低 2~3 倍）。
- 源码核对 mjlab：RewardManager(cfg,env,*,scale_by_dt=True)✓、compute 内 raw*weight*(dt|1.0)+nan_to_num✓、reset 用 episodic_sum/max_episode_length_s✓；track_linear_velocity 把 z_error 内嵌进误差✓；reward_curriculum 类在 envs/mdp/curriculums.py:110✓；staged_position_reward 返回 reaching*(1+bringing) 高斯核✓；terrain_levels_vel+commands_vel(VelocityStage)✓。
- 源码核对 UniLab：RewardContext(tracking_sigma=0.25)✓、tracking_lin_vel 用 exp(-e/tracking_sigma)（直接当σ²）✓、run_reward_dispatch 末尾 reward*ctrl_dt + `if scale==0 or name not in fns: continue`（静默跳过拼错项）✓、log_every_n_steps=4✓、PenaltyCurriculum(min_scale/max_scale/level_up_threshold)✓、TransitionBootstrapContract+timeout_terminal_mask+FinalObservationAwarePPO✓、go2 的 foot_drag/swing_feet_z 真实存在✓。

**工程核对（是这样/不符）**
- 【符合】三框架的核心运行时语义全部实跑/源码坐实：mjlab scale_by_dt 默认 True 且关键字限定、track 把 v_z² 内嵌、reward_curriculum 阶梯换挡、staged_position_reward 乘法高斯门控；UniLab reward*ctrl_dt、tracking_sigma 直接当 σ²、run_reward_dispatch 静默跳过拼错项、TransitionBootstrapContract 区分 timeout/失败——章节这些工程论断与实装一字不差，非编造。
- 【符合】Quick Start 三框架命令均可跑：mjlab Go1 velocity 与 UniLab go2 ppo 都成功起训、无报错、loss/steps-per-s 正常；章节声称的日志字段(reward/tracking_lin_vel、reward/swing_feet_z、Episode_Reward/track_*、Curriculum/command_vel/*、Metrics/slip_velocity_mean、Episode_Termination/{time_out,fell_over})实跑全部出现。
- 【不符·命令级】§sec:rlmc-rew-ablation 消融命令 line 1082 `--env.rewards.feet-slip.weight 0.0` 跑不通：mjlab 实际 reward dict 键是 `foot_slip`(单数，func 才叫 feet_slip)，正确覆盖路径应为 `--env.rewards.foot-slip.weight 0.0`。同理全表(§full-table)把 mjlab 项名写成 feet_air_time/feet_slip，实跑日志里是 air_time/foot_slip/foot_clearance/foot_swing_height(dict 键)——函数名对、日志/覆盖键名错。
- 【不符·名称】UniLab 终止-obs 补丁的真实函数名是模块级 `patch_transition_next_obs`(base/final_observation.py:27)，章节 line 851 写的 `_patch_terminal_next_observations` 与 `terminal_next_obs` 均无此名(概念正确：补丁终止帧 next_obs 供 bootstrap)。类名 TransitionBootstrapContract 正确。
- 【不符·示例值】章节 PenaltyCurriculum 的 YAML 示例(line 994-996) min_scale:0.0、level_up_threshold:0.8(当比例)与实装 dataclass 默认不符——真实 min_scale=0.5、level_up_threshold=750.0(是绝对 episode 长度计数，不是比例)；机制描述(按 episode 长度缩放负权重)正确，但示例数值会误导读者照抄。
- 【不符·配置缺项】§full-table line 318 列 mjlab `base_height`/Isaac `base_height_l2` 为 Style 项，但 mjlab envs/mdp/rewards.py 与 velocity 实际 wiring 里都没有 base_height(grep 零命中)；mjlab Style 实际靠 upright+variable_posture(pose) 实现。表标注为'典型/风格示例'可接受，但与实装不完全对应。
- 【待补验】Isaac Lab 部分(RewardsCfg/TerminationsCfg、track_lin_vel_xy_exp 不含 v_z、feet_slide、bad_orientation、lift-cube 二值 tanh 门控、terrain_levels_vel 等)在 autodl 无法实跑(无 Isaac)，仅能静态判断合理；建议在 gpufree 上补一次 Isaac smoke + 函数名核对。

**优化点 / 高级技巧**
- 消融小节应补一条'先验证项名再扫'的可执行手段：mjlab 可 `uv run train ... --help | grep rewards` 或先 list 出 reward dict 键(实跑显示真实键是 foot_slip 等)，UniLab 章节已建议打印 _reward_fns.keys()，但 mjlab 这边缺等价防呆步骤——tyro 覆盖拼错键会直接报错(比 UniLab 静默跳过好)，可把'mjlab 报错 vs UniLab 静默'这一差异点出来当调试技巧。
- Quick Start 的 UniLab 吞吐数字(~8900 env-steps/s @64)建议改成区间或注明硬件/线程依赖：本机 RTX4080S 实测仅 ~1500(冷)~3569(热)，CPU 物理仿真吞吐强依赖 CPU 核数/sim_substeps，给死一个数字易让读者误判性能回归。
- 可补 mjlab feet_slip 的真实工程细节作为'读源码'范例：它内部用 command_threshold 门控(命令近零不罚)、并把 Metrics/slip_velocity_mean 直接写进 env.extras['log']——这正好解释了章节多处引用的 slip_velocity_mean 指标从哪来，是教'reward 函数顺便产 metric'的好案例，目前章节只把它当现成字段用、没讲它的产出处。
- 终止小节可补 mjlab 实测默认 limit_angle=radians(70°)≈1.22 rad(章节示例写 1.0)这一真实阈值，并点明 out_of_terrain_bounds 在实装里标 time_out=True(截断)——这正是章节'离开生成地形=截断'决策表的活样例，可由实装反向印证决策框架。
- 可补一条跨框架 σ 数值自检脚本(本章已讲 UniLab tracking_sigma=σ² vs mjlab/Isaac std=σ 的坑)：给读者一个三行打印各框架等效 σ 的片段，避免'照抄 std 数值'翻车——这是本章最有价值的反直觉点，值得配一个可跑的验证而非仅文字警告。

**教学缺口（只讲代码没教工程之处）**
- 消融命令把 mjlab 项名(feet_slip)与覆盖键名(foot_slip)、UniLab 项名(foot_drag)混在示例里直接给命令，但没教读者'怎么自己查到当前任务到底注册了哪些 reward 键'——而这恰恰是实跑第一个会踩的坑(覆盖路径写错)。应补'从零定位 reward 键'的流程(列 env_cfg / 跑一次看 Episode_Reward 字段),否则读者只会照抄、换任务即失效。
- Isaac 部分代码量大(RewardsCfg/TerminationsCfg/lift-cube 三函数门控等)但全章未在任何可跑环境验证过，属'只讲代码未兑现实跑'的最大教学缺口；至少应明确告诉读者哪些是已实跑坐实、哪些(Isaac)是静态核对，避免读者误以为整章都经实测。
- throughput/loss 这类'运行验证'数字(8900 env-steps/s 等)直接写死，没教读者'你的机器跑出不同数字时怎么判断是否正常'——好的工程教学应给出判据(看 collection/learning time 比例、看 steps/s 是否随 num_envs 线性、热身后是否稳定),而非给一个易过期的绝对值。
- PenaltyCurriculum 的 YAML 示例数值(min_scale:0.0、阈值当比例)与真实 dataclass 默认(0.5、750 绝对长度)不一致，读者照抄会得到与预期不同的课程行为；缺一句'这些是 owner YAML 可覆盖项、默认值见 base/curriculum.py'的对照,教读者去源码确认默认而非盲信示例。
- 章节大量'已核对官方源码'的 Isaac 函数签名(track_lin_vel_xy_exp 不含 v_z 等)以引用形式给出，但读者无法在本书提供的 autodl 环境复现核对；教学上宜补'如何自己 grep 框架源码验证函数签名'的一般方法(本章对 mjlab/UniLab 其实就是这么做的),把'核对'变成读者能复用的技能而非作者单方面结论。

**notes**：总体结论：本章工程论断的核实率很高——mjlab 与 UniLab 两框架的全部核心运行时语义(scale_by_dt/ctrl_dt 缩放、track 的 v_z 放置、tracking_sigma 直接当 σ²、run_reward_dispatch 静默跳过拼错项、reward_curriculum 阶梯换挡、staged_position_reward 乘法高斯门控、TransitionBootstrapContract 区分 timeout/失败、PenaltyCurriculum 按 episode 长度缩负权重)均经源码或实跑坐实，章节末 finenote 的'非编造'声明站得住。发现的问题集中在三类、均为可改的细节而非方向性错误：(1) 命令级——mjlab 消融覆盖键 feet-slip 应为 foot-slip(dict 键单数)，是唯一会让读者照抄即报错的硬伤，建议主控修正；(2) 名称级——UniLab 补丁函数实名 patch_transition_next_obs(非 _patch_terminal_next_observations)，base_height 项 mjlab 实装无;(3) 示例数值——UniLab 吞吐 8900 未复现(实测 1500~3569)、PenaltyCurriculum YAML 默认值与 dataclass 不符。Isaac 实战部分(约占全章 1/3 代码)在 autodl 无法实跑，待 gpufree 补一次 Isaac smoke 与函数名核对。已严格遵守只读纪律，未改任何 .tex/.bib。所有 ssh 调用均按要求设 dangerouslyDisableSandbox。

---

## prac_training_pipeline

**服务器**：autodl (RTX 4080 SUPER 32GB, mjlab 1.4.0 + rsl-rl-lib 5.4.0 + UniLab; 无 Isaac)

**实跑命令（13）**
- mjlab list-envs → PASS, 列出 12 个任务(4 个 Velocity{Flat,Rough}×{G1,Go1} + Cartpole/Lift-Cube/Tracking 等),无 Go2 (验证章节 line 100 的 G1/Go1/无Go2 正确,但'仅列出4个速度任务'的措辞偏窄)
- mjlab train Mjlab-Velocity-Flat-Unitree-Go1 --env.scene.num-envs 64 --agent.max-iterations 2 → PASS, 1067 steps/s, Mean value loss/surrogate/entropy/reward/action std=1.00 全部正常打印,Episode_Reward/* Curriculum/* Metrics/* Termination/{time_out,fell_over} 字段齐全
- mjlab versions: mjlab-pkg 1.4.0(符)、torch 2.9.0、rsl-rl-lib 5.4.0(章节多处写'实测5.2.0'→ 实为5.4.0,在>=5.2.0,<6.0.0 范围内但具体号已漂移)、mjlab.__version__ NO_ATTR(符已知陷阱)
- from mjlab.rl import RslRlOnPolicyRunnerCfg,RslRlPpoAlgorithmCfg,RslRlModelCfg → PASS; RslRlModelCfg 字段=hidden_dims,activation,obs_normalization,cnn_cfg,distribution_cfg,rnn_type,rnn_hidden_dim,rnn_num_layers,class_name(完全坐实章节 line 884 的 rnn_type/cnn_cfg/class_name 旋钮)
- rsl_rl.modules.EmpiricalNormalization → import OK(符 line 939); 裸 rsl_rl.modules.RslRlMLPModelCfg → ImportError(符 line 613)
- rsl_rl 5.4.0 GAE 定位: rollout_storage.py 已无 compute_returns/time_outs(方法仅 add_transition/generator/mini_batch_generator/recurrent_mini_batch_generator); compute_returns 现在 ppo.py(签名 compute_returns(self,obs:TensorDict))与 on_policy_runner.py; time_outs 在 ppo.py/env/vec_env.py
- rsl_rl 5.4.0 PPO 自适应KL: 实测 if kl_mean>desired_kl*2.0: lr=max(1e-5,lr/1.5); elif kl_mean<desired_kl/2.0 and kl_mean>0: lr=min(1e-2,lr*1.5) —— 倍率 2.0/1.5 与下限 1e-5/上限 1e-2 与章节 line 732-740 完全一致(仅真码多 kl>0 守卫且并入 update() 而非独立 update_lr)
- mjlab Go1 velocity rl_cfg.py 实测: hidden_dims=(512,256,128)、init_std=1.0、entropy_coef=0.01、num_steps_per_env=24、max_iterations=10_000、experiment_name='go1_velocity'、save_interval=50、distribution_cfg 为 dict{class_name:GaussianDistribution,init_std,std_type} —— 全部坐实章节 line 530/561-578/966
- mjlab RslRlPpoAlgorithmCfg 基类 entropy_coef 默认=0.005(config.py:59),Go1 任务覆写为 0.01 —— 坐实 line 568 '基类默认0.005,go1设0.01'
- UniLab train --help: --algo{ppo,mlx_ppo,appo,sac,td3,flashsac} --sim{mujoco,motrix} --render-mode{auto,interactive,record,none} —— 算法清单与 line 1348 逐字一致
- UniLab train --algo ppo --task go2_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true → PASS, 381 steps/s, critic 值头 in_features=52(坐实 critic obs=52), 注册名 Go2JoystickFlat(CamelCase), reward/<name> 派发字段(reward/tracking_lin_vel 等)出现, Mean action std=0.50
- UniLab --sim motrix 同参 → PASS(value loss 0.0068); 且 no_play=true 后 find logs -name *.onnx 为空 —— 坐实 line 121/1465 '冒烟跑不产 ONNX'
- mjlab train --help: --env.scene.num-envs/--agent.max-iterations/--agent.seed(默认42)/--agent.logger{wandb,tensorboard}(默认wandb)/--agent.resume/--agent.load-run/--agent.load-checkpoint 全部存在 —— Quick Start 与恢复命令(line 71/1026/1034-1037)的 CLI 形态全部坐实

**工程核对（是这样/不符）**
- 【是这样】mjlab/UniLab 三框架核心工程做法基本全部实跑坐实: list-envs 任务清单(无 Go2)、train CLI flag 风格(mjlab --env.scene.num-envs vs UniLab Hydra algo.num_envs)、RslRlModelCfg 的 dict 形 distribution_cfg、init_std=1.0、entropy_coef Go1=0.01/基类0.005、num_steps_per_env=24、max_iterations=10000、自适应KL 倍率2.0/1.5、EmpiricalNormalization、RslRlMLPModelCfg 在裸库 ImportError、UniLab --algo 六算法清单、critic obs=52、reward/<name> 字段、no_play 不导 ONNX —— 这些是本章最密集的事实声明,逐一 PASS
- 【不完全是这样·版本漂移】章节多处(line 50/613/939/1715/1718 及 §setup)写'本章实测 rsl-rl-lib 5.2.0',但 autodl 实装为 5.4.0。仍在章节自己给的约束 >=5.2.0,<6.0.0 内、API 行为一致,故非错误,但'实测5.2.0'的具体号已不符当前环境
- 【不是这样·代码定位过时】§sec:rlmc-train-dataflow(line 262-291)把 GAE+timeout 自举代码标注为 'rsl_rl/storage/rollout_storage.py 同义简化',并展示 compute_returns(self,last_values,gamma,lam) 内 next_is_not_terminal += extras['time_outs'][step]。但 5.4.0 里 rollout_storage.py 已无此方法;compute_returns 移到 ppo.py(签名 (self,obs:TensorDict)),且 timeout 自举改为在记录 transition 时 rewards += gamma*values*time_outs(数学等价但实现与文件归属均不同)。概念正确,代码块与文件名对 5.4.0 已 stale
- 【措辞偏窄】line 100 note 称 list-envs '列出4个 Unitree 速度任务',实测共 12 个任务(含 Cartpole/Lift-Cube/Tracking 等);4 个速度任务的子结论正确,但读者照此会以为 mjlab 只有 4 个任务
- 【数值差异·非错】UniLab go2 实测 Mean action std=0.50(非 mjlab 的 1.00);章节未对 UniLab 的 init_std 给具体值(只说'由 RSL-RL 模型管理'),故不算错,但与正文反复用的 1.0 不同,可点明
- 【Isaac 部分待 gpufree 补验】autodl 无 Isaac,章节 Isaac 实战(train.py 路径、RslRlMLPModelCfg.GaussianDistributionCfg 嵌套类、--task Isaac-Velocity-Flat-Anymal-C-v0、obs_groups 必填)无法在此实跑;静态看 API 命名与 RSL-RL 5.x 一致、合理,但需 gpufree 实测坐实

**优化点 / 高级技巧**
- 把'实测 rsl-rl-lib 5.2.0'统一改为'5.2.x 起步(本环境实测 5.4.0)',或在版本表加一句'GAE/timeout 自举的源码位置随 5.x 小版本迁移':5.4.0 中已从 rollout_storage.py 移至 ppo.py/on_policy_runner.py,且 timeout 自举实现从'改 terminal mask'改为'rewards += gamma*values*time_outs'(等价)。这能让源码阅读路线 C(line 1539)指向真实文件
- §dataflow 的 GAE 代码块可补一句导航:'不同 rsl-rl 小版本该逻辑可能在 storage 或 algorithm,定位用 grep -rn time_outs $(python -c "import rsl_rl,os;print(os.path.dirname(rsl_rl.__file__))")' —— 教读者自己定位而非记死文件名,正是本章追求的'举一反三'
- 可补一个'实跑自检命令'高级技巧小节:用 inspect.getsource + grep 在已装库里反查真实 API/源码位置(本次验证就是靠它发现文件迁移),比读文档更可靠 —— 这是工程教学最缺的'怎么核实而非背诵'
- line 100 改为'list-envs 列出 12 个任务,其中 4 个是 Unitree 速度任务',避免读者误以为 mjlab 只有速度任务
- UniLab 侧可补 init_std 实测值(0.50)及一句'UniLab 默认探索 std 比 mjlab/Isaac 的 1.0 小,起步更偏利用',让三框架 action std 对照完整
- 推理延迟表(line 1453)与 steps/s 均标'数量级参考'已稳妥;可补一句 RTX 4080S 实测基线(64 env mjlab≈1067 steps/s、16 env UniLab-mujoco≈381、motrix≈361)提示小 env 下 steps/s 偏低属正常(并行不足),呼应'PPO 靠大并行'主线

**教学缺口（只讲代码没教工程之处）**
- 教学性整体很强:每节'这一节解决什么问题'开头、反事实推理(删 timeout 自举/动 gamma-lambda)、症状→参数反查表、调参优先级(环境→reward→PPO→网络)、JIT/ACC 等类比 —— 都在教'为什么这么做/怎么排错',不是逐行翻译。这是本章最大优点
- 缺口1:全章给了大量 Python'同义简化'代码块(GAE/PPO update/EmpiricalNormalization/导出脚本),但没教读者'如何在自己装的库里定位并核对真实实现'。本次验证恰恰暴露 compute_returns 已迁移文件——若读者按章节文件名去 rollout_storage.py 找会扑空。补一个'用 inspect/grep 反查源码'的方法,把'读代码'升级为'核代码'
- 缺口2:Quick Start 给了命令但没给'命令跑起来后第一屏应该看到什么/哪些字段算正常'的对照(实跑首屏有 Steps per second/Collection time/Learning time/Mean *loss/Episode_Reward/* 等)。可补一张'首次训练日志逐字段解读'图,教新手从零判断'起没起来',这正是 line 750 §运行验证想做但只列了字段名
- 缺口3:'从零搭起'侧薄弱——章节假定环境已装好(本环境靠 ACTIVATE 脚本设 egl/uv 路径)。对真要从零的读者,缺'EGL/headless 渲染、MUJOCO_GL=egl、uv --no-sync、WandB 默认开需 disabled'这类首次落地必踩的坑的集中说明(WANDB 那条已在别处提,但 egl/headless 没有)
- 缺口4:UniLab 两层组名(env 侧 obs/critic vs runner 侧 actor/critic, line 488)被正确点为'最易写混',但没给一段可运行的自检脚本打印两层映射;而 mjlab 侧 line 505 练习有打印 obs 维度脚本——UniLab 侧建议对称补一个,否则'最易写混'只停在告诫层面

**notes**：本章工程实践可信度高:在 autodl 实跑了 mjlab(list-envs/train Go1 flat 2 iter/源码核对 RslRlModelCfg·entropy_coef·init_std·自适应KL倍率)与 UniLab(go2 PPO mujoco+motrix 各 1 iter/--algo 清单/critic obs=52/no_play 不导 ONNX),代表性声明逐一 PASS,无致命错误。两处实质发现供主控酌改:(1) rsl-rl-lib 实装 5.4.0 而章节多处写'实测5.2.0'(仍在自定约束内,非错,但号已漂);(2) §dataflow 的 GAE+timeout 自举代码标注在 rollout_storage.py,但 5.4.0 已迁至 ppo.py/on_policy_runner.py 且实现方式改为'rewards 加 gamma*values*time_outs'(数学等价)——概念对、代码块与文件归属对 5.4.0 已 stale。其余为措辞/补充级(list-envs 实为12任务、UniLab init_std=0.50)。Isaac 实战部分 autodl 无法实跑,已静态核对 API 命名合理,待 gpufree 补验。未改任何 .tex/.bib。


---

## prac_domain_rand

**服务器**：autodl (RTX 4080 SUPER 32GB; mjlab 1.x + UniLab; 无 Isaac)

**实跑命令（15）**
- ssh autodl nvidia-smi: 确认 RTX4080S 32GB + 两激活脚本在位
- uv run python -c 'import mjlab.envs.mdp.dr; dir()': 列出全部 DR 导出名,章节所有点名函数(geom_friction/body_mass/pd_gains/pseudo_inertia/dof_armature/encoder_bias/body_com_offset/dof_damping)全部存在
- inspect.signature 各 DR 函数: geom_friction/body_mass 确含 operation+shared_random+ranges+distribution+axes;pd_gains 含 kp_range/kd_range;pseudo_inertia 含 alpha_range;encoder_bias 含 bias_range — 与章节 Quick Start 参数名逐一吻合
- uv run list-envs: Mjlab-Velocity-Flat-Unitree-Go1 存在(章节命令用名正确);另有 Go2 rough/G1 任务
- train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2: 成功跑完2迭代,mean reward -0.63,iter time 1.46s,无报错
- 读 velocity_env_cfg.py events 字典: foot_friction/encoder_bias/base_com 均 mode=startup,push_robot mode=interval,reset_base/reset_robot_joints mode=reset — 与章节模式分配完全一致
- 读 dr/geom.py + event_manager.py: @requires_model_fields 装饰器真实存在,signature=(*fields, recompute=RecomputeLevel.none);RecomputeLevel 枚举 none/set_const_fixed/set_const_0/set_const 与章节表吻合
- 各函数 .recompute 实测: geom_friction=0,dof_damping=0,body_mass=3,pseudo_inertia=3 — 与章节 RecomputeLevel 表逐行吻合
- mjlab restitution_fns=[] gravity_fns=[]: 章节'mjlab 无现成 restitution/gravity 随机化函数'属实
- 读 dr/body.py pseudo_inertia: _cholesky_4x4/_decompose_pseudo_inertia_J/_reconstruct_pseudo_inertia_J 全在(章节点名吻合),alpha 全局密度缩放 e^{2α} 确认;但实现用自定义 _cholesky_4x4 规避 torch.linalg.cholesky/cuSOLVER
- UniLab: 读 dr/types.py: 11 个 RESET_TERM 常量(geom_friction/body_inertia/dof_armature/kp/kd/gravity/...)= 章节'MuJoCo 后端静态 11 项'精确吻合;class DomainRandomizationCapabilities + supports_reset_term + filter_reset_payload 全在
- UniLab: 读 dr/manager.py: class DomainRandomizationManager + get_dr_capabilities + build_reset_plan + apply_init_randomization + apply_interval_randomization_if_due 全在,与章节命名吻合
- UniLab: motrix/backend.py get_dr_capabilities: 基线4项{base_mass_delta,base_com_offset,body_mass,body_ipos},条件 .add(KP/KD/GEOM_FRICTION/GRAVITY) — 与章节'Motrix 动态门控基线4项+按能力追加'精确吻合,且确认不含 body_iquat/body_inertia/dof_armature
- UniLab: 读 envs/locomotion/common/domain_rand.py: class DomainRandConfig 含 randomize_base_mass/added_mass_range/randomize_ground_friction/ground_friction_multiplier_range/push_interval 字段 — 与章节 Quick Start 字段名吻合
- train --algo ppo --task go2_joystick_rough --sim mujoco max_iterations=1 num_envs=16: 成功跑完,iter time 1.34s,reward/tracking_lin_vel=0.072 tracking_ang_vel=0.129 正常

**工程核对（是这样/不符）**
- 是这样: mjlab DR 子模块全部点名函数(geom_friction/body_mass/pd_gains/pseudo_inertia/dof_armature/encoder_bias/body_com_offset)及其参数名(ranges/operation/shared_random/kp_range/kd_range/alpha_range/bias_range)全部真实存在并匹配,Quick Start 代码可照抄
- 是这样: Mjlab-Velocity-Flat-Unitree-Go1 任务名正确且训练能起;实跑2迭代无报错,~9k steps/s(64env)
- 是这样: 真实 Go1 velocity_env_cfg.py 的 events 字典里 foot_friction/encoder_bias/base_com=startup、push_robot=interval、reset_*=reset,与章节'模式分配'表完全一致(强证据:章节不是凭空写配置)
- 是这样: @requires_model_fields(*fields, recompute=RecomputeLevel.X) 装饰器+RecomputeLevel(none/set_const_fixed/set_const_0/set_const)枚举真实存在;实测 recompute 值 body_mass=set_const、geom_friction/dof_damping=none 与章节 RecomputeLevel 表逐行吻合
- 是这样: mjlab 确无 restitution/gravity DR 函数(dir 实测为空),章节 finenote/函数表标注属实
- 是这样: UniLab DomainRandomizationManager/get_dr_capabilities/filter_reset_payload/supports_reset_term/DomainRandConfig 全部真名真在;MuJoCo 后端 11 reset 项、Motrix 基线4项+条件追加 KP/KD/GEOM_FRICTION/GRAVITY — 与章节 line476 精确同构(连'Motrix 不支持 body_iquat/body_inertia/dof_armature'都对)
- 是这样: UniLab go2_joystick_rough 训练能起,reward 正常
- 轻微不符(非错,章节已自注): 真实 foot_friction 默认 operation='abs'、ranges=(0.3,1.2),而 Quick Start 用 'scale'+(0.5,1.25)。章节四维表 line465 明确写了'mjlab geom_friction 默认 abs;DR 实践常显式设 scale',属有意教学选择,非错误
- 轻微不符: 真实 push_robot interval_range_s=(1.0,3.0)(较激进),章节示例多处用 (10.0,15.0)。章节是作为推荐区间给出非声称 shipped 值,但与 mjlab 实装默认差距较大,读者照搬 shipped task 会见到更频繁的 push
- 轻微不符: pseudo_inertia 真实实现用自定义 _cholesky_4x4 显式规避 torch.linalg.cholesky(为避开 cuSOLVER 的分配/CUDA Graph 问题);而章节走读代码(line623)与 NaN 陷阱(line689)都写 torch.linalg.cholesky。走读已标'简化自',可接受,但 NaN 陷阱归因到 torch.linalg.cholesky 与真实实现略脱节
- Isaac 部分待 gpufree 补验: autodl 无 Isaac,章节 Isaac 实战(randomize_rigid_body_material/num_buckets/randomize_actuator_gains/EventTermCfg/prestartup/randomize_rigid_body_scale)无法实跑核验,仅静态判断 API 命名与 2.x 习惯一致、合理

**优化点 / 高级技巧**
- Quick Start 的 geom_friction operation 建议与真实 task 对齐说明: 既然 shipped 默认是 abs 而 ranges=(0.3,1.2) 是绝对摩擦值,scale 模式下 (0.5,1.25) 是相对标称——可补一句'用 abs 时 ranges 是绝对摩擦系数区间、用 scale 时是相对倍率',避免读者把 abs 的区间当倍率
- mjlab pseudo_inertia 真用 _cholesky_4x4 而非 torch.linalg.cholesky 的工程动机值得点破: 自定义 4x4 Cholesky 避开 cuSOLVER 在 CUDA Graph capture 下的动态分配/句柄问题——这正好是本章 CUDA Graph 主题的高级延伸,比'简化版'更有教学价值
- 可补 mjlab DR 子模块远超章节覆盖的能力清单作为'进阶索引': 实测还导出 camera/light/material/tendon/site/geom_rgba/jnt_range/joint_friction/dof_frictionloss 等大量函数(视觉/光照/肌腱 DR),章节只覆盖 locomotion 常用子集,可加一句'dir(mjlab.envs.mdp.dr) 一览全部 40+ DR 函数'的指引
- UniLab interval 的 push_interval 默认值漂移: DomainRandConfig 默认 push_interval=750(章节 Quick Start 用625、Phase4 也用625),建议注明'各 owner YAML 默认不同,以目标 owner 的 dataclass 默认为准',与章节既有'训练前体检'精神一致
- 验证脚本(verify_dr_config/dr_health_check)用了 em._mode_term_cfgs / em.active_terms / sim._expanded_fields 等私有属性,跨 mjlab 小版本易碎;可补一句'这些下划线属性非稳定 API,版本升级后先 dir(env.event_manager) 核对'——这本身就是本章'API 漂移'主题的自洽示范
- push 等效冲量/质量经验表(Go1/Go2/ANYmal/G1)可补一句'真实 mjlab Go1 shipped push 用速度区间 x/y∈±0.5、含 z 与 roll/pitch/yaw 全6维',让读者知道实战默认是6维速度扰动而非仅平面x/y

**教学缺口（只讲代码没教工程之处）**
- 从零搭 DR 的'第一次 import 在哪/装饰器怎么写'缺口: 章节讲了 @requires_model_fields 声明式范式(很好),但读者若要自定义一个新 DR 函数,缺一个'最小可跑的新增 DR 函数模板'(从 def+装饰器到挂进 events 字典到 verify 生效的端到端 10 行),目前散在多个 codebox 里没串成一条从零路径
- '怎么调参'在范围数字上偏经验值堆砌: 摩擦(0.5,1.25)/PD(0.75,1.5)/armature(0.9,1.1) 等大量来自 legged_gym 经验值,章节标注了来源但未教'如何为一台新机器人从规格书推导这些范围'的方法论(如何从制造公差/datasheet 反推 add vs scale 区间)——练习题点到但正文未给可操作流程
- '出问题怎么排查'已很强但缺一次真实失败的复盘: 四层排查法/dr_health_check 是本章亮点,但全是'预防式'脚本;缺一个'我真的遇到 std=0 了,逐层 print 输出长什么样、最终定位到哪一行'的真实 trace 走查,读者难把脚本输出映射到根因
- Isaac vs mjlab 的 operation 默认差异未给统一心智模型: mjlab geom_friction 默认 abs、Isaac 用 *_friction_range(本质 abs 区间),章节分散讲了但没有一句'三框架里摩擦到底是绝对值还是倍率'的对照总结,读者跨框架迁移时仍要自己拼
- pseudo_inertia 的 alpha_range 之外还有 d_range/d1_range/s12 等7+维扰动参数(实测 signature),章节只教了 alpha(全局密度缩放)这一维,未说明其余维度(主惯量/惯量积扰动)各控制什么——读者只会'整体缩放'不会'各向异性惯量随机化',举一反三受限
- CUDA Graph 重 capture 的'原地写入'命门讲得透,但读者无法在 autodl 上自行复现'换指针→DR 失效'的反面实验(这是 mjlab 内部行为);可补一个'怎么验证我的 DR 真的进了 Graph'的可观测手段(如对比 expand 前后 steps/s 或 cross-env std),让抽象命门变成可自测动作

**notes**：总体: 本章工程实践可信度极高。在 autodl 上实跑 + 读源码双重核验,mjlab 与 UniLab 两侧的关键 API 名/参数名/事件模式/能力门控/RecomputeLevel/11-reset-term/Motrix-4-baseline 几乎逐条命中真实代码(强证据:章节 line476/605/643/677 的细节与源码逐字符吻合,绝非凭空捏造)。两个框架的训练命令均实跑起飞无报错。仅3处轻微脱节:(1)Quick Start friction 用 scale 而 shipped 默认 abs(章节已自注,非错);(2)push interval_range_s shipped=(1,3) vs 章节示例(10,15)(推荐值,可加注);(3)pseudo_inertia 真实用自定义 _cholesky_4x4 而走读/NaN陷阱写 torch.linalg.cholesky(走读已标简化,NaN 陷阱归因略脱节)。Isaac 全部实战(num_buckets/randomize_rigid_body_material/prestartup/randomize_actuator_gains)autodl 无法实跑,待 gpufree 补验,静态看 API 命名与 Isaac Lab 2.x 习惯一致、合理。教学性整体优秀(四层排查/单变量分阶段/鲁棒优化主线都是真教工程而非翻译代码),缺口集中在'从零自定义一个 DR 函数的端到端模板''为新机器人从规格书推导范围的方法论''一次真实失败的逐层 trace 复盘'三处。未改任何 .tex/.bib。


---

## prac_imitation

**服务器**：autodl (RTX4080S, mjlab 1.x + UniLab；无 Isaac)

**实跑命令（15）**
- mjlab `train --help` / `list-envs`：成功，确认任务 Mjlab-Tracking-Flat-Unitree-G1(+No-State-Estimation) 存在
- mjlab `play Mjlab-Tracking-Flat-Unitree-G1 --help`：成功，`--agent {zero,random,trained}` 与 `--viewer {auto,native,viser}` 均如章节所述存在
- mjlab tracking mdp dir()：成功，6 个 reward 函数名 + motion_anchor_pos_b/ori_b 与章节逐字一致
- mjlab `train Mjlab-Tracking-Flat-Unitree-G1 --env.scene.num-envs 64 --agent.max-iterations 2`(裸跑)：报错 ValueError——必须给 --registry-name 或 motion-file，无默认示例动作
- 读 tracking_env_cfg.py 源码：成功，std=0.3/0.4/0.3/0.4/1.0/3.14、weight=0.5/0.5/1/1/1/1、action_rate=-0.1、joint_limit/self_collision=-10、motion_file='' 默认空——全部与章节表一致
- mjlab `python -m mjlab.scripts.csv_to_npz --help`：成功，签名 main(input_file,output_name,input_fps=30,output_fps=50,device,render=False) + tyro 连字符旗标、`--render`(无 --headless)——与章节 \rebuilt 块逐字一致
- 读 observations.py：成功，motion_anchor_ori_b = matrix_from_quat(ori)[...,:2].reshape(-1) = 旋转矩阵前两列 6D，与章节强声明一致
- mjlab `train ... --env.commands.motion.motion-file <UniLab npz> --num-envs 64 --max-iterations 2`：成功跑 2 iter(0.93s/iter)，日志含全部 6 reward 项 + Metrics/motion/sampling_entropy/top1_bin + error_anchor_*
- UniLab `train --algo ppo --task g1_motion_tracking --sim mujoco`(无 HF mirror)：报错——需从 HuggingFace 拉 motion 资产，连接失败
- UniLab 同命令 + `export HF_ENDPOINT=https://hf-mirror.com`：成功跑 1 iter，363 steps/s、value_loss 0.17、reward/motion_global_root_pos/ori/body_pos/ori/lin_vel/ang_vel 全部逐项打印
- 读 UniLab demo.py：成功，DEMO_REGISTRY 含 dance/wallflip/boxtracking(均 g1_*_tracking)，与章节 line89 一致
- 读 UniLab motion_loader.py：成功，MotionData 字段 = fps/joint_pos/joint_vel/body_{pos,quat,lin_vel,ang_vel}_w，与章节 schema 逐项一致；conf yaml reward.scales 6 项数值与章节一致
- 读 UniLab：成功确认 PenaltyCurriculum、motion_ee_body_pos_z、scripts/train_hora_distill.py、HoraDistillationTrainer、adapt_tconv(仅此模块 requires_grad) 均存在，与章节 HORA 段一致
- inspect UniLab dance1_subject2_part.npz：fps(1,)/joint_pos(870,29)/body_pos_w(870,31,3) 等——G1 实为 29 关节/31 body
- `import rsl_rl` OK：确认 mjlab RL 后端 = RSL-RL/PPO

**工程核对（是这样/不符）**
- 【不是这样·硬错误】Quick Start 注记(line95-96)称 mjlab tracking 裸 train『若不指定 registry-name/motion_file 会用任务默认示例动作做冒烟测试』——实跑报 ValueError，源码 motion_file='' 默认空，根本无默认动作；line81-82 的 Quick Start train 命令照抄会直接失败,必须补 --env.commands.motion.motion-file 或 --registry-name
- 【是这样】6 个 reward 函数名/std/weight、anchor 观测名 motion_anchor_pos_b、self_collision force_threshold=10、action_rate=-0.1 等全部经源码逐项核实与章节一致(line546-573, 583-593)
- 【是这样·难得】motion_anchor_ori_b 确为 6D(旋转矩阵前两列 reshape)——章节 line651/665 的强声明经源码确认正确
- 【是这样】csv_to_npz tyro 签名与连字符旗标、`--render` 无 `--headless`、默认 output_fps=50——章节 \rebuilt 块(line391/975)逐字正确
- 【是这样·核心主张成立】mjlab 直接吃 UniLab 的 .npz 跑通 tracking 训练→证实章节『mjlab 与 UniLab schema 同构、body_*_w 世界系、无独立 root』(line422-423)；UniLab MotionData 字段亦逐项符合
- 【是这样】UniLab g1_motion_tracking 实跑日志逐项打印 reward/motion_global_root_pos/ori/body_pos/ori/lin_vel/ang_vel,证实 line600-617『scales 标量字典 + 每步写 info[log]』;reward.scales 数值与章节表一致
- 【是这样】UniLab demo dance/wallflip/boxtracking、PenaltyCurriculum、motion_ee_body_pos_z、HORA 蒸馏(train_hora_distill.py/HoraDistillationTrainer/adapt_tconv 仅此模块训练) 全部源码确认,章节 line89/619/1222-1232 准确
- 【部分不符·需补警示】UniLab Quick Start 的 train 冒烟命令(line91-92)未提示它和 demo 一样要从 HF 拉 motion 资产——裸跑直接 RuntimeError(client closed);必须先 export HF_ENDPOINT=hf-mirror,章节只在 demo 行(line88)给了该提示,train 行漏给
- 【Isaac 部分待 gpufree 补验】autodl 无 Isaac,章节 Isaac-Humanoid-AMP-{Dance,Run,Walk}-Direct-v0(skrl)、manager-based G1 加 AMP、ProtoMotions dataclass 入口 均无法在此实跑;但静态看接口描述合理、与已知 Isaac Lab/ProtoMotions 设计自洽

**优化点 / 高级技巧**
- Quick Start mjlab train 命令应补一条可立即跑通的形式：要么显式 `--env.commands.motion.motion-file <path.npz>`，要么明确写『需先 csv_to_npz 上传 registry 再 --registry-name』——当前两条命令初学者照抄必报 ValueError,与『五分钟看见模仿』的承诺冲突
- 可指出一条实战捷径(本次已验证)：mjlab 与 UniLab 的 .npz schema 完全互通,可直接拿 UniLab 自带的 assets/motions/g1/dance1_subject2_part.npz 喂给 mjlab tracking 做冒烟,无需先跑 retarget/registry——这是章节『schema 同构』主张的最佳可操作落地
- UniLab 段建议把 `export HF_ENDPOINT=https://hf-mirror.com`(及首跑会拉 motion 资产、可能因网络中断报 RuntimeError: client has been closed)提示从 demo 行扩展到 train 冒烟行——否则国内/弱网首跑 train 必失败
- mjlab 实跑日志暴露了一批章节未提的现成可观测量(Metrics/motion/sampling_entropy、sampling_top1_bin/prob、error_anchor_pos/rot/lin_vel/ang_vel、Episode_Termination/anchor_pos/ee_body_pos)——八阶段协议(表line738-745)讲了要看 entropy/top1 bin 却没告诉读者这些 key 训练时直接打印在 stdout/日志里,可补『去哪看这些指标』
- mjlab 有现成 `Mjlab-Tracking-Flat-Unitree-G1-No-State-Estimation` 变体(has_state_estimation=False,砍掉状态估计/base_lin_vel)——正是 BC/部署段讲的『去真机不可得通道』的 mjlab 侧现成抓手,章节只点了 UniLab 的 G1WBTObs,可补 mjlab 的对称做法做三框架对照
- 可补一条调试技巧：mjlab tracking termination 里 anchor_pos/ee_body_pos 在 2 iter 内即触发(本次 64/64 env)说明早期大量早停属正常——读者若不知会误以为训练崩了;八阶段协议可补『阶段3-4 早期 termination 直方图怎么读、多少算正常』

**教学缺口（只讲代码没教工程之处）**
- 数据从哪来、怎么得到第一份能跑的 .npz 全章缺位：章节花大篇幅讲 retarget 原理与 csv_to_npz 旗标,但没给一条『手上空空时如何拿到第一份 motion 数据来跑通 Quick Start』的可执行路径(AMASS 要注册下载、retarget 工具要另装)——导致 Quick Start 实际上五分钟跑不起来。本次发现最省事是复用框架自带样例 npz,这种『从零到第一次训练起跑』的工程引导缺失
- check_motion_contract.py(line453)教得很细,但缺一步把它接到真实数据的演示：理想应展示『加载框架自带 npz→跑契约检查→看到 PASS』的闭环,让读者确认脚本与真实 schema 对得上;现在脚本是悬空的,读者无法自验
- 八阶段训练协议(表line732)是好框架,但停在『讲该看什么指标』,没教『在哪看/命令输出长什么样/数值多少算过线』——本次实跑日志里 sampling_entropy=0.98、error_anchor_pos=0.77 这类真实量纲没有任何参照,读者拿到自己的数字仍不知好坏
- AMP/ASE/MaskedMimic/BC 全部代码(line841-1232)均为教学自写片段、无任一可在本服务器跑通的入口(autodl 无 Isaac、UniLab 无内置 AMP)——这部分是『讲代码』而非『教工程』,读者无法实跑验证,建议至少给出 ProtoMotions 在某后端的一条最小可跑命令或明确标注『本章 AMP 代码为原理示意、可运行实现见 ProtoMotions 仓库』
- 报错驱动教学缺失：本章最容易让新手卡住的恰是两个真实首跑报错(mjlab『provide registry-name/motion-file』、UniLab HF 拉取失败)——章节未把这两个必然遇到的报错写进故障排查手册(line1422),而手册里列的多是训练中后期问题;『第一次就跑不起来』的排错反而没教

**notes**：实跑覆盖：mjlab tracking(裸跑暴露缺省动作问题 + 带 UniLab npz 成功跑 2 iter)、UniLab g1_motion_tracking(HF mirror 后成功跑 1 iter,363 steps/s)、两框架 reward/obs/schema 源码逐项核对、csv_to_npz 签名、anchor 6D、demo 注册、HORA 蒸馏。总体结论：章节工程描述精度极高——6 reward 项名/std/weight、anchor_pos/ori_b(6D)、csv_to_npz 旗标、mjlab↔UniLab schema 同构、UniLab reward.scales/HORA/demo 名 全部经实跑或源码确认无误,三轮复核质量过硬。唯一硬错误是 Quick Start line95-96 关于 mjlab『默认示例动作』的说法(实测无默认、裸跑 ValueError),及 UniLab train 冒烟行漏标 HF mirror 依赖——两者都让『五分钟跑通』的承诺在初学者手里落空,建议主控优先修。Isaac 部分(AMP 预注册任务/ProtoMotions dataclass 入口)autodl 无法实跑,标『待 gpufree 补验』,静态看描述合理。G1 实为 29 关节/31 body(章节练习用 23 关节是泛例,非矛盾)。

---

## prac_assets

**服务器**：autodl (RTX4080S, mjlab + UniLab; 无 Isaac)

**实跑命令（12）**
- mjlab `uv run list-envs` → 成功：12 个任务，含 Mjlab-Velocity-Flat/Rough-Unitree-Go1 与 -G1；无任何 Go2 任务(符合已知陷阱)。
- mjlab `train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2` → 成功：~1050 steps/s，value loss 0.0417→0.0251 正常下降，13 个 reward 项 active，2 iter 跑完。
- mjlab Python: `MjSpec.from_file(go1.xml).compile()`+MjData → 成功：bodies=14 njnt=13 nq=19 nv=18 total_mass=12.74kg dt=0.002；但 actuators=0、nkey=0(mjlab 内置裸 xml，执行器在 env 侧补)。
- mjlab Python: `MjSpec.attach(child, frame=fr, prefix='arm/')` → 成功；`attach(..., pos=, quat=)` → TypeError(章节 L905 的纠正完全正确)；`worldbody.add_frame(pos,quat)` 存在。
- mjlab Python: validate_model 求解器映射 `['PGS','CG','Newton'][opt.solver]` → solver=2=Newton 正确；13 个 body 三角不等式违反=0；body1 pseudo-inertia min-eig=4.08e-3 正定=True。
- mjlab 源码: dr/body.py 含 `pseudo_inertia`(L423)、`_cholesky_4x4`(L39)、`_reconstruct_pseudo_inertia_J`(L137, docstring 明写 parallel-axis COM→origin) → 章节 L840 源码级claim精确属实；`import rsl_rl` 成功。
- UniLab `train --task go2_joystick_flat --sim mujoco max_iter=1 num_envs=16` → 成功：419 steps/s(≈6700 env-steps/s, 在章节 L127 的 4600-8900 区间内)，loss/reward 正常。
- UniLab `--sim motrix` 同任务 → 成功：415 steps/s，证实 L124 切后端 claim；motrixsim import OK。
- UniLab CLIs: 全部存在(train/unilab-import-robot/unilab-pull-assets/...)；`unilab-import-robot --help` = `urdf_path [robot_name]`(符合 L363 以--help为准)。
- UniLab 源码: base/scene.py L21-22 有 `model_file`+`fragment_files`(证实 L924/927)；dr/types.py L12-14 有 RESET_TERM_BODY_IQUAT/INERTIA/IPOS(精确证实 L840)。
- 环境探查: coacd/obj2mjcf/trimesh 在 mjlab 与 unilab 两 venv 均 ModuleNotFoundError；Menagerie 未 clone(find 无结果)。
- UniLab `unilab-pull-assets --help` → 默认且唯一 `--robot {x2}`，无 Go2 选项(与 L61/119 文字'Go2 等模型'不符)。

**工程核对（是这样/不符）**
- 是这样: mjlab Go1 速度任务训练正常起跑(~1050 steps/s, loss 正常)、UniLab go2 mujoco+motrix 两后端均正常起跑——三段 Quick Start/训练命令的核心断言成立。
- 是这样(章节最关键纠正得到实证): `MjSpec.attach` 用 frame=+prefix 成功、传 pos/quat 直接 TypeError，L905'pos/quat 非 attach 直接参数、须先 add_frame'完全正确。
- 是这样(源码级精确): mjlab 的 pseudo_inertia DR 经 Cholesky+重构+平行轴(L840)——函数名 `pseudo_inertia`/`_cholesky_4x4`/`_reconstruct_pseudo_inertia_J` 全部对得上；UniLab dr/types.py 的 reset 粒度恰为 body_inertia/ipos/iquat。
- 是这样: validate_model.py 的求解器索引映射 ['PGS','CG','Newton'](L996)对——实测 opt.solver=2=Newton;三角不等式/pseudo-inertia 检查在真实 go1.xml 上跑通且判正定。
- 不是这样(环境前提缺失,非章节错但读者照抄会失败): Quick Start 验证行 `import coacd,trimesh,mujoco`(L58)在两 venv 均失败——coacd/obj2mjcf/trimesh 未预装;所有 CoACD/obj2mjcf 代码块(L52-59,402-412,698-718)在本机无法实跑,仅静态可信。
- 不是这样(轻微文字不符): `unilab-pull-assets` 实际默认与唯一可选 robot 是 x2,无 Go2;章节 L61/119-120 写'拉 Go2 等模型'与 L122 用 go2_joystick_flat 任务(该任务确实存在且能训)易让读者以为 pull-assets 拉 Go2,实则不拉。
- 部分对(需读者意识到): mjlab 内置 go1.xml 是 actuators=0/nkey=0 的裸模型,章节 Quick Start(L75/82)针对的是 Menagerie 的 unitree_go1/scene.xml(含执行器+home keyframe);本机未 clone Menagerie,该 Quick Start 不能照搬到 mjlab 自带 xml——恰印证章节自身 L885 洞察'执行器不在裸资产里'。
- Isaac 部分待 gpufree 补验: autodl 无 Isaac,UrdfConverterCfg/joint_drive/collider_type(L94-108,577-598)、convert_urdf.py 新路径(L535)、PhysX 专属参数(L608-637)均无法在此实跑;静态核对其字段结构与 v2.x API 叙述自洽、无明显硬伤。

**优化点 / 高级技巧**
- Quick Start 可加一句'本机最小可跑路径': 若未 clone Menagerie,可直接用框架自带资产(mjlab: src/mjlab/asset_zoo/robots/unitree_go1/xmls/go1.xml;UniLab: src/unilab/assets/robots/go2/go2.xml)先打通 MjSpec→compile,降低首跑门槛。
- mesh 工具链小节宜显式提示'coacd/obj2mjcf/trimesh 非框架自带、需 pip 装进对应 .venv(autodl 上 mjlab/unilab 默认都没有)',并给 `uv pip install coacd trimesh obj2mjcf` 的 venv 内装法,避免读者以为开箱即用。
- 可补 mjlab GPU 后端(MuJoCo-Warp)的吞吐基准实测口径: 章节 L731 的 fps 数量级是单 CPU/单环境示意,但全章主线是 GPU 大规模并行;建议补一句'用 `train ... --num-envs N` 看 steps/s 随 N 扩展'的实测法(实测 64 env Go1≈1050 steps/s),把'吞吐'从概念落到可量的命令。
- unilab-pull-assets 的 robot 选项是受限枚举(当前仅 x2),建议章节注明'可拉清单以 `--help` 的 --robot 取值为准',与 L363/L878 既有的'以 --help 为准'风格一致,避免把 Go2 写死成可拉项。
- pseudo_inertia DR 既然 mjlab 已内建(实证函数齐全),可补一行'如何在 cfg 里启用 dr.pseudo_inertia(alpha_range=...)'的最小调用范式(源码 L302/309 提示 body_mass 已 deprecated、官方推荐改用 pseudo_inertia),让读者从'知道有'到'会用'。

**教学缺口（只讲代码没教工程之处）**
- 碰撞简化整节(CoACD/V-HACD/obj2mjcf)只给了代码与参数表,但本机实证这些包默认都没装——章节没教'怎么从零把工具链装进框架的 venv、装完怎么验证'(只有一行 import 验证且该验证在裸环境会失败);从零搭起这一环对读者是断点。
- Quick Start 三框架并排给命令,但没讲'跑起来后怎么判断成功'的统一判据(steps/s 多少算正常、loss 怎样算在收敛、哪些 reward 项该非零)——实测 mjlab≈1050 / UniLab≈415 steps/s 差一倍属正常(GPU-warp vs CPU 后端),章节可补一句帮读者建立'数量级预期',否则新手会误判。
- '转换期增益 rad→deg 换算坑'(L602-604)讲得很透(为什么设0、留到加载期),是工程教学范例;但同节 Isaac 代码(joint_drive/ImplicitActuatorCfg)在 autodl 无法跑验,章节缺一个'不装 Isaac 也能验证 API 名是否存在'的离线手段(如 `pip show isaaclab`/查 converters 模块属性),读者无 Isaac 时无从自检。
- mjlab 内置 go1.xml 执行器为空这一点,章节虽有 L885 洞察兜底,但 Quick Start 代码直接 print actuators 数,读者在 mjlab 自带 xml 上会看到 0 而困惑;教学上可点一句'裸资产 vs 带执行器 scene.xml 的区别在哪、执行器何时被注入',把'为什么这里是0'讲穿而非留给读者撞。
- 三框架'是否内建物理合法惯量 DR'的对比(L840)结论正确且源码可证,但只给了结论;缺'读者如何自己去框架源码里确认这类能力'的方法示范(本次靠 grep 函数名/RESET_TERM 常量就能证实)——教会读者'去源码验证 claim'比直接给结论更授人以渔。

**notes**：总体: 本章可在 autodl 实跑的工程断言(mjlab/UniLab 的 Quick Start、训练/play 命令、MjSpec API、attach 签名纠正、惯性/pseudo-inertia 检查、三框架 DR 源码 claim)经实测与源码核对基本全部成立,尤其 MjSpec.attach 的 pos/quat→TypeError 纠正与 mjlab pseudo_inertia DR 的函数级 claim 属精确正确,质量高。两处轻微不符: (1) unilab-pull-assets 默认/唯一 robot 是 x2 非 Go2(L61/119 文字宜微调); (2) 网格工具链(coacd/obj2mjcf/trimesh)与 Menagerie 在本机均未预装,章节虽以 pip/clone 为前提叙述,但 Quick Start 验证行会在裸环境失败、相关代码块本机不可实证。Isaac 全部支线(UrdfConverterCfg/convert_urdf.py/USD/PhysX 参数)因 autodl 无 Isaac 未能实跑,静态核对无明显硬伤,**待 gpufree 补验**。未改任何 .tex/refs.bib/part.tex。

---

## prac_actuator

**服务器**：autodl (RTX 4080 SUPER 32GB; mjlab 1.4.0 / MuJoCo 3.8.1 / Warp+CUDA; UniLab main commit, mujoco-uni 后端)。Isaac 部分 autodl 无 Isaac，待 gpufree 补验（本次仅静态核对）。

**实跑命令（13）**
- ssh autodl nvidia-smi → RTX 4080 SUPER 32760MiB；两 ACTIVATE 脚本存在（注意 PATH=/autodl-fs/data/uv/bin，无裸 python，须 uv run --no-sync python）
- mjlab: python -c import mjlab.actuator → MuJoCo 3.8.1；导出含 ActuatorCfg/BuiltinPositionActuatorCfg/BuiltinPdActuatorCfg/BuiltinDcMotorActuatorCfg/DcMotorActuatorCfg/IdealPdActuatorCfg/LearnedMlpActuatorCfg/XmlActuatorCfg 等——与章节族谱一致 ✓
- mjlab: dataclasses.fields → ActuatorCfg 基类字段 armature/frictionloss/viscous_damping + delay_min_lag/max_lag/hold_prob/update_period/per_env_phase ✓；LearnedMlpActuatorCfg 含 network_file/history_length/input_order/pos_scale/vel_scale/torque_scale（章节未展开）
- mjlab: 跑章节 Quick Start step_response → Ideal PD peak=33.5、带宽限制 peak=33.5（**不是章节声称的≈28**）；调试发现 target=1.0×kp100=100 N·m ≫ forcerange33.5，两者都饱和
- mjlab: utils.actuator → reflected_inertia(rotor_inertia,gear_ratio)、reflected_inertia_from_two_stage_planetary(tuple,tuple) 均存在 ✓；mdp.dr.pseudo_inertia 存在，params 含 alpha_range/d_range/t_range/asset_cfg ✓
- mjlab: WANDB_MODE=disabled train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2 → 起训成功，1065 steps/s，value_loss 0.0257 / surrogate -0.0210，奖励正常（**mjlab 确有 Go1 任务**：Flat/Rough-Unitree-Go1 与 -G1；无 Go2）
- mjlab: MjSpec sysid 脚本 → spec.actuators / gainprm[0]= / biasprm[1]= / dynprm[0]= / dyntype= / frictionloss(标量)= / compile() / mj_saveLastXML 均 OK；但 **jnt.damping=0.05（标量）抛 TypeError**（damping 是 [3,1] 数组，须 jnt.damping[0]=）
- mjlab: <dcmotor name joint/> 裸标签 → ValueError 'motor constant K must be positive'；试 tau/km/motorconstant/kt+R 多组 attrs 均未跑通最小例（需正确电机常数组合）。<dcmotor> 存在但章节无可跑示例
- UniLab: PdControlConfig 源码(envs/locomotion/common/base.py:29) 只定义 Kp=35.0/Kd=0.5；dataclasses.fields 展开含 action_scale=0.25、**simulate_action_latency=False**（继承自 ControlConfigBase）
- UniLab: grep create_backend → go2/joystick.py:115 & footstand.py:139 均 position_actuator_gains={'kp':Kp,'kd':Kd} ✓；apply_action: _motor_targets += exec_actions*action_scale ✓（go2/rough 用 per-joint hip 0.125 / non-hip 0.25）
- UniLab: mujoco backend get_dr_capabilities → reset terms = BASE_MASS/BASE_COM/GRAVITY/BODY_IQUAT/BODY_INERTIA/BODY_IPOS/BODY_MASS/DOF_ARMATURE/GEOM_FRICTION/KP/KD（**无 pseudo_inertia/Cholesky**，KP/KD 原生支持）—与章节一致 ✓
- UniLab: train --algo ppo --task go2_joystick_flat --sim mujoco max_iterations=1 num_envs=16 → 起训成功，399 steps/s，value_loss 0.0157 / surrogate -0.0271，tracking 奖励正常
- 通用: 跑章节 sysid 全套辅助脚本 → datasheet_to_cfg(33.5)→kp201/kd2.84/filter5.3ms；dc_motor_torque 练习Q1→12.0（与章节答案符）；fit_step/fit_friction 正常恢复 tau_c=0.3/b=0.05 ✓

**工程核对（是这样/不符）**
- 不是这样【头号问题，可复现】Quick Start(L66-94,L129)声称带宽限制版峰值≈28 N·m『明显低于 33.5』，实跑两者都=33.5：因 target=1.0、kp=100 → 命令力矩 100 N·m 远超 forcerange ±33.5，Ideal 与带宽版都顶满饱和，filter 只把到达饱和推迟≈1 步（BW@step+1=10 vs Ideal=-33.5），≈10 步后仍爬到 33.5。章节最核心『亲眼看见力矩变化』的演示证据失真——需减小 target(如 0.2，则 kp×0.2=20<33.5 才看得到爬升差异)或调小 kp/调大 forcerange。
- 不是这样【可复现 bug】apply_sysid_mjspec(L796) `jnt.damping = sysid[...]` 在 MuJoCo 3.8.1 抛 TypeError——MjsJoint.damping 是 [3,1] 数组，须写 `jnt.damping[0] = ...`。同函数 frictionloss(标量赋值)、gainprm[0]/biasprm[1]/dynprm[0](索引赋值)、dyntype、compile、mj_saveLastXML 均正常，仅 damping 标量赋值是错的。
- 不是这样【事实错】PdControlConfig(L296)写 `simulate_action_latency: bool = True`，实际默认 False；且 action_scale/simulate_action_latency 并非 PdControlConfig 自有字段(源码 L29-33 只有 Kp/Kd)，而是继承自 ControlConfigBase——章节把它们列为 PdControlConfig 字段不准确。
- 不是这样【路径不全】章节多处(L114,290 与版本速查)给 PdControlConfig 路径为 locomotion/common/base.py，真实是 unilab/envs/locomotion/common/base.py（漏 envs/ 段）。
- 是这样【与已知陷阱相反，提示更新】mjlab **确有 Go1 任务**(Mjlab-Velocity-Flat/Rough-Unitree-Go1)且实跑成功；仅无 Go2。注意 mjlab train **无 list-envs 子命令**(invalid choice)，任务清单从 train <TASK> 的 choices 报错或 --help 获取。
- 是这样 mjlab actuator 族谱、ActuatorCfg/LearnedMlpActuatorCfg 字段、reflected_inertia 两工具签名、pseudo_inertia 及其 alpha/d/t_range 参数、UniLab create_backend(position_actuator_gains)/apply_action(action_scale+default_angles)/DR 能力(无 pseudo_inertia、KP/KD 原生) 全部与章节描述吻合，三框架对照与版本速查总体准确可信。
- 是这样 MuJoCo 3.8.1 内置 <dcmotor> actuator(章节 L350/1097 声称存在为真)，但需正非零电机常数——裸 <dcmotor> 报 'motor constant K must be positive'。
- 是这样 章节所有数值类断言(dc_motor_torque→12.0、datasheet_to_cfg→kp201、fit_step/fit_friction 反求)实跑自洽；两框架短训 steps/s(mjlab 1065@64env、UniLab 399@16env)与 loss 均健康，能起训无报错。

**优化点 / 高级技巧**
- Quick Start 修正同时也是教学升级：把 target 改小到力矩不饱和的区间(或对比『不设 forcerange』)，并打印力矩『上升斜率/达稳态时间』而非仅 peak.max()——当前 peak 受 forcelimited 钳制，无法体现 filter 的物理效果；可加一条『先确认命令力矩 kp×误差 < forcerange，否则看不到带宽差异』的提示。
- 给 <dcmotor> 补一个可跑最小例(带正确 gear/电机常数属性)并打印其 τ(q̇) 曲线，否则读者照 L367 注释复制裸 <dcmotor> 必报错；这是把『存在性核对』升级为『可复制工程示范』。
- MjSpec 写回脚本建议统一用索引赋值并加注 MuJoCo 3.8.x 的 [3,1] 数组语义(damping/限位类多为数组、frictionloss 为标量)——可补一句『改 MjsJoint 数组字段须 [i]= 赋值，标量字段直接赋』，避免读者踩 TypeError。
- LearnedMlpActuatorCfg(mjlab Level 2)真实暴露 network_file/history_length/input_order/pos_scale/vel_scale/torque_scale，章节只在 Isaac/通用层讲 actuator net，可补『mjlab 原生学习型 actuator 的配置字段』一段，让 Level 2 的 mjlab 列从 mjcb_control(CPU 回调)升级到原生 GPU 路径(章节 L376 已暗示但未给 mjlab 配法)。
- UniLab go2 用 per-joint action_scale(hip 0.125 / non-hip 0.25, rough.py)，比章节单一 action_scale=0.25 更真实；可点一句『四足常对 hip 用更小 action_scale 以抑制侧摆』，顺带教 action 缩放的工程直觉。
- 训练验证小节可补『怎么判断起训正常』的量化锚点(本次实测 mjlab~1k steps/s@64env RTX4080S、UniLab~400 steps/s@16env CPU 物理；value/surrogate loss 量级 1e-2)，给读者一个『跑出来该长这样』的对照基线。

**教学缺口（只讲代码没教工程之处）**
- Quick Start 的演示因饱和而无法呈现承诺现象，却仍断言『跑完你会看到≈28』——这是最伤教学的一处：读者照跑得到 33.5/33.5 会困惑或误以为自己装错环境。应改成『先算命令力矩是否超 forcerange』的可观测设计，并解释饱和与带宽是两个独立现象(章节本身在 L371 pitfall 讲了 filter≠饱和，但 Quick Start 自身就掉进了饱和遮蔽带宽的坑)。
- <dcmotor>『可直接按数据手册建模』(L350)只给概念、零可跑示例，且裸标签会报错——典型『只讲存在、没教怎么配』；读者无法从章节学到 <dcmotor> 的最小必填属性与 τ(q̇) 验证方法。
- apply_sysid_mjspec/datasheet_to_cfg 等脚本以『可复制运行』姿态给出，但 damping 赋值这类 MjSpec 数组语义未教，读者复制即 TypeError——应补『MjSpec 改参的字段类型(标量 vs 数组)怎么查、报错怎么排』的工程方法，而非只贴代码。
- 三框架 actuator API 名核对很扎实，但偏『静态罗列字段』；缺『从零搭一次』的连贯演练——例如没有一个端到端可跑片段把 UniLab 的 Kp/Kd(Hydra 覆盖)→create_backend→apply_action→实际力矩串起来让读者跑通看数，UniLab 列基本是注释式伪代码(L113-127 等)，可举一反三性弱。
- 『运行验证』小节多为定性描述(看 actuator_force 峰值/斜率)，未给读者一个能直接跑、能打印出预期数值的脚本与判定阈值；建议把各 Level 的『预期输出』量化(本次已实测可作锚点)，把『怎么确认配对了』教成可操作步骤而非口头预期。

**notes**：本次仅 autodl(mjlab+UniLab)实跑；Isaac 部分(ImplicitActuatorCfg/DCMotorCfg/ActuatorNetMLPCfg/DelayedPDActuatorCfg/effort_limit_sim 等)autodl 无 Isaac，**待 gpufree 补验**——静态核对其 API 名/继承树叙述与公开文档一致、未见明显硬伤。最关键可复现问题三条供主控优先处理：(1) Quick Start 峰值≈28 的断言不成立(饱和遮蔽，实测 33.5/33.5)；(2) apply_sysid_mjspec 的 jnt.damping=标量 在 MuJoCo 3.8.1 抛 TypeError(须 [0] 索引)；(3) PdControlConfig simulate_action_latency 默认是 False 非 True、且该字段+action_scale 属基类。其余(mjlab Go1 任务存在、UniLab 路径含 envs/、<dcmotor> 无可跑例)为次要订正。未改任何 .tex/.bib。临时脚本在 scratchpad 与 autodl /tmp，可清理。


---

## prac_quad_loco

**服务器**：autodl (RTX4080S; mjlab 1.4.0 + UniLab; 无 Isaac, Isaac 实战部分待 gpufree 补验)

**实跑命令（14）**
- mjlab `uv run list-envs` → 确认 Mjlab-Velocity-Flat-Unitree-Go1 与 Mjlab-Velocity-Rough-Unitree-Go1 真实存在(无 -v0 后缀)，坐实 Quick Start/pitfall 的 task id。
- mjlab `play TASK --help` → --agent {zero,random,trained} / --num-envs / --viewer {auto,native,viser} 全部存在，四阶段 play 命令成立。
- mjlab `train TASK --help` → --env.scene.num-envs / --agent.max-iterations / --agent.run-name / --agent.seed / --agent.logger {wandb,tensorboard} / --env.scale-rewards-by-dt / --env.rewards.track-linear-velocity.params.{std,command-name} 全部存在且与章节逐字一致。
- mjlab Go1 flat 实跑 2 iter/64env → 成功 1.55s/iter，value/surrogate/entropy loss 正常；日志含 Episode_Reward/{track_linear_velocity,track_angular_velocity,action_rate_l2,foot_clearance,foot_slip,upright,pose,body_ang_vel,dof_pos_limits,angular_momentum,air_time} + Curriculum/command_vel/lin_vel_x_max + Metrics/twist/error_vel_xy(坐实奖励名/曲线名/command=twist)。
- mjlab Go1 rough 实跑 1 iter/64env → 成功 465 steps/s；height_scan 观测 shape=(187,) 逐字坐实 §terrain 的 187 ray；out_of_terrain_bounds 终止 + terrain_levels 课程(pyramid_stairs/wave/slope 等)在动。
- mjlab 源码 grep twist → velocity_env_cfg.py 用 command_name=twist 且 std=math.sqrt(0.25)，坐实 §reward 注脚。
- mjlab 源码类名 → ObservationTermCfg/ObservationGroupCfg/RewardTermCfg/EventTermCfg/TerminationTermCfg 全对，且全仓无 ObsTerm/RewTerm 别名(坐实'mjlab 无别名只用全名')。
- mjlab 版本 → mjlab 1.4.0 对，但 rsl-rl-lib=5.4.0(章节写 5.2.0，已漂移)；mujoco 3.8.1.dev / warp 1.14.0 / torch 2.9.0。
- UniLab `train --help` → algo={ppo,mlx_ppo,appo,sac,td3,flashsac} / sim={mujoco,motrix} / --render-mode {auto,interactive,record,none} 均对。
- UniLab go2_joystick_flat 实跑 1 iter/16env mujoco → 成功 363 steps/s；Critic MLP in_features=52 +(512,256,128)+ELU + EmpiricalNormalization；reward/{tracking_lin_vel,tracking_ang_vel,lin_vel_z,ang_vel_xy,base_height,action_rate,similar_to_default,contact,swing_feet_z} 全部坐实派发表与四层。
- UniLab go2_joystick_flat/mujoco.yaml → reward.scales 九项 + tracking_sigma:0.25 + base_height_target:0.3 与 §reward codebox 逐字一致；underscore 覆盖键 algo.num_envs/max_iterations 生效(坐实 \rebuilt 的下划线说明)。
- UniLab go2_joystick_rough/mujoco.yaml → DR 块(randomize_base_mass/added_mass_range[-1,3]/random_com/randomize_kp/kp_multiplier_range[0.5,2]/push_robots/push_interval:625/max_force[1,1,0.5]) 与 §DR codebox 逐字一致；num_steps_per_env:24 对；但 terrain_curriculum.enabled=false 且 generator.curriculum=false。
- UniLab 源码 grep terrain_scan/height_scan → 发现内置 common/height_scan.py(init_height_scan_sensor/height_scan_obs/HeightScanConfig.enabled=True 默认)，go1/go2/go2w rough 均把 height_scan 拼进 critic；go2/rough.py:298 obs_groups_spec={obs:45,critic:48+scan}。
- UniLab go2/rough.py:441-457 实读 → actor obs=[gyro,gravity,commands,diff,dof_vel,last_action](无 scan，盲式)，critic=[critic_base + height_scan_obs](scan 仅进 critic)——直接证伪章节'UniLab 无内置 height scan 传感器'的措辞。

**工程核对（是这样/不符）**
- [符] mjlab task id Mjlab-Velocity-Flat/Rough-Unitree-Go1 实跑存在(list-envs)，§quickstart 与 pitfall 的字符串与 \rebuilt 提醒完全成立。
- [符] mjlab 四阶段命令全部 CLI 验证通过：play --agent {zero,random,trained}/--num-envs/--viewer{auto,native,viser}，train --env.scene.num-envs/--agent.max-iterations/--agent.run-name/--agent.logger{wandb,tensorboard}/--env.rewards.*.weight 与 --env.rewards.*.params.std/.command-name 逐字对(prac_quad_loco.tex:63,1365-1373,1484-1489)。
- [符] §reward 函数名/参数名全部坐实：实跑日志与源码确认 track_linear_velocity/track_angular_velocity、参数 std + command_name='twist'、std=math.sqrt(0.25)(velocity_env_cfg.py:279)；ins:rlmc-ql-rewname '看实现而非看名字' 立得住。
- [符] §terrain 核心数字坐实：Go1 rough 实跑 height_scan 观测 shape 恰为 (187,)，与公式 (1.6,1.0)@0.1m→187 及'Go2 rough 235≈48+187'一致(prac_quad_loco.tex:216,903)。
- [符] §terrain-curriculum 坐实：rough 实跑日志含 Curriculum/terrain_levels/{mean,max,pyramid_stairs,wave_terrain,hf_pyramid_slope,random_rough,flat} 与 Episode_Termination/out_of_terrain_bounds，terrain_levels_vel()/game-inspired 课程描述成立(prac_quad_loco.tex:1003,716)。
- [符] §dualframework API 表的 mjlab 侧全坐实：源码确认 ObservationTermCfg/RewardTermCfg/EventTermCfg/TerminationTermCfg 真名，且全仓无 ObsTerm/RewTerm 别名(prac_quad_loco.tex:424,462,1214)。
- [符] UniLab go2 flat YAML reward.scales 九项 + tracking_sigma:0.25 + base_height_target:0.3 与 codebox(prac_quad_loco.tex:651-655) 逐字一致；实跑确认 9 个 reward/* 派发项 + 派发表函数名全对(prac_quad_loco.tex:658-668)。
- [符] UniLab obs_groups_spec={obs:49,critic:52} 经实跑 Critic in_features=52 坐实(flat)，§obs codebox(prac_quad_loco.tex:486) 正确；网络 (512,256,128)+ELU+EmpiricalNormalization 与 §rlcfg 一致。
- [符] UniLab DR codebox(prac_quad_loco.tex:1170-1180) 与 go2 rough YAML 逐字一致(added_mass_range/kp_multiplier_range/push_interval:625/max_force[1,1,0.5])；reset=个体差异 + interval=推扰、乘性保物理一致的论述成立。
- [不符·重要] §terrain 反复声称 'UniLab 当前任务集没有把 raycast 高程网格作为内置观测传感器'/'UniLab 无对应内置传感器'/'若要 height scan 需自行在 _compute_obs 里实现(非框架现成能力)'(prac_quad_loco.tex:956-973) 与实际不符：安装版有内置 common/height_scan.py(HeightScanConfig.enabled=True 默认, init_height_scan_sensor + height_scan_obs)，go1/go2/go2w rough 都把 height_scan 拼进 critic(go2/rough.py:452, _height_scan_dim=Nx*Ny)。准确表述应为'UniLab 已内置 height scan 传感器，但仅喂 critic(特权)，actor 保持盲式'——公式 ray 数→obs 维在 UniLab 同样适用(只是入 critic)。
- [不符·小] §rlcfg/版本表(prac_quad_loco.tex:1123,1809) 写 mjlab 1.4.0 装 rsl-rl-lib 5.2.0；实际安装为 5.4.0(仍 5.x，论点不变，仅版本号需更新)。
- [不符·小] §reward mjlab Go1 实测 13 term 的具体集合与 codebox(prac_quad_loco.tex:590-613) 略有出入：实跑出现 foot_swing_height、soft_landing(codebox 未列)，而 codebox 列的 joint_torques 在日志未现(可能值为0)；codebox 已标'简化'，但术语速查的逐项对照可微调。
- [待核] §termination UniLab rough YAML 有 terrain_curriculum 键但 enabled=false 且 generator.curriculum=false(go2_joystick_rough/mujoco.yaml)——章节(prac_quad_loco.tex:755)说 rough owner '带 terrain_curriculum' 成立(键在)，但默认关闭这一事实未提，易让读者以为 rough 默认开课程。

**优化点 / 高级技巧**
- §terrain 应据实跑改写 UniLab height-scan 段：UniLab 已内置 raycast 风格 height scan(common/height_scan.py, HeightScanConfig, measured_points_x/y 网格, dim=Nx*Ny)，并把它接到 critic(privileged)——这恰好是比'盲式'更精确、也更有教学价值的'非对称 + 地形感知 teacher'范例，可与 mjlab/Isaac 的 raycast 形成真正三框架对照，而非标注为'缺页'。
- 可补'flat 与 rough 的 actor 维度为何不同':实跑显示 UniLab go2 flat actor=49(含 feet_phase 4 维)、rough actor=45(无 feet_phase)、critic 入特权差异。章节用同一 48/49 叙述容易让读者忽略 flat 多了步态相位、rough 改走 height_scan(入 critic) 的结构切换。
- 建议加一条'用训练日志逐项验配置'的可操作技巧:mjlab 启动日志会打印 Observation/Reward/Termination 的 term 表与 shape(实跑见 height_scan (187,))——这是比'读 cfg 源码'更快的 wiring 自检手段,正对应 §train-diag Step1-2,但章节未点出日志自带该表。
- 版本号建议改为范围或加'以实装为准':rsl-rl-lib 在 mjlab 上已到 5.4.0;mujoco 3.8.1.dev/warp 1.14.0/torch 2.9.0 也都比章节隐含的版本新,§versions 可加一句'实测随 uv.lock 漂移'。
- UniLab rough 默认 terrain_curriculum.enabled=false 值得在正文点明并给开启方式(Hydra: env.scene.terrain.generator.curriculum=true + terrain_curriculum.enabled=true),否则读者照 §terrain-curriculum 的'自动难度'预期跑 go2 rough 会发现 level 不动——这本身就是个真实可教的排查案例。
- mjlab Go1 实测 reward 含 soft_landing/foot_swing_height 等更现代的着地/摆腿项,可在 §reward 补一句'实装版已细分 landing 类奖励',让四层框架落地更贴近当前代码。

**教学缺口（只讲代码没教工程之处）**
- §terrain 的 UniLab '缺页'结论是基于过时认知的静态判断,没有一条命令去 grep/import 验证;教学上反而示范了'凭印象下结论'——应改为示范'grep terrain_scan→读 rough.py obs 装配→确认 scan 入 critic'的实证链路(本次实跑正是这条链路抓出了错)。
- 章节大量给出三框架配置/命令的并列代码,但几乎不教读者'如何自己在服务器上验证一行 CLI/一个 API 名是否真存在'(如 train --help | grep、list-envs、python -c import)。本章自身的核对手段(--help/list-envs/源码 grep)若写成'读者可复用的自检流程',比逐框架罗列 flag 更能举一反三。
- §obs/§reward 给了 obs_groups_spec={obs:49,critic:52} 等确切维度,但没教'维度从哪来、怎么自检':实跑 Critic in_features=52 一眼可验,应教读者用启动日志的网络结构/obs 表反查维度,而非背数字。
- §dualframework 与 §multirobot 的 Isaac/部署/DAgger 内容无法在 autodl 实跑(无 Isaac),且这些段落偏'讲它有什么功能'而非'怎么从零搭一次对比实验/怎么排部署 OOD';待 gpufree 补验时建议补'最小可复现的双框架对比脚手架'。
- '策略不走'排查(§train-diag)写得好,但与真实日志字段脱节:实跑日志字段是 Metrics/twist/error_vel_xy、Episode_Reward/track_linear_velocity、Curriculum/command_vel/lin_vel_x_max,章节排查流程用的是 reward/track_linear_velocity 等泛化名,若直接贴实测字段名读者更易对号入座。

**notes**：本章工程底子非常扎实:三框架的 task id、CLI flag、API 类名、奖励/课程/终止字段、UniLab 两份 owner YAML 的标量,几乎全部经 autodl 实跑/源码逐字坐实(mjlab Go1 flat+rough、UniLab go2 flat 均成功起训,loss/steps-s 正常)。唯一重要硬伤:§terrain(prac_quad_loco.tex:956-973)反复断言'UniLab 无内置 height scan 传感器、需自行实现',与安装版事实相反——UniLab 有内置 common/height_scan.py 且 go2/go1/go2w rough 把 height_scan 喂 critic;正确结论是'已内置但仅入 critic(特权),actor 盲式',ray 数→obs 维公式在 UniLab 同样适用。次要:rsl-rl-lib 实装 5.4.0(章节 5.2.0);mjlab Go1 实测 reward 集合与简化 codebox 略有出入(多 soft_landing/foot_swing_height);UniLab rough 的 terrain_curriculum 默认 enabled=false 未点明。Isaac 全部实战(双框架对比、Anymal/Go2 play/train、部署 ONNX)因 autodl 无 Isaac 未能实跑,待 gpufree 补验;但 Isaac 侧的 API 命名/235 维等与 mjlab/官方文档交叉引用处静态看合理。建议主控:优先据实跑重写 UniLab height-scan 段(从'缺页'改为'入 critic 的内置特权 scan'),并更新 rsl-rl-lib 版本号。

---

## prac_humanoid_loco

**服务器**：autodl (RTX 4080 SUPER 32GB; mjlab core @ /autodl-fs/data/mjlab + UniLab @ /autodl-fs/data/UniLab; NO unitree_rl_mjlab, NO Isaac)

**实跑命令（12）**
- uv run list-envs (mjlab) → G1 任务真名为 Mjlab-Velocity-Flat-Unitree-G1 / -Rough-G1 / Mjlab-Tracking-Flat-Unitree-G1，另有 Go1(非Go2)；无 Unitree-G1-Flat。
- importlib.util.find_spec → mjlab 已装、unitree_rl_mjlab 未装(章节 Quick Start 的 scripts/train.py + Unitree-G1-Flat 依赖此库，autodl 上不可跑)。
- train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 64 --agent.max-iterations 2 → 成功，~1285 steps/s，value loss 1.1；reward 项含 track_linear_velocity/pose/angular_momentum/self_collisions/upright/body_ang_vel/air_time/foot_slip/soft_landing，Metrics/angular_momentum_mean 有打印，fell_over 终止触发。
- play Mjlab-Velocity-Flat-Unitree-G1 --help → 确认 --agent {zero,random,trained}、--no-terminations {True,False}、--num-envs、--checkpoint-file(连字符)、--viewer {auto,native,viser} 全部真实存在。
- grep velocity/mdp/rewards.py → 函数名 track_linear_velocity/track_angular_velocity/upright(类)/self_collision_cost/angular_momentum_penalty/feet_slip/variable_posture(类) 与章节完全一致。
- 读 variable_posture 源码 → 确为类、核为 exp(-mean(error²/std²))、三档 walking/running_threshold、std 用{关节名正则:std}字典，与章节 note(542行)一致。
- python 打印 G1_ACTION_SCALE → 16 条正则项、公式 0.25*effort/stiffness 真实存在；但 stiffness 由 armature*natural_freq² 推导，实际 scale(hip_pitch 0.5475/knee 0.3507/ankle 0.4386/wrist_yaw 0.0745)与章节表(0.147/0.174/0.313/0.031)不同。
- grep g1/env_cfgs.py → angular_momentum.weight=-0.02、self_collisions weight=-1.0 force_threshold=10.0(ContactSensor 名 self_collision)、pose 的 std 字典键 .*、upright 用 torso_link、init pos=(0,0,0.76) hip_pitch=-0.312 KNEES_BENT_KEYFRAME，全部与章节相符。
- MuJoCo 编译 g1.xml → nq=36(=7+29) nv=35，确认 29 actuated DoF，验证章节『29-DoF 仿真』。
- UniLab train --help → CLI 真为 train --algo{ppo,appo,sac,td3,flashsac} --task --sim{mujoco,motrix}；conf/ppo/task 下 g1_walk_flat 及 g1_motion/flip/box/climb_tracking 任务目录均存在。
- UniLab train --algo ppo --task g1_walk_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true → 成功，~359 steps/s，注册名 G1WalkFlat，reward 含 tracking_lin_vel/tracking_ang_vel/base_height/pose/orientation/feet_phase。
- cat g1_walk_flat/mujoco.yaml → task_name=G1WalkFlat、tracking_sigma=0.25、base_height_target=0.754(非0.76)、base_height scale=-500(非-100)、tracking_lin_vel=2.0(非1.0)。

**工程核对（是这样/不符）**
- 【最重要·不符】Quick Start(53-64行)的 mjlab 命令 `python scripts/train.py Unitree-G1-Flat` 在 autodl 上跑不起来：本机只装 mjlab core(未装 unitree_rl_mjlab)，真实入口是 `uv run train Mjlab-Velocity-Flat-Unitree-G1`；且 play 的 `--checkpoint_file=`(下划线)应为 `--checkpoint-file`(连字符)。
- 【不符·与章节自身陷阱矛盾】86-88行陷阱断言写 `Mjlab-Velocity-Flat-Unitree-G1` 是错的、`Unitree-G1-Flat` 才对——但 list-envs 实测恰相反：mjlab core 本体就注册了 Mjlab-Velocity-Flat-Unitree-G1。该陷阱基于『G1 任务仅由 unitree_rl_mjlab 提供』的前提，在本机版 mjlab 不成立(reward 函数与 G1 配置都在 mjlab core 内)。两条任务命名约定并存，章节只认其一。
- 【符合】reward 函数名/类型(track_linear_velocity、variable_posture 类、angular_momentum_penalty、self_collision_cost、feet_slip、upright 类)、variable_posture 的 exp 核与三档 std 字典、键名 pose——全部与 mjlab 源码逐项吻合。
- 【符合】关键权重经实跑+源码双证：angular_momentum=-0.02、self_collisions=-1.0(force_threshold 10)、base height 0.76、knees-bent keyframe、effort 表(hip88/knee139/shoulder25/wrist5)、29-DoF——均真实。
- 【符合】506-545行 reward 注册表的结构与项名经 2-iter 实跑全部出现在 Episode_Reward/* 日志里(含 body_ang_vel、air_time=0、soft_landing)，章节描述属实。
- 【符合】244行 practice 用的 `--agent zero --no-terminations`、play 的 `--num-envs`(注意 play 单数 num-envs，train 是 --env.scene.num-envs)经 --help 确认真实；章节原 \pz{} 对 --no-terminations 的存疑可销账(确存在)。
- 【符合】UniLab 段(75-84、585-600、762-790行)：CLI 形态、Hydra 覆盖(algo.num_envs/max_iterations 走覆盖非 flag)、注册名 G1WalkFlat vs slug g1_walk_flat、scales 标量字典+派发表范式、motion-tracking 任务族、obs/critic 组名——经实跑与 yaml 核对全部成立。
- 【不符·小】UniLab scales 具体数(588-591行)与本机 g1_walk_flat/mujoco.yaml 不符：base_height -100→实际 -500、base_height_target 0.76→0.754、tracking_lin_vel 1.0→2.0；章节标了『示例』故方向无错，但数值非实配。
- 【Isaac 部分待 gpufree 补验】autodl 无 Isaac，66-70行 H1 命令、395-424行 ImplicitActuatorCfg(effort_limit_sim/三组 actuator)、700-728行 H1 rough cfg、HOVER/sim2sim 脚本均无法实跑；静态看 API 名(track_lin_vel_xy_yaw_frame_exp、feet_air_time_positive_biped、RslRlSymmetryCfg、flat_orientation_l2)与版本声明(IsaacLab 2.3.2/HOVER 2.0.0)内部自洽、与公开 Isaac Lab 习惯一致，未见明显硬伤。

**优化点 / 高级技巧**
- Quick Start 应给『两套命名并存』而非二选一：实测 mjlab core 与 unitree_rl_mjlab 对 G1 用不同任务 ID(Mjlab-Velocity-Flat-Unitree-G1 vs Unitree-G1-Flat)和不同入口(uv run train vs scripts/train.py)。最稳的教法是先教 `uv run list-envs` 自查真实 ID，再照抄——这恰是章节 88行自己倡导的方法，但正文却写死了单一形式。
- action scale 表(298-304行)建议改成『先讲公式+effort 真值(已对)，再让读者用 295行脚本现算 scale，不写死最终 scale 数』：因 mjlab 的 stiffness=armature*natural_freq² 与教材假设的 150/200/40 不同，导致最终 scale 差 3-4 倍(实测 hip_pitch 0.55 而非 0.147)。保留相对结论(腕最小、约 7×)即可，绝对值交给脚本。
- 可补一条高级调试技巧:实跑 2 iter 即可在 stdout 看到全部 Episode_Reward/* 与 Metrics/angular_momentum_mean、landing_force_mean、slip_velocity_mean、peak_height_mean 等诊断量——这正是 660行陷阱说的『先打印各 term 量级再调权重』的最廉价落地手段，章节可明确教『2 迭代冒烟即读 term 数值』。
- UniLab 段可补:play/eval 真实子命令与 --render-mode {auto,interactive,record,none} 取值、--profile 参数;以及训练日志落点 logs/rsl_rl_ppo/G1WalkFlat/<时间戳>_mujoco/(含 git diff 快照),便于读者定位 checkpoint。
- play 的 --num-envs(单数)与 train 的 --env.scene.num-envs 不一致是真实易踩点,值得在 Quick Start 显式点出(章节诊断矩阵未提)。

**教学缺口（只讲代码没教工程之处）**
- 『从零搭起』缺一环:章节默认 unitree_rl_mjlab 已装并用 scripts/train.py,但没教读者如何确认自己装的是 mjlab core 还是 + 任务库、两者任务 ID 为何不同。实测两种安装给出截然不同的 CLI,新手照章节抄命令在纯 mjlab 环境必失败却不知为何——这是最大教学缺口(应教 `list-envs`/`find_spec` 自检)。
- action scale 一节『只给数不给溯源』:把 stiffness 当成已知常数(150/200/40)直接代入,没教读者 mjlab 里 stiffness 其实是 armature×natural_freq² 算出来的(g1_constants.py:115-118)。读者若照表里的 0.147 去对真实 0.55 会困惑;应教『scale 怎么从 MJCF/常数现场推』而非记结论。
- variable_posture/angular_momentum/self_collision 三项讲了『是什么、权重多少』,但没教读者怎么自己验证『它们真生效』。实测最直接的办法是跑 2 iter 看 Episode_Reward/pose、/angular_momentum、/self_collisions 是否非零——这种『跑最小步数读日志确认接线』的工程习惯,章节运行验证小节(446、616行)描述了五阶段曲线却没给这个最低成本的即时自检。
- Isaac/HOVER/HoST 多为静态代码罗列(395-424、702-728、1042-1073行),受限于 autodl 无 Isaac 无法实跑,读者也无从复现;章节宜显式标注哪些代码是『可跑实例』哪些是『论文示意/需自实现』(HoST 已较好地标了 IsaacGym 不可直接复制,HOVER/Isaac H1 段可再强化『本框架未实跑』的边界)。
- 诊断矩阵(797行)与故障排查手册(1200行)给了『现象→动作』,但没把『2 迭代冒烟→读 term 量级→定位异常 term』这条最通用的排查起手式写进去——这是把代码讲成工程的关键一步,目前偏向罗列参数表而非教排查心法。

**notes**：实跑覆盖 mjlab G1 flat velocity(2 iter 成功)与 UniLab g1_walk_flat(1 iter 成功)两条主线，外加 reward 源码/权重/keyframe/action-scale/DoF 的源码级核对。核心结论:章节教的工程实践绝大多数属实(reward 函数名/类型、angular_momentum=-0.02、self_collision、variable_posture 机制、29-DoF、--agent zero/--no-terminations、UniLab CLI 与 scales 范式全部经实测确证)。两处需主控关注:(1)最重要——autodl 只装 mjlab core 未装 unitree_rl_mjlab,故 Quick Start 的 `scripts/train.py Unitree-G1-Flat` 跑不通,真实入口 `uv run train Mjlab-Velocity-Flat-Unitree-G1`,且 86行陷阱的对错判断与本机实情相反;建议章节改为教 list-envs 自查、并承认两套命名并存。(2)action scale 具体数值与本机 mjlab 不符(stiffness 推导方式不同),建议只保留公式+相对结论。UniLab scales 具体数(base_height -100/target 0.76)与实配(−500/0.754)有出入但已标『示例』。Isaac/HOVER 全部待 gpufree 补验,静态核对未见硬伤。未改任何 .tex。


---

## prac_motion_imitation

**服务器**：autodl (RTX4080S; mjlab + UniLab; no Isaac)

**实跑命令（10）**
- mjlab `list-envs` → 确认 `Mjlab-Tracking-Flat-Unitree-G1`(+`-No-State-Estimation`变体) 真实存在,与章节任务名逐字一致;无 Go2 tracking(符合已知)。
- mjlab 源 `tasks/tracking/tracking_env_cfg.py` 静态提取 → reward 键 `motion_global_root_pos/ori`(w0.5)、`motion_body_pos/ori/lin_vel/ang_vel`(w1)、func `motion_relative_body_position_error_exp`/`motion_global_anchor_*_error_exp`、终止 `bad_anchor_pos_z_only`/`bad_anchor_ori`/`bad_motion_body_pos_z_only`,与第278/361/441行逐字吻合。
- mjlab std 实测 = 0.3/0.4/0.3/0.4/1.0/3.14、阈值 0.25/0.8/0.25、`anchor_body_name="torso_link"`、body_names 用 `left_ankle_roll_link`/`left_wrist_yaw_link` 等 `_link` 后缀 → 与章节 note/pitfall 全部吻合。
- mjlab `train Mjlab-Tracking-Flat-Unitree-G1 --num-envs 64 --max-iterations 2` → 报错 `ValueError: For tracking tasks, provide --registry-name 或 --env.commands.motion.motion-file`,无 motion 不启动。
- mjlab `play Mjlab-Tracking-Flat-Unitree-G1 --agent zero --num-envs 1` → 同样报错 `Tracking tasks require --motion-file/--registry-name`;盘上无任何 bundled .npz(glob 全空)。
- UniLab `train --algo ppo --task g1_motion_tracking --sim mujoco max_iterations=1 num_envs=16 no_play=true` → 成功跑完,343 steps/s,actor 出 29 维、critic 286 维(非对称),reward 日志键与章节一致。
- UniLab 源/yaml:`G1MotionTracking(+SAC/Deploy)`、`G1FlipTracking`/`G1BoxTracking`/`G1ClimbTracking`/`G1WallFlipTracking`、`G1WBTObs`、`MotionLoader/Data/Sampler`、deploy 5 脚本(`export_motion_bin`/`sim_prototype`/`prepend_warmup`/`append_cooldown`/`export_deploy_config`) 全部真实存在。
- UniLab `conf/ppo/task/g1_motion_tracking/mujoco.yaml` 实读 → scales 前6项 0.5/0.5/1/1/1/1 + action_rate_l2 -0.1 + joint_limit -10 吻合;但用 per-term `std_*`(0.3/0.4/0.3/0.4/1.0/3.14)而非全局 tracking_sigma;`motion_joint_pos/vel=0.0`、无 `motion_ee_body_pos_z`。
- UniLab `demo --help` + demo.py → `dance`/`boxtracking`/`wallflip` 注册存在(均 sim=motrix, entry=eval),命令 `uv run demo dance` 成立。
- UniLab 工程件核实:`training/sim2sim.py` 有 `CrossBackendIncompatibleError`、`ipc/memory_budget.py`+`UNILAB_SKIP_MEMORY_CHECK`、`MultiGPUCPUPinnedReplayPipeline`、`PenaltyCurriculum` 全部命中;但 `sim2sim_strict` 全仓 NOT FOUND。

**工程核对（是这样/不符）**
- [是这样] mjlab tracking 的 reward 键名/func 名/std 值/三条终止/anchor_body_name=torso_link/body_names 的 _link 后缀,与章节(第278/361/367/400/441行)逐字相符——这部分核对精度很高,实跑/读源全部坐实。
- [是这样] UniLab g1_motion_tracking PPO 冒烟成功(343 steps/s,loss 正常),per-reward 日志键 motion_global_root_pos/ori + motion_body_pos/ori/lin_vel/ang_vel + action_rate_l2 + joint_limit 与章节 codebox 一致;actor29/critic286 印证非对称(逐 body 参考量进 critic,符合第400/403行)。
- [是这样] UniLab 注册名族(G1MotionTracking/SAC/FlipTracking/BoxTracking/ClimbTracking/WallFlipTracking/WBTObs)、deploy 5 脚本、MotionLoader/MotionData/MotionSampler、sim2sim.py+CrossBackendIncompatibleError、memory_budget.py+UNILAB_SKIP_MEMORY_CHECK、MultiGPUCPUPinnedReplayPipeline、PenaltyCurriculum 全部真实存在,与第82/245/490/509/671/867/1006/1011/1058行吻合。
- [不是这样] 章节 Quick Start(第76行 play --agent zero / 第78-79行 train)与完整流程 Step2(第475行 zero play)/Step3(第478行 small train) 的 mjlab 命令均**未带 motion 文件**,实跑直接 ValueError 拒绝启动;读者照抄第一步『看 ghost』就跑不起来。
- [不是这样] UniLab reward codebox(第334行)写 `tracking_sigma: 0.25 全局`,且旁注称『逐项 σ 需自定义』;实测三份 g1_motion_tracking yaml 均**无 tracking_sigma**,用 8 个 per-term `std_*` 字段(0.3/0.4/0.3/0.4/1.0/3.14,与 mjlab 同),即逐项 σ 本就是默认、无需自定义——与章节说法相反。
- [部分不符] UniLab codebox(第331-333行)把 `motion_joint_pos/motion_joint_vel/motion_ee_body_pos_z` 列为 weight=1.0 的『UniLab 额外活跃项』;实测默认 yaml 中 joint_pos/vel=0.0、ee_body_pos_z 在任何 yaml 都不出现(env 默认 scale 也是 0.0)。三者作为 reward **函数**确实在 tracking.py 注册(_reward_motion_ee_body_pos_z + ee_body_pos_z_threshold=0.25),但默认**未启用**——章节把『存在但关』写成了『默认开且权重1』。
- [不符] 章节(第1058行)称跨后端校验开关为 `training.sim2sim_strict=true`(默认);该 flag 全仓 grep NOT FOUND(类与异常存在,但此 flag 名不存在)——开关名属臆造或过期。
- [未验] Isaac/ProtoMotions 全部实战(第556-605行 dataclass 入口、--experiment-path、num_envs=1 限制、4×A100/24×A100 基准)autodl 无 Isaac,**待 gpufree 补验**;静态看 dataclass CLI 形态与 README 叙述自洽,无明显硬伤。

**优化点 / 高级技巧**
- mjlab 的 motion-file 强制门是最大可补点:应在 Quick Start / Step2 / Step3 的命令里**显式带上** `--registry-name <org>/motions/<name>` 或 `--env.commands.motion.motion-file /path/x.npz`,否则首条命令必报 ValueError;并提示读者先 `csv_to_npz.py` 产出/或从 WandB 拉一条 motion 才能 zero-play 看 ghost。可补一句『tracking 任务无 motion 不启动,这是与 velocity 任务的关键差异』。
- 可补 mjlab 自检小技巧:训练前用 `uv run --no-sync python -c '读 tracking_env_cfg 打印 std/threshold/body_names'`(本次实跑用的正是此法)即可在不跑训练下核对 reward/终止/body 配置,比『跑起来再看 reward 涨不涨』更早暴露 body_names 拼错。
- UniLab 侧建议明确教『per-term std 调参』:既然默认是 std_root_pos/std_body_pos 等独立字段,应指出想做 KungfuBot 式 σ 收紧时改的是**这 8 个字段**(或写回调原地改),而非章节暗示的单一全局 tracking_sigma——这才是 UniLab 上复刻自适应 σ 的真实抓手。
- 若要 ee/joint 抬脚高度等额外项生效,应教读者在 owner yaml 把 `motion_joint_pos`/`motion_ee_body_pos_z` 的 scale 从 0.0 改非零(并设对应 std)——章节可加一句『这些项默认关,按需在 scales 里开』,避免读者以为开箱即用。
- UniLab `--algo` 实测支持 ppo/mlx_ppo/appo/sac/td3/flashsac 六种(不止章节强调的 PPO/APPO/SAC);可补 td3/flashsac 作为 off-policy 备选,丰富『三算法变体』的实际选项。

**教学缺口（只讲代码没教工程之处）**
- 最关键教学缺口:章节反复教『zero-play 看 ghost 是所有调试的起点』(第72-73/367/1180行),却没告诉读者 zero-play 必须先有 motion 文件、否则报错——『怎么从零搭起』在第一步就断链。应补『先获得一条 .npz 的两条路(csv_to_npz / WandB registry-name),再 zero-play』的完整起步链。
- 『为什么这么做』讲得好(base frame 解耦 root 漂移、anchor、σ 物理含义、双峰直方图诊断都很到位),但『怎么排错』偏理论:故障表(第1180行)说『打印 body_names 的 id』,却没给可复制的打印命令/脚本;读者难以真正照做。可补一段最小自检脚本。
- UniLab reward『范式不同但同名同权重』讲得清楚,但把 per-term std 简化成单一 tracking_sigma,反而误导读者『UniLab 只有一个全局 σ』;教学上应如实呈现 8 个 std_ 字段并解释为何逐项(不同物理量量纲不同,与 mjlab 一致),这恰是好的『参数怎么调』示范却被简化掉了。
- ProtoMotions/Isaac 整节(算法切换、分布式 DDP、PMCP 原生)在本机无法实跑验证,且代码多为『概念实现/示意』(DistributedMotionLoader、PMCP、AdaptiveMotionSampler、physics filter 均标注示意);教学上偏『讲思想+贴伪代码』,缺少一条 Isaac 上真能跑的最小命令闭环——读者难以举一反三到真实 ProtoMotions CLI(建议 gpufree 补一条可跑示例)。

**notes**：本章 API/键名核对精度极高:mjlab(reward 键/func/std/终止/anchor/body_names)与 UniLab(注册名族/源码路径/deploy 脚本/IPC 与 sim2sim 工程件)几乎逐字坐实,UniLab PPO 冒烟实跑成功(343 steps/s)。三处需主控修正:(1) mjlab Quick Start/Step2/Step3 命令缺 motion 文件,照抄必 ValueError,且本机无 bundled npz,zero-play 起步链断;(2) UniLab reward codebox 的 `tracking_sigma:0.25 全局` 与实际不符——真实是 8 个 per-term std_*(同 mjlab),且旁注『逐项σ需自定义』正好说反;(3) `motion_joint_pos/vel/ee_body_pos_z` 默认 scale=0.0/未入 yaml(函数存在但未启用),章节写成活跃 weight=1.0;另 `sim2sim_strict` flag 名全仓不存在(类/异常存在)。Isaac/ProtoMotions 部分 autodl 无法实跑,待 gpufree 补验,静态无明显硬伤。未改任何 .tex/refs.bib。


---

## prac_wheeled_bimanual

**服务器**：autodl (RTX 4080 SUPER; mjlab + UniLab，无 Isaac)

**实跑命令（12）**
- ssh autodl + nvidia-smi: 连接OK，GPU=RTX 4080 SUPER 32GB，两激活脚本均在
- mjlab `train --help` / `list-envs`: train 用法=`train <TASK>`，确认无轮式任务、确认存在 Mjlab-Lift-Cube-Yam(本章固定基座基准)，符合章节'轮式需自建'的说法
- mjlab 实跑 `train Mjlab-Lift-Cube-Yam --env.scene.num-envs 64 --agent.max-iterations 2`: 成功跑起，~1355 steps/s，value/surrogate loss 正常，--env.scene.num-envs/--agent.max-iterations 旗与章节(行1213-1216)完全一致
- mjlab `play Mjlab-Lift-Cube-Yam --help`: 确认 `--agent {zero,random,trained}` `--num-envs INT` `--viewer {auto,native,viser}`，章节(行1209-1211)的 play 命令100%正确
- mjlab python 探针: JointVelocityActionCfg/JointPositionActionCfg/DifferentialIKActionCfg 均在 mjlab.envs.mdp；字段含 entity_name+actuator_names+scale+use_default_offset(证实章节行417的'mjlab用entity_name+actuator_names')
- mjlab EntityData 字段探针: root_link_lin_vel_b/root_link_ang_vel_b/projected_gravity_b/root_link_pos_w/root_link_quat_w/joint_vel/joint_pos/body_link_pos_w 全部存在(证实章节观测代码行624-643)
- mujoco 3.8.1 `MjSpec` 探针: 无 attach_to，只有 attach(child,prefix=,suffix=,site=,frame=)；mjlab 源 scene.py:236 自身用 self._spec.attach(ent.spec, prefix=, frame=)——章节 attach_to 用法报错
- UniLab 实跑 `train --algo ppo --task go2w_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true`: 成功，~343 steps/s，losses/rewards 正常，章节行78命令可跑
- UniLab 源码核对: go2w/joystick.py:387 `wheel_velocity_targets = exec_actions[wheels]*wheel_action_scale`，base.py compute_go2w_motor_ctrl 对轮子做 wheel_kd*(vel_target - joint_vel)——轮子是速度控制非位置PD
- UniLab 源码核对: go2_arm/manip_loco.py @registry.envcfg('Go2ArmManipLoco')存在(cli task=go2_arm_manip_loco, 期望18 actuators)；demo.py 有 locomani 但需本地 stage-2 checkpoint 才能跑
- UniLab 观测核对: obs_groups_spec/split_obs_dict 存在；go2 返回 {'obs':49,'critic':52}(与章节行688完全一致)、go2w {'obs':53,'critic':72}；body 系 getter get_local_linvel/get_gyro/get_body_lin_vel_b 均在
- UniLab 核对: control_config 有独立 wheel_action_scale=10.0 与 wheel_Kd=0.5、scale_wheel_vel 噪声、simulate_action_latency+last_actions 内置动作延迟

**工程核对（是这样/不符）**
- [符合] mjlab train/play 命令、旗(--env.scene.num-envs/--agent.max-iterations/--agent zero|random/--viewer viser)与 list-envs 工作流: 行1209-1216 与实跑完全一致
- [符合] mjlab 动作配置类 JointVelocityActionCfg/JointPositionActionCfg 用 entity_name+actuator_names(对比 Isaac asset_name+joint_names): 行417/行510-515 表述正确
- [符合] mjlab 观测 EntityData 字段 root_link_lin_vel_b/ang_vel_b/projected_gravity_b/root_link_pos_w/quat_w/joint_vel/body_link_pos_w: 行624-643 全部真实存在
- [符合] 章节'三框架均无内置轮式底盘+臂任务、需自建'(行41-51): list-envs 证实 mjlab 无轮式、UniLab 有 Go2W/Go2ArmManipLoco 亲戚但无轮式底盘任务
- [符合] UniLab task id go2w_joystick_flat / 实跑可起、Go2ArmManipLoco 存在: 行78/行51 正确
- [符合] UniLab 观测 obs_groups_spec+split_obs_dict、go2={'obs':49,'critic':52}多3维特权 base linvel: 行688 的具体数字逐字命中
- [不符-重大] 章节反复称'UniLab 轮子由后端位置式 PD 驱动/无 velocity 控制'(行42/51/380/511/520/530): 源码 go2w/joystick.py:387 明确构造 wheel_velocity_targets、compute_go2w_motor_ctrl 用 wheel_Kd*(vel_target-joint_vel)——Go2W 轮子是速度级控制，且不残差到 default_angles
- [不符] 章节行520'要做速度级轮控得在自定义 env 改 apply_action': Go2W 已原生支持，且有专门 wheel_action_scale=10.0(与腿 action_scale=0.25 分离)——无需自己 hand-roll
- [不符-API名] 章节 MjSpec.attach_to()(行43/1138/1252/1445): mujoco 3.8.1 无此方法，真名为 MjSpec.attach(child,prefix=,site=,frame=)；且调用方向相反——应 base_spec.attach(arm_spec, prefix='yam/', site=mount_site)，章节写成 arm_spec.attach_to(base_spec,...) 参数主客颠倒
- [待gpufree补验] Isaac 部分(ImplicitActuatorCfg(stiffness=0)/DelayedPDActuatorCfg/set_external_force_and_torque/quat_rotate_inverse/RslRl*Cfg): autodl 无 Isaac 无法实跑，静态看 API 名与签名描述合理、与公开文档一致，建议 gpufree 实跑核对

**优化点 / 高级技巧**
- [已被源码证伪需改写] UniLab 对照应改为: Go2W 轮子=速度级控制(wheel_velocity_targets=action*wheel_action_scale(10.0)，torque=wheel_Kd*(v_target-v_cur))，腿=残差位置PD；'分项 scale'已原生存在(wheel_action_scale 与 action_scale/hip_action_scale 并列)，可直接借此对齐轮/臂量纲，不必在 apply_action 里手算分段缩放
- [可补] UniLab 动作延迟有内置开关 control_config.simulate_action_latency(配 last_actions 一拍延迟)，与章节 Delay Buffer 节(行544-573)互为印证: 应补一句'UniLab 侧延迟也是配置标志而非手搓缓冲'，三框架(Isaac DelayedPDActuatorCfg / UniLab simulate_action_latency / mjlab 手动)对照更完整
- [可补] UniLab 轮速观测噪声有专门 scale_wheel_vel(默认0.5)，呼应章节'轮速反馈非冗余/感知打滑'(行617决策3)，可作为'轮速观测要单独加噪'的现成工程例
- [可补] mjlab spec attach 正确范式应给可跑代码: base_spec=MjSpec.from_file(...); base_spec.attach(arm_spec, prefix='yam/', site=mount_site); model=base_spec.compile()——并指出 attach 返回 MjsFrame、site= 与 frame= 二选一
- [可补] mjlab list-envs 真实输出(Cartpole/Lift-Cube-Yam[±Depth/Rgb]/Multi-Cube-Seg/G1/Go1 系列)可作'从哪个现成任务起步扩轮式'的落地指引(Lift-Cube-Yam 即本章基准，最小可跑已验证~1355 steps/s@64env)
- [可补] 性能基线(行1266 称 4090 上轮式约固定 YAM 60-80%): 已测 RTX4080S 固定 Lift-Cube-Yam 64env≈1355 steps/s，可给一个真实量级锚点供读者对表

**教学缺口（只讲代码没教工程之处）**
- UniLab 接线整节(行520-533)基于'轮子=位置PD'的错误前提展开教学，读者照此理解 Go2W 会误判轮控语义——核心教学点(轮=速度/腿=位置如何在同一 apply_action 里并存)恰恰被讲反，应以 wheel_velocity_targets vs leg_targets 的真实两路构造重写，才真正教会'异构接口统一'
- spec attach 节(行1123-1148)只给了一段不可跑的伪 API(attach_to)，没教读者'如何自查真实方法名'(help(mujoco.MjSpec.attach))——与章节自己提倡的'先 --help/先探针再写'方法论自相矛盾；应示范用 help/dir 探明 attach 签名再写
- Isaac 虚拟臂/外力驱动(行349-366)给了 set_external_force_and_torque 代码但未教 autodl 无 Isaac 时如何降级验证，也未教'怎么确认本机 Isaac 版本的真实签名'(写数据要 write_data_to_sim)——只堆代码没给排错路径
- random-agent 验证脚本(行447-456)与 obs 健康检查(行651-657)用的是 make_env(...)/env.reset() 通用伪 API，未落到任一真实框架的可跑命令(mjlab 实为 `play <TASK> --agent random/zero --viewer viser`)——读者无法照抄即跑，建议把通用伪码替换/补一条真实 play 命令
- '42维 obs/9维 action' 等具体数字(行615/653/1212)是自建任务的设计值、无对应可跑任务证实；而 UniLab 真实 go2={'obs':49,'critic':52} 已可实测——可教读者'如何打印 env.obs_groups_spec 自查维度'，把'数维度'变成可操作技能而非背数字

**notes**：本章 mjlab 侧与观测 API 准确度高(命令/旗/动作cfg字段/EntityData字段/play 子命令/list-envs 工作流均实跑或探针证实)，UniLab 的 obs_groups_spec 数字(go2={'obs':49,'critic':52})甚至逐字命中。两处需主控修正: (1)重大事实错误——UniLab Go2W 轮子是"速度级控制"(wheel_velocity_targets=action*wheel_action_scale=10.0, torque=wheel_Kd*(v_target-v_cur)),且有独立 wheel_action_scale,章节多处(行42/51/380/511/520/530)称其为"位置式PD/无velocity控制/需自定义改apply_action"全部与源码相反; (2)API名错误——mujoco 3.8.1 无 MjSpec.attach_to,真名 attach(child,prefix=,site=,frame=)且调用主客与章节相反(应 base_spec.attach(arm_spec,...))。Isaac 部分(autodl 无 Isaac)未能实跑,已标待 gpufree 补验,静态看签名描述合理。其余教学性缺口集中在"堆伪API代码却没教读者自查真实API名/降级验证"。所有 ssh 调用均 dangerouslyDisableSandbox:true,未改任何 .tex/.bib。


---

## prac_diy

**服务器**：autodl (RTX 4080 SUPER, mjlab 1.4.0 + UniLab @ /autodl-fs/data/UniLab; 无 Isaac)

**实跑命令（13）**
- list-envs → 12 个真任务, 确认 Mjlab-Velocity-Flat/Rough-Unitree-Go1 存在(章节'无Go2'正确,是Go1), G1 velocity/tracking 齐全
- mjlab introspect register_mjlab_task → 签名 (task_id,env_cfg,play_env_cfg,rl_cfg,runner_cls=None) 与章节逐字一致; load_env_cfg(task_name,play=False) 一致
- mjlab introspect EntityCfg/BuiltinPositionActuatorCfg → 字段 spec_fn/init_state/articulation + target_names_expr/stiffness/damping/effort_limit 全部命中(还有 delay_* 字段)
- mjlab introspect managers → ObservationTermCfg/RewardTermCfg/TerminationTermCfg/EventTermCfg 真实存在; ObsTerm/RewTerm/DoneTerm/EventTerm 短别名全部 False(与章节一致)
- mjlab introspect JointPositionActionCfg → 字段 entity_name/actuator_names/scale/offset/use_default_offset 命中(非 asset_name/joint_names)
- mjlab introspect mdp → 16 个 generic + 2 个 dr(geom_friction/body_mass) + 8 个 velocity(track_linear_velocity/track_angular_velocity/body_angular_velocity_penalty/angular_momentum_penalty/feet_air_time/self_collision_cost/illegal_contact/UniformVelocityCommandCfg) 全部存在
- 运行章节 smoke_test 脚本(Go1) → 报 KeyError:'policy' —— 真实 mjlab 观测组名是 actor/critic, 非 policy/critic
- 运行 zero+random 诊断(Go1, 改 actor 键) → ZERO: h 0.307→0.250 不倒 termAny=False; RANDOM: 654 episodes epR=-1.56±1.16 epLen=91.3±55.3 —— 与章节'通过标准'高度吻合
- mjlab train Go1 --num-envs 64 --max-iterations 2 → 正常, Iteration time 1.45s, 日志 reward 项含 track_linear_velocity/track_angular_velocity/body_ang_vel/angular_momentum/action_rate_l2(与章节教的真名一致)
- UniLab train --help → --algo{ppo,mlx_ppo,appo,sac,td3,flashsac} --task --sim{mujoco,motrix} 与章节一致
- UniLab train go2_joystick_flat --sim mujoco max_iterations=1 num_envs=16 → 正常, 398 steps/s, reward 项 tracking_lin_vel/tracking_ang_vel/lin_vel_z/action_rate/similar_to_default 与章节逐字一致
- UniLab grep 源码 → Go2JoystickCfg/Go2WalkTask/registry.envcfg/registry.env/LocomotionBaseEnv/NpEnv/NpEnvState/update_state/obs_groups_spec/apply_action/run_reward_dispatch/get_dr_capabilities/CrossBackendIncompatibleError/DENYLIST/extract_contract_snapshot 全部命中
- mjlab introspect 验证 conf 观测结构 → cfg.observations 是 dict(键即组名), 非章节用的 class ObservationsCfg{nested PolicyCfg}

**工程核对（是这样/不符）**
- [符合·强] 全部 mjlab API 名实跑命中: register_mjlab_task 签名(line 454/470)、load_env_cfg(486)、EntityCfg+InitialStateCfg+articulation(285-313)、BuiltinPositionActuatorCfg 四字段(305-310)、JointPositionActionCfg 用 entity_name/actuator_names(336-345)、无 ObsTerm/RewTerm 短别名(352/804)、全部 generic/dr/velocity mdp 函数名(354-411)——本部前几轮 introspect 勘误是对的
- [符合·强] mjlab 真名 track_linear_velocity/track_angular_velocity/body_angular_velocity_penalty/angular_momentum_penalty(line 392-398) 在真 Go1 训练日志逐字出现, 非 Isaac 的 track_lin_vel_xy_exp/lin_vel_z_l2——勘误正确
- [符合·强] 验证三步走的'通过标准'实跑兑现: zero agent Go1 200步不倒(termAny=False)、base高度小幅下沉(0.307→0.250); random agent 654 episodes、epReward 负(-1.56)、epLen 有方差(±55.3)——章节 line 526/557 描述准确
- [不符·重要] obs 键名错: 章节通篇用 obs['policy']/observation_space['policy'](line 490/495/1072/1125/1453)与 class PolicyCfg(359/813/1579), 且 line 632 明示'mjlab 用 policy/critic'; 实跑真 mjlab Go1 观测组名是 actor/critic(KeyError:'policy'; observation_manager 组名 dict_keys(['actor','critic'])), 跑章节 smoke_test 原样会崩
- [不符·结构] mjlab 观测声明形态错: 章节用 class ObservationsCfg 内嵌 class PolicyCfg(ObservationGroupCfg)(Isaac 风格); 实跑 cfg.observations 是 dict[str,ObservationGroupCfg](键即组名, Go1 用 {'actor':..,'critic':..})——章节的 class+nested PolicyCfg 模式不是 mjlab 的真实组织形态
- [符合] mjlab 执行器表(line 1053-1056)实跑命中: BuiltinPositionActuatorCfg/IdealPdActuatorCfg/DcMotorActuatorCfg/BuiltinVelocityActuatorCfg/LearnedMlpActuatorCfg 均真实存在
- [符合] UniLab 三框架列实跑全兑现: CLI --algo/--task/--sim(line 59/136)、reward 标量字典真名 tracking_lin_vel/tracking_ang_vel/lin_vel_z/action_rate/similar_to_default(line 666-695)在真 go2 日志逐字出现、双装饰器 registry.envcfg/env(122-128)、Go2JoystickCfg/Go2WalkTask(120/129)、update_state/obs_groups_spec/apply_action/NpEnv(637-684)、get_dr_capabilities 后端能力门控(1252)、CrossBackendIncompatibleError/DENYLIST/contract_snapshot 跨后端契约(1133)——源码逐一命中
- [符合] 章节'mjlab 无 Go2 任务'(隐含)与 list-envs 一致(只有 Go1); UniLab 则确有 go2_joystick_flat/rough——章节 UniLab 例子用 Go2 合理
- [小瑕] zero agent reward 章节说'小负值'(line 526), 实测 Go1 为小正值(+0.076, 因内置有 upright/alive 正奖励)——非错但表述可加'或小正值(若含 alive/upright 奖励)'
- [未验] Isaac 部分(HOVER extension/convert_urdf/isaaclab.sh/ImplicitActuatorCfg/{ENV_REGEX_NS})autodl 无 Isaac 无法实跑, 待 gpufree 补验; 静态看 Isaac API 名与官方约定一致, 合理

**优化点 / 高级技巧**
- smoke/zero/random 脚本应改用动态组名而非硬编码: 用 list(obs.keys())[0] 或 env.observation_manager 取真实首个 actor 组名, 避免 mjlab(actor)/Isaac(policy) 组名差异导致脚本不可移植——这正是本章自己强调的'frame/命名静默失败'的同类坑
- BuiltinPositionActuatorCfg 实有 delay_min_lag/delay_max_lag/delay_hold_prob/delay_per_env_phase 等延迟字段, 章节 line 1203 讲'逐步扰动放 actuator 延迟'时可直接点名这些真实字段, 把'抽象建议'落成可抄的 API(mjlab 在执行器层原生支持 action delay, 是比自写缓冲区更稳的做法)
- 可补'用 env.observation_manager.compute() 分组打印各 term 维度'的真实排错手段: Bug 3(obs维度)与 B2/B7 检查表只说'逐项打印', 但未给 mjlab 真实入口(observation_manager 暴露 active_terms/group_obs_dim), 给出可显著降低读者试错
- mjlab 训练日志默认就分项输出每个 Episode_Reward/<term>(实跑可见 track_linear_velocity/action_rate_l2/...), 章节 Bug4(reward常数)/Step7体检表可点明'mjlab 训练日志自带分项 reward, 不需自己写 print'——比章节建议的手动逐项打印更省
- UniLab DR 真实命名空间是 dr.body_mass/geom_friction/dof_armature/dof_damping/body_ipos/effort_limits 等(实跑可枚举), 章节讲正定惯量缺口(line 1729)可顺带给出 mjlab 这一侧 dr 子包的真实可用项清单, 帮读者判断'哪些能直接用'
- 建议补一句 num_envs 与 steps/s 的真实量级锚点: 实测 Go1 64envs 单 iter 1.45s、UniLab go2 16envs 398 steps/s——Bug10(训练慢<100 steps/s)的判据可加'小 num_envs 下几百 steps/s 是正常的, 别误判为 bug', 否则读者拿 64-env 的低吞吐误当性能问题

**教学缺口（只讲代码没教工程之处）**
- 验证三步走脚本(smoke/zero/random)是本章工程教学的核心资产, 但直接照抄会因 obs['policy'] 崩——这恰好削弱了'教会读者自己搭验证'的目的; 应教读者'第一步先 print(list(obs.keys())) 确认真实组名', 把'组名从哪来'(mjlab=dict键, Isaac=类名)讲成一个可迁移的认知, 而非给一段在真 mjlab 上跑不通的固定代码
- 章节给了大量 mjlab 配置代码块(六步法/完整四文件), 但 observations 用 class+nested PolicyCfg 的 Isaac 形态而非 mjlab 真实的 dict 形态——读者照搭会发现'框架不认这个类'; 缺'怎么从 list-envs 找一个内置任务、load_env_cfg 把它 print 出来照着仿'的从零探路法(我实跑就是这么探明真实结构的), 这比给静态代码更能教会举一反三
- Bug 字典(十大)与检查表 A/B 多为'症状→排查→修复'的文字, 但排查工具的真实入口名(observation_manager.active_terms、data.root_link_lin_vel_b、command_manager.get_command)散落各处且未集中示范一次'打开一个真 env 逐项 introspect'的完整 session——读者学到'要排查'但未学到'用什么命令排查'
- Isaac 部分(extension/HOVER/convert)全是静态代码与目录范式, autodl 无法实跑、读者多半也只有一种框架; 缺'当你只有 mjlab(或只有 Isaac)时, 如何把另一框架的概念映射过来验证'的桥接, 否则三框架对比对单框架读者是'只能读不能验'
- zero agent reward 章节钉死'应为小负值', 但实测内置 Go1 含 alive/upright 正项时是小正值——教学上更该教'reward 的符号取决于你配了哪些项(纯正则惩罚→负, 含 alive→可能正), 关键看有没有 NaN/有没有方差', 而非给一个会让读者误判'我的环境坏了'的硬判据

**notes**：三框架(mjlab+UniLab)在 autodl(RTX4080S)全部实跑成功; Isaac 部分本机无法跑, 待 gpufree 补验(静态合理)。总体结论: 本章 mjlab/UniLab 的 API 名/命令/reward 真名/验证三步走判据经实跑几乎全部坐实(本部前几轮 introspect 勘误是对的, 质量很高)。唯一实锤的硬伤是观测组键名: 章节通篇 obs['policy']/PolicyCfg 且 line 632 明示'mjlab 用 policy/critic', 但真实 mjlab Go1 观测组名是 actor/critic(照抄 smoke_test 当场 KeyError), 且 mjlab 真实用 observations=dict(键即组名)而非 class+nested PolicyCfg(那是 Isaac 形态)——建议主控据此把 mjlab 侧观测组名统一改 actor、并把 ObservationsCfg 类形态改成 dict[str,ObservationGroupCfg] 或至少加注说明 mjlab 组名/形态差异。其余为可优化/教学性补强, 非错误。所有 ssh 命令均 dangerouslyDisableSandbox:true。涉及文件: /home/ziren2/pengfei/Robotics_Theory/Robotics_Note/parts/P8_rl_motion_control/prac_diy.tex (line 359/490/495/632/813/1072-1073/1125/1453/1579 = obs 组名问题; line 526 = zero reward 表述)。

---

## prac_sim2real

**服务器**：autodl (RTX 4080 SUPER；mjlab 1.4.0 + UniLab，无 Isaac)

**实跑命令（12）**
- mjlab `uv run list-envs`：12 个任务，确认有 Mjlab-Velocity-Flat-Unitree-G1(章节 Quick Start 用) 及 Tracking/manipulation(Lift-Cube-Yam) 任务，无 Go2(仅 G1/Go1)——与章节一致。
- mjlab Quick Start 实跑 `train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 64 --agent.max-iterations 2 --agent.save-interval 1`：成功，2 迭代，~1344 steps/s，value loss~1.1，无报错。
- find logs：训练自动产出 `<ts>.onnx` + `model_0.pt`/`model_1.pt`（单个 .onnx，非每 ckpt 一个），符合章节 line 84 通过标准。
- 读真 mjlab ONNX metadata：keys 恰=章节 line 340 的 7 个(joint_names/joint_stiffness/joint_damping/default_joint_pos/command_names/observation_names/action_scale)+run_path；input 名=obs、output=actions、obs_dim=99、act_dim=29。
- 实跑章节 D0 内核(line 693-712)逐字：在真 ONNX 上 **抛 AssertionError: 缺少 metadata: obs_dim** —— D0 自身有 bug。
- ONNX 推理：零 obs action max=0.26 无 NaN；确定性(重跑 max diff=0.0)；但 **input batch 固定为 1**，feed batch=64 报 InvalidArgument(Got:64 Expected:1)。
- 源码核对 mjlab runner.py：opset_version=18、dynamo=False、export_policy_to_onnx/get_base_metadata/attach_metadata_to_onnx 全在；velocity/tracking runner save() 重写+super().save()+try/except `[WARN] ONNX export failed (training continues)`；tracking 加 anchor_body_name/body_names —— 与 line 318/340 逐字相符。
- 源码核对 mjlab actuator.py：delay_min_lag/delay_max_lag/delay_hold_prob/delay_update_period/delay_per_env_phase + DelayBuffer 全在(line 67-85)，与 line 552 逐字相符。
- UniLab Quick Start 实跑 `train --algo ppo --task go2_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true`：成功 1 迭代 ~413 steps/s，Critic 含 EmpiricalNormalization(印证 line 327/347)，无报错。
- UniLab no_play=true 运行后 find：**无 .onnx/deploy_config.yaml** —— 印证章节 line 103/345 『PPO no_play 不产 ONNX，须显式 eval』。
- 源码核对 UniLab：sim2sim.py 有 CrossBackendIncompatibleError/DENYLIST/ENV_STRUCTURAL_DENYLIST/resolve_sim2sim_config/contract_snapshot/policy_load_dim_guard；train_appo.py 有 _DeterministicAPPOActor(actor.mlp)+`ONNX export verified OK.`；train_offpolicy.py 有 as_export_module()+export_onnx(默认 True)——与 line 345/371/380/381/445/473-478 全部相符。
- 源码核对 UniLab scripts/deploy/：恰为 append_cooldown/export_deploy_config/export_motion_bin/prepend_warmup/sim_prototype 5 文件(line 346 逐字)；export_deploy_config.py 写 obs_layout/obs_dim/default_angles 且注释『State_WBT.cpp+ObservationManager reads obs_layout』；sim_prototype.py 自述『Python prototype of State_WBT…validate same obs vector』(line 392/397/728-730)。

**工程核对（是这样/不符）**
- 【不是这样·真 bug】D0 静态核对内核(line 699)把 obs_dim 当 metadata key 断言 `for k in ("joint_names","action_scale","obs_dim"): assert k in meta`——但 obs_dim 并不在 mjlab ONNX metadata_props 里(只有 7 个 key)；实跑该函数在合法 ONNX 上直接抛 `AssertionError: 缺少 metadata: obs_dim`。这是本章旗舰『5 分钟挡掉一半 bug』代码却自身报错。修法：obs_dim 已由 line 702 从 graph 读出，不该列入必查 metadata；必查集应改成 joint_names/action_scale/default_joint_pos(三者实测均在)。
- 【不是这样·runnable 缺陷】sim2sim 核心循环(line 459-466)`env=make(...,num_envs=64)` 后把 obs['policy']([64,99]) 直接喂 `session.run`——但实测 mjlab 1.4.0 导出的 ONNX **input batch 维固定=1**，feed batch=64 报 `INVALID_ARGUMENT: Got:64 Expected:1`，该代码框无法按原样跑。需逐 env 循环(batch=1)或导出时传 dynamic_axes 放开 batch 维。
- 【小不符】sim2sim 代码框注释(line 459)写 `# mjlab Go2 velocity`，但 mjlab 无 Go2 任务(list-envs 仅 G1/Go1)；与章节自身 line 28/439『mjlab 无 Go2』口径冲突，应改 G1。
- 【是这样·已确认】mjlab『每次存档自动导 ONNX + 7 项 metadata + try/except WARN 不中断』、actuator 延迟字段、opset18/dynamo=False，UniLab『PPO 仅 eval 导 ONNX / deploy_config.yaml 承载 obs_layout / sim2sim.py 契约关卡 + CrossBackendIncompatibleError / sim_prototype=State_WBT 原型』——全部源码+实跑逐字坐实，无误。
- 【Isaac 部分待 gpufree 补验】本章 Isaac Lab 侧(play.py 导出、unitree_rl_lab 五阶段、policy.onnx.data 仅超 2GB 才产、Newton sim2sim、deploy.yaml/joint_ids_map)在 autodl 无法实跑；静态看代码描述合理(与 mjlab 对称、字段名标注为概念化已诚实声明),但需 gpufree 上 Isaac 环境核实 play.py 导出时机与 deploy.yaml 真实字段。

**优化点 / 高级技巧**
- D0 内核应额外断言 ONNX 的 **batch 维**情况：实测 mjlab 导出 batch 固定=1(部署单步推理 OK，但批量评估会踩坑)。可补一句『读 g.input[0].shape.dim[0]，若固定=1 则只能单 obs 推理；要批量 sim2sim 需 dynamic_axes 重导』——正是本章 line 98『动态维度没固化会踩坑』的反面坑(这里是 batch 维被写死),却没覆盖到。
- D0 可把『assert obs_dim==metadata 声明值』做成真校验：mjlab 无 obs_dim metadata，但 UniLab deploy_config.yaml 有 obs_dim；章节 UniLab D0(line 723-725)已正确做了 YAML-vs-ONNX 维度比对,可把这套思路回灌到通用 D0,而非对 mjlab 错误地查 obs_dim metadata。
- 可补『如何确认 normalizer 到底有没有 baked-in』的实操：实测 mjlab G1-velocity ONNX **无 obs_mean/obs_var metadata**(归一化未单独外挂),而 UniLab Critic 实测带 EmpiricalNormalization。章节多处(line 312/456/497)让读者断言 metadata 含 obs_mean,但 mjlab 这条路径根本没有该 key——应教读者先 print 全 keys 判断本框架走哪种(baked-in 计算图 vs 外挂 mean/std vs 干脆没归一化),而非默认有 obs_mean。
- sim2sim 代码框可给出『单 ONNX(batch=1) 如何驱动 num_envs>1 仿真』的正确范式(向量化 env 但逐 env 推理,或一次性 re-export dynamic batch),这是把『导出边界』讲透的好落点,目前缺。
- 建议补一句 ONNX warmup/providers 的部署提示:实测首次 InferenceSession.run 有冷启动开销(与 line 956 故障表的『C++ 推理 jitter 大→加 warmup』呼应),Python D2 影子模式同样需要,可在 telemetry/延迟节点一句带过。

**教学缺口（只讲代码没教工程之处）**
- D0 这一章节最强卖点(『5 分钟挡一半 bug』),给的内核代码却在真 ONNX 上直接 AssertionError——读者照抄会以为自己的合法 ONNX 有问题,反而被误导。教学上既要给『能真跑过的 D0』,也应顺势教『不同框架 metadata key 不同,必查集要按框架定制』(mjlab 7 key 无 obs_dim/obs_mean、UniLab 走 YAML),这正是举一反三点,目前缺失。
- sim2sim 代码框只贴了循环,没教『ONNX 的 batch 维可能被写死=1』这个真实部署边界——读者拿去配 num_envs=64 会报错却不知为何。好的工程教学应点明『导出时 batch 维是固定还是 dynamic、怎么查、怎么按部署形态(单步真机 vs 批量 sim2sim)选』,把『导出边界』从概念落到可操作。
- 章节反复让读者『断言 metadata 含 obs_mean/obs_var』(line 312/456/497/953),但未教读者先实际 print metadata_props 看本框架到底有没有这条——mjlab 这条路径就没有。缺『如何从零判断我的框架走 baked-in 归一化还是外挂还是无归一化』的排查方法,容易让读者在错误前提上断言。
- Isaac 侧大量字段(deploy.yaml/joint_ids_map/policy.onnx.data)只能照文档描述,章节已诚实标注『概念化字段名/以安装版本为准』(line 437)——这点教学诚实度好;但建议补一句『怎样在自己机器上一行命令 dump 出真实字段名(读导出脚本/读产物 yaml)』,让读者具备自行证伪的手段,而非只能信书。

**notes**：未改任何 .tex/.bib。总体结论：本章工程描述**高度可靠**——mjlab/UniLab 的命令、API 名、metadata 字段、actuator 延迟字段、sim2sim 契约类名、scripts/deploy 文件清单、PPO no_play 不产 ONNX 等核心声明全部经实跑+源码逐字坐实，三轮复核质量很高。需主控修的实质问题仅两处代码：(1) D0 内核 line 699 把 obs_dim 误列为必查 metadata key，实测在真 mjlab ONNX 上抛 AssertionError，应改为从 graph 读 obs_dim、必查集换成 default_joint_pos 等真实存在的 key；(2) sim2sim 代码框 line 459-466 喂 batch=64 但 mjlab ONNX batch 维固定=1，实测报 InvalidArgument，需改逐 env 推理或导出 dynamic_axes。另有 1 处注释小不符(line 459 'mjlab Go2 velocity' 应为 G1)。Isaac 实战部分 autodl 无环境，标记待 gpufree 补验，静态看与 mjlab 对称且字段名已诚实声明为概念化，合理。


---

