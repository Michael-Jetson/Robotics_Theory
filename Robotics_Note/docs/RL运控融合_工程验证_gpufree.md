# P8 工程验证报告 · gpufree 路（14 章，含 Isaac）

> 服务器实跑核验工程实践 + 找优化 + 评教学性。2026-06-25。每章含：实跑命令 / 工程核对 / 优化点 / 教学缺口。

## prac_ecosystem

**服务器**：gpufree (RTX 4090 24GB, Xeon Gold 6430 64C; mjlab 1.4.0 / Isaac Lab 2.3.2 / UniLab 三框架齐全)

**实跑命令（13）**
- list-envs → 恰12个任务,仅G1/Go1无Go2,任务名与章节全表一致(line 770)
- python -c importlib.metadata → mjlab 1.4.0 + rsl-rl-lib 5.2.0,与版本表(line 1003-1006)一致
- dir(mjlab.managers) → 9个Manager含MetricsManager(mjlab独有),无ObsTerm/RewTerm别名(False/False)
- WANDB_MODE=disabled train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 64 --agent.max-iterations 2 → 跑通,1600-2570 steps/s,reward非NaN,输出含Metrics/*行(证实MetricsManager)
- grep velocity/mdp → 奖励真名 track_linear_velocity/track_angular_velocity(非Isaac的*_exp),证实line 624-625
- grep play.py → agent: Literal['zero','random','trained'],证实Quick Start的--agent zero/random(line 91-93)
- grep velocity_env_cfg.py → obs组名 actor/critic(非policy),证实line 621/701;timestep=0.005+decimation=4证实line 344-346
- isaaclab.sh train.py --task Isaac-Cartpole-v0 --headless --max_iterations 2 → 跑通2迭代,无NaN,RSL-RL输出Episode_Reward/*
- UniLab: registry.list_registered_envs() → dict,count=26(精确证实line 770'26个注册任务')
- UniLab train --algo ppo --task go2_joystick_flat --sim mujoco algo.num_envs=16 → 跑通,578 steps/s(CPU),reward/*分解与scales字典一致
- grep UniLab src → NpEnvState/DomainRandomizationManager/PenaltyCurriculum/run_reward_dispatch/NanGuard/FinalObservationAwarePPO(PPO)/SharedWeightSync/memory_budget.py 全部存在
- cat conf/ppo/task/go2_joystick_flat/mujoco.yaml → reward.scales.tracking_lin_vel=1.0等,与line 681-687逐字一致;appo/flashsac配置树亦存在
- grep Isaac源码 → ObservationTermCfg/RewardTermCfg/EventTermCfg存在,ObsTerm确为'import as'别名(证实line 612/702);grep mjlab → register_mjlab_task存在(证实三种注册机制line 709)

**工程核对（是这样/不符）**
- 全部实跑通过、未发现任何不符:三框架训练命令(mjlab/Isaac/UniLab)按章节写法均跑通,2迭代无NaN,steps/s与loss正常
- 版本号全中:mjlab 1.4.0、rsl-rl-lib 5.2.0,与版本表line 1003-1006一致;mjlab无ObsTerm/RewTerm别名(实测dir()返回False)亦准确
- mjlab任务清单精确:恰12个、仅G1/Go1无Go2,任务名逐一对上(line 770)
- mjlab独有MetricsManager属实:dir(mjlab.managers)含MetricsManager且训练输出真有Metrics/angular_momentum_mean等行(line 529)
- mjlab奖励/obs命名差异属实:track_linear_velocity(非*_exp)、obs组名actor/critic(非policy)、env基类ManagerBasedRlEnvCfg(小写l)——全部与章节并排表(line 698-704)一致
- UniLab异构架构全部细节核对无误:NpEnvState/DomainRandomizationManager/PenaltyCurriculum/run_reward_dispatch/NanGuard/SharedWeightSync/memory_budget.py 源码确在;FinalObservationAwarePPO确实extends RSL-RL的PPO(line 848)
- UniLab '26个注册任务'(line 770)精确属实:registry返回dict长度恰26;reward YAML路径与scales字典(line 681-687)逐字一致;appo/flashsac off-policy配置树亦在(证实line 769/855)
- Quick Start的--agent zero/random体检属实:play.py确有该Literal,DUMMY_MODE分支存在
- decimation示例属实:mjlab velocity实测timestep=0.005+decimation=4=50Hz,与line 344-346一致
- 三种注册机制属实:mjlab register_mjlab_task(传已实例化cfg)、Isaac gym.register、UniLab @registry.env(双装饰器多后端)——line 709描述准确

**优化点 / 高级技巧**
- mjlab可补'真实steps/s基准':本机RTX4090上64envs仅~2.5k steps/s(kernel未饱和),章节line 357的'~10^5 steps/s'是4096envs量级——可提示读者小env数下吞吐远低于峰值,首跑别被低数字吓到(JIT/CUDA-graph预热也占首迭代)
- Isaac Sim启动慢可给量化预期:实测Cartpole从启动到出第1迭代>300s(纯shader/USD加载),章节line 38只说'数十秒'偏乐观——可建议读者首跑预留5-8分钟、用--headless、并说明这是一次性编译缓存(二次更快)
- 可补'bare python -c import isaaclab会因缺pxr报ModuleNotFoundError'这一真实坑:必须用./isaaclab.sh -p而非系统python,否则import isaaclab.envs即崩——这是新手极易踩的环境隔离坑,章节未点明
- UniLab可补off-policy实跑入口:conf下确有appo/offpolicy(flashsac)树,章节line 855提到APPO/SAC是一等公民却无对应run命令示例,可补一条'train --algo appo/flashsac ...'让读者真上手异步路径
- 可补显存实测校验ins:rlmc-eco-vram(line 377):RTX4090=24GB,公式估算与实跑OOM边界可给一个真实数对照,让估算公式从'够用'升级为'可信'

**教学缺口（只讲代码没教工程之处）**
- zero/random体检(line 91-103、ins:rlmc-eco-zerorandom)讲了'要跑'与'看什么',但没给一条可复制的命令看输出长什么样——读者跑完--agent zero后该在终端/Viser里具体看哪个量、瘫软vs站立如何判读,仍停在概念;补一段真实输出截屏会让'调试仪器'角色落地
- sanity_check代码(line 723-738)是Isaac风格(obs_dict['policy']),但mjlab组名是actor、UniLab是obs/critic——章节注释提了'mjlab用actor',却没把这段实战代码在三框架各跑一遍给出差异,读者照抄到mjlab会KeyError(章节自己line 716也警告了这点,却没在验证小节兑现)
- '怎么从零搭一个新任务'缺失:全章是'选型+对比+读已有配置',但没教读者真正新建一个Manager任务/新UniLab NpEnv子类的最小步骤(注册→cfg→跑通),练习line 744只是'加一行RewTerm';从零搭起这一最能教会工程的环节没覆盖
- 排错教学偏'症状表'(故障排查手册line 969)而少'排查过程':给了症状→原因→相关节,但没演示一次真实排错的命令序列(如OOM了如何用nvidia-smi定位、KeyError如何打印实际组名),读者学到'是什么'但没学到'怎么自己查'
- steps/s量级(line 357)标了'待核实'但没给读者'如何自己测吞吐'的方法(跑N迭代看Steps per second行)——本可借实跑教会读者用框架自带输出做性能自检,是个被放过的工程教学点

**notes**：本章工程准确度极高:在gpufree实跑13组命令,覆盖三框架的版本/任务清单/训练命令/API名/配置结构/奖励函数名/注册机制——逐一核对全部属实,未发现任何编造或不符(此前三轮复核+旗核销显然到位)。三框架训练均跑通、无NaN。优化与教学缺口均为'锦上添花'级(可补真实输出/吞吐基准/从零搭建/排错过程),非错误。唯一与正文措辞略乐观处:Isaac Sim启动实测>300s远超'数十秒'(line 38),建议主控酌情补'首跑预留数分钟'。所有.tex未改动。

---

## prac_manager_arch

**服务器**：gpufree (RTX 4090; mjlab 1.4.0 / Isaac Lab 2.x@/workspace/isaaclab / UniLab@/root/gpufree-data/src/UniLab)

**实跑命令（14）**
- list-envs → 确认 mjlab 12 任务、有 G1/Go1+Rough、无 Go2(与章节一致)
- mjlab managers import 全名 cfg 成功；ObsTerm/RewTerm/DoneTerm/EventTerm/TermTerm/ObsTermBase/RewardTermBase 全部 correctly-absent(印证 ins:rlmc-mgr-alias)
- go1/rl_cfg.py 源码: RslRlModelCfg/RslRlPpoAlgorithmCfg/RslRlOnPolicyRunnerCfg/num_learning_epochs/clip_param 全 FOUND, num_epochs/clip_range absent(good) — 但 max_iterations=10_000(章节写30000)
- train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2 → 跑通, 1817→2474 steps/s, value loss 0.0417→0.0246, surrogate sane, 有 Metrics/* — 健康
- mjlab train --help: 真实开关 --env.scale-rewards-by-dt(default True) 印证 scale_by_dt; --env.rewards.track-linear-velocity.* 印证 tyro 任意深度覆盖+mjlab 函数名 track_linear_velocity
- play --agent {zero,random,trained} + --num-envs 存在 → 印证 Quick Start --agent zero
- mjlab manager_based_rl_env.py step() 源码: decimation 子循环(apply/write/step/update/substep)→termination(436)→reward(440)→reset(447)→sim.forward(454)→command(456)→event step+interval(459/461)→sim.sense(463)→obs(464) 全部与 18 步表相对顺序一致; mode=step 确实存在
- EntityData 属性: root_link_*/joint_pos/default_joint_pos(data.py:50 dataclass 字段) FOUND；但无 body_pos_w(实为 body_link_pos_w/body_com_pos_w)；Entity 无 write_joint_targets(实为 set_joint_position_target/write_joint_position_to_sim)
- Isaac Cartpole train --max_iterations 2 → 跑通, value/surrogate loss + Episode_Reward/<term> 分项日志(印证 tab:rlmc-mgr-ppolog)
- Isaac managers/__init__ 不导出 ObsTerm/RewTerm/DoneTerm/EventTerm/CurrTerm/ObsGroup(仅 EventTermCfg) → 印证别名是 import 习惯非库符号
- Isaac manager_based_rl_env.py step(): termination(204)→reward(208)→obs(212,reset前)→reset(221)→command(232)→event interval(235)→obs(238,reset后)→return 5元组(obs,reward,terminated,time_outs,extras)(241)
- Isaac EventManager 模式=prestartup/startup/reset/interval, 无 step, 仅 interval 在 step 内自动触发 → 印证 4-mode(mjlab) vs 3+prestartup(Isaac)
- UniLab registry: 26 个 CamelCase envcfg(Go2JoystickFlat/G1WalkFlat/...) → 印证'实测26个'; conf 下 go2_joystick_flat/g1_walk_flat/go2_arm_manip_loco/g1_motion_tracking slug 齐全
- UniLab train --algo ppo --task go2_joystick_flat --sim mujoco max_iterations=1 num_envs=16 → 跑通, 588 steps/s, critic=52→512→256→128→1(印证 critic:52), reward/* 标量分项(tracking_lin_vel 等)印证标量字典派发表

**工程核对（是这样/不符）**
- 是这样: mjlab 不导出 ObsTerm/RewTerm/DoneTerm 等短别名、唯一 term 基类 ManagerTermBase — 实跑逐一核销, 完全符合(ins:rlmc-mgr-alias, line 733-734)
- 是这样: 三框架训练命令均跑通且 loss/steps-s 正常 — mjlab Go1(~2k steps/s)、Isaac Cartpole、UniLab go2(588 steps/s) 全部起得来、无报错、loss 下降
- 是这样: mjlab env.step 18步相对顺序(termination→reward→reset→forward→command→event→sense→obs)与源码逐行吻合(manager_based_rl_env.py:419-464); mode='step' 确实存在(line 459)
- 是这样: Isaac step 返回 5元组(obs,reward,terminated,time_outs,extras)、永远 auto-reset、奖惩 reset 前算、obs reset 后取 — 源码 line 204-241 核实
- 是这样: Isaac EventManager 无 step 模式(只 prestartup/startup/reset/interval)、仅 interval 在 step 内自动触发 — 源码核实, 章节 4-vs-3 模式对比准确
- 是这样: UniLab 26 个 CamelCase 任务、reward 为标量字典派发(reward/tracking_lin_vel 等分项日志)、critic 输入维度=52 — 实跑日志直接印证(line 661/748/839)
- 是这样: scale_by_dt 真实 CLI 开关名 --env.scale-rewards-by-dt(default True)、tyro 任意深度覆盖(--env.rewards.track-linear-velocity.*)、mjlab 函数名 track_linear_velocity(非 Isaac 的 track_lin_vel_xy_exp) — 全部核实
- 是这样: play --agent {zero,random,trained}、mjlab 自带 MjlabOnPolicyRunner(rl/runner.py:11) 基类, VelocityOnPolicyRunner 继承之 — 章节 line 241/671 两处命名都对
- 不是这样(API名错1): tab:rlmc-mgr-entitydata(line 1047)与 entityapi(line 1067-1075相邻)的 body_pos_w 在 mjlab 1.4.0 不存在 — 实为 body_link_pos_w / body_com_pos_w; data.py 仅有这两个属性
- 不是这样(API名错2): ActionManager codebox(line 875)与 entityapi 表(line 1073)的 write_joint_targets(...) 在 mjlab 1.4.0 全库不存在(grep 0命中) — 实为 set_joint_position_target(...) / write_joint_position_to_sim(...)
- 不是这样(数字过时): max_iterations Go1 章节反复强调=30000(line 226/241), 但 mjlab 1.4.0 实测 go1/rl_cfg.py = 10_000 (num_steps_per_env=24 一致)。章节把30000当'实测核实'值, 与现版本不符

**优化点 / 高级技巧**
- mjlab CLI 已有 --agent.max-iterations / --env.scene.num-envs / --enable-nan-guard, 但章节 Quick Start 与练习里多处仍写 uv run train/play(开发者 source 安装形态); 服务器上直接用 train/play 入口脚本即可——可补一句'装好后有 train/play 控制台脚本, 不必 uv run'降低读者照搬失败率
- tab:rlmc-mgr-entitydata 可升级为'按访问语义区分 link vs com': mjlab 1.4.0 把 body/root 速度位姿都拆成了 _link_ 与 _com_ 两套(body_link_pos_w/body_com_pos_w、root_link_*/root_com_*), 这是比 Isaac 更细的设计点, 章节只给单一名字反而丢了这个可教的工程区分
- UniLab 实测 reward 日志 key 是 reward/<term>(不是 mjlab/Isaac 的 Episode_Reward/<term>), 三框架日志命名空间不同 — tab:rlmc-mgr-ppolog 与诊断流程可补一句'UniLab 日志前缀是 reward/ 非 Episode_Reward/', 否则读者跨框架查曲线会找不到
- 可补'用 --env.rewards.<name>.weight N 做单项消融实测命中'的一行真命令(已验证 tyro 支持), 把 ins:rlmc-mgr-value 的'改一行 config'从断言变成可复现操作, 教学说服力更强
- Isaac step 真实源码在 reset 前后各算一次 obs(line 212 & 238), 章节简化示意只画了 reset 后一次 — 可加一句脚注点明'Isaac 实现 reset 前也算了一次 obs 供日志/extras, 但返回的是 reset 后那次', 避免读者对源码时困惑

**教学缺口（只讲代码没教工程之处）**
- EntityData API 表是'抄属性名'式罗列(line 1036-1077): 既未教读者'怎么自己列出某版本所有属性'(一句 [a for a in dir(entity.data) if not a.startswith('_')] 或读 data.py dataclass 字段就能自查), 也因此把 body_pos_w/write_joint_targets 这种过时名直接写进表里 — 正印证章节自述方法论('学会查')未落到这张最易漂移的表上
- 三框架训练命令(Quick Start line 1000-1002、自定义 term 三步 line 998-1005)只给了命令, 没教'跑起来后看哪一行确认成功': 实跑可见 steps/s、value loss、reward/<term> 三个信号缺一即出问题, 但正文未把'第一次跑通该盯哪三行输出'写成可操作清单(tab:rlmc-mgr-ppolog 偏理论曲线, 没对应到首跑终端输出)
- max_iterations=30000 这类'实测锚点'数字未给读者复核路径: 应教'打开你这版 rl_cfg.py 看 max_iterations='(本次实测 Go1=10000≠30000), 否则读者把书里数字当真理反被误导 — 这恰是章节反复强调'以你机器实测为准'却未在此处示范的地方
- '能跑但不学习'诊断章(line 1099-1170)给了6步诊断表却无一条真命令: 如何'打印每个 reward term'(mjlab 看 Episode_Reward/<term> 全零项)、如何'打印 obs shape'在三框架各怎么落地, 全是文字描述; 读者无法照着排错, 建议至少补 mjlab 一条'看哪个 Episode_Reward/<term>=0 即定位失效 term'的实操(本次 Cartpole 实跑已展示该日志形态)
- UniLab 'critic 52/obs 49'这类维度断言(line 748/792)未教读者怎么自查: 实测跑一次训练看 critic 网络首层 in_features 即得(本次见 52), 章节可补一句'首跑日志会打印 actor/critic MLP, 首层维度即各组 obs 维度'——把硬编码数字变成可验证习惯

**notes**：三框架(mjlab 1.4.0 / Isaac Lab 2.x / UniLab)全部实跑成功, 训练均起得来、loss/steps-s 正常。章节核心架构叙述(env.step 18步时序、Manager 加载序、terminated/truncated、三框架范式对比、短别名陷阱、Isaac 4-vs-3 event 模式、UniLab 非 Manager-Based 标量字典 reward)经源码与实跑逐项核实, 高度准确, 这是一份质量很高的章节。仅发现 3 处需主控修正的硬事实: (1) body_pos_w 应为 body_link_pos_w(tab line 1047); (2) write_joint_targets 应为 set_joint_position_target/write_joint_position_to_sim(line 875+1073); (3) Go1 max_iterations 实测=10_000 而非章节断言的 30000(line 226/241), 后者还被标为'已实测核实', 与现版本不符。教学性主要缺口是 EntityData API 表与'能跑但不学习'诊断、三步流程等处偏'给命令/列名字'、缺'首跑盯哪几行、出错怎么自查'的可操作落地——而这恰与章节自我标榜的'教方法不背数字'方法论存在落差。所有 .tex 未改动。

---

## prac_obs_action

**服务器**：gpufree (RTX 4090 24GB; mjlab 1.4.0 / Isaac Lab 2.3.2 / UniLab 全在位)

**实跑命令（11）**
- mjlab list-envs → 12 任务,确认有 Velocity/Tracking-Unitree-G1/Go1、无 Go2、有 Velocity-Rough-Unitree-G1 (章 L833 引用的存在)
- mjlab python -c import ObservationTermCfg → 字段恰为 [func,params,noise,clip,scale,delay_min_lag,delay_max_lag,...,history_length,flatten_history_dim],ObsGroupCfg 有 enable_corruption → 完全匹配章 L225/L229/L487 的 compute→noise→clip→scale→delay→history 管线与 per-term delay
- mjlab play Mjlab-Velocity-Flat-Unitree-G1 --help → 确认 --agent {zero,random,trained} 与 --num-envs INT 存在 (章 smoke-test box L823-825 命令成立)
- isaaclab -p probe_isaac2.py (AppLauncher) → 8 个 obs 函数名(base_lin_vel/projected_gravity/joint_pos_rel/last_action/generated_commands/height_scan 等)全 True;Unoise=n_min/n_max,Gauss=mean/std,AdditiveUniformNoiseCfg is UniformNoiseCfg 别名 → 全匹配 L213/L289
- isaaclab probe → JointPositionActionCfg/JointVelocityActionCfg 有 use_default_offset、JointEffortActionCfg 无 → 精确匹配章 L526 论断
- isaaclab probe → DifferentialInverseKinematicsActionCfg 字段=[asset_name,clip,joint_names,body_name,body_offset,scale,controller];无 delta_pos_scale/max_dq;错误名 DifferentialIKActionCfg 不存在;DiffIKControllerCfg 有 ik_params(无 damping 字段);export_policy_as_onnx 存在 → 全匹配 L645-649 的'编造字段已纠正'与 L786
- WANDB_MODE=disabled mjlab train Mjlab-Velocity-Flat-Unitree-G1 --num-envs 64 --max-iterations 2 → EXIT 0,1595→2487 steps/s,value/surrogate/entropy loss 均有限,无 NaN (验证 smoke-test 方法学)
- mjlab train Mjlab-Velocity-Rough-Unitree-G1 --num-envs 32 --max-iterations 1 → EXIT 0;实测 obs group 字面叫 'actor'(286)/'critic'(298),critic 多出 foot_height(2) 特权项+height_scan(187) → 活体印证章 L264-276 foot_height 特权例 + L359/L410 非对称 actor-critic
- UniLab train --algo ppo --task go2_joystick_flat --sim mujoco max_iterations=1 num_envs=16 → EXIT 0,589 steps/s,reward 含 action_rate/similar_to_default,无 NaN (验证 UniLab '短训练代替 zero/random' 范式 L836)
- UniLab grep 源码 → common/base.py 存在,LocomotionBaseEnv.apply_action L100 字面 'exec_actions*action_scale+default_angles' 匹配章 L517;go2/joystick.py obs_groups_spec 返回 {'obs':49,'critic':52} 与章 codebox L423 逐字一致;split_obs_dict(observations.py:16) 返回 (obs['obs'], obs.get('critic',actor)) 匹配 L416
- UniLab grep → _obs_noise(data,scale)+noise_cfg.scale_gyro/scale_gravity/scale_joint_angle/scale_joint_vel 匹配 L313/L430;utils/nan_guard.py(NanGuard)+pyproject unilab-viz-nan 入口 匹配 L886;scripts/deploy/export_deploy_config.py+sim_prototype.py 存在 匹配 L808;tracking_obs.py(G1WBTObs) 存在 匹配 L770

**工程核对（是这样/不符）**
- 全部'是这样':三框架所有 load-bearing API 名/字段/源码路径/方法学逐项实跑核实通过,未发现任何编造或不符——本章准确度极高
- mjlab 管线声明完全属实:ObservationTermCfg 字段顺序 noise→clip→scale→delay(delay_min_lag/delay_max_lag)→history 与章 L225 一字不差,per-term delay 确为 mjlab 独有(Isaac 的 ObsTerm 无此字段)
- Isaac Lab 三处'已核实纠正'的论断全部成立:JointEffortActionCfg 确无 use_default_offset(L526);DiffIK 类名/字段确如章所述、编造的 delta_pos_scale/max_dq/DifferentialIKActionCfg 确不存在(L645);noise 字段确为 n_min/n_max 非别的(L289)
- 活体最强印证:mjlab rough G1 实跑打出 group 名就是 actor/critic、critic 仅多 foot_height 特权项,与章 codebox(foot_height 仅给 critic L264、CriticCfg 追加 base_lin_vel L410)的设计意图完全吻合——不是纸上对照而是跑出来的
- UniLab {'obs':49,'critic':52} 这个具体维度与章 codebox L423 逐字相同,说明 codebox 是照真实源码誊写而非杜撰;LocomotionBaseEnv.apply_action 的 action_scale+default_angles 硬编码、PdControlConfig、simulate_action_latency 全部属实
- 三框架训练均一次跑通无 NaN(mjlab 64env 2487 steps/s、UniLab 16env 589 steps/s、mjlab rough 32env),章的 smoke-test 七步法与'极短训练看 steps/s 与 loss 是否正常'方法学经得起实跑

**优化点 / 高级技巧**
- 诊断脚本(L861)可补一招更现代的 Isaac Lab 调试入口:env 创建后直接打印 env.observation_manager 与 env.action_manager 的 __str__——实跑 mjlab rough 时框架自动表格化打印了每个 group 的逐项 shape(actor(286)/critic(298)/height_scan(187)/foot_height(2)),这正是 L479 Bug2'打印 actor/critic 实际维度'的现成手段,值得在正文点名'manager 的 repr 自带此表,无需手写'
- scale 阶段(L315)可补 Isaac Lab 的 obs clip 与 RunningMeanStd/empirical_normalization 的部署导出细节:rsl-rl 的 empirical_normalization 开关与 normalizer 是否随 ONNX 导出是真机最常见踩坑点,章 L811 已警示原理但未给'如何核对 ONNX 里确实含 normalizer 层'的具体动作(如 onnx.load 后查 graph 节点)
- history 维度预算(L329)可补一句性能向工程量化:实跑显示小 env 数下 steps/s 受 CPU/启动主导(UniLab 16env 仅 589、mjlab 64env 1595→稳态 2487),正文若给一句'history_length 翻倍主要吃显存与回合 buffer、对 GPU steps/s 影响在大 batch 下才显著',能帮读者建立'何时该担心维度'的直觉
- UniLab 段可补 NanGuard+unilab-viz-nan 的实操价值点:这是三框架里独有的'NaN 离线复现'工具链(已核实 tools/viz_nan.py 存在),建议在 L886 之外的正文显眼处把它抬成'UniLab 相对另两框架的一个真实工程优势',而非仅作脚注式对照
- mjlab/Isaac 可补 --agent trained 的存在(实跑 play --help 显示 zero/random/trained 三选项):章只讲了 zero/random 两个 smoke agent,trained 是加载 checkpoint 回放的入口,部署前可视化常用,值得在 smoke-test 一节顺带点名

**教学缺口（只讲代码没教工程之处）**
- 本章教学性整体极强(全书少见):几乎每个 API 都先给'为什么这么做'(insight)+'违反症状→修复方向'表+'怎么排查'(诊断脚本/打印维度),远超'逐行翻译代码'——这是范本级,以下仅为锦上添花的小缺口
- '从零搭起'维度略弱:章教了'改 obs/action 后怎么验',但没给'拿到一台全新机器人、从空 cfg 到第一个能跑的 velocity 任务'的最小骨架步骤(先放哪几项 obs→怎么定 action_scale 初值→第一次 zero/random 看什么)。实跑表明框架会自动打印 manager 表,可借此组织一节'第一次接线 checklist'
- scale 调参'怎么调'给了现象→问题映射表(L538/L884)很好,但缺一个量化锚点:如'random agent 下 processed action 该落在关节限位的百分之多少算健康'、'steps/s 掉到多少该怀疑 Python 循环'(实跑 mjlab 稳态 ~2500 steps/s@64env 可作参照),给个数量级能让读者把'异常'判得更准
- delay 阶段(L320)讲清了'lag 单位是 env step 不是 physics step'与换算,但未教'怎么从真机/数据手册反推该设多大 lag'的排查路径(练习 2 涉及但正文未展开方法),对真要落地的读者是个小缺口
- UniLab'非声明式=你得自己保证顺序正确'(L443)反复强调风险,但未给一个'手写 _compute_obs 的自检清单'(如:每项加噪了吗?命令进 actor 了吗?critic 那路是干净值吗?切片顺序与部署一致吗?)——把散落各处的告诫收拢成一张清单会更可操作

**notes**：本章经实跑全面验证,准确度与教学性俱为范本级:三框架(mjlab1.4.0/IsaacLab2.3.2/UniLab)所有 load-bearing 的 API 名、字段、源码路径、方法学逐项核实通过,未发现任何编造、报错或不符。最强印证来自 mjlab rough G1 实跑直接打出 group 名 actor/critic 且 critic 仅多 foot_height 特权项(活体重现章节的非对称设计),以及 UniLab {'obs':49,'critic':52} 与 codebox 逐字一致。三处'已核实纠正'的反编造论断(JointEffort 无 use_default_offset、DiffIK 字段、noise 字段名)全部成立。三框架训练均一次跑通无 NaN。engineeringFindings 全为正面确认,无需主控修正 .tex;optimizations/pedagogyGaps 均为锦上添花的可选增补,非错误。所有路径为绝对/模块相对,已与远程实际路径对齐(UniLab 真实根 /root/gpufree-data/src/UniLab,源码在 src/unilab/ 下,章节用的 base/... envs/... 相对引用正确)。


---

## prac_privileged

**服务器**：gpufree (RTX 4090, mjlab 1.4.0 / rsl-rl-lib 5.2.0 / Isaac Lab 2.3.2 / UniLab)

**实跑命令（14）**
- ssh gpufree nvidia-smi + list-envs → 连接OK，RTX4090 24GB；mjlab 12 个任务，含 Mjlab-Velocity-Rough-Unitree-Go1（章节 Quick Start 用的就是它，存在）
- mjlab train Mjlab-Velocity-Rough-Unitree-Go1 --num-envs 64 --max-iterations 2 (WANDB_MODE=disabled) → 成功跑通，1480→2376 steps/s，value loss 0.023/0.029 正常，无报错
- import mjlab.envs.mdp → projected_gravity/height_scan/base_lin_vel/builtin_sensor 存在；foot_contact_forces/friction_coefficients/contact_forces 在全局 mdp 不存在
- import mjlab.tasks.velocity.mdp → foot_contact_forces/foot_contact/foot_height/foot_air_time 均存在（章节 Quick Start 的 critic term 名在 velocity 任务命名空间下是真的）；friction_coefficients/friction_coeffs 全仓不存在
- 读 mjlab tasks/velocity/velocity_env_cfg.py → 真实任务结构=actor 组(enable_corruption=True)+critic 组(继承 actor+foot_contact/foot_contact_forces/foot_height/foot_air_time, enable_corruption=False)，与章节非对称 AC 叙事完全吻合
- probe mjlab.rl → 配置类是 RslRlModelCfg（非 RslRlMLPModelCfg），有 obs_normalization/cnn_cfg/rnn_type 字段；RslRlOnPolicyRunnerCfg 有 obs_groups 字段（确认）
- probe ObservationGroupCfg 字段 → terms/concatenate_terms/enable_corruption/history_length 全部存在（确认章节）；位置在 mjlab.managers.observation_manager（非章节隐含的 manager_term_config）
- grep Isaac Lab isaaclab_rl/rsl_rl/rl_cfg.py + anymal_c/g1 rsl_rl_ppo_cfg.py → 真实用统一 RslRlPpoActorCriticCfg(actor_hidden_dims=[512,256,128], critic_hidden_dims=..., actor_obs_normalization, critic_obs_normalization)，无 RslRlMLPModelCfg，无 per-model RunnerCfg(actor=,critic=)
- Isaac Lab train Isaac-Cartpole-v0 --headless --max_iterations 2 → 成功跑通，value loss 正常；运行时 warn: obs_groups 缺 policy/critic 键时回退用 policy set（确认 routing 机制，键名是 policy 非 actor）
- grep isaaclab_rl exporter.py → export_policy_as_onnx(policy, path, normalizer=None, filename, verbose) + _OnnxPolicyExporter + normalizer 烘焙(x=self.normalizer(x))，与章节 1232 行声明逐字一致
- UniLab train go2_joystick_flat ppo mujoco max_iterations=1 → 成功跑通，583 steps/s，critic MLP 输入 52 维，value loss 0.0157 正常
- grep UniLab src/unilab/base/observations.py + base.py → split_obs_dict(16行)/obs_groups_spec(base.py:165)/get_obs_dims 真实存在；go2w rough.py 真实 critic=concat([linvel,gyro,...]) 即特权 base linvel 前置（确认章节 UniLab 叙事）
- grep mjlab tracking config/g1/env_cfgs.py → has_state_estimation:bool=True，not has_state_estimation 时移除 [motion_anchor_pos_b, base_lin_vel]（章节 1441 行声明完全正确）
- grep UniLab algos/torch/hora/distill.py → adapt_tconv/priv_info/ProprioAdaptTConv 存在，teacher 主干冻结、只训 adapt_tconv（确认章节 HORA 蒸馏=RMA phase-2 叙事）

**工程核对（是这样/不符）**
- [严重·sec:rlmc-priv-rslrl4 + 行40-42 pitfall + 行587-599 代码框] 章节把『RSL-RL 4.0 解耦=actor/critic 各一套 RslRlMLPModelCfg、RslRlOnPolicyRunnerCfg(actor=,critic=)』当作 isaaclab_rl 的 API——实跑证伪：装的 Isaac Lab 2.3.2/rsl-rl-lib 5.2.0 真实仍用统一 RslRlPpoActorCriticCfg(actor_hidden_dims=[512,256,128], critic_hidden_dims=...)，没有 RslRlMLPModelCfg、没有 per-model RunnerCfg。那套 per-model 结构其实是 mjlab 的 RslRlModelCfg(actor=,critic=)。章节把两框架 API 张冠李戴，还把 Isaac Lab 当前 API 误标成『旧版≤3.x』。
- [中·行47/588/1224/1587] 章节多处称 RSL-RL『>=4.0』『实测 4.0』，实测 rsl-rl-lib=5.2.0；版本号本身偏旧（不影响 DistillationRunner/OnPolicyRunner 都在的结论，但『4.0』描述不准）。
- [中·Quick Start 行86-91/612-617] 章节用 gym.make('Mjlab-Velocity-Rough-Unitree-Go1').reset() 取 obs['actor']/obs['critic'] 做维度自检——mjlab 不走 gymnasium 注册（用 mjlab.tasks.registry 的 load_env_cfg/load_rl_cfg），该 gym.make 写法跑不起来；Isaac Lab 半行 gym.make(...).observation_manager.compute() 同理是简写、非可直接执行脚本。
- [已被章节充分 \pz 对冲，但仍记] Quick Start/映射表用的 friction_coefficients/friction_coeffs term 在 mjlab 全仓不存在；真实 velocity 任务的特权 term 是 foot_contact/foot_contact_forces/foot_height/foot_air_time（这些确实存在于 tasks.velocity.mdp）。即章节 Quick Start 的『摩擦真值进 critic』是想象示例，真任务用接触/足高类特权。
- [正面] mjlab 非对称 AC 的核心声明全部坐实：ObservationGroupCfg 有 enable_corruption/concatenate_terms/history_length；真实 velocity 任务正是 actor(开噪)+critic(继承+特权,关噪)；RslRlOnPolicyRunnerCfg 有 obs_groups。训练 1480-2376 steps/s 正常。
- [正面] ONNX exporter 声明逐字正确：export_policy_as_onnx(policy, path, normalizer=None, ...) + normalizer 烘焙进 forward；empirical_normalization 已废弃→actor/critic_obs_normalization 也正确。
- [正面] UniLab 整节最准：split_obs_dict/obs_groups_spec/get_obs_dims 真实存在于 base/observations.py+base.py；go2 env 真实 critic 拼 linvel 特权；HORA 蒸馏(adapt_tconv 冻结 teacher 主干、priv_info 必需)与 has_state_estimation 移除 base_lin_vel+motion_anchor_pos_b 全部坐实。仅 TemporalAdaptationEncoder 实名为 ProprioAdaptTConv（概念对、类名小差）。

**优化点 / 高级技巧**
- sec:rlmc-priv-rslrl4 应按框架分开讲：mjlab 用 per-model RslRlModelCfg(actor=,critic=,obs_normalization)（实测字段含 cnn_cfg/rnn_type）；Isaac Lab 用统一 RslRlPpoActorCriticCfg(actor_hidden_dims/critic_hidden_dims/actor_obs_normalization/critic_obs_normalization)+RunnerCfg.obs_groups。当前合写成一套『RSL-RL 4.0』会让读者照搬 RslRlMLPModelCfg 在 Isaac Lab 上直接 ImportError。
- obs_groups 键名按框架不同应点明：Isaac Lab/rsl_rl 算法侧 set 名是 policy/critic（实跑 deprecation warning 实锤：缺 policy/critic 键会回退用 policy set），mjlab 用 actor/critic。章节行594 在『isaaclab_rl』语境里写 {'actor':['actor']} 是 mjlab 键名，会误导。
- 可补一条实测调试技巧：Isaac Lab 跑训练时 rsl_rl 会打印『obs_groups 必须含 policy/critic 键，否则 critic 复用 policy set，将来版本移除该回退』——这正是章节维度自检/routing 小节最好的真实佐证，且能教读者从 warning 反查 routing 是否生效。
- mjlab 维度自检宜改用真实 API：from mjlab.tasks.registry import load_env_cfg 构造 env，或直接训练日志里看 actor/critic 网络的 in_features（实测 UniLab/mjlab 训练启动都会打印网络结构，512/256/128，可直接读出特权维差），比 gym.make 的伪代码更可落地。
- 可补 mjlab ObservationTermCfg 的 delay_min_lag/delay_max_lag/delay_hold_prob 等真实延迟字段（实测存在）——章节 checklist 第10条提『delay lag 换算物理时间』却没给出 mjlab 真有这些字段，正好补成可操作配置。

**教学缺口（只讲代码没教工程之处）**
- 『从零搭起』缺口：章节给的 mjlab/Isaac Lab 配置都是节选代码框，但没教读者怎么从框架自带的真实任务(mjlab tasks/velocity/config/go1/env_cfgs.py、Isaac Lab .../velocity/config/anymal_c)出发改非对称——这才是工程上真实路径(复制现成 cfg→改 critic_terms→改 hidden_dims)。读者照代码框从空白写反而踩 import 路径/类名坑(如 ObservationGroupCfg 真实在 managers.observation_manager)。
- 版本核对教学缺口:章节开篇让读者 pip show rsl-rl-lib 确认>=4.0,但没教读者如何辨别『我装的 Isaac Lab 用统一 cfg 还是 per-model cfg』——而这恰恰决定该用 actor_hidden_dims 还是 RslRlModelCfg。应教一句:打开你项目里现成 rsl_rl_ppo_cfg.py 看它用哪个类,而非照书假设。
- 维度自检只给了 gym.make 伪代码,没教读者在 mjlab 真实工作流里怎么打印 actor/critic 维度(实测:训练启动日志直接打印两网络 in_features;或从 obs_groups_spec/env_cfg 读)。『怎么排错』在这里落空——读者跑章节代码会先撞 ModuleNotFoundError: gymnasium。
- RMA/蒸馏/extreme-parkour 三大节全是自定义教学代码(EnvironmentEncoder/collect_adaptation_dataset/warm-start 部分加载),没有任何一段映射到框架里真实可跑的入口。对比之下 UniLab HORA 节给了真实 scripts/train_hora_distill.py + algos/torch/hora/distill.py 的真实类名/字段(adapt_tconv 冻结逻辑),教学价值明显更高——RMA/蒸馏节也应指一条『在 RSL-RL DistillationRunner 或 UniLab HORA 上真实跑通』的最小路径,而非只给悬空 nn.Module。
- 章节反复用 \pz『以本地版本为准』对冲 term 名/维度,工程上正确;但对最关键的 RSL-RL 配置类结构(RslRlMLPModelCfg vs RslRlPpoActorCriticCfg)没有对冲、且给错——这正是读者最会照抄、最会一抄就报错的地方,反而该配一个版本判别 checklist。

**notes**：三框架训练均实跑成功(mjlab Go1 velocity 1480-2376 steps/s、UniLab go2 583 steps/s、Isaac Lab Cartpole 均 value loss 正常无报错)。章节工程主干(非对称 actor/critic 分组+enable_corruption、UniLab obs_groups_spec/split_obs/critic 拼 linvel、HORA=RMA phase-2、mjlab has_state_estimation 移除 base_lin_vel、ONNX exporter 签名+normalizer 烘焙、DistillationRunner/OnPolicyRunner 存在)绝大多数被实跑坐实,且 \\pz 对冲到位。唯一硬伤:sec:rlmc-priv-rslrl4 把 mjlab 的 per-model RslRlModelCfg 结构误当成 isaaclab_rl『RSL-RL 4.0』的 RslRlMLPModelCfg,而装的 Isaac Lab 2.3.2 真实仍是统一 RslRlPpoActorCriticCfg(actor_hidden_dims/critic_hidden_dims)——读者照该代码框在 Isaac Lab 上会 ImportError。次要:RSL-RL 实为 5.2.0(非『4.0』)、mjlab 维度自检的 gym.make 伪代码跑不通(mjlab 不走 gymnasium)。建议主控:把 RSL-RL 4.0 节按 mjlab/Isaac Lab 两套 API 拆开重写,并把 obs_groups 键名(policy vs actor)按框架订正。未改任何 .tex/.bib。

---

## prac_visuomotor

**服务器**：gpufree (RTX 4090, mjlab 1.4.0 + Isaac Lab 2.3.2)

**实跑命令（12）**
- ssh gpufree nvidia-smi + list-envs → 连通; mjlab 真任务12个, 含 Mjlab-Lift-Cube-Yam-Depth/-Rgb 与 Mjlab-Multi-Cube-Seg-Yam(章节未提及这些真·视觉任务存在)
- from mjlab.sensor import CameraSensorCfg → OK; 字段全集=['name','camera_name','parent_body','pos','quat','fovy','width','height','data_types','use_textures','use_shadows','enabled_geom_groups','orthographic','clone_data'](比章节列的多5个: use_textures/use_shadows/enabled_geom_groups/orthographic/clone_data)
- CameraSensorData → rgb=[N,H,W,3]uint8, depth=[N,H,W,1]float32(与章节逐字一致), segmentation=[N,H,W,2]int32(章节未给seg形状)
- WANDB_MODE=disabled MUJOCO_GL=egl train Mjlab-Lift-Cube-Yam-Depth --num-envs 64 --max-iterations 2 → 跑通无报错, 5206 steps/s, MUJOCO_GL=egl 确实生效(章节Quick Start用占位符<你的视觉任务>, 此真任务可直接替换)
- train Mjlab-Lift-Cube-Yam-Rgb --max-iterations 1 → 跑通, obs显示 camera_d405_rgb (3,32,32) CHW, 1282 steps/s, 且带 geom_rgba DR项(印证章节'mjlab视觉DR=基础颜色替换')
- train Mjlab-Velocity-Flat-Unitree-Go1 → 1881-2515 steps/s (非视觉loco对照; 但与Lift-Depth任务不同, 不能直接验证'渲染5-20x'说法)
- isaaclab.sh -p (app launched) import TiledCameraCfg/PinholeCameraCfg → OK; OffsetCfg字段=[pos,rot,convention]逐字符合; PinholeCameraCfg含clipping_range/focal_length(印证Isaac把clip放相机配置, 与mjlab不同)
- 裸 isaaclab.sh -p -c 'import TiledCameraCfg' → ModuleNotFoundError: omni.usd (必须先AppLauncher启动Isaac Sim才能import相机配置类)
- grep mjlab tasks → 真实depth配置: CameraSensorCfg(camera_name='robot/camera_d405',height=32,width=32,data_types=('depth',)) 包裹MJCF相机(章节方式B); depth经ObservationTermCfg(func=manipulation_mdp.camera_depth)挂入obs组
- 读 mjlab camera_depth() 源码 → permute(0,3,1,2)+clamp(min_depth,cutoff)+/cutoff+clamp(0,1); 即章节preprocess_depth的真实生产版(但归一化是 d/far 非 (d-near)/(far-near), 且无NaN/inf处理)
- grep isaaclab camera.py:397 + tiled_camera.py:145 → 'A camera was spawned without the --enable_cameras flag...' 报错串逐字存在(章节引用准确)
- CameraSensorCfg(data_types=('lidar',)) → ValueError: Invalid camera data types {'lidar'}. Valid: {segmentation,depth,rgb} (data_types在构造期即校验)

**工程核对（是这样/不符）**
- [是这样] mjlab CameraSensorCfg API 与 import 路径(from mjlab.sensor)、字段 camera_name/parent_body/pos/quat/fovy/width/height/data_types、输出 depth=[N,H,W,1]f32 / rgb=[N,H,W,3]uint8 —— 全部实测吻合(prac_visuomotor.tex:399-418, :72)
- [是这样] mjlab CameraSensorCfg 无 clip/normalize 字段, 裁剪归一化在预处理做 —— 源码确认 camera_depth() 才做 clamp/归一化(prac_visuomotor.tex:416-417)
- [是这样] Isaac TiledCameraCfg.OffsetCfg=[pos,rot,convention]、PinholeCameraCfg 含 clipping_range —— app内import实测吻合, 章节配置代码API准确(prac_visuomotor.tex:349-367)
- [是这样] Isaac 漏 --enable_cameras 抛 RuntimeError(非黑屏), 报错串逐字命中源码 camera.py:397(prac_visuomotor.tex:76, :1113)
- [是这样] MUJOCO_GL=egl 无显示器服务器渲染生效 —— Mjlab-Lift-Cube-Yam-Depth 实跑5206 steps/s 跑通(prac_visuomotor.tex:97)
- [是这样] 章节 preprocess_depth 的 HWC→permute→clamp→归一化流程 = mjlab 生产函数 camera_depth() 的真实实现, 顺序完全一致(prac_visuomotor.tex:517-527)
- [不是这样/不全] 章节多处暗示'mjlab视觉只是配置能力, 无现成视觉任务可跑', Quick Start用占位符<你的视觉任务>; 实测 list-envs 有 Mjlab-Lift-Cube-Yam-Depth/-Rgb、Mjlab-Multi-Cube-Seg-Yam 三个真·视觉任务可直接跑(虽是Yam机械臂manip而非loco)
- [不全] 章节列的 CameraSensorCfg 字段缺 use_textures/use_shadows/enabled_geom_groups/orthographic/clone_data 5个真实字段; 其中 use_shadows/use_textures/enabled_geom_groups 有'同场景所有相机必须一致'的硬约束(源码docstring), 章节未提
- [偏差] 章节归一化公式 (d-near)/(far-near)(eq:rlmc-vm-norm) 与 mjlab 生产实现 d/cutoff 不同(后者忽略near偏移); 两者都对但读者照搬章节公式做部署预处理会与mjlab内置obs不一致
- [偏差] 章节'渲染开销约纯物理5-20倍'(prac_visuomotor.tex:79)在本服务器无法用现成任务验证: Depth任务(5206)比Velocity-loco(1881)还快, 因二者任务不同(Yam manip vs 粗糙地形loco), 非同任务加/不加相机对照
- [空白未证] extreme-parkour depth_backbone.py(convnet→GRU(32,512)→Linear(512,34))源码不在本服务器, 无法实跑核验(章节注释称已跨源核对, 本次仅记为未在gpufree复核)

**优化点 / 高级技巧**
- Quick Start(sec:rlmc-vm-quickstart)可把占位符 <你的视觉任务> 直接换成实测可跑的 'WANDB_MODE=disabled MUJOCO_GL=egl train Mjlab-Lift-Cube-Yam-Depth --env.scene.num-envs 64 --agent.max-iterations 2' —— 给读者一条真能复制粘贴、5206 steps/s 立即出结果的命令, 而非自己找任务
- 应补一条高发坑: Isaac 的相机配置类(TiledCameraCfg)不能在裸Python里import(拉起omni.usd报ModuleNotFoundError), 必须先 AppLauncher(enable_cameras=True) 启动Isaac Sim才能import —— 这正是想'快速查API'的读者第一个撞墙处, 章节完全没提
- 预处理一节(sec:rlmc-vm-preprocess)可直接引用 mjlab 生产函数 mjlab.tasks.manipulation.mdp.observations.camera_depth 作为'真实世界的preprocess_depth', 并指出它用 d/cutoff 归一化、且未处理NaN/inf —— 比纯手写示例更有说服力, 也暴露'生产代码省了无效像素处理'这一可讨论点
- 可补 mjlab 把 depth 接入策略的真实姿势: ObservationTermCfg(func=camera_depth, params={cutoff_distance:..}) + 单独 ObservationGroupCfg(concatenate_terms=True) —— 这才是章节checklist'depth obs挂进actor group'在mjlab的具体落地, 现章节只给了独立的preprocess函数没给接线
- CameraSensorCfg 的 use_textures/use_shadows/enabled_geom_groups '同场景所有相机必须一致' 硬约束值得作为pitfall: 多相机(sec:rlmc-vm-multicam)若前视/下视相机这三项设不同会在构造期报错, 章节多相机一节未警示
- depth-only 任务建议用 enabled_geom_groups 排除装饰几何(真实Yam任务用(0,3))+ use_shadows=False 省渲染 —— 可作为'depth渲染性能优化'的具体技巧补进 sec:rlmc-vm-resolution

**教学缺口（只讲代码没教工程之处）**
- 全章给了大量配置/网络代码, 但从未展示一次'端到端真能跑起来的视觉任务'——读者学完不知道按哪条命令能看到depth在动; 而服务器上 Mjlab-Lift-Cube-Yam-Depth 一条命令即可, 这种'从零跑通第一帧'的肌肉记忆缺位(Quick Start全是占位符)
- 预处理(preprocess_depth)与'如何把depth喂进ObservationManager'是断开的: 章节教了独立函数, 也在checklist要求'depth进actor group', 但中间'怎么把这个函数注册成obs term'的接线一步(mjlab的ObservationTermCfg(func=...)模式)从未演示, 读者照着学不会从零搭一个视觉obs组
- 归一化只给了 (d-near)/(far-near) 一个公式, 没告诉读者真实框架(mjlab)用的是更简单的 d/far、也没讲'为什么生产代码敢省near偏移/省NaN处理'——这恰是'教会工程判断'(何时可简化)的好机会, 现在只是给了一个学院派公式
- '渲染比物理慢5-20倍'是贯穿全章的资源论证(决定num_envs砍到几百、决定为何用蒸馏), 但章节没教读者'怎么自己测这个倍数'(同任务开/关相机对比steps/s); 给个可操作的测量配方比直接抛一个区间更能教会举一反三
- Isaac相机API无法裸import这一真实工程障碍没讲, 导致章节'写出TiledCameraCfg'的验收(sec:rlmc-vm-goals第1条)读者一上手就会卡在ImportError却不知所以——少了'相机API必须在SimApp内'这条关键前置认知

**notes**：总体: 章节的 load-bearing API 事实(mjlab CameraSensorCfg 字段与输出形状、Isaac TiledCameraCfg/OffsetCfg/clipping_range、--enable_cameras 报错串、MUJOCO_GL=egl、preprocess 流程)经实跑/源码逐项核验, 准确度高, 三轮复核+核销卓有成效。最大问题不在'对不对'而在'教不教得会': 通篇占位符、无一条可复制即跑的真命令, 而服务器上 mjlab 明明有 Depth/Rgb/Seg 三个现成视觉任务(5206/1282 steps/s 实测跑通)。次要硬伤: CameraSensorCfg 字段列表少5个(含有'全场景一致'硬约束的3个)、归一化公式与生产实现(d/far)有出入、'渲染5-20x'在现有任务下无法对照验证。extreme-parkour depth_backbone 源码不在本机, 该条未在 gpufree 复核。未改任何 .tex/.bib(只读核验)。

---

## prac_multimodal

**服务器**：gpufree (RTX4090; mjlab 1.4.0 / UniLab commit-99936e08 / Isaac Lab 2.3.2 装齐, ProtoMotions 未装)

**实跑命令（11）**
- list-envs → mjlab 12 个任务, 含 Mjlab-Tracking-Flat-Unitree-G1(本章 tracker 底座), 无 Go2(符合)
- mjlab/UniLab.venv/IsaacSim python -c 'import clip'/'import open_clip' → 三个环境全部 ModuleNotFoundError(Quick Start 第一步不可开箱即跑)
- find / -iname '*protomotion*' / train_agent.py → 全系统无 ProtoMotions(第二列所有命令本机无法实跑)
- WANDB_MODE=disabled train Mjlab-Tracking-Flat-Unitree-G1 --num-envs 64 --max-iterations 2 → ValueError: tracking 任务必须给 --registry-name 或 --env.commands.motion.motion-file(章节未给 mjlab 喂 motion 的命令)
- UniLab: train --algo ppo --task go2_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 → 成功, 打印 reward/tracking_lin_vel 等, 0.66s/iter(章节 train 命令形式正确)
- UniLab: train --algo ppo --task g1_motion_tracking --sim mujoco ... → 失败: huggingface_hub 下载 motion 资产报 'Cannot send a request, client closed'(g1_motion_tracking 需 HF motion 资产, 离线/无 mirror 跑不起)
- demo.py: dance→g1_motion_tracking, wallflip→g1_wall_flip_tracking, boxtracking→g1_box_tracking, 均 sim=motrix entry=eval(任务名真实; 但 demo 默认 motrix 非 mujoco)
- grep MotionData → joint_pos/joint_vel/body_pos_w/body_quat_w/body_lin_vel_w/body_ang_vel_w + fps, 与章节 line1010 NPZ 契约逐字一致
- grep RewardContext/_reward_fns → RewardContext 含 info:dict 字段, _reward_fns={...} 派发表在各 env(tracking.py:743 等), 调用 self._reward_fns[name](ctx); 章节 CALM 挂钩范式(line630-641)与 calm_style_reward(ctx)/ctx.info 完全成立
- grep hora/distill.py → class HoraDistillationTrainer(:298), adapt_tconv 才 requires_grad=True, teacher state 排除 adapt_tconv. 键; 章节 line752 RMA 蒸馏描述逐字属实
- ls isaaclab rsl_rl/train.py → 存在(Isaac Lab 在位; 但本章 ProtoMotions 走 IsaacGym 后端, 本机无法验)

**工程核对（是这样/不符）**
- 不是这样(可跑性): Quick Start 第一步 `import clip; clip.load("ViT-B/32")` 号称'立即可复制'+'秒级', 但 clip/open_clip 在 mjlab/UniLab/IsaacSim 三环境全未安装 → 读者照抄即 ModuleNotFoundError; 章节缺一句 `pip install git+https://github.com/openai/CLIP` 或 `open_clip_torch` 的安装前置
- 不是这样(本机不可验): 整个 ProtoMotions 第二列(Quick Start 第二步 inference_agent.py、CALM train_agent.py/inference_agent.py、MaskedMimic)在 gpufree 上无 ProtoMotions, 全部无法实跑 — 这些命令是据文档'已核'而非实跑; 与 '三框架齐全' 的设定不符(ProtoMotions 缺装)
- 不是这样(命令不完整): mjlab Mjlab-Tracking-Flat-Unitree-G1 实跑直接报错, 必须显式 `--env.commands.motion.motion-file /path.npz` 或 `--registry-name org/motions/...`; 本章把该 tracker 当'发动机'反复引用, 却从未给出 mjlab 侧喂 motion 文件的命令(只在 UniLab/ProtoMotions 侧给了 motion-file), 读者跑不通 mjlab tracker
- 是这样(强): UniLab 第三列高度可信 — go2_joystick_flat 实跑 1 iter 成功; MotionData 6 键 NPZ 契约、RewardContext.info、_reward_fns 派发表、HoraDistillationTrainer 的 adapt_tconv-only 解冻 全部与章节逐字吻合; demo dance/wallflip/boxtracking 任务名真实
- 部分不符(小): demo 实际 sim=motrix(MotrixSim) 且 entry=eval, 章节 line108 `uv run demo dance` 任务名对, 但读者会以为是 mujoco; g1_motion_tracking 训练需从 HF 拉 motion 资产, 章节只在 demo 注释提了 HF_ENDPOINT, Quick Start 的 train 命令(line110)未标注同样依赖 HF, 离线必失败

**优化点 / 高级技巧**
- Quick Start 补 CLIP 安装前置与环境隔离: 明确 `pip install ftfy regex git+https://github.com/openai/CLIP.git`(或 open_clip_torch)且应装进'生成侧'env 而非 tracker env(正与本章 sec:rlmc-mm-setup 的双环境主张呼应), 否则第一步即挂
- 给 mjlab tracker 一条真能跑的最小命令: 例如先 `csv_to_npz`/下载一个 LAFAN motion 落成 npz, 再 `train Mjlab-Tracking-Flat-Unitree-G1 --env.commands.motion.motion-file xxx.npz`; 当前章节通篇靠 mjlab tracker 当执行层却无可跑入口, 是最该补的工程缺口
- UniLab g1_motion_tracking 加离线/镜像说明: 标注它会从 HF 拉 motion 资产, 给 `HF_ENDPOINT=https://hf-mirror.com` 或预下载到本地 + 指定本地 motion 路径的做法; 并说明 demo 默认 motrix、要 mujoco 需 `--sim mujoco`(或本机无 MotrixSim 时如何回退)
- 可补'判别器与 PPO learner 同步更新'的落地细节: 章节 CALM-into-UniLab 说判别器'在 GPU learner 侧与 PPO 同步更新', 但 UniLab 是 collector/learner 异构(APPO), 自建 ConditionalAMPDiscriminator 要塞进哪个进程、d_fake 如何回填 ctx.info 跨进程传, 是真正的难点, 现仅一句带过
- 性能/调参可补: mjlab/UniLab 实跑 steps/s 与单 iter 时间(UniLab go2 0.66s/iter@16envs)给个量级基准, 帮助读者判断'训练是否正常起'; 以及 num_envs 在 4090(24G) 上 4096 是否会 OOM 的经验值

**教学缺口（只讲代码没教工程之处）**
- 只讲代码没教'怎么从零搭': Quick Start 第一步默认 clip 已装, 但没教'装在哪个 env、和 tracker env 为何要隔离、装失败(ftfy/regex 缺)怎么办'; 与本章自己强调的'生成/跟踪双环境'主张脱节, 教学上自相矛盾
- ProtoMotions 第二列全是'示意命令'缺可执行闭环: CALM/MaskedMimic 大段贴 dataclass 命令与配置片段, 但未教读者'如何确认本机 ProtoMotions 版本、experiment 文件实际 key 怎么查、--motion-file 的 .pt 怎么从 AMASS+BABEL 打包出来' — 堆了命令却无法举一反三到自己的数据
- mjlab tracker'喂什么 motion'整章空白: 反复说 tracker 是发动机、要'烧燃料', 却没有任何一处教 mjlab 侧 motion 文件从哪来/什么格式/怎么传 → 读者拿到的是概念闭环而非工程闭环
- 排错教学偏理论: 故障排查手册条目(CALM disc/accuracy、穿地、帧率)都对, 但缺'命令真跑不起来'这类一线错误的排查(ModuleNotFoundError clip、HF 下载超时、mjlab tracking 缺 motion-file 报错), 而这恰是初学者第一个小时最常撞的墙
- CALM/MaskedMimic/TextOp 的概念代码('概念性实现')未点明'本机不可直接跑、需自己接 ProtoMotions/UniLab': 读者易误以为贴上去就能用; 应像 UniLab 列那样明确'哪些是可跑的、哪些是需自实现的脚手架'(UniLab 列其实标注得很好, 可作其他两列的范本)

**notes**：本章三框架对比的"概念-工程接线"质量很高,尤其UniLab第三列(NPZ 6键契约/RewardContext.info/_reward_fns派发表/HORA的adapt_tconv-only蒸馏)经实跑与读源码逐字证实,作者据服务器实装填写,可信度极高。核心问题集中在"可开箱跑"层面而非"讲错":(1)clip未装致Quick Start第一步必挂;(2)ProtoMotions本机未装,第二列全部不可验(与'三框架齐全'设定有出入,实为mjlab+UniLab+IsaacLab,ProtoMotions缺);(3)mjlab tracker通篇当执行层却无喂motion的可跑命令。无任何API名编造或参数错误被发现;论文归属/版本号未在本轮重核(属前几轮职责)。未改任何.tex/refs.bib/part.tex。建议主控据engineeringFindings第1-3条补三处可跑入口与安装前置,据pedagogyGaps把UniLab列"可跑vs需自实现"的标注范式复制到另两列。

---

## prac_manipulation

**服务器**：gpufree (RTX 4090, mjlab 1.4.0 / Isaac Lab 2.3.2 / UniLab)

**实跑命令（13）**
- ssh gpufree 'list-envs' → 确认 4 个 YAM 变体真实存在(Mjlab-Lift-Cube-Yam/-Depth/-Rgb/Mjlab-Multi-Cube-Seg-Yam), 与第319-331行表格逐字符吻合; 无 mjlab.__version__
- WANDB_MODE=disabled train Mjlab-Lift-Cube-Yam --env.scene.num-envs 64 --max-iterations 2 → 成功跑通, 1789→4306 steps/s, value loss≈0.02, entropy≈9.9, 管线 actor/critic 创建+rollout 无 shape 崩(印证 Quick Start 命令结构有效)
- train 输出 Actor/Critic Model → 二者 in_features 均=29(actor 输出=7), 与章节宣称 obs=27(第359-394行)矛盾
- python 编译 YAM MJCF → nq=8 nu=7 njnt=8 neq=1; 关节为 joint1-6+left_finger+right_finger → 揭示 obs=29 真因(joint_pos/vel 各含8个被动耦合指关节, 非7)
- grep yam.xml:182 → equality joint1=left_finger joint2=right_finger polycoef='0 -1 0 0 0' 与第340行引用逐字符吻合
- python introspect DifferentialIKActionCfg → 15 字段(entity_name/actuator_names/frame_type='body'/frame_name/use_relative_mode=True/delta_pos_scale=1.0/delta_ori_scale=1.0/damping=0.05/max_dq=0.5/position_weight/orientation_weight/joint_limit_weight=0.0/posture_weight=0.0/posture_target) 与第902-919行全部吻合
- grep yam_constants.py:222 → YAM_ACTION_SCALE[n]=0.25*e/s(e=effort_limit,s=stiffness) 与第454行公式吻合; 仅 left_finger 驱动
- cat rewards.py:20-42 staged_position_reward → reach_error=sum(square(ee-obj)); reaching=exp(-/std**2); return reaching*(1.0+bringing) 与第510-518行代码走读逐行吻合
- grep config/yam/rl_cfg.py → _VISION_CNN_CFG output_channels=[16,32] kernel_size=[5,3] stride=[2,2] spatial_softmax=True temperature=1.0 模型 SpatialSoftmaxCNNModel, 与第1141行全部吻合
- python introspect CameraSensorCfg(mjlab.sensor.camera_sensor) → 字段 name/camera_name/parent_body/pos/quat/fovy/width/height/data_types/use_textures/use_shadows/enabled_geom_groups/orthographic, 印证第1171-1179行(无 cutoff_distance/normalize)
- cat lift_cube_env_cfg.py:26-58 → 5 actor_terms(joint_pos/joint_vel/ee_to_cube/cube_to_goal/actions)+critic_terms={**actor_terms} 与第359-389行吻合
- cat config/yam/env_cfgs.py → reaching_std=0.2 bringing_std=0.3, resampling_time_range=(4.0,4.0); 与章节示例值(σreach≈0.15/σbring≈0.2, resample(5,5))不完全一致
- ls UniLab/conf/ppo/task/ → allegro_inhand(_grasp)/sharpa_inhand(_grasp) 真实存在, 印证第635行 in-hand 任务名

**工程核对（是这样/不符）**
- [符合] Quick Start/smoke train 命令真实可跑: Mjlab-Lift-Cube-Yam 64 env 2 iter 成功, steps/s 与 loss 正常, 管线接线(actor/critic/rollout)全通——章节'证明整条链是通的'目标兑现
- [符合] 4 个 YAM 变体 task ID(第319-331行)、staged reward 乘法门控源码(第510-518行)、YAM_ACTION_SCALE=0.25*e/s(第454行)、DiffIK 全部15字段及默认值(第902-919行)、视觉CNN配置(第1141行)、CameraSensorCfg字段与导入路径(第1171行)、equality polycoef'0 -1 0 0 0'(第340行)、5个obs term+critic对称(第359-389行)——均逐项实跑/源码核实吻合
- [不符-数值bug] 第359/387/394行宣称 actor/critic obs 维度=27(7+7+3+3+7)并要求读者打印 actor_obs.shape 核对=27; 实测网络 in_features=29(nq=8: joint_pos/joint_vel 各含被动耦合 right_finger)。章节把'驱动DOF=7(nu)'误用到 joint_pos/joint_vel 观测维(实为nq=8)。读者照章核对27会困惑——这是最该修的硬错
- [不符-配置值] 第427行 LiftingCommandCfg codebox 标注'字段经1.4.0源码核实'写 resampling_time_range=(5.0,5.0)、object_pose_range x=(0.3,0.35)/y=(-0.1,0.1); 实测 YAM 注册任务用 resampling=(4.0,4.0)、object_pose x=(0.2,0.4)/y=(-0.2,0.2)/含yaw=(-3.14,3.14)。声称'源码核实'但与实际注册值不符
- [不符-奖励表] 第458-472行正则reward表列 action_rate_l2/joint_acc_l2/joint_pos_limits(权重0.1); 实测YAM rewards 实为 action_rate_l2(-0.01)+joint_pos_limits(-10.0)+joint_vel_hinge(-0.01,带curriculum渐增到-1.0)。joint_acc_l2 不存在; joint_pos_limits 权重(-10)与章节(0.1)差两个数量级; 真实的 joint_vel_hinge 及其 reward_curriculum 章节完全未提
- [不符-小] 第454行称夹爪'接近-1闭合/+1张开,方向取决于关节定义'属对; 但 YAM 夹爪实为 crank gripper(DM4310 曲柄转线性, yam_constants.py:89), 非简单直驱指——章节'两指反向耦合'对, 但机构细节(crank)未提
- [符合] UniLab in-hand 任务(sharpa_inhand/allegro_inhand+_grasp)真实注册(第635行); mjlab 无 Go2、无独立 lift_cube 顶层模块(在 tasks.manipulation.lift_cube_env_cfg)——三框架对比的诚实性站得住

**优化点 / 高级技巧**
- obs维度教学应改为'先跑一次训练看 Linear in_features, 而非手算'——本章正栽在手算(27 vs 实际29)。可补一句:被动/耦合关节仍进 joint_pos_rel(按nq不按nu), 这是操作任务最易错的维度坑, 比locomotion更隐蔽
- 可补 mjlab 真实的 reward_curriculum 机制(joint_vel_hinge 权重按 step 从-0.01渐增到-1.0, env_cfgs.py): 这是比章节静态权重表更高级、且YAM实际在用的做法——'惩罚项随训练渐强'是操作收敛的实用技巧, 章节漏了
- 章节σ标定四步法(第527-531行)很好, 但应直接给出'YAM实测用 reaching_std=0.2/bringing_std=0.3'作锚, 而非只给推导出的0.15/0.2示例值——读者对照源码会发现对不上
- 可补:DR 在YAM是 startup 模式的三轴 fingertip friction(slide/spin/roll, 用abs+log_uniform, ranges如spin(1e-4,2e-2)), 比章节笼统的'手动设范围'具体得多, 且揭示了灵巧接触摩擦要分slide/spin/roll三向单独随机——这是操作DR的高级实践
- 训练命令实测 64 env 仅 ~1800-4300 steps/s(2 iter, 含编译预热), 章节第1008行引'YAM<5min解cube lifting'宜补一句吞吐量级与env数关系, 否则读者按64 env体感会觉得慢
- 建议补 episode 终止真相: YAM 实有 ee_ground_collision 终止(illegal_contact force>10N)+time_out, 章节第218行'固定基座永不摔倒/只有成功+超时终止'不完整——撞桌面也终止

**教学缺口（只讲代码没教工程之处）**
- obs维度小节(sec:rlmc-manip-obs)只'教'了手算7+7+3+3+7=27并让读者验证, 却未教'驱动DOF(nu)≠观测关节数(nq)'这一操作任务核心陷阱——恰恰算错。这是'只讲了表面算法没教会工程排查'的典型:正确教法应是'打印网络in_features, 若与手算不符就去查nq vs nu/被动关节'
- staged reward 教了乘法门控数学与单函数源码, 但未点破YAM实际是两个独立reward term(lift=staged + lift_precise=bring_object_reward std=0.05)叠加——读者按章节以为一个函数搞定, 实际工程里'粗门控+精定位'分两项是更稳的做法, 这个'为什么拆两项'没教
- 正则reward只堆了一张典型权重表, 没教'权重怎么定/为什么YAM用-10量级惩罚关节极限/为什么需要curriculum渐增'——读者拿到表也不会调参, 缺'从默认值出发怎么试'的工程方法
- DiffIK/OSC 字段列得很全且核实准确, 但全是'配置项罗列', 没教'从JointPos切DiffIK后训练崩了怎么排查'的完整闭环(只在pitfall给了body_name不一致一例)——读者会配不会救
- 视觉/privileged泄漏三层防线讲得好(静态/梯度/遮蔽), 是本章教学性最强处; 但CNN代码给的是'三层教学示例'而非mjlab默认两层——已诚实标注, 不算缺口
- '怎么从零搭一个新操作任务'始终没有端到端示范: 章节假定用现成YAM, 但目标(第132行)说要'调通四变体', 读者真要新建一个lift任务(改MJCF→配obs→定reward→跑通)缺一条主线串起来

**notes**：总体: 本章工程质量高, 实跑核实的~15项具体API/源码claim(task ID、staged reward代码、YAM_ACTION_SCALE公式、DiffIK15字段、视觉CNN配置、CameraSensorCfg、equality polycoef、obs term结构、UniLab任务名)绝大多数逐字符吻合, 训练管线真实可跑通, '三框架对比诚实性'站得住。最该修的硬伤=obs维度: 章节多处(第359/387/394行)称27并要求读者验证=27, 实测Actor/Critic网络 in_features=29, 根因是 nq=8(被动耦合right_finger进joint_pos/vel)而章节按 nu=7 手算——既是数值错也是教学盲点(没教nu≠nq陷阱)。次要: LiftingCommandCfg codebox声称'源码核实'的resampling(5,5)/object_pose范围与实际YAM注册值(4.0/含yaw)不符; 正则reward表的joint_acc_l2不存在、joint_pos_limits权重差两数量级、漏了真实在用的joint_vel_hinge+reward_curriculum。这些都是据实可改的点, 不影响主线正确性。所有发现均来自gpufree实跑/源码introspection, 未改任何.tex。

---

## prac_humanoid_wbc

**服务器**：gpufree (RTX 4090; mjlab 1.4.0 @ /root/gpufree-data/envs/mjlab, Isaac Lab 2.3.2 @ /root/isaaclab, UniLab @ /root/gpufree-data/src/UniLab)

**实跑命令（11）**
- list-envs → 确认 mjlab 仅 12 任务，含 #7 Mjlab-Tracking-Flat-Unitree-G1 + #8 -No-State-Estimation 变体，无 Go2 (与章节 line44/65/88 一致)
- train Mjlab-Tracking-Flat-Unitree-G1 --help → 确认 --registry-name 与 --env.scene.num-envs/--env.commands.motion.motion-file 旗存在
- train Mjlab-Tracking-Flat-Unitree-G1 (无 registry) → 报错 ValueError: provide --registry-name OR --env.commands.motion.motion-file /path/motion.npz；即章节 Quick Start(line67-70)的占位 registry 不能裸跑
- train Mjlab-Velocity-Flat-Unitree-G1 --num-envs 64 --max-iterations 2 → 成功，2570 steps/s，value loss~1.1，reward 上升 (locomotion 基线健康)
- UniLab find conf + 源码 → 确认真实任务名 g1_motion_tracking/g1_walk_flat/g1_flip_tracking/g1_climb_tracking/go2_joystick_flat 等；box_tracking.py 与 demo boxtracking/wallflip/dance 真实存在
- grep UniLab tracking.py → G1MotionTrackingEnv(line448)/G1MotionTrackingCfg(133)/MotionLoader(39)/_write_motion_anchor_transform(280)/_motion_anchor_pos_b(3)/_ori_b(6) 全部命中；obs 注释 command(2n)+anchor_pos_b(3)+anchor_ori_b(6) 与章节 line492 逐字一致
- grep g1_motion_tracking/mujoco.yaml → reward.scales 键 motion_global_root_pos/ori、motion_body_pos/ori/lin_vel/ang_vel、motion_joint_pos/vel 与章节 codebox(line482-491)逐项一致
- train g1_motion_tracking (无 HF_ENDPOINT) → 失败：huggingface_hub 下载 dance1_subject2_part.npz 网络错 (RuntimeError client closed)
- HF_ENDPOINT=https://hf-mirror.com train g1_motion_tracking → 成功跑通 1 iter，per-term reward/motion_* 全部正常打印 (证明 train 路径也依赖 HF)
- grep Isaac source → Isaac-Velocity-Flat/Rough-G1-v0 + Isaac-PickPlace-Locomanipulation-G1-Abs-v0 真实注册；另发现 Locomanipulation-G1-Abs-Mimic-v0/SteeringWheel/FixedBaseUpperBodyIK 等额外 G1 任务
- train Mjlab-Tracking-Flat-Unitree-G1 --env.commands.motion.motion-file <dummy.npz> → 越过 registry 报错进入仿真初始化(仅因 dummy NPZ 维度非法触发 CUDA assert)，证明本地文件旗真实可用

**工程核对（是这样/不符）**
- 是这样：UniLab 全套深度 API 引用全部实测命中——G1MotionTrackingEnv/G1MotionTrackingCfg/MotionLoader/MotionData/_write_motion_anchor_transform、motion_anchor_pos_b(3)+ori_b(6)、NPZ 键 fps/joint_pos/joint_vel/body_{pos,quat,lin_vel,ang_vel}_w(tracking.py/motion_loader.py)，章节 line477/492 obs 注释与源码逐字一致
- 是这样：章节 UniLab reward.scales codebox(line482-491)与 conf/ppo/task/g1_motion_tracking/mujoco.yaml 逐项匹配(root_pos/ori 0.5、body_pos/ori/lin_vel/ang_vel 1.0、joint_pos/vel 默认 0.0)；运行时按 reward/motion_* 分项打印
- 是这样：mjlab 任务表(list-envs)与 Isaac G1 任务 ID(grep)与章节完全吻合——Mjlab-Tracking-Flat-Unitree-G1、Isaac-Velocity-Flat/Rough-G1-v0、Isaac-PickPlace-Locomanipulation-G1-Abs-v0 均真实注册；mjlab 无 Go2、无动作专名任务，章节 line88 陷阱正确
- 不是这样(Quick Start 不可裸跑)：章节 mjlab Quick Start(line67-70)用 --registry-name your-org/motions/wave-hand 这一占位值，实测裸跑报 ValueError 要求真实 WandB registry 或本地 motion-file；读者照抄得到的是报错而非冒烟通过(WandB 需登录+真实 motion)
- 不是这样(UniLab Quick Start 缺网络前提)：章节 line82-84 把 HF_ENDPOINT=hf-mirror 提示只挂在 demo 行，但 train g1_motion_tracking 也需从 HF 拉 dance1_subject2_part.npz——无镜像实测网络报错，加镜像后才跑通；train 行同样需要该前提
- 是这样：mjlab Velocity 基线实测健康(64 env 2570 steps/s、value loss~1.1、reward 上升)，章节'建立在已调好的 locomotion 之上'的前提成立

**优化点 / 高级技巧**
- mjlab Quick Start 应补本地离线路径：报错信息明示 --env.commands.motion.motion-file /path/to/motion.npz 是无需 WandB/网络的替代，实测可越过 registry 进入仿真——章节通篇只给 --registry-name 占位值，应增列本地文件法(对无 WandB key 的读者才真能冒烟)
- UniLab train 冒烟行应把 HF_ENDPOINT=https://hf-mirror.com 前置到 train(不止 demo)，并提示首跑会自动下载 dance1_subject2_part.npz；或给出离线复用本地 NPZ 的做法(tracking.py:142 motion_file 默认指向 assets/motions/g1/*.npz，可换 gangnam_style/LAFAN 等)
- Isaac loco-manip 资产比章节所列丰富：除 Isaac-PickPlace-Locomanipulation-G1-Abs-v0 外，本版还注册了 Isaac-Locomanipulation-G1-Abs-Mimic-v0、Isaac-G1-SteeringWheel-Locomanipulation、Isaac-PickPlace-FixedBaseUpperBodyIK-G1-Abs-v0、-InspireFTP-Abs-v0——可作为 loco-manip 任务谱的更全数据点
- mjlab tracking 有 -No-State-Estimation 变体(list-envs #8)，与正篇 sim-to-sim/状态估计鲁棒性主题强相关，章节未提，可作'去状态估计冒烟对照'的一句话补充
- UniLab g1_motion_tracking 真实 reward 还含 action_rate_l2:-0.1 与 joint_limit:-10.0 两项正则(章节 codebox 用 ...省略)，且配套 std_* 容差字典(std_body_pos 0.3 等)——这正是章节反复强调的'每项 σ 容差'的现成实例，值得点名作为 ExBody2 σ 表的落地对照

**教学缺口（只讲代码没教工程之处）**
- Quick Start 三框架命令都'给了代码没教会跑通':mjlab 用占位 registry、UniLab train 缺 HF 前提——读者照抄三条里至少两条直接报错。教学应在每条命令旁标注'这条需要什么外部依赖(WandB 登录/HF 网络/本地 NPZ)、不满足时的报错长什么样、最小可离线替代是什么'，而非默认环境齐备
- mjlab tracking 的'参考动作从哪来'是全章 Quick Start 最大的从零搭建缺口:章节只说'经 WandB Motions registry'，但没教读者在没有 org/registry 时如何获得一个能跑的 motion(本地 NPZ 路径、用 scripts/motion/csv_to_npz.py 之类从 CSV 生成)——这正是初学者第一步就卡死的地方
- 章节大量给出'与框架无关的概念骨架'(PeriodicLocalFrame/ContactStage/PolicySwitcher/HierarchicalHumanoidPolicy)且明示非框架原生 API，但未给任一可在本仓库实跑的最小落地示例——读者无法验证自己写对没有。可至少把 UniLab 现成的 g1_motion_tracking(已实测可跑)作为'解耦 reward 的真实起点'，让读者在能跑的基线上改而非从空白概念码起
- 诊断层次表(line970-986)与故障排查手册(line1076)是纯文字 checklist，未与任一真实可观测量绑定:例如'Layer1 纯走稳'对应的就是实测的 Mjlab-Velocity-Flat-Unitree-G1(2570 steps/s/value loss~1.1 的健康基线)，可把'健康训练曲线长什么样'用真实数字锚定，教读者判读 steps/s 与各 loss

**notes**：未改任何 .tex/refs.bib/part.tex。本章三框架的"事实层"(任务名、API 类/方法名、reward 标量键、obs 布局、Isaac 任务 ID、mjlab 无 Go2/无动作专名)经实跑与源码 grep 高度准确，三轮复核质量好。真正暴露的问题集中在"可复现性/从零搭起"维度:mjlab Quick Start 用占位 WandB registry 不可裸跑(且未给已存在的本地 motion-file 离线路径)、UniLab train 冒烟缺 HF 镜像前提(只挂在 demo)。建议主控据 optimizations 的前两条补 Quick Start 的依赖标注与离线替代。所有训练只跑 1-2 iter 验证起步/报错/吞吐,未做满训。

---

## prac_legged_manip

**服务器**：gpufree (RTX 4090; mjlab 1.4.0 / Isaac Lab 2.3.2 / UniLab ~commit 99936e08)

**实跑命令（10）**
- mjlab list-envs → 确认 12 任务、无 Go2、无 Velocity-Flat-Go2(与正文/line35 一致, 正确)
- WANDB_MODE=disabled mjlab/bin/train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2 → 成功跑通, 2527 steps/s, value_loss 0.026 正常
- python -c 'from mjlab.envs import mdp; hasattr base_ang_vel/projected_gravity/joint_pos_rel/last_action' → 全 True, 证实 line688-695 观测是函数 正确
- python -c dataclasses.fields(JointPositionActionCfg) → 含 entity_name+actuator_names+scale+use_default_offset, 证实 line526/533 字段名正确
- UniLab train --algo ppo --task go2_arm_manip_loco --sim mujoco max_iterations=1 num_envs=16 → 成功跑通 575 steps/s, reward/object_distance=1.88、arm_collision、tracking_lin_vel 正常; critic in_features=79
- UniLab train --task go2_arm → 报错 'No owner config exists ... conf/ppo/task/go2_arm/mujoco.yaml'(章 line94/95/866 用的 go2_arm 不存在)
- cat conf/ppo/task/go2_arm_manip_loco/mujoco.yaml → 真实 reward.scales=object_distance/object_distance_l2/arm_collision+全套loco; 有 ik:块(damping/gain/dq_clip/orientation_mode), arm_action_scale=0.0; 有 arm_stage.freeze_arm_joints
- 读 src/unilab/envs/locomotion/go2_arm/manip_loco.py(1043行) → apply_action: arm=arm_dof_pos+act*arm_action_scale+ik.gain*dq_ik(IK残差); 23个reward全是loco+EE-reach无gripper/grasp; _compute_raw_obs 单步79维(…+ee_local_pos3+ee_goal_cart3+ee_error3+last_actions18); obs_groups_spec 返回{'obs','critic'}; _compute_obs actor add_noise=True/critic add_noise=False
- UniLab demo --help → 子命令存在(demo <demo_name>), 章 'demo locomani' 形态可信
- grep isaaclab/source → Isaac-PickPlace-Locomanipulation-G1-Abs-v0 确实注册(manager_based/locomanipulation/pick_place/__init__.py:15), 且属 Mimic/teleop/IK 生态(test_pink_ik、XR teleop CHANGELOG、姊妹 Isaac-Locomanipulation-G1-Abs-Mimic-v0) → 章 line83 任务id存在、line102-104 pitfall 定性正确

**工程核对（是这样/不符）**
- 不符(重要): 章 Quick Start(line94/95)与 UniLab owner YAML 路径(line866 conf/ppo/task/go2_arm/mujoco.yaml)用任务名 go2_arm — 实跑报 'No owner config'; 真名为 go2_arm_manip_loco(注册名 Go2ArmManipLoco)。这是会让读者照抄即失败的硬错
- 不符(重要): 章把 UniLab go2_arm 讲成 抓取/夹爪 任务(line707-741 obs含grip/obj_quat、line844-879 reward含 grasp_success/stop_during_grasp/locomanip_approach/夹爪闭合&物体抬起)。真实 go2_arm_manip_loco 全程无 gripper/grasp: reward 只有 object_distance/object_distance_l2/arm_collision+loco项, 是'边走边用IK把末端送到采样EE目标'的 loco-manip reaching, 非 pick-and-place
- 不符: 章称 UniLab 臂走'关节位置目标残差 ctrl=act*action_scale+default_angles'(line99/411)。腿确实如此(已实证), 但臂实为 arm_dof_pos+act*arm_action_scale+ik.gain*dq_ik — 是 Diff-IK 残差(默认 arm_action_scale=0.0 即纯IK), 章未提 IK-in-the-loop 这一真实(且更高级)机制
- 不符: 章 UniLab 非对称AC声称 critic 比 actor 多拼 object_lin_vel/friction/arm_payload/contact_force_gripper 等特权维(line713/730-737, 标 obs64/critic78)。真实 env actor 与 critic 共用同一 79维布局(line778-830), 唯一差别是 actor add_noise=True/critic add_noise=False; 不存在那套 object/接触力特权拆分; critic 单步维=79 非78
- 不符(轻): env 源码路径 章 line35 写 envs/locomotion/go2_arm, 真实在 src/unilab/envs/locomotion/go2_arm/manip_loco.py(缺 src/ 前缀与文件名); 且该 UniLab 复合平台臂是 Airbot(docs/manip_loco.md 明示'Go2运动与Airbot机械臂结合'), 非章通篇举例的 ARX
- 符合: mjlab 任务清单(12个/无Go2)、mjlab 观测是 mdp 函数(base_ang_vel等)、JointPositionActionCfg(entity_name+actuator_names+scale+use_default_offset)、mjlab obs 组名 actor/critic — 全部实测属实
- 符合: UniLab reward 派发(∑ scale*fn 跳过 scale==0、再 *ctrl_dt、每4步写 info['log']) 与 章 line842/861-863 完全一致; obs_groups_spec 返回 {'obs','critic'} 与 actor加噪/critic干净 机制属实
- 符合: Isaac-PickPlace-Locomanipulation-G1-Abs-v0 确为注册任务且属 teleop/Mimic/IK(非现成RL reward环境), 章 line83 任务id 与 line102-104/1252 pitfall 定性均正确

**优化点 / 高级技巧**
- 把 UniLab 章节示例任务名从 go2_arm 全改为 go2_arm_manip_loco(Quick Start line94/95、owner YAML 路径 line866、line35 注册名表述), 否则读者命令必失败; 顺手把源码路径补成 src/unilab/envs/locomotion/go2_arm/manip_loco.py
- UniLab 真实 go2_arm_manip_loco 是 IK 驱动的 loco-manip reaching(臂=IK残差, EE目标在 goal_ee 的 sphere_l/phi/theta 上采样, 有 ik.orientation_mode=target/zero_error 两档)。这正是章主线 VBC'分层低层用IK跟踪EE目标'的现成可跑实例 — 建议直接拿它当 UniLab 实战载体, 比虚构 gripper 抓取 demo 更贴真实、且能实跑; 可加一句 'arm_action_scale=0.0 即纯IK, >0 即 IK+学习残差' 的调参点(docs/manip_loco.md 已列为首要调参旋钮)
- 章 UniLab 非对称AC示例(obj_lin_vel/friction/payload/contact_force_gripper 特权拆分)与真实 env 不符; 真实 env 的'非对称'仅体现为 actor加噪/critic不加噪(同维 79)。若要保留'critic可加特权'的教学, 应改用 mjlab/IsaacLab 的真例, 或明确标注 UniLab go2_arm_manip_loco 此版未做对象级特权拆分(只做 noise 拆分)
- 章未提 arm_stage.freeze_arm_joints 这一真实配置项: 它正是章 Phase0/Phase1'冻结臂/冻结底盘'课程的官方落点(apply_action 里 freeze 时 effective_actions[:,12:18]=0 且 arm_ctrl 锁 default_angles)。建议把四阶段课程的 UniLab 兑现指向 env.arm_stage.freeze_arm_joints + env.goal_ee.disable_ee_goal_trajectory, 让'课程'有可一条 Hydra 覆盖落地的真实开关
- 可补 Isaac 端更贴RL的 loco-manip 任务: grep 到 Isaac-Tracking-LocoManip-Digit-v0(manager_based/locomanipulation/tracking) 与 Isaac-G1-SteeringWheel-Locomanipulation, 比纯 teleop 的 PickPlace-G1 更适合做 RL reward 教学对照, 章可在'四足+臂须自定义'之外提一句这些现成 locomanip tracking 任务
- UniLab docs/manip_loco.md 给了真实'近风险自检'命令(pytest tests/envs/locomotion/go2_arm + test_mujoco_site_jacobian, 以及 mujoco.MjModel.from_xml_path('.../go2_arm/scene_flat.xml') 打印 nq/nv/nu/nsensor)。章'复合模型10分钟验证清单'(line418-429)可直接引这套官方实测命令, 把抽象 checklist 落成可跑的一行验证

**教学缺口（只讲代码没教工程之处）**
- UniLab 真实任务的'arm 由 IK 求解、动作是 IK 残差'是本章三重耦合(逆运动学耦合)最好的活教材, 但章把 UniLab 讲成纯关节残差+夹爪抓取, 反而错过了'框架里 IK 怎么和 RL 动作叠加'这一最能教会工程的点; 建议补一段: 为什么带臂任务用 IK 跟 EE 目标而不让策略直接出 6 臂关节(降维、与 base 系相对坐标契约), 配真实 ik.gain/dq_clip/orientation_mode 讲'怎么调'
- 章 Quick Start 三框架命令'看机器人站住'的意图好, 但 UniLab 段(line86-100)绕了一大圈解释'为何不用 zero agent', 却没给一条真正会成功的命令(go2_arm 直接失败)。教学上应给'最小一定能跑'的命令(go2_arm_manip_loco max_iterations=1 + 实跑应看到的真实 reward 行 object_distance≈1.8), 让读者第一步就有正反馈
- 章四阶段课程(冻臂→冻底盘→全身→加DR)讲得很足, 但全是伪代码 phase0_cfg=dict(...) + 自造 uv run train LocoManip-Go2Arx-Phase2(任务名不存在)。读者无法照着从零搭起。真实 UniLab 已用 arm_stage.freeze_arm_joints + domain_rand 一组开关表达'阶段', 应教读者用真实 Hydra 覆盖跑出 Phase0/Phase3, 而非给跑不起来的虚构命令
- 章 line866-879 的 UniLab reward YAML(scales: locomanip_approach/grasp_success/stop_during_grasp...)整段是虚构权重表, 与真实 mujoco.yaml(tracking_lin_vel/object_distance/arm_collision...)无一对应。'改权重不碰代码、一条Hydra覆盖'这条工程要点是对的, 但例子应换成真实 reward 键(如 reward.scales.object_distance=3.0), 否则读者一覆盖就报 key 不存在
- 非对称 AC 这一节(line705-741)用 UniLab 手写拼接讲'critic追加特权', 机制描述(actor加噪/critic干净)与真实 env 一致是好的, 但虚构的特权维清单(物体真值速度/摩擦/payload/夹爪接触力)会误导读者去找 env 里不存在的字段; 应明确区分'noise 非对称'(本任务真有)与'对象级特权非对称'(本任务没有, 是别的任务/章节的做法)

**notes**：三框架均连通并实跑: mjlab(Go1 velocity)与 UniLab(go2_arm_manip_loco)训练 1-2 iter 均成功、steps/s 与 loss 正常; Isaac 任务经源码 grep 确认注册(其裸 import 因缺 pxr 需 SimApp 启动, 故未跑训练, 但 task id 存在性已证)。核心问题集中在 UniLab 段: 章把真实的'IK驱动 loco-manip reaching(Go2+Airbot, 无夹爪)'写成了'关节残差+夹爪抓取(Go2+ARX)', 连带任务名(go2_arm vs go2_arm_manip_loco)、reward键、critic维度(78 vs 79)、obs特权拆分 都与服务器实装不符 — 任务名错会让读者照抄即报错, 优先级最高。mjlab/IsaacLab 段经实测基本属实, 仅个别版本号细节无碍。所有结论附命令/行号, 未改任何 .tex。

---

## prac_large_scale

**服务器**：gpufree (RTX 4090 24GB, mjlab 1.4.0 / rsl-rl-lib 5.2.0 / Isaac Lab 2.3.2 / UniLab commit-pinned)

**实跑命令（8）**
- mjlab list-envs → 12 个真实任务名与章节完全一致(Velocity/Tracking-Flat-Unitree-G1/Go1, Lift-Cube-Yam, Cartpole-*, 无 Go2);
- mjlab train Mjlab-Velocity-Flat-Unitree-Go1 --env.scene.num-envs 64 --agent.max-iterations 2 --gpu-ids '[0]' → 成功跑通, ~1280-1355 steps/s, value/surrogate/entropy loss 正常(章节 Quick Start 命令属实);
- mjlab train ... --enable-nan-guard True → 打印 '[INFO] NaN guard enabled, output dir: /tmp/mjlab/nan_dumps', 无崩溃(旗标+默认目录与章节 line 596/602 逐字一致);
- train Mjlab-Velocity-Flat-Unitree-Go1 --help → 确证 --enable-nan-guard / --gpu-ids / --env.sim.nan-guard.{enabled,output-dir,buffer-size,max-envs-to-dump} / --agent.seed(默认42) / --agent.logger{wandb,tensorboard}(默认wandb) 全部存在;
- UniLab train --algo ppo --task g1_walk_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 → 成功跑通, ~593 steps/s(CPU sim), reward 分项(tracking_lin_vel/feet_phase/...)正常, 自动存 git diff(印证运行包实践);
- Isaac Lab Isaac-Cartpole-v0 --headless --max_iterations 2 → 成功跑通, 100k→300k steps/s, reward/episode 正常;
- UniLab conf/{ppo,offpolicy,appo}/config.yaml → nan_guard.enabled: true(三处全 true), multi_gpu_sync_mode: local_sgd, multi_gpu_sync_interval: 1, num_gpus: 1, buffer_size:100, max_envs_to_dump:5 — 与章节 line 340/627/1459 完全吻合;
- 源码核对: mjlab utils/nan_guard.py(NanGuard/watch/states_step_*/_metadata/nan_env_ids/默认/tmp/mjlab/nan_dumps全对), sim/sim.py(create_graph 捕4图 step/forward/reset/sense + expand_model_fields 后自动重捕, 门控 wp.is_mempool_enabled), rsl_rl normalization.py(.std 属性存在, update_normalization 在 ppo.py process_env_step), UniLab ipc/{weight_sync.py SharedWeightSync 版本化, replay_pipelines/multi_gpu_cpu_pinned.py per-rank H2D stream, memory_budget.py estimate_offpolicy/appo_bytes + /dev/shm 0.8 阈值 + MemoryError + UNILAB_SKIP_MEMORY_CHECK} 全部存在且语义一致;

**工程核对（是这样/不符）**
- 是这样: 三框架代表性训练命令(mjlab Go1 velocity / UniLab ppo g1_walk_flat / Isaac Cartpole)全部实跑跑通, steps/s 与各 loss 正常, 章节 Quick Start 与 §多GPU 的 mjlab/UniLab 命令骨架在真机可直接复制运行;
- 是这样: mjlab NaN Guard 全链路属实——--enable-nan-guard True 实跑打印默认目录 /tmp/mjlab/nan_dumps; 源码 NanGuard.watch / dump 的 npz 键 states_step_* + _metadata(nan_env_ids) 与章节 line 610-625 的 analyze_nan_dump 读法逐字吻合;
- 是这样: CUDA Graph 五大根因(根因5)与 pitfall 准确——sim.py 确有 create_graph() 捕 step/forward/reset/sense 4 图, 由 __init__ 与 expand_model_fields() 自动重捕, 门控 wp.is_mempool_enabled, CPU/无mempool 为 no-op; '用户不应手动调 create_graph()' 的告诫成立;
- 是这样: UniLab 异构多 GPU 三处源码全部坐实——SharedWeightSync(版本化共享内存/单写)、MultiGPUCPUPinnedReplayPipeline(per-rank 独立 host slot+H2D stream, 无 rank0 漏斗)、multi_gpu_sync_mode∈{local_sgd,sync_sgd} 默认 local_sgd, interval=1; 与 §unilab-multigpu 与 note(line 340)完全一致;
- 澄清(非错): 章节反复称 UniLab NaN 守卫'默认开'——NanGuardCfg dataclass 裸默认其实是 enabled=False, 但 conf/{ppo,offpolicy,appo}/config.yaml 三处训练配置均覆盖为 enabled: true, 故'训练时默认开'的表述在工程语义上正确, 无需改;
- 小瑕疵(非错, 可精修): 章节 line 742-744 称 mjlab play '无 --max-steps/--no-render 这类跑N步即退旗标、是 viewer 驱动'——基本属实(确无 --max-steps), 但 play 实有 --video/--video-length INT(可定长录制即退)与 --viewer{auto,native,viser} 选择器, 措辞可略放宽;
- 是这样: RSL-RL EmpiricalNormalization 的 .std 属性确实存在(normalization.py:42), update_normalization 确在 process_env_step(ppo.py:133-134 actor+critic 各调一次), 章节根因3与 warmup 体检代码(line 553-554 用 .std**2)在真机可用。

**优化点 / 高级技巧**
- mjlab 有一条章节 NaN 节完全没提的更轻量防线: train 暴露 --env.observations.{actor,critic}.nan-policy{disabled,warn,sanitize,error} 与 --nan-check-per-term(默认 True)——即 obs 组级 NaN 处理策略, 比重型 --enable-nan-guard dump 更早、更细地在观测入口拦 NaN(error=严格开发模式直接抛, per-term 定位是哪一维), 建议在 NaN 节补一句'obs 入口的 nan-policy 是 NaN Guard 之外的第一道闸';
- 章节 §性能 给 mjlab 纯 env 吞吐建议用自定义 step 循环、不要用 play 当 benchmark——可补充: play 其实支持 --viewer native|viser 与 --video-length, 真要无头快测可用 --viewer 关交互, 不必只走自写循环(给读者多一个低成本选项);
- UniLab note(line 340)已点明 multi_gpu_sync_interval 默认=1(等于逐 iteration 同步, 并非真·local SGD), 可再强调一句工程含义: 想要'少同步换吞吐'必须显式把 interval 调大(如 8/16), 否则 local_sgd 名不副实——这是个容易被默认值坑到的调参点;
- mjlab steps/s 在 64 envs 仅 ~1300、Isaac Cartpole 在小 env 数首迭代 100k 次迭代 300k——可在 §性能 提醒读者: 小 env 数下吞吐受 CUDA Graph 一次性捕获 + 启动开销主导, benchmark 必须按章节十字段协议加 warmup 且用目标 env 数(4096), 小规模数字无参考意义(章节已有 warmup 纪律, 但可显式连到实测现象);
- 可补一条现代排错技巧: mjlab/rsl-rl 训练时设 CUDA_LAUNCH_BLOCKING=1 + torch.autograd.set_detect_anomaly(True) 能把'随机 NaN'定位到具体 kernel/算子, 与章节'禁用 graph 二分'互补, 属更细的数值排错手段。

**教学缺口（只讲代码没教工程之处）**
- NaN 节几乎全部围绕 mjlab 的 --enable-nan-guard 重型 dump 工具, 却没教读者 mjlab 自带的 obs 组级 nan-policy(disabled/warn/sanitize/error)+per-term 检查这条更轻、更适合日常开发的闸——读者学不到'分层设防(obs入口 error → 整训 nan-guard dump)'的工程梯度;
- §多GPU 的 mjlab/Isaac 双卡命令本机(单卡 4090)无法真正跑通验证, 章节也未提示读者'单卡如何先验证 --gpu-ids 解析/--distributed 链路不报错'——缺一个'没有第二块卡时怎么烟雾测试多卡配置'的可操作降级路径, 教学上留了个只能照抄不能自验的盲区;
- '默认开'这类结论章节直接给值(line 627), 但没教读者怎么自己查证'某框架某开关的真实生效默认'——即区分 dataclass 裸默认 vs Hydra/conf 覆盖默认(本章 UniLab nan_guard 正是 False 被 conf 覆盖成 true 的典型), 这个'查默认值要看 conf 不是看 struct'的排错方法论值得显式点出;
- §性能 的 2.5x 加速案例与各层成本比例是 A100 旁征的'案例值/量级参考', 章节诚实标注了, 但没给读者一套'在自己机器上从零复现这张分层测量表'的最小脚本步骤(先关sensor测物理→加sensor→加manager→全训), find_optimal_num_envs/nconmax 给了函数却没串成'怎么一步步搭出这张表'的操作流, 略偏'给代码'而非'教搭法';
- AGILE/UniLab 大量 API(SharedWeightSync/TransitionBootstrapContract/PenaltyCurriculum 等)源码确实存在且被准确引用, 但读者无法在本书所给三框架命令里直接触达这些类(多 GPU/APPO 需要第二块卡或特定 conf), 教学上是'可信但不可手动复现'——可补一句'这些类如何在单机最小配置下被实例化看到'的指引。

**notes**：本章工程可信度很高: 在 gpufree(RTX4090) 实跑了三框架代表性训练(mjlab Go1 velocity / UniLab ppo g1_walk_flat / Isaac Cartpole 全部跑通)+ mjlab --enable-nan-guard + 深入核对 mjlab/rsl-rl/UniLab 三处源码, 章节的任务名、CLI 旗标、NaN Guard 链路、CUDA Graph 机制、UniLab 异构多 GPU(SharedWeightSync/per-rank H2D/local_sgd 默认)、EmpiricalNormalization 等关键工程做法全部对得上, 未发现任何实质性错误(报错/API 不存在/参数不对)。唯一需澄清(非错)的是 UniLab NaN 守卫'默认开'是 conf 层覆盖而非 dataclass 默认, 章节表述工程语义正确。绝未修改任何 .tex/refs.bib/part.tex。优化点主要集中在: 补 mjlab obs 组级 nan-policy 这道被遗漏的轻量防线、multi_gpu_sync_interval 默认=1 的调参陷阱、以及单卡环境下如何烟雾测试多卡配置的教学降级路径。


---

## prac_tennis_env

**服务器**：gpufree (RTX4090; mjlab 1.4.0 / MuJoCo 3.8.1 / Isaac Lab 2.3.x / UniLab main)

**实跑命令（15）**
- list-envs: 实出 12 个任务、无任何 tennis/网球任务 — 印证章节『mjlab 无 tennis 内置任务』(line 64) 正确
- mjlab train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2: 跑通, ~2473 steps/s, loss 正常 — 框架端到端可用
- import mjlab.managers 列 dir: ManagerTermBase/CommandTerm/EventTermCfg/TerminationTermCfg/ObservationTermCfg/RewardTermCfg 全在; 无 EventTerm/TerminationTerm/ObsTerm 短名 — 印证 line 689/692/847/934
- Entity.write_external_wrench_to_sim 签名 = (forces, torques, env_ids=None, body_ids=None) — 与 line 719 章节声称逐字一致
- Entity.write_root_link_pose_to_sim / write_root_link_velocity_to_sim 存在 — 印证 line 590-591
- MjSpec: add_body/add_freejoint/add_geom 均存在且可调用 (MuJoCo 3.8.1) — 印证 line 365-377
- EntityData dir: root_pos_w/root_lin_vel_w/root_ang_vel_w/root_lin_vel_b 全部【不存在】; 真名是 root_link_pos_w/root_link_lin_vel_w/root_link_ang_vel_w/root_com_lin_vel_b — 章节~9 处代码用了不存在的短名
- CommandTerm._resample_command/_update_command/_update_metrics 存在 — 印证 LaunchCommand 重写 (line 565/596)
- UniLab: NpEnv(update_state+apply_action)、SceneCfg(model_file+fragment_files+visual_model_file)、registry.envcfg/env 均存在 — 印证 line 974/981/988/1091
- MuJoCo 实跑: freejoint nq=7 nv=6 — 印证 line 352-354
- COR 标定实跑: 章节 solref=(-3500,-2.0)+solimp=(0.95...) 从 2.54m 落体测得 COR≈0.95 (回弹89%), 远高于章节自称的 0.73-0.76/回弹53-58% (line 414/467/811)
- solref 扫描: 直接格式 -3500 后, 阻尼项 -2→COR0.95, -12→0.74(命中目标), -30→0.50, -60→0.29 (单调); 正格式 [0.005~0.02,1.0~1.5]+常规 solimp 一律 COR0.06-0.14(几乎不弹)
- vertical COR vs slide friction: 0.6→0.739, 0.2→0.738 (≈相等) — 印证练习答案 line 869
- condim 实跑: condim=1 滑3s 后 x=15m vx=5.0 不减速; condim=3 后 x=10.9m vx=3.57 减速 — 印证 condim 决策树 line 405-412
- play.py: agent: Literal['zero','random','trained']; zero→DUMMY_MODE+torch.zeros(action_shape) — 印证 Quick Start --agent zero (line 79); export_scene.py 存在 (印证实验2)

**工程核对（是这样/不符）**
- 【不符·关键API】章节~9 处『mjlab 1.4.0 风格』代码用 ball.data.root_pos_w / root_lin_vel_w / root_ang_vel_w / root_lin_vel_b (line 699,700,822,824,850,917,919,921,967), 但实测 mjlab 1.4.0 EntityData 无这些短名(那是 Isaac Lab 习惯); 真名须是 root_link_pos_w / root_link_lin_vel_w / root_link_ang_vel_w / root_link_lin_vel_b(或 root_com_*) — 照抄会 AttributeError
- 【不符·关键物理】章节自称 solref=(-3500,-2.0)、solimp=(0.95,0.99,0.001,0.5,2.0) 是『实测调参值』使硬地 COR 0.73-0.76(line 414,467,811), 但实跑 2.54m 落体 COR≈0.95(回弹89%)严重偏弹; 要命中 0.73-0.76 阻尼项须约 -12(实测 -3500 -12 → COR0.739)
- 【不符·教学方向反】练习 line 500 与排障表暗示 solref=[0.005,1.0](硬)→高恢复、增大 dampratio 降 COR; 实测正格式 timeconst/dampratio 在常规 solimp 下 COR 仅 0.06-0.14(几乎死球)且 dampratio 几乎不动 COR; 真正能调出网球弹性的是直接负格式(或同时拉高 solimp/刚度), 章节 solref_hard『快弹高恢复』直觉对真实落体误导
- 【符·已就地修正项全部属实】mjlab 无 tennis 任务、write_external_wrench_to_sim 签名、ManagerTermBase 唯一基类 + 无 EventTerm/TerminationTerm 短名、CommandTerm 真存在、MjSpec add_freejoint/add_geom、--agent zero/random、export-scene、freejoint qpos7/qvel6 — 全部实跑核实为真
- 【符·UniLab 接线属实】NpEnv(update_state/apply_action)、SceneCfg.model_file+fragment_files、@registry.envcfg/env 三装饰器、UniLab 26 任务无网球 — 均与服务器实装一致
- 【符·物理教学属实】vertical COR 与 slide friction 无关(0.739 vs 0.738)、condim=1 无限滑/condim=3 减速 — 两条核心物理教学实跑成立
- 【次要】排障表 line 1347『弹太高 COR>0.85 → 增大 dampratio 至 1.2-1.5』: 在正(timeconst/dampratio)格式语境下 1.2-1.5 量级几乎不影响 COR(实测), 该量级建议对直接负格式才有效, 表述与所用格式不匹配

**优化点 / 高级技巧**
- 给 COR 标定补『先验收一次落体再写死参数』的工程闭环: 正文应直接贴一段 10 行的 MuJoCo standalone 落体脚本(本次实跑那段), 让读者跑出真实 COR 而非信任纸面 solref — 章节有 calibrate_cor 但走 env.step(高耦合), 缺无 mjlab 也能跑的纯 MuJoCo 最小标定
- 明确『COR 主旋钮是直接负格式 solref 的第二项(阻尼)』并给单调标定表(本次实测: -2/0.95, -12/0.74, -30/0.50), 比章节当前『solref 与 COR 非线性、需实验标定』这种只给方向不给可复现数据的说法更可操作
- 统一所有 mjlab 数据读取为 root_link_*_w / root_com_*_b 长名, 并加一句『mjlab 显式区分 link 帧 vs com 帧, 没有 Isaac 的 root_pos_w 短名别名』的工程提示 — 这正是跨框架迁移最常踩的 API 坑, 适合做成一个 pitfall
- 外力施加可补 write_external_wrench_to_sim 之后必须 write_data_to_sim() 落盘的说明(Entity 确有 write_data_to_sim); 章节只在 Isaac Lab 侧讲了 write_data_to_sim, mjlab 侧的 buffer→sim 落盘时机没点透, 易写了外力不生效
- 可补 sphere 高速穿薄面的可复现演示: 用本次的纯 MuJoCo 脚本给定 30m/s + 1cm box, 实测单步位移 vs 厚度比, 把 line 290/1033 的『理论可能穿透』变成实跑数字, 教学冲击力更强
- 建议给一条『RTX4090 单卡基线吞吐』锚点(实测 Go1 64envs≈2473 steps/s)让读者校准自己环境是否正常 — 章节通篇无任何 steps/s 量级参照

**教学缺口（只讲代码没教工程之处）**
- COR 一节是最大教学缺口: 给了 solref 数值与『目标0.73-0.76』却没教读者『拿到一组 solref 后怎么自己测出 COR、测出来不对怎么反推该调哪个分量』; 实测章节给的数值根本测不出它宣称的 COR, 说明作者很可能没跑过这条标定闭环 — 应改成『先教测量法、再给须自标的范围』而非直接抛一组(且错的)魔法常量
- 数据访问 API 只给代码不教『怎么自己发现真名』: 应教读者用 dir(env.scene['ball'].data) / 报错即查 的排错法, 因为短名照抄必崩, 这正是『从零搭起』最先撞墙处, 章节却把错误短名当成可照抄范本
- solref/solimp 五元组只解释了 timeconst/dampratio 两个直觉, 对实际决定弹性的 solimp(dmax 等)与直接负格式的物理含义几乎没教, 读者无法举一反三调出目标 COR — 接触阻抗这块是『堆了参数没教会调参』的典型
- 大量 bash 命令(uv run play Mjlab-Tennis-Launcher ...)因该任务不存在而无法真跑, 章节已诚实标注是占位/示意, 但对『从零搭起』的读者缺一条可立即上手的替代验证路径(如用现成 Mjlab-Cartpole 或纯 MuJoCo 脚本先把 freejoint/COR/condim 这些可迁移概念跑通)
- 排障表与练习里多处『增大/减小 dampratio 调 COR』的因果, 与实跑的格式相关性(正格式几乎无效)对不上 — 教学上把『改一个参数→COR 怎么变』讲成了与实际相反的心智模型, 读者照此排障会越调越偏

**notes**：实跑覆盖三框架(mjlab 训练+API+物理、UniLab import、Isaac 目录存在)与 MuJoCo 物理(COR/condim/friction/freejoint), 未改任何 .tex。最值得主控处理的两点: (1) mjlab 数据访问短名 root_pos_w/root_lin_vel_w 等不存在, 真名 root_link_*(约9处代码受影响); (2) 章节自称『实测』的 solref=(-3500,-2.0) 落体 COR≈0.95 而非 0.73-0.76, 且正格式 timeconst/dampratio 示例基本调不出网球弹性 —— 两者都指向 COR/接触参数这一节的代码与数值未经真跑, 是本章工程可信度的主要薄弱点。其余已三轮核销的 API/版本/UniLab/论文归属/condim/freejoint 物理, 本次实跑全部证实为真。Mjlab-Tennis-Launcher 等命令因任务不存在无法直跑(章节已诚实标注示意), 故对其 CLI 形状未做端到端验证, 但 --agent zero/export-scene/play 入口本身已实跑确认存在。

---

## prac_tennis_perception

**服务器**：gpufree (RTX4090; mjlab 1.4.0 @ /root/gpufree-data/envs/mjlab, torch 2.12.1+cu130; UniLab @ /root/gpufree-data/src/UniLab commit-era 99936e08)

**实跑命令（9）**
- list-envs → 12 mjlab tasks; NO tennis task (confirms Mjlab-Tennis-Launcher absent), but RGB/Depth/Seg tasks DO exist: Mjlab-Lift-Cube-Yam-Rgb / -Depth / Mjlab-Multi-Cube-Seg-Yam
- Quick Start EKF (verbatim, dt=0.01) → raw 3.14cm / EKF 2.58cm = only ~18% reduction (NOT '明显更小'); re-run seeded 60Hz → raw 3.29 / EKF 2.67 = 19%
- grep UniLab src/unilab/base/backend/{base,mujoco,motrix}/backend.py → get_body_pos_w / get_body_lin_vel_w / get_body_lin_vel_b / get_sensor_data ALL exist (chapter UniLab API claims CONFIRMED)
- grep UniLab obs_groups_spec + np_env.py:67 docstring literally '{"obs":98,"critic":101}' + split_obs_dict in observations.py:16 → asymmetric-AC two-group claim CONFIRMED
- UniLab hora/__init__.py + distill.py:298 → class HoraDistillationTrainer + adapt_tconv (models.py:142) CONFIRMED exactly as chapter cites
- UniLab ppo/task = 21 files (allegro/g1/go1/go2/go2w/sharpa/stewart/x2); grep camera/rgb/tiledcamera in envs/ → 0 real hits (confirms '无相机任务')
- mjlab manager_based_rl_env.py:266 step_dt = cfg.sim.mujoco.timestep*cfg.decimation; episode_length_buf+common_step_counter present → timestamp-derivation claim CONFIRMED
- mjlab entity/data.py: NO root_pos_w/root_lin_vel_w/root_ang_vel_w (grep empty); actual names are root_link_pos_w / root_link_lin_vel_w / root_link_ang_vel_w (+ root_com_* variants)
- mjlab sensor/camera_sensor.py → class CameraSensorCfg, data_types Literal['rgb','depth','segmentation']; sensor/contact_sensor.py → class ContactSensorCfg with force/netforce — both first-class in mjlab

**工程核对（是这样/不符）**
- [L54/202/269/290/338-339,383-385] FALSE: chapter uses ball.data.root_pos_w / root_lin_vel_w / root_ang_vel_w for mjlab and calls it '逐字相同' with Isaac Lab — but mjlab EntityData has NO such attributes; real mjlab names are root_link_pos_w / root_link_lin_vel_w / root_link_ang_vel_w (Isaac-Lab-only uses the _pos_w form). collect_ground_truth_label / ball_state_privileged would AttributeError on mjlab as written.
- [L443-448,435] MISLEADING: chapter says only Isaac Lab has a camera-config class (TiledCameraCfg/CameraCfg) and mjlab is just 'MuJoCo Warp 渲染管线 / depth buffer / segmentation buffer' — but mjlab has a first-class CameraSensorCfg with data_types=('rgb','depth','segmentation') AND working tasks Mjlab-Lift-Cube-Yam-Rgb/-Depth, Mjlab-Multi-Cube-Seg-Yam. mjlab is NOT camera-less.
- [L768] INCOMPLETE: contact-event row gives mjlab only 'MjData.contact' while reserving ContactSensorCfg for Isaac Lab — mjlab also ships its own ContactSensorCfg (sensor/contact_sensor.py, force/netforce/maxforce reduce), the cleaner API to recommend.
- [L62,435,1161] CONFIRMED: UniLab backend getters (get_body_pos_w/get_body_lin_vel_w/get_body_lin_vel_b/get_sensor_data), obs_groups_spec {obs,critic} two-group, split_obs_dict, and HoraDistillationTrainer+adapt_tconv all exist verbatim in source — UniLab column is accurate.
- [L278,977,297] CONFIRMED: mjlab step_dt = sim.timestep*decimation, episode_length_buf + common_step_counter all present; timestamp = episode_length_buf*step_dt is valid mjlab code.
- [L102-103 comment '# 应明显更小', L593 '低50%+'] OVERSTATED: verbatim Quick Start EKF gives only ~18-19% RMSE reduction (raw 3.14→EKF 2.58cm), not '明显更小'/'50%+'. With 2cm noise on a directly-observed position and Q=1e-3/R=4e-4, a position-only KF cannot smooth much; the 50%+ claim needs higher noise, coarser dt, or judging velocity (where KF wins big), not position RMSE at dt=0.01.
- [L109] 'uv run play Mjlab-Tennis-Launcher' is non-runnable today: list-envs has no tennis task (chapter already flags tennis as 预规划, so consistent, but the bash box reads as a runnable command).

**优化点 / 高级技巧**
- Make the mjlab-vs-Isaac frame table honest about the naming split: add a row 'mjlab: root_link_pos_w / root_link_lin_vel_w / root_link_ang_vel_w  vs  Isaac: root_pos_w / root_lin_vel_w / root_ang_vel_w' and drop '逐字相同' for the getters — this is exactly the kind of cross-framework gotcha the chapter exists to teach.
- Teach mjlab's real CameraSensorCfg(data_types=('rgb','depth','segmentation')) and point at the shipped Mjlab-*-Rgb/-Depth/-Seg tasks as a runnable camera reference; segmentation-as-teacher-signal can be demoed in mjlab today, strengthening the privileged-learning thread instead of deferring all vision to Isaac.
- Recommend mjlab ContactSensorCfg (force/netforce, filter to ball-ground pair) over raw MjData.contact for bounce detection — it's the batched, GPU-friendly path and parallels Isaac's ContactSensorCfg, making the 'contact event source' row symmetric.
- Fix the Quick Start expectation: either bump obs noise to ~5cm + dt to 1/60 and report the velocity-RMSE win (KF crushes finite-difference there), or relabel the position-RMSE gain as 'modest (~20%) — the real payoff is velocity & missed-frame handling'. As written it sets readers up to think their correct EKF is broken.
- Add a one-liner that mjlab entity also exposes root_com_* (COM-frame) vs root_link_* (link-frame) — relevant because for a single-body ball link≈COM, but for the downstream humanoid it matters which frame feeds the predictor; good teaching moment the chapter could seize.
- Quick Start could note num-envs/batched form: the minimal KF is single-env loop; chapter's own BallEKF uses torch.bmm — bridging the two (why bmm, expand vs repeat, .clone() on expand) would close the gap between toy and production code.

**教学缺口（只讲代码没教工程之处）**
- The whole chapter hinges on ball.data.root_pos_w being a shared cross-framework name, but never has the reader actually print the attribute on a real entity — a 3-line 'instantiate a RigidObject, dir(ball.data), confirm the attribute name on YOUR framework' exercise would have caught the mjlab root_link_ vs root_ naming itself and teaches API-discovery (the transferable skill) instead of asserting names.
- Quick Start says output '应明显更小' with no acceptance number and no explanation of WHEN a position KF helps little vs a lot (it's a function of Q/R and whether you score position or velocity). Reader who gets 18% will think they have a bug. Teach the diagnostic: position-observed KF mainly de-noises velocity; score velocity RMSE or NIS, not just position RMSE.
- Q/R tuning section (L576-591) gives a good NIS recipe but never has the reader compute NIS on the Quick Start trajectory — the one place they have ground truth and could SEE chi-square≈3. The recipe stays theoretical; a worked NIS print on the toy data would make 'how to tune' concrete.
- Camera config is presented purely as a comparison table (mjlab=渲染管线, Isaac=TiledCameraCfg) with zero runnable handle, yet mjlab ships RGB/Depth/Seg tasks — chapter could send reader to `list-envs` + run Mjlab-Lift-Cube-Yam-Rgb to actually see a camera tensor shape, turning '预规划' vapor into a 5-min hands-on, and teaching how to discover sensor output dtypes/shapes from a real env.
- The 'from-scratch' build path is thin for the framework seam: chapter says 'change the backend getter' but never shows the minimal mjlab snippet (resolve body id via indexing, read root_link_pos_w, subtract scene.env_origins). Reader is left to guess the exact mjlab call — the most error-prone 10 lines get only a table cell.

**notes**：Core pure-PyTorch content (EKF math, Joseph form, drag Jacobian, GRU residual, delay 3-step, PredictorOutput, distillation) is sound and framework-agnostic as claimed — I ran the Quick Start EKF and it executes correctly (only the '明显更小' magnitude is off). All UniLab-column claims (backend getters, obs_groups_spec, HoraDistillationTrainer/adapt_tconv, no-camera) verified against source and are accurate. The ONE substantive engineering error is the mjlab ball-state API: chapter reuses Isaac Lab's root_pos_w/root_lin_vel_w/root_ang_vel_w names for mjlab and labels them '逐字相同', but mjlab's EntityData uses root_link_pos_w/root_link_lin_vel_w/root_link_ang_vel_w — the code as written AttributeErrors on mjlab. Secondary: chapter under-credits mjlab's vision (it has real CameraSensorCfg with rgb/depth/segmentation + shipped Rgb/Depth/Seg tasks, and its own ContactSensorCfg), so the 'mjlab basically can't do cameras' framing is wrong. Did not run Isaac Lab (no Isaac-side claim was in doubt and chapter already hedges TiledCameraCfg/ContactSensorCfg naming to 'as-of-your-version'); did not run full mjlab training since the chapter's tennis task does not exist and no training claim was load-bearing. No .tex/.bib touched.

---

## prac_tennis_control

**服务器**：gpufree (RTX 4090 24GB; mjlab 1.4.0 / Isaac Lab 2.3.2 / UniLab commit齐全)

**实跑命令（12）**
- list-envs → 真实12个任务,确认无Go2/无网球task(本章API为示意,符合chapter自述)
- train Mjlab-Velocity-Flat-Unitree-G1 --num-envs 64 --max-iterations 2 → 成功,2537 steps/s,value/surrogate/entropy loss正常
- train Mjlab-Lift-Cube-Yam --num-envs 32 --max-iterations 1 → 成功,689 steps/s,ActionManager(7维)+action_rate_l2奖励项确认
- python -c 'from mjlab.envs.mdp.actions import DifferentialIKActionCfg' → 类存在,字段=entity_name/actuator_names/frame_type/frame_name/use_relative_mode/delta_pos_scale/delta_ori_scale/damping/max_dq + (额外)position_weight/orientation_weight/joint_limit_weight/posture_weight/posture_target
- dataclass defaults: damping=0.05 ✓ max_dq=0.5 ✓ use_relative_mode=True ✓ delta_pos_scale=1.0 delta_ori_scale=1.0(默认未缩放)
- from mjlab.envs.mdp import dr → body_mass(默认operation=scale)/geom_friction(默认operation=abs)/pd_gains(kp_range,kd_range)签名与chapter完全一致;pseudo_inertia存在(印证UniLab对照中'mjlab独有'的说法)
- ObservationGroupCfg字段=terms/concatenate_terms/concatenate_dim/enable_corruption/history_length/flatten_history_dim/nan_policy/nan_check_per_term;ObservationTermCfg字段=func/params/noise/clip/scale/delay_min_lag/delay_max_lag/delay_per_env/delay_hold_prob/...
- GaussianNoiseCfg in mjlab.utils.noise,字段mean/std/operation ✓
- grep UniLab源码: run_reward_dispatch/TransitionBootstrapContract/PenaltyCurriculum/get_dr_capabilities/DomainRandomizationCapabilities/resolve_sim2sim_config/CrossBackendIncompatibleError 全部命中真实文件(非编造)
- run_reward_dispatch签名: scales/fns/ctx/ctrl_dt齐全,docstring明示'scales × fns(ctx) reduction'且'Returns reward * ctrl_dt' —— 与chapter描述逐字吻合
- UniLab action_scale: float=0.25默认(go2w/base.py)✓;公式 last_actions*action_scale+default_angles(tracking_obs.py:326)✓
- 观测delay机制实跑确认: observation_manager.py引用delay_min_lag共32处,actuator/actuator.py与builtin_group.py实际使用 —— mjlab 1.4.0原生支持per-term观测/执行器延迟

**工程核对（是这样/不符）**
- [准确] mjlab DifferentialIKActionCfg 全部扁平字段名(damping/delta_pos_scale/delta_ori_scale/max_dq/frame_type/frame_name/actuator_names/use_relative_mode)实跑introspection逐一吻合,默认值damping=0.05、max_dq=0.5也与chapter标注一致(L334-336,L365-366)
- [准确] mjlab DR包 mjlab/envs/mdp/dr/ 真实存在,dr.body_mass/geom_friction/pd_gains 签名(ranges+operation,pd_gains用kp_range/kd_range)与chapter L1034-1058完全一致;geom_friction默认operation='abs'、body_mass默认'scale' 也对
- [准确] ObservationGroupCfg.enable_corruption 确为组级字段(在group而非term),ObservationTermCfg有noise/clip/scale —— chapter核心论断'enable_corruption是组级总闸'(L276,L293)实跑证实
- [准确] GaussianNoiseCfg在mjlab.utils.noise,mean/std字段齐(L903) —— 正确
- [准确] UniLab所有被点名的源码符号(run_reward_dispatch/TransitionBootstrapContract/PenaltyCurriculum/get_dr_capabilities/DomainRandomizationCapabilities/resolve_sim2sim_config/CrossBackendIncompatibleError)均在真实文件中命中,非编造API;run_reward_dispatch的'加法聚合×ctrl_dt'(L680,L698)与docstring逐字吻合;action_scale=0.25默认(L412)正确
- [准确] mjlab训练实跑正常: G1速度2537 steps/s、Lift-Cube 689 steps/s,loss曲线健康,印证'PPO足够'(L25)的工程主线;ActionManager/RewardManager(action_rate_l2)按chapter所述工作
- [轻微不符] chapter多处称mjlab的观测延迟'须自定义DelayedObservationBuffer'(L962,L979,L1417附近),但实跑确认mjlab 1.4.0 ObservationTermCfg原生带delay_min_lag/delay_max_lag/delay_per_env/delay_hold_prob等字段且已在observation_manager中实装(32处引用)——mjlab其实有内置per-term观测延迟,此处描述过时
- [符合但不完整] chapter的IK示例把delta_pos_scale/delta_ori_scale显式设为0.05/0.1是对的,但未提醒读者mjlab默认值是1.0(未缩放)——直接省略scale会得到危险的大动作,属footgun未点破

**优化点 / 高级技巧**
- mjlab已内置观测/动作延迟(ObservationTermCfg.delay_min_lag/max_lag/per_env/hold_prob/update_period),应替代章节里手写的DelayedObservationBuffer与PerceptionNoiseManager延迟部分——一行配置即可做per-env随机延迟,比自维护buffer更省且不会漏reset(还能顺手消解L989的'延迟buffer未清空'陷阱)
- mjlab ObservationGroupCfg原生有history_length/flatten_history_dim;章节用last_action手搭动作历史(L273),可直接用group级history_length得到N步堆叠观测,更通用(对time_to_impact等时序量尤其有用)
- ObservationGroupCfg有nan_policy/nan_check_per_term内置NaN守卫;章节多处担心IK发散致NaN(L443,L1366),应介绍这个内置防护作为'IK数值炸了不至于整批崩'的兜底,而非只靠调大lambda
- DifferentialIKActionCfg有position_weight/orientation_weight/joint_limit_weight/posture_weight/posture_target(加权+零空间姿态正则IK),章节只讲了基础DLS;对冗余臂/全身IK,posture_target可把'举拍ready pose'编码进零空间,比单独加give-up reward更直接——这是被忽略的高级用法
- mjlab dr包远比章节展示的丰富(dof_armature/dof_frictionloss/joint_default_pos/encoder_bias/effort_limits/jnt_range等40+项),章节执行gap只用了pd_gains;encoder_bias/effort_limits/dof_frictionloss恰好对应章节L1022'关节延迟/饱和/backlash'那一行的sim2real gap,可直接用内置项而非泛泛带过
- 应提醒delta_pos_scale/delta_ori_scale默认=1.0(未缩放);新手沿用默认会让50Hz下单步动作过大、关节跳变,这是比lambda更常踩的坑,值得一条pitfall

**教学缺口（只讲代码没教工程之处）**
- 全章无任何可直接跑通的最小实跑命令:Quick Start(L72)是纯算术,'运行验证'小节(L432,L650,L1087,L1526)都只描述'应看到什么',但mjlab/UniLab的train入口、list-envs、import自检这些'从零起步第一步'一个真命令都没给——读者学完仍不知如何在真机上敲第一行(实跑证明这些命令存在且好用,应补)
- 章节坦诚网球task是示意(无现成env),但没给'如何在mjlab里从一个stock task(如Lift-Cube)改造出strike task'的脚手架路径;实跑显示Lift-Cube就是最接近的manip+IK起点,教学上可指'照Lift-Cube的EnvCfg改'比凭空写TennisStrikeEnvCfg(L1444)更能教会举一反三
- staged reward的σ/τ/t_ready/各权重(L471表,L500,L607)给了建议值却没教'怎么从contact rate曲线反推该调哪个参数到多少'——有诊断流程图(L666)但停在定性'增大/减小',缺一次真实调参walkthrough(如σ从0.5调到0.3 contact rate怎么变)
- 大量代码块(StrikeEventBuffer/PerceptionNoiseManager/LagrangianPPOForTennis/TennisFrequencySeparation)是完整类实现,但都无法在任何框架直接实例化(依赖env.perception/event_buffer等虚构属性);属'只讲代码不能跑',应明确标注为伪代码骨架或给出最小可运行替身,否则读者照抄必报AttributeError
- 三框架对比是本章亮点且API核对扎实,但'为什么选mjlab而非Isaac/UniLab做主线'的工程权衡(启动速度/显存/接触精度/调试便利)没讲——实跑中mjlab秒级起、Isaac Sim数分钟起、UniLab CPU物理,这种一手体感正是选型教学最该传递的,却缺席

**notes**：总评:本章工程严谨度高于一般教材,所有load-bearing的mjlab/UniLab API(类名/字段/默认值/函数签名)经gpufree实跑introspection逐一核对,命中率近乎100%,finenote(L1701)自称'实跑introspection核对'属实、非虚标;UniLab对照尤其难得,点名的7个源码符号全部在真实文件命中,未编造等价API。三框架训练实跑均能起、loss正常。主要可改进项是两类:(1)轻微过时——mjlab其实内置了per-term观测延迟/历史/NaN守卫/加权姿态IK,章节把它们当成'须自实现',应改为推荐内置设施;(2)教学性缺口——全章无一条可照敲的真实命令、核心类多为不可实例化的伪代码、调参从曲线到数值的闭环walkthrough缺失,这些是'教代码vs教工程'的典型短板。无任何编造API或数值错误需紧急修正;建议主控按optimizations补内置延迟/历史/IK姿态正则、按pedagogyGaps补一条端到端可跑的mjlab最小命令链。所有结论基于实跑,未改动任何.tex。

---

## prac_diagnostics

**服务器**：gpufree (RTX4090; mjlab 1.4.0 / rsl-rl-lib 5.2.0, Isaac Lab 2.3.2 / rsl-rl-lib 3.1.2, UniLab 三框架齐全)

**实跑命令（10）**
- list-envs + 版本：mjlab 1.4.0 / rsl-rl-lib 5.2.0，12 任务仅 G1/Go1 无 Go2 → 与章节 §配置/note(L112) 完全一致
- 源码核对 rsl_rl/algorithms/ppo.py：entropy_coef=0.01 / max_grad_norm=1.0 / desired_kl=0.01 / schedule=adaptive → 与 pitfall(L239)+版本表(L1640) 一致
- 源码核对 ppo.py L243-246 自适应KL bang-bang：kl>desired*2→lr=max(1e-5,lr/1.5)；kl<desired/2→lr=min(1e-2,lr*1.5) → 与 codebox(L491-498) 逐字一致
- WANDB_MODE=disabled train Mjlab-Velocity-Flat-Unitree-Go1 --num-envs 64 --max-iterations 2：跑通，1892→2526 steps/s，value/surrogate/entropy 正常，entropy=17.01 nats、action std=1.00 → 证实 §entropy d=12,σ0=1→≈17 的公式(eq L267,L535)
- 源码核对 rsl_rl/utils/logger.py：L183 add_scalar(f'Loss/{key}')、L187 'Policy/mean_std'、L174 含'/'的extras原样写 → 证实 mjlab 键名(L246-258)
- 源码核对 mjlab/managers/reward_manager.py L108 extras['Episode_Reward/'+key]、L41 注释'scaled by dt' → 证实分项reward键 Episode_Reward/<term> 与 ÷dt 机制(L256,L331,L429)
- 源码核对 mjlab tasks/.../rl_cfg.py：obs normalization 开关名是 obs_normalization（actor/critic 各一），并非 actor_obs_normalization → 与章节 L991/L1008 claim 不符（见 findings）
- rl_cfg.py 同时证实 value_loss_coef=1.0/clip_param=0.2/gamma=0.99/lam=0.95/init_std=1.0/std_type=scalar → 与版本表(L1640) 一致
- UniLab PPO：train --algo ppo --task go2_joystick_flat --sim mujoco algo.max_iterations=1 algo.num_envs=16 training.no_play=true：跑通，控制台逐项 reward/tracking_lin_vel...reward/swing_feet_z，落 git/UniLab.diff → 证实 §UniLab reward/<term> 分派(L287,L976) 与 git diff 元数据(L982)
- Isaac：./isaaclab.sh -p 取 rsl-rl-lib=3.1.2；源码 actor_critic_recurrent.py L26-27 有 actor_obs_normalization/critic_obs_normalization；on_policy_runner.py L216 'Policy/mean_noise_std'、L212 f'Loss/{key}'、L231 'Train/mean_reward' → 证实 Isaac 键名差异表(L948-949) 与归一化开关(Isaac侧正确)

**工程核对（是这样/不符）**
- 不符(载重): L991 与 L1008 称 mjlab(5.2.0) 的 obs 归一化开关是 actor_obs_normalization/critic_obs_normalization 且‘源码核对、两版同名’——实测 mjlab 5.2.0 经 RslRlModelCfg 暴露的字段是 obs_normalization（actor=/critic= 块内各一个布尔，rl_cfg.py:16/26），rsl_rl 5.2.0 包内 grep 不到 actor_obs_normalization；该名仅在 Isaac 3.1.2 的 ActorCritic 成立。属版本漂移，建议改为‘Isaac3.1.2=actor/critic_obs_normalization；mjlab5.2.0=RslRlModelCfg.obs_normalization’
- 是这样(强证): 自适应KL bang-bang 控制器(L491-498 codebox 及 L1640 版本表) 与 ppo.py L243-246 逐字一致(阈值×2/÷2、lr×1.5/÷1.5、界[1e-5,1e-2]、解析高斯KL via get_kl_divergence)
- 是这样: 五大默认超参 entropy_coef=0.01 / max_grad_norm=1.0 / desired_kl=0.01 / clip_param=0.2 / value_loss_coef=1.0 / gamma=0.99 / lam=0.95 全部源码命中(ppo.py + go1 rl_cfg.py)，pitfall(L239) 关于 0.01 而非 0.001 成立
- 是这样: mjlab 键名 Loss/value、Loss/surrogate、Loss/entropy、Loss/learning_rate、Policy/mean_std、Train/mean_reward、Perf/total_fps、Episode_Reward/<term> 全部经源码或实跑证实；Loss/value vs value_function 的差异由 loss_dict 键(而非 runner) 决定，章节机理描述正确
- 是这样: Isaac 3.1.2 键 Policy/mean_noise_std、Train/mean_reward、f'Loss/{key}'、actor/critic_obs_normalization 全部源码命中；rsl-rl-lib=3.1.2 实测，与章节差异表完全吻合
- 是这样: KL 不单独 add_scalar、只驱动 lr(看 Loss/learning_rate 锯齿) — logger/runner 源码均无独立 KL scalar，证实 note(L219,L959)
- 是这样: mjlab 无 Go2、仅 G1/Go1、12 任务、rsl-rl-lib 5.2.0、mjlab 1.4.0 — list-envs 与版本实测全部吻合(L112,L1644)
- 是这样: EmpiricalDiscountedVariationNormalization 确在 rsl_rl.modules 中导出(章节称其仅 RND 内在奖励用，L424)；核心 PPO 主任务 reward/value 不归一化的论断与库结构一致
- 是这样: UniLab PPO 走 rsl-rl OnPolicyRunner(控制台同款 Mean value/surrogate/entropy loss)，分项 reward 经 reward/<term> 入 info['log']，落 run 目录 + git/UniLab.diff，与章节(L976-982) 吻合；真实任务名 go2_joystick_flat 有效(无 Go2 限制仅限 mjlab)
- 教学性副证: 实跑 Go1 entropy=17.0152 与章节 ‘d=12,σ0=1→≈17 nats’(L267 公式,L535 表) 数值吻合，eq(4.x) 的初值预测可直接被一条 2-iter 训练验证

**优化点 / 高级技巧**
- obs_normalization 一节可补‘怎么自己查开关名’的工程动作：mjlab 用 python -c 看 RslRlModelCfg 字段 / 读 run 目录 config.yaml，而非记名字——正好呼应章节自己提倡的‘以本机实际加载值为基线’(pitfall L239)，把版本漂移变成可操作技能
- Quick Start 的 mjlab 训练命令(L72) 用 --agent.logger wandb，但全书已知无 key 会崩；可补一行最稳的本地起手式 WANDB_MODE=disabled ... 或 --agent.logger tensorboard，让读者第一条命令必跑通(实跑证实 disabled 必通)
- 可补‘如何自加一条 KL 数值曲线’的最小代码：章节多处说‘要 KL 曲线须自行加 add_scalar’，但只在 rsl_rl 是 f'Loss/{key}' 动态键的前提下，最省事是在 env wrapper 往 extras 塞 'Loss/kl' 键(logger 见含/原样写出，L174 已证)，可直接落盘，无需改 rsl_rl 源码——这是比‘改 runner’更轻的现代做法
- 分项 reward 的 Episode_Reward/<term> ÷ max_episode_length_s 这一‘系统性低估短episode’事实(L429) 很关键但只用文字讲；可补一张‘同一 run 里 Episode_Reward/<term> 与 per-step 真值’的对照小实验配方(play 时打印 term raw 值 vs 面板值)，让读者亲眼看到分母效应
- 三框架键名差异可加一句运维级提示：跨 mjlab↔Isaac 复制 WandB 面板时，把 panel 配置里的 Loss/value↔Loss/value_function、Policy/mean_std↔mean_noise_std 做成一张 sed 映射，省得手改——属面板复用的高级技巧
- Isaac 侧 Perf 键名实测为 'Perf/collection time'(含空格) 与 'Perf/learning_time'(下划线) 混用(runner L218-219)，是真实的不一致坑；若章节将来列 Isaac Perf 键，值得标注此 typo 以免读者搜不到

**教学缺口（只讲代码没教工程之处）**
- 归一化开关名错(actor_obs_normalization)恰恰暴露教学法可强化点：章节反复教‘版本漂移要以本机为准’，却在 L991/L1008 自己把一个随版本变的字段名钉成‘两版同名、源码核对’——应身教一致，给出‘查字段名’的命令而非记忆值，这比单纯改对名字更能教会读者举一反三
- L2C2 / checkpoint forensics / reward decomposition / Lipschitz 检测等多段是高质量但‘悬空’代码(compute_l2c2_loss、forensics_analysis、check_lipschitz_sensitivity 均依赖未给出的 actor_critic.act/evaluate、evaluate_checkpoint)，读者无法照抄即跑；缺‘如何挂进 rsl_rl 的 update()/如何拿到 next_obs’的接入点说明，属‘给了零件没给装配图’
- 贯穿案例 Phase0-5.5 命令几乎全用示意任务名 My-Velocity-Rough-Go2-v0(章节已诚实声明是自建占位)，但没给‘如何从 list-envs 真实任务(Go1)最小改出一个 rough 自建任务’的从零搭建路径；初学者卡在第一步无法把 Phase 流程真正跑起来——可补一个用现成 Mjlab-Velocity-Rough-Unitree-Go1 走完 Phase0-3 的可执行替身
- 症状索引表/九种模式信息密度极高且‘只读不练’：表给了症状→参数→章节，但多数行没有‘怎么确认这就是该症状’的一句话判据(如何 30 秒读出 KL 锯齿、如何在面板叠加 Episode_Reward/*)；建议每类至少配一个‘最小判据命令’把查表能力落到操作
- AGILE 三大 GUI / motion-quality / scaled-dict 等大量内容来自单篇 arXiv:2603.20147(2026-03)且 mjlab/UniLab 均无现成等价物(章节已诚实标注),教学上偏‘介绍论文’而非‘教你搭’；对‘没有这套 GUI 的读者怎么用 uv run demo 自写最小预检脚本’只给了一句话，缺可操作骨架——这是工程可迁移性的缺口

**notes**：总体：本章工程准确度很高，三框架(mjlab/Isaac/UniLab)均在 gpufree 实跑通过，自适应KL bang-bang 块、五大PPO默认值、Loss/Policy/Train/Episode_Reward 键名族、KL不单独记、UniLab reward/<term>分派与git-diff元数据、entropy≈17nats 等核心声明全部被源码核对或2-iter训练证实，且诚实标注(无Go2/UniLab无预检GUI/不内置constrained PPO)均属实。唯一载重不符：L991+L1008 把 obs 归一化开关名钉为 actor_obs_normalization/critic_obs_normalization 并称‘mjlab5.2.0与Isaac3.1.2两版同名、源码核对’——实测该名仅 Isaac3.1.2(ActorCritic)成立，mjlab5.2.0 经 RslRlModelCfg 暴露为 obs_normalization(rl_cfg.py:16/26)，rsl_rl5.2.0包内无此名。建议主控据此分版本改写该两处。未改任何 .tex。教学缺口集中在‘高质量但悬空的工具代码缺接入点’与‘示意任务名挡住从零搭建第一步’，可用现成 Go1 任务补一条可跑替身。

---

