# P8「强化学习运动控制」实践优化记录

> 据 gpufree/autodl 两路工程实跑验证报告（`docs/RL运控融合_工程验证_{gpufree,autodl}.md`）逐章优化。
> workflow `rlmc-p8-practice-optimize` run `wf_3e19bccf-f47` + resume `wodekuf8h`（28 章；21 缓存 + 7 续修）。
> 每章：修硬不符（实跑确认的真 bug）/ 补教学缺口（从零搭建·排错过程）/ 补高价值优化。**0 新 bib；cite 全闭合；begin/end 平衡**。

**合计：硬不符修正 157 · 教学缺口 100 · 优化 86（共 28 章）**

## prac_ecosystem.tex（P8 强化学习运动控制 / 具身RL仿真生态：mjlab、Isaac Lab 与 UniLab）
**硬不符修正 (2)**
- 无硬不符可修：报告 ## prac_ecosystem 的「工程核对」明确写『全部实跑通过、未发现任何不符』，13 组三框架实跑（版本/任务清单/训练命令/API名/配置/奖励名/注册机制）逐一核对全属实，notes 亦确认『未发现任何编造或不符』——按纪律『标是这样/属实的不动』，故未做任何 bug 改正。
- 唯一修正：删除我自己在 line 756 sanity_check pitfall 末尾误引入的一个多余右花括号（『而不是猜}。}』→『而不是猜}。』），使全文花括号配平 diff=0。
**教学缺口补充 (4)**
- 教学缺口1(zero/random 只讲概念没给可看输出)：在 ins:rlmc-eco-smoke-read 落到三处可观测信号——Viser localhost:8080 看站姿、是否炸飞/NaN、首行训练日志盯 steps-s+value loss+reward 三个数，并给判读与对应处置。
- 教学缺口2(sanity_check 是 Isaac 风格 obs_dict['policy'] 照搬 mjlab 会 KeyError)：把 line 732 诊断脚本改为组名自适应 next(k in policy/actor/obs) 且先 print 真实组名，另加 pitfall 点明三框架组名差异与『先 print(keys) 再取值』习惯。
- 教学缺口3(全章缺『从零搭一个新任务』最小步骤,练习只加一行 RewTerm)：新增 subsection 从零起步(sec:rlmc-eco-fromscratch)，给 mjlab/Isaac/UniLab 各自『复制最近邻任务→改唯一注册名→list-envs 确认→2迭代冒烟』可复制命令 + ins:rlmc-eco-copyfrom 讲为何复制胜过白纸手写，并把练习首条改为『复制 velocity 任务注册成功再冒烟』。
- 教学缺口4(故障表只列症状缺排查过程)：故障排查手册后新增两段真实排查命令序列——OOM→watch nvidia-smi+num-envs 砍半二分、KeyError→print(obs_dict.keys()) 看真名，加 ins:rlmc-eco-debugmethod 讲『把书里名字/数字当待验证假设、跑一次问框架要真值』。
**优化补充 (4)**
- 优化点1(真实 steps/s 基准 + 如何自测吞吐)：steps/s 表后新增 ins:rlmc-eco-throughput——RTX4090 64envs 实测仅 ~1.6-2.5k steps/s(远低于 4096envs 的 ~10^5)、首迭代含 JIT/CUDA-graph 预热最慢，教读者读训练自带稳态 steps/s 行做性能自检与回归基准。
- 优化点2(Isaac Sim 启动时间量化预期)：新增 pitfall 给量化预期——Cartpole 实测首跑 >5 分钟(shader/USD 一次性编译缓存,二次更快)、建议 --headless+预留 5-8 分钟别中断；并把系统表(line 38)与对比表的『数十秒』订正为『数十秒；首跑≥5 分钟编译』。
- 优化点3(裸 python import isaaclab 缺 pxr 报 ModuleNotFoundError)：同 pitfall 补『快速查 API 不能用系统 python、必须 ./isaaclab.sh -p 且先 AppLauncher』这一新手必踩的环境隔离坑。
- 优化点4(UniLab off-policy 实跑入口)：ins:rlmc-eco-ppo 后新增 note，给 ppo/appo/flashsac 三条仅改 --algo 的可复制命令，落地『算法层可独立替换』，让异步/off-policy 从断言变可上手。

## prac_manager_arch.tex
**硬不符修正 (6)**
- 代码走读 codebox(原 L226): max_iterations Go1 由 30000 改为实测 10_000(并把注释从『实测30000』更正为『mjlab 1.4.0 Go1 实测 10_000』)
- 实测核实段(原 L241): 把『max_iterations 按任务定 G1=30000』更正为『mjlab 1.4.0 的 Go1 实测为 10_000』,消除被误标为『已实测核实』的过时数字
- ActionManager codebox(原 L875): 不存在的 self.entity.write_joint_targets(torques) 改为真实 API self.entity.set_joint_position_target(self.processed_action),并加注 write_joint_position_to_sim 备选、点明无 write_joint_targets 此名
- entityapi 对照表(原 L1073): mjlab 列的 write_joint_targets(...) 改为 set_joint_position_target(...),行首改为『设置关节位置目标』
- EntityData 表(原 L1047): 删除 mjlab 1.4.0 不存在的 body_pos_w,替换为真实的 body_link_pos_w(连杆原点)/body_com_pos_w(质心)两行
- 练习2(原 L1094): 把引用 body_pos_w 改为 body_link_pos_w 取足端 z 分量,并提示想清为何用 _link_ 而非 _com_
**教学缺口补充 (4)**
- 新增 insight『一行列出你这版全部字段』(ins:rlmc-mgr-introspect, EntityData 表后): 给可复制的 [a for a in dir(robot.data) if not a.startswith('_')] 自查法 + 读 data.py dataclass 字段,把『学查法不背名字』落到最易漂移的 API 表上
- 新增 insight『max_iterations 这种实测锚点务必自查』(ins:rlmc-mgr-recheck): 教读者打开本机 rl_cfg.py 读 max_iterations=,或用 `mjlab train ... --help | grep max-iterations` 一行打出当前生效值(含预期输出 default:10000),把复核路径示范出来
- 新增小节『一次真实排错: 从 reward 不涨到定位失效 term』(sec:rlmc-mgr-debugwalk): 把诊断表步1-3 落成可照抄命令序列——`WANDB_MODE=disabled mjlab train ... --agent.max-iterations 2` 看 Episode_Reward/<term>,给出 feet_slip=0.0000 即定位失效 term 的真实输出样例与收敛逻辑
- 新增表『首跑必盯的三行终端输出』(tab:rlmc-mgr-firstrun): Steps per second / Value function loss / Episode_Reward/<term> 三行的健康样子 vs 不对劲信号,把首跑成功判据写成可操作清单
**优化补充 (4)**
- EntityData 表后补段『_link_ 与 _com_: mjlab 1.4.0 把每个位姿/速度拆成两套』: 讲清连杆原点 vs 质心的物理区别与各自该用场景(几何/落点用 _link_、动量/力用 _com_),把过时单名升级为比 Isaac 更细的可教工程区分
- speedup 表后补段+codebox『单项 reward 消融: 一行 CLI 覆盖权重』: 给 `--env.rewards.feet-slip.weight 0.0` / `--env.rewards.track-angular-velocity.weight 1.5` 实测命中命令,点明连字符↔下划线 tyro 约定与 deepcopy 不污染原 cfg,把 ins:rlmc-mgr-value 的『改一行 config』从断言变成可复现操作
- 调试小节内补 note『三框架 reward 日志前缀不同』: mjlab/Isaac 是 Episode_Reward/<term>、UniLab 实测是 reward/<term>(无 Episode_ 前缀),提示跨框架查曲线别搜错命名空间
- 三步流程后补 note『uv run train/play 还是直接 train/play』: 说明 mjlab 把 train/play 注册为 console_scripts 入口,装好后可直接敲不必前缀 uv run(本服务器即直接用入口脚本),降低读者照搬 `uv: command not found` 的失败率

## prac_obs_action.tex（观测与动作空间设计）
**硬不符修正 (1)**
- 报告对本章工程核对全部为'是这样'（三框架所有 load-bearing API 名/字段/源码路径/方法学逐项实跑核实通过，未发现任何编造或不符；notes 明确'engineeringFindings 全为正面确认,无需主控修正 .tex'）——故本章无硬不符可改，未臆造任何'修正'，仅做教学缺口与优化补充。
**教学缺口补充 (4)**
- sec:rlmc-fromzero（新子节，插在七步 smoke test 前）：补'拿到一台新机器人从空 cfg 到第一个能跑任务'的最小骨架五步（先放哪5项最小 actor 观测→critic 只加 base_lin_vel 一项特权→关节位置+默认偏置→scale 用 random agent 反推校准而非查表→manager repr 核对维度后再训），并点明工程真实起点是'复制最近的现成任务再改'
- ins:rlmc-firstwire（新 insight）：把上述五步压成'第一次接线 checklist'六条可勾选项（actor 仅部署可得?命令进 actor?critic=actor+低维特权且关 corruption?默认偏置 zero 下是站姿?random 下 processed 落限位中部?actor/critic 维差=特权维度和?），作为四原则在搭建期的落地形态
- 新增'一次真实排错' practice（在诊断节）：给三段服务器可复现的命令序列——场景一 KeyError（极短训练暴露 group 名错+照栈里打印的真实 group 名改 obs_groups）、场景二能跑不学习（看哪个 Episode_Reward/<term> 恒零定位失效 term，含三框架日志前缀差异）、场景三 OOM（nvidia-smi -l 实时看显存+先砍 num_envs 后砍 history），每段带可复制命令+预期输出+读法，把'诊断流程'落成键盘动作
- ins:rlmc-uniselfcheck（新 insight，插在 sec:rlmc-groups UniLab codebox 后）：把全章散落的 UniLab'非声明式=你得自己保证正确'告诫收成一张'手写 _compute_obs 自检清单'六条（每项加噪?命令进 actor?critic 是干净值?特权只在 critic?切片顺序对齐 deploy obs_layout?无副作用?）
**优化补充 (3)**
- ins:rlmc-managerstr（新 insight，诊断节）：补'别手写打印维度——print(env.observation_manager/action_manager) 的 repr 自带逐项 shape 表'，引报告实跑值 mjlab rough G1 自动打出 actor=286/critic=298（差12=foot_height 2+height_scan 187 等特权），作为 Bug2'打印 actor/critic 实际维度'的现成探针（维度相同≈特权没进 critic）；UniLab 无 manager 对象则看训练启动日志的 MLP 首层 in_features（go2 实测 obs 49/critic 52）
- ins:rlmc-healthy-range（新 insight，诊断节）：补两个实跑量化锚点——random agent 下 processed 关节目标健康区间为限位中部 50%-80%偶尔触界（贴满 100%调小/挤在<20%调大）；steps/s 量级：小 env 数 CPU/启动主导属正常（mjlab 64env 1.6k→稳态~2.5k、UniLab 16env~0.6k，别被首迭代低值吓到），真正告警是'同规模吞吐突掉一个数量级'→疑 Python for 循环
- note '--agent trained 回放 checkpoint'（smoke test 节）：补 play --agent 实测三值 {zero,random,trained}，trained 是加载已训 checkpoint 在 Viser 回放、部署前看步态发抖/打滑的常用入口（Isaac 对应 play.py --checkpoint、UniLab 对应 eval --load-run -1）

## prac_privileged
**硬不符修正 (10)**
- 环境配置指南-引言(L23): 原把 actor/critic 解耦框成『RSL-RL 4.x 破坏性更新』并写统一→per-model 升级，改为按框架分形态——mjlab 用 per-model RslRlModelCfg、Isaac Lab 实测仍是统一 RslRlPpoActorCriticCfg（实测 rsl-rl-lib 5.2.0），照搬一套到另一框架会 ImportError。
- 环境配置表(L33)+版本信息表(L1587): RSL-RL 版本由『≥4.0』改为实测『rsl-rl-lib 5.2.0』，能力描述由『actor/critic 解耦』改为『非对称 actor/critic』。
- 版本错配 pitfall(L40-42): 原『在 RSL-RL ≤3.x 上写 4.0 的 RslRlMLPModelCfg』完全重写为『把 mjlab 的 per-model cfg 照搬到 Isaac Lab』——实测 RslRlMLPModelCfg 在两框架都不存在、RslRlModelCfg 只在 mjlab、RslRlPpoActorCriticCfg 只在 Isaac Lab。
- 版本核对 codebox(L49): 期望值由『rsl_rl>=4.0.0』改为『实测 5.2.0』，并补 import 判别命令教读者辨别自己装的是统一还是 per-model 形态。
- Quick Start critic_terms codebox(L64-73): 删掉不存在的 friction_coefficients term，换为实测存在的 foot_height（取自 mjlab.tasks.velocity.mdp），并注明 friction 真值 term 全仓不存在。
- Quick Start 维度自检 codebox(L85-91): gym.make('Mjlab-...').reset() 跑不通(mjlab 不走 gymnasium)，改用 mjlab.tasks.registry.load_env_cfg 取 cfg + 训练日志读 in_features。
- Quick Start \pz 注(L93): term 名说明由 foot_contact_forces/friction_coefficients 更正为 foot_contact_forces/foot_height，并点明 friction 类 term 不存在、给出 dir() 自查命令。
- RSL-RL 解耦大节(sec:rlmc-priv-rslrl4, L594-603): 标题由『RSL-RL 4.0：actor/critic 解耦』改为『RSL-RL 配置：actor/critic 网络两形态(mjlab vs Isaac Lab)』；原单一 RslRlMLPModelCfg codebox 拆成 mjlab(RslRlModelCfg) 与 Isaac Lab(RslRlPpoActorCriticCfg) 两个实测正确 codebox，并补张冠李戴 pitfall。
- 维度一致性检查 codebox(L610-618): 两处 gym.make('Mjlab-...')/gym.make('Isaac-...') 伪代码改为 mjlab registry + Isaac Lab 看启动日志/deprecation warning(obs_groups 缺 policy/critic 键回退用 policy set，键名 policy 非 actor)。
- 本章目标(L120)/知识点表第8项(L1507)/normalizer 杀手节问题一(L1243): 三处『RSL-RL 4.0 解耦』表述改为『mjlab per-model / Isaac 统一两形态』，消除 4.0 误标。
**教学缺口补充 (2)**
- 新增 sec:rlmc-priv-fromzero『从零搭一个非对称任务的最小步骤(工程真实路径)』: 教读者复制框架自带任务(python -c 打印 env_cfgs.py 路径)→先原样跑通基线(给实测 1480-2376 steps/s 锚点)→只改 critic_terms→按本框架形态改 hidden_dims→跑 1 迭代看 in_features 验证维度差，每步都有可回退基线；正补报告点名的『从零搭起』缺口。
- 新增 sec:rlmc-priv-realdebug『一次真实排错:从「critic 维度不对」到定位』: 给三条可复制 bash 命令+预期输出——dir() 自查特权函数名(替代背书名字)、训练日志 grep ActorCritic 读真实 in_features(替代跑不通的 gym.make)、据 C-A 是否为 0 二分定位；落实报告点名的『排查过程而非症状表』缺口，并提炼『先问框架哪行输出能证明生效』方法论。
**优化补充 (3)**
- 在 sec:rlmc-priv-rslrl4 与维度自检处补 obs_groups 键名按框架不同的实测点(Isaac Lab 算法侧是 policy/critic 非 actor，缺键会打 deprecation warning 回退用 policy set 兼当 critic)，并补 mjlab 真实维度自检入口(load_env_cfg + 训练日志 in_features)。
- checklist 第10条补 mjlab ObservationTermCfg 实测真有的延迟字段 delay_min_lag/delay_max_lag/delay_hold_prob，并给出 lag(env step) × decimation × sim_dt = 物理时间 的换算，把『delay lag 换算合理』从空话变成可操作配置。
- 多处补 mjlab Go1 velocity 实测吞吐基准(1480-2376 steps/s、value loss≈0.02-0.03)作为『先跑通基线再改』的健康锚点。

## prac_visuomotor
**硬不符修正 (5)**
- Quick Start (sec:rlmc-vm-quickstart, 引言段+新增 mjlab codebox 行95-113): 报告'不是这样'——章节用占位符<你的视觉任务>暗示 mjlab 无现成视觉任务可跑;改为如实说明 mjlab 自带 Mjlab-Lift-Cube-Yam-Depth/-Rgb/Multi-Cube-Seg-Yam 三个真视觉任务,并给一条实测可复制命令(MUJOCO_GL=egl train ...-Depth, ~5200 steps/s)。
- CameraSensorCfg 字段(note 行72 + codebox 行411-430): 报告'不全'——章节字段表缺 use_textures/use_shadows/enabled_geom_groups/orthographic/clone_data 5 个真实字段;补全这 5 项并加上'同场景所有相机 use_textures/use_shadows/enabled_geom_groups 必须一致'的硬约束(源码 docstring)。
- data_types 构造期校验(note 行72): 报告实测 CameraSensorCfg(data_types=('lidar',)) 当场抛 ValueError;补'构造期即校验非法值,非运行时才报错'。
- segmentation 输出形状(note 行72 + codebox 行427): 报告实测 segmentation=[N,H,W,2] int32 而章节未给;在 note 与 codebox 输出注释里补上该形状。
- 归一化公式偏差(新增 note 行564-571 + 部署链路隐含): 报告'偏差'——mjlab 生产 camera_depth 用 d/cutoff(=d/far),与章节 eq:rlmc-vm-norm 的 (d-near)/(far-near) 不同,读者照章节公式做部署会与 mjlab 内置 obs 对不上;新增 note 并排两公式、点明'部署侧必须用与训练同一条公式',消除 sim2real 隐患。
**教学缺口补充 (4)**
- 从零跑通第一帧(Quick Start 新 codebox 行99-113): 给一条真能复制粘贴、立即出 ~5200 steps/s 结果的 mjlab 视觉任务命令 + 预期输出(reward 非 NaN/depth 在动/RGB 版含 camera_d405_rgb (3,32,32) CHW + geom_rgba DR),补上原 Quick Start 全占位符导致的'端到端跑通'肌肉记忆缺位。
- 从预处理函数到 actor group 的接线一步(新增小节 sec:rlmc-vm-wire 行573-596 + ins:rlmc-vm-wire): 报告缺口——章节教了 preprocess_depth 又在 checklist 要求'depth 进 actor group',中间'怎么把函数注册成 obs term'从未演示;补 mjlab ObservationTermCfg(func=camera_depth, params={cutoff_distance})+ ObservationGroupCfg 真实接线 codebox,并点明三段链路(预处理函数→term 注册→放进 actor group)缺一则 CNN 第一层梯度恒零。
- 工程判断:生产代码省了什么、凭什么敢省(新增 note 行564-571): 不只给学院派完整公式,而是用 mjlab 生产 camera_depth 教读者'为什么敢省近偏移(near≪far 时两公式几乎重合)''为什么敢省 NaN 处理(仿真 depth 几何精确无空洞,但换真机/加 dropout 就必须补回)'——教会'何时可简化'的工程判断。
- 如何自己测'渲染慢 5-20 倍'(sec:rlmc-vm-camera-verify 新增段+codebox 行498-512): 报告缺口——'5-20倍'贯穿全章却没教怎么测;补一条铁律(必须同一任务加/不加相机对照,只切'是否渲染'一个变量)+ 可复制命令(读稳态 steps/s,跳过被 JIT/CUDA-graph/USD 预热污染的首迭代)+实测基准(RTX4090 小 env 数下吞吐远低于峰值,首跑别被低数字吓到),并说明跨任务比 steps/s 无意义。
**优化补充 (4)**
- 真实 steps/s 基准 + 首跑预热提醒(sec:rlmc-vm-camera-verify 行509): 补 mjlab Depth 任务 RTX4090 实测 ~5200 steps/s、RGB ~1300 steps/s,并提示小 env 数(64)下远低于数千 env 峰值、首迭代被预热污染——让'渲染慢'从抽象区间落到可信实测。
- Isaac 相机配置类不能裸 import 的高发坑(新增 pitfall 行79-80): 补'python -c from isaaclab.sensors import TiledCameraCfg 直接 ModuleNotFoundError: omni.usd,必须先 AppLauncher(enable_cameras=True) 启动 Isaac Sim 才能 import',这是想快速查 API 的读者第一个撞墙处,原章节完全没提;并给'在已起 app 的脚本里 print(__dataclass_fields__)'的正确查字段姿势。
- depth 渲染性能优化技巧(CameraSensorCfg codebox 行428-430): 补 use_shadows=False(depth 不需阴影)+ enabled_geom_groups=(0,3)(排除装饰几何,真实 Yam 任务即用此值)两条省渲染技巧,作为 depth-only 任务的具体优化点。
- 引用 mjlab 生产函数作为真实 preprocess 范例(新增 note 行564-571): 把 mjlab.tasks.manipulation.mdp.observations.camera_depth 作为'真实世界的 preprocess_depth'引入,比纯手写示例更有说服力,也暴露'生产代码省了无效像素处理'这一可讨论点。

## prac_multimodal
**硬不符修正 (4)**
- Quick Start 第一步 CLIP(line 85 前): 报告实测 clip/open_clip 在 mjlab/UniLab/IsaacSim 三环境全 ModuleNotFoundError, '立即可复制+秒级'不成立 -> 新增前置安装框 'pip install ftfy regex git+.../CLIP.git'(或 open_clip_torch)并明确装进生成侧 env 非 tracker env。
- Quick Start 第二步 ProtoMotions(line 94): 报告实测 gpufree 无 ProtoMotions、第二列全部不可实跑(设定'三框架齐全'实为 mjlab+UniLab+IsaacLab) -> 在 codebox 顶部加 'ProtoMotions 须单独 clone+pip install -e .' 前置, 并新增'三列可跑性分级'note 明确 ProtoMotions 列命令仅形式核对、概念代码是脚手架非贴上即跑。
- mjlab tracker 喂 motion 整章空白(报告核对#3, 通篇当'发动机'却无可跑入口): 在 sec:rlmc-mm-calm-three 末尾新增最小可跑闭环 codebox 'csv_to_npz -> .npz -> mjlab train --env.commands.motion.motion-file /abs/x.npz', 并在 Quick Start 排错框给出裸跑报错原文 ValueError: provide --registry-name OR --env.commands.motion.motion-file。
- UniLab demo/train(line 106-112): 报告核对#5 demo 默认 sim=motrix(非 mujoco)、entry=eval, 且 g1_motion_tracking train 同样需 HF 资产(line 110 原缺标注, 离线必失败) -> codebox 内补 'demo 默认 MotrixSim 后端'说明、把 HF_ENDPOINT 镜像前置到 demo 与 train 两处并点名会拉 dance1_subject2_part.npz。
**教学缺口补充 (4)**
- 新增'第一个小时最常撞的三堵墙'pitfall(Quick Start 后): 用一段真实排错命令序列给出 ModuleNotFoundError clip / RuntimeError client closed(HF 超时) / mjlab ValueError 缺 motion-file 三个一线错误的'报错原文 -> 一行修法', 并教排错心法'先读报错最后一行再决定装依赖/设环境变量/补旗, 而非盲改代码'——直接补报告点名的'命令真跑不起来'排错缺口。
- 新增 mjlab tracker 最小闭环 codebox + 说明段(sec:rlmc-mm-calm-three 末): 教读者'从零把一段动作喂进 tracker'的最小步骤(csv_to_npz 落 npz -> 训练旗 -> 看 steps/s/value loss 冒烟), 并点明本章任何来源(CALM/TextOp/视频)产物走同一条 .npz 契约路、三框架喂入口异名同思想, 把'来源->tracker'从概念闭环升级为工程闭环。
- 新增三列'可跑性分级'note: 明确 UniLab 列照抄即跑、ProtoMotions 列需自装且概念代码是脚手架、mjlab 列 tracker 需补喂 motion——把 UniLab 列'可跑 vs 需自实现'的标注范式推广到另两列, 解决报告指出的'概念代码易被误以为贴上即用'。
- 故障排查手册新增 3 行命令级条目(表首): ModuleNotFoundError clip / RuntimeError client closed / mjlab tracking ValueError 缺 motion-file 各配排查步骤+相关节, 补齐报告所指'手册偏症状表、缺命令真跑不起来一线错误'。
**优化补充 (1)**
- 新增'训练正常起了吗'吞吐/显存基准 note(RTX 4090 实测): 给 UniLab go2 ~0.66 s/iter@16envs、mjlab 64envs 首迭代 1.5-1.8k 升稳态 ~2.5k steps/s、4096envs 才进 1e4~1e5 量级的真实基准(小 env 数受 CPU/启动主导, 别被低数字吓到), 并给 24GB 显存 num_envs=4096 OOM 边界与'先 64 冒烟再二分逼近'经验; 同时教读者用框架自带 steps per second 行做性能自检(无需另写计时器)。

## prac_manipulation (机械臂与灵巧手操作)
**硬不符修正 (8)**
- obs codebox (sec:rlmc-manip-obs): joint_pos/joint_vel 由 [N,7] 改为 [N,8]，并加注「按 nq 计，含双指；right_finger 虽被 equality 耦合成被动指仍计入 nq」——实测网络 in_features=29 非 27
- 维度计算段: 把 7+7+3+3+7=27 改正为 8+8+3+3+7=29，点破 nq(=8) vs nu(=7) 陷阱（被动指进 joint_pos_rel），并把'手算核对 27'改为'打印网络 in_features 自查'
- obscfg 表: 'actor obs 维度 =27（YAM 7 DOF）' 改为 '=29（关节项按 nq=8 非 nu=7）'
- LiftingCommandCfg codebox: resampling_time_range (5.0,5.0)→(4.0,4.0)；object_pose_range x=(0.3,0.35)/y=(-0.1,0.1) 改为实测 x=(0.2,0.4)/y=(-0.2,0.2) 且补 yaw=(-3.14,3.14)；正文 (5,5) 平衡值改为 YAM 实测 (4,4)
- 正则 reward 表: 删除不存在的 joint_acc_l2；joint_pos_limits 权重 0.1→-10（差两数量级）；新增真实在用的 joint_vel_hinge(-0.01→-1.0 课程渐增)；action_rate_l2 权重定为 YAM 实测 -0.01
- sigma 标定示例: 把虚的 σreach≈0.15/σbring≈0.2 补上 YAM 源码实测真值 reaching_std=0.2/bringing_std=0.3，并说明四步法给量级、对不上源码不是错
- 四阶段节运行验证: 纠正'固定基座永不摔倒/只有成功+超时终止'——补 YAM 实测有 ee_ground_collision 终止(非法接触力>10N)+time_out
- MJCF joint equality 项: 明确 equality 只降 nu(8→7) 不降 nq；补机构细节 YAM 实为 crank gripper(DM4310 曲柄转线性)非直驱平行指
**教学缺口补充 (4)**
- 补'别手算去问框架'的可复制命令序列: 法一编译 MJCF 打印 nq/nu/njnt/neq(预期 nq8 nu7)、法二跑 1 迭代读 Actor MLP in_features(预期29)——把最易漂移的维度数从'背书里的数'变成'自查习惯'
- 补一次真实排错命令序列(sec:rlmc-manip-diag)'reach 高 bring 卡零怎么自己查': 4 步 grep reward 分项→random play 看夹爪/接触→手动塞方块测物理可行→load_env_cfg 查 ee_to_cube 是否进 actor，并讲'从外到内、先排物理再排策略'的排查逻辑(每步给跑什么/看什么/判什么)
- 新增 insight'生产配置常是粗门控+精定位两项叠加': 点破 YAM 实际是 staged(大 std 软门控)+bring_object_reward(std≈0.05 精定位)两个独立 term，讲清'为什么拆两项而非调一个 std'(单核两难)，区分'读懂源码 reward 组合'与'看懂一个函数'
- smoke train note 补'首跑盯三行'可操作清单(steps/s 升/value loss 有限不爆/reward 非 NaN)，作为操作任务最快跑通自检
**优化补充 (3)**
- smoke train 真实吞吐基准 note: RTX4090 64env 实测 1800→4300 steps/s(首迭代含编译预热), 并解释与'4096env <5min 解 cube'(~1e5 steps/s 量级)不矛盾、吞吐随 num-envs 成比例——破'64env 太慢'误解
- YAM 真实接触 DR note: 指尖摩擦按 slide/spin/roll 三向独立随机(startup 模式+abs+log_uniform, 如 spin(1e-4,2e-2)), 讲清为何分三向(spin/roll 失真致真机转脱手)、为何用 startup 非 interval——比'手动设范围'具体得多
- 新增 insight'惩罚项课程式渐增(joint_vel_hinge -0.01→-1.0)': 讲'早期正则太重策略不敢动'→开训放松收敛收紧的实用调参直觉, 给'卡死不动先砍正则到 1/10 试'的落地动作

## prac_humanoid_wbc
**硬不符修正 (2)**
- Quick Start mjlab codebox (line~63-78): 原 `uv run train Mjlab-Tracking-Flat-Unitree-G1 --registry-name your-org/motions/wave-hand` 照抄裸跑实测报 ValueError(占位 registry 不存在)——改为两条路径:路径A(WandB)注释化并标注会 ValueError、路径B 用实测可越过 registry 进仿真的本地文件法 `--env.commands.motion.motion-file /path/to/your_motion.npz` 作默认冒烟命令。
- Quick Start UniLab codebox (line~87-96): 原 `HF_ENDPOINT=hf-mirror` 提示只挂在 `demo` 行,但实测 `train g1_motion_tracking` 首跑也从 HF 拉 dance1_subject2_part.npz(无镜像 RuntimeError)——把 `HF_ENDPOINT=https://hf-mirror.com` 前置到 train 行(并保留在 demo 行),加注释说明 train 首跑也需该前提。
**教学缺口补充 (3)**
- Quick Start 后新增 note(命令依赖与离线替代表):把三条命令逐一标注'外部依赖/不满足时的报错长什么样/最小离线替代'(mjlab=WandB或本地NPZ二选一→ValueError;UniLab=HF拉NPZ→RuntimeError→HF_ENDPOINT;Isaac=无外部依赖可裸跑),教读者照抄前先看清依赖。
- 同 note 内补'没有 WandB org 时怎么从零拿到一个能跑的 motion'的从零搭建路径:用框架自带/本地 assets/motions/g1/*.npz、NPZ 键格式与 csv_to_npz 类 retarget 管线指向 \cref{ch:rlmc-motimit},并给出'先验证接线(不碰WandB/网络)再换真动作'的两步法。
- 诊断层次 codebox Layer 1:把纯文字'纯行走是否稳定'锚定到 gpufree 实测健康基准——给出可复制命令 `WANDB_MODE=disabled uv run train Mjlab-Velocity-Flat-Unitree-G1 --env.scene.num-envs 64 --agent.max-iterations 2` 与预期输出(64env ~2.5k steps/s、value loss 量级~1、reward 上升、无 NaN),教读者判读健康曲线长什么样。
**优化补充 (3)**
- 版本信息表 Isaac Lab 行:补 2.3.2 实测另注册的更全 loco-manip 任务谱(Isaac-Locomanipulation-G1-Abs-Mimic-v0、Isaac-G1-SteeringWheel-Locomanipulation、Isaac-PickPlace-FixedBaseUpperBodyIK-G1-Abs-v0/-InspireFTP-Abs-v0)。
- 版本信息表 mjlab 全身跟踪行:补 list-envs 实测的 -No-State-Estimation 变体(去状态估计冒烟对照,与 sim-to-sim 鲁棒性相关)。
- UniLab G1MotionTracking reward 标量字典 codebox:把原 `...` 占位补成实测真值——action_rate_l2:-0.1、joint_limit:-10.0 两项正则,并补 std_* 容差字典(std_body_pos 0.3 等)作为 ExBody2 逐项 σ 表的现成落地实例。

## prac_legged_manip
**硬不符修正 (8)**
- Quick Start 代码框(原 line94/95): 任务名 go2_arm → go2_arm_manip_loco(注册名 Go2ArmManipLoco); go2_arm 实跑报 'No owner config'。同时补上实测真值反馈行(reward/object_distance=1.88、critic in_features=79、~575 steps/s)。
- UniLab obs/非对称AC 整段(原 line705-741): 删除虚构的'critic 多拼 object_lin_vel/friction/payload/contact_force_gripper 特权维(obs64/critic78)'; 改正为实测真值——actor 与 critic 共用同一 79 维布局, 唯一差别 actor add_noise=True / critic add_noise=False; _compute_obs 改写为共享 _compute_raw_obs(add_noise) 返回两份同维数组; critic 维度 78→79。
- UniLab obs 内容(原 line712 注释 grip/obj_quat): 删除夹爪/物体真值维; 换成实测单步79维真实布局 ee_local_pos(3)+ee_goal_cart(3)+ee_error(3)+last_actions(18) + 底盘/腿/臂本体 + commands。
- UniLab 臂控机制(原 line99/411 称臂走'ctrl=act*action_scale+default_angles'): 改正——腿确为关节位置残差, 但臂是 Diff-IK 残差 arm_dof_pos+act*arm_action_scale+ik.gain*dq_ik(默认 arm_action_scale=0.0 即纯IK); 章原文未提 IK-in-the-loop, 已在 actuator 段、Quick Start note、obs 段三处补正。
- UniLab reward 整段(原 line842-879): 删除虚构 reward 键 locomanip_approach/grasp_success/stop_during_grasp 及虚构 YAML; 换成实测真值——loco 全套(tracking_lin_vel/tracking_ang_vel/orientation/action_rate...)加三个 loco-manip 项 object_distance/object_distance_l2/arm_collision; 全程无夹爪/抓取。派发机制(∑scale*fn 跳过scale==0、*ctrl_dt、每4步写info['log'])报告确认属实故保留。
- owner YAML 路径(原 line866/892 conf/ppo/task/go2_arm/mujoco.yaml): 改为 go2_arm_manip_loco/mujoco.yaml; 并加注'覆盖不存在的键(如 grasp_success)会报 key not found'。
- 源码路径与平台(原 line35 envs/locomotion/go2_arm): 补全为 src/unilab/envs/locomotion/go2_arm/manip_loco.py; 平台从'ARX'纠正为 Go2+Airbot 臂(章首注释块 + 版本表 line35)。
- 章首注释块(原 line13-14): 任务名 Go2ArmManipLoco → go2_arm_manip_loco, 并补真实特征(Go2+Airbot、无夹爪、Diff-IK 残差、球面采样 reaching、actor/critic 同79维仅noise非对称)。
**教学缺口补充 (4)**
- 新增 insight[为什么带臂任务给策略'EE误差+IK残差'而非直接出6臂关节](ins:rlmc-lm-ikresidual): 把 UniLab 真实的 IK-in-the-loop 讲成三重耦合(逆运动学耦合)的活教材——讲清为什么(降维+base系相对坐标契约)、怎么调(arm_action_scale=0.0纯IK先跑通→>0加学习残差; ik.gain/dq_clip/orientation_mode 各管什么)。补报告点名的'框架里IK怎么和RL动作叠加'这一最能教会工程的缺口。
- Quick Start UniLab 段: 把原'绕一大圈解释为何不用zero agent却没给能成功的命令'改成'一定能跑的一行'(go2_arm_manip_loco max_iterations=1)+实跑应看到的真实 reward 行(object_distance≈1.88)+critic in_features=79, 让读者第一步就有正反馈。
- 新增小节[从零搭课程: UniLab go2_arm_manip_loco 的真实开关](sec:rlmc-lm-curr-unilab): 把四阶段课程从'伪代码+不存在的任务名 LocoManip-Go2Arx-Phase2'落成可复制的真实 Hydra 覆盖——env.arm_stage.freeze_arm_joints(=Phase0冻臂) / env.goal_ee.disable_ee_goal_trajectory / domain_rand.*(=Phase3加DR); 给 Phase0/Phase2/Phase3 三条可跑命令。并诚实指出'Phase1冻底盘'本任务无现成开关、需自己读源码加对称开关(一次好的工程练习)。
- 新增 UniLab 官方模型自检 codebox(复合模型10分钟验证清单后): 把抽象 checklist 落成 docs/manip_loco.md 的真实一行命令——mujoco.MjModel.from_xml_path(scene_flat.xml) 打印 nq/nv/nu/nsensor + pytest tests/envs/locomotion/go2_arm + test_mujoco_site_jacobian(验证 Diff-IK 依赖的 site 雅可比)。
**优化补充 (4)**
- 补真实基准吞吐: RTX 4090 上 go2_arm_manip_loco CPU-sim ~575 steps/s(写进 Quick Start 实跑反馈), 让读者校准自己环境。
- 补 arm_action_scale 调参旋钮(ins:rlmc-lm-ikresidual): 0.0=纯IK(最稳/先跑通) vs >0=IK打底+学习残差(修正IK动态偏差), docs/manip_loco.md 列为首要调参点; 配 ik.gain/dq_clip/orientation_mode(target vs zero_error) 三个子旋钮含义。
- 补 Isaac 端更贴RL的 loco-manip 现成任务对照: 在'四足+臂须自定义'pitfall 末尾加一句 Isaac-Tracking-LocoManip-Digit-v0(manager_based/locomanipulation/tracking) 与 Isaac-G1-SteeringWheel-Locomanipulation, 比纯 teleop 的 PickPlace-G1 更适合做 RL reward 教学对照。
- 把 go2_arm_manip_loco 明确点为本章主线 VBC'分层低层用IK跟踪EE目标'的现成可跑实例: EE 目标在 goal_ee 球面 sphere_l/phi/theta 采样, 即 VBC'height-invariant sphere'的现成实现(obs 段已连到 sec:rlmc-lm-vbc-stage1)。

## prac_large_scale.tex（大规模训练：性能优化与 NaN 排查）
**硬不符修正 (1)**
- 无硬不符可改：报告 prac_large_scale 小节『工程核对』8 条全部标『是这样/属实』，三框架训练命令/NaN Guard 链路/CUDA Graph/UniLab 异构多 GPU/EmpiricalNormalization 实跑逐一坐实，未发现任何 API 不存在/数值过时/讲错/报错。报告唯一的『澄清(非错)』(UniLab NaN 守卫『默认开』是 conf 覆盖而非 dataclass 裸默认) 明示『工程语义正确, 无需改』——故按纪律未改其结论，仅顺手把该处措辞精确化为『训练时默认就开/训练 conf 默认开』(正文 line665 + 版本表 line1521)，不算改 bug。
**教学缺口补充 (3)**
- NaN 节(mjlab analyze_nan_dump 后, line~626)新增『NaN 防线分层』教学块+insight(ins:rlmc-ls-nan-layered)：补报告点名遗漏的 mjlab obs 组级 --env.observations.{actor,critic}.nan-policy{disabled,warn,sanitize,error}+--nan-check-per-term(实跑 train --help 核实存在)这道轻量闸，教读者『obs 入口 error → 整训 nan-guard dump』由轻到重的分层设防梯度(可复制命令+档位含义)。
- 多 GPU 节运行验证小节(line~371)新增『只有一块卡时怎么烟雾测试多卡配置』段+note：补报告点名缺失的单卡降级路径——给 Isaac torchrun --nproc_per_node=1 --distributed 与 mjlab --gpu-ids "[0]" 两条 world_size=1 烟雾测试命令，并讲清『能验证 90% 起不来错误(旗标/进程组/入口)、不能验证 all-reduce 数值/NCCL/单写者』的边界，教『单卡跑通链路→多卡验同步→再扩规模』流程。
- UniLab NaN 段(line~666)新增 pitfall『查某开关真实默认开没开要看 conf 覆盖不是看 dataclass 裸默认』：以 UniLab nan_guard(裸默认 False 被 conf 覆盖成 true)为活例，教 Hydra/OmegaConf『两层默认』分辨法与『打印合成后 config 落盘那份才作数』的可迁移排错方法论。
**优化补充 (3)**
- 性能节快速 benchmark codebox 后(line~764)新增 note『小 env 数 steps/s 几乎无参考价值』：补实测真值锚点(gpufree RTX4090 mjlab Go1 64envs 仅~1300 steps/s vs 4096 envs 才 10^5 量级；Isaac Cartpole 头几迭代 100k→稳态 300k)，解释小 env 数 kernel 未饱和+CUDA Graph/JIT 首迭代预热两因，并连到十字段协议『先 warmup 再测+用目标 env 数』纪律——治『首跑被低数字吓到』。
- UniLab local-SGD note(line~340)末强化 multi_gpu_sync_interval 默认=1 的『默认值陷阱』：点明 interval=1 时 local_sgd 名不副实(通信量同 sync_sgd)，要『少同步换吞吐』必须显式调大(8/16)，是吞吐 vs 收敛的权衡。
- NaN 节黄金法则 insight 后(line~714)新增『PyTorch 侧细粒度定位』段：补 CUDA_LAUNCH_BLOCKING=1(强制同步让报错栈指向真正出错 kernel)+torch.autograd.set_detect_anomaly(True)(抓首个产 NaN 算子)两手，与正文已有『禁用 graph 二分』互补(前者定位是哪个算子、后者定位是不是 graph 问题)，并注明 detect_anomaly 显著拖慢须诊断完关掉。

## prac_tennis_env.tex（网球场环境构建与球物理）
**硬不符修正 (12)**
- mjlab 数据访问短名 root_lin_vel_w/root_ang_vel_w（BallAerodynamics, 原 line 699-700）→ root_link_lin_vel_w/root_link_ang_vel_w（实测 mjlab1.4.0 无短名,照抄 AttributeError）
- calibrate_cor 里 root_pos_w/root_lin_vel_w（原 822/824）→ root_link_pos_w/root_link_lin_vel_w
- BallNetPlaneContact 语义撞网 root_pos_w（原850）→ root_link_pos_w
- Observation terms 三处 root_pos_w/root_lin_vel_b/root_lin_vel_w（原917/919/921）→ root_link_ 长名
- ball_landed_in_service_box root_pos_w（原967）→ root_link_pos_w
- obs frame pitfall 正文 root_ang_vel_w（原925）→ root_link_ang_vel_w
- COR 数值硬错（原435/811）：章节称 solref=(-3500,-2.0)+solimp 实测使硬地 COR0.73-0.76,实跑真值 COR≈0.95(过弹);改写为『-2实测0.95,要命中0.73-0.76须把阻尼项调到≈-12』并贴单调标定表(-2/0.95,-12/0.74,-30/0.50,-60/0.29)
- 教学方向反（原435 solref_hard『快弹高恢复』+原500练习扫正格式期待单调）：实测正格式 timeconst/dampratio 在常规 solimp 下 COR 仅0.06-0.14且 dampratio 几乎不动;改述为『COR 主旋钮是直接负格式第二项,正格式调不出网球弹性』,练习改扫负格式
- COR 通过标准（原811）『弹太高增大 dampratio/弹太低减小 timeconst』方向错 → 改为按直接负格式阻尼项绝对值调(-2→-12)并显式警告别用正格式 dampratio
- UniLab scene MJCF（原467）court_surface solref="-3500 -2.0" 注释『COR 标定』过弹 → 改 -12 并注明实测 COR≈0.74/须自标
- 故障排查表（原1347）『COR>0.85 增大 dampratio 至1.2-1.5』格式不匹配 → 改『增大直接负格式 solref 阻尼项绝对值(-2→-12)』
- 调参决策树（原1368）『弹太高→增大 dampratio』→ 改『增大直接负格式 solref 阻尼项绝对值』
**教学缺口补充 (4)**
- 补『怎么自己发现数据字段真名』工程动作：新增可跑 codebox 教 dir(env.scene[...].data) 过滤 pos_w/vel + 靠 AttributeError 反查真名两条排错路径(用 stock 任务 Mjlab-Velocity-Flat-Unitree-Go1 等价验证),把『短名照抄必崩』转成可迁移的 API 自查肌肉记忆
- 补 COR 标定的『先测再信参数』纯 MuJoCo standalone 落体脚本(约20行,不依赖 mjlab,照敲即跑,含 freejoint qpos 取 z + 实跑预期输出 -2→COR0.95/-12→COR0.74),替代原 calibrate_cor 走 env.step 的高耦合路径
- 补『Mjlab-Tennis-Launcher 今天敲不出来』pitfall：list-envs 实测12任务无 tennis,给从零搭起读者两条今天能跑的替代验证路径(纯 MuJoCo 脚本跑 COR/condim/freejoint + stock Go1 任务跑通 list-envs/play/viewer/num-envs)
- 补 mjlab link帧 vs com帧 区分的教学点(单体球 link≈com,下游人形/球拍喂 predictor 是 link 还是 com 位姿结果不同)——把 API 坑升华成『为什么 mjlab 不给短名别名』的设计意图
**优化补充 (3)**
- 补 RTX4090 单卡一手吞吐锚点(Mjlab Go1 64envs≈2473 steps/s)+ Isaac Sim 数分钟启动 vs mjlab 秒级 vs UniLab CPU 物理 的启动/吞吐预期,让读者校准自己环境是否正常(原章通篇无任何 steps/s 参照)
- 补 COR↔solref 单调实测标定表(阻尼项 -2/-12/-30/-60 → COR 0.95/0.74/0.50/0.29)+『先跑落体反推阻尼项加减、迭代两三次定死』工作流,比原『solref 与 COR 非线性、需实验标定』只给方向不给数更可操作
- 补 write_external_wrench_to_sim 之后须 write_data_to_sim() 落盘的说明(Entity 确有该方法;mjlab 侧 buffer→sim 落盘时机原只在 Isaac 侧讲,易写了外力不生效),并加『mjlab 显式区分 link/com 帧、无 Isaac root_pos_w 短名别名』工程提示做成 pitfall

## prac_tennis_perception
**硬不符修正 (6)**
- 球状态 API 名(报告头号实跑 bug): 章节对 mjlab 用 Isaac-only 的 root_pos_w/root_lin_vel_w/root_ang_vel_w 并标'逐字相同', 实跑 mjlab EntityData 无此属性、照写会 AttributeError——已逐处改为 mjlab root_link_pos_w/root_link_lin_vel_w/root_link_ang_vel_w 并注明 Isaac 改名: 五层表(L202)、frame 动机段(L257)、label 规格表三行(L268-270)、collect_ground_truth_label 代码框(L290-292)、Hamilton 段(L303)、frame 对比表(L338-339)、ball_state_privileged 代码框(L383-385)、intro(L32)、Quick Start mjlab bash 框(L108)、方案A(L407)、含drag三框架段(L684 删除假的'三框架同名')、distill teacher-label 注释(L1171)、obs 速度 pitfall(L241)、版本信息表(L1289-1291)。
- frame 段'逐字相同'的错误归因(L282): 原称 root_pos_w/root_lin_vel_w 在 manager-based 框架统一命名——改为'逻辑逐行相同但球状态属性名有一处必须按框架改', 点明 mjlab 拆 _link_/_com_ 两套、无裸 root_pos_w。
- mjlab 相机能力被低估(报告 MISLEADING, L435/443-448): 章节称只有 Isaac 有相机配置类、mjlab 仅'Warp 渲染管线/depth buffer'——实为 mjlab 1.4.0 有一等 CameraSensorCfg(from mjlab.sensor import, data_types Literal['rgb','depth','segmentation'], 输出 rgb[N,H,W,3]uint8/depth[N,H,W,1]f32/seg[N,H,W,2]int32)且带可跑任务 Mjlab-Lift-Cube-Yam-Rgb/-Depth/Multi-Cube-Seg-Yam; 已重写段落与对比表(补'现成可跑视觉任务'行), 明确'缺的不是 mjlab 相机能力而是缺挂相机的网球任务'。
- 接触事件源不全(报告 INCOMPLETE, L760/768): 章节 mjlab 列只给 MjData.contact——实测 mjlab 亦有一等 ContactSensorCfg(force/netforce/maxforce); 已在弹跳三框架段与对比表把 mjlab 接触源改为'ContactSensorCfg(推荐)/MjData.contact', 并指出两框架在接触事件源上其实对称。
- Quick Start EKF 降幅夸大(报告 OVERSTATED, L102-103 注释'应明显更小'+L593'低50%+'): 实跑该设定位置 RMSE 仅 raw~3.1cm→EKF~2.6cm(~18%)非50%——已改 codebox 注释与'运行验证'段为实测~18%、说明'不是 bug'(位置直接被观测、噪声小则平滑空间有限)、指出 KF 真正大赢在速度(sigma_v~2.8m/s)、给出要看50%+需调噪声5cm+dt=1/60s 的条件。
- Mjlab-Tennis-Launcher bash 框读作可跑命令(报告 L109): 已在框内注明该任务当前未注册(list-envs 12 任务无它、属本章预规划)、现在跑会 task-not-found、先用纯 PyTorch EKF 验证。
**教学缺口补充 (3)**
- 从零搭建/排错的可迁移工程基本功——'别记 API 名, 去机器上问它': frame 节新增 3 行 dir(ball.data) 自查代码框(catch root_link_ vs root_), 演示 mjlab 打印会看到 root_link_pos_w/root_com_pos_w 而无裸 root_pos_w(写它即 AttributeError)、Isaac 打印 root_pos_w、UniLab 列 get_body* 后端 getter; 把'这名字对不对'从赌书变成跑一行确认, 正是能自查出本章那个命名 bug 的可迁移技能。
- EKF 调参从'公式'到'跑一遍看见 NIS≈3': Q/R 调参节新增可复制 NIS 打印框(在 Quick Start 玩具轨迹这唯一有真值处累计 NIS=y^T S^-1 y, 打印均值, 健康区间[2,4]), 并讲清'为什么 NIS 比 RMSE 会调参'(NIS 衡量滤波器对自估不确定性诚不诚实=Q/R 该往哪调的方向盘; 新息系统性偏移则是漏 drag/Magnus 调 Q 治不了)。
- 把'预规划'相机变成 5 分钟实跑 hands-on: obs 节练习新增[实跑]项——mjlab list-envs 确认 Mjlab-Lift-Cube-Yam-Rgb 在列, 给可复制命令 WANDB_MODE=disabled MUJOCO_GL=egl mjlab train ...(讲清两个关键环境变量为何需要), 让读者亲眼看 camera_*_rgb (3,32,32) 张量形状, 教如何从真 env 自查 sensor 输出 dtype/shape。
**优化补充 (3)**
- mjlab _link_ vs _com_ 的工程区分(高价值跨框架 gotcha): frame 对比表后新增 finenote——网球单刚体 link≈com 数值无差, 但下游人形击球时喂 predictor/算击球点用某 body 时 _link_(运动学落点)与 _com_(动力学积分点)差几厘米, 球状态一律用 _link_ 不要顺手抄成 _com_; Isaac 只有单一 root_pos_w 无此二选一。
- 诚实的 mjlab-vs-Isaac frame 命名对照(报告首条优化): 把'逐字相同'换成显式命名差异行, 并在多处把抽象的'统一命名'落成 mjlab root_link_*/Isaac root_* 的并排标注——正是本章存在意义(教跨框架接线)该传递的真实差异。
- mjlab ContactSensorCfg 优先于裸 MjData.contact 做弹跳检测(报告优化): 在弹跳三框架段点明它是批量、GPU 友好、过滤球-地接触对的路径, 比每帧扫 contact 数组更省更稳, 与 Isaac ContactSensorCfg 对称。

## prac_tennis_control
**硬不符修正 (3)**
- 噪声节 三框架对比段(原L962/990): 原称 mjlab 观测延迟须自写 buffer(过时)→改正为「mjlab 1.4.0 原生支持 per-term 观测延迟 delay_min_lag/delay_max_lag/delay_per_env/delay_hold_prob，无需手写 buffer；仅 Isaac/UniLab 须自写」，并在延迟小节加 note(label note:rlmc-strike-mjlab-delay)给出内置字段配置示意。
- UniLab 对照段(原L992)与 _compute_obs codebox 末注释(原L979): 原把 mjlab 与 Isaac/UniLab 一并归为「延迟须自定义 buffer」→改为「Isaac/UniLab 须自写；mjlab 有内置字段」。
- IK action codebox(原L318-339): 原显式给 delta_pos_scale=0.05/delta_ori_scale=0.1 却未点破 mjlab 1.4.0 默认是 1.0(不缩放)这一 footgun→codebox 顶部加实测默认值告警(damping=0.05/max_dq=0.5/use_relative_mode=True 默认，delta_*_scale 默认 1.0 必须显式改小)，并新增一条 pitfall『沿用 delta_*_scale 默认值 1.0』(50Hz 下首帧即关节跳变/NaN)。
**教学缺口补充 (4)**
- 环境配置节(sec:rlmc-strike-env)新增『动手第一步：先用 stock task 确认环境能起』段+bash codebox: 给出 4 条 gpufree 实测可跑命令(mjlab list-envs / WANDB_MODE=disabled train Mjlab-Velocity-Flat-Unitree-G1 2 迭代 / train Mjlab-Lift-Cube-Yam / python -c import DifferentialIKActionCfg 列字段)各带预期输出，并讲清『为什么先跑 stock task』(版本/GPU链路/API名三自检)——补全报告所述『全章无一条可照敲命令』缺口。
- 新增 insight(ins:rlmc-strike-retrofit)『从 stock task 改造而非凭空写 strike task』: 指明 Mjlab-Lift-Cube-Yam 是最近的 manip+IK 起点，给出复制 EnvCfg→换球→换 staged reward→换 IK action→升频率的四步改造路线，每步 train --max-iterations 2 验证——补『如何从 stock task 改造』缺口。
- 奖励节诊断流程图后新增『一次真实调参 walkthrough：把 σ 从 0.5 调到 0.3』: 四步可操作闭环(确认刷shaping→量饱和半径(σ=0.5 时 d=0.25m reward≈0.78)→改一个数短训只盯 contact_rate→设刹车 σ∈[0.2,0.4]防过冲)，把图里定性的『减小σ』变成『曲线→参数→曲线』可归因流程——补『从 contact rate 反推调参』缺口。
- 章首新增 note(note:rlmc-strike-pseudocode)『代码可跑度约定』: 明确区分『框架配置类(照真实API、可挂框架)』与『算法/管理器骨架类(StrikeEventBuffer/PerceptionNoiseManager/LagrangianPPOForTennis 等引用 env.perception/event_buffer 等虚构属性、直接实例化会 AttributeError 的伪代码骨架)』，并指向 stock-task 改造路径——补『核心类多为不可实例化伪代码却未标注』缺口。
**优化补充 (3)**
- MDP节新增 subsection(sec:rlmc-strike-mjlab-mainline)『为什么本章用 mjlab 做主线』: 表格化三框架一手体感(冷启动: mjlab 秒级 / Isaac Sim 数分钟 / UniLab 秒级CPU; 实测吞吐 G1~2500、Lift-Cube~700、go2~590 steps/s; task-space IK action 与观测延迟 谁内置)，给出按『你最频繁的操作是什么』的选型判据——补报告所述缺失的选型工程权衡。
- 观测配置 代码走读 处补『两个被忽略的内置字段』: mjlab ObservationGroupCfg 原生 history_length/flatten_history_dim(可替手搭 last_action 做 N 步时序堆叠，对 time_to_impact 尤有用) 与 nan_policy/nan_check_per_term(内置 NaN 守卫，IK 奇异处发散不至于整批 rollout 作废，胜过只调大 lambda)。
- IK action codebox 末补 mjlab 1.4.0 DifferentialIKActionCfg 进阶字段 position_weight/orientation_weight/joint_limit_weight/posture_weight/posture_target(加权+零空间姿态正则 IK): 冗余臂可把『举拍 ready pose』编码进 posture_target，比单加 give-up reward 更直接——报告点名的被忽略高级用法。

## prac_diagnostics.tex（训练诊断与调参地图）
**硬不符修正 (2)**
- L991 §EmpiricalNormalization 段首：原称 mjlab(5.2.0) 的 obs 归一化开关是 actor_obs_normalization/critic_obs_normalization 且『源码核对、两版同名』——报告实测该名仅 Isaac 3.1.2(ActorCritic) 成立，mjlab 5.2.0 经 RslRlModelCfg 的 actor=/critic= 块各暴露一个 obs_normalization（rl_cfg.py:16/26），rsl_rl 5.2.0 包内 grep actor_obs_normalization 零命中；已改为按版本分述两版不同名，并配一个自查字段名的 pitfall+codebox。
- L1018（同节后半）：原句『两版同名…开关都是 actor_obs_normalization/critic_obs_normalization』把归一化『类名一致』错误外推成『开关名一致』；已改为只声明 EmpiricalNormalization 类名两版一致，并显式补『但配置开关名两版不同』（Isaac=actor/critic_obs_normalization；mjlab=RslRlModelCfg.obs_normalization），提醒勿把类名一致误当开关名一致。
**教学缺口补充 (4)**
- 贯穿案例 §walkthrough（Phase 0-2 前）：补『可执行替身』——用 mjlab 自带真实任务 Mjlab-Velocity-Rough-Unitree-Go1（gpufree 实跑可通）把 Phase 0-3 空跑一遍的可复制命令（list-envs grep / play --agent zero,random / WANDB_MODE=disabled 64env 2迭代），解决报告所指『示意任务名 My-...-Go2 让初学者卡在第一步』。
- 同处补『从零搭一个自建 rough 任务的最小步骤』四步（定位现成 cfg→复制改三处→register_mjlab_task 注册→跑通即返回 Phase0），强调工程真实路径是『复制现成改三处』而非从空 cfg 手写，覆盖报告所指『从零搭建路径缺失』。
- §index 索引表入口：补『30 秒确认症状的三条最小判据』——读 Loss/learning_rate 锯齿代替找不存在的 KL 曲线、叠加 Episode_Reward/* 看是谁在动、打印 obs 逐维 std 判信息是否进来，把索引表从『查得到』升级为『判得准』，覆盖报告所指『症状表只读不练、缺一句话判据』。
- §EmpiricalNormalization：新增 pitfall『把 Isaac 的 actor_obs_normalization 当成 mjlab 同名开关』并给出自查命令（python -c 列 RslRlModelCfg 字段 / grep run 目录 config.yaml），身教报告强调的『以本机实际加载值为基线』，把版本漂移变成可操作技能。
**优化补充 (2)**
- Quick Start §quickstart mjlab 读盘 codebox：补 Step 0 最稳起手式 WANDB_MODE=disabled uv run train ...（gpufree 实跑必通），并加注『缺 key 时 --agent.logger wandb 会直接报错退出』，避免读者第一条命令就崩。
- §logging KL note 后：补『动手：给 rsl-rl 后端自加一条 KL 数值曲线（不改库源码）』——给出高斯解析 KL 函数 + 往 extras['log']['Loss/kl'] 塞标量（利用 logger 含/键原样写出，5.2.0 logger.py:174 实测），并说明这比改 runner 更轻（不被 pip 覆盖、随项目走），兑现章节多处『要 KL 曲线须自行加 add_scalar』却从未给法的承诺。

## prac_setup — 三框架环境搭建与第一次运行
**硬不符修正 (7)**
- 读日志表(原L719-736): 字段名与真实 mjlab 终端对不齐——把单列「字段」改为「通用语义 + mjlab/RSL-RL 实际字面」双列, mean_reward/policy_loss/value_loss→Mean reward/Mean surrogate loss/Mean value loss/Mean entropy loss, fps→Steps per second, 并补 Collection time/Learning time 行(实跑真值)
- 读日志表: 删掉 mjlab 终端根本不打印的 kl/entropy 两行, 新增 pitfall 讲清『mjlab 摘要不打印 kl, KL 藏在 adaptive LR 调度(用 desired_kl 自适应), 要看须开 --agent.logger wandb/tensorboard』——堵住读者照表找 kl 扑空
- 本章目标(L219): 把『读懂 reward/ep_len/kl/fps』改为按语义表述并注明各框架字面不同、KL 不在 mjlab 终端而在日志后端
- ins:rlmc-read-log: 把正文里的 fps 改为真实字面 Steps per second, 并用 Collection time/Learning time 替代『采集/学习时间』口径
- Quick Start 坑note(L189-191)与措辞: 『mjlab 默认无头』只对 train 准确——改为分命令(train 默认无头/play 默认即起 Viser listening 8080), 对齐实跑
- 吞吐基线(L617): RTX4090 32env~4600/64env~8900 标注为『热稳态大并行』, 补 RTX4080S 16env 冷启 mujoco~68/motrix~425 实测对照, 并声明冒烟阶段吞吐不作通过判据(纠『读者跑小冒烟见几十sps误判异常』)
- 运行验证表吞吐行(L611): 异常原因补『先排除冷启/小并行(良性)再查 CPU 核不足/shm 抖动』, 补全报告指出缺失的最常见良性原因
**教学缺口补充 (4)**
- 新增 ins:rlmc-throughput-sanity『吞吐正常性自己估』: 给估算心法 env-steps/s≈N_env×f_ctrl×η_parallel + 三条推论(应随 num_envs 近线性/冷启小并行良性低/排除后才疑故障)——教读者换任何机器自判, 不背绝对 sps 表
- 读日志 pitfall 落到可复制命令: 看 KL 给出 --agent.logger wandb|tensorboard 与 tensorboard --logdir logs/ 的实操路径, 把『去哪看 KL』从抽象讲到敲哪条命令
- mjlab play vs train CLI option 不通用: 新增 pitfall(play 不认 --env.scene.num-envs 实测报 Unrecognized), 给通则『换子命令就 --help 一次』, 并同步修正 mjlab 动手任务3 标注该 flag 只属 train
- Isaac Lab 节首新增可见『实跑覆盖声明』note: 明示本节 Isaac 命令未在撰写配置实跑、只有非 Isaac 环境者可跳过用 mjlab/UniLab 等价练习(方法论通用), 避免读者怀疑自己装错——把原本仅在末尾 finenote 的提示前置可见
**优化补充 (3)**
- mjlab 四维表(L347-358)补 --agent.algorithm.learning-rate(默认0.001)/--agent.algorithm.schedule(adaptive) 两行 + 新增 note『超参也走 --agent.algorithm.*(tyro 暴露整棵配置树)』含可跑覆盖命令, 并与 UniLab Hydra 点路径 algo.learning_rate= 做两范式对照(--前缀+短横线 vs 无前缀+下划线+等号)
- 新增 pitfall『UniLab 注册名 CamelCase(Go2JoystickFlat) 但 --task 接 snake_case(go2_joystick_flat)』: 实跑确认 slug 被映射, 给『看 conf/ppo/task/ 目录或 --help』的自查法, 并对比 mjlab 任务名大小写敏感原样传
- 吞吐双基线落地为实测锚点(RTX4080S 16env 冷启 mujoco~68/motrix~425 sps), 让读者小冒烟见低 sps 有据可依

## prac_physics.tex（物理引擎：MuJoCo 软接触模型与 PhysX 对比）
**硬不符修正 (2)**
- sim2sim bash codebox（原 line 814）: mjlab `play --export-onnx` flag 不存在→删除该命令, 改为注明 ONNX 由训练结束时 runner.export_policy_to_onnx() 自动产出(落 logs/<exp>/<run>/exported/policy.onnx), play 改用真实加载源 flag --agent trained --checkpoint-file/--registry-name/--wandb-run-path 做可视化复核(非导出)
- 稳定性/吞吐节 UniLab Profiling 项(原 line 942): 删去'无字面 --profile flag'的错误措辞(实测 cli.py:262 该 flag 存在)→改为'CLI 里 --profile 不是性能剖析, 它选任务 owner 变体(如 --profile hora), 性能剖析始终走 training.trace_enabled'
**教学缺口补充 (3)**
- 接触调参案例集开头(§rlmc-phys-tuning): 新增『症状到底敲哪条命令才看得见』段+bash codebox+insight ins:rlmc-phys-seeit——给 mjlab `play --agent zero --viewer viser`(实跑 Viser :8080)看穿透/微滑、contact buffer overflow=nconmax 太小的信号、UniLab `--render-mode interactive`+`unilab-viz-nan` 离线回放, 把案例反复说的『可视化确认』落到可复制命令与『先看→回方程→查参数』的排错链
- GPU 数据流节(§rlmc-phys-dataflow)『第一次 DR 变慢』note 后: 新增可自证锚点 Python codebox——锚点1 reset 后读 per-world geom_friction 跨 env std>1e-6 即 DR 生效(否则 DR 没挂进 events), 锚点2 用 torch.cuda.synchronize+perf_counter 测出第一次 DR 那步的 graph 重建一次性尖峰, 把『读源码听讲』升级为『读者能自证』
- sim2sim ONNX 衔接: 新增 note『ONNX 何时导出/落在哪/怎么确认』——讲清 mjlab 训练结束自动导出且 metadata 经 attach_metadata_to_onnx 随 ONNX 走(obs 契约不必另存 json)、UniLab 在 play/eval 阶段导出(no_play=true 不产)、并给 `find logs -name '*.onnx'`+onnxruntime 加载自检的确认手段, 兜住原命令照抄会卡第一步的缺口
**优化补充 (3)**
- 吞吐排查表后新增 insight ins:rlmc-phys-spsbaseline: 给出 steps/s≈num_envs×控制频率×并行效率 的估算心法+RTX4080S 实测锚点(mjlab Go1 64env≈1000 sps、UniLab go2 16env≈360 sps, 均冷启小 batch)+点明小冒烟低吞吐是良性(η<<1)非故障, 与练习里 RTX4090/4096env/~60k 的大并行稳态值对照, 教读者先判『我这台该多少 sps』再排查
- MujocoCfg 配置 codebox+note: 补 ls_iterations=50(线搜索, 接触不收敛时比加 iterations 更有效)与 jacobian=auto/dense/sparse 两个旋钮, 并把 jacobian 接到本章『MuJoCo Warp 偏好 dense Jacobian』能力边界——GPU 设 auto 落 dense, 盲迁 CPU(sparse)配置到 GPU 大 DOF 显存上涨, 落点呼应『别把 CPU 配置盲迁 GPU』
- UniLab sim2sim insight 后新增 note: 把抽象的『逐字段比 DENYLIST』落成实测字段清单(env.control_config.action_scale/algo.obs_groups/hidden_dims/empirical_normalization/sampling_mode + ENV_STRUCTURAL_DENYLIST), 让读者预判『训练后再改 action_scale/网络宽度/obs 分组→跨后端 eval 抛 CrossBackendIncompatibleError』, 比抽象描述更能举一反三

## prac_reward.tex（奖励、课程与终止设计）
**硬不符修正 (7)**
- §ablation 消融命令(原 line 1079-1082): mjlab 关脚滑用 `--env.rewards.feet-slip.weight 0.0` 跑不通 → 改为 dict 键单数 `--env.rewards.foot-slip.weight 0.0`(func 才叫 feet_slip), run-name 同步改 no_foot_slip, 注释讲清函数名 vs dict 键不一致。
- §full-table(line 322-323): mjlab 风格名列把 `feet_air_time`/`feet_slip` 写成函数名 → 改为实跑日志/dict 键 `air_time`/`foot_slip`(单数), Isaac 等价列保留。
- §full-table 表后散文(line 329): '脚滑在 mjlab 叫 feet_slip' 笼统 → 改为'函数 feet_slip 但 dict/日志键是单数 foot_slip(另 air_time/foot_clearance/foot_swing_height), 覆盖须用 dict 键否则 tyro 报 unrecognized'。
- §full-table 表后(line 329): 补 base_height 勘误——实跑 grep 确认 mjlab 无 base_height 项(Isaac 侧 base_height_l2 才有), mjlab Style 实际靠 upright+variable_posture, 标为'典型示例'并指向'从零定位 reward 键'。
- §term UniLab 补丁(line 851): 函数名 `_patch_terminal_next_observations`/`terminal_next_obs` 不存在 → 改为模块级真名 `patch_transition_next_obs`(base/final_observation.py)。
- §curr PenaltyCurriculum YAML(line 991-996): `min_scale:0.0`、`level_up_threshold:0.8`(当比例) 与实装 dataclass 默认不符 → 改为 `min_scale:0.5`、`level_up_threshold:750.0`(绝对 episode 长度计数非比例)、level_down 同改绝对计数, 并加'默认见 base/curriculum.py、照抄会得不同课程行为'告诫。
- §Quick Start UniLab(line 67): '~8900 env-steps/s @64' 未复现 → 删死数, 改为 RTX4080S 实测冷启~1500/热~3569 + 指向 sps 判据小节。
**教学缺口补充 (4)**
- 新增 §sec:rlmc-rew-locatekeys『从零定位一个任务的 reward 键(消融前必做)』: 三框架可复制命令(mjlab 跑 2-iter 冒烟 grep Episode_Reward/ 取真实 dict 键 + 实例化 cfg 打印 rewards.keys(); UniLab ensure_registries+list_registered_envs 与 print(_reward_fns.keys())), 每条给预期输出形态——教读者换任务自查而非照抄示例名(报告点名的最大教学缺口)。
- 新增 insight ins:rlmc-rew-keytypo『mjlab 拼错键即报错 vs UniLab 静默跳过』: 把'一个报错一个静默'当调试杠杆(mjlab 故意拼错看建议键反查; UniLab 消融前必 print 派发表键, 否则'扫半天曲线不动'是键名没匹配)。
- 新增 note『把作者已核对变成你自己的技能: grep/inspect 反查函数签名』: 三招(grep -rn 定位文件行、inspect.signature 取真实签名、inspect.getsource 看实现)——本章所有'已核对'结论的产出方式, 教'怎么核实而非背诵', 兼修复读者无法在 autodl 复现核对的缺口。
- §smoke 补『怎么判断我这台机的 steps/s 算不算正常』: 给四条判据(量级估算 sps≈num_envs×控制频率×并行效率、线性性、冷热、Collection/Learning time 结构比)替代死记绝对值, 兜住报告'小并行冷启低 sps 是良性'的缺口。
**优化补充 (4)**
- §trackimpl 补『跨框架 σ 自检脚本』(可跑 Python): 把 Isaac/mjlab 的 std 与 UniLab 的 tracking_sigma 统一折算成物理 σ 打印, 预期三行等效 σ≈0.5, 点破'照抄 std 数值→σ 变 0.707 核宽放大 1.4×'的翻车根源(报告称本章最有价值的反直觉点)。
- §contact 补 note『一个 reward 函数顺便产 metric——slip_velocity_mean 从哪来』: 实跑读 mjlab foot_slip 源码揭示 command_threshold 门控 + 把滑移均值写进 env.extras['log'], 解释本章多处当现成字段用的 Metrics/slip_velocity_mean 出处, 作'reward 兼做可观测性'范例。
- §termimpl 补 mjlab 实测真值: bad_orientation 的 shipped 默认 limit_angle=radians(70°)≈1.22 rad(章节示例 1.0), 且 out_of_terrain_bounds 标 time_out=True——由实装反向印证'离开生成地形=截断'决策表。
- §Quick Start 与 §smoke 补 RTX4080S 实测吞吐锚点(UniLab 64env mujoco 冷启~1500/热~3569、mjlab Go1 64env~1000)并说明与'大并行稳态'差一个量级是并行不足+JIT 未热所致, 防误判性能回归。

## prac_training_pipeline.tex（训练管线与超参调优）
**硬不符修正 (9)**
- L50 RSL-RL pitfall：把『本章实测 5.2.0』改为『版本约束 >=5.2.0,<6.0.0 起步，实测环境 5.4.0』并加一句『同 5.x 内 API 行为一致但源码文件归属会迁移』——版本号已漂(实装 5.4.0)。
- L266 GAE codebox 标题：『rsl_rl/storage/rollout_storage.py 同义简化』→『RSL-RL 同义简化；真实文件归属随版本，见下方导航』——5.4.0 里 compute_returns 已不在该文件。
- L293 走读句：删去把关键行错绑到『对应 rollout_storage.py 的 GAE 段』的措辞——避免读者照文件名扑空。
- GAE 走读后新增导航 note：明确 5.4.0 中 compute_returns 已移到 algorithms/ppo.py(签名 compute_returns(self,obs)) 与 runners/on_policy_runner.py、timeout 自举改为 rewards += gamma*values*time_outs(数学等价)——修正代码定位过时硬不符。
- L100 list-envs note：『列出 4 个 Unitree 速度任务』→『共 12 个任务(含 Cartpole/Lift-Cube/Tracking)，其中 4 个是速度任务』——修措辞偏窄(读者会误以为只有 4 个任务)。
- L613 mjlab API note：『实测 mjlab 1.4.0 + rsl-rl-lib 5.2.0』→『5.4.0』。
- L939 EmpiricalNormalization note：『rsl-rl-lib 5.2.0』→『5.4.0』。
- L1539 源码阅读路线 C：补『实测 5.4.0 里 compute_returns 与 timeout 自举在 ppo.py、不在 rollout_storage.py』并补 on_policy_runner.py 的 learn() 一环——让路线 C 指向真实文件。
- L1715 版本信息速查表 RSL-RL 行：『实测 5.2.0』→『实测 5.4.0』并补一句『compute_returns 等在 5.x 内随小版本在 rollout_storage.py 与 ppo.py/on_policy_runner.py 间迁移』。
**教学缺口补充 (4)**
- 新增『用 inspect/grep 在已装库里自己定位真实实现』可复制命令序列(grep time_outs/compute_returns + inspect.getsourcefile/getsource)——把『读代码』升级为『核代码』，正面教读者举一反三、不背死文件名(本次发现迁移就是靠它)。
- 环境配置指南新增『从零在无显示器服务器落地：四个首次必踩坑』pitfall——逐条现象→为什么→怎么自查：(1) headless 渲染 export MUJOCO_GL=egl；(2) uv run --no-sync 跳过联网同步；(3) WANDB 默认开需 export WANDB_MODE=disabled / --agent.logger tensorboard；(4) 先验 torch.cuda.is_available()。按『渲染→包→日志→算力』排序即最常见失败顺序。
- 诊断节新增『第一屏长什么样、哪些数算正常』note：给出 mjlab 终端实测字段名(Mean reward/Mean value loss/Mean surrogate loss/Mean entropy loss/Mean action std/Steps per second/Collection·Learning time)与逐行判读、『起来了的最小判据(字段齐全+每迭代在变+无 NaN)』，并点破『终端不打印 kl/entropy 要去 TensorBoard 看 Loss/mean_kl』『认语义不认字面』。
- wrapper 节新增 UniLab『两层组名自检脚本』note：打印 env 侧组名维度(obs/critic)与 runner 侧 obs_groups 路由并当场对账，标出红旗(actor==critic 维或 actor 误指 critic 组)——把『最易写混』从告诫升级为可机器核对，与 mjlab 侧打印 obs 维度的练习对称。
**优化补充 (3)**
- Quick Start 新增 steps/s 估算心法 + RTX 4080S 冒烟实测锚点表(mjlab 64env≈1067、UniLab mujoco 16env≈381、motrix 16env≈361)：明确『steps/s≈num_envs×控制频率×并行效率、小 env 冷启见几十~几百 sps 属正常』，并把正文 8900 标注为大并行热稳态值——兜住『拿小冒烟数对稳态数会误判性能回归』。
- 把 UniLab init_std=0.50(实测) 的事实补进首屏判读 note：点明『UniLab 默认探索 std 比 mjlab/Isaac 的 1.0 小、起步更偏利用』，补全三框架 action std 对照。
- 源码阅读路线 C 与版本表均补『inspect+grep 自定位』的高级核实技巧索引，统一指向新增导航 note——本章追求的『怎么核实而非背诵』。

## prac_domain_rand.tex (域随机化)
**硬不符修正 (3)**
- NaN 陷阱(原 line689): 原文把 pseudo-inertia 分解失败归因到 torch.linalg.cholesky, 报告实测 mjlab 真实实现用自定义 _cholesky_4x4(为避 cuSOLVER 在 CUDA Graph capture 下的动态分配)而非 torch.linalg.cholesky——改为'与用哪种 Cholesky 实现无关, mjlab 并不调 torch.linalg.cholesky 而是自写 _cholesky_4x4, 但近奇异致 NaN 对任何 Cholesky 都成立', 并加 finenote 点破其工程动机, 修正报告所指'NaN 陷阱归因略脱节'。
- Quick Start 摩擦语义(报告'轻微不符': shipped Go1 默认 operation=abs、ranges=(0.3,1.2)绝对值, 章节用 scale+(0.5,1.25)倍率): 章节的 scale 是有意教学选择(报告判'非错'), 故未改代码, 改为补一条跨框架 finenote 厘清'ranges 到底是绝对摩擦系数还是倍率'(mjlab 取决于 operation 默认 abs/Isaac 恒绝对/UniLab 恒倍率), 避免读者把 abs 区间当倍率误缩摩擦。
- push 参数(报告'轻微不符': shipped interval_range_s=(1.0,3.0)较激进且 velocity_range 为完整6维, 章节示例用(10,15)与平面x/y): 章节是作为推荐区间给出(报告判可加注非错), 故未改示例, 在 force-tune 节补 finenote 注明 shipped 默认值与差异, 防读者把'被推频繁+上下颠簸'当 bug。
**教学缺口补充 (3)**
- 从零搭建缺口(报告教学缺口#1, 最看重): 在 sec:rlmc-dr-inertia-expand 新增'从零写一个新 DR 函数: 端到端最小路径'——把散在各处的装饰器声明/原地写/挂events/verify 串成一条可照敲的五步通用模板(以 dof_frictionloss 为例, 10行可跑), 并给排错指针(assert挂查装饰器import, std≈0查换指针), 教读者随机化章节未覆盖字段时举一反三。
- 一次真实排错命令序列(报告教学缺口#3, 最看重): 在 sec:rlmc-dr-diag 新增'一次真实排错的完整命令序列: 从 std=0 到定位到那一行'——用 [text] codebox 复盘 cross-env std=0 的逐层 ipython/pdb trace(体检总览→以 mass 同机制正常排除全局未expand→pattern匹配数排除配置层→data_ptr() 前后比对实锤仿真层换指针bug→改原地写复活), 把四层法从概念变成可照敲命令, 并提炼'只有一项 vs 全部为零'的排除法次序。
- 为新机器人从规格书推导 DR 范围(报告教学缺口#2): 在 sec:rlmc-dr-func-dist 新增方法论段——5步反推流程(先列不确定性来源选add/scale; 质量从BOM+负载反推add; PD从标定精度反推scale+log_uniform; 传感器噪声查datasheet; 无datasheet时窄区间起步+system ID锚定), 把'经验值堆砌'变成'每项都能在BOM/datasheet/标定报告指出物理来源'的可操作方法。
**优化补充 (3)**
- 真实吞吐基准(报告优化点'真实steps/s'): 在 sec:rlmc-dr-env 加 finenote 给 RTX4080S 实测锚: train Mjlab-Velocity-Flat-Unitree-Go1 @64env≈1.46s/iter≈9k env-steps/s、2迭代mean reward≈-0.63无报错; UniLab go2_joystick_rough @16env≈1.34s/iter tracking首迭代非零——作为'DR开着也能起飞'的下限锚, 并提示低一数量级多半是 set_const 字段误入高频模式。
- mjlab DR 子模块进阶索引(报告优化点'40+ DR函数'): 在函数对照表 finenote 末补——dr 子模块实测导出40+函数, 列出 camera/light/material/geom_rgba(视觉渲染DR)、tendon_*/site/jnt_range/joint_friction/dof_frictionloss(肌腱/限位/摩擦损耗)等超 locomotion 子集的能力, 给 dir/print 列举一行+'能复用就别自己写'指引接到从零模板。
- pseudo_inertia 的 _cholesky_4x4 工程动机(报告优化点'自定义4x4 Cholesky规避cuSOLVER'): 随 NaN 陷阱修正一并加 finenote 点破——cuSOLVER 运行期动态分配/句柄踩 CUDA Graph capture 雷区, 闭式手写4x4 Cholesky无库句柄、纯逐元素kernel天然Graph安全, 与'原地写入'同源, 是本章 CUDA Graph 主题的高级延伸。

## prac_imitation.tex（模仿学习工程实践 / P8 强化学习运动控制部）
**硬不符修正 (3)**
- Quick Start note(原 line95-96)『mjlab tracking 裸 train 会用任务默认示例动作做冒烟』——报告实跑确认实为 ValueError(源码 motion_file='' 默认空, 无任何默认动作)：改写整条 note, 明说『不带任何默认/示例动作, 裸跑直接 ValueError』, 并给出两条喂动作的正确路子(--env.commands.motion.motion-file 或 --registry-name)。
- mjlab Quick Start codebox(原 line77-83)两条命令照抄必报 ValueError——按报告实跑事实重写：补 Step 0 借框架样例 .npz 设 $MOTION, play/train 均显式加 --env.commands.motion.motion-file, 并把训练改为实跑过的 64env/2iter(~0.9s/iter)。
- UniLab Quick Start codebox(原 line87-93) train 冒烟行漏标 HF 依赖, 报告实跑裸跑 RuntimeError: client closed——在 codebox 顶部统一 export HF_ENDPOINT=https://hf-mirror.com(覆盖 demo+train 两行), 并补实测 363 steps/s 与逐项 reward 打印说明。
**教学缺口补充 (5)**
- 新增小节 sec:rlmc-imit-data-bootstrap『从零起步：拿到第一份能跑的 .npz 并自验契约』——补报告头号教学缺口(数据从哪来全章缺位): 给零下载/零 retarget 捷径(复用 UniLab 样例 npz), 含可复制命令 + text 预期输出(打印『数据契约通过』+实测 shape joint_pos(870,29)/body_pos_w(870,31,3)), 把悬空的 check_motion_contract 接到真实数据看到 PASS 形成闭环; 另附 note 讲正经项目第一份数据三条来路。
- 在八阶段协议后新增『这些指标在哪看、长什么样、多少算过线』段 + 表 tab:rlmc-imit-logkeys——补报告教学缺口(协议只讲看什么不讲在哪看/数值参照): 列实跑日志真实 key(Metrics/motion/sampling_entropy≈0.98、error_anchor_pos≈0.77、Episode_Termination/anchor_pos 等)与量纲参照, 教读者去 stdout/WandB 看而非另写脚本。
- 新增 pitfall『早期大量「早停」不是训练崩了』——补报告排错缺口: 解释 mjlab tracking 早期 64/64 env 在头 2 iter 大量触发 anchor_pos/ee_body_pos 早停属正常, 教『看趋势不看单帧绝对值』的判健康纪律。
- 故障排查手册表头新增两行——补报告缺口(最易卡新手的两个首跑报错未进手册): mjlab 裸跑 ValueError→加 motion-file/registry-name; UniLab RuntimeError client closed→先 export HF_ENDPOINT 镜像。
- AMP 节开头新增 note『本节代码是原理示意, 可运行实现走 ProtoMotions』——补报告缺口(AMP/ASE/BC 代码无可实跑入口, 是讲代码非教工程): 明确判别器/encoder/BC 片段为教学示意, 指向 ProtoMotions train_agent.py 实跑入口, 教读者把五个技巧/监控指标对到其日志。
**优化补充 (3)**
- 实战捷径(报告已验证): mjlab↔UniLab .npz schema 完全互通, 可直接拿 UniLab 自带 dance1_subject2_part.npz 喂 mjlab tracking 冒烟, 无须 retarget/registry——落到新 sec:rlmc-imit-data-bootstrap 的命令里, 是『schema 同构』主张最省事的可操作落地。
- 真实 steps/s 与迭代时间基准(报告实跑): mjlab tracking ~0.9s/iter、UniLab g1_motion_tracking ~363 steps/s——写进两处 Quick Start codebox 注释, 给读者『跑出来该长这样』的锚点。
- mjlab 现成部署变体 Mjlab-Tracking-Flat-Unitree-G1-No-State-Estimation(has_state_estimation=False, 砍 base_lin_vel)——补在 anchor 观测段(line703), 与 UniLab G1WBTObs 对称, 形成三框架『特权 teacher→可部署 student 信息边界』现成对照(原章节只点了 UniLab 一侧)。

## prac_assets.tex — 机器人资产全链路：URDF 到 MJCF/USD（P8 强化学习运动控制部）
**硬不符修正 (2)**
- L621-635 note 环境未闭合(预存 LaTeX bug,非报告项): \begin{note}[...离线自检...] 缺 \end{note},导致其后 joint_drive 段+rad/deg pitfall 被吞进 note → 在 L634『要点』段后补 \end{note},全文 note 环境 6/6 配平
- 报告所有『工程核对·不符』项经核查均已在前一轮修订中改正,本轮无需重复改: unilab-pull-assets --robot 仅 x2 不拉 Go2(已在 L64/127-130 改正)、coacd/obj2mjcf/trimesh 非自带须 pip 装(已在 L50/62)、裸 go1.xml actuators=0(已在 note L93-97)、MjSpec.attach 传 pos/quat→TypeError(报告判『是这样/纠正正确』,L941 保留不动)、validate_model 求解器映射['PGS','CG','Newton'](报告判正确,L1032 不动)
**教学缺口补充 (1)**
- 惯性节 L879 新增 note『别只信这段结论:教你自己去框架源码核实内建了什么 DR』(label note:rlmc-assets-verify-claim): 把报告教学缺口#5『只给结论没教读者怎么自验框架能力 claim』补成可复制命令序列——dir() 列 DR 能力清单 + inspect.signature 验 alpha_range 旋钮 + grep RESET_TERM_BODY 定位 UniLab reset 粒度,每条带期望输出,授人以渔且复用 note:rlmc-assets-offline-api 同一手法
**优化补充 (2)**
- 惯性节 L876 段后新增 \paragraph『从知道有到会用:在 cfg 里启用 mjlab 的 pseudo_inertia DR』+ Python codebox: 补报告优化点#5,给 dr.pseudo_inertia(alpha_range=...) 的最小调用范式(点明 alpha_range 是 e^{2α} 密度对数扰动、Cholesky 重构内部保正定无需再兜底),从『知道有』落到『会用』
- 同处补 \pzr 警示: mjlab 源码里直接乘质量的 body_mass DR 已 deprecated、官方推荐改 pseudo_inertia(因 body_mass 只缩放质量不动惯量/质心、物理不自洽),呼应报告优化点#5 的源码 L302/309 提示

## prac_actuator (Actuator 建模与系统辨识) — /home/ziren2/pengfei/Robotics_Theory/Robotics_Note/parts/P8_rl_motion_control/prac_actuator.tex
**硬不符修正 (6)**
- Quick Start step_response (原 L81-94): 原 target=1.0 使命令力矩 kp×1.0=100 N·m 远超 forcerange±33.5, Ideal/带宽两版都饱和到 33.5, 章节『带宽峰值≈28 明显低于 33.5』失真——改 target=0.2(命令力矩 20<33.5 不饱和)、新增 rise_time_ms() 辅助、改打印『峰值+上升时间』而非仅 peak.max(), 让带宽差异(上升时间)真正可见。
- 正文 L129 误述(『带宽峰值约 28~N·m』)与 L477 运行验证(『爬升到约 28』): 均改为『不饱和时两版峰值都≈20、差异在上升时间』, 删去失真的 28 数值。
- apply_sysid_mjspec (原 L797): `jnt.damping = sysid[...]` 在 MuJoCo 3.8.1 抛 TypeError(MjsJoint.damping 是 [3,1] 数组)——改为 `jnt.damping[0] = ...` 索引赋值, frictionloss 标量赋值保留并加注。
- UniLab PdControlConfig (原 L296): `simulate_action_latency: bool = True` 事实错——改为默认 False; 并据源码(L29-33 仅 Kp/Kd)标注 action_scale/simulate_action_latency 系继承自 ControlConfigBase 而非本类自有字段。
- PdControlConfig 源码路径(原 L115/L290 注释 + L1100 版本速查): `locomotion/common/base.py` 缺 envs/ 段——三处统一改为 `unilab/envs/locomotion/common/base.py`。
- 版本信息速查 UniLab actuator 条(L1100): 同步修正 simulate_action_latency 默认 False+继承关系、路径补 envs/。
**教学缺口补充 (4)**
- 新增 pitfall『别让饱和遮蔽带宽』: 讲清饱和(forcerange 钳幅)与带宽(filter 让力矩爬升)是两个独立现象、命令力矩超 forcerange 时饱和会盖住带宽差异, 给出『想看带宽先让它别饱和』口诀+可观测设计(先算 kp×误差<forcerange、看上升时间)——直接补上 Quick Start 演示掉进自己坑的最伤教学缺口。
- 新增 pitfall『MjSpec 数组字段标量赋值抛 TypeError』: 教读者数组字段(damping[3,1]/gainprm/biasprm/dynprm)须 [i]= 索引赋值、标量字段(frictionloss/dyntype)直接赋, 并给『先 print(type(...), shape) 再决定赋值方式』的通用排错手段(查而非背)。
- 新增『从零跑通一次改了 actuator 增益的训练并自检』bash 命令序列: mjlab(WANDB_MODE=disabled+--env.scene.num-envs/--agent.max-iterations)与 UniLab(Hydra env.control_config.Kp/Kd 落地本节增益)两侧端到端可复制命令, 每条标注预期输出(steps/s、loss 量级、reward 字段), 并教用 `train <TASK> --help | grep` 自查真实 flag 名而非背。
- <dcmotor> 段补『裸标签编译报 motor constant K must be positive』实跑坑+『查官方 XMLReference 而非照抄博客』习惯; 并新增一段已实跑的 mjcb_control 替代演示(按力矩-速度公式四象限显式 clip + 扫速度打印 tau_max(q_dot) 预期 0→33.5/5→25.5/10→17.5/20→1.6), 把『只讲 <dcmotor> 存在』升级为『看得见、可自验的 DC Motor 约束』。
**优化补充 (3)**
- 新增 insight『实测锚点：怎么判断 steps/s 算不算正常』(label ins:rlmc-act-sps-anchor): 给 RTX4080S 实测 mjlab Go1@64env≈1000 steps/s、UniLab go2@16env≈400 steps/s、loss 量级 1e-2; 配『稳态 steps/s≈num_envs×控制频率×并行效率, 小并行/冷启低属良性』估算心法与三点判正常法(无 NaN/随 num_envs 线性/热身趋稳)。
- Level 2 集成段新增 mjlab 原生学习型 actuator LearnedMlpActuatorCfg 配置示例(字段 network_file/history_length/input_order/pos_scale/vel_scale/torque_scale, 实跑核对), 把 mjlab Level 2 从 mjcb_control(CPU 回调)升级到 GPU 原生路径、与 Isaac ActuatorNetMLPCfg 对位; 并点明三个 *_scale 须等于训练集归一化统计量。
- UniLab Quick Start apply_action 注释补 per-joint action_scale 工程细节(go2 rough: hip 0.125/非 hip 0.25, 实跑核对), 解释『对 hip 用更小 scale 抑制侧摆』及 action_scale 作为除 kp/kd 外另一个稳定性旋钮的工程直觉。

## prac_quad_loco
**硬不符修正 (6)**
- §terrain (旧 line 956-973): 原断言「UniLab 当前任务集没有把 raycast 高程网格作为内置观测传感器 / 无对应内置传感器 / 需自行在 _compute_obs 实现」是硬错——实测 UniLab 内置 common/height_scan.py(HeightScanConfig.enabled 默认 True, init_height_scan_sensor + height_scan_obs)，go1/go2/go2w rough 都把 height_scan 拼进 critic。已整段改写为「内置但只喂 critic(特权)，actor 盲式」，并改 codebox 为 go2/rough.py 实测 obs 装配(obs:45/critic:48+scan)，明确 ray 数→obs 维公式在 UniLab 同样适用(只是进 critic)。
- 环境配置表(line 43): UniLab raycast 列「CPU 后端 heightfield(无 GPU raycast)」会被误读为无 scan obs，改为「CPU 后端 height scan(内置，默认只进 critic)」，并给 mjlab/Isaac 列补「默认进 actor」。
- §rlcfg note(line 1123): mjlab 1.4.0 绑定的 rsl-rl-lib 版本 5.2.0 → 5.4.0(实测，注随 uv.lock 漂移)；同段源稿纠错句里的「mjlab 1.4.0 装 5.2.0」一并改 5.4.0。
- §versions 表(line 1809): RSL-RL 行「mjlab 1.4.0 装 5.2.0」→「装 5.4.0(实测，随 uv.lock 漂移)」。
- §termination(line 755 codebox)+§terrain-curriculum(line 1005 后): 补「UniLab go2 rough 的 terrain_curriculum 键虽在但实测默认 enabled=false 且 generator.curriculum=false」——即「rough owner 带 curriculum 配置 ≠ 默认开启」，原文只说带 curriculum 易让读者误以为默认开。
- §reward codebox(line 590-613)解读: 补一条——mjlab 1.4.0 实跑 Go1 flat 的 Episode_Reward/* 实际还出现 foot_swing_height/soft_landing(codebox 未列)，而 codebox 列的 joint_torques 因权重极小当步可能不显(值≈0)；强调以日志为准核 term 名。
**教学缺口补充 (4)**
- 新增 insight ins:rlmc-ql-grepchain「『缺页』往往是没去 grep 出来的幻觉」: 给出可复用三步实证链(grep 源码关键词 → import 实例化看默认值 → 读真正的 obs 装配确认进 actor 还是 critic)，配可复制 bash 命令，把『某框架到底有没有 X 能力』的判断从凭印象升级为去装好的库里自证——正用本次抓出的 height-scan 错为反面教材。
- §train-diag 日志字段对齐真实输出: codebox Step2 与其后新增 note 把泛化字段名(reward/track_*、policy/kl、terrain/level)逐一映射到 mjlab 1.4.0 实测真实字段(Episode_Reward/track_linear_velocity、Metrics/twist/error_vel_xy、Curriculum/command_vel/lin_vel_x_max、Curriculum/terrain_levels/{mean,max}、Episode_Termination/{time_out,fell_over})，并点明 mjlab 终端摘要根本不打印 kl(要看 KL 须开 WandB/TB)——避免读者照表找字段扑空，并教『去 scalar 树对号入座』的通法。
- §obs 新增 note『维度可一眼自验』: 教读者不靠背数字核 obs 维——UniLab go2 起训日志打印的 Critic in_features 直接等于 critic obs 维(flat 实测 52)，mjlab 看启动 Observation term 表与各 term shape(rough 实测 height_scan (187,))；维度对不上第一步永远读这张启动表而非去 cfg 数 term。
- §terrain-curriculum 新增 pitfall(开启 UniLab 地形课程): 把『go2 rough 跑起来 terrain/level 不动』做成一次真实排错——先 grep 出两个开关真实取值，再给同时打开两处的可复制 Hydra 命令(env.scene.terrain.generator.curriculum=true + env.domain_rand.terrain_curriculum.enabled=true)，并提炼『配置项存在但行为没发生 → 先 grep 实际默认值』的纪律。
**优化补充 (3)**
- 三框架真正的 height-scan 对照(替代原『缺页』标注): 新增『三框架对照』段，把 UniLab 的内置 scan→critic 定位为比 mjlab/Isaac 默认 scan→actor 更干净的非对称 teacher 范例(特权地形信息只训练侧用，actor 盲式部署，sim-to-real 负担更小)，让本节 ray 数→obs 维公式在三框架统一成立，只需核对『进 actor 还是 critic』。
- §obs 新增 note 点明 flat 与 rough 的 actor 维度结构不同: go2 flat actor=49(含 4 维 feet_phase 步态相位)，rough actor=45(无 feet_phase、盲式) + height_scan 进 critic；纠正原文用同一 48/49 套两个任务、强调 flat→rough 是『换一组本体特征 + scan 只补 critic』而非『actor 加地形维』。
- §reward 解读补『用训练日志逐项验 term』高级技巧: 给可复制命令(train ... --agent.logger tensorboard，首跑加 WANDB_MODE=disabled)，说明第一屏起 Episode_Reward/<term> 每行即一个真实 reward term——比读 cfg 源码更快的 wiring/term 名自检，并衔接 ablation 改权重前的防呆(改错 term 键 mjlab 会 tyro 直接报错)。

## prac_humanoid_loco.tex（人形 Locomotion：从四足到双足）
**硬不符修正 (6)**
- mjlab Quick Start codebox(~L53): `python scripts/train.py Unitree-G1-Flat` 在 autodl 跑不通(只装 mjlab core 未装 unitree_rl_mjlab)→改为先 `uv run list-envs` 自查, 再给路线A(`uv run train Mjlab-Velocity-Flat-Unitree-G1`)与路线B(scripts/train.py), 并把 `--checkpoint_file=`(下划线)全改为 `--checkpoint-file`(连字符)
- 命名陷阱(~L86): 原断言写 `Mjlab-Velocity-Flat-Unitree-G1` 错、`Unitree-G1-Flat` 对——与实跑相反(mjlab core 本体即注册前者)→整条重写为『两套命名并存, 装什么决定能用哪套』, 给出 list-envs+importlib.find_spec 自查命令
- UniLab scales codebox(~L588): 与本机 g1_walk_flat/mujoco.yaml 不符→tracking_lin_vel 1.0→2.0、base_height -100→-500、base_height_target 0.76→0.754, 并注明取自实跑核对
- mjlab friction/push 扫描 codebox(~L907): `Unitree-G1-Flat --checkpoint_file=`→改为 mjlab core 名 `Mjlab-Velocity-Flat-Unitree-G1 --checkpoint-file`(连字符, autodl 可跑形式)
- action scale 示例表(~L298-320): 表内 s_j 绝对值(0.147/0.174/0.313/0.031)与本机实测(~0.55/0.35/0.44/0.075)差3-4倍, 因 mjlab stiffness 由 armature×natural_freq² 推导→补 \pzr 标注真实值与差异, 保留『腕最小约7×』相对结论, 指向脚本现算
- 本节动手实验命令(~L244): `python scripts/play.py Unitree-G1-Flat`→`uv run play Mjlab-Velocity-Flat-Unitree-G1`; 销账原对 `--no-terminations` 存疑的 \pz(实跑确认存在), 改注 play 单数 --num-envs 与 train 的 --env.scene.num-envs 不通用
**教学缺口补充 (4)**
- 『从零搭起·安装自查』: 在重写的命名陷阱内嵌入可复制的 list-envs+find_spec 自查 codebox, 教读者『装什么决定任务名』, 报错前先自查而非背任务名
- 新增『2 迭代冒烟读 term 量级』最小步骤(sec:rlmc-hl-reward 运行验证处): 给可复制命令(WANDB_MODE=disabled uv run train ... --agent.max-iterations 2)+预期 stdout 节选(Episode_Reward/各项、Metrics/*、fell_over), 教读者用最低成本确认接线/各 reward term 真生效, 并配新 insight ins:rlmc-hl-smoke 讲『出现且非零=接线对、看初始量级再调权重』
- 提取脚本溯源教学(~L364): 补 \pz 说明该脚本读已编译模型 actuator_gainprm 的最终 stiffness, 故无论资产怎么写都给真值——这正是示例表对不上时应以脚本输出为准, 教『配 scale 前先跑一遍』的工程习惯
- 故障排查手册(~L1196)补『通用起手式』: 第0步 list-envs/find_spec 自查环境(两套命名), 第1步 2迭代冒烟读 term——把最通用的排查起手式写进手册, 并说明下表只用于冒烟通过却跑偏的行为级症状
**优化补充 (4)**
- 真实 steps/s 基准: 标注 autodl(RTX 4080S, 64 env) 实测 ~1285 steps/s、~1.4s/iter, 并提示小并行偏低属正常(PPO 靠大并行, 4096 env 才近稳态), 兜住读者跑小冒烟见低 sps 的误判
- WANDB_MODE=disabled 前提: 冒烟命令显式带上(mjlab 默认 logger=wandb, 否则冒烟也触发登录/上传)
- UniLab checkpoint 定位: 补 \pz 给出训练日志落点 logs/rsl_rl_ppo/G1WalkFlat/<时间戳>_mujoco/(CamelCase 注册名、含 git diff 快照), 并解释 --load-run -1 与 --render-mode {auto,interactive,record,none} 取值, 便于读者找回放 run
- play/train 并行数 flag 不一致(play 单数 --num-envs vs train --env.scene.num-envs)在 Quick Start 实验注与练习里显式点出, 避免照搬另一个报 Unrecognized

## prac_motion_imitation
**硬不符修正 (11)**
- Quick Start codebox (L74-83): mjlab `play/train Mjlab-Tracking-Flat-Unitree-G1` 原缺 motion 文件 → 实跑直接 ValueError 拒绝启动；两条命令均补 `--env.commands.motion.motion-file /path/to/g1_walking.npz`，并加注前提步骤。
- 新增 pitfall (L86): 据报告实测真值，明确「tracking 任务无 motion 直接 ValueError: For tracking tasks, provide --registry-name or --env.commands.motion.motion-file」「仓库不附带任何 bundled .npz（glob 全空）」「这是与 velocity 任务的关键差异」。
- 完整流程 codebox (L489-509): Step2 zero play / Step3 small train / Step4 large train / Step5 eval 原均缺 motion → 全部补 `--env.commands.motion.motion-file $MOTION`，并把 Step1 补成真实 csv_to_npz 产出+赋值 MOTION 变量。
- Note『与速度跟踪 Quick Start 的关键差别』(L99): 原说零动作回放即见 ghost → 改为「必须先带一条 motion 文件，跑起来才会有 ghost」，呼应上文 ValueError pitfall。
- UniLab reward codebox (L331-342): 原写全局 `tracking_sigma: 0.25` 且旁注『逐项σ需自定义』(说反) → 改为真实的 8 个 per-term `std_*` 字段(0.3/0.4/0.3/0.4/1.0/3.14，与 mjlab 同)，删除全局 tracking_sigma。
- UniLab reward codebox (L331-340): `motion_joint_pos/motion_joint_vel/motion_ee_body_pos_z` 原标 weight=1.0『活跃额外项』 → 实测默认 scale=0.0(函数已注册但未启用)，改为 0.0 并注明 ee_body_pos_z_threshold=0.25。
- 新增 pitfall (L353): 澄清「UniLab 跟踪奖励无全局 tracking_sigma——是 8 个逐项 std_*(与 mjlab 一致)，逐项σ本就是默认无需自定义」，含『不同物理量量纲不同所以必须逐项 std』的为什么。
- prose L319: 原把 motion_ee_body_pos_z 描述为 UniLab 活跃额外项 → 改为「预置但默认关(scale=0.0)，含 joint_pos/vel/ee_body_pos_z 三者，函数在 tracking.py 注册但默认未启用」。
- KungfuBot 节 prose (L1068): 原说『UniLab 用一个全局 reward.tracking_sigma』『手调 cfg.reward_config.tracking_sigma』 → 改为「逐项 std(8个 std_*，无全局 tracking_sigma)，复刻 tracking factor 的真实抓手是这 8 个 std_* 字段」。
- 跨框架 insight (L1058)+三框架表 (L551): 原称跨后端校验开关 `training.sim2sim_strict=true`(默认) → 全仓 grep NOT FOUND，改为「sim2sim.py 与 CrossBackendIncompatibleError 确在，但不存在 sim2sim_strict flag，该校验是流程内置硬行为」；run 表回放列改标 `play --agent zero --motion-file（必带）`。
- 故障排查表 (L1237): 新增一行『play/train 报 ValueError 拒绝启动 → tracking 未带 motion → 加 --motion-file/--registry-name；仓库无 bundled npz』。
**教学缺口补充 (2)**
- 新增 practice『从零搭起：第一次跑通一条 tracking 任务的最小步骤序列』(L512): 补齐报告点名的『起步链断裂』——5 步最小闭环(拿到 motion→zero-play 看 ghost→静态核配置→small train 冒烟→正式训练+评估)，每步给『跑完该看到什么』与排错分支(仍 ValueError⇒路径拼错；ghost 不显示⇒回数据契约而非调训练)。
- 新增静态自检 codebox (L519): 教『不跑训练先核配置』——先 `grep class.*EnvCfg $PKG/tasks/tracking/` 自己定位本地 cfg 类名(不记死)，再 import dataclass 遍历打印 weight/std/body_names，让 body_names 拼错在训练前现形；并附『关键不在脚本逐字照跑而在掌握这一招』的举一反三提示(回应报告『故障表只说打印 id 却无可复制命令』的缺口)。
**优化补充 (3)**
- 新增 Note『一条 UniLab 跟踪冒烟的实测基准(RTX 4080S)』(L565): 补真实 ~343 steps/s 与非对称观测维度 actor 29 / critic 286——作为 ins:rlmc-mi-obs『逐 body 参考量只喂 critic』的实测印证；并提示 16env 冷启偏低属良性、给 sps≈N_env×f_ctrl×η 估算心法。
- 同 Note 补 off-policy 入口: `--algo` 实测六种 ppo/mlx_ppo/appo/sac/td3/flashsac，td3/flashsac 可作 off-policy 备选(走 training.num_gpus=N 多卡原生)。
- 训练前静态自检 codebox 同时作为『读源码核对真值』的优化技巧落地(本章 std/阈值/body_names 真值正是这样静态读出来的)，比『跑起来再看 reward 涨不涨』更早暴露配置错误。

## prac_wheeled_bimanual.tex（轮式底盘与双臂移动操作）
**硬不符修正 (9)**
- 【API名·MjSpec.attach_to不存在】spec attach 代码框(原 arm_spec.attach_to(base_spec,mount_site,prefix=))→改正为 base_spec.attach(arm_spec, prefix='yam/', site=mount_site)，并加注 mujoco 3.8.1 实测真名为 attach、调用主客方向相反、site/frame 二选一、返回 MjsFrame。
- 【API名·attach_to】attach 节正文(原'臂 spec attach 到底盘')→补明'真实方法 parent.attach(child,...)，mujoco 3.8.1 无 attach_to，底盘在前臂作参数'。
- 【API名·attach_to】三处表格单元(setup表 line43 / 集成对照表 line1252 / 版本速查表 line1448 的 \verb|MjSpec.attach_to()|)→全部改为 base_spec.attach(arm_spec,...) 形式；版本表 MuJoCo 行补'实测3.8.1；MjSpec.attach 无 attach_to'。
- 【概念讲反·UniLab Go2W轮控】setup表 line42 单元(原'位置式 PD（无 velocity-action 类）')→改为'速度级（Go2W 轮 wheel_Kd*(v_tgt-v)，专属 wheel_action_scale；无 velocity-action 类）'。
- 【概念讲反】亲戚说明 line51(原'轮子由后端位置执行器驱动')→改为实测速度级：wheel_velocity_targets=action*wheel_action_scale、compute_go2w_motor_ctrl 用 wheel_Kd*(v_target-v_cur)、不残差到 default_angles。
- 【概念讲反】Quick Start 注释 line77(原'轮子由后端位置式 PD 执行器驱动')→改为速度级公式说明。
- 【概念讲反】base-api 对照表 line380 / 混合动作对照表 line511 两单元(原'位置式 PD'/'无 velocity-action 类（位置 PD）')→改为'速度级（Go2W 轮 wheel_Kd*(v_tgt-v)）；无 velocity-action 类'，并区分'无 cfg 类'≠'只能位置控制'。
- 【概念讲反·核心教学点被讲反】UniLab 混合动作整段 line520 + 代码框 line522-533(原把轮=位置PD、称'要速度控制得自己改 apply_action')→重写为 Go2W 真实双路并存：腿走残差位置PD、轮走速度级(各引 go2w/joystick.py 与 base.py)，代码框改成 wheel_act 速度PD 段 + arm_act 残差位置段两路；并改正'无需 hand-roll 速度控制、轮子有专属 wheel_action_scale=10.0'。
- 【概念讲反·连带】章首 line27 UniLab 接线描述(原'残差式位置 PD 动作')→改为'臂走残差位置PD、轮走速度级两路并存'；line51 末 rebuilt 注(原称轮速接口未逐字核对)→改为已实跑核对。
**教学缺口补充 (4)**
- 【从零自查真实API+一次真实排错】attach 代码框后新增'别背 API 名，现场探明它'段+bash 代码框：展示草稿误写 attach_to 触发的真实 AttributeError，再用 dir(mujoco.MjSpec) 过滤关键字 + help(mujoco.MjSpec.attach) 两步问出真实签名（含实测输出），并配 insight(ins:rlmc-wh-probe)'查不到就探针'心法——直接补齐报告教学缺口#2(章节自相矛盾于'先探针再写')。
- 【通用伪码→现成可跑命令】Action scale 节 random-agent 伪码(make_env)后新增 bash 代码框：mjlab 现成 play <TASK_ID> --agent zero/random --viewer viser，含 zero/random 各自预期现象、scale 选大选小的体感判据、play 用自身 --num-envs 而非 train 的 --env.scene.num-envs 的提醒——补报告教学缺口#4。
- 【别背维度·让框架自报】obs 健康检查框后新增 note：点明 actor 组键名 mjlab=policy vs UniLab=obs(拿错 KeyError)，并给 UniLab obs_groups_spec 可实测自查 bash 代码框(Go2JoystickFlat 实测 {'obs':49,'critic':52})，把'数 42 维'变成'打印 spec 自查'的可操作技能——补报告教学缺口#5。
- 【Quick Start 可复制命令+预期输出+诚实化】UniLab Quick Start：冒烟参数改为 16env/1iter 并补预期首屏(~340 steps/s 冷启属正常/reward 更新/无 NaN)；demo locomani 标注实测需本地 stage-2 checkpoint 否则缺权重退出，避免读者照抄卡住。
**优化补充 (3)**
- 【真实 steps/s 基准锚点】性能节(原仅 4090 60-80% 相对比例)补 RTX4080S 实测锚点：固定基座 Mjlab-Lift-Cube-Yam 64env≈1355 steps/s、UniLab go2w_joystick_flat 16env 冷启≈340 steps/s，并补'小并行偏低属正常、吞吐随 num_envs 近线性、别拿冒烟数字当性能回归'的判断心法。
- 【三框架动作延迟对照补全】Action Delay 节 note 由'Isaac 内置 vs mjlab 手动'两方扩为三方 itemize，补 UniLab 内置开关 control_config.simulate_action_latency(+last_actions 一拍延迟)，与本节'延迟放 actuator/action 层非 step event'论断互证。
- 【轮/臂量纲对齐现成旋钮】混合动作段与代码框补 Go2W 自带的专属 wheel_action_scale=10.0(与腿 action_scale=0.25 分离)，指出可直接借此对齐两段量纲、不必在 apply_action 里手算分段缩放。

## prac_diy.tex（DIY 实战：从自定义机器人到完整训练）
**硬不符修正 (12)**
- mjlab Step 4 观测块(原§sec:rlmc-diy-mjlab-sixstep)：原用 Isaac 风格 class ObservationsCfg/class PolicyCfg → 改为 mjlab 真实的 dict[str,ObservationGroupCfg] 形态、键名 actor/critic（实跑真名，非 policy）；并加一段说明 cfg.observations 是 dict、组名是 actor
- mjlab Step 6 组装(MyQuadEnvCfg)：observations 字段从 ObservationsCfg 类 → dict（default_factory=lambda: observations），与上面 dict 形态一致
- smoke_test.py(§sec:rlmc-diy-threestep 第一步)：obs["policy"] 两处 → 动态 ACTOR=next(iter(obs.keys()))（真 mjlab 跑原代码 KeyError:'policy'）；并加 print(list(obs.keys())) 为必做第一步
- zero agent 通过标准：删除错误硬判据『reward 应为小负值』→ 改为『符号取决于配了哪些项』（实测 Go1 含 alive/upright 时 +0.076 为小正值），真判据=不倒+无NaN+小常数；补 base 高度 0.307→0.250 实测
- §Step3-4 UniLab 正文：『mjlab 用 policy/critic』→『mjlab 内置任务用 actor/critic，Isaac Lab 用 policy/critic』（原文是被报告 line632 点名的错误声明）
- Isaac Lab Step 3 引言：补两框架差异说明——mjlab 用 dict+actor 组名、Isaac 用类树+policy 组名（保留 Isaac 侧 class PolicyCfg 不动，因其为 Isaac 正确写法）
- 双框架一致性脚本(§sec:rlmc-diy-pipe-dual)：mjlab 侧 observation_space["policy"] → ["actor"]，Isaac 侧保留 ["policy"]，并加注两框架 actor 组名不同
- sim2sim_eval(§Step8)：obs["policy"] → 动态 ACTOR 键（eval 默认 MyQuad 为 mjlab 任务）
- Bug 3(obs维度)：补『第0步先 print 组名确认 actor/critic vs policy、拿错组名直接 KeyError』+ mjlab 官方 introspect 入口 observation_manager.compute()/group_obs_dim/active_terms
- 完整 mjlab 代码 文件2/4(§sec:rlmc-diy-fullcode-mjlab)：class ObservationsCfg/PolicyCfg → dict 形态 actor/critic；并修 MyQuadVelocityFlatEnvCfg.observations 字段为 dict
- 观测设计决策树正文：『放 actor 组（PolicyCfg）』→『放 actor 组（mjlab 键名 actor、Isaac 类名 PolicyCfg）』
- 新增 pitfall『mjlab 观测组名是 actor/critic，照 Isaac 写 obs["policy"] 当场 KeyError』，根因+正确做法（先 print 组名再索引）
**教学缺口补充 (4)**
- 新增子节『从零探路：照着一个真内置任务把结构抄对』(sec:rlmc-diy-explore)：给出可直接复制的 introspect_builtin.py 完整 session——list-envs 取模板任务→load_env_cfg→建2-env env→对 observation/action/reward/command 四个 manager 逐个 print active_terms/维度，正好覆盖 MDP 五锚点(S/A/R/T/指令)；教『先 print 真实结构再仿』而非背 API
- 新增 insight 『查得到比背得住值钱：把核实变成可复用技能』(ins:rlmc-diy-introspect)：把『拿到陌生框架怎么三五行 print 出真实结构』讲成 DIY 最保值的元技能，并给 mjlab/UniLab/Isaac 三框架各自的 introspect 入口对照
- smoke_test 处教『第一行 print(list(obs.keys())) 不是可选的』、用 next(iter(obs.keys())) 取 actor 组使脚本跨框架可移植——把『组名从哪来(mjlab=dict键/Isaac=类名)』讲成可迁移认知
- zero/random agent 通过标准补实测锚点(Go1 zero +0.076、random epR≈-1.56/epLen 91±55)，把『reward 符号』从硬判据降级为推论，教读者不要据此误判环境坏了
**优化补充 (3)**
- Bug 10(训练慢)：补 RTX 4080S 实测 steps/s 锚点(mjlab Go1 64env≈1.45s/iter、UniLab go2 16env≈398 sps，含 JIT 冷启)，并给『steps/s≈num_envs×控制频率×并行效率』估算心法——明确『小并行下几百 sps 是正常的，不是 bug』，避免误判
- 逐步扰动 pitfall(§sec:rlmc-diy-events-mode)：点名 mjlab BuiltinPositionActuatorCfg 原生 action-delay 真实字段 delay_min_lag/delay_max_lag/delay_hold_prob/delay_per_env_phase，把『抽象建议』落成可抄的真实 API(比自写缓冲区更稳)
- Bug 4(reward常数)：补『mjlab/UniLab 训练日志默认就分项输出 Episode_Reward/<term> 与 reward/<name>，不用自己写 print』，比章节原建议的手动逐项打印更省

## prac_sim2real.tex（Sim2Real 部署全链路）
**硬不符修正 (4)**
- D0 静态核对内核(原L699): `assert "obs_dim" in meta` 在合法 mjlab ONNX 上抛 AssertionError(obs_dim 不在 metadata_props)→必查集改为实测都在的 joint_names/action_scale/default_joint_pos, obs_dim 仍从 graph 读, 并加 print(keys) 与 batch 维 WARN
- sim2sim 核心循环(原L459-466): obs['policy'][64,99] 整批喂 ONNX→实测报 INVALID_ARGUMENT Got:64 Expected:1(mjlab 1.4.0 batch 维写死=1); 改写为逐 env batch=1 推理循环, 并文档化 dynamic_axes 重导这条备选
- sim2sim 代码框注释(原L459) `# mjlab Go2 velocity`→改 `mjlab G1 velocity(mjlab 无 Go2,仅 G1/Go1)`, 消除与全章口径冲突
- 归一化 obs_mean 误断言连锁修正: sim2sim normalizer pitfall + 部署-normalizer pitfall 收尾句(原『D0 会显式查 obs_mean』) + 故障表对应行, 统一改为『先 print 全 key 判归一化架构(mjlab/UniLab baked-in 时本就无 obs_mean key, 不要无脑断言)』; sim2sim 代码框的归一化分支改为 has_norm 守卫(只在外挂 mean/std 时触发)
**教学缺口补充 (3)**
- 新增『从零跑通一遍: 命令序列+预期输出+一次真实排错』块(Quick Start 区): 短训练→find .onnx→一行 dump 真实边界(keys/in/out shape)→D0, 每条附 RTX4080S 实测预期输出; 并完整演示真实报错 INVALID_ARGUMENT Got:64 Expected:1, 教读者读 `index:0`=batch 维而非 obs_dim、根因(legacy trace 烤死 batch)、两条修法——教工程排错而非贴代码
- 新增 pitfall『把 obs_mean/obs_dim 当成每框架都有的 metadata key 去断言』: 讲清 metadata key 集随框架而异(mjlab 7 key 无 obs_dim/obs_mean、UniLab 走 deploy_config.yaml), 正确姿势是先 print 判断归一化走 baked-in/外挂/无 三条路再定必查集——举一反三点
- 新增 Isaac 侧诚实声明+自验 finenote(unitree_rl_lab 段): 显式标注『本节 Isaac 侧未在本配置实跑』, 并给一行命令 dump 真实 deploy.yaml 顶层字段 / print joint_ids_map 看是否恒等排列 / grep exporter 源码看字段来源——把『信书上字段名』升级为『读产物+读导出脚本自行证伪』
**优化补充 (3)**
- mjlab 1.4.0 真实基准/边界锚点(融入新命令块): ~1300 steps/s @64env、每次存档导单个 .onnx(非每 ckpt 一个)、input 名=obs [1,99] / output=actions [1,29]、batch 维固定=1; 配套教『没有 .onnx 时去 grep [WARN] ONNX export failed』
- ONNX Runtime warmup 部署提示(延迟节新增 finenote): 首次 .run() 冷启动尖峰(内核选择/内存竞技场/graph 优化)常被误诊为执行器/通信 jitter; 修法是部署与 D2 影子模式进控制循环前空跑 5-10 次 warmup, 对 Python D2 与 C++ 真机均适用, 呼应故障表『C++ 推理 jitter 大』一行
- D0 内核补 batch 维感知: 从 graph 读 input[0] 第 0 维, ==1 时打 WARN『只能单 obs 推理, 批量 sim2sim 需 dynamic_axes 重导』, 正好补上『动态维度没固化』的对偶坑(这里是 batch 维被写死)
