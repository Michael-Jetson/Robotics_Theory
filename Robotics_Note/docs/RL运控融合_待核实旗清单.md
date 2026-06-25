# RL运控部 P8 待核实旗清单（\rebuilt / \pz{待核实}）
自动提取，共 **212** 条。核销原则：坐实→去旗转确定陈述 / 与源不符→改正 / 确属版本免责→保留并精确化。


## prac_actuator.tex  (2 条)
- **[rebuilt] L437** [其他/待判]: 力矩常数约 0.63895~N$\cdot$m/A（出自 Unitree actuator SDK 的输出端换算，未在官方数据手册逐位核到，按 SDK 实测值为准）
- **[rebuilt] L935** [其他/待判]: 数字为相对 Ideal PD baseline 的近似经验估计，随任务/机器人/DR 配置变化，不作定量承诺

## prac_assets.tex  (3 条)
- **[pz] L718** [版本/API免责]: 待核实：coacd.run\_coacd 的入参在不同版本间略有差异——较新版本接受 coacd.Mesh 对象，旧版本直接接受 (vertices, faces)；threshold 取值范围约 0.01--1。以你安装版本的 README 为准。
- **[pz] L879** [版本/API免责]: 待核实：isaaclab\_assets 中各机器人 CFG 的确切符号名与可用清单（如 UNITREE\_GO2\_CFG、ANYMAL\_C\_CFG 等）随 Isaac Lab 版本增删，使用前请在目标版本中确认导入成功，否则 NameError。
- **[rebuilt] L905** [版本/API免责]: 上面 attach 的精确签名需以你安装的 MuJoCo 版本为准：官方文档记录的形式是 \texttt{parent.attach(child, frame=<父级 frame>, prefix=..., suffix=...)} 或用 \texttt{site=}，二者互斥；\texttt{pos}/\texttt{quat} 并非 \texttt{attach} 的直接参数，挂载位姿应设在被…

## prac_diagnostics.tex  (5 条)
- **[pz] L424** [版本/API免责]: 待核实：AGILE 实现里 \eqref{eq:rlmc-diag-rewnorm} 的具体 EMA $\beta$、$c$、$\epsilon$ 取值——以其开源仓库代码为准。
- **[rebuilt] L673** [其他/待判]: $\sigma=5$（$\gamma=0.99$ 时对应 value 空间约 500 的正向激励）
- **[pz] L901** [版本/API免责]: 待核实：源稿引用的具体 GitHub Issue 编号（unitree\_rl\_lab \#82）作为社区证据存在，但 issue 内容/编号请以仓库实际为准；正文已弱化为"社区部署中反复出现"。
- **[rebuilt] L1409** [其他/待判]: HoST 报告 L2C2 对 reward 影响 $<3\%$、对部署平滑性提升 $>50\%$
- **[pz] L1443** [版本/API免责]: 待核实：该系统报告的羽毛球来球速度上限约 $12.06$ m/s（系\emph{球速}，非球拍末端线速度）；此处作为外部工程例证引用，精确数字与上下文以原文为准。

## prac_diy.tex  (9 条)
- **[pz] L79** [其他/待判]: 待核实
- **[rebuilt] L79** [其他/待判]: 
- **[rebuilt] L1203** [版本/API免责]: 源稿出现的 \texttt{randomize\_action\_delay + mode="step"} 非框架内置 API，已按上述更正。
- **[pz] L1356** [版本/API免责]: 待核实：HOVER 的 student 历史帧数、各 mask 通道维度、训练 wall-clock，依硬件与版本差异较大，以 \texttt{NVlabs/HOVER} README 与你的硬件为准。
- **[rebuilt] L1363** [论文/数值]: 源稿把 G1 写成 29-DOF，经核论文用「23 可控 DoF（排除腕部）」，本章已改正。
- **[rebuilt] L1394** [版本/API免责]: 上面的系数 0.45 为示意值，确切 rake 角与拟合系数以 HUSKY 仓库的 MJCF 为准。
- **[rebuilt] L1969** [版本/API免责]: 凡 task id 确切字符串、各框架字段名/默认值、\texttt{register\_mjlab\_task} 签名、HOVER/HUSKY 仓库脚本路径，均以你所装版本的 \texttt{--help} 与官方文档/仓库 README 为准；本章已对未逐字确认处加注。
- **[pz] L1972** [其他/待判]: 待核实
- **[rebuilt] L1972** [其他/待判]: 

## prac_domain_rand.tex  (3 条)
- **[pz] L450** [版本/API免责]: 待核实：本部主线锚定 mjlab 1.2.0，上述 \texttt{dr} 子模块名以 1.4.0 实测为准；若用 1.2.0，配置前请在目标版本里 \texttt{import mjlab.envs.mdp.dr; dir(...)} 复核一遍（DR 包名跨小版本可能微调）。
- **[rebuilt] L748** [版本/API免责]: 各平台质量随版本/配置变动，请以目标 URDF/MJCF 的实际 \texttt{body\_mass} 求和为准
- **[rebuilt] L851** [其他/待判]: 具体规格随传感器型号变化，部署前请查目标硬件 datasheet

## prac_ecosystem.tex  (7 条)
- **[pz] L355** [性能量级]: 待核实：具体 steps/s 强依赖任务与硬件，源稿给出的 A100 上 mjlab/Isaac Lab 约 $4$--$5\times10^5$ steps/s 为社区量级估计，未逐一复现
- **[rebuilt] L364** [性能量级]: 量级参考
- **[rebuilt] L365** [性能量级]: 量级参考
- **[rebuilt] L366** [性能量级]: 量级参考
- **[pz] L454** [版本/API免责]: 待核实：源稿另列的「Newton 1.0 GA 含 7 个求解器」等求解器细分计数本书未独立核到，请以 Newton 官方文档为准。
- **[pz] L527** [版本/API免责]: 待核实：两框架公开 Manager 的确切个数随版本浮动，请以各自当时版本的 API 文档为准；本书只依赖「每个 Manager 对应 MDP 一个维度、改一个不影响其他」这一稳定事实。
- **[pz] L1013** [版本/API免责]: 待核实：以上 3.0 变更项依据源稿与社区信息整理，最终以 Isaac Lab 官方迁移指南为准。

## prac_humanoid_loco.tex  (18 条)
- **[rebuilt] L36** [版本/API免责]: main 分支
- **[rebuilt] L87** [版本/API免责]: 命名与 CLI 随仓库版本可能微调，以你装的 README 为准
- **[rebuilt] L244** [版本/API免责]: flag 名以你装的 README 为准
- **[pz] L545** [版本/API免责]: 待核实：以你所装 \texttt{unitree\_rl\_mjlab} 的 G1 \texttt{env\_cfgs.py} 实际覆盖值为准
- **[rebuilt] L579** [其他/待判]: 如 \texttt{body\_ang\_vel\_w} 等，\texttt{body\_inertia\_w} 不一定可用
- **[pz] L696** [其他/待判]: 待核实：H1 各关节力矩（如膝、髋、踝、臂）的官方数值需对 Unitree H1 硬件手册核对，本章不写死具体 Nm。
- **[rebuilt] L741** [版本/API免责]: 以 README 为准
- **[rebuilt] L747** [其他/待判]: 常被归为 2024 年人形 sim2real 工作；本章不断言其发表于某会议
- **[rebuilt] L759** [版本/API免责]: 具体字段可用性以你所装 RSL-RL 版本为准。
- **[pz] L881** [其他/待判]: 待核实：HOVER 脚本路径/参数名（\texttt{train\_teacher\_policy.py}、\texttt{--sim mujoco}）取自 Tutorial 描述，需与当前 \texttt{NVlabs/HOVER} 仓库核对。
- **[pz] L975** [版本/API免责]: 待核实：上表 wall-clock 时间（Tutorial 给“RTX 4090 约 12--23h teacher / 约 16min student”）依硬件与版本差异较大，本章只给量级；具体以 HOVER README 与你的硬件为准。
- **[rebuilt] L992** [其他/待判]: Tutorial 记为 2.0.0
- **[rebuilt] L1031** [版本/API免责]: 分组名以 HoST 论文（arXiv:2502.08378）为准：原文将 reward 分为 Task / Style / Regularization / Post-task 四组（each 一个独立 critic）。Tutorial 旧稿曾写作 Posture/Balance/Progress/Safety，经核查与论文不符，已更正；上表 terms 为按论文语义归并的示意，精确清单以仓库 \…
- **[rebuilt] L1052** [论文/数值]: Tutorial 源把该机制冠以 “L2C2 / Learning to Coordinate Critics” 之名，但联网核查未能证实此命名——arXiv:2202.07152 的 L2C2 实为 “Locally Lipschitz Continuous Constraint”（一种平滑正则），与多 critic 协调无关；故此处只描述机制本身，不沿用该名。
- **[rebuilt] L1073** [其他/待判]: 各阶段 force 比例与迭代区间为 Tutorial 示意，非 HoST 官方超参。
- **[rebuilt] L1227** [其他/待判]: 较新版 \texttt{data\_augmentation\_func}
- **[rebuilt] L1228** [其他/待判]: README 标定 Isaac Lab 2.0.0
- **[pz] L1261** [版本/API免责]: 待核实：上表逐关节 effort/stiffness 为 Tutorial 给出的典型仿真值，应以你所装 \texttt{unitree\_rl\_mjlab} 的 G1 MJCF \texttt{<actuator>} 实际值为准（用 \cref{sec:rlmc-hl-action} 的提取脚本读出）。

## prac_humanoid_wbc.tex  (12 条)
- **[pz] L48** [版本/API免责]: 待核实：与当前 MuJoCo 配套版本
- **[rebuilt] L88** [版本/API免责]: Tutorial 旧稿臆造的 \texttt{Mjlab-Spinkick-Unitree-G1} 等动作专名未在注册表出现；以 \texttt{list-envs} 或仓库 README 为准。
- **[rebuilt] L185** [版本/API免责]: 上表 DoF 数为各平台典型仿真配置；G1 基础 $23$-DoF（$12$ 腿 $+$ $10$ 臂 $+$ $1$ 腰），扩展版加腰 roll/pitch 与腕 pitch/yaw 可达 $29$，含手更高（官方 Ultimate 版达 $43$ DoF）。HumanoidBench（arXiv:2403.10506）的 H1 $+$ 双 Shadow Hand 配置：动作空间 $61$ 维（…
- **[rebuilt] L303** [其他/待判]: 需自行移植 mask 逻辑
- **[pz] L375** [版本/API免责]: 待核实：ExBody2 的正式发表 venue（Tutorial 记为 RSS'25 Workshop Spotlight，官方页未标注，以发表后为准）。
- **[rebuilt] L400** [版本/API免责]: 上表 $\sigma_i$、$T_{\mathrm{reset}}$、权重为 Tutorial 给出的工程经验值，非 ExBody2 论文原始超参；ExBody2 论文（arXiv:2412.13196）Section III 给出其精确实现，落地以论文/代码为准。
- **[rebuilt] L413** [性能量级]: 具体降幅（如目标组 error 降 $30$--$50\%$、其他组降 $20$--$40\%$）为 Tutorial 给出的量级示意，非 ExBody2 论文报告的精确数值。
- **[rebuilt] L508** [论文/数值]: 保留率为经验范围，随数据集与机器人而变。
- **[pz] L528** [其他/待判]: 待核实：官方项目页只标 CoRL 2024，未标 oral。
- **[rebuilt] L561** [版本/API免责]: 上表 exit\_condition（$0.15$\,m $+$ 接触力）、force\_clip $100$\,N、意外接触罚 $2\times$、stage bonus $5.0$ 均为 Tutorial 给出的工程起点，未与 WoCoCo 论文（arXiv:2406.06005）逐项核对；落地以你的任务/平台调参为准。
- **[rebuilt] L613** [版本/API免责]: 此为 Tutorial 给出的工程封装，非 WoCoCo 官方 API；落地按你框架的 reward 机制实现。
- **[rebuilt] L965** [性能量级]: 权重为 Tutorial 给出的工程起点，依任务/平台/框架而调；angular\_momentum 的权重量级与机理见 \cref{ch:rlmc-humanoidloco} 的 \cref{ins:rlmc-hl-esp}（它是 ESP 不是刹车，过大逼出极小步幅）。

## prac_imitation.tex  (4 条)
- **[rebuilt] L287** [版本/API免责]: 源稿给出的 346 受试者 / 11{,}451 条动作为更精确的计数，与官方「300+ / 11{,}000+」一致，但具体数值随版本可能微调。
- **[rebuilt] L391** [版本/API免责]: 命令的归属需分清两个仓库。上面的 \texttt{--input\_file/--input\_fps/--output\_name/--headless} 形式已对照 BeyondMimic 原始 Isaac Lab 复现仓库 HybridRobotics/whole\_body\_tracking 的 README 逐字核实（该仓库用 argparse，确有 \texttt{--headles…
- **[pz] L1467** [其他/待判]: 待核实
- **[rebuilt] L1467** [其他/待判]: 

## prac_large_scale.tex  (8 条)
- **[pz] L67** [其他/待判]: 待核实
- **[rebuilt] L67** [其他/待判]: 
- **[rebuilt] L1095** [其他/待判]: 
- **[rebuilt] L1208** [其他/待判]: 
- **[rebuilt] L1211** [其他/待判]: AGILE 的 PPO 默认配置（数值待官方实现核对）
- **[rebuilt] L1215** [其他/待判]: 
- **[rebuilt] L1451** [其他/待判]: 
- **[rebuilt] L1463** [其他/待判]: 

## prac_legged_manip.tex  (10 条)
- **[pz] L47** [其他/待判]: 待核实：该仓库的确切平台清单与许可随提交变化，使用前请核对仓库当前 README。
- **[pz] L103** [版本/API免责]: 待核实：以你所用版本源码的 reward/curriculum 字段为准
- **[pz] L183** [版本/API免责]: 待核实：ARX L5 Pro 工作空间半径约 45\,cm，以厂商规格为准
- **[rebuilt] L662** [版本/API免责]: RSL-RL/Isaac Lab 的经验归一化用的是 \texttt{EmpiricalNormalization} 模块（非 Gym/SB3 的 \texttt{VecNormalize} wrapper），其暴露字段随版本变化、部分已弃用——以当前 \texttt{isaaclab\_rl.rsl\_rl} cfg 为准。
- **[rebuilt] L964** [其他/待判]: 实测 hot-start 的 Phase 0 常在数百至约 1000 iteration 内收敛（vs 从零的数千 iteration），具体数字依平台/实现而异。
- **[rebuilt] L1039** [其他/待判]: 相比直接端到端训练（常需 $20000+$ iteration、$30\!\sim\!50\,$小时且收敛不稳）大幅节省；具体数字依平台而异。
- **[pz] L1099** [论文/数值]: 待核实：VBC 论文将低层描述为「RL 从零训 + ROA」，\emph{未}明述「从 locomotion 策略热启动」；本章 \cref{ins:rlmc-lm-hotstart} 把 hot-start 作为一条\emph{独立推荐}的通用工程加速技术讲（对扩维到带臂的 whole-body tracker 是标准且合理的做法），但不应把它当成 VBC 的论文事实。
- **[rebuilt] L1119** [性能量级]: 各 $\sigma$/权重/迭代数为典型量级，非 VBC 论文逐字数值
- **[rebuilt] L1339** [版本/API免责]: 上述数值多取自源稿与论文，平台相关项（质量、臂长、kp 区间）为典型量级，精确值以你所用模型/URDF 与论文为准。
- **[rebuilt] L1383** [版本/API免责]: 凡 task id 确切字符串、各框架字段名与默认值、平台质量/臂长/kp 等硬件相关量，均以你所装版本的 \texttt{--help}、官方文档与具体模型/URDF 为准；本章已对未逐字确认处加 \texttt{\textbackslash pz}/\texttt{\textbackslash rebuilt} 标注。

## prac_manager_arch.tex  (15 条)
- **[rebuilt] L44** [版本/API免责]: 本章给出的具体版本号为教学锚定值；落地时请以你机器上 \texttt{pip show} / 仓库 CHANGELOG 实测为准。
- **[rebuilt] L175** [性能量级]: 近期已有为腿足 locomotion 适配的高性能 SAC 实现（如 RSL-RL 的 SAC 扩展），表明「locomotion 必用 PPO」并非铁律；本书后续算法选型章会讨论。
- **[rebuilt] L215** [版本/API免责]: 上面 2.0 / 0.5 / 1.5 等倍率与 $[10^{-5},10^{-2}]$ 夹断区间为源稿给出的示意值，跨版本可能不同；务必对照你所用 \texttt{rsl\_rl} 版本的 \texttt{ppo.py} 核实。
- **[rebuilt] L241** [版本/API免责]: mjlab 实测（G1/Go1 velocity \texttt{rl\_cfg.py}）的网络配置类为 \texttt{RslRlModelCfg}（actor/critic 分开）、算法配置类为 \texttt{RslRlPpoAlgorithmCfg}、runner 为 \texttt{RslRlOnPolicyRunnerCfg}（mjlab 自带 runner 类 \texttt{Mjl…
- **[rebuilt] L326** [版本/API免责]: 这是据 mjlab 1.4.0 \texttt{manager\_based\_rl\_env.py} 整理的「18 步」教学划分；具体步数与拆分粒度随版本变，请以你机器上的源码为准——本表的价值在相对顺序，而非「恰好 18」这个数字。
- **[rebuilt] L414** [性能量级]: 源稿给出「吞吐提升 5--10\%」「滞后 0.005s」等具体数字，依硬件/任务/decimation 而变，应理解为量级而非定值。
- **[rebuilt] L459** [版本/API免责]: 上段为高层 MDP 对齐的简化示意。已据源码对齐的事实：Isaac Lab \emph{永远} auto-reset（无 \texttt{auto\_reset} 开关，那是 mjlab 的）、奖惩在 reset 前算、obs 在 reset 后取、返回 5 元组 \texttt{(obs, reward, terminated, time\_outs, extras)} 且 \texttt{te…
- **[rebuilt] L598** [性能量级]: 下表「工时/加速倍数」为源稿经验值，因人/项目而异，应理解为量级对比而非测得基准。
- **[rebuilt] L819** [其他/待判]: 本节用的函数名（\texttt{track\_lin\_vel\_xy\_exp}/\texttt{track\_ang\_vel\_z\_exp}/\texttt{foot\_slip}）取自 Isaac Lab 的 \texttt{mdp} 库，便于与理论部呼应；mjlab 的同功能函数\emph{命名不同}——其 \texttt{velocity/mdp} 里跟踪奖励叫 \texttt{tr…
- **[rebuilt] L1005** [性能量级]: 上面的 \texttt{uv run play/train} 与 \texttt{--agent zero} 等 CLI 为 mjlab 约定；Isaac Lab 对应 \texttt{python scripts/.../train.py --task ...}；UniLab 对应 \texttt{uv run train --algo ppo --task <slug> --sim mujoc…
- **[rebuilt] L1056** [版本/API免责]: 属性名以各框架对应版本为准；Isaac Lab 3.0 的数据返回类型与四元数约定有变（见下方陷阱）。
- **[rebuilt] L1084** [版本/API免责]: Isaac Lab 3.0 改为 xyzw（请按你所用版本与文档确认）
- **[rebuilt] L1088** [版本/API免责]: Isaac Lab 3.0 的 \texttt{data.*} 数据管线返回类型有变（源稿称返回 Warp/Proxy 数组，需显式转 torch）——确切类型与转换 API 以 3.0 正式文档为准。
- **[rebuilt] L1160** [版本/API免责]: NaN Guard 的开关名（源稿作 \texttt{--enable-nan-guard}）与回放命令以 mjlab 版本为准。
- **[rebuilt] L1306** [其他/待判]: 

## prac_manipulation.tex  (5 条)
- **[rebuilt] L581** [版本/API免责]: 注：Dexsuite 的 Lift/Reorient 环境与 ADR/PBT/dictionary-obs 支持为 Isaac Lab 2.3 引入，已据 NVIDIA 开发者博客与 Isaac Lab 文档核实\,\cite{nvidia2025isaaclab23}；具体 task ID 字符串以你所装版本的 \texttt{list-envs} 输出为准。
- **[pz] L681** [论文/数值]: 待核实：DexPBT 论文未明确说明 exploit 时如何处理 optimizer 状态（只描述复制网络权重 $\theta$ 与变异超参），下文「用方式 (1)」为常见工程做法的推断而非论文确证
- **[rebuilt] L683** [其他/待判]: Isaac Lab 文档列出 PBT 实际\emph{变异}的超参为 \texttt{learning\_rate}、\texttt{grad\_norm}、\texttt{entropy\_coef}、\texttt{critic\_coef}、\texttt{bounds\_loss\_coef}、\texttt{kl\_threshold}、\texttt{gamma}、\texttt{ta…
- **[pz] L1190** [其他/待判]: UniLab 无对应：当前 26 个注册任务全是本体感知（无 RGB/depth 观测任务），操作上只有 in-hand 旋转用物体位姿 state，无视觉操作变体。MotrixSim 有渲染后端，但用于回放可视化（如 demo teaser），不是训练时的相机观测。因此本章视觉操作的三框架对比在此退化为 mjlab + Isaac Lab 两家；UniLab 的 depth/RGB obs 与相…
- **[pz] L1372** [其他/待判]: 待核实

## prac_motion_imitation.tex  (5 条)
- **[rebuilt] L581** [版本/API免责]: 「约 30 行」是源稿对历史 Hydra 配置的估计；现行 dataclass 实验文件的确切行数请以你本地仓库为准。
- **[rebuilt] L626** [版本/API免责]: 某些 G1 通用 tracker 的官方预训练据称用了 24$\times$A100（需 sharded MotionLib），具体耗时以官方日志为准
- **[rebuilt] L642** [性能量级]: 24$\times$A100
- **[rebuilt] L642** [其他/待判]: 查日志
- **[rebuilt] L823** [版本/API免责]: PHC+ 据称在 AMASS 上达到 100\% 的 eval\_success\_rate（约 5 轮 mining、5 个 primitive）；该「100\%」是后续扩展版本的声明，确切复现以官方仓库/论文为准。

## prac_multimodal.tex  (1 条)
- **[pz] L871** [性能量级]: 待核实：源稿给 RobotMDAR 单 chunk 推理约 20ms、感知延迟通常 $<200$ms（含推理 + 过渡），此数值未联网确证，按来源量级理解。

## prac_obs_action.tex  (1 条)
- **[pz] L767** [论文/数值]: 待核实：HOVER 论文给出 $25$ 帧堆叠，但未明写控制频率；$50\,$Hz 系 H1 同源工作的常见取值

## prac_physics.tex  (1 条)
- **[pz] L746** [版本/API免责]: 待核实：3.0 Beta 的 warp-native 数据管线细节随版本变动，以 Release Notes 为准。

## prac_privileged.tex  (12 条)
- **[pz] L344** [版本/API免责]: 待核实：源稿给出 critic 总维 249、信息差 204（含 187 维 terrain\_heights）。具体维度强依赖 height\_scan 网格分辨率与机器人关节数，请以你本地任务实际打印为准，不要照搬数字。
- **[pz] L489** [版本/API免责]: 待核实：上面 \texttt{base\_ang\_vel}/\texttt{projected\_gravity}/\texttt{joint\_pos\_rel}/\texttt{joint\_vel\_rel}/\texttt{last\_action}/\texttt{generated\_commands}/\texttt{height\_scan} 均为 \texttt{isaacla…
- **[pz] L826** [论文/数值]: 待核实：TTT 在腿足在线适应上的具体代表工作与年份，源稿标注为 "Sun et al. 2024"，本书未独立核到对应论文，故正文不附引用键，建议补一篇确凿的 legged-TTT 文献。
- **[pz] L839** [其他/待判]: 待核实
- **[pz] L1044** [论文/数值]: 待核实：具体 iter 数与 GPU-小时（源稿给「base 约 10--15k iter/3090 约 8--10\,h、蒸馏约 5--10k iter/约 5--10\,h」）未在论文摘要核到，强依赖硬件与配置，落地前对照官方仓库 README。
- **[pz] L1097** [论文/数值]: 待核实：阈值 0.6 弧度与「与 waypoint 真值方向比较」的精确定义请对照 extreme-parkour 论文/代码确认；此处数值据源稿转述。
- **[pz] L1178** [性能量级]: 待核实：HOVER「支持 $>15$ 种控制模式」与 RTX 4090/1024 envs 下 teacher$\approx$0.84\,s/iter、student$\approx$0.097\,s/iter 的吞吐数据为据源稿转述，建议对照 HOVER 论文/仓库确认。
- **[pz] L1199** [论文/数值]: 待核实：源稿称 Miki 2022「1700m 零跌倒」，该论文摘要强调的是阿尔卑斯长程徒步等野外测试；具体「1700m 零跌倒」数字未在摘要核到，引用时建议改述为「长程野外零样本」或核对原文。
- **[pz] L1232** [版本/API免责]: 待核实：具体参数顺序随 \texttt{isaaclab\_rl} 小版本可能微调，落地前核对当前版本 \texttt{exporter.py}。
- **[pz] L1237** [版本/API免责]: 待核实：\#3008/\#3009 的合入状态随版本变化，部署前请查你所用版本是否已含 GRU 导出修复。
- **[pz] L1389** [版本/API免责]: 待核实：上表「+10--30\% sim 表现」「1.2/2/3--5$\times$ 训练时间」等为源稿给出的经验数量级，强依赖任务/实现，正文已弱化为「数量级参考」，引用时勿当成保证值。
- **[pz] L1441** [版本/API免责]: 待核实：\texttt{has\_state\_estimation}/\texttt{motion\_anchor\_pos\_b} 的精确命名以本地 mjlab 版本为准。

## prac_quad_loco.tex  (8 条)
- **[rebuilt] L49** [其他/待判]: 源稿曾把 unitree\_rl\_mjlab 写成支持 Go1，经核为 Go2 起步，本章已改正。
- **[rebuilt] L83** [版本/API免责]: Hydra 覆盖键在不同 UniLab 版本可能是 \texttt{algo.num\_envs} 或 \texttt{algo.num-envs}，以 \texttt{train --help} 与 \texttt{conf/} 实际字段为准。
- **[rebuilt] L87** [版本/API免责]: 本章用 \texttt{Mjlab-Velocity-Flat/Rough-Unitree-Go1} 作为占位拼写，以 \texttt{--help} 实际输出为准。
- **[rebuilt] L1123** [版本/API免责]: 源稿曾称该版本为“RSL-RL 4.0”——经核 PyPI \texttt{rsl-rl-lib} 无 4.0 这一对外版本号（Isaac 钉 3.0.1、mjlab 装 5.2.0），故不写“4.0”；模块抽象的确切字段名以你所装版本为准。
- **[rebuilt] L1404** [版本/API免责]: Hydra 覆盖键 \texttt{algo.num\_envs}/\texttt{algo.num-envs}、\texttt{training.no\_play}/\texttt{training.no-play} 的连字符写法随版本，以实际 \texttt{conf/} 字段为准。
- **[rebuilt] L1589** [版本/API免责]: 上述 Go1/Go2 的质量、腿长、关节范围为典型量级，精确值以具体 MJCF/USD 为准。
- **[rebuilt] L1766** [版本/API免责]: 上述具体数值多取自源稿与官方文档，少数（如 Go1 各关节 $k_p/k_d$ 区间、质量腿长）为典型量级，精确值以模型文件为准。
- **[rebuilt] L1814** [版本/API免责]: 凡 task id 确切字符串、各框架字段名与默认值，均以你所装版本的 \texttt{--help} 与官方文档为准；本章在文中已对未逐字确认处加注。

## prac_reward.tex  (5 条)
- **[pz] L234** [其他/待判]: 待核实：mjlab 各 mdp 子模块的确切相对路径
- **[rebuilt] L485** [版本/API免责]: 以实装仓库为准
- **[rebuilt] L692** [版本/API免责]: 以实装仓库为准，源稿给的 mjlab staged_position_reward 形式未在官方确认
- **[pz] L1279** [版本/API免责]: 待核实：版本号
- **[pz] L1286** [其他/待判]: 待核实

## prac_setup.tex  (3 条)
- **[rebuilt] L128** [版本/API免责]: 3.0 GA 的确切时间表以官方发布为准。
- **[rebuilt] L386** [版本/API免责]: 源教程多用 \texttt{Mjlab-Velocity-Flat-Unitree-Go2}；官方文档明确出现的是 \texttt{...-Unitree-G1}（Go1 为其四足参考任务）。Go2 任务名以你安装版本的 \texttt{list-envs} 为准。
- **[rebuilt] L478** [版本/API免责]: \texttt{isaaclab.\_\_version\_\_} 与 \texttt{from isaaclab.envs import ManagerBasedRLEnv} 的可用性随版本而变（3.0 模块前缀已从 \texttt{omni.isaac.lab.*} 改为 \texttt{isaaclab.*}）。

## prac_sim2real.tex  (3 条)
- **[rebuilt] L318** [其他/待判]: Isaac Lab 侧 \texttt{policy.onnx.data} 仅在模型超过 protobuf 2\,GB 上限、走 ONNX external-data 时才出现，常规策略不产出
- **[rebuilt] L431** [其他/待判]: 产物含 \texttt{exported/policy.onnx}、\texttt{params/deploy.yaml}（关节映射、scale 等）；\texttt{policy.onnx.data} 仅在用 ONNX external-data 时出现，并非必然产物。
- **[rebuilt] L437** [版本/API免责]: 动作/观测配置（字段名如 \texttt{step\_dt} / \texttt{actions} / \texttt{observations} 以目标版本导出脚本为准）

## prac_tennis_control.tex  (5 条)
- **[rebuilt] L146** [论文/数值]: WoCoCo 部署用 Butterworth 滤波这一细节出自其代码/部署实践，论文摘要未明述；具体阶数与截止频率见 \cref{sec:rlmc-strike-integration} 的标注。
- **[rebuilt] L820** [论文/数值]: 把 KungfuBot 的自适应机制直接套到二值击球任务上的可行性，是本书的工程推断，KungfuBot 原文针对动作跟踪；列入 claimsToVerify。
- **[pz] L1351** [其他/待判]: 待核实：约 2000\,Hz
- **[pz] L1352** [其他/待判]: 待核实：约 1000\,Hz
- **[rebuilt] L1502** [其他/待判]: Butterworth 阶数 2、截止 4\,Hz @ 50\,Hz 控制（约控制频率的 8\%），出自 WoCoCo 部署实践的常见配置；具体数值建议按真机电机带宽现场标定，列入 claimsToVerify。

## prac_tennis_env.tex  (26 条)
- **[rebuilt] L64** [版本/API免责]: 经联网核对，mjlab core 1.4.0 官方只内置 velocity tracking、motion tracking（运动模仿）与 manipulation 三类参考任务，并\emph{不}附带 tennis 任务。
- **[pz] L184** [性能量级]: 待核实：HITTER（Isaac Lab）与 Phybot（Isaac Gym）的底层仿真频率官方未逐字给出，表中仿真频率列以「—」留空。球速列为粗略量级：Phybot 官方报「出球速度上限约 $10$\,m/s、挥拍速度约 $5.3$\,m/s」（已据此校正，原 $19.1$ 系误植）；ETH 训练用入射球速 $-19$--$-13$\,m/s、挥拍达 $12.06$\,m/s，表中「$\le1…
- **[rebuilt] L184** [论文/数值]: LATENT 论文明确「仿真频率设为 $2000$\,Hz 以准确建模球-拍与球-地接触」。
- **[pz] L204** [论文/数值]: 待核实：LATENT 选 MuJoCo 而非 PhysX 是否\emph{明确}以「接触求解更稳」为由，论文未逐字断言，此处为工程经验推断。
- **[rebuilt] L204** [其他/待判]: PhysX 用 restitution coefficient 直接控制弹跳（即 COR），而 MuJoCo 经 \verb|solref| 间接控制。
- **[rebuilt] L359** [其他/待判]: LATENT 在 README 专门标注四元数顺序约定——跨框架协作时的必要文档。
- **[rebuilt] L402** [其他/待判]: MuJoCo 的 \texttt{friction} 是 \texttt{[slide, torsion, roll]} 三元组，\emph{实际生效维数由 condim 决定}：\texttt{condim=3} 只用第一个（slide）；torsion 需 \texttt{condim}$\ge4$、roll 需 \texttt{condim=6} 才被使用（见 MuJoCo XML refe…
- **[rebuilt] L414** [性能量级]: 课程示例球-地面用 direct format \texttt{solref=(-3500.0,-2.0)}、\texttt{solimp=(0.95,0.99,0.001,0.5,2.0)}，经调使 $2.54$\,m 落球弹跳接近真实网球量级。
- **[rebuilt] L494** [论文/数值]: LATENT 明确指出「球的 dynamics randomization」是 Sim2Real 成功的必要条件：去掉后真机成功率从正手 $90.9\%$ / 反手 $77.8\%$ 骤降到正手 $16.7\%$ / 反手 $25.0\%$。
- **[rebuilt] L683** [版本/API免责]: 真实网球 $C_L$ 在高转速会\emph{饱和}（趋向上限约 $0.3$--$0.4$），并非常数；课程用 $C_L=1.0$ 结合体积项近似，源码可配 \texttt{max\_lift\_coefficient} 限幅。
- **[rebuilt] L685** [其他/待判]: 课程示例的 BallAerodynamics 事件\emph{未}实现旋转衰减——角速度在飞行中不变。
- **[rebuilt] L725** [版本/API免责]: 经核 Isaac Lab v2.3.2 起，旧的 \texttt{set\_external\_force\_and\_torque()}（articulation / rigid body 上）已\emph{弃用}，改用新的\textbf{可组合 wrench 系统}——经 \texttt{permanent\_wrench\_composer.set\_forces\_and\_torques…
- **[rebuilt] L861** [其他/待判]: LATENT 用 $2000$\,Hz 仿真，确保每次球拍-球接触至少 $6$--$10$ 步；用 $500$\,Hz 则一次 $3$\,ms 接触只有 $1$--$2$ 步，接触力解算严重不足。
- **[rebuilt] L1033** [其他/待判]: MuJoCo 的 \texttt{margin}/\texttt{gap} 是接触检测阈值与 inactive contact 缓冲，并\emph{非}连续碰撞检测；MuJoCo 文档中的 CCD 指 Convex Collision Detection（GJK/EPA 凸碰撞检测管线），不是「连续碰撞检测」。
- **[rebuilt] L1051** [其他/待判]: LATENT 在 Unitree G1 上的硬件适配涉及三处工程改动：(1) 3D 打印球拍适配器，使拍柄轴与前臂成约 $15^\circ$（MJCF 中对应 \texttt{euler="0 -15 0"}）；(2) 末端关节连接器加固以承受挥拍惯性力与击球反力（仿真中对应 actuator 的 \texttt{forcerange}/\texttt{ctrlrange}，太小则策略学到「不敢用…
- **[rebuilt] L1053** [其他/待判]: 基于 mjlab 的人形 $+$ 外部物体（滑板）集成项目
- **[rebuilt] L1159** [其他/待判]: 这并不严格
- **[rebuilt] L1183** [其他/待判]: 清华/北大/Galbot 合作，目前最完整的人形网球系统，在 Unitree G1（$29$ DoF）上实现人-机多拍对打。
- **[rebuilt] L1183** [论文/数值]: 训练在 MuJoCo JAX、PPO、$50$\,Hz policy、$2000$\,Hz 仿真；真机成功率 $90.9\%$ 正手 / $77.8\%$ 反手，可与人类稳定多拍对打。
- **[rebuilt] L1188** [论文/数值]: 真机 $90.9\%$ 正手 / $77.8\%$ 反手成功率，可与人类多拍对打；消融显示去掉 ball dynamics randomization 真机成功率跌到正手 $16.7\%$ / 反手 $25.0\%$，去掉观测噪声反手成功率跌到 $0\%$（正手仍约 $50\%$）。
- **[rebuilt] L1193** [其他/待判]: 用「分层规划 $+$ 学习」在 Unitree G1（$29$ 关节）上实现乒乓球：model-based planner 由解析弹道模型预测击球点、优化器算球拍目标速度，再用 RL policy 做全身控制；$50$\,Hz、asymmetric actor-critic（critic 用 privileged 完美球状态，actor 只用可部署观测）。真机返球率 $92.3\%$（命中率 $9…
- **[rebuilt] L1193** [论文/数值]: 经核 HITTER 感知用 $9$ 个 OptiTrack 相机、动捕系统 $360$\,Hz、毫米级精度；底层训练仿真频率官方未逐字给出（仅说明用 Isaac Lab）。
- **[rebuilt] L1195** [其他/待判]: Yuntao Ma 等「Learning coordinated badminton skills for legged manipulators」，在 ANYmal-D 四足 $+$ 动态臂 $+$ 球拍（拍面约 $45^\circ$）上实现人机羽毛球；用\emph{统一端到端 RL}（不分 perception/planning/control），一个 policy 直接从观测输出全身动作。
- **[pz] L1197** [论文/数值]: 待核实：CyboRacket（arXiv:2603.14605，Unitree G1 上「onboard 视觉 $+$ 物理弹道预测 $+$ 预训练 WBC」的感知-动作框架）作为另一参考系统，其具体 benchmark 数字未在本章引用。
- **[rebuilt] L1197** [版本/API免责]: 「Humanoid Whole-Body Badminton via Multi-Stage RL」，自研 Phybot C1（$1.28$\,m、$30$\,kg、$21$ DoF），用三阶段 curriculum（footwork $\to$ swing $\to$ task）实现\emph{无需运动先验}的全身羽毛球；部署用 EKF 估计与预测羽毛球轨迹。仿真两机可维持 $21$ 拍 ral…
- **[pz] L1395** [其他/待判]: 待核实

## prac_tennis_perception.tex  (20 条)
- **[rebuilt] L64** [版本/API免责]: \cref{ch:rlmc-tennisenv} 已核：mjlab core 1.4.0 与 Isaac Lab 均\emph{不}附带官方网球任务；当前课程示例的 tennis task 没有相机 sensor，\texttt{actions} 字典为空（Phase 1）。
- **[pz] L230** [性能量级]: 待核实：LATENT 论文只述「光学动捕」未具名品牌、亦未给动捕频率/延迟；表中 LATENT 与 Phybot 的 $100+$\,Hz/$\sim5$\,ms 为同类系统的量级估计。HITTER 的 $9\times$OptiTrack/$360$\,Hz、ETH 的 $60$\,Hz 已逐字核实；表中 HITTER 的 $\sim3$\,ms 延迟系按 $360$\,Hz 单帧周期（$\ap…
- **[rebuilt] L232** [性能量级]: ETH 的系统是\emph{四足} ANYmal-D（非人形）配 $6$ 自由度 DynaArm 与 ZED X 立体相机，用 perception-aware 训练实现了机载感知部署，是目前唯一做到这点的球类系统；其对象羽毛球（被击后因强空气阻力迅速减速，飞行速度量级远低于网球发球），机载感知运行在 $60$\,Hz、状态估计 $400$\,Hz、控制 $100$\,Hz（Jetson AGX …
- **[rebuilt] L278** [性能量级]: 关于仿真时间的取法（已核 mjlab 源）：manager-based 框架\emph{无} \texttt{env.sim\_time} 现成属性，须由「每环境步计数 $\times$ 控制步长」算——mjlab/Isaac Lab 用 \texttt{episode\_length\_buf * step\_dt}（\texttt{step\_dt = sim.dt * decimation}…
- **[rebuilt] L325** [性能量级]: 这是\emph{教学选择}；PACE 官方实现的 predictor 更轻量，输入近期若干帧、输出目标击球点（$\mathbb R^3$ 量级）而非一整条多步状态轨迹\cite{tennis_pace2025}。
- **[rebuilt] L391** [其他/待判]: ETH ANYmal-D（四足）是目前唯一用机载立体相机的球类系统，核心创新是 perception-aware training——训练时注入随距离/运动变化的噪声模型让策略学会在噪声下工作\cite{tennis_eth2025badminton}。
- **[pz] L409** [论文/数值]: 待核实：该事件相机乒乓球工作的精确延迟数字与 arXiv 编号未逐字复核，仅作前沿方向举例。
- **[rebuilt] L435** [性能量级]: Isaac Lab 2.3.x 提供 \texttt{CameraCfg}（USD 相机）与高吞吐的 \texttt{TiledCameraCfg}（多环境瓦片渲染，强化学习首选），\texttt{data\_types} 支持 \texttt{rgb}、\texttt{distance\_to\_image\_plane}（深度）、\texttt{semantic\_segmentation} …
- **[rebuilt] L435** [其他/待判]: 当前 tennis task 三框架\emph{都}没有相机 sensor。
- **[rebuilt] L595** [其他/待判]: Isaac Lab 本身\emph{不}提供 EKF 实现
- **[pz] L678** [性能量级]: 待核实：上表为源稿给出的量级估计，未在本仓库实测复现；含 drag 优于纯重力是定性确定的（\cref{ch:rlmc-tennisenv} 已证阻力让球减速 $30$--$50\%$），但具体百分比依发球分布、$\Delta t$、$\sigma_a$ 而变，部署前应在自己的轨迹上实测。
- **[rebuilt] L760** [其他/待判]: Isaac Lab 用 \texttt{ContactSensorCfg} 读球-地接触
- **[pz] L791** [论文/数值]: 待核实：源稿称 gray-box 误差比 black-box 低 $40$--$60\%$，该具体百分比未在论文摘要逐字确认，方向性结论（gray-box 分布外更优）已确认。
- **[pz] L865** [性能量级]: 待核实：上述误差量级（physics-only $20$--$40$\,cm、physics$+$GRU $5$--$15$\,cm、GRU 残差改进 $30$--$50\%$）为源稿教学预期，未在本仓库实测；定性结论（physics$+$GRU 在 OOD 优于 pure GRU）由 gray-box 论文\cite{achterhold2023graybox}支撑。
- **[rebuilt] L867** [论文/数值]: PACE（arXiv:2509.21690）用了一个优雅的双通道设计，已 zero-shot 部署到真实 Booster T1 人形（23 关节），仿真里 hit rate $\ge 96\%$、success rate $\ge 92\%$\cite{tennis_pace2025}
- **[rebuilt] L943** [论文/数值]: LATENT（arXiv:2603.12686）的消融是球类 RL 中最有说服力的证据
- **[pz] L958** [论文/数值]: 待核实：LATENT 的逐项噪声/延迟数值（$2$\,cm/$0.5$\,m\,s$^{-1}$/$0$--$3$ 帧/$5\%$）系据论文文字描述重构，非官方逐字表格；成功率三档（$90.9\%$/$77.8\%$、$14$--$29\%$、$50\%$/$0\%$）已联网核实。
- **[rebuilt] L1017** [论文/数值]: HITTER 论文并未提出「先 timing 后 position」的优化\emph{顺序}，此处仅强调两者都关键、且时间误差代价更隐蔽。
- **[rebuilt] L1124** [其他/待判]: ETH ANYmal-D（四足）的创新不是蒸馏，而是在 RL 训练阶段就注入\emph{基于真实相机标定}的误差模型
- **[pz] L1298** [其他/待判]: 待核实

## prac_training_pipeline.tex  (7 条)
- **[rebuilt] L32** [其他/待判]: 
- **[rebuilt] L469** [其他/待判]: 
- **[rebuilt] L478** [版本/API免责]: 较新版本亦保存
- **[pz] L884** [版本/API免责]: 待核实：两框架对 RNN/CNN 的 ONNX 导出完备度（尤其 RNN hidden-state I/O）以本地版本为准。
- **[rebuilt] L1447** [版本/API免责]: 数量级量级参考，随硬件/实现波动；请以本地基准为准
- **[pz] L1514** [版本/API免责]: 待核实：CycloneDDS 的确切兼容版本号以 Unitree SDK2 当前文档为准。
- **[rebuilt] L1709** [其他/待判]: 

## prac_visuomotor.tex  (4 条)
- **[rebuilt] L70** [其他/待判]: 源稿把 MuJoCo Warp 渲染器称作 “rasterizer”，经核为 ray-tracing 后端，本章已改。
- **[pz] L959** [论文/数值]: 待核实：按论文，depth backbone 与 base policy 跑在 Jetson 上经 UDP 通信，depth 约 $10$\,Hz、base policy 约 $50$\,Hz（D435 输出约 $10\pm 2$\,Hz）
- **[pz] L1082** [其他/待判]: 待核实：torchvision 没有 \texttt{vit\_small\_patch14\_dinov2}+pretrained 这种写法；timm/HF 用 \texttt{vit\_small\_patch14\_dinov2.lvd142m} 之类模型名
- **[pz] L1260** [其他/待判]: 待核实

## prac_wheeled_bimanual.tex  (10 条)
- **[rebuilt] L51** [版本/API免责]: Go2W / Go2ArmManipLoco 的轮速接口与臂动作切分细节、以及 awesome-loco-manipulation 仓库的确切 URDF 清单，本章未逐字核对，使用前以 UniLab 当前源码与仓库内容为准。
- **[rebuilt] L346** [版本/API免责]: 方案 B 来自 Isaac Lab 社区讨论的工程经验，非官方 API 一部分；具体外力施加接口以你所装版本为准。
- **[rebuilt] L477** [版本/API免责]: 循环模型配置类的确切类名随 rsl-rl / Isaac Lab 版本变化，以你所装版本文档为准。
- **[rebuilt] L669** [版本/API免责]: 开启归一化的\emph{配置标志名}随版本变化：旧版用 \texttt{empirical\_normalization}，rsl-rl $\ge$ 4.0.0 改用 \texttt{obs\_normalization}（或策略级 \texttt{actor\_obs\_normalization}/\texttt{critic\_obs\_normalization}）；以你所装版本为准。
- **[rebuilt] L810** [版本/API免责]: 两框架 curriculum 触发条件的确切字段名以所装版本为准；本章给出的是通用结构。
- **[rebuilt] L1036** [论文/数值]: 源稿称其漂移任务是“文献中首个免在线微调的 sim2real 漂移策略”，论文摘要表述为“state-of-the-art zero-shot”，该“首个”超级表述本书未能逐字证实，按可证实口径降为“零样本 sim2real 漂移的代表性工作”。
- **[rebuilt] L1056** [性能量级]: 上表中 Wheeled Lab 的 IMU 噪声具体数值为源稿给出，论文正文未逐字核对，按数量级参考即可。
- **[rebuilt] L1220** [其他/待判]: 本章正文用占位 \texttt{<TASK\_ID>} 指代你注册的任务名，不假定任何固定字符串。
- **[rebuilt] L1266** [版本/API免责]: 此吞吐比例为源稿经验值，随硬件/版本波动，仅作量级参考。
- **[rebuilt] L1453** [版本/API免责]: 凡 task id 确切字符串、各框架字段名与默认值，均以你所装版本的 \texttt{--help} 与官方文档为准；本章在文中已对未逐字确认处加注。轮式任务为自建，task id 完全取决于你的注册调用。