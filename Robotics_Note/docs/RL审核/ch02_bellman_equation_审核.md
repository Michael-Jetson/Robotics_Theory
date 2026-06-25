# 第2章《状态值与贝尔曼方程》中文 OCR MD 保真度审核

## 概述

- 审核对象：`强化学习的数学原理_分章/02_第2章_状态值与贝尔曼方程.md`
- 真值参照：英文权威版 `Book-MathFoundationRL-en/3 - Chapter 2 ...pdf`（及 `/tmp/rlref_en/ch2.txt`）
- 结论：中文 MD 的**核心数学公式整体高保真**，未发现下标/上标/常数/符号层面的硬性 OCR 错误（如 $\frac{\gamma}{1-\gamma}$、矩阵 $P$、$(I-\gamma P_\pi)^{-1}$、Gershgorin 半径式、收敛证明 $\delta_{k+1}=\gamma^{k+1}P_\pi^{k+1}\delta_0$ 等均正确）。
- 主要问题集中在**漏识**（英文有、中文缺的整句/整段）与若干**版本差异**（中文 MD 似对应稍新/扩充版，含英文版没有的内容）。版本差异本身非 OCR 缺陷，但因任务以英文为真值，一并列出并标注。
- 公式编号体系两版不同（英文贝尔曼方程为 (2.7)，中文为 (2.9)；英文动作值矩阵式 (2.15)，中文 (2.18)），属版本差异，不逐条计为问题。

## 问题清单

| 位置[节/公式] | 类型 | 中文 MD 现状 | 英文正确形式 | 建议 |
|---|---|---|---|---|
| 2.4 第一项说明（式(2.7)/英(2.5)后） | 漏识 | 仅写到「类似地，R 也可能依赖于 (s,a)。」即结束 | 英文继续："We drop the dependence on s or (s, a) for the sake of simplicity in this book. Nevertheless, the conclusions are still valid in the presence of dependence."（为简便本书省略对 s 或 (s,a) 的依赖，但存在依赖时结论依旧成立） | 补全此两句，说明「省略依赖、结论仍成立」 |
| 2.8.2 末段（文献[5]前） | 漏识 | 「相比基于状态值的贝尔曼方程……策略仅被包含在 Π 中。更多性质可以参见文献[5]。」 | 英文在「policy is embedded in Π.」与「More details ... [5].」之间还有一句："It can be verified that (2.15) is also a contraction mapping and has a unique solution that can be iteratively solved."（可验证 (2.18)/(2.15) 也是压缩映射，有唯一解且可迭代求解） | 补回「压缩映射 / 唯一解 / 可迭代求解」一句 |
| 2.8.1 示例末尾 | 漏识 | 中文 2.8.1 在「第二，为什么我们要关心策略不会选择的动作？……找到每个状态下的最优动作。」后即结束本节 | 英文 2.8.1 末尾还有收尾计算："Finally, after computing the action values, we can also calculate the state value according to (2.13): $v_\pi(s_1)=0.5q_\pi(s_1,a_2)+0.5q_\pi(s_1,a_3)=0.5[0+\gamma v_\pi(s_3)]+0.5[-1+\gamma v_\pi(s_2)]$." | 补回「由动作值反算状态值」的收尾公式（演示式(2.16)的用法） |
| 2.4「等价形式」引语 | 公式不符/版本差异 | 「接下来我们介绍三种常见的等价形式。」并给出三式（其中第二式为「贝尔曼期望方程」） | 英文权威版仅："We next introduce **two** equivalent expressions."，且**无**单独的「Bellman expectation equation」小节（其第一、第二式对应中文第一、第三式） | 以英文为真值应为「两种」；若保留中文扩充的第三式（贝尔曼期望方程），需确认所据版本，至少改正「三种」与英文「两种」的不一致并标注来源 |
| 2.4 第二等价形式（贝尔曼期望方程） | 漏识(逆向)/版本差异 | 中文额外给出 $v_\pi(s)=\mathbb{E}[R_{t+1}+\gamma v_\pi(S_{t+1})\mid S_t=s]$ 及推导 | 英文权威版正文中**无**此独立等价形式 | 系中文版新增内容，数学正确；标注为版本差异，非英文真值 |
| 2.3 第三条说明（$v_\pi(s)$ 不依赖 t） | 内容不符/版本差异 | 「不论 t 选取什么值得到的结果都是相同的，这本质上是因为系统是平稳的，不会随着时间变化。」 | 英文："If the agent moves in the state space, t represents the current time step. The value of $v_\pi(s)$ is determined once the policy is given." | 含义等价（平稳性 vs「策略给定即确定」），属表述/版本差异，低严重度；如需严格对齐英文真值可改用英文措辞 |
| 2.1 关于 return₃ 的脚注 | 内容不符/版本差异 | 「return₃ 并没有严格遵守回报的定义：回报的定义只是针对一条轨迹，而 return₃ 是两条轨迹回报的平均值。」 | 英文："return3 does not strictly comply with the definition of returns because it is more like an expected value." | 两版解释不同但结论一致（return₃ 实为期望/状态值）；属版本差异，低严重度 |
| 图2.1（章在全书位置） | 漏识 | 中文仅以图片占位，正文未含图中文字 | 英文 OCR 含各章标签（Ch.3 Bellman Optimality、Ch.4 Value/Policy Iteration 等）。属图内文字，正常以图呈现 | 无需处理（图片形式合理），仅记录以备核图 |

## 已逐项核对、确认无误的关键点（抽样）

- 式 return₁/₂/₃ 的常数与 $\frac{\gamma}{1-\gamma}$、$-0.5+\frac{\gamma}{1-\gamma}$ 均正确；不等式 (2.1) $\text{return}_1>\text{return}_3>\text{return}_2$ 正确。
- 自举方程组 (2.3)、矩阵式 (2.4) 中 $P$ 的循环置换结构（次对角线为 1、左下角 1）正确。
- 状态值定义 $v_\pi(s)\doteq\mathbb{E}[G_t|S_t=s]$、$G_t$ 拆分、(2.6)/(2.7)/(2.8)/(2.9) 各求和指标 $a\in\mathcal A,\ r\in\mathcal R,\ s'\in\mathcal S$ 与马尔可夫性说明均正确。
- 例1/例2 数值：$\gamma=0.9$ 下 $v_\pi(s_4)=s_3=s_2=10$、例1 $v_\pi(s_1)=9$、例2 $v_\pi(s_1)=-0.5+9=8.5$ 全部正确。
- 矩阵-向量形式 (2.11)–(2.13)、$P_\pi\ge 0$、$P_\pi\mathbf 1=\mathbf 1$、图2.6 具体矩阵（行 [0,0.5,0.5,0] 等）正确。
- 解析解 $v_\pi=(I-\gamma P_\pi)^{-1}r_\pi$、Gershgorin 圆盘中心 $1-\gamma p_\pi(s_i|s_i)$、半径与级数 $(I-\gamma P_\pi)^{-1}=I+\gamma P_\pi+\gamma^2P_\pi^2+\cdots\ge I\ge 0$、性质三均正确。
- 迭代式 (2.14)、收敛 (2.15) 与方框2.1 证明（$\delta_{k+1}=\gamma P_\pi\delta_k=\cdots=\gamma^{k+1}P_\pi^{k+1}\delta_0\to0$）正确。
- 图2.7 两组状态值表（含 -6.6/-7.3/… 与 0.0/-9.0/0.5 等）与英文一致。
- 动作值定义、(2.16)/(2.17)、动作值贝尔曼方程及矩阵式 (2.18) 含 $\tilde r$、$P$、块对角 $\Pi$ 定义均正确。

## 问题计数

- 实质漏识（英文真值有、中文缺）：**3** 处（2.4 省略依赖说明、2.8.2 压缩映射句、2.8.1 反算状态值收尾）
- 与英文真值不符/版本差异：**4** 处（2.4「三种 vs 两种」+ 新增贝尔曼期望方程、2.3 第三条说明、2.1 return₃ 脚注）
- 合计标注问题：**7** 处（其中硬性 OCR 数学错误 0 处）
