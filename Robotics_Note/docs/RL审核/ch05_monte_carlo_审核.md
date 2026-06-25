# 第5章《蒙特卡罗方法》中文 OCR MD 保真度审核

## 概述

- 审核对象：`强化学习的数学原理_分章/05_第5章_蒙特卡罗方法.md`（中文结构化 OCR）
- 真值基准：英文权威版 `Mathematical Foundations of Reinforcement Learning` 第5章（`/tmp/rlref_en/ch5.txt` + 原版 PDF）
- 审核原则：以英文为数学/内容真值；只比实质内容与数学，不比中文行文优劣；忽略英文 pdftotext 提取噪声。

**总体结论**：本章 OCR 质量较高，核心公式（大数定律、动作值定义、(5.1)/(5.2)、MC Basic/ES/ε-Greedy 三套伪代码、ε-Greedy 概率公式、(5.4)/(5.5) 及其解）均正确还原，未发现严重数学错误。问题集中在少量**内容漏译/漏识**（整句或半句意思缺失）以及个别**符号/不等号缺失**，无影响理解的硬伤。

## 问题清单

| 位置（节/公式） | 类型 | 中文 MD 现状 | 英文正确形式 | 建议 |
|---|---|---|---|---|
| 5.1 §（式 E[X]=Σp(x)x 后，MD 行22~28 之间） | 漏识 | 缺一条脚注 | 英文脚注：“In this book, we use the terms expected value, mean, and average interchangeably.”（expected value / mean / average 三词在本书互换使用） | 补译该脚注，便于后文“期望值/均值/平均值”混用时不致疑惑 |
| 5.2.3 一个简单示例（MD 行188） | 漏识/公式不符 | $q_{\pi_0}(s_1,a_2)=q_{\pi_0}(s_1,a_3)=\dfrac{\gamma^3}{1-\gamma}$ ，随后才说“相比其他动作值是最大值” | 英文为 $q_{\pi_0}(s_1,a_2)=q_{\pi_0}(s_1,a_3)=\dfrac{\gamma^3}{1-\gamma}\,\mathbf{>0}$ are the maximum values | 公式末补 `>0`（英文显式标出该量为正，是“最大值”论断的依据） |
| 5.2.3 一个简单示例结尾（MD 行197） | 漏识 | “很明显，在 $s_1$ 选择 $a_2$ 或 $a_3$ 是最优策略。因此，对这个简单例子，我们仅使用一次迭代就可以成功得到最优策略。更复杂的场景则需要更多次的迭代。” | 英文多一句关键说明：“In this simple example, the initial policy is already optimal for all the states except $s_1$ and $s_3$. Therefore, the policy can become optimal after merely a single iteration.”（之所以一次迭代即收敛，是因为初始策略除 $s_1,s_3$ 外对所有状态已最优） | 补译“初始策略除 $s_1,s_3$ 外已对其他所有状态最优，故仅需一次迭代”这一因果解释 |
| 5.3.3 算法描述（MD 行332、5.3 节末） | 漏识 | “……这个条件在许多应用中很难满足，因为我们难以确保有足够多的回合从每一个状态-动作出发。” | 英文：“this condition is difficult to meet in many applications, **especially those involving physical interactions with environments**.”（尤其是涉及与环境物理交互的应用） | 补译“尤其在涉及与真实环境物理交互的场景”这一限定 |
| 5.5 探索与利用（MD 行459~461，ε-Greedy 平衡探索利用段） | 漏识 | 该段仅说明 ε-Greedy 如何平衡探索/利用，未提及其在其他算法中的应用 | 英文有一句：“ε-greedy policies are used not only in MC-based reinforcement learning but also in other reinforcement learning algorithms such as temporal-difference learning as introduced in Chapter 7.”（ε-Greedy 不仅用于 MC，也用于第7章时序差分等其他算法） | 补译该句（点明 ε-Greedy 的跨章节适用性，呼应第7章 TD） |
| 5.5 §「ε-Greedy 策略的最优性」（MD 行471，对应图5.6） | 漏识 | “从(a)~(d)图可以看出，随着 $\epsilon$ 的增加，这些 $\epsilon$-Greedy 策略的状态值不断下降……因此收到的奖励变小了。” | 英文额外指出：“Notably, the value of the target state becomes the smallest when $\epsilon$ is as large as 0.5. This is because, when $\epsilon$ is large, the agent starting from the target area may enter the surrounding forbidden areas and hence receive negative rewards with a higher probability.”（当 ε=0.5 时目标状态的值反而最小，因大 ε 下从目标出发易进入周围禁区而得负奖励） | 补译“ε=0.5 时目标状态值最小及其原因”这一观察（与后文“目标状态最优策略变为逃离”相呼应） |
| 5.4 引言（MD 行354，软策略段） | 漏识（轻微） | “给定一个软策略，即使只有一个回合，只要这个回合足够长，它就会多次访问每个状态-动作。” | 英文此处带图引用：“…can visit every state-action pair many times (see the examples in Figure 5.8).” | 可补“（见图5.8示例）”的图引用 |
| 算法5.2 / 算法5.3（MD 行338、412 等） | OCR 错（轻微/命名一致性） | 计数变量写作 `Number(s,a)`，回报累加写作 `Return(s,a)` | 英文伪代码为 `Num(s,a)` 与 `Returns(s,a)`（复数 Returns） | 数学含义无误，属变量名本地化差异；若追求与原版一致可改为 `Returns/Num`，否则可忽略 |
| 图5.3（MD 行147 表格） | OCR 错（轻微，仅图示） | 表格第三行第三格识别为 `Sg` | 应为目标状态 $s_9$（英文图5.3 为 $s_9$） | 将 `Sg` 修正为 $s_9$（OCR 把 9 误识为 g） |

## 备注（非问题项，供参考）

- (5.4) 用 $\Pi$、(5.5) 用 $\Pi_\epsilon$，中文 MD（行383/397）区分正确，无误。
- ε-Greedy 概率公式、归一化不等式 $1-\tfrac{\epsilon}{|\mathcal A(s)|}(|\mathcal A(s)|-1)=1-\epsilon+\tfrac{\epsilon}{|\mathcal A(s)|}\ge\tfrac{\epsilon}{|\mathcal A(s)|}$（行367）与英文一致。
- 图5.4(行215)、图5.6/5.7 各 ε 下状态值表与英文数值逐格核对一致（含 ε=0 表中 10.0、ε=0.5 表中负值等）。
- 5.2.1 节中文对算法的额外铺垫性解说（行79、行91 等）系译者补充，不属漏译缺陷。
- 5.5 节首段（行457）及图5.6 讨论段中文为重组/凝练表达，意思与英文等价，未单列为问题。

## 问题计数

**共 9 处问题**：漏识 6 处（其中 1 处含公式不符）、OCR 错 2 处（图示 `Sg`→$s_9$、变量命名）、轻微图引用漏识 1 处。其中实质性内容漏译 5 处（脚注、`>0`、初始策略已最优说明、TD 适用性句、目标状态值最小观察、物理交互限定），建议优先补译。
