# 第7章《时序差分方法》中文 OCR MD 保真度审核

## 概述

以英文权威版（赵世钰《Mathematical Foundations of Reinforcement Learning》第7章）为数学/内容真值，
对中文结构化 OCR MD `07_第7章_时序差分方法.md` 逐节比对。

总体结论：中文 MD 的结构、公式编号、绝大多数公式与英文版一致，OCR 质量较高。
发现的实质性问题主要集中在：(1) 一处公式下标缺失（Q-learning 的 max 作用域）；
(2) 少量内容/语义层面的漏译或误译；(3) 个别图题/参数处的合并差异。
其中只有 1 处属于会影响数学正确性的硬错误（max 下标），其余多为内容完整性问题。

注：英文 `ch7.txt` 由 pdftotext 提取，ε-greedy 公式中分子 ε 被提取丢失（显示为 `1 - |A(st)|(...)`），
这是英文提取噪声，**不计为中文问题**；中文 MD 的 ε-greedy 公式 `1-\frac{\epsilon}{|\mathcal{A}(s_t)|}(|\mathcal{A}(s_t)|-1)`
与 `\frac{\epsilon}{|\mathcal{A}(s_t)|}` 实为正确标准形式，已与原版 PDF 核对一致。

## 问题清单

| 位置[节/公式] | 类型 | 中文MD现状 | 英文正确形式 | 建议 |
|---|---|---|---|---|
| 式(7.18)，第7.4.1节（MD 第484行） | OCR错/公式不符 | TD 目标中写作 `r_{t+1}+\gamma \max_{a\in\mathcal{A}} q_t(s_{t+1},a)`，max 下标只有 `a\in\mathcal{A}`，丢失了状态参数 | PDF 原版为 `r_{t+1}+\gamma \max_{a\in\mathcal{A}(s_{t+1})} q_t(s_{t+1},a)`，max 作用域应为 `\mathcal{A}(s_{t+1})` | 将 `\max_{a\in\mathcal{A}}` 改为 `\max_{a\in\mathcal{A}(s_{t+1})}`，与方框7.5中 `\max_{a\in\mathcal{A}(s')}` 的写法保持一致 |
| 第7.4.4节，不同行为策略（MD 第622行） | 公式不符/误译 | "导致经验样本不合理" | 英文：the experience samples are **insufficient**（经验样本**不足**） | 将"不合理"改为"不足"，以符合原意（探索性弱 → 样本不足，而非不合理） |
| 第7.1.3节，定理7.1后关于 α 的讨论（MD 第170行） | 漏识（内容缺失） | 仅写"所以该条件实际上是要求有足够多的经验数据" | 英文明确给出实现方式：requires either the condition of **exploring starts** or an **exploratory policy** so that every state-action pair can possibly be visited many times | 建议补回"探索性出发(exploring starts)或探索性策略"这一具体条件的表述（英文原文要点） |
| 第7.4.2节，Sarsa On-policy 样本生成说明（MD 第543行） | 漏识（细节简化） | 直接说 "Sarsa 用这个经验数据来估计 q_{\pi_b}(s_t,a_t)…目标策略就是用来生成样本的策略" | 英文用 `π_b` 与 `π_T` 两符号严格区分行为/目标策略并论证 `π_T = π_b`（the policy denoted as π_T … π_T is the same as π_b） | 内容等价，建议保留 π_T/π_b 的符号区分以与英文一致；非硬错误 |
| 第7.4.2节，Q-learning Off-policy 样本说明（MD 第555-557行） | 漏识（句子截断/缺失） | 以"因此，(s_t,a_t)的最优动作值的估计不再涉及 π_b。"结束 | 英文还有后续两句：we can use **any π_b** to generate a_t at s_t；且 the **target policy π_T here is the greedy policy**（Algorithm 7.3），the behavior policy does not have to be the same as π_T | 建议补回"可用任意 π_b 生成 a_t""目标策略 π_T 为 greedy 策略（见算法7.3）"两点结论 |
| 表7.1，TD vs MC（MD 第166行，自举行） | 漏识（半句缺失） | MC 行："非自举：MC算法不是自举的，因为它可以直接估计状态值/动作值，而无需初始值" | 英文："…without **initial guesses**"（无需初始**猜测/估计**）——含义一致，但同段 TD 行英文有 "Continuing tasks **may not have terminal states**"（持续任务可能没有终止状态）这一句，中文 TD 持续任务格未含该补充说明 | 可在 TD"持续任务"格补一句"持续性任务可能没有终止状态"以完整对应英文 |
| 第7.2.2节正文（MD 第314行） | 漏识（一句缺失） | 描述任务后未提全局/局部最优的警示 | 英文：However, if we do not explore all the states, the final path may be **locally optimal rather than globally optimal** | 建议补回"若不探索所有状态，最终路径可能只是局部最优而非全局最优"一句 |
| 图7.2图题（MD 第338行） | 漏识（图题细节） | "右图显示了每个回合的回报和长度的变化过程"，未含奖励/学习率参数 | 英文图题含 `r_target=0, r_forbidden=r_boundary=-10, r_other=-1, α=0.1, ε=0.1` 及 "the blue cell" 目标格说明 | 图题参数在正文(第341行)已给出，非实质丢失；可选择性在图题补参数 |
| 第7.2.2节，回报说明（MD 第345行） | 漏识（定义句缺失） | "图7.2中的右上方子图展示了每个回合的回报逐渐变化的过程" | 英文额外定义：Here, the total reward is the **non-discounted sum** of all immediate rewards（回报为所有即时奖励的非折扣总和） | 建议补回"此处回报指所有即时奖励的非折扣总和"这一定义 |

## 复核确认无误的关键点（抽查）

- 式(7.1)(7.2)(7.3)(7.4)(7.5)：与英文逐项一致（含 `R_{t+1}+\gamma G_{t+1}`、`v_\pi(s_{t+1})` vs `v_t(s_{t+1})` 的区别说明）。
- 方框7.1 TD 推导、噪声观测分解 g + η：一致。
- 式(7.6) TD 目标/TD 误差标注、`\delta_t`、`\bar{v}_t`：一致。
- 方框7.2 收敛性证明（Δ_t、η_t、式(7.9)(7.10)(7.11)、第二/第三条件、参考[32]）：一致。
- 表7.1 TD/MC 对比四行主体（增量/回合制/自举/方差，含 `|\mathcal{A}|^L`、三随机变量 R_{t+1},S_{t+1},A_{t+1}）：一致。
- 定理7.1、7.2 条件 `\sum\alpha_t=\infty, \sum\alpha_t^2<\infty`：一致。
- 式(7.12) Sarsa、式(7.13) 动作值贝尔曼方程、方框7.3 推导：一致。
- 方框7.4 Expected Sarsa（含式(7.15)、期望 TD 目标、方差变量集合 {…}）：一致。
- 第7.3节 n-step Sarsa：G_t 分解、式(7.16)(7.17)、n=1/n=∞/一般 n 三情形、t+n 时刻改写式：一致。
- 方框7.5 证明(7.19)是 BOE：`\max_{a\in\mathcal{A}(s')}` 下标正确，与式(7.18)的缺失形成对照。
- 算法7.1/7.2/7.3 伪代码（含 ε-greedy 系数、greedy 目标策略 1/0）：与原版 PDF 一致（英文 txt 此处为提取噪声）。
- 式(7.20) 统一框架、表7.2（四算法 TD 目标 + 求解方程 BE/BOE）：一致。
- 第7.5/7.6/7.7（统一观点、总结、Q&A 全部7问）：内容一致；On/Off-policy 与 Online/Offline 论述一致。

## 问题计数

合计 **9** 处问题：
- 公式/OCR 硬错误（影响数学正确性）：1 处（式(7.18) max 下标缺失）
- 误译（影响语义）：1 处（"不合理" 应为 "不足"）
- 漏识/内容缺失（句、定义、细节）：7 处
