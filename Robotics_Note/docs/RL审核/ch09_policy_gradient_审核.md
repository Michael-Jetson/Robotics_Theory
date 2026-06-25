# 第9章《策略梯度方法》中文 OCR MD 保真度审核报告

## 概述

- 审核对象：`强化学习的数学原理_分章/09_第9章_策略梯度方法.md`（中文 OCR 结构化 MD）
- 真值参照：`/tmp/rlref_en/ch9.txt`（英文权威版 pdftotext 提取，已忽略页码/断行等噪声）+ 原版 PDF
- 结论：核心数学公式（目标函数定义、策略梯度定理、引理 9.1–9.3、定理 9.2–9.5、泊松方程、REINFORCE 算法及其分析）**整体转写准确**，未发现影响推导正确性的公式级 OCR 错误或常数/符号错误。
- 主要问题集中在**漏识若干英文句子/从句**（多为说明性、定性的补充语句），以及个别**表/标题信息缺失**与**轻微改写偏离原意**。这些不改变数学结论，但属于内容缺失，建议补回。
- 未发现下标/上标错误、字母误识（l/1、O/0、希腊字母）、错号或错位类的实质性 OCR 错误。

## 问题清单

| 位置（节/公式） | 类型 | 中文 MD 现状 | 英文正确形式 | 建议 |
|---|---|---|---|---|
| 9.3.1 节，式(324 行)旁，状态值性质 | 漏识 | 仅写"它们满足 $v_\pi(s)=\sum_{a}\pi(a\|s,\theta)q_\pi(s,a)$" | "It holds that $v_\pi(s)=\sum_a\pi q_\pi$ **and the state value satisfies the Bellman equation.**" | 补"且状态值满足贝尔曼方程"一句 |
| 9.3 节，定理 9.1 说明第一条（278 行） | 漏识 | 结尾止于"式(9.8)可能变成严格的等式或一个近似" | 原文末尾另有一句"**The distribution η also varies in different scenarios.**"（分布 η 在不同场景中也不同） | 补回该句 |
| 9.3 节，Softmax 段落（308 行） | 漏识 | 仅说明"策略必须随机且探索性"，随后接 Softmax 公式 | 9.12 式后另有："Since π>0 for all a, the policy is stochastic and hence exploratory. **The policy does not directly tell which action to take. Instead, the action should be generated according to the probability distribution of the policy.**" | 补"该策略并不直接给出应采取哪个动作，而应按策略的概率分布来生成动作" |
| 9.3 节，定理 9.5 后说明（678 行） | 漏识 | "……更为优美，这是因为式(9.28)是严格成立的。" | "more elegant in the sense that (9.28) is strictly valid **and S obeys the stationary distribution.**" | 补"且 S 服从平稳分布" |
| 9.4 节，"如何采样 A"（822 行） | 漏识 | "采样 A 的理想方式是按照 $\pi(a\|s_t,\theta_t)$ 采样得到 $a_t$。" | 其后另有一句"**Therefore, the policy gradient algorithm is on-policy.**"（因此策略梯度算法是同策略的） | 补回此句（on-policy 为重要性质） |
| 9.4 节，REINFORCE 第一条结论（790 行附近） | 漏识 | "如果 $\beta_t\ge0$，则在 $s_t$ 选择 $a_t$ 的概率会增大" | 该条下另有一句"**The greater βt is, the stronger the enhancement is.**"（βt 越大，增强越强） | 补回该句 |
| 9.6 节，第一个问答（845 行） | 漏识 | 答案止于"用随机梯度来近似真实梯度" | 英文答案末尾另有："**The most important theoretical result regarding this method is the policy gradient given in Theorem 9.1.**" | 补"该方法最重要的理论结果是定理 9.1 中的策略梯度" |
| 9.4 节，采样实践说明（824 行） | 公式不符/改写偏离 | "实际中往往不会严格按照上述理论采样 S 和 A，这主要是因为**实际中的样本可能是稀缺的，例如我们不太可能等到策略运行了很久并进入平稳态之后才使用其经验样本**" | "the ideal ways for sampling S and A are not strictly followed in practice **due to their low efficiency of sample usage. A more sample-efficient implementation of (9.32) is given in Algorithm 9.1.**" | 原因应为"采样效率低"，并应引出"算法 9.1 给出更高效实现"；中文改写为"样本稀缺/等待平稳态"偏离原意，建议校正 |
| 表 9.1 标题（17 行） | 漏识 | "表 9.1 用表格来表示策略。" | "A tabular representation of a policy. **There are nine states and five actions for each state.**" | 补"共有 9 个状态，每个状态 5 个动作" |
| 9.2 节，选择分布 d（88 行） | 漏识 | "如何选择概率分布 $d(s)$ 呢？有如下两种常见情况。" | "How to select the distribution d? **This is an important question.** There are two cases." | 可补"这是一个重要问题"（次要） |
| 式(9.31)说明（756/758 行） | OCR/用词 | "其中 $\alpha>0$ 是学习率。" | "α > 0 is a **constant** learning rate."（常数学习率） | 补"常数"以与原文一致（次要） |

## 说明

- 上表所有条目均为**说明性文字/标题信息的缺失或轻微改写**，不涉及公式、下标上标、常数或符号的错误，核心数学推导链条（9.1–9.33 全部公式、三套梯度定理、泊松方程定理 9.4 及引理 9.3 的纠错性论述）经逐式比对**与英文权威版一致**。
- 中文 MD 文件末尾（867 行）混入了一段与本章内容无关的英文 OCR 规则自述文本（"The Ground Truth image displays a single, solid horizontal line…"），属 OCR 流水线噪声，非正文，建议删除。

## 问题计数

共 **11** 项问题：漏识 8 项、改写偏离原意 1 项、用词/标题信息缺失 2 项（其中表 9.1 标题计入漏识、9.31 用词单列）。按主要类型计：漏识 9、改写/用词偏离 2。核心公式错误 0 项。
