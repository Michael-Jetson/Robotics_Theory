# 第1章《基本概念》中文 OCR MD 保真度审核

## 概述

审核对象: `强化学习的数学原理_分章/01_第1章_基本概念.md`
真值参考: 英文权威版 `ch1.txt` / `3 - Chapter 1 Basic Concepts.pdf`,辅以中文源页图 `/tmp/rlsrc/ch1/`。

**总体结论**: 该 MD 对**中文纸质版**的 OCR 转写质量很高 —— 9 节结构完整,所有公式(状态转移概率、策略概率、回报、折扣回报、马尔可夫性质式(1.4)、Q&A 中的 `p(r|s,a)=∑ p(r|s,a,s')p(s'|s,a)`)以及三张表格(表1.1/1.2/1.3)数值均与英文一致,**未发现真正的数学错误或下标误识**。

需要注意的是:**中文纸质版本身**相对英文版做了若干**翻译层删改/增补**(英文有而中译没有、或中译多出内容)。这类不是 OCR 缺陷,而是版本差异;MD 忠实复制了中文版。下表对其做"版本差异"标注,供整书统一时参考。报告末尾的"问题计数"仅统计**对中文版的 OCR 缺陷**,版本差异单列。

另外:MD 文件**末尾(第320行)残留一段英文 OCR 流水线的内部说明**("The Ground Truth image displays a single, solid horizontal line ..."),与正文无关,属噪声残留,应删除。

---

## 一、OCR 缺陷(相对中文纸质版的转写问题)

| 位置[节/公式] | 类型 | 中文MD现状 | 应有形式 | 建议 |
|---|---|---|---|---|
| 文件末尾 第320行 | OCR错(流水线噪声残留) | 残留英文段 "The Ground Truth image displays a single, solid horizontal line ... inconsistent with the Ground Truth." | 中文版无此内容,系 OCR 工具自审日志误入 | 删除整段 |
| §1.6 式(1.3) 第240行 | OCR错(空格/排版) | `0 + \gamma 0 + \gamma^ {2} 0 + \gamma^ {3} 1 + \gamma^ {4} 1 + \gamma^ {5} 1` 上标处多空格 `\gamma^ {2}` | `\gamma^{2}` 等(数学正确,仅 LaTeX 空格不规范) | 数值无误,清理 `^ {` 多余空格即可 |
| §1.6 第246行 | OCR错(空格/排版) | `\gamma^ {3} \frac {1}{1 - \gamma}`、`discounted   return` 多空格 | `\gamma^{3}\frac{1}{1-\gamma}` | 数值无误,清理空格 |
| §1.6 第234行 | OCR错(排版) | `\mathrm{return} = 0 + 0 + 0 + 1 + 1 + 1 + \dots = \infty` 中 `\mathrm{discountedreturn}` 等连写 | 数值/含义正确,仅 `\mathrm{}` 内单词连写 | 可加空格,不影响保真 |
| §1.5 第153/155行 | OCR错(符号不一致) | 列表项符号混用: 前两条无符号、后两条用 `◇` | 中文版四条均为同一项目符号 | 统一为 `-` 或 `◇`,不影响内容 |

> 说明: 以上 §1.6 各条均为 LaTeX 空格/连写的排版瑕疵,**公式数值与英文真值完全一致**(`0+γ0+γ²0+γ³1+γ⁴1+γ⁵1`,以及 `γ³·1/(1-γ)`),不构成数学错误。

---

## 二、版本差异(英文有而中文版删/简,或中文版增补;非 OCR 缺陷)

| 位置[节] | 类型 | 中文MD现状 | 英文权威版 | 建议 |
|---|---|---|---|---|
| §1.6 吸收状态 第255行 | 漏识(中译简化) | `\mathcal{A}(s_9) = \{a_5\}`,仅给一种写法 | `A(s9) = {a5}` **或** `A(s9) = {a1,...,a5} with p(s9|s9,ai)=1 for all i=1,...,5` | 英文给了第二种等价写法;经核中文源页 p033 确实只有第一种,属中译删减。整书统一时可补注 |
| §1.7 第293行 | 漏识(中译删整句) | 仅"一旦策略确定,MDP 退化为 MP……本书主要考虑有限 MDP" | 英文多出: "In the literature on stochastic processes, a Markov process is also called a **Markov chain** if it is a discrete-time process and the number of states is finite or countable [1]. In this book, the terms 'Markov process' and 'Markov chain' are used interchangeably when the context is clear." | 中文版**整段省略了"马尔可夫链(Markov chain)"的定义与互换约定**。建议补译,因后续章节会用到"马尔可夫链"一词 |
| §1.7 智能体-环境 第302行 | 改写(中译重写) | 智能体=感知者(眼)/决策者(脑)/执行者(操作机构),三点式 | 英文: agent = decision-maker that can sense state, maintain policies, execute actions; actuator 执行; **by using interpreters** 解释新状态与奖励; **a closed loop can be formed**(闭环) | 内容大体对应但中译省去了"interpreters/闭环(closed loop)"表述;属翻译改写,语义不缺核心 |
| §1.6 回报构成 第191行 | 改写/位置差异 | 即时奖励+未来奖励段落紧接式(1.1)之后;结尾"因此总奖励是1" | 英文该段在式(1.2)之后(第672–677行),且结尾为 "...should be determined by the return (i.e., the total reward) rather than the immediate reward **to avoid short-sighted decisions**." | 中译把该段提前并改写,省略"避免短视决策"一句;不影响数学 |
| §1.6 回合定义 第251行 | 增补(中译加注) | 增加"Episode 多种翻译(回合/情节/集/轮)…与 epoch 区分""回合≈有限长轨迹"等译者注 | 英文无这些译注,且英文有"if everything is deterministic, we always obtain the same episode"一句 | 中译增补译注属正常;英文"确定性则得同一回合"一句中译未直接对应,可不补 |
| §1.9 Q&A 第314行 | 改写(中译换措辞) | "由于 r_target=-1 相比 r_forbidden=-3 惩罚更少…已经算鼓励了…更多信息可参见**第3.5节中的定理3.6**" | 英文: "That is because **optimal policies are invariant to affine transformations of the rewards**. Details will be given in Chapter 3.5."(未给定理号) | 中文源页 p035 确认其本就如此(给出定理3.6、用直观解释替代"仿射变换不变性"陈述);属中译版差异,MD 转写正确 |
| 图1.1 第3–7行 | 信息量差异(图为位图) | 仅"图1.1 本章在全书中的位置" | 英文图含各章标题文字(Ch2 Bellman Eq, Ch3 BOE, …, Ch10 Actor-Critic 等) | 图内文字属图片,OCR 未抽取正常;无需处理 |

---

## 问题计数

- **OCR 缺陷(相对中文版,需修正)**: 5 处
  - 其中实质需删除 1 处(末尾英文噪声段);排版/空格类 3 处;符号统一 1 处。
  - **数学错误: 0 处**(所有公式、表格数值均正确)。
- **版本差异(英文 vs 中文版,非 MD 缺陷,供整书统一参考)**: 7 处
  - 其中建议补译的实质内容缺失 2 处(§1.7 马尔可夫链定义段;§1.6 吸收状态第二种等价写法)。

**合计问题条目: 12**(OCR 缺陷 5 + 版本差异 7)。
