# 复核报告：新部「学习时代的 SLAM 与空间 AI」（吸收 SLAM Handbook ch13/15/16/17/19）

对抗式只读审查。审查对象：`parts/P5_frontier/` 五章（learning / dynamic / semantic / openworld / epilogue）+ part.tex。
四条红线：① 浑然一体无接缝 ② 硬核数学正确（能算就算）③ 独立性（禁 punt/ventriloquize/单书叙述依赖）④ 无重复/记号一致。

**总体结论：质量很高，「浑然一体」基本达成。** 五章读起来是同一作者沿「学习→动态→语义→开放世界→结语」的连贯演进，全部扣住因子图母方程 `eq:intro-nls`、右扰动、log-odds 三条缝合线，过渡自然。所有被审的跨章「同构/推广」论断（可微分 BA≅重投影 BA、动态因子图退化为静态、多类 log-odds⊃二值、信息瓶颈接本书互信息、ARAP≅Procrustes）经独立验算/核对**成立**；全部外部 `\cref` 目标存在且内容相符。

但有 **1 个数学 blocker（右扰动雅可比写错，恰是本章卖点）** 与若干 major/minor。下面按严重度列出，每条给 `文件:行号` + 问题 + 修复建议。

---

## BLOCKER（必须修：数学错误，且踩中本章核心卖点）

### B1. semantic：物体位姿的右扰动雅可比写错（用了左扰动的式子）
- **位置**：`metric_semantic_slam.tex:287`（推导框「物体因子对相机位姿与物体位姿的右扰动雅可比」第 3 步「位姿段（物体）」）
- **问题**：该步写
  $$\delta\mathbf{Q}_\ell^{*}=(\delta\boldsymbol\phi)^\wedge\mathbf{Q}_\ell^{*}+\mathbf{Q}_\ell^{*}(\delta\boldsymbol\phi)^{\wedge\top}.$$
  但 `eq:semantic-quadric-Ql` 是 $\mathbf{Q}_\ell^{*}\propto\mathbf{q}\,\mathbf{S}_\lambda\mathbf{Q}_s^{*}\mathbf{S}_\lambda^\top\mathbf{q}^\top=\mathbf{q}\,X\,\mathbf{q}^\top$（$X=\mathbf{S}_\lambda\mathbf{Q}_s^*\mathbf{S}_\lambda^\top$ 固定）。在**本书声明的右扰动** $\mathbf{q}\leftarrow\mathbf{q}\,\mathrm{Exp}(\delta\boldsymbol\phi)$ 下，$\mathrm{Exp}(\delta\boldsymbol\phi)$ 夹在 $\mathbf{q}$ 与 $X$ **之间**，故 hat 须被 $\mathbf{q}$ **共轭**：
  $$\delta\mathbf{Q}_\ell^{*}=\mathbf{q}(\delta\boldsymbol\phi)^\wedge X\,\mathbf{q}^\top+\mathbf{q}X(\delta\boldsymbol\phi)^{\wedge\top}\mathbf{q}^\top=\big(\mathbf{q}(\delta\boldsymbol\phi)^\wedge\mathbf{q}^{-1}\big)\mathbf{Q}_\ell^{*}+\mathbf{Q}_\ell^{*}\big(\mathbf{q}(\delta\boldsymbol\phi)^\wedge\mathbf{q}^{-1}\big)^\top.$$
  书中漏掉了 $\mathbf{q}(\cdot)\mathbf{q}^{-1}$ 这层共轭。**数值验证**：书中式与真扰动残差约 9.4（量级完全不符）；书中式恰好等于**左扰动** $\mathbf{q}\leftarrow\mathrm{Exp}(\delta\boldsymbol\phi)\mathbf{q}$ 的结果（残差 ~1e-6）。即此步**实际写的是左扰动**，与章首 note(`:94`)、本推导框首句(`:283`)、习题(`:762`)反复声明的右扰动**自相矛盾**。
- **为何是 blocker**：本章把「用本书右扰动对 $\mathbf{T},\mathbf{q}$ 求导」当作相对 Handbook 的核心增量（`:289`「关键是两处扰动都右乘——这正是本书相对 Handbook 原文所做的统一」）。恰恰这个卖点的物体侧算错了。
- **修复建议**：把 `:287` 改为带共轭的形式（上式）；或显式引入物体位姿伴随 $\mathrm{Ad}_{\mathbf{q}}$，写成 $\delta\mathbf{Q}_\ell^*=\boldsymbol\zeta^\wedge\mathbf{Q}_\ell^*+\mathbf{Q}_\ell^*\boldsymbol\zeta^{\wedge\top}$、$\boldsymbol\zeta=\mathrm{Ad}_{\mathbf{q}}\,\delta\boldsymbol\phi$。注意相机侧(`:286`)结论**正确**（见 N1 仅是中间话术小瑕）——因为那里扰动 $\mathrm{Exp}(-\delta\boldsymbol\xi)$ 落在 $M=\mathbf{T}^{-1}\mathbf{Q}^*\mathbf{T}^{-\top}$ 的**外侧**，无需共轭。错因正是把相机侧「同形」照抄到物体侧而忽略了 $\mathbf{q}$ 在内侧。

---

## MAJOR

### M1. dynamic：`eq:dyn-couple` 的去齐次算子 `h^{-1}` 用法不自洽（除零）
- **位置**：`dynamic_deformable_slam.tex:245-250`（`eq:dyn-couple`）+ 算子定义 `:100`/`:250`
- **问题**：残差 $\mathbf{e}_{\mathrm{couple}}=h^{-1}\!\big((\mathbf{T}^k_{w,i+1}-\mathbf{T}^k_{wi}\Delta\mathbf{T}^k)\,\tilde{\mathbf{p}}^k_j\big)$，其中 `h^{-1}:(x,y,z,λ)↦(x/λ,y/λ,z/λ)`。两个 $\mathrm{SE}(3)$ 矩阵末行都是 $[0\,0\,0\,1]$，其**差**末行为 $[0\,0\,0\,0]$，故 $(\,\cdot\,)\tilde{\mathbf p}$ 的第 4 分量 $\lambda\equiv 0$——`h^{-1}` 要除以 0，**字面无定义**。意图显然是「取差的欧氏（前 3）分量」（两位姿一致时残差为 0，这是对的），但用定义为「除以 λ」的 `h^{-1}` 写不通。源于 Handbook 原式照搬。
- **修复建议**：此处不应用 `h^{-1}`，直接取前三行：$\mathbf{e}_{\mathrm{couple}}=\big[(\mathbf{T}^k_{w,i+1}-\mathbf{T}^k_{wi}\Delta\mathbf{T}^k)\tilde{\mathbf p}^k_j\big]_{1:3}$（或定义一个「取前三维」算子，与逐点先各自去齐次再相减区分清楚）。

### M2. dynamic：两处「Handbook 提出/给出」属单书叙述依赖（独立性红线 ③）
- **位置**：`dynamic_deformable_slam.tex:128`「Handbook 提出一套三轴刻画框架\cite{}」；`:145`「Handbook 给出一个更本质的定义\cite{}」
- **问题**：以「Handbook」作主语「提出/给出」内容，属 independence-standard 第 3 类 narration_dependence（即便带 `\cite`、无强调动词）。本部其余四章已把同类内容处理成本书直述（如 semantic `:128` 表注「据…，本书重组」、`:145`「本书把它形式化如下」），唯 dynamic 这两处露出源书作叙述载体。程度较轻（紧接着就「我们沿用」「本书把它形式化」夺回所有权）。
- **修复建议**：删主语，改本书直述 + `\cite`，例：「动态效应的复杂度随所考虑效应增多指数上升；本书采用一套*三轴*刻画框架\cite{schmid2026dynamic} 作组织骨架」；「短期/长期之分并非按绝对时长，更本质的判据是变化率与观测率之比\cite{schmid2026dynamic}，本书形式化如下」。

---

## MINOR

### N1. semantic：相机侧右扰动的中间话术指向了错误的量（结论对）
- **位置**：`metric_semantic_slam.tex:286`
- **问题**：用「注意 $\mathbf{T}_i^{-1}(\mathbf{T}_i\mathrm{Exp}(\delta\boldsymbol\xi))=\mathrm{Exp}(\delta\boldsymbol\xi)$」来引出 $\delta M$。但进入 $M=\mathbf{T}^{-1}\mathbf{Q}^*\mathbf{T}^{-\top}$ 的是 $(\mathbf{T}\mathrm{Exp}(\delta\boldsymbol\xi))^{-1}=\mathrm{Exp}(-\delta\boldsymbol\xi)\mathbf{T}^{-1}$，即 $\mathrm{Exp}(-\delta\boldsymbol\xi)$ 而非 $+$。所引恒等式虽真，却**不是**产生那个负号的量。最终式 $\delta M=-(\delta\boldsymbol\xi)^\wedge M-M(\delta\boldsymbol\xi)^{\wedge\top}$ **符号正确**（已数值验证 ~3e-5）。仅中间一句误导。
- **修复建议**：把该句换成「$(\mathbf{T}\,\mathrm{Exp}(\delta\boldsymbol\xi))^{-1}=\mathrm{Exp}(-\delta\boldsymbol\xi)\mathbf{T}^{-1}$，故 $M\mapsto\mathrm{Exp}(-\delta\boldsymbol\xi)M\,\mathrm{Exp}(-\delta\boldsymbol\xi)^\top$，一阶展开得…」。

### N2. semantic：多类 log-odds 维度「$\mathbb{R}^{K+1}$」含一冗余分量（表述可更准）
- **位置**：`metric_semantic_slam.tex:440`（`eq:semantic-logodds-vec`）
- **问题**：定义式 $\mathbf{h}=[\log\frac{p_1}{p_0}\cdots\log\frac{p_K}{p_0}]^\top$ 只有 $K$ 个分量，但正文与脚注约定首元 $\log\frac{p_0}{p_0}=0$ 补成 $\mathbb{R}^{K+1}$。即向量真自由度 $K$、却存 $K{+}1$ 维（首位恒 0）。与 softmax 取 $K{+}1$ 类一致、不算错，只是「枢轴」分量冗余。脚注已正确指出并改正 Handbook 印刷笔误（值得肯定）。
- **修复建议**（可选）：一句话点明「首元恒 0、不携信息，仅为与 $K{+}1$ 类 softmax 对齐而保留」，避免读者误以为 $K{+}1$ 个独立量。当前脚注已接近，可不动。

### N3. learning：`eq:learn-dust3r-loss` 旁白「与 eq:learn-dust3r-loss 之外、DROID 的置信…」自指措辞绕口
- **位置**：`learning_slam.tex:442`
- **问题**：「注意这与 `\cref{eq:learn-dust3r-loss}` 之外、DROID 的置信 $w_{ij}$…异曲同工」——句中 `\cref` 指向的正是本式自身，「之外」语义含混（本意应是「与 DROID 那边的置信 $w_{ij}$ 异曲同工」）。非数学错，纯文字。
- **修复建议**：改为「这与 DROID 的置信 $w_{ij}$（`\cref{eq:learn-droid-cost}`）异曲同工——都让网络自报『信我几分』」。

### N4. semantic：`eq:semantic-mesh-residual`（IoU 倒数）作「残差」与最小二乘语义略有出入
- **位置**：`metric_semantic_slam.tex:196-201`
- **问题**：把残差取「掩膜与渲染投影的并/交」（IoU 倒数），最优时该比为 1（非 0），与全书「残差→0、$\|\cdot\|^2$」的因子语义不齐；真正喂 GN 时通常用 $1-\mathrm{IoU}$ 或 BCE（正文也提了 $\ell_2$/BCE 备选）。标「选读」、且忠于源，影响小。
- **修复建议**（可选）：注明「此式取最小值 1 而非 0，实际多用 $1-\mathrm{IoU}$ 或 BCE 接入最小二乘」。

### N5. epilogue：引用 `\cite{teed2026boosting}` 标注「Handbook 的结语（ch19）」可能张冠李戴
- **位置**：`epilogue.tex:132`（延伸阅读首条）
- **问题**：本章自述吸收 Handbook **ch19 Epilogue**，但延伸阅读把「本章精神之源…结语」挂在 `\cite{teed2026boosting}`（该键按 learning 章 `:577` 是 **ch13《Boosting SLAM with Deep Learning》**）。ch19 结语与 ch13 应是不同条目；这里很可能误用了 ch13 的 bib 键代指 ch19。注：题述「cite 零未定义」只保证键存在，不保证键**语义对**。
- **修复建议**：若 `refs.bib` 有独立的 ch19/Handbook-epilogue 条目则换用之；若全书统一用一个 Handbook 键，则改述为「SLAM Handbook\cite{...} 的结语章」并确保该键确实涵盖 ch19（建议人工核 bib）。

---

## 复核确认无误的要点（供放心）

- **缝合线一致**：五章 + part.tex 注释均以 `eq:intro-nls` + 右扰动 + log-odds 为缝合线，且每章首「前置知识桥接」「如果跳过」「本章与前后章关系」三件套同构、过渡句（semantic→openworld 的 `sec:semantic-closing`、openworld→epilogue、epilogue 承上启下）自然。**浑然一体达成**。
- **数学（已逐一验算）**：
  - learning 可微分 BA 反传**正确地** `\cref{sec:nlopt-diffopt}` 而非重推（该基节确含 unrolled/隐式/HVP 全套且用右扰动，自包含属实）；`eq:learn-flow-pred` 与重投影 BA 同构✔；`eq:learn-droid-cost` GN 目标合理✔；RGB-D/双目加因子推广✔。
  - dynamic：`eq:dyn-reproj` 令 $\mathbf{T}^k_{wi}\equiv\mathbf I$ 退回静态✔；`eq:dyn-increment` 明确标注为匀速螺旋**一阶近似**（略去左雅可比，诚实）✔；ARAP `eq:def-arap` 局部旋转闭式 = Procrustes/Kabsch✔（数值验证）；Killing 对称化梯度判据✔；非刚性融合复用 `ch:dense` TSDF✔。
  - semantic：物体观测残差 `eq:semantic-obj-residual` 与 `eq:intro-nls` 同形✔；$\mathbf{Q}_s^*\propto\mathbf{Q}_s^{-1}$✔；对偶圆锥解析投影✔；半向量化化标准最小二乘✔；**相机侧**右扰动符号✔（数值）；多类 log-odds 加性更新 = 二值 `eq:dense-octo-logodds` 向量化、$K{=}1$ 退化为 sigmoid✔；PGMO 化 PGO✔；treewidth 界接 `thm:elimination`✔。
  - openworld：`eq:ow-mapelem` 几何/语义解耦立场✔；range-feature 接 range-category✔；信息瓶颈 `eq:ow-ib` 方向与 Tishby 一致（$\beta$ 大→保真）、接本书 `eq:mutual-info`✔；`eq:ow-aib` 的 $D_{\mathrm{JS}}$ 确为 `eq:gauss-kl` 之对称化✔；CLIP 零样本 `eq:ow-clip-cls`→`eq:ow-query` 同形✔。
- **跨章 `\cref` 目标全部存在且语义相符**：`eq:intro-nls / sec:nlopt-diffopt(+eq:nlopt-blo/implicit-grad/hvp) / eq:dense-octo-logodds(+logit/clamp/bayes/sensor) / eq:dense-tsdf-fuse/proj / thm:fg-map / eq:fg-ls / eq:mutual-info / eq:gauss-kl / thm:elimination / thm:intro-scale / sec:est-mahalanobis` 等逐一核对通过（含 `eq:mutual-info` 确在 `sec:est-mahalanobis` 下）。
- **无重复**：learning 反传不重推、明确只用结论；semantic/openworld 复用 dynamic 的 ED 图/`sumner2007embedded`、复用 ch:dense 的表示，均交叉引用而非重写。
- **记号一致**：右扰动 `T Exp(δξ)`/`q Exp(δφ)` 全 5 章统一（无左扰动滑落）；光流恒带 `^flow`、特征恒用 `ψ`（openworld 显式避让 `f`/`f^flow`）、相关体 `C^corr` vs 置信图 `C`、`\mathbf R` 只用于旋转——跨章无打架。各章首记号 note 均显式登记避让。
- **独立性（除 M2 外）干净**：epilogue（最高 ventriloquize 风险）未照搬格言，「学者…寄语」仅描述源章性质并声明「熔成本书自己的收束之辞」；Cadena 历史叙事「借…\cite」合法。openworld §17.4（立场重灾区）一律「实验事实/基准上/事实面」陈述、立场只以「本书的判断/立场/口吻」给出，stance-stripping 到位。「Handbook 原文…改写为本书下标式」「Handbook 源文印刷笔误…本书改正」等均是合法的约定声明/勘误（反而强化独立性）。

---

## 摘要（计数 + top 问题 + 浑然一体评价）

- **Blocker：1** — B1 semantic `:287` 物体位姿右扰动雅可比写成了**左扰动**式（漏 $\mathbf{q}(\cdot)\mathbf{q}^{-1}$ 共轭；数值验证残差量级不符），且恰是本章「用本书右扰动统一」的核心卖点。
- **Major：2** — M1 dynamic `eq:dyn-couple` 的 `h^{-1}` 除零（第 4 分量恒 0，应取前三维）；M2 dynamic `:128/:145`「Handbook 提出/给出」单书叙述依赖（独立性红线，余四章均无此问题）。
- **Minor：5** — N1 semantic 相机侧中间话术指错量（结论对）；N2 多类 log-odds 维度冗余表述；N3 learning `:442` 自指措辞绕口；N4 mesh IoU 残差最小值非 0;N5 epilogue 结语 `\cite` 可能用了 ch13 键代指 ch19（需人工核 bib）。

**Top 问题**：B1（右扰动算错、踩中卖点，必修）> M1（除零，必修）> M2（独立性，宜修）。

**「浑然一体」评价：是（局部需微调）。** 作为一个 part，五章是一位通读全源后的作者写出的连贯演进，无三明治拼贴、记号/详略不割裂，缝合线（母方程/右扰动/log-odds）真正贯穿，过渡自然、首尾（openworld+epilogue 呼应绪论 `sec:intro-need`/`sec:intro-frontier`）扣合。扣分仅在 B1/N1（右扰动推导的物体侧—这是数学正确性问题而非接缝问题）与 M2（dynamic 两句叙述口吻没跟上其余四章的独立性水准）。修掉 B1+M1+M2 后即为高完成度的无接缝新部。
