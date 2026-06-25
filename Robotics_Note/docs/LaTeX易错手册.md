# LaTeX 易错手册（《机器人学笔记》工程）

> 汇集本工程编译 / 排版踩过的坑。每条：**现象 → 原因 → 修复 → 出处**。
> 与 [`绘图手册.md`](绘图手册.md) 同级，编写新章前速查。
> 类（ElegantBook v4.x）+ XeLaTeX + biber，Docker `texlive/texlive:latest`，`./compile.sh` 一键编译。

---

## 1. 浮动体与表格

### 1.1 小结 / 参考表漂走、标题与表分离
- **现象**：`\begin{table}[ht]` 的小结表浮到页末或下页，正文与表之间留大块白。
- **原因**：`[ht]` 允许浮动；小结 / 符号表 / 速查表应「就地」不浮动。
- **修复**：`\usepackage{float}`（已在 `common/preamble.tex`）+ 用 `[H]` 强制就位。
- **注**：`[H]` 不浮动也**不跨页**——超长表改 `longtable` / `supertabular`。

### 1.2 `\paragraph{X。}` + 紧跟表格 = 假标题
- **现象**：表格上方的「标题」其实是正文段：带句号、不编号、不进表目录。
- **原因**：`\paragraph` 是节级命令，不是表标题。
- **修复**：
  - 参考 / 速查表 → `\caption{X}`（编号「表 N」，进表目录）。
  - 节级小结（本章与后续关系 / 故障排查手册等）→ `\section*{X}`。

### 1.3 表 / 图按「章.序」编号
- `styles.tex`：`\numberwithin{table}{chapter}`、`\numberwithin{figure}{chapter}`（须给表加 `\caption` 才随之编号）。

### 1.4 `\paragraph` 冒出 `2.2.0.0.1` 式深层编号 / ToC 过深
- **原因**：默认 `secnumdepth` 较深，`\paragraph`（4 级）也被编号 → `2.2.0.0.1`。
- **修复**：`styles.tex` `\setcounter{secnumdepth}{2}`（只编到 section）、`\setcounter{tocdepth}{1}`（ToC 只列到 section）。

---

## 2. 数学公式

### 2.1 矩阵列数 >10 → `Extra alignment tab has been changed to \cr`
- **原因**：amsmath 默认 `MaxMatrixCols=10`；如 DLT 的 12 列设计矩阵触发。
- **修复**：`styles.tex` `\setcounter{MaxMatrixCols}{20}`。

### 2.2 一个公式两个 `\label` → `Multiple \label's`
- **修复**：删多余 / 死标签，一式一标签。排查死标签：标签名全局 `grep` 引用计数为 0 即可删。

### 2.3 裸数学命令落在中文文本里 → `Missing $ inserted`
- **现象**：`…就属此列）\to 增大阻尼项…`（`\to` 不在 `$…$` 内）报 `Missing $ inserted`，TeX 在 `\to` 前自插 `$`。
- **原因**：`\to \Rightarrow \le \ge \approx \pm \times \cdot \in \sim …` 是**数学模式专用**命令，文本模式裸用即缺 `$`。多源合并时草稿里「中文 + 裸数学箭头」最易漏包；同行已在 `$…$` 内的同名命令（如 `$-2\to-12$`）正常，**只裸用的坏**。
- **修复**：包行内数学 `$\to$`；只包裸用的，已在 `$…$`/`\[..\]`/equation 内的勿重复包。
- **排查**：须**数学模式感知**（普通 grep 把 `$…$` 内误报）——跟踪 `$`/`\[`/`\(`/math-env 切换后只标文本态裸用。**坑**：跨行 `\[..\]` 与相邻 `$a$$b$` 的 `$$` 易让脚本失配，宁可只检测、人工核改。实例：`prac_tennis_env.tex:838` 两处 `\to`。

---

## 3. 代码框 codebox

### 3.1 标题里的 `_` → `Missing $ inserted`
- **原因**：`codebox` 标题（#2）经 tcolorbox `title=` 传入，**非 verbatim**，`_` 被当下标。
- **修复**：标题里转义 `\_`。实例：`gen\_pattern.py`、`LaValle BUILD\_ROADMAP`、**`obs\_groups\_spec`（gpu P8 `wheeled_bimanual:688`，本会话；第三次复发）**。
- **注**：框体（listing 内容）里的 `_` 无害，**只标题要转义**。错误**报在 `\end{codebox}` 行**（裸 `_` 开的数学跑到框尾才插 `$`），别被行号骗去查框体。
- **排查**：`scan_box_titles.py`（提取 `[lang]{title}` 的 title 做括号匹配，跳 `$…$` 后查裸 `_^#&`；普通 grep 会把 `$v_z$` 的 `_` 误报）。

### 3.2 `[lang]` 非 listings 合法语言 → `Package Listings Error: Couldn't load language`
- **现象**：`\begin{codebox}[text]{...}`（伪代码框）报错。
- **原因**：listings 无内置 `text` 语言。
- **修复**：`styles.tex` `\lstdefinelanguage{text}{}`（空定义＝纯文本、无关键字高亮）。
- **合法语言**：`C++` / `Python` / `bash` / `text`(自定义)。新语言先确认 listings 支持或自定义空语言。
- **注**：codebox 右上角语言角标取 `#1`；留空（裸 `\begin{codebox}{...}`）则角标不显。

### 3.3 `\verb|…|` / `\lstinline|…|` 代码含 `|` → 提前闭合 → `Extra }`
- **现象**：`\verb|wait(lock, [&]{return updated || !running;})|` 报 `Extra }, or forgotten \endgroup`。
- **原因**：`\verb`/`\lstinline` 以**首个定界符**配对；代码里 C++ `||` 的第一个 `|` 提前终止 verb → 残留 `| !running;})` 落入普通 TeX，`}` 无配对 `{` → `Extra }`。
- **修复**：定界符换成**代码中不出现**的字符。本工程 `slam_system.tex` 用 `@`：`\verb@wait(lock, [&]{return updated || !running;})@`（代码无 `@`；`^ ~ # / + = ?` 亦常可选）。
- **排查**：`grep -rnP '\\(verb|lstinline)\|[^|]*\|\|' parts/`。

### 3.4 `\verb|…|` 落在别的命令参数（`\textbf{…\verb…}`）内 → `Missing $` / `\verb ended by end of line`
- **现象**：`\textbf{真实方法是 \verb|parent.attach()|（…）}` 报 `Missing $ inserted` + `\verb ended by end of line` + 其后整段 `Extra }` 连锁（一处坏 → 定界失配，串几十行；曾一个 `\emph` 内 `\verb` 引出 100 条级联）。
- **原因**：`\verb` 脆弱——参数已按固定 catcode **读入后** `\verb` 才改 catcode，太晚 → 定界 `|` 解析错乱。**任何**命令参数内（`\textbf`/`\emph`/`\texttt`/`\section`/box 标题/表格单元）都不能放 `\verb`。（与 §3.3 不同：那是定界符撞内容 `|`；这是 `\verb` 根本不该进参数。）
- **修复**：把 `\verb` 移**出**参数——强调只覆盖散文、代码作独立 `\verb`：`\textbf{真实方法是} \verb|parent.attach()|（…）`。**勿**改 `\texttt{…}` 代替（含 `--`/`<`/`>` 触发连字、`_` 需转义，污染代码字形）。
- **排查**：括号深度跟踪脚本（普通 grep 难判嵌套）——`find_verb_in_arg.py`（跟踪 `\textbf{…}` 等参数栈、标其内 `\verb`）。gpu P8 一次抓 26 处。

---

## 4. 字形 / 字体

### 4.1 emoji / 特殊符号豆腐块 → `Missing character: There is no X in font [TeXGyreTermes...]`
- **元凶**：正文字体 TeX Gyre Termes 无此字形。本工程遇到：
  - `⭐`(U+2B50) 难度星、`🔬`(U+1F52C) 装饰、`↔`(U+2194) 双向箭头。
- **修复**：
  - 难度星 → 数学 `$\bigstar$`（amssymb，从数学字体出，与正文字体无关）。
    **且星只进章末「知识点总表」，不进章节 / 节标题**——标题进 ToC / 书签 / 页眉，须干净（对齐 `camera_model` 范式：标题无星，难度在总表）。
  - 装饰 emoji（🔬 等）→ 直接删。
  - `↔` → `$\leftrightarrow$`。
- **排查**：`grep -rnP '[\x{2B50}\x{1F52C}\x{2194}]' parts/`（按需加码点）。

### 4.2 nullfont 的 `;`（良性，**忽略**）
- **现象**：`Missing character: There is no ; ("3B) in font nullfont!`，成簇（数十次，量随**跨页**代码框增减——曾见 80 / 83，新增代码章会涨）、全是 `;`。
- **原因（已实测定位）**：**tcolorbox `breakable` 的断点试排**。长 `codebox`（尤其 C++、分号密）**跨页**时，tcolorbox 为定断点对上半部做一遍**测量试排**，那遍字体未激活（nullfont）→ 代码里的 `;` 报缺字。**只有跨页的代码框触发**；单页装得下的无此遍、无警告（故全书几十个 C++ 框只少数报）。
- **更正**：早期此条曾归因「TikZ `\foreach{\draw…;}`」，**误判**——图里的 `;` 由 pgf 解析吞掉、根本不排版。本会话按**页号**定位（物理页 360 = §12.6 `loop_closure.cpp` 代码框，42 个 `;`），才查实是代码框试排，与 TikZ 无关；文件名定位曾被 `.log` 换行骗到 lidar（lidar 章 0 代码框）。
- **处理**：**忽略**。试排那遍不进最终输出，**PDF 里分号俱全**。实证：`pdftotext book.pdf -` 提取到 `db.add(descriptors[i]);`、`for (int i = 0; i < ...; i++)` 分号在 → 输出无损、不计入「零缺字」。
- **勿**全局 `\tracinglostchars=0` 消警告——会连真·中文豆腐块（§4.1）一起瞎，得不偿失。
- **辨真假**：缺字簇**全是 `;`**（无中文/符号）+ `pdftotext book.pdf - | grep -nE 'db\.add|for \('` 见代码分号在 → 良性。
- **彻底消除（可选，未做）**：给 `codebox` 样式改 `enhanced jigsaw`，或给跨页长框钉 `breakable=false`/挪位避免跨页。动共享样式有中风险，本工程选「记为良性、忽略」。

### 4.3 标题含数学 / 特殊符号 → hyperref `Token not allowed in a PDF string`
- **原因**：hyperref 把标题转成 PDF 书签 / 字符串时，`$...$`、`\bigstar` 等非纯文本 token 不被允许（书签会丢字或告警）。
- **修复**：`\texorpdfstring{<排版用>}{<书签 / ToC 用纯文本>}`。本工程 P0 章难度星即此法：
  ```latex
  \section{四元数 \texorpdfstring{$\star\star\star$}{***}}   % 页面出 ★★★、书签/ToC 出 ***
  ```
- **关联**：见 §11.1——难度星「进标题」须配 `\texorpdfstring`，「只进总表」则无此问题。

---

## 5. 交叉引用与书签

### 5.1 `\cref{...}` 显示「?? N」——号码出来了、类型词成 ??（另见 §5.5：整条空 `??`）
- **现象**：如 `\cref{ex:kf-1d}` 渲染成「?? 4.1」（应为「例 4.1」）。
- **原因**：cleveref 的 `\crefname` 绑定的**计数器名写错**。ElegantBook 的 `example`(例) 是 amsthm 式、真实计数器为 **`exam`**，并非 `tcb@cnt@example`（theorem/definition 等才是 `tcb@cnt@*`）。
- **排查**：容器内编译产出 `build/book.aux`，看 `\newlabel{X@cref}{{[计数器]...}}` 直接揭示真计数器名。
  ```
  ex:kf-1d@cref -> {[exam][1][4]4.1...}      # → 计数器是 exam
  thm:kf@cref   -> {[tcb@cnt@theorem]...}    # → tcb@cnt@theorem
  ```
- **修复**：`styles.tex` `\crefname{exam}{例}{例}`（不是 `tcb@cnt@example`）。
- **地面真相**：`pdftotext build/book.pdf - | grep -c '??'` 数 PDF 里真正的 ?? —— latexmk 判「收敛」不等于零 ??。

### 5.2 `\chapter*` 不自动进目录；目录页本身无书签
- **现象**：renewed `\tableofcontents` 用 `\chapter*{目录}`，目录页无 PDF 书签。
- **修复（复用模板、非自定义）**：
  - **仅书签**（不在目录里加自指条目）→ hyperref `\pdfbookmark[0]{目录}{toc}`（模板 cls L107 已载 hyperref、L108 `bookmarksnumbered/open` 已开）。本工程 `book.tex` 用此法。
  - **既书签又进目录** → 模板自身惯用 `\phantomsection\addcontentsline{toc}{chapter}{X}`（cls L1237 同款）。
- **注**：`\pdfbookmark` / `\addcontentsline` 都是 hyperref / 内核命令，**不是自定义书签功能**；优先复用，勿造轮子。

### 5.3 缺 cite key「毒化」biblatex 收敛
- **现象**：文献明明加了却一直 undefined；latexmk 反复「(re)run Biber」、ref 未定义计数虚高。
- **原因**：任一 `\cite` 的 key 不在 `.bib`，biblatex 不收敛，连带污染整体引用解析。
- **修复**：补全**所有** key（尤其 `\nocite{*}` 时，bib 里每条都会被引，任何残缺 / 语法错都暴露）。

### 5.4 example 之外：exercise / problem 的 crefname 也可能错（潜在）
- ElegantBook 的 `exercise`(练习) / `problem`(习题) 是 `\newenvironment`（cls L1169 / L1180），计数器名未必是 `tcb@cnt@exercise/problem`。
- **现状**：全书无 `\cref{exer:/prob:}`，故 `styles.tex` 那两条 crefname 即便错也**无害**（不触发 ??）。
- **将来若 `\cref` 练习 / 习题**：先按 §5.1 用 `.aux` 的 `@cref` 条目验真实计数器名，再绑 `\crefname`。

### 5.5 `\cref` 指向无编号单元（`\chapter*` / frontmatter）→ `??`（另见 §5.1：有号的 `?? N`）
- **现象**：`\cref{ch:notation}` 渲染成 `??`（连号码都没有、整条空）。本工程前言「主要符号与记号约定」用 `\chapter*`（无号）+ `\addcontentsline` + `\label{ch:notation}`。
- **原因**：`\chapter*` 不步进章计数器，`\label` 捕获的 `\@currentlabel` 为空/陈旧 → cleveref 无号可印 → `??`。
- **修复 / 约定**：无编号单元一律 `\nameref{ch:notation}`（印章节**名**「主要符号与记号约定」、仍超链可点；`nameref` 由 hyperref 自带）。或 `\hyperref[ch:notation]{自定文字}` / `\cpageref`（印页码）。**新章引记号约定 = 直接 `\nameref{ch:notation}`，勿 `\cref`。**
- **复发史**：先 `intro_slam` / `rigid_body_motion` 各 1 处（已 nameref）；后 **P5_frontier 新 4 章**（`learning_slam` / `open_world_slam` / `dynamic_deformable_slam` / `metric_semantic_slam`）又各误 `\cref{ch:notation}` → 4 个 `??`，本会话批量 `sed` 改 `\nameref` 修平。**易复发坑**——新章作者照搬旧章正文易再犯。
- **辨识**：`comm -23 <(被引 label 排序) <(已定义 label 排序)` **为空**却仍有 `??` → 八成是「label 存在但指向无号单元」，而非缺 label。查 label 所在是否 `\chapter*` / `\section*`。
- **排查**：`grep -rn '\\cref{ch:notation}' parts/`（应空；命中即坏引用，改 `\nameref`）。

---

## 6. 宏定义

### 6.1 需参宏「裸用」吞掉后随 token
- **现象**：`\rebuilt`（定义 `\newcommand{\rebuilt}[1]{...}` 吃 1 参）被裸用：`\paragraph{...}\rebuilt` 后接 `\textbf{...}` → `\rebuilt` 吞掉 `\textbf` 当参数 → `Argument of \textbf has an extra }` / `Paragraph ended before \textbf was complete`。
- **修复**：改双形宏，`\@ifnextchar\bgroup` 探测后随是否 `{`：
  ```latex
  \makeatletter
  \newcommand{\rebuilt}{\@ifnextchar\bgroup\rebuilt@arg\rebuilt@bare}
  \newcommand{\rebuilt@arg}[1]{\todo[color=orange!40]{重建·待核对：#1}}
  \newcommand{\rebuilt@bare}{\todo[color=orange!40]{重建·待核对}}
  \makeatother
  ```
  裸用 → 走无参分支不吞 token；`\rebuilt{X}` → 走带参分支。一处定义修全部裸用、不动正文。
- **排查**：`grep -rnP '\\rebuilt(?!\{)' parts/` 找裸用。

### 6.2 `\noindent` 紧跟中文字符 → `Undefined control sequence`
- **现象**：`\noindent于是得到…`（`\noindent` 后**直接**中文、无空格无 `{`）报 `Undefined control sequence`，但错误行 hexdump 字节全干净、无隐藏字符，看不出哪个命令未定义。
- **原因（精确机制，2026-06 二次复现后 bisect 定位）**：控制**字**（多字母命令）的名字由「连续 catcode=letter 字符」界定；xeCJK 下 CJK 汉字 catcode=11（letter），故 `\noindent该算法…` 里 `\noindent` 后的汉字被**并入控制字名**，整串 `\noindent该算法对浮动基座一视同仁` 成了**一个未定义的多字控制字**——这就是「hexdump 干净却找不到哪个命令未定义」的原因：未定义的正是这串巨型 cs 本身。凡「多字母命令**紧贴** CJK」皆中招（`\par中`、`\centering中` 同理）；命令后跟 `{`/`\cref` 因 `{`、`\` 非 letter 会正确断名，故从不暴露。
- **修复**：插空组断名——`\noindent{}于是…`（`{}` 的 `{` 是 catcode=1，立即终止控制字名；比尾随空格更稳）。
- **已三次踩中**：`dense_mapping`（首现）、`quad_kin:740`（四足部合并再现）、**`prac_tennis_perception:353`（gpu P8 并入，本会话）**——均「写作子 agent / 外部作者照搬草稿」复发。
- **诊断教训（本会话最耗时坑）**：line 353「hexdump 字节全干净、env 平衡、宏全标准，却 `Undefined control sequence`」——一度全书 bisect 半天。**正解**：遇此症状**先**跑下面排查 grep（秒级定位），勿从头 bisect；或最小文档 `\input` 单文件复现（~30s/轮，远快于全书 10min）。错误显示的 top line 末尾（断点处）即那串巨型 cs 的尾字。
- **排查（通用，不止 `\noindent`）**：`grep -rnP '\\[a-zA-Z]+[\x{4e00}-\x{9fff}]' parts/` 抓**任意控制字紧贴 CJK**（比只查 `\noindent` 更全）。

### 6.3 `\qed` 裸用（`proof` 环境外）→ `Undefined control sequence`
- **现象**：行内推导收尾裸写 `\qed`（如 `…故 $\boldsymbol{\tau}=\mathbf{J}^\top\mathbf{F}$。\qed`）报 `Undefined control sequence`。
- **原因**：本工程的证毕记号由 `proof` 环境在 `\end{proof}` 处**自动**插入；并未全局定义可裸用的 `\qed`（amsthm 的 `\qed` 在本配置正文不可直接裸用）。四足 `quad_kin` 在 `proof` 外裸用 `\qed` 三处 → 未定义。
- **修复**：`common/preamble.tex` 加 `\providecommand{\qed}{\hfill\ensuremath{\square}}`——全书正文裸用 `\qed` 收尾皆可，且 `providecommand` 仅在未定义时定义、与 `proof` 自带 qed 不冲突。
- **排查**：`grep -rnP '\\qed(?!here)' parts/` 看裸用 `\qed` 是否落在 `proof` 之外。

### 6.4 meta 宏（`\rebuilt`/`\pz`/`\pzr`）落进 `\caption` 等移动参数 → 脆弱报错
- **现象**：`\caption{…\rebuilt{…}…}` 报 `Argument of \caption has an extra }`——`\rebuilt`（§6.1 的 `\@ifnextchar` 双形宏）在移动参数里展开 → 破坏 brace 计数。
- **原因**：`\rebuilt`/`\pz`/`\pzr` 非 robust；`\caption`/`\section` 等把参数写进 `.lot/.lof/.toc`（移动参数），脆弱命令在那里展开即坏。
- **修复**：定义改 `\DeclareRobustCommand`（自带 `\protect`，移动参数里安全）：`\DeclareRobustCommand{\rebuilt}{\@ifnextchar\bgroup\rebuilt@arg\rebuilt@bare}`；`\pz`/`\pzr` 同改。一处改、全局生效（`common/preamble.tex`，本会话已改）。
- **注（元词汇，待用户定夺）**：`\rebuilt`/`\pz`/`\pzr` 渲染出 `[重建待核对：…]`/`[批注：…]`/`[重点：…]` **可见进 PDF**——属「草稿批注」，与「最终成稿无元审稿词汇」原则相悖；并入正式书前宜统一**剥除**（内容留、标记去），记录转 errata MD。gpu P8 含 91 处 `\rebuilt` + 若干 `\pz`/`\pzr`，**暂保留**（先保编译通过），见 §12。

---

## 7. 页面样式

### 7.1 每章首页 / 目录首页无页码
- **现象**：每章首页、目录首页底部无页码，其余页有（正文阿拉伯、前言区罗马）。
- **原因**：ElegantBook `\pagestyle{fancy}` 把页码放**页脚居中**（cls L1262 `\fancyfoot[c]{\color{structurecolor}\small\thepage}`）；但章首页（含目录，经 `\chapter*`）切到 `plain`，而 cls 把 `plain` 重定义成 `\fancyhf{}` **全清空**（L1274）→ 无页码。
- **修复（复用 cls 自己的页码 spec）**：`styles.tex` 覆盖 `plain`，补回**同款**页脚页码、仅清页眉：
  ```latex
  \fancypagestyle{plain}{%
    \fancyhf{}%
    \fancyfoot[C]{\color{structurecolor}\small\thepage}%
    \renewcommand{\headrulewidth}{0pt}\renewcommand{\headrule}{}%
  }
  ```
- **影响面**：所有走 `plain` 的页（章 / 部首页、目录、前言、参考文献首页）统一显页脚页码；**封面**（`\maketitle`→`empty`）与 `\cleardoublepage` 空白页**不受影响**（用 `empty` 非 `plain`）。

---

## 8. 编译与 biblatex

### 8.1 latexmk 收敛判词
- **权威成功**：`Latexmk: All targets (build/book.xdv build/book.pdf) are up-to-date` 且**无** `Latex failed to resolve N reference/citation(s)`。
- **虚高假象**（别据此判）：逐趟 `There were undefined references`、累积 `undefined on input` 数——含第 1 趟（读 .aux 前一切未定义），会几千计。
- **地面真相**：
  - `pdftotext build/book.pdf - | grep -c '??'` 数真悬空引用。
  - `Reference X@cref` 未定义名 vs 源 `\label` 全集做 `comm` 交叉核验，定位真缺失（0 = 纯收敛假象）。

### 8.2 `compile.sh` 机制
- 源 `/src` **只读**挂载；容器内 `cp -a` 到 `/tmp/work` 编译。**成功**仅 `build/<target>.pdf` 回宿主、`.aux/.log` 随 `--rm` 丢弃（`build/` 只剩 PDF）；**失败**导出 `<target>.log` 供排查（见 §8.4）。
- 抓全 log 调试：`./compile.sh book > /tmp/x.log 2>&1`；`-file-line-error` 让硬错带 `file:line:` 前缀。
- `.latexmkrc`：`$out_dir='build'`、`$pdf_mode=5`(xelatex)、`$bibtex_use=2`(biber)、无 `$max_repeat`（默认 5，本工程够用）。
- 需 `dangerouslyDisableSandbox`：docker / ssh / rsync。

### 8.3 排查工具（宿主 poppler / python）
- `pdftotext -f N -l N -layout book.pdf -`：看某页页脚页码 / 正文。
- `pdftoppm -png -r 90 -f A -l B book.pdf /tmp/pg`：渲染页面图核对版式（封面 / 目录 / 章首页页码与页眉）。
- `pip install --user pypdf` → `PdfReader(pdf).outline` + `get_destination_page_number(it)`：验书签存在与落点物理页。

### 8.4 `compile.sh` 失败时导出错误日志（本会话改进）
- **痛点**：原脚本 `set -e` + latexmk 非零退出 → 跳过 `cp pdf`、`.log` 随 `--rm` 蒸发、宿主 `build/` 空 → 无从排查，每次失败得另跑诊断 docker 才能拿 log。
- **改进**：容器内把 latexmk 包进 `if…then cp pdf; else 导出 .log + 摘要; exit rc; fi`：
  ```bash
  if latexmk -xelatex -interaction=nonstopmode -file-line-error "$TARGET.tex"; then
    cp "/tmp/work/build/$TARGET.pdf" /out/                 # 成功：只回传 PDF（build/ 干净）
  else
    rc=$?; echo "‼ latexmk 失败 (rc=$rc) — 导出 $TARGET.log"
    cp "/tmp/work/build/$TARGET.log" /out/ 2>/dev/null || true
    grep -nE "^\./[^:]+\.(tex|sty):[0-9]+:" "/tmp/work/build/$TARGET.log" | head -30 || true
    exit $rc
  fi
  ```
  外层 `if docker run …; then ✅ else ❌（日志见 build/）fi`。**成功仍只留 PDF**、失败留 `book.log` + 打印前 30 条 `file:line:` 错误，一步定位。本会话 5957 连锁错误即靠它一步拿到摘要、收敛到 1 个根因。

### 8.5 裸 `%` 在 `.bib` 字段值 → `.bbl` runaway → frontmatter 全崩 + Emergency stop
- **现象**：`book.tex:18: File ended while scanning use of \field.`（行号指 `\begin{document}`）+ 前言/notation 满屏 `Illegal parameter number in \NewCounter`/`Incomplete \iffalse`/`\\itemize doesn't match` + `! Emergency stop`。看着像 frontmatter 崩，**实与前言无关**。
- **原因**：`.bib` 某字段值含**裸 `%`**（如 `note = {… 98.9% on AMASS …}`）。biber 照原样写进 `book.bbl`，xelatex 读 `.bbl` 时 `%` 当**注释**吃掉行尾（含该 `\field` 的闭 `}`）→ `Runaway argument` → 吞下一个 `\field` → 扫到 EOF → `File ended while scanning \field` → 其后所有源码错位（前言被当 .bbl 残体解析）→ Emergency stop。
- **关键诊断**：biber 的 `.blg` **干净无错**（biber 不校验 `%`）→ 易误判「biber 成功、.bbl 没问题」。真相在 latexmk 全 log 的 `Runaway argument? {…(98.9\field …` —— 那截即 `%` 后被吞处。**`.bbl` 读崩 ≠ biber 失败**：biber 写得完整，是 `%` 让 xelatex 读崩。
- **隐蔽性（为何晚现）**：只要 pass1（首遍 xelatex）有**任何**致命错（如本会话 P8 未修完时 `slam_2d` 爆栈），biber/pass2 根本不跑、`.bbl` 不被读 → `%` 坑被**掩盖**；待全部源码错修净、pass1 首次跑通，biber+pass2 启动才暴露。**「改完最后一个源码错，却冒出一堆 frontmatter 错」= 高度疑此类被掩盖的 .bbl/biber 坑**，别去查前言。
- **修复**：字段值里 `%`→`\%`（`#`→`\#`、`&`→`\&` 同理，均 .bbl 危险；`%` 最烈因吞整行）。本会话 `refs.bib:5535`（P8 自动并入条目 `note` 字段两处 `98.9%`/`100%`）。
- **排查**：`grep -nP '(?<!\\)%' refs.bib` 看**非行首**命中（行首 `%`/`%%` 是 .bib 注释、无害；字段值 `{…%…}` 内才坏）；括号平衡检查查不出 `%`，须单列。
- **同族（`_`/`^`/`&`）**：裸 `_`/`^` 在字段值 → `Missing $ inserted`（症状不同于 `%` 的吞行）；`&` → `Misplaced alignment tab`。本会话 `refs.bib:6243`/`6254` 的 `note` 内裸 `legged_control`。排查 `_`/`^` 须**先剥 `\url{…}`、`$…$`、url/doi/eprint 字段**再查（否则 DOI/URL 的 `_` 海量误报；脚本逐字段 strip 后只剩真坑）。
- **快验**：改完别全编（~2900pp/10min）——最小文档 `\input{common/preamble}` + `\nocite{*}` + `\printbibliography` 单编（~2min）即验全文献无排版错。

### 8.6 书体量增大 → `TeX capacity exceeded [main memory]`（默认 5M 不够）
- **现象**：`….tex:N: TeX capacity exceeded, sorry [main memory size=5000000].` 报在某 `\end{codebox}`（breakable tcolorbox）的**输出例程**（`<argument> \vbox_unpack:N \@outputbox …`），且 `strings`/`string characters` 都没满——**纯主内存池满**，非 runaway、非该代码框本身错。
- **原因**：全书 >2000pp + 海量 breakable 代码框（最耗内存）+ biblatex 在 `\begin{document}` 预载所有被引文献数据 → 主内存基线高；默认 `main_memory=5000000` words 在某 breakable 框分页时溢出。**报错行只是压垮处、非根因**（本会话报在早期 `rigid_body_motion:883`，实因全局体量；gpu 并入 P8/P3b 后书显著变大）。
- **隐蔽性**：同 §8.5——pass1 更早有致命错则到不了这里；错全修净 + 书够大才暴露。
- **修复**：调大内存。XeTeX **运行时**从 `texmf.cnf` 读 `main_memory` 等（**无需重建 format**）。`compile.sh` 容器内写本地 cnf + `export TEXMFCNF`：
  ```bash
  mkdir -p /tmp/texmf-mem
  printf "main_memory=12000000\nextra_mem_top=12000000\nextra_mem_bot=12000000\nsave_size=300000\npool_size=12000000\nbuf_size=2000000\nhash_extra=200000\n" > /tmp/texmf-mem/texmf.cnf
  export TEXMFCNF="/tmp/texmf-mem:"
  ```
  验证：`kpsewhich -var-value=main_memory` 应回 `12000000`；trivial doc 编后 log「words of memory out of **17000000**」（5M→17M）。本会话从 5M 提到 12M（总 17M），过 `rigid_body_motion`。
- **注**：书继续长大（预告还有大量新章）→ 再溢出继续上调；lualatex 内存动态是另一退路（要改引擎，本工程暂留 xelatex+大内存）。

---

## 9. TikZ / 绘图陷阱

### 9.1 样式名撞 TikZ 保留键（`out`/`in`/`at`…）→ 整图崩 + 连锁污染其后所有 enhanced tcolorbox
- **现象**：某 `tikzpicture` 里 `\node[out]{...}` 报 `Package pgfkeys Error: The key '/tikz/out'` + `LaTeX Error: Not allowed in LR mode` + `Giving up on this path` + `Missing/Extra \endgroup`；**且其后每个** `\begin{practice}`/`\begin{definition}`（凡 tcolorbox `enhanced`）都报 `Undefined control sequence \tikz@intersect@namedpaths`。本会话**一处撞名 → 5957 条连锁错误**。
- **原因**：把 TikZ 样式命名为 `out`（`out/.style={...}`），但 `out` 是 TikZ **保留路径键**（`to[out=30,in=150]` 的出射角）。`\node[out]` 触发原 `out` 键（要角度）→ path 中途崩、`\endgroup` 失衡泄漏 → 破坏 TikZ intersections 全局态 → 其后**每个 enhanced tcolorbox**（边框用 TikZ path、收尾走 `\tikz@intersect@finish`）因 `\tikz@intersect@namedpaths` 未初始化而报未定义。
- **修复**：样式名避开保留键——`out`→`outb`、`in`→`inb` 等（本工程 `loop_closure.tex` 改 `out`→`outb` ×3）。
- **诊断要诀**：满屏 `Undefined control sequence` 别慌——**按编译顺序找第一条「非未定义类」硬错误**（这里是 `out` 撞键），它常是唯一根因、其余全连锁。`grep -nE '^\./[^:]+\.tex:[0-9]+:' build/book.log` 列全部 `file:line` 错误看**最早**那条；按文件名去重 + 各文件最早行，即得真源清单。
- **保留键黑名单（勿做样式名）**：`out` `in` `at` `to` `node` `edge` `circle` `rectangle` `grid` `text` `name` `pos` `scale` `rotate` `shift` `anchor` `fill` `draw` `color` `above` `below` `left` `right` `cycle` `arc` `line` `coordinate`。`every node/.style` 等 `every X` 是**合法**惯用法、不算撞名。
- **排查**：`grep -rnP '\b(out|in|at|to|node|edge|circle|rectangle|grid|text|name|pos|scale|rotate|shift|anchor|fill|draw|color)/\.style' parts/`。

### 9.2 `calc` 因子 `n*\foreachvar` 在 `\foreach` 内 → 解析跑飞 / `TeX capacity exceeded`（致命）
- **现象**：`\foreach \p in {(0.3,0.3),…}{ \fill ($(3.7,0.75)+0.5*\p$) circle…; }` 报 `Paragraph ended before \tikz@cc@parse@factor was complete` → 一路吞到 `\end{tikzpicture}` 外 → `Not allowed in LR mode`/`titlesec horizontal mode` 连锁 → 终 `TeX capacity exceeded`（爆栈、**致命中止**，整轮编译停）。
- **原因**：`calc` 的 `<因子>*<坐标>` 语法对 `\foreach` 循环变量的展开时机敏感，`parse@factor` 解析不完整。
- **修复**：避开——把 `base+因子*offset` **预算成绝对坐标字面量**（视觉等价、最稳），或改 partway 语法 `($(a)!t!(b)$)`（全书其它图惯用此式，可靠）。实例：`slam_2d.tex` 金字塔图两处 `\foreach` 预算绝对坐标。
- **排查**：`grep -rnP '\(\$[^$]*\*\\[a-zA-Z]' parts/`（因子乘宏的 calc 表达式）。

### 9.3 tikz `\node` 文本里 `\\` 写进 `\textbf{}` 内 / node 缺 `align` → `Not allowed in LR mode`
- **现象**：(a) `\node{\textbf{(c) 视觉惯性\\ 联合 MAP}\\ …}`——`\\` 嵌在 `\textbf{}` **内**报 `Not allowed in LR mode`+`Giving up on this path`+`Extra }`；(b) `\node[anchor=west]{卸载阈值\\(…)}`——node 有 `\\` 但**无 `align`** 同报 LR mode 错。
- **原因**：node 内 `\\` 换行需该 node 处于「带 `align` 的多行模式」，且 `\\` 不能嵌在 LR-mode 的 `\textbf{}` 里。
- **修复**：(a) 把 `\\` 移出 `\textbf`——`\textbf{(c) 视觉惯性}\\ \textbf{联合 MAP}`；(b) 给 node 加 `align=left`（或 center）。实例：`vio.tex:1143-44`、`lidar_localization.tex:318`。

---

## 10. 协作同步（三端：local ↔ gpu / ai）

**拓扑**：**local = 中心（权威源）** `/home/gpf/Note/SimpleSLAM_Theory/`（书在子目录 `Robotics_Note/`）；gpu、ai 为镜像。
- **gpu-server**（ssh 别名 `gpu-server`，ziren2@oronzo，时区 Europe/Rome）：`/home/ziren2/pengfei/Robotics_Theory/Robotics_Note/`（**子目录**；研究根另含 gpu 专属源料，不同步）。
- **ai-server**（ssh 别名 `ai-server`，root@146.190.109.243）：`/root/gpf/Robotics_Theory/`。

### 10.1 ⚠ ai-server 连接坑（裸 ssh / rsync 必失败）
- **现象**：`ssh ai-server '<cmd>'` 或对 ai 跑 rsync → 报 `Cannot execute command-line and remote command`。
- **原因**：其 ssh config Host 块设 `RemoteCommand cd /root/gpf && exec $SHELL -l` + `RequestTTY yes`，与「命令行带命令」互斥。
- **修复**：命令/rsync **必须**加 `-o RemoteCommand=none -o RequestTTY=no`；rsync 用 `-e 'ssh -o RemoteCommand=none -o RequestTTY=no'`。gpu-server 无此坑、裸 ssh 即可。
- 需 `dangerouslyDisableSandbox`：ssh / rsync / docker 同。

### 10.2 rsync 规约（两端通用）
- **严禁 `rsync --delete`**（任一对分叉会丢数据）。
- **校验逐字节一致**：`rsync -azcni` 干跑（`-c` checksum 比对、免时区干扰）；推前 diff = 已知编辑、推后残差应空。
- **排除**：`build/ .git/ .claude/ __pycache__/` + latex 中间产物（`*.aux/log/toc/bcf/bbl/blg/out/fls/fdb_latexmk/synctex.gz/xdv/run.xml/pyc`）。
- **方向核验**：dry-run itemize 全 `>f`（远→本）或全 `<f`（本→远）单向、无反向即安全；混向＝两端分叉，停下核对。
- **rsync itemize 读法**：`>f`＝远端→本地；`<f`＝本地→远端；`+++++++++`＝远端新建；`fcst`＝内容/大小/时间差；`.f....og`＝仅属主属组元数据（无内容传输）；`.f..t`＝仅时间戳差（内容相同）。
- **推送量级自检**：diff / 传输文件数应对应编辑量（如删 N 处 ⭐ → diff `2N` 行 `<>`）；对不上即可能撞远端独立改动，停下核对。

---

## 11. 全书一致性待办（已发现的不统一）

### 11.1 难度星约定「三套并存」
| 范式 | 写法 | 出处 |
|---|---|---|
| 标题内带星 | 节标题 `\texorpdfstring{$\star\star$}{**}` | P0（rigid_body 等） |
| 标题净 + 表内星 | 标题无星，难度 `$\bigstar$` 入「知识点总表」 | camera_model |
| 标题净 + 表内星 | 标题无星（⭐ 已删），总表 `$\star$` | lidar_slam（本会话改） |

- 量：全书 `\star` **436** / `\bigstar` **58** / `⭐` **0**。
- **结论**：三套不统一。"对齐旧章"对 lidar_slam ＝ 对齐 camera_model（表内），但与 P0（标题内）仍不一致；且 `\star` 与 `\bigstar` 两种字形混用。
- **待定（全书统一二选一）**：
  - **A** 标题一律去星、难度只入「知识点总表」（删 P0 等标题里的 `\texorpdfstring{$\star..$}{..}`）。
  - **B** 标题一律带星、统一 `\texorpdfstring{$\star..$}{*..}`（给 camera_model / lidar_slam 标题补回）。
  - 并：星形统一为 `\star` 或 `\bigstar` 之一。

### 11.2 难度星字形：`\star`(空心小) vs `\bigstar`(实心大)
- 同上，58 处 `\bigstar`（多在 camera_model）与 436 处 `\star` 混用；统一时一并定。

---

## 12. 多源 / 草稿（markdown 习惯）合并污染（gpu P8 并入实录）

gpu 端独立写的 **P8（强化学习运控，29 章）** 以**草稿 / markdown 风格**成文，并回主书后**集中触发一批编译错**——多为「已知坑复发」+ 草稿习惯。归档此役，备下次多源合并例行排查。

### 12.1 syndrome 清单（gpu P8 实触发，按修复顺序）
| # | 类 | 现象 | 修法 | 条目 |
|---|---|---|---|---|
| 1 | TikZ calc×foreach | parse@factor 爆栈（致命） | 预算绝对坐标 | §9.2 |
| 2 | tikz `\\` in `\textbf` / 缺 align | Not allowed in LR mode | 移 `\\` 出 / 加 align | §9.3 |
| 3 | `\verb` 在命令参数内（26 处） | `\verb ended by end of line` | 移出参数 | §3.4 |
| 4 | markdown 反引号 `` `code` `` | 内裸 `_` → Missing $ | 转 `\verb` | §12.2 |
| 5 | 裸数学命令 `\to` 在文本 | Missing $ | 包 `$…$` | §2.3 |
| 6 | codebox 标题裸 `_` | Missing $（标题非 verbatim） | 转 `\_` | §3.1 |
| 7 | meta 宏在 `\caption` | brace 计数坏 | `\DeclareRobustCommand` | §6.4 |
| 8 | `\noindent` 紧贴 CJK | Undefined control seq（巨型 cs） | `\noindent ` 加空格 | §6.2 |

- **诊断教训**：#8 最耗时（`tennis_perception:353`「干净却未定义」，全书 bisect 半天才悟）——遇「hexdump 干净的 `Undefined control sequence`」**第一时间**查 §6.2 的 grep，别从头 bisect；最小文档 `\input` 单文件复现 ~30s/轮 ≪ 全书 10min。
- **复发性**：#6（§3.1）、#8（§6.2）均**第三次**踩中——外部作者 / 写作子 agent 照搬草稿易再犯；多源并入**例行**先跑 §12.3 套件。
- **biber 旁注**：本会话还遇一次 `book.bbl` 截断 → frontmatter 全 `\field`/`\NewCounter`/`Incomplete \iffalse` 级联 + Emergency stop——非源码错，是 **biber 被杀（内存）→ 半截 .bbl**；refs.bib 无裸 `#` 即排除真错，**重编即愈**（见 §8）。别被 frontmatter 级联骗去查前言。

### 12.2 markdown 反引号代码段 `` `code` `` → 内裸 `_` → `Missing $`
- **现象**：散文里 markdown 行内代码 `` `heading_alignment_reward` ``、`` `log_std` ``——反引号本身只渲染成弯引号（不雅但不报错），**内含的裸 `_` 才致 `Missing $`**。
- **修复**：转 house 风格 `\verb`——`` `X` `` → `\verb<d>X<d>`（`d` 取代码中不出现的定界符）。**仅散文内转**；codebox / 注释（`#…`）内的反引号是字面、**勿动**（需 codebox 深度跟踪）。`fix_backticks.py`（跟踪 codebox 深度 + 智能选定界符）一次转 `wheeled_bimanual` 29 处。
- **辨误报（勿改）**：`**2/sigma**`＝Python 幂、非 markdown 粗体；`[name](ctx)`＝索引记法、非链接；行首 `#`＝codebox 内 shell 注释。

### 12.3 多源并入排查套件（一次跑全，先于编译）
```bash
# 1 任意控制字紧贴 CJK（§6.2，巨型未定义 cs；最易漏最难查，第一优先）
grep -rnP '\\[a-zA-Z]+[\x{4e00}-\x{9fff}]' parts/
# 2 calc 因子乘宏（§9.2，爆栈致命）
grep -rnP '\(\$[^$]*\*\\[a-zA-Z]' parts/
# 3 verb 含定界符 |（§3.3）
grep -rnP '\\(verb|lstinline)\|[^|]*\|\|' parts/
# 4 脚本类（scripts/latex_lint/，见其 README）：
#   find_verb_in_arg.py   \verb 在命令参数内（§3.4，括号栈跟踪）
#   scan_box_titles.py    codebox 标题裸 _^#&（§3.1，跳 $…$）
#   fix_backticks.py      markdown 反引号→\verb（§12.2，codebox 感知；就地改）
#   fix_math_in_text.py   裸数学命令检测（§2.3，数学模式感知；宁只检测+人工核）
```

---

*维护：随新坑追加。最近更新见 git / 各条出处对应 `styles.tex`、`common/preamble.tex`、`book.tex`。*
