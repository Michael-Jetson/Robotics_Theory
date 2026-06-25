# ElegantBook 改动记录（《机器人学笔记》工程）

> 记录本工程相对官方 **ElegantBook v4.7** 原版的【全部】改动：**覆盖类**（改原版默认行为）、**扩展类**（加原版没有的功能）、**构建类**（编译配置）。
> 总原则（见 `common/preamble.tex` 抬头）：**版式基线 = 官方原版**——页眉 / 页边 / 字体 / 章节标题配色 / 代码默认 frame 全部沿用官方，**仅叠加必要微调**。如需调版式，在此基础上小步微调、勿大改。
> 每条格式：**改动 → 原版默认 → 位置 → 原因**。与 [`绘图手册.md`](绘图手册.md)、[`LaTeX易错手册.md`](LaTeX易错手册.md) 同级。
>
> 改动落点共 6 文件：`common/preamble.tex`（类/选项/宏包/宏）、`styles.tex`（版式/盒子/引用）、`book.tex`（结构/书签）、`figs.sty`（绘图宏包）、`.latexmkrc` + `compile.sh`（构建）。

---

## 0. 文档类与全局选项

| 项 | 值 | 说明 |
|---|---|---|
| 文档类 | `elegantbook`（官方 v4.7） | 基线 |
| 选项 | `lang=cn` | 中文（载 ctexbook、中文标题/日期/字体） |
| | `10pt` | 正文 10pt |
| | `device=normal` | 纸张/版心走「常规」设备（非 pad/screen） |

- **位置**：`common/preamble.tex` L11 `\documentclass[lang=cn, 10pt, device=normal]{elegantbook}`。
- **唯一定义处**：全集 `book.tex` 与各分册 `vol_*.tex` 均首行 `\input{common/preamble}`，全局选项只此一改。

---

## 1. 版式覆盖（改原版默认行为）

### 1.1 公式 / 图 / 表编号改「章.序」
- **改动**：`\numberwithin{equation}{chapter}`、`\numberwithin{figure}{chapter}`、`\numberwithin{table}{chapter}` → 式 3.1 / 图 2.1 / 表 2.1。
- **原版**：ElegantBook 默认**全书连续**编号（式 1、式 2 … 不带章号）。
- **位置**：`styles.tex`。
- **原因**：教材按章定位，跨章引用更清晰。**注**：表须加 `\caption` 才随之编号。

### 1.2 章节编号与目录深度
- **改动**：`\setcounter{secnumdepth}{2}`（只编到 section）、`\setcounter{tocdepth}{1}`（目录只列到 section）。
- **原版**：有效默认更深 → `\paragraph`（4 级）也被编号、出现 `2.2.0.0.1` 式深层号，目录过长。
- **位置**：`styles.tex`。
- **原因**：`\paragraph` 在本工程作「段首小标题」大量使用，不应带编号；目录只到节即够。

### 1.3 章首页 / 目录首页显示页码（覆盖 `plain`）
- **改动**：重定义 `\fancypagestyle{plain}`——补回与正文同款「页脚居中页码」`\color{structurecolor}\small\thepage`，仅清页眉与页眉线。
- **原版**：ElegantBook 把 `plain` 重定义为 `\fancyhf{}` **全清空**（cls L1274）→ 每章首页、目录首页、前言/参考文献首页**无页码**。
- **位置**：`styles.tex`。
- **原因**：章首页缺页码不便引用。**复用 cls 自身页码 spec**（cls L1262），非自定义。**封面**（`\maketitle`→`empty`）与 `\cleardoublepage` 空白页用 `empty` 不受影响。

### 1.4 `\paragraph` 小标题「标题前距」3.25ex → 1.2ex
- **改动**：`\renewcommand\paragraph{\@startsection{paragraph}{4}{\z@}{1.2ex \@plus.5ex \@minus.2ex}{-1em}{\normalfont\normalsize\bfseries}}`——仅缩 beforeskip，粗体随行小标题与 `-1em` 后距（run-in）保持原样。
- **原版**：`book.cls` L416 默认 beforeskip `3.25ex plus1ex minus.2ex`（≈1.4 行）；elegantbook/ctexbook 均未重定义 `\paragraph`。
- **位置**：`styles.tex`。
- **原因**：正文大量用 `\paragraph{...}` 作段首小标题，默认前距过松（视觉上"整空行"）；收紧到 ≈半行更紧凑。**效果**：全书省约 8 页（492→484）。

### 1.5 `\emph` 中文强调：楷体 → 黑体
- **改动**：`\renewcommand{\emph}[1]{{\normalfont\sffamily #1}}`——强调用中文黑体（`\sffamily`）。
- **原版**：CJK 下 `\emph` 默认回退「楷体」（异体、偏大、非斜体），不宜做术语强调。
- **位置**：`common/preamble.tex`。
- **原因**：黑体干净，且与 `\textbf`（粗宋）形成两级强调。作者反馈。

### 1.6 矩阵列上限 10 → 20
- **改动**：`\setcounter{MaxMatrixCols}{20}`。
- **原版**：amsmath 默认 `MaxMatrixCols=10`。
- **位置**：`styles.tex`。
- **原因**：如 DLT 的 12 列设计矩阵超默认上限会报 `Extra alignment tab`。

---

## 2. 学习笔记批注工具（注释用，不改版式）

> 仅注释/重建标记工具；`todonotes` 仍载备用，但本组宏**不再调** `\todo`（旁注长批注会溢出版心，作者反馈）→ 全部改**行内彩字**。

### 2.1 `\pz` / `\pzr` 行内彩注
- **改动**：`\pz{...}`＝行内小字蓝注 `[批注：…]`；`\pzr{...}`＝行内小字红注 `[重点：…]`。
- **历史**：曾用 `\todo[color=...]` 旁注 → 长批注溢出页面 → 改行内。
- **位置**：`common/preamble.tex`。

### 2.2 `\rebuilt` 重建标记（双形宏）
- **改动**：`\rebuilt{X}` 标具体公式/项、裸 `\rebuilt` 标整段，均输出行内 `[重建待核对…]`（`second` 色）。经 `\@ifnextchar\bgroup` 探测后随是否 `{`。
- **原因**：单参版裸用会吞掉后随 token（如 `\textbf`）→ 双形宏。详见 `LaTeX易错手册.md` §6.1。
- **位置**：`common/preamble.tex`。

---

## 3. 自定义内容盒（扩展，ElegantBook 原版无）

> 基于 tcolorbox（类已载）。区分「代码 / 推导 / 陷阱 / 练习」四类语义盒 + 三类定理式新单元。

### 3.1 `codebox` 代码框
- **改动**：`\newtcblisting{codebox}[2][]{...}`——带标题彩框（绿框 `main`）、可选 `#1`＝listings 语言（驱动高亮 + 右上角语言角标）、`#2`＝标题。可跨页。
- **用法**：`\begin{codebox}[C++]{标题} ... \end{codebox}`。
- **位置**：`styles.tex`。
- **坑**：标题里 `_` 须转义；`[text]` 等非内置语言须先 `\lstdefinelanguage`（见 §4.1、`LaTeX易错手册.md` §3）。

### 3.2 `derivation` 推导框
- **改动**：`\newtcolorbox{derivation}[1][推导]{...}`——蓝框（`third`），可跨页。
- **用法**：`\begin{derivation}[可选标题] ... \end{derivation}`。
- **位置**：`styles.tex`。

### 3.3 `pitfall` 陷阱盒 / `practice` 练习盒（无计数器）
- **改动**：`pitfall`＝红框警示（概念误区）；`practice`＝橙框（练习/前置自测）。二者**无计数器**、不参与 `\cref`。
- **位置**：`styles.tex`。

### 3.4 定理类新单元 `algo` / `insight` / `paper`
- **改动**：经官方接口 `\elegantnewtheorem{名}{显示名}{风格}{引用前缀}` 新增：
  - `algo`（算法，prostyle 蓝）、`insight`（洞见，defstyle 绿）、`paper`（论文精读，thmstyle 橙）。
  - 均带计数器、可 `\cref`（前缀 `alg:` / `ins:` / `paper:`）。
- **位置**：`styles.tex`。
- **原因**：教材需「算法 / 洞见 / 论文精读」专用可引用单元。

---

## 4. 代码排版（listings）

### 4.1 `notecode` 风格 + `text` 空语言
- **改动**：
  - `\tcbuselibrary{listings}`；`\lstdefinestyle{notecode}{...}`（配主题色 `main`/`second`、行号、断行）。
  - `\lstdefinelanguage{text}{}`——空定义＝纯文本伪代码（listings 无内置 `text` 语言，否则报 `Couldn't load language`）。
- **位置**：`styles.tex`。

---

## 5. 交叉引用（cleveref，中文 + 扩展）

### 5.1 中文 `\crefformat`（不硬编码章号）
- **改动**：`\usepackage{cleveref}` + 中文格式：章→`第N章`、节→`第N节`、图→`图 N`、表→`表 N`、式→`式 (N)`；并 `\crefname` 定义/定理/例。
- **位置**：`common/preamble.tex`。
- **原因**：自动中文引用、章号不写死。

### 5.2 容器盒 `\crefname` 绑定（含 `exam` 坑）
- **改动**：`styles.tex` 绑定 tcolorbox 计数器名：`tcb@cnt@theorem/definition/proposition/lemma/corollary/axiom`、`tcb@cnt@algo/insight/paper`、`tcb@cnt@exercise/problem`；**例外**：例用 amsthm 计数器 **`exam`**（非 `tcb@cnt@example`），写错 `\cref` 显示 `??`。
- **位置**：`styles.tex`。
- **坑**：详见 `LaTeX易错手册.md` §5.1 / §5.5。

---

## 6. 数学宏

### 6.1 `\SO \SE \so \se`
- **改动**：`\SO=\mathrm{SO}`、`\SE=\mathrm{SE}`、`\so=\mathfrak{so}`、`\se=\mathfrak{se}`（李群/李代数记号）。
- **位置**：`common/preamble.tex`。

---

## 7. 绘图子系统 `figs.sty`（附加宏包）

### 7.1 TikZ 库 / pgfplots / pgfgantt / 3dplot / bayesnet + 共享样式
- **改动**：`\usepackage{figs}`（`common/preamble.tex` 引入）。`figs.sty` 在 preamble 已载基础上追加：
  - TikZ 库：`3d, intersections, decorations.pathreplacing/markings, angles, quotes, shapes.geometric, fit, backgrounds, spy, matrix, patterns, bending`。
  - 宏包：`tikz-3dplot`、`pgfplots`(compat=1.18)、`pgfgantt`、`bayesnet`（因子图/贝叶斯网）。
  - 共享样式：`vec`（箭头向量）、`axisarrow`（坐标轴箭头）。
- **位置**：`figs.sty`（工程根）；详见 `绘图手册.md`。
- **注**：`intersections` 库使每条 TikZ path 挂 intersect-finish 钩子——是 `LaTeX易错手册.md` §9.1「样式撞保留键→连锁污染 tcolorbox」的硬件基础。

---

## 8. PDF 书签 / 前置

### 8.1 目录 PDF 书签
- **改动**：`book.tex` 在 `\frontmatter` 后、`\tableofcontents` 前加 `\pdfbookmark[0]{目录}{toc}`——PDF 大纲加「目录」一条，可直接跳转。
- **原版**：`\tableofcontents`（经 `\chapter*`）不自动产生目录页书签。
- **位置**：`book.tex`。
- **注**：`\pdfbookmark` 是 hyperref 原生（ElegantBook 已载 hyperref 并开 `bookmarksnumbered/open`），**非自定义书签功能**。详见 `LaTeX易错手册.md` §5.2。

---

## 9. 构建配置（工程工具，非模板改动·附记）

### 9.1 `.latexmkrc`
- `$pdf_mode=5`（xelatex）、`$xelatex` 带 `-interaction=nonstopmode -file-line-error -synctex=1`、`$out_dir='build'`、`$bibtex_use=2`（自动 biber）。

### 9.2 `compile.sh`（Docker）
- 源 `/src` **只读**挂载、容器内副本编译；**成功**仅回传 `build/<target>.pdf`、**失败**导出 `<target>.log` + 前 30 条 `file:line:` 错误摘要（本会话改进，见 `LaTeX易错手册.md` §8.4）。
- 分册（`vol_slam` 等）先建全集产 `book.aux` 供 xr 跨卷 `\cref`。

---

## 附：明确「未改动」声明（沿用官方原版）

以下**全部沿用 ElegantBook v4.7 原版**，本工程未覆盖：
- 页眉（页眉线、章节名页眉）、页边距 / 版心、正文与标题**字体**、**章节标题配色**与样式（chapter/section/subsection）。
- 主题（`\documentclass` 未指定 color，用默认）、定理盒（theorem/definition/… 原生外观）、代码默认 `lstset` frame。
- 封面 `\maketitle` 版式、目录 `\tableofcontents` 外观（仅加书签、补首页页码，未改排版）。

> 维护：每次新增对原版的改动，按「改动→原版→位置→原因」追加。最近：§1.4 `\paragraph` 前距 1.2ex（本会话）。
