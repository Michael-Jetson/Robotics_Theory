# figures/ —— 绘图子系统

本书所有数学/概念图的源与产物。详细工具用法见 **`docs/绘图手册.md`**（API/案例/坑）。

## 选定工具栈（3 主力 + 2 备选）

| 角色 | 工具 | 用途 |
|------|------|------|
| 枢纽·示意图 | **TikZ/PGF + 库**（tikz-3dplot / bayesnet / pgfgantt / tikz-timing） | 因子图、重投影、罗德里格斯、时序、坐标系、框图 |
| 数据图 | **Matplotlib** | 协方差椭圆、收敛曲线、轨迹、误差 |
| 真三维 | **Asymptote** | 流形+切空间(SO3)、相机锥、复杂 3D |
| 备选·几何草稿 | GeoGebra → 导 TikZ/PDF | 拖出几何关系再导出 |
| 备选·动画（非书内） | Manim CE | 讲座视频 |

## 目录

```
figures/
  figs.sty 不在此（在项目根）   # 内联图的库打包，正文 \usepackage{figs}
  src/        # 可编辑源（进版本库）
    *.asy     # Asymptote 真三维
    *.tex     # standalone TikZ 图（自带 \documentclass{standalone}）
    *.py      # matplotlib 脚本
  pdf/        # 构建产物（也进版本库，供只读编译 \includegraphics）
  build_figures.sh
```

种子图：`so3_tangent.asy`、`rodrigues.asy`/`rodrigues.tex`、`reproj_error.tex`、`factor_graph.tex`、`deskew_timeline.tex`、`covariance_ellipse.py`。

## 两种接入方式

1. **内联**（TikZ）：正文里直接 `\begin{tikzpicture}…`，靠 `\usepackage{figs}`（preamble 已加载）提供全部库。编译期即时出图、字体自动统一。
2. **独立 PDF**（asy / matplotlib / standalone .tex）：先 `build_figures.sh` 生成 `pdf/*.pdf`，正文 `\includegraphics{figures/pdf/<name>.pdf}`。

```latex
\begin{figure}[t]\centering
  \includegraphics[width=.6\linewidth]{figures/pdf/rodrigues.pdf}
  \caption{罗德里格斯旋转的几何分解。}\label{fig:rodrigues}
\end{figure}
```

## 构建

```bash
bash figures/build_figures.sh      # 本地跑 .py + Docker 跑 .asy/.tex -> figures/pdf/
```

- 本机仅有 `python3`(matplotlib) 与 `docker`；`.asy`/`.tex` 在 `texlive/texlive:latest` 容器内编（含 asy/xelatex/ctex/各宏包，已核实）。
- **顺序**：先 `build_figures.sh`（可写预处理，生成图 PDF）→ 再 `./compile.sh`（只读源编书）。图 PDF 必须先于只读编译就位。

## 新增一张图

- 数据图 → 写 `src/foo.py`，`savefig("foo.pdf")`（cwd 即 `pdf/`）。
- 真三维 → 写 `src/foo.asy`。
- 静态示意 → 写 `src/foo.tex`（`\documentclass[border=4pt]{standalone}`）。
- 跑 `build_figures.sh`，正文 `\includegraphics{figures/pdf/foo.pdf}`。

## 约定

- **代码注释一律 ASCII**（无 Unicode 数学符号）；图内标签文本可用中文（由 ctex 渲染）。
- **可复现**：源 + 产物 PDF 都进版本库；删 `pdf/` 重跑脚本应原样重建。
- 矢量优先（PDF），勿用 PNG 截图（放大糊、破版面）。
