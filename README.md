# 机器人学笔记 Robotics Note（LaTeX / ElegantBook）

标准 ElegantBook + 学习笔记批注（原生彩色盒 + 页边 todonotes）。
独立 TeX Live Docker 容器编译，不污染本机 TeX。

## 编译

```bash
./compile.sh          # 产物：build/book.pdf
```

首次需拉镜像：`docker pull texlive/texlive:latest`

## 批注用法

- **原生彩色盒**（行内）：`\begin{definition}…\end{definition}`、`theorem` / `example` / `note` …
- **页边批注**：`\pz{黄色普通批注}`、`\pzr{红色重点批注}`
- **纯文字侧注**：`\marginnote{…}`

## 结构

```
book.tex              全集主文件（P0–P4 + 附录）；vol_*.tex 分册驱动
common/preamble.tex   共享导言（宏包+样式）；parts/Pk_xxx/ 各部章；styles.tex 自定义环境
refs.bib              文献
.latexmkrc            XeLaTeX + biber 配置
compile.sh            Docker 编译脚本
```
