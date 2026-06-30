# latexmk 配置（手动用：cd src && latexmk book.tex）。
# compile.sh 走 docker 时显式传 -outdir=/work/build，覆盖此处 out_dir。
$pdf_mode   = 5;          # 5 = xelatex
$xelatex    = 'xelatex -interaction=nonstopmode -file-line-error -synctex=1 %O %S';
$out_dir    = '../build'; # 源在 src/，中间文件落根级 build/
$bibtex_use = 2;          # 自动跑 biber
@default_files = ('book.tex');
