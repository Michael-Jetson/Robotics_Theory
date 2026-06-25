# latexmk 配置：XeLaTeX + biber，产物进 build/
$pdf_mode = 5;          # 5 = xelatex
$xelatex  = 'xelatex -interaction=nonstopmode -file-line-error -synctex=1 %O %S';
$out_dir  = 'build';
$bibtex_use = 2;        # 自动跑 biber/bibtex
@default_files = ('main.tex');
