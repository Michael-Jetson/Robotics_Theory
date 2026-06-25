# LaTeX 多源合并 lint 脚本

排查「外部作者 / 草稿（markdown 习惯）成文并回主书」常见编译坑。详见 `docs/LaTeX易错手册.md` §12。
用法（仓库根目录运行）：

```bash
python3 scripts/latex_lint/find_verb_in_arg.py 'parts/P8_rl_motion_control/*.tex'   # §3.4 \verb 在命令参数内
python3 scripts/latex_lint/scan_box_titles.py  'parts/**/*.tex'                     # §3.1 codebox 标题裸 _^#&
python3 scripts/latex_lint/fix_backticks.py    'parts/P8_rl_motion_control/*.tex'   # §12.2 markdown 反引号→\verb（就地改）
python3 scripts/latex_lint/fix_math_in_text.py 'parts/*.tex'                        # §2.3 裸数学命令（检测；就地改有风险，宜先备份）
```

加 grep 类（手册 §12.3）：
- 控制字紧贴 CJK：`grep -rnP '\\[a-zA-Z]+[\x{4e00}-\x{9fff}]' parts/`
- calc 因子乘宏：`grep -rnP '\(\$[^$]*\*\\[a-zA-Z]' parts/`

> `fix_*` 会就地改文件——先 `git`/备份。`find_*`/`scan_*` 只读报告。
