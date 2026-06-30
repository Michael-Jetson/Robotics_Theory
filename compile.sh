#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════
#  一键编译（Docker，TeX Live 容器）—— 目录结构：
#    src/    全部源（book.tex / vol_*.tex / styles.tex / figs.sty / refs.bib / common/ parts/ figures/）
#    build/  中间文件（aux/log/toc/fdb/bbl…）持久保留 → latexmk 跨次增量复用
#    根目录  最终 PDF（book.pdf / vol_*.pdf / fast.pdf）
#  · 用法：./compile.sh [target]       默认 book
#         六卷：./compile.sh vol_math | vol_slam | vol_planning | vol_control | vol_optimal_control | vol_rl_motion
#         全部：./compile.sh all        book + 六卷一气呵成（book 只建一次供各卷 xr 跨卷 \cref）
#         草稿：./compile.sh fast <章名…>  只编指定章（跳 biber、秒级）；跨章引用显 ??（验版面/越界够用）
#  · 挂载：src/ 只读（源受保护）；root 可写仅用于把成功的 PDF cp 回根目录。
#  · 中间文件直接写 build/（持久）→ 改动小时 latexmk 自动少跑几遍。
#  · 每次抓 Overfull \hbox 超界 → build/<target>.overfull.log（按 章名›节名 + 对象类型 定位源 tex）。
#  · 仅编译成功才用新 PDF 覆盖根目录旧 PDF；失败保留旧 PDF，只在 build/ 留 <t>.log 供排查。
# ════════════════════════════════════════════════════════════════
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${TEXLIVE_IMAGE:-texlive/texlive:latest}"
TARGET="${1:-book}"
VOLS="vol_math vol_slam vol_planning vol_control vol_optimal_control vol_rl_motion"
FASTCH=""
if [ "$TARGET" = fast ]; then shift; FASTCH="$*"; [ -z "$FASTCH" ] && { echo "用法: ./compile.sh fast <章名…>（如 prac_physics prac_ecosystem）"; exit 2; }; fi

mkdir -p "$DIR/build"
# 清旧失败日志（stale .log）；旧 PDF 不动（仅成功时容器内 cp 覆盖）。
if [ "$TARGET" = all ]; then for t in book $VOLS; do rm -f "$DIR/build/$t.log"; done
elif [ "$TARGET" = fast ]; then rm -f "$DIR/build/fast.log" "$DIR/build/fast.overfull.log"
else rm -f "$DIR/build/$TARGET.log"; fi

if docker run --rm \
  -v "$DIR":/work \
  -v "$DIR/src":/work/src:ro \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -e TARGET="$TARGET" -e VOLS="$VOLS" -e FASTCH="$FASTCH" \
  "$IMAGE" bash -c '
    set -e
    cd /work/src                 # 源只读；所有产物经 -outdir 落 /work/build
    # 书体量大（>3800pp + 海量代码框 + 膨胀 biblatex 预载）→ 默认 main_memory=5M 溢出。
    # XeTeX 运行时读 texmf.cnf，用本地 cnf 顶替即可（无需重建 format）。
    # max_print_line 调大：日志不折行，Overfull 归因「(./parts/…tex」整行可解析。
    mkdir -p /tmp/texmf-mem
    printf "main_memory=12000000\nextra_mem_top=12000000\nextra_mem_bot=12000000\nsave_size=300000\npool_size=12000000\nbuf_size=2000000\nhash_extra=200000\nmax_print_line=100000\n" > /tmp/texmf-mem/texmf.cnf
    export TEXMFCNF="/tmp/texmf-mem:"
    LMK="latexmk -xelatex -interaction=nonstopmode -file-line-error -outdir=/work/build"

    # 单 target：成功则抓 overfull + cp PDF 回根目录（原地覆盖）；失败导出摘要、返回 rc。
    build_one() {
      local t="$1" rc n
      if $LMK "$t.tex"; then
        perl /work/scripts/overfull_report.pl "/work/build/$t.log" /work/src > "/work/build/$t.overfull.log" 2>/dev/null || true
        n=$(grep -cE "^[[:space:]]*[0-9.]+pt  " "/work/build/$t.overfull.log" 2>/dev/null || true)
        cp "/work/build/$t.pdf" "/work/$t.pdf"               # 成功：PDF 落根目录
        echo "✓ $t.pdf — Overfull 超界 ${n:-0} 处 → build/$t.overfull.log"
        return 0
      else
        rc=$?
        echo "‼ $t latexmk 失败 (rc=$rc) — 日志 build/$t.log"
        echo "===== $t 错误摘要（前 30 条 file:line:）====="
        grep -nE "\.(tex|sty):[0-9]+:" "/work/build/$t.log" | head -30 || true
        return $rc
      fi
    }

    if [ "$TARGET" = fast ]; then
      # 草稿：拼临时 driver（\input 相对 cwd=src 解析），单 latexmk、无 biber → 秒级。
      { printf "\\\\input{common/preamble}\n\\\\begin{document}\n"
        for stem in $FASTCH; do
          hit=$(find parts -name "$stem.tex" | head -1)
          if [ -n "$hit" ]; then printf "\\\\input{%s}\n" "$hit"; else echo "‼ 未找到章 $stem" >&2; fi
        done
        printf "\\\\end{document}\n"; } > /tmp/fast.tex
      if $LMK /tmp/fast.tex; then
        perl /work/scripts/overfull_report.pl /work/build/fast.log /work/src > /work/build/fast.overfull.log 2>/dev/null || true
        n=$(grep -cE "^[[:space:]]*[0-9.]+pt  " /work/build/fast.overfull.log 2>/dev/null || true)
        cp /work/build/fast.pdf /work/fast.pdf
        echo "✓ fast.pdf（$FASTCH）— Overfull 超界 ${n:-0} 处 → build/fast.overfull.log"
      else
        echo "‼ fast 失败 — 日志 build/fast.log"
        grep -nE "\.(tex|sty):[0-9]+:" /work/build/fast.log | head -30 || true
        exit 1
      fi
    elif [ "$TARGET" = all ]; then
      # book 先行：一次构建即产 build/book.aux 供各卷 xr 跨卷引用。失败不中断、末尾统一非零退出。
      fails=""
      build_one book || fails="$fails book"
      for v in $VOLS; do build_one "$v" || fails="$fails $v"; done
      if [ -n "$fails" ]; then echo "❌ 失败 target:$fails"; exit 1; fi
    elif [ "$TARGET" = book ]; then
      build_one book
    else
      $LMK book.tex || true        # 单卷：先建 book 产 book.aux 供 xr，再建本卷
      build_one "$TARGET"
    fi
  '; then
  if [ "$TARGET" = all ]; then
    echo "✅ all 完成 -> 根目录 PDF（中间文件在 build/）："; ls -1 "$DIR"/*.pdf 2>/dev/null | sed "s#^#   #"
  elif [ "$TARGET" = fast ]; then
    echo "✅ 草稿 -> $DIR/fast.pdf（仅 $FASTCH；跨章引用显 ??；越界见 build/fast.overfull.log）"
  else
    echo "✅ PDF -> $DIR/$TARGET.pdf （中间文件在 build/，下次增量复用）"
  fi
else
  if [ "$TARGET" = all ]; then echo "❌ all：有 target 失败 — 失败卷 .log 在 build/（旧 PDF 保留）"
  else echo "❌ 编译失败 — 日志 $DIR/build/$TARGET.log（旧 PDF 保留）"; fi
  exit 1
fi
