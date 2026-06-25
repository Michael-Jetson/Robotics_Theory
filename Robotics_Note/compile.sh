#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════
#  一键编译（Docker，TeX Live 容器）
#  · 用法：./compile.sh [target]    默认 target=book（全集）
#         分册：./compile.sh vol_slam | vol_planning | vol_control
#  · 源目录【只读】挂载到 /src，绝不在本地写中间文件
#  · 中间文件留容器 /tmp/work（随 --rm 丢弃），仅最终 build/<target>.pdf 回本地
#  · 编译分册时先构建 book（生成 build/book.aux），供 xr-hyper 跨卷 \cref 解析
# ════════════════════════════════════════════════════════════════
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${TEXLIVE_IMAGE:-texlive/texlive:latest}"
TARGET="${1:-book}"

# 本地 build/ 只留最终 PDF：先清空
rm -rf "$DIR/build"
mkdir -p "$DIR/build"

if docker run --rm \
  -v "$DIR":/src:ro \
  -v "$DIR/build":/out \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -e TARGET="$TARGET" \
  "$IMAGE" bash -c '
    set -e
    cp -a /src /tmp/work          # 容器内工作副本（源保持只读、干净）
    cd /tmp/work
    # 书体量增大（>2000pp + 海量代码框 + 膨胀文献 biblatex 预载）→ 默认 main_memory=5M 溢出
    # （报 TeX capacity exceeded [main memory] 于某 breakable codebox 输出例程）。
    # XeTeX 运行时从 texmf.cnf 读内存参数，无需重建 format；用本地 cnf 顶替即可。
    mkdir -p /tmp/texmf-mem
    printf "main_memory=12000000\nextra_mem_top=12000000\nextra_mem_bot=12000000\nsave_size=300000\npool_size=12000000\nbuf_size=2000000\nhash_extra=200000\n" > /tmp/texmf-mem/texmf.cnf
    export TEXMFCNF="/tmp/texmf-mem:"
    if [ "$TARGET" != "book" ]; then
      # 分册：先建全集以产出 build/book.aux 供 xr 跨卷引用
      latexmk -xelatex -interaction=nonstopmode -file-line-error book.tex || true
    fi
    if latexmk -xelatex -interaction=nonstopmode -file-line-error "$TARGET.tex"; then
      cp "/tmp/work/build/$TARGET.pdf" /out/                 # 成功：只回传 PDF（保持 build/ 干净）
    else
      rc=$?                                                  # 失败：导出 .log + 错误摘要供排查
      echo "‼ latexmk 失败 (rc=$rc) — 导出 $TARGET.log 到本地 build/"
      cp "/tmp/work/build/$TARGET.log" /out/ 2>/dev/null || true
      echo "===== 错误摘要（前 30 条 file:line:）====="
      grep -nE "^\./[^:]+\.(tex|sty):[0-9]+:" "/tmp/work/build/$TARGET.log" | head -30 || true
      exit $rc
    fi
  '; then
  echo "✅ PDF -> $DIR/build/$TARGET.pdf （中间文件留容器内，已随容器丢弃）"
else
  echo "❌ 编译失败 — 完整日志：$DIR/build/$TARGET.log（错误摘要见上）"
  exit 1
fi
