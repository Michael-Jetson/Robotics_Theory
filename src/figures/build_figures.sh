#!/usr/bin/env bash
# build_figures.sh -- regenerate all figure PDFs into figures/pdf/.
# Split toolchain (this box has only python3 + docker locally):
#   - matplotlib (.py)        : run locally  (host python3 with matplotlib/numpy)
#   - asymptote (.asy)        : compiled in the texlive Docker image
#   - standalone LaTeX (.tex) : compiled with xelatex in the texlive Docker image
# This is a PRE-STAGE: it runs on the writable source tree and writes
# figures/pdf/*.pdf, which the read-only book compile (compile.sh) then
# \includegraphics. Commit BOTH src/ sources and pdf/ outputs -- deleting pdf/
# and re-running this script must reproduce them (reproducibility rule).
set -uo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # figures/
ROOT="$(cd "$DIR/.." && pwd)"                          # project root (figs.sty lives here)
IMAGE="${TEXLIVE_IMAGE:-texlive/texlive:latest}"
mkdir -p "$DIR/pdf"
fail=0

echo "== [1/2] matplotlib (.py) -- local python3 =="
for f in "$DIR"/src/*.py; do
  [ -e "$f" ] || continue
  b="$(basename "$f" .py)"
  echo "  -> $b.pdf"
  ( cd "$DIR/pdf" && python3 "$f" ) || { echo "  !! FAIL $b"; fail=1; }
done

echo "== [2/2] asymptote (.asy) + standalone LaTeX (.tex) -- Docker $IMAGE =="
docker run --rm \
  -v "$ROOT":/work \
  -u "$(id -u):$(id -g)" -e HOME=/tmp \
  "$IMAGE" bash -c '
    set -u
    cd /work/figures
    export TEXINPUTS="/work:"      # allow standalone figs to \usepackage{figs}
    rc=0
    for f in src/*.asy; do
      [ -e "$f" ] || continue
      b="$(basename "$f" .asy)"
      echo "  -> $b.pdf (asy)"
      # cd into pdf/ + bare basename: avoids asy out-path-with-slash bug;
      # -render=0 = headless pure-vector (no OpenGL / freeglut display needed)
      ( cd pdf && asy -f pdf -render=0 "../src/$b.asy" ) || { echo "  !! FAIL $b"; rc=1; }
    done
    rm -f pdf/*_.tex pdf/*_.dvi pdf/*_.aux pdf/*_.log pdf/*.pre 2>/dev/null || true   # asy temp
    for f in src/*.tex; do
      [ -e "$f" ] || continue
      b="$(basename "$f" .tex)"
      echo "  -> $b.pdf (xelatex)"
      if xelatex -interaction=nonstopmode -halt-on-error -output-directory=pdf "$f" >"pdf/$b.buildlog" 2>&1; then
        rm -f "pdf/$b.aux" "pdf/$b.log" "pdf/$b.out" "pdf/$b.buildlog"
      else
        echo "  !! FAIL $b (see figures/pdf/$b.buildlog)"; rc=1
      fi
    done
    exit $rc
  ' || fail=1

echo
if [ "$fail" -eq 0 ]; then
  echo "OK -- all figure PDFs are in $DIR/pdf/"
else
  echo "DONE WITH FAILURES -- check messages above (*.buildlog kept for failed LaTeX)"
fi
exit $fail
