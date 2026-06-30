#!/usr/bin/env perl
# ════════════════════════════════════════════════════════════════
#  overfull_report.pl <logfile> [srcroot]
#  解析 latexmk/xelatex 日志里的 Overfull \hbox（横向超出页面边界：图/表/代码框/
#  长公式/不可断长串），归因到 parts/*.tex:行号，并回源文件取最近的 \chapter /
#  \section 标题——按【名】定位（tex 无硬编码章号），再标出越界对象的环境类型。
#  环境变量：MINPT(默认2，pt阈值) VBOX(默认0，置1同列 Overfull \vbox 纵向过高)
#  前提：编译时 max_print_line 调大（compile.sh 已注入），否则「(./parts/…tex」被
#        日志折行打断会导致文件归因错位。
# ════════════════════════════════════════════════════════════════
use strict; use warnings;
my ($log, $root) = @ARGV; $root //= ".";
my $MINPT = $ENV{MINPT} // 1;     # 更严格：默认 ≥1pt(≈0.35mm)即报越文字块；MINPT=0.1 抓 TeX 全部，MINPT=5 只看明显
my $VBOX  = $ENV{VBOX}  // 1;     # 默认含纵向 Overfull \vbox（超 \textheight → 侵下边距）

open my $lh, "<", $log or die "无法打开日志 $log: $!";
my ($cur, %seen);
while (my $line = <$lh>) {
  # 跟踪「最近打开的 parts 源文件」：每个 \input 文件顺序处理完才开下一个，故不需 pop
  while ($line =~ /\((\.?\/?[^()\s]*parts\/[^()\s]*\.tex)/g) { $cur = $1 }
  my $kind = $line =~ /Overfull \\hbox/ ? "hbox"
           : ($VBOX && $line =~ /Overfull \\vbox/) ? "vbox" : "";
  next unless $kind;
  next unless $line =~ /\(([\d.]+)pt too (?:wide|high)\)/;
  my $pt = $1;
  my ($x, $y) = (0, 0);
  if    ($line =~ /at lines (\d+)--(\d+)/) { ($x, $y) = ($1, $2) }
  elsif ($line =~ /at line (\d+)/)         { ($x, $y) = ($1, $1) }
  (my $f = $cur // "?") =~ s{.*?(parts/)}{$1};
  my $k = "$f\t$x\t$y\t$kind";
  $seen{$k} = $pt if !exists $seen{$k} || $seen{$k} < $pt;   # 多遍编译去重，保留最大 pt
}
close $lh;

my %cache;
sub slurp {
  my $f = shift; return $cache{$f} if exists $cache{$f};
  my @a; if (open my $fh, "<", "$root/$f") { local $/; my $c = <$fh>; close $fh; @a = split /\n/, $c; }
  $cache{$f} = \@a; return \@a;
}
sub clean {
  my $t = shift // "";
  $t =~ s/\\label\{[^}]*\}//g; $t =~ s/\\[a-zA-Z]+\*?//g;
  $t =~ s/\$[^\$]*\$//g; $t =~ s/[{}]//g; $t =~ s/^\s+|\s+$//g; return $t;
}
my %ENVLAB = (forest=>"forest树图", tikzpicture=>"TikZ图", tabularx=>"表", tabular=>"表",
  array=>"矩阵/array", verbatim=>"verbatim", codebox=>"代码框", lstlisting=>"代码框",
  minted=>"代码框", equation=>"公式", align=>"公式", figure=>"图", longtable=>"长表");
sub annotate {
  my ($f, $x, $y) = @_;
  return ("?", "—", "?") if $f eq "?" || !$x;
  my $a = slurp($f); return ("?", "—", "?") unless @$a;
  my ($chap, $sec) = ("", "");
  for (my $i = $x-1; $i >= 0 && $i <= $#$a; $i--) {
    my $l = $a->[$i] // "";
    if (!$sec && $l =~ /\\(?:sub)?section\*?\s*(?:\[[^\]]*\])?\{(.+?)\}/) { $sec = $1 }
    if ($l =~ /\\chapter\*?\s*(?:\[[^\]]*\])?\{(.+?)\}/) { $chap = $1; last }
  }
  # 越界对象环境类型：扫 [x-3 .. y] 行内的 \begin{…}
  my $ek = "正文/公式";
  my $lo = $x > 3 ? $x-3 : 0; my $hi = ($y && $y <= $#$a) ? $y : $x;
  for (my $i = $lo; $i <= $hi && $i <= $#$a; $i++) {
    if (($a->[$i] // "") =~ /\\begin\{(forest|tikzpicture|tabularx|tabular|array|verbatim|codebox|lstlisting|minted|align\*?|equation\*?|figure|longtable)\}/) {
      (my $e = $1) =~ s/\*$//; $ek = $ENVLAB{$e} // $e; last;
    }
  }
  return (clean($chap) || "?", clean($sec) || "—", $ek);
}

my @rows = grep { $seen{$_} >= $MINPT } keys %seen;
@rows = sort { $seen{$b} <=> $seen{$a} } @rows;
my ($b20, $b5, $b1, $nv) = (0, 0, 0, 0);
for my $k (@rows) { my $pt = $seen{$k}; my $kd = (split /\t/, $k)[3];
  $nv++ if $kd eq "vbox";
  if ($pt >= 20) { $b20++ } elsif ($pt >= 5) { $b5++ } else { $b1++ } }
printf "# Overfull 超界清单 — 共 %d 处 (阈值 %gpt；含 hbox 横向越右边距 + vbox 纵向越下边距)。\n", scalar @rows, $MINPT;
printf "# 严重度： ≥20pt(明显) %d ｜ 5–20pt(可见) %d ｜ %g–5pt(细微) %d ｜ 其中纵向 vbox %d 处\n", $b20, $b5, $MINPT, $b1, $nv;
print  "# 列：超界pt │ 对象 │ 源文件:行 │〔章名〕› 节名   —— 文字块=页边距内文本区，越出即列\n";
print  "# （CJK 段落 Underfull \\hbox = 中文两端对齐拉伸，benign，已排除）\n\n";
for my $k (@rows) {
  my ($f, $x, $y, $kd) = split /\t/, $k;
  my ($chap, $sec, $ek) = annotate($f, $x, $y);
  printf "%8.1fpt  %-9s  %s:%s  〔%s〕› %s\n", $seen{$k}, $ek, $f, $x, $chap, $sec;
}
printf "\n共 %d 处%s。降序，先修最宽的。\n", scalar @rows, (@rows ? "" : "（全部在界内 ✓）");
