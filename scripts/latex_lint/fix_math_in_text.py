#!/usr/bin/env python3
"""Wrap math-only macros that appear bare in TEXT into $...$.
Skips math ($...$, \(..\), \[..\], math envs single- or multi-line), verbatim envs, comments."""
import sys, re, glob

MATH_ONLY = set("""to gets mapsto longrightarrow longleftarrow rightarrow leftarrow
Rightarrow Leftarrow leftrightarrow Leftrightarrow Longrightarrow Longleftarrow
Longleftrightarrow uparrow downarrow hookrightarrow xrightarrow xleftarrow rightsquigarrow
le ge leq geq neq ne ll gg leqslant geqslant approx approxeq equiv simeq cong propto asymp doteq triangleq
pm mp times div cdot ast star circ bullet oplus ominus otimes odot boxplus boxtimes
in notin ni subset supset subseteq supseteq subsetneq supsetneq cup cap setminus uplus sqcup sqcap
forall exists nexists nabla partial infty emptyset varnothing aleph hbar
land lor lnot neg implies impliedby iff models vdash dashv
lVert rVert lvert rvert lceil rceil lfloor rfloor
perp parallel angle triangle cdots vdots ddots
succ prec succeq preceq sim mid nmid otimes oslash bigcup bigcap bigoplus bigotimes prod coprod""".split())

MATH_ENVS = set("""equation align gather multline eqnarray math displaymath array cases split
aligned alignedat alignat IEEEeqnarray flalign dmath dgroup""".split())
VERB_ENVS = ('codebox','verbatim','lstlisting','minted','Verbatim')
begin_re=re.compile(r'\\begin\{([A-Za-z]+)\*?\}')
end_re=re.compile(r'\\end\{([A-Za-z]+)\*?\}')
macro_re=re.compile(r'\\([a-zA-Z]+)')

def wrap_line(line):
    res=[]; i=0; n=len(line); math=0; cnt=0
    while i<n:
        c=line[i]
        if c=='%': res.append(line[i:]); break
        if c=='$':
            if line[i:i+2]=='$$': res.append('$$'); i+=2; math^=1; continue
            math^=1; res.append('$'); i+=1; continue
        if c=='\\':
            two=line[i:i+2]
            if two in ('\\[','\\('): math=1; res.append(two); i+=2; continue
            if two in ('\\]','\\)'): math=0; res.append(two); i+=2; continue
            mb=begin_re.match(line,i)
            if mb and mb.group(1) in MATH_ENVS: math=1; res.append(mb.group(0)); i=mb.end(); continue
            me=end_re.match(line,i)
            if me and me.group(1) in MATH_ENVS: math=0; res.append(me.group(0)); i=me.end(); continue
            m=macro_re.match(line,i)
            if m:
                tok=m.group(0); name=m.group(1)
                if (not math) and name in MATH_ONLY:
                    res.append('$'+tok+'$'); cnt+=1; i=m.end(); continue
                res.append(tok); i=m.end(); continue
            res.append(two); i+=2; continue
        res.append(c); i+=1
    return ''.join(res),cnt

def process(path):
    lines=open(path,encoding='utf-8').read().split('\n')
    env_stack=[]; out=[]; fixes=[]
    for li,line in enumerate(lines):
        in_skip = any((e in VERB_ENVS or e in MATH_ENVS) for e in env_stack)
        if in_skip:
            out.append(line)
        else:
            nl,c=wrap_line(line)
            out.append(nl)
            if c: fixes.append((li+1,c))
        for mm in begin_re.finditer(line): env_stack.append(mm.group(1))
        for mm in end_re.finditer(line):
            nm=mm.group(1)
            for k in range(len(env_stack)-1,-1,-1):
                if env_stack[k]==nm: del env_stack[k]; break
    if fixes: open(path,'w',encoding='utf-8').write('\n'.join(out))
    return fixes

total=0
for f in sorted(glob.glob(sys.argv[1])):
    fx=process(f)
    if fx:
        c=sum(x[1] for x in fx)
        print(f"{f.split('/')[-1]}: wrapped {c}  (lines {[x[0] for x in fx][:25]})")
        total+=c
print(f"=== TOTAL math-macros wrapped: {total} ===")
