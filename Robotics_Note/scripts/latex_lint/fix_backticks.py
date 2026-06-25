#!/usr/bin/env python3
"""Convert markdown inline-code `X` -> \verb<d>X<d> in PROSE only (skip codebox/verbatim
and skip comment portions). Picks a delimiter char not present in X."""
import sys, re, glob

VERB_ENVS = ('codebox','verbatim','lstlisting','minted','Verbatim','verbatimtab')
DELIMS = ['|','!','@','=','+','/','?',';',':','~','^']
begin_re = re.compile(r'\\begin\{([A-Za-z*]+)\}')
end_re   = re.compile(r'\\end\{([A-Za-z*]+)\}')
# single-backtick span: `content` where content has no backtick, non-empty
tick_re  = re.compile(r'`([^`\n]+?)`')

def strip_comment(line):
    # return (code_part, comment_part) splitting at first unescaped %
    out=[]; i=0
    while i<len(line):
        if line[i]=='\\':
            out.append(line[i:i+2]); i+=2; continue
        if line[i]=='%':
            return ''.join(out), line[i:]
        out.append(line[i]); i+=1
    return ''.join(out), ''

def conv(m):
    x=m.group(1)
    for d in DELIMS:
        if d not in x:
            return f'\\verb{d}{x}{d}'
    return m.group(0)  # give up (all delims present) -> leave

def process(path):
    lines=open(path,encoding='utf-8').read().split('\n')
    depth=0; n=0; changed=[]
    for idx,line in enumerate(lines):
        # update env depth from this line first (so \begin line itself is "inside")
        opens=[e for e in begin_re.findall(line) if e in VERB_ENVS]
        closes=[e for e in end_re.findall(line) if e in VERB_ENVS]
        inside = depth>0 or bool(opens)
        if not inside and '`' in line:
            code,comment=strip_comment(line)
            if '`' in code:
                new=tick_re.sub(conv,code)
                if new!=code:
                    cnt=len(tick_re.findall(code))
                    lines[idx]=new+comment
                    n+=cnt; changed.append((idx+1,cnt))
        depth += len(opens)-len(closes)
        if depth<0: depth=0
    if n:
        open(path,'w',encoding='utf-8').write('\n'.join(lines))
    return n,changed

total=0
for f in sorted(glob.glob(sys.argv[1])):
    n,ch=process(f)
    if n:
        print(f"{f.split('/')[-1]}: {n} ticks -> verb  (lines {[c[0] for c in ch]})")
        total+=n
print(f"=== TOTAL backticks converted: {total} ===")
