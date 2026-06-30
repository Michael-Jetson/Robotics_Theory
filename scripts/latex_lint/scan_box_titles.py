#!/usr/bin/env python3
"""Scan box-env titles (the {title} arg, processed as normal LaTeX) for raw specials:
unescaped _ ^ # & ~ or $-imbalance. Body is verbatim so ignored. Brace-matched extraction."""
import sys, re, glob

ENVS = ('codebox','note','finenote','insight','pitfall','keypoint','tcblisting')
open_re = re.compile(r'\\begin\{('+'|'.join(ENVS)+r')\}')

def grab_title(s, pos):
    """from index just after \begin{env}, skip optional [..], return ({title}, endidx) brace-matched."""
    i=pos
    # skip whitespace
    while i<len(s) and s[i] in ' \t': i+=1
    # optional [ ... ] (bracket-matched, may contain {})
    if i<len(s) and s[i]=='[':
        depth=0
        while i<len(s):
            if s[i]=='[': depth+=1
            elif s[i]==']':
                depth-=1
                if depth==0: i+=1; break
            i+=1
    while i<len(s) and s[i] in ' \t': i+=1
    if i>=len(s) or s[i]!='{': return None,i
    # brace-match title
    depth=0; start=i
    while i<len(s):
        if s[i]=='\\': i+=2; continue
        if s[i]=='{': depth+=1
        elif s[i]=='}':
            depth-=1
            if depth==0: return s[start+1:i], i+1
        i+=1
    return None,i

def raw_specials(title):
    issues=[]
    i=0; dollar=0
    while i<len(title):
        c=title[i]
        if c=='\\': i+=2; continue   # escaped char or macro -> skip the next char
        if c=='_': issues.append('_')
        elif c=='^': issues.append('^')
        elif c=='#': issues.append('#')
        elif c=='&': issues.append('&')
        elif c=='~': issues.append('~')
        elif c=='$': dollar+=1
        i+=1
    if dollar%2: issues.append('$-imbalance')
    return issues

for f in sorted(glob.glob(sys.argv[1])):
    text=open(f,encoding='utf-8').read()
    # work line-aware for reporting line numbers
    for m in open_re.finditer(text):
        title,end=grab_title(text,m.end())
        if title is None: continue
        iss=raw_specials(title)
        if iss:
            lineno=text.count('\n',0,m.start())+1
            print(f"{f.split('/')[-1]}:{lineno}  [{','.join(sorted(set(iss)))}]  title={title[:80]}")
