#!/usr/bin/env python3
"""Detect \verb / \verb* used INSIDE a command argument {...} or box optional title [...].
Such usage is illegal (\verb is fragile; arg read with fixed catcodes -> breaks)."""
import sys, re, glob

ARG_CMDS = {'emph','textbf','textit','texttt','textsf','footnote','caption',
            'paragraph','subparagraph','section','subsection','subsubsection',
            'st','sout','uline','underline','hl','item','marginpar','textrm',
            'href','text','mbox','fbox','title','author','textsc','enquote'}

def scan(path):
    hits=[]
    for lineno,raw in enumerate(open(path,encoding='utf-8'),1):
        # strip comment (unescaped %)
        line=re.sub(r'(?<!\\)%.*$','',raw)
        i=0; n=len(line)
        stack=[]      # list of (kind) for each open { : cmd-name or '{'
        bracket=0     # depth of optional-arg [ following \begin{box} or arg cmd
        pending_cmd=None
        while i<n:
            c=line[i]
            if c=='\\':
                m=re.match(r'\\([a-zA-Z@]+)\*?',line[i:])
                if m:
                    name=m.group(1)
                    if name in ('verb',):
                        # next char is delimiter
                        j=i+m.end()-i  # index after \verb(*)
                        j=i+len(m.group(0))
                        if j<n:
                            delim=line[j]
                            inside_arg=any(s in ARG_CMDS for s in stack)
                            if inside_arg:
                                ctx=[s for s in stack if s in ARG_CMDS]
                                hits.append((lineno,
                                    ('arg<'+ctx[-1]+'>'),
                                    raw.rstrip()[:140]))
                            # skip verb content
                            k=line.find(delim,j+1)
                            i=(k+1) if k!=-1 else n
                            continue
                    pending_cmd=name if name in ARG_CMDS else None
                    i+=len(m.group(0)); continue
                else:
                    i+=2; continue
            if c=='{':
                stack.append(pending_cmd if pending_cmd else '{')
                pending_cmd=None; i+=1; continue
            if c=='}':
                if stack: stack.pop()
                i+=1; continue
            if c=='[':
                # optional arg right after \begin{...} or an arg cmd -> treat as title
                bracket+=1; i+=1; continue
            if c==']':
                if bracket>0: bracket-=1
                i+=1; continue
            if not c.isspace():
                pending_cmd=None
            i+=1
    return hits

files=sorted(glob.glob(sys.argv[1]))
total=0
for f in files:
    h=scan(f)
    if h:
        print(f"\n### {f.split('/')[-1]}  ({len(h)} hits)")
        for ln,ctx,snip in h:
            print(f"  {ln}\t{ctx}\t{snip}")
        total+=len(h)
print(f"\n=== TOTAL verb-in-arg: {total} ===")
