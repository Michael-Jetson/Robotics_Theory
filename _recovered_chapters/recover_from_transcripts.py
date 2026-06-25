import json, os, glob, re

WFROOT = "/home/ziren2/.claude/projects/-home-ziren2-pengfei-Robotics-Theory/ef9f6219-d972-4281-9026-824f45a24e7d/subagents/workflows"
OUT = "/home/ziren2/pengfei/Robotics_Theory/_recovered_chapters"
targets = ["nonlinear_optimization", "slam_state_estimation", "kalman_eskf"]
CATN = re.compile(r"^\s*(\d+)\t(.*)$")

def find_blocks(o, kind, out):
    if isinstance(o, dict):
        if o.get("type") == kind: out.append(o)
        for v in o.values(): find_blocks(v, kind, out)
    elif isinstance(o, list):
        for v in o: find_blocks(v, kind, out)

def result_to_text(c):
    if isinstance(c, str): return c
    if isinstance(c, list): return "\n".join(b.get("text","") if isinstance(b,dict) else str(b) for b in c)
    return ""

def agent_ops(jf):
    id2res={}; uses=[]
    with open(jf, errors="replace") as f:
        for line in f:
            line=line.strip()
            if not line: continue
            try: obj=json.loads(line)
            except Exception: continue
            tus=[]; trs=[]
            find_blocks(obj,"tool_use",tus); find_blocks(obj,"tool_result",trs)
            for tu in tus: uses.append((tu.get("id"),tu.get("name"),tu.get("input") or {}))
            for tr in trs: id2res[tr.get("tool_use_id")]=result_to_text(tr.get("content"))
    return uses,id2res

def build(jf, target):
    uses,id2res=agent_ops(jf)
    linemap={}; phase_reads=True; base=None; edits_applied=0
    for uid,name,inp in uses:
        if os.path.basename(inp.get("file_path","") or "")!=target+".tex": continue
        if name=="Read" and phase_reads:
            for ln in (id2res.get(uid,"") or "").split("\n"):
                m=CATN.match(ln)
                if m:
                    no=int(m.group(1)); t=m.group(2)
                    if no not in linemap or len(t)>len(linemap[no]): linemap[no]=t
        elif name in ("Write","Edit"):
            if phase_reads:
                if not linemap: return None
                mx=max(linemap)
                base_gaps=[i for i in range(1,mx+1) if i not in linemap]
                base="\n".join(linemap.get(i,"") for i in range(1,mx+1))
                phase_reads=False
            if name=="Edit" and base is not None:
                old=inp.get("old_string",""); new=inp.get("new_string","")
                if old and old in base:
                    base = base.replace(old,new) if inp.get("replace_all") else base.replace(old,new,1)
                    edits_applied+=1
    if phase_reads:  # 全是 Read，无 edit
        if not linemap: return None
        mx=max(linemap)
        base_gaps=[i for i in range(1,mx+1) if i not in linemap]
        base="\n".join(linemap.get(i,"") for i in range(1,mx+1))
    return base, base_gaps, edits_applied

print("=== base(一致Read) + 重放同agent Edit ===")
for ch in targets:
    best=None
    for jf in sorted(glob.glob(WFROOT+"/wf_*/agent-*.jsonl")):
        r=build(jf,ch)
        if not r: continue
        body,gaps,ne=r
        bo=len(re.findall(r"\\begin\{",body)); eo=len(re.findall(r"\\end\{",body))
        score=(0 if gaps else 1, 1 if bo==eo else 0, body.count("\n")+1)
        if best is None or score>best[0]:
            best=(score,body,gaps,ne,bo,eo,jf.split("/")[-2])
    score,body,gaps,ne,bo,eo,run=best
    i=body.find("\\chapter{")
    if i>0: body=body[i:]
    if not body.endswith("\n"): body+="\n"
    open(os.path.join(OUT,ch+".tex"),"w").write(body)
    print("%-26s 行=%-5d gap=%d edits应用=%d begin/end=%d/%d %s" %
          (ch, body.count("\n"), len(gaps), ne, bo, eo, "CLEAN" if (not gaps and bo==eo) else "需查"))
