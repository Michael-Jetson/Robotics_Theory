#!/usr/bin/env python3
"""按章节拆分 视觉SLAM十四讲第二版.pdf（扫描版/图像PDF）。
边界与中文名已逐页视觉核对。物理页 = 书印刷页 + 23。
物理173(印刷150 '第2部分'分篇页) 按用户要求删除，不进任何文件。"""
import fitz, os, re

SRC = "/home/gpf/下载/十四讲第二版.pdf"
OUT = "/home/gpf/Note/SimpleSLAM_Theory"

# (文件序号, 中文名, 物理起始页, 物理结束页)  —— 含首尾，1-based
CHAPTERS = [
    ("01", "预备知识",            24,  33),
    ("02", "初识SLAM",            34,  62),
    ("03", "三维空间刚体运动",      63,  93),
    ("04", "李群与李代数",          94, 117),
    ("05", "相机与图像",          118, 141),
    ("06", "非线性优化",          142, 172),   # 跳过物理173分篇页
    ("07", "视觉里程计1",         174, 227),
    ("08", "视觉里程计2",         228, 254),
    ("09", "后端1",              255, 287),
    ("10", "后端2",              288, 305),
    ("11", "回环检测",            306, 327),
    ("12", "建图",                328, 367),
    ("13", "实践设计SLAM系统",     368, 385),
    ("14", "SLAM现在与未来",       386, 399),
    ("15", "附录与参考文献",        400, 419),  # 附录A/B/C + 参考文献(用户未列, 附带)
]


def safe(name):
    return re.sub(r'[\\/:*?"<>|]+', "_", name).strip()


def main():
    doc = fitz.open(SRC)
    N = doc.page_count
    print(f"源 {N} 物理页\n")
    total = 0
    for tag, name, s, e in CHAPTERS:
        assert 1 <= s <= e <= N, f"范围非法 {tag} {name}: {s}-{e}"
        out = fitz.open()
        out.insert_pdf(doc, from_page=s - 1, to_page=e - 1)
        fn = os.path.join(OUT, f"{tag}_{safe(name)}.pdf")
        out.save(fn)
        out.close()
        sz = os.path.getsize(fn) / 1e6
        npg = e - s + 1
        total += npg
        print(f"{tag} {name:<14} 物理{s:>3}-{e:<3} {npg:>2}页  {sz:5.1f}MB  {os.path.basename(fn)}")
    doc.close()
    print(f"\n共 {len(CHAPTERS)} 文件, {total} 页 (源419, 删物理173分篇页 -> 418)")


if __name__ == "__main__":
    main()
