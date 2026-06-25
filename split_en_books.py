#!/usr/bin/env python3
"""把两本英文LaTeX书按章拆成章PDF，供 marker 逐章转 MD。
页码均为 PDF 物理页(1-based, 含首尾)。Part分隔页/Prelude 已按范围跳过。"""
import fitz, os, re

BOOKS = {
    "handbook": {
        "src": "/home/gpf/下载/main.pdf",
        "out": "/tmp/split_handbook",
        # (序号, 名, 起, 止)  —— 范围 = 18章 + Notation + Epilogue (跳Prelude/References/索引)
        "chapters": [
            ("00", "Notation", 14, 16),
            ("01", "Factor Graphs for SLAM", 35, 68),
            ("02", "Advanced State Variable Representations", 69, 90),
            ("03", "Robustness to Data Association and Outliers", 91, 116),
            ("04", "Differentiable Optimization", 117, 134),
            ("05", "Dense Map Representations", 135, 164),
            ("06", "Certifiably Optimal Solvers", 165, 198),
            ("07", "Visual SLAM", 209, 239),
            ("08", "LiDAR SLAM", 240, 265),
            ("09", "Radar SLAM", 266, 297),
            ("10", "Event-based SLAM", 298, 319),
            ("11", "Inertial Odometry for SLAM", 320, 348),
            ("12", "Leg Odometry for SLAM", 349, 374),
            ("13", "Boosting SLAM with Deep Learning", 382, 412),
            ("14", "Differentiable Volume Rendering Maps", 413, 432),
            ("15", "Dynamic and Deformable SLAM", 433, 469),
            ("16", "Metric-Semantic SLAM", 470, 505),
            ("17", "Towards Open-World Spatial AI", 506, 536),
            ("18", "Computational Structure of Spatial AI", 537, 563),
            ("19", "Epilogue", 564, 566),
        ],
    },
    "barfoot": {
        "src": "/home/gpf/下载/barfoot_ser24.pdf",
        "out": "/tmp/split_barfoot",
        # 10正文章 + Introduction + Notation + 附录A-D (跳References/索引)
        "chapters": [
            ("00", "Notation", 19, 20),
            ("00b", "Introduction", 21, 28),
            ("01", "Primer on Probability Theory", 29, 62),
            ("02", "Linear-Gaussian Estimation", 63, 124),
            ("03", "Nonlinear Non-Gaussian Estimation", 125, 190),
            ("04", "Handling Nonidealities in Estimation", 191, 218),
            ("05", "Variational Inference", 219, 254),
            ("06", "Primer on Three-Dimensional Geometry", 257, 298),
            ("07", "Matrix Lie Groups", 299, 394),
            ("08", "Pose Estimation Problems", 397, 454),
            ("09", "Pose-and-Point Estimation Problems", 455, 474),
            ("10", "Continuous-Time Estimation", 475, 492),
            ("A", "Appendix A Matrix Primer", 495, 520),
            ("B", "Appendix B Rotation and Pose Extras", 521, 534),
            ("C", "Appendix C Miscellaneous Extras", 535, 548),
            ("D", "Appendix D Solutions to Exercises", 549, 570),
        ],
    },
}


def safe(name):
    return re.sub(r'[\\/:*?"<>|]+', "_", name).strip().replace(" ", "_")


def main():
    for key, b in BOOKS.items():
        os.makedirs(b["out"], exist_ok=True)
        doc = fitz.open(b["src"])
        N = doc.page_count
        print(f"\n### {key}  ({N} 页) -> {b['out']}")
        for tag, name, s, e in b["chapters"]:
            assert 1 <= s <= e <= N, f"{key} {tag} 范围非法 {s}-{e}/{N}"
            out = fitz.open()
            out.insert_pdf(doc, from_page=s - 1, to_page=e - 1)
            fn = os.path.join(b["out"], f"{tag}_{safe(name)}.pdf")
            out.save(fn); out.close()
            print(f"  {tag:<3} {name[:42]:<42} p{s:>3}-{e:<3} {e-s+1:>3}页")
        doc.close()


if __name__ == "__main__":
    main()
