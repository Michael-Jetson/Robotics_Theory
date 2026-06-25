# 抽取留痕：《视觉SLAM十四讲》第7章 视觉里程计1

> 本文件是项目内部「抽取留痕」（非成书正文）。目标：把源材料【全量保真】抽取下来，供后续综合 agent 写成自包含书章。
> 源文件：`/home/ziren2/pengfei/Robotics_Theory/视觉SLAM十四讲_md/07_视觉里程计1.md`（共 1906 行，已完整读取）。
> 按源小节号组织；保留全部公式（LaTeX）、推导（中间代数不跳）、例题/数值例、定义/性质、表/分类、算法步骤、代码片段。

---

## 0. 记号约定（本源 vs 本书统一约定）

> 本节由抽取专员根据全章用法显式整理，方便综合时转换。

| 量 | 本源（高翔《十四讲》第7章）记号 | 本书统一约定 | 差异/转换说明 |
|---|---|---|---|
| 旋转矩阵 | $\boldsymbol R \in \mathrm{SO}(3)$ | $R \in \mathrm{SO}(3)$ | 一致（本源用粗体 $\boldsymbol R$） |
| 位姿（变换） | $\boldsymbol T \in \mathrm{SE}(3)$，李代数 $\boldsymbol\xi \in \mathfrak{se}(3)$ | $T \in \mathrm{SE}(3)$ | 一致 |
| 李代数排序 | $\boldsymbol\xi = [\boldsymbol\rho; \boldsymbol\phi]$，即**平移在前、旋转在后**（"se(3) 的定义方式是旋转在前，平移在后，则只要把雅可比的前 3 列与后 3 列对调"——见 §7.7.3 式(7.46) 后注） | $\xi=[\rho;\phi]$（平移在前、旋转在后） | **一致**（本源默认 $\rho$ 在前 $\phi$ 在后；雅可比式(7.46)前3列对应平移、后3列对应旋转） |
| 扰动方式 | **左扰动**（左乘 $\delta\boldsymbol\xi$）：BA 雅可比推导用 $\frac{\partial \boldsymbol e}{\partial \delta\boldsymbol\xi}=\lim_{\delta\boldsymbol\xi\to0}\frac{\boldsymbol e(\delta\boldsymbol\xi\oplus\boldsymbol\xi)-\boldsymbol e(\boldsymbol\xi)}{\delta\boldsymbol\xi}$，$\oplus$ 指李代数上**左乘扰动**；代码中 `pose = SE3::exp(dx) * pose`（左乘更新）、g2o 顶点 `oplusImpl` 注释 "left multiplication on SE3"。 | **右扰动为主** | **关键差异**：本源 BA/ICP 雅可比是**左扰动**结果。综合时若改右扰动需重推（雅可比形式会变，伴随 Ad 不同）。具体见 §7.7.3、§7.9.2。 |
| 旋转向量 | $\boldsymbol r$（OpenCV `solvePnP` 输出，`cv::Rodrigues` 转 $R$） | — | 罗德里格斯向量 |
| 反对称算子 | $\cdot^{\wedge}$（hat，向量→反对称矩阵）；其逆 $\cdot^{\vee}$ | $[\cdot]_\times$ / $\cdot^{\wedge}$ | 一致用 $\wedge$ |
| $\mathrm{SE}(3)$ 点扰动算子 | $(\boldsymbol{TP})^{\odot}=\begin{bmatrix}\boldsymbol I & -\boldsymbol P'^{\wedge}\\ \mathbf0^{\mathrm T}&\mathbf0^{\mathrm T}\end{bmatrix}$（odot 算子，4维齐次点对 $\delta\xi$ 的导数，见式(7.44)） | — | 本书亦用 $\odot$ 表齐次点对位姿扰动的导数 |
| 相机内参 | $\boldsymbol K=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}$ | $K$ | 一致 |
| 本质矩阵 | $\boldsymbol E=\boldsymbol t^{\wedge}\boldsymbol R$ | $E$ | 一致 |
| 基础矩阵 | $\boldsymbol F=\boldsymbol K^{-\mathrm T}\boldsymbol E\boldsymbol K^{-1}$ | $F$ | 一致 |
| 单应矩阵 | $\boldsymbol H$（Homography） | $H$ | 一致 |
| 尺度相等 | $\simeq$（equal up to a scale，尺度意义下相等） | $\simeq$ | 一致 |
| 归一化平面坐标 | $\boldsymbol x = \boldsymbol K^{-1}\boldsymbol p$（去内参后的归一化坐标） | $x$ | 一致 |
| 深度/尺度因子 | $s$（齐次→非齐次的尺度，$s_i$ 第 $i$ 点深度） | $s$ | 一致 |
| 协方差/信息矩阵 | 实践中用 `setInformation(Identity)`（信息矩阵），未引入专门字母 $\Sigma$ 作协方差（$\Sigma$ 在本章专指 SVD 的奇异值矩阵） | $\Sigma$（协方差）/ $\Omega$（信息） | 注意本章 $\Sigma$=奇异值矩阵，非协方差 |
| 四元数 | 本章未使用四元数 | Hamilton | 无差异（不涉及） |
| 运动方向约定 | 对极几何/PnP：$\boldsymbol R,\boldsymbol t$ 指 $R_{21},t_{21}$（把第 1 坐标系点转到第 2 坐标系，$s_2\boldsymbol p_2=\boldsymbol K(\boldsymbol{RP}+\boldsymbol t)$）。ICP（SVD 实现）：按 $\boldsymbol p_i=\boldsymbol R\boldsymbol p_i'+\boldsymbol t$，得到的是**第二帧到第一帧**的变换，与 PnP 部分相反。 | — | **重要**：ICP-SVD 实现里 $R,t$ 方向与 PnP/对极几何相反，综合时务必标明。 |

**特别提示（左扰动 vs 右扰动）**：本书统一以右扰动为主，而本源第7章 BA、ICP 的全部解析雅可比（式 7.42–7.48, 7.61，以及代码中的 `linearizeOplus`）均基于**左扰动**（$\boldsymbol e(\delta\boldsymbol\xi\oplus\boldsymbol\xi)$，$\oplus$ 为左乘）。综合时如需统一为右扰动，需重新推导（结果中 $-\boldsymbol P'^{\wedge}$ 等项会因伴随作用而变化）。

---

## 主要目标（源：开头）

1. 理解图像特征点的意义，并掌握在单幅图像中提取特征点及多幅图像中匹配特征点的方法。
2. 理解对极几何的原理，利用对极几何的约束，恢复图像之间的摄像机的三维运动。
3. 理解 PnP 问题，以及利用已知三维结构与图像的对应关系求解摄像机的三维运动。
4. 理解 ICP 问题，以及利用点云的匹配关系求解摄像机的三维运动。
5. 理解如何通过三角化获得二维图像上对应点的三维结构。

本讲与下一讲主要介绍两类视觉里程计常用方法：**特征点法**和**光流法**。本讲介绍：什么是特征点、如何提取和匹配特征点、如何根据配对特征点估计相机运动。

> **抽取专员注（覆盖范围）**：本章（第7章）**只含特征点法**。本【章节聚焦】要求的「直接法 / 光流 (LK) 完整推导」**不在本章**——它属于第 8 章《视觉里程计2》。本文件覆盖：特征点法/ORB、对极几何（E/F/H）八点法、三角化、PnP（DLT/P3P/BA）、ICP（SVD/BA）、前端架构（两类方法分类）。直接法/光流见第8章抽取。

---

## 7.1 特征点法

SLAM 前端 = 视觉里程计：根据相邻图像信息估计粗略相机运动，给后端提供较好初始值。
视觉里程计算法两大类：**特征点法**和**直接法**。基于特征点法的前端长久以来被认为是主流方法，优势：稳定、对光照/动态物体不敏感、比较成熟。本讲从特征点法入手：提取、匹配图像特征点 → 估计两帧间相机运动和场景结构 → 实现两帧间视觉里程计。这类算法有时称**两视图几何（Two-view geometry）**。

### 7.1.1 特征点

- 视觉里程计核心问题：如何根据图像估计相机运动。图像本身是亮度/色彩矩阵，直接从矩阵层面估计运动很困难。
- 做法：先从图像选取有代表性的点（相机视角少量变化后保持不变），能在各图像中找到相同点；再在这些点上讨论相机位姿估计与点定位。经典 SLAM 中称这些点为**路标**；视觉 SLAM 中路标指**图像特征（Feature）**。
- 图像特征 = 一组与计算任务相关的信息（维基定义[37]）。单个图像像素也是一种"特征"，但灰度值受光照、形变、材质影响严重，不稳定。
- 特征点是图像中特别的地方：角点、边缘、区块（图 7-1）。角点辨识度最强；边缘次之（沿边缘局部相似）；区块最难。角点提取算法：Harris[38]、FAST[39]、GFTT[40]（多为 2000 年前算法）。
- 单纯角点不够：远处角点近看可能不是角点；旋转后外观变化难辨认。于是设计了更稳定的局部特征：SIFT[41]、SURF[42]、ORB[43]。

**人工设计特征点应有的性质：**
1. **可重复性（Repeatability）**：相同特征可在不同图像中找到。
2. **可区别性（Distinctiveness）**：不同特征有不同表达。
3. **高效率（Efficiency）**：同一图像中特征点数量应远小于像素数量。
4. **本地性（Locality）**：特征仅与一小片图像区域相关。

**特征点构成 = 关键点（Key-point）+ 描述子（Descriptor）：**
- 关键点：特征点在图像里的位置（有些还有朝向、大小）。
- 描述子：通常是一个向量，按人为设计方式描述关键点周围像素信息。设计原则："外观相似的特征应有相似的描述子"。两个描述子在向量空间距离相近 → 认为是同样特征点。

**各特征性能对比（数值例，源 §7.1.1）：** SIFT（尺度不变特征变换 Scale-Invariant Feature Transform）最经典，考虑光照/尺度/旋转，但计算量极大；截至 2016 年普通 CPU 无法实时计算 SIFT。FAST 关键点计算特别快（注意：FAST 只有关键点、没有描述子）。**ORB（Oriented FAST and Rotated BRIEF）**改进了 FAST 不具方向性的问题，采用速度极快的二进制描述子 BRIEF（Binary Robust Independent Elementary Feature）[44]。

> **ORB / SURF / SIFT 提取约 1000 个特征点的耗时（作者论文测试，源 §7.1.1）：**
> - ORB：约 **15.3 毫秒**
> - SURF：约 **217.3 毫秒**
> - SIFT：约 **5228.7 毫秒**

GPU 加速后的 SIFT 可满足实时，但提升 SLAM 成本。结论：ORB 是质量与性能之间较好的折中，本书以 ORB 为代表介绍特征提取。

### 7.1.2 ORB 特征

ORB = 关键点（"Oriented FAST"，改进 FAST 角点）+ 描述子（BRIEF）。提取分两步：
1. **FAST 角点提取**：找出图像中"角点"。相较原版 FAST，ORB 计算了特征点主方向，为后续 BRIEF 增加旋转不变性。
2. **BRIEF 描述子**：对前一步特征点周围图像区域描述。ORB 改进 BRIEF：使用先前计算的方向信息。

#### FAST 关键点

FAST：检测局部像素灰度变化明显处，速度快。思想：若一个像素与邻域差别较大（过亮/过暗），则更可能是角点。只需比较像素亮度大小。

**检测过程（图 7-2）：**
1. 在图像中选取像素 $p$，假设其亮度为 $I_p$。
2. 设置阈值 $T$（比如 $I_p$ 的 20%）。
3. 以 $p$ 为中心，选取半径为 3 的圆上的 16 个像素点。
4. 假如圆上有**连续 $N$ 个点**亮度大于 $I_p+T$ 或小于 $I_p-T$，则 $p$ 被认为是特征点（$N$ 通常取 12，即 **FAST-12**；其他常用 $N$=9、11，称 FAST-9、FAST-11）。
5. 循环以上四步，对每一像素执行相同操作。

**FAST-12 预测试加速：** 对每个像素，直接检测邻域圆上第 **1, 5, 9, 13** 个像素的亮度。只有当这 4 个像素中有 **3 个**同时大于 $I_p+T$ 或小于 $I_p-T$ 时，当前像素才可能是角点，否则直接排除 → 大大加速。
**非极大值抑制（Non-maximal suppression）：** 原始 FAST 角点常"扎堆"，第一遍检测后用非极大值抑制，在一定区域内仅保留响应极大值的角点，避免集中。

**FAST 缺点：** 重复性不强、分布不均匀；不具方向信息；固定半径 3 → 有尺度问题（远处像角点近看不是）。

**ORB 的改进：**
- **尺度不变性**：构建图像金字塔，并在金字塔每一层检测角点。金字塔底层是原始图像，每往上一层做固定倍率缩放 → 不同分辨率图像。较小图像可看成远处场景。匹配不同层图像 → 尺度不变性。例：相机后退 → 应能在上一图像金字塔的上层与下一图像金字塔的下层中找到匹配（图 7-3）。
- **旋转（灰度质心法 Intensity Centroid，[46]）**：质心 = 以图像块灰度值为权重的中心。具体步骤：

  **步骤 1**：在小图像块 $B$ 中，定义图像块的矩为
  $$
  m_{pq}=\sum_{x,y\in B} x^{p}y^{q} I(x,y), \quad p,q=\{0,1\}.
  $$

  **步骤 2**：通过矩找到图像块的质心：
  $$
  C=\left(\frac{m_{10}}{m_{00}}, \frac{m_{01}}{m_{00}}\right).
  $$

  **步骤 3**：连接图像块几何中心 $O$ 与质心 $C$，得方向向量 $\overrightarrow{OC}$，特征点方向定义为
  $$
  \theta=\arctan(m_{01}/m_{10}).
  $$

经以上改进，FAST 具有尺度与旋转描述 → ORB 中称 **Oriented FAST**。

#### BRIEF 描述子

- BRIEF 是**二进制描述子**，描述向量由许多 0/1 组成，编码关键点附近两个随机像素（如 $p$、$q$）的大小关系：若 $p$ 比 $q$ 大取 1，否则取 0。取 128 个这样的 $p,q$ → 128 维 0/1 向量[44]。
- BRIEF 用随机选点比较，速度极快；二进制表达存储方便，适用实时图像匹配。
- 原始 BRIEF 不具旋转不变性（图像旋转时容易丢失）。ORB 在 FAST 阶段计算了关键点方向 → 利用方向信息计算旋转后的 **"Steer BRIEF"** 特征，使描述子具有较好旋转不变性。
- 综合：ORB 在平移、旋转、缩放变换下仍有良好表现；FAST+BRIEF 组合高效 → ORB 在实时 SLAM 中受欢迎。

### 7.1.3 特征匹配

- 特征匹配解决 SLAM 的**数据关联（data association）**问题：确定当前看到的路标与之前路标的对应关系。
- 误匹配广泛存在（局部特性 + 重复纹理）→ 成为视觉 SLAM 性能瓶颈。
- 设两个时刻图像：$I_t$ 中提取特征点 $x_t^m,\ m=1,\dots,M$；$I_{t+1}$ 中提取 $x_{t+1}^n,\ n=1,\dots,N$。
- **暴力匹配（Brute-Force Matcher）**：对每个 $x_t^m$ 与所有 $x_{t+1}^n$ 测量描述子距离，排序，取最近的作为匹配。
  - **浮点描述子** → 用**欧氏距离**。
  - **二进制描述子（如 BRIEF）** → 用**汉明距离（Hamming distance）**：两个二进制串不同位数的个数。
- 特征点很多时，暴力匹配运算量大（尤其匹配某帧与一张地图）→ 不满足实时性。此时**快速近似最近邻（FLANN）**更适合匹配点极多情况[47]。

---

## 7.2 实践：特征提取和匹配

### 7.2.1 OpenCV 的 ORB 特征

源码 `slambook2/ch7/orb_cv.cpp`（提取+匹配）：

```cpp
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    if (argc != 3) {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    //-- 第一步:检测Oriented FAST角点位置
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算BRIEF描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg1);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用Hamming距离
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

    //-- 第四步:匹配点对筛选
    // 计算最小距离和最大距离
    auto min_max = minmax_element(matches.begin(), matches.end(),
        [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时，即认为匹配有误。但有时最小距离会非常小，
    //所以要设置一个经验值30作为下限
    std::vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);
    return 0;
}
```

**匹配筛选准则（源）：** 当描述子距离 $>$ 两倍最小距离即认为匹配有误；最小距离可能极小，故设经验下限 **30**。即保留满足 `distance <= max(2*min_dist, 30.0)` 的匹配。这是工程经验，不一定有理论依据。

**终端输出（数值例）：**
```
% build/orb_cv 1.png 2.png
extract ORB cost = 0.0229183 seconds.
match ORB cost = 0.000751868 seconds.
-- Max dist : 95.000000
-- Min dist : 4.000000
```
说明：未筛选匹配带大量误匹配；筛选后匹配数减少但多数正确。ORB 提取花 **22.9 毫秒**（两张图），匹配花 **0.75 毫秒** → 大部分计算量在特征提取。后续运动估计仍需去除误匹配的算法。

### 7.2.2 手写 ORB 特征

源码 `slambook2/ch7/orb_self.cpp`（核心片段）。用 256 位二进制描述 = 8 个 32 位 unsigned int（`typedef vector<uint32_t> DescType`）。代码通过三角函数原理回避复杂 arctan 及 sin/cos 计算以加速；`BfMatch` 中用 SSE 指令 `_mm_popcnt_u32` 计算 unsigned int 中 1 的个数（即汉明距离）。

```cpp
typedef vector<uint32_t> DescType;
// ... 省略图片读取部分代码和测试代码
// compute the descriptor
void ComputeORB(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, vector<DescType> &descriptors) {
    const int half_patch_size = 8;
    const int half_boundary = 16;
    int bad_points = 0;
    for (auto &kp: keypoints) {
        if (kp.pt.x < half_boundary || kp.pt.y < half_boundary ||
            kp.pt.x >= img.cols - half_boundary || kp.pt.y >= img.rows - half_boundary) {
            // outside
            bad_points++;
            descriptors.push_back({});
            continue;
        }
        float m01 = 0, m10 = 0;
        for (int dx = -half_patch_size; dx < half_patch_size; ++dx) {
            for (int dy = -half_patch_size; dy < half_patch_size; ++dy) {
                uchar pixel = img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
                m01 += dx * pixel;
                m10 += dy * pixel;
            }
        }

        // angle should be arc tan(m01/m10);
        float m_sqrt = sqrt(m01 * m01 + m10 * m10);
        float sin_theta = m01 / m_sqrt;
        float cos_theta = m10 / m_sqrt;

        // compute the angle of this point
        DescType desc(8, 0);
        for (int i = 0; i < 8; i++) {
            uint32_t d = 0;
            for (int k = 0; k < 32; k++) {
                int idx_pq = i * 8 + k;
                cv::Point2f p(ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1]);
                cv::Point2f q(ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3]);

                // rotate with theta
                cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y,
                                             sin_theta * p.x + cos_theta * p.y) + kp.pt;
                cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y,
                                             sin_theta * q.x + cos_theta * q.y) + kp.pt;
                if (img.at<uchar>(pp.y, pp.x) < img.at<uchar>(qq.y, qq.x)) {
                    d |= 1 << k;
                }
            }
            desc[i] = d;
        }
        descriptors.push_back(desc);
    }
    cout << "bad/total: " << bad_points << "/" << keypoints.size() << endl;
}

// brute-force matching
void BfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2,
             vector<cv::DMatch> &matches) {
    const int d_max = 40;
    for (size_t i1 = 0; i1 < desc1.size(); ++i1) {
        if (desc1[i1].empty()) continue;
        cv::DMatch m{i1, 0, 256};
        for (size_t i2 = 0; i2 < desc2.size(); ++i2) {
            if (desc2[i2].empty()) continue;
            int distance = 0;
            for (int k = 0; k < 8; k++) {
                distance += _mm_popcnt_u32(desc1[i1][k] ^ desc2[i2][k]);
            }
            if (distance < d_max && distance < m.distance) {
                m.distance = distance;
                m.trainIdx = i2;
            }
        }
        if (m.distance < d_max) {
            matches.push_back(m);
        }
    }
}
```

> 注意（源）：旋转公式中用质心法得到 $\sin\theta=m_{01}/\sqrt{m_{01}^2+m_{10}^2}$、$\cos\theta=m_{10}/\sqrt{m_{01}^2+m_{10}^2}$，再对 BRIEF 采样模式 $(p,q)$ 旋转。匹配阈值 `d_max=40`，初始 `DMatch` 距离设 256（=描述子位数上限）。

**终端输出（数值例）：**
```
bad/total: 43/638
bad/total: 8/595
extract ORB cost = 0.00390721 seconds.
match ORB cost = 0.000862984 seconds.
matches: 51
```
说明：手写 ORB 提取仅 **3.9 毫秒**、匹配 **0.86 毫秒** → 比 OpenCV 提取加速约 **5.8 倍**。编译需 CPU 支持 SSE 指令集。进一步并行化可继续加速。

### 7.2.3 计算相机运动（前端三种情形分类）

得到匹配点对后，根据相机原理分三种情况估计运动：
1. **单目**：只知 2D 像素坐标 → 两组 2D 点估计运动 → **对极几何**解决。
2. **双目 / RGB-D（或已知距离）**：两组 3D 点估计运动 → **ICP** 解决。
3. **一组 3D、一组 2D**（已知 3D 点及其投影位置）→ **PnP** 求解。

下面从信息最少的 2D-2D 出发。

---

## 7.3 2D-2D: 对极几何

### 7.3.1 对极约束

**几何术语（图 7-9）：** 两帧 $I_1, I_2$，第一帧到第二帧运动 $R, t$，相机中心 $O_1, O_2$。$I_1$ 特征点 $p_1$ 对应 $I_2$ 特征点 $p_2$（同一空间点 $P$ 在两成像平面的投影）。
- 连线 $\overrightarrow{O_1p_1}$ 与 $\overrightarrow{O_2p_2}$ 在三维空间相交于 $P$。
- $O_1, O_2, P$ 确定一平面 → **极平面（Epipolar plane）**。
- $O_1O_2$ 连线与像平面 $I_1, I_2$ 的交点 $e_1, e_2$ → **极点（Epipoles）**；$O_1O_2$ → **基线**。
- 极平面与两像平面相交线 $l_1, l_2$ → **极线（Epipolar line）**。
- 第一帧角度看：射线 $\overrightarrow{O_1p_1}$ 是该像素可能的空间位置（射线上所有点投影到同一像素）。若不知 $P$ 位置，则第二图中连线 $\overrightarrow{e_2p_2}$（第二图极线）是 $P$ 可能投影位置（即 $\overrightarrow{O_1p_1}$ 在第二相机的投影）。匹配确定 $p_2$ → 推断 $P$ 空间位置及相机运动。无匹配则需在极线上搜索（第 12 讲）。

**代数推导：** 第一帧坐标系下设 $P$ 空间位置
$$
\boldsymbol P=[X,Y,Z]^{\mathrm T}.
$$
针孔模型，两像素点位置（式 7.1）：
$$
s_1\boldsymbol p_1=\boldsymbol K\boldsymbol P,\quad s_2\boldsymbol p_2=\boldsymbol K(\boldsymbol R\boldsymbol P+\boldsymbol t).
$$
$K$ 为内参，$R,t$ 为两坐标系相机运动（具体是 $R_{21},t_{21}$，把第一坐标系坐标转到第二坐标系）。

**齐次坐标与尺度相等（式 7.2）：** 齐次坐标下，向量等于自身乘任意非零常数，记**尺度意义下相等（equal up to a scale）**：
$$
s\boldsymbol p\simeq \boldsymbol p.
$$
于是两投影关系（式 7.3）：
$$
\boldsymbol p_1\simeq \boldsymbol K\boldsymbol P,\quad \boldsymbol p_2\simeq \boldsymbol K(\boldsymbol R\boldsymbol P+\boldsymbol t).
$$
取归一化平面坐标（式 7.4）：
$$
\boldsymbol x_1=\boldsymbol K^{-1}\boldsymbol p_1,\quad \boldsymbol x_2=\boldsymbol K^{-1}\boldsymbol p_2.
$$
代入（式 7.5）：
$$
\boldsymbol x_2\simeq \boldsymbol R\boldsymbol x_1+\boldsymbol t.
$$
两边左乘 $\boldsymbol t^{\wedge}$（即两侧与 $t$ 做外积，式 7.6）：
$$
\boldsymbol t^{\wedge}\boldsymbol x_2\simeq \boldsymbol t^{\wedge}\boldsymbol R\boldsymbol x_1.
$$
两侧左乘 $\boldsymbol x_2^{\mathrm T}$（式 7.7）：
$$
\boldsymbol x_2^{\mathrm T}\boldsymbol t^{\wedge}\boldsymbol x_2\simeq \boldsymbol x_2^{\mathrm T}\boldsymbol t^{\wedge}\boldsymbol R\boldsymbol x_1.
$$
**关键观察**：左侧 $\boldsymbol t^{\wedge}\boldsymbol x_2$ 是与 $t$ 和 $x_2$ 都垂直的向量，再与 $x_2$ 做内积 → 0。左侧严格为零，乘任意非零常数仍为零，故 $\simeq$ 可写成等号（式 7.8）：
$$
\boldsymbol x_2^{\mathrm T}\boldsymbol t^{\wedge}\boldsymbol R\boldsymbol x_1=0.
$$
重新代入 $p_1, p_2$（式 7.9）：
$$
\boldsymbol p_2^{\mathrm T}\boldsymbol K^{-\mathrm T}\boldsymbol t^{\wedge}\boldsymbol R\boldsymbol K^{-1}\boldsymbol p_1=0.
$$

**两式称对极约束**，几何意义：$O_1, P, O_2$ 三者共面。同时含平移和旋转。定义两个矩阵：**基础矩阵（Fundamental Matrix）$F$** 和 **本质矩阵（Essential Matrix）$E$**（式 7.10）：
$$
\boldsymbol E=\boldsymbol t^{\wedge}\boldsymbol R,\quad \boldsymbol F=\boldsymbol K^{-\mathrm T}\boldsymbol E\boldsymbol K^{-1},\quad \boldsymbol x_2^{\mathrm T}\boldsymbol E\boldsymbol x_1=\boldsymbol p_2^{\mathrm T}\boldsymbol F\boldsymbol p_1=0.
$$

**相机位姿估计两步：**
1. 根据配对点像素位置求 $E$ 或 $F$。
2. 根据 $E$ 或 $F$ 求 $R, t$。

$E$ 与 $F$ 只相差相机内参（SLAM 中内参通常已知），实践常用形式更简单的 $E$。

### 7.3.2 本质矩阵

$\boldsymbol E=\boldsymbol t^{\wedge}\boldsymbol R$，$3\times3$，9 个未知数。**$E$ 的内在性质：**
- $E$ 在不同尺度下等价（对极约束等式为零，$E$ 乘任意非零常数仍满足）。
- 由 $E=t^{\wedge}R$ 可证[3]，$E$ 的**奇异值必为 $[\sigma,\sigma,0]^{\mathrm T}$** 形式（内在性质）。
- 平移和旋转各 3 自由度 → $t^{\wedge}R$ 共 6 自由度；由尺度等价性 → $E$ 实际 **5 个自由度**。

5 自由度 → 最少 5 对点求 $E$；但内在性质是非线性，估计麻烦。也可只考虑尺度等价性，用 **8 对点**估计 $E$ → 经典**八点法（Eight-point-algorithm）**[48,49]，只用 $E$ 的线性性质，可在线性代数框架求解。

**八点法推导：** 一对匹配点归一化坐标 $\boldsymbol x_1=[u_1,v_1,1]^{\mathrm T},\ \boldsymbol x_2=[u_2,v_2,1]^{\mathrm T}$。对极约束（式 7.11）：
$$
(u_2,v_2,1)\begin{pmatrix}e_1&e_2&e_3\\e_4&e_5&e_6\\e_7&e_8&e_9\end{pmatrix}\begin{pmatrix}u_1\\v_1\\1\end{pmatrix}=0.
$$
把 $E$ 展成向量：
$$
\boldsymbol e=[e_1,e_2,e_3,e_4,e_5,e_6,e_7,e_8,e_9]^{\mathrm T},
$$
对极约束写成关于 $e$ 的线性形式（式 7.12）：
$$
[u_2u_1,\ u_2v_1,\ u_2,\ v_2u_1,\ v_2v_1,\ v_2,\ u_1,\ v_1,\ 1]\cdot\boldsymbol e=0.
$$
所有点放入一个方程组（$u^i,v^i$ 表第 $i$ 个特征点，式 7.13）：
$$
\begin{pmatrix}
u_2^1u_1^1 & u_2^1v_1^1 & u_2^1 & v_2^1u_1^1 & v_2^1v_1^1 & v_2^1 & u_1^1 & v_1^1 & 1\\
u_2^2u_1^2 & u_2^2v_1^2 & u_2^2 & v_2^2u_1^2 & v_2^2v_1^2 & v_2^2 & u_1^2 & v_1^2 & 1\\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \\
u_2^8u_1^8 & u_2^8v_1^8 & u_2^8 & v_2^8u_1^8 & v_2^8v_1^8 & v_2^8 & u_1^8 & v_1^8 & 1
\end{pmatrix}
\begin{pmatrix}e_1\\e_2\\e_3\\e_4\\e_5\\e_6\\e_7\\e_8\\e_9\end{pmatrix}=0.
$$
8 个方程构成线性方程组，系数矩阵大小 $8\times9$，$\boldsymbol e$ 位于其零空间。若系数矩阵满秩（秩 8），零空间维数为 1（$e$ 构成一条线，与尺度等价性一致）→ $E$ 各元素可解。

**从 $E$ 恢复 $R, t$（SVD）：** 设 $E$ 的 SVD（式 7.14）
$$
\boldsymbol E=\boldsymbol U\boldsymbol\Sigma\boldsymbol V^{\mathrm T},
$$
$U,V$ 正交阵，$\Sigma$ 奇异值矩阵。由内在性质 $\Sigma=\mathrm{diag}(\sigma,\sigma,0)$。对任意 $E$，存在两个可能的 $t,R$（式 7.15）：
$$
\boldsymbol t_1^{\wedge}=\boldsymbol U\boldsymbol R_Z\!\left(\frac{\pi}{2}\right)\boldsymbol\Sigma\boldsymbol U^{\mathrm T},\quad \boldsymbol R_1=\boldsymbol U\boldsymbol R_Z^{\mathrm T}\!\left(\frac{\pi}{2}\right)\boldsymbol V^{\mathrm T}
$$
$$
\boldsymbol t_2^{\wedge}=\boldsymbol U\boldsymbol R_Z\!\left(-\frac{\pi}{2}\right)\boldsymbol\Sigma\boldsymbol U^{\mathrm T},\quad \boldsymbol R_2=\boldsymbol U\boldsymbol R_Z^{\mathrm T}\!\left(-\frac{\pi}{2}\right)\boldsymbol V^{\mathrm T}.
$$
其中 $R_Z(\frac{\pi}{2})$ 表示沿 $Z$ 轴旋转 $90°$ 的旋转矩阵。又因 $-E$ 与 $E$ 等价，对任一 $t$ 取负号也得同样结果 → **从 $E$ 分解到 $t,R$ 共 4 个可能解**。

**4 解的选取（图 7-10）：** 已知空间点在相机上的投影（红点），4 种可能情况中**只有一种解使 $P$ 在两相机中都具正深度**。把任意一点代入 4 种解，检测该点在两相机下深度 → 确定正确解。

**5 点法：** 利用内在性质只 5 自由度 → 最少 5 对点[50,51]，但形式复杂；通常有几十至上百对匹配点，8→5 意义不明显，故只介绍八点法。

**$E$ 的内在性质投影（式 7.16）：** 线性解出的 $E$ 奇异值不一定为 $\sigma,\sigma,0$。对八点法求得的 $E$ 做 SVD 得 $\Sigma=\mathrm{diag}(\sigma_1,\sigma_2,\sigma_3)$（设 $\sigma_1\geqslant\sigma_2\geqslant\sigma_3$），取
$$
\boldsymbol E=\boldsymbol U\,\mathrm{diag}\!\left(\frac{\sigma_1+\sigma_2}{2},\frac{\sigma_1+\sigma_2}{2},0\right)\boldsymbol V^{\mathrm T}.
$$
相当于把求出的矩阵投影到 $E$ 所在流形上。更简单做法：奇异值取 $\mathrm{diag}(1,1,0)$（因 $E$ 尺度等价，合理）。

### 7.3.3 单应矩阵

**单应矩阵（Homography）$H$**：描述两个平面间的映射关系。若场景特征点都在同一平面（墙、地面等），可用单应性进行运动估计（俯视/顶视相机常见）。

设 $I_1, I_2$ 匹配点 $p_1, p_2$，落在平面 $P$ 上，平面方程（式 7.17）：
$$
\boldsymbol n^{\mathrm T}\boldsymbol P+d=0.
$$
整理（式 7.18）：
$$
-\frac{\boldsymbol n^{\mathrm T}\boldsymbol P}{d}=1.
$$
回顾式(7.1)，**推导（中间代数不跳）**：
$$
\begin{aligned}
\boldsymbol p_2 &\simeq \boldsymbol K(\boldsymbol R\boldsymbol P+\boldsymbol t)\\
&\simeq \boldsymbol K\left(\boldsymbol R\boldsymbol P+\boldsymbol t\cdot\left(-\frac{\boldsymbol n^{\mathrm T}\boldsymbol P}{d}\right)\right)\\
&\simeq \boldsymbol K\left(\boldsymbol R-\frac{\boldsymbol t\boldsymbol n^{\mathrm T}}{d}\right)\boldsymbol P\\
&\simeq \boldsymbol K\left(\boldsymbol R-\frac{\boldsymbol t\boldsymbol n^{\mathrm T}}{d}\right)\boldsymbol K^{-1}\boldsymbol p_1.
\end{aligned}
$$
把中间部分记为 $H$（式 7.19）：
$$
\boldsymbol p_2\simeq \boldsymbol H\boldsymbol p_1.
$$
$H$ 定义与旋转、平移及平面参数有关，$3\times3$。展开（式 7.20）：
$$
\begin{pmatrix}u_2\\v_2\\1\end{pmatrix}\simeq\begin{pmatrix}h_1&h_2&h_3\\h_4&h_5&h_6\\h_7&h_8&h_9\end{pmatrix}\begin{pmatrix}u_1\\v_1\\1\end{pmatrix}.
$$
注意等号仍为 $\simeq$，$H$ 可乘任意非零常数。实际可令 $h_9=1$（非零时）。根据第 3 行去掉非零因子：
$$
u_2=\frac{h_1u_1+h_2v_1+h_3}{h_7u_1+h_8v_1+h_9},\quad
v_2=\frac{h_4u_1+h_5v_1+h_6}{h_7u_1+h_8v_1+h_9}.
$$
整理：
$$
\begin{aligned}
h_1u_1+h_2v_1+h_3-h_7u_1u_2-h_8v_1u_2 &= u_2\\
h_4u_1+h_5v_1+h_6-h_7u_1v_2-h_8v_1v_2 &= v_2.
\end{aligned}
$$
一组匹配点 → 两项约束（实际 3 个约束，但线性相关只取前两个）。自由度 8 的单应矩阵可由 **4 对匹配点**算出（非退化：不能有三点共线）。线性方程组（当 $h_9=1$，式 7.21）：
$$
\begin{pmatrix}
u_1^1 & v_1^1 & 1 & 0 & 0 & 0 & -u_1^1u_2^1 & -v_1^1u_2^1\\
0 & 0 & 0 & u_1^1 & v_1^1 & 1 & -u_1^1v_2^1 & -v_1^1v_2^1\\
u_1^2 & v_1^2 & 1 & 0 & 0 & 0 & -u_1^2u_2^2 & -v_1^2u_2^2\\
0 & 0 & 0 & u_1^2 & v_1^2 & 1 & -u_1^2v_2^2 & -v_1^2v_2^2\\
u_1^3 & v_1^3 & 1 & 0 & 0 & 0 & -u_1^3u_2^3 & -v_1^3u_2^3\\
0 & 0 & 0 & u_1^3 & v_1^3 & 1 & -u_1^3v_2^3 & -v_1^3v_2^3\\
u_1^4 & v_1^4 & 1 & 0 & 0 & 0 & -u_1^4u_2^4 & -v_1^4u_2^4\\
0 & 0 & 0 & u_1^4 & v_1^4 & 1 & -u_1^4v_2^4 & -v_1^4v_2^4
\end{pmatrix}
\begin{pmatrix}h_1\\h_2\\h_3\\h_4\\h_5\\h_6\\h_7\\h_8\end{pmatrix}
=\begin{pmatrix}u_2^1\\v_2^1\\u_2^2\\v_2^2\\u_2^3\\v_2^3\\u_2^4\\v_2^4\end{pmatrix}.
$$
此法把 $H$ 看成向量，解线性方程恢复 $H$ → **直接线性变换法（Direct Linear Transform，DLT）**。

**$H$ 的分解**：求出 $H$ 后需分解得 $R, t$。方法：数值法[52,53]、解析法[54]。与 $E$ 类似，单应分解返回 **4 组** $R, t$，同时可算出对应场景点所在平面的法向量。若已知成像地图点深度全正（相机前方）→ 排除两组解，剩两组，需更多先验信息判断（如假设场景平面法向量；若平面与相机平面平行，$n$ 理论值为 $[0,0,1]^{\mathrm T}$；**源中写作 $1^{\mathrm T}$，疑为排版问题，几何含义即沿光轴的单位法向**）。

**退化（degenerate）与 H 的意义：** 当特征点共面或相机纯旋转时，基础矩阵自由度下降 → 退化。有噪声时继续用八点法求 $F$，多余自由度主要由噪声决定。为避免退化影响，通常**同时估计 $F$ 和 $H$，选重投影误差较小者**作为最终运动估计。

---

## 7.4 实践：对极约束求解相机运动

源码 `slambook2/ch7/pose_estimation_2d2d.cpp`（片段）：

```cpp
void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, Mat &t) {
    // 相机内参，TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;
    for (int i = 0; i < (int) matches.size(); i++) {
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    //-- 计算本质矩阵
    Point2d principal_point(325.1, 249.7); //相机光心，TUM dataset标定值
    double focal_length = 521; //相机焦距，TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout << "essential_matrix is " << endl << essential_matrix << endl;

    //-- 计算单应矩阵
    //-- 但是本例中场景不是平面，单应矩阵意义不大
    Mat homography_matrix;
    homography_matrix = findHomography(points1, points2, RANSAC, 3);
    cout << "homography_matrix is " << endl << homography_matrix << endl;

    //-- 从本质矩阵中恢复旋转和平移信息
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}
```

主函数（验证 $E=t^{\wedge}R\cdot scale$、对极约束）：

```cpp
int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    //-- 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    //-- 验证E=t^R*scale
    Mat t_x =
        (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
         t.at<double>(2, 0), 0, -t.at<double>(0, 0),
         -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    cout << "t^R=" << endl << t_x * R << endl;

    //-- 验证对极约束
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (DMatch m: matches) {
        Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
    return 0;
}
```

**终端输出（数值例）：**
```
% build/pose_estimation_2d2d 1.png 2.png
-- Max dist : 95.000000
-- Min dist : 4.000000
一共找到了79组匹配点

fundamental_matrix is
[4.844484382466111e-06, 0.0001222601840188731, -0.01786737827487386;
 -0.0001174326832719333, 2.122888800459598e-05, -0.01775877156212593;
 0.01799658210895528, 0.008143605989020664, 1]
essential_matrix is
[-0.0203618550523477, -0.4007110038118445, -0.03324074249824097;
 0.3939270778216369, -0.03506401846698079, 0.5857110303721015;
 -0.006788487241438284, -0.5815434272915686, -0.01438258684486258]
homography_matrix is
[0.9497129583105288, -0.143556453147626, 31.20121878625771;
 0.04154536627445031, 0.9715568969832015, 5.306887618807696;
 -2.81813676978796e-05, 4.353702039810921e-05, 1]
R is
[0.9985961798781875, -0.05169917220143662, 0.01152671359827873;
 0.05139607508976055, 0.9983603445075083, 0.02520051547522442;
 -0.01281065954813571, -0.02457271064688495, 0.9996159607036126]
t is
[-0.8220841067933337;
 -0.03269742706405412;
 0.5684264241053522]

t^R=
[0.02879601157010516, 0.5666909361828478, 0.04700950886436416;
 -0.5570970160413605, 0.0495880104673049, -0.8283204827837456;
 0.009600370724838804, 0.8224266019846683, 0.02034004937801349]
epipolar constraint = [0.002528128704106625]
epipolar constraint = [-0.001663727901710724]
epipolar constraint = [-0.0008009088410884102]
......
```
对极约束满足精度约 $10^{-3}$ 量级。OpenCV 会检测角点深度是否为正，选出正确解。

### 讨论（源 §7.4 之后）

- $E$ 与 $F$ 相差相机内参矩阵；$E,F,H$ 都可分解出运动，但 $H$ 需假设点在平面上（本实验不成立，主要用 $E$）。
- $E$ 尺度等价 → 分解的 $t,R$ 也有尺度等价性；$R\in\mathrm{SO}(3)$ 自身有约束，故 $t$ 具一个尺度。对 $t$ 乘任意非零常数分解都成立 → 通常把 $t$ 归一化（长度 1）。

**尺度不确定性（源）：** 对 $t$ 长度归一化直接导致单目尺度不确定性。例：程序中 $t$ 第一维约 0.822，这 0.822 是米还是厘米无法确定（乘任意比例常数对极约束仍成立）。换言之，单目 SLAM 对轨迹和地图同时缩放任意倍数，图像不变。
- 单目两图 $t$ 归一化 = 固定尺度 → 称**单目 SLAM 初始化**。初始化后用 3D-2D 算运动；后续轨迹/地图单位即初始化固定的尺度。
- **单目必有一步不可避免的初始化**；初始化两图必须有一定平移。
- 另一种固定尺度的方法：令初始化时所有特征点**平均深度为 1**。相比令 $t=1$，深度归一化可控制场景规模，数值上更稳定。

**初始化的纯旋转问题（源）：** 从 $E$ 分解 $R,t$ 时，若相机纯旋转 → $t=0$ → $E=0$ → 无从求 $R$。此时可靠 $H$ 求旋转，但仅旋转时无法三角测量估计特征点空间位置 → **单目初始化不能只有纯旋转，必须有一定平移**。平移太小则位姿求解与三角化不稳定 → 失败。实践经验：左右平移而非原地旋转，更易单目初始化。

**多于 8 对点的情况（源）：** 给定点数多于 8 对（例：79 对匹配）→ 可算最小二乘解。把式(7.13)左侧系数矩阵记为 $A$（式 7.22）：
$$
\boldsymbol A\boldsymbol e=\boldsymbol 0.
$$
八点法 $A$ 为 $8\times9$；点多于 8 → 超定方程，不一定存在 $e$。通过最小化二次型求（式 7.23）：
$$
\min_{\boldsymbol e}\|\boldsymbol A\boldsymbol e\|_2^2=\min_{\boldsymbol e}\boldsymbol e^{\mathrm T}\boldsymbol A^{\mathrm T}\boldsymbol A\boldsymbol e.
$$
→ 求出最小二乘意义下的 $E$。存在误匹配时更倾向用**随机采样一致性（Random Sample Consensus，RANSAC）**而非最小二乘（RANSAC 可处理带错误匹配的数据）。

---

## 7.5 三角测量

得到运动后用相机运动估计特征点空间位置。单目仅单张图像无法获得深度 → 用**三角测量（Triangulation / 三角化）**估计地图点深度（图 7-11）。三角测量：通过不同位置观察同一路标点，从位置推断距离（高斯提出，应用于测量学/天文/地理）。

设左图为参考，右图变换矩阵 $T$，光心 $O_1, O_2$。$I_1$ 特征点 $p_1$ 对应 $I_2$ 特征点 $p_2$。理论上 $O_1p_1$ 与 $O_2p_2$ 相交于地图点 $P$；但噪声使两直线常无法相交 → 用最小二乘法求解。

设 $x_1, x_2$ 为归一化坐标，满足（式 7.24）：
$$
s_2\boldsymbol x_2=s_1\boldsymbol R\boldsymbol x_1+\boldsymbol t.
$$
已知 $R, t$，求两特征点深度 $s_1, s_2$。求 $s_1$：两侧左乘 $\boldsymbol x_2^{\wedge}$（式 7.25）：
$$
s_2\boldsymbol x_2^{\wedge}\boldsymbol x_2=0=s_1\boldsymbol x_2^{\wedge}\boldsymbol R\boldsymbol x_1+\boldsymbol x_2^{\wedge}\boldsymbol t.
$$
左侧为零，右侧可看成 $s_1$ 的方程（**注**：源行文中先说求 $s_1$，式中左乘后剩 $s_1\boldsymbol x_2^{\wedge}\boldsymbol R\boldsymbol x_1+\boldsymbol x_2^{\wedge}\boldsymbol t=0$，文字写"可根据它直接求得 $s_2$"——按几何，先消去 $x_2$ 自身项后解出 $s_1$，再回代得 $s_2$；源文字此处 $s_1/s_2$ 表述略有交错，本质是先解一个、再回代另一个）。由噪声，$R,t$ 不一定精确使式(7.25)为零，更常见做最小二乘解。

---

## 7.6 实践：三角测量

### 7.6.1 三角测量代码

源码 `slambook2/ch7/triangulation.cpp`（片段），调用 OpenCV `triangulatePoints`：

```cpp
void triangulation(
    const vector<KeyPoint> &keypoint_1,
    const vector<KeyPoint> &keypoint_2,
    const std::vector<DMatch> &matches,
    const Mat &R, const Mat &t,
    vector<Point3d> &points) {
    Mat T1 = (Mat_<float>(3, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point2f> pts_1, pts_2;
    for (DMatch m:matches) {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
    }
    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        Point3d p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}
```
main 函数中加入三角测量并画各点深度示意图（读者自行运行）。

### 7.6.2 讨论

- **纯旋转无法三角测量**：三角测量由平移得到（有平移才有对极几何三角形）；平移为零时对极约束一直为零。
- **三角化不确定性（图 7-12，三角测量的矛盾）**：平移很小时，像素不确定性导致较大深度不确定性。特征点运动一个像素 $\delta x$ → 视线角变 $\delta\theta$ → 深度值变 $\delta d$。$t$ 较大时 $\delta d$ 明显变小（同样相机分辨率下平移大三角化更精确）。可用**正弦定理**定量分析。
- 提高三角化精度两种方式：(1) 提高特征点提取精度（提高图像分辨率，但增计算成本）；(2) 增大平移量（但导致外观明显变化、特征匹配困难）。**三角化矛盾**：增大平移可能匹配失效，平移太小则精度不够 → 称"视差（parallax）"。
- 单目中需等待特征点被追踪几帧产生足够视角再三角化新增特征点深度 → **延迟三角化**[55]。原地旋转视差小 → 不好估计深度（机器人原地旋转常见）→ 可能追踪失败、尺度不正确。
- 可定量计算每个特征点位置及不确定性。假设特征点服从高斯分布并不断观测，信息正确时方差不断减小收敛 → 得**深度滤波器（Depth Filter）**（原理较复杂，后面章节详述）。

---

## 7.7 3D-2D: PnP

**PnP（Perspective-n-Point）**：求解 3D 到 2D 点对运动的方法。知道 $n$ 个 3D 空间点及其投影位置时，如何估计相机位姿。
- 2D-2D 对极几何需 8 个或以上点对（八点法），且有初始化、纯旋转、尺度问题。
- 若一张图特征点 3D 位置已知 → 最少只需 **3 个点对（+ 至少 1 个额外验证点）**即可估计运动。3D 位置可由三角化或 RGB-D 深度图确定。
- 双目/RGB-D 视觉里程计可直接用 PnP；单目必须先初始化才能用 PnP。
- 3D-2D 不需对极约束，又可在很少匹配点中获较好运动估计 → 最重要的姿态估计方法。

**PnP 求解方法分类：** P3P（3 对点）[56]、直接线性变换 DLT、EPnP（Efficient PnP）[57]、UPnP[58]；以及非线性优化（最小二乘迭代）即**光束法平差（Bundle Adjustment，BA）**。先介绍 DLT，再 BA。

### 7.7.1 直接线性变换（DLT）

问题：已知一组 3D 点位置及其在某相机的投影位置，求相机位姿。
空间点齐次坐标 $\boldsymbol P=(X,Y,Z,1)^{\mathrm T}$，在 $I_1$ 投影到特征点 $\boldsymbol x_1=(u_1,v_1,1)^{\mathrm T}$（归一化平面齐次坐标）。位姿 $R,t$ 未知。定义增广矩阵 $[\boldsymbol R|\boldsymbol t]$ 为 $3\times4$。展开（式 7.26）：
$$
s\begin{pmatrix}u_1\\v_1\\1\end{pmatrix}=\begin{pmatrix}t_1&t_2&t_3&t_4\\t_5&t_6&t_7&t_8\\t_9&t_{10}&t_{11}&t_{12}\end{pmatrix}\begin{pmatrix}X\\Y\\Z\\1\end{pmatrix}.
$$
用最后一行消去 $s$，得两约束：
$$
u_1=\frac{t_1X+t_2Y+t_3Z+t_4}{t_9X+t_{10}Y+t_{11}Z+t_{12}},\quad
v_1=\frac{t_5X+t_6Y+t_7Z+t_8}{t_9X+t_{10}Y+t_{11}Z+t_{12}}.
$$
定义 $T$ 的行向量：
$$
\boldsymbol t_1=(t_1,t_2,t_3,t_4)^{\mathrm T},\ \boldsymbol t_2=(t_5,t_6,t_7,t_8)^{\mathrm T},\ \boldsymbol t_3=(t_9,t_{10},t_{11},t_{12})^{\mathrm T},
$$
于是：
$$
\boldsymbol t_1^{\mathrm T}\boldsymbol P-\boldsymbol t_3^{\mathrm T}\boldsymbol P\, u_1=0,
$$
和
$$
\boldsymbol t_2^{\mathrm T}\boldsymbol P-\boldsymbol t_3^{\mathrm T}\boldsymbol P\, v_1=0.
$$
$t$ 是待求变量，每个特征点提供两个关于 $t$ 的线性约束。$N$ 个特征点（式 7.27）：
$$
\begin{pmatrix}
\boldsymbol P_1^{\mathrm T} & 0 & -u_1\boldsymbol P_1^{\mathrm T}\\
0 & \boldsymbol P_1^{\mathrm T} & -v_1\boldsymbol P_1^{\mathrm T}\\
\vdots & \vdots & \vdots\\
\boldsymbol P_N^{\mathrm T} & 0 & -u_N\boldsymbol P_N^{\mathrm T}\\
0 & \boldsymbol P_N^{\mathrm T} & -v_N\boldsymbol P_N^{\mathrm T}
\end{pmatrix}
\begin{pmatrix}\boldsymbol t_1\\\boldsymbol t_2\\\boldsymbol t_3\end{pmatrix}=0.
$$
$t$ 共 12 维 → 最少 **6 对匹配点**线性求解 $T$（DLT）；大于 6 对可用 SVD 求超定方程最小二乘解。

**旋转矩阵约束的处理：** DLT 把 $T$ 看成 12 未知数，忽略 $R\in\mathrm{SO}(3)$ 约束（解是一般矩阵）。平移属向量空间易办；旋转需针对 DLT 估计的 $T$ 左 $3\times3$ 块找最好旋转矩阵近似。可由 QR 分解[3,59]，或如下计算[6,60]（式 7.28）：
$$
\boldsymbol R\leftarrow(\boldsymbol R\boldsymbol R^{\mathrm T})^{-\frac12}\boldsymbol R.
$$
相当于把结果从矩阵空间重投影到 $\mathrm{SE}(3)$ 流形。
$x_1$ 用归一化平面坐标去掉 $K$ 影响（内参已知）。内参未知时也能用 PnP 估计 $K,R,t$ 三量，但未知量增多效果差。

### 7.7.2 P3P

P3P 仅用 3 对匹配点（推导借鉴文献[61]）。输入 3 对 3D-2D 匹配点：3D 点 $A,B,C$，2D 点 $a,b,c$（小写为大写在成像平面投影，图 7-13）。需一对验证点 $D$-$d$ 从可能解中选正确解。相机光心 $O$。已知 $A,B,C$ 在世界坐标系坐标（非相机坐标系）。算出 3D 点在相机坐标系坐标 → 得 3D-3D 对应点 → PnP 转 ICP 问题。

三角形对应关系（式 7.29）：
$$
\Delta Oab-\Delta OAB,\quad \Delta Obc-\Delta OBC,\quad \Delta Oac-\Delta OAC.
$$
考虑 $Oab$ 与 $OAB$，余弦定理（式 7.30）：
$$
OA^2+OB^2-2\,OA\cdot OB\cdot\cos\langle a,b\rangle=AB^2.
$$
对另两个三角形同理（式 7.31）：
$$
OA^2+OB^2-2\,OA\cdot OB\cdot\cos\langle a,b\rangle=AB^2
$$
$$
OB^2+OC^2-2\,OB\cdot OC\cdot\cos\langle b,c\rangle=BC^2
$$
$$
OA^2+OC^2-2\,OA\cdot OC\cdot\cos\langle a,c\rangle=AC^2.
$$
全体除以 $OC^2$，记 $x=OA/OC,\ y=OB/OC$（式 7.32）：
$$
x^2+y^2-2xy\cos\langle a,b\rangle=AB^2/OC^2
$$
$$
y^2+1^2-2y\cos\langle b,c\rangle=BC^2/OC^2
$$
$$
x^2+1^2-2x\cos\langle a,c\rangle=AC^2/OC^2.
$$
记 $v=AB^2/OC^2,\ uv=BC^2/OC^2,\ wv=AC^2/OC^2$（式 7.33）：
$$
x^2+y^2-2xy\cos\langle a,b\rangle-v=0
$$
$$
y^2+1^2-2y\cos\langle b,c\rangle-uv=0
$$
$$
x^2+1^2-2x\cos\langle a,c\rangle-wv=0.
$$
把第一式中 $v$ 放到一边代入后两式（式 7.34）：
$$
(1-u)y^2-ux^2-\cos\langle b,c\rangle\, y+2ux y\cos\langle a,b\rangle+1=0
$$
$$
(1-w)x^2-wy^2-\cos\langle a,c\rangle\, x+2wx y\cos\langle a,b\rangle+1=0.
$$
**已知/未知分析：** 已知 2D 图像位置 → 3 个余弦角 $\cos\langle a,b\rangle,\cos\langle b,c\rangle,\cos\langle a,c\rangle$ 已知；$u=BC^2/AB^2,\ w=AC^2/AB^2$ 由 $A,B,C$ 世界坐标算出（变换到相机坐标系比值不变）。$x,y$ 未知（随相机移动变化）→ 关于 $x,y$ 的二元二次方程（多项式方程）。求解析解需**吴消元法**（解法不展开，见文献[56]）。类似分解 $E$，方程最多 4 个解，用验证点选最可能解 → 得 $A,B,C$ 相机坐标系 3D 坐标 → 由 3D-3D 点对算 $R,t$（§7.9）。

**P3P 思路与问题：** 利用三角形相似求投影点 $a,b,c$ 相机坐标系 3D 坐标 → 转 3D-3D 位姿估计（带匹配 3D-3D 求解非常容易）。EPnP 等也采用此思路。**P3P 问题：** (1) 只用 3 点信息，配对点多于 3 组时难利用更多信息；(2) 3D/2D 点受噪声或误匹配则算法失效。→ 提出 EPnP、UPnP 等（用更多信息+迭代优化消噪）。**SLAM 常做法**：先用 P3P/EPnP 估计位姿，再构建最小二乘优化（BA）调整；相机运动连续时可假设不动或匀速，用推测值作初值优化。

### 7.7.3 最小化重投影误差求解 PnP（BA）

把 PnP 构建成关于重投影误差的非线性最小二乘问题（用第 4、5 讲知识）。线性方法先求位姿再求点位置；非线性优化把它们都当优化变量一起优化。把相机和三维点放一起最小化的问题统称 **Bundle Adjustment**。本节给两视图基本形式（较大规模 BA 见第 9 讲）。

考虑 $n$ 个三维空间点 $P$ 及其投影 $p$，求相机位姿 $R,t$（李群 $T$）。设空间点 $\boldsymbol P_i=[X_i,Y_i,Z_i]^{\mathrm T}$，投影像素坐标 $\boldsymbol u_i=[u_i,v_i]^{\mathrm T}$。像素与空间点关系（式 7.35）：
$$
s_i\begin{bmatrix}u_i\\v_i\\1\end{bmatrix}=\boldsymbol K\boldsymbol T\begin{bmatrix}X_i\\Y_i\\Z_i\\1\end{bmatrix}.
$$
矩阵形式：
$$
s_i\boldsymbol u_i=\boldsymbol K\boldsymbol T\boldsymbol P_i.
$$
（隐含齐次→非齐次转换）。误差求和构建最小二乘（式 7.36）：
$$
\boldsymbol T^{*}=\arg\min_{\boldsymbol T}\frac12\sum_{i=1}^{n}\left\|\boldsymbol u_i-\frac{1}{s_i}\boldsymbol K\boldsymbol T\boldsymbol P_i\right\|_2^2.
$$
误差项 = 3D 点投影位置 − 观测位置 → **重投影误差**。齐次坐标误差 3 维（最后一维恒为零）→ 多用非齐次坐标 → 误差 2 维（图 7-14）。

**线性化（式 7.37）：**
$$
\boldsymbol e(\boldsymbol x+\Delta\boldsymbol x)\approx \boldsymbol e(\boldsymbol x)+\boldsymbol J^{\mathrm T}\Delta\boldsymbol x.
$$
$e$ 为像素坐标误差（2 维），$x$ 为相机位姿（6 维）→ $J^{\mathrm T}$ 为 $2\times6$ 矩阵。

**推导 $J^{\mathrm T}$（左扰动模型）：** 记相机坐标系下空间点前 3 维（式 7.38）：
$$
\boldsymbol P'=(\boldsymbol T\boldsymbol P)_{1:3}=[X',Y',Z']^{\mathrm T}.
$$
相机投影模型（式 7.39）：
$$
s\boldsymbol u=\boldsymbol K\boldsymbol P'.
$$
展开（式 7.40）：
$$
\begin{bmatrix}su\\sv\\s\end{bmatrix}=\begin{bmatrix}f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}\begin{bmatrix}X'\\Y'\\Z'\end{bmatrix}.
$$
第 3 行消 $s$（$s$ 即 $P'$ 距离 $Z'$），得（式 7.41）：
$$
u=f_x\frac{X'}{Z'}+c_x,\quad v=f_y\frac{Y'}{Z'}+c_y.
$$
对 $T$ **左乘扰动** $\delta\boldsymbol\xi$，链式法则（式 7.42）：
$$
\frac{\partial\boldsymbol e}{\partial\delta\boldsymbol\xi}=\lim_{\delta\boldsymbol\xi\to0}\frac{\boldsymbol e(\delta\boldsymbol\xi\oplus\boldsymbol\xi)-\boldsymbol e(\boldsymbol\xi)}{\delta\boldsymbol\xi}=\frac{\partial\boldsymbol e}{\partial\boldsymbol P'}\frac{\partial\boldsymbol P'}{\partial\delta\boldsymbol\xi}.
$$
$\oplus$ 指李代数上的**左乘扰动**。

第一项（误差关于投影点，由式 7.41，式 7.43）：
$$
\frac{\partial\boldsymbol e}{\partial\boldsymbol P'}=-\begin{bmatrix}\frac{\partial u}{\partial X'}&\frac{\partial u}{\partial Y'}&\frac{\partial u}{\partial Z'}\\\frac{\partial v}{\partial X'}&\frac{\partial v}{\partial Y'}&\frac{\partial v}{\partial Z'}\end{bmatrix}=-\begin{bmatrix}\frac{f_x}{Z'}&0&-\frac{f_xX'}{Z'^2}\\0&\frac{f_y}{Z'}&-\frac{f_yY'}{Z'^2}\end{bmatrix}.
$$
第二项（变换后点关于李代数，由 4.3.5 节，式 7.44）：
$$
\frac{\partial(\boldsymbol T\boldsymbol P)}{\partial\delta\boldsymbol\xi}=(\boldsymbol T\boldsymbol P)^{\odot}=\begin{bmatrix}\boldsymbol I & -\boldsymbol P'^{\wedge}\\\mathbf0^{\mathrm T}&\mathbf0^{\mathrm T}\end{bmatrix}.
$$
取前 3 维（式 7.45）：
$$
\frac{\partial\boldsymbol P'}{\partial\delta\boldsymbol\xi}=[\boldsymbol I,\ -\boldsymbol P'^{\wedge}].
$$
两项相乘 → $2\times6$ 雅可比（式 7.46）：
$$
\frac{\partial\boldsymbol e}{\partial\delta\boldsymbol\xi}=-\begin{bmatrix}
\frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} & -\frac{f_xX'Y'}{Z'^2} & f_x+\frac{f_xX'^2}{Z'^2} & -\frac{f_xY'}{Z'}\\
0 & \frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2} & -f_y-\frac{f_yY'^2}{Z'^2} & \frac{f_yX'Y'}{Z'^2} & \frac{f_yX'}{Z'}
\end{bmatrix}.
$$
说明：保留负号（误差 = 观测 − 预测）；若定义"预测 − 观测"去掉负号。**若 $\mathfrak{se}(3)$ 定义为旋转在前、平移在后，则把矩阵前 3 列与后 3 列对调**（→ 印证本源默认 $\xi=[\rho;\phi]$ 平移在前）。

**误差关于空间点 $P$ 的导数（式 7.47）：**
$$
\frac{\partial\boldsymbol e}{\partial\boldsymbol P}=\frac{\partial\boldsymbol e}{\partial\boldsymbol P'}\frac{\partial\boldsymbol P'}{\partial\boldsymbol P}.
$$
按 $\boldsymbol P'=(\boldsymbol T\boldsymbol P)_{1:3}=\boldsymbol R\boldsymbol P+\boldsymbol t$，$P'$ 对 $P$ 求导只剩 $R$，于是（式 7.48）：
$$
\frac{\partial\boldsymbol e}{\partial\boldsymbol P}=-\begin{bmatrix}\frac{f_x}{Z'}&0&-\frac{f_xX'}{Z'^2}\\0&\frac{f_y}{Z'}&-\frac{f_yY'}{Z'^2}\end{bmatrix}\boldsymbol R.
$$
两个导数矩阵在优化中提供梯度方向。

---

## 7.8 实践：求解 PnP

### 7.8.1 使用 EPnP 求解位姿

用 RGB-D 深度图（`1_depth.png`）作特征点 3D 位置，避免初始化。OpenCV `solvePnP`：

```cpp
// slambook2/ch7/pose_estimation_3d2d.cpp（片段）
int main(int argc, char** argv) {
    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV的PnP求解，可选择EPNP、DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;
}
```
做法：得到配对特征点后，在第一图深度图找深度求空间位置（3D 点），以第二图像素位置为 2D 点，调 EPnP 求 PnP。

**终端输出（数值例）：**
```
% build/pose_estimation_3d2d 1.png 2.png d1.png d2.png
-- Max dist : 95.000000
-- Min dist : 4.000000
一共找到了79组匹配点
3d-2d pairs: 76
R=
[0.9978662025826269, -0.05167241613316376, 0.03991244360207524;
 0.0505958915956335, 0.998339762771668, 0.02752769192381471;
 -0.04126860182960625, -0.025449547736074, 0.998823919929363]
t=
[-0.1272259656955879;
 -0.007507297652615337;
 0.06138584177157709]
```
与 2D-2D 比较：有 3D 信息时 $R$ 几乎相同，$t$ 相差较多（引入深度信息所致）。Kinect 深度图有误差，3D 点不准。较大规模 BA 中希望位姿和所有三维特征点同时优化。

### 7.8.2 手写位姿估计（高斯牛顿 PnP）

源码 `slambook2/ch7/pose_estimation_3d2d.cpp`（片段）：

```cpp
void bundleAdjustmentGaussNewton(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose) {
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (int iter = 0; iter < iterations; iter++) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < points_3d.size(); i++) {
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
            Eigen::Vector2d e = points_2d[i] - proj;
            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z,
                0,
                fx * pc[0] * inv_z2,
                fx * pc[0] * pc[1] * inv_z2,
                -fx - fx * pc[0] * pc[0] * inv_z2,
                fx * pc[1] * inv_z,
                0,
                -fy * inv_z,
                fy * pc[1] * inv_z,
                fy + fy * pc[1] * pc[1] * inv_z2,
                -fy * pc[0] * pc[1] * inv_z2,
                -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        Vector6d dx;
        dx = H.ldlt().solve(b);

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        pose = Sophus::SE3d::exp(dx) * pose;   // 左乘更新
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
        if (dx.norm() < 1e-6) {
            // converge
            break;
        }
    }
    cout << "pose by g-n: \n" << pose.matrix() << endl;
}
```
> 注意（雅可比 J 与式 7.46 一致，但代码里 J 是按 `proj` 与误差 `e=观测-投影` 定义；第一行 6 列 + 第二行 6 列，对应平移3列+旋转3列）。更新 `pose = SE3::exp(dx) * pose` 为**左乘**（左扰动）。

### 7.8.3 使用 g2o 进行 BA 优化

图优化建模（图 7-15）：
1. **节点**：第二个相机的位姿节点 $T\in\mathrm{SE}(3)$。
2. **边**：每个 3D 点在第二相机的投影，观测方程 $\boldsymbol z_j=h(\boldsymbol T,\boldsymbol P_j)$。

第一相机位姿固定为零（画虚线，不优化）。g2o 提供 BA 节点/边（如 `g2o/types/sba/types_six_dof_expmap.h`）。本书自实现 `VertexPose` 和 `EdgeProjection`：

```cpp
/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;   // 左乘更新
    }

    virtual bool read(istream &in) override {}
    virtual bool write(ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

    virtual void computeError() override {
        const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z2 = Z * Z;
        _jacobianOplusXi
            << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
               0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }
    virtual bool read(istream &in) override {}
    virtual bool write(ostream &out) const override {}
private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};
```

构建图优化：

```cpp
void bundleAdjustmentG2O(
    const VecVector3d &points_3d,
    const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &pose) {
    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN、LM、DogLeg中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;    // 图模型
    optimizer.setAlgorithm(solver);    // 设置求解器
    optimizer.setVerbose(true);        // 打开调试输出

    // vertex
    VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    // K
    Eigen::Matrix3d K_eigen;
    K_eigen <<
        K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // edges
    int index = 1;
    for (size_t i = 0; i < points_2d.size(); ++i) {
        auto p2d = points_2d[i];
        auto p3d = points_3d[i];
        EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
    pose = vertex_pose->estimate();
}
```

**终端输出（数值例，三法对比）：**
```
./build/pose_estimation_3d2d 1.png 2.png 1_depth.png 2_depth.png
-- Max dist : 95.000000
-- Min dist : 4.000000
一共找到了79组匹配点
3d-2d pairs: 76
solve pnp in opencv cost time: 0.000332991 seconds.
R=
[0.9978662025826269, -0.05167241613316376, 0.03991244360207524;
 0.0505958915956335, 0.998339762771668, 0.02752769192381471;
 -0.04126860182960625, -0.025449547736074, 0.998823919929363]
t=
[-0.1272259656955879;
 -0.007507297652615337;
 0.06138584177157709]

calling bundle adjustment by gauss newton
iteration 0 cost=645538.1857253
iteration 1 cost=12750.239874896
iteration 2 cost=12301.774589343
iteration 3 cost=12301.427574651
iteration 4 cost=12301.426806652
pose by g-n:
0.99786618832 -0.0516873580423 0.039893448423 -0.127218696289
0.0506143671126 0.998340854865 0.0274540224544 -0.00738695798083
-0.0412462852904 -0.0253762590968 0.998826706403 0.0617019263823
0 0 0 1
solve pnp by gauss newton cost time: 0.000159492 seconds.

calling bundle adjustment by g2o
iteration= 0 chi2= 413.390599 time= 2.7291e-05 cumTime= 2.7291e-05 edges= 76 schur= 0 lambda= 79.000412 levenbergIter= 1
iteration= 1 chi2= 301.367030 time= 1.47e-05   cumTime= 4.1991e-05 edges= 76 schur= 0 lambda= 26.333471 levenbergIter= 1
iteration= 2 chi2= 301.365779 time= 1.7794e-05 cumTime= 5.9785e-05 edges= 76 schur= 0 lambda= 17.555647 levenbergIter= 1
iteration= 3 chi2= 301.365779 time= 1.4875e-05 cumTime= 7.466e-05  edges= 76 schur= 0 lambda= 11.703765 levenbergIter= 1
iteration= 4 chi2= 301.365779 time= 1.3132e-05 cumTime= 8.7792e-05 edges= 76 schur= 0 lambda= 7.802510  levenbergIter= 1
iteration= 5 chi2= 301.365779 time= 2.0379e-05 cumTime= 0.000108171 edges= 76 schur= 0 lambda= 41.613386 levenbergIter= 3
iteration= 6 chi2= 301.365779 time= 3.4186e-05 cumTime= 0.000142357 edges= 76 schur= 0 lambda= 2859650082279.672363 levenbergIter= 8
optimization costs time: 0.000763649 seconds.
pose estimated by g2o =
0.997866202583 -0.0516724161336 0.0399124436024 -0.127225965696
0.050595891596 0.998339762772 0.0275276919261 -0.00750729765631
-0.04126860183 -0.0254495477384 0.998823919929 0.0613858417711
0 0 0 1
solve pnp by g2o cost time: 0.000923095 seconds.
```
**结论：** 三者结果基本一致。耗时排序：手写高斯牛顿（0.15 ms）< OpenCV PnP < g2o；三者均 <1 ms，位姿估计不耗计算量。BA 通用，可放多幅图像位姿和空间点迭代优化，甚至整个 SLAM（规模大，主要后端用，第 10 讲）。前端通常考虑局部小型 BA，实时求解。

---

## 7.9 3D-3D: ICP

一组配对好的 3D 点（如两幅 RGB-D 图像匹配）：
$$
\boldsymbol P=\{\boldsymbol p_1,\dots,\boldsymbol p_n\},\quad \boldsymbol P'=\{\boldsymbol p_1',\dots,\boldsymbol p_n'\},
$$
求欧氏变换 $R,t$ 使
$$
\forall i,\ \boldsymbol p_i=\boldsymbol R\boldsymbol p_i'+\boldsymbol t.
$$
用**迭代最近点（Iterative Closest Point，ICP）**求解。3D-3D 位姿估计无相机模型（仅两组 3D 点变换，与相机无关）→ 激光 SLAM 也用 ICP（激光特征不丰富，无匹配关系，认距离最近两点为同一 → 故称迭代最近点）；视觉中特征点提供较好匹配 → 更简单。本章 ICP 指**已匹配两组点**间运动估计。

ICP 求解两种方式：线性代数（SVD）、非线性优化（类似 BA）。

### 7.9.1 SVD 方法

第 $i$ 对点误差项（式 7.49）：
$$
\boldsymbol e_i=\boldsymbol p_i-(\boldsymbol R\boldsymbol p_i'+\boldsymbol t).
$$
最小二乘（式 7.50）：
$$
\min_{\boldsymbol R,\boldsymbol t}\frac12\sum_{i=1}^{n}\|(\boldsymbol p_i-(\boldsymbol R\boldsymbol p_i'+\boldsymbol t))\|_2^2.
$$
定义两组点质心（无下标，式 7.51）：
$$
\boldsymbol p=\frac1n\sum_{i=1}^{n}(\boldsymbol p_i),\quad \boldsymbol p'=\frac1n\sum_{i=1}^{n}(\boldsymbol p_i').
$$
**误差函数处理（推导，中间代数不跳）：**
$$
\begin{aligned}
\frac12\sum_{i=1}^{n}\|\boldsymbol p_i-(\boldsymbol R\boldsymbol p_i'+\boldsymbol t)\|^2
&=\frac12\sum_{i=1}^{n}\|\boldsymbol p_i-\boldsymbol R\boldsymbol p_i'-\boldsymbol t-\boldsymbol p+\boldsymbol R\boldsymbol p'+\boldsymbol p-\boldsymbol R\boldsymbol p'\|^2\\
&=\frac12\sum_{i=1}^{n}\|(\boldsymbol p_i-\boldsymbol p-\boldsymbol R(\boldsymbol p_i'-\boldsymbol p'))+(\boldsymbol p-\boldsymbol R\boldsymbol p'-\boldsymbol t)\|^2\\
&=\frac12\sum_{i=1}^{n}\Big(\|\boldsymbol p_i-\boldsymbol p-\boldsymbol R(\boldsymbol p_i'-\boldsymbol p')\|^2+\|\boldsymbol p-\boldsymbol R\boldsymbol p'-\boldsymbol t\|^2\\
&\qquad +2(\boldsymbol p_i-\boldsymbol p-\boldsymbol R(\boldsymbol p_i'-\boldsymbol p'))^{\mathrm T}(\boldsymbol p-\boldsymbol R\boldsymbol p'-\boldsymbol t)\Big).
\end{aligned}
$$
交叉项中 $(\boldsymbol p_i-\boldsymbol p-\boldsymbol R(\boldsymbol p_i'-\boldsymbol p'))$ 求和后为零 → 目标函数简化（式 7.52）：
$$
\min_{\boldsymbol R,\boldsymbol t}J=\frac12\sum_{i=1}^{n}\|\boldsymbol p_i-\boldsymbol p-\boldsymbol R(\boldsymbol p_i'-\boldsymbol p')\|^2+\|\boldsymbol p-\boldsymbol R\boldsymbol p'-\boldsymbol t\|^2.
$$
左项只含 $R$；右项含 $R$ 和 $t$（只与质心相关）。求得 $R$ 后令第二项为零得 $t$。

**ICP 三步求解：**
1. 计算质心 $\boldsymbol p,\boldsymbol p'$，去质心坐标：
   $$
   \boldsymbol q_i=\boldsymbol p_i-\boldsymbol p,\quad \boldsymbol q_i'=\boldsymbol p_i'-\boldsymbol p'.
   $$
2. 计算旋转（式 7.53）：
   $$
   \boldsymbol R^{*}=\arg\min_{\boldsymbol R}\frac12\sum_{i=1}^{n}\|\boldsymbol q_i-\boldsymbol R\boldsymbol q_i'\|^2.
   $$
3. 由 $R$ 计算 $t$（式 7.54）：
   $$
   \boldsymbol t^{*}=\boldsymbol p-\boldsymbol R\boldsymbol p'.
   $$

**$R$ 的计算（展开误差项，式 7.55）：**
$$
\frac12\sum_{i=1}^{n}\|\boldsymbol q_i-\boldsymbol R\boldsymbol q_i'\|^2=\frac12\sum_{i=1}^{n}\left(\boldsymbol q_i^{\mathrm T}\boldsymbol q_i+\boldsymbol q_i'^{\mathrm T}\boldsymbol R^{\mathrm T}\boldsymbol R\boldsymbol q_i'-2\boldsymbol q_i^{\mathrm T}\boldsymbol R\boldsymbol q_i'\right).
$$
第一项与 $R$ 无关；第二项因 $R^{\mathrm T}R=I$ 亦与 $R$ 无关 → 优化目标变为（式 7.56）：
$$
\sum_{i=1}^{n}-\boldsymbol q_i^{\mathrm T}\boldsymbol R\boldsymbol q_i'=\sum_{i=1}^{n}-\operatorname{tr}(\boldsymbol R\boldsymbol q_i'\boldsymbol q_i^{\mathrm T})=-\operatorname{tr}\left(\boldsymbol R\sum_{i=1}^{n}\boldsymbol q_i'\boldsymbol q_i^{\mathrm T}\right).
$$
**SVD 求最优 $R$**（最优性证明见文献[62,63]）。定义矩阵（式 7.57）：
$$
\boldsymbol W=\sum_{i=1}^{n}\boldsymbol q_i\boldsymbol q_i'^{\mathrm T}.
$$
$W$ 为 $3\times3$，SVD 分解（式 7.58）：
$$
\boldsymbol W=\boldsymbol U\boldsymbol\Sigma\boldsymbol V^{\mathrm T}.
$$
$\Sigma$ 奇异值对角矩阵（对角元从大到小），$U,V$ 为正交（**源原文写"对角矩阵"，应为正交矩阵 orthogonal**）。$W$ 满秩时（式 7.59）：
$$
\boldsymbol R=\boldsymbol U\boldsymbol V^{\mathrm T}.
$$
求 $R$ 后按式(7.54)求 $t$。**若此时 $\det\boldsymbol R<0$，取 $-\boldsymbol R$ 作最优值。**

### 7.9.2 非线性优化方法

李代数表达位姿，目标函数（式 7.60）：
$$
\min_{\boldsymbol\xi}=\frac12\sum_{i=1}^{n}\|(\boldsymbol p_i-\exp(\boldsymbol\xi^{\wedge})\boldsymbol p_i')\|_2^2.
$$
单个误差项关于位姿导数（**左扰动模型**，式 7.61）：
$$
\frac{\partial\boldsymbol e}{\partial\delta\boldsymbol\xi}=-(\exp(\boldsymbol\xi^{\wedge})\boldsymbol p_i')^{\odot}.
$$
不断迭代找极小值。

**ICP 解的性质（重要，源 §7.9.2）：** 可证[6]，ICP 问题存在**唯一解或无穷多解**。唯一解情况下，只要找到极小值就是**全局最优值**（不会遇局部极小而非全局最小）→ ICP 可任意选初值（已匹配点求 ICP 的一大好处）。
匹配已知时此最小二乘问题有**解析解**[64-66]，无需迭代。ICP 研究者更关心匹配未知情况。介绍基于优化 ICP 的原因：RGB-D 中一像素深度可能有/无 → **混合用 PnP 和 ICP**：深度已知特征点建模 3D-3D 误差；深度未知建模 3D-2D 重投影误差 → 所有误差放同一问题，求解方便。

---

## 7.10 实践：求解 ICP

### 7.10.1 实践：SVD 方法

两幅 RGB-D 图像 → 特征匹配获两组 3D 点 → ICP 算位姿。OpenCV 无带匹配 ICP，自实现：

```cpp
// slambook2/ch7/pose_estimation_3d3d.cpp（片段）
void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t) {
    Point3f p1, p2; // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) *
             Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    if (R_.determinant() < 0) {
        R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (Mat_<double>(3, 3) <<
        R_(0, 0), R_(0, 1), R_(0, 2),
        R_(1, 0), R_(1, 1), R_(1, 2),
        R_(2, 0), R_(2, 1), R_(2, 2)
    );
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}
```
> **方向注意（源）**：推导按 $\boldsymbol p_i=\boldsymbol R\boldsymbol p_i'+\boldsymbol t$，这里 $R,t$ 是**第二帧到第一帧**的变换，与 PnP 部分相反。输出同时打印逆变换。

**终端输出（数值例）：**
```
./build/pose_estimation_3d3d 1.png 2.png 1_depth.png 2_depth.png
-- Max dist : 95.000000
-- Min dist : 4.000000
一共找到了79组匹配点
3d-3d pairs: 74
W=  11.9404 -0.567258   1.64182
   -1.79283   4.31299  -6.57615
    3.12791  -6.55815   10.8576
U=  0.474144 -0.880373 -0.0114952
   -0.460275 -0.258979  0.849163
    0.750556  0.397334  0.528006
V=  0.535211 -0.844064 -0.0332488
   -0.434767 -0.309001  0.84587
    0.724242  0.438263  0.532352
ICP via SVD results:
R = [0.9972395977366739, 0.05617039856770099, -0.04855997354553433;
     -0.05598345194682017, 0.9984181427731508, 0.005202431117423125;
     0.0487753812298326, -0.002469515369266572, 0.9988067198811421]
t = [0.1417248739257469;
     -0.05551033302525193;
     -0.03119093188273858]
R_inv = [0.9972395977366739, -0.05598345194682017, 0.0487753812298326;
         0.05617039856770099, 0.9984181427731508, -0.002469515369266572;
         -0.04855997354553433, 0.005202431117423125, 0.9988067198811421]
t_inv = [-0.1429199667309695;
         0.04738475446275858;
         0.03832465717628181]
```
比较 ICP/PnP/对极几何：信息越来越多（无深度→一图深度→两图深度）→ 深度准确时估计越准。但 Kinect 深度有噪声/数据丢失 → 丢弃无深度特征点 → ICP 可能不准；特征点丢太多可能无法运动估计。

### 7.10.2 实践：非线性优化方法

李代数优化位姿。RGB-D 每次观测路标三维位置（3D 观测）。用上一实验 `VertexPose`，定义 3D-3D 一元边：

```cpp
// slambook2/ch7/pose_estimation_3d3d.cpp
/// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    virtual void computeError() override {
        const VertexPose *pose = static_cast<const VertexPose *> (_vertices[0]);
        _error = _measurement - pose->estimate() * _point;
    }

    virtual void linearizeOplus() override {
        VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = pose->estimate();
        Eigen::Vector3d xyz_trans = T * _point;
        _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
    }

    bool read(istream &in) {}
    bool write(ostream &out) const {}

protected:
    Eigen::Vector3d _point;
};
```
> 一元边，观测量 3 维（非 2 维），内部无相机模型，只关联一个节点。雅可比为 $3\times6$，须与推导一致。前 3 列（平移）= $-\boldsymbol I$，后 3 列（旋转）= $[\boldsymbol{T}\boldsymbol{point}]^{\wedge}$（即 `SO3::hat(xyz_trans)`，对应式 7.61 左扰动结果展开）。

**终端输出（数值例）：**
```
iteration= 0 chi2= 1.811539 time= 1.7046e-05 cumTime= 1.7046e-05 edges= 74 schur= 0
iteration= 1 chi2= 1.811051 time= 1.0422e-05 cumTime= 2.7468e-05 edges= 74 schur= 0
iteration= 2 chi2= 1.811050 time= 9.589e-06  cumTime= 3.7057e-05 edges= 74 schur= 0
...中间略
iteration= 9 chi2= 1.811050 time= 9.113e-06  cumTime= 0.000100604 edges= 74 schur= 0
optimization costs time: 0.000559208 seconds.
after optimization:
T=
0.99724    0.0561704 -0.04856   0.141725
-0.0559834 0.998418   0.00520242 -0.0555103
0.0487754  -0.0024695 0.998807  -0.0311913
0 0 0 1
```
**结论：** 迭代一次后总体误差稳定 → 一次迭代即收敛；位姿结果与 SVD 几乎一模一样 → 说明 SVD 已给出解析解（本实验可认 SVD 是最优值）。
**灵活性（源）：** 本例用两图都有深度的特征点；只要一图深度确定，可用类似 PnP 误差方式加入优化。除位姿外，把空间点也作优化变量是另一种方式。问题更自由可能得其他解（如相机少转、点多移）→ 反映 BA 中希望尽可能多约束（多次观测带来更多信息，更准估计每个变量）。

---

## 7.11 小结

本讲介绍基于特征点的视觉里程计几个重要问题：
1. 特征点如何提取并匹配。
2. 如何通过 2D-2D 特征点估计相机运动。
3. 如何从 2D-2D 匹配估计点的空间位置（三角化）。
4. 3D-2D 的 PnP 问题，线性解法和 BA 解法。
5. 3D-3D 的 ICP 问题，线性解法和 BA 解法。

**省略的特殊情况讨论（源提示，留作研究）：** 对极几何中给定特征点共面/共线会怎样（单应 H 中提到共面）；PnP/ICP 中给定这样的解会怎样；算法能否识别这些特殊情况并报告解不可靠；能否给出估计 $T$ 的不确定度（见文献[3]）。

---

## 习题（源 §习题，全列）

1. 除了 ORB 特征点，你还能找到哪些特征点？请说说 SIFT 或 SURF 的原理，并对比它们与 ORB 之间的优劣。
2. 设计程序调用 OpenCV 中其他种类特征点。统计在提取 1000 个特征点时在你机器上所用时间。
3.\* 我们发现 OpenCV 提供的 ORB 特征点在图像中分布不够均匀。你是否能找到或提出让特征点分布更均匀的方法？
4. 研究 FLANN 为何能快速处理匹配问题。除 FLANN 外，还有哪些可加速匹配的手段？
5. 把演示程序使用的 EPnP 改成其他 PnP 方法，并研究它们的工作原理。
6. 在 PnP 优化中，将第一个相机的观测也考虑进来，程序应如何书写？最后结果会有何变化？
7. 在 ICP 程序中，将空间点也作为优化变量考虑进来，程序应如何书写？最后结果会有何变化？
8.\* 在特征点匹配过程中不可避免会遇到误匹配。如果把错误匹配输入到 PnP 或 ICP 中会发生怎样的情况？你能想到哪些避免误匹配的方法？
9.\* 使用 Sophus 的 SE3 类，自己设计 g2o 的节点与边，实现 PnP 和 ICP 的优化。

---

## 抽取专员补充说明（综合时注意）

1. **扰动方式**：本章全部解析雅可比（式 7.42–7.48、7.61，及代码 `linearizeOplus`、`oplusImpl`、`pose=SE3::exp(dx)*pose`）均为**左扰动 / 左乘更新**。本书统一右扰动为主，综合时需重推或显式标注差异。
2. **$\xi$ 排序**：本源默认 $\xi=[\rho;\phi]$（平移在前、旋转在后），与本书一致；式(7.46) 后注明确"若旋转在前平移在后则前后 3 列对调"。
3. **运动方向**：对极几何/PnP 的 $R,t$ 为 $R_{21},t_{21}$（1→2）；ICP-SVD 实现的 $R,t$ 为第二帧→第一帧（与 PnP 相反）——综合写公式时务必统一并标明。
4. **$\Sigma$ 多义**：本章 $\Sigma$ 专指 SVD 奇异值矩阵，不是协方差。信息矩阵在代码里用 `setInformation`，未用专门符号。
5. **未尽部分（不在本章源材料）**：直接法 / 光流 LK 的完整推导**不在第7章**，属第8章《视觉里程计2》；本文件不含。本章亦未展开：P3P 多项式方程的吴消元法解法细节（源明确略去，见文献[56]）、$E$ 奇异值为 $[\sigma,\sigma,0]$ 的证明（源引用文献[3]，未给）、ICP 最优 $R=UV^{\mathrm T}$ 的最优性证明（源引用[62,63]，未给）、单应矩阵数值/解析分解的具体公式（源引用[52,53,54]，未给）、5 点法（源引用[50,51]，未给）、深度滤波器原理（源称留到后面章节）。这些为源材料本身刻意省略/外引的内容，已逐处标注引用文献号。
