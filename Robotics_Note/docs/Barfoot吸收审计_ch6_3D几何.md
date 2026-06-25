# Barfoot 吸收审计 — 第 6 章《三维几何基元》(Primer on Three-Dimensional Geometry，内部式号 7.x)

> 只读审计报告。审计对象：Barfoot《State Estimation for Robotics》2nd ed. 第 6 章（§7.1–§7.6），核查其内容是否已被《机器人学笔记》全量吸收、独立重写。
> 源文件：`/root/gpf/Robotics_Theory/Barfoot_SER2_md/06_Primer_on_Three-Dimensional_Geometry/06_Primer_on_Three-Dimensional_Geometry.md`（1341 行，已分块通读全文）。
> 对照范围：grep 了整个 `parts/` 树；逐节落点见下。
> 记号说明：按**数学内容**匹配，不因符号不同报缺（Barfoot 用 $\mathbf{C}_{ba}$ 记旋转、$\mathbf{T}$ 记变换、左扰动、矢部在前四元数 $[\boldsymbol\varepsilon;\eta]$；本书用 $\mathbf{R}$、右扰动、实部在前 $[w,\mathbf v]$——已确认这些是“同一内容换记号”，不计为缺口）。
> 审计日期：2026-06-18。**不重复审计 ch1–5**（另见 `docs/Barfoot吸收审计.md`）。

---

## ① 总体结论

**吸收度估计：约 92–94%。** Barfoot ch6 是“表示 + 运动学 + 传感器模型 + 3D 概率”四大块，本书把它们**拆分落到四个文件**，且在重叠处普遍做到**完整推导、独立重写、平衡多书对照**，质量很高。仅有 **2 处真正缺口（均为传感器模型）** 与 **1 处轻量缺口**。

**落点分工（Barfoot 一章 → 本书四处）：**

| Barfoot ch6 板块 | 本书落点 | 吸收状态 |
|---|---|---|
| §7.1 向量/坐标系/内积/外积/反对称算子 | `P0_math/rigid_body_motion.tex` §1 | ✅ 全量 + 超出（恒等式 I1–I6 集中证明） |
| §7.2.1–7.2.3 旋转矩阵/主轴/欧拉角/Euler 参数/四元数/Gibbs | `rigid_body_motion.tex` §2,5,6,7,8 | ✅ 全量 + 超出（多处补全 Barfoot 仅陈述的证明） |
| §7.2.4 旋转运动学（Poisson/哥氏/角速度↔矩阵/欧拉速率 S 阵） | `rigid_body_motion.tex` §9 | ✅ 全量 |
| §7.2.5 扰动旋转（含 Example 7.1） | `P0_math/lie_theory.tex` §扰动模型 | ✅ found_elsewhere（升级为右扰动/李代数版，更干净） |
| §7.3 位姿/变换矩阵/齐次坐标/机器人约定/Frenet–Serret/独轮车 | `rigid_body_motion.tex` §3,4,9 | ✅ 全量 |
| §7.4.1 透视相机（归一化/本质/基础/单应/完整模型） | `P2_slam/camera_model.tex` §对极几何 | ✅ 全量（含全部证明） |
| §7.4.2 双目（中点/左目/视差/M 阵） | `camera_model.tex` §双目 | ✅ 全量 |
| **§7.4.3 RAE / 激光（球坐标观测模型）** | **— 全树无 —** | ❌ **confirmed_missing（头号缺口）** |
| **§7.4.4 IMU（含杆臂加速度计模型 7.151–7.153）** | `P2_slam/imu_model.tex` | ⚠️ **partial_thin（陀螺/比力有，杆臂缺）** |
| §7.5 / §7.6 小结 / 习题 | 分散于各章练习 | ✅ 习题精神已吸收（部分原题作为练习出现） |

**独立性：四个对照文件均未发现 external_punt / ventriloquize / narration_dependence 违规。** 大量 `\cite{barfoot2024state}` 都是合法的“出处标注 + 本书口吻平衡多书对照”（点名 Barfoot/十四讲/Handbook/Solà 各自取法），符合 [[independence-standard]]。详见 §③。

---

## ② 逐节 gap 清单

### 缺口 A（confirmed_missing，头号）：RAE / 激光 range-azimuth-elevation 传感器模型 — Barfoot §7.4.3（式 7.143–7.147）

- **缺什么**：Barfoot 把激光雷达（lidar）建模为在球坐标下观测一个点 $P$ 的 **RAE（range–azimuth–elevation）传感器模型**，给出
  - 正向（球坐标 → 笛卡尔）：$\boldsymbol\rho=\mathbf{C}_3^\top(\alpha)\,\mathbf{C}_2^\top(-\epsilon)\,[r,0,0]^\top$，乘开即 $[x,y,z]^\top=[\,r\cos\alpha\cos\epsilon,\ r\sin\alpha\cos\epsilon,\ r\sin\epsilon\,]^\top$（式 7.144–7.145）；
  - 逆向（即真正要的传感器模型 $\mathbf{s}(\boldsymbol\rho)$）：$[r,\alpha,\epsilon]^\top=[\sqrt{x^2+y^2+z^2},\ \tan^{-1}(y/x),\ \sin^{-1}(z/\sqrt{x^2+y^2+z^2})]^\top$（式 7.146）；
  - 平面退化（$z=0,\epsilon=0$）得移动机器人最常用的 **range-bearing（距离-方位）模型** $[r,\alpha]^\top=[\sqrt{x^2+y^2},\ \tan^{-1}(y/x)]^\top$（式 7.147）。
- **Barfoot 出处**：§7.4.3，式 7.143–7.147，Figure 7.13。
- **全树核查**：grep 整个 `parts/` 树确认无此模型。`P2_slam/lidar_slam.tex` 虽是激光章，但它把 LiDAR 建模为 **ToF 测距 + range image**（`d=½cΔt`、`1800×16` 距离图像），其 ICP/LOAM/LIO 误差全部建在**点云笛卡尔坐标**上，**从未给出 RAE 球坐标观测模型 $\mathbf{s}(\boldsymbol\rho)$ 及其雅可比**。`camera_model.tex` 只在第 9 行 build-comment 里写了“RAE（待吸收）”，`rigid_body_motion.tex` 第 1077 行“延伸阅读”仅提及 Barfoot ch6 含 RAE——二者都不是正文内容。`P1_estimation` 里的 range-bearing 也未出现。
- **为何重要**：(1) RAE/range-bearing 是经典滤波 SLAM（EKF-SLAM、FastSLAM）最标准的观测模型，是把激光/声呐/雷达接入状态估计的“教科书第一例”，与相机透视模型并列为本书两大主力传感器之一；(2) 它把本章的主轴旋转矩阵 $\mathbf{C}_2,\mathbf{C}_3$ 用到了实处，是“几何工具→传感器”的关键闭环；(3) 平面 range-bearing 模型在 P3 规划/P1 滤波里本应可直接引用，现无落点。**这是 Barfoot ch6 唯一一块在全书完全没有对应物的内容。**
- **应落到**：首选 `P2_slam/lidar_slam.tex` 开头“为什么需要激光 SLAM：从测距到一致地图”节之后，新增一小节《激光的观测模型：从 RAE 到点云》——补 7.144–7.147 的正/逆推导 + 平面 range-bearing 退化 + 球坐标几何图。次选：若希望与滤波耦合，可在 `P1_estimation`（EKF/UKF 观测模型示例处）以 range-bearing 作 worked example。建议同时在 `camera_model.tex` 第 9 行把“RAE”从 build-comment 移除（已落他处）。

### 缺口 B（partial_thin，重要）：IMU 加速度计的**杆臂（lever-arm）/传感器偏置几何**模型 — Barfoot §7.4.4（式 7.151–7.153），及 7.148 / 7.154

- **缺什么**：`imu_model.tex` 以 **Forster TRO2017 预积分谱系**（+VINS/Kalibr/OpenVINS）为骨架，**假定传感器系与体系 $b$ 重合**，因此缺 Barfoot 处理“IMU 不在体心、与体系有固定偏移 $\mathbf{r}_v^{sv}$”的那条几何推导：
  - 位置关系 $\mathbf{r}_i^{si}=\mathbf{r}_i^{vi}+\mathbf{C}_{vi}^\top\mathbf{r}_v^{sv}$（式 7.151）；
  - 二次求导（用 Poisson 方程 + $\dot{\mathbf r}_v^{sv}=\mathbf 0$）得 $\ddot{\mathbf{r}}_i^{si}=\ddot{\mathbf{r}}_i^{vi}+\mathbf{C}_{vi}^\top\dot{\boldsymbol\omega}_v^{vi\wedge}\mathbf{r}_v^{sv}+\mathbf{C}_{vi}^\top\boldsymbol\omega_v^{vi\wedge}\boldsymbol\omega_v^{vi\wedge}\mathbf{r}_v^{sv}$（式 7.152，**显式给出杆臂引起的角加速度项 + 向心项**）；
  - 最终加速度计模型 $\mathbf{a}=\mathbf{C}_{sv}\big(\mathbf{C}_{vi}(\ddot{\mathbf{r}}_i^{vi}-\mathbf{g}_i)+\dot{\boldsymbol\omega}_v^{vi\wedge}\mathbf{r}_v^{sv}+\boldsymbol\omega_v^{vi\wedge}\boldsymbol\omega_v^{vi\wedge}\mathbf{r}_v^{sv}\big)$（式 7.153）；
  - 以及把 $\mathbf{a},\boldsymbol\omega$ 叠成单一 $[\mathbf a;\boldsymbol\omega]$ 6 维测量的合记模型（式 7.154），与把角加速度 $\dot{\boldsymbol\omega}_v^{vi}$ 列为状态量（式 7.148）。
- **已落地的部分（不计为缺）**：陀螺模型 $\boldsymbol\omega=\mathbf{C}_{sv}\boldsymbol\omega_v^{vi}$（7.149）→ `imu_model.tex` 的 $\tilde{\boldsymbol\omega}=\boldsymbol\omega_b^b+\mathbf b^g+\boldsymbol\eta^g$（含 OpenVINS 内参版的 $\mathbf{C}_{sv}$）；加速度计**比力**模型 $\mathbf{a}=\mathbf{C}_{si}(\ddot{\mathbf r}-\mathbf g)$（7.150）→ $\tilde{\mathbf a}=\mathbf{R}_{wb}^\top(\mathbf a^w-\mathbf g^w)+\mathbf b^a+\boldsymbol\eta^a$；高端 IMU 测地球自转 $\boldsymbol\omega_{ie}$ 的注记（Fig 7.15）→ line 872/898。这些是**同一物理量换 Forster 记号**，已自包含。
- **关键提醒**：Barfoot 7.152 的杆臂项（哥氏/向心/角加速度）其**纯运动学母式**（式 7.43–7.44 的四项加速度分解）**已在 `rigid_body_motion.tex` §9 line 736–743 完整吸收**，且该处明言“这正是 IMU 加速度计模型中‘杆臂效应’各项的来源”。**所缺的只是把这条母式落到 IMU 加速度计模型上的那一步**（7.150→7.153 的展开），并非运动学本身缺失。
- **Barfoot 出处**：§7.4.4，式 7.148、7.151–7.154。
- **为何重要**：(1) 真实 IMU 几乎从不装在体心/相机光心，杆臂效应在大角速度（手持、足式、旋翼）下不可忽略，是 VIO/INS 标定的实际工程问题；(2) 它把 §9 的哥氏母式落到具体传感器，是“运动学→IMU”的闭环示范；(3) 标定章/外参章会用到 $\{\mathbf{r}_v^{sv},\mathbf{C}_{sv}\}$ 的几何，现仅以“重合”一笔带过。属 partial_thin 而非 confirmed_missing（比力模型在、母式在，仅差杆臂展开）。
- **应落到**：`P2_slam/imu_model.tex`，在比力加速度计模型 $\tilde{\mathbf a}=\mathbf R_{wb}^\top(\mathbf a^w-\mathbf g^w)+\dots$ 之后，新增一段《传感器不在体心：杆臂效应》，把“假定 $\mathbf s\equiv\mathbf b$”的简化**显式化**并补 7.151→7.153 的两次求导（可直接引用 `rigid_body_motion.tex` §9 的 $\mathbf{C}_{12}[\ddot{\mathbf r}_2+2\boldsymbol\omega^\wedge\dot{\mathbf r}_2+\dot{\boldsymbol\omega}^\wedge\mathbf r_2+\boldsymbol\omega^\wedge\boldsymbol\omega^\wedge\mathbf r_2]$ 的母式，省去重复推导）。同时补一句“合记 $[\mathbf a;\boldsymbol\omega]$ 6 维测量”对应 7.154。

### 已 found_elsewhere（从 gap 中剔除，注明落点）

- **§7.2.5 扰动旋转 + Example 7.1（式 7.56–7.72）**：Barfoot 用欧拉角对 $\mathbf{C}(\boldsymbol\theta)\mathbf v$ 做一阶 Taylor，导出 $\mathbf{C}(\bar{\boldsymbol\theta}+\delta\boldsymbol\theta)\approx(\mathbf 1-\delta\boldsymbol\phi^\wedge)\mathbf{C}(\bar{\boldsymbol\theta})$（$\delta\boldsymbol\phi=\mathbf S\delta\boldsymbol\theta$），并以 Example 7.1（$J=\mathbf u^\top\mathbf C\mathbf v$ 线性化求雅可比）收尾。→ **`lie_theory.tex` §扰动模型（line 423–509）全量吸收并升级**：本书直接用**右扰动** $\mathbf R\,\mathrm{Exp}(\boldsymbol\varphi)$ 在李代数上求导，得 $\partial(\mathbf R\mathbf p)/\partial\boldsymbol\varphi=-(\mathbf R\mathbf p)^\wedge$（不含 $\mathbf J_l$，比 Barfoot 的欧拉角路径更干净），并显式做了“导数模型 vs 扰动模型”对照（line 433–455，正对应 Barfoot 用欧拉角避开约束的动机），左扰动版亦并列给出。这是**有意的方法升级**（Barfoot 自己在 ch7 也转向李群），不计缺口。
- **§“3D 中的概率”（把不确定度搬到旋转/位姿上）**：→ **`lie_theory.tex` §李群上的不确定度（line 574–601）**：切空间高斯 $\mathbf T=\bar{\mathbf T}\,\mathrm{Exp}(\boldsymbol\epsilon),\ \boldsymbol\epsilon\sim\mathcal N(\mathbf 0,\boldsymbol\Sigma)$、位姿复合协方差传播（用伴随 $\mathrm{Ad}(\bar{\mathbf T}_1)$）、四阶修正与“香蕉形”分布——均已落地（这本属 Barfoot ch7 范畴，本书提前在李群章处理，吸收充分）。
- **§7.4.1 透视相机全部子项**：归一化坐标（7.105–7.106）、本质矩阵 + 对极约束**含完整证明**（7.107–7.112 → `camera_model.tex` `thm:essential`，逐步保留，旋转 $\mathbf C\to\mathbf R$）、内参矩阵 $\mathbf K$（7.113）、基础矩阵 + 对极线**含证明**（7.114–7.117 → `thm:fundamental` + 对极线 insight）、完整透视模型 $\mathbf s=\mathbf P\mathbf K\tfrac1z\boldsymbol\rho$ 与投影矩阵 $\mathbf P$（7.118–7.119）、单应矩阵**含完整推导 + 纯旋转特例 + 逆**（7.120–7.134 → `thm:homography`+`cor:homography`）——**全部 found 且自包含**。
- **§7.4.2 双目**：中点模型 + M 阵 + “M 不可逆 / $v_\ell=v_r$”证明（7.135–7.139）、左目模型 + 视差 $d=u_\ell-u_r=f_ub/z$（7.140–7.142）→ `camera_model.tex` §双目（`thm:stereo`, `eq:stereo-M`, `prop:stereo-M`, `thm:stereo-backproj`）全量。
- **§7.2.4 运动学全部**：Poisson 方程 $\dot{\mathbf C}_{21}=-\boldsymbol\omega^\wedge\mathbf C_{21}$（7.45–7.46）、哥氏关系 + 四项加速度（7.39/7.43–7.44）、欧拉角速率 $\mathbf S(\theta_2,\theta_3)$ 及其逆含 $\sec\theta_2$（7.51–7.53）→ `rigid_body_motion.tex` §9 全量（`eq:poisson`, `eq:rb-coriolis-acc`, `eq:rb-eulerrate`）。
- **§7.3.3 Frenet–Serret + 独轮车**：F–S 方程（7.91–7.93）、合并为位姿运动学 $\dot{\mathbf T}=\boldsymbol\varpi^\wedge\mathbf T$ 的广义形式（7.98–7.100）、平面退化独轮车 $\dot x=v\cos\theta$ 等（7.103）→ `rigid_body_motion.tex` §9（`eq:rb-se3-kin`, `eq:rb-unicycle`）全量。
- **§7.2 各表示**：主轴矩阵（7.5–7.7）、3-1-3 与 1-2-3 欧拉角 + 万向锁（7.8–7.9）、无穷小旋转 $\mathbf 1-\boldsymbol\theta^\times$（7.10）、Euler 旋转定理 + 轴角式（7.11–7.13）、Euler 参数（7.14–7.15）、四元数 $\pm,\oplus$ 代数（7.16–7.26）、Gibbs 向量 + Rodrigues 两证（7.27–7.35）→ `rigid_body_motion.tex` §2/5/6/7/8 全量，且多处补全 Barfoot 仅“without proof”陈述的证明（Rodrigues 几何 + 级数两证、四元数→矩阵推导）。
- **§7.3.1–7.3.2 变换矩阵 / 机器人约定**：$\mathbf T$ 定义、逆 $\mathbf T_{iv}^{-1}=\mathbf T_{vi}$、复合、齐次坐标 + 尺度因子 + Möbius 史话、机器人左/右手约定与 $+\sin$ 版（7.77–7.90）→ `rigid_body_motion.tex` §3/4 全量（含 Möbius 注记、$\mathbf t_{wc}\neq-\mathbf t_{cw}$ 陷阱、Barfoot 左手版识别）。

### 轻量观察（不单列缺口，但可顺手补强）

- **本质矩阵在双目特例的对极线水平证明**（Barfoot 7.139 多项式化简到 $v_r=v_\ell$）：`camera_model.tex` `prop:stereo-M` 已有（用基础矩阵约束证），✅。
- **Barfoot 习题（§7.6）**：7.1（$\mathbf u^\wedge\mathbf v=-\mathbf v^\wedge\mathbf u$）、7.2（从轴角证 $\mathbf C^{-1}=\mathbf C^\top$）、7.3（$(\mathbf C\mathbf v)^\wedge=\mathbf C\mathbf v^\wedge\mathbf C^\top$）已作为 `rigid_body_motion.tex` 练习/附录出现；7.4–7.5（Frenet 平面化）、7.6（直线投影为直线）已在正文/相机章 insight 出现；7.10（激光建图位姿链）、7.11（含地球自转 IMU）**未作为练习落地**——与缺口 A/B 同源，补完 A/B 后可顺带补这两道题。

---

## ③ 独立性问题清单

**结论：四个对照文件（`rigid_body_motion.tex` / `lie_theory.tex` / `camera_model.tex` / `imu_model.tex` / `notation.tex`）均未发现违反 [[independence-standard]] 的写法。** 逐类核查：

- **external_punt（把内容推给原书）**：未发现。涉及 Barfoot 的地方内容都**就地补全**：相机章证明标“（保 Barfoot 全步骤，旋转 $\mathbf C\to\mathbf R$）”但**全步骤在场**；`rigid_body_motion.tex` 对 Barfoot“仅陈述不证”的 Rodrigues/正交性**反而补出本书证明**。唯一形似 punt 的是 `lie_theory.tex` 附录把两条**极冗长**闭式（$\mathbf Q_l$ 四阶项、Barfoot 8.91）放附录并“见 §附录”——但这是**章内自引**且附录确有该式，不算外部 punt。
- **ventriloquize（借原书之口“Barfoot 坦言/强调/指出”）**：未发现。涉及 Barfoot 的句子都是**本书直述 + `\cite`**（如“Barfoot 用 vectrix 给出被动型的等价推导”是中性的方法归属，非“Barfoot 强调/认为”式转述）。
- **narration_dependence（以单一源书作叙述载体）**：未发现单书依赖。相反，重叠处普遍是**平衡多书对照**的范例——例如 `rigid_body_motion.tex` 的“三书帧记号对照表”（本书/Barfoot/Handbook 并列）、四元数“Hamilton vs JPL + Barfoot 矢部在前”对照、Poisson 方程“本书/Barfoot/Handbook 三式符号差异来自三要素组合”、扰动“十四讲、Barfoot 多用左扰动；本书右扰动”。这正是 standard 鼓励的写法。

**唯一可议的细节（非违规，提请注意）**：`rigid_body_motion.tex` line 1077“延伸阅读”把 Barfoot ch6 描述为含“相机/立体/RAE/IMU 传感器模型”，但 **RAE 本书并未落地**（缺口 A）。这不是独立性问题，而是**延伸阅读与正文不一致**——补完缺口 A 后此条自洽；在补完之前，该行不构成 punt（它是延伸阅读、非正文推内容），但读者可能据此误以为正文有 RAE。建议补 A 时一并复核此行。

---

## ④ 最该补的项（按“对完备性贡献 × 工作量”排序）

1. **【头号，强烈建议】补 RAE / range-bearing 激光观测模型**（缺口 A，Barfoot 7.143–7.147）。
   - *贡献*：极高——这是 Barfoot ch6 唯一全书无对应物的内容，且 range-bearing 是滤波 SLAM 的教科书级标准观测模型，本书激光章/滤波章都缺它会显得“有激光算法、无激光观测模型”。
   - *工作量*：小——一节即可（球坐标正/逆推导 + 平面退化 + 一张几何图），数学简单（就是主轴旋转 $\mathbf C_2,\mathbf C_3$ 的应用，本书 §2 已有 $\mathbf C_i$）。
   - *落点*：`P2_slam/lidar_slam.tex` §1 之后新增《激光的观测模型：从 RAE 到点云》；顺带在 `P1_estimation` 用 range-bearing 作 EKF 观测 worked example（若该章需要）。性价比最高，应排第一。

2. **【重要】补 IMU 杆臂（lever-arm）加速度计模型**（缺口 B，Barfoot 7.151–7.153 + 合记 7.154）。
   - *贡献*：高——补上“传感器不在体心”的真实工程情形，并把 `rigid_body_motion.tex` §9 已有的哥氏母式落到具体传感器，形成“运动学→IMU”闭环；标定/外参章可引用。
   - *工作量*：小到中——母式（四项加速度）已在 §9，仅需把 7.151→7.153 的两次求导落到加速度计模型，约一段 + 一两个公式；可直接复用 §9 的 $\mathbf C_{12}[\ddots]$ 而不重复推导。
   - *落点*：`P2_slam/imu_model.tex` 比力模型之后，新增《传感器不在体心：杆臂效应》。

3. **【轻量】补 Barfoot 习题 7.10 / 7.11 为练习**（与 A/B 同源）。
   - *贡献*：中——7.10（用激光读数 + 位姿链画地图、并讨论单次读数能否唯一定位）是 RAE + 位姿复合 + 可观性的综合好题；7.11（含地球自转的 IMU 模型）承接缺口 B。
   - *工作量*：小——补完 A/B 后自然附带，作为对应章节的 practice 即可。
   - *落点*：分别置于 `lidar_slam.tex` / `imu_model.tex` 的练习区。

4. **【一致性，零数学】复核 `rigid_body_motion.tex` line 1077 延伸阅读**——补完缺口 A 后该行（提及 Barfoot ch6 含 RAE）即与正文自洽；在此之前确认它不致让读者误判正文已含 RAE。工作量近乎为零，随缺口 A 一并处理。

---

### 审计方法备注
- Barfoot ch6 全文 1341 行已分块通读（§7.1 向量起，至 §7.6 习题止）；逐知识单元（推导/worked example/定理证明/表/图/数值）枚举后，grep 整个 `parts/` 树定位。
- 重叠最密的 `rigid_body_motion.tex`（1137 行）、`lie_theory.tex`（897 行）逐行读毕；`camera_model.tex`/`imu_model.tex`/`notation.tex` 经定向通读 + 关键证明逐字核验（本质/基础/单应证明已比对 Barfoot 7.108–7.117/7.131 逐步一致）。
- 缺口 A（RAE）经 `lidar_slam.tex` 全节标题 + 球坐标/方位/俯仰关键词 grep 确认全树无落点（lidar 章建模为 ToF 测距 + range image，非 RAE 观测模型）。
- 缺口 B（杆臂）经 `杆臂/lever/科氏/哥氏/向心/离心` 全树 grep 确认：哥氏母式在 `rigid_body_motion.tex` §9，但 IMU 加速度计的杆臂展开无落点（`imu_model.tex` 明确假定传感器系≡体系）。
