# 单刚体模型凸 MPC（四足运动控制）—— 联网 SOTA 调研 + 文献增补

> 用途：为《机器人学笔记》新部「四足机器人运动控制」**范本章 = 单刚体模型 MPC**（拟落 `parts/P7_legged/srbd_mpc.tex`）提供 §9 网络增补素材、bib 条目、关键公式/范式、教学增补。
> 调研日期：2026-06-23。工具：WebSearch / WebFetch（Semantic Scholar MCP 当时因 SOCKS 代理缺 `socksio` 不可用，已绕开）。
> 标注约定（沿用 `docs/编写规范.md`）：`\pz{}`=存疑待核（黄），`\rebuilt{}`=重建/改写待核（橙）。本文档内用 **[存疑]** 行内标出尚未逐字核实之处，落 .tex 时转成相应批注 + 登 claims 台账。
> 重要：**奠基与综述两条目已在 `refs.bib` 内**——`dicarlo2018cheetah`（line 700）、`wensing2024legged`（line 701）；切勿重复添加。其余条目经查 `refs.bib` 不存在（grep 过 kajita/orin/raibert/chignoli/lipm/impulse/ding），可安全新增。

---

## 0. 核实结论速览（bib 键清单）

| bib 键 | 角色 | 核实状态 |
|---|---|---|
| `dicarlo2018cheetah` | **奠基**：单刚体凸 MPC 标准范式（IROS 2018） | ✅ **已在 refs.bib**；作者序经 Wensing 主页核实 |
| `wensing2024legged` | 权威综述：Optimization-Based Control for Dynamic Legged Robots（T-RO 2024） | ✅ **已在 refs.bib** |
| `bledt2018cheetah3` | 平台/控制体系：MIT Cheetah 3 设计与控制（IROS 2018） | ✅ 作者序经 Wensing 主页核实，待新增 |
| `kim2019wbic` | WBIC + MPC 分层（arXiv/CoRR 2019） | ✅ 作者序、arXiv 号经 dblp 核实，待新增 |
| `ding2021repfree` | 表示无关 MPC（变分线性化于 SO(3)，T-RO 2021） | ✅ 作者序、卷、arXiv 号经 arXiv 核实，待新增 |
| `ding2019realtime` | 先驱：实时 MPC 多样动态动作（ICRA 2019） | ✅ 作者序经 Ding 主页核实，待新增 |
| `raibert1986legged` | 历史奠基：Legged Robots That Balance（落足启发式来源） | ✅ MIT Press 1986，待新增 |
| `kajita2001lipm` | 对照模型：3D 线性倒立摆 LIPM（IROS 2001） | ✅ 卷/页/作者经多源核实，待新增 |
| `orin2013centroidal` | 对照模型：质心动力学（Autonomous Robots 2013） | ✅ 卷/页经多源核实，待新增 |
| `dingpark2022so3mpc` | 相关：单刚体 SO(3) 凸 MPC（RA-L/IROS 2022） | ⚠ **[存疑]** 完整作者序未逐字核实，见 §1 注 |

---

## 1. bib 条目（biblatex 格式，供 refs.bib）

> 风格对齐 `refs.bib` 既有条目（紧凑单行或多行皆可；title 内专有名词加 `{}` 保护大小写，如 `{MIT}`、`{MPC}`、`{SO}(3)`、`{LIP}`）。

```bibtex
% ---- 已在 refs.bib，勿重复添加（此处仅备查/复核作者序）----
% @inproceedings{dicarlo2018cheetah, author={Di Carlo, Jared and Wensing, Patrick M. and Katz, Benjamin and Bledt, Gerardo and Kim, Sangbae}, title={Dynamic Locomotion in the {MIT} Cheetah 3 Through Convex Model-Predictive Control}, booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, pages={1--9}, year={2018}}
% @article{wensing2024legged, author={Wensing, Patrick M. and Posa, Michael and Hu, Yue and Escande, Adrien and Mansard, Nicolas and Del Prete, Andrea}, title={Optimization-Based Control for Dynamic Legged Robots}, journal={IEEE Transactions on Robotics}, volume={40}, pages={43--63}, year={2024}, note={arXiv:2211.11644}}

% ---- 待新增 ----

@inproceedings{bledt2018cheetah3,
  author    = {Bledt, Gerardo and Powell, Matthew J. and Katz, Benjamin and Di Carlo, Jared and Wensing, Patrick M. and Kim, Sangbae},
  title     = {{MIT} Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot},
  booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages     = {2245--2252},
  year      = {2018},
  note      = {\pz{页码 2245--2252 取自 IEEE 索引，待逐字核}},
}

@article{kim2019wbic,
  author        = {Kim, Donghyun and Di Carlo, Jared and Katz, Benjamin and Bledt, Gerardo and Kim, Sangbae},
  title         = {Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control},
  journal       = {arXiv preprint arXiv:1909.06586},
  year          = {2019},
  note          = {\pz{仅检索到 arXiv/CoRR 2019 版；是否另刊于期刊未确认}},
}

@article{ding2021repfree,
  author    = {Ding, Yanran and Pandala, Abhishek and Li, Chuanzheng and Shin, Young-Ha and Park, Hae-Won},
  title     = {Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds},
  journal   = {IEEE Transactions on Robotics},
  volume    = {37},
  number    = {4},
  pages     = {1154--1171},
  year      = {2021},
  note      = {arXiv:2012.10002; \pz{卷期号/页码待逐字核}},
}

@inproceedings{ding2019realtime,
  author    = {Ding, Yanran and Pandala, Abhishek and Park, Hae-Won},
  title     = {Real-time Model Predictive Control for Versatile Dynamic Motions in Quadrupedal Robots},
  booktitle = {2019 IEEE International Conference on Robotics and Automation (ICRA)},
  pages     = {8484--8490},
  year      = {2019},
  note      = {\pz{页码取自 IEEE 索引，待逐字核}},
}

@book{raibert1986legged,
  author    = {Raibert, Marc H.},
  title     = {Legged Robots That Balance},
  publisher = {MIT Press},
  address   = {Cambridge, MA},
  year      = {1986},
}

@inproceedings{kajita2001lipm,
  author    = {Kajita, Shuuji and Kanehiro, Fumio and Kaneko, Kenji and Yokoi, Kazuhito and Hirukawa, Hirohisa},
  title     = {The {3D} Linear Inverted Pendulum Mode: A Simple Modeling for a Biped Walking Pattern Generation},
  booktitle = {2001 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  volume    = {1},
  pages     = {239--246},
  year      = {2001},
}

@article{orin2013centroidal,
  author    = {Orin, David E. and Goswami, Ambarish and Lee, Sung-Hee},
  title     = {Centroidal Dynamics of a Humanoid Robot},
  journal   = {Autonomous Robots},
  volume    = {35},
  number    = {2-3},
  pages     = {161--176},
  year      = {2013},
}

% [存疑] 相关——单刚体凸 MPC 直接建模于 SO(3)。检索强烈指向
% Yanran Ding & Hae-Won Park, RA-L 2022, IEEE Xplore doc 9811926;
% 但完整作者序未能逐字核实(IEEE 页空、RG/PDF 取不到文本)。
% 落 .tex 前务必核作者全名与卷期；暂可不引、改以 ding2021repfree 承担"几何/SO(3)"主线。
@article{dingpark2022so3mpc,
  author    = {Ding, Yanran and Park, Hae-Won},
  title     = {Convex Model Predictive Control of Single Rigid Body Model on {SO}(3) for Versatile Dynamic Legged Motions},
  journal   = {IEEE Robotics and Automation Letters},
  year      = {2022},
  note      = {\pz{作者序/卷期/年份待核；IEEE Xplore doc 9811926}},
}
```

---

## 2. 关键公式 / 标准凸 MPC 范式要点

来源：Di Carlo et al. 2018（IROS）`dicarlo2018cheetah`；交叉核对 MIT DSpace 原文摘要 + 技术笔记 kyo-kutsuzawa 整理 + Ding et al. 综述脉络。**两处数值口径有出入，已在 §2.5 标注。**

### 2.1 单刚体（SRBD）简化前提
将整机近似为**集中于质心的单个刚体**，**忽略腿部质量与动力学**（腿轻、点接触）。各支撑腿对身体施加**地面反作用力**（GRF）$\mathbf f_i\in\mathbb R^3$，作用点为足端相对质心位置 $\mathbf r_i\in\mathbb R^3$。这把"全身高维非线性动力学"压成一个"刚体 + 若干外力"的低维问题，从而可凸化。

### 2.2 连续动力学（牛顿—欧拉）
$$
\ddot{\mathbf p} \;=\; \frac{1}{m}\sum_{i=1}^{n}\mathbf f_i \;-\; \mathbf g ,
\qquad
\frac{\mathrm d}{\mathrm d t}\!\left(\mathbf I\,\boldsymbol\omega\right)
\;=\;\sum_{i=1}^{n}\mathbf r_i\times\mathbf f_i ,
\qquad
\dot{\mathbf R}=[\boldsymbol\omega]_\times\,\mathbf R .
$$
其中 $m$ 为总质量，$\mathbf g$ 重力加速度，$\mathbf I$ 世界系惯量，$\mathbf R\in SO(3)$ 身体姿态，$[\cdot]_\times$ 反对称矩阵。

### 2.3 线性化（凸化的三处关键近似）
1. **丢陀螺项**：$\boldsymbol\omega\times(\mathbf I\boldsymbol\omega)\approx \mathbf 0$，于是 $\mathbf I\dot{\boldsymbol\omega}\approx\sum_i \mathbf r_i\times\mathbf f_i$。动态步态中机身角速度不大，该项小，去掉后转动方程对 $\mathbf f_i$ **线性**。
2. **小横滚/俯仰 + ZYX 欧拉角近似**：姿态用 ZYX 欧拉角 $\boldsymbol\Theta=[\phi,\theta,\psi]^\top$（roll, pitch, yaw）。角速度到欧拉角速率的映射在小 $\phi,\theta$ 下近似为**仅含偏航 $\psi$** 的旋转：
$$
\boldsymbol\omega \;\approx\; \mathbf R_z(\psi)\,\dot{\boldsymbol\Theta}
\quad\Longrightarrow\quad
\dot{\boldsymbol\Theta}\approx \mathbf R_z(\psi)^{-1}\boldsymbol\omega .
$$
这是把姿态运动学线性化、并令系统矩阵仅依赖**当前偏航 $\psi$**（而非全姿态）的关键，使每个 MPC 时刻得到一个**线性时变**（LTV）模型。
3. **惯量取常值**：世界系惯量近似为绕标称偏航旋转的常量 $\hat{\mathbf I}=\mathbf R_z(\psi)\,{}^{\mathcal B}\mathbf I\,\mathbf R_z(\psi)^\top$。

### 2.4 状态空间与 13 维状态（重力增广技巧）
$$
\boxed{\;\dot{\mathbf x}(t)=\mathbf A_c(\psi)\,\mathbf x(t)+\mathbf B_c(\mathbf r_1,\dots,\mathbf r_n,\psi)\,\mathbf u(t)\;}
\qquad\xrightarrow{\text{离散}}\qquad
\mathbf x_{k+1}=\mathbf A_k\mathbf x_k+\mathbf B_k\mathbf u_k .
$$
**状态向量（13 维）**：
$$
\mathbf x=\big[\;\underbrace{\boldsymbol\Theta}_{3}\;\;\underbrace{\mathbf p}_{3}\;\;\underbrace{\boldsymbol\omega}_{3}\;\;\underbrace{\dot{\mathbf p}}_{3}\;\;\underbrace{g}_{1}\;\big]^\top\in\mathbb R^{13}.
$$
即【姿态欧拉角 / 位置 / 角速度 / 线速度 / 重力】。第 13 维把常重力 $g$ 并入状态，使含 $-\mathbf g$ 的**仿射**项以**齐次（线性）**形式进入 $\mathbf A_c$（$\dot g=0$），整体保持 $\dot{\mathbf x}=\mathbf A_c\mathbf x+\mathbf B_c\mathbf u$ 的线性结构——这是该范式标志性的小技巧。

**控制量**：$\mathbf u=[\mathbf f_1^\top,\dots,\mathbf f_n^\top]^\top\in\mathbb R^{3n}$（$n=4$ 腿 → $\mathbb R^{12}$），即**足底反作用力为决策变量**。求得 $\mathbf f_i$ 后经支撑腿雅可比映射为关节力矩：$\boldsymbol\tau_i=\mathbf J_i^\top\mathbf R_i^\top\mathbf f_i$（前馈，再叠加全身/关节反馈）。

### 2.5 离散 QP（目标 + 约束）
在长度 $k$ 的预测窗上，凸 QP：
$$
\min_{\{\mathbf x\},\{\mathbf u\}}\;\sum_{i=0}^{k-1}\underbrace{\lVert \mathbf x_{i+1}-\mathbf x_{i+1}^{\mathrm{ref}}\rVert^2_{\mathbf Q_i}}_{\text{状态跟踪}}+\underbrace{\lVert \mathbf u_i\rVert^2_{\mathbf R_i}}_{\text{力正则}}
\quad\text{s.t.}
$$
- **动力学**（等式）：$\mathbf x_{i+1}=\mathbf A_i\mathbf x_i+\mathbf B_i\mathbf u_i$；
- **摩擦锥（金字塔线性化）**：$-\mu f_{z,i}\le f_{x,i}\le \mu f_{z,i}$，$-\mu f_{z,i}\le f_{y,i}\le \mu f_{z,i}$（把二阶锥近似为线性不等式，保持 QP）；
- **法向力上下界**：$f_{\min}\le f_{z,i}\le f_{\max}$；
- **接触调度分离**（contact schedule）：摆动腿强制零力 $\mathbf f_i=\mathbf 0$（以 $\mathbf D_i\mathbf u_i=\mathbf 0$ 选择子集）。**步态/触地时序由独立的有限状态机（gait scheduler）给出，不进 MPC 优化**——这是"凸"的前提：一旦把"哪条腿何时触地"也作决策，问题变为含整数变量的混合整数/非凸问题。

整理为标准型 $\min_{\mathbf U}\ \mathbf U^\top\mathbf H\mathbf U+\mathbf U^\top\mathbf g$ s.t. $\underline{\mathbf c}\le\mathbf C\mathbf U\le\overline{\mathbf c}$，用稠密 QP 求解器（原文 **qpOASES** + Eigen）实时解。

**报告指标**（⚠ 两处口径出入，落文须标 `\pz` 并以原文为准）：
- 预测窗 $\approx 0.5\,\mathrm s$（约 10 步）；
- 求解时间：**摘要原文"under 1 ms"**；技术笔记整理为"典型 0.2--0.3 ms、最坏 <0.5 ms"。→ **[存疑]** 取"$<1\,\mathrm{ms}$"为稳妥引用，细分数值回查原文实验节。
- MPC 频率 $\approx 20\text{--}30\,\mathrm{Hz}$（摘要"20--30 Hz"；技术笔记"30 Hz"）；
- 实现多种步态（trot/gallop/flying-trot/pace/bound/pronk/3 腿等），鲁棒变速行走。

---

## 3. 教学增补（历史脉络 / 直觉 / 与质心模型·LIPM 对照 / 最新进展）

### 3.1 历史脉络（落足启发式 → 简化模型 → 凸 MPC）
- **Raibert（1986，`raibert1986legged`）**：单腿/双足跳跃机器人，提出**落足启发式**——把足端落在"中性点"附近，落点 = **机身速度的线性函数**，即可调速、稳定。这是"用落足控平衡"的源头；其结构与后来 Pratt 的**捕获点**（capture point）、四足里的 Raibert heuristic 落足一脉相承，至今仍作凸 MPC 的**落足点对照/初值**。
- **LIPM（Kajita 2001，`kajita2001lipm`）**：3D 线性倒立摆，把双足机器人简化成"质点 + 无质量伸缩腿"，质心高度恒定 → 线性动力学 → 解析步态生成（ZMP/捕获点框架）。**质量集中于一点、无转动**。
- **质心动力学（Orin 2013，`orin2013centroidal`）**：把全身动量压成 6 维质心动量（线动量 + 角动量），比 LIPM 多了**角动量**，但仍需外接 CMM（质心动量矩阵）映射，且角动量项使其一般**非凸**。
- **单刚体凸 MPC（Di Carlo 2018，`dicarlo2018cheetah`）**：本范本主角。相对 LIPM **保留了完整三维转动**（有惯量 $\mathbf I$、有姿态 $SO(3)$），相对质心模型**用"丢陀螺项 + 小角近似 + 重力增广"换来凸性**，从而能在 mm 级嵌入式 CPU 上实时跑出高度动态步态。MIT Cheetah 3 平台（`bledt2018cheetah3`）是其硬件载体。

### 3.2 直觉：为什么"单刚体"够用、且能凸
- **够用**：动态步态里腿很轻、动得快，腿的动量相对机身小；把腿当"会瞬时改变着力点的无质量推杆"，主导平衡的是机身这块大刚体的牛顿—欧拉方程。
- **能凸**：三处近似（§2.3）各自把一处非线性"拍平"——陀螺项→丢、姿态运动学→小角线性化、重力仿射项→增广进状态。代价是大横滚/俯仰、强自旋时模型失真；这正是后续 SO(3)/几何 MPC 要修的。
- **一句话对照**：**LIPM = 质点无转动**；**质心模型 = 6 维动量（含角动量、一般非凸）**；**单刚体凸 MPC = 完整刚体转动 + 三处近似换凸性**。

### 3.3 与质心模型 / LIPM 的对照表

| 维度 | LIPM（Kajita 2001） | 质心动力学（Orin 2013） | 单刚体凸 MPC（Di Carlo 2018） |
|---|---|---|---|
| 简化对象 | 质点 + 无质量伸缩腿 | 全身压成 6 维质心动量 | 集中质心的单刚体（忽略腿动力学） |
| 转动 | 无（仅平移） | 有角动量，无显式姿态 | 有姿态 $SO(3)$ + 惯量 $\mathbf I$ |
| 凸性 | 线性（解析） | 一般非凸（角动量耦合） | **凸 QP**（三处近似换来） |
| 控制量 | ZMP / 落足点 | 质心力/力矩（接触力） | **足底反作用力 $\mathbf f_i$** |
| 典型用途 | 双足 ZMP 步态生成 | 人形多接触规划 | **四足实时动态步态** |
| 主要近似代价 | 丢转动、恒质心高 | 需 CMM、非凸 | 大角度/强自旋失真 |

### 3.4 最新进展（SOTA，供 §9 网络增补）
- **分层 WBIC + MPC（Kim 2019，`kim2019wbic`）**：MPC（单刚体、低频、定反作用力/质心轨迹）+ **全身脉冲控制 WBIC**（高频、含全身动力学、把 MPC 的力/加速度作任务）。解决"凸 MPC 用简化模型 → 落到真机要补全身动力学/接触一致性"的落差。**这是把单刚体 MPC 真正跑上高动态硬件的标准工业级架构。**
- **几何 / 表示无关 MPC（Ding 2021，`ding2021repfree`；及 `dingpark2022so3mpc` [存疑]）**：直接在 $SO(3)$ 上用**变分线性化**，以旋转矩阵演化转动，**避开欧拉角奇异与小角近似**，仍化为实时 QP。修的正是 §2.3 近似 2 的失真，能做更剧烈的翻滚/跳跃。其先驱为 Ding 2019（`ding2019realtime`）。
- **鲁棒 / 不确定性凸 MPC**：如 Trivedi et al. 2024（`trivedi2025friction`，**已在 refs.bib**）把摩擦锥/负载不确定性写成**机会约束**，仍解 QP。
- **非线性 MPC / 采样式 MPC**：综述 `wensing2024legged`（**已在 refs.bib**）系统对比 SRBD 凸 MPC、全质心非线性 MPC、全阶 NMPC；近年还有采样/扩散式 MPC（`xue2025dial`、`huang2025diffusionmpc`，**均已在 refs.bib**）走全阶力矩级。教学定位：**单刚体凸 MPC = 实时性与建模保真度折中的"甜点"**，是理解全谱系的基线。
- **落足点对照**：Raibert 启发式仍常作凸 MPC 的落足初值/对照基线（如近期 Dual-MPC footstep planning，arXiv:2511.07921，未入库，可选）。

---

## 4. 落 .tex 时的提示（§9 网络增补怎么写）

1. **§9 命名**沿用全书惯例（参 `parts/P0_intro/intro_slam.tex` 末节、`parts/P3_planning/*` 的"前沿/网络增补"段）：可作"\section{延伸与前沿}"或章末"网络增补"小节，开头用 `\rebuilt{}` 声明"本节据联网调研补入，论文事实层经核实，存疑处已 \pz 标注、入 claims 台账"。
2. **正文主线**只深讲 `dicarlo2018cheetah` 的范式（§2 全套公式逐式 `\cite{dicarlo2018cheetah}`）；其余条目在 §9 一段话点到 + `\cite`，避免堆砌（编写规范第 9 条）。
3. **务必 `\pz` 标注**：求解时间/频率口径出入（§2.5）、`dingpark2022so3mpc` 作者序、各 `note` 中标 `\pz` 的页码/卷期。落文前若有条件，回查 IEEE/原 PDF 逐字核实再去标。
4. **去重**：与 `parts/P3_planning/` 已有的鲁棒/分布式 MPC、`parts/P2_slam/leg_odometry.tex`（腿式里程计）交叉处，用 `\cref` 指向，单刚体 MPC 本身只此一处深讲。

---

## 5. 来源链接（核实溯源）

- Di Carlo et al. 2018, MIT DSpace 原文：https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf （摘要："under 1 ms … 20–30 Hz"）
- 技术笔记整理（formulation + "0.2–0.3 ms / 30 Hz / qpOASES"）：https://kyo-kutsuzawa.github.io/technical-note/carlo2018.html
- ACM/IEEE 索引（IROS 2018, DOI 10.1109/IROS.2018.8594448）：https://dl.acm.org/doi/10.1109/IROS.2018.8594448
- Wensing 主页（核实 `dicarlo2018cheetah`、`bledt2018cheetah3` 作者序）：https://sites.nd.edu/pwensing/publications-2/
- Bledt et al. 2018 Cheetah 3, DSpace：https://dspace.mit.edu/bitstream/handle/1721.1/126619/IROS.pdf （DOI 10.1109/IROS.2018.8593885）
- Kim et al. 2019 WBIC, dblp（arXiv:1909.06586, CoRR 2019）：https://dblp.org/rec/journals/corr/abs-1909-06586.html
- Ding et al. 2021 Representation-Free MPC, arXiv:2012.10002（T-RO 2021）：https://arxiv.org/abs/2012.10002
- Ding et al. 2019 Real-time MPC（ICRA 2019），Ding 主页：https://sites.google.com/view/yanranding/publications
- Ding & Park SO(3) MPC [存疑], IEEE Xplore doc 9811926：https://ieeexplore.ieee.org/document/9811926/
- Kajita et al. 2001 3D-LIPM（IROS 2001, vol.1 pp.239–246）：https://researchr.org/publication/KajitaKKYH01
- Orin et al. 2013 Centroidal Dynamics（Autonomous Robots 35:161–176）：https://www.academia.edu/28216207/Centroidal_dynamics_of_a_humanoid_robot
- Raibert 1986 Legged Robots That Balance（MIT Press）：https://books.google.com/books/about/Legged_Robots_that_Balance.html
