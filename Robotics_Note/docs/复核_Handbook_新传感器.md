# 对抗式复核报告：Handbook 新增 3 个传感器章

**审查对象**：`parts/P2_slam/` 的 `radar_slam.tex`(ch:radar)、`event_camera_slam.tex`(ch:event)、`leg_odometry.tex`(ch:leg)，及其与现有章（lidar/imu/vio/eskf/lie/slam_est/control）的缝合。
**审查方式**：三新章 + 所承接的现有章关键节全部读完；硬核数学逐式独立验算；记号/独立性/缝合逐条对照。**只读审查，未改任何 .tex / refs.bib。**
**日期**：2026-06-18

---

## 严重度计数

| 严重度 | 数量 |
|---|---|
| **Blocker** | 0 |
| **Major** | 2 |
| **Minor** | 6 |

静态层（label/cref/cite）确如交接所述全过：3 新章 32 个 cite key 全部命中 `refs.bib`；所依赖的跨章锚点（`ins:beyond-euler`、`def:vio-coupling`、`sec:ctrl-legged-cheetah`、`eq:ctrl-friction-pyramid`、`thm:intro-scale`、`tab:filter-vs-opt`、`paper:inekf`、`eq:imu-residual`、`eq:cov-recursion`、`sec:est-steam` 等）全部 resolve。

---

## Major

### M1. leg `eq:leg-vel-meas` 世界系机身速度公式帧不一致（招牌结论出错）
**文件:行号**：`leg_odometry.tex:362`（推导步 `:358`，"仅差 R_wb" 注 `:365`）

**问题**：盒装结论
```
v_b^w = -ω_b^w × f_p(q) - R_wb J_v(q) q̇          (eq:leg-vel-meas)
```
其叉积项 `ω_b^w × f_p(q)` 把**世界系**角速度 `ω_b^w` 与**机身系**杆臂 `f_p(q)`（脚在机身系的位置，本章自定义）相叉，帧不一致。推导步（`:358`）`v_k^w = v_b^w + ω_b^w × t_k^b + R_wb v_k^b` 同病——`t_k^b = f_p` 是机身系量，必须先旋到世界系才能与 `ω_b^w` 相叉。

把正确的机身系版本（`:365` 的 `v_b^b = -ω_b^b × f_p - J_v q̇`，与 Handbook 12.21 一致、本身正确）整体左乘 `R_wb`，用恒等式 `R(a×b)=(Ra)×(Rb)` 得正确世界系式：
```
v_b^w = -ω_b^w × (R_wb f_p) - R_wb J_v q̇
```
即**杆臂 f_p 也要旋到世界系**。因此 `:365` 那句"二者仅差一个 R_wb"是错的——叉积项差的是"把 f_p 旋一下"，不是整体一个 R_wb 因子。

**自证一致性反例**：本章下游 `eq:leg-vel-noisy`(`:575`) 写 `ṽ = -J_v q̇ - ω×f_p + η_v`（**无 R_wb**，即机身系 v_b^b），随后 `eq:leg-vel-sum`(`:580`) 用 `ΔR̃_ik ṽ_k` 把机身系速度旋进 i 系累积位移、`eq:leg-vel-residual`(`:585`) 对 `R_i^T(t_j-t_i)`——整条预积分链**正确地、隐式地按机身系 v_b^b 处理**，恰好与盒装的世界系 `eq:leg-vel-meas` 自相矛盾。即本章自己的后续公式证明了盒装式有误。

**影响面大**：`eq:leg-vel-meas` 是本章两块基石之一，被小结速查表(`:794`)、习题 2(`:900`)、故障排查(`:864`)反复引用，且数值例(`:719-731`)亦以它为题（例中 `R_wb=I` 故数值碰巧不暴露 bug，但 step 2 文字未声明 R_wb 仍为 I，读者按盒装式在 R_wb≠I 时复算会错）。

**修复建议**：将 `eq:leg-vel-meas` 改为 `v_b^w = -ω_b^w × (R_wb f_p) - R_wb J_v q̇`；删去/改写 `:365` "仅差一个 R_wb" 注为"机身系版去掉两处 R_wb 旋转即得"；或更稳妥——把招牌结论直接以机身系 `v_b^b` 给出（与 Handbook 12.21、与下游 `eq:leg-vel-noisy` 一致），世界系式仅作附注。数值例 step 2 显式补 "R_wb 仍 = I"。

---

### M2. 雷达章"三问入图"统一范式未被两兄弟章兑现（缝合的核心承诺落空）
**文件:行号**：定义于 `radar_slam.tex:429-430`（`ins:radar-factor-recipe`）

**问题**：雷达章在 `ins:radar-factor-recipe` 提出全章群的统一装置——"新传感器进因子图只需回答三问（约束哪些状态/残差怎么写/信息矩阵多大）"，并**明文承诺**：
> "`\cref{ch:event}` 的事件因子、`\cref{ch:leg}` 的接触/速度预积分因子都将照这三问构造——这是全书'异构传感器、同一张图'的统一范式。"

但核查 `event_camera_slam.tex` 与 `leg_odometry.tex`：**两章均无一处 `\cref{ins:radar-factor-recipe}`，也无"三问"的回扣措辞**。
- event 章的入图叙述走自己的 `ins`（CMax↔直接法、三线汇流），未接"三问"；
- leg 章的统一完全经"IMU 预积分同构"(`ins:leg-preint-iso`)与"InEKF/SE₂(3) 三线汇流"(`ins:leg-three-lines`)展开，亦未接"三问"。

结果：本应贯穿三章的统一脉络由雷达**单方面宣告**、被两个兄弟章无视，"浑然一体"出现可见接缝。这正是审查红线 1 点名要查的"radar 提的三问是否真把三章统一起来"——**答案是没有**。

**修复建议**（任一即可，建议都做）：(a) 在 event 章构造事件 CMax 因子/事件-惯性因子处加一句"恰是 `\cref{ins:radar-factor-recipe}` 三问的实例（约束连续轨迹控制点/对齐残差/IWE 噪声逆）"；(b) 在 leg 章 `subsec:leg-fk-factor` 或 `ins:leg-preint-iso` 处加"接触/速度因子同样答完 `\cref{ins:radar-factor-recipe}` 三问"。两处各一句即可把承诺闭合，且与现有 `ins:leg-preint-iso` 不冲突（IMU 同构是"三问"的具体填法，二者相容）。

---

## Minor

### m1. 雷达-惯性状态无位置，却宣称复用 9 维 IMU 残差
**文件:行号**：`radar_slam.tex:386`(`eq:radar-state`)、`:392`/`:395`(`eq:radar-cost`)

**问题**：`eq:radar-state` 取 `x_k=[v^s; q_s^w; b]`（**无位置 p**），但 `eq:radar-cost` 的惯性项写 `‖r_{I_{k,k+1}}‖²_Λ` 并注"即 `\cref{ch:imu}` 的预积分残差 `\cref{eq:imu-residual}`"。而 `eq:imu-residual` 是 9 维 `[r_ΔR; r_Δv; r_Δp]`，其位置残差行 `r_Δp` 显含 `p_i,p_j`——状态里没有位置，无法逐字套用完整 9 维残差。雷达-惯性速度估计实际只用预积分的旋转+速度部分（位置不可由"多普勒速度+陀螺"观测），故应是**约简残差**。

**修复建议**：在 `eq:radar-cost` 后补一句"因状态不含位置，惯性项取 `eq:imu-residual` 的旋转+速度子残差 `[r_ΔR; r_Δv]`（位置在纯速度估计中不可观，略去）"。符号表(`:567`)里 `r_I` 条目同步注明"约简版"。

### m2. event `tab:event-systems` 把 Ultimate-SLAM 列为"间接 + 滑窗"，与正文措辞略有张力
**文件:行号**：`event_camera_slam.tex:401`（表）vs `:345`/`:370`（正文）

**问题**：表中 USLAM 归"间接 + 滑窗"；正文(`:370`)称其"事件 + 帧 + IMU 三模态紧耦合"。USLAM 的事件前端用的是特征跟踪（间接）不假，但"间接"标签与正文强调的"紧耦合三模态"并列时，易让读者以为整系统纯间接。非错误，属归类标签与正文侧重的轻微不一致。

**修复建议**：表中 USLAM"直接/间接"列改为"间接特征 + 紧耦合"或加脚注，与正文对齐。

### m3. leg `eq:leg-grf-from-torque` 的 `F`、`h_q` 引入偏简，浮基动力学方程未给
**文件:行号**：`leg_odometry.tex:428-431`

**问题**：GRF 反演式 `f_i = -(J̄_{i,v}^T)^{-1}(τ_i - h_{q,i} - F^T v̇)` 直接抛出，`F`（"质量阵的一块"）、`h_{q,i}`、`v̇` 来历仅一句带过，未给浮基动力学母式 `M q̈ + h = S^T τ + J_c^T f` 的形态。本章他处（fk/J、零速度、预积分、InEKF）都做到自包含可独立读懂，唯此式偏黑盒，略破坏"全章自包含"基调。考虑到该式属选学小节(`subsec:leg-grf`)且明确 punt 给 Featherstone，可接受，但与本章其余严谨度有落差。

**修复建议**：补一行浮基动力学母式与 `F,h_q` 在其中的位置（一两句即可），或明确标注"此式细节见 `\cref{sec:ctrl-legged}` 浮基动力学，此处仅示意反演结构"。

### m4. event 章习题 5 引用 `eq:steam-error`，但该 label 指向的是 WNOA 误差因子——需确认语义对得上
**文件:行号**：`event_camera_slam.tex:537`（习题 5 引 `eq:steam-error`）

**问题**：习题 5 要求"把事件 CMax 对齐因子与 `\cref{eq:steam-error}` 的 WNOA 运动先验因子拼进 `eq:steam-A`"。`eq:steam-error`(`slam_state_estimation.tex:1394`) 确为 WNOA/GP 先验误差项，语义正确。仅提示：该题对读者跨章定位要求高，且 `eq:steam-A` 的箭头矩阵在 STEAM 节是含路标块的完整版，事件场景（多为纯位姿/速度轨迹）拼入时结构略有出入。非错误，属习题难度/语境提示可加强。

**修复建议**：习题 5 加半句提示"事件场景通常无路标块，箭头矩阵退化为 `A_11` 块三对角"。

### m5. leg `eq:leg-cop` 的不等式方向/上界记法易误读
**文件:行号**：`leg_odometry.tex:401-404`

**问题**：CoP 约束写 `[-τ_y/f_z; τ_x/f_z] ≤ [CoP_x; CoP_y]`，文字说 `CoP_{x,y}` 是"压力中心分量的上界"。但 CoP 落在支撑多边形内是**双边**约束（既有上界也有下界 `CoP_x^min ≤ -τ_y/f_z ≤ CoP_x^max`），单写 `≤` 上界省了下界，且把"上界"记成 `CoP_x`（与"压力中心本身"同名）易混。属记法简化导致的轻微歧义，非数学错误（教学上常如此简写）。

**修复建议**：改记上界为 `CoP_x^{max}` 等，或加一句"对称地有下界，此处只写上界示意"。

### m6. event `eq:cmax-iwe` 的 δ 与离散插值核衔接可更显式
**文件:行号**：`event_camera_slam.tex:240-243`、雅可比 `:314`

**问题**：IWE 定义用连续冲激 `δ(p - x_k')`，雅可比 `eq:cmax-grad` 又对 `δ` 求空间梯度 `∇_p δ`。正文已注"离散实现中 δ 用双线性插值核替代"，但对 `∇_p δ`（连续冲激的梯度在解析上是分布导数，唯有替换成可微插值核后才良定义）这一步的"必须先离散化才可微"逻辑可更醒目，否则严谨读者会疑惑对 δ 求导的合法性。属表述清晰度，数学结论正确。

**修复建议**：在 `eq:cmax-grad` 前明示"以下对 δ 的求导均在'δ 已替换为双线性插值核 `κ`'的离散意义下进行，`∇_p δ` 即 `∇_p κ`"。

---

## 逐红线结论

### 红线 1 · 浑然一体 / 无接缝 —— **大体良好，一处真接缝（M2）**
- **承接 lidar RAE**：radar `ins:radar-three-fft` 明确复用 `eq:lidar-rae-fwd/model/jac`（已核对：去掉 v 后 `[r,θ,φ]` 与 RAE `[r,α,ε]` 同构，3×3 雅可比原样适用），衔接干净自然。
- **复用 GICP/NDT**：radar `sec:radar-reg` 处处以"`ch:pointcloud` 内核的雷达变体"展开（CFEAR/APDGICP/D2D），不重推 ICP，复用到位。
- **复用预积分/`Λ` 记号**：radar `eq:radar-cost`、leg `ins:leg-preint-iso` 均以 `eq:imu-residual`/`eq:cov-recursion`/`Λ=Σ^{-1}` 为母版，记号一脉相承（leg 的预积分同构表是全书最漂亮的缝合之一）。
- **互相呼应**：三章互引兄弟章、各章末"与相邻章节关系"表齐全；event↔radar"某维度从配角变主心骨"的对照(`event:132`)、event↔STEAM/GP 预积分伏笔兑现都很自然。
- **唯一真接缝（M2）**：radar 单方面宣告的"三问入图"统一范式未被 event/leg 回扣。这是审查特别点名之处，需补两句闭合。
- 另：radar 引子(`:84`)预告"`ch:event` 与 `ch:leg` 反复出现'每种新传感器都有一维别人没有的信号'"——event 章兑现了(`:132`)，leg 章未明确以"多一维信号"措辞呼应（leg 的卖点是"额外一路本体感受高频测量"，框架相容但口径略偏），属 M2 的同类轻微脱节。

### 红线 2 · 硬核数学 —— **自补 punt-fill 总体扎实，一处招牌式出错（M1）**
逐式独立验算结果：
- **radar 全对**：FMCW 测距 `d=cf_0/2S`（=`cf_0 T/2B` 自洽）、锯齿测速 `v=λΔφ/4πT`、三角解耦、AoA、ego-velocity 加权最小二乘法方程、**单雷达角速度不可观的秩论证**（`(ω×r)·u=0` 因 `ω×r⊥r` 且 `u∥r`，正确）、**杆臂耦合 `eq:radar-leverarm`**（`v^s=R_bs^T(v^b+ω^b×r_b^sb)`，全在机身系算叉积再旋到雷达系，与 IMU `eq:imu-leverarm-vel` 帧一致——正确）、多普勒残差符号(`e=v+v^s·u`)自洽。
- **event 全对**：事件生成 `ΔL=pC`、亮度恒定→事件即边缘、CMax 弯曲映射两特例、IWE、方差目标、`eq:cmax-grad` 雅可比结构（与 `eq:vo-direct-J` 同源：像素/事件梯度 × 投影雅可比 × 位姿雅可比）、纯旋转特例用右扰动求导的形式，均正确；崩塌全局最优陷阱诊断准确。
- **leg 多数对，一处错（M1）**：fk/J 定义、相对位姿 `eq:leg-fk-meas`(`fk(q)fk(q')^-1`，复合方向核对正确)、接触系几何、接触预积分残差(`r_C=[Log(B_i^T B_j); B_i^T(c_j-c_i)]`，由"理想不变"导出正确)、协方差 ∝Δt(随机游走律，正确)、GRF 反演结构、**接触辅助 InEKF**（增广 `X=[R,r,v,c]`、右不变观测 `Y=X^{-1}b`、`X^{-1}` 各平移列 `-R^T(·)` 结构同 `eq:se23-inverse`，更新雅可比不依赖状态——论证正确）。**唯 `eq:leg-vel-meas` 世界系式帧不一致（M1）**，且被自身下游公式反证。

### 红线 3 · 独立性 —— **优秀，无违规**
- 全树扫描三章：**无 punt/ventriloquize**（"见原书/参见原文/留给读者/原文给出"等措辞零命中）。
- 本书口吻贯穿，`\cite` 规范（carlone2026handbook、gallego2022eventsurvey、hartley2020contact、wisth2023vilens、bloesch2013state 等 32 key 全命中 refs.bib）。
- 自包含声明属实：三章均把核心推导独立重写（FMCW 物理、多普勒里程计、CMax、fk/J、零速度、预积分、InEKF 增广），可不依赖源书读懂。
- 平衡多书对照做得好：leg 章融 Camurri-Mattamala(Handbook)+Bloesch+Hartley+Wisth 四源，event 章融 Gallego 综述+CMax 系列，非单书叙述依赖。
- 唯 m3（GRF 反演偏黑盒、punt 给 Featherstone）是独立性的轻微让步，但属选学小节、可接受。

### 红线 4 · 记号一致 —— **优秀**
- **`\mathbf R` 专留旋转**：三章逐一扫描，R 仅用于旋转，无误用。
- **右扰动**：radar(`q⊗[½δφ;1]`)、event(`T Exp(δξ)`)、leg(`R Exp(δφ)`、`X Exp(δξ)`)全为右扰动，与全书一致。
- **帧下标 `R_ab`（b→a）**：radar `R_bs`、leg `R_wb`/`C_i^{-1}T fk` 均遵此约定；radar 用 `R_bs^T` 而非 `R_sb` 表 body→sensor，正确。
- **SE₂(3) `[R,r,v]` 逐元素一致**：leg `eq:leg-inekf-state` 的 `X=[R,r,v,c]` 与权威 `def:se23`(`lie_theory.tex:986`) 的列序（第2列位置 r、第3列速度 v）**逐元素吻合**；`X^{-1}` 结构与 `eq:se23-inverse` 一致；leg 明文回指 `def:se23`、`eq:se23-exp`、`eq:se23-inverse` 且引用准确（含"多一列平移类量"洞见的直接推广，与 `lie_theory.tex:1032` 的 `ins` 呼应）。`ξ=[ρ;ν;φ]`（平移类在前）一致。
- radar 噪声协方差用 `Σ_v`（避与 R/v 撞，沿 lidar 约定）、`Λ=Σ^{-1}`，与 imu 章一致；event 噪声协方差用 `Σ`、`C/C±` 阈值，自洽。

---

## 总评："章群是否浑然一体"

**结论：底子很整、缝合用心，距"读不出接缝"差两步——一处数学硬伤（M1）+ 一处统一承诺落空（M2）。**

这是一组**质量明显高于一般教材吸收**的新增章：记号 100% 对齐全书（R 专留旋转、右扰动、SE₂(3) 逐元素一致）、独立性零违规、绝大多数自补推导（含 radar 角速度不可观秩论证、leg 接触辅助 InEKF 群结构、event CMax 雅可比）经独立验算正确，且与 lidar RAE / pointcloud GICP / imu 预积分 / STEAM 的承接处处显式、自然。leg 章的"接触/速度预积分 = IMU 预积分换种高频测量"同构表、与 SE₂(3)/InEKF/控制三线汇流，是全书缝合的范例。

但要达到"浑然一体、读不出接缝"，必须修两处：
1. **M1** —— leg 招牌速度公式 `eq:leg-vel-meas` 帧不一致，且被本章自己的预积分链反证，属"自补 punt-fill 最易错"的典型，**必修**（改 1 个公式 + 1 句注 + 例 step2 补 R_wb=I）。
2. **M2** —— radar 宣告的"三问入图"统一范式未被 event/leg 回扣，正是把三章"焊"成一个章群的那条焊缝没焊上，**应修**（两章各补一句 `\cref{ins:radar-factor-recipe}`）。

修掉这两处（外加 m1 雷达残差约简的一句澄清），三章即可称得上"浑然一体"。其余 minor 多为表述清晰度，可在打磨阶段顺手处理，不阻塞。
