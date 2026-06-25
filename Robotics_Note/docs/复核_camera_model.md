# 独立复核报告：parts/P2_slam/camera_model.tex

> **总评：PASS** — 知识全量、公式与数值例全部核对无误、9 处独立性改写后行文顺畅过渡不断裂、四坐标链叙事完整、独立性/LaTeX 干净。仅 1 处 minor（导航段 8 个 `\ref` 未用 `\cref`，属范围引用惯例，可不改）。
>
> 复核员：fresh 独立复核（未参与本轮润色）。只读，未改稿。复核日期 2026-06-18。
> 复核对象 1261 行全文 + 十四讲源 ch05 + 审计 ch05 节 + 编写规范 §1/§6/§7。

---

## 一、知识完整性 —— **结论：PASS（全量吸收且大幅增厚）**

逐项核对十四讲 ch5 的全部知识点，确认均在本章独立落地：

| 十四讲 ch5 知识点 | 本章位置 | 证据 |
|---|---|---|
| 针孔模型/相似三角形/倒像挪前方 | §针孔 `thm:pinhole` + `note`(L145) | 共线证明（L141）+ 正面投影模型 note，比原书更厚 |
| 内参 $f_x,f_y,c_x,c_y$ 来历与单位 | §内参化 `def:K` | $f_x=\alpha f$ 单位推导（L181）完整 |
| 径向/切向畸变 + 桶形/枕形 | §畸变 `eq:distortion` | L237 桶形/枕形、"过中心直线不变"均在 |
| 成像三步 + 5 畸变系数取舍 | `alg:cam-distort`(L272) + L278 | "只用 $k_1,p_1,p_2$"取舍在 |
| 去畸变两种方向 + 后文假定已去畸变 | `insight`(L287) + L291 约定 | 补"无解析解"洞察（增厚） |
| 外参/世界→相机/$\mathbf{KTP}_w$ 降维 | §外参 `thm:full-projection` + `pitfall`(L326) | "你能看出来吗"的隐藏降维升格为 pitfall |
| 四坐标链 + 单目成像全流程总结 | `fig:coord-chain`(L333) + `alg:cam-pipeline`(L375) | 四步逐一对照坐标系 |
| 归一化平面 + 深度丢失 = 单目测不出深度 | `insight`(L358) | 同乘常数不变的论证在 |
| 双目原理/基线/视差/$z=fb/d$ | §双目 `thm:stereo` | 相似三角形全步骤证明（L433） |
| 视差性质：成反比/最小1px/$z_{\max}=fb$/GPU | L441–442 | 全在，"人眼看飞机"直觉保留 |
| 视差计算难（对应关系/纹理/计算量） | L442 | "人类易计算机难"在 |
| RGB-D 结构光/ToF 两类 + 拆开有发射接收器 | §RGB-D L535–539 | Kinect1/2、RealSense、Tango 例全在 |
| RGB-D 深度→对齐→点云 | L539 + `thm:rgbd-backproj` | 反投影闭式 |
| RGB-D 局限（日光/互扰/透明/成本功耗） | L561 | 全在 |
| 图像作函数 $I(x,y):\mathbb{R}^2\to\mathbb{R}$ | §实践 `eq:image-function`(L944) | 在 |
| 二维数组 `[480][640]`/行=高列=宽 | L949–952 | "为什么是 480×640"段完整 |
| `image[y][x]` 像素访问 + x,y 陷阱 | L952 + `pitfall`(L961) | 见维度二 |
| 8/16/24 位：灰度/深度65535≈65m/彩色通道 | L954–955 | 全在 |
| OpenCV BGR 通道序/RGBA | L955 | 在 |
| imageBasics：读图判空/行指针遍历/深浅拷贝 | `codebox`(L968) + L987 | 12.7ms/Release 都在 |
| 去畸变代码（反查） | `codebox`(L996) + `note`(L1023) | "反查避空洞"补充洞察 |
| 双目 SGBM 视差→点云（除16/过滤） | `ex:cam-stereo` + L1027 | 在 |
| RGB-D 点云拼接（depthScale/四元数序/外参） | `ex:cam-rgbd` + L1027 | 四元数末位实部陷阱在 |
| 习题 5.2/5.4/5.5（分辨率/快门/Kinect标定） | `pitfall`(L212)、practice(L772/L576) | 全部落地 |

**`[minor][confirmed]` 唯一弱项（与审计一致，非缺口）**：OpenCV「从源码安装」逐步教程（apt 依赖串/下载 URL/make install 时长）未复现。本章以 `note`(L989) 一段交代"apt 预编译 vs 源码编译两路、按平台、以官方文档为准"。**此处理得当**——属一次性工具配置、与成像理论无关，符合"自包含理论教材"定位，且**不是 punt 到十四讲**（见维度三、四）。

**增厚部分**（超出十四讲、知识更全）：Barfoot 完整透视模型 $\mathbf{M}$、双目中点/左目/视差三式 + $\mathbf{M}$ 不可逆证明、本质/基础/单应矩阵含完整证明、投影雅可比 $\mathbf{S}$/BA 雅可比 $\mathbf{G}$/Schur 补、不确定度一/二/四阶 + sigmapoint；Handbook 抽象 $\pi/\pi^{-1}$、按 FOV 镜头谱系（rad-tan/BC/KB/DS）、重投影误差 MLE 推导、卷帘/光度模型。无 orphan formula。

---

## 二、正确性（抽查公式与数值例）—— **结论：PASS（全部核对无误）**

**针孔/畸变/双目公式**：
- `[OK]` 针孔倒像式 `eq:pinhole-neg` $Z/f=-X/X'=-Y/Y'$ — 与十四讲 5.1 逐字一致；正面投影 `eq:pinhole` $X'=fX/Z$ 共线证明（$\lambda=f/Z$）严密。
- `[OK]` 内参 `eq:intrinsic-scalar` $u=f_xX/Z+c_x$、`eq:K` $Z[u,v,1]^\top=\mathbf{KP}_c$ — 与十四讲 5.5/5.7 一致；单位推导 $f_x=\alpha f$（米×像素/米=像素）正确。
- `[OK]` 畸变 `eq:distortion`（径向 $1+k_1r^2+k_2r^4+k_3r^6$ + 切向 $2p_1xy+p_2(r^2+2x^2)$）— 与十四讲 5.10–5.12 逐项一致；作用在归一化平面、内参在后（`alg:cam-distort`）顺序正确。
- `[OK]` 双目视差 `eq:stereo` $z=fb/d,\ d:=u_L-u_R$ — 证明 L433 从 $\frac{z-f}{z}=\frac{b-(u_L-u_R)}{b}$ 推出，与十四讲 5.14（$\frac{b-u_L+u_R}{b}$）**代数恒等**（$b-u_L+u_R\equiv b-(u_L-u_R)$）。图注（L422 $d=u_L-u_R$、$u_R<0$）/定理/证明三处自洽。
- `[OK]` $z_{\max}=fb$（$d_{\min}=1$px）正确。
- `[OK]` Barfoot 双目 $\mathbf{M}$ `eq:stereo-M`、`prop:stereo-M`（第2、4行相同⇒$\mathbf{M}$不可逆⇒$v_r=v_\ell$）证明用 $\mathbf{F}$ 约束二次验证，自洽。
- `[OK]` 本质矩阵 `eq:essential` $\mathbf{E}_{ab}=\mathbf{R}_{ba}^\top(\mathbf{r}_b^{ab})^\wedge$ — 证明用 $\boldsymbol\rho_a=\mathbf{R}_{ba}^\top(\boldsymbol\rho_b-\mathbf{r}_b^{ab})$（即 $\boldsymbol\rho_b=\mathbf{R}_{ba}\boldsymbol\rho_a+\mathbf{r}_b^{ab}$ 的逆）正确，反对称恒等式 $\mathbf{u}^\top\mathbf{u}^\wedge=\mathbf{0}^\top$ 用对。基础矩阵 `eq:fundamental` $\mathbf{F}=\mathbf{K}_a^{-\top}\mathbf{E}\mathbf{K}_b^{-1}$、单应 `eq:homography` 推导均逐步无误。
- `[OK]` KB 鱼眼 `eq:kb` $r(\theta)=\theta+\sum_{n=1}^4 k_n\theta^{2n+1}$（奇次多项式）、双球 `eq:ds` — 标准形式正确。
- `[OK]` 投影雅可比 `eq:mono-jac` $\frac1z[f_u,0,-f_ux/z;0,f_v,-f_vy/z]$ — 逐项偏导（$\partial(x/z)/\partial z=-x/z^2$）正确；BA 雅可比 `eq:ba-jac` $\mathbf{G}=\mathbf{S}\mathbf{Z}$ 分块、Schur 补 `thm:ba-schur` 左乘消元推导无误。

**数值例（独立 Python 重算，全部吻合）**：
- `[OK]` `ex:cam-intr` 内参 + 畸变系数 = 十四讲 undistortImage.cpp L510–512 完全一致。
- `[OK]` `ex:cam-stereo` $d=48$：$718.856\times0.573/48=8.5813$m（文中"≈8.58"✓）。practice $(700,200),d=32$：$z=12.872,x=1.662,y=0.265$ 自洽。
- `[OK]` `ex:cam-rgbd` 内参 = 十四讲 jointMap.cpp L670–674；practice $(400,300),2500$mm：$X=0.360,Y=0.224,Z=2.5$ ✓。

**x,y 像素访问陷阱表述**：`[OK]` 准确。L952 "`image[y][x]` 对应 $I(x,y)$"、`pitfall`(L961) "数组下标 `image[行=y][列=x]`，与数学 $(x,y)$ 相反，编译器不报错只运行时越界" — 与十四讲 L320–322 一致且更显式；并正确补充 OpenCV `image.at<T>(v,u)` 是 `(row=v,col=u)` 的对照。无误导。

---

## 三、行文/脉络（本章重点）—— **结论：PASS（9 处独立改写后行文顺、过渡不断、四链完整）**

逐一核对审计 ch05 节点名的 9 处独立性改写，确认**残留转述措辞已全部清除**（grep `十四讲特意/提醒/坦言/强调/原文/在本节/你能看出来吗/主源` 全文**零命中**），且改写后行文自然：

| 审计原指（旧行号） | 现状（已改写为本书直述） | 过渡是否顺畅 |
|---|---|---|
| `:952` "十四讲特意强调：调换 x,y 是隐蔽错误" | L952 "调换 $x,y$ 顺序是隐蔽的常见 bug：编译器不会报错…`\cite`" | 顺，直述+脚注引 |
| `:385` "十四讲提醒读者务必厘清四种坐标" | L385 "务必厘清这四种坐标的关系`\cite`——它就是整个成像过程的骨架" | 顺 |
| `:373` "十四讲在本节末串成叙事，我们也照此" | L372 "针孔、内参、畸变、外参四块拼图已经齐了。下面把整个单目成像串成一条流程`\cite`" | **顺，且收束有力**（"闭着眼睛说出三维点怎么变像素"） |
| `:327` "十四讲原文也以'你能看出来吗？'提醒" | L327 升格为 `pitfall`"$\mathbf{KTP}_w$ 里藏着一次降维"，"初学者极易忽略`\cite`" | 顺，转述消失 |
| `:105` 记号对照 | L105 note 平衡多书对照（十四讲/Barfoot/Handbook 记号并列）+ `\cite` | 合法多书对照 |
| `:565` 内参 | `ex:cam-rgbd` L565 "取自`\cite` 的点云拼接数据集"，数据为主 | 顺，出处化 |
| `:966` "简化自十四讲 imageBasics" | L966 "数据/思路参考`\cite` 的 imageBasics"、codebox 标题"思路参考 imageBasics" | 顺，弱化叙述主体 |

**四坐标链叙事完整性**：贯穿全章主线连贯——
- 知识导航图（L46 tikz）+ 推荐路径（L72）建立"针孔→内参→畸变→坐标链→双目/RGB-D→多视图→现代谱系→观测模型/BA"主轴；
- `fig:coord-chain`(L333) 四坐标链图 + `alg:cam-pipeline`(L375) 四步流程 + 常见误解表 + 速查表前后呼应；
- 节间过渡句到位：L385 "别忘了第2步那个除以 $Z$——这正是下一节双目要补的窟窿"（→双目）；L399 "单目丢了深度（§链）"（承上）；L531 "双目被动算视差，弱纹理处失效。RGB-D 改为主动"（→RGB-D）；L584 "前几节讲一个点→一张图。但 SLAM 核心是多视图"（→对极）；L688 "针孔只是众多镜头之一"（→现代谱系）；L780 "前八节把点→像素正向讲透了。但后端要反过来"（→观测模型）。
- 记号 note(L105) 已预先厘清"相机坐标 $\mathbf{P}_c$（米）vs 归一化坐标 $[x,y,1]$（无量纲）严格区分"，并点名十四讲"成像总结"曾混用——此为合法的本书直述式辨析，反而强化了四链的清晰度。

**OpenCV 安装 note 是否得当**：`[OK]` 得当，非 punt。L989 note 给出"apt 预编译库（最省事）vs 源码编译（自选版本/开 GPU）"两条路 + "随平台版本而异、以官方文档为准、本书不展开"。这是合理的范围划定（一次性工具配置 ≠ 成像理论），**未**指向"详见十四讲 §5.3.1"，符合规范 §1.8 自包含铁律。

**复核无误**：未发现脉络断裂、过渡突兀、或改写造成的指代悬空。

---

## 四、独立性 + LaTeX —— **结论：PASS（独立性干净、LaTeX 一致）**

**独立性（narration/ventriloquize/punt 扫描）**：
- `[OK]` ventriloquize 残留：**零**。全文 grep "十四讲/Barfoot/Handbook + 坦言/强调/提醒/特意/原文" 无命中。
- `[OK]` 对 Barfoot/Handbook 的处理合规：均为本书直述 + `\cite` 出处，或合法平衡对照。例：L146 "Barfoot 称这一约定为正面投影模型`\cite`"、L324 "Barfoot 把这一链拆得更细`\cite`"、L444 "Barfoot 把双目写成统一矩阵模型`\cite`"、L791 "Handbook 写法等价`\cite`" — 全是"某书提出/采用 X 概念 + 引用"，非借其口吻叙事。`note`(L105/L203) 三书记号对照为规范 §3 鼓励的"平衡多书对照"。
- `[OK]` external_punt：**零**。grep "详见/见原书/参见…节/完整…见(十四讲/Barfoot/原书)" 无命中。所有"留 `\cref{ch:vo}`/`\cref{ch:calib}`"是**本书内部**前向指针（合法跨章去重，规范 §7 纪律），非指向外部著作。
- `[OK]` 元叙事词（G5 闸）：grep "本轮/润色/复核/审稿/审计/草稿/worker/TODO/待补/FIXME" 无命中；`\pz`/`\pzr`/`\rebuilt` 无残留。

**LaTeX 一致性**：
- `[OK]` `\cref` 全覆盖：244 处 `\cref` vs 8 处 `\ref`；所有交叉引用 label 解析——82 个 `\label` 全部被正确引用，**零悬空 `\cref`**；8 个跨章 `ch:` 目标（calib/eskf/lie/nlopt/rigid_body/slam_est/vio/vo）经全书 grep **全部存在**。
- `[OK]` 环境配平：`begin`/`end` 逐类型计数完全相等（theorem×11、proof×12、derivation×6、insight×9、pitfall×11、practice×12、note×7、codebox×2、algo×2、table/tabular×8、figure×2、tikzpicture×3、example×4、definition/proposition/corollary 各 1…）。无错配。
- `[OK]` 公式标号/盒子语义符合规范 §8（定理用 theorem、推导用 derivation 盒、直觉用 insight、陷阱用 pitfall、代码用 codebox、对照用 note）。

**`[minor]` 唯一可改项（不阻断、可不动）**：
- `[minor] L72, L75 — 8 个 `\ref{sec:...}` 未用 `\cref` — 规范 §2.3/§8 "禁手写章号、统一 `\cref`"。**但**此处全是"第 `\ref{}`--`\ref{}` 节"的**范围引用**（如"第 2--4 节"），裸 `\ref` 只出数字、配中文"第…节"读起来通顺，而 `\cref` 会插入"节"字导致"第 节2--节4 节"重复。这是范围引用的合理惯例，建议**保留现状**；若强求一致可改 `\crefrange{sec:cam-pinhole}{sec:cam-chain}`。不影响编译，不影响正确性。

---

## 复核小结

| 维度 | 结论 | 严重度 |
|---|---|---|
| 1 知识完整 | PASS（十四讲 ch5 全量 + 大幅增厚） | — |
| 2 正确性 | PASS（公式/数值例/x,y 陷阱全核对无误） | — |
| 3 行文/脉络 | PASS（9 处改写后行文顺、四链完整、OpenCV note 得当） | — |
| 4 独立性+LaTeX | PASS（ventriloquize/punt/元叙事零残留；cref 全解析、环境配平） | 1 minor（导航段 `\ref` 范围引用，建议保留） |

**唯一 minor**：`[minor] L72/L75 导航段 8 处 `\ref` — 范围引用惯例，建议保留，非缺陷。

本章为高质量成稿，建议直接进入编译验证（本环境无 Docker，未代跑 `compile.sh`；静态层面 cref/label/环境均已通过）。
