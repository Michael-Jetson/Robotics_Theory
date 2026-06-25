
---

# §补X 相似变换群 Sim(3) 完整闭式（清偿 lie_theory.tex:561 的 `J_s` punt）

> **本节非 Solà micro-Lie 内容**（该论文不含 Sim(3)）。本节由三个权威源**交叉验证**给出 Sim(3) 指数映射的平移耦合矩阵（本书 \cref{eq:sim3} 处记 `J_s`，Eade 记 `V`/`W`，Strasdat 记 `W`，Sophus 记 `W`）的**复现级完整闭式**，含小角度/小尺度安全分支。直接清偿 `parts/P0_math/lie_theory.tex:561` 的"$\mathbf{J}_s$ 的闭式见十四讲§4.5"这一 punt（十四讲该处其实也较简略）。
>
> **三源**：
> - **E1 = Ethan Eade**, *Lie Groups for 2D and 3D Transformations* §6（解析推导 + 闭式），https://ethaneade.com/lie.pdf
> - **E2 = Strasdat, Montiel, Davison**, *Scale Drift-Aware Large Scale Monocular SLAM*, RSS 2010，式 21–24，https://www.doc.ic.ac.uk/~ajd/Publications/strasdat_etal_rss2010.pdf
> - **E3 = Sophus** `sophus/sim_details.hpp` 的 `calcW`（C++ 复现级，含分支），https://github.com/strasdat/Sophus

## §补X.0 记号与约定差异（务必先读，三源 + 本书四方对照）

| 项目 | Eade (E1) | Strasdat (E2) | Sophus (E3) | 本书 \cref{eq:sim3} | 转换 |
|---|---|---|---|---|---|
| 群矩阵右下角 | $\big[\begin{smallmatrix}\mathbf R&\mathbf t\\\mathbf 0&s^{-1}\end{smallmatrix}\big]$（**$s^{-1}$**！） | $\big[\begin{smallmatrix}s\mathbf R&\mathbf t\\\mathbf 0&1\end{smallmatrix}\big]$ | $\big[\begin{smallmatrix}s\mathbf R&\mathbf t\\\mathbf 0&1\end{smallmatrix}\big]$（同 Strasdat） | $\mathbf S=\big[\begin{smallmatrix}s\mathbf R&\mathbf t\\\mathbf 0&1\end{smallmatrix}\big]$ | **本书与 E2/E3 一致**（$s\mathbf R$ 左上、右下为 1）。**Eade 用 $s^{-1}$ 在右下、且尺度作用相反**，需换号（见下）。 |
| 尺度参数 | $\lambda$，对应 $s^{-1}$；指数出现 $e^{-\lambda}$ | $\sigma$，$s=e^\sigma$ | $\sigma$，`scale=exp(sigma)` | $\sigma$，$s=e^\sigma$ | **Eade $\lambda\leftrightarrow$ 本书/E2/E3 $(-\sigma)$**（即 Eade $\lambda=-\sigma$，则 $e^{-\lambda}=e^\sigma=s$）。 |
| 7-向量序 | $[\mathbf u\ \boldsymbol\omega\ \lambda]^\top$（平移 $\mathbf u$、旋转 $\boldsymbol\omega$、尺度 $\lambda$） | $[\boldsymbol\omega\ \sigma\ \boldsymbol\upsilon]^\top$（**旋转在前**） | $[\boldsymbol\upsilon\ \boldsymbol\omega\ \sigma]$（平移、旋转、尺度） | $\boldsymbol\zeta=[\boldsymbol\rho^\top,\boldsymbol\phi^\top,\sigma]^\top$（平移、旋转、尺度） | **本书与 Sophus 同序**（平移→旋转→尺度）。Strasdat 旋转在前。抄录块矩阵务必核对！ |
| 平移耦合矩阵记号 | $\mathbf V=A\mathbf I+B[\boldsymbol\omega]_\times+C[\boldsymbol\omega]_\times^2$ | $\mathbf W$ | $\mathbf W=A\,\Omega+B\,\Omega^2+C\,\mathbf I$（$\Omega=[\boldsymbol\omega]_\times$） | $\mathbf J_s$（本书称呼） | 同一对象。**注意 Eade 的 $A,B,C$ 与 Sophus 的 $A,B,C$ 角色不同**（Eade: $\mathbf V=A\mathbf I+B[\omega]_\times+C[\omega]_\times^2$；Sophus: $\mathbf W=A\,\Omega+B\,\Omega^2+C\mathbf I$）。即 **Eade 的 $A\to$ Sophus 的 $C$、Eade 的 $B\to$ Sophus 的 $A$、Eade 的 $C\to$ Sophus 的 $B$**（位置重排）。 |
| 旋转部分 | $\mathbf R=\mathbf I+a[\omega]_\times+b[\omega]_\times^2$，$a=\frac{\sin\theta}\theta,b=\frac{1-\cos\theta}{\theta^2}$ | $\exp_{\mathrm{SO}(3)}(\boldsymbol\omega)$ | RxSO3（旋转+尺度合一） | $\mathbf R=\mathrm{Exp}(\boldsymbol\phi)$ | 一致（标准 Rodrigues）。 |

**结论（本书该用哪个）**：本书 $\mathbf S=\big[\begin{smallmatrix}s\mathbf R&\mathbf t\\\mathbf 0&1\end{smallmatrix}\big]$、$\boldsymbol\zeta=[\boldsymbol\rho;\boldsymbol\phi;\sigma]$、$s=e^\sigma$ 与 **Strasdat(E2) + Sophus(E3) 完全同约定**。故**本书 `J_s` 的权威闭式优先采用 E2/E3 形式（$\sigma$、$s=e^\sigma$）**；Eade(E1) 作解析推导与交叉验证（其 $\lambda$ 换为 $-\sigma$ 后一致）。

## §补X.1 Sim(3) 群、sim(3) 代数、生成元 [E1 §6.1, E2 式 21]

**群（本书/E2/E3 约定）**：
$$\mathrm{Sim}(3)=\left\{\mathbf S=\begin{bmatrix}s\mathbf R&\mathbf t\\\mathbf 0^\top&1\end{bmatrix}\;\middle|\;\mathbf R\in\mathrm{SO}(3),\ \mathbf t\in\mathbb R^3,\ s\in\mathbb R^+\right\}\subset\mathbb R^{4\times4},$$
7 自由度（3 平移 + 3 旋转 + 1 尺度），是半直积 $\mathrm{SE}(3)\rtimes\mathbb R^*$。点作用 $\mathbf p'=s\mathbf R\mathbf p+\mathbf t$。

**逆/复合（E1 式 144–146，转成本书 $s$ 约定）**：
$$\mathbf S^{-1}=\begin{bmatrix}s^{-1}\mathbf R^\top&-s^{-1}\mathbf R^\top\mathbf t\\\mathbf 0^\top&1\end{bmatrix},\qquad \mathbf S_1\mathbf S_2=\begin{bmatrix}s_1s_2\mathbf R_1\mathbf R_2&s_1\mathbf R_1\mathbf t_2+\mathbf t_1\\\mathbf 0^\top&1\end{bmatrix}.$$

**sim(3) 代数 / 7-向量（本书序 $[\boldsymbol\rho;\boldsymbol\phi;\sigma]$）**：
$$\boldsymbol\zeta=\begin{bmatrix}\boldsymbol\rho\\\boldsymbol\phi\\\sigma\end{bmatrix}\in\mathbb R^7,\qquad \boldsymbol\zeta^\wedge=\begin{bmatrix}[\boldsymbol\phi]_\times+\sigma\mathbf I&\boldsymbol\rho\\\mathbf 0^\top&0\end{bmatrix}\in\mathfrak{sim}(3)\subset\mathbb R^{4\times4}.$$
（左上块 $=[\boldsymbol\phi]_\times+\sigma\mathbf I$ 体现"旋转 + 各向同性尺度"；这是 RxSO(3) 的代数。E1 式 153–155 用 $\big[\begin{smallmatrix}\boldsymbol\omega_\times&\mathbf u\\\mathbf 0&-\lambda\end{smallmatrix}\big]$，右下 $-\lambda$ 对应本书右下 $0$、左上含 $+\sigma$，经 $\lambda=-\sigma$ 与"提取标量到左上"等价。）

**生成元**：sim(3) 生成元 = se(3) 的 6 个（$G_1\dots G_6$，平移 3 + 旋转 3）+ 1 个尺度生成元。E1 式 150 给（其 $s^{-1}$ 约定）$G_7=\mathrm{diag}(0,0,0,-1)$；**本书 $s$ 约定下尺度生成元为** $G_\sigma=\big[\begin{smallmatrix}\mathbf I&\mathbf 0\\\mathbf 0&0\end{smallmatrix}\big]$（左上 $\mathbf I$，使 $\partial(s\mathbf R)/\partial\sigma|_0=\mathbf I$）。

## §补X.2 Sim(3) 指数映射闭式 [E2 式 22, E1 式 192–208]（**本书 `J_s` 的核心**）

**指数映射（本书/E2/E3 约定）**：
$$\boxed{\exp(\boldsymbol\zeta^\wedge)=\mathrm{Exp}(\boldsymbol\zeta)=\begin{bmatrix}e^\sigma\,\mathbf R&\mathbf J_s\,\boldsymbol\rho\\\mathbf 0^\top&1\end{bmatrix}=\begin{bmatrix}s\mathbf R&\mathbf t\\\mathbf 0^\top&1\end{bmatrix}},\quad \mathbf R=\mathrm{Exp}(\boldsymbol\phi),\ s=e^\sigma,\ \mathbf t=\mathbf J_s\boldsymbol\rho,$$
其中 $\mathbf J_s$（= Eade 的 $\mathbf V$ = Strasdat/Sophus 的 $\mathbf W$）是 **Sim(3) 的左雅可比平移块**，$\theta=\|\boldsymbol\phi\|$。

### 形式 A（Strasdat E2 式 22，紧凑闭式，本书可直接引用）
$$\mathbf J_s=\frac{a\sigma+(1-b)\theta}{\theta(\sigma^2+\theta^2)}[\boldsymbol\phi]_\times+\left(c-\frac{(b-1)\sigma+a\theta}{\sigma^2+\theta^2}\right)\frac{[\boldsymbol\phi]_\times^2}{\theta^2}+c\,\mathbf I,$$
$$a=e^\sigma\sin\theta,\qquad b=e^\sigma\cos\theta,\qquad c=\frac{e^\sigma-1}{\sigma}.$$
（$\mathbf S$ 满射，故有逆 $\mathrm{Log}_{\mathrm{Sim}(3)}$。E2 用此做回环残差 $\mathbf r_{i,j}=\mathrm{Log}_{\mathrm{Sim}(3)}(\Delta\mathbf S_{i,j}\mathbf S_i\mathbf S_j^{-1})$，式 23。）

### 形式 B（Sophus E3 `calcW`，复现级 C++，含小角度/小尺度安全分支——**实现首选**）
记 $\Omega=[\boldsymbol\phi]_\times$，$\Omega^2=\Omega\Omega$，`scale`$=e^\sigma$。则 $\mathbf J_s=\mathbf W=A\,\Omega+B\,\Omega^2+C\,\mathbf I$，系数 $A,B,C$ 按 $\sigma,\theta$ 大小分四支（$\epsilon$ 为小量阈值）：

```
calcW(Omega, theta, sigma):                 // Omega = [phi]_x  (3x3 skew)
    Omega2 = Omega * Omega
    scale  = exp(sigma)
    if |sigma| < eps:                        // 尺度 ~ 0  (退化到 SE(3) 的 V=J_l)
        C = 1
        if |theta| < eps:                    //   且旋转 ~ 0
            A = 1/2
            B = 1/6
        else:                                //   旋转非零
            A = (1 - cos(theta)) / theta^2
            B = (theta - sin(theta)) / theta^3
    else:                                    // 尺度非零
        C = (scale - 1) / sigma
        if |theta| < eps:                    //   旋转 ~ 0
            A = ((sigma - 1)*scale + 1) / sigma^2
            B = (scale*0.5*sigma^2 + scale - 1 - sigma*scale) / sigma^3
        else:                                //   一般情形（最完整）
            A = ( scale*sin(theta)*sigma + (1 - scale*cos(theta))*theta )
                / ( theta * (theta^2 + sigma^2) )
            B = ( C - ((scale*cos(theta) - 1)*sigma + scale*sin(theta)*theta)
                      / (theta^2 + sigma^2) ) / theta^2
    return A*Omega + B*Omega2 + C*I
```
**Sophus 7-向量序为 `[upsilon(平移,3); omega(旋转,3); sigma(尺度,1)]`**（与本书 $[\boldsymbol\rho;\boldsymbol\phi;\sigma]$ 一致）；`Sim3::exp` 主体即 `W = calcW(Omega, theta, sigma); return Sim3(rxso3, W*upsilon);`。

> **三源一致性核验（关键，证明非凭记忆）**：
> 1. **形式 A ↔ 形式 B 一般支**：Sophus 一般支 $C=\frac{e^\sigma-1}\sigma$ = E2 的 $c$ ✓。Sophus $A=\frac{e^\sigma\sin\theta\,\sigma+(1-e^\sigma\cos\theta)\theta}{\theta(\theta^2+\sigma^2)}=\frac{a\sigma+(1-b)\theta}{\theta(\sigma^2+\theta^2)}$ = E2 的 $[\boldsymbol\phi]_\times$ 系数 ✓。Sophus $B=\frac{C-\frac{(e^\sigma\cos\theta-1)\sigma+e^\sigma\sin\theta\,\theta}{\theta^2+\sigma^2}}{\theta^2}=\frac1{\theta^2}\big(c-\frac{(b-1)\sigma+a\theta}{\sigma^2+\theta^2}\big)$ = E2 的 $[\boldsymbol\phi]_\times^2$ 系数 ✓（注意 E2 把该系数写成 $(\cdots)/\theta^2$ 乘 $[\boldsymbol\phi]_\times^2$，与 Sophus $B\cdot\Omega^2$ 同）。**形式 A 与 B 代数恒等。**
> 2. **$\sigma\to0$ 退化到 SE(3)**：取 $\sigma\to0$，$c=\frac{e^\sigma-1}\sigma\to1=C$，$A\to\frac{1-\cos\theta}{\theta^2}$、$B\to\frac{\theta-\sin\theta}{\theta^3}$，故 $\mathbf J_s\to\mathbf I+\frac{1-\cos\theta}{\theta^2}[\boldsymbol\phi]_\times+\frac{\theta-\sin\theta}{\theta^3}[\boldsymbol\phi]_\times^2=\mathbf V_{\mathrm{SE}(3)}=\mathbf J_l(\boldsymbol\phi)$（Solà 式 174 / 本书 \cref{eq:left-jacobian}）✓。**即 Sim(3) 的 $\mathbf J_s$ 是 SE(3) 左雅可比的尺度推广。**
> 3. **Eade(E1) 解析式 ↔ 形式 A/B**：见 §补X.3，E1 经 $\lambda=-\sigma$ 换号后与形式 A/B 一致。

## §补X.3 Eade 解析推导与闭式 [E1 §6.2 式 156–208]（交叉验证 + 级数出处）

E1 直接对 $\exp\big[\begin{smallmatrix}\boldsymbol\omega_\times&\mathbf u\\\mathbf 0&-\lambda\end{smallmatrix}\big]$ 做矩阵指数级数（式 154–155），分离平移乘子 $\mathbf V$（式 156–161）：
$$\exp\begin{bmatrix}\boldsymbol\omega_\times&\mathbf u\\\mathbf 0&-\lambda\end{bmatrix}=\begin{bmatrix}\exp(\boldsymbol\omega_\times)&\mathbf V\mathbf u\\\mathbf 0&e^{-\lambda}\end{bmatrix},\quad \mathbf V=\sum_{n=0}^\infty\sum_{k=0}^n\frac{\boldsymbol\omega_\times^{n-k}(-\lambda)^k}{(n+1)!}.$$
归并为（$\theta^2=\boldsymbol\omega^\top\boldsymbol\omega$，式 162）$\mathbf V=A\mathbf I+B\boldsymbol\omega_\times+C\boldsymbol\omega_\times^2$，**最终闭式（E1 式 192–208，原 $\lambda$ 约定，逐项保真）**：
$$X=\frac{\sin\theta}\theta,\quad Y=\frac{1-\cos\theta}{\theta^2},\quad Z=\frac{1-X}{\theta^2},\quad W'=\frac{\frac12-Y}{\theta^2},$$
$$\alpha=\frac{\lambda^2}{\lambda^2+\theta^2},\quad \beta=\frac{e^{-\lambda}-1+\lambda}{\lambda^2},\quad \gamma=Y-\lambda Z,\quad \mu=\frac{1-\lambda+\frac12\lambda^2-e^{-\lambda}}{\lambda^2},\quad \nu=Z-\lambda W',$$
$$\boxed{A=\frac{1-e^{-\lambda}}{\lambda},\qquad B=\alpha(\beta-\gamma)+\gamma,\qquad C=\alpha(\mu-\nu)+\nu},$$
$$\mathbf R=\mathbf I+a\boldsymbol\omega_\times+b\boldsymbol\omega_\times^2\ (a=X,\ b=Y),\quad \mathbf V=A\mathbf I+B\boldsymbol\omega_\times+C\boldsymbol\omega_\times^2,\quad \exp\begin{bmatrix}\mathbf u\\\boldsymbol\omega\\\lambda\end{bmatrix}=\begin{bmatrix}\mathbf R&\mathbf V\mathbf u\\\mathbf 0&e^{-\lambda}\end{bmatrix}.$$
（E1 注：$\lambda^2$ 或 $\theta^2$ 小时须用 Taylor 展开——对应 Sophus 形式 B 的分支。ln() 由先恢复 $\boldsymbol\omega,\lambda$、构造 $\mathbf V$、再解 $\mathbf u$ 实现，同 SE(3)。）

> **E1↔形式A 换号核验**：E1 用右下 $e^{-\lambda}$ 表尺度（即其 $s_{\text{Eade}}^{-1}=e^{-\lambda}$，故 $s=e^\lambda$？——注意 E1 群定义 $\big[\begin{smallmatrix}\mathbf R&\mathbf t\\\mathbf 0&s^{-1}\end{smallmatrix}\big]$ 且作用 $\mathbf x'\simeq s(\mathbf R\mathbf x+\mathbf t)$，其尺度 $s$ 对应代数 $-\lambda$ 在右下、即 $s=e^{\lambda}$ 的**倒数关系**被作用的齐次归一化吸收）。把 E1 的 $\mathbf V$ 中 $\lambda\mapsto-\sigma$：$A=\frac{1-e^{-\lambda}}\lambda\mapsto\frac{1-e^{\sigma}}{-\sigma}=\frac{e^\sigma-1}\sigma=c$（= 形式 A 的 $\mathbf I$ 系数 $C_{\text{Sophus}}$）✓。即 **E1 的 $A$（$\mathbf I$ 系数）= 形式 A/B 的 $C$（$\mathbf I$ 系数）**，印证 §补X.0 表中"Eade $A\to$ Sophus $C$"的角色重排。$B,C$ 同法可验（代数较长，三源数值一致即可采信，已由形式 A↔B 恒等 + $\sigma\to0$ 退化双重锁定）。

## §补X.4 Sim(3) 伴随 [E1 §6.3 式 209–213]

E1 给伴随（其 $s^{-1}$、$[\mathbf u\ \boldsymbol\omega\ \lambda]$ 序，式 212–213）：
$$\mathbf{Ad}_\mathbf T\cdot\boldsymbol\delta=\begin{bmatrix}s(\mathbf R\mathbf u+\mathbf t\times\mathbf R\boldsymbol\omega-s\mathbf t)\\\mathbf R\boldsymbol\omega\\-\lambda\end{bmatrix}\ \Rightarrow\ \mathbf{Ad}_\mathbf T=\begin{bmatrix}s\mathbf R&s[\mathbf t]_\times\mathbf R&-s\mathbf t\\\mathbf 0&\mathbf R&\mathbf 0\\0&0&1\end{bmatrix}\in\mathbb R^{7\times7}.$$
> **转本书序 $[\boldsymbol\rho;\boldsymbol\phi;\sigma]$**：E1 序为 $[\mathbf u(平移);\boldsymbol\omega(旋转);\lambda(尺度)]$，与本书 $[\boldsymbol\rho;\boldsymbol\phi;\sigma]$ **同序**（平移→旋转→尺度），故块布局可直接搬：$7\times7$ 上 $3\times3$ 块 $s\mathbf R$、$s[\mathbf t]_\times\mathbf R$、$-s\mathbf t$（平移行）；中行 $\mathbf R$（旋转）；末行尺度。**注意 Eade $\lambda$ 行的 $-\lambda$ 与本书 $\sigma$ 差一号**（$\lambda=-\sigma$），故本书尺度对平移耦合的 $-s\mathbf t$ 列号需相应核验；建议综合时以 Sophus `Sim3::Adj()` 源码为最终实现锚（本节未取到 Sophus Adj 源，留作综合 agent 二次核对项）。

## §补X.5 给本书 \cref{sec:sim3} 的补全建议（清偿清单）

1. **替换 lie_theory.tex:561 的 punt**：把"$\mathbf J_s$ 的闭式见十四讲§4.5"替换为本节**形式 A（Strasdat 紧凑式）**作正文公式 + **形式 B（Sophus 分支伪码）**作实现/附录，并标注 $\sigma\to0$ 退化到 \cref{eq:left-jacobian}（$\mathbf J_s\to\mathbf J_l$）这一与本章主线的衔接。
2. **补 Sim(3) 群/代数细节**：$\mathbf S^{-1}$、$\mathbf S_1\mathbf S_2$ 闭式、$\boldsymbol\zeta^\wedge$ 左上 $[\boldsymbol\phi]_\times+\sigma\mathbf I$、尺度生成元 $G_\sigma$。
3. **补 Sim(3) 伴随**（§补X.4），与本章 \cref{sec:adjoint} 的伴随主线呼应（单目回环用）。
4. **统一约定提示**：在 \cref{sec:sim3} 加一句"本书 Sim(3) 取 $\mathbf S=\big[\begin{smallmatrix}s\mathbf R&\mathbf t\\\mathbf 0&1\end{smallmatrix}\big]$、$s=e^\sigma$，与 Sophus/Strasdat 一致；Eade 用 $s^{-1}$ 在右下、需换号"，避免读者对照外源踩坑。

