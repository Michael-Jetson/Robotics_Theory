# SLAM理论

# 概述

这种机器人的自主运动过程中，有两大基本问题——我在什么地方？周围环境是什么样子的？也就是定位和建图的问题，而想完成定位与建图问题，其中定位侧重于对自身的了解，建图侧重于对外在环境的了解，二者相互关联，而这，就需要依靠传感器完成对外界环境的感知，也就是说，在没有环境先验信息的情况下，在运动过程中建立环境的模型，同时估计自己的运动

传感器主要是有两种：

\(1\) 携带于机器人本体上的传感器，例：机器人的轮式编码器、相机、激光传感器、惯性测量单元\(IMU\)等

\(2\) 安装于环境之中的传感器，例：导轨、二维码标志等

但是环境传感器是有限制的，比如说GPS需要在能接收到卫星信号的环境下才可以工作

而本体上传感器具有以下优点：激光、相机等携带式传感器测量的通常都是一些间接的物理量而不是直接的位置数据，所以更加自由使用携带式传感器来完成SLAM也是我们重点关注的问题

## slam介绍

slam是什么呢？slam就是同时建图与定位，我们先不用学术的说法去介绍它，而是先用实际生活中的例子来介绍。

大家知道，如果你在一个地方生活久了，就会很熟悉这个地方，可能随便把你扔在其中某一个地点，你就可以知道你在什么位置，你应该怎么走回家，但是如果你出现在一个陌生的没去过的地方，你可能就一时半会不知道自己的位置了。

这个过程其实就是一个slam的过程，你在熟悉的环境下走路，相当于已经有一个地图在你大脑里面了，你需要根据眼睛看到的信息（相当于传感器感知环境）来判断自己在这个地图的什么地方，然后就可以进行自主导航了。

## 视觉传感器

视觉SLAM是本书的主题，所以我们非常关心小萝卜的眼睛能够做些什么事。即如何用相机解决定位和建图的问题。

- SLAM中使用的相机更加简单，以一定速率采集图像、形成视频,相机的特点就是以二维投影形式记录了三维世界的信息

- 但是该过程丢掉了一个维度：距离\(或深度\)

相机分类

- 单目相机 Monocular：只使用一个摄像头的相机，通过相机的运动形成视差，可以测量物体相对深度

    - 优点：结构简单，成本低，便于标定和识别

    - 缺点：在单张图片里，无法确定一个物体的真实大小。它可能是一个很大但很远的物体，也可能是一个很近很小的物体，即单目SLAM估计的轨迹和地图将与真实的轨迹和地图相差一个因子也就是尺度\(scale\)，单凭图像无法确定这个真实尺度，所以称尺度不确定性。

- 双目相机\(立体相机\)Stereo：由两个单目相机组成，通过基线来估计每个像素的空间位置\(类似于人眼\)

    - 优点:基线距离越大，能够测量的距离就越远;并且可以运用到室内和室外。

    - 缺点:配置与标定较为复杂，深度量程和精度受到双目基线与分辨率限制，计算非常消耗计算资源，需要GPU\(图形处理器\)/FPGA设备\(现场可编程门阵列\)加速用两部相机来定位。

- 深度相机 RGB\-D：通过红外结构光或ToF\(time of fly\)的物理方法测量物体深度信息

    - 优点:相比于双目相机可节省大量的计算资源。

    - 缺点:是测量范围窄，噪声大，视野小，易受日光干扰，无法测量透射材质等问题，主要用在室内，室外很难应用。
    深度相机主要用来三维成像，和距离的测量。

- 其他相机：全景相机、Event Camera（事件相机）

## SLAM流程

1、传感器信息读取:在视觉SLAM中主要为相机图像信息的读取和预处理。

2、前端视觉里程计\(Visual Odometry，VO\)视觉里程计的任务是估计相邻图像间相机的运动，以及局部地图的样子。\(VO又称为前端\)

3、回环检测\(Loop Closure Detection\)用于判断机器人是否到达过先前的位置。

4、后端\(非线性\)优化\(optimization\)。对不同时刻的视觉里程计测量的相机位姿及回环检测的信息进行优化，得到全局一致的轨迹和地图。

5、建图\(Mapping\)。根据估计的轨迹，建立任务要求对应的地图。

### 前端视觉里程计

视觉里程计通过相邻帧间的图像估计相机运动，并恢复场景的空间结构，但只计算相邻时刻的运动，不关心再往前的信息。但前端过程中必然存在误差，误差会不断累积，形成累积漂移\(会发现原本直的走廊变成了斜的，而原本90°的直角变成一歪的\)。为消除漂移，我们需要回环检测和后端优化。

### 后端优化

定义:如何处理前端所传噪声的数据，从带有噪声的数据中估计整个系统的状态，以及这个状态估计的不确定性有多大——称为最大后验概率估计

通常来说，前端给后端提供待优化的数据，以及这些数据的初始值。后端负责整体的优化过程，它往往面对的就只有数据。其反映了SLAM问题的本质:对运动主体自身和周围环境空间不确定性的估计。\(状态估计理论——估计状态的均值和不确定性\)

### 回环检测

回环检测的作用:主要解决位置估计随时间漂移的问题\(通俗的理解就是，假设机器人经过一段时间又回到了原点\(事实\)，但是我们的位置估计值没有回到原点，怎么解决\)

回环检测要达到的目标:通过某种手段，让机器人知道“回到原点”这件事情，让机器人具有识别到过的场景的能力。再把位置估计值“拉”过去

相机检测手段:判断与之前位置的差异，计算图像间相似性。

回环检测后:可将所得的信息告诉后端优化算法，把轨迹和地图调整到符合回环检测结果的样子。

### 地图

地图大体上可分为以下两类:

1. 度量地图\(强调精确的表示地图中的位置关系\)，常用稀疏与稠密进行分类
稀疏地图:即由路标组成的地图
稠密地图:着重于建模所有看到的东西\(可用于导航\)\(耗费大量的储存空间\)

2. 拓扑地图\(更加强调元素之间的关系\)，是一个图:由节点和边组成，例:只关注A、B点是连通的，而不考虑如何从A点到达B点，不适用于表达较为复杂结构的地图

## 传感器对比

激光雷达与相机对比

- 激光雷达有效距离远，可以达到百米级别（可达300m），相机有效距离相对近

- 雷达贵，动辄上万，相机较为便宜，高速相机不过上千

- 激光雷达受天气影响较大，雨雪都会成为巨大干扰，因为雨雪也会反射激光

- 雷达重，激光较轻

- 在纹理不清楚或者光线过强过弱的地方，相机几乎无法运作

## SLAM数学表述

但是只有对各个模块的组成和功能有一定的认知还是不够的，我们无法根据这种直观了解写出可以运行的程序，需要上升到数学层面进行描述和建模
首先，机器人会携带某种传感器在未知环境中运动，如何用数学语言描述这件事呢，我们知道，相机通常是在某些时刻采集数据的，所以我们也关心这些时刻的位置和地图，也就是一段连续时间的运动变成了离散时刻当中发生的事，在这些时刻，使用 $\boldsymbol x$ 表示机器人的位置，使用下标来区分不同时刻的位置，这些时刻的位置就构成了机器人的轨迹

假设地图是由许多个路标组成，每个时刻，传感器会测量到一部分路标点，得到他们的观测数据，设路标点有N个，用 $\boldsymbol{y}_1,\boldsymbol{y}_2,\cdots,\boldsymbol{y}_N$ 表示

这样，我们需要考虑以下两件事情，还真是，机器人带着传感器在环境中运动是由如下两件事描述的：

1. 什么是运动？从 $k-1$ 时刻到 $k$ 时刻，小萝卜的位置是如何变化的？

2. 什么是观测？假设小萝卜在 $k$ 时刻位于 $\boldsymbol{x}_k$ 处观测到了某个路标 $\boldsymbol{y}_k$ ，我们如何用数学语言描述呢？

首先，运动模型可以描述运动，资料机器人会携带一个测量自身运动的传感器，这个传感器可以测量有关运动的读数，但不一定直接就是位置之差，还可能是加速度、角速度这些信息。我们可以用一个抽象的数学模型来描述

$\boldsymbol{x}_k = f(\boldsymbol{x}_{k-1}, \boldsymbol{u}_k, \boldsymbol{w}_k) $

$\boldsymbol{u}_k$ 是运动传感器的读数，$ \boldsymbol{w}_k $ 是运动过程中加入的噪声（因为真实物理世界中的传感器都会带有噪声）

然后是观测模型，机器人在 $\boldsymbol{x}_k$ 位置上看到某个路标点 $\boldsymbol{y}_j$，产生了一个观测数据 $\boldsymbol{z}_{k,j}$，同样可以用一个抽象的数学模型描述：

$\boldsymbol{z}_{kj} = h(\boldsymbol{y}_j, \boldsymbol{x}_k, \boldsymbol{v}_{k,j}) \quad $

$\boldsymbol{v}_{k,j}$ 是观测的噪声

实际上，这只是一个简化的方程，描述了一种形式，在真实世界中，根据机器人的真实运动和传感器的种类，存在着若干种参数化形式。而考虑视觉SLAM时，传感器是相机，则观测方程就是“对路标点拍摄后，得到图像中的像素”的过程，若考虑激光SLAM时，传感器是激光雷达，则观测方程就是对路标点（或环境表面）进行扫描后，得到该点在雷达坐标系下的三维坐标（或距离与方位角）

### 运动方程

这里举一个运动方程的例子，假设机器人在平面运动，那么位姿（位置\+姿态）由两个位置的坐标和一个转角来描述 $\boldsymbol x_k = (x, y, \theta)^T$，同时，运动传感器能够测量到机器人在任意两个时间间隔位置和转角的变化量 $\boldsymbol u_k = (\Delta x, \Delta y, \Delta \theta)^T$，于是，此时运动方程就可以写成：

$\begin{pmatrix}
x \\
y \\
\theta
\end{pmatrix}_{k} =
\begin{pmatrix}
x \\
y \\
\theta
\end{pmatrix}_{k-1} +
\begin{pmatrix}
\Delta x \\\Delta y \\\Delta \theta\end{pmatrix}_{k} + \boldsymbol w_k$

学过现代控制理论、理论力学等课程的同学可能会对这种方程有印象，实际上运动方程本质上是通过系统运行的物理规律构建出的方程，而且上面的方程是一种很简单的线性方程，但是并不是所有的输入指令都会如此简单，诸如油门和操纵杆的输入就是速度或者加速度量，并且也有其他的更加复杂的运动方程形式

### 观测方程

关于观测方程，比如机器人携带着一个二维激光传感器（当激光传感器观测一个2D路标点时，可以测到路标点与机器人之间的距离 $r$ 和夹角 $\phi$，记路标点 $\boldsymbol y_j = [y_1, y_2]^T_j$，位姿为 $\boldsymbol x_k = [x_1, x_2]^T_k$，观测数据为 $\boldsymbol z_{k,j} = [r_{k,j}, \phi_{k,j}]^T$，那么观测方程可以写成

$\begin{bmatrix}
r_{k,j} \\\phi_{k,j}
\end{bmatrix}_{k} =
\begin{bmatrix}
\sqrt{(y_{1,j} - x_{1,k})^2 + (y_{2,j} - x_{2,k})^2}  \\\arctan \left( \frac{y_{2,j} - x_{2,k}}{y_{1,j} - x_{1,k}} \right) 
\end{bmatrix}_{k} +\boldsymbol v$

### SLAM基本方程

针对不同的传感器，两个方程有不同的参数化形式，如果出于通用性考虑，那么就可以对其进行抽象，并且总结为两个基本方程

$\begin{cases} 
\boldsymbol{x}_{k}=f\left(\boldsymbol{x}_{k-1},\boldsymbol{u}_{k},\boldsymbol{w}_{k}\right), & k=1,\cdots,K \\ 
\boldsymbol{z}_{k,j}=h\left(\boldsymbol{y}_{j},\boldsymbol{x}_{k},\boldsymbol{v}_{k,j}\right), & (k,j)\in O 
\end{cases}$

*O* 表示观测集合，即哪些时刻观测到了哪些路标

# 三维空间刚体运动

想描述三维空间中的物体运动，就必须先确定坐标系的概念，因为运动都是相对的，我们无法描述一个物体的绝对运动情况，只能描述一个物体的相对运动情况

## 点与坐标系

三维空间由3个轴组成，则一个空间点的位置可以由3个坐标指定，对于一个刚体（在运动和受力作用后，形状和大小不变，而且内部各点的相对位置不变的物体，与软体相对），不光有位置，还有自身的姿态，如相机可以看成三维空间中的刚体，则位置就是说相机在空间中的哪个地方，姿态是指相机的朝向（相机处于空间\(0，0，0\)处，朝向正前方），这些情况都需要使用数学语言进行描述

- 点：没有长度，没有体积，但是点和点可以组成向量

- 向量是带指向性的箭头\(有方向性\)，可以进行加法、减法等运算

- 坐标：当我们指定坐标系后，才可以谈论该向量在此坐标系下的坐标

- 坐标系：实际上是构成线性空间的一组基，分为左手系和右手系，在机器人领域，一般使用右手系，机器人的运动也都是在右手系里面进行讨论

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NzY5YzViNWU1ZWY5NTFjZWE2NzM5NTdhYjRiNmExYjBfY2RmZjE1MDI2Y2ZkMTViM2U0NTAyNzAzZmM3ZmQ5MDVfSUQ6NzU3OTIwODUzMDg4NTEzNTU4MF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

对于向量的运算，如内积、外积，这里不再赘述，但是对于外积，这里引入一个特殊的记法，如下

$\boldsymbol a \times \boldsymbol b = 
\begin{bmatrix}
i & j & k \\
a_1 & a_2 & a_3 \\
b_1 & b_2 & b_3
\end{bmatrix} =
\begin{bmatrix}
a_2 b_3 - a_3 b_2 \\
a_3 b_1 - a_1 b_3 \\
a_1 b_2 - a_2 b_1
\end{bmatrix} =
\begin{bmatrix}
0 & -a_3 & a_2 \\
a_3 & 0 & -a_1 \\
-a_2 & a_1 & 0
\end{bmatrix} \boldsymbol b \triangleq \boldsymbol a^\wedge \boldsymbol b$

也就是将叉乘写成矩阵相乘的方式，并且对应的叉乘矩阵 $\boldsymbol a^\wedge$ 实际上是一个反对称矩阵，也就是满足

$(\boldsymbol a^\wedge)^{-1}=(\boldsymbol a^\wedge)^T$

并且很容易得知，反对称符号实际上是一种一一映射，如此记法可以简化数学推理

## 坐标变换与旋转矩阵

首先提出一个问题：如果我们在相机或者雷达坐标系下观测到一个对象，那么这个对象在世界坐标系下或者机器人坐标系下的位置是如何表示的呢？这里就需要用数学公式来表述了，也就是坐标变换的问题：如何计算同一个向量在不同坐标系里的坐标？

实际上，两个坐标系之间的关系，只有旋转和平移两种，或者说，两个坐标系之间的运动就是一个旋转加上一个平移，这种运动称为**刚体运动**

刚体运动中的旋转实际上可以通过一个旋转矩阵进行定义，至于旋转矩阵的具体来龙去脉请深入学习机器人学课程，这里我们可以知道，旋转矩阵是由两组基之间的内积组成的，刻画了旋转前后同一个向量的坐标变换关系，也就是只要旋转是一样的，旋转矩阵就是一样的

旋转矩阵有一些很特殊的性质，比如说其行列式为1且为正交矩阵，反之，行列式为1的正交矩阵也是一个旋转矩阵，所以可以将 n 维旋转矩阵的集合定义如下形式

$SO(n) = \{\boldsymbol{R} \in \mathbb{R}^{n \times n} \mid \boldsymbol{R} \boldsymbol{R}^T = \boldsymbol{I}, \det(\boldsymbol{R}) = 1\}$

其中 $SO(n)$ 是特殊正交群

由于旋转矩阵为正交矩阵，所以旋转矩阵的逆阵就是其本身的转职，也就是描述了相反方向的旋转

如果考虑坐标系之间的旋转与平移，那么可以定义坐标系1和坐标系2，那么向量 $\boldsymbol a$ 在两个坐标系下的坐标是 $\boldsymbol a_1$ 和 $\boldsymbol a_2$，则存在如下的关系

$\boldsymbol{a}_1 = \boldsymbol{R}_{12} \boldsymbol{a}_2 + \boldsymbol{t}$ 

也存在行列式为\-1的旋转矩阵，但是这种矩阵表示的是瑕旋转，即一次旋转加一次反射

但是上面的变换方程并不是一个线性方程，如果进行了两次或多次变换会难以描述和推导，因此引入了齐次坐标和对应的变换矩阵

$\begin{bmatrix} \boldsymbol{a}' \\ 1 \end{bmatrix} = 
\begin{bmatrix} \boldsymbol{R} & \boldsymbol{t} \\ \boldsymbol{0}^T & 1 \end{bmatrix} 
\begin{bmatrix} \boldsymbol{a} \\ 1 \end{bmatrix} 
\triangleq \boldsymbol{T} 
\begin{bmatrix} \boldsymbol{a} \\ 1 \end{bmatrix}$

这是一个数学技巧，其中的四维向量称为齐次坐标，记为 $\tilde{\boldsymbol{a}}$，并且可以使用一个变换矩阵同时描述旋转和平移，我们将其定义为 $\boldsymbol T$，这种矩阵又称为特殊欧式群

$SE(3) = \left\{ \boldsymbol{T} = 
\begin{bmatrix}
\boldsymbol{R} & \boldsymbol{t} \\\boldsymbol{0}^T & 1
\end{bmatrix} 
\in \mathbb{R}^{4 \times 4} \mid \boldsymbol{R} \in SO(3), \boldsymbol{t} \in \mathbb{R}^3 \right\}$

并且可以定义反向的变换

$\boldsymbol{T}^{-1} = 
\begin{bmatrix}
\boldsymbol{R}^T & -\boldsymbol{R}^T \boldsymbol{t} \\\boldsymbol{0}^T & 1
\end{bmatrix}$

### 旋转矩阵的左乘/右乘与主动被动变换

实际上前面的旋转是一个“多义词”，比如说谁旋转了、以什么为准旋转，当你跟朋友出去玩，你在原地不动，但是朋友动了，那两个时刻下你在以朋友为准的坐标系下的位置就发生了变换，若你动但是朋友不动，则又是另外的情况，因此需要具体讨论

首先介绍主动变换 \(Active Transformation\) 与 被动变换 \(Passive Transformation\) 的概念

- 主动变换：顾名思义，向量主动变化，但是坐标系不变，如机械臂抓取的场景中，桌子为坐标系是不动的，机械臂需要旋转到新的位置来抓取，也就是物体在动

- 被动变换：向量不动，但是坐标系变化，如在相机观测场景中，相机在两个位置对物体进行观测，尽管物体不动，但是观测到的位置也会不同，也就是观测者在动

此外，旋转矩阵的左乘和右乘在数学和物理上有不同的意义，也就是旋转矩阵在向量的哪一侧相乘，这种矩阵乘法的顺序直接决定了旋转是相对于谁发生的，也就是你绕朋友旋转和原地旋转是截然不同的结果

- 左乘：矩阵在左，绕固定的全局坐标系旋转，一般用于地图矫正

- 右乘：矩阵在右，绕局部或者说自身坐标系旋转，一般用于 IMU 积分，因为 IMU 测量的是自身的角速度

因此可以进行列表，其中使用 $\boldsymbol R_{curr}$ 表示当前姿态，使用 $\Delta \boldsymbol R$ 表示旋转增量

|乘法顺序|**变换方式**|解读/用途|公式|
|---|---|---|---|
|左乘<br>绕固定系转动|**主动变换**||$$|
||**被动变换**||$\boldsymbol R_{new}=\Delta\boldsymbol R \cdot\boldsymbol R_{curr}$|
|右乘<br>绕自身系转动|**主动变换**||$$|
||**被动变换**||$$|

但是这种变换矩阵的方法是有缺陷的：

1. SO\(3\)的旋转矩阵有九个量，但一次旋转只有三个自由度。因此这种表达方式是冗余的。同理，变换矩阵用十六个量表达了六自由度的变换。那么，是否有更紧凑的表示呢?

2. 旋转矩阵自身带有约束：它必须是个正交矩阵,且行列式为 1。变换矩阵也是如此。当我们想要估计或优化一个旋转矩阵,变换矩阵时，这些约束会使得求解变得更困难。

## 旋转向量与欧拉角

### 旋转向量——除了旋转矩阵之外的旋转表示

旋转矩阵表示旋转是冗杂的\(旋转矩阵有9个量，但一次旋转只有3个自由度并且旋转矩阵自身带有约束\)，我们希望有一个紧凑和无约束的形式表示旋转和平移，所以有了新的表示方法——旋转向量，不过要注意一下，旋转向量与旋转矩阵只是表达方式不同，但是表达的内容是相同的
事实上很容易理解，任意一个旋转都可以使用一个旋转轴和旋转角描述，也就是绕该轴旋转了多少角度，因此可以定义一个旋转向量，其方向与旋转轴一致，长度等于旋转角度，也就是我们可以使用一个三维向量就可以表示旋转

那么对于同一个旋转，旋转矩阵形式和旋转向量形式之间有什么联系呢？实际上这种联系就是罗德里格斯公式

$\boldsymbol{R} = \cos \theta \boldsymbol{I} + (1 - \cos \theta) \boldsymbol{n} \boldsymbol{n}^T + \sin \theta \boldsymbol{n}^\wedge$

其中的 $\theta$ 是旋转角度，$\boldsymbol n$ 是旋转轴方向的单位向量

其中转轴是矩阵 R 特征值1对应的特征向量

实际上的计算推导过程可以看下列视频，大概从22分钟开始讲解

[https://www.bilibili.com/video/BV1Wa411L71b?spm_id_from=333.788.player.switch&vd_source=eea47a16439992e41b232bc5d5684e27]()

当然也可以逆向，通过旋转矩阵计算旋转向量，对于转角 ，可以对两侧取迹

$\begin{aligned}
\operatorname{tr}(\boldsymbol{R}) &= \cos\theta\operatorname{tr}(\boldsymbol{I}) + (1 - \cos\theta)\operatorname{tr}\left(\boldsymbol{n}\boldsymbol{n}^T\right) + \sin\theta\operatorname{tr}(\boldsymbol{n}^\wedge)\\
&= 3\cos\theta + (1 - \cos\theta)\\
&= 1 + 2\cos\theta
\end{aligned}$

因此可以获取旋转角的表达式

$\theta = \arccos\frac{\operatorname{tr}(\boldsymbol{R}) - 1}{2}$

关于转轴，易知旋转轴上的向量在旋转后不发生改变，说明转轴是矩阵 R 特征值1对应的特征向量

$\boldsymbol R \boldsymbol n = \boldsymbol n$

解此方程并且归一化就得到了旋转轴

### 欧拉角

无论是旋转矩阵、旋转向量，虽然它们能描述旋转，但对我们人类是非常不直观的。当我们看到一个旋转矩阵或旋转向量时很难想象出来这个旋转究竟是什么样的。当它们变换时，我们也不知道物体是向哪个方向在转动。

而欧拉角提供了一种非常直观的方式，欧拉角将旋转分解为三次不同轴上的转动，以便理解

- 例如按照Z\-Y\-X转动

- 轴顺序亦可不同，因此存在许多种定义方式不同的欧拉角

- 其中ZYX 顺序（航向\-俯仰\-滚转）是常用的一种，顺序上首先围绕 z 轴旋转，接着围绕新的 y 轴旋转，最后围绕新的 x 轴旋转。广泛应用于航空航天和机器人学中，用来描述航向（yaw）、俯仰（pitch）和滚转（roll）。

实际上，欧拉角的定义方式比较多（XYZ三轴不同的先后顺序），而且会存在奇异性问题（万向锁这种），所以一般不会直接使用，在SLAM中也很少使用欧拉角表示姿态，因为欧拉角存在不连续和奇异点问题

而万向锁就是欧拉角奇异性问题的一种体现，即旋转角在特定值的时候，会有两个旋转轴重合，这种情况下旋转自由度减一

由于万向锁问题，欧拉角不适合插值和迭代，往往用于人机交互中，并且可以证明，用三个实数来表达三维旋转时，会不可避免地碰到奇异性问题。所以SLAM程序中很少直接用欧拉角表示姿态，或对于某些二维平面运动的场景中，也可以将旋转分解为三个欧拉角，然后将其中一个（尤其是是偏航角）拿出作为定位信息使用

正常情况下

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmY4YTA0NjY4YmU2ODMxMWZiNGIwMjU3ZjY0ZDVkNWVfZmE5NGI4NzFkMjA1ZGFjMTFmOTYxMmMwNDVkOWVkNWRfSUQ6NzU3NTIyNTQ2NDU1MDE0OTMyMF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

奇异情况下

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTQwNGJkYjY4ZGNhMGIyNTI4NTc2NzEwNmQ3YjQ3ODVfMmZlOWVjNzk3NWQwYzhlNDBhZGQzMTM3YzNhZGQ5M2JfSUQ6NzU3NTIyNTQ1NTUxNTczMzE5Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

在这种时候，两轴重合

但是，如果使用四个数来表达旋转，则不会出现这种情况，这也就是四元数的用处

## 四元数

这是一种节省空间（紧凑）而且没有奇异性的表达形式，可以用来描述旋转

2D 情况下，可用单位复数表达旋转，三维情况下，四元数就是复数的扩充

四元数（Quaternion）有一个实部和三个虚部，形式如下

$\boldsymbol{q} = q_0 + q_1 i + q_2 j + q_3 k = [q_0, \boldsymbol{v}]^T$

并且三个虚部满足以下关系

$\begin{cases}
\begin{aligned}
i^2 &= j^2 = k^2 = ijk = -1 \\
ij &= k, \quad ji = -k \\
jk &= i, \quad kj = -i \\
ki &= j, \quad ik = -j
\end{aligned}
\end{cases}$

如何使用四元数描述旋转呢？实际上是通过单位四元数实现的，单位四元数表示三维空间中的任意一个旋转，但是我们可以先了解一下四元数的一些运算，定义两个四元数

$\begin{aligned}
\boldsymbol q_a&=[s_a,\boldsymbol v_a]^T=s_a+x_a i+y_a j+z_a k\\
\boldsymbol q_b&=[s_b,\boldsymbol v_b]^T=s_b+x_b i+y_b j+z_b k\\
\end{aligned}$

1. 加减法：四元数的加减法很简单，对应位置的元素直接加减即可

2. 乘法：乘法是第一个四元数的每一项与第二个四元数的每一项相乘

    1. 可以给出直接形式：$\begin{aligned}
\boldsymbol{q}_a \otimes \boldsymbol{q}_b = &\ (q_{a0}q_{b0} - q_{a1}q_{b1} - q_{a2}q_{b2} - q_{a3}q_{b3}) \\
& + (q_{a0}q_{b1} + q_{a1}q_{b0} + q_{a2}q_{b3} - q_{a3}q_{b2}) i \\
& + (q_{a0}q_{b2} - q_{a1}q_{b3} + q_{a2}q_{b0} + q_{a3}q_{b1}) j \\
& + (q_{a0}q_{b3} + q_{a1}q_{b2} - q_{a2}q_{b1} + q_{a3}q_{b0}) k
\end{aligned}$

    2. 也可以给出简洁形式：$\boldsymbol{q}_a \otimes \boldsymbol{q}_b = 
\begin{bmatrix}
s_a s_b - \boldsymbol{v}_a \cdot \boldsymbol{v}_b \\
s_a \boldsymbol{v}_b + s_b \boldsymbol{v}_a + \boldsymbol{v}_a \times \boldsymbol{v}_b
\end{bmatrix}$

    3. 乘法具有非交换性，满足结合律和分配律

3. 模长

    1. 定义模长为：$\|\boldsymbol{q}\| = \sqrt{s^2 + x^2 + y^2 + z^2}$

    2. 对于两个四元数的模长有：$\|\boldsymbol{q}_a \otimes \boldsymbol{q}_b\| = \|\boldsymbol{q}_a\| \cdot \|\boldsymbol{q}_b\|$

4. 共轭：四元数的共轭就是把虚部取成相反数

    1. 定义：$\boldsymbol{q}^* = s - x i - y j - z k = [s, -\boldsymbol{v}]^T$

    2. 相乘：$\boldsymbol{q} \otimes \boldsymbol{q}^* = \boldsymbol{q}^* \otimes \boldsymbol{q}=[s_a^2+\boldsymbol v^T\boldsymbol v,0]^T$

5. 逆

    1. 定义：$\boldsymbol{q}^{-1} = \frac{\boldsymbol{q}^*}{\|\boldsymbol{q}\|^2}$

    2. 四元数和自己的逆的乘积为实四元数 1

那么如何使用四元数描述旋转呢？首先我们有一个旋转向量，那么就可以根据这个旋转向量计算出一个四元数，实际上这与之前的罗德里格斯公式有异曲同工之处

$\boldsymbol{q}=\left[
\cos \frac \theta 2,\boldsymbol{n}
\sin \frac \theta 2
\right]$

那么这个四元数如何实现旋转计算呢？首先我们有一个三维空间中的点 $\boldsymbol p=[x,y,z]^T\in \mathbb R^3$，然后定义其旋转之后的点为 $\boldsymbol p^\prime$，那么我们先使用一个虚四元数描述该点

$\boldsymbol p=[0,x,y,z]^T=[0, \boldsymbol v]^T$

相对于把四元数中的三个虚部与空间中的三个轴对应，那么旋转之后的点就可以表示为

$\boldsymbol p^\prime= \boldsymbol q \boldsymbol p \boldsymbol q^{-1}$

注意一下，上式实际上是一个四元数乘法，使用结果也是四元数，需要最后将虚部取出，然后才可以得到旋转之后的点坐标

## 程序设计

使用动态矩阵的时候，运算会比较慢

# 李群和李代数

## 背景

在SLAM中，除了表示之外，还要对它们进行估计和优化，因为SLAM整个过程就是在不断地估计机器人的位姿与地图，该位姿是由旋转矩阵或变换矩阵描述的。为了优化位姿，需要对变换矩阵进行插值、求导、迭代等操作，比如说当我们去估计相机位姿的时候，当估计不准确的时候，要对旋转和平移进行微调。

设某个时刻机器人的位姿为 $\boldsymbol{T}_{cw}$，它观察到了一个世界坐标位于 $\boldsymbol{P}_w$ 的点，产生了一个观测数据 $\boldsymbol{Z}_c$，根据坐标变换有

$ \boldsymbol{Z}_c = \boldsymbol{T}_{cw} \boldsymbol{P}_{w} + \boldsymbol{w}$

那我们实际要做的事情是求一个欧氏变换 $\boldsymbol{T}_{cw}$，使得 $\boldsymbol{T}_{cw}$ 满足上式。

然而，由于观测噪声 $\boldsymbol{w}$ 的存在，$\boldsymbol{z}$ 往往不可能精确地满足 $\boldsymbol{z} = \boldsymbol{T}\boldsymbol{p}$ 的关系。所以，我们通常会计算理想的观测与实际数据的误差：$\boldsymbol{e} = \boldsymbol{z} - \boldsymbol{T}\boldsymbol{p}$

假设一共有 $N$ 个这样的路标点和观测，则就有 $N$ 个上式，则对于机器人的位姿估计，相当于寻找一个最优的 $\boldsymbol{T}$，使得整体误差最小化：

$\min_{\boldsymbol{T}} J(\boldsymbol{T}) = \sum_{i=1}^{N} \|\boldsymbol{z}_i - \boldsymbol{T}\boldsymbol{p}_i\|_2^2$

计算最优就需要求导，求导就需要进行加减，但是由于其性质，我们无法完成这个求导操作，自然无法完成优化，所以我们需要用一种新理论去完成这个操作

## 代数基础

之前的章节介绍了旋转矩阵和变换矩阵的定义。当时，我们说三维旋转矩阵构成了特殊正交群 $SO(3)$，而变换矩阵构成了特殊欧氏群 $SE(3)$。它们写起来像这样：

$SO(3) = \{ \boldsymbol{R} \in \mathbb{R}^{3 \times 3} \mid \boldsymbol{R}\boldsymbol{R}^T = \boldsymbol{I}, \det(\boldsymbol{R}) = 1 \}\\

SE(3) = \left\{ \boldsymbol{T} = 
\begin{bmatrix}
\boldsymbol{R} & \boldsymbol{t} \\\boldsymbol{0}^T & 1
\end{bmatrix}
\in \mathbb{R}^{4 \times 4} \mid \boldsymbol{R} \in SO(3), \boldsymbol{t} \in \mathbb{R}^3 \right\}$

不过，当时我们并未详细解释群的含义。细心的读者应该会注意到，旋转矩阵也好，变换矩阵也好，它们对加法是不封闭的。换句话说，对于任意两个旋转矩阵 $\boldsymbol{R}_1$，$\boldsymbol{R}_2$，按照矩阵加法的定义，和不再是一个旋转矩阵：

$\boldsymbol{R}_1 + \boldsymbol{R}_2 \notin SO(3), \quad \boldsymbol{T}_1 + \boldsymbol{T}_2 \notin SE(3)$

你也可以说两种矩阵并没有良好定义的加法，或者通常矩阵加法对这两个集合不封闭。相对地，它们只有一种较好的运算：乘法。$SO(3)$ 和 $SE(3)$ 关于乘法是封闭的：

$\boldsymbol{R}_1 \boldsymbol{R}_2 \in SO(3), \quad \boldsymbol{T}_1 \boldsymbol{T}_2 \in SE(3)$

同时我们也可以对任何一个旋转或变换矩阵（在乘法的意义上）求逆。我们知道，乘法对应着旋转或变换的复合，两个旋转矩阵相乘表示做了两次旋转。对于这种只有一个（良好的）运算的集合，我们称之为群

那么如何理解这种概念呢？回想一下线性代数中的线性空间或者向量空间的概念，线性空间的定义就是满足若干公理的向量的集合，如加法、数乘、交换律、封闭性等，其定义了一个平整光滑的空间，不能弯曲、闭合和存在边界，就如同一张无限大的纸

那么如果砍去其中的一些性质，如砍掉数乘，但是仍然满足交换律等，那么就构成了一个阿贝尔群，也就是其中的公理只涉及向量集合内部的元素，不涉及外部的标量；如果继续砍去一些性质要求，就构成了李群，可以理解为李群是弱约束下的线性空间，线性空间是强约束下的李群

那么为什么要如此定义呢？因为线性空间必须可以数乘，因此必须平直，但是李群并没有那么多要求，只需要满足互操作即可，空间就可以是弯曲和封闭（比如说首尾相连），如旋转群 $SO(3)$ 就像一个球体表面。你在球面上走（旋转），没法定义“把这个旋转放大 2\.5 倍”而不离开球面（数乘失效），但你可以定义“先转 A 再转 B”（群乘法有效）

群（Group）是一种集合加上一种运算的代数结构。我们把集合记作 $A$，运算记作 $\cdot$，那么群可以记作 $G = (A, \cdot)$。群要求这个运算满足以下几个条件或者说公理：

1. 封闭性：$\forall a_1, a_2 \in A, \quad a_1 \cdot a_2 \in A$

2. 结合律：$\forall a_1, a_2, a_3 \in A, \quad (a_1 \cdot a_2) \cdot a_3 = a_1 \cdot (a_2 \cdot a_3)$

3. 幺元（也是单位元）：$\exists a_0 \in A, \quad \text{s.t.} \quad \forall a \in A, \quad a_0 \cdot a = a \cdot a_0 = a$

4. 逆：$\forall a \in A, \quad \exists a^{-1} \in A, \quad \text{s.t.} \quad a \cdot a^{-1} = a_0$

群结构保证了在群上的运算具有良好的性质

对于旋转矩阵和变换矩阵群，上面的性质都很容易证明与理解：

1. 旋转矩阵与旋转矩阵的乘积仍然是旋转矩阵

2. 旋转矩阵的连续乘法满足结合律

3. 单位矩阵即为幺元，也就是旋转角度为零

4. 旋转矩阵存在逆阵，表示反向旋转

上述性质对于变换矩阵同样适用

## 李群概念

### 几何理解

李群是具有连续性质的群，或者说这个群是光滑可微的（可以想象成没有尖刺和棱角的封闭几何体的表面），所以既是群也是流形（Manifold），直观上看，一个刚体能够连续地在空间中运动，也就有连续的位姿，相应的旋转矩阵和变换矩阵也是连续的，故 $SO(3)$ 和 $SE(3)$ 都是李群

所有李群都是流形，但并非所有流形都是李群。李理论的基本现象是，人们可以以一种自然的方式将李群 $\mathcal G$ 与李代数 $\mathfrak g$ 联系起来。李代数 $\mathfrak g$ 首先是一个向量空间，其次被赋予了一个双线性非结合乘积，称为李方括号 $[\cdot,\cdot]$。令人惊讶的是，群 $\mathcal G$ 几乎完全由李代数 $\mathfrak g$ 和它的李括号决定。因此，处于许多目的，我们可以用李代数 $\mathfrak g$ 代替李群 $\mathcal G$  。由于李群 $\mathcal G$ 是一个复杂的非线性对象，而 $\mathfrak g$ 只是一个向量空间，所以使用 $\mathfrak g$ 和李括号通常要简单得多，这是李理论力量的来源之一

具体来说，流形可以被定义为一个空间（可以想成一个曲面），因为流形是光滑的，所以它在每个点处都有且只有一个“切空间（切线或者切平面）”，切空间是一个局部欧几里得空间或者说线性向量空间，其维度等于流形的自由度，它可以用欧几里得几何的方法来描述，然后我们可以使用切空间的一些性质来近似表示局部曲面的性质（类似于函数可以使用若干阶导数的多项式近似表示，甚至二者之间可以形成一个双射关系\)，这种性质可以用来解决位姿求导和状态估计的问题

1. **概率分布的定义**：高斯分布（Gaussian Distribution）定义在向量空间上。我们无法在球面上直接定义标准高斯分布，但可以在切平面上定义，这代表了围绕某一名义状态的不确定性。这一点可用于预积分、里程计等

2. **微积分的运算**：导数和积分本质上是线性的极限操作，它们在弯曲空间难以直接定义，但在切空间中却轻而易举。

下图展示了李群和李代数之间的关系，李群流形 $\mathcal{M}$ 是三维空间中的蓝色球面，李代数 $T_{\mathcal{X}}\mathcal{M}$ 是红色平面所表示的切空间，切点位于 $\mathcal{E}$，通过指数映射，经过李代数切空间原点的每条直线 $\boldsymbol vt$ 产生了一条围绕流形的路径 $\exp(\boldsymbol vt)$ ，它沿着各自的测地线（geodesic）进行移动。相反地，群中的每个元素在李代数中都有一个等价的元素。这个关系是如此深刻，以至于（几乎）群中的所有操作，它是弯曲的和非线性的，在李代数中有一个精确的等价性，它是一个线性的向量空间。虽然三维空间中的球体不是一个李群（我们只是用它作为一个可以在纸上绘制的表示），但四维欧式空间中的球体是一个李群，一个单位四元数的群

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Y2JjM2RjNzU5NTJkMTgxNWI2NDZiNTk2NzAxODBjYTBfM2RhM2I4YTg2Y2IyMTc4NWIyZjc0ZWYwNGM4NzJjNDdfSUQ6NzU3OTQ0NTIwMzQzMDQ2MDYyMV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

三维球面为二维的流形，因为可由一群二维的平面图形来叠加（广义加法）表示

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODNiYjNhOTRmYjE1MDJlNWFmOTAzM2VkZGRmYzcyNzlfMWVhYjA2MmZlZmE5NDUzNzI0NTlkMzAzZGMwZjRkYzJfSUQ6NzU3OTQ0NTI1NDM0MjIwMDUxOF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YjE5MzJhYjM0MWFmMTMwMGM2YzljZjA1NzdmOWY5YmJfMzZkYzRlNTM4YTA3OTZjY2IyZDRiODYzMThjMWVjODBfSUQ6NzU3OTQ0NTIzOTkxNzcyNjY2OV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)



如图所示的地球球面就是一个2维流形。因此，对于球面上的一个曲面三角形，可以摊开展成（即流动变形成）一个2维欧几里得空间上的平面三角形。此外，因为地球实在太大，我们往往把地球上的一块足够小的（曲面）局部区域当作平面来丈量，而不用担心会引起大的误差。比如，你要丈量学校操场的面积，根本不用把它认为是地球上的一块曲面，而直接看作一块平面即可。所以，光滑流形其足够小的结构是“硬”的（如可以固定丈量），而整体结构则是“柔软”的（可流动变形）。也就是我们可以使用一个平面来表示局部的曲面，并且带来计算上的方便，因为很多计算在曲面上是难以实现或者完全无法实现的。

也就是这种方法可以在局部欧式空间中的使用常规方法，因此不需要考虑复杂的全局拓扑结构，也能够精确估计出高维空间中物体的运动状态。

流形（Manifold）可看作是很多曲面片的叠加（这些曲面在大的尺度上即为平面，叠加即为广义加法）。这些平面可以看成位姿增量，将平面指数映射成曲面，而曲面的叠加即构成李群，也就是地球的球面一个三维空间中的二维的流形。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NmE3MzgzZTY5MjAxMWI3ZGI1ZGQxNjM5OWIxMzc1ZDhfOTBkNTMxNzQyMTRlZmEyNzlhZWU3MDI1MDdkMjc5NTNfSUQ6NzU3OTQ0NTYyMjI5OTAwMzg1OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

我们所能观察到的数据（**r**）实际上是由一个低维流形映射到高维空间上的，即这些数据所在的空间是“嵌入在高维空间的低维流形。这个 **r** 是迭代卡尔曼每次迭代出的位姿增量，即李代数，也是欧几里得空间中的平面，只有李代数才满足广义加法。**从整体观察：流形即为李群，从局部观察：流形近似为欧式空间。**

### 直观样例

#### 单位复数群

第一个李群的例子是复乘法下的单位复数群，这是最容易可视化的。单位复数的形式为：$\boldsymbol z=\cos \theta +i\sin\theta$

1. **Action 动作**：向量 $\boldsymbol x$ 在平面中旋转角度 $\theta$，通过复数乘法，$\boldsymbol x '=\boldsymbol z \boldsymbol x$

2. **Group facts 群的事实**：单位复数的乘积是一个单位复数，幺元为1，且逆为共轭 $\boldsymbol z^*$ 。

3. **Manifold facts 流形的事实**：单位范数约束定义了在复平面内的单位圆（它可以看作是1维球 1\-sphere，命名为 $\boldsymbol S^1$，如下图中的蓝色圆形所示），这是一条**在2维空间中自由度为1的曲线，也是一个流形**。单位复数在这个圆上随时间演化。群（圆）局部调整线性空间（切线），而不是全局。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTVlNDc2NDljYzcyZmY0MDc3OGVmZDI2MzcxNzlhODNfZTkwYTM0YWViZWY5NDEzYzFhZTI0ZjA2Y2NiODE0MDlfSUQ6NzU3OTUyNDI2MTQ5MDkzNzA0Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

流形 $\boldsymbol S^1$ 是复平面 $\mathbb C$ 中的单位圆（蓝色），其中单位复数始终满足 $\mathbf{z^*z}=1$。李代数 $\mathfrak s^1=T_{\mathcal E} S^1 $ 是虚部 $i\mathbb R$（红色）的线条，且任意切空间 $TS^1$ 是与（红色）线 $\mathbb R$ **同构的（isomorphic）**。切向量（深红色片段）缠绕贴合到流形上得到圆弧（蓝色弧线）。两种映射 exp 和 log（黑色箭头）将虚部 $i\mathbb R$ 的元素 **缠绕wrap** 或 **掰直unwrap**为流形 $\boldsymbol S^1$ 中的元素（蓝色弧线）。**单位复数之间的增量（increment）通过合成和指数映射在切线空间中表示**（为此，我们将定义特殊的运算符 $\oplus$ ）

#### 单位四元数群

李群的第二个例子是在四元数乘法组合运算背景下**单位四元数群**，它也是相当容易可视化理解的。单位四元数的形式为：$\mathbf q=\cos(\theta/2)+\mathbf u\sin(\theta/2) $ ，其中 $\mathbf u=iu_x+ju_y+ku_z$ 是一个单位旋转轴，$\theta$ 是旋转角度。

- **Action 动作**：向量  x=ix\+jy\+kz\\mathbf x=ix\+jy\+kz  在三维空间中通过两次四元数乘法  x′=qxq∗\\mathbf \{x'=qxq^\*\}  绕单位轴  u\\mathbf u  旋转 θ\\theta 角。

- **Group facts 群的事实**：单位四元数的乘积是仍是一个单位四元数，幺元为1，逆位共轭四元数q∗\\mathbf q^\*。

- **Manifold facts 流形的事实**：单位范数约束定义了一个三维球体  S3S^3  ，**四维空间中的一个球形三维曲面或者三维流形**。单位四元数在这个曲面上随着时间变化。群（球体）局部重构了线性空间（切超平面  R3⊂R4\\mathbb R^3\\subset\\mathbb R^4  ），但不是全局的。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmJkYjQzOWU2NjAzZjcwNmI4YTU4ZjUwZGNmMGJiOGZfNTc1MjFiOTM0MmNiMTQ1M2NhM2QzM2E4MDMxZTVkZWNfSUQ6NzU3OTUyOTU1NjU2Nzc1NTczNl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

如下图4所示。  S3S^3  流形是在四元数的四维空间中的一个**单位三维球体（unit 3\-sphere）**（蓝色），其中始终保持着 q∗q=1\\mathbf\{q^\* q\}=1  。**李代数是纯虚四元数  **ix\+jy\+kz∈H**ix\+jy\+kz\\in\\mathbb H  所在的空间**，同构于超平面  R3\\mathbb R^3  （红色网格），任何其它切线空间 TS3TS^3 也与  R3\\mathbb R^3  同构。**切向量 （深红色线段）贴着优弧（great arc）或者测地线（geodesic）（蓝色虚线）缠绕（wrap）到流形上**。中间和右边两图显示了经过这条测地线的侧视图（注意看它如何类似于图3中的流形  S1S^1 ）。带箭头的黑线表示的两种映射运算  exp\\exp  和 log\\log 将  Hp\\mathbb H\_p  中的元素映射 到/自  S3S^3  中的元素（深蓝色弧线）。四元数之间的增量通过运算  ⊕,⊖\\oplus,\\ominus  在切空间中进行表示。

## 李代数概念

李代数是与李群对应的一种结构，位于向量空间，对应李群的正切空间，描述了李群局部的导数，记作 

$\mathfrak{so}(3)$ 和 $\mathfrak{se}(3)$

从旋转矩阵可以引出李代数，我们考虑任意旋转矩阵 $\boldsymbol{R}$，满足

$\boldsymbol{R}\boldsymbol{R}^T = \boldsymbol{I}$

在连续运动过程中，显然 $\boldsymbol{R}$ 是连续时间的函数，我们记为

$\boldsymbol{R}(t)\boldsymbol{R}(t)^T = \boldsymbol{I}$

两侧对时间求导

$\dot{\boldsymbol{R}}(t)\boldsymbol{R}(t)^T + \boldsymbol{R}(t)\dot{\boldsymbol{R}}(t)^T = 0 \\\dot{\boldsymbol{R}}(t)\boldsymbol{R}(t)^T = -(\dot{\boldsymbol{R}}(t)\boldsymbol{R}(t)^T)^T$

如果我们将 $\dot{\boldsymbol{R}}(t)\boldsymbol{R}(t)^T$ 看做一个整体，我们就发现其为一个反对称矩阵，三维的反对称矩阵与三维向量是一一对应的，因此可以引入反对称符号来表示

$\dot{\boldsymbol{R}}(t)\boldsymbol{R}(t)^T = \boldsymbol{\phi}(t)^\wedge$

两边右乘 $\boldsymbol{R}(t)$ 可得

$\dot{\boldsymbol{R}}(t)\boldsymbol{R}(t)^T \boldsymbol{R}(t) = \boldsymbol{\phi}(t)^\wedge \boldsymbol{R}(t)$

其中 $\boldsymbol{R}(t)^T \boldsymbol{R}(t) = \boldsymbol{I}$，消去后得到

$\dot{\boldsymbol{R}}(t) = \boldsymbol{\phi}(t)^\wedge \boldsymbol{R}(t)$

可以看成求导之后，左侧多出一个 $\boldsymbol{\phi}(t)^\wedge$，或者说，每对旋转矩阵求导一次，只需要左乘一个此矩阵（当然此矩阵不是一个常数），这类似乎指数函数的操作，变量的导数等于其本身乘以一个系数

$y = e^{kx} \rightarrow y' = ke^{kx} \rightarrow y' = ky$

从简单情况考虑，当 $t_0 = 0$, $\boldsymbol{R}(0) = \boldsymbol{I}$ 的时候

$\begin{aligned}
\boldsymbol{R}(t) &\approx \boldsymbol{R}(t_0) + \dot{\boldsymbol{R}}(t_0)(t - t_0) \\
&= \boldsymbol{I} + \boldsymbol{\phi}(t_0)^\wedge(t)
\end{aligned}$

在这里，$\phi^{\wedge}$ 为 $R(t)$ 的李代数，是李群在单位元 $t_0$ 处的正切空间

在 $t_0$ 附近，设 $\phi$ 保持为常数向量 $\phi(t_0) = \phi_0$，则有微分方程

$\dot{R}(t) = \phi(t_0)^{\wedge} R(t) = \phi_0^{\wedge} R(t)$

已知初始情况，解得

$R(t) = \exp(\phi_0^{\wedge} t)$

$R(t)$ 与 $\phi$ 之间的关系称为指数映射，这里的 $\phi$ 称为 $SO(3)$ 对应的李代数：$\mathfrak{so}(3)$

但是新的问题来了，$\mathfrak{so}(3)$ 的定义和性质是什么呢？这个指数映射应该怎么求呢

实际上每个李群都有与之对应的李代数，李代数描述了李群单位元数的正切空间性质。

李代数由一个集合 $\mathbb V$，一个数域 $\mathbb{F}$ 和一个二元运算 $[,]$ 组成。如果它们满足以下几条性质，称 $(\mathbb V,\mathbb{F},[,])$ 为一个李代数，记作 $\mathfrak{g}$

1. 封闭性：$\forall X,Y \in \mathbb V,[X,Y] \in \mathbb V$

2. 双线性：$\forall X,Y,Z \in \mathbb V,a,b \in \mathbb{F}$，有$[aX + bY,Z] = a[X,Z] + b[Y,Z], [Z,aX + bY] = a[Z,X] + b[Z,Y]$

3. 自反性：$\forall X \in \mathbb V,[X,X] = 0$

4. 雅可比等价：$\forall X,Y,Z \in \mathbb V,[X,[Y,Z]] + [Z,[Y,X]] + [Y,[Z,X]] = 0 $

二元运算被称为李括号，例子：三维空间向量加叉积运算构成李代数，当然，实际上我们不需要去记忆这些性质

对于李群 $SO(3)$，有李代数 $\mathfrak{so}(3)$，实际上该李代数就是定义在三维空间上的向量或三维反对称矩阵，只不过向量形式更加自然，且可以用于表达旋转矩阵的导数

$\mathfrak{so}(3) = \{\phi \in \mathbb{R}^3, \Phi = \phi^{\wedge} \in \mathbb{R}^{3 \times 3}\}\\[0.5em]
\Phi = \phi^{\wedge} = 
\begin{bmatrix}
0 & -\phi_3 & \phi_2 \\\phi_3 & 0 & -\phi_1 \\
-\phi_2 & \phi_1 & 0
\end{bmatrix} \in \mathbb{R}^{3 \times 3}$ 

在此定义下，两个向量的李括号为

$[\phi_1, \phi_2] = (\Phi_1 \Phi_2 - \Phi_2 \Phi_1)^{\vee}$

从物理角度理解，李代数就是旋转向量，李括号是两个角速度向量的叉积，它度量了两个无穷小旋转在交换顺序时产生的净旋转误差 / 额外角速度 / 耦合效应，而具体的推导会在后面展开。

对于 $SE(3)$，它也有对应的李代数 $\mathfrak{se}(3)$。为节省篇幅，这里就不介绍如何引出 $\mathfrak{se}(3)$ 了。与 $\mathfrak{so}(3)$ 相似，$\mathfrak{se}(3)$ 位于 $\mathbb{R}^6$ 空间中：

$\mathfrak{se}(3) = \left\{ 
\boldsymbol\xi = 
\left[ 
\begin{array}{c}
\boldsymbol\rho \\
\boldsymbol\phi\end{array}
\right] 
\in \mathbb{R}^6, \quad \boldsymbol\rho \in \mathbb{R}^3, \boldsymbol\phi \in \mathfrak{so}(3), \quad \boldsymbol\xi^{\wedge} = 
\left[ 
\begin{array}{cc}
\boldsymbol\phi^{\wedge} & \boldsymbol\rho \\
0^T & 0
\end{array}
\right] 
\in \mathbb{R}^{4 \times 4}
\right\}$



我们把每个 $\mathfrak{se}(3)$ 元素记作 $\boldsymbol\xi$，它是一个六维向量。前三维为平移（但含义与变换矩阵中的平移不同，分析见后），记作 $\boldsymbol\rho$；后三维为旋转，记作 $\boldsymbol\phi$，实质上是 $\mathfrak{so}(3)$ 元素。同时，我们拓展了符号的含义。在 $\mathfrak{se}(3)$ 中，同样使用 $\wedge$ 符号，将一个六维向量转换成四维矩阵，但这里不再表示反对称：

$\boldsymbol\xi^{\wedge} = 
\left[ 
\begin{array}{cc}
\boldsymbol\phi^{\wedge} & \boldsymbol\rho \\
0^T & 0
\end{array}
\right] 
\in \mathbb{R}^{4 \times 4}$

我们仍使用 $\wedge$ 和 $\vee$ 符号来指代"从向量到矩阵"和"从矩阵到向量"的关系，以保持和 $\mathfrak{so}(3)$ 上的一致性。它们依旧是一一对应的。读者可以简单地把 $\mathfrak{se}(3)$ 理解成"由一个平移加上一个 $\mathfrak{so}(3)$ 元素构成的向量"（尽管这里的 $\boldsymbol\rho$ 还不直接是平移）。同样，李代数 $\mathfrak{se}(3)$ 亦有类似于 $\mathfrak{so}(3)$ 的李括号：

$[\boldsymbol\xi_1, \boldsymbol\xi_2] = (\boldsymbol\xi_1^{\wedge} \boldsymbol\xi_2^{\wedge} - \boldsymbol\xi_2^{\wedge} \boldsymbol\xi_1^{\wedge})^{\vee}$

### 指数映射

指数映射反映了从李代数到李群的对应关系，并且任意矩阵的指数映射可以写成一个泰勒展开，但是只有在收敛的情况下才会有结果，其结果仍是一个矩阵

$\exp(\boldsymbol A) = \sum_{n=0}^{\infty} \frac{1}{n!} \boldsymbol A^n$

同样地，对 $\mathfrak{so}(3)$ 中任意元素 $\phi$，我们亦可按此方式定义它的指数映射

$\exp(\phi^{\wedge}) = \sum_{n=0}^{\infty} \frac{1}{n!} (\phi^{\wedge})^n$

但这个定义没法直接计算，因为我们不想计算矩阵的无穷次幂。下面我们推导一种计算指数映射的简便方法。由于 $\phi$ 是三维向量，我们可以定义它的模长和它的方向，分别记作 $\theta$ 和 $\boldsymbol a$，于是有 $\phi = \theta \boldsymbol a$。这里 $\boldsymbol a$ 是一个长度为1的方向向量，即 $\|\boldsymbol a\| = 1$。首先，对于$\boldsymbol a^{\wedge}$，有以下两条性质：

$\boldsymbol a^{\wedge} \boldsymbol a^{\wedge} = 
\begin{bmatrix}
-a_2^2 - a_3^2 & a_1 a_2 & a_1 a_3 \\
a_1 a_2 & -a_1^2 - a_3^2 & a_2 a_3 \\
a_1 a_3 & a_2 a_3 & -a_1^2 - a_2^2
\end{bmatrix} = \boldsymbol a \boldsymbol a^{\top} - \boldsymbol I \\[0.5em]
\boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} = \boldsymbol{a}^{\wedge} (\boldsymbol{a} \boldsymbol{a}^{\top} - \boldsymbol{I}) = -\boldsymbol{a}^{\wedge}$

这两个式子提供了处理 $\boldsymbol{a}^{\wedge}$ 高阶项的方法。我们可以把指数映射写成：

$\begin{aligned}\exp (\boldsymbol{\phi}^{\wedge}) &= \exp (\theta \boldsymbol{a}^{\wedge}) = \sum_{n=0}^{\infty} \frac{1}{n!} (\theta \boldsymbol{a}^{\wedge})^n \\&= \boldsymbol{I} + \theta \boldsymbol{a}^{\wedge} + \frac{1}{2!} \theta^2 \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} + \frac{1}{3!} \theta^3 \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} + \frac{1}{4!} \theta^4 (\boldsymbol{a}^{\wedge})^4 + \cdots 
\\&= \boldsymbol{a} \boldsymbol{a}^{\top} - \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} + \theta \boldsymbol{a}^{\wedge} + \frac{1}{2!} \theta^2 \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} - \frac{1}{3!} \theta^3 \boldsymbol{a}^{\wedge} - \frac{1}{4!} \theta^4 (\boldsymbol{a}^{\wedge})^2 + \cdots \\&= \boldsymbol{a} \boldsymbol{a}^{\top} + \underbrace{\left( \theta - \frac{1}{3!} \theta^3 + \frac{1}{5!} \theta^5 - \cdots \right)}_{\sin\theta} \boldsymbol{a}^{\wedge} - \underbrace{\left( 1 - \frac{1}{2!} \theta^2 + \frac{1}{4!} \theta^4 - \cdots \right)}_{\cos\theta}  \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge}  \\&= \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} + \boldsymbol{I} + \sin \theta \boldsymbol{a}^{\wedge} - \cos \theta \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} \\&= (1 - \cos \theta) \boldsymbol{a}^{\wedge} \boldsymbol{a}^{\wedge} + \boldsymbol{I} + \sin \theta \boldsymbol{a}^{\wedge} \\&= \cos \theta \boldsymbol{I} + (1 - \cos \theta) \boldsymbol{a} \boldsymbol{a}^{\top} + \sin \theta \boldsymbol{a}^{\wedge}.
\end{aligned}$

实际上这是一个似曾相识的结果——罗德里格斯公式

1. $\mathfrak{so}(3)$ 的物理意义就是旋转向量，即 $\mathfrak{so}(3)$ 的李代数空间就是由旋转向量组成的线性空间。

2. 如果李群\(旋转矩阵，$\boldsymbol R(t)$，类似一个函数\)代表一个球面，那么球上所有点的切线\(单位元处李群的切空间李代数，旋转向量\)，也会组成一个球面，而且这个球面和原来的球面一样。

我们可以使用下图来可视化的理解李群和李代数的关系

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzgzMGQ1Y2RiODNhMDhlNjU1OTgwZDIwNzI0YjdmM2JfOTIwNzc1YTA1NWEwNzhkNTlhNzM0NzIyMmUzMzUxZDlfSUQ6NzU3OTQ2MjUyNDg2NTY5NDY4NV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

其中的指数映射就是李代数向量到旋转矩阵的映射，通过罗德里格斯公式完成旋转矩阵计算，对数映射就是从旋转矩阵到李代数的映射，通过逆向求解罗德里格斯公式，也就是通过求迹和解特征方程的方法解出，而不必专门计算泰勒展开，其中定义对数映射如下

$\boldsymbol{\phi} = \ln\left(\boldsymbol{R}\right)^\vee = \left(\sum_{n=0}^\infty \frac{(-1)^n}{n+1} (\boldsymbol{R}-\boldsymbol{I})^{n+1}\right)^\vee$

现在，我们介绍了指数映射的计算方法，那么指数映射性质如何呢？是否对于任意的 $\boldsymbol{R}$ 都能找到一个唯一的 $\boldsymbol{\phi}$？很遗憾，指数映射只是一个满射，并不是单射。这意味着每个 $SO(3)$ 中的元素，都可以找到一个 $\mathfrak{so}(3)$ 元素与之对应；但是可能存在多个 $\mathfrak{so}(3)$ 中的元素，对应到同一个 $SO(3)$。至少对于旋转角 $\theta$，我们知道多转 $360^\circ$ 和没有转是一样的——它具有周期性。但是，如果我们把旋转角度固定在 $\pm\pi$ 之间，那么李群和李代数元素是一一对应的，矩阵的导数可以由旋转向量指定，指导着如何在旋转矩阵中进行微积分运算。

## 李群与李代数

需要注意的是，我们经常会构建与位姿有关的函数然后讨论该函数对于位姿的导数，从而调整当前的估计值，但是基于旋转矩阵的方法是无法计算导数的，所以使用李群和李代数的方法进行位姿导数的计算

使用李代数解决求导问题的思路分为两种

1. 用李代数表示姿态，然后根据李代数加法来对李代数求导

2. 对李群左乘或右乘微小扰动，然后对该扰动求导

## 李代数上的求导与扰动模型

使用李代数的一大动机是进行优化，而在优化过程中导数是非常必要的信息。下面来考虑一个问题。虽然我们已经清楚了 $SO(3)$ 和 $SE(3)$ 上的李群与李代数关系，但是，当在 $SO(3)$ 中完成两个矩阵乘法时，李代数中 $\mathfrak{so}(3)$ 上发生了什么改变呢？反过来说，当 $\mathfrak{so}(3)$ 上做两个李代数的加法时，$SO(3)$ 上是否对应着两个矩阵的乘积？如果成立，相当于：

$\exp(\boldsymbol{\phi}_1^{\wedge})\exp(\boldsymbol{\phi}_2^{\wedge})=\exp((\boldsymbol{\phi}_1+\boldsymbol{\phi}_2)^{\wedge})$

如果 $\boldsymbol{\phi}_1,\boldsymbol{\phi}_2$ 为标量，那显然该式成立；但此处我们计算的是矩阵的指数函数，而非标量的指数。换言之，我们在研究下式是否成立：

$\ln(\exp(\boldsymbol{A})\exp(\boldsymbol{B}))=\boldsymbol{A}+\boldsymbol{B}$

很遗憾，该式在矩阵时并不成立。两个李代数指数映射乘积的完整形式，由 Baker\-Campbell\-Hausdorff 公式（BCH 公式）给出。由于其完整形式较复杂，我们只给出其展开式的前几项：

$\ln(\exp(\boldsymbol{A})\exp(\boldsymbol{B}))=\boldsymbol{A}+\boldsymbol{B}+\frac{1}{2}[\boldsymbol{A},\boldsymbol{B}]+\frac{1}{12}[\boldsymbol{A},[\boldsymbol{A},\boldsymbol{B}]]-\frac{1}{12}[\boldsymbol{B},[\boldsymbol{A},\boldsymbol{B}]]+\cdots$

其中 $[\ ,\ ]$ 为李括号。BCH 公式告诉我们，当处理两个矩阵指数之积时，它们会产生一些由李括号组成的余项。特别地，考虑 $SO(3)$ 上的李代数 $\ln(\exp(\boldsymbol{\phi}_1^{\wedge})\exp(\boldsymbol{\phi}_2^{\wedge}))^{\vee}$，当 $\boldsymbol{\phi}_1$ 或 $\boldsymbol{\phi}_2$ 为小量时，小量二次以上的项都可以被忽略掉。此时，BCH 拥有线性近似表达：

$\ln(\exp(\boldsymbol{\phi}_1^{\wedge})\exp(\boldsymbol{\phi}_2^{\wedge}))^{\vee}\approx\begin{cases} 
\boldsymbol{J}_l(\boldsymbol{\phi}_2)^{-1}\boldsymbol{\phi}_1+\boldsymbol{\phi}_2 & \text{当 }\boldsymbol{\phi}_1\text{ 为小量}, \\\boldsymbol{J}_r(\boldsymbol{\phi}_1)^{-1}\boldsymbol{\phi}_2+\boldsymbol{\phi}_1 & \text{当 }\boldsymbol{\phi}_2\text{ 为小量}. 
\end{cases}$

1. 对李群左乘或者右乘微小扰动，然后对这个扰动求导，即把增量的扰动直接添加在李群上，然后利用李代数表示此扰动。

2. 把增量直接定义在李群上需要注意：传统上我们通常用加法表示增量，而李群对加法不封闭。所以这里的增量不再用加法表示，而是乘法。

3. 乘法：增量指的是，在原来的基础上改变一点点。当对旋转矩阵做乘法，乘以的是一个趋近于单位矩阵，也就是差不多没旋转，那这样就是对其“加了一个小量

4. 单位矩阵的李代数为0。

5. 与导数模型相比，省去了一个雅可比的计算，更为实用。

## 应用

因为李代数是线性的，而李群则是非线性的，而二者之间还有一一对应的关系，所以我们可以使用李代数来进行插值等操作，比如说两个时刻我们获取了两个位姿或者说变换矩阵，那么如何求得两个时刻之间任意时刻的位姿呢？这里就可以基于前一时刻，以前一时刻的位置为原点，计算后一时刻相对的位姿变换矩阵，然后将此矩阵变为李代数，在得到线性的李代数之后，就可以计算出中间任意时刻的对应的李代数，进而求得中间任意时刻的李群也即位姿

## Sophus库

Sophus库是一个专门处理李群和李代数的C\+\+库，安装的时候需要注意一下

```C++
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
cmake -S . -B build   -DCMAKE_BUILD_TYPE=Release   -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
cmake --install build
```

编译安装完成之后，在CMakeLists中，可以使用如下命令导入Sophus库

```Plaintext
find_package(Sophus REQUIRED)
```

然后就可以在CPP中使用此库了，下面是一段代码

```C++
#include <Eigen/Core>
#include"sophus/so3.h"
#include"sophus/se3.h"
//导入库，Eigen和Sophus
Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d(0.1.0)).toRotationMatrix();//沿Y轴转45度的旋转矩阵

//Sophus的构造方式
Sophus::SO3 rot_r(R);//从旋转矩阵构造
Sophus::SO3 SO3_v(0,M_PI/4,0);// 从旋转向量构造
Eigen::Quaterniond q(R);//从四元数构造
Sophus::SO3 SO3_q(q);
//当使用cout输出的时候，就会以李代数形式输出，或者可以使用.matrix()方法输出矩阵
```

# 相机模型

观测，也就是机器人如何观测外部世界，如果使用激光雷达观测或者使用相机观测，也就构成了激光slam或者视觉slam

## 缺失距离维度的照片

照片记录了真实世界在成像平面上的投影，这个过程丢弃了“距离”维度上的信息，就比如说下面这个照片，实际上两个人是一样大小，但是照片中仿佛棕色衣服的是巨人一样

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDE0MjgyYzM1NTAxYzI5ZGYyNGNiZjk1OWQ4NDEwMDhfZmVkZjNkZjI1ZTUwYmFjMGRmOWY0MjQwODFlMDE1NWFfSUQ6NzU3NTIyNTQ1NTY2MjQ4NDY4NF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

## 小孔成像模型

这里我们需要稍微暂停一下，定义几个常用坐标系



- 世界坐标系：代表物体在真实世界的三维坐标 $(X_w, Y_w, Z_w)$，实际上是一种全局坐标系

- 相机坐标系：以相机光学中心 $O$ 为原点的坐标系，Z轴与光轴重合 $(X_c, Y_c, Z_c)$，正方向朝外

- 图像坐标系：代表相机拍摄的图像的坐标系，原点为相机光轴与成像平面的交点 $(x, y)$

- 像素坐标系：在图像的平面上，基本单位是像素，原点一般在相片左上角 $(u, v)$

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODUxYTU2M2IzOWUxYTE1N2FhNjMyNGY2YjJmOGM4MzhfMTVjZjQ1ZWExMTJjYTZmYTEyMDU4OGFiNDVhNTMxNWJfSUQ6NzU4MTMyOTU5MzkzNzA4NzQzN18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

考虑到小孔成像本身为**倒像**，而实际我们实际拿到的相片都是正向的，因此通常采用等价形式，将小孔模型的成像平面前移

我们来用数学的方法描述一遍相机的成像过程，给定一个世界坐标点 $(X_w, Y_w, Z_w)$，得到其最终像素平面的坐标 $(u, v)$

首先是世界坐标系到相机坐标系，通过旋转和平移矩阵 $R, T$ 将点进行变换

$\begin{bmatrix}
X_c \\
Y_c \\
Z_c
\end{bmatrix} = R \begin{bmatrix}
X_w \\
Y_w \\
Z_w
\end{bmatrix} + T \Rightarrow TP_w$

也可以采用齐次坐标的形式

$\begin{bmatrix}
X_c \\
Y_c \\
Z_c \\
1
\end{bmatrix} = \begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix} \begin{bmatrix}
X_w \\
Y_w \\
Z_w \\
1
\end{bmatrix} \Rightarrow TP_w$

> 齐次坐标（homogeneous coordinates）是射影几何常用的一种表示形式，简单来说其采用增加一个维度的方式来描述当前点，如常见的2D/3D 点最后维度补1，实际使用时保证该值为1（比如除以该值）。其可以非常方便的描述射影几何的一些特殊情况，如无穷远点（最后一位补0）等，有兴趣可以参考《多视图几何》。这里我们使用该方式以方便后续的矩阵运算，如从相机坐标变换至世界坐标等。
> 
> 

我们来用数学的方法描述一遍相机的成像过程，给定一个世界坐标点 $$\(X\_w, Y\_w, Z\_w\)$$，得到其最终像素平面的坐标 $(u, v)$

之后我们采用投影公式进行投影

$\begin{cases} 
x = \frac{f}{Z_c} X_c \\ 
y = \frac{f}{Z_c} Y_c 
\end{cases}$

也可以使用矩阵的形式表示

$\begin{bmatrix} 
x \\ 
y \\ 
1 
\end{bmatrix} = 
\begin{bmatrix} 
\frac{f}{Z_c} & 0 & 0 & 0 \\ 
0 & \frac{f}{Z_c} & 0 & 0 \\ 
0 & 0 & \frac{1}{Z_c} & 0 
\end{bmatrix} 
\begin{bmatrix} 
X_c \\ 
Y_c \\ 
Z_c \\ 
1 
\end{bmatrix} \Rightarrow K'P_c$

这里我们损失了距离信息。

再就是图像坐标系到像素坐标系，图像坐标系和像素坐标系存在一个比例关系，设图像x方向每毫米有α个像素，y方向每毫米有β个像素，也就是放缩和偏移，则有：

$\begin{cases} 
u = c_x + x \cdot \alpha \\ 
v = c_y + y \cdot \beta 
\end{cases}$

矩阵形式为

$\begin{bmatrix} 
u \\ 
v \\ 
1 
\end{bmatrix} = 
\begin{bmatrix} 
\alpha & 0 & c_x \\ 
0 & \beta & c_y \\ 
0 & 0 & 1 
\end{bmatrix} 
\begin{bmatrix} 
x \\ 
y \\ 
1 
\end{bmatrix} \Rightarrow K''Pxy$

其中 $c_x, c_y$ 为成像中心在像素坐标中的位置。

将上述公式统一，有

$Puv = K''K'TP_w \Rightarrow sKTP_w$

其中

$s = \frac{1}{Z_c}; K = 
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}\\[0.5em]
f_x = \alpha f; f_y = \beta f$

如果相机的成像是

早期的相机有可能会存在像素本身是平行四边形而非矩形的问题，因此增加一个参数来描述，则有

$K = 
\begin{bmatrix}
f_x & skew & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}$

个参数用来建模像素是平行四边形而不是矩形，这个参数同样可以认为是传感器的安置不严格与相机主光轴垂直造成的变形的近似，事实与像素坐标系的X、Y轴之间的夹角的正切值成反比，因此当$$skew = 0$$表示像素为矩形。

通常我们称K为相机内参矩阵，而包含旋转和平移关系的T为外参矩阵。至此，简单的针孔相机模型就完成了。

## 鱼眼相机

小孔成像模型中，投影的过程可以理解为是一个三角相似变换的过程，也就是可以使用下图的过程描述，物体点沿着穿过光心的射线投影到

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTJiYzZlNjA1MTY3M2Q1MDY0Njk1Y2IxOTY1NGZmNDFfODFmYjQ3MjY1NzIwNGFkZGE5YmYwMWU0NDBlMTVhYmZfSUQ6NzU4MTM3NDUyMjQzNjQ4ODE1Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

# 传感器标定

我们所处的世界是三维的，而相机拍摄的照片却是二维的，丢失了其中**距离/深度**的信息。从数学上可以简单理解为，相机本身类似一个映射函数，其将输入的场景，通过某种关系映射为一张RGB图片，而我们永远无法完全准确的描述这个过程，只能是尽可能接近

而相机标定，就是使用数学模型和数学方法来近似**逼近这一复杂映射函数的过程**。标定后的相机即具有了描述这一过程的能力，从而可以用于各种计算机视觉的任务，如深度恢复、三维重建等，本质上都是对丢失的距离信息的恢复。为了更好地恢复这种信息，需要通过相机标定来求得这种映射关系的参数。这些参数包括内参数（例如焦距、光学中心位置等）和外参数（即相机相对于某个坐标系的方向和位置），以及畸变参数。 

标定的作用主要有两个方面：**校正镜头畸变和重构三维场景**。

1. **校正镜头畸变：**由于每个镜头的畸变程度各不相同，通过相机标定可以校正这种镜头畸变，生成矫正后的图像。现实中的直线，在未经校正的图片中可能会呈现弯曲的形态，而标定后可以纠正这种情况。

2. **重构三维场景：**标定的过程涉及到一系列的三维点与它们在图像上对应的二维点的数学变换，通过这个过程可以求出相机的内外参数。有了这些参数，就可以根据获得的图像来重构场景的三维模型。

需要标定的场景有如下几种：

1. 单目视觉应用，如单帧测距、车载 ADAS 辅助、单目SLAM等，这类应用，我们通常需要**相机自身的成像模型参数以及相机相对某个坐标系下的相对位姿**。再如工业上比较常见的机器人控制，需要构建机器人坐标系和其视觉坐标系之间的相对位置关系（也就是手眼标定），或者单目深度恢复等

2. 双目/多目/RGB\-D组合：这类应用更加常见，我们需要获取**相机自身信息，以及各个相机之间的相对位姿关系，有时也需要获取其和某固定坐标系之间的关系**，如车载环视相机、相机阵列（用于构建三维人体位姿等）等

目前相机标定方法主要分为三类：

|标定方法|简述|优点|缺点|
|---|---|---|---|
|自标定|使用一些几何约束，如消失点等进行标定|支持在线标定，不需要特殊的设备|精度和鲁棒性较差|
|主动视觉相机标定|使用特定设备来控制相机进行特定运动，根据已知运动和图像变化关系进行标定|不需要标定物，鲁棒性高|离线标定，成本高昂|
|传统相机标定|使用特殊标志物进行标定|标定简单，成本较低，鲁棒性高|离线标定|

而传统相机标定中张正友标定法仅使用一个规格已知的平面标定板进行标定，制作成本较低，因此是目前主流的标定方案。

## 相机成像与畸变问题

**小孔成像原理**：小孔成像利用的是光的直线传播。物体表面反射的光通过小孔投到屏上，物体不同位置上的光投到屏上的不同位置，形成倒像。

但是在真实世界中，没有真正意义上的小孔，因为孔径越小透过的光线越少，曝光时间就越长，但是如果孔不是那么小（或者称为大孔），那么就会出现如下情况：当光通过一个较大的孔径时，光线会发生弯曲和交叠，导致成像变得模糊。这种现象被称为衍射模糊，情况如下图所示。而小孔由于其尺寸较小，光线的衍射效应较小，因此可以形成相对清晰的成像。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGE3N2IzOGEzMTU1MzcwZjk1YWUyZTc3YzczNWZhMTZfMmU3NWIxMGVjZDM1MDUxZWI5NDIyNjk4ZTkxMmUwNjNfSUQ6NzU4MDA0MDM3MTkzMDA1NzY5MF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

针孔相机的针孔，就可以看做最早的镜头了，可惜这个「镜头」性能实在不能满足要求。针孔相机若要成像清晰，针孔的大小就必然不大，经由针孔进入相机的光线也是少的可怜；即使如此，在成像平面上的像也不是那么清晰。直到后来，人类发明了凸透镜，利用凸透镜成像，取代小孔，成为相机真正意义上的镜头。凸透镜做镜头有着显而易见的好处。凸透镜可以做得比小孔大的多，从而进入相机的光线也多的多，落在相机成像平面上的像自然也明亮的多了。此外凸透镜成像的清晰度可比小孔要高多了，而且不会明显受到透镜大小的影响。

由于相机本身存在一个透镜组，我们简单的针孔模型并不足以描述透镜组引入的一些光线扭曲，比如我们说的超广角镜头就会存在明显的**畸变**。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGE4ZTQwNjgxMDQyMzZlMzI2YzMwOGY5NWI1OWZkZjJfMTE1NTg1MjY5MzQ3ZWQ4YTUyM2UxZTAxYTIyNGQwOTRfSUQ6NzU4MTM4MjM3NTI0Nzg5MTQwOV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

畸变主要分为桶型畸变和枕型畸变两种常见畸变形态，如下图所示。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTJiMWQ5M2Y5Y2FiNjQxZGQ1OTU1YWFiMzIzMjU2OTFfMmMxN2UwNWQ3OGFjNjIzMGQ3ZjRlNjIxZGM3YTFkZTRfSUQ6NzU4MDA0NjExNzcxMTM1MDk5MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

**桶形畸变**（Barrel Distortion）又称桶形失真，是指光学系统引起的成像画面呈桶形膨胀状的失真现象。桶形畸变在摄影镜头成像尤其是广角镜头成像时较为常见。 

**枕形畸变**（Pincushion Distortion）又称枕形失真，它是指光学系统引起的成像画面向中间“收缩”的现象。枕形畸变在长焦镜头成像时较为常见。 

**较短的焦距可以捕捉更宽广的画面，但可能会引起桶型畸变；而较长的焦距可以捕捉较为狭窄的画面，但可能会引起枕型畸变。**

数学上，我们使用 Brown\-Conrady 模型近似描述畸变\(以下公式均在**图像坐标系**下\)：

1. 径向畸变：透镜的厚薄不一，折射率不同，使得直线在投影后变成曲线

$x_{distorted} = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)\\
y_{distorted} = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)\\
r_d = r(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$

2. 切向畸变：机械组装过程中，透镜和成像平面不可能完全平行，从而导致切向畸变。

$x_{distorted} = x + 2p_1 xy + p_2 (r^2 + 2x^2)\\
y_{distorted} = y + p_1 (r^2 + 2y^2) + 2p_2 xy$

最终考虑到所有畸变，有

$x_{distorted} = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2 (r^2 + 2x^2)\\
y_{distorted} = y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2y^2) + 2p_2 xy$

则最终的像素坐标可以表示为

$u=f_x*x_d+c_x\\
v=f_x*y_d+y_x\\$

当然，由于制造工艺的提升，目前相机畸变主要是径向畸变，具体的使用可以灵活选择，比如使用单独的 $k_1、k_2$

当然，上面的去畸变公式也可以使用更低阶的形式或者更高阶的形式，没有十分的严格，参数量越多则近似效果越强，但是也会带来更高的计算量

鱼眼相机也可以使用类似的方式描述畸变，不过其调整的是

在工业上，我们也会使用畸变率来描述畸变情况，定义畸变率为

$d = (r_d - r)/r\\
d = r_d/r - 1\\
d + 1 = r_d/r$

其在一定程度上表征了径向畸变的程度。和上述的径向畸变对比来看，我们可以简单的得到

$d = k_1r^2 + k_2r^4 + k_3r^6$

此外，畸变模型不止上面两种，事实上，畸变会存在多种模型，如最新的除法模型（最新的opencv已经采用），精度会更高

$r_d = r\frac{1 + k_1r^2 + k_2r^4 + k_3r^6}{1 + k_4r^2 + k_5r^4 + k_6r^6}$

或者另外的除法模型

$r_d = r\frac{1}{1 + k_1r^2 + k_2^4 + k_3^6}$

考虑相机传感器本身的复杂性，而且每个相机的结构都有一定的差异性（哪怕是同一批相机，在参数上都会有微小的差异），而针孔相机模型仅仅是一种真实相机的成像过程的近似，甚至于我们可以说这是一种非常粗糙的近似，因此相机标定实际上只能说近似真值而无法获得真值，那么想获取更加近似更加完美的结果，就需要使用更加准确的模型，但是更加准确的模型一般都会有更多的参数量，但是要注意的是，如果引入了过于高阶的量，就容易导致过拟合和龙格现象

最后，标定很难定量评估，除非有更真的 ”真值”，用更精确的测量设备进行精确的角度和位移测量。那么在无法获取绝对真值的情况下如何处理呢？可以通过构建一个三维空间中的直线，然后对拍摄到的图像进行去畸变操作，然后在处理后的图片的直线上进行采样，以此拟合该直线，然后对其统计方差等，以此来判断去畸变的效果，如果足够好就认为其是好内参

## 去畸变问题

去畸变本质上是对畸变模型的一次反向计算，我们需要通过已知的模型和 $r_d$ 来计算出 $r$ 的关系。当然，这个多项式本身相对来说求解比较复杂，一般会考虑使用优化的方法来计算，opencv 提供了相应的函数

如何计算出畸变模型的参数是一个非常实际的问题。要解出 Brown\-Conrady 模型中的参数，本质上是一个**非线性优化问题**。你无法像解二元一次方程组那样直接算出一个确定的解，而是需要通过“逼近”的方式来寻找最优解，这个过程实际上就是重投影误差的最小化计算，就是通过给定一些 3D 点和对应 2D 角点的坐标，使用给定的相机模型，通过最小化两者的重投影误差，来优化相机的参数，因此可以构建一个无约束优化问题

$F(x) = \frac{1}{2} \sum_{i=1}^{m} (f_i(x))^2 = \frac{1}{2}  \| f(x) \|_2^2$

我们希望能够通过最小化 $ F(x) $ 的方法，找到某个给定形式的函数 $ f_i(x) $ 的系数。

## PnP问题

PnP \(Perspective\-n\-Point\) 问题是计算机视觉中的一个问题，目的是在已知一定数量的三维空间点及其在图像上对应的二维点时，估计相机的位姿，即求解世界坐标系到相机坐标系的旋转矩阵R和平移向量。



1. 最少点数要求：如果已知特征点的三维位置，至少需要4个点对才能解算出相机的位姿。这是因为PnP问题涉及求解12个未知数（旋转矩阵R有3个自由度，平移向量t有3个自由度），而每个点对提供两个方程，因此至少需要6个方程来解这个系统，对应至少4个点对。

    

3. 求解方法：有多种算法可以解决PnP问题，包括直接线性变换法（DLT）、迭代最近点法（ICP）等。每种方法都有其特定的假设和适用场景。例如，DLT方法假设摄像机已经校准过，而EPnP方法则是当摄像机未校准或校准参数不准确时的一个好的选择。

    

4. 应用场景：PnP算法在许多领域都有广泛应用，比如自动驾驶、增强现实、机器人导航以及视觉SLAM（Simultaneous Localization and Mapping）等领域。在这些应用中，能够准确地从图像中恢复出相机的位姿对于理解环境和做出决策至关重要。

    

5. 算法挑战：PnP问题的求解可能会受到多种因素的影响，如特征点的选择、噪声、遮挡等。此外，不同的算法在速度、准确性和鲁棒性方面也有所不同，因此在实际应用中需要根据具体情况选择合适的算法。

## 张正友标定法

张正友标定方案使用平面标志物，通常是规整的棋盘格或者点阵图，将标志物打印后，使用待标定相机拍摄**不同角度**多组标定图案，当然，通常在多相机情况下为了方便区分图片中棋盘格朝向，我们一般使用**宽高**不同的棋盘格，单目情况下正方形即可

这里有几点需要注意：

1. 拍摄数量通常为15\~20张

    1. 实际上，按照标定的理论，三张图片就可以完成标定，但是为了减小标定误差，拍摄图片会稍微多一些

2. 标定图案需要保证平整，每张图片中标志物尽量占据画面1/4以上，不同标定板角度尽量存在一个相对明显的旋转和平移

3. 拍摄时应尽量保证相机参数不变

    1. 这里主要是焦距不变，焦距变化会导致部分fov变化，从而影响几乎全部的标定参数。当然，大多数场景中相机本身焦距都是固定的，而消费级，如手机镜头由于其本身畸变不大，且焦距较短，因此影响会比较小。

4. 保证拍摄图案清晰，无明显模糊，由于图点在模糊情况下不会改变其中心点位置，精度理论上会更好一些

5. 拍摄图案的总和需要覆盖整个画面

    1. 我们知道标定本身可以理解为一个拟合各个像素位置成像的过程，如果覆盖不完全，就会导致某些区域像素处于无约束状态，从而出现标定错误。我们常见的图片去畸变后边缘扭曲大多是该原因导致的。

6. 使用标定工具进行标定计算

张正友标定法的数学流程是什么样呢？根据相机模型并暂时忽略畸变，有

$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = A [R \quad T] \begin{bmatrix} X_w \\ Y_W \\ Z_W \\ 1 \end{bmatrix} = A [r_1 \quad r_2 \quad r_3 \quad t] \begin{bmatrix} X_w \\ Y_W \\ Z_W \\ 1 \end{bmatrix}$

其中 $s$ 是尺度因子，且有内参矩阵如下

$A = \begin{bmatrix} \alpha & \gamma & u_0 \\ 0 & \beta & v_0 \\ 0 & 0 & 1 \end{bmatrix}$

考虑到我们标定时使用的是平面标定板，我们将 $XOY$ 平面设置为标定板平面上，$z$ 轴垂直向外，这样对于检测的所有特征点都有

$Z_W = 0$

代入上式，有

$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = A [R \quad T] \begin{bmatrix} X_w \\ Y_W \\ 0 \\ 1 \end{bmatrix} = A [r_1 \quad r_2 \quad t] \begin{bmatrix} X_w \\ Y_W \\ 1 \end{bmatrix}$

这里 $r_i$ 代表旋转矩阵的第 $i$ 个列向量。令 $\overrightarrow{M} = [X \quad Y \quad 1]^T, \quad \overrightarrow{m} = [u \quad v \quad 1]^T$, 上式简写为

$s\widetilde{m} = H\widetilde{M}$

我们称 $H$ 为单应矩阵，把矩阵展开，有

$\begin{cases}
su &= h_{11}X + h_{12}Y + h_{13} \\
sv &= h_{21}X + h_{22}Y + h_{23} \\
s &= h_{31}X + h_{32}Y + h_{33}
\end{cases}$

从而有

$\begin{cases}
uXh_{31} + uYh_{32} + h_{33}u &= h_{11}X + h_{12}Y + h_{13} \\
vXh_{31} + vYh_{32} + h_{33}v &= h_{21}X + h_{22}Y + h_{23}
\end{cases}$

可以看到，如果对两个式子都除以 $ h_{33} $，并不会对整体的形式产生影响，因此，我们一般令 $ h_{33} = 1 $。也就是说，对于单位矩阵，其自由度并不是 9，而是 8

定义

$h' = [h_{11} \quad h_{12} \quad h_{13} \quad h_{21} \quad h_{22} \quad h_{23} \quad h_{31} \quad h_{32}]$

那么上述可以修改为矩阵形式

$\begin{bmatrix}
X & Y & 1 & 0 & 0 & 0 &-uX & -uY & -u \\
0 & 0 & 0 & X & Y & 1 & -vX & -vY & -v 
\end{bmatrix}
h' = 0$

上式是一个很经典的线性方程，我们将上式写为 $ Sh' = 0 $，那么矩阵 $ S^T S $ 的最小特征值就对应该方程的最小二乘解。至此就求解出了单应矩阵，我们希望使用单应矩阵对外参矩阵进行拆解

接下来就是求解外参矩阵，我们上面求得的单位矩阵 $ H $ 可能和真实的值存在一个尺度因子，我们增加一个尺度因子 $\lambda$，有

$\lambda [h_1 \quad h_2 \quad h_3] = A [r_1 \quad r_2 \quad t]$

由上节我们提到的旋转矩阵的性质可得两个约束条件

$r_1^T r_1 = r_2^T r_2 = 1 \\
r_1^T r_2 = 0$

调整上式，有

$\begin{cases}
\lambda h_1^T A^{-T} A^{-1} h_2 = 0 \\
\lambda h_1^T A^{-T} A^{-1} h_1 = h_2^T A^{-T} A^{-1} h_2 = 1 
\end{cases}$

我们知道，$A$ 的逆矩阵为

$A^{-1} =
\begin{bmatrix}
\frac{1}{\alpha} & -\frac{\gamma}{\alpha \beta} & \frac{\gamma v_0 - \beta u_0}{\alpha \beta} \\[0.5em]
0 & \frac{1}{\beta} & -\frac{v_0}{\beta} \\[0.5em]
0 & 0 & 1 
\end{bmatrix}$

# IMU模型

## 概述

IMU也就是惯性测量单元，是测量物体三轴姿态角（或角速率）以及加速度的装置

- 6轴：三轴加速度计\+三轴陀螺仪（角速度传感器）

- 9轴：6轴\+三轴磁力计（角度）

那么为什么用IMU呢？主要有以下几个原因

- 直接输出加速度角速度

- 频率高（100\-400HZ），相机和雷达的频率基本上只有几十赫兹

- 不受外界干扰（磁力计可能受干扰），相机容易受到光照干扰，雷达在雨雾天可能受干扰

- 价格不贵，基本上几十几百就可以买到（几千几万的也有，但是一般精度的几十几百即可）

但是受自身温度、零偏、振动等因素干扰，积分得到的平移和旋转容易漂移，并且高精度的IMU价格昂贵，所以IMU只适合计算短时间且快速的运动

## 测量原理

实际上分为三种测量，加速度计、陀螺仪和磁力计

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjdhZDJmOWUyNDI1MGIwM2Q1NWI2ODFhNjI1M2FlMzRfNmFkYTE0OTE5N2VkYThlYWFiZGYyZDg3MTZlNDQ4MjVfSUQ6NzU3NTIyNTQ1NDQwODI3MzExNl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGM1OTQ3NzBlMGY5ZjI1ZDk5M2E5M2RkZGJjM2VlYTlfY2Y4MGU1ZDBiZmRjYTA2OWNhNmJhMWNjYTQ1MDk1YjFfSUQ6NzU3NTIyNTQ1MDk1NjQ1OTIzMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

加速度计实际上就是运用了胡克定律，也就是在弹簧固定下的质量块在有加速度的情况下会有位移，根据牛顿第二定律就可以知道加速度大小，当然在IMU中是使用电容差计算位移进而计算加速度

磁力计的工作原理跟指南针类似，通过霍尔效应计算磁场强度

## 误差模型

IMU的误差可以分为确定误差（可以通过标定获取，是一个确定值）和随机误差（也就是随机噪声，无法提取获取，但是可以计算出协方差矩阵）

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Y2JiNGVjOWZlOTY4OWRkYmViZWE0ZmRmNGM2MmE3NGNfNjkzMDllZjk2ZGE3YmZlNzA5MDBiZjE4Nzg2ZTg2MjVfSUQ6NzU3NTIyNTQ1OTQ1ODE4MjM1N18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

其中，偏置是会随着时间而变化的，所以还要在 SLAM 过程中不断标定或者说估计，很多紧耦合的框架都会在过程中不断估计

## 测量模型

世界坐标系为 G，IMU坐标系为 I，通常忽略 Scale

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NWZjZDgyNzM2MmYwZjUzNzlkMzg3MTlhN2JhMzM5YzFfYThiMDkzZThmZGNlNzgzZDRjYTEwYTQ4MDIyODY3M2JfSUQ6NzU3NTIyNTQ1MTgyODc3NjE1NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

一般认为重力加速度不变，导数为零，偏置服从随机游走模型，其导数是高斯的

所以，在世界坐标系下的速度等状态量如下

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTZiYmFjOTcwYmZlMGJkNDA0OWZlYzYwY2FlNmE4ZTFfOTViMjU0NGY3MDMzNWI2YmE1YWM4MTdmNTI5MGRkZjBfSUQ6NzU3NTIyNTQ1NzkyNzM1OTY3OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## 运动模型（连续\+离散）

基于连续的数学运动模型考虑

那么在 G 也就是全局坐标系下，连续运动的导数为

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDM0YWJlODE1YmI0NGIyY2RkY2ZmMGY5MmUwOTJiYmVfOGE5NGU2NWFhZWFiZGMyMDgzN2Q3ODVlOWY5Mjg5NjhfSUQ6NzU3NTIyNTQ0OTMzMzIxNDQzMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

后面两个容易理解，速度求导为加速度，位置求导为速度

对于第一个公式，我们假设一个从原点出发的向量 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTYwYTI4MzlmOTlmMTU5ZTg5NzY2YjRmNWJhYjBkNWZfMTE4YTU4ZjczNWI1YzQyNjhkZmM5YjA1OWY2ZmUzNTFfSUQ6NzU3NTIyNTQ1MDcxMzMyMDY0NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

绕某一个方向上的单位轴 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGM5ZDZlZDQ2YzU5NTQ1NGNlNTYxMTNlOGRkMTIyZDdfMWU4OGNhZjBhMDFmOTEwMWIwNzBhNzMyMGJjNmFiMTdfSUQ6NzU3NTIyNTQ1NDQwODM4NzgwNF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

旋转，那么角速度为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGE4NzA3ZGY3ZjM2MTZmZTg0YTdlZWFhMWIyYjFjZDVfMTcyMjI4MmM4OTdiNzNkYWZlNzQ2NmNlMTVmZTBhN2FfSUQ6NzU3NTIyNTQ0OTU1OTc1NTczNl8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

，角速度大小为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Y2U4Mzk0YzFhYWI5MTc0NzhkNzYwYjE4YTU3YzQwY2JfZmEwZTk4MjNmM2EwNTA4ZjlkZTJjMTllMDZmNzhjNzdfSUQ6NzU3NTIyNTQ1OTYxNzYxNTA0OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

，那么根据刚体旋转动力学，任意点绕固定轴旋转的线速度由角速度向量和该点位置向量的叉乘确定

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NTEzYjkyNTViMjllMTc5NzczMjRiODRiMTE2ZDA5N2JfYzM0NDdkYzA4MDQyYmFiOThjYTVhZDA4YzJmZjM0YzJfSUQ6NzU3NTIyNTQ2NDM4MjI5NTI0NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

那么，坐标系沿着单位轴旋转，其三个轴的导数

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmFmNzA5YjEwZDc1NzhhM2I4NjdhNTA1M2U2NWY2NWZfNzk2YzQzZmQ0Mjg5MjA0ZjY5NDMzZjRiYWE5M2JkNzBfSUQ6NzU3NTIyNTQ1NDg4MjE5NjcwM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

但是，机器无法处理连续数值，所以只能进行离散化才可以进一步处理，也就是进行欧拉积分，当然实际上也可以使用两帧之间的平均值计算（也就是中值积分）

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NzQwNDg3MjUxODE1YTJhOGRlNjFhMDEzYzkwM2RkZTJfNmI3OGQxZjg4NGU2ZmU0OTZmYWY0YmYyNDVkMGVmY2VfSUQ6NzU3NTIyNTQ1NjMxNjc5NjExNV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

这里因为时间间隔很小，所以在实际代码中可以忽略二次项

然后把测量模型的数值带入离散模型，就可以得到如下的计算模型，然后循环执行计算即可，进而不断更新状态量

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGYwMzAyNjIxMDE4Y2ZiMjJhYTZhZWZhNDllNzRlZTlfOTc4Zjk4ODhhOTAyYzEwNzk5NDYzNDAzMzgxMWY5NGNfSUQ6NzU3NTIyNTQ1Nzg3NzE5MTg5N18xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

## 使用方法

IMU 的用途主要是作为先验估计来优化激光点，在实际中，我们希望是所有点云在同一时刻的采样，但是激光雷达的采样频率相对较慢而且存在运动畸变，而 IMU 的采样频率则非常高，所以可以使用 IMU 来去除畸变，并且把所有的点云统一到同一个时刻进行处理，文中是在 **tk** 时刻。因此，我们根据 IMU 积分估计的位姿，把 $t_{k-1}$ 每个点转到 **tk** 时刻。即同一时刻与同一位姿，发射与接受激光束。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTk0ODMxMjEwYmZiZjJjYWFmOGQ3NzI5Nzk3YmQxZGVfYTUxYjhmNjAyNTk1ZWFiODI4NjE0MDNmOTI1MGM3ZDdfSUQ6NzU3NTIyNTQ1MTI4Nzc3NjQ1OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 从尾部开始遍历找 head 对应的imu数据

2. head对应的IMU状态之前已经计算出来了，计算当前点到head的时间间隔

3. 假设tail对应的IMU数据为 当前点对应的IMU数据，因为IMU频率比较高，短时间内变化不大

4. 计算head到当前点的位姿变换：∆T，并广义加到head对应的位姿，得到当前点 it 对应的位姿：P\_i、R\_i

5. 计算当前点到 end\_lidar的位姿变换：T\_ei、R\_ei

在激光SLAM中，大多数的算法都是以启动系统的时候第一帧 IMU 为参考，但是在安装 IMU 的时候可能存在安装偏差，导致 IMU 坐标系相对地面系有一个微小旋转

## ROS1代码

在 ROS1 中，使用 `sensor_msgs/Imu` 来作为消息类型传递 IMU 数据的，其中包括了四部分

1. 头消息：`std_msgs/Header header`，其中包含时间戳 `stamp`，时间戳可以使用 toSec\(\) 方法转换以便求时间差

2. 四元数及协方差：`geometry_msgs/Quaternion orientation`，表示朝向，并且有协方差数组`float64[9] orientation_covariance`

3. 角速度及协方差：`geometry_msgs/Vector3 angular_velocity` 和 `float64[9] angular_velocity_covariance`

4. 加速度及协方差：`geometry_msgs/Vector3 Linear_velocity` 和 `float64[9] Linear_velocity_covariance`

# 点云处理

## 激光雷达模型

目前的激光雷达基本上是光学雷达，通过发射脉冲激光，基于飞行时间法测量距离（也有部分初级雷达使用三角测距法），并且多线雷达还会同时发射多组激光来感知环境

激光雷达主要是以下三种，其中机械式激光雷达较为常见，受限于机械式结构，寿命相对较短，难以实现车规级运用

目前相对好的是混合固态的，相对机械式寿命相对更长，其中有大疆的Livox系列

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NmQwOTc4Y2Q2NWQzYzc2OWU4MjkyZmY2YzlmMDlhZjhfNzRlNjQ5MWJlMWRlM2NmNDg3MDdjODg2N2VkYTM5ZDVfSUQ6NzU3NTIyNTQ1NDg0NDYxMTU0OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## 点云运动畸变矫正

因为雷达与机器人是刚性连接，并且是不断进行采样而非一次性完成采样的，所以会发生一个情况，点云中的点不是同一时间完成采集的，并且在这个过程中机器人是在运动的，所以同一个物体会造成多个位置不同的采样点，如下图所示，五角星表示物体采样的点，在一段时间的起止时刻，因为机器人的位置或者说坐标系发生了变化，所以测量出来的位置就会不一样，或者说产生了错位，因为在起始时刻和结束时刻的点云是在两个不同的坐标系下采样的，但是统一在结束时刻的坐标系下进行处理，也就是产生了运动畸变

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODg1ZjFhMzhhYzU0Y2M4OWE4ZTk0NjIzYmNlNjViMjJfZmUyM2ZmYWZkYjUyYTFmNmU1MTE0ZGY4ZTJiYTdkZjhfSUQ6NzU3NTIyNTQ0ODUzMjI0OTc5Nl8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

那么如何进行矫正呢，实际上就是把所有的点归到同一个坐标系下，然后把误差补偿掉

具体步骤如下图所示，根据从里程计（比如说IMU或者轮速里程计）处获得的位姿信息，计算出来结束时刻相对起始时刻的位姿变化，然后插值计算出来每一次点云扫描时刻的位姿，基于这个插值出来的位姿变化矫正点云坐标，使其统一转化到起始时刻的坐标下或者结束时刻的坐标下

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NzJiYWEwNGQ5ZTAyMzQ1ZmYwYjYxZWFhNWJhZGUzYTdfNGE3NTgyZThiNWVhNDJiZDU4ZjMzMjdjNTQzZjRiNjJfSUQ6NzU3NTIyNTQ1MDEwMDc3MjA0Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## 解决畸变工程技巧

工程上的一些技巧

- 激光雷达安装方式，通常y轴朝向载体左右两侧

- 相对位姿变换可以通过GPS或者IMU或者轮式里程计获取

## 点云下采样

因为直接从激光雷达获取的点云中含有大量的点，所以需要进行下采样减少点的数量，其中体素滤波方法较为常见，但是要注意体素大小，太大的话会造成点云细节缺失

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDVjMGU2ZTQxZjI4NjRmYjMyMTBiZWI1MmNhNTg0NmZfYWNlMzIzZDRmYmU1NmY1ZmUyNmIwYWJjYjA5MTMwMjJfSUQ6NzU3NTIyNTQ1NTE2NzYzODczM18xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

## 特征点提取

特征点提取方法中的角点和面点提取的方法来源于LOAM论文，这是激光SLAM的开篇之作，实际上这种特征点是根据曲率来判断并且提取的

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTc0MzQxZTliMGNjNTdhMDAxMTNmNzRlMTQwZDdmNzJfN2RmYmRiMzZjNTM1YzJjMzM2OWEzMWY5YmE1MjZkYTdfSUQ6NzU3NTIyNTQ0ODMzOTMyODIxM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

具体的提取方法是这样的，对于同一个scan（也就是同一个射线上的点），判断其曲率，曲率公式如上所示，选取一个近邻点集进行曲率计算，然后根据阈值判断是否为角点或者面点，上图右下角的图片中，绿色圆形就是面点，橙色三角就是角点

区分角点和面点的原因是，在计算残差的时候，两种特征点的计算方式不一样

## 残差

将去除畸变后的特征点转换到全局坐标系下

两种点的残差计算方式如下，而我们要做的事情就是通过优化位姿，使得残差接近零

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NzcyMTBiYTE4Nzg4Y2ZhNDczNDM2Y2U0MzI5OTc3MDlfY2NmMDAxNThhOTYwNDQwMGY3MzcwZDkxZDg2YTJkYWNfSUQ6NzU3NTIyNTQ2MTU5MzI3OTcwOV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

其中角点的残差计算方式如图，其中 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2JkYjZhY2EzMjJlNDVjMWI5MTg0MmI3ZGE5ODQwZmFfYzJiMTA0MzYzNjFhMTkwMWI3NzE1MGFmNWJlM2JlY2ZfSUQ6NzU3NTIyNTQ0NzIwNjg0OTc1NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

表示第 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2U2NjNjZTU5OWYzYjRkYzkwNjE3NmQ1YmE0ZTIwNGJfOTc1MTBiZGM2ZTNkOGFiMGRkYWMzZDMwY2Q3M2VmZWNfSUQ6NzU3NTIyNTQ1NTc0MjI0MTk5NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

帧的第 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmQ3MDIxZWY0NmU3N2Y1ODU5YjkzNTk2MTk0NjA5MWZfNmMzZjU4M2YzMTcwNTFiMmY1NmJiODkwOTg0NjNiZWJfSUQ6NzU3NTIyNTQ1NzI5ODE2NDkzOV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

个点的坐标（波浪线表示已经乘以待优化位姿的坐标，也就是转到世界坐标系下的坐标），然后我们计算此点到近邻直线的距离

## ROS1 代码

点云是激光雷达产生的，而不同的激光雷达因为工作原理等不同，所以给出的点云格式也不同，这需要不同的厂家给出相应的代码从而得到 `sensor_msgs::PointCloud2` 格式的 ROS点云消息，然后我们需要使用 PCL 进行下采样和滤波操作

```C++
pcl::PointCloud<rslidar_ros::Point> pl_orig;// shengc 
pcl::fromROSMsg(msg,pl_orig);
std::cout<<"lidar point size: "<<pl_orig.points.size()<<std::endl;

pcl::VoxelGrid<pcl::PointXYZI>
```

# 激光SLAM前端

激光里程计实际上就是一种点云配准的方法，通过两帧点云之间的相对位姿变换来计算里程计，而想实现两帧点云的相对位姿变换求解，就要进行点云配准，其中经典方法有ICP和NDT，两种方法都可以在一些点云库中调用现成接口实现

## ICP方法

这是一种经典方法，思想是两帧点云中距离最近的点是近邻点，下图中 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjQ1MDRkMzhlMjMxZjRhNGQ2Mjk4NjE3YTNiOGNiM2FfZmQ1NjM1ZTMxZmFjOWMzYzhlOTU1NjFjNWE1YzA0YTJfSUQ6NzU3NTIyNTQ1OTU4MTY2ODI5Nl8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzJjOGE4MWU2NjMxNjJhMGViOTU5MzM3MWU0OWMwNDFfNTMyYmEzOGMyYjlmN2QwYzc0NmI1MzZmZTVmNjFmNTBfSUQ6NzU3NTIyNTQ1NzExNzc2MDcxNl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

是两帧点云，

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDRmY2M2NDllY2M2MDdjNGJlNDRmMmZiYmY1OTFhNzBfZTk2Mzk1YjVkZmJhMDlhMWZkOGM1NGI1OWNjMzU1MjJfSUQ6NzU3NTIyNTQ1MzAwMzM5NDI1OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODkxMWViOTdjODgzNDhkNzIxMmI3NDc3YTVmZmMxNmZfYWRmNDBmYTZlYjMwODQ0OWM5MDEyNGQ1YmVkODRjNzZfSUQ6NzU3NTIyNTQ2MDIzMDA0ODk3MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

是两帧点云的求和

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjE5NjUwZTg2ZWI2MDBmOWExZWRjZDFkODg4OWE5MmRfM2Y4NjI3YjA1N2M3YTI3ZTM5MTBjNDZiN2VjN2ZhNzFfSUQ6NzU3NTIyNTQ1OTMwMzE1NjkzMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

然后先后分布求解 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTYxYTYwMjU1YjkyZjYxYWEyZTM2ZGFiNWZmNDE1YzRfOWY1YjJjYWRkZDY1OWI5MTE1MmEwYmNkZWQ2NTI5MDNfSUQ6NzU3NTIyNTQ0OTcwNjUwNzIyMF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MGFiYTk3NDZhMzFkMjNlNTlkZTUyODU1ZGZjMmVmMzlfNWQ2NGQyN2RmMzBjMjc5NzNlNjY2MDk0ZjE4YWVlNGZfSUQ6NzU3NTIyNTQ1OTUzODE2ODc4MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjI4NmJiYWJiODBlMzA2MjUyNjY1MzU2MWZlOTIwZjNfMDQzYWZhMDhmZTVkYTY0ZTZlYjgwOTVkOTdjZDIyOTZfSUQ6NzU3NTIyNTQ2MzMxNjk3NDU0MF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## NDT方法

NDT方法也就是正态分布变换方法，是另一种经典的方法，思想是，两帧点云的分布要尽可能接近，具体操作是对上一帧点云计算分布，然后下一帧点云要尽可能接近这个分布，对应的姿态就是要寻找的姿态或者说最准确的姿态

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjJhOTg2NjkzYmRjNmE0YmVlNzgzMjU4NDk0ZDQ3MjFfZGFhMmZkYmM4MTQzODE4NGJhMDZmMTMyYmU5MzBhNzFfSUQ6NzU3NTIyNTQ2MDE1MDI5MTY3M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

如上图所示，对上一帧点云 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmJiNjE1MTE4ZDU5M2VmOGIyYTBkMjAwMmYzNjMzNGVfNjg2NzdjYzk4OWFkZDg1NzIxNGUwMTJhMGZkZDI3NTRfSUQ6NzU3NTIyNTQ1MTM0NjQxNDc4MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

进行均值和协方差矩阵计算，得到了高斯形式的概率密度函数，然后使得概率乘积尽可能更大，然后使用极大似然估计方法进行求解

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWQwYTU5MjkxZDNjZjM3Mzk2ZjY2ZGI5MzdhMDdjYjNfZDA3MjVkNTgwMTI0NDUyN2I0ZGI4YWU0NmRiYTMzYjlfSUQ6NzU3NTIyNTQ1OTYxNzgyODA0MF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

作者提出的一种改进如上图左侧所示，原来的概率分布（红线）可能会出现概率为0或者无穷大的情况，所以要做一点改动，如上图左下角公式所示，可以避免异常情况，变成绿线所示

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NTQxZmFlZjZmMWE3ZDVkNWIyOTQzMDU0NWQwMDkzOGJfMDdkM2M5Zjk2MjNjNGI1NTRkNDIwMzlmZjg0ODI5MzdfSUQ6NzU3NTIyNTQ1NTY2MjU5OTM3Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

这里，NDT更不容易受到噪声干扰，因为点云中的点是存在噪声的，在ICP中噪声也会参与计算，造成干扰，导致优化陷入局部最优，此外NDT是点云的概率分布函数进行计算，所以计算上更快

## 特征点配准方法

这是一种优化的配准方法，选取点云中一些特殊的点来进行配准，在减少了点云数量的同时，基于特征点配准也会降低噪声的干扰，在LOAM中使用的特征点是角点和面点，也就是曲率满足一定条件的点

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MWE0NWNhN2E3NWRiMTE0ZDg5NWVlZmRlOTgzYzIwMDdfN2NhYWU4OTJlODI2NjFmNDFmYzFiMDYzZTljODZmYjdfSUQ6NzU3NTIyNTQ2MDAzNjk5NjMwMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

LOAM的代码是这样操作的，代码如下

```C++
for(size_t i = startIdx，regionIdx = 0; i <= endIdx;i++，regionIdx++){
    //遍历点云，求取第 i 个点的曲率，pointWeight是权重，这里是-10，要与采样点数量对应或者说为相反数，便于后面的计算
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;
    
    //_config.curvatureRegion是求曲率范围，这里是5，也就是选取半径为5，选择临近的10个点求曲率
        for(int j = 1;j <= _config.curvatureRegion; j++){
        //实际上就是计算差值，因为diffX等是负值，并且已经乘以权重了，遍历求和等于计算所有邻近点与此点的差值并且求和，相当于计算所有近邻点相对此点的偏移量之和
        diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
        diffY += _lasercloud[i + j].y + _laserCloud[i - j].y;
        diffZ += _lasercloud[i + j].z + _laserCloud[i - j].z;
    }
    _regionCurvature[regionIdx] = diffx* diffx + diffy * diffy + diffz * diffz;
    //对三维上的偏移量求平方和，以此来判断曲率是否过大或者过小
    _regionSortIndices[regionIdx]= i;
}
```

## PL\-ICP方法

除去点到点的配准方法，还有一种是点到线的配准方法，也就是PL\-ICP方法，也即Point\-Line ICP，其精度相对点到点方法更高，这也是LOAM采用的配准方法

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWQzYTgyZjM0YmU2YTM0NzJmNWZiZmRlZjcwNzdiNDdfNzhjZGFhOGNjODJlY2RkYzk4ZWE5YTkzMTI2ZThjYzNfSUQ6NzU3NTIyNTQ1NDM5NTgzNzY0M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

# VSLAM前端

## 2D\-2D

假设在相邻时刻，相机分别观测到两个图像，并且在其中匹配到了对应点，如何求两个图像对应位姿之间的位姿变换

假设对应点的齐次像素坐标为 $p_1$ 和 $p_2$，那么在两个时刻的相机坐标系下有下面的公式，其中相机内参矩阵已知
$p_1=KP_{C1}\\p_2=KP_{C2}$

那么以第一个相机坐标系为基准（或者为世界坐标系的话），那么有
$P_{C2}=RP_{C1}+t\\K^{-1}p_2=P_{C2}=RP_{C1}+t=RK^{-1}p_{1}+t\\t^\wedge K^{-1}p_2=t^\wedge R K^{-1}p_{1}+t^\wedge t\\其中，向量与自身的叉乘为零$
然后左边乘以 $(K^{-1}p_2)^T$，其中左侧为 $t^\wedge K^{-1}p_2$，也就是 t 与向量的叉乘，方向上垂直于此向量，再进行与向量的点乘结果为0，故有
$$

\(K^\{\-1\}p\_2\)^Tt^\\wedge R K^\{\-1\}p\_\{1\}=p\_2^TK^\{\-T\}t^\\wedge R K^\{\-1\}p\_\{1\}=0
$$

# VIO

## 融合方案介绍

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTVhZTNiOWI0NTNkOTVkNmRkZjNkNTAwNWM1MTZhZTJfOWZhNGI0ZDcxOWRlY2RkNjEwNDEwNmRmMThkMGQ1OThfSUQ6NzU3NTIyNTQ1NTk2ODYxOTQ2N18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDdhNDc0MjQ3MWZlNjBmMTAzZDhjYzZiYzFiZWU4YzlfZWFiNWY5MWFmYTlkNzA0NjFjZGNkMWFkMDQyMTQ4NTFfSUQ6NzU3NTIyNTQ2MTA4MTQ1OTkzMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

# 激光SLAM框架

## LOAM框架

LOAM框架是14年提出的三维激光框架，非常经典的激光里程记和建图方案，也是其他 LOAM 方案的鼻祖，LOAM 只基于激光雷达（可选 IMU），通过把 SLAM 拆分成一个高频低精的前端以及一个低频高精的后端来实现 lidar 里程记的实时性。

其系统架构如下，前端就是激光配准和点云里程计，里程计输出会有一个漂移，所以后面跟着一个Mapping的环节来优化这种情况，并且LOAM没有回环检测功能

在里程计部分，通过提取特征点，加上优化点到线的距离和点到平面的距离并且采用L\-M方法求最优解，得到两帧点云之间的位姿关系

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWViZjdhYzQ2YmUxOGRhZmM3MDk1YWYwNWEyZjIxMmNfMjAzZmFkYThjZmE0YzdkZWM4YTMxMTI0MWJhNTg3YTJfSUQ6NzU3NTIyNTQ2NDkwNjY0ODc4MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

在后面的地图部分，还会进行一次地图匹配，重新计算一次位姿变换，并且对两次的数据进行融合

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGQyMjE0NDhkYTNkOTMyOGZiOTIwZDYyNzY0YmNhZDJfMjZlYzkxMWI2MzUzODI3MTg5NmMxYmY2YzBkODBjYjNfSUQ6NzU3NTIyNTQ1ODY5OTE5MzU2OV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

### 特征点提取

在LOAM的代码中，特征点提取部分考虑了特征点的均匀性，按照角度把点云分为了四个部分或者说四个扇形，每个扇形里面提取出固定数量的角点和面点

同时，对于一些特殊情况（如存在遮挡等情况的点云，也就是坏点）进行了剔除操作

### 点云配准

为了找到特征点的对应关系，要进行一次运动畸变矫正，然后才可以更为准确的进行运动畸变矫正

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MmJlMDdhNTUzZWZlYTRkNzBlNGI5MzcwMmQ4MDRhYWRfZWQ2YzU0MGZmZDNjZTg1Y2JkYjI5ZmM3YTc2ZWEzY2NfSUQ6NzU3NTIyNTQ2Mjc4MDE4NTc5N18xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

## LEGO\-LOAM

这是IROS2018的文章，基于LOAM进行了改进，并且一个很大的改进就是加入了回环检测功能

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGJhYzZmMTExMTIyNzgxMjUxYTEwY2ZmOWMyNTIzZjdfYzVjMDEwNjdjZjFlODM3MjExZDg2MjdlYmU3NDc3OTZfSUQ6NzU3NTIyNTQ1MDEwMDcwNjUxMF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## Cartographer

这是一个非常精简的框架，基于图优化算法，其在算法上并没有非常先进，但是其优点在于代码工程化非常好，而且不需要依赖于PCL、g2o等庞大的代码库

其分前端和后端，并且分别维护局部坐标系和全局坐标系（也就是 local 和 global），局部坐标系是不会动的，建图之后就固定了，而全局坐标系是动态维护更新的

在前端部分，其会将一次 scan 来与子地图（也就是submap）进行匹配，并且使用非线性优化来进行估计

地图是一种概率栅格地图，其中表达的是此处有障碍或者被占用的概率是多少，而且将雷达点分为了两部分，如果点落在某一个格子上，则此格子称为 hit（意思是打中了障碍物），而击中点与雷达中心之间连线所经过的格子则称为miss（意思是这个路径是空的或者说通畅的）

### Branch\-and\-Bound scan matching

分支定界算法，最初来源于数学中的混合整数线性规划问题，在Cartographer中的核心思想是，将解空间划分为树状结构，每一个节点代表一个解，叶子节点代表真正的解，这个算法分为三部分：选择，分支，定界

选择算法是使用深度优先搜索也就是DFS算法

## FAST\-LIO框架

创新点有二：一是提出了一种紧耦合的迭代卡尔曼滤波方法，以此融合雷达特征点和 IMU，二是提出了新的卡尔曼滤波增益公式

FAST\-LIO的主体框架如下图所示，输入部分是雷达和IMU，雷达的输入是100k\-500kHz，实际上因为这是固态雷达，一个点就相当于一个采样，所以是一秒有100k\-500k的点云输入，当然一个点是无法进行SLAM的，所以会积累几十或者上百k个单位的点之后才会作为一次输入送入SLAM（一般取决于里程计频率，比如说里程计频率为50Hz时，则在20ms内积累点云，成为一帧点云并处理一次，这20ms内的点云为一次scan），然后进行特征提取（方法与LOAM一致，就是面点和角点提取，或者说是平面点和边缘点）；而IMU的输入会送入前向传播模块（这里传播的是状态量，也就是待估计的位移、偏置等），前向传播的目的有

- 在两帧雷达数据之间有很多帧IMU数据，将这些IMU数据进行粗略积分，得到位姿变化的估计，用于卡尔曼滤波的预测值、反向传播的运动补偿

- 传递误差量，尤其是传递误差量的变化和协方差矩阵（在后面迭代更新的时候会用到）

然后就是反向传播和运动补偿，因为进行了前向传播，所以每一帧的IMU都有一个预估的位姿，通过这个位姿对激光雷达点云数据进行运动补偿，降低运动产生的失真

然后将补偿后的点云和预估的状态量进行一个残差的计算，然后通过误差状态卡尔曼滤波器进行迭代更新状态量，如果收敛的话就进行一个里程计输出，否则继续进行迭代

收敛并且输出里程计之后，就会根据里程计信息或者说位姿信息，把新的点云插入到全局地图中去，当然也会在全局地图中根据位姿信息进行一个下采样，得到一个子地图，根据这个子地图去寻找近邻点进行匹配

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzY0OWUyY2RkZWVmM2MxYzdhOWU5MTI0ODIzZGRlNjdfNDViMTJiZmM4NTI0MGQ1MTE4ODY3N2U3NzMzMDk4YjVfSUQ6NzU3NTIyNTQ1NTY0MTQ4MDQxMF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

在一次scan的过程中，记录起止时间为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTI1ZWU0OWVhYzk3MjM0NmRmMjMxMjYxYzZmNjQ5YTlfMjAzN2JmZGY4OTdiMmQyOWFkZTg0NGM1YmM4OWU4ZjRfSUQ6NzU3NTIyNTQ1OTg2NTE3NzMxMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjJkMjEwM2E2YmU3YjYxOTJjYTJhOTFjYmQ0NzhhY2VfZTIyN2NmYTJiOTc1Y2E1OThmNjQwYWZlZjljYTc5YWNfSUQ6NzU3NTIyNTQ1MDg2NDIxNzI4OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

，后者为当前时刻，可以认为点云特征是在时间轴上均匀分布并且首尾对齐的，但是IMU的数据并非首尾对齐的，所以需要统一投影到当前时刻下或者说最后一帧点云下，这里使用 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDUwNjEzNjYyYjA3MTVjN2VjODYzOWExYTZjOWFiYmZfYzQ5NGIxODcxNWMwODAxYzQyZjFkNzU1NGYyMzJmMzlfSUQ6NzU3NTIyNTQ1NDg4MjU0MDc2N18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzQ4ZWUwZjU5OTVhOWNjNjY1N2E1YTA3ZDhiNDhkM2VfYjBlNTg3N2I1MjRjZDliNzEzNjNmNmM2ZDEzZTA0MzNfSUQ6NzU3NTIyNTQ1MTM0NjYxMTM4OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

来表示 IMU 和 LiDAR 的时刻

### 数学公式

这里定义了一种广义的加减法或者说操作，首先我们定义 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDJkOWI0ZDEwMDliMjg2N2I1ZTNjMWJiMzEwY2QzNmFfYTc0YjVhOTg4NWVmZDcwNWFhMzg2MWVkMjBiMTUyNGFfSUQ6NzU3NTIyNTQ1NDM5NTcwNjU3MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

是流形，例如 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTU3OWVhYWRjMmYxODU5NzM5Yjk5ZDVlYjI3OGI3MDlfOTdlZWJkYTQ5MDIyMzI2NDU5Y2RkOGYyNDcwZGQ1N2FfSUQ6NzU3NTIyNTQ2MzgzNzE2NjU0NV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWVhNGNjYmEzZGM3YzI3ODgyNzc4YzkzNjFkODZjNWRfZDI0NDMwZTZkNmU1ZTFjYmRhZGFhODgzZTgyZTYyMmFfSUQ6NzU3NTIyNTQ2Mzg3NDgwMDg0MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

如果 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2Q1MjUwYWVhMDI5Y2NlZjRlYmIwODU2OTgwM2QzM2RfMjc0NzhiNDJiZGMxYTU4NDc4ZTI2YjcwZmQxZmI2OThfSUQ6NzU3NTIyNTQ2MTU5MzU0MTg1M18xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

是李群，那么则有：

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGM4NDdkMGFiNTIxMGE1ZjgzNzc2ZTAzNDI0Yjg1NmVfYTg4NmEyODljMWY1NTVlYzU2MTdjMjg2MGQwMTIxNjlfSUQ6NzU3NTIyNTQ1NTY5MjAwODY2Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

如果是向量的话，那么广义加减就定义为向量的加减

实际上就相当于使用了一个 C\+\+ 中的运算重载，以此来方便运算

### IMU离散模型

在模型中，当时间间隔为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDcwMzgzMGE0OWQ3YTE5YzRmYTUxNWU5ZTI4OWI2YWVfMWU4NWE0NGUxMThjNDc5OTYwYmNjYjIwNWU3NmIwMmVfSUQ6NzU3NTIyNTQ1OTkzNjM4MjE1M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

的时候，设状态量为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTIzMDJlNDExNjI1MDA2YWJmOTMyYzI3NTQ5NjcyNmFfMmFjMGRjZTNkZjVkZmM3ZmMzMzg4YzUzMjlhZDAxNzhfSUQ6NzU3NTIyNTQ1NTY5MTk5MjI3OF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

，则离散模型中的数据可以表示为

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDlmMjhlN2U5NTQ0OTY0ZmZmMzZmM2Y5OGI0OGMyYWJfMDE4ZTcyN2IwNjc3YjUzY2QwZjNlNDg0MjZhMWI2N2RfSUQ6NzU3NTIyNTQ2MjM5MDE2NDY4Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

其中

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MmI4MDRlNWQzODVmNTFiOGE4NDQxNzBmOTcxYzViZTFfNjNjMzUzYzcxMjBlNzYxYzY3OTc5YjgxYzBhN2QyMjVfSUQ6NzU3NTIyNTQ1MTAzMTk1NjcwNF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

是状态量，即为所有的待估计变量，共18维，我们要实时估计的是一个18维的量，它包含旋转矩阵，位置，速度、角速度零偏、加速度零偏以及重力向量。因此它是一个紧耦合的框架。

输入是 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGZiMjcyYTU4MTRiNzdjZDZlOGY1OGRhNzJjMmMzZThfZGYzZmEwZjQ0YTkxZDUyN2RmYWNiYWI1ODc2ZTJiMWZfSUQ6NzU3NTIyNTQ0Njk4MDE5MzQ5OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

（也即加速度测量值）,

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWI1ZDVhZDczOWJjYjAwOTM3ZDFhMGYwZDJhZTkxMDlfYzJmN2E0YzM1ZmI5MWM3MTYyMTUzYzA5YTIxZWVjY2FfSUQ6NzU3NTIyNTQ0OTgxNTU0MjczN18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

为所有的噪声量，包含测量噪声（前两项）和bias随机游走噪声（后两项），每一项都是三维的

而论文中的离散模型如下所示

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NWNkNjJiNzZmMmJiY2EyZjgwOGIzZmQ2ZDliMGEyNGFfYTBiMGJmOTFhNWQ2ZjI0ZWJkZjQ2MGM3NGFjNDdlYTdfSUQ6NzU3NTIyNTQ1MTE5MTM4OTE0M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmFlY2UxYWVjNzU5NDAxN2NjOTI0NDNlNWMzYTc4NmZfM2FjYWVlN2QzZTEzMzZiZGE2ZjcxNzkyYTkxMGYwMzdfSUQ6NzU3NTIyNTQ2MDIzMDA2NTM1NV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

是 IMU 测量的序号或者说索引，这个索引并不是整个过程中的，而是两帧 LiDAR 之间的索引，也就是每测量一帧的 LiDAR 都会执行一次上面的公式，然后重新计算 IMU 测量的索引

其中

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTMwN2RkYzU3ZGU4MGE1YzBlMTI1ZGE1OTc5NjM4OWVfYzM2YjllZmRiZTQwZDk1OTBmNTlmMTM1NjAzNzRmYmJfSUQ6NzU3NTIyNTQ1OTYxNzcxMzM1Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

实际上广义加可以拆开来看，不同的位置单独执行广义计算

### 状态估计

这里使用纯字母表示真值，使用波浪线上标表示误差，使用横线表示最优估计，使用上三角表示估计值

### 前向传播

前向传播有两个内容，第一个就是基于IMU积分计算一个粗略的状态量，这个状态量用于后续的反向传播来补偿运动失真
$\hat{x}_{i+1}=\hat{x}_i\boxplus (\Delta tf(\hat{x}_i,u_i,0)),\hat{x}_0=\bar{x}_{k-1}$
其中的时间差表示的是相邻两帧 IMU 的时间差，噪声量为0是因为不知道噪声的实际大小，因此在传播过程中设为0，但是会在后续的误差状态方程中考虑噪声

这个公式与离散模型公式一致，我们每接收一个 IMU 都会进行一次上述计算，直到计算到最后一个 IMU 帧为止。

另一个内容是传播误差量，并计算对应的协方差矩阵。这里的问题是我们不知道真值，怎么计算误差呢？实际上，我们计算的误差量，也是一个近似值，因此它才会有对应的协方差矩阵来评判置信度。和传统的卡尔曼滤波器不同的是，传统的卡尔曼滤波器直接估计状态量，它的运动方程和观测方程通常长这样：
$x_k=f(x_{k-1},u_k)+w_k\\z_k=h(x_k)+v_k$
而文中使用的误差状态卡尔曼滤波器\(Error state Kalman flter，ESKF\)，以误差量作为待估计量，也就是把上式的 x 用 $\widetilde{x}$ 代替

我们现在要估计的是误差量，而不是直接估计状态量。而有了误差量的估计，再直接加上状态量的估计就是我们求得的最优估计，其中误差量
$\widetilde{x}_{k-1}=x_{k-1}\boxminus \bar{x}_{k-1}$

这将带来以下好处:

- 在旋转的处理上，ESKF的状态变量可以采用最小化的参数表达，也就是使用三维变量来表达旋转的增量。而传统KF需要用到四元数或者更高维的表达（如九维旋转矩阵），或采用带有奇异性的表达方式\(欧拉角\)

- ESKF 总是在原点附近，离奇异点较远，并且也不会由于离工作点太远而导致线性化近似不够的问题

- ESKF的状态量为小量，其二阶变量相对来说可以忽略。同时大多数雅可比矩阵在小量情况下变得非常简单，甚至可以用单位阵代替

- 误差状态的运动学也相比原状态变量要来得更小，因为我们可以把大量更新部分放到原状态变量中

# 卡尔曼滤波

现在以误差状态卡尔曼滤波为主

## ESKF公式推导

在现代的大多数IMU系统中，人们往往使用误差状态卡尔曼滤波器（Error state Kalman filter, ESKF）而非原始状态的卡尔曼滤波器。大部分基于滤波器的LIO或VIO实现中，都使用ESKF作为状态估计方法。相比于传统KF，ESKF的优点可以总结如下：

1. 在旋转的处理上，ESKF的状态变量可以采用最小化的参数表达，也就是使用三维变量来表达旋转的增量。而传统KF需要用到四元数（4维）或者更高维的表达（旋转矩阵，9维），要不就得采用带有奇异性的表达方式（欧拉角）。

2. ESKF总是在原点附近，离奇异点较远，并且也不会由于离工作点太远而导致线性化近似不够的问题。

3. ESKF的状态量为小量，其二阶变量相对来说可以忽略。同时大多数雅可比矩阵在小量情况下变得非常简单，甚至可以用单位阵代替。

4. 误差状态的运动学也相比原状态变量要来得更小，因为我们可以把大量更新部分放到原状态变量中。

在ESKF中，我们通常把原状态变量称为**名义状态变量（Nominal State）**，然后把ESKF里的状态变量称为**误差状态变量（Error State）**。

- **标称状态**：承载了系统运动的主要分量（大信号），其动力学方程通常是非线性的。在滤波过程中，标称状态根据IMU数据进行积分预测，并在每次测量更新后加上误差状态的估计值进行修正。

- **误差状态**：表示真值与标称值之间的微小偏差（小信号）。由于误差量通常在零附近微小波动，其动力学方程极其适合进行线性化处理，满足卡尔曼滤波对线性高斯系统的假设。

ESKF整体流程如下：当IMU测量数据到达时，我们把它积分后，放入名义状态变量中。由于这种做法没有考虑噪声，其结果自然会快速漂移，于是我们希望把误差部分作为误差变量，放在ESKF中。ESKF内部会考虑各种噪声和零偏的影响，并且给出误差状态的一个高斯分布描述。同时，ESKF本身作为一种卡尔曼滤波器，也具有预测过程和修正过程，其中修正过程需要依赖IMU以外的传感器观测。当然，在修正之后，ESKF可以给出后验的误差高斯分布，随后我们可以把这部分误差放入名义状态变量中，并把ESKF置零，这样就完成了一次循环

注意一下，标称状态并不是测量值的等价概念，其准确定义是基于运动模型和 IMU 测量值（作为控制输入），在不考虑噪声的情况下推演出的“理想”预测值，是需要进一步优化的状态，但是其实际上也是测量得出的——基于上一次优化后的状态然后结合测量值的积分计算得出

### ESKF状态方程

然后可以认为系统的真实状态是标称状态加上一个误差状态的，因此可以定义真实状态变量、名义状态变量和误差状态变量为：

$\begin{gathered}
\boldsymbol x_t=[\boldsymbol{p}_t ,\boldsymbol{R}_t ,\boldsymbol{v}_t ,\boldsymbol{b}_{a,t},\boldsymbol{b}_{g,t},\boldsymbol{g}_t]^T \newline
\boldsymbol x=[\boldsymbol{p} ,\boldsymbol{R} ,\boldsymbol{v} ,\boldsymbol{b}_a,\boldsymbol{b}_g,\boldsymbol{g}]^T \\
\delta \boldsymbol x=[\delta\boldsymbol{p} ,\delta\boldsymbol{R} ,\delta\boldsymbol{v} ,\delta\boldsymbol{b}_a,\delta\boldsymbol{b}_g,\delta\boldsymbol{g}]^T
\end{gathered}$

其中 $p$为相对于世界坐标系的平移，$R$ 为相对于世界坐标系的旋转，$v$为相对于世界坐标系的速度，$b_a$为当前时刻的加速度计随机游走偏置，$b_g$为陀螺仪的随机游走偏置，$g$为世界坐标系下的重量向量，每个状态量的自由度为 3 维，其中带下标 $t$的表示真值，并且认为各种状态都是时间的函数

然后根据相关理论，很容易推导出状态变量导数相对于观测量的关系式，其中，在连续时间上我们记录 IMU 读数为 $\tilde{\boldsymbol{\omega}}$ 与 $\tilde{\boldsymbol{a}}$：

$\begin{gather}
\dot{\boldsymbol{p}}_t = \boldsymbol{v}_t\\
\dot{\boldsymbol{v}}_t = \boldsymbol{R}_t (\tilde{\boldsymbol{a}} - \boldsymbol{b}_{a,t} - \boldsymbol{\eta}_a) + \boldsymbol{g}\\
\dot{\boldsymbol{R}}_t = \boldsymbol{R} _t(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge\\
\dot{\boldsymbol{b}}_{g,t} = \boldsymbol{\eta}_{b,g}\\
\dot{\boldsymbol{b}}_{a,t} = \boldsymbol{\eta}_{b,a}\\
\dot{\boldsymbol{g}} = \boldsymbol{0}
\end{gather}$

这里把重力考虑进来的主要理由是方便确定IMU的初始姿态。如果我们不在状态方程里写出重力变量，那么必须事先确定初始时刻的IMU朝向 $\boldsymbol R(0)$，才可以执行后续的计算。此时IMU的姿态就是相对于初始的水平面来描述的。而如果把重力写出来，就可以设IMU的初始姿态为单位矩阵，而把重力方向作为IMU当前姿态相比于水平面的一个度量。二种方法都是可行的，不过将重力方向单独表达出来会使得初始姿态表达更加简单，同时还可以增加一些线性性

六项公式很容易理解：

1. 对位置求导获取速度

2. 世界坐标系下的加速度变换，其中的其中的 $\boldsymbol{n}_a$是加速度计高斯白噪声

3. 旋转向量与旋转矩阵的导数之间的变换关系，其中的 $\hat{\boldsymbol \omega}$是陀螺仪的测量值， $\boldsymbol{n}_\omega$是陀螺仪高斯白噪声

4. 陀螺仪偏置求导，认为导数是高斯白噪声

5. 加速度计偏置求导，认为导数是高斯噪声

6. 重力认为是常量，导数为零

如果把观测量和噪声量整理成一个向量，我们也可以把上式整理成矩阵形式。不过这里的矩阵形式将含有很多的零项，相比上式并不会有明显简化，所以我们就先使用这种散开的公式。下面我们来推导误差状态方程。首先定义误差状态变量为：

$\begin{gather}
\boldsymbol{p}_t = \boldsymbol{p} + \delta\boldsymbol{p}\\
\boldsymbol{R}_t = \boldsymbol{R}\,\delta\boldsymbol{R}\\
\boldsymbol{v}_t = \boldsymbol{v} + \delta\boldsymbol{v}\\
\boldsymbol{b}_{a,t} = \boldsymbol{b}_a + \delta\boldsymbol{b}_a\\
\boldsymbol{b}_{g,t} = \boldsymbol{b}_g + \delta\boldsymbol{b}_g\\
\boldsymbol{g}_t = \boldsymbol{g} + \delta\boldsymbol{g}
\end{gather}$

这里其他的项都是线性的，因此直接叠加，但是旋转矩阵是不满足加法而满足乘法的，因此是相乘，并且误差旋转矩阵是相对于机身坐标系而不是世界坐标系的旋转误差，此外 IMU 一般是固定在机身上，因此是右乘

不带下标的就是名义状态变量，名义状态变量的运动学方程式与真值相同，只是不必考虑噪声（因为噪声在误差状态方程中考虑了）。其中旋转部分的 $\delta\boldsymbol{R}$ 可以用它的李代数 $\text{Exp}(\delta\boldsymbol{\theta})$ 来表示，此时旋转公式也需要改成用指数形式来表达。关于误差变量的平移、零偏和重力公式，都很容易得出对应的时间导数表达式，只需在等式两侧分别对时间求导即可

$\begin{gather}
\delta \dot{\boldsymbol{p}}=\delta {\boldsymbol{v}} \\
\delta \dot{\boldsymbol{b}}_g=\boldsymbol \eta_g \\
\delta \dot{\boldsymbol{b}}_a =\boldsymbol \eta_a \\
\delta {\boldsymbol{g}}=0 \\
\end{gather}$

其中因为速度和旋转两个方程与 $\delta \boldsymbol R$ 有关，需要单独推导，具体推导过程在下面两小节给出，这里先给出完整的误差变量的运动学状态方程

$\begin{split}
    \delta \dot{\boldsymbol{p}} &= \delta \boldsymbol{v} \\
    \delta \dot{\boldsymbol{v}} &= -\boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a)^\wedge \delta \boldsymbol{\theta} - \boldsymbol{R} \delta \boldsymbol{b}_a - \boldsymbol{\eta}_a + \delta \boldsymbol{g} \\
    \delta \dot{\boldsymbol{\theta}} &= -(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_g)^\wedge \delta \boldsymbol{\theta} - \delta \boldsymbol{b}_g - \boldsymbol{\eta}_g \\
    \delta \dot{\boldsymbol{b}}_g &= \boldsymbol{\eta}_{bg} \\
    \delta \dot{\boldsymbol{b}}_a &= \boldsymbol{\eta}_{ba} \\
    \delta \dot{\boldsymbol{g}} &= \boldsymbol{0}
\end{split}$

### 误差状态的旋转项

将旋转误差方程两侧分别对时间求导可得：

$\begin{gather}
\begin{split}
\dot{\boldsymbol{R}}_t &= 
\dot{\boldsymbol{R}}\delta \boldsymbol{R}+
\boldsymbol{R}\delta \dot{\boldsymbol{R}}\\
&=\dot{\boldsymbol{R}}\text{Exp}(\delta\boldsymbol{\theta}) + \boldsymbol{R}\dot{\text{Exp}(\delta\boldsymbol{\theta})} \\
&= \boldsymbol{R} _t(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge 
\end{split}
\end{gather}$

又有公式：$\dot{\text{Exp}(\delta\boldsymbol{\theta})}=\text{Exp}(\delta\boldsymbol{\theta})\delta\dot{\boldsymbol{\theta}}^\wedge$，可以将其中的对应项进行转换，将第二行化为如下形式，并且标称状态是不考虑噪声的理想值，因此噪声项为 0

$\begin{gather}
\dot{\boldsymbol{R}}\text{Exp}(\delta\boldsymbol{\theta}) + \boldsymbol{R}\dot{\text{Exp}(\delta\boldsymbol{\theta})} = \boldsymbol{R} \left( \tilde{\boldsymbol \omega} - \boldsymbol{b}_g \right)^\wedge \text{Exp}(\delta\boldsymbol{\theta}) + \boldsymbol{R}\text{Exp}(\delta\boldsymbol{\theta})\delta\dot{\boldsymbol{\theta}}^\wedge
\end{gather}$

再将第三行的真值消去，有如下形式

$\begin{gather}
\boldsymbol{R}_t (\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge = \boldsymbol{R}\text{Exp}(\delta\boldsymbol{\theta}) (\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge
\end{gather}$

根据公式20可知，公式21、22是相等的，因此可以联立两式，将其中的 $\dot{\delta \boldsymbol{\theta}}^\wedge$ 移动到一侧，并且约掉左侧的旋转矩阵，并且整理类似项，可以有如下形式

$\begin{gather}
\text{Exp}(\delta\boldsymbol{\theta})\dot{\delta \boldsymbol{\theta}}^\wedge = \text{Exp}(\delta\boldsymbol{\theta}) (\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{n}_g)^\wedge - \left( \tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t}\right)^\wedge \text{Exp}(\delta\boldsymbol{\theta})
\end{gather}$

注意 $\text{Exp}(\delta\boldsymbol{\theta})$ 本身是一个SO\(3\)矩阵，利用SO\(3\)上的伴随性质用来交换，且其中根据旋转矩阵的性质有：$\text{Exp}(\delta\boldsymbol{\theta})^T=\text{Exp}(-\delta\boldsymbol{\theta})$

李群的伴随性质为：$\boldsymbol\phi^\wedge\boldsymbol R=\boldsymbol R(\boldsymbol R^T \boldsymbol \phi)^\wedge$

然后李群的伴随性质，可以将上面公式至的最后一项 $\left( \tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t}\right)^\wedge \text{Exp}(\delta\boldsymbol{\theta})$项进行交换

$\begin{gather}
\begin{split}
\text{Exp}(\delta\boldsymbol{\theta})\delta\dot{\boldsymbol{\theta}}^\wedge
&= \text{Exp}(\delta\boldsymbol{\theta}) (\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge-\text{Exp}(\delta\boldsymbol{\theta}) [\text{Exp}(-\delta\boldsymbol{\theta})(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g})]^\wedge\\
&= \text{Exp}(\delta\boldsymbol{\theta}) [(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge-(\text{Exp}(\delta\boldsymbol{\theta})(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g} ))^\wedge]\\
&\approx \text{Exp}(\delta\boldsymbol{\theta})[(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g,t} - \boldsymbol{\eta}_g)^\wedge-((\boldsymbol I-\delta\boldsymbol{\theta}^\wedge)(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_{g} ))^\wedge]\\
&= \text{Exp}(\delta\boldsymbol{\theta})[\boldsymbol{b}_{g}-\boldsymbol{b}_{g,t}-\boldsymbol{\eta}_g+\delta\boldsymbol{\theta}^\wedge \tilde{\boldsymbol{\omega}}-\delta\boldsymbol{\theta}^\wedge\boldsymbol{b}_{g}]^\wedge\\
&= \text{Exp}(\delta\boldsymbol{\theta})[(-\tilde{\boldsymbol{\omega}}+\boldsymbol{b}_g)^\wedge \delta\boldsymbol{\theta}-\delta\boldsymbol{b}_g-\boldsymbol{\eta}_g]^\wedge

\end{split}
\end{gather}$

然后约掉左侧的系数 $\text{Exp}(\delta\boldsymbol{\theta})$即可得到：

$\begin{gather}
\delta\dot{\boldsymbol{\theta}} \approx \left( -\tilde{\boldsymbol{\omega}} + \boldsymbol{b}_g \right)^\wedge \delta\boldsymbol{\theta} - \delta\boldsymbol{b}_g - \boldsymbol{\eta}_{g}
\end{gather}$

### 误差状态的速度项

接下来考虑速度方程的误差形式，获取误差状态速度项的表达式，对速度求导即为加速度，因此速度真值的导数等于加速度真值，根据状态方程有：

$\begin{gather}
\begin{split}
    \dot{\boldsymbol{v}}_t &= \boldsymbol{R}_t (\tilde{\boldsymbol{a}} - \boldsymbol{b}_{a,t} - \boldsymbol{\eta}_a) + \boldsymbol{g}_t \\
    &= \boldsymbol{R} \text{Exp}(\delta\boldsymbol{\theta}) (\tilde{\boldsymbol{a}} - \boldsymbol{b}_a - \delta\boldsymbol{b}_a - \boldsymbol{\eta}_a) + \boldsymbol{g} + \delta\boldsymbol{g} \\
    &\approx \boldsymbol{R} (\boldsymbol{I} + \delta\boldsymbol{\theta}^\wedge) (\tilde{\boldsymbol{a}} - \boldsymbol{b}_a - \delta\boldsymbol{b}_a - \boldsymbol{\eta}_a) + \boldsymbol{g} + \delta\boldsymbol{g} \\
    &\approx \boldsymbol{R}\tilde{\boldsymbol{a}} - \boldsymbol{R}\boldsymbol{b}_a - \boldsymbol{R}\delta\boldsymbol{b}_a - \boldsymbol{R}\boldsymbol{\eta}_a + \boldsymbol{R}\delta\boldsymbol{\theta}^\wedge \boldsymbol{a} - \boldsymbol{R}\delta\boldsymbol{\theta}^\wedge \boldsymbol{b}_a + \boldsymbol{g} + \delta\boldsymbol{g} \\
    &= \boldsymbol{R}\tilde{\boldsymbol{a}} - \boldsymbol{R}\boldsymbol{b}_a - \boldsymbol{R}\delta\boldsymbol{b}_a - \boldsymbol{R}\boldsymbol{\eta}_a - \boldsymbol{R}\tilde{\boldsymbol{a}}^\wedge \delta\boldsymbol{\theta} + \boldsymbol{R}\boldsymbol{b}_a^\wedge \delta\boldsymbol{\theta} + \boldsymbol{g} + \delta\boldsymbol{g}
\end{split}

\end{gather}$

从第三行推向第四行时，需要忽略 $\delta\boldsymbol{\theta}^\wedge$ 与 $\boldsymbol{\eta}_a$以及 $\delta\boldsymbol{b}_a$ 相乘的二阶小量。从第四行推第五行则用到了叉乘符号交换顺序之后需加负号的性质。另一方面，等式右侧为

$\begin{gather}
 \dot{\boldsymbol{v}} + \delta\dot{\boldsymbol{v}} = \boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a) + \boldsymbol{g} + \delta\dot{\boldsymbol{v}}
\end{gather}$

因为上面两式是相等的，因此可以得到

$\begin{gather}
\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a)^\wedge \delta\boldsymbol{\theta} - \boldsymbol{R} \delta\boldsymbol{b}_a - \boldsymbol{R} \boldsymbol{\eta}_a + \delta\boldsymbol{g} 
\end{gather}$

这样我们就得到了 $\delta{\boldsymbol{v}}$ 的运动学模型。需要补充一句，由于上式中 $\boldsymbol{\eta}_a$ 是一个零均值白噪声，它乘上任意旋转矩阵之后仍然是一个零均值白噪声，而且由于 $\boldsymbol{R}^T\boldsymbol{R}=\boldsymbol{I}$ ，其协方差矩阵也不变（留作习题）。所以，也可以把上式简化为：

$\begin{gather}
\delta\dot{\boldsymbol{v}} = -\boldsymbol{R}(\bar{\boldsymbol{a}} - \boldsymbol{b}_a)^\wedge \delta\boldsymbol{\theta} - \boldsymbol{R} \delta\boldsymbol{b}_a - \boldsymbol{\eta}_a + \delta\boldsymbol{g}
\end{gather}$

### 离散时间ESKF运动学方程

上面给出的是连续时间下的状态方程，但是计算机只能处理离散数据，而如果进行数值近似的话会导致计算量的暴增，因此需要转换为离散时间下的状态方程，很容易可以得出名义状态变量的离散时间方程：

$\begin{split}
    \boldsymbol{p}(t + \Delta t) &= \boldsymbol{p}(t) + \boldsymbol{v}\Delta t + \frac{1}{2} (\boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a)) \Delta t^2 + \frac{1}{2} \boldsymbol{g} \Delta t^2 \\
    \boldsymbol{v}(t + \Delta t) &= \boldsymbol{v}(t) + \boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a) \Delta t + \boldsymbol{g} \Delta t \\
    \boldsymbol{R}(t + \Delta t) &= \boldsymbol{R}(t) \text{Exp}\left((\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_g)\Delta t\right) \\
    \boldsymbol{b}_g(t + \Delta t) &= \boldsymbol{b}_g(t) \\
    \boldsymbol{b}_a(t + \Delta t) &= \boldsymbol{b}_a(t) \\
    \boldsymbol{g}(t + \Delta t) &= \boldsymbol{g}(t)
\end{split}$

该式只需在上面的基础上添加零偏项与重力项即可。而误差状态的离散形式则只需要处理连续形式中的旋转部分。参考角速度的积分公式，可以将误差状态方程写为：

$\begin{split}
    \delta\boldsymbol{p}(t + \Delta t) &= \delta\boldsymbol{p} + \delta\boldsymbol{v} \Delta t \\
    \delta\boldsymbol{v}(t + \Delta t) &= \delta\boldsymbol{v} + \left( -\boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a)^\wedge \delta\boldsymbol{\theta} - \boldsymbol{R}\delta\boldsymbol{b}_a + \delta\boldsymbol{g} \right) \Delta t + \boldsymbol{\eta}_v \\
    \delta\boldsymbol{\theta}(t + \Delta t) &= \text{Exp}\left( -(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_g)\Delta t \right) \delta\boldsymbol{\theta} - \delta\boldsymbol{b}_g \Delta t - \boldsymbol{\eta}_\theta \\
    \delta\boldsymbol{b}_g(t + \Delta t) &= \delta\boldsymbol{b}_g + \boldsymbol{\eta}_g \\
    \delta\boldsymbol{b}_a(t + \Delta t) &= \delta\boldsymbol{b}_a + \boldsymbol{\eta}_a \\
    \delta\boldsymbol{g}(t + \Delta t) &= \delta\boldsymbol{g}
\end{split}$

注意：

1. 右侧部分我们省略了括号里的 $t$以简化公式；

2. 关于旋转部分的积分，我们可以将连续形式看成关于 $\delta \boldsymbol\theta$ 的微分方程然后求解。求解过程类似于对角速度进行积分。

3. 噪声项并不参与递推，需要把它们单独归入噪声部分中。连续时间的噪声项可以视为随机过程的能量谱密度，而离散时间下的噪声变量就是我们日常看到的随机变量了。这些噪声随机变量的标准差可以列写如下： 

    $\sigma(\boldsymbol{\eta}_v) = \sqrt{\Delta t} \sigma_{a} \quad \sigma(\boldsymbol{\eta}_\theta) = \sqrt{\Delta t} \sigma_{g} \quad \sigma(\boldsymbol{\eta}_g) = \sqrt{\Delta t} \sigma_{bg} \quad \sigma(\boldsymbol{\eta}_a) = \sqrt{\Delta t} \sigma_{ba}$

其中前两式的 $\Delta t$ 是由积分关系导致的。

至此，我们给出了如何在ESKF中进行IMU递推的过程，对应于卡尔曼滤波器中的状态方程。为了让滤波器收敛，我们通常需要外部的观测来对卡尔曼滤波器进行修正，也就是所谓的组合导航。当然，组合导航的方法有很多，从传统的EKF，到本节介绍的ESKF，以及后续章节将要介绍预积分和图优化技术，都可以应用于组合导航中。

### 运动过程

根据上述讨论，我们可以写出ESKF的运动过程。误差状态变量 $\delta \boldsymbol x$ 的离散时间运动方程已经在上式给出，我们可以整体地记为

$\delta\boldsymbol{x} = f(\delta\boldsymbol{x}) + \boldsymbol{w}, \quad \boldsymbol{w} \sim \mathcal{N}(\boldsymbol{0}, \boldsymbol{Q})$

其中 $\boldsymbol w$ 为噪声。按照前面的定义，$\boldsymbol Q$ 应该为：

$\boldsymbol{Q} = \text{diag}(\boldsymbol{0}_3, \text{Cov}(\boldsymbol{\eta}_v), \text{Cov}(\boldsymbol{\eta}_\theta), \text{Cov}(\boldsymbol{\eta}_g), \text{Cov}(\boldsymbol{\eta}_a), \boldsymbol{0}_3)$

两侧的零是由于第一个和最后一个方程本身没有噪声导致的。

为了保持与EKF的符号统一，我们计算运动方程的线性化形式：

$\delta\boldsymbol{x} = \boldsymbol{F}\delta\boldsymbol{x} + \boldsymbol{w}$

其中 $\boldsymbol F$ 为线性化后的雅可比矩阵。由于我们列写的运动方程已经是线性化的了，只需把它们的线性系统拿出来即可

$\boldsymbol{F} = 
    \begin{bmatrix}
        \boldsymbol{I} & \boldsymbol{I}\Delta t & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} \\\boldsymbol{0} & \boldsymbol{I} & -\boldsymbol{R}(\tilde{\boldsymbol{a}} - \boldsymbol{b}_a)^\wedge \Delta t & -\boldsymbol{R}\Delta t & \boldsymbol{0} & \boldsymbol{I}\Delta t \\
\boldsymbol{0} & \boldsymbol{0} & \text{Exp}(-(\tilde{\boldsymbol{\omega}} - \boldsymbol{b}_g)\Delta t) & \boldsymbol{0} & -\boldsymbol{I}\Delta t & \boldsymbol{0} \\\boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} & \boldsymbol{0} \\\boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I} & \boldsymbol{0} \\\boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{0} & \boldsymbol{I}
    \end{bmatrix}$

在此基础上，我们执行ESKF的预测过程。预测过程包括对名义状态的预测（IMU积分）以及对误差状态的预测：

$\delta\boldsymbol{x}_{\text{pred}} = \boldsymbol{F}\delta\boldsymbol{x} \\\boldsymbol{P}_{\text{pred}} = \boldsymbol{F}\boldsymbol{P}\boldsymbol{F}^T + \boldsymbol{Q}$

不过由于ESKF的误差状态在每次更新以后会被重置，因此运动方程的均值部分没有太大意义，而方差部分则可以指导整个误差估计的分布情况

### ESKF的更新过程

前面介绍的是ESKF的运动过程，现在我们来考虑更新过程。假设一个抽象的传感器能够对状态变量产生观测，其观测方程为抽象的 $h$ ，那么可以写为：

$\boldsymbol{z} = h(\boldsymbol{x}) + \boldsymbol{v}, \quad \boldsymbol{v} \sim \mathcal{N}(\boldsymbol{0}, \boldsymbol{V})$

其中 $\boldsymbol{z}$ 为观测数据， $\boldsymbol{v}$为观测噪声， $\boldsymbol{V}$为该噪声的协方差矩阵。由于状态变量里已经有 $\boldsymbol{R}$ 了，这里我们换个符号。

在传统EKF中，我们可以直观对观测方程线性化，求出观测方程相对于状态变量的雅可比矩阵，进而更新卡尔曼滤波器。而在ESKF中，我们当前拥有名义状态 $\boldsymbol{x}$ 的估计以及误差状态 $\delta\boldsymbol{x}$ 的估计，且希望更新的是误差状态，因此要计算观测方程相比于误差状态的雅可比矩阵：

$\boldsymbol{H} = \frac{\partial h}{\partial \delta \boldsymbol{x}}
$

然后再计算卡尔曼增益，进而计算误差状态的更新过程：

$\boldsymbol{K} = \boldsymbol{P}_{\text{pred}} \boldsymbol{H}^T (\boldsymbol{H} \boldsymbol{P}_{\text{pred}} \boldsymbol{H}^T + \boldsymbol{V})^{-1}
\\


\delta \boldsymbol{x} = \boldsymbol{K}(\boldsymbol{z} - h(\boldsymbol{x}_t))
\\
\boldsymbol{P} = (\boldsymbol{I} - \boldsymbol{K} \boldsymbol{H}) \boldsymbol{P}_{\text{pred}}
\\

$

其中 $\boldsymbol{K}$ 为卡尔曼增益，$\boldsymbol{P}_{pred}$ 为预测的协方差矩阵，最后的 $\boldsymbol{P}$为修正后的协方差矩阵。这里的 $\boldsymbol{H}$的计算可以通过链式法则来生成：

$\boldsymbol{H} = \frac{\partial h}{\partial \boldsymbol{x}} \frac{\partial \boldsymbol{x}}{\partial \delta \boldsymbol{x}}$

其中第一项只需对观测方程进行线性化，第二项，根据我们之前对状态变量的定义，可以得到：

$\frac{\partial \boldsymbol{x}}{\partial \delta \boldsymbol{x}} = \text{diag}\left(\boldsymbol{I}_3, \boldsymbol{I}_3, \frac{\partial \log(\boldsymbol{R} \exp(\delta\boldsymbol{\theta}))}{\partial \delta \boldsymbol{\theta}}, \boldsymbol{I}_3, \boldsymbol{I}_3, \boldsymbol{I}_3\right)$

其他几种都是平凡的，只有旋转部分，因为 $\delta \boldsymbol \theta$ 定义为 $\boldsymbol R$ 的右乘，我们用右乘的BCH即可：

$
\frac{\partial \log(\boldsymbol{R} \exp(\delta\boldsymbol{\theta}))}{\partial \delta \boldsymbol{\theta}} = \boldsymbol{J}_r^{-1}(\boldsymbol{R})$

最后，我们可以给每个变量加下标  k，表示在 k 时刻进行状态估计。

### ESKF的误差状态后续处理

在经过预测和更新过程之后，我们修正了误差状态的估计。接下来，只需把误差状态归入名义状态，然后重置ESKF即可。归入部分可以简单地写为：

$\begin{split}
\boldsymbol{p}_{k+1} &= \boldsymbol{p}_k + \delta\boldsymbol{p}_k \\
\boldsymbol{v}_{k+1} &= \boldsymbol{v}_k + \delta\boldsymbol{v}_k \\
\boldsymbol{R}_{k+1} &= \boldsymbol{R}_k \text{Exp}(\delta\boldsymbol{\theta}_k) \\
\boldsymbol{b}_{g,k+1} &= \boldsymbol{b}_{g,k} + \delta\boldsymbol{b}_{g,k} \\
\boldsymbol{b}_{a,k+1} &= \boldsymbol{b}_{a,k} + \delta\boldsymbol{b}_{a,k} \\
\boldsymbol{g}_{k+1} &= \boldsymbol{g}_k + \delta\boldsymbol{g}_k
\end{split}$

有些文献如FAST\-LIO里也会定义为广义的状态变量加法

$\boldsymbol{x}_{k+1} = \boldsymbol{x}_k \oplus \delta\boldsymbol{x}_k\\
\boldsymbol{x}_{k+1} = \boldsymbol{x}_k \boxplus \delta\boldsymbol{x}_k$

这种写法可以简化整体的表达式。不过，如果公式里出现太多的广义加减法，可能让人不好马上辨认它们的具体含义，所以本书还是倾向于将各状态分别写开，或者直接用加法而非广义加法符号。

ESKF的重置分为均值部分和协方差部分。均值部分可以简单地实现为：

$\delta \boldsymbol x=0$

由于均值被重置了，之前我们描述的是关于 $\boldsymbol x_k$ 切空间中的协方差，而现在描述的是 $\boldsymbol x_{k+1}$ 中的协方差。这次重置会带来一些微小的差异，主要影响旋转部分。事实上，在重置前，卡尔曼滤波器刻画了 $\boldsymbol x_{pred}$ 切空间处的一个高斯分布 $\mathcal{N}(\delta \boldsymbol x,\boldsymbol P)$，而重置之后，应该刻画 $\boldsymbol{x}_{pred} \boxplus \delta\boldsymbol{x}_k$ 处的一个 $\mathcal{N}(0,\boldsymbol P_{reset})$。

我们设重置前的名义旋转估计为 $\boldsymbol R_k$，误差状态为 $\delta \boldsymbol \theta$，卡尔曼滤波器的增量计算结果为 $\delta \boldsymbol \theta_k$，注意此处 $\delta \boldsymbol \theta_k$ 是已知的，而 $\delta \boldsymbol \theta$ 是一个随机变量。重置之后的名义旋转部分为 $\boldsymbol R_k \text{Exp}(\delta\boldsymbol{\theta}_k) = \boldsymbol{R}^+$，误差状态为 $\delta\boldsymbol{\theta}^+$。由于误差状态被重置了，显然此时 $\delta\boldsymbol{\theta}^+=0$。但我们关心的并不是它们直接的取值，而是 $\delta\boldsymbol{\theta}^+$ 与 $\delta\boldsymbol{\theta}$ 的线性化关系。把实际的重置过程写出来：

$\boldsymbol{R}^+\text{Exp}(\delta\boldsymbol{\theta}^+) = \boldsymbol{R}_k \text{Exp}(\delta\boldsymbol{\theta}_k) \text{Exp}(\delta\boldsymbol{\theta}^+) = \boldsymbol{R}_k \text{Exp}(\delta\boldsymbol{\theta})$

不难得到

$\text{Exp}(\delta\boldsymbol{\theta}^+) = \text{Exp}(-\delta\boldsymbol{\theta}_k) \text{Exp}(\delta\boldsymbol{\theta})$

注意这里 $\delta \boldsymbol \theta$ 为小量，利用线性化后的BCH公式，可以得到：

$\delta\boldsymbol{\theta}^+ = -\delta\boldsymbol{\theta}_k + \delta\boldsymbol{\theta} - \frac{1}{2} \delta\boldsymbol{\theta}_k^\wedge \delta\boldsymbol{\theta} + o((\delta\boldsymbol{\theta})^2)$

于是有

$\frac{\partial \delta\boldsymbol{\theta}^+}{\partial \delta\boldsymbol{\theta}} \approx \boldsymbol{I} - \frac{1}{2} \delta\boldsymbol{\theta}_k^\wedge$

该式表明重置前后的误差状态相差一个旋转方面的小雅可比矩阵，我们记作 $\boldsymbol{J}_\theta = \boldsymbol{I} - \frac{1}{2} \delta\boldsymbol{\theta}_k^\wedge$ 。把这个小雅可比阵放到整个状态变量维度下，并保持其他部分为单位矩阵，可以得到一个完整的雅可比阵：

$\boldsymbol{J}_k = \text{diag}(\boldsymbol{I}_3, \boldsymbol{I}_3, \boldsymbol{J}_\theta, \boldsymbol{I}_3, \boldsymbol{I}_3, \boldsymbol{I}_3)$

因此，在把误差状态的均值归零同时，它们的协方差矩阵也应该进行线性变换：

$\boldsymbol{P}_{\text{reset}} = \boldsymbol{J}_k \boldsymbol{P} \boldsymbol{J}_k^T$

不过，由于 $\delta \boldsymbol \theta_k$ 并不大，这里的 $\boldsymbol J_k$ 仍然十分接近于单位矩阵，所以大部分材料里并不处理这一项，而是直接把前面估计的 $\boldsymbol P$ 阵作为下一时刻的起点。但本书仍然要介绍这一点，并且会在后面第9章中继续讨论这个问题。该问题实际意义是做了切空间投影，即把一个切空间中的高斯分布投影到另一个切空间中。在ESKF中，两者没有明显差异，但后文的迭代卡尔曼滤波器还牵扯到多次切空间的变换，我们必须在此加以介绍。

# 非线性优化

在SLAM中，经常性会碰到各种优化问题，比如说给一个目标函数，求出使其最小化的解，并且这个目标函数往往是非线性的

当然，目前很多库，比如说Ceres、g2o还有gtsam等库都可以实现非线性优化具体求解操作，我们只需要把待求解函数输入即可

## 概述

在三维世界中，可以通过旋转向量、旋转矩阵、欧拉角和四元数等等方法来描述刚体的运动，并且可以通过李群李代数来进行优化，此外通过相机进行观测世界，但是回归最初的问题，也就是位姿方程和观测方程，其中位姿可以使用变换矩阵描述，然后使用李代数进行优化，观测方程由相机成像模型给出

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTYzZDU5ZDJkZDc1OTlhZjlhZDMwZWVjNjQxMTkzNDRfODg5MjMwNGU1OGU1OTVkOGE5ZDBhMWMwNWYxODc2ZTRfSUQ6NzU3NTIyNTQ0OTMzMzE5ODA0N18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

但是由于噪声的存在，上述等式无法准确成立，则在给定模型和具体观测的时候，需要进行优化，并且是非线性优化

首先可以考虑噪声模型，认为噪声服从正态分布，也即

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGJkNDIxZmYwNDYyOTcyMDVmMjE4NDY2MDAzY2QzMDNfMzQ0MGJlOGFjZTFjNGI4NzA3M2EwYzVlMzhmNWYyNWJfSUQ6NzU3NTIyNTQ2NDc1NTYwNDQzOV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

并且，其他的变量可以做以下定义：

位姿变量：

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTQzYjFjMjA3NzRmZWI3NTYyN2I3MDY2OGVhNzdiYzJfMGUyM2U3NTcyMjUxNmUwNjc0MzVlYjY2MzM0MmI5MWJfSUQ6NzU3NTIyNTQ1MDk5NDI1NzEyMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)



路标点：

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTY3ZmEzMTRmMDI2NDE3NGYxM2YzMTcxMjQ2ZmVlNTZfZmRiNTE0YTllZmM1YWRhZjhlMGQyZDQ5NGE0OTRhYjlfSUQ6NzU3NTIyNTQ2Mjk4MTU5NDMwM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

在 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzQ4YzgzZjQ3YjgzNjdlZTNhNjZjYTEyZGUzMDRlZDRfOGEwMzU2NWY4MWM4OGI0NTg5YzM4NGRiZjUzNjc5ZTBfSUQ6NzU3NTIyNTQ1NTQ0MDQ0ODcxOF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

处对路标 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWJlNzc5ZTMxOWUwN2NjN2NlYWE3MzdiN2IwNWM4MWRfNGJkYTc3ZTc5MGFhZDVkZGIzMzNiYTQzZTAzMGMwYWNfSUQ6NzU3NTIyNTQ1MjMxMTM1MDQ2M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

进行了一次观测，对应到图像上像素位置 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmVhZjdmMTMzMDZhMzI3ZjFlMTU4ZjYwZTQ1ODQzNTFfNGZiZDBlNjJmMTVhM2NhOTg5YjMzMzdkMDFhNTE5YjRfSUQ6NzU3NTIyNTQ1NzQ5OTYwNTk2NV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDVkYzhkOTdiYjhkNjllZGIyYWRiYzZkMTQyN2U1ZWJfN2I1ZDdlNmQ2NTdlZDNjNWI1Y2YwZDVmMTc5NDlhN2VfSUQ6NzU3NTIyNTQ1MDEyMTg0MTg4Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

然后我们得到了带有噪声的观测数据和传感器数据，就可以用来估计位姿 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGY1YmIzZjc3NGE3M2I4ZGI2Y2E3MDdlYmI5OTU2ZmRfOGIxMjBhOTQ4MDUzN2JlMGQ1OTU2ZGYxZWVhNjk5NTFfSUQ6NzU3NTIyNTQ1NzI5ODEzMjE3MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

和地图路标点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGQzOWU0NjA2YjczOGExZTJmYTM2OTEwN2RkMGRkNjBfMmQyN2I0N2FiMmI4NmFhZmM1MjhiNGZlMzMwNTAyZGJfSUQ6NzU3NTIyNTQ1MDEyMTk0MDE4Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

，具体方法可以使用滤波器（卡尔曼滤波）和非线性优化

从概率角度，所有待求解的量称为状态变量

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NWNmZGQwZmJkNTMyMGEyN2Q3YWYxYzRkZjk5MzUzMzZfZmM0NWRmYmRiMWQyZmE5YjZiMDEwZWI3ZWE5NWYwYjlfSUQ6NzU3NTIyNTQ0NjI5MjQ3NTEwM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

状态估计等同于求已知输入数据 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2MxN2FmYzI1NTI4NGE2OWY1NTk5MDVkY2ZjOTQzNzZfYjNjNzg1MzBjN2IwOWQyYzIzNGE4NGVlYWUyYjE5OGVfSUQ6NzU3NTIyNTQ0OTUzMDU0MzMyM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

和观测数据 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzdjMTIzY2ZlMWI3ZmQ5MTA2ZWYwYzI4YTk4Yjc1NjBfYjhiOTE5MTBlZDk1YTI1NGU1MTc3ZGU1ZGJlNzE4YzNfSUQ6NzU3NTIyNTQ2MDA5MTY4NjA3OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

的条件下，状态 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDM2ODU2NWU3OTlkM2FhNDU2NWMwY2EyYjc4NDIyMzlfZDE5ZjkyN2E4ZmVlYjZhZDAwM2ZjYjZmNTE2OTI4NWZfSUQ6NzU3NTIyNTQ0ODYyNDYzOTE3N18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

的条件概率分布

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDk1ZWE0MTEyNzAyN2M0Mjk0OTQ1ZDdkZWRhODY5MTRfOGU0MTZmNTg3ZjNhMmI2Y2QxMDQ2OGQ1ZjMzYTQ3YWVfSUQ6NzU3NTIyNTQzODcxMzI1MjgxOV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

先考虑特殊情况，也就是没有运动测量的传感器，只有观测数据

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Y2Y1ZDZjNjRmMjQ5ODk2OTQzYjZmN2I2M2RiNTA3YTdfNDgzOTBmODc0NjYzYmIzNGE2Njk1ODVmZGZjOWI5Y2NfSUQ6NzU3NTIyNTQzNjY0OTcyMTA1Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

### 数学定义

对于一个函数 $ f : F \subseteq R^n $

- 连续：$f$ 在每一点都连续

- 连续可微：在每一点 $ x \in F $ 处，每一个偏导数 $ \frac{\partial f(x)}{\partial x_i} $ 存在且连续

- 二次连续可微：在每一点 $ x \in F $ 处，每一个偏导数 $ \frac{\partial^2 f(x)}{\partial x_i \partial y_j} $ 存在且连续

如果函数为一阶连续可微的，则

- 梯度\(grad\)：$ \nabla f(x) = \left( \frac{\partial f(x)}{\partial x_1}, \frac{\partial f(x)}{\partial x_2}, \cdots, \frac{\partial f(x)}{\partial x_n} \right)^T $

- 海森阵\(Hessian\)，其总为对称阵：

$\nabla^2 f(x) =
\begin{pmatrix}
\frac{\partial^2 f(x)}{\partial x_1^2} & \frac{\partial f^2(x)}{\partial x_1 \partial x_2} & \cdots & \frac{\partial^2 f(x)}{\partial x_1 \partial x_n} \\[0.5em]
\frac{\partial^2 f(x)}{\partial x_2 \partial x_1} & \frac{\partial f^2(x)}{\partial x_2 \partial x_2} & \cdots & \frac{\partial^2 f(x)}{\partial x_2 \partial x_n} \\[0.5em]
\vdots & \vdots & \ddots & \vdots \\[0.5em]
\frac{\partial^2 f(x)}{\partial x_n \partial x_1} & \frac{\partial f^2(x)}{\partial x_n \partial x_2} & \cdots & \frac{\partial^2 f(x)}{\partial x_n \partial x_n}
\end{pmatrix}_{m \times n}
= \left( \frac{\partial^2 f(x)}{\partial x_i \partial x_j} \right)_{m \times n}$



二次函数 $f(x) = \frac{1}{2}x^TAx + b^Tx + c$ ，其中 $ A \in R^{m \times n} $，$ b \in R^n $，$ c \in R^n$，则有

$\nabla f(x) = Ax + b\\[0.5em]
\nabla^2 f(x) = A\\$

- 若 $ F $ 为多变量矩阵，则其一阶导雅可比阵\(Jacobi\)为

$F'(x) =
\begin{pmatrix}
\frac{\partial F_1(x)}{\partial x_1} & \frac{\partial F_1(x)}{\partial x_2} & \cdots & \frac{\partial F_1(x)}{\partial x_n} \\[0.5em]
\frac{\partial F_2(x)}{\partial x_1} & \frac{\partial F_2(x)}{\partial x_2} & \cdots & \frac{\partial F_2(x)}{\partial x_n} \\[0.5em]
\vdots & \vdots & \ddots & \vdots \\[0.5em]
\frac{\partial F_n(x)}{\partial x_1} & \frac{\partial F_n(x)}{\partial x_2} & \cdots & \frac{\partial F_n(x)}{\partial x_n}
\end{pmatrix}_{m \times n}
= \left( \frac{\partial F_i(x)}{\partial x_j} \right)_{m \times n}$

如多变量函数 $ F(x) = Ax $ 则其 Jacobi 为 $ F'(x) = A $

## 先验、后验、似然

### 后验（知果求因）

假设，隔壁小哥要去15公里外的一个公园，他可以选择步行走路，骑自行车或者开车，然后通过其中一种方式花了一段时间到达公园。这件事中采用哪种交通方式是因，花了多长时间是果。

假设我们已经知道小哥花了1个小时到了公园，那么你猜他是怎么去的\(走路or开车or自行车\)，事实上我们不能百分百确定他的交通方式，我们正常人的思路是他很大可能是骑车过去的，当然也不排除开车过去却由于堵车严重花了很长时间。

这种预先已知结果\(路上花的时间，在机器学习中就是观测到的X\)，然后根据结果估计原因\(交通方式\)的概率分布即后验概率。

### 先验（由历史求因）

假设隔壁小哥还没去,我们根据他的个人历史习惯来推测他会以哪种方式出行。

假设我们比较了解小哥的个人习惯，小哥是个健身爱好者就喜欢跑步运动，这个时候我们可以猜测他更可能倾向于走路过去。当然我的隔壁小哥是个大死肥宅，这个时候我们猜测他更可能倾向于坐车，连骑自行车的可能性都不大。

这个情景中隔壁小哥的交通工具选择与花费时间不再相关。因为我们是在结果发生前就开始猜的，根据历史规律确定原因\(交通方式\)的概率分布，即先验概率。

### 似然估计（知因求果）

换个情景，先考虑小哥去公园的交通方式。

假设隔壁小哥步行走路去，一般情况下小哥大概要用2个小时;假设小哥决定开车，到公园半个小时是非常可能的。

这种先定下来原因，根据原因\(出行方式\)来估计结果的概率分布即似然估计， 根据原因来统计各种可能结果的概率即似然函数。

### 状态估计

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Yzg2NDI1NGIwODY4NWQ4MmQzNDE1NGI1OWY3NjNiYzNfNTdiZDAyY2I3ZGY1MjJjZDZiMjg2ZmUzMzQ0N2Q1MWRfSUQ6NzU3NTIyNTQzOTQyNjM1MDMwMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

直接求后验分布是困难的，但是求一个状态最优估计，使得在该状态下后验概率最大化，是可行的。

最大后验估计（MAP）：求一个状态最优估计，使得在该状态下后验概率最大化

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NWVjZDFmNTdmMzExNDNmNDlmODc3OGM5MGE3YTQwOTVfNjJmNjIzZDdhMmI2MjdhNGNkOGYxZTRkYjAyOGQxZGRfSUQ6NzU3NTIyNTQzOTEzNjkyNjkyMF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

最大似然估计（MLE）：在哪种状态下，最容易产生当前的观测

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTM5NjMwMDM0Mzk4ZTQ3M2RiNTVmMjk1MmQxODdmZWVfYmI3MTU3OGE3ODBkZDhmYmI4MzIyNWMxOWI1ZTUwMWZfSUQ6NzU3NTIyNTQzNzY2MDQ5ODg4OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## 最小二乘的引出

由观测方程和噪声服从高斯分布可知

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NTVlMDAyZTE4NmMxZTM4OTg4YjQ1NzcyMGZlNGU3N2ZfNDRjMzJhYWZkNGM0OTVlNjJkOTJiOWUyY2IwNDI1YmNfSUQ6NzU3NTIyNTQzOTYwMjM5NjM4OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

高维的高斯分布：

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YjM2NjJkNDUyMTUxNGFlNGNiYTliZTc3NzRkMmY0YjRfMTNhMjI2YWU1N2YyN2NjODhiNGNlZjhjNjIyNzYwMWVfSUQ6NzU3NTIyNTQ0MjY4NTM5MDAyNV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

则概率密度为

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTg4YTRjZjg5YTU1MzE4NzEyMTc0MTQ3NTExNmJmNjVfNDE5YWM5NGI0YTYzMjlhZDYzYjBlYmJhZDc0MTA5M2VfSUQ6NzU3NTIyNTQzNTYzNDc4MTM5NV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

对其取负对数，则有

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTdlMDBiZTJlMzc0YzJiZTEwZTM5OTdhZTZlNDkzMWFfMjBmZDcyODBiOWRhMmUwYjYxOWM1YTJjOTAxNDUyYjdfSUQ6NzU3NTIyNTQzODM4NjE5NTY2MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

其中，左边两项是固定的值，也就是说当最小化 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzZiN2U3NWI2ZDBkZDAzOTJiYWI3MjE1NzJhMmQ1Y2FfOTRhNzVkMzkzNmFmMTFlZTI3YTI1YzQ4MmFhYTVkZmJfSUQ6NzU3NTIyNTQzODY3NTY1MTc4OV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

时，只与最右侧项有关

这样原问题的最大化，相当于负对数的最小化，进一步，最小化右侧二次型项，就得到了对状态的最大似然估计，也就是最小二乘的问题

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmU0MTlkOWNjMTA0Mzc5NDQ1YmQyY2VmOGMyNThhNzVfYzcyYzhlZmM4NWMyNjVhNTNmNDIxN2RjYzQzMjJjMDhfSUQ6NzU3NTIyNTQzODQ4MjYzMTkwOF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

考虑实际中批量处理数据时，由于各个时刻的输入和观测相互独立，数学语言就是

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGM4NjlmNmNjMTM0YTZkN2RjM2MzZWIzNmI4ZDJlN2JfZGM1MzQwMzg4NjgzZTJiODBlMmZjNzU2OTMzMTJlNmJfSUQ6NzU3NTIyNTQ0MjgwMjYwMTE1NV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

对数据的最小化估计误差也就是最大似然估计

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTI3M2FmNWJlNmM1YTRiZTJiNTE0MzUyNTljMjk4MTZfNjViNjY0NjlmOTFhOTVlMWM0MGYxYjk5OWIyNWJlY2NfSUQ6NzU3NTIyNTQ0MDYzMDEzMTkyNl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

由SLAM的状态估计，得到了最小二乘问题：最优解等价于状态的最小二乘问题 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjhkNGViYWRjYjNhYjZiZDc5M2JiZjcwMzQzMGFiMWRfODBmYmZiMzljNzg4OWY4YzY5ZTMxOWYwNWEyZDZkMjlfSUQ6NzU3NTIyNTQzODczODMzNjczM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

，其由许多个误差的平方和组成。虽然整体维度较高，但是每个项很简单，仅与一两个状态变量有关\(比如运动误差只与 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjMzNGNlNzk2YTU2NzY3Nzg2MzgxM2M5NDBjZTJmZTFfNjJmMjYzZmU2MzI2NWUyMTkxODAwYTRhMTQzYTA1NWRfSUQ6NzU3NTIyNTQ0MjEyMzI3MTM5Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

有关,观测误差只与 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODkyZWJlNjIwZjVmOTA3MjRmNjc2MGE1MGUzZDc2NDFfZjIzMDBhN2U4NzAxZDBmNzFhYjA3NjEyZDhhYTE4MmZfSUQ6NzU3NTIyNTQzNDgwMDAzMjk2OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

有关\)

如果用李代数表达位姿，则是无约束优化问题，那么如何求解此类的非线性最小二乘问题？

## 非线性最小二乘法

先考虑最简单的问题：

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NmNjMDI5ZTllYTAzOTBmNmIzMTBlYjYwN2Q1M2RiNTVfMDIyZDE4MGM0NjY3MjQxOGQ1YTlkZDIzODFiODY1ODFfSUQ6NzU3NTIyNTQzNzY5OTAzNDMzM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

其中 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjAwMTU3Nzg5MmI1NjliMmM1OWQ3Njc2NGNmMDI2NDhfMTY0NmI4ZWZlNzcwZDEzNDYzMTAxNWU2ZjQyOTMxYTFfSUQ6NzU3NTIyNTQzOTYzNTk4MzU1NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

是任意的标量非线性函数，

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGRlYjA1YTllNTMyN2Y2ODQ2ZmViNTQ3ODFmNTA1ZjBfNzg4Nzc1YzllZDFkZThkYjQxYTJiNWM0ZThhZjZhOTZfSUQ6NzU3NTIyNTQ0MTUyNzU0OTExOF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

可以是向量

当 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MWNiNThkNWI1ZDgzMzg0YjVjNWVhZTI4NDBlMTRjODJfOWQ1MmE2ZWNkZGE4OTJmM2M5YmZlZjdkMzVjYjQ5M2RfSUQ6NzU3NTIyNTQzODc1NTE0Njk3Ml8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

很简单的时候，令目标函数的导数为零，就可以求解最小值，从一系列极值中筛选

当 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzRkNDNkNWFmNjE2ZGVmODc5NGI3YjY1ZmJmNDUyYmFfMWNlYzQyZDdkZWRlNmI3M2ZmYmI5OGNkZjhmN2RjY2NfSUQ6NzU3NTIyNTQzOTEzNjk0MzMwNF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

很复杂的时候，就难以使用这种方法求解，所以需要一种迭代的方法求解，从一个初始值出发，不断优化当前的优化变量，使得目标函数下降，直到直到某个时刻增量非常小，无法使函数下降，则算法收敛，目标达到了个极小，也就完成寻找极小值的过程。但是迭代过程中的增量如何寻找呢？这就需要使用梯度法了。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWZhM2M3MGMzYTEwZjFiMGY0NDNhMjAwMzA3ZjViNGJfYzkxYThlNjBmMzg5ZDc0ZmFiNzE2YTg5MGI2MGI2ZjBfSUQ6NzU3NTIyNTQzNzk4MzM5NTAyM18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODE3MDk5NWUwNzdmMTEyZGZhNzVkZjI4MzY2NGUzZWFfYjE1YjIyNGVjMWNhMDRlZDU3MDBkMTZiMGFhYmYyNzVfSUQ6NzU3NTIyNTQzODYyMTA3NjcwMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWEyZjkyMzg1NGQ0ODllMWYzMjI0MTJhNGYxN2ZhMzZfYzczNGFkZDk4N2I0ZWFlZDA4ZmZlZmUzYmVhNDlkNzhfSUQ6NzU3NTIyNTQzODc1NTEzMDU4OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MmU5YTQ3ZjU2MWExNTA0MjI4NDE4M2I2ZWEzZjUxOTJfYmEyN2UwNzY1YjZiNWEyZjM2NGQyNDFiOWVkZDdkZTBfSUQ6NzU3NTIyNTQzOTc0OTI0NjE3M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDUyOTUzNTYwNDViMzgwYzk1MzA1NzcxNTFlMTFmMjJfYjY1MGY5NWIzMGIyOWY4NWU5MTc5ZGI1N2NjMDBlMTBfSUQ6NzU3NTIyNTQzODY3NTU1MzQ4NV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MmU1MDZhNzg2MjUzNjU5OWFlZjZmNmVlZDA3YzFkMDlfYjFhNDE4NzRmNjM5Y2RhNWUzNTNlYzU2MWNkMTM3NWVfSUQ6NzU3NTIyNTQ0MjY5MzYxNDc4OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjBhZTFkZDc0MWJiN2QwZDY4MDJiZTM1YzM4MWI1MTJfMjRiOGNkNDRkOWU4YmI3MzBjMzBhNWZiMWQ0ZmU3MDFfSUQ6NzU3NTIyNTQzNjM5ODEyODMyN18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTdmYjg0MjZjMzA5NTE5NDRjZGE0NmUxNDliMWYzYTFfNWQ2NzVlYjRkMDE4MTFjMjU3OWYwZDNiNTI0NzljNGJfSUQ6NzU3NTIyNTQzODgyNjcxMjI3MF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

# 视觉里程计

## 特征点提取与匹配概述

现在开始就是SLAM系统的重要算法，其中视觉里程计方法就是对图像进行特征提取与匹配，计算出来两帧之间相机的位姿变换等，这里会涉及特征点法的使用，主要是有如下几点需要注意

- 理解图像特征点的意义，掌握在单幅图像中提取特征点及多幅图像中匹配特征点的方法

- 理解对极几何的原理，利用对极几何的约束，恢复出图像之间的摄像机的三维运动

- 理解如何通过三角化，获得二维图像上对应的三维结构

- 理解PNP问题，及利用已知三维结构与图像的对应关系，求解摄像机的三维运动

- 理解ICP问题，及利用点云的匹配关系，求解摄像机的三维运动

经典SLAM模型中以相机位姿\-路标来描述SLAM过程，路标是三维空间中固定不变的点，可以在特定位姿下观测到，在视觉SLAM中，可利用图像特征点作为SLAM中的路标

特征点是图像当中具有代表性的部分，如轮点，较暗区域中的亮点较亮区域中的暗点等。特征点应该有如下特点，以便于快速准确的进行匹配

- 可重复性：相同的区域可以在不同的图像中找到

- 可区别性：不同的区域有不同的表达

- 高效率：同一图像中，特征点的数量应该小于像素的位置

- 本地性：特征仅与一小片图像区域有关

特征点的信息有关键点和描述子两部分，关键点指的是位置、大小、方向、评分等，描述子是特征点周围的图像信息，如：当谈论在一张图像中计算SIFT特征时，是指“提取SIFT关键点并计算SIFT描述子”。

## ORB特征

ORB特征是一种更为高效的方法，其来源于著名的VSLAM框架ORB\-SLAM，关键点是Oriented FAST\(一种改进的FAST角点\)，描述子是改进 BRIEF

### 关键点

FAST：主要检测局部像素灰度变化明显的地方\(如果一个像素与领域的像素差别较大，则更可能是角点\)

1. 在图像中选取像素 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NTZjNmEyNTk4OGMzMDM4MjAwNTFhNzdlNjk1N2I3OTVfNGFmZWE4MzI4NzE4NzcwYWM1NDYwOThlOTU2MzRhMjZfSUQ6NzU3NTIyNTQzODA3MTU0MDk1NF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. ，假设它的亮度为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Njg5YzE0YTAyNjhlZDQyN2UwMzhhMzAzMGI3ZTVmYWRfZjJiYTRlYmRmMjZkNzJhZGZjZGIxOTE1NmU0ZGMyOWZfSUQ6NzU3NTIyNTQ0MDQ3NDc5NDk1N18xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. 设置一个阈值 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDYwY2YzMjA1N2Q4MmQ2MDgwYTUzMDk4OTkyM2U4ZWVfM2U5ZDI2YTk5OWQxNDgwYzg5YzBjMTRmODhmMDg3NzhfSUQ6NzU3NTIyNTQzOTYzNTk5OTkzOF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. （比如 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmQzZTdhZmJmNzdlNzNlZDRiODYyODcyODgyMWM0ODhfNjc4MzNkMzczNjcwMTQwZWI5N2I2YjYzYzMyMGYzNmJfSUQ6NzU3NTIyNTQzODU2NjQwMzI2Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 的20%）。

2. 以像素 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDFmNWMzNjhjYjkxYmMwMjkxMGYxNmFhZWFiOTIwMDZfNjAyMjE4ZjEzNjM1ZjdjMDRiMDBmMDVmZjg2ZWY3ZTdfSUQ6NzU3NTIyNTQzNzAxMDQ2Mzk2MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 为中心,选取半径为3的圆上的16个像素点。

2. 假如选取的圆上，有连续的N个点的亮度大于 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MWRhNzY3NmJjMGY5OTk4Yjc4YjRlOGEyZjdjZjdjMThfZjYyYTY4YzVkYTgxY2Y1YTgyOWMzMTQxMWFjYzBmZDdfSUQ6NzU3NTIyNTQ0Mjg2MTMyMTQxNV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. 或小于 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODEyNTIzOTFlYjkxYzU4ZWFlMDBjZGNmMGZhMTI3YWVfMjJkZGQ4NmNjYTcwZTMzNDE4N2M4Y2I1NTU2NWY2NGNfSUQ6NzU3NTIyNTQzODY3NTQzODc5N18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. ，那么像素 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWY1MWE3MjA3ZTM5OWE4ZmI0MWZkYjRiYTljNDBiMzhfOWU2ZDc1MWFkNTAzMTkxMWMwY2FjZGExZTFjMzNjN2RfSUQ6NzU3NTIyNTQzODUwNzgzMDQ4OV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. 可以被认为是特征点\(N通常取12，即为FAST\-12。其它常用的N取值为9和11，他们分别被称为 FAST\-9，FAST\-11\)。

2. 循环以上四步，对每一个像素执行相同的操作。

在FAST12中，提出一个高效的测试，来快速排除一大部分非特征点的点。该测试仅仅检查在位置1、5、9、13四个位置的像素，如果不满足至少三个角点亮度大于 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWZlNWU5OWY0MTA2MGRlM2Y3ODdkZTVlY2VkMTk2MGVfYWJmNmU0ZDQyZWIyYzBmOWExYjEzNWE0YTBhN2JlZGVfSUQ6NzU3NTIyNTQzNjM5ODI0MzAxNV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

或小于 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzliYjRlNWMxNGEzZDMxYjI3ZGI3Y2Y1YmY0ODBjYWVfNTJhMDkyNzU1ZmQ4NGM4MDg5ZjM2MmNmZTU2MGI1NTlfSUQ6NzU3NTIyNTQ0MzA3NTI2MzcwMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

，那么 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmVhNjdlNjI2ZDE1N2FkNjM1YjI1ZDMyMWMxMDMzMWVfNTQyYmY4ZWYzNzMxYzMzY2ViMjBmMDU4OTYwN2Q1YzhfSUQ6NzU3NTIyNTQzODgyMjMyMTEwMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

不可能是一个角点。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjhmNjI0NWU3YzRiNTI1MTU1NDVjNDc2YzViMzdiMjhfOTRjNWU0MGRmMGRhYjU1NDAzZDU1MDc5MGU1ODIxNGRfSUQ6NzU3NTIyNTQ0MDYzMDA4Mjc3NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

当然FAST也有缺点，原始FAST角点经常出现扎堆的现象\(分布不均匀\)。所以在第一遍检测之后，还需要用非极大值抑制，在一定区域内仅保留响应极大值的角点，避免角点集中问题。

由干FAST角点不具有方向信息且存在尺度问题，ORB添加了尺度和旋转的描述：尺度不变性通过构建图像金字塔来实现，旋转是由灰度质心法来实现。

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OWMzMDc4MDExYzRjNGJmZGM1YjhjNzg3MjE1NDgzMThfNzY4ZDcwYTM1NTgxZWM2M2Q0ZmUwMGFmYjg4MmZiNGJfSUQ6NzU3NTIyNTQzODM5ODcyOTQzOF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDBlZDBiZmI2OTE0MWQyOGIyODVlYmRiZDVlMTFhN2NfMDRiYWFmOTAzNjMxODdlYzc3OWYwYzJhNWZjYWFkNTRfSUQ6NzU3NTIyNTQzODQ1NzQ0OTY5Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

### 描述子

然后描述子部分是BRIEF，一种二进制描述子，其描述向量由许多个0和1组成，这些描述了周围128个点的亮度值，或者说对比了周围两个像素点的亮度值

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGNiMjAyNGY4OTc5MzRiYmNmMTZkNjY0NmI1YjBiMTVfY2Y0NDllMmUyM2JiMDRlOTFkY2Q0ZjIzZTc1MTFmOGJfSUQ6NzU3NTIyNTQzNzY5OTE2NTQwNV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

实际上操作方法如下多种，只不过工程上经常性使用第二种方法：

1. 在图像块内平均采样

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YjJmZTMxMjgzNjgwYmNjZjYxZDRkZWE4NmM0YTcxMWNfM2FhZjQxYTc4MmJmZjczYjVjM2Q2NzVmM2JiMWE1ZDJfSUQ6NzU3NTIyNTQzODI2MDIzNTQ4MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDU0N2U3YmU1NzE3ODBkM2QwODg4M2IzYTRkZjBiMzVfZDBiMjFkYjQ5YWIxNjgyYTI1YTI1ZmI2ZTk3MjIxZGZfSUQ6NzU3NTIyNTQ0MjQwNDIwNzgxMF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 都符合 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2M1MDlmZDVhMDgzNjNmY2NmYTg0MWRlZmU1ZGM3NjZfOWQ4MGVmNDkxNzNhZTJjNzllYmY1NmExMGNjNjhhZDJfSUQ6NzU3NTIyNTQ0MTUyNzU2NTUwMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 的高斯分布，或者也有这种情况：

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjgwMzQ3MmRiZjg1YmRkYjI2ZjJkZTU5NmVkYjQxMjFfMjMxODJiZTQ3NTcxMzYyZWMyNjY3ZTlhNWUyYjc3YTlfSUQ6NzU3NTIyNTQ0MjgwMjYxNzUzOV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 符合 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDc2Yzc4OWZiYjhkODY3OTE1ZjVhM2FhZjNlM2M5NmNfMWQ0MjhmZDMwMTI2OGUyNDViZjQxMjI4ZThiODg1NGJfSUQ6NzU3NTIyNTQ0MjM2MjIzMjAwN18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 的高斯分布，而 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWE5MGM2OTRiYmVhNDcyZjA1YmUwNzc3NzVjMDAwOGRfOGFjYmJkYTlmOGU2MDFhOGU5OTM3MzFkNzFiNTE3ZjJfSUQ6NzU3NTIyNTQ0MjQzODU0ODY3MV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. 符合 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmRmYTZmMGUxMzgyZTcxNzVkZmUyOTUwNjBkOGVkM2NfZjMxOTYyNjJhZmNjYzQ3YTE4YWJkY2ZmOGI2MTNiYjJfSUQ6NzU3NTIyNTQzNTMxMTYzOTc1OF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 的高斯分布

2. 在空间量化极坐标下的离散位置随机采样

3. 把 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NzIxZTQ1ODNjZTRmOTNkMGRjZDc3YWE4NDlkMTJkMTBfNzM5YzEyNzQ0NjljYjY3ZWU3YWRlZmMzOTFhMjJkZGZfSUQ6NzU3NTIyNTQzODk0NDAyMTcyNV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 固定为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2ViY2UwYzRhOWUyMjMzNjYyYzkzZjUxMjc0NjhmNzdfZDU1ZDkyMGIzNDQ1M2JiYjVlZjVmNzZiMGRiODFiMmFfSUQ6NzU3NTIyNTQzODEwOTI1NjkyNV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. ，

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGZjN2IwYjYxYThhMzI0MjA3Y2JkODY1NjA5OTRhYjlfMzY4ZDA1NGQ1OWNmNDJmOTU0Njc1NTQ1M2U2MmFjZGNfSUQ6NzU3NTIyNTQzNzk4MzUwOTcxMV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 在周围平均采样（实际上通过预先定义好的位置采用）

总结：描述子:BRIEF总结

1. 优点：BRIEF使用了随机选点的比较，速度比较快，而且由于使用了二进制表达，存储起来也十分方便。

2. 缺点：原始的BRIEF描述子不具有旋转不变性，在图像发生旋转时容易走失。而ORB在FAST特征点提取阶段计算了关键点的方向，计算了旋转之后的“BRIEF”特征使ORB的描述子具有较好的旋转不变性。

3. 注意: BRIEF是一种二进制描述，需要用汉明距离度量

### ORB流程

ORB的流程如下图所示，

1. 根据改进特征点法计算出来特征关键点（这里是篮球的中心点）为圆心（几何质心），然后以 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MWQzYjBiNzNkODMwZDYzY2QwN2IxZTI2MzMwYjkxNDJfOGIzNTI0NGVkMDg1NjlhN2M4YjU4NDgyYTk3ODkyNGJfSUQ6NzU3NTIyNTQ0MDk2OTc1NTYwMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 为半径作圆

2. 计算质心，得到特征点方向（绿色箭头）

3. 选取某一个模式下的pq点对，书上代码是128对，这里为了便于表示只选取2对

4. 根据特征点方向，将其旋转到与 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MGY4OTEzYjJhMjJiYWRiNDlmNzk1OWQ4MGQ5MDRhYmNfMDI1NTA2OTY0MTYyMjMxOGZlOTAxODgwMjBmYjA0ZThfSUQ6NzU3NTIyNTQzNjAyMDU5MTU4MF8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. 轴平行，并且对所有的pq点对进行旋转（这里利用了特征点的方向性和旋转不变性），这样，这一帧的特征点就可以跟上一帧的特征点匹配上

2. 按照规则对比所有的pq点对的光照强度或者灰度值

3. 按照顺序排列描述子

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjA0ZTVjNTM5YjdlNGE0MTU1ZDYwZGYxN2RkMTI4OWRfZDUxNWQwOGQ3NmI1OTEyYzBhN2ZlYmJhZmM5NWFiNjJfSUQ6NzU3NTIyNTQzODY3NTU2OTg2OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## 特征匹配

特征匹配解决了SLAM中的数据关联问题，即确定当前看到的路标与之前看到的路标之间的对应关系。

通过对图像与图像或者图像与地图之间的描述子进行准确匹配，可以为后续的姿态估计、优化等操作减轻大量负担；然而，由于图像特征的局部特性，误匹配的情况存在。

考虑两个时刻的图像：

1. 在图像 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjdlYWJiMDA3YTE3ODE5M2RhNzc0ZTRhNDk2YjgzODFfYmQ2MzE4M2Y2NjU4MGJjNDdkNzY3OTAxMjBjYjZlNjVfSUQ6NzU3NTIyNTQzOTYwMjQyOTE1Nl8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

1. 中提取到特征点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2Y3MmU3MWQ0MGNmZDE3M2FmMDk4NzZhYmRiZGYwN2RfYTAzMzM5ZmQ3MzRmMDA2NDQ2NzdmMDY5NGE2ZjkzZTNfSUQ6NzU3NTIyNTQzODE0NzA1NDgxMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. ；在图像 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjY2YzJkNTAzZmU0MzExMzQyNWJkMTdhOGY2MTNkYmNfMmY0YzIzODY1ZjA0YWZkZGE1MmEyYzMxN2EyOTUzNWNfSUQ6NzU3NTIyNTQzNDY0OTAwNTI1Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 中提取到特征点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWQ4M2I1NTFmMjUxY2E1ZDZkMzIwZjk1NmY2ZWUxMzhfNTU2NDFmYThjNDNhNDEyZWIwMDA3OWM3ZGFmMjczZDZfSUQ6NzU3NTIyNTQ0MTUxMDc3MTkxN18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 暴力匹配：对每一个特征点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTY3M2YxMzQwM2ZhNGJjOGQ2MGMxZGM5YjRkN2FjYzdfOTkwMzBhMzFkMzM5ZjJiNTc1OTdkOGYxZDVlYjFkZTVfSUQ6NzU3NTIyNTQzODEwOTE5MTM4OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 与所有的 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YjI2MDE3ZTNlNTIyNjZkNDRjZDNhNTA2MWY4NTc0MTJfMmU1ZjI4NDZkZTBlZDIzMWY2MWI1Y2NkNjUwZWExMDZfSUQ6NzU3NTIyNTQzNzIyNDM0MDY4M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

1. 测量描述子的距离，然后排序，取最近的一个作为匹配点

2. 当特征点数量很大时，暴力匹配的运算量会变得很大，选用一些改进算法，如：快速近似最近邻\(FLANN\)算法

在进行了特征点匹配方法之后，就可以进一步计算相机的运动了

## 计算相机运动

计算有三种情况，也就是在输入数据是二维图像还是三维点云的情况下，如何进行计算

- 如果只有两个单目图像，得到2D\-2D间的关系\(对极几何\)

- 得到一些3D点和2维图像投影，即得到3D\-2D间的关系\(PNP\)

- 当相机为双目、RGB\-D，或者通过某种方法得到距离信息得到3D\-3D间的关系\(ICP\)

### 单目图像——对极几何

已知的信息如下：

- 三维空间点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDI3ZjY5NzNiYTgzNGU1MjQxNWU1OTI0NzU5ZmQ2ZjZfMjQxOTQ1ZDc2M2QxNDYzZGE3YTIwNWJkNzM5Zjk4N2NfSUQ6NzU3NTIyNTQzNzIyNDI0MjM3OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- 在像素坐标平面的投影点为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Nzk1MzEyMDk1Y2VlYzMxMGZhNTJjNjM2ODk4MTU3ZmNfMzBjNjcwNWUyMzgxZjljYmY1MDFhZjZkZDkzNDU2NjRfSUQ6NzU3NTIyNTQzNTgyNzU4ODMwNl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- 和 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTA2Yjk3NmEyYzFiNmUyNWJmMzBmY2Y4NGQxMzIzMjJfOWFhNTM2ZWJkODQwMThiNGQyYmRmODU2MjdhYWEzYjdfSUQ6NzU3NTIyNTQ0MjEyMzMwNDE2NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- （像素坐标）；

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGUwNTY2NWM3OTE4MjEwZDkxM2ZlYzRmZDA5NGQ1NzFfNDZkYTM0OGI3MWYyYTkzZGMwZDcxNGYxNzY1MTljY2ZfSUQ6NzU3NTIyNTQ0MjY4NTM3MzY0MV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- 的一个特征点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjdiMmFjMDI0YmIxMWEyMjRlZjVhYjZiYzYyMjhiMzNfZTQ2NTI4YTE5NGI1MjUyMzRmM2RhNjNkNGUzODAzYmVfSUQ6NzU3NTIyNTQzNzk4MzQ3Njk0M18xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- 在 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjY5ZmI1M2FlYzAzNzI5NDFmZDVjYjJkODBiYzhkNjJfMjgxMzlkYTA0ZGM0ZTk2ZWM0MWY3YjRhMjEzMmE5NGNfSUQ6NzU3NTIyNTQzODMzOTk2MDAwMl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- 中对应着特征点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGJkMjc2MjZkYzlkMTQ1OGNlNmEyMWU5NjNiOWI5YThfZjRjOGQ0MWFkYzA0NWFkZGFmNjNhMWVmZTA4MGQ4OGZfSUQ6NzU3NTIyNTQ0MDk2OTgwNDc1NF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

- \(匹配正确的情况下\)；相机内参

想求：第一帧到第二帧的相机运动为 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGRmODM5NmU5ZmVmNzkzOGYyN2FiZDhmYmQ0OGU1NTRfZjNmODI2ODhkMjU4NGJkMjA3YmM1NDUyZmZkOWViNjhfSUQ6NzU3NTIyNTQzNzk5MTg2NTU3Ml8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjFiOGFhY2ExYTFkNTVhODQzMGM0MTFmYmJhYzM0YWJfMGY3YjY5NTMyZTkyNTJkZDBiM2I0MTQ1NTFjOWEyNWNfSUQ6NzU3NTIyNTQzODMzOTk3NjM4Nl8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

上图中，基线是 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTUwYmE4YmQwMzkyY2E3ZjVmYjNhMjM5MTRmM2Y2NzlfNzMxZTAzNmMyZmZjNmExZTY2YmYwMTJkMWJlNTY5NmRfSUQ6NzU3NTIyNTQzOTU4NTY1MTY3NV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

，极点是基线和像平面的交点 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjliYjZhMmY3YTk1OTA4NDY2NGMwMjVmOTA1MDhhZDlfYTY0NTZlZGU1YTk0ODhiMjkwMDA3ODVmYWIzOWNhMjdfSUQ6NzU3NTIyNTQzOTkzMzc0NjM2Nl8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

，极平面是包含基线的平面（或者说，是 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjNkYWRjOTdkNDBmZmQ2MTYzMGU5Nzg5YmFlMjhlZWFfODE5NzdmZDM1NGQ4MzhiOWIwM2I2NmE2YjFmY2E4MjBfSUQ6NzU3NTIyNTQzODYyMDk3ODM5N18xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

的平面，三个点确定一个平面），极线就是极平面与图像平面的交线

单张图像只能确定 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjhiYWMxZjE0YjY5ZjExZjIzMWRmOGZhNGM3NmRjNTRfZTcyYWJmMmIxYzU1Y2Q2YjgzZTNjYTkxOTE0NTMyN2ZfSUQ6NzU3NTIyNTQzODI2MDM1MDE2OV8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

在某个直线上，无法确定深度信息，但是两张图像就可以在有约束的情况下恢复

# Docker镜像

为方便开发，本人自己制作了一个Dockerfile文件用于构建`slam_ros1`项目，提供文件如下，其中部署了很多主流SLAM算法，目前已部署的有

- FAST\-LIVO2

- R3Live

- FAST\-LIO（simple版本）

- Point\-LIO

\[Dockerfile\]

同时本人基于云原生方法，在国内服务器上构建了Docker镜像，大家可以直接自行拉取并运行，不必再自行构建，同时拉取速度十分有保障，拉取方法如下，同时提供镜像仓库地址：[slam\_ros1镜像仓库](https://cnb.cool/gpf2025/slam_ros1)，在该仓库的制品一栏中可以看到对应的镜像

```Shell
docker pull docker.cnb.cool/gpf2025/slam_ros1:v1
```

拉取下来之后就可以运行，关于docker的一些具体使用方法在网上有大量教程，因此在这里不在赘述，只提供一个运行指令并附详细解释，也可以根据自己的具体情况对指令进行修改

```Bash
docker run -it --rm \
    --memory=16g \
    --memory-swap=20g \
    --cpus=8 \
    -e "DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v "$(pwd)/c013_graco:/root/c013_graco" \
    --gpus all \
    -e "QT_X11_NO_MITSHM=1"  \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    --network=host  \
    docker.cnb.cool/gpf2025/slam_ros1:v1
```

- `-e "DISPLAY=$DISPLAY"`可视化用

- `-v "/tmp/.X11-unix:/tmp/.X11-unix:rw"`可视化用

- `-v "$HOME/.Xauthority:/root/.Xauthority:rw"`可视化用

- `-v "$(pwd)/outdoor.bag:/root/slam_ws/rosbag.bag"`将本地数据包挂载到镜像中使用，以冒号`:`作为区分，冒号之前的是本地文件路径，冒号后的是容器中文件挂载路径

- `--gpus all`使用主机GPU，如果主机无GPU可忽略

- `-e "QT_X11_NO_MITSHM=1"`可视化用

- `-e "NVIDIA_DRIVER_CAPABILITIES=all"`映射计算库（Compute）和绘图（Graphics/Display）进容器，这对 Rviz 正常工作至关重要，没有这一项的话Rviz会卡顿

- `docker.cnb.cool/gpf2025/slam_ros1:v1`要运行的镜像名

Cloud\_regi因为Gazebo仿真器需要GPU支持才可以流畅运行，所以建议主机配有英伟达显卡并装好驱动，如果没有则会导致运行该镜像的Gazebo时非常卡顿

如果是虚拟机中运行该镜像，则将`--gpus all`去掉，否则可能报错，因为虚拟机无法使用主机的显卡

另注意，在启动镜像之前要在主机终端中输入指令：`xhost + ``localhost`，这个指令可以让容器中的GUI界面显示出来，如果不输入这个指令，容器在运行Rviz的时候可能就会报错并且无法显示

如果本地下载了rosbag，但是想在镜像中使用的话，就需要设置挂载，将本地文件挂载到镜像中，使其可以读取和使用

然后需要安装NVIDIA Container Toolkit，否则Docker中无法使用GPU进行计算，或者说无法访问主机GPU

```Bash
# 添加密钥和仓库
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
# 安装
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
# 更新Docker配置
# 生成配置文件
sudo nvidia-ctk runtime configure --runtime=docker

# 重启Docker服务
sudo systemctl restart docker
```



# 导航小车仿真（ROS1版本）

直接运行官方的镜像，选择的依然是ROS1 Noetic，注意一下官方镜像直接进去的话是某个目录而非根目录，因此需要先输入一个`cd`切换到正常根目录

```Bash
docker run -it --rm \
    -e "DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    --gpus all \
    -e "QT_X11_NO_MITSHM=1"  \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    --network=host  \
    ros:noetic-perception-focal
```

并且要注意一下，刚刚进去的镜像中不存在Git等，因此需要单独安装

```C++
apt update && apt install git wget
```

之后就是编译安装一系列的依赖，但是方法上需要注意一下，Sophus的编译安装得稍加处理

```C++
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
make install

git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
perl -0777 -pi -e 's/SO2::SO2\(\)\s*\{\s*unit_complex_\.real\(\)\s*=\s*1\.\s*;\s*unit_complex_\.imag\(\)\s*=\s*0\.\s*;\s*\}/SO2::SO2()\n{\n  unit_complex_.real(1.);\n  unit_complex_.imag(0.);\n}/s' sophus/so2.cpp
# 方法一
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
cmake --install build
```

当然，Sophus的安装方法似乎有点迷，因为我曾碰到过明明编译安装成功但是最终编译整个工作空间的时候发现找不到Sophus，因此我给出了两种编译安装的方法，在一套方法无法成功的时候可以切换另一套，一般都可以解决

```C++
# 方法二
mkdir build && cd build
cmake .. && make
make install
```

然后开始构建一系列的代码，主要是包含Gazebo的仿真项目、Mid360激光雷达的插件、FAST\-LIO算法的运行等等

```C++
mkdir -p slam_ws/src && cd slam_ws/src
git clone https://github.com/blackcoffeerobotics/bcr_bot.git
cd bcr_bot && git switch ros1 && cd ..
git clone https://github.com/zlwang7/S-FAST_LIO.git
git clone https://github.com/Livox-SDK/livox_ros_driver.git
git clone https://github.com/fratopa/Mid360_simulation_plugin.git
```

PS：如果容器中无法正常下载，那么可以考虑在主机中下载然后传入进去



然后刷新环境变量之后就可以开始运行了，首先是第一个指令，可以验证环境是否正常

```C++
roslaunch bcr_bot gazebo.launch
```

如果出现以下报错，就说明Gazebo没有成功显示，这主要是因为容器中的可视化界面想显示在主机上需要权限的，但是主机中并没有加入这个权限

\[gazebo\_gui\-3\] process has died \[pid 29062, exit code 134, cmd /opt/ros/noetic/lib/gazebo\_ros/gzclient \_\_name:=gazebo\_gui \_\_log:=/root/\.ros/log/ea21918a\-f5ae\-11f0\-bc4c\-a0ad9fd115f2/gazebo\_gui\-3\.log\]\.

log file: /root/\.ros/log/ea21918a\-f5ae\-11f0\-bc4c\-a0ad9fd115f2/gazebo\_gui\-3\*\.log

所以需要单独开启一个主机的终端，输入指令`xhost +`

如果一切正常，应该会显示如下的界面，这就是成功运行了仿真，下一步就是进行各种修改

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjhkYmE2NDgwYzRkZmI4YjRlN2YwY2I0MmY5OGY1ODFfNjNiN2U0YjhhODE1YWQ2YWFmZDJiYjQ4ZjZkNTIwNDFfSUQ6NzU5NzI3ODYzNTI3MTgwMjA0NV8xNzgxNjIzNDE4OjE3ODE3MDk4MThfVjM)

## 添加Mid360插件仿真

目前Mid360及其他激光雷达是非常主流的雷达，也得益于FAST\-LIO等算法的推广，因此在这里我们使用Mid360进行导航等，但是原始的Gazebo中并没有Mid系列的插件，就需要单独进行处理，所幸Github上有一个仿真插件，并且可以用于Noetic上，也就是`Mid360_simulation_plugin`仓库，只需要将其下载到工作空间中即可（之前已经下载了）

然后查看BCR Bot，因为已经切换到了ROS1分支，因此可以看到其中的`gazebo.launch`文件就是开启仿真的接口，深入挖掘可以发现其分为三部分内容：参数声明、仿真环境搭建与创建机器人，因此主要的修改就是针对机器人部分，这一部分的文件在`bcr_bot_spawn.launch`中，而第一部分的参数声明可以看到，其中定义了很多的使能选项，根据命名可以很容易理解就是是否开启2D雷达、单目相机、是否发布轮式里程计等，为了防止后面对三维雷达的干扰，因此统一设置为false

在`bcr_bot_spawn.launch`中可以看到，解析的目标文件是`urdf/bcr_bot.xacro`，这是机器人的最终主文件，将其打开可以发现，里面定义了机器人主要功能（也就是各种传感器），至于机器人的机身等定义在其他文件中，因此我们选择在其中添加自定义的Mid360传感器，效仿其他传感器的声明方式作如下声明

```XML
<!-- ===================== -->
    <!-- MID-360 + IMU (enable together) -->
    <!-- ===================== -->

    <xacro:if value="$(arg mid360_enabled)">

    <!-- MID-360 main body link -->

    <link name="mid360">
        <collision>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <!-- 外形仅用于碰撞/可视化，可自行改尺寸 -->
        <geometry>
            <cylinder length="0.06" radius="0.055"/>
        </geometry>
        </collision>

        <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <cylinder length="0.06" radius="0.055"/>
        </geometry>
        <material name="aluminium"/>
        </visual>

        <inertial>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <mass value="0.1"/>
        <xacro:cylinder_inertia m="0.1" r="0.055" h="0.06"/>
        </inertial>
    </link>

    <joint name="mid360_joint" type="fixed">
        <parent link="base_link"/>
        <child link="mid360"/>
        <!-- 安装高度按你车体改 -->
        <origin xyz="0 0 0.2" rpy="0 0 0" />
    </joint>

    <!-- Gazebo: MID-360 lidar sensor + plugin -->

    <gazebo reference="mid360">
        <material>Gazebo/White</material>

        <sensor type="ray" name="laser_livox">
        <!-- 传感器相对 mid360 link 的位姿 -->
        <pose>0 0 0.05 0 0 0</pose>
        <visualize>true</visualize>
        <always_on>true</always_on>
        <update_rate>10</update_rate>

        <plugin name="gazebo_ros_laser_controller" filename="liblivox_laser_simulation.so">
            <ray>
            <scan>
                <horizontal>
                <samples>100</samples>
                <resolution>1</resolution>
                <min_angle>-3.1415926535897931</min_angle>
                <max_angle> 3.1415926535897931</max_angle>
                </horizontal>
                <vertical>
                <samples>50</samples>
                <resolution>1</resolution>
                <min_angle>-3.1415926535897931</min_angle>
                <max_angle> 3.1415926535897931</max_angle>
                </vertical>
            </scan>

            <range>
                <min>0.1</min>
                <max>40</max>
                <resolution>1</resolution>
            </range>

            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.03</stddev>
            </noise>
            </ray>

            <visualize>false</visualize>
            <samples>20000</samples>
            <downsample>1</downsample>

            <csv_file_name>mid360-real-centr.csv</csv_file_name>
            <!-- 2: sensor_msgs/PointCloud2 | 3: livox_ros_driver/CustomMsg -->
            <publish_pointcloud_type>2</publish_pointcloud_type>

            <!-- 这里沿用你之前的“绝对 topic”风格 -->
            <ros_topic>/livox/lidar</ros_topic>

            <!-- 建议用雷达自身 frame -->
            <frameName>mid360</frameName>
        </plugin>
        </sensor>
    </gazebo>

    <!-- Gazebo: IMU sensor + GazeboRosImuSensor plugin -->

    <gazebo reference="mid360">
        <material>Gazebo/White</material>

        <sensor type="imu" name="mid360_imu">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>200</update_rate>

        <plugin name="mid360_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
            <!-- 设为空可得到根命名空间 “/”，从而发布到 /livox/imu -->
            <robotNamespace></robotNamespace>

            <!-- 注意：不要用前导 /，插件内部会拼接命名空间 -->
            <topicName>livox/imu</topicName>

            <!-- 必填：缺失会直接失败 -->
            <frameName>mid360</frameName>

            <updateRateHZ>200</updateRateHZ>
            <gaussianNoise>0.0002</gaussianNoise>

            <!-- 可选：偏置/安装误差 -->
            <xyzOffset>0 0 0</xyzOffset>
            <rpyOffset>0 0 0</rpyOffset>
        </plugin>
        </sensor>
    </gazebo>

    </xacro:if>
```

内容很容易理解，就是在`base_link`坐标系下定义了一个mid360坐标系，然后构建了一个固定的关节，声明了位移关系（激光雷达在机身上面），然后构建一个圆柱状碰撞体，最后就是在mid360系上构建传感器插件了，包含了IMU和Mid360雷达插件，这里需要注意一下是雷达的消息类型，其中2和3表示两种类型

将上面的内容添加进去，然后在文件最顶部加入如下选项，就可以成功开启了Mid360仿真

```XML
<xacro:arg name="mid360_enabled" default="true"/>
```

最后将修改内容保存，然后重新开启仿真就可以看到成功有激光雷达的加入了，在Rviz或者其他内容中使用激光点云即可

## 坐标系构建

这里包含了一个ROS中的坐标变换问题，因为如果详细分析BCR的Xacro中的坐标系关系，或者运行仿真之后使用`rosrun rqt_tf_tree rqt_tf_tree`查看TF树，就会发现其中定义的坐标变换关系是`base_footprint->base_link->mid360`，其中根节点是`base_footprint`，原生的FAST\-LIO输出的坐标变换是`camera_init->body`，在很多SLAM框架中，**“camera\_init” 通常可以理解为系统启动时建立的局部世界坐标系（世界/里程计的起点）**：启动时第一帧（更准确说是初始化成功时刻）的位姿被当作原点与参考方向，之后滤波/前端输出的高频位姿都是相对于这个系来表达的。所以它在工程语义上**非常接近 ROS 里的 ****`odom`****（局部连续、可能漂移的里程计世界系）**，至于 `body`，**通常就是滤波器状态所在的机体系（body frame）**，往往与 IMU 坐标系一致或非常接近

但是其中有一个问题，如果直接运行仿真和FAST\-LIO，就会导致TF树的割裂——有不联通的节点或者说变成了两个TF树，这在ROS中是不被允许的，也无法作为后续导航的TF树，因此必须将一个完整的TF树构建出来，第一个思想就是，既然我仿真中构建的TF树存在mid360坐标系（雷达坐标系），FAST\-LIO中的body系实际上也是雷达坐标系，那么我将仿真TF树中的mid360坐标系改名为body系或者将FAST\-LIO中输出的body系改名为mid360系不就可以了，但是这会导致一个TF树的节点出现两个父节点，会导致TF树的断裂——具体会表现为，TF中的雷达系节点会FAST\-LIO发布的坐标变换占据，也就是说这种方法是行不通的，因为每个TF树中的节点最多有一个父节点

实际上雷达坐标系和雷达的IMU坐标系不是一个概念，这是两个坐标系，但是一般会将点云变换到IMU系下，因此在这里直接称为雷达坐标系

考虑到不论是仿真还是现实中，雷达系都是固定在车上的，也就是说雷达系因此我只需要获取一次坐标变换关系，然后求逆变换，然后使用一个静态发布节点发布即可，这样就为`base_footprint`添加了一个`body`父节点，但是需要注意的是，不能直接如此发布变换，否则会导致TF树中出现环路，这也是不允许的，因此需要专门处理。

具体的方法是，创建一个`body`系（FAST\-LIO生成）和`mid360`系（仿真模型文件中定义），这两个坐标系实际上都是雷达坐标系，相当于进行了复制，然后求出`base_footprint->mid360`的逆向变换，该变换也等价于`body->base_footprint`的变换，然后使用一个静态坐标发布者就可以进行发布，如此就可以串联起整个TF树

```XML
rosrun tf tf_echo base_footprint mid360
```

运行仿真之后，上述指令可以输出变换关系，其中有

- Translation: `t = (tx, ty, tz)`

- Rotation \(quaternion\): `q = (qx, qy, qz, qw)`

然后使用下列代码计算出逆变换的参数

```Python
import numpy as np
from math import *
tx,ty,tz = 0.0,0.0,0.25          # <-- 改成 tf_echo 打印的 translation
qx,qy,qz,qw = 0.0,0.0,0.0,1.0     # <-- 改成 tf_echo 打印的 quaternion

# quaternion inverse (unit quaternion)
qinv = np.array([-qx,-qy,-qz,qw], dtype=float)

# rotation matrix from quaternion
x,y,z,w = qinv
R = np.array([
    [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
    [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
    [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
], dtype=float)

t = np.array([tx,ty,tz], dtype=float)
tinv = -R.dot(t)

print("t_inv:", *tinv)
print("q_inv:", *qinv)
```

然后在launch文件中添加下面的四元数版本静态变换发布即可

```XML
<node pkg="tf2_ros" type="static_transform_publisher" name="tf_body_to_base_footprint"
      args="X Y Z QX QY QZ QW lio_body base_footprint" />
```

然后就可以获得下面的TF树，非常完整

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjFjMWE5ZDkyNjA2NmQwNzQ4N2ZlOWQyM2E3YTc4NDZfNzFkODJlZjVmMGJiYjY1ZGFkNTdjNTU4ZTI3YjE3OTBfSUQ6NzU5NzM4OTU1MjI0ODUwNzYxNF8xNzgxNjIzNDE5OjE3ODE3MDk4MTlfVjM)

## 导航地图构建

FAST\-LIO会输出点云地图，但是这种地图是三维的，如果想执行规划算法就必须使用三维规划算法，但是FAST\-Planner等三维算法很难直接应用于地面机器人上，因此需要进行转换，转为二维栅格地图进行规划，先安装

```XML
apt install ros-noetic-octomap-server
```

OctoMap是一个**基于八叉树（octree）的概率 3D 占据地图**框架：用“占据 / 空闲 / 未知”的概率（通常用 log\-odds 实现）来表示空间，并且支持**增量更新、动态扩展、多分辨率**与相对紧凑的内存占用。官方介绍里强调了它的 3D 建模、可更新（概率融合）、多分辨率与高效存储等特性

关键是：它**也会发布一个 2D 投影栅格**，标准话题名就是 **`/projected_map`**（`nav_msgs/OccupancyGrid`），这个投影之后的就是可以用于二维导航的地图，其中的投影机制如下，分为两步

A\. 先把 3D 点云融合为 3D Octree（占据体素）

1. **TF 变换**：把输入点云从其 `frame_id` 变换到 `octomap_server` 的 `frame_id`（你之前遇到 target\_frame 不存在，就是这里失败）。TF 不对/目标坐标系不存在时，octomap 会“收得到点云但插不进去或不发布”。（同类问题在社区里经常被归因到全局 frame 与 TF 树不匹配。） 

2. **点云预过滤**：例如按高度裁剪。`pointcloud_[min|max]_[x|y|z]` 会在插入前剔除不在范围内的点。 

3. **地面滤波（可选）**：`filter_ground` 打开后，会尝试用 PCL 的平面分割检测地面并忽略（不作为障碍插入）。

4. **射线插入**：用传感器模型（`sensor_model/max_range` 等）对 octree 进行 free/occupied 更新。 

> 一个非常典型的“为什么没地图”的原因：`pointcloud_min_z` 和 `pointcloud_max_z` 设得不合理，把点全过滤掉了，导致 octree 为空（Nothing to publish / octree is empty）。社区的示例里就明确指出了这种情况。 
> 
> 

B\. 再把 3D Octree 沿 Z 方向“压扁”为 2D OccupancyGrid

`/projected_map` 的本质是：选定一个 Z 范围（例如离地 \-0\.2 到 2\.0m），把该高度带内所有 **occupied 体素**投影到 XY 平面网格上：

- 某个 \(x,y\) 的竖直柱子里，只要出现“足够占据”的体素，就把该 2D cell 标为 occupied；

- 未被观测/不确定的保持 unknown（或在下游 costmap 里按配置处理）；

- 分辨率由 `resolution` 决定，越小越细，但 CPU/内存越吃紧。

具体的代码如下所示

```XML
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    <remap from="cloud_in" to="/cloud_registered" />

    <param name="frame_id" type="string" value="camera_init" />
    <param name="base_frame_id" type="string" value="base_footprint" />

    <param name="resolution" value="0.05" />

    <param name="occupancy_min_z" value="-0.1" />
    <param name="occupancy_max_z" value="0.5" />
    
    <param name="sensor_model/max_range" value="50.0" />

    <param name="sensor_model/hit" value="0.7" />
    <param name="sensor_model/miss" value="0.4" />
    <param name="sensor_model/min" value="0.12" />
    <param name="sensor_model/max" value="0.97" />
    
    <param name="latch" value="false" /> 
  </node>
</launch>
```

其中重要的参数解析如下：

- **`frame_id`**: 这是**建图的基准坐标系**。Octomap 会把所有时刻进来的点云，全部转换到这个坐标系下，然后把“砖块”（体素）一个个堆在这个坐标系里。因此必须设置为你的全局坐标系，或者说它必须是**绝对静止**的（相对于世界），如 FastLIO 的 `camera_init`

- **`base_frame_id`**: 是**机器人的基准坐标系**。Octomap 需要知道在每一帧数据采集时机器人在哪里，用于去除动态障碍和滤除地面

- **`cloud_in`**: Remap 到 FastLIO 输出的稠密点云话题（通常是 `/cloud_registered` 或 `/Cloud_Map`，取决于你是否开启了Dense Map

- **`occupancy_min_z`**** / ****`max_z`**: 这是物理过滤的核心，决定了多高范围内的物体会被投影到 2D 地图中。

不过有一个点需要注意一下，在很多情况下，激光雷达是不会水平放置的，会存在不同程度的倾斜，如果SLAM算法中没有初始化过程，也就是没有使用重力方向进行对齐，那么仍然使用上述的方法进行转换的话，就会导致投影得到的栅格地图是斜的，在这种情况下就需要进行处理

处理方法很简单，设置一个新的全局固定坐标系即可，也就是可以设置一个`odom`坐标系，该坐标系与`camera_init`坐标系存在一个坐标变换关系，就类似于车身与雷达一样，然后在这个坐标系下进行投影即可





# 导航小车仿真ROS2版本

ROS1毕竟已经不再更新了，因此给出ROS2版本的实现，先搞一个镜像

```Bash
docker pull osrf/ros:humble-desktop-full
```

然后运行

```SQL
docker run -it --rm \
    -e "DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    --gpus all \
    -e "QT_X11_NO_MITSHM=1"  \
    -e "NVIDIA_DRIVER_CAPABILITIES=all" \
    --network=host  \
     osrf/ros:humble-desktop-full
```

进去之后就开始创建

```Bash
cd && mkdir -p slam_ws/src
cd slam_ws/src
git clone https://github.com/blackcoffeerobotics/bcr_bot.git
git clone https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
git clone https://github.com/liangheming/FASTLIO2_ROS2.git
# 下面这些内容需要单独编译安装的
git clone https://github.com/borglab/gtsam.git
git clone https://github.com/Livox-SDK/Livox-SDK2.git
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
```

然后安装一些依赖

```Bash
apt update
apt install ros-humble-gazebo-ros-pkgs
```

第一步就是在仿真小车上添加Mid360插件，在这个仓库中，作者非常好的封装了一份插件，也就是只需要把Xacro的内容插入进去就可以使用











```Go
ros2 launch bcr_bot gazebo.launch.py \
        camera_enabled:=True \
        mid360_enabled:=True \
        stereo_camera_enabled:=False \
        position_x:=0.0 \
        position_y:=0.0 \
        orientation_yaw:=0.0 \
        odometry_source:=world \
        world_file:=small_warehouse.sdf \
        robot_namespace:="bcr_bot"
```





