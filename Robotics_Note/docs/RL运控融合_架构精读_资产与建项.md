# 架构精读（补充）：资产管线 与 从零搭项目

> 用途：本文是「强化学习运动控制」部的**教学型架构精读**素材，专攻官方文档讲不清的两件事——
> **「资产怎么来的、怎么导入新资产、怎么从头搭一个项目」**。素材全部来自 gpufree 上三框架源码的**逐文件精读**（Isaac Lab 源码 `/root/isaaclab/source` + 转换工具 `scripts/tools` + `isaaclab/sim/converters`；mjlab 1.4.0 `site-packages/mjlab`），并以官方文档联网对照。
>
> 对应章节：
> - **Ch11 资产链路**（`prac_assets.tex`，label `ch:rlmc-assets`，**已存在且很厚**）→ 本文的「资产管线」「导入新资产」两节作为**源码级补强与勘误**，不重复其已覆盖的 CoACD/V-HACD、Menagerie 六步调优等。
> - **Ch22 DIY 从零搭项目**（计划中的 `prac_diy`，label `prac_diy`／`ch:rlmc-diy`，**.tex 尚未建文件**）→ 本文的「从零搭项目」一节是其**主体骨架素材**（净新增）。
>
> 标注约定：✅=已读源码并核实；🌐=官方文档已对照；⚠️=源码与文档/常识不一致（已回写《问题与解决记录》）；❓=未实跑/存疑。
>
> 写稿纪律：本文是 md 素材，**不**直接改任何 `.tex`（避免与迁移竞写，统帅后续合并）。LaTeX 化时遵守 `/tmp/rl_authoring_rules.md`（定理类单方括号、codebox 语言白名单与标题里 `\_`、无 emoji 入正文、`\nameref` 等）。

---

## 0. 写给读者的一句话：为什么官方文档讲不清这两件事

官方文档是**技术参考（reference）**：它告诉你 `UrdfConverterCfg` 有哪些字段、`gym.register` 怎么调用，但不告诉你**「一份 CAD 图纸要经过几次格式转换、每次守恒了什么、丢失了什么、由谁补回来」**，也不告诉你**「一个新任务工程在磁盘上长什么样、哪些文件是必须手写的、注册是怎么被触发的」**。本部的差异化价值正在于：**从源码拆架构、用教学语言讲运行逻辑与组织逻辑，并在 mjlab / Isaac Lab / UniLab 三框架间对比同一概念的不同组织方式。**

两条主线贯穿全文：

1. **资产是一条「物理语义补全链」**：CAD 只有几何外形 → URDF 补上运动学树与基本惯性、但丢弃接触/执行器/求解器 → MJCF 与 USD 各自把这三类语义补回来，**但补法不同**。
2. **「从零搭项目」在两框架是同一套「分层配置 + 注册触发」模式**：一个**机器人无关的环境配置基类** + 一个**注入具体机器人资产的子类** + 一个**训练超参配置**，最后由一次**注册调用**（被「导入即注册」机制自动触发）接入训练入口。理解这套模式，比记住任何单个 API 都重要。

---

# 第一部分 · 资产管线：资产从哪来

## 1.1 三种格式与各自的「语义边界」

| 格式 | 属于谁 | 描述了什么 | **丢失了什么**（需后补） |
|---|---|---|---|
| **URDF** (`.urdf`) | ROS 通用 | 运动学树（link/joint）、基本惯性（mass/com/inertia）、visual/collision 几何引用 | 接触参数、执行器模型、求解器设置；不支持闭链/腱/site（标准 URDF） |
| **MJCF** (`.xml`) | MuJoCo / mjlab | URDF 全部 **+** 接触（`solref`/`solimp`/`condim`/`priority`）、执行器（`<actuator>`）、site/腱/相机、`<default>` 类继承 | （MuJoCo 自洽，几乎不丢；但执行器语义与 PhysX 不同） |
| **USD** (`.usd`/`.usda`) | Isaac Sim / Omniverse | 场景图 + PhysX 刚体/关节/碰撞 schema + 物理材质 + instanceable 引用 | （PhysX 自洽；执行器在 Isaac Lab 侧由 `ActuatorCfg` 补，而非 USD 内） |

**教学要点（本质洞察）**：URDF 是「最小公约数」——它能被两个仿真器都读懂，正因为它**只描述两个仿真器都同意的那部分语义**（运动学+惯性）。接触模型、执行器模型恰恰是 MuJoCo（软接触、凸优化）与 PhysX（刚性接触、TGS 迭代）**最不一致**的地方，所以 URDF 干脆不碰；这部分由各自的目标格式补全，于是「同一份 URDF 在两框架行为有别」是**结构性必然**，不是 bug。这一点 Ch11 的 `ins:rlmc-assets-contact-models` 已立，本文不重复，只强调它是「语义边界」的直接推论。

## 1.2 一个「可训练资产」的五类物理语义

无论最终落到 MJCF 还是 USD，一个能上 RL 的机器人资产都要补齐五类语义（Ch11 已用此框架，本文给出**两框架的落点对照表**作为源码级补强）：

| 语义 | URDF 是否含 | mjlab（MJCF + EntityCfg）落点 | Isaac Lab（USD + ArticulationCfg）落点 |
|---|---|---|---|
| 运动学树 link/joint | ✅ | MJCF `<body>/<joint>` | USD Xform/PhysicsJoint（转换期生成） |
| 惯性 mass/com/inertia | ✅(可缺) | MJCF `<inertial>`；缺则 `link_density` 补 | USD MassAPI；缺则转换期 `link_density` 补 |
| 接触几何 collision | ✅(引用) | MJCF `<geom class="collision">` + `CollisionCfg` 编辑器 | USD CollisionAPI + `CollisionPropertiesCfg` / 网格近似 schema |
| 执行器驱动 | ❌ | `EntityArticulationInfoCfg.actuators=(ActuatorCfg,...)` | `ArticulationCfg.actuators={name: ActuatorCfg}` |
| 求解器设置 | ❌ | `SimulationCfg`/MJCF `<option>` | `ArticulationRootPropertiesCfg`（solver iter 等） |

记住这张表：**「能把模型加载进去」与「能把模型调到可训练」的分水岭，就是后两行（执行器、求解器）有没有被正确补全。**

## 1.3 库存资产放在哪里、配置怎么指向它

### Isaac Lab：Nucleus 资产根 + `isaaclab_assets` 声明 ✅

**（a）资产根的解析逻辑**（`isaaclab/utils/assets.py`，✅逐行读过）：

```python
NUCLEUS_ASSET_ROOT_DIR = carb.settings.get_settings().get("/persistent/isaac/asset_root/cloud")
NVIDIA_NUCLEUS_DIR  = f"{NUCLEUS_ASSET_ROOT_DIR}/NVIDIA"
ISAAC_NUCLEUS_DIR   = f"{NUCLEUS_ASSET_ROOT_DIR}/Isaac"
ISAACLAB_NUCLEUS_DIR= f"{ISAAC_NUCLEUS_DIR}/IsaacLab"
```

- 库存 USD 默认**不**随仓库分发，而是托管在 **Omniverse Nucleus 云资产服务器**上；上面四个常量就是路径前缀。机器人 CFG 里写 `usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/Go1/go1.usd"` 即引用云端资产。
- `check_file_path(path)` 返回 `0/1/2`：**0=不存在、1=本地有、2=Nucleus 上有**；`retrieve_file_path()` 会在「Nucleus 有」时自动下载到本地临时目录再返回本地路径。**教学点**：这解释了「为什么第一次跑某任务会卡在下载资产」——`usd_path` 指向云端，首次按需拉取。要离线，把资产镜像到本地并改 `usd_path` 为本地绝对路径即可。

**（b）`isaaclab_assets` 是「配方库」而非「资产库」** ✅。它**不存** USD/网格，存的是一批 **`ArticulationCfg` 实例**（已配好 `usd_path`、执行器、碰撞、初始姿态），路径 `source/isaaclab_assets/isaaclab_assets/robots/*.py`，约二十余款（`anymal.py`/`unitree.py`/`cartpole.py`/`spot.py`/`humanoid.py`/`franka.py`/`g1`(在 unitree.py)…）。最小例子（cartpole，✅原文）：

```python
# isaaclab_assets/robots/cartpole.py
CARTPOLE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Classic/Cartpole/cartpole.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(...),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4,
            solver_velocity_iteration_count=0, sleep_threshold=0.005, ...),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 2.0), joint_pos={"slider_to_cart": 0.0, "cart_to_pole": 0.0}),
    actuators={
        "cart_actuator": ImplicitActuatorCfg(joint_names_expr=["slider_to_cart"],
            effort_limit_sim=400.0, stiffness=0.0, damping=10.0),
        "pole_actuator": ImplicitActuatorCfg(joint_names_expr=["cart_to_pole"],
            effort_limit_sim=400.0, stiffness=0.0, damping=0.0),
    },
)
```

**`ArticulationCfg` 的三段式结构**（贯穿所有机器人 CFG）：
- `spawn=UsdFileCfg(usd_path=..., rigid_props=..., articulation_props=..., activate_contact_sensors=...)` —— **资产从哪来 + PhysX 刚体/求解器属性**；
- `init_state=InitialStateCfg(pos=..., joint_pos={正则: 值})` —— **初始位姿/关节角**（关节名用正则批量赋值，如 `".*HAA": 0.0`）；
- `actuators={组名: ActuatorCfg(...)}` —— **执行器模型**（见 §1.5）。

### mjlab：`asset_zoo` 内置 MJCF + `EntityCfg.spec_fn` ✅

mjlab 走 MJCF/MuJoCo 路线，库存资产**随包分发**在 `mjlab/asset_zoo/robots/<robot>/`：

```
asset_zoo/robots/unitree_g1/xmls/g1.xml          (+ xmls/assets/*.STL 网格)
asset_zoo/robots/unitree_go1/xmls/go1.xml        (+ assets/)
asset_zoo/robots/i2rt_yam/xmls/yam.xml           (+ assets/)
```
（✅注意：只有 **G1 / Go1 / YAM**，**无 Go2**——与任务列表一致；`asset_zoo/robots/__init__.py` 导出 `get_<robot>_robot_cfg()` 与 `<ROBOT>_ACTION_SCALE`。）

**关键差异（教学对比点）**：Isaac Lab 用**字符串路径** `usd_path` 指向资产；mjlab 用一个**可调用对象 `spec_fn`** 指向资产——它返回一个 `mujoco.MjSpec`（可编程的模型规格对象），约定写法是 `lambda: mujoco.MjSpec.from_file(str(XML_PATH))`。✅G1 的规范声明（`asset_zoo/robots/unitree_g1/g1_constants.py`）：

```python
from mjlab import MJLAB_SRC_PATH            # = Path(mjlab/__init__.py).parent
G1_XML = MJLAB_SRC_PATH/"asset_zoo"/"robots"/"unitree_g1"/"xmls"/"g1.xml"
def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(G1_XML))     # ← 这就是 spec_fn

def get_g1_robot_cfg() -> EntityCfg:                # 每次调用返回全新实例（防共享可变状态）
    return EntityCfg(
        init_state=KNEES_BENT_KEYFRAME,             # InitialStateCfg
        collisions=(FULL_COLLISION,),               # CollisionCfg 编辑器（见 §1.6）
        spec_fn=get_spec,
        articulation=G1_ARTICULATION,               # EntityArticulationInfoCfg(actuators=(...), soft_joint_pos_limit_factor=0.9)
    )
```

**`EntityCfg` 的结构**（`mjlab/entity/entity.py`，✅）：

```python
@dataclass
class EntityCfg:
    init_state: InitialStateCfg            # pos/rot(wxyz)/lin_vel/ang_vel + joint_pos/joint_vel (正则→值)
    spec_fn: Callable[[], mujoco.MjSpec]   # ← 资产来源：返回 MjSpec 的零参函数
    articulation: EntityArticulationInfoCfg | None  # actuators + soft_joint_pos_limit_factor
    # 一组「spec 编辑器」，在 spec 加载后依次施加：
    lights / cameras / textures / materials / collisions: tuple[...Cfg, ...]
    sort_actuators: bool = False
```

`EntityArticulationInfoCfg`（✅）只装两样：`actuators: tuple[ActuatorCfg,...]` 与 `soft_joint_pos_limit_factor: float`。

## 1.4 模型是「编译」出来的，不是运行时读 XML/USD

这是一个常被忽略、但对理解两框架都很关键的点。

- **mjlab**：`Entity.compile()` ✅就是 `return self.spec.compile()`，即 `mujoco.MjSpec.compile() → MjModel`。完整资产流水线是：
  **XML 文件 → `MjSpec.from_file` → spec 编辑器（collisions/actuators 等）→ `spec.compile()` → `MjModel` → 包进 `mujoco_warp`（GPU 批量）**。
  `Entity.__init__` 顺序（✅`entity.py:155–166`）：`_build_spec()`（调 `spec_fn`，浮动基座可自动包一层 mocap）→ `_identify_joints()` → `_apply_spec_editors()`（灯光/相机/碰撞）→ `_add_actuators()` → `_add_initial_state_keyframe()`。
  场景把所有 entity 的 spec **合并成一个 spec 再 compile 一次**。`Entity.write_xml(path)` 可把（编辑后的）spec 序列化回 XML 供检查（`spec.to_xml()`）。
  **教学价值**：mjlab 的资产是「**可编程的**」——你可以在 Python 里对 `MjSpec` 做结构化修改（加灯光、改碰撞、换摩擦），再 compile，无需手改 XML。这是它相对「死的 USD 文件」的灵活点。

- **Isaac Lab**：转换期由 URDF/MJCF importer 生成 USD 文件落盘，运行时 `UsdFileCfg.usd_path` 把 USD **加载进 stage**，再由 PhysX 解析成可仿真的 articulation；执行器模型则**不在 USD 内**，而在加载后由 `ArticulationCfg.actuators` 的 `ActuatorCfg` 在 Isaac Lab 侧补上（见 §1.5）。

## 1.5 执行器（驱动）怎么挂上去——两框架对照

执行器是 URDF **没有**、必须后补的语义，也是两框架差异最大处之一。

### Isaac Lab：隐式 vs 显式执行器模型 ✅

`isaaclab/actuators/` 提供一套执行器模型类（✅`actuator_pd_cfg.py` / `actuator_net_cfg.py` / `actuator_base_cfg.py`）：

| 类 | 类型 | 力矩在哪算 | 用途 |
|---|---|---|---|
| `ImplicitActuatorCfg` | **隐式** | **PhysX 内部**算 PD | 默认、最快；stiffness/damping 直接进物理引擎 |
| `IdealPDActuatorCfg` | **显式** | **执行器模型**算 PD，再下发力矩 | 需要在模型层裁剪/加工力矩时 |
| `DCMotorCfg`(继承 IdealPD) | 显式 | 加电机**饱和**（`saturation_effort`）转矩-转速曲线 | 直流电机真实限幅 |
| `DelayedPDActuatorCfg` | 显式 | IdealPD + 力矩**延迟** `min_delay/max_delay`（物理步） | 建模控制时延，利于 sim2real |
| `RemotizedPDActuatorCfg` | 显式 | DelayedPD + 力矩限按 `joint_parameter_lookup` 角度插值 | 连杆传动比随角度变化 |
| `ActuatorNetMLPCfg` / `ActuatorNetLSTMCfg` | 显式·**学习型** | 神经网络从历史(pos/vel)预测力矩，`network_file=*.pt` | ANYmal/Go1 实测执行器网络，sim2real 最准 |

**隐式 vs 显式的本质区别**（`actuator_base_cfg.py` 注释，✅，重要教学点）：
- **隐式**：`stiffness`/`damping` **直接写进 PhysX**，PD 由引擎算；`effort_limit` 与 `effort_limit_sim` 等价。
- **显式**：`stiffness`/`damping` 由**执行器模型**用来算力矩；`effort_limit` 在**模型里**裁剪输出，而 `effort_limit_sim` 是给物理引擎的安全限（显式默认设成 `1e9` 即不干预，让模型自己裁）。
- 关节绑定一律靠 `joint_names_expr`（正则列表，如 `[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"]`）。其余可选字段：`armature`（加进关节空间惯量、稳数值）、`friction`/`dynamic_friction`/`viscous_friction`、`velocity_limit*`。

✅真实例（`anymal.py`，电机网络 + DC 两种写法都有）：

```python
ANYDRIVE_3_LSTM_ACTUATOR_CFG = ActuatorNetLSTMCfg(
    joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
    network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
    saturation_effort=120.0, effort_limit=80.0, velocity_limit=7.5,
)
# 用在 ArticulationCfg(..., actuators={"legs": ANYDRIVE_3_LSTM_ACTUATOR_CFG})
```

### mjlab：内建/电机/学习型执行器，按 `target_names_expr` 绑定 ✅

`mjlab/actuator/actuator.py` 的基类 `ActuatorCfg`（✅`@dataclass(kw_only=True)`）：

```python
class ActuatorCfg:
    target_names_expr: tuple[str, ...]    # ← 绑定到哪些关节（正则/名字）；注意：这一层叫 target_names_expr
    transmission_type: TransmissionType = JOINT   # JOINT | TENDON | SITE
    armature / frictionloss / viscous_damping: float | None = None   # None = 保留 XML 原值
    delay_min_lag / max_lag / hold_prob / update_period / per_env_phase  # 执行器延迟模型
```

具体类（`actuator/__init__.py` 导出）：`BuiltinPositionActuatorCfg`、`BuiltinPdActuatorCfg`、`BuiltinMotorActuatorCfg`、`BuiltinVelocityActuatorCfg`、`BuiltinMuscleActuatorCfg`、`DcMotorActuatorCfg`、`IdealPdActuatorCfg`、`LearnedMlpActuatorCfg`、`XmlActuatorCfg`。PD 类额外带 `stiffness`/`damping`。

✅G1 把关节按电机型号分成 6 个执行器组，增益**物理推导**而非手调：`STIFFNESS = armature·ωₙ²`、`DAMPING = 2·ζ·armature·ωₙ`（ωₙ=10 Hz, ζ=2.0），armature 由两级行星减速反射惯量 `reflected_inertia_from_two_stage_planetary(...)` 算出。这是「执行器建模/系统辨识」（Ch12）很好的真实范例。

```python
G1_ACTUATOR_7520_22 = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hip_roll_joint", ".*_knee_joint"),
    stiffness=STIFFNESS_7520_22, damping=DAMPING_7520_22,
    effort_limit=ACTUATOR_7520_22.effort_limit, armature=ACTUATOR_7520_22.reflected_inertia,
)
```

> ⚠️**两层命名的坑（已回写问题记录）**：mjlab 在**实体层**执行器绑定用 `target_names_expr`（关节名），但在**动作层** `JointPositionActionCfg` 用 `actuator_names`（执行器名），内部经 `entity.find_joints_by_actuator_names()` 解析。两者别混。Isaac Lab 没有这层区分，动作直接落到 `joint_names`。

## 1.6 碰撞 / 质量 / 惯量 怎么挂上去

### Isaac Lab：USD schema 属性配置 ✅

`isaaclab/sim/schemas/schemas_cfg.py`（✅）把物理属性拆成一组 `@configclass`，在 `spawn`/转换期挂到 USD：
- `MassPropertiesCfg`（质量）、`RigidBodyPropertiesCfg`（刚体：max 速度、阻尼、`max_depenetration_velocity` 等）、`CollisionPropertiesCfg`（`collision_enabled`、contact/rest offset）、`ArticulationRootPropertiesCfg`（`enabled_self_collisions`、solver 迭代数、sleep/stabilization 阈值）、`JointDrivePropertiesCfg`。
- **碰撞网格近似**是一棵类层次（`MeshCollisionPropertiesCfg` 派生）：`ConvexHullPropertiesCfg` / `ConvexDecompositionPropertiesCfg` / `TriangleMeshPropertiesCfg` / `TriangleMeshSimplificationPropertiesCfg` / `SDFMeshPropertiesCfg` / `BoundingCubePropertiesCfg` / `BoundingSpherePropertiesCfg`。`convert_mesh.py` 的 `--collision-approximation` 选项就是映射到这些类（✅`collision_approximation_map`）。
- **自碰撞**由 `ArticulationRootPropertiesCfg.enabled_self_collisions`（单 bool）控制；RL 里常关掉，用奖励惩罚代替（吞吐考量）。

### mjlab：`CollisionCfg` 是「正则→属性」的 spec 编辑器 ✅

`mjlab/utils/spec_config.py` 的 `CollisionCfg`（✅）是加载后施加的编辑器：

```python
class CollisionCfg(SpecCfg):
    geom_names_expr: tuple[str, ...]                 # 作用到哪些 geom
    contype:     int | dict[str,int] = 1
    conaffinity: int | dict[str,int] = 1
    condim:      int | dict[str,int] = 3             # ∈{1,3,4,6}
    priority:    int | dict[str,int] = 0
    friction:    tuple[float,...] | dict[str,tuple] | None = None
```

- **dict 值表示「正则 pattern → 值」**，因此可逐 geom 精调（脚底 `condim=3` 给摩擦锥，其他 `condim=1`）。
- ⚠️**自碰撞不是单个 bool**，而是经 `contype/conaffinity` 位掩码控制：G1 的 `FULL_COLLISION` 全开；`FULL_COLLISION_WITHOUT_SELF` 设 `contype=0, conaffinity=1`（关自碰）；`FEET_ONLY_COLLISION` 只开脚。这与 Isaac Lab 的单 bool 形成鲜明的「组织方式差异」——MuJoCo 的位掩码更细粒度。

---

# 第二部分 · 如何导入一个全新资产（含可复现命令）

> 本节是 Ch11 转换工具一节的**源码级补强 + 命令兑现**。Ch11 已讲 URDF→MJCF 六步调优、CoACD/V-HACD、`UrdfConverterCfg` 字段，本文聚焦：**(1) 转换器确实存在且 CLI 已实测可跑；(2) 几个源码里才看得到的关键行为（扩展版本钉死、rad→deg 内部换算）；(3) URDF→MJCF 的 MuJoCo 原生路径。**

## 2.1 Isaac Lab 转换工具链：确认存在 + 实测 CLI ✅🌐

`/root/isaaclab/scripts/tools/` 下有四个转换脚本（✅`ls` 确认）：`convert_urdf.py`、`convert_mjcf.py`、`convert_mesh.py`、`convert_instanceable.py`。它们都先 `AppLauncher` 起 Isaac Sim，再调 `isaaclab.sim.converters` 里的转换器类。

**✅实测**：`./isaaclab.sh -p scripts/tools/convert_urdf.py --help` 能正常打印用法（仅因缺 `input/output` 位置参数而报参数错——证明 CLI 表面与文档一致、脚本可加载）。

### URDF → USD（locomotion/manipulation 主路径）✅🌐

```bash
cd /root/isaaclab
./isaaclab.sh -p scripts/tools/convert_urdf.py \
    my_robot.urdf  ./usd_out/my_robot.usd \
    --merge-joints \                 # 合并 fixed joint 连接的 link（减复杂度）
    --fix-base \                     # 固定基座（机械臂用；locomotion 不要这个，要浮动基座）
    --joint-stiffness 0.0 \          # 转换期写进 USD 的 drive 刚度（locomotion 常设 0，留给加载期 ActuatorCfg）
    --joint-damping 0.0 \
    --joint-target-type none \       # position | velocity | none
    --headless
```

CLI 旗标（✅与 `convert_urdf.py` 源码一一对应）：`--merge-joints`、`--fix-base`、`--joint-stiffness`(默认100)、`--joint-damping`(默认1)、`--joint-target-type {position,velocity,none}`。脚本内部构造 `UrdfConverterCfg(...)` 后 `UrdfConverter(cfg)` 触发转换。

**⚠️源码才看得到的两个关键行为（Ch11 未明写，已回写问题记录）**：
1. **Isaac Sim 5.1 起，URDF importer 钉死老版本扩展 `isaacsim.asset.importer.urdf-2.4.31`**（✅`urdf_converter.py:__init__`：`if get_isaac_sim_version() >= Version("5.1"): manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf-2.4.31", True)`）。原因：5.1 默认改了 fixed-joint 合并行为（带 mass/inertia 的 link 不再被合并），钉老版本是为**向后兼容**老 URDF。**教学点**：这解释了「为什么同一份 URDF 在不同 Isaac Sim 版本里合并结果不同」。
2. **stiffness/damping 对旋转关节会做 rad→deg 内部换算**（✅`_set_joint_drive_stiffness`：非 prismatic 关节 `set_strength(math.pi/180 * stiffness)`）。**坑**：你写的 `--joint-stiffness 100` 对旋转关节不是直接进 USD 的 100。所以 locomotion 实践里**转换期常把增益设 0、把真正的 PD 留到加载期的 `ActuatorCfg`**（量纲清楚、可正则分组）——这正是库存 CFG（cartpole/anymal）`spawn` 里不写 drive、只在 `actuators=` 里写 stiffness 的原因。

`UrdfConverterCfg` 关键字段（✅`urdf_converter_cfg.py`）：`fix_base`(MISSING)、`merge_fixed_joints`(True)、`collider_type∈{"convex_hull","convex_decomposition"}`、`self_collision`(False)、`replace_cylinders_with_capsules`(False)、`link_density`(0.0，补缺失惯性)、`convert_mimic_joints_to_normal_joints`(False)、`root_link_name`、`joint_drive=JointDriveCfg(...)`。`JointDriveCfg` 支持两种增益式：`PDGainsCfg(stiffness, damping)` 或 `NaturalFrequencyGainsCfg(natural_frequency, damping_ratio)`（按目标带宽设计 PD，✅）。

### MJCF → USD（把 mjlab 精调好的模型搬进 Isaac Lab）✅🌐

```bash
./isaaclab.sh -p scripts/tools/convert_mjcf.py \
    my_robot.xml  ./usd_out/my_robot.usd \
    --import-sites \                 # 解析 <site>（足端/IMU 等参考点）
    --make-instanceable \            # 多环境克隆省显存
    --fix-base \
    --headless
```

✅旗标：`--fix-base`、`--import-sites`、`--make-instanceable`。`MjcfConverterCfg`（✅）字段：`import_inertia_tensor`(True)、`import_sites`(True)、`fix_base`(MISSING)、`self_collision`(False)、`link_density`(0.0)。**用途**：mjlab 侧已六步调优好 MJCF 时，直接转 USD 很省事；但⚠️PhysX 执行器模型 ≠ MuJoCo，转后仍要在 `ArticulationCfg` 重配 `actuators`（Ch11 `ins:rlmc-assets-contact-models` 旁注已点，本文给出确切 CLI）。

### Mesh(OBJ/STL/FBX) → USD（导入物体/地形几何）✅🌐

```bash
./isaaclab.sh -p scripts/tools/convert_mesh.py \
    object.obj  ./usd_out/object.usd \
    --make-instanceable \
    --collision-approximation convexDecomposition \   # 见下方枚举；"none"=只做视觉无碰撞
    --mass 1.0 \                                       # 省略则为静态(无质量)资产
    --headless
```

✅`--collision-approximation` 合法值：`convexDecomposition`(默认) / `convexHull` / `triangleMesh` / `meshSimplification` / `sdf` / `boundingCube` / `boundingSphere` / `none`。`--mass` 省略 ⇒ 静态物体；`--collision-approximation none` ⇒ 只有视觉、无碰撞。

### URDF→USD 转换前的 URDF「体检」清单（🌐官方 how-to 补强，Ch11 可并入）

🌐官方导入 how-to 明确建议的**预处理**（很多坑出在这一步）：
- 删 `<gazebo>` 与 `<transmission>` 标签（Isaac importer 不需要、可能干扰）；
- 精简 collision body（CAD 网格太重）；
- **把 joint damping/friction 设 0.0**（留给加载期执行器）；
- 对**想保留**的 fixed joint 加 `<dont_collapse>`（否则 `--merge-joints` 会把它合并掉）。

## 2.2 URDF → MJCF：MuJoCo 原生路径（给 mjlab/MuJoCo）✅

mjlab 用 MJCF，不需要上面的 USD 转换。把 URDF 变成 MJCF 有两条路：

**路径 A（推荐，MuJoCo 原生编译器）**：MuJoCo 自带「读 URDF → 存 MJCF」能力。最小可复现：

```bash
# 命令行（MuJoCo 自带工具，把 URDF 编译后存成 MJCF）
python -c "import mujoco; m=mujoco.MjSpec.from_file('my_robot.urdf'); open('my_robot.xml','w').write(m.to_xml())"
# 或交互式 compile/保存：python -m mujoco.viewer --mjcf my_robot.urdf  （在 viewer 里 Save xml）
```

随后**必做的六步调优**（Ch11 `sec` 已详述，本文只列锚点，不复述）：补 `<default>` 类、设 `solref/solimp` 接触、加 `<actuator>` 执行器、按 mjlab 习惯把碰撞 geom 归类、设 site、核对量纲。Menagerie 每个模型的 README 记录了这些真实决策，是最好的教材。

**路径 B（SolidWorks/CAD 直出）**：用 SolidWorks 的 `sw2urdf` 插件先出 URDF，再走路径 A。⚠️常见坑（Ch11 已列）：网格路径用了相对/绝对混用、单位（mm vs m）、关节轴与 body 主轴完全平行触发解析告警（微扰 1e-6）、惯量未做物理一致性校验。

**接入 mjlab 的最后一步——写 `EntityCfg`**（把上面调好的 MJCF 接进框架，✅范式同 §1.3 的 G1）：

```python
import mujoco
from mjlab.entity.entity import EntityCfg, EntityArticulationInfoCfg
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.utils.spec_config import CollisionCfg

MY_XML = "/abs/path/my_robot.xml"
def my_spec(): return mujoco.MjSpec.from_file(MY_XML)

def get_my_robot_cfg() -> EntityCfg:
    return EntityCfg(
        init_state=EntityCfg.InitialStateCfg(pos=(0,0,0.5), joint_pos={".*": 0.0}),
        spec_fn=my_spec,
        collisions=(CollisionCfg(geom_names_expr=(".*foot.*",), condim=3, priority=1),),
        articulation=EntityArticulationInfoCfg(
            actuators=(BuiltinPositionActuatorCfg(
                target_names_expr=(".*",), stiffness=40.0, damping=2.0),),
            soft_joint_pos_limit_factor=0.9,
        ),
    )
```

## 2.3 导入新资产的「通病」检查表（两框架通用）

| 坑 | 症状 | 根因 | 处理 |
|---|---|---|---|
| mesh 路径错 | 加载白模/找不到网格 | URDF `package://` 或相对路径解析失败 | 用绝对路径或确保 `meshdir`/`Props/` 同迁 |
| collision 过精细 | 数千环境时吞吐骤降/接触抖 | 直接用 CAD 凹网格 | convex decomposition（抓取用 CoACD，locomotion 用 convex_hull）|
| 惯量不合法 | 仿真发散/翻滚 | URDF 惯量张量非正定或缺失 | 校验/重算（mjlab 有 `pseudo_inertia` 物理一致 DR）|
| 关节方向/驱动反 | 训练难收敛 | URDF 轴向、转换期 drive 与加载期 PD 不一致 | 两道增益设置点必须对齐 |
| 单位/scale | 模型巨大或极小 | mm vs m、`distance_scale` | URDF importer `set_distance_scale(1.0)`=米；MJCF `<compiler>` 单位 |
| instanceable 漏拷 | 迁移后变骨架 | `make_instanceable` 把 mesh 分到 `Props/` | 连 `Props/` 整体迁移（Ch11 `pitfall` 已立）|
| fixed joint 被合并 | 想保留的坐标系没了 | `--merge-joints` 默认合并 | 想保留的加 `<dont_collapse>` |

---

# 第三部分 · 从零搭一个新任务工程（Ch22 主体素材）

> 本节是计划中 Ch22「DIY 从零搭项目」的核心，对应章在 .tex 层**尚未建文件**（见《问题与解决记录》新增条）。给出两框架**完整、可复现**的从零路径：组织模式 → 官方/最小骨架 → 注册机制 → 训练命令。

## 3.0 两框架共享的「心智模型」

不管哪个框架，搭一个新 RL 任务都是同一套**分层配置 + 导入即注册**模式：

```
┌─────────────────────────────────────────────────────────────┐
│ ① 机器人无关的「环境配置基类」                                  │
│    = 场景(scene) + 8 类 Manager(obs/action/reward/term/event/  │
│      command/curriculum/metrics) + 仿真时序(dt/decimation/...)  │
├─────────────────────────────────────────────────────────────┤
│ ② 机器人特定的「子类/覆盖」：把具体资产(ArticulationCfg/EntityCfg)│
│    注入场景，并按真实 body/joint 名改连线                        │
├─────────────────────────────────────────────────────────────┤
│ ③ 训练超参配置 (RslRlOnPolicyRunnerCfg / PPORunnerCfg)          │
├─────────────────────────────────────────────────────────────┤
│ ④ 一次「注册调用」，被「导入即注册」机制自动触发 → 接入训练入口   │
└─────────────────────────────────────────────────────────────┘
```

差异只在**用什么注册**：Isaac Lab 用 `gym.register`（标准 gymnasium），mjlab 用自己的 `register_mjlab_task`（私有注册表，⚠️**不是** gym，见 §3.3）。

## 3.1 Isaac Lab：官方模板生成器 `./isaaclab.sh --new`（最快的从零路径）✅🌐

**Isaac Lab 自带一个交互式工程生成器**——这是官方推荐的「从零」方式，但官方文档没讲清它在磁盘上生成了什么。源码精读结论（✅`isaaclab.sh:708-716` + `tools/template/{cli,generator,common}.py`）：

```bash
cd /root/isaaclab
./isaaclab.sh --new            # 等价 -n；先装 InquirerPy/rich/Jinja2，再跑 tools/template/cli.py
```

**它会交互式问你**（✅`common.py`/`cli.py`，方向键移动、空格选、回车确认）：
1. **external project（仓库外独立工程）** vs **internal task（贡献回 Isaac Lab 仓库内）**；
2. **workflow**：`manager-based` 或 `direct`；**type**：`single-agent` 或 `multi-agent`；
3. **RL 库**：`rsl_rl` / `rl_games` / `skrl` / `sb3`；
4. **算法**：单智能体 `[AMP, PPO]`、多智能体 `[IPPO, MAPPO]`（✅`common.py` 仅列这些；其他组合需对应 `templates/agents/<lib>_<algo>_cfg` 模板存在，缺则静默跳过）。

整套生成是 **Jinja2 模板渲染**（✅`jinja2.Environment(FileSystemLoader(TEMPLATE_DIR))`）。external 工程任务 id 前缀 `Template-`，internal 前缀 `Isaac-`；manager-based ⇒ `Template-<Name>-v0`，direct ⇒ `Template-<Name>-Direct-v0`。

### 生成的 external 工程目录树（✅按 `generator.py:_external` 逐调用重建；目录结构已核，个别文件名 ❓未实跑生成）

以 `my_project`、manager-based + rsl_rl(PPO) 为例：

```
my_project/
├── pyproject.toml                      # 从仓库根拷贝
├── .gitattributes .gitignore .dockerignore .pre-commit-config.yaml
├── README.md                           # 含安装/训练命令
├── .vscode/                            # tasks/launch 渲染
├── scripts/
│   ├── rsl_rl/                         # 从 scripts/reinforcement_learning/rsl_rl/ 拷贝
│   │   ├── train.py  play.py  cli_args.py    # 占位注释替换为 `import my_project.tasks  # noqa: F401`
│   ├── list_envs.py                    # "Isaac"→"Template-"
│   ├── zero_agent.py  random_agent.py
└── source/
    └── my_project/
        ├── pyproject.toml  setup.py    # 扩展包元数据（名字模板化）
        ├── config/extension.toml
        └── my_project/
            ├── __init__.py
            └── tasks/
                ├── __init__.py         # 调 import_packages(__name__, ["utils", ".mdp"])
                └── manager_based/
                    └── my_project/
                        ├── __init__.py            # ← gym.register(id="Template-My-Project-v0", ...)
                        ├── my_project_env_cfg.py   # ★ 一份「可跑的 cartpole」起步 cfg
                        ├── mdp/{__init__.py, rewards.py}   # 从模板拷贝
                        └── agents/{__init__.py, rsl_rl_ppo_cfg.py}
```

✅生成末尾自动 `git init && git add -f . && git commit -m "Initial commit"`。**教学要点**：「从零」不是从空文件开始——**生成器给你一个完整可跑的 cartpole 任务**（✅`templates/tasks/manager-based_single-agent/env_cfg` 就是带 `CARTPOLE_CFG.replace(...)` 的全套 cfg），你**把 cartpole 改成你的机器人**即可。这是降低「白页恐惧」的关键设计，官方文档完全没强调。

⚠️注意：源码里 external 的 `docker/` 输出块**当前被注释掉**（模板存在但不生成）；internal task 路径则不建工程树，直接把新任务文件夹丢进 `source/isaaclab_tasks/isaaclab_tasks/`。

### 安装 + 列出 + 训练（🌐README/官方文档命令）

```bash
python -m pip install -e source/my_project          # editable 安装
python scripts/list_envs.py                          # 验证注册（过滤 "Template-"）
python scripts/rsl_rl/train.py --task=Template-My-Project-v0
python scripts/zero_agent.py   --task=Template-My-Project-v0   # 零动作 sanity
python scripts/random_agent.py --task=Template-My-Project-v0   # 随机动作 sanity
```

## 3.2 Isaac Lab：不用生成器，手写一个 manager-based 任务（理解每个文件）✅

为讲清「每个文件干什么」，以内置 velocity locomotion 任务为解剖样本（✅`source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/`）：

```
velocity/
├── velocity_env_cfg.py          # ★ 机器人无关的环境配置基类（scene + 8 manager + 时序）
├── mdp/                          # 任务特有的 MDP 「项函数库」(rewards.py/curriculums.py/terminations.py)
│   └── __init__.py              #   re-export isaaclab.envs.mdp.* + 上面的自定义函数
└── config/                       # 机器人特定特化 + 注册
    ├── __init__.py              #   空（仅使 config 可导入）
    └── go2/
        ├── __init__.py          # ← gym.register(...) 在这里
        ├── rough_env_cfg.py     #   子类化基类、注入 UNITREE_GO2_CFG
        ├── flat_env_cfg.py      #   再子类化 rough、铺平地形
        └── agents/
            ├── rsl_rl_ppo_cfg.py        # RslRlOnPolicyRunnerCfg 子类（Python）
            └── skrl_*.yaml / rl_games_*.yaml / sb3_*.yaml   # 其他库用 YAML
```

**关键架构事实**：有**两级 `mdp/`**——`velocity/mdp/` 是共享的「**项函数库**」（普通函数 + 少量 cfg 类如 `UniformVelocityCommandCfg`），它 re-export `isaaclab.envs.mdp` 的通用项；env cfg 用 `mdp.feet_air_time`、`mdp.track_lin_vel_xy_exp` 引用。

### ① 环境配置基类骨架（✅`velocity_env_cfg.py` 删减版）

**导入别名块**（✅这就是 `ObsTerm` 等别名的来源，注意 `RL` 大写——与 mjlab 相反）：

```python
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
```

**场景**——机器人留 `MISSING`，由机器人子类填：

```python
@configclass
class MySceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="generator", ...)
    robot: ArticulationCfg = MISSING                    # ← 子类注入
    height_scanner = RayCasterCfg(prim_path="{ENV_REGEX_NS}/Robot/base", ...)
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*",
                                      history_length=3, track_air_time=True)
```

**每个 Manager 是独立 `@configclass`**（观测组继承 `ObsGroup`）：

```python
@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"],
                                           scale=0.5, use_default_offset=True)

@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        actions  = ObsTerm(func=mdp.last_action)
        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
    policy: PolicyCfg = PolicyCfg()

@configclass
class RewardsCfg:
    track_lin_vel_xy_exp = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)})
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    feet_air_time  = RewTerm(func=mdp.feet_air_time, weight=0.125,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*FOOT"),
                "command_name": "base_velocity", "threshold": 0.5})

@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0})

@configclass
class EventCfg:    # 域随机化：mode ∈ startup/reset/interval
    add_base_mass = EventTerm(func=mdp.randomize_rigid_body_mass, mode="startup", params={...})
    reset_base    = EventTerm(func=mdp.reset_root_state_uniform,  mode="reset",   params={...})
    push_robot    = EventTerm(func=mdp.push_by_setting_velocity, mode="interval",
                              interval_range_s=(10.0, 15.0), params={...})
```

**顶层 env cfg 把 manager 拼起来 + `__post_init__` 设时序**（✅）：

```python
@configclass
class LocomotionVelocityRoughEnvCfg(ManagerBasedRLEnvCfg):
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()
    def __post_init__(self):
        self.decimation = 4                       # 每 4 个物理步控制一次
        self.episode_length_s = 20.0
        self.sim.dt = 0.005                        # 200Hz 物理 → 50Hz 控制
        self.sim.render_interval = self.decimation
```

### ② 机器人子类——「注入资产」的那一行（✅`config/go2/rough_env_cfg.py`）

```python
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

@configclass
class UnitreeGo2RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")  # ← 注入资产的关键一行
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base"
        self.actions.joint_pos.scale = 0.25
        self.events.push_robot = None                                  # ← 把某项设 None = 删除它
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base"
```

**三个必须讲清的教学惯用法**（✅）：
1. 特化用「`super().__post_init__()` 后改字段」分层叠加；
2. `CFG.replace(prim_path=...)` 把库存资产 cfg 克隆进场景；
3. **把任何项/Manager 属性设成 `None` 就是删除它**——这是 Manager 系统关功能的惯用法（`flat_env_cfg.py` 就靠 `self.scene.height_scanner = None` 等把地形/高度扫描全删）。

### ③ 注册（✅`config/go2/__init__.py` 原文）

```python
import gymnasium as gym
from . import agents

gym.register(
    id="Isaac-Velocity-Flat-Unitree-Go2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",          # manager-based 一律用这个通用 env 类
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeGo2FlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeGo2FlatPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_flat_ppo_cfg.yaml",
    },
)
```

**关键模式**（✅）：manager-based 任务的 `entry_point` **永远是通用的 `isaaclab.envs:ManagerBasedRLEnv`**——任务身份**全靠 `env_cfg_entry_point`**（一个 `"模块:类名"` 字符串）携带。每个任务通常注册 **4 个 id**：`Flat`/`Rough` × 正式/`-Play-v0`（Play=小场景评估变体）。训练库各自的 cfg 用额外 entry_point 键传（`rsl_rl_cfg_entry_point` 指 Python 类，其他指 `.yaml`）。

### ④ 注册怎么被触发——「导入即注册」✅

`source/isaaclab_tasks/isaaclab_tasks/__init__.py` 末尾（✅）：

```python
from .utils import import_packages
_BLACKLIST_PKGS = ["utils", ".mdp", "pick_place", ...]
import_packages(__name__, _BLACKLIST_PKGS)
```

`import_packages`（✅`utils/importer.py`）**递归 import 每个子包**，于是每个 `config/<robot>/__init__.py` 被执行，其 `gym.register(...)` 作为 import 副作用触发。**所以「只要 import 了 `isaaclab_tasks`，所有内置任务就注册好了」**；`.mdp` 黑名单防止把项函数库当任务导入。external 工程同理——它的 `tasks/__init__.py` 也调 `import_packages`，训练脚本里那行 `import my_project.tasks  # noqa` 就是触发器。

### Isaac Lab 训练命令（✅入口存在）

```bash
cd /root/isaaclab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-v0 --num_envs 4096 --headless
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-Unitree-Go2-Play-v0 --num_envs 50
```
（✅`train.py` 旗标：`--task --num_envs --max_iterations --seed --headless --video --distributed` + Hydra 透传；`train.py` 钉 `RSL_RL_VERSION="3.0.1"`。）

### Isaac Lab 从零 7 步（Ch22 可直接用作 checklist）

1. `./isaaclab.sh --new` → external + manager-based + single-agent + rsl_rl/PPO → 得到 cartpole 骨架（自动 git init）；
2. `pip install -e source/<name>`；
3. 改 `<name>_env_cfg.py`：把 `CARTPOLE_CFG` 换成你的 `ArticulationCfg`，重写 Actions/Observations/Rewards/Terminations/Events，`__post_init__` 设 `decimation/sim.dt/episode_length_s`；
4. 在任务 `mdp/` 加自定义项函数（签名 `func(env, ...) -> torch.Tensor`）；
5. 改 `gym.register(...)` 的 id 与 cfg 类名；
6. 调 `agents/rsl_rl_ppo_cfg.py`（`PPORunnerCfg`）；
7. `python scripts/list_envs.py` → `python scripts/rsl_rl/train.py --task=<id> --headless`。

## 3.3 mjlab：从零搭任务（结构 + 注册 + 命令）✅

mjlab 用**同样的分层模式**，但注册机制不同。解剖样本 `mjlab/tasks/velocity/`（✅）：

```
tasks/velocity/
├── velocity_env_cfg.py         # make_velocity_env_cfg() → ManagerBasedRlEnvCfg（机器人无关基类）
├── mdp/                         # 任务特有项「函数」(rewards.py/observations.py/terminations.py/velocity_command.py/curriculums.py)
├── rl/runner.py                # 自定义 runner：VelocityOnPolicyRunner
└── config/
    ├── __init__.py             # 空(0字节)
    └── g1/
        ├── __init__.py         # ← 注册在这里（register_mjlab_task ×2）
        ├── env_cfgs.py         # unitree_g1_{flat,rough}_env_cfg() —— 机器人特定覆盖
        └── rl_cfg.py           # unitree_g1_ppo_runner_cfg() → RslRlOnPolicyRunnerCfg
```

### 三层配置（✅与 Isaac Lab 对应，但风格不同）

1. **`velocity_env_cfg.py::make_velocity_env_cfg()`**——机器人无关**工厂函数**，返回填好 scene + 8 个 manager 的 `ManagerBasedRlEnvCfg`；机器人相关位留哨兵（`body_name=""`、`scale=0.5  # Override per-robot`）。
2. **`config/g1/env_cfgs.py`**——调工厂后**直接 mutate**：`cfg.scene.entities = {"robot": get_g1_robot_cfg()}`，把传感器坐标系接到真实 body/site 名，设动作 scale、奖励 asset_cfg、play 覆盖。`flat = rough + 差异`。
3. **`config/g1/rl_cfg.py`**——PPO runner cfg。

**关键风格差异（教学对比）**：mjlab 的 **Manager 是普通 dict `{name: TermCfg}`**，不是嵌套 config 类；项 cfg 用**全名**从 `mjlab.managers` 导入（`ObservationTermCfg`/`RewardTermCfg`/…），⚠️**没有** `RewTerm` 这类短别名（那是 Isaac Lab 习惯）。env cfg 长这样（✅删减）：

```python
ManagerBasedRlEnvCfg(
  scene=SceneCfg(terrain=..., sensors=(...), num_envs=1, extent=2.0),
  observations={"actor": ObservationGroupCfg(terms={...}, concatenate_terms=True, enable_corruption=True),
                "critic": ObservationGroupCfg(...)},
  actions={"joint_pos": JointPositionActionCfg(entity_name="robot", actuator_names=(".*",),
                                               scale=0.5, use_default_offset=True)},
  commands={"twist": UniformVelocityCommandCfg(entity_name="robot", ...)},
  events={...}, rewards={...}, terminations={...}, curriculum={...}, metrics={...},
  sim=SimulationCfg(nconmax=35, njmax=1500,
                    mujoco=MujocoCfg(timestep=0.005, iterations=10, ls_iterations=20)),
  decimation=4, episode_length_s=20.0,
)
```
- ✅机器人经 `cfg.scene.entities = {"robot": get_g1_robot_cfg()}` 进场；`"robot"` 这个 key 就是各项里 `SceneEntityCfg("robot", ...)` / `entity_name` 用的句柄。
- ✅动作层 `JointPositionActionCfg` 字段是 `entity_name`（非 `asset_name`）+ `actuator_names`（非 `joint_names`），`use_default_offset` 仅 Position/Velocity 类有。

### 注册（⚠️不是 gym！）✅

✅`config/g1/__init__.py` 原文：

```python
from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
from .env_cfgs import unitree_g1_flat_env_cfg, unitree_g1_rough_env_cfg
from .rl_cfg import unitree_g1_ppo_runner_cfg

register_mjlab_task(
    task_id="Mjlab-Velocity-Flat-Unitree-G1",
    env_cfg=unitree_g1_flat_env_cfg(),
    play_env_cfg=unitree_g1_flat_env_cfg(play=True),
    rl_cfg=unitree_g1_ppo_runner_cfg(),
    runner_cls=VelocityOnPolicyRunner,
)   # 另一条注册 Rough 变体
```

⚠️**重大组织差异（已回写问题记录）**：**mjlab 内置任务用 `register_mjlab_task` 写进私有注册表 `_REGISTRY`，不用 `gym.register`**（✅`mjlab/tasks/registry.py`：模块级 `_REGISTRY: dict[str,_TaskCfg]`，存 `_TaskCfg(env_cfg, play_env_cfg, rl_cfg, runner_cls)`，重复注册报错；访问器 `list_tasks()/load_env_cfg(name,play)/load_rl_cfg/load_runner_cls`）。`mjlab/__init__.py` 里唯一的 gymnasium 提及是一段陈旧 docstring（讲**外部** pip 包经 `mjlab.tasks` entry-point 加载，那条路才可能 gym.register；12 个内置任务都不用）。

**discovery**（✅`mjlab/tasks/__init__.py`）：同样是「导入即注册」——

```python
from mjlab.utils.lab_api.tasks.importer import import_packages
_BLACKLIST_PKGS = ["utils", ".mdp"]
import_packages(__name__, _BLACKLIST_PKGS)
```

`import_packages`（✅一个 vendored 的 Isaac-Lab 工具）递归 import `mjlab.tasks` 下每个子包，import `config/g1/__init__.py` 即触发其 `register_mjlab_task`。

### mjlab 新任务最小骨架（✅必须✓/可继承⊙）

```
tasks/<mytask>/
├── __init__.py                   ⊙ 可空（仅成包，供 discovery 走到）
├── <mytask>_env_cfg.py           ✓ 工厂 make_<mytask>_env_cfg() → ManagerBasedRlEnvCfg
│                                    （scene + 8 manager dict + sim/decimation/episode_length_s）
├── mdp/                          ⊙ 仅当需要自定义项函数；否则全用 mjlab.envs.mdp / 别的任务的 mdp
└── config/<robot>/
    ├── __init__.py               ✓ register_mjlab_task(task_id=..., env_cfg=..., play_env_cfg=..., rl_cfg=...)
    ├── env_cfgs.py               ✓ <robot>_env_cfg(play=False): 调工厂 + cfg.scene.entities={"robot": get_<robot>_robot_cfg()}
    └── rl_cfg.py                 ✓ <robot>_ppo_runner_cfg() → RslRlOnPolicyRunnerCfg
```

**用户实际要写的最少符号**：① 返回 `ManagerBasedRlEnvCfg` 的 env-cfg 工厂；② 把 `EntityCfg` 注入 `cfg.scene.entities` 的机器人 cfg 函数；③ `RslRlOnPolicyRunnerCfg` 函数；④ 一个落在 `mjlab.tasks` 下的 `config/<robot>/__init__.py` 里的 `register_mjlab_task(...)`。`runner_cls` 可选（默认 `MjlabOnPolicyRunner`）。`mdp/` 全可选（通用项 `joint_pos_rel/last_action/action_rate_l2/reset_root_state_uniform/dr.*` 都在 `mjlab.envs.mdp`）。❓出仓库（独立 pip 包）的任务则改为在包元数据声明 `mjlab.tasks` entry point，由 `_import_registered_packages` 加载（读 docstring，未实跑）。

### mjlab 训练命令（✅console script + 两段 tyro 解析）

✅入口 `entry_points.txt`：`train = mjlab.scripts.train:main`（还有 `play/demo/list-envs/export-scene/viz-nan`）。`train.py::main` 先 `import mjlab.tasks` 填注册表，再两段 tyro 解析（**第一个位置参数=任务 id**，其余进 `TrainConfig`，默认 `TrainConfig.from_task(task)`）。

```bash
train Mjlab-Velocity-Flat-MyRobot \
    --env.scene.num-envs 4096 \      # --env.* = ManagerBasedRlEnvCfg 树
    --agent.max-iterations 30000 \   # --agent.* = RslRlOnPolicyRunnerCfg
    --enable-nan-guard True
play Mjlab-Velocity-Flat-MyRobot --num-envs 32 --viewer native
```
⚠️tyro 旗标（✅`TYRO_FLAGS`）：布尔用 `--flag False`（**无 `--no-flag`**）；元组用 `--x "(1,2,3)"`。`--num-iterations` 不存在（用 `--agent.max-iterations`）。

---

# 第四部分 · 三框架对比小结 + UniLab 钩子

| 维度 | mjlab | Isaac Lab | UniLab |
|---|---|---|---|
| 资产格式 | MJCF（MuJoCo） | USD（PhysX，URDF/MJCF 转入） | ❓见下 |
| 资产来源指向 | `EntityCfg.spec_fn`（返回 `MjSpec` 的**函数**） | `UsdFileCfg.usd_path`（**字符串路径**，可云端 Nucleus） | ❓ |
| 库存资产 | 随包 `asset_zoo/`（G1/Go1/YAM） | 云端 Nucleus + `isaaclab_assets` 配方库 | ❓ |
| 模型构建 | `MjSpec.compile()`→MjModel→mjwarp | USD 加载→PhysX | ❓ |
| 执行器 | `ActuatorCfg.target_names_expr` | `ActuatorCfg.joint_names_expr`（隐式/显式/网络） | ❓ |
| 自碰撞 | `contype/conaffinity` 位掩码 | `enabled_self_collisions` 单 bool | ❓ |
| Manager 组织 | 普通 dict `{name: TermCfg}` | 嵌套 `@configclass` | ❓ |
| 注册 | ⚠️`register_mjlab_task`→私有 `_REGISTRY` | `gym.register`（gymnasium 标准） | ❓ |
| discovery | `import_packages` 导入即注册 | `import_packages` 导入即注册 | ❓ |
| 训练入口 | console `train <ID> --env.* --agent.*` | `./isaaclab.sh -p .../train.py --task <ID>` | ❓ |
| 从零脚手架 | 手写最小骨架（无官方生成器） | `./isaaclab.sh --new`（Jinja2，cartpole 起步） | ❓ |

**UniLab 钩子（另一个 agent 负责，本文留坑）**：UniLab 是异构「CPU 仿真 + GPU 策略」框架（`github.com/unilabsim/UniLab`，后端 mujoco/motrix），其资产/项目模型与上面两者差异较大——配置走 **Hydra dataclass**（`conf/<algo>/task/<task>/<backend>.yaml`），不是 mjlab 式 Manager 也不是 gym.register；env 基类在 `src/unilab/base/`（`NpEnv/NpEnvState` 等）。**资产导入与从零搭项目的 UniLab 落点 → 由 UniLab 专责 agent 补全，本文各 ❓格待并。**

---

# 附：本文「做了什么 / 核实了什么」

**直接读源码核实（✅）**：Isaac Lab 四个转换脚本存在 + `convert_urdf.py --help` 实测可跑；`urdf_converter.py`/`urdf_converter_cfg.py`/`mjcf_converter_cfg.py`/`mesh_converter_cfg.py`/`asset_converter_base_cfg.py` 全文；`utils/assets.py` 资产根逻辑；`actuator_base_cfg.py`/`actuator_pd_cfg.py` 执行器模型；`schemas_cfg.py` 物理属性类；`isaaclab_assets` 的 cartpole/anymal/unitree CFG。mjlab：`entity.py`(EntityCfg)、`g1_constants.py`、`actuator/`、`spec_config.py`(CollisionCfg)、`tasks/velocity/` 全树、`registry.py`、`tasks/__init__.py`、`scripts/train.py`、`entry_points.txt`。Isaac Lab 任务侧：`velocity/` 全树、`config/go2/{__init__,rough_env_cfg,flat_env_cfg}.py`、`isaaclab_tasks/__init__.py`、`tools/template/{cli,generator,common}.py`。

**官方文档对照（🌐）**：Isaac Lab「Import a New Asset」how-to（三条转换命令 + URDF 预处理清单）；「Create new project or task」模板生成器页（external/internal、workflow、RL 库、生成树、安装/训练命令）。两者与源码一致。

**关键勘误（⚠️，已回写《问题与解决记录》）**：(1) mjlab 内置任务用 `register_mjlab_task`→私有 `_REGISTRY`，**非 `gym.register`**；(2) URDF importer 在 Isaac Sim 5.1 钉死扩展 `2.4.31`（fixed-joint 合并向后兼容）；(3) URDF 转换器对旋转关节 stiffness/damping 做 rad→deg 内部换算；(4) Ch22「从零搭项目」对应 .tex 文件尚未建立。

**未实跑/存疑（❓）**：生成器实际产物的个别文件名（未真跑生成，按 `generator.py` 逻辑重建，目录结构已核）；mjlab 出仓库 entry-point 扩展路径（读 docstring）；全部 UniLab 格（另 agent 负责）。Isaac Lab 全仿真在 gpufree 被 GPU 抢占卡死，故 Isaac 侧只做 import/CLI-help/源码级核实（详见《问题与解决记录》C.1）。
