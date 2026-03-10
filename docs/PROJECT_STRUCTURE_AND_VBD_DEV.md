# NovaPhy 项目结构与 VBD/AVBD 集成说明

本文档主要记录三件事：（1）NovaPhy 的整体项目结构；（2）从 Python 到 C++ 内核的调用链；（3）VBD/AVBD 集成的设计方案与按日期推进的开发日志（包括 2026-03-03 之后新增的 AVBD 骨架与重写工作）。

---

## 1. 整体结构概览（NovaPhy 核心）

下面只列出与**物理模拟与 Python 绑定**强相关的部分目录，便于在汇报中快速定位模块位置：

```
NovaPhy/
├── CMakeLists.txt
├── pyproject.toml
├── environment.yml
│
├── include/novaphy/                  # 公共 C++ 头文件（与 src/ 一一对应）
│   ├── math/                         # 数学类型与工具（Vec3f / Quatf / Mat3f 等）
│   ├── core/                         # RigidBody, Shape, Model, ModelBuilder, SimState
│   ├── collision/                    # Broadphase, Narrowphase, AABB, Contact
│   ├── dynamics/                     # Integrator, FreeBodySolver, Featherstone
│   ├── sim/                          # World（刚体仿真入口）
│   ├── ipc/                          # IPCWorld, IPCConfig（libuipc 适配层）
│   ├── fluid/                        # PBF 粒子流体模块
│   └── vbd/                          # ★ 新增：VBD/AVBD 公共接口（本次工作重点）
│       ├── vbd_config.h              # VBDConfig（dt / gravity / iterations / alpha / gamma / beta_*）
│       ├── vbd_solver.h              # VbdSolver & AvbdContact（3D AVBD 核心接口）
│       └── vbd_world.h               # VBDWorld（World 风格包装类）
│
├── src/                              # C++ 实现
│   ├── math/
│   ├── core/
│   ├── collision/
│   ├── dynamics/
│   ├── sim/                          # World::step()
│   ├── ipc/                          # IPC 适配 libuipc
│   ├── fluid/
│   └── vbd/                          # ★ 新增：VBD/AVBD 模块具体实现
│       ├── CMakeLists.txt            # VBD 子模块构建脚本
│       ├── vbd_solver.cpp            # 3D AVBD 求解器实现（primal / dual / 投影 / 持久化）
│       └── vbd_world.cpp             # VBDWorld 实现（桥接到 SimState）
│
├── python/
│   ├── novaphy/                      # Python 包（__init__.py 导出 API，包括 VBDWorld）
│   └── bindings/                     # pybind11 绑定（bind_*）
│       └── bind_vbd.cpp              # ★ 新增：VBDConfig / VBDWorld 绑定
│
├── demos/                            # Python demo（包括 VBD/AVBD 验证场景）
│   ├── demo_stack.py                 # PGS 版 box 堆叠
│   ├── demo_pyramid.py               # PGS 版 4-3-2-1 金字塔
│   ├── demo_vbd_stack.py             # ★ 新增：VBD 版 box 堆叠（对标 avbd-demo3d sceneStack）
│   └── demo_vbd_pyramid.py           # ★ 新增：VBD 版金字塔（对标 avbd-demo3d scenePyramid）
│
├── tests/                            # Python / C++ 测试
└── external/
    └── libuipc/                      # 第三方 IPC 库（3D, CUDA 后端）
```

---

## 2. 调用栈总览（Python → C++）

### 2.1 自由刚体管线（`World`）

以 `demos/demo_stack.py` 为例，调用链为：

1. **Python 层**  
   - 使用 `novaphy.ModelBuilder` 构建场景：`model = builder.build()`  
   - 构造 `World`：`world = novaphy.World(model, settings)`  
   - 主循环：`world.step(dt)` → 读取 `state = world.state`  

2. **Python 绑定层**  
   - `bind_main.cpp` 调用 `bind_sim(m)` 注册模拟相关类型  
   - `bind_sim.cpp` 暴露：`ModelBuilder`、`Model`、`SolverSettings`、`SimState`、`World`，其构造与 `step()` 直接映射到 C++ `novaphy::World`  

3. **C++ 内核层**  
   - `include/novaphy/sim/world.h` 声明 `World`  
   - `src/sim/world.cpp` 实现 `World::step(dt)`：  
     - 根据重力 / 外力积分速度；  
     - Broadphase（`SweepAndPrune`）+ Narrowphase（`collide_shapes`）；  
     - 通过 `FreeBodySolver`（PGS / Sequential Impulse）求解接触；  
     - 积分位置、更新 `SimState`。  

这一整条管线完全是 3D 的：位置用 `Vec3f`，旋转用 `Quatf`，刚体有 3D 惯性张量，形状是 3D box/sphere/plane。

### 2.2 IPC 管线（`IPCWorld`）

以 `demos/demo_ipc_stack.py` 为例，调用链为：

1. **Python 层**  
   - 同样先通过 `ModelBuilder` 构建 `Model`。  
   - 配置 `IPCConfig`（时间步长 dt、摩擦、kappa 等）。  
   - 构造 `IPCWorld`：`world = novaphy.IPCWorld(model, config)`。  
   - 主循环：`world.step()` → 读取 `state = world.state`。  

2. **Python 绑定层**  
   - `bind_main.cpp` 在启用 IPC 时调用 `bind_ipc(m)`。  
   - `bind_ipc.cpp` 暴露 `IPCConfig`、`IPCWorld` 到 Python。  

3. **C++ 内核层（libuipc 适配）**  
   - `include/novaphy/ipc/ipc_world.h` 声明 `IPCWorld`。  
   - `src/ipc/ipc_world.cpp`：  
     - 将 NovaPhy 的 `Model` 转成 libuipc 所需的数据结构（3D 网格、材料、接触模型等）；  
     - 初始化 libuipc 的 `Engine` / `World` / `Scene`；  
     - 每次 `step()` 调用 `uipc::World::advance/sync/retrieve`，再将结果写回 NovaPhy 的 `SimState`。  

同样，这条管线也是完整 3D：libuipc 是 3D 引擎（刚体 / 可变形体网格、3D 接触），NovaPhy 把 3D 几何和变换直接传给它。

### 2.3 新 VBD/AVBD 管线（`VBDWorld`）

这是本次 AVBD 重写新增的一条管线，用于跑 3D AVBD 求解器，对标 avbd-demo3d 的 `Solver`：

1. **Python 层**  
   - 用 `ModelBuilder` 构建 `Model`。  
   - 创建 `VBDConfig`：`cfg = novaphy.VBDConfig()`（字段与 demo3d `Solver` 参数一致）。  
   - 创建 `VBDWorld`：`world = novaphy.VBDWorld(model, cfg)`。  
   - 主循环：`world.step()` → 读取 `state = world.state`。  

2. **Python 绑定层**  
   - `bind_main.cpp` 调用 `bind_vbd(m)`。  
   - `bind_vbd.cpp` 暴露 `VBDConfig`、`VBDWorld`。  

3. **C++ 内核层（AVBD 求解器）**  
   - `include/novaphy/vbd/vbd_world.h` 声明 `VBDWorld`。  
   - `src/vbd/vbd_world.cpp`：内部持有 `Model`、`SimState` 与 `VbdSolver`，构造函数从 `model.initial_transforms` 初始化 `state`，并调用 `solver.set_model(model)`；`step()` 则直接调用 `solver.step(model, state)`。  
   - `include/novaphy/vbd/vbd_solver.h` / `src/vbd/vbd_solver.cpp`：  
     - 定义 3D 版 `VbdSolver`，实现 `set_config` / `set_model` / `step`；  
     - 内部包含接触数据结构 `AvbdContact`、per-body 6×6 primal/dual 逻辑、contact persistence（跨帧 warmstart）、位置投影、速度更新等。  

---

## 3. 3D vs 2D: NovaPhy vs AVBD demo

- **NovaPhy core and IPC**:
  - All rigid-body dynamics, collision detection, and IPC integration in
    NovaPhy are 3D.
  - `World` works with 3D positions and rotations, and shapes are 3D
    primitives (boxes, spheres, planes).
  - `IPCWorld` uses libuipc, which is a 3D library handling rigid and
    deformable objects, collisions, and contacts in 3D.

- **AVBD reference project (`avbd-demo2d`)**:
  - The Utah AVBD demo ([project page](https://graphics.cs.utah.edu/research/projects/avbd/),
    [reference code](https://github.com/savant117/avbd-demo2d.git)) is a
    **2D** implementation intended to clearly demonstrate the algorithm.
  - It uses 2D boxes (x, y, rotation θ) and OpenGL/SDL for visualization.

- **Integration plan**:
  - NovaPhy will host a **3D VBD/AVBD pipeline** (`VBDWorld`) that borrows
    the mathematical structure from the 2D AVBD demo (block descent, augmented
    Lagrangian, constraint rows, warmstarting, etc.), but operates on
    3D bodies and constraints.
  - This means you conceptually port AVBD from 2D → 3D inside `VbdSolver`,
    while keeping the overall engine (NovaPhy core + IPC + VBD) purely 3D.

---

### 3.3 AVBD 实现相关目录一览（本次改动）

> 这一节专门列出“这次 AVBD 重写/集成”直接涉及的目录和文件，方便组会时一张图讲清楚改动范围。

```text
NovaPhy/
├── include/novaphy/
│   └── vbd/                          # ★ 新增：VBD/AVBD 公共接口
│       ├── vbd_config.h              # VBDConfig（dt / gravity / iterations / alpha / gamma / beta_*）
│       ├── vbd_solver.h              # VbdSolver & AvbdContact（3D AVBD 接触/求解接口）
│       └── vbd_world.h               # VBDWorld 封装（对齐 World / IPCWorld 用法）
│
├── src/
│   └── vbd/                          # ★ 新增：AVBD 具体实现
│       ├── CMakeLists.txt            # VBD 子模块构建脚本
│       ├── vbd_solver.cpp            # 3D AVBD 求解器核心（primal / dual / contact persistence / 投影）
│       └── vbd_world.cpp             # VBDWorld 实现（内部持有 Model / SimState / VbdSolver）
│
├── python/
│   ├── bindings/
│   │   └── bind_vbd.cpp              # ★ 新增：VBDConfig / VBDWorld 的 pybind11 绑定
│   │
│   └── novaphy/
│       └── __init__.py               # ★ 修改：导出 VBDConfig / VBDWorld 到 Python API
│
└── demos/                            # ★ 新增：用于验证 AVBD 行为的 demo
    ├── demo_vbd_stack.py             # VBD 版 box 堆叠（对标 avbd-demo3d sceneStack）
    └── demo_vbd_pyramid.py           # VBD 版金字塔（对标 avbd-demo3d scenePyramid）
```

---

## 4. 每日开发记录

本节只用中文记录你在 NovaPhy 中进行 VBD/AVBD 集成的每天工作内容，方便后续写周报、月报和论文中的工程部分（代码和文件名保持英文）。

### 2026-03-03 —— VBD/AVBD 集成骨架（仅 CPU 占位实现）

- 新增公共头文件与接口：
  - `include/novaphy/vbd/vbd_config.h`  
    - 定义 `VBDConfig` 结构，包含 `dt`、`iterations`、`beta`、`alpha`、`gamma`、`post_stabilize`、`gravity` 等字段，作为后续 VBD/AVBD 求解器的统一配置入口。  
  - `include/novaphy/vbd/vbd_world.h`  
    - 定义 `VBDWorld`，对外接口尽量与 `World` / `IPCWorld` 保持一致，包括：  
      - 构造函数 `VBDWorld(const Model&, const VBDConfig& = VBDConfig{})`  
      - `step()`、`state()`、`model()`、`config()` 等方法。  
    - 使用 pimpl（`Impl`）隐藏内部实现，方便后续迭代算法与 GPU 实现而不破坏 API。

- 新增 VBD 模块实现骨架：
  - `src/vbd/CMakeLists.txt`  
    - 新建静态库 `novaphy_vbd`，源码包含 `vbd_world.cpp`、`vbd_solver.cpp`，并链接 `novaphy_core`，公共头文件目录为 `${CMAKE_SOURCE_DIR}/include`。  
  - `src/vbd/vbd_world.cpp`  
    - 实现 `VBDWorld::Impl`：持有 `Model`、`VBDConfig`、`SimState`、`VbdSolver`。  
    - 构造函数中使用 `model.initial_transforms` 初始化 `state`，并调用 `solver.set_model(model)`。  
    - `VBDWorld::step()` 内部简单调用 `impl_->solver.step(model, state)`，为后续真正的 VBD/AVBD 求解逻辑预留入口。  
  - `include/novaphy/vbd/vbd_solver.h`（头文件）、`src/vbd/vbd_solver.cpp`（实现）  
    - 定义 `VbdSolver` 接口：`set_config`、`set_model`、`step`。  
    - 当天实现为最简占位：`set_model` 暂为空，`step` 中只对非静态刚体施加重力并做显式积分，用来打通 `Python → VBDWorld → VbdSolver` 的整体调用链。

- CMake 集成：
  - 顶层 `CMakeLists.txt` 在 `add_subdirectory(src)` 之后新增 `add_subdirectory(src/vbd)`，确保 VBD 模块随 NovaPhy 一起构建。  
  - `python/bindings/CMakeLists.txt` 中将 `bind_vbd.cpp` 加入 `BIND_SOURCES`，并将 `_core` 目标链接到 `novaphy_vbd`。

- Python 绑定与导出：
  - `python/bindings/bind_vbd.cpp`  
    - 绑定 `VBDConfig`：导出所有字段，并提供简单的 `__repr__`。  
    - 绑定 `VBDWorld`：  
      - 构造函数 `__init__(Model, VBDConfig)`；  
      - 方法/属性：`step()`、`state`、`model`、`config`。  
      - 文档字符串中说明当前实现只是“重力 + 显式积分”的骨架，后续会被真正的 VBD/AVBD 求解替换。  
  - `python/bindings/bind_main.cpp` 中声明并调用 `bind_vbd(m)`；  
  - `python/novaphy/__init__.py` 中尝试导入 `VBDConfig`、`VBDWorld`，并在成功时把它们加入 `__all__`。

- 当天状态与下一步计划：
  - 当天结束时，已经可以在 Python 中写出如下代码并成功运行（使用占位求解逻辑）：  
    ```python
    import novaphy

    builder = novaphy.ModelBuilder()
    # build bodies/shapes/ground plane...
    model = builder.build()

    cfg = novaphy.VBDConfig()
    cfg.dt = 1.0 / 60.0
    world = novaphy.VBDWorld(model, cfg)
    world.step()
    state = world.state
    ```  
  - 计划下一步：  
    1. 在 `VbdSolver` 内部设计刚体与约束的数据结构（参考 `avbd-demo2d` 的 `Rigid` / `Force`）；  
    2. 将 AVBD 的迭代逻辑逐步迁移到 `VbdSolver::step` 中；  
    3. 在 `demos/` 下增加 `demo_vbd_stack.py`，与 `demo_stack.py` 做对比。

### 2026-03-04 —— 首个 3D VBD 风格求解器与 VBD Demo

- 将 `VbdSolver` 从“纯重力占位”改为简单的 3D VBD 风格接触求解：
  - 文件：`src/vbd/vbd_solver.h`  
    - 引入碰撞与接触相关头文件，以复用现有 3D 碰撞管线：  
      - `novaphy/collision/broadphase.h`  
      - `novaphy/collision/narrowphase.h`  
      - `novaphy/core/contact.h`  
    - 新增内部成员：  
      - `SweepAndPrune broadphase_;`  
      - `std::vector<ContactPoint> contacts_;`  
  - 文件：`src/vbd/vbd_solver.cpp`  
    - 将 `step` 改为三阶段流程：  
      1. **惯性预测（Inertial Prediction）**  
         - 对每个非静态刚体：  
           - 记录旧位置 `prev_positions[i]`；  
           - 线速度加上重力 `v += g * dt`；  
           - 显式积分位置 `x += v * dt`。  
      2. **接触迭代位置投影（Iterative Positional Projection）**  
         - 外层循环 `config_.iterations` 次：  
           - 使用当前 `state.transforms` 为每个 shape 构建 AABB；  
           - 调用 `broadphase_.update` 得到潜在碰撞对；  
           - 通过 `collide_shapes` 得到 `ContactPoint` 列表 `contacts_`，并用 `combine_friction`、`combine_restitution` 处理摩擦与恢复系数；  
           - 对每个有正穿透的接触点：  
             - 计算两侧刚体的逆质量和总逆质量 `invMassSum`；  
             - 沿接触法线方向计算修正向量 `correction = (penetration / invMassSum) * normal`；  
             - 对非静态刚体分别按 `invMassA`、`invMassB` 比例移动位置，静态刚体不移动。  
         - 这个步骤本质是一个简单的 3D VBD/PBD 风格 block update：逐步消除穿透，使盒子堆叠稳定。  
      3. **根据位置变化更新速度（Velocity from Position Difference）**  
         - 在全部位置投影结束后，对每个非静态刚体：  
           - 使用 BDF1 风格更新速度 `v = (x_new - x_prev) / dt`；  
         - 角速度与旋转暂时保持不变，后续引入完整 AVBD 旋转公式后再处理。
    - 总结：  
      - 现在的 `VbdSolver::step` 不再是“纯重力显式欧拉”，而是一个真正的 3D VBD 风格接触求解器雏形：  
        - 使用 NovaPhy 现有的 3D 碰撞检测（broadphase + narrowphase + `collide_shapes`）；  
        - 用多次迭代的位置投影来消除穿透，使刚体堆叠稳定；  
        - 最后根据位置差分更新线速度。  
      - 与论文里的完整 3D AVBD 相比，目前还缺少：  
        - 增广拉格朗日的 lambda/penalty 更新（没有引入对偶变量和 penalty 的自动调整）；  
        - 精确的几何刚度矩阵 G 和 J/H 的构造；  
        - 摩擦锥约束、转动自由度和关节约束等高级特性。  
      - 但这个版本已经非常适合作为后续把 AVBD 数学一条条“填进去”的基础框架。

- 新增 VBD Demo：`demos/demo_vbd_stack.py`  
  - 搭建与 `demo_stack.py` 类似的 3 盒子堆叠场景：  
    - `builder.add_ground_plane(y=0.0, ...)`；  
    - 循环创建 3 个 box，按 y 方向依次堆叠。  
  - 使用 `VBDWorld` 而不是 `World`：  
    ```python
    cfg = novaphy.VBDConfig()
    cfg.dt = 1.0 / 120.0
    cfg.iterations = 10
    cfg.gravity = np.array([0.0, -9.81, 0.0], dtype=np.float32)
    world = novaphy.VBDWorld(model, cfg)
    ```  
  - 先实现 headless 运行：固定步数循环 `world.step()`，定期打印每个刚体的 y 坐标，观察是否下落并最终堆叠稳定。  


### 2026-03-05 —— 3D AVBD 算法完善（增广拉格朗日 + 6-DOF 接触 + 旋转）

- 参考 Utah AVBD / PhysX_AVBD 思路，在 `VbdSolver` 中实现完整的 3D AVBD 流程：  
  - **约束数据结构**（`include/novaphy/vbd/vbd_solver.h`）：  
    - 新增 `AvbdContactConstraint`：每个接触点维护雅可比行 `jacobian_a`、`jacobian_b`（6-DOF spatial，Featherstone 顺序 [角; 线]）、有效质量 `effective_mass`、对偶变量 `lambda`、罚参数 `penalty`，以及切向有效质量/λ 预留（摩擦后续可加）。  
  - **6-DOF 接触雅可比与有效质量**（`build_contact_constraints`）：  
    - 接触约束 C = 穿透深度（正值表示穿透）；J_A = [r_a×n; -n]，J_B = [-r_b×n; n]。  
    - 有效质量 = 1 / (J_A^T W_A J_A + J_B^T W_B J_B)，其中 W 为 6×6 逆惯性（对角块 inv_I 与 inv_mass）。  
  - **增广拉格朗日 + 块下降**（`avbd_iteration`）：  
    - 位置级修正：`delta_lambda = effective_mass * C`，保证单步位移量级 ~C，数值稳定。  
    - 对偶更新：`lambda = max(0, lambda + penalty*C)`，用于下一帧 warmstart；可选 penalty 增长（gamma）。  
    - `apply_contact_correction`：对两体施加 delta_pos = inv_mass * J_lin * delta_lambda，delta_theta = inv_I_world * J_ang * delta_lambda，旋转步长限制 max_angle=0.1 rad 以防漂移。  
  - **惯性预测与速度更新**（`step`）：  
    - 预测：重力积分位置；角速度积分旋转（q *= quat(omega*dt)）。  
    - 多轮迭代：每轮重新 build 接触 + 一次 avbd_iteration。  
    - 速度由位形差分（BDF1）：线速度 = (x_new - x_prev)/dt；角速度由四元数差分 dq 导出 omega = (2/dt)*axis(dq)/sin(angle/2)。  
  - **配置与稳定性**：  
    - `VBDConfig::beta` 默认改为 1e3，避免过大 penalty 导致爆炸；旋转单步上限 0.1 rad。  
- Python：`python/novaphy/__init__.py` 中增加对 `VBDConfig`、`VBDWorld` 的导入与 `__all__` 导出，使 `import novaphy` 后可直接使用 VBD 接口。  
- 当前状态：`demos/demo_vbd_stack.py` 可稳定跑通；盒子先下落堆叠到 y≈[0.5,1.5,2.5]，若长时间步数后出现缓慢漂移可再调 max_angle 或角速度阻尼。

### 2026-03-06 —— 按 2D avbd-demo2d 补齐 λ/penalty 公式与参数

- 参考仓库 `/home/ydt/projects/avbd-demo2d/source/solver.{h,cpp}` 与 `maths.h`，补齐 3D AVBD 中对偶变量与罚参数的算法细节：  
  - **约束行参数映射**（`include/novaphy/vbd/vbd_solver.h`）：  
    - 在 `AvbdContactConstraint` 中新增 2D AVBD 中 `Force` 的核心字段：  
      - `C`：当前位置下的约束值（对接触即 `penetration`）；  
      - `fmin/fmax`：力空间下界/上界；  
      - `stiffness`：材料刚度（接触默认视作硬约束 `INF`）；  
      - `fracture`：断裂阈值（目前接触默认 `INF`，暂不禁用约束，仅保留接口）。  
  - **构建约束行时的初始化**（`build_contact_constraints`）：  
    - 对每个接触：  
      - 写入 `cc.C = cp.penetration`；  
      - `fmin = 0`、`fmax = +INF`、`stiffness = +INF`、`fracture = +INF`；  
      - 若 `penalty` 尚未初始化（`<=0`），则设为 `max(beta, kPenaltyMin)`，其中 `kPenaltyMin=1.0`、`kPenaltyMax=1e9`，与 2D demo 中的 `PENALTY_MIN/MAX` 保持同量级。  
    - geometry/J/有效质量部分保持之前的 6-DOF 实现（广义坐标 [角; 线]）。  
  - **Warmstart 与罚参数衰减**（`VbdSolver::step`）：  
    - 在惯性预测后、第一次 `build_contact_constraints` 之后，对所有 `AvbdContactConstraint` 做一次 warmstart：  
      - 若 `post_stabilize = true`：只衰减罚参数 `penalty = clamp(penalty * gamma, PENALTY_MIN, PENALTY_MAX)`，保留上一帧的 `lambda`；  
      - 否则：  
        - `lambda = lambda * alpha * gamma`（对偶变量按论文 Eq.19 衰减）；  
        - `penalty = clamp(penalty * gamma, PENALTY_MIN, PENALTY_MAX)`；  
      - 两种模式下都保证 `penalty <= stiffness`。  
    - 这一段逻辑直接对应 2D 版本 `Solver::step` 中的 “Initialize and warmstart forces”。  
  - **Primal + Dual 更新（位置级 + 对偶）**（`avbd_iteration`）：  
    - 为每个接触重新计算当前帧的 `C`、`J` 与 `effective_mass`（类似 2D 的 `computeConstraint` + `computeDerivatives`）。  
    - **Primal（位置级更新）**：  
      - 使用一阶近似：`delta_lambda = effective_mass * C`，通过 `apply_contact_correction` 将其转化为两刚体的 `delta_pos` 和 `delta_theta`，等价于一次 VBD 风格的块下降更新，使位置修正量级约为 C。  
    - **Dual（Eq.11 & Eq.16 对偶/罚参数更新）**：  
      - 视接触为硬约束：`is_hard = isinf(stiffness)`，此时 `lambda_eff = lambda`；  
      - Eq.11：  
        - `lambda_new = clamp(penalty * C + lambda_eff, fmin, fmax)`；  
        - 写回 `cc.lambda = lambda_new`；  
      - 断裂（fracture）：若 `|lambda| >= fracture`，当前只将 `penalty` 置 0，视作“软化为无效约束”，后续可以扩展为真正从接触集中移除；  
      - Eq.16：当 `lambda` 严格位于 `(fmin, fmax)` 区间时，增加罚参数以提高刚度：  
        - `penalty = min(penalty + beta * |C|, min(PENALTY_MAX, stiffness))`。  
    - 在 `step` 中的迭代结构调整为：  
      1. 惯性预测；  
      2. `build_contact_constraints` 一次 + warmstart；  
      3. for `iterations` 次：每轮 `build_contact_constraints`（更新几何）+ `avbd_iteration`；  
      4. 若 `post_stabilize` 再做一轮仅位置级/对偶校正。  
  - **结果**：  
    - 现在 3D AVBD 中 **λ/penalty 的更新公式、warmstart 与罚参数增长策略已与 2D avbd-demo2d 的 `Solver::step` 对齐**，只是 Primal 仍采用 6-DOF block-descent（不组装/求解 per-body 线性系统），几何刚度矩阵 G 也暂时未引入（二阶项仍在 TODO 中）。  

### 2026-03 —— 论文 Eq 18（Section 3.6）防爆炸修正与反弹抑制

- **文献**：Giles et al., *Augmented Vertex Block Descent*, ACM ToG 44(4), 2025 (SIGGRAPH 2025).
- **Section 3.6**：使用正则化约束避免单帧内过猛修正导致动量注入与反弹：  
  `C_eff = C*(x) - α·C*(x_t)`，其中 `C*(x_t)` 为本步初的约束误差。
- **实现**：  
  - 每接触增加 `C_at_step_start`，在首轮 `build_contact_constraints` 后写入当前 C。  
  - Primal/Dual 中统一使用 `C_eff = C - position_error_alpha * C_at_step_start`（默认 `position_error_alpha=0.95`）。  
  - 配置项：`VBDConfig::position_error_alpha`、`velocity_damping`、`angular_damping`（与 PhysX AVBD 一致）。
- **效果**：将“上一帧残留误差”的修正分摊到多帧，减轻单帧大修正 → 速度突增 → 反弹。

### 2026-03-06 —— 接触下沉修复 + 摩擦锥 + 自由刚体关节 + 几何刚度

- **接触符号约定对齐到 NovaPhy `ContactPoint`**：
  - `ContactPoint.normal` 定义为 **从 body A 指向 body B**，`penetration > 0` 表示重叠。
  - AVBD 接触约束统一使用：`C = penetration`，法向力/对偶变量 **非负** \(f \in [0,+\infty)\)。
  - 修复了此前由于符号不一致导致的堆叠 **“逐渐下沉/穿透累计”**（最底层穿过地面继续下落）。

- **接触行持久化 warmstart（降低接触行跳变导致的抖动）**：
  - 为每条接触行构建稳定 `key`（body pair + 量化接触点 + 量化法线）。
  - 在 `build_contact_constraints()` 中将历史 `lambda/penalty/C_at_step_start` 映射到新接触行，避免按数组下标 warmstart 的错配。

- **摩擦锥（Coulomb friction cone）接入 AVBD**：
  - 为每个接触点增加 2 个切向约束行（`tangent1/tangent2`），并执行库仑界投影：\(|f_t| \le \mu f_n\)。
  - 切向也使用增广拉格朗日的 `lambda_t1/lambda_t2` 与 `penalty_t1/penalty_t2`，并随接触 `key` 持久化 warmstart。

- **自由刚体硬关节（非 Featherstone articulation）**：
  - 新增 `RigidJointConstraint` 并加入 `Model`/`ModelBuilder`/Python bindings：
    - `Ball`：3 行（位置锚点一致）。
    - `Hinge`：5 行（位置 + 2 个角向约束；绕铰链轴自由旋转，铰链轴为关节帧的 +Z）。
    - `Fixed`：6 行（位置 + 角度）。
  - 关节行与接触行一起进入每体 6×6 primal 与 dual 更新。

- **几何刚度（Geometric stiffness）开关**：
  - 在接触法向行的角角块加入一个对称化的稳定项（可通过 `VBDConfig.geometric_stiffness`/`geometric_stiffness_scale` 控制），
    用于提升高质量堆叠与摩擦场景下的收敛/稳定性。

- **新增 demo**：
  - `demos/demo_vbd_joint_pair.py`：两个盒子通过 `ball/hinge/fixed` 连接，支持 headless 与 GUI，用于验证关节与摩擦。

### 2026-03-09 —— AVBD 按 avbd-demo3d 重写 + 稳定性修复（当前状态）

- **参考**：`/home/ydt/projects/avbd-demo3d`。按 demo3d 的 `Solver::step()`、Manifold 流程重写 VbdSolver，融入 NovaPhy 的 Model/SimState/碰撞管线。
- **已实现**：per-body 6×6 primal、3 维接触约束（法向 + 两切向）、C0/warmstart、BDF1 速度；`body_index == -1` 表示世界/静态；`VBDConfig` 含 `max_contacts_per_pair`（默认 8，与 demo 一致）、`iterations` 等。
- **当前效果**：Pyramid 与 Stack demo 在**地面为静态 box** 时均可稳定堆叠、静止不倾覆；地面为 plane 时金字塔仍可能静止后倾覆（根因见下节）。

#### 今日完成（工作日志）

1. **角部 RHS 与解算符号与 demo3d 对齐**  
   - 角部 RHS 改为与 demo 一致：`rot_err = (current - inertial)`，并传 `-rhsLin, -rhsAng` 给 6×6 求解，修正此前角部“力”方向反号导致的爆炸与乱转。
2. **角部统一为世界系**  
   - 角部未知数、Jacobian、旋转更新均按世界系处理，与 demo3d（`positionAng + dxAng` 左乘、无 body 系变换）一致；惯性块使用 body 对角 `MAng = body.inertia`。
3. **切向基与 demo3d 一致**  
   - `orthonormal_basis` 按 demo3d `maths.h` 的 `orthonormal(normal)` 实现：`t1 = (|n.x|>|n.z|) ? (-n.y,n.x,0) : (0,-n.z,n.y)`，`t2 = n×t1`，避免切向轴选取不同导致摩擦方向偏差。
4. **防爆安全限制移除**  
   - 移除单步位移/角位移 clamp 与角速度上限，仅保留与 demo 一致的公式，便于确认爆炸根因在角部符号。
5. **金字塔地面 plane ↔ box 对比与根因确认**  
   - 现象：地面为 plane 时金字塔静止后易向后倾覆，改为静态大 box 后不倾覆。  
   - 根因：plane 仅在穿透时生成接触，贴地时接触数 0～4 随帧抖动，支撑不稳定且 warmstart 易失效；box 地面为稳定 box–box manifold，接触数与支撑一致。  
   - 结论：需“静止不倾覆”时地面用静态 box；根因与对策已写入下文“地面 Plane 与 Box 的差异”。

### 与 avbd-demo3d 的差异与已知现象（简要）

- **差异参考**：地面可为 plane（`body_index=-1`）或静态大盒（与 demo 一致）；坐标系 Y-up vs Z-up；接触为 plane–box + box–box 或全 box–box。实现上已按 demo 的 C0、warmstart、primal/dual 顺序与公式对齐。

#### 地面 Plane 与 Box 的差异（金字塔倾覆根因，已确认）

- **现象**：金字塔在**平面地面（plane）**上静止一段时间后会向一侧（如“向后”）倾覆，甚至 2 层也会；换成**大 box 作为地面**后则不倾覆，与 avbd-demo3d 行为一致。
- **根因（简要）**：  
  1. **接触生成条件不同**  
     - **Plane**：`collide_box_plane` 仅在 `distance < 0`（角点穿透）时生成接触；贴地或略浮时 `distance ≥ 0`，接触数可为 0/1/2/3/4，随数值误差逐帧变化。  
     - **Box**：box–box 用 SAT + 面裁剪，得到稳定的一簇接触点（至多 8 个），参考面固定，接触数在静止时稳定。  
  2. **支撑稳定性**  
     - Plane 时接触数 flicker → 支撑多边形时有时无或不对称 → 合力/合力矩有微小偏差 → 多帧积累成可见倾覆。  
     - Box 地面时接触簇稳定、对称 → 支撑一致 → 无系统性力矩，静止不倾覆。  
  3. **Warmstart 与刚度**  
     - Plane 侧 `body_a=-1`，接触 key 仅依赖 box 侧；接触数变化时旧 key 对不上，warmstart 常失效，等效刚度在帧间跳动。  
     - Box–box 两侧都有稳定 body 与 feature，warmstart 持续生效，收敛更稳。
- **结论与建议**：  
  - **金字塔/堆叠等需要“长时间静止不倾覆”的场景**：地面用**静态大 box**（与 demo3d 一致），避免用 infinite plane。  
  - 若必须用 plane：需在 narrowphase 中为“贴地”增加 rest margin（如 `distance < ε` 也生成接触）并谨慎验证，或接受轻微倾覆/调参缓解。

### 实现要点（算法与符号，与 demo3d 对齐）

- **步骤**：每步建一次接触（broadphase + build_contact_constraints）；循环内 `update_contact_kinematics` → primal（per-body 6×6）→ dual；BDF1 速度；位置投影与接触法向速度归零、接触体阻尼。
- **Jacobian 符号**：`ContactPoint.normal` 从 A 指向 B；分离时 A 沿 -n、B 沿 +n，故 `jacobian_a` 线部 = -n、`jacobian_b` 线部 = +n；误用会穿地/爆炸。
- **Warmstart**：`lambda *= alpha*gamma`，`penalty = clamp(penalty*gamma, MIN, MAX)`；新接触 penalty 默认 0 再 clamp，由 dual 按 beta 增长。
- **穿透/振荡**：已通过 `position_error_alpha`、`iterations`、法向速度归零、接触阻尼（0.98）、可选 `post_stabilize` 与全局阻尼缓解。

### 当前实现是否为完整 AVBD？—— 已实现与节略项对照

**结论：当前已具备 AVBD 主框架（增广拉格朗日 + per-body 6×6 primal + Eq 18）并补齐了摩擦锥、自由刚体关节与几何刚度开关；与论文/PhysX 的差距主要在更完整的 manifold/接触缓存策略、更多关节类型/限位，以及更严格的能量控制/收敛策略。**

| 项目 | 完整 AVBD（论文/参考） | 当前 NovaPhy 实现 | 说明 |
|------|------------------------|-------------------|------|
| **惯性预测** | 位置 + 旋转显式积分 | ✅ 已实现 | 重力积分位置，角速度积分旋转 q*=quat(ω dt)，accelWeight 自适应 |
| **位置级约束 C(x)** | 接触不穿透 C≥0 | ✅ 已实现 | C = 穿透深度，仅法向 |
| **Eq 18 防爆炸修正** | C_eff = C − α·C(x_t) | ✅ 已实现 | position_error_alpha=0.95，每步存 C_at_step_start |
| **雅可比 J、有效质量 J^T W J** | 6-DOF 每接触 | ✅ 已实现 | J_A/J_B spatial，有效质量标量 |
| **对偶变量 λ、罚参数 penalty** | 每约束维护，warmstart | ✅ 已实现 | lambda、penalty 每接触，alpha/gamma 暖启动 |
| **对偶更新** | λ ← clamp(penalty·C+λ, fmin, fmax) | ✅ 已实现 | 使用 C_eff，penalty 增长仅当 λ 在界内 |
| **Primal 更新方式** | per-body 6×6 线性系统 H Δx = f | ✅ 已实现 | 组装 M/dt²+Σ J^T penalty J，解 LDLT，position += dq，旋转 cap |
| **旋转修正** | 6-DOF 块更新含旋转 | ✅ 已实现 | delta_θ = inv_I * J_ang * Δλ，步长 cap 0.1 rad |
| **速度更新** | 由位形差分（BDF1） | ✅ 已实现 | 线速度 (x_new−x_prev)/dt，角速度由四元数差分 |
| **摩擦锥** | 法向 + 切向摩擦约束 | ✅ 已实现 | 每接触 2 个切向行，\(|f_t| \le \mu f_n\) 投影，切向 warmstart |
| **关节约束（自由刚体）** | 球铰/铰链/固定等硬约束 | ✅ 已实现 | Ball(3)/Hinge(5)/Fixed(6) 作为 AVBD 约束行加入 6×6 求解 |
| **几何刚度 G / Hessian H** | 二阶项加速收敛（Section 3.5） | ✅ 部分实现 | 接触法向行加入一个可开关的几何刚度稳定项（近似项） |
| **GPU 并行** | 并行 per-body/per-contact | ❌ 未实现 | 单线程 CPU |

**简要总结：**

- **已实现：** 3D 刚体、per-body 6×6 primal（H Δx = f）、Eq 18 防爆炸修正（C_eff）、增广拉格朗日对偶（λ/penalty）、warmstart、penalty floor、BDF1 速度、velocity/angular damping、接触法向速度投影。
- **仍可优化：** manifold 聚合与接触缓存（减少接触行跳变）、更多关节类型/限位、几何刚度更完整形式、GPU 并行。

因此，当前实现已对齐论文 **Sections 3.1–3.2、3.6、3.7** 及算法 1 的主流程；反弹主要靠 **Eq 18** 与阻尼抑制。若需与论文/PhysX 完全一致，需补摩擦、关节与 G 项。

### 后续待办（向“更完整/更稳的 3D AVBD”继续靠拢）

- manifold/缓存：对 shape pair 生成稳定 manifold，多点接触与缓存点（进一步减少抖动）。  
- 关节扩展：限位、马达、更多轴约束，与 articulation 管线的统一接口。  
- 几何刚度：补更严格/更接近论文的形式，并评估对收敛的提升。  
- GPU 并行化：per-contact/body 块更新迁至 CUDA，数据布局与 CPU 一致便于对比。

---

## 5. 运行说明

本节用中文说明：一台新机器上，从环境准备 → 编译安装 → 运行不同求解器的 demo 的完整流程。团队同学只要按步骤执行，就可以复现当前的 World / IPCWorld / VBDWorld 三条管线。

### 5.1 环境准备

1. **克隆仓库**  
   ```bash
   cd ~/projects
   git clone --recurse-submodules <NovaPhy 仓库地址> NovaPhy
   cd NovaPhy
   ```

2. **安装 Miniconda / Conda（如果还没有）**  
   - 按官网说明安装 Miniconda 或 Anaconda，确保终端里能执行：  
     ```bash
     conda --version
     ```

3. **用 environment.yml 创建并激活环境**  
   ```bash
   cd ~/projects/NovaPhy
   conda env create -f environment.yml
   conda activate novaphy
   ```

4. **安装 vcpkg**  
   ```bash
   cd ~
   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   ./bootstrap-vcpkg.sh
   ```
   - 建议把 `VCPKG_ROOT` 写入 `~/.bashrc`（可选）：  
     ```bash
     echo 'export VCPKG_ROOT=$HOME/vcpkg' >> ~/.bashrc
     source ~/.bashrc
     ```

5. **安装 CUDA 12.8（团队都要用 IPC + GPU 时需要）**  
   - 按 NVIDIA 官方文档或前面提到的方式安装 CUDA Toolkit 12.8，确保：  
     ```bash
     nvcc --version   # 能看到 12.8.x
     ```  
   - 默认安装路径通常为 `/usr/local/cuda-12.8`。

### 5.2 构建与安装

> 说明：下面的命令默认 vcpkg 在 `~/vcpkg`，CUDA 在 `/usr/local/cuda-12.8`，并将 IPC（libuipc）和 VBD 模块一起编译进来。

**一键构建（IPC + CUDA，复制整段执行）：**

```bash
conda activate novaphy

export CMAKE_ARGS="-DNOVAPHY_WITH_IPC=ON \
    -DCMAKE_TOOLCHAIN_FILE=$HOME/vcpkg/scripts/buildsystems/vcpkg.cmake \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc"

export CMAKE_BUILD_PARALLEL_LEVEL=4

pip install -e .
```

分步说明（可选）：

1. **进入工程并激活环境**  
   ```bash
   cd ~/projects/NovaPhy
   conda activate novaphy
   ```

2. **设置 CMake 参数（IPC + vcpkg + CUDA）**  
   ```bash
   export CMAKE_ARGS="-DNOVAPHY_WITH_IPC=ON \
       -DCMAKE_TOOLCHAIN_FILE=$HOME/vcpkg/scripts/buildsystems/vcpkg.cmake \
       -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc"
   ```

3. **限制编译线程数（避免机器卡死）**  
   - 如果机器配置一般，可以先用 3 线程：  
     ```bash
     export CMAKE_BUILD_PARALLEL_LEVEL=3
     ```  
   - 如果是多核 + 大内存，可以尝试 4 或 5（例如）：  
     ```bash
     export CMAKE_BUILD_PARALLEL_LEVEL=4
     ```  
     如果发现编译时系统明显卡顿、甚至强制注销，就退回 3。

4. **安装 NovaPhy（开发模式，可重复执行）**  
   ```bash
   pip install -e .
   ```  
   - 这一步会：  
     - 编译 `novaphy_core`（C++17 核心库）；  
     - 编译 `novaphy_ipc` + libuipc（启用 IPC + CUDA）；  
     - 编译 `novaphy_vbd`（VBD/AVBD 模块）；  
     - 构建 Python 扩展模块 `_core` 并安装为 `novaphy` 包。

### 5.3 运行不同求解器的 Demo

> 所有命令都假设当前在 `~/projects/NovaPhy`，并已执行 `conda activate novaphy`。

#### 5.3.1 World（普通刚体 + PGS 接触）

- 典型 demo：3 盒子堆叠  
  ```bash
  python demos/demo_stack.py
  ```  
  - 有 Polyscope 时，会弹出 3D 可视化窗口；  
  - 没有 Polyscope 时，会 headless 跑若干步并在终端打印位置。

#### 5.3.2 IPCWorld（libuipc + CUDA 的 IPC 求解器）

- IPC box 堆叠 demo（需要本机已安装 CUDA 且构建时启用了 IPC）：  
  ```bash
  cd ~/projects/NovaPhy
  conda activate novaphy
  python demos/demo_ipc_stack.py
  ```  
  - **无需**再手动设置 `LD_LIBRARY_PATH` 或在项目根目录创建 libuipc 的 `.so` 软链接。  
  - 原因：工程已做两处永久修复：  
    1. **`python/novaphy/__init__.py`**：在 Linux 下，`import novaphy` 时会自动在 `build/*/Release/bin` 下查找并预加载所有 `libuipc*.so`（通过 `ctypes.CDLL(..., RTLD_GLOBAL)`），保证 `_core.so` 加载时能解析到 libuipc 及其后端。  
    2. **`src/ipc/ipc_world.cpp`**：在 Linux 下通过 `dladdr` 获取包含 `uipc::init` 的 .so 所在目录（即 `build/.../Release/bin`），将该路径作为 `module_dir` 传给 `uipc::init()`，这样 libuipc 会从该目录加载 `libuipc_backend_cuda.so`、`libuipc_sanity_check.so` 等插件，不再依赖当前工作目录或环境变量。  
  - 有 Polyscope 时会有 GUI，没有时会 headless 打印位姿。

#### 5.3.3 VBDWorld

- VBD 版 box 堆叠 demo（与 `demo_stack.py`、`demo_ipc_stack.py` 一样支持 3D 与 headless）：  
  ```bash
  python demos/demo_vbd_stack.py           # 有 Polyscope 时弹出 3D 窗口
  python demos/demo_vbd_stack.py --headless   # 仅终端打印
  ```  
  - 有 Polyscope 时：使用与 `demo_stack` 相同的 `SceneVisualizer`，实时显示 3 个盒子下落与堆叠。  
  - 无 Polyscope 或加 `--headless` 时：headless 运行，在终端打印每隔若干步的 y 坐标，例如：  
    ```text
    Running VBD box stack demo (headless)...
    step    0: y = ['1.499', '2.699', '3.899']
    step   40: y = ['0.913', '2.114', '3.314']
    ...
    step  239: y = [...]
    Done.
    ```  
  - 随着步数增加，盒子会在重力和接触投影作用下逐渐下落并堆叠，y 坐标达到一个相对稳定的高度，说明 VBD 管线已经工作正常。

### 5.4 团队协作中的注意事项

- 所有人在构建 IPC + VBD 版本时，最好统一使用相同的：  
  - CUDA 版本（建议 12.8）；  
  - vcpkg 路径（例如统一为 `$HOME/vcpkg`）；  
  - CMake 参数（`CMAKE_ARGS`）配置。  
- **Linux 下运行 IPC demo**：不需要再在 `~/.bashrc` 里加 `LD_LIBRARY_PATH`，也不需要在项目根目录建 libuipc 的 `.so` 软链接；`python/novaphy/__init__.py` 和 `src/ipc/ipc_world.cpp` 已做永久处理，只要从项目根目录执行 `python demos/demo_ipc_stack.py` 即可。  
- 如果某台机器没有 NVIDIA GPU，或者不想启用 IPC，可以临时关闭 IPC：  
  - 顶层 `CMakeLists.txt` 把 `NOVAPHY_WITH_IPC` 设为 `OFF`，或运行时不带 `-DNOVAPHY_WITH_IPC=ON`；  
  - 此时仍然可以使用 `World` 和 `VBDWorld`，只是不能跑 IPCWorld 相关 demo。  

---

## 6. VBD/AVBD 新增文件与目录总览（git 视角）

> 便于组会/文档中快速说明“这一轮 VBD/AVBD 重写具体加了哪些文件”。

### 6.1 头文件与 C++ 实现（核心求解器）

- **`include/novaphy/vbd/vbd_config.h`**  
  - 定义 `VBDConfig`，与 `avbd-demo3d` 的 `Solver` 参数一一对应：`dt`、`gravity`、`iterations`、`alpha`、`gamma`、`beta_linear`、`beta_angular`。  

- **`include/novaphy/vbd/vbd_solver.h`**  
  - 定义单点 3D 接触结构 `AvbdContact`（法向 + 两切向、`C0`、`lambda`、`penalty`、`stick`），以及 3D AVBD 求解器类 `VbdSolver`（per-body 6×6 primal + dual）。  

- **`include/novaphy/vbd/vbd_world.h`**  
  - 暴露给上层的 `VBDWorld` 包装类：持有 `Model`、`SimState` 与 `VbdSolver`，对 Python 侧只提供 `step()` / `state` 接口。  

- **`src/vbd/CMakeLists.txt`**  
  - 新增 VBD 子目录的 CMake 构建脚本，编译 `vbd_solver.cpp` / `vbd_world.cpp` 并链接到核心库。  

- **`src/vbd/vbd_solver.cpp`**  
  - AVBD 核心实现文件：  
    - `VbdSolver::step`：broadphase → `build_contact_constraints` → 惯性预测 / `initial` → 迭代 `avbd_primal` / `avbd_dual` → 位置投影 → BDF1 速度 → 法向速度投影 + 接触阻尼。  
    - `build_contact_constraints`：调用 `collide_shapes` 生成 `AvbdContact`，并做 **contact persistence**（用近似 key 复用上一帧的 `lambda` / `penalty` / `stick`，真正启用 warmstart）。  
    - `avbd_primal`：每个刚体组装 6×6 线性系统（质量 + 所有接触的 `J^T K J`），用 LDLT 解出 `dq` 并更新位姿。  
    - `avbd_dual`：按 demo3d 公式更新 `lambda` 与 `penalty`，包含摩擦锥与 `stick` 标记。  

- **`src/vbd/vbd_world.cpp`**  
  - `VBDWorld` 的 C++ 实现：持有 `Model`、`SimState`、`VbdSolver`，在 `step()` 里调用 AVBD 求解器并更新状态。  

### 6.2 Python 绑定与包导出

- **`python/bindings/bind_vbd.cpp`**  
  - 新增 pybind11 绑定：  
    - 导出 `novaphy.VBDConfig`（直接映射 `VBDConfig` 字段）。  
    - 导出 `novaphy.VBDWorld`（构造时接收 `Model` + `VBDConfig`，对 Python 暴露 `step()` 与 `state`）。  

- **`python/bindings/CMakeLists.cpp` / `python/bindings/bind_main.cpp` / `python/bindings/bind_sim.cpp`（修改）**  
  - 将 `bind_vbd.cpp` 纳入构建，并在主绑定入口里调用 `bind_vbd(m)`，把 VBD 接口注入到 Python 模块。  

- **`python/novaphy/__init__.py`（修改）**  
  - 从 C++ 扩展中 re-export `VBDConfig`、`VBDWorld`，在 Python 侧可直接：  
    - `cfg = novaphy.VBDConfig()`  
    - `world = novaphy.VBDWorld(model, cfg)`  

### 6.3 Demo 脚本（对标 avbd-demo3d 场景）

- **`demos/demo_vbd_stack.py`**  
  - VBD 版 box 堆叠 demo：对标 `demo_stack.py` 与 demo3d 的 `sceneStack`。  
  - 使用 `VBDWorld` 模拟 5 个 box 在地面上的堆叠（支持 GUI 与 `--headless` 打印 y 坐标）。  

- **`demos/demo_vbd_pyramid.py`**
  - VBD 版金字塔 demo：对标 avbd-demo3d 的 `scenePyramid`。
  - 场景：固定 `SIZE` 行金字塔（如 10 层）+ ground plane；用于压测接触收敛与稳定性。已知现象：静止后可能缓慢向后倾覆，见上文「已知现象：金字塔静止后向后倒塌」。
  - 提供 `--headless` 模式，打印全部刚体 y 坐标的摘要。

> 组会时可以直接引用以上小节，配合几张堆叠 / 金字塔的截图，就能清晰说明：  
> “这轮工作从 C++ 求解器（`src/vbd/vbd_solver.cpp`）、头文件、Python 绑定，到两个 demo，完整打通了一条 AVBD 流程，并对标了 avbd-demo3d 的 `sceneStack` / `scenePyramid` 场景。”
