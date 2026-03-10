# AVBD 集成进度总结与逻辑/复用检查

本文档总结当前 NovaPhy 中 AVBD 集成的进度，并判断已写逻辑与对现有模块的复用是否正确、是否存在问题。

---

## 一、当前集成进度总结

### 1.1 已完成的模块与职责

| 模块 | 文件 | 状态 | 说明 |
|------|------|------|------|
| **VBDWorld** | `src/vbd/vbd_world.cpp` | ✅ 完成 | 持有 Model、SimState、VbdSolver；`step()` 仅调用 `solver.step(model, state)`，与 World/IPCWorld 用法一致 |
| **VbdSolver** | `src/vbd/vbd_solver.cpp` | ✅ 完成 | 单步流程：存 initial → 建接触(C0/warmstart) → 惯性预测 → primal/dual 迭代 → BDF1 速度 |
| **接触构建** | `build_contact_constraints()` | ✅ 完成 | 复用 broadphase + narrowphase → ContactPoint，转 AvbdContact，contact key 持久化 warmstart |
| **Primal** | `avbd_primal()` | ✅ 完成 | 每体 6×6 (M/dt² + J^T K J)，LDLT 求解，位置/旋转更新，摩擦锥投影 |
| **Dual** | `avbd_dual()` | ✅ 完成 | λ←F，penalty 增长，stick 标记 |
| **配置与绑定** | VBDConfig、bind_vbd、bind_main | ✅ 完成 | Python 可构造 VBDConfig、VBDWorld，bind_main 中已调用 bind_vbd(m) |
| **Demo** | demo_vbd_stack.py、demo_vbd_pyramid.py | ✅ 完成 | 与 demo_stack / demo_ipc_stack 同风格，支持 GUI 与 headless，适配 SceneVisualizer |

### 1.2 与 avbd-demo3d 的对应关系

- **流程**：建接触 → 惯性位姿 + initial → 主循环(primal → dual) → BDF1 速度，与 demo3d 的 `Solver::step()` 一致。
- **符号**：法向取 B→A（basis 行0 = -n），C0 = basis*(xA-xB) + (margin,0,0)，法向力 F[0]≤0（支撑力），与文档和 demo3d 约定一致。
- **Warmstart**：λ *= alpha*gamma，penalty = clamp(penalty*gamma, MIN, MAX)；新接触 penalty 从 0 clamp 到 PENALTY_MIN，逻辑正确。

### 1.3 尚未实现或可后续增强的部分

- 关节约束（RigidJointConstraint）未在本次 VbdSolver 中接入（详见下节「关节：能否复用老师的 Joint？」）。
- 位置投影与接触法向速度归零、post_stabilize 等稳定化在文档中有描述，当前 step 中未再做单独一轮。
- 几何刚度更完整形式、GPU 并行等为后续优化项。

### 1.4 关节：能否复用老师的 Joint？需要自己写吗？

结论先说：**老师的 Joint 是另一套管线（多体树 + 广义坐标），不能直接给 VBD 用；RigidJointConstraint 目前代码里不存在，是文档里的规划。要做 VBD 关节，需要（1）在 Model 里加“两体之间的关节”描述，（2）在 VbdSolver 里自己写约束行 C/J 并接入 primal/dual。**

- **老师已有的关节**  
  - 在 **`core/joint.h`** 和 **`core/articulation.h`**：`Joint` + `JointType`（Revolute / Fixed / Free / Slide / Ball），用于 **Articulation**（多体树）。  
  - 每条 link 一个 joint 连到 parent，状态是 **广义坐标 (q, qd)**，用 Featherstone 的 `joint_transform(q)`、`motion_subspace` 等，走的是 **ArticulatedSolver**（如 `demo_joint_chain.py`），不是 `Model` + `World`/`VBDWorld`。  
  - **Model**（给 World/VBDWorld 用的）里只有 `bodies`、`initial_transforms`、`shapes`，**没有**任何关节或 RigidJointConstraint。

- **RigidJointConstraint 是什么**  
  - 只在 **文档**（如 PROJECT_STRUCTURE_AND_VBD_DEV.md）里作为规划出现：“新增 RigidJointConstraint 并加入 Model/ModelBuilder”。  
  - 含义是：**两个自由刚体**之间的一条约束（Ball=3 行、Hinge=5 行、Fixed=6 行），在 AVBD 里和接触一样作为约束行参与 primal/dual。  
  - 当前代码里 **没有** RigidJointConstraint 类型，也没有在 Model/ModelBuilder 里存“body A 和 body B 之间一个关节”的结构。

- **能不能复用、要不要自己写**  
  - **不能**直接把老师的 `Joint`/`Articulation` 拿来给 VBD 用：  
    - 数据不在 Model 里，而在 Articulation 里；  
    - 表述方式不同：Articulation 是树 + 广义坐标，VBD 是笛卡尔位姿 + 约束 C(x)=0。  
  - **可以复用**的只有 **关节类型语义**（Ball/Hinge/Fixed 的命名和约束个数），例如在 Model 里加一种“两体关节”描述时沿用 `JointType` 或相同命名。  
  - **需要你（或和老师一起）做的**：  
    1. **Model 层**：在 Model/ModelBuilder 里增加“两体之间的关节”描述（例如：body_a、body_b、关节类型、锚点/轴等），并暴露给 Python。若老师已有或计划做这部分，就复用接口，否则需要新增。  
    2. **VbdSolver 层**：根据这些关节描述，在 AVBD 里 **自己写**约束的 C(x) 和雅可比 J（笛卡尔位姿下，例如 Ball：两点重合；Hinge：两点重合 + 绕轴旋转自由等），并像接触一样加入 `avbd_primal`/`avbd_dual`。老师现有的 `Joint::joint_transform(q)` 等是广义坐标形式，不能直接当 AVBD 的约束行用。

因此：**不是“老师已经写好了 RigidJointConstraint 你直接复用”，而是“老师写的是另一套树形关节；若要在 VBD 里支持两体关节，需要在 Model 里加描述 + 在 VbdSolver 里自己实现约束行”，关节类型名可以沿用老师的 Ball/Hinge/Fixed。**

---

## 二、复用的正确性判断

### 2.1 正确复用的部分 ✅

1. **碰撞管线**
   - 使用 `SweepAndPrune::update(shape_aabbs, shape_static)` 与 `get_pairs()`，对返回的 pair 按 **shape 索引** 使用 `model.shapes[pair.body_a/b]`，与 `World::step()` 一致，正确。
   - 使用 `collide_shapes(sa, ta, sb, tb, new_contacts)`，得到 `ContactPoint` 列表；narrowphase 已正确填写 `body_a`、`body_b`、`feature_id`（box-box 等），复用正确。

2. **接触与摩擦**
   - 使用 `core/contact.h` 的 `ContactPoint`、`combine_friction(sa.friction, sb.friction)`，不依赖 `ContactPoint.friction` 字段，摩擦系数来自 shape，正确。
   - 法向约定：NovaPhy 的 normal 为 A→B、penetration>0 表示穿透；你在 AVBD 侧用 `basis = orthonormal_basis(-n)` 使行0 为 B→A，与 C0 和 F(0)≤0 的约定一致，正确。

3. **场景与状态**
   - 使用同一套 `Model`（bodies、shapes、initial_transforms）和 `SimState`（transforms、linear/angular_velocities），与 World 一致，正确。

4. **Python 与绑定**
   - `bind_main.cpp` 中已调用 `bind_vbd(m)`；Model 的 `num_bodies` 以只读属性暴露，demo 中 `model.num_bodies` 用法正确。

### 2.2 需要留意或可改进的点

1. **惯性张量坐标系（建议后续统一）**
   - `RigidBody::inertia` 在 `body.h` 中明确为 **body-frame** 惯性张量。
   - `avbd_primal` 中角部使用世界系小角增量 `dqAng`，但当前 `MAng = body.inertia` 未做旋转到世界系。
   - 严格做法应为 `I_world = R * I_body * R^T`（R 为当前旋转）。若 demo3d 使用“对角 moment”且角步长小，短期可能不明显，但为与 Featherstone/多体一致，建议改为世界系惯性或显式注明假设。

2. **静态/世界体的 rA/rB**
   - 静态或世界体（body_index==-1）时，`ac.rA`/`ac.rB` 存的是世界系位置（pA/pB），在 primal 中 `rA_w`/`rB_w` 对 world 直接取 `ac.rA`/`ac.rB`，不乘旋转，与“不迭代静态体”一致，当前用法正确。

### 2.3 AVBD 接触用到的 `include/novaphy/core` 一览

你写 AVBD 接触时，**用到的** core 里的只有下面这些；**其他 core 头文件接触管线不需要**。

| core 头文件 | 是否用于 AVBD 接触 | 用途 |
|-------------|--------------------|------|
| **contact.h** | ✅ 用 | `ContactPoint`（narrowphase 输出）、`combine_friction()`（摩擦系数合并） |
| **model.h** | ✅ 用 | `Model`（bodies、shapes、num_bodies、num_shapes） |
| **body.h** | ✅ 用（经 model.h） | `RigidBody`（mass、inertia、is_static、inv_mass 等，primal 里 M 与惯性） |
| **shape.h** | ✅ 用（经 model.h + narrowphase） | `CollisionShape`（type、body_index、compute_aabb、friction、box/sphere/plane） |
| **aabb.h** | ✅ 用（经 broadphase.h + shape） | `AABB`（broadphase 输入、`CollisionShape::compute_aabb` 返回） |
| **joint.h** | ❌ 不用 | Articulation 树形关节，非自由刚体接触 |
| **articulation.h** | ❌ 不用 | 多体树 + Featherstone，非 Model/接触管线 |
| **model_builder.h** | ❌ 接触不用 | 建场景时用（Python/demo），求解器内不读 |

**总结**：AVBD 接触只依赖 **contact.h、model.h、body.h、shape.h、aabb.h**；**joint.h、articulation.h、model_builder.h** 接触不需要。

3. **新接触的 penalty 初值**
   - 新接触未命中 old_cache 时，`ac.penalty` 默认零向量，随后执行 `ac.penalty.x() = clampf(ac.penalty.x()*gamma, PENALTY_MIN, PENALTY_MAX)` 等，得到 PENALTY_MIN，逻辑正确。

---

## 三、逻辑正确性判断

### 3.1 step() 顺序

- 先保存 `initial_positions_` / `initial_rotations_`（本步初状态）。
- 再 `build_contact_constraints(model, state)`，此时 state 仍是步初，C0 与 warmstart 基于步初位姿，正确。
- 再写惯性预测到 `state.transforms`，然后进行 primal/dual 迭代，最后 BDF1 更新速度，顺序与 demo3d 一致，正确。

### 3.2 Primal 中的 C 与 RHS

- C 用 `C0*(1-alpha) + J*dq` 形式，与 Eq.18 防爆炸修正一致。
- RHS 为 `M/dt²*(pos - inertial)`（线部）与 `MAng/dt²*(rot_err)`（角部），再取负号进求解器，与“惯性 + 约束反力”的写法一致。

### 3.3 摩擦锥

- 法向 F(0)≤0；切向用 `|f_t| ≤ μ|f_n|` 做缩放投影，与库仑锥一致；stick 在 dual 中根据切向位移阈值设置，逻辑正确。

### 3.4 正则化（已移除，与 demo3d 一致）

- **LDLT 失败时**：demo3d 不做正则化；此前在 LHS 对角加 reg 并重算会改变解、可能引入额外力矩导致微抖动或金字塔“向后倾覆”。已改为失败时直接 `dq.setZero()`，不再对 LHS 做任何正则化。

---

## 四、结论表

| 项目 | 结论 |
|------|------|
| 整体流程与 demo3d 对齐 | ✅ 是 |
| 碰撞/接触/摩擦复用 | ✅ 正确（broadphase、narrowphase、ContactPoint、combine_friction） |
| 法向与约束符号 | ✅ 正确（A→B + basis=-n + F(0)≤0） |
| Warmstart 与 contact key | ✅ 正确 |
| 惯性张量坐标系 | ⚠️ 当前用 body-frame，角部为世界系时可考虑改为 I_world |
| Python 与 demo | ✅ 正确（num_bodies 为属性，bind_vbd 已注册） |

**总体结论**：当前 AVBD 集成在流程、复用手法和约束符号上正确；主要建议是后续将角部惯性统一为世界系（或明确文档假设），其余为可选稳定化与性能增强。
