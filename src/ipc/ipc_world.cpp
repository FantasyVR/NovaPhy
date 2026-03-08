#include "novaphy/ipc/ipc_world.h"
#include "novaphy/ipc/shape_converter.h"

// libuipc headers stay in the implementation file to avoid leaking them into
// public NovaPhy headers.
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/geometry/utils.h>
#include <uipc/common/unit.h>

#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <algorithm>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#if defined(__linux__)
#include <dlfcn.h>
#endif

namespace novaphy {

// ---- Pimpl implementation ----

struct IPCWorld::Impl {
    Model model;
    IPCConfig config;
    SimState state;
    int frame_count = 0;

    // libuipc objects
    std::unique_ptr<uipc::core::Engine> engine;
    std::unique_ptr<uipc::core::World> world;
    std::unique_ptr<uipc::core::Scene> scene;

    // Resolved contact resistance, computed once during init().
    uipc::Float resolved_kappa = 0.0;

    // Track which NovaPhy body index maps to which libuipc geometry slot ID.
    struct BodyMapping {
        int novaphy_body_idx;
        uipc::IndexT geometry_id;
    };
    std::vector<BodyMapping> body_mappings;

    void init();
    void convert_bodies();
    void advance_and_retrieve();
};

// ---- Helper: convert NovaPhy Transform to libuipc Matrix4x4 ----

static uipc::Matrix4x4 to_uipc_transform(const Transform& tf) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot = tf.rotation.cast<double>().toRotationMatrix();
    m.block<3, 3>(0, 0) = rot;
    m(0, 3) = static_cast<double>(tf.position.x());
    m(1, 3) = static_cast<double>(tf.position.y());
    m(2, 3) = static_cast<double>(tf.position.z());
    return m;
}

// ---- Helper: extract NovaPhy Transform from libuipc Matrix4x4 ----

static Transform from_uipc_transform(const uipc::Matrix4x4& m) {
    Eigen::Matrix3d rot = m.block<3, 3>(0, 0);
    Eigen::Quaterniond qd(rot);
    Vec3f pos(static_cast<float>(m(0, 3)),
              static_cast<float>(m(1, 3)),
              static_cast<float>(m(2, 3)));
    Quatf qf(static_cast<float>(qd.w()),
              static_cast<float>(qd.x()),
              static_cast<float>(qd.y()),
              static_cast<float>(qd.z()));
    return Transform(pos, qf);
}

static bool is_positive_finite(float v) {
    return std::isfinite(v) && v > 0.0f;
}

// Backward-compatible contact stiffness resolution:
// - `kappa` is the primary IPC barrier/contact stiffness knob.
// - `contact_resistance` is treated as an explicit override when customized.
static uipc::Float resolve_contact_resistance(const IPCConfig& cfg) {
    using uipc::Float;
    constexpr float kDefaultContactResistance = 1e9f;
    constexpr float kDefaultKappa = 1e8f;
    constexpr float kRelEps = 1e-5f;

    const bool has_custom_contact_resistance =
        is_positive_finite(cfg.contact_resistance) &&
        std::abs(cfg.contact_resistance - kDefaultContactResistance) >
            (kDefaultContactResistance * kRelEps);

    if (has_custom_contact_resistance) {
        return static_cast<Float>(cfg.contact_resistance);
    }
    if (is_positive_finite(cfg.kappa)) {
        return static_cast<Float>(cfg.kappa);
    }
    if (is_positive_finite(cfg.contact_resistance)) {
        return static_cast<Float>(cfg.contact_resistance);
    }
    return static_cast<Float>(kDefaultKappa);
}

void IPCWorld::Impl::init() {
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;

    // Initialize libuipc with module_dir pointing to backend shared objects.
    // Find the directory containing uipc_core by querying the loaded library path.
    std::string module_dir;
#ifdef _WIN32
    HMODULE hmod = GetModuleHandleA("uipc_core.dll");
    if (hmod) {
        char path[MAX_PATH];
        if (GetModuleFileNameA(hmod, path, MAX_PATH) > 0) {
            module_dir = std::filesystem::path(path).parent_path().string();
        }
    }
#elif defined(__linux__)
    // On Linux use dladdr to find the shared object that contains
    // the uipc symbols (e.g. libuipc_core.so) and use its directory.
    Dl_info dlinfo;
    // Get address of a uipc symbol (function) and query its shared object.
    void* fn_addr = reinterpret_cast<void*>(reinterpret_cast<void (*)()>(&uipc::init));
    if (dladdr(fn_addr, &dlinfo) != 0 && dlinfo.dli_fname) {
        module_dir = std::filesystem::path(dlinfo.dli_fname).parent_path().string();
    }
#endif

    if (module_dir.empty()) {
        module_dir = ".";
    }

    // Set the DLL search path before uipc::init() so that any DLLs loaded
    // during initialization (sanity checks, backends with sibling deps) can
    // resolve their transitive dependencies.
#ifdef _WIN32
    char prev_dll_dir[MAX_PATH] = {};
    GetDllDirectoryA(MAX_PATH, prev_dll_dir);
    if (!module_dir.empty() && module_dir != ".") {
        SetDllDirectoryA(module_dir.c_str());
    }
#endif

    {
        auto uipc_cfg = uipc::default_config();
        uipc_cfg["module_dir"] = module_dir;
        uipc::init(uipc_cfg);
    }

    // Create engine
    engine = std::make_unique<Engine>(config.backend, config.workspace);

    // Create world
    world = std::make_unique<World>(*engine);

    // Configure scene
    auto scene_config = Scene::default_config();
    scene_config["gravity"] = Vector3{
        static_cast<double>(config.gravity.x()),
        static_cast<double>(config.gravity.y()),
        static_cast<double>(config.gravity.z())
    };
    scene_config["dt"] = static_cast<double>(config.dt);
    scene_config["contact"]["d_hat"] = static_cast<Float>(
        is_positive_finite(config.d_hat) ? config.d_hat : 0.01f);

    resolved_kappa = resolve_contact_resistance(config);
    scene_config["contact"]["adaptive"]["min_kappa"] = resolved_kappa;
    scene_config["contact"]["adaptive"]["init_kappa"] = resolved_kappa;
    scene_config["contact"]["adaptive"]["max_kappa"] = resolved_kappa;

    scene_config["newton"]["max_iter"] =
        static_cast<IndexT>(std::max(1, config.newton_max_iter));
    const Float newton_tol = static_cast<Float>(
        is_positive_finite(config.newton_tol) ? config.newton_tol : 1e-2f);
    scene_config["newton"]["velocity_tol"] = newton_tol;
    scene_config["newton"]["transrate_tol"] = newton_tol;

    scene = std::make_unique<Scene>(scene_config);

    // Initialize NovaPhy SimState
    state.init(model.initial_transforms);

    // Convert bodies to libuipc
    convert_bodies();

    // Init the libuipc world with the scene
    world->init(*scene);

#ifdef _WIN32
    // Restore previous DLL search directory to avoid leaking global state.
    SetDllDirectoryA(prev_dll_dir[0] ? prev_dll_dir : nullptr);
#endif
}

void IPCWorld::Impl::convert_bodies() {
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    // Setup constitution
    AffineBodyConstitution abd;
    scene->constitution_tabular().insert(abd);

    // Setup default contact model
    scene->contact_tabular().default_model(static_cast<Float>(config.friction),
                                           resolved_kappa);
    auto default_element = scene->contact_tabular().default_element();

    // Group shapes by body_index
    // body_index -> vector of shape indices
    std::unordered_map<int, std::vector<int>> body_shapes;
    for (int si = 0; si < model.num_shapes(); ++si) {
        const auto& shape = model.shapes[si];
        if (shape.type == ShapeType::Plane) {
            // Handle planes separately as implicit geometry
            auto ground_geo = ground(
                static_cast<Float>(shape.plane.offset),
                Vector3{
                    static_cast<double>(shape.plane.normal.x()),
                    static_cast<double>(shape.plane.normal.y()),
                    static_cast<double>(shape.plane.normal.z())
                }
            );
            auto obj = scene->objects().create("ground_plane");
            obj->geometries().create(ground_geo);
            continue;
        }
        body_shapes[shape.body_index].push_back(si);
    }

    // Convert each body
    for (int bi = 0; bi < model.num_bodies(); ++bi) {
        const auto& body = model.bodies[bi];
        const auto& transform = model.initial_transforms[bi];

        auto it = body_shapes.find(bi);
        if (it == body_shapes.end()) continue;  // body has no shapes

        // Merge all shapes for this body into a single tetmesh.
        // Each shape's vertices are transformed by its local_transform
        // before merging, so the combined mesh is in body-local space.
        std::vector<Vector3> verts;
        std::vector<Vector4i> tets;

        for (int shape_idx : it->second) {
            const auto& shape = model.shapes[shape_idx];

            TetMeshData tet_data;
            switch (shape.type) {
                case ShapeType::Box:
                    tet_data = box_to_tetmesh(shape.box.half_extents);
                    break;
                case ShapeType::Sphere:
                    tet_data = sphere_to_tetmesh(shape.sphere.radius, 1);
                    break;
                default:
                    continue;  // skip unknown types
            }

            // Apply shape's local transform to vertices
            Eigen::Matrix3d local_rot = shape.local_transform.rotation.cast<double>().toRotationMatrix();
            Eigen::Vector3d local_pos = shape.local_transform.position.cast<double>();

            int vert_offset = static_cast<int>(verts.size());
            for (const auto& v : tet_data.vertices) {
                verts.push_back(local_rot * v + local_pos);
            }
            for (const auto& t : tet_data.tetrahedra) {
                tets.push_back(Vector4i(t[0] + vert_offset, t[1] + vert_offset,
                                        t[2] + vert_offset, t[3] + vert_offset));
            }
        }

        if (verts.empty()) continue;  // all shapes were unsupported types

        // Compute merged tetmesh volume in body-local space so we can map each
        // body's mass to density before applying the IPC constitution.
        double mesh_volume = 0.0;
        for (const auto& t : tets) {
            const Vector3& p0 = verts[t[0]];
            const Vector3& p1 = verts[t[1]];
            const Vector3& p2 = verts[t[2]];
            const Vector3& p3 = verts[t[3]];
            const double signed_volume =
                (p1 - p0).dot((p2 - p0).cross(p3 - p0)) / 6.0;
            mesh_volume += std::abs(signed_volume);
        }

        // Create simplicial complex
        SimplicialComplex mesh = tetmesh(verts, tets);

        // Apply constitution (affine body)
        const Float body_kappa = static_cast<Float>(config.body_kappa);
        float density_value = is_positive_finite(config.mass_density)
                                  ? config.mass_density
                                  : 1e3f;

        if (!body.is_static()) {
            constexpr double kVolumeEps = 1e-12;
            if (mesh_volume > kVolumeEps && is_positive_finite(body.mass)) {
                density_value = static_cast<float>(
                    static_cast<double>(body.mass) / mesh_volume);
            }
        }
        const Float density = static_cast<Float>(density_value);
        abd.apply_to(mesh, body_kappa, density);

        // Apply contact model
        default_element.apply_to(mesh);

        // Label surface for contact detection
        label_surface(mesh);
        label_triangle_orient(mesh);

        // Set initial transform via the instance transforms attribute
        {
            auto& xforms = mesh.transforms();
            auto xform_view = view(xforms);
            xform_view[0] = to_uipc_transform(transform);
        }

        // Mark static bodies (mass <= 0)
        if (body.is_static()) {
            auto is_fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
            auto fixed_view = view(*is_fixed);
            fixed_view[0] = 1;
        }

        // Create libuipc object
        std::string obj_name = "body_" + std::to_string(bi);
        auto obj = scene->objects().create(obj_name);
        auto geo_slots = obj->geometries().create(mesh);

        // Record mapping for state retrieval
        BodyMapping mapping;
        mapping.novaphy_body_idx = bi;
        mapping.geometry_id = obj->geometries().ids()[0];
        body_mappings.push_back(mapping);
    }
}

void IPCWorld::Impl::advance_and_retrieve() {
    using namespace uipc;
    using namespace uipc::geometry;

    // Advance IPC simulation
    world->advance();
    world->sync();
    world->retrieve();

    frame_count++;

    // Read back transforms and compute velocities via finite difference
    float inv_dt = 1.0f / config.dt;

    for (const auto& mapping : body_mappings) {
        auto geo_slots = scene->geometries().find(mapping.geometry_id);
        if (!geo_slots.geometry) continue;

        auto* sc = dynamic_cast<SimplicialComplex*>(&geo_slots.geometry->geometry());
        if (!sc) continue;

        const auto& xforms = sc->transforms();
        auto xform_view = xforms.view();
        if (xform_view.empty()) continue;

        int bi = mapping.novaphy_body_idx;
        Transform prev = state.transforms[bi];
        Transform curr = from_uipc_transform(xform_view[0]);
        state.transforms[bi] = curr;

        // Linear velocity: finite difference of position
        state.linear_velocities[bi] = (curr.position - prev.position) * inv_dt;

        // Angular velocity: from quaternion difference
        // omega = 2 * (q_curr * q_prev^-1 - identity) / dt  (small angle approx)
        Quatf dq = curr.rotation * prev.rotation.inverse();
        // Ensure shortest path
        if (dq.w() < 0.0f) {
            dq.coeffs() = -dq.coeffs();
        }
        state.angular_velocities[bi] = Vec3f(dq.x(), dq.y(), dq.z()) * (2.0f * inv_dt);
    }
}

// ---- IPCWorld public interface ----

IPCWorld::IPCWorld(const Model& model, const IPCConfig& config)
    : impl_(std::make_unique<Impl>())
{
    impl_->model = model;
    impl_->config = config;
    impl_->init();
}

IPCWorld::~IPCWorld() = default;

IPCWorld::IPCWorld(IPCWorld&&) noexcept = default;
IPCWorld& IPCWorld::operator=(IPCWorld&&) noexcept = default;

void IPCWorld::step() {
    impl_->advance_and_retrieve();
}

SimState& IPCWorld::state() { return impl_->state; }
const SimState& IPCWorld::state() const { return impl_->state; }

const Model& IPCWorld::model() const { return impl_->model; }

const IPCConfig& IPCWorld::config() const { return impl_->config; }

int IPCWorld::frame() const { return impl_->frame_count; }

}  // namespace novaphy
