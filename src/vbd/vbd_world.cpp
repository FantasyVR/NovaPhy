#include "novaphy/vbd/vbd_world.h"

#include "novaphy/vbd/vbd_solver.h"

namespace novaphy {

struct VBDWorld::Impl {
    Model model;
    VBDConfig config;
    SimState state;
    VbdSolver solver;

    Impl(const Model& m, const VBDConfig& cfg)
        : model(m), config(cfg), solver(cfg) {
        // Initialize state buffers from the model's initial transforms.
        state.init(model.initial_transforms);
        solver.set_model(model);
    }

    void step_one() {
        solver.step(model, state);
    }
};

VBDWorld::VBDWorld(const Model& model, const VBDConfig& config)
    : impl_(std::make_unique<Impl>(model, config)) {}

VBDWorld::~VBDWorld() = default;

VBDWorld::VBDWorld(VBDWorld&&) noexcept = default;
VBDWorld& VBDWorld::operator=(VBDWorld&&) noexcept = default;

void VBDWorld::step() {
    impl_->step_one();
}

void VBDWorld::clear_forces() { impl_->solver.clear_forces(); }

void VBDWorld::add_ignore_collision(int body_a, int body_b) {
    impl_->solver.add_ignore_collision(body_a, body_b);
}

int VBDWorld::add_joint(int body_a, int body_b,
                        const Vec3f& rA, const Vec3f& rB,
                        float stiffnessLin, float stiffnessAng, float fracture) {
    return impl_->solver.add_joint(body_a, body_b, rA, rB, stiffnessLin, stiffnessAng, fracture);
}

int VBDWorld::add_spring(int body_a, int body_b,
                         const Vec3f& rA, const Vec3f& rB,
                         float stiffness, float rest) {
    return impl_->solver.add_spring(body_a, body_b, rA, rB, stiffness, rest);
}

SimState& VBDWorld::state() { return impl_->state; }
const SimState& VBDWorld::state() const { return impl_->state; }

const Model& VBDWorld::model() const { return impl_->model; }

const VBDConfig& VBDWorld::config() const { return impl_->config; }

}  // namespace novaphy

