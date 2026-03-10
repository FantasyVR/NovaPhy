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
        // 初始化状态缓冲区，与普通 World 保持一致：一体一 transform。
        state.init(model.num_bodies(), model.initial_transforms);
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

SimState& VBDWorld::state() { return impl_->state; }
const SimState& VBDWorld::state() const { return impl_->state; }

const Model& VBDWorld::model() const { return impl_->model; }

const VBDConfig& VBDWorld::config() const { return impl_->config; }

}  // namespace novaphy

