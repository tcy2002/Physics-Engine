#include "simulator.h"

namespace pe_core {

    void Simulator::init(const pe::Array<pe_phys_object::CollisionBody*>& objs) {
        for (auto& obj : objs) {
            _world.addCollisionBody(obj);
        }
    }

    SimulatorWithUi::SimulatorWithUi(const std::string& name) {
        simple_viewer::OpenglViewer::open(name);
    }

} // namespace pe_core