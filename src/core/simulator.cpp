#include "simulator.h"

namespace pe_core {

    void Simulator::init(const pe::Array<pe_phys_object::CollisionBody*>& objs) {
        for (auto& obj : objs) {
            _world.addCollisionBody(obj);
        }
    }

    SimulatorWithUi::SimulatorWithUi(const std::string& name) {

    }

} // namespace pe_core