#include "simulator.h"
#include "ui/viewer.h"

using namespace pe_core;

void Simulator::init(const pe::Array<pe_phys_object::CollisionBody*>& objs) {
    for (auto& obj : objs) {
        _world.addCollisionBody(obj);
    }
}

SimulatorWithUi::SimulatorWithUi(const std::string& name) {
    pe_ui::Viewer::initViewerInstance(name, 800, 600)
    .setCamera({0, 0, 3}, 0, 0);
}
