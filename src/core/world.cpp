#include "world.h"

using namespace pe_core;

void World::addCollisionBody(pe_phys_object::CollisionBody* cb) {
    _cbs.insert_or_assign(cb->getGlobalId(), cb);
}

void World::step() {
    for (auto& cb : _cbs) {
        if (!cb.second->isActive() || cb.second->isKinematic()) continue;
        cb.second->applyForce(_gravity, cb.second->getTransform().getOrigin(), _time_step);
    }

    for (auto& cb : _cbs) {
        cb.second->updateTransform(_time_step);
    }
}
