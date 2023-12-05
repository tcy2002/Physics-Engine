#pragma once

#include "phys/public_include.h"
#include "common/vector3.h"
#include "phys/object/rigidbody.h"

namespace pe_core {

class World {
    PE_MEMBER_SET_GET(pe_common::Vector3, gravity, Gravity);
    PE_MEMBER_SET_GET(PEReal, time_step, TimeStep);

protected:
    pe::KVStore<uint32_t, pe_phys_object::CollisionBody*> _cbs;

public:
    World(): _gravity(0, -9.8, 0), _time_step(0.01) {}

    void addCollisionBody(pe_phys_object::CollisionBody* cb);
    void step();
};

} // namespace pe_core
