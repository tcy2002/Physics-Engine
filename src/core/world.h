#pragma once

#include "phys/phys_general.h"
#include "common/vector3.h"
#include "phys/object/collision_body.h"
#include "phys/object/rigidbody.h"
#include "phys/collision/broad_phase/broad_phase.h"
#include "phys/collision/narrow_phase/narrow_phase.h"

namespace pe_core {

    class World {
        COMMON_MEMBER_SET_GET(pe::Vector3, gravity, Gravity);
        COMMON_MEMBER_SET_GET(pe::Real, time_step, TimeStep);

    protected:
        pe::Array<pe_phys_object::CollisionBody*> _cbs;
        pe_phys_collision::BroadPhaseBase* _broad_phase;
        pe_phys_collision::NarrowPhaseBase* _narrow_phase;

        void updateAABBs();
        void updateCollisionBodies();
        void applyExternalForce();

    public:
        World():
            _gravity(0, -9.8, 0), _time_step(0.01),
            _broad_phase(new pe_phys_collision::BroadPhaseSweepAndPrune),
            _narrow_phase(new pe_phys_collision::SimpleNarrowPhase) {}

        void addCollisionBody(pe_phys_object::CollisionBody* cb);
        void step();
    };

} // namespace pe_core
