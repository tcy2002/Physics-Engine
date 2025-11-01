#pragma once

#include "physics/physics.h"
#include "physics/collision/broad_phase/broad_phase_base.h"
#include "contact_result.h"
#include "physics/collision/collision_algorithm/collision_algorithm.h"

namespace pe_physics_collision {

    class NarrowPhaseBase {
    public:
        NarrowPhaseBase() {}
        virtual ~NarrowPhaseBase() {}

        static CollisionAlgorithm* getAlgorithm(pe_physics_shape::ShapeType type_a, pe_physics_shape::ShapeType type_b);
        virtual void calcContactResults(const pe::Array<CollisionPair>& pairs, pe::Array<ContactResult*>& results) = 0;
    };

} // namespace pe_physics_collision
