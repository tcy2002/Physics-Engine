#pragma once

#include "rigid/phys_general.h"
#include "rigid/collision/broad_phase/broad_phase_base.h"
#include "contact_result.h"
#include "rigid/collision/collision_algorithm/collision_algorithm.h"

namespace pe_phys_collision {

    class NarrowPhaseBase {
    public:
        NarrowPhaseBase() {}
        virtual ~NarrowPhaseBase() {}

        static CollisionAlgorithm* getAlgorithm(pe_phys_shape::ShapeType type_a, pe_phys_shape::ShapeType type_b);
        virtual void calcContactResults(const pe::Array<CollisionPair>& pairs, pe::Array<ContactResult*>& results) = 0;
    };

} // namespace pe_phys_collision