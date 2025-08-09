#pragma once

#include "broad_phase_base.h"

namespace pe_phys_collision {

    class SimpleBroadPhase : public BroadPhaseBase {
    public:
        SimpleBroadPhase() {}
        ~SimpleBroadPhase() {}
        void calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects,
                                pe::Array<CollisionPair>& pairs) override;
    };

} // namespace pe_phys_collision