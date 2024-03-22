#pragma once

#include "broad_phase_base.h"

namespace pe_phys_collision {

    class BroadPhaseSweepAndPrune : public BroadPhaseBase {
    protected:
        int _target_axis;

    public:
        BroadPhaseSweepAndPrune(): BroadPhaseBase(), _target_axis(0) {}
        ~BroadPhaseSweepAndPrune() {}
        void calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects,
                                pe::Array<CollisionPair>& pairs) override;
    };

} // namespace pe_phys_collision