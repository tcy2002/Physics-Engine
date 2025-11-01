#pragma once

#include "broad_phase_base.h"

namespace pe_physics_collision {

class BroadPhaseSweepAndPrune : public BroadPhaseBase {
protected:
    int _target_axis;

public:
    BroadPhaseSweepAndPrune(): _target_axis(0) {}
    virtual ~BroadPhaseSweepAndPrune() = default;

    void calcCollisionPairs(pe::Array<pe_physics_object::RigidBody*> objects,
                            pe::Array<CollisionPair>& pairs) override;
};

} // namespace pe_physics_collision