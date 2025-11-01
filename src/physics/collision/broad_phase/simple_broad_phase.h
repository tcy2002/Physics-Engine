#pragma once

#include "broad_phase_base.h"

namespace pe_physics_collision {

class SimpleBroadPhase : public BroadPhaseBase {
public:
    SimpleBroadPhase() = default;
    virtual ~SimpleBroadPhase() = default;

    void calcCollisionPairs(pe::Array<pe_physics_object::RigidBody*> objects,
                            pe::Array<CollisionPair>& pairs) override;
};

} // namespace pe_physics_collision