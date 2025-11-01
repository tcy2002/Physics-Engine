#pragma once

#include "physics/physics.h"
#include "physics/object/rigidbody.h"

namespace pe_physics_collision {

typedef pe::KV<pe_physics_object::RigidBody*, pe_physics_object::RigidBody*> CollisionPair;

class BroadPhaseBase {
public:
    BroadPhaseBase() = default;
    virtual ~BroadPhaseBase() = default;

    virtual void calcCollisionPairs(pe::Array<pe_physics_object::RigidBody*> objects,
                                    pe::Array<CollisionPair>& pairs) = 0;
    virtual bool validateCollisionPair(pe_physics_object::RigidBody*, pe_physics_object::RigidBody*) const;
};

} // namespace pe_physics_collision