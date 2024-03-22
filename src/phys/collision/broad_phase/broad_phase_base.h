#pragma once

#include "common/general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_collision {
    typedef std::pair<pe_phys_object::RigidBody*, pe_phys_object::RigidBody*> CollisionPair;

    class BroadPhaseBase {
    public:
        BroadPhaseBase() {}
        virtual ~BroadPhaseBase() {}

        // todo: do not copy
        virtual void calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects,
                                        pe::Array<CollisionPair>& pairs) = 0;

    protected:
        virtual bool validateCollisionPair(pe_phys_object::RigidBody*, pe_phys_object::RigidBody*) const;
    };

} // namespace pe_phys_collision