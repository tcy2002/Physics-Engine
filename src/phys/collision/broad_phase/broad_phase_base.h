#pragma once

#include "common/general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_collision {
    typedef std::pair<pe_phys_object::RigidBody*, pe_phys_object::RigidBody*> CollisionPair;

    class BroadPhaseBase {
    protected:
        pe::Array<CollisionPair> _collision_pairs;

    public:
        BroadPhaseBase() {}
        virtual ~BroadPhaseBase() {}

        virtual void calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects) = 0;
        void clearCollisionPairs() { _collision_pairs.clear(); }
        const pe::Array<CollisionPair>& getCollisionPairs() const { return _collision_pairs; }

    protected:
        virtual bool validateCollisionPair(pe_phys_object::RigidBody*, pe_phys_object::RigidBody*) const;
    };

} // namespace pe_phys_collision