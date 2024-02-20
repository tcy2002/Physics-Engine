#pragma once

#include "common/general.h"
#include "phys/object/collision_body.h"

namespace pe_phys_collision {
    typedef std::pair<pe_phys_object::CollisionBody*, pe_phys_object::CollisionBody*> CollisionPair;

    class BroadPhaseBase {
    protected:
        pe::Array<CollisionPair> _collision_pairs; // TODO

    public:
        BroadPhaseBase() {}
        virtual ~BroadPhaseBase() {}

        virtual void calcCollisionPairs(pe::Array<pe_phys_object::CollisionBody*> collision_objects) = 0;
        void clearCollisionPairs() { _collision_pairs.clear(); }
        const pe::Array<CollisionPair>& getCollisionPairs() const { return _collision_pairs; }

    protected:
        virtual bool validateCollisionPair(pe_phys_object::CollisionBody*, pe_phys_object::CollisionBody*) const;
        bool testCollisionPair(pe_phys_object::CollisionBody*, pe_phys_object::CollisionBody*) const;
    };

    class BroadPhaseSweepAndPrune : public BroadPhaseBase {
    protected:
        int _target_axis;

    public:
        BroadPhaseSweepAndPrune(): BroadPhaseBase(), _target_axis(0) {}
        ~BroadPhaseSweepAndPrune() {}
        void calcCollisionPairs(pe::Array<pe_phys_object::CollisionBody*> collision_objects) override;
    };

} // namespace pe_phys_collision