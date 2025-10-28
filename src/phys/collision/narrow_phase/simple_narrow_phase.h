#pragma once

#include "narrow_phase_base.h"
#include "utils/object_pool.h"

namespace pe_phys_collision {

    class SimpleNarrowPhase: public NarrowPhaseBase {
    private:
        utils::ObjectPool<ContactResult, 16384> _cr_pool;

    public:
        SimpleNarrowPhase() {}
        virtual ~SimpleNarrowPhase() {}

        void calcContactResults(const pe::Array<CollisionPair>& pairs, pe::Array<ContactResult*>& results) override;
    };

} // namespace pe_phys_collision