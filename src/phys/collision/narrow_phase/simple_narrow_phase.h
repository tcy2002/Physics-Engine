#pragma once

#include "narrow_phase_base.h"
#include "utils/pool.h"

namespace pe_phys_collision {

    class SimpleNarrowPhase: public NarrowPhaseBase {
    private:
        utils::Pool<ContactResult, 131072> _cr_pool;

    public:
        SimpleNarrowPhase(): NarrowPhaseBase() {}
        ~SimpleNarrowPhase() {}

        void calcContactResults(const pe::Array<CollisionPair>& pairs, pe::Array<ContactResult*>& results) override;
    };

} // namespace pe_phys_collision