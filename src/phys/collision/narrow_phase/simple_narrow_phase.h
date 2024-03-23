#pragma once

#include "narrow_phase_base.h"

namespace pe_phys_collision {

    class SimpleNarrowPhase: public NarrowPhaseBase {
    public:
        SimpleNarrowPhase(): NarrowPhaseBase() {}
        ~SimpleNarrowPhase() {}

        void calcContactResults(const pe::Array<CollisionPair>& pairs, pe::Array<ContactResult>& results) override;
    };

} // namespace pe_phys_collision