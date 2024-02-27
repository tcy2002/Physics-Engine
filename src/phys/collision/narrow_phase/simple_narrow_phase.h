#pragma once

#include "narrow_phase_base.h"

namespace pe_phys_collision {

    class SimpleNarrowPhase: public NarrowPhaseBase {
    private:
        pe::Array<ContactResult> _contact_results;

    public:
        SimpleNarrowPhase(): NarrowPhaseBase() {}
        ~SimpleNarrowPhase() {}

        void calcContactResults(const pe::Array<CollisionPair>& pairs) override;
        void clearContactResults() override;
        pe::Array<ContactResult>& getContactResults() override;
    };

} // namespace pe_phys_collision