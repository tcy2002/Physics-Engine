#pragma once

namespace pe_phys_collision {

    class NarrowPhaseBase {
    public:
        virtual void calcContactResults() = 0;
        virtual void clearContactResults() = 0;
        virtual void getContactResults() = 0;
    };

    class SimpleNarrowPhase: public NarrowPhaseBase {
    public:
        ~SimpleNarrowPhase() { clearContactResults(); } // NOLINT

        void calcContactResults() override;
        void clearContactResults() override;
        void getContactResults() override;
    };

} // namespace pe_phys_collision