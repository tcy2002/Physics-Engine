#pragma once

#include "phys/phys_general.h"
#include "phys/collision/narrow_phase/contact_result.h"
#include "constraint.h"

namespace pe_phys_constraint {

    class FrictionContactConstraint : public Constraint {
    private:
        COMMON_MEMBER_SET_GET(pe_phys_collision::ContactResult, contact_result, ContactResult)

        struct ConstraintInfo {
            pe::Vector3 rA;
            pe::Vector3 rB;
            pe::Vector3 n;
            pe::Vector3 t0;
            pe::Vector3 t1;
            pe::Real nRhs = 0;
            pe::Real nPenetrationRhs = 0;
            pe::Real nDenomInv = 0;
            pe::Real t0DenomInv = 0;
            pe::Real t1DenomInv = 0;
            pe::Real nAppliedImpulse = 0;
            pe::Real nAppliedPenetrationImpulse = 0;
            pe::Real t0AppliedImpulse = 0;
            pe::Real t1AppliedImpulse = 0;
            pe::Real frictionCoeff = 0;
        };
        pe::Array<ConstraintInfo> _cis;

    public:
        FrictionContactConstraint();
        virtual ~FrictionContactConstraint() {};

        void initSequentialImpulse(const ConstraintParam& param) override;
        void iterateSequentialImpulse(int iter) override;
        void iterateSequentialImpulseForPenetration(int iter) override;
        void afterSequentialImpulse() override;
        void warmStart() override;
    };

} // namespace pe_phys_constraint