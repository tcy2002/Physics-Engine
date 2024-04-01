#pragma once

#include "phys/phys_general.h"
#include "phys/collision/narrow_phase/contact_result.h"
#include "constraint.h"

#define PE_MAX_CONTACT_POINT 8

namespace pe_phys_constraint {

    class FrictionContactConstraint : public Constraint {
    private:
        COMMON_MEMBER_SET_GET(pe_phys_collision::ContactResult, contact_result, ContactResult)

        struct ConstraintInfo {
            pe::Vector3 r_a;
            pe::Vector3 r_b;
            pe::Vector3 n;
            pe::Vector3 t0;
            pe::Vector3 t1;
            pe::Real n_rhs = 0;
            pe::Real n_denom_inv = 0;
            pe::Real t0_denom_inv = 0;
            pe::Real t1_denom_inv = 0;
            pe::Real n_applied_impulse = 0;
            pe::Real t0_applied_impulse = 0;
            pe::Real t1_applied_impulse = 0;
            pe::Real friction_coeff = 0;
        };
        pe::Array<ConstraintInfo> _cis;

    public:
        FrictionContactConstraint() {}
        virtual ~FrictionContactConstraint() {};

        virtual void initSequentialImpulse(const ConstraintParam& param) override;
        virtual void warmStart() override;
        virtual void iterateSequentialImpulse(int iter) override;
        virtual void afterSequentialImpulse() override;
    };

} // namespace pe_phys_constraint