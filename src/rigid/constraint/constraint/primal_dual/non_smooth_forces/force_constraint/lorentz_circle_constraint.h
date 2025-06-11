#pragma once

#include "force_constraint_base.h"

namespace pe_phys_constraint {

    class LorentzCircleConstraint : public ForceConstraintBase {
    public:
        explicit LorentzCircleConstraint(pe::Real friction_coeff): _friction_coeff(friction_coeff) {}

        pe::Real constraint(const pe::VectorX& in) const override;
        pe::VectorX gradient(const pe::VectorX& in) const override;
        pe::MatrixMN hessian(const pe::VectorX& in) const override;
        pe::Real radius_at(pe::Real i0) const override;
        pe::VectorX project(const pe::VectorX& in, const pe::VectorX& start, pe::Real target) const override;

        pe::Real _friction_coeff;
        pe::Real _perturbation = 1e-16;
        pe::Real _shift = 1e-8;
    };

} // namespace pe_phys_constraint