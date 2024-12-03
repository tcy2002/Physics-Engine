#pragma once

#include "force_constraint_base.h"

namespace pe_phys_constraint {

    class LorentzCircleConstraint : public ForceConstraintBase {
    public:
        explicit LorentzCircleConstraint(pe::Real friction_coeff): _friction_coeff(friction_coeff) {}

        pe::Real constraint(const pe::VectorX &in) const override {
            const pe::Real r2 = in[1] * in[1] + in[2] * in[2] + _perturbation;
            const pe::Real c = PE_SQRT(r2) - _friction_coeff * in[0] - PE_SQRT(_perturbation) - _shift;
            return c;
        }

        pe::VectorX gradient(const pe::VectorX &in) const override {
            const pe::Real r2 = in[1] * in[1] + in[2] * in[2] + _perturbation;
            const pe::Real r = PE_SQRT(r2);
            pe::VectorX ret(2);
            ret[0] = in[1] / r;
            ret[1] = in[2] / r;
            return std::move(ret);
        }

        pe::MatrixMN hessian(const pe::VectorX &in) const override {
            const pe::Real p = PE_MAX(_perturbation, 1e-6);
            const pe::Real r2 = in[1] * in[1] + in[2] * in[2] + p;
            pe::MatrixMN ret(2, 2);
            const pe::Real r3 = r2 * PE_SQRT(r2);
            ret(0, 0) = in[2] * in[2] + p;
            ret(0, 1) = ret(1, 0) = -in[1] * in[2];
            ret(1, 1) = in[1] * in[1] + p;
            ret /= r3;
            return std::move(ret);
        }

        pe::Real radius_at(pe::Real i0) const override {
            const pe::Real k = _friction_coeff * i0 + PE_SQRT(_perturbation) + _shift;
            const pe::Real r = PE_SQRT(k * k - _perturbation);
            return r;
        }

        pe::VectorX project(const pe::VectorX &in, const pe::VectorX &start, pe::Real target) const override {
            pe::VectorX out = in;
            out[0] = out[0] < 0 ? 0 : out[0];
            if (constraint(out) > target) {
                const pe::Real r = PE_MAX(radius_at(in[0]) + target, 0);
                const pe::Real k = r / PE_SQRT(out[1] * out[1] + out[2] * out[2]);
                out[1] *= k;
                out[2] *= k;
            }
            return std::move(out);
        }

        pe::Real _friction_coeff;
        pe::Real _perturbation = 1e-16;
        pe::Real _shift = 1e-8;
    };

} // namespace pe_phys_constraint