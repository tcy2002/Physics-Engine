#pragma once

#include "force_constraint_base.h"

namespace pe_phys_constraint {

    class LorentzCircleConstraint : public ForceConstraintBase {
    public:
        explicit LorentzCircleConstraint(pe::Real friction_coeff): _friction_coeff(friction_coeff) {}

        pe::Real constraint(const pe::VectorX &in) const override {
            // checked1
            const pe::Real r2 = in[1] * in[1] + in[2] * in[2] + _perturbation;
            const pe::Real c = PE_SQRT(r2) - _friction_coeff * in[0] - PE_SQRT(_perturbation) - _shift;
            return c;
        }

        pe::VectorX gradient(const pe::VectorX &in) const override {
            // checked1
            const pe::Real r2 = in[1] * in[1] + in[2] * in[2] + _perturbation;
            const pe::Real r = PE_SQRT(r2);
            pe::Vector2 ret;
            ret << in[1] / r, in[2] / r;
            return std::move(ret);
        }

        pe::MatrixMN hessian(const pe::VectorX &in) const override {
            // checked1
            const pe::Real p = PE_MAX(_perturbation, 1e-6);
            const pe::Real r2 = in[1] * in[1] + in[2] * in[2] + p;
            pe::Matrix2 ret = pe::Matrix2::Zero();
            const pe::Real r3 = PE_POW(r2, -1.5);
            pe::Vector2 zmy;
            zmy << in[2], -in[1];
            ret << zmy * zmy.transpose() + p * pe::Matrix2::Identity();
            ret *= r3;
            return std::move(ret);
        }

        pe::Real radius_at(pe::Real i0) const override {
            // checked1
            const pe::Real k = _friction_coeff * i0 + PE_SQRT(_perturbation) + _shift;
            const pe::Real r = PE_SQRT(k * k - _perturbation);
            return r;
        }

        pe::VectorX project(const pe::VectorX &in, const pe::VectorX &start, pe::Real target) const override {
            // checked1
            pe::VectorX out = in;
            if (in(0) < 0) {
                out(0) = 0;
            }
            if (constraint(out) > target) {
                const pe::Real r = PE_MAX(radius_at(int(0)) + target, pe::Real(0));
                out.tail(2) *= r / out.tail(2).norm();
            }
            return std::move(out);
        }

        pe::Real _friction_coeff;
        pe::Real _perturbation = 1e-16;
        pe::Real _shift = 1e-8;
    };

} // namespace pe_phys_constraint