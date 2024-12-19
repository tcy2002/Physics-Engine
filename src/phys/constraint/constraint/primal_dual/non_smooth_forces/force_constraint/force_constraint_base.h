#pragma once

#include "phys/phys_general.h"

namespace pe_phys_constraint {

    class ForceConstraintBase {
    public:
        ForceConstraintBase() {}
        virtual ~ForceConstraintBase() {}
        virtual pe::Real constraint(const pe::VectorX& in) const = 0;
        virtual pe::VectorX gradient(const pe::VectorX& in) const = 0;
        virtual pe::MatrixMN hessian(const pe::VectorX& in) const = 0;
        virtual pe::Real radius_at(pe::Real i0) const { return 0; }
        virtual pe::VectorX project(const pe::VectorX& in, const pe::VectorX& start, pe::Real target = 0) const;
    };

} // namespace pe_phys_constraint