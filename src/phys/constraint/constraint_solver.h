#pragma once

#include "phys/phys_general.h"
#include "phys/constraint/constraint/constraint.h"
#include "phys/collision/narrow_phase/contact_result.h"

namespace pe_phys_constraint {

    class ConstraintSolver {
        COMMON_MEMBER_SET_GET(ConstraintParam, param, Param);
        COMMON_MEMBER_SET_GET(int, iteration, Iteration);

    public:
        ConstraintSolver(): _param(ConstraintParam()), _iteration(1) {}
        virtual ~ConstraintSolver() {}
        virtual void setupSolver(const pe::Array<pe_phys_object::CollisionBody*>& collision_bodies,
                                 const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                                 const pe::Array<Constraint*>& constraints) {}
        virtual void solve() {}
    };

    class SequentialImpulseConstraintSolver : public ConstraintSolver {
    private:
        pe::Array<pe_phys_object::CollisionBody*> _cbs;
        pe::Array<Constraint*> _constraints;

    public:
        SequentialImpulseConstraintSolver();
        virtual ~SequentialImpulseConstraintSolver() {};

        void setupSolver(const pe::Array<pe_phys_object::CollisionBody*>& collision_bodies,
                         const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                         const pe::Array<Constraint*>& constraints) override;
        void solve() override;
    };

} // namespace pe_phys_constraint