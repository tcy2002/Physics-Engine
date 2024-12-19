#pragma once

#include "phys/constraint/constraint/friction_contact_constraint.h"
#include "constraint_solver.h"

namespace pe_phys_constraint {

    class PrimalDualSolver : public ConstraintSolver {
    private:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        FrictionContactConstraint _fcc_constraint;

    public:
        PE_API PrimalDualSolver();
        virtual ~PrimalDualSolver() {}

        ConstraintSolverType getType() const override { return ConstraintSolverType::CST_PRIMAL_DUAL; }
        void setupSolver(pe::Real dt, const pe::Vector3& gravity,
                         const pe::Array<pe_phys_object::RigidBody*>& objects,
                         const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
                         const pe::Array<Constraint*>& constraints) override;
        void solve() override;
    };

} // namespace pe_phys_constraint