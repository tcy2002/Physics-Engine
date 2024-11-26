#pragma once

#include "phys/constraint/constraint/friction_contact_constraint.h"
#include "constraint_solver.h"
#include "utils/object_pool.h"

namespace pe_phys_constraint {

    class PrimalDualSolver : public ConstraintSolver {
    private:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        pe::Array<Constraint*> _fcc_constraints;
        utils::ObjectPool<FrictionContactConstraint, 2048> _fcc_pool;

    public:
        PrimalDualSolver();
        virtual ~PrimalDualSolver() {}

        virtual ConstraintSolverType getType() const override { return ST_PRIMAL_DUAL; }
        virtual void setupSolver(pe::Real dt, const pe::Vector3& gravity,
                                 const pe::Array<pe_phys_object::RigidBody*>& objects,
                                 const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
                                 const pe::Array<Constraint*>& constraints) override;
        virtual void solve() override;
    };

} // namespace pe_phys_constraint