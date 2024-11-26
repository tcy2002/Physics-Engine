#pragma once

#include "phys/constraint/constraint/friction_contact_constraint.h"
#include "constraint_solver.h"
#include "utils/object_pool.h"

namespace pe_phys_constraint {

    // matrix-free Gauss-Seidel-styled sequential impulse solver
    class SequentialImpulseSolver : public ConstraintSolver {
    private:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        pe::Array<Constraint*> _fcc_constraints;
        pe::Array<Constraint*> _other_constraints;
        utils::ObjectPool<FrictionContactConstraint, 2048> _fcc_pool;

    public:
        SequentialImpulseSolver();
        virtual ~SequentialImpulseSolver() {}

        virtual ConstraintSolverType getType() const override { return ST_SEQUENTIAL_IMPULSE; }
        virtual void setupSolver(pe::Real dt, const pe::Vector3& gravity,
                                 const pe::Array<pe_phys_object::RigidBody*>& objects,
                                 const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
                                 const pe::Array<Constraint*>& constraints) override;
        virtual void solve() override;
    };

} // namespace pe_phys_constraint