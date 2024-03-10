#pragma once

#include "constraint_solver.h"

namespace pe_phys_constraint {

    // matrix-free Gauss-Seidel-styled sequential impulse solver
    class SequentialImpulseConstraintSolver : public ConstraintSolver {
    private:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        pe::Array<Constraint*> _constraints;

    public:
        SequentialImpulseConstraintSolver();
        virtual ~SequentialImpulseConstraintSolver() {};

        void setupSolver(const pe::Array<pe_phys_object::RigidBody*>& objects,
                         const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                         const pe::Array<Constraint*>& constraints) override;
        void solve() override;
    };

} // namespace pe_phys_constraint