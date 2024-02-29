#pragma once

#include "solver.h"

namespace pe_phys_constraint {

    class SequentialImpulseSolver : public Solver {
    private:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        pe::Array<Constraint*> _constraints;

    public:
        SequentialImpulseSolver();
        virtual ~SequentialImpulseSolver() {};

        void setupSolver(const pe::Array<pe_phys_object::RigidBody*>& objects,
                         const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                         const pe::Array<Constraint*>& constraints) override;
        void solve() override;
    };

} // namespace pe_phys_constraint