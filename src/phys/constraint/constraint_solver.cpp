#include "constraint_solver.h"
#include "constraint/friction_contact_constraint.h"
#include "common/thread_pool.h"

namespace pe_phys_constraint {

    SequentialImpulseConstraintSolver::SequentialImpulseConstraintSolver(): ConstraintSolver() {
        _iteration = 10;
    }

    void SequentialImpulseConstraintSolver::setupSolver(const pe::Array<pe_phys_object::CollisionBody*>& collision_bodies,
                                                        const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                                                        const pe::Array<Constraint*>& constraints) {
        _cbs = collision_bodies;
        for (auto cb : _cbs) {
            (static_cast<pe_phys_object::RigidBody*>(cb))->clearTempVelocity();
        }

        // init contact constraints
        pe::Array<Constraint*> contact_constraints;
        contact_constraints.insert(contact_constraints.end(), constraints.begin(), constraints.end());
        const int oldSize = contact_constraints.size();
        const int newSize = contact_results.size();

        // TODO: why?
        if (oldSize < newSize) {
            contact_constraints.resize(newSize);
            for(int i = oldSize; i < newSize; i++)
                contact_constraints[i] = new FrictionContactConstraint();
        } else if(oldSize > newSize) {
            for(int i = newSize; i < oldSize; i++){
                delete contact_constraints[i];
            }
            contact_constraints.resize(newSize);
        }

        for (int i = 0; i < newSize; i++) {
            (static_cast<FrictionContactConstraint*>(contact_constraints[i]))->setContactResult(contact_results[i]);
            contact_constraints[i]->initSequentialImpulse(_param);
            contact_constraints[i]->warmStart();
        }
    }

    void SequentialImpulseConstraintSolver::solve() {
        // solve contact constraints
        for (int i = 0; i < _iteration; i++) {
            for (auto constraint : _constraints) {
                constraint->iterateSequentialImpulse(i);
            }
        }

        // sync velocity
        for (auto cb : _cbs) {
            (static_cast<pe_phys_object::RigidBody*>(cb))->syncTempVelocity();
        }

        if (_param.splitPenetrationConstraintFlag) {
            for (auto cb : _cbs) {
                (static_cast<pe_phys_object::RigidBody*>(cb))->penetrationStep(_param.dt);
            }
        }

        // after solving
        for (auto constraint : _constraints) {
            constraint->afterSequentialImpulse();
        }
    }

} // namespace pe_phys_constraint