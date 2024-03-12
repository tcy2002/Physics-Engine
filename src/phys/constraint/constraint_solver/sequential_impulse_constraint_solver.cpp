#include "sequential_impulse_constraint_solver.h"
#include "phys/constraint/constraint/friction_contact_constraint.h"
#include "utils/thread_pool.h"

namespace pe_phys_constraint {

    SequentialImpulseConstraintSolver::SequentialImpulseConstraintSolver(): ConstraintSolver() {
        _iteration = 16;
    }

    void SequentialImpulseConstraintSolver::setupSolver(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                                        const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                                                        const pe::Array<Constraint*>& constraints) {
        _collision_objects = objects;
        for (auto co : _collision_objects) {
            co->clearTempVelocity();
        }

        // init contact constraints: the start order doesn't matter, so we can use multi-thread
        pe::Array<Constraint*> contact_constraints(contact_results.size());
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forEach(contact_results.begin(), contact_results.end(),
                                   [this, &contact_constraints](const pe_phys_collision::ContactResult& cr,
                                           int idx){
           auto fcc = new FrictionContactConstraint();
           fcc->setContactResult(cr);
           fcc->initSequentialImpulse(_param);
           fcc->warmStart();
           contact_constraints[idx] = fcc;
        });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < contact_results.size(); i++) {
            auto fcc = new FrictionContactConstraint();
            fcc->setContactResult(contact_results[i]);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
            contact_constraints[i] = fcc;
        }
#   endif

        _constraints = contact_constraints;
    }

    void SequentialImpulseConstraintSolver::solve() {
        // solve contact constraints: the execution order is significant, so we use single-thread
        for (int i = 0; i < _iteration; i++) {
            for (auto constraint : _constraints) {
                constraint->iterateSequentialImpulse(i);
            }
        }

        // sync velocity
        for (auto co : _collision_objects) {
            co->syncTempVelocity();
        }

        // after solving
        for (auto constraint : _constraints) {
            constraint->afterSequentialImpulse();
        }
    }

} // namespace pe_phys_constraint