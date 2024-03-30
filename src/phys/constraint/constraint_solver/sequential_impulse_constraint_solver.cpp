#include "sequential_impulse_constraint_solver.h"
#include "utils/thread_pool.h"

namespace pe_phys_constraint {

    SequentialImpulseConstraintSolver::SequentialImpulseConstraintSolver(): ConstraintSolver() {
        _iteration = 16;
    }

    void SequentialImpulseConstraintSolver::setupSolver(
            const pe::Array<pe_phys_object::RigidBody*>& objects,
            const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
            const pe::Array<Constraint*>& constraints) {
        _collision_objects = objects;
        for (auto co : _collision_objects) {
            co->clearTempVelocity();
        }

        // clear old constraints
        const int old_size = (int)_constraints.size();
        const int new_size = (int)contact_results.size();
        if (old_size < new_size) {
            _constraints.resize(new_size);
            for (int i = old_size; i < new_size; i++) {
                _constraints[i] = _fcc_pool.create();
            }
        } else {
            for (int i = new_size; i < old_size; i++) {
                _fcc_pool.destroy((FrictionContactConstraint*)_constraints[i]);
            }
            _constraints.resize(new_size);
        }

        // init contact constraints: the start order doesn't matter, so we can use multi-thread
#   ifdef PE_MULTI_THREAD
        auto c = this;
        utils::ThreadPool::forEach(contact_results.begin(), contact_results.end(),
                                   [&c](pe_phys_collision::ContactResult* cr, int idx){
                                       auto fcc = (FrictionContactConstraint*)(c->_constraints[idx]);
                                       fcc->setContactResult(*cr);
                                       fcc->initSequentialImpulse(c->_param);
                                       fcc->warmStart();
                                   });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < contact_results.size(); i++) {
            auto fcc = new FrictionContactConstraint();
            fcc->setContactResult(*contact_results[i]);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
            _constraints[i] = fcc;
        }
#   endif
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