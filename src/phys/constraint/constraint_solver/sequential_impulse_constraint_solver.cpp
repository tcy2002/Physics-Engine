#include "sequential_impulse_constraint_solver.h"
#include "utils/thread_pool.h"

namespace pe_phys_constraint {

    SequentialImpulseConstraintSolver::SequentialImpulseConstraintSolver(): ConstraintSolver() {
        _iteration = 10;
    }

    void SequentialImpulseConstraintSolver::setupSolver(
            pe::Real dt,
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
        _param.dt = dt;
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(contact_results.size(), 0, [&](int i){
            auto fcc = (FrictionContactConstraint*)(_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
        });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < contact_results.size(); i++) {
            auto fcc = (FrictionContactConstraint*)(_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
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
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(_collision_objects.size(), 0,[&](int i){
            _collision_objects[i]->syncTempVelocity();
        });
        utils::ThreadPool::join();
#   else
        for (auto rb : _collision_objects) {
            rb->syncTempVelocity();
        }
#   endif

        // after solving
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(_constraints.size(), 0,[&](int i){
            _constraints[i]->afterSequentialImpulse();
        });
        utils::ThreadPool::join();
#   else
        for (auto constraint : _constraints) {
            constraint->afterSequentialImpulse();
        }
#   endif
    }

} // namespace pe_phys_constraint