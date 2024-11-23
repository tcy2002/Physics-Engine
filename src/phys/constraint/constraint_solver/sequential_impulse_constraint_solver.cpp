#include "sequential_impulse_constraint_solver.h"
#include "utils/thread_pool.h"

// style-checked
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
        for (const auto co : _collision_objects) {
            co->clearTempVelocity();
        }

        // clear old constraints
        const int old_size = I(_fcc_constraints.size());
        const int new_size = I(contact_results.size());
        if (old_size < new_size) {
            _fcc_constraints.resize(new_size);
            for (int i = old_size; i < new_size; i++) {
                _fcc_constraints[i] = _fcc_pool.create();
            }
        } else {
            for (int i = new_size; i < old_size; i++) {
                _fcc_pool.destroy(dynamic_cast<FrictionContactConstraint*>(_fcc_constraints[i]));
            }
            _fcc_constraints.resize(new_size);
        }

        _param.dt = dt;
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(UI(contact_results.size()), 0, [&](int i){
            const auto fcc = dynamic_cast<FrictionContactConstraint *>(_fcc_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
        });
        utils::ThreadPool::forBatchedLoop(UI(constraints.size()), 0, [&](int i){
            constraints[i]->initSequentialImpulse(_param);
            constraints[i]->warmStart();
        });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < I(contact_results.size()); i++) {
            auto fcc = dynamic_cast<FrictionContactConstraint *>(_fcc_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
        }
        for (auto constraint : constraints) {
            constraint->initSequentialImpulse(_param);
        }
#   endif

        _other_constraints = constraints;
    }

    void SequentialImpulseConstraintSolver::solve() {
        // solve contact constraints
        for (int i = 0; i < _iteration; i++) {
#   ifdef PE_MULTI_THREAD
            utils::ThreadPool::forBatchedLoop(UI(_fcc_constraints.size()), 0,[&](int i){
                _fcc_constraints[i]->iterateSequentialImpulse(i);
            });
            utils::ThreadPool::forBatchedLoop(UI(_other_constraints.size()), 0,[&](int i){
                _other_constraints[i]->iterateSequentialImpulse(i);
            });
            utils::ThreadPool::join();
#   else
            for (auto constraint : _fcc_constraints) {
                constraint->iterateSequentialImpulse(i);
            }
            for (auto constraint : _other_constraints) {
                constraint->iterateSequentialImpulse(i);
            }
#   endif
        }

        // sync velocity
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(UI(_collision_objects.size()), 0,[&](int i){
            _collision_objects[i]->syncTempVelocity();
        });
        utils::ThreadPool::join();
#   else
        for (auto rb : _collision_objects) {
            rb->syncTempVelocity();
        }
#   endif
    }

} // namespace pe_phys_constraint