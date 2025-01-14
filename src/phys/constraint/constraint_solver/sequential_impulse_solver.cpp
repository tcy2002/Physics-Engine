#include "sequential_impulse_solver.h"
#include "utils/thread_pool.h"

// style-checked
namespace pe_phys_constraint {

    SequentialImpulseSolver::SequentialImpulseSolver() {
        _iteration = 30;
    }

    void SequentialImpulseSolver::setupSolver(
            pe::Real dt, const pe::Vector3& gravity,
            const pe::Array<pe_phys_object::RigidBody*>& objects,
            const pe::Array<pe_phys_collision::ContactResult*>& contact_results,
            const pe::Array<Constraint*>& constraints) {
        _collision_objects = objects;
        for (const auto co : _collision_objects) {
            co->clearTempVelocity();
        }

        // clear old constraints
        const int old_size = PE_I(_fcc_constraints.size());
        const int new_size = PE_I(contact_results.size());
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
        utils::ThreadPool::forLoop(PE_UI(contact_results.size()), [&](int i){
            const auto fcc = dynamic_cast<FrictionContactConstraint *>(_fcc_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
            fcc->initSequentialImpulse(_param);
        });
        utils::ThreadPool::forLoop(PE_UI(constraints.size()), [&](int i){
            constraints[i]->initSequentialImpulse(_param);
        });
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

    void SequentialImpulseSolver::solve() {
        // solve contact constraints
        for (int i = 0; i < _iteration; i++) {
            for (auto constraint : _fcc_constraints) {
                constraint->iterateSequentialImpulse(i);
            }
            for (auto constraint : _other_constraints) {
                constraint->iterateSequentialImpulse(i);
            }
        }

        // sync velocity
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forLoop(PE_UI(_collision_objects.size()),[&](int i){
            _collision_objects[i]->syncTempVelocity();
        });
#   else
        for (auto rb : _collision_objects) {
            rb->syncTempVelocity();
        }
#   endif
    }

} // namespace pe_phys_constraint