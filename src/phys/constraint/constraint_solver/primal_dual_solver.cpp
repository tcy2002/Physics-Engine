#include "primal_dual_solver.h"
#include "utils/thread_pool.h"

namespace pe_phys_constraint {

    void PrimalDualSolver::setupSolver(
            pe::Real dt, const pe::Vector3& gravity,
            const pe::Array<pe_phys_object::RigidBody *> &objects,
            const pe::Array<pe_phys_collision::ContactResult *> &contact_results,
            const pe::Array<Constraint *> &constraints) {
        _collision_objects = objects;
        for (const auto co : _collision_objects) {
            co->clearTempVelocity();
            // apply the acceleration due to gravity
            co->setTempLinearVelocity(gravity * dt);
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
        _param.gravity = gravity;
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forLoop(UI(contact_results.size()), [&](int i){
            const auto fcc = dynamic_cast<FrictionContactConstraint *>(_fcc_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
            fcc->initPrimalDual(_param);
        });
#   else
        for (int i = 0; i < I(contact_results.size()); i++) {
            auto fcc = dynamic_cast<FrictionContactConstraint *>(_fcc_constraints[i]);
            fcc->setContactResult(*contact_results[i]);
        }
#   endif
    }


} // namespace pe_phys_constraint