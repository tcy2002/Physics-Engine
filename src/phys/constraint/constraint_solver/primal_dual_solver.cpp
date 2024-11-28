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

        _param.dt = dt;
        _param.gravity = gravity;
        _fcc_constraint.setObjects(&_collision_objects);
        _fcc_constraint.setContactResults(&contact_results);
        _fcc_constraint.initPrimalDual(_param);
    }

} // namespace pe_phys_constraint