#include "sequential_impulse_solver.h"
#include "phys/constraint/constraint/friction_contact_constraint.h"

namespace pe_phys_constraint {

    SequentialImpulseSolver::SequentialImpulseSolver(): Solver() {
        _iteration = 8;
    }

    void SequentialImpulseSolver::setupSolver(const pe::Array<pe_phys_object::RigidBody*>& objects,
                                              const pe::Array<pe_phys_collision::ContactResult>& contact_results,
                                              const pe::Array<Constraint*>& constraints) {
        _collision_objects = objects;
        for (auto cb : _collision_objects) {
            ((pe_phys_object::RigidBody*)cb)->clearTempVelocity();
        }

        // init contact constraints
        pe::Array<Constraint*> contact_constraints;
        for (auto& cr : contact_results) {
            auto fcc = new FrictionContactConstraint();
            fcc->setContactResult(cr);
            fcc->initSequentialImpulse(_param);
            fcc->warmStart();
            contact_constraints.push_back(fcc);
        }

        _constraints = contact_constraints;
    }

    void SequentialImpulseSolver::solve() {
        //// solve contact constraints
        for (int i = 0; i < _iteration; i++) {
            for (auto constraint : _constraints) {
                constraint->iterateSequentialImpulse(i);
            }
        }
        // TODO: why?
//        if (_param.splitPenetrationConstraintFlag) {
//            for (int i = 0; i < _iteration; i++) {
//                for (auto constraint : _constraints) {
//                    constraint->iterateSequentialImpulseForPenetration(i);
//                }
//            }
//        }

        //// sync velocity
        for (auto cb : _collision_objects) {
            ((pe_phys_object::RigidBody*)cb)->syncTempVelocity();
        }
        // TODO: why?
//        if (_param.splitPenetrationConstraintFlag) {
//            for (auto cb : _cbs) {
//                ((pe_phys_object::RigidBody*)cb)->penetrationStep(_param.dt);
//            }
//        }

        // after solving
        for (auto constraint : _constraints) {
            constraint->afterSequentialImpulse();
        }
    }

} // namespace pe_phys_constraint