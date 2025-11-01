#include "world.h"
#include "utils/thread_pool.h"
#include "physics/object/rigidbody.h"
#include "physics/constraint/constraint_solver/sequential_impulse_solver.h"
#include "physics/collision/broad_phase/broad_phase_sweep_and_prune.h"
#include "physics/collision/narrow_phase/simple_narrow_phase.h"

namespace pe_interface {

World::World():
    _gravity(0, -9.8, 0),
    _dt(0.01),
    _sleep_lin_vel2_threshold(0),
    _sleep_ang_vel2_threshold(0),
    _sleep_time_threshold(0),
    _broad_phase(new pe_physics_collision::BroadPhaseSweepAndPrune),
    _narrow_phase(new pe_physics_collision::SimpleNarrowPhase),
    _constraint_solver(new pe_physics_constraint::SequentialImpulseSolver),
    _fracture_solver(new pe_physics_fracture::SimpleFractureSolver) {
#ifdef PE_MULTI_THREAD
    utils::ThreadPool::init();
#endif
}

World::~World() {
    delete _broad_phase;
    delete _narrow_phase;
    delete _constraint_solver;
    delete _fracture_solver;
}

void World::updateObjectStatus() {
#ifdef PE_MULTI_THREAD
    utils::ThreadPool::forLoop(PE_UI(_collision_objects.size()), [&](int i) {
        auto rb = _collision_objects[i];
        if (rb->isKinematic()) {
            rb->step(_dt); // only update AABB
            rb->setSleep(true);
            return;
        }
        if (rb->isSleep()) {
            if (rb->getLinearVelocity().norm2() >= _sleep_lin_vel2_threshold ||
                rb->getAngularVelocity().norm2() >= _sleep_ang_vel2_threshold) {
                rb->setSleep(false);
                rb->resetSleepTime();
            }
        } else {
            if (!rb->step(_dt)) {
                _rigidbodies_to_remove.push_back(rb);
                _collision_objects.erase(_collision_objects.begin() + i--);
                return;
            }
            rb->applyDamping(_dt);
            if (rb->getAvgLinearVelocity2Frames().norm2() < _sleep_lin_vel2_threshold &&
                rb->getAvgAngularVelocity2Frames().norm2() < _sleep_ang_vel2_threshold) {
                rb->updateSleepTime(_dt);
                if (rb->getSleepTime() >= _sleep_time_threshold) {
                    rb->setSleep(true);
                    rb->setLinearVelocity(pe::Vector3::zeros());
                    rb->setAngularVelocity(pe::Vector3::zeros());
                }
            } else {
                rb->resetSleepTime();
            }
        }
    });
#else
    for (int i = 0; i < I(_collision_objects.size()); i++) {
        auto rb = _collision_objects[i];
        if (rb->isKinematic()) continue;
        const auto ratio = (rb->getStaticCount() + 1) / R(rb->getDynamicCount() + rb->getStaticCount() + 1);
        if (rb->isSleep()) {
            if (rb->getLinearVelocity().norm2() >= _sleep_lin_vel2_threshold * ratio ||
                rb->getAngularVelocity().norm2() >= _sleep_ang_vel2_threshold * ratio) {
                rb->setSleep(false);
                rb->resetSleepTime();
            }
        } else {
            if (!rb->step(_dt)) {
                _rigidbodies_to_remove.push_back(rb);
                _collision_objects.erase(_collision_objects.begin() + i--);
                continue;
            }
            rb->applyDamping(_dt);
            if (rb->getLinearVelocity().norm2() < _sleep_lin_vel2_threshold * ratio &&
                rb->getAngularVelocity().norm2() < _sleep_ang_vel2_threshold * ratio) {
                rb->updateSleepTime(_dt);
                if (rb->getSleepTime() >= _sleep_time_threshold) {
                    rb->setSleep(true);
                    rb->setLinearVelocity(pe::Vector3::zeros());
                    rb->setAngularVelocity(pe::Vector3::zeros());
                }
            } else {
                rb->resetSleepTime();
            }
        }
        rb->resetStaticCount();
        rb->resetDynamicCount();
    }
#endif
}

void World::applyExternalForce() {
    for (auto& rb : _collision_objects) {
        if (rb->isKinematic()) continue;
        rb->addCentralForce(_gravity * rb->getMass());
        rb->applyForce(_dt);
    }
}

void World::calcDamageEffects() {
    if (_fracture_sources.empty()) {
        return;
    }
    for (int i = 0; i < PE_I(_collision_objects.size()); i++) {
        auto rb = _collision_objects[i];
        if (rb->isFracturable()) {
            _fracture_solver->setFracturableObject((pe_physics_object::FracturableObject*)rb);
            _fracture_solver->solve(_fracture_sources);
            if (!_fracture_solver->getFragments().empty()) {
                for (auto frag : _fracture_solver->getFragments()) {
                    _collision_objects.push_back(frag);
                    _rigidbodies_to_add.push_back(frag);
                }
                _rigidbodies_to_remove.push_back(rb);
                _collision_objects.erase(_collision_objects.begin() + i--);
                _fracture_solver->clearFragments();
            }
        }
    }
    _fracture_sources.clear();
}


void World::addRigidBody(pe_physics_object::RigidBody* rigidbody) {
    _collision_objects.push_back(rigidbody);
    _rigidbodies_to_add.push_back(rigidbody);
}

void World::removeRigidBody(pe_physics_object::RigidBody *rigidbody) {
    for (int i = 0; i < PE_I(_collision_objects.size()); i++) {
        if (_collision_objects[i]->getGlobalId() == rigidbody->getGlobalId()) {
            _collision_objects.erase(_collision_objects.begin() + i);
            _rigidbodies_to_remove.push_back(rigidbody);
            break;
        }
    }
}

void World::updateRigidBody(pe_physics_object::RigidBody *rigidbody) {
    _rigidbodies_to_remove.push_back(rigidbody);
    _rigidbodies_to_add.push_back(rigidbody);
}

void World::removeConstraint(pe_physics_constraint::Constraint *constraint) {
    for (int i = 0; i < PE_I(_constraints.size()); i++) {
        if (_constraints[i]->getGlobalId() == constraint->getGlobalId()) {
            _constraints.erase(_constraints.begin() + i);
            break;
        }
    }
}

void World::step() {
    // std::cout << "1" << std::endl;
    // update status
    auto start = COMMON_GetMicroTickCount();
    updateObjectStatus();

    // fracture
    calcDamageEffects();
    // std::cout << "2" << std::endl;

    // external force
    applyExternalForce();
    auto end = COMMON_GetMicroTickCount();
    update_status_time += PE_R(end - start) * PE_R(0.000001);
    // std::cout << "3" << std::endl;

    // collision detection
    start = COMMON_GetMicroTickCount();
    _broad_phase->calcCollisionPairs(_collision_objects, _collision_pairs);
    end = COMMON_GetMicroTickCount();
    broad_phase_time += PE_R(end - start) * PE_R(0.000001);
    // std::cout << "4" << std::endl;

    start = COMMON_GetMicroTickCount();
    _narrow_phase->calcContactResults(_collision_pairs, _contact_results);
    end = COMMON_GetMicroTickCount();
    narrow_phase_time += PE_R(end - start) * PE_R(0.000001);
    // std::cout << "5" << std::endl;

    // constraints
    start = COMMON_GetMicroTickCount();
    _constraint_solver->setupSolver(_dt, _gravity, _collision_objects, _contact_results, _constraints);
    _constraint_solver->solve();
    end = COMMON_GetMicroTickCount();
    constraint_solver_time += PE_R(end - start) * PE_R(0.000001);
    // std::cout << "6" << std::endl;
}

} // namespace pe_interface