#include "world.h"
#include "phys/constraint/constraint_solver/sequential_impulse_constraint_solver.h"
#include "phys/collision/broad_phase/broad_phase_sweep_and_prune.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_intf {

    World::World():
        _gravity(0, -9.8, 0),
        _dt(0.01),
        _sleep_lin_vel2_threshold(0.00),
        _sleep_ang_vel2_threshold(0.00),
        _sleep_time_threshold(0.0),
        _broad_phase(new pe_phys_collision::BroadPhaseSweepAndPrune),
        _narrow_phase(new pe_phys_collision::SimpleNarrowPhase),
        _constraint_solver(new pe_phys_constraint::SequentialImpulseConstraintSolver) {
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::init(std::thread::hardware_concurrency() * 2);
#   endif
    }

    World::~World() {
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::deinit();
#   endif
        delete _broad_phase;
        delete _narrow_phase;
        delete _constraint_solver;
    }

    void World::updateAABBs() {
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [](pe_phys_object::RigidBody* rb, int idx) {
                                       rb->computeAABB();
                                   });
        utils::ThreadPool::join();
#   else
        for (auto& co : _collision_objects) {
            co->computeAABB();
        }
#   endif
    }

    void World::updateObjectStatus() {
#   ifdef PE_MULTI_THREAD
        auto c = this;
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [&c](pe_phys_object::RigidBody* rb, int idx){
                                       if (rb->isKinematic()) return;
                                       if (rb->isSleep()) {
                                           if (rb->getLinearVelocity().norm2() >= c->_sleep_lin_vel2_threshold ||
                                               rb->getAngularVelocity().norm2() >= c->_sleep_ang_vel2_threshold) {
                                               rb->setSleep(false);
                                           }
                                       } else {
                                           rb->step(c->_dt);
                                           rb->applyDamping(c->_dt);
                                           if (rb->getLinearVelocity().norm2() < c->_sleep_lin_vel2_threshold &&
                                               rb->getAngularVelocity().norm2() < c->_sleep_ang_vel2_threshold) {
                                               rb->updateSleepTime(c->_dt);
                                               if (rb->getSleepTime() >= c->_sleep_time_threshold) {
                                                   rb->setSleep(true);
                                                   rb->setLinearVelocity(pe::Vector3::zeros());
                                                   rb->setAngularVelocity(pe::Vector3::zeros());
                                               }
                                           } else {
                                               rb->resetSleepTime();
                                           }
                                       }
                                   });
        utils::ThreadPool::join();
#   else
        for (auto& rb : _collision_objects) {
            if (rb->isKinematic()) continue;
            if (rb->isSleep()) {
                if (rb->getLinearVelocity().norm2() >= _sleep_lin_vel2_threshold ||
                    rb->getAngularVelocity().norm2() >= _sleep_ang_vel2_threshold) {
                    rb->setSleep(false);
                }
            } else {
                rb->step(_dt);
                rb->applyDamping(_dt);
                if (rb->getLinearVelocity().norm2() < _sleep_lin_vel2_threshold &&
                    rb->getAngularVelocity().norm2() < _sleep_ang_vel2_threshold) {
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
        }
#   endif
    }

    void World::applyExternalForce() {
#   ifdef PE_MULTI_THREAD
        auto c = this;
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [&c](pe_phys_object::RigidBody* rb, int idx){
                                       if (rb->isKinematic()) return;
                                       rb->addCentralForce(c->_gravity * rb->getMass());
                                       rb->applyForce(c->_dt);
                                   });
        utils::ThreadPool::join();
#   else
        for (auto& rb : _collision_objects) {
            if (rb->isKinematic()) continue;
            rb->addCentralForce(_gravity * rb->getMass());
            rb->applyForce(_dt);
        }
#   endif
    }

    void World::addRigidBody(pe_phys_object::RigidBody* rigidbody) {
        _collision_objects.push_back(rigidbody);
    }

    void World::step() {
        // update status
        updateObjectStatus();

        // external force
        applyExternalForce();

        // collision detection
        auto start = COMMON_GetTickCount();
        updateAABBs();
        _broad_phase->calcCollisionPairs(_collision_objects, _collision_pairs);
        auto end = COMMON_GetTickCount();
        broad_phase_time += (pe::Real)(end - start) * pe::Real(0.001);

        start = COMMON_GetTickCount();
        _narrow_phase->calcContactResults(_collision_pairs, _contact_results);
        end = COMMON_GetTickCount();
        narrow_phase_time += (pe::Real)(end - start) * pe::Real(0.001);

        // constraints
        start = COMMON_GetTickCount();
        _constraint_solver->setupSolver(_collision_objects,
                                        _contact_results,
                                        _constraints);
        _constraint_solver->solve();
        end = COMMON_GetTickCount();
        constraint_solver_time += (pe::Real)(end - start) * pe::Real(0.001);
    }

} // namespace pe_core