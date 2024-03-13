#include "world.h"
#include "phys/constraint/constraint_solver/sequential_impulse_constraint_solver.h"
#include "phys/collision/broad_phase/broad_phase_sweep_and_prune.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "utils/thread_pool.h"
#include "viewer.h"

namespace pe_core {

    World::World():
        _gravity(0, -9.8, 0),
        _dt(0.01),
        _sleep_lin_vel2_threshold(0.04),
        _sleep_ang_vel2_threshold(0.02),
        _sleep_time_threshold(0.4),
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
        pe::Real dt = _dt;
        pe::Real sleep_lin_vel2_threshold = _sleep_lin_vel2_threshold;
        pe::Real sleep_ang_vel2_threshold = _sleep_ang_vel2_threshold;
        pe::Real sleep_time_threshold = _sleep_time_threshold;
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [&](pe_phys_object::RigidBody* rb, int idx){
            if (rb->isKinematic()) return;
            if (rb->isSleep()) {
                if (rb->getLinearVelocity().norm2() >= sleep_lin_vel2_threshold ||
                    rb->getAngularVelocity().norm2() >= sleep_ang_vel2_threshold) {
                    rb->setSleep(false);
                }
            } else {
                rb->step(dt);
                rb->applyDamping(dt);
                if (rb->getLinearVelocity().norm2() < sleep_lin_vel2_threshold &&
                    rb->getAngularVelocity().norm2() < sleep_ang_vel2_threshold) {
                    rb->updateSleepTime(dt);
                    if (rb->getSleepTime() >= sleep_time_threshold) {
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
        pe::Vector3 gravity = _gravity;
        pe::Real dt = _dt;
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [&](pe_phys_object::RigidBody* rb, int idx){
            if (rb->isKinematic()) return;
            rb->addCentralForce(gravity * rb->getMass());
            rb->applyForce(dt);
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
        // external force
        applyExternalForce();

        // collision detection
        updateAABBs();
        _broad_phase->clearCollisionPairs();
        _broad_phase->calcCollisionPairs(_collision_objects);
        _narrow_phase->clearContactResults();
        _narrow_phase->calcContactResults(_broad_phase->getCollisionPairs());

//        static pe::Array<int> debug_points;
//        for (auto id : debug_points) {
//            pe_core::Viewer::removeCube(id);
//        }
//        debug_points.clear();
//        for (auto& cr : _narrow_phase->getContactResults()) {
//            if (cr.getObjectA()->getGlobalId() != 2 && cr.getObjectB()->getGlobalId() != 2) continue;
//            for (int i = 0; i < cr.getPointSize(); i++) {
//                auto point = cr.getContactPoint(i).getWorldPos();
//                int id = pe_core::Viewer::addCube({0.2, 0.2, 0.2});
//                pe_core::Viewer::updateCubeColor(id, {1, 0, 0});
//                pe_core::Viewer::updateCubeTransform(id, pe::Transform(pe::Matrix3::identity(), point));
//                debug_points.push_back(id);
//            }
//        }

        // constraints
        _constraint_solver->setupSolver(_collision_objects,
                                        _narrow_phase->getContactResults(),
                                        _constraints);
        _constraint_solver->solve();

        // update status
        updateObjectStatus();
    }

} // namespace pe_core