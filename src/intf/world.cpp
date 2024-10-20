#include "world.h"
#include "phys/object/rigidbody.h"
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
        _sleep_pos_threshold(0.0),
        _sleep_time_threshold(0.0),
        _broad_phase(new pe_phys_collision::BroadPhaseSweepAndPrune),
        _narrow_phase(new pe_phys_collision::SimpleNarrowPhase),
        _constraint_solver(new pe_phys_constraint::SequentialImpulseConstraintSolver),
        _fracture_solver(new pe_phys_fracture::SimpleFractureSolver) {
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::init();
#   endif
    }

    World::~World() {
        delete _broad_phase;
        delete _narrow_phase;
        delete _constraint_solver;
        delete _fracture_solver;
    }

    void World::updateAABBs() {
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forBatchedLoop(_collision_objects.size(), 0,[&](int i) {
            _collision_objects[i]->computeAABB();
        });
        utils::ThreadPool::join();
#   else
        for (auto& co : _collision_objects) {
            co->computeAABB();
        }
#   endif
    }

    void World::updateObjectStatus() {
        for (int i = 0; i < (int)_collision_objects.size(); i++) {
            auto rb = _collision_objects[i];
            if (rb->isKinematic()) continue;
            auto ratio = (rb->getStaticCount() + 1) / (pe::Real)(rb->getDynamicCount() + rb->getStaticCount() + 1);
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
    }

    void World::applyExternalForce() {
        for (auto& rb : _collision_objects) {
            if (rb->isKinematic()) continue;
            rb->addCentralForce(_gravity * rb->getMass());
            rb->applyForce(_dt);
        }
    }

    void World::execCollisionCallbacks() {
        // not suitable for multi-thread
        for (auto& cr : _contact_results) {
            if (cr->getPointSize() == 0) continue;
            auto rb1 = cr->getObjectA();
            auto rb2 = cr->getObjectB();
            if (rb1->getCollisionCallbacks().empty() && rb2->getCollisionCallbacks().empty()) continue;

            pe::Vector3 pos = pe::Vector3::zeros();
            pe::Vector3 nor = pe::Vector3::zeros();
            pe::Vector3 vel = pe::Vector3::zeros();
            pe::Real depth = 0;
            for (int i = 0; i < cr->getPointSize(); i++) {
                pos += cr->getContactPoint(i).getWorldPos();
                nor += cr->getContactPoint(i).getWorldNormal();
                depth += cr->getContactPoint(i).getDistance();
                vel += rb1->getLinearVelocityAtLocalPoint(cr->getContactPoint(i).getLocalPosA());
                vel -= rb2->getLinearVelocityAtLocalPoint(cr->getContactPoint(i).getLocalPosB());
            }
            pos /= (pe::Real)cr->getPointSize();
            nor.normalize();
            depth /= (pe::Real)cr->getPointSize();
            vel /= (pe::Real)cr->getPointSize();

            for (auto& cb : rb1->getCollisionCallbacks()) {
                cb(rb1, rb2, pos + nor * depth, -nor, -vel);
            }
            for (auto& cb : rb2->getCollisionCallbacks()) {
                cb(rb2, rb1, pos, nor, vel);
            }
        }
    }

    void World::addRigidBody(pe_phys_object::RigidBody* rigidbody) {
        _collision_objects.push_back(rigidbody);
        _rigidbodies_to_add.push_back(rigidbody);
    }

    void World::removeRigidBody(pe_phys_object::RigidBody *rigidbody) {
        for (int i = 0; i < (int)_collision_objects.size(); i++) {
            if (_collision_objects[i] == rigidbody) {
                _collision_objects.erase(_collision_objects.begin() + i);
                _rigidbodies_to_remove.push_back(rigidbody);
                break;
            }
        }
    }

    void World::updateRigidBody(pe_phys_object::RigidBody *rigidbody) {
        _rigidbodies_to_remove.push_back(rigidbody);
        _rigidbodies_to_add.push_back(rigidbody);
    }

    void World::step() {
        // update status
        auto start = COMMON_GetMicroseconds();
        updateObjectStatus();
        auto end = COMMON_GetMicroseconds();
        update_status_time += (pe::Real)(end - start) * pe::Real(0.000001);

        // fracture
        if (!_fracture_sources.empty()) {
            for (int i = 0; i < (int)_collision_objects.size(); i++) {
                auto rb = _collision_objects[i];
                if (rb->isFracturable()) {
                    _fracture_solver->setFracturableObject((pe_phys_object::FracturableObject*)rb);
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

        // external force
        applyExternalForce();

        // collision detection
        start = COMMON_GetMicroseconds();
        updateAABBs();
        _broad_phase->calcCollisionPairs(_collision_objects, _collision_pairs);
        end = COMMON_GetMicroseconds();
        broad_phase_time += (pe::Real)(end - start) * pe::Real(0.000001);

        start = COMMON_GetMicroseconds();
        _narrow_phase->calcContactResults(_collision_pairs, _contact_results);
        execCollisionCallbacks();
        end = COMMON_GetMicroseconds();
        narrow_phase_time += (pe::Real)(end - start) * pe::Real(0.000001);

        // constraints
        start = COMMON_GetMicroseconds();
        _constraint_solver->setupSolver(_dt,
                                        _collision_objects,
                                        _contact_results,
                                        _constraints);
        _constraint_solver->solve();
        end = COMMON_GetMicroseconds();
        constraint_solver_time += (pe::Real)(end - start) * pe::Real(0.000001);
    }

} // namespace pe_intf