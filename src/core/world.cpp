#include "world.h"
#include "phys/constraint/constraint_solver/sequential_impulse_constraint_solver.h"
#include "phys/collision/broad_phase/broad_phase_sweep_and_prune.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_core {

    World::World():
        _gravity(0, -9.8, 0), _dt(0.01),
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
                                   [](pe_phys_object::RigidBody* rb, int _) {
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
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [this](pe_phys_object::RigidBody* rb, int idx){
                                       if (!rb->isKinematic()) {
                                           rb->step(_dt);
                                       }
                                   });
        utils::ThreadPool::join();
#   else
        for (auto& co : _collision_objects) {
            if (!co->isKinematic()) {
                co->step(_dt);
            }
        }
#   endif
    }

    void World::applyExternalForce() {
#   ifdef PE_MULTI_THREAD
        utils::ThreadPool::forEach(_collision_objects.begin(), _collision_objects.end(),
                                   [this](pe_phys_object::RigidBody* rb, int idx){
                                       if (!rb->isKinematic()) {
                                           rb->addCentralForce(_gravity * rb->getMass());
                                           rb->applyForce(_dt);
                                       }
                                   });
        utils::ThreadPool::join();
#   else
        for (auto& co : _collision_objects) {
            if (!co->isKinematic()) {
                auto rb = (pe_phys_object::RigidBody*)co;
                rb->addCentralForce(_gravity * rb->getMass());
                rb->applyForce(_dt);
            }
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

        // constraints
        _constraint_solver->setupSolver(_collision_objects,
                                        _narrow_phase->getContactResults(),
                                        _constraints);
        _constraint_solver->solve();

        // update status
        updateObjectStatus();
    }

} // namespace pe_core