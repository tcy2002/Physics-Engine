#include "world.h"
#include "common/thread_pool.h"

namespace pe_core {

    World::World():
        _gravity(0, -9.8, 0), _time_step(0.01),
        _broad_phase(new pe_phys_collision::BroadPhaseSweepAndPrune),
        _narrow_phase(new pe_phys_collision::SimpleNarrowPhase),
        _constraint_solver(new pe_phys_constraint::SequentialImpulseConstraintSolver) {}

    World::~World() {
        delete _broad_phase;
        delete _narrow_phase;
        delete _constraint_solver;
    }

    void World::updateAABBs() {
        common::ThreadPool::forEach(_cbs.begin(), _cbs.end(), [](auto& cb) {
            cb->computeAABB();
        });
        common::ThreadPool::join();
    }

    void World::updateCollisionBodies() {
        common::ThreadPool::forEach(_cbs.begin(), _cbs.end(), [this](auto& cb) {
            if (!cb->isDeformable()) {
                static_cast<pe_phys_object::RigidBody*>(cb)->step(_time_step);
            }
        });
        common::ThreadPool::join();
    }

    void World::applyExternalForce() {
        common::ThreadPool::forEach(_cbs.begin(), _cbs.end(), [this](auto& cb) {
            // gravity
            if (!cb->isDeformable()) {
                static_cast<pe_phys_object::RigidBody*>(cb)->addForce(pe::Vector3::zeros(), _gravity);
            }
        });
        common::ThreadPool::join();
    }

    void World::addCollisionBody(pe_phys_object::CollisionBody* cb) {
        _cbs.push_back(cb);
    }

    void World::step() {
        // external force
        applyExternalForce();

        // collision detection
        updateAABBs();
        _broad_phase->calcCollisionPairs(_cbs);
        _narrow_phase->clearContactResults();
        _narrow_phase->calcContactResults(_broad_phase->getCollisionPairs());

        // constraints
        _constraint_solver->setupSolver(_cbs, _narrow_phase->getContactResults(), _constraints);
        _constraint_solver->solve();

        // update status
        updateCollisionBodies();
    }

} // namespace pe_core