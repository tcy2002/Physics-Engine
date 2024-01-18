#include "world.h"
#include "common/thread_pool.h"

namespace pe_core {

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
        _narrow_phase->calcContactResults();

        // update status
        updateCollisionBodies();

        // TODO: Constraints solver
    }

} // namespace pe_core