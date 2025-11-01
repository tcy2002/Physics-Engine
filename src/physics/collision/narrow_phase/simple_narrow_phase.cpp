#include "simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_physics_collision {

void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs,
                                           pe::Array<ContactResult*>& results) {
    // clear old contact results
    const int old_size = PE_I(results.size());
    const int new_size = PE_I(pairs.size());
    if (old_size < new_size) {
        results.resize(new_size);
        for (int i = old_size; i < new_size; i++) {
            results[i] = _cr_pool.create();
        }
    } else {
        for (int i = new_size; i < old_size; i++) {
			_cr_pool.destroy(results[i]);
        }
        results.resize(new_size);
    }

#ifdef PE_MULTI_THREAD
    utils::ThreadPool::forLoop(PE_UI(pairs.size()), [&](int i) {
        results[i]->clearContactPoints();
        pe_physics_object::RigidBody* obj_a = pairs[i].first, *obj_b = pairs[i].second;
        if (obj_a->getGlobalId() < obj_b->getGlobalId()) PE_SWAP(obj_a, obj_b);
        results[i]->setObjects(obj_a, obj_b);
        const auto shape_a = obj_a->getCollisionShape();
        const auto shape_b = obj_b->getCollisionShape();
        const auto type_a = shape_a->getType();
        const auto type_b = shape_b->getType();
        auto& trans_a = obj_a->getTransform();
        auto& trans_b = obj_b->getTransform();
        const auto algo = getAlgorithm(type_a, type_b);
        if (algo == nullptr) return;
        const pe::Real refScale = (obj_a->getAABBScale() + obj_b->getAABBScale()) * PE_DIST_REF_RADIO;
        algo->processCollision(shape_a, shape_b, trans_a, trans_b, refScale, *results[i]);
        results[i]->sortContactPoints();
    });
#else
    for (int i = 0; i < I(pairs.size()); i++) {
        results[i]->clearContactPoints();
        pe_phys_object::RigidBody* obj_a = pairs[i].first, *obj_b = pairs[i].second;
        if (obj_b->getTag() == "wheel") PE_SWAP(obj_a, obj_b);
        results[i]->setObjects(obj_a, obj_b);
        const auto shape_a = obj_a->getCollisionShape();
        const auto shape_b = obj_b->getCollisionShape();
        const auto type_a = shape_a->getType();
        const auto type_b = shape_b->getType();
        auto& trans_a = obj_a->getTransform();
        auto& trans_b = obj_b->getTransform();
        const auto algo = getAlgorithm(type_a, type_b);
        if (algo == nullptr) return;
        const pe::Real refScale = (obj_a->getAABBScale() + obj_b->getAABBScale()) * PE_DIST_REF_RADIO;
        algo->processCollision(shape_a, shape_b, trans_a, trans_b, refScale, *results[i]);
        results[i]->sortContactPoints();
    }
#endif
}

} // namespace pe_physics_collision
