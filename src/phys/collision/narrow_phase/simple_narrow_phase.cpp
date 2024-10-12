#include "simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs,
                                               pe::Array<ContactResult*>& results) {
        // clear old contact results
        const int old_size = (int)results.size();
        const int new_size = (int)pairs.size();
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

#   ifdef PE_MULTI_THREAD
        auto c = this;
        utils::ThreadPool::forEach(pairs.begin(), pairs.end(),
                                   [&](const CollisionPair& pair, int idx) {
                                       auto shape_a = pair.first->getCollisionShape();
                                       auto shape_b = pair.second->getCollisionShape();
                                       auto trans_a = pair.first->getTransform();
                                       auto trans_b = pair.second->getTransform();
                                       auto type_a = shape_a->getType();
                                       auto type_b = shape_b->getType();
                                       results[idx]->clearContactPoints();
                                       results[idx]->setObjects(pairs[idx].first, pairs[idx].second);
                                       getAlgorithm(type_a, type_b)->processCollision(shape_a, shape_b, trans_a, trans_b, *results[idx]);
                                       results[idx]->sortContactPoints();
                                   });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < (int)pairs.size(); i++) {
            auto shape_a = pairs[i].first->getCollisionShape();
            auto shape_b = pairs[i].second->getCollisionShape();
            auto trans_a = pairs[i].first->getTransform();
            auto trans_b = pairs[i].second->getTransform();
            auto type_a = shape_a->getType();
            auto type_b = shape_b->getType();
            results[i]->clearContactPoints();
            results[i]->setObjects(pairs[i].first, pairs[i].second);
            getAlgorithm(type_a, type_b)->processCollision(shape_a, shape_b, trans_a, trans_b, *results[i]);
            results[i]->sortContactPoints();
        }
#   endif

        // remove empty contact results
        for (int i = (int)results.size() - 1; i >= 0; i--) {
            if (results[i]->getPointSize() == 0) {
                _cr_pool.destroy(results[i]);
                results.erase(results.begin() + i);
            }
        }
    }

} // namespace pe_phys_collision