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
                                       auto type_a = pair.first->getCollisionShape()->getType();
                                       auto type_b = pair.second->getCollisionShape()->getType();
                                       int algo_idx = type_a * 4 + type_b;
                                       c->_algos[algo_idx] &&
                                       c->_algos[algo_idx]->processCollision(pair.first, pair.second,
                                                                             *results[idx]);
                                   });
        utils::ThreadPool::join();
#   else
        for (int i = 0; i < (int)pairs.size(); i++) {
            auto type_a = pairs[i].first->getCollisionShape()->getType();
            auto type_b = pairs[i].second->getCollisionShape()->getType();
            int algo_idx = type_a * 4 + type_b;
            _algos[algo_idx] &&
            _algos[algo_idx]->processCollision(pairs[i].first, pairs[i].second, *results[i]);
        }
#   endif

        // remove empty results
        for (int i = (int)results.size() - 1; i >= 0; i--) {
            if (results[i]->getPointSize() == 0) {
                _cr_pool.destroy(results[i]);
                results.erase(results.begin() + i);
            }
        }
    }

} // namespace pe_phys_collision