#include "simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs,
                                               pe::Array<ContactResult>& results) {
#   ifdef PE_MULTI_THREAD
        results.resize(pairs.size());
        auto c = this;
        utils::ThreadPool::forEach(pairs.begin(), pairs.end(),
                                   [&](const CollisionPair& pair, int idx) {
                                       auto type_a = pair.first->getCollisionShape()->getType();
                                       auto type_b = pair.second->getCollisionShape()->getType();
                                       int algo_idx = type_a * 4 + type_b;
                                       c->_algos[algo_idx] &&
                                       c->_algos[algo_idx]->processCollision(pair.first, pair.second,
                                                                             results[idx]);
                                   });
        utils::ThreadPool::join();

        // remove empty results
        for (int i = (int)results.size() - 1; i >= 0; i--) {
            if (results[i].getPointSize() == 0) {
                results.erase(results.begin() + i);
            }
        }

#   else
        for (auto& pair : pairs) {
            ContactResult result;
            auto type_a = pair.first->getCollisionShape()->getType();
            auto type_b = pair.second->getCollisionShape()->getType();
            int algo_idx = type_a * 4 + type_b;
            if (_algos[algo_idx] &&
                _algos[algo_idx]->processCollision(pair.first, pair.second, result)) {
                _contact_results.push_back(result);
            }
        }
#   endif
    }

} // namespace pe_phys_collision