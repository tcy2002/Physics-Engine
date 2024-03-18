#include "simple_narrow_phase.h"
#include "utils/thread_pool.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs) {
#   ifdef PE_MULTI_THREAD
        _contact_results.resize(pairs.size());
        auto c = this;
        utils::ThreadPool::forEach(pairs.begin(), pairs.end(),
                                   [&c](const CollisionPair& pair, int idx) {
                                       auto type_a = pair.first->getCollisionShape()->getType();
                                       auto type_b = pair.second->getCollisionShape()->getType();
                                       int algo_idx = type_a * 4 + type_b;
                                       c->_algos[algo_idx] &&
                                       c->_algos[algo_idx]->processCollision(pair.first, pair.second,
                                                                             c->_contact_results[idx]);
                                   });
        utils::ThreadPool::join();

        // remove empty results
        for (int i = (int)_contact_results.size() - 1; i >= 0; i--) {
            if (_contact_results[i].getPointSize() == 0) {
                _contact_results.erase(_contact_results.begin() + i);
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

    void SimpleNarrowPhase::clearContactResults() {
        _contact_results.clear();
    }

    pe::Array<ContactResult>& SimpleNarrowPhase::getContactResults() {
        return _contact_results;
    }

} // namespace pe_phys_collision