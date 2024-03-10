#include "simple_narrow_phase.h"
#include "core/viewer.h"
#include "utils/thread_pool.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs) {
#   ifdef PE_MULTI_THREAD
        _contact_results.resize(pairs.size());
        utils::ThreadPool::forEach(pairs.begin(), pairs.end(),
                                   [this](const CollisionPair& pair, int idx) {
            ContactResult result;
            pe::Vector3 overlap_min, overlap_max;
            auto type_a = pair.first->getCollisionShape()->getType();
            auto type_b = pair.second->getCollisionShape()->getType();
            bool ret = false;
            if (type_a == pe_phys_shape::ShapeType::Box &&
                type_b == pe_phys_shape::ShapeType::Box) {
                ret = _algos[0]->processCollision(pair.first, pair.second, result,
                                                  overlap_min, overlap_max);
            } else if (type_a == pe_phys_shape::ShapeType::ConvexMesh &&
                       type_b == pe_phys_shape::ShapeType::ConvexMesh) {
                ret = _algos[1]->processCollision(pair.first, pair.second, result,
                                                  overlap_min, overlap_max);
            }
            if (ret) {
                _contact_results[idx] = result;
            }
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
            pe::Vector3 overlap_min, overlap_max;
            auto type_a = pair.first->getCollisionShape()->getType();
            auto type_b = pair.second->getCollisionShape()->getType();
            bool ret = false;
            if (type_a == pe_phys_shape::ShapeType::Box &&
                type_b == pe_phys_shape::ShapeType::Box) {
                ret = _algos[0]->processCollision(pair.first, pair.second, result,
                                                  overlap_min, overlap_max);
            } else if (type_a == pe_phys_shape::ShapeType::ConvexMesh &&
                       type_b == pe_phys_shape::ShapeType::ConvexMesh) {
                ret = _algos[1]->processCollision(pair.first, pair.second, result,
                                                  overlap_min, overlap_max);
            }
            if (ret) {
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