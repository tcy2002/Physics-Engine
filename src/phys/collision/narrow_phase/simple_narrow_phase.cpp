#include "simple_narrow_phase.h"
#include "phys/collision/collision_algorithm/box_box_collision_algorithm.h"

namespace pe_phys_collision {

    void SimpleNarrowPhase::calcContactResults(const pe::Array<CollisionPair>& pairs) {
        // TODO: map pairs to collision algorithms
        static CollisionAlgorithm* alg = new BoxBoxCollisionAlgorithm();
        for (auto& pair : pairs) {
            ContactResult result;
            pe::Vector3 overlap_min, overlap_max;
            if (alg->processCollision(pair.first, pair.second, result, overlap_min, overlap_max)) {
                _contact_results.push_back(result);
            }
        }
    }

    void SimpleNarrowPhase::clearContactResults() {
        _contact_results.clear();
    }

    pe::Array<ContactResult>& SimpleNarrowPhase::getContactResults() {
        return _contact_results;
    }

} // namespace pe_phys_collision