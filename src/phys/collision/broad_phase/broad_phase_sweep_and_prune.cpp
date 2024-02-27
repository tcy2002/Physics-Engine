#include "broad_phase_sweep_and_prune.h"
#include <algorithm>

namespace pe_phys_collision {

    bool BroadPhaseBase::validateCollisionPair(pe_phys_object::CollisionObject* cb1, pe_phys_object::CollisionObject* cb2) const {
        if (!cb1 || !cb2) return false;
        if (cb1->getGlobalId() == cb2->getGlobalId()) return false;
        if (cb1->isKinematic() && cb2->isKinematic()) return false;
        if (cb1->isIgnoreCollisionId(cb2->getGlobalId())) return false;
        if (cb2->isIgnoreCollisionId(cb1->getGlobalId())) return false;
        return true;
    }

    bool BroadPhaseBase::testCollisionPair(pe_phys_object::CollisionObject* cb1, pe_phys_object::CollisionObject* cb2) const {
        if (!validateCollisionPair(cb1, cb2)) return false;
        auto &min1 = cb1->getAABBMin(), &max1 = cb1->getAABBMax();
        auto &min2 = cb2->getAABBMin(), &max2 = cb2->getAABBMax();
        if (min1.x > max2.x || min2.x > max1.x) return false;
        if (min1.y > max2.y || min2.y > max1.y) return false;
        if (min1.z > max2.z || min2.z > max1.z) return false;
        return true;
    }

    void BroadPhaseSweepAndPrune::calcCollisionPairs(pe::Array<pe_phys_object::CollisionObject*> collision_objects) {
        if (collision_objects.size() < 2) return;

        // sort collision objects by their min x value
        std::sort(collision_objects.begin(), collision_objects.end(),
                  [this](pe_phys_object::CollisionObject* cb1, pe_phys_object::CollisionObject* cb2) {
            return cb1->getAABBMin()[_target_axis] < cb2->getAABBMin()[_target_axis];
        });

        // sweep the sorted array and find collision pairs
        pe::Vector3 s = pe::Vector3::zeros(), s2 = pe::Vector3::zeros();
        for (int i = 0; i < (int)collision_objects.size(); i++) {
            // update sum and sum of squares to calculate mean and variance
            pe_phys_object::CollisionObject* cb1 = collision_objects[i];
            pe::Vector3 center = (cb1->getAABBMin() + cb1->getAABBMax()) * 0.5;
            s += center;
            s2 += center * center;

            // test collision pairs
            for (int j = i + 1; j < (int)collision_objects.size(); j++) {
                pe_phys_object::CollisionObject* cb2 = collision_objects[j];
                if (cb2->getAABBMin()[_target_axis] > cb1->getAABBMax()[_target_axis]) break;
                if (validateCollisionPair(cb1, cb2) && testCollisionPair(cb1, cb2)) {
                    _collision_pairs.emplace_back(cb1, cb2);
                }
            }
        }

        // update axis sorted to be the one with the largest variance
        pe::Vector3 v = s2 - s * s / (int)collision_objects.size();
        if (v.y > v.x) _target_axis = 1;
        if (v.z > v[_target_axis]) _target_axis = 2;
    }

} // namespace pe_phys_collision