#include "broad_phase_sweep_and_prune.h"
#include <algorithm>

namespace pe_phys_collision {

    bool BroadPhaseBase::validateCollisionPair(pe_phys_object::RigidBody* co1, pe_phys_object::RigidBody* co2) const {
        if (!co1 || !co2) return false;
        if (co1->getGlobalId() == co2->getGlobalId()) return false;
        if (co1->isKinematic() && co2->isKinematic()) return false;
        if (co1->isIgnoreCollisionId(co2->getGlobalId())) return false;
        if (co2->isIgnoreCollisionId(co1->getGlobalId())) return false;
        return true;
    }

    bool BroadPhaseBase::testCollisionPair(pe_phys_object::RigidBody* co1, pe_phys_object::RigidBody* co2) const {
        if (!validateCollisionPair(co1, co2)) return false;
        auto &min1 = co1->getAABBMin(), &max1 = co1->getAABBMax();
        auto &min2 = co2->getAABBMin(), &max2 = co2->getAABBMax();
        if (min1.x > max2.x || min2.x > max1.x) return false;
        if (min1.y > max2.y || min2.y > max1.y) return false;
        if (min1.z > max2.z || min2.z > max1.z) return false;
        return true;
    }

    void BroadPhaseSweepAndPrune::calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects) {
        if (objects.size() < 2) return;

        // sort collision objects by their min x value
        std::sort(objects.begin(), objects.end(),
                  [this](pe_phys_object::RigidBody* cb1, pe_phys_object::RigidBody* cb2) {
            return cb1->getAABBMin()[_target_axis] < cb2->getAABBMin()[_target_axis];
        });

        // sweep the sorted array and find collision pairs
        pe::Vector3 s = pe::Vector3::zeros(), s2 = pe::Vector3::zeros();
        for (int i = 0; i < (int)objects.size(); i++) {
            // update sum and sum of squares to calculate mean and variance
            pe_phys_object::RigidBody* cb1 = objects[i];
            pe::Vector3 center = (cb1->getAABBMin() + cb1->getAABBMax()) * 0.5;
            s += center;
            s2 += center * center;

            // test collision pairs
            for (int j = i + 1; j < (int)objects.size(); j++) {
                pe_phys_object::RigidBody* cb2 = objects[j];
                if (cb2->getAABBMin()[_target_axis] > cb1->getAABBMax()[_target_axis]) break;
                if (validateCollisionPair(cb1, cb2) && testCollisionPair(cb1, cb2)) {
                    _collision_pairs.emplace_back(cb1, cb2);
                }
            }
        }

        // update axis sorted to be the one with the largest variance
        pe::Vector3 v = s2 - s * s / (int)objects.size();
        if (v.y > v.x) _target_axis = 1;
        if (v.z > v[_target_axis]) _target_axis = 2;
    }

} // namespace pe_phys_collision