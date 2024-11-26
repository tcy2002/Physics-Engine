#include "broad_phase_sweep_and_prune.h"
#include <algorithm>

namespace pe_phys_collision {

    void BroadPhaseSweepAndPrune::calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects,
                                                     pe::Array<CollisionPair>& pairs) {
        if (objects.size() < 2) return;
        pairs.clear();

        // sort collision objects by their min x value
        std::sort(objects.begin(), objects.end(),
                  [this](const pe_phys_object::RigidBody* cb1, const pe_phys_object::RigidBody* cb2) {
            return cb1->getAABBMin()[_target_axis] < cb2->getAABBMin()[_target_axis];
        });

        // sweep the sorted array and find collision pairs
        pe::Vector3 s = pe::Vector3::zeros(), s2 = pe::Vector3::zeros();
        for (int i = 0; i < I(objects.size()); i++) {
            // update sum and sum of squares to calculate mean and variance
            pe_phys_object::RigidBody* cb1 = objects[i];
            pe::Vector3 center = (cb1->getAABBMin() + cb1->getAABBMax()) * 0.5;
            s += center;
            s2 += center.mult(center);

            // test collision pairs
            for (int j = i + 1; j < I(objects.size()); j++) {
                pe_phys_object::RigidBody* cb2 = objects[j];
                if (cb2->getAABBMin()[_target_axis] > cb1->getAABBMax()[_target_axis]) break;
                if (validateCollisionPair(cb1, cb2)) {
                    pairs.emplace_back(cb1, cb2);
                }
            }
        }

        // update axis sorted to be the one with the largest variance
        pe::Vector3 v = s2 - s.mult(s) / R(objects.size());
        _target_axis = 0;
        if (v.y > v.x) _target_axis = 1;
        if (v.z > v[_target_axis]) _target_axis = 2;
    }

} // namespace pe_phys_collision