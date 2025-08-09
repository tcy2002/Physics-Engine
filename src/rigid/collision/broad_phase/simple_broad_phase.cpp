#include "simple_broad_phase.h"

namespace pe_phys_collision {

    void SimpleBroadPhase::calcCollisionPairs(pe::Array<pe_phys_object::RigidBody*> objects,
                                             pe::Array<CollisionPair>& pairs) {
        if (objects.size() < 2) return;
        pairs.clear();

        for (int i = 0; i < PE_I(objects.size()); i++) {
            auto obj1 = objects[i];
            for (int j = i + 1; j < PE_I(objects.size()); j++) {
                auto obj2 = objects[j];
                if (validateCollisionPair(obj1, obj2)) {
                    pairs.emplace_back(obj1, obj2);
                }
            }
        }
    }

} // namespace pe_phys_collision