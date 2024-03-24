#include "broad_phase_base.h"

namespace pe_phys_collision {

    bool BroadPhaseBase::validateCollisionPair(pe_phys_object::RigidBody* co1, pe_phys_object::RigidBody* co2) const {
        if (!co1 || !co2) return false;
        if (!co1->getCollisionShape() || !co2->getCollisionShape()) return false;
        if (co1->getGlobalId() == co2->getGlobalId()) return false;
        if (co1->isIgnoreCollisionId(co2->getGlobalId())) return false;
        if (co2->isIgnoreCollisionId(co1->getGlobalId())) return false;
        auto &min1 = co1->getAABBMin(), &max1 = co1->getAABBMax();
        auto &min2 = co2->getAABBMin(), &max2 = co2->getAABBMax();
        if (min1.x > max2.x || min2.x > max1.x) return false;
        if (min1.y > max2.y || min2.y > max1.y) return false;
        if (min1.z > max2.z || min2.z > max1.z) return false;
        return true;
    }

} // namespace pe_phys_collision