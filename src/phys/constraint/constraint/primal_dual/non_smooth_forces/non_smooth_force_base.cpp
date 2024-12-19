#include "non_smooth_force_base.h"

namespace pe_phys_constraint {

    void NonSmoothForceBase::linearSystemReserve(const pe::Array<pe_phys_collision::ContactResult*>& contacts,
        const pe::Array<pe_phys_object::RigidBody*>& objects,
        const pe::Map<pe_phys_object::RigidBody*, size_t>& object2index,
        pe::Set<pe::KV<size_t, size_t>>& obj_pairs,
        pe::Array<Eigen::Triplet<pe::Real>>& triplets) {
        // checked1
        for (const auto contact : contacts) {
            if (contact->getPointSize() == 0) {
                continue;
            }
            int obj1 = PE_I(object2index.at(contact->getObjectA()));
            int obj2 = PE_I(object2index.at(contact->getObjectB()));
            const int obj_ids[2] = { obj1, obj2 };
            if (obj1 > obj2) PE_SWAP(obj1, obj2);
            if (obj_pairs.count({ obj1, obj2 })) continue;
            obj_pairs.insert({ obj1, obj2 });

            for (int i = 0; i < 2; i++) {
                if (objects[obj_ids[i]]->isKinematic()) {
                    continue;
                }

                for (int j = 0; j < 2; j++) {
                    if (objects[obj_ids[j]]->isKinematic()) {
                        continue;
                    }

                    for (int row = 0; row < 6; row++) {
                        for (int col = 0; col < 6; col++) {
                            triplets.push_back({ obj_ids[i] * 6 + row, obj_ids[j] * 6 + col, 1 });
                        }
                    }
                }
            }
        }
    }

} // namespace pe_phys_constraint