#pragma once

#include "phys/object/rigidbody.h"

namespace pe_phys_raycast {

    struct RaycastResult {
        pe::Real m_distance = PE_REAL_MAX;
        pe::Vector3 m_hitPoint = pe::Vector3::zeros();
        pe::Vector3 m_hitNormal = pe::Vector3::zeros();
        const pe_phys_object::RigidBody* m_collisionObject = nullptr;
        bool hasHit() const { return m_collisionObject != nullptr; }
    };

    class RaycastSolverBase {
    public:
        RaycastSolverBase() {}
        virtual ~RaycastSolverBase() {}
        virtual bool performRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real length,
                                    const pe::Array<pe_phys_object::RigidBody*>& objects,
                                    const pe::Uint32HashList& ignores,
                                    RaycastResult& result) = 0;
    };

} // namespace pe_phys_ray