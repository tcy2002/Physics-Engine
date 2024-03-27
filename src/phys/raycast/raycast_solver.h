#pragma once

#include <functional>
#include "phys/object/rigidbody.h"

namespace pe_phys_ray {
    
    class RaycastSolver {
    public:
        struct RayResultCallback {
            pe::Real m_distance;
            pe::Vector3 m_hitPoint;
            pe::Vector3 m_normal;
            const pe_phys_object::RigidBody* m_collisionObject;

            virtual ~RayResultCallback() {}
            bool hasHit() const { return (m_collisionObject != 0); }

            RayResultCallback();
        };

        pe::Vector3 m_start, m_direction, m_localStart, m_localDirection;
        pe::Real m_length;
        RayResultCallback* m_resultCallback;
        std::function<void(pe::Real, pe::Vector3)> m_callback;
        std::function<void(RayResultCallback*)> m_closestHitCallback;

    public:
        RaycastSolver(pe::Vector3 start, pe::Vector3 direction, pe::Real length);
        virtual ~RaycastSolver() { delete m_resultCallback; }
        static pe::Real maxItem(pe::Vector3 vec);
        static pe::Real minItem(pe::Vector3 vec);
        void setDirection(const pe::Matrix3& local2world) { m_direction = local2world * m_localDirection; }
        void setStart(const pe::Transform& trans) { m_start = trans * m_localStart; }
        void bindCallback(const std::function<void(pe::Real, pe::Vector3)>& callback) { m_callback = callback; }
        void bindClosetHitCallback(const std::function<void(RayResultCallback*)>& callback)
        { m_closestHitCallback = callback; }
        void performRayTest(uint32_t id, const pe::Array<pe_phys_object::RigidBody*>& objs,
                            const pe::Uint32HashList& excludeIds);
    };
    
} // namespace pe_phys_ray