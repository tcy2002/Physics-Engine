#include "raycast_solver.h"

namespace pe_phys_ray {

    Raycast::RayResultCallback::RayResultCallback() :
            m_distance(pe::Real(0.)),
            m_hitPoint(pe::Vector3(0, 0, 0)),
            m_collisionObject(0),
            m_normal(pe::Vector3(0, 0, 0)) {}

    Raycast::Raycast(pe::Vector3 start, pe::Vector3 direction, pe::Real length) {
        m_localStart = start;
        m_start = start;
        m_localDirection = direction;
        m_direction = direction;
        m_length = length;
        m_resultCallback = new RayResultCallback();
        m_resultCallback->m_distance = length;
        m_closestHitCallback = nullptr;
    }

    pe::Real Raycast::maxItem(pe::Vector3 vec) {
        if (vec.x > vec.y) {
            if (vec.z > vec.x) return vec.z;
            else return vec.x;
        } else {
            if (vec.z > vec.y) return vec.z;
            else return vec.y;
        }
    }

    pe::Real Raycast::minItem(pe::Vector3 vec) {
        if (vec.x < vec.y) {
            if (vec.z < vec.x) return vec.z;
            else return vec.x;
        } else {
            if (vec.z < vec.y) return vec.z;
            else return vec.y;
        }
    }

    void Raycast::performRayTest(uint32_t id, const pe::Array<pe_phys_object::RigidBody *> &objs,
                                 const pe::HashList<uint32_t>& excludeIds) {
        //avoid division by zero
        pe::Vector3 tmp_direction = pe::Vector3(m_direction.x != 0 ? m_direction.x : 0.001,
                                                m_direction.y != 0 ? m_direction.y : 0.001,
                                                m_direction.z != 0 ? m_direction.z : 0.001);
        pe::Real d_min = m_length;
        for (int i = 0; i < objs.size(); i++) {
            if (objs[i]->getGlobalId() != id && !excludeIds.contains(objs[i]->getGlobalId())) {
                //test AABB intersection
                pe_phys_object::RigidBody* obj = objs[i];
                pe::Real offset = obj->getAABBScale();
                pe::Transform t = obj->getTransform();
                pe::Vector3 pos = t.getOrigin();
                pe::Real d_projection = (pos - m_start).dot(m_direction);
                pe::Vector3 AabbMin = obj->getAABBMin();
                pe::Vector3 AabbMax = obj->getAABBMax();

                if (d_projection < d_min + 2 * offset && d_projection > -d_min - 2 * offset) {
                    pe::Real d2_center = (pos - m_start).norm() - d_projection * d_projection;
                    if (d2_center < offset * offset) {

                        pe::Vector3 diagonal = AabbMax - AabbMin;
                        //find closest and furthest point projected on direction
                        pe::Vector3 closest_point = pe::Vector3((diagonal.x * tmp_direction.x) > 0 ?
                                AabbMin.x : AabbMax.x, (diagonal.y * tmp_direction.y) > 0 ?
                                AabbMin.y : AabbMax.y, (diagonal.z * tmp_direction.z) > 0 ?
                                AabbMin.z : AabbMax.z);
                        pe::Vector3 furthest_point = pe::Vector3((diagonal.x * tmp_direction.x) < 0 ?
                                AabbMin.x : AabbMax.x, (diagonal.y * tmp_direction.y) < 0 ?
                                AabbMin.y : AabbMax.y, (diagonal.z * tmp_direction.z) < 0 ?
                                AabbMin.z : AabbMax.z);
                        pe::Vector3 d1 = closest_point - m_start;
                        pe::Vector3 d2 = furthest_point - m_start;
                        pe::Vector3 closest_intersections = pe::Vector3(d1.x / tmp_direction.x,
                                                                        d1.y / tmp_direction.y,
                                                                        d1.z / tmp_direction.z);
                        pe::Vector3 furthest_intersections = pe::Vector3(d2.x / tmp_direction.x,
                                                                         d2.y / tmp_direction.y,
                                                                         d2.z / tmp_direction.z);

                        pe::Real closest_max = maxItem(closest_intersections);
                        pe::Real furthest_min = minItem(furthest_intersections);
                        if (closest_max < furthest_min && closest_max < d_min && closest_max > 0) {
                            //d_min = d_projection;      //center distance
                            d_min = closest_max;          //face distance
                            m_resultCallback->m_collisionObject = obj;
                        }

                    }
                }

            }
        }
        m_resultCallback->m_distance = d_min;
        m_resultCallback->m_hitPoint = m_start + d_min * m_direction;
        m_resultCallback->m_normal = pe::Vector3(0, 1, 0); // temporary;

        if (m_closestHitCallback && m_resultCallback->m_distance != m_length) {
            m_closestHitCallback(m_resultCallback);
        }
    }

} // namespace pe_phys_ray