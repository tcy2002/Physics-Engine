#pragma once

#include "phys/object/collision_object.h"
#include "phys/shape/shape.h"
#include "phys/collision/narrow_phase/contact_result.h"

namespace pe_phys_collision {

    class CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::CollisionBody* body_a, pe_phys_object::CollisionBody* body_b,
                                      ContactResult& result, pe::Vector3 overlapMin, pe::Vector3 overlapMax) = 0;

        static bool isInsideTriangle(const pe::Array<pe::Vector3>& triangle, const pe::Vector3& normal,
                                     const pe::Vector3& point, pe::Real tolerance);
        static void calcTriangleAABB(const pe::Array<pe::Vector3>& triangle, pe::Vector3& min, pe::Vector3& max);
        static bool isSeparateOnAxis(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                     const pe::Transform& transform_a, const pe::Transform& transform_b,
                                     pe::Vector3& axis, pe::Real& depth,
                                     pe::Vector3& contact_point_a, pe::Vector3& contact_point_b);
        static bool findSeparateAxis(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                     const pe::Transform& transform_a, const pe::Transform& transform_b,
                                     const pe::Vector3& overlapMin, const pe::Vector3& overlapMax,
                                     pe::Vector3& axis, ContactResult& result);
    };

} // pe_phys_collision
