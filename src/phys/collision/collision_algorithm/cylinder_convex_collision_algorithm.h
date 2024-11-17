#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CylinderConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static bool pointInsideCylinder(const pe::Vector3& v, pe::Real h, pe::Real r,
                                        const pe::Transform& trans, pe::Real margin, ContactResult& result);
        static void intersectSegmentCylinder(const pe::Vector3& v1, const pe::Vector3& v2,
                                             const pe::Vector3 &n, pe::Real h, pe::Real r,
                                             const pe::Transform& trans, pe::Real margin, ContactResult& result);
        static void intersectTriangleCylinder(const pe::Vector3 vs[3], const pe::Vector3& n, pe::Real h, pe::Real r,
                                              const pe::Transform& trans, pe::Real margin, ContactResult& result);
    };

} // pe_phys_collision