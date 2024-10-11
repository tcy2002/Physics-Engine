#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class SphereConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      ContactResult& result) override;

        static void getClosestPoints(pe_phys_shape::Shape* shape_a, const pe::Transform& trans_a,
                                     const pe::Vector3 vertices[],
                                     const pe::Transform& transTri, ContactResult& result);
        static bool collideSphereTriangle(const pe::Vector3& sphereCenter, pe::Real sphereRadius,
                                          const pe::Vector3 vertices[], pe::Vector3& point,
                                          pe::Vector3& resultNormal, pe::Real& depth);
        static bool pointInTriangle(const pe::Vector3 vertices[], const pe::Vector3& normal, pe::Vector3* p);
        static bool faceContains(const pe::Vector3& p, const pe::Vector3* vertices, pe::Vector3& normal);
    };

} // pe_phys_collision