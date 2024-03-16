#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CylinderConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result) override;

        static void getClosestPoints(pe_phys_object::RigidBody* object_a, pe::Vector3 vertices[],
                                     const pe::Transform& transTri, ContactResult& result);
        static bool collideCylinderTriangle(pe::Real cylinderRadius, pe::Real cylinderHeight,
                                            const pe::Vector3 vertices[], pe::Vector3& point,
                                            pe::Vector3& resultNormal, pe::Real& depth, pe::Real margin);
        static bool intersectEdgeCylinder(const pe::Vector3& p0, const pe::Vector3& p1,
                                          pe::Real radius, pe::Real height);
        static bool intersectTriangleCylinder(const pe::Vector3 vertices[], pe::Real radius, pe::Real height);
    };

} // pe_phys_collision