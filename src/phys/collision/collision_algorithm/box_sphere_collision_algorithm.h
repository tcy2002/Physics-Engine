#pragma once

#include "collision_algorithm.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/sphere_shape.h"

namespace pe_phys_collision {

    class BoxSphereCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result) override;

        static bool getSphereDistance(const pe_phys_shape::BoxShape* boxShape, const pe::Transform& boxTrans,
                                      const pe::Vector3& sphereCenter, pe::Real radius,
                                      pe::Vector3& ptOnBox, pe::Vector3& normal, pe::Real& dist);
        static pe::Real getSpherePenetration(const pe::Vector3& boxHalfExt, const pe::Vector3& sphereRelPos,
                                             pe::Vector3& closestPoint, pe::Vector3& normal);
    };

} // pe_phys_collision