#pragma once

#include "rigid/shape/sphere_shape.h"
#include "rigid/shape/cylinder_shape.h"
#include "collision_algorithm.h"

namespace pe_phys_collision {

    class SphereCylinderCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real refScale, ContactResult& result) override;

        static bool getClosestPoints(pe_phys_shape::SphereShape* shape_sph, pe_phys_shape::CylinderShape* shape_cyl,
                                     const pe::Transform& trans_sph, const pe::Transform& trans_cyl,
                                     pe::Real margin, ContactResult& result);
    };

} // pe_phys_collision