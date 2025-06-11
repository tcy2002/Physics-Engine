#pragma once

#include "collision_algorithm.h"
#include "rigid/shape/box_shape.h"
#include "rigid/shape/sphere_shape.h"

namespace pe_phys_collision {

    class BoxSphereCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real refScale, ContactResult& result) override;

        static bool getClosestPoints(pe_phys_shape::SphereShape* shape_sph, pe_phys_shape::BoxShape* shape_box,
                                     const pe::Transform& trans_sph, const pe::Transform& trans_box,
                                     const pe::Vector3& center_sph, pe::Real radius_sph,
                                     pe::Real margin, ContactResult& result);
    };

} // pe_phys_collision