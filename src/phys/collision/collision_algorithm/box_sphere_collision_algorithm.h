#pragma once

#include "collision_algorithm.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/sphere_shape.h"

namespace pe_phys_collision {

    class BoxSphereCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static bool getSphereDistance(const pe_phys_shape::BoxShape* shape_box, const pe::Transform& trans_box,
                                      const pe::Vector3& center_sph, pe::Real radius_sph,
                                      pe::Vector3& pt_on_box, pe::Vector3& normal, pe::Real& dist);
        static pe::Real getSpherePenetration(const pe::Vector3& half_extent, const pe::Vector3& pos_sph2box,
                                             pe::Vector3& closest_point, pe::Vector3& normal);
    };

} // pe_phys_collision