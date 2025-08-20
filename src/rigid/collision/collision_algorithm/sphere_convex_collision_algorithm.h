#pragma once

#include "rigid/shape/sphere_shape.h"
#include "collision_algorithm.h"

namespace pe_phys_collision {

    class SphereConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real ref_scale, ContactResult& result) override;

        static bool getClosestPoints(pe_phys_shape::SphereShape* shape_sph, const pe::Transform& trans_sph,
                                     const pe::Vector3 vertices[], const pe::Transform& trans_tri,
                                     pe::Real margin, ContactResult& result);
        static bool getClosestPoints(pe_phys_shape::SphereShape* shape_sph, pe_phys_shape::Shape* shape_mesh,
                                     const pe::Mesh& mesh, const pe::Transform& trans_sph, const pe::Transform& trans_mesh,
                                     pe::Real margin, ContactResult& result);
    };

} // pe_phys_collision