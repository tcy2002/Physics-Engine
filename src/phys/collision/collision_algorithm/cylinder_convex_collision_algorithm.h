#pragma once

#include "phys/shape/cylinder_shape.h"
#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CylinderConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real refScale, ContactResult& result) override;

        static bool getClosestPoints(pe_phys_shape::Shape* shape_o, pe_phys_shape::CylinderShape* shape_cyl,
                                     const pe::Transform& trans_o, const pe::Transform& trans_cyl,
                                     pe::Real margin, ContactResult& result);

        static bool intersectSegmentFace(const pe::Mesh::Face& face, const pe::Mesh& mesh,
                                         const pe::Vector3& pos_seg, const pe::Vector3& dir_seg, pe::Real l_seg,
                                         pe::Real margin, pe::Real& t1, pe::Real& t2, pe::Real& d1, pe::Real& d2);
    };

} // pe_phys_collision