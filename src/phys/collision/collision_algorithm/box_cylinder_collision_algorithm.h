#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class BoxCylinderCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static bool pointInsideBox(const pe::Vector3& half_extent, const pe::Vector3& loc);
        static int findClosestBoxFace(const pe::Vector3& half_extent, const pe::Vector3& loc);
        static bool intersectLinePlane(const pe::Vector3& start_line, const pe::Vector3& dir_line, const pe::Vector3& plane_pos,
                                       const pe::Vector3& plane_nor, pe::Real tol, pe::Real& t);
        static bool intersectSegmentBox(const pe::Vector3& half_extent, const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl,
                                        pe::Real h_cyl, pe::Real tol, pe::Real &t_min, pe::Real& t_max);
        static bool intersectSegmentCylinder(const pe::Vector3& start_seg, const pe::Vector3& dir_seg, pe::Real len_seg,
                                             const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl, pe::Real h_cyl,
                                             pe::Real radius_cyl, pe::Real tol, pe::Real& t_min, pe::Real& t_max);
    };

} // pe_phys_collision