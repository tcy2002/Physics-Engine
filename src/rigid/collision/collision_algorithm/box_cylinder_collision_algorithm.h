#pragma once

#include "rigid/shape/cylinder_shape.h"
#include "rigid/shape/box_shape.h"
#include "collision_algorithm.h"

namespace pe_phys_collision {

    class BoxCylinderCollisionAlgorithm : public CollisionAlgorithm {
    public:
        bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                              pe::Transform trans_a, pe::Transform trans_b,
                              pe::Real refScale, ContactResult& result) override;

        static bool getClosestPoints(pe_phys_shape::BoxShape* shape_box, pe_phys_shape::CylinderShape* shape_cyl,
                                     const pe::Transform& trans_box, const pe::Transform& trans_cyl,
                                     pe::Real radius_cyl, pe::Real height_cyl, const pe::Vector3& box_half_extent,
                                     pe::Real margin, ContactResult& result);

        static void addContactPointOnBox(const pe::Vector3& pc, int i_face, const pe::Vector3& half_extent,
                                         const pe::Transform& trans_other, pe::Real margin, ContactResult& result);
        static void addContactPointOnCylinder(const pe::Vector3& p,
                                              const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl, pe::Real radius_cyl, pe::Real h_cyl,
                                              const pe::Transform& trans_other, pe::Real margin, ContactResult& result);

        static bool intersectSegmentBox(const pe::Vector3& half_extent, const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl,
                                        pe::Real h_cyl, pe::Real tol, pe::Real& t_min, pe::Real& t_max);
        static bool intersectSegmentCylinder(const pe::Vector3& start_seg, const pe::Vector3& dir_seg, pe::Real len_seg,
                                             const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl, pe::Real h_cyl, pe::Real radius_cyl,
                                             pe::Real tol, pe::Real &t_min, pe::Real &t_max);

    };

} // pe_phys_collision