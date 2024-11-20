#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class CylinderConvexCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static bool intersectSegmentFace(const pe::Mesh::Face& face, const pe::Mesh& mesh,
                                         const pe::Vector3& pos_seg, const pe::Vector3& dir_seg, pe::Real l_seg,
                                         pe::Real margin, pe::Real& t1, pe::Real& t2, pe::Real& d1, pe::Real& d2);
        // static bool pointInsideCylinder(const pe::Vector3& v, pe::Real h, pe::Real r,
        //                                 const pe::Transform& trans, pe::Real margin, ContactResult& result);
        // static void intersectSegmentCylinder(const pe::Vector3& v1, const pe::Vector3& v2,
        //                                      pe::Real h, pe::Real r, const pe::Transform& trans,
        //                                      pe::Real margin, ContactResult& result);
        // static void intersectFaceCylinder(const pe::Array<pe::Vector3>& vs, const pe::Vector3& n, pe::Real h, pe::Real r,
        //                                   const pe::Transform& trans, pe::Real margin, ContactResult& result);
    };

} // pe_phys_collision