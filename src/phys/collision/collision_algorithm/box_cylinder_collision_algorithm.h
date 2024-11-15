#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    class BoxCylinderCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                      pe::Transform trans_a, pe::Transform trans_b,
                                      pe::Real refScale, ContactResult& result) override;

        static bool PointInsideBox(const pe::Vector3& hdims, const pe::Vector3& loc);
        static int FindClosestBoxFace(const pe::Vector3& hdims, const pe::Vector3& loc);
        static bool IntersectLinePlane(const pe::Vector3& lP, const pe::Vector3& lD, const pe::Vector3& pP,
                                const pe::Vector3& pN, const pe::Real tol, pe::Real& t);
        static bool IntersectSegmentBox(const pe::Vector3& hdims, const pe::Vector3& c, const pe::Vector3& a,
                                 const pe::Real hlen, const pe::Real tol, pe::Real &tMin, pe::Real& tMax);
        static bool IntersectSegmentCylinder(const pe::Vector3& sC, const pe::Vector3& sD, const pe::Real sH,
                                      const pe::Vector3& cC, const pe::Vector3& cD, const pe::Real cH,
                                      const pe::Real cR, const pe::Real tol, pe::Real& tMin, pe::Real& tMax);
    };

} // pe_phys_collision