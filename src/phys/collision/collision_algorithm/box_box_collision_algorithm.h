#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

    typedef pe::Real dMatrix3[4 * 3];

    class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                      ContactResult& result, pe::Vector3 overlapMin, pe::Vector3 overlapMax) override;

        void getClosestPoint(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b,
                                    ContactResult& result);
        static void dLineClosestApproach(const pe::Vector3& pa, const pe::Vector3& ua,
                                         const pe::Vector3& pb, const pe::Vector3& ub,
                                         pe::Real& alpha, pe::Real& beta);
        static int intersectRectQuad2(pe::Real h[2], pe::Real p[8], pe::Real ret[16]);
        static void cullPoints2(int n, pe::Real p[], int m, int i0, int i_ret[]);
        static int dBoxBox2(const pe::Vector3& p1, const dMatrix3 R1, const pe::Vector3& side1,
                            const pe::Vector3& p2, const dMatrix3 R2, const pe::Vector3& side2,
                            pe::Vector3& normal, pe::Real margin, pe::Real& depth, int& return_code,
                            int max_c, ContactResult& result);
    };

} // pe_phys_collision