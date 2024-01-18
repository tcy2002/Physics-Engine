#pragma once

#include "collision_algorithm.h"

namespace pe_phys_collision {

#   define dDOTpq(a, b, p, q) ((a)[0] * (b)[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)])
#   define dMULTIPLY0_331(A, B, C) {   \
		(A)[0] = dDOT41((B), (C));     \
		(A)[1] = dDOT41((B + 4), (C)); \
		(A)[2] = dDOT41((B + 8), (C)); \
	}
#   define dMULTIPLY1_331(A, B, C) {   \
		(A)[0] = dDOT41((B), (C));     \
		(A)[1] = dDOT41((B + 1), (C)); \
		(A)[2] = dDOT41((B + 2), (C)); \
	}

    typedef pe::Real dMatrix3[4 * 3];

    class BoxBoxCollisionAlgorithm : public CollisionAlgorithm {
    public:
        virtual bool processCollision(pe_phys_object::CollisionBody* body_a, pe_phys_object::CollisionBody* body_b,
                                      ContactResult& result, pe::Vector3 overlapMin, pe::Vector3 overlapMax) override;

        static void getClosestPoints(pe_phys_object::CollisionBody* body_a, pe_phys_object::CollisionBody* body_b,
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

        static pe::Real dDOT(const pe::Real* a, const pe::Vector3& b) { return dDOTpq(a, b, 1, 1); }
        static pe::Real dDOT44(const pe::Real* a, const pe::Vector3& b) { return dDOTpq(a, b, 4, 4); }
        static pe::Real dDOT14(const pe::Real* a, const pe::Vector3& b) { return dDOTpq(a, b, 1, 4); }
        static pe::Real dDOT41(const pe::Real* a, const pe::Vector3& b) { return dDOTpq(a, b, 4, 1); }
        static pe::Real dDOT(const pe::Vector3& a, const pe::Real* b) { return dDOTpq(a, b, 1, 1); }
        static pe::Real dDOT44(const pe::Vector3& a, const pe::Real* b) { return dDOTpq(a, b, 4, 4); }
        static pe::Real dDOT14(const pe::Vector3& a, const pe::Real* b) { return dDOTpq(a, b, 1, 4); }
        static pe::Real dDOT41(const pe::Vector3& a, const pe::Real* b) { return dDOTpq(a, b, 4, 1); }
        static pe::Real dDOT(const pe::Real* a, const pe::Real* b) { return dDOTpq(a, b, 1, 1); }
        static pe::Real dDOT44(const pe::Real* a, const pe::Real* b) { return dDOTpq(a, b, 4, 4); }
        static pe::Real dDOT14(const pe::Real* a, const pe::Real* b) { return dDOTpq(a, b, 1, 4); }
        static pe::Real dDOT41(const pe::Real* a, const pe::Real* b) { return dDOTpq(a, b, 4, 1); }
        static pe::Real dDOT(const pe::Vector3& a, const pe::Vector3& b) { return dDOTpq(a, b, 1, 1); }
        static pe::Real dDOT44(const pe::Vector3& a, const pe::Vector3& b) { return dDOTpq(a, b, 4, 4); }
        static pe::Real dDOT14(const pe::Vector3& a, const pe::Vector3& b) { return dDOTpq(a, b, 1, 4); }
        static pe::Real dDOT41(const pe::Vector3& a, const pe::Vector3& b) { return dDOTpq(a, b, 4, 1); }
    };

} // pe_phys_collision