#include "box_box_collision_algorithm.h"
#include "phys/shape/box_shape.h"
#include <string.h>

namespace pe_phys_collision {

#   define dDOTpq(a, b, p, q) ((a)[0] * (b)[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)])
#   define dMULTIPLY0_331(A, B, C) {   \
		(A)[0] = dDOT((B), (C));     \
		(A)[1] = dDOT(((B) + 4), (C)); \
		(A)[2] = dDOT(((B) + 8), (C)); \
	}
#   define dMULTIPLY1_331(A, B, C) {   \
		(A)[0] = dDOT41((B), (C));     \
		(A)[1] = dDOT41(((B) + 1), (C)); \
		(A)[2] = dDOT41(((B) + 2), (C)); \
	}

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

    // box-box collision (bullet)
    bool BoxBoxCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                    pe::Transform trans_a, pe::Transform trans_b,
                                                    ContactResult& result) {
        if (shape_a->getType() != pe_phys_shape::ShapeType::Box ||
            shape_b->getType() != pe_phys_shape::ShapeType::Box) {
            return false;
        }
        getClosestPoint((pe_phys_shape::BoxShape*)shape_a, (pe_phys_shape::BoxShape*)shape_b, trans_a, trans_b, result);
        return true;
    }

    void BoxBoxCollisionAlgorithm::getClosestPoint(pe_phys_shape::BoxShape* shape_a, pe_phys_shape::BoxShape* shape_b,
                                                   pe::Transform& trans_a, pe::Transform& trans_b,
                                                   ContactResult& result) {
        dMatrix3 R1, R2;
        pe::Vector3 normal;
        pe::Real depth;
        int return_code;
        int max_c = 4;
        pe::Real margin = PE_MARGIN;

        auto& basis_a = trans_a.getBasis();
        auto& basis_b = trans_b.getBasis();
        for (int j = 0; j < 3; j++) {
            R1[0 + 4 * j] = basis_a[j][0];
            R2[0 + 4 * j] = basis_b[j][0];
            R1[1 + 4 * j] = basis_a[j][1];
            R2[1 + 4 * j] = basis_b[j][1];
            R1[2 + 4 * j] = basis_a[j][2];
            R2[2 + 4 * j] = basis_b[j][2];
        }

        dBoxBox2(trans_a.getOrigin(), R1, shape_a->getSize(),
                 trans_b.getOrigin(), R2, shape_b->getSize(),
                 normal, margin, depth, return_code, max_c, result);
    }

    void BoxBoxCollisionAlgorithm::dLineClosestApproach(const pe::Vector3& pa, const pe::Vector3& ua,
                                                        const pe::Vector3& pb, const pe::Vector3& ub,
                                                        pe::Real& alpha, pe::Real& beta) {
        pe::Vector3 p = pb - pa;
        pe::Real ua_ub = dDOT(ua, ub);
        pe::Real q1 = dDOT(ua, p);
        pe::Real q2 = -dDOT(ub, p);
        pe::Real d = 1 - ua_ub * ua_ub;

        if (d <= PE_EPS) {
            alpha = 0;
            beta = 0;
        } else {
            d = pe::Real(1.0) / d;
            alpha = (q1 + ua_ub * q2) * d;
            beta = (ua_ub * q1 + q2) * d;
        }
    }

    int BoxBoxCollisionAlgorithm::intersectRectQuad2(const pe::Real h[2], pe::Real p[8], pe::Real ret[16]) {
        // q (and r) contain nq (and nr) coordinate points for the current (and
        // chopped) polygons
        pe::Real buffer[16];
        int nq = 4, nr = 0;
        pe::Real* q = p;
        pe::Real* r = ret;

        for (int dir = 0; dir <= 1; dir++) {
            // direction notation: xy[0] = x axis, xy[1] = y axis
            for (int sign = -1; sign <= 1; sign += 2) {
                // chop q along the line xy[dir] = sign*h[dir]
                pe::Real* pq = q;
                pe::Real* pr = r;
                nr = 0;
                for (int i = nq; i > 0; i--) {
                    // go through all points in q and all lines between adjacent points
                    if (sign * pq[dir] < h[dir]) {
                        // this point is inside the chopping line
                        pr[0] = pq[0];
                        pr[1] = pq[1];
                        pr += 2;
                        nr++;
                        if (nr & 8) {
                            q = r;
                            goto done;
                        }
                    }
                    pe::Real* next_q = (i > 1) ? pq + 2 : q;
                    if ((sign * pq[dir] < h[dir]) ^ (sign * next_q[dir] < h[dir])) {
                        // this line crosses the chopping line
                        pr[1 - dir] = pq[1 - dir] + (next_q[1 - dir] - pq[1 - dir]) /
                                (next_q[dir] - pq[dir]) * (sign * h[dir] - pq[dir]);
                        pr[dir] = sign * h[dir];
                        pr += 2;
                        nr++;
                        if (nr & 8) {
                            q = r;
                            goto done;
                        }
                    }
                    pq += 2;
                }
                q = r;
                r = (q == ret) ? buffer : ret;
                nq = nr;
            }
        }
        done:
        if (q != ret) memcpy(ret, q, nr * 2 * sizeof(pe::Real));
        return nr;
    }

    void BoxBoxCollisionAlgorithm::cullPoints2(int n, pe::Real p[], int m, int i0, int i_ret[]) {
        // compute the centroid of the polygon in cx,cy
        int i, j;
        pe::Real a, cx, cy, q;
        if (n == 1) {
            cx = p[0];
            cy = p[1];
        } else if (n == 2) {
            cx = pe::Real(0.5) * (p[0] + p[2]);
            cy = pe::Real(0.5) * (p[1] + p[3]);
        } else {
            a = 0;
            cx = 0;
            cy = 0;
            for (i = 0; i < (n - 1); i++) {
                pe::Real p0 = p[i * 2], p1 = p[i * 2 + 1], p2 = p[i * 2 + 2], p3 = p[i * 2 + 3];
                q = p0 * p3 - p2 * p1;
                a += q;
                cx += q * (p0 + p2);
                cy += q * (p1 + p3);
            }
            pe::Real p0 = p[0], p1 = p[1], pm1 = p[n * 2 - 1], pm2 = p[n * 2 - 2];
            q = pm2 * p1 - p0 * pm1;
            if (PE_ABS(a + q) > PE_EPS) {
                a = pe::Real(1.0) / (pe::Real(3.0) * (a + q));
            } else {
                a = PE_REAL_MAX;
            }
            cx = a * (cx + q * (pm2 + p0));
            cy = a * (cy + q * (pm1 + p1));
        }

        // compute the angle of each point w.r.t. the centroid
        // search for points that have angles closest to A[i0] + i*(2*pi/m).
        pe::Real A[8]; int avail[8];
        for (i = 0; i < n; i++) {
            A[i] = std::atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);
            avail[i] = 1;
        }
        avail[i0] = 0;
        i_ret[0] = i0;
        i_ret++;
        for (j = 1; j < m; j++) {
            a = pe::Real(j) * (2 * PE_PI / m) + A[i0];
            if (a > PE_PI) a -= 2 * PE_PI;
            pe::Real max_diff = 1e9, diff;

            *i_ret = i0;  // i_ret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

            for (i = 0; i < n; i++) {
                if (avail[i]) {
                    diff = PE_ABS(A[i] - a);
                    if (diff > PE_PI) diff = 2 * PE_PI - diff;
                    if (diff < max_diff) {
                        max_diff = diff;
                        *i_ret = i;
                    }
                }
            }
            avail[*i_ret] = 0;
            i_ret++;
        }
    }

    int BoxBoxCollisionAlgorithm::dBoxBox2(const pe::Vector3& p1, const dMatrix3 R1, const pe::Vector3& side1,
                                           const pe::Vector3& p2, const dMatrix3 R2, const pe::Vector3& side2,
                                           pe::Vector3& normal, pe::Real margin, pe::Real& depth, int& return_code,
                                           int max_c, ContactResult& result) {
        const pe::Real fudge_factor = pe::Real(1.05);
        pe::Vector3 p, pp, normalC{0,0,0};
        const pe::Real* normalR = 0;
        pe::Real A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33,
                Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
        int i, j, invert_normal, code;

        // get vector from centers of box 1 to box 2, relative to box 1
        p = p2 - p1;
        dMULTIPLY1_331(pp, R1, p)  // get pp = p relative to body 1

        // get side lengths / 2
        A[0] = side1[0] * pe::Real(0.5);
        A[1] = side1[1] * pe::Real(0.5);
        A[2] = side1[2] * pe::Real(0.5);
        B[0] = side2[0] * pe::Real(0.5);
        B[1] = side2[1] * pe::Real(0.5);
        B[2] = side2[2] * pe::Real(0.5);

        // Rij is R1'*R2, i.e. the relative rotation between R1 and R2
        R11 = dDOT44(R1 + 0, R2 + 0);
        R12 = dDOT44(R1 + 0, R2 + 1);
        R13 = dDOT44(R1 + 0, R2 + 2);
        R21 = dDOT44(R1 + 1, R2 + 0);
        R22 = dDOT44(R1 + 1, R2 + 1);
        R23 = dDOT44(R1 + 1, R2 + 2);
        R31 = dDOT44(R1 + 2, R2 + 0);
        R32 = dDOT44(R1 + 2, R2 + 1);
        R33 = dDOT44(R1 + 2, R2 + 2);

        Q11 = PE_ABS(R11);
        Q12 = PE_ABS(R12);
        Q13 = PE_ABS(R13);
        Q21 = PE_ABS(R21);
        Q22 = PE_ABS(R22);
        Q23 = PE_ABS(R23);
        Q31 = PE_ABS(R31);
        Q32 = PE_ABS(R32);
        Q33 = PE_ABS(R33);

        // for all 15 possible separating axes:
        //   * see if the axis separates the boxes. if so, return 0.
        //   * find the depth of the penetration along the separating axis (s2)
        //   * if this is the largest depth so far, record it.
        // the normal vector will be set to the separating axis with the smallest
        // depth. note: normalR is set to point to a column of R1 or R2 if that is
        // the smallest depth normal so far. otherwise normalR is 0 and normalC is
        // set to a vector relative to body 1. invert_normal is 1 if the sign of
        // the normal should be flipped.

#   define TST(expr1, expr2, norm, cc)      \
        s2 = PE_ABS(expr1) - (expr2);     \
        if (s2 > 0) return 0;               \
        if (s2 > s) {                       \
            s = s2;                         \
            normalR = norm;                 \
            invert_normal = ((expr1) < 0);  \
            code = (cc);                    \
        }

        s = PE_REAL_MIN;
        invert_normal = 0;
        code = 0;

        // separating axis = u1,u2,u3
        TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1 + 0, 1)
        TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1 + 1, 2)
        TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1 + 2, 3)

        // separating axis = v1,v2,v3
        TST(dDOT41(R2 + 0, p), (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2 + 0, 4)
        TST(dDOT41(R2 + 1, p), (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2 + 1, 5)
        TST(dDOT41(R2 + 2, p), (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2 + 2, 6)

        // note: cross product axes need to be scaled when s is computed.
        // normal (n1,n2,n3) is relative to box 1.
#   undef TST
#   define TST(expr1, expr2, n1, n2, n3, cc)                    \
        s2 = PE_ABS(expr1) - (expr2);                         \
        if (s2 > PE_EPS) return 0;                              \
        l = std::sqrt((n1) * (n1) + (n2) * (n2) + (n3) * (n3)); \
        if (l > PE_EPS) {                                       \
            s2 /= l;                                            \
            if (s2 * fudge_factor > s) {                        \
                s = s2;                                         \
                normalR = 0;                                    \
                normalC[0] = (n1) / l;                          \
                normalC[1] = (n2) / l;                          \
                normalC[2] = (n3) / l;                          \
                invert_normal = ((expr1) < 0);                  \
                code = (cc);                                    \
            }                                                   \
        }

#   define fudge2 1.0e-5f

        Q11 += fudge2;
        Q12 += fudge2;
        Q13 += fudge2;

        Q21 += fudge2;
        Q22 += fudge2;
        Q23 += fudge2;

        Q31 += fudge2;
        Q32 += fudge2;
        Q33 += fudge2;

#   undef fudge2

        // separating axis = u1 x (v1,v2,v3)
        TST(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), 0, -R31, R21, 7)
        TST(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), 0, -R32, R22, 8)
        TST(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), 0, -R33, R23, 9)

        // separating axis = u2 x (v1,v2,v3)
        TST(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31, 0, -R11, 10)
        TST(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32, 0, -R12, 11)
        TST(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33, 0, -R13, 12)

        // separating axis = u3 x (v1,v2,v3)
        TST(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 0, 13)
        TST(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 0, 14)
        TST(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 0, 15)

#   undef TST

        if (!code) return 0;

        // if we get to this point, the boxes interpenetrate. compute the normal
        // in global coordinates.
        if (normalR) {
            normal[0] = normalR[0];
            normal[1] = normalR[4];
            normal[2] = normalR[8];
        } else {
            dMULTIPLY0_331(normal, R1, normalC)
        }
        if (invert_normal) {
            normal[0] = -normal[0];
            normal[1] = -normal[1];
            normal[2] = -normal[2];
        }
        depth = -s;

        // compute contact point(s)
        if (code > 6) {
            // an edge from box 1 touches an edge from box 2.
            // find a point pa on the intersecting edge of box 1
            pe::Vector3 pa;
            pe::Real sign;
            pa[0] = p1[0];
            pa[1] = p1[1];
            pa[2] = p1[2];

            for (j = 0; j < 3; j++) {
                sign = (dDOT14(normal, R1 + j) > 0) ? pe::Real(1.0) : pe::Real(-1.0);
                pa[0] += sign * A[j] * R1[0 * 4 + j];
                pa[1] += sign * A[j] * R1[1 * 4 + j];
                pa[2] += sign * A[j] * R1[2 * 4 + j];
            }

            // find a point pb on the intersecting edge of box 2
            pe::Vector3 pb;
            for (i = 0; i < 3; i++) pb[i] = p2[i];
            for (j = 0; j < 3; j++) {
                sign = (dDOT14(normal, R2 + j) > 0) ? pe::Real(-1.0) : pe::Real(1.0);
                pb[0] += sign * B[j] * R2[0 * 4 + j];
                pb[1] += sign * B[j] * R2[1 * 4 + j];
                pb[2] += sign * B[j] * R2[2 * 4 + j];
            }

            pe::Real alpha, beta;
            pe::Vector3 ua, ub;
            {
                int quotient = ((code)-7) / 3, remainder = ((code)-7) % 3;
                ua[0] = R1[quotient + 0 * 4];
                ua[1] = R1[quotient + 1 * 4];
                ua[2] = R1[quotient + 2 * 4];

                ub[0] = R2[remainder + 0 * 4];
                ub[1] = R2[remainder + 1 * 4];
                ub[2] = R2[remainder + 2 * 4];
            }

            dLineClosestApproach(pa, ua, pb, ub, alpha, beta);
            pa[0] += ua[0] * alpha;
            pa[1] += ua[1] * alpha;
            pa[2] += ua[2] * alpha;

            pb[0] += ub[0] * beta;
            pb[1] += ub[1] * beta;
            pb[2] += ub[2] * beta;

            {
                const pe::Vector3 normVec(normal[0], normal[1], normal[2]);
                result.addContactPoint(-normVec, pe::Vector3(pb[0], pb[1], pb[2]) +
                                        normVec * margin, -depth + margin * 2);
                return_code = code;
            }
            return 1;
        }

        // okay, we have a face-something intersection (because the separating
        // axis is perpendicular to a face). define face 'a' to be the reference
        // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
        // the incident face (the closest face of the other box).

        const pe::Real *Ra, *Rb, *pa, *pb, *Sa, *Sb;
        if (code <= 3) {
            result.setSwapFlag(false);
            Ra = R1;
            Rb = R2;
            pa = &p1[0];
            pb = &p2[0];
            Sa = A;
            Sb = B;
        } else {
            result.setSwapFlag(true);
            Ra = R2;
            Rb = R1;
            pa = &p2[0];
            pb = &p1[0];
            Sa = B;
            Sb = A;
        }

        // nr = normal vector of reference face dotted with axes of incident box.
        // anr = absolute values of nr.
        pe::Vector3 normal2, nr, anr;
        if (code <= 3) {
            normal2[0] = normal[0];
            normal2[1] = normal[1];
            normal2[2] = normal[2];
        } else {
            normal2[0] = -normal[0];
            normal2[1] = -normal[1];
            normal2[2] = -normal[2];
        }
        dMULTIPLY1_331(nr, Rb, normal2)
        anr[0] = PE_ABS(nr[0]);
        anr[1] = PE_ABS(nr[1]);
        anr[2] = PE_ABS(nr[2]);

        // find the largest component of anr: this corresponds to the normal
        // for the incident face. the other axis numbers of the indicent face
        // are stored in a1,a2.
        int lan_r, a1, a2;
        if (anr[1] > anr[0]) {
            if (anr[1] > anr[2]) {
                a1 = 0;
                lan_r = 1;
                a2 = 2;
            } else {
                a1 = 0;
                a2 = 1;
                lan_r = 2;
            }
        } else {
            if (anr[0] > anr[2]) {
                lan_r = 0;
                a1 = 1;
                a2 = 2;
            } else {
                a1 = 0;
                a2 = 1;
                lan_r = 2;
            }
        }

        // compute center point of incident face, in reference-face coordinates
        pe::Vector3 center;
        if (nr[lan_r] < 0) {
            const pe::Real sb_lan_r = Sb[lan_r];
            center[0] = pb[0] - pa[0] + sb_lan_r * Rb[0 * 4 + lan_r];
            center[1] = pb[1] - pa[1] + sb_lan_r * Rb[1 * 4 + lan_r];
            center[2] = pb[2] - pa[2] + sb_lan_r * Rb[2 * 4 + lan_r];
        } else {
            const pe::Real sb_lan_r = Sb[lan_r];
            center[0] = pb[0] - pa[0] - sb_lan_r * Rb[0 * 4 + lan_r];
            center[1] = pb[1] - pa[1] - sb_lan_r * Rb[1 * 4 + lan_r];
            center[2] = pb[2] - pa[2] - sb_lan_r * Rb[2 * 4 + lan_r];
        }

        // find the normal and non-normal axis numbers of the reference box
        int codeN, code1, code2;
        if (code <= 3)
            codeN = code - 1;
        else
            codeN = code - 4;
        if (codeN == 0) {
            code1 = 1;
            code2 = 2;
        } else if (codeN == 1) {
            code1 = 0;
            code2 = 2;
        } else {
            code1 = 0;
            code2 = 1;
        }

        // find the four corners of the incident face, in reference-face coordinates
        pe::Real quad[8];  // 2D coordinate of incident face (x,y pairs)
        pe::Real c1, c2, m11, m12, m21, m22;
        c1 = dDOT14(center, Ra + code1);
        c2 = dDOT14(center, Ra + code2);
        // optimize this? - we have already computed this data above, but it is not
        // stored in an easy-to-index format. for now it's quicker just to recompute
        // the four dot products.
        m11 = dDOT44(Ra + code1, Rb + a1);
        m12 = dDOT44(Ra + code1, Rb + a2);
        m21 = dDOT44(Ra + code2, Rb + a1);
        m22 = dDOT44(Ra + code2, Rb + a2);
        {
            const pe::Real k1 = m11 * Sb[a1];
            const pe::Real k2 = m21 * Sb[a1];
            const pe::Real k3 = m12 * Sb[a2];
            const pe::Real k4 = m22 * Sb[a2];
            const pe::Real c1mk1 = c1 - k1;
            const pe::Real c2mk2 = c2 - k2;
            quad[0] = c1mk1 - k3;
            quad[1] = c2mk2 - k4;
            quad[2] = c1mk1 + k3;
            quad[3] = c2mk2 + k4;
            const pe::Real c1pk1 = c1 + k1;
            const pe::Real c2pk2 = c2 + k2;
            quad[4] = c1pk1 + k3;
            quad[5] = c2pk2 + k4;
            quad[6] = c1pk1 - k3;
            quad[7] = c2pk2 - k4;
        }

        // find the size of the reference face
        pe::Real rect[2]{Sa[code1], Sa[code2]};

        // intersect the incident and reference faces
        pe::Real ret[16];
        int n = intersectRectQuad2(rect, quad, ret);
        if (n < 1) return 0;  // this should never happen

        // convert the intersection points into reference-face coordinates,
        // and compute the contact position and depth for each point. only keep
        // those points that have a positive (penetrating) depth. delete points in
        // the 'ret' array as necessary so that 'point' and 'ret' correspond.
        pe::Real point[3 * 8];  // penetrating contact points
        pe::Real dep[8];        // depths for those points
        pe::Real det1 = 1.f / (m11 * m22 - m12 * m21);
        m11 *= det1;
        m12 *= det1;
        m21 *= det1;
        m22 *= det1;
        int cnum = 0;  // number of penetrating contact points found
        for (j = 0; j < n; j++) {
            pe::Real k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
            pe::Real k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
            point[cnum * 3 + 0] = center[0] + k1 * Rb[0 * 4 + a1] + k2 * Rb[0 * 4 + a2];
            point[cnum * 3 + 1] = center[1] + k1 * Rb[1 * 4 + a1] + k2 * Rb[1 * 4 + a2];
            point[cnum * 3 + 2] = center[2] + k1 * Rb[2 * 4 + a1] + k2 * Rb[2 * 4 + a2];

            dep[cnum] = Sa[codeN] - dDOT(normal2, point + cnum * 3);
            if (dep[cnum] >= 0) {
                ret[cnum * 2] = ret[j * 2];
                ret[cnum * 2 + 1] = ret[j * 2 + 1];
                cnum++;
            }
        }
        if (cnum < 1) return 0;  // this should never happen

        // we can't generate more contacts than we actually have
        max_c = PE_MIN(max_c, cnum);
        max_c = PE_MAX(max_c, 1);

        const pe::Vector3 normVec(normal[0], normal[1], normal[2]);
        const pe::Vector3 marginVec = normVec * margin;

        if (cnum <= max_c) {
            if (code < 4) {
                // we have less contacts than we need, so we use them all
                for (j = 0; j < cnum; j++) {
                    const pe::Vector3 pointInWorld(
                            point[j * 3 + 0] + pa[0],
                            point[j * 3 + 1] + pa[1],
                            point[j * 3 + 2] + pa[2]);
                    result.addContactPoint(-normVec, pointInWorld + marginVec,
                                           -dep[j] + margin * 2);
                }
            } else {
                // we have less contacts than we need, so we use them all
                for (j = 0; j < cnum; j++) {
                    const pe::Vector3 pointInWorld(
                            point[j * 3 + 0] + pa[0],
                            point[j * 3 + 1] + pa[1],
                            point[j * 3 + 2] + pa[2]);
                    result.addContactPoint(normVec, pointInWorld - marginVec,
                                           -dep[j] + margin * 2);//// add inverse normal
                }
            }
        } else {
            // we have more contacts than are wanted, some of them must be culled.
            // find the deepest point, it is always the first contact.
            int i1 = 0;
            pe::Real max_depth = dep[0];
            for (i = 1; i < cnum; i++) {
                if (dep[i] > max_depth) {
                    max_depth = dep[i];
                    i1 = i;
                }
            }

            int i_ret[8];
            cullPoints2(cnum, ret, max_c, i1, i_ret);

            for (j = 0; j < max_c; j++) {
                const pe::Vector3 posInWorld(
                        point[i_ret[j] * 3 + 0] + pa[0],
                        point[i_ret[j] * 3 + 1] + pa[1],
                        point[i_ret[j] * 3 + 2] + pa[2]
                );

                if (code < 4) {
                    result.addContactPoint(-normVec, posInWorld + marginVec,
                                           -dep[i_ret[j]] + margin * 2);
                } else {
                    result.addContactPoint(normVec, posInWorld - marginVec,
                                           -dep[i_ret[j]] + margin * 2);
                }
            }
            cnum = max_c;
        }

        return_code = code;
        return cnum;
    }

} // pe_phys_collision