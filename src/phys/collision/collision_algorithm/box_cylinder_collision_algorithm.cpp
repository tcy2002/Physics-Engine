#include "box_cylinder_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/default_mesh.h"
#include "phys/shape/convex_mesh_shape.h"
#include "convex_convex_collision_algorithm.h"

// box-cylinder collision (from damps)
namespace pe_phys_collision {

    static void addContactPoint(const pe::Vector3& pc, int iface, const pe::Vector3& hdims,
                                const pe::Transform& X_box, const pe::Real margin, ContactResult& result)
    {
        if (!(iface >= -3 && iface <= +3 && iface != 0))
            return;

        // No contact if point outside box
        if (!BoxCylinderCollisionAlgorithm::PointInsideBox(hdims, pc))
            return; // no contacts added

        // Find point projection on box face and calculate normal and penetration
        // (still working in the box frame)
        pe::Vector3 p = pc;
        pe::Vector3 n(0, 0, 0);
        pe::Real penetration;
        if (iface > 0) {
            // "positive" box face
            int i = iface - 1;
            p[i] = hdims[i];
            n[i] = 1;
            penetration = pc[i] - hdims[i];
        } else {
            // "negative" box face
            int i = -iface - 1;
            p[i] = -hdims[i];
            n[i] = -1;
            penetration = -pc[i] - hdims[i];
        }

        // A new contact point must specify (in absolute frame):
        //   normal, pointing from B towards A
        //   point, located on surface of B
        //   distance, negative for penetration
        pe::Vector3 normal = X_box.getBasis() * n;
        pe::Vector3 point = X_box * p;
        result.addContactPoint(normal, point - normal * margin, penetration + 2 * margin);
    }

    static void addContactPoint(const pe::Vector3& p, const pe::Vector3& c, const pe::Vector3& a, const pe::Real h,
                                const pe::Real r, const pe::Transform& X_box, const pe::Real margin, ContactResult& result)
    {
        // Find closest point on cylindrical surface to given location
        pe::Vector3 q = c + (p - c).dot(a) * a;
        pe::Vector3 v = p - q;
        pe::Real dist = v.norm();
        pe::Vector3 n = v / dist;

        pe::Vector3 normal = X_box.getBasis() * (-n);
        pe::Vector3 point = X_box * p;
        result.addContactPoint(normal, point - normal * margin, dist - r + 2 * margin);
    }

    bool BoxCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                         pe::Transform trans_a, pe::Transform trans_b,
                                                         pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Box &&
               shape_b->getType() == pe_phys_shape::ShapeType::Cylinder) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::Box))) {
            return false;
        }
        pe::Real margin = PE_MARGIN;

#   if false
        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       ((pe_phys_shape::CylinderShape*)shape_a)->getMesh() :
                       ((pe_phys_shape::BoxShape*)shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       ((pe_phys_shape::CylinderShape*)shape_b)->getMesh() :
                       ((pe_phys_shape::BoxShape*)shape_b)->getMesh();
        auto& edges_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        ((pe_phys_shape::CylinderShape*)shape_a)->getUniqueEdges() :
                        ((pe_phys_shape::BoxShape*)shape_a)->getUniqueEdges();
        auto& edges_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        ((pe_phys_shape::CylinderShape*)shape_b)->getUniqueEdges() :
                        ((pe_phys_shape::BoxShape*)shape_b)->getUniqueEdges();

        pe::Vector3 sep;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(shape_a, shape_b,
                                                                mesh_a, mesh_b,
                                                                edges_a, edges_b,
                                                                trans_a, trans_b, sep, margin, result)) {
            return false;
        }
        ConvexConvexCollisionAlgorithm::clipHullAgainstHull(sep,
                                                            mesh_a, mesh_b, trans_a, trans_b,
                                                            -refScale, margin,
                                                            world_verts_b1, world_verts_b2,
                                                            margin, result);
        return true;
#   else
        auto shape_box = shape_a->getType() == pe_phys_shape::ShapeType::Box ?
                         (pe_phys_shape::BoxShape*)shape_a : (pe_phys_shape::BoxShape*)shape_b;
        auto shape_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                         (pe_phys_shape::CylinderShape*)shape_a : (pe_phys_shape::CylinderShape*)shape_b;
        auto& trans_box = shape_a->getType() == pe_phys_shape::ShapeType::Box ? trans_a : trans_b;
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? trans_a : trans_b;

        const pe::Real cyl_r = shape_cyl->getRadius();
        const pe::Real cyl_h = shape_cyl->getHeight() / pe::Real(2.0);
        pe::Vector3 hdims = shape_box->getSize() / pe::Real(2.0);
        pe::Transform trans_cyl_r = trans_box.inverse() * trans_cyl;
        pe::Vector3 a = trans_cyl_r.getBasis().getColumn(1);
        pe::Vector3 c = trans_cyl_r.getOrigin();

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::Box);

        // - Loop over each direction of the box frame (i.e., each of the 3 face normals).
        // - For each direction, consider two segments on the cylindrical surface that are on a plane defined by the
        //   axis and the face normal. (Note that, in principle, we could only consider the segment "closest" to the box, but
        //   that is not trivial to define in all configurations). All segments are parameterized by t in [-H,H].
        // - For each segment, if the segment intersects the box, consider 3 candidate contact points: the 2
        //   intersection points and their midpoint. A contact is added if the segment point is inside the box.
        //   Furthermore, the corresponding box point is located on the box face that is closest to the intersection
        //   midpoint candidate.
        for (int idir = 0; idir < 3; idir++) {
            // current box direction
            pe::Vector3 ndir(0, 0, 0);
            ndir[idir] = 1;

            // If the axis is parallel to the current direction, no contact.
            if (PE_ABS(a[idir] - 1) < PE_EPS || PE_ABS(a[idir] + 1) < PE_EPS)
                continue;

            // Direction perpendicular to cylinder axis (in direction opposite to ndir)
            pe::Vector3 v = ndir.cross(a);
            pe::Vector3 r = v.cross(a);
            // assert(r.length() > PE_EPS);
            r.normalize();

            // Consider segments in both "negative" and "positive" r direction
            pe::Real dir[2] = {-1, 1};
            for (int jdir = 0; jdir < 2; jdir++) {
                // Calculate current segment center
                pe::Vector3 cs = c + dir[jdir] * cyl_r * r;
                // Check for intersection with box
                pe::Real tMin, tMax;
                if (IntersectSegmentBox(hdims, cs, a, cyl_h, PE_EPS, tMin, tMax)) {
                    // Consider the intersection points and their midpoint as candidates
                    pe::Vector3 pMin = cs + a * tMin;
                    pe::Vector3 pMax = cs + a * tMax;
                    pe::Vector3 pMid = cs + a * ((tMin + tMax) / 2);

                    // Pick box face that is closest to midpoint
                    int iface = FindClosestBoxFace(hdims, pMid);

                    // Add a contact for any of the candidate points that is inside the box
                    addContactPoint(pMin, iface, hdims, trans_box, margin, result); // 1st segment end
                    addContactPoint(pMax, iface, hdims, trans_box, margin, result); // 2nd segment end
                    addContactPoint(pMid, iface, hdims, trans_box, margin, result); // intersection midpoint
                }
            }
        }

        /// @bug bugs when cylinder roll down the box
        // ---------------------------------------------------------------------
        // If a box face supports the cylinder, do not check box edges.
        // if (num_contacts > 0) {
        //     return resultOut.getPointSize();
        // }
        // -------------------------------------------------------------------

        // - Loop over each direction of the box frame.
        // - For each direction, check intersection with the cylinder for all 4 edges parallel to that direction.
        // - If an edge intersects the cylinder, consider 3 candidate contact points: the 2 intersection points
        //   and their midpoint.
        for (int idir = 0; idir < 3; idir++) {
            // current box edge direction and halflength
            pe::Vector3 eD(0, 0, 0);
            eD[idir] = 1;
            pe::Real eH = hdims[idir];
            // The other two box directions
            int jdir = (idir + 1) % 3;
            int kdir = (idir + 2) % 3;
            for (int j = -1; j <= +1; j += 2) {
                for (int k = -1; k <= +1; k += 2) {
                    pe::Vector3 eC;
                    eC[idir] = 0;
                    eC[jdir] = j * hdims[jdir];
                    eC[kdir] = k * hdims[kdir];
                    // Check for edge intersection with cylinder
                    pe::Real tMin, tMax;
                    if (IntersectSegmentCylinder(eC, eD, eH, c, a, cyl_h, cyl_r, PE_EPS, tMin, tMax)) {
                        // Consider the intersection points and their midpoint as candidates
                        pe::Vector3 pMin = eC + eD * tMin;
                        pe::Vector3 pMax = eC + eD * tMax;
                        pe::Vector3 pMid = eC + eD * ((tMin + tMax) / 2);

                        // Add a contact for any of the candidate points that is inside the cylinder
                        addContactPoint(pMin, c, a, cyl_h, cyl_r, trans_box, margin, result);
                        addContactPoint(pMax, c, a, cyl_h, cyl_r, trans_box, margin, result);
                        addContactPoint(pMid, c, a, cyl_h, cyl_r, trans_box, margin, result);
                    }
                }
            }
        }

        result.setSwapFlag(false);
        return true;
#   endif
    }

    bool BoxCylinderCollisionAlgorithm::PointInsideBox(const pe::Vector3 &hdims, const pe::Vector3 &loc) {
        for (int i = 0; i < 3; i++) {
            if (loc[i] > hdims[i] || loc[i] < -hdims[i])
                return false;
        }
        return true;
    }

    int BoxCylinderCollisionAlgorithm::FindClosestBoxFace(const pe::Vector3 &hdims, const pe::Vector3 &loc) {
        int code = 0;
        pe::Real dist = PE_REAL_MAX;
        for (int i = 0; i < 3; i++) {
            pe::Real p_dist = PE_ABS(loc[i] - hdims[i]);
            pe::Real n_dist = PE_ABS(loc[i] + hdims[i]);
            if (p_dist < dist) {
                code = i + 1;
                dist = p_dist;
            }
            if (n_dist < dist) {
                code = -(i + 1);
                dist = n_dist;
            }
        }
        return code;
    }

    bool BoxCylinderCollisionAlgorithm::IntersectLinePlane(const pe::Vector3 &lP, const pe::Vector3 &lD, const pe::Vector3 &pP, const pe::Vector3 &pN, const pe::Real tol, pe::Real &t) {
        pe::Real nd = pN.dot(lD);

        if (PE_ABS(nd) < tol) {
            // Line parallel to plane
            return false;
        }

        t = pN.dot(pP - lP) / nd;
        return true;
    }

    bool BoxCylinderCollisionAlgorithm::IntersectSegmentBox(const pe::Vector3 &hdims, const pe::Vector3 &c, const pe::Vector3 &a, const pe::Real hlen, const pe::Real tol, pe::Real &tMin, pe::Real &tMax) {
        tMin = PE_REAL_MIN;
        tMax = PE_REAL_MAX;

        if (PE_ABS(a.x) < tol) {
            // Segment parallel to the box x-faces
            if (PE_ABS(c.x) > hdims.x)
                return false;
        } else {
            pe::Real t1 = (-hdims.x - c.x) / a.x;
            pe::Real t2 = (+hdims.x - c.x) / a.x;

            tMin = PE_MAX(tMin, PE_MIN(t1, t2));
            tMax = PE_MIN(tMax, PE_MAX(t1, t2));

            if (tMin > tMax)
                return false;
        }

        if (PE_ABS(a.y) < tol) {
            // Segment parallel to the box y-faces
            if (PE_ABS(c.y) > hdims.y)
                return false;
        } else {
            pe::Real t1 = (-hdims.y - c.y) / a.y;
            pe::Real t2 = (+hdims.y - c.y) / a.y;

            tMin = PE_MAX(tMin, PE_MIN(t1, t2));
            tMax = PE_MIN(tMax, PE_MAX(t1, t2));

            if (tMin > tMax)
                return false;
        }

        if (PE_ABS(a.z) < tol) {
            // Capsule axis parallel to the box z-faces
            if (PE_ABS(c.z) > hdims.z)
                return false;
        } else {
            pe::Real t1 = (-hdims.z - c.z) / a.z;
            pe::Real t2 = (+hdims.z - c.z) / a.z;

            tMin = PE_MAX(tMin, PE_MIN(t1, t2));
            tMax = PE_MIN(tMax, PE_MAX(t1, t2));

            if (tMin > tMax)
                return false;
        }

        // If both intersection points are outside the segment, no intersection
        if ((tMin < -hlen && tMax < -hlen) || (tMin > +hlen && tMax > +hlen))
            return false;

        // Clamp intersection points to segment length
        tMin = PE_CLAMP(tMin, -hlen, +hlen);
        tMax = PE_CLAMP(tMax, -hlen, +hlen);

        return true;
    }

    bool BoxCylinderCollisionAlgorithm::IntersectSegmentCylinder(const pe::Vector3 &sC, const pe::Vector3 &sD, const pe::Real sH, const pe::Vector3 &cC, const pe::Vector3 &cD, const pe::Real cH, const pe::Real cR, const pe::Real tol, pe::Real &tMin, pe::Real &tMax) {
        tMin = PE_REAL_MIN;
        tMax = PE_REAL_MAX;

        pe::Vector3 v = sC - cC;
        pe::Real cDsD = cD.dot(sD);
        pe::Real vcD = v.dot(cD);
        pe::Real vsD = v.dot(sD);
        pe::Real vv = v.dot(v);
        pe::Real a = 1 - cDsD * cDsD;
        pe::Real b = vsD - vcD * cDsD;
        pe::Real c = vv - vcD * vcD - cR * cR;

        // Intersection with cylindrical surface.
        // a >= 0 always
        // a == 0 indicates line parallel to cylinder axis
        if (PE_ABS(a) < tol) {
            // line parallel to cylinder axis
            pe::Real dist2 = (v - vcD * cD).norm2();
            if (dist2 > cR * cR)
                return false;
            tMin = -sH;
            tMax = +sH;
        } else {
            // line intersects cylindrical surface
            pe::Real discr = b * b - a * c;
            if (discr < 0)
                return false; // no real roots, no intersection
            discr = std::sqrt(discr);
            tMin = (-b - discr) / a;
            tMax = (-b + discr) / a;
        }

        // Intersection with end-caps.
        pe::Real t1;
        bool code1 = IntersectLinePlane(sC, sD, cC + cH * cD, cD, tol, t1);
        pe::Real t2;
        bool code2 = IntersectLinePlane(sC, sD, cC - cH * cD, cD, tol, t2);
        if (code1 && code2) {
            // line intersects end-caps
            if (t1 < t2) {
                tMin = PE_MAX(tMin, t1);
                tMax = PE_MIN(tMax, t2);
            } else {
                tMin = PE_MAX(tMin, t2);
                tMax = PE_MIN(tMax, t1);
            }
            if (tMax < tMin)
                return false;
        } else {
            // line parallel to end-cap planes
            pe::Real d1 = PE_ABS(cD.dot(cC + cH * cD - sC));
            pe::Real d2 = PE_ABS(cD.dot(cC - cH * cD - sC));
            if (d1 > 2 * cH || d2 > 2 * cH)
                return false;
        }

        // If both intersection points are outside the segment, no intersection
        if ((tMin < -sH && tMax < -sH) || (tMin > +sH && tMax > +sH))
            return false;

        // Clamp to segment length
        tMin = PE_CLAMP(tMin, -sH, +sH);
        tMax = PE_CLAMP(tMax, -sH, +sH);

        return true;
    }

} // pe_phys_collision