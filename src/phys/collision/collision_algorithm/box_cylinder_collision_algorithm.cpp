#include "box_cylinder_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/default_mesh.h"
#include "phys/shape/convex_mesh_shape.h"
#include "convex_convex_collision_algorithm.h"
#include "cylinder_convex_collision_algorithm.h"

// box-cylinder collision (from damps, still has bugs)
// style-checked.
namespace pe_phys_collision {

    bool BoxCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                         pe::Transform trans_a, pe::Transform trans_b,
                                                         pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Box &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Box))) {
            return false;
        }
        constexpr auto margin = PE_MARGIN;

        if (result.getObjectA()->getTag() == "wheel") {
#       if false
            const auto shape_box = shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? shape_a : shape_b;
            const auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape*>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? shape_a : shape_b);
            auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? trans_a : trans_b;
            auto& trans_box = shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? trans_a : trans_b;

            result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_Box);
            bool ret = CylinderConvexCollisionAlgorithm::getClosestPoints(shape_box, shape_cyl, trans_box, trans_cyl, margin, result);
            result.setSwapFlag(false);

            return ret;
#       else
            auto shape_box = dynamic_cast<pe_phys_shape::BoxShape*>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? shape_a : shape_b);
            auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape*>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? shape_a : shape_b);
            auto& trans_box = shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? trans_a : trans_b;
            auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? trans_a : trans_b;

            const pe::Real cyl_r = shape_cyl->getRadius();
            const pe::Real cyl_h = shape_cyl->getHeight() / PE_R(2.0);
            const pe::Vector3 box_half_extent = shape_box->getSize() / PE_R(2.0);

            result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_Box);
            bool ret = getClosestPoints(shape_box, shape_cyl, trans_box, trans_cyl, cyl_r, cyl_h, box_half_extent, margin, result);
            result.setSwapFlag(false);

            return ret;
#       endif
        }
        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ?
                       dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a)->getMesh() :
                       dynamic_cast<pe_phys_shape::BoxShape *>(shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder ?
                       dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b)->getMesh() :
                       dynamic_cast<pe_phys_shape::BoxShape *>(shape_b)->getMesh();
        auto& edges_a = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ?
                        dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::BoxShape *>(shape_a)->getUniqueEdges();
        auto& edges_b = shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder ?
                        dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::BoxShape *>(shape_b)->getUniqueEdges();

        return ConvexConvexCollisionAlgorithm::getClosestPoints(shape_a, shape_b, mesh_a, mesh_b, edges_a, edges_b,
                                                                trans_a, trans_b, margin, refScale, result);

    }

    static bool pointInsideBox(const pe::Vector3& half_extent, const pe::Vector3& loc) {
        for (int i = 0; i < 3; i++) {
            if (loc[i] > half_extent[i] || loc[i] < -half_extent[i])
                return false;
        }
        return true;
    }

    void BoxCylinderCollisionAlgorithm::addContactPointOnBox(const pe::Vector3& pc, const int i_face, const pe::Vector3& half_extent,
                                                             const pe::Transform& trans_other, pe::Real margin, ContactResult& result) {
        if (!(i_face >= -3 && i_face <= +3 && i_face != 0))
            return;

        // No contact if point outside box
        if (!pointInsideBox(half_extent, pc))
            return; // no contacts added

        // Find point projection on box face and calculate normal and penetration
        // (still working in the box frame)
        pe::Vector3 p = pc;
        pe::Vector3 n = pe::Vector3::Zero();
        pe::Real penetration;
        if (i_face > 0) {
            // "positive" box face
            const int i = i_face - 1;
            p[i] = half_extent[i];
            n[i] = 1;
            penetration = pc[i] - half_extent[i];
        } else {
            // "negative" box face
            const int i = -i_face - 1;
            p[i] = -half_extent[i];
            n[i] = -1;
            penetration = -pc[i] - half_extent[i];
        }

        // A new contact point must specify (in absolute frame):
        //   normal, pointing from B towards A
        //   point, located on surface of B
        //   distance, negative for penetration
        const pe::Vector3 normal = trans_other.getBasis() * n;
        const pe::Vector3 point = trans_other * p;
        result.addContactPoint(normal, point - normal * margin, penetration + 2 * margin);
    }

    void BoxCylinderCollisionAlgorithm::addContactPointOnCylinder(
            const pe::Vector3& p, const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl, pe::Real radius_cyl, pe::Real h_cyl,
            const pe::Transform& trans_other, pe::Real margin, ContactResult& result) {
        // Find the closest point on cylindrical surface to the given location
        const pe::Vector3 r = p - pos_cyl;
        const pe::Real dist_y = r.dot(axis_cyl);
        const pe::Real dist_xz = PE_SQRT(r.squaredNorm() - dist_y * dist_y);
        const pe::Real dist2side = radius_cyl - dist_xz;
        const pe::Real dist2top = h_cyl - PE_ABS(dist_y);

        pe::Vector3 normal;
        pe::Real dist;
        if (dist2side < dist2top) {
            dist = dist2side;
            normal = (axis_cyl * dist_y - r).normalized();
        } else {
            dist = dist2top;
            normal = dist_y < 0 ? axis_cyl : -axis_cyl;
        }

        normal = trans_other.getBasis() * normal;
        const pe::Vector3 point = trans_other * p;
        result.addContactPoint(normal, point - normal * margin, -dist + 2 * margin);
    }

    static int findClosestBoxFace(const pe::Vector3& half_extent, const pe::Vector3& loc) {
        int code = 0;
        auto dist = PE_REAL_MAX;
        for (int i = 0; i < 3; i++) {
            const pe::Real p_dist = PE_ABS(loc[i] - half_extent[i]);
            const pe::Real n_dist = PE_ABS(loc[i] + half_extent[i]);
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

    static bool intersectLinePlane(const pe::Vector3& start_line, const pe::Vector3& dir_line, const pe::Vector3& plane_pos, const pe::Vector3& plane_nor, const pe::Real tol, pe::Real& t) {
        const pe::Real nd = plane_nor.dot(dir_line);

        if (PE_ABS(nd) < tol) {
            // Line parallel to plane
            return false;
        }

        t = plane_nor.dot(plane_pos - start_line) / nd;
        return true;
    }

    bool BoxCylinderCollisionAlgorithm::intersectSegmentBox(
            const pe::Vector3& half_extent, const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl,
            pe::Real h_cyl, pe::Real tol, pe::Real& t_min, pe::Real& t_max) {
        t_min = PE_REAL_MIN;
        t_max = PE_REAL_MAX;

        if (PE_ABS(axis_cyl.x()) < tol) {
            // Segment parallel to the box x-faces
            if (PE_ABS(pos_cyl.x()) > half_extent.x())
                return false;
        } else {
            const pe::Real t1 = (-half_extent.x() - pos_cyl.x()) / axis_cyl.x();
            const pe::Real t2 = (+half_extent.x() - pos_cyl.x()) / axis_cyl.x();

            t_min = PE_MAX(t_min, PE_MIN(t1, t2));
            t_max = PE_MIN(t_max, PE_MAX(t1, t2));

            if (t_min > t_max)
                return false;
        }

        if (PE_ABS(axis_cyl.y()) < tol) {
            // Segment parallel to the box y-faces
            if (PE_ABS(pos_cyl.y()) > half_extent.y())
                return false;
        } else {
            const pe::Real t1 = (-half_extent.y() - pos_cyl.y()) / axis_cyl.y();
            const pe::Real t2 = (+half_extent.y() - pos_cyl.y()) / axis_cyl.y();

            t_min = PE_MAX(t_min, PE_MIN(t1, t2));
            t_max = PE_MIN(t_max, PE_MAX(t1, t2));

            if (t_min > t_max)
                return false;
        }

        if (PE_ABS(axis_cyl.z()) < tol) {
            // Capsule axis parallel to the box z-faces
            if (PE_ABS(pos_cyl.z()) > half_extent.z())
                return false;
        } else {
            const pe::Real t1 = (-half_extent.z() - pos_cyl.z()) / axis_cyl.z();
            const pe::Real t2 = (+half_extent.z() - pos_cyl.z()) / axis_cyl.z();

            t_min = PE_MAX(t_min, PE_MIN(t1, t2));
            t_max = PE_MIN(t_max, PE_MAX(t1, t2));

            if (t_min > t_max)
                return false;
        }

        // If both intersection points are outside the segment, no intersection
        if ((t_min < -h_cyl && t_max < -h_cyl) || (t_min > +h_cyl && t_max > +h_cyl))
            return false;

        // Clamp intersection points to segment length
        t_min = PE_CLAMP(t_min, -h_cyl, +h_cyl);
        t_max = PE_CLAMP(t_max, -h_cyl, +h_cyl);

        return true;
    }

    bool BoxCylinderCollisionAlgorithm::intersectSegmentCylinder(
            const pe::Vector3& start_seg, const pe::Vector3& dir_seg, pe::Real len_seg,
            const pe::Vector3& pos_cyl, const pe::Vector3& axis_cyl, pe::Real h_cyl, pe::Real radius_cyl,
            pe::Real tol, pe::Real &t_min, pe::Real &t_max) {
        t_min = PE_REAL_MIN;
        t_max = PE_REAL_MAX;

        const pe::Vector3 v = start_seg - pos_cyl;
        const pe::Real cd_sd = axis_cyl.dot(dir_seg);
        const pe::Real v_cd = v.dot(axis_cyl);
        const pe::Real v_sd = v.dot(dir_seg);
        const pe::Real v_v = v.dot(v);
        const pe::Real a = PE_R(1.0) - cd_sd * cd_sd;
        const pe::Real b = v_sd - v_cd * cd_sd;
        const pe::Real c = v_v - v_cd * v_cd - radius_cyl * radius_cyl;

        // Intersection with cylindrical surface.
        // a >= 0 always
        // a == 0 indicates line parallel to cylinder axis
        if (PE_ABS(a) < tol) {
            // line parallel to cylinder axis
            const pe::Real dist2 = (v - v_cd * axis_cyl).squaredNorm();
            if (dist2 > radius_cyl * radius_cyl) {
                return false;
            }
            t_min = -len_seg;
            t_max = +len_seg;
        } else {
            // line intersects cylindrical surface
            pe::Real delta = b * b - a * c;
            if (delta < 0) {
                return false; // no real roots, no intersection
            }
            delta = PE_SQRT(delta);
            t_min = (-b - delta) / a;
            t_max = (-b + delta) / a;
        }

        // Intersection with end-caps.
        pe::Real t1;
        const bool code1 = intersectLinePlane(start_seg, dir_seg, pos_cyl + h_cyl * axis_cyl, axis_cyl, tol, t1);
        pe::Real t2;
        const bool code2 = intersectLinePlane(start_seg, dir_seg, pos_cyl - h_cyl * axis_cyl, axis_cyl, tol, t2);
        if (code1 && code2) {
            // line intersects end-caps
            if (t1 < t2) {
                t_min = PE_MAX(t_min, t1);
                t_max = PE_MIN(t_max, t2);
            } else {
                t_min = PE_MAX(t_min, t2);
                t_max = PE_MIN(t_max, t1);
            }
            if (t_max < t_min)
                return false;
        } else {
            // line parallel to end-cap planes
            const pe::Real d1 = PE_ABS(axis_cyl.dot(pos_cyl + h_cyl * axis_cyl - start_seg));
            const pe::Real d2 = PE_ABS(axis_cyl.dot(pos_cyl - h_cyl * axis_cyl - start_seg));
            if (d1 > 2 * h_cyl || d2 > 2 * h_cyl)
                return false;
        }

        // If both intersection points are outside the segment, no intersection
        if ((t_min < -len_seg && t_max < -len_seg) || (t_min > +len_seg && t_max > +len_seg))
            return false;

        // Clamp to segment length
        t_min = PE_CLAMP(t_min, -len_seg, +len_seg);
        t_max = PE_CLAMP(t_max, -len_seg, +len_seg);

        return true;
    }

    bool BoxCylinderCollisionAlgorithm::getClosestPoints(pe_phys_shape::BoxShape *shape_box, pe_phys_shape::CylinderShape *shape_cyl,
                                                         const pe::Transform &trans_box, const pe::Transform &trans_cyl,
                                                         pe::Real radius_cyl, pe::Real height_cyl, const pe::Vector3 &box_half_extent,
                                                         pe::Real margin, ContactResult &result) {
        const pe::Transform trans_cyl2box = trans_box.inverse() * trans_cyl;
        const pe::Vector3 axis_cyl = trans_cyl2box.getBasis().col(1);
        const pe::Vector3 pos_cyl = trans_cyl2box.getOrigin();

        // - Loop over each direction of the box frame (i.e., each of the 3 face normals).
        // - For each direction, consider two segments that are on the cylindrical plane defined by the
        //   axis and the face normal. (Note that, in principle, we could only consider the segment "closest" to the box,
        //   but that is not trivial to define in all configurations). All segments are parameterized by t in [-H,H].
        // - For each segment, if the segment intersects the box, consider 2 candidate contact points: the 2
        //   intersection points. A contact is added if the segment point is inside the box.
        //   Furthermore, the corresponding box point is located on the box face that is closest to the intersection
        //   midpoint candidate.
        for (int i_dir = 0; i_dir < 3; i_dir++) {
            // current box direction
            pe::Vector3 n_dir = pe::Vector3::Zero();
            n_dir[i_dir] = 1;

            if (PE_ABS(axis_cyl[i_dir] - 1) < PE_EPS || PE_ABS(axis_cyl[i_dir] + 1) < PE_EPS) {
                continue;
            }

            pe::Vector3 r = n_dir.cross(axis_cyl).cross(axis_cyl).normalized() * radius_cyl;

            // Consider segments in both "negative" and "positive" r direction
            static pe::Real dir[2] = {-1, 1};
            for (const pe::Real j_dir : dir) {
                // Calculate current segment center
                pe::Vector3 cs = pos_cyl + j_dir * r;
                // Check for intersection with box
                pe::Real t_min, t_max;
                if (intersectSegmentBox(box_half_extent, cs, axis_cyl, height_cyl, PE_EPS, t_min, t_max)) {
                    // Consider the intersection points and their midpoint as candidates
                    pe::Vector3 p_min = cs + axis_cyl * t_min;
                    pe::Vector3 p_max = cs + axis_cyl * t_max;
                    pe::Vector3 p_mid = cs + axis_cyl * ((t_min + t_max) / 2);

                    // Pick box face that is closest to midpoint
                    const int i_face = findClosestBoxFace(box_half_extent, p_mid);

                    // Add a contact for any of the candidate points that is inside the box
                    addContactPointOnBox(p_min, i_face, box_half_extent, trans_box, margin, result); // 1st segment end
                    addContactPointOnBox(p_max, i_face, box_half_extent, trans_box, margin, result); // 2nd segment end
                }
            }
        }

        // - Loop over each direction of the box frame.
        // - For each direction, check intersection with the cylinder for all 4 edges parallel to that direction.
        // - If an edge intersects the cylinder, consider 3 candidate contact points: the 2 intersection points
        //   and their midpoint.
        for (int i_dir = 0; i_dir < 3; i_dir++) {
            // current box edge direction and half-length
            pe::Vector3 e_d = pe::Vector3::Zero();
            e_d[i_dir] = 1;
            const pe::Real e_h = box_half_extent[i_dir];
            // The other two box directions
            const int j_dir = (i_dir + 1) % 3;
            const int k_dir = (i_dir + 2) % 3;
            for (int j = -1; j <= +1; j += 2) {
                for (int k = -1; k <= +1; k += 2) {
                    pe::Vector3 e_c;
                    e_c[i_dir] = 0;
                    e_c[j_dir] = j * box_half_extent[j_dir];
                    e_c[k_dir] = k * box_half_extent[k_dir];
                    // Check for edge intersection with cylinder
                    pe::Real t_min, t_max;
                    if (intersectSegmentCylinder(e_c, e_d, e_h, pos_cyl, axis_cyl, height_cyl, radius_cyl, PE_EPS, t_min, t_max)) {
                        // Consider the intersection points and their midpoint as candidates
                        pe::Vector3 p_mid = e_c + e_d * ((t_min + t_max) / 2);

                        // Add a contact for any of the candidate points that is inside the cylinder
                        addContactPointOnCylinder(p_mid, pos_cyl, axis_cyl, radius_cyl, height_cyl, trans_box, margin, result);
                    }
                }
            }
        }

        return true;
    }


} // pe_phys_collision