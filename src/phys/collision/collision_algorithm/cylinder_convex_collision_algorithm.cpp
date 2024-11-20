#include "cylinder_convex_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "convex_convex_collision_algorithm.h"
#include "box_cylinder_collision_algorithm.h"

// cylinder-convex collision: refer to box-cylinder collision
namespace pe_phys_collision {

    bool CylinderConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                            pe::Transform trans_a, pe::Transform trans_b,
                                                            pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
               shape_b->getType() == pe_phys_shape::ShapeType::Cylinder))) {
            return false;
        }
        constexpr auto margin = PE_MARGIN;

#   if false
        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a)->getMesh() :
                       dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b)->getMesh() :
                       dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b)->getMesh();
        auto& edges_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a)->getUniqueEdges();
        auto& edges_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b)->getUniqueEdges();

        pe::Vector3 sep;

        VertexArray world_vertices_b1;
        VertexArray world_vertices_b2;

        if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(shape_a, shape_b,
                                                                mesh_a, mesh_b,
                                                                edges_a, edges_b,
                                                                trans_a, trans_b, sep, margin, result)) {
            return false;
        }
        ConvexConvexCollisionAlgorithm::clipHullAgainstHull(sep,
                                                            mesh_a, mesh_b, trans_a, trans_b,
                                                            -refScale, margin,
                                                            world_vertices_b1, world_vertices_b2,
                                                            margin, result);
        return true;
#   else
        const auto shape_mesh = dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b);
        const auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? shape_a : shape_b);
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? trans_a : trans_b;
        auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;

        auto& mesh = shape_mesh->getMesh();
        const pe::Real cyl_r = shape_cyl->getRadius();
        const pe::Real cyl_h = shape_cyl->getHeight() / pe::Real(2.0);
        pe::Transform trans_cyl2mesh = trans_mesh.inverse() * trans_cyl;
        pe::Vector3 axis_cyl = trans_cyl2mesh.getBasis().getColumn(1);
        pe::Vector3 pos_cyl = trans_cyl2mesh.getOrigin();

        pe::Vector3 cyl_AA, cyl_BB;
        shape_cyl->getAABB(trans_cyl2mesh, cyl_AA, cyl_BB);
        pe::Array<int> intersect;
        shape_mesh->getIntersectFaces(cyl_AA, cyl_BB, intersect);

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh);

        // check whether there is a face that supports the cylinder
        // consider the four directions of the cylinder
        static int dir_x[4] = {1, -1, -1, 1};
        static int dir_y[4] = {1, 1, -1, -1};
        for (const auto fi : intersect) {
            auto& f = mesh.faces[fi];

            pe::Real a_dot_c = axis_cyl.dot(f.normal);
            pe::Vector3 d_x, d_y = axis_cyl, d_z;
            bool vertical_flag = false;
            if (PE_APPROX_EQUAL(PE_ABS(a_dot_c), 1)) {
                d_z = axis_cyl.cross(pe::Vector3::right()).normalized();
                d_x = axis_cyl.cross(d_z).normalized();
                vertical_flag = true;
            } else {
                d_x = f.normal.cross(axis_cyl).cross(axis_cyl).normalized();
            }

            for (int i_dir = 0; i_dir < 4; i_dir++) {
                const pe::Vector3 pos_seg = pos_cyl + d_x * dir_x[i_dir] * cyl_r + d_y * dir_y[i_dir] * cyl_h;
                const pe::Vector3 dir_seg = (dir_x[i_dir] == dir_y[i_dir] ? d_y : d_x) * -dir_x[i_dir];
                const pe::Real l_seg = (dir_x[i_dir] == dir_y[i_dir] ? cyl_h : cyl_r) * 2;
                pe::Real t1, t2, d1, d2;
                if (intersectSegmentFace(f, mesh, pos_seg, dir_seg, l_seg, margin, t1, t2, d1, d2)) {
                    pe::Vector3 p1 = trans_mesh * (pos_seg + dir_seg * t1 - f.normal * d1);
                    pe::Vector3 p2 = trans_mesh * (pos_seg + dir_seg * t2 - f.normal * d2);
                    pe::Vector3 n = trans_mesh.getBasis() * f.normal;
                    result.addContactPoint(n, p1 - n * margin, d1 + margin * 2);
                    result.addContactPoint(n, p2 - n * margin, d2 + margin * 2);
                }
            }
            // to obtain stability, add two more contact points
            if (vertical_flag) {
                for (int i_dir = 0; i_dir < 3; i_dir += 2) {
                    const pe::Vector3 pos_seg = pos_cyl + d_z * dir_x[i_dir] * cyl_r + d_y * dir_y[i_dir] * cyl_h;
                    const pe::Vector3 dir_seg = d_y * -dir_x[i_dir];
                    const pe::Real l_seg = cyl_h * 2;
                    pe::Real t1, t2, d1, d2;
                    if (intersectSegmentFace(f, mesh, pos_seg, dir_seg, l_seg, margin, t1, t2, d1, d2)) {
                        pe::Vector3 p1 = trans_mesh * (pos_seg + dir_seg * t1 - f.normal * d1);
                        pe::Vector3 p2 = trans_mesh * (pos_seg + dir_seg * t2 - f.normal * d2);
                        pe::Vector3 n = trans_mesh.getBasis() * f.normal;
                        result.addContactPoint(n, p1 - n * margin, d1 + margin * 2);
                        result.addContactPoint(n, p2 - n * margin, d2 + margin * 2);
                    }
                }
            }
        }

        // check whether there is an edge that intersects the cylinder
        for (const auto fi : intersect) {
            auto& f = mesh.faces[fi];
            for (int i = 0; i < (int)f.indices.size(); i++) {
                const auto& v1 = mesh.vertices[f.indices[i]].position;
                const auto& v2 = mesh.vertices[f.indices[(i + 1) % f.indices.size()]].position;
                pe::Real t1, t2;
                pe::Vector3 start_seg = (v1 + v2) / 2;
                pe::Real l_seg = (v2 - v1).norm() / 2;
                pe::Vector3 dir_seg = (v2 - v1) / (l_seg * 2);
                if (BoxCylinderCollisionAlgorithm::intersectSegmentCylinder(start_seg, dir_seg, l_seg, pos_cyl, axis_cyl, cyl_h, cyl_r, margin, t1, t2)) {
                    pe::Vector3 p1 = start_seg + dir_seg * t1;
                    pe::Vector3 p2 = start_seg + dir_seg * t2;
                    pe::Vector3 p_mid = (p1 + p2) / 2;
                    BoxCylinderCollisionAlgorithm::addContactPointOnCylinder(p_mid, pos_cyl, axis_cyl, cyl_r, cyl_h, trans_mesh, margin, result);
                }
            }
        }

        result.setSwapFlag(false);
        return true;
#   endif
    }

    bool CylinderConvexCollisionAlgorithm::intersectSegmentFace(
            const pe::Mesh::Face& face, const pe::Mesh& mesh,
            const pe::Vector3& pos_seg, const pe::Vector3& dir_seg, pe::Real l_seg,
            pe::Real margin, pe::Real& t1, pe::Real& t2, pe::Real& d1, pe::Real& d2) {
        const pe::Vector3 origin = mesh.vertices[face.indices[0]].position;
        const pe::Vector3 end_seg = pos_seg + dir_seg * l_seg;
        const pe::Vector3 r1 = pos_seg - origin;
        const pe::Vector3 r2 = end_seg - origin;
        const pe::Real dist1 = r1.dot(face.normal);
        const pe::Real dist2 = r2.dot(face.normal);

        // both on the same side of the face plane
        if ((dist1 > 0 && dist2 > 0) || (dist1 <= 0 && dist2 <= 0)) {
            return false;
        }

        // one upside and one downside the face plane
        const pe::Real t = dist1 / (dist1 - dist2) * l_seg;
        const pe::Vector3 v = pos_seg + dir_seg * t;

        // check if the start or end point is inside the vertical face defined by the edge and the normal
        pe::Real t_min = 0, t_max = l_seg;
        auto min_dist_to_edge = PE_REAL_MAX;
        for (int i = 0; i < (int)face.indices.size(); i++) {
            const pe::Vector3& v0 = mesh.vertices[face.indices[i]].position;
            const pe::Vector3& v1 = mesh.vertices[face.indices[(i + 1) % face.indices.size()]].position;
            const pe::Vector3 to_center = face.normal.cross(v1 - v0).normalized();
            const pe::Real dir_dot_to_center = dir_seg.dot(to_center);
            const pe::Real dist_to_edge = (v0 - pos_seg).dot(to_center);
            min_dist_to_edge = PE_MIN(min_dist_to_edge, PE_ABS(dist_to_edge));
            if (!PE_APPROX_EQUAL(dir_dot_to_center, 0)) {
                if (dir_dot_to_center > 0) {
                    t_min = PE_MAX(t_min, dist_to_edge / dir_dot_to_center);
                } else {
                    t_max = PE_MIN(t_max, dist_to_edge / dir_dot_to_center);
                }
            } else {
                if ((v0 - v).cross(v1 - v).dot(face.normal) < 0) {
                    return false;
                }
            }
        }

        if (t < t_min || t > t_max) {
            return false;
        }
        t1 = dist1 <= 0 ? t_min : t;
        t2 = dist2 <= 0 ? t_max : t;
        d1 = dist1 <= 0 ? dist1 * (t - t_min) / t : 0;
        d2 = dist2 <= 0 ? dist2 * (t_max - t) / (l_seg - t) : 0;
        if (-d1 > min_dist_to_edge + margin || -d2 > min_dist_to_edge + margin) {
            return false;
        }
        return true;
    }

    // bool CylinderConvexCollisionAlgorithm::pointInsideCylinder(const pe::Vector3 &v, pe::Real h, pe::Real r, const pe::Transform& trans,
    //                                                            pe::Real margin, ContactResult& result) {
    //     if (PE_ABS(v.y) > h) {
    //         return false;
    //     }
    //     pe::Real rp = PE_SQRT(v.x * v.x + v.z * v.z);
    //     if (rp > r) {
    //         return false;
    //     }
    //     pe::Real dist2bottom = h - PE_ABS(v.y);
    //     pe::Real dist2side = r - rp;
    //     pe::Vector3 n = dist2bottom < dist2side ? pe::Vector3(0, (v.y > 0 ? 1.0 : -1.0), 0) : pe::Vector3(v.x, 0, v.z).normalized();
    //     pe::Vector3 p = dist2bottom < dist2side ? v + n * dist2bottom : v + n * dist2side;
    //     pe::Real depth = dist2bottom < dist2side ? dist2bottom : dist2side;
    //     p = trans * p;
    //     n = trans.getBasis() * n;
    //     result.addContactPoint(n, p - n * margin, -depth + margin * 2);
    //     std::cout << "add point 1: " << n << p << depth << std::endl;
    //     return true;
    // }
    //
    // static bool intersectSegmentCircle(const pe::Vector3 &v1, const pe::Vector3 &v2, pe::Real r, pe::Real h, pe::Vector3& t) {
    //     if (PE_APPROX_EQUAL(v1.y, v2.y) || (v1.y >= h && v2.y >= h) || (v1.y <= h && v2.y <= h)) {
    //         return false;
    //     }
    //     pe::Vector3 origin = pe::Vector3(0, h, 0);
    //     pe::Vector3 d1 = v1 - origin;
    //     pe::Vector3 d2 = v2 - origin;
    //     pe::Vector3 v = origin + d1 * (h - v2.y) / (v1.y - v2.y) + d2 * (h - v1.y) / (v2.y - v1.y);
    //     pe::Real dist = (v - origin).norm2();
    //     if (dist > r * r) {
    //         return false;
    //     }
    //     t = v;
    //     return true;
    // }
    //
    // static bool intersectSegmentCylinderSide(const pe::Vector3 &v1, const pe::Vector3 &v2, pe::Real r, pe::Real h,
    //                                          int& c, pe::Vector3& t1, pe::Vector3& t2) {
    //     c = 0;
    //     pe::Vector3 dir = (v2 - v1).normalized();
    //     if (PE_APPROX_EQUAL(dir.y, 1)) return false;
    //     pe::Real dir_xz2 = dir.x * dir.x + dir.z * dir.z;
    //     pe::Real d = PE_ABS(v1.dot(pe::Vector3(-dir.z, 0, dir.x).normalized()));
    //     if (d > r) return false;
    //     pe::Real l0 = (v1.y * dir.y - v1.dot(dir)) / dir_xz2;
    //     pe::Real l = (v2 - v1).norm();
    //     pe::Real dl = PE_SQRT((r * r - d * d) / dir_xz2);
    //
    //     if (l0 - dl > 0 && l0 - dl < l) {
    //         pe::Vector3 p1 = v1 + dir * (l0 - dl);
    //         if (PE_ABS(p1.y) < h) {
    //             t1 = p1;
    //             c++;
    //         }
    //     }
    //     if (l0 + dl > 0 && l0 + dl < l) {
    //         pe::Vector3 p2 = v1 + dir * (l0 + dl);
    //         if (PE_ABS(p2.y) < h) {
    //             (c == 1 ? t2 : t1) = p2;
    //             c++;
    //         }
    //     }
    //     return c > 0;
    // }
    //
    // void CylinderConvexCollisionAlgorithm::intersectSegmentCylinder(const pe::Vector3 &v1, const pe::Vector3 &v2,
    //                                                                 pe::Real h, pe::Real r,
    //                                                                 const pe::Transform& trans,
    //                                                                 pe::Real margin, ContactResult& result) {
    //     pe::Vector3 p_top, p_bottom, p_side1, p_side2;
    //     bool i_top = intersectSegmentCircle(v1, v2, r, h, p_top);
    //     bool i_bottom = intersectSegmentCircle(v1, v2, r, -h, p_bottom);
    //     int c;
    //     bool i_side = intersectSegmentCylinderSide(v1, v2, r, h, c, p_side1, p_side2);
    //     if (!i_side && !i_top && !i_bottom) {
    //         return;
    //     }
    //
    //     if (i_top && i_bottom) {
    //         pe::Real l_top = PE_SQRT(p_top.x * p_top.x + p_top.z * p_top.z);
    //         pe::Real l_bottom = PE_SQRT(p_bottom.x * p_bottom.x + p_bottom.z * p_bottom.z);
    //         pe::Vector3 n_top = pe::Vector3(p_top.x, 0, p_top.z) / l_top;
    //         pe::Vector3 n_bottom = pe::Vector3(p_bottom.x, 0, p_bottom.z) / l_bottom;
    //         pe::Vector3 v_top = trans * (pe::Vector3(0, h, 0) + n_top * r);
    //         pe::Vector3 v_bottom = trans * (pe::Vector3(0, -h, 0) + n_bottom * r);
    //         n_top = trans.getBasis() * n_top;
    //         n_bottom = trans.getBasis() * n_bottom;
    //         pe::Real d_top = r - l_top;
    //         pe::Real d_bottom = r - l_bottom;
    //         result.addContactPoint(n_top, v_top - n_top * margin, -d_top + margin * 2);
    //         result.addContactPoint(n_bottom, v_bottom - n_bottom * margin, -d_bottom + margin * 2);
    //         std::cout << "add edge 2: " << n_top << v_top << d_top << n_bottom << v_bottom << d_bottom << std::endl;
    //     } else if (i_side && (i_top || i_bottom) && c == 1) {
    //         pe::Real cy = p_side1.y + (i_top ? h : -h);
    //         pe::Vector3 t = pe::Vector3(p_side1.x, cy > 0 ? h : -h, p_side1.z);
    //         pe::Vector3 n = pe::Vector3(t.x, t.y - t.dot(v1 - v2) / (v1.y - v2.y), t.z).normalized();
    //         pe::Real d = PE_ABS((v1 - t).dot(n));
    //         pe::Vector3 n_w = trans.getBasis() * n;
    //         t = trans * t;
    //         result.addContactPoint(n_w, t - n_w * margin, -d + margin * 2);
    //         std::cout << "add edge 1-1: " << n_w << t << d << std::endl;
    //     } else if (i_side && c == 2) {
    //         pe::Vector3 p_side = (p_side1 + p_side2) / 2;
    //         pe::Real l = pe::Vector3(p_side.x, 0, p_side.z).norm();
    //         pe::Real d_side = r - l;
    //         pe::Real d_top = h - PE_ABS(p_side.y);
    //         pe::Vector3 n = d_side < d_top ? pe::Vector3(p_side.x, 0, p_side.z) / l : pe::Vector3(0, p_side.y > 0 ? 1.0 : -1.0, 0);
    //         if (d_side < d_top) {
    //             pe::Vector3 t = trans * (p_side + d_side * n);
    //             n = trans.getBasis() * n;
    //             result.addContactPoint(n, t - n * margin, -d_side + margin * 2);
    //             std::cout << "add edge 1-2-1: " << n << t << d_side << std::endl;
    //         } else {
    //             pe::Real d1 = PE_ABS((p_side.y > 0 ? h : -h) - p_side1.y);
    //             pe::Vector3 t1 = trans * (p_side1 + d1 * n);
    //             pe::Real d2 = PE_ABS((p_side.y > 0 ? h : -h) - p_side2.y);
    //             pe::Vector3 t2 = trans * (p_side2 + d2 * n);
    //             n = trans.getBasis() * n;
    //             result.addContactPoint(n, t1 - n * margin, -d1 + margin * 2);
    //             result.addContactPoint(n, t2 - n * margin, -d2 + margin * 2);
    //             std::cout << "add edge 1-2-2: " << n << t1 << d1 << n << t2 << d2 << std::endl;
    //         }
    //     }
    // }
    //
    // static bool pointInsideFace(const pe::Array<pe::Vector3>& vs, const pe::Vector3& n, const pe::Vector3& p) {
    //     for (int i = 0; i < (int)vs.size(); i++) {
    //         int v1 = i;
    //         int v2 = (i + 1) % (int)vs.size();
    //         if ((vs[v1] - p).cross(vs[v2] - p).dot(n) < 0) {
    //             return false;
    //         }
    //     }
    //     return true;
    // }
    //
    // static bool intersectFaceCircle(const pe::Array<pe::Vector3>& vs, const pe::Vector3& n, pe::Real r, pe::Real h,
    //                                 int& c, pe::Vector3 ts[2]) {
    //     c = 0;
    //     pe::Real xz2 = n.x * n.x + n.z * n.z;
    //     if (PE_APPROX_EQUAL(xz2, 0)) {
    //         return false;
    //     }
    //     pe::Real t = vs[0].dot(n) - h * n.y;
    //     pe::Real dt2 = r * r * xz2 - t * t;
    //     if (dt2 < 0) {
    //         return false;
    //     }
    //     pe::Real dt = PE_SQRT(dt2);
    //     pe::Real z1 = (n.z * t + n.x * dt) / xz2;
    //     pe::Real z2 = (n.z * t - n.x * dt) / xz2;
    //     pe::Real x1 = (n.x * t - n.z * dt) / xz2;
    //     pe::Real x2 = (n.x * t + n.z * dt) / xz2;
    //     pe::Vector3 p1 = pe::Vector3(x1, h, z1);
    //     pe::Vector3 p2 = pe::Vector3(x2, h, z2);
    //     ts[0] = p1;
    //     ts[1] = p2;
    //     bool pit1 = pointInsideFace(vs, n, p1);
    //     bool pit2 = pointInsideFace(vs, n, p2);
    //     c = (pit1 * 1) | (pit2 * 2);
    //     return c > 0;
    // }
    //
    // static bool intersectFaceCylinderSide(const pe::Array<pe::Vector3>& vs, const pe::Vector3& n, pe::Real r, pe::Real h,
    //                                       int& c, pe::Vector3 ts[4]) {
    //     c = 0;
    //     if (PE_APPROX_EQUAL(n.y, 0)) {
    //         return false;
    //     }
    //     pe::Real dxz = PE_SQRT(n.x * n.x + n.z * n.z);
    //     pe::Real h0 = n.dot(vs[0]) / n.y;
    //     pe::Real h1 = h0 - r * dxz / n.y;
    //     pe::Real h2 = pe::Real(2.0) * h0 - h1;
    //     pe::Vector3 ps[4];
    //     if (PE_APPROX_EQUAL(dxz, 0)) {
    //         ps[0] = {-r, h0, 0};
    //         ps[1] = {r, h0, 0};
    //         ps[2] = {0, h0, -r};
    //         ps[3] = {0, h0, r};
    //     } else {
    //         pe::Vector3 nor = pe::Vector3(n.x, 0, n.z) / dxz;
    //         ps[0] = nor * r; ps[0].y = h1;
    //         ps[1] = -nor * r; ps[1].y = h2;
    //         PE_SWAP(nor.x, nor.z);
    //         nor.x = -nor.x;
    //         ps[2] = nor * r; ps[2].y = h0;
    //         ps[3] = -nor * r; ps[3].y = h0;
    //     }
    //     for (const auto& p : ps) {
    //         if (PE_ABS(p.y) < h && pointInsideFace(vs, n, p)) {
    //             ts[c++] = p;
    //         }
    //     }
    //     return c > 0;
    // }
    //
    // void CylinderConvexCollisionAlgorithm::intersectFaceCylinder(const pe::Array<pe::Vector3>& vs, const pe::Vector3& n, pe::Real h, pe::Real r,
    //                                                              const pe::Transform &trans, pe::Real margin, ContactResult &result) {
    //     int c_top, c_bottom, c_side;
    //     pe::Vector3 p_tops[2], p_bottoms[2];
    //     bool i_top = intersectFaceCircle(vs, n, r, h, c_top, p_tops);
    //     bool i_bottom = intersectFaceCircle(vs, n, r, -h, c_bottom, p_bottoms);
    //     pe::Vector3 p_sides[4];
    //     bool i_side = intersectFaceCylinderSide(vs, n, r, h, c_side, p_sides);
    //
    //     if (i_top) {
    //         pe::Vector3 p_top = (p_tops[0] + p_tops[1]) / 2;
    //         pe::Real l_top = PE_SQRT(p_top.x * p_top.x + p_top.z * p_top.z);
    //         pe::Real d_top = r - l_top;
    //         pe::Vector3 n_top = pe::Vector3(p_top.x, 0, p_top.z) / l_top;
    //         n_top = trans.getBasis() * n_top;
    //         if (c_top == 3 || c_top == 1) {
    //             p_top = trans * p_tops[0];
    //             result.addContactPoint(n_top, p_top - n_top * margin, -d_top + margin * 2);
    //             std::cout << "add triangle 2-1.1: " << n_top << p_top << d_top << std::endl;
    //         }
    //         if (c_top == 3 || c_top == 2) {
    //             p_top = trans * p_tops[1];
    //             result.addContactPoint(n_top, p_top - n_top * margin, -d_top + margin * 2);
    //             std::cout << "add triangle 2-1.2: " << n_top << p_top << d_top << std::endl;
    //         }
    //     }
    //     if (i_bottom) {
    //         pe::Vector3 p_bottom = (p_bottoms[0] + p_bottoms[1]) / 2;
    //         pe::Real l_bottom = PE_SQRT(p_bottom.x * p_bottom.x + p_bottom.z * p_bottom.z);
    //         pe::Real d_bottom = r - l_bottom;
    //         pe::Vector3 n_bottom = pe::Vector3(p_bottom.x, 0, p_bottom.z) / l_bottom;
    //         n_bottom = trans.getBasis() * n_bottom;
    //         if (c_bottom == 3 || c_bottom == 1) {
    //             p_bottom = trans * p_bottoms[0];
    //             result.addContactPoint(n_bottom, p_bottom - n_bottom * margin, -d_bottom + margin * 2);
    //             std::cout << "add triangle 2-2.1: " << n_bottom << p_bottom << d_bottom << std::endl;
    //         }
    //         if (c_bottom == 3 || c_bottom == 2) {
    //             p_bottom = trans * p_bottoms[1];
    //             result.addContactPoint(n_bottom, p_bottom - n_bottom * margin, -d_bottom + margin * 2);
    //             std::cout << "add triangle 2-2.2: " << n_bottom << p_bottom << d_bottom << std::endl;
    //         }
    //     }
    //
    //     if (i_side && (i_top || i_bottom) && c_side == 1) {
    //         pe::Vector3 p_top = trans * pe::Vector3(p_sides[0].x, i_top ? h : -h, p_sides[0].z);
    //         pe::Real d = PE_ABS(((i_top ? h : -h) - p_sides[0].y) * n.y);
    //         pe::Vector3 n_w = trans.getBasis() * n;
    //         result.addContactPoint(-n_w, p_top + n_w * margin, -d + margin * 2);
    //         std::cout << "add triangle 1-1: " << -n_w << pe::Vector3(p_sides[0].x, i_top ? h : -h, p_sides[0].z) << d << std::endl;
    //     } else if (i_side && c_side >= 2) {
    //         pe::Real center_h = (p_sides[0].y + p_sides[1].y) / pe::Real(2.0);
    //         for (int i = 0; i < c_side; i++) {
    //             pe::Vector3 p_top = trans * pe::Vector3(p_sides[i].x, center_h > 0 ? h : -h, p_sides[i].z);
    //             pe::Real d = h - PE_ABS(p_sides[i].y);
    //             pe::Vector3 n_w = trans.getBasis() * pe::Vector3(0, pe::Real(center_h > 0 ? 1.0 : -1.0), 0);
    //             result.addContactPoint(n_w, p_top - n_w * margin, -d + margin * 2);
    //             std::cout << "add triangle 1-2." + std::to_string(i) << ": " << n_w << p_top << d << std::endl;
    //         }
    //     }
    // }

} // pe_phys_collision