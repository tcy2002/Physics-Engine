#include "cylinder_convex_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "convex_convex_collision_algorithm.h"
#include "utils/logger.h"

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
        pe::Real margin = PE_MARGIN;

#   if false
        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       ((pe_phys_shape::CylinderShape*)shape_a)->getMesh() :
                       ((pe_phys_shape::ConvexMeshShape*)shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       ((pe_phys_shape::CylinderShape*)shape_b)->getMesh() :
                       ((pe_phys_shape::ConvexMeshShape*)shape_b)->getMesh();
        auto& edges_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        ((pe_phys_shape::CylinderShape*)shape_a)->getUniqueEdges() :
                        ((pe_phys_shape::ConvexMeshShape*)shape_a)->getUniqueEdges();
        auto& edges_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                        ((pe_phys_shape::CylinderShape*)shape_b)->getUniqueEdges() :
                        ((pe_phys_shape::ConvexMeshShape*)shape_b)->getUniqueEdges();

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
        auto shape_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b;
        auto& mesh = ((pe_phys_shape::ConvexMeshShape*)shape_mesh)->getMesh();
        auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;
        auto shape_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? shape_a : shape_b;
        pe::Real cyl_r = ((pe_phys_shape::CylinderShape*)shape_cyl)->getRadius();
        pe::Real cyl_h = ((pe_phys_shape::CylinderShape*)shape_cyl)->getHeight() / pe::Real(2.0);
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? trans_a : trans_b;

        pe::Transform trans_mesh2cyl = trans_cyl.inverse() * trans_mesh;
        pe::Transform trans_cyl2mesh = trans_mesh2cyl.inverse();
        pe::Vector3 cyl_AA, cyl_BB;
        shape_cyl->getAABB(trans_cyl2mesh, cyl_AA, cyl_BB);
        pe::Array<int> intersect;
        ((pe_phys_shape::ConvexMeshShape*)shape_mesh)->getIntersectFaces(cyl_AA, cyl_BB, intersect);

        pe::Vector3 vertices[3];
        result.setSwapFlag(shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh);
        for (auto fi : intersect) {
            auto& f = mesh.faces[fi];
            for (int i = 0; i < (int)f.indices.size() - 2; i++) {
                vertices[0] = trans_mesh2cyl * mesh.vertices[f.indices[0]].position;
                vertices[1] = trans_mesh2cyl * mesh.vertices[f.indices[i + 1]].position;
                vertices[2] = trans_mesh2cyl * mesh.vertices[f.indices[i + 2]].position;
                pe::Vector3 n = trans_mesh2cyl.getBasis() * f.normal;

                bool point_inside[3]{false};
                for (int i = 0; i < 3; i++) {
                    point_inside[i] = pointInsideCylinder(vertices[i], cyl_h, cyl_r, trans_cyl, margin, result);
                }

                for (int i = 0; i < 3; i++) {
                    int v1 = i;
                    int v2 = (i + 1) % 3;
                    if (!point_inside[v1] && !point_inside[v2]) {
                        intersectSegmentCylinder(vertices[v1], vertices[v2], n, cyl_h, cyl_r, trans_cyl, margin, result);
                    }
                }

                if (!point_inside[0] && !point_inside[1] && !point_inside[2]) {
                    intersectTriangleCylinder(vertices, n, cyl_h, cyl_r, trans_cyl, margin, result);
                }
            }
        }
        result.setSwapFlag(false);

        return true;
#   endif
    }

    bool CylinderConvexCollisionAlgorithm::pointInsideCylinder(const pe::Vector3 &v, pe::Real h, pe::Real r,
                                                               const pe::Transform& trans, pe::Real margin, ContactResult& result) {
        if (PE_ABS(v.y) > h) {
            return false;
        }
        pe::Real rp = PE_SQRT(v.x * v.x + v.z * v.z);
        if (rp > r) {
            return false;
        }
        pe::Real dist2bottom = h - PE_ABS(v.y);
        pe::Real dist2side = r - rp;
        pe::Vector3 n = dist2bottom < dist2side ? pe::Vector3(0, v.y, 0).normalized() : pe::Vector3(v.x, 0, v.z).normalized();
        pe::Vector3 p = dist2bottom < dist2side ? v + n * dist2bottom : v + n * dist2side;
        pe::Real depth = dist2bottom < dist2side ? dist2bottom : dist2side;
        p = trans * p;
        n = trans.getBasis() * n;
        result.addContactPoint(n, p - n * margin, -depth + margin * 2);
        std::cout << "add point 1: " << n << p << depth << std::endl;
        return true;
    }

    static bool intersectSegmentCircle(const pe::Vector3 &v1, const pe::Vector3 &v2, pe::Real r, pe::Real h, pe::Vector3& t) {
        if (PE_APPROX_EQUAL(v1.y, v2.y) || (v1.y >= h && v2.y >= h) || (v1.y <= h && v2.y <= h)) {
            return false;
        }
        pe::Vector3 origin = pe::Vector3(0, h, 0);
        pe::Vector3 d1 = v1 - origin;
        pe::Vector3 d2 = v2 - origin;
        pe::Vector3 v = origin + d1 * (h - v2.y) / (v1.y - v2.y) + d2 * (h - v1.y) / (v2.y - v1.y);
        pe::Real dist = (v - origin).norm2();
        if (dist > r * r) {
            return false;
        }
        t = v;
        return true;
    }

    static bool intersectSegmentCylinderSide(const pe::Vector3 &v1, const pe::Vector3 &v2, pe::Real r, pe::Real h,
                                             int& c, pe::Vector3& t1, pe::Vector3& t2) {
        c = 0;
        pe::Vector3 dir = (v2 - v1).normalized();
        if (PE_APPROX_EQUAL(dir.y, 1)) return false;
        pe::Real dir_xz2 = dir.x * dir.x + dir.z * dir.z;
        pe::Real d = PE_ABS(v1.dot(pe::Vector3(-dir.z, 0, dir.x).normalized()));
        if (d > r) return false;
        pe::Real l0 = (v1.y * dir.y - v1.dot(dir)) / dir_xz2;
        pe::Real l = (v2 - v1).norm();
        pe::Real dl = PE_SQRT((r * r - d * d) / dir_xz2);

        if (l0 - dl > 0 && l0 - dl < l) {
            pe::Vector3 p1 = v1 + dir * (l0 - dl);
            if (PE_ABS(p1.y) < h) {
                t1 = p1;
                c++;
            }
        }
        if (l0 + dl > 0 && l0 + dl < l) {
            pe::Vector3 p2 = v1 + dir * (l0 + dl);
            if (PE_ABS(p2.y) < h) {
                (c == 1 ? t2 : t1) = p2;
                c++;
            }
        }
        return c > 0;
    }

    void CylinderConvexCollisionAlgorithm::intersectSegmentCylinder(const pe::Vector3 &v1, const pe::Vector3 &v2,
                                                                    const pe::Vector3 &n, pe::Real h, pe::Real r,
                                                                    const pe::Transform& trans, pe::Real margin, ContactResult& result) {
        pe::Vector3 p_top, p_bottom;
        bool i_top = intersectSegmentCircle(v1, v2, r, h, p_top);
        bool i_bottom = intersectSegmentCircle(v1, v2, r, -h, p_bottom);

        if (i_top && i_bottom) {
            pe::Vector3 n_top = (p_top - pe::Vector3(0, h, 0)).normalized();
            pe::Vector3 n_bottom = (p_bottom - pe::Vector3(0, -h, 0)).normalized();
            pe::Vector3 v_top = trans * (pe::Vector3(0, h, 0) + n_top * r);
            pe::Vector3 v_bottom = trans * (pe::Vector3(0, -h, 0) + n_bottom * r);
            n_top = trans.getBasis() * n_top;
            n_bottom = trans.getBasis() * n_bottom;
            pe::Real d_top = r - (p_top - pe::Vector3(0, h, 0)).norm();
            pe::Real d_bottom = r - (p_bottom - pe::Vector3(0, -h, 0)).norm();
            result.addContactPoint(n_top, v_top - n_top * margin, -d_top + margin * 2);
            result.addContactPoint(n_bottom, v_bottom - n_bottom * margin, -d_bottom + margin * 2);
            std::cout << "add edge 2: " << n_top << v_top << d_top << n_bottom << v_bottom << d_bottom << std::endl;
        }

        pe::Vector3 p_side1, p_side2;
        int c;
        bool i_side = intersectSegmentCylinderSide(v1, v2, r, h, c, p_side1, p_side2);
        if (!i_side && !i_top && !i_bottom) {
            return;
        }

        if (i_side && (i_top || i_bottom) && c == 1) {
            pe::Real center = ((i_top ? h : -h) + p_side1.y) / 2;
            pe::Vector3 t = pe::Vector3(p_side1.x, center > 0 ? h : -h, p_side1.z);
            pe::Real d = (h - PE_ABS(p_side1.y)) * PE_ABS(n.y);
            pe::Vector3 n_w = trans.getBasis() * -n;
            t = trans * t;
            result.addContactPoint(n_w, t - n_w * margin, -d + margin * 2);
            std::cout << "add edge 1-1: " << n_w << t << d << std::endl;
        } else if (i_side && c == 2) {
            pe::Vector3 p_side = (p_side1 + p_side2) / 2;
            pe::Real dt = (p_side - pe::Vector3(0, p_side.y, 0)).norm();
            pe::Real d_side = r - dt;
            pe::Real d_top = h - PE_ABS(p_side.y);
            pe::Vector3 n = d_side < d_top ? (p_side - pe::Vector3(0, p_side.y, 0)) / dt : pe::Vector3(0, p_side.y, 0).normalized();
            if (d_side < d_top) {
                pe::Vector3 t = trans * (p_side + d_side * n);
                n = trans.getBasis() * n;
                result.addContactPoint(n, t - n * margin, -d_side + margin * 2);
                std::cout << "add edge 1-2-1: " << t << n << d_side << std::endl;
            } else {
                pe::Real d1 = PE_ABS((p_side.y > 0 ? h : -h) - p_side1.y);
                pe::Vector3 t1 = trans * (p_side1 + d1 * n);
                pe::Real d2 = PE_ABS((p_side.y > 0 ? h : -h) - p_side2.y);
                pe::Vector3 t2 = trans * (p_side2 + d2 * n);
                n = trans.getBasis() * n;
                result.addContactPoint(n, t1 - n * margin, -d1 + margin * 2);
                result.addContactPoint(n, t2 - n * margin, -d2 + margin * 2);
                std::cout << "add edge 1-2-2: " << n << t1 << d1 << n << t2 << d2 << std::endl;
            }

        } else {
            PE_LOG_ERROR << "intersectSegmentCylinder: unexpected case" << PE_ENDL;
        }
    }

    static bool pointInsideTriangle(const pe::Vector3 vs[3], const pe::Vector3& n, const pe::Vector3& p) {
        return (vs[0] - p).cross(vs[1] - p).dot(n) > 0 &&
               (vs[1] - p).cross(vs[2] - p).dot(n) > 0 &&
               (vs[2] - p).cross(vs[0] - p).dot(n) > 0;
    }

    static bool intersectTriangleCircle(const pe::Vector3 vs[3], const pe::Vector3& n, pe::Real r, pe::Real h,
                                        int& c, pe::Vector3 ts[2]) {
        c = 0;
        pe::Real xz2 = n.x * n.x + n.z * n.z;
        if (PE_APPROX_EQUAL(xz2, 0)) {
            return false;
        }
        pe::Real t = vs[0].dot(n) - h * n.y;
        pe::Real dt2 = r * r * xz2 - t * t;
        if (dt2 < 0) {
            return false;
        }
        pe::Real dt = PE_SQRT(dt2);
        pe::Real z1 = (n.z * t + n.x * dt) / xz2;
        pe::Real z2 = (n.z * t - n.x * dt) / xz2;
        pe::Real x1 = (n.x * t - n.z * dt) / xz2;
        pe::Real x2 = (n.x * t + n.z * dt) / xz2;
        pe::Vector3 p1 = pe::Vector3(x1, h, z1);
        pe::Vector3 p2 = pe::Vector3(x2, h, z2);
        ts[0] = p1;
        ts[1] = p2;
        bool pit1 = pointInsideTriangle(vs, n, p1);
        bool pit2 = pointInsideTriangle(vs, n, p2);
        c = (pit1 * 1) | (pit2 * 2);
        return c > 0;
    }

    static bool intersectTriangleCylinderSide(const pe::Vector3 vs[3], const pe::Vector3& n, pe::Real r, pe::Real h,
                                              int& c, pe::Vector3 ts[4]) {
        c = 0;
        if (PE_APPROX_EQUAL(n.y, 0)) {
            return false;
        }
        pe::Real dxz = PE_SQRT(n.x * n.x + n.z * n.z);
        pe::Real h0 = n.dot(vs[0]) / n.y;
        pe::Real h1 = h0 - r * dxz / n.y;
        pe::Real h2 = pe::Real(2.0) * h0 - h1;
        pe::Vector3 ps[4];
        if (PE_APPROX_EQUAL(dxz, 0)) {
            ps[0] = {-r, h0, 0};
            ps[1] = {r, h0, 0};
            ps[2] = {0, h0, -r};
            ps[3] = {0, h0, r};
        } else {
            pe::Vector3 nor = pe::Vector3(n.x, 0, n.z) / dxz;
            ps[0] = nor * r; ps[0].y = h1;
            ps[1] = -nor * r; ps[1].y = h2;
            PE_SWAP(nor.x, nor.z);
            nor.x = -nor.x;
            ps[2] = nor * r; ps[2].y = h0;
            ps[3] = -nor * r; ps[3].y = h0;
        }
        for (const auto& p : ps) {
            if (PE_ABS(p.y) < h && pointInsideTriangle(vs, n, p)) {
                ts[c++] = p;
            }
        }
        return c > 0;
    }

    void CylinderConvexCollisionAlgorithm::intersectTriangleCylinder(const pe::Vector3 vs[3], const pe::Vector3& n, pe::Real h, pe::Real r,
                                                                     const pe::Transform &trans, pe::Real margin, ContactResult &result) {
        int c_top, c_bottom;
        pe::Vector3 p_tops[2], p_bottoms[2];
        bool i_top = intersectTriangleCircle(vs, n, r, h, c_top, p_tops);
        bool i_bottom = intersectTriangleCircle(vs, n, r, -h, c_bottom, p_bottoms);

        if (i_top && i_bottom) {
            pe::Vector3 p_top = (p_tops[0] + p_tops[1]) / 2;
            pe::Real d_top = r - (p_top - pe::Vector3(0, h, 0)).norm();
            pe::Vector3 n_top = trans.getBasis() * (p_top - pe::Vector3(0, h, 0)).normalized();
            if (c_top == 3 || c_top == 1) {
                p_top = trans * p_tops[0];
                result.addContactPoint(n_top, p_top - n_top * margin, -d_top + margin * 2);
                std::cout << "add triangle 2-1.1: " << n_top << p_top << d_top << std::endl;
            }
            if (c_top == 3 || c_top == 2) {
                p_top = trans * p_tops[1];
                result.addContactPoint(n_top, p_top - n_top * margin, -d_top + margin * 2);
                std::cout << "add triangle 2-1.2: " << n_top << p_top << d_top << std::endl;
            }

            pe::Vector3 p_bottom = (p_bottoms[0] + p_bottoms[1]) / 2;
            pe::Real d_bottom = r - (p_bottom - pe::Vector3(0, -h, 0)).norm();
            pe::Vector3 n_bottom = trans.getBasis() * (p_bottom - pe::Vector3(0, -h, 0)).normalized();
            if (c_bottom == 3 || c_bottom == 1) {
                p_bottom = trans * p_bottoms[0];
                result.addContactPoint(n_bottom, p_bottom - n_bottom * margin, -d_bottom + margin * 2);
                std::cout << "add triangle 2-2.1: " << n_bottom << p_bottom << d_bottom << std::endl;
            }
            if (c_bottom == 3 || c_bottom == 2) {
                p_bottom = trans * p_bottoms[1];
                result.addContactPoint(n_bottom, p_bottom - n_bottom * margin, -d_bottom + margin * 2);
                std::cout << "add triangle 2-2.2: " << n_bottom << p_bottom << d_bottom << std::endl;
            }
        }

        int c_side;
        pe::Vector3 p_sides[4];
        bool i_side = intersectTriangleCylinderSide(vs, n, r, h, c_side, p_sides);

        if (i_side && (i_top || i_bottom) && c_side == 1) {
            pe::Vector3 p_top = trans * pe::Vector3(p_sides[0].x, i_top ? h : -h, p_sides[0].z);
            pe::Real d = PE_ABS((i_top ? h : -h) - p_sides[0].y) * PE_ABS(n.y);
            pe::Vector3 n_w = trans.getBasis() * n;
            result.addContactPoint(-n_w, p_top + n_w * margin, -d + margin * 2);
            std::cout << "add triangle 1-1: " << -n_w << pe::Vector3(p_sides[0].x, i_top ? h : -h, p_sides[0].z) << d << std::endl;
        } else if (i_side && c_side >= 2) {
            for (int i = 0; i < c_side; i++) {
                pe::Vector3 p_top = trans * pe::Vector3(p_sides[i].x, p_sides[i].y > 0 ? h : -h, p_sides[i].z);
                pe::Real d = h - PE_ABS(p_sides[i].y);
                pe::Vector3 n_w = trans.getBasis() * pe::Vector3(0, pe::Real(p_sides[i].y > 0 ? 1.0 : -1.0), 0);
                result.addContactPoint(n_w, p_top - n_w * margin, -d + margin * 2);
                std::cout << "add triangle 1-2." + std::to_string(i) << ": " << n_w << p_top << d << std::endl;
            }
        }
    }

} // pe_phys_collision