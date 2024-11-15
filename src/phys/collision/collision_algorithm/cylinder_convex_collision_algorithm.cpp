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

#   if true
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

                bool point_inside[3]{false};
                for (int i = 0; i < 3; i++) {
                    point_inside[i] = pointInsideCylinder(vertices[i], cyl_h, cyl_r, trans_cyl, margin, result);
                    std::cout << "point " << i << ": " << point_inside[i] << std::endl;
                }

                for (int i = 0; i < 3; i++) {
                    int v1 = i;
                    int v2 = (i + 1) % 3;
                    if (!point_inside[v1] && !point_inside[v2]) {
                        bool f = intersectSegmentCylinder(vertices[v1], vertices[v2], cyl_h, cyl_r, trans_cyl, margin, result);
                        std::cout << "segment " << i << ": " << f << std::endl;
                    }
                }

                if (!point_inside[0] && !point_inside[1] && !point_inside[2]) {
                    intersectTriangleCylinder(vertices, cyl_h, cyl_r, trans_cyl, margin, result);
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
        return true;
    }

    static bool intersectSegmentCircle(const pe::Vector3 &v1, const pe::Vector3 &v2, pe::Real r, pe::Real h, pe::Vector3& t) {
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
        pe::Vector3 d_ST = (v2 - v1).normalized();
        pe::Real l_FT = PE_ABS(v1.x * d_ST.z - v1.z * d_ST.x);

        c = 0;
        if (l_FT > r) return false;

        pe::Real l_SF = PE_SQRT(v1.x * v1.x + v1.z * v1.z);
        pe::Real l0 = PE_SQRT(l_SF * l_SF - l_FT * l_FT);
        pe::Real right = r * r - l_FT * l_FT;
        pe::Real left = pe::Real(1.0) - d_ST.y * d_ST.y;
        pe::Real dl = PE_SQRT(right / left);
        pe::Real l = (v2 - v1).norm();

        if (l0 - dl > 0 && l0 - dl < l) {
            pe::Vector3 p1 = v1 + d_ST * (l0 - dl);
            if (PE_ABS(p1.y < h)) {
                t1 = p1;
                c++;
            }
        }
        if (l0 + dl > 0 && l0 + dl < l) {
            pe::Vector3 p2 = v1 + d_ST * (l0 + dl);
            if (PE_ABS(p2.y) < h) {
                (c == 1 ? t1 : t2) = p2;
                c++;
            }
        }
        return c > 0;
    }

    static bool pointInsideTriangle(const pe::Vector3 vs[3], const pe::Vector3& n, const pe::Vector3& p) {
        return (vs[0] - p).cross(vs[1] - p).dot(n) > 0 &&
               (vs[1] - p).cross(vs[2] - p).dot(n) > 0 &&
               (vs[2] - p).cross(vs[0] - p).dot(n) > 0;
    }

    static bool intersectTriangleCylinderSide(const pe::Vector3 vs[3], const pe::Vector3& n, pe::Real r, pe::Real h,
                                              int& c, pe::Vector3 ts[4]) {
        c = 0;
        if (PE_APPROX_EQUAL(n.y, 0)) {
            std::cout << "a" << std::endl;
            pe::Vector3 nor = pe::Vector3(n.x, 0, n.z).normalized();
            pe::Vector3 p1 = (vs[0] - pe::Vector3(0, h, 0)).dot(n) / nor.dot(n) * nor;
            pe::Vector3 p2 = (vs[0] - pe::Vector3(0, -h, 0)).dot(n) / nor.dot(n) * nor;
            p1.y = h;
            p2.y = -h;
            if (pointInsideTriangle(vs, n, p1)) {
                ts[c++] = p1;
            }
            if (pointInsideTriangle(vs, n, p2)) {
                ts[c++] = p2;
            }
        } else {
            pe::Real sign = pe::Vector3(vs[0].x, 0, vs[0].z).dot(n) > 0 ? pe::Real(1.0) : pe::Real(-1.0);
            pe::Real dxz = PE_SQRT(n.x * n.x + n.z * n.z);
            pe::Real tan = sign * dxz / n.y;
            pe::Real h1 = vs[0].y + PE_SQRT(vs[0].x * vs[0].x + vs[0].z * vs[0].z) * tan - r * tan;
            pe::Real h2 = h1 + r * tan * 2;
            std::cout << "dxz: " << dxz << " tan: " << tan << " h1: " << h1 << " h2: " << h2 << std::endl;
            if (PE_ABS(h1) < h && PE_ABS(h2) < h) {
                pe::Real avg_h = (h1 + h2) / 2;
                pe::Vector3 ps[4];
                if (PE_APPROX_EQUAL(dxz, 0)) {
                    ps[0] = {-r, avg_h, 0};
                    ps[1] = {r, avg_h, 0};
                    ps[2] = {0, avg_h, -r};
                    ps[3] = {0, avg_h, r};
                } else {
                    pe::Vector3 nor = pe::Vector3(n.x, 0, n.z).normalized();
                    ps[0] = nor * r; ps[0].y = h1;
                    ps[1] = -nor * r; ps[1].y = h2;
                    PE_SWAP(nor.x, nor.z);
                    nor.x = -nor.x;
                    ps[2] = nor * r; ps[2].y = avg_h;
                    ps[3] = -nor * r; ps[3].y = avg_h;
                }
                for (const auto& p : ps) {
                    if (pointInsideTriangle(vs, n, p)) {
                        ts[c++] = p;
                    }
                }
            }
        }
        return c > 0;
    }

    bool CylinderConvexCollisionAlgorithm::intersectSegmentCylinder(const pe::Vector3 &v1, const pe::Vector3 &v2, pe::Real h, pe::Real r,
                                                                    const pe::Transform& trans, pe::Real margin, ContactResult& result) {
        pe::Vector3 p_top, p_bottom;
        bool i_top, i_bottom;
        if (PE_APPROX_EQUAL(v1.y, v2.y)) {
            i_top = i_bottom = false;
        } else {
            i_top = intersectSegmentCircle(v1, v2, r, h, p_top);
            i_bottom = intersectSegmentCircle(v1, v2, r, -h, p_bottom);
        }
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
            return true;
        }

        pe::Vector3 p_side1, p_side2;
        int c;
        bool i_side;
        if (PE_APPROX_EQUAL(v1.x, v2.x) && PE_APPROX_EQUAL(v1.z, v2.z)) {
            i_side = false;
        } else {
            i_side = intersectSegmentCylinderSide(v1, v2, r, h, c, p_side1, p_side2);
        }
        if (!i_side && !i_top && !i_bottom) {
            return false;
        }

        if (i_side && (i_top || i_bottom) && c == 1) {
            pe::Vector3 v_top_or_bottom = i_top ? p_top : p_bottom;
            pe::Vector3 v_side = (p_side1 - pe::Vector3(0, p_side1.y, 0)).normalized() * r;
            pe::Vector3 t = (v_side + v_top_or_bottom) / 2;
            pe::Vector3 n = (v_side - v_top_or_bottom).cross(pe::Vector3::up().cross(v_side - v_top_or_bottom)).normalized();
            pe::Real d1 = (v_top_or_bottom - pe::Vector3(0, (i_top ? h : -h), 0)).norm();
            pe::Real d2 = h - PE_ABS(v_side.y);
            pe::Real d = d1 * d2 / PE_SQRT(d1 * d1 + d2 * d2);
            n = trans.getBasis() * n;
            t = trans * t;
            result.addContactPoint(n, t - n * margin, -d + margin * 2);
        } else if (i_side && c == 2) {
            pe::Vector3 p_side = (p_side1 + p_side2) / 2;
            pe::Real d1 = r - (p_side - pe::Vector3(0, p_side.y, 0)).norm();
            pe::Real d2 = h - PE_ABS(p_side.y);
            pe::Vector3 n = d1 < d2 ? (p_side - pe::Vector3(0, p_side.y, 0)).normalized() : pe::Vector3(0, p_side.y, 0).normalized();
            pe::Vector3 t = d1 < d2 ? p_side + d1 * n : p_side + d2 * n;
            pe::Real d = d1 < d2 ? d1 : d2;
            n = trans.getBasis() * n;
            result.addContactPoint(n, t - n * margin, -d + margin * 2);
        } else {
            PE_LOG_ERROR << "intersectSegmentCylinder: unexpected case" << PE_ENDL;
            return false;
        }

        return true;
    }

    bool CylinderConvexCollisionAlgorithm::intersectTriangleCylinder(const pe::Vector3 vs[3], pe::Real h, pe::Real r,
                                                                     const pe::Transform &trans, pe::Real margin, ContactResult &result) {
        int c;
        pe::Vector3 ts[4];
        pe::Vector3 n_tri = (vs[1] - vs[0]).cross(vs[2] - vs[0]).normalized();
        bool i_side = intersectTriangleCylinderSide(vs, n_tri, r, h, c, ts);
        if (!i_side) {
            return false;
        }

        if (c == 1) {
            std::cout << 1 << std::endl;
            pe::Vector3 p_top = pe::Vector3(ts[0].x, ts[0].y > 0 ? h : -h, ts[0].z);
            pe::Real dist = (p_top - ts[0]).dot(-n_tri);
            p_top = trans * p_top;
            n_tri = trans.getBasis() * n_tri;
            result.addContactPoint(n_tri, p_top - n_tri * margin, -dist + margin * 2);
        } else if (c == 2) {
            std::cout << 2 << std::endl;
            pe::Vector3 n = pe::Vector3(ts[0].x, 0, ts[0].z).normalized();
            pe::Vector3 p_side1 = n * r + pe::Vector3(0, ts[0].y > 0 ? h : -h, 0);
            pe::Vector3 p_side2 = n * r + pe::Vector3(0, ts[1].y > 0 ? h : -h, 0);
            pe::Real dist1 = (p_side1 - ts[0]).norm();
            pe::Real dist2 = (p_side2 - ts[1]).norm();
            n = trans.getBasis() * n;
            p_side1 = trans * p_side1;
            p_side2 = trans * p_side2;
            result.addContactPoint(n, p_side1 - n * margin, -dist1 + margin * 2);
            result.addContactPoint(n, p_side2 - n * margin, -dist2 + margin * 2);
        } else {
            std::cout << c << std::endl;
            pe::Vector3 n = trans.getBasis() * (ts[0].y > 0 ? pe::Vector3::up() : -pe::Vector3::up());
            pe::Real dist1 = h - PE_ABS(ts[0].y);
            pe::Real dist2 = h - PE_ABS(ts[1].y);
            pe::Real dist3 = h - PE_ABS(ts[2].y);
            pe::Vector3 p1 = trans * pe::Vector3(ts[0].x, ts[0].y > 0 ? h : -h, ts[0].z);
            pe::Vector3 p2 = trans * pe::Vector3(ts[1].x, ts[1].y > 0 ? h : -h, ts[1].z);
            pe::Vector3 p3 = trans * pe::Vector3(ts[2].x, ts[2].y > 0 ? h : -h, ts[2].z);
            result.addContactPoint(n, p1 - n * margin, -dist1 + margin * 2);
            result.addContactPoint(n, p2 - n * margin, -dist2 + margin * 2);
            result.addContactPoint(n, p3 - n * margin, -dist3 + margin * 2);
            if (c == 4) {
                pe::Real dist4 = h - PE_ABS(ts[3].y);
                pe::Vector3 p4 = trans * pe::Vector3(ts[3].x, ts[3].y > 0 ? h : -h, ts[3].z);
                result.addContactPoint(n, p4 - n * margin, -dist4 + margin * 2);
            }
        }
    }

} // pe_phys_collision