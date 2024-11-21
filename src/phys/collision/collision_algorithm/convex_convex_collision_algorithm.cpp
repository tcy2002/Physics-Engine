#include "convex_convex_collision_algorithm.h"
#include "phys/shape/convex_mesh_shape.h"

// sat convex collision (bullet)
// style-checked.
namespace pe_phys_collision {

    bool ConvexConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                          pe::Transform trans_a, pe::Transform trans_b,
                                                          pe::Real refScale, ContactResult &result) {
        if (shape_a->getType() != pe_phys_shape::ShapeType::ConvexMesh ||
            shape_b->getType() != pe_phys_shape::ShapeType::ConvexMesh) {
            return false;
        }

        const auto shape_mesh_a = dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a);
        const auto shape_mesh_b = dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b);
        auto& mesh_a = shape_mesh_a->getMesh();
        auto& mesh_b = shape_mesh_b->getMesh();
        auto& edges_a = shape_mesh_a->getUniqueEdges();
        auto& edges_b = shape_mesh_b->getUniqueEdges();

        constexpr auto margin = PE_MARGIN;

        return getClosestPoints(shape_a, shape_b, mesh_a, mesh_b, edges_a, edges_b,
                               trans_a, trans_b, margin, refScale, result);
    }

    static void clipFace(const VertexArray &p_in, VertexArray &p_out,
                                                  const pe::Vector3 &plane_normal_w, pe::Real plane_eq_w) {
        const int num_vertices = (int)p_in.size();
        if (num_vertices < 2) return;

        pe::Vector3 firstVertex = p_in[p_in.size() - 1];

        pe::Real ds = plane_normal_w.dot(firstVertex) + plane_eq_w;

        for (int ve = 0; ve < num_vertices; ve++) {
            pe::Vector3 endVertex = p_in[ve];

            const pe::Real de = plane_normal_w.dot(endVertex) + plane_eq_w;

            if (ds < 0) {
                if (de < 0) {
                    // Start < 0, end < 0, so output endVertex
                    p_out.push_back(endVertex);
                } else {
                    // Start < 0, end >= 0, so output intersection
                    p_out.push_back(firstVertex.lerp(endVertex, R(ds * 1.f / (ds - de))));
                }
            } else {
                if (de < 0) {
                    // Start >= 0, end < 0 so output intersection and end
                    p_out.push_back(firstVertex.lerp(endVertex, R(ds * 1.f / (ds - de))));
                    p_out.push_back(endVertex);
                }
            }
            firstVertex = endVertex;
            ds = de;
        }
    }

    static void clipFaceAgainstHull(const pe::Vector3 &sep_normal,
                                    const pe::Mesh& mesh_a, const pe::Transform &trans_a,
                                    VertexArray &world_vertices_b1, VertexArray &world_vertices_b2,
                                    pe::Real min_dist, pe::Real max_dist,
                                    pe::Real margin, ContactResult &result) {
        world_vertices_b2.resize(0);
        VertexArray* p_in = &world_vertices_b1;
        VertexArray* p_out = &world_vertices_b2;
        p_out->reserve(p_in->size());

        int closest_face_a = -1;
        {
            auto dMin = PE_REAL_MAX;
            for (int face = 0; face < (int)mesh_a.faces.size(); face++) {
                const pe::Vector3 normal = mesh_a.faces[face].normal;
                const pe::Vector3 face_a_normal_w = trans_a.getBasis() * normal;

                const pe::Real d = face_a_normal_w.dot(sep_normal);
                if (d < dMin) {
                    dMin = d;
                    closest_face_a = face;
                }
            }
        }
        if (closest_face_a < 0) {
            return;
        }

        const auto& poly_a = mesh_a.faces[closest_face_a];

        // clip polygon to back of planes of all faces of hull A that are adjacent to witness face
        const int num_vertices_a = (int)poly_a.indices.size();
        const pe::Vector3 world_plane_a_normal1 = trans_a.getBasis() * poly_a.normal;
        for (int e0 = 0; e0 < num_vertices_a; e0++) {
            pe::Vector3 a = mesh_a.vertices[poly_a.indices[e0]].position;
            pe::Vector3 b = mesh_a.vertices[poly_a.indices[(e0 + 1) % num_vertices_a]].position;
            const pe::Vector3 edge0 = a - b;
            const pe::Vector3 edge0_w = trans_a.getBasis() * edge0;

            pe::Vector3 plane_normal_w = -edge0_w.cross(world_plane_a_normal1);  //.cross(WorldEdge0);
            pe::Vector3 world_a = trans_a * a;
            const pe::Real plane_eq_w = -world_a.dot(plane_normal_w);

            //clip face
            clipFace(*p_in, *p_out, plane_normal_w, plane_eq_w);
            std::swap(p_in, p_out);
            p_out->resize(0);
        }

        // only keep points that are behind the witness face
        {
            const pe::Vector3 plane_v = mesh_a.vertices[poly_a.indices[0]].position;
            const pe::Vector3 local_plane_normal = poly_a.normal;

            for (int i = 0; i < (int)p_in->size(); i++) {
                pe::Vector3 vtx = p_in->at(i);
                const pe::Real depth = local_plane_normal.dot(trans_a.inverseTransform(vtx) - plane_v);
                // if (depth <= minDist) {
                //     depth = minDist;
                // }

                if (depth <= max_dist && depth >= min_dist) {
                    pe::Vector3 point = p_in->at(i);
                    result.addContactPoint(sep_normal, point - sep_normal * margin,
                                           depth + margin * 2);
                }
            }
        }
    }

    static void clipHullAgainstHull(const pe::Vector3 &sep_normal1,
                                                             const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                                             const pe::Transform &trans_a, const pe::Transform &trans_b,
                                                             const pe::Real min_dist, const pe::Real max_dist,
                                                             VertexArray &world_vertices_b1, VertexArray &world_vertices_b2,
                                                             const pe::Real margin, ContactResult &result) {
        int closest_face_b = -1;
        {
            auto d_max = PE_REAL_MIN;
            for (int face = 0; face < (int)mesh_b.faces.size(); face++) {
                const pe::Vector3 normal = mesh_b.faces[face].normal;
                const pe::Vector3 world_normal = trans_b.getBasis() * normal;
                const pe::Real d = world_normal.dot(sep_normal1);
                if (d > d_max) {
                    d_max = d;
                    closest_face_b = face;
                }
            }
        }
        if (closest_face_b < 0) {
            return;
        }

        world_vertices_b1.resize(0);
        {
            const auto& poly_b = mesh_b.faces[closest_face_b];
            const int numVertices = (int)poly_b.indices.size();
            for (int e0 = 0; e0 < numVertices; e0++) {
                pe::Vector3 b = mesh_b.vertices[poly_b.indices[e0]].position;
                world_vertices_b1.push_back(trans_b * b);
            }
        }

        if (closest_face_b >= 0) {
            clipFaceAgainstHull(sep_normal1, mesh_a, trans_a, world_vertices_b1, world_vertices_b2,
                                min_dist, max_dist, margin, result);
        }
    }

    static void segmentsClosestPoints(pe::Vector3& pts_vector, pe::Vector3& offset_a, pe::Vector3& offset_b,
                                      pe::Real& t_a, pe::Real& t_b, const pe::Vector3& translation,
                                      const pe::Vector3& dir_a, pe::Real h_len_a,
                                      const pe::Vector3& dir_b, pe::Real h_len_b) {
        // compute the parameters of the closest points on each line segment

        const pe::Real dir_a_dot_dir_b = dir_a.dot(dir_b);
        const pe::Real dir_a_dot_trans = dir_a.dot(translation);
        const pe::Real dir_b_dot_trans = dir_b.dot(translation);

        const pe::Real d = R(1.0) - dir_a_dot_dir_b * dir_a_dot_dir_b;

        if (d == 0) {
            t_a = 0;
        } else {
            t_a = (dir_a_dot_trans - dir_b_dot_trans * dir_a_dot_dir_b) / d;
            if (t_a < -h_len_a) t_a = -h_len_a;
            else if (t_a > h_len_a) t_a = h_len_a;
        }

        t_b = t_a * dir_a_dot_dir_b - dir_b_dot_trans;

        if (t_b < -h_len_b) {
            t_b = -h_len_b;
            t_a = t_b * dir_a_dot_dir_b + dir_a_dot_trans;

            if (t_a < -h_len_a) t_a = -h_len_a;
            else if (t_a > h_len_a) t_a = h_len_a;
        } else if (t_b > h_len_b) {
            t_b = h_len_b;
            t_a = t_b * dir_a_dot_dir_b + dir_a_dot_trans;

            if (t_a < -h_len_a) t_a = -h_len_a;
            else if (t_a > h_len_a) t_a = h_len_a;
        }

        // compute the closest points relative to segment centers.

        offset_a = dir_a * t_a;
        offset_b = dir_b * t_b;

        pts_vector = translation - offset_a + offset_b;
    }

    static bool testSepAxis(const pe_phys_shape::Shape* object_a, const pe_phys_shape::Shape* object_b,
                            const pe::Transform& trans_a, const pe::Transform& trans_b,
                            const pe::Vector3& sep_axis, pe::Real& depth,
                            pe::Vector3& witness_point_a, pe::Vector3& witness_point_b) {
        pe::Real min0, max0;
        pe::Real min1, max1;
        pe::Vector3 witness_pt_min_a, witness_pt_max_a;
        pe::Vector3 witness_pt_min_b, witness_pt_max_b;

        object_a->project(trans_a, sep_axis, min0, max0, witness_pt_min_a, witness_pt_max_a);
        object_b->project(trans_b, sep_axis, min1, max1, witness_pt_min_b, witness_pt_max_b);

        if (max0 < min1 || max1 < min0) return false;

        const pe::Real d0 = max0 - min1;
        const pe::Real d1 = max1 - min0;
        if (d0 < d1) {
            depth = d0;
            witness_point_a = witness_pt_max_a;
            witness_point_b = witness_pt_min_b;
        } else {
            depth = d1;
            witness_point_a = witness_pt_min_a;
            witness_point_b = witness_pt_max_b;
        }

        return true;
    }

    static bool isOnLine(const pe::Vector3& v0, const pe::Vector3& v1, const pe::Vector3& v2) {
        const pe::Vector3 edge = v1 - v0;
        const pe::Vector3 normal = edge.cross(v2 - v0);
        return normal.norm2() < PE_EPS;
    }

    static bool findSeparatingAxis(const pe_phys_shape::Shape* shape_a, const pe_phys_shape::Shape* shape_b,
                                   const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                   const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_a,
                                   const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& unique_edges_b,
                                   const pe::Transform &trans_a, const pe::Transform &trans_b,
                                   pe::Vector3 &sep, pe::Real margin, ContactResult &result) {
        const pe::Vector3 c0 = trans_a.getOrigin();
        const pe::Vector3 c1 = trans_b.getOrigin();
        const pe::Vector3 delta_c2 = c0 - c1;

        auto d_min = PE_REAL_MAX;
        pe::Vector3 d_min_pt_on_b;
        bool pt_on_a = false;

        // Test normals from hullA
        for (const auto & face : mesh_a.faces) {
            const pe::Vector3 normal = face.normal;
            pe::Vector3 face_a_normal_w = trans_a.getBasis() * normal;
            if (delta_c2.dot(face_a_normal_w) < 0)
                face_a_normal_w *= R(-1.0);

            pe::Real d;
            pe::Vector3 w_a, w_b;
            if (!testSepAxis(shape_a, shape_b, trans_a, trans_b,
                             face_a_normal_w, d, w_a, w_b)) {
                return false;
            }

            if (d < d_min) {
                d_min = d;
                d_min_pt_on_b = w_b;
                pt_on_a = false;
                sep = face_a_normal_w;
            }
        }

        // Test normals from hullB
        for (const auto & face : mesh_b.faces) {
            const pe::Vector3 normal = face.normal;
            pe::Vector3 world_normal = trans_b.getBasis() * normal;
            if (delta_c2.dot(world_normal) < 0) {
                world_normal *= R(-1.0);
            }

            pe::Real d;
            pe::Vector3 wA, wB;
            if (!testSepAxis(shape_a, shape_b, trans_a, trans_b,
                             world_normal, d, wA, wB)) {
                return false;
            }

            if (d < d_min) {
                d_min = d;
                d_min_pt_on_b = wA + world_normal * d;
                pt_on_a = true;
                sep = world_normal;
            }
        }

        // To prevent one corner case: the actual deepest penetration point is not on the witness face
        // if ((ptOnA && shapeB->localIsInside(transB.inverseTransform(dMinPtOnB))) ||
        //     (!ptOnA && shapeA->localIsInside(transA.inverseTransform(dMinPtOnB)))) {
        //     result.addContactPoint(sep, dMinPtOnB - sep * margin,
        //                            -dMin + margin * 2);
        // }

        int edge_a = -1;
        int edge_b = -1;
        pe::Vector3 world_edge_a;
        pe::Vector3 world_edge_b;
        pe::Vector3 witness_point_a(0, 0, 0), witness_point_b(0, 0, 0);

        // Test edges
        for (int e0 = 0; e0 < (int)unique_edges_a.size(); e0++) {
            const pe::Vector3 edge0 = (unique_edges_a[e0].first - unique_edges_a[e0].second).normalized();
            const pe::Vector3 world_edge0 = trans_a.getBasis() * edge0;
            for (int e1 = 0; e1 < (int)unique_edges_b.size(); e1++) {
                const pe::Vector3 edge1 = (unique_edges_b[e1].first - unique_edges_b[e1].second).normalized();
                const pe::Vector3 world_edge1 = trans_b.getBasis() * edge1;

                pe::Vector3 cross = world_edge0.cross(world_edge1);
                if (!PE_APPROX_EQUAL(cross.norm(), 0)) {
                    cross.normalize();
                    if (delta_c2.dot(cross) < 0) {
                        cross *= R(-1.0);
                    }

                    pe::Real dist;
                    pe::Vector3 w_a, w_b;
                    if (!testSepAxis(shape_a, shape_b, trans_a, trans_b,
                                     cross, dist, w_a, w_b)) {
                        return false;
                    }

                    if (dist < d_min) {
                        if (!isOnLine(unique_edges_a[e0].first, unique_edges_a[e0].second, trans_a.inverseTransform(w_a)) ||
                            !isOnLine(unique_edges_b[e1].first, unique_edges_b[e1].second, trans_b.inverseTransform(w_b))) {
                            continue;
                        }
                        d_min = dist;
                        sep = cross;
                        edge_a = e0;
                        edge_b = e1;
                        world_edge_a = world_edge0;
                        world_edge_b = world_edge1;
                        witness_point_a = w_a;
                        witness_point_b = w_b;
                    }
                }
            }
        }

        if (edge_a >= 0 && edge_b >= 0) {
            //add an edge-edge contact
            pe::Vector3 pts_vector;
            pe::Vector3 offset_a;
            pe::Vector3 offset_b;
            pe::Real t_a;
            pe::Real t_b;

            pe::Vector3 translation = witness_point_b - witness_point_a;

            pe::Vector3 dir_a = world_edge_a;
            pe::Vector3 dir_b = world_edge_b;

            auto h_len_b = PE_REAL_MAX;
            auto h_len_a = PE_REAL_MAX;

            segmentsClosestPoints(pts_vector, offset_a, offset_b, t_a, t_b,
                                  translation, dir_a, h_len_a, dir_b, h_len_b);

            pe::Real nl_sqrt = pts_vector.norm2();
            if (nl_sqrt > PE_EPS) {
                pe::Real nl = std::sqrt(nl_sqrt);
                pts_vector *= R(1.0) / nl;
                if (pts_vector.dot(delta_c2) < 0) {
                    pts_vector *= R(-1.0);
                }
                pe::Vector3 ptOnB = witness_point_b + offset_b;
                pe::Real distance = nl;

                result.addContactPoint(pts_vector, ptOnB - pts_vector * margin,
                                       -distance + margin * 2);
            }
        }

        if (delta_c2.dot(sep) < 0) {
            sep = -sep;
        }

        return true;
    }

    bool ConvexConvexCollisionAlgorithm::getClosestPoints(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                          const pe::Mesh &mesh_a, const pe::Mesh &mesh_b,
                                                          const pe::Array<pe::KV<pe::Vector3, pe::Vector3>> &unique_edges_a,
                                                          const pe::Array<pe::KV<pe::Vector3, pe::Vector3>> &unique_edges_b,
                                                          const pe::Transform &trans_a, const pe::Transform &trans_b,
                                                          pe::Real margin, pe::Real refScale, ContactResult& result) {
        pe::Vector3 sep;

        VertexArray world_vertices_b1;
        VertexArray world_vertices_b2;

        if (!findSeparatingAxis(shape_a, shape_b, mesh_a, mesh_b,
                                unique_edges_a, unique_edges_b,
                                trans_a, trans_b, sep, margin, result)) {
            return false;
        }
        clipHullAgainstHull(sep, mesh_a, mesh_b, trans_a, trans_b,
                            -refScale, margin, world_vertices_b1, world_vertices_b2,
                            margin, result);
        return true;
    }

} // pe_phys_collision