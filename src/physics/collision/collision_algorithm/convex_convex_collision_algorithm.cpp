#include "convex_convex_collision_algorithm.h"
#include "physics/shape/convex_mesh_shape.h"

namespace pe_physics_collision {

#define PE_CONVEX_CONVEX_USE_METHOD1 false

bool ConvexConvexCollisionAlgorithm::processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                                                      pe::Transform trans_a, pe::Transform trans_b,
                                                      pe::Real refScale, ContactResult &result) {
    if (shape_a->getType() != pe_physics_shape::ShapeType::ST_ConvexMesh ||
        shape_b->getType() != pe_physics_shape::ShapeType::ST_ConvexMesh) {
        return false;
    }

    const auto shape_mesh_a = dynamic_cast<pe_physics_shape::ConvexMeshShape *>(shape_a);
    const auto shape_mesh_b = dynamic_cast<pe_physics_shape::ConvexMeshShape *>(shape_b);
    auto& mesh_a = shape_mesh_a->getMesh();
    auto& mesh_b = shape_mesh_b->getMesh();
    auto& edges_a = shape_mesh_a->getUniqueEdges();
    auto& edges_b = shape_mesh_b->getUniqueEdges();

    constexpr auto margin = PE_MARGIN;

    return getClosestPoints(shape_a, shape_b, mesh_a, mesh_b, edges_a, edges_b,
                           trans_a, trans_b, margin, refScale, result);
}

/***********************************************************
 * Convex-Convex SAT Method 1:
 * Find the separating axis using face normals and edge cross products
 * Then use both to generate polygon-polygon contacts
 *
 * The performance is not as good as Method 2 in most cases
 * So it is disabled by default
 ***********************************************************/

static COMMON_FORCE_INLINE void clipFace(const VertexArray &p_in, VertexArray &p_out,
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
                p_out.push_back(firstVertex.lerp(endVertex, PE_R(ds * 1.f / (ds - de))));
            }
        } else {
            if (de < 0) {
                // Start >= 0, end < 0 so output intersection and end
                p_out.push_back(firstVertex.lerp(endVertex, PE_R(ds * 1.f / (ds - de))));
                p_out.push_back(endVertex);
            }
        }
        firstVertex = endVertex;
        ds = de;
    }
}

static COMMON_FORCE_INLINE void clipFaceAgainstHull(const pe::Vector3 &sep_normal,
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

            if (depth <= max_dist && depth >= min_dist) {
                pe::Vector3 point = p_in->at(i);
                result.addContactPoint(sep_normal, point - sep_normal * margin,
                                       depth + margin);
            }
        }
    }
}

static COMMON_FORCE_INLINE void clipHullAgainstHull(const pe::Vector3 &sep_normal1,
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

static COMMON_FORCE_INLINE void segmentsClosestPoints(pe::Vector3& pts_vector, pe::Vector3& offset_a, pe::Vector3& offset_b,
                                  pe::Real& t_a, pe::Real& t_b, const pe::Vector3& translation,
                                  const pe::Vector3& dir_a, pe::Real h_len_a,
                                  const pe::Vector3& dir_b, pe::Real h_len_b) {
    // compute the parameters of the closest points on each line segment

    const pe::Real dir_a_dot_dir_b = dir_a.dot(dir_b);
    const pe::Real dir_a_dot_trans = dir_a.dot(translation);
    const pe::Real dir_b_dot_trans = dir_b.dot(translation);

    const pe::Real d = PE_R(1.0) - dir_a_dot_dir_b * dir_a_dot_dir_b;

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

static COMMON_FORCE_INLINE bool testSepAxis(const pe_physics_shape::Shape* object_a, const pe_physics_shape::Shape* object_b,
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

static COMMON_FORCE_INLINE bool isOnLine(const pe::Vector3& v0, const pe::Vector3& v1, const pe::Vector3& v2) {
    const pe::Vector3 edge = v1 - v0;
    const pe::Vector3 normal = edge.cross(v2 - v0);
    return normal.norm2() < PE_EPS;
}

static COMMON_FORCE_INLINE bool findSeparatingAxis(const pe_physics_shape::Shape* shape_a, const pe_physics_shape::Shape* shape_b,
                               const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                               const pe::Array<pe_physics_shape::UniqueEdge>& unique_edges_a,
                               const pe::Array<pe_physics_shape::UniqueEdge>& unique_edges_b,
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
            face_a_normal_w *= PE_R(-1.0);

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
            world_normal *= PE_R(-1.0);
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

    // To prevent one corner case: the actual deepest penetration point is not on the witness face (Important)
    if ((pt_on_a && shape_b->localIsInside(trans_b.inverseTransform(d_min_pt_on_b))) ||
     (!pt_on_a && shape_a->localIsInside(trans_a.inverseTransform(d_min_pt_on_b)))) {
     result.addContactPoint(sep, d_min_pt_on_b - sep * margin,
                            -d_min + margin);
    }

    int edge_a = -1;
    int edge_b = -1;
    pe::Vector3 world_edge_a;
    pe::Vector3 world_edge_b;
    pe::Vector3 witness_point_a(0, 0, 0), witness_point_b(0, 0, 0);

    // Test edges
    for (int e0 = 0; e0 < (int)unique_edges_a.size(); e0++) {
        const pe::Vector3 edge0 = (unique_edges_a[e0].start - unique_edges_a[e0].end).normalized();
        const pe::Vector3 world_edge0 = trans_a.getBasis() * edge0;
        for (int e1 = 0; e1 < (int)unique_edges_b.size(); e1++) {
            const pe::Vector3 edge1 = (unique_edges_b[e1].start - unique_edges_b[e1].end).normalized();
            const pe::Vector3 world_edge1 = trans_b.getBasis() * edge1;

            pe::Vector3 cross = world_edge0.cross(world_edge1);
            if (!PE_APPROX_EQUAL(cross.norm(), 0)) {
                cross.normalize();
                if (delta_c2.dot(cross) < 0) {
                    cross *= PE_R(-1.0);
                }

                pe::Real dist;
                pe::Vector3 w_a, w_b;
                if (!testSepAxis(shape_a, shape_b, trans_a, trans_b,
                                 cross, dist, w_a, w_b)) {
                    return false;
                }

                if (dist < d_min) {
                    // Check if the witness points are actually on the edges (Important)
                    if (!isOnLine(unique_edges_a[e0].start, unique_edges_a[e0].end, trans_a.inverseTransform(w_a)) ||
                        !isOnLine(unique_edges_b[e1].start, unique_edges_b[e1].end, trans_b.inverseTransform(w_b))) {
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
            pts_vector *= PE_R(1.0) / nl;
            if (pts_vector.dot(delta_c2) < 0) {
                pts_vector *= PE_R(-1.0);
            }
            pe::Vector3 ptOnB = witness_point_b + offset_b;
            pe::Real distance = nl;

            result.addContactPoint(pts_vector, ptOnB - pts_vector * margin,
                                   -distance + margin);
        }
    }

    if (delta_c2.dot(sep) < 0) {
        sep = -sep;
    }

    return true;
}

/***********************************************************
 * Convex-Convex SAT Method 2:
 * Only use face normals to generate polygon-polygon contacts
 ***********************************************************/

static COMMON_FORCE_INLINE pe::Real testSingleFaceDirectionPolyhedronVsPolyhedron(const pe_physics_shape::Shape* shape_a, const pe_physics_shape::Shape* shape_b,
                                                       const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                                       const pe::Transform &trans_a2b,
                                                       uint32_t face_index) {
    const pe::Mesh::Face& face = mesh_a.faces[face_index];

    // Get the face normal
    const pe::Vector3& face_normal = face.normal;

    // Convert the face normal into the local-space of polyhedron 2
    const pe::Vector3 face_normal_polyhedron2space = trans_a2b.getBasis() * face_normal;

    // Get the support point of polyhedron 2 in the inverse direction of face normal
    const pe::Vector3 supportPoint = shape_b->getLocalSupportPoint(-face_normal_polyhedron2space);

    // Compute the penetration depth
    const pe::Vector3 faceVertex = trans_a2b * mesh_a.vertices[face.indices[0]].position;
    pe::Real penetrationDepth = (faceVertex - supportPoint).dot(face_normal_polyhedron2space);

    return penetrationDepth;
}

static COMMON_FORCE_INLINE pe::Real testFacesDirectionPolyhedronVsPolyhedron(const pe_physics_shape::Shape* shape_a, const pe_physics_shape::Shape* shape_b,
                                                  const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                                  const pe::Transform &trans_a2b,
                                                  uint32_t& min_face_index) {
    pe::Real min_penetration_depth = PE_REAL_MAX;

    // For each face of the first polyhedron
    for (uint32_t f = 0; f < mesh_a.faces.size(); f++) {

        pe::Real penetrationDepth = testSingleFaceDirectionPolyhedronVsPolyhedron(shape_a, shape_b, mesh_a, mesh_b, trans_a2b, f);

        // If the penetration depth is negative, we have found a separating axis
        if (penetrationDepth <= PE_R(0.0)) {
            min_face_index = f;
            return penetrationDepth;
        }

        // Check if we have found a new minimum penetration axis
        if (penetrationDepth < min_penetration_depth) {
            min_penetration_depth = penetrationDepth;
            min_face_index = f;
        }
    }

    return min_penetration_depth;
}

static COMMON_FORCE_INLINE bool testGaussMapArcsIntersect(const pe::Vector3& a, const pe::Vector3& b, const pe::Vector3& c, const pe::Vector3& d,
                                      const pe::Vector3& b_cross_a, const pe::Vector3& d_cross_c) {
    const pe::Real cba = c.dot(b_cross_a);
    const pe::Real dba = d.dot(b_cross_a);
    const pe::Real adc = a.dot(d_cross_c);
    const pe::Real bdc = b.dot(d_cross_c);

    return cba * dba < PE_R(0.0) && adc * bdc < PE_R(0.0) && cba * bdc > PE_R(0.0);
}

static COMMON_FORCE_INLINE bool testEdgesBuildMinkowskiFace(const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                        const pe_physics_shape::UniqueEdge& edge_a, const pe_physics_shape::UniqueEdge& edge_b,
                                        const pe::Transform& trans_a2b) {
    const pe::Vector3 a = trans_a2b.getBasis() * mesh_a.faces[edge_a.face1_id].normal;
    const pe::Vector3 b = trans_a2b.getBasis() * mesh_a.faces[edge_a.face2_id].normal;

    const pe::Vector3& c = mesh_b.faces[edge_b.face1_id].normal;
    const pe::Vector3& d = mesh_b.faces[edge_b.face2_id].normal;

    // Compute b.cross(a) using the edge direction
    const pe::Vector3 b_cross_a = trans_a2b.getBasis() * (edge_a.start - edge_a.end);

    // Compute d.cross(c) using the edge direction
    const pe::Vector3 d_cross_c = edge_b.start - edge_b.end;

    // Test if the two arcs of the Gauss Map intersect (therefore forming a minkowski face)
    // Note that we negate the normals of the second polyhedron because we are looking at the
    // Gauss map of the minkowski difference of the polyhedrons
    return testGaussMapArcsIntersect(a, b, -c, -d, b_cross_a, d_cross_c);
}

static COMMON_FORCE_INLINE bool areParallelVectors(const pe::Vector3& vector1, const pe::Vector3& vector2) {
    return vector1.cross(vector2).norm2() < PE_EPS;
}

pe::Real computeDistanceBetweenEdges(const pe::Vector3& edge1_a, const pe::Vector3& edge2_a,
                                     const pe::Vector3& polyhedron1centroid, const pe::Vector3& polyhedron2centroid,
                                     const pe::Vector3& edge1_direction, const pe::Vector3& edge2_direction,
                                     bool is_shape1triangle, pe::Vector3& out_separating_axis_polyhedron2space) {
    // If the two edges are parallel
    if (areParallelVectors(edge1_direction, edge2_direction)) {
        // Return a large penetration depth to skip those edges
        return PE_REAL_MAX;
    }

    // Compute the candidate separating axis (cross product between two polyhedrons edges)
    pe::Vector3 axis = edge1_direction.cross(edge2_direction).normalized();

    // Make sure the axis direction is going from first to second polyhedron
    pe::Real dot_prod;
    if (is_shape1triangle) {
        // The shape 1 is a triangle. It is safer to use a vector from
        // centroid to edge of the shape 2 because for a triangle, we
        // can have a vector that is orthogonal to the axis

        dot_prod = axis.dot(edge2_a - polyhedron2centroid);
    } else {
        // The shape 2 might be a triangle. It is safer to use a vector from
        // centroid to edge of the shape 2 because for a triangle, we
        // can have a vector that is orthogonal to the axis

        dot_prod = axis.dot(polyhedron1centroid - edge1_a);
    }
    if (dot_prod > PE_R(0.0)) {
        axis = -axis;
    }

    out_separating_axis_polyhedron2space = axis;

    // Compute and return the distance between the edges
    return -axis.dot(edge2_a - edge1_a);
}

// Compute the intersection between a plane and a segment
// Let the plane define by the equation planeNormal.dot(X) = planeD with X a point on the plane and "planeNormal" the plane normal. This method
// computes the intersection P between the plane and the segment (segA, segB). The method returns the value "t" such
// that P = segA + t * (segB - segA). Note that it only returns a value in [0, 1] if there is an intersection. Otherwise,
// there is no intersection between the plane and the segment.
static COMMON_FORCE_INLINE pe::Real computePlaneSegmentIntersection(const pe::Vector3& seg_a, const pe::Vector3& seg_b, const pe::Real plane_d, const pe::Vector3& plane_normal) {
    const pe::Real parallel_epsilon = PE_R(0.0001);
    pe::Real t = PE_R(-1);

    const pe::Real n_dot_ab = plane_normal.dot(seg_b - seg_a);

    // If the segment is not parallel to the plane
    if (PE_ABS(n_dot_ab) > parallel_epsilon) {
        t = (plane_d - plane_normal.dot(seg_a)) / n_dot_ab;
    }

    return t;
}

static COMMON_FORCE_INLINE void clipPolygonWithPlane(pe::Array<pe::Vector3>& polygon_vertices, const pe::Vector3& plane_point,
                                 const pe::Vector3& plane_normal, pe::Array<pe::Vector3>& out_clipped_polygon_vertices) {
    const uint32_t nb_input_vertices = (uint32_t)polygon_vertices.size();

    uint32_t v_start_index = nb_input_vertices - 1;

    const pe::Real plane_normal_dot_plane_point = plane_normal.dot(plane_point);

    pe::Real v_start_dot_n = (polygon_vertices[v_start_index] - plane_point).dot(plane_normal);

    // For each edge of the polygon
    for (int v_end_index = 0; v_end_index < nb_input_vertices; v_end_index++) {
        const pe::Vector3& v_start = polygon_vertices[v_start_index];
        const pe::Vector3& v_end = polygon_vertices[v_end_index];

        const pe::Real v_end_dot_n = (v_end - plane_point).dot(plane_normal);

        // If the second vertex is in front of the clippling plane
        if (v_end_dot_n >= PE_R(0.0)) {
            // If the first vertex is not in front of the clippling plane
            if (v_start_dot_n < PE_R(0.0)) {
                // The second point we keep is the intersection between the segment v1, v2 and the clipping plane
                const pe::Real t = computePlaneSegmentIntersection(v_start, v_end, plane_normal_dot_plane_point, plane_normal);

                if (t >= PE_R(0) && t <= PE_R(1.0)) {
                    out_clipped_polygon_vertices.push_back(v_start + t * (v_end - v_start));
                } else {
                    out_clipped_polygon_vertices.push_back(v_end);
                }
            }

            // Add the second vertex
            out_clipped_polygon_vertices.push_back(v_end);
        } else { // If the second vertex is behind the clipping plane
            // If the first vertex is in front of the clippling plane
            if (v_start_dot_n >= PE_R(0.0)) {
                // The first point we keep is the intersection between the segment v1, v2 and the clipping plane
                const pe::Real t = computePlaneSegmentIntersection(v_start, v_end, -plane_normal_dot_plane_point, -plane_normal);

                if (t >= PE_R(0.0) && t <= PE_R(1.0)) {
                    out_clipped_polygon_vertices.push_back(v_start + t * (v_end - v_start));
                } else {
                    out_clipped_polygon_vertices.push_back(v_start);
                }
            }
        }

        v_start_index = v_end_index;
        v_start_dot_n = v_end_dot_n;
    }
}

static COMMON_FORCE_INLINE pe::Vector3 projectPointOntoPlane(const pe::Vector3& point, const pe::Vector3& unit_plane_normal, const pe::Vector3& plane_point) {
    return point - unit_plane_normal.dot(point - plane_point) * unit_plane_normal;
}

static COMMON_FORCE_INLINE pe::Real getFaceSupportValue(const pe::Mesh& mesh, const pe::Mesh::Face& face, const pe::Vector3& dir) {
    pe::Real max_dot_product = PE_REAL_MIN;
    for (auto i : face.indices) {
        const auto& p = mesh.vertices[i].position;
        pe::Real dot_product = p.dot(dir);
        if (dot_product > max_dot_product) {
            max_dot_product = dot_product;
        }
    }
    return max_dot_product;
}

static COMMON_FORCE_INLINE bool computePolyhedronVsPolyhedronFaceContactPoints(bool is_min_penetration_face_normal_polyhedron1,
                                                    const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                                    const pe::Transform& trans_a, const pe::Transform& trans_b,
                                                    const pe::Transform& trans_a2b, const pe::Transform& trans_b2a,
                                                    uint32_t min_face_index, pe::Real margin, ContactResult &result) {
    const pe::Mesh* reference_polyhedron;
    const pe::Mesh* incident_polyhedron;
    const pe::Transform& reference_to_incident_transform = is_min_penetration_face_normal_polyhedron1 ? trans_a2b : trans_b2a;
    const pe::Transform& incident_to_reference_transform = is_min_penetration_face_normal_polyhedron1 ? trans_b2a : trans_a2b;

    if (is_min_penetration_face_normal_polyhedron1) {
        reference_polyhedron = &mesh_a;
        incident_polyhedron = &mesh_b;
    } else {
        reference_polyhedron = &mesh_b;
        incident_polyhedron = &mesh_a;
    }

    const pe::Vector3 axis_reference_space = reference_polyhedron->faces[min_face_index].normal;
    const pe::Vector3 axis_incident_space = reference_to_incident_transform.getBasis() * axis_reference_space;

    // Compute the world normal
    const pe::Vector3 normal_world = is_min_penetration_face_normal_polyhedron1 ? trans_a.getBasis() * axis_reference_space :
        -trans_b.getBasis() * axis_reference_space;

    // Get the reference face
    const pe::Mesh::Face& reference_face = reference_polyhedron->faces[min_face_index];

    // Find the incident face on the other polyhedron (most anti-parallel face)
    // Fixed: most anti-parallel is not enough, the support point must lie in the face
    int incident_face_index = -1;
    {
        pe::Real min_dot_product = PE_REAL_MAX;
        pe::Real max_support_value = PE_REAL_MIN;

        // For each face of the polyhedron
        const int nb_faces = (int)incident_polyhedron->faces.size();
        for (int i = 0; i < nb_faces; i++) {
            // Get the face normal
            const pe::Real support_value = getFaceSupportValue(*incident_polyhedron, incident_polyhedron->faces[i], -axis_incident_space);
            const pe::Real dot_product = incident_polyhedron->faces[i].normal.dot(axis_incident_space);
            if (support_value > max_support_value + PE_EPS) {
                max_support_value = support_value;
                min_dot_product = dot_product;
                incident_face_index = i;
            } else if (support_value >= max_support_value - PE_EPS && dot_product < min_dot_product) {
                min_dot_product = dot_product;
                incident_face_index = i;
            }
            /*if (dot_product < min_dot_product) {
                min_dot_product = dot_product;
                incident_face_index = i;
            }*/
        }
    }
    if (incident_face_index == -1) {
        exit(2);
    }

    // Get the incident face
    const pe::Mesh::Face& incident_face = incident_polyhedron->faces[incident_face_index];

    const uint32_t nb_incident_face_vertices = (uint32_t)incident_face.indices.size();
    const uint32_t nb_max_elements = nb_incident_face_vertices * 2 * (uint32_t)reference_face.indices.size();
    pe::Array<pe::Vector3> vertices_temp1;
    pe::Array<pe::Vector3> vertices_temp2;
    vertices_temp1.reserve(nb_max_elements);
    vertices_temp2.reserve(nb_max_elements);

    // Get all the vertices of the incident face (in the reference local-space)
    for (uint32_t i = 0; i < nb_incident_face_vertices; i++) {
        const pe::Vector3 face_vertex_incident_space = incident_polyhedron->vertices[incident_face.indices[i]].position;
        vertices_temp1.push_back(incident_to_reference_transform * face_vertex_incident_space);
    }

    // For each edge of the reference we use it to clip the incident face polygon using Sutherland-Hodgman algorithm
    uint32_t edge_v1_index = reference_face.indices.size() - 1;
    uint32_t edge_v2_index = 0;
    bool are_vertices1_input = false;
    uint32_t nb_output_vertices;

    pe::Vector3 edge_v1 = reference_polyhedron->vertices[reference_face.indices[edge_v1_index]].position;

    do {
        // Switch the input/output arrays of vertices
        are_vertices1_input = !are_vertices1_input;

        // Get the edge vertices and edge direction
        const pe::Vector3& edge_v2 = reference_polyhedron->vertices[reference_face.indices[edge_v2_index]].position;
        const pe::Vector3 edge_direction = edge_v2 - edge_v1;

        // Compute the normal of the clipping plane for this edge
        // The clipping plane is perpendicular to the edge direction and the reference face normal
        const pe::Vector3 plane_normal = axis_reference_space.cross(edge_direction);

        // Clip the incident face with one adjacent plane (corresponding to one edge) of the reference face
        clipPolygonWithPlane(are_vertices1_input ? vertices_temp1 : vertices_temp2, edge_v1, plane_normal, are_vertices1_input ? vertices_temp2 : vertices_temp1);

        edge_v1 = edge_v2;
        edge_v2_index = (edge_v2_index + 1) % reference_face.indices.size();

        // Clear the input array of vertices before the next loop
        if (are_vertices1_input) {
            vertices_temp1.clear();
            nb_output_vertices = (uint32_t)vertices_temp2.size();
        } else {
            vertices_temp2.clear();
            nb_output_vertices = (uint32_t)vertices_temp1.size();
        }
    } while (edge_v2_index != 0 && nb_output_vertices > 0);

    // Reference to the output clipped polygon vertices
    pe::Array<pe::Vector3>& clipped_polygon_vertices = are_vertices1_input ? vertices_temp2 : vertices_temp1;

    // We only keep the clipped points that are below the reference face
    const pe::Vector3& reference_face_vertex = reference_polyhedron->vertices[reference_face.indices[edge_v1_index]].position;
    bool contact_points_found = false;
    const uint32_t nb_clip_polygon_vertices = clipped_polygon_vertices.size();
    for (uint32_t i = 0; i < nb_clip_polygon_vertices; i++) {
        // Compute the penetration depth of this contact point (can be different from the minPenetration depth which is
        // the maximal penetration depth of any contact point for this separating axis
        pe::Real penetration_depth = (reference_face_vertex - clipped_polygon_vertices[i]).dot(axis_reference_space);

        // If the clip point is below the reference face
        if (penetration_depth > PE_R(0.0)) {
            contact_points_found = true;

            // If we need to report contacts
            {
                pe::Vector3 out_world_normal = normal_world;

                // Convert the clip incident polyhedron vertex into the incident polyhedron local-space
                pe::Vector3 contact_point_incident_polyhedron = reference_to_incident_transform * clipped_polygon_vertices[i];

                // Project the contact point onto the reference face
                pe::Vector3 contact_point_reference_polyhedron = projectPointOntoPlane(clipped_polygon_vertices[i], axis_reference_space, reference_face_vertex);

                // Create a new contact point
                result.addContactPoint(-out_world_normal, // Always points from B to A
                    trans_b * (is_min_penetration_face_normal_polyhedron1 ? // Always use point on B
                        contact_point_incident_polyhedron :
                        contact_point_reference_polyhedron)
                    + out_world_normal * margin,
                    -penetration_depth + margin);
            }
        }
    }

    return contact_points_found;
}

// Compute the closest points between two segments
// This method uses the technique described in the book Real-Time
// collision detection by Christer Ericson.
static COMMON_FORCE_INLINE void computeClosestPointBetweenTwoSegments(const pe::Vector3& seg1_point_a, const pe::Vector3& seg1_point_b,
                                                                      const pe::Vector3& seg2_point_a, const pe::Vector3& seg2_point_b,
                                                                      pe::Vector3& closest_point_seg1, pe::Vector3& closest_point_seg2) {
    const pe::Vector3 d1 = seg1_point_b - seg1_point_a;
    const pe::Vector3 d2 = seg2_point_b - seg2_point_a;
    const pe::Vector3 r = seg1_point_a - seg2_point_a;
    const pe::Real a = d1.norm2();
    const pe::Real e = d2.norm2();
    const pe::Real f = d2.dot(r);
    pe::Real s, t;

    // If both segments degenerate into points
    if (a <= PE_EPS && e <= PE_EPS) {
        closest_point_seg1 = seg1_point_a;
        closest_point_seg2 = seg2_point_a;
        return;
    }
    // If first segment degenerates into a point
    if (a <= PE_EPS) {
        s = PE_R(0.0);

        // Compute the closest point on second segment
        t = PE_CLAMP(f / e, PE_R(0.0), PE_R(1.0));
    } else {
        const pe::Real c = d1.dot(r);

        // If the second segment degenerates into a point
        if (e <= PE_EPS) {
            t = PE_R(0.0);
            s = PE_CLAMP(-c / a, PE_R(0.0), PE_R(1.0));
        } else {
            const pe::Real b = d1.dot(d2);
            const pe::Real denom = a * e - b * b;

            // If the segments are not parallel
            if (denom != PE_R(0.0)) {
                // Compute the closest point on line 1 to line 2 and
                // clamp to first segment.
                s = PE_CLAMP((b * f - c * e) / denom, PE_R(0.0), PE_R(1.0));
            } else {
                // Pick an arbitrary point on first segment
                s = PE_R(0.0);
            }

            // Compute the point on line 2 closest to the closest point
            // we have just found
            t = (b * s + f) / e;

            // If this closest point is inside second segment (t in [0, 1]), we are done.
            // Otherwise, we clamp the point to the second segment and compute again the
            // closest point on segment 1
            if (t < PE_R(0.0)) {
                t = PE_R(0.0);
                s = PE_CLAMP(-c / a, PE_R(0.0), PE_R(1.0));
            } else if (t > PE_R(1.0)) {
                t = PE_R(1.0);
                s = PE_CLAMP((b - c) / a, PE_R(0.0), PE_R(1.0));
            }
        }
    }

    // Compute the closest points on both segments
    closest_point_seg1 = seg1_point_a + d1 * s;
    closest_point_seg2 = seg2_point_a + d2 * t;
}

const pe::Real SEPARATING_AXIS_RELATIVE_TOLERANCE = PE_R(1.002);
const pe::Real SEPARATING_AXIS_ABSOLUTE_TOLERANCE = PE_R(0.0005);

static COMMON_FORCE_INLINE bool findSeparatingAxis2(const pe_physics_shape::Shape* shape_a, const pe_physics_shape::Shape* shape_b,
                                const pe::Mesh& mesh_a, const pe::Mesh& mesh_b,
                                const pe::Array<pe_physics_shape::UniqueEdge>& unique_edges_a,
                                const pe::Array<pe_physics_shape::UniqueEdge>& unique_edges_b,
                                const pe::Transform &trans_a, const pe::Transform &trans_b,
                                pe::Real margin, ContactResult &result) {
    pe::Real min_penetration_depth = PE_REAL_MAX;
    uint32_t min_face_index = 0;
    bool is_min_penetration_face_normal = false;
    bool is_min_penetration_face_normal_polyhedron1 = false;
    pe::Vector3 separating_edge1_a, separating_edge1_b;
    pe::Vector3 separating_edge2_a, separating_edge2_b;
    pe::Vector3 min_edge_vs_edge_separating_axis_polyhedron2space;
    const pe::Transform polyhedron1_to_polyhedron2 = trans_b.inverse() * trans_a;
    const pe::Transform polyhedron2_to_polyhedron1 = polyhedron1_to_polyhedron2.inverse();

    // Test all the face normals of the polyhedron 1 for separating axis
    uint32_t face_index1;
    pe::Real penetration_depth1 = testFacesDirectionPolyhedronVsPolyhedron(shape_a, shape_b, mesh_a, mesh_b, polyhedron1_to_polyhedron2, face_index1);
    if (penetration_depth1 <= PE_R(0.0)) {
        // We have found a separating axis
        return true;
    }

    // Test all the face normals of the polyhedron 2 for separating axis
    uint32_t face_index2;
    pe::Real penetration_depth2 = testFacesDirectionPolyhedronVsPolyhedron(shape_b, shape_a, mesh_b, mesh_a, polyhedron2_to_polyhedron1, face_index2);
    if (penetration_depth2 <= PE_R(0.0)) {
        // We have found a separating axis
        return true;
    }

    // Here we know that we have found penetration along both axis of a face of polyhedron1 and a face of
    // polyhedron2. If the two penetration depths are almost the same, we need to make sure we always prefer
    // one axis to the other for consistency between frames. This is to prevent the contact manifolds to switch
    // from one reference axis to the other for a face to face resting contact for instance. This is better for
    // stability. To do this, we use a relative and absolute bias to move penetrationDepth2 a little bit to the right.
    // Now if:
    //  penetrationDepth1 < penetrationDepth2: Nothing happens and we use axis of polygon 1
    //  penetrationDepth1 ~ penetrationDepth2: Until penetrationDepth2 becomes significantly less than penetrationDepth1 we still use axis of polygon 1
    //  penetrationDepth1 >> penetrationDepth2: penetrationDepth2 is now significantly less than penetrationDepth1 and we use polygon 2 axis
    if (penetration_depth1 < penetration_depth2 * SEPARATING_AXIS_RELATIVE_TOLERANCE + SEPARATING_AXIS_ABSOLUTE_TOLERANCE) {
        // We use penetration axis of polygon 1
        is_min_penetration_face_normal = true;
        min_penetration_depth = PE_MIN(penetration_depth1, penetration_depth2);
        min_face_index = face_index1;
        is_min_penetration_face_normal_polyhedron1 = true;
    } else {
        // We use penetration axis of polygon 2
        is_min_penetration_face_normal = true;
        min_penetration_depth = PE_MIN(penetration_depth1, penetration_depth2);
        min_face_index = face_index2;
        is_min_penetration_face_normal_polyhedron1 = false;
    }

    bool separating_axis_found = false;

    // Test the cross products of edges of polyhedron 1 with edges of polyhedron 2 for separating axis
    for (const auto & edge1 : unique_edges_a) {
        // Get an edge of polyhedron 1
        const pe::Vector3 edge1_a = polyhedron1_to_polyhedron2 * edge1.start;
        const pe::Vector3 edge1_b = polyhedron1_to_polyhedron2 * edge1.end;
        const pe::Vector3 edge1_direction = edge1_b - edge1_a;

        for (const auto & edge2 : unique_edges_b) {
            // Get an edge of polyhedron 2
            const pe::Vector3& edge2_a = edge2.start;
            const pe::Vector3& edge2_b = edge2.end;
            const pe::Vector3 edge2_direction = edge2_b - edge2_a;

            // If the two edges build a minkowski face (and the cross product is therefore a candidate for separating axis
            if (testEdgesBuildMinkowskiFace(mesh_a, mesh_b, edge1, edge2, polyhedron1_to_polyhedron2)) {
                pe::Vector3 separating_axis_polyhedron2space;

                // Compute the penetration depth
                bool is_shape1_triangle = mesh_a.faces[edge1.face1_id].indices.size() == 3 || mesh_a.faces[edge1.face2_id].indices.size() == 3;
                pe::Real penetration_depth = computeDistanceBetweenEdges(edge1_a, edge2_a, polyhedron1_to_polyhedron2.getOrigin(),
                    pe::Vector3::zeros(), edge1_direction, edge2_direction, is_shape1_triangle, separating_axis_polyhedron2space);

                if (penetration_depth <= PE_R(0.0)) {
                    // We have found a separating axis
                    separating_axis_found = true;
                    break;
                }

                // If the current minimum penetration depth is along a face normal axis (isMinPenetrationFaceNormal=true) and we have found a new
                // smaller penetration depth along an edge-edge cross-product axis we want to favor the face normal axis because contact manifolds between
                // faces have more contact points and therefore more stable than the single contact point of an edge-edge collision. It means that if the new minimum
                // penetration depth from the edge-edge contact is only a little bit smaller than the current minPenetrationDepth (from a face contact), we favor
                // the face contact and do not generate an edge-edge contact. However, if the new penetration depth from the edge-edge contact is really smaller than
                // the current one, we generate an edge-edge contact.
                // To do this, we use a relative and absolute bias to increase a little bit the new penetration depth from the edge-edge contact during the comparison test
                if ((is_min_penetration_face_normal && penetration_depth * SEPARATING_AXIS_RELATIVE_TOLERANCE + SEPARATING_AXIS_ABSOLUTE_TOLERANCE < min_penetration_depth) ||
                    (!is_min_penetration_face_normal && penetration_depth < min_penetration_depth)) {
                    min_penetration_depth = penetration_depth;
                    is_min_penetration_face_normal_polyhedron1 = false;
                    is_min_penetration_face_normal = false;
                    separating_edge1_a = edge1_a;
                    separating_edge1_b = edge1_b;
                    separating_edge2_a = edge2_a;
                    separating_edge2_b = edge2_b;
                    min_edge_vs_edge_separating_axis_polyhedron2space = separating_axis_polyhedron2space;
                }
            }
        }

        if (separating_axis_found) {
            break;
        }
    }

    if (separating_axis_found) {
        return true;
    }

    // Here we know the shapes are overlapping on a given minimum separating axis.
    // Now, we will clip the shapes along this axis to find the contact points

    // If the minimum separating axis is a face normal
    if (is_min_penetration_face_normal) {
        // Compute the contact points between two faces of two convex polyhedra.
        bool contacts_found = computePolyhedronVsPolyhedronFaceContactPoints(is_min_penetration_face_normal_polyhedron1, mesh_a, mesh_b,
            trans_a, trans_b, polyhedron1_to_polyhedron2, polyhedron2_to_polyhedron1, min_face_index, margin, result);

        // There should be clipping points here. If it is not the case, it might be
        // because of a numerical issue
        if (!contacts_found) {
            return true;
        }
    } else { // If we have an edge vs edge contact
        // Compute the closest points between the two edges (in the local-space of poylhedron 2)
        pe::Vector3 closest_point_polyhedron1_edge, closest_point_polyhedron2_edge;
        computeClosestPointBetweenTwoSegments(separating_edge1_a, separating_edge1_b, separating_edge2_a, separating_edge2_b,
            closest_point_polyhedron1_edge, closest_point_polyhedron2_edge);

        // Compute the world normal
        pe::Vector3 normal_world = trans_b.getBasis() * min_edge_vs_edge_separating_axis_polyhedron2space;

        // Create the contact point
        result.addContactPoint(-normal_world, // Always points from B to A
            trans_b * closest_point_polyhedron2_edge // Always use point on B
            + normal_world * margin,
            -min_penetration_depth + margin);
    }

    return false;
}

bool ConvexConvexCollisionAlgorithm::getClosestPoints(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                                                      const pe::Mesh &mesh_a, const pe::Mesh &mesh_b,
                                                      const pe::Array<pe_physics_shape::UniqueEdge> &unique_edges_a,
                                                      const pe::Array<pe_physics_shape::UniqueEdge> &unique_edges_b,
                                                      const pe::Transform &trans_a, const pe::Transform &trans_b,
                                                      pe::Real margin, pe::Real refScale, ContactResult& result) {
    if (PE_CONVEX_CONVEX_USE_METHOD1) {
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
    } else {
        return findSeparatingAxis2(shape_a, shape_b, mesh_a, mesh_b,
            unique_edges_a, unique_edges_b, trans_a, trans_b, margin, result);
    }
}

} // pe_physics_collision
