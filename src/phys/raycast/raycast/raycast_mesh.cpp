#include "raycast_mesh.h"

#include <phys/shape/concave_mesh_shape.h>

#include "phys/shape/convex_mesh_shape.h"
#include "phys/raycast/raycast/raycast_box.h"

namespace pe_phys_raycast {

    bool RaycastMesh::processRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real max_dist,
                                     pe_phys_shape::Shape* shape, pe::Transform trans,
                                     pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {

        auto shape_mesh = shape->getType() == pe_phys_shape::ShapeType::ConvexMesh ?
            dynamic_cast<pe_phys_shape::ConvexMeshShape*>(shape) :
            dynamic_cast<pe_phys_shape::ConcaveMeshShape*>(shape);
        const pe::Mesh* mesh = &shape_mesh->getMesh();

        const pe::Vector3 start_local = trans.inverseTransform(start);
        const pe::Vector3 dir_local = trans.getBasis().transposed() * direction;
        const pe::Vector3 end_local = start_local + dir_local * max_dist;
        const pe::Vector3 box_min = pe::Vector3::min2(start_local, end_local) - pe::Vector3::ones() * R(0.1);
        const pe::Vector3 box_max = pe::Vector3::max2(start_local, end_local) + pe::Vector3::ones() * R(0.1);
        pe::Array<int> hit_faces;
        shape_mesh->getIntersectFaces(box_min, box_max, hit_faces);

        distance = PE_REAL_MAX;
        hit_point = pe::Vector3::zeros();
        hit_normal = pe::Vector3::zeros();
        for (const auto fi : hit_faces) {
            auto& face = mesh->faces[fi];
            pe::Real d;
            pe::Vector3 hit;
            for (int i = 0; i < I(face.indices.size()) - 2; i++) {
                auto& v0 = mesh->vertices[face.indices[0]].position;
                auto& v1 = mesh->vertices[face.indices[i + 1]].position;
                auto& v2 = mesh->vertices[face.indices[i + 2]].position;
                if (rayHitTriangle(start_local, dir_local, v0, v1, v2, d, hit)) {
                    if (d < distance) {
                        distance = d;
                        hit_point = hit;
                        hit_normal = face.normal;
                    }
                    break; // will hit one triangle at most
                }
            }
        }
        if (distance < max_dist) {
            hit_point = trans * hit_point;
            hit_normal = trans.getBasis() * hit_normal;
            return true;
        }
        return false;
    }

    bool RaycastMesh::rayHitTriangle(const pe::Vector3& start, const pe::Vector3& direction,
                                     const pe::Vector3& v0, const pe::Vector3& v1, const pe::Vector3& v2,
                                     pe::Real& distance, pe::Vector3& hitPoint) {
        const pe::Vector3 edge1 = v1 - v0, edge2 = v2 - v0;
        const pe::Vector3 p_vec = direction.cross(edge2);
        const pe::Real det = edge1.dot(p_vec);
        if (det > -PE_EPS && det < PE_EPS) {
            return false;
        }
        const pe::Real inv_det = R(1.0) / det;
        const pe::Vector3 t_vec = start - v0;
        const pe::Real u = t_vec.dot(p_vec) * inv_det;
        if (u < 0 || u > R(1.0)) {
            return false;
        }
        const pe::Vector3 q_vec = t_vec.cross(edge1);
        const pe::Real v = direction.dot(q_vec) * inv_det;
        if (v < 0 || u + v > R(1.0)) {
            return false;
        }
        distance = edge2.dot(q_vec) * inv_det;
        hitPoint = start + direction * distance;
        return true;
    }

} // namespace pe_phys_ray