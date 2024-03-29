#include "raycast_mesh.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/concave_mesh_shape.h"

namespace pe_phys_ray {

    bool RaycastMesh::processRaycast(const pe::Vector3& start, const pe::Vector3& direction,
                                       pe_phys_object::RigidBody* object,
                                       pe::Real& distance, pe::Vector3& hit_point, pe::Vector3& hit_normal) {
        auto& trans = object->getTransform();
        pe::Vector3 start_local = trans.inverseTransform(start);
        pe::Vector3 dir_local = trans.getBasis().transposed() * direction;
        const pe::Mesh* mesh;
        if (object->getCollisionShape()->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            mesh = &((pe_phys_shape::ConvexMeshShape*)object->getCollisionShape())->getMesh();
        } else if (object->getCollisionShape()->getType() == pe_phys_shape::ShapeType::ConcaveMesh) {
            mesh = &((pe_phys_shape::ConcaveMeshShape*)object->getCollisionShape())->getMesh();
        } else {
            return false;
        }

        distance = PE_REAL_MAX;
        for (auto& face: mesh->faces) {
            pe::Real d;
            pe::Vector3 hit;
            for (int i = 0; i < (int)face.indices.size() - 2; i++) {
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
        if (distance < PE_REAL_MAX) {
            hit_point = trans * hit_point;
            hit_normal * trans.getBasis() * hit_normal;
            return true;
        }
        return false;
    }

    bool RaycastMesh::rayHitTriangle(const pe::Vector3& start, const pe::Vector3& direction,
                                     const pe::Vector3& v0, const pe::Vector3& v1, const pe::Vector3& v2,
                                     pe::Real& distance, pe::Vector3& hitPoint) {
        pe::Vector3 edge1 = v1 - v0, edge2 = v2 - v0;
        pe::Vector3 p_vec = direction.cross(edge2);
        pe::Real det = edge1.dot(p_vec);
        if (det > -PE_EPS && det < PE_EPS) {
            return false;
        }
        pe::Real inv_det = 1.0 / det;
        pe::Vector3 t_vec = start - v0;
        pe::Real u = t_vec.dot(p_vec) * inv_det;
        if (u < 0.0 || u > 1.0) {
            return false;
        }
        pe::Vector3 q_vec = t_vec.cross(edge1);
        pe::Real v = direction.dot(q_vec) * inv_det;
        if (v < 0.0 || u + v > 1.0) {
            return false;
        }
        distance = edge2.dot(q_vec) * inv_det;
        hitPoint = start + direction * distance;
        return true;
    }

} // namespace pe_phys_ray