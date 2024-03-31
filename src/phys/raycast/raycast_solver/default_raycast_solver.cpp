#include "default_raycast_solver.h"
#include "phys/raycast/raycast/raycast_box.h"
#include "phys/raycast/raycast/raycast_sphere.h"
#include "phys/raycast/raycast/raycast_cylinder.h"
#include "phys/raycast/raycast/raycast_mesh.h"

namespace pe_phys_raycast {

    bool DefaultRaycastSolver::performRaycast(const pe::Vector3& start, const pe::Vector3& direction, pe::Real length,
                                              const pe::Array<pe_phys_object::RigidBody*>& objects,
                                              const pe::Uint32HashList& ignores,
                                              RaycastResult& result) {
        result.m_distance = PE_REAL_MAX;
        result.m_collisionObject = nullptr;

        static RaycastBox r_box;
        static RaycastSphere r_sphere;
        static RaycastCylinder r_cylinder;
        static RaycastMesh r_mesh;

        for (auto rb : objects) {
            if (rb->isIgnoreCollision() || ignores.find(rb->getGlobalId()) != ignores.end()) {
                continue;
            }

            pe::Real distance;
            pe::Vector3 hit_point, hit_normal;
            bool ret;
            switch (rb->getCollisionShape()->getType()) {
                case pe_phys_shape::ShapeType::Box:
                    ret = r_box.processRaycast(start, direction, rb, distance, hit_point, hit_normal);
                    break;
                case pe_phys_shape::ShapeType::Sphere:
                    ret = r_sphere.processRaycast(start, direction, rb, distance, hit_point, hit_normal);
                    break;
                case pe_phys_shape::ShapeType::Cylinder:
                    ret = r_cylinder.processRaycast(start, direction, rb, distance, hit_point, hit_normal);
                    break;
                case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
                    ret = r_mesh.processRaycast(start, direction, rb, distance, hit_point, hit_normal);
                    break;
            }

            if (ret && distance <= length && distance < result.m_distance) {
                result.m_distance = distance;
                result.m_hitPoint = hit_point;
                result.m_hitNormal = hit_normal;
                result.m_collisionObject = rb;
            }
        }
        return result.hasHit();
    }

} // namespace pe_phys_ray