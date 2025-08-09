#include "default_raycast_solver.h"
#include "rigid/raycast/raycast/raycast_box.h"
#include "rigid/raycast/raycast/raycast_sphere.h"
#include "rigid/raycast/raycast/raycast_cylinder.h"
#include "rigid/raycast/raycast/raycast_mesh.h"
#include "rigid/shape/compound_shape.h"

// style-checked
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

        for (const auto rb : objects) {
            if (rb->isIgnoreCollision() || ignores.find(rb->getGlobalId()) != ignores.end()) {
                continue;
            }

            pe::Real distance;
            pe::Vector3 hit_point, hit_normal;
            bool ret = false;
            const auto shape = rb->getCollisionShape();
            auto trans = rb->getTransform();
            const auto type = shape->getType();
            switch (type) {
                case pe_phys_shape::ShapeType::ST_Box:
                    ret = r_box.processRaycast(start, direction, length, shape, trans, distance, hit_point, hit_normal);
                    break;
                case pe_phys_shape::ShapeType::ST_Sphere:
                    ret = r_sphere.processRaycast(start, direction, length, shape, trans, distance, hit_point, hit_normal);
                    break;
                case pe_phys_shape::ShapeType::ST_Cylinder:
                    ret = r_cylinder.processRaycast(start, direction, length, shape, trans, distance, hit_point, hit_normal);
                    break;
                case pe_phys_shape::ShapeType::ST_ConvexMesh:
                case pe_phys_shape::ShapeType::ST_ConcaveMesh:
                case pe_phys_shape::ShapeType::ST_Compound:
                    // first, check if the ray hit the AABB of the mesh
                    if (!RaycastBox::rayHitBox(start, direction, rb->getAABBMin(), rb->getAABBMax(),
                                               distance, hit_point, hit_normal)) {
                        return false;
                    }
                    if (type == pe_phys_shape::ShapeType::ST_ConvexMesh || type == pe_phys_shape::ShapeType::ST_ConcaveMesh) {
                        ret = r_mesh.processRaycast(start, direction, length, shape, trans, distance, hit_point, hit_normal);
                    } else {
                        auto compound = dynamic_cast<pe_phys_shape::CompoundShape *>(shape);
                        ret = false;
                        for (auto& child: compound->getShapes()) {
                            const auto& trans_chd = trans * child.local_transform;
                            switch (child.shape->getType()) {
                                case pe_phys_shape::ShapeType::ST_Box:
                                    ret |= r_box.processRaycast(start, direction, length, child.shape, trans_chd, distance, hit_point, hit_normal);
                                    break;
                                case pe_phys_shape::ShapeType::ST_Sphere:
                                    ret |= r_sphere.processRaycast(start, direction, length, child.shape, trans_chd, distance, hit_point, hit_normal);
                                    break;
                                case pe_phys_shape::ShapeType::ST_Cylinder:
                                    ret |= r_cylinder.processRaycast(start, direction, length, child.shape, trans_chd, distance, hit_point, hit_normal);
                                    break;
                                case pe_phys_shape::ShapeType::ST_ConvexMesh:
                                    ret |= r_mesh.processRaycast(start, direction, length, child.shape, trans_chd, distance, hit_point, hit_normal);
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
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