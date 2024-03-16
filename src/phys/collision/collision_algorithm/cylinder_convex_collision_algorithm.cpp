#include "cylinder_convex_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"

namespace pe_phys_collision {

    bool CylinderConvexCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                          pe_phys_object::RigidBody* object_b,
                                                          ContactResult& result) {
        if (object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            std::swap(object_a, object_b);
        }
        if (object_a->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Cylinder ||
            object_b->getCollisionShape()->getType() != pe_phys_shape::ShapeType::ConvexMesh) {
            return false;
        }

        auto shape_b = (pe_phys_shape::ConvexMeshShape *) object_b->getCollisionShape();
        auto& mesh_b = shape_b->getMesh();
        auto& trans_b = object_b->getTransform();

        pe::Vector3 vertices[3];
        result.setObjects(object_a, object_b);
        for (auto& f : mesh_b.faces) {
            for (int i = 0; i < f.indices.size() - 2; i++) {
                vertices[0] = mesh_b.vertices[f.indices[0]].position;
                vertices[1] = mesh_b.vertices[f.indices[i + 1]].position;
                vertices[2] = mesh_b.vertices[f.indices[i + 2]].position;
                getClosestPoints(object_a, vertices, trans_b, result);
            }
        }

        result.sortContactPoints();
        return result.getPointSize() > 0;
    }

    void CylinderConvexCollisionAlgorithm::getClosestPoints(pe_phys_object::RigidBody* object_a,
                                                          pe::Vector3 vertices[], const pe::Transform& transTri,
                                                          ContactResult& result) {
        const pe::Transform& transCyl = object_a->getTransform();

        pe::Vector3 point, normal;
        pe::Real depth = 0;
        pe::Real margin = 0.005;
        pe::Real radius = ((pe_phys_shape::CylinderShape*)object_a->getCollisionShape())->getRadius();
        pe::Real height = ((pe_phys_shape::CylinderShape*)object_a->getCollisionShape())->getHeight() * 0.5;

        for (int i = 0; i < 3; i++) {
            vertices[i] = transCyl.inverseTransform(transTri * vertices[i]);
        }

        if (collideCylinderTriangle(radius, height, vertices,
                                    point, normal, depth, margin)) {
            result.addContactPoint(transCyl.getBasis() * normal, transCyl * point, depth);
        }
    }

    bool CylinderConvexCollisionAlgorithm::collideCylinderTriangle(pe::Real radius, pe::Real height,
                                                                   const pe::Vector3 vertices[], pe::Vector3& point,
                                                                   pe::Vector3& resultNormal, pe::Real& depth,
                                                                   pe::Real margin) {

    }

} // pe_phys_collision