#include "box_cylinder_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/default_mesh.h"
#include "convex_convex_collision_algorithm.h"

namespace pe_phys_collision {

    bool BoxCylinderCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                         pe_phys_object::RigidBody* object_b,
                                                         ContactResult& result) {
        if (object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Cylinder) {
            std::swap(object_a, object_b);
        }
        if (!(object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Box &&
              object_b->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Cylinder)) {
            return false;
        }

        auto shape_a = (pe_phys_shape::BoxShape*)object_a->getCollisionShape();
        auto shape_b = (pe_phys_shape::CylinderShape*)object_b->getCollisionShape();
        auto mesh_a = pe_phys_shape::_box_mesh;
        auto& size = shape_a->getSize();
        for (auto& v : mesh_a.vertices) {
            v.position.x *= size.x;
            v.position.y *= size.y;
            v.position.z *= size.z;
        }
        auto mesh_b = pe_phys_shape::_cylinder_mesh;
        pe::Real radius = shape_b->getRadius();
        pe::Real height = shape_b->getHeight() * 0.5;
        for (auto& v : mesh_b.vertices) {
            v.position.x *= radius;
            v.position.z *= radius;
            v.position.y *= height;
        }
        auto transA = object_a->getTransform();
        auto transB = object_b->getTransform();

        pe::Vector3 sep;
        pe::Real margin = 0.005;
        result.setObjects(object_a, object_b);

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        result.cleanContactPointFlag();
        if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(shape_a, shape_b,
                                                                mesh_a, mesh_b,
                                                                pe_phys_shape::_box_unique_edges,
                                                                pe_phys_shape::_cylinder_unique_edges,
                                                                transA, transB, sep, margin, result)) {
            return false;
        }
        ConvexConvexCollisionAlgorithm::clipHullAgainstHull(sep,
                                                            mesh_a, mesh_b, transA, transB,
                                                            PE_REAL_MIN, 0,
                                                            world_verts_b1, world_verts_b2,
                                                            margin, result);
        result.sortContactPoints();
        return result.getPointSize() > 0;
    }

} // pe_phys_collision