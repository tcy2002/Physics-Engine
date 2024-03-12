#include "box_convex_collision_algorithm.h"

#include "convex_convex_collision_algorithm.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/fracture/fracture_utils/default_mesh.h"

namespace pe_phys_collision {

    bool BoxConvexCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                       pe_phys_object::RigidBody* object_b, ContactResult& result) {
//        if (!((object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
//               object_b->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Box) ||
//              (object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Box &&
//               object_b->getCollisionShape()->getType() == pe_phys_shape::ShapeType::ConvexMesh))) {
//            return false;
//        }
//        if (object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Box) {
//            std::swap(object_a, object_b);
//        }
//        auto shape_a = (pe_phys_shape::ConvexMeshShape*)object_a->getCollisionShape();
//        auto shape_b = (pe_phys_shape::BoxShape*)object_b->getCollisionShape();
//
//        auto& mesh_a = shape_a->getMesh();
//        auto mesh_b = pe_phys_fracture::_box_mesh;
//        auto& size = shape_b->getSize();
//        for (auto& v : mesh_b.vertices) {
//            v.position = v.position * size;
//        }
//        auto transA = object_a->getTransform();
//        auto transB = object_b->getTransform();
//
//        pe::Vector3 sep;
//        pe::Real margin = 0.005;
//        result.setObjects(object_a, object_b);
//
//        VertexArray world_verts_b1;
//        VertexArray world_verts_b2;
//
//        result.cleanContactPointFlag();
//        if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(object_a, object_b, transA, transB,
//                                                                sep, margin, result)) {
//            return false;
//        }
//        ConvexConvexCollisionAlgorithm::clipHullAgainstHull(sep,
//                                                            shape_a, shape_b, transA, transB,
//                                                            PE_REAL_MIN, 0,
//                                                            world_verts_b1, world_verts_b2,
//                                                            margin, result);
//        result.sortContactPoints();
//        return result.getPointSize() > 0;
    }

} // pe_phys_collision