#include "cylinder_convex_collision_algorithm.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/default_mesh.h"
#include "convex_convex_collision_algorithm.h"

namespace pe_phys_collision {

    bool CylinderConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                            pe::Transform trans_a, pe::Transform trans_b,
                                                            ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
               shape_b->getType() == pe_phys_shape::ShapeType::Cylinder))) {
            return false;
        }

        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       ((pe_phys_shape::CylinderShape*)shape_a)->getMesh() :
                       ((pe_phys_shape::ConvexMeshShape*)shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ?
                       ((pe_phys_shape::CylinderShape*)shape_b)->getMesh() :
                       ((pe_phys_shape::ConvexMeshShape*)shape_b)->getMesh();

        pe::Vector3 sep;
        pe::Real margin = PE_MARGIN;

        VertexArray world_verts_b1;
        VertexArray world_verts_b2;

        if (!ConvexConvexCollisionAlgorithm::findSeparatingAxis(shape_a, shape_b,
                                                                mesh_a, mesh_b,
                                                                shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? pe_phys_shape::_cylinder_unique_edges : ((pe_phys_shape::ConvexMeshShape*)shape_a)->getUniqueEdges(),
                                                                shape_b->getType() == pe_phys_shape::ShapeType::Cylinder ? pe_phys_shape::_cylinder_unique_edges : ((pe_phys_shape::ConvexMeshShape*)shape_b)->getUniqueEdges(),
                                                                trans_a, trans_b, sep, margin, result)) {
            return false;
        }
        ConvexConvexCollisionAlgorithm::clipHullAgainstHull(sep,
                                                            mesh_a, mesh_b, trans_a, trans_b,
                                                            PE_REAL_MIN, 0,
                                                            world_verts_b1, world_verts_b2,
                                                            margin, result);
        return true;
    }

} // pe_phys_collision