#include "box_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"
#include "rigid/shape/box_shape.h"
#include "rigid/shape/convex_mesh_shape.h"

namespace pe_phys_collision {

    bool BoxConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                       pe::Transform trans_a, pe::Transform trans_b,
                                                       pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Box &&
              shape_b->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh &&
              shape_b->getType() == pe_phys_shape::ShapeType::ST_Box))) {
            return false;
        }

        auto& mesh_a = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ?
                       dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a)->getMesh() :
                       dynamic_cast<pe_phys_shape::BoxShape *>(shape_a)->getMesh();
        auto& mesh_b = shape_b->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ?
                       dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b)->getMesh() :
                       dynamic_cast<pe_phys_shape::BoxShape *>(shape_b)->getMesh();
        auto& edges_a = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ?
                        dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_a)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::BoxShape *>(shape_a)->getUniqueEdges();
        auto& edges_b = shape_b->getType() == pe_phys_shape::ShapeType::ST_ConvexMesh ?
                        dynamic_cast<pe_phys_shape::ConvexMeshShape *>(shape_b)->getUniqueEdges() :
                        dynamic_cast<pe_phys_shape::BoxShape *>(shape_b)->getUniqueEdges();

        constexpr auto margin = PE_MARGIN;

        return ConvexConvexCollisionAlgorithm::getClosestPoints(shape_a, shape_b, mesh_a, mesh_b,
                                                                edges_a, edges_b, trans_a, trans_b,
                                                                margin, refScale, result);
    }

} // pe_phys_collision