#include "concave_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"

namespace pe_phys_collision {

    bool ConcaveConvexCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh &&
            shape_b->getType() == pe_phys_shape::ShapeType::ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ConvexMesh))) {
            return false;
        }

        auto shape_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? shape_a : shape_b;
        auto shape_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? shape_a : shape_b;
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ConcaveMesh ? trans_a : trans_b;
        auto trans_convex = shape_a->getType() == pe_phys_shape::ShapeType::ConvexMesh ? trans_a : trans_b;
        return false;
    }

} // pe_phys_collision