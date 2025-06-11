#include "concave_box_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "rigid/shape/box_shape.h"
#include "rigid/shape/concave_mesh_shape.h"

// style-checked.
namespace pe_phys_collision {

    bool ConcaveBoxCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                        pe::Transform trans_a, pe::Transform trans_b,
                                                        pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Box &&
            shape_b->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ST_Box))) {
            return false;
        }

        auto shape_concave = dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? shape_a : shape_b);
        auto shape_box = dynamic_cast<pe_phys_shape::BoxShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? shape_a : shape_b);
        auto& trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? trans_a : trans_b;
        auto& trans_box = shape_a->getType() == pe_phys_shape::ShapeType::ST_Box ? trans_a : trans_b;
        auto& mesh_concave = shape_concave->getMesh();
        auto& mesh_box = shape_box->getMesh();
        auto& edges_box = shape_box->getUniqueEdges();

        constexpr auto margin = PE_MARGIN;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh);
        bool ret = ConcaveConvexCollisionAlgorithm::getClosestPoints(
            shape_concave, shape_box, trans_concave, trans_box, edges_box,
            mesh_concave, mesh_box, margin, refScale, result);
        result.setSwapFlag(false);

        return ret;
    }
    
} // pe_phys_collision