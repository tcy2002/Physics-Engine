#include "concave_cylinder_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "cylinder_convex_collision_algorithm.h"
#include "rigid/shape/cylinder_shape.h"
#include "rigid/shape/concave_mesh_shape.h"

// style-checked.
namespace pe_phys_collision {

    bool ConcaveCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                        pe::Transform trans_a, pe::Transform trans_b,
                                                        pe::Real refScale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder &&
            shape_b->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder))) {
            return false;
        }
        constexpr auto margin = PE_MARGIN;

#   if false
        auto shape_concave = dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? shape_a : shape_b);
        auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? shape_a : shape_b);
        auto trans_concave = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? trans_a : trans_b;
        auto trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? trans_a : trans_b;
        auto& mesh_concave = shape_concave->getMesh();
        auto& mesh_cyl = shape_cyl->getMesh();
        auto& edges_cyl = shape_cyl->getUniqueEdges();

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh);
        bool ret = ConcaveConvexCollisionAlgorithm::getClosestPoints(
            shape_concave, shape_cyl, trans_concave, trans_cyl, edges_cyl,
            mesh_concave, mesh_cyl, margin, refScale, result);
        result.setSwapFlag(false);

        return ret;
#   else
        const auto shape_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? shape_a : shape_b;
        const auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? shape_a : shape_b);
        auto& trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? trans_a : trans_b;
        auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? trans_a : trans_b;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh);
        bool ret = CylinderConvexCollisionAlgorithm::getClosestPoints(shape_mesh, shape_cyl, trans_mesh, trans_cyl, margin, result);
        result.setSwapFlag(false);

        return ret;
#   endif
    }
    
} // pe_phys_collision