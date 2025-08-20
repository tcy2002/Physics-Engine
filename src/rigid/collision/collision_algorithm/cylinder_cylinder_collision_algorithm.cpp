#include "cylinder_cylinder_collision_algorithm.h"
#include "rigid/shape/cylinder_shape.h"
#include "rigid/shape/default_mesh.h"
#include "convex_convex_collision_algorithm.h"

// style-checked.
namespace pe_phys_collision {

    bool CylinderCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                              pe::Transform trans_a, pe::Transform trans_b,
                                                              pe::Real ref_scale, ContactResult& result) {
        if (!(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder &&
              shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder)) {
            return false;
        }
        constexpr auto margin = PE_MARGIN;

#   if true
        const auto shape_cyl_a = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a);
        const auto shape_cyl_b = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_b);
        auto& mesh_a = shape_cyl_a->getMesh();
        auto& mesh_b = shape_cyl_b->getMesh();
        auto& edges_a = shape_cyl_a->getUniqueEdges();
        auto& edges_b = shape_cyl_b->getUniqueEdges();

        return ConvexConvexCollisionAlgorithm::getClosestPoints(shape_a, shape_b, mesh_a, mesh_b,
            edges_a, edges_b, trans_a, trans_b, margin, ref_scale, result);
#   else
        // TODO
#   endif
    }

} // pe_phys_collision