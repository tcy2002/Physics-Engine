#include "concave_sphere_collision_algorithm.h"
#include "sphere_convex_collision_algorithm.h"
#include "rigid/shape/sphere_shape.h"
#include "rigid/shape/concave_mesh_shape.h"

// style-checked.
namespace pe_phys_collision {

    bool ConcaveSphereCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                           pe::Transform trans_a, pe::Transform trans_b,
                                                           pe::Real ref_scale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere &&
            shape_b->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh) ||
            (shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh &&
                shape_b->getType() == pe_phys_shape::ShapeType::ST_Sphere))) {
            return false;
        }

        const auto shape_mesh = dynamic_cast<pe_phys_shape::ConcaveMeshShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? shape_a : shape_b);
        auto& mesh = shape_mesh->getMesh();
        const auto& trans_mesh = shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh ? trans_a : trans_b;
        const auto shape_sph = dynamic_cast<pe_phys_shape::SphereShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere ? shape_a : shape_b);
        const auto& trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere ? trans_a : trans_b;

        constexpr auto margin = PE_MARGIN;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_ConcaveMesh);
        bool ret = SphereConvexCollisionAlgorithm::getClosestPoints(shape_sph, shape_mesh, mesh, trans_sph, trans_mesh, margin, result);
        result.setSwapFlag(false);

        return ret;
    }

} // pe_phys_collision