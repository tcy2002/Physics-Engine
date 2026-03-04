#include "concave_capsule_collision_algorithm.h"
#include "sphere_convex_collision_algorithm.h"
#include "physics/shape/capsule_shape.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/concave_mesh_shape.h"

namespace pe_physics_collision {

bool ConcaveCapsuleCollisionAlgorithm::processCollision(pe_physics_shape::Shape* shape_a, pe_physics_shape::Shape* shape_b,
                                                       pe::Transform trans_a, pe::Transform trans_b,
                                                       pe::Real refScale, ContactResult& result) {
    if (!((shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule &&
        shape_b->getType() == pe_physics_shape::ShapeType::ST_ConcaveMesh) ||
        (shape_a->getType() == pe_physics_shape::ShapeType::ST_ConcaveMesh &&
            shape_b->getType() == pe_physics_shape::ShapeType::ST_Capsule))) {
        return false;
    }

    const auto shape_mesh = dynamic_cast<pe_physics_shape::ConcaveMeshShape *>(shape_a->getType() == pe_physics_shape::ShapeType::ST_ConcaveMesh ? shape_a : shape_b);
    auto& mesh = shape_mesh->getMesh();
    const auto& trans_mesh = shape_a->getType() == pe_physics_shape::ShapeType::ST_ConcaveMesh ? trans_a : trans_b;
    const auto shape_cap = dynamic_cast<pe_physics_shape::CapsuleShape *>(shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule ? shape_a : shape_b);
    const auto& trans_cap = shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule ? trans_a : trans_b;

    constexpr auto margin = PE_MARGIN;

    const pe::Real s_radius = shape_cap->getRadius() + shape_cap->getHeight() / PE_R(2.0);
    pe_physics_shape::SphereShape shape_sph(s_radius);

    result.setSwapFlag(shape_a->getType() == pe_physics_shape::ShapeType::ST_ConcaveMesh);
    bool ret = SphereConvexCollisionAlgorithm::getClosestPoints(&shape_sph, shape_mesh, mesh, trans_cap, trans_mesh, margin, result);
    result.setSwapFlag(false);

    return ret;
}

} // pe_physics_collision
