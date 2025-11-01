#include "sphere_capsule_collision_algorithm.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/capsule_shape.h"

namespace pe_physics_collision {

bool SphereCapsuleCollisionAlgorithm::processCollision(pe_physics_shape::Shape *shape_a, pe_physics_shape::Shape *shape_b,
                                                       pe::Transform trans_a, pe::Transform trans_b,
                                                       pe::Real refScale, ContactResult &result) {
    if (!((shape_a->getType() == pe_physics_shape::ShapeType::ST_Sphere &&
           shape_b->getType() == pe_physics_shape::ShapeType::ST_Capsule) ||
          (shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule &&
           shape_b->getType() == pe_physics_shape::ShapeType::ST_Sphere))) {
        return false;
    }

    const auto shape_capsule = dynamic_cast<pe_physics_shape::CapsuleShape *>(shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule ? shape_a : shape_b);
    const auto shape_sphere = dynamic_cast<pe_physics_shape::SphereShape *>(shape_a->getType() == pe_physics_shape::ShapeType::ST_Sphere ? shape_a : shape_b);
    const auto& trans_capsule = shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule ? trans_a : trans_b;
    const auto& trans_sphere = shape_a->getType() == pe_physics_shape::ShapeType::ST_Sphere ? trans_a : trans_b;

    constexpr auto margin = PE_MARGIN;

    result.setSwapFlag(shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule);
    bool ret = getClosestPoints(shape_sphere, shape_capsule, trans_sphere, trans_capsule, margin, result);
    result.setSwapFlag(false);

    return ret;
}

bool SphereCapsuleCollisionAlgorithm::getClosestPoints(pe_physics_shape::SphereShape* shape_a, pe_physics_shape::CapsuleShape* shape_b,
                                                       const pe::Transform& trans_a, const pe::Transform& trans_b,
                                                       pe::Real margin, ContactResult& result) {
    const pe::Real capsule_half_height = shape_b->getHeight() / PE_R(2.0);
    const pe::Real capsule_radius = shape_b->getRadius();
    const pe::Real sphere_radius = shape_a->getRadius();
    const pe::Vector3 offset_b = trans_b.getOrigin() - trans_a.getOrigin();

    //The contact for a sphere-capsule pair is based on the closest point of the sphere center to the capsule internal line segment.
    const pe::Vector3 x = trans_b.getBasis() * pe::Vector3(1, 0, 0);
    const pe::Vector3 y = trans_b.getBasis() * pe::Vector3(0, 1, 0);
    pe::Real t = y.dot(offset_b);
    t = PE_MIN(capsule_half_height, PE_MAX(-capsule_half_height, -t));
    const pe::Vector3 capsule_local_closest_point_on_line_segment = y * t;

    const pe::Vector3 sphere_to_internal_segment = offset_b + capsule_local_closest_point_on_line_segment;
    const pe::Real internal_distance = sphere_to_internal_segment.norm();
    //Note that the normal points from B to A by convention. Here, the sphere is A, the capsule is B, so the normalization requires a negation.
    const pe::Real inverse_distance = PE_R(-1.0) / internal_distance;
    pe::Vector3 normal = sphere_to_internal_segment * inverse_distance;
    const bool normal_is_valid = internal_distance > PE_R(0.0);
    //If the center of the sphere is on the internal line segment, then choose a direction on the plane defined by the capsule's up vector.
    //We computed one such candidate earlier. Note that we could usually get away with choosing a completely arbitrary direction, but
    //going through the extra effort to compute a true local horizontal direction avoids some nasty corner case surprises if a user is trying
    //to do something like manually resolving collisions or other query-based logic.
    //A cheaper option would be to simply use the y axis as the normal. That's known to be suboptimal, but if we don't guarantee minimum penetration depth, that's totally fine.
    //My guess is that computing x will be so cheap as to be irrelevant.
    normal = normal_is_valid ? normal : x;
    const pe::Real depth = capsule_radius + sphere_radius - internal_distance;

    //The contact position relative to object A (the sphere) is computed as the average of the extreme point along the normal toward the opposing shape on each shape, averaged.
    //For capsule-sphere, this can be computed from the normal and depth.
    const pe::Real negative_offset_from_sphere = depth - sphere_radius;
    const pe::Vector3 world_a = normal * negative_offset_from_sphere + trans_a.getOrigin();

    if (depth > PE_R(0.0)) {
        result.addContactPoint(normal, world_a - normal * margin, -depth + margin);
        return true;
    }
    return false;
}

} // namespace pe_physics_collision
