#include "capsule_capsule_collision_algorithm.h"
#include "physics/shape/capsule_shape.h"

namespace pe_physics_collision {

bool CapsuleCapsuleCollisionAlgorithm::processCollision(pe_physics_shape::Shape *shape_a, pe_physics_shape::Shape *shape_b,
                                                        pe::Transform trans_a, pe::Transform trans_b,
                                                        pe::Real refScale, ContactResult &result) {
    if (!(shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule &&
          shape_b->getType() == pe_physics_shape::ShapeType::ST_Capsule)) {
        return false;
    }

    const auto shape_capsule_a = dynamic_cast<pe_physics_shape::CapsuleShape *>(shape_a);
    const auto shape_capsule_b = dynamic_cast<pe_physics_shape::CapsuleShape *>(shape_b);

    constexpr auto margin = PE_MARGIN;

    return getClosestPoints(shape_capsule_a, shape_capsule_b, trans_a, trans_b, margin, result);
}

bool CapsuleCapsuleCollisionAlgorithm::getClosestPoints(pe_physics_shape::CapsuleShape *shape_a, pe_physics_shape::CapsuleShape *shape_b,
                                                        const pe::Transform &trans_a, const pe::Transform &trans_b,
                                                        pe::Real margin, ContactResult &result) {
    const pe::Real radius_a = shape_a->getRadius();
    const pe::Real half_height_a = shape_a->getHeight() / PE_R(2.0);
    const pe::Real radius_b = shape_b->getRadius();
    const pe::Real half_height_b = shape_b->getHeight() / PE_R(2.0);
    const pe::Vector3 offset_b = trans_b.getOrigin() - trans_a.getOrigin();

    //Compute the closest points between the two line segments. No clamping to begin with.
    //We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
    //Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
    //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))
    const pe::Vector3 xa = trans_a.getBasis() * pe::Vector3(1, 0, 0);
    const pe::Vector3 da = trans_a.getBasis() * pe::Vector3(0, 1, 0);
    const pe::Vector3 db = trans_b.getBasis() * pe::Vector3(0, 1, 0);
    const pe::Real da_offset_b = da.dot(offset_b);
    const pe::Real db_offset_b = db.dot(offset_b);
    const pe::Real da_db = da.dot(db);
    //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
    pe::Real ta = (da_offset_b - db_offset_b * da_db) / PE_MAX(PE_R(1e-15), PE_R(1.0) - da_db * da_db);
    //tb = ta * (da * db) - db * (b - a)
    pe::Real tb = ta * da_db - db_offset_b;

    //We cannot simply clamp the ta and tb values to the capsule line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
    //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
    //The projected intervals are:
    //B onto A: +-BHalfLength * (da * db) + da * offsetB
    //A onto B: +-AHalfLength * (da * db) - db * offsetB
    const pe::Real abs_da_db = PE_ABS(da_db);
    const pe::Real b_onto_a_offset = half_height_b * abs_da_db;
    const pe::Real a_onto_b_offset = half_height_a * abs_da_db;
    pe::Real a_min = PE_MAX(-half_height_a, PE_MIN(half_height_a, da_offset_b - b_onto_a_offset));
    pe::Real a_max = PE_MIN(half_height_a, PE_MAX(-half_height_a, da_offset_b + b_onto_a_offset));
    const pe::Real b_min = PE_MAX(-half_height_b, PE_MIN(half_height_b, -db_offset_b - a_onto_b_offset));
    const pe::Real b_max = PE_MIN(half_height_b, PE_MAX(-half_height_b, -db_offset_b + a_onto_b_offset));
    ta = PE_MIN(PE_MAX(ta, a_min), a_max);
    tb = PE_MIN(PE_MAX(tb, b_min), b_max);

    const pe::Vector3 closest_point_on_a = da * ta;
    const pe::Vector3 closest_point_on_b = db * tb + offset_b;
    //Note that normals are calibrated to point from B to A by convention.
    pe::Vector3 normal = closest_point_on_a - closest_point_on_b;
    const pe::Real distance = normal.norm();
    const pe::Real inverse_distance = PE_R(1.0) / distance;
    normal *= inverse_distance;
    //In the event that the line segments are touching, the normal doesn't exist and we need an alternative. Any direction along the local horizontal (XZ) plane of either capsule
    //is valid. (Normals along the local Y axes are not guaranteed to be as quick of a path to separation due to nonzero line length.)
    const bool normal_is_valid = distance > PE_R(1e-7);
    normal = normal_is_valid ? normal : xa;

    //In the event that the two capsule axes are coplanar, we accept the whole interval as a source of contact.
    //As the axes drift away from coplanarity, the accepted interval rapidly narrows to zero length, centered on ta and tb.
    //We rate the degree of coplanarity based on the angle between the capsule axis and the plane defined by the opposing segment and contact normal:
    //sin(angle) = dot(da, (db x normal)/||db x normal||)
    //Finally, note that we are dealing with extremely small angles, and for small angles sin(angle) ~= angle,
    //and also that fade behavior is completely arbitrary, so we can directly use squared angle without any concern.
    //angle^2 ~= dot(da, (db x normal))^2 / ||db x normal||^2
    //Note that if ||db x normal|| is zero, then any da should be accepted as being coplanar because there is no restriction. ConditionalSelect away the discontinuity.
    const pe::Vector3 plane_normal = db.cross(normal);
    const pe::Real plane_normal_length_squared = plane_normal.norm2();
    const pe::Real numerator_unsquared = da.dot(plane_normal);
    const pe::Real squared_angle = plane_normal_length_squared < PE_R(1e-10) ? PE_R(0.0) :
                                   numerator_unsquared * numerator_unsquared / plane_normal_length_squared;

    //Convert the squared angle to a lerp parameter. For squared angle from 0 to lowerThreshold, we should use the full interval (1). From lowerThreshold to upperThreshold, lerp to 0.
    const pe::Real lower_threshold_angle = PE_R(0.01);
    const pe::Real upper_threshold_angle = PE_R(0.05);
    const pe::Real lower_threshold = lower_threshold_angle * lower_threshold_angle;
    const pe::Real upper_threshold = upper_threshold_angle * upper_threshold_angle;
    const pe::Real interval_weight = PE_MAX(PE_R(0.0), PE_MIN(PE_R(1.0), (upper_threshold - squared_angle) / (upper_threshold - lower_threshold)));
    //If the line segments intersect, even if they're coplanar, we would ideally stick to using a single point. Would be easy enough,
    //but we don't bother because it's such a weird and extremely temporary corner case. Not really worth handling.
    const pe::Real weighted_ta = ta - ta * interval_weight;
    a_min = interval_weight * a_min + weighted_ta;
    a_max = interval_weight * a_max + weighted_ta;

    const pe::Vector3 offset_a0 = da * a_min;
    const pe::Vector3 offset_a1 = da * a_max;
    //In the coplanar case, there are two points. We need a method of computing depth which gives a reasonable result to the second contact.
    //Note that one of the two contacts should end up with a distance equal to the previously computed segment distance, so we're doing some redundant work here.
    //It's just easier to do that extra work than it would be to track which endpoint contributed the lower distance.
    //Unproject the final interval endpoints from a back onto b.
    //dot(offsetB + db * tb0, da) = ta0
    //tb0 = (ta0 - daOffsetB) / dadb
    //distance0 = dot(a0 - (offsetB + tb0 * db), normal)
    //distance1 = dot(a1 - (offsetB + tb1 * db), normal)
    const pe::Real db_normal = db.dot(normal);
    const pe::Vector3 offset_b0 = offset_a0 - offset_b;
    const pe::Vector3 offset_b1 = offset_a1 - offset_b;
    //Note potential division by zero. In that case, treat both projected points as the closest point. (Handled by the conditional select that chooses the previously computed distance.)
    const pe::Real inverse_da_db = PE_R(1.0) / da_db;
    const pe::Real projected_tb0 = PE_MAX(b_min, PE_MIN(b_max, (a_min - da_offset_b) * inverse_da_db));
    const pe::Real projected_tb1 = PE_MAX(b_min, PE_MIN(b_max, (a_max - da_offset_b) * inverse_da_db));
    const pe::Real b0_normal = offset_b0.dot(normal);
    const pe::Real b1_normal = offset_b1.dot(normal);
    const bool capsules_are_perpendicular = PE_ABS(da_db) < PE_R(1e-7);
    const pe::Real distance0 = capsules_are_perpendicular ? distance : b0_normal - projected_tb0 * db_normal;
    const pe::Real distance1 = capsules_are_perpendicular ? distance : b1_normal - projected_tb1 * db_normal;
    const pe::Real combined_radius = radius_a + radius_b;
    const pe::Real depth0 = combined_radius - distance0;
    const pe::Real depth1 = combined_radius - distance1;

    //Apply the normal offset to the contact positions.
    const pe::Real negative_offset_from_a0 = depth0 - radius_a;
    const pe::Real negative_offset_from_a1 = depth1 - radius_a;
    const pe::Vector3 normal_push0 = normal * negative_offset_from_a0;
    const pe::Vector3 normal_push1 = normal * negative_offset_from_a1;
    const pe::Vector3 world_a0 = offset_a0 + normal_push0 + trans_a.getOrigin();
    const pe::Vector3 world_a1 = offset_a1 + normal_push1 + trans_a.getOrigin();

    bool ret = false;
    if (depth0 > PE_R(0.0)) {
        result.addContactPoint(normal, world_a0 - normal * margin, -depth0 + margin);
        ret = true;
    }
    if (depth1 > PE_R(0.0) && distance1 != distance0) {
        result.addContactPoint(normal, world_a1 - normal * margin, -depth1 + margin);
        ret = true;
    }
    return ret;
}


} // namespace pe_physics_collision
