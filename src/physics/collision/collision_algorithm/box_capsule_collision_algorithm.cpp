#include "box_capsule_collision_algorithm.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/capsule_shape.h"

namespace pe_physics_collision {

bool BoxCapsuleCollisionAlgorithm::processCollision(pe_physics_shape::Shape *shape_a, pe_physics_shape::Shape *shape_b,
                                                    pe::Transform trans_a, pe::Transform trans_b,
                                                    pe::Real refScale, ContactResult &result) {
    if (!((shape_a->getType() == pe_physics_shape::ShapeType::ST_Box &&
           shape_b->getType() == pe_physics_shape::ShapeType::ST_Capsule) ||
          (shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule &&
           shape_b->getType() == pe_physics_shape::ShapeType::ST_Box))) {
        return false;
    }

    const auto shape_capsule = dynamic_cast<pe_physics_shape::CapsuleShape *>(shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule ? shape_a : shape_b);
    const auto shape_box = dynamic_cast<pe_physics_shape::BoxShape *>(shape_a->getType() == pe_physics_shape::ShapeType::ST_Box ? shape_a : shape_b);
    const auto& trans_capsule = shape_a->getType() == pe_physics_shape::ShapeType::ST_Capsule ? trans_a : trans_b;
    const auto& trans_box = shape_a->getType() == pe_physics_shape::ShapeType::ST_Box ? trans_a : trans_b;

    constexpr auto margin = PE_MARGIN;

    result.setSwapFlag(shape_a->getType() == pe_physics_shape::ShapeType::ST_Box);
    bool ret = getClosestPoints(shape_capsule, shape_box, trans_capsule, trans_box, margin, result);
    result.setSwapFlag(false);

    return ret;
}

static COMMON_FORCE_INLINE void prepare(pe_physics_shape::CapsuleShape* capsule, pe_physics_shape::BoxShape* box,
                    const pe::Vector3& offset_b, const pe::Matrix3& orientation_a, const pe::Matrix3& orientation_b,
                    pe::Vector3& local_offset_a, pe::Vector3& capsule_axis, pe::Vector3& edge_centers) {
    const pe::Matrix3 to_local_box = orientation_b.transposed();
    local_offset_a = -(to_local_box * offset_b);
    const pe::Matrix3 box_local_orient_capsule = to_local_box * orientation_a;
    capsule_axis = box_local_orient_capsule * pe::Vector3::up();

    //Get the closest point on the capsule segment to the box center to choose which edge to use.
    //(Pointless to test the other 9; they're guaranteed to be further away.)
    //closestPointOnCapsuleToBoxPosition = clamp((boxPosition - capsulePosition) * capsuleAxis, halfLength) * capsuleAxis + capsulePosition
    //offsetFromBoxToCapsule = closestPointOnCapsuleToBoxPosition - boxPosition
    //offsetFromBoxToCapsule = clamp(-localOffsetA * capsuleAxis, halfLength) * capsuleAxis + localOffsetA
    //offsetFromBoxToCapsule = localOffsetA - clamp(localOffsetA * capsuleAxis, halfLength) * capsuleAxis

    const pe::Real dot = local_offset_a.dot(capsule_axis);
    const pe::Real capsule_half_height = capsule->getHeight() / PE_R(2.0);
    const pe::Vector3 box_half_extent = box->getSize() / PE_R(2.0);
    const pe::Real clamped_dot = PE_MIN(capsule_half_height, PE_MAX(-capsule_half_height, dot));
    pe::Vector3 offset_to_capsule_from_box = capsule_axis * clamped_dot;
    offset_to_capsule_from_box = local_offset_a - offset_to_capsule_from_box;
    edge_centers.x = offset_to_capsule_from_box.x < PE_R(0.0) ? -box_half_extent.x : box_half_extent.x;
    edge_centers.y = offset_to_capsule_from_box.y < PE_R(0.0) ? -box_half_extent.y : box_half_extent.y;
    edge_centers.z = offset_to_capsule_from_box.z < PE_R(0.0) ? -box_half_extent.z : box_half_extent.z;
}

static COMMON_FORCE_INLINE void testBoxEdge(const pe::Real& offset_a_x, const pe::Real& offset_a_y, const pe::Real& offset_a_z,
                                            const pe::Real& capsule_axis_x, const pe::Real& capsule_axis_y, const pe::Real& capsule_axis_z,
                                            const pe::Real& capsule_half_height, const pe::Real& box_edge_center_x, const pe::Real& box_edge_center_y,
                                            const pe::Real& box_half_extent_x, const pe::Real& box_half_extent_y, const pe::Real& box_half_extent_z,
                                            pe::Real& ta_min, pe::Real& ta_max, pe::Vector3& closest_point_on_a,
                                            pe::Real& nx, pe::Real& ny, pe::Real& nz, pe::Real& ta, pe::Real& epsilon) {
    //From CapsulePairCollisionTask, point of closest approach along the capsule axis, unbounded:
    //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))
    //where da = capsuleAxis, db = (0,0,1), a = offsetA, b = (boxEdgeCenterX, boxEdgeCenterY, 0)
    //so da * db is simply capsuleAxis.Z.
    const pe::Real abx = box_edge_center_x - offset_a_x;
    const pe::Real aby = box_edge_center_y - offset_a_y;
    const pe::Real da_offset_b = capsule_axis_x * abx + capsule_axis_y * aby - capsule_axis_z * offset_a_z;
    //Note potential division by zero. Even though ta is not mathematically relevant when parallel (or even coplanar),
    //the max is a very easy way to stop NaNs from infecting later calculations.
    ta = (da_offset_b + offset_a_z * capsule_axis_z) / PE_MAX(PE_R(1.0) - capsule_axis_z * capsule_axis_z, PE_R(1e-15));
    //tb = ta * (da * db) - db * (b - a)
    pe::Real tb = ta * capsule_axis_z + offset_a_z;

    //Clamp solution to valid regions on edge line segment.
    //B onto A: +-BHalfLength * (da * db) + da * offsetB
    //A onto B: +-AHalfLength * (da * db) + db * offsetA
    const pe::Real abs_da_db = PE_ABS(capsule_axis_z);
    const pe::Real b_onto_a_offset = box_half_extent_z * abs_da_db;
    const pe::Real a_onto_b_offset = capsule_half_height * abs_da_db;
    ta_min = PE_MAX(-capsule_half_height, PE_MIN(capsule_half_height, da_offset_b - b_onto_a_offset));
    ta_max = PE_MIN(capsule_half_height, PE_MAX(-capsule_half_height, da_offset_b + b_onto_a_offset));
    const pe::Real b_min = PE_MAX(-box_half_extent_z, PE_MIN(box_half_extent_z, offset_a_z - a_onto_b_offset));
    const pe::Real b_max = PE_MIN(box_half_extent_z, PE_MAX(-box_half_extent_z, offset_a_z + a_onto_b_offset));
    ta = PE_MIN(PE_MAX(ta, ta_min), ta_max);
    tb = PE_MIN(PE_MAX(tb, b_min), b_max);

    //Note that we leave the normal as non-unit length. If this turns out to be zero length, we will have to resort to the interior test for the normal.
    //We can, however, still make use the interval information for position.
    closest_point_on_a.x = ta * capsule_axis_x + offset_a_x;
    closest_point_on_a.y = ta * capsule_axis_y + offset_a_y;
    closest_point_on_a.z = ta * capsule_axis_z + offset_a_z;
    nx = closest_point_on_a.x - box_edge_center_x;
    ny = closest_point_on_a.x - box_edge_center_y;
    nz = closest_point_on_a.z - tb;
    pe::Real squared_length = nx * nx + ny * ny + nz * nz;
    //If the box edge and capsule segment intersect, the normal will be zero. That's inconvenient, so add in a fallback based on cross(capsuleAxis, boxEdge).
    //(That simplifies to (-capsuleAxisY, capsuleAxisX, 0) thanks to the fact that boxEdge is (0,0,1).)
    const pe::Real fallback_squared_length = capsule_axis_x * capsule_axis_x + capsule_axis_y * capsule_axis_y;
    //But that can ALSO be zero length if the axes are parallel. If both those conditions are met, then we just pick (1,0,0).
    epsilon = PE_R(1e-10);
    const bool use_fall_back = squared_length < epsilon;
    const bool use_second_fallback = use_fall_back && fallback_squared_length < epsilon;
    squared_length = use_second_fallback ? PE_R(1.0) : (use_fall_back ? fallback_squared_length : squared_length);
    nx = use_second_fallback ? PE_R(1.0) : (use_fall_back ? -capsule_axis_y : nx);
    ny = use_second_fallback ? PE_R(0.0) : (use_fall_back ? capsule_axis_x : ny);
    nz = use_second_fallback ? PE_R(0.0) : (use_fall_back ? PE_R(0.0) : nz);

    //Calibrate the normal to point from B to A.
    const pe::Real calibration_dot = nx * offset_a_x + ny * offset_a_y + nz * offset_a_z;
    const bool should_negate = calibration_dot < PE_R(0.0);
    nx = should_negate ? -nx : nx;
    ny = should_negate ? -ny : ny;
    nz = should_negate ? -nz : nz;

    const pe::Real inverse_length = PE_R(1.0) / PE_SQRT(squared_length);
    nx *= inverse_length;
    ny *= inverse_length;
    nz *= inverse_length;
}

static COMMON_FORCE_INLINE void testAndRefineBoxEdge(const pe::Real& offset_a_x, const pe::Real& offset_a_y, const pe::Real& offset_a_z,
                                                     const pe::Real& capsule_axis_x, const pe::Real& capsule_axis_y, const pe::Real& capsule_axis_z,
                                                     const pe::Real& capsule_half_height, const pe::Real& box_edge_center_x, const pe::Real& box_edge_center_y,
                                                     const pe::Real& box_half_extent_x, const pe::Real& box_half_extent_y, const pe::Real& box_half_extent_z,
                                                     pe::Real& ta, pe::Real& depth, pe::Real& nx, pe::Real& ny, pe::Real& nz) {
    pe::Real ta_min, ta_max, epsilon;
    pe::Vector3 closest_point_on_a;
    testBoxEdge(offset_a_x, offset_a_y, offset_a_z, capsule_axis_x, capsule_axis_y, capsule_axis_z,
                capsule_half_height, box_edge_center_x, box_edge_center_y,
                box_half_extent_x, box_half_extent_y, box_half_extent_z,
                ta_min, ta_max, closest_point_on_a, nx, ny, nz, ta, epsilon);

    //Compute the depth along that normal.
    const pe::Real box_extreme = PE_ABS(nx) * box_half_extent_x +
                                 PE_ABS(ny) * box_half_extent_y +
                                 PE_ABS(nz) * box_half_extent_z;
    const pe::Real capsule_extreme = nx * closest_point_on_a.x +
                                     ny * closest_point_on_a.y +
                                     nz * closest_point_on_a.z;
    depth = box_extreme - capsule_extreme;
}

static COMMON_FORCE_INLINE void testBoxFace(const pe::Real& offset_a_z, const pe::Real& capsule_axis_z, const pe::Real& capsule_half_height,
                                            const pe::Real& box_half_extent_z, pe::Real& depth, pe::Real& normal_sign) {
    normal_sign = offset_a_z > PE_R(0.0) ? PE_R(1.0) : PE_R(-1.0);
    depth = box_half_extent_z + PE_ABS(capsule_axis_z) * capsule_half_height - normal_sign * offset_a_z;
}

static COMMON_FORCE_INLINE void select(pe::Real& depth, pe::Real& ta,
                                       pe::Real& local_normal_x, pe::Real& local_normal_y, pe::Real& local_normal_z,
                                       const pe::Real& depth_candidate, const pe::Real& ta_candidate,
                                       const pe::Real& local_normal_candidate_x, const pe::Real& local_normal_candidate_y, const pe::Real& local_normal_candidate_z) {
    const bool use_candidate = depth_candidate < depth;
    ta = use_candidate ? ta_candidate : ta;
    depth = use_candidate ? depth_candidate : depth;
    local_normal_x = use_candidate ? local_normal_candidate_x : local_normal_x;
    local_normal_y = use_candidate ? local_normal_candidate_y : local_normal_y;
    local_normal_z = use_candidate ? local_normal_candidate_z : local_normal_z;
}

static COMMON_FORCE_INLINE void select(pe::Real& depth,
                                       pe::Real& local_normal_x, pe::Real& local_normal_y, pe::Real& local_normal_z,
                                       const pe::Real& depth_candidate,
                                       const pe::Real& local_normal_candidate_x, const pe::Real& local_normal_candidate_y, const pe::Real& local_normal_candidate_z) {
    const bool use_candidate = depth_candidate < depth;
    depth = use_candidate ? depth_candidate : depth;
    local_normal_x = use_candidate ? local_normal_candidate_x : local_normal_x;
    local_normal_y = use_candidate ? local_normal_candidate_y : local_normal_y;
    local_normal_z = use_candidate ? local_normal_candidate_z : local_normal_z;
}

bool BoxCapsuleCollisionAlgorithm::getClosestPoints(pe_physics_shape::CapsuleShape* shape_a, pe_physics_shape::BoxShape* shape_b,
                                                    const pe::Transform& trans_a, const pe::Transform& trans_b,
                                                    pe::Real margin, ContactResult &result) {
    pe::Vector3 local_offset_a, capsule_axis, edge_centers;
    prepare(shape_a, shape_b, trans_b.getOrigin() - trans_a.getOrigin(),
            trans_a.getBasis(), trans_b.getBasis(),
            local_offset_a, capsule_axis, edge_centers);

    const pe::Real capsule_radius = shape_a->getRadius();
    const pe::Real capsule_half_height = shape_a->getHeight() / PE_R(2.0);
    const pe::Vector3 box_half_extent = shape_b->getSize() / PE_R(2.0);

    //Swizzle XYZ -> YZX
    pe::Vector3 local_normal;
    pe::Real ta, depth;
    testAndRefineBoxEdge(local_offset_a.y, local_offset_a.z, local_offset_a.x,
                         capsule_axis.y, capsule_axis.z, capsule_axis.x,
                         capsule_half_height,
                         edge_centers.y, edge_centers.z,
                         box_half_extent.y, box_half_extent.z, box_half_extent.x,
                         ta, depth, local_normal.y, local_normal.z, local_normal.x);
    //Swizzle XYZ -> ZXY
    pe::Real ey_ta, ey_depth, ey_nx, ey_ny, ey_nz;
    testAndRefineBoxEdge(local_offset_a.z, local_offset_a.x, local_offset_a.y,
                         capsule_axis.z, capsule_axis.x, capsule_axis.y,
                         capsule_half_height,
                         edge_centers.z, edge_centers.x,
                         box_half_extent.z, box_half_extent.x, box_half_extent.y,
                         ey_ta, ey_depth, ey_nz, ey_nx, ey_ny);
    select(depth, ta, local_normal.x, local_normal.y, local_normal.z,
           ey_depth, ey_ta, ey_nx, ey_ny, ey_nz);
    //Swizzle XYZ -> XYZ
    pe::Real ez_ta, ez_depth;
    pe::Vector3 ez_normal;
    testAndRefineBoxEdge(local_offset_a.x, local_offset_a.y, local_offset_a.z,
                         capsule_axis.x, capsule_axis.y, capsule_axis.z,
                         capsule_half_height,
                         edge_centers.x, edge_centers.y,
                         box_half_extent.x, box_half_extent.y, box_half_extent.z,
                         ez_ta, ez_depth, ez_normal.x, ez_normal.y, ez_normal.z);
    select(depth, ta, local_normal.x, local_normal.y, local_normal.z,
           ez_depth, ez_ta, ez_normal.x, ez_normal.y, ez_normal.z);

    //Face X
    pe::Real fx_depth, fx_n;
    testBoxFace(local_offset_a.x, capsule_axis.x, capsule_half_height,
                box_half_extent.x, fx_depth, fx_n);
    select(depth, local_normal.x, local_normal.y, local_normal.z,
           fx_depth, fx_n, PE_R(0.0), PE_R(0.0));
    //Face Y
    pe::Real fy_depth, fy_n;
    testBoxFace(local_offset_a.y, capsule_axis.y, capsule_half_height,
                box_half_extent.y, fy_depth, fy_n);
    select(depth, local_normal.x, local_normal.y, local_normal.z,
           fy_depth, PE_R(0.0), fy_n, PE_R(0.0));
    //Face Z
    pe::Real fz_depth, fz_n;
    testBoxFace(local_offset_a.z, capsule_axis.z, capsule_half_height,
                box_half_extent.z, fz_depth, fz_n);
    select(depth, local_normal.x, local_normal.y, local_normal.z,
           fz_depth, PE_R(0.0), PE_R(0.0), fz_n);

    //While the above chooses a minimal depth, edge-edge contact will frequently produce interval lengths of 0 and end up overwriting near-equivalent face intervals.
    //One option would be to bias the comparison to accept face contacts more readily, but we can do something a little less fragile, and which also gives us a
    //way to safely compute depth on a per contact basis:
    //choose a representative box face based on the collision normal detected above, and compute the interval of intersection along the capsule axis of the box face projected onto the capsule axis.
    //We'll compute this by unprojecting the capsule axis onto the box face plane along the local normal.
    //Note that this interval always includes the closest point.
    const pe::Real x_dot = local_normal.x * fx_n;
    const pe::Real y_dot = local_normal.y * fy_n;
    const pe::Real z_dot = local_normal.z * fz_n;
    const bool use_x = x_dot > PE_MAX(y_dot, z_dot);
    const bool use_y = !use_x && y_dot > z_dot;
    const bool use_z = !use_x && !use_y;

    //Unproject the capsule center and capsule axis onto the representative face plane.
    //unprojectedAxis = capsuleAxis - localNormal * dot(capsuleAxis, faceNormal) / dot(localNormal, faceNormal)
    //unprojectedCenter = capsuleCenter - localNormal * dot(capsuleCenter - pointOnFace, faceNormal) / dot(localNormal, faceNormal)
    const pe::Real face_normal_dot_local_normal = use_x ? x_dot : (use_y ? y_dot : z_dot);
    const pe::Real inverse_face_normal_dot_local_normal = PE_R(1.0) / PE_MAX(PE_R(1e-15), face_normal_dot_local_normal);
    const pe::Real capsule_axis_dot_face_normal = use_x ? capsule_axis.x * fx_n : (use_y ? capsule_axis.y * fy_n : capsule_axis.z * fz_n);
    const pe::Real capsule_center_dot_face_normal = use_x ? local_offset_a.x * fx_n : (use_y ? local_offset_a.y * fy_n : local_offset_a.z * fz_n);
    const pe::Real face_plane_offset = use_x ? box_half_extent.x : (use_y ? box_half_extent.y : box_half_extent.z);
    const pe::Real t_axis = capsule_axis_dot_face_normal * inverse_face_normal_dot_local_normal;
    const pe::Real t_center = (capsule_center_dot_face_normal - face_plane_offset) * inverse_face_normal_dot_local_normal;

    //Work in tangent space.
    //Face X uses tangents Y and Z.
    //Face Y uses tangents X and Z.
    //Face Z uses tangents X and Y.
    const pe::Vector3 axis_offset = local_normal * t_axis;
    const pe::Vector3 center_offset = local_normal * t_center;
    const pe::Vector3 unprojected_axis = capsule_axis - axis_offset;
    const pe::Vector3 unprojected_center = local_offset_a - center_offset;
    pe::Real tangent_space_axis_x = use_x ? unprojected_axis.y : unprojected_axis.x;
    pe::Real tangent_space_axis_y = use_z ? unprojected_axis.y : unprojected_axis.z;
    pe::Real tangent_space_center_x = use_x ? unprojected_center.y : unprojected_center.x;
    pe::Real tangent_space_center_y = use_x ? unprojected_center.y : unprojected_center.z;
    //Slightly boost the size of the face to avoid minor numerical issues that could block coplanar contacts.
    // const pe::Real epsilon_scale = PE_MIN(PE_MAX(box_half_extent.x, PE_MAX(box_half_extent.y, box_half_extent.z)), PE_MAX(capsule_half_height, capsule_radius));
    // const pe::Real epsilon = epsilon_scale * PE_R(1e-3);
    // const pe::Real half_extent_x = epsilon + (use_x ? box_half_extent.y : box_half_extent.x);
    // const pe::Real half_extent_y = epsilon + (use_z ? box_half_extent.y : box_half_extent.z);
    const pe::Real half_extent_x = use_x ? box_half_extent.y : box_half_extent.x;
    const pe::Real half_extent_y = use_z ? box_half_extent.y : box_half_extent.z;

    //Compute interval bounded by edge normals pointing along tangentX.
    //tX = -dot(tangentSpaceCenter +- halfExtentX, edgeNormal) / dot(unprojectedCapsuleAxis, edgeNormal)

    const pe::Real inverse_axis_x = PE_R(-1.0) / tangent_space_axis_x;
    const pe::Real inverse_axis_y = PE_R(-1.0) / tangent_space_axis_y;
    const pe::Real tx_0 = (tangent_space_center_x - half_extent_x) * inverse_axis_x;
    const pe::Real tx_1 = (tangent_space_center_x + half_extent_x) * inverse_axis_x;
    const pe::Real ty_0 = (tangent_space_center_y - half_extent_y) * inverse_axis_y;
    const pe::Real ty_1 = (tangent_space_center_y + half_extent_y) * inverse_axis_y;
    pe::Real min_x = PE_MIN(tx_0, tx_1);
    pe::Real max_x = PE_MAX(tx_0, tx_1);
    pe::Real min_y = PE_MIN(ty_0, ty_1);
    pe::Real max_y = PE_MAX(ty_0, ty_1);
    //Protect against division by zero. If the unprojected capsule is within the slab, use an infinite interval. If it's outside and parallel, use an invalid interval.
    const bool use_fallback_x = PE_ABS(tangent_space_axis_x, PE_R(1e-15));
    const bool use_fallback_y = PE_ABS(tangent_space_axis_y, PE_R(1e-15));
    const bool center_contained_x = PE_ABS(tangent_space_center_x <= half_extent_x);
    const bool center_contained_y = PE_ABS(tangent_space_center_y <= half_extent_y);
    min_x = use_fallback_x ? (center_contained_x ? PE_REAL_MIN : PE_REAL_MAX) : min_x;
    max_x = use_fallback_x ? (center_contained_x ? PE_REAL_MAX : PE_REAL_MIN) : max_x;
    min_y = use_fallback_y ? (center_contained_y ? PE_REAL_MIN : PE_REAL_MAX) : min_y;
    max_y = use_fallback_y ? (center_contained_y ? PE_REAL_MAX : PE_REAL_MIN) : max_y;

    const pe::Real face_min = PE_MAX(min_x, min_y);
    const pe::Real face_max = PE_MIN(max_x, max_y);
    //Clamp the resulting interval to the capsule axis.
    pe::Real t_min = PE_MAX(PE_MIN(face_min, capsule_half_height), -capsule_half_height);
    pe::Real t_max = PE_MAX(PE_MIN(face_max, capsule_half_height), -capsule_half_height);
    const bool face_interval_exists = face_max >= face_min;
    t_min = face_interval_exists ? PE_MIN(t_min, ta) : ta;
    t_max = face_interval_exists ? PE_MAX(t_max, ta) : ta;

    //Each contact may have its own depth.
    //Imagine a face collision- if the capsule axis isn't fully parallel with the plane's surface, it would be strange to use the same depth for both contacts.
    //We have two points on the capsule and box. We can reuse the unprojeection from earlier to compute the offset between them.
    const pe::Real separation_min = t_center + t_axis * t_min;
    const pe::Real separation_max = t_center + t_axis * t_max;
    const pe::Real depth0 = capsule_radius - separation_min;
    const pe::Real depth1 = capsule_radius - separation_max;

    const pe::Vector3 local_a0 = capsule_axis * t_min;
    const pe::Vector3 local_a1 = capsule_axis * t_max;

    //Transform A0, A1, and the normal into world space.
    const pe::Vector3 world_normal = trans_b.getBasis() * local_normal;
    pe::Vector3 world_a0 = trans_b.getBasis() * local_a0 + trans_a.getOrigin();
    pe::Vector3 world_a1 = trans_b.getBasis() * local_a1 + trans_a.getOrigin();

    //Apply the normal offset to the contact positions.
    const pe::Real negative_offset_from_a0 = depth0 - capsule_radius;
    const pe::Real negative_offset_from_a1 = depth1 - capsule_radius;
    const pe::Vector3 normal_push0 = world_normal * negative_offset_from_a0;
    const pe::Vector3 normal_push1 = world_normal * negative_offset_from_a1;
    world_a0 += normal_push0;
    world_a1 += normal_push1;

    bool ret = false;
    if (depth0 > PE_R(0.0)) {
        if (depth0 > PE_R(0.1)) {
            std::cout << "error depth0: " << depth0 << std::endl;
        }
        result.addContactPoint(world_normal, world_a0 - world_normal * margin, -depth0 + margin);
        ret = true;
    }
    if (depth1 > PE_R(0.0)) {
        if (depth0 > PE_R(0.1)) {
            std::cout << "error depth1: " << depth1 << std::endl;
        }
        result.addContactPoint(world_normal, world_a1 - world_normal * margin, -depth1 + margin);
        ret = true;
    }
    return ret;
}

} // namespace pe_physics_collision
