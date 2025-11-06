#pragma once

#include "physics/physics.h"
#include "interface/world.h"
#include "physics/object/rigidbody.h"
#include "physics/constraint/constraint/constraint.h"
#include "json/json.hpp"

namespace pe_vehicle {

class Chassis;

class Track {
    COMMON_MEMBER_GET(pe::Real, length, Length)
    COMMON_MEMBER_GET(pe::Real, radius, Radius)
    COMMON_MEMBER_GET(pe::Real, width, Width)
    COMMON_MEMBER_GET(int, segment_count, SegmentCount)
    COMMON_MEMBER_GET(pe::Real, setment_height, SegmentHeight)
    COMMON_MEMBER_GET(pe::Real, setment_length_ratio, SegmentLengthRatio)
    COMMON_MEMBER_GET(pe::Real, segment_mass, SegmentMass)
    COMMON_MEMBER_GET(pe::Real, segment_friction, SegmentFriction)
    COMMON_MEMBER_GET(pe::Real, tightening_ratio, TighteningRatio)

protected:
    pe::Array<pe_physics_object::RigidBody*> _segments;
    pe::Array<pe_physics_constraint::Constraint*> _joints;
    pe::Vector3 _anchor_chassis;

public:
    Track() = delete;
    Track(Chassis* chassis, const pe::Vector3& anchor, pe::Real length, pe::Real radius, pe::Real width,
          int segment_count, pe::Real segment_height, pe::Real segment_length_ratio,
          pe::Real segment_mass, pe::Real segment_friction, pe::Real tightening_ratio);
    virtual ~Track();

    virtual void init(pe_interface::World* phys_world);
    virtual void step(pe::Real dt);

    virtual void loadConfigFromJson(const nlohmann::json& j) {}
};

} // namespace pe_vehicle
