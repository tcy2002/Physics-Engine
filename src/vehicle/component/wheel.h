#pragma once

#include "physics/physics.h"
#include "physics/object/rigidbody.h"

namespace pe_vehicle {

enum class WheelType {
    WT_Sphere,
    WT_Capsule,
    WT_Cylinder
};

class Wheel {
    COMMON_MEMBER_GET(pe::Real, radius, Radius)
    COMMON_MEMBER_GET(pe::Real, width, Width)
    COMMON_MEMBER_GET(pe::Real, mass, Mass)
    COMMON_MEMBER_GET(pe::Real, friction, Friction)

protected:
    pe_physics_object::RigidBody* _wheel = nullptr;

public:
    Wheel() = delete;
    Wheel(WheelType type, pe::Real radius, pe::Real width, pe::Real mass, pe::Real friction);
    virtual ~Wheel();
};

} // namespace pe_vehicle
