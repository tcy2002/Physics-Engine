#pragma once

#include "wheel.h"
#include "physics/physics.h"
#include "interface/world.h"
#include "physics/object/rigidbody.h"
#include "physics/constraint/constraint/six_dof_constraint.h"

namespace pe_vehicle {

class Wheel;
class Chassis;

/*
 * A suspension connects a wheel to the chassis.
 * It is represented by several parameters: rest length, stiffness, damping, and anchor point on the chassis.
 * The anchor point on the wheel is always at the wheel center.
 * The axis of the suspension is always along the chassis local up direction.
 */
class Suspension {
    COMMON_MEMBER_GET(pe::Real, rest_length, RestLength)
    COMMON_MEMBER_GET(pe::Real, stiffness, Stiffness)
    COMMON_MEMBER_GET(pe::Real, damping, Damping)
    COMMON_MEMBER_GET(pe::Vector3, anchor_chassis, AnchorChassis)

protected:
    pe_physics_constraint::SixDofConstraint* _constraint = nullptr;

public:
    Suspension() = delete;
    Suspension(Chassis* chassis, Wheel* wheel, pe::Real rest_length, pe::Real stiffness,
               pe::Real damping, const pe::Vector3& anchor_chassis);
    virtual ~Suspension();

    void init(pe_interface::World* phys_world);
    void step(pe::Real dt);
};

} // namespace pe_vehicle
