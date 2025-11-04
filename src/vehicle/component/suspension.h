#pragma once

#include "wheel.h"
#include "physics/physics.h"
#include "interface/world.h"
#include "physics/object/rigidbody.h"
#include "physics/constraint/constraint/six_dof_constraint.h"
#include "json/json.hpp"

namespace pe_vehicle {

class Wheel;
class Chassis;

enum class AxleType {
    AT_NONE,
    AT_FRONT, // steering axle
    AT_REAR,  // non-steering axle
};

/*
    * A suspension connects a wheel to the chassis.
    * It is represented by several parameters: rest length, stiffness, damping, and anchor point on the chassis.
    * The default forward direction in world space is (0, 0, -1)
    * The default wheel axle in wheel's local space is (0, 1, 0), i.e., you must give the wheel like a vertical cylinder.
    * The anchor point on the wheel is always at the wheel center.
    * The axis of the suspension in chassis' local space is always (0, -1, 0)
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
    Suspension(Chassis* chassis, Wheel* wheel, AxleType axle_type,
               pe::Real rest_length, pe::Real stiffness, pe::Real damping, const pe::Vector3& anchor_chassis);
    virtual ~Suspension();

    void setSteerAngle(pe::Real angle);
    void releaseSteerAngle();
    void setTargetWheelSpeed(pe::Real speed);

    virtual void init(pe_interface::World* phys_world);
    virtual void step(pe::Real dt);

    virtual void loadConfigFromJson(const nlohmann::json& j) {}
};

} // namespace pe_vehicle
