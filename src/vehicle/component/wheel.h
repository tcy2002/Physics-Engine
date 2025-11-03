#pragma once

#include "physics/physics.h"
#include "interface/world.h"
#include "physics/object/rigidbody.h"

namespace pe_vehicle {

enum class WheelType {
    WT_Sphere,
    WT_Capsule,
    WT_Cylinder
};

/*
 * A wheel is represented as a rigid body with a specific shape: sphere, capsule, or cylinder.
 * (cylinder shape is not implemented yet)
 * The wheel can be attached to a chassis via suspension.
 */
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

    pe_physics_object::RigidBody* getBody() const { return _wheel; }

    void init(pe_interface::World* phys_world);
    void step(pe::Real dt);

    const pe::Transform& getTransform() const { return _wheel->getTransform(); }
};

} // namespace pe_vehicle
