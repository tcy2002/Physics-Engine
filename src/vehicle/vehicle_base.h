#pragma once

#include "physics/physics.h"
#include "interface/world.h"
#include "physics/object/rigidbody.h"
#include "vehicle/component/chassis.h"
#include "vehicle/component/wheel.h"
#include "vehicle/component/suspension.h"
#include "vehicle/component/engine.h"

namespace pe_vehicle {

/*
 * This class provides management for wheels.
 * Provided the wheel count and necessary info, wheels and suspensions are created automatically.
 * Control api is also provided for reacting with the internal engine
 */
class VehicleBase {
    COMMON_MEMBER_PTR_SET_GET(Chassis, chassis, Chassis)
    COMMON_MEMBER_PTR_SET_GET(Engine, engine, Engine)

    COMMON_MEMBER_SET_GET(int, gear, Gear)
    COMMON_MEMBER_SET_GET(pe::Real, throttle, Throttle)
    COMMON_MEMBER_SET_GET(pe::Real, brake, Brake)

    // The distance between the first wheel and the last wheel on one side
    COMMON_MEMBER_SET_GET(pe::Real, wheel_region_length, WheelRegionLength)
    // The distance between the left side and the right side
    COMMON_MEMBER_SET_GET(pe::Real, wheel_region_width, WheelRegionWidth)
    // The z offset of the wheel ragion's middle position 
    COMMON_MEMBER_SET_GET(pe::Vector3, wheel_region_offset, WheelRegionOffset)
    COMMON_MEMBER_SET_GET(pe::Real, wheel_width, WheelWidth)
    COMMON_MEMBER_SET_GET(int, wheel_count_per_side, WheelCountPerSide)

public:
    struct WheelInfo {
        AxleType type = AxleType::AT_NONE;
        bool motor;
        pe::Real radius;
        pe::Real mass;
        pe::Real friction;
        pe::Real anchor_offset_y; // how far is the anchor away from the original xOz plane
        pe::Real anchor_offset_z; // how far is the anchor away from the original z position
        pe::Real suspension_rest_length;
        pe::Real suspension_stiffness;
        pe::Real suspension_damping;
    };
protected:
    pe::Array<WheelInfo> _wheel_info;
    pe::Array<Wheel*> _wheels;
    pe::Array<Suspension*> _suspensions;

public:
    VehicleBase(): _wheel_region_length(PE_R(2.0)), _wheel_region_width(PE_R(2.0)), _wheel_width(PE_R(1.2)), _wheel_count_per_side(2),
                   _gear(0), _throttle(PE_R(0.0)), _brake(false) {}
    virtual ~VehicleBase();

    void setWheelInfo(int index, AxleType type, bool motor, pe::Real radius, pe::Real mass, pe::Real friction, pe::Real anchor_offset_y, 
                      pe::Real anchor_offset_z, pe::Real suspension_rest_length, pe::Real suspension_stiffness, pe::Real suspension_damping);

    void setSteerAngle(int wheel_index, pe::Real angle);

    virtual void init(pe_interface::World* phys_world);
    virtual void step(pe::Real dt);

    virtual void setTransform(const pe::Transform& trans);
    virtual pe::Transform getTransform() const;

    virtual void loadConfigFromJson(const nlohmann::json& j) {}
};

} // namespace pe_vehicle
