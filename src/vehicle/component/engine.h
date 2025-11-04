#pragma once

#include "physics/physics.h"
#include "json/json.hpp"

namespace pe_vehicle {

/*
 * An engine controls the power output of a vehicle.
 * It can be connected to the drivetrain to provide torque to the wheels.
 * Each gear has a transmission ratio, so the torque can be calculated like:
 *     output_torque = engine_torque_curve(wheel_rpm * gear_ratio) * gear_ratio
 *
 * Gear Count: number of forward gears (not including R and P)
 */
class Engine {
protected:
    COMMON_MEMBER_GET(int, gear_count, GearCount)
    COMMON_MEMBER_GET(int, gear, Gear)
    pe::Array<pe::Real> _gear_ratios;
    pe::Array<pe::KV<pe::Real, pe::Real>> _torque_curve; // (rpm, torque)

public:
    Engine() = delete;
    explicit Engine(int gear_count = 6): _gear_count(gear_count), _gear(0) { _gear_ratios.resize(gear_count + 1, 0); }

    /*
     * Set the current gear.
     * gear: 0: P, -1: R, 1~: D1~
     */
    void setGear(int gear);

    /* Set the gear ratio for a specific gear.
     * gear_index: 0: R, 1~: D1~
     */
    void setGearRatio(int gear_index, pe::Real ratio);
    pe::Real getGearRatio(int gear_index) const;

    void addTorquePoint(pe::Real rpm, pe::Real torque);
    void clearTorqueCurve();
    const pe::Array<pe::KV<pe::Real, pe::Real>>& getTorqueCurve() const { return _torque_curve; }
    pe::Real getTorque(pe::Real wheel_rpm) const;

    virtual void loadConfigFromJson(const nlohmann::json& j);
};

} // namespace pe_vehicle
