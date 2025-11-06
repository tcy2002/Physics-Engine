#pragma once

#include "vehicle/vehicle_base.h"
#include "track.h"

namespace pe_vehicle {

class TrackedVehicle : public VehicleBase {
    COMMON_MEMBER_SET_GET(pe::Real, left_throttle, LeftThrottle)
    COMMON_MEMBER_SET_GET(pe::Real, right_throttle, RightThrottle)

protected:
    Track* _left_track = nullptr;
    Track* _right_track = nullptr;

public:

public:
    TrackedVehicle(): VehicleBase(), _left_throttle(PE_R(0.0)), _right_throttle(PE_R(0.0)) {}
    virtual ~TrackedVehicle();

    virtual void init(pe_interface::World* phys_world) override;
    virtual void step(pe::Real dt) override;

    virtual void loadConfigFromJson(const nlohmann::json& j) {}
};

} // namespace pe_vehicle

