#pragma once

#include "vehicle/vehicle_base.h"

namespace pe_vehicle {

class TrackedVehicle : public VehicleBase {
    COMMON_MEMBER_SET_GET(pe::Vector3, chassis_size, ChassisSize)
    COMMON_MEMBER_SET_GET(pe::Real, chassis_mass, ChassisMass)
    COMMON_MEMBER_SET_GET(pe::Real, wheel_region_length, WheelRegionLength)
    COMMON_MEMBER_SET_GET(pe::Real, wheel_region_offset, WheelRegionOffset)
    COMMON_MEMBER_SET_GET(int, wheel_count_per_side, WheelCountPerSide)
    COMMON_MEMBER_SET_GET(pe::Real, track_width, TrackWidth)
    COMMON_MEMBER_SET_GET(pe::Real, track_segment_mass, TrackSegmentMass)
    COMMON_MEMBER_SET_GET(int, track_segment_count_per_side, TrackSegmentCountPerSide)

protected:
    struct WheelInfo {
        pe::Real radius;
        pe::Real width;
        pe::Real anchor_offset;
        pe::Real suspension_rest_length;
    };
    pe::Array<WheelInfo> _wheel_info;

public:
    TrackedVehicle() = default;
    virtual ~TrackedVehicle();

    virtual void init(pe_interface::World* phys_world) override;
    virtual void step(pe::Real dt) override;

    virtual pe::Transform getTransform() const override;

    virtual void loadConfigFromJson(const nlohmann::json& j) {}
};

} // namespace pe_vehicle

