#pragma once

#include "rigid/phys_general.h"

namespace pe_phys_vehicle {

    /// VehicleRaycaster provides interface for between vehicle simulation and raycasting
    struct VehicleRaycaster {
        virtual ~VehicleRaycaster() {}
        struct VehicleRaycasterResult {
            VehicleRaycasterResult(): m_distFraction(PE_R(-1.)) {};
            pe::Vector3 m_hitPointInWorld;
            pe::Vector3 m_hitNormalInWorld;
            pe::Real m_distFraction;
        };

        virtual void* castRay(uint32_t rigid_idx, const pe::Uint32HashList& excludeIds,
                              const pe::Vector3& from, const pe::Vector3& direction,
                              pe::Real length, VehicleRaycasterResult& result) = 0;
    };

} // namespace pe_phys_vehicle
