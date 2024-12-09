#include "wheel_info.h"

namespace pe_phys_vehicle {

    pe::Real ContactWheelInfo::getSuspensionRestLength() const {
        return m_suspensionRestLength1;
    }

    ContactWheelInfo::ContactWheelInfo(ContactWheelInfoConstructionInfo& ci) {
        m_suspensionRestLength1 = ci.m_suspensionRestLength;
        m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

        m_wheelsRadius = ci.m_wheelRadius;
        m_suspensionStiffness = ci.m_suspensionStiffness;
        m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
        m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
        m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
        m_wheelDirectionCS = ci.m_wheelDirectionCS;
        m_wheelAxleCS = ci.m_wheelAxleCS;
        m_frictionSlip = ci.m_frictionSlip;
        m_steering = PE_R(0.);
        m_engineForce = PE_R(0.);
        m_rotation = PE_R(0.);
        m_deltaRotation = PE_R(0.);
        m_brake = PE_R(0.);
        m_rollInfluence = PE_R(0.1);
        m_bIsFrontWheel = ci.m_bIsFrontWheel;
        m_maxSuspensionForce = ci.m_maxSuspensionForce;
    }

} // namespace pe_phys_vehicle
