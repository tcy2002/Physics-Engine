#include "wheel_info.h"

namespace pe_phys_vehicle {

    pe::Real WheelInfo::getSuspensionRestLength() const {
        return m_suspensionRestLength1;
    }

    WheelInfo::WheelInfo(WheelInfoConstructionInfo& ci) {
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
        m_steering = pe::Real(0.);
        m_engineForce = pe::Real(0.);
        m_rotation = pe::Real(0.);
        m_deltaRotation = pe::Real(0.);
        m_brake = pe::Real(0.);
        m_rollInfluence = pe::Real(0.1);
        m_bIsFrontWheel = ci.m_bIsFrontWheel;
        m_maxSuspensionForce = ci.m_maxSuspensionForce;
    }

    void WheelInfo::updateWheel(pe_phys_object::RigidBody& chassis, RaycastInfo& raycastInfo) {
        (void)raycastInfo;

        if (m_raycastInfo.m_isInContact) {
            pe::Real project = m_raycastInfo.m_contactNormalWS.dot(m_raycastInfo.m_wheelDirectionWS);
            pe::Vector3 chassis_velocity_at_contactPoint;
            pe::Vector3 rel_pos = m_raycastInfo.m_contactPointWS - chassis.getTransform().getOrigin();
            chassis_velocity_at_contactPoint = chassis.getLinearVelocityAtLocalPoint(rel_pos);
            pe::Real projVel = m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);
            if (project >= pe::Real(-0.1)) {
                m_suspensionRelativeVelocity = pe::Real(0.0);
                m_clippedInvContactDotSuspension = pe::Real(1.0) / pe::Real(0.1);
            } else {
                pe::Real inv = pe::Real(-1.) / project;
                m_suspensionRelativeVelocity = projVel * inv;
                m_clippedInvContactDotSuspension = inv;
            }
        } else { // Not in contact : position wheel in a nice (rest length) position
            m_raycastInfo.m_suspensionLength = this->getSuspensionRestLength();
            m_suspensionRelativeVelocity = pe::Real(0.0);
            m_raycastInfo.m_contactNormalWS = -m_raycastInfo.m_wheelDirectionWS;
            m_clippedInvContactDotSuspension = pe::Real(1.0);
        }
    }

} // namespace pe_phys_vehicle
