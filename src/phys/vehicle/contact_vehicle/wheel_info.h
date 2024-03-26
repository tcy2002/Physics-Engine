#pragma once

#include "phys/object/rigidbody.h"

namespace pe_phys_vehicle {

    struct ContactWheelInfoConstructionInfo {
        pe::Vector3 m_chassisConnectionCS;
        pe::Vector3 m_wheelDirectionCS;
        pe::Vector3 m_wheelAxleCS;
        pe::Real m_suspensionRestLength;
        pe::Real m_maxSuspensionTravelCm;
        pe::Real m_wheelRadius;

        pe::Real m_suspensionStiffness;
        pe::Real m_wheelsDampingCompression;
        pe::Real m_wheelsDampingRelaxation;
        pe::Real m_frictionSlip;
        pe::Real m_maxSuspensionForce;
        bool m_bIsFrontWheel;
    };

    /// WheelInfo contains information per wheel about friction and suspension.
    struct ContactWheelInfo {
        struct ContactInfo {
            //set by raycaster
            pe::Real m_contactDepth;
            pe::Vector3 m_contactNormalWS;  //contact normal
            pe::Vector3 m_contactPointWS;   //raycast hit point
            pe::Real m_suspensionLength;
            pe::Vector3 m_hardPointWS;       //raycast starting point
            pe::Vector3 m_wheelDirectionWS;  //direction in world space
            pe::Vector3 m_wheelAxleWS;       // axle in world space
            pe::Vector3 m_wheelCenterWS;     // center of the wheel
            bool m_isInContact;
            void* m_groundObject;  //could be general void* ptr
        };

        ContactInfo m_contactInfo;

        pe::Transform m_worldTransform;

        pe::Vector3 m_chassisConnectionPointCS;  //const
        pe::Vector3 m_wheelDirectionCS;          //const
        pe::Vector3 m_wheelAxleCS;               // const or modified by steering
        pe::Real m_suspensionRestLength1;      //const
        pe::Real m_maxSuspensionTravelCm;
        pe::Real getSuspensionRestLength() const;
        pe::Real m_wheelsRadius;              //const
        pe::Real m_suspensionStiffness;       //const
        pe::Real m_wheelsDampingCompression;  //const
        pe::Real m_wheelsDampingRelaxation;   //const
        pe::Real m_frictionSlip;
        pe::Real m_steering;
        pe::Real m_rotation;
        pe::Real m_deltaRotation;
        pe::Real m_rollInfluence;
        pe::Real m_rollDamping;
        pe::Real m_maxSuspensionForce;

        pe::Real m_engineForce;

        pe::Real m_brake;

        bool m_bIsFrontWheel;

        void* m_clientInfo;  //can be used to store pointer to sync transforms...

        ContactWheelInfo() {}

        explicit ContactWheelInfo(ContactWheelInfoConstructionInfo& ci);

        pe::Real m_clippedInvContactDotSuspension;
        pe::Real m_suspensionRelativeVelocity;
        //calculated by suspension
        pe::Real m_wheelsSuspensionForce;
        pe::Real m_skidInfo;
    };

} // namespace pe_phys_vehicle
