#pragma once

#include "intf/world.h"
#include "vehicle_raycaster.h"
#include "wheel_info.h"
#include "phys/raycast/raycast.h"
#include "utils/jacobian_entry.h"

namespace pe_phys_vehicle {

    ///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
    class RaycastVehicle
    {
        pe::Array<pe::Vector3> m_forwardWS;
        pe::Array<pe::Vector3> m_axle;
        pe::Array<pe::Real> m_forwardImpulse;
        pe::Array<pe::Real> m_sideImpulse;

        ///backwards compatibility
        int m_userConstraintType;
        int m_userConstraintId;

    public:
        class VehicleTuning {
        public:
            VehicleTuning();
            pe::Real m_suspensionStiffness;
            pe::Real m_suspensionCompression;
            pe::Real m_suspensionDamping;
            pe::Real m_maxSuspensionTravelCm;
            pe::Real m_frictionSlip;
            pe::Real m_maxSuspensionForce;
        };

    private:
        VehicleRaycaster* m_vehicleRaycaster;
        pe::Real m_pitchControl;
        pe::Real m_steeringValue;
        pe::Real m_currentVehicleSpeedKmHour;

        pe_phys_object::RigidBody* m_chassisBody;

        int m_indexRightAxis;
        int m_indexUpAxis;
        int m_indexForwardAxis;

        void defaultInit(const VehicleTuning& tuning);
        static pe_phys_object::RigidBody& getFixedBody();

    public:
        //constructor to create a car from an existing rigidbody
        RaycastVehicle(const VehicleTuning& tuning, pe_phys_object::RigidBody* chassis, VehicleRaycaster* raycaster);
        virtual ~RaycastVehicle() {}

        const pe::Transform& getChassisWorldTransform() const;

        pe::Real rayCast(WheelInfo& wheel);

        pe::Real getSteeringValue(int wheel) const;
        void setSteeringValue(pe::Real steering, int wheel);
        void applyEngineForce(pe::Real force, int wheel);

        pe::Array<WheelInfo> m_wheelInfo;
        const WheelInfo& getWheelInfo(int index) const;
        WheelInfo& getWheelInfo(int index);
        inline int getNumWheels() const { return int(m_wheelInfo.size()); }
        void updateWheelTransform(int wheelIndex, bool interpolatedTransform = true);
        const pe::Transform& getWheelTransformWS(int wheelIndex) const;
        void updateWheelTransformsWS(WheelInfo& wheel, bool interpolatedTransform = true) const;
        WheelInfo& addWheel(const pe::Vector3& connectionPointCS0, const pe::Vector3& wheelDirectionCS0,
                            const pe::Vector3& wheelAxleCS, pe::Real suspensionRestLength, pe::Real wheelRadius,
                            const VehicleTuning& tuning, bool isFrontWheel);

        void resetSuspension();
        void setBrake(pe::Real brake, int wheelIndex);
        void setPitchControl(pe::Real pitch) { m_pitchControl = pitch; }

        virtual void updateAction(pe::Real step) { updateVehicle(step); }
        virtual void updateVehicle(pe::Real step);
        void updateSuspension(pe::Real deltaTime);
        virtual void updateFriction(pe::Real timeStep);

        inline pe_phys_object::RigidBody* getRigidBody() { return m_chassisBody; }
        const pe_phys_object::RigidBody* getRigidBody() const { return m_chassisBody; }

        virtual void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex);
        inline int getRightAxis() const { return m_indexRightAxis; }
        inline int getUpAxis() const { return m_indexUpAxis; }
        inline int getForwardAxis() const { return m_indexForwardAxis; }
        pe::Vector3 getForwardVector() const; ///Worldspace forward vector

        ///Velocity of vehicle (positive if velocity vector has same direction as forward vector)
        pe::Real getCurrentSpeedKmHour() const { return m_currentVehicleSpeedKmHour; }

        ///backwards compatibility
        int getUserConstraintType() const { return m_userConstraintType; }
        void setUserConstraintType(int userConstraintType) { m_userConstraintType = userConstraintType; };
        void setUserConstraintId(int uid) { m_userConstraintId = uid; }
        int getUserConstraintId() const { return m_userConstraintId; }
    };

    class DefaultVehicleRaycaster : public VehicleRaycaster {
    private:
        pe_intf::World* m_world;

    public:
        explicit DefaultVehicleRaycaster(pe_intf::World* world): m_world(world) {}

        virtual void* castRay(uint32_t rigid_idx, const pe::Vector3& from, const pe::Vector3& direction,
                              pe::Real length, VehicleRaycasterResult& result) override;
    };

} // namespace pe_phys_vehicle
