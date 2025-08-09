#pragma once

#include "intf/world.h"
#include "vehicle_raycaster.h"
#include "rigid/raycast/raycast_solver/default_raycast_solver.h"
#include "wheel_info.h"

namespace pe_phys_vehicle {

    ///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
    class RaycastVehicle {
        pe::Array<pe::Vector3> m_forwardWS;
        pe::Array<pe::Vector3> m_axle;
        pe::Array<pe::Real> m_forwardImpulse;
        pe::Array<pe::Real> m_sideImpulse;
        pe::Uint32HashList m_raycastExcludeIds;

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
        pe::Real m_currentVehicleSpeedKmHour;

        pe_phys_object::RigidBody* m_chassisBody;

        int m_indexRightAxis;
        int m_indexUpAxis;
        int m_indexForwardAxis;

        int m_numWheelsOnGround;

        static pe_phys_object::RigidBody& getFixedBody();

    public:
        //constructor to create a car from an existing rigidbody
        RaycastVehicle(const VehicleTuning& tuning, pe_phys_object::RigidBody* chassis, VehicleRaycaster* raycaster);
        virtual ~RaycastVehicle() {}

        const pe::Transform& getChassisWorldTransform() const;

        void addRaycastExcludeId(uint32_t id) { m_raycastExcludeIds.push_back(id); }
        void removeRaycastExcludeId(uint32_t id) { m_raycastExcludeIds.erase(m_raycastExcludeIds.find(id)); }
        pe::Real rayCast(RaycastWheelInfo& wheel);

        pe::Real getSteeringValue(int wheel) const;
        void setSteeringValue(pe::Real steering, int wheel);
        void applyEngineForce(pe::Real force, int wheel);

        pe::Array<RaycastWheelInfo> m_wheelInfo;
        const RaycastWheelInfo& getWheelInfo(int index) const;
        RaycastWheelInfo& getWheelInfo(int index);
        inline int getNumWheels() const { return int(m_wheelInfo.size()); }
        inline int getNumWheelsOnGround() const { return m_numWheelsOnGround; }
        void updateWheelTransform(int wheelIndex);
        const pe::Transform& getWheelTransformWS(int wheelIndex) const;
        void updateWheelTransformsWS(RaycastWheelInfo& wheel) const;
        RaycastWheelInfo& addWheel(const pe::Vector3& connectionPointCS0, const pe::Vector3& wheelDirectionCS0,
                                   const pe::Vector3& wheelAxleCS, pe::Real suspensionRestLength, pe::Real wheelRadius,
                                   const VehicleTuning& tuning, bool isFrontWheel);

        void resetSuspension();
        void setBrake(pe::Real brake, int wheelIndex);

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
        pe::Vector3 getUpVector() const; ///Worldspace up vector

        ///Velocity of vehicle (positive if velocity vector has same direction as forward vector)
        pe::Real getCurrentSpeedKmHour() const { return m_currentVehicleSpeedKmHour; }
    };

    class DefaultVehicleRaycaster : public VehicleRaycaster {
    private:
        pe_intf::World* m_world;
        pe_phys_raycast::DefaultRaycastSolver m_raySolver;

    public:
        explicit DefaultVehicleRaycaster(pe_intf::World* world): m_world(world) {}

        virtual void* castRay(uint32_t rigid_idx, const pe::Uint32HashList& excludeIds,
                              const pe::Vector3& from, const pe::Vector3& direction,
                              pe::Real length, VehicleRaycasterResult& result) override;
    };

} // namespace pe_phys_vehicle
