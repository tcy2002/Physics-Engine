#pragma once

#include "collision_body.h"

namespace pe_phys_object {

    class RigidBody : public CollisionBody {
    private:
        pe::Matrix3 _world_inv_inertia;
        void updateWorldInvInertia();

    protected:
        COMMON_MEMBER_SET_GET(pe::Vector3, force, Force)
        COMMON_MEMBER_SET_GET(pe::Vector3, torque, Torque)
        COMMON_MEMBER_SET_GET(pe::Vector3, gravity, Gravity)

        COMMON_MEMBER_SET_GET(pe::Vector3, temp_linear_velocity, TempLinearVelocity)
        COMMON_MEMBER_SET_GET(pe::Vector3, temp_angular_velocity, TempAngularVelocity)
        COMMON_MEMBER_SET_GET(pe::Vector3, penetration_linear_velocity, PenetrationLinearVelocity)
        COMMON_MEMBER_SET_GET(pe::Vector3, penetration_angular_velocity, PenetrationAngularVelocity)

    public:
        bool isDeformable() const override { return false; }
        virtual bool isBreakable() const { return false; }

        RigidBody();

        pe::Vector3 getLinearMomentum() const { return _linear_velocity * _mass; }
        pe::Vector3 getAngularMomentum() const { return _inertia * _angular_velocity; }
        pe::Vector3 getLinearVelocityAt(const pe::Vector3& world_point) const;
        pe::Real getKineticEnergy();
        pe::Real getImpulseDenominator(const pe::Vector3& world_point, const pe::Vector3& world_normal) const;

        void syncTempVelocity();
        void clearTempVelocity();

        void applyTempImpulse(const pe::Vector3& point, const pe::Vector3& impulse);
        void applyImpulse(const pe::Vector3& point, const pe::Vector3& impulse);
        void applyPenetrationImpulse(const pe::Vector3& point, const pe::Vector3& impulse);

        void addForce(const pe::Vector3& point, const pe::Vector3& force);
        void addTorque(const pe::Vector3& torque) { _torque += torque; }

        void applyDamping(pe::Real dt);

        void penetrationStep(pe::Real dt);
        void step(pe::Real dt);
    };

} // namespace pe_phys_object