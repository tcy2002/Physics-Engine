#include "rigidbody.h"

namespace pe_phys_object {

    void RigidBody::updateWorldInvInertia() {
        _world_inv_inertia = _transform.getBasis() * _inv_inertia * _transform.getBasis().transposed();
    }

    RigidBody::RigidBody():
        _force(pe::Vector3::zeros()),
        _torque(pe::Vector3::zeros()),
        _gravity(pe::Vector3::zeros()),
        _temp_linear_velocity(pe::Vector3::zeros()),
        _temp_angular_velocity(pe::Vector3::zeros()),
        _penetration_linear_velocity(pe::Vector3::zeros()),
        _penetration_angular_velocity(pe::Vector3::zeros()) {
        updateWorldInvInertia();
    }

    pe::Vector3 RigidBody::getLinearVelocity(const pe::Vector3& point) const {
        return _linear_velocity + _angular_velocity.cross(point - _transform.getOrigin());
    }

    pe::Real RigidBody::getKineticEnergy() {
        return 0.5 * _mass * _linear_velocity.dot(_linear_velocity) +
            0.5 * _angular_velocity.dot(_inertia * _angular_velocity);
    }

    pe::Real RigidBody::getImpulseDenominator(const pe::Vector3& point, const pe::Vector3& normal) const {
        pe::Vector3 r = point - _transform.getOrigin();
        pe::Vector3 c = r.cross(normal);
        pe::Vector3 vec = (_world_inv_inertia * c).cross(r);
        return 1.0 / _mass + normal.dot(vec);
    }

    void RigidBody::syncTempVelocity() {
        _linear_velocity = _temp_linear_velocity;
        _angular_velocity = _temp_angular_velocity;
    }

    void RigidBody::clearTempVelocity() {
        _temp_linear_velocity = _linear_velocity;
        _temp_angular_velocity = _angular_velocity;
        _penetration_linear_velocity = pe::Vector3::zeros();
        _penetration_angular_velocity = pe::Vector3::zeros();
    }

    void RigidBody::applyTempImpulse(const pe::Vector3& point, const pe::Vector3& impulse) {
        if (isKinematic()) return;
        _temp_linear_velocity += impulse / _mass;
        _temp_angular_velocity += _world_inv_inertia * (point - _transform.getOrigin()).cross(impulse);
    }

    void RigidBody::applyImpulse(const pe::Vector3& point, const pe::Vector3& impulse) {
        if (isKinematic()) return;
        _linear_velocity += impulse / _mass;
        _angular_velocity += _world_inv_inertia * (point - _transform.getOrigin()).cross(impulse);
    }

    void RigidBody::applyPenetrationImpulse(const pe::Vector3 &point, const pe::Vector3 &impulse) {
        if (isKinematic()) return;
        _penetration_linear_velocity += impulse / _mass;
        _penetration_angular_velocity += _world_inv_inertia * (point - _transform.getOrigin()).cross(impulse);
    }

    void RigidBody::addForce(const pe::Vector3 &point, const pe::Vector3 &force) {
        _force += force;
        _torque += (point - _transform.getOrigin()).cross(force);
    }

    void RigidBody::applyDamping(pe::Real dt) {
        _linear_velocity *= std::pow(1.0 - _linear_damping, dt);
        _angular_velocity *= std::pow(1.0 - _angular_damping, dt);
    }

    void RigidBody::penetrationStep(pe::Real dt) {
        _transform.setOrigin(_transform.getOrigin() + _penetration_linear_velocity * dt);
        pe::Matrix3 rot;
        rot.setRotation(_penetration_angular_velocity.normalized(),
                        _penetration_angular_velocity.norm() * dt);
        _transform.setBasis(_transform.getBasis() * rot);
        _penetration_linear_velocity = pe::Vector3::zeros();
        _penetration_angular_velocity = pe::Vector3::zeros();
        updateWorldInvInertia();
    }

    void RigidBody::step(pe::Real dt) {
        _transform.setOrigin(_transform.getOrigin() + _linear_velocity * dt);
        pe::Matrix3 rot;
        rot.setRotation(_angular_velocity.normalized(), _angular_velocity.norm() * dt);
        _transform.setBasis(_transform.getBasis() * rot);
        updateWorldInvInertia();
    }

} // namespace pe_phys_object