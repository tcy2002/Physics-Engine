#include <algorithm>
#include "rigidbody.h"

// style-checked
namespace pe_phys_object {

    std::atomic<uint32_t> RigidBody::_globalIdCounter(0);

    void RigidBody::setCollisionShape(pe_phys_shape::Shape *shape) {
        if (shape == nullptr) {
            throw std::runtime_error("Collision shape cannot be null.");
        }
        _collision_shape = shape;
        _local_inertia = _collision_shape->getLocalInertia() * _mass;
        if (_mass > PE_EPS) {
            _local_inv_inertia = _local_inertia.inverse();
        } else {
            _local_inv_inertia = pe::Matrix3::Zero();
        }
        updateWorldInertia();
    }

    void RigidBody::setMass(pe::Real mass) {
        _mass = mass;
        _inv_mass = mass > PE_EPS ? PE_R(1.0) / mass : 0;
        if (_collision_shape != nullptr) {
            _local_inertia = _collision_shape->getLocalInertia() * mass;
        } else if (_mass > PE_EPS) {
            _local_inertia = pe::Matrix3::Identity() * mass;
        } else {
            _local_inertia = pe::Matrix3::Zero();
        }
        if (_mass > PE_EPS) {
            _local_inv_inertia = _local_inertia.inverse();
        } else {
            _local_inv_inertia = pe::Matrix3::Zero();
        }
        updateWorldInertia();
    }

    void RigidBody::updateWorldInertia() {
        _world_inertia = _transform.getBasis() * _local_inertia * _transform.getBasis().transpose();
        _world_inv_inertia = _transform.getBasis() * _local_inv_inertia * _transform.getBasis().transpose();
    }

    void RigidBody::setTransform(const pe::Transform &transform) {
        _transform = transform;
        updateWorldInertia();
    }

    const pe::Vector3& RigidBody::getTempLinearVelocity() {
//        std::lock_guard<std::mutex> lock(_temp_linear_velocity_mutex);
        return _temp_linear_velocity;
    }

    const pe::Vector3& RigidBody::getTempAngularVelocity() {
//        std::lock_guard<std::mutex> lock(_temp_linear_velocity_mutex);
        return _temp_angular_velocity;
    }

    void RigidBody::setTempLinearVelocity(const pe::Vector3 &v) {
//        std::lock_guard<std::mutex> lock(_temp_linear_velocity_mutex);
        if (isKinematic()) return;
        _temp_linear_velocity = v;
    }

    void RigidBody::setTempAngularVelocity(const pe::Vector3 &v) {
//        std::lock_guard<std::mutex> lock(_temp_angular_velocity_mutex);
        if (isKinematic()) return;
        _temp_angular_velocity = v;
    }

    RigidBody::RigidBody():
            _global_id(++_globalIdCounter),
            _kinematic(false),
            _ignore_collision(false),
            _mass(1.),
            _inv_mass(1.),
            _local_inertia(pe::Matrix3::Identity()),
            _local_inv_inertia(pe::Matrix3::Identity()),
            _world_inertia(pe::Matrix3::Identity()),
            _world_inv_inertia(pe::Matrix3::Identity()),
            _life_time(PE_REAL_MAX),
            _last_time(0),
            _friction_coeff(0.5),
            _restitution_coeff(0.5),
            _linear_damping(0.0),
            _angular_damping(0.0),
            _transform(pe::Transform::Identity()),
            _linear_velocity(pe::Vector3::Zero()),
            _angular_velocity(pe::Vector3::Zero()),
            _force(pe::Vector3::Zero()),
            _torque(pe::Vector3::Zero()),
            _temp_linear_velocity(pe::Vector3::Zero()),
            _temp_angular_velocity(pe::Vector3::Zero()),
            _aabb_min(pe::Vector3::Zero()),
            _aabb_max(pe::Vector3::Zero()),
            _sleep(false),
            _sleep_time(0),
            _static_count(0),
            _dynamic_count(0) {
        updateWorldInertia();
    }

    pe::Real RigidBody::getAABBScale() const {
        const pe::Real scale = (_aabb_max - _aabb_min).norm();
        const pe::Real diff = (_aabb_max + _aabb_min - _transform.getOrigin() * 2).norm();
        return (scale + diff) * PE_R(0.5);
    }

    void RigidBody::removeIgnoreCollisionId(uint32_t id) {
        for (auto it = _ignore_collision_ids.begin(); it != _ignore_collision_ids.end(); it++) {
            if (*it == id) {
                _ignore_collision_ids.erase(it);
                return;
            }
        }
    }

    bool RigidBody::ignoreCollisionId(uint32_t id) const {
        return _ignore_collision_ids.find(id) != _ignore_collision_ids.end();
    }

    pe::Vector3 RigidBody::getLinearVelocityAtLocalPoint(const pe::Vector3& local_p) const {
        return _linear_velocity + _angular_velocity.cross(local_p);
    }

    pe::Real RigidBody::getKineticEnergy() const {
        return (_linear_velocity.dot(_mass * _linear_velocity) +
            _angular_velocity.dot(_local_inertia * _angular_velocity)) * PE_R(0.5);
    }

    pe::Real RigidBody::getImpulseDenominator(const pe::Vector3& world_point, const pe::Vector3& world_normal) const {
        const pe::Vector3 r = world_point - _transform.getOrigin();
        const pe::Vector3 c = r.cross(world_normal);
        const pe::Vector3 vec = (_world_inv_inertia * c).cross(r);
        return _inv_mass + world_normal.dot(vec);
    }

    void RigidBody::syncTempVelocity() {
        if (isKinematic()) return;
        _linear_velocity = _temp_linear_velocity;
        _angular_velocity = _temp_angular_velocity;
    }

    void RigidBody::clearTempVelocity() {
        if (isKinematic()) return;
        _temp_linear_velocity = _linear_velocity;
        _temp_angular_velocity = _angular_velocity;
    }

    void RigidBody::applyTempImpulse(const pe::Vector3& world_rel_vec, const pe::Vector3& impulse) {
        if (isKinematic()) return;
        _temp_linear_velocity_mutex.lock();
        _temp_linear_velocity += impulse * _inv_mass;
        _temp_linear_velocity_mutex.unlock();
        _temp_angular_velocity_mutex.lock();
        _temp_angular_velocity += _world_inv_inertia * world_rel_vec.cross(impulse);
        _temp_angular_velocity_mutex.unlock();
    }

    void RigidBody::applyImpulse(const pe::Vector3& world_rel_vec, const pe::Vector3& impulse) {
        if (isKinematic()) return;
        _linear_velocity += impulse * _inv_mass;
        _angular_velocity += _world_inv_inertia * world_rel_vec.cross(impulse);
    }

    void RigidBody::addForce(const pe::Vector3 &world_point, const pe::Vector3 &force) {
        _force += force;
        _torque += (world_point - _transform.getOrigin()).cross(force);
    }

    void RigidBody::applyForce(pe::Real dt) {
        if (isKinematic()) return;
        _linear_velocity += _force * _inv_mass * dt;
        _angular_velocity += _world_inv_inertia * (_torque * dt);
        _force = pe::Vector3::Zero();
        _torque = pe::Vector3::Zero();
    }

    void RigidBody::applyDamping(pe::Real dt) {
        if (isKinematic()) return;
        _linear_velocity *= PE_POW(PE_R(1.0) - _linear_damping, dt);
        _angular_velocity *= PE_POW(PE_R(1.0) - _angular_damping, dt);
    }

    bool RigidBody::step(pe::Real dt) {
        if (isKinematic()) {
            if (_collision_shape != nullptr) {
                _collision_shape->getAABB(_transform, _aabb_min, _aabb_max);
            }
            return true;
        }

        _last_time += dt;
        if (_last_time >= _life_time) {
            return false;
        }

        _transform.setOrigin(_transform.getOrigin() + _linear_velocity * dt);
#   ifdef PE_USE_QUATERNION
        auto q = pe::Quaternion(_transform.getBasis());
        const pe::Vector3 dr = _angular_velocity * dt * PE_R(0.5);
        const auto dq = pe::Quaternion(0, dr.x(), dr.y(), dr.z()) * q;
        q = pe::Quaternion(q.w() + dq.w(), q.x() + dq.x(), q.y() + dq.y(), q.z() + dq.z());
        q.normalize();
        _transform.setBasis(q.toRotationMatrix());
#   else
        pe::Matrix3 rot;
        const pe::Real angle_speed = _angular_velocity.norm();
        if (angle_speed > PE_EPS) {
            rot.setRotation(_angular_velocity.normalized(), angle_speed * dt);
            _transform.setBasis(rot * _transform.getBasis());
        }
#   endif
        updateWorldInertia();

        if (_collision_shape != nullptr) {
            _collision_shape->getAABB(_transform, _aabb_min, _aabb_max);
        }

        return true;
    }

} // namespace pe_phys_object