#include "collision_object.h"

namespace pe_phys_object {

    std::atomic<uint32_t> CollisionObject::_globalIdCounter(0);

    void CollisionObject::setMass(pe::Real mass) {
        _mass = mass;
        _inv_mass = mass > 0 ? 1.0 / mass : 0;
    }

    void CollisionObject::setLocalInertia(const pe::Matrix3& local_inertia) {
        _local_inertia = local_inertia;
        _local_inv_inertia = local_inertia.inverse();
        updateWorldInertia();
    }

    void CollisionObject::updateWorldInertia() {
        _world_inertia = _transform.getBasis() * _local_inertia * _transform.getBasis().transposed();
        _world_inv_inertia = _transform.getBasis() * _local_inv_inertia * _transform.getBasis().transposed();
    }

    void CollisionObject::setTransform(const pe::Transform &transform) {
        _transform = transform;
        updateWorldInertia();
    }

    CollisionObject::CollisionObject():
        _global_id(++_globalIdCounter),
        _kinematic(false),
        _collision_shape(nullptr),
        _mass(1.),
        _inv_mass(1.),
        _local_inertia(pe::Matrix3::identity()),
        _local_inv_inertia(pe::Matrix3::identity()),
        _world_inertia(pe::Matrix3::identity()),
        _world_inv_inertia(pe::Matrix3::identity()),
        _friction_coeff(0.5),
        _restitution_coeff(0.5),
        _linear_damping(0.),
        _angular_damping(0.),
        _transform(pe::Transform::identity()),
        _linear_velocity(pe::Vector3::zeros()),
        _angular_velocity(pe::Vector3::zeros()) {}

    void CollisionObject::computeAABB() {
        if (_collision_shape) {
            _collision_shape->getAABB(_transform, _aabb_min, _aabb_max);
        } else {
            _aabb_min = pe::Vector3::zeros();
            _aabb_max = pe::Vector3::zeros();
        }
    }

    pe::Real CollisionObject::getAABBScale() const {
        pe::Real scale = (_aabb_max - _aabb_min).norm();
        pe::Real diff = (_aabb_max + _aabb_min - _transform.getOrigin() * 2).norm();
        return (scale + diff) * 0.5;
    }

    void CollisionObject::removeIgnoreCollisionId(uint32_t id) {
        for (auto it = _ignore_collision_ids.begin(); it != _ignore_collision_ids.end(); it++) {
            if (*it == id) {
                _ignore_collision_ids.erase(it);
                return;
            }
        }
    }

    bool CollisionObject::isIgnoreCollisionId(uint32_t id) const {
        for (auto i : _ignore_collision_ids) {
            if (i == id) {
                return true;
            }
        }
        return false;
    }

} // namespace pe_phys_object