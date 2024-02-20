#include "collision_body.h"

namespace pe_phys_object {

    std::atomic<uint32_t> CollisionBody::_globalIdCounter(0);

    CollisionBody::CollisionBody():
        _global_id(++_globalIdCounter),
        _kinematic(false),
        _collision_shape(nullptr),
        _mass(1.),
        _inv_mass(1.),
        _inertia(pe::Matrix3::identity()),
        _inv_inertia(pe::Matrix3::identity()),
        _friction_coeff(0.5),
        _restitution_coeff(0.5),
        _linear_damping(0.),
        _angular_damping(0.),
        _transform(pe::Transform::identity()),
        _linear_velocity(pe::Vector3::zeros()),
        _angular_velocity(pe::Vector3::zeros()) {}

    void CollisionBody::computeAABB() {
        if (_collision_shape) {
            _collision_shape->getAABB(_transform, _aabb_min, _aabb_max);
        } else {
            _aabb_min = pe::Vector3::zeros();
            _aabb_max = pe::Vector3::zeros();
        }
    }

    pe::Real CollisionBody::getAABBScale() const {
        pe::Real scale = (_aabb_max - _aabb_min).norm();
        pe::Real diff = (_aabb_max + _aabb_min - _transform.getOrigin() * 2).norm();
        return scale + diff;
    }

    void CollisionBody::removeIgnoreCollisionId(uint32_t id) {
        for (uint32_t i = 0; i < _ignore_collision_ids.size(); i++) {
            if (_ignore_collision_ids[i] == id) {
                _ignore_collision_ids.erase(_ignore_collision_ids.begin() + i);
                return;
            }
        }
    }

    bool CollisionBody::isIgnoreCollisionId(uint32_t id) const {
        uint32_t size = _ignore_collision_ids.size();
        for (uint32_t i = 0; i < size; i++) {
            if (_ignore_collision_ids[i] == id) return true;
        }
        return false;
    }

} // namespace pe_phys_object