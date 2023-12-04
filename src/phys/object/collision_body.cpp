#include "collision_body.h"

using namespace pe_phys_object;

uint32_t CollisionBody::_globalIdCounter = 0;

void CollisionBody::applyForce(const pe::Vector3& force, const pe::Vector3& point, pe::Real dt) {
    if (_kinematic) return;
    _velocity += force * _inv_mass * dt;
    _angular_velocity += (point - _transform.getOrigin()).cross(force) * _inv_mass * dt;
}

void CollisionBody::updateTransform(pe::Real dt) {
    if (_kinematic) return;
    pe::Transform new_trans;

    new_trans.setTranslation(_velocity * dt);

    pe::Real angular_velocity_abs = _angular_velocity.norm();
    if (!PE_APPROX_EQUAL(angular_velocity_abs, 0)) {
        new_trans.setRotation(_angular_velocity.normalized(), angular_velocity_abs);
    }

    _transform = new_trans * _transform;
}
