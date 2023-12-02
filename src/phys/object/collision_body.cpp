#include "collision_body.h"

using namespace pe_phys_object;

uint32_t CollisionBody::_globalIdCounter = 0;

void CollisionBody::applyForce(const pe_common::Vector3& force, const pe_common::Vector3& point, PEReal dt) {
    if (_kinematic) return;
    _velocity += force * _inv_mass * dt;
    _angular_velocity += (point - _transform.getOrigin()).cross(force) * _inv_mass * dt;
}

void CollisionBody::updateTransform(PEReal dt) {
    if (_kinematic) return;
    pe_common::Transform new_trans;

    new_trans.setTranslation(_velocity * dt);

    PEReal angular_velocity_abs = _angular_velocity.norm();
    if (!PE_APPROX_EQUAL(angular_velocity_abs, 0)) {
        new_trans.setRotation(_angular_velocity.normalized(), angular_velocity_abs);
    }

    _transform = new_trans * _transform;
}
