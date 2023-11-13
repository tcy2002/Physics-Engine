#include "collision_body.h"

using namespace pe_phys_object;

uint32_t CollisionBody::_globalIdCounter = 0;

void CollisionBody::applyForce(const pe_cg::Vector3& force, const pe_cg::Vector3& point, real dt) {
    if (_kinematic) return;
    _velocity += force * _inv_mass * dt;
    _angular_velocity += (point - _transform.getOrigin()).cross(force) * _inv_mass * dt;
}

void CollisionBody::updateTransform(real dt) {
    if (_kinematic) return;
    pe_cg::Transform new_trans;

    new_trans.setTranslation(_velocity * dt);

    real angular_velocity_abs = _angular_velocity.norm();
    if (!APPROX_EQUAL(angular_velocity_abs, 0)) {
        new_trans.setRotation(_angular_velocity.normalized(), angular_velocity_abs);
    }

    _transform = new_trans * _transform;
}
