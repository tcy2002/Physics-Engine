#include "ball_joint_constraint.h"

namespace pe_physics_constraint {

void BallJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
    if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

    auto& trans_a = _object_a->getTransform();
    auto& trans_b = _object_b->getTransform();
    _r_a = trans_a.getBasis() * _anchor_a;
    _r_b = trans_b.getBasis() * _anchor_b;

    const pe::Real inv_mass_a = _object_a->isKinematic() ? PE_R(0.0) : _object_a->getInvMass();
    const pe::Real inv_mass_b = _object_b->isKinematic() ? PE_R(0.0) : _object_b->getInvMass();
    const pe::Matrix3& inv_inertia_a = _object_a->isKinematic() ? pe::Matrix3::zeros() : _object_a->getWorldInvInertia();
    const pe::Matrix3& inv_inertia_b = _object_b->isKinematic() ? pe::Matrix3::zeros() : _object_b->getWorldInvInertia();
    const pe::Real inv_mass_sum = inv_mass_a + inv_mass_b;

    pe::Matrix3 rx_a, rx_b;
    getSkewSymmetricMatrix(_r_a, rx_a);
    getSkewSymmetricMatrix(_r_b, rx_b);
    _jmj_inv = (pe::Matrix3::identity() * inv_mass_sum +
        rx_a * inv_inertia_a * rx_a.transposed() +
        rx_b * inv_inertia_b * rx_b.transposed()).inverse();
    _rhs = _jmj_inv * (trans_a * _anchor_a - trans_b * _anchor_b) * (-param.kerp / param.dt);
}

void BallJointConstraint::iterateSequentialImpulse(int iter) {
    if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

    const pe::Vector3 vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
    const pe::Vector3 vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
    const pe::Vector3 tmp_impulse = _rhs - _jmj_inv * (vel_a - vel_b);
    _object_a->applyTempImpulse(_r_a, tmp_impulse);
    _object_b->applyTempImpulse(_r_b, -tmp_impulse);
}

} // namespace pe_physics_constraint
