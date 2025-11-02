#include "ball_joint_constraint.h"

namespace pe_physics_constraint {

void BallJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
    auto& trans_a = _object_a->getTransform();
    auto& trans_b = _object_b->getTransform();
    _r_a = trans_a.getBasis() * _anchor_a;
    _r_b = trans_b.getBasis() * _anchor_b;

    pe::Matrix3 rx_a, rx_b;
    getSkewSymmetricMatrix(_r_a, rx_a);
    getSkewSymmetricMatrix(_r_b, rx_b);
    _jmj_inv = (pe::Matrix3::identity() * (_object_a->getInvMass() + _object_b->getInvMass()) +
        rx_a * _object_a->getWorldInvInertia() * rx_a.transposed() +
        rx_b * _object_b->getWorldInvInertia() * rx_b.transposed()).inverse();
    _rhs = _jmj_inv * (trans_a * _anchor_a - trans_b * _anchor_b) * (-param.kerp / param.dt);
}

void BallJointConstraint::iterateSequentialImpulse(int iter) {
    const pe::Vector3 vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
    const pe::Vector3 vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
    const pe::Vector3 tmp_impulse = _rhs - _jmj_inv * (vel_a - vel_b);
    _object_a->applyTempImpulse(_r_a, tmp_impulse);
    _object_b->applyTempImpulse(_r_b, -tmp_impulse);
}

} // namespace pe_physics_constraint
