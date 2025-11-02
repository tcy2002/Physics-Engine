#include "hinge_joint_constraint.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_constraint {

void HingeJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
    auto& trans_a = _object_a->getTransform();
    auto& trans_b = _object_b->getTransform();
    _r_a = trans_a.getBasis() * _anchor_a;
    _r_b = trans_b.getBasis() * _anchor_b;

    _w_axis_a = trans_a.getBasis() * _axis_a;
    _w_axis_b = trans_b.getBasis() * _axis_b;
    pe_physics_collision::ContactPoint::getOrthoUnits(_axis_a, _w_t_a[0], _w_t_a[1]);
    _w_t_a[0] = trans_a.getBasis() * _w_t_a[0];
    _w_t_a[1] = trans_a.getBasis() * _w_t_a[1];
    if (_use_limits) {
        pe_physics_collision::ContactPoint::getOrthoUnits(_axis_b, _w_t_b[0], _w_t_b[1]);
        _w_t_b[0] = trans_b.getBasis() * _w_t_b[0];
        _w_t_b[1] = trans_b.getBasis() * _w_t_b[1];
    }

    // ball jmj
    pe::Matrix3 rx_a, rx_b;
    getSkewSymmetricMatrix(_r_a, rx_a);
    getSkewSymmetricMatrix(_r_b, rx_b);
    _jmj_inv_ball = (pe::Matrix3::identity() * (_object_a->getInvMass() + _object_b->getInvMass()) +
        rx_a * _object_a->getWorldInvInertia() * rx_a.transposed() +
        rx_b * _object_b->getWorldInvInertia() * rx_b.transposed()).inverse();

    // ball rhs
    _rhs_ball = _jmj_inv_ball * (trans_a * _anchor_a - trans_b * _anchor_b) * (-param.kerp / param.dt);

    // hinge jmj
    const pe::Matrix3& inv_inertia_sum = _object_a->getWorldInvInertia() + _object_b->getWorldInvInertia();
    _jmj_inv_hinge[0] = PE_R(1.0) / _w_t_a[0].dot(inv_inertia_sum * _w_t_a[0]);
    _jmj_inv_hinge[1] = PE_R(1.0) / _w_t_a[1].dot(inv_inertia_sum * _w_t_a[1]);

    // hinge rhs
    const pe::Vector3& u = _w_axis_a.cross(_w_axis_b);
    // Using arc-sin here would be more accurate, but not necessary
    _rhs_hinge[0] = _jmj_inv_hinge[0] * (u.dot(_w_t_a[0]) * param.kerp_angle / param.dt);
    _rhs_hinge[1] = _jmj_inv_hinge[1] * (u.dot(_w_t_a[1]) * param.kerp_angle / param.dt);

    // motor and limits
    if (_use_motor || _use_limits) {
        _jmj_inv_motor_limit = PE_R(1.0) / _w_axis_a.dot(inv_inertia_sum * _w_axis_a);
    }
    if (_use_motor) {
        _rhs_motor = _jmj_inv_motor_limit * -_target_speed;
    }
    if (_use_limits) {
        const pe::Real& angle_cos = _w_t_a[0].dot(_w_t_b[0]);
        const pe::Real& angle_sin = _w_t_a[0].cross(_w_t_b[0]).dot(_w_axis_a);
        const pe::Real& angle = std::atan2(angle_sin, angle_cos);
        _limit_exceeded = false;
        if (angle < _min_angle) {
            _limit_exceeded = true;
            _rhs_limit = -_jmj_inv_motor_limit * (_min_angle - angle) * param.kerp_angle / param.dt;
        } else if (angle > _max_angle) {
            _limit_exceeded = true;
            _rhs_limit = -_jmj_inv_motor_limit * (_max_angle - angle) * param.kerp_angle / param.dt;
        }
    }
}

void HingeJointConstraint::iterateSequentialImpulse(int iter) {
    // position impulse
    const pe::Vector3& vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
    const pe::Vector3& vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
    const pe::Vector3& ball_impulse = _rhs_ball - _jmj_inv_ball * (vel_a - vel_b);
    _object_a->applyTempImpulse(_r_a, ball_impulse);
    _object_b->applyTempImpulse(_r_b, -ball_impulse);

    // rotation impulse
    const pe::Vector3& w_a = _object_a->getTempAngularVelocity();
    const pe::Vector3& w_b = _object_b->getTempAngularVelocity();
    const pe::Real& hinge_impulse0 = _rhs_hinge[0] - _jmj_inv_hinge[0] * _w_t_a[0].dot(w_a - w_b);
    const pe::Real& hinge_impulse1 = _rhs_hinge[1] - _jmj_inv_hinge[1] * _w_t_a[1].dot(w_a - w_b);
    const pe::Vector3 impulse_vector = _w_t_a[0] * hinge_impulse0 + _w_t_a[1] * hinge_impulse1;
    _object_a->applyTempAngularImpulse(impulse_vector);
    _object_b->applyTempAngularImpulse(-impulse_vector);

    // motor impulse
    if (_use_motor) {
        const pe::Real& motor_impulse = _rhs_motor - _jmj_inv_motor_limit * _w_axis_a.dot(w_a - w_b);
        _object_a->applyTempAngularImpulse(_w_axis_a * motor_impulse);
        _object_b->applyTempAngularImpulse(-_w_axis_a * motor_impulse);
    }

    // limit impulse
    if (_use_limits && _limit_exceeded) {
        const pe::Real& delta = _w_axis_a.dot(w_a - w_b);
        const pe::Real& limit_impulse = _rhs_limit - _jmj_inv_motor_limit * delta;
        _object_a->applyTempAngularImpulse(_w_axis_a * limit_impulse);
        _object_b->applyTempAngularImpulse(-_w_axis_a * limit_impulse);
    }
}

} // namespace pe_physics_constraint
