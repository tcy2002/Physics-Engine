#include "hinge_joint_constraint.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_constraint {

void HingeJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
    if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

    auto& trans_a = _object_a->getTransform();
    auto& trans_b = _object_b->getTransform();
    _r_a = trans_a.getBasis() * _anchor_a;
    _r_b = trans_b.getBasis() * _anchor_b;

    _w_axis_a = trans_a.getBasis() * _axis_a;
    _w_axis_b = trans_b.getBasis() * _axis_b;
    pe_physics_collision::ContactPoint::getOrthoUnits(_axis_a, _w_t_a[0], _w_t_a[1]);
    pe_physics_collision::ContactPoint::getOrthoUnits(_axis_b, _w_t_b[0], _w_t_b[1]);
    _w_t_a[0] = trans_a.getBasis() * _w_t_a[0];
    _w_t_a[1] = trans_a.getBasis() * _w_t_a[1];
    _w_t_b[0] = trans_b.getBasis() * _w_t_b[0];
    _w_t_b[1] = trans_b.getBasis() * _w_t_b[1];

    const pe::Real inv_mass_a = _object_a->isKinematic() ? PE_R(0.0) : _object_a->getInvMass();
    const pe::Real inv_mass_b = _object_b->isKinematic() ? PE_R(0.0) : _object_b->getInvMass();
    const pe::Matrix3& inv_inertia_a = _object_a->isKinematic() ? pe::Matrix3::zeros() : _object_a->getWorldInvInertia();
    const pe::Matrix3& inv_inertia_b = _object_b->isKinematic() ? pe::Matrix3::zeros() : _object_b->getWorldInvInertia();
    const pe::Real inv_mass_sum = inv_mass_a + inv_mass_b;
    const pe::Matrix3 inv_inertia_sum = inv_inertia_a + inv_inertia_b;

    // ball jmj
    pe::Matrix3 rx_a, rx_b;
    getSkewSymmetricMatrix(_r_a, rx_a);
    getSkewSymmetricMatrix(_r_b, rx_b);
    _jmj_inv_ball = (pe::Matrix3::identity() * inv_mass_sum +
        rx_a * inv_inertia_a * rx_a.transposed() +
        rx_b * inv_inertia_b * rx_b.transposed()).inverse();

    // ball rhs
    _rhs_ball = _jmj_inv_ball * (trans_a * _anchor_a - trans_b * _anchor_b) * (-param.kerp / param.dt);

    // hinge jmj
    _jmj_inv_hinge[0] = PE_R(1.0) / _w_t_a[0].dot(inv_inertia_sum * _w_t_a[0]);
    _jmj_inv_hinge[1] = PE_R(1.0) / _w_t_a[1].dot(inv_inertia_sum * _w_t_a[1]);

    // hinge rhs
    const pe::Vector3& u = _w_axis_a.cross(_w_axis_b);
    // Using arc-sin here would be more accurate, but not necessary
    _rhs_hinge[0] = _jmj_inv_hinge[0] * (u.dot(_w_t_a[0]) * param.kerp / param.dt);
    _rhs_hinge[1] = _jmj_inv_hinge[1] * (u.dot(_w_t_a[1]) * param.kerp / param.dt);

    // motor and limits
    _jmj_inv_motor_limit = PE_R(1.0) / _w_axis_a.dot(inv_inertia_sum * _w_axis_a);
    const pe::Real angle_cos = _w_t_a[0].dot(_w_t_b[0]);
    const pe::Real angle_sin = _w_t_a[0].cross(_w_t_b[0]).dot(_w_axis_a);
    const pe::Real angle = PE_ATAN2(angle_sin, angle_cos);
    _limit_exceeded_type = 0;
    _total_impulse_limit = PE_R(0.0);
    if (angle < _min_angle && (_limit_type == ConstraintLimitType::CLT_LOWER || _limit_type == ConstraintLimitType::CLT_LOWER_UPPER)) {
        _limit_exceeded_type = 1;
        _rhs_limit = -_jmj_inv_motor_limit * (_min_angle - angle) * param.kerp / param.dt;
    } else if (angle > _max_angle && (_limit_type == ConstraintLimitType::CLT_UPPER || _limit_type == ConstraintLimitType::CLT_LOWER_UPPER)) {
        _limit_exceeded_type = 2;
        _rhs_limit = -_jmj_inv_motor_limit * (_max_angle - angle) * param.kerp / param.dt;
    }
    _rhs_motor = -_jmj_inv_motor_limit * (_target_angle - angle) * param.kerp / param.dt;
}

void HingeJointConstraint::iterateSequentialImpulse(int iter) {
    if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

    // position impulse
    const pe::Vector3 vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
    const pe::Vector3 vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
    const pe::Vector3 ball_impulse = _rhs_ball - _jmj_inv_ball * (vel_a - vel_b);
    _object_a->applyTempImpulse(_r_a, ball_impulse);
    _object_b->applyTempImpulse(_r_b, -ball_impulse);

    // rotation impulse
    const pe::Vector3& w_a = _object_a->getTempAngularVelocity();
    const pe::Vector3& w_b = _object_b->getTempAngularVelocity();
    const pe::Real hinge_impulse0 = _rhs_hinge[0] - _jmj_inv_hinge[0] * _w_t_a[0].dot(w_a - w_b);
    const pe::Real hinge_impulse1 = _rhs_hinge[1] - _jmj_inv_hinge[1] * _w_t_a[1].dot(w_a - w_b);
    const pe::Vector3 impulse_vector = _w_t_a[0] * hinge_impulse0 + _w_t_a[1] * hinge_impulse1;
    _object_a->applyTempAngularImpulse(impulse_vector);
    _object_b->applyTempAngularImpulse(-impulse_vector);

    // motor impulse
    if (_motor_type == ConstraintMotorType::CMT_VELOCITY) {
        const pe::Vector3 delta_w = w_a - w_b;
        const pe::Vector3 target_w = _w_axis_a * _target_speed;
        const pe::Real motor_impulse = -_jmj_inv_motor_limit * _w_axis_a.dot(delta_w - target_w);
        _object_a->applyTempAngularImpulse(_w_axis_a * motor_impulse);
        _object_b->applyTempAngularImpulse(-_w_axis_a * motor_impulse);
    } else if (_motor_type == ConstraintMotorType::CMT_POSITION) {
        const pe::Real delta = _w_axis_a.dot(w_a - w_b);
        const pe::Real motor_impulse = _rhs_motor - _jmj_inv_motor_limit * delta;
        _object_a->applyTempAngularImpulse(_w_axis_a * motor_impulse);
        _object_b->applyTempAngularImpulse(-_w_axis_a * motor_impulse);
    }

    // limit impulse
    if (_limit_exceeded_type > 0) {
        const pe::Real delta = _w_axis_a.dot(w_a - w_b);
        pe::Real limit_impulse = _rhs_limit - _jmj_inv_motor_limit * delta;
        if (_limit_exceeded_type == 1) {
            limit_impulse = PE_MIN(limit_impulse, -_total_impulse_limit);
        } else if (_limit_exceeded_type == 2) {
            limit_impulse = PE_MAX(limit_impulse, -_total_impulse_limit);
        }
        _total_impulse_limit += limit_impulse;
        _object_a->applyTempAngularImpulse(_w_axis_a * limit_impulse);
        _object_b->applyTempAngularImpulse(-_w_axis_a * limit_impulse);
    }
}

} // namespace pe_physics_constraint
