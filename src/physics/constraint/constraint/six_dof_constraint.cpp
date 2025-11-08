#include "six_dof_constraint.h"

namespace pe_physics_constraint {

void SixDofConstraint::initSequentialImpulse(const ConstraintParam &param) {
    if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

    auto& trans_a = _object_a->getTransform();
    auto& trans_b = _object_b->getTransform();
    _r_a = trans_a.getBasis() * _frame_a.getOrigin();
    _r_b = trans_b.getBasis() * _frame_b.getOrigin();
    _w_axis_a = trans_a.getBasis() * _frame_a.getBasis();
    _w_axis_b = trans_b.getBasis() * _frame_b.getBasis();

    const pe::Real inv_mass_a = _object_a->isKinematic() ? PE_R(0.0) : _object_a->getInvMass();
    const pe::Real inv_mass_b = _object_b->isKinematic() ? PE_R(0.0) : _object_b->getInvMass();
    const pe::Matrix3& inv_inertia_a = _object_a->isKinematic() ? pe::Matrix3::zeros() : _object_a->getWorldInvInertia();
    const pe::Matrix3& inv_inertia_b = _object_b->isKinematic() ? pe::Matrix3::zeros() : _object_b->getWorldInvInertia();
    const pe::Real inv_mass_sum = inv_mass_a + inv_mass_b;
    const pe::Matrix3 inv_inertia_sum = inv_inertia_a + inv_inertia_b;

    /*
     * Position constraint:
     * position of rigid_b on an axis in local frame of rigid_a should be zero
     */
    const pe::Vector3& rel_pos = trans_a * _frame_a.getOrigin() - trans_b * _frame_b.getOrigin();
    for (int i = 0; i < 3; i++) {
        if (!_pos_fixed[i]) continue;
        const pe::Vector3& rxn_a = _r_a.cross(_w_axis_a.getColumn(i));
        const pe::Vector3& rxn_b = _r_b.cross(_w_axis_a.getColumn(i));
        _jmj_inv_pos[i] = PE_R(1.0) / (inv_mass_sum + (inv_inertia_a * rxn_a).dot(rxn_a)
            + (inv_inertia_b * rxn_b).dot(rxn_b));
        _rhs_pos[i] = _jmj_inv_pos[i] * (rel_pos.dot(_w_axis_a.getColumn(i)) * (-param.kerp / param.dt));
    }

    /*
     * Rotation constraint:
     * When one axis is fixed, x for example, the axis y of rigid_a in world space should be perpendicular
     * to axis z of rigid_b in world space. When 2 axis are fixed, x and z for example, the axis y of rigid_a
     * should be aligned with axis y of rigid_b.
     *
     * One fixed axis gives a perpendicular hinge2 constraint
     * Two fixed axis gives a hinge constraint
     * Three fixed axis gives a fixed rotation constraint
     */
    _rhs_rot = pe::Vector3::zeros();
    for (int i = 0; i < 3; i++) {
        if (!_rot_fixed[i]) continue;
        const int axis_a_idx = _rot_fixed[(i + 1) % 3] ? (i + 2) % 3 : (i + 1) % 3;
        const int axis_b_idx = _rot_fixed[(i + 1) % 3] ? (i + 1) % 3 : (i + 2) % 3;
        const pe::Vector3 axis_a = _w_axis_a.getColumn(axis_a_idx);
        const pe::Vector3 axis_b = _w_axis_b.getColumn(axis_b_idx);
        _axb[i] = axis_a.cross(axis_b);
        if (_axb[i].norm2() < PE_R(1e-10)) {
            // choose axis i of rigid_a as the rotation axis
            _axb[i] = _w_axis_a.getColumn(i);
        }
        _jmj_inv_rot[i] = PE_R(1.0) / _axb[i].dot(inv_inertia_sum * _axb[i]);
        // Also, using arc-cos here would be more accurate, but not necessary
        _rhs_rot[i] = _jmj_inv_rot[i] * (axis_a.dot(axis_b) * -param.kerp / param.dt);
    }

    /*
     * Rotation limits and motors
     */
    _rhs_limit = pe::Vector3::zeros();
    _rhs_motor = pe::Vector3::zeros();
    for (int i = 0; i < 3; i++) {
        if (_rot_fixed[i]) continue;
        const pe::Vector3 axis_a = _rot_fixed[(i + 1) % 3] ? _w_axis_a.getColumn((i + 2) % 3) : _w_axis_a.getColumn((i + 1) % 3);
        const pe::Vector3 axis_b = _rot_fixed[(i + 1) % 3] ? _w_axis_b.getColumn((i + 2) % 3) : _w_axis_b.getColumn((i + 1) % 3);
        _nxn[i] = _rot_fixed[(i + 1) % 3] ? _w_axis_b.getColumn(i) : _w_axis_a.getColumn(i);
        _jmj_inv_motor_limit[i] = PE_R(1.0) / _nxn[i].dot(inv_inertia_sum * _nxn[i]);
        const pe::Real angle_cos = axis_a.dot(axis_b);
        const pe::Real angle_sin = axis_a.cross(axis_b).dot(_nxn[i]);
        const pe::Real angle = PE_ATAN2(angle_sin, angle_cos);
        _limit_exceeded_type[i] = 0;
        _total_impulse_limit[i] = PE_R(0.0);
        if (angle < _min_angle[i] && (_limit_type[i] == ConstraintLimitType::CLT_LOWER || _limit_type[i] == ConstraintLimitType::CLT_LOWER_UPPER)) {
            _limit_exceeded_type[i] = 1;
            _rhs_limit[i] = -_jmj_inv_motor_limit[i] * (_min_angle[i] - angle) * param.kerp / param.dt;
        } else if (angle > _max_angle[i] && (_limit_type[i] == ConstraintLimitType::CLT_UPPER || _limit_type[i] == ConstraintLimitType::CLT_LOWER_UPPER)) {
            _limit_exceeded_type[i] = 2;
            _rhs_limit[i] = -_jmj_inv_motor_limit[i] * (_max_angle[i] - angle) * param.kerp / param.dt;
        }
        _rhs_motor[i] = -_jmj_inv_motor_limit[i] * (_target_angle[i] - angle) * param.kerp / param.dt;
    }
}

void SixDofConstraint::iterateSequentialImpulse(int iter) {
    if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

    // position impulse
    const pe::Vector3& vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
    const pe::Vector3& vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
    pe::Vector3 pos_impulse_vector = pe::Vector3::zeros();
    for (int i = 0; i < 3; i++) {
        if (!_pos_fixed[i]) continue;
        const pe::Real& pos_impulse = _rhs_pos[i] - _jmj_inv_pos[i] * _w_axis_a.getColumn(i).dot(vel_a - vel_b);
        pos_impulse_vector += _w_axis_a.getColumn(i) * pos_impulse;
    }
    _object_a->applyTempImpulse(_r_a, pos_impulse_vector);
    _object_b->applyTempImpulse(_r_b, -pos_impulse_vector);

    // rotation impulse
    const pe::Vector3& w_a = _object_a->getTempAngularVelocity();
    const pe::Vector3& w_b = _object_b->getTempAngularVelocity();
    pe::Vector3 rot_impulse_vector = pe::Vector3::zeros();
    for (int i = 0; i < 3; i++) {
        if (!_rot_fixed[i]) continue;
        const pe::Real& rot_impulse = _rhs_rot[i] - _jmj_inv_rot[i] * _axb[i].dot(w_a - w_b);
        rot_impulse_vector += _axb[i] * rot_impulse;
    }
    _object_a->applyTempAngularImpulse(rot_impulse_vector);
    _object_b->applyTempAngularImpulse(-rot_impulse_vector);

    // rotation motor impulse
    for (int i = 0; i < 3; i++) {
        if (_rot_fixed[i]) continue;
        if (_motor_type[i] == ConstraintMotorType::CMT_VELOCITY) {
            const pe::Vector3 delta_w = w_a - w_b;
            const pe::Real target_w = _target_speed[i];
            const pe::Real motor_impulse = -_jmj_inv_motor_limit[i] * _nxn[i].dot(delta_w - _nxn[i] * target_w);
            _object_a->applyTempAngularImpulse(_nxn[i] * motor_impulse);
            _object_b->applyTempAngularImpulse(-_nxn[i] * motor_impulse);
        } else if (_motor_type[i] == ConstraintMotorType::CMT_POSITION) {
            const pe::Real delta = _nxn[i].dot(w_a - w_b);
            const pe::Real motor_impulse = _rhs_motor[i] - _jmj_inv_motor_limit[i] * delta;
            _object_a->applyTempAngularImpulse(_nxn[i] * motor_impulse);
            _object_b->applyTempAngularImpulse(-_nxn[i] * motor_impulse);
        }
    }

    // rotation limit impulse
    for (int i = 0; i < 3; i++) {
        if (_rot_fixed[i]) continue;
        if (_limit_exceeded_type[i] > 0) {
            // const pe::Vector3 axis_a = _w_axis_a.getColumn(i);
            const pe::Real delta = _nxn[i].dot(w_a - w_b);
            pe::Real limit_impulse = _rhs_limit[i] - _jmj_inv_motor_limit[i] * delta;
            if (_limit_exceeded_type[i] == 1) {
                limit_impulse = PE_MIN(limit_impulse, -_total_impulse_limit[i]);
            } else if (_limit_exceeded_type[i] == 2) {
                limit_impulse = PE_MAX(limit_impulse, -_total_impulse_limit[i]);
            }
            _total_impulse_limit[i] += limit_impulse;
            _object_a->applyTempAngularImpulse(_nxn[i] * limit_impulse);
            _object_b->applyTempAngularImpulse(-_nxn[i] * limit_impulse);
        }
    }
}

} // namespace pe_physics_constraint