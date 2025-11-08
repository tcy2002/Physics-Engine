#include "slider_joint_constraint.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_constraint {

    void SliderJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
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

        // position 0 jmj
        const pe::Vector3& rxn_a_t0 = _r_a.cross(_w_t_a[0]);
        const pe::Vector3& rxn_b_t0 = _r_b.cross(_w_t_a[0]);
        _jmj_inv_pos[0] = PE_R(1.0) / (inv_mass_sum + (inv_inertia_a * rxn_a_t0).dot(rxn_a_t0)
            + (inv_inertia_b * rxn_b_t0).dot(rxn_b_t0));

        // position 1 jmj
        const pe::Vector3& rxn_a_t1 = _r_a.cross(_w_t_a[1]);
        const pe::Vector3& rxn_b_t1 = _r_b.cross(_w_t_a[1]);
        _jmj_inv_pos[1] = PE_R(1.0) / (inv_mass_sum + (inv_inertia_a * rxn_a_t1).dot(rxn_a_t1)
            + (inv_inertia_b * rxn_b_t1).dot(rxn_b_t1));

        // position rhs
        const pe::Vector3& rel_pos = trans_a * _anchor_a - trans_b * _anchor_b;
        _rhs_pos[0] = _jmj_inv_pos[0] * (rel_pos.dot(_w_t_a[0]) * (-param.kerp / param.dt));
        _rhs_pos[1] = _jmj_inv_pos[0] * (rel_pos.dot(_w_t_a[1]) * (-param.kerp / param.dt));

        // motor and limits
        _jmj_inv_motor_limit = PE_R(1.0) / inv_mass_sum;
        const pe::Real pos = -rel_pos.dot(_w_axis_a);
        _limit_exceeded_type = 0;
        _total_impulse_limit = PE_R(0.0);
        if (pos < _min_position && (_limit_type == ConstraintLimitType::CLT_LOWER || _limit_type == ConstraintLimitType::CLT_LOWER_UPPER)) {
            _limit_exceeded_type = 1;
            _rhs_limit = -_jmj_inv_motor_limit * (_min_position - pos) * param.kerp / param.dt;
        } else if (pos > _max_position && (_limit_type == ConstraintLimitType::CLT_UPPER || _limit_type == ConstraintLimitType::CLT_LOWER_UPPER)) {
            _limit_exceeded_type = 2;
            _rhs_limit = -_jmj_inv_motor_limit * (_max_position - pos) * param.kerp / param.dt;
        }
        _rhs_motor = -_jmj_inv_motor_limit * (_target_position - pos) * param.kerp / param.dt;

        // rotation jmj
        _jmj_inv_rot = inv_inertia_sum.inverse();

        // rotation rhs
        // Also, using arc-sin here would be more accurate, but not necessary
        _rhs_rot = _jmj_inv_rot * (_w_t_a[0].cross(_w_t_b[0]) + _w_t_a[1].cross(_w_t_b[1])) * (param.kerp / param.dt);
    }

    void SliderJointConstraint::iterateSequentialImpulse(int iter) {
        if (!_object_a || !_object_b || (_object_a->isKinematic() && _object_b->isKinematic())) return;

        // position impulse
        const pe::Vector3& vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
        const pe::Vector3& vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
        const pe::Real& pos_impulse0 = _rhs_pos[0] - _jmj_inv_pos[0] * _w_t_a[0].dot(vel_a - vel_b);
        const pe::Real& pos_impulse1 = _rhs_pos[1] - _jmj_inv_pos[1] * _w_t_a[1].dot(vel_a - vel_b);
        const pe::Vector3 pos_impulse_vector = _w_t_a[0] * pos_impulse0 + _w_t_a[1] * pos_impulse1;
        _object_a->applyTempImpulse(_r_a, pos_impulse_vector);
        _object_b->applyTempImpulse(_r_b, -pos_impulse_vector);

        // motor impulse
        if (_motor_type == ConstraintMotorType::CMT_VELOCITY) {
            const pe::Vector3 rel_vel = vel_a - vel_b;
            const pe::Vector3 target_v = _w_axis_a * -_target_speed;
            const pe::Real motor_impulse = -_jmj_inv_motor_limit * (rel_vel - target_v).dot(_w_axis_a);
            const pe::Vector3 motor_impulse_vector = _w_axis_a * motor_impulse;
            _object_a->applyTempImpulse(pe::Vector3::zeros(), motor_impulse_vector);
            _object_b->applyTempImpulse(pe::Vector3::zeros(), -motor_impulse_vector);
        } else if (_motor_type == ConstraintMotorType::CMT_POSITION) {
            const pe::Real delta = _w_axis_a.dot(vel_a - vel_b);
            const pe::Real motor_impulse = _rhs_motor - _jmj_inv_motor_limit * delta;
            const pe::Vector3 motor_impulse_vector = _w_axis_a * motor_impulse;
            _object_a->applyTempImpulse(pe::Vector3::zeros(), motor_impulse_vector);
            _object_b->applyTempImpulse(pe::Vector3::zeros(), -motor_impulse_vector);
        }

        // limit impulse
        if (_limit_exceeded_type > 0) {
            const pe::Vector3& rel_vel = vel_a - vel_b;
            pe::Real limit_impulse = _rhs_limit - _jmj_inv_motor_limit * rel_vel.dot(_w_axis_a);
            const pe::Vector3 limit_impulse_vector = _w_axis_a * limit_impulse;
            if (_limit_exceeded_type == 1) {
                limit_impulse = PE_MAX(limit_impulse, -_total_impulse_limit);
            } else if (_limit_exceeded_type == 2) {
                limit_impulse = PE_MIN(limit_impulse, -_total_impulse_limit);
            }
            _total_impulse_limit += limit_impulse;
            _object_a->applyTempImpulse(pe::Vector3::zeros(), limit_impulse_vector);
            _object_b->applyTempImpulse(pe::Vector3::zeros(), -limit_impulse_vector);
        }

        // rotation impulse
        const pe::Vector3& w_a = _object_a->getTempAngularVelocity();
        const pe::Vector3& w_b = _object_b->getTempAngularVelocity();
        const pe::Vector3& rot_impulse = _rhs_rot - _jmj_inv_rot * (w_a - w_b);
        _object_a->applyTempAngularImpulse(rot_impulse);
        _object_b->applyTempAngularImpulse(-rot_impulse);
    }

} // namespace pe_physics_constraint
