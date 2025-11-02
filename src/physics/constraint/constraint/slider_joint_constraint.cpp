#include "slider_joint_constraint.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_constraint {

    void SliderJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
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

        const pe::Real& inv_mass_sum = _object_a->getInvMass() + _object_b->getInvMass();
        const pe::Matrix3& inv_inertia_a = _object_a->getWorldInvInertia();
        const pe::Matrix3& inv_inertia_b = _object_b->getWorldInvInertia();

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

        // rotation jmj
        _jmj_inv_rot = (inv_inertia_a + inv_inertia_b).inverse();

        // rotation rhs
        // Also, using arc-sin here would be more accurate, but not necessary
        _rhs_rot = _jmj_inv_rot * (_w_t_a[0].cross(_w_t_b[0]) + _w_t_a[1].cross(_w_t_b[1])) * (param.kerp_angle / param.dt);
    }

    void SliderJointConstraint::iterateSequentialImpulse(int iter) {
        // position impulse
        const pe::Vector3& vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
        const pe::Vector3& vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
        const pe::Real& pos_impulse0 = _rhs_pos[0] - _jmj_inv_pos[0] * _w_t_a[0].dot(vel_a - vel_b);
        const pe::Real& pos_impulse1 = _rhs_pos[1] - _jmj_inv_pos[1] * _w_t_a[1].dot(vel_a - vel_b);
        const pe::Vector3 pos_impulse_vector = _w_t_a[0] * pos_impulse0 + _w_t_a[1] * pos_impulse1;
        _object_a->applyTempImpulse(_r_a, pos_impulse_vector);
        _object_b->applyTempImpulse(_r_b, -pos_impulse_vector);

        // rotation impulse
        const pe::Vector3& w_a = _object_a->getTempAngularVelocity();
        const pe::Vector3& w_b = _object_b->getTempAngularVelocity();
        const pe::Vector3& rot_impulse = _rhs_rot - _jmj_inv_rot * (w_a - w_b);
        _object_a->applyTempAngularImpulse(rot_impulse);
        _object_b->applyTempAngularImpulse(-rot_impulse);
    }

} // namespace pe_physics_constraint
