#include "six_dof_constraint.h"

namespace pe_physics_constraint {

void SixDofConstraint::initSequentialImpulse(const ConstraintParam &param) {
    if (!_object_a || !_object_b) return;

    auto& trans_a = _object_a->getTransform();
    auto& trans_b = _object_b->getTransform();
    _r_a = trans_a.getBasis() * _frame_a.getOrigin();
    _r_b = trans_b.getBasis() * _frame_b.getOrigin();
    _w_axis_a = trans_a.getBasis() * _frame_a.getBasis();
    _w_axis_b = trans_b.getBasis() * _frame_b.getBasis();

    const pe::Real inv_mass_sum = _object_a->getInvMass() + _object_b->getInvMass();
    const pe::Matrix3& inv_inertia_a = _object_a->getWorldInvInertia();
    const pe::Matrix3& inv_inertia_b = _object_b->getWorldInvInertia();
    const pe::Matrix3 inv_inertia_sum = inv_inertia_a + inv_inertia_b;

    /*
     * Position constraint on an axis:
     * position of rigid_b on the axis in local frame of rigid_a should be zero
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
     * Rotation constraint on an axis:
     * Take x for example, the axis y of rigid_a in world space should be perpendicular
     * to axis z of rigid_b in world space
     * Each fixed axis gives one perpendicular hinge2 constraint
     */
    _rhs_rot = pe::Vector3::zeros();
    _rot_fixed_count = _rot_fixed[0] + _rot_fixed[1] + _rot_fixed[2];
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
}

void SixDofConstraint::iterateSequentialImpulse(int iter) {
    if (!_object_a || !_object_b) return;

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
}

} // namespace pe_physics_constraint