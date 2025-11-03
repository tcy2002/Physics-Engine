#include "friction_contact_constraint.h"

namespace pe_physics_constraint {
    
void FrictionContactConstraint::initSequentialImpulse(const ConstraintParam& param) {
    if (!_contact_result) return;
    const int point_size = PE_MIN(PE_MAX_CONTACT_POINT, _contact_result->getPointSize());
    _cis.resize(point_size);

    _object_a = _contact_result->getObjectA();
    _object_b = _contact_result->getObjectB();
    if (!_object_a || !_object_b) return;
    const pe::Transform& transform_a = _object_a->getTransform();
    const pe::Transform& transform_b = _object_b->getTransform();

    for (int i = 0; i < point_size; i++) {
        const pe_physics_collision::ContactPoint& cp = _contact_result->getContactPoint(i);
        ConstraintInfo& ci = _cis[i];

        const pe::Vector3 r_a = transform_a.getBasis() * cp.getLocalPosA();
        const pe::Vector3 r_b = transform_b.getBasis() * cp.getLocalPosB();

        //// setup ci
        ci.r_a = r_a;
        ci.r_b = r_b;
        ci.n = cp.getWorldNormal();
        ci.t0 = cp.getTangent(0);
        ci.t1 = cp.getTangent(1);

        const pe::Real& inv_mass_sum = _object_a->getInvMass() + _object_b->getInvMass();
        const pe::Matrix3& inv_inertia_a = _object_a->getWorldInvInertia();
        const pe::Matrix3& inv_inertia_b = _object_b->getWorldInvInertia();

        // normal jmj
        const pe::Vector3& rxn_a_n = r_a.cross(ci.n);
        const pe::Vector3& rxn_b_n = r_b.cross(ci.n);
        ci.n_jmj_inv = PE_R(1.0) / (inv_mass_sum + (inv_inertia_a * rxn_a_n).dot(rxn_a_n)
            + (inv_inertia_b * rxn_b_n).dot(rxn_b_n));

        // tangent 0 jmj
        const pe::Vector3& rxn_a_t0 = r_a.cross(ci.t0);
        const pe::Vector3& rxn_b_t0 = r_b.cross(ci.t0);
        ci.t0_jmj_inv = PE_R(1.0) / (inv_mass_sum + (inv_inertia_a * rxn_a_t0).dot(rxn_a_t0)
            + (inv_inertia_b * rxn_b_t0).dot(rxn_b_t0));

        // tangent 1 jmj
        pe::Vector3 rxn_a_t1 = r_a.cross(ci.t1);
        pe::Vector3 rxn_b_t1 = r_b.cross(ci.t1);
        ci.t1_jmj_inv = PE_R(1.0) / (inv_mass_sum + (inv_inertia_a * rxn_a_t1).dot(rxn_a_t1)
            + (inv_inertia_b * rxn_b_t1).dot(rxn_b_t1));

        const pe::Vector3& vel_a = _object_a->getLinearVelocity() + _object_a->getAngularVelocity().cross(r_a);
        const pe::Vector3& vel_b = _object_b->getLinearVelocity() + _object_b->getAngularVelocity().cross(r_b);

        // normal rhs
        pe::Real rev_vel_r = -ci.n.dot(vel_a - vel_b);
        rev_vel_r *= _contact_result->getRestitutionCoeff();
        pe::Real penetration = cp.getDistance();
        ci.n_rhs = (rev_vel_r - param.kerp * penetration / param.dt) * ci.n_jmj_inv;

        ci.n_applied_impulse = 0;
        ci.t0_applied_impulse = 0;
        ci.t1_applied_impulse = 0;
        ci.friction_coeff = _contact_result->getFrictionCoeff();
    }
}

void FrictionContactConstraint::iterateSequentialImpulse(int iter) {
    if (!_object_a || !_object_b) return;

    for (auto& ci : _cis) {
        const pe::Vector3 vel_r = (_object_a->getTempLinearVelocity() +
            _object_a->getTempAngularVelocity().cross(ci.r_a))
            - (_object_b->getTempLinearVelocity()
            + _object_b->getTempAngularVelocity().cross(ci.r_b));

        // normal impulse (contact)
        pe::Real n_impulse = ci.n_rhs - ci.n.dot(vel_r) * ci.n_jmj_inv;
        n_impulse = PE_MAX(n_impulse, -ci.n_applied_impulse);
        ci.n_applied_impulse += n_impulse;

        // tangent impulse (friction)
        pe::Real t0_total_impulse = ci.t0_applied_impulse - ci.t0.dot(vel_r) * ci.t0_jmj_inv;
        pe::Real t1_total_impulse = ci.t1_applied_impulse - ci.t1.dot(vel_r) * ci.t1_jmj_inv;
        const pe::Real scale = ci.friction_coeff * ci.n_applied_impulse /
            PE_SQRT(PE_SQR(t0_total_impulse) + PE_SQR(t1_total_impulse));
        if (scale < 1) {
            t0_total_impulse *= scale;
            t1_total_impulse *= scale;
        }

        // total impulse
        const pe::Vector3& impulse_vector = n_impulse * ci.n +
            (t0_total_impulse - ci.t0_applied_impulse) * ci.t0 +
            (t1_total_impulse - ci.t1_applied_impulse) * ci.t1;

        ci.t0_applied_impulse = t0_total_impulse;
        ci.t1_applied_impulse = t1_total_impulse;

        _object_a->applyTempImpulse(ci.r_a, impulse_vector);
        _object_b->applyTempImpulse(ci.r_b, -impulse_vector);
    }
}

} // namespace pe_physics_constraint
