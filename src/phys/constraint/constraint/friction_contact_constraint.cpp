#include "friction_contact_constraint.h"

// style-checked
namespace pe_phys_constraint {
    
    void FrictionContactConstraint::initSequentialImpulse(const ConstraintParam& param) {
        if (!_contact_result) return;
        const int point_size = PE_MIN(PE_MAX_CONTACT_POINT, _contact_result->getPointSize());
        _cis.resize(point_size);

        _object_a = _contact_result->getObjectA();
        _object_b = _contact_result->getObjectB();
        const pe::Transform& transform_a = _object_a->getTransform();
        const pe::Transform& transform_b = _object_b->getTransform();

        for (int i = 0; i < point_size; i++) {
            const pe_phys_collision::ContactPoint& cp = _contact_result->getContactPoint(i);
            ConstraintInfo& ci = _cis[i];

            const pe::Vector3 r_a = transform_a.getBasis() * cp.getLocalPosA();
            const pe::Vector3 r_b = transform_b.getBasis() * cp.getLocalPosB();

            //// setup ci
            ci.r_a = r_a;
            ci.r_b = r_b;
            ci.n = cp.getWorldNormal();
            ci.t0 = cp.getTangent(0);
            ci.t1 = cp.getTangent(1);

            const pe::Real inv_mass_sum = _object_a->getInvMass() + _object_b->getInvMass();
            const pe::Matrix3 rot_inv_inertia_a = _object_a->getWorldInvInertia();
            const pe::Matrix3 rot_inv_inertia_b = _object_b->getWorldInvInertia();

            //// normal denom
            {
                pe::Vector3 rxn_a = r_a.cross(ci.n);
                pe::Vector3 rxn_b = r_b.cross(ci.n);
                ci.n_denom_inv = R(1.0) / (inv_mass_sum + (rot_inv_inertia_a * rxn_a).dot(rxn_a) +
                                        (rot_inv_inertia_b * rxn_b).dot(rxn_b));
            }
            //// tangent denom
            {
                pe::Vector3 rxn_a = r_a.cross(ci.t0);
                pe::Vector3 rxn_b = r_b.cross(ci.t0);
                ci.t0_denom_inv = R(1.0) / (inv_mass_sum + (rot_inv_inertia_a * rxn_a).dot(rxn_a) +
                                         (rot_inv_inertia_b * rxn_b).dot(rxn_b));
            }
            {
                pe::Vector3 rxn_a = r_a.cross(ci.t1);
                pe::Vector3 rxn_b = r_b.cross(ci.t1);
                ci.t1_denom_inv = R(1.0) / (inv_mass_sum + (rot_inv_inertia_a * rxn_a).dot(rxn_a) +
                                         (rot_inv_inertia_b * rxn_b).dot(rxn_b));
            }

            const pe::Vector3 vel_a = _object_a->getLinearVelocity() + _object_a->getAngularVelocity().cross(r_a);
            const pe::Vector3 vel_b = _object_b->getLinearVelocity() + _object_b->getAngularVelocity().cross(r_b);

            //// normal rhs
            {
                pe::Real rev_vel_r = -ci.n.dot(vel_a - vel_b);
                if (PE_ABS(rev_vel_r) < param.restitutionVelocityThreshold) {
                    rev_vel_r = 0;
                }
                rev_vel_r *= _contact_result->getRestitutionCoeff();

                pe::Real penetration = cp.getDistance();
                penetration = PE_MAX(penetration, -param.penetrationThreshold);
                ci.n_rhs = (rev_vel_r - param.kerp * penetration / param.dt) * ci.n_denom_inv;
            }

            ci.n_applied_impulse = 0;
            ci.t0_applied_impulse = 0;
            ci.t1_applied_impulse = 0;
            ci.friction_coeff = _contact_result->getFrictionCoeff();
        }
    }

    void FrictionContactConstraint::warmStart() {
        for (int i = 0; i < I(_cis.size()); i++) {
            auto& cp = _contact_result->getContactPoint(i);
            ConstraintInfo& ci = _cis[i];
            ci.n_applied_impulse = cp.getAppliedImpulse().dot(ci.n) * R(0.9);

            _object_a->applyTempImpulse(ci.r_a, ci.n_applied_impulse * ci.n);
            _object_b->applyTempImpulse(ci.r_b, -ci.n_applied_impulse * ci.n);
        }
    }

    void FrictionContactConstraint::iterateSequentialImpulse(int iter) {
        for (auto& ci : _cis) {
            const pe::Vector3& r_a = ci.r_a;
            const pe::Vector3& r_b = ci.r_b;
            const pe::Vector3& n = ci.n;
            const pe::Vector3& t0 = ci.t0;
            const pe::Vector3& t1 = ci.t1;
            const pe::Vector3 vel_r = (_object_a->getTempLinearVelocity() +
                    _object_a->getTempAngularVelocity().cross(r_a))
                    - (_object_b->getTempLinearVelocity()
                    + _object_b->getTempAngularVelocity().cross(r_b));

            // compute impulse
            pe::Real n_impulse = ci.n_rhs - n.dot(vel_r) * ci.n_denom_inv;
            n_impulse = PE_MAX(n_impulse, -ci.n_applied_impulse);
            ci.n_applied_impulse += n_impulse;

#       if true
            pe::Real t0_total_impulse = ci.t0_applied_impulse - t0.dot(vel_r) * ci.t0_denom_inv;
            pe::Real max_friction = ci.friction_coeff * ci.n_applied_impulse;
            pe::Real t1_total_impulse = ci.t1_applied_impulse - t1.dot(vel_r) * ci.t1_denom_inv;
            t0_total_impulse = PE_MIN(t0_total_impulse, max_friction);
            t1_total_impulse = PE_MIN(t1_total_impulse, max_friction);
            t0_total_impulse = PE_MAX(t0_total_impulse, -max_friction);
            t1_total_impulse = PE_MAX(t1_total_impulse, -max_friction);
#       else
            pe::Real t0_total_impulse = ci.t0_applied_impulse - t0.dot(vel_r) * ci.t0_denom_inv;
            pe::Real t1_total_impulse = ci.t1_applied_impulse - t1.dot(vel_r) * ci.t1_denom_inv;
            const pe::Real scale = ci.friction_coeff * ci.n_applied_impulse /
                    PE_SQRT(PE_SQR(t0_total_impulse) + PE_SQR(t1_total_impulse));
            if (scale < 1) {
                t0_total_impulse *= scale;
                t1_total_impulse *= scale;
            }
#       endif


            pe::Vector3 impulse_vector = n_impulse * n + (t0_total_impulse - ci.t0_applied_impulse) * t0 +
                    (t1_total_impulse - ci.t1_applied_impulse) * t1;

            ci.t0_applied_impulse = t0_total_impulse;
            ci.t1_applied_impulse = t1_total_impulse;

            _object_a->applyTempImpulse(r_a, impulse_vector);
            _object_b->applyTempImpulse(r_b, -impulse_vector);
        }
    }
    
} // namespace pe_phys_constraint