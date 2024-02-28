#include "friction_contact_constraint.h"

namespace pe_phys_constraint {
    
    void FrictionContactConstraint::initSequentialImpulse(const ConstraintParam& param) {
        int p_size = _contact_result.getPointSize();
        _cis.resize(p_size);

        _body_a = (pe_phys_object::RigidBody*)(_contact_result.getObjectA());
        _body_b = (pe_phys_object::RigidBody*)(_contact_result.getObjectB());
        const pe::Transform& transform_a = _body_a->getTransform();
        const pe::Transform& transform_b = _body_b->getTransform();

        for(int i = 0; i < _contact_result.getPointSize(); i++){
            const pe_phys_collision::ContactPoint& cp = _contact_result.getContactPoint(i);
            ConstraintInfo& ci = _cis[i];

            const pe::Vector3 r_a = transform_a.getBasis() * cp.getLocalPosA();
            const pe::Vector3 r_b = transform_b.getBasis() * cp.getLocalPosB();

            //// setup ci
            ci.r_a = r_a;
            ci.r_b = r_b;
            ci.n = cp.getWorldNormal();
            ci.t0 = cp.getTangent(0);
            ci.t1 = cp.getTangent(1);

            const pe::Real inv_mass_sum = _body_a->getInvMass() + _body_b->getInvMass();
            const pe::Matrix3 world_inv_inertia_a = _body_a->getWorldInvInertia();
            const pe::Matrix3 world_inv_inertia_b = _body_b->getWorldInvInertia();

            //// normal denom
            {
                pe::Vector3 rxn_a = r_a.cross(ci.n);
                pe::Vector3 rxn_b = r_b.cross(ci.n);
                ci.n_denom_inv = 1.0 / (inv_mass_sum + (world_inv_inertia_a * rxn_a).dot(rxn_a) +
                        (world_inv_inertia_b * rxn_b).dot(rxn_b));
            }
            //// tangent denom 
            {
                pe::Vector3 rxn_a = r_a.cross(ci.t0);
                pe::Vector3 rxn_b = r_b.cross(ci.t0);
                ci.t0_denom_inv = 1.0 / (inv_mass_sum + (world_inv_inertia_a * rxn_a).dot(rxn_a) +
                        (world_inv_inertia_b * rxn_b).dot(rxn_b));
            }
            {
                pe::Vector3 rxn_a = r_a.cross(ci.t1);
                pe::Vector3 rxn_b = r_b.cross(ci.t1);
                ci.t1_denom_inv = 1.0 / (inv_mass_sum + (world_inv_inertia_a * rxn_a).dot(rxn_a) +
                        (world_inv_inertia_b * rxn_b).dot(rxn_b));
            }

            const pe::Vector3 vel_a = _body_a->getLinearVelocity() + _body_a->getAngularVelocity().cross(r_a);
            const pe::Vector3 vel_b = _body_b->getLinearVelocity() + _body_b->getAngularVelocity().cross(r_b);

            //// normal rhs
            {
                pe::Real rev_vel_r = -ci.n.dot(vel_a - vel_b);
                if (std::abs(rev_vel_r) < param.restitutionVelocityThreshold) {
                    rev_vel_r = 0;
                }
                rev_vel_r *= _contact_result.getRestitutionCoeff();

                pe::Real penetration = cp.getDistance();
                penetration = std::max(penetration, -param.penetrationThreshold);
                if (param.splitPenetrationConstraintFlag) {
                    ci.n_rhs = rev_vel_r * ci.n_denom_inv;
                    ci.n_penetration_rhs = -param.kerp * penetration / param.dt * ci.n_denom_inv;
                } else {
                    ci.n_rhs = (rev_vel_r - param.kerp * penetration / param.dt) * ci.n_denom_inv;
                    ci.n_penetration_rhs = 0;
                }
            }

            ci.n_applied_impulse = 0;
            ci.n_applied_penetration_impulse = 0;
            ci.t0_applied_impulse = 0;
            ci.t1_applied_impulse = 0;
            ci.friction_coeff = _contact_result.getFrictionCoeff();
        }
    }

    void FrictionContactConstraint::afterSequentialImpulse() {
        for(int i = 0; i < (int)_cis.size(); i++){
            const ConstraintInfo& ci = _cis[i];
            _contact_result.getContactPoint(i).setAppliedImpulse(ci.n_applied_impulse * ci.n);
        }
    }

    void FrictionContactConstraint::iterateSequentialImpulse(int iter) {
        for (int i = 0; i < (int)_cis.size(); i++) {
            ConstraintInfo& ci = _cis[i];
            const pe::Vector3& r_a = ci.r_a;
            const pe::Vector3& r_b = ci.r_b;
            const pe::Vector3& n = ci.n;
            const pe::Vector3& t0 = ci.t0;
            const pe::Vector3& t1 = ci.t1;
            const pe::Vector3 vel_r = (_body_a->getTempLinearVelocity() +
                    _body_a->getTempAngularVelocity().cross(r_a))
                    - (_body_b->getTempLinearVelocity()
                    + _body_b->getTempAngularVelocity().cross(r_b));

            //// compute impulse
            pe::Real n_impulse = ci.n_rhs - n.dot(vel_r) * ci.n_denom_inv;
            n_impulse = std::max(n_impulse, -ci.n_applied_impulse);
            ci.n_applied_impulse += n_impulse;

            // TODO: fix it: incorrect coulomb cone(2 direction)
#           define PE_USE_COULOMB_CONE true
#           if PE_USE_COULOMB_CONE
                //// compute impulse
                pe::Real t0_total_impulse = ci.t0_applied_impulse - t0.dot(vel_r) * ci.t0_denom_inv;
                pe::Real max_friction = ci.friction_coeff * ci.n_applied_impulse;
                pe::Real t1_total_impulse = ci.t1_applied_impulse - t1.dot(vel_r) * ci.t1_denom_inv;

                t0_total_impulse = std::min(t0_total_impulse, max_friction);
                t1_total_impulse = std::min(t1_total_impulse, max_friction);
                t0_total_impulse = std::max(t0_total_impulse, -max_friction);
                t1_total_impulse = std::max(t1_total_impulse, -max_friction);
#           else
                pe::Real t0_total_impulse = ci.t0_applied_impulse - t0.dot(vel_r) * ci.t0_denom_inv;
                pe::Real t1_total_impulse = ci.t1_applied_impulse - t1.dot(vel_r) * ci.t1_denom_inv;
                pe::Real scale = ci.friction_coeff * ci.n_applied_impulse / std::sqrt(t0_total_impulse * t0_total_impulse + t1_total_impulse * t1_total_impulse);
                if(scale < 1) {
                    t0_total_impulse *= scale;
                    t1_total_impulse *= scale;
                }
#           endif

            const pe::Vector3 impulse_vector = n_impulse * n + (t0_total_impulse - ci.t0_applied_impulse) * t0 +
                    (t1_total_impulse - ci.t1_applied_impulse) * t1;

            ci.t0_applied_impulse = t0_total_impulse;
            ci.t1_applied_impulse = t1_total_impulse;

            _body_a->applyTempImpulse(r_a, impulse_vector);
            _body_b->applyTempImpulse(r_b, -impulse_vector);
        }
    }

    void FrictionContactConstraint::iterateSequentialImpulseForPenetration(int iter) {
        for(int i = 0; i < (int)_cis.size(); i++){
            ConstraintInfo& ci = _cis[i];
            const pe::Vector3& n = ci.n;
            const pe::Vector3 vel_r = (_body_a->getPenetrationLinearVelocity() +
                    _body_a->getPenetrationAngularVelocity().cross(ci.r_a))
                    - (_body_b->getPenetrationLinearVelocity() +
                    _body_b->getPenetrationAngularVelocity().cross(ci.r_b));

            //// compute impulse
            pe::Real temp_impulse = (ci.n_penetration_rhs - n.dot(vel_r) * ci.n_denom_inv);

            //// clamp impulse, if it causes negative total impulse
            if(temp_impulse < -ci.n_applied_penetration_impulse){
                temp_impulse = -ci.n_applied_penetration_impulse;
            }

            // apply impulse
            ci.n_applied_penetration_impulse += temp_impulse;
            const pe::Vector3 impulse_vector = temp_impulse * n;

            _body_a->applyPenetrationImpulse(ci.r_a, impulse_vector);
            _body_b->applyPenetrationImpulse(ci.r_b, -impulse_vector);
        }
    }

    void FrictionContactConstraint::warmStart() {
        for(int i = 0; i < (int)_cis.size(); i++) {
            pe_phys_collision::ContactPoint& cp = _contact_result.getContactPoint(i);
            ConstraintInfo& ci = _cis[i];
            ci.n_applied_impulse = cp.getAppliedImpulse().dot(ci.n) * 0.9;
            _body_a->applyTempImpulse(ci.r_a, ci.n_applied_impulse * ci.n);
            _body_b->applyTempImpulse(ci.r_b, -ci.n_applied_impulse * ci.n);
        }
    }
    
} // namespace pe_phys_constraint