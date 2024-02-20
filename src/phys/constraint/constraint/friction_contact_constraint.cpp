#include "friction_contact_constraint.h"

namespace pe_phys_constraint {
    
    void FrictionContactConstraint::initSequentialImpulse(const ConstraintParam& param) {
        const int psize = _contact_result.getPointSize();
        _cis.resize(psize);

        _body_a = (pe_phys_object::RigidBody*)(_contact_result.getBodyA());
        _body_b = (pe_phys_object::RigidBody*)(_contact_result.getBodyB());
        const pe::Transform& transformA = _body_a->getTransform();
        const pe::Transform& transformB = _body_b->getTransform();

        for(int i = 0; i < _contact_result.getPointSize(); i++){
            const pe_phys_collision::ContactPoint& cp = _contact_result.getContactPoint(i);
            ConstraintInfo& ci = _cis[i];

            const pe::Vector3 rA = transformA.getBasis() * cp.getLocalPosA();
            const pe::Vector3 rB = transformB.getBasis() * cp.getLocalPosB();

            //// setup ci
            ci.rA = rA;
            ci.rB = rB;
            ci.n = cp.getWorldNormal();
            ci.t0 = cp.getTangent(0);
            ci.t1 = cp.getTangent(1);

            const pe::Real invMassSum = _body_a->getInvMass() + _body_b->getInvMass();
            const pe::Matrix3 worldInvInertiaA = _body_a->getInvInertia();
            const pe::Matrix3 worldInvInertiaB = _body_b->getInvInertia();
            //// normal denom
            {
                pe::Vector3 rxnA = rA.cross(ci.n);
                pe::Vector3 rxnB = rB.cross(ci.n);
                ci.nDenomInv = 1.0/(invMassSum + (worldInvInertiaA*rxnA).dot(rxnA) + (worldInvInertiaB*rxnB).dot(rxnB));
            }
            //// tangent denom 
            {
                pe::Vector3 rxnA = rA.cross(ci.t0);
                pe::Vector3 rxnB = rB.cross(ci.t0);
                ci.t0DenomInv = 1.0/(invMassSum + (worldInvInertiaA*rxnA).dot(rxnA) + (worldInvInertiaB*rxnB).dot(rxnB));
            }
            {
                pe::Vector3 rxnA = rA.cross(ci.t1);
                pe::Vector3 rxnB = rB.cross(ci.t1);
                ci.t1DenomInv = 1.0/(invMassSum + (worldInvInertiaA*rxnA).dot(rxnA) + (worldInvInertiaB*rxnB).dot(rxnB));
            }


            const pe::Vector3 velA = _body_a->getLinearVelocity() + _body_a->getAngularVelocity().cross(rA);
            const pe::Vector3 velB = _body_b->getLinearVelocity() + _body_b->getAngularVelocity().cross(rB);

            //// normal rhs
            {
                pe::Real revVelR = -ci.n.dot(velA-velB);
                // if(velR > - param.restitutionVelocityThreshold)
                if(std::abs(revVelR) < param.restitutionVelocityThreshold)
                    revVelR = 0;
                revVelR *= _contact_result.getRestitutionCoeff();

                pe::Real penetration = cp.getDistance();
                if(penetration < -param.penetrationThreshold)
                    penetration = -param.penetrationThreshold;
                if(param.splitPenetrationConstraintFlag){
                    ci.nRhs = revVelR * ci.nDenomInv;
                    ci.nPenetrationRhs = - param.kerp * penetration / param.dt * ci.nDenomInv;
                }else{
                    ci.nRhs = (revVelR - param.kerp * penetration / param.dt) * ci.nDenomInv;
                    ci.nPenetrationRhs = 0;
                }
            }

            ci.nAppliedImpulse = 0;
            ci.nAppliedPenetrationImpulse = 0;
            ci.t0AppliedImpulse = 0;
            ci.t1AppliedImpulse = 0;
            ci.frictionCoeff = _contact_result.getFrictionCoeff();
        }
    }

    void FrictionContactConstraint::afterSequentialImpulse() {
        for(int i = 0; i < _cis.size(); i++){
            const ConstraintInfo& ci = _cis[i];
            _contact_result.getContactPoint(i).setImpulse(ci.nAppliedImpulse * ci.n);
        }
    }

    void FrictionContactConstraint::iterateSequentialImpulse(int iter) {
        const int size = _cis.size();
        for(int i = 0; i < size; i++){
            ConstraintInfo& ci = _cis[i];
            const pe::Vector3& rA = ci.rA;
            const pe::Vector3& rB = ci.rB;
            const pe::Vector3& n = ci.n;
            const pe::Vector3& t0 = ci.t0;
            const pe::Vector3& t1 = ci.t1;
            const pe::Vector3 velR = (_body_a->getTempLinearVelocity() + _body_a->getTempAngularVelocity().cross(rA))
                    - (_body_b->getTempLinearVelocity() + _body_b->getTempAngularVelocity().cross(rB));

            //// compute impulse
            pe::Real nImpulse = ci.nRhs - n.dot(velR) * ci.nDenomInv;
            // nImpulse = std::max(nImpulse, -ci.nAppliedImpulse);
            if(nImpulse < -ci.nAppliedImpulse){
                nImpulse = -ci.nAppliedImpulse;
            }

            ci.nAppliedImpulse += nImpulse;

            //// TODO: fix me: incorrect coulomb cone(2 direction)
#           define PE_USE_COULOMB_CONE true
#           if PE_USE_COULOMB_CONE
                //// compute impulse
                pe::Real t0TotalImpulse = ci.t0AppliedImpulse -t0.dot(velR) * ci.t0DenomInv;
                const pe::Real maxFriction = ci.frictionCoeff * ci.nAppliedImpulse;
                pe::Real t1TotalImpulse = ci.t1AppliedImpulse -t1.dot(velR) * ci.t1DenomInv;

                // const Real maxFriction = ci.frictionCoeff * ci.nAppliedImpulse;
                t0TotalImpulse = std::min(t0TotalImpulse, maxFriction);
                t1TotalImpulse = std::min(t1TotalImpulse, maxFriction);
                t0TotalImpulse = std::max(t0TotalImpulse, -maxFriction);
                t1TotalImpulse = std::max(t1TotalImpulse, -maxFriction);
#           else
                Real t0TotalImpulse = ci.t0AppliedImpulse -t0.dot(velR) * ci.t0DenomInv;
                Real t1TotalImpulse = ci.t1AppliedImpulse -t1.dot(velR) * ci.t1DenomInv;
                const Real scale = ci.frictionCoeff * ci.nAppliedImpulse / sqrtr(t0TotalImpulse * t0TotalImpulse + t1TotalImpulse * t1TotalImpulse);
                if(scale < 1) {
                    t0TotalImpulse *= scale;
                    t1TotalImpulse *= scale;
                }
#           endif

            const pe::Vector3 impulseVector = nImpulse*n + (t0TotalImpulse - ci.t0AppliedImpulse)*t0 + (t1TotalImpulse - ci.t1AppliedImpulse)*t1;

            ci.t0AppliedImpulse = t0TotalImpulse;
            ci.t1AppliedImpulse = t1TotalImpulse;

            _body_a->applyTempImpulse(rA, impulseVector);
            _body_b->applyTempImpulse(rB, -impulseVector);
        }
    }

    void FrictionContactConstraint::iterateSequentialImpulseForPenetration(int iter) {
        int size = _cis.size();
        for(int i = 0; i < size; i++){
            ConstraintInfo& ci = _cis[i];
            const pe::Vector3& n = ci.n;
            const pe::Vector3 velR = (_body_a->getPenetrationLinearVelocity() + _body_a->getPenetrationAngularVelocity().cross(ci.rA))
                    - (_body_b->getPenetrationLinearVelocity() + _body_b->getPenetrationAngularVelocity().cross(ci.rB));

            //// compute impulse
            pe::Real tempImpulse = (ci.nPenetrationRhs - n.dot(velR) * ci.nDenomInv);

            //// clamp impulse, if it causes negative total impulse
            if(tempImpulse < -ci.nAppliedPenetrationImpulse){
                tempImpulse = -ci.nAppliedPenetrationImpulse;
            }

            // apply impulse
            ci.nAppliedPenetrationImpulse += tempImpulse;
            const pe::Vector3 impulseVector = tempImpulse * n;

            _body_a->applyPenetrationImpulse(ci.rA, impulseVector);
            _body_b->applyPenetrationImpulse(ci.rB, -impulseVector);
        }
    }

    void FrictionContactConstraint::warmStart() {
        for(int i = 0; i < _cis.size(); i++){
            pe_phys_collision::ContactPoint& cp = _contact_result.getContactPoint(i);
            ConstraintInfo& ci = _cis[i];
            ci.nAppliedImpulse = cp.getImpulse().dot(ci.n) * 0.9;
            _body_a->applyTempImpulse(ci.rA, ci.nAppliedImpulse * ci.n);
            _body_b->applyTempImpulse(ci.rB, -ci.nAppliedImpulse * ci.n);
        }
    }
    
} // namespace pe_phys_constraint