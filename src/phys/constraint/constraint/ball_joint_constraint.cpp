#include "ball_joint_constraint.h"

namespace pe_phys_constraint {

    void BallJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
        auto& transA = _object_a->getTransform();
        auto& transB = _object_b->getTransform();
        _r_a = transA.getBasis() * _anchor_a;
        _r_b = transB.getBasis() * _anchor_b;

        pe::Matrix3 rxA, rxB;
        getSkewSymmetricMatrix(_r_a, rxA);
        getSkewSymmetricMatrix(_r_b, rxB);
        _jmj_inv = (pe::Matrix3::identity() * (_object_a->getInvMass() + _object_b->getInvMass()) + rxA * _object_a->getWorldInvInertia() * rxA.transposed() + rxB * _object_b->getWorldInvInertia() * rxB.transposed()).inverse();
        _rhs = _jmj_inv * (transA * _anchor_a - transB * _anchor_b) * (-param.kerp / param.dt);
    }

    void BallJointConstraint::iterateSequentialImpulse(int iter) {
        const pe::Vector3 vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
        const pe::Vector3 vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
        const pe::Vector3 tmp_impulse = _rhs - _jmj_inv * (vel_a - vel_b);
        _object_a->applyTempImpulse(_r_a, tmp_impulse);
        _object_b->applyTempImpulse(_r_b, -tmp_impulse);
    }


} // namespace pe_phys_constraint