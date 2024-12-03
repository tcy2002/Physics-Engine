#include "ball_joint_constraint.h"

// style-checked
namespace pe_phys_constraint {

    void BallJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
        auto& transA = _object_a->getTransform();
        auto& transB = _object_b->getTransform();
        _r_a = transA.getBasis() * _anchor_a;
        _r_b = transB.getBasis() * _anchor_b;

        pe::Matrix3 rxA, rxB;
        getSkewSymmetricMatrix(_r_a, rxA);
        getSkewSymmetricMatrix(_r_b, rxB);
        _jmj_inv = (pe::Matrix3::Identity() * (_object_a->getInvMass() + _object_b->getInvMass()) + rxA * _object_a->getWorldInvInertia() * rxA.transpose() + rxB * _object_b->getWorldInvInertia() * rxB.transpose()).inverse();
        _rhs = _jmj_inv * (transA * _anchor_a - transB * _anchor_b) * (-param.kerp / param.dt);
    }

    void BallJointConstraint::iterateSequentialImpulse(int iter) {
        const pe::Vector3 vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
        const pe::Vector3 vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
        const pe::Vector3 tmp_impulse = _rhs - _jmj_inv * (vel_a - vel_b);
        _object_a->applyTempImpulse(_r_a, tmp_impulse);
        _object_b->applyTempImpulse(_r_b, -tmp_impulse);
    }

    void BallJointConstraint::getSkewSymmetricMatrix(const pe::Vector3& v, pe::Matrix3& m) {
        m(0, 0) = 0; m(0, 1) = -v.z(); m(0, 2) = v.y();
        m(1, 0) = v.z(); m(1, 1) = 0; m(1, 2) = -v.x();
        m(2, 0) = -v.y(); m(2, 1) = v.x(); m(2, 2) = 0;
    }

} // namespace pe_phys_constraint