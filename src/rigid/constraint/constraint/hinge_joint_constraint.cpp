#include "hinge_joint_constraint.h"
#include "rigid/collision/narrow_phase/contact_result.h"

namespace pe_phys_constraint {

    void HingeJointConstraint::initSequentialImpulse(const ConstraintParam &param) {
        auto& transA = _object_a->getTransform();
        auto& transB = _object_b->getTransform();
        _r_a = transA.getBasis() * _anchor_a;
        _r_b = transB.getBasis() * _anchor_b;

        _w_axis_a = transA.getBasis() * _axis_a;
        _w_axis_b = transB.getBasis() * _axis_b;
        pe_phys_collision::ContactPoint::getOrthoUnits(_axis_a, _w_t_a[0], _w_t_a[1]);
        _w_t_a[0] = transA.getBasis() * _w_t_a[0];
        _w_t_a[1] = transA.getBasis() * _w_t_a[1];


        // ball jmj
        pe::Matrix3 rxA, rxB;
        getSkewSymmetricMatrix(_r_a, rxA);
        getSkewSymmetricMatrix(_r_b, rxB);
        _jmj_inv_ball = (pe::Matrix3::Identity() * (_object_a->getKinematicInvMass() + _object_b->getKinematicInvMass()) +
            rxA * _object_a->getKinematicWorldInvInertia() * rxA.transpose() +
            rxB * _object_b->getKinematicWorldInvInertia() * rxB.transpose()).inverse();

        // ball rhs
        _rhs_ball = _jmj_inv_ball * (transA * _anchor_a - transB * _anchor_b) * (-param.kerp / param.dt);

        // hinge jmj
        const pe::Matrix3& inv_inertia_sum = _object_a->getKinematicWorldInvInertia() + _object_b->getKinematicWorldInvInertia();
        _jmj_inv_hinge[0] = pe::Real(1.0) / _w_t_a[0].dot(inv_inertia_sum * _w_t_a[0]);
        _jmj_inv_hinge[1] = pe::Real(1.0) / _w_t_a[1].dot(inv_inertia_sum * _w_t_a[1]);

        // hinge rhs
        const pe::Vector3& u = _w_axis_a.cross(_w_axis_b);
        _rhs_hinge[0] = _jmj_inv_hinge[0] * (u.dot(_w_t_a[0]) * param.kerp / param.dt);
        _rhs_hinge[1] = _jmj_inv_hinge[1] * (u.dot(_w_t_a[1]) * param.kerp / param.dt);
    }

    void HingeJointConstraint::iterateSequentialImpulse(int iter) {
        // position impulse
        const pe::Vector3& vel_a = _object_a->getTempLinearVelocity() + _object_a->getTempAngularVelocity().cross(_r_a);
        const pe::Vector3& vel_b = _object_b->getTempLinearVelocity() + _object_b->getTempAngularVelocity().cross(_r_b);
        const pe::Vector3& ball_impulse = _rhs_ball - _jmj_inv_ball * (vel_a - vel_b);
        _object_a->applyTempImpulse(_r_a, ball_impulse);
        _object_b->applyTempImpulse(_r_b, -ball_impulse);

        // rotation impulse
        const pe::Vector3& w_a = _object_a->getTempAngularVelocity();
        const pe::Vector3& w_b = _object_b->getTempAngularVelocity();
        const pe::Real& hinge_impulse0 = _rhs_hinge[0] - _jmj_inv_hinge[0] * _w_t_a[0].dot(w_a - w_b);
        const pe::Real& hinge_impulse1 = _rhs_hinge[1] - _jmj_inv_hinge[1] * _w_t_a[1].dot(w_a - w_b);
        _object_a->applyTempAngularImpulse(_w_t_a[0] * hinge_impulse0 + _w_t_a[1] * hinge_impulse1);
        _object_b->applyTempAngularImpulse(-_w_t_a[0] * hinge_impulse0 - _w_t_a[1] * hinge_impulse1);
    }


} // namespace pe_phys_constraint