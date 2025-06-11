#pragma once

#include "rigid/phys_general.h"
#include "constraint.h"

namespace pe_phys_constraint {

    class BallJointConstraint : public Constraint {
        COMMON_MEMBER_SET_GET(pe::Vector3, anchor_a, AnchorA)
        COMMON_MEMBER_SET_GET(pe::Vector3, anchor_b, AnchorB)

    protected:
        pe::Vector3 _r_a;
        pe::Vector3 _r_b;
        pe::Vector3 _rhs;
        pe::Matrix3 _jmj_inv;

    public:
        ConstraintType getType() const override { return ConstraintType::CT_BALL_JOINT; }

        BallJointConstraint(): _anchor_a(pe::Vector3::Zero()), _anchor_b(pe::Vector3::Zero()) {}
        virtual ~BallJointConstraint() {}

        PE_API void initSequentialImpulse(const ConstraintParam& param) override;
        PE_API void iterateSequentialImpulse(int iter) override;

        static void getSkewSymmetricMatrix(const pe::Vector3& v, pe::Matrix3& m);
    };

} // namespace pe_phys_constraint