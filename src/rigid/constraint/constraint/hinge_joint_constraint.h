#pragma once

#include "rigid/phys_general.h"
#include "constraint.h"

namespace pe_phys_constraint {

    class HingeJointConstraint : public Constraint {
        COMMON_MEMBER_SET_GET(pe::Vector3, anchor_a, AnchorA)
        COMMON_MEMBER_SET_GET(pe::Vector3, anchor_b, AnchorB)
        COMMON_MEMBER_SET_GET(pe::Vector3, axis_a, AxisA)
        COMMON_MEMBER_SET_GET(pe::Vector3, axis_b, AxisB)

    protected:
        pe::Vector3 _r_a;
        pe::Vector3 _r_b;
        pe::Vector3 _w_axis_a;
        pe::Vector3 _w_axis_b;
        pe::Vector3 _w_t_a[2]{};

        pe::Vector3 _rhs_ball;
        pe::Matrix3 _jmj_inv_ball;

        pe::Real _rhs_hinge[2]{};
        pe::Real _jmj_inv_hinge[2]{};

    public:
        ConstraintType getType() const override { return ConstraintType::CT_HINGE_JOINT; }

        HingeJointConstraint(): _anchor_a(pe::Vector3::Zero()), _anchor_b(pe::Vector3::Zero()),
                                _axis_a(pe::Vector3::UnitX()), _axis_b(pe::Vector3::UnitX()) {}
        virtual ~HingeJointConstraint() {}

        PE_API void initSequentialImpulse(const ConstraintParam& param) override;
        PE_API void iterateSequentialImpulse(int iter) override;
    };

} // namespace pe_phys_constraint