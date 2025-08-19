#pragma once

#include "rigid/phys_general.h"
#include "constraint.h"

namespace pe_phys_constraint {

    class SliderJointConstraint : public Constraint {
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
        pe::Vector3 _w_t_b[2]{};

        pe::Real _rhs_pos[2]{};
        pe::Real _jmj_inv_pos[2]{};

        pe::Vector3 _rhs_rot;
        pe::Matrix3 _jmj_inv_rot;

    public:
        ConstraintType getType() const override { return ConstraintType::CT_SLIDER_JOINT; }

        SliderJointConstraint(): _anchor_a(pe::Vector3::Zero()), _anchor_b(pe::Vector3::Zero()) {}
        virtual ~SliderJointConstraint() {}

        PE_API void initSequentialImpulse(const ConstraintParam& param) override;
        PE_API void iterateSequentialImpulse(int iter) override;
    };

} // namespace pe_phys_constraint