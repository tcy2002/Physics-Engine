#pragma once

#include "physics/physics.h"
#include "constraint.h"

namespace pe_physics_constraint {

    class SliderJointConstraint : public Constraint {
        COMMON_MEMBER_SET_GET(pe::Vector3, anchor_a, AnchorA)
        COMMON_MEMBER_SET_GET(pe::Vector3, anchor_b, AnchorB)
        COMMON_MEMBER_SET_GET(pe::Vector3, axis_a, AxisA)
        COMMON_MEMBER_SET_GET(pe::Vector3, axis_b, AxisB)

        COMMON_MEMBER_SET_GET(ConstraintLimitType, limit_type, LimitType)
        COMMON_MEMBER_SET_GET(pe::Real, min_position, MinPosition)
        COMMON_MEMBER_SET_GET(pe::Real, max_position, MaxPosition)

        COMMON_MEMBER_SET_GET(ConstraintMotorType, motor_type, MotorType)
        COMMON_MEMBER_SET_GET(pe::Real, target_speed, TargetSpeed)
        COMMON_MEMBER_SET_GET(pe::Real, target_position, TargetPosition)

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

        pe::Real _rhs_limit;
        pe::Real _rhs_motor;
        int _limit_exceeded_type;
        pe::Real _jmj_inv_motor_limit;
        pe::Real _total_impulse_limit;

    public:
        ConstraintType getType() const override { return ConstraintType::CT_SLIDER_JOINT; }

        SliderJointConstraint(): _anchor_a(pe::Vector3::zeros()), _anchor_b(pe::Vector3::zeros()),
                                 _axis_a(pe::Vector3::right()), _axis_b(pe::Vector3::right()) {}
        virtual ~SliderJointConstraint() = default;

        PE_API void initSequentialImpulse(const ConstraintParam& param) override;
        PE_API void iterateSequentialImpulse(int iter) override;
    };

} // namespace pe_physics_constraint
