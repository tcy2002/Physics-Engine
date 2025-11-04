#pragma once

#include "physics/physics.h"
#include "constraint.h"

namespace pe_physics_constraint {

class SixDofConstraint : public Constraint {
    COMMON_MEMBER_SET_GET(pe::Transform, frame_a, FrameA)
    COMMON_MEMBER_SET_GET(pe::Transform, frame_b, FrameB)

protected:
    bool _pos_fixed[3]{ false, false, false };
    bool _rot_fixed[3]{ false, false, false };

    pe::Vector3 _r_a, _r_b;
    pe::Matrix3 _w_axis_a, _w_axis_b;
    pe::Vector3 _axb[3]{};
    pe::Vector3 _nxn[3]{};

    pe::Vector3 _rhs_pos;
    pe::Vector3 _jmj_inv_pos;
    pe::Vector3 _rhs_rot;
    pe::Vector3 _jmj_inv_rot;

    // limits and motor for rotation
    ConstraintLimitType _limit_type[3]{ ConstraintLimitType::CLT_NONE, ConstraintLimitType::CLT_NONE, ConstraintLimitType::CLT_NONE };
    pe::Vector3 _min_angle, _max_angle;
    ConstraintMotorType _motor_type[3]{ ConstraintMotorType::CMT_NONE, ConstraintMotorType::CMT_NONE, ConstraintMotorType::CMT_NONE };
    pe::Vector3 _target_speed, _target_angle;

    pe::Vector3 _rhs_limit, _rhs_motor;
    pe::Vector3 _jmj_inv_motor_limit;
    pe::Vector3 _total_impulse_limit;
    int _limit_exceeded_type[3]{ 0, 0, 0 };

public:
    ConstraintType getType() const override { return ConstraintType::CT_SIX_DOF; }

    SixDofConstraint(): _frame_a(pe::Transform::identity()), _frame_b(pe::Transform::identity()) {}
    virtual ~SixDofConstraint() = default;

    void setXPosFixed(bool fixed) { _pos_fixed[0] = fixed; }
    void setYPosFixed(bool fixed) { _pos_fixed[1] = fixed; }
    void setZPosFixed(bool fixed) { _pos_fixed[2] = fixed; }
    void setXRotFixed(bool fixed) { _rot_fixed[0] = fixed; }
    void setYRotFixed(bool fixed) { _rot_fixed[1] = fixed; }
    void setZRotFixed(bool fixed) { _rot_fixed[2] = fixed; }

    void setXRotLimitType(ConstraintLimitType type) { _limit_type[0] = type; }
    void setYRotLimitType(ConstraintLimitType type) { _limit_type[1] = type; }
    void setZRotLimitType(ConstraintLimitType type) { _limit_type[2] = type; }
    void setXRotMotorType(ConstraintMotorType type) { _motor_type[0] = type; }
    void setYRotMotorType(ConstraintMotorType type) { _motor_type[1] = type; }
    void setZRotMotorType(ConstraintMotorType type) { _motor_type[2] = type; }

    void setMinAngleX(pe::Real angle) { _min_angle.x = angle; }
    void setMinAngleY(pe::Real angle) { _min_angle.y = angle; }
    void setMinAngleZ(pe::Real angle) { _min_angle.z = angle; }
    void setMaxAngleX(pe::Real angle) { _max_angle.x = angle; }
    void setMaxAngleY(pe::Real angle) { _max_angle.y = angle; }
    void setMaxAngleZ(pe::Real angle) { _max_angle.z = angle; }

    void setTargetSpeedX(pe::Real speed) { _target_speed.x = speed; }
    void setTargetSpeedY(pe::Real speed) { _target_speed.y = speed; }
    void setTargetSpeedZ(pe::Real speed) { _target_speed.z = speed; }
    void setTargetAngleX(pe::Real angle) { _target_angle.x = angle; }
    void setTargetAngleY(pe::Real angle) { _target_angle.y = angle; }
    void setTargetAngleZ(pe::Real angle) { _target_angle.z = angle; }

    PE_API void initSequentialImpulse(const ConstraintParam& param) override;
    PE_API void iterateSequentialImpulse(int iter) override;
};

} // namespace pe_physics_constraint
