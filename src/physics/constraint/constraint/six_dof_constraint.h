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
    int _rot_fixed_count = 0;

    pe::Vector3 _r_a, _r_b;
    pe::Matrix3 _w_axis_a, _w_axis_b;
    pe::Vector3 _axb[3]{};

    pe::Vector3 _rhs_pos;
    pe::Vector3 _jmj_inv_pos;
    pe::Vector3 _rhs_rot;
    pe::Vector3 _jmj_inv_rot;

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

    PE_API void initSequentialImpulse(const ConstraintParam& param) override;
    PE_API void iterateSequentialImpulse(int iter) override;
};

} // namespace pe_physics_constraint
