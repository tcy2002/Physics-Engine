#include "phys/phys_general.h"
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
        virtual ConstraintType getType() const override { return CT_BALL_JOINT; }

        BallJointConstraint(): _anchor_a(pe::Vector3::zeros()), _anchor_b(pe::Vector3::zeros()) {}
        virtual ~BallJointConstraint() {}

        PE_API virtual void initSequentialImpulse(const ConstraintParam& param) override;
        virtual void warmStart() override {}
        PE_API virtual void iterateSequentialImpulse(int iter) override;
        virtual void afterSequentialImpulse() override {}
    };

} // namespace pe_phys_constraint