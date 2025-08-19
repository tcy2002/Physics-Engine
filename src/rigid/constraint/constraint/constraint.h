#pragma once

#include "rigid/phys_general.h"
#include "rigid/object/rigidbody.h"

namespace pe_phys_constraint {

    enum class ConstraintType {
        CT_BALL_JOINT,
        CT_HINGE_JOINT,
        CT_SLIDER_JOINT,
        CT_FRICTION_CONTACT,
    };

    struct ConstraintParam {
        pe::Real dt = PE_R(0.01);

        // for sequential impulse solver
        pe::Real restitutionVelocityThreshold = PE_R(0.1);
        pe::Real penetrationThreshold = PE_R(0.3);
        pe::Real kerp = PE_R(0.2);

        // for primal-dual solver
        pe::Vector3 gravity;
    };

    class Constraint {
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::RigidBody, object_a, ObjectA)
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::RigidBody, object_b, ObjectB)
        COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)

    protected:
        static std::atomic<uint32_t> _globalIdCounter;

    public:
        PE_API Constraint();
        virtual ~Constraint() {}

        virtual ConstraintType getType() const = 0;

        // for sequential impulse solver
        virtual void initSequentialImpulse(const ConstraintParam& param) {}
        virtual void iterateSequentialImpulse(int iter) {}

        // for primal-dual solver
        virtual void initPrimalDual(const ConstraintParam& param) {}
        virtual bool iteratePrimalDual(int iter, pe::LDLT& ldlt, pe::Real& hu,
                                       pe::VectorX& du, pe::VectorX& df, pe::VectorX& dl,
                                       pe::VectorX& ru, pe::VectorX& ru_add,
                                       pe::VectorX& rf, pe::VectorX& wrf, pe::VectorX& rl,
                                       pe::Real& exit_err, pe::Real tol) { return false; }
        virtual void afterPrimalDual() {}

        static void getSkewSymmetricMatrix(const pe::Vector3& v, pe::Matrix3& m);
    };

} // namespace pe_phys_constraint