#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_constraint {

    enum class ConstraintType {
        CT_BALL_JOINT,
        CT_FRICTION_CONTACT,
    };

    struct ConstraintParam {
        pe::Real dt = R(0.01);

        // for sequential impulse solver
        pe::Real restitutionVelocityThreshold = R(0.1);
        pe::Real penetrationThreshold = R(0.3);
        pe::Real kerp = R(0.2);

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
        virtual void iteratePrimalDual(int iter) {}
    };

} // namespace pe_phys_constraint