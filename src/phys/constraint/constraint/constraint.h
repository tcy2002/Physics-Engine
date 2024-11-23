#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"

namespace pe_phys_constraint {

    enum ConstraintType {
        CT_BALL_JOINT,
        CT_FRICTION_CONTACT,
    };

    struct ConstraintParam {
        pe::Real dt = pe::Real(0.01);
        pe::Real restitutionVelocityThreshold = pe::Real(0.1);
        pe::Real penetrationThreshold = pe::Real(0.3);
        pe::Real kerp = pe::Real(0.2);
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

        // projected gauss seidel
        virtual void initSequentialImpulse(const ConstraintParam& param) = 0;
        virtual void warmStart() = 0;
        virtual void iterateSequentialImpulse(int iter) = 0;

        static void getSkewSymmetricMatrix(const pe::Vector3& v, pe::Matrix3& m);
    };

} // namespace pe_phys_constraint