#pragma once

#include "physics/physics.h"
#include "physics/object/rigidbody.h"

namespace pe_physics_constraint {

enum class ConstraintType {
    CT_BALL_JOINT,
    CT_HINGE_JOINT,
    CT_SLIDER_JOINT,
    CT_SIX_DOF,
    CT_FRICTION_CONTACT,
};

struct ConstraintParam {
    pe::Real dt = PE_R(0.01);

    // correction factor
    pe::Real kerp = PE_R(0.2);
};

enum class ConstraintLimitType {
    CLT_NONE,
    CLT_LOWER,
    CLT_UPPER,
    CLT_LOWER_UPPER,
};

enum class ConstraintMotorType {
    CMT_NONE,
    CMT_VELOCITY,
    CMT_POSITION,
};

class Constraint {
    COMMON_MEMBER_PTR_GET(pe_physics_object::RigidBody, object_a, ObjectA)
    COMMON_MEMBER_PTR_GET(pe_physics_object::RigidBody, object_b, ObjectB)
    COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)

public:
    PE_API void setObjectA(pe_physics_object::RigidBody* obj);
    PE_API void setObjectB(pe_physics_object::RigidBody* obj);

protected:
    static std::atomic<uint32_t> _globalIdCounter;

public:
    PE_API Constraint();
    virtual ~Constraint() = default;

    virtual ConstraintType getType() const = 0;

    // for sequential impulse solver
    virtual void initSequentialImpulse(const ConstraintParam& param) {}
    virtual void iterateSequentialImpulse(int iter) {}

    static pe_physics_object::RigidBody* getStaticBody();
    static void getSkewSymmetricMatrix(const pe::Vector3& v, pe::Matrix3& m);
};

} // namespace pe_physics_constraint
