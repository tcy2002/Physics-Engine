#pragma once

#include "physics/physics.h"
#include "physics/collision/narrow_phase/contact_result.h"
#include "constraint.h"

#define PE_MAX_CONTACT_POINT 8

namespace pe_physics_constraint {

class FrictionContactConstraint : public Constraint {
private:
    struct ConstraintInfo {
        pe::Vector3 r_a;
        pe::Vector3 r_b;
        pe::Vector3 n;
        pe::Vector3 t0;
        pe::Vector3 t1;
        pe::Real n_rhs = 0;
        pe::Real n_jmj_inv = 0;
        pe::Real t0_jmj_inv = 0;
        pe::Real t1_jmj_inv = 0;
        pe::Real n_applied_impulse = 0;
        pe::Real t0_applied_impulse = 0;
        pe::Real t1_applied_impulse = 0;
        pe::Real friction_coeff = 0;
    };
    pe::Array<ConstraintInfo> _cis;
    pe_physics_collision::ContactResult* _contact_result = nullptr;

public:
    void setContactResult(pe_physics_collision::ContactResult& cr) { _contact_result = &cr; }
    ConstraintType getType() const override { return ConstraintType::CT_FRICTION_CONTACT; }

    FrictionContactConstraint() = default;
    virtual ~FrictionContactConstraint() = default;

    void initSequentialImpulse(const ConstraintParam& param) override;
    void iterateSequentialImpulse(int iter) override;
};

} // namespace pe_physics_constraint
