#pragma once

#include "physics/physics.h"
#include "physics/constraint/constraint/constraint.h"
#include "physics/collision/narrow_phase/contact_result.h"

namespace pe_physics_constraint {

enum class ConstraintSolverType {
    CST_SEQUENTIAL_IMPULSE,
    CST_PRIMAL_DUAL,
};

class ConstraintSolver {
    COMMON_MEMBER_SET_GET(ConstraintParam, param, Param);
    COMMON_MEMBER_SET_GET(int, iteration, Iteration);

public:
    ConstraintSolver(): _param(ConstraintParam()), _iteration(1) {}
    virtual ~ConstraintSolver() {}
    virtual ConstraintSolverType getType() const = 0;
    virtual void setupSolver(
            pe::Real dt, const pe::Vector3& gravity,
            const pe::Array<pe_physics_object::RigidBody*>& objects,
            const pe::Array<pe_physics_collision::ContactResult*>& contact_results,
            const pe::Array<Constraint*>& constraints) = 0;
    virtual void solve() = 0;
};

} // namespace pe_physics_constraint
