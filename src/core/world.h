#pragma once

#include "common/vector3.h"
#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"
#include "phys/collision/broad_phase/broad_phase_base.h"
#include "phys/collision/narrow_phase/narrow_phase_base.h"
#include "phys/constraint/constraint/constraint.h"
#include "phys/constraint/constraint_solver/constraint_solver.h"

namespace pe_core {

    class World {
        COMMON_MEMBER_SET_GET(pe::Vector3, gravity, Gravity);
        COMMON_MEMBER_SET_GET(pe::Real, dt, Dt);

    protected:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        pe::Array<pe_phys_constraint::Constraint*> _constraints;
        pe_phys_collision::BroadPhaseBase* _broad_phase;
        pe_phys_collision::NarrowPhaseBase* _narrow_phase;
        pe_phys_constraint::ConstraintSolver* _constraint_solver;

        void updateAABBs();
        void updateObjectStatus();
        void applyExternalForce();

    public:
        World();
        ~World();

        const std::vector<pe_phys_object::RigidBody*>& getRigidBodies() const { return _collision_objects; }
        void addRigidBody(pe_phys_object::RigidBody* rigidbody);
        void step();
    };

} // namespace pe_core
