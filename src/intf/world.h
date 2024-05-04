#pragma once

#include "common/vector3.h"
#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"
#include "phys/collision/broad_phase/broad_phase_base.h"
#include "phys/collision/narrow_phase/narrow_phase_base.h"
#include "phys/constraint/constraint/constraint.h"
#include "phys/constraint/constraint_solver/constraint_solver.h"
#include "phys/fracture/fracture_solver/simple_fracture_solver.h"

// NO include: will cause circular dependency
namespace pe_phys_vehicle { class ContactVehicle; }

namespace pe_intf { // interface

    /*
     * @brief The basic physics world manager
     * You can get a physics instance by new World(), add rigidbodies to
     * the world, and call step() to update it in each frame.
     * NOT thread-safe
     */
    class World {
        COMMON_MEMBER_SET_GET(pe::Vector3, gravity, Gravity);
        COMMON_MEMBER_SET_GET(pe::Real, dt, Dt);
        COMMON_MEMBER_SET_GET(pe::Real, sleep_lin_vel2_threshold, SleepLinVel2Threshold);
        COMMON_MEMBER_SET_GET(pe::Real, sleep_ang_vel2_threshold, SleepAngVel2Threshold);
        COMMON_MEMBER_SET_GET(pe::Real, sleep_time_threshold, SleepTimeThreshold);

    protected:
        pe::Array<pe_phys_object::RigidBody*> _collision_objects;
        pe::Array<pe_phys_constraint::Constraint*> _constraints;
        pe_phys_collision::BroadPhaseBase* _broad_phase;
        pe_phys_collision::NarrowPhaseBase* _narrow_phase;
        pe_phys_constraint::ConstraintSolver* _constraint_solver;
        pe_phys_fracture::FractureSolver* _fracture_solver;

        pe::Array<pe_phys_fracture::FractureSource> _fracture_sources;
        pe::Array<pe_phys_object::RigidBody*> _rigidbodies_to_add;
        pe::Array<pe_phys_object::RigidBody*> _rigidbodies_to_remove;
        pe::Array<pe_phys_collision::CollisionPair> _collision_pairs;
        pe::Array<pe_phys_collision::ContactResult*> _contact_results;
        friend class pe_phys_vehicle::ContactVehicle;

        void updateAABBs();
        void updateObjectStatus();
        void applyExternalForce();
        void execCollisionCallbacks();

    public:
        PE_API World();
        PE_API ~World();

        /**** for performance analysis and debug *****/
        pe::Real update_status_time = pe::Real(0.0);
        pe::Real broad_phase_time = pe::Real(0.0);
        pe::Real narrow_phase_time = pe::Real(0.0);
        pe::Real constraint_solver_time = pe::Real(0.0);
        const pe::Array<pe_phys_collision::ContactResult*>& getContactResults() { return _contact_results; }
        /*********************************************/

        /**** rigid body *****************************/
        const std::vector<pe_phys_object::RigidBody*>& getRigidBodies() const { return _collision_objects; }
        pe_phys_object::RigidBody* getRigidBody(uint32_t idx) { return _collision_objects[idx]; }
        PE_API void addRigidBody(pe_phys_object::RigidBody* rigidbody);
        PE_API void removeRigidBody(pe_phys_object::RigidBody* rigidbody);
        PE_API const pe::Array<pe_phys_object::RigidBody*>& getRigidBodiesToAdd() { return _rigidbodies_to_add; }
        PE_API void clearRigidBodiesToAdd() { _rigidbodies_to_add.clear(); }
        PE_API const pe::Array<pe_phys_object::RigidBody*>& getRigidBodiesToRemove() { return _rigidbodies_to_remove; }
        PE_API void clearRigidBodiesToRemove() { _rigidbodies_to_remove.clear(); }
        /*********************************************/

        /**** fracture *******************************/
        PE_API void addFractureSource(const pe_phys_fracture::FractureSource& source) { _fracture_sources.push_back(source); }
        /*********************************************/

        /**** world advance **************************/
        PE_API void step();
        /*********************************************/
    };

} // namespace pe_core
