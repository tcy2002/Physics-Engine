#pragma once

#include <mutex>
#include <functional>
#include "phys/shape/shape.h"

namespace pe_phys_object {

    class RigidBody {
        /* Base properties */
        COMMON_MEMBER_SET_GET(std::string, name, Name)
        COMMON_MEMBER_SET_GET(std::string, tag, Tag)
        COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)
        COMMON_BOOL_SET_GET(kinematic, Kinematic)
        COMMON_BOOL_SET_GET(ignore_collision, IgnoreCollision)
        COMMON_MEMBER_PTR_SET_GET(pe_phys_shape::Shape, collision_shape, CollisionShape)
        COMMON_MEMBER_GET(pe::Real, mass, Mass)
        COMMON_MEMBER_GET(pe::Real, inv_mass, InvMass)
    public:
        PE_API void setMass(pe::Real mass);
        COMMON_MEMBER_GET(pe::Matrix3, local_inertia, LocalInertia)
        COMMON_MEMBER_GET(pe::Matrix3, local_inv_inertia, LocalInvInertia)
    public:
        PE_API void setLocalInertia(const pe::Matrix3& local_inertia);
        COMMON_MEMBER_GET(pe::Matrix3, world_inertia, WorldInertia)
        COMMON_MEMBER_GET(pe::Matrix3, world_inv_inertia, WorldInvInertia)
    protected:
        PE_API void updateWorldInertia();

        /* Life Time */
        COMMON_MEMBER_SET_GET(pe::Real, life_time, LifeTime)
        pe::Real _last_time;

        /* Physical properties: friction, restitution, damping */
        COMMON_MEMBER_SET_GET(pe::Real, friction_coeff, FrictionCoeff)
        COMMON_MEMBER_SET_GET(pe::Real, restitution_coeff, RestitutionCoeff)
        COMMON_MEMBER_SET_GET(pe::Real, linear_damping, LinearDamping)
        COMMON_MEMBER_SET_GET(pe::Real, angular_damping, AngularDamping)

        /* Dynamic properties: transform, velocity, force, torque */
        COMMON_MEMBER_GET(pe::Transform, transform, Transform)
    public:
        PE_API void setTransform(const pe::Transform& transform);
        COMMON_MEMBER_SET_GET(pe::Vector3, linear_velocity, LinearVelocity)
        COMMON_MEMBER_SET_GET(pe::Vector3, angular_velocity, AngularVelocity)
        COMMON_MEMBER_GET(pe::Vector3, force, Force)
        COMMON_MEMBER_GET(pe::Vector3, torque, Torque)
        // Temp velocity should be protected by lock
    protected:
        pe::Vector3 _temp_linear_velocity;
        pe::Vector3 _temp_angular_velocity;
        std::mutex _temp_linear_velocity_mutex;
        std::mutex _temp_angular_velocity_mutex;
    public:
        PE_API pe::Vector3 getTempLinearVelocity();
        PE_API pe::Vector3 getTempAngularVelocity();
        PE_API void setTempLinearVelocity(const pe::Vector3& v);
        PE_API void setTempAngularVelocity(const pe::Vector3& v);

        /* Geometric properties for collision detection */
        COMMON_MEMBER_GET(pe::Vector3, aabb_min, AABBMin)
        COMMON_MEMBER_GET(pe::Vector3, aabb_max, AABBMax)
    private:
        static std::atomic<uint32_t> _globalIdCounter;
        pe::Array<uint32_t> _ignore_collision_ids;

        /* Sleep */
        COMMON_BOOL_SET_GET(sleep, Sleep)
        COMMON_MEMBER_GET(pe::Real, sleep_time, SleepTime)

        /* Collision Callback */
    public:
        typedef std::function<void(RigidBody*, RigidBody*,
                const pe::Vector3&, const pe::Vector3&, const pe::Vector3&)> CollisionCallback;
    private:
        pe::Array<CollisionCallback> _collision_callbacks;
    public:
        PE_API const pe::Array<CollisionCallback>& getCollisionCallbacks() const { return _collision_callbacks; }
        PE_API void addCollisionCallback(CollisionCallback&& callback)
        { _collision_callbacks.push_back(std::move(callback)); }
        PE_API void clearCollisionCallbacks() { _collision_callbacks.clear(); }

        /* Constructor */
    public:
        PE_API RigidBody();
        virtual ~RigidBody() { setCollisionShape(0); }

        virtual bool isDeformable() const { return false; }
        virtual bool isFracturable() const { return false; }

        void updateSleepTime(pe::Real dt) { _sleep_time += dt; }
        void resetSleepTime() { _sleep_time = 0; }

        PE_API void computeAABB();
        PE_API pe::Real getAABBScale() const;

        void addIgnoreCollisionId(uint32_t id) { _ignore_collision_ids.push_back(id); }
        PE_API void removeIgnoreCollisionId(uint32_t id);
        PE_API bool isIgnoreCollisionId(uint32_t id) const;

        pe::Vector3 getWorldLinearMomentum() const { return _linear_velocity * _mass; }
        pe::Vector3 getWorldAngularMomentum() const { return _world_inertia * _angular_velocity; }
        PE_API pe::Vector3 getLinearVelocityAtLocalPoint(const pe::Vector3& local_p) const;
        PE_API pe::Real getKineticEnergy();
        PE_API pe::Real getImpulseDenominator(const pe::Vector3& world_point, const pe::Vector3& world_normal) const;

        PE_API void syncTempVelocity();
        PE_API void clearTempVelocity();

        PE_API void applyTempImpulse(const pe::Vector3& world_rel_vec, const pe::Vector3& impulse);
        PE_API void applyImpulse(const pe::Vector3& world_rel_vec, const pe::Vector3& impulse);

        PE_API void addForce(const pe::Vector3& world_point, const pe::Vector3& force);
        PE_API void addCentralForce(const pe::Vector3& force) { _force += force; }
        PE_API void addTorque(const pe::Vector3& torque) { _torque += torque; }

        PE_API void applyForce(pe::Real dt);
        PE_API void applyDamping(pe::Real dt);

        PE_API bool step(pe::Real dt);
    };

} // namespace pe_phys_object