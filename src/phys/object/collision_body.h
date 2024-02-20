#pragma once

#include <atomic>
#include "phys/shape/shape.h"

namespace pe_phys_object {

class CollisionBody {
protected:
    COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)
    COMMON_BOOL_SET_GET(kinematic, Kinematic)
    COMMON_MEMBER_PTR_SET_GET(pe_phys_shape::Shape, collision_shape, CollisionShape)

    COMMON_MEMBER_SET_GET(pe::Real, mass, Mass)
    COMMON_MEMBER_SET_GET(pe::Real, inv_mass, InvMass)
    COMMON_MEMBER_SET_GET(pe::Matrix3, inertia, Inertia)
    COMMON_MEMBER_SET_GET(pe::Matrix3, inv_inertia, InvInertia)

    COMMON_MEMBER_SET_GET(pe::Real, friction_coeff, FrictionCoeff)
    COMMON_MEMBER_SET_GET(pe::Real, restitution_coeff, RestitutionCoeff)
    COMMON_MEMBER_SET_GET(pe::Real, linear_damping, LinearDamping)
    COMMON_MEMBER_SET_GET(pe::Real, angular_damping, AngularDamping)

    COMMON_MEMBER_SET_GET(pe::Transform, transform, Transform)
    COMMON_MEMBER_SET_GET(pe::Vector3, linear_velocity, LinearVelocity)
    COMMON_MEMBER_SET_GET(pe::Vector3, angular_velocity, AngularVelocity)

    COMMON_MEMBER_GET(pe::Vector3, aabb_min, AABBMin)
    COMMON_MEMBER_GET(pe::Vector3, aabb_max, AABBMax)

private:
    static std::atomic<uint32_t> _globalIdCounter;
    pe::Array<uint32_t> _ignore_collision_ids;

public:
    CollisionBody();
    virtual ~CollisionBody() { setCollisionShape(0); }

    virtual bool isDeformable() const = 0;
    void computeAABB();
    pe::Real getAABBScale() const;
    void addIgnoreCollisionId(uint32_t id) { _ignore_collision_ids.push_back(id); }
    void removeIgnoreCollisionId(uint32_t id);
    bool isIgnoreCollisionId(uint32_t id) const;
};

} // namespace pe_phys_object