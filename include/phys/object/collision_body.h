#pragma once

#include "phys/shape/shape.h"

namespace pe_phys_object {

class CollisionBody {
    static uint32_t _globalIdCounter;

    COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)
    COMMON_BOOL_SET_GET(kinematic, Kinematic)
    COMMON_BOOL_SET_GET(active, Active)
    COMMON_MEMBER_PTR_SET_GET(pe_phys_shape::Shape, shape, Shape)

    COMMON_MEMBER_GET(pe::Real, mass, Mass)
    COMMON_MEMBER_GET(pe::Real, inv_mass, InvMass)
public:
    void setMass(pe::Real mass) { _mass = mass; _inv_mass = 1. / mass; }

    COMMON_MEMBER_SET_GET(pe::Real, friction_coeff, FrictionCoeff)
    COMMON_MEMBER_SET_GET(pe::Real, restitution_coeff, RestitutionCoeff)
    COMMON_MEMBER_GET(pe::Real, linear_damping, LinearDamping)
    COMMON_MEMBER_GET(pe::Real, angular_damping, AngularDamping)

    COMMON_MEMBER_SET_GET(pe::Transform, transform, Transform)
    COMMON_MEMBER_SET_GET(pe::Vector3, velocity, Velocity)
    COMMON_MEMBER_SET_GET(pe::Vector3, angular_velocity, AngularVelocity)

public:
    CollisionBody():
        _global_id(++_globalIdCounter),
        _kinematic(false),
        _active(true),
        _mass(1.),
        _inv_mass(1.),
        _shape(nullptr),
        _transform(pe::Transform::identity()),
        _velocity(pe::Vector3::zeros()),
        _friction_coeff(0.5),
        _restitution_coeff(0.5),
        _linear_damping(0.),
        _angular_damping(0.) {}
    ~CollisionBody() { setShape(0); }

    virtual bool isDeformable() const = 0;
    void applyForce(const pe::Vector3& force, const pe::Vector3& point, pe::Real dt);
    void updateTransform(pe::Real dt);
};

} // namespace pe_phys_object
