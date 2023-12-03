#pragma once

#include "phys/shape/shape.h"

namespace pe_phys_object {

class CollisionBody {
    static uint32_t _globalIdCounter;

    PE_MEMBER_GET(uint32_t, global_id, GlobalId)
    PE_BOOL_SET_GET(kinematic, Kinematic)
    PE_BOOL_SET_GET(active, Active)
    PE_MEMBER_PTR_SET_GET(pe_phys_shape::Shape, shape, Shape)

    PE_MEMBER_GET(PEReal, mass, Mass)
    PE_MEMBER_GET(PEReal, inv_mass, InvMass)
public:
    void setMass(PEReal mass) { _mass = mass; _inv_mass = 1. / mass; }

    PE_MEMBER_SET_GET(PEReal, friction_coeff, FrictionCoeff)
    PE_MEMBER_SET_GET(PEReal, restitution_coeff, RestitutionCoeff)
    PE_MEMBER_GET(PEReal, linear_damping, LinearDamping)
    PE_MEMBER_GET(PEReal, angular_damping, AngularDamping)

    PE_MEMBER_SET_GET(pe_common::Transform, transform, Transform)
    PE_MEMBER_SET_GET(pe_common::Vector3, velocity, Velocity)
    PE_MEMBER_SET_GET(pe_common::Vector3, angular_velocity, AngularVelocity)

public:
    CollisionBody():
        _global_id(++_globalIdCounter),
        _kinematic(false),
        _active(true),
        _mass(1.),
        _inv_mass(1.),
        _shape(nullptr),
        _transform(pe_common::Transform::identity()),
        _velocity(pe_common::Vector3::zeros()),
        _friction_coeff(0.5),
        _restitution_coeff(0.5),
        _linear_damping(0.),
        _angular_damping(0.) {}
    ~CollisionBody() { setShape(0); }

    virtual bool isDeformable() const = 0;
    void applyForce(const pe_common::Vector3& force, const pe_common::Vector3& point, PEReal dt);
    void updateTransform(PEReal dt);
};

} // namespace pe_phys_object
