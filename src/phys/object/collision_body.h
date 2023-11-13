#pragma once

#include "phys/shape/shape.h"

namespace pe_phys_object {

class CollisionBody {
    static uint32_t _globalIdCounter;

    PE_MEMBER_GET(uint32_t, global_id, GlobalId)
    PE_BOOL_SET_GET(kinematic, Kinematic)
    PE_BOOL_SET_GET(active, Active)
    PE_MEMBER_PTR_SET_GET(pe_phys_shape::Shape, shape, Shape)

    PE_MEMBER_GET(real, mass, Mass)
    PE_MEMBER_GET(real, inv_mass, InvMass)
public:
    void setMass(real mass) { _mass = mass; _inv_mass = 1. / mass; }

    PE_MEMBER_SET_GET(real, friction_coeff, FrictionCoeff)
    PE_MEMBER_SET_GET(real, restitution_coeff, RestitutionCoeff)
    PE_MEMBER_GET(real, linear_damping, LinearDamping)
    PE_MEMBER_GET(real, angular_damping, AngularDamping)

    PE_MEMBER_SET_GET(pe_cg::Transform, transform, Transform)
    PE_MEMBER_SET_GET(pe_cg::Vector3, velocity, Velocity)
    PE_MEMBER_SET_GET(pe_cg::Vector3, angular_velocity, AngularVelocity)

public:
    CollisionBody():
        _global_id(++_globalIdCounter),
        _kinematic(false),
        _active(true),
        _mass(1.),
        _inv_mass(1.),
        _shape(nullptr),
        _transform(pe_cg::Transform::identity()),
        _velocity(pe_cg::Vector3::zeros()),
        _friction_coeff(0.5),
        _restitution_coeff(0.5),
        _linear_damping(0.),
        _angular_damping(0.) {}
    ~CollisionBody() { setShape(0); }

    virtual bool isDeformable() const = 0;
    void applyForce(const pe_cg::Vector3& force, const pe_cg::Vector3& point, real dt);
    void updateTransform(real dt);
};

} // namespace pe_phys_object
