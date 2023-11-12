#pragma once

#include "phys/shape/shape.h"

namespace pe_phys_object {

class CollisionBody {
    PE_MEMBER_GET(uint32_t, globalId, GlobalId)
    PE_MEMBER_SET_GET(bool, isKinematic, IsKinematic)
    PE_MEMBER_SET_GET(real, mass, Mass)
    PE_MEMBER_PTR_SET_GET(pe_phys_shape::Shape, shape, Shape)

    PE_MEMBER_SET_GET(real, frictionCoeff, FrictionCoeff)
    PE_MEMBER_SET_GET(real, restitutionCoeff, RestitutionCoeff)
    PE_MEMBER_GET(real, linearDamping, LinearDamping)
    PE_MEMBER_GET(real, angularDamping, AngularDamping)

    PE_MEMBER_SET_GET(pe_cg::Vector3, force, Force)
    PE_MEMBER_SET_GET(pe_cg::Vector3, torque, Torque)
    PE_MEMBER_SET_GET(pe_cg::Transform, transform, Transform)
    PE_MEMBER_SET_GET(pe_cg::Vector3, velocity, Velocity)
    PE_MEMBER_SET_GET(pe_cg::Vector3, angularVelocity, AngularVelocity)

public:
    CollisionBody():
    _globalId(0),
    _isKinematic(false),
    _mass(1.),
    _shape(nullptr),
    _transform(pe_cg::Transform::identity()),
    _velocity(pe_cg::Vector3::zeros()),
    _frictionCoeff(0.5),
    _restitutionCoeff(0.5),
    _linearDamping(0.),
    _angularDamping(0.) {}
    ~CollisionBody() { setShape(0); }

    virtual bool isDeformable() const = 0;
};

} // namespace pe_phys_object
