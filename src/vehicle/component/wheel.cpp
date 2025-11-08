#include "wheel.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/capsule_shape.h"
#include "utils/logger.h"

namespace pe_vehicle {

Wheel::Wheel(WheelType type, pe::Real radius, pe::Real width, pe::Real mass, pe::Real friction):
    _radius(radius), _width(width), _mass(mass), _friction(friction) {
    _wheel = new pe_physics_object::RigidBody();
    _wheel->setMass(mass);
    _wheel->setFrictionCoeff(friction);
    _wheel->setAngularDamping(PE_R(0.8));
    _delta_transform = pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / PE_R(2.0)), pe::Vector3::zeros());
    _wheel->setTransform(_delta_transform);

    pe_physics_shape::Shape* shape = nullptr;
    switch (type) {
        case WheelType::WT_Sphere:
            shape = new pe_physics_shape::SphereShape(radius);
            break;
        case WheelType::WT_Capsule:
            shape = new pe_physics_shape::CapsuleShape(radius, PE_MAX(width - PE_R(2.0) * radius, PE_R(0.0)));
            break;
        case WheelType::WT_Cylinder:
            // not implemented yet
            throw std::runtime_error("Cylinder wheel shape is not implemented yet.");
    }
    _wheel->setCollisionShape(shape);
}

Wheel::~Wheel() {
    delete _wheel->getCollisionShape();
    delete _wheel;
}

void Wheel::applyTorque(pe::Real torque) {
    _wheel->addTorque(_wheel->getTransform().getBasis().getColumn(1) * torque);
}

void Wheel::init(pe_interface::World* phys_world) {
    phys_world->addRigidBody(_wheel);
}

void Wheel::step(pe::Real dt) {
    (void)dt;
    // nothing to do for now
    /*if (_wheel->getGlobalId() == 9) {
        std::cout << "wheel ang vel: " << _wheel->getAngularVelocity() << std::endl;
    }*/
}

void Wheel::setTransform(const pe::Transform& trans) {
    _wheel->setTransform(trans * _delta_transform);
}

pe::Transform Wheel::getTransform() const {
    return _wheel->getTransform() * _delta_transform.inverse();
}

} // namespace pe_vehicle