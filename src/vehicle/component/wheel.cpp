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

    pe_physics_shape::Shape* shape = nullptr;
    switch (type) {
        case WheelType::WT_Sphere:
            shape = new pe_physics_shape::SphereShape(radius);
            break;
        case WheelType::WT_Capsule:
            shape = new pe_physics_shape::CapsuleShape(radius, width);
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

void Wheel::init(pe_interface::World* phys_world) {
    phys_world->addRigidBody(_wheel);
}

void Wheel::step(pe::Real dt) {
    (void)dt;
    // nothing to do for now
}

} // namespace pe_vehicle