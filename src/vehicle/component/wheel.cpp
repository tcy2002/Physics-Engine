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
            PE_LOG_ERROR << "Cylinder wheel shape is not implemented yet." << PE_ENDL;
            break;
    }
    _wheel->setCollisionShape(shape);
}

} // namespace pe_vehicle