#include "suspension.h"
#include "chassis.h"

namespace pe_vehicle {

Suspension::Suspension(Chassis *chassis, Wheel *wheel, pe::Real rest_length, pe::Real stiffness,
                       pe::Real damping, const pe::Vector3 &anchor_chassis) {
    _rest_length = rest_length;
    _stiffness = stiffness;
    _damping = damping;
    _anchor_chassis = anchor_chassis;

    // create six dof constraint
    _constraint = new pe_physics_constraint::SixDofConstraint();
    _constraint->setObjectA(chassis->getBasePart().body);
    _constraint->setObjectB(wheel->getBody());
    pe::Transform frame_chassis = pe::Transform::identity();
    frame_chassis.setOrigin(anchor_chassis);
    pe::Transform frame_wheel = pe::Transform::identity();
    frame_wheel.setOrigin(pe::Vector3::zeros());
    _constraint->setFrameA(frame_wheel);
    _constraint->setFrameB(frame_chassis);
    _constraint->setXPosFixed(true);
    _constraint->setYPosFixed(false);
    _constraint->setZPosFixed(true);
    _constraint->setXRotFixed(false);
    _constraint->setYRotFixed(false);
    _constraint->setZRotFixed(true); // means the left direction of the wheel must be perpendicular to the chassis up direction
}

Suspension::~Suspension() {
    delete _constraint;
}

void Suspension::init(pe_interface::World* phys_world) {
    phys_world->addConstraint(_constraint);
}

void Suspension::step(pe::Real dt) {
    (void)dt;
    // nothing to do for now
}

} // namespace pe_vehicle