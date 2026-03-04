#include "suspension.h"
#include "chassis.h"

namespace pe_vehicle {

Suspension::Suspension(Chassis *chassis, Wheel *wheel, AxleType axle_type,
                       pe::Real rest_length, pe::Real stiffness, pe::Real damping, const pe::Vector3 &anchor_chassis) {
    if (chassis == nullptr || wheel == nullptr) {
        throw std::runtime_error("Suspension constructor error: chassis or wheel object is null.");
    }

    _rest_length = rest_length;
    _stiffness = stiffness;
    _damping = damping;
    _chassis = chassis;
    _wheel = wheel;
    _anchor_chassis = chassis->getBasePart().local_transform.inverseTransform(anchor_chassis);
    _axis_chassis = chassis->getBasePart().local_transform.getBasis().transposed() * -pe::Vector3::up();

    // create six dof constraint
    _constraint = new pe_physics_constraint::SixDofConstraint();
    _constraint->setObjectA(chassis->getBasePart().body);
    _constraint->setObjectB(wheel->getBody());
    _constraint->setXPosFixed(true);
    _constraint->setYPosFixed(false);
    _constraint->setZPosFixed(true);
    _constraint->setFrameA(chassis->getBasePart().local_transform.inverse() * pe::Transform(pe::Matrix3::identity(), anchor_chassis));
    if (axle_type == AxleType::AT_FRONT) { // Hinge2
        _constraint->setXRotFixed(true);
        _constraint->setFrameB(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::right(), -PE_PI / 2), pe::Vector3::zeros()));
        // set Y axis limits for steering
        _constraint->setYRotLimitType(pe_physics_constraint::ConstraintLimitType::CLT_LOWER_UPPER);
        _constraint->setMinAngleY(-PE_PI * PE_R(3.0 / 4));
        _constraint->setMaxAngleY(-PE_PI * PE_R(1.0 / 4));
    } else if (axle_type == AxleType::AT_REAR) { // Hinge
        _constraint->setYRotFixed(true);
        _constraint->setZRotFixed(true);
        _constraint->setFrameB(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), -PE_PI / 2), pe::Vector3::zeros()));
    } else {
        throw std::runtime_error("Suspension constructor error: axle type not set.");
    }
}

Suspension::~Suspension() {
    delete _constraint;
}

void Suspension::setSteerAngle(pe::Real angle) {
    _constraint->setYRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_POSITION);
    _constraint->setTargetAngleY(angle);
}

void Suspension::releaseSteerAngle() {
    _constraint->setYRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_NONE);
}

void Suspension::init(pe_interface::World* phys_world) {
    phys_world->addConstraint(_constraint);
}

void Suspension::setTargetWheelSpeed(pe::Real speed) {
    if (_constraint->isXRotFixed()) {
        _constraint->setZRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
        _constraint->setTargetSpeedZ(speed);
    } else {
        _constraint->setXRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
        _constraint->setTargetSpeedX(speed);
    }
}

void Suspension::releaseTargetWheelSpeed() {
    if (_constraint->isXRotFixed()) {
        _constraint->setZRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_NONE);
    }
    else {
        _constraint->setXRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_NONE);
    }
}

void Suspension::step(pe::Real dt) {
    const pe::Vector3& wheel_pos = _constraint->getObjectB()->getTransform().getOrigin();
    const pe::Transform& trans_chassis = _constraint->getObjectA()->getTransform();
    const pe::Vector3& axis_chassis = trans_chassis.getBasis() * _axis_chassis;
    const pe::Vector3 anchor_pos = trans_chassis * _anchor_chassis;
    const pe::Vector3 anchor_pos_world_rel = anchor_pos - trans_chassis.getOrigin();

    const pe::Real current_length = (wheel_pos - anchor_pos).dot(axis_chassis);
    const pe::Real length_error = current_length - _rest_length;
    const pe::Vector3 correction_impulse = -_stiffness * length_error * axis_chassis;

    const pe::Vector3 rel_vel = _constraint->getObjectB()->getLinearVelocity() -
                                _constraint->getObjectA()->getLinearVelocityAtLocalPoint(anchor_pos_world_rel);
    const pe::Vector3 damping_impulse = -_damping * rel_vel.dot(axis_chassis) * axis_chassis;

    const pe::Vector3 total_impulse = correction_impulse + damping_impulse;
    _constraint->getObjectB()->applyImpulse(pe::Vector3::zeros(), total_impulse * dt);
    _constraint->getObjectA()->applyImpulse(anchor_pos_world_rel, -total_impulse * dt);
}

} // namespace pe_vehicle