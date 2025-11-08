#include "chassis.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/capsule_shape.h"
#include "physics/constraint/constraint/ball_joint_constraint.h"
#include "physics/constraint/constraint/hinge_joint_constraint.h"
#include "physics/constraint/constraint/slider_joint_constraint.h"
#include "physics/constraint/constraint/six_dof_constraint.h"
#include "utils/logger.h"

namespace pe_vehicle {

ChassisPart Chassis::createBoxPart(const pe::Transform& part_trans, const pe::Vector3& size, pe::Real mass) const {
    auto shape = new pe_physics_shape::BoxShape(size);
    auto body = new pe_physics_object::RigidBody();
    body->setMass(mass);
    body->setCollisionShape(shape);
    body->setTransform(getTransform() * part_trans);
    return ChassisPart{body, part_trans};
}

ChassisPart Chassis::createSpherePart(const pe::Transform& part_trans, pe::Real radius, pe::Real mass) const {
    auto shape = new pe_physics_shape::SphereShape(radius);
    auto body = new pe_physics_object::RigidBody();
    body->setMass(mass);
    body->setCollisionShape(shape);
    body->setTransform(getTransform() * part_trans);
    return ChassisPart{body, part_trans};
}

ChassisPart Chassis::createCapsulePart(const pe::Transform& part_trans, pe::Real radius, pe::Real height, pe::Real mass) const {
    auto shape = new pe_physics_shape::CapsuleShape(radius, height);
    auto body = new pe_physics_object::RigidBody();
    body->setMass(mass);
    body->setCollisionShape(shape);
    body->setTransform(getTransform() * part_trans);
    return ChassisPart{body, part_trans};
}

Chassis::~Chassis() {
    delete _base_part.body->getCollisionShape();
    delete _base_part.body;
    for (const auto part : _other_parts) {
        delete part.body->getCollisionShape();
        delete part.body;
    }
    for (const auto link : _links) {
        delete link;
    }
    _other_parts.clear();
    _links.clear();
}

void Chassis::init(pe_interface::World* phys_world) {
    phys_world->addRigidBody(_base_part.body);
    for (auto& part : _other_parts) {
        phys_world->addRigidBody(part.body);
    }
    for (auto& link : _links) {
        phys_world->addConstraint(link);
    }
}

void Chassis::step(pe::Real dt) {
    (void)dt;
}

void Chassis::setTransform(const pe::Transform& trans) {
    if (_base_part.body == nullptr) return;
    const pe::Transform base_part_new_trans = trans * _base_part.local_transform;
    const pe::Transform delta_trans = _base_part.body->getTransform().inverse() * base_part_new_trans;
    _base_part.body->setTransform(base_part_new_trans);
    for (auto& part : _other_parts) {
        const pe::Transform part_new_trans = part.body->getTransform() * delta_trans;
        part.body->setTransform(part_new_trans);
    }
}

pe::Transform Chassis::getTransform() const {
    if (_base_part.body == nullptr) return pe::Transform::identity();
    return _base_part.body->getTransform() * _base_part.local_transform.inverse();
}

int Chassis::setBoxBase(const pe::Transform& part_trans, const pe::Vector3& size, pe::Real mass) {
    _base_part = createBoxPart(part_trans, size, mass);
    return 0;
}

int Chassis::setSphereBase(const pe::Transform& part_trans, pe::Real radius, pe::Real mass) {
    _base_part = createSpherePart(part_trans, radius, mass);
    return 0;
}

int Chassis::setCapsuleBase(const pe::Transform& part_trans, pe::Real radius, pe::Real height, pe::Real mass) {
    _base_part = createCapsulePart(part_trans, radius, height, mass);
    return 0;
}

int Chassis::addBoxPart(const pe::Transform& part_trans, const pe::Vector3& size, pe::Real mass) {
    _other_parts.push_back(createBoxPart(part_trans, size, mass));
    return PE_I(_other_parts.size());
}

int Chassis::addSpherePart(const pe::Transform& part_trans, pe::Real radius, pe::Real mass) {
    _other_parts.push_back(createSpherePart(part_trans, radius, mass));
    return PE_I(_other_parts.size());
}

int Chassis::addCapsulePart(const pe::Transform& part_trans, pe::Real radius, pe::Real height, pe::Real mass) {
    _other_parts.push_back(createCapsulePart(part_trans, radius, height, mass));
    return PE_I(_other_parts.size());
}

const ChassisPart& Chassis::getPart(int index) const {
    if (index == 0) {
        return _base_part;
    } else if (index > 0 && index <= PE_I(_other_parts.size())) {
        return _other_parts[index - 1];
    } else {
        throw std::out_of_range("Invalid part index in Chassis::getPart: " + std::to_string(index));
    }
}

int Chassis::addBallLink(int part1_index, int part2_index, const pe::Vector3 &anchor1, const pe::Vector3 &anchor2) {
    if (part1_index < 0 || part1_index >= PE_I(_other_parts.size() + 1) ||
        part2_index < 0 || part2_index >= PE_I(_other_parts.size() + 1)) {
        PE_LOG_ERROR << "Invalid part index for adding ball link: " << part1_index << ", " << part2_index << PE_CUSTOM_ENDL;
        return -1;
    }

    auto link = new pe_physics_constraint::BallJointConstraint();
    pe_physics_object::RigidBody* body1 = part1_index == 0 ? _base_part.body : _other_parts[part1_index - 1].body;
    pe_physics_object::RigidBody* body2 = part2_index == 0 ? _base_part.body : _other_parts[part2_index - 1].body;
    link->setObjectA(body1);
    link->setObjectB(body2);
    link->setAnchorA(anchor1);
    link->setAnchorB(anchor2);
    _links.push_back(link);

    return PE_I(_links.size() - 1);
}

int Chassis::addHingeLink(int part1_index, int part2_index,
                          const pe::Vector3 &anchor1, const pe::Vector3 &axis1,
                          const pe::Vector3 &anchor2, const pe::Vector3 &axis2,
                          pe_physics_constraint::ConstraintLimitType type, pe::Real min_angle, pe::Real max_angle) {
    if (part1_index < 0 || part1_index >= PE_I(_other_parts.size() + 1) ||
        part2_index < 0 || part2_index >= PE_I(_other_parts.size() + 1)) {
        PE_LOG_ERROR << "Invalid part index for adding ball link: " << part1_index << ", " << part2_index << PE_CUSTOM_ENDL;
        return -1;
    }

    auto link = new pe_physics_constraint::HingeJointConstraint();
    pe_physics_object::RigidBody* body1 = part1_index == 0 ? _base_part.body : _other_parts[part1_index - 1].body;
    pe_physics_object::RigidBody* body2 = part2_index == 0 ? _base_part.body : _other_parts[part2_index - 1].body;
    link->setObjectA(body1);
    link->setObjectB(body2);
    link->setAnchorA(anchor1);
    link->setAxisA(axis1);
    link->setAnchorB(anchor2);
    link->setAxisB(axis2);
    link->setLimitType(type);
    link->setMinAngle(min_angle);
    link->setMaxAngle(max_angle);
    _links.push_back(link);

    return PE_I(_links.size() - 1);
}

int Chassis::addSliderLink(int part1_index, int part2_index,
                            const pe::Vector3 &anchor1, const pe::Vector3 &axis1,
                            const pe::Vector3 &anchor2, const pe::Vector3 &axis2) {
    if (part1_index < 0 || part1_index >= PE_I(_other_parts.size() + 1) ||
        part2_index < 0 || part2_index >= PE_I(_other_parts.size() + 1)) {
        PE_LOG_ERROR << "Invalid part index for adding ball link: " << part1_index << ", " << part2_index << PE_CUSTOM_ENDL;
        return -1;
    }

    auto link = new pe_physics_constraint::SliderJointConstraint();
    pe_physics_object::RigidBody* body1 = part1_index == 0 ? _base_part.body : _other_parts[part1_index - 1].body;
    pe_physics_object::RigidBody* body2 = part2_index == 0 ? _base_part.body : _other_parts[part2_index - 1].body;
    link->setObjectA(body1);
    link->setObjectB(body2);
    link->setAnchorA(anchor1);
    link->setAxisA(axis1);
    link->setAnchorB(anchor2);
    link->setAxisB(axis2);
    _links.push_back(link);

    return PE_I(_links.size() - 1);
}

int Chassis::addSixDofLink(int part1_index, int part2_index,
                           const pe::Transform &frame1, const pe::Transform &frame2,
                           bool x_pos_fixed, bool y_pos_fixed, bool z_pos_fixed,
                           bool x_rot_fixed, bool y_rot_fixed, bool z_rot_fixed) {
    if (part1_index < 0 || part1_index >= PE_I(_other_parts.size() + 1) ||
        part2_index < 0 || part2_index >= PE_I(_other_parts.size() + 1)) {
        PE_LOG_ERROR << "Invalid part index for adding ball link: " << part1_index << ", " << part2_index << PE_CUSTOM_ENDL;
        return -1;
    }

    auto link = new pe_physics_constraint::SixDofConstraint();
    pe_physics_object::RigidBody* body1 = part1_index == 0 ? _base_part.body : _other_parts[part1_index - 1].body;
    pe_physics_object::RigidBody* body2 = part2_index == 0 ? _base_part.body : _other_parts[part2_index - 1].body;
    link->setObjectA(body1);
    link->setObjectB(body2);
    link->setFrameA(frame1);
    link->setFrameB(frame2);
    link->setXPosFixed(x_pos_fixed);
    link->setYPosFixed(y_pos_fixed);
    link->setZPosFixed(z_pos_fixed);
    link->setXRotFixed(x_rot_fixed);
    link->setYRotFixed(y_rot_fixed);
    link->setZRotFixed(z_rot_fixed);
    _links.push_back(link);

    return PE_I(_links.size() - 1);
}

void Chassis::controlHingeLink(int link_index, pe_physics_constraint::ConstraintMotorType type,
                               pe::Real target_speed_or_angle) const {
    if (link_index < 0 || link_index >= PE_I(_links.size())) {
        PE_LOG_ERROR << "Invalid link index for controlling hinge link: " << link_index << PE_CUSTOM_ENDL;
        return;
    }

    if (_links[link_index]->getType() != pe_physics_constraint::ConstraintType::CT_HINGE_JOINT) {
        PE_LOG_ERROR << "The specified link is not a hinge link: " << link_index << PE_CUSTOM_ENDL;
        return;
    }

    auto hinge_link = static_cast<pe_physics_constraint::HingeJointConstraint*>(_links[link_index]);

    hinge_link->setMotorType(type);
    if (type == pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY) {
        hinge_link->setTargetSpeed(target_speed_or_angle);
    } else if (type == pe_physics_constraint::ConstraintMotorType::CMT_POSITION) {
        hinge_link->setTargetAngle(target_speed_or_angle);
    }
}

void Chassis::controlSliderLink(int link_index, pe_physics_constraint::ConstraintMotorType type,
                                pe::Real target_speed_or_angle) const {
    if (link_index < 0 || link_index >= PE_I(_links.size())) {
        PE_LOG_ERROR << "Invalid link index for controlling slider link: " << link_index << PE_CUSTOM_ENDL;
        return;
    }

    if (_links[link_index]->getType() != pe_physics_constraint::ConstraintType::CT_SLIDER_JOINT) {
        PE_LOG_ERROR << "The specified link is not a slider link: " << link_index << PE_CUSTOM_ENDL;
        return;
    }

    auto slider_link = static_cast<pe_physics_constraint::SliderJointConstraint*>(_links[link_index]);

    slider_link->setMotorType(type);
    if (type == pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY) {
        slider_link->setTargetSpeed(target_speed_or_angle);
    } else if (type == pe_physics_constraint::ConstraintMotorType::CMT_POSITION) {
        slider_link->setTargetPosition(target_speed_or_angle);
    }
}

} // namespace pe_vehicle
