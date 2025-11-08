#include "track.h"
#include "vehicle/component/chassis.h"
#include "physics/shape/box_shape.h"
#include "physics/constraint/constraint/ball_joint_constraint.h"
#include "physics/constraint/constraint/six_dof_constraint.h"
#include "physics/constraint/constraint/hinge_joint_constraint.h"

namespace pe_vehicle {

#define PE_TRACK_USE_HINGE_JOINT_CONSTRAINT

static pe::Transform getTrackSegmentTransform(pe::Real length, pe::Real radius, pe::Real segment_count, pe::Real current_length) {
    if (current_length < PE_PI * radius) {
        const pe::Real angle = current_length / radius;
        const pe::Real x = -radius * PE_SIN(angle) - length / PE_R(2.0);
        const pe::Real y = radius * PE_COS(angle);
        return pe::Transform(pe::Matrix3::fromRotation(-pe::Vector3::right(), angle), pe::Vector3(0, y, x));
    }

    current_length -= PE_PI * radius;
    if (current_length < length) {
        const pe::Real x = -length / PE_R(2.0) + current_length;
        return pe::Transform(pe::Matrix3::fromRotation(-pe::Vector3::right(), PE_PI), pe::Vector3(0, -radius, x));
    }

    current_length -= length;
    if (current_length < PE_PI * radius) {
        const pe::Real angle = current_length / radius;
        const pe::Real x = radius * PE_SIN(angle) + length / PE_R(2.0);
        const pe::Real y = -radius * PE_COS(angle);
        return pe::Transform(pe::Matrix3::fromRotation(-pe::Vector3::right(), PE_PI + angle), pe::Vector3(0, y, x));
    }

    current_length -= PE_PI * radius;
    const pe::Real x = length / PE_R(2.0) - current_length;
    return pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, radius, x));
}

Track::Track(Chassis* chassis, const pe::Vector3& anchor, pe::Real length, pe::Real radius, pe::Real width,
             int segment_count, pe::Real segment_height, pe::Real segment_length_ratio,
             pe::Real segment_mass, pe::Real segment_friction, pe::Real tightening_ratio) :
    _length(length), _radius(radius), _width(width),
    _segment_count(segment_count), _setment_height(segment_height),
    _setment_length_ratio(segment_length_ratio), _segment_mass(segment_mass),
    _segment_friction(segment_friction), _tightening_ratio(tightening_ratio) {

    const pe::Transform trans_chassis = chassis->getTransform();
    const pe::Transform trans_track = trans_chassis * pe::Transform(pe::Matrix3::identity(), anchor);
    _anchor_chassis = chassis->getBasePart().local_transform.inverseTransform(anchor);

    const pe::Real track_length = PE_R(2.0) * PE_PI * radius + PE_R(2.0) * length;
    const pe::Vector3 segment_size = pe::Vector3(width, segment_height, track_length * segment_length_ratio / PE_R(segment_count));
    const pe::Real segment_gap = track_length / PE_R(segment_count);

    for (int i = 0; i < segment_count; ++i) {
        const pe::Real current_length = track_length * PE_R(i) / PE_R(segment_count);
        const pe::Transform trans_seg = getTrackSegmentTransform(length, radius, segment_count, current_length);
        auto segment = new pe_physics_object::RigidBody();
        segment->setMass(segment_mass);
        auto shape = new pe_physics_shape::BoxShape(segment_size);
        segment->setCollisionShape(shape);
        segment->setTransform(trans_track * trans_seg);
        segment->setFrictionCoeff(segment_friction);
        segment->setAngularDamping(PE_R(0.8));
        segment->addIgnoreCollisionId(chassis->getBasePart().body->getGlobalId());
        _segments.push_back(segment);
    }

    const pe::Real half_segment_dist = segment_gap * tightening_ratio / PE_R(2.0);
    const pe::Real half_segment_width = width / PE_R(2.0);
    for (int i = 0; i < segment_count; ++i) {
#   ifdef PE_TRACK_USE_HINGE_JOINT_CONSTRAINT
        auto joint = new pe_physics_constraint::HingeJointConstraint();
        joint->setObjectA(_segments[segment_count - 1 - i]);
        joint->setObjectB(_segments[(segment_count - i) % segment_count]);
        joint->setAnchorA(pe::Vector3(0, 0, -half_segment_dist));
        joint->setAnchorB(pe::Vector3(0, 0, half_segment_dist));
        joint->setAxisA(pe::Vector3::right());
        joint->setAxisB(pe::Vector3::right());
        _joints.push_back(joint);
#   else
        auto joint1 = new pe_physics_constraint::BallJointConstraint();
        joint1->setObjectA(_segments[i]);
        joint1->setObjectB(_segments[(i + 1) % segment_count]);
        joint1->setAnchorA(pe::Vector3(half_segment_width, 0, -half_segment_dist));
        joint1->setAnchorB(pe::Vector3(half_segment_width, 0, half_segment_dist));
        _joints.push_back(joint1);
        auto joint2 = new pe_physics_constraint::BallJointConstraint();
        joint2->setObjectA(_segments[i]);
        joint2->setObjectB(_segments[(i + 1) % segment_count]);
        joint2->setAnchorA(pe::Vector3(-half_segment_width, 0, -half_segment_dist));
        joint2->setAnchorB(pe::Vector3(-half_segment_width, 0, half_segment_dist));
        _joints.push_back(joint2);
#   endif
    }

    const pe::Real dist_to_chassis_x = anchor.x;
    for (int i = 0; i < segment_count; i += segment_count / 6) {
        auto joint = new pe_physics_constraint::SixDofConstraint();
        joint->setObjectA(chassis->getBasePart().body);
        joint->setObjectB(_segments[i]);
        joint->setFrameA(chassis->getBasePart().local_transform.inverse() * pe::Transform(pe::Matrix3::identity(), pe::Vector3(dist_to_chassis_x, 0, 0)));
        joint->setFrameB(pe::Transform::identity());
        joint->setXPosFixed(true);
        joint->setYRotFixed(true);
        joint->setZRotFixed(true);
        _joints.push_back(joint);
    }
}

Track::~Track() {
    for (auto joint : _joints) {
        delete joint;
    }
    for (auto segment : _segments) {
        delete segment->getCollisionShape();
        delete segment;
    }
}

void Track::init(pe_interface::World* phys_world) {
    for (auto segment : _segments) {
        phys_world->addRigidBody(segment);
    }
    
    // add joints in two passes to avoid instability (tricky but significant)
    for (int i = 0; i < PE_I(_joints.size()); i += 2) {
        phys_world->addConstraint(_joints[i]);
    }
    for (int i = 1; i < PE_I(_joints.size()); i += 2) {
        phys_world->addConstraint(_joints[i]);
    }
}

void Track::step(pe::Real dt) {
    (void)dt;
    // nothing to do for now
}

} // namespace pe_vehicle
