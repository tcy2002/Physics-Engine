#pragma once

#include "physics/physics.h"
#include "interface/world.h"
#include "physics/object/rigidbody.h"

namespace pe_vehicle {

struct ChassisPart {
    pe_physics_object::RigidBody* body = nullptr;
    pe::Transform local_transform = pe::Transform::identity();
};

/*
 * A chassis is composed of several rigid bodies (parts) linked together.
 * The first part (index 0) is the base part, and other parts (index 1~) are additional parts.
 * You can freely add constraints (links) between the parts to form a complex chassis structure.
 *
 * Note: we do not offer removal for parts and links, so you can't modify the links during runtime,
 * and you must call init() before you can see the chassis acts correctly in the physics world.
 *
 * There is one link type that can be controlled during runtime: HingeJointConstraint. For example,
 * the tank turret can rotate around the base, and the tank barrel can elevate up and down.
 */
class Chassis {
protected:
    ChassisPart _base_part;
    pe::Array<ChassisPart> _other_parts;
    pe::Array<pe_physics_constraint::Constraint*> _links;

    ChassisPart createBoxPart(const pe::Transform& part_trans, const pe::Vector3& size, pe::Real mass) const;
    ChassisPart createSpherePart(const pe::Transform& part_trans, pe::Real radius, pe::Real mass) const;
    ChassisPart createCapsulePart(const pe::Transform& part_trans, pe::Real radius, pe::Real height, pe::Real mass) const;

public:
    Chassis() = default;
    virtual ~Chassis();

    virtual void init(pe_interface::World* phys_world);
    virtual void step(pe::Real dt);

    virtual void setTransform(const pe::Transform& trans);
    virtual pe::Transform getTransform() const;

    /* The index of base part is 0 */
    int setBoxBase(const pe::Transform& part_trans, const pe::Vector3& size, pe::Real mass);
    /* The index of base part is 0 */
    int setSphereBase(const pe::Transform& part_trans, pe::Real radius, pe::Real mass);
    /* The index of base part is 0 */
    int setCapsuleBase(const pe::Transform& part_trans, pe::Real radius, pe::Real height, pe::Real mass);

    const ChassisPart& getBasePart() const { return _base_part; }

    /* Return the index of the added part */
    int addBoxPart(const pe::Transform& part_trans, const pe::Vector3& size, pe::Real mass);
    /* Return the index of the added part */
    int addSpherePart(const pe::Transform& part_trans, pe::Real radius, pe::Real mass);
    /* Return the index of the added part */
    int addCapsulePart(const pe::Transform& part_trans, pe::Real radius, pe::Real height, pe::Real mass);

    const ChassisPart& getPart(int index) const;

    int addBallLink(int part1_index, int part2_index, const pe::Vector3& anchor1, const pe::Vector3& anchor2);
    int addHingeLink(int part1_index, int part2_index,
                     const pe::Vector3& anchor1, const pe::Vector3& axis1,
                     const pe::Vector3& anchor2, const pe::Vector3& axis2,
                     pe_physics_constraint::ConstraintLimitType type = pe_physics_constraint::ConstraintLimitType::CLT_NONE,
                     pe::Real min_angle = 0, pe::Real max_angle = 0);
    int addSliderLink(int part1_index, int part2_index,
                      const pe::Vector3& anchor1, const pe::Vector3& axis1,
                      const pe::Vector3& anchor2, const pe::Vector3& axis2);
    int addSixDofLink(int part1_index, int part2_index,
                      const pe::Transform& frame1, const pe::Transform& frame2,
                      bool x_pos_fixed, bool y_pos_fixed, bool z_pos_fixed, 
                      bool x_rot_fixed, bool y_rot_fixed, bool z_rot_fixed);

    void controlHingeLink(int link_index, pe_physics_constraint::ConstraintMotorType type,
                          pe::Real target_speed_or_angle) const;
    void controlSliderLink(int link_index, pe_physics_constraint::ConstraintMotorType type,
                           pe::Real target_speed_or_position) const;
};

} // namespace pe_vehicle
