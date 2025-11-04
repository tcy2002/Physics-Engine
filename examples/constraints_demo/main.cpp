#include "interface/simulator.h"
#include "physics/constraint/constraint/ball_joint_constraint.h"
#include "physics/constraint/constraint/hinge_joint_constraint.h"
#include "physics/constraint/constraint/slider_joint_constraint.h"
#include "physics/constraint/constraint/six_dof_constraint.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/capsule_shape.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class ConstraintsSimulator : public pe_interface::Simulator {
public:
    ConstraintsSimulator() {}
    virtual ~ConstraintsSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        // _world.setSleepLinVel2Threshold(PE_R(0.01)); // linear velocity threshold for sleep
        // _world.setSleepAngVel2Threshold(PE_R(0.005)); // angular velocity threshold for sleep
        // _world.setSleepTimeThreshold(PE_R(2.0));     // sleep time threshold

        // add a ground
        auto ground = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(100, 10, 100), 1000);
        ground->setKinematic(true);
        _world.addRigidBody(ground); // a rigidbody must be added into the _world to perform physical effects

        // add a ceiling
        auto ceiling = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 55, 0)),
                                      pe::Vector3(100, 10, 100), 1000);
        ceiling->setKinematic(true);
        _world.addRigidBody(ceiling);

        // add a wall
        auto wall1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-55, 25, 0)),
                                      pe::Vector3(10, 60, 100), 1000);
        wall1->setKinematic(true);
        _world.addRigidBody(wall1);

        // add a wall
        auto wall2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(55, 25, 0)),
                                      pe::Vector3(10, 60, 100), 1000);
        wall2->setKinematic(true);
        _world.addRigidBody(wall2);

        // add a wall
        auto wall3 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, -55)),
                                      pe::Vector3(100, 60, 10), 1000);
        wall3->setKinematic(true);
        _world.addRigidBody(wall3);

        // add a wall
        auto wall4 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, 55)),
                                      pe::Vector3(100, 60, 10), 1000);
        wall4->setKinematic(true);
        _world.addRigidBody(wall4);

        /***************** ball joint ****************/
        // add a ball base
        auto rb2 = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0.5, 0)),
                                         PE_R(0.5), 1);
        rb2->setKinematic(true);
        _world.addRigidBody(rb2);

        // add a stick
        pe::Transform trans;
        trans.setRotation(pe::Vector3::forward(), PE_PI / 6);
        trans.setOrigin(pe::Vector3(-PE_R(3.3) * PE_SIN(PE_PI / 6), PE_R(3.3) * PE_COS(PE_PI / 6) + PE_R(0.5), 0));
        auto rb3 = createCapsuleRigidBody(trans, PE_R(0.2), 6, 1);
        rb2->addIgnoreCollisionId(rb3->getGlobalId());
        _world.addRigidBody(rb3);

        // add a ball joint constraint
        auto c1 = new pe_physics_constraint::BallJointConstraint();
        c1->setObjectA(rb2);
        c1->setObjectB(rb3);
        c1->setAnchorA(pe::Vector3(0, 0, 0));
        c1->setAnchorB(pe::Vector3(0, PE_R(-3.3), 0));
        _world.addConstraint(c1);

        // add the second sphere base
        trans.setBasis(pe::Matrix3::identity());
        trans.setOrigin(pe::Vector3(PE_R(-6.6) * PE_SIN(PE_PI / 6), PE_R(6.6) * PE_COS(PE_PI / 6) + PE_R(0.5), 0));
        rb4 = createSphereRigidBody(trans, PE_R(0.4), 1);
        rb4->addIgnoreCollisionId(rb3->getGlobalId());
        _world.addRigidBody(rb4);

        // add the second stick
        trans.setRotation(pe::Vector3::forward(), PE_PI / 2);
        trans.setOrigin(pe::Vector3(PE_R(-3.3) - PE_R(6.6) * PE_SIN(PE_PI / 6), PE_R(6.6) * PE_COS(PE_PI / 6) + PE_R(0.5), 0));
        auto rb5 = createCapsuleRigidBody(trans, PE_R(0.2), 6, 1);
        rb4->addIgnoreCollisionId(rb5->getGlobalId());
        _world.addRigidBody(rb5);

        // add the second ball joint constraint
        auto c2 = new pe_physics_constraint::BallJointConstraint();
        c2->setObjectA(rb4);
        c2->setObjectB(rb5);
        c2->setAnchorA(pe::Vector3(0, 0, 0));
        c2->setAnchorB(pe::Vector3(0, PE_R(-3.3), 0));
        _world.addConstraint(c2);

        // add the third ball joint constraint
        auto c3 = new pe_physics_constraint::BallJointConstraint();
        c3->setObjectA(rb3);
        c3->setObjectB(rb4);
        c3->setAnchorA(pe::Vector3(0, PE_R(3.3), 0));
        c3->setAnchorB(pe::Vector3(0, 0, 0));
        _world.addConstraint(c3);

        /***************** hinge joint ****************/
        // add a pole base
        pe::Transform trans1, trans2;
        trans1.setRotation(pe::Vector3::right(), PE_PI / 6);
        trans1.setOrigin(pe::Vector3(4, 8, 0));
        auto rb6 = createCapsuleRigidBody(trans1, PE_R(0.2), 1, 1);
        rb6->setKinematic(true);
        _world.addRigidBody(rb6);

        // add a stick to rotate
        trans2.setRotation(-pe::Vector3::right(), PE_PI / 2);
        trans2.setOrigin(pe::Vector3(2, 0, 0));
        rb7 = createCapsuleRigidBody(trans1 * trans2, PE_R(0.2), PE_R(4), 1);
        rb6->addIgnoreCollisionId(rb7->getGlobalId());
        _world.addRigidBody(rb7);

        // add a hinge joint constraint
        c4 = new pe_physics_constraint::HingeJointConstraint();
        c4->setObjectA(rb6);
        c4->setObjectB(rb7);
        c4->setAnchorA(pe::Vector3(0, 0, 0));
        c4->setAnchorB(pe::Vector3(0, -2, 0));
        c4->setAxisA(pe::Vector3::up());
        c4->setAxisB(-pe::Vector3::right());
        c4->setLimitType(pe_physics_constraint::ConstraintLimitType::CLT_LOWER_UPPER);
        c4->setMinAngle(-PE_PI / 3);
        c4->setMaxAngle(PE_PI / 3);
        _world.addConstraint(c4);

        // add a second stick to rotate
        trans2.setRotation(pe::Vector3::right(), PE_PI / 2);
        trans2.setOrigin(pe::Vector3(4, 0, 0.75));
        auto rb8 = createCapsuleRigidBody(trans1 * trans2, PE_R(0.2), PE_R(2), 1);
        rb7->addIgnoreCollisionId(rb8->getGlobalId());
        _world.addRigidBody(rb8);

        // add a second hinge joint constraint
        auto c5 = new pe_physics_constraint::HingeJointConstraint();
        c5->setObjectA(rb7);
        c5->setObjectB(rb8);
        c5->setAnchorA(pe::Vector3(0, 2, 0));
        c5->setAnchorB(pe::Vector3(0, -0.75, 0));
        c5->setAxisA(pe::Vector3::up());
        c5->setAxisB(-pe::Vector3::forward());
        _world.addConstraint(c5);

        /***************** slider joint ****************/
        // add a slider base
        pe::Transform trans3, trans4, trans5;
        trans3.setRotation(pe::Vector3::up(), -PE_PI / 4);
        trans3.setOrigin(pe::Vector3(-1, 6, 0));
        trans4.setRotation(pe::Vector3::forward(), PE_PI / 6);
        trans4.setOrigin(pe::Vector3::zeros());
        auto rb9_0 = createBoxRigidBody(trans3 * trans4, pe::Vector3(PE_R(0.3), 10, PE_R(0.3)), 1);
        _world.addRigidBody(rb9_0);
        trans5.setBasis(pe::Matrix3::identity());
        trans5.setOrigin(pe::Vector3(0, PE_R(5.2), 0));
        auto rb9_1 = createBoxRigidBody(trans3 * trans4 * trans5, pe::Vector3(1, PE_R(0.4), 1), 1);
        // rb9_1->setKinematic(true);
        _world.addRigidBody(rb9_1);
        trans5.setOrigin(pe::Vector3(0, PE_R(-5.2), 0));
        auto rb9_2 = createBoxRigidBody(trans3 * trans4 * trans5, pe::Vector3(1, PE_R(0.4), 1), 1);
        // rb9_2->setKinematic(true);
        _world.addRigidBody(rb9_2);
        rb9_0->addIgnoreCollisionId(rb9_1->getGlobalId());
        rb9_0->addIgnoreCollisionId(rb9_2->getGlobalId());

        // add a slider
        trans5.setOrigin(pe::Vector3(0, PE_R(4.4), 0));
        rb10 = createBoxRigidBody(trans3 * trans4 * trans5, pe::Vector3(1, PE_R(1.2), 1), 1);
        rb9_0->addIgnoreCollisionId(rb10->getGlobalId());
        _world.addRigidBody(rb10);

        // add a slider joint constraint
        auto c6 = new pe_physics_constraint::SliderJointConstraint();
        c6->setObjectA(rb9_0);
        c6->setObjectB(rb10);
        c6->setAnchorA(pe::Vector3(0, 0, 0));
        c6->setAnchorB(pe::Vector3(0, 0, 0));
        c6->setAxisA(pe::Vector3::up());
        c6->setAxisB(pe::Vector3::up());
        c6->setLimitType(pe_physics_constraint::ConstraintLimitType::CLT_LOWER_UPPER);
        c6->setMaxPosition(PE_R(4.4));
        c6->setMinPosition(-PE_R(4.4));
        _world.addConstraint(c6);

        auto s_dof_1 = new pe_physics_constraint::SixDofConstraint();
        s_dof_1->setObjectA(rb9_0);
        s_dof_1->setObjectB(rb9_1);
        s_dof_1->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 5.2, 0)));
        s_dof_1->setFrameB(pe::Transform::identity());
        s_dof_1->setXPosFixed(true);
        s_dof_1->setYPosFixed(true);
        s_dof_1->setZPosFixed(true);
        _world.addConstraint(s_dof_1);

        auto s_dof_2 = new pe_physics_constraint::SixDofConstraint();
        s_dof_2->setObjectA(rb9_0);
        s_dof_2->setObjectB(rb9_2);
        s_dof_2->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5.2, 0)));
        s_dof_2->setFrameB(pe::Transform::identity());
        s_dof_2->setXPosFixed(true);
        s_dof_2->setYPosFixed(true);
        s_dof_2->setZPosFixed(true);
        _world.addConstraint(s_dof_2);

        /***************** 6-dof joint ****************/
        person = createCapsuleRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 4), pe::Vector3(0, 21, -3)),
                                               PE_R(0.5), 1, 4);
        _world.addRigidBody(person);
        auto sdof = new pe_physics_constraint::SixDofConstraint();
        sdof->setObjectA(nullptr);
        sdof->setObjectB(person);
        sdof->setFrameA(pe::Transform::identity());
        sdof->setFrameB(pe::Transform::identity());
        sdof->setXRotFixed(true);
        sdof->setZRotFixed(true);
        _world.addConstraint(sdof);
    }

    pe_physics_object::RigidBody* rb4 = nullptr;
    pe_physics_object::RigidBody* rb7 = nullptr;
    pe_physics_object::RigidBody* rb10 = nullptr;
    pe_physics_constraint::HingeJointConstraint* c4 = nullptr;
    pe_physics_object::RigidBody* person = nullptr;
    void step() override {
        // if (pe_interface::Viewer::getKeyState('i') == 0 && person != nullptr) {
        //     person->addTorque(pe::Vector3::right() * -10);
        // }
        // if (pe_interface::Viewer::getKeyState('k') == 0 && person != nullptr) {
        //     person->addTorque(pe::Vector3::right() * 10);
        // }
        // if (pe_interface::Viewer::getKeyState('j') == 0 && person != nullptr) {
        //     person->addTorque(pe::Vector3::forward() * 10);
        // }
        // if (pe_interface::Viewer::getKeyState('l') == 0 && person != nullptr) {
        //     person->addTorque(pe::Vector3::forward() * -10);
        // }
        // if (pe_interface::Viewer::getKeyState('u') == 0 && person != nullptr) {
        //     person->addTorque(pe::Vector3::up() * 10);
        // }
        // if (pe_interface::Viewer::getKeyState('o') == 0 && person != nullptr) {
        //     person->addTorque(pe::Vector3::up() * -10);
        // }

        if (pe_interface::Viewer::getKeyState('i') == 0 && person != nullptr) {
            person->addCentralForce(-pe::Vector3::forward() * 50);
        }
        if (pe_interface::Viewer::getKeyState('k') == 0 && person != nullptr) {
            person->addCentralForce(pe::Vector3::forward() * 50);
        }
        if (pe_interface::Viewer::getKeyState('j') == 0 && person != nullptr) {
            person->addCentralForce(-pe::Vector3::right() * 50);
        }
        if (pe_interface::Viewer::getKeyState('l') == 0 && person != nullptr) {
            person->addCentralForce(pe::Vector3::right() * 50);
        }
        if (pe_interface::Viewer::getKeyState('u') == 0 && person != nullptr) {
            person->addCentralForce(pe::Vector3::up() * 50);
        }
        if (pe_interface::Viewer::getKeyState('o') == 0 && person != nullptr) {
            person->addCentralForce(-pe::Vector3::up() * 50);
        }

        if (pe_interface::Viewer::getKeyState('n') == 0 && rb4 != nullptr) {
            rb4->addCentralForce(pe::Vector3::up() * 25);
        }
        if (pe_interface::Viewer::getKeyState('m') == 0 && rb10 != nullptr) {
            rb10->addCentralForce(rb10->getTransform().getBasis() * pe::Vector3::up() * 50);
        }
        if (c4 != nullptr) {
            if (pe_interface::Viewer::getKeyState(',') == 0) {
                c4->setMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
                c4->setTargetSpeed(-2);
            } else if (pe_interface::Viewer::getKeyState('.') == 0) {
                c4->setMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
                c4->setTargetSpeed(2);
            } else {
                c4->setMotorType(pe_physics_constraint::ConstraintMotorType::CMT_NONE);
            }
        }
    }

protected:

    static pe_physics_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(PE_R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(PE_R(0.8)); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_physics_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass) {
        /* This function creates a sphere-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }

    static pe_physics_object::RigidBody* createCapsuleRigidBody(const pe::Transform& trans,
                                                              pe::Real radius, pe::Real height, pe::Real mass) {
        /* This function creates a cylinder-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::CapsuleShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(ConstraintsSimulator, 100)
