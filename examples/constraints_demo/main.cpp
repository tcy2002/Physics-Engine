#include "intf/simulator.h"
#include "rigid/constraint/constraint/ball_joint_constraint.h"
#include "rigid/constraint/constraint/hinge_joint_constraint.h"
#include "rigid/constraint/constraint/slider_joint_constraint.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class ConstraintsSimulator : public pe_intf::Simulator {
public:
    ConstraintsSimulator() {}
    virtual ~ConstraintsSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        // _world.setSleepLinVel2Threshold(PE_R(0.01)); // linear velocity threshold for sleep
        // _world.setSleepAngVel2Threshold(PE_R(0.01)); // angular velocity threshold for sleep
        // _world.setSleepTimeThreshold(PE_R(1.0));     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(30, 10, 30), 10000);
        rb1->setKinematic(true);
        rb1->setRestitutionCoeff(0.5);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        /***************** ball joint ****************/
        // add a ball base
        auto rb2 = createSphereRigidBody(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 0.5, 0)),
                                         PE_R(0.5), 1);
        rb2->setKinematic(true);
        _world.addRigidBody(rb2);

        // add a stick
        pe::Transform trans;
        trans.setRotation(pe::Vector3::UnitZ(), PE_PI / 6);
        trans.setOrigin(pe::Vector3(-PE_R(3.3) * PE_SIN(PE_PI / 6), PE_R(3.3) * PE_COS(PE_PI / 6) + PE_R(0.5), 0));
        auto rb3 = createCylinderRigidBody(trans, PE_R(0.2), 6, 1);
        rb2->addIgnoreCollisionId(rb3->getGlobalId());
        _world.addRigidBody(rb3);

        // add a ball joint constraint
        auto c1 = new pe_phys_constraint::BallJointConstraint();
        c1->setObjectA(rb2);
        c1->setObjectB(rb3);
        c1->setAnchorA(pe::Vector3(0, 0, 0));
        c1->setAnchorB(pe::Vector3(0, PE_R(-3.3), 0));
        _world.addConstraint(c1);

        // add the second sphere base
        trans.setBasis(pe::Matrix3::Identity());
        trans.setOrigin(pe::Vector3(PE_R(-6.6) * PE_SIN(PE_PI / 6), PE_R(6.6) * PE_COS(PE_PI / 6) + PE_R(0.5), 0));
        rb4 = createSphereRigidBody(trans, PE_R(0.4), 1);
        rb4->addIgnoreCollisionId(rb3->getGlobalId());
        _world.addRigidBody(rb4);

        // add the second stick
        trans.setRotation(pe::Vector3::UnitZ(), PE_PI / 2);
        trans.setOrigin(pe::Vector3(PE_R(-3.3) - PE_R(6.6) * PE_SIN(PE_PI / 6), PE_R(6.6) * PE_COS(PE_PI / 6) + PE_R(0.5), 0));
        auto rb5 = createCylinderRigidBody(trans, PE_R(0.2), 6, 1);
        rb4->addIgnoreCollisionId(rb5->getGlobalId());
        _world.addRigidBody(rb5);

        // add the second ball joint constraint
        auto c2 = new pe_phys_constraint::BallJointConstraint();
        c2->setObjectA(rb4);
        c2->setObjectB(rb5);
        c2->setAnchorA(pe::Vector3(0, 0, 0));
        c2->setAnchorB(pe::Vector3(0, PE_R(-3.3), 0));
        _world.addConstraint(c2);

        // add the third ball joint constraint
        auto c3 = new pe_phys_constraint::BallJointConstraint();
        c3->setObjectA(rb3);
        c3->setObjectB(rb4);
        c3->setAnchorA(pe::Vector3(0, PE_R(3.3), 0));
        c3->setAnchorB(pe::Vector3(0, 0, 0));
        _world.addConstraint(c3);

        /***************** hinge joint ****************/
        // add a pole base
        trans.setRotation(pe::Vector3::UnitX(), PE_PI / 6);
        trans.setOrigin(pe::Vector3(4, 6, 0));
        auto rb6 = createCylinderRigidBody(trans, PE_R(0.2), 1, 1);
        rb6->setKinematic(true);
        _world.addRigidBody(rb6);

        // add a stick to rotate
        pe::Transform trans2;
        trans2.setRotation(-pe::Vector3::UnitZ(), PE_PI / 2);
        trans2.setOrigin(pe::Vector3(2, 0, 0));
        rb7 = createCylinderRigidBody(trans * trans2, PE_R(0.2), PE_R(4), 1);
        rb6->addIgnoreCollisionId(rb7->getGlobalId());
        _world.addRigidBody(rb7);

        // add a hinge joint constraint
        auto c4 = new pe_phys_constraint::HingeJointConstraint();
        c4->setObjectA(rb6);
        c4->setObjectB(rb7);
        c4->setAnchorA(pe::Vector3(0, 0, 0));
        c4->setAnchorB(pe::Vector3(0, -2, 0));
        c4->setAxisA(pe::Vector3::UnitY());
        c4->setAxisB(-pe::Vector3::UnitX());
        _world.addConstraint(c4);

        // add a second stick to rotate
        trans2.setRotation(pe::Vector3::UnitX(), PE_PI / 2);
        trans2.setOrigin(pe::Vector3(4, 0, 0.75));
        auto rb8 = createCylinderRigidBody(trans * trans2, PE_R(0.2), PE_R(2), 1);
        rb7->addIgnoreCollisionId(rb8->getGlobalId());
        _world.addRigidBody(rb8);

        // add a second hinge joint constraint
        auto c5 = new pe_phys_constraint::HingeJointConstraint();
        c5->setObjectA(rb7);
        c5->setObjectB(rb8);
        c5->setAnchorA(pe::Vector3(0, 2, 0));
        c5->setAnchorB(pe::Vector3(0, -0.75, 0));
        c5->setAxisA(pe::Vector3::UnitY());
        c5->setAxisB(-pe::Vector3::UnitZ());
        _world.addConstraint(c5);

        /***************** slider joint ****************/
        // add a slider base
        pe::Transform trans0;
        trans0.setRotation(pe::Vector3::UnitY(), -PE_PI / 4);
        trans0.setOrigin(pe::Vector3(-1, 6, 0));
        trans.setRotation(pe::Vector3::UnitZ(), PE_PI / 3);
        trans.setOrigin(pe::Vector3::Zero());
        auto rb9_0 = createBoxRigidBody(trans0 * trans, pe::Vector3(PE_R(0.3), 10, PE_R(0.3)), 1);
        rb9_0->setKinematic(true);
        _world.addRigidBody(rb9_0);
        trans2.setBasis(pe::Matrix3::Identity());
        trans2.setOrigin(pe::Vector3(0, PE_R(5.2), 0));
        auto rb9_1 = createBoxRigidBody(trans0 * trans * trans2, pe::Vector3(1, PE_R(0.4), 1), 1);
        rb9_1->setKinematic(true);
        _world.addRigidBody(rb9_1);
        trans2.setOrigin(pe::Vector3(0, PE_R(-5.2), 0));
        auto rb9_2 = createBoxRigidBody(trans0 * trans * trans2, pe::Vector3(1, PE_R(0.4), 1), 1);
        rb9_2->setKinematic(true);
        _world.addRigidBody(rb9_2);

        // add a slider
        trans2.setOrigin(pe::Vector3(0, PE_R(4.4), 0));
        rb10 = createBoxRigidBody(trans0 * trans * trans2, pe::Vector3(1, PE_R(1.2), 1), 1);
        rb9_0->addIgnoreCollisionId(rb10->getGlobalId());
        _world.addRigidBody(rb10);

        // add a slider joint constraint
        auto c6 = new pe_phys_constraint::SliderJointConstraint();
        c6->setObjectA(rb9_0);
        c6->setObjectB(rb10);
        c6->setAnchorA(pe::Vector3(0, 0, 0));
        c6->setAnchorB(pe::Vector3(0, 0, 0));
        c6->setAxisA(pe::Vector3::UnitY());
        c6->setAxisB(pe::Vector3::UnitY());
        _world.addConstraint(c6);
    }

    pe_phys_object::RigidBody* rb4 = nullptr;
    pe_phys_object::RigidBody* rb7 = nullptr;
    pe_phys_object::RigidBody* rb10 = nullptr;
    void step() override {
        if (pe_intf::Viewer::getKeyState('j') == 0 && rb4 != nullptr) {
            rb4->addCentralForce(pe::Vector3::UnitY() * 25);
        }
        if (pe_intf::Viewer::getKeyState('k') == 0 && rb10 != nullptr) {
            rb10->addCentralForce(rb10->getTransform().getBasis() * pe::Vector3::UnitY() * 25);
        }
        if (pe_intf::Viewer::getKeyState('l') == 0 && rb7 != nullptr) {
            rb7->addTorque(rb7->getTransform().getBasis() * -pe::Vector3::UnitX() * 40);
        }
    }

protected:
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(PE_R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(PE_R(0.8)); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass) {
        /* This function creates a sphere-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }

    static pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Transform& trans,
                                                              pe::Real radius, pe::Real height, pe::Real mass) {
        /* This function creates a cylinder-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::CylinderShape(radius, height);
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
