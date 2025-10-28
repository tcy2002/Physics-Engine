#include "intf/simulator.h"
#include "phys/constraint/constraint/ball_joint_constraint.h"

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
        _world.setGravity(pe::Vector3(0, R(-9.8), 0));
        _world.setSleepLinVel2Threshold(R(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(R(0.01)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(R(1.0));     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(30, 10, 30), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a ball base
        auto rb2 = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0.5, 0)),
                                         R(0.5), 10);
        rb2->setKinematic(true);
        _world.addRigidBody(rb2);

        // add a stick
        pe::Transform trans;
        trans.setRotation(pe::Vector3::forward(), PE_PI / 6);
        trans.setOrigin(pe::Vector3(-R(3.3) * PE_SIN(PE_PI / 6), R(3.3) * PE_COS(PE_PI / 6) + R(0.5), 0));
        auto rb3 = createCylinderRigidBody(trans, R(0.2), 6, 1);
        rb2->addIgnoreCollisionId(rb3->getGlobalId());
        _world.addRigidBody(rb3);

        // add a ball joint constraint
        auto c1 = new pe_phys_constraint::BallJointConstraint();
        c1->setObjectA(rb2);
        c1->setObjectB(rb3);
        c1->setAnchorA(pe::Vector3(0, 0, 0));
        c1->setAnchorB(pe::Vector3(0, R(-3.3), 0));
        _world.addConstraint(c1);

        // add the second sphere base
        trans.setBasis(pe::Matrix3::identity());
        trans.setOrigin(pe::Vector3(R(-6.6) * PE_SIN(PE_PI / 6), R(6.6) * PE_COS(PE_PI / 6) + R(0.5), 0));
        auto rb4 = createSphereRigidBody(trans, R(0.4), 10);
        rb4->addIgnoreCollisionId(rb3->getGlobalId());
        _world.addRigidBody(rb4);

        // add the second stick
        trans.setRotation(pe::Vector3::forward(), PE_PI / 2);
        trans.setOrigin(pe::Vector3(R(-3.3) - R(6.6) * PE_SIN(PE_PI / 6), R(6.6) * PE_COS(PE_PI / 6) + R(0.5), 0));
        auto rb5 = createCylinderRigidBody(trans, R(0.2), 6, 1);
        rb4->addIgnoreCollisionId(rb5->getGlobalId());
        _world.addRigidBody(rb5);

        // add the second ball joint constraint
        auto c2 = new pe_phys_constraint::BallJointConstraint();
        c2->setObjectA(rb4);
        c2->setObjectB(rb5);
        c2->setAnchorA(pe::Vector3(0, 0, 0));
        c2->setAnchorB(pe::Vector3(0, R(-3.3), 0));
        _world.addConstraint(c2);

        // add the third ball joint constraint
        auto c3 = new pe_phys_constraint::BallJointConstraint();
        c3->setObjectA(rb3);
        c3->setObjectB(rb4);
        c3->setAnchorA(pe::Vector3(0, R(3.3), 0));
        c3->setAnchorB(pe::Vector3(0, 0, 0));
        _world.addConstraint(c3);
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
        rb->setFrictionCoeff(R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(R(0.8)); // angular damping parameter (slows down the rotation speed)
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
        rb->setFrictionCoeff(R(0.5));
        rb->setRestitutionCoeff(R(0.5));
        rb->setAngularDamping(R(0.8));
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
        rb->setFrictionCoeff(R(0.5));
        rb->setRestitutionCoeff(R(0.5));
        rb->setAngularDamping(R(0.8));
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(ConstraintsSimulator, 100)
