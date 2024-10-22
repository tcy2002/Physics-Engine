#include "intf/simulator.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `x` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class BombSimulator : public pe_intf::Simulator<pe_intf::UseViewer::True> {
public:
    BombSimulator() {}
    virtual ~BombSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, -9.8, 0));
        _world.setSleepLinVel2Threshold(0.04); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(0.08); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(1.0);     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, -5, 0)),
                                      pe::Vector3(250, 10, 250), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add tower1
        createTower(pe::Vector3(0, 0, -20), 4, 12, 8);
        createTower(pe::Vector3(0, 0, -20), 6, 12, 12);
        createTower(pe::Vector3(0, 0, -20), 8, 11, 16);
        createTower(pe::Vector3(0, 0, -20), 10, 10, 20);
        createTower(pe::Vector3(0, 0, -20), 12, 9, 24);
        createTower(pe::Vector3(0, 0, -20), 14, 8, 28);

        // add tower2
        createTower(pe::Vector3(0, 0, -60), 4, 28, 8);
        createTower(pe::Vector3(0, 0, -60), 6, 27, 12);
        createTower(pe::Vector3(0, 0, -60), 8, 26, 16);

        //// add tower3
        //createTower(pe::Vector3(0, 0, -100), 4, 28, 8);
        //createTower(pe::Vector3(0, 0, -100), 6, 27, 12);
        //createTower(pe::Vector3(0, 0, -100), 8, 26, 16);

        // add a bomb
        auto rb2 = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                       pe::Vector3(0, 2, 50)),
                                         1.2, 50);
        rb2->setLinearVelocity(pe::Vector3(0, 0, -100)); // give an initial velocity
        _world.addRigidBody(rb2);
    }

    void createTower(const pe::Vector3& pos, pe::Real radius, int layer, int brick_per_layer) {
        /* This function creates a tower of cubic bricks, how it is built is not important */

        pe::Real angle = pe::Real(2.0) * PE_PI / pe::Real(brick_per_layer);
        pe::Real brick_length = radius * angle / pe::Real(1.25);
        pe::Real brick_width = brick_length / pe::Real(4.0);
        pe::Real brick_height = brick_width * pe::Real(1.5);

        for (int i = 0; i < layer; i++) {
            pe::Real offset = (i % 2) * angle / pe::Real(2.0);
            for (int n = 0; n < brick_per_layer; n++) {
                pe::Real brick_angle = offset + n * angle;
                pe::Matrix3 mat;
                mat.setRotation(pe::Vector3(0, 1, 0), -brick_angle);
                pe::Vector3 vec;
                vec.x = radius * std::cos(brick_angle);
                vec.y = brick_height * pe::Real(0.5 + i);
                vec.z = radius * std::sin(brick_angle);
                auto rb = createBoxRigidBody(pe::Transform(mat, pos + vec),
                                             pe::Vector3(brick_width, brick_height, brick_length), 1.0);
                _world.addRigidBody(rb);
            }
        }
    }

protected:
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigid body */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass)); // inertia tensor matrix
        rb->setFrictionCoeff(0.5); // friction coefficient
        rb->setRestitutionCoeff(0.5); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(0.8); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass) {
        /* This function creates a sphere-shaped rigid body */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(BombSimulator, 200)
