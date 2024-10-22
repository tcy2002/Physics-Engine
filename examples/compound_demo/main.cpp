#include "intf/simulator.h"
#include "phys/object/fracturable_object.h"
#include "phys/fracture/fracture_solver/simple_fracture_solver.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `x` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class FractureSimulator : public pe_intf::Simulator<pe_intf::UseViewer::True> {
public:
    FractureSimulator() {}
    virtual ~FractureSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, pe::Real(-9.8), 0));
        _world.setSleepLinVel2Threshold(pe::Real(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(pe::Real(0.01)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(pe::Real(1.0));     // sleep time threshold

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(250, 10, 250), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add some other dynamic objects
        auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 2, 0)),
                                      pe::Vector3(1, 1, 1), 1);
        _world.addRigidBody(rb2);
        auto rb3 = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(pe::Real(0.1), 4, pe::Real(0.1))),
                                         pe::Real(0.5), 1);
        _world.addRigidBody(rb3);
        auto rb4 = createCylinderRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(pe::Real(0.2), 6, pe::Real(0.2))),
                                           pe::Real(0.5), 1, 1);
        _world.addRigidBody(rb4);

        // add some compound-shaped rigidbodies
        for (int i = 0; i < 10; i++) {
            auto rb = createCompoundRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                            pe::Vector3(0, pe::Real(10 + i * 4), 0)), 1);
            _world.addRigidBody(rb);
        }
    }

protected:
    static pe::Matrix3 fromRotation(const pe::Vector3& axis, pe::Real angle) {
        /* This function creates a rotation matrix from an axis and an angle */

        pe::Real c = std::cos(angle);
        pe::Real s = std::sin(angle);
        pe::Real t = 1 - c;
        pe::Real x = axis.x;
        pe::Real y = axis.y;
        pe::Real z = axis.z;
        return pe::Matrix3(t * x * x + c, t * x * y - s * z, t * x * z + s * y,
                           t * x * y + s * z, t * y * y + c, t * y * z - s * x,
                           t * x * z - s * y, t * y * z + s * x, t * z * z + c);
    }

    static pe_phys_object::RigidBody* createCompoundRigidBody(const pe::Transform& trans, pe::Real mass) {
        /* This function creates a compound-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape1 = new pe_phys_shape::BoxShape(pe::Vector3(1, 1, 1));
        auto shape2 = new pe_phys_shape::CylinderShape(pe::Real(0.2), 1);
        auto shape3 = new pe_phys_shape::CylinderShape(pe::Real(0.2), 1);
        auto shape4 = new pe_phys_shape::CylinderShape(pe::Real(0.2), 1);
        auto shape5 = new pe_phys_shape::CylinderShape(pe::Real(0.2), 1);
        auto shape6 = new pe_phys_shape::CylinderShape(pe::Real(0.2), 1);
        auto shape7 = new pe_phys_shape::CylinderShape(pe::Real(0.2), 1);
        auto shapeBox = new pe_phys_shape::BoxShape(pe::Vector3(3, 3, 3));
        auto shape = new pe_phys_shape::CompoundShape();
        shape->addShape(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)), pe::Real(0.4), shape1);
        shape->addShape(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1, 0)), pe::Real(0.1), shape2);
        shape->addShape(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -1, 0)), pe::Real(0.1), shape3);
        shape->addShape(pe::Transform(fromRotation(pe::Vector3::forward(), PE_PI / 2), pe::Vector3(1, 0, 0)), pe::Real(0.1), shape4);
        shape->addShape(pe::Transform(fromRotation(pe::Vector3::forward(), PE_PI / 2), pe::Vector3(-1, 0, 0)), pe::Real(0.1), shape5);
        shape->addShape(pe::Transform(fromRotation(pe::Vector3::right(), PE_PI / 2), pe::Vector3(0, 0, 1)), pe::Real(0.1), shape6);
        shape->addShape(pe::Transform(fromRotation(pe::Vector3::right(), PE_PI / 2), pe::Vector3(0, 0, -1)), pe::Real(0.1), shape7);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shapeBox->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass)); // inertia tensor matrix
        rb->setFrictionCoeff(pe::Real(0.5)); // friction coefficient
        rb->setRestitutionCoeff(pe::Real(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(pe::Real(0.8)); // angular damping parameter (slows down the rotation speed)
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
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(pe::Real(0.5));
        rb->setRestitutionCoeff(pe::Real(0.5));
        rb->setAngularDamping(pe::Real(0.8));
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
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(pe::Real(0.5));
        rb->setRestitutionCoeff(pe::Real(0.5));
        rb->setAngularDamping(pe::Real(0.8));
        return rb;
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(FractureSimulator, 60)
