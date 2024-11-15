#include <phys/shape/default_mesh.h>

#include "intf/simulator.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `x` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class CompoundSimulator : public pe_intf::Simulator {
public:
    CompoundSimulator() {}
    virtual ~CompoundSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, pe::Real(-9.8), 0));
        _world.setSleepLinVel2Threshold(pe::Real(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(pe::Real(0.01)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(pe::Real(1.0));     // sleep time threshold

        // // add a ground
        // auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
        //                               pe::Vector3(30, 10, 30), 10000);
        // rb1->setKinematic(true);
        // _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        auto rb1 = new pe_phys_object::RigidBody();
        rb1->setMass(10000);
        auto shape1 = new pe_phys_shape::ConvexMeshShape();
        pe::Mesh box_mesh = PE_BOX_DEFAULT_MESH;
        for (auto& v : box_mesh.vertices) {
            v.position.x *= 30;
            v.position.y *= 10;
            v.position.z *= 30;
        }
        shape1->setMesh(box_mesh);
        rb1->setCollisionShape(shape1);
        rb1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)));
        rb1->setKinematic(true);
        _world.addRigidBody(rb1);

        // // add some other dynamic objects
        // auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 2, 0)),
        //                               pe::Vector3(1, 1, 1), 1);
        // _world.addRigidBody(rb2);
        // auto rb3 = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(pe::Real(0.1), 4, pe::Real(0.1))),
        //                                  pe::Real(0.5), 1);
        // _world.addRigidBody(rb3);
        pe::Transform trans = pe::Transform::identity();
        // trans.setRotation({0, 0, 1}, PE_PI / 6);
        trans.setOrigin({5, 6, -5});
        auto rb4 = createCylinderRigidBody(trans,
                                           pe::Real(0.5), 1, 1);
        _world.addRigidBody(rb4);

        // // add some compound-shaped rigidbodies
        // for (int i = 0; i < 10; i++) {
        //     auto rb = createCompoundRigidBody(pe::Transform(pe::Matrix3::identity(),
        //                                                     pe::Vector3(0, pe::Real(10 + i * 4), 0)), 1);
        //     _world.addRigidBody(rb);
        // }

        //saveScene("");
    }

protected:
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
        auto shape = new pe_phys_shape::CompoundShape();
        shape->addShape(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)), pe::Real(0.4), shape1);
        shape->addShape(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1, 0)), pe::Real(0.1), shape2);
        shape->addShape(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -1, 0)), pe::Real(0.1), shape3);
        pe::Transform trans1;
        trans1.setRotation(pe::Vector3::forward(), PE_PI / 2);
        trans1.setTranslation(pe::Vector3(1, 0, 0));
        shape->addShape(trans1, pe::Real(0.1), shape4);
        trans1.setRotation(pe::Vector3::forward(), PE_PI / 2);
        trans1.setTranslation(pe::Vector3(-1, 0, 0));
        shape->addShape(trans1, pe::Real(0.1), shape5);
        trans1.setRotation(pe::Vector3::right(), PE_PI / 2);
        trans1.setTranslation(pe::Vector3(0, 0, 1));
        shape->addShape(trans1, pe::Real(0.1), shape6);
        trans1.setRotation(pe::Vector3::right(), PE_PI / 2);
        trans1.setTranslation(pe::Vector3(0, 0, -1));
        shape->addShape(trans1, pe::Real(0.1), shape7);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
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
        rb->setFrictionCoeff(pe::Real(0.5));
        rb->setRestitutionCoeff(pe::Real(0.5));
        rb->setAngularDamping(pe::Real(0.8));
        return rb;
    }
};

// Simulator class, Delta time, Max frame
PE_CUSTOM_MAIN(CompoundSimulator, 100)
