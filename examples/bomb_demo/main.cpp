#include <iostream>
#include "intf/simulator.h"
#include "phys/object/rigidbody.h"
#include "phys/object/fracturable_object.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"

class BombSimulator : public pe_intf::Simulator<true> {
public:
    BombSimulator() {}
    virtual ~BombSimulator() {}

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity
        _world.setGravity(pe::Vector3(0, -9.8, 0));

        // create a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, -0.5, 0)),
                                      pe::Vector3(250, 1, 250), 8);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1);

        // tower 1
        createTower(pe::Vector3(0, 0, 0), 4, 12, 8);
        createTower(pe::Vector3(0, 0, 0), 6, 12, 12);
        createTower(pe::Vector3(0, 0, 0), 8, 11, 16);
        createTower(pe::Vector3(0, 0, 0), 10, 10, 20);
        createTower(pe::Vector3(0, 0, 0), 12, 9, 24);
        createTower(pe::Vector3(0, 0, 0), 14, 8, 28);

        // tower 2
        createTower(pe::Vector3(0, 0, -50), 4, 28, 8);
        createTower(pe::Vector3(0, 0, -50), 6, 27, 12);
        createTower(pe::Vector3(0, 0, -50), 8, 26, 16);

        // bomb
        auto rb2 = createSphereRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                       pe::Vector3(0, 5, 50)),
                                         1, 50);
        rb2->setLinearVelocity(pe::Vector3(0, 0, -500));
        _world.addRigidBody(rb2);
    }

    void createTower(const pe::Vector3& pos, pe::Real radius, int layer, int brick_per_layer) {
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
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass) {
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

    static pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Transform& trans,
                                                              pe::Real radius, pe::Real height, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::CylinderShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::FracturableObject* createFracturableObject(const pe::Transform& trans,
                                                                      const pe::Vector3& size, pe::Real th) {
        auto rb = new pe_phys_object::FracturableObject();
        rb->setMass(1.0);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(1.0));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        rb->setThreshold(th);
        return rb;
    }
};

int main() {
    BombSimulator simulator;
    simulator.run(0.01, 100);
    return 0;
}