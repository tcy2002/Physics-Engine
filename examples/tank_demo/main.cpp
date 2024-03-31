#include <iostream>
#include "intf/simulator.h"
#include "intf/viewer.h"
#include "phys/object/rigidbody.h"
#include "phys/vehicle/tank/tank_template.h"

class TankSimulator : public pe_intf::Simulator<true> {
protected:
    pe_phys_vehicle::TankTemplate* _tank;

public:
    TankSimulator(): _tank(nullptr) {}
    virtual ~TankSimulator() { delete _tank; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity
        _world.setGravity(pe::Vector3(0, -9.8, 0));

        // create a ground
        auto rb1 = createBoxRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(1000, 1, 1000), 8);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1);

        // add a slope
        auto rb2 = createBoxRigidBody(pe::Vector3(0, 0, 0), pe::Vector3(10, 0.3, 10), 8);
        rb2->setKinematic(true);
        pe::Matrix3 mat;
        mat.setRotation(pe::Vector3(1, 0, 0), PE_PI / pe::Real(12.0));
        rb2->setTransform(pe::Transform(mat, pe::Vector3(0, 1.4, -10)));
        _world.addRigidBody(rb2);

        // add some steps
        for (int i = 0; i < 10; i++) {
            auto rb = createBoxRigidBody(pe::Vector3(0, 0.15 + 0.3 * i, -30 - i * 0.5), pe::Vector3(10, 0.3, 10), 8);
            rb->setKinematic(true);
            _world.addRigidBody(rb);
        }

        // add a tank
        _tank = new pe_phys_vehicle::TankTemplate();
        _tank->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1.5, 0)));
        _tank->init(&_world);
    }

    void step() override {
        /* Called every frame to update the physics world */

        // update the tank
        _tank->advance(_world.getDt());
        if (pe_intf::Viewer::getKeyState('i') == 0) {
            _tank->moveForward();
        } else if (pe_intf::Viewer::getKeyState('k') == 0) {
            _tank->moveBackward();
        } else if (pe_intf::Viewer::getKeyState('j') == 0) {
            _tank->turnLeft();
        } else if (pe_intf::Viewer::getKeyState('l') == 0) {
            _tank->turnRight();
        } else if (pe_intf::Viewer::getKeyState('u') == 0) {
            _tank->barrelRotLeft(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('o') == 0) {
            _tank->barrelRotRight(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState(' ') == 0) {
            _tank->brake();
        } else {
            _tank->idle();
        }
    }

protected:
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Vector3& pos, const pe::Vector3& size, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createSphereRigidBody(const pe::Vector3& pos, pe::Real radius, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }

    static pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Vector3& pos, pe::Real radius, pe::Real height, pe::Real mass) {
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::CylinderShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        rb->setLocalInertia(shape->calcLocalInertia(mass));
        rb->setFrictionCoeff(0.5);
        rb->setRestitutionCoeff(0.5);
        rb->setAngularDamping(0.8);
        return rb;
    }
};

int main() {
    TankSimulator simulator;
    simulator.run(0.01, 100000);
    return 0;
}