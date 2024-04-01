#include <iostream>
#include "intf/simulator.h"
#include "intf/viewer.h"
#include "phys/object/rigidbody.h"
#include "phys/vehicle/tank/tank_template.h"

// true/false: simulate with/without viewer
// if using viewer, press `r` to start simulation 
class TankSimulator : public pe_intf::Simulator<true> {
protected:
    // i/j/k/l: move forward/leftward/backward/rightward
    // u/p: rotate barrel leftward/rightward
    // CASE sensitive
    pe_phys_vehicle::TankTemplate* _tank;

public:
    TankSimulator(): _tank(nullptr) {}
    virtual ~TankSimulator() { delete _tank; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, -9.8, 0));

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), 
                                                    pe::Vector3(0, -0.5, 0)), 
                                      pe::Vector3(1000, 1, 1000), 8);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be put into the _world to perform physical effects

        // add a slope
        auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, 0, 0)),
                                      pe::Vector3(10, 0.3, 10), 8);
        rb2->setKinematic(true);
        pe::Matrix3 mat;
        mat.setRotation(pe::Vector3(1, 0, 0), PE_PI / pe::Real(12.0));
        rb2->setTransform(pe::Transform(mat, pe::Vector3(0, 1.4, -10)));
        _world.addRigidBody(rb2);

        // add some steps
        for (int i = 0; i < 10; i++) {
            auto rb = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                       pe::Vector3(0, 0.15 + 0.3 * i, -30 - i * 0.5)), 
                                         pe::Vector3(10, 0.3, 10), 8);
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
    static pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
        const pe::Vector3& size, pe::Real mass) {
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
};

int main() {
    TankSimulator simulator;
    // delta time per frame, max frame
    simulator.run(0.016, 100000);
    return 0;
}