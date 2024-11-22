#include "intf/simulator.h"
#include "phys/vehicle/tank/tank_template.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class TankSimulator : public pe_intf::Simulator {
protected:
    // i/j/k/l: move forward/leftward/backward/rightward
    // u/o: rotate barrel leftward/rightward
    pe_phys_vehicle::TankTemplate* _tank1;

public:
    TankSimulator(): _tank1(nullptr) {}
    virtual ~TankSimulator() { delete _tank1; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, R(-9.8), 0));

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, -5, 0)),
                                      pe::Vector3(50, 10, 50), 8);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a slope
        auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, 0, 0)),
                                      pe::Vector3(10, R(0.3), 10), 8);
        rb2->setKinematic(true);
        pe::Matrix3 mat;
        mat.setRotation(pe::Vector3(1, 0, 0), PE_PI / R(12.0));
        rb2->setTransform(pe::Transform(mat, pe::Vector3(0, R(1.4), -10)));
        _world.addRigidBody(rb2);

        // add a tank
        _tank1 = new pe_phys_vehicle::TankTemplate();
        _tank1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 5, 0)));
        _tank1->init(&_world);
    }

    void step() override {
        /* Called every frame to update the physics world */

        // update the tank
        _tank1->advance(_world.getDt());
        if (pe_intf::Viewer::getKeyState('i') == 0) {
            _tank1->moveForward();
        } else if (pe_intf::Viewer::getKeyState('k') == 0) {
            _tank1->moveBackward();
        } else if (pe_intf::Viewer::getKeyState('j') == 0) {
            _tank1->turnLeft();
        } else if (pe_intf::Viewer::getKeyState('l') == 0) {
            _tank1->turnRight();
        } else if (pe_intf::Viewer::getKeyState('u') == 0) {
            _tank1->barrelRotLeft(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('o') == 0) {
            _tank1->barrelRotRight(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState(' ') == 0) {
            _tank1->brake();
        } else {
            _tank1->idle();
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
        rb->setFrictionCoeff(R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(R(0.8)); // angular damping parameter (slows down the rotation speed)
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
PE_CUSTOM_MAIN(TankSimulator, 100)
