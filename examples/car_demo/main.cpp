#include "intf/simulator.h"
#include "phys/vehicle/car/car_template.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `x` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class CarSimulator : public pe_intf::Simulator<pe_intf::UseViewer::True> {
protected:
    // i/j/k/l: move forward/leftward/backward/rightward
    // u/o: rotate barrel leftward/rightward
    // CASE sensitive
    // do not try to drive the car upside down, it will cause something unexpected <^v^>
    pe_phys_vehicle::CarTemplate* _car1;

public:
    CarSimulator(): _car1(nullptr) {}
    virtual ~CarSimulator() { delete _car1; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, pe::Real(-9.8), 0));

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, -5, 0)),
                                      pe::Vector3(250, 10, 250), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a slope
        auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                    pe::Vector3(0, 0, 0)),
                                      pe::Vector3(10, pe::Real(0.3), 10), 8);
        rb2->setKinematic(true);
        pe::Matrix3 mat;
        mat.setRotation(pe::Vector3(1, 0, 0), PE_PI / pe::Real(12.0));
        rb2->setTransform(pe::Transform(mat, pe::Vector3(0, pe::Real(1.4), -10)));
        _world.addRigidBody(rb2);

        // add a car
        _car1 = new pe_phys_vehicle::CarTemplate();
        _car1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 5, 0)));
        _car1->init(&_world);
    }

    void step() override {
        /* Called every frame to update the physics world */

        // update the car
        _car1->advance(_world.getDt());
        if (pe_intf::Viewer::getKeyState('i') == 0) {
            _car1->moveForward();
        } else if (pe_intf::Viewer::getKeyState('k') == 0) {
            _car1->moveBackward();
        } else if (pe_intf::Viewer::getKeyState('j') == 0) {
            _car1->turnLeft();
        } else if (pe_intf::Viewer::getKeyState('l') == 0) {
            _car1->turnRight();
        } else if (pe_intf::Viewer::getKeyState('m') == 0) {
            _car1->turnStraight();
        } else if (pe_intf::Viewer::getKeyState(' ') == 0) {
            _car1->brake();
        } else {
            _car1->idle();
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
        rb->setLocalInertia(shape->calcLocalInertia(mass)); // inertia tensor matrix
        rb->setFrictionCoeff(pe::Real(0.5)); // friction coefficient
        rb->setRestitutionCoeff(pe::Real(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(pe::Real(0.8)); // angular damping parameter (slows down the rotation speed)
        return rb;
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(CarSimulator, 100)
