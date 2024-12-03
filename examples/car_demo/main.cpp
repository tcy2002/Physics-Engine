#include "intf/simulator.h"
#include "phys/vehicle/car/car_template.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class CarSimulator : public pe_intf::Simulator {
protected:
    // i/k: move forward/backward
    // j/l/m: turn left/right/straight
    pe_phys_vehicle::CarTemplate* _car1;

public:
    CarSimulator(): _car1(nullptr) {}
    virtual ~CarSimulator() { delete _car1; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, R(-9.8), 0));

        // add a ground
        auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                                    pe::Vector3(0, R(-0.5), -10)),
                                      pe::Vector3(30, 1, 30), 10000);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a slope
        auto rb2 = createBoxRigidBody(pe::Transform(pe::Matrix3::Identity(),
                                                    pe::Vector3(0, 0, 0)),
                                      pe::Vector3(10, R(0.5), 10), 8);
        rb2->setKinematic(true);
        pe::Matrix3 mat = Eigen::AngleAxis<pe::Real>(PE_PI / R(12.0), pe::Vector3::UnitX()).toRotationMatrix();
        rb2->setTransform(pe::Transform(mat, pe::Vector3(0, R(1.4), -10)));
        _world.addRigidBody(rb2);

        // add a car
        _car1 = new pe_phys_vehicle::CarTemplate();
        _car1->setTransform(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 5, 0)));
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

        //std::cout << _car1->getTransform().getOrigin() << std::endl;
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
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(CarSimulator, 100)
