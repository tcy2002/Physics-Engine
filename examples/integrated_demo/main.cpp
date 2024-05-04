#include "intf/simulator.h"
#include "phys/vehicle/tank/tank_template.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `r` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class TankSimulator : public pe_intf::Simulator<pe_intf::UseViewer::True> {
protected:
    // i/j/k/l: move forward/leftward/backward/rightward
    // u/o: rotate barrel leftward/rightward
    // CASE sensitive
    // do not try to drive the tank upside down, it will cause something unexpected <^v^>
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
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a tank
        _tank = new pe_phys_vehicle::TankTemplate();
        _tank->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 5, 0)));
        _tank->init(&_world);

        // add a building
        createBuilding(pe::Vector3(8, 6, 8), pe::Vector3(0, 0, -20), 1.0);
    }

    void step() override {
        /* Called every frame to update the physics world */

        // update the tank
        _tank->advance(_world.getDt());

        // move
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

        // shoot
        if (pe_intf::Viewer::getKeyState('m') == 2) {
            _tank->shoot(&_world, 50, 20, 0.3, 5);
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
        rb->setFrictionCoeff(0.5); // friction coefficient
        rb->setRestitutionCoeff(0.5); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(0.8); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_phys_object::RigidBody* createBoxFracturableObject(const pe::Transform& trans,
                                                                 const pe::Vector3& size, pe::Real mass,
                                                                 pe::Real th) {
        /* This function creates a box-shaped fracturable body */

        auto rb = new pe_phys_object::FracturableObject();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setLocalInertia(shape->calcLocalInertia(mass)); // inertia tensor matrix
        rb->setFrictionCoeff(0.5); // friction coefficient
        rb->setRestitutionCoeff(0.5); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(0.8); // angular damping parameter (slows down the rotation speed)
        rb->setThreshold(th);
        return rb;
    }

    void createBuilding(const pe::Vector3& size, const pe::Vector3& pos, pe::Real wall_thickness) {
        // callback function for collision
        static auto callback = [&](
                pe_phys_object::RigidBody* self, pe_phys_object::RigidBody* other,
                const pe::Vector3& pos, const pe::Vector3& nor, const pe::Vector3& vel) {
            if (other->getTag() != "bullet" || !self->isFracturable()) return;
            auto fb = (pe_phys_object::FracturableObject*)self;
            if (vel.norm() * other->getMass() < fb->getThreshold() * 800) return;
            pe_phys_fracture::FractureSource src;
            src.position = pos;
            src.type = pe_phys_fracture::FractureType::Sphere;
            src.intensity = pe::Vector3(1.5, 1.5, 1.5);
            _world.addFractureSource(src);
            _world.removeRigidBody(other);
        };

        // ceiling
        auto rb1 = createBoxFracturableObject(
                pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y - wall_thickness / 2, 0)),
                pe::Vector3(size.x - wall_thickness * 2, wall_thickness, size.z - wall_thickness * 2), 8, 1);
        rb1->setKinematic(true);
        rb1->addCollisionCallback(callback);
        _world.addRigidBody(rb1);

        // front wall
        auto rb2 = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y / 2,
                                                                                                       size.z / 2 -
                                                                                                       wall_thickness /
                                                                                                       2)),
                                              pe::Vector3(size.x - wall_thickness * 2, size.y, wall_thickness), 8, 1);
        rb2->setKinematic(true);
        rb2->addCollisionCallback(callback);
        _world.addRigidBody(rb2);

        // back wall
        auto rb3 = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y / 2,
                                                                                                       -size.z / 2 +
                                                                                                       wall_thickness /
                                                                                                       2)),
                                              pe::Vector3(size.x - wall_thickness * 2, size.y, wall_thickness), 8, 1);
        rb3->setKinematic(true);
        rb3->addCollisionCallback(callback);
        _world.addRigidBody(rb3);

        // left wall
        auto rb4 = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(
                                                      size.x / 2 - wall_thickness / 2, size.y / 2, 0)),
                                              pe::Vector3(wall_thickness, size.y, size.z), 8, 1);
        rb4->setKinematic(true);
        rb4->addCollisionCallback(callback);
        _world.addRigidBody(rb4);

        // right wall
        auto rb5 = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(
                                                      -size.x / 2 + wall_thickness / 2, size.y / 2, 0)),
                                              pe::Vector3(wall_thickness, size.y, size.z), 8, 1);
        rb5->setKinematic(true);
        rb5->addCollisionCallback(callback);
        _world.addRigidBody(rb5);
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(TankSimulator, 0.016, 10000000)
