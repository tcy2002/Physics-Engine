#include "intf/simulator.h"
#include "phys/vehicle/car/car_template.h"
#include "phys/vehicle/tank/tank_template.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class TerrainCarSimulator : public pe_intf::Simulator {
protected:
    // i/k: move forward/backward
    // j/l/m: turn left/right/straight
    pe_phys_vehicle::CarTemplate* _car1;
    // i/j/k/l: move forward/leftward/backward/rightward
    // u/o: rotate barrel leftward/rightward
    pe_phys_vehicle::TankTemplate* _tank1;

public:
    TerrainCarSimulator(): _car1(nullptr) {}
    virtual ~TerrainCarSimulator() { delete _car1; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, R(-9.8), 0));

        // add a terrain ground
        auto rb1 = createConcaveRigidBody(TERRAIN_CAR_DEMO_SOURCE_DIR "/terrain.obj",
                                          pe::Transform(pe::Matrix3::identity(), pe::Vector3(-10, -1200, -10)),
                                          10000, 0.1);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1); // a rigidbody must be added into the _world to perform physical effects

        // add a car
        // _car1 = new pe_phys_vehicle::CarTemplate();
        // _car1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 5, 0)));
        // _car1->init(&_world);

        // add a tank
        _tank1 = new pe_phys_vehicle::TankTemplate();
        _tank1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 13, 0)));
        _tank1->init(&_world);
    }

    void step() override {
        /* Called every frame to update the physics world */

        // update the car
        // _car1->advance(_world.getDt());
        // if (pe_intf::Viewer::getKeyState('i') == 0) {
        //     _car1->moveForward();
        // } else if (pe_intf::Viewer::getKeyState('k') == 0) {
        //     _car1->moveBackward();
        // } else if (pe_intf::Viewer::getKeyState('j') == 0) {
        //     _car1->turnLeft();
        // } else if (pe_intf::Viewer::getKeyState('l') == 0) {
        //     _car1->turnRight();
        // } else if (pe_intf::Viewer::getKeyState('m') == 0) {
        //     _car1->turnStraight();
        // } else if (pe_intf::Viewer::getKeyState(' ') == 0) {
        //     _car1->brake();
        // } else {
        //     _car1->idle();
        // }

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
    static pe_phys_object::RigidBody* createConcaveRigidBody(const std::string& obj_path, const pe::Transform& trans, pe::Real mass, pe::Real size) {
        static pe::Mesh mesh;
        if (mesh.empty())
            pe::Mesh::loadFromObj(obj_path, mesh, pe::Vector3(size, size, size));
        auto rb = new pe_phys_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_phys_shape::ConcaveMeshShape();
        shape->setMeshPath(obj_path);
        shape->setScale(pe::Vector3(3, 3, 3));
        shape->setMesh(mesh);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(R(0.5));
        rb->setRestitutionCoeff(R(0.5));
        rb->setKinematic(true);
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
        rb->setFrictionCoeff(R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(R(0.8)); // angular damping parameter (slows down the rotation speed)
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(TerrainCarSimulator, 100)
