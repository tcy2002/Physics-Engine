#include "intf/simulator.h"
#include "phys/vehicle/tank/tank_template.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"

// pe_intf::UseViewer::True/False: simulate with/without viewer
// If using viewer, press `x` to start simulation
// See SimpleViewer/include/opengl_viewer.h to learn the view control
class TankSimulator : public pe_intf::Simulator<pe_intf::UseViewer::True> {
protected:
    // i/j/k/l: move forward/leftward/backward/rightward
    // u/o: rotate barrel leftward/rightward
    // CASE sensitive
    // do not try to drive the tank upside down, it will cause something unexpected <^v^>
    pe_phys_vehicle::TankTemplate* _tank1;
    pe_phys_vehicle::TankTemplate* _tank2;

public:
    TankSimulator(): _tank1(nullptr), _tank2(nullptr) {}
    virtual ~TankSimulator() { delete _tank1; delete _tank2; }

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: outward screen)
        _world.setGravity(pe::Vector3(0, -9.8, 0));

        // tank 1
        _tank1 = new pe_phys_vehicle::TankTemplate();
        _tank1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-5, 5, 0)));
        _tank1->init(&_world);

        // tank 2
        _tank2 = new pe_phys_vehicle::TankTemplate();
        _tank2->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(5, 5, 0)));
        _tank2->init(&_world);

        // create the urban layout
        createUrbanLayout();
    }

    void step() override {
        /* Called every frame to update the physics world */

        // update the tank
        _tank1->advance(_world.getDt());
        _tank2->advance(_world.getDt());

        // move tank1
        if (pe_intf::Viewer::getKeyState('p') == 0) {
            _tank1->moveForward();
        } else if (pe_intf::Viewer::getKeyState(';') == 0) {
            _tank1->moveBackward();
        } else if (pe_intf::Viewer::getKeyState('l') == 0) {
            _tank1->turnLeft();
        } else if (pe_intf::Viewer::getKeyState('\'') == 0) {
            _tank1->turnRight();
        } else if (pe_intf::Viewer::getKeyState('o') == 0) {
            _tank1->barrelRotLeft(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('[') == 0) {
            _tank1->barrelRotRight(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('i') == 0) {
            _tank1->barrelRotUp(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('k') == 0) {
            _tank1->barrelRotDown(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('.') == 0) {
            _tank1->brake();
        } else {
            _tank1->idle();
        }

        // move tank2
        if (pe_intf::Viewer::getKeyState('y') == 0) {
            _tank2->moveForward();
        } else if (pe_intf::Viewer::getKeyState('h') == 0) {
            _tank2->moveBackward();
        } else if (pe_intf::Viewer::getKeyState('g') == 0) {
            _tank2->turnLeft();
        } else if (pe_intf::Viewer::getKeyState('j') == 0) {
            _tank2->turnRight();
        } else if (pe_intf::Viewer::getKeyState('t') == 0) {
            _tank2->barrelRotLeft(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('u') == 0) {
            _tank2->barrelRotRight(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('r') == 0) {
            _tank2->barrelRotUp(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('f') == 0) {
            _tank2->barrelRotDown(_world.getDt());
        } else if (pe_intf::Viewer::getKeyState('b') == 0) {
            _tank2->brake();
        } else {
            _tank2->idle();
        }

        // shoot
        if (pe_intf::Viewer::getKeyState(',') == 2) {
            _tank1->shoot(&_world, 50, 20, 0.3, 5);
        }
        if (pe_intf::Viewer::getKeyState('v') == 2) {
            _tank2->shoot(&_world, 50, 20, 0.3, 5);
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
        /* This function creates a building with a ceiling and four walls */

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

        pe_phys_object::RigidBody* wall;

        // ceiling
        wall = createBoxFracturableObject(
                pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y - wall_thickness / 2, 0)),
                pe::Vector3(size.x - wall_thickness * 2, wall_thickness, size.z - wall_thickness * 2), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        _world.addRigidBody(wall);

        // front wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y / 2,
                                                                                                   size.z / 2 -
                                                                                                   wall_thickness /
                                                                                                   2)),
                                          pe::Vector3(size.x - wall_thickness * 2, size.y, wall_thickness), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        _world.addRigidBody(wall);

        // back wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y / 2,
                                                                                                   -size.z / 2 +
                                                                                                   wall_thickness /
                                                                                                   2)),
                                          pe::Vector3(size.x - wall_thickness * 2, size.y, wall_thickness), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        _world.addRigidBody(wall);

        // left wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(
                                                  size.x / 2 - wall_thickness / 2, size.y / 2, 0)),
                                          pe::Vector3(wall_thickness, size.y, size.z), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        _world.addRigidBody(wall);

        // right wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(
                                                  -size.x / 2 + wall_thickness / 2, size.y / 2, 0)),
                                          pe::Vector3(wall_thickness, size.y, size.z), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        _world.addRigidBody(wall);
    }

    void createStairs(const pe::Vector3& size, const pe::Vector3& pos, pe::Real stair_num, pe::Real height, int dir) {
        /* This function creates a stair */

    }

    void createUrbanLayout() {
        /* This function creates a 160x96 urban layout */

        pe_phys_object::RigidBody* block;

        // block [0-4,0]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, -1, 0)),
                                   pe::Vector3(80, 2, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [0-1,1]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-24, 2, -16)),
                                   pe::Vector3(32, 8, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [3-4,1]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(24, 2, -16)),
                                   pe::Vector3(32, 8, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [0-4,2]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, 2, -32)),
                                   pe::Vector3(80, 8, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [0,3-4]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-32, 3.5, -56)),
                                   pe::Vector3(16, 11, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [1-2,4]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-8, 5, -64)),
                                   pe::Vector3(32, 14, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [2-3,3]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(8, 0.5, -48)),
                                   pe::Vector3(32, 5, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [3,4]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(16, 2, -64)),
                                   pe::Vector3(16, 8, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [4,3-5]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(32, 2, -64)),
                                   pe::Vector3(16, 8, 48), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [0,5]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-32, 3.5, -80)),
                                   pe::Vector3(16, 11, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [2,5]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, 3.5, -80)),
                                   pe::Vector3(16, 11, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [0-1,6-7]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-24, 2, -104)),
                                   pe::Vector3(32, 8, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [2,7]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, 3.5, -112)),
                                   pe::Vector3(16, 11, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [0-2,8-9]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-16, 2, -136)),
                                   pe::Vector3(48, 8, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [3-4,6-9]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(24, 5, -104)),
                                   pe::Vector3(32, 14, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [3,8]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(16, 6.5, -128)),
                                   pe::Vector3(16, 17, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [3-4,9]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(24, 8, -144)),
                                   pe::Vector3(32, 20, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(TankSimulator, 0.016, 10000000)
