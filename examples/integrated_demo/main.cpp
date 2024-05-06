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
        _tank1->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 5, 0)));
        _tank1->init(&_world);

        // tank 2
        _tank2 = new pe_phys_vehicle::TankTemplate();
        pe::Matrix3 mat;
        mat.setRotation(pe::Vector3(0, 1, 0), PE_PI);
        _tank2->setTransform(pe::Transform(mat, pe::Vector3(0, 5, -160)));
        _tank2->init(&_world);

        // create the urban layout
        createUrbanLayout();
    }

    void step() override {
        /* Called every frame to update the physics world */

        // set the color of fragments
        for (auto rb : _world.getRigidBodiesToAdd()) {
            rb->setTag("color:0.8,0.8,0.3");
        }

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

    void createBuilding(const pe::Vector3& size, const pe::Vector3& pos, pe::Real wall_thickness, const std::string& color) {
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
        wall->setTag("color:" + color);
        _world.addRigidBody(wall);

        // front wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y / 2,
                                                                                                   size.z / 2 -
                                                                                                   wall_thickness /
                                                                                                   2)),
                                          pe::Vector3(size.x - wall_thickness * 2, size.y, wall_thickness), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        wall->setTag("color:" + color);
        _world.addRigidBody(wall);

        // back wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(0, size.y / 2,
                                                                                                   -size.z / 2 +
                                                                                                   wall_thickness /
                                                                                                   2)),
                                          pe::Vector3(size.x - wall_thickness * 2, size.y, wall_thickness), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        wall->setTag("color:" + color);
        _world.addRigidBody(wall);

        // left wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(
                                                  size.x / 2 - wall_thickness / 2, size.y / 2, 0)),
                                          pe::Vector3(wall_thickness, size.y, size.z), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        wall->setTag("color:" + color);
        _world.addRigidBody(wall);

        // right wall
        wall = createBoxFracturableObject(pe::Transform(pe::Matrix3::identity(), pos + pe::Vector3(
                                                  -size.x / 2 + wall_thickness / 2, size.y / 2, 0)),
                                          pe::Vector3(wall_thickness, size.y, size.z), 8, 1);
        wall->setKinematic(true);
        wall->addCollisionCallback(callback);
        wall->setTag("color:" + color);
        _world.addRigidBody(wall);
    }

    void createStairs(const pe::Vector3& size, const pe::Vector3& pos, pe::Real stair_num, pe::Real height, int dir) {
        /* This function creates a stair */

        pe_phys_object::RigidBody* stair;

        // base
        stair = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pos + pe::Vector3(0, size.y / 2, 0)),
                                   size, 8);
        stair->setKinematic(true);
        _world.addRigidBody(stair);

        // stairs
        pe::Real stair_width = (dir < 2 ? size.z : size.x) / pe::Real(stair_num + 1);
        pe::Real stair_height = height / pe::Real(stair_num);
        for (int i = 1; i <= stair_num; i++) {
            pe::Real size_x = dir < 2 ? size.x : size.x - stair_width * i;
            pe::Real size_z = dir < 2 ? size.z - stair_width * i : size.z;
            pe::Real pos_x = (dir >= 2) * (stair_width * i / 2) * (dir == 2 ? 1 : -1);
            pe::Real pos_y = size.y + stair_height * (i - 0.5);
            pe::Real pos_z = (dir < 2) * (stair_width * i / 2) * (dir == 0 ? 1 : -1);
            stair = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                     pos + pe::Vector3(pos_x, pos_y, pos_z)),
                                       pe::Vector3(size_x, stair_height, size_z), 8);
            stair->setKinematic(true);
            _world.addRigidBody(stair);
        }
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
        createBuilding(pe::Vector3(22, 4, 10), pe::Vector3(-24, 6, -16), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(22, 4, 10), pe::Vector3(-24, 10, -16), 1.0, "0.78,0.78,0.28");

        // block [2,1]
        createStairs(pe::Vector3(16, 2, 16), pe::Vector3(0, -2, -16),
                     25, 6, 1);
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(16, 6, -16), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(16, 10, -16), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(32, 6, -16), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(32, 10, -16), 1.0, "0.78,0.78,0.28");

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
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(-32, 6, -32), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(32, 6, -32), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(32, 10, -32), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(32, 14, -32), 1.0, "0.8,0.8,0.3");

        // block [0,3-5]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-32, 3.5, -64)),
                                   pe::Vector3(16, 11, 48), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(9, 4, 9), pe::Vector3(-32, 9, -48), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(9, 4, 9), pe::Vector3(-32, 13, -48), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(-32, 9, -64), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(-32, 13, -64), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(-32, 17, -64), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(-32, 9, -80), 1.0, "0.8,0.8,0.3");

        // block [1,3]
        createStairs(pe::Vector3(16, 8, 16), pe::Vector3(-16, -2, -48),
                     25, 6, 1);

        // block [1-2,4]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-8, 5, -64)),
                                   pe::Vector3(32, 14, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(12, 4, 12), pe::Vector3(0, 12, -64), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(12, 4, 12), pe::Vector3(0, 16, -64), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(12, 4, 12), pe::Vector3(0, 20, -64), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(12, 4, 12), pe::Vector3(0, 24, -64), 1.0, "0.78,0.78,0.28");

        // block [2-3,3]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(8, 0.5, -48)),
                                   pe::Vector3(32, 5, 16), 8);
        block->setKinematic(true);
        block->setIgnoreCollision(true);
        block->setTag("color:0.3,0.3,0.8");
        _world.addRigidBody(block);
        createStairs(pe::Vector3(12, 0.2, 8), pe::Vector3(16, 6, -52),
                     2, 0.4, 0);
        createStairs(pe::Vector3(12, 0.2, 8), pe::Vector3(16, 6, -44),
                     2, 0.4, 1);

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
        createBuilding(pe::Vector3(9, 4, 30), pe::Vector3(32, 6, -64), 1.0, "0.8,0.8,0.3");

        // block [1,5]
        createStairs(pe::Vector3(16, 8, 16), pe::Vector3(-16, -2, -80),
                     25, 6, 0);

        // block [2,5]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, 3.5, -80)),
                                   pe::Vector3(16, 11, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [3,5]
        createStairs(pe::Vector3(16, 8, 16), pe::Vector3(16, -2, -80),
                     25, 6, 1);

        // block [0-1,6-7]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-24, 2, -104)),
                                   pe::Vector3(32, 8, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(9, 4, 24), pe::Vector3(-32, 6, -104), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(9, 4, 24), pe::Vector3(-32, 10, -104), 1.0, "0.78,0.78,0.28");

        // block [2,6]
        createStairs(pe::Vector3(16, 8, 16), pe::Vector3(0, -2, -96),
                     25, 6, 2);
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(0, 9, -80), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(0, 13, -80), 1.0, "0.78,0.78,0.28");

        // block [2,7]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, 3.5, -112)),
                                   pe::Vector3(16, 11, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(0, 9, -112), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(8, 4, 8), pe::Vector3(0, 13, -112), 1.0, "0.78,0.78,0.28");


        // block [0-1,8-9]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(-24, 2, -136)),
                                   pe::Vector3(32, 8, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(10, 4, 24), pe::Vector3(-32, 6, -136), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 24), pe::Vector3(-32, 10, -136), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(-16, 6, -144), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(-16, 10, -144), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(-16, 14, -144), 1.0, "0.8,0.8,0.3");

        // block [2,8]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, 2, -128)),
                                   pe::Vector3(16, 8, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);

        // block [2,9]
        createStairs(pe::Vector3(16, 2, 16), pe::Vector3(0, -2, -144),
                     25, 6, 0);

        // block [3-4,6-7]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(24, 5, -104)),
                                   pe::Vector3(32, 14, 32), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(16, 12, -112), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(16, 16, -112), 1.0, "0.78,0.78,0.28");
        createBuilding(pe::Vector3(10, 4, 10), pe::Vector3(16, 20, -112), 1.0, "0.8,0.8,0.3");

        // block [3,8]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(16, 6.5, -128)),
                                   pe::Vector3(16, 17, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(9, 4, 9), pe::Vector3(16, 15, -128), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(9, 4, 9), pe::Vector3(16, 19, -128), 1.0, "0.78,0.78,0.28");

        // block [4,8]
        createStairs(pe::Vector3(16, 14, 16), pe::Vector3(32, -2, -128),
                     25, 6, 1);

        // block [3-4,9]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(24, 8, -144)),
                                   pe::Vector3(32, 20, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
        createBuilding(pe::Vector3(22, 4, 10), pe::Vector3(24, 18, -144), 1.0, "0.8,0.8,0.3");
        createBuilding(pe::Vector3(22, 4, 10), pe::Vector3(24, 22, -144), 1.0, "0.78,0.78,0.28");

        // block [0-4,10]
        block = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(),
                                                 pe::Vector3(0, -1, -160)),
                                   pe::Vector3(80, 2, 16), 8);
        block->setKinematic(true);
        _world.addRigidBody(block);
    }
};

// Simulator class, Delta time, Max frame
PE_SIM_MAIN(TankSimulator, 0.016, 10000000)
