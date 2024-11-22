#include "intf/simulator.h"
#include "phys/vehicle/tank/tank_template.h"

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class TankSimulator : public pe_intf::Simulator {
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
        _world.setGravity(pe::Vector3(0, R(-9.8), 0));

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

        // add callback functions
        // callback function for wall collision
        static auto callback = [&](
                pe_phys_object::RigidBody* self, pe_phys_object::RigidBody* other,
                const pe::Vector3& pos, const pe::Vector3& nor, const pe::Vector3& vel) {
            if (other->getTag() != "bullet" || !self->isFracturable()) return;
            auto fb = (pe_phys_object::FracturableObject*)self;
            if (vel.norm() * other->getMass() < fb->getThreshold() * 800) return;
            pe_phys_fracture::FractureSource src;
            src.position = pos;
            src.type = pe_phys_fracture::FractureType::Sphere;
            src.intensity = pe::Vector3(R(1.5), R(1.5), R(1.5));
            _world.addFractureSource(src);
            _world.removeRigidBody(other);
        };

        for (auto rb : _world.getRigidBodies()) {
            if (rb->getTag().find("building") != std::string::npos) {
                rb->addCollisionCallback(callback);
            }
        }
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
            _tank1->shoot(&_world, 50, 20, pe::Real(0.3), 5);
        }
        if (pe_intf::Viewer::getKeyState('v') == 2) {
            _tank2->shoot(&_world, 50, 20, pe::Real(0.3), 5);
        }
    }
};

// Simulator class, Target frame rate
PE_CONFIG_CUSTOM_MAIN(TankSimulator, 100)
