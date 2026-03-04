#include "interface/simulator.h"
#include "physics/constraint/constraint/ball_joint_constraint.h"
#include "physics/constraint/constraint/hinge_joint_constraint.h"
#include "physics/constraint/constraint/slider_joint_constraint.h"
#include "physics/constraint/constraint/six_dof_constraint.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/capsule_shape.h"
#include "physics/shape/concave_mesh_shape.h"
#include "vehicle/component/chassis.h"
#include "vehicle/vehicle_base.h"
#include "vehicle/tracked_vehicle/tracked_vehicle.h"
#include "json/json.hpp"
#include "utils/logger.h"
#include <fstream>

// See SimpleViewer/include/opengl_viewer.h to learn the view control
// To turn off the viewer, set use_gui = false in init()
class VehicleSimulator : public pe_interface::Simulator {
public:
    VehicleSimulator() {}
    virtual ~VehicleSimulator() {}

    pe_vehicle::VehicleBase* _vehicle = nullptr;
    pe_vehicle::TrackedVehicle* _tracked = nullptr;

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        _world.setSleepLinVel2Threshold(PE_R(0.01)); // linear velocity threshold for sleep
        _world.setSleepAngVel2Threshold(PE_R(0.005)); // angular velocity threshold for sleep
        _world.setSleepTimeThreshold(PE_R(2.0));     // sleep time threshold

        //// add a ground
        //auto ground = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
        //                              pe::Vector3(100, 10, 100), 1000);
        //ground->setKinematic(true);
        ////ground->setFrictionCoeff(PE_R(1.0));
        //_world.addRigidBody(ground); // a rigidbody must be added into the _world to perform physical effects

        //// add a ceiling
        //auto ceiling = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 55, 0)),
        //                              pe::Vector3(100, 10, 100), 1000);
        //ceiling->setKinematic(true);
        //_world.addRigidBody(ceiling);

        //// add a wall
        //auto wall1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-55, 25, 0)),
        //                              pe::Vector3(10, 60, 100), 1000);
        //wall1->setKinematic(true);
        //_world.addRigidBody(wall1);

        //// add a wall
        //auto wall2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(55, 25, 0)),
        //                              pe::Vector3(10, 60, 100), 1000);
        //wall2->setKinematic(true);
        //_world.addRigidBody(wall2);

        //// add a wall
        //auto wall3 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, -55)),
        //                              pe::Vector3(100, 60, 10), 1000);
        //wall3->setKinematic(true);
        //_world.addRigidBody(wall3);

        //// add a wall
        //auto wall4 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, 55)),
        //                              pe::Vector3(100, 60, 10), 1000);
        //wall4->setKinematic(true);
        //_world.addRigidBody(wall4);

        const std::string path = "D:/ClionProjects/Physics-Engine/examples/terrain_car_demo/terrain.obj";
        auto rb1 = createConcaveRigidBody(path,
            pe::Transform(pe::Matrix3::identity(), pe::Vector3(-10, -1, -10)),
            10000, 0.1);
        rb1->setKinematic(true);
        _world.addRigidBody(rb1);

        try {
            auto chassis = new pe_vehicle::Chassis();
            int id0 = chassis->setBoxBase(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)),
                pe::Vector3(4, 1, 7), 1000);
            int id1 = chassis->addBoxPart(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1, 0.2)),
                pe::Vector3(4, 1, 5), 500);
            chassis->addSixDofLink(id0, id1,
                pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1, 0.2)), pe::Transform::identity(),
                true, true, true,
                true, true, true);

            auto engine = new pe_vehicle::Engine(6);
            std::fstream file("D:/ClionProjects/Physics-Engine-clean/Physics-Engine-84da628/examples/vehicle_demo/config/Engine.json");
            nlohmann::json j = nlohmann::json::parse(file);
            engine->loadConfigFromJson(j);

            auto tracked = new pe_vehicle::TrackedVehicle();
            tracked->setChassis(chassis);
            tracked->setEngine(engine);
            tracked->setWheelCountPerSide(6);
            tracked->setWheelWidth(PE_R(1.0));
            tracked->setWheelRegionLength(PE_R(5.5));
            tracked->setWheelRegionWidth(PE_R(4.0));
            tracked->setWheelRegionOffset(pe::Vector3(0, -1.5, 0));
            tracked->setWheelInfo(0, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.4), 50, PE_R(1.0), PE_R(0.5), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(1, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(2, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(3, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(4, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(5, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.4), 50, PE_R(1.0), PE_R(0.5), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(6, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.4), 50, PE_R(1.0), PE_R(0.5), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(7, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(8, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(9, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(10, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.2), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(11, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.4), 50, PE_R(1.0), PE_R(0.5), PE_R(0.0), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setTransform(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3(1, 2, 3), 0), pe::Vector3(0, 5, 0)));
            tracked->init(&_world);
            _tracked = tracked;
        } catch (const std::exception& e) {
            PE_LOG_ERROR << "Exception caught during vehicle creation: " << e.what() << PE_ENDL;
        }

        /*try {
            auto chassis = new pe_vehicle::Chassis();
            chassis->setBoxBase(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)),
                pe::Vector3(2, 1, 4), 1000);

            auto engine = new pe_vehicle::Engine(6);
            std::fstream file("D:/ClionProjects/Physics-Engine-clean/Physics-Engine-84da628/examples/vehicle_demo/config/Engine.json");
            nlohmann::json j = nlohmann::json::parse(file);
            engine->loadConfigFromJson(j);

            auto vehicle = new pe_vehicle::VehicleBase();
            vehicle->setChassis(chassis);
            vehicle->setEngine(engine);
            vehicle->setWheelCountPerSide(2);
            vehicle->setWheelRegionLength(PE_R(3.0));
            vehicle->setWheelRegionWidth(PE_R(3.0));
            vehicle->setWheelRegionOffset(pe::Vector3(0, -1, 0));
            vehicle->setWheelInfo(0, pe_vehicle::AxleType::AT_FRONT, true, PE_R(0.5), 50, PE_R(5.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            vehicle->setWheelInfo(1, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(5.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            vehicle->setWheelInfo(2, pe_vehicle::AxleType::AT_FRONT, true, PE_R(0.5), 50, PE_R(5.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            vehicle->setWheelInfo(3, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(5.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            vehicle->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 3.5, 0)));
            vehicle->init(&_world);
            _vehicle = vehicle;
        }
        catch (const std::exception& e) {
            PE_LOG_ERROR << "Exception caught during vehicle creation: " << e.what() << PE_ENDL;
        }*/
    }

    void step() override {
        if (_vehicle) {
            if (pe_interface::Viewer::getKeyState('b') == 0) {
                _vehicle->setGear(1);
            }
            else if (pe_interface::Viewer::getKeyState('n') == 0) {
                _vehicle->setGear(2);
            }
            else if (pe_interface::Viewer::getKeyState('m') == 0) {
                _vehicle->setGear(3);
            }
            else if (pe_interface::Viewer::getKeyState(',') == 0) {
                _vehicle->setGear(4);
            }
            else if (pe_interface::Viewer::getKeyState('.') == 0) {
                _vehicle->setGear(5);
            }
            else if (pe_interface::Viewer::getKeyState('/') == 0) {
                _vehicle->setGear(6);
            }

            if (pe_interface::Viewer::getKeyState('h') == 0 || pe_interface::Viewer::getKeyState(';') == 0) {
                _vehicle->setBrake(true);
            } else {
                _vehicle->setBrake(false);
            }

            if (pe_interface::Viewer::getKeyState('i') == 0) {
                if (_vehicle->getGear() <= 0) {
                    _vehicle->setGear(1);
                }
                _vehicle->setThrottle(PE_R(1.0));
            }
            else if (pe_interface::Viewer::getKeyState('k') == 0) {
                _vehicle->setGear(-1);
                _vehicle->setThrottle(PE_R(1.0));
            }
            else if (_vehicle) {
                _vehicle->setThrottle(PE_R(0.0));
            }

            if (pe_interface::Viewer::getKeyState('j') == 0) {
                _vehicle->setSteerAngle(0, PE_PI / PE_R(6));
                _vehicle->setSteerAngle(2, PE_PI / PE_R(6));
            }
            else if (pe_interface::Viewer::getKeyState('l') == 0) {
                _vehicle->setSteerAngle(0, -PE_PI / PE_R(6));
                _vehicle->setSteerAngle(2, -PE_PI / PE_R(6));
            }
            else if (_vehicle) {
                _vehicle->setSteerAngle(0, 0);
                _vehicle->setSteerAngle(2, 0);
            }

            _vehicle->step(_world.getDt());

            const pe::Transform vehi_trans = _vehicle->getTransform();
            pe::Vector3 vehi_backward_horizon = -vehi_trans.getAxis(0);
            vehi_backward_horizon.y = PE_R(0.0);
            vehi_backward_horizon.normalize();
            const pe::Vector3 cam_pos = vehi_trans.getOrigin() + vehi_backward_horizon * 10 + pe::Vector3(0, 5, 0);
            pe::Real cam_yaw = pe::Vector3::forward().angle(vehi_backward_horizon);
            if (pe::Vector3::forward().cross(vehi_backward_horizon).dot(pe::Vector3::up()) < 0) cam_yaw = -cam_yaw;
            pe_interface::Viewer::setCamera(cam_pos, cam_yaw, PE_PI / PE_R(8.0));
        }

        if (_tracked) {
            if (pe_interface::Viewer::getKeyState('b') == 0) {
                _tracked->setGear(1);
            }
            else if (pe_interface::Viewer::getKeyState('n') == 0) {
                _tracked->setGear(2);
            }
            else if (pe_interface::Viewer::getKeyState('m') == 0) {
                _tracked->setGear(3);
            }
            else if (pe_interface::Viewer::getKeyState(',') == 0) {
                _tracked->setGear(4);
            }
            else if (pe_interface::Viewer::getKeyState('.') == 0) {
                _tracked->setGear(5);
            }
            else if (pe_interface::Viewer::getKeyState('/') == 0) {
                _tracked->setGear(6);
            }

            if (pe_interface::Viewer::getKeyState('h') == 0 || pe_interface::Viewer::getKeyState(';') == 0) {
                _tracked->setBrake(PE_R(0.05));
            }
            else {
                _tracked->setBrake(0);
            }

            if (pe_interface::Viewer::getKeyState('i') == 0) {
                if (_tracked->getGear() <= 0) {
                    _tracked->setGear(1);
                }
                _tracked->setLeftThrottle(PE_R(1.0));
                _tracked->setRightThrottle(PE_R(1.0));
            }
            else if (pe_interface::Viewer::getKeyState('k') == 0) {
                _tracked->setGear(-1);
                _tracked->setLeftThrottle(PE_R(1.0));
                _tracked->setRightThrottle(PE_R(1.0));
            }
            else if (pe_interface::Viewer::getKeyState('j') == 0) {
                _tracked->setGear(-1);
                _tracked->setLeftThrottle(PE_R(1.0));
                _tracked->setRightThrottle(PE_R(-1.0));
            }
            else if (pe_interface::Viewer::getKeyState('l') == 0) {
                _tracked->setGear(-1);
                _tracked->setLeftThrottle(PE_R(-1.0));
                _tracked->setRightThrottle(PE_R(1.0));
            }
            else {
                _tracked->setLeftThrottle(PE_R(0.0));
                _tracked->setRightThrottle(PE_R(0.0));
            }

            _tracked->step(_world.getDt());

            const pe::Transform vehi_trans = _tracked->getTransform();
            pe::Vector3 vehi_backward_horizon = vehi_trans.getAxis(2);
            pe::Vector3 vehi_leftward_horizon = -vehi_trans.getAxis(0);
            vehi_backward_horizon.y = PE_R(0.0);
            vehi_backward_horizon.normalize();
            vehi_leftward_horizon.y = PE_R(0.0);
            vehi_leftward_horizon.normalize();
            const pe::Vector3 cam_pos = vehi_trans.getOrigin() + vehi_backward_horizon * 15 + vehi_leftward_horizon * 4 + pe::Vector3(0, 5, 0);
            pe::Real cam_yaw = (-pe::Vector3::forward()).angle(vehi_backward_horizon);
            if ((-pe::Vector3::forward()).cross(vehi_backward_horizon).dot(pe::Vector3::up()) < 0) cam_yaw = -cam_yaw;
            pe_interface::Viewer::setCamera(cam_pos, cam_yaw + PE_PI, PE_PI / PE_R(8.0));
        }

        static int frame = 0;
        frame++;
        std::cout << frame << std::endl;
        //if (frame == 1000) {
        //    _world.setSleepLinVel2Threshold(PE_R(0.0)); // linear velocity threshold for sleep
        //    _world.setSleepAngVel2Threshold(PE_R(0.0)); // angular velocity threshold for sleep
        //    auto rbs = _world.getRigidBodies();
        //    for (auto rb : rbs) {
        //        if (rb->getGlobalId() == 9) continue;
        //        if (rb->getGlobalId() == 17) _world.removeRigidBody(rb);
        //        //if (rb->getGlobalId() == 18) _world.removeRigidBody(rb);
        //        //if (rb->getGlobalId() == 19) _world.removeRigidBody(rb);
        //        if (rb->getGlobalId() == 20) _world.removeRigidBody(rb);
        //        rb->setKinematic(true);
        //    }
        //}
        //if (frame >= 1100 && frame <= 1200) {
        //    auto rbs = _world.getRigidBodies();
        //    for (auto rb : rbs) {
        //        if (rb->getGlobalId() == 9) {
        //            rb->setAngularVelocity(pe::Vector3::zeros());
        //        }
        //    }
        //}
    }

protected:
    static pe_physics_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
        /* This function creates a box-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::BoxShape(size);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5)); // friction coefficient
        rb->setRestitutionCoeff(PE_R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
        rb->setAngularDamping(PE_R(0.8)); // angular damping parameter (slows down the rotation speed)
        return rb;
    }

    static pe_physics_object::RigidBody* createSphereRigidBody(const pe::Transform& trans,
                                                            pe::Real radius, pe::Real mass) {
        /* This function creates a sphere-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::SphereShape(radius);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }

    static pe_physics_object::RigidBody* createCapsuleRigidBody(const pe::Transform& trans,
                                                              pe::Real radius, pe::Real height, pe::Real mass) {
        /* This function creates a cylinder-shaped rigidbody */

        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::CapsuleShape(radius, height);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setAngularDamping(PE_R(0.8));
        return rb;
    }

    static pe_physics_object::RigidBody* createConcaveRigidBody(const std::string& obj_path, const pe::Transform& trans, pe::Real mass, pe::Real size) {
        static pe::Mesh mesh;
        if (mesh.empty())
            pe::Mesh::loadFromObj(obj_path, mesh, pe::Vector3(size, size, size));
        auto rb = new pe_physics_object::RigidBody();
        rb->setMass(mass);
        auto shape = new pe_physics_shape::ConcaveMeshShape();
        shape->setMeshPath(obj_path);
        shape->setScale(pe::Vector3(3, 3, 3));
        shape->setMesh(mesh);
        rb->setCollisionShape(shape);
        rb->setTransform(trans);
        rb->setFrictionCoeff(PE_R(0.5));
        rb->setRestitutionCoeff(PE_R(0.5));
        rb->setKinematic(true);
        return rb;
    }
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(VehicleSimulator, 100)
