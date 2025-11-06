#include "interface/simulator.h"
#include "physics/constraint/constraint/ball_joint_constraint.h"
#include "physics/constraint/constraint/hinge_joint_constraint.h"
#include "physics/constraint/constraint/slider_joint_constraint.h"
#include "physics/constraint/constraint/six_dof_constraint.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"
#include "physics/shape/capsule_shape.h"
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

    pe_physics_object::RigidBody* _chassis = nullptr;
    pe_vehicle::VehicleBase* _vehicle = nullptr;
    pe_vehicle::TrackedVehicle* _tracked = nullptr;

    void init() override {
        /* Initialize the physics world here before running */

        // set gravity (in our physics world, we use the same right-hand coordinates as opengl,
        // namely, x: right, y: up, z: screen outward)
        _world.setGravity(pe::Vector3(0, PE_R(-9.8), 0));
        // _world.setSleepLinVel2Threshold(PE_R(0.01)); // linear velocity threshold for sleep
        // _world.setSleepAngVel2Threshold(PE_R(0.005)); // angular velocity threshold for sleep
        // _world.setSleepTimeThreshold(PE_R(2.0));     // sleep time threshold

        // add a ground
        auto ground = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                      pe::Vector3(100, 10, 100), 1000);
        ground->setKinematic(true);
        //ground->setFrictionCoeff(PE_R(1.0));
        _world.addRigidBody(ground); // a rigidbody must be added into the _world to perform physical effects

        // add a ceiling
        auto ceiling = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 55, 0)),
                                      pe::Vector3(100, 10, 100), 1000);
        ceiling->setKinematic(true);
        _world.addRigidBody(ceiling);

        // add a wall
        auto wall1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-55, 25, 0)),
                                      pe::Vector3(10, 60, 100), 1000);
        wall1->setKinematic(true);
        _world.addRigidBody(wall1);

        // add a wall
        auto wall2 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(55, 25, 0)),
                                      pe::Vector3(10, 60, 100), 1000);
        wall2->setKinematic(true);
        _world.addRigidBody(wall2);

        // add a wall
        auto wall3 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, -55)),
                                      pe::Vector3(100, 60, 10), 1000);
        wall3->setKinematic(true);
        _world.addRigidBody(wall3);

        // add a wall
        auto wall4 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 25, 55)),
                                      pe::Vector3(100, 60, 10), 1000);
        wall4->setKinematic(true);
        _world.addRigidBody(wall4);

        try {
            auto chassis = new pe_vehicle::Chassis();
            chassis->setBoxBase(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)),
                pe::Vector3(2, 1, 4), 1000);
            _chassis = chassis->getBasePart().body;

            auto engine = new pe_vehicle::Engine(6);
            std::fstream file("D:/ClionProjects/Physics-Engine-clean/Physics-Engine-84da628/examples/vehicle_demo/config/Engine.json");
            nlohmann::json j = nlohmann::json::parse(file);
            engine->loadConfigFromJson(j);

            auto tracked = new pe_vehicle::TrackedVehicle();
            tracked->setChassis(chassis);
            tracked->setEngine(engine);
            tracked->setWheelCountPerSide(4);
            tracked->setWheelWidth(PE_R(1.0));
            tracked->setWheelRegionLength(PE_R(4.5));
            tracked->setWheelRegionWidth(PE_R(3.0));
            tracked->setWheelRegionOffset(pe::Vector3(0, -1, 0));
            tracked->setWheelInfo(0, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(1, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(2, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(3, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            //tracked->setWheelInfo(4, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(4, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(5, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(6, pe_vehicle::AxleType::AT_REAR, false, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setWheelInfo(7, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            //tracked->setWheelInfo(9, pe_vehicle::AxleType::AT_REAR, true, PE_R(0.5), 50, PE_R(1.0), PE_R(0.5), PE_R(-0.5), PE_R(50000.0), PE_R(500.0));
            tracked->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 3.5, 0)));
            tracked->init(&_world);
            _tracked = tracked;
        } catch (const std::exception& e) {
            PE_LOG_ERROR << "Exception caught during vehicle creation: " << e.what() << PE_ENDL;
        }

        /*try {
            auto chassis = new pe_vehicle::Chassis();
            chassis->setBoxBase(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, 0)),
                pe::Vector3(2, 1, 4), 1000);
            _chassis = chassis->getBasePart().body;

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

         //// add a chasis
         //_chassis = createBoxRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::right(), 0), pe::Vector3(0, 3, 0)),
         //                              pe::Vector3(2, 1, 4), 50);
         //_world.addRigidBody(_chassis);
         //// _chassis->setKinematic(true);
        
         //// add six wheels
         //auto wheel1 = createCapsuleRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 2),
         //                                                              pe::Vector3(-1.5, 1.5, -1.5)),
         //                                            PE_R(0.5), PE_R(0.2), 10);
         //_world.addRigidBody(wheel1);
         //auto wheel2 = createCapsuleRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 2),
         //                                                              pe::Vector3(1.5, 1.5, -1.5)),
         //                                            PE_R(0.5), PE_R(0.2), 10);
         //_world.addRigidBody(wheel2);
         //auto wheel3 = createSphereRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 2),
         //                                                              pe::Vector3(-1.5, 1.5, 0)),
         //                                            PE_R(0.5), 10);
         //_world.addRigidBody(wheel3);
         //auto wheel4 = createSphereRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 2),
         //                                                              pe::Vector3(1.5, 1.5, 0)),
         //                                            PE_R(0.5), 10);
         //_world.addRigidBody(wheel4);
         //auto wheel5 = createSphereRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 2),
         //                                                              pe::Vector3(-1.5, 1.5, 1.5)),
         //                                            PE_R(0.5), 10);
         //_world.addRigidBody(wheel5);
         //auto wheel6 = createSphereRigidBody(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), PE_PI / 2),
         //                                                              pe::Vector3(1.5, 1.5, 1.5)),
         //                                            PE_R(0.5), 10);
         //_world.addRigidBody(wheel6);
        
         //wheel1->addIgnoreCollisionId(_chassis->getGlobalId());
         //wheel2->addIgnoreCollisionId(_chassis->getGlobalId());
         //wheel3->addIgnoreCollisionId(_chassis->getGlobalId());
         //wheel4->addIgnoreCollisionId(_chassis->getGlobalId());
         //wheel5->addIgnoreCollisionId(_chassis->getGlobalId());
         //wheel6->addIgnoreCollisionId(_chassis->getGlobalId());
        
         //// add constraints between chassis and wheels
         //// auto sus1 = new pe_physics_constraint::SixDofConstraint();
         //// sus1->setObjectA(_chassis);
         //// sus1->setObjectB(wheel1);
         //// sus1->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-1.5, -0.5, -1.5)));
         //// sus1->setFrameB(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::right(), -PE_PI / 2), pe::Vector3::zeros()));
         //// sus1->setXPosFixed(true);
         //// sus1->setYPosFixed(true);
         //// sus1->setZPosFixed(true);
         //// sus1->setXRotFixed(true);
         //// sus1->setYRotFixed(false);
         //// sus1->setZRotFixed(false);
         //// sus1->setYRotLimitType(pe_physics_constraint::ConstraintLimitType::CLT_LOWER_UPPER);
         //// sus1->setMinAngleY(PE_PI * PE_R(-3.0 / 4));
         //// sus1->setMaxAngleY(PE_PI * PE_R(-1.0 / 4));
         //// // sus1->setZRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_POSITION);
         //// // sus1->setTargetAngleZ(PE_PI * PE_R(-1.5 / 4));
         //// // sus1->setYRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
         //// // sus1->setTargetSpeedY(-1);
         //// _world.addConstraint(sus1);
        
         //auto sus1 = new pe_physics_constraint::SixDofConstraint();
         //sus1->setObjectA(_chassis);
         //sus1->setObjectB(wheel1);
         //sus1->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(-1.5, -0.5, -1.5)));
         //sus1->setFrameB(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::forward(), -PE_PI / 2), pe::Vector3::zeros()));
         //sus1->setXPosFixed(true);
         //sus1->setYPosFixed(true);
         //sus1->setZPosFixed(true);
         //sus1->setXRotFixed(false);
         //sus1->setYRotFixed(true);
         //sus1->setZRotFixed(true);
         //// sus1->setZRotLimitType(pe_physics_constraint::ConstraintLimitType::CLT_LOWER_UPPER);
         //// sus1->setMinAngleZ(PE_PI * PE_R(-3.0 / 4));
         //// sus1->setMaxAngleZ(PE_PI * PE_R(-1.0 / 4));
         //// sus1->setZRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_POSITION);
         //// sus1->setTargetAngleZ(PE_PI * PE_R(-1.5 / 4));
         //// sus1->setXRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
         //// sus1->setTargetSpeedX(1);
         //_world.addConstraint(sus1);
        
         //auto sus2 = new pe_physics_constraint::SixDofConstraint();
         //sus2->setObjectA(_chassis);
         //sus2->setObjectB(wheel2);
         //sus2->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(1.5, -0.5, -1.5)));
         //sus2->setFrameB(pe::Transform(pe::Matrix3::fromRotation(pe::Vector3::right(), -PE_PI / 2), pe::Vector3::zeros()));
         //sus2->setXPosFixed(true);
         //sus2->setYPosFixed(true);
         //sus2->setZPosFixed(true);
         //sus2->setXRotFixed(true);
         //sus2->setYRotFixed(false);
         //sus2->setZRotFixed(false);
         //// sus2->setYRotLimitType(pe_physics_constraint::ConstraintLimitType::CLT_LOWER_UPPER);
         //// sus2->setMinAngleY(PE_PI * PE_R(-3.0 / 4));
         //// sus2->setMaxAngleY(PE_PI * PE_R(-1.0 / 4));
         //sus2->setYRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_POSITION);
         //sus2->setTargetAngleY(PE_PI * PE_R(-2.0 / 4));
         //// sus2->setZRotMotorType(pe_physics_constraint::ConstraintMotorType::CMT_VELOCITY);
         //// sus2->setTargetSpeedZ(-1);
         //_world.addConstraint(sus2);
         //auto sus3 = new pe_physics_constraint::HingeJointConstraint();
         //sus3->setObjectA(_chassis);
         //sus3->setObjectB(wheel3);
         //sus3->setAnchorA(pe::Vector3(-1.5, -0.5, 0));
         //sus3->setAnchorB(pe::Vector3(0, 0, 0));
         //sus3->setAxisA(-pe::Vector3::right());
         //sus3->setAxisB(pe::Vector3::up());
         //_world.addConstraint(sus3);
         //auto sus4 = new pe_physics_constraint::HingeJointConstraint();
         //sus4->setObjectA(_chassis);
         //sus4->setObjectB(wheel4);
         //sus4->setAnchorA(pe::Vector3(1.5, -0.5, 0));
         //sus4->setAnchorB(pe::Vector3(0, 0, 0));
         //sus4->setAxisA(-pe::Vector3::right());
         //sus4->setAxisB(pe::Vector3::up());
         //_world.addConstraint(sus4);
         //auto sus5 = new pe_physics_constraint::HingeJointConstraint();
         //sus5->setObjectA(_chassis);
         //sus5->setObjectB(wheel5);
         //sus5->setAnchorA(pe::Vector3(-1.5, -0.5, 1.5));
         //sus5->setAnchorB(pe::Vector3(0, 0, 0));
         //sus5->setAxisA(-pe::Vector3::right());
         //sus5->setAxisB(pe::Vector3::up());
         //_world.addConstraint(sus5);
         //auto sus6 = new pe_physics_constraint::HingeJointConstraint();
         //sus6->setObjectA(_chassis);
         //sus6->setObjectB(wheel6);
         //sus6->setAnchorA(pe::Vector3(1.5, -0.5, 1.5));
         //sus6->setAnchorB(pe::Vector3(0, 0, 0));
         //sus6->setAxisA(-pe::Vector3::right());
         //sus6->setAxisB(pe::Vector3::up());
         //_world.addConstraint(sus6);
        
         //// add left track
         //createTrack(_chassis, pe::Transform(pe::Matrix3::identity(), pe::Vector3(-1.5, 2.5, 0)),
         //            PE_R(3.0), PE_R(0.52), PE_R(0.8), PE_R(10), PE_R(0.1),
         //            PE_R(0.8), 30, 1);
        
         //// add right track
         //createTrack(_chassis, pe::Transform(pe::Matrix3::identity(), pe::Vector3(1.5, 2.5, 0)),
         //            PE_R(3.0), PE_R(0.52), PE_R(0.8), PE_R(10), PE_R(0.1),
         //            PE_R(0.8), 30, 1);
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

            /*const pe::Transform vehi_trans = _vehicle->getTransform();
            pe::Vector3 vehi_backward_horizon = vehi_trans.getAxis(0);
            vehi_backward_horizon.y = PE_R(0.0);
            vehi_backward_horizon.normalize();
            const pe::Vector3 cam_pos = vehi_trans.getOrigin() + vehi_backward_horizon * 10 + pe::Vector3(0, 5, 0);
            pe::Real cam_yaw = pe::Vector3::forward().angle(vehi_backward_horizon);
            if (pe::Vector3::forward().cross(vehi_backward_horizon).dot(pe::Vector3::up()) < 0) cam_yaw = -cam_yaw;
            pe_interface::Viewer::setCamera(cam_pos, cam_yaw, PE_PI / PE_R(8.0));*/
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
                _tracked->setBrake(true);
            }
            else {
                _tracked->setBrake(false);
            }

            if (pe_interface::Viewer::getKeyState('i') == 0) {
                std::cout << 1 << std::endl;
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
        }
    }

protected:
    static pe::Transform getTrackSegmentTransform(pe::Real length, pe::Real radius, pe::Real segment_count, pe::Real current_length) {
        if (current_length < PE_PI * radius) {
            const pe::Real angle = current_length / radius;
            const pe::Real x = -radius * PE_SIN(angle) - length / PE_R(2.0);
            const pe::Real y = radius * PE_COS(angle);
            return pe::Transform(pe::Matrix3::fromRotation(-pe::Vector3::right(), angle), pe::Vector3(0, y, x));
        }

        current_length -= PE_PI * radius;
        if (current_length < length) {
            const pe::Real x = -length / PE_R(2.0) + current_length;
            return pe::Transform(pe::Matrix3::fromRotation(-pe::Vector3::right(), PE_PI), pe::Vector3(0, -radius, x));
        }

        current_length -= length;
        if (current_length < PE_PI * radius) {
            const pe::Real angle = current_length / radius;
            const pe::Real x = radius * PE_SIN(angle) + length / PE_R(2.0);
            const pe::Real y = -radius * PE_COS(angle);
            return pe::Transform(pe::Matrix3::fromRotation(-pe::Vector3::right(), PE_PI + angle), pe::Vector3(0, y, x));
        }

        current_length -= PE_PI * radius;
        const pe::Real x = length / PE_R(2.0) - current_length;
        return pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, radius, x));
    }

    void createTrack(pe_physics_object::RigidBody* chassis,
                     const pe::Transform& trans, pe::Real length, pe::Real radius, pe::Real width, pe::Real segment_mass,
                     pe::Real segment_height, pe::Real segment_length_ratio, int segment_count, pe::Real tighten_ratio) {
        const pe::Real track_length = PE_R(2.0) * PE_PI * radius + PE_R(2.0) * length;
        const pe::Vector3 segment_size = pe::Vector3(width, segment_height, track_length * segment_length_ratio / PE_R(segment_count));
        const pe::Real segment_gap = track_length / PE_R(segment_count);

        pe::Array<pe_physics_object::RigidBody*> segments;
        for (int i = 0; i < segment_count; ++i) {
            const pe::Real current_length = track_length * PE_R(i) / PE_R(segment_count);
            const pe::Transform trans_seg = getTrackSegmentTransform(length, radius, segment_count, current_length);
            auto segment = createBoxRigidBody(trans * trans_seg, segment_size, segment_mass);
            segment->setFrictionCoeff(PE_R(1.0));
            _world.addRigidBody(segment);
            segments.push_back(segment);
        }

        const pe::Real half_segment_dist = segment_gap * tighten_ratio / PE_R(2.0);
        const pe::Real half_segment_width = width / PE_R(2.0);
        for (int i = 0; i < segment_count; ++i) {
            // auto joint = new pe_physics_constraint::SixDofConstraint();
            // joint->setObjectA(segments[i]);
            // joint->setObjectB(segments[(i + 1) % segment_count]);
            // joint->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, -half_segment_dist)));
            // joint->setFrameB(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0, half_segment_dist)));
            // joint->setXPosFixed(true);
            // joint->setYPosFixed(true);
            // joint->setZPosFixed(true);
            // joint->setYRotFixed(true);
            // joint->setZRotFixed(true);
            // _world.addConstraint(joint);
            // auto joint = new pe_physics_constraint::HingeJointConstraint();
            // joint->setObjectA(segments[i]);
            // joint->setObjectB(segments[(i + 1) % segment_count]);
            // joint->setAnchorA(pe::Vector3(0, 0, -half_segment_dist));
            // joint->setAnchorB(pe::Vector3(0, 0, half_segment_dist));
            // joint->setAxisA(pe::Vector3::right());
            // joint->setAxisB(pe::Vector3::right());
            // _world.addConstraint(joint);
            auto joint1 = new pe_physics_constraint::BallJointConstraint();
            joint1->setObjectA(segments[i]);
            joint1->setObjectB(segments[(i + 1) % segment_count]);
            joint1->setAnchorA(pe::Vector3(half_segment_width, 0, -half_segment_dist));
            joint1->setAnchorB(pe::Vector3(half_segment_width, 0, half_segment_dist));
            _world.addConstraint(joint1);
            auto joint2 = new pe_physics_constraint::BallJointConstraint();
            joint2->setObjectA(segments[i]);
            joint2->setObjectB(segments[(i + 1) % segment_count]);
            joint2->setAnchorA(pe::Vector3(-half_segment_width, 0, -half_segment_dist));
            joint2->setAnchorB(pe::Vector3(-half_segment_width, 0, half_segment_dist));
            _world.addConstraint(joint2);
        }

        pe::Real dist_to_chassis_x = (trans.getOrigin() - chassis->getTransform().getOrigin())
                                     .dot(chassis->getTransform().getBasis().getColumn(0));
        for (int i = 0; i < segment_count; i += segment_count / 6) {
            auto joint = new pe_physics_constraint::SixDofConstraint();
            joint->setObjectA(chassis);
            joint->setObjectB(segments[i]);
            joint->setFrameA(pe::Transform(pe::Matrix3::identity(), pe::Vector3(dist_to_chassis_x, 0, 0)));
            joint->setFrameB(pe::Transform::identity());
            joint->setXPosFixed(true);
            _world.addConstraint(joint);
        }
    }

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
};

// Simulator class, Target frame rate
PE_CUSTOM_MAIN(VehicleSimulator, 100)
