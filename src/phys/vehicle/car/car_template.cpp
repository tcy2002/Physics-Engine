#include "car_template.h"
#include "phys/shape/box_shape.h"

#define PE_CAR_SUS_OFFSET pe::Real(0.15)
#define PE_CAR_WHEEL_MARGIN pe::Real(0.025)

namespace pe_phys_vehicle {

    void CarTemplate::initBody(pe_intf::World* dw) {
        body = new pe_phys_object::RigidBody();
        auto shape_b = new pe_phys_shape::BoxShape(pe::Vector3(
                _bodyWidth, _bodyHeight, _bodyLength));
        body->setCollisionShape(shape_b);
        body->setTransform(_transform);
        body->setMass(_bodyMass);
        auto c = this;
        body->addCollisionCallback([dw, c](pe_phys_object::RigidBody* self, pe_phys_object::RigidBody* other,
                                        const pe::Vector3& pos, const pe::Vector3& nor, const pe::Vector3& vel) {
            if (other->getTag() == "bullet") {
                c->setActive(false, dw);
                dw->removeRigidBody(other);
            }
        });
        dw->addRigidBody(body);

        cabin = new pe_phys_object::RigidBody();
        cabin->addIgnoreCollisionId(body->getGlobalId());
        auto shape_t = new pe_phys_shape::BoxShape(pe::Vector3(
                _cabinWidth, _cabinHeight, _cabinLength));
        cabin->setCollisionShape(shape_t);
        cabin->setMass(_cabinMass);
        cabin->addIgnoreCollisionId(body->getGlobalId());
        dw->addRigidBody(cabin);
        cabinTrl = pe::Vector3(0, (_bodyHeight + _cabinHeight) / 2, _bodyLength / 20);
    }

    void CarTemplate::initVehicle(pe_intf::World* dw) {
#   if PE_USE_CONTACT_VEHICLE
        ContactVehicle::VehicleTuning m_tuning;
        vehicle = new ContactVehicle(m_tuning, body, dw);
#   else
        RaycastVehicle::VehicleTuning m_tuning;
        DefaultVehicleRaycaster* rayCaster = new DefaultVehicleRaycaster(dw);
        vehicle = new RaycastVehicle(m_tuning, body, rayCaster);
#   endif

        vehicle->setCoordinateSystem(0, 1, 2);
#   if !PE_USE_CONTACT_VEHICLE
        vehicle->addRaycastExcludeId(body->getGlobalId());
        vehicle->addRaycastExcludeId(cabin->getGlobalId());
#   endif
    }

    void CarTemplate::initWheels(pe_intf::World* dw) {
        pe::Real connectionHeight = -PE_CAR_SUS_OFFSET;
        pe::Vector3 connectionPointCS0;
        pe::Vector3 wheelDirectionCS0(0, -1, 0);
        pe::Vector3 wheelAxleCS(-1, 0, 0);
        pe::Real suspensionRestLength = _bodyHeight / 2;
#   if PE_USE_CONTACT_VEHICLE
        ContactVehicle::VehicleTuning m_tuning;
#   else
        RaycastVehicle::VehicleTuning m_tuning;
#   endif

        for (int i = 0; i < 2; i++) {
            connectionPointCS0 = pe::Vector3(-_bodyWidth / 2 - (R(0.5) * _wheelWidth),
                                             connectionHeight,
                                             (_bodyLength / 2 - _bodyLength * i) * (R(5.0) / 7));
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
                              suspensionRestLength, _wheelRadius,
                              m_tuning, false);
            connectionPointCS0 = pe::Vector3(_bodyWidth / 2 + (R(0.5) * _wheelWidth),
                                             connectionHeight,
                                             (_bodyLength / 2 - _bodyLength * i) * (R(5.0) / 7));
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
                              suspensionRestLength, _wheelRadius,
                              m_tuning, false);
        }

        for (int i = 0; i < 4; i++) {
            pe_phys_object::RigidBody* wheel = new pe_phys_object::RigidBody();
            wheel->addIgnoreCollisionId(body->getGlobalId());
#       if PE_USE_SPHERE_WHEEL
            auto shape = new pe_phys_shape::SphereShape(
                    vehicle->getWheelInfo(i).m_wheelsRadius + PE_CAR_WHEEL_MARGIN);
#       else
            auto shape = new pe_phys_shape::CylinderShape(
                    vehicle->getWheelInfo(i).m_wheelsRadius + PE_CAR_WHEEL_MARGIN,
                    _wheelWidth - PE_CAR_WHEEL_MARGIN * 2);
#       endif
            wheel->setCollisionShape(shape);
            wheel->setMass(_wheelMass);
            wheel->setTag("wheel");
            wheels.push_back(wheel);
            dw->addRigidBody(wheel);
            vehicle->getWheelInfo(i).m_clientInfo = wheel;
#       if !PE_USE_CONTACT_VEHICLE
            vehicle->addRaycastExcludeId(wheel->getGlobalId());
#       endif
        }

        for (int i = 0; i < 4; i++) {
            auto& wheel = vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = _suspensionStiffness;
            wheel.m_wheelsDampingRelaxation = _suspensionDamping;
            wheel.m_wheelsDampingCompression = _suspensionCompression;
            wheel.m_frictionSlip = _wheelFriction;
            wheel.m_rollInfluence = _wheelRollInfluence;
            wheel.m_rollDamping = _wheelRollDamping;
        }

        vehicle->resetSuspension();
        for (int i = 0; i < 4; i++) {
            //synchronize the wheels with the (interpolated) chassis world transform
            vehicle->updateWheelTransform(i);
        }

        updateWheelsTransform();
        updateCabinTransform();
    }

    void CarTemplate::updateCabinTransform() {
        auto& bodyTrans = body->getTransform();
        auto& bodyLinVel = body->getLinearVelocity();
        auto& bodyAngVel = body->getAngularVelocity();

        pe::Transform transTurret = bodyTrans * pe::Transform(pe::Matrix3::identity(), cabinTrl);
        cabin->setTransform(transTurret);
        cabin->setLinearVelocity(bodyLinVel);
        cabin->setAngularVelocity(bodyAngVel);
    }

    void CarTemplate::updateWheelsTransform() {
        static pe::Matrix3 wheelRot = pe::Matrix3::identity();
        static pe::Real theta = PE_PI / R(2.0);
        static bool init = false;
        if (!init) {
            init = true;
            wheelRot[0][0] = cos(theta);
            wheelRot[0][1] = -sin(theta);
            wheelRot[1][0] = sin(theta);
            wheelRot[1][1] = cos(theta);
        }

        for (int i = 0; i < (int)wheels.size(); i++) {
            auto& wi = vehicle->getWheelInfo(i);
            pe::Transform tr = wi.m_worldTransform;
            tr.setBasis(tr.getBasis() * wheelRot);
            wheels[i]->setTransform(tr);
            wheels[i]->setLinearVelocity(body->getLinearVelocity());
        }
    }

    void CarTemplate::setBrake(bool brake) {
        for (int i = 0; i < vehicle->getNumWheels(); i++) {
            vehicle->setBrake(brake ? brakeForce : 0, i);
        }
    }

    CarTemplate::CarTemplate():
            _transform(pe::Transform::identity()),
            _bodyWidth(R(2.3)),
            _bodyLength(R(7.)),
            _bodyHeight(R(1.0)),
            _bodyMass(R(30.0)),
            _cabinWidth(R(2.3)),
            _cabinHeight(R(0.8)),
            _cabinLength(R(3.5)),
            _cabinMass(R(1.)),
            _wheelRadius(R(0.55)),
            _wheelWidth(R(0.4)),
            _wheelFriction(R(0.9)),
            _wheelRollInfluence(R(0.1)),
            _wheelRollDamping(R(0.03)),
            _wheelMass(R(1.)),
            _suspensionStiffness(R(25.0)),
            _suspensionDamping(R(2.0)),
            _suspensionCompression(R(2.0)),
            _engineForce(R(50.)),
            _maxSpeed(R(50.0)),
            _maxRotSpeed(R(2.0)) {}

    void CarTemplate::init(pe_intf::World* dw) {
        forwardForce = _engineForce;
        backwardForce = _engineForce;
        brakeForce = _engineForce / 50;
        initBody(dw);
        initVehicle(dw);
        initWheels(dw);
    }

    void CarTemplate::advance(pe::Real step) {
        updateCabinTransform();
        updateWheelsTransform();
        vehicle->updateVehicle(step);
    }

    void CarTemplate::idle() {
        setBrake(false);
    }

    void CarTemplate::brake() {
        setBrake(true);
    }

    void CarTemplate::moveForward() {
        setBrake(false);
        pe::Vector3 force = getSpeedKmHour() < _maxSpeed ?
                vehicle->getForwardVector() * -forwardForce : pe::Vector3(0, 0, 0);
        pe::Vector3 forceUp = vehicle->getUpVector() * ((body->getMass() * R(1.1)));
        if (vehicle->getNumWheelsOnGround() == 0) {
            force = pe::Vector3(0, 0, 0);
            forceUp = pe::Vector3(0, 0, 0);
        }
        body->addForce(vehicle->getWheelTransformWS(0).getOrigin(), force + forceUp);
        body->addForce(vehicle->getWheelTransformWS(1).getOrigin(), force + forceUp);
        body->addForce(vehicle->getWheelTransformWS(vehicle->getNumWheels() - 1).getOrigin(),
                       force - forceUp);
        body->addForce(vehicle->getWheelTransformWS(vehicle->getNumWheels() - 2).getOrigin(),
                       force - forceUp);
    }

    void CarTemplate::moveBackward() {
        setBrake(false);
        pe::Vector3 force = getSpeedKmHour() > -_maxSpeed ?
                            vehicle->getForwardVector() * backwardForce : pe::Vector3(0, 0, 0);
        pe::Vector3 forceUp = vehicle->getUpVector() * (body->getMass() * R(1.0));
        if (vehicle->getNumWheelsOnGround() == 0) {
            force = pe::Vector3(0, 0, 0);
            forceUp = pe::Vector3(0, 0, 0);
        }
        body->addForce(vehicle->getWheelTransformWS(0).getOrigin(), force - forceUp);
        body->addForce(vehicle->getWheelTransformWS(1).getOrigin(), force - forceUp);
        body->addForce(vehicle->getWheelTransformWS(vehicle->getNumWheels() - 1).getOrigin(),
                       force + forceUp);
        body->addForce(vehicle->getWheelTransformWS(vehicle->getNumWheels() - 2).getOrigin(),
                       force + forceUp);
    }

    void CarTemplate::turnLeft() {
        setBrake(false);
        vehicle->setSteeringValue(R(0.6), 2);
        vehicle->setSteeringValue(R(0.6), 3);
    }

    void CarTemplate::turnRight() {
        setBrake(false);
        vehicle->setSteeringValue(R(-0.6), 2);
        vehicle->setSteeringValue(R(-0.6), 3);
    }

    void CarTemplate::turnStraight() {
        setBrake(false);
        vehicle->setSteeringValue(0, 2);
        vehicle->setSteeringValue(0, 3);
    }

    pe::Real CarTemplate::getSpeedKmHour() const {
        return -vehicle->getCurrentSpeedKmHour();
    }

    void CarTemplate::setActive(bool active, pe_intf::World* dw) {
        body->setKinematic(!active);
        if (!active) {
            body->setTag("color:0.9,0.1,0.1");
            dw->updateRigidBody(body);
            cabin->setTag("color:0.9,0.1,0.1");
            dw->updateRigidBody(cabin);
            for (auto wheel : wheels) {
                wheel->setTag("color:0.9,0.1,0.1");
                dw->updateRigidBody(wheel);
            }
        } else {
            body->setTag("");
            dw->updateRigidBody(body);
            cabin->setTag("");
            dw->updateRigidBody(cabin);
            for (auto wheel : wheels) {
                wheel->setTag("");
                dw->updateRigidBody(wheel);
            }
        }
    }

} // namespace pe_phys_vehicle