#include "tank_template.h"
#include "phys/shape/box_shape.h"

#define PE_TANK_SUS_OFFSET R(0.15)
#define PE_TANK_WHEEL_MARGIN R(0.025)
#define PE_TANK_TRACK_MARGIN R(0.01)

namespace pe_phys_vehicle {

    void TankTemplate::initBody(pe_intf::World* dw) {
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

        turret = new pe_phys_object::RigidBody();
        turret->addIgnoreCollisionId(body->getGlobalId());
        auto shape_t = new pe_phys_shape::BoxShape(pe::Vector3(
                _turretWidth, _turretHeight, _turretLength));
        turret->setCollisionShape(shape_t);
        turret->setMass(_turretMass);
        turret->addIgnoreCollisionId(body->getGlobalId());
        dw->addRigidBody(turret);
        turretTrl = pe::Vector3(0, (_bodyHeight + _turretHeight) / 2, _bodyLength / 20);

        barrel = new pe_phys_object::RigidBody();
        barrel->addIgnoreCollisionId(turret->getGlobalId());
        auto shape_r = new pe_phys_shape::BoxShape(pe::Vector3(
                _barrelRadius * 2, _barrelRadius * 2, _barrelLength));
        barrel->setCollisionShape(shape_r);
        barrel->setMass(_barrelMass);
        barrel->addIgnoreCollisionId(turret->getGlobalId());
        dw->addRigidBody(barrel);
        barrelTrl = pe::Vector3(0, 0, -(_turretLength + _barrelLength) / 2);

        updateTurretAndBarrelTransform(); // update transforms of turret and barrel
    }

    void TankTemplate::initVehicle(pe_intf::World* dw) {
#   if PE_TANK_USE_CONTACT_VEHICLE
        ContactVehicle::VehicleTuning m_tuning;
        vehicle = new ContactVehicle(m_tuning, body, dw);
#   else
        RaycastVehicle::VehicleTuning m_tuning;
        DefaultVehicleRaycaster* rayCaster = new DefaultVehicleRaycaster(dw);
        vehicle = new RaycastVehicle(m_tuning, body, rayCaster);
#   endif

        vehicle->setCoordinateSystem(0, 1, 2);
#   if !PE_TANK_USE_CONTACT_VEHICLE
        vehicle->addRaycastExcludeId(body->getGlobalId());
        vehicle->addRaycastExcludeId(turret->getGlobalId());
        vehicle->addRaycastExcludeId(barrel->getGlobalId());
#   endif
    }

    void TankTemplate::initWheels(pe_intf::World* dw) {
        pe::Real connectionHeight = -PE_TANK_SUS_OFFSET;
        pe::Real connectionHeightEnds = _bodyHeight / 2 - PE_TANK_SUS_OFFSET;
        pe::Vector3 connectionPointCS0;
        pe::Vector3 wheelDirectionCS0(0, -1, 0);
        pe::Vector3 wheelAxleCS(-1, 0, 0);
        pe::Real suspensionRestLength = _bodyHeight / 2;
        pe::Real gap = _bodyLength / R(_wheelNum / 2 - 1); //NOLINT
#   if PE_TANK_USE_CONTACT_VEHICLE
        ContactVehicle::VehicleTuning m_tuning;
#   else
        RaycastVehicle::VehicleTuning m_tuning;
#   endif

        for (int i = 0; i < _wheelNum / 2; i++) {
            pe::Real wheelRadius = i == 0 || i == _wheelNum / 2 - 1 ? _powerWheelRadius : _drivenWheelRadius;
            connectionPointCS0 = pe::Vector3(-_bodyWidth / 2 - (R(0.5) * _wheelWidth),
                                             (i == 0 || i == _wheelNum / 2 - 1) ?
                                             connectionHeightEnds : connectionHeight,
                                             gap * i - _bodyLength / 2);
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
                              suspensionRestLength, wheelRadius,
                              m_tuning, false);
            connectionPointCS0 = pe::Vector3(_bodyWidth / 2 + (R(0.5) * _wheelWidth),
                                             (i == 0 || i == _wheelNum / 2 - 1) ?
                                             connectionHeightEnds : connectionHeight,
                                             gap * i - _bodyLength / 2);
            vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
                              suspensionRestLength, wheelRadius,
                              m_tuning, false);
        }

        for (int i = 0; i < _wheelNum; i++) {
            pe_phys_object::RigidBody* wheel = new pe_phys_object::RigidBody();
            wheel->addIgnoreCollisionId(body->getGlobalId());
            wheel->addIgnoreCollisionId(turret->getGlobalId());
            wheel->addIgnoreCollisionId(barrel->getGlobalId());
#       if PE_TANK_USE_SPHERE_WHEEL
            auto shape = new pe_phys_shape::SphereShape(
                    vehicle->getWheelInfo(i).m_wheelsRadius + PE_TANK_WHEEL_MARGIN);
#       else
            auto shape = new pe_phys_shape::CylinderShape(
                    vehicle->getWheelInfo(i).m_wheelsRadius + PE_TANK_WHEEL_MARGIN,
                    _wheelWidth - PE_TANK_WHEEL_MARGIN * 2);
#       endif
            wheel->setCollisionShape(shape);
            wheel->setMass(_wheelMass);
            wheel->setTag("wheel");
            wheels.push_back(wheel);
            dw->addRigidBody(wheel);
            vehicle->getWheelInfo(i).m_clientInfo = wheel;
#       if !PE_TANK_USE_CONTACT_VEHICLE
            vehicle->addRaycastExcludeId(wheel->getGlobalId());
#       endif
        }

        for (int i = 0; i < _wheelNum; i++) {
            auto& wheel = vehicle->getWheelInfo(i);
            wheel.m_suspensionStiffness = _suspensionStiffness;
            wheel.m_wheelsDampingRelaxation = _suspensionDamping;
            wheel.m_wheelsDampingCompression = _suspensionCompression;
            wheel.m_frictionSlip = _wheelFriction;
            wheel.m_rollInfluence = _wheelRollInfluence;
            wheel.m_rollDamping = _wheelRollDamping;
        }

        vehicle->resetSuspension();
        for (int i = 0; i < _wheelNum; i++) {
            //synchronize the wheels with the (interpolated) chassis world transform
            vehicle->updateWheelTransform(i);
        }

        updateWheelsTransform();
    }

    void TankTemplate::initTracks(pe_intf::World* dw) {
        (void)dw;

        trackHoldingPoints.resize(_wheelNum * 2);
        trackHoldingWheels.resize(_wheelNum * 2);
        updateTrackHoldings();

        // all the track segments are the same
        for (int i = 0; i < _trackSegmentNum * 2; i++) {
            pe_phys_object::RigidBody* rb = new pe_phys_object::RigidBody();
            // current track is totally fake
            rb->setCollisionShape(new pe_phys_shape::BoxShape(pe::Vector3(
                    _wheelWidth, _trackThickness, _trackSegmentWidth)));
            trackSegments.push_back(rb);
            rb->setIgnoreCollision(true);
            dw->addRigidBody(rb);
        }

        updateTracksTransform();
    }

    void TankTemplate::updateTurretAndBarrelTransform() {
        auto& bodyTrans = body->getTransform();
        auto& bodyLinVel = body->getLinearVelocity();
        auto& bodyAngVel = body->getAngularVelocity();

        pe::Matrix3 rotTurret;
        rotTurret.setRotation(pe::Vector3::up(), turretAngle);
        pe::Transform transTurret = bodyTrans * pe::Transform(rotTurret, turretTrl);
        turret->setTransform(transTurret);
        turret->setLinearVelocity(bodyLinVel);
        turret->setAngularVelocity(bodyAngVel);

        auto rotBarrel = pe::Matrix3::identity();
        rotBarrel.setRotation(pe::Vector3::right(), barrelAngle);
        auto barrelTrlUp = pe::Vector3(0, _barrelLength / 2 * std::sin(barrelAngle),
                                         -_barrelLength / 2 * (std::cos(barrelAngle) - 1));
        pe::Transform transBarrel = transTurret * pe::Transform(rotBarrel, barrelTrl + barrelTrlUp);
        barrel->setTransform(transBarrel);
        barrel->setLinearVelocity(bodyLinVel);
        barrel->setAngularVelocity(bodyAngVel);
    }

    void TankTemplate::updateWheelsTransform() {
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

    void TankTemplate::updateTracksTransform() {
        updateOneSideTrackTransform(0);
        updateOneSideTrackTransform(1);
    }

    void TankTemplate::updateOneSideTrackTransform(int side) {
        int startIdx = _wheelNum * side;
        int segStartIdx = _trackSegmentNum * side;
        int segIdx = 0;
        pe::Real length = 0;
        pe::Real offset = side == 0 ? trackOffsetLeft : trackOffsetRight;
        pe::Real segmentWidth = side == 0 ? trackSegmentWidthLeft : trackSegmentWidthRight;
        pe::Vector3 leftVec(-1, 0, 0);
        pe::Vector3 upVec(0, 1, 0);
        auto& bodyTrans = body->getTransform();

        int i = 0, loop = 0;
        while (loop++ < _wheelNum) {
            auto& p1 = trackHoldingPoints[startIdx + i];
            auto& p2 = trackHoldingPoints[startIdx + i + 1];
            auto& p3 = trackHoldingPoints[startIdx + (i + 2) % _wheelNum];
            auto& wheel = trackHoldingWheels[startIdx + i + 1];

            auto dir = (p1 - p2).normalized();
            auto vec = (p2 - wheel).normalized();
            auto alpha = leftVec.dot(vec.cross(upVec)) < 0 ? acos(vec.y) : -acos(vec.y);
            pe::Matrix3 rotMat;
            rotMat.setRotation(leftVec, alpha);
            pe::Real restLength = (p1 - p2).norm() - offset;
            if (restLength < 0) {
                offset = -restLength;
            }
            else {
                offset = 0;
                restLength -= length;
                while (restLength > 0) {
                    auto center = p2 + dir * restLength;
                    pe::Transform trans = bodyTrans * pe::Transform(rotMat, center);
                    trackSegments[segStartIdx + segIdx]->setTransform(trans);
                    restLength -= segmentWidth;
                    if (++segIdx >= _trackSegmentNum) return;
                }
                length = -restLength;
            }

            pe::Real radius = (p2 - wheel).norm();
            auto vec1 = (p2 - wheel).normalized();
            auto vec2 = (p3 - wheel).normalized();
            pe::Real angle = acos(clampReal(-1, 1, vec1.dot(vec2))) - offset / radius;
            pe::Real deltaAngle = segmentWidth / radius;
            if (angle < 0) {
                offset = -angle * radius;
            }
            else {
                offset = 0;
                angle -= length / radius;
                while (angle > 0) {
                    auto vec3 = rotateVector3(vec2, leftVec, -angle);
                    auto theta = leftVec.dot(vec3.cross(upVec)) < 0 ? acos(vec3.y) : -acos(vec3.y);
                    pe::Matrix3 rot;
                    rot.setRotation(leftVec, theta);
                    auto center = wheel + vec3 * radius;
                    pe::Transform trans = bodyTrans * pe::Transform(rot, center);
                    trackSegments[segStartIdx + segIdx]->setTransform(trans);
                    angle -= deltaAngle;
                    if (++segIdx >= _trackSegmentNum) return;
                }
                length = -angle * radius;
            }

            i = (i + 2) % _wheelNum;
        }
    }

    void TankTemplate::updateTrackHoldings() {
        updateOneSideTrackHoldings(0);
        updateOneSideTrackHoldings(1);
    }

    void TankTemplate::updateOneSideTrackHoldings(int side) {
        int startIdx = _wheelNum * side;
        pe::Vector3 rightVec = pe::Vector3(1, 0, 0);
        auto& bodyTrans = body->getTransform();
        pe::Real newLength = 0;

        for (int i = side; i < _wheelNum; i += 2) {
            int j = (i + 2) % _wheelNum;
            auto p1 = bodyTrans.inverseTransform(vehicle->getWheelTransformWS(i).getOrigin());
            auto p2 = bodyTrans.inverseTransform(vehicle->getWheelTransformWS(j).getOrigin());
            pe::Real r1 = vehicle->getWheelInfo(i).m_wheelsRadius - PE_TANK_TRACK_MARGIN;
            pe::Real r2 = vehicle->getWheelInfo(j).m_wheelsRadius - PE_TANK_TRACK_MARGIN;

            auto forwardVec = (p1 - p2).normalized();
            auto downVec = (p1 - p2).cross(rightVec).normalized();
            pe::Real t1 = (r2 - r1) / (p1 - p2).norm();
            pe::Real t2 = sqrt(clampReal(0, 1, 1 - t1 * t1));
            auto dir = forwardVec * t1 + downVec * t2;

            auto u1 = p1 + dir * r1;
            auto u2 = p2 + dir * r2;
            trackHoldingWheels[startIdx + i - side] = p1;
            trackHoldingPoints[startIdx + i - side] = u1;
            trackHoldingWheels[startIdx + i - side + 1] = p2;
            trackHoldingPoints[startIdx + i - side + 1] = u2;
            newLength += (u1 - u2).norm();
        }

        for (int i = 0; i < _wheelNum; i += 2) {
            auto& p1 = trackHoldingPoints[startIdx + i];
            auto& p2 = trackHoldingPoints[startIdx + (i + _wheelNum - 1) % _wheelNum];
            auto& center = trackHoldingWheels[startIdx + i];

            auto dir1 = (p1 - center).normalized();
            auto dir2 = (p2 - center).normalized();
            pe::Real length = (p1 - center).norm() * acos(clampReal(-1, 1, dir1.dot(dir2)));
            newLength += length;
        }

        (side == 0 ? trackLengthLeft : trackLengthRight) = newLength;
        (side == 0 ? trackSegmentWidthLeft : trackSegmentWidthRight) = newLength / _trackSegmentNum;
    }

    void TankTemplate::uniformTrackVelocity() {
        uniformTrackVelocityOneSide(0);
        uniformTrackVelocityOneSide(1);
    }

    void TankTemplate::uniformTrackVelocityOneSide(int side) {
        int num = (int)wheels.size();
        pe::Real avgDeltaDist = 0;
        for (int i = side + 2; i < num - 2; i += 2) {
            avgDeltaDist += vehicle->getWheelInfo(i).m_deltaRotation * vehicle->getWheelInfo(i).m_wheelsRadius;
        }
        avgDeltaDist /= R(num / 2 - 2); //NOLINT

        auto& offset = (side == 0 ? trackOffsetLeft : trackOffsetRight);
        auto length = (side == 0 ? trackLengthLeft : trackLengthRight);
        offset -= avgDeltaDist;
        if (offset < 0) offset += length;
        if (offset > length) offset -= length;

        for (int i = side; i < num; i += 2) {
            pe::Real oldDeltaAngle = vehicle->getWheelInfo(i).m_deltaRotation;
            pe::Real newDeltaAngle = avgDeltaDist / vehicle->getWheelInfo(i).m_wheelsRadius;
            vehicle->getWheelInfo(i).m_deltaRotation = newDeltaAngle;
            vehicle->getWheelInfo(i).m_rotation += (newDeltaAngle - oldDeltaAngle);
        }
    }

    void TankTemplate::setBrake(bool brake) {
        for (int i = 0; i < vehicle->getNumWheels(); i++) {
            vehicle->setBrake(brake ? brakeForce : 0, i);
        }
    }

    TankTemplate::TankTemplate():
            _transform(pe::Transform::identity()),
            _bodyWidth(R(2.3)),
            _bodyLength(R(7.)),
            _bodyHeight(R(1.0)),
            _bodyMass(R(30.0)),
            _turretWidth(R(2.7)),
            _turretHeight(R(0.8)),
            _turretLength(R(3.5)),
            _turretMass(R(1.)),
            _turretRotSpeed(R(1.0)),
            _turretMaxAngle(PE_PI / R(2.4)),
            _barrelRadius(R(0.128)),
            _barrelLength(R(4.2)),
            _barrelMass(R(1.)),
            _barrelRotSpeed(R(1.0)),
            _barrelMaxAngle(PE_PI / R(6.)),
            _wheelNum(16),
            _powerWheelRadius(R(0.3)),
            _drivenWheelRadius(R(0.4)),
            _wheelWidth(R(0.6)),
            _wheelFriction(R(0.9)),
            _wheelRollInfluence(R(0.1)),
            _wheelRollDamping(R(0.03)),
            _wheelMass(R(1.)),
            _trackThickness(R(0.1)),
            _trackSegmentNum(80),
            _trackSegmentWidth(R(0.18)),
            _suspensionStiffness(R(12.0)),
            _suspensionDamping(R(2.0)),
            _suspensionCompression(R(2.0)),
            _engineForce(R(50.)),
            _maxSpeed(R(50.0)),
            _maxRotSpeed(R(2.0)) {}

    void TankTemplate::init(pe_intf::World* dw) {
        forwardForce = _engineForce;
        backwardForce = _engineForce;
        turnForce = _engineForce;
        brakeForce = _engineForce / 50;
        initBody(dw);
        initVehicle(dw);
        initWheels(dw);
        initTracks(dw);
    }

    void TankTemplate::advance(pe::Real step) {
        updateTurretAndBarrelTransform();
        updateWheelsTransform();
        uniformTrackVelocity();
        updateTrackHoldings();
        updateTracksTransform();
        vehicle->updateVehicle(step);
    }

    void TankTemplate::shoot(pe_intf::World *dw, pe::Real speed, pe::Real mass, pe::Real radius, pe::Real lifeTime) {
        // calculate the position and direction of the bullet
        auto pos = barrel->getTransform().getOrigin();
        auto dir = barrel->getTransform().getBasis() * pe::Vector3(0, 0, -1);
        pos += dir * _barrelLength / 2;
        auto vel = dir * speed;

        // create the bullet
        auto bullet = new pe_phys_object::RigidBody();
        auto shape = new pe_phys_shape::SphereShape(radius);
        bullet->setCollisionShape(shape);
        bullet->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
        bullet->setMass(mass);
        bullet->setLinearVelocity(vel);
        bullet->setLifeTime(lifeTime);
        bullet->setTag("bullet");
        dw->addRigidBody(bullet);

        // add recoil force
        auto recoil = dir * (-mass * speed * dw->getDt());
        auto recoilPos = barrel->getTransform().getOrigin() - body->getTransform().getOrigin();
        body->applyImpulse(recoilPos, recoil);
    }

    void TankTemplate::idle() {
        setBrake(false);
    }

    void TankTemplate::brake() {
        setBrake(true);
    }

    void TankTemplate::moveForward() {
        setBrake(false);
        pe::Vector3 force = getSpeedKmHour() < _maxSpeed ?
                vehicle->getForwardVector() * -forwardForce : pe::Vector3(0, 0, 0);
        pe::Vector3 forceUp = vehicle->getUpVector() * ((body->getMass() * R(0.4)));
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

    void TankTemplate::moveBackward() {
        setBrake(false);
        pe::Vector3 force = getSpeedKmHour() > -_maxSpeed ?
                            vehicle->getForwardVector() * backwardForce : pe::Vector3(0, 0, 0);
        pe::Vector3 forceUp = vehicle->getUpVector() * (body->getMass() * R(0.3));
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

    void TankTemplate::turnLeft() {
        setBrake(false);
        if (body->getAngularVelocity().dot(vehicle->getUpVector()) > _maxRotSpeed) return;
        pe::Vector3 torque = body->getTransform().getBasis() * vehicle->getUpVector() * turnForce * 10;
        body->addTorque(torque);
    }

    void TankTemplate::turnRight() {
        setBrake(false);
        if (body->getAngularVelocity().dot(vehicle->getUpVector()) < -_maxRotSpeed) return;
        pe::Vector3 torque = body->getTransform().getBasis() * vehicle->getUpVector() * -turnForce * 10;
        body->addTorque(torque);
    }

    void TankTemplate::barrelRotLeft(pe::Real step) {
        if (turretAngle < _turretMaxAngle && !body->isKinematic()) {
            turretAngle += step * _turretRotSpeed;
        }
    }

    void TankTemplate::barrelRotRight(pe::Real step) {
        if (turretAngle > -_turretMaxAngle && !body->isKinematic()) {
            turretAngle -= step * _turretRotSpeed;
        }
    }

    void TankTemplate::barrelRotUp(pe::Real step) {
        if (barrelAngle < _barrelMaxAngle && !body->isKinematic()) {
            barrelAngle += step * _barrelRotSpeed;
        }
    }

    void TankTemplate::barrelRotDown(pe::Real step) {
        if (barrelAngle > 0 && !body->isKinematic()) {
            barrelAngle -= step * _barrelRotSpeed;
        }
    }

    pe::Real TankTemplate::getSpeedKmHour() const {
        return -vehicle->getCurrentSpeedKmHour();
    }

    void TankTemplate::setActive(bool active, pe_intf::World* dw) {
        body->setKinematic(!active);
        if (!active) {
            body->setTag("color:0.9,0.1,0.1");
            dw->updateRigidBody(body);
            turret->setTag("color:0.9,0.1,0.1");
            dw->updateRigidBody(turret);
            barrel->setTag("color:0.9,0.1,0.1");
            dw->updateRigidBody(barrel);
            for (auto wheel : wheels) {
                wheel->setTag("color:0.9,0.1,0.1");
                dw->updateRigidBody(wheel);
            }
            for (auto track : trackSegments) {
                track->setTag("color:0.9,0.1,0.1");
                dw->updateRigidBody(track);
            }
        } else {
            body->setTag("");
            dw->updateRigidBody(body);
            turret->setTag("");
            dw->updateRigidBody(turret);
            barrel->setTag("");
            dw->updateRigidBody(barrel);
            for (auto wheel : wheels) {
                wheel->setTag("");
                dw->updateRigidBody(wheel);
            }
            for (auto track : trackSegments) {
                track->setTag("");
                dw->updateRigidBody(track);
            }
        }
    }

} // namespace pe_phys_vehicle