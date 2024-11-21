#include "contact_vehicle.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "phys/shape/sphere_shape.h"

//#define ROLLING_INFLUENCE_FIX

namespace pe_phys_vehicle {

    ContactVehicle::VehicleTuning::VehicleTuning():
            m_suspensionStiffness(R(5.88)),
            m_suspensionCompression(R(0.83)),
            m_suspensionDamping(R(0.88)),
            m_maxSuspensionTravelCm(R(500.)),
            m_frictionSlip(R(10.5)),
            m_maxSuspensionForce(R(1000.)) {}

    ContactVehicle::ContactVehicle(const VehicleTuning& tuning, pe_phys_object::RigidBody* chassis,
                                   pe_intf::World* world) {
        m_chassisBody = chassis;
        m_world = world;
        m_indexRightAxis = 0;
        m_indexUpAxis = 2;
        m_indexForwardAxis = 1;
        m_currentVehicleSpeedKmHour = 0;
    }

    pe_phys_object::RigidBody& ContactVehicle::getFixedBody() {
        static pe_phys_object::RigidBody s_fixed;
        s_fixed.setCollisionShape(new pe_phys_shape::SphereShape(0.0));
        s_fixed.setTransform(pe::Transform::identity());
        s_fixed.setMass(0.0);
        return s_fixed;
    }

    //
    // basically most of the code is general for 2- or 4-wheel vehicles, but some of it needs to be reviewed
    //
    ContactWheelInfo& ContactVehicle::addWheel(const pe::Vector3& connectionPointCS,
                                               const pe::Vector3& wheelDirectionCS0,
                                               const pe::Vector3& wheelAxleCS, pe::Real suspensionRestLength,
                                               pe::Real wheelRadius, const VehicleTuning& tuning, bool isFrontWheel) {
        ContactWheelInfoConstructionInfo ci;

        ci.m_chassisConnectionCS = connectionPointCS;
        ci.m_wheelDirectionCS = wheelDirectionCS0;
        ci.m_wheelAxleCS = wheelAxleCS;
        ci.m_suspensionRestLength = suspensionRestLength;
        ci.m_wheelRadius = wheelRadius;
        ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
        ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
        ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
        ci.m_frictionSlip = tuning.m_frictionSlip;
        ci.m_bIsFrontWheel = isFrontWheel;
        ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
        ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

        m_wheelInfo.push_back(ContactWheelInfo(ci));

        ContactWheelInfo& wheel = m_wheelInfo[getNumWheels() - 1];

        updateWheelTransformsWS(wheel);
        updateWheelTransform(getNumWheels() - 1);
        return wheel;
    }

    const pe::Transform& ContactVehicle::getWheelTransformWS(int wheelIndex) const {
        const ContactWheelInfo& wheel = m_wheelInfo[wheelIndex];
        return wheel.m_worldTransform;
    }

    void ContactVehicle::updateWheelTransform(int wheelIndex) {
        ContactWheelInfo& wheel = m_wheelInfo[wheelIndex];
        updateWheelTransformsWS(wheel);
        pe::Vector3 up = -wheel.m_contactInfo.m_wheelDirectionWS;
        const pe::Vector3& right = wheel.m_contactInfo.m_wheelAxleWS;
        pe::Vector3 fwd = up.cross(right);
        fwd.normalize();

        //rotate around steering over de wheelAxleWS
        pe::Real steering = wheel.m_steering;

        pe::Matrix3 steeringMat;
        steeringMat.setRotation(up, steering);

        pe::Matrix3 rotatingMat;
        rotatingMat.setRotation(right, -wheel.m_rotation);

        pe::Matrix3 basis2;
        basis2[0][m_indexRightAxis] = -right[0];
        basis2[1][m_indexRightAxis] = -right[1];
        basis2[2][m_indexRightAxis] = -right[2];

        basis2[0][m_indexUpAxis] = up[0];
        basis2[1][m_indexUpAxis] = up[1];
        basis2[2][m_indexUpAxis] = up[2];

        basis2[0][m_indexForwardAxis] = fwd[0];
        basis2[1][m_indexForwardAxis] = fwd[1];
        basis2[2][m_indexForwardAxis] = fwd[2];

        wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
        wheel.m_worldTransform.setOrigin(
                wheel.m_contactInfo.m_hardPointWS + wheel.m_contactInfo.m_wheelDirectionWS *
                                                    wheel.m_contactInfo.m_suspensionLength);
    }

    void ContactVehicle::resetSuspension() {
        int i;
        for (i = 0; i < m_wheelInfo.size(); i++) {
            ContactWheelInfo& wheel = m_wheelInfo[i];
            wheel.m_contactInfo.m_suspensionLength = wheel.getSuspensionRestLength();
            wheel.m_suspensionRelativeVelocity = R(0.0);

            wheel.m_contactInfo.m_contactNormalWS = -wheel.m_contactInfo.m_wheelDirectionWS;
            wheel.m_clippedInvContactDotSuspension = R(1.0);
        }
    }

    void ContactVehicle::updateWheelTransformsWS(ContactWheelInfo& wheel) const {
        wheel.m_contactInfo.m_isInContact = false;

        pe::Transform chassisTrans = getChassisWorldTransform();

        wheel.m_contactInfo.m_hardPointWS = chassisTrans * wheel.m_chassisConnectionPointCS;
        wheel.m_contactInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
        wheel.m_contactInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
        wheel.m_contactInfo.m_wheelCenterWS = wheel.m_contactInfo.m_hardPointWS +
                                              wheel.m_contactInfo.m_wheelDirectionWS *
                                              wheel.m_contactInfo.m_suspensionLength;
    }

    pe::Real ContactVehicle::contactResolve(ContactWheelInfo& wheel) {
        updateWheelTransformsWS(wheel);

        // get the closest point from the contact info
        pe_phys_collision::ContactPoint* closest_point = nullptr;
        pe::Real maxDepth = PE_REAL_MIN;
        uint32_t rb_id = ((pe_phys_object::RigidBody*)wheel.m_clientInfo)->getGlobalId();
        for (auto cr : m_world->_contact_results) {
            if (cr->getObjectA()->getGlobalId() == rb_id || cr->getObjectB()->getGlobalId() == rb_id) {
                for (int i = 0; i < PE_MIN(1, cr->getPointSize()); i++) {
                    auto& cp = cr->getContactPoint(i);
                    pe::Real dist = -cp.getDistance();
                    if (dist > maxDepth) {
                        maxDepth = dist;
                        closest_point = &cp;
                    }
                }
            }
        }

        wheel.m_contactInfo.m_groundObject = 0;

        if (closest_point) {
            // if the contact point is on the side of the wheel, ignore it
            if (PE_APPROX_EQUAL(closest_point->getWorldNormal().cross(wheel.m_contactInfo.m_wheelAxleWS).norm2(), 0)) {
                return 0;
            }
            if ((closest_point->getWorldPos() - wheel.m_contactInfo.m_wheelCenterWS)
                .dot(wheel.m_contactInfo.m_wheelDirectionWS) < 0) {
                return 0;
            }

            wheel.m_contactInfo.m_isInContact = true;
            wheel.m_contactInfo.m_contactPointWS = closest_point->getWorldPos();
            wheel.m_contactInfo.m_contactNormalWS = closest_point->getWorldNormal();
            if (wheel.m_contactInfo.m_contactNormalWS
                .dot(wheel.m_contactInfo.m_contactPointWS - wheel.m_contactInfo.m_wheelCenterWS) > 0) {
                wheel.m_contactInfo.m_contactNormalWS = -wheel.m_contactInfo.m_contactNormalWS;
            }

            wheel.m_contactInfo.m_groundObject = &getFixedBody();  ///@todo for driving on dynamic/movable objects!;

            // calculate suspension length
            pe::Real fwdExtent = (wheel.m_contactInfo.m_contactPointWS - wheel.m_contactInfo.m_hardPointWS)
                    .dot(getForwardVector());
            pe::Real whlExtent =
                    std::sqrt(PE_MAX(0, wheel.m_wheelsRadius * wheel.m_wheelsRadius - fwdExtent * fwdExtent));
            pe::Real susExtent = (wheel.m_contactInfo.m_contactPointWS - wheel.m_contactInfo.m_hardPointWS)
                    .dot(wheel.m_contactInfo.m_wheelDirectionWS);
            if (wheel.m_contactInfo.m_contactNormalWS.dot(wheel.m_contactInfo.m_wheelDirectionWS) < pe::Real(0.0)) {
                wheel.m_contactInfo.m_suspensionLength = susExtent - whlExtent;
            } else {
                wheel.m_contactInfo.m_suspensionLength = susExtent + whlExtent;
            }
            wheel.m_contactInfo.m_suspensionLength =
                    PE_MIN(wheel.getSuspensionRestLength(), wheel.m_contactInfo.m_suspensionLength);

            //clamp on max suspension travel
            pe::Real minSuspensionLength = wheel.getSuspensionRestLength() -
                    wheel.m_maxSuspensionTravelCm * R(0.01);
            pe::Real maxSuspensionLength = wheel.getSuspensionRestLength() +
                    wheel.m_maxSuspensionTravelCm * R(0.01);
            if (wheel.m_contactInfo.m_suspensionLength < minSuspensionLength) {
                wheel.m_contactInfo.m_suspensionLength = minSuspensionLength;
            }
            if (wheel.m_contactInfo.m_suspensionLength > maxSuspensionLength) {
                wheel.m_contactInfo.m_suspensionLength = maxSuspensionLength;
            }

            pe::Real denominator = wheel.m_contactInfo.m_contactNormalWS.dot(wheel.m_contactInfo.m_wheelDirectionWS);

            pe::Vector3 chassis_velocity_at_contactPoint;
            pe::Vector3 relPos = wheel.m_contactInfo.m_contactPointWS - getRigidBody()->getTransform().getOrigin();

            chassis_velocity_at_contactPoint = getRigidBody()->getLinearVelocityAtLocalPoint(relPos);

            pe::Real projVel = wheel.m_contactInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

            if (denominator >= R(-0.1)) {
                wheel.m_contactInfo.m_contactDepth = R(0.0);
                wheel.m_suspensionRelativeVelocity = R(0.0);
                wheel.m_clippedInvContactDotSuspension = R(1.0) / R(0.1);
            } else {
                pe::Real inv = R(-1.) / denominator;
                wheel.m_contactInfo.m_contactDepth = maxDepth * inv;
                wheel.m_suspensionRelativeVelocity = projVel * inv;
                wheel.m_clippedInvContactDotSuspension = inv;
            }
        } else {
            //put wheel info as in rest position
            if (wheel.m_contactInfo.m_suspensionLength < wheel.getSuspensionRestLength()) {
                wheel.m_contactInfo.m_suspensionLength += wheel.m_contactInfo.m_suspensionDelta;
                wheel.m_contactInfo.m_suspensionDelta += R(0.003);
            } else {
                wheel.m_contactInfo.m_suspensionLength = wheel.getSuspensionRestLength();
                wheel.m_contactInfo.m_suspensionDelta = R(0.003);
            }
            wheel.m_suspensionRelativeVelocity = R(0.0);
            wheel.m_contactInfo.m_contactNormalWS = -wheel.m_contactInfo.m_wheelDirectionWS;
            wheel.m_clippedInvContactDotSuspension = R(1.0);
        }

        return maxDepth;
    }

    const pe::Transform& ContactVehicle::getChassisWorldTransform() const {
        return getRigidBody()->getTransform();
    }

    void ContactVehicle::updateVehicle(pe::Real step) {
        {
            for (int i = 0; i < getNumWheels(); i++) {
                updateWheelTransform(i);
            }
        }

        m_currentVehicleSpeedKmHour = R(3.6) * getRigidBody()->getLinearVelocity().dot(getForwardVector());

        const pe::Transform& chassisTrans = getChassisWorldTransform();

        //
        // simulate suspension
        //

        int i;
        for (i = 0; i < m_wheelInfo.size(); i++) {
            contactResolve(m_wheelInfo[i]);
        }

        updateSuspension(step);

        for (i = 0; i < m_wheelInfo.size(); i++) {
            //apply suspension force
            ContactWheelInfo& wheel = m_wheelInfo[i];

            pe::Real suspensionForce = wheel.m_wheelsSuspensionForce;

            if (suspensionForce > wheel.m_maxSuspensionForce) {
                suspensionForce = wheel.m_maxSuspensionForce;
            }
            pe::Vector3 impulse = wheel.m_contactInfo.m_contactNormalWS * suspensionForce * step;
            pe::Vector3 relPos = wheel.m_contactInfo.m_contactPointWS - getRigidBody()->getTransform().getOrigin();

            getRigidBody()->applyImpulse(relPos, impulse);
        }

        updateFriction(step);

        for (i = 0; i < (int)m_wheelInfo.size(); i++) {
            ContactWheelInfo& wheel = m_wheelInfo[i];
            pe::Vector3 relPos = wheel.m_contactInfo.m_hardPointWS - getRigidBody()->getTransform().getOrigin();
            pe::Vector3 vel = getRigidBody()->getLinearVelocityAtLocalPoint(relPos);

            if (wheel.m_contactInfo.m_isInContact) {
                const pe::Transform& chassisWorldTransform = getChassisWorldTransform();

                pe::Vector3 fwd(
                        chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
                        chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
                        chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

                pe::Real proj = fwd.dot(wheel.m_contactInfo.m_contactNormalWS);
                fwd -= wheel.m_contactInfo.m_contactNormalWS * proj;

                pe::Real proj2 = fwd.dot(vel);

                wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
                wheel.m_rotation += wheel.m_deltaRotation;
            } else {
                wheel.m_rotation += wheel.m_deltaRotation;
            }

            wheel.m_deltaRotation *= R(0.99);  //damping of rotation when not in contact
        }
    }

    void ContactVehicle::setSteeringValue(pe::Real steering, int wheel) {
        ContactWheelInfo& wheelInfo = getWheelInfo(wheel);
        wheelInfo.m_steering = steering;
    }

    pe::Real ContactVehicle::getSteeringValue(int wheel) const {
        return getWheelInfo(wheel).m_steering;
    }

    void ContactVehicle::applyEngineForce(pe::Real force, int wheel) {
        ContactWheelInfo& wheelInfo = getWheelInfo(wheel);
        wheelInfo.m_engineForce = force;
    }

    const ContactWheelInfo& ContactVehicle::getWheelInfo(int index) const {
        return m_wheelInfo[index];
    }

    ContactWheelInfo& ContactVehicle::getWheelInfo(int index) {
        return m_wheelInfo[index];
    }

    void ContactVehicle::setBrake(pe::Real brake, int wheelIndex) {
        getWheelInfo(wheelIndex).m_brake = brake;
    }

    void ContactVehicle::updateSuspension(pe::Real deltaTime) {
        (void)deltaTime;

        pe::Real chassisMass = R(1.) / m_chassisBody->getInvMass();

        for (int w_it = 0; w_it < getNumWheels(); w_it++) {
            ContactWheelInfo& wheel_info = m_wheelInfo[w_it];

            if (wheel_info.m_contactInfo.m_isInContact) {
                pe::Real force;
                //	Spring
                {
                    pe::Real susp_length = wheel_info.getSuspensionRestLength();
                    pe::Real current_length = wheel_info.m_contactInfo.m_suspensionLength -
                            wheel_info.m_contactInfo.m_contactDepth;

                    pe::Real length_diff = (susp_length - current_length);

                    force = wheel_info.m_suspensionStiffness * length_diff *
                            wheel_info.m_clippedInvContactDotSuspension;
                }

                // Damper
                {
                    pe::Real projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
                    {
                        pe::Real susp_damping;
                        if (projected_rel_vel < 0) {
                            susp_damping = wheel_info.m_wheelsDampingCompression;
                        } else {
                            susp_damping = wheel_info.m_wheelsDampingRelaxation;
                        }
                        force -= susp_damping * projected_rel_vel;
                    }
                }

                // RESULT
                wheel_info.m_wheelsSuspensionForce = force * chassisMass;
                if (wheel_info.m_wheelsSuspensionForce < 0) {
                    wheel_info.m_wheelsSuspensionForce = 0;
                }
            } else {
                wheel_info.m_wheelsSuspensionForce = 0;
            }
        }
    }

    struct WheelContactPoint {
        pe_phys_object::RigidBody* m_body0;
        pe_phys_object::RigidBody* m_body1;
        pe::Vector3 m_frictionPositionWorld;
        pe::Vector3 m_frictionDirectionWorld;
        pe::Real m_jacDiagABInv;
        pe::Real m_maxImpulse;

        WheelContactPoint(pe_phys_object::RigidBody* body0, pe_phys_object::RigidBody* body1,
                          const pe::Vector3& frictionPosWorld, const pe::Vector3& frictionDirectionWorld,
                          pe::Real maxImpulse):
                m_body0(body0),
                m_body1(body1),
                m_frictionPositionWorld(frictionPosWorld),
                m_frictionDirectionWorld(frictionDirectionWorld),
                m_maxImpulse(maxImpulse) {
            pe::Real denom0 = body0->getImpulseDenominator(
                    frictionPosWorld, frictionDirectionWorld);
            pe::Real denom1 = body1->getImpulseDenominator(
                    frictionPosWorld, frictionDirectionWorld);
            pe::Real relaxation = R(1.0);
            m_jacDiagABInv = relaxation / (denom0 + denom1);
        }
    };

    static pe::Real calcRollingFriction(WheelContactPoint& contactPoint, int numWheelsOnGround) {
        pe::Real j1;

        const pe::Vector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

        pe::Vector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getTransform().getOrigin();
        pe::Vector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getTransform().getOrigin();

        pe::Real maxImpulse = contactPoint.m_maxImpulse;

        pe::Vector3 vel1 = contactPoint.m_body0->getLinearVelocityAtLocalPoint(rel_pos1);
        pe::Vector3 vel2 = contactPoint.m_body1->getLinearVelocityAtLocalPoint(rel_pos2);
        pe::Vector3 vel = vel1 - vel2;

        pe::Real vRel = contactPoint.m_frictionDirectionWorld.dot(vel);

        // calculate j that moves us to zero relative velocity
        j1 = -vRel * contactPoint.m_jacDiagABInv / R(numWheelsOnGround);
        j1 = PE_MIN(j1, maxImpulse);
        j1 = PE_MAX(j1, -maxImpulse);

        return j1;
    }

    //bilateral constraint between two dynamic objects
    static void resolveSingleBilateral(pe_phys_object::RigidBody& body1, const pe::Vector3& pos1,
                                pe_phys_object::RigidBody& body2, const pe::Vector3& pos2,
                                pe::Real distance, const pe::Vector3& normal, pe::Real& impulse, pe::Real timeStep) {
        (void)timeStep;
        (void)distance;

        pe::Real normalLenSqr = normal.norm();
        if (normalLenSqr > R(1.1)) {
            impulse = 0;
            return;
        }
        pe::Vector3 rel_pos1 = pos1 - body1.getTransform().getOrigin();
        pe::Vector3 rel_pos2 = pos2 - body2.getTransform().getOrigin();
        //this jacobian entry could be re-used for all iterations

        pe::Vector3 vel1 = body1.getLinearVelocityAtLocalPoint(rel_pos1);
        pe::Vector3 vel2 = body2.getLinearVelocityAtLocalPoint(rel_pos2);
        pe::Vector3 vel = vel1 - vel2;

        utils::JacobianEntry jac(body1.getTransform().getBasis().transposed(),
                                 body2.getTransform().getBasis().transposed(),
                                 rel_pos1, rel_pos2, normal,
                                 body1.getLocalInvInertia().getDiag(), body1.getInvMass(),
                                 pe::Vector3::zeros(), 0);

        pe::Real jacDiagAB = jac.getDiagonal();
        pe::Real jacDiagABInv = R(1.) / jacDiagAB;

        pe::Real rel_vel = normal.dot(vel);

        pe::Real contactDamping = R(0.2);

#   ifdef ONLY_USE_LINEAR_MASS
        Real massTerm = Real(1.) / (body1.getInvMass() + body2.getInvMass());
	    impulse = -contactDamping * rel_vel * massTerm;
#   else
        pe::Real velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
        impulse = velocityImpulse;
#   endif
    }

    static pe::Real sideFrictionStiffness2 = R(1.0);
    void ContactVehicle::updateFriction(pe::Real timeStep) {
        //calculate the impulse, so that the wheels don't move sidewards
        int numWheel = getNumWheels();
        if (!numWheel)
            return;

        m_forwardWS.resize(numWheel);
        m_axle.resize(numWheel);
        m_forwardImpulse.resize(numWheel);
        m_sideImpulse.resize(numWheel);

        m_numWheelsOnGround = 0;

        //collapse all those loops into one!
        for (int i = 0; i < getNumWheels(); i++) {
            ContactWheelInfo& wheelInfo = m_wheelInfo[i];
            class pe_phys_object::RigidBody* groundObject =
                    (class pe_phys_object::RigidBody*)wheelInfo.m_contactInfo.m_groundObject;
            if (groundObject)
                m_numWheelsOnGround++;
            m_sideImpulse[i] = 0;
            m_forwardImpulse[i] = 0;
        }

        {
            for (int i = 0; i < getNumWheels(); i++) {
                ContactWheelInfo& wheelInfo = m_wheelInfo[i];

                class pe_phys_object::RigidBody* groundObject =
                        (class pe_phys_object::RigidBody*)wheelInfo.m_contactInfo.m_groundObject;

                if (groundObject) {
                    const pe::Transform& wheelTrans = getWheelTransformWS(i);

                    pe::Matrix3 wheelBasis0 = wheelTrans.getBasis();
                    m_axle[i] = -pe::Vector3(
                            wheelBasis0[0][m_indexRightAxis],
                            wheelBasis0[1][m_indexRightAxis],
                            wheelBasis0[2][m_indexRightAxis]);

                    const pe::Vector3& surfNormalWS = wheelInfo.m_contactInfo.m_contactNormalWS;
                    pe::Real proj = m_axle[i].dot(surfNormalWS);
                    m_axle[i] -= surfNormalWS * proj;
                    m_axle[i].normalize();

                    m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
                    m_forwardWS[i].normalize();

                    resolveSingleBilateral(*m_chassisBody, wheelInfo.m_contactInfo.m_contactPointWS,
                                           *groundObject, wheelInfo.m_contactInfo.m_contactPointWS,
                                           -wheelInfo.m_contactInfo.m_contactDepth, m_axle[i],
                                           m_sideImpulse[i], timeStep);

                    m_sideImpulse[i] *= sideFrictionStiffness2;
                }
            }
        }

        pe::Real sideFactor = R(1.);
        pe::Real fwdFactor = R(0.5);

        bool sliding = false;
        {
            for (int wheel = 0; wheel < getNumWheels(); wheel++) {
                ContactWheelInfo& wheelInfo = m_wheelInfo[wheel];
                class pe_phys_object::RigidBody* groundObject =
                        (class pe_phys_object::RigidBody*)wheelInfo.m_contactInfo.m_groundObject;

                pe::Real rollingFriction;

                if (groundObject) {
                    if (wheelInfo.m_engineForce != 0.f) {
                        rollingFriction = wheelInfo.m_engineForce * timeStep;
                    } else {
                        pe::Real maxImpulse = (wheelInfo.m_brake != 0) ?
                                wheelInfo.m_brake : wheelInfo.m_rollDamping;
                        WheelContactPoint contactPt(m_chassisBody, groundObject,
                                                    wheelInfo.m_contactInfo.m_contactPointWS,
                                                    m_forwardWS[wheel], maxImpulse);
                        rollingFriction = calcRollingFriction(contactPt, m_numWheelsOnGround);
                        // to avoid sliding on slopes
                        if (wheelInfo.m_brake != 0) {
                            rollingFriction = rollingFriction > 0 ?
                                    PE_MAX(wheelInfo.m_brake / R(10), rollingFriction) :
                                    PE_MIN(-wheelInfo.m_brake / R(10), rollingFriction);
                        }
                    }
                }

                //switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

                m_forwardImpulse[wheel] = 0;
                m_wheelInfo[wheel].m_skidInfo = R(1.);

                if (groundObject) {
                    m_wheelInfo[wheel].m_skidInfo = R(1.);

                    pe::Real maxImp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
                    pe::Real maxImpSide = maxImp;

                    pe::Real maxImpSquared = maxImp * maxImpSide;

                    m_forwardImpulse[wheel] = rollingFriction;  //wheelInfo.m_engineForce* timeStep;

                    pe::Real x = (m_forwardImpulse[wheel]) * fwdFactor;
                    pe::Real y = (m_sideImpulse[wheel]) * sideFactor;

                    pe::Real impulseSquared = (x * x + y * y);

                    if (impulseSquared > maxImpSquared) {
                        sliding = true;

                        pe::Real factor = maxImp / std::sqrt(impulseSquared);

                        m_wheelInfo[wheel].m_skidInfo *= factor;
                    }
                }
            }
        }

        if (sliding) {
            for (int wheel = 0; wheel < getNumWheels(); wheel++) {
                if (m_sideImpulse[wheel] != 0) {
                    if (m_wheelInfo[wheel].m_skidInfo < R(1.)) {
                        m_forwardImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
                        m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
                    }
                }
            }
        }

        // apply the impulses
        {
            for (int wheel = 0; wheel < getNumWheels(); wheel++) {
                ContactWheelInfo& wheelInfo = m_wheelInfo[wheel];

                pe::Vector3 rel_pos = wheelInfo.m_contactInfo.m_contactPointWS -
                                      m_chassisBody->getTransform().getOrigin();

                if (m_forwardImpulse[wheel] != 0) {
                    m_chassisBody->applyImpulse(rel_pos,
                                                m_forwardWS[wheel] * (m_forwardImpulse[wheel]));
                }
                if (m_sideImpulse[wheel] != 0) {
                    class pe_phys_object::RigidBody* groundObject =
                            (class pe_phys_object::RigidBody*)m_wheelInfo[wheel].m_contactInfo.m_groundObject;

                    pe::Vector3 rel_pos2 = wheelInfo.m_contactInfo.m_contactPointWS -
                                           groundObject->getTransform().getOrigin();

                    pe::Vector3 sideImp = m_axle[wheel] * m_sideImpulse[wheel];

#if defined ROLLING_INFLUENCE_FIX  // fix. It only worked if car's up was along Y - VT.
                    pe::Vector3 vChassisWorldUp = getRigidBody()->getTransform().getBasis().getColumn(m_indexUpAxis);
                    rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f - wheelInfo.m_rollInfluence));
#else
                    rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
                    m_chassisBody->applyImpulse(rel_pos, sideImp);

                    //apply friction impulse on the ground
                    groundObject->applyImpulse(rel_pos2, -sideImp);
                }
            }
        }
    }

    pe::Vector3 ContactVehicle::getForwardVector() const {
        const pe::Transform& chassisTrans = getChassisWorldTransform();

        pe::Vector3 forwardW(
                chassisTrans.getBasis()[0][m_indexForwardAxis],
                chassisTrans.getBasis()[1][m_indexForwardAxis],
                chassisTrans.getBasis()[2][m_indexForwardAxis]);

        return forwardW;
    }

    pe::Vector3 ContactVehicle::getUpVector() const {
        const pe::Transform& chassisTrans = getChassisWorldTransform();

        pe::Vector3 up(
                chassisTrans.getBasis()[0][m_indexUpAxis],
                chassisTrans.getBasis()[1][m_indexUpAxis],
                chassisTrans.getBasis()[2][m_indexUpAxis]);

        return up;
    }

    void ContactVehicle::setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
        m_indexRightAxis = rightIndex;
        m_indexUpAxis = upIndex;
        m_indexForwardAxis = forwardIndex;
    }

} // namespace pe_phys_vehicle
