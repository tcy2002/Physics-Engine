#include "raycast_vehicle.h"

#define ROLLING_INFLUENCE_FIX

RaycastVehicle::VehicleTuning::VehicleTuning():
        m_suspensionStiffness(pe::Real(5.88)),
        m_suspensionCompression(pe::Real(0.83)),
        m_suspensionDamping(pe::Real(0.88)),
        m_maxSuspensionTravelCm(pe::Real(500.)),
        m_frictionSlip(pe::Real(10.5)),
        m_maxSuspensionForce(pe::Real(6000.)) {}

RaycastVehicle::RaycastVehicle(const VehicleTuning& tuning, pe_phys_object::RigidBody* chassis,
                               VehicleRaycaster* raycaster):
                               m_vehicleRaycaster(raycaster), m_pitchControl(pe::Real(0.)) {
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	defaultInit(tuning);
}

void RaycastVehicle::defaultInit(const VehicleTuning& tuning) {
	(void)tuning;
	m_currentVehicleSpeedKmHour = pe::Real(0.);
	m_steeringValue = pe::Real(0.);
}

pe_phys_object::RigidBody& RaycastVehicle::getFixedBody() {
    static pe_phys_object::RigidBody s_fixed;
    s_fixed.setTransform(pe::Transform::identity());
    s_fixed.setMass(0);
    s_fixed.setLocalInertia(pe::Matrix3::identity());
    return s_fixed;
}

//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
WheelInfo& RaycastVehicle::addWheel(const pe::Vector3& connectionPointCS, const pe::Vector3& wheelDirectionCS0,
                                    const pe::Vector3& wheelAxleCS, pe::Real suspensionRestLength,
                                    pe::Real wheelRadius, const VehicleTuning& tuning, bool isFrontWheel) {
	WheelInfoConstructionInfo ci;

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

	m_wheelInfo.push_back(WheelInfo(ci));

	WheelInfo& wheel = m_wheelInfo[getNumWheels() - 1];

	updateWheelTransformsWS(wheel, false);
	updateWheelTransform(getNumWheels() - 1, false);
	return wheel;
}

const pe::Transform& RaycastVehicle::getWheelTransformWS(int wheelIndex) const {
	const WheelInfo& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;
}

void RaycastVehicle::updateWheelTransform(int wheelIndex, bool interpolatedTransform) {
	WheelInfo& wheel = m_wheelInfo[wheelIndex];
	updateWheelTransformsWS(wheel, interpolatedTransform);
	pe::Vector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const pe::Vector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	pe::Vector3 fwd = up.cross(right);
	fwd = fwd.normalized();

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
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS *
        wheel.m_raycastInfo.m_suspensionLength);
}

void RaycastVehicle::resetSuspension() {
	int i;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		WheelInfo& wheel = m_wheelInfo[i];
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = pe::Real(0.0);

		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		//wheel_info.setContactFriction(pe::Real(0.0));
		wheel.m_clippedInvContactDotSuspension = pe::Real(1.0);
	}
}

void RaycastVehicle::updateWheelTransformsWS(WheelInfo& wheel, bool interpolatedTransform) const {
	wheel.m_raycastInfo.m_isInContact = false;

	pe::Transform chassisTrans = getChassisWorldTransform();

	wheel.m_raycastInfo.m_hardPointWS = chassisTrans * wheel.m_chassisConnectionPointCS;
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

pe::Real RaycastVehicle::rayCast(WheelInfo& wheel) {
	updateWheelTransformsWS(wheel, false);

	pe::Real depth = -1;

	pe::Real rayLen = wheel.getSuspensionRestLength() + wheel.m_wheelsRadius;

	pe::Vector3 rayVector = wheel.m_raycastInfo.m_wheelDirectionWS * (rayLen);
	const pe::Vector3& source = wheel.m_raycastInfo.m_hardPointWS;
	wheel.m_raycastInfo.m_contactPointWS = source + rayVector;
	const pe::Vector3& target = wheel.m_raycastInfo.m_contactPointWS;

	pe::Real param;

	VehicleRaycaster::VehicleRaycasterResult rayResults;

	void* object = m_vehicleRaycaster->castRay(source, target, rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object) {
		param = rayResults.m_distFraction;
		depth = rayLen * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;

		wheel.m_raycastInfo.m_groundObject = &getFixedBody();  ///@todo for driving on dynamic/movable objects!;
		//wheel.m_raycastInfo.m_groundObject = object;

		pe::Real hitDistance = param * rayLen;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
		//clamp on max suspension travel

		pe::Real minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm * pe::Real(0.01);
		pe::Real maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.m_maxSuspensionTravelCm * pe::Real(0.01);
		if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength) {
			wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
		}
		if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength) {
			wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
		}

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

		pe::Real denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_raycastInfo.m_wheelDirectionWS);

		pe::Vector3 chassis_velocity_at_contactPoint;
		pe::Vector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getTransform().getOrigin();

		chassis_velocity_at_contactPoint = getRigidBody()->getLinearVelocityAtLocalPoint(relpos);

		pe::Real projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

		if (denominator >= pe::Real(-0.1)) {
			wheel.m_suspensionRelativeVelocity = pe::Real(0.0);
			wheel.m_clippedInvContactDotSuspension = pe::Real(1.0) / pe::Real(0.1);
		} else {
			pe::Real inv = pe::Real(-1.) / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}
	} else {
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = pe::Real(0.0);
		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = pe::Real(1.0);
	}

	return depth;
}

const pe::Transform& RaycastVehicle::getChassisWorldTransform() const {
	/*if (getRigidBody()->getMotionState())
	{
		pe::Transform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/

	return getRigidBody()->getTransform();
}

void RaycastVehicle::updateVehicle(pe::Real step) {
	{
		for (int i = 0; i < getNumWheels(); i++) {
			updateWheelTransform(i, false);
		}
	}

	m_currentVehicleSpeedKmHour = pe::Real(3.6) * getRigidBody()->getLinearVelocity().norm();

	const pe::Transform& chassisTrans = getChassisWorldTransform();

	pe::Vector3 forwardW(
		chassisTrans.getBasis()[0][m_indexForwardAxis],
		chassisTrans.getBasis()[1][m_indexForwardAxis],
		chassisTrans.getBasis()[2][m_indexForwardAxis]);

	if (forwardW.dot(getRigidBody()->getLinearVelocity()) < pe::Real(0.)) {
		m_currentVehicleSpeedKmHour *= pe::Real(-1.);
	}

	//
	// simulate suspension
	//

    int i;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		//pe::Real depth;
		//depth =
		rayCast(m_wheelInfo[i]);
	}

	updateSuspension(step);

	for (i = 0; i < m_wheelInfo.size(); i++)
	{
		//apply suspension force
		WheelInfo& wheel = m_wheelInfo[i];

		pe::Real suspensionForce = wheel.m_wheelsSuspensionForce;

		if (suspensionForce > wheel.m_maxSuspensionForce)
		{
			suspensionForce = wheel.m_maxSuspensionForce;
		}
		pe::Vector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		pe::Vector3 relPos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getTransform().getOrigin();

		getRigidBody()->applyImpulse(impulse, relPos);
	}

	updateFriction(step);

	for (i = 0; i < (int)m_wheelInfo.size(); i++) {
		WheelInfo& wheel = m_wheelInfo[i];
		pe::Vector3 relPos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getTransform().getOrigin();
		pe::Vector3 vel = getRigidBody()->getLinearVelocityAtLocalPoint(relPos);

		if (wheel.m_raycastInfo.m_isInContact) {
			const pe::Transform& chassisWorldTransform = getChassisWorldTransform();

			pe::Vector3 fwd(
				chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

			pe::Real proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
			fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			pe::Real proj2 = fwd.dot(vel);

			wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
			wheel.m_rotation += wheel.m_deltaRotation;
		} else {
			wheel.m_rotation += wheel.m_deltaRotation;
		}

		wheel.m_deltaRotation *= pe::Real(0.99);  //damping of rotation when not in contact
	}
}

void RaycastVehicle::setSteeringValue(pe::Real steering, int wheel) {
	WheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}

pe::Real RaycastVehicle::getSteeringValue(int wheel) const {
	return getWheelInfo(wheel).m_steering;
}

void RaycastVehicle::applyEngineForce(pe::Real force, int wheel) {
	WheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}

const WheelInfo& RaycastVehicle::getWheelInfo(int index) const {
	return m_wheelInfo[index];
}

WheelInfo& RaycastVehicle::getWheelInfo(int index) {
	return m_wheelInfo[index];
}

void RaycastVehicle::setBrake(pe::Real brake, int wheelIndex) {
	getWheelInfo(wheelIndex).m_brake = brake;
}

void RaycastVehicle::updateSuspension(pe::Real deltaTime) {
	(void)deltaTime;

	pe::Real chassisMass = pe::Real(1.) / m_chassisBody->getInvMass();

	for (int w_it = 0; w_it < getNumWheels(); w_it++) {
		WheelInfo& wheel_info = m_wheelInfo[w_it];

		if (wheel_info.m_raycastInfo.m_isInContact) {
			pe::Real force;
			//	Spring
			{
				pe::Real susp_length = wheel_info.getSuspensionRestLength();
				pe::Real current_length = wheel_info.m_raycastInfo.m_suspensionLength;

				pe::Real length_diff = (susp_length - current_length);

				force = wheel_info.m_suspensionStiffness * length_diff * wheel_info.m_clippedInvContactDotSuspension;
			}

			// Damper
			{
				pe::Real projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				{
					pe::Real susp_damping;
					if (projected_rel_vel < pe::Real(0.0)) {
						susp_damping = wheel_info.m_wheelsDampingCompression;
					} else {
						susp_damping = wheel_info.m_wheelsDampingRelaxation;
					}
					force -= susp_damping * projected_rel_vel;
				}
			}

			// RESULT
			wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			if (wheel_info.m_wheelsSuspensionForce < pe::Real(0.)) {
				wheel_info.m_wheelsSuspensionForce = pe::Real(0.);
			}
		} else {
			wheel_info.m_wheelsSuspensionForce = pe::Real(0.0);
		}
	}
}

struct btWheelContactPoint
{
	pe_phys_object::RigidBody* m_body0;
	pe_phys_object::RigidBody* m_body1;
	pe::Vector3 m_frictionPositionWorld;
	pe::Vector3 m_frictionDirectionWorld;
	pe::Real m_jacDiagABInv;
	pe::Real m_maxImpulse;

	btWheelContactPoint(pe_phys_object::RigidBody* body0, pe_phys_object::RigidBody* body1, const pe::Vector3& frictionPosWorld, const pe::Vector3& frictionDirectionWorld, pe::Real maxImpulse)
		: m_body0(body0),
		  m_body1(body1),
		  m_frictionPositionWorld(frictionPosWorld),
		  m_frictionDirectionWorld(frictionDirectionWorld),
		  m_maxImpulse(maxImpulse) {
		pe::Real denom0 = body0->getImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		pe::Real denom1 = body1->getImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		pe::Real relaxation = 1.f;
		m_jacDiagABInv = relaxation / (denom0 + denom1);
	}
};

pe::Real calcRollingFriction(btWheelContactPoint& contactPoint, int numWheelsOnGround) {
	pe::Real j1;

	const pe::Vector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

	pe::Vector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getTransform().getOrigin();
	pe::Vector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getTransform().getOrigin();

	pe::Real maxImpulse = contactPoint.m_maxImpulse;

	pe::Vector3 vel1 = contactPoint.m_body0->getLinearVelocityAtLocalPoint(rel_pos1);
	pe::Vector3 vel2 = contactPoint.m_body1->getLinearVelocityAtLocalPoint(rel_pos2);
	pe::Vector3 vel = vel1 - vel2;

	pe::Real vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

	// calculate j that moves us to zero relative velocity
	j1 = -vrel * contactPoint.m_jacDiagABInv / pe::Real(numWheelsOnGround);
    j1 = PE_MIN(j1, maxImpulse);
    j1 = PE_MAX(j1, -maxImpulse);

	return j1;
}

//bilateral constraint between two dynamic objects
void resolveSingleBilateral(pe_phys_object::RigidBody& body1, const pe::Vector3& pos1,
                            pe_phys_object::RigidBody& body2, const pe::Vector3& pos2,
                            pe::Real distance, const pe::Vector3& normal, pe::Real& impulse, pe::Real timeStep) {
    (void)timeStep;
    (void)distance;

    pe::Real normalLenSqr = normal.norm();
    if (normalLenSqr > pe::Real(1.1)) {
        impulse = pe::Real(0.);
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
                             pe::Vector3(0.0f, 0.0f, 0.0f), 0.0f);

    pe::Real jacDiagAB = jac.getDiagonal();
    pe::Real jacDiagABInv = pe::Real(1.) / jacDiagAB;

    pe::Real rel_vel = normal.dot(vel);

    //todo: move this into proper structure
    pe::Real contactDamping = pe::Real(0.2);

#ifdef ONLY_USE_LINEAR_MASS
    Real massTerm = Real(1.) / (body1.getInvMass() + body2.getInvMass());
	impulse = -contactDamping * rel_vel * massTerm;
#else
    pe::Real velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
    impulse = velocityImpulse;
#endif
}

pe::Real sideFrictionStiffness2 = pe::Real(1.0);
void RaycastVehicle::updateFriction(pe::Real timeStep) {
	//calculate the impulse, so that the wheels don't move sidewards
	int numWheel = getNumWheels();
	if (!numWheel)
		return;

	m_forwardWS.resize(numWheel);
	m_axle.resize(numWheel);
	m_forwardImpulse.resize(numWheel);
	m_sideImpulse.resize(numWheel);

	int numWheelsOnGround = 0;

	//collapse all those loops into one!
	for (int i = 0; i < getNumWheels(); i++) {
		WheelInfo& wheelInfo = m_wheelInfo[i];
		class pe_phys_object::RigidBody* groundObject = (class pe_phys_object::RigidBody*)wheelInfo.m_raycastInfo.m_groundObject;
		if (groundObject)
			numWheelsOnGround++;
		m_sideImpulse[i] = pe::Real(0.);
		m_forwardImpulse[i] = pe::Real(0.);
	}

	{
		for (int i = 0; i < getNumWheels(); i++) {
			WheelInfo& wheelInfo = m_wheelInfo[i];

			class pe_phys_object::RigidBody* groundObject = (class pe_phys_object::RigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

			if (groundObject) {
				const pe::Transform& wheelTrans = getWheelTransformWS(i);

				pe::Matrix3 wheelBasis0 = wheelTrans.getBasis();
				m_axle[i] = -pe::Vector3(
					wheelBasis0[0][m_indexRightAxis],
					wheelBasis0[1][m_indexRightAxis],
					wheelBasis0[2][m_indexRightAxis]);

				const pe::Vector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
				pe::Real proj = m_axle[i].dot(surfNormalWS);
				m_axle[i] -= surfNormalWS * proj;
				m_axle[i] = m_axle[i].normalized();

				m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
				m_forwardWS[i].normalize();

				resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
									   *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
									   pe::Real(0.), m_axle[i], m_sideImpulse[i], timeStep);

				m_sideImpulse[i] *= sideFrictionStiffness2;
			}
		}
	}

	pe::Real sideFactor = pe::Real(1.);
	pe::Real fwdFactor = 0.5;

	bool sliding = false;
	{
		for (int wheel = 0; wheel < getNumWheels(); wheel++) {
			WheelInfo& wheelInfo = m_wheelInfo[wheel];
			class pe_phys_object::RigidBody* groundObject = (class pe_phys_object::RigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

			pe::Real rollingFriction;

			if (groundObject) {
				if (wheelInfo.m_engineForce != 0.f) {
					rollingFriction = wheelInfo.m_engineForce * timeStep;
				} else {
					pe::Real defaultRollingFrictionImpulse = 0.f;
					pe::Real maxImpulse = (wheelInfo.m_brake != 0) ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
					btWheelContactPoint contactPt(m_chassisBody, groundObject, wheelInfo.m_raycastInfo.m_contactPointWS, m_forwardWS[wheel], maxImpulse);
					rollingFriction = calcRollingFriction(contactPt, numWheelsOnGround);
				}
			}

			//switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

			m_forwardImpulse[wheel] = pe::Real(0.);
			m_wheelInfo[wheel].m_skidInfo = pe::Real(1.);

			if (groundObject) {
				m_wheelInfo[wheel].m_skidInfo = pe::Real(1.);

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
			if (m_sideImpulse[wheel] != pe::Real(0.)) {
				if (m_wheelInfo[wheel].m_skidInfo < pe::Real(1.)) {
					m_forwardImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
					m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
				}
			}
		}
	}

	// apply the impulses
	{
		for (int wheel = 0; wheel < getNumWheels(); wheel++) {
			WheelInfo& wheelInfo = m_wheelInfo[wheel];

			pe::Vector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS -
								m_chassisBody->getTransform().getOrigin();

			if (m_forwardImpulse[wheel] != pe::Real(0.)) {
				m_chassisBody->applyImpulse(m_forwardWS[wheel] * (m_forwardImpulse[wheel]), rel_pos);
			}
			if (m_sideImpulse[wheel] != pe::Real(0.)) {
				class pe_phys_object::RigidBody* groundObject =
                        (class pe_phys_object::RigidBody*)m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

				pe::Vector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS -
									 groundObject->getTransform().getOrigin();

				pe::Vector3 sideImp = m_axle[wheel] * m_sideImpulse[wheel];

#if defined ROLLING_INFLUENCE_FIX  // fix. It only worked if car's up was along Y - VT.
				pe::Vector3 vChassisWorldUp = getRigidBody()->getTransform().getBasis().getColumn(m_indexUpAxis);
				rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f - wheelInfo.m_rollInfluence));
#else
				rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
				m_chassisBody->applyImpulse(sideImp, rel_pos);

				//apply friction impulse on the ground
				groundObject->applyImpulse(-sideImp, rel_pos2);
			}
		}
	}
}

pe::Vector3 RaycastVehicle::getForwardVector() const {
    const pe::Transform& chassisTrans = getChassisWorldTransform();

    pe::Vector3 forwardW(
            chassisTrans.getBasis()[0][m_indexForwardAxis],
            chassisTrans.getBasis()[1][m_indexForwardAxis],
            chassisTrans.getBasis()[2][m_indexForwardAxis]);

    return forwardW;
}

void RaycastVehicle::setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
    m_indexRightAxis = rightIndex;
    m_indexUpAxis = upIndex;
    m_indexForwardAxis = forwardIndex;
}

void* DefaultVehicleRaycaster::castRay(int rigid_idx, const pe::Vector3& from, const pe::Vector3& direction,
                                       pe::Real length, VehicleRaycasterResult& result)
{
	//	RayResultCallback& resultCallback;

    pe_phys_ray::Raycast* ray = new pe_phys_ray::Raycast(from, direction, length);
    ray->performRayTest(rigid_idx, m_world->getRigidBodies());


    if (ray->m_resultCallback->hasHit()) {
        pe_phys_object::RigidBody* body = m_world->getRigidBody(
                ray->m_resultCallback->m_collisionObject->getGlobalId());
        if (body) {
            result.m_hitPointInWorld = ray->m_resultCallback->m_hitPoint;
            result.m_hitNormalInWorld = ray->m_resultCallback->m_normal;
            result.m_hitNormalInWorld.normalize();
            result.m_distFraction = ray->m_resultCallback->m_distance;
            return (void*)body;
        }
    }
    return 0;
}
