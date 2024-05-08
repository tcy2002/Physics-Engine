#include "phys/vehicle/raycast_vehicle/raycast_vehicle.h"
#include "phys/vehicle/contact_vehicle/contact_vehicle.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/sphere_shape.h"

#define PE_USE_CONTACT_VEHICLE true
#define PE_USE_SPHERE_WHEEL false

namespace pe_phys_vehicle {

    class TankTemplate {
    private:
#   if PE_USE_CONTACT_VEHICLE
        ContactVehicle* vehicle;
#   else
        RaycastVehicle* vehicle;
#   endif
        pe_phys_object::RigidBody* body, * turret, * barrel;
        pe::Array<pe_phys_object::RigidBody*> wheels;
        pe::Array<pe_phys_object::RigidBody*> trackSegments;

        pe::Array<pe::Vector3> trackHoldingPoints;
        pe::Array<pe::Vector3> trackHoldingWheels;

        COMMON_MEMBER_SET_GET(pe::Transform, transform, Transform)
        COMMON_MEMBER_SET_GET(pe::Real, bodyWidth, BodyWidth)
        COMMON_MEMBER_SET_GET(pe::Real, bodyLength, BodyLength)
        COMMON_MEMBER_SET_GET(pe::Real, bodyHeight, BodyHeight)
        COMMON_MEMBER_SET_GET(pe::Real, bodyMass, BodyMass)
        COMMON_MEMBER_SET_GET(pe::Real, turretWidth, TurretWidth)
        COMMON_MEMBER_SET_GET(pe::Real, turretLength, TurretLength)
        COMMON_MEMBER_SET_GET(pe::Real, turretHeight, TurretHeight)
        COMMON_MEMBER_SET_GET(pe::Real, turretMass, TurretMass)
        COMMON_MEMBER_SET_GET(pe::Real, turretRotSpeed, TurretRotSpeed)
        COMMON_MEMBER_SET_GET(pe::Real, turretMaxAngle, TurretMaxAngle)
        COMMON_MEMBER_SET_GET(pe::Real, barrelRadius, BarrelRadius)
        COMMON_MEMBER_SET_GET(pe::Real, barrelLength, BarrelLength)
        COMMON_MEMBER_SET_GET(pe::Real, barrelMass, BarrelMass)
        COMMON_MEMBER_SET_GET(pe::Real, barrelRotSpeed, BarrelRotSpeed)
        COMMON_MEMBER_SET_GET(pe::Real, barrelMaxAngle, BarrelMaxAngle)
        COMMON_MEMBER_SET_GET(int, wheelNum, WheelNum)
        COMMON_MEMBER_SET_GET(pe::Real, powerWheelRadius, PowerWheelRadius)
        COMMON_MEMBER_SET_GET(pe::Real, drivenWheelRadius, DrivenWheelRadius)
        COMMON_MEMBER_SET_GET(pe::Real, wheelWidth, WheelWidth)
        COMMON_MEMBER_SET_GET(pe::Real, wheelFriction, WheelFriction)
        COMMON_MEMBER_SET_GET(pe::Real, wheelRollInfluence, WheelRollInfluence)
        COMMON_MEMBER_SET_GET(pe::Real, wheelRollDamping, WheelRollDamping)
        COMMON_MEMBER_SET_GET(pe::Real, wheelMass, WheelMass)
        COMMON_MEMBER_SET_GET(pe::Real, trackThickness, TrackThickness)
        COMMON_MEMBER_SET_GET(int, trackSegmentNum, TrackSegmentNum)
        COMMON_MEMBER_SET_GET(pe::Real, trackSegmentWidth, TrackSegmentWidth)
        COMMON_MEMBER_SET_GET(pe::Real, suspensionStiffness, SuspensionStiffness)
        COMMON_MEMBER_SET_GET(pe::Real, suspensionDamping, SuspensionDamping)
        COMMON_MEMBER_SET_GET(pe::Real, suspensionCompression, SuspensionCompression)
        COMMON_MEMBER_SET_GET(pe::Real, engineForce, EngineForce)
        COMMON_MEMBER_SET_GET(pe::Real, maxSpeed, MaxSpeed)
        COMMON_MEMBER_SET_GET(pe::Real, maxRotSpeed, MaxRotSpeed)

        pe::Real forwardForce;
        pe::Real backwardForce;
        pe::Real turnForce;
        pe::Real brakeForce;

        pe::Vector3 turretTrl;
        pe::Vector3 barrelTrl;

        pe::Real turretAngle = 0;
        pe::Real barrelAngle = 0;

        pe::Real trackOffsetLeft = 0;
        pe::Real trackOffsetRight = 0;
        pe::Real trackLengthLeft = 0;
        pe::Real trackLengthRight = 0;
        pe::Real trackSegmentWidthLeft = 0;
        pe::Real trackSegmentWidthRight = 0;

        // rotate a vector by axis and angle
        static inline pe::Vector3 rotateVector3(const pe::Vector3& vec, const pe::Vector3& axis, pe::Real angle) {
            pe::Real s = sin(angle), c = cos(angle);
            return vec * c + axis.cross(vec) * s + axis * axis.dot(vec) * (1 - c);
        }

        // limit a value to a range
        static inline pe::Real clampReal(pe::Real l, pe::Real h, pe::Real x) {
            if (x < l) return l;
            if (x > h) return h;
            return x;
        }

        // initialize rigid body of tank
        void initBody(pe_intf::World* dw);

        // initialize raycast vehicle
        void initVehicle(pe_intf::World* dw);

        // initialize wheels
        void initWheels(pe_intf::World* dw);

        // initialize tracks
        void initTracks(pe_intf::World* dw);

        // update transform of turret and barrel
        void updateTurretAndBarrelTransform();

        // update transform of wheels
        void updateWheelsTransform();

        // update transform of tracks
        void updateTracksTransform();

        // update transform of single side of tracks
        void updateOneSideTrackTransform(int side);

        // synchronize the holding points of tracks
        void updateTrackHoldings();

        // synchronize the holding points of single side of tracks
        void updateOneSideTrackHoldings(int side);

        // synchronize the velocity of tracks
        void uniformTrackVelocity();

        // synchronize the velocity of single side of tracks
        void uniformTrackVelocityOneSide(int side);

        // set brake
        void setBrake(bool brake);

    public:
        // refer to tank 99a: 7.6m long and 3.5m wide.
        PE_API TankTemplate();
        ~TankTemplate() { delete vehicle; }

        PE_API void init(pe_intf::World* dw);
        pe_phys_object::RigidBody* getBody() { return body; }
        pe_phys_object::RigidBody* getTurret() { return turret; }
        pe_phys_object::RigidBody* getBarrel() { return barrel; }
        pe::Array<pe_phys_object::RigidBody*>& getWheels() { return wheels; }
        pe::Array<pe_phys_object::RigidBody*>& getTrackSegments() { return trackSegments; }

        PE_API void advance(pe::Real step);

        PE_API void shoot(pe_intf::World* dw, pe::Real speed, pe::Real mass, pe::Real radius, pe::Real lifeTime);

        PE_API void idle();
        PE_API void brake();
        PE_API void moveForward();
        PE_API void moveBackward();
        PE_API void turnLeft();
        PE_API void turnRight();
        PE_API void barrelRotLeft(pe::Real step);
        PE_API void barrelRotRight(pe::Real step);
        PE_API void barrelRotUp(pe::Real step);
        PE_API void barrelRotDown(pe::Real step);
        PE_API pe::Real getSpeedKmHour() const;
        PE_API void setActive(bool active, pe_intf::World* dw);
    };

} // namespace pe_phys_vehicle