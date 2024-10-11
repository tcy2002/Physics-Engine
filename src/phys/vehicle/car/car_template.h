#include "phys/vehicle/raycast_vehicle/raycast_vehicle.h"
#include "phys/vehicle/contact_vehicle/contact_vehicle.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/sphere_shape.h"

#define PE_USE_CONTACT_VEHICLE false
#define PE_USE_SPHERE_WHEEL false

namespace pe_phys_vehicle {

    class CarTemplate {
    private:
#   if PE_USE_CONTACT_VEHICLE
        ContactVehicle* vehicle;
#   else
        RaycastVehicle* vehicle;
#   endif
        pe_phys_object::RigidBody* body, * cabin;
        pe::Array<pe_phys_object::RigidBody*> wheels;

        COMMON_MEMBER_SET_GET(pe::Transform, transform, Transform)
        COMMON_MEMBER_SET_GET(pe::Real, bodyWidth, BodyWidth)
        COMMON_MEMBER_SET_GET(pe::Real, bodyLength, BodyLength)
        COMMON_MEMBER_SET_GET(pe::Real, bodyHeight, BodyHeight)
        COMMON_MEMBER_SET_GET(pe::Real, bodyMass, BodyMass)
        COMMON_MEMBER_SET_GET(pe::Real, cabinWidth, CabinWidth)
        COMMON_MEMBER_SET_GET(pe::Real, cabinLength, CabinLength)
        COMMON_MEMBER_SET_GET(pe::Real, cabinHeight, CabinHeight)
        COMMON_MEMBER_SET_GET(pe::Real, cabinMass, CabinMass)
        COMMON_MEMBER_SET_GET(pe::Real, wheelRadius, WheelRadius)
        COMMON_MEMBER_SET_GET(pe::Real, wheelWidth, WheelWidth)
        COMMON_MEMBER_SET_GET(pe::Real, wheelFriction, WheelFriction)
        COMMON_MEMBER_SET_GET(pe::Real, wheelRollInfluence, WheelRollInfluence)
        COMMON_MEMBER_SET_GET(pe::Real, wheelRollDamping, WheelRollDamping)
        COMMON_MEMBER_SET_GET(pe::Real, wheelMass, WheelMass)
        COMMON_MEMBER_SET_GET(pe::Real, suspensionStiffness, SuspensionStiffness)
        COMMON_MEMBER_SET_GET(pe::Real, suspensionDamping, SuspensionDamping)
        COMMON_MEMBER_SET_GET(pe::Real, suspensionCompression, SuspensionCompression)
        COMMON_MEMBER_SET_GET(pe::Real, engineForce, EngineForce)
        COMMON_MEMBER_SET_GET(pe::Real, maxSpeed, MaxSpeed)
        COMMON_MEMBER_SET_GET(pe::Real, maxRotSpeed, MaxRotSpeed)

        pe::Real forwardForce;
        pe::Real backwardForce;
        pe::Real brakeForce;

        pe::Vector3 cabinTrl;
        pe::Vector3 barrelTrl;

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

        // update transform of cabin
        void updateCabinTransform();

        // update transform of wheels
        void updateWheelsTransform();

        // set brake
        void setBrake(bool brake);

    public:
        // refer to tank 99a: 7.6m long and 3.5m wide.
        PE_API CarTemplate();
        ~CarTemplate() { delete vehicle; }

        PE_API void init(pe_intf::World* dw);
        pe_phys_object::RigidBody* getBody() { return body; }
        pe_phys_object::RigidBody* getCabin() { return cabin; }
        pe::Array<pe_phys_object::RigidBody*>& getWheels() { return wheels; }

        PE_API void advance(pe::Real step);

        PE_API void idle();
        PE_API void brake();
        PE_API void moveForward();
        PE_API void moveBackward();
        PE_API void turnLeft();
        PE_API void turnRight();
        PE_API void turnStraight();
        PE_API pe::Real getSpeedKmHour() const;
        PE_API void setActive(bool active, pe_intf::World* dw);
    };

} // namespace pe_phys_vehicle