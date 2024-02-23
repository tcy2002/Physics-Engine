#pragma once

#include "phys/phys_general.h"
#include "phys/object/collision_object.h"

#define PE_CONTACT_MAX_POINTS 4

namespace pe_phys_collision {

    class ContactPoint {
    protected:
        COMMON_MEMBER_SET_GET(pe::Vector3, world_pos, WorldPos)
        COMMON_MEMBER_SET_GET(pe::Vector3, world_normal, WorldNormal)

        COMMON_MEMBER_SET_GET(pe::Vector3, local_pos_a, LocalPosA)
        COMMON_MEMBER_SET_GET(pe::Vector3, local_pos_b, LocalPosB)

        COMMON_MEMBER_SET_GET(pe::Real, distance, Distance)
        COMMON_MEMBER_SET_GET(pe::Vector3, impulse, Impulse)

        pe::Array<pe::Vector3> _tangents;
    public:
        const pe::Array<pe::Vector3>& getTangents() const { return _tangents; }
        const pe::Vector3& getTangent(int index) const { return _tangents[index]; }

    private:
        static void getOrthoUnits(pe::Vector3 normal, pe::Vector3& tangent1, pe::Vector3& tangent2);

    public:
        ContactPoint();
        ContactPoint(const pe::Vector3& world_pos, const pe::Vector3& world_normal,
                     const pe::Vector3& local_pos_a, const pe::Vector3& local_pos_b);

        COMMON_FORCE_INLINE void invalidate() { _world_pos = PE_VEC_MAX; }
        COMMON_FORCE_INLINE bool isValid() const { return _world_pos.x == PE_REAL_MAX; }
    };

    class ContactResult {
    protected:
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::CollisionObject, body_a, BodyA)
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::CollisionObject, body_b, BodyB)

        COMMON_MEMBER_SET_GET(pe::Real, friction_coeff, FrictionCoeff)
        COMMON_MEMBER_SET_GET(pe::Real, restitution_coeff, RestitutionCoeff)

        COMMON_MEMBER_GET(int, point_size, PointSize)

        bool _swap_flag;
        ContactPoint _points[PE_CONTACT_MAX_POINTS];

    public:
        ContactResult();

        void setBodies(pe_phys_object::CollisionObject* body_a, pe_phys_object::CollisionObject* body_b);
        void swapBodies() { auto tmp = _body_a; _body_a = _body_b; _body_b = tmp; }

        void cleanContactPointFlag();
        void moveContactPointFlag(int index) { _points[index].invalidate(); }

        void addContactPoint(pe::Vector3 world_normal, pe::Vector3 world_pos, pe::Real depth);
        ContactPoint& getContactPoint(int index) { return _points[index]; }
        void editContactPoint(int index, pe::Vector3 normal, pe::Vector3 world_pos, pe::Real depth);
        void sortContactPoints();

    protected:
        int getExistingClosestPoint(const pe::Vector3& local_pos_b) const;
        pe::Real getSameContactPointDistanceThreshold() const;
    };

} // namespace pe_phys_collision