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
        COMMON_MEMBER_SET_GET(pe::Vector3, applied_impulse, AppliedImpulse)

        pe::Array<pe::Vector3> _tangents;
    public:
        const pe::Array<pe::Vector3>& getTangents() const { return _tangents; }
        const pe::Vector3& getTangent(int index) const { return _tangents[index]; }

    private:
        static void getOrthoUnits(pe::Vector3 normal, pe::Vector3& tangent1, pe::Vector3& tangent2);

    public:
        ContactPoint();
        ContactPoint(const pe::Vector3& world_pos, const pe::Vector3& world_normal,
                     const pe::Vector3& local_pos_a, const pe::Vector3& local_pos_b, pe::Real distance);

        COMMON_FORCE_INLINE void invalidate() { _world_pos = PE_VEC_MAX; }
        COMMON_FORCE_INLINE bool isValid() const { return _world_pos.x != PE_REAL_MAX; }
    };

    class ContactResult {
    protected:
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::CollisionObject, object_a, ObjectA)
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::CollisionObject, object_b, ObjectB)

        COMMON_MEMBER_SET_GET(pe::Real, friction_coeff, FrictionCoeff)
        COMMON_MEMBER_SET_GET(pe::Real, restitution_coeff, RestitutionCoeff)

        COMMON_MEMBER_GET(int, point_size, PointSize)
        COMMON_MEMBER_SET_GET(bool, swap_flag, SwapFlag)

        ContactPoint _points[PE_CONTACT_MAX_POINTS];

    public:
        ContactResult();

        void setObjects(pe_phys_object::CollisionObject* object_a, pe_phys_object::CollisionObject* object_b);
        void swapObjects() { std::swap(_object_a, _object_b); }

        void cleanContactPointFlag();
        void moveContactPointFlag(int index) { _points[index].invalidate(); }

        void addContactPoint(const pe::Vector3& world_normal, const pe::Vector3& world_pos, pe::Real depth);
        ContactPoint& getContactPoint(int index) { return _points[index]; }
        void editContactPoint(int index, pe::Vector3 normal, pe::Vector3 world_pos, pe::Real distance);
        void sortContactPoints();

    protected:
        int getExistingClosestPoint(const pe::Vector3& local_pos_b) const;
        pe::Real getSameContactPointDistanceThreshold() const;
    };

} // namespace pe_phys_collision