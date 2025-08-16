#pragma once

#include "rigid/phys_general.h"
#include "rigid/object/rigidbody.h"

#define PE_CONTACT_CACHE_SIZE 8

namespace pe_phys_collision {

    class ContactPoint {
    protected:
        COMMON_MEMBER_SET_GET(pe::Vector3, world_pos, WorldPos)
        COMMON_MEMBER_SET_GET(pe::Vector3, world_normal, WorldNormal)

        COMMON_MEMBER_SET_GET(pe::Vector3, local_pos_a, LocalPosA)
        COMMON_MEMBER_SET_GET(pe::Vector3, local_pos_b, LocalPosB)

        COMMON_MEMBER_SET_GET(pe::Real, distance, Distance)

        pe::Vector3 _tangents[2];
    public:
        const pe::Vector3& getTangent(int index) const { return _tangents[index]; }

        // for primal-dual
    protected:
        COMMON_MEMBER_SET_GET(pe::Vector3, world_pos_half, WorldPosHalf)
        COMMON_MEMBER_SET_GET(pe::Real, distance_non_neg, DistanceNonNeg)
        pe::MatrixMN trans1;
        pe::MatrixMN trans2;
    public:
        pe::Vector3 toLocal(bool t, const pe::Vector6& global_quant) const {
            return -(t ? trans2 : trans1).transpose() * global_quant;
        }
        pe::Vector3 toLocal(const pe::Vector6& global_quant1, const pe::Vector6& global_quant2) const {
            return -trans1.transpose() * global_quant1 -
                trans2.transpose() * global_quant2;
        }
        pe::Real toNormal(const pe::Vector6& global_quant1, const pe::Vector6& global_quant2) const {
            return -trans1.col(0).dot(global_quant1) -
                trans2.col(0).dot(global_quant2);
        }
        pe::Vector2 toTangent(const pe::Vector6& global_quant1, const pe::Vector6& global_quant2) const {
            return -trans1.block<6, 2>(0, 1).transpose() * global_quant1 -
                trans2.block<6, 2>(0, 1).transpose() * global_quant2;
        }
        pe::Vector6 toGlobal(bool t, const pe::Vector3& local_quant) const {
            return -(t ? trans2 : trans1) * local_quant;
        }
        pe::Vector6 toGlobalNormal(bool t, const pe::Real& local_quant) const {
            return -(t ? trans2 : trans1).col(0) * local_quant;
        }
        pe::Vector6 toGlobalTangent(bool t, const pe::Vector2& local_quant) const {
            return -(t ? trans2 : trans1).block<6, 2>(0, 1) * local_quant;
        }
        Eigen::Matrix<pe::Real, 6, 3> getTrans(bool t) {
            return -(t ? trans2 : trans1);
        }
        pe::Vector6 getTransNormal(bool t) const {
            return -(t ? trans2 : trans1).col(0);
        }
        Eigen::Matrix<pe::Real, 6, 2> getTransTangent(bool t) const {
            return -(t ? trans2 : trans1).block<6, 2>(0, 1);
        }

        static void getOrthoUnits(pe::Vector3 normal, pe::Vector3& tangent1, pe::Vector3& tangent2);

        ContactPoint();
        ContactPoint(const pe::Vector3& world_pos, const pe::Vector3& world_normal,
                     const pe_phys_object::RigidBody* object_a, const pe_phys_object::RigidBody* object_b,
                     const pe::Vector3& world_vel_a, const pe::Vector3& world_vel_b,
                     const pe::Vector3& local_pos_a, const pe::Vector3& local_pos_b, pe::Real distance);

        COMMON_FORCE_INLINE void invalidate() { _world_pos = PE_VEC_MAX; }
        COMMON_FORCE_INLINE bool isValid() const { return _world_pos.x() < PE_REAL_MAX + 0.1; }
    };

    class ContactResult {
    protected:
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::RigidBody, object_a, ObjectA)
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::RigidBody, object_b, ObjectB)

        COMMON_MEMBER_SET_GET(pe::Real, friction_coeff, FrictionCoeff)
        COMMON_MEMBER_SET_GET(pe::Real, restitution_coeff, RestitutionCoeff)

        COMMON_MEMBER_GET(int, point_size, PointSize)
        COMMON_MEMBER_SET_GET(bool, swap_flag, SwapFlag)

        ContactPoint _points[PE_CONTACT_CACHE_SIZE];

    public:
        ContactResult();

        void setObjects(pe_phys_object::RigidBody* object_a, pe_phys_object::RigidBody* object_b);

        void addContactPoint(const pe::Vector3& world_normal, const pe::Vector3& world_pos, pe::Real depth);
        ContactPoint& getContactPoint(int index) { return _points[index]; }
        const ContactPoint& getContactPoint(int index) const { return _points[index]; }
        void sortContactPoints();
        void clearContactPoints();

    protected:
        int getExistingClosestPoint(const pe::Vector3& local_pos_b) const;
        pe::Real getSameContactPointDistanceThreshold() const;
    };

} // namespace pe_phys_collision