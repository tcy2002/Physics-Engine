#include "jacobian_entry.h"

namespace pe_phys_vehicle {

    JacobianEntry::JacobianEntry(
            const pe::Matrix3& world2A,
            const pe::Matrix3& world2B,
            const pe::Vector3& rel_pos1,
            const pe::Vector3& rel_pos2,
            const pe::Vector3& jointAxis,
            const pe::Vector3& inertiaInvA,
            const pe::Real massInvA,
            const pe::Vector3& inertiaInvB,
            const pe::Real massInvB) : m_linearJointAxis(jointAxis) {
        m_aJ = world2A * (rel_pos1.cross(m_linearJointAxis));
        m_bJ = world2B * (rel_pos2.cross(-m_linearJointAxis));
        m_0MinvJt = inertiaInvA.cwiseProduct(m_aJ);
        m_1MinvJt = inertiaInvB.cwiseProduct(m_bJ);
        m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);
    }

    pe::Real JacobianEntry::getNonDiagonal(const JacobianEntry& jacB, const pe::Real massInvA) const {
        const JacobianEntry& jacA = *this;
        const pe::Real lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
        const pe::Real ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
        return lin + ang;
    }

    pe::Real JacobianEntry::getNonDiagonal(const JacobianEntry& jacB, const pe::Real massInvA,
                                          const pe::Real massInvB) const {
        const JacobianEntry& jacA = *this;
        const pe::Vector3 lin = jacA.m_linearJointAxis.cwiseProduct(jacB.m_linearJointAxis);
        const pe::Vector3 ang0 = jacA.m_0MinvJt.cwiseProduct(jacB.m_aJ);
        const pe::Vector3 ang1 = jacA.m_1MinvJt.cwiseProduct(jacB.m_bJ);
        const pe::Vector3 lin0 = massInvA * lin;
        const pe::Vector3 lin1 = massInvB * lin;
        const pe::Vector3 sum = ang0 + ang1 + lin0 + lin1;
        return sum[0] + sum[1] + sum[2];
    }

    pe::Real JacobianEntry::getRelativeVelocity(const pe::Vector3& lin_vel_a, const pe::Vector3& ang_vel_a,
                                                const pe::Vector3& lin_vel_b, const pe::Vector3& ang_Vel_b) const {
        pe::Vector3 lin_rel = lin_vel_a - lin_vel_b;
        pe::Vector3 ang_vel_a1 = ang_vel_a.cwiseProduct(m_aJ);
        const pe::Vector3 ang_vel_b1 = ang_Vel_b.cwiseProduct(m_bJ);
        lin_rel = lin_rel.cwiseProduct(m_linearJointAxis);
        ang_vel_a1 += ang_vel_b1;
        ang_vel_a1 += lin_rel;
        const pe::Real rel_vel2 = ang_vel_a1[0] + ang_vel_a1[1] + ang_vel_a1[2];
        return rel_vel2 + 1e-7f;
    }

} // namespace pe_phys_vehicle