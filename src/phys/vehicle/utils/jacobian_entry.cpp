#include "jacobian_entry.h"

namespace pe_phys_vehicle {

    JacobianEntry::JacobianEntry(
            const pe::Matrix3& world2A,
            const pe::Matrix3& world2B,
            const common::Vector3<pe::Real>& rel_pos1,
            const common::Vector3<pe::Real>& rel_pos2,
            const common::Vector3<pe::Real>& jointAxis,
            const common::Vector3<pe::Real>& inertiaInvA,
            const pe::Real massInvA,
            const common::Vector3<pe::Real>& inertiaInvB,
            const pe::Real massInvB) : m_linearJointAxis(jointAxis) {
        m_aJ = world2A * (rel_pos1.cross(m_linearJointAxis));
        m_bJ = world2B * (rel_pos2.cross(-m_linearJointAxis));
        m_0MinvJt = inertiaInvA.mult(m_aJ);
        m_1MinvJt = inertiaInvB.mult(m_bJ);
        m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);
    }

    pe::Real JacobianEntry::getNonDiagonal(const JacobianEntry& jacB, const pe::Real massInvA) const {
        const JacobianEntry& jacA = *this;
        pe::Real lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
        pe::Real ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
        return lin + ang;
    }

    pe::Real JacobianEntry::getNonDiagonal(const JacobianEntry& jacB, const pe::Real massInvA,
                                          const pe::Real massInvB) const {
        const JacobianEntry& jacA = *this;
        common::Vector3<pe::Real> lin = jacA.m_linearJointAxis.mult(jacB.m_linearJointAxis);
        common::Vector3<pe::Real> ang0 = jacA.m_0MinvJt.mult(jacB.m_aJ);
        common::Vector3<pe::Real> ang1 = jacA.m_1MinvJt.mult(jacB.m_bJ);
        common::Vector3<pe::Real> lin0 = massInvA * lin;
        common::Vector3<pe::Real> lin1 = massInvB * lin;
        common::Vector3<pe::Real> sum = ang0 + ang1 + lin0 + lin1;
        return sum[0] + sum[1] + sum[2];
    }

    pe::Real JacobianEntry::getRelativeVelocity(const common::Vector3<pe::Real>& linvelA,
                                               const common::Vector3<pe::Real>& angvelA,
                                               const common::Vector3<pe::Real>& linvelB,
                                               const common::Vector3<pe::Real>& angvelB) {
        common::Vector3<pe::Real> linrel = linvelA - linvelB;
        common::Vector3<pe::Real> angvela = angvelA.mult(m_aJ);
        common::Vector3<pe::Real> angvelb = angvelB.mult(m_bJ);
        linrel = linrel.mult(m_linearJointAxis);
        angvela += angvelb;
        angvela += linrel;
        pe::Real rel_vel2 = angvela[0] + angvela[1] + angvela[2];
        return rel_vel2 + 1e-7f;
    }

} // namespace pe_phys_vehicle