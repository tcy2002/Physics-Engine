#pragma once

#include "phys/phys_general.h"

namespace pe_phys_vehicle {
    
    class JacobianEntry {
    public:
        JacobianEntry() {}
        //constraint between two different rigidbodies
        JacobianEntry(
                const pe::Matrix3& world2A,
                const pe::Matrix3& world2B,
                const pe::Vector3& rel_pos1,
                const pe::Vector3& rel_pos2,
                const pe::Vector3& jointAxis,
                const pe::Vector3& inertiaInvA,
                pe::Real massInvA,
                const pe::Vector3& inertiaInvB,
                pe::Real massInvB);

        pe::Real getDiagonal() const { return m_Adiag; }

        // for two constraints on the same rigidbody (for example vehicle friction)
        pe::Real getNonDiagonal(const JacobianEntry& jacB, pe::Real massInvA) const;

        // for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
        pe::Real getNonDiagonal(const JacobianEntry& jacB, pe::Real massInvA, pe::Real massInvB) const;

        pe::Real getRelativeVelocity(const pe::Vector3& linvelA, const pe::Vector3& angvelA,
                                     const pe::Vector3& linvelB, const pe::Vector3& angvelB);

    private:
        pe::Vector3 m_linearJointAxis;
        pe::Vector3 m_aJ;
        pe::Vector3 m_bJ;
        pe::Vector3 m_0MinvJt;
        pe::Vector3 m_1MinvJt;
        //Optimization: can be stored in the w/last component of one of the vectors
        pe::Real m_Adiag;
    };
    
} // namespace pe_phys_vehicle