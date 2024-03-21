#pragma once

#include <common/vector3.h>
#include <common/matrix3x3.h>

#ifdef PE_USE_DOUBLE
#define JE_REAL double
#else
#define JE_REAL float
#endif

namespace utils {
    
    class JacobianEntry {
    public:
        JacobianEntry() {}
        //constraint between two different rigidbodies
        JacobianEntry(
                const common::Matrix3x3<JE_REAL>& world2A,
                const common::Matrix3x3<JE_REAL>& world2B,
                const common::Vector3<JE_REAL>& rel_pos1,
                const common::Vector3<JE_REAL>& rel_pos2,
                const common::Vector3<JE_REAL>& jointAxis,
                const common::Vector3<JE_REAL>& inertiaInvA,
                JE_REAL massInvA,
                const common::Vector3<JE_REAL>& inertiaInvB,
                JE_REAL massInvB);

        JE_REAL getDiagonal() const { return m_Adiag; }

        // for two constraints on the same rigidbody (for example vehicle friction)
        JE_REAL getNonDiagonal(const JacobianEntry& jacB, JE_REAL massInvA) const;

        // for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
        JE_REAL getNonDiagonal(const JacobianEntry& jacB, JE_REAL massInvA, JE_REAL massInvB) const;

        JE_REAL getRelativeVelocity(const common::Vector3<JE_REAL>& linvelA, const common::Vector3<JE_REAL>& angvelA,
                                    const common::Vector3<JE_REAL>& linvelB, const common::Vector3<JE_REAL>& angvelB);

    private:
        common::Vector3<JE_REAL> m_linearJointAxis;
        common::Vector3<JE_REAL> m_aJ;
        common::Vector3<JE_REAL> m_bJ;
        common::Vector3<JE_REAL> m_0MinvJt;
        common::Vector3<JE_REAL> m_1MinvJt;
        //Optimization: can be stored in the w/last component of one of the vectors
        JE_REAL m_Adiag;
    };
    
} // namespace utils