#pragma once

#include "phys/phys_general.h"

namespace pe_phys_object {

    class Terrain {
        COMMON_MEMBER_SET_GET(pe::Vector3, scale, Scale)
        COMMON_MEMBER_SET_GET(pe::Vector3, origin, Origin)

        COMMON_MEMBER_SET_GET(pe::Real, baseFrictionCoeff, BaseFrictionCoeff)
        COMMON_MEMBER_SET_GET(pe::Real, baseRestitutionCoeff, BaseRestitutionCoeff)

    public:
        Terrain():
            _scale(1, 1, 1),
            _origin(0, 0, 0),
            _baseFrictionCoeff(0.8),
            _baseRestitutionCoeff(0.5) {}
        virtual ~Terrain();

        virtual pe::Real getHeightAt(const pe::Vector3& pos) const { return pe::Real(0.0); }
        virtual pe::Vector3 getNormalAt(const pe::Vector3& pos) const { return pe::Vector3::up(); }
        virtual pe::Real getFrictionCoeffAt(const pe::Vector3& pos) const { return _baseFrictionCoeff; }
        virtual pe::Real getRestitutionCoeffAt(const pe::Vector3& pos) const { return _baseRestitutionCoeff; }
    };

} // namespace pe_phys_object