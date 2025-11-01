#pragma once

#include "shape.h"

namespace pe_physics_shape {

class SphereShape: public Shape {
    COMMON_MEMBER_GET(pe::Real, radius, Radius);

public:
    PE_API explicit SphereShape(pe::Real radius);
    virtual ~SphereShape() = default;
    ShapeType getType() const override { return ShapeType::ST_Sphere; }
    bool isConvex() const override { return true; }
    PE_API void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
    PE_API bool localIsInside(const pe::Vector3& point) const override;
    PE_API pe::Vector3 getLocalSupportPoint(const pe::Vector3 &dir) const override;
    PE_API void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                        pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
};

} // namespace pe_physics_shape