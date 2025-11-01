#pragma once

#include "shape.h"

namespace pe_physics_shape {

class BoxShape: public Shape {
    COMMON_MEMBER_GET(pe::Vector3, size, Size);
    COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh);

protected:
    pe::Vector3 _half_size;
    pe::Array<UniqueEdge> _unique_edges;

public:
    const pe::Array<UniqueEdge>& getUniqueEdges() const { return _unique_edges; }

public:
    PE_API explicit BoxShape(const pe::Vector3& size);
    virtual ~BoxShape() = default;
    ShapeType getType() const override { return ShapeType::ST_Box; }
    bool isConvex() const override { return true; }
    PE_API void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
    PE_API bool localIsInside(const pe::Vector3& point) const override;
    PE_API pe::Vector3 getLocalSupportPoint(const pe::Vector3 &dir) const override;
    PE_API void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                        pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
};

} // namespace pe_physics_shape
