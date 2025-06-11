#pragma once

#include "shape.h"

namespace pe_phys_shape {

    class CylinderShape: public Shape {
        COMMON_MEMBER_GET(pe::Real, radius, Radius);
        COMMON_MEMBER_GET(pe::Real, height, Height);
        COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh);

    protected:
        pe::Array<pe::KV<pe::Vector3, pe::Vector3>> _unique_edges;

    public:
        const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& getUniqueEdges() const { return _unique_edges; }

    public:
        PE_API CylinderShape(pe::Real radius, pe::Real height);
        virtual ~CylinderShape() {}
        ShapeType getType() const override { return ShapeType::ST_Cylinder; }
        bool isConvex() const override { return true; }
        PE_API void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API bool localIsInside(const pe::Vector3& point) const override;
        PE_API void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                            pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
    };

} // namespace pe_phys_shape