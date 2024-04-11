#pragma once

#include "shape.h"

namespace pe_phys_shape {

    class CylinderShape: public Shape {
        COMMON_MEMBER_GET(pe::Real, radius, Radius);
        COMMON_MEMBER_GET(pe::Real, height, Height);
        COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh);

    public:
        PE_API CylinderShape(pe::Real radius, pe::Real height);
        virtual ~CylinderShape() override {}
        virtual ShapeType getType() const override { return ShapeType::Cylinder; }
        virtual bool isConvex() const override { return true; }
        PE_API virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API virtual bool localIsInside(const pe::Vector3& point) const override;
        PE_API virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                    pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
        PE_API virtual pe::Matrix3 calcLocalInertia(pe::Real mass) const override;
    };

} // namespace pe_phys_shape