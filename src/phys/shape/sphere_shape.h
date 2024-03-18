#pragma once

#include "shape.h"

namespace pe_phys_shape {

    class SphereShape: public Shape {
        COMMON_MEMBER_GET(pe::Real, radius, Radius);

    public:
        explicit SphereShape(pe::Real radius);
        virtual ~SphereShape() override = default;
        virtual ShapeType getType() const override { return ShapeType::Sphere; }
        virtual bool isConvex() const override { return true; }
        virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        virtual bool localIsInside(const pe::Vector3& point) const override;
        virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                             pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
        virtual pe::Matrix3 calcLocalInertia(pe::Real mass) const override;
    };

} // namespace pe_phys_shape