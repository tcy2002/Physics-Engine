#pragma once

#include "shape.h"

namespace pe_phys_shape {

    class BoxShape: public Shape {
        COMMON_MEMBER_SET_GET(pe::Vector3, size, Size);

    private:
        pe::Vector3 _half_size;

    public:
        explicit BoxShape(const pe::Vector3& size);
        virtual ~BoxShape() override = default;
        virtual ShapeType getType() const override { return ShapeType::Box; }
        virtual bool isConvex() const override { return true; }
        virtual pe::Vector3 localGetSupportVertex(const pe::Vector3 &dir) const override;
        virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        virtual bool localIsInside(const pe::Vector3& point) const override;
        virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                             pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
    };

} // namespace pe_phys_shape