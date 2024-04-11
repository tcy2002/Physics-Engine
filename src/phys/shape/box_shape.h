#pragma once

#include "shape.h"

namespace pe_phys_shape {

    class BoxShape: public Shape {
        COMMON_MEMBER_GET(pe::Vector3, size, Size);
        COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh);

    private:
        pe::Vector3 _half_size;

    public:
        PE_API explicit BoxShape(const pe::Vector3& size);
        virtual ~BoxShape() override {}
        virtual ShapeType getType() const override { return ShapeType::Box; }
        virtual bool isConvex() const override { return true; }
        PE_API virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API virtual bool localIsInside(const pe::Vector3& point) const override;
        PE_API virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                    pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
        PE_API virtual pe::Matrix3 calcLocalInertia(pe::Real mass) const override;
    };

} // namespace pe_phys_shape