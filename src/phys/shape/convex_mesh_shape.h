#pragma once

#include "phys/phys_general.h"
#include "shape.h"

namespace pe_phys_shape {

    class ConvexMeshShape: public Shape {
        COMMON_MEMBER_SET_GET(pe::Mesh, mesh, Mesh);

    public:
        explicit ConvexMeshShape(pe::Mesh mesh);
        virtual ~ConvexMeshShape() override = default;
        virtual ShapeType getType() const override { return ShapeType::ConvexMesh; }
        virtual bool isConvex() const override { return true; }
        virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        virtual bool isInside(const pe::Transform& transform, const pe::Vector3& point) const override;
        virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                             pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
    };

} // namespace pe_phys_shape