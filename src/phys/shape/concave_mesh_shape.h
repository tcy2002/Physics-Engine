#pragma once

#include "phys/phys_general.h"
#include "shape.h"

namespace pe_phys_shape {

    class ConcaveMeshShape: public Shape {
    protected:
        COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh)

    public:
        PE_API explicit ConcaveMeshShape(pe::Mesh mesh): _mesh(std::move(mesh)) {}
        virtual ~ConcaveMeshShape() override {}
        virtual ShapeType getType() const override { return ShapeType::ConcaveMesh; }
        virtual bool isConvex() const override { return false; }
        PE_API virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API virtual bool localIsInside(const pe::Vector3& point) const override;
        PE_API virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                    pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
        PE_API virtual pe::Matrix3 calcLocalInertia(pe::Real mass) const override;
    };

} // namespace pe_phys_shape