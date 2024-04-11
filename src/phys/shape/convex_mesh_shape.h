#pragma once

#include "phys/phys_general.h"
#include "shape.h"

namespace pe_phys_shape {

    class ConvexMeshShape: public Shape {
    protected:
        COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh)
        pe::Array<pe::Vector3> _unique_edges;
        pe::Array<pe::Vector3> _unique_verts;
        pe::Array<pe::Array<uint32_t>> _unique_faces;

    public:
        pe::Vector3 setMesh(pe::Mesh mesh);
        const pe::Array<pe::Vector3>& getUniqueEdges() const { return _unique_edges; }

        ConvexMeshShape() {}
        virtual ~ConvexMeshShape() override {}
        virtual ShapeType getType() const override { return ShapeType::ConvexMesh; }
        virtual bool isConvex() const override { return true; }
        PE_API virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API virtual bool localIsInside(const pe::Vector3& point) const override;
        PE_API virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                    pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
        PE_API virtual pe::Matrix3 calcLocalInertia(pe::Real mass) const override;

    public:
        PE_API static pe::Real calcMeshVolume(const pe::Mesh& mesh);
        PE_API static pe::Vector3 calcMeshCentroid(const pe::Mesh& mesh);
    };

} // namespace pe_phys_shape