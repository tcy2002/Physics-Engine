#pragma once

#include "phys/phys_general.h"
#include "shape.h"
#include "utils/bvh.h"

namespace pe_phys_collision {
    class ConcaveConvexCollisionAlgorithm;
}

namespace pe_phys_shape {

    class ConvexMeshShape: public Shape {
        friend class pe_phys_collision::ConcaveConvexCollisionAlgorithm;

    protected:
        COMMON_MEMBER_GET(pe::Mesh, mesh, Mesh)
        COMMON_MEMBER_SET_GET(std::string, mesh_path, MeshPath)
        COMMON_MEMBER_SET_GET(pe::Vector3, scale, Scale) // scale relative to the original mesh from the obj file
    protected:
        utils::BVH _bvh;
        pe::Array<pe::KV<pe::Vector3, pe::Vector3>> _unique_edges;

    public:
        PE_API virtual pe::Vector3 setMesh(pe::Mesh mesh);
        const pe::Array<pe::KV<pe::Vector3, pe::Vector3>>& getUniqueEdges() const { return _unique_edges; }
        void getIntersectFaces(const pe::Vector3& AA, const pe::Vector3& BB, pe::Array<int>& intersect) const;

        ConvexMeshShape() {}
        virtual ~ConvexMeshShape() override {}
        virtual ShapeType getType() const override { return ShapeType::ConvexMesh; }
        virtual bool isConvex() const override { return true; }
        PE_API virtual void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API virtual bool localIsInside(const pe::Vector3& point) const override;
        PE_API virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                    pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
    };

} // namespace pe_phys_shape