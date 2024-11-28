#pragma once

#include "convex_mesh_shape.h"

namespace pe_phys_shape {

    class ConcaveMeshShape: public ConvexMeshShape {
    public:
        PE_API pe::Vector3 setMesh(pe::Mesh mesh) override;

        ConcaveMeshShape() {}
        virtual ~ConcaveMeshShape() {}

        ShapeType getType() const override { return ShapeType::ST_ConcaveMesh; }
        bool isConvex() const override { return false; }
    };

} // namespace pe_phys_shape