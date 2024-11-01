#pragma once

#include "convex_mesh_shape.h"

namespace pe_phys_shape {

    class ConcaveMeshShape: public ConvexMeshShape {
    public:
        PE_API virtual void setMesh(pe::Mesh mesh) override;

        ConcaveMeshShape() {}
        virtual ~ConcaveMeshShape() override {}

        virtual ShapeType getType() const override { return ShapeType::ConcaveMesh; }
        virtual bool isConvex() const override { return false; }
    };

} // namespace pe_phys_shape