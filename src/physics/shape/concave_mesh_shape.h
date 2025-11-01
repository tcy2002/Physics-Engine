#pragma once

#include "convex_mesh_shape.h"

namespace pe_physics_shape {

class ConcaveMeshShape: public ConvexMeshShape {
public:
    PE_API pe::Vector3 setMesh(pe::Mesh mesh) override;

    ConcaveMeshShape() = default;
    virtual ~ConcaveMeshShape() = default;

    ShapeType getType() const override { return ShapeType::ST_ConcaveMesh; }
    bool isConvex() const override { return false; }
    bool localIsInside(const pe::Vector3 &point) const override { return false; } // does not support
};

} // namespace pe_physics_shape
