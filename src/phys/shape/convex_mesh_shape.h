#pragma once

#include "phys/phys_general.h"
#include "shape.h"

namespace pe_phys_shape {

class ConvexMeshShape: public Shape {
    COMMON_MEMBER_SET_GET(pe::Mesh, mesh, Mesh);

public:
    explicit ConvexMeshShape(pe::Mesh mesh);
    ~ConvexMeshShape() override = default;
    ShapeType getType() const override { return ShapeType::MESH; }
    bool isConvex() const override { return true; }
    void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
    bool isInside(const pe::Transform& transform, const pe::Vector3& point) const override;
    void project(const pe::Transform& transform, const pe::Vector3& axis, pe::Real& min, pe::Real& max) const override;
};

} // namespace pe_phys_shape