#pragma once

#include "shape.h"

namespace pe_phys_shape {

class BoxShape: public Shape {
    COMMON_MEMBER_SET_GET(pe::Vector3, size, Size);

private:
    pe::Vector3 _half_size;

public:
    explicit BoxShape(const pe::Vector3& size);
    ~BoxShape() override = default;
    ShapeType getType() const override { return ShapeType::BOX; }
    bool isConvex() const override { return true; }
    void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
    bool isInside(const pe::Transform& transform, const pe::Vector3& point) const override;
    void project(const pe::Transform& transform, const pe::Vector3& axis, pe::Real& min, pe::Real& max) const override;
};

} // namespace pe_phys_shape