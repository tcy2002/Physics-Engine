#pragma once

#include "shape.h"

namespace pe_phys_shape {

class BoxShape: public Shape {
private:
    pe_common::Vector3 _half_size;

public:
    explicit BoxShape(const pe_common::Vector3& size);
    ~BoxShape() override = default;
    ShapeType getType() const override;
    bool isConvex() const override;
    void getAABB(const pe_common::Transform& transform, pe_common::Vector3& min, pe_common::Vector3& max) const override;
    bool isInside(const pe_common::Transform& transform, const pe_common::Vector3& point) const override;
    void project(const pe_common::Transform& transform, const pe_common::Vector3& axis, PEReal& min, PEReal& max) const override;
};

} // namespace pe_phys_shape