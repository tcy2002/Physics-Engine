#pragma once

#include "shape.h"

namespace pe_phys_shape {

class BoxShape: public Shape {
private:
    pe_cg::Vector3 _half_size;

public:
    explicit BoxShape(const pe_cg::Vector3& size);
    ~BoxShape() override = default;
    ShapeType getType() const override;
    bool isConvex() const override;
    void getAABB(const pe_cg::Transform& transform, pe_cg::Vector3& min, pe_cg::Vector3& max) const override;
    bool isInside(const pe_cg::Transform& transform, const pe_cg::Vector3& point) const override;
    void project(const pe_cg::Transform& transform, const pe_cg::Vector3& axis, real& min, real& max) const override;
};

} // namespace pe_phys_shape