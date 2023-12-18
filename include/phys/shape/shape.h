#pragma once

#include "phys/phys_general.h"

namespace pe_phys_shape {

enum ShapeType {
    BOX, SPHERE, CYLINDER, MESH
};

class Shape {
public:
    Shape() = default;
    virtual ~Shape() = default;
    virtual ShapeType getType() const = 0;
    virtual bool isConvex() const = 0;
    virtual void getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const = 0;
    virtual bool isInside(const pe::Transform &transform, const pe::Vector3 &point) const = 0;
    virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &min, pe::Real &max) const = 0;
};

#define PE_MAX_VEC(a, b) pe::Vector3(std::max((a).x, (b).x), std::max((a).y, (b).y), std::max((a).z, (b).z))
#define PE_MIN_VEC(a, b) pe::Vector3(std::min((a).x, (b).x), std::min((a).y, (b).y), std::min((a).z, (b).z))

#define PE_VEC_MAX pe::Vector3(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX)
#define PE_VEC_MIN pe::Vector3(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN)

} // namespace pe_phys_shape