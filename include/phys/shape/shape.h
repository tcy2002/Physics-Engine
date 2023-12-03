#pragma once

#include "common/vector3.h"
#include "common/transform.h"
#include "common/mesh.h"

namespace pe_phys_shape {

enum ShapeType {
    BOX, SPHERE, CYLINDER, MESH
};

class Shape {
protected:
    pe_common::Mesh _mesh;
public:
    Shape() = default;
    virtual ~Shape() = default;
    virtual ShapeType getType() const = 0;
    virtual bool isConvex() const = 0;
    virtual void getAABB(const pe_common::Transform &transform, pe_common::Vector3 &min, pe_common::Vector3 &max) const = 0;
    virtual bool isInside(const pe_common::Transform &transform, const pe_common::Vector3 &point) const = 0;
    virtual void project(const pe_common::Transform &transform, const pe_common::Vector3 &axis, PEReal &min, PEReal &max) const = 0;
    virtual const pe_common::Mesh& getMesh() const { return _mesh; }
};

#define PE_MAX_VEC(a, b) pe_common::Vector3(std::max((a).x, (b).x), std::max((a).y, (b).y), std::max((a).z, (b).z))
#define PE_MIN_VEC(a, b) pe_common::Vector3(std::min((a).x, (b).x), std::min((a).y, (b).y), std::min((a).z, (b).z))

#define PE_VEC_MAX pe_common::Vector3(PE_REAL_MAX, PE_REAL_MAX, PE_REAL_MAX)
#define PE_VEC_MIN pe_common::Vector3(PE_REAL_MIN, PE_REAL_MIN, PE_REAL_MIN)

} // namespace pe_phys_shape