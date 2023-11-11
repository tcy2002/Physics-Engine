#pragma once

#include "cg/vector3.h"
#include "cg/transform.h"
#include "cg/mesh.h"

namespace pe_phys_shape {

enum ShapeType {
    BOX, SPHERE, CYLINDER, MESH
};

class Shape {
protected:
    pe_cg::Mesh _mesh;
public:
    Shape() = default;
    virtual ~Shape() = default;
    virtual ShapeType getType() const = 0;
    virtual bool isConvex() const = 0;
    virtual void getAABB(const pe_cg::Transform &transform, pe_cg::Vector3 &min, pe_cg::Vector3 &max) const = 0;
    virtual bool isInside(const pe_cg::Transform &transform, const pe_cg::Vector3 &point) const = 0;
    virtual void project(const pe_cg::Transform &transform, const pe_cg::Vector3 &axis, real &min, real &max) const = 0;
    virtual const pe_cg::Mesh& getMesh() const { return _mesh; }
};

#define MAX_VEC(a, b) pe_cg::Vector3(std::max((a).x, (b).x), std::max((a).y, (b).y), std::max((a).z, (b).z))
#define MIN_VEC(a, b) pe_cg::Vector3(std::min((a).x, (b).x), std::min((a).y, (b).y), std::min((a).z, (b).z))

#define VEC_MAX pe_cg::Vector3(REAL_MAX, REAL_MAX, REAL_MAX)
#define VEC_MIN pe_cg::Vector3(REAL_MIN, REAL_MIN, REAL_MIN)

} // namespace pe_phys_shape