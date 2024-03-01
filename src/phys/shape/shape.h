#pragma once

#include "phys/phys_general.h"

namespace pe_phys_shape {

    enum ShapeType {
        Box, Sphere, Cylinder, ConvexMesh
    };

    class Shape {
    public:
        Shape() = default;
        virtual ~Shape() = default;
        virtual ShapeType getType() const = 0;
        virtual bool isConvex() const = 0;
        virtual void getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const = 0;
        virtual bool isInside(const pe::Transform &transform, const pe::Vector3 &point) const = 0;
        virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                             pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const = 0;
    };

} // namespace pe_phys_shape