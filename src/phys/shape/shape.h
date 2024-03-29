#pragma once

#include "phys/phys_general.h"

namespace pe_phys_shape {

    enum ShapeType {
        Box = 0, Sphere = 1, Cylinder = 2, ConvexMesh = 3, ConcaveMesh = 4
    };

    class Shape {
    public:
        Shape() = default;
        virtual ~Shape() = default;
        virtual ShapeType getType() const = 0;
        virtual bool isConvex() const = 0;
        virtual void getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const = 0;
        virtual bool localIsInside(const pe::Vector3 &point) const = 0;
        virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                             pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const = 0;
        virtual pe::Matrix3 calcLocalInertia(pe::Real mass) const = 0;
    };

} // namespace pe_phys_shape