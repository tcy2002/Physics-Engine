#pragma once

#include "phys/phys_general.h"

namespace pe_phys_shape {

    enum ShapeType {
        Box = 0, Sphere = 1, Cylinder = 2, ConvexMesh = 3, ConcaveMesh = 4, Compound = 5
    };

    class Shape {
        COMMON_MEMBER_GET(uint32_t, global_id, GlobalId)

    protected:
        static std::atomic<uint32_t> _globalIdCounter;
        pe::Matrix3 _local_inertia; // assume the density is 1 and the center of mass is at the origin
        pe::Real _volume;

    public:
        const pe::Matrix3& getLocalInertia() const { return _local_inertia; }
        pe::Real getVolume() const { return _volume; }

        PE_API Shape();
        virtual ~Shape() {}
        virtual ShapeType getType() const = 0;
        virtual bool isConvex() const = 0;
        virtual void getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const = 0;
        virtual bool localIsInside(const pe::Vector3 &point) const = 0;
        virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                             pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const = 0;
    };

} // namespace pe_phys_shape