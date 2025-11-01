#pragma once

#include "physics/physics.h"

namespace pe_physics_shape {

enum class ShapeType {
    ST_Box, ST_Sphere, ST_ConvexMesh, ST_ConcaveMesh, ST_Capsule, ST_Compound
};

// An edge that is shared by only two faces
struct UniqueEdge {
    pe::Vector3 start;
    pe::Vector3 end;
    int face1_id = -1;
    int face2_id = -1;
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
    virtual ~Shape() = default;
    virtual ShapeType getType() const = 0;
    virtual bool isConvex() const = 0;
    virtual void getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const = 0;
    virtual bool localIsInside(const pe::Vector3 &point) const = 0;
    virtual pe::Vector3 getLocalSupportPoint(const pe::Vector3 &dir) const = 0;
    virtual void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                         pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const = 0;
};

} // namespace pe_physics_shape