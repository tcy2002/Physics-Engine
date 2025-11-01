#pragma once

#include "physics/physics.h"
#include "shape.h"

namespace pe_physics_shape {

class CompoundShape: public Shape {
protected:
    struct SubShape {
        pe::Transform local_transform;
        pe::Real mass_ratio;
        Shape* shape;
    };
    pe::Array<SubShape> _shapes;
    pe::Real _total_mass = 0;

public:
    PE_API void addShape(const pe::Transform& pos, pe::Real massRatio, Shape* shape);
    PE_API const pe::Array<SubShape>& getShapes() const { return _shapes; }
    PE_API void clearShapes() { _shapes.clear(); _total_mass = 0; }
    PE_API pe::Vector3 init();

    CompoundShape() = default;
    virtual ~CompoundShape() = default;
    ShapeType getType() const override { return ShapeType::ST_Compound; }
    bool isConvex() const override { return false; }
    PE_API void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
    PE_API bool localIsInside(const pe::Vector3& point) const override;
    PE_API pe::Vector3 getLocalSupportPoint(const pe::Vector3 &dir) const override;
    PE_API void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                        pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
};

} // namespace pe_physics_shape
