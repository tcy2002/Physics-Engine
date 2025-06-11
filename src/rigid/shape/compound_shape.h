#pragma once

#include "rigid/phys_general.h"
#include "shape.h"

namespace pe_phys_shape {

    class CompoundShape: public Shape {
    protected:
        struct ShapeEle {
            pe::Transform local_transform;
            pe::Real mass_ratio;
            Shape* shape;
        };
        pe::Array<ShapeEle> _shapes;
        pe::Real _mass_ratio = 0;

    public:
        PE_API void addShape(const pe::Transform& pos, pe::Real massRatio, Shape* shape);
        PE_API const pe::Array<ShapeEle>& getShapes() const { return _shapes; }
        PE_API void clearShapes() { _shapes.clear(); _mass_ratio = 0; }

        CompoundShape() {}
        virtual ~CompoundShape() {}
        ShapeType getType() const override { return ShapeType::ST_Compound; }
        bool isConvex() const override { return false; }
        PE_API void getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const override;
        PE_API bool localIsInside(const pe::Vector3& point) const override;
        PE_API void project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                            pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const override;
    };

} // namespace pe_phys_shape