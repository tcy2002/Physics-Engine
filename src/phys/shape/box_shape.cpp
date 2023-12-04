#include "box_shape.h"

using namespace pe_phys_shape;

BoxShape::BoxShape(const pe::Vector3& size):
    _half_size(size / 2) {
    pe::Real x = _half_size.x, y = _half_size.y, z = _half_size.z;
    _mesh = {
            {
                    {{-x,  y, -z}, {0, 1, 0}},
                    {{-x,  y,  z}, {0, 1, 0}},
                    {{ x,  y,  z}, {0, 1, 0}},
                    {{ x,  y, -z}, {0, 1, 0}},
                    {{-x, -y,  z}, {0, -1, 0}},
                    {{-x, -y, -z}, {0, -1, 0}},
                    {{ x, -y, -z}, {0, -1, 0}},
                    {{ x, -y,  z}, {0, -1, 0}},
                    {{-x,  y,  z}, {0, 0, 1}},
                    {{-x, -y,  z}, {0, 0, 1}},
                    {{ x, -y,  z}, {0, 0, 1}},
                    {{ x,  y,  z}, {0, 0, 1}},
                    {{ x,  y, -z}, {0, 0, -1}},
                    {{ x, -y, -z}, {0, 0, -1}},
                    {{-x, -y, -z}, {0, 0, -1}},
                    {{-x,  y, -z}, {0, 0, -1}},
                    {{-x,  y, -z}, {-1, 0, 0}},
                    {{-x, -y, -z}, {-1, 0, 0}},
                    {{-x, -y,  z}, {-1, 0, 0}},
                    {{-x,  y,  z}, {-1, 0, 0}},
                    {{ x,  y,  z}, {1, 0, 0}},
                    {{ x, -y,  z}, {1, 0, 0}},
                    {{ x, -y, -z}, {1, 0, 0}},
                    {{ x,  y, -z}, {1, 0, 0}}
            },
            {
                    {{0, 1, 2, 3}, {0, 1, 0}},
                    {{4, 5, 6, 7 }, {0, -1, 0}},
                    {{8, 9, 10, 11 }, {0, 0, 1}},
                    {{12, 13, 14, 15 }, {0, 0, -1}},
                    {{16, 17, 18, 19 }, {-1, 0, 0}},
                    {{20, 21, 22, 23 }, {1, 0, 0}}
            }
    };
}

ShapeType BoxShape::getType() const {
    return ShapeType::BOX;
}

bool BoxShape::isConvex() const {
    return true;
}

void BoxShape::getAABB(const pe::Transform& transform, pe::Vector3& min, pe::Vector3& max) const {
    min = PE_VEC_MAX;
    max = PE_VEC_MIN;
    auto& rot = transform.getBasis();
    auto& pos = transform.getOrigin();
    pe::Vector3 p;
    for (int i = 0; i < 8; i++) {
        p.x = i & 1 ? _half_size.x : -_half_size.x;
        p.y = i & 2 ? _half_size.y : -_half_size.y;
        p.z = i & 4 ? _half_size.z : -_half_size.z;
        auto v = rot * p;
        min = PE_MIN_VEC(min, v);
        max = PE_MAX_VEC(max, v);
    }
    min += pos;
    max += pos;
}

bool BoxShape::isInside(const pe::Transform& transform, const pe::Vector3& point) const {
    auto local_pos = transform.inverseTransform(point);
    return std::abs(local_pos.x) <= _half_size.x &&
           std::abs(local_pos.y) <= _half_size.y &&
           std::abs(local_pos.z) <= _half_size.z;
}

void BoxShape::project(const pe::Transform& transform, const pe::Vector3& axis, pe::Real& min, pe::Real& max) const {
    auto local_axis = transform.getBasis().transposed() * axis;
    pe::Real offset = transform.getOrigin().dot(axis);
    pe::Vector3 ext;
    ext.x = local_axis.x > 0 ? _half_size.x : -_half_size.x;
    ext.y = local_axis.y > 0 ? _half_size.y : -_half_size.y;
    ext.z = local_axis.z > 0 ? _half_size.z : -_half_size.z;
    pe::Real half_ext = ext.dot(local_axis);
    min = offset - half_ext;
    max = offset + half_ext;
}
