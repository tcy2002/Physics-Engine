#include "compound_shape.h"
#include <algorithm>

namespace pe_phys_shape {

    void CompoundShape::addShape(const pe::Transform& pos, pe::Real massRatio, Shape *shape) {
        if (shape->getType() == ShapeType::Compound) {
            throw std::runtime_error("CompoundShape cannot contain another CompoundShape");
        }
        _shapes.push_back({pos, massRatio, shape});
        _mass_ratio += massRatio;
    }

    void CompoundShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        for (auto& s: _shapes) {
            pe::Vector3 s_min, s_max;
            s.shape->getAABB(transform * s.local_transform, s_min, s_max);
            min = pe::Vector3::min2(min, s_min);
            max = pe::Vector3::max2(max, s_max);
        }
    }

    bool CompoundShape::localIsInside(const pe::Vector3 &point) const {
        return std::any_of(_shapes.begin(), _shapes.end(), [&](auto &s) {
            pe::Vector3 local_point = s.local_transform.inverseTransform(point);
            return s.shape->localIsInside(local_point);
        });
    }

    void CompoundShape::project(const pe::Transform &transform, const pe::Vector3 &axis, pe::Real &minProj,
                                pe::Real &maxProj, pe::Vector3& minPoint, pe::Vector3& maxPoint) const {
        minProj = PE_REAL_MAX;
        maxProj = PE_REAL_MIN;
        for (auto& s: _shapes) {
            pe::Real s_min, s_max;
            pe::Vector3 s_min_point, s_max_point;
            s.shape->project(transform * s.local_transform, axis, s_min, s_max,
                             s_min_point, s_max_point);
            if (s_min < minProj) {
                minProj = s_min;
                minPoint = s_min_point;
            }
            if (s_max > maxProj) {
                maxProj = s_max;
                maxPoint = s_max_point;
            }
        }
    }

    pe::Matrix3 CompoundShape::calcLocalInertia(pe::Real mass) const {
        pe::Matrix3 inertia;
        for (auto& s: _shapes) {
            pe::Matrix3 s_inertia = s.shape->calcLocalInertia(mass * s.mass_ratio / _mass_ratio);
            pe::Matrix3 t_inertia = pe::Matrix3::identity();
            pe::Vector3 pos = s.local_transform.getOrigin();
            pe::Matrix3 rot = s.local_transform.getBasis();
            t_inertia[0][0] = pos[0] * pos[0];
            t_inertia[1][1] = pos[1] * pos[1];
            t_inertia[2][2] = pos[2] * pos[2];
            inertia += (rot * s_inertia * rot.transposed() + t_inertia);
        }
        return inertia;
    }
}