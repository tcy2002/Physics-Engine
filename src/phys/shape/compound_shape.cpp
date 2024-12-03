#include "compound_shape.h"
#include <algorithm>

// style-checked
namespace pe_phys_shape {

    void CompoundShape::addShape(const pe::Transform& pos, pe::Real massRatio, Shape *shape) {
        if (shape->getType() == ShapeType::ST_Compound) {
            throw std::runtime_error("CompoundShape cannot contain another CompoundShape");
        }
        if (shape->getType() == ShapeType::ST_ConcaveMesh) {
            throw std::runtime_error("CompoundShape cannot contain ConcaveMeshShape");
        }
        if (massRatio < PE_EPS) {
            throw std::runtime_error("mass ratio must be greater than 0");
        }
        _shapes.push_back({pos, massRatio, shape});
        _mass_ratio += massRatio;
        _volume += shape->getVolume();

        // update local inertia and volume
        _local_inertia = pe::Matrix3::Zero();
        for (auto& s: _shapes) {
            pe::Matrix3 s_inertia = s.shape->getLocalInertia();
            pe::Vector3 p = s.local_transform.getOrigin();
            pe::Matrix3 rot = s.local_transform.getBasis();
            pe::Matrix3 t_inertia = pe::Matrix3::Identity() * (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    t_inertia(i, j) -= p[i] * p[j];
                }
            }
            _local_inertia += (rot * s_inertia * rot.transpose() + t_inertia) * (s.mass_ratio / _mass_ratio);
        }
    }

    void CompoundShape::getAABB(const pe::Transform &transform, pe::Vector3 &min, pe::Vector3 &max) const {
        min = PE_VEC_MAX;
        max = PE_VEC_MIN;
        for (auto& s: _shapes) {
            pe::Vector3 s_min, s_max;
            s.shape->getAABB(transform * s.local_transform, s_min, s_max);
            min = PE_VEC_MIN2(min, s_min);
            max = PE_VEC_MAX2(max, s_max);
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

}