#include "contact_result.h"
#include "utils/logger.h"
#include <algorithm>

// style-checked
namespace pe_phys_collision {

    void ContactPoint::getOrthoUnits(pe::Vector3 normal, pe::Vector3 &tangent1, pe::Vector3 &tangent2) {
        normal.normalize();
        if (PE_ABS(normal.z()) > PE_R(0.7071)) {
            // choose tangent in y-z plane
            const pe::Real a = normal.y() * normal.y() + normal.z() * normal.z();
            const pe::Real k = PE_R(1.0) / PE_SQRT(a);
            tangent1.x() = 0;
            tangent1.y() = -normal.z() * k;
            tangent1.z() = normal.y() * k;
            tangent2.x() = a * k;
            tangent2.y() = -normal.x() * tangent1.z();
            tangent2.z() = normal.x() * tangent1.y();
        } else {
            // choose tangent in x-y plane
            const pe::Real a = normal.x() * normal.x() + normal.y() * normal.y();
            const pe::Real k = PE_R(1.0) / PE_SQRT(a);
            tangent1.x() = -normal.y() * k;
            tangent1.y() = normal.x() * k;
            tangent1.z() = 0;
            tangent2.x() = -normal.z() * tangent1.y();
            tangent2.y() = normal.z() * tangent1.x();
            tangent2.z() = a * k;
        }
    }

    ContactPoint::ContactPoint():
        _world_pos(PE_VEC_MAX),
        _world_normal(pe::Vector3::UnitY()),
        _local_pos_a(pe::Vector3::Zero()),
        _local_pos_b(pe::Vector3::Zero()),
        _distance(PE_REAL_MAX),
        _world_pos_half(PE_VEC_MAX),
        _distance_non_neg(PE_REAL_MIN) {}

    ContactPoint::ContactPoint(const pe::Vector3& world_pos, const pe::Vector3& world_normal,
                               const pe_phys_object::RigidBody* object_a, const pe_phys_object::RigidBody* object_b,
                               const pe::Vector3& world_vel_a, const pe::Vector3& world_vel_b,
                               const pe::Vector3& local_pos_a, const pe::Vector3& local_pos_b, pe::Real distance):
        _world_pos(world_pos),
        _world_normal(world_normal),
        _local_pos_a(local_pos_a),
        _local_pos_b(local_pos_b),
        _distance(distance) {
        getOrthoUnits(world_normal, _tangents[0], _tangents[1]);

        // for primal-dual
        _world_pos_half = world_pos + (distance / 2) * world_normal;
        _distance_non_neg = -distance;
        const pe::Vector3 axis = pe::Vector3(1, 0, 0).cross(-world_normal);
        const pe::Real c = pe::Vector3(1, 0, 0).dot(-world_normal);
        const pe::Real angle = std::atan2(axis.norm(), c);
        const pe::Matrix3 rot = Eigen::AngleAxis<pe::Real>(angle, axis.normalized()).toRotationMatrix();
        
        const pe::Vector3 c1 = _world_pos_half - object_a->getTransform().getOrigin();
        const pe::Vector3 c2 = _world_pos_half - object_b->getTransform().getOrigin();
        trans1 = pe::MatrixMN(6, 3);
        trans2 = pe::MatrixMN(6, 3);
        trans1 << rot, c1.cross(rot.col(0)), c1.cross(rot.col(1)), c1.cross(rot.col(2));
        trans2 << -rot, c2.cross(-rot.col(0)), c2.cross(-rot.col(1)), c2.cross(-rot.col(2));
        //PE_LOG_DEBUG << "rotation1: " << object_a->getTransform().getBasis() << PE_ENDL;
        //PE_LOG_DEBUG << "translation1: " << object_a->getTransform().getOrigin().transpose() << PE_ENDL;
        //PE_LOG_DEBUG << "rotation2: " << object_b->getTransform().getBasis() << PE_ENDL;
        //PE_LOG_DEBUG << "translation2: " << object_b->getTransform().getOrigin().transpose() << PE_ENDL;
        //PE_LOG_DEBUG << "trans1: " << trans1 << PE_ENDL;
        //PE_LOG_DEBUG << "trans2: " << trans2 << PE_ENDL;
    }

    ContactResult::ContactResult():
        _friction_coeff(0),
        _restitution_coeff(0),
        _point_size(0),
        _swap_flag(false) {}

    void ContactResult::setObjects(pe_phys_object::RigidBody* object_a,
                                   pe_phys_object::RigidBody* object_b) {
        _object_a = object_a;
        _object_b = object_b;
        _friction_coeff = PE_SQRT(object_a->getFrictionCoeff() * object_b->getFrictionCoeff());
        _restitution_coeff = PE_SQRT(object_a->getRestitutionCoeff() * object_b->getRestitutionCoeff());
    }

    void ContactResult::addContactPoint(const pe::Vector3& world_normal,
                                        const pe::Vector3& world_pos, pe::Real depth) {
        if (_object_a == nullptr || _object_b == nullptr) {
            return;
        }

        pe::Vector3 point_a = world_pos;
        pe::Vector3 point_b = world_pos;
        pe::Vector3 n = world_normal;
        pe::Vector3 point = world_pos;

        if (_swap_flag) {
            n = -n;
            point_b = world_pos + world_normal * depth;
            point = point_b;
        } else {
            point_a = world_pos + world_normal * depth;
        }

        const pe::Vector3 local_pos_a = _object_a->getTransform().inverseTransform(point_a);
        const pe::Vector3 local_pos_b = _object_b->getTransform().inverseTransform(point_b);
        const pe::Vector3 world_vel_a = _object_a->getLinearVelocityAtLocalPoint(local_pos_a);
        const pe::Vector3 world_vel_b = _object_b->getLinearVelocityAtLocalPoint(local_pos_b);

        // find the same closest point
        const int cp_idx = getExistingClosestPoint(local_pos_b);
        if (cp_idx >= 0) {
            // if found, update the contact point info when the new depth is bigger
            if (depth < _points[cp_idx].getDistance()) {
                _points[cp_idx] = ContactPoint(point, n, _object_a, _object_b, world_vel_a, world_vel_b, local_pos_a, local_pos_b, depth);
            }
        } else {
            // otherwise, find an empty slot and replace it
            bool found = false;
            for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
                if (!_points[i].isValid()) {
                    _points[i] = ContactPoint(point, n, _object_a, _object_b, world_vel_a, world_vel_b, local_pos_a, local_pos_b, depth);
                    found = true;
                    break;
                }
            }
            // if no empty slot found, replace point with the minimum (abs) depth
            if (!found) {
                auto max_dist = PE_REAL_MIN;
                int idx = -1;
                for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
                    if (_points[i].getDistance() > max_dist) {
                        max_dist = _points[i].getDistance();
                        idx = i;
                    }
                }
                if (idx >= 0) {
                    _points[idx] = ContactPoint(point, n, _object_a, _object_b, world_vel_a, world_vel_b, local_pos_a, local_pos_b, depth);
                }
            }
        }
    }

    void ContactResult::sortContactPoints() {
        std::sort(_points, _points + PE_CONTACT_CACHE_SIZE,
                  [](const ContactPoint& a, const ContactPoint& b) {
            return a.getDistance() < b.getDistance();
        });
        for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
            if (!_points[i].isValid()) {
                _point_size = i;
                return;
            }
        }
        _point_size = PE_CONTACT_CACHE_SIZE;
    }

    void ContactResult::clearContactPoints() {
        for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
            _points[i].invalidate();
        }
        _point_size = 0;
        _swap_flag = false;
    }

    int ContactResult::getExistingClosestPoint(const pe::Vector3 &local_pos_b) const {
        pe::Real min_dist = getSameContactPointDistanceThreshold();
        min_dist *= min_dist;

        int nearest = -1;
        for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
            if (!_points[i].isValid()) continue;
            pe::Vector3 diff = _points[i].getLocalPosB() - local_pos_b;
            const pe::Real dist = diff.dot(diff);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
        return nearest;
    }

    pe::Real ContactResult::getSameContactPointDistanceThreshold() const {
        const pe::Real a_scale = _object_a->getAABBScale();
        const pe::Real b_scale = _object_b->getAABBScale();
        return PE_MIN(a_scale, b_scale) * PE_DIST_TH;
    }

} // pe_phys_collision