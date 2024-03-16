#include "sphere_cylinder_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/cylinder_shape.h"

namespace pe_phys_collision {

    bool SphereCylinderCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                            pe_phys_object::RigidBody* object_b,
                                                            ContactResult& result) {
        if (object_a->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Sphere) {
            std::swap(object_a, object_b);
        }
        if (object_a->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Cylinder ||
            object_b->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Sphere) {
            return false;
        }

        auto shape_a = (pe_phys_shape::CylinderShape*)object_a->getCollisionShape();
        auto shape_b = (pe_phys_shape::SphereShape*)object_b->getCollisionShape();
        pe::Real s_r = shape_b->getRadius();
        pe::Real c_r = shape_a->getRadius();
        pe::Real c_h = shape_a->getHeight() * 0.5;
        pe::Vector3 s_pos = object_a->getTransform().inverseTransform(object_b->getTransform().getOrigin());
        pe::Real margin = 0.005;

        if (s_pos.y < -c_h - s_r || s_pos.y > c_h + s_r ||
            s_pos.x * s_pos.x + s_pos.z * s_pos.z > (c_r + s_r) * (c_r + s_r)) {
            return false;
        }

        result.setObjects(object_b, object_a);
        pe::Real r = std::sqrt(s_pos.x * s_pos.x + s_pos.z * s_pos.z);
        if (s_pos.y >= -c_h && s_pos.y <= c_h) {
            pe::Vector3 normal = pe::Vector3(s_pos.x, 0, s_pos.z).normalized();
            pe::Vector3 ptOnCyl = normal * (c_r + margin);
            ptOnCyl.y = s_pos.y;
            pe::Vector3 wNormal = object_a->getTransform().getBasis() * normal;
            pe::Vector3 wPtOnCyl = object_a->getTransform() * ptOnCyl;
            pe::Real depth = c_r + s_r - r;
            result.addContactPoint(wNormal, wPtOnCyl, -depth + 2 * margin);
        } else if (r <= c_r) {
            pe::Vector3 normal = s_pos.y > 0 ? pe::Vector3::up() : -pe::Vector3::up();
            pe::Vector3 ptOnCyl = pe::Vector3(s_pos.x, s_pos.y > 0 ? c_h : -c_h, s_pos.z);
            pe::Vector3 wNormal = object_a->getTransform().getBasis() * normal;
            pe::Vector3 wPtOnCyl = object_a->getTransform() * (ptOnCyl + normal * margin);
            pe::Real depth = s_r + c_h - std::abs(s_pos.y);
            result.addContactPoint(wNormal, wPtOnCyl, -depth + 2 * margin);
        } else {
            pe::Vector3 ptOnCyl = pe::Vector3(s_pos.x, 0, s_pos.z).normalized() * c_r;
            ptOnCyl.y = s_pos.y > 0 ? c_h : -c_h;
            pe::Vector3 wNormal = object_a->getTransform().getBasis() * (s_pos - ptOnCyl).normalized();
            pe::Vector3 wPtOnCyl = object_a->getTransform() * ptOnCyl + wNormal * margin;
            pe::Real depth = s_r - (s_pos - ptOnCyl).norm();
            result.addContactPoint(wNormal, wPtOnCyl, -depth + 2 * margin);
        }
        result.sortContactPoints();
        return true;
    }

} // pe_phys_collision