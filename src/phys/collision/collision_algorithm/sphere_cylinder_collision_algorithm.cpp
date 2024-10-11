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
        auto& c_trans = object_a->getTransform();
        pe::Real s_r = shape_b->getRadius();
        pe::Real c_r = shape_a->getRadius();
        pe::Real c_h = shape_a->getHeight() * 0.5;
        pe::Vector3 s_pos = object_a->getTransform().inverseTransform(object_b->getTransform().getOrigin());
        pe::Real margin = 0.005;

        result.clearContactPoints();
        result.setObjects(object_a, object_b);
        pe::Real r = std::sqrt(s_pos.x * s_pos.x + s_pos.z * s_pos.z);
        pe::Real d = std::abs(s_pos.y) - c_h;

        if (s_pos.y < -c_h - s_r || s_pos.y > c_h + s_r ||
            r * r > (c_r + s_r) * (c_r + s_r) ||
            (r - c_r) * (r - c_r) + d * d > s_r * s_r) {
            return false;
        }

        pe::Vector3 normal;
        pe::Vector3 ptOnSph;
        pe::Real depth;
        if (s_pos.y >= -c_h && s_pos.y <= c_h && r > 0) { // r > 0 to avoid normalizing zero vector
            // hit the side
            normal = pe::Vector3(-s_pos.x, 0, -s_pos.z).normalized();
            ptOnSph = normal * s_r + s_pos;
            depth = c_r + s_r - r;
        } else if (r <= c_r) {
            // hit the bottom or top
            normal = s_pos.y > 0 ? -pe::Vector3::up() : pe::Vector3::up();
            ptOnSph = s_pos + normal * s_r;
            depth = s_r + c_h - std::abs(s_pos.y);
        } else {
            // hit at the edge
            pe::Vector3 ptOnCyl = pe::Vector3(s_pos.x, 0, s_pos.z).normalized() * c_r;
            ptOnCyl.y = s_pos.y > 0 ? c_h : -c_h;
            normal = (ptOnCyl - s_pos).normalized();
            ptOnSph = normal * s_r + s_pos;
            depth = s_r - (s_pos - ptOnCyl).norm();
        }

        normal = c_trans.getBasis() * normal;
        ptOnSph = c_trans * ptOnSph;
        result.addContactPoint(normal, ptOnSph - normal * margin, -depth + 2 * margin);
        result.sortContactPoints();
        return true;
    }

} // pe_phys_collision