#include "sphere_cylinder_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/cylinder_shape.h"

namespace pe_phys_collision {

    bool SphereCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                            pe::Transform trans_a, pe::Transform trans_b,
                                                            ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::Sphere &&
               shape_b->getType() == pe_phys_shape::ShapeType::Cylinder) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::Sphere))) {
            return false;
        }

        auto shape_sph = (pe_phys_shape::SphereShape*)(shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? shape_a : shape_b);
        auto shape_cyl = (pe_phys_shape::CylinderShape*)(shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? shape_a : shape_b);
        auto trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::Sphere ? trans_a : trans_b;
        auto trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::Cylinder ? trans_a : trans_b;

        pe::Real s_r = shape_sph->getRadius();
        pe::Real c_r = shape_cyl->getRadius();
        pe::Real c_h = shape_cyl->getHeight() * 0.5;
        pe::Vector3 s_pos = trans_cyl.inverseTransform(trans_sph.getOrigin());
        pe::Real margin = 0.005;

        pe::Real r = std::sqrt(s_pos.x * s_pos.x + s_pos.z * s_pos.z);
        pe::Real d = std::abs(s_pos.y) - c_h;

        if (s_pos.y < -c_h - s_r || s_pos.y > c_h + s_r ||
            r * r > (c_r + s_r) * (c_r + s_r) ||
            (d > 0 && ((r - c_r) * (r - c_r) + d * d > s_r * s_r))) {
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

        normal = trans_cyl.getBasis() * normal;
        ptOnSph = trans_cyl * ptOnSph;
        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::Sphere);
        result.addContactPoint(normal, ptOnSph - normal * margin, -depth + 2 * margin);
        return true;
    }

} // pe_phys_collision