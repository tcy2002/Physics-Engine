#include "sphere_cylinder_collision_algorithm.h"
#include "rigid/shape/sphere_shape.h"
#include "rigid/shape/cylinder_shape.h"

// style-checked.
namespace pe_phys_collision {

    bool SphereCylinderCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                            pe::Transform trans_a, pe::Transform trans_b,
                                                            pe::Real ref_scale, ContactResult& result) {
        if (!((shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Cylinder) ||
              (shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder &&
               shape_b->getType() == pe_phys_shape::ShapeType::ST_Sphere))) {
            return false;
        }

        const auto shape_sph = dynamic_cast<pe_phys_shape::SphereShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere ? shape_a : shape_b);
        const auto shape_cyl = dynamic_cast<pe_phys_shape::CylinderShape *>(shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? shape_a : shape_b);
        const auto trans_sph = shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere ? trans_a : trans_b;
        const auto trans_cyl = shape_a->getType() == pe_phys_shape::ShapeType::ST_Cylinder ? trans_a : trans_b;

        constexpr auto margin = PE_MARGIN;

        result.setSwapFlag(shape_a->getType() == pe_phys_shape::ShapeType::ST_Sphere);
        bool ret = getClosestPoints(shape_sph, shape_cyl, trans_sph, trans_cyl, margin, result);
        result.setSwapFlag(false);

        return ret;
    }

    bool SphereCylinderCollisionAlgorithm::getClosestPoints(
            pe_phys_shape::SphereShape *shape_sph, pe_phys_shape::CylinderShape *shape_cyl,
            const pe::Transform &trans_sph, const pe::Transform &trans_cyl,
            pe::Real margin, ContactResult &result) {
        const pe::Real s_r = shape_sph->getRadius();
        const pe::Real c_r = shape_cyl->getRadius();
        const pe::Real c_h = shape_cyl->getHeight() * pe::Real(0.5);
        const pe::Vector3 s_pos = trans_cyl.inverseTransform(trans_sph.getOrigin());

        const pe::Real r = PE_SQRT(s_pos.x() * s_pos.x() + s_pos.z() * s_pos.z());
        const pe::Real d = PE_ABS(s_pos.y()) - c_h;

        if (s_pos.y() < -c_h - s_r || s_pos.y() > c_h + s_r ||
        r * r > (c_r + s_r) * (c_r + s_r) ||
        (d > 0 && r > c_r && (r - c_r) * (r - c_r) + d * d > s_r * s_r)) {
            return false;
        }

        pe::Vector3 normal;
        pe::Vector3 ptOnSph;
        pe::Real depth;
        if (s_pos.y() >= -c_h && s_pos.y() <= c_h && r > 0) { // r > 0 to avoid normalizing zero vector
            // hit the side
            normal = pe::Vector3(-s_pos.x(), 0, -s_pos.z()).normalized();
            ptOnSph = normal * s_r + s_pos;
            depth = c_r + s_r - r;
        } else if (r <= c_r) {
            // hit the bottom or top
            normal = pe::Vector3::UnitY();
            if (s_pos.y() > 0) normal = -normal;
            ptOnSph = s_pos + normal * s_r;
            depth = s_r + c_h - PE_ABS(s_pos.y());
        } else {
            // hit at the edge
            pe::Vector3 ptOnCyl = pe::Vector3(s_pos.x(), 0, s_pos.z()).normalized() * c_r;
            ptOnCyl.y() = s_pos.y() > 0 ? c_h : -c_h;
            normal = (ptOnCyl - s_pos).normalized();
            ptOnSph = normal * s_r + s_pos;
            depth = s_r - (s_pos - ptOnCyl).norm();
        }

        normal = trans_cyl.getBasis() * normal;
        ptOnSph = trans_cyl * ptOnSph;
        result.addContactPoint(normal, ptOnSph - normal * margin, -depth + 2 * margin);
        return true;
    }

} // pe_phys_collision