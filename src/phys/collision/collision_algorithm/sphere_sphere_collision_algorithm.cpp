#include "sphere_sphere_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"

// style-checked.
namespace pe_phys_collision {

    bool SphereSphereCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                          pe::Transform trans_a, pe::Transform trans_b,
                                                          pe::Real refScale, ContactResult& result) {
        if (shape_a->getType() != pe_phys_shape::ShapeType::Sphere ||
            shape_b->getType() != pe_phys_shape::ShapeType::Sphere) {
            return false;
        }

        const pe::Real radius_a = dynamic_cast<pe_phys_shape::SphereShape *>(shape_a)->getRadius();
        const pe::Real radius_b = dynamic_cast<pe_phys_shape::SphereShape *>(shape_b)->getRadius();
        const pe::Vector3 rel = trans_a.getOrigin() - trans_b.getOrigin();
        const pe::Real dist = rel.norm();
        constexpr auto margin = PE_MARGIN;

        if (dist > radius_a + radius_b || PE_APPROX_EQUAL(dist, 0)) {
            return false;
        }

        const pe::Vector3 normal = rel / dist;
        const pe::Vector3 wPtOnB = trans_b.getOrigin() + normal * radius_b;
        const pe::Real depth = dist - radius_a - radius_b;
        result.addContactPoint(normal, wPtOnB - normal * margin, depth + 2 * margin);
        return true;
    }

} // pe_phys_collision