#include "sphere_sphere_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"

namespace pe_phys_collision {

    bool SphereSphereCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                          pe::Transform trans_a, pe::Transform trans_b,
                                                          ContactResult& result) {
        if (shape_a->getType() != pe_phys_shape::ShapeType::Sphere ||
            shape_b->getType() != pe_phys_shape::ShapeType::Sphere) {
            return false;
        }

        pe::Real radius_a = ((pe_phys_shape::SphereShape*)shape_a)->getRadius();
        pe::Real radius_b = ((pe_phys_shape::SphereShape*)shape_b)->getRadius();
        pe::Vector3 rel = trans_a.getOrigin() - trans_b.getOrigin();
        pe::Real dist = rel.norm();
        pe::Real margin = 0.005;

        if (dist > radius_a + radius_b || PE_APPROX_EQUAL(dist, 0)) {
            return false;
        }

        pe::Vector3 normal = rel / dist;
        pe::Vector3 wPtOnB = trans_b.getOrigin() + normal * radius_b;
        pe::Real depth = dist - radius_a - radius_b;
        result.addContactPoint(normal, wPtOnB - normal * margin, depth + 2 * margin);
        result.sortContactPoints();
        return true;
    }

} // pe_phys_collision