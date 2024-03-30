#include "sphere_sphere_collision_algorithm.h"
#include "phys/shape/sphere_shape.h"

namespace pe_phys_collision {

    bool SphereSphereCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                          pe_phys_object::RigidBody* object_b,
                                                          ContactResult& result) {
        if (object_a->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Sphere ||
            object_b->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Sphere) {
            return false;
        }

        pe::Real radius_a = ((pe_phys_shape::SphereShape*)object_a->getCollisionShape())->getRadius();
        pe::Real radius_b = ((pe_phys_shape::SphereShape*)object_b->getCollisionShape())->getRadius();
        pe::Vector3 rel = object_a->getTransform().getOrigin() - object_b->getTransform().getOrigin();
        pe::Real dist = rel.norm();
        pe::Real margin = 0.005;

        if (dist > radius_a + radius_b || PE_APPROX_EQUAL(dist, 0)) {
            return false;
        }

        result.clearContactPoints();
        result.setObjects(object_a, object_b);
        pe::Vector3 normal = rel / dist;
        pe::Vector3 wPtOnB = object_b->getTransform().getOrigin() + normal * radius_b;
        pe::Real depth = dist - radius_a - radius_b;
        result.addContactPoint(normal, wPtOnB - normal * margin, depth + 2 * margin);
        result.sortContactPoints();
        return true;
    }

} // pe_phys_collision