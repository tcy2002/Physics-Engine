#include "compound_compound_collision_algorithm.h"
#include "box_box_collision_algorithm.h"
#include "box_sphere_collision_algorithm.h"
#include "box_cylinder_collision_algorithm.h"
#include "box_convex_collision_algorithm.h"
#include "sphere_sphere_collision_algorithm.h"
#include "sphere_cylinder_collision_algorithm.h"
#include "sphere_convex_collision_algorithm.h"
#include "cylinder_cylinder_collision_algorithm.h"
#include "cylinder_convex_collision_algorithm.h"
#include "convex_convex_collision_algorithm.h"
#include "phys/shape/compound_shape.h"

namespace pe_phys_collision {

    bool CompoundCompoundCollisionAlgorithm::processCollision(pe_phys_object::RigidBody* object_a,
                                                              pe_phys_object::RigidBody* object_b,
                                                              ContactResult& result) {
        if (object_b->getCollisionShape()->getType() == pe_phys_shape::ShapeType::Compound) {
            std::swap(object_a, object_b);
        }
        if (object_a->getCollisionShape()->getType() != pe_phys_shape::ShapeType::Compound) {
            return false;
        }

        auto shape_a = (pe_phys_shape::CompoundShape*)object_a->getCollisionShape();
        auto shape_b = object_b->getCollisionShape();

        result.clearContactPoints();
        result.setObjects(object_a, object_b);
        bool has_contact = false;

        for (auto& s : shape_a->getShapes()) {
            pe::Transform trans_a = object_a->getTransform() * s.local_transform;
            if (shape_b->getType() == pe_phys_shape::ShapeType::Compound) {
                auto compound_b = (pe_phys_shape::CompoundShape*)shape_b;
                for (auto& s_b : compound_b->getShapes()) {
                    pe::Transform trans_b = object_b->getTransform() * s_b.local_transform;
                    has_contact |= processSubCollision(s.shape, s_b.shape,
                                                       trans_a, trans_b, result);
                }
            } else {
                has_contact |= processSubCollision(s.shape, shape_b,
                                                   trans_a, object_b->getTransform(), result);
            }
        }

        if (has_contact) {
            result.sortContactPoints();
        }
        return has_contact;
    }

    CollisionAlgorithm* CompoundCompoundCollisionAlgorithm::getCollisionAlgorithm(int index) {
        BoxBoxCollisionAlgorithm box_box;
        BoxSphereCollisionAlgorithm box_sphere;
        BoxCylinderCollisionAlgorithm box_cylinder;
        BoxConvexCollisionAlgorithm box_convex;
        SphereSphereCollisionAlgorithm sphere_sphere;
        SphereCylinderCollisionAlgorithm sphere_cylinder;
        SphereConvexCollisionAlgorithm sphere_convex;
        CylinderCylinderCollisionAlgorithm cylinder_cylinder;
        CylinderConvexCollisionAlgorithm cylinder_convex;
        ConvexConvexCollisionAlgorithm convex_convex;
        static CollisionAlgorithm* algos[] = {
            &box_box, &box_sphere, &box_cylinder, &box_convex,
            &box_sphere, &sphere_sphere, &sphere_cylinder, &sphere_convex,
            &box_cylinder, &sphere_cylinder, &cylinder_cylinder, &cylinder_convex,
            &box_convex, &sphere_convex, &cylinder_convex, &convex_convex
        };
        return algos[index];
    }

    bool CompoundCompoundCollisionAlgorithm::processSubCollision(pe_phys_shape::Shape *shape_a,
                                                                 pe_phys_shape::Shape *shape_b,
                                                                 const pe::Transform& trans_a,
                                                                 const pe::Transform& trans_b,
                                                                 ContactResult &result) {
        static pe_phys_object::RigidBody rb_a, rb_b;
        rb_a.setCollisionShape(shape_a);
        rb_b.setCollisionShape(shape_b);
        rb_a.setTransform(trans_a);
        rb_b.setTransform(trans_b);

        int algo_index = (int)shape_a->getType() * 4 + (int)shape_b->getType();
        auto algo = getCollisionAlgorithm(algo_index);
        return algo->processCollision(&rb_a, &rb_b, result);
    }

} // pe_phys_collision