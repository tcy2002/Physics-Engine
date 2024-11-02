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
#include "concave_sphere_collision_algorithm.h"
#include "concave_box_collision_algorithm.h"
#include "concave_cylinder_collision_algorithm.h"
#include "concave_convex_collision_algorithm.h"
#include "phys/shape/compound_shape.h"

namespace pe_phys_collision {

    bool CompoundCompoundCollisionAlgorithm::processCollision(pe_phys_shape::Shape* shape_a, pe_phys_shape::Shape* shape_b,
                                                              pe::Transform trans_a, pe::Transform trans_b,
                                                              pe::Real refScale, ContactResult& result) {
        if (shape_b->getType() == pe_phys_shape::ShapeType::Compound) {
            std::swap(shape_a, shape_b);
            std::swap(trans_a, trans_b);
            result.setObjects(result.getObjectB(), result.getObjectA());
        }
        if (shape_a->getType() != pe_phys_shape::ShapeType::Compound) {
            return false;
        }

        bool has_contact = false;

        for (auto& s : ((pe_phys_shape::CompoundShape*)shape_a)->getShapes()) {
            pe::Transform trans_a_w = trans_a * s.local_transform;
            if (shape_b->getType() == pe_phys_shape::ShapeType::Compound) {
                auto compound_b = (pe_phys_shape::CompoundShape*)shape_b;
                for (auto& s_b : compound_b->getShapes()) {
                    pe::Transform trans_b_w = trans_b * s_b.local_transform;
                    has_contact |= processSubCollision(s.shape, s_b.shape,
                                                       trans_a_w, trans_b_w, refScale, result);
                }
            } else {
                has_contact |= processSubCollision(s.shape, shape_b,
                                                   trans_a_w, trans_b, refScale, result);
            }
        }

        return has_contact;
    }

    CollisionAlgorithm* CompoundCompoundCollisionAlgorithm::getCollisionAlgorithm(int index) {
        static BoxBoxCollisionAlgorithm box_box;
        static BoxSphereCollisionAlgorithm box_sphere;
        static BoxCylinderCollisionAlgorithm box_cylinder;
        static BoxConvexCollisionAlgorithm box_convex;
        static SphereSphereCollisionAlgorithm sphere_sphere;
        static SphereCylinderCollisionAlgorithm sphere_cylinder;
        static SphereConvexCollisionAlgorithm sphere_convex;
        static CylinderCylinderCollisionAlgorithm cylinder_cylinder;
        static CylinderConvexCollisionAlgorithm cylinder_convex;
        static ConvexConvexCollisionAlgorithm convex_convex;
        static ConcaveSphereCollisionAlgorithm concave_sphere;
        static ConcaveBoxCollisionAlgorithm concave_box;
        static ConcaveCylinderCollisionAlgorithm concave_cylinder;
        static ConcaveConvexCollisionAlgorithm concave_convex;
        static CollisionAlgorithm* algos[] = {
            &box_box, &box_sphere, &box_cylinder, &box_convex, &concave_box,
            &box_sphere, &sphere_sphere, &sphere_cylinder, &sphere_convex, &concave_sphere,
            &box_cylinder, &sphere_cylinder, &cylinder_cylinder, &cylinder_convex, &concave_cylinder,
            &box_convex, &sphere_convex, &cylinder_convex, &convex_convex, &concave_convex,
            &concave_box, &concave_sphere, &concave_cylinder, &concave_convex, nullptr
        };
        return algos[index];
    }

    bool CompoundCompoundCollisionAlgorithm::processSubCollision(pe_phys_shape::Shape *shape_a,
                                                                 pe_phys_shape::Shape *shape_b,
                                                                 pe::Transform& trans_a,
                                                                 pe::Transform& trans_b,
                                                                 pe::Real refScale, ContactResult &result) {
        int algo_index = (int)shape_a->getType() * 5 + (int)shape_b->getType();
        auto algo = getCollisionAlgorithm(algo_index);
        if (algo == nullptr) return false;
        return algo->processCollision(shape_a, shape_b, trans_a, trans_b, refScale, result);
    }

} // pe_phys_collision