#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"
#include "phys/collision/broad_phase/broad_phase_base.h"
#include "contact_result.h"
#include "phys/collision/collision_algorithm/box_box_collision_algorithm.h"
#include "phys/collision/collision_algorithm/convex_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/box_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/sphere_sphere_collision_algorithm.h"
#include "phys/collision/collision_algorithm/box_sphere_collision_algorithm.h"
#include "phys/collision/collision_algorithm/sphere_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/sphere_cylinder_collision_algorithm.h"
#include "phys/collision/collision_algorithm/cylinder_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/box_cylinder_collision_algorithm.h"
#include "phys/collision/collision_algorithm/cylinder_cylinder_collision_algorithm.h"
#include "phys/collision/collision_algorithm/compound_compound_collision_algorithm.h"

namespace pe_phys_collision {

    class NarrowPhaseBase {
    protected:
        pe::Array<CollisionAlgorithm*> _algos;
        CollisionAlgorithm* getAlgorithm(pe_phys_shape::ShapeType type_a, pe_phys_shape::ShapeType type_b) {
            return _algos[(int)type_a * 5 + (int)type_b];
        }

    public:
        NarrowPhaseBase() {
            _algos = {
                    new BoxBoxCollisionAlgorithm(), new BoxSphereCollisionAlgorithm(), new BoxCylinderCollisionAlgorithm(), new BoxConvexCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
                    new BoxSphereCollisionAlgorithm(), new SphereSphereCollisionAlgorithm(), new SphereCylinderCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
                    new BoxCylinderCollisionAlgorithm(), new SphereCylinderCollisionAlgorithm(), new CylinderCylinderCollisionAlgorithm(), new CylinderConvexCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
                    new BoxConvexCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new CylinderConvexCollisionAlgorithm(), new ConvexConvexCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
                    new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm()
            };
        }
        virtual ~NarrowPhaseBase() {
            for (auto algo : _algos) {
                delete algo;
            }
        }

        virtual void calcContactResults(const pe::Array<CollisionPair>& pairs, pe::Array<ContactResult*>& results) = 0;
    };

} // namespace pe_phys_collision