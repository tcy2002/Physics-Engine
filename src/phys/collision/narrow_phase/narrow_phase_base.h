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

namespace pe_phys_collision {

    class NarrowPhaseBase {
    protected:
        pe::Array<CollisionAlgorithm*> _algos;

    public:
        NarrowPhaseBase() {
            _algos = {
                    new BoxBoxCollisionAlgorithm(), new BoxSphereCollisionAlgorithm(), new BoxCylinderCollisionAlgorithm(), new BoxConvexCollisionAlgorithm(),
                    new BoxSphereCollisionAlgorithm(), new SphereSphereCollisionAlgorithm(), new SphereCylinderCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(),
                    new BoxCylinderCollisionAlgorithm(), new SphereCylinderCollisionAlgorithm(), new CylinderCylinderCollisionAlgorithm(), new CylinderConvexCollisionAlgorithm(),
                    new BoxConvexCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new CylinderConvexCollisionAlgorithm(), new ConvexConvexCollisionAlgorithm()
            };
        }
        virtual ~NarrowPhaseBase() {
            for (auto algo : _algos) {
                delete algo;
            }
        }

        virtual void calcContactResults(const pe::Array<CollisionPair>&) = 0;
        virtual void clearContactResults() = 0;
        virtual pe::Array<ContactResult>& getContactResults() = 0;
    };

} // namespace pe_phys_collision