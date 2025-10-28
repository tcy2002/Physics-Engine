#include "narrow_phase_base.h"
#include "phys/collision/collision_algorithm/box_box_collision_algorithm.h"
#include "phys/collision/collision_algorithm/convex_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/box_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/sphere_sphere_collision_algorithm.h"
#include "phys/collision/collision_algorithm/box_sphere_collision_algorithm.h"
#include "phys/collision/collision_algorithm/sphere_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/compound_compound_collision_algorithm.h"
#include "phys/collision/collision_algorithm/concave_sphere_collision_algorithm.h"
#include "phys/collision/collision_algorithm/concave_box_collision_algorithm.h"
#include "phys/collision/collision_algorithm/concave_convex_collision_algorithm.h"

namespace pe_phys_collision {

    CollisionAlgorithm* NarrowPhaseBase::getAlgorithm(pe_phys_shape::ShapeType type_a, pe_phys_shape::ShapeType type_b) {
        static CollisionAlgorithm* algorithms[25] = {
            new BoxBoxCollisionAlgorithm(), new BoxSphereCollisionAlgorithm(), new BoxConvexCollisionAlgorithm(), new ConcaveBoxCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new BoxSphereCollisionAlgorithm(), new SphereSphereCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new ConcaveSphereCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new BoxConvexCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new ConvexConvexCollisionAlgorithm(), new ConcaveConvexCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new ConcaveBoxCollisionAlgorithm(), new ConcaveSphereCollisionAlgorithm(), new ConcaveConvexCollisionAlgorithm(), nullptr, new CompoundCompoundCollisionAlgorithm(),
            new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm()
        };
        return algorithms[I(type_a) * 5 + I(type_b)];
    }

} // namespace pe_phys_collision