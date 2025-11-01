#include "narrow_phase_base.h"
#include "physics/collision/collision_algorithm/box_box_collision_algorithm.h"
#include "physics/collision/collision_algorithm/convex_convex_collision_algorithm.h"
#include "physics/collision/collision_algorithm/box_convex_collision_algorithm.h"
#include "physics/collision/collision_algorithm/sphere_sphere_collision_algorithm.h"
#include "physics/collision/collision_algorithm/box_sphere_collision_algorithm.h"
#include "physics/collision/collision_algorithm/sphere_convex_collision_algorithm.h"
#include "physics/collision/collision_algorithm/compound_compound_collision_algorithm.h"
#include "physics/collision/collision_algorithm/concave_sphere_collision_algorithm.h"
#include "physics/collision/collision_algorithm/concave_box_collision_algorithm.h"
#include "physics/collision/collision_algorithm/concave_convex_collision_algorithm.h"
#include "physics/collision/collision_algorithm/box_capsule_collision_algorithm.h"
#include "physics/collision/collision_algorithm/sphere_capsule_collision_algorithm.h"
#include "physics/collision/collision_algorithm/capsule_capsule_collision_algorithm.h"

namespace pe_physics_collision {

CollisionAlgorithm* NarrowPhaseBase::getAlgorithm(pe_physics_shape::ShapeType type_a, pe_physics_shape::ShapeType type_b) {
    static CollisionAlgorithm* algorithms[36] = {
        new BoxBoxCollisionAlgorithm(), new BoxSphereCollisionAlgorithm(), new BoxConvexCollisionAlgorithm(), new ConcaveBoxCollisionAlgorithm(), new BoxCapsuleCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
        new BoxSphereCollisionAlgorithm(), new SphereSphereCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new ConcaveSphereCollisionAlgorithm(), new SphereCapsuleCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
        new BoxConvexCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new ConvexConvexCollisionAlgorithm(), new ConcaveConvexCollisionAlgorithm(), nullptr, new CompoundCompoundCollisionAlgorithm(),
        new ConcaveBoxCollisionAlgorithm(), new ConcaveSphereCollisionAlgorithm(), new ConcaveConvexCollisionAlgorithm(), nullptr, nullptr, new CompoundCompoundCollisionAlgorithm(),
        new BoxCapsuleCollisionAlgorithm(), new SphereCapsuleCollisionAlgorithm(), nullptr, nullptr, new CapsuleCapsuleCollisionAlgorithm(), nullptr,
        new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), nullptr, new CompoundCompoundCollisionAlgorithm()
    };
    return algorithms[PE_I(type_a) * 6 + PE_I(type_b)];
}

} // namespace pe_physics_collision
