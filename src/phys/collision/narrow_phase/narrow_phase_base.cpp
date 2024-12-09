#include "narrow_phase_base.h"
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
#include "phys/collision/collision_algorithm/concave_sphere_collision_algorithm.h"
#include "phys/collision/collision_algorithm/concave_box_collision_algorithm.h"
#include "phys/collision/collision_algorithm/concave_convex_collision_algorithm.h"
#include "phys/collision/collision_algorithm/concave_cylinder_collision_algorithm.h"

namespace pe_phys_collision {

    CollisionAlgorithm* NarrowPhaseBase::getAlgorithm(pe_phys_shape::ShapeType type_a, pe_phys_shape::ShapeType type_b) {
        static CollisionAlgorithm* algorithms[36] = {
            new BoxBoxCollisionAlgorithm(), new BoxSphereCollisionAlgorithm(), new BoxCylinderCollisionAlgorithm(), new BoxConvexCollisionAlgorithm(), new ConcaveBoxCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new BoxSphereCollisionAlgorithm(), new SphereSphereCollisionAlgorithm(), new SphereCylinderCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new ConcaveSphereCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new BoxCylinderCollisionAlgorithm(), new SphereCylinderCollisionAlgorithm(), new CylinderCylinderCollisionAlgorithm(), new CylinderConvexCollisionAlgorithm(), new ConcaveCylinderCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new BoxConvexCollisionAlgorithm(), new SphereConvexCollisionAlgorithm(), new CylinderConvexCollisionAlgorithm(), new ConvexConvexCollisionAlgorithm(), new ConcaveConvexCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(),
            new ConcaveBoxCollisionAlgorithm(), new ConcaveSphereCollisionAlgorithm(), new ConcaveCylinderCollisionAlgorithm(), new ConcaveConvexCollisionAlgorithm(), nullptr, new CompoundCompoundCollisionAlgorithm(),
            new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm(), new CompoundCompoundCollisionAlgorithm()
        };
        return algorithms[PE_I(type_a) * 6 + PE_I(type_b)];
    }

} // namespace pe_phys_collision