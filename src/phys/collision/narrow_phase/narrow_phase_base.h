#pragma once

#include "phys/phys_general.h"
#include "phys/object/rigidbody.h"
#include "phys/collision/broad_phase/broad_phase_base.h"
#include "contact_result.h"
#include "phys/collision/collision_algorithm/box_box_collision_algorithm.h"
#include "phys/collision/collision_algorithm/mesh_mesh_collision_algorithm.h"

namespace pe_phys_collision {

    class NarrowPhaseBase {
    protected:
        pe::Array<CollisionAlgorithm*> _algos;

    public:
        NarrowPhaseBase() {
            _algos = {new BoxBoxCollisionAlgorithm(), new MeshMeshCollisionAlgorithm() };
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