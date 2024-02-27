#pragma once

#include "phys/phys_general.h"
#include "phys/object/collision_object.h"
#include "phys/collision/broad_phase/broad_phase_base.h"
#include "contact_result.h"

namespace pe_phys_collision {

    class NarrowPhaseBase {
    public:
        NarrowPhaseBase() {}
        virtual ~NarrowPhaseBase() {}

        virtual void calcContactResults(const pe::Array<CollisionPair>&) = 0;
        virtual void clearContactResults() = 0;
        virtual pe::Array<ContactResult>& getContactResults() = 0;
    };

} // namespace pe_phys_collision