#pragma once

#include <iostream>
#include "phys/phys_general.h"
#include "phys/object/collision_body.h"
#include "world.h"

namespace pe_core {

    class Simulator {
        COMMON_MEMBER_SET_GET(pe::Real, phys_dt, PhysDt);

    protected:
        World _world;

    public:
        void init(const pe::Array<pe_phys_object::CollisionBody*>& objs);
    };

    class SimulatorWithUi : public Simulator {
        COMMON_MEMBER_SET_GET(pe::Real, render_dt, RenderDt);

    public:
        explicit SimulatorWithUi(const std::string& name);
    };

} // namespace pe_core
