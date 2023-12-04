#pragma once

#include <iostream>
#include "phys/def.h"
#include "world.h"
#include "phys/object/collision_body.h"

namespace pe_core {

class Simulator {
    PE_MEMBER_SET_GET(PEReal, phys_dt, PhysDt);

protected:
    World _world;

public:
    void init(const pe::Array<pe_phys_object::CollisionBody*>& objs);
};

class SimulatorWithUi : public Simulator {
    PE_MEMBER_SET_GET(PEReal, render_dt, RenderDt);

public:
    explicit SimulatorWithUi(const std::string& name);
};

} // namespace pe_core
