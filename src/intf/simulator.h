#pragma once

#include "world.h"
#include "viewer.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/concave_mesh_shape.h"
#include "phys/shape/convex_mesh_shape.h"

namespace pe_intf { // interface

    enum class UseViewer { False = 0, True = 1 };

    // Simulator base class
    template <UseViewer UV>
    class Simulator {
    protected:
        World _world;

    public:
        Simulator() {}
        virtual ~Simulator() {}

        // Initialize the physics world here before running
        virtual void init() {}
        // Called every frame to update the physics world
        virtual void step() {}

        void start(pe::Real dt = pe::Real(0.01), int max_frame = 1024);

    private:
        pe::Map<pe_phys_object::RigidBody*, int> _id_map;
        bool renderInit();
        bool renderStep();
        void addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
    };

    #include "simulator.cpp"

} // namespace pe_intf

#define PE_SIM_MAIN(Simulator, Dt, MaxFrame) \
int main() { \
    Simulator sim; \
    sim.start(Dt, MaxFrame); \
    return 0; \
}
