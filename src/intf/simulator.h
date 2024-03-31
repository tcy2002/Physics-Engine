#pragma once

#include "world.h"
#include "viewer.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/concave_mesh_shape.h"
#include "phys/shape/convex_mesh_shape.h"

namespace pe_intf { // interface

    // Simulator base class
    template <bool UseViewer>
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

        void run(pe::Real dt = pe::Real(0.01), int max_frame = 1024);

    private:
        pe::Map<int, pe_phys_object::RigidBody*> _id_map;
        void renderInit();
        bool renderStep();
    };

    #include "simulator.cpp"

} // namespace pe_intf