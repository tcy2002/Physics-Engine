#pragma once

#include "world.h"
#include "viewer.h"
#include "utils/logger.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/compound_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include <sstream>

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

        void start(int target_frame_rate = 60);

    private:
        pe::Map<pe_phys_object::RigidBody*, pe::Array<int>> _id_map;
        void toggleLine();
        bool renderInit();
        bool renderStep();
        void addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void updateColor(int id, pe_phys_shape::ShapeType type, const std::string& tag, bool kinematic);
    };

    #include "simulator.cpp"

} // namespace pe_intf

#define PE_SIM_MAIN(Simulator, TargetFrameRate) \
int main() { \
    Simulator sim; \
    sim.start(TargetFrameRate); \
    return 0; \
}
