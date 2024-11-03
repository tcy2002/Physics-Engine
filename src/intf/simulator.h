#pragma once

#include "world.h"
#include "viewer.h"
#include "utils/logger.h"
#include "utils/thread_pool.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/sphere_shape.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/shape/compound_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/concave_mesh_shape.h"
#include "json/json.hpp"
#include "utils/file_system.h"

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

        // Store the current scene as a config file
        void saveScene();
        // Load a config file
        bool loadScene(int argc, char** argv);
        // Store the current scene as a binary file
        void saveState();
        // Load a binary file
        bool loadState();

        // Initialize the physics world here before running
        virtual void init() {}
        // Called every frame to update the physics world
        virtual void step() {}

        void start(int target_frame_rate = 60);

    private:
        pe::Map<pe_phys_object::RigidBody*, pe::Array<int>> _id_map;
        bool renderInit();
        bool renderStep();
        void addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void updateColor(int id, pe_phys_shape::ShapeType type, const std::string& tag, bool kinematic);
    };

    #include "simulator.cpp"

} // namespace pe_intf

#define PE_CONFIG_MAIN(TargetFrameRate) \
int main(int argc, char** argv) { \
    pe_intf::Simulator<pe_intf::UseViewer::True> sim; \
    if (sim.loadScene(argc, argv)) { \
        sim.start(TargetFrameRate); \
    } \
    if (WIN32) system("pause"); \
    return 0; \
}

#define PE_CUSTOM_MAIN(Simulator, TargetFrameRate) \
int main() { \
    Simulator sim; \
    sim.start(TargetFrameRate); \
    if (WIN32) system("pause"); \
    return 0; \
}

#define PE_CONFIG_CUSTOM_MAIN(Simulator, TargetFrameRate) \
int main(int argc, char** argv) { \
    Simulator sim; \
    if (sim.loadScene(argc, argv)) { \
        sim.start(TargetFrameRate); \
    } \
    if (WIN32) system("pause"); \
    return 0; \
}
