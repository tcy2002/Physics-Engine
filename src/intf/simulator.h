#pragma once

#include "world.h"
#include "viewer.h"
#include "utils/logger.h"
#include "utils/thread_pool.h"
#include "rigid/shape/box_shape.h"
#include "rigid/shape/sphere_shape.h"
#include "rigid/shape/cylinder_shape.h"
#include "rigid/shape/compound_shape.h"
#include "rigid/shape/convex_mesh_shape.h"
#include "rigid/shape/concave_mesh_shape.h"

namespace pe_intf { // interface

    // Simulator base class
    class Simulator {
    protected:
        World _world;

    public:
        bool use_gui = true;
        int max_frame = INT32_MAX;
        int target_framerate = 60;
        bool saving = false;
        std::string save_path = "./data";

        Simulator() {}
        virtual ~Simulator() {}

        // Store the current scene as a config file
        void saveScene(const std::string& path_to_write) const;
        // Load a config file
        bool loadScene(int argc, char** argv);
        // store the simulation flow into a gltf file
        void saveGltf() const;

        // Initialize the physics world here before running
        virtual void init() {}
        // Called every frame to update the physics world
        virtual void step() {}

        PE_API void start();

    private:
        pe::Map<pe_phys_object::RigidBody*, pe::Array<int>> _id_map;
        pe::Map<pe_phys_object::ClothObject*, int> _cloth_id_map;
        bool renderInit();
        void renderStep();
        void addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void addModels(const pe::Array<pe_phys_object::ClothObject*>& cloths);
        void removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void removeModels(const pe::Array<pe_phys_object::ClothObject*>& cloths);
        void updateColor(int id, pe_phys_shape::ShapeType type, const std::string& tag, bool kinematic);
    };

} // namespace pe_intf

#ifdef WIN32
#define PAUSE system("pause")
#else
#define PAUSE 
#endif

#define PE_CONFIG_MAIN() \
int main(int argc, char** argv) { \
    pe_intf::Simulator sim; \
    if (sim.loadScene(argc, argv)) { \
        sim.start(); \
    } \
    PAUSE; \
    return 0; \
}

#define PE_CUSTOM_MAIN(Simulator, TargetFrameRate) \
int main() { \
    Simulator sim; \
    sim.target_framerate = TargetFrameRate; \
    sim.start(); \
    PAUSE; \
    return 0; \
}

#define PE_CONFIG_CUSTOM_MAIN(Simulator, TargetFrameRate) \
int main(int argc, char** argv) { \
    Simulator sim; \
    if (sim.loadScene(argc, argv)) { \
        sim.target_framerate = TargetFrameRate; \
        sim.start(); \
    } \
    PAUSE; \
    return 0; \
}
