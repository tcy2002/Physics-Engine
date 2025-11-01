#pragma once

#include "world.h"
#include "viewer.h"
#include "utils/thread_pool.h"

namespace pe_interface {

// Simulator base class
class Simulator {
protected:
    World _world;
    bool _saving = false;
    bool _show_debug_points = false;
    void showDebugPoints();

public:
    bool use_gui = true;
    int max_frame = INT32_MAX;
    int target_framerate = 60;

    Simulator() = default;
    virtual ~Simulator() = default;

    // Initialize the physics world here before running
    virtual void init() {}
    // Called every frame to update the physics world
    virtual void step() {}

    void start();

private:
    pe::Map<pe_physics_object::RigidBody*, pe::Array<int>> _id_map;
    void renderInit();
    bool renderStep(uint64_t& blocking_time);
    void addModels(const pe::Array<pe_physics_object::RigidBody*>& rbs);
    void removeModels(const pe::Array<pe_physics_object::RigidBody*>& rbs);
    void updateColor(int id, pe_physics_shape::ShapeType type, const std::string& tag, bool kinematic);
};

} // namespace pe_interface

#define PE_CUSTOM_MAIN(Simulator, TargetFrameRate) \
int main() { \
    Simulator sim; \
    sim.target_framerate = TargetFrameRate; \
    sim.start(); \
    if (WIN32) system("pause"); \
    return 0; \
}
