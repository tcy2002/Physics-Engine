#pragma once

#include "world.h"
#include "viewer.h"
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

        void start(pe::Real dt = pe::Real(0.01), int max_frame = 1024);

    private:
        pe::Map<pe_phys_object::RigidBody*, pe::Array<int>> _id_map;
        void toggleLine();
        bool renderInit();
        bool renderStep();
        void addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
        void removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs);
    };

    #include "simulator.cpp"

    template <UseViewer UV>
    void Simulator<UV>::start(pe::Real dt, int max_frame) {
        _world.setDt(dt);
        init();
        if (UV == UseViewer::True) {
            if (!renderInit()) {
                return;
            }
        }

        int frame = 0;
        int target_dt = (int)(dt * 1000);
        auto start = COMMON_GetTickCount();
        while (++frame < max_frame) {
            auto t = COMMON_GetTickCount();

            _world.step();

            if (UV == UseViewer::True) {
                if (!_world.getRigidBodiesToRemove().empty()) {
                    removeModels(_world.getRigidBodiesToRemove());
                    _world.clearRigidBodiesToRemove();
                }
                if (!_world.getRigidBodiesToAdd().empty()) {
                    addModels(_world.getRigidBodiesToAdd());
                    _world.clearRigidBodiesToAdd();
                }
            }

            step();

            if (UV == UseViewer::True) {
                if (!renderStep()) {
                    break;
                }
            }

            auto actual_dt = (int)(COMMON_GetTickCount() - t);
            COMMON_Sleep(target_dt - actual_dt);

            // to use the actual dt
            _world.setDt(actual_dt < target_dt ? dt : (pe::Real)(actual_dt) * pe::Real(0.001));
        }

        auto end = COMMON_GetTickCount();
        pe::Real total_time = (pe::Real)(end - start) * pe::Real(0.001);
        std::cout << "fps: " << (pe::Real)frame / total_time << std::endl;
        std::cout << "total time: " << total_time << "s" << std::endl;
        std::cout << "update status time: " << _world.update_status_time << "s " << _world.update_status_time / total_time << std::endl;
        std::cout << "broad phase time: " << _world.broad_phase_time << "s " << _world.broad_phase_time / total_time << std::endl;
        std::cout << "narrow phase time: " << _world.narrow_phase_time << "s " << _world.narrow_phase_time / total_time << std::endl;
        std::cout << "constraint solver time: " << _world.constraint_solver_time << "s " << _world.constraint_solver_time / total_time << std::endl;
        pe::Real other_time = total_time - _world.update_status_time - _world.broad_phase_time - _world.narrow_phase_time - _world.constraint_solver_time;
        std::cout << "other time: " << other_time << "s" << " " << other_time / total_time << std::endl;
    }

    template <UseViewer UV>
    void Simulator<UV>::toggleLine() {
        static bool show_line = false;
        if (pe_intf::Viewer::getKeyState('c') == 2) {
            show_line = !show_line;
            pe_intf::Viewer::showLine(show_line, 1);
        }
    }

    template <UseViewer UV>
    bool Simulator<UV>::renderInit() {
        pe_intf::Viewer::open("PhysicsDemo", 800, 600, {0, 10, 20}, 0, (float)(PE_PI / 12.0));

        // initialize models
        addModels(_world.getRigidBodiesToAdd());
        _world.clearRigidBodiesToAdd();

        // wait for the window to open
        while (!pe_intf::Viewer::isOpen()) {
            COMMON_Sleep(10);
        }

        // wait for key 'r' to start
        while (pe_intf::Viewer::getKeyState('x') != 0) {
            COMMON_Sleep(10);
            if (!pe_intf::Viewer::isOpen() || pe_intf::Viewer::getKeyState(27) == 0) {
                pe_intf::Viewer::close();
                return false;
            }
            toggleLine();
        }
        return true;
    }

    template <UseViewer UV>
    bool Simulator<UV>::renderStep() {
        if (!pe_intf::Viewer::isOpen() || pe_intf::Viewer::getKeyState(27) == 0) {
            pe_intf::Viewer::close();
            return false;
        }

        toggleLine();

        for (auto& rb : _id_map) {
            if (rb.second.size() == 1 && rb.second[0] == -1) {
                continue;
            }
            switch (rb.first->getCollisionShape()->getType()) {
                case pe_phys_shape::ShapeType::Box:
                    pe_intf::Viewer::updateCubeTransform(rb.second[0], rb.first->getTransform());
                    break;
                case pe_phys_shape::ShapeType::Sphere:
                    pe_intf::Viewer::updateSphereTransform(rb.second[0], rb.first->getTransform());
                    break;
                case pe_phys_shape::ShapeType::Cylinder:
                    pe_intf::Viewer::updateCylinderTransform(rb.second[0], rb.first->getTransform());
                    break;
                case pe_phys_shape::ShapeType::ConvexMesh:
                    pe_intf::Viewer::updateMeshTransform(rb.second[0], rb.first->getTransform());
                    break;
                case pe_phys_shape::ShapeType::Compound:
                    int i = 0;
                    for (auto& s : ((pe_phys_shape::CompoundShape*)rb.first->getCollisionShape())->getShapes()) {
                        switch (s.shape->getType()) {
                            case pe_phys_shape::ShapeType::Box:
                                pe_intf::Viewer::updateCubeTransform(rb.second[i++], rb.first->getTransform() * s.local_transform);
                                break;
                            case pe_phys_shape::ShapeType::Sphere:
                                pe_intf::Viewer::updateSphereTransform(rb.second[i++], rb.first->getTransform() * s.local_transform);
                                break;
                            case pe_phys_shape::ShapeType::Cylinder:
                                pe_intf::Viewer::updateCylinderTransform(rb.second[i++], rb.first->getTransform() * s.local_transform);
                                break;
                            case pe_phys_shape::ShapeType::ConvexMesh:
                                pe_intf::Viewer::updateMeshTransform(rb.second[i++], rb.first->getTransform() * s.local_transform);
                            default:
                                break;
                        }
                    }
                    break;
            }
        }
        return true;
    }

    template <UseViewer UV>
    void Simulator<UV>::addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
        pe::Array<pe::KV<int, pe_phys_object::RigidBody*>> pairs;

        // add models and initialize transform
        for (auto rb : rbs) {
            pe::Array<int> ids;
            switch (rb->getCollisionShape()->getType()) {
                case pe_phys_shape::ShapeType::Box:
                    ids.push_back(pe_intf::Viewer::addCube(((pe_phys_shape::BoxShape*)rb->getCollisionShape())->getSize()));
                    pe_intf::Viewer::updateCubeColor(ids[0], pe::Vector3(0.3, 0.8, 0.8));
                    pe_intf::Viewer::updateCubeTransform(ids[0], rb->getTransform());
                    break;
                case pe_phys_shape::ShapeType::Sphere:
                    ids.push_back(pe_intf::Viewer::addSphere(((pe_phys_shape::SphereShape*)rb->getCollisionShape())->getRadius()));
                    pe_intf::Viewer::updateSphereColor(ids[0], pe::Vector3(0.8, 0.3, 0.3));
                    pe_intf::Viewer::updateSphereTransform(ids[0], rb->getTransform());
                    break;
                case pe_phys_shape::ShapeType::Cylinder:
                    ids.push_back(pe_intf::Viewer::addCylinder(((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getRadius(),
                                                      ((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getHeight()));
                    pe_intf::Viewer::updateCylinderColor(ids[0], pe::Vector3(0.3, 0.8, 0.3));
                    pe_intf::Viewer::updateCylinderTransform(ids[0], rb->getTransform());
                    break;
                case pe_phys_shape::ShapeType::ConvexMesh:
                    ids.push_back(pe_intf::Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)rb->getCollisionShape())->getMesh()));
                    pe_intf::Viewer::updateMeshColor(ids[0], pe::Vector3(0.8, 0.8, 0.3));
                    pe_intf::Viewer::updateMeshTransform(ids[0], rb->getTransform());
                    break;
                case pe_phys_shape::ShapeType::Compound:
                    int i = 0;
                    for (auto& s : ((pe_phys_shape::CompoundShape*)rb->getCollisionShape())->getShapes()) {
                        switch (s.shape->getType()) {
                            case pe_phys_shape::ShapeType::Box:
                                ids.push_back(pe_intf::Viewer::addCube(((pe_phys_shape::BoxShape*)s.shape)->getSize()));
                                pe_intf::Viewer::updateCubeColor(ids[i], pe::Vector3(0.3, 0.8, 0.8));
                                pe_intf::Viewer::updateCubeTransform(ids[i++], rb->getTransform() * s.local_transform);
                                break;
                            case pe_phys_shape::ShapeType::Sphere:
                                ids.push_back(pe_intf::Viewer::addSphere(((pe_phys_shape::SphereShape*)s.shape)->getRadius()));
                                pe_intf::Viewer::updateSphereColor(ids[i], pe::Vector3(0.8, 0.3, 0.3));
                                pe_intf::Viewer::updateSphereTransform(ids[i++], rb->getTransform() * s.local_transform);
                                break;
                            case pe_phys_shape::ShapeType::Cylinder:
                                ids.push_back(pe_intf::Viewer::addCylinder(((pe_phys_shape::CylinderShape*)s.shape)->getRadius(),
                                                                  ((pe_phys_shape::CylinderShape*)s.shape)->getHeight()));
                                pe_intf::Viewer::updateCylinderColor(ids[i], pe::Vector3(0.3, 0.8, 0.3));
                                pe_intf::Viewer::updateCylinderTransform(ids[i++], rb->getTransform() * s.local_transform);
                                break;
                            case pe_phys_shape::ShapeType::ConvexMesh:
                                ids.push_back(pe_intf::Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)s.shape)->getMesh()));
                                pe_intf::Viewer::updateMeshColor(ids[i], pe::Vector3(0.8, 0.8, 0.3));
                                pe_intf::Viewer::updateMeshTransform(ids[i++], rb->getTransform() * s.local_transform);
                            default:
                                break;
                        }
                    }
                    break;
            }
            _id_map[rb] = ids;
            pairs.emplace_back(id, rb);
        }

        // set color for each model
        for (auto& rb : pairs) {
            if (rb.second->getTag().substr(0, 6) == "color:") {
                std::stringstream ss(rb.second->getTag().substr(6));
                pe::Real r, g, b;
                char delim;
                ss >> r >> delim >> g >> delim >> b;
                if (ss.eof()) {
                    pe_intf::Viewer::updateCubeColor(rb.first, pe::Vector3(r, g, b));
                    pe_intf::Viewer::updateSphereColor(rb.first, pe::Vector3(r, g, b));
                    pe_intf::Viewer::updateCylinderColor(rb.first, pe::Vector3(r, g, b));
                    pe_intf::Viewer::updateMeshColor(rb.first, pe::Vector3(r, g, b));
                    continue;
                } else {
                    std::cerr << "invalid color tag: " << r << " " << g << " " << b << std::endl;
                }
            }
            if (rb.second->isKinematic()) {
                pe_intf::Viewer::updateCubeColor(rb.first, pe::Vector3(0.3, 0.8, 0.8));
                pe_intf::Viewer::updateSphereColor(rb.first, pe::Vector3(0.3, 0.8, 0.8));
                pe_intf::Viewer::updateCylinderColor(rb.first, pe::Vector3(0.3, 0.8, 0.8));
                pe_intf::Viewer::updateMeshColor(rb.first, pe::Vector3(0.3, 0.8, 0.8));
                continue;
            }
            switch (rb.second->getCollisionShape()->getType()) {
                case pe_phys_shape::ShapeType::Box:
                    pe_intf::Viewer::updateCubeColor(rb.first, pe::Vector3(0.3, 0.3, 0.8));
                    break;
                case pe_phys_shape::ShapeType::Sphere:
                    pe_intf::Viewer::updateSphereColor(rb.first, pe::Vector3(0.8, 0.3, 0.3));
                    break;
                case pe_phys_shape::ShapeType::Cylinder:
                    pe_intf::Viewer::updateCylinderColor(rb.first, pe::Vector3(0.3, 0.8, 0.3));
                    break;
                case pe_phys_shape::ShapeType::ConvexMesh:
                    pe_intf::Viewer::updateMeshColor(rb.first, pe::Vector3(0.8, 0.8, 0.3));
                    break;
                case pe_phys_shape::ShapeType::Compound:
                    pe_intf::Viewer::updateMeshColor(rb.first, pe::Vector3(0.3, 0.8, 0.8));
                    break;
            }
        }
    }

    template <UseViewer UV>
    void Simulator<UV>::removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
        for (auto rb : rbs) {
            if (_id_map.find(rb) != _id_map.end()) {
                switch (rb->getCollisionShape()->getType()) {
                    case pe_phys_shape::ShapeType::Box:
                        pe_intf::Viewer::removeCube(_id_map[rb]);
                        break;
                    case pe_phys_shape::ShapeType::Sphere:
                        pe_intf::Viewer::removeSphere(_id_map[rb]);
                        break;
                    case pe_phys_shape::ShapeType::Cylinder:
                        pe_intf::Viewer::removeCylinder(_id_map[rb]);
                        break;
                    case pe_phys_shape::ShapeType::ConvexMesh:
                        pe_intf::Viewer::removeMesh(_id_map[rb]);
                        break;
                    case pe_phys_shape::ShapeType::Compound:
                        for (auto& s : ((pe_phys_shape::CompoundShape*)rb->getCollisionShape())->getShapes()) {
                            switch (s.shape->getType()) {
                                case pe_phys_shape::ShapeType::Box:
                                    pe_intf::Viewer::removeCube(_id_map[rb]);
                                    break;
                                case pe_phys_shape::ShapeType::Sphere:
                                    pe_intf::Viewer::removeSphere(_id_map[rb]);
                                    break;
                                case pe_phys_shape::ShapeType::Cylinder:
                                    pe_intf::Viewer::removeCylinder(_id_map[rb]);
                                    break;
                                case pe_phys_shape::ShapeType::ConvexMesh:
                                    pe_intf::Viewer::removeMesh(_id_map[rb]);
                                default:
                                    break;
                            }
                        }
                        break;
                }
                _id_map[rb] = -1;
            }
        }
    }

} // namespace pe_intf

#define PE_SIM_MAIN(Simulator, Dt, MaxFrame) \
int main() { \
    Simulator sim; \
    sim.start(Dt, MaxFrame); \
    return 0; \
}
