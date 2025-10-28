#include "simulator.h"
#include "phys/shape/default_mesh.h"

namespace pe_intf {

    static void printWelcomeMessage() {
        PE_LOG_CUSTOM_INFO << "Press `x` to start simulation" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `r` to simulate for a period while pressing" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `t` to simulate a single step" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `c` to show edges" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `n` to download the simulation data to json files (default in ./data/)" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Press `esc` to exit" << PE_CUSTOM_ENDL;
        PE_LOG_CUSTOM_INFO << "Camera control: UE mode" << PE_CUSTOM_ENDL;
    }

    void Simulator::showDebugPoints() {
        static pe::Array<int> ids;
            for (auto id : ids) {
                Viewer::remove(id);
            }
            ids.clear();
            if (_show_debug_points) {
                for (auto cr : _world.getContactResults()) {
                    for (int i = 0; i < cr->getPointSize(); i++) {
                        auto p = cr->getContactPoint(i).getWorldPos();
                        auto id = Viewer::addSphere(0.1);
                        Viewer::updateTransform(id, pe_phys_shape::ShapeType::ST_Sphere, pe::Transform(pe::Matrix3::identity(), p));
                        Viewer::updateColor(id, pe_phys_shape::ShapeType::ST_Sphere, pe::Vector3(0.8, 0.3, 0.3));
                        ids.push_back(id);
                    }
                }
                PE_LOG_DEBUG << "contact point count: " << ids.size() << PE_ENDL;
            }
    }

    void Simulator::start() {
        printWelcomeMessage();

        if (target_framerate <= 0) {
            PE_LOG_CUSTOM_ERROR << "Invalid target frame rate: " << target_framerate << PE_CUSTOM_ENDL;
            return;
        }

        pe::Real dt = R(1.0) / R(target_framerate);
        _world.setDt(dt);
        init();
        if (use_gui) {
            if (!renderInit()) {
                return;
            }
        }

        const uint64_t target_tick = dt * 1000000;
        uint64_t frame = 0;
	    uint64_t total_step_tick = 0;
        uint64_t total_tick = 0;

        while (true) {
            auto t = COMMON_GetMicroTickCount();

		    auto step_start = COMMON_GetMicroTickCount();
            _world.step();
		    auto step_end = COMMON_GetMicroTickCount();
		    total_step_tick += step_end - step_start;

            if (use_gui) {
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

            uint64_t blocking_time = 0;
            if (use_gui) {
                if (!renderStep(blocking_time)) {
                    break;
                }
            }

            ++frame;
            auto actual_tick = COMMON_GetMicroTickCount() - t - blocking_time;
            if (target_tick * frame > total_tick + actual_tick) {
                COMMON_USleep(target_tick * frame - total_tick - actual_tick);
            }
            total_tick += COMMON_GetMicroTickCount() - t - blocking_time;
            if (frame >= max_frame) {
                break;
            }
        }

        pe::Real total_time = R(total_tick) * R(0.000001);
        pe::Real step_time = R(total_step_tick) * R(0.000001);
        std::cout << "frame count: " << frame << std::endl;
	    std::cout << "simulation time: " << step_time << "s, simulation fps: " << R(frame) / step_time << std::endl;
        std::cout << "total time: " << total_time << "s, fps: " << R(frame) / total_time << std::endl;
        std::cout << "update status time: " << _world.update_status_time << "s " << _world.update_status_time / total_time << std::endl;
        std::cout << "broad phase time: " << _world.broad_phase_time << "s " << _world.broad_phase_time / total_time << std::endl;
        std::cout << "narrow phase time: " << _world.narrow_phase_time << "s " << _world.narrow_phase_time / total_time << std::endl;
        std::cout << "constraint solver time: " << _world.constraint_solver_time << "s " << _world.constraint_solver_time / total_time << std::endl;
        pe::Real other_time = total_time - _world.update_status_time - _world.broad_phase_time - _world.narrow_phase_time - _world.constraint_solver_time;
        std::cout << "other time: " << other_time << "s" << " " << other_time / total_time << std::endl;
    }

    static void toggleLine() {
        static bool show_line = false;
        if (Viewer::getKeyState('c') == 2) {
            show_line = !show_line;
            Viewer::showLine(show_line, 1);
        }
    }

    bool Simulator::renderInit() {
        Viewer::open("PhysicsDemo", 800, 600, {0, 10, 20}, 0, (float)(PE_PI / 12.0));

        // initialize models
        addModels(_world.getRigidBodiesToAdd());
        _world.clearRigidBodiesToAdd();

        // wait for the window to open
        while (!Viewer::isOpen()) {
            COMMON_USleep(1000);
        }

        return true;
    }

    bool Simulator::renderStep(uint64_t& blocking_time) {
        for (auto& rb : _id_map) {
            if (rb.second.empty()) {
                continue;
            }
            auto type = rb.first->getCollisionShape()->getType();
            if (type != pe_phys_shape::ShapeType::ST_Compound) {
                updateColor(rb.second[0], type, rb.first->getTag(), rb.first->isKinematic() || rb.first->isSleep());
                if (!rb.first->isSleep()) {
                    Viewer::updateTransform(rb.second[0], type, rb.first->getTransform());
                }
            } else {
                int i = 0;
                for (auto& s : dynamic_cast<pe_phys_shape::CompoundShape *>(rb.first->getCollisionShape())->getShapes()) {
                    updateColor(rb.second[i], s.shape->getType(), rb.first->getTag(), rb.first->isKinematic() || rb.first->isSleep());
                    if (!rb.first->isSleep()) {
                        Viewer::updateTransform(rb.second[i], s.shape->getType(), rb.first->getTransform() * s.local_transform);
                    }
                    i++;
                }
            }
        }

        showDebugPoints();

        if (!Viewer::isOpen() || Viewer::getKeyState(27) == 0) {
            Viewer::close();
            return false;
        }
        auto t = COMMON_GetMicroTickCount();
        static bool blocking = true;
        if (blocking) {
            while (Viewer::getKeyState('r') != 0 && Viewer::getKeyState('t') != 0) {
                COMMON_USleep(1000);
                toggleLine();
                if (Viewer::getKeyState('y') == 2) {
                    _show_debug_points = !_show_debug_points;
                    showDebugPoints();
                }
                if (Viewer::getKeyState('x') == 2) {
                    blocking = false;
                    break;
                }
                if (!Viewer::isOpen() || Viewer::getKeyState(27) == 0) {
                    Viewer::close();
                    return false;
                }
            }
            if (Viewer::getKeyState('t') == 0) {
                COMMON_USleep(300000);
            }
        } else {
            toggleLine();
            if (Viewer::getKeyState('y') == 2) {
                _show_debug_points = !_show_debug_points;
            }
        }
        blocking_time = COMMON_GetMicroTickCount() - t;
        return true;
    }

    void Simulator::addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
        // add models and initialize transform
        for (auto rb : rbs) {
            pe::Array<int> ids;
            auto type = rb->getCollisionShape()->getType();
            switch (type) {
                case pe_phys_shape::ShapeType::ST_Box:
                    ids.push_back(Viewer::addCube(dynamic_cast<pe_phys_shape::BoxShape *>(rb->getCollisionShape())->getSize()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], type, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::ST_Sphere:
                    ids.push_back(Viewer::addSphere(dynamic_cast<pe_phys_shape::SphereShape *>(rb->getCollisionShape())->getRadius()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], pe_phys_shape::ShapeType::ST_Sphere, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::ST_ConvexMesh: case pe_phys_shape::ShapeType::ST_ConcaveMesh:
                    ids.push_back(Viewer::addMesh(dynamic_cast<pe_phys_shape::ConvexMeshShape *>(rb->getCollisionShape())->getMesh()));
                    Viewer::updateTransform(ids[0], type, rb->getTransform());
                    updateColor(ids[0], pe_phys_shape::ShapeType::ST_ConvexMesh, rb->getTag(), rb->isKinematic());
                    break;
                case pe_phys_shape::ShapeType::ST_Compound:
                    int i = 0;
                    for (auto& s : dynamic_cast<pe_phys_shape::CompoundShape *>(rb->getCollisionShape())->getShapes()) {
                        auto sub_type = s.shape->getType();
                        switch (sub_type) {
                            case pe_phys_shape::ShapeType::ST_Box:
                                ids.push_back(Viewer::addCube(dynamic_cast<pe_phys_shape::BoxShape *>(s.shape)->getSize()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                                break;
                            case pe_phys_shape::ShapeType::ST_Sphere:
                                ids.push_back(Viewer::addSphere(dynamic_cast<pe_phys_shape::SphereShape *>(s.shape)->getRadius()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                                break;
                            case pe_phys_shape::ShapeType::ST_ConvexMesh: case pe_phys_shape::ShapeType::ST_ConcaveMesh:
                                ids.push_back(Viewer::addMesh(dynamic_cast<pe_phys_shape::ConvexMeshShape *>(s.shape)->getMesh()));
                                Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                                updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                            default:
                                break;
                        }
                    }
                    break;
            }
            _id_map[rb] = ids;
        }
    }

    void Simulator::updateColor(int id, pe_phys_shape::ShapeType type, const std::string &tag, bool kinematic) {
        auto color_p = tag.find("color:");
        if (color_p != std::string::npos) {
            std::stringstream ss(tag.substr(color_p + 6));
            pe::Real r, g, b;
            char delim;
            ss >> r >> delim >> g >> delim >> b;
            if (ss.good() || ss.eof()) {
                Viewer::updateColor(id, type, pe::Vector3(r, g, b));
                return;
            } else {
                PE_LOG_CUSTOM_ERROR << "invalid color tag" << PE_CUSTOM_ENDL;
            }
        }

        if (kinematic) {
            Viewer::updateColor(id, type, pe::Vector3(0.3, 0.8, 0.8));
            return;
        }

        switch (type) {
            case pe_phys_shape::ShapeType::ST_Box:
                Viewer::updateColor(id, type, pe::Vector3(0.3, 0.3, 0.8));
                break;
            case pe_phys_shape::ShapeType::ST_Sphere:
                Viewer::updateColor(id, type, pe::Vector3(0.8, 0.3, 0.3));
                break;
            case pe_phys_shape::ShapeType::ST_ConvexMesh: case pe_phys_shape::ShapeType::ST_ConcaveMesh:
                Viewer::updateColor(id, type, pe::Vector3(0.8, 0.8, 0.3));
            default:
                break;
        }
    }

    void Simulator::removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
        for (auto rb : rbs) {
            if (_id_map.find(rb) != _id_map.end()) {
                for (auto id : _id_map[rb]) {
                    Viewer::remove(id);
                }
                _id_map[rb] = {};
            }
        }
    }
}
