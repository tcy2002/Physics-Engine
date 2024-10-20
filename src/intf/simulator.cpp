template <UseViewer UV>
void Simulator<UV>::start(int target_frame_rate) {
    if (target_frame_rate <= 0) {
        PE_LOG_ERROR << "Invalid target frame rate: " << target_frame_rate << PE_ENDL;
        return;
    }

    pe::Real dt = 1.0 / (pe::Real)target_frame_rate;
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
    while (true) {
        auto t = COMMON_GetTickCount();

        _world.step();

#   if false
        static pe::Array<int> ids;
        for (auto id : ids) {
            pe_intf::Viewer::remove(id);
        }
        ids.clear();
        for (auto cr : _world.getContactResults()) {
            for (int i = 0; i < cr->getPointSize(); i++) {
                auto p = cr->getContactPoint(i).getWorldPos();
                auto id = pe_intf::Viewer::addSphere(0.03);
                pe_intf::Viewer::updateTransform(id, pe_phys_shape::ShapeType::Sphere, pe::Transform(pe::Matrix3::identity(), p));
                ids.push_back(id);
            }
        }
#   endif

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
        frame++;
    }

    auto end = COMMON_GetTickCount();
    pe::Real total_time = (pe::Real)(end - start) * pe::Real(0.001);
    std::cout << "frame count: " << frame << ", fps: " << (pe::Real)frame / total_time << std::endl;
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
        if (rb.second.empty()) {
            continue;
        }
        auto type = rb.first->getCollisionShape()->getType();
        if (type != pe_phys_shape::ShapeType::Compound) {
            updateColor(rb.second[0], type, rb.first->getTag(), rb.first->isKinematic() || rb.first->isSleep());
            if (!rb.first->isSleep()) {
                pe_intf::Viewer::updateTransform(rb.second[0], type, rb.first->getTransform());
            }
        } else {
            int i = 0;
            for (auto& s : ((pe_phys_shape::CompoundShape*)rb.first->getCollisionShape())->getShapes()) {
                updateColor(rb.second[i], s.shape->getType(), rb.first->getTag(), rb.first->isKinematic() || rb.first->isSleep());
                if (!rb.first->isSleep()) {
                    pe_intf::Viewer::updateTransform(rb.second[i], s.shape->getType(), rb.first->getTransform() * s.local_transform);
                }
                i++;
            }
        }
    }
    return true;
}

template <UseViewer UV>
void Simulator<UV>::addModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
    // add models and initialize transform
    for (auto rb : rbs) {
        pe::Array<int> ids;
        auto type = rb->getCollisionShape()->getType();
        switch (type) {
            case pe_phys_shape::ShapeType::Box:
                ids.push_back(pe_intf::Viewer::addCube(((pe_phys_shape::BoxShape*)rb->getCollisionShape())->getSize()));
                pe_intf::Viewer::updateTransform(ids[0], type, rb->getTransform());
                updateColor(ids[0], type, rb->getTag(), rb->isKinematic());
                break;
            case pe_phys_shape::ShapeType::Sphere:
                ids.push_back(pe_intf::Viewer::addSphere(((pe_phys_shape::SphereShape*)rb->getCollisionShape())->getRadius()));
                pe_intf::Viewer::updateTransform(ids[0], type, rb->getTransform());
                updateColor(ids[0], pe_phys_shape::ShapeType::Sphere, rb->getTag(), rb->isKinematic());
                break;
            case pe_phys_shape::ShapeType::Cylinder:
                ids.push_back(pe_intf::Viewer::addCylinder(((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getRadius(),
                                                           ((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getHeight()));
                pe_intf::Viewer::updateTransform(ids[0], type, rb->getTransform());
                updateColor(ids[0], pe_phys_shape::ShapeType::Cylinder, rb->getTag(), rb->isKinematic());
                break;
            case pe_phys_shape::ShapeType::ConvexMesh:
                ids.push_back(pe_intf::Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)rb->getCollisionShape())->getMesh()));
                pe_intf::Viewer::updateTransform(ids[0], type, rb->getTransform());
                updateColor(ids[0], pe_phys_shape::ShapeType::ConvexMesh, rb->getTag(), rb->isKinematic());
                break;
            case pe_phys_shape::ShapeType::Compound:
                int i = 0;
                for (auto& s : ((pe_phys_shape::CompoundShape*)rb->getCollisionShape())->getShapes()) {
                    auto sub_type = s.shape->getType();
                    switch (sub_type) {
                        case pe_phys_shape::ShapeType::Box:
                            ids.push_back(pe_intf::Viewer::addCube(((pe_phys_shape::BoxShape*)s.shape)->getSize()));
                            pe_intf::Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                            updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                            break;
                        case pe_phys_shape::ShapeType::Sphere:
                            ids.push_back(pe_intf::Viewer::addSphere(((pe_phys_shape::SphereShape*)s.shape)->getRadius()));
                            pe_intf::Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                            updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                            break;
                        case pe_phys_shape::ShapeType::Cylinder:
                            ids.push_back(pe_intf::Viewer::addCylinder(((pe_phys_shape::CylinderShape*)s.shape)->getRadius(),
                                                                       ((pe_phys_shape::CylinderShape*)s.shape)->getHeight()));
                            pe_intf::Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
                            updateColor(ids[i++], sub_type, rb->getTag(), rb->isKinematic());
                            break;
                        case pe_phys_shape::ShapeType::ConvexMesh:
                            ids.push_back(pe_intf::Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)s.shape)->getMesh()));
                            pe_intf::Viewer::updateTransform(ids[i], sub_type, rb->getTransform() * s.local_transform);
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

template <UseViewer UV>
void Simulator<UV>::updateColor(int id, pe_phys_shape::ShapeType type, const std::string &tag, bool kinematic) {
    if (tag.substr(0, 6) == "color:") {
        std::stringstream ss(tag.substr(6));
        pe::Real r, g, b;
        char delim;
        ss >> r >> delim >> g >> delim >> b;
        if (ss.eof()) {
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(r, g, b));
            return;
        } else {
            std::cerr << "invalid color tag: " << r << " " << g << " " << b << std::endl;
        }
    }

    if (kinematic) {
        pe_intf::Viewer::updateColor(id, type, pe::Vector3(0.3, 0.8, 0.8));
        return;
    }

    switch (type) {
        case pe_phys_shape::ShapeType::Box:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(0.3, 0.3, 0.8));
            break;
        case pe_phys_shape::ShapeType::Sphere:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(0.8, 0.3, 0.3));
            break;
        case pe_phys_shape::ShapeType::Cylinder:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(0.3, 0.8, 0.3));
            break;
        case pe_phys_shape::ShapeType::ConvexMesh:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(0.8, 0.8, 0.3));
        default:
            break;
    }
}

template <UseViewer UV>
void Simulator<UV>::removeModels(const pe::Array<pe_phys_object::RigidBody*>& rbs) {
    for (auto rb : rbs) {
        if (_id_map.find(rb) != _id_map.end()) {
            for (auto id : _id_map[rb]) {
                pe_intf::Viewer::remove(id);
            }
            _id_map[rb] = {};
        }
    }
}