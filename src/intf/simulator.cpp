template<UseViewer UV>
void Simulator<UV>::saveScene() {
    std::string json_file = utils::FileSystem::fileDialog(1, PE_DATA_PATH, "Scene Files", "json");
    if (json_file.empty()) {
        PE_LOG_ERROR << "No scene file specified." << PE_ENDL;
        return;
    }

    PE_LOG_INFO << "Save scene to: " << json_file << PE_ENDL;

    //////////////////////////////////////////////////////////////////////////
    // save configuration
    //////////////////////////////////////////////////////////////////////////
    nlohmann::json data;
    data["Configuration"]["gravity"] = {_world.getGravity().x, _world.getGravity().y, _world.getGravity().z};
    data["Configuration"]["sleep_linear_velocity2_threshold"] = _world.getSleepLinVel2Threshold();
    data["Configuration"]["sleep_angular_velocity2_threshold"] = _world.getSleepAngVel2Threshold();
    data["Configuration"]["sleep_time_threshold"] = _world.getSleepTimeThreshold();
    PE_LOG_INFO << "Saved configuration." << PE_ENDL;

    //////////////////////////////////////////////////////////////////////////
    // save rigidbodies
    //////////////////////////////////////////////////////////////////////////
    auto& rbs = data["RigidBodies"];
    for (auto rb : _world.getRigidBodies()) {
        nlohmann::json rb_data;
        rb_data["name"] = rb->getName();
        rb_data["tag"] = rb->getTag();
        rb_data["ignore_collision"] = rb->isIgnoreCollision();
        rb_data["kinematic"] = rb->isKinematic();
        if (!rb->isKinematic()) {
            rb_data["linear_damping"] = rb->getLinearDamping();
            rb_data["angular_damping"] = rb->getAngularDamping();
            rb_data["linear_velocity"] = {rb->getLinearVelocity().x, rb->getLinearVelocity().y, rb->getLinearVelocity().z};
            rb_data["angular_velocity"] = {rb->getAngularVelocity().x, rb->getAngularVelocity().y, rb->getAngularVelocity().z};
        }
        rb_data["lifetime"] = rb->getLifeTime();
        auto transform = rb->getTransform();
        rb_data["position"] = {transform.getOrigin().x, transform.getOrigin().y, transform.getOrigin().z};
        auto rot = transform.getBasis();
        rb_data["rotation"] = {rot[0][0], rot[0][1], rot[0][2],
                                 rot[1][0], rot[1][1], rot[1][2],
                                 rot[2][0], rot[2][1], rot[2][2]};
        rb_data["mass"] = rb->getMass();
        rb_data["friction"] = rb->getFrictionCoeff();
        rb_data["restitution"] = rb->getRestitutionCoeff();
        auto shape = rb->getCollisionShape();
        switch (shape->getType()) {
            case pe_phys_shape::ShapeType::Box: {
                rb_data["type"] = "box";
                auto size = static_cast<pe_phys_shape::BoxShape*>(shape)->getSize();
                rb_data["scale"] = {size.x, size.y, size.z};
                break;
            }
            case pe_phys_shape::ShapeType::Sphere: {
                rb_data["type"] = "sphere";
                auto radius = static_cast<pe_phys_shape::SphereShape*>(shape)->getRadius();
                rb_data["scale"] = {radius};
                break;
            }
            case pe_phys_shape::ShapeType::Cylinder: {
                rb_data["type"] = "cylinder";
                auto radius = static_cast<pe_phys_shape::CylinderShape*>(shape)->getRadius();
                auto height = static_cast<pe_phys_shape::CylinderShape*>(shape)->getHeight();
                rb_data["scale"] = {radius, height};
                break;
            }
            case pe_phys_shape::ShapeType::ConvexMesh: {
                rb_data["type"] = "convex";
                rb_data["mesh"] = ""; // user should change this in the json file
                break;
            }
            case pe_phys_shape::ShapeType::ConcaveMesh: {
                rb_data["type"] = "concave";
                rb_data["mesh"] = ""; // user should change this in the json file
                break;
            }
            case pe_phys_shape::ShapeType::Compound: {
                rb_data["type"] = "compound";
                auto compound = static_cast<pe_phys_shape::CompoundShape*>(shape);
                auto& shapes = compound->getShapes();
                auto& shapes_data = rb_data["shapes"];
                for (auto& s : shapes) {
                    nlohmann::json shape_data;
                    shape_data["mass_ratio"] = s.mass_ratio;
                    shape_data["position"] = {s.local_transform.getOrigin().x, s.local_transform.getOrigin().y, s.local_transform.getOrigin().z};
                    auto rot = s.local_transform.getBasis();
                    shape_data["rotation"] = {rot[0][0], rot[0][1], rot[0][2],
                                              rot[1][0], rot[1][1], rot[1][2],
                                              rot[2][0], rot[2][1], rot[2][2]};
                    switch (s.shape->getType()) {
                        case pe_phys_shape::ShapeType::Box: {
                            shape_data["type"] = "box";
                            auto size = static_cast<pe_phys_shape::BoxShape*>(s.shape)->getSize();
                            shape_data["scale"] = {size.x, size.y, size.z};
                            break;
                        }
                        case pe_phys_shape::ShapeType::Sphere: {
                            shape_data["type"] = "sphere";
                            auto radius = static_cast<pe_phys_shape::SphereShape*>(s.shape)->getRadius();
                            shape_data["scale"] = {radius};
                            break;
                        }
                        case pe_phys_shape::ShapeType::Cylinder: {
                            shape_data["type"] = "cylinder";
                            auto radius = static_cast<pe_phys_shape::CylinderShape*>(s.shape)->getRadius();
                            auto height = static_cast<pe_phys_shape::CylinderShape*>(s.shape)->getHeight();
                            shape_data["scale"] = {radius, height};
                            break;
                        }
                        case pe_phys_shape::ShapeType::ConvexMesh: {
                            shape_data["type"] = "convex";
                            shape_data["mesh"] = ""; // user should change this in the json file
                            break;
                        }
                        default:
                            break; // concave and compound shapes are not supported for compound sub-shapes
                    }
                    shapes_data.push_back(shape_data);
                }
            }
        }
        rbs.push_back(rb_data);
    }
    PE_LOG_INFO << "Saved " << rbs.size() << " rigidbodies." << PE_ENDL;

    //////////////////////////////////////////////////////////////////////////
    // save constraints
    //////////////////////////////////////////////////////////////////////////
    // not implemented

    std::ofstream file(json_file);
    file << data.dump(4);
    file.close();
    PE_LOG_INFO << "Scene saved." << PE_ENDL;
}

template<UseViewer UV>
bool Simulator<UV>::loadScene(int argc, char** argv) {
    std::string json_file;
    if (argc == 2) {
        json_file = argv[1];
    } else if (argc == 1) {
        json_file = utils::FileSystem::fileDialog(0, PE_DATA_PATH, "Scene Files", "json");
        if (json_file.empty()) {
            PE_LOG_ERROR << "No scene file specified." << PE_ENDL;
            return false;
        }
    } else {
        PE_LOG_ERROR << "No scene file specified." << PE_ENDL;
        return false;
    }

    PE_LOG_INFO << "Load scene: " << json_file << PE_ENDL;

    std::ifstream file(json_file);
    if (!file.is_open()) {
        PE_LOG_ERROR << "Failed to open file: " << json_file << PE_ENDL;
        return false;
    }

    nlohmann::json data;
    try {
        data = nlohmann::json::parse(file);
    } catch (const std::exception& e) {
        PE_LOG_ERROR << "Failed to parse json file: " << e.what() << PE_ENDL;
        return false;
    }

    //////////////////////////////////////////////////////////////////////////
    // read configuration
    //////////////////////////////////////////////////////////////////////////
    if (data.find("Configuration") != data.end()) {
        auto& config = data["Configuration"];
        if (config.find("gravity") != config.end()) {
            _world.setGravity(pe::Vector3(config["gravity"][0], config["gravity"][1], config["gravity"][2]));
        }
        if (config.find("sleep_linear_velocity2_threshold") != config.end()) {
            _world.setSleepLinVel2Threshold(config["sleep_linear_velocity2_threshold"]);
        }
        if (config.find("sleep_angular_velocity2_threshold") != config.end()) {
            _world.setSleepAngVel2Threshold(config["sleep_angular_velocity2_threshold"]);
        }
        if (config.find("sleep_time_threshold") != config.end()) {
            _world.setSleepTimeThreshold(config["sleep_time_threshold"]);
        }
    }
    PE_LOG_INFO << "Loaded configuration." << PE_ENDL;

    //////////////////////////////////////////////////////////////////////////
    // read rigidbodies
    //////////////////////////////////////////////////////////////////////////
    int k = 0;
    if (data.find("RigidBodies") != data.end()) {
        auto& rbs = data["RigidBodies"];
        if (rbs.type() != nlohmann::json::value_t::array) {
            PE_LOG_ERROR << "RigidBodies should be an array." << PE_ENDL;
            return false;
        }
        for (auto& rb : rbs) {
            auto rb_obj = new pe_phys_object::RigidBody();
            if (rb.find("name") != rb.end()) rb_obj->setName(rb["name"]);
            if (rb.find("tag") != rb.end()) rb_obj->setTag(rb["tag"]);
            if (rb.find("ignore_collision") != rb.end()) rb_obj->setIgnoreCollision(rb["ignore_collision"]);
            if (rb.find("kinematic") != rb.end()) rb_obj->setKinematic(rb["kinematic"]);
            if (!rb_obj->isKinematic()) {
                if (rb.find("linear_damping") != rb.end()) rb_obj->setLinearDamping(rb["linear_damping"]);
                if (rb.find("angular_damping") != rb.end()) rb_obj->setAngularDamping(rb["angular_damping"]);
                if (rb.find("linear_velocity") != rb.end())
                    rb_obj->setLinearVelocity(pe::Vector3(rb["linear_velocity"][0], rb["linear_velocity"][1], rb["linear_velocity"][2]));
                if (rb.find("angular_velocity") != rb.end())
                    rb_obj->setAngularVelocity(pe::Vector3(rb["angular_velocity"][0], rb["angular_velocity"][1], rb["angular_velocity"][2]));
            }
            if (rb.find("lifetime") != rb.end()) {
                auto lifetime = rb["lifetime"].get<pe::Real>();
                lifetime = lifetime < 0 ? PE_REAL_MAX : lifetime;
                rb_obj->setLifeTime(lifetime);
            }
            auto transform = pe::Transform::identity();
            if (rb.find("position") != rb.end())
                transform.setOrigin(pe::Vector3(rb["position"][0], rb["position"][1], rb["position"][2]));
            if (rb.find("rotation") != rb.end()) {
                if (rb.find("rotation").value().size() == 4) {
                    auto q = pe::Quaternion(rb["rotation"][0], rb["rotation"][1], rb["rotation"][2], rb["rotation"][3]);
                    transform.setBasis(q.toRotationMatrix());
                } else if (rb.find("rotation").value().size() == 9) {
                    auto m = pe::Matrix3(rb["rotation"][0], rb["rotation"][1], rb["rotation"][2],
                                         rb["rotation"][3], rb["rotation"][4], rb["rotation"][5],
                                         rb["rotation"][6], rb["rotation"][7], rb["rotation"][8]);
                    transform.setBasis(m);
                }
            }
            rb_obj->setTransform(transform);
            if (rb.find("mass") != rb.end()) rb_obj->setMass(rb["mass"]);
            if (rb.find("friction") != rb.end()) rb_obj->setFrictionCoeff(rb["friction"]);
            if (rb.find("restitution") != rb.end()) rb_obj->setRestitutionCoeff(rb["restitution"]);
            if (rb.find("type") != rb.end()) {
                auto type = rb["type"].get<std::string>();
                pe_phys_shape::Shape* shape;
                if (type == "box") {
                    auto size = pe::Vector3(rb["scale"][0], rb["scale"][1], rb["scale"][2]);
                    shape = new pe_phys_shape::BoxShape(size);
                } else if (type == "sphere") {
                    auto radius = rb["scale"][0].get<pe::Real>();
                    shape = new pe_phys_shape::SphereShape(radius);
                } else if (type == "cylinder") {
                    auto radius = rb["scale"][0].get<pe::Real>();
                    auto height = rb["scale"][1].get<pe::Real>();
                    shape = new pe_phys_shape::CylinderShape(radius, height);
                } else if (type == "convex" || type == "concave") {
                    auto obj_path = rb["mesh"].get<std::string>();
                    auto size = pe::Vector3(rb["scale"][0], rb["scale"][1], rb["scale"][2]);
                    pe::Mesh mesh;
                    pe::Mesh::loadFromObj(obj_path, mesh, size);
                    if (type == "convex") {
                        shape = new pe_phys_shape::ConvexMeshShape();
                        ((pe_phys_shape::ConvexMeshShape*)shape)->setMesh(mesh);
                    } else {
                        shape = new pe_phys_shape::ConcaveMeshShape();
                        ((pe_phys_shape::ConcaveMeshShape*)shape)->setMesh(mesh);
                    }
                } else if (type == "compound") {
                    shape = new pe_phys_shape::CompoundShape();
                    auto& shapes = rb["shapes"];
                    if (shapes.type() != nlohmann::json::value_t::array) {
                        PE_LOG_ERROR << "Compound sub-shapes should be an array." << PE_ENDL;
                        delete rb_obj;
                        continue;
                    }
                    for (auto& s : shapes) {
                        pe::Real mass_ratio = 1.0;
                        pe::Transform local_transform = pe::Transform::identity();
                        if (s.find("mass_ratio") != s.end()) mass_ratio = s["mass_ratio"];
                        if (s.find("position") != s.end())
                            local_transform.setOrigin(pe::Vector3(s["position"][0], s["position"][1], s["position"][2]));
                        if (s.find("rotation") != s.end()) {
                            if (s.find("rotation").value().size() == 4) {
                                auto q = pe::Quaternion(s["rotation"][0], s["rotation"][1], s["rotation"][2], s["rotation"][3]);
                                local_transform.setBasis(q.toRotationMatrix());
                            } else if (s.find("rotation").value().size() == 9) {
                                auto m = pe::Matrix3(s["rotation"][0], s["rotation"][1], s["rotation"][2],
                                                     s["rotation"][3], s["rotation"][4], s["rotation"][5],
                                                     s["rotation"][6], s["rotation"][7], s["rotation"][8]);
                                local_transform.setBasis(m);
                            }
                        }
                        pe_phys_shape::Shape* sub_shape;
                        auto sub_type = s["type"].get<std::string>();
                        if (sub_type == "box") {
                            auto size = pe::Vector3(s["scale"][0], s["scale"][1], s["scale"][2]);
                            sub_shape = new pe_phys_shape::BoxShape(size);
                        } else if (sub_type == "sphere") {
                            auto radius = s["scale"][0].get<pe::Real>();
                            sub_shape = new pe_phys_shape::SphereShape(radius);
                        } else if (sub_type == "cylinder") {
                            auto radius = s["scale"][0].get<pe::Real>();
                            auto height = s["scale"][1].get<pe::Real>();
                            sub_shape = new pe_phys_shape::CylinderShape(radius, height);
                        } else if (sub_type == "convex") {
                            auto obj_path = s["mesh"].get<std::string>();
                            auto size = pe::Vector3(s["scale"][0], s["scale"][1], s["scale"][2]);
                            pe::Mesh mesh;
                            pe::Mesh::loadFromObj(obj_path, mesh, size);
                            shape = new pe_phys_shape::ConvexMeshShape();
                            ((pe_phys_shape::ConvexMeshShape*)shape)->setMesh(mesh);
                        } else if (sub_type == "compound" || sub_type == "concave") {
                            PE_LOG_ERROR << "Compound sub-shapes cannot be compound or concave." << PE_ENDL;
                            delete rb_obj;
                            continue;
                        } else {
                            PE_LOG_ERROR << "Invalid compound sub-shape type: " << sub_type << PE_ENDL;
                            delete rb_obj;
                            continue;
                        }
                        ((pe_phys_shape::CompoundShape*)shape)->addShape(local_transform, mass_ratio, sub_shape);
                    }
                } else {
                    PE_LOG_ERROR << "Invalid rigidbody type: " << type << PE_ENDL;
                    delete rb_obj;
                    continue;
                }
                rb_obj->setCollisionShape(shape);
            } else {
                PE_LOG_ERROR << "RigidBody type not specified." << PE_ENDL;
                delete rb_obj;
                continue;
            }
            _world.addRigidBody(rb_obj);
            k++;
        }
    }
    PE_LOG_INFO << "Loaded " << k << " rigidbodies." << PE_ENDL;

    //////////////////////////////////////////////////////////////////////////
    // read constraints
    //////////////////////////////////////////////////////////////////////////
    // not implemented

    PE_LOG_INFO << "Scene loaded." << PE_ENDL;
    return true;
}

template <UseViewer UV>
void Simulator<UV>::start(int target_frame_rate) {
    if (target_frame_rate <= 0) {
        PE_LOG_ERROR << "Invalid target frame rate: " << target_frame_rate << PE_ENDL;
        return;
    }

    pe::Real dt = pe::Real(1.0) / (pe::Real)target_frame_rate;
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

	pe::Real total_step_time = 0;
    while (true) {
        auto t = COMMON_GetTickCount();

		auto step_start = COMMON_GetMicroseconds();
        _world.step();
		auto step_end = COMMON_GetMicroseconds();
		total_step_time += (pe::Real)(step_end - step_start);

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
        if (target_dt > actual_dt) {
            COMMON_Sleep(target_dt - actual_dt);
        }
        frame++;

#   if false
        static pe::Array<int> ids;
        for (auto id : ids) {
            pe_intf::Viewer::remove(id);
        }
        ids.clear();
        for (auto cr : _world.getContactResults()) {
            if (cr->getObjectA()->getGlobalId() == 1 || cr->getObjectB()->getGlobalId() == 1) continue;
            for (int i = 0; i < cr->getPointSize(); i++) {
                auto p = cr->getContactPoint(i).getWorldPos();
                auto n = cr->getContactPoint(i).getWorldNormal();
                auto d = cr->getContactPoint(i).getDistance();
                auto id = pe_intf::Viewer::addSphere(0.03);
                pe_intf::Viewer::updateTransform(id, pe_phys_shape::ShapeType::Sphere, pe::Transform(pe::Matrix3::identity(), p));
                ids.push_back(id);
            }
        }
#   endif
    }

    auto end = COMMON_GetTickCount();
    pe::Real total_time = (pe::Real)(end - start) * pe::Real(0.001);
	std::cout << "step time: " << total_step_time * pe::Real(0.000001) << "s" << std::endl;
    std::cout << "frame count: " << frame << ", fps: " << (pe::Real)frame / total_time << std::endl;
    std::cout << "total time: " << total_time << "s" << std::endl;
    std::cout << "update status time: " << _world.update_status_time << "s " << _world.update_status_time / total_time << std::endl;
    std::cout << "broad phase time: " << _world.broad_phase_time << "s " << _world.broad_phase_time / total_time << std::endl;
    std::cout << "narrow phase time: " << _world.narrow_phase_time << "s " << _world.narrow_phase_time / total_time << std::endl;
    std::cout << "constraint solver time: " << _world.constraint_solver_time << "s " << _world.constraint_solver_time / total_time << std::endl;
    pe::Real other_time = total_time - _world.update_status_time - _world.broad_phase_time - _world.narrow_phase_time - _world.constraint_solver_time;
    std::cout << "other time: " << other_time << "s" << " " << other_time / total_time << std::endl;
}

static void toggleLine() {
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

    return true;
}

template <UseViewer UV>
bool Simulator<UV>::renderStep() {
    if (!pe_intf::Viewer::isOpen() || pe_intf::Viewer::getKeyState(27) == 0) {
        pe_intf::Viewer::close();
        return false;
    }
    static bool blocking = true;
    if (blocking) {
        while (pe_intf::Viewer::getKeyState('r') != 0 && pe_intf::Viewer::getKeyState('t') != 0) {
            COMMON_Sleep(1);
            toggleLine();
            if (pe_intf::Viewer::getKeyState('x') == 2) {
                blocking = false;
                break;
            }
        }
        if (pe_intf::Viewer::getKeyState('t') == 0) {
            COMMON_Sleep(300);
        }
    } else {
        toggleLine();
    }

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
            case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
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
                        case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
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
        pe_intf::Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.3), pe::Real(0.8), pe::Real(0.8)));
        return;
    }

    switch (type) {
        case pe_phys_shape::ShapeType::Box:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.3), pe::Real(0.3), pe::Real(0.8)));
            break;
        case pe_phys_shape::ShapeType::Sphere:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.8), pe::Real(0.3), pe::Real(0.3)));
            break;
        case pe_phys_shape::ShapeType::Cylinder:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.3), pe::Real(0.8), pe::Real(0.3)));
            break;
        case pe_phys_shape::ShapeType::ConvexMesh: case pe_phys_shape::ShapeType::ConcaveMesh:
            pe_intf::Viewer::updateColor(id, type, pe::Vector3(pe::Real(0.8), pe::Real(0.8), pe::Real(0.3)));
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