template <bool UseViewer>
void Simulator<UseViewer>::start(pe::Real dt, int max_frame) {
    _world.setDt(dt);
    init();
    if (UseViewer) {
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
        step();

        if (UseViewer) {
            if (!renderStep()) {
                break;
            }
        }

        auto actual_dt = (int)(COMMON_GetTickCount() - t);
        COMMON_Sleep(target_dt - actual_dt);
        _world.setDt(actual_dt < target_dt ? dt : (pe::Real)(actual_dt) * pe::Real(0.001));
    }
    auto end = COMMON_GetTickCount();
    pe::Real total_time = (pe::Real)(end - start) * pe::Real(0.001);
    std::cout << "fps: " << (pe::Real)frame / total_time << std::endl;
    std::cout << "total time: " << total_time << "s" << std::endl;
    std::cout << "broad phase time: " << _world.broad_phase_time << "s" << std::endl;
    std::cout << "narrow phase time: " << _world.narrow_phase_time << "s" << std::endl;
    std::cout << "constraint solver time: " << _world.constraint_solver_time << "s" << std::endl;
}

template <bool UseViewer>
bool Simulator<UseViewer>::renderInit() {
    pe_intf::Viewer::open("PhysicsDemo", 800, 600, {0, 10, 20}, 0, (float)(PE_PI / 12.0));

    // add models and initialize transform
    for (auto rb : _world.getRigidBodies()) {
        int id;
        switch (rb->getCollisionShape()->getType()) {
            case pe_phys_shape::ShapeType::Box:
                id = pe_intf::Viewer::addCube(((pe_phys_shape::BoxShape*)rb->getCollisionShape())->getSize());
                pe_intf::Viewer::updateCubeColor(id, pe::Vector3(0.3, 0.8, 0.8));
                pe_intf::Viewer::updateCubeTransform(id, rb->getTransform());
                break;
            case pe_phys_shape::ShapeType::Sphere:
                id = pe_intf::Viewer::addSphere(((pe_phys_shape::SphereShape*)rb->getCollisionShape())->getRadius());
                pe_intf::Viewer::updateSphereColor(id, pe::Vector3(0.8, 0.3, 0.3));
                pe_intf::Viewer::updateSphereTransform(id, rb->getTransform());
                break;
            case pe_phys_shape::ShapeType::Cylinder:
                id = pe_intf::Viewer::addCylinder(((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getRadius(),
                                                  ((pe_phys_shape::CylinderShape*)rb->getCollisionShape())->getHeight());
                pe_intf::Viewer::updateCylinderColor(id, pe::Vector3(0.3, 0.8, 0.3));
                pe_intf::Viewer::updateCylinderTransform(id, rb->getTransform());
                break;
            case pe_phys_shape::ShapeType::ConcaveMesh:
                id = pe_intf::Viewer::addMesh(((pe_phys_shape::ConcaveMeshShape*)rb->getCollisionShape())->getMesh());
                goto col;
            case pe_phys_shape::ShapeType::ConvexMesh:
                id = pe_intf::Viewer::addMesh(((pe_phys_shape::ConvexMeshShape*)rb->getCollisionShape())->getMesh());
            col:
                pe_intf::Viewer::updateMeshColor(id, pe::Vector3(0.8, 0.8, 0.3));
                pe_intf::Viewer::updateMeshTransform(id, rb->getTransform());
                break;
        }
        _id_map[id] = rb;
    }

    // set color for each model
    for (auto& rb : _id_map) {
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
            case pe_phys_shape::ShapeType::ConcaveMesh: case pe_phys_shape::ShapeType::ConvexMesh:
                pe_intf::Viewer::updateMeshColor(rb.first, pe::Vector3(0.8, 0.8, 0.3));
                break;
        }
    }

    // wait for key 'r' to start
    while (pe_intf::Viewer::getKeyState('r') != 0) {
        COMMON_Sleep(10);
        if (pe_intf::Viewer::getKeyState(27) == 0) {
            pe_intf::Viewer::close();
            return false;
        }
    }
    return true;
}

template <bool UseViewer>
bool Simulator<UseViewer>::renderStep() {
    if (pe_intf::Viewer::getKeyState(27) == 0) {
        pe_intf::Viewer::close();
        return false;
    }

    for (auto& rb : _id_map) {
        switch (rb.second->getCollisionShape()->getType()) {
            case pe_phys_shape::ShapeType::Box:
                pe_intf::Viewer::updateCubeTransform(rb.first, rb.second->getTransform());
                break;
            case pe_phys_shape::ShapeType::Sphere:
                pe_intf::Viewer::updateSphereTransform(rb.first, rb.second->getTransform());
                break;
            case pe_phys_shape::ShapeType::Cylinder:
                pe_intf::Viewer::updateCylinderTransform(rb.first, rb.second->getTransform());
                break;
            case pe_phys_shape::ShapeType::ConcaveMesh:
            case pe_phys_shape::ShapeType::ConvexMesh:
                pe_intf::Viewer::updateMeshTransform(rb.first, rb.second->getTransform());
                break;
        }
    }
    return true;
}