template <bool UseViewer>
void Simulator<UseViewer>::run(pe::Real dt, int max_frame) {
    _world.setDt(dt);
    init();
    if (UseViewer) renderInit();

    int frame = 0;
    int dt_ms = (int)(dt * 1000);
    while (++frame < max_frame) {
        auto t = COMMON_GetTickCount();

        step();

        if (UseViewer) {
            if (!renderStep()) {
                break;
            }
        }

        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }
}

template <bool UseViewer>
void Simulator<UseViewer>::renderInit() {
    pe_intf::Viewer::open("PhysicsDemo", 800, 600,
                          {0, 10, 20}, 0, (float)(PE_PI / 6.0));

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
}

template <bool UseViewer>
bool Simulator<UseViewer>::renderStep() {
    if (pe_intf::Viewer::getKeyState(27) == 0) {
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