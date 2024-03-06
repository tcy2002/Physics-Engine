#include "core/world.h"
#include "phys/shape/box_shape.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/fracture/fracture_solver/fracture_solver.h"
#include "phys/object/fracturable_object.h"
#include "core/viewer.h"

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(1.0);
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    pe::Real x = size.x, y = size.y, z = size.z;
    rb->setLocalInertia({(y * y + z * z) / 12, 0, 0,
                         0, (x * x + z * z) / 12, 0,
                         0, 0, (x * x + y * y) / 12});
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    return rb;
}

pe_phys_object::FracturableObject* createFracturableObject(const pe::Vector3& pos, const pe::Vector3& size, pe::Real th) {
    auto rb = new pe_phys_object::FracturableObject();
    rb->setMass(1.0);
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    pe::Real x = size.x, y = size.y, z = size.z;
    rb->setLocalInertia({(y * y + z * z) / 12, 0, 0,
                         0, (x * x + z * z) / 12, 0,
                         0, 0, (x * x + y * y) / 12});
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    rb->setThreshold(th);
    return rb;
}

void testWorld() {
    int rb_num = 1000;
    auto world = new pe_core::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    pe_core::Viewer viewer;
    pe_core::Viewer::open();

    // create rigid bodies
    auto rb1 = createRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(20, 1, 20));
    rb1->setKinematic(true);
    auto rb2 = createFracturableObject(pe::Vector3(3, 5, 3), pe::Vector3(4, 4, 4), 1);
    auto fs = new pe_phys_fracture::FractureSolver();
    pe_phys_fracture::FractureSource src;
    src.type = pe_phys_fracture::FractureType::Sphere;
    src.position = pe::Vector3(4.5, 6.5, 4.5);
    src.intensity = pe::Vector3(0.5, 0.5, 0.5);
    fs->setFracturableObject(rb2);
    fs->solve({src});
    std::cout << "fragments count: " << fs->getFragments().size() << std::endl;

    // add to world
    world->addRigidBody(rb1);
    pe::Array<pe_phys_object::RigidBody*> rbs;
    for (int i = 0; i < rb_num; i++) {
        auto rb = createRigidBody(pe::Vector3(0, 1 + i * 1.1, 0), pe::Vector3(1, 1, 1));
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }
    for (auto rb : fs->getFragments()) {
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }

    // add to viewer
    int id1 = viewer.addCube(pe::Vector3(20, 1, 20));
    viewer.updateCubeColor(id1, pe::Vector3(0.3, 0.3, 0.8));
    viewer.updateCubeTransform(id1, rb1->getTransform());
    pe::Array<int> ids;
    for (int i = 0; i < rb_num; i++) {
        int id = viewer.addCube(pe::Vector3(1, 1, 1));
        viewer.updateCubeColor(id, pe::Vector3(0.8, 0.3, 0.3));
        viewer.updateCubeTransform(id, rbs[i]->getTransform());
        ids.push_back(id);
    }
    for (auto rb : fs->getFragments()) {
        auto shape = (pe_phys_shape::ConvexMeshShape*)(rb->getCollisionShape());
        auto id = viewer.addMesh(shape->getMesh());
        viewer.updateMeshColor(id, pe::Vector3(0.3, 0.8, 0.3));
        viewer.updateMeshTransform(id, rb->getTransform());
        ids.push_back(id);
    }

    // main loop
    int frame = 0, th = 5000;
    while (pe_core::Viewer::getKeyState('q') != 0) {
        frame++;
        if (frame > th) while (pe_core::Viewer::getKeyState('e') != 0);
        auto t = COMMON_GetTickCount();
        world->step();
        for (int i = 0; i < ids.size(); i++) {
            viewer.updateCubeTransform(ids[i], rbs[i]->getTransform());
            viewer.updateMeshTransform(ids[i], rbs[i]->getTransform());
        }
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
        if (frame > th) while (pe_core::Viewer::getKeyState('e') == 0);
    }

    pe_core::Viewer::close();
}

int main() {
    testWorld();
}