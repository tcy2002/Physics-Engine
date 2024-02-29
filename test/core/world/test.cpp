#include "core/world.h"
#include "phys/shape/box_shape.h"
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

void testWorld() {
    int rb_num = 10;
    auto world = new pe_core::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    pe_core::Viewer viewer;
    pe_core::Viewer::open();

    auto rb1 = createRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(10, 1, 10));
    rb1->setKinematic(true);
    world->addRigidBody(rb1);

    pe::Array<pe_phys_object::RigidBody*> rbs;
    for (int i = 0; i < rb_num; i++) {
        auto rb = createRigidBody(pe::Vector3(0, 1 + i * 2, 0), pe::Vector3(1, 1, 1));
        rbs.push_back(rb);
        world->addRigidBody(rb);
    }

    int id1 = viewer.addCube(pe::Vector3(10, 1, 10));
    viewer.updateCubeColor(id1, pe::Vector3(0.3, 0.3, 0.8));
    viewer.updateCubeTransform(id1, rb1->getTransform());

    pe::Array<int> ids;
    for (int i = 0; i < rb_num; i++) {
        int id = viewer.addCube(pe::Vector3(1, 1, 1));
        viewer.updateCubeColor(id, pe::Vector3(0.8, 0.3, 0.3));
        viewer.updateCubeTransform(id, rbs[i]->getTransform());
        ids.push_back(id);
    }

    int frame = 0, th = 5000;
    while (pe_core::Viewer::getKeyState('q') != 0) {
        frame++;
        if (frame > th) while (pe_core::Viewer::getKeyState('e') != 0);
        auto t = COMMON_GetTickCount();
        world->step();
        for (int i = 0; i < rb_num; i++) {
            viewer.updateCubeTransform(ids[i], rbs[i]->getTransform());
        }
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
        if (frame > th) while (pe_core::Viewer::getKeyState('e') == 0);
    }

    pe_core::Viewer::close();
}

int main() {
    testWorld();
}