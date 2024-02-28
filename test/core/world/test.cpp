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
    auto world = new pe_core::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    pe_core::Viewer viewer;
    pe_core::Viewer::open();

    auto rb1 = createRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(5, 1, 5));
    auto rb2 = createRigidBody(pe::Vector3(0, 2.5, 0), pe::Vector3(1, 1, 1));
    auto rb3 = createRigidBody(pe::Vector3(0, 5.5, 0), pe::Vector3(1, 1, 1));
    rb1->setKinematic(true);
    world->addCollisionObject(rb1);
    world->addCollisionObject(rb2);
    world->addCollisionObject(rb3);

    int id1 = viewer.addCube(pe::Vector3(5, 1, 5));
    viewer.updateCubeColor(id1, pe::Vector3(0.3, 0.3, 0.8));
    viewer.updateCubeTransform(id1, rb1->getTransform());
    int id2 = viewer.addCube(pe::Vector3(1, 1, 1));
    viewer.updateCubeColor(id2, pe::Vector3(0.8, 0.3, 0.3));
    viewer.updateCubeTransform(id2, rb2->getTransform());
    int id3 = viewer.addCube(pe::Vector3(1, 1, 1));
    viewer.updateCubeColor(id3, pe::Vector3(0.8, 0.3, 0.3));
    viewer.updateCubeTransform(id3, rb3->getTransform());

    int frame = 0, th = 500;
    while (pe_core::Viewer::getKeyState('q') != 0) {
        frame++;
        if (frame > th) while (pe_core::Viewer::getKeyState('e') != 0);
        auto t = COMMON_GetTickCount();
        world->step();
        viewer.updateCubeTransform(id2, rb2->getTransform());
        viewer.updateCubeTransform(id3, rb3->getTransform());
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
        if (frame > th) while (pe_core::Viewer::getKeyState('e') == 0);
    }

    pe_core::Viewer::close();
}

int main() {
    testWorld();
}