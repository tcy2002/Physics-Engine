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

    auto rb1 = createRigidBody(pe::Vector3(0, -3, 0), pe::Vector3(40, 2, 40));
    auto rb2 = createRigidBody(pe::Vector3(0, 3, 0), pe::Vector3(2, 2, 2));
    rb1->setKinematic(true);
    pe::Matrix3 rot;
    rot.setRotation(pe::Vector3(1, 0, 0).normalized(), M_PI / 6);
    rb2->setTransform(pe::Transform(rot, pe::Vector3(0, 3, 0)));
    world->addCollisionObject(rb1);
    world->addCollisionObject(rb2);

    int id1 = viewer.addCube(pe::Vector3(40, 2, 40));
    viewer.updateCubeColor(id1, pe::Vector3(0.3, 0.3, 0.7));
    viewer.updateCubeTransform(id1, rb1->getTransform());
    int id2 = viewer.addCube(pe::Vector3(2, 2, 2));
    viewer.updateCubeColor(id2, pe::Vector3(0.7, 0.3, 0.3));

    while (pe_core::Viewer::getKeyState('q') != 0) {
        auto t = COMMON_GetTickCount();
        world->step();
        viewer.updateCubeTransform(id2, rb2->getTransform());
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    pe_core::Viewer::close();
}

int main() {
    testWorld();
}