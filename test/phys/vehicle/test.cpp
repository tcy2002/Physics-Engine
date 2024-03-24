#include "phys/vehicle/tank/tank_template.h"
#include "intf/viewer.h"
#include "phys/constraint/constraint/friction_contact_constraint.h"

using namespace pe_phys_vehicle;

void testTank() {
    auto world = new pe_intf::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    pe::Map<int, pe_phys_object::RigidBody*> id_map;
    int id;

    // open viewer
    pe_intf::Viewer::open();

    // add ground
    auto ground = new pe_phys_object::RigidBody();
    auto ground_shape = new pe_phys_shape::BoxShape(pe::Vector3(100, 1, 100));
    ground->setCollisionShape(ground_shape);
    pe::Matrix3 mat = pe::Matrix3::identity();
//    mat.setRotation(pe::Vector3(0, 0, 1), -PE_PI / 12);
    ground->setTransform(pe::Transform(mat, pe::Vector3(0, -0.5, 0)));
    ground->setKinematic(true);
    ground->setLocalInertia(ground_shape->calcLocalInertia(1.0));
    world->addRigidBody(ground);
    int ground_id = pe_intf::Viewer::addCube(pe::Vector3(100, 1, 100));
    pe_intf::Viewer::updateCubeColor(ground_id, pe::Vector3(0.3, 0.3, 0.8));
    pe_intf::Viewer::updateCubeTransform(ground_id, ground->getTransform());

    // add some obstacle
    auto box = new pe_phys_object::RigidBody();
    auto shape_box = new pe_phys_shape::BoxShape(pe::Vector3(10, 0.3, 10));
    box->setCollisionShape(shape_box);
    mat = pe::Matrix3::identity();
//    mat.setRotation(pe::Vector3(0, 1, 0), PE_PI / 4);
    box->setTransform(pe::Transform(mat, pe::Vector3(0, 0.15, -10)));
    box->setKinematic(true);
    box->setLocalInertia(shape_box->calcLocalInertia(1.0));
    world->addRigidBody(box);
    int box_id = pe_intf::Viewer::addCube(shape_box->getSize());
    pe_intf::Viewer::updateCubeColor(box_id, pe::Vector3(0.8, 0.3, 0.3));
    pe_intf::Viewer::updateCubeTransform(box_id, box->getTransform());
    id_map[box_id] = box;

    // init tank
    auto tank = new TankTemplate();
    mat = pe::Matrix3::identity();
//    mat.setRotation(pe::Vector3(0, 0, 1), PE_PI);
    tank->setTransform(pe::Transform(mat, pe::Vector3(0, 1.3, 0)));
    tank->init(world);

    // add body to viewer
    auto shape_b = dynamic_cast<pe_phys_shape::BoxShape*>(tank->getBody()->getCollisionShape());
    id = pe_intf::Viewer::addCube(shape_b->getSize());
    pe_intf::Viewer::updateCubeColor(id, pe::Vector3(0.8, 0.8, 0.3));
    pe_intf::Viewer::updateCubeTransform(id, tank->getBody()->getTransform());
    id_map[id] = tank->getBody();

    // add turret to viewer
    auto shape_t = dynamic_cast<pe_phys_shape::BoxShape*>(tank->getTurret()->getCollisionShape());
    id = pe_intf::Viewer::addCube(shape_t->getSize());
    pe_intf::Viewer::updateCubeColor(id, pe::Vector3(0.8, 0.8, 0.3));
    pe_intf::Viewer::updateCubeTransform(id, tank->getTurret()->getTransform());
    id_map[id] = tank->getTurret();

    // add barrel to viewer
    auto shape_r = dynamic_cast<pe_phys_shape::BoxShape*>(tank->getBarrel()->getCollisionShape());
    id = pe_intf::Viewer::addCube(shape_r->getSize());
    pe_intf::Viewer::updateCubeColor(id, pe::Vector3(0.8, 0.8, 0.3));
    pe_intf::Viewer::updateCubeTransform(id, tank->getBarrel()->getTransform());
    id_map[id] = tank->getBarrel();

    // add wheels to viewer
    for (auto& wheel : tank->getWheels()) {
        auto shape = dynamic_cast<pe_phys_shape::CylinderShape*>(wheel->getCollisionShape());
        id = pe_intf::Viewer::addCylinder(shape->getRadius(), shape->getHeight());
        pe_intf::Viewer::updateCylinderColor(id, pe::Vector3(0.8, 0.8, 0.3));
        pe_intf::Viewer::updateCylinderTransform(id, wheel->getTransform());
        id_map[id] = wheel;
    }

    // add tracks to viewer
    for (auto& track : tank->getTrackSegments()) {
        auto shape = dynamic_cast<pe_phys_shape::BoxShape*>(track->getCollisionShape());
        id = pe_intf::Viewer::addCube(shape->getSize());
        pe_intf::Viewer::updateCubeColor(id, pe::Vector3(0.8, 0.8, 0.3));
        pe_intf::Viewer::updateCubeTransform(id, track->getTransform());
        id_map[id] = track;
    }

    while (true) {
//        while (pe_intf::Viewer::getKeyState('r') != 0)
            if (pe_intf::Viewer::getKeyState('q') == 0) goto ret;

        auto t = COMMON_GetTickCount();

        world->step();

        if (pe_intf::Viewer::getKeyState('i') == 0) {
            tank->moveForward();
        } else if (pe_intf::Viewer::getKeyState('k') == 0) {
            tank->moveBackward();
        } else if (pe_intf::Viewer::getKeyState('j') == 0) {
            tank->turnLeft();
        } else if (pe_intf::Viewer::getKeyState('l') == 0) {
            tank->turnRight();
        } else if (pe_intf::Viewer::getKeyState('u') == 0) {
            tank->barrelRotLeft(world->getDt());
        } else if (pe_intf::Viewer::getKeyState('o') == 0) {
            tank->barrelRotRight(world->getDt());
        } else if (pe_intf::Viewer::getKeyState(' ') == 0) {
            tank->brake();
        } else {
            tank->idle();
        }

        tank->advance(world->getDt());

        for (auto rb : id_map) {
            pe_intf::Viewer::updateCubeTransform(rb.first, rb.second->getTransform());
            pe_intf::Viewer::updateCylinderTransform(rb.first, rb.second->getTransform());
        }

//        while (pe_intf::Viewer::getKeyState('r') != 1)
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    ret:
    delete world;
    delete tank;
    pe_intf::Viewer::close();
}

int main() {
    testTank();
}