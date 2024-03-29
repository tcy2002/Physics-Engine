#include <iomanip>
#include "phys/vehicle/tank/tank_template.h"
#include "intf/viewer.h"
#include "phys/constraint/constraint/friction_contact_constraint.h"

using namespace pe_phys_vehicle;

#define PE_TEST_GROUND_MARGIN pe::Vector3(0.05, 0.03, 0.05)
#define PE_TEST_OBJ false
#define PE_TEST_OBJ_NUM 150

pe_phys_object::RigidBody* createBoxRigidBody(const pe::Vector3& pos, const pe::Vector3& size, pe::Real mass) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(mass);
    auto shape = new pe_phys_shape::BoxShape(size);
    rb->setCollisionShape(shape);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setLocalInertia(shape->calcLocalInertia(mass));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.5);
    rb->setAngularDamping(0.8);
    return rb;
}

pe_phys_object::RigidBody* createSphereRigidBody(const pe::Vector3& pos, pe::Real radius, pe::Real mass) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(mass);
    auto shape = new pe_phys_shape::SphereShape(radius);
    rb->setCollisionShape(shape);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setLocalInertia(shape->calcLocalInertia(mass));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.5);
    rb->setAngularDamping(0.8);
    return rb;
}

pe_phys_object::RigidBody* createCylinderRigidBody(const pe::Vector3& pos, pe::Real radius, pe::Real height, pe::Real mass) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(mass);
    auto shape = new pe_phys_shape::CylinderShape(radius, height);
    rb->setCollisionShape(shape);
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    rb->setLocalInertia(shape->calcLocalInertia(mass));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.5);
    rb->setAngularDamping(0.8);
    return rb;
}

void testTank() {
    auto world = new pe_intf::World();
    world->setDt(0.01);
    world->setGravity(pe::Vector3(0, -9.8, 0));

    pe::Map<int, pe_phys_object::RigidBody*> id_map;
    int id;

    // open viewer
    pe_intf::Viewer::open("VehicleTest", 800, 600,
                          {-25, 15, -10},
                          (float)(-PE_PI / 2.0), (float)(PE_PI / 6.0));

    // add ground
    auto ground = new pe_phys_object::RigidBody();
    auto ground_shape = new pe_phys_shape::BoxShape(pe::Vector3(1000, 1, 1000));
    ground->setCollisionShape(ground_shape);
    pe::Matrix3 mat = pe::Matrix3::identity();
//    mat.setRotation(pe::Vector3(0, 0, 1), -PE_PI / 12);
    ground->setTransform(pe::Transform(mat, pe::Vector3(0, -0.5, 0)));
    ground->setKinematic(true);
    ground->setLocalInertia(ground_shape->calcLocalInertia(1.0));
    world->addRigidBody(ground);
    int ground_id = pe_intf::Viewer::addCube(ground_shape->getSize() - PE_TEST_GROUND_MARGIN * 2);
    pe_intf::Viewer::updateCubeColor(ground_id, pe::Vector3(0.3, 0.3, 0.8));
    pe_intf::Viewer::updateCubeTransform(ground_id, ground->getTransform());

    // add a slope
    auto box = new pe_phys_object::RigidBody();
    auto shape_box = new pe_phys_shape::BoxShape(pe::Vector3(10, 0.3, 10));
    box->setCollisionShape(shape_box);
    mat = pe::Matrix3::identity();
    mat.setRotation(pe::Vector3(1, 0, 0), PE_PI / 12);
    box->setTransform(pe::Transform(mat, pe::Vector3(0, 1.4, -10)));
    box->setKinematic(true);
    box->setLocalInertia(shape_box->calcLocalInertia(1.0));
    world->addRigidBody(box);
    int box_id = pe_intf::Viewer::addCube(shape_box->getSize() - PE_TEST_GROUND_MARGIN * 2);
    pe_intf::Viewer::updateCubeColor(box_id, pe::Vector3(0.8, 0.3, 0.3));
    pe_intf::Viewer::updateCubeTransform(box_id, box->getTransform());
    id_map[box_id] = box;

    // add some steps
    for (int i = 0; i < 10; i++) {
        auto step = new pe_phys_object::RigidBody();
        auto shape_step = new pe_phys_shape::BoxShape(pe::Vector3(10, 0.3, 10));
        step->setCollisionShape(shape_step);
        step->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 0.15 + 0.3 * i, -30 - i * 0.5)));
        step->setKinematic(true);
        step->setLocalInertia(shape_step->calcLocalInertia(1.0));
        world->addRigidBody(step);
        int step_id = pe_intf::Viewer::addCube(shape_step->getSize() - PE_TEST_GROUND_MARGIN * 2);
        pe_intf::Viewer::updateCubeColor(step_id, pe::Vector3(0.8, 0.3, 0.3));
        pe_intf::Viewer::updateCubeTransform(step_id, step->getTransform());
        id_map[step_id] = step;
    }

#if PE_TEST_OBJ
    // add some obstacles
    pe::Real mass = 1.0;
    for (int i = 0; i < PE_TEST_OBJ_NUM; i++) {
        pe_phys_object::RigidBody* rb;
        if (i % 3 == 0) {
            rb = createBoxRigidBody(pe::Vector3(0, 10 + i * 1.1, -50), pe::Vector3(0.2, 0.2, 0.2), mass);
            id = pe_intf::Viewer::addCube(pe::Vector3(0.2, 0.2, 0.2));
            pe_intf::Viewer::updateCubeColor(id, pe::Vector3(0.3, 0.8, 0.3));
        } else if (i % 3 == 1) {
            rb = createSphereRigidBody(pe::Vector3(0, 10 + i * 1.1, -50), 0.1, mass);
            id = pe_intf::Viewer::addSphere(0.1);
            pe_intf::Viewer::updateSphereColor(id, pe::Vector3(0.3, 0.8, 0.3));
        } else {
            rb = createCylinderRigidBody(pe::Vector3(0, 10 + i * 1.1, -50), 0.1, 0.2, mass);
            id = pe_intf::Viewer::addCylinder(0.1, 0.2);
            pe_intf::Viewer::updateCylinderColor(id, pe::Vector3(0.3, 0.8, 0.3));
        }
        world->addRigidBody(rb);
        id_map[id] = rb;
    }
#endif

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
#   if PE_USE_SPHERE_WHEEL
        auto shape = dynamic_cast<pe_phys_shape::SphereShape*>(wheel->getCollisionShape());
#   else
        auto shape = dynamic_cast<pe_phys_shape::CylinderShape*>(wheel->getCollisionShape());
#   endif
        id = pe_intf::Viewer::addCylinder(shape->getRadius(), 0.5);
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

    int frame = 0;
    bool run = false;
    while (true) {
//        while (pe_intf::Viewer::getKeyState('r') != 0)
            if (pe_intf::Viewer::getKeyState(27) == 0) goto ret;
        if (pe_intf::Viewer::getKeyState('r') == 0) run = true;
        if (!run) continue;

        auto t = COMMON_GetTickCount();

        world->step();
        tank->advance(world->getDt());
        for (auto rb : id_map) {
            pe_intf::Viewer::updateCubeTransform(rb.first, rb.second->getTransform());
            pe_intf::Viewer::updateCylinderTransform(rb.first, rb.second->getTransform());
            pe_intf::Viewer::updateSphereTransform(rb.first, rb.second->getTransform());
        }

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
        if (++frame == 30) {
            frame = 0;
            std::cout << "\r" << std::fixed << std::setprecision(2) << tank->getSpeedKmHour() << " km/h    ";
        }

//        while (pe_intf::Viewer::getKeyState('r') != 1)
        COMMON_Sleep(10 - (int)(COMMON_GetTickCount() - t));
    }

    ret:
    pe_intf::Viewer::close();
    delete world;
    delete tank;
}

int main() {
    testTank();
}