#include "phys/raycast/raycast/raycast_cylinder.h"
#include "phys/shape/cylinder_shape.h"
#include "phys/object/rigidbody.h"

using namespace pe_phys_raycast;

void testRaycastCylinder() {
    RaycastCylinder rc;
    pe::Vector3 start(0, 0, 0);
    pe::Vector3 direction(1, 1, 0);
    direction.normalize();
    pe::Vector3 hit_point, hit_normal;
    pe::Real distance;
    bool ret;

    auto obj = new pe_phys_object::RigidBody();
    obj->setCollisionShape(new pe_phys_shape::CylinderShape(0.5, 1.0));
    obj->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1.0, 0)));
    ret = rc.processRaycast(start, direction, obj->getCollisionShape(), obj->getTransform(), distance, hit_point, hit_normal);
    std::cout << ret << hit_point << hit_normal << distance << std::endl;
}

int main() {
    testRaycastCylinder();
}