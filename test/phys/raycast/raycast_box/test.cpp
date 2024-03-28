#include "phys/raycast/raycast/raycast_box.h"
#include "phys/shape/box_shape.h"
#include "phys/object/rigidbody.h"

using namespace pe_phys_ray;

void testRaycastBox() {
    RaycastBox rb;
    pe::Vector3 start(-0.0, 0.1, 0);
    pe::Vector3 direction(2, 1, 1);
    direction.normalize();
    pe::Vector3 box_min(0.1, 0.1, 0.1);
    pe::Vector3 box_max(1, 1, 1);
    pe::Vector3 hit_point, hit_normal;
    pe::Real distance;
    bool ret;

    ret = RaycastBox::rayHitBox(start, direction, box_min, box_max, distance, hit_point, hit_normal);
    std::cout << ret << hit_point << hit_normal << distance << std::endl;

    auto obj = new pe_phys_object::RigidBody();
    obj->setCollisionShape(new pe_phys_shape::BoxShape(pe::Vector3(1000, 1, 1000)));
    obj->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -0.5, 0)));
    start = pe::Vector3(0, 0.7, 0);
    direction = pe::Vector3(0, -1, 0).normalized();
    ret = rb.processRaycast(start, direction, obj, distance, hit_point, hit_normal);
    std::cout << ret << hit_point << hit_normal << distance << std::endl;
}

int main() {
    testRaycastBox();
}