#include "phys/raycast/raycast/raycast_sphere.h"
#include "phys/shape/sphere_shape.h"
#include "phys/object/rigidbody.h"

using namespace pe_phys_raycast;

void testRaycastSphere() {
    RaycastSphere rs;
    pe::Vector3 start(1, 0, 0);
    pe::Vector3 direction(0, 1, 1);
    direction.normalize();
    pe::Vector3 hit_point, hit_normal;
    pe::Real distance;
    bool ret;

    auto obj = new pe_phys_object::RigidBody();
    obj->setCollisionShape(new pe_phys_shape::SphereShape(1.0));
    obj->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(1, 1, 1)));
    ret = rs.processRaycast(start, direction, obj, distance, hit_point, hit_normal);
    std::cout << ret << hit_point << hit_normal << distance << std::endl;
}

int main() {
    testRaycastSphere();
}