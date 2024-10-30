#include "phys/raycast/raycast/raycast_mesh.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/default_mesh.h"
#include "phys/object/rigidbody.h"

using namespace pe_phys_raycast;

void testRaycastMesh() {
    RaycastMesh rm;
    pe::Vector3 start(0, 0, 0);
    pe::Vector3 direction(0, 1, 0);
    direction.normalize();
    pe::Vector3 hit_point, hit_normal;
    pe::Real distance;
    bool ret;

    auto obj = new pe_phys_object::RigidBody();
	auto shape = new pe_phys_shape::ConvexMeshShape();
    shape->setMesh(PE_CYLINDER_DEFAULT_MESH);
    obj->setCollisionShape(shape);
    obj->setTransform(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, 1.0, 0)));
    ret = rm.processRaycast(start, direction, obj->getCollisionShape(), obj->getTransform(), distance, hit_point, hit_normal);
    std::cout << ret << hit_point << hit_normal << distance << std::endl;
}

int main() {
    testRaycastMesh();
}