#include "test_general.h"
#include "phys/collision/collision_algorithm/convex_convex_collision_algorithm.h"
#include "phys/object/rigidbody.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/default_mesh.h"
#include <fstream>
#include <sstream>

using namespace pe_phys_collision;

pe::Mesh resizeCylinder(pe::Real radius, pe::Real height) {
    pe::Mesh result = PE_CYLINDER_DEFAULT_MESH;
    for (auto& v : result.vertices) {
        v.position.y() *= height;
        v.position.x() *= (radius / 0.5);
        v.position.z() *= (radius / 0.5);
    }
    return std::move(result);
}

pe::Mesh resizeBox(const pe::Vector3& size) {
    pe::Mesh result = PE_BOX_DEFAULT_MESH;
    for (auto& v : result.vertices) {
        v.position = v.position.cwiseProduct(size);
    }
    return std::move(result);
}

void testMeshMesh() {
    pe::Mesh mesh;
    pe::Mesh::saveToObj(CURRENT_TEST_SOURCE_DIR "/mesh.obj", mesh, pe::Vector3::Ones());
    auto rb1 = new pe_phys_object::RigidBody();
    auto shape = new pe_phys_shape::ConvexMeshShape();
    shape->setMesh(mesh);
    rb1->setCollisionShape(shape);
    rb1->setTransform(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, 0, 0)));
    auto rb2 = new pe_phys_object::RigidBody();
    shape = new pe_phys_shape::ConvexMeshShape();
    shape->setMesh(resizeBox({20, 1, 20}));
    rb2->setCollisionShape(shape);
    rb2->setTransform(pe::Transform(pe::Matrix3::Identity(), pe::Vector3(0, -0.5, 0)));

    auto alg = new ConvexConvexCollisionAlgorithm();
    ContactResult result;
    result.setObjectA(rb1);
	result.setObjectB(rb2);
    pe::Real refScale = (rb1->getAABBScale() + rb2->getAABBScale()) * PE_DIST_REF_RADIO;
    alg->processCollision(rb1->getCollisionShape(), rb2->getCollisionShape(), rb1->getTransform(), rb2->getTransform(), refScale, result);
    result.sortContactPoints();

    std::cout << result.getPointSize() << std::endl;
    for (int i = 0; i < result.getPointSize(); i++) {
        auto& p = result.getContactPoint(i);
        std::cout << p.getDistance() << " ";
        std::cout << p.getWorldPos();
        std::cout << p.getWorldNormal();
        std::cout << p.getLocalPosA();
        std::cout << p.getLocalPosB() << std::endl;
    }
}

int main() {
    testMeshMesh();
}