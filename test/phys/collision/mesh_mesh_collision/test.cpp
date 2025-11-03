#include "test_general.h"
#include "../../../../src/physics"
#include "../../../../src/physics"
#include "../../../../src/physics"
#include "../../../../src/physics"
#include "../../../../src/physics"

using namespace pe_phys_collision;

pe_phys_object::RigidBody* createBoxRigidBody(const pe::Transform& trans,
                                                         const pe::Vector3& size, pe::Real mass) {
    /* This function creates a box-shaped rigidbody */

    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(mass);
    auto shape = new pe_phys_shape::BoxShape(size);
    rb->setCollisionShape(shape);
    rb->setTransform(trans);
    rb->setFrictionCoeff(R(0.5)); // friction coefficient
    rb->setRestitutionCoeff(R(0.5)); // restitution coefficient (the radio of relative velocity after/before collision)
    rb->setAngularDamping(R(0.8)); // angular damping parameter (slows down the rotation speed)
    return rb;
}

void testMeshMesh() {
    // add a ground
    auto rb1 = createBoxRigidBody(pe::Transform(pe::Matrix3::identity(), pe::Vector3(0, -5, 0)),
                                  pe::Vector3(30, 10, 30), 10);
    rb1->setKinematic(true);

    pe::Mesh mesh;
    pe::Mesh::loadFromObj("D:\\ClionProjects\\Physics-Engine-clean\\Physics-Engine-84da628\\test.obj", mesh, pe::Vector3::ones());
    auto shape = new pe_phys_shape::ConvexMeshShape();
    auto offset = shape->setMesh(mesh);
    pe::Transform offsetTrans(pe::Matrix3::identity(), offset);
    pe::Transform trans(pe::Matrix3::fromRotation(pe::Vector3::right(), PE_PI / 2.01), pe::Vector3(0, 1, 0));
    // pe::Transform trans(pe::Matrix3(1, -0, 0, 0, 3.96461e-13, 1, -0, -1, 3.96461e-13),
    //     pe::Vector3(-0.812162, 0.777112, -0.516398));
    // pe::Transform trans(pe::Matrix3(1, -0, 0, 0, 3.96461e-13, 1, -0, -1, 3.96461e-13),
    //     pe::Vector3(-0.812162, 0.777112, -0.516398));
    auto rb2 = new pe_phys_object::RigidBody();
    rb2->setMass(1.0);
    rb2->setCollisionShape(shape);
    rb2->setTransform(trans * offsetTrans);

    auto alg = new BoxConvexCollisionAlgorithm();
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