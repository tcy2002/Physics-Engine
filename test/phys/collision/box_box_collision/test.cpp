#include "test_general.h"
#include "phys/collision/collision_algorithm/box_box_collision_algorithm.h"
#include "phys/object/rigidbody.h"
#include "phys/shape/box_shape.h"

using namespace pe_phys_collision;

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::Identity(), pos));
    return rb;
}

void testBoxBox() {
    auto rb1 = createRigidBody(pe::Vector3(0, 0.49, 0), pe::Vector3(1, 1, 1));
    auto rb2 = createRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(20, 1, 20));
    auto alg = new BoxBoxCollisionAlgorithm();
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
    testBoxBox();
}