#include "phys/object/rigidbody.h"
#include "phys/shape/box_shape.h"
#include "test_general.h"

using namespace pe_phys_object;

RigidBody* getRigidBody() {
    RigidBody* rb = new RigidBody();
    rb->setMass(2.0);
    rb->setCollisionShape(new pe_phys_shape::BoxShape(pe::Vector3(1.0, 1.0, 1.0)));
    rb->setLocalInertia({1.0 / 6, 0, 0, 0, 1.0 / 6, 0, 0, 0, 1.0 / 6});
    rb->setTransform(pe::Transform::identity());
    return rb;
}

RigidBody* getKinematicRigidBody() {
    RigidBody* rb = new RigidBody();
    rb->setCollisionShape(new pe_phys_shape::BoxShape(pe::Vector3(1.0, 1.0, 1.0)));
    rb->setTransform(pe::Transform::identity());
    rb->setKinematic(true);
    return rb;
}

void testVelocity() {
    auto rb = getRigidBody();
    rb->setLinearVelocity(pe::Vector3(1.0, 0.0, 0.0));
    rb->setAngularVelocity(pe::Vector3(0.0, 1.0, 0.0));
    rb->step(0.01);
    ASSERT_VECTOR3_EQUAL(rb->getTransform().getOrigin(), pe::Vector3(0.01, 0.0, 0.0));
    pe::Matrix3 rot;
    rot.setRotation(pe::Vector3(0.0, 1.0, 0.0), 0.01);
    ASSERT_MATRIX3_EQUAL(rb->getTransform().getBasis(), rot);
}

void testForce() {
    auto rb = getRigidBody();
    rb->addForce(pe::Vector3(0.0, 0.0, 0.5), pe::Vector3(1.0, 0.0, 0.0));
    rb->addCentralForce(pe::Vector3(0.0, 1.0, 0.0));
    rb->applyForce(0.01);
    ASSERT_VECTOR3_EQUAL(rb->getLinearVelocity(), pe::Vector3(0.005, 0.005, 0.0));
    ASSERT_VECTOR3_EQUAL(rb->getAngularVelocity(), pe::Vector3(0.0, 0.03, 0.0));
}

int main() {
    testVelocity();
    testForce();
}