#include "test_general.h"
#include "phys/collision/broad_phase/broad_phase_sweep_and_prune.h"
#include "phys/object/rigidbody.h"
#include "phys/shape/box_shape.h"

using namespace pe_phys_collision;

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    return rb;
}

void testCollisionPair() {
    auto rb1 = createRigidBody(pe::Vector3(0, 0, 0), pe::Vector3(1, 1, 1));
    auto rb2 = createRigidBody(pe::Vector3(0, 0.5, 0), pe::Vector3(1, 1, 1));
    auto rb3 = createRigidBody(pe::Vector3(0, 1.1, 0), pe::Vector3(1, 1, 1));
    auto rb4 = createRigidBody(pe::Vector3(0, 1.6, 0), pe::Vector3(1, 1, 1));
    rb3->setKinematic(true);
    rb4->setKinematic(true);

    rb1->computeAABB();
    rb2->computeAABB();
    rb3->computeAABB();
    rb4->computeAABB();

    auto bp = new BroadPhaseSweepAndPrune();
    pe::Array<pe_phys_object::CollisionObject*> collision_objects = {rb1, rb2, rb3, rb4};
    bp->calcCollisionPairs(collision_objects);
    auto collision_pairs = bp->getCollisionPairs();

    ASSERT_EQUAL_INT(collision_pairs.size(), 2)
}

int main() {
    testCollisionPair();
}