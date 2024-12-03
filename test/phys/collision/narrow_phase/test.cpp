#include "test_general.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "phys/object/rigidbody.h"
#include "phys/shape/box_shape.h"
#include "utils/thread_pool.h"

using namespace pe_phys_collision;

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::Identity(), pos));
    return rb;
}

void testNarrowPhase() {
    auto rb1 = createRigidBody(pe::Vector3(0, 0, 0), pe::Vector3(1, 1, 1));
    auto rb2 = createRigidBody(pe::Vector3(0, 0.99, 0), pe::Vector3(1, 1, 1));
    auto np = new SimpleNarrowPhase();
    pe::Array<CollisionPair> pairs = {{rb1, rb2}};
    pe::Array<pe_phys_collision::ContactResult*> result;
    np->calcContactResults(pairs, result);
    ASSERT_EQUAL_INT(result.size(), 1)
    ASSERT_EQUAL_INT(result[0]->getPointSize(), 4)
}

int main() {
    utils::ThreadPool::init();
    testNarrowPhase();
    return 0;
}