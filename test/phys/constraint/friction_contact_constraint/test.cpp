#include "test_general.h"
#include "phys/constraint/constraint/friction_contact_constraint.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "phys/shape/box_shape.h"
#include "utils/thread_pool.h"

using namespace pe_phys_constraint;

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(1.0);
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::identity(), pos));
    pe::Real x = size.x, y = size.y, z = size.z;
    rb->setLocalInertia({(y * y + z * z) / 12, 0, 0,
                         0, (x * x + z * z) / 12, 0,
                         0, 0, (x * x + y * y) / 12});
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(1.0);
    return rb;
}

void testFrictionContactConstraint() {
    auto rb1 = createRigidBody(pe::Vector3(0, -1, 0), pe::Vector3(40, 2, 40));
    auto rb2 = createRigidBody(pe::Vector3(0, 0.999, 0), pe::Vector3(2, 2, 2));
    rb2->setLinearVelocity(pe::Vector3(0, -4, 0));

    auto np = pe_phys_collision::SimpleNarrowPhase();
    pe::Array<pe_phys_collision::CollisionPair> pairs = {{rb2, rb1}};
    pe::Array<pe_phys_collision::ContactResult*> result;
    np.calcContactResults(pairs, result);
    ASSERT_EQUAL_INT(result.size(), 1)
    ASSERT_EQUAL_INT(result[0]->getPointSize(), 4)

    auto fcc = FrictionContactConstraint();
    fcc.setContactResult(*result[0]);
    ConstraintParam param{};

    rb1->clearTempVelocity();
    rb2->clearTempVelocity();
    fcc.initSequentialImpulse(param);
    fcc.warmStart();
    for (int i = 0; i < 8; i++) {
        fcc.iterateSequentialImpulse(i);
    }
    rb1->syncTempVelocity();
    rb2->syncTempVelocity();
    fcc.afterSequentialImpulse();

    std::cout << rb1->getLinearVelocity() << std::endl;
    std::cout << rb2->getLinearVelocity() << std::endl;
}

int main() {
    utils::ThreadPool::init();
    testFrictionContactConstraint();
}