#include "test_general.h"
#include "phys/constraint/solver/sequential_impulse_solver.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "phys/shape/box_shape.h"

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
    rb->setRestitutionCoeff(0.8);
    return rb;
}

void testFrictionContactConstraint() {
    auto rb1 = createRigidBody(pe::Vector3(0, 0, 0), pe::Vector3(1, 1, 1));
    auto rb2 = createRigidBody(pe::Vector3(0, 0.99, 0), pe::Vector3(1, 1, 1));
    rb1->setFrictionCoeff(0.5);
    rb1->setRestitutionCoeff(0.5);
    rb2->setFrictionCoeff(0.5);
    rb2->setRestitutionCoeff(0.5);

    auto np = pe_phys_collision::SimpleNarrowPhase();
    pe::Array<pe_phys_collision::CollisionPair> pairs = {{rb1, rb2}};
    np.calcContactResults(pairs);
    auto solver = new SequentialImpulseSolver();
    solver->setupSolver({rb1, rb2}, np.getContactResults(), {});
    solver->solve();

    std::cout << rb1->getLinearVelocity() << std::endl;
    std::cout << rb2->getLinearVelocity() << std::endl;
}

int main() {
    testFrictionContactConstraint();
}