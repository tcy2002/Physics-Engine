#include "test_general.h"
#include "phys/constraint/constraint_solver/primal_dual_solver.h"
#include "phys/constraint/constraint_solver/sequential_impulse_solver.h"
#include "phys/collision/narrow_phase/simple_narrow_phase.h"
#include "phys/shape/box_shape.h"
#include "utils/thread_pool.h"

using namespace pe_phys_constraint;

pe_phys_object::RigidBody* createRigidBody(const pe::Vector3& pos, const pe::Vector3& size) {
    auto rb = new pe_phys_object::RigidBody();
    rb->setMass(1.0);
    rb->setCollisionShape(new pe_phys_shape::BoxShape(size));
    rb->setTransform(pe::Transform(pe::Matrix3::Identity(), pos));
    rb->setFrictionCoeff(0.5);
    rb->setRestitutionCoeff(0.8);
    return rb;
}

void testFrictionContactConstraint() {
    auto rb0 = createRigidBody(pe::Vector3(0, -0.99, 0), pe::Vector3(2, 1, 2));
    rb0->setKinematic(true);
    auto rb1 = createRigidBody(pe::Vector3(0, 0, 0), pe::Vector3(1, 1, 1));
    auto rb2 = createRigidBody(pe::Vector3(0, 0.99, 0), pe::Vector3(1, 1, 1));

    auto np = pe_phys_collision::SimpleNarrowPhase();
    pe::Array<pe_phys_collision::CollisionPair> pairs = { {rb0, rb1} };
    pe::Array<pe_phys_collision::ContactResult*> result;
    np.calcContactResults(pairs, result);
    std::cout << "pos; " << result[0]->getContactPoint(0).getWorldPosHalf() << std::endl;
    std::cout << "nor: " << result[0]->getContactPoint(0).getWorldNormal() << std::endl;
    auto solver = new PrimalDualSolver();
    //auto solver = new SequentialImpulseSolver();
    solver->setupSolver(pe::Real(0.01), { 0, 0, 0 }, { rb0, rb1 }, result, {});
    solver->solve();

    std::cout << rb0->getLinearVelocity() << std::endl;
    std::cout << rb1->getLinearVelocity() << std::endl;
    std::cout << rb2->getLinearVelocity() << std::endl;
}

int main() {
    utils::ThreadPool::init();
    testFrictionContactConstraint();
}