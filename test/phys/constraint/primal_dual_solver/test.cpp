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
    auto rb0 = createRigidBody(pe::Vector3(0, -0.5, 0), pe::Vector3(2, 1, 2));
    rb0->setMass(4.0);
    rb0->setKinematic(true);
    pe::Matrix3 mat;
    pe::Vector3 vec;
    mat << 0.987341, -0.158614, 0, 0.158614, 0.987341, 0, 0, 0, 1;
    vec << 0.258654, 0.571685, -0.1;
    auto rb1 = createRigidBody(pe::Vector3::Zero(), pe::Vector3(1, 1, 1));
    rb1->setTransform(pe::Transform(mat, vec));

    auto np = pe_phys_collision::SimpleNarrowPhase();
    pe::Array<pe_phys_collision::CollisionPair> pairs = { {rb0, rb1} };
    pe::Array<pe_phys_collision::ContactResult*> result;
//    np.calcContactResults(pairs, result);
    result.push_back(new pe_phys_collision::ContactResult());
    result[0]->setObjectA(rb0);
    result[0]->setObjectB(rb1);
    result[0]->addContactPoint(-pe::Vector3(0, 1, 0), pe::Vector3(-0.155709, -0.001291916, -0.6), -0.001291916);
    result[0]->addContactPoint(-pe::Vector3(0, 1, 0), pe::Vector3(-0.155709, -0.001291916, 0.4), -0.001291916);
    result[0]->sortContactPoints();
    auto solver = new PrimalDualSolver();
    //auto solver = new SequentialImpulseSolver();
    solver->setupSolver(pe::Real(0.0167), { 0, -9.81, 0 }, { rb0, rb1 }, result, {});
    solver->solve();

    std::cout << rb0->getLinearVelocity() << std::endl;
    std::cout << rb1->getLinearVelocity() << std::endl;
}

int main() {
    utils::ThreadPool::init();
    testFrictionContactConstraint();
}