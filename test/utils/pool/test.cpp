#include "utils/pool.h"
#include "phys/constraint/constraint/friction_contact_constraint.h"

using namespace utils;

void testPool() {
    Pool<pe_phys_constraint::FrictionContactConstraint, 131072> pool;
    std::vector<pe_phys_constraint::FrictionContactConstraint*> list;
    for (int i = 0; i < 64; i++) {
        list.push_back(pool.create());
    }
    for (auto fcc : list) {
        pool.destroy(fcc);
    }
}

void testInvalidPool() {
    Pool<pe_phys_constraint::FrictionContactConstraint, 4096> pool;
}

int main() {
    testPool();
    testInvalidPool();
}