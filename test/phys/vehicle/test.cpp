#include "phys/vehicle/tank/tank_template.h"

using namespace pe_phys_vehicle;

void testTank() {
    auto world = new pe_intf::World();
    auto tank = new TankTemplate();
    tank->init(world);
}

int main() {
    testTank();
}