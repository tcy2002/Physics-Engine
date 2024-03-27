#include "phys/raycast/raycast/raycast_box.h"

using namespace pe_phys_ray;

void testRaycastBox() {
    RaycastBox rb;
    pe::Vector3 start(-0.0, 0.1, 0);
    pe::Vector3 direction(2, 1, 1);
    pe::Vector3 box_min(0.1, 0.1, 0.1);
    pe::Vector3 box_max(1, 1, 1);

    pe::Vector3 hit_point, hit_normal;
    pe::Real distance;
    bool ret = RaycastBox::rayHitBox(start, direction, box_min, box_max, hit_point, hit_normal, distance);
    std::cout << ret << hit_point << hit_normal << distance << std::endl;
}

int main() {
    testRaycastBox();
}