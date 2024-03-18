#include "utils/pool.h"
#include <common/mesh.h>

using namespace utils;

void testPool() {
    Pool<common::Mesh<float>, 4096> pool;
    std::cout << sizeof(common::Mesh<float>) << std::endl;
}

int main() {
    testPool();
}