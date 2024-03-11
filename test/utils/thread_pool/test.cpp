#include <iostream>
#include "utils/thread_pool.h"
#include <vector>

using namespace utils;

void testThreadPool() {
    utils::ThreadPool::init();
    utils::ThreadPool::addTask([]{ std::cout << "Hello, world!" << std::endl; });
    utils::ThreadPool::join();
    std::vector<int> v(100);
    utils::ThreadPool::forEach(v.begin(), v.end(), [](int& i, int index){ i = index; });
    utils::ThreadPool::join();
    for (int i = 0; i < 100; i++) {
        std::cout << v[i] << " ";
    }
}

int main() {
    testThreadPool();
}