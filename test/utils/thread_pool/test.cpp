#include <iostream>
#include "utils/thread_pool.h"
#include <vector>

using namespace utils;

void test(int i, int idx) {
    COMMON_Sleep(idx * 1000);
    std::cout << idx << std::endl;
}

void testThreadPool() {
    utils::ThreadPool::init();

    utils::ThreadPool::addTask([]{ std::cout << "Hello, world!" << std::endl; });
    utils::ThreadPool::join();
    std::vector<int> v(10);
    utils::ThreadPool::forEach(v.begin(), v.end(), test);
    utils::ThreadPool::join();
    std::cout << "end" << std::endl;

    utils::ThreadPool::deinit();
}

int main() {
    testThreadPool();
}