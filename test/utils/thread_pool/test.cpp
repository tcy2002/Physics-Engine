#include <iostream>
#include "utils/thread_pool.h"
#include <vector>

using namespace utils;

#define K 1000000000.0

void test(int i, int idx) {
    double n = idx;
    for (double j = 1.0; j < K; j += 1.0) {
        n += j;
        n -= j;
        n *= j;
        n /= j;
    }
    std::cout << n << std::endl;
}

void testThreadPool() {
    utils::ThreadPool::init();

    utils::ThreadPool::addTask([]{ std::cout << "Hello, world!" << std::endl; });
    utils::ThreadPool::join();
    std::vector<int> v(100);
    utils::ThreadPool::forEach(v.begin(), v.end(), test);
    utils::ThreadPool::join();
    std::cout << "end" << std::endl;

    utils::ThreadPool::deinit();
}

int main() {
    testThreadPool();
}