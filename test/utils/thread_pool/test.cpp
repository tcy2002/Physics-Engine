#include <iostream>
#include "utils/thread_pool.h"
#include <vector>

using namespace utils;

#define K 100000000.0

void test(int i) {
    double n = i;
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

    int a = 1;
    utils::ThreadPool::addTask([](int& b){ std::cout << "Hello, world!" << std::endl; b = 2; }, a);
    utils::ThreadPool::join();
    std::cout << a << std::endl;
    std::vector<int> v(100, 100);
    utils::ThreadPool::forEach(v.begin(), v.end(), v.size(), test);
    utils::ThreadPool::join();
}

int main() {
    testThreadPool();
}