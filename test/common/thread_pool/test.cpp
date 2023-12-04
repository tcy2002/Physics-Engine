#include <iostream>
#include <random>
#include "common/thread_pool.h"

void exec(uint64_t& t) {
    std::default_random_engine e(t);
    std::uniform_int_distribution<int> u(1, 2);
    int mat_size = 100;
    auto mat = new uint64_t[mat_size];

    for (int i = 0; i < mat_size; i++) {
        mat[i] = i + 1;
    }
    for (int i = 0; i < u(e); i++) {
        for (int j = 0; j < mat_size; j++) {
            for(int k = 0; k < mat_size; k++) {
                mat[j] = mat[j] + mat[k] * 997;
                mat[k] = mat[j] - mat[k] * 1003;
            }
        }
    }
    for (int i = 0; i < mat_size; i++) {
        t += mat[i];
    }

    delete[] mat;
}

int main() {
    uint64_t start, time1, time2;
    int max_pool_size = 32;
    int avg_time = 10;
    int thread_num = 1024;

    // init vectors for parallel and sequential execution
    uint64_t vec[thread_num], vec_seq[thread_num];
    for (int i = 0; i < thread_num; i++) {
        vec[i] = i + 1;
        vec_seq[i] = i + 1;
    }

    for (int n = 1; n <= max_pool_size; n++) {
        time1 = time2 = 0;
        for (int i = 0; i < avg_time; i++) {
            common::ThreadPool::init(n);

            // parallel test
            start = COMMON_GetTickCount();
            common::ThreadPool::forEach(vec + 0, vec + thread_num, exec);
            common::ThreadPool::join();
            time1 += COMMON_GetTickCount() - start;

            // sequential test
            start = COMMON_GetTickCount();
            for (int j = 0; j < thread_num; j++) {
                exec(vec_seq[j]);
            }
            time2 += COMMON_GetTickCount() - start;

            // check equalities of two vectors
            for (int j = 0; j < thread_num; j++) {
                if (vec[j] != vec_seq[j]) {
                    std::cout << "error: " << vec[j] << " != " << vec_seq[j] << std::endl;
                    return 0;
                }
            }

            common::ThreadPool::deinit();
        }
        std::cout << "pool size: " << n <<
                  " sequential: " << (double)time2 / avg_time <<
                  "ms, parallel: " << (double)time1 / avg_time <<
                  "ms, ratio: " << (double)time2 / (double)time1 << std::endl;
    }
    return 0;
}
