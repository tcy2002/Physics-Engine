#include "thread_pool.h"

namespace common {

    ThreadPool& ThreadPool::getInstance() {
        static ThreadPool instance;
        return instance;
    }

    void ThreadPool::init(int pool_size) {
        auto& inst = getInstance();
        inst._stop.store(false);
        inst._task_num = 0;
        inst._size = pool_size > 0 ? pool_size : std::thread::hardware_concurrency();

        for (uint32_t i = 0; i < inst._size; i++) {
            inst._pool.push_back(new std::thread([]{
                auto& inst = getInstance();
                while(true) {
                    std::unique_lock<std::mutex> lock(inst._mtx);
                    if (inst._tasks.empty()) {
                        if (inst._stop.load()) return;
                        inst._cv.wait(lock, [&]{ return inst._stop || !inst._tasks.empty(); });
                        if (inst._stop.load()) return;
                    }
                    Task task(std::move(inst._tasks.front()));
                    inst._tasks.pop();
                    lock.unlock();

                    task();

                    lock.lock();
                    inst._task_num--;
                    inst._cv_join.notify_one();
                }
            }));
        }
    }

    void ThreadPool::deinit() {
        if (getInstance()._size == 0) return; // not initialized
        auto& inst = getInstance();
        join();
        inst._stop.store(true);
        inst._cv.notify_all();
        for (auto th : inst._pool) {
            if (th->joinable())
                th->join();
            delete th;
        }
        inst._pool.clear();
    }

    void ThreadPool::join() {
        if (getInstance()._size == 0) return; // not initialized
        auto& inst = getInstance();
        std::unique_lock<std::mutex> lock(inst._mtx);
        if (inst._task_num == 0) return;
        inst._cv_join.wait(lock, [&]{ return inst._task_num == 0; });
    }

} // namespace common
