#include "thread_pool.h"

namespace utils {

    ThreadPool& ThreadPool::getInstance() {
        static ThreadPool instance;
        return instance;
    }

    void ThreadPool::init(uint32_t pool_size) {
        auto& inst = getInstance();
		if (inst._size > 0) return; // already initialized
        inst._size = pool_size > 0 ? pool_size : std::thread::hardware_concurrency();
        inst._task_count = 0;

        for (uint32_t i = 0; i < inst._size; i++) {
            inst._pool.push_back(new std::thread([]{
                auto& inst = getInstance();
                while(true) {
                    std::unique_lock<std::mutex> lock(inst._mtx);
                    inst._cv.wait(lock, [&]{ return !inst._tasks.empty(); });
                    Task task(std::move(inst._tasks.front()));
                    inst._tasks.pop();
                    lock.unlock();

                    task();

                    lock.lock();
                    inst._task_count--;
                    inst._cv.notify_all();
                }
            }));
        }
    }

    void ThreadPool::join() {
        auto& inst = getInstance();
        if (inst._size == 0) return; // not initialized
        std::unique_lock<std::mutex> lock(inst._mtx);
        inst._cv.wait(lock, [&]{ return inst._task_count == 0; });
    }

} // namespace common
