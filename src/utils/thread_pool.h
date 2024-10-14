#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <vector>
#include <queue>
#include <iostream>
#include <common/general.h>

namespace utils {

    /**
     * @brief A simple singleton thread pool.
     */
    class ThreadPool {
    public:
        static void init(uint32_t pool_size = 0);
        static void join();

        template<typename Function, typename... Args>
        static void addTask(Function&& fn, Args&&... args);

        template <typename Iterator, typename Function>
        static void forEach(Iterator first, Iterator last, Function&& fn);

        template <typename Function>
        static void forLoop(uint32_t count, Function&& fn);

        template <typename Function>
        static void forBatchedLoop(uint32_t count, uint32_t batchSize, Function&& fn);

    public:
        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

    private:
        ThreadPool() {}
        ~ThreadPool() { join(); }

        uint32_t _size = 0;
        std::vector<std::thread*> _pool;
        using Task = std::function<void()>;
        std::queue<Task> _tasks;
        int _task_count = 0;
        std::mutex _mtx;
        std::condition_variable _cv;

        static ThreadPool& getInstance();
    };

    template<typename Function, typename... Args>
    void ThreadPool::addTask(Function&& fn, Args&&... args) {
        auto& inst = getInstance();
        if (inst._size == -1) return;
        std::unique_lock<std::mutex> lock(inst._mtx);
        inst._tasks.emplace([fn, &args...]{ fn(std::forward<Args>(args)...); });
        inst._task_count++;
        inst._cv.notify_all();
    }

    template <typename Iterator, typename Function>
    void ThreadPool::forEach(Iterator first, Iterator last, Function&& fn) {
        std::vector<int> a(0);
        if (first == last) return;
        auto& inst = getInstance();
        if (inst._size == -1) return;
        std::unique_lock<std::mutex> lock(inst._mtx);
        auto p = first;
        while (p != last) {
            inst._tasks.emplace([fn, p, first]{ fn(*p, (int)(p - first)); }); // tricky: cannot use &fn
            inst._task_count++;
            p++;
        }
        inst._cv.notify_all();
    }

    template <typename Function>
    void ThreadPool::forLoop(uint32_t count, Function&& fn) {
        if (count == 0) return;
        auto& inst = getInstance();
        if (inst._size == -1) return;
        std::unique_lock<std::mutex> lock(inst._mtx);
        for (uint32_t i = 0; i < count; i++) {
            inst._tasks.emplace([fn, i]{ fn(i); });
            inst._task_count++;
        }
        inst._cv.notify_all();
    }

    template <typename Function>
    void ThreadPool::forBatchedLoop(uint32_t count, uint32_t batchSize, Function&& fn) {
        if (count == 0) return;
        auto& inst = getInstance();
        if (inst._size == -1) return;
        if (batchSize == 0) batchSize = count / inst._size / 3 + 1;
        std::unique_lock<std::mutex> lock(inst._mtx);
        for (uint32_t i = 0; i < count; i += batchSize) {
            inst._tasks.emplace([fn, i, count, batchSize]{
                for (uint32_t j = i; j < i + batchSize && j < count; j++) {
                    fn(j);
                }
            });
            inst._task_count++;
        }
        inst._cv.notify_all();
    }

} // namespace common