#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <vector>
#include <queue>
#include "def.h"

namespace common {

/**
 * @brief A simple singleton thread pool.
 */
class ThreadPool {
public:
    static void init(int pool_size = 0);
    static void deinit();
    static void join();

    template<typename Function, typename... Args>
    static void addTask(Function&& fn, Args&&... args) {
        auto& inst = getInstance();
        std::unique_lock<std::mutex> lock(inst._mtx);
        inst._tasks.emplace([&]{ fn(args...); });
        inst._task_num++;
        inst._cv.notify_one();
    }

    template <typename Iterator, typename Function>
    static void forEach(Iterator&& first, Iterator&& last, Function&& fn) {
        auto& inst = getInstance();
        std::unique_lock<std::mutex> lock(inst._mtx);
        while (first != last) {
            inst._tasks.emplace([&fn, first]{ fn(*first); });
            inst._task_num++;
            ++first;
        }
        inst._cv.notify_all();
    }

private:
    ThreadPool() = default;
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ~ThreadPool() { deinit(); }

    uint32_t _size;
    std::vector<std::thread*> _pool;
    using Task = std::function<void()>;
    std::queue<Task> _tasks;
    int _task_num;
    std::mutex _mtx;
    std::condition_variable _cv;
    std::condition_variable _cv_join;
    std::atomic<bool> _stop;

    static ThreadPool& getInstance();
};

} // namespace common