#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>
#include "logger.h"

namespace utils {

    template <typename T, size_t BlockSize>
    class Pool {
    protected:
        std::vector<void*> _blocks;

        struct FreeNode {
            FreeNode* next;
        };
        FreeNode* _free_node;

        int _used_num;
        int _free_num;

        static void* aligned_malloc(size_t size, int align);
        static void aligned_free(T* ptr);

        T* allocate();
        void deallocate(T* ptr);

        void pushFreeNode(FreeNode* ptr);
        void allocBlock();
        void destroyAll();

    public:
        Pool();
        ~Pool();

        template <typename ...Args>
        T* create(Args&&... args);
        void destroy(T* ptr);

        int getUsedNum() { return _used_num; }
        int getFreeNum() { return _free_num; }
    };

    #include "pool.cpp"

} // namespace utils