#pragma once

#include <vector>
#include <algorithm>
#include "logger.h"

namespace utils {

    template <typename T, size_t BlockSize>
    class ObjectPool {
    protected:
        std::vector<void*> _blocks;

        struct FreeNode {
            FreeNode* next;
        };
        FreeNode* _free_node;

        static void* aligned_malloc(size_t size, int align);
        static void aligned_free(T* ptr);

        T* allocate();
        void deallocate(T* ptr);

        void pushFreeNode(FreeNode* ptr);
        void allocBlock();
        void destroyAll();

    public:
        ObjectPool();
        ~ObjectPool();

        template <typename ...Args>
        T* create(Args&&... args);
        void destroy(T* ptr);
    };

    #include "object_pool.cpp"

} // namespace utils