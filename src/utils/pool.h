#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

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

        T* allocate();
        void deallocate(T* ptr);

        void pushFreeNode(FreeNode* ptr);
        void allocBlock();
        void destroyAll();

    public:
        Pool(): _used_num(0), _free_num(0), _free_node(nullptr) {}
        ~Pool();

        template <typename ...Args>
        T* create(Args&&... args);
        void destroy(T* ptr);
    };

    #include "pool.cpp"

} // namespace utils