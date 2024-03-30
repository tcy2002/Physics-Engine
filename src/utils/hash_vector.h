#pragma once

#include <vector>
#include <functional>

namespace utils {

    /**
     * @brief A hash vector, similar usage to std::vector, but
     * can provide high efficiency on sequential write and random
     * search.
     */
    template <typename T, typename HashFunc, typename EqualFunc>
    class hash_vector {
    private:
        struct link_node {
            uint32_t val;
            link_node* next;
            link_node(): val(-1), next(nullptr) {}
            explicit link_node(uint32_t v, link_node* next = nullptr): val(v), next(next) {}
        };

        std::vector<T> list;
        std::vector<link_node*> index_table;
        HashFunc hash_func;
        EqualFunc equal_func;
        uint32_t capacity;
        uint32_t hash(const T& val) const { return hash_func(val) % capacity; }

    public:
        explicit hash_vector(uint32_t capacity = 997);
        hash_vector(const hash_vector& other) { *this = other; }
        ~hash_vector();
        hash_vector& operator=(const hash_vector& other);

        T& operator[](uint32_t idx) { return list[idx]; }
        T& back() { return list.back(); }
        auto begin() -> decltype(list.begin()) { return list.begin(); }
        auto end() -> decltype(list.end()) { return list.end(); }
        uint32_t size() const { return (uint32_t)list.size(); }
        bool empty() const { return list.empty(); }
        std::vector<T> to_vector() const { return list; }

        bool contains(const T& item) const;
        uint32_t index_of(const T& item) const;
        bool push_back(const T& item);
        void pop_back();
        bool insert(const T& item, uint32_t idx);
        bool append(const hash_vector& other);
        bool append(const std::vector<T>& other);
        bool erase(const T& item);
        void erase_at(uint32_t idx);
        bool replace(const T& old_item, const T& new_item);
        bool replace_at(uint32_t idx, const T& new_item);
        void clear();
    };

    #include "hash_vector.cpp"

} // namespace utils