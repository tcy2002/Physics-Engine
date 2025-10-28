#pragma once

#include <vector>
#include <unordered_map>

namespace utils {

    /**
     * @brief A hash vector, similar usage to std::vector, but
     * provides high efficiency on sequential write and random
     * search.
     * extra api: find, replace
     */
    template <typename T, typename HashFunc, typename EqualFunc>
    class hash_vector {
    private:
        std::vector<T> list;
        std::unordered_multimap<uint32_t, uint32_t> index_table;
        HashFunc hash_func;
        EqualFunc equal_func;

    public:
        explicit hash_vector(uint32_t capacity = 997) {}
        ~hash_vector() {}

        const T& operator[](uint32_t idx) { return list[idx]; }
        const T& back() { return list.back(); }
        typename std::vector<T>::const_iterator begin() const { return list.cbegin(); }
        typename std::vector<T>::const_iterator end() const { return list.cend(); }
        uint32_t size() const { return (uint32_t)list.size(); }
        bool empty() const { return list.empty(); }
        std::vector<T> to_vector() const { return list; }

        bool push_back(const T& item);
        void pop_back();
        bool insert(typename std::vector<T>::const_iterator iterator, const T& item);
        bool insert(typename std::vector<T>::const_iterator iterator,
                    typename std::vector<T>::const_iterator iterator_begin,
                    typename std::vector<T>::const_iterator iterator_end);
        void erase(typename std::vector<T>::const_iterator iterator);
        void erase(typename std::vector<T>::const_iterator iterator_begin,
                   typename std::vector<T>::const_iterator iterator_end);
        void clear();

        typename std::vector<T>::const_iterator find(const T& item) const;
        bool replace(typename std::vector<T>::const_iterator iterator, const T& item);
    };

    #include "hash_vector.cpp"

} // namespace utils