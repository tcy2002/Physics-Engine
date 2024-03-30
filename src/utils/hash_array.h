#pragma once

#include <vector>
#include <unordered_map>
#include <functional>
#include <iostream>

namespace utils {

    /**
     * @brief A hash array, similar usage to std::vector, but
     * can provide high efficiency on sequential write and random
     * search.
     */
    template <typename T, typename HashFunc, typename EqualFunc>
    class HashArray {
    private:
        std::unordered_multimap<uint32_t, int> _table;
        std::vector<T> _list;
        HashFunc _hash_func;
        EqualFunc _equal_func;

    public:
        HashArray() {}
        ~HashArray() {}

        T& operator[](uint32_t idx) { return _list[idx]; }
        T& back() { return _list.back(); }
        typename std::vector<T>::iterator begin() { return _list.begin(); }
        typename std::vector<T>::iterator end() { return _list.end(); }
        uint32_t size() const { return (uint32_t)_list.size(); }
        bool empty() const { return _list.empty(); }

        bool contains(const T& item) const;
        void push_back(const T& item);
        void pop_back();
        void insert(typename std::vector<T>::iterator iterator, const T& item);
        void insert(typename std::vector<T>::iterator iterator, const std::vector<T>& items);
        void erase(typename std::vector<T>::iterator iterator);
        void erase(typename std::vector<T>::iterator it_begin, typename std::vector<T>::iterator it_end);
        void clear() { _table.clear(); _list.clear(); }

        std::vector<T> to_vector() const { return _list; }
        typename std::vector<T>::iterator find_first(const T& item);

        void debug() {
            for (auto& it : _table) {
                std::cout << it.first << " " << it.second << std::endl;
            }
        }
    };

    #include "hash_array.cpp"

} // namespace utils