#include <iostream>
#include "utils/hash_array.h"
#include "phys/phys_general.h"

using namespace utils;

void testHashArray() {
    HashArray<uint32_t, pe::uint32_hash, pe::uint32_equal> array;
    array.push_back(1);
    array.push_back(1);
    array.push_back(2);
    array.push_back(3);
    array.push_back(3);
    array.push_back(3);
    array.push_back(4);
    array.push_back(5);
    for (auto& i : array) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    array.pop_back();
    for (auto& i : array) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    array.push_back(5);
    for (auto& i : array) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    array.insert(array.end(), 6);
    std::vector<uint32_t> vec = {7, 8, 8, 9};
    array.insert(array.begin(), vec);
    for (auto& i : array) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    array.erase(array.begin());
    array.erase(array.begin(), array.begin() + 2);
    for (auto& i : array) {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << array.contains(1) << std::endl;
    std::cout << array.contains(7) << std::endl;
    std::cout << (array.find_first(111) - array.begin()) << std::endl;

    array.debug();
}

int main() {
    testHashArray();
}