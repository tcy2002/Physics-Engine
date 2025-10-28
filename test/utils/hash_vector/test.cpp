#include <iostream>
#include "utils/hash_vector.h"
#include "test_general.h"

using namespace utils;

void testPushBack() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);

    ASSERT_EQUAL_INT(hashVector.size(), 3)
    ASSERT_EQUAL_INT(hashVector.push_back(3), 0)
    ASSERT_EQUAL_INT(hashVector.push_back(4), 1)
    ASSERT_EQUAL_INT(hashVector.size(), 4)
}

void testPopBack() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);

    hashVector.pop_back();
    ASSERT_EQUAL_INT(hashVector.size(), 2)
    hashVector.pop_back();
    ASSERT_EQUAL_INT(hashVector.size(), 1)
    hashVector.pop_back();
    ASSERT_EQUAL_INT(hashVector.size(), 0)
}

void testInsert() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);

    ASSERT_EQUAL_INT(hashVector.insert(hashVector.begin(), 0), 1)
    ASSERT_EQUAL_INT(hashVector.size(), 4)
    ASSERT_EQUAL_INT(hashVector[0], 0)
    ASSERT_EQUAL_INT(hashVector[1], 1)
    ASSERT_EQUAL_INT(hashVector[2], 2)
    ASSERT_EQUAL_INT(hashVector[3], 3)

    ASSERT_EQUAL_INT(hashVector.insert(hashVector.begin() + 2, 2), 0)
    ASSERT_EQUAL_INT(hashVector.size(), 4)
    ASSERT_EQUAL_INT(hashVector[0], 0)
    ASSERT_EQUAL_INT(hashVector[1], 1)
    ASSERT_EQUAL_INT(hashVector[2], 2)
    ASSERT_EQUAL_INT(hashVector[3], 3)
}

void testInsertMany() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);
    std::vector<uint32_t> vec = {1, 4, 5};
    std::vector<uint32_t> vec2 = {4, 5};

    ASSERT_EQUAL_INT(hashVector.insert(hashVector.begin(), vec.begin(), vec.end()), 0)
    ASSERT_EQUAL_INT(hashVector.size(), 3)
    ASSERT_EQUAL_INT(hashVector[0], 1)
    ASSERT_EQUAL_INT(hashVector[1], 2)
    ASSERT_EQUAL_INT(hashVector[2], 3)

    ASSERT_EQUAL_INT(hashVector.insert(hashVector.begin(), vec2.begin(), vec2.end()), 1)
    ASSERT_EQUAL_INT(hashVector.size(), 5)
    ASSERT_EQUAL_INT(hashVector[0], 4)
    ASSERT_EQUAL_INT(hashVector[1], 5)
    ASSERT_EQUAL_INT(hashVector[2], 1)
    ASSERT_EQUAL_INT(hashVector[3], 2)
    ASSERT_EQUAL_INT(hashVector[4], 3)
}

void testErase() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);

    hashVector.erase(hashVector.begin());
    ASSERT_EQUAL_INT(hashVector.size(), 2)
    ASSERT_EQUAL_INT(hashVector[0], 2)
    ASSERT_EQUAL_INT(hashVector[1], 3)

    hashVector.erase(hashVector.begin(), hashVector.end());
    ASSERT_EQUAL_INT(hashVector.size(), 0)
}

void testFind() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);

    ASSERT_EQUAL_INT(*hashVector.find(1), 1)
    ASSERT_EQUAL_INT(*hashVector.find(2), 2)
    ASSERT_EQUAL_INT(*hashVector.find(3), 3)
    ASSERT_EQUAL_INT(hashVector.find(4), hashVector.end())
}

void testReplace() {
    hash_vector<uint32_t, std::hash<uint32_t>, std::equal_to<uint32_t>> hashVector;
    hashVector.push_back(1);
    hashVector.push_back(2);
    hashVector.push_back(3);

    ASSERT_EQUAL_INT(hashVector.replace(hashVector.begin(), 4), 1)
    ASSERT_EQUAL_INT(hashVector.size(), 3)
    ASSERT_EQUAL_INT(hashVector[0], 4)
    ASSERT_EQUAL_INT(hashVector[1], 2)
    ASSERT_EQUAL_INT(hashVector[2], 3)

    ASSERT_EQUAL_INT(*hashVector.find(4), 4)
    ASSERT_EQUAL_INT(*hashVector.find(2), 2)
    ASSERT_EQUAL_INT(*hashVector.find(3), 3)
}

int main() {
    testPushBack();
    testPopBack();
    testInsert();
    testInsertMany();
    testErase();
    testFind();
    testReplace();
}