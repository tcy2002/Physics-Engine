#include "utils/vector_x.h"
#include "utils/matrix_m_n.h"

using namespace utils;

void testConstruction() {
    VectorX<double, 4> vec1(3);
    std::cout << vec1 << std::endl;
    VectorX<double, 4> vec2 = vec1;
    std::cout << vec2 << std::endl;
    MatrixMN<double, 3, 2> mat1(5);
    std::cout << mat1 << std::endl;
    MatrixMN<double, 3, 2> mat2 = mat1;
    std::cout << mat2 << std::endl;
}

void testMultiply() {
    VectorX<double, 4> vec1;
    for (int i = 0; i < 4; i++) {
        vec1[i] = i;
    }
    MatrixMN<double, 3, 4> mat1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            mat1[i][j] = i * 4 + j;
        }
    }
    MatrixMN<double, 4, 2> mat2;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            mat2[i][j] = i * 2 + j;
        }
    }
    std::cout << vec1 << std::endl;
    std::cout << mat1 << std::endl;
    std::cout << mat1 * vec1 << std::endl;
    std::cout << mat1 * mat2 << std::endl;
}

int main() {
    testConstruction();
    testMultiply();
}