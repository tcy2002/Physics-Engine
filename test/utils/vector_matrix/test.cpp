#include "utils/vector_x.h"
#include "utils/matrix_m_n.h"

using namespace utils;

void testConstruction() {
    VectorX<double> vec1(4, 3);
    std::cout << vec1 << std::endl;
    VectorX<double> vec2 = vec1;
    std::cout << vec2 << std::endl;
    MatrixMN<double> mat1(3, 2, 5);
    std::cout << mat1 << std::endl;
    MatrixMN<double> mat2 = mat1;
    std::cout << mat2 << std::endl;
    MatrixMN<double> mat3;
    mat3 = mat1;
    std::cout << mat3 << std::endl;
}

void testMultiply() {
    VectorX<double> vec1(4);
    for (int i = 0; i < 4; i++) {
        vec1[i] = i;
    }
    MatrixMN<double> mat1(3, 4);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            mat1[i][j] = i * 4 + j;
        }
    }
    MatrixMN<double> mat2(4, 2);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            mat2[i][j] = i * 2 + j;
        }
    }
    std::cout << vec1 << std::endl;
    std::cout << mat1 << std::endl;
    std::cout << mat1 * vec1 << std::endl;
    MatrixMN<double> mat3(3, 2);
    mat3 = mat1 * mat2;
    std::cout << mat3 << std::endl;
}

int main() {
    testConstruction();
    testMultiply();
}