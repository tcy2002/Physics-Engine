#pragma once

#include <iostream>
#include "vector_x.h"
#include <common/matrix3x3.h>

namespace utils {

    template <typename Scalar>
    class MatrixMN {
    protected:
        Scalar** _data;
        size_t _rows, _cols;
        bool _sub_ref = false;
        Scalar** getSubData(int rows, int cols, int start_row, int start_col) const;

    public:
        MatrixMN(): _data(nullptr), _rows(0), _cols(0) {}
        MatrixMN(size_t M, size_t N);
        MatrixMN(size_t M, size_t N, Scalar** data);
        MatrixMN(Scalar value, size_t M, size_t N);
        explicit MatrixMN(const common::Matrix3x3<Scalar>& mat3x3);
        MatrixMN(const MatrixMN& other);
        MatrixMN(MatrixMN&& other) noexcept;
        MatrixMN& operator=(const MatrixMN& other);
        MatrixMN& operator=(MatrixMN&& other) noexcept ;
        ~MatrixMN();

        size_t rows() const { return _rows; }
        size_t cols() const { return _cols; }
        void resize(size_t M, size_t N);
        void resize(size_t M, size_t N, Scalar value);
        void setValue(Scalar v);

        Scalar* operator[](size_t i) { return _data[i]; }
        const Scalar* operator[](size_t i) const { return _data[i]; }

        MatrixMN operator-() const;
        MatrixMN operator+(const MatrixMN& other) const;
        MatrixMN operator-(const MatrixMN& other) const;
        MatrixMN operator*(Scalar s) const;
        MatrixMN operator/(Scalar s) const;
        MatrixMN& operator+=(const MatrixMN& other);
        MatrixMN& operator-=(const MatrixMN& other);
        MatrixMN& operator*=(Scalar s);
        MatrixMN& operator/=(Scalar s);
        MatrixMN operator*(const MatrixMN& other) const;
        VectorX<Scalar> operator*(const VectorX<Scalar>& vec) const;

        void transpose();
        MatrixMN transposed() const;
        VectorX<Scalar> getColumn(size_t index) const;
        MatrixMN getSubMatrix(size_t rows, size_t cols, size_t start_row, size_t start_col) const;
        MatrixMN getRefSubMatrix(size_t rows, size_t cols, size_t start_row, size_t start_col);
        VectorX<Scalar> getSubRowVector(size_t cols, size_t start_row, size_t start_col) const;
        VectorX<Scalar> getRefSubRowVector(size_t cols, size_t start_row, size_t start_col);
        void setSubMatrix(const MatrixMN& mat, size_t start_row, size_t start_col);
        void setSubVector(const VectorX<Scalar>& vec, size_t start_row, size_t start_col, bool in_row);
        common::Matrix3x3<Scalar> toMatrix3x3() const;

        static const MatrixMN& identity(size_t size);
        static const MatrixMN& zeros(size_t M, size_t N);
        static const MatrixMN& ones(size_t M, size_t N);
    };

    template <typename Scalar>
    MatrixMN<Scalar> operator*(Scalar s, const MatrixMN<Scalar>& v);

    template <typename Scalar>
    VectorX<Scalar> operator*(const VectorX<Scalar>& v, const MatrixMN<Scalar>& mat);

    template <typename Scalar>
    std::ostream& operator<<(std::ostream& os, const MatrixMN<Scalar>& mat);

    #include "matrix_m_n.cpp"

} // namespace utils