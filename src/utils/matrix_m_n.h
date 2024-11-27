#pragma once

#include <iostream>
#include "vector_x.h"

namespace utils {

    template <typename Scalar>
    class MatrixMN {
    protected:
        Scalar** _data;
        size_t _rows, _cols;

    public:
        MatrixMN() = delete;
        MatrixMN(size_t M, size_t N);
        MatrixMN(size_t M, size_t N, Scalar value);
        MatrixMN(const MatrixMN& other);
        MatrixMN(MatrixMN&& other) noexcept;
        MatrixMN& operator=(const MatrixMN& other);
        MatrixMN& operator=(MatrixMN&& other) noexcept ;
        ~MatrixMN();

        size_t rows() const { return _rows; }
        size_t cols() const { return _cols; }

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
        static const MatrixMN& identity(size_t size);
        static const MatrixMN& zeros(size_t M, size_t N);
        static const MatrixMN& ones(size_t M, size_t N);
    };

    template <typename Scalar>
    std::ostream& operator<<(std::ostream& os, const MatrixMN<Scalar>& mat);

    #include "matrix_m_n.cpp"

} // namespace utils