#pragma once

#include <iostream>
#include "vector_x.h"
#include "object_pool.h"

namespace utils {

    template <typename Scalar, size_t M, size_t N>
    class MatrixMN {
    protected:
        struct MatrixMNData {
            Scalar data[M][N];
        } *_m;
        static ObjectPool<MatrixMNData, M * N * sizeof(Scalar) * 256> _pool;

    public:
        MatrixMN(): _m(_pool.create()) {}
        explicit MatrixMN(Scalar value);
        MatrixMN(const MatrixMN& other);
        MatrixMN(MatrixMN&& other) noexcept;
        MatrixMN& operator=(const MatrixMN& other);
        MatrixMN& operator=(MatrixMN&& other) noexcept ;
        ~MatrixMN() { _pool.destroy(_m); }

        static size_t rows() { return M; }
        static size_t cols() { return N; }

        Scalar* operator[](size_t i) { return _m->data[i]; }
        const Scalar* operator[](size_t i) const { return _m->data[i]; }

        MatrixMN operator-() const;
        MatrixMN operator+(const MatrixMN& other) const;
        MatrixMN operator-(const MatrixMN& other) const;
        MatrixMN operator*(Scalar s) const;
        MatrixMN operator/(Scalar s) const;
        MatrixMN& operator+=(const MatrixMN& other);
        MatrixMN& operator-=(const MatrixMN& other);
        MatrixMN& operator*=(Scalar s);
        MatrixMN& operator/=(Scalar s);
        template <size_t N_OTHER>
        MatrixMN<Scalar, M, N_OTHER> operator*(const MatrixMN<Scalar, N, N_OTHER>& other) const;
        VectorX<Scalar, M> operator*(const VectorX<Scalar, N>& vec) const;
        static const MatrixMN& identity();
        static const MatrixMN& zeros();
        static const MatrixMN& ones();
    };

    template <typename Scalar, size_t M, size_t N>
    std::ostream& operator<<(std::ostream& os, const MatrixMN<Scalar, M, N>& mat);

    #include "matrix_m_n.cpp"

} // namespace utils