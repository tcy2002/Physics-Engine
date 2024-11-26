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
        explicit MatrixMN(Scalar value): _m(_pool.create()) {
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    _m->data[i][j] = value;
                }
            }
        }
        MatrixMN(const MatrixMN& other): _m(_pool.create()) {
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    _m->data[i][j] = other[i][j];
                }
            }
        }
        MatrixMN(MatrixMN&& other) noexcept: _m(other._m) {
            other._m = nullptr;
        }
        MatrixMN& operator=(const MatrixMN& other) {
            if (this != &other) {
                for (size_t i = 0; i < M; i++) {
                    for (size_t j = 0; j < N; j++) {
                        _m->data[i][j] = other[i][j];
                    }
                }
            }
            return *this;
        }
        MatrixMN& operator=(MatrixMN&& other) noexcept {
            if (this != &other) {
                for (size_t i = 0; i < M; i++) {
                    for (size_t j = 0; j < N; j++) {
                        _m->data[i][j] = other[i][j];
                    }
                }
            }
            return *this;
        }
        ~MatrixMN() { _pool.destroy(_m); }

        static size_t rows() { return M; }
        static size_t cols() { return N; }

        Scalar* operator[](size_t i) { return _m->data[i]; }
        const Scalar* operator[](size_t i) const { return _m->data[i]; }

        MatrixMN operator-() const {
            MatrixMN res;
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    res[i][j] = -_m->data[i][j];
                }
            }
            return std::move(res);
        }
        MatrixMN operator+(const MatrixMN& other) const {
            MatrixMN res;
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    res[i][j] = _m->data[i][j] + other[i][j];
                }
            }
            return std::move(res);
        }
        MatrixMN operator-(const MatrixMN& other) const {
            MatrixMN res;
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    res[i][j] = _m->data[i][j] - other[i][j];
                }
            }
            return std::move(res);
        }
        MatrixMN operator*(Scalar s) const {
            MatrixMN res;
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    res[i][j] = _m->data[i][j] * s;
                }
            }
            return std::move(res);
        }
        MatrixMN operator/(Scalar s) const {
            MatrixMN res;
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    res[i][j] = _m->data[i][j] / s;
                }
            }
            return std::move(res);
        }
        MatrixMN& operator+=(const MatrixMN& other) {
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    _m->data[i][j] += other[i][j];
                }
            }
            return *this;
        }
        MatrixMN& operator-=(const MatrixMN& other) {
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    _m->data[i][j] -= other[i][j];
                }
            }
            return *this;
        }
        MatrixMN& operator*=(Scalar s) {
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    _m->data[i][j] *= s;
                }
            }
            return *this;
        }
        MatrixMN& operator/=(Scalar s) {
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    _m->data[i][j] /= s;
                }
            }
            return *this;
        }
        template <size_t N_OTHER>
        MatrixMN<Scalar, M, N_OTHER> operator*(const MatrixMN<Scalar, N, N_OTHER>& other) const {
            MatrixMN<Scalar, M, N_OTHER> res(0);
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N_OTHER; j++) {
                    for (size_t k = 0; k < N; k++) {
                        res[i][j] += _m->data[i][k] * other[k][j];
                    }
                }
            }
            return std::move(res);
        }
        VectorX<Scalar, M> operator*(const VectorX<Scalar, N>& vec) const {
            VectorX<Scalar, M> res(0);
            for (size_t i = 0; i < M; i++) {
                for (size_t j = 0; j < N; j++) {
                    res[i] += _m->data[i][j] * vec[j];
                }
            }
            return std::move(res);
        }

        static const MatrixMN& identity() {
            static MatrixMN res(0);
            static bool initialized = false;
            if (!initialized) {
                for (size_t i = 0; i < M && i < N; i++) {
                    res[i][i] = 1;
                }
                initialized = true;
            }
            return res;
        }
        static const MatrixMN& zeros() {
            static MatrixMN res(0);
            return res;
        }
        static const MatrixMN& ones() {
            static MatrixMN res(1);
            return res;
        }
    };

    template <typename Scalar, size_t M, size_t N>
    ObjectPool<typename MatrixMN<Scalar, M, N>::MatrixMNData, M * N * sizeof(Scalar) * 256> MatrixMN<Scalar, M, N>::_pool;

    template <typename Scalar, size_t M, size_t N>
    std::ostream& operator<<(std::ostream& os, const MatrixMN<Scalar, M, N>& mat) {
        os << "[";
        for (size_t i = 0; i < M; i++) {
            if (i != 0) {
                os << " ";
            }
            for (size_t j = 0; j < N; j++) {
                os << mat[i][j];
                if (j < N - 1) {
                    os << "\t";
                }
            }
            if (i < M - 1) {
                os << "\n";
            }
        }
        os << "]";
        return os;
    }

} // namespace utils