#pragma once

#include <iostream>
#include "object_pool.h"

namespace utils {

    template <typename Scalar, size_t X>
    class VectorX {
    protected:
        struct VectorXData {
            Scalar data[X];
        } *_data;
        static ObjectPool<VectorXData, X * sizeof(Scalar) * 256> _pool;

    public:
        VectorX(): _data(_pool.create()) {}
        explicit VectorX(Scalar value): _data(_pool.create()) {
            for (size_t i = 0; i < X; i++) {
                _data->data[i] = value;
            }
        }
        VectorX(const VectorX& other): _data(_pool.create()) {
            for (size_t i = 0; i < X; i++) {
                _data->data[i] = other[i];
            }
        }
        VectorX(VectorX&& other) noexcept : _data(other._data) {
            other._data = nullptr;
        }
        VectorX& operator=(const VectorX& other) {
            if (this != &other) {
                for (size_t i = 0; i < X; i++) {
                    _data->data[i] = other[i];
                }
            }
            return *this;
        }
        VectorX& operator=(VectorX&& other) noexcept {
            if (this != &other) {
                for (size_t i = 0; i < X; i++) {
                    _data->data[i] = other[i];
                }
            }
            return *this;
        }
        ~VectorX() { _pool.destroy(_data); }

        static size_t size() { return X; }

        Scalar& operator[](size_t i) { return _data->data[i]; }
        const Scalar& operator[](size_t i) const { return _data->data[i]; }
        Scalar* data() { return _data->data; }
        const Scalar* data() const { return _data->data; }

        VectorX operator-() const {
            VectorX result;
            for (size_t i = 0; i < X; i++) {
                result[i] = -_data->data[i];
            }
            return std::move(result);
        }
        VectorX operator+(const VectorX& other) const {
            VectorX result;
            for (size_t i = 0; i < X; i++) {
                result[i] = _data->data[i] + other[i];
            }
            return std::move(result);
        }
        VectorX operator-(const VectorX& other) const {
            VectorX result;
            for (size_t i = 0; i < X; i++) {
                result[i] = _data->data[i] - other[i];
            }
            return std::move(result);
        }
        VectorX operator*(Scalar s) const {
            VectorX result;
            for (size_t i = 0; i < X; i++) {
                result[i] = _data->data[i] * s;
            }
            return std::move(result);
        }
        VectorX operator/(Scalar s) const {
            VectorX result;
            for (size_t i = 0; i < X; i++) {
                result[i] = _data->data[i] / s;
            }
            return std::move(result);
        }
        VectorX& operator+=(const VectorX& other) {
            for (size_t i = 0; i < X; i++) {
                _data->data[i] += other[i];
            }
            return *this;
        }
        VectorX& operator-=(const VectorX& other) {
            for (size_t i = 0; i < X; i++) {
                _data->data[i] -= other[i];
            }
            return *this;
        }
        VectorX& operator*=(Scalar s) {
            for (size_t i = 0; i < X; i++) {
                _data->data[i] *= s;
            }
            return *this;
        }
        VectorX& operator/=(Scalar s) {
            for (size_t i = 0; i < X; i++) {
                _data->data[i] /= s;
            }
            return *this;
        }

        Scalar norm() const {
            Scalar sum = 0;
            for (size_t i = 0; i < X; i++) {
                sum += _data->data[i] * _data->data[i];
            }
            return std::sqrt(sum);
        }
        Scalar norm2() const {
            Scalar sum = 0;
            for (size_t i = 0; i < X; i++) {
                sum += _data->data[i] * _data->data[i];
            }
            return sum;
        }
        VectorX normalized() const {
            Scalar n = norm();
            if (n == 0) {
                return *this;
            }
            return *this / n;
        }
        void normalize() {
            Scalar n = norm();
            if (n == 0) {
                return;
            }
            *this /= n;
        }
        VectorX mult(const VectorX& v) const {
            VectorX result;
            for (size_t i = 0; i < X; i++) {
                result[i] = _data->data[i] * v[i];
            }
            return std::move(result);
        }
        Scalar dot(const VectorX& v) const {
            Scalar sum = 0;
            for (size_t i = 0; i < X; i++) {
                sum += _data->data[i] * v[i];
            }
            return sum;
        }

        static const VectorX& zeros() {
            static VectorX zeros(0);
            return zeros;
        }
        static const VectorX& ones() {
            static VectorX ones(1);
            return ones;
        }
    };

    template <typename Scalar, size_t X>
    ObjectPool<typename VectorX<Scalar, X>::VectorXData, X * sizeof(Scalar) * 256> VectorX<Scalar, X>::_pool;

    template <typename Scalar, size_t X>
    std::ostream& operator<<(std::ostream& os, const VectorX<Scalar, X>& v) {
        os << "[";
        for (size_t i = 0; i < X; i++) {
            os << v[i];
            if (i < X - 1) {
                os << " ";
            }
        }
        os << "]";
        return os;
    }

} // namespace utils