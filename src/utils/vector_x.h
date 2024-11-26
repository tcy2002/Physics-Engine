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
        explicit VectorX(Scalar value);
        VectorX(const VectorX& other);
        VectorX(VectorX&& other) noexcept;
        VectorX& operator=(const VectorX& other);
        VectorX& operator=(VectorX&& other) noexcept;
        ~VectorX() { _pool.destroy(_data); }

        static size_t size() { return X; }

        Scalar& operator[](size_t i) { return _data->data[i]; }
        const Scalar& operator[](size_t i) const { return _data->data[i]; }
        Scalar* data() { return _data->data; }
        const Scalar* data() const { return _data->data; }

        VectorX operator-() const;
        VectorX operator+(const VectorX& other) const;
        VectorX operator-(const VectorX& other) const;
        VectorX operator*(Scalar s) const;
        VectorX operator/(Scalar s) const;
        VectorX& operator+=(const VectorX& other);
        VectorX& operator-=(const VectorX& other);
        VectorX& operator*=(Scalar s);
        VectorX& operator/=(Scalar s);

        Scalar norm() const;
        Scalar norm2() const;
        VectorX normalized() const;
        void normalize();
        VectorX mult(const VectorX& v) const;
        Scalar dot(const VectorX& v) const;

        static const VectorX& zeros();
        static const VectorX& ones();
    };

    template <typename Scalar, size_t X>
    std::ostream& operator<<(std::ostream& os, const VectorX<Scalar, X>& v);

    #include "vector_x.cpp"

} // namespace utils