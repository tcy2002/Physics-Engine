#pragma once

#include <iostream>

namespace utils {

    template <typename Scalar>
    class VectorX {
    protected:
        Scalar* _data;
        size_t _size;

    public:
        VectorX(): _data(nullptr), _size(0) {}
        explicit VectorX(size_t size): _data(new Scalar[size]), _size(size) {}
        VectorX(size_t size, Scalar value);
        VectorX(const VectorX& other);
        VectorX(VectorX&& other) noexcept;
        VectorX& operator=(const VectorX& other);
        VectorX& operator=(VectorX&& other) noexcept;
        ~VectorX() { delete[] _data; }

        size_t size() const { return _size; }

        Scalar& operator[](size_t i) { return _data[i]; }
        const Scalar& operator[](size_t i) const { return _data[i]; }
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

        static const VectorX& zeros(size_t size);
        static const VectorX& ones(size_t size);
    };

    template <typename Scalar>
    std::ostream& operator<<(std::ostream& os, const VectorX<Scalar>& v);

    #include "vector_x.cpp"

} // namespace utils