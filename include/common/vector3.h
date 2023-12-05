#pragma once

#include <iostream>
#include <cmath>
#include "general.h"

namespace common {

template <typename Scalar>
class Vector3 {
public:
    Scalar x, y, z;

    Vector3(): x(0.), y(0.), z(0.) {}
    Vector3(Scalar x, Scalar y, Scalar z): x(x), y(y), z(z) {}

    COMMON_FORCE_INLINE Scalar& operator[](int);
    COMMON_FORCE_INLINE Scalar operator[](int) const;
    COMMON_FORCE_INLINE Vector3 operator-() const;
    COMMON_FORCE_INLINE Vector3 operator+(const Vector3&) const;
    COMMON_FORCE_INLINE Vector3 operator-(const Vector3&) const;
    COMMON_FORCE_INLINE Vector3 operator*(Scalar) const;
    COMMON_FORCE_INLINE Vector3 operator/(Scalar) const;
    COMMON_FORCE_INLINE Vector3& operator+=(const Vector3&);
    COMMON_FORCE_INLINE Vector3& operator-=(const Vector3&);
    COMMON_FORCE_INLINE Vector3& operator*=(Scalar);
    COMMON_FORCE_INLINE Vector3& operator/=(Scalar);

    COMMON_FORCE_INLINE Scalar norm() const;
    COMMON_FORCE_INLINE Vector3 normalized() const;
    COMMON_FORCE_INLINE void normalize();
    COMMON_FORCE_INLINE Scalar dot(const Vector3& v) const;
    COMMON_FORCE_INLINE Vector3 cross(const Vector3& v) const;
    COMMON_FORCE_INLINE Vector3 project(const Vector3& v) const;
    COMMON_FORCE_INLINE Vector3 reflect(const Vector3& n) const;
    COMMON_FORCE_INLINE Vector3 rotate(const Vector3& axis, Scalar angle) const;
    COMMON_FORCE_INLINE Scalar angle(const Vector3& other) const;

    COMMON_FORCE_INLINE static const Vector3& zeros();
    COMMON_FORCE_INLINE static const Vector3& ones();
    COMMON_FORCE_INLINE static const Vector3& forward();
    COMMON_FORCE_INLINE static const Vector3& up();
    COMMON_FORCE_INLINE static const Vector3& right();
};

template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const Vector3<Scalar>& v);

#include "vector3.inl"

} // namespace common