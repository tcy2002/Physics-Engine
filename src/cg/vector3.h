#pragma once

#include <iostream>
#include <cmath>
#include "def.h"

namespace pe_cg {

class Vector3 {
public:
    real x, y, z;

    Vector3(): x(0.), y(0.), z(0.) {}
    Vector3(real x, real y, real z): x(x), y(y), z(z) {}

    PHYS_FORCE_INLINE real& operator[](int);
    PHYS_FORCE_INLINE real operator[](int) const;
    PHYS_FORCE_INLINE Vector3 operator-() const;
    PHYS_FORCE_INLINE Vector3 operator+(const Vector3&) const;
    PHYS_FORCE_INLINE Vector3 operator-(const Vector3&) const;
    PHYS_FORCE_INLINE Vector3 operator*(real) const;
    PHYS_FORCE_INLINE Vector3 operator/(real) const;
    PHYS_FORCE_INLINE Vector3& operator+=(const Vector3&);
    PHYS_FORCE_INLINE Vector3& operator-=(const Vector3&);
    PHYS_FORCE_INLINE Vector3& operator*=(real);
    PHYS_FORCE_INLINE Vector3& operator/=(real);

    PHYS_FORCE_INLINE real norm() const;
    PHYS_FORCE_INLINE Vector3 normalized() const;
    PHYS_FORCE_INLINE void normalize();
    PHYS_FORCE_INLINE real dot(const Vector3& v) const;
    PHYS_FORCE_INLINE Vector3 cross(const Vector3& v) const;
    PHYS_FORCE_INLINE Vector3 project(const Vector3& v) const;
    PHYS_FORCE_INLINE Vector3 reflect(const Vector3& n) const;
    PHYS_FORCE_INLINE Vector3 rotate(const Vector3& axis, real angle) const;
    PHYS_FORCE_INLINE real angle(const Vector3& other) const;

    PHYS_FORCE_INLINE static const Vector3& zeros();
    PHYS_FORCE_INLINE static const Vector3& ones();
    PHYS_FORCE_INLINE static const Vector3& forward();
    PHYS_FORCE_INLINE static const Vector3& up();
    PHYS_FORCE_INLINE static const Vector3& right();

    PHYS_FORCE_INLINE friend std::ostream& operator<<(std::ostream& os, const Vector3& v);
};

#include "vector3.inl"

} // namespace pe_cg