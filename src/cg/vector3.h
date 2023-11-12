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

    PE_FORCE_INLINE real& operator[](int);
    PE_FORCE_INLINE real operator[](int) const;
    PE_FORCE_INLINE Vector3 operator-() const;
    PE_FORCE_INLINE Vector3 operator+(const Vector3&) const;
    PE_FORCE_INLINE Vector3 operator-(const Vector3&) const;
    PE_FORCE_INLINE Vector3 operator*(real) const;
    PE_FORCE_INLINE Vector3 operator/(real) const;
    PE_FORCE_INLINE Vector3& operator+=(const Vector3&);
    PE_FORCE_INLINE Vector3& operator-=(const Vector3&);
    PE_FORCE_INLINE Vector3& operator*=(real);
    PE_FORCE_INLINE Vector3& operator/=(real);

    PE_FORCE_INLINE real norm() const;
    PE_FORCE_INLINE Vector3 normalized() const;
    PE_FORCE_INLINE void normalize();
    PE_FORCE_INLINE real dot(const Vector3& v) const;
    PE_FORCE_INLINE Vector3 cross(const Vector3& v) const;
    PE_FORCE_INLINE Vector3 project(const Vector3& v) const;
    PE_FORCE_INLINE Vector3 reflect(const Vector3& n) const;
    PE_FORCE_INLINE Vector3 rotate(const Vector3& axis, real angle) const;
    PE_FORCE_INLINE real angle(const Vector3& other) const;

    PE_FORCE_INLINE static const Vector3& zeros();
    PE_FORCE_INLINE static const Vector3& ones();
    PE_FORCE_INLINE static const Vector3& forward();
    PE_FORCE_INLINE static const Vector3& up();
    PE_FORCE_INLINE static const Vector3& right();

    PE_FORCE_INLINE friend std::ostream& operator<<(std::ostream& os, const Vector3& v);
};

#include "vector3.inl"

} // namespace pe_cg