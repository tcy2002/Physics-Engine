#pragma once

#include <iostream>
#include <cmath>
#include "def.h"

namespace pe_common {

class Vector3 {
public:
    PEReal x, y, z;

    Vector3(): x(0.), y(0.), z(0.) {}
    Vector3(PEReal x, PEReal y, PEReal z): x(x), y(y), z(z) {}

    PE_FORCE_INLINE PEReal& operator[](int);
    PE_FORCE_INLINE PEReal operator[](int) const;
    PE_FORCE_INLINE Vector3 operator-() const;
    PE_FORCE_INLINE Vector3 operator+(const Vector3&) const;
    PE_FORCE_INLINE Vector3 operator-(const Vector3&) const;
    PE_FORCE_INLINE Vector3 operator*(PEReal) const;
    PE_FORCE_INLINE Vector3 operator/(PEReal) const;
    PE_FORCE_INLINE Vector3& operator+=(const Vector3&);
    PE_FORCE_INLINE Vector3& operator-=(const Vector3&);
    PE_FORCE_INLINE Vector3& operator*=(PEReal);
    PE_FORCE_INLINE Vector3& operator/=(PEReal);

    PE_FORCE_INLINE PEReal norm() const;
    PE_FORCE_INLINE Vector3 normalized() const;
    PE_FORCE_INLINE void normalize();
    PE_FORCE_INLINE PEReal dot(const Vector3& v) const;
    PE_FORCE_INLINE Vector3 cross(const Vector3& v) const;
    PE_FORCE_INLINE Vector3 project(const Vector3& v) const;
    PE_FORCE_INLINE Vector3 reflect(const Vector3& n) const;
    PE_FORCE_INLINE Vector3 rotate(const Vector3& axis, PEReal angle) const;
    PE_FORCE_INLINE PEReal angle(const Vector3& other) const;

    PE_FORCE_INLINE static const Vector3& zeros();
    PE_FORCE_INLINE static const Vector3& ones();
    PE_FORCE_INLINE static const Vector3& forward();
    PE_FORCE_INLINE static const Vector3& up();
    PE_FORCE_INLINE static const Vector3& right();

    PE_FORCE_INLINE friend std::ostream& operator<<(std::ostream& os, const Vector3& v);
};

#include "vector3.inl"

} // namespace pe_common