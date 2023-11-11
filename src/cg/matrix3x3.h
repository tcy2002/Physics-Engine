#pragma once

#include <iostream>
#include <cmath>
#include "def.h"
#include "vector3.h"

namespace pe_cg {

class Matrix3x3 {
private:
protected:
    real _m[3][3];

public:
    Matrix3x3(): _m{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} {}
    Matrix3x3(real m00, real m01, real m02, real m10, real m11, real m12, real m20, real m21, real m22):
    _m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

    PHYS_FORCE_INLINE real* operator[](int);
    PHYS_FORCE_INLINE const real* operator[](int) const;
    PHYS_FORCE_INLINE Matrix3x3 operator-() const;
    PHYS_FORCE_INLINE Matrix3x3 operator+(const Matrix3x3&) const;
    PHYS_FORCE_INLINE Matrix3x3 operator-(const Matrix3x3&) const;
    PHYS_FORCE_INLINE Matrix3x3 operator*(real) const;
    PHYS_FORCE_INLINE Matrix3x3 operator/(real) const;
    PHYS_FORCE_INLINE Matrix3x3& operator+=(const Matrix3x3&);
    PHYS_FORCE_INLINE Matrix3x3& operator-=(const Matrix3x3&);
    PHYS_FORCE_INLINE Matrix3x3& operator*=(real);
    PHYS_FORCE_INLINE Matrix3x3& operator/=(real);
    PHYS_FORCE_INLINE Matrix3x3 operator*(const Matrix3x3&) const;
    PHYS_FORCE_INLINE Matrix3x3& operator*=(const Matrix3x3&);
    PHYS_FORCE_INLINE Vector3 operator*(const Vector3&) const;

    PHYS_FORCE_INLINE real determinant() const;
    PHYS_FORCE_INLINE real trace() const;
    PHYS_FORCE_INLINE Matrix3x3 transposed() const;
    PHYS_FORCE_INLINE void transpose();
    PHYS_FORCE_INLINE Matrix3x3 inverse() const;
    PHYS_FORCE_INLINE void invert();
    PHYS_FORCE_INLINE void setRotation(const Vector3 &axis, real angle);

    PHYS_FORCE_INLINE static const Matrix3x3& identity();
    PHYS_FORCE_INLINE static const Matrix3x3& zeros();
    PHYS_FORCE_INLINE static const Matrix3x3& ones();

    PHYS_FORCE_INLINE friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& mat);
};

#include "matrix3x3.inl"

} // namespace data_cg