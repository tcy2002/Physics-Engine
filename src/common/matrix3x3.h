#pragma once

#include <iostream>
#include <cmath>
#include "vector3.h"

namespace common {

    template <typename Scalar>
    class Matrix3x3 {
    private:
    protected:
        Scalar _m[3][3];

    public:
        Matrix3x3(): _m{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}} {}
        Matrix3x3(Scalar m00, Scalar m01, Scalar m02,
                  Scalar m10, Scalar m11, Scalar m12,
                  Scalar m20, Scalar m21, Scalar m22):
            _m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

        COMMON_FORCE_INLINE Scalar* operator[](int);
        COMMON_FORCE_INLINE const Scalar* operator[](int) const;
        COMMON_FORCE_INLINE Matrix3x3 operator-() const;
        COMMON_FORCE_INLINE Matrix3x3 operator+(const Matrix3x3&) const;
        COMMON_FORCE_INLINE Matrix3x3 operator-(const Matrix3x3&) const;
        COMMON_FORCE_INLINE Matrix3x3 operator*(Scalar) const;
        COMMON_FORCE_INLINE Matrix3x3 operator/(Scalar) const;
        COMMON_FORCE_INLINE Matrix3x3& operator+=(const Matrix3x3&);
        COMMON_FORCE_INLINE Matrix3x3& operator-=(const Matrix3x3&);
        COMMON_FORCE_INLINE Matrix3x3& operator*=(Scalar);
        COMMON_FORCE_INLINE Matrix3x3& operator/=(Scalar);
        COMMON_FORCE_INLINE Matrix3x3 operator*(const Matrix3x3&) const;
        COMMON_FORCE_INLINE Matrix3x3& operator*=(const Matrix3x3&);
        COMMON_FORCE_INLINE Vector3<Scalar> operator*(const Vector3<Scalar>&) const;

        COMMON_FORCE_INLINE Scalar determinant() const;
        COMMON_FORCE_INLINE Scalar trace() const;
        COMMON_FORCE_INLINE Matrix3x3 mult(const Matrix3x3&) const;
        COMMON_FORCE_INLINE Matrix3x3 transposed() const;
        COMMON_FORCE_INLINE void transpose();
        COMMON_FORCE_INLINE Matrix3x3 inverse() const;
        COMMON_FORCE_INLINE void invert();
        COMMON_FORCE_INLINE void setRotation(const Vector3<Scalar> &axis, Scalar angle);

        COMMON_FORCE_INLINE static const Matrix3x3& identity();
        COMMON_FORCE_INLINE static const Matrix3x3& zeros();
        COMMON_FORCE_INLINE static const Matrix3x3& ones();
    };

    template <typename Scalar>
    COMMON_FORCE_INLINE Matrix3x3<Scalar> operator*(Scalar, const Matrix3x3<Scalar>&);

    template <typename Scalar>
    COMMON_FORCE_INLINE Vector3<Scalar> operator*(const Vector3<Scalar>&, const Matrix3x3<Scalar>&);

    template <typename Scalar>
    COMMON_FORCE_INLINE std::ostream& operator<<(std::ostream& os, const Matrix3x3<Scalar>& mat);

#include "matrix3x3.cpp"

} // namespace common