#pragma once

#include "vector3.h"
#include "matrix3x3.h"

namespace common {

    template <typename Scalar>
    class Quaternion {
    public:
        Scalar w, x, y, z;

        Quaternion(): w(1.), x(0.), y(0.), z(0.) {}
        Quaternion(Scalar w, Scalar x, Scalar y, Scalar z): w(w), x(x), y(y), z(z) {}

        COMMON_FORCE_INLINE Scalar& operator[](int);
        COMMON_FORCE_INLINE const Scalar& operator[](int) const;
        COMMON_FORCE_INLINE Quaternion operator-() const;
        COMMON_FORCE_INLINE Quaternion operator+(const Quaternion&) const;
        COMMON_FORCE_INLINE Quaternion operator-(const Quaternion&) const;
        COMMON_FORCE_INLINE Quaternion operator*(Scalar) const;
        COMMON_FORCE_INLINE Quaternion operator/(Scalar) const;
        COMMON_FORCE_INLINE Quaternion& operator+=(const Quaternion&);
        COMMON_FORCE_INLINE Quaternion& operator-=(const Quaternion&);
        COMMON_FORCE_INLINE Quaternion& operator*=(Scalar);
        COMMON_FORCE_INLINE Quaternion& operator/=(Scalar);
        COMMON_FORCE_INLINE Quaternion operator*(const Quaternion&) const;
        COMMON_FORCE_INLINE Quaternion& operator*=(const Quaternion&);

        COMMON_FORCE_INLINE void normalize();
        COMMON_FORCE_INLINE Quaternion normalized() const;
        COMMON_FORCE_INLINE Matrix3x3<Scalar> toRotationMatrix() const;
        COMMON_FORCE_INLINE static Quaternion fromRotationMatrix(const Matrix3x3<Scalar>& m);
        COMMON_FORCE_INLINE Vector3<Scalar> toEulerXYZ() const;
        COMMON_FORCE_INLINE static Quaternion fromEulerXYZ(const Vector3<Scalar>& v);

        COMMON_FORCE_INLINE static const Quaternion& identity();
        COMMON_FORCE_INLINE static const Quaternion& zeros();
    };

    #include "quaternion.cpp"

} // namespace common