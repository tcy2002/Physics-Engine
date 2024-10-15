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

        COMMON_FORCE_INLINE static const Quaternion& identity();
        COMMON_FORCE_INLINE static const Quaternion& zeros();
    };

    template <typename Scalar>
    Scalar& Quaternion<Scalar>::operator[](int i) {
        return (&w)[i];
    }

    template <typename Scalar>
    const Scalar& Quaternion<Scalar>::operator[](int i) const {
        return (&w)[i];
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::operator-() const {
        return {-w, -x, -y, -z};
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::operator+(const Quaternion<Scalar>& q) const {
        return {w + q.w, x + q.x, y + q.y, z + q.z};
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::operator-(const Quaternion<Scalar>& q) const {
        return {w - q.w, x - q.x, y - q.y, z - q.z};
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::operator*(Scalar r) const {
        return {w * r, x * r, y * r, z * r};
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::operator/(Scalar r) const {
        return {w / r, x / r, y / r, z / r};
    }

    template <typename Scalar>
    Quaternion<Scalar>& Quaternion<Scalar>::operator+=(const Quaternion<Scalar>& q) {
        w += q.w;
        x += q.x;
        y += q.y;
        z += q.z;
        return *this;
    }

    template <typename Scalar>
    Quaternion<Scalar>& Quaternion<Scalar>::operator-=(const Quaternion<Scalar>& q) {
        w -= q.w;
        x -= q.x;
        y -= q.y;
        z -= q.z;
        return *this;
    }

    template <typename Scalar>
    Quaternion<Scalar>& Quaternion<Scalar>::operator*=(Scalar r) {
        w *= r;
        x *= r;
        y *= r;
        z *= r;
        return *this;
    }

    template <typename Scalar>
    Quaternion<Scalar>& Quaternion<Scalar>::operator/=(Scalar r) {
        w /= r;
        x /= r;
        y /= r;
        z /= r;
        return *this;
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::operator*(const Quaternion<Scalar>& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }

    template <typename Scalar>
    Quaternion<Scalar>& Quaternion<Scalar>::operator*=(const Quaternion<Scalar>& q) {
        *this = *this * q;
        return *this;
    }

    template <typename Scalar>
    void Quaternion<Scalar>::normalize() {
        Scalar n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n != 0) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::normalized() const {
        Scalar n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n != 0) {
            return {w / n, x / n, y / n, z / n};
        }
        return {1., 0., 0., 0.};
    }

    template <typename Scalar>
    Matrix3x3<Scalar> Quaternion<Scalar>::toRotationMatrix() const {
        Scalar xx = x * x;
        Scalar xy = x * y;
        Scalar xz = x * z;
        Scalar xw = x * w;
        Scalar yy = y * y;
        Scalar yz = y * z;
        Scalar yw = y * w;
        Scalar zz = z * z;
        Scalar zw = z * w;

        Scalar m00 = 1 - 2 * (yy + zz);
        Scalar m01 = 2 * (xy - zw);
        Scalar m02 = 2 * (xz + yw);
        Scalar m10 = 2 * (xy + zw);
        Scalar m11 = 1 - 2 * (xx + zz);
        Scalar m12 = 2 * (yz - xw);
        Scalar m20 = 2 * (xz - yw);
        Scalar m21 = 2 * (yz + xw);
        Scalar m22 = 1 - 2 * (xx + yy);

        return {m00, m01, m02, m10, m11, m12, m20, m21, m22};
    }

    template <typename Scalar>
    Quaternion<Scalar> Quaternion<Scalar>::fromRotationMatrix(const Matrix3x3<Scalar>& m) {
        Quaternion<Scalar> q;
        Scalar t = m.trace();
        if (t > 0) {
            Scalar s = std::sqrt(t + 1) * 2;
            q.w = 0.25 * s;
            q.x = (m[2][1] - m[1][2]) / s;
            q.y = (m[0][2] - m[2][0]) / s;
            q.z = (m[1][0] - m[0][1]) / s;
        } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
            Scalar s = std::sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2;
            q.w = (m[2][1] - m[1][2]) / s;
            q.x = 0.25 * s;
            q.y = (m[0][1] + m[1][0]) / s;
            q.z = (m[0][2] + m[2][0]) / s;
        } else if (m[1][1] > m[2][2]) {
            Scalar s = std::sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2;
            q.w = (m[0][2] - m[2][0]) / s;
            q.x = (m[0][1] + m[1][0]) / s;
            q.y = 0.25 * s;
            q.z = (m[1][2] + m[2][1]) / s;
        } else {
            Scalar s = std::sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2;
            q.w = (m[1][0] - m[0][1]) / s;
            q.x = (m[0][2] + m[2][0]) / s;
            q.y = (m[1][2] + m[2][1]) / s;
            q.z = 0.25 * s;
        }
        return q;
    }

    template <typename Scalar>
    const Quaternion<Scalar>& Quaternion<Scalar>::identity() {
        static const Quaternion<Scalar> q(1., 0., 0., 0.);
        return q;
    }

    template <typename Scalar>
    const Quaternion<Scalar>& Quaternion<Scalar>::zeros() {
        static const Quaternion<Scalar> q(0., 0., 0., 0.);
        return q;
    }

} // namespace common