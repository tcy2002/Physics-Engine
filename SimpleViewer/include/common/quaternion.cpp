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
        q.w = Scalar(0.25) * s;
        q.x = (m[2][1] - m[1][2]) / s;
        q.y = (m[0][2] - m[2][0]) / s;
        q.z = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        Scalar s = std::sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2;
        q.w = (m[2][1] - m[1][2]) / s;
        q.x = Scalar(0.25) * s;
        q.y = (m[0][1] + m[1][0]) / s;
        q.z = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        Scalar s = std::sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2;
        q.w = (m[0][2] - m[2][0]) / s;
        q.x = (m[0][1] + m[1][0]) / s;
        q.y = Scalar(0.25) * s;
        q.z = (m[1][2] + m[2][1]) / s;
    } else {
        Scalar s = std::sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2;
        q.w = (m[1][0] - m[0][1]) / s;
        q.x = (m[0][2] + m[2][0]) / s;
        q.y = (m[1][2] + m[2][1]) / s;
        q.z = Scalar(0.25) * s;
    }
    return q;
}

template <typename Scalar>
Vector3<Scalar> Quaternion<Scalar>::toEulerXYZ() const {
    Vector3<Scalar> v;
    Scalar sin_r_cos_p = 2 * (w * x + y * z);
    Scalar cos_r_cos_p = 1 - 2 * (x * x + y * y);
    v.x = std::atan2(sin_r_cos_p, cos_r_cos_p);

    Scalar sin_p = 2 * (w * y - z * x);
    if (std::abs(sin_p) >= 1) {
        v.y = std::copysign(1.570796, sin_p);
    } else {
        v.y = std::asin(sin_p);
    }

    Scalar sin_y_cos_p = 2 * (w * z + x * y);
    Scalar cos_y_cos_p = 1 - 2 * (y * y + z * z);
    v.z = std::atan2(sin_y_cos_p, cos_y_cos_p);

    v.x = v.x / Scalar(3.14159265) * Scalar(180);
    v.y = v.y / Scalar(3.14159265) * Scalar(180);
    v.z = v.z / Scalar(3.14159265) * Scalar(180);
    return v;
}

template <typename Scalar>
Quaternion<Scalar> Quaternion<Scalar>::fromEulerXYZ(const Vector3<Scalar>& v) {
    Scalar x = v.x / Scalar(180) * Scalar(3.14159265);
    Scalar y = v.y / Scalar(180) * Scalar(3.14159265);
    Scalar z = v.z / Scalar(180) * Scalar(3.14159265);

    Scalar cy = std::cos(v.z * 0.5);
    Scalar sy = std::sin(v.z * 0.5);
    Scalar cp = std::cos(v.y * 0.5);
    Scalar sp = std::sin(v.y * 0.5);
    Scalar cr = std::cos(v.x * 0.5);
    Scalar sr = std::sin(v.x * 0.5);

    Quaternion<Scalar> q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
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