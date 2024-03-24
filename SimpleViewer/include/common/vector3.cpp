template <typename Scalar>
Scalar& Vector3<Scalar>::operator[](int i) {
    return (&x)[i];
}

template <typename Scalar>
const Scalar& Vector3<Scalar>::operator[](int i) const {
    return (&x)[i];
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::operator-() const {
    return {-x, -y, -z};
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::operator+(const Vector3<Scalar>& v) const {
    return {x + v.x, y + v.y, z + v.z};
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::operator-(const Vector3<Scalar>& v) const {
    return {x - v.x, y - v.y, z - v.z};
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::operator*(Scalar r) const {
    return {x * r, y * r, z * r};
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::operator/(Scalar r) const {
    return {x / r, y / r, z / r};
}

template <typename Scalar>
Vector3<Scalar>& Vector3<Scalar>::operator+=(const Vector3<Scalar>& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

template <typename Scalar>
Vector3<Scalar>& Vector3<Scalar>::operator-=(const Vector3<Scalar>& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

template <typename Scalar>
Vector3<Scalar>& Vector3<Scalar>::operator*=(Scalar r) {
    x *= r;
    y *= r;
    z *= r;
    return *this;
}

template <typename Scalar>
Vector3<Scalar>& Vector3<Scalar>::operator/=(Scalar r) {
    x /= r;
    y /= r;
    z /= r;
    return *this;
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::operator*(const Vector3<Scalar>& v) const {
    return {x * v.x, y * v.y, z * v.z};
}

template <typename Scalar>
Scalar Vector3<Scalar>::norm() const {
    return sqrt(x * x + y * y + z * z);
}

template <typename Scalar>
Scalar Vector3<Scalar>::norm2() const {
    return x * x + y * y + z * z;
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::normalized() const {
    return *this / norm();
}

template <typename Scalar>
void Vector3<Scalar>::normalize() {
    *this /= norm();
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::mult(const Vector3<Scalar>& v) const {
    return {x * v.x, y * v.y, z * v.z};
}

template <typename Scalar>
Scalar Vector3<Scalar>::dot(const Vector3<Scalar>& v) const {
    return x * v.x + y * v.y + z * v.z;
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::cross(const Vector3<Scalar>& v) const {
    return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::lerp(const Vector3<Scalar>& v, Scalar t) const {
    return *this + (v - *this) * t;
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::projectToVec(const Vector3<Scalar>& v) const {
    return v * (dot(v) / v.dot(v));
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::projectToPlane(const Vector3<Scalar>& v) const {
    return *this - projectToVec(v);
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::reflect(const Vector3<Scalar>& n) const {
    return n * (2 * dot(n) / n.dot(n)) - *this;
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::rotate(const Vector3<Scalar>& axis, Scalar theta) const {
    auto n = axis.normalized();
    auto s = sin(theta), c = cos(theta);
    return *this * c + n.cross(*this) * s + n * dot(n) * (1 - c);
}

template <typename Scalar>
Scalar Vector3<Scalar>::angle(const Vector3<Scalar>& other) const {
    return acos(dot(other) / (norm() * other.norm()));
}

template <typename Scalar>
Vector3<Scalar> Vector3<Scalar>::getAbsolute() const {
    return {std::abs(x), std::abs(y), std::abs(z)};
}

template <typename Scalar>
const Vector3<Scalar>& Vector3<Scalar>::zeros() {
    static Vector3<Scalar> zeros;
    return zeros;
}

template <typename Scalar>
const Vector3<Scalar>& Vector3<Scalar>::ones() {
    static Vector3<Scalar> ones(1., 1., 1.);
    return ones;
}

template <typename Scalar>
const Vector3<Scalar>& Vector3<Scalar>::forward() {
    static Vector3<Scalar> forward(0., 0., 1.);
    return forward;
}

template <typename Scalar>
const Vector3<Scalar>& Vector3<Scalar>::up() {
    static Vector3<Scalar> up(0., 1., 0.);
    return up;
}

template <typename Scalar>
const Vector3<Scalar>& Vector3<Scalar>::right() {
    static Vector3<Scalar> left(1., 0., 0.);
    return left;
}

template <typename Scalar>
Vector3<Scalar> operator*(Scalar r, const Vector3<Scalar>& v) {
    return v * r;
}

template <typename Scalar>
std::ostream& operator<<(std::ostream& os, const Vector3<Scalar>& v) {
    os << "[" << v.x << " " << v.y << " " << v.z << "]";
    return os;
}
