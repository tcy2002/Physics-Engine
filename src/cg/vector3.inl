real& Vector3::operator[](int i) {
    return (&x)[i];
}

real Vector3::operator[](int i) const {
    return (&x)[i];
}

Vector3 Vector3::operator-() const {
    return {-x, -y, -z};
}

Vector3 Vector3::operator+(const Vector3& v) const {
    return {x + v.x, y + v.y, z + v.z};
}

Vector3 Vector3::operator-(const Vector3& v) const {
    return {x - v.x, y - v.y, z - v.z};
}

Vector3 Vector3::operator*(real r) const {
    return {x * r, y * r, z * r};
}

Vector3 Vector3::operator/(real r) const {
    return {x / r, y / r, z / r};
}

Vector3& Vector3::operator+=(const Vector3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vector3& Vector3::operator*=(real r) {
    x *= r;
    y *= r;
    z *= r;
    return *this;
}

Vector3& Vector3::operator/=(real r) {
    x /= r;
    y /= r;
    z /= r;
    return *this;
}

real Vector3::norm() const {
    return sqrt(x * x + y * y + z * z);
}

Vector3 Vector3::normalized() const {
    return *this / norm();
}

void Vector3::normalize() {
    *this /= norm();
}

real Vector3::dot(const Vector3& v) const {
    return x * v.x + y * v.y + z * v.z;
}

Vector3 Vector3::cross(const Vector3& v) const {
    return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
}

Vector3 Vector3::project(const Vector3& v) const {
    return v * (dot(v) / v.dot(v));
}

Vector3 Vector3::reflect(const Vector3& n) const {
    return n * (2 * dot(n) / n.dot(n)) - *this;
}

Vector3 Vector3::rotate(const Vector3& axis, real theta) const {
    auto n = axis.normalized();
    auto s = sin(theta), c = cos(theta);
    return *this * c + n.cross(*this) * s + n * dot(n) * (1 - c);
}

real Vector3::angle(const Vector3& other) const {
    return acos(dot(other) / (norm() * other.norm()));
}

const Vector3& Vector3::zeros() {
    static Vector3 zeros;
    return zeros;
}

const Vector3& Vector3::ones() {
    static Vector3 ones(1., 1., 1.);
    return ones;
}

const Vector3& Vector3::forward() {
    static Vector3 forward(0., 0., 1.);
    return forward;
}

const Vector3& Vector3::up() {
    static Vector3 up(0., 1., 0.);
    return up;
}

const Vector3& Vector3::right() {
    static Vector3 left(1., 0., 0.);
    return left;
}

std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << "[" << v.x << " " << v.y << " " << v.z << "]";
    return os;
}
