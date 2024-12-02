template <typename Scalar>
VectorX<Scalar>::VectorX(Scalar value, size_t size): _data(new Scalar[size]), _size(size) {
    for (size_t i = 0; i < size; i++) {
        _data[i] = value;
    }
}

template <typename Scalar>
VectorX<Scalar>::VectorX(const common::Vector3<Scalar>& vec3): _data(new Scalar[3]), _size(3) {
    _data[0] = vec3.x;
    _data[1] = vec3.y;
    _data[2] = vec3.z;
}

template <typename Scalar>
VectorX<Scalar>::VectorX(const VectorX& other): _data(new Scalar[other._size]), _size(other._size) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] = other._data[i];
    }
}

template <typename Scalar>
VectorX<Scalar>::VectorX(VectorX&& other) noexcept : _data(other._data), _size(other._size) {
    _sub_ref = other._sub_ref;
    other._data = nullptr;
    other._size = 0;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator=(const VectorX& other) {
    if (this != &other) {
        if (_size != other._size) {
            if (_sub_ref) {
                throw std::invalid_argument("Cannot set sub-referenced vector with a different size");
            }
            if (_data != nullptr) {
                delete[] _data;
            }
            _data = new Scalar[other._size];
            _size = other._size;
        }
        for (size_t i = 0; i < _size; i++) {
            _data[i] = other._data[i];
        }
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator=(VectorX&& other) noexcept {
    if (this != &other) {
        if (_size != other._size) {
            if (_sub_ref) {
                throw std::invalid_argument("Cannot set sub-referenced vector with a different size");
            }
            if (_data != nullptr) {
                delete[] _data;
            }
            _data = other._data;
            _size = other._size;
            _sub_ref = other._sub_ref;
            other._data = nullptr;
            other._size = 0;
            return *this;
        }
        for (size_t i = 0; i < _size; i++) {
            _data[i] = other._data[i];
        }
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>::~VectorX() {
    _size = 0;
    if (_data == nullptr) {
        return;
    }
    if (!_sub_ref) {
        delete[] _data;
    }
    _data = nullptr;
}

template<typename Scalar>
void VectorX<Scalar>::resize(size_t size) {
    if (_sub_ref) {
        throw std::invalid_argument("Cannot resize sub-referenced vector");
    }
    if (size != _size) {
        delete[] _data;
        _data = new Scalar[size];
        _size = size;
    }
}

template<typename Scalar>
void VectorX<Scalar>::resize(size_t size, Scalar value) {
    if (_sub_ref) {
        throw std::invalid_argument("Cannot resize sub-referenced vector");
    }
    resize(size);
    for (size_t i = 0; i < size; i++) {
        _data[i] = value;
    }
}

template<typename Scalar>
void VectorX<Scalar>::setValue(Scalar v) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] = v;
    }
}


template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator-() const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = -_data[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator+(const VectorX& other) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] + other._data[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator-(const VectorX& other) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] - other._data[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator*(Scalar s) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] * s;
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::operator/(Scalar s) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] / s;
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator+=(const VectorX& other) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] += other._data[i];
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator-=(const VectorX& other) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] -= other._data[i];
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator*=(Scalar s) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] *= s;
    }
    return *this;
}

template <typename Scalar>
VectorX<Scalar>& VectorX<Scalar>::operator/=(Scalar s) {
    for (size_t i = 0; i < _size; i++) {
        _data[i] /= s;
    }
    return *this;
}

template <typename Scalar>
Scalar VectorX<Scalar>::norm() const {
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i] * _data[i];
    }
    return std::sqrt(sum);
}

template <typename Scalar>
Scalar VectorX<Scalar>::norm2() const {
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i] * _data[i];
    }
    return sum;
}

template <typename Scalar>
Scalar VectorX<Scalar>::mean() const {
    if (_size == 0) {
        return 0;
    }
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i];
    }
    return sum / _size;
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::normalized() const {
    Scalar n = norm();
    if (n == 0) {
        return *this;
    }
    return *this / n;
}

template <typename Scalar>
void VectorX<Scalar>::normalize() {
    Scalar n = norm();
    if (n == 0) {
        return;
    }
    *this /= n;
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::mult(const VectorX& v) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] * v[i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::div(const VectorX& v) const {
    VectorX result(_size);
    for (size_t i = 0; i < _size; i++) {
        result._data[i] = _data[i] / v[i];
    }
    return std::move(result);
}

template <typename Scalar>
Scalar VectorX<Scalar>::dot(const VectorX& v) const {
    Scalar sum = 0;
    for (size_t i = 0; i < _size; i++) {
        sum += _data[i] * v[i];
    }
    return sum;
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::getSubVector(size_t size, size_t start) const {
    if (start < 0 || start + size > _size || size <= 0) {
        throw std::invalid_argument("Sub-vector indices are out of bound");
    }
    VectorX result(size);
    for (size_t i = 0; i < size; i++) {
        result[i] = _data[start + i];
    }
    return std::move(result);
}

template <typename Scalar>
VectorX<Scalar> VectorX<Scalar>::getRefSubVector(size_t size, size_t start) {
    if (start < 0 || start + size > _size || size <= 0) {
        throw std::invalid_argument("Sub-vector indices are out of bound");
    }
    VectorX result(size, _data + start);
    result._sub_ref = true;
    return std::move(result);
}

template<typename Scalar>
void VectorX<Scalar>::setSubVector(const VectorX &vec, size_t start) {
    if (start < 0 || start + vec.size() > _size) {
        throw std::invalid_argument("Sub-vector indices are out of bound");
    }
    for (size_t i = 0; i < vec.size(); i++) {
        _data[start + i] = vec[i];
    }
}

template <typename Scalar>
common::Vector3<Scalar> VectorX<Scalar>::toVector3() const {
    if (_size != 3) {
        throw std::invalid_argument("Vector size is not 3");
    }
    return common::Vector3<Scalar>(_data[0], _data[1], _data[2]);
}

template <typename Scalar>
const VectorX<Scalar>& VectorX<Scalar>::zeros(size_t size) {
    static VectorX zeros(0, size);
    return zeros;
}

template <typename Scalar>
const VectorX<Scalar>& VectorX<Scalar>::ones(size_t size) {
    static VectorX ones(1, size);
    return ones;
}

template <typename Scalar>
VectorX<Scalar> operator*(Scalar s, const VectorX<Scalar>& v) {
    return v * s;
}

template <typename Scalar>
    std::ostream& operator<<(std::ostream& os, const VectorX<Scalar>& v) {
    os << "[";
    for (size_t i = 0; i < v.size(); i++) {
        os << v[i];
        if (i < v.size() - 1) {
            os << " ";
        }
    }
    os << "]";
    return os;
}
