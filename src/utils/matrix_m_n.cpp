template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(size_t M, size_t N): _rows(M), _cols(N) {
    _data = new Scalar*[M];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[N];
    }
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(size_t M, size_t N, Scalar value): _rows(M), _cols(N) {
    _data = new Scalar*[M];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[N];
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] = value;
        }
    }
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(const MatrixMN& other): _rows(other._rows), _cols(other._cols) {
    _data = new Scalar*[_rows];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[_cols];
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] = other._data[i][j];
        }
    }
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(MatrixMN&& other) noexcept: _data(other._data), _rows(other._rows), _cols(other._cols) {
    other._data = nullptr;
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator=(const MatrixMN& other) {
    if (this != &other) {
        if (_rows != other._rows || _cols != other._cols) {
            for (size_t i = 0; i < _rows; i++) {
                delete[] _data[i];
            }
            delete[] _data;
            _rows = other._rows;
            _cols = other._cols;
            _data = new Scalar*[_rows];
            for (size_t i = 0; i < _rows; i++) {
                _data[i] = new Scalar[_cols];
            }
        }
        for (size_t i = 0; i < _rows; i++) {
            for (size_t j = 0; j < _cols; j++) {
                _data[i][j] = other._data[i][j];
            }
        }
    }
    return *this;
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator=(MatrixMN&& other) noexcept {
    if (this != &other) {
        if (_rows != other._rows || _cols != other._cols) {
            for (size_t i = 0; i < _rows; i++) {
                delete[] _data[i];
            }
            delete[] _data;
            _rows = other._rows;
            _cols = other._cols;
            _data = other._data;
            other._data = nullptr;
            return *this;
        }
        for (size_t i = 0; i < _rows; i++) {
            for (size_t j = 0; j < _cols; j++) {
                _data[i][j] = other._data[i][j];
            }
        }
    }
    return *this;
}

template <typename Scalar>
MatrixMN<Scalar>::~MatrixMN() {
    if (_data == nullptr) {
        return;
    }
    for (size_t i = 0; i < _rows; i++) {
        delete[] _data[i];
    }
    delete[] _data;
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::operator-() const {
    MatrixMN res(_rows, _cols);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res._data[i][j] = -_data[i][j];
        }
    }
    return std::move(res);
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::operator+(const MatrixMN& other) const {
    MatrixMN res(_rows, _cols);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res._data[i][j] = _data[i][j] + other._data[i][j];
        }
    }
    return std::move(res);
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::operator-(const MatrixMN& other) const {
    MatrixMN res(_rows, _cols);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res._data[i][j] = _data[i][j] - other._data[i][j];
        }
    }
    return std::move(res);
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::operator*(Scalar s) const {
    MatrixMN res(_rows, _cols);
    for (size_t i = 0; i < _cols; i++) {
        for (size_t j = 0; j < _rows; j++) {
            res._data[i][j] = _data[i][j] * s;
        }
    }
    return std::move(res);
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::operator/(Scalar s) const {
    MatrixMN res(_rows, _cols);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res._data[i][j] = _data[i][j] / s;
        }
    }
    return std::move(res);
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator+=(const MatrixMN& other) {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] += other._data[i][j];
        }
    }
    return *this;
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator-=(const MatrixMN& other) {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] -= other._data[i][j];
        }
    }
    return *this;
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator*=(Scalar s) {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] *= s;
        }
    }
    return *this;
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator/=(Scalar s) {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] /= s;
        }
    }
    return *this;
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::operator*(const MatrixMN& other) const {
    if (_cols != other._rows) {
        throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
    }
    MatrixMN res(_rows, other._cols, 0);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < other._cols; j++) {
            for (size_t k = 0; k < _cols; k++) {
                res._data[i][j] += _data[i][k] * other._data[k][j];
            }
        }
    }
    return std::move(res);
}

template <typename Scalar>
VectorX<Scalar> MatrixMN<Scalar>::operator*(const VectorX<Scalar>& vec) const {
    if (_cols != vec.size()) {
        throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
    }
    VectorX<Scalar> res(_rows, 0);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res[i] += _data[i][j] * vec[j];
        }
    }
    return std::move(res);
}

template <typename Scalar>
const MatrixMN<Scalar>& MatrixMN<Scalar>::identity(size_t size) {
    static MatrixMN res(size, size, 0);
    static bool initialized = false;
    if (!initialized) {
        for (size_t i = 0; i < size; i++) {
            res[i][i] = 1;
        }
        initialized = true;
    }
    return res;
}

template <typename Scalar>
const MatrixMN<Scalar>& MatrixMN<Scalar>::zeros(size_t M, size_t N) {
    static MatrixMN res(M, N, 0);
    return res;
}

template <typename Scalar>
const MatrixMN<Scalar>& MatrixMN<Scalar>::ones(size_t M, size_t N) {
    static MatrixMN res(M, N, 1);
    return res;
}

template <typename Scalar>
    std::ostream& operator<<(std::ostream& os, const MatrixMN<Scalar>& mat) {
    os << "[";
    for (size_t i = 0; i < mat.rows(); i++) {
        if (i != 0) {
            os << " ";
        }
        for (size_t j = 0; j < mat.cols(); j++) {
            os << mat[i][j];
            if (j < mat.cols() - 1) {
                os << "\t";
            }
        }
        if (i < mat.rows() - 1) {
            os << "\n";
        }
    }
    os << "]";
    return os;
}
