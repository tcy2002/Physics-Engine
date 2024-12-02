template<typename Scalar>
Scalar** MatrixMN<Scalar>::getSubData(int rows, int cols, int start_row, int start_col) const {
    Scalar** data = new Scalar*[rows];
    for (size_t i = 0; i < rows; i++) {
        data[i] = _data[start_row + i] + start_col;
    }
    return data;
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(size_t M, size_t N): _rows(M), _cols(N) {
    _data = new Scalar*[M];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[N];
    }
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(size_t M, size_t N, Scalar** data): _rows(M), _cols(N) {
    _data = new Scalar*[M];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[N];
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] = data[i][j];
        }
    }
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(Scalar value, size_t M, size_t N): _rows(M), _cols(N) {
    _data = new Scalar*[M];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[N];
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] = value;
        }
    }
}

template <typename Scalar>
MatrixMN<Scalar>::MatrixMN(const common::Matrix3x3<Scalar>& mat3x3): _rows(3), _cols(3) {
    _data = new Scalar*[3];
    for (size_t i = 0; i < 3; i++) {
        _data[i] = new Scalar[3];
        for (size_t j = 0; j < 3; j++) {
            _data[i][j] = mat3x3[i][j];
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
    other._cols = other._rows = 0;
}

template <typename Scalar>
MatrixMN<Scalar>& MatrixMN<Scalar>::operator=(const MatrixMN& other) {
    if (this != &other) {
        if (_rows != other._rows || _cols != other._cols) {
            if (_sub_ref) {
                throw std::invalid_argument("Cannot set sub-referenced matrix with a different size");
            }
            if (_data != nullptr) {
                for (size_t i = 0; i < _rows; i++) {
                    delete[] _data[i];
                }
                delete[] _data;
            }
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
            if (_sub_ref) {
                throw std::invalid_argument("Cannot set sub-referenced matrix with a different size");
            }
            if (_data != nullptr) {
                for (size_t i = 0; i < _rows; i++) {
                    delete[] _data[i];
                }
                delete[] _data;
            }
            _rows = other._rows;
            _cols = other._cols;
            _data = other._data;
            other._data = nullptr;
            other._cols = other._rows = 0;
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
    _rows = _cols = 0;
    if (_data == nullptr) {
        return;
    }
    if (!_sub_ref) {
        for (size_t i = 0; i < _rows; i++) {
            delete[] _data[i];
        }
    }
    delete[] _data;
    _data = nullptr;
}

template<typename Scalar>
void MatrixMN<Scalar>::resize(size_t M, size_t N) {
    if (_sub_ref) {
        throw std::invalid_argument("Cannot resize sub-referenced matrix");
    }
    if (M == _rows && N == _cols) {
        return;
    }
    if (_data != nullptr) {
        for (size_t i = 0; i < _rows; i++) {
            delete[] _data[i];
        }
        delete[] _data;
    }
    _rows = M;
    _cols = N;
    _data = new Scalar*[_rows];
    for (size_t i = 0; i < _rows; i++) {
        _data[i] = new Scalar[_cols];
    }
}

template<typename Scalar>
void MatrixMN<Scalar>::resize(size_t M, size_t N, Scalar value) {
    if (_sub_ref) {
        throw std::invalid_argument("Cannot resize sub-referenced matrix");
    }
    resize(M, N);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] = value;
        }
    }
}

template<typename Scalar>
void MatrixMN<Scalar>::setValue(Scalar v) {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            _data[i][j] = v;
        }
    }
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
    MatrixMN res(0, _rows, other._cols);
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
    VectorX<Scalar> res(0, _rows);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res[i] += _data[i][j] * vec[j];
        }
    }
    return std::move(res);
}

template <typename Scalar>
void MatrixMN<Scalar>::transpose() {
    if (_rows == _cols) {
        for (size_t i = 0; i < _rows; i++) {
            for (size_t j = i + 1; j < _cols; j++) {
                std::swap(_data[i][j], _data[j][i]);
            }
        }
    } else {
        MatrixMN res(_cols, _rows);
        for (size_t i = 0; i < _rows; i++) {
            for (size_t j = 0; j < _cols; j++) {
                res._data[j][i] = _data[i][j];
            }
        }
        *this = std::move(res);
    }
}

template <typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::transposed() const {
    MatrixMN res(_cols, _rows);
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            res._data[j][i] = _data[i][j];
        }
    }
    return std::move(res);
}

template<typename Scalar>
VectorX<Scalar> MatrixMN<Scalar>::getColumn(size_t index) const {
    if (index < 0 || index >= _cols) {
        throw std::invalid_argument("Column index is out of bound");
    }
    VectorX<Scalar> res(_rows);
    for (size_t i = 0; i < _rows; i++) {
        res[i] = _data[i][index];
    }
    return std::move(res);
}

template<typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::getSubMatrix(size_t rows, size_t cols, size_t start_row, size_t start_col) const {
    if (start_row < 0 || start_row + rows > _rows || start_col < 0 || start_col + cols > _cols) {
        throw std::invalid_argument("Sub-matrix indices are out of bound");
    }
    MatrixMN res(rows, cols);
    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            res._data[i][j] = _data[start_row + i][start_col + j];
        }
    }
    return std::move(res);
}

template<typename Scalar>
MatrixMN<Scalar> MatrixMN<Scalar>::getRefSubMatrix(size_t rows, size_t cols, size_t start_row, size_t start_col) {
    if (start_row < 0 || start_row + rows > _rows || start_col < 0 || start_col + cols > _cols) {
        throw std::invalid_argument("Sub-matrix indices are out of bound");
    }
    MatrixMN res(rows, cols, getSubData(rows, cols, start_row, start_col));
    res._sub_ref = true;
    return std::move(res);
}

template<typename Scalar>
VectorX<Scalar> MatrixMN<Scalar>::getSubRowVector(size_t cols, size_t start_row, size_t start_col) const {
    if (start_row < 0 || start_row >= _rows || start_col < 0 || start_col + cols > _cols) {
        throw std::invalid_argument("Sub-vector indices are out of bound");
    }
    VectorX<Scalar> res(cols);
    for (size_t i = 0; i < cols; i++) {
        res[i] = _data[start_row][start_col + i];
    }
    return std::move(res);
}

template<typename Scalar>
VectorX<Scalar> MatrixMN<Scalar>::getRefSubRowVector(size_t cols, size_t start_row, size_t start_col) {
    if (start_row < 0 || start_row >= _rows || start_col < 0 || start_col + cols > _cols) {
        throw std::invalid_argument("Sub-vector indices are out of bound");
    }
    VectorX<Scalar> res(cols, _data[start_row] + start_col);
    res._sub_ref = true;
    return std::move(res);
}

template<typename Scalar>
void MatrixMN<Scalar>::setSubMatrix(const MatrixMN& mat, size_t start_row, size_t start_col) {
    if (start_row < 0 || start_row + mat._rows > _rows || start_col < 0 || start_col + mat._cols > _cols) {
        throw std::invalid_argument("Sub-matrix indices are out of bound");
    }
    for (size_t i = 0; i < mat._rows; i++) {
        for (size_t j = 0; j < mat._cols; j++) {
            _data[start_row + i][start_col + j] = mat._data[i][j];
        }
    }
}

template<typename Scalar>
void MatrixMN<Scalar>::setSubVector(const VectorX<Scalar>& vec, size_t start_row, size_t start_col, bool in_row) {
    if (start_row < 0 || start_row + (in_row ? 1 : vec.size()) > _rows || start_col < 0 || start_col + (in_row ? vec.size() : 1) > _cols) {
        throw std::invalid_argument("Sub-matrix indices are out of bound");
    }
    for (size_t i = 0; i < vec.size(); i++) {
        if (in_row) {
            _data[start_row][start_col + i] = vec[i];
        } else {
            _data[start_row + i][start_col] = vec[i];
        }
    }
}

template<typename Scalar>
common::Matrix3x3<Scalar> MatrixMN<Scalar>::toMatrix3x3() const {
    if (_rows != 3 || _cols != 3) {
        throw std::invalid_argument("Matrix dimensions are not 3x3");
    }
    common::Matrix3x3<Scalar> res;
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            res[i][j] = _data[i][j];
        }
    }
    return res;
}

template <typename Scalar>
const MatrixMN<Scalar>& MatrixMN<Scalar>::identity(size_t size) {
    static MatrixMN res(0, size, size);
    static bool initialized = false;
    if (!initialized) {
        for (size_t i = 0; i < size; i++) {
            res._data[i][i] = 1;
        }
        initialized = true;
    }
    return res;
}

template <typename Scalar>
const MatrixMN<Scalar>& MatrixMN<Scalar>::zeros(size_t M, size_t N) {
    static MatrixMN res(0, M, N);
    return res;
}

template <typename Scalar>
const MatrixMN<Scalar>& MatrixMN<Scalar>::ones(size_t M, size_t N) {
    static MatrixMN res(1, M, N);
    return res;
}

template <typename Scalar>
MatrixMN<Scalar> operator*(Scalar s, const MatrixMN<Scalar>& v) {
    return v * s;
}

template <typename Scalar>
VectorX<Scalar> operator*(const VectorX<Scalar>& v, const MatrixMN<Scalar>& mat) {
    if (v.size() != mat.rows()) {
        throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
    }
    VectorX<Scalar> res(0, mat.cols());
    for (size_t i = 0; i < mat.cols(); i++) {
        for (size_t j = 0; j < mat.rows(); j++) {
            res[i] += v[j] * mat[j][i];
        }
    }
    return std::move(res);
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
