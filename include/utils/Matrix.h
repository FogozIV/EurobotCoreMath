//
// Created by fogoz on 16/05/2025.
//

#ifndef MATRIX_H
#define MATRIX_H
#include <array>
#include <math.h>

#ifndef ARDUINO
#include <iostream>
#include <iomanip>
#endif

template<size_t rows, size_t cols>
#ifndef ARDUINO
class Matrix {
#else
class Matrix : public Printable{
#endif
    std::array<std::array<double, cols>, rows> data;
public:
    const size_t _rows = rows;
    const size_t _cols = cols;
#ifndef ARDUINO
    friend constexpr std::ostream& operator<<(std::ostream& os, const Matrix<rows, cols>& matrix) {
        for (size_t i = 0; i < rows; ++i) {
            os << (i == 0 ? "[" : " ");  // Opening bracket for first row
            for (size_t j = 0; j < cols; ++j) {
                os << std::setw(10) << matrix.data[i][j];  // Fixed-width formatting
                if (j < cols - 1) os << ", ";
            }
            os << (i == rows - 1 ? "]" : "\n");  // Closing bracket for last row
        }
        return os;
    }
#endif
    constexpr Matrix(std::array<std::array<double, cols>, rows> parameters) : data(parameters) {
    }
    constexpr Matrix() {
        for (auto& row : data) {
            for (auto& col : row) {
                col = 0;
            }
        }

    }
    constexpr Matrix(const Matrix& other) = default;

    static constexpr Matrix identity() {
        Matrix m;
        size_t smaller = std::min(rows, cols);
        for (size_t i = 0; i < smaller; i++) {
            m.data[i][i] = 1;
        }
        return m;
    }
    static constexpr Matrix zero() {
        return Matrix();
    }

    constexpr double& operator()(size_t row, size_t col) {
        return data[row][col];
    }

    constexpr double operator()(size_t row, size_t col) const {
        return data[row][col];
    }

    constexpr Matrix<cols, rows> transpose() const{
        Matrix<cols, rows>  m;
        for (size_t i = 0; i < rows; i++) {
            for (size_t j = 0; j < cols; j++) {
                m(j, i) = data[i][j];
            }
        }
        return m;
    }
    constexpr Matrix operator*(double scalar) const {
        Matrix result;
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                result(i, j) = data[i][j] * scalar;
        return result;
    }
    constexpr std::optional<std::pair<Matrix<rows, cols>, Matrix<rows, cols>>> luDecompose() const {
        if (rows != cols) {
            return std::nullopt; // LU decomposition only works for square matrices
        }

        Matrix<rows, cols> L, U;

        // Initialize L to identity matrix and U to a copy of the matrix
        for (size_t i = 0; i < rows; ++i) {
            L(i, i) = 1;  // L is the identity matrix initially
            for (size_t j = 0; j < rows; ++j) {
                U(i, j) = data[i][j];  // U starts as a copy of the matrix
            }
        }

        for (size_t i = 0; i < rows; ++i) {
            // Perform LU Decomposition
            for (size_t j = i + 1; j < rows; ++j) {
                if (U(i, i) == 0) {
                    return std::nullopt; // Singular matrix, no LU decomposition possible
                }

                double factor = U(j, i) / U(i, i);
                L(j, i) = factor;

                // Update U matrix (eliminate lower part)
                for (size_t k = i; k < rows; ++k) {
                    U(j, k) -= factor * U(i, k);
                }
            }
        }

        return std::make_pair(L, U);
    }

    constexpr Matrix<rows, cols>& operator*=(const Matrix<cols, rows>& rhs) {
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                double sum = 0;
                for (size_t k = 0; k < cols; ++k) {
                    sum += data[i][k] * rhs(k, j);
                }
                data[i][j] = sum;
            }
        }
        return *this;
    }

    constexpr Matrix<rows, cols>& operator*=(double d) {
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                data[i][j] *= d;
            }
        }
        return *this;
    }

    // Inverse using LU Decomposition
    constexpr std::optional<Matrix<rows, cols>> inverse() const {
        if (rows != cols) {
            return std::nullopt; // Only square matrices have an inverse
        }

        auto lu = luDecompose();
        if (!lu.has_value()) {
            return std::nullopt; // Singular matrix (no inverse)
        }

        Matrix<rows, cols> L = lu->first, U = lu->second;

        // Use forward substitution to solve L * Y = I
        Matrix<rows, cols> Y;
        for (size_t j = 0; j < cols; ++j) {
            for (size_t i = 0; i < rows; ++i) {
                double sum = 0;
                for (size_t k = 0; k < i; ++k) {
                    sum += L(i, k) * Y(k, j);
                }
                Y(i, j) = (i == j ? 1.0 : 0.0) - sum;  // Handle identity matrix
            }
        }

        // Use backward substitution to solve U * X = Y
        Matrix<rows, cols> X;
        for (size_t j = 0; j < cols; ++j) {
            for (size_t i = rows - 1; i < rows; --i) {  // Note: `size_t` underflow hack
                double sum = 0;
                for (size_t k = i + 1; k < rows; ++k) {
                    sum += U(i, k) * X(k, j);
                }
                X(i, j) = (Y(i, j) - sum) / U(i, i);
            }
        }

        return X;
    }

    constexpr Matrix operator+(const Matrix & matrix) const {
        Matrix result;
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result(i, j) = data[i][j] + matrix(i, j);
            }
        }
        return result;
    }

    constexpr Matrix& operator+=(const Matrix & rhs) {
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                data[i][j] += rhs.data[i][j];
            }
        }
        return *this;
    }

    constexpr Matrix operator-(const Matrix & matrix) const {
        Matrix result;
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result(i, j) = data[i][j] - matrix(i, j);
            }
        }
        return result;
    }

    constexpr double norm() {
        double sum = 0;
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                sum += data[i][j] * data[i][j];
            }
        }
        return sqrt(sum);
    }

    constexpr double det() {
        if (rows != cols) {
            return 0; // Only square matrices have a determinant
        }
        double result = 1;
        for (size_t i = 0; i < rows; ++i) {
            result *= data[i][i];
        }
        return result;
    }
    constexpr Matrix& operator=(const Matrix& other) {
        if (this != &other) {
            data = other.data;
        }
        return *this;
    }

    constexpr Matrix& operator=(Matrix&& other) noexcept {
        if (this != &other) {
            data = std::move(other.data);
        }
        return *this;
    }

    constexpr void clear() {
        for (auto& row : data) {
            for (auto& col : row) {
                col = 0;
            }
        }
    }
#ifdef ARDUINO
    size_t printTo(Print &p) const override {
        size_t n = 0;
        for (size_t i = 0; i < rows; ++i) {
            n += p.print(i == 0 ? "[" : " ");  // Opening bracket for first row
            for (size_t j = 0; j < cols; ++j) {
                n += p.print(data[i][j]);  // Fixed-width formatting
                if (j < cols - 1) n+=p.print(", ");
            }
            n += p.print(i == rows - 1 ? "]" : "\n");  // Closing bracket for last row
        }
        return n;
    }
#endif
};

template<size_t R1, size_t C1, size_t C2>
constexpr Matrix<R1, C2> operator*(const Matrix<R1, C1>& lhs, const Matrix<C1, C2>& rhs) {
    Matrix<R1, C2> result;

    for (size_t i = 0; i < R1; ++i) {
        for (size_t j = 0; j < C2; ++j) {
            double sum = 0;
            for (size_t k = 0; k < C1; ++k) {
                sum += lhs(i, k) * rhs(k, j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

template<size_t R, size_t C>
constexpr Matrix<R, C> operator*(double scalar, const Matrix<R, C>& mat) {
    return mat * scalar;
}

#ifndef __cpp_constexpr_math
inline Matrix<2,2> transformationMatrix(double theta) {
    return Matrix<2,2>({
        std::array{std::cos(theta), -std::sin(theta)},
        {std::sin(theta),  std::cos(theta)}
    });
}
#else
constexpr Matrix<2,2> transformationMatrix(double theta) {
    return Matrix<2,2>({
        std::array<double, 2>({cos(theta), -sin(theta)}),
        {sin(theta), cos(theta)}
    });
}
#endif

#endif //MATRIX_H
