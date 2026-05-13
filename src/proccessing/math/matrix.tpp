/**
 * @file matrix.tpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Template implementation for matrix.hpp.
 *
 * @date 2026-05-13
 */

#ifndef MATRIX_TPP
#define MATRIX_TPP

#ifndef MATRIX_HPP
#include "proccessing/math/matrix.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace sf::math
{

namespace
{

inline std::string matrixIndexError(size_t row, size_t col, size_t rows,
                                    size_t cols)
{
    return "Matrix index (" + std::to_string(row) + ", " + std::to_string(col) +
           ") out of range for matrix with shape (" + std::to_string(rows) +
           ", " + std::to_string(cols) + ")";
}

} // namespace

template <typename T>
Matrix<T>::Matrix(size_t rows, size_t cols, T initial)
    : rows_(rows), cols_(cols), data_()
{
    if (rows != 0 && cols > std::numeric_limits<size_t>::max() / rows)
    {
        throw std::overflow_error("Matrix dimensions overflow storage size");
    }

    data_.assign(rows * cols, initial);
}

template <typename T> T &Matrix<T>::operator()(size_t row, size_t col)
{
    if (row >= rows_ || col >= cols_)
    {
        throw std::out_of_range(matrixIndexError(row, col, rows_, cols_));
    }

    return data_[row * cols_ + col];
}

template <typename T>
const T &Matrix<T>::operator()(size_t row, size_t col) const
{
    if (row >= rows_ || col >= cols_)
    {
        throw std::out_of_range(matrixIndexError(row, col, rows_, cols_));
    }

    return data_[row * cols_ + col];
}

template <typename T> size_t Matrix<T>::rows() const
{
    return rows_;
}

template <typename T>
void Matrix<T>::set_col(const size_t &col, const std::vector<T> &data)
{
    if (col >= cols())
    {
        throw std::out_of_range("Matrix column index " + std::to_string(col) +
                                " out of range for matrix with " +
                                std::to_string(cols()) + " columns");
    }

    if (data.size() != rows())
    {
        throw std::out_of_range("Can't insert column vector of size " +
                                std::to_string(data.size()) +
                                " into matrix with column length " +
                                std::to_string(rows()));
    }

    for (size_t row = 0; row < rows(); ++row)
    {
        (*this)(row, col) = data[row];
    }
}

template <typename T>
void Matrix<T>::set_row(const size_t &row, const std::vector<T> &data)
{
    if (row >= rows())
    {
        throw std::out_of_range("Matrix row index " + std::to_string(row) +
                                " out of range for matrix with " +
                                std::to_string(rows()) + " rows");
    }

    if (data.size() != cols())
    {
        throw std::out_of_range(
            "Can't insert row vector of size " + std::to_string(data.size()) +
            " into matrix with row length " + std::to_string(cols()));
    }

    for (size_t col = 0; col < cols(); ++col)
    {
        (*this)(row, col) = data[col];
    }
}

template <typename T> size_t Matrix<T>::cols() const
{
    return cols_;
}

template <typename T>
std::vector<T> Matrix<T>::solveLinearSystem(std::vector<T> b)
{
    Matrix<T> A = *this;
    if (A.rows() != A.cols())
    {
        throw std::invalid_argument("Linear solve requires a square matrix");
    }

    const size_t n = A.rows();

    if (n == 0)
    {
        throw std::invalid_argument("Linear solve requires a non-empty matrix");
    }

    if (b.size() != n)
    {
        throw std::invalid_argument(
            "Linear solve right-hand side size does not match matrix rows");
    }

    for (size_t col = 0; col < n; ++col)
    {
        size_t pivot_row = col;
        auto pivot_abs = std::abs(A(col, col));

        for (size_t row = col + 1; row < n; ++row)
        {
            const auto candidate_abs = std::abs(A(row, col));

            if (candidate_abs > pivot_abs)
            {
                pivot_abs = candidate_abs;
                pivot_row = row;
            }
        }

        if (pivot_abs <= std::numeric_limits<T>::epsilon())
        {
            throw std::runtime_error("Linear solve matrix is singular");
        }

        if (pivot_row != col)
        {
            for (size_t swap_col = col; swap_col < n; ++swap_col)
            {
                std::swap(A(col, swap_col), A(pivot_row, swap_col));
            }

            std::swap(b[col], b[pivot_row]);
        }

        for (size_t row = col + 1; row < n; ++row)
        {
            const T factor = A(row, col) / A(col, col);
            A(row, col) = T{};

            for (size_t update_col = col + 1; update_col < n; ++update_col)
            {
                A(row, update_col) -= factor * A(col, update_col);
            }

            b[row] -= factor * b[col];
        }
    }

    std::vector<T> x(n, T{});

    for (size_t reverse_row = n; reverse_row > 0; --reverse_row)
    {
        const size_t row = reverse_row - 1;
        T sum = b[row];

        for (size_t col = row + 1; col < n; ++col)
        {
            sum -= A(row, col) * x[col];
        }

        x[row] = sum / A(row, row);
    }

    return x;
}

} // namespace sf::math

#endif // MATRIX_TPP
