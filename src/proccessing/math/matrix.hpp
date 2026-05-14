/**
 * @file matrix.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Small 2D matrix helpers for filtering and processing math.
 * @date 2026-05-13
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <cstddef>
#include <vector>

namespace sf::math
{

/**
 * @brief Dense row-major 2D matrix.
 * @tparam T Value type stored in the matrix.
 *
 * This class stores matrix data contiguously in row-major order, so element
 * `(row, col)` is stored at `row * cols() + col`. It is intended as a small
 * utility for processing algorithms that need basic matrix storage without
 * pulling in a larger linear algebra dependency.
 */
template <typename T = double> class Matrix
{
  public:
    using value_type = T; ///< Type stored by each matrix element.

    /**
     * @brief Construct a matrix with all entries initialized to one value.
     * @param rows Number of matrix rows.
     * @param cols Number of matrix columns.
     * @param initial Initial value assigned to every element.
     */
    Matrix(size_t rows, size_t cols, T initial = T{});

    /**
     * @brief Write a column from a vector of values.
     * @param col  Zero-based column index.
     * @param data Values to write; must have exactly @c rows() elements.
     */
    void set_col(const size_t &col, const std::vector<T> &data);

    /**
     * @brief Extract a column as a vector.
     * @param col  Zero-based column index.
     * @return Vector of @c rows() values from the requested column.
     */
    std::vector<T> get_col(const size_t &col) const;

    /**
     * @brief Write a row from a vector of values.
     * @param row  Zero-based row index.
     * @param data Values to write; must have exactly @c cols() elements.
     */
    void set_row(const size_t &row, const std::vector<T> &data);
    /**
     * @brief Access a mutable matrix element.
     * @param row Zero-based row index.
     * @param col Zero-based column index.
     * @return Reference to the requested element.
     */
    T &operator()(size_t row, size_t col);

    /**
     * @brief Access a read-only matrix element.
     * @param row Zero-based row index.
     * @param col Zero-based column index.
     * @return Value of the requested element.
     */
    const T &operator()(size_t row, size_t col) const;

    /**
     * @brief Get the number of rows in the matrix.
     * @return Matrix row count.
     */
    size_t rows() const;

    /**
     * @brief Get the number of columns in the matrix.
     * @return Matrix column count.
     */
    size_t cols() const;

    /**
     * @brief Solve a dense linear system.
     * @param A Coefficient matrix for the system. Passed by value so the solver
     *          may freely modify it during elimination.
     * @param b Right-hand-side vector. Passed by value so the solver may freely
     *          modify it during elimination.
     * @return Solution vector `x` such that `A * x = b`.
     */
    std::vector<T> solveLinearSystem(std::vector<T> b);

    /**
     * @brief Return a copy of this matrix with row order reversed.
     * @return New matrix whose first row is this matrix's last row, etc.
     */
    Matrix<T> flip_rows() const;

  private:
    size_t rows_;         ///< Number of rows in the matrix.
    size_t cols_;         ///< Number of columns in the matrix.
    std::vector<T> data_; ///< Matrix values stored in row-major order.
};

} // namespace sf::math

#include "proccessing/math/matrix.tpp"

#endif // MATRIX_HPP
