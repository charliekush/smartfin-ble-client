/**
 * @file lin_alg.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Linear algebra primitives
 * @date 2026-04-30
 *
 *
 */
#ifndef LIN_ALG_HPP
#define LIN_ALG_HPP

namespace math3d {

/**
 * @brief Small threshold used for zero-magnitude and normalization checks.
 */
constexpr double EPSILON = 1e-6f;

/**
 * @brief Three-component double-precision vector.
 */
struct Vec3 {
    double x = 0.0f; ///< X component.
    double y = 0.0f; ///< Y component.
    double z = 0.0f; ///< Z component.

    /**
     * @brief Construct a zero vector.
     */
    constexpr Vec3() = default;

    /**
     * @brief Construct a vector from explicit components.
     * @param x_ X component.
     * @param y_ Y component.
     * @param z_ Z component.
     */
    constexpr Vec3(double x_, double y_, double z_)
        : x(x_), y(y_), z(z_) {}

    /**
     * @brief Add two vectors component-wise.
     * @param rhs Right-hand operand.
     * @return Sum of the two vectors.
     */
    constexpr Vec3 operator+(Vec3 rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    /**
     * @brief Subtract two vectors component-wise.
     * @param rhs Right-hand operand.
     * @return Difference of the two vectors.
     */
    constexpr Vec3 operator-(Vec3 rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    /**
     * @brief Negate the vector.
     * @return Vector with each component sign-flipped.
     */
    constexpr Vec3 operator-() const {
        return {-x, -y, -z};
    }

    /**
     * @brief Scale the vector by a scalar.
     * @param s Scale factor.
     * @return Scaled vector.
     */
    constexpr Vec3 operator*(double s) const
    {
        return {x * s, y * s, z * s};
    }

    /**
     * @brief Divide the vector by a scalar.
     * @param s Divisor.
     * @return Scaled vector.
     */
    constexpr Vec3 operator/(double s) const
    {
        return {x / s, y / s, z / s};
    }

    /**
     * @brief Add another vector in place.
     * @param rhs Right-hand operand.
     * @return Reference to this vector.
     */
    Vec3& operator+=(Vec3 rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    /**
     * @brief Subtract another vector in place.
     * @param rhs Right-hand operand.
     * @return Reference to this vector.
     */
    Vec3& operator-=(Vec3 rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    /**
     * @brief Scale this vector in place.
     * @param s Scale factor.
     * @return Reference to this vector.
     */
    Vec3 &operator*=(double s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    /**
     * @brief Divide this vector by a scalar in place.
     * @param s Divisor.
     * @return Reference to this vector.
     */
    Vec3 &operator/=(double s)
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }
};

/**
 * @brief Scale a vector by a scalar with the scalar on the left.
 * @param s Scale factor.
 * @param v Vector to scale.
 * @return Scaled vector.
 */
constexpr Vec3 operator*(double s, Vec3 v)
{
    return v * s;
}

/**
 * @brief Compute the dot product of two vectors.
 * @param a Left-hand operand.
 * @param b Right-hand operand.
 * @return Dot product.
 */
double dot(Vec3 a, Vec3 b);

/**
 * @brief Compute the cross product of two vectors.
 * @param a Left-hand operand.
 * @param b Right-hand operand.
 * @return Cross product.
 */
Vec3 cross(Vec3 a, Vec3 b);

/**
 * @brief Compute the squared Euclidean norm of a vector.
 * @param v Input vector.
 * @return Squared magnitude.
 */
double normSquared(Vec3 v);

/**
 * @brief Compute the Euclidean norm of a vector.
 * @param v Input vector.
 * @return Magnitude.
 */
double norm(Vec3 v);

/**
 * @brief Check whether all vector components are finite.
 * @param v Input vector.
 * @return @c true if every component is finite.
 */
bool isFinite(Vec3 v);

/**
 * @brief Check whether a vector magnitude exceeds the epsilon threshold.
 * @param v Input vector.
 * @return @c true if the vector is considered non-zero.
 */
bool isNonzero(Vec3 v);

/**
 * @brief Normalize a vector.
 * @param v Input vector.
 * @return Unit vector, or the zero vector when the input is too small or non-finite.
 */
Vec3 normalize(Vec3 v);

/**
 * @brief Quaternion with scalar-first storage.
 */
struct Quaternion {
    double w = 1.0f; ///< Scalar component.
    double x = 0.0f; ///< X vector component.
    double y = 0.0f; ///< Y vector component.
    double z = 0.0f; ///< Z vector component.

    /**
     * @brief Construct the identity quaternion.
     */
    constexpr Quaternion() = default;

    /**
     * @brief Construct a quaternion from explicit components.
     * @param w_ Scalar component.
     * @param x_ X vector component.
     * @param y_ Y vector component.
     * @param z_ Z vector component.
     */
    constexpr Quaternion(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    /**
     * @brief Get the multiplicative identity quaternion.
     * @return Identity quaternion.
     */
    static constexpr Quaternion identity() {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    /**
     * @brief Build a pure quaternion from a vector.
     * @param v Vector part to embed.
     * @return Quaternion with zero scalar part.
     */
    static constexpr Quaternion pure(Vec3 v) {
        return {0.0f, v.x, v.y, v.z};
    }

    /**
     * @brief Extract the vector part of the quaternion.
     * @return Vector portion of the quaternion.
     */
    constexpr Vec3 vector() const {
        return {x, y, z};
    }

    /**
     * @brief Add two quaternions component-wise.
     * @param rhs Right-hand operand.
     * @return Sum quaternion.
     */
    constexpr Quaternion operator+(Quaternion rhs) const {
        return {
            w + rhs.w,
            x + rhs.x,
            y + rhs.y,
            z + rhs.z
        };
    }

    /**
     * @brief Subtract two quaternions component-wise.
     * @param rhs Right-hand operand.
     * @return Difference quaternion.
     */
    constexpr Quaternion operator-(Quaternion rhs) const {
        return {
            w - rhs.w,
            x - rhs.x,
            y - rhs.y,
            z - rhs.z
        };
    }

    /**
     * @brief Scale a quaternion by a scalar.
     * @param s Scale factor.
     * @return Scaled quaternion.
     */
    constexpr Quaternion operator*(double s) const
    {
        return {
            w * s,
            x * s,
            y * s,
            z * s
        };
    }

    /**
     * @brief Add another quaternion in place.
     * @param rhs Right-hand operand.
     * @return Reference to this quaternion.
     */
    Quaternion& operator+=(Quaternion rhs) {
        w += rhs.w;
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    /**
     * @brief Scale this quaternion in place.
     * @param s Scale factor.
     * @return Reference to this quaternion.
     */
    Quaternion &operator*=(double s)
    {
        w *= s;
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
};

/**
 * @brief Scale a quaternion by a scalar with the scalar on the left.
 * @param s Scale factor.
 * @param q Quaternion to scale.
 * @return Scaled quaternion.
 */
constexpr Quaternion operator*(double s, Quaternion q)
{
    return q * s;
}

/**
 * @brief Multiply two quaternions.
 * @param a Left-hand operand.
 * @param b Right-hand operand.
 * @return Product quaternion.
 */
Quaternion operator*(Quaternion a, Quaternion b);

/**
 * @brief Compute the squared Euclidean norm of a quaternion.
 * @param q Input quaternion.
 * @return Squared magnitude.
 */
double normSquared(Quaternion q);

/**
 * @brief Compute the Euclidean norm of a quaternion.
 * @param q Input quaternion.
 * @return Magnitude.
 */
double norm(Quaternion q);

/**
 * @brief Check whether all quaternion components are finite.
 * @param q Input quaternion.
 * @return @c true if every component is finite.
 */
bool isFinite(Quaternion q);

/**
 * @brief Compute the quaternion conjugate.
 * @param q Input quaternion.
 * @return Conjugated quaternion.
 */
Quaternion conjugate(Quaternion q);

/**
 * @brief Normalize a quaternion.
 * @param q Input quaternion.
 * @return Unit quaternion, or the identity quaternion when the input is too small or non-finite.
 */
Quaternion normalize(Quaternion q);

/**
 * @brief Rotate a vector by a quaternion.
 * @param q Rotation quaternion.
 * @param v Vector to rotate.
 * @return Rotated vector.
 */
Vec3 rotateVector(Quaternion q, Vec3 v);

/**
 * @brief Estimate the gravity direction encoded by an orientation quaternion.
 * @param q Orientation quaternion.
 * @return Gravity vector in the body frame.
 */
Vec3 gravityFromQuaternion(Quaternion q);

/**
 * @brief Estimate the east direction encoded by an orientation quaternion.
 * @param q Orientation quaternion.
 * @return East vector in the body frame.
 */
Vec3 eastFromQuaternion(Quaternion q);

} // namespace math3d
#endif // LIN_ALG_HPP
