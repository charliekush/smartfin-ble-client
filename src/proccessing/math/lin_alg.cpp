/**
 * @file lin_alg.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief linear algebra primitives implementation.
 * @date 2026-05-07
 *
 *
 */

#include "proccessing/math/lin_alg.hpp"

#include <cmath>

namespace math3d
{

    double dot(Vec3 a, Vec3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vec3 cross(Vec3 a, Vec3 b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    }

    double normSquared(Vec3 v)
    {
        return dot(v, v);
    }

    double norm(Vec3 v)
    {
        return std::sqrt(normSquared(v));
    }

    bool isFinite(Vec3 v)
    {
        return std::isfinite(v.x) &&
               std::isfinite(v.y) &&
               std::isfinite(v.z);
    }

    bool isNonzero(Vec3 v)
    {
        return normSquared(v) > EPSILON * EPSILON;
    }

    Vec3 normalize(Vec3 v)
    {
        const double n = norm(v);

        if (!std::isfinite(n) || n < EPSILON)
        {
            return {0.0f, 0.0f, 0.0f};
        }

        return v / n;
    }

    Quaternion operator*(Quaternion a, Quaternion b)
    {
        return {
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    double normSquared(Quaternion q)
    {
        return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    }

    double norm(Quaternion q)
    {
        return std::sqrt(normSquared(q));
    }

    bool isFinite(Quaternion q)
    {
        return std::isfinite(q.w) &&
               std::isfinite(q.x) &&
               std::isfinite(q.y) &&
               std::isfinite(q.z);
    }

    Quaternion conjugate(Quaternion q)
    {
        return {q.w, -q.x, -q.y, -q.z};
    }

    Quaternion normalize(Quaternion q)
    {
        const double n = norm(q);

        if (!std::isfinite(n) || n < EPSILON)
        {
            return Quaternion::identity();
        }

        return q * (1.0f / n);
    }

    Vec3 rotateVector(Quaternion q, Vec3 v)
    {
        const Quaternion qn = normalize(q);
        const Quaternion result = qn * Quaternion::pure(v) * conjugate(qn);
        return result.vector();
    }

    Vec3 gravityFromQuaternion(Quaternion q)
    {
        q = normalize(q);

        return {
            2.0f * q.x * q.z - 2.0f * q.w * q.y,
            2.0f * q.y * q.z + 2.0f * q.w * q.x,
            2.0f * q.w * q.w - 1.0f + 2.0f * q.z * q.z};
    }

    Vec3 eastFromQuaternion(Quaternion q)
    {
        q = normalize(q);

        return {
            -2.0f * q.w * q.w + 1.0f - 2.0f * q.x * q.x,
            -2.0f * q.x * q.y + 2.0f * q.w * q.z,
            -2.0f * q.x * q.z - 2.0f * q.w * q.y};
    }

} // namespace math3d
