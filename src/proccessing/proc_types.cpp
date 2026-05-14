/**
 * @file proc_types.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief OrientedSample constructors.
 * @date 2026-05-12
 */

#include "proccessing/proc_types.hpp"
#include "proccessing/math/constants.hpp"

namespace sf::proc {

/**
 * @brief Construct an OrientedSample from a DMP quaternion IMU ensemble.
 * @param s  Decoded quaternion IMU ensemble.
 */
OrientedSample::OrientedSample(const sf::protocol::DecodedQuatImu& s)
    : elapsed_time_ms(s.elapsed_time_ms), q(s.q[0], s.q[1], s.q[2], s.q[3])
{
    const math3d::Vec3 accel_g{
        static_cast<double>(s.accel_ms2[0]) / G0,
        static_cast<double>(s.accel_ms2[1]) / G0,
        static_cast<double>(s.accel_ms2[2]) / G0,
    };
    const math3d::Vec3 gravity  = math3d::gravityFromQuaternion(q);
    const math3d::Vec3 accel_0g = accel_g - gravity;
    accel_global = math3d::rotateVector(q, accel_0g);
}

} // namespace sf::proc
