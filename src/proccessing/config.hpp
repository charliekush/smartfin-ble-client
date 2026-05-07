/**
 * @file config.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Project-wide scalar type and default sensor calibration constants.
 * @date 2026-05-07
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "lin_alg.hpp"

namespace CFG
{
/// @brief Floating point precision used throughout the processing pipeline.
typedef double Scalar;

/// @brief Default gyroscope bias in rad/s. Replace with lab-calibrated values.
/// @todo Collect calibrated gyro bias in lab.
constexpr math3d::Vec3 default_gyro_bias_rad_s = {0, 0, 0};

} // namespace CFG

#endif // CONFIG_HPP
