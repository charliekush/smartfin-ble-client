/**
 * @file constants.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Physical and unit-conversion constants used by the processing pipeline.
 * @date 2026-05-07
 */

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "config.hpp"

#include <numbers>

/// @brief Standard gravity in m/s^2.
constexpr CFG::Scalar G0 = 9.80665;

/// @brief Degrees to radians.
constexpr CFG::Scalar DEG_TO_RAD = std::numbers::pi / 180.0;

/// @brief Radians to degrees.
constexpr CFG::Scalar RAD_TO_DEG = 180.0 / std::numbers::pi;

/// @brief Milliseconds to seconds.
constexpr CFG::Scalar MS_TO_S = 1e-3;

#endif // CONSTANTS_HPP
