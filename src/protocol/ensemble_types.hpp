/**
 * @file ensemble_types.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Decoded ensemble structs and the EnsembleId enum mirroring firmware.
 * @date 2026-04-30
 *
 */

#ifndef ENSEMBLE_TYPES_HPP
#define ENSEMBLE_TYPES_HPP

#include <cstdint>
#include <variant>

namespace sf::protocol {

    enum class EnsembleId : uint8_t
    {
        Temp = 0x01,            ///< Temperature only — SS_Ensemble01_Func.
        RawHighRateImu = 0x0C,  ///< High-rate IMU payload — SS_Ensemble12_x0C.
        QuatHighRateImu = 0x0C, ///< High-rate IMU payload — SS_Ensemble12_x0C.
        Text = 0x0F,            ///< ASCII text payload — SS_fwVerFunc.
    };

    /**
     * @brief Decoded payload for ENS_TEMP (0x01), Ensemble01_data_t.
     */
    struct DecodedTemp
    {
        uint32_t elapsed_time_ms; ///< Milliseconds since session start.
        float temp_c;             ///< Water temperature in degrees Celsius.
        bool in_water;            ///< True when the fin detects water immersion.
    };

/**
 * @brief Decoded payload for ENS_TEMP_HIGH_DATA_RATE_IMU (0x0C), Ensemble12_data_t.
 *
 * Fixed-point wire values are scaled to physical units on decode:
 * Q14 -> m/s^2, Q7 -> deg/s, Q3 -> uT.
 */
struct DecodedImu {
    uint32_t elapsed_time_ms; ///< Milliseconds since session start.
    float accel_ms2[3];       ///< Linear acceleration in m/s^2 [x, y, z].
    float gyro_dps[3];        ///< Angular rate in deg/s [x, y, z].
    float mag_uT[3];          ///< Magnetic field in uT [x, y, z].
};

/**
 * @brief Decoded payload for ENS_TEXT (0x0F) — SS_fwVerFunc.
 *
 * Wire layout: uint8_t nChars, char verBuf[nChars] (no null terminator).
 * Format: "FW<major>.<minor>.<build><branch>", max 32 chars.
 */
struct DecodedFwVersion {
    uint32_t elapsed_time_ms;
    char     version[33]; ///< Null-terminated version string, max 32 wire chars.
};

using DecodedEnsemble = std::variant<DecodedTemp, DecodedImu, DecodedFwVersion>;

} // namespace sf::protocol

#endif
