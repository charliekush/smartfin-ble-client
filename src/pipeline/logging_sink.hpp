/**
 * @file logging_sink.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief ISampleSink that prints each received ensemble to stdout.
 * @date 2026-05-06
 */

#ifndef LOGGING_SINK_HPP
#define LOGGING_SINK_HPP

#include "sample_sink.hpp"
#include "protocol/ensemble_types.hpp"

#include <cstdio>
#include <cstdint>
#include <atomic>

namespace sf::pipeline {

    /**
     * @brief ISampleSink that logs every decoded ensemble to stdout in real time.
     *
     * Intended for interactive testing: connect the fin, watch ensembles scroll
     * by, and verify that the BLE transport + decoder are working end-to-end.
     *
     * @par Output format
     * @code
     *   [TEMP  #    1]  t=   0.0 s  temp= 21.35 °C  in_water=0
     *   [IMU   #    1]  t=   0.0 s  ax= -0.012  ay=  0.005  az=  9.807 m/s^2
     *                               gx=  0.10   gy= -0.05   gz=  0.02  °/s
     *                               mx= 20.1    my=-14.3    mz= 42.7   uT
     * @endcode
     */
    class LoggingSink final : public ISampleSink
    {
    public:
        /**
         * @brief Print a temperature ensemble to stdout.
         * @param s  Decoded temperature ensemble.
         */
        void on_temperature(const sf::protocol::DecodedTemp &s) override
        {
            const uint32_t n = ++temp_count_;
            std::printf("[TEMP  #%5u]  t=%7.1f s  temp=%6.2f °C  in_water=%d\n",
                        n,
                        s.elapsed_time_ms / 1000.0f,
                        s.temp_c,
                        static_cast<int>(s.in_water));
            std::fflush(stdout);
        }

        /**
         * @brief Print a raw IMU ensemble to stdout
         * @param s  Decoded IMU ensemble
         */
        void on_imu(const sf::protocol::DecodedImu &s) override
        {
            const uint32_t n = ++imu_count_;
            std::printf(
                "[IMU   #%5u]  t=%7.1f s"
                "  ax=%7.3f  ay=%7.3f  az=%7.3f m/s^2\n"
                "                         "
                "  gx=%7.2f  gy=%7.2f  gz=%7.2f °/s\n"
                "                         "
                "  mx=%7.1f  my=%7.1f  mz=%7.1f uT\n",
                n,
                s.elapsed_time_ms / 1000.0f,
                s.accel_ms2[0], s.accel_ms2[1], s.accel_ms2[2],
                s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2],
                s.mag_uT[0], s.mag_uT[1], s.mag_uT[2]);
            std::fflush(stdout);
        }

        /**
         * @brief Print a firmware version ensemble to stdout
         * @param s  Decoded firmware version ensemble
         */
        void on_fw_version(const sf::protocol::DecodedFwVersion &s) override
        {
            std::printf("[FWVER      ]  t=%7.1f s  version=%s\n",
                        s.elapsed_time_ms / 1000.0f,
                        s.version);
            std::fflush(stdout);
        }

        /// @brief Number of temperature ensembles received since construction
        uint32_t temp_count() const { return temp_count_.load(); }
        /// @brief Number of IMU ensembles received since construction
        uint32_t imu_count() const { return imu_count_.load(); }

    private:
        /// Running count of temperature ensembles
        std::atomic<uint32_t> temp_count_{0};
        /// Running count of IMU ensembles
        std::atomic<uint32_t> imu_count_{0};
    };

} // namespace sf::pipeline

#endif // LOGGING_SINK_HPP
