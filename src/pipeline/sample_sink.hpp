/**
 * @file sample_sink.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief ISampleSink interface for consuming decoded sensor samples.
 * @date 2026-04-30
 * 
 */

#ifndef SAMPLE_SINK_HPP
#define SAMPLE_SINK_HPP

#include "protocol/ensemble_types.hpp"

namespace sf::pipeline {

    /**
     * @brief Interface implemented by anything that consumes decoded sensor samples.
     *
     * Implementations receive one call per decoded ensemble from the Session
     * worker thread. Examples:
     *   - BufferSink     - accumulate samples for post-ride processing
     *   - MadgwickSink   - feed accel + gyro + mag into a Madgwick AHRS filter
     *   - FftSink        - accumulate a window then run a real FFT
     *   - CsvSink        - write columns to a file
     *   - MultiSink      - fan-out to N child sinks
     *
     * @note @c on_imu and @c on_temperature are called from the Session worker
     *       thread. Implementations must be thread-safe with respect to that
     *       thread. If processing is slow (e.g. FFT), buffer internally and hand
     *       off to a dedicated compute thread.
     */
    class ISampleSink
    {
    public:
        /**
         * @brief Destroy the sink interface.
         */
        virtual ~ISampleSink() = default;

        /**
         * @brief Called once per decoded temperature ensemble.
         * @param sample  Decoded temperature and water-immersion state.
         */
        virtual void on_temperature(const sf::protocol::DecodedTemp &sample) = 0;

        /**
         * @brief Called once per decoded IMU ensemble.
         * @param sample  Decoded accel, gyro, and magnetometer vectors.
         */
        virtual void on_imu(const sf::protocol::DecodedImu &sample) = 0;
    };

}

#endif
