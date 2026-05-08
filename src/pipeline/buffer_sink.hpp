/**
 * @file buffer_sink.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief ISampleSink that accumulates samples for post-ride processing.
 * @date 2026-04-30
 * 
 */

#ifndef BUFFER_SINK_HPP
#define BUFFER_SINK_HPP

#include "sample_sink.hpp"
#include "protocol/ensemble_types.hpp"

#include <optional>
#include <span>
#include <vector>

namespace sf::pipeline {

/**
 * @brief ISampleSink that accumulates all samples in memory during a session.
 *
 * Designed for the post-ride processing model: collect raw samples while BLE
 * is connected, then run algorithms (Madgwick, FFT, wave detection) over the
 * complete buffer after @c Session::run() returns.
 *
 * @par Usage
 * @code
 *   BufferSink sink;
 *   Session session(adapter, sink, cfg);
 *   session.run();                       // blocks until disconnect
 *
 *   auto imus  = sink.imu_samples();     // full ride IMU data
 *   auto temps = sink.temperatures();    // full ride temp data
 *   // then run Madgwick, FFT, CSV export, wave detection, etc.
 * @endcode
 *
 * @par Memory
 * At 100 Hz IMU for a 1-hour session: ~360 k samples × 40 bytes ≈ 14 MB.
 * Acceptable on desktop and iOS; profile before using on watchOS.
 */
class BufferSink final : public ISampleSink {
public:
    /**
     * @brief Append a temperature sample to the internal buffer.
     * @param s  Decoded temperature ensemble.
     */
    void on_temperature(const sf::protocol::DecodedTemp& s) override {
        temps_.push_back(s);
    }

    /**
     * @brief Append an IMU sample to the internal buffer.
     * @param s  Decoded IMU ensemble.
     */
    void on_imu(const sf::protocol::DecodedImu& s) override {
        imu_.push_back(s);
    }

    /**
     * @brief Store the firmware version string (last received wins).
     * @param s  Decoded firmware version ensemble.
     */
    void on_fw_version(const sf::protocol::DecodedFwVersion& s) override {
        fw_version_ = s;
    }

    /**
     * @brief Read-only view of all accumulated temperature samples.
     * @return Span over the internal temperature vector.
     */
    std::span<const sf::protocol::DecodedTemp> temperatures() const {
        return temps_;
    }

    /**
     * @brief Read-only view of all accumulated IMU samples.
     * @return Span over the internal IMU vector.
     */
    std::span<const sf::protocol::DecodedImu> imu_samples() const {
        return imu_;
    }

    /**
     * @brief Return the firmware version if one was received this session.
     */
    const std::optional<sf::protocol::DecodedFwVersion>& fw_version() const {
        return fw_version_;
    }

    /**
     * @brief Discard all buffered samples.
     */
    void clear() {
        temps_.clear();
        imu_.clear();
        fw_version_.reset();
    }

private:
    std::vector<sf::protocol::DecodedTemp>              temps_;      ///< Accumulated temperature samples.
    std::vector<sf::protocol::DecodedImu>               imu_;        ///< Accumulated IMU samples.
    std::optional<sf::protocol::DecodedFwVersion>       fw_version_; ///< Most recent firmware version, if received.
};

} // namespace sf::pipeline

#endif // BUFFER_SINK_HPP
