/**
 * @file receiver.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Entry point for the Smartfin BLE receiver.
 * @date 2026-04-30
 * 
 */

#include "ble_config.hpp"

#include "pipeline/buffer_sink.hpp"
#include "session/session.hpp"
#include "transport/simpleble_adapter.hpp"

#include <cstdio>
#include <cstdlib>

/**
 * @brief Run post-ride processing after the BLE session completes.
 *
 * This hook runs after @c session.run() returns. Replace the body with
 * Madgwick filtering, FFT analysis, wave detection, or export logic.
 *
 * @param temps Buffered temperature samples from the session.
 * @param imus Buffered IMU samples from the session.
 */
static void process_ride(std::span<const sf::protocol::DecodedTemp> temps,
                         std::span<const sf::protocol::DecodedImu>  imus) {
    std::printf("\n=== post-ride processing ===\n");
    std::printf("  temp samples : %zu\n", temps.size());
    std::printf("  IMU  samples : %zu\n", imus.size());

    if (!temps.empty()) {
        float min_t = temps[0].temp_c, max_t = temps[0].temp_c;
        for (const auto& s : temps) {
            if (s.temp_c < min_t) min_t = s.temp_c;
            if (s.temp_c > max_t) max_t = s.temp_c;
        }
        std::printf("  water temp   : %.2f – %.2f °C\n", min_t, max_t);
    }

    if (!imus.empty()) {
        /// Report the elapsed time covered by the buffered IMU samples.
        const uint32_t duration_ms = imus.back().elapsed_time_ms
                                   - imus.front().elapsed_time_ms;
        std::printf("  IMU duration : %.1f s\n", duration_ms / 1000.0f);
    }

    /// @todo Add Madgwick filtering to compute an attitude time series.
    /// @todo Add FFT analysis on @c accel_ms2[2] to estimate wave period.
    /// @todo Add wave detection metrics such as count, height, and duration.
    /// @todo Export processed results to JSON or CSV.
}

/**
 * @brief Program entry point.
 * @return @c EXIT_SUCCESS on a successful session, otherwise @c EXIT_FAILURE.
 */
int main() {
    sf::transport::SimpleBleAdapter adapter;
    sf::pipeline::BufferSink        sink;

    sf::session::SessionConfig cfg{
        .device_name      = cfg::DEVICE_NAME,
        .service_uuid     = cfg::SERVICE_UUID,
        .telemetry_uuid   = cfg::TELEMETRY_UUID,
        .control_uuid     = cfg::CONTROL_UUID,
        .scan_timeout_s   = cfg::SCAN_TIMEOUT_S,
        .run_duration_s   = cfg::RUN_DURATION_S,
        .target_ensembles = cfg::TARGET_ENSEMBLES,
    };

    sf::session::Session session(adapter, sink, cfg);

    if (!session.run()) {
        std::fprintf(stderr, "session failed - no ensembles received\n");
        return EXIT_FAILURE;
    }

    process_ride(sink.temperatures(), sink.imu_samples());
    return EXIT_SUCCESS;
}
