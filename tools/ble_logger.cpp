/**
 * @file ble_logger.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Interactive BLE receiver that logs every decoded ensemble to stdout.
 *
 * Streams indefinitely (Ctrl-C to stop). Useful for verifying BLE transport
 * and decoder correctness before adding downstream processing.
 *
 * @date 2026-05-06
 */

#include "receiver/ble_config.hpp"
#include "pipeline/logging_sink.hpp"
#include "session/session.hpp"
#include "transport/simpleble_adapter.hpp"

#include <cstdio>
#include <cstdlib>

int main() {
    sf::transport::SimpleBleAdapter adapter;
    sf::pipeline::LoggingSink        sink;

    sf::session::SessionConfig cfg{
        .device_name      = cfg::DEVICE_NAME,
        .service_uuid     = cfg::SERVICE_UUID,
        .telemetry_uuid   = cfg::TELEMETRY_UUID,
        .control_uuid     = cfg::CONTROL_UUID,
        .scan_timeout_s   = cfg::SCAN_TIMEOUT_S,
        .run_duration_s   = 3600,  // stream for up to 1 hour; Ctrl-C to stop early
        .target_ensembles = 0,     // no ensemble cap  -  log everything
    };

    std::printf("[ble_logger] Scanning for \"%s\"...\n", cfg.device_name.c_str());

    sf::session::Session session(adapter, sink, cfg);

    if (!session.run()) {
        std::fprintf(stderr, "[ble_logger] Session ended with no ensembles received.\n");
        return EXIT_FAILURE;
    }

    const auto& stats = session.stats();
    std::printf("\n[ble_logger] Done  -  %llu ensembles (temp=%u  imu=%u)  %.1f Hz\n",
                static_cast<unsigned long long>(stats.ensemble_count),
                sink.temp_count(),
                sink.imu_count(),
                stats.avg_rate_hz);

    return EXIT_SUCCESS;
}
