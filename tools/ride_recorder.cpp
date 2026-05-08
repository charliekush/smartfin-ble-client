/**
 * @file ride_recorder.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Records a BLE session to a binary ride file for offline processing.
 *
 * Usage: ./ride_recorder [output.sfdat]
 *   Defaults to ride_<timestamp>.sfdat in the current directory.
 *   Press Ctrl-C to stop early.
 *
 * @date 2026-05-08
 */

#include "receiver/ble_config.hpp"
#include "pipeline/file_sink.hpp"
#include "session/session.hpp"
#include "transport/simpleble_adapter.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

static std::string default_filename()
{
    const std::time_t now = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "ride_%Y%m%d_%H%M%S.sfdat",
                  std::localtime(&now));
    return buf;
}

int main(int argc, char* argv[])
{
    const std::string path = (argc >= 2) ? argv[1] : default_filename();

    std::printf("[ride_recorder] Output: %s\n", path.c_str());
    std::printf("[ride_recorder] Scanning for \"%s\"...\n",
                cfg::DEVICE_NAME);

    sf::transport::SimpleBleAdapter adapter;
    sf::pipeline::FileSink           sink(path);

    sf::session::SessionConfig cfg{
        .device_name      = cfg::DEVICE_NAME,
        .service_uuid     = cfg::SERVICE_UUID,
        .telemetry_uuid   = cfg::TELEMETRY_UUID,
        .control_uuid     = cfg::CONTROL_UUID,
        .scan_timeout_s   = cfg::SCAN_TIMEOUT_S,
        .run_duration_s   = 7200, // up to 2 hours; Ctrl-C to stop early
        .target_ensembles = 0,
    };

    sf::session::Session session(adapter, sink, cfg);

    if (!session.run()) {
        std::fprintf(stderr,
            "[ride_recorder] Session ended with no ensembles.\n");
        return EXIT_FAILURE;
    }

    const auto& stats = session.stats();
    std::printf(
        "[ride_recorder] Done — %llu ensembles  %.1f Hz  -> %s\n",
        static_cast<unsigned long long>(stats.ensemble_count),
        stats.avg_rate_hz,
        path.c_str());

    return EXIT_SUCCESS;
}
