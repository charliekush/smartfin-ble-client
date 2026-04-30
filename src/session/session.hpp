/**
 * @file session.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief BLE session orchestrator: scan, connect, stream, decode, sink.
 * @date 2026-04-30
 * 
 */

#ifndef SESSION_HPP
#define SESSION_HPP

#include "pipeline/sample_sink.hpp"
#include "transport/ble_adapter.hpp"

#include <cstdint>
#include <string>

namespace sf::session {

/**
 * @brief Configuration parameters for a single BLE session.
 */
struct SessionConfig {
    std::string device_name      = "Smartfin";                             ///< Advertised peripheral name to scan for.
    std::string service_uuid     = "a86d7b16-dd6c-434b-a7ee-f0ca33ac614c"; ///< GATT service UUID.
    std::string telemetry_uuid   = "deeddb00-166e-407c-8158-7b9693ad2685"; ///< Telemetry NOTIFY characteristic UUID.
    std::string control_uuid     = "c39513e6-631e-439a-9b3b-affa0635b3d1"; ///< Control WRITE_WO_RSP characteristic UUID.
    int         scan_timeout_s   = 30;                                     ///< Maximum BLE scan duration in seconds.
    int         run_duration_s   = 60;                                     ///< Maximum streaming duration in seconds.
    uint32_t    target_ensembles = 300;                                    ///< Stop early when this many ensembles are decoded, or 0 for no limit.
};

/**
 * @brief Transport and decode statistics collected during a session.
 */
struct SessionStats {
    uint64_t notify_count    = 0;   ///< Total BLE notifications received.
    uint64_t ensemble_count  = 0;   ///< Total ensembles successfully decoded.
    uint64_t decode_errors   = 0;   ///< Packets that failed transport-header validation.
    uint64_t dropped_packets = 0;   ///< Packets dropped due to queue overflow or oversized payload.
    double   run_time_s      = 0.0; ///< Actual session duration in seconds.
    double   avg_rate_hz     = 0.0; ///< Average notification rate in hertz.
};

/**
 * @brief Orchestrates one BLE session: scan → connect → subscribe → decode → sink.
 *
 * @par Threading model
 * The BLE stack delivers notification callbacks on its own internal thread.
 * Session maintains a bounded queue: the callback thread pushes raw bytes,
 * and a dedicated worker thread pops them, decodes each packet via
 * @c decode_packet(), and forwards results to the @ref sf::pipeline::ISampleSink.
 *
 * @note @c std::jthread / @c std::stop_token are not yet available in Apple
 *       libc++. The worker uses @c std::thread with a manual atomic stop flag.
 *
 * @par Usage
 * @code
 *   SimpleBleAdapter adapter;
 *   BufferSink       sink;
 *   Session session(adapter, sink, cfg);
 *   session.run();
 *   process_ride(sink.imu_samples());
 * @endcode
 */
class Session {
public:
    /**
     * @brief Construct a session with the given adapter, sink, and config.
     * @param adapter  BLE hardware abstraction (injected; not owned).
     * @param sink     Sample consumer (injected; not owned).
     * @param cfg      Session parameters.
     */
    Session(sf::transport::IBleAdapter& adapter,
            sf::pipeline::ISampleSink& sink,
            const SessionConfig& cfg = {});

    /**
     * @brief Destroy the session and stop the worker thread if it is running.
     */
    ~Session();

    /**
     * @brief Run the full session: scan, connect, stream, disconnect.
     *
     * Blocks until @c run_duration_s elapses, @c target_ensembles is reached,
     * or the peripheral disconnects unexpectedly.
     *
     * @return @c true if at least one ensemble was received.
     */
    bool run();

    /**
     * @brief Access transport statistics from the most recent @c run() call.
     * @return Const reference to the internal stats struct.
     */
    const SessionStats& stats() const { return stats_; }

private:
    /**
     * @brief Drain queued packets, decode them, and dispatch typed samples.
     */
    void worker_loop();

    /**
     * @brief Forward a decoded ensemble to the appropriate sink callback.
     * @param ensemble Decoded ensemble variant to dispatch.
     */
    void dispatch(const sf::protocol::DecodedEnsemble& ensemble);

    /**
     * @brief BLE adapter implementation used for transport operations.
     */
    sf::transport::IBleAdapter& adapter_;

    /**
     * @brief Sample sink that receives decoded ensembles.
     */
    sf::pipeline::ISampleSink&  sink_;

    /**
     * @brief Session configuration parameters.
     */
    SessionConfig               cfg_;

    /**
     * @brief Statistics accumulated during the most recent run.
     */
    SessionStats                stats_;

    /**
     * @brief Fixed-size packet buffer used for cross-thread handoff.
     */
    struct RawPacket {
        static constexpr size_t MAX_SIZE = 256; ///< Maximum supported ATT payload.
        uint8_t data[MAX_SIZE];                 ///< Packet bytes copied from the BLE callback.
        size_t  len;                            ///< Number of valid bytes stored in @ref data.
    };

    /**
     * @brief Maximum number of raw packets buffered before older packets are dropped.
     */
    static constexpr size_t QUEUE_CAPACITY = 128;

    /**
     * @brief Opaque implementation that owns synchronization primitives and the worker thread.
     */
    struct Impl;

    /**
     * @brief Pointer to the private implementation state.
     */
    Impl* impl_;
};

}

#endif
