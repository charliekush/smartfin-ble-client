/**
 * @file session.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief BLE session orchestrator implementation.
 * @date 2026-04-30
 *
 */

#include "session.hpp"

#include "protocol/ensemble_decoder.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <span>
#include <thread>
#include <type_traits>
#include <variant>
#include <vector>

namespace sf::session
{

/**
 * @brief Private session state shared between the BLE callback and worker
 * thread.
 *
 * Apple libc++ does not yet provide the desired @c std::jthread and
 * @c std::stop_token support here, so the implementation uses
 * @c std::thread plus an atomic stop flag.
 */
struct Session::Impl
{
    std::deque<RawPacket> queue;
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> stop{false};
    std::thread worker;
};

Session::Session(sf::transport::IBleAdapter &adapter,
                 sf::pipeline::ISampleSink &sink, const SessionConfig &cfg)
    : adapter_(adapter), sink_(sink), cfg_(cfg), impl_(new Impl{})
{
}

Session::~Session()
{
    if (impl_->worker.joinable())
    {
        impl_->stop.store(true);
        impl_->cv.notify_all();
        impl_->worker.join();
    }
    delete impl_;
}

void Session::worker_loop()
{
    std::vector<sf::protocol::DecodedEnsemble> decoded;
    decoded.reserve(32);

    while (true)
    {
        RawPacket pkt{};
        {
            std::unique_lock lock(impl_->mtx);
            impl_->cv.wait(
                lock,
                [this] { return !impl_->queue.empty() || impl_->stop.load(); });
            if (impl_->queue.empty())
                break;

            pkt = impl_->queue.front();
            impl_->queue.pop_front();
        }

        decoded.clear();
        if (!sf::protocol::decode_packet(
                std::span<const uint8_t>(pkt.data, pkt.len), decoded))
        {
            ++stats_.decode_errors;
            continue;
        }

        for (const auto &ens : decoded)
        {
            ++stats_.ensemble_count;
            dispatch(ens);
        }
    }
}

void Session::dispatch(const sf::protocol::DecodedEnsemble &ens)
{
    std::visit(
        [this](const auto &sample)
        {
            using T = std::decay_t<decltype(sample)>;
            if constexpr (std::is_same_v<T, sf::protocol::DecodedTemp>)
            {
                sink_.on_temperature(sample);
            }
            else if constexpr (std::is_same_v<T, sf::protocol::DecodedImu>)
            {
                sink_.on_imu(sample);
            }
            else if constexpr (std::is_same_v<T, sf::protocol::DecodedQuatImu>)
            {
                sink_.on_quat_imu(sample);
            }
            else if constexpr (std::is_same_v<T,
                                              sf::protocol::DecodedFwVersion>)
            {
                sink_.on_fw_version(sample);
            }
        },
        ens);
}

bool Session::run()
{
    stats_ = {};

    if (!adapter_.scan_and_connect(cfg_.device_name, cfg_.scan_timeout_s))
    {
        return false;
    }

    impl_->stop.store(false);
    impl_->worker = std::thread([this] { worker_loop(); });

    bool subscribed = adapter_.subscribe_notify(
        cfg_.service_uuid, cfg_.telemetry_uuid,
        [this](std::span<const uint8_t> packet)
        {
            ++stats_.notify_count;

            if (packet.empty() || packet.size() > RawPacket::MAX_SIZE)
            {
                ++stats_.dropped_packets;
                return;
            }

            RawPacket pkt{};
            pkt.len = packet.size();
            std::copy(packet.begin(), packet.end(), pkt.data);

            {
                std::lock_guard lock(impl_->mtx);
                if (impl_->queue.size() >= QUEUE_CAPACITY)
                {
                    impl_->queue.pop_front();
                    ++stats_.dropped_packets;
                }
                impl_->queue.push_back(pkt);
            }
            impl_->cv.notify_one();
        });

    if (!subscribed)
    {
        std::cerr << "[session] Failed to subscribe to telemetry\n";
        impl_->stop.store(true);
        impl_->cv.notify_all();
        impl_->worker.join();
        adapter_.disconnect();
        return false;
    }

    std::cout << "[session] Streaming for " << cfg_.run_duration_s << " s"
              << (cfg_.target_ensembles
                      ? " (or " + std::to_string(cfg_.target_ensembles) +
                            " ensembles)"
                      : "")
              << "\n";

    const auto t_start = std::chrono::steady_clock::now();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        const double elapsed_s = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - t_start)
                                     .count();

        std::cout << "\r  notifies=" << stats_.notify_count
                  << "  ensembles=" << stats_.ensemble_count
                  << "  elapsed=" << static_cast<int>(elapsed_s) << "s   "
                  << std::flush;

        if (elapsed_s >= cfg_.run_duration_s)
            break;
        if (cfg_.target_ensembles > 0 &&
            stats_.ensemble_count >= cfg_.target_ensembles)
        {
            std::cout << "\n[session] Target reached (" << stats_.ensemble_count
                      << " ensembles)\n";
            break;
        }
        if (!adapter_.is_connected())
        {
            std::cout << "\n[session] Device disconnected\n";
            break;
        }
    }

    std::cout << "\n";

    impl_->stop.store(true);
    impl_->cv.notify_all();
    impl_->worker.join();

    const double total = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - t_start)
                             .count();

    stats_.run_time_s = total;
    stats_.avg_rate_hz =
        total > 0.0 ? static_cast<double>(stats_.notify_count) / total : 0.0;

    adapter_.disconnect();

    std::cout << "=== session summary ===\n"
              << "  run time   : " << stats_.run_time_s << " s\n"
              << "  notifies   : " << stats_.notify_count << "\n"
              << "  ensembles  : " << stats_.ensemble_count << "\n"
              << "  dropped    : " << stats_.dropped_packets << "\n"
              << "  dec errors : " << stats_.decode_errors << "\n"
              << "  avg rate   : " << stats_.avg_rate_hz << " Hz\n";

    return stats_.ensemble_count > 0;
}

} // namespace sf::session
