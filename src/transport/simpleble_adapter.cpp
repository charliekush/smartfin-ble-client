/**
 * @file simpleble_adapter.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief SimpleBLE-backed IBleAdapter implementation.
 * @date 2026-04-30
 *
 */

#include "simpleble_adapter.hpp"

#include <chrono>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <span>
#include <thread>
#include <utility>

namespace sf::transport
{

SimpleBleAdapter::~SimpleBleAdapter()
{
    disconnect();
}

bool SimpleBleAdapter::scan_and_connect(const std::string &device_name,
                                        int timeout_s)
{
    if (!SimpleBLE::Adapter::bluetooth_enabled())
    {
        std::cerr << "[ble] Bluetooth is not enabled\n";
        return false;
    }

    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty())
    {
        std::cerr << "[ble] No Bluetooth adapters found\n";
        return false;
    }

    // Store the adapter as a member so CoreBluetooth's CBCentralManager stays
    // alive for the full lifetime of the connection (not just this call).
    adapter_ = std::move(adapters[0]);
    std::cout << "[ble] Using adapter: " << adapter_->identifier() << "\n";

    std::mutex mtx;
    std::condition_variable cv;
    std::optional<SimpleBLE::Peripheral> found;

    adapter_->set_callback_on_scan_found(
        [&](SimpleBLE::Peripheral p)
        {
            if (p.identifier() == device_name)
            {
                std::lock_guard lock(mtx);
                if (!found)
                {
                    found = std::move(p);
                    cv.notify_one();
                }
            }
        });

    std::cout << "[ble] Scanning for \"" << device_name
              << "\" (timeout=" << timeout_s << "s)...\n";
    adapter_->scan_start();

    {
        std::unique_lock lock(mtx);
        cv.wait_for(lock, std::chrono::seconds(timeout_s),
                    [&] { return found.has_value(); });
    }

    adapter_->scan_stop();

    if (!found)
    {
        std::cerr << "[ble] Device \"" << device_name << "\" not found\n";
        return false;
    }

    peripheral_ = std::move(found);
    std::cout << "[ble] Found: name=\"" << peripheral_->identifier()
              << "\" addr=" << peripheral_->address()
              << " rssi=" << peripheral_->rssi() << " dBm\n";

    // Brief pause: CoreBluetooth on macOS needs a moment after scan_stop
    // before the peripheral is ready to accept a connection request.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    constexpr int MAX_CONNECT_ATTEMPTS = 3;
    for (int attempt = 1; attempt <= MAX_CONNECT_ATTEMPTS; ++attempt)
    {
        std::cout << "[ble] Connecting (attempt " << attempt << "/"
                  << MAX_CONNECT_ATTEMPTS << ")...\n";
        try
        {
            peripheral_->connect();
            std::cout << "[ble] Connected  mtu=" << peripheral_->mtu() << "\n";
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[ble] connect() threw: " << e.what() << "\n";
            if (attempt < MAX_CONNECT_ATTEMPTS)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(500 * attempt));
            }
        }
    }

    peripheral_.reset();
    return false;
}

bool SimpleBleAdapter::subscribe_notify(const std::string &service_uuid,
                                        const std::string &char_uuid,
                                        NotifyCallback callback)
{
    if (!peripheral_ || !peripheral_->is_connected())
    {
        return false;
    }
    try
    {
        peripheral_->notify(
            service_uuid, char_uuid,
            [cb = std::move(callback)](SimpleBLE::ByteArray payload)
            { cb(std::span<const uint8_t>(payload.data(), payload.size())); });
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ble] notify subscription failed: " << e.what() << "\n";
        return false;
    }
}

bool SimpleBleAdapter::write_control(const std::string &service_uuid,
                                     const std::string &char_uuid,
                                     const std::vector<uint8_t> &payload)
{
    if (!peripheral_ || !peripheral_->is_connected())
    {
        return false;
    }
    try
    {
        SimpleBLE::ByteArray data(payload.begin(), payload.end());
        peripheral_->write_command(service_uuid, char_uuid, data);
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ble] write_control failed: " << e.what() << "\n";
        return false;
    }
}

void SimpleBleAdapter::disconnect()
{
    if (peripheral_ && peripheral_->is_connected())
    {
        try
        {
            peripheral_->disconnect();
        }
        catch (...)
        {
        }
    }
    peripheral_.reset();
}

bool SimpleBleAdapter::is_connected()
{
    return peripheral_.has_value() && peripheral_->is_connected();
}

uint16_t SimpleBleAdapter::mtu()
{
    if (!peripheral_)
        return 0;
    return peripheral_->mtu();
}

} // namespace sf::transport
