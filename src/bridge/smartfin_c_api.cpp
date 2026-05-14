/**
 * @file smartfin_c_api.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Implementation of the pure-C bridge over the Smartfin C++ pipeline.
 * @date 2026-05-14
 */

#include "bridge/smartfin_c_api.h"

#include "pipeline/file_sink.hpp"
#include "pipeline/sample_sink.hpp"
#include "proccessing/proc_types.hpp"
#include "proccessing/processor.hpp"
#include "protocol/ensemble_decoder.hpp"
#include "protocol/ensemble_types.hpp"

#include <cstring>
#include <new>
#include <span>
#include <type_traits>
#include <variant>
#include <vector>

namespace
{

/**
 * @brief ISampleSink that buffers all three ensemble types for the C bridge.
 *
 * BufferSink only stores IMU and temperature; this variant also captures
 * DecodedQuatImu, which Processor::process requires.
 */
struct BridgeSink final : sf::pipeline::ISampleSink
{
    std::vector<sf::protocol::DecodedImu> imu; ///< Raw IMU samples.
    std::vector<sf::protocol::DecodedQuatImu>
        quat_imu;                                 ///< Quaternion IMU samples.
    std::vector<sf::protocol::DecodedTemp> temps; ///< Temperature samples.

    void on_imu(const sf::protocol::DecodedImu &s) override
    {
        imu.push_back(s);
    }
    void on_quat_imu(const sf::protocol::DecodedQuatImu &s) override
    {
        quat_imu.push_back(s);
    }
    void on_temperature(const sf::protocol::DecodedTemp &s) override
    {
        temps.push_back(s);
    }
    void on_fw_version(const sf::protocol::DecodedFwVersion &) override
    {
    }

    void clear()
    {
        imu.clear();
        quat_imu.clear();
        temps.clear();
    }
};

} // namespace

/**
 * @brief Opaque sink implementation backing the @c SF_Sink C handle.
 */
struct SF_Sink_
{
    BridgeSink sink;
};

/**
 * @brief Opaque processor implementation backing the @c SF_Proc C handle.
 */
struct SF_Proc_
{
    sf::proc::Processor proc;
};

SF_Sink *sf_sink_create()
{
    return new (std::nothrow) SF_Sink_{};
}

void sf_sink_destroy(SF_Sink *s)
{
    delete s;
}

int sf_sink_push_packet(SF_Sink *s, const uint8_t *data, size_t len)
{
    if (!s || !data)
        return -1;

    std::vector<sf::protocol::DecodedEnsemble> ensembles;
    if (!sf::protocol::decode_packet(std::span{data, len}, ensembles))
        return -1;

    for (const auto &ensemble : ensembles)
    {
        std::visit(
            [&](const auto &e)
            {
                using T = std::decay_t<decltype(e)>;
                if constexpr (std::is_same_v<T, sf::protocol::DecodedImu>)
                    s->sink.on_imu(e);
                else if constexpr (std::is_same_v<T,
                                                  sf::protocol::DecodedQuatImu>)
                    s->sink.on_quat_imu(e);
                else if constexpr (std::is_same_v<T, sf::protocol::DecodedTemp>)
                    s->sink.on_temperature(e);
                else if constexpr (std::is_same_v<
                                       T, sf::protocol::DecodedFwVersion>)
                    s->sink.on_fw_version(e);
            },
            ensemble);
    }

    return 0;
}

size_t sf_sink_imu_count(const SF_Sink *s)
{
    return s ? s->sink.imu.size() : 0;
}
size_t sf_sink_quat_imu_count(const SF_Sink *s)
{
    return s ? s->sink.quat_imu.size() : 0;
}
size_t sf_sink_temp_count(const SF_Sink *s)
{
    return s ? s->sink.temps.size() : 0;
}

void sf_sink_get_imu(const SF_Sink *s, size_t i, SF_Imu *out)
{
    if (!s || !out || i >= s->sink.imu.size())
        return;
    const auto &src = s->sink.imu[i];
    out->elapsed_ms = src.elapsed_time_ms;
    std::memcpy(out->accel, src.accel_ms2, sizeof(out->accel));
    std::memcpy(out->gyro, src.gyro_dps, sizeof(out->gyro));
    std::memcpy(out->mag, src.mag_uT, sizeof(out->mag));
}

void sf_sink_get_quat_imu(const SF_Sink *s, size_t i, SF_QuatImu *out)
{
    if (!s || !out || i >= s->sink.quat_imu.size())
        return;
    const auto &src = s->sink.quat_imu[i];
    out->elapsed_ms = src.elapsed_time_ms;
    out->heading_accuracy_deg = src.heading_accuracy_deg;
    out->quat_valid = src.quat_valid ? 1 : 0;
    std::memcpy(out->accel, src.accel_ms2, sizeof(out->accel));
    std::memcpy(out->gyro, src.gyro_dps, sizeof(out->gyro));
    std::memcpy(out->mag, src.mag_uT, sizeof(out->mag));
    std::memcpy(out->q, src.q, sizeof(out->q));
}

void sf_sink_get_temp(const SF_Sink *s, size_t i, SF_Temp *out)
{
    if (!s || !out || i >= s->sink.temps.size())
        return;
    const auto &src = s->sink.temps[i];
    out->elapsed_ms = src.elapsed_time_ms;
    out->temp_c = src.temp_c;
    out->in_water = src.in_water ? 1 : 0;
}

void sf_sink_clear(SF_Sink *s)
{
    if (s)
        s->sink.clear();
}

SF_Proc *sf_proc_create()
{
    return new (std::nothrow) SF_Proc_{};
}

void sf_proc_destroy(SF_Proc *p)
{
    delete p;
}

SF_OrientedSample *sf_proc_run(SF_Proc *p, SF_Sink *s, size_t *out_count)
{
    if (!p || !s || !out_count)
        return nullptr;
    *out_count = 0;

    sf::pipeline::RideData data;
    data.imu = s->sink.imu;
    data.quat_imu = s->sink.quat_imu;
    data.temps = s->sink.temps;

    sf::proc::ProcessedRide result = p->proc.process(data);
    const auto &samples = result.ride.samples;
    if (samples.empty())
        return nullptr;

    auto *out = new (std::nothrow) SF_OrientedSample[samples.size()];
    if (!out)
        return nullptr;

    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto &src = samples[i];
        out[i].elapsed_ms = src.elapsed_time_ms;
        out[i].q[0] = static_cast<float>(src.q.w);
        out[i].q[1] = static_cast<float>(src.q.x);
        out[i].q[2] = static_cast<float>(src.q.y);
        out[i].q[3] = static_cast<float>(src.q.z);
        out[i].accel_global[0] = static_cast<float>(src.accel_global.x);
        out[i].accel_global[1] = static_cast<float>(src.accel_global.y);
        out[i].accel_global[2] = static_cast<float>(src.accel_global.z);
    }

    *out_count = samples.size();
    return out;
}

void sf_oriented_free(SF_OrientedSample *samples)
{
    delete[] samples;
}
