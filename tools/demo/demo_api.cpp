/**
 * @file demo_api.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief C API implementation wrapping the processing pipeline for Python ctypes.
 * @date 2026-05-15
 */

#include "demo_api.h"

#include "proccessing/processor.hpp"
#include "proccessing/config.hpp"
#include "proccessing/welch/welch.hpp"
#include "filter/butterworth.hpp"
#include "filter/decimator.hpp"
#include "filter/filtfilt.hpp"
#include "pipeline/file_sink.hpp"
#include "protocol/ensemble_types.hpp"

#include <complex>
#include <vector>

extern "C" {

int sf_demo_nperseg(void)
{
    return CFG::welch_nperseg;
}

int sf_demo_orient(
    const uint32_t *elapsed_ms,
    const float    *accel_ms2,
    const float    *gyro_dps,
    const float    *mag_uT,
    int             n_in,
    uint32_t       *out_elapsed_ms,
    double         *out_q,
    double         *out_accel)
{
    std::vector<sf::protocol::DecodedImu> imu(n_in);
    for (int i = 0; i < n_in; ++i)
    {
        imu[i].elapsed_time_ms = elapsed_ms[i];
        imu[i].accel_ms2[0] = accel_ms2[i * 3 + 0];
        imu[i].accel_ms2[1] = accel_ms2[i * 3 + 1];
        imu[i].accel_ms2[2] = accel_ms2[i * 3 + 2];
        imu[i].gyro_dps[0]  = gyro_dps[i * 3 + 0];
        imu[i].gyro_dps[1]  = gyro_dps[i * 3 + 1];
        imu[i].gyro_dps[2]  = gyro_dps[i * 3 + 2];
        imu[i].mag_uT[0]    = mag_uT[i * 3 + 0];
        imu[i].mag_uT[1]    = mag_uT[i * 3 + 1];
        imu[i].mag_uT[2]    = mag_uT[i * 3 + 2];
    }

    sf::pipeline::RideData rd;
    rd.imu = std::move(imu);

    sf::proc::Processor proc;
    sf::proc::OrientedRide ride = proc.orient_ride(rd);

    const int n_out = static_cast<int>(ride.samples.size());
    for (int i = 0; i < n_out; ++i)
    {
        const auto &s      = ride.samples[i];
        out_elapsed_ms[i]  = s.elapsed_time_ms;
        out_q[i * 4 + 0]  = s.q.w;
        out_q[i * 4 + 1]  = s.q.x;
        out_q[i * 4 + 2]  = s.q.y;
        out_q[i * 4 + 3]  = s.q.z;
        out_accel[i * 3 + 0] = s.accel_global.x;
        out_accel[i * 3 + 1] = s.accel_global.y;
        out_accel[i * 3 + 2] = s.accel_global.z;
    }
    return n_out;
}

int sf_demo_decimate(
    const uint32_t *elapsed_ms,
    const double   *accel,
    int             n_in,
    uint32_t       *out_elapsed_ms,
    double         *out_accel)
{
    sf::filter::Decimator dx, dy, dz;
    int n_out = 0;

    for (int i = 0; i < n_in; ++i)
    {
        double x = 0.0, y = 0.0, z = 0.0;
        const bool ok = dx.push(accel[i * 3 + 0], x)
                      & dy.push(accel[i * 3 + 1], y)
                      & dz.push(accel[i * 3 + 2], z);
        if (ok)
        {
            out_elapsed_ms[n_out]       = elapsed_ms[i];
            out_accel[n_out * 3 + 0]   = x;
            out_accel[n_out * 3 + 1]   = y;
            out_accel[n_out * 3 + 2]   = z;
            ++n_out;
        }
    }
    return n_out;
}

int sf_demo_bandpass(
    const uint32_t *elapsed_ms,
    double         *accel,
    int             n)
{
    if (n <= 9)
        return -1;

    const double duration_s =
        (elapsed_ms[n - 1] - elapsed_ms[0]) / 1000.0;
    if (duration_s <= 0.0)
        return -1;

    const double fs = (n - 1) / duration_s;

    auto hp = sf::filter::butterworth(
        4, CFG::bandpass_lo_hz, fs, sf::filter::FilterType::Highpass);
    auto lp = sf::filter::butterworth(
        4, CFG::bandpass_hi_hz, fs, sf::filter::FilterType::Lowpass);

    std::vector<double> ax(n), ay(n), az(n);
    for (int i = 0; i < n; ++i)
    {
        ax[i] = accel[i * 3 + 0];
        ay[i] = accel[i * 3 + 1];
        az[i] = accel[i * 3 + 2];
    }

    ax = sf::filter::filtfilt(hp, ax);
    ay = sf::filter::filtfilt(hp, ay);
    az = sf::filter::filtfilt(hp, az);
    ax = sf::filter::filtfilt(lp, ax);
    ay = sf::filter::filtfilt(lp, ay);
    az = sf::filter::filtfilt(lp, az);

    for (int i = 0; i < n; ++i)
    {
        accel[i * 3 + 0] = ax[i];
        accel[i * 3 + 1] = ay[i];
        accel[i * 3 + 2] = az[i];
    }
    return 0;
}

void sf_demo_fft(double *re, double *im)
{
    const int n = CFG::welch_nperseg;
    std::vector<std::complex<double>> x(n);
    for (int i = 0; i < n; ++i)
        x[i] = {re[i], im[i]};

    sf::welch::fft(x);

    for (int i = 0; i < n; ++i)
    {
        re[i] = x[i].real();
        im[i] = x[i].imag();
    }
}

void sf_demo_real_dft_mag_sq(const double *signal, double *mag_sq)
{
    const int n = CFG::welch_nperseg;
    const std::vector<double> sig(signal, signal + n);
    std::vector<double> out;
    sf::welch::real_dft_mag_sq(sig, out);
    for (int i = 0; i < static_cast<int>(out.size()); ++i)
        mag_sq[i] = out[i];
}

} // extern "C"
