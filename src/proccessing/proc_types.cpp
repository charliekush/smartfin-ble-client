/**
 * @file proc_types.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief OrientedSample constructors, OrientedRide::sort (Timsort),
 *        orient_from_imu, and orient_from_quat_imu.
 * @date 2026-05-12
 */

#include "proc_types.hpp"
#include "pipeline/file_sink.hpp"
#include "proccessing/AHRS/madgwick_ahrs.hpp"
#include "proccessing/math/constants.hpp"
#include "proccessing/sort/timsort.hpp"

#include <algorithm>

namespace sf::proc
{

    OrientedSample::OrientedSample(const sf::protocol::DecodedQuatImu &s)
        : elapsed_time_ms(s.elapsed_time_ms), q(s.q[0], s.q[1], s.q[2], s.q[3])
    {
        const math3d::Vec3 accel_g{
            static_cast<double>(s.accel_ms2[0]) / G0,
            static_cast<double>(s.accel_ms2[1]) / G0,
            static_cast<double>(s.accel_ms2[2]) / G0,
        };
        const math3d::Vec3 gravity = math3d::gravityFromQuaternion(q);
        const math3d::Vec3 accel_0g = accel_g - gravity;
        accel_global = math3d::rotateVector(q, accel_0g);
    }

    OrientedRide::OrientedRide(sf::pipeline::RideData &data)
    {
        auto by_time = [](const auto &a, const auto &b)
        {
            return a.elapsed_time_ms < b.elapsed_time_ms;
        };
        timsort(data.imu, by_time);
        timsort(data.quat_imu, by_time);

        std::vector<OrientedSample>
            imu_samples = orient_from_imu(
                data.imu);
        std::vector<OrientedSample> quat_imu_samples = orient_from_quat_imu(
            data.quat_imu);
        samples.reserve(imu_samples.size() + quat_imu_samples.size());
        
        std::vector<OrientedSample>::iterator imu_itr = imu_samples.begin();
        std::vector<OrientedSample>::iterator quat_itr = 
                                                    quat_imu_samples.begin();
        while(samples.size() < imu_samples.size() + quat_imu_samples.size())
        {
            if(imu_itr == imu_samples.end())
                samples.insert(samples.end(), quat_itr, quat_imu_samples.end());
            else if(imu_itr == imu_samples.end())
                samples.insert(samples.end(), quat_itr, quat_imu_samples.end());
            else
            {
                if ((*imu_itr).elapsed_time_ms < (*quat_itr).elapsed_time_ms)
                    samples.emplace_back(*imu_itr++);
                else
                    samples.emplace_back(*quat_itr++);
            }
        }
    }

    std::vector<OrientedSample> OrientedRide::orient_from_imu(
        const std::vector<sf::protocol::DecodedImu> &decoded_samples)
    {
        sf::ahrs::AHRS filter;
        std::vector<OrientedSample> samples;
        samples.reserve(decoded_samples.size());

        for (const auto &imu : decoded_samples)
        {
            const auto out = filter.update(imu);
            if (!out.updated)
                continue;

            OrientedSample s;
            s.elapsed_time_ms = imu.elapsed_time_ms;
            s.q = out.q;
            s.accel_global = out.accel_global;
            samples.emplace_back(s);
        }

        return samples;
    }

    std::vector<OrientedSample> OrientedRide::orient_from_quat_imu(
        const std::vector<sf::protocol::DecodedQuatImu> &decoded_samples)
    {

        std::vector<OrientedSample> samples;
        samples.reserve(decoded_samples.size());

        for (const auto &q_imu : decoded_samples)
            samples.emplace_back(q_imu);

        return samples;
    }

} // namespace sf::proc
