/**
 * @file processor.cpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Top-level processing pipeline: orient → filter → welch.
 * @date 2026-05-12
 */

#include "proccessing/processor.hpp"

namespace sf::proc {

Processor::Processor(const ProcessorConfig& cfg) : cfg_(cfg) {}

ProcessedRide Processor::process(sf::pipeline::RideData &data)
{
    ProcessedRide result;

    result.ride = orient_ride(data);
    // filter(result.ride);
    // welch(result.ride);

    return result;
}

} // namespace sf::proc
