/**
 * @file smartfin_c_api.h
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Pure-C bridge over the Smartfin C++ processing pipeline.
 * @date 2026-05-14
 *
 * Intended for Swift consumption via a bridging header on watchOS/iOS.
 * CoreBluetooth owns the BLE layer; this library handles packet decoding
 * and offline signal processing.
 *
 * @code
 *   SF_Sink* sink = sf_sink_create();
 *   SF_Proc* proc = sf_proc_create();
 *
 *   // For each CoreBluetooth notification:
 *   sf_sink_push_packet(sink, bytes, length);
 *
 *   // After the session ends:
 *   size_t n = 0;
 *   SF_OrientedSample* result = sf_proc_run(proc, sink, &n);
 *   // ... consume result[0..n-1] ...
 *   sf_oriented_free(result);
 *
 *   sf_proc_destroy(proc);
 *   sf_sink_destroy(sink);
 * @endcode
 */

#ifndef SMARTFIN_C_API_H
#define SMARTFIN_C_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decoded temperature ensemble.
 */
typedef struct {
    uint32_t elapsed_ms; ///< Milliseconds since session start.
    float    temp_c;     ///< Water temperature in degrees Celsius.
    int      in_water;   ///< Non-zero when the fin detects water immersion.
} SF_Temp;

/**
 * @brief Decoded raw IMU ensemble.
 */
typedef struct {
    uint32_t elapsed_ms; ///< Milliseconds since session start.
    float    accel[3];   ///< Linear acceleration in m/s^2 [x, y, z].
    float    gyro[3];    ///< Angular rate in deg/s [x, y, z].
    float    mag[3];     ///< Magnetic field in uT [x, y, z].
} SF_Imu;

/**
 * @brief Decoded quaternion IMU ensemble.
 *
 * Extends @ref SF_Imu with a unit quaternion fused on-device.
 */
typedef struct {
    uint32_t elapsed_ms;           ///< Milliseconds since session start.
    float    accel[3];             ///< Linear acceleration in m/s^2 [x, y, z].
    float    gyro[3];              ///< Angular rate in deg/s [x, y, z].
    float    mag[3];               ///< Magnetic field in uT [x, y, z].
    float    q[4];                 ///< Unit quaternion [q0, q1, q2, q3].
    float    heading_accuracy_deg; ///< Estimated heading accuracy in degrees.
    int      quat_valid;           ///< Non-zero when accuracy is within threshold.
} SF_QuatImu;

/**
 * @brief World-frame oriented sample produced by the processing pipeline.
 */
typedef struct {
    uint32_t elapsed_ms;      ///< Milliseconds since session start.
    float    q[4];            ///< Orientation quaternion [q0, q1, q2, q3].
    float    accel_global[3]; ///< World-frame acceleration in m/s^2 [x, y, z].
} SF_OrientedSample;

/**
 * @brief Opaque sink handle.
 *
 * Accumulates decoded IMU, quaternion IMU, and temperature samples.
 * Create with @ref sf_sink_create; destroy with @ref sf_sink_destroy.
 */
typedef struct SF_Sink_ SF_Sink;

/**
 * @brief Create a new, empty sink.
 * @return Allocated sink, or @c NULL on failure.
 */
SF_Sink* sf_sink_create(void);

/**
 * @brief Destroy a sink. No-op if @p sink is @c NULL.
 * @param sink  Handle returned by @ref sf_sink_create.
 */
void sf_sink_destroy(SF_Sink* sink);

/**
 * @brief Decode one raw BLE notification payload and append results to @p sink.
 *
 * Unknown ensemble types are silently skipped.
 *
 * @param sink  Destination sink.
 * @param data  Raw bytes from a single CoreBluetooth notification.
 * @param len   Number of bytes in @p data.
 * @return      0 on success; -1 if the header is malformed or arguments are @c NULL.
 */
int sf_sink_push_packet(SF_Sink* sink, const uint8_t* data, size_t len);

/**
 * @brief Return the number of raw IMU samples buffered.
 * @param sink  Sink to query.
 * @return Number of raw IMU samples buffered, or 0 if @p sink is @c NULL.
 */
size_t sf_sink_imu_count(const SF_Sink* sink);

/**
 * @brief Return the number of quaternion IMU samples buffered.
 * @param sink  Sink to query.
 * @return Number of quaternion IMU samples buffered, or 0 if @p sink is @c NULL.
 */
size_t sf_sink_quat_imu_count(const SF_Sink* sink);

/**
 * @brief Return the number of temperature samples buffered.
 * @param sink  Sink to query.
 * @return Number of temperature samples buffered, or 0 if @p sink is @c NULL.
 */
size_t sf_sink_temp_count(const SF_Sink* sink);

/**
 * @brief Copy the raw IMU sample at @p index into @p out.
 *
 * No-op if @p sink or @p out is @c NULL, or @p index is out of range.
 *
 * @param sink   Source sink.
 * @param index  Zero-based sample index.
 * @param out    Destination struct.
 */
void sf_sink_get_imu(const SF_Sink* sink, size_t index, SF_Imu* out);

/**
 * @brief Copy the quaternion IMU sample at @p index into @p out.
 *
 * No-op if @p sink or @p out is @c NULL, or @p index is out of range.
 *
 * @param sink   Source sink.
 * @param index  Zero-based sample index.
 * @param out    Destination struct.
 */
void sf_sink_get_quat_imu(const SF_Sink* sink, size_t index, SF_QuatImu* out);

/**
 * @brief Copy the temperature sample at @p index into @p out.
 *
 * No-op if @p sink or @p out is @c NULL, or @p index is out of range.
 *
 * @param sink   Source sink.
 * @param index  Zero-based sample index.
 * @param out    Destination struct.
 */
void sf_sink_get_temp(const SF_Sink* sink, size_t index, SF_Temp* out);

/**
 * @brief Discard all buffered samples without destroying the sink.
 * @param sink  Sink to clear. No-op if @c NULL.
 */
void sf_sink_clear(SF_Sink* sink);

/**
 * @brief Opaque processor handle.
 *
 * Runs the orient and filter pipeline stages on buffered sink data.
 * Create with @ref sf_proc_create; destroy with @ref sf_proc_destroy.
 */
typedef struct SF_Proc_ SF_Proc;

/**
 * @brief Create a processor with default configuration.
 * @return Allocated processor, or @c NULL on failure.
 */
SF_Proc* sf_proc_create(void);

/**
 * @brief Destroy a processor. No-op if @p proc is @c NULL.
 * @param proc  Handle returned by @ref sf_proc_create.
 */
void sf_proc_destroy(SF_Proc* proc);

/**
 * @brief Run the processing pipeline on the samples currently buffered in @p sink.
 *
 * The returned array is heap-allocated and must be freed with @ref sf_oriented_free.
 *
 * @param proc       Processor handle.
 * @param sink       Populated sink from @ref sf_sink_push_packet calls.
 * @param out_count  Receives the number of elements in the returned array.
 * @return           Heap-allocated @ref SF_OrientedSample array, or @c NULL on
 *                   failure or when no oriented samples could be produced.
 */
SF_OrientedSample* sf_proc_run(SF_Proc* proc, SF_Sink* sink, size_t* out_count);

/**
 * @brief Free an array returned by @ref sf_proc_run. No-op if @p samples is @c NULL.
 * @param samples  Pointer returned by @ref sf_proc_run.
 */
void sf_oriented_free(SF_OrientedSample* samples);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SMARTFIN_C_API_H */
