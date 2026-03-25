/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _ENVIRONMENTAL_H_
#define _ENVIRONMENTAL_H_

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SAMPLES_PER_BATCH 10  /* Publish every x samples to batch and reduce messaging load */


/* Environmental data flows via dedicated message queue (environmental_msgq.h) 
 * with no zbus pub/sub. Channel below is only for storage_data compatibility.
 */
ZBUS_CHAN_DECLARE(
	environmental_chan
);

enum environmental_msg_type {
	/* Output message types */

	/* Response message to a request for current environmental sensor values.
	 * The sampled values are found in the respective fields of the message structure.
	 */
	ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE = 0x1,

	/* Input message types */

	/* Request to sample the current environmental sensor values.
	 * The response is sent as a ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE message.
	 */
	ENVIRONMENTAL_SENSOR_SAMPLE_REQUEST,
};

/* Fixed-point scaling: store float as int by multiplying by scale factor */
#define ACCEL_SCALE 1000        /* ±16g range at 0.001 g/LSB */
#define GYRO_SCALE 100          /* ±2000 dps range at 0.01 dps/LSB */
#define PRESSURE_SCALE 1        /* Pa as int32_t */

struct environmental_msg {
	enum environmental_msg_type type;

	/** Number of samples currently in this batch message (0 to SAMPLES_PER_BATCH) */
	uint8_t sample_count;

	/** Latest pressure in Pa (updated at 0.1 Hz, sparse).
	 *  Store once per batch instead of per-sample to save RAM.
	 */
	int32_t pressure;
	bool pressure_valid;

	/** Timestamp of first sample in batch (milliseconds: unix time or uptime) */
	uint32_t batch_timestamp_ms;

	/** Per-sample time deltas from batch_timestamp_ms in milliseconds (assuming ~10 Hz = ~100 ms apart) */
	uint16_t timestamp_delta_ms[SAMPLES_PER_BATCH];

	/** Contains the current acceleration values in fixed-point (value * 1000 = g).
	 *  Fixed-point int16_t: range ±32.767g, resolution 0.001g
	 */
	int16_t accel_hp[3][SAMPLES_PER_BATCH];

	/** Contains the current gyroscope values in fixed-point (value * 100 = dps).
	 *  Fixed-point int16_t: range ±327.67 dps, resolution 0.01 dps
	 */
	int16_t gyro_hp[3][SAMPLES_PER_BATCH];

	/** Contains the current low-power acceleration values in fixed-point (value * 1000 = g).
	 *  Fixed-point int16_t: range ±32.767g, resolution 0.001g
	 */
	int16_t accel_lp[3][SAMPLES_PER_BATCH];
};

#define MSG_TO_ENVIRONMENTAL_MSG(_msg)	(*(const struct environmental_msg *)_msg)


#ifdef __cplusplus
}
#endif

#endif /* _ENVIRONMENTAL_H_ */
