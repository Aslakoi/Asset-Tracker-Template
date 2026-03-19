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

#define SAMPLES_PER_BATCH 30  /* Publish every x samples to batch and reduce zbus load */

/* Channels provided by this module */
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

struct environmental_msg {
	enum environmental_msg_type type;

	/** Number of samples currently in this batch message (0 to SAMPLES_PER_BATCH) */
	uint8_t sample_count;

	/** Contains the current pressure in Pa (from BME680). */
	float pressure[SAMPLES_PER_BATCH];

	/** Contains the current acceleration values in g. */
	float accel_hp[3][SAMPLES_PER_BATCH];

	/** Contains the current gyroscope values in dps. */
	float gyro_hp[3][SAMPLES_PER_BATCH];

	/** Contains the current low-power acceleration values in g. */
	float accel_lp[3][SAMPLES_PER_BATCH];

	/** Timestamp when the sample was taken in milliseconds.
	 *  This is either:
	 * - Unix time in milliseconds if the system clock was synchronized at sampling time, or
	 * - Uptime in milliseconds if the system clock was not synchronized at sampling time.
	 * Only valid for ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE events.
	 */
	int64_t timestamp[SAMPLES_PER_BATCH];
};

#define MSG_TO_ENVIRONMENTAL_MSG(_msg)	(*(const struct environmental_msg *)_msg)


#ifdef __cplusplus
}
#endif

#endif /* _ENVIRONMENTAL_H_ */
