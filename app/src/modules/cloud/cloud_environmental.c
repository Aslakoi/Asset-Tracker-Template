/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <net/nrf_cloud_coap.h>

#include "cloud_environmental.h"

LOG_MODULE_DECLARE(cloud, CONFIG_APP_CLOUD_LOG_LEVEL);

int cloud_environmental_send(const struct environmental_msg *env,
			     int64_t timestamp_ms,
			     bool confirmable)
{
	int err;

	if (!env || env->sample_count == 0) {
		return 0; /* Nothing to send */
	}

	/* Extract pressure if available (stored as int32_t Pa) */
	if (env->pressure_valid && env->pressure != 0) {
		err = nrf_cloud_coap_sensor_send(NRF_CLOUD_JSON_APPID_VAL_AIR_PRESS,
						 (double)env->pressure,
						 timestamp_ms,
						 confirmable);
		if (err) {
			LOG_ERR("Failed to send pressure data to cloud, error: %d", err);
			return err;
		}

		LOG_DBG("Environmental pressure data sent to cloud: P=%.1f Pa", 
			(double)env->pressure);
	}

	return 0;
}
