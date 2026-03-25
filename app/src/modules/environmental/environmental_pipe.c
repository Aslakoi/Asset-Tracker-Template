/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "environmental_pipe.h"

/* Pipe for streaming environmental batches to storage.
 * Sized for ~2.4 seconds of high-frequency samples at 5 Hz batch rate:
 * 5 batches/sec * 2.4 sec ≈ 12 messages.
 * Combined with 10-message drain loop in storage, provides good buffering
 * while remaining within RAM constraints (zbus net_buf pool reduced to 32).
 */
#define ENV_PIPE_CAPACITY (sizeof(struct environmental_msg) * 12)
K_PIPE_DEFINE(environmental_pipe, ENV_PIPE_CAPACITY, 4);

/*
 * Write all bytes to pipe. The provided timeout is reused for each attempt to
 * keep logic simple and predictable. Loop until full structure is transferred.
 * Returns 0 on success, negative error code on failure.
 */
int environmental_pipe_write(const struct environmental_msg *msg, k_timeout_t timeout)
{
	size_t written = 0;
	const uint8_t *buf = (const uint8_t *)msg;
	size_t len = sizeof(struct environmental_msg);

	while (written < len) {
		int ret = k_pipe_write(&environmental_pipe, (char *)(buf + written),
				       len - written, K_NO_WAIT);
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			/* No data written, apply timeout */
			if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
				return -EAGAIN;
			}
			k_sleep(K_MSEC(1));
			continue;
		}

		written += (size_t)ret;
	}

	return 0;
}

/*
 * Read exact number of bytes from pipe. Loop until full structure is transferred or timeout.
 * With K_NO_WAIT: returns immediately if no data available.
 * With K_MSEC(N): retries for up to N milliseconds if getting partial data.
 * Returns 0 on success, -EAGAIN if timeout, negative error code on failure.
 */
int environmental_pipe_read(struct environmental_msg *msg, k_timeout_t timeout)
{
	size_t read_total = 0;
	uint8_t *buf = (uint8_t *)msg;
	size_t len = sizeof(struct environmental_msg);
	int retry_count = 0;
	const int max_retries = 10;  /* Limit retries to 10 attempts to avoid infinite loops */

	while (read_total < len) {
		int ret = k_pipe_read(&environmental_pipe, buf + read_total,
				      len - read_total, K_NO_WAIT);
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			/* No data available */
			if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
				return -EAGAIN;
			}
			
			if (retry_count >= max_retries) {
				/*Timeout: tried max_retries times but couldn't complete read */
				return -EAGAIN;
			}
			
			retry_count++;
			k_sleep(K_MSEC(1));
			continue;
		}

		read_total += (size_t)ret;
		retry_count = 0;  /* Reset retry counter on successful read */
	}

	return 0;
}
