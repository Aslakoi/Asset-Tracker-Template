/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include "environmental_msgq.h"

/* Message queue for environmental batches.
 * Capacity: 64 messages with optimized fixed-point message structure
 * Optimizations:
 * - Queue slots: 128 * 20 / 10 = 256 (formula: old_slots * old_batch / new_batch)
 * - Message size: ~176 bytes (down from ~488 bytes)
 *   * float→int16_t for accel/gyro (18 bytes vs 36 bytes per sample)
 *   * single pressure per batch (4 bytes vs 20 bytes per sample)
 *   * uint16_t deltas for timestamps (20 bytes vs 80 bytes per batch)
 * - Total queue RAM: ~11 KB (down from ~62 KB)
 * - Buffering: ~20+ seconds at 5 Hz batch rate (1.4 batches/sec)
 */
K_MSGQ_DEFINE(environmental_msgq, sizeof(struct environmental_msg), 128, 4);

int environmental_msgq_write(const struct environmental_msg *msg, k_timeout_t timeout)
{
	return k_msgq_put(&environmental_msgq, (void *)msg, timeout);
}

int environmental_msgq_read(struct environmental_msg *msg, k_timeout_t timeout)
{
	return k_msgq_get(&environmental_msgq, msg, timeout);
}
