/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ENVIRONMENTAL_MSGQ_H
#define ENVIRONMENTAL_MSGQ_H

#include <zephyr/kernel.h>
#include "environmental.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write environmental batch message to message queue
 *
 * Atomically enqueues an environmental sensor batch message.
 * Uses K_MSGQ for guaranteed message boundaries and alignment.
 *
 * @param msg Pointer to environmental message
 * @param timeout Timeout for write operation
 * @return 0 on success, -ENOMSG if queue full (with K_NO_WAIT), negative error code on failure
 */
int environmental_msgq_write(const struct environmental_msg *msg, k_timeout_t timeout);

/**
 * @brief Read environmental batch message from message queue
 *
 * Atomically dequeues an environmental sensor batch message.
 * Guaranteed to return complete, aligned messages.
 *
 * @param msg Pointer to buffer for environmental message
 * @param timeout Timeout for read operation (K_NO_WAIT for non-blocking)
 * @return 0 on success, -ENOMSG if queue empty (with K_NO_WAIT), negative error code on failure
 */
int environmental_msgq_read(struct environmental_msg *msg, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* ENVIRONMENTAL_MSGQ_H */
