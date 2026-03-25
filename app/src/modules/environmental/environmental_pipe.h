/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ENVIRONMENTAL_PIPE_H
#define ENVIRONMENTAL_PIPE_H

#include <zephyr/kernel.h>
#include "environmental.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write environmental batch message to pipe
 *
 * Direct pipe write for environmental sensor batch data.
 * This bypasses zbus to reduce overhead for high-frequency sampling.
 *
 * @param msg Pointer to environmental message
 * @param timeout Timeout for write operation
 * @return 0 on success, negative error code on failure
 */
int environmental_pipe_write(const struct environmental_msg *msg, k_timeout_t timeout);

/**
 * @brief Read environmental batch message from pipe
 *
 * Direct pipe read for environmental sensor batch data.
 *
 * @param msg Pointer to buffer for environmental message
 * @param timeout Timeout for read operation
 * @return 0 on success, negative error code on failure
 */
int environmental_pipe_read(struct environmental_msg *msg, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* ENVIRONMENTAL_PIPE_H */
