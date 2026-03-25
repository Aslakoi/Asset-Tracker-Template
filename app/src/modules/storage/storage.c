/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/smf.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/sys/iterable_sections.h>
#include <string.h>
#include <stdint.h>

#include "storage.h"
#include "storage_backend.h"
#include "storage_data_types.h"
#include "app_common.h"
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

#ifdef CONFIG_APP_POWER
#include "power.h"
#endif
#ifdef CONFIG_APP_ENVIRONMENTAL
#include "environmental.h"
#include "environmental_msgq.h"
#endif
#ifdef CONFIG_APP_LOCATION
#include "location.h"
#endif

/* Register log module */
LOG_MODULE_REGISTER(storage, CONFIG_APP_STORAGE_LOG_LEVEL);

/* Timeout for batch session activity (to prevent stuck sessions) */
#define STORAGE_SESSION_TIMEOUT_SECONDS		CONFIG_APP_STORAGE_SESSION_TIMEOUT_SECONDS

/* Register zbus subscriber */
ZBUS_MSG_SUBSCRIBER_DEFINE(storage_subscriber);

/* Ensure that the watchdog timeout is greater than the message processing timeout
 * and the watchdog timeout is greater than the watchdog margin.
 */
BUILD_ASSERT(CONFIG_APP_STORAGE_WATCHDOG_TIMEOUT_SECONDS >
	     CONFIG_APP_STORAGE_MSG_PROCESSING_TIMEOUT_SECONDS);

/* Calculate the maximum message size from the list of channels */

/* Create channel list from DATA_SOURCE_LIST.
 * This is used to calculate the maximum message size and to add observers.
 * The DATA_SOURCE_LIST macro is defined in `storage_data_types.h`.
 */

/* Calculate maximum size needed for any message type */
#define STORAGE_MSG_UNION_MEMBER(_name, _chan, _msg_type, ...) _msg_type _name##_msg_member;

#define STORAGE_MAX_MSG_SIZE_FROM_LIST(_DATA_SOURCE_LIST_LIST) \
	sizeof(union {_DATA_SOURCE_LIST_LIST(STORAGE_MSG_UNION_MEMBER)})

/* Use the larger of: largest message type or storage_msg struct */
#define MAX_MSG_SIZE	MAX(STORAGE_MAX_MSG_SIZE_FROM_LIST(DATA_SOURCE_LIST), \
			    sizeof(struct storage_msg))

/* Private storage channel message types */
enum priv_storage_msg {
	STORAGE_BATCH_SESSION_TIMEOUT,
};

/* Add storage_subscriber as observer to each enabled channel.
 * The data source list is defined in `storage_data_types.h`.
 * Environmental channel is excluded since environmental data uses a dedicated pipe.
 */
#define ADD_OBSERVERS(_n, _chan, _t, _dt, _c, _e) \
	ZBUS_CHAN_ADD_OBS(_chan, storage_subscriber, 0);

/* Register observers for power and location channels only.
 * Environmental channel is not subscribed here since it uses a dedicated pipe.
 */
#ifdef CONFIG_APP_POWER
ZBUS_CHAN_ADD_OBS(power_chan, storage_subscriber, 0);
#endif /* CONFIG_APP_POWER */

#ifdef CONFIG_APP_LOCATION
ZBUS_CHAN_ADD_OBS(location_chan, storage_subscriber, 0);
#endif /* CONFIG_APP_LOCATION */

/* Create the storage channel */
ZBUS_CHAN_DEFINE(storage_chan,
		 struct storage_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

/* Create the storage data channel */
ZBUS_CHAN_DEFINE(storage_data_chan,
		 struct storage_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

/* Create private storage channel for internal messaging */
ZBUS_CHAN_DEFINE(priv_storage_chan,
		 enum priv_storage_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS(storage_subscriber),
		 STORAGE_BATCH_SESSION_TIMEOUT
);

/* Forward declarations of state handlers */
static void state_running_entry(void *o);
static enum smf_state_result state_running_run(void *o);

static enum smf_state_result state_buffer_idle_run(void *o);

static void state_buffer_pipe_active_entry(void *o);
static enum smf_state_result state_buffer_pipe_active_run(void *o);
static void state_buffer_pipe_active_exit(void *o);

/*Tracking number of bytes written to flash*/
static size_t bytes_written = 0;
bool storage_full = false;
static size_t max_bytes = 0x1880000; //Roughly 10% margin (LittleFS has overhead)

/* Environmental dedicated fast-path storage.
 * Drains multiple messages from msgq and writes them in a single filesystem operation.
 * This bypasses the generic backend's header churn and per-message overhead.
 */
#ifdef CONFIG_APP_ENVIRONMENTAL
#define ENV_STREAM_FILE_PATH "/att_storage/ENVIRONMENTAL_STREAM.bin"
#define ENV_BULK_WRITE_MAX_RECORDS 8

struct env_stream_state {
	bool initialized;
	uint32_t records_written;
	size_t file_offset;
};

static struct env_stream_state env_stream;
#endif /* CONFIG_APP_ENVIRONMENTAL */

/* Storage pipe for streaming data to consumers */
K_PIPE_DEFINE(storage_pipe, CONFIG_APP_STORAGE_BATCH_BUFFER_SIZE, 4);

/* Storage pipe item header (compact, fixed-width) */
struct storage_pipe_header {
	uint8_t type;
	uint16_t data_size;
};

/* Pipe session tracking */
struct pipe_session {
	uint32_t session_id;
	size_t total_items;
	size_t items_sent;
	bool more_data;
};

/* Storage module state object */
struct storage_state {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last channel type that a message was received on */
	const struct zbus_channel *chan;

	/* Last received message */
	uint8_t msg_buf[MAX_MSG_SIZE];

	/* Current batch session state */
	struct pipe_session current_session;

	/* Session timeout work */
	struct k_work_delayable session_timeout_work;

	/* Buffer threshold limit */
	uint32_t buffer_threshold_limit;

	/* Track per-type threshold crossing state to prevent repeated notifications */
	bool threshold_notified[STORAGE_DATA_TYPE_COUNT];
};

/* Delayable work for session timeout */
static void session_timeout_work_fn(struct k_work *work);

/* Defining the storage module states */
enum storage_module_state {
	STATE_RUNNING,
	STATE_BUFFER_IDLE,
	STATE_BUFFER_PIPE_ACTIVE,
};

/* Construct state table */
static const struct smf_state states[] = {
	[STATE_RUNNING] =
		SMF_CREATE_STATE(state_running_entry, state_running_run, NULL,
				 NULL, /* No parent state */
				 &states[STATE_BUFFER_IDLE]), /* Initial transition */
	[STATE_BUFFER_IDLE] =
		SMF_CREATE_STATE(NULL, state_buffer_idle_run, NULL,
				 &states[STATE_RUNNING],
				 NULL),
	[STATE_BUFFER_PIPE_ACTIVE] =
		SMF_CREATE_STATE(state_buffer_pipe_active_entry, state_buffer_pipe_active_run,
				 state_buffer_pipe_active_exit,
				 &states[STATE_RUNNING],
				 NULL),
};

/* Static helper functions */
static void task_wdt_callback(int channel_id, void *user_data)
{
	LOG_ERR("Watchdog expired, Channel: %d, Thread: %s",
		channel_id, k_thread_name_get((k_tid_t)user_data));

	SEND_FATAL_ERROR_WATCHDOG_TIMEOUT();
}

static void session_timeout_work_fn(struct k_work *work)
{
	int err;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct storage_state *state = CONTAINER_OF(dwork, struct storage_state,
						   session_timeout_work);
	enum priv_storage_msg msg = STORAGE_BATCH_SESSION_TIMEOUT;

	if (state->current_session.session_id == 0) {
		LOG_WRN("Session timeout fired but no active session");

		__ASSERT_NO_MSG(false);

		return;
	}

	LOG_WRN("Session timeout: closing session 0x%X", state->current_session.session_id);

	err = zbus_chan_pub(&priv_storage_chan, &msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("Failed to publish session timeout message: %d", err);
		/* If we fail to publish, we can't do much else from work queue context */
	}
}

/*
 * Write all bytes to pipe. The provided timeout is reused for each attempt to
 * keep logic simple and predictable. The helper is needed because
 * k_pipe_write() does not guarantee that all bytes will be written. Returns
 * number of bytes written if successful, negative error code if failed.
 */
static int pipe_write_all(struct k_pipe *pipe,
	const uint8_t *buf,
	size_t len,
	k_timeout_t timeout)
{
	int ret;
	size_t written_total = 0;

	while (written_total < len) {
		ret = k_pipe_write(pipe, buf + written_total, len - written_total, timeout);
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return -EAGAIN;
		}

		written_total += (size_t)ret;
	}

	return (int)written_total;
}

/*
 * Read exact number of bytes from pipe. The provided timeout is reused for each
 * attempt to keep logic simple and predictable.
 * Returns number of bytes read if successful, negative error code if failed.
 */
static int pipe_read_exact(struct k_pipe *pipe,
	uint8_t *buf,
	size_t len,
	k_timeout_t timeout)
{
	int ret;
	size_t read_total = 0;

	while (read_total < len) {
		ret = k_pipe_read(pipe, buf + read_total, len - read_total, timeout);
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return -EAGAIN;
		}

		read_total += (size_t)ret;
	}

	return (int)read_total;
}

static void check_and_notify_buffer_threshold(const struct storage_state *state_object,
					     const struct storage_data *type)
{
	int err;
	int count;
	const struct storage_backend *backend = storage_backend_get();

	if (state_object->buffer_threshold_limit == 0) {
		/* Threshold limit of 0 means threshold is disabled */
		return;
	}

	count = backend->count(type);

	if (count < 0) {
		LOG_ERR("Failed to get count for %p, error: %d", type->name, count);
		return;
	}

	if ((uint32_t)count >= state_object->buffer_threshold_limit) {
		/* Count is above threshold */
		if (!state_object->threshold_notified[type->data_type]) {
			/* Threshold crossed - notify once */
			LOG_DBG("Buffer threshold limit reached for %s: count=%d, limit=%u",
				type->name, count, state_object->buffer_threshold_limit);

			struct storage_msg threshold_msg = {
				.type = STORAGE_THRESHOLD_REACHED,
				.data_type = type->data_type,
				.data_len = (uint16_t)count,
			};

			err = zbus_chan_pub(&storage_chan, &threshold_msg, PUB_TIMEOUT);
			if (err) {
				LOG_ERR("Failed to publish buffer threshold message, error: %d", err);
				SEND_FATAL_ERROR();
			}

			/* Mark that we've notified for this data type */
			((struct storage_state *)state_object)->threshold_notified[type->data_type] = true;
		}
	} else {
		/* Count is below threshold - clear the notified flag for next crossing */
		((struct storage_state *)state_object)->threshold_notified[type->data_type] = false;
	}
}

#ifdef CONFIG_APP_ENVIRONMENTAL
/* Initialize environmental stream state from file.
 * Loads file size to determine write offset for append operations.
 */
static int environmental_stream_init(void)
{
	struct fs_dirent entry;
	int ret;

	if (env_stream.initialized) {
		return 0;
	}

	ret = fs_stat(ENV_STREAM_FILE_PATH, &entry);
	if (ret < 0) {
		/* File probably does not exist yet; that's fine */
		env_stream.file_offset = 0;
		env_stream.records_written = 0;
	} else {
		env_stream.file_offset = entry.size;
		/* Rough estimate: each record is one environmental_msg */
		env_stream.records_written = entry.size / sizeof(struct environmental_msg);
	}

	env_stream.initialized = true;
	return 0;
}

/* Bulk append environmental messages to the dedicated stream file.
 * Opens file once, seeks to end, writes all records, closes.
 * This avoids the per-message overhead of the generic backend.
 */
static int environmental_stream_store_bulk(const struct environmental_msg *msgs, size_t count)
{
	struct fs_file_t file;
	int ret;
	size_t total_size;
	uint32_t write_start_ms;

	if ((msgs == NULL) || (count == 0) || (count > ENV_BULK_WRITE_MAX_RECORDS)) {
		return -EINVAL;
	}

	ret = environmental_stream_init();
	if (ret) {
		return ret;
	}

	total_size = count * sizeof(struct environmental_msg);

	fs_file_t_init(&file);

	write_start_ms = k_uptime_get();

	ret = fs_open(&file, ENV_STREAM_FILE_PATH, FS_O_CREATE | FS_O_RDWR);
	if (ret < 0) {
		LOG_ERR("Failed to open %s: %d", ENV_STREAM_FILE_PATH, ret);
		return ret;
	}

	ret = fs_seek(&file, env_stream.file_offset, FS_SEEK_SET);
	if (ret < 0) {
		LOG_ERR("Failed to seek %s: %d", ENV_STREAM_FILE_PATH, ret);
		(void)fs_close(&file);
		return ret;
	}

	ret = fs_write(&file, msgs, total_size);
	if (ret < 0) {
		LOG_ERR("Failed to write environmental stream: %d", ret);
		(void)fs_close(&file);
		return ret;
	}

	if ((size_t)ret != total_size) {
		LOG_ERR("Short write to environmental stream: %d/%zu", ret, total_size);
		(void)fs_close(&file);
		return -EIO;
	}

	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("Failed to close %s: %d", ENV_STREAM_FILE_PATH, ret);
		return ret;
	}

	uint32_t write_time_ms = k_uptime_get() - write_start_ms;

	if (write_time_ms > 1000) {
		LOG_WRN("Slow environmental write: %u ms for %zu records", write_time_ms, count);
	}

	env_stream.file_offset += total_size;
	env_stream.records_written += count;
	bytes_written += total_size;

	return 0;
}

/* Read and print environmental data stored in the dedicated stream file.
 * Displays first 10 records with timestamp, pressure, sample count, and first sample sensor values.
 */
static void environmental_stream_debug_print(void)
{
	struct fs_file_t file;
	struct environmental_msg msg;
	int ret;
	int record_count = 0;
	const int MAX_RECORDS_TO_PRINT = 10;
	int retry_count = 0;
	const int MAX_RETRIES = 3;

	/* Retry open in case storage thread is accessing file */
	while (retry_count < MAX_RETRIES) {
		ret = fs_open(&file, ENV_STREAM_FILE_PATH, FS_O_READ);
		if (ret == 0) {
			break;
		}
		if (ret != -EBUSY && ret != -EAGAIN) {
			LOG_INF("Environmental stream file not found or empty (error: %d)", ret);
			return;
		}
		retry_count++;
		if (retry_count < MAX_RETRIES) {
			k_msleep(50);
		}
	}

	if (ret < 0) {
		LOG_INF("Failed to open environmental stream file after %d retries (error: %d)", 
			MAX_RETRIES, ret);
		return;
	}

	LOG_INF("=== Environmental Data Storage ===");
	LOG_INF("Reading up to %d stored records:", MAX_RECORDS_TO_PRINT);
	LOG_INF("");

	while (record_count < MAX_RECORDS_TO_PRINT) {
		ret = fs_read(&file, &msg, sizeof(msg));

		if (ret == 0) {
			/* End of file */
			break;
		}

		if (ret < 0) {
			LOG_ERR("Failed to read environmental stream: %d", ret);
			break;
		}

		if ((size_t)ret != sizeof(msg)) {
			LOG_WRN("Partial read: %d/%zu bytes", ret, sizeof(msg));
			break;
		}

		/* Print record summary */
		LOG_INF("[Record %d]", record_count);
		LOG_INF("  Timestamp: %u ms, Samples: %u, Pressure: %d Pa (valid: %d)",
			msg.batch_timestamp_ms, msg.sample_count, msg.pressure, msg.pressure_valid);

		if (msg.sample_count > 0) {
			/* Print first sample data */
			float accel_x = (float)msg.accel_hp[0][0] / ACCEL_SCALE;
			float accel_y = (float)msg.accel_hp[1][0] / ACCEL_SCALE;
			float accel_z = (float)msg.accel_hp[2][0] / ACCEL_SCALE;
			float gyro_x = (float)msg.gyro_hp[0][0] / GYRO_SCALE;
			float gyro_y = (float)msg.gyro_hp[1][0] / GYRO_SCALE;
			float gyro_z = (float)msg.gyro_hp[2][0] / GYRO_SCALE;

			LOG_INF("  Sample 0 - Accel: (%.3f, %.3f, %.3f) g, Gyro: (%.2f, %.2f, %.2f) dps",
				accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
		}

		LOG_INF("");
		record_count++;
	}

	if (record_count == 0) {
		LOG_INF("No records found in environmental stream");
	} else {
		LOG_INF("Total records displayed: %d", record_count);
	}

	LOG_INF("=== End of Environmental Data ===");

	(void)fs_close(&file);
}

/* Public interface for shell commands */
void storage_env_print(void)
{
	environmental_stream_debug_print();
}

/* Drain multiple environmental messages from the queue and write them in bulk.
 * This replaces the per-message generic backend path with a single filesystem operation.
 */
static int handle_environmental_direct(struct storage_state *state_object)
{
	int err;
	struct environmental_msg msgs[ENV_BULK_WRITE_MAX_RECORDS];
	size_t msg_count = 0;

	ARG_UNUSED(state_object);

	/* Drain up to ENV_BULK_WRITE_MAX_RECORDS messages from queue */
	while (msg_count < ENV_BULK_WRITE_MAX_RECORDS) {
		err = environmental_msgq_read(&msgs[msg_count], K_NO_WAIT);
		if (err == -ENOMSG) {
			break;
		}
		if (err) {
			LOG_ERR("Failed to read from environmental msgq: %d", err);
			return err;
		}

		if (msgs[msg_count].type != ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE) {
			continue;
		}

		if (bytes_written + sizeof(msgs[msg_count]) > max_bytes) {
			LOG_WRN("Flash full limit reached, stopping program");
			storage_full = true;
			return -ENOSPC;
		}

		msg_count++;
	}

	if (msg_count == 0) {
		return 0;
	}

	/* Write all drained messages in one operation */
	err = environmental_stream_store_bulk(msgs, msg_count);
	if (err) {
		LOG_ERR("Failed to store %zu environmental messages: %d", msg_count, err);
		return err;
	}

	LOG_DBG("Drained and wrote %zu environmental messages", msg_count);

	return (int)msg_count;
}
#endif /* CONFIG_APP_ENVIRONMENTAL */

static void handle_data_message(const struct storage_state *state_object,
				const struct storage_data *type,
				const uint8_t *buf)
{
	int err;
	uint8_t data[STORAGE_MAX_DATA_SIZE];
	const struct storage_backend *backend = storage_backend_get();

	LOG_DBG("Handle data message for %s", type->name);

	if (!type->should_store(buf)) {
		return;
	}

	type->extract_data(buf, (void *)data);

	LOG_INF("Data size: %zu", type->data_size);
	LOG_INF("Bytes written: %zu", bytes_written);

	if (bytes_written + type->data_size > max_bytes) {
		LOG_WRN("Flash full limit reached, stopping program");
		storage_full = true; 
		return;
	}

	err = backend->store(type, (const void *)data, type->data_size);
	if (err) {
		LOG_ERR("Failed to store %s data, error: %d", type->name, err);
	}

	bytes_written += type->data_size;

	check_and_notify_buffer_threshold(state_object, type);
}

static void flush_stored_data(void)
{
	int count;
	struct storage_msg msg = {0};
	const struct storage_backend *backend = storage_backend_get();
	uint8_t data[STORAGE_MAX_DATA_SIZE];

	STRUCT_SECTION_FOREACH(storage_data, type) {
		count = backend->count(type);
		if (count < 0) {
			LOG_ERR("Failed to get count for %p, error: %d", type->name, count);
			continue;
		}

		LOG_DBG("Flushing %d %s records", count, type->name);

		while (count > 0) {
			int ret;

			msg.type = STORAGE_DATA;
			msg.data_type = type->data_type;

			ret = backend->retrieve(type, data, sizeof(data));
			if (ret < 0) {
				LOG_ERR("Failed to retrieve %s data, error: %d", type->name, ret);
				break;
			}

			__ASSERT_NO_MSG(ret <= sizeof(data));
			__ASSERT_NO_MSG(ret == type->data_size);

			memcpy((void *)msg.buffer, data, ret);

			msg.data_len = (uint16_t)ret;

			ret = zbus_chan_pub(&storage_data_chan, &msg, PUB_TIMEOUT);
			if (ret) {
				LOG_ERR("Failed to publish %s data, error: %d", type->name, ret);
				SEND_FATAL_ERROR();
			}

			count--;
		}
	}
}

static void storage_clear(void)
{
	int err;

	LOG_INF("=== Storage Clear Starting ===");

	/* Clear all stored data from generic backend */
	err = storage_backend_get()->clear();
	if (err) {
		LOG_ERR("Failed to clear storage backend, error: %d", err);
		SEND_FATAL_ERROR();
	}
	LOG_INF("Generic backend storage cleared successfully");

#ifdef CONFIG_APP_ENVIRONMENTAL
	/* Clear dedicated environmental stream file */
	LOG_INF("Clearing environmental stream file at %s...", ENV_STREAM_FILE_PATH);
	
	/* Check if file exists before trying to delete */
	struct fs_dirent file_entry;
	err = fs_stat(ENV_STREAM_FILE_PATH, &file_entry);
	if (err < 0) {
		/* File doesn't exist - this is expected */
		LOG_INF("Environmental stream file doesn't exist");
	} else {
		/* File exists, delete it */
		err = fs_unlink(ENV_STREAM_FILE_PATH);
		if (err < 0) {
			LOG_ERR("Failed to delete %s: %d", ENV_STREAM_FILE_PATH, err);
			SEND_FATAL_ERROR();
		} else {
			LOG_INF("Environmental stream file deleted successfully");
		}
	}

	env_stream.initialized = false;
	env_stream.file_offset = 0;
	env_stream.records_written = 0;
#endif

	/* Reset capacity tracking */
	bytes_written = 0;
	storage_full = false;
	LOG_INF("=== Storage Clear Complete ===");
}

static void update_threshold(struct storage_state *state_object, uint32_t new_threshold)
{
	if (new_threshold > 0 && new_threshold <= CONFIG_APP_STORAGE_MAX_RECORDS_PER_TYPE) {
		LOG_DBG("Updating buffer threshold limit: %u", new_threshold);
	} else if (new_threshold == 0) {
		LOG_DBG("Disabling buffer threshold limit");
	} else {
		LOG_ERR("Invalid threshold value: %u. Must be between 1 and %u, or 0 to disable.",
			new_threshold, CONFIG_APP_STORAGE_MAX_RECORDS_PER_TYPE);
		return;
	}

	state_object->buffer_threshold_limit = new_threshold;
}

/* Drain any remaining data from the pipe */
static void drain_pipe(void)
{
	uint8_t dummy_buffer[64];

	while (k_pipe_read(&storage_pipe, dummy_buffer, sizeof(dummy_buffer), K_NO_WAIT) > 0) {
		/* Drain pipe */
	}
}

/* Send batch response message with session_id and optional data_len */
static void send_batch_response(enum storage_msg_type response_type,
			       uint32_t session_id,
			       size_t data_len,
			       bool more_data)
{
	int err;
	struct storage_msg response_msg = { 0 };

	response_msg.type = response_type;
	response_msg.session_id = session_id;
	response_msg.data_len = (uint16_t)MIN(data_len, (size_t)UINT16_MAX);
	response_msg.more_data = more_data;

	err = zbus_chan_pub(&storage_chan, &response_msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("Failed to send batch response type %d: %d", response_type, err);
		SEND_FATAL_ERROR();
	}
}

/* Convenience wrappers for batch responses */
static void send_batch_busy_response(uint32_t session_id)
{
	send_batch_response(STORAGE_BATCH_BUSY, session_id, 0, false);
}

static void send_batch_empty_response(uint32_t session_id)
{
	send_batch_response(STORAGE_BATCH_EMPTY, session_id, 0, false);
}

static void send_batch_error_response(uint32_t session_id)
{
	send_batch_response(STORAGE_BATCH_ERROR, session_id, 0, false);
}

static void send_batch_available_response(uint32_t session_id, size_t item_count,
					  bool more_available)
{
	send_batch_response(STORAGE_BATCH_AVAILABLE, session_id, item_count, more_available);
}

/* Populate the pipe with all stored data
 *
 * @return 0 on success (all data written or pipe full)
 * @return -EIO on data retrieval or size validation error
 */
static int populate_pipe(struct storage_state *state_object)
{
	const struct storage_backend *backend = storage_backend_get();
	size_t total_bytes_sent = 0;

	state_object->current_session.more_data = false;

	/* Populate pipe with all stored data */
	STRUCT_SECTION_FOREACH(storage_data, type) {
		int count = backend->count(type);

		while (count > 0) {
			int ret;
			size_t total_size;
			uint8_t item_buffer[sizeof(struct storage_pipe_header) +
					    STORAGE_MAX_DATA_SIZE];
			struct storage_pipe_header *header =
				(struct storage_pipe_header *)item_buffer;
			uint8_t *data = item_buffer + sizeof(struct storage_pipe_header);

			/* Peek at size without copying data (data = NULL) */
			ret = backend->peek(type, NULL, 0);
			if (ret == -EAGAIN) {
				/* No more data of this type */
				break;
			} else if (ret < 0) {
				LOG_ERR("Failed to peek %s data size: %d", type->name, ret);

				return -EIO;
			}

			/* Prepare header with actual size */
			header->type = (uint8_t)type->data_type;

			if ((ret < 0) || (ret > (int)STORAGE_MAX_DATA_SIZE) ||
			    (ret > UINT16_MAX)) {
				LOG_ERR("Invalid data size for header: %d", ret);

				return -EIO;
			}

			header->data_size = (uint16_t)ret;

			/* Calculate exact total size needed using actual data size */
			total_size = sizeof(struct storage_pipe_header) + ret;
			if (total_size > sizeof(item_buffer)) {
				LOG_ERR("Combined data too large: %zu > %zu",
					total_size, sizeof(item_buffer));

				return -EIO;
			}

			/* Check if exact size fits in remaining pipe buffer space */
			if (total_bytes_sent + total_size > CONFIG_APP_STORAGE_BATCH_BUFFER_SIZE) {
				/* Pipe buffer full - stop here without consuming data */
				LOG_DBG("Pipe buffer full");

				state_object->current_session.more_data = true;

				break;
			}

			/* Now that we know it fits, retrieve the data from backend */
			ret = backend->retrieve(type, data, STORAGE_MAX_DATA_SIZE);
			if (ret < 0) {
				LOG_ERR("Failed to retrieve %s data after peek: %d",
					type->name, ret);

				return -EIO;
			}

			/* Sanity check: retrieved size should match peeked size */
			__ASSERT_NO_MSG(ret == (int)header->data_size);

			/* Write combined buffer atomically to pipe */
			ret = pipe_write_all(&storage_pipe, item_buffer, total_size, PUB_TIMEOUT);
			if (ret < 0) {
				/* This should never happen since we checked space above */
				LOG_ERR("Unexpected pipe write failure after space check: %d", ret);

				return -EIO;
			}

			__ASSERT_NO_MSG(ret == (int)total_size);

			/* Update session progress and byte tracking */
			state_object->current_session.items_sent++;
			total_bytes_sent += total_size;

			count--;
		}
	}

	LOG_DBG("Batch population complete for session 0x%X: %zu/%zu items",
		state_object->current_session.session_id,
		state_object->current_session.items_sent,
		state_object->current_session.total_items);

	return 0;
}

/* Start a new batch session.
 * If the batch is empty, STORAGE_BATCH_EMPTY is sent and the session is not started.
 * If the batch is not empty, the session is started and STORAGE_BATCH_AVAILABLE is sent.
 * Returns 0 if successful, -ENODATA if batch is empty, and other error codes if failed.
 */
static int start_batch_session(struct storage_state *state_object,
			       const struct storage_msg *request_msg)
{
	int err;
	const struct storage_backend *backend = storage_backend_get();
	size_t total_items = 0;

	/* Enforce non-zero session id */
	if (request_msg->session_id == 0U) {
		send_batch_error_response(request_msg->session_id);
		return -EINVAL;
	}

	/* Count total items available */
	STRUCT_SECTION_FOREACH(storage_data, type) {
		int count = backend->count(type);

		if (count > 0) {
			total_items += count;
		}
	}

	if (total_items == 0) {
		send_batch_empty_response(request_msg->session_id);

		return -ENODATA;
	}

	/* Clear any stale data from pipe before starting new session */
	drain_pipe();

	/* Start new session using requester's session ID */
	state_object->current_session.session_id = request_msg->session_id;
	state_object->current_session.total_items = total_items;
	state_object->current_session.items_sent = 0;

	/* Try to populate the pipe */
	err = populate_pipe(state_object);
	if (err < 0) {
		/* Error occurred during pipe population */
		send_batch_error_response(request_msg->session_id);

		LOG_ERR("Failed to populate pipe for session 0x%X: %d",
			state_object->current_session.session_id, err);

		return err;
	}

	/* Success - pipe populated (fully or partially) */
	send_batch_available_response(request_msg->session_id,
				      state_object->current_session.items_sent,
				      state_object->current_session.more_data);

	LOG_DBG("Started batch session (session_id 0x%X), %zu items in batch (%zu total)",
		state_object->current_session.session_id,
		state_object->current_session.items_sent,
		total_items);

	return 0;
}

/* New API: Read one item from storage batch */
int storage_batch_read(struct storage_data_item *out_item, k_timeout_t timeout)
{
	struct storage_pipe_header header;
	int ret;

	if (!out_item) {
		return -EINVAL;
	}

	/* Session validation is implicit - if there's no active session,
	 * the pipe will be empty and this function will return -EAGAIN.
	 */

	/* First, try to read just the header to get the data size */
	ret = pipe_read_exact(&storage_pipe, (uint8_t *)&header,
			       sizeof(header), timeout);
	if (ret < 0) {
		return ret;  /* -EAGAIN (no data in timeout) or other error */
	}

	if (ret != sizeof(header)) {
		LOG_ERR("Incomplete header read: %d/%zu bytes",
			ret, sizeof(header));
		return -EIO;
	}

	/* Validate header */
	if ((size_t)header.data_size > sizeof(out_item->data)) {
		LOG_ERR("Data size too large: %u > %zu",
			header.data_size, sizeof(out_item->data));
		return -EMSGSIZE;
	}

	/* Read the data portion */
	ret = pipe_read_exact(&storage_pipe, (uint8_t *)&out_item->data,
			  (size_t)header.data_size, timeout);
	if (ret < 0) {
		LOG_ERR("Failed to read data from pipe: %d", ret);
		return ret;
	}

	if (ret != header.data_size) {
		LOG_ERR("Incomplete data read: %d/%u bytes",
			ret, header.data_size);
		return -EIO;
	}

	/* Fill output structure */
	out_item->type = header.type;

	LOG_DBG("Read storage item: type=%u, size=%u", header.type, header.data_size);

	/* Session will be closed via explicit STORAGE_BATCH_CLOSE messages */

	return 0;
}

#ifdef CONFIG_APP_STORAGE_SHELL_STATS
static void handle_storage_stats(void)
{
	const struct storage_backend *backend = storage_backend_get();
	int total_records = 0;
	int total_types = 0;

	LOG_INF("=== Storage Statistics ===");
	LOG_INF("Backend: %s: %s", backend ? "Available" : "Not available",
		IS_ENABLED(CONFIG_APP_STORAGE_BACKEND_RAM)        ? "RAM"
		: IS_ENABLED(CONFIG_APP_STORAGE_BACKEND_LITTLEFS) ? "LittleFS"
								  : "Unknown");
	if (!backend) {
		LOG_ERR("No storage backend available");
		return;
	}

	/* Iterate through all registered storage data types */
	STRUCT_SECTION_FOREACH(storage_data, type) {
		int count = backend->count(type);

		if (count < 0) {
			LOG_ERR("Failed to get count for %s, error: %d", type->name, count);
			continue;
		}

		LOG_INF("%s: %d records", type->name, count);

		total_records += count;
		total_types++;
	}

	LOG_INF("Total: %d records across %d data types", total_records, total_types);
	LOG_INF("Max records per type: %d", CONFIG_APP_STORAGE_MAX_RECORDS_PER_TYPE);
	LOG_INF("========================");
}
#endif /* CONFIG_APP_STORAGE_SHELL_STATS */

/* Handler for STATE_RUNNING */
static void state_running_entry(void *o)
{
	int err;
	const struct storage_backend *backend = storage_backend_get();
	struct storage_state *storage_state = (struct storage_state *)o;

	LOG_DBG("%s", __func__);

	err = backend->init();
	if (err) {
		LOG_ERR("Failed to initialize storage backend, error: %d", err);
		SEND_FATAL_ERROR();

		return;
	}

	/* Initialize threshold notification latch flags */
	memset(storage_state->threshold_notified, 0, sizeof(storage_state->threshold_notified));

	k_work_init_delayable(&storage_state->session_timeout_work, session_timeout_work_fn);

	/* Clear storage on boot to ensure clean state */
	LOG_INF("Clearing storage on boot...");
	storage_clear();

#ifdef CONFIG_APP_ENVIRONMENTAL
	/* Debug: Read and display stored environmental data */
	environmental_stream_debug_print();
#endif /* CONFIG_APP_ENVIRONMENTAL */
}

static enum smf_state_result state_running_run(void *o)
{
	struct storage_state *state_object = (struct storage_state *)o;
	const struct storage_msg *msg = (const struct storage_msg *)state_object->msg_buf;

	LOG_DBG("%s", __func__);

	if (state_object->chan == &storage_chan) {
		switch (msg->type) {
		case STORAGE_CLEAR:
			/* Clear all stored data */
			storage_clear();
			break;

		case STORAGE_FLUSH:
			flush_stored_data();
			break;

		case STORAGE_SET_THRESHOLD:
			/* Update buffer threshold limit */
			update_threshold(state_object, msg->data_len);
			break;

#ifdef CONFIG_APP_STORAGE_SHELL_STATS
		case STORAGE_STATS:
			/* Show storage statistics */
			handle_storage_stats();
			break;
#endif /* CONFIG_APP_STORAGE_SHELL_STATS */
		default:
			break;
		}
	}

	/* Check if message is from a registered data type */
	STRUCT_SECTION_FOREACH(storage_data, type) {
		if (state_object->chan == type->chan) {
			handle_data_message(state_object, type, state_object->msg_buf);

			return SMF_EVENT_HANDLED;
		}
	}

	return SMF_EVENT_PROPAGATE;
}

static enum smf_state_result state_buffer_idle_run(void *o)
{
	struct storage_state *state_object = (struct storage_state *)o;
	const struct storage_msg *msg = (const struct storage_msg *)state_object->msg_buf;

	LOG_DBG("%s", __func__);

	if (state_object->chan == &storage_chan && msg->type == STORAGE_BATCH_REQUEST) {
		LOG_DBG("Batch request received, switching to batch active state");
		/* Set up session ID for the upcoming batch session */
		state_object->current_session.session_id = msg->session_id;
		smf_set_state(SMF_CTX(state_object), &states[STATE_BUFFER_PIPE_ACTIVE]);

		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

static void state_buffer_pipe_active_entry(void *o)
{
	int err;
	struct storage_state *state_object = (struct storage_state *)o;
	const struct storage_msg *msg = (const struct storage_msg *)state_object->msg_buf;

	LOG_DBG("%s", __func__);

	err = start_batch_session(state_object, msg);
	if (err == -ENODATA) {
		/* No data available, report it */
		send_batch_empty_response(msg->session_id);

		return;
	} else if (err) {
		LOG_ERR("Failed to start pipe session: %d", err);
		send_batch_error_response(msg->session_id);

		return;
	}

	/* Start session timeout */
	k_work_schedule(&state_object->session_timeout_work,
			K_SECONDS(STORAGE_SESSION_TIMEOUT_SECONDS));

	LOG_DBG("Batch session started, session_id: %u", state_object->current_session.session_id);
}

static enum smf_state_result state_buffer_pipe_active_run(void *o)
{
	struct storage_state *state_object = (struct storage_state *)o;
	const struct storage_msg *msg = (const struct storage_msg *)state_object->msg_buf;

	LOG_DBG("%s", __func__);

	if (state_object->chan == &storage_chan) {
		switch (msg->type) {
		case STORAGE_CLEAR:
			LOG_WRN("Cannot clear storage while batch session is active");

			return SMF_EVENT_HANDLED;

		case STORAGE_BATCH_CLOSE:
			if (state_object->current_session.session_id == msg->session_id) {
				smf_set_state(SMF_CTX(state_object), &states[STATE_BUFFER_IDLE]);
			} else {
				LOG_WRN("Invalid session ID: 0x%X (current: 0x%X)",
					msg->session_id, state_object->current_session.session_id);
			}

			return SMF_EVENT_HANDLED;

		case STORAGE_BATCH_REQUEST:
			LOG_DBG("Batch request received, session_id: 0x%X", msg->session_id);

			if (state_object->current_session.session_id &&
			    (state_object->current_session.session_id != msg->session_id)) {
				send_batch_busy_response(msg->session_id);
				LOG_DBG("Session ID mismatch: 0x%X (current: 0x%X)",
					msg->session_id, state_object->current_session.session_id);

				return SMF_EVENT_HANDLED;
			}

			/* We allow multiple requests in the same session.
			 * The batch will be refreshed with new data.
			 */
			start_batch_session(state_object, msg);
			LOG_DBG("Session started: 0x%X", state_object->current_session.session_id);

			/* Reset session timeout on activity */
			k_work_reschedule(&state_object->session_timeout_work,
					  K_SECONDS(STORAGE_SESSION_TIMEOUT_SECONDS));

			return SMF_EVENT_HANDLED;

		default:
			/* Don't care */
			break;
		}
	}

	if (state_object->chan == &priv_storage_chan) {
		enum priv_storage_msg priv_msg = *(enum priv_storage_msg *)state_object->msg_buf;

		if (priv_msg == STORAGE_BATCH_SESSION_TIMEOUT) {
			struct storage_msg close_msg = {0};

			close_msg.type = STORAGE_BATCH_CLOSE;
			close_msg.session_id = state_object->current_session.session_id;

			LOG_WRN("Session timeout processed, closing session 0x%X",
				close_msg.session_id);

			/* Notify other modules (like cloud) that the batch is closed */
			zbus_chan_pub(&storage_chan, &close_msg, PUB_TIMEOUT);

			/* Force transition to idle state */
			smf_set_state(SMF_CTX(state_object), &states[STATE_BUFFER_IDLE]);

			return SMF_EVENT_HANDLED;
		}
	}

	return SMF_EVENT_PROPAGATE;
}

static void state_buffer_pipe_active_exit(void *o)
{
	struct storage_state *state_object = (struct storage_state *)o;

	LOG_DBG("%s", __func__);

	/* Cancel session timeout */
	k_work_cancel_delayable(&state_object->session_timeout_work);

	/* Drain any remaining data from pipe */
	drain_pipe();

	/* Clear session state */
	memset(&state_object->current_session, 0, sizeof(state_object->current_session));
}

static void storage_thread(void)
{
	int err;
	int task_wdt_id;
	const uint32_t wdt_timeout_ms = CONFIG_APP_STORAGE_WATCHDOG_TIMEOUT_SECONDS * MSEC_PER_SEC;
	static struct storage_state storage_state;

	storage_state.buffer_threshold_limit = CONFIG_APP_STORAGE_INITIAL_THRESHOLD;

	LOG_DBG("Storage module task started");

	task_wdt_id = task_wdt_add(wdt_timeout_ms, task_wdt_callback,
				  (void *)k_current_get());
	if (task_wdt_id < 0) {
		LOG_ERR("Failed to add task to watchdog: %d", task_wdt_id);
		SEND_FATAL_ERROR();

		return;
	}

	err = zbus_chan_add_obs(&storage_chan, &storage_subscriber, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("Failed to add observer to storage_chan, error: %d", err);
		SEND_FATAL_ERROR();

		return;
	}

	/* Initialize the state machine */
	smf_set_initial(SMF_CTX(&storage_state), &states[STATE_RUNNING]);

while (true) {
	bool did_work = false;

	err = task_wdt_feed(task_wdt_id);
	if (err) {
		LOG_ERR("task_wdt_feed, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

#ifdef CONFIG_APP_ENVIRONMENTAL
	int drained = handle_environmental_direct(&storage_state);
	if (drained < 0) {
		LOG_ERR("handle_environmental_direct failed: %d", drained);
		SEND_FATAL_ERROR();
		return;
	}
	if (drained > 0) {
		did_work = true;
	}
#endif

	err = zbus_sub_wait_msg(&storage_subscriber, &storage_state.chan,
				storage_state.msg_buf, K_NO_WAIT);
	if (err == 0) {
		did_work = true;

		err = smf_run_state(SMF_CTX(&storage_state));
		if (err) {
			LOG_ERR("smf_run_state(), error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	} else if (err != -ENOMSG) {
		LOG_ERR("zbus_sub_wait_msg, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	if (!did_work) {
		k_sleep(K_MSEC(5));
	}
}
}

K_THREAD_DEFINE(storage_thread_id,
		CONFIG_APP_STORAGE_THREAD_STACK_SIZE,
		storage_thread, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
