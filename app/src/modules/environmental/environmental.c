/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/smf.h>
#include <zephyr/sys/ring_buffer.h>
#include <date_time.h>

#include "app_common.h"
#include "environmental.h"

#define FS 25 /* Sampling frequency in Hz */
#define FS_MS 1000 / FS /* Sampling frequency in milliseconds (1000/1) */

#define ENV_FS 1 /* Environmental sensor (BME680) sampling frequency in Hz */
#define ENV_FS_MS 1000 / ENV_FS /* Environmental sensor sampling frequency in milliseconds */
#define ENV_SAMPLES_BETWEEN_PUBLISH 50 /* Include environmental data in every Nth IMU message */

/* Batch message accumulation */
static struct environmental_msg batch_msg = {
	.type = ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE,
	.sample_count = 0,
};
static uint8_t imu_sample_count = 0;  /* Counter for IMU samples to include pressure periodically */

/* Latest environmental sensor readings (updated at 1 Hz) */
static float latest_pressure = 0.0f;
LOG_MODULE_REGISTER(environmental, CONFIG_APP_ENVIRONMENTAL_LOG_LEVEL);

/* Define channels provided by this module */
ZBUS_CHAN_DEFINE(environmental_chan,
		 struct environmental_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

/* Register subscriber */
ZBUS_MSG_SUBSCRIBER_DEFINE(environmental);

/* Observe channels */
ZBUS_CHAN_ADD_OBS(environmental_chan, environmental, 0);

#define MAX_MSG_SIZE sizeof(struct environmental_msg)

BUILD_ASSERT(CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS >
	     CONFIG_APP_ENVIRONMENTAL_MSG_PROCESSING_TIMEOUT_SECONDS,
	     "Watchdog timeout must be greater than maximum message processing time");

/* State machine */

/* Environmental module states.
 */
enum environmental_module_state {
	/* The module is running and waiting for sensor value requests */
	STATE_RUNNING,
};

/* State object.
 * Used to transfer context data between state changes.
 */
struct environmental_state_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last channel type that a message was received on */
	const struct zbus_channel *chan;

	/* Buffer for last zbus message */
	uint8_t msg_buf[MAX_MSG_SIZE];

	/* Pointer to the BME680 sensor device */
	const struct device *const bme680;

	const struct device *const bmi270;

	float accel_hp[3][SAMPLES_PER_BATCH];
	float gyro_hp[3][SAMPLES_PER_BATCH];

	const struct device *const adxl367;

	float accel_lp[3][SAMPLES_PER_BATCH];
};



/* Global sensor device pointers for work handlers */
static const struct device *g_bmi270;
static const struct device *g_adxl367;
static const struct device *g_bme680;

/* Forward declarations */
static enum smf_state_result state_running_run(void *obj);
static void sample_publish_work_handler(struct k_work *work);
static void sample_collect_work_handler(struct k_work *work);
static void env_sample_work_handler(struct k_work *work);

/* Work items for sensor sampling */
static K_WORK_DELAYABLE_DEFINE(sample_publish_work, sample_publish_work_handler);
static K_WORK_DELAYABLE_DEFINE(sample_collect_work, sample_collect_work_handler);
static K_WORK_DELAYABLE_DEFINE(env_sample_work, env_sample_work_handler);

/* State machine definition */
static const struct smf_state states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(NULL, state_running_run, NULL, NULL, NULL),
};

/* Publish accumulated batch message to zbus */
static void sample_publish_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (batch_msg.sample_count == 0) {
		LOG_WRN("sample_publish_work_handler: batch_msg.sample_count is 0");
		return;
	}

	LOG_INF("Publishing batch with %d samples",
		batch_msg.sample_count);

	err = zbus_chan_pub(&environmental_chan, &batch_msg, PUB_TIMEOUT);
	if (err) {
		if (err == -ENOMEM || err == -EAGAIN) {
			/* Temporary resource exhaustion (net_buf pool full) - retry later */
			LOG_WRN("zbus_chan_pub batch failed with %d (resource issue), retrying in 10ms", err);
			k_work_reschedule(&sample_publish_work, K_MSEC(10));
			return;
		} else {
			LOG_ERR("zbus_chan_pub batch, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	}

	/* Reset batch message for next batch */
	batch_msg.sample_count = 0;
}

/* Periodic work handler to sample sensors at 50 Hz and accumulate into batch message */
static void sample_collect_work_handler(struct k_work *work)
{
	int err;
	uint8_t idx;  /* Index into batch arrays */

	ARG_UNUSED(work);

	/* If batch is full, don't collect more samples until it's published */
	if (batch_msg.sample_count >= SAMPLES_PER_BATCH) {
		LOG_WRN("Batch buffer full, skipping sample");
		return;
	}

	idx = batch_msg.sample_count;

	/* Fetch data from BMI270 (accelerometer and gyroscope - high performance) */
	if (g_bmi270 && device_is_ready(g_bmi270)) {
		struct sensor_value accel_vals[3] = { 0 };
		struct sensor_value gyro_vals[3] = { 0 };

		/* Full fetch ensures atomic read of all sensor data */
		err = sensor_sample_fetch(g_bmi270);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_sample_fetch bmi270, error: %d", err);
			return;
		}

		err = sensor_channel_get(g_bmi270, SENSOR_CHAN_ACCEL_XYZ, accel_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get accel_xyz, error: %d", err);
			return;
		}

		batch_msg.accel_hp[0][idx] = sensor_value_to_double(&accel_vals[0]);
		batch_msg.accel_hp[1][idx] = sensor_value_to_double(&accel_vals[1]);
		batch_msg.accel_hp[2][idx] = sensor_value_to_double(&accel_vals[2]);

		err = sensor_channel_get(g_bmi270, SENSOR_CHAN_GYRO_XYZ, gyro_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get gyro_xyz, error: %d", err);
			return;
		}

		batch_msg.gyro_hp[0][idx] = sensor_value_to_double(&gyro_vals[0]);
		batch_msg.gyro_hp[1][idx] = sensor_value_to_double(&gyro_vals[1]);
		batch_msg.gyro_hp[2][idx] = sensor_value_to_double(&gyro_vals[2]);

		LOG_DBG("BMI270 sampled[%d]: accel_hp[%.2f, %.2f, %.2f] g, gyro_hp[%.2f, %.2f, %.2f] dps",
			idx,
			(double)batch_msg.accel_hp[0][idx], (double)batch_msg.accel_hp[1][idx], (double)batch_msg.accel_hp[2][idx],
			(double)batch_msg.gyro_hp[0][idx], (double)batch_msg.gyro_hp[1][idx], (double)batch_msg.gyro_hp[2][idx]);
	}

	/* Fetch data from ADXL367 (accelerometer - low power) */
	if (g_adxl367 && device_is_ready(g_adxl367)) {
		struct sensor_value accel_vals[3] = { 0 };

		/* Full fetch ensures atomic read of all sensor data */
		err = sensor_sample_fetch(g_adxl367);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_sample_fetch adxl367, error: %d", err);
			return;
		}

		err = sensor_channel_get(g_adxl367, SENSOR_CHAN_ACCEL_XYZ, accel_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get accel_lp_xyz, error: %d", err);
			return;
		}

		batch_msg.accel_lp[0][idx] = sensor_value_to_double(&accel_vals[0]);
		batch_msg.accel_lp[1][idx] = sensor_value_to_double(&accel_vals[1]);
		batch_msg.accel_lp[2][idx] = sensor_value_to_double(&accel_vals[2]);

		LOG_DBG("ADXL367 sampled[%d]: accel_lp[%.2f, %.2f, %.2f] g",
			idx,
			(double)batch_msg.accel_lp[0][idx], (double)batch_msg.accel_lp[1][idx], (double)batch_msg.accel_lp[2][idx]);
	}

	batch_msg.timestamp[idx] = k_uptime_get();
	err = date_time_now(&batch_msg.timestamp[idx]);
	if (err != 0 && err != -ENODATA) {
		LOG_WRN("date_time_now, error: %d", err);
	}

	/* Include pressure data every ENV_SAMPLES_BETWEEN_PUBLISH samples */
	imu_sample_count++;
	if (imu_sample_count >= ENV_SAMPLES_BETWEEN_PUBLISH) {
		batch_msg.pressure[idx] = latest_pressure;
		imu_sample_count = 0;
		LOG_DBG("Adding pressure data[%d]: press=%.2f Pa",
			idx,
			(double)batch_msg.pressure[idx]);
	}

	/* Increment sample count */
	batch_msg.sample_count++;

	/* Trigger publish when batch is full */
	if (batch_msg.sample_count >= SAMPLES_PER_BATCH) {
		LOG_DBG("Batch full (%d samples), submitting publish work", batch_msg.sample_count);
		k_work_schedule(&sample_publish_work, K_NO_WAIT);
	}

	/* Work completes without rescheduling - only fires on interrupt triggers */
	return;
}

/* Periodic work handler to sample BME680 environmental sensor at 1 Hz */
static void env_sample_work_handler(struct k_work *work)
{
	int err;
	struct sensor_value press = { 0 };

	ARG_UNUSED(work);

	LOG_DBG("env_sample_work_handler: starting BME680 pressure sample");

	if (!g_bme680) {
		LOG_WRN("env_sample_work_handler: g_bme680 is NULL");
		goto reschedule_env;
	}

	if (!device_is_ready(g_bme680)) {
		LOG_WRN("env_sample_work_handler: BME680 device not ready");
		goto reschedule_env;
	}

	err = sensor_sample_fetch(g_bme680);
	if (err) {
		LOG_WRN("env_sample_work_handler: sensor_sample_fetch error: %d", err);
		goto reschedule_env;
	}

	err = sensor_channel_get(g_bme680, SENSOR_CHAN_PRESS, &press);
	if (err) {
		LOG_WRN("env_sample_work_handler: sensor_channel_get PRESS error: %d", err);
		goto reschedule_env;
	}

	/* Update latest pressure value */
	latest_pressure = (float)sensor_value_to_double(&press);

	LOG_INF("env_sample_work_handler: BME680 pressure sampled - press=%.2f Pa",
		(double)latest_pressure);

reschedule_env:
	/* Reschedule for next 1 Hz sampling interval */
	k_work_schedule(&env_sample_work, K_MSEC(ENV_FS_MS));
}

/* Sensor trigger callback for interrupt-based triggers */
static void sensor_trigger_callback(const struct device *sensor, const struct sensor_trigger *trigger)
{
	ARG_UNUSED(trigger);

	LOG_DBG("Sensor data-ready trigger from %s, submitting sample work", sensor->name);
	/* Offload work to system workqueue - keep ISR short.
	 * This follows the Zephyr ISR offloading pattern: signal a work item
	 * to do interrupt-related processing outside of ISR context.
	 * Use k_work_schedule with K_NO_WAIT for delayable work.
	 */
	k_work_schedule(&sample_collect_work, K_NO_WAIT);
}

/* Initialize sensor triggers and start periodic sampling */
static int sensors_init(const struct device *bmi270, const struct device *adxl367, const struct device *bme680)
{
	int err = 0;
	int bmi270_trigger_ok = 0;
	int adxl367_trigger_ok = 0;
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	/* Store global pointers for use in work handlers */
	g_bmi270 = bmi270;
	g_adxl367 = adxl367;
	g_bme680 = bme680;

	/* Log sensor device pointers */
	LOG_INF("sensors_init: g_bmi270=%p, g_adxl367=%p, g_bme680=%p", g_bmi270, g_adxl367, g_bme680);
	LOG_INF("sensors_init: bmi270_ready=%d, adxl367_ready=%d, bme680_ready=%d", 
		bmi270 ? device_is_ready(bmi270) : 0, 
		adxl367 ? device_is_ready(adxl367) : 0,
		bme680 ? device_is_ready(bme680) : 0);

	/* Configure BMI270 with data-ready trigger if available */
	if (bmi270 && device_is_ready(bmi270)) {
		LOG_DBG("BMI270 device ready, configuring sensor");

		struct sensor_value full_scale, sampling_freq, oversampling;

		/* Configure accelerometer: 2g full scale, FS Hz, 1x oversampling */
		full_scale.val1 = 2;
		full_scale.val2 = 0;
		sampling_freq.val1 = FS;
		sampling_freq.val2 = 0;
		oversampling.val1 = 1;
		oversampling.val2 = 0;

		sensor_attr_set(bmi270, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
		sensor_attr_set(bmi270, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
		err = sensor_attr_set(bmi270, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
		if (err) {
			LOG_WRN("BMI270 accel configuration failed, error: %d", err);
		} else {
			LOG_DBG("BMI270 accel configured: 2g, %d Hz", FS);
		}

		/* Configure gyroscope: 500 dps full scale, FS Hz, 1x oversampling */
		full_scale.val1 = 500;
		full_scale.val2 = 0;
		sampling_freq.val1 = FS;
		sampling_freq.val2 = 0;
		oversampling.val1 = 1;
		oversampling.val2 = 0;

		sensor_attr_set(bmi270, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
		sensor_attr_set(bmi270, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
		err = sensor_attr_set(bmi270, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
		if (err) {
			LOG_WRN("BMI270 gyro configuration failed, error: %d", err);
		} else {
			LOG_DBG("BMI270 gyro configured: 500 dps, %d Hz", FS);
		}

		/* Perform a full sample fetch to initialize and wake up the sensor */
		err = sensor_sample_fetch(bmi270);
		if (err && err != -ENOTSUP) {
			LOG_WRN("Initial sensor_sample_fetch for BMI270 failed, error: %d", err);
		} else {
			LOG_DBG("BMI270 initial fetch completed");
		}

		err = sensor_trigger_set(bmi270, &trig, sensor_trigger_callback);
		if (err) {
			LOG_ERR("BMI270 data-ready trigger not supported on this device, error: %d", err);
		} else {
			LOG_INF("BMI270 data-ready trigger configured successfully");
			bmi270_trigger_ok = 1;
		}
	}

	/* Configure ADXL367 with data-ready trigger if available */
	if (adxl367 && device_is_ready(adxl367)) {
		LOG_DBG("ADXL367 device ready, configuring sensor");

		struct sensor_value full_scale, sampling_freq, oversampling;

		/* Configure accelerometer: 4g full scale, FS Hz, 1x oversampling */
		full_scale.val1 = 4;
		full_scale.val2 = 0;
		sampling_freq.val1 = FS;
		sampling_freq.val2 = 0;
		oversampling.val1 = 1;
		oversampling.val2 = 0;

		sensor_attr_set(adxl367, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
		sensor_attr_set(adxl367, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
		err = sensor_attr_set(adxl367, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
		if (err) {
			LOG_WRN("ADXL367 configuration failed, error: %d", err);
		} else {
			LOG_DBG("ADXL367 configured: 4g, %d Hz", FS);
		}

		/* Perform a full sample fetch to initialize and wake up the sensor */
		err = sensor_sample_fetch(adxl367);
		if (err && err != -ENOTSUP) {
			LOG_WRN("Initial sensor_sample_fetch for ADXL367 failed, error: %d", err);
		} else {
			LOG_DBG("ADXL367 initial fetch completed");
		}

		err = sensor_trigger_set(adxl367, &trig, sensor_trigger_callback);
		if (err) {
			LOG_ERR("ADXL367 data-ready trigger not supported on this device, error: %d", err);
		} else {
			LOG_INF("ADXL367 data-ready trigger configured successfully");
			adxl367_trigger_ok = 1;
		}
	}

	/* Only start periodic sampling as fallback if trigger setup failed completely */
	if (!bmi270_trigger_ok || !adxl367_trigger_ok) {
		LOG_INF("Interrupt-based triggers not available, starting periodic sampling at %d Hz (every %d ms)",
			FS, FS_MS);
		err = k_work_schedule(&sample_collect_work, K_MSEC(FS_MS));
		if (err < 0) {
			LOG_ERR("k_work_schedule sample_collect_work, error: %d", err);
			return err;
		}
	} else {
		LOG_INF("Interrupt-driven sampling configured (no periodic fallback)");
	}

	/* Start environmental sensor (BME680) sampling at 1 Hz */
	if (bme680 && device_is_ready(bme680)) {
		err = k_work_schedule(&env_sample_work, K_MSEC(ENV_FS_MS));
		if (err < 0) {
			LOG_ERR("k_work_schedule env_sample_work, error: %d", err);
			return err;
		}
		LOG_INF("Environmental sensor (BME680) sampling scheduled at %d Hz", ENV_FS);
	} else {
		LOG_WRN("BME680 device not ready during initialization, environmental sampling disabled");
	}

	return 0;
}

/* Define interrupt handlers*/



/* Note: On-demand sampling removed.
 * Environmental data is now sampled continuously at 1 Hz via env_sample_work_handler()
 * and included in batch messages indirectly. ENVIRONMENTAL_SENSOR_SAMPLE_REQUEST
 * messages are ignored as we don't support on-demand sampling anymore.
 */

static void env_wdt_callback(int channel_id, void *user_data)
{
	LOG_ERR("Watchdog expired, Channel: %d, Thread: %s",
		channel_id, k_thread_name_get((k_tid_t)user_data));

	SEND_FATAL_ERROR_WATCHDOG_TIMEOUT();
}

/* State handlers */

static enum smf_state_result state_running_run(void *obj)
{
	struct environmental_state_object const *state_object = obj;

	if (&environmental_chan == state_object->chan) {
		struct environmental_msg msg = MSG_TO_ENVIRONMENTAL_MSG(state_object->msg_buf);

		if (msg.type == ENVIRONMENTAL_SENSOR_SAMPLE_REQUEST) {
			/* On-demand sampling not supported - environmental data is sampled continuously */
			LOG_DBG("Environmental sample request ignored (continuous sampling mode)");
			return SMF_EVENT_HANDLED;
		}
	}

	return SMF_EVENT_PROPAGATE;
}

static void env_module_thread(void)
{
	int err;
	int task_wdt_id;
	const uint32_t wdt_timeout_ms =
		(CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS * MSEC_PER_SEC);
	const uint32_t execution_time_ms =
		(CONFIG_APP_ENVIRONMENTAL_MSG_PROCESSING_TIMEOUT_SECONDS * MSEC_PER_SEC);
	const k_timeout_t zbus_wait_ms = K_MSEC(wdt_timeout_ms - execution_time_ms);
	static struct environmental_state_object environmental_state = {
		.bme680 = DEVICE_DT_GET(DT_NODELABEL(bme680)),
		.bmi270 = DEVICE_DT_GET(DT_NODELABEL(accelerometer_hp)),
		.adxl367 = DEVICE_DT_GET(DT_NODELABEL(accelerometer_lp)),
	};

	LOG_DBG("Environmental module task started");

	/* Ring buffer removed - batch message accumulates samples directly */

	/* Initialize sensor triggers and start periodic sampling */
	err = sensors_init(environmental_state.bmi270, environmental_state.adxl367, environmental_state.bme680);
	if (err) {
		LOG_ERR("sensors_init, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	task_wdt_id = task_wdt_add(wdt_timeout_ms, env_wdt_callback, (void *)k_current_get());
	if (task_wdt_id < 0) {
		LOG_ERR("Failed to add task to watchdog: %d", task_wdt_id);
		SEND_FATAL_ERROR();
		return;
	}

	smf_set_initial(SMF_CTX(&environmental_state), &states[STATE_RUNNING]);

	while (true) {
		err = task_wdt_feed(task_wdt_id);
		if (err) {
			LOG_ERR("task_wdt_feed, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = zbus_sub_wait_msg(&environmental,
					&environmental_state.chan,
					environmental_state.msg_buf,
					zbus_wait_ms);
		if (err == -ENOMSG) {
			continue;
		} else if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = smf_run_state(SMF_CTX(&environmental_state));
		if (err) {
			LOG_ERR("smf_run_state(), error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	}
}

K_THREAD_DEFINE(environmental_module_thread_id,
			CONFIG_APP_ENVIRONMENTAL_THREAD_STACK_SIZE,
		env_module_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
