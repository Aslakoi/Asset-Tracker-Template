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
#include "environmental_msgq.h"
#include "../storage/storage.h"
#include "../led/led.h"

#define FS 100 /* Sampling frequency in Hz */
#define FS_MS 1000 / FS /* Sampling frequency in milliseconds (100ms at 10 Hz) */

#define ENV_FS 0.1 /* Environmental sensor (BME680) sampling frequency in Hz */
#define ENV_FS_MS 10000/* Environmental sensor sampling frequency in milliseconds (1000/0.1) */
#define ENV_SAMPLES_BETWEEN_PUBLISH FS * 10 /* Include environmental data in every Nth IMU message */

/* Batch message accumulation */
static struct environmental_msg batch_msg = {
	.type = ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE,
	.sample_count = 0,
	.batch_timestamp_ms = 0,
	.pressure = 0,
};
static uint8_t imu_sample_count = 0;  /* Counter for IMU samples to include pressure periodically */

/* Latest environmental sensor readings (updated at 0.1 Hz) */
static int32_t latest_pressure_pa = 0;  /* Pressure in Pa as fixed-point int32_t */

/* Debug counter for periodic sensor reading printouts */
static uint32_t batch_publish_count = 0;
#define DEBUG_PRINT_INTERVAL 10  /* Print detailed sensor data every N batches */

LOG_MODULE_REGISTER(environmental, CONFIG_APP_ENVIRONMENTAL_LOG_LEVEL);

/* LED indicator colors - sent via zbus led_chan */
static const struct led_msg led_purple = {
	.type = LED_RGB_SET,
	.red = 128, .green = 0, .blue = 128,    /* Purple: waiting for startup */
	.duration_on_msec = 1000, .duration_off_msec = 1000,
	.repetitions = -1  /* Infinite blinking */
};

static const struct led_msg led_red = {
	.type = LED_RGB_SET,
	.red = 255, .green = 0, .blue = 0,      /* Red: sampling in progress */
	.duration_on_msec = 1000, .duration_off_msec = 1000,
	.repetitions = -1
};

static const struct led_msg led_green = {
	.type = LED_RGB_SET,
	.red = 0, .green = 255, .blue = 0,      /* Green: storage full */
	.duration_on_msec = 1000, .duration_off_msec = 1000,
	.repetitions = -1
};

/* Helper function to send LED messages via zbus */
static int send_led_message(const struct led_msg *led_msg)
{
	int err = zbus_chan_pub(&led_chan, led_msg, K_MSEC(100));
	if (err) {
		LOG_WRN("Failed to publish LED message: %d", err);
		return err;
	}
	return 0;
}

/* Environmental data now flows via message queue (environmental_msgq.h) only,not zbus.
 * Channel definition below is only for storage_data structure compatibility.
 * It won't be used for pub/sub - storage reads directly from msgq.
 */
ZBUS_CHAN_DEFINE(environmental_chan,
		 struct environmental_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

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

	//float accel_hp[3][SAMPLES_PER_BATCH];
	//float gyro_hp[3][SAMPLES_PER_BATCH];

	const struct device *const adxl367;

	//float accel_lp[3][SAMPLES_PER_BATCH];
};



/* Global sensor device pointers for work handlers */
static const struct device *g_bmi270;
static const struct device *g_adxl367;
static const struct device *g_bme680;

/* Forward declarations */
static void sample_publish_work_handler(struct k_work *work);
static void sample_collect_work_handler(struct k_work *work);
static void env_sample_work_handler(struct k_work *work);
static void startup_delay_work_handler(struct k_work *work);
static void led_update_work_handler(struct k_work *work);
static int sensors_init(const struct device *bmi270, const struct device *adxl367, const struct device *bme680);

/* Tracking whether sampling has been started */
static bool sampling_started = false;

/* Track current LED color for periodic updates */
static const struct led_msg *current_led = NULL;

/* Dedicated workqueue for environmental sampling at priority 11.
 * Priority 11 is between storage thread (12) and system workqueue (-1).
 * This prevents environmental work from starving the storage thread while
 * allowing environmental to be scheduled when storage is not blocking on I/O.
 */
static struct k_work_q environmental_workqueue;
static K_THREAD_STACK_DEFINE(environmental_workqueue_stack, 2048);

/* Work items for sensor sampling - use dedicated environmental workqueue */
static K_WORK_DELAYABLE_DEFINE(sample_publish_work, sample_publish_work_handler);
static K_WORK_DELAYABLE_DEFINE(sample_collect_work, sample_collect_work_handler);
static K_WORK_DELAYABLE_DEFINE(env_sample_work, env_sample_work_handler);
static K_WORK_DELAYABLE_DEFINE(startup_delay_work, startup_delay_work_handler);
static K_WORK_DELAYABLE_DEFINE(led_update_work, led_update_work_handler);

/* State machine no longer used - environmental is fully asynchronous */

/* Publish accumulated batch message */
static void sample_publish_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (batch_msg.sample_count == 0) {
		LOG_WRN("sample_publish_work_handler: batch_msg.sample_count is 0");
		return;
	}

	if (storage_full) {
		LOG_DBG("Storage full: skipping batch publish");
		batch_msg.sample_count = 0;
		return;
	}

	LOG_INF("Publishing batch with %d samples (msgq)",
		batch_msg.sample_count);

	/* Periodic debug print: show all sensor readings from first sample */
	batch_publish_count++;
	if (batch_publish_count % DEBUG_PRINT_INTERVAL == 0) {
		float accel_hp_x = (float)batch_msg.accel_hp[0][0] / ACCEL_SCALE;
		float accel_hp_y = (float)batch_msg.accel_hp[1][0] / ACCEL_SCALE;
		float accel_hp_z = (float)batch_msg.accel_hp[2][0] / ACCEL_SCALE;
		float gyro_hp_x = (float)batch_msg.gyro_hp[0][0] / GYRO_SCALE;
		float gyro_hp_y = (float)batch_msg.gyro_hp[1][0] / GYRO_SCALE;
		float gyro_hp_z = (float)batch_msg.gyro_hp[2][0] / GYRO_SCALE;
		float accel_lp_x = (float)batch_msg.accel_lp[0][0] / ACCEL_SCALE;
		float accel_lp_y = (float)batch_msg.accel_lp[1][0] / ACCEL_SCALE;
		float accel_lp_z = (float)batch_msg.accel_lp[2][0] / ACCEL_SCALE;
		
		LOG_DBG("=== SENSOR READINGS (Sample 0) ===");
		LOG_DBG("  BMI270 Accel: (%.3f, %.3f, %.3f) g", (double)accel_hp_x, (double)accel_hp_y, (double)accel_hp_z);
		LOG_DBG("  BMI270 Gyro:  (%.2f, %.2f, %.2f) dps", (double)gyro_hp_x, (double)gyro_hp_y, (double)gyro_hp_z);
		LOG_DBG("  ADXL367 Accel: (%.3f, %.3f, %.3f) g", (double)accel_lp_x, (double)accel_lp_y, (double)accel_lp_z);
		LOG_DBG("  BME680 Pressure: %d Pa", batch_msg.pressure);
		LOG_DBG("  Timestamp: %u ms, Samples in batch: %u", 
			batch_msg.batch_timestamp_ms, batch_msg.sample_count);
		LOG_DBG("=== END SENSOR READINGS ===");
	}

	/* Non-blocking write to environmental msgq.
	* If queue is full (128 slots), drop the batch as backpressure signal.
	* In practice, queue provides ~12+ seconds of buffering at 10 Hz sampling,
	* so occasional slow flash writes (50-200ms) are absorbed without loss.
	* Queue only fills if storage thread is blocked for >2 seconds.
	*/
	err = environmental_msgq_write(&batch_msg, K_NO_WAIT);
	if (err) {
		LOG_WRN("environmental_msgq_write failed: %d (msgq full or timeout)", err);
		/* Don't retry - the msgq full is backpressure. Storage thread will drain it.
		 * Drop this batch to prevent sample accumulation. */
		batch_msg.sample_count = 0;
		return;
	}

	/* Reset batch message for next batch only after successful write */
	batch_msg.sample_count = 0;
}

/* Periodic work handler to sample sensors at x Hz and accumulate into batch message */
static void sample_collect_work_handler(struct k_work *work)
{
	int err;
	uint8_t idx;  /* Index into batch arrays */

	ARG_UNUSED(work);

	if (storage_full) {
		LOG_DBG("sample_collect_work_handler: storage full, stopping IMU collection");
		return;
	}

	/* If batch is full, don't collect more samples until it's published */
	if (batch_msg.sample_count >= SAMPLES_PER_BATCH) {
		LOG_WRN("Batch buffer full, skipping sample");
		return;
	}

	idx = batch_msg.sample_count;

	/* Fetch data from BMI270 (accelerometer and gyroscope) */
	if (g_bmi270 && device_is_ready(g_bmi270)) {
		struct sensor_value accel_vals[3] = { 0 };
		struct sensor_value gyro_vals[3] = { 0 };
		double accel_d[3], gyro_d[3];

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

		accel_d[0] = sensor_value_to_double(&accel_vals[0]);
		accel_d[1] = sensor_value_to_double(&accel_vals[1]);
		accel_d[2] = sensor_value_to_double(&accel_vals[2]);

		/* Convert to fixed-point: value * 1000 = g */
		batch_msg.accel_hp[0][idx] = (int16_t)(accel_d[0] * ACCEL_SCALE);
		batch_msg.accel_hp[1][idx] = (int16_t)(accel_d[1] * ACCEL_SCALE);
		batch_msg.accel_hp[2][idx] = (int16_t)(accel_d[2] * ACCEL_SCALE);

		err = sensor_channel_get(g_bmi270, SENSOR_CHAN_GYRO_XYZ, gyro_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get gyro_xyz, error: %d", err);
			return;
		}

		gyro_d[0] = sensor_value_to_double(&gyro_vals[0]);
		gyro_d[1] = sensor_value_to_double(&gyro_vals[1]);
		gyro_d[2] = sensor_value_to_double(&gyro_vals[2]);

		/* Convert to fixed-point: value * 100 = dps */
		batch_msg.gyro_hp[0][idx] = (int16_t)(gyro_d[0] * GYRO_SCALE);
		batch_msg.gyro_hp[1][idx] = (int16_t)(gyro_d[1] * GYRO_SCALE);
		batch_msg.gyro_hp[2][idx] = (int16_t)(gyro_d[2] * GYRO_SCALE);

		LOG_DBG("BMI270 sampled[%d]: accel_hp[%.2f, %.2f, %.2f] g, gyro_hp[%.2f, %.2f, %.2f] dps",
			idx, accel_d[0], accel_d[1], accel_d[2],
			gyro_d[0], gyro_d[1], gyro_d[2]);
	}

	/* Fetch data from ADXL367 (accelerometer - low power) */
	if (g_adxl367 && device_is_ready(g_adxl367)) {
		struct sensor_value accel_vals[3] = { 0 };
		double accel_lp_d[3];

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

		accel_lp_d[0] = sensor_value_to_double(&accel_vals[0]);
		accel_lp_d[1] = sensor_value_to_double(&accel_vals[1]);
		accel_lp_d[2] = sensor_value_to_double(&accel_vals[2]);

		/* Convert to fixed-point: value * 1000 = g */
		batch_msg.accel_lp[0][idx] = (int16_t)(accel_lp_d[0] * ACCEL_SCALE);
		batch_msg.accel_lp[1][idx] = (int16_t)(accel_lp_d[1] * ACCEL_SCALE);
		batch_msg.accel_lp[2][idx] = (int16_t)(accel_lp_d[2] * ACCEL_SCALE);

		LOG_DBG("ADXL367 sampled[%d]: accel_lp[%.2f, %.2f, %.2f] g",
			idx, accel_lp_d[0], accel_lp_d[1], accel_lp_d[2]);
	}

	/* Initialize batch timestamp on first sample (unix time in ms or uptime in ms) */
	if (batch_msg.sample_count == 0) {
		uint32_t sample_time_ms = k_uptime_get();
		uint64_t unix_time_ms = sample_time_ms;

		err = date_time_now(&unix_time_ms);
		if (err == 0) {
			/* Unix time obtained successfully */
			batch_msg.batch_timestamp_ms = (uint32_t)(unix_time_ms & 0xFFFFFFFFUL);
		} else if (err == -ENODATA) {
			/* Not synced yet, use uptime */
			batch_msg.batch_timestamp_ms = sample_time_ms;
		} else {
			LOG_WRN("date_time_now error: %d", err);
			batch_msg.batch_timestamp_ms = sample_time_ms;
		}
	}

	/* Store timestamp delta as sample_index * sample_period (100ms at 10 Hz).
	 * This avoids tracking absolute time and prevents overflow issues.
	 * Delta is approximate but accurate for periodic sampling.
	 */
	uint16_t delta_ms = idx * FS_MS;  /* 100ms per sample at 10 Hz */
	if (delta_ms > UINT16_MAX) {
		delta_ms = UINT16_MAX;  /* Clamp to max uint16_t */
		LOG_WRN("Timestamp delta exceeds uint16_t at sample %d", idx);
	}
	batch_msg.timestamp_delta_ms[idx] = delta_ms;

	/* Include pressure data once per batch when available */
	imu_sample_count++;
	if (imu_sample_count >= ENV_SAMPLES_BETWEEN_PUBLISH && batch_msg.sample_count == 0) {
		batch_msg.pressure = latest_pressure_pa;
		imu_sample_count = 0;
		LOG_DBG("Batch pressure updated: %d Pa", batch_msg.pressure);
	}

	/* Increment sample count */
	batch_msg.sample_count++;

	/* Trigger publish when batch is full */
	if (batch_msg.sample_count >= SAMPLES_PER_BATCH) {
		LOG_DBG("Batch full (%d samples), submitting publish work", batch_msg.sample_count);
		k_work_schedule_for_queue(&environmental_workqueue, &sample_publish_work, K_NO_WAIT);
	}

	/* Work completes without rescheduling - only fires on interrupt triggers */
	return;
}

/* Periodic work handler to sample BME680 environmental sensor at 0.1 Hz */
static void env_sample_work_handler(struct k_work *work)
{
	int err;
	struct sensor_value press = { 0 };

	ARG_UNUSED(work);

	if (storage_full) {
		LOG_DBG("env_sample_work_handler: storage full, stopping environmental sampling");
		return;
	}

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

	/* Update latest pressure value as fixed-point int32_t */
	double press_d = sensor_value_to_double(&press);
	latest_pressure_pa = (int32_t)press_d;

	LOG_INF("env_sample_work_handler: BME680 pressure sampled - press=%.2f Pa",
		press_d);

reschedule_env:
	/* Reschedule for next 0.1 Hz sampling interval */
	k_work_schedule_for_queue(&environmental_workqueue, &env_sample_work, K_MSEC(ENV_FS_MS));
}

/* Sensor trigger callback for interrupt-based triggers */
static void sensor_trigger_callback(const struct device *sensor, const struct sensor_trigger *trigger)
{
	ARG_UNUSED(trigger);

	LOG_DBG("Sensor data-ready trigger from %s, submitting sample work", sensor->name);
	/* Offload work to environmental workqueue - keep ISR short.
	 * This follows the Zephyr ISR offloading pattern: signal a work item
	 * to do interrupt-related processing outside of ISR context.
	 * Use dedicated environmental workqueue at priority 11 to prevent starvation of storage thread.
	 */
	k_work_schedule_for_queue(&environmental_workqueue, &sample_collect_work, K_NO_WAIT);
}

/* Periodic LED update handler - resends LED message to keep pattern active */
static void led_update_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (storage_full) {
		/* Storage full - stop LED updates */
		return;
	}

	/* Resend current LED message to override any transient messages from main */
	if (current_led) {
		send_led_message(current_led);
	}

	/* Reschedule for next update (every 5 seconds) */
	k_work_schedule_for_queue(&environmental_workqueue, &led_update_work, K_SECONDS(15));
}

/* Startup delay work handler - called after delay to begin sampling */
static void startup_delay_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (sampling_started) {
		LOG_WRN("Sampling already started, skipping duplicate startup");
		return;
	}

	LOG_INF("Startup delay complete, beginning environmental sampling");

	err = sensors_init(g_bmi270, g_adxl367, g_bme680);
	if (err) {
		LOG_ERR("sensors_init failed: %d", err);
		SEND_FATAL_ERROR();
	}

	sampling_started = true;

	/* Indicate sampling is now active with red LED */
	current_led = &led_red;
	send_led_message(&led_red);

	/* Start periodic LED update to keep pattern active (every 5 seconds) */
	k_work_schedule_for_queue(&environmental_workqueue, &led_update_work, K_SECONDS(15));
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

		/* Do not use ADXL367 as an independent pacing trigger.
		 * We still read ADXL367 in sample_collect_work_handler().
		 * Using only BMI270 trigger reduces collection frequency to actual 25 Hz
		 * instead of ~50 Hz with both sensors triggering.
		 */
		/* err = sensor_trigger_set(adxl367, &trig, sensor_trigger_callback);
		if (err) {
			LOG_ERR("ADXL367 data-ready trigger not supported on this device, error: %d", err);
		} else {
			LOG_INF("ADXL367 data-ready trigger configured successfully");
			adxl367_trigger_ok = 1;
		}
		*/
	}

	/* Only start periodic sampling as fallback if trigger setup failed completely */
	if (!bmi270_trigger_ok && !adxl367_trigger_ok) {
		LOG_INF("Interrupt-based triggers not available, starting periodic sampling at %d Hz (every %d ms)",
			FS, FS_MS);
		err = k_work_schedule_for_queue(&environmental_workqueue, &sample_collect_work, K_MSEC(FS_MS));
		if (err < 0) {
			LOG_ERR("k_work_schedule_for_queue sample_collect_work, error: %d", err);
			return err;
		}
	} else {
		LOG_INF("Interrupt-driven sampling configured (no periodic fallback)");
	}

	/* Start environmental sensor (BME680) sampling at 0.1 Hz */
	if (bme680 && device_is_ready(bme680)) {
		err = k_work_schedule_for_queue(&environmental_workqueue, &env_sample_work, K_MSEC(ENV_FS_MS));
		if (err < 0) {
			LOG_ERR("k_work_schedule_for_queue env_sample_work, error: %d", err);
			return err;
		}
		LOG_INF("Environmental sensor (BME680) sampling scheduled at %.1f Hz", ENV_FS);
	} else {
		LOG_WRN("BME680 device not ready during initialization, environmental sampling disabled");
	}

	return 0;
}

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

static void env_module_thread(void)
{
	int err;
	int task_wdt_id;
	const uint32_t wdt_timeout_ms =
		(CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS * MSEC_PER_SEC);
	static struct environmental_state_object environmental_state = {
		.bme680 = DEVICE_DT_GET(DT_NODELABEL(bme680)),
		.bmi270 = DEVICE_DT_GET(DT_NODELABEL(accelerometer_hp)),
		.adxl367 = DEVICE_DT_GET(DT_NODELABEL(accelerometer_lp)),
	};

	LOG_DBG("Environmental module task started");

	/* Initialize dedicated environmental workqueue at priority 13.
	 * Priority 13 is LOWER priority than storage thread (12).
	 * Lower number = higher priority in Zephyr.
	 * This allows storage thread to preempt environmental work and drain the queue,
	 * preventing environmental from starving the storage thread.
	 */
	k_work_queue_init(&environmental_workqueue);

	k_work_queue_start(&environmental_workqueue, environmental_workqueue_stack,
			   K_THREAD_STACK_SIZEOF(environmental_workqueue_stack),
			   13, NULL);

	/* Store sensor device pointers globally for work handlers */
	g_bmi270 = environmental_state.bmi270;
	g_adxl367 = environmental_state.adxl367;
	g_bme680 = environmental_state.bme680;

	/* Schedule sensor initialization and sampling start with configured delay */
	uint32_t startup_delay_ms = CONFIG_APP_ENVIRONMENTAL_STARTUP_DELAY_SECONDS * MSEC_PER_SEC;
	
	/* Indicate LED status: purple if waiting for startup delay, red if starting immediately */
	if (startup_delay_ms > 0) {
		LOG_INF("Environmental sampling delayed by %u seconds", 
			CONFIG_APP_ENVIRONMENTAL_STARTUP_DELAY_SECONDS);
		current_led = &led_purple;
		send_led_message(&led_purple);
		/* Start periodic updates to keep purple LED active during wait */
		err = k_work_schedule_for_queue(&environmental_workqueue, &led_update_work, K_SECONDS(15));
		if (err < 0) {
			LOG_WRN("Failed to schedule LED update work: %d", err);
			return;
		}
	} else {
		current_led = &led_red;
		send_led_message(&led_red);
		/* Start periodic updates to keep red LED active during sampling */
		err = k_work_schedule_for_queue(&environmental_workqueue, &led_update_work, K_SECONDS(15));
		if (err < 0) {
			LOG_WRN("Failed to schedule LED update work: %d", err);
			return;
		}
	}

	err = k_work_schedule_for_queue(&environmental_workqueue, &startup_delay_work, K_MSEC(startup_delay_ms));
	if (err < 0) {
		LOG_ERR("Failed to schedule startup delay work: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	task_wdt_id = task_wdt_add(wdt_timeout_ms, env_wdt_callback, (void *)k_current_get());
	if (task_wdt_id < 0) {
		LOG_ERR("Failed to add task to watchdog: %d", task_wdt_id);
		SEND_FATAL_ERROR();
		return;
	}

	/* State machine no longer needed - environmental is fully asynchronous via triggers and work items */

	while (true) {
		err = task_wdt_feed(task_wdt_id);
		if (err) {
			LOG_ERR("task_wdt_feed, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		/* Environmental data is sampled continuously via sensor triggers and work items.
		 * Thread only needs to feed watchdog periodically. No zbus messages expected.
		 * Sleep for 50ms to allow watchdog feeding without blocking the system.
		 */
		k_sleep(K_MSEC(50));
	}
}

K_THREAD_DEFINE(environmental_module_thread_id,
			CONFIG_APP_ENVIRONMENTAL_THREAD_STACK_SIZE,
		env_module_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
