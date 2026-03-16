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

#define FS 50 /* Sampling frequency in Hz */
#define FS_MS 1000 / FS /* Sampling frequency in milliseconds (1000/1) */

#define ENV_FS 1 /* Environmental sensor (BME680) sampling frequency in Hz */
#define ENV_FS_MS 1000 / ENV_FS /* Environmental sensor sampling frequency in milliseconds */
#define ENV_SAMPLES_BETWEEN_PUBLISH 50 /* Include environmental data in every Nth IMU message */

/* Ring buffer for sensor samples */
#define SAMPLE_BUFFER_SIZE 2048
#define SAMPLES_PER_BATCH 10  /* Publish every x samples to batch and reduce zbus load */
static uint8_t sample_buffer[SAMPLE_BUFFER_SIZE];
static struct ring_buf sample_ring_buf;
static uint8_t sample_count = 0;  /* Counter for batching */
static uint8_t imu_sample_count = 0;  /* Counter for IMU samples to include env data periodically */

/* Latest environmental sensor readings (updated at 1 Hz) */
static float latest_temperature = 0.0f;
static float latest_pressure = 0.0f;
static float latest_humidity = 0.0f;
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

	/* Sensor values */
	int16_t temperature;
	int16_t pressure;
	int16_t humidity;

	const struct device *const bmi270;

	float accel_hp[3];
	float gyro_hp[3];

	const struct device *const adxl367;

	float accel_lp[3];
};

/* Sample data structure for storing */
struct sensor_sample {
	float accel_hp[3];
	float gyro_hp[3];
	float accel_lp[3];
	int64_t timestamp;
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
static K_WORK_DEFINE(sample_publish_work, sample_publish_work_handler);
static K_WORK_DELAYABLE_DEFINE(sample_collect_work, sample_collect_work_handler);
static K_WORK_DELAYABLE_DEFINE(env_sample_work, env_sample_work_handler);

/* State machine definition */
static const struct smf_state states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(NULL, state_running_run, NULL, NULL, NULL),
};

/* Process samples from ring buffer and publish them to zbus */
static void sample_publish_work_handler(struct k_work *work)
{
	int err;
	struct sensor_sample sample;

	ARG_UNUSED(work);

	/* Process all available samples in ring buffer */
	while (ring_buf_get(&sample_ring_buf, (uint8_t *)&sample, sizeof(sample)) == sizeof(sample)) {
		struct environmental_msg msg = {
			.type = ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE,
			.accel_hp[0] = sample.accel_hp[0],
			.accel_hp[1] = sample.accel_hp[1],
			.accel_hp[2] = sample.accel_hp[2],
			.gyro_hp[0] = sample.gyro_hp[0],
			.gyro_hp[1] = sample.gyro_hp[1],
			.gyro_hp[2] = sample.gyro_hp[2],
			.accel_lp[0] = sample.accel_lp[0],
			.accel_lp[1] = sample.accel_lp[1],
			.accel_lp[2] = sample.accel_lp[2],
			.timestamp = sample.timestamp,
		};

		/* Include environmental data every ENV_SAMPLES_BETWEEN_PUBLISH samples */
		imu_sample_count++;
		if (imu_sample_count >= ENV_SAMPLES_BETWEEN_PUBLISH) {
			msg.temperature = latest_temperature;
			msg.pressure = latest_pressure;
			msg.humidity = latest_humidity;
			imu_sample_count = 0;
			LOG_INF("Publishing sensor sample with ENV: accel_hp[%.2f, %.2f, %.2f] g, "
				"gyro_hp[%.2f, %.2f, %.2f] dps, accel_lp[%.2f, %.2f, %.2f] g, "
				"temp=%.2f C, press=%.2f Pa, humidity=%.2f %%",
				(double)msg.accel_hp[0], (double)msg.accel_hp[1], (double)msg.accel_hp[2],
				(double)msg.gyro_hp[0], (double)msg.gyro_hp[1], (double)msg.gyro_hp[2],
				(double)msg.accel_lp[0], (double)msg.accel_lp[1], (double)msg.accel_lp[2],
				(double)msg.temperature, (double)msg.pressure, (double)msg.humidity);
		} else {
			LOG_INF("Publishing sensor sample: accel_hp[%.2f, %.2f, %.2f] g, "
				"gyro_hp[%.2f, %.2f, %.2f] dps, accel_lp[%.2f, %.2f, %.2f] g",
				(double)msg.accel_hp[0], (double)msg.accel_hp[1], (double)msg.accel_hp[2],
				(double)msg.gyro_hp[0], (double)msg.gyro_hp[1], (double)msg.gyro_hp[2],
				(double)msg.accel_lp[0], (double)msg.accel_lp[1], (double)msg.accel_lp[2]);
		}

		err = zbus_chan_pub(&environmental_chan, &msg, PUB_TIMEOUT);
		if (err) {
			LOG_ERR("zbus_chan_pub, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	}
}

/* Periodic work handler to sample sensors at 50 Hz */
static void sample_collect_work_handler(struct k_work *work)
{
	int err;
	struct sensor_sample sample = { 0 };

	ARG_UNUSED(work);

	/* Fetch data from BMI270 (accelerometer and gyroscope - high performance) */
	if (g_bmi270 && device_is_ready(g_bmi270)) {
		struct sensor_value accel_vals[3] = { 0 };
		struct sensor_value gyro_vals[3] = { 0 };

		/* Full fetch ensures atomic read of all sensor data */
		err = sensor_sample_fetch(g_bmi270);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_sample_fetch bmi270, error: %d", err);
			goto reschedule;
		}

		err = sensor_channel_get(g_bmi270, SENSOR_CHAN_ACCEL_XYZ, accel_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get accel_xyz, error: %d", err);
			goto reschedule;
		}

		sample.accel_hp[0] = sensor_value_to_double(&accel_vals[0]);
		sample.accel_hp[1] = sensor_value_to_double(&accel_vals[1]);
		sample.accel_hp[2] = sensor_value_to_double(&accel_vals[2]);

		err = sensor_channel_get(g_bmi270, SENSOR_CHAN_GYRO_XYZ, gyro_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get gyro_xyz, error: %d", err);
			goto reschedule;
		}

		sample.gyro_hp[0] = sensor_value_to_double(&gyro_vals[0]);
		sample.gyro_hp[1] = sensor_value_to_double(&gyro_vals[1]);
		sample.gyro_hp[2] = sensor_value_to_double(&gyro_vals[2]);

		LOG_DBG("BMI270 sampled: accel_hp[%.2f, %.2f, %.2f] g, gyro_hp[%.2f, %.2f, %.2f] dps",
			(double)sample.accel_hp[0], (double)sample.accel_hp[1], (double)sample.accel_hp[2],
			(double)sample.gyro_hp[0], (double)sample.gyro_hp[1], (double)sample.gyro_hp[2]);
	}

	/* Fetch data from ADXL367 (accelerometer - low power) */
	if (g_adxl367 && device_is_ready(g_adxl367)) {
		struct sensor_value accel_vals[3] = { 0 };

		/* Full fetch ensures atomic read of all sensor data */
		err = sensor_sample_fetch(g_adxl367);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_sample_fetch adxl367, error: %d", err);
			goto reschedule;
		}

		err = sensor_channel_get(g_adxl367, SENSOR_CHAN_ACCEL_XYZ, accel_vals);
		if (err && err != -ENOTSUP) {
			LOG_ERR("sensor_channel_get accel_lp_xyz, error: %d", err);
			goto reschedule;
		}

		sample.accel_lp[0] = sensor_value_to_double(&accel_vals[0]);
		sample.accel_lp[1] = sensor_value_to_double(&accel_vals[1]);
		sample.accel_lp[2] = sensor_value_to_double(&accel_vals[2]);

		LOG_DBG("ADXL367 sampled: accel_lp[%.2f, %.2f, %.2f] g",
			(double)sample.accel_lp[0], (double)sample.accel_lp[1], (double)sample.accel_lp[2]);
	}

	sample.timestamp = k_uptime_get();
	err = date_time_now(&sample.timestamp);
	if (err != 0 && err != -ENODATA) {
		LOG_WRN("date_time_now, error: %d", err);
	}

	/* Add sample to ring buffer */
	if (ring_buf_put(&sample_ring_buf, (uint8_t *)&sample, sizeof(sample)) != sizeof(sample)) {
		LOG_WRN("Sample ring buffer full, dropping sample");
	}

	/* Trigger sample publishing work after batching SAMPLES_PER_BATCH samples */
	sample_count++;
	if (sample_count >= SAMPLES_PER_BATCH) {
		sample_count = 0;
		k_work_submit(&sample_publish_work);
	}

	/* Work completes without rescheduling - only fires on interrupt triggers */
	return;

reschedule:
	/* Error fallback: reschedule work to retry if sample collection fails.
	 * This acts as a safety net to ensure samples are eventually collected
	 * even if something goes wrong with the interrupt. */
	LOG_ERR("Sample collection error detected, enabling periodic fallback (%d ms)", FS_MS);
	k_work_schedule(&sample_collect_work, K_MSEC(FS_MS));
}

/* Periodic work handler to sample BME680 environmental sensor at 1 Hz */
static void env_sample_work_handler(struct k_work *work)
{
	int err;
	struct sensor_value temp = { 0 };
	struct sensor_value press = { 0 };
	struct sensor_value humidity = { 0 };

	ARG_UNUSED(work);

	LOG_DBG("env_sample_work_handler: starting BME680 sample");

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

	err = sensor_channel_get(g_bme680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err) {
		LOG_WRN("env_sample_work_handler: sensor_channel_get TEMP error: %d", err);
		goto reschedule_env;
	}

	err = sensor_channel_get(g_bme680, SENSOR_CHAN_PRESS, &press);
	if (err) {
		LOG_WRN("env_sample_work_handler: sensor_channel_get PRESS error: %d", err);
		goto reschedule_env;
	}

	err = sensor_channel_get(g_bme680, SENSOR_CHAN_HUMIDITY, &humidity);
	if (err) {
		LOG_WRN("env_sample_work_handler: sensor_channel_get HUMIDITY error: %d", err);
		goto reschedule_env;
	}

	/* Update latest environmental values */
	latest_temperature = (float)sensor_value_to_double(&temp);
	latest_pressure = (float)sensor_value_to_double(&press);
	latest_humidity = (float)sensor_value_to_double(&humidity);

	LOG_INF("env_sample_work_handler: BME680 sampled successfully - "
		"temp=%.2f C, press=%.2f Pa, humidity=%.2f %% "
		"(raw: temp.val1=%d temp.val2=%d, press.val1=%d, humid.val1=%d)",
		(double)latest_temperature, (double)latest_pressure, (double)latest_humidity,
		temp.val1, temp.val2, press.val1, humidity.val1);

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



static void sample_sensors(const struct device *const bme680)
{
	int err;
	struct sensor_value temp = { 0 };
	struct sensor_value press = { 0 };
	struct sensor_value humidity = { 0 };

	err = sensor_sample_fetch(bme680);
	if (err) {
		LOG_ERR("sensor_sample_fetch, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	err = sensor_channel_get(bme680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err) {
		LOG_ERR("sensor_channel_get, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	err = sensor_channel_get(bme680, SENSOR_CHAN_PRESS, &press);
	if (err) {
		LOG_ERR("sensor_channel_get, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	err = sensor_channel_get(bme680, SENSOR_CHAN_HUMIDITY, &humidity);
	if (err) {
		LOG_ERR("sensor_channel_get, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	struct environmental_msg msg = {
		.type = ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE,
		.temperature = sensor_value_to_double(&temp),
		.pressure = sensor_value_to_double(&press),
		.humidity = sensor_value_to_double(&humidity),
		.timestamp = k_uptime_get(),
	};

	err = date_time_now(&msg.timestamp);
	if (err != 0 && err != -ENODATA) {
		LOG_ERR("date_time_now, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}

	/* Log the environmental values and limit to 2 decimals */
	LOG_DBG("Temperature: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%",
		(double)msg.temperature, (double)msg.pressure, (double)msg.humidity);

	err = zbus_chan_pub(&environmental_chan, &msg, PUB_TIMEOUT);
	if (err) {
		LOG_ERR("zbus_chan_pub, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}
}

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
			LOG_DBG("Environmental values sample request received, getting data");
			sample_sensors(state_object->bme680);

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

	/* Initialize ring buffer for sensor samples */
	ring_buf_init(&sample_ring_buf, SAMPLE_BUFFER_SIZE, sample_buffer);

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
