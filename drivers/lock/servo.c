/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT lock_servo

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include <app/drivers/lock.h>

LOG_MODULE_REGISTER(servo, CONFIG_LOCK_LOG_LEVEL);

/** Feedback polling time (msec) */
#define FB_POLL_TIME 100

struct lock_servo_config {
	struct pwm_dt_spec servo;
	uint32_t open_pulse_us;
	uint32_t closed_pulse_us;
	struct adc_dt_spec adc;
	int32_t fb_gain;
	int32_t fb_offset_uv;
	int32_t max_target_err_us;
	uint32_t max_action_time_ms;
};

/*******************************************************************************
 * Private
 ******************************************************************************/

/**
 * @brief Set servo to the position given by the @p target_pulse_us.
 *
 * @param dev Lock device instance.
 * @param target_pulse_us Target pulse (usec).
 *
 * @retval 0 On success.
 * @retval -ETIMEDOUT If @p target_pulse is not reached.
 * @retval -errno Other negative errno in case of error.
 */
static int servo_set_pulse(const struct device *dev, uint32_t target_pulse_us)
{
	const struct lock_servo_config *config = dev->config;
	struct adc_sequence adc_seq;
	int16_t buf;
	int32_t pulse_mv;
	int32_t pulse_us;
	int32_t timeout = (int32_t)config->max_action_time_ms;
	bool finished = false;
	int ret;

	/* set target PWM pulse */
	ret = pwm_set_pulse_dt(&config->servo, PWM_USEC(target_pulse_us));
	if (ret < 0) {
		return ret;
	}

	/* read feedback to make sure we reach target pulse */
	adc_seq.buffer = &buf;
	adc_seq.calibrate = false;
	adc_seq.buffer_size = sizeof(buf);
	adc_seq.channels = BIT(config->adc.channel_id);
	adc_seq.resolution = config->adc.resolution;
	adc_seq.oversampling = config->adc.oversampling;
	adc_seq.options = NULL;

	while (!finished) {
		ret = adc_read(config->adc.dev, &adc_seq);
		if (ret < 0) {
			return ret;
		}

		pulse_mv = buf;
		ret = adc_raw_to_millivolts_dt(&config->adc, &pulse_mv);
		if (ret < 0) {
			return ret;
		}

		/* convert to usec */
		pulse_us = ((pulse_mv * 1000) - config->fb_offset_uv) /
			   config->fb_gain;

		if (target_pulse_us >= (pulse_us - config->max_target_err_us) &&
		    target_pulse_us <= (pulse_us + config->max_target_err_us)) {
			finished = true;
		} else {
			k_sleep(K_MSEC(FB_POLL_TIME));

			timeout -= FB_POLL_TIME;
			if (timeout <= 0) {
				LOG_ERR("Target reach timed out (t: %d, p: %d)",
					target_pulse_us, pulse_us);
				ret = -ETIMEDOUT;
				finished = true;
			}
		}
	}

	/* stop PWM (reached or timeout) */
	(void)pwm_set_pulse_dt(&config->servo, 0U);

	return ret;
}

/*******************************************************************************
 * API
 ******************************************************************************/

static int lock_servo_open(const struct device *dev)
{
	const struct lock_servo_config *config = dev->config;

	return servo_set_pulse(dev, config->open_pulse_us);
}

static int lock_servo_close(const struct device *dev)
{
	const struct lock_servo_config *config = dev->config;

	return servo_set_pulse(dev, config->closed_pulse_us);
}

static const struct lock_driver_api lock_servo_api = {
	.open = lock_servo_open,
	.close = lock_servo_close,
};

static int lock_servo_init(const struct device *dev)
{
	const struct lock_servo_config *config = dev->config;
	int ret;

	if (!pwm_is_ready_dt(&config->servo)) {
		LOG_ERR("Servo PWM controller not ready");
		return -ENODEV;
	}

	if (!adc_is_ready_dt(&config->adc)) {
		LOG_ERR("Servo feedback ADC device not ready");
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&config->adc);
	if (ret < 0) {
		LOG_ERR("Could not configure ADC cannel (%d)", ret);
		return ret;
	}

	return 0;
}

#define LOCK_SERVO_DEFINE(i)                                                   \
	static const struct lock_servo_config lock_servo_config_##i = {        \
		.servo = PWM_DT_SPEC_INST_GET(i),                              \
		.open_pulse_us = DT_INST_PROP(i, open_pulse_us),               \
		.closed_pulse_us = DT_INST_PROP(i, closed_pulse_us),           \
		.adc = ADC_DT_SPEC_INST_GET(i),                                \
		.fb_gain = DT_INST_PROP(i, fb_gain),                           \
		.fb_offset_uv = DT_INST_PROP(i, fb_offset_microvolt),          \
		.max_target_err_us = DT_INST_PROP(i, max_target_err_us),       \
		.max_action_time_ms = DT_INST_PROP(i, max_action_time_ms),     \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(i, lock_servo_init, NULL, NULL,                  \
			      &lock_servo_config_##i, POST_KERNEL,             \
			      CONFIG_LOCK_INIT_PRIORITY, &lock_servo_api);

DT_INST_FOREACH_STATUS_OKAY(LOCK_SERVO_DEFINE)
