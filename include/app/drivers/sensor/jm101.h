/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_JM101_H_
#define ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_JM101_H_

#include <zephyr/drivers/sensor.h>

/** @brief Any identifier (used to do 1:N identification). */
#define JM101_ID_ANY 0xFFFFU

/** @brief JM101 custom channels. */
enum jm101_channel {
	/** Fingerprint verification. */
	JM101_CHAN_FINGERPRINT = SENSOR_CHAN_PRIV_START,
};

/** @brief JM101 custom attributes. */
enum jm101_attribute {
	/** Fingerprint ID used when verifying. */
	JM101_ATTR_ID_NUM = SENSOR_ATTR_PRIV_START,
	/** Run the enrolling sequence. */
	JM101_ATTR_ENROLL,
	/** Emtpies the fingerprint database. */
	JM101_ATTR_EMPTYDB,
};

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_JM101_H_ */
