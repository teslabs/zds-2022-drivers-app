/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <app/drivers/lock.h>
#include <app/drivers/sensor/jm101.h>

static const struct device *fpreader = DEVICE_DT_GET(DT_NODELABEL(fpreader));
static const struct device *lock = DEVICE_DT_GET(DT_NODELABEL(lock));

/** @brief Lock Service UUID (equal to Nordic LBS). */
#define BT_UUID_LOCK_VAL                                                       \
	BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_LOCK BT_UUID_DECLARE_128(BT_UUID_LOCK_VAL)

/** @brief Lock action (open/close) characteristic UUID. */
#define BT_UUID_LOCK_ACTION_VAL                                                \
	BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_LOCK_ACTION BT_UUID_DECLARE_128(BT_UUID_LOCK_ACTION_VAL)

/** @brief Bluetooth advertisement data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/** @brief Bluetooth service discovery data */
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LOCK_VAL),
};

/*******************************************************************************
 * Bluetooth services handling
 ******************************************************************************/

static ssize_t write_lock(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset,
			  uint8_t flags)
{
	uint8_t val;

	if (len != 1U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	val = *((uint8_t *)buf);

	if (val == 0x00U) {
		lock_close(lock);
	} else if (val == 0x01U) {
		lock_open(lock);
	} else {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	return len;
}

/* Lock Service Declaration */
BT_GATT_SERVICE_DEFINE(lock_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LOCK),
		       BT_GATT_CHARACTERISTIC(BT_UUID_LOCK_ACTION,
					      BT_GATT_CHRC_WRITE,
					      BT_GATT_PERM_WRITE, NULL,
					      write_lock, NULL), );

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/*******************************************************************************
 * Fingerprint handling
 ******************************************************************************/

static int enroll(void)
{
	const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	int ret;

	if (!device_is_ready(btn.port)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	if (gpio_pin_get_dt(&btn) > 0) {
		struct sensor_value val = {
			.val1 = 1U, /* store as fingerprint ID #1 */
			.val2 = 2U, /* number of readings */
		};

		/* empty database first */
		ret = sensor_attr_set(fpreader, JM101_CHAN_FINGERPRINT,
				      JM101_ATTR_EMPTYDB, NULL);
		if (ret < 0) {
			printk("Could not empty database\n");
			return ret;
		}

		printk("Place finger on sensor for enrollment\n");
		ret = sensor_attr_set(fpreader, JM101_CHAN_FINGERPRINT,
				      JM101_ATTR_ENROLL, &val);
		if (ret < 0) {
			printk("Enrollment failed\n");
			return ret;
		}

		printk("Enrollment finished successfully!\n");
	}

	return 0;
}

void fp_trig_handler(const struct device *dev,
		     const struct sensor_trigger *trigger)
{
	int ret;
	struct sensor_value val;

	ARG_UNUSED(trigger);

	printk("Finger detected, checking...\n");

	ret = sensor_sample_fetch_chan(dev, JM101_CHAN_FINGERPRINT);
	if (ret < 0) {
		printk("Verification failed\n");
		return;
	}

	(void)sensor_channel_get(dev, JM101_CHAN_FINGERPRINT, &val);

	printk("Verified fingerprint #%d, score: %d\n", val.val1, val.val2);

	printk("Opening lock...\n");
	ret = lock_open(lock);
	if (ret < 0) {
		printk("Could not open lock\n");
		return;
	}

	printk("Will close lock in 5 seconds...\n");
	k_sleep(K_SECONDS(5));

	ret = lock_close(lock);
	if (ret < 0) {
		printk("Could not close\n");
		return;
	}

	printk("Closed!\n");
}

/*******************************************************************************
 * Entry point
 ******************************************************************************/

int main(void)
{
	int ret;
	const struct sensor_trigger trig = {
		.type = SENSOR_TRIG_TAP,
		.chan = JM101_CHAN_FINGERPRINT,
	};

	if (!device_is_ready(fpreader)) {
		printk("Fingerprint reader not ready");
		return 0;
	}

	if (!device_is_ready(lock)) {
		printk("Lock not ready");
		return 0;
	}

	/* run enroll (will be skipped if sw0 is not pressed) */
	(void)enroll();

	/* configure fingerprint trigger */
	ret = sensor_trigger_set(fpreader, &trig, fp_trig_handler);
	if (ret < 0) {
		printk("Could not configure trigger");
	}

	/* enable Bluetooth and start advertising */
	ret = bt_enable(NULL);
	if (ret < 0) {
		printk("Bluetooth init failed (ret %d)\n", ret);
		return 0;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (ret < 0) {
		printk("Advertising failed to start (%d)", ret);
		return 0;
	}

	return 0;
}
