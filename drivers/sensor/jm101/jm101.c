/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zeantec_jm101

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>

#include <app/drivers/sensor/jm101.h>

LOG_MODULE_REGISTER(jm101, CONFIG_SENSOR_LOG_LEVEL);

/* packet buffer size, RX queue size */
#define PKT_BUF_SIZE  32U
#define RX_QUEUE_SIZE 2U

/* commands */
#define CMD_EMPTY      0x0DU
#define CMD_AUTOENROLL 0x31U
#define CMD_AUTOID     0x32U

/* packet field positions */
#define PKT_HEADER_POS	      0U
#define PKT_ADDR_POS	      2U
#define PKT_ID_POS	      6U
#define PKT_LEN_POS	      7U
#define PKT_CMD_POS	      9U
#define PKT_CONFIRM_POS	      9U
#define PKT_DATA_POS	      10U
#define PKT_SUM_POS(data_len) (PKT_DATA_POS + (data_len))

/* packet field values */
#define PKT_HEADER_VAL	      0xEF01U
#define PKT_LEN_VAL(data_len) (3U + (data_len))

/* packet total length */
#define PKT_LEN(data_len) (12U + (data_len))

/* packet checksum start position & number of bytes */
#define PKT_SUM_START	      PKT_ID_POS
#define PKT_SUM_CNT(data_len) (4U + (data_len))

/* packet IDs */
#define PKT_ID_CMD 0x01U

/* result codes */
#define RESULT_OK 0x00U

/* autoidentify sequence field positions & step IDs */
#define AUTOID_DATA_LEN	    5U
#define AUTOID_POS_STEP	    0U
#define AUTOID_POS_ID	    1U
#define AUTOID_POS_SCORE    3U
#define AUTOID_STEP_RESULTS 0x05U

/* autoenroll sequence field positions & step IDs */
#define AUTOENROLL_TX_DATA_LEN	5U
#define AUTOENROLL_RX_DATA_LEN	2U
#define AUTOENROLL_POS_ID	0U
#define AUTOENROLL_POS_FREQ	2U
#define AUTOENROLL_POS_STEP	0U
#define AUTOENROLL_STEP_STORAGE 6U

/** @brief JM101 data. */
struct jm101_data {
	/** RX queue buffer. */
	uint8_t rx_queue_buf[PKT_BUF_SIZE * RX_QUEUE_SIZE];
	/** RX/TX buffer. */
	uint8_t buf[PKT_BUF_SIZE];
	/** RX/TX buffer pointer. */
	uint8_t *buf_ptr;
	/** RX/TX buffer counter. */
	uint8_t buf_ctr;
	/** Expected reception data length. */
	uint8_t rx_data_len;
	/** ID used for verification. */
	uint16_t verify_id;
	/** Sensor value buffer. */
	struct sensor_value val;
	/** RX queue. */
	struct k_msgq rx_queue;
#ifdef CONFIG_JM101_TRIGGER
	/** Trigger handler. */
	sensor_trigger_handler_t trig_handler;
	/** Trigger work queue. */
	struct k_work trig_work;
	/** Touch GPIO callback. */
	struct gpio_callback touch_cb;
	/** Self reference (used in work queue context). */
	const struct device *dev;
#endif /* CONFIG_JM101_TRIGGER */
};

/** @brief JM101 configuration. */
struct jm101_config {
	/** Device address. */
	uint32_t addr;
	/** UART instance. */
	const struct device *uart;
#ifdef CONFIG_JM101_TRIGGER
	/** Touch input (optional). */
	struct gpio_dt_spec touch;
#endif
};

/** @brief JM101 UART configuration. */
static const struct uart_config uart_config = {
	.baudrate = 57600U,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_2,
};

/*******************************************************************************
 * Private
 ******************************************************************************/

/**
 * @brief Calculate checksum.
 *
 * @param buf Buffer to calculate checksum.
 * @param len Buffer length.
 *
 * @return Checksum.
 */
static uint16_t jm101_checksum(uint8_t *buf, uint8_t len)
{
	uint16_t sum = 0U;

	for (uint8_t i = 0U; i < len; i++) {
		sum += buf[i];
	}

	return sum;
}

/**
 * @brief UART RX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_rx_handler(const struct device *dev)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	int n;

	n = uart_fifo_read(config->uart, data->buf_ptr,
			   PKT_BUF_SIZE - data->buf_ctr);
	data->buf_ctr += (uint8_t)n;
	data->buf_ptr += (uint8_t)n;

	if (data->buf_ctr >= PKT_LEN(data->rx_data_len)) {
		LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "RX");
		if (k_msgq_put(&data->rx_queue, data->buf, K_NO_WAIT) < 0) {
			LOG_ERR("RX queue full, dropping packet");
		}
		data->buf_ctr = 0U;
		data->buf_ptr = data->buf;
	}
}

/**
 * @brief UART TX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_tx_handler(const struct device *dev)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	int n;

	if (data->buf_ctr > 0U) {
		n = uart_fifo_fill(config->uart, data->buf_ptr, data->buf_ctr);
		data->buf_ctr -= (uint8_t)n;
		data->buf_ptr += (uint8_t)n;
		return;
	}

	if (uart_irq_tx_complete(config->uart) > 0) {
		data->buf_ptr = data->buf;
		uart_irq_tx_disable(config->uart);
		uart_irq_rx_enable(config->uart);
	}
}

/**
 * @brief UART IRQ handler.
 *
 * @param dev UART device instance.
 * @param user_data User data (sensor instance).
 */
static void uart_cb_handler(const struct device *dev, void *user_data)
{
	const struct device *sensor = user_data;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
		if (uart_irq_rx_ready(dev)) {
			uart_cb_rx_handler(sensor);
		}

		if (uart_irq_tx_ready(dev)) {
			uart_cb_tx_handler(sensor);
		}
	}
}

#ifdef CONFIG_JM101_TRIGGER
/**
 * @brief Trigger work queue handler.
 *
 * @param item Work queue item.
 */
static void jm101_trig_work_handler(struct k_work *item)
{
	struct jm101_data *data =
		CONTAINER_OF(item, struct jm101_data, trig_work);
	const struct jm101_config *config = data->dev->config;

	if (data->trig_handler != NULL) {
		struct sensor_trigger trig = {
			.chan = JM101_CHAN_FINGERPRINT,
			.type = SENSOR_TRIG_TAP,
		};

		data->trig_handler(data->dev, &trig);

		/* re-enable touch interrupts (if handler did not disable) */
		if (data->trig_handler != NULL) {
			(void)gpio_pin_interrupt_configure_dt(
				&config->touch, GPIO_INT_EDGE_TO_ACTIVE);
		}
	}
}

/**
 * @brief Touch GPIO interrupt handler.
 *
 * @param dev GPIO device instance.
 * @param cb GPIO callback reference.
 * @param pins GPIO pins that triggered IRQ.
 */
static void jm101_touch_callback_handler(const struct device *port,
					 struct gpio_callback *cb,
					 gpio_port_pins_t pins)
{
	struct jm101_data *data = CONTAINER_OF(cb, struct jm101_data, touch_cb);
	const struct jm101_config *config = data->dev->config;

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* disable any new touch interrupts until work is processed */
	(void)gpio_pin_interrupt_configure_dt(&config->touch, GPIO_INT_DISABLE);

	k_work_submit(&data->trig_work);
}
#endif /* CONFIG_JM101_TRIGGER */

/**
 * @brief Send a command to the sensor.
 *
 * @param dev Sensor instance.
 * @param cmd Command to send.
 * @param[in] tx_data Data to send.
 * @param tx_data_len Data length.
 * @param rx_data_len Expected data reception length.
 *
 * @retval 0 Success.
 * @retval -EMSGSIZE If TX/RX data length is too large.
 */
static int jm101_send(const struct device *dev, uint8_t cmd,
		      const uint8_t *tx_data, uint8_t tx_data_len,
		      uint8_t rx_data_len)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	uint16_t checksum;

	if (PKT_LEN(tx_data_len) > sizeof(data->buf) ||
	    PKT_LEN(rx_data_len) > sizeof(data->buf)) {
		return -EMSGSIZE;
	}

	/* assemble request packet */
	sys_put_be16(PKT_HEADER_VAL, &data->buf[PKT_HEADER_POS]);
	sys_put_be32(config->addr, &data->buf[PKT_ADDR_POS]);
	data->buf[PKT_ID_POS] = PKT_ID_CMD;
	sys_put_be16(PKT_LEN_VAL(tx_data_len), &data->buf[PKT_LEN_POS]);
	data->buf[PKT_CMD_POS] = cmd;
	memcpy(&data->buf[PKT_DATA_POS], tx_data, tx_data_len);
	checksum = jm101_checksum(&data->buf[PKT_SUM_START],
				  PKT_SUM_CNT(tx_data_len));
	sys_put_be16(checksum, &data->buf[PKT_SUM_POS(tx_data_len)]);

	/* store expected reception data length */
	data->rx_data_len = rx_data_len;

	/* trigger transmission */
	data->buf_ctr = PKT_LEN(tx_data_len);
	data->buf_ptr = data->buf;
	LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "TX");
	uart_irq_rx_disable(config->uart);
	uart_irq_tx_enable(config->uart);

	return 0;
}

/**
 * Receive response from the sensor.
 *
 * @param dev Sensor instance.
 * @param[out] result Pointer where to store the result.
 * @param[out] buf Buffer to store the response.
 * @param buf_len Buffer length.
 *
 * @retval 0 Success.
 * @retval -EMSGSIZE If RX data length does not match current settings.
 * @retval -EAGAIN If reception times out.
 */
static int jm101_recv(const struct device *dev, uint8_t *result,
		      uint8_t *rx_data, uint8_t rx_data_len)
{
	struct jm101_data *data = dev->data;
	uint8_t buf[PKT_BUF_SIZE];
	int ret;

	if (rx_data_len != data->rx_data_len) {
		return -EMSGSIZE;
	}

	ret = k_msgq_get(&data->rx_queue, buf, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	*result = buf[PKT_CONFIRM_POS];
	memcpy(rx_data, &buf[PKT_DATA_POS], rx_data_len);

	return 0;
}

/**
 * @brief Run the auto identification sequence.
 *
 * @param dev Sensor instance.
 *
 * @retval 0 Success.
 * @retval -EIO If auto identification fails.
 * @retval -errno Other negative error numbers from sub-functions.
 */
static int jm101_auto_identify(const struct device *dev)
{
	struct jm101_data *data = dev->data;
	uint8_t buf[AUTOID_DATA_LEN] = {0U};
	int ret;
	uint8_t result;
	bool finished = false;

	/* trigger auto identification */
	sys_put_be16(data->verify_id, &buf[AUTOID_POS_ID]);
	(void)jm101_send(dev, CMD_AUTOID, buf, AUTOID_DATA_LEN,
			 AUTOID_DATA_LEN);

	/* wait for auto identification to finish */
	while (!finished) {
		ret = jm101_recv(dev, &result, buf, AUTOID_DATA_LEN);
		if (ret < 0) {
			return ret;
		}

		if ((result == RESULT_OK) &&
		    (buf[AUTOID_POS_STEP] == AUTOID_STEP_RESULTS)) {
			data->val.val1 = sys_get_be16(&buf[AUTOID_POS_ID]);
			data->val.val2 = sys_get_be16(&buf[AUTOID_POS_SCORE]);
			finished = true;
		} else if (result != RESULT_OK) {
			LOG_ERR("Auto-identify failed: %02X", result);
			return -EIO;
		}
	}

	return 0;
}

/**
 * @brief Run the fingerprint enrolling sequence.
 *
 * @param dev Sensor instance.
 * @param id Fingerprint ID.
 * @param readings Number of readings to take.
 *
 * @retval 0 Success.
 * @retval -EIO If enrolling fails.
 * @retval -errno Other negative error numbers from sub-functions.
 */
static int jm101_auto_enroll(const struct device *dev, uint16_t id,
			     uint8_t readings)
{
	uint8_t buf[AUTOENROLL_TX_DATA_LEN] = {0U};
	int ret;
	uint8_t result;
	bool finished = false;

	/* trigger auto enroll */
	sys_put_be16(id, &buf[AUTOENROLL_POS_ID]);
	buf[AUTOENROLL_POS_FREQ] = readings;
	(void)jm101_send(dev, CMD_AUTOENROLL, buf, AUTOENROLL_TX_DATA_LEN,
			 AUTOENROLL_RX_DATA_LEN);

	/* wait for auto enroll to finish */
	while (!finished) {
		ret = jm101_recv(dev, &result, buf, AUTOENROLL_RX_DATA_LEN);
		if (ret < 0) {
			return ret;
		}

		if ((result == RESULT_OK) &&
		    (buf[AUTOENROLL_POS_STEP] == AUTOENROLL_STEP_STORAGE)) {
			finished = true;
		} else if (result != RESULT_OK) {
			LOG_ERR("Auto-enroll failed: %02X", result);
			return -EIO;
		}
	}

	return 0;
}

/**
 * @brief Empty the fingerprint database.
 *
 * @param dev Sensor instance.
 *
 * @retval 0 Success.
 * @retval -EIO If auto identification fails.
 * @retval -errno Other negative error numbers from sub-functions.
 */
static int jm101_empty(const struct device *dev)
{
	int ret;
	uint8_t result;

	(void)jm101_send(dev, CMD_EMPTY, NULL, 0U, 0U);

	ret = jm101_recv(dev, &result, NULL, 0U);
	if (ret < 0) {
		return ret;
	}

	if (result != RESULT_OK) {
		LOG_ERR("Empty failed: %02X", result);
		return -EIO;
	}

	return 0;
}

/*******************************************************************************
 * API
 ******************************************************************************/

static int jm101_sample_fetch(const struct device *dev,
			      enum sensor_channel chan)
{
	if ((enum jm101_channel)chan != JM101_CHAN_FINGERPRINT) {
		return -ENOTSUP;
	}

	return jm101_auto_identify(dev);
}

static int jm101_channel_get(const struct device *dev, enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct jm101_data *data = dev->data;

	if ((enum jm101_channel)chan != JM101_CHAN_FINGERPRINT) {
		return -ENOTSUP;
	}

	*val = data->val;

	return 0;
}

static int jm101_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr,
			  const struct sensor_value *val)
{
	struct jm101_data *data = dev->data;

	if ((enum jm101_channel)chan != JM101_CHAN_FINGERPRINT) {
		return -ENOTSUP;
	}

	switch ((enum jm101_attribute)attr) {
	case JM101_ATTR_ID_NUM:
		data->verify_id = (uint16_t)val->val1;
		break;
	case JM101_ATTR_ENROLL:
		return jm101_auto_enroll(dev, val->val1, val->val2);
	case JM101_ATTR_EMPTYDB:
		return jm101_empty(dev);
	default:
		return -ENOTSUP;
	}

	return 0;
}

#ifdef CONFIG_JM101_TRIGGER
static int jm101_trigger_set(const struct device *dev,
			     const struct sensor_trigger *trig,
			     sensor_trigger_handler_t handler)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	gpio_flags_t flags;
	int ret;

	if ((trig->type != SENSOR_TRIG_TAP) ||
	    ((enum jm101_channel)trig->chan != JM101_CHAN_FINGERPRINT) ||
	    (config->touch.port == NULL)) {
		return -ENOTSUP;
	}

	if (handler != NULL) {
		flags = GPIO_INT_EDGE_TO_ACTIVE;
	} else {
		flags = GPIO_INT_DISABLE;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->touch, flags);
	if (ret < 0) {
		return ret;
	}

	data->trig_handler = handler;

	return 0;
}
#endif /* CONFIG_JM101_TRIGGER */

static const struct sensor_driver_api jm101_api = {
	.sample_fetch = &jm101_sample_fetch,
	.channel_get = &jm101_channel_get,
	.attr_set = &jm101_attr_set,
#ifdef CONFIG_JM101_TRIGGER
	.trigger_set = &jm101_trigger_set,
#endif /* CONFIG_JM101_TRIGGER */
};

static int jm101_init(const struct device *dev)
{
	const struct jm101_config *config = dev->config;
	struct jm101_data *data = dev->data;
	int ret;

	/* configure UART */
	if (!device_is_ready(config->uart)) {
		LOG_ERR("UART not ready");
		return -ENODEV;
	}

	ret = uart_configure(config->uart, &uart_config);
	if (ret < 0) {
		return ret;
	}

	uart_irq_callback_user_data_set(config->uart, uart_cb_handler,
					(void *)dev);

	k_msgq_init(&data->rx_queue, data->rx_queue_buf, PKT_BUF_SIZE,
		    RX_QUEUE_SIZE);

#ifdef CONFIG_JM101_TRIGGER
	/* configure touch GPIO */
	if (config->touch.port != NULL) {
		if (!device_is_ready(config->touch.port)) {
			LOG_ERR("Touch GPIO controller not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->touch, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Could not configure touch GPIO (%d)", ret);
			return ret;
		}

		gpio_init_callback(&data->touch_cb,
				   jm101_touch_callback_handler,
				   BIT(config->touch.pin));

		ret = gpio_add_callback(config->touch.port, &data->touch_cb);
		if (ret < 0) {
			LOG_ERR("Could not add touch GPIO callback (%d)", ret);
			return ret;
		}

		data->dev = dev;
		k_work_init(&data->trig_work, jm101_trig_work_handler);
	}
#endif /* CONFIG_JM101_TRIGGER */

	return 0;
}

#define JM101_DEFINE(i)                                                        \
	static struct jm101_data jm101_data_##i = {                            \
		.verify_id = JM101_ID_ANY,                                     \
	};                                                                     \
                                                                               \
	static const struct jm101_config jm101_config_##i = {                  \
		.uart = DEVICE_DT_GET(DT_INST_BUS(i)),                         \
		.addr = DT_INST_REG_ADDR(i),                                   \
		IF_ENABLED(CONFIG_JM101_TRIGGER,                               \
			   (.touch = GPIO_DT_SPEC_INST_GET_OR(i, touch_gpios,  \
							      {}), )) /* */    \
	};                                                                     \
                                                                               \
	DEVICE_DT_INST_DEFINE(i, jm101_init, NULL, &jm101_data_##i,            \
			      &jm101_config_##i, POST_KERNEL,                  \
			      CONFIG_SENSOR_INIT_PRIORITY, &jm101_api);

DT_INST_FOREACH_STATUS_OKAY(JM101_DEFINE)
