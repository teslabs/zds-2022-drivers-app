/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef APP_INCLUDE_APP_DRIVERS_LOCK_H_
#define APP_INCLUDE_APP_DRIVERS_LOCK_H_

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

/** @cond INTERNAL_HIDDEN */

typedef int (*lock_open_t)(const struct device *dev);
typedef int (*lock_close_t)(const struct device *dev);

__subsystem struct lock_driver_api {
	lock_open_t open;
	lock_close_t close;
};

/** @endcond */

/**
 * @brief Open the lock.
 *
 * @param dev Lock device instance.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int lock_open(const struct device *dev);

static inline int z_impl_lock_open(const struct device *dev)
{
	const struct lock_driver_api *api =
		(const struct lock_driver_api *)dev->api;

	return api->open(dev);
}

/**
 * @brief Close the lock.
 *
 * @param dev Lock device instance.
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
__syscall int lock_close(const struct device *dev);

static inline int z_impl_lock_close(const struct device *dev)
{
	const struct lock_driver_api *api =
		(const struct lock_driver_api *)dev->api;

	return api->close(dev);
}

#include <syscalls/lock.h>

#endif /* APP_INCLUDE_APP_DRIVERS_SENSOR_GC0328_H_ */
