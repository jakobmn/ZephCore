/*
 * SPDX-License-Identifier: Apache-2.0
 * LR20xx HAL implementation for Zephyr - ZephCore
 *
 * Zephyr hardware context for the Semtech lr20xx_driver HAL interface.
 */

#ifndef LR20XX_HAL_ZEPHYR_H
#define LR20XX_HAL_ZEPHYR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "lr20xx_hal.h"

/**
 * @brief LR20xx HAL context for Zephyr
 *
 * Passed as the 'context' pointer to all lr20xx_hal_* functions.
 * Contains all hardware configuration needed to communicate with the radio.
 *
 * CRITICAL: All SPI operations must be protected by the driver's spi_mutex.
 * The LR2021 radio is accessed from two threads (main event loop + DIO1 work
 * queue).  Without the mutex, concurrent SPI access corrupts the command/response
 * protocol and the BUSY pin gets stuck HIGH permanently.
 */
struct lr20xx_hal_context {
	/* SPI device */
	const struct device *spi_dev;
	struct spi_config spi_cfg;

	/* GPIO pins */
	struct gpio_dt_spec nss;    /* Chip select (direct GPIO, not SPI peripheral CS) */
	struct gpio_dt_spec reset;  /* Reset pin (active-low) */
	struct gpio_dt_spec busy;   /* Busy pin (high = busy) */
	struct gpio_dt_spec dio1;   /* DIO1 interrupt pin */

	/* State tracking */
	volatile bool radio_is_sleeping;
};

/**
 * @brief Initialize HAL context GPIOs
 *
 * Must be called before any other HAL functions.
 *
 * @param ctx HAL context with gpio specs already filled in
 * @return 0 on success, negative errno on failure
 */
int lr20xx_hal_init(struct lr20xx_hal_context *ctx);

/**
 * @brief GPIO callback type for DIO1 interrupt
 */
typedef void (*lr20xx_dio1_callback_t)(void *user_data);

/**
 * @brief Set DIO1 interrupt callback
 *
 * @param ctx HAL context
 * @param cb  Callback (called directly from GPIO ISR — must be ISR-safe)
 * @param user_data User data passed to callback
 */
void lr20xx_hal_set_dio1_callback(struct lr20xx_hal_context *ctx,
				   lr20xx_dio1_callback_t cb, void *user_data);

/** @brief Enable DIO1 edge interrupt */
void lr20xx_hal_enable_dio1_irq(struct lr20xx_hal_context *ctx);

/** @brief Disable DIO1 interrupt */
void lr20xx_hal_disable_dio1_irq(struct lr20xx_hal_context *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LR20XX_HAL_ZEPHYR_H */
