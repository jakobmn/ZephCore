/*
 * SPDX-License-Identifier: Apache-2.0
 * LR20xx Zephyr LoRa driver
 *
 * Implements the standard Zephyr lora_driver_api using the Semtech lr20xx_driver
 * SDK. All SPI access, DIO1 IRQ handling, and radio state management is internal.
 */

#define DT_DRV_COMPAT semtech_lr2021

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include "lr20xx_lora.h"
#include "lr20xx_hal_zephyr.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_common_types.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_lora_types.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_system.h"
#include "lr20xx_system_types.h"

LOG_MODULE_REGISTER(lr20xx_lora, CONFIG_LORA_LOG_LEVEL);

/* Dedicated DIO1 work queue — keeps LoRa interrupt processing off the
 * system work queue so USB/BLE/timer work items cannot delay packet RX. */
#define LR20XX_DIO1_WQ_STACK_SIZE 2560
K_THREAD_STACK_DEFINE(lr20xx_dio1_wq_stack, LR20XX_DIO1_WQ_STACK_SIZE);

/* ── Driver data structures ─────────────────────────────────────────── */

struct lr20xx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec dio1;
	uint16_t tcxo_voltage_mv;
	uint32_t tcxo_startup_delay_ms;
	bool rx_boosted;
	/* RF switch DIO bitmasks (bit 0 = DIO5, bit 1 = DIO6, ...) */
	uint8_t rfswitch_enable;
	uint8_t rfswitch_standby;
	uint8_t rfswitch_rx;
	uint8_t rfswitch_tx;
	uint8_t rfswitch_tx_hp;
	/* PA config */
	uint8_t pa_hp_sel;    /* maps to pa_lf_slices in LR20xx */
	uint8_t pa_duty_cycle; /* maps to pa_lf_duty_cycle in LR20xx */
};

struct lr20xx_data {
	const struct device *dev;
	struct lr20xx_hal_context hal_ctx;
	struct k_mutex spi_mutex;

	/* Cached modem config from lora_config() */
	struct lora_modem_config modem_cfg;
	bool configured;

	/* Async RX state */
	lora_recv_cb async_rx_cb;
	void *async_rx_user_data;

	/* Async TX state */
	struct k_poll_signal *tx_signal;

	/* DIO1 work — runs on dedicated queue, not system work queue */
	struct k_work dio1_work;
	struct k_work_q dio1_wq;

	/* Radio state */
	volatile bool tx_active;
	volatile bool in_rx_mode;

	/* Extension features */
	bool rx_duty_cycle_enabled;
	bool rx_boost_enabled;
	bool rx_boost_applied;

	/* Deferred hardware init */
	bool hw_initialized;

	/* DIO1 stuck-HIGH detection */
	int dio1_stuck_count;

	/* RX data buffer */
	uint8_t rx_buf[256];
};

/* ── Helpers ────────────────────────────────────────────────────────── */

static lr20xx_radio_lora_bw_t bw_enum_to_lr20xx(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_31_KHZ:  return LR20XX_RADIO_LORA_BW_31;
	case BW_41_KHZ:  return LR20XX_RADIO_LORA_BW_41;
	case BW_62_KHZ:  return LR20XX_RADIO_LORA_BW_62;
	case BW_125_KHZ: return LR20XX_RADIO_LORA_BW_125;
	case BW_250_KHZ: return LR20XX_RADIO_LORA_BW_250;
	case BW_500_KHZ: return LR20XX_RADIO_LORA_BW_500;
	default:         return LR20XX_RADIO_LORA_BW_125;
	}
}

static lr20xx_radio_lora_cr_t cr_enum_to_lr20xx(enum lora_coding_rate cr)
{
	switch (cr) {
	case CR_4_5: return LR20XX_RADIO_LORA_CR_4_5;
	case CR_4_6: return LR20XX_RADIO_LORA_CR_4_6;
	case CR_4_7: return LR20XX_RADIO_LORA_CR_4_7;
	case CR_4_8: return LR20XX_RADIO_LORA_CR_4_8;
	default:     return LR20XX_RADIO_LORA_CR_4_8;
	}
}

static lr20xx_system_tcxo_supply_voltage_t get_tcxo_voltage(uint16_t mv)
{
	if (mv >= 3300) return LR20XX_SYSTEM_TCXO_CTRL_3_3V;
	if (mv >= 3000) return LR20XX_SYSTEM_TCXO_CTRL_3_0V;
	if (mv >= 2700) return LR20XX_SYSTEM_TCXO_CTRL_2_7V;
	if (mv >= 2400) return LR20XX_SYSTEM_TCXO_CTRL_2_4V;
	if (mv >= 2200) return LR20XX_SYSTEM_TCXO_CTRL_2_2V;
	if (mv >= 1800) return LR20XX_SYSTEM_TCXO_CTRL_1_8V;
	return LR20XX_SYSTEM_TCXO_CTRL_1_6V;
}

/* Get kHz value from Zephyr BW enum — used for LDRO/PPM calculation */
static float bw_enum_to_khz(enum lora_signal_bandwidth bw)
{
	switch (bw) {
	case BW_7_KHZ:   return 7.81f;
	case BW_10_KHZ:  return 10.42f;
	case BW_15_KHZ:  return 15.63f;
	case BW_20_KHZ:  return 20.83f;
	case BW_31_KHZ:  return 31.25f;
	case BW_41_KHZ:  return 41.67f;
	case BW_62_KHZ:  return 62.5f;
	case BW_125_KHZ: return 125.0f;
	case BW_250_KHZ: return 250.0f;
	case BW_500_KHZ: return 500.0f;
	default:         return 125.0f;
	}
}

/* ── Configure RF switch DIOs ───────────────────────────────────────── */

static void lr20xx_configure_rfswitch(void *ctx, const struct lr20xx_config *cfg)
{
	/* LR20xx RF switch uses per-DIO configuration.
	 * DIO5..DIO8 map to enable bitmask bits 0..3.
	 * For each enabled DIO, compute which operational modes
	 * should drive it HIGH by looking at the per-mode bitmasks. */
	for (int i = 0; i < 4; i++) {
		if (!(cfg->rfswitch_enable & BIT(i))) {
			continue;
		}

		lr20xx_system_dio_t dio = (lr20xx_system_dio_t)(LR20XX_SYSTEM_DIO_5 + i);

		/* Set this DIO function to RF switch control */
		lr20xx_system_set_dio_function(ctx, dio,
					       LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH);

		/* Build the per-DIO mode bitmask:
		 * which operational modes drive this DIO HIGH */
		lr20xx_system_dio_rf_switch_cfg_t sw_cfg = 0;

		if (cfg->rfswitch_standby & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_STANDBY;
		}
		if (cfg->rfswitch_rx & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_LF |
				  LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF;
		}
		if (cfg->rfswitch_tx & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF;
		}
		if (cfg->rfswitch_tx_hp & BIT(i)) {
			sw_cfg |= LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF;
		}

		lr20xx_system_set_dio_rf_switch_cfg(ctx, dio, sw_cfg);
	}
}

/* ── Hardware reset (BUSY stuck recovery) ───────────────────────────── */

static void lr20xx_hardware_reset(struct lr20xx_data *data,
				  const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_WRN("LR2021 hardware reset (BUSY stuck recovery)");

	lr20xx_hal_reset(ctx);

	if (cfg->tcxo_voltage_mv > 0) {
		lr20xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    (cfg->tcxo_startup_delay_ms * 1000) / 31);
	}

	lr20xx_system_set_reg_mode(ctx, LR20XX_SYSTEM_REG_MODE_DCDC);

	lr20xx_configure_rfswitch(ctx, cfg);

	lr20xx_system_calibrate(ctx, 0x6F);

	lr20xx_radio_common_set_rx_tx_fallback_mode(ctx,
						    LR20XX_RADIO_FALLBACK_STDBY_RC);

	lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);

	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	data->rx_boost_applied = false;

	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	LOG_WRN("LR2021 recovered from hardware reset");
}

/* ── Apply modem configuration ──────────────────────────────────────── */

static void lr20xx_apply_modem_config(struct lr20xx_data *data,
				      const struct lr20xx_config *cfg,
				      bool tx_mode)
{
	void *ctx = &data->hal_ctx;
	struct lora_modem_config *mc = &data->modem_cfg;

	lr20xx_radio_common_set_rf_freq(ctx, mc->frequency);

	/* LR20xx uses PPM offset instead of explicit LDRO.
	 * PPM_1_4 (1 bin every 4) is equivalent to LDRO for high-SF
	 * wide-time-on-air configurations. Use recommended value. */
	lr20xx_radio_lora_mod_params_t mod = {
		.sf  = (lr20xx_radio_lora_sf_t)mc->datarate,
		.bw  = bw_enum_to_lr20xx(mc->bandwidth),
		.cr  = cr_enum_to_lr20xx(mc->coding_rate),
		.ppm = lr20xx_radio_lora_get_recommended_ppm_offset(
			(lr20xx_radio_lora_sf_t)mc->datarate,
			bw_enum_to_lr20xx(mc->bandwidth)),
	};
	lr20xx_radio_lora_set_modulation_params(ctx, &mod);

	lr20xx_radio_lora_pkt_params_t pkt = {
		.preamble_len_in_symb = mc->preamble_len,
		.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = 255,
		.crc = mc->packet_crc_disable ? LR20XX_RADIO_LORA_CRC_DISABLED
					      : LR20XX_RADIO_LORA_CRC_ENABLED,
		.iq = mc->iq_inverted ? LR20XX_RADIO_LORA_IQ_INVERTED
				      : LR20XX_RADIO_LORA_IQ_STANDARD,
	};
	lr20xx_radio_lora_set_packet_params(ctx, &pkt);

	lr20xx_radio_lora_set_syncword(ctx,
				       mc->public_network ? 0x34 : 0x12);

	if (tx_mode) {
		/* LR20xx set_tx_params uses half-dBm (multiply by 2) */
		lr20xx_radio_common_set_tx_params(ctx,
						  (int8_t)(mc->tx_power * 2),
						  LR20XX_RADIO_COMMON_RAMP_48_US);

		lr20xx_radio_common_select_pa(ctx, LR20XX_RADIO_COMMON_PA_SEL_LF);

		lr20xx_radio_common_pa_cfg_t pa = {
			.pa_sel          = LR20XX_RADIO_COMMON_PA_SEL_LF,
			.pa_lf_mode      = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
			.pa_lf_duty_cycle = cfg->pa_duty_cycle,
			.pa_lf_slices    = cfg->pa_hp_sel,
			.pa_hf_duty_cycle = 16,  /* unused for LF, default */
		};
		lr20xx_radio_common_set_pa_cfg(ctx, &pa);
	}

	/* Route IRQ events to DIO5 (physical DIO1 pin on the board) */
	lr20xx_system_set_dio_irq_cfg(ctx, LR20XX_SYSTEM_DIO_5,
		LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TX_DONE |
		LR20XX_SYSTEM_IRQ_TIMEOUT | LR20XX_SYSTEM_IRQ_CRC_ERROR |
		LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR);
}

/* ── Start RX (internal) ────────────────────────────────────────────── */

static void lr20xx_start_rx(struct lr20xx_data *data,
			     const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_DBG("start_rx: t=%lld", k_uptime_get());

	/* Standby first — wake from any sleep state */
	data->hal_ctx.radio_is_sleeping = true;
	lr20xx_status_t rc = lr20xx_system_set_standby_mode(ctx,
							    LR20XX_SYSTEM_STANDBY_MODE_RC);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("standby failed (rc=%d) — triggering HW reset", rc);
		lr20xx_hardware_reset(data, cfg);
	}

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	lr20xx_apply_modem_config(data, cfg, false);

	/* Apply RX boost if needed */
	if (data->rx_boost_enabled && !data->rx_boost_applied) {
		lr20xx_radio_common_set_rx_path(
			ctx, LR20XX_RADIO_COMMON_RX_PATH_LF,
			LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4);
		data->rx_boost_applied = true;
	}

	/* Start continuous RX using RTC-step API (0xFFFFFF = continuous) */
	lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);

	/* Clear any IRQ flags set during modem configuration */
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	data->in_rx_mode = true;
	data->tx_active = false;
}

/* ── Lightweight RX restart (no modem reconfig) ─────────────────────── */

static void lr20xx_restart_rx(struct lr20xx_data *data)
{
	void *ctx = &data->hal_ctx;

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);

	data->in_rx_mode = true;
}

/* ── DIO1 IRQ handler (work queue, thread context) ──────────────────── */

static void lr20xx_dio1_callback(void *user_data);

static void lr20xx_dio1_work_handler(struct k_work *work)
{
	struct lr20xx_data *data = CONTAINER_OF(work, struct lr20xx_data,
						dio1_work);
	const struct lr20xx_config *cfg = data->dev->config;
	void *ctx = &data->hal_ctx;
	bool rx_restarted = false;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Combined get + clear IRQ status */
	lr20xx_system_irq_mask_t irq = 0;
	lr20xx_status_t rc = lr20xx_system_get_and_clear_irq_status(ctx, &irq);

	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("Failed to read IRQ status (rc=%d)", rc);
		goto safety_check;
	}

	LOG_DBG("DIO1 IRQ: 0x%08x tx=%d t=%lld", irq, data->tx_active,
		k_uptime_get());

	if (irq & LR20XX_SYSTEM_IRQ_ERROR) {
		LOG_WRN("IRQ hardware ERROR: 0x%08x", irq);
	}

	if (irq != 0) {
		data->dio1_stuck_count = 0;
	}

	/* ── RX done ── */
	if (irq & LR20XX_SYSTEM_IRQ_RX_DONE) {
		uint16_t pkt_len = 0;
		lr20xx_radio_common_get_rx_packet_length(ctx, &pkt_len);

		if (pkt_len > 0 && pkt_len <= 255) {
			lr20xx_radio_lora_packet_status_t pkt_stat;
			lr20xx_radio_lora_get_packet_status(ctx, &pkt_stat);

			lr20xx_radio_fifo_read_rx(ctx, data->rx_buf,
						  (uint16_t)pkt_len);

			/* Restart RX before firing callback */
			lr20xx_restart_rx(data);
			rx_restarted = true;

			/* When SNR < 0, use signal RSSI for a more
			 * accurate reading on weak links. */
			int16_t rssi = pkt_stat.rssi_pkt_in_dbm;
			int8_t snr = pkt_stat.snr_pkt_raw / 4;

			if (snr < 0 &&
			    pkt_stat.rssi_signal_pkt_in_dbm > rssi) {
				rssi = pkt_stat.rssi_signal_pkt_in_dbm;
			}

			k_mutex_unlock(&data->spi_mutex);

			if (data->async_rx_cb) {
				data->async_rx_cb(data->dev, data->rx_buf,
						  (uint8_t)pkt_len,
						  rssi, snr,
						  data->async_rx_user_data);
			}
			return;
		}

		LOG_WRN("RX: invalid len %d", pkt_len);
		lr20xx_restart_rx(data);
		rx_restarted = true;
	}

	/* ── TX done ── */
	if (irq & LR20XX_SYSTEM_IRQ_TX_DONE) {
		LOG_DBG("TX done");
		data->tx_active = false;

		lr20xx_start_rx(data, cfg);
		rx_restarted = true;

		if (data->tx_signal) {
			k_poll_signal_raise(data->tx_signal, 0);
		}
	}

	/* ── Timeout ── */
	if (irq & LR20XX_SYSTEM_IRQ_TIMEOUT) {
		LOG_DBG("Timeout IRQ — restarting RX");
		if (!data->tx_active) {
			lr20xx_restart_rx(data);
			rx_restarted = true;
		}
	}

	/* ── CRC / Header error ── */
	if (irq & LR20XX_SYSTEM_IRQ_CRC_ERROR ||
	    ((irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) &&
	     !(irq & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID))) {
		LOG_WRN("RX error: CRC=%d HDR=%d",
			(irq & LR20XX_SYSTEM_IRQ_CRC_ERROR) ? 1 : 0,
			(irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) ? 1 : 0);

		if (!data->tx_active) {
			lr20xx_restart_rx(data);
			rx_restarted = true;
		}

		k_mutex_unlock(&data->spi_mutex);

		if (data->async_rx_cb) {
			data->async_rx_cb(data->dev, NULL, 0, 0, 0,
					  data->async_rx_user_data);
		}
		return;
	}

safety_check:
	if (!rx_restarted && data->in_rx_mode && !data->tx_active) {
		LOG_WRN("DIO1 safety: no IRQ handled (0x%08x rc=%d), "
			"restarting RX", irq, rc);
		lr20xx_restart_rx(data);
	}

	/* Edge-triggered DIO1: if still HIGH, re-submit for pending flags.
	 * Guard against stuck DIO1: after 5 empty cycles, hardware reset. */
	if (gpio_pin_get_dt(&data->hal_ctx.dio1)) {
		data->dio1_stuck_count++;
		if (data->dio1_stuck_count >= 5) {
			LOG_ERR("DIO1 stuck HIGH for %d cycles — "
				"hardware reset", data->dio1_stuck_count);
			data->dio1_stuck_count = 0;
			lr20xx_hardware_reset(data, cfg);
			lr20xx_start_rx(data, cfg);
		} else {
			k_work_submit_to_queue(&data->dio1_wq,
					       &data->dio1_work);
		}
	} else {
		data->dio1_stuck_count = 0;
	}

	k_mutex_unlock(&data->spi_mutex);
}

static void lr20xx_dio1_callback(void *user_data)
{
	struct lr20xx_data *data = (struct lr20xx_data *)user_data;
	k_work_submit_to_queue(&data->dio1_wq, &data->dio1_work);
}

/* Forward declaration */
static int lr20xx_hw_init(struct lr20xx_data *data,
			  const struct lr20xx_config *cfg);

/* ── Driver API: config ─────────────────────────────────────────────── */

static int lr20xx_lora_config(const struct device *dev,
			      struct lora_modem_config *config)
{
	struct lr20xx_data *data = dev->data;

	if (!data->hw_initialized) {
		int ret = lr20xx_hw_init(data, dev->config);
		if (ret != 0) {
			LOG_ERR("Hardware init failed: %d", ret);
			return ret;
		}
	}

	memcpy(&data->modem_cfg, config, sizeof(*config));
	data->configured = true;

	/* Image calibration at operating frequency */
	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr20xx_radio_common_front_end_calibration_value_t cal = {
		.rx_path          = LR20XX_RADIO_COMMON_RX_PATH_LF,
		.frequency_in_hertz = config->frequency,
	};
	lr20xx_radio_common_calibrate_front_end_helper(&data->hal_ctx,
						       &cal, 1);
	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("config: %uHz SF%d BW%d CR%d pwr=%d tx=%d",
		config->frequency, config->datarate, config->bandwidth,
		config->coding_rate, config->tx_power, config->tx);

	return 0;
}

/* ── Driver API: airtime ────────────────────────────────────────────── */

static uint32_t lr20xx_lora_airtime(const struct device *dev,
				    uint32_t data_len)
{
	struct lr20xx_data *data = dev->data;
	struct lora_modem_config *mc = &data->modem_cfg;

	uint8_t sf = (uint8_t)mc->datarate;
	float bw = bw_enum_to_khz(mc->bandwidth) * 1000.0f;
	uint8_t cr = (uint8_t)mc->coding_rate + 4;

	float ts = (float)(1 << sf) / bw;
	int de = (sf >= 11 && bw <= 125000.0f) ? 1 : 0;
	float n_payload = 8.0f + fmaxf(
		ceilf((8.0f * data_len - 4.0f * sf + 28.0f + 16.0f) /
		      (4.0f * (sf - 2.0f * de))) * cr,
		0.0f);
	float t_preamble = (mc->preamble_len + 4.25f) * ts;
	float t_payload = n_payload * ts;

	return (uint32_t)((t_preamble + t_payload) * 1000.0f);
}

/* ── Driver API: send_async ─────────────────────────────────────────── */

static int lr20xx_lora_send_async(const struct device *dev,
				  uint8_t *buf, uint32_t data_len,
				  struct k_poll_signal *async)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	void *ctx = &data->hal_ctx;

	if (!data->configured) return -EINVAL;
	if (data->tx_active) return -EBUSY;
	if (data_len > 255 || data_len == 0) return -EINVAL;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = NULL;
	data->in_rx_mode = false;

	lr20xx_hal_disable_dio1_irq(&data->hal_ctx);

	/* Standby — wake from sleep if needed */
	data->hal_ctx.radio_is_sleeping = true;
	lr20xx_status_t rc = lr20xx_system_set_standby_mode(ctx,
							    LR20XX_SYSTEM_STANDBY_MODE_RC);
	if (rc != LR20XX_STATUS_OK) {
		LOG_ERR("TX standby failed — HW reset");
		lr20xx_hardware_reset(data, cfg);
	}

	lr20xx_apply_modem_config(data, cfg, true);

	/* Set TX-specific packet length */
	lr20xx_radio_lora_pkt_params_t pkt = {
		.preamble_len_in_symb = data->modem_cfg.preamble_len,
		.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = (uint8_t)data_len,
		.crc = data->modem_cfg.packet_crc_disable
			? LR20XX_RADIO_LORA_CRC_DISABLED
			: LR20XX_RADIO_LORA_CRC_ENABLED,
		.iq = data->modem_cfg.iq_inverted
			? LR20XX_RADIO_LORA_IQ_INVERTED
			: LR20XX_RADIO_LORA_IQ_STANDARD,
	};
	lr20xx_radio_lora_set_packet_params(ctx, &pkt);

	/* Write to TX FIFO */
	lr20xx_radio_fifo_write_tx(ctx, buf, (uint16_t)data_len);

	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);
	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	data->tx_signal = async;
	data->tx_active = true;
	lr20xx_radio_common_set_tx(ctx, 5000);

	k_mutex_unlock(&data->spi_mutex);

	LOG_DBG("TX started: len=%u", data_len);
	return 0;
}

/* ── Driver API: send (sync) ────────────────────────────────────────── */

static int lr20xx_lora_send(const struct device *dev,
			    uint8_t *buf, uint32_t data_len)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);

	int ret = lr20xx_lora_send_async(dev, buf, data_len, &done);
	if (ret < 0) return ret;

	uint32_t air_time = lr20xx_lora_airtime(dev, data_len);
	ret = k_poll(&evt, 1, K_MSEC(2 * air_time + 1000));
	if (ret < 0) {
		LOG_ERR("TX sync timeout");
		return ret;
	}

	return 0;
}

/* ── Driver API: recv_async ─────────────────────────────────────────── */

static int lr20xx_lora_recv_async(const struct device *dev,
				  lora_recv_cb cb, void *user_data)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;

	if (cb == NULL) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		data->async_rx_cb = NULL;
		data->async_rx_user_data = NULL;
		data->in_rx_mode = false;
		k_mutex_unlock(&data->spi_mutex);
		return 0;
	}

	if (!data->configured) return -EINVAL;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	data->async_rx_cb = cb;
	data->async_rx_user_data = user_data;

	lr20xx_start_rx(data, cfg);

	k_mutex_unlock(&data->spi_mutex);

	LOG_INF("recv_async started (continuous RX%s)",
		data->rx_boost_enabled ? ", boosted" : "");

	return 0;
}

/* ── Driver API: recv (sync) ────────────────────────────────────────── */

static int lr20xx_lora_recv(const struct device *dev, uint8_t *buf,
			    uint8_t size, k_timeout_t timeout,
			    int16_t *rssi, int8_t *snr)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(buf);
	ARG_UNUSED(size);
	ARG_UNUSED(timeout);
	ARG_UNUSED(rssi);
	ARG_UNUSED(snr);
	return -ENOTSUP;
}

/* ── LR20xx extension API ───────────────────────────────────────────── */

int16_t lr20xx_get_rssi_inst(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	int16_t rssi = 0;
	uint8_t half_dbm = 0;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr20xx_radio_common_get_rssi_inst(&data->hal_ctx, &rssi, &half_dbm);
	k_mutex_unlock(&data->spi_mutex);

	return rssi;
}

bool lr20xx_is_receiving(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;

	/* LR20xx has no non-destructive IRQ read (only get_and_clear).
	 * Consuming bits here races with the DIO1 work handler — RX_DONE
	 * could be cleared before the handler delivers the packet.
	 * Use radio state flags as proxy: radio is "receiving" if in
	 * continuous RX, not in TX, and DIO1 is not asserted (asserted
	 * means a completion event is pending, not a mid-preamble state). */
	if (!data->in_rx_mode || data->tx_active) {
		return false;
	}

	return !gpio_pin_get_dt(&data->hal_ctx.dio1);
}

void lr20xx_set_rx_duty_cycle(const struct device *dev, bool enable)
{
	struct lr20xx_data *data = dev->data;

	data->rx_duty_cycle_enabled = enable;
	LOG_INF("RX duty cycle %s", enable ? "enabled" : "disabled");
}

void lr20xx_set_rx_boost(const struct device *dev, bool enable)
{
	struct lr20xx_data *data = dev->data;

	if (data->rx_boost_enabled == enable) {
		return;
	}

	data->rx_boost_enabled = enable;
	LOG_INF("RX boost %s", enable ? "enabled" : "disabled");

	if (data->in_rx_mode && data->configured) {
		k_mutex_lock(&data->spi_mutex, K_FOREVER);
		lr20xx_radio_common_set_rx_path(
			&data->hal_ctx, LR20XX_RADIO_COMMON_RX_PATH_LF,
			enable ? LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4
			       : LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE);
		data->rx_boost_applied = enable;
		k_mutex_unlock(&data->spi_mutex);
	} else {
		data->rx_boost_applied = false;
	}
}

uint32_t lr20xx_get_random(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	uint32_t random = 0;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);
	lr20xx_system_get_random_number(
		&data->hal_ctx,
		LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_PLL |
		LR20XX_SYSTEM_RANDOM_ENTROPY_SOURCE_ADC,
		&random);
	k_mutex_unlock(&data->spi_mutex);

	return random;
}

void lr20xx_reset_agc(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	void *ctx = &data->hal_ctx;

	k_mutex_lock(&data->spi_mutex, K_FOREVER);

	/* Warm sleep — powers down analog frontend (resets AGC gain state).
	 * is_ram_retention_enabled=true = warm sleep (equivalent to LR11xx warm_start). */
	lr20xx_system_sleep_cfg_t sleep_cfg = {
		.is_clk_32k_enabled       = false,
		.is_ram_retention_enabled = true,
	};
	lr20xx_system_set_sleep_mode(ctx, &sleep_cfg, 0);
	k_sleep(K_USEC(500));

	lr20xx_system_set_standby_mode(ctx, LR20XX_SYSTEM_STANDBY_MODE_RC);

	lr20xx_system_calibrate(ctx, 0x6F);

	if (data->configured) {
		lr20xx_radio_common_front_end_calibration_value_t cal = {
			.rx_path          = LR20XX_RADIO_COMMON_RX_PATH_LF,
			.frequency_in_hertz = data->modem_cfg.frequency,
		};
		lr20xx_radio_common_calibrate_front_end_helper(ctx, &cal, 1);
	}

	if (data->rx_boost_enabled) {
		lr20xx_radio_common_set_rx_path(
			ctx, LR20XX_RADIO_COMMON_RX_PATH_LF,
			LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4);
		data->rx_boost_applied = true;
	}

	k_mutex_unlock(&data->spi_mutex);
}

/* ── Deferred hardware init ─────────────────────────────────────────── */

static int lr20xx_hw_init(struct lr20xx_data *data,
			  const struct lr20xx_config *cfg)
{
	void *ctx = &data->hal_ctx;

	LOG_INF("LR20xx hardware init starting");

	lr20xx_system_version_t ver;
	bool found = false;

	for (int attempt = 0; attempt < 3; attempt++) {
		lr20xx_hal_status_t hal_rc = lr20xx_hal_reset(ctx);
		if (hal_rc != LR20XX_HAL_STATUS_OK) {
			LOG_WRN("LR20xx reset failed (attempt %d)", attempt);
			k_msleep(10);
			continue;
		}

		lr20xx_status_t st = lr20xx_system_get_version(ctx, &ver);
		if (st == LR20XX_STATUS_OK) {
			found = true;
			break;
		}

		LOG_WRN("LR20xx get_version failed (attempt %d)", attempt);
		k_msleep(10);
	}

	if (!found) {
		LOG_ERR("LR20xx not found after 3 attempts");
		return -EIO;
	}

	LOG_INF("LR20xx HW:0x%02X FW:0x%04X", ver.hw, ver.fw);

	if (cfg->tcxo_voltage_mv > 0) {
		/* Convert ms to RTC steps (31.25 us per step) */
		uint32_t rtc_steps = (cfg->tcxo_startup_delay_ms * 1000) / 31;
		lr20xx_system_set_tcxo_mode(ctx,
					    get_tcxo_voltage(cfg->tcxo_voltage_mv),
					    rtc_steps);
		LOG_DBG("TCXO: %dmV", cfg->tcxo_voltage_mv);
	}

	lr20xx_system_set_reg_mode(ctx, LR20XX_SYSTEM_REG_MODE_DCDC);

	lr20xx_configure_rfswitch(ctx, cfg);

	LOG_INF("RF switch: en=0x%02x rx=0x%02x tx=0x%02x txhp=0x%02x",
		cfg->rfswitch_enable, cfg->rfswitch_rx,
		cfg->rfswitch_tx, cfg->rfswitch_tx_hp);

	/* Calibrate all 7 blocks (LF_RC=1, HF_RC=2, PLL=4, AAF=8, MU=32, PA_OFF=64) */
	lr20xx_system_calibrate(ctx, 0x6F);
	LOG_INF("Calibration OK");

	lr20xx_radio_common_set_rx_tx_fallback_mode(ctx,
						    LR20XX_RADIO_FALLBACK_STDBY_RC);

	lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);

	lr20xx_system_errors_t sys_errors = 0;
	lr20xx_system_get_errors(ctx, &sys_errors);
	if (sys_errors) {
		LOG_WRN("System errors at init: 0x%04x — clearing", sys_errors);
	}
	lr20xx_system_clear_errors(ctx);
	lr20xx_system_clear_irq_status(ctx, LR20XX_SYSTEM_IRQ_ALL_MASK);

	lr20xx_hal_enable_dio1_irq(&data->hal_ctx);

	data->rx_boost_enabled = cfg->rx_boosted;
	data->rx_boost_applied = false;

	data->hw_initialized = true;
	LOG_INF("LR20xx driver ready");
	return 0;
}

/* ── Driver init (lightweight — runs at POST_KERNEL) ────────────────── */

static int lr20xx_lora_init(const struct device *dev)
{
	struct lr20xx_data *data = dev->data;
	const struct lr20xx_config *cfg = dev->config;
	int ret;

	data->dev = dev;
	data->hw_initialized = false;

	k_mutex_init(&data->spi_mutex);
	k_work_init(&data->dio1_work, lr20xx_dio1_work_handler);

	k_work_queue_start(&data->dio1_wq, lr20xx_dio1_wq_stack,
			   K_THREAD_STACK_SIZEOF(lr20xx_dio1_wq_stack),
			   K_PRIO_COOP(7), NULL);
	k_thread_name_set(&data->dio1_wq.thread, "lr20xx_dio1");

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	memset(&data->hal_ctx, 0, sizeof(data->hal_ctx));
	data->hal_ctx.spi_dev = cfg->bus.bus;
	data->hal_ctx.spi_cfg = cfg->bus.config;
	/* Manual NSS control — disable SPI peripheral CS */
	data->hal_ctx.spi_cfg.cs.cs_is_gpio = false;
	data->hal_ctx.spi_cfg.cs.gpio.port = NULL;
	data->hal_ctx.nss.port  = cfg->bus.config.cs.gpio.port;
	data->hal_ctx.nss.pin   = cfg->bus.config.cs.gpio.pin;
	data->hal_ctx.nss.dt_flags = cfg->bus.config.cs.gpio.dt_flags;
	data->hal_ctx.reset = cfg->reset;
	data->hal_ctx.busy  = cfg->busy;
	data->hal_ctx.dio1  = cfg->dio1;
	data->hal_ctx.radio_is_sleeping = false;

	ret = lr20xx_hal_init(&data->hal_ctx);
	if (ret != 0) {
		LOG_ERR("HAL init failed: %d", ret);
		return ret;
	}

	lr20xx_hal_set_dio1_callback(&data->hal_ctx, lr20xx_dio1_callback,
				     data);

	LOG_INF("LR20xx driver registered (hw init deferred to first config)");
	return 0;
}

/* ── Device instantiation ───────────────────────────────────────────── */

static DEVICE_API(lora, lr20xx_lora_api) = {
	.config     = lr20xx_lora_config,
	.airtime    = lr20xx_lora_airtime,
	.send       = lr20xx_lora_send,
	.send_async = lr20xx_lora_send_async,
	.recv       = lr20xx_lora_recv,
	.recv_async = lr20xx_lora_recv_async,
};

#define LR20XX_INIT(n)                                                       \
	static const struct lr20xx_config lr20xx_config_##n = {              \
		.bus = SPI_DT_SPEC_INST_GET(n,                               \
			SPI_WORD_SET(8) | SPI_OP_MODE_MASTER |               \
			SPI_TRANSFER_MSB),                                   \
		.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),              \
		.busy  = GPIO_DT_SPEC_INST_GET(n, busy_gpios),              \
		.dio1  = GPIO_DT_SPEC_INST_GET(n, dio1_gpios),              \
		.tcxo_voltage_mv =                                           \
			DT_INST_PROP_OR(n, tcxo_voltage_mv, 0),             \
		.tcxo_startup_delay_ms =                                     \
			DT_INST_PROP_OR(n, tcxo_startup_delay_ms, 5),       \
		.rx_boosted       = DT_INST_PROP(n, rx_boosted),            \
		.rfswitch_enable  = DT_INST_PROP_OR(n, rfswitch_enable, 0), \
		.rfswitch_standby = DT_INST_PROP_OR(n, rfswitch_standby, 0),\
		.rfswitch_rx      = DT_INST_PROP_OR(n, rfswitch_rx, 0),     \
		.rfswitch_tx      = DT_INST_PROP_OR(n, rfswitch_tx, 0),     \
		.rfswitch_tx_hp   = DT_INST_PROP_OR(n, rfswitch_tx_hp, 0),  \
		.pa_hp_sel        = DT_INST_PROP_OR(n, pa_hp_sel, 7),       \
		.pa_duty_cycle    = DT_INST_PROP_OR(n, pa_duty_cycle, 4),   \
	};                                                                   \
	static struct lr20xx_data lr20xx_data_##n;                           \
	DEVICE_DT_INST_DEFINE(n, lr20xx_lora_init, NULL,                     \
			      &lr20xx_data_##n, &lr20xx_config_##n,          \
			      POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,        \
			      &lr20xx_lora_api);

DT_INST_FOREACH_STATUS_OKAY(LR20XX_INIT)
