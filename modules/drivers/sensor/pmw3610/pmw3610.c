/*
 * PMW3610 ultra-low-power optical motion sensor — Zephyr input driver
 *
 * Hardware SPI with 3-wire SDIO (MOSI and MISO wired to the same pin).
 * Reports raw relative X/Y deltas — acceleration and axis transforms
 * are handled by ZMK input processors in the listener pipeline.
 *
 * Init is async (non-blocking) using delayable work items so other
 * SPI peripherals are not blocked during boot.
 */

#define DT_DRV_COMPAT pixart_pmw3610_custom

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <zephyr/sys/util.h>

#include <zmk/events/activity_state_changed.h>

#include "pmw3610.h"

LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

/* ── Dedicated work queue for motion processing ────────────────────── */

K_THREAD_STACK_DEFINE(pmw3610_work_stack, 1024);
static struct k_work_q pmw3610_work_q;

/* ── Async init step definitions ───────────────────────────────────── */

enum pmw3610_init_step {
	ASYNC_INIT_STEP_POWER_UP,
	ASYNC_INIT_STEP_CLEAR_OB1,
	ASYNC_INIT_STEP_CHECK_OB1,
	ASYNC_INIT_STEP_CONFIGURE,
	ASYNC_INIT_STEP_COUNT
};

static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
	[ASYNC_INIT_STEP_POWER_UP]  = 60,
	[ASYNC_INIT_STEP_CLEAR_OB1] = 200,
	[ASYNC_INIT_STEP_CHECK_OB1] = 50,
	[ASYNC_INIT_STEP_CONFIGURE] = 0,
};

/* ── Per-instance data ─────────────────────────────────────────────── */

struct pmw3610_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec motion_gpio;
	uint16_t cpi;
	bool smart_mode;
};

struct pmw3610_data {
	const struct device *dev;
	struct gpio_callback motion_cb;
	struct k_work trigger_work;
	struct k_work_delayable init_work;
	int async_init_step;
	bool ready;
	int err;
	uint16_t cpi;
	int32_t dx_acc;
	int32_t dy_acc;
};

/* ── SPI register access ──────────────────────────────────────────── *
 *
 * Single spi_transceive_dt calls keep CS asserted for the entire
 * transaction.  The PMW3610 needs T_SRAD (4 us) between address and
 * data, but at <= 2 MHz the SPIM inter-byte gap + DMA turnaround
 * provides sufficient delay (same approach as little-wing).
 */

static int pmw3610_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct pmw3610_config *cfg = dev->config;
	uint8_t addr = reg & 0x7F;

	const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 },   /* skip address byte */
		{ .buf = val, .len = 1 },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct pmw3610_config *cfg = dev->config;
	uint8_t buf[] = { reg | SPI_WRITE_BIT, val };

	const struct spi_buf tx_buf = { .buf = buf, .len = sizeof(buf) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_busy_wait(T_CLOCK_ON_DELAY_US);

	int err = pmw3610_write_reg(dev, reg, val);

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
	return err;
}

static int pmw3610_burst_read(const struct device *dev, uint8_t reg,
			      uint8_t *buf, uint8_t len)
{
	const struct pmw3610_config *cfg = dev->config;
	uint8_t addr = reg & 0x7F;

	const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 },   /* skip address byte */
		{ .buf = buf, .len = len },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

/* ── CPI control ───────────────────────────────────────────────────── */

static int pmw3610_set_cpi(const struct device *dev, uint16_t cpi)
{
	struct pmw3610_data *data = dev->data;

	if (cpi < PMW3610_MIN_CPI || cpi > PMW3610_MAX_CPI) {
		LOG_ERR("CPI %u out of range", cpi);
		return -EINVAL;
	}

	uint8_t value = cpi / 200;

	uint8_t addr[] = { PMW3610_REG_SPI_PAGE0, PMW3610_REG_RES_STEP, PMW3610_REG_SPI_PAGE0 };
	uint8_t vals[] = { 0xFF, value, 0x00 };

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_busy_wait(T_CLOCK_ON_DELAY_US);

	int err = 0;
	for (size_t i = 0; i < ARRAY_SIZE(addr) && !err; i++) {
		err = pmw3610_write_reg(dev, addr[i], vals[i]);
	}

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

	if (!err) {
		data->cpi = cpi;
		LOG_INF("CPI set to %u (reg=0x%02x)", cpi, value);
	}

	return err;
}

/* ── Downshift / sample time configuration ─────────────────────────── */

static int pmw3610_set_downshift_time(const struct device *dev,
				      uint8_t reg_addr, uint32_t time_ms)
{
	uint32_t mintime;

	switch (reg_addr) {
	case PMW3610_REG_RUN_DOWNSHIFT:
		mintime = 32;
		break;
	case PMW3610_REG_REST1_DOWNSHIFT:
		mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
		break;
	case PMW3610_REG_REST2_DOWNSHIFT:
		mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
		break;
	default:
		return -ENOTSUP;
	}

	uint8_t value = CLAMP(time_ms / mintime, 1, 255);
	return pmw3610_write(dev, reg_addr, value);
}

static int pmw3610_set_sample_time(const struct device *dev,
				   uint8_t reg_addr, uint32_t ms)
{
	uint8_t value = CLAMP(ms / 10, 1, 255);
	return pmw3610_write(dev, reg_addr, value);
}

/* ── Force-awake control ───────────────────────────────────────────── */

static int pmw3610_set_performance(const struct device *dev, bool force_awake)
{
	uint8_t value;
	int err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
	if (err) return err;

	uint8_t perf = value & 0x0F;
	if (force_awake) {
		perf |= 0xF0;
	}

	if (perf != value) {
		err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
	}
	return err;
}

/* ── Interrupt control ─────────────────────────────────────────────── */

static int pmw3610_set_interrupt(const struct device *dev, bool en)
{
	const struct pmw3610_config *cfg = dev->config;
	return gpio_pin_interrupt_configure_dt(&cfg->motion_gpio,
					       en ? GPIO_INT_LEVEL_ACTIVE
						  : GPIO_INT_DISABLE);
}

/* ── Async init steps ──────────────────────────────────────────────── */

static int pmw3610_async_init_power_up(const struct device *dev)
{
	return pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET,
				 PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev)
{
	return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev)
{
	uint8_t value;
	int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
	if (err) return err;

	if ((value & 0x0F) != 0x0F) {
		LOG_ERR("Self-test failed (0x%x)", value);
		return -EINVAL;
	}

	uint8_t pid;
	err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &pid);
	if (err) return err;

	if (pid != PMW3610_PRODUCT_ID) {
		LOG_ERR("Wrong product ID: 0x%02x (expected 0x%02x)", pid, PMW3610_PRODUCT_ID);
		return -EIO;
	}

	LOG_INF("PMW3610 detected (ID 0x%02x)", pid);
	return 0;
}

static int pmw3610_async_init_configure(const struct device *dev)
{
	const struct pmw3610_config *cfg = dev->config;
	int err = 0;

	/* Clear motion registers (datasheet requirement) */
	for (uint8_t reg = 0x02; reg <= 0x05 && !err; reg++) {
		uint8_t dummy;
		err = pmw3610_read_reg(dev, reg, &dummy);
	}

	if (!err) {
		err = pmw3610_write(dev, PMW3610_REG_SMART_MODE,
				    cfg->smart_mode ? PMW3610_SMART_MODE_ENABLE
						    : PMW3610_SMART_MODE_DISABLE);
	}
	if (!err) err = pmw3610_set_performance(dev, true);
	if (!err) err = pmw3610_set_cpi(dev, cfg->cpi);

	if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
						    CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
	if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
						    CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
	if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
						    CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);

	if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
						 CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
	if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
						 CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
	if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
						 CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);

	return err;
}

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
	[ASYNC_INIT_STEP_POWER_UP]  = pmw3610_async_init_power_up,
	[ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
	[ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
	[ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

static void pmw3610_async_init(struct k_work *work)
{
	struct k_work_delayable *dwork = (struct k_work_delayable *)work;
	struct pmw3610_data *data = CONTAINER_OF(dwork, struct pmw3610_data, init_work);
	const struct device *dev = data->dev;

	LOG_INF("PMW3610 async init step %d", data->async_init_step);

	data->err = async_init_fn[data->async_init_step](dev);
	if (data->err) {
		LOG_ERR("Init failed at step %d", data->async_init_step);
		return;
	}

	data->async_init_step++;

	if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
		data->ready = true;
		LOG_INF("PMW3610 ready (CPI=%u)", data->cpi);
		pmw3610_set_interrupt(dev, true);
	} else {
		k_work_schedule(&data->init_work,
				K_MSEC(async_init_delay[data->async_init_step]));
	}
}

/* ── Motion data processing & reporting ────────────────────────────── */

static int pmw3610_report_data(const struct device *dev)
{
	struct pmw3610_data *data = dev->data;
	uint8_t buf[PMW3610_BURST_SIZE];

	if (unlikely(!data->ready)) return -EBUSY;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	static int64_t last_rpt_time = 0;
	int64_t now = k_uptime_get();
#endif

	int err = pmw3610_burst_read(dev, PMW3610_REG_MOTION_BURST,
				     buf, sizeof(buf));
	if (err) return err;

	if (!(buf[0] & 0x80)) {
		return 0;
	}

	int16_t x = sign_extend(((buf[PMW3610_XY_H_POS] & 0xF0) << 4) |
				 buf[PMW3610_X_L_POS], 11);
	int16_t y = sign_extend(((buf[PMW3610_XY_H_POS] & 0x0F) << 8) |
				 buf[PMW3610_Y_L_POS], 11);


	data->dx_acc += x;
	data->dy_acc += y;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	if (now - last_rpt_time < CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
		return 0;
	}
#endif

	int16_t rx = (int16_t)CLAMP(data->dx_acc, INT16_MIN, INT16_MAX);
	int16_t ry = (int16_t)CLAMP(data->dy_acc, INT16_MIN, INT16_MAX);

	data->dx_acc = 0;
	data->dy_acc = 0;
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	last_rpt_time = now;
#endif

	if (rx != 0 || ry != 0) {
		if (rx != 0) {
			input_report_rel(dev, INPUT_REL_X, rx, ry == 0, K_NO_WAIT);
		}
		if (ry != 0) {
			input_report_rel(dev, INPUT_REL_Y, ry, true, K_NO_WAIT);
		}
	}

	return 0;
}

/* ── Motion interrupt handler ──────────────────────────────────────── */

static void pmw3610_gpio_callback(const struct device *gpiob,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct pmw3610_data *data = CONTAINER_OF(cb, struct pmw3610_data, motion_cb);
	const struct device *dev = data->dev;

	pmw3610_set_interrupt(dev, false);
	k_work_submit_to_queue(&pmw3610_work_q, &data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work)
{
	struct pmw3610_data *data = CONTAINER_OF(work, struct pmw3610_data, trigger_work);
	const struct device *dev = data->dev;

	pmw3610_report_data(dev);
	pmw3610_set_interrupt(dev, true);
}

/* ── PM callbacks ──────────────────────────────────────────────────── */

#ifdef CONFIG_PM_DEVICE
static int pmw3610_pm_action(const struct device *dev,
			     enum pm_device_action action)
{
	struct pmw3610_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		data->ready = false;
		pmw3610_set_interrupt(dev, false);
		pmw3610_write_reg(dev, PMW3610_REG_SHUTDOWN, PMW3610_SHUTDOWN_VAL);
		return 0;

	case PM_DEVICE_ACTION_RESUME:
		pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET,
				  PMW3610_POWERUP_CMD_WAKEUP);
		data->async_init_step = 0;
		k_work_schedule(&data->init_work,
				K_MSEC(async_init_delay[0]));
		return 0;

	default:
		return -ENOTSUP;
	}
}
#endif

/* ── Device init ───────────────────────────────────────────────────── */

static int pmw3610_init(const struct device *dev)
{
	const struct pmw3610_config *cfg = dev->config;
	struct pmw3610_data *data = dev->data;
	int ret;

	data->dev = dev;
	data->ready = false;
	data->cpi = cfg->cpi;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->motion_gpio)) {
		LOG_ERR("Motion GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
	if (ret) return ret;

	gpio_init_callback(&data->motion_cb, pmw3610_gpio_callback,
			   BIT(cfg->motion_gpio.pin));
	ret = gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb);
	if (ret) return ret;

	static bool work_q_started;
	if (!work_q_started) {
		k_work_queue_init(&pmw3610_work_q);
		k_work_queue_start(&pmw3610_work_q, pmw3610_work_stack,
				   K_THREAD_STACK_SIZEOF(pmw3610_work_stack),
				   K_PRIO_COOP(4), NULL);
		work_q_started = true;
	}

	k_work_init(&data->trigger_work, pmw3610_work_callback);

	k_work_init_delayable(&data->init_work, pmw3610_async_init);
	k_work_schedule(&data->init_work, K_MSEC(async_init_delay[0]));

	return 0;
}

/* ── DT instance macros ───────────────────────────────────────────── */

#define PMW3610_INST(n)                                                \
	static struct pmw3610_data pmw3610_data_##n;                   \
	static const struct pmw3610_config pmw3610_config_##n = {      \
		.spi         = SPI_DT_SPEC_INST_GET(n,                \
				SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | \
				SPI_MODE_CPOL | SPI_MODE_CPHA |        \
				SPI_TRANSFER_MSB, 0),                  \
		.motion_gpio = GPIO_DT_SPEC_INST_GET(n, motion_gpios), \
		.cpi         = DT_PROP(DT_DRV_INST(n), cpi),          \
		.smart_mode  = DT_PROP(DT_DRV_INST(n), smart_mode),   \
	};                                                             \
	PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);                \
	DEVICE_DT_INST_DEFINE(n,                                       \
			      pmw3610_init,                             \
			      PM_DEVICE_DT_INST_GET(n),                \
			      &pmw3610_data_##n,                       \
			      &pmw3610_config_##n,                     \
			      POST_KERNEL,                             \
			      CONFIG_INPUT_INIT_PRIORITY,              \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_INST)

/* ── Activity state listener (force-awake on active, release on idle) */

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),
static const struct device *pmw3610_devs[] = {
	DT_FOREACH_STATUS_OKAY(pixart_pmw3610_custom, GET_PMW3610_DEV)
};

static int on_activity_state(const zmk_event_t *eh)
{
	struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
	if (!ev) return 0;

	bool enable = (ev->state == ZMK_ACTIVITY_ACTIVE);
	for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
		pmw3610_set_performance(pmw3610_devs[i], enable);
	}
	return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
