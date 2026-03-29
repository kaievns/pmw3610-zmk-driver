/*
 * PMW3610 ultra-low-power optical motion sensor — Zephyr input driver
 *
 * Hardware SPI with 3-wire SDIO (MOSI and MISO wired to the same pin).
 * Runtime-configurable acceleration profiles (CPI + power curve).
 * Deep-sleep via SHUTDOWN register.
 *
 * Init is async (non-blocking) using delayable work items so other
 * SPI peripherals are not blocked during boot.
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

#include <zmk/events/activity_state_changed.h>

#include "pmw3610.h"

LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

/* ── Acceleration ──────────────────────────────────────────────────── *
 *
 * Pre-computed LUT mapping input delta -> output delta (8.8 fixed-point).
 * Rebuilt whenever acceleration parameters change.
 *
 * Model:  output = d^(exponent/100)
 *
 * Smooth power curve with no threshold discontinuity.
 *   exponent=100  linear (no acceleration)
 *   exponent=120  mild (MX Ergo-like)
 *   exponent=140  moderate
 *   exponent=200  strong
 */

#define ACCEL_LUT_SIZE  128
#define ACCEL_FP_SHIFT  8   /* 8.8 fixed-point */

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
};

struct pmw3610_data {
	const struct device *dev;
	struct gpio_callback motion_cb;
	struct k_work trigger_work;
	struct k_work_delayable init_work;
	int async_init_step;
	bool ready;
	bool sw_smart_flag;
	int err;

	/* acceleration profile */
	uint16_t cpi;
	uint16_t accel_exponent;

	/* pre-computed acceleration lookup table (8.8 fixed-point) */
	uint16_t accel_lut[ACCEL_LUT_SIZE];
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

/* Write with SPI clock enable/disable (required for config registers) */
static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
	k_busy_wait(T_CLOCK_ON_DELAY_US);

	int err = pmw3610_write_reg(dev, reg, val);

	pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
	return err;
}

/* Burst read: single transaction, CS stays asserted throughout */
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

int pmw3610_set_cpi(const struct device *dev, uint16_t cpi)
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

/* ── Acceleration control ──────────────────────────────────────────── */

/*
 * Lightweight base^exp without math.h.
 * Uses log2/exp2 decomposition with polynomial approximation.
 * Accurate to <1% for base in [1, 128], exp in [1.0, 2.0].
 */
static float approx_powf(float base, float exponent)
{
	if (base <= 1.0f) return base;

	/* log2(base): decompose base = 2^n × m, m in [1,2) */
	float m = base;
	int n = 0;
	while (m >= 2.0f) { m *= 0.5f; n++; }

	/* log2(m) for m in [1,2): Remez minimax polynomial */
	float log2_m = -1.7417939f + m * (2.8212026f +
			m * (-1.4699568f + m * 0.44717955f));
	float y = exponent * (n + log2_m);

	/* 2^y: split into integer + fraction */
	int yi = (int)y;
	float yf = y - yi;

	float result = 1.0f;
	for (int i = 0; i < yi; i++) result *= 2.0f;

	/* 2^yf for yf in [0,1): Taylor-derived polynomial */
	result *= 1.0f + yf * (0.6931472f + yf * (0.2402265f + yf * 0.0558011f));

	return result;
}

static void rebuild_accel_lut(struct pmw3610_data *data)
{
	float power = data->accel_exponent / 100.0f;

	if (power < 1.0f) power = 1.0f;

	data->accel_lut[0] = 0;
	for (int d = 1; d < ACCEL_LUT_SIZE; d++) {
		float out = approx_powf((float)d, power);
		uint32_t fp = (uint32_t)(out * (1 << ACCEL_FP_SHIFT) + 0.5f);
		data->accel_lut[d] = (uint16_t)MIN(fp, UINT16_MAX);
	}

	LOG_INF("Accel LUT rebuilt: power=%u/100 "
		"lut[1]=%u lut[5]=%u lut[10]=%u lut[30]=%u",
		data->accel_exponent,
		data->accel_lut[1], data->accel_lut[5],
		data->accel_lut[10], data->accel_lut[30]);
}

int pmw3610_set_acceleration(const struct device *dev, uint16_t exponent)
{
	struct pmw3610_data *data = dev->data;
	data->accel_exponent = exponent;
	rebuild_accel_lut(data);
	return 0;
}

static int32_t apply_acceleration_fp(const struct pmw3610_data *data, int16_t dt)
{
	if (dt == 0) return 0;

	int sign = (dt > 0) ? 1 : -1;
	uint16_t d = (uint16_t)abs(dt);
	uint32_t out;

	if (d < ACCEL_LUT_SIZE) {
		out = data->accel_lut[d];
	} else {
		uint16_t last = data->accel_lut[ACCEL_LUT_SIZE - 1];
		uint16_t prev = data->accel_lut[ACCEL_LUT_SIZE - 2];
		uint16_t slope = last - prev;
		out = last + (uint32_t)slope * (d - ACCEL_LUT_SIZE + 1);
	}

	return sign * (int32_t)out;
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

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

static int pmw3610_report_data(const struct device *dev)
{
	struct pmw3610_data *data = dev->data;
	uint8_t buf[PMW3610_BURST_SIZE];

	if (unlikely(!data->ready)) return -EBUSY;

	static int32_t dx_acc = 0;
	static int32_t dy_acc = 0;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	static int64_t last_smp_time = 0;
	static int64_t last_rpt_time = 0;
	int64_t now = k_uptime_get();
#endif

	int err = pmw3610_burst_read(dev, PMW3610_REG_MOTION_BURST,
				     buf, sizeof(buf));
	if (err) return err;

	int16_t x = TOINT16((buf[PMW3610_X_L_POS] +
			     ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
	int16_t y = TOINT16((buf[PMW3610_Y_L_POS] +
			     ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
	int16_t tmp = x; x = y; y = tmp;
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)
	x = -x;
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
	y = -y;
#endif

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
	int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8)
			+ buf[PMW3610_SHUTTER_L_POS];
	if (data->sw_smart_flag && shutter < 45) {
		pmw3610_write(dev, 0x32, 0x00);
		data->sw_smart_flag = false;
	}
	if (!data->sw_smart_flag && shutter > 45) {
		pmw3610_write(dev, 0x32, 0x80);
		data->sw_smart_flag = true;
	}
#endif

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	if (now - last_smp_time >= CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
		dx_acc = 0;
		dy_acc = 0;
	}
	last_smp_time = now;
#endif

	dx_acc += x;
	dy_acc += y;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	if (now - last_rpt_time < CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
		return 0;
	}
#endif

	/* Apply acceleration and report */
	int16_t rx = (int16_t)(apply_acceleration_fp(data,
			(int16_t)CLAMP(dx_acc, INT16_MIN, INT16_MAX)) >> ACCEL_FP_SHIFT);
	int16_t ry = (int16_t)(apply_acceleration_fp(data,
			(int16_t)CLAMP(dy_acc, INT16_MIN, INT16_MAX)) >> ACCEL_FP_SHIFT);

	dx_acc = 0;
	dy_acc = 0;
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
	k_work_submit(&data->trigger_work);
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
	data->sw_smart_flag = false;
	data->cpi = cfg->cpi;

	/* Default acceleration: d^1.2 power curve */
	data->accel_exponent = 120;
	rebuild_accel_lut(data);

	/* Verify SPI bus is ready */
	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure motion interrupt GPIO */
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

	k_work_init(&data->trigger_work, pmw3610_work_callback);

	/* Kick off async init */
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
				SPI_TRANSFER_MSB, 0),                  \
		.motion_gpio = GPIO_DT_SPEC_INST_GET(n, motion_gpios), \
		.cpi         = DT_PROP(DT_DRV_INST(n), cpi),          \
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
	DT_FOREACH_STATUS_OKAY(pixart_pmw3610, GET_PMW3610_DEV)
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
