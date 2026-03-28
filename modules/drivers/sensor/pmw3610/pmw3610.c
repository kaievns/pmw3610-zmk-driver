/*
 * PMW3610 ultra-low-power optical motion sensor — Zephyr input driver
 *
 * GPIO bit-bang 3-wire SPI (bidirectional SDIO) for the little-eye breakout.
 * Ported from little-wing/zmk-pmw3610-driver with additions:
 *   - GPIO bit-bang instead of hardware SPIM (3-wire SDIO)
 *   - Runtime-configurable acceleration profiles (CPI + curve)
 *   - Deep-sleep via SHUTDOWN register + NCS wake
 *
 * Init is async (non-blocking) using delayable work items so the
 * display and other SPI peripherals are not blocked during boot.
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/device.h>
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
 * Below threshold: output = d * multiplier  (linear, smooth for fine work)
 * Above threshold: output ramps up quadratically (snappy for big swings)
 *
 * Parameters (set via pmw3610_set_acceleration):
 *   multiplier  x100  base scaling (100 = 1:1 at low speed)
 *   threshold         deltas <= this are purely linear
 *   exponent    x100  acceleration growth (0=none, 20=mild, 50=strong)
 */

#define ACCEL_LUT_SIZE  64
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
	[ASYNC_INIT_STEP_POWER_UP]  = 60,   /* > 10 ms, extra margin for cold boot */
	[ASYNC_INIT_STEP_CLEAR_OB1] = 200,  /* wait for self-test */
	[ASYNC_INIT_STEP_CHECK_OB1] = 50,   /* verify self-test + product ID */
	[ASYNC_INIT_STEP_CONFIGURE] = 0,
};

/* ── Per-instance data ─────────────────────────────────────────────── */

struct pmw3610_config {
	struct gpio_dt_spec cs_gpio;
	struct gpio_dt_spec clk_gpio;
	struct gpio_dt_spec sdio_gpio;
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

	/* acceleration profile (runtime-configurable) */
	uint16_t cpi;
	uint16_t accel_multiplier;   /* x100 */
	uint16_t accel_threshold;
	uint16_t accel_exponent;     /* x100: growth factor for quadratic ramp */

	/* pre-computed acceleration lookup table (8.8 fixed-point) */
	uint16_t accel_lut[ACCEL_LUT_SIZE];
};

/* ── 3-wire SPI bit-bang (mode 3: CPOL=1 CPHA=1) ──────────────────── */

static inline void spi_cs_assert(const struct pmw3610_config *cfg)
{
	gpio_pin_set_dt(&cfg->cs_gpio, 1);
}

static inline void spi_cs_deassert(const struct pmw3610_config *cfg)
{
	gpio_pin_set_dt(&cfg->cs_gpio, 0);
}

static void spi_write_byte(const struct pmw3610_config *cfg, uint8_t val)
{
	gpio_pin_configure_dt(&cfg->sdio_gpio, GPIO_OUTPUT);

	for (int i = 7; i >= 0; i--) {
		gpio_pin_set_dt(&cfg->clk_gpio, 0);
		gpio_pin_set_dt(&cfg->sdio_gpio, (val >> i) & 1);
		k_busy_wait(1);
		gpio_pin_set_dt(&cfg->clk_gpio, 1);
		k_busy_wait(1);
	}
}

static uint8_t spi_read_byte(const struct pmw3610_config *cfg)
{
	uint8_t val = 0;
	gpio_pin_configure_dt(&cfg->sdio_gpio, GPIO_INPUT);

	for (int i = 7; i >= 0; i--) {
		gpio_pin_set_dt(&cfg->clk_gpio, 0);
		k_busy_wait(1);
		gpio_pin_set_dt(&cfg->clk_gpio, 1);
		val |= (gpio_pin_get_dt(&cfg->sdio_gpio) & 1) << i;
		k_busy_wait(1);
	}

	return val;
}

/* ── Register access with SPI clock enable/disable ─────────────────── */

static int pmw3610_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct pmw3610_config *cfg = dev->config;

	spi_cs_assert(cfg);
	spi_write_byte(cfg, reg & 0x7F);
	k_busy_wait(T_SRAD_US);
	*val = spi_read_byte(cfg);
	spi_cs_deassert(cfg);
	k_busy_wait(T_BEXIT_US);
	gpio_pin_configure_dt(&cfg->sdio_gpio, GPIO_OUTPUT);

	return 0;
}

static int pmw3610_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct pmw3610_config *cfg = dev->config;

	spi_cs_assert(cfg);
	spi_write_byte(cfg, reg | SPI_WRITE_BIT);
	spi_write_byte(cfg, val);
	spi_cs_deassert(cfg);
	k_busy_wait(T_BEXIT_US);

	return 0;
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

/* Burst read: send address once, read N consecutive bytes */
static int pmw3610_burst_read(const struct device *dev, uint8_t reg,
			      uint8_t *buf, uint8_t len)
{
	const struct pmw3610_config *cfg = dev->config;

	spi_cs_assert(cfg);
	spi_write_byte(cfg, reg & 0x7F);
	k_busy_wait(T_SRAD_US);

	for (uint8_t i = 0; i < len; i++) {
		buf[i] = spi_read_byte(cfg);
	}

	spi_cs_deassert(cfg);
	k_busy_wait(T_BEXIT_US);
	gpio_pin_configure_dt(&cfg->sdio_gpio, GPIO_OUTPUT);

	return 0;
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

	/* CPI register requires page switch + SPI clock */
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
 * Rebuild the acceleration LUT from current parameters.
 * Called once when parameters change — never on the hot path.
 *
 * Model:
 *   d <= threshold:  out = d * mul            (linear, pixel-accurate)
 *   d >  threshold:  out = d * mul * (1 + growth * excess^2)
 *
 * where excess = (d - threshold), growth = exponent / 10000.0
 * The quadratic ramp feels natural: gentle onset, strong at big flicks.
 *
 * Output stored as 8.8 fixed-point so small multipliers don't lose precision.
 */
static void rebuild_accel_lut(struct pmw3610_data *data)
{
	float mul = data->accel_multiplier / 100.0f;
	float growth = data->accel_exponent / 10000.0f;
	uint8_t thr = data->accel_threshold;

	for (int d = 0; d < ACCEL_LUT_SIZE; d++) {
		float out;
		if (d <= thr || data->accel_exponent == 0) {
			out = d * mul;
		} else {
			float excess = d - thr;
			out = d * mul * (1.0f + growth * excess * excess);
		}
		/* Clamp to uint16_t range (max representable: 255.99) */
		uint32_t fp = (uint32_t)(out * (1 << ACCEL_FP_SHIFT) + 0.5f);
		data->accel_lut[d] = (uint16_t)MIN(fp, UINT16_MAX);
	}

	LOG_INF("Accel LUT rebuilt: mul=%u/100 thr=%u growth=%u/100 "
		"lut[1]=%u lut[10]=%u lut[30]=%u lut[63]=%u",
		data->accel_multiplier, thr, data->accel_exponent,
		data->accel_lut[1], data->accel_lut[10],
		data->accel_lut[30], data->accel_lut[63]);
}

int pmw3610_set_acceleration(const struct device *dev,
			     uint16_t multiplier,
			     uint16_t threshold,
			     uint16_t exponent)
{
	struct pmw3610_data *data = dev->data;

	data->accel_multiplier = multiplier;
	data->accel_threshold = MIN(threshold, ACCEL_LUT_SIZE - 1);
	data->accel_exponent = exponent;

	rebuild_accel_lut(data);
	return 0;
}

/*
 * Hot-path acceleration: pure integer LUT lookup, no floats.
 * Returns 8.8 fixed-point so the fractional accumulator preserves sub-pixel precision.
 */
static int32_t apply_acceleration_fp(const struct pmw3610_data *data, int16_t dt)
{
	if (dt == 0) return 0;

	int sign = (dt > 0) ? 1 : -1;
	uint16_t d = (uint16_t)abs(dt);
	uint32_t out;

	if (d < ACCEL_LUT_SIZE) {
		out = data->accel_lut[d];
	} else {
		/* Linear extrapolation beyond LUT */
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

/* 12-bit two's complement -> int16_t (from little-wing) */
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

	/* Reject obviously corrupted SPI reads.
	 * At 800 CPI / 8ms, max plausible single-read delta is ~100. */
	if (abs(x) > 200 || abs(y) > 200) {
		return 0;
	}

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

	/* Fractional accumulator for sub-pixel precision (8.8 fixed-point).
	 * Carries remainder across reports so slow movements stay smooth. */
	static int32_t frac_x = 0;
	static int32_t frac_y = 0;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
	if (now - last_smp_time >= CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
		dx_acc = 0;
		dy_acc = 0;
		frac_x = 0;
		frac_y = 0;
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

	/* Apply acceleration via LUT (8.8 fixed-point) */
	int16_t dx_in = (int16_t)CLAMP(dx_acc, INT16_MIN, INT16_MAX);
	int16_t dy_in = (int16_t)CLAMP(dy_acc, INT16_MIN, INT16_MAX);

	frac_x += apply_acceleration_fp(data, dx_in);
	frac_y += apply_acceleration_fp(data, dy_in);

	/* Extract integer part, keep fractional remainder */
	int16_t rx = (int16_t)(frac_x >> ACCEL_FP_SHIFT);
	int16_t ry = (int16_t)(frac_y >> ACCEL_FP_SHIFT);
	frac_x -= (int32_t)rx << ACCEL_FP_SHIFT;
	frac_y -= (int32_t)ry << ACCEL_FP_SHIFT;

	/* Always consume accumulated input and update report time */
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
		/* Full re-init via async sequence */
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

	/* Default acceleration profile */
	data->accel_multiplier = 100; /* 1.0x */
	data->accel_threshold = 5;
	data->accel_exponent = 20;    /* mild quadratic growth */
	rebuild_accel_lut(data);

	/* Configure GPIOs */
	if (!gpio_is_ready_dt(&cfg->cs_gpio) ||
	    !gpio_is_ready_dt(&cfg->clk_gpio) ||
	    !gpio_is_ready_dt(&cfg->sdio_gpio) ||
	    !gpio_is_ready_dt(&cfg->motion_gpio)) {
		LOG_ERR("GPIO device(s) not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) return ret;

	ret = gpio_pin_configure_dt(&cfg->clk_gpio, GPIO_OUTPUT_ACTIVE); /* idle HIGH */
	if (ret) return ret;

	ret = gpio_pin_configure_dt(&cfg->sdio_gpio, GPIO_OUTPUT);
	if (ret) return ret;

	ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
	if (ret) return ret;

	/* Motion interrupt */
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
		.cs_gpio     = GPIO_DT_SPEC_INST_GET(n, cs_gpios),    \
		.clk_gpio    = GPIO_DT_SPEC_INST_GET(n, clk_gpios),   \
		.sdio_gpio   = GPIO_DT_SPEC_INST_GET(n, sdio_gpios),  \
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
