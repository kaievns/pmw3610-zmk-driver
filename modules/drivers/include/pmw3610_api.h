/*
 * PMW3610 public API — callable from external modules (e.g. trackball profiles)
 */

#ifndef PMW3610_API_H_
#define PMW3610_API_H_

#include <zephyr/device.h>
#include <stdint.h>

/**
 * Set sensor CPI (counts per inch).
 * @param cpi  Value in range 200–3200, rounded to nearest multiple of 200.
 */
int pmw3610_set_cpi(const struct device *dev, uint16_t cpi);

/**
 * Set acceleration curve parameters.
 *
 * @param multiplier  Base speed factor, fixed-point ×100 (100 = 1.0×).
 * @param threshold   Delta magnitude below which no acceleration is applied.
 * @param exponent    Curve power, fixed-point ×100 (0 = linear, 50 = √, 100 = quadratic).
 *                    Uses lookup tables internally; values > 100 are clamped.
 */
int pmw3610_set_acceleration(const struct device *dev,
			     uint16_t multiplier,
			     uint16_t threshold,
			     uint16_t exponent);

#endif /* PMW3610_API_H_ */
