/*
 * PMW3610 public API — callable from external modules (e.g. trackball profiles)
 */

#ifndef PMW3610_API_H_
#define PMW3610_API_H_

#include <zephyr/device.h>
#include <stdint.h>

/**
 * Set sensor CPI (counts per inch).
 * @param cpi  Value in range 200-3200, rounded to nearest multiple of 200.
 */
int pmw3610_set_cpi(const struct device *dev, uint16_t cpi);

/**
 * Set acceleration power curve exponent.
 *
 * @param exponent  Power curve exponent, fixed-point x100.
 *                  100 = linear (no acceleration)
 *                  120 = MX Ergo-like (smooth, mild)
 *                  140 = moderate
 *                  200 = aggressive
 */
int pmw3610_set_acceleration(const struct device *dev, uint16_t exponent);

#endif /* PMW3610_API_H_ */
