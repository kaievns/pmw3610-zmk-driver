/*
 * PMW3610 register map and constants
 *
 * Ported from little-wing/zmk-pmw3610-driver with additions for
 * GPIO bit-bang SPI and configurable acceleration profiles.
 */

#ifndef PMW3610_H_
#define PMW3610_H_

/* ── Register addresses ────────────────────────────────────────────── */
#define PMW3610_REG_PRODUCT_ID       0x00
#define PMW3610_REG_REVISION_ID      0x01
#define PMW3610_REG_MOTION           0x02
#define PMW3610_REG_DELTA_X_L        0x03
#define PMW3610_REG_DELTA_Y_L        0x04
#define PMW3610_REG_DELTA_XY_H       0x05
#define PMW3610_REG_SQUAL            0x06
#define PMW3610_REG_SHUTTER_HIGHER   0x07
#define PMW3610_REG_SHUTTER_LOWER    0x08
#define PMW3610_REG_PIX_MAX          0x09
#define PMW3610_REG_PIX_AVG          0x0A
#define PMW3610_REG_PIX_MIN          0x0B

#define PMW3610_REG_CRC0             0x0C
#define PMW3610_REG_CRC1             0x0D
#define PMW3610_REG_CRC2             0x0E
#define PMW3610_REG_CRC3             0x0F
#define PMW3610_REG_SELF_TEST        0x10

#define PMW3610_REG_PERFORMANCE      0x11
#define PMW3610_REG_MOTION_BURST     0x12

#define PMW3610_REG_RUN_DOWNSHIFT    0x1B
#define PMW3610_REG_REST1_RATE       0x1C
#define PMW3610_REG_REST1_DOWNSHIFT  0x1D
#define PMW3610_REG_REST2_RATE       0x1E
#define PMW3610_REG_REST2_DOWNSHIFT  0x1F
#define PMW3610_REG_REST3_RATE       0x20

#define PMW3610_REG_SMART_MODE       0x32

#define PMW3610_REG_OBSERVATION      0x2D
#define PMW3610_REG_PIXEL_GRAB       0x35
#define PMW3610_REG_FRAME_GRAB       0x36

#define PMW3610_REG_POWER_UP_RESET   0x3A
#define PMW3610_REG_SHUTDOWN         0x3B

#define PMW3610_REG_SPI_CLK_ON_REQ   0x41
#define PMW3610_REG_RES_STEP         0x85

/* Page select: write PMW3610_PAGE_{0,1}_VAL to PMW3610_REG_PAGE_SELECT
 * to switch register pages. PAGE0 is the default; PAGE1 holds the
 * RES_STEP register used for CPI and axis inversion. */
#define PMW3610_REG_PAGE_SELECT      0x7F
#define PMW3610_PAGE0_VAL            0x00
#define PMW3610_PAGE1_VAL            0xFF

/* ── Constants ─────────────────────────────────────────────────────── */
#define PMW3610_PRODUCT_ID           0x3E

#define PMW3610_POWERUP_CMD_RESET    0x5A
#define PMW3610_POWERUP_CMD_WAKEUP   0x96

#define PMW3610_SPI_CLOCK_CMD_ENABLE  0xBA
#define PMW3610_SPI_CLOCK_CMD_DISABLE 0xB5

#define PMW3610_SHUTDOWN_VAL         0xB6

#define PMW3610_SMART_MODE_ENABLE    0x00
#define PMW3610_SMART_MODE_DISABLE   0x80

/* Burst read size: motion + dx_l + dy_l + dxy_h + squal + shutter_h + shutter_l */
#define PMW3610_BURST_SIZE           7

/* Burst-read buffer positions */
#define PMW3610_X_L_POS              1
#define PMW3610_Y_L_POS              2
#define PMW3610_XY_H_POS             3
#define PMW3610_SHUTTER_H_POS        5
#define PMW3610_SHUTTER_L_POS        6

/* CPI range */
#define PMW3610_MIN_CPI              200
#define PMW3610_MAX_CPI              3200

/* RES_STEP register bits (page 1) */
#define PMW3610_RES_STEP_INV_X_BIT   6
#define PMW3610_RES_STEP_INV_Y_BIT   5
#define PMW3610_RES_STEP_RES_MASK    0x1F

/* SPI write bit */
#define SPI_WRITE_BIT                BIT(7)

/* SPI timing (µs) */
#define T_CLOCK_ON_DELAY_US          300
#define T_SRAD_US                    4
#define T_BEXIT_US                   1

#endif /* PMW3610_H_ */
