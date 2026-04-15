# PMW3610 ZMK Driver

Zephyr/ZMK driver for the PixArt PMW3610 ultra-low-power optical motion sensor,
designed for trackball applications in wireless split keyboards.

It is based on pmw3610 drivers work by @ufan, @badjeff, and the ZMK's official
driver donated by Google. It was extended and fine tuned for BLE/trackball
applications and then stripped down to essential functionality and meant to be
used with ZMK input processors pipeline.

## Features

- Hardware SPI with 3-wire SDIO (MOSI/MISO on same pin)
- Configurable CPI (200-3200, in steps of 200)
- Hardware axis inversion via `invert-x` / `invert-y`
- Automatic power state management (RUN -> REST1 -> REST2 -> REST3)
- 1-pixel raw deadzone to suppress sensor jitter
- Accumulator staleness guard to prevent stale motion replay
- Rate-limited reporting with delayed flush for BLE stability
- PM device suspend/resume with deferred SPI initialisation
- Smart tracking mode for difficult surfaces

## Usage

### West / Submodule

Add this repo as a git submodule or include it via west, then reference it in
your build:

```
EXTRA_MODULES := /path/to/pmw3610-zmk-driver
```

### Devicetree

```dts
&pinctrl {
    spi1_default: spi1_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK,  1, 10)>,
                    <NRF_PSEL(SPIM_MOSI, 1, 13)>,
                    <NRF_PSEL(SPIM_MISO, 1, 13)>;  /* same pin as MOSI */
        };
    };
};

&spi1 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi1_default>;
    pinctrl-names = "default";
    cs-gpios = <&gpio1 15 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        compatible = "pixart,pmw3610-custom";
        reg = <0>;
        spi-max-frequency = <1200000>;
        motion-gpios = <&gpio1 11 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>; /* IRQ */
        cpi = <1600>;
        /* invert-x; */
        /* invert-y; */
        /* smart-mode; */
    };
};
```

### Kconfig

```
CONFIG_PMW3610=y
CONFIG_PMW3610_REPORT_INTERVAL_MIN=8
```

## Copyright & License

Everything is released under the terms of the MIT license

© 2026 Kai Evans
