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
- Optional packed-XY emit mode that halves split-BLE notify rate for
  diagonal motion, with a matching unpack input processor

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
CONFIG_PMW3610_REPORT_INTERVAL_MIN=12
```

## Packed-XY transport (optional)

By default the driver emits motion as two separate Zephyr input events per
report (`INPUT_REL_X` then `INPUT_REL_Y`). On a split BLE keyboard this
becomes two GATT notifies per diagonal-motion report, which dominates the
peripheral->central traffic at high CPI where almost every report is
diagonal.

Enable `CONFIG_PMW3610_PACKED_REPORTS` on the side that hosts the sensor
(typically the peripheral) and the driver instead emits one event per
report with code `PMW3610_REL_PACKED_XY` (0x40, outside Linux's standard
REL range so it can't collide). The 32-bit value carries signed X in the
high 16 bits and signed Y in the low 16 bits.

```
# in the trackball-side .conf
CONFIG_PMW3610=y
CONFIG_PMW3610_REPORT_INTERVAL_MIN=12
CONFIG_PMW3610_PACKED_REPORTS=y
```

A matching input processor (`zip_pmw3610_unpack_xy`, ships with this
module) reverses the packing on the consumer side. It rewrites the
current event in place to the X half and re-emits the Y half via
`input_report_rel` on the same source device. Wire it as the **first**
stage of your listener pipeline so downstream processors and the input
listener see normal `REL_X` / `REL_Y` events:

```dts
trackball_listener {
    compatible = "zmk,input-listener";
    device = <&split_input>;
    input-processors =
        <&zip_pmw3610_unpack_xy>,
        <&zip_spike_filter 5>,
        /* ... rest of the pipeline ... */
        ;
};
```

The unpack processor passes through any non-packed event unchanged, so
it's safe to leave it wired in regardless of whether the producing side
has packing turned on. `CONFIG_ZMK_INPUT_PROCESSOR_PMW3610_UNPACK_XY`
auto-enables when the device-tree node is referenced.

In a typical left-central / right-peripheral split with the trackball on
the right: turn `CONFIG_PMW3610_PACKED_REPORTS=y` on in the right shield
config, and put `&zip_pmw3610_unpack_xy` at the head of the central's
trackball listener pipeline.

## Copyright & License

Everything is released under the terms of the MIT license

© 2026 Kai Evans
