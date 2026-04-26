/*
 * PMW3610 packed-XY unpack input processor.
 *
 * Reverses CONFIG_PMW3610_PACKED_REPORTS on the consumer side: takes a
 * single INPUT_EV_REL event with code PMW3610_REL_PACKED_XY whose 32-bit
 * value carries (X << 16) | (Y & 0xffff), and re-emits it as separate
 * INPUT_REL_X and INPUT_REL_Y events so downstream processors and the
 * input listener see the motion in the standard form.
 *
 * Implementation:
 *   - The current event is rewritten in place to be the X half (sync=0
 *     when Y is also non-zero, sync=1 otherwise). It then continues
 *     through the rest of the processor chain on the same path.
 *   - When Y is non-zero, a separate REL_Y event is enqueued via
 *     input_report_rel() on the source device. CONFIG_INPUT_MODE_THREAD
 *     is enabled here, so this is a queue-and-return — no recursion into
 *     the current handler. The Y event re-enters the listener's full
 *     processor chain, including this unpacker, which passes it through
 *     because the code is no longer PACKED_XY.
 *
 * Non-packed events (code != PMW3610_REL_PACKED_XY) are passed through
 * unchanged, so the unpacker is safe to leave wired into the pipeline
 * regardless of whether the producing side has packing turned on.
 */

#define DT_DRV_COMPAT zmk_input_processor_pmw3610_unpack_xy

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>

#include "pmw3610.h"

static int unpack_handle_event(const struct device *dev,
			       struct input_event *event,
			       uint32_t param1, uint32_t param2,
			       struct zmk_input_processor_state *state)
{
	if (event->type != INPUT_EV_REL ||
	    event->code != PMW3610_REL_PACKED_XY) {
		return 0;
	}

	int16_t x = (int16_t)((uint32_t)event->value >> 16);
	int16_t y = (int16_t)((uint32_t)event->value & 0xffffu);

	if (x != 0 && y != 0) {
		event->code  = INPUT_REL_X;
		event->value = x;
		event->sync  = 0;
		input_report_rel(event->dev, INPUT_REL_Y, y, true, K_NO_WAIT);
	} else if (x != 0) {
		event->code  = INPUT_REL_X;
		event->value = x;
		event->sync  = 1;
	} else if (y != 0) {
		event->code  = INPUT_REL_Y;
		event->value = y;
		event->sync  = 1;
	} else {
		/* Both zero — driver shouldn't emit this, but if it does,
		 * neuter the event so downstream sees a no-op REL_X. */
		event->code  = INPUT_REL_X;
		event->value = 0;
		event->sync  = 1;
	}

	return 0;
}

static struct zmk_input_processor_driver_api unpack_api = {
	.handle_event = unpack_handle_event,
};

#define UNPACK_INST(n)                                                    \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL,                              \
			      NULL, NULL,                                 \
			      POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,        \
			      &unpack_api);

DT_INST_FOREACH_STATUS_OKAY(UNPACK_INST)
