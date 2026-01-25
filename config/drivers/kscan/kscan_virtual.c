/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Virtual KScan driver - provides software-defined key positions
 * that can be triggered by other subsystems (e.g., input events).
 *
 * This driver creates virtual switches with no physical GPIO backing.
 * External triggers (like gpio-qdec input events) can activate these
 * virtual keys, making them appear in ZMK Studio as editable positions.
 */

#define DT_DRV_COMPAT zmk_kscan_virtual

#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct kscan_virtual_config {
    uint32_t num_keys;
    /* Input event triggers - each key can be triggered by an axis+direction */
    const uint16_t *trigger_axes;      /* INPUT_REL_* axis codes */
    const int8_t *trigger_directions;  /* 1 for positive, -1 for negative */
};

struct kscan_virtual_data {
    kscan_callback_t callback;
    const struct device *dev;
    bool enabled;
};

/* Static reference to the kscan device instance (supports single instance) */
static const struct device *kscan_virtual_dev;

/* Forward declaration */
static void kscan_virtual_input_cb(struct input_event *evt);

/*
 * Find virtual key index by matching axis and direction.
 * Returns -1 if not found.
 */
static int find_key_by_trigger(const struct kscan_virtual_config *config,
                               uint16_t axis, int8_t direction)
{
    for (uint32_t i = 0; i < config->num_keys; i++) {
        if (config->trigger_axes[i] == axis &&
            config->trigger_directions[i] == direction) {
            return (int)i;
        }
    }
    return -1;
}

/*
 * Input event callback - receives events from gpio-qdec devices.
 * Triggers virtual key press/release based on configured mappings.
 */
static void kscan_virtual_input_cb(struct input_event *evt)
{
    const struct device *dev = kscan_virtual_dev;
    if (dev == NULL) {
        return;
    }

    struct kscan_virtual_data *data = dev->data;
    const struct kscan_virtual_config *config = dev->config;

    if (!data->enabled || data->callback == NULL) {
        return;
    }

    /* Only handle relative axis events (from encoders) */
    if (evt->type != INPUT_EV_REL) {
        return;
    }

    /* Determine direction from event value */
    int8_t direction = (evt->value > 0) ? 1 : -1;

    /* Find which virtual key this event triggers */
    int key_idx = find_key_by_trigger(config, evt->code, direction);

    if (key_idx < 0) {
        /* No virtual key configured for this axis+direction */
        return;
    }

    /* Virtual keys are laid out as single row: row=0, col=key_idx */
    uint32_t row = 0;
    uint32_t col = (uint32_t)key_idx;

    LOG_INF("Virtual key %d triggered: axis=0x%04x, direction=%s",
            key_idx, evt->code, (direction > 0) ? "+" : "-");

    /* Emit key press then release */
    data->callback(dev, row, col, true);
    data->callback(dev, row, col, false);
}

/* Register global input callback */
INPUT_CALLBACK_DEFINE(NULL, kscan_virtual_input_cb);

static int kscan_virtual_configure(const struct device *dev,
                                   kscan_callback_t callback)
{
    struct kscan_virtual_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    LOG_INF("Virtual kscan callback configured");
    return 0;
}

static int kscan_virtual_enable(const struct device *dev)
{
    struct kscan_virtual_data *data = dev->data;
    const struct kscan_virtual_config *config = dev->config;

    data->enabled = true;

    LOG_INF("Virtual kscan ENABLED with %d keys", config->num_keys);
    for (uint32_t i = 0; i < config->num_keys; i++) {
        LOG_INF("  Key %d: axis=0x%04x, direction=%s",
                i, config->trigger_axes[i],
                (config->trigger_directions[i] > 0) ? "CW/+" : "CCW/-");
    }

    return 0;
}

static int kscan_virtual_disable(const struct device *dev)
{
    struct kscan_virtual_data *data = dev->data;

    data->enabled = false;
    LOG_INF("Virtual kscan disabled");

    return 0;
}

static int kscan_virtual_init(const struct device *dev)
{
    struct kscan_virtual_data *data = dev->data;
    const struct kscan_virtual_config *config = dev->config;

    data->dev = dev;
    kscan_virtual_dev = dev;

    LOG_INF("Initialized virtual kscan with %d keys", config->num_keys);
    return 0;
}

static const struct kscan_driver_api kscan_virtual_api = {
    .config = kscan_virtual_configure,
    .enable_callback = kscan_virtual_enable,
    .disable_callback = kscan_virtual_disable,
};

/*
 * Devicetree instantiation macros
 *
 * The driver expects:
 *   - input-codes: array of INPUT_REL_* axis codes
 *   - input-directions: array of directions (1 = positive/CW, -1 = negative/CCW)
 */

#define KSCAN_VIRTUAL_INIT(inst)                                                \
    static const uint16_t trigger_axes_##inst[] =                               \
        DT_INST_PROP(inst, input_codes);                                        \
                                                                                \
    static const int8_t trigger_directions_##inst[] =                           \
        DT_INST_PROP(inst, input_directions);                                   \
                                                                                \
    static const struct kscan_virtual_config config_##inst = {                  \
        .num_keys = DT_INST_PROP_LEN(inst, input_codes),                        \
        .trigger_axes = trigger_axes_##inst,                                    \
        .trigger_directions = trigger_directions_##inst,                        \
    };                                                                          \
                                                                                \
    static struct kscan_virtual_data data_##inst = {                            \
        .enabled = false,                                                       \
    };                                                                          \
                                                                                \
    DEVICE_DT_INST_DEFINE(inst, kscan_virtual_init, NULL, &data_##inst,         \
                          &config_##inst, APPLICATION,                          \
                          CONFIG_APPLICATION_INIT_PRIORITY, &kscan_virtual_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_VIRTUAL_INIT)
