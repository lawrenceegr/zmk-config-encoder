/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * ZMK KScan driver that consumes Zephyr's gpio-qdec input events
 * and converts them to key matrix events (press/release pairs).
 *
 * This driver listens to INPUT_REL_* events from gpio-qdec devices
 * and emits kscan callbacks for ZMK's key matrix system.
 */

#define DT_DRV_COMPAT zmk_kscan_gpio_qdec

#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define MAX_ENCODERS 4

/* Encoder-to-axis mapping entry */
struct encoder_axis_map {
    const struct device *device;
    uint16_t axis;
};

struct kscan_gpio_qdec_config {
    uint32_t num_encoders;
    struct encoder_axis_map encoders[MAX_ENCODERS];
};

struct kscan_gpio_qdec_data {
    kscan_callback_t callback;
    const struct device *dev;
    bool enabled;
};

/* Forward declaration for input callback */
static void kscan_gpio_qdec_input_cb(struct input_event *evt, void *user_data);

/*
 * Find encoder index by matching the source device.
 * Returns -1 if not found.
 */
static int find_encoder_by_device(const struct kscan_gpio_qdec_config *config,
                                   const struct device *source_dev)
{
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        if (config->encoders[i].device == source_dev) {
            return (int)i;
        }
    }
    return -1;
}

/*
 * Find encoder index by matching the axis code.
 * Returns -1 if not found.
 */
static int find_encoder_by_axis(const struct kscan_gpio_qdec_config *config,
                                 uint16_t axis)
{
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        if (config->encoders[i].axis == axis) {
            return (int)i;
        }
    }
    return -1;
}

/*
 * Input event callback - receives events from gpio-qdec devices.
 * Converts rotation direction to key matrix events.
 *
 * Matrix layout per encoder:
 *   Row = encoder index
 *   Col 0 = CW rotation (positive value)
 *   Col 1 = CCW rotation (negative value)
 */
static void kscan_gpio_qdec_input_cb(struct input_event *evt, void *user_data)
{
    const struct device *dev = (const struct device *)user_data;
    struct kscan_gpio_qdec_data *data = dev->data;
    const struct kscan_gpio_qdec_config *config = dev->config;

    if (!data->enabled || data->callback == NULL) {
        return;
    }

    /* Only handle relative axis events (from encoders) */
    if (evt->type != INPUT_EV_REL) {
        return;
    }

    /* Find which encoder this event belongs to */
    int encoder_idx = find_encoder_by_device(config, evt->dev);
    if (encoder_idx < 0) {
        /* Try matching by axis code as fallback */
        encoder_idx = find_encoder_by_axis(config, evt->code);
    }

    if (encoder_idx < 0) {
        /* Event not from a configured encoder */
        return;
    }

    /* Determine direction from event value */
    /* Positive value = CW (col 0), Negative value = CCW (col 1) */
    uint32_t row = (uint32_t)encoder_idx;
    uint32_t col = (evt->value > 0) ? 0 : 1;

    LOG_INF("QDEC encoder %d: value=%d, direction=%s, row=%d, col=%d",
            encoder_idx, evt->value, (evt->value > 0) ? "CW" : "CCW", row, col);

    /* Emit key press then release */
    data->callback(dev, row, col, true);
    data->callback(dev, row, col, false);
}

/* Register global input callback - will filter events in the callback */
INPUT_CALLBACK_DEFINE(NULL, kscan_gpio_qdec_input_cb, NULL);

static int kscan_gpio_qdec_configure(const struct device *dev,
                                      kscan_callback_t callback)
{
    struct kscan_gpio_qdec_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    LOG_INF("QDEC kscan callback configured");
    return 0;
}

static int kscan_gpio_qdec_enable(const struct device *dev)
{
    struct kscan_gpio_qdec_data *data = dev->data;
    const struct kscan_gpio_qdec_config *config = dev->config;

    data->enabled = true;

    LOG_INF("QDEC kscan ENABLED with %d encoders", config->num_encoders);
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        LOG_INF("  Encoder %d: device=%s, axis=0x%04x",
                i,
                config->encoders[i].device ? config->encoders[i].device->name : "NULL",
                config->encoders[i].axis);
    }

    return 0;
}

static int kscan_gpio_qdec_disable(const struct device *dev)
{
    struct kscan_gpio_qdec_data *data = dev->data;

    data->enabled = false;
    LOG_INF("QDEC kscan disabled");

    return 0;
}

static int kscan_gpio_qdec_init(const struct device *dev)
{
    struct kscan_gpio_qdec_data *data = dev->data;
    const struct kscan_gpio_qdec_config *config = dev->config;

    data->dev = dev;

    if (config->num_encoders > MAX_ENCODERS) {
        LOG_ERR("Too many encoders: %d (max %d)", config->num_encoders, MAX_ENCODERS);
        return -EINVAL;
    }

    /* Verify all encoder devices are ready */
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        if (config->encoders[i].device == NULL) {
            LOG_WRN("Encoder %d device is NULL, will match by axis only", i);
        } else if (!device_is_ready(config->encoders[i].device)) {
            LOG_ERR("Encoder %d device not ready: %s",
                    i, config->encoders[i].device->name);
            return -ENODEV;
        }
    }

    LOG_INF("Initialized QDEC kscan with %d encoders", config->num_encoders);
    return 0;
}

static const struct kscan_driver_api kscan_gpio_qdec_api = {
    .config = kscan_gpio_qdec_configure,
    .enable_callback = kscan_gpio_qdec_enable,
    .disable_callback = kscan_gpio_qdec_disable,
};

/*
 * Devicetree instantiation macros
 *
 * The driver expects a "devices" property with phandles to gpio-qdec nodes,
 * and an "axes" property with corresponding INPUT_REL_* axis codes.
 */

#define QDEC_DEVICE_PTR(node_id) DEVICE_DT_GET_OR_NULL(node_id)

#define QDEC_ENCODER_ENTRY(idx, inst)                                           \
    {                                                                            \
        .device = QDEC_DEVICE_PTR(DT_INST_PHANDLE_BY_IDX(inst, devices, idx)),  \
        .axis = DT_INST_PROP_BY_IDX(inst, axes, idx),                           \
    },

#define KSCAN_GPIO_QDEC_INIT(inst)                                              \
    static const struct kscan_gpio_qdec_config config_##inst = {                \
        .num_encoders = DT_INST_PROP_LEN(inst, devices),                        \
        .encoders = {                                                            \
            LISTIFY(DT_INST_PROP_LEN(inst, devices), QDEC_ENCODER_ENTRY, (), inst) \
        },                                                                       \
    };                                                                           \
                                                                                 \
    static struct kscan_gpio_qdec_data data_##inst = {                          \
        .enabled = false,                                                        \
    };                                                                           \
                                                                                 \
    /* Must init after gpio-qdec devices (APPLICATION level) */                 \
    DEVICE_DT_INST_DEFINE(inst, kscan_gpio_qdec_init, NULL, &data_##inst,       \
                          &config_##inst, APPLICATION,                           \
                          CONFIG_APPLICATION_INIT_PRIORITY, &kscan_gpio_qdec_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_GPIO_QDEC_INIT)
