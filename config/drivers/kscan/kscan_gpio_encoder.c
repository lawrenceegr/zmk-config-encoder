/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_gpio_encoder

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define ENCODER_STATE_MASK 0x03
#define MAX_ENCODERS 4

struct encoder_state {
    uint8_t last_state;
    uint8_t position;
};

struct kscan_gpio_encoder_config {
    uint32_t num_encoders;
    const struct gpio_dt_spec *a_gpios;
    const struct gpio_dt_spec *b_gpios;
    uint32_t debounce_period_ms;
    uint32_t poll_period_ms;
};

struct kscan_gpio_encoder_data {
    kscan_callback_t callback;
    struct k_work_delayable work;
    const struct device *dev;
    struct encoder_state encoders[MAX_ENCODERS];
    uint32_t debounce_counters[MAX_ENCODERS];
    bool enabled;
};

// Read AB state - matches ZMK EC11 driver format
static uint8_t read_encoder_pins(const struct device *dev, uint32_t encoder_idx) {
    const struct kscan_gpio_encoder_config *config = dev->config;

    // Read pins: A in bit 1, B in bit 0 (same as official EC11 driver)
    return (gpio_pin_get_dt(&config->a_gpios[encoder_idx]) << 1) |
           gpio_pin_get_dt(&config->b_gpios[encoder_idx]);
}

// Decode quadrature state - returns -1 (CW), 0 (no change), or 1 (CCW)
static int8_t decode_quadrature(uint8_t current_state, uint8_t prev_state) {
    // Combine current and previous state (4-bit pattern)
    // This matches the official ZMK EC11 driver logic
    uint8_t combined = current_state | (prev_state << 2);

    switch (combined) {
        // Clockwise rotation
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
            return -1;

        // Counter-clockwise rotation
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            return 1;

        // No change or invalid transition
        default:
            return 0;
    }
}

static void kscan_gpio_encoder_poll(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_gpio_encoder_data *data =
        CONTAINER_OF(dwork, struct kscan_gpio_encoder_data, work);
    const struct device *dev = data->dev;
    const struct kscan_gpio_encoder_config *config = dev->config;
    static uint32_t poll_count = 0;

    if (!data->enabled) {
        return;
    }

    poll_count++;

    // Debug: Log polling activity every 1000 polls
    if (poll_count % 1000 == 0) {
        LOG_INF("Encoder poll running (count: %d)", poll_count);
    }

    for (uint32_t i = 0; i < config->num_encoders; i++) {
        uint8_t current_state = read_encoder_pins(dev, i);
        uint8_t last_state = data->encoders[i].last_state;

        // Debug: Log state changes (even if no rotation detected)
        if (current_state != last_state) {
            LOG_INF("Encoder %d state change: 0x%02x -> 0x%02x",
                    i, last_state, current_state);
        }

        // Decode quadrature signal using official EC11 algorithm
        int8_t delta = decode_quadrature(current_state, last_state);

        if (delta != 0) {
            // Accumulate rotation
            data->encoders[i].position += delta;

            // Emit key event on each detent
            // Row = encoder index
            // Col = 0 for CW (delta < 0), 1 for CCW (delta > 0)
            uint32_t row = i;
            uint32_t col = (delta > 0) ? 1 : 0;

            LOG_INF("ENCODER %d ROTATED: delta=%d, direction=%s, row=%d, col=%d",
                    i, delta, delta > 0 ? "CCW" : "CW", row, col);

            // Emit key press/release events
            if (data->callback != NULL) {
                data->callback(dev, row, col, true);
                // Release immediately - the kscan layer handles timing
                data->callback(dev, row, col, false);
            } else {
                LOG_ERR("Encoder callback is NULL!");
            }
        }

        // Always update state
        data->encoders[i].last_state = current_state;
    }

    // Schedule next poll
    k_work_schedule(&data->work, K_MSEC(config->poll_period_ms));
}

static int kscan_gpio_encoder_configure(const struct device *dev,
                                        kscan_callback_t callback) {
    struct kscan_gpio_encoder_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    LOG_INF("Encoder kscan callback configured");
    return 0;
}

static int kscan_gpio_encoder_enable(const struct device *dev) {
    struct kscan_gpio_encoder_data *data = dev->data;
    const struct kscan_gpio_encoder_config *config = dev->config;

    data->enabled = true;

    // Initialize encoder states
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        uint8_t initial_state = read_encoder_pins(dev, i);
        data->encoders[i].last_state = initial_state;
        data->encoders[i].position = 0;
        data->debounce_counters[i] = 0;
        LOG_INF("Encoder %d initial state: 0x%02x", i, initial_state);
    }

    // Start polling
    k_work_schedule(&data->work, K_MSEC(config->poll_period_ms));

    LOG_INF("Encoder kscan ENABLED with %d encoders, poll period: %dms",
            config->num_encoders, config->poll_period_ms);
    return 0;
}

static int kscan_gpio_encoder_disable(const struct device *dev) {
    struct kscan_gpio_encoder_data *data = dev->data;

    data->enabled = false;
    k_work_cancel_delayable(&data->work);

    return 0;
}

static int kscan_gpio_encoder_init(const struct device *dev) {
    struct kscan_gpio_encoder_data *data = dev->data;
    const struct kscan_gpio_encoder_config *config = dev->config;

    data->dev = dev;

    if (config->num_encoders > MAX_ENCODERS) {
        LOG_ERR("Too many encoders: %d (max %d)", config->num_encoders, MAX_ENCODERS);
        return -EINVAL;
    }

    // Configure GPIO pins
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        if (!device_is_ready(config->a_gpios[i].port)) {
            LOG_ERR("GPIO port for encoder %d A pin not ready", i);
            return -ENODEV;
        }

        if (!device_is_ready(config->b_gpios[i].port)) {
            LOG_ERR("GPIO port for encoder %d B pin not ready", i);
            return -ENODEV;
        }

        int ret = gpio_pin_configure_dt(&config->a_gpios[i], GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure encoder %d A pin", i);
            return ret;
        }

        ret = gpio_pin_configure_dt(&config->b_gpios[i], GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure encoder %d B pin", i);
            return ret;
        }

        data->encoders[i].last_state = 0;
        data->encoders[i].position = 0;
        data->debounce_counters[i] = 0;
    }

    k_work_init_delayable(&data->work, kscan_gpio_encoder_poll);

    LOG_INF("Initialized GPIO encoder kscan with %d encoders", config->num_encoders);
    return 0;
}

static const struct kscan_driver_api kscan_gpio_encoder_api = {
    .config = kscan_gpio_encoder_configure,
    .enable_callback = kscan_gpio_encoder_enable,
    .disable_callback = kscan_gpio_encoder_disable,
};

#define GPIO_DT_SPEC_GET_BY_IDX_OR(node_id, prop, idx, default_value)                   \
    COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop),                                        \
                (GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)), (default_value))

#define ENCODER_GPIO_SPEC(idx, inst, prop)                                              \
    GPIO_DT_SPEC_GET_BY_IDX_OR(DT_DRV_INST(inst), prop, idx,                           \
                               (struct gpio_dt_spec){0})

#define KSCAN_GPIO_ENCODER_INIT(inst)                                                   \
    static const struct gpio_dt_spec encoder_a_gpios_##inst[] = {                       \
        LISTIFY(DT_INST_PROP_LEN(inst, a_gpios), ENCODER_GPIO_SPEC, (,), inst, a_gpios) \
    };                                                                                   \
                                                                                         \
    static const struct gpio_dt_spec encoder_b_gpios_##inst[] = {                       \
        LISTIFY(DT_INST_PROP_LEN(inst, b_gpios), ENCODER_GPIO_SPEC, (,), inst, b_gpios) \
    };                                                                                   \
                                                                                         \
    static const struct kscan_gpio_encoder_config config_##inst = {                     \
        .num_encoders = DT_INST_PROP_LEN(inst, a_gpios),                                \
        .a_gpios = encoder_a_gpios_##inst,                                              \
        .b_gpios = encoder_b_gpios_##inst,                                              \
        .debounce_period_ms = DT_INST_PROP_OR(inst, debounce_period_ms, 2),            \
        .poll_period_ms = DT_INST_PROP_OR(inst, poll_period_ms, 5),                    \
    };                                                                                   \
                                                                                         \
    static struct kscan_gpio_encoder_data data_##inst = {                               \
        .enabled = false,                                                                \
    };                                                                                   \
                                                                                         \
    DEVICE_DT_INST_DEFINE(inst, kscan_gpio_encoder_init, NULL, &data_##inst,           \
                          &config_##inst, POST_KERNEL,                                   \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &kscan_gpio_encoder_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_GPIO_ENCODER_INIT)
