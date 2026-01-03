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
    struct encoder_state *encoders;
    uint32_t *debounce_counters;
    bool enabled;
};

static const int8_t encoder_state_table[16] = {
    0,  // 0000: No change
    -1, // 0001: CCW
    1,  // 0010: CW
    0,  // 0011: Invalid
    1,  // 0100: CW
    0,  // 0101: No change
    0,  // 0110: Invalid
    -1, // 0111: CCW
    -1, // 1000: CCW
    0,  // 1001: Invalid
    0,  // 1010: No change
    1,  // 1011: CW
    0,  // 1100: Invalid
    1,  // 1101: CW
    -1, // 1110: CCW
    0   // 1111: No change
};

static uint8_t read_encoder_pins(const struct device *dev, uint32_t encoder_idx) {
    const struct kscan_gpio_encoder_config *config = dev->config;
    uint8_t state = 0;

    int a_val = gpio_pin_get_dt(&config->a_gpios[encoder_idx]);
    int b_val = gpio_pin_get_dt(&config->b_gpios[encoder_idx]);

    if (a_val > 0) state |= 0x01;
    if (b_val > 0) state |= 0x02;

    return state;
}

static void kscan_gpio_encoder_poll(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_gpio_encoder_data *data =
        CONTAINER_OF(dwork, struct kscan_gpio_encoder_data, work);
    const struct device *dev = data->dev;
    const struct kscan_gpio_encoder_config *config = dev->config;

    if (!data->enabled) {
        return;
    }

    for (uint32_t i = 0; i < config->num_encoders; i++) {
        uint8_t current_state = read_encoder_pins(dev, i);
        uint8_t last_state = data->encoders[i].last_state;

        // Combine last and current state for lookup
        uint8_t combined = ((last_state & ENCODER_STATE_MASK) << 2) |
                          (current_state & ENCODER_STATE_MASK);

        int8_t direction = encoder_state_table[combined];

        if (direction != 0) {
            // Debouncing: require consistent reading
            if (data->debounce_counters[i] < config->debounce_period_ms) {
                data->debounce_counters[i]++;
            } else {
                // Emit key event
                // Row = encoder index
                // Col = 0 for CCW, 1 for CW
                uint32_t row = i;
                uint32_t col = (direction > 0) ? 1 : 0;

                LOG_DBG("Encoder %d rotated %s (row=%d, col=%d)",
                        i, direction > 0 ? "CW" : "CCW", row, col);

                // Press
                if (data->callback != NULL) {
                    data->callback(dev, row, col, true);

                    // Immediate release
                    k_msleep(10);
                    data->callback(dev, row, col, false);
                }

                data->debounce_counters[i] = 0;
            }
        } else {
            // Reset debounce if no movement
            data->debounce_counters[i] = 0;
        }

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
    return 0;
}

static int kscan_gpio_encoder_enable(const struct device *dev) {
    struct kscan_gpio_encoder_data *data = dev->data;
    const struct kscan_gpio_encoder_config *config = dev->config;

    data->enabled = true;

    // Initialize encoder states
    for (uint32_t i = 0; i < config->num_encoders; i++) {
        data->encoders[i].last_state = read_encoder_pins(dev, i);
        data->encoders[i].position = 0;
        data->debounce_counters[i] = 0;
    }

    // Start polling
    k_work_schedule(&data->work, K_MSEC(config->poll_period_ms));

    LOG_DBG("Encoder kscan enabled with %d encoders", config->num_encoders);
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

    // Allocate encoder state tracking
    data->encoders = k_malloc(sizeof(struct encoder_state) * config->num_encoders);
    if (!data->encoders) {
        LOG_ERR("Failed to allocate encoder state memory");
        return -ENOMEM;
    }

    data->debounce_counters = k_malloc(sizeof(uint32_t) * config->num_encoders);
    if (!data->debounce_counters) {
        LOG_ERR("Failed to allocate debounce counter memory");
        k_free(data->encoders);
        return -ENOMEM;
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
                          CONFIG_ZMK_KSCAN_INIT_PRIORITY, &kscan_gpio_encoder_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_GPIO_ENCODER_INIT)
