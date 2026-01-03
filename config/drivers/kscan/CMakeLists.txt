# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_KSCAN_GPIO_ENCODER)
  zephyr_library_sources(kscan_gpio_encoder.c)
endif()
