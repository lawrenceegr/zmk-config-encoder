# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_KSCAN_GPIO_ENCODER)
  target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/kscan/kscan_gpio_encoder.c)
endif()
